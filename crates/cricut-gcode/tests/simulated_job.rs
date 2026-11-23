//! End-to-end jobs against the in-memory simulated machine.

use std::fmt::Write as _;
use std::time::Duration;

use cricut_gcode::job::{self, Clamp, PARK, Speed};
use cricut_gcode::protocol::op;
use cricut_gcode::transport::{FifoModel, MockLink, SimulatedMachine};
use cricut_gcode::{
    Config, Cricut, Error, Event, GcodeOptions, Job, JobStatus, Material, MaterialSelection,
    Origin, Pacing, PressureAdjust,
};

/// 20 cut points -> 2 frames (13 + 7 vectors).
fn job() -> Job {
    let mut src = String::from("G20\nG0 X1 Y1\n");
    for i in 1..20 {
        writeln!(src, "G1 X{} Y1", 1.0 + f64::from(i) * 0.1).unwrap();
    }
    let opts = GcodeOptions {
        origin: Origin::TopLeft,
        ..Default::default()
    };
    Job::from_gcode(&src, &opts).unwrap()
}

async fn connect() -> (SimulatedMachine, Cricut<MockLink>) {
    let sim = SimulatedMachine::new();
    let cricut = Cricut::connect(sim.link(), Config::default())
        .await
        .unwrap();
    (sim, cricut)
}

/// Play the user: load the mat when asked and press Go when it blinks.
fn operator(
    sim: &SimulatedMachine,
    cricut: &Cricut<MockLink>,
) -> tokio::task::JoinHandle<Vec<Event>> {
    let sim = sim.clone();
    let mut events = cricut.subscribe();
    tokio::spawn(async move {
        let mut seen = Vec::new();
        while let Ok(ev) = events.recv().await {
            match &ev {
                Event::JobStatusChanged(JobStatus::WaitingForMat) => sim.load_mat(),
                Event::JobStatusChanged(JobStatus::WaitingForGo) => sim.press_go(),
                _ => {}
            }
            let terminal = matches!(
                ev,
                Event::JobStatusChanged(
                    JobStatus::Done | JobStatus::Cancelled | JobStatus::Failed { .. }
                )
            );
            seen.push(ev);
            if terminal {
                break;
            }
        }
        seen
    })
}

/// A job's speed and clamp choose its setup commands; everything after them is unchanged.
#[tokio::test(start_paused = true)]
async fn a_job_sends_the_setup_of_its_clamp_and_speed() {
    let (sim, cricut) = connect().await;
    let ops = operator(&sim, &cricut);
    let speed = Speed::from_mm_s(76.2, 76.2);

    cricut
        .run_job(
            job()
                .material(MaterialSelection::Pressure(120))
                .clamp(Clamp::A)
                .speed(speed),
        )
        .await
        .unwrap();
    ops.await.unwrap();

    let st = sim.state().clone();
    assert_eq!(st.bad_frames, 0);
    let expected: Vec<(u8, Vec<u8>)> = job::setup_commands_with(Clamp::A, speed, 120)
        .into_iter()
        .map(|c| (c.opcode, c.payload))
        .collect();
    let got: Vec<(u8, Vec<u8>)> = st
        .received
        .iter()
        .take(expected.len())
        .map(|c| (c.opcode, c.payload.clone()))
        .collect();
    assert_eq!(got, expected);
    // A pen is never turned like a blade.
    assert!(st.received.iter().all(|c| c.opcode != op::ORIENT_BLADE));
}

#[tokio::test(start_paused = true)]
async fn full_job_exact_command_stream() {
    let (sim, cricut) = connect().await;
    let ops = operator(&sim, &cricut);

    let report = cricut.run_job(job()).await.unwrap();
    let events = ops.await.unwrap();

    assert_eq!(report.frames, 2);
    assert_eq!(report.vectors, 20);
    assert!(report.parked);
    assert_eq!(report.material.material, Some(Material::Paper)); // simulator dial = 3
    assert_eq!(report.material.pressure, 157);

    // Every encrypted frame decrypted and checksummed correctly, in the exact expected order.
    let st = sim.state().clone();
    assert_eq!(st.bad_frames, 0);
    let mut expected: Vec<(u8, Vec<u8>)> = job::setup_commands(157)
        .into_iter()
        .map(|c| (c.opcode, c.payload))
        .collect();
    for f in job::vector_frames(&job().plan.polylines) {
        expected.push((op::MULTI_DRAW_VECTOR, f));
    }
    expected.push((op::END_DRAW, vec![0, 0, 0]));
    expected.push((op::BEGIN_DRAW, vec![0, 0, 0]));
    expected.push((op::MULTI_DRAW_VECTOR, job::park_frame()));
    expected.push((op::END_DRAW, vec![0, 0, 0]));
    let got: Vec<(u8, Vec<u8>)> = st
        .received
        .iter()
        .map(|c| (c.opcode, c.payload.clone()))
        .collect();
    assert_eq!(got, expected);
    // first frame was the full 13-vector, 128-byte-body frame
    assert_eq!(got[11].1.len(), 13 * 9);
    assert!((f64::from(st.position.0) - PARK.x).abs() < 0.01);

    let statuses: Vec<JobStatus> = events
        .iter()
        .filter_map(|e| match e {
            Event::JobStatusChanged(s) => Some(s.clone()),
            _ => None,
        })
        .collect();
    assert_eq!(
        statuses,
        vec![
            JobStatus::WaitingForMat,
            JobStatus::WaitingForGo,
            JobStatus::Cutting,
            JobStatus::Finishing,
            JobStatus::Done
        ]
    );
    assert!(
        events
            .iter()
            .any(|e| matches!(e, Event::MatLoadedChanged(true)))
    );
    assert!(
        events
            .iter()
            .any(|e| matches!(e, Event::MaterialSelected(_)))
    );
    assert!(
        events
            .iter()
            .any(|e| matches!(e, Event::Progress(p) if p.frames_sent == 2))
    );

    cricut.unload().await.unwrap();
    assert!(!sim.state().mat_loaded);
    assert_eq!(cricut.state().job, JobStatus::Idle);
}

#[tokio::test(start_paused = true)]
async fn dropped_frames_are_resent() {
    let (sim, cricut) = connect().await;
    let _ops = operator(&sim, &cricut);
    // Drop the first setup command, and the first vector frame twice.
    {
        let mut st = sim.state();
        st.drop_next = 1;
        st.drop_vectors = 2;
    }
    let job = job().material(MaterialSelection::Material(
        Material::Vinyl,
        PressureAdjust::More,
    ));
    let report = cricut.run_job(job).await.unwrap();
    assert_eq!(report.retries, 3);
    assert_eq!(report.material.pressure, 188);
    let st = sim.state();
    assert_eq!(st.dropped, 3);
    // still exactly one copy of each accepted command
    assert_eq!(st.received.len(), 11 + 2 + 1 + 3);
    assert_eq!(st.received[1].payload[10..], 188u16.to_be_bytes());
}

#[tokio::test(start_paused = true)]
async fn cancel_while_waiting_for_mat() {
    let (sim, cricut) = connect().await;
    let c = cricut.clone();
    let handle = tokio::spawn(async move { c.run_job(job()).await });
    tokio::time::sleep(Duration::from_secs(2)).await;
    assert_eq!(cricut.state().job, JobStatus::WaitingForMat);
    assert!(matches!(cricut.run_job(job()).await, Err(Error::Busy)));
    cricut.cancel();
    assert!(matches!(handle.await.unwrap(), Err(Error::Cancelled)));
    assert_eq!(cricut.state().job, JobStatus::Cancelled);
    assert_eq!(
        sim.state().received,
        [] as [cricut_gcode::transport::mock::ReceivedCommand; 0]
    );
    cricut.unload().await.unwrap();
}

#[tokio::test(start_paused = true)]
async fn dial_on_custom_requires_explicit_material() {
    let (sim, cricut) = connect().await;
    sim.set_dial(1);
    assert!(matches!(
        cricut.run_job(job()).await,
        Err(Error::DialOnCustom)
    ));
    assert!(matches!(cricut.state().job, JobStatus::Failed { .. }));
}

#[tokio::test(start_paused = true)]
async fn idle_polling_reports_dial_and_mat() {
    let (sim, cricut) = connect().await;
    let mut events = cricut.subscribe();
    sim.set_dial(11);
    sim.load_mat();
    let mut got_dial = false;
    let mut got_mat = false;
    while !(got_dial && got_mat) {
        match tokio::time::timeout(Duration::from_secs(5), events.recv())
            .await
            .unwrap()
            .unwrap()
        {
            Event::DialChanged(d) => got_dial = d.material() == Some(Material::Cardstock),
            Event::MatLoadedChanged(true) => got_mat = true,
            _ => {}
        }
    }
}

#[tokio::test(start_paused = true)]
async fn reply_ending_in_escape_byte_is_not_lost() {
    // The dial reply is `02 38 5E`: its last byte looks like half an escape pair.
    let (sim, cricut) = connect().await;
    sim.set_dial(0x5E);
    let t0 = tokio::time::Instant::now();
    let dial = cricut.read_dial().await.unwrap();
    assert_eq!(dial, cricut_gcode::DialSetting::Unknown(0x5E));
    assert!(t0.elapsed() < Duration::from_millis(500));
}

#[tokio::test(start_paused = true)]
async fn consecutive_jobs_on_one_mat_load_each_set_pressure() {
    let (sim, cricut) = connect().await;
    let _ops = operator(&sim, &cricut);
    let first = job().material(MaterialSelection::Pressure(145));
    cricut.run_job(first).await.unwrap();
    // Mat stays in; the second job starts without a Load or Go prompt.
    assert!(sim.state().mat_loaded);
    let second = job()
        .material(MaterialSelection::Pressure(157))
        .wait_for_go(false);
    let report = tokio::time::timeout(Duration::from_secs(60), cricut.run_job(second))
        .await
        .expect("second job should not wait for the operator")
        .unwrap();
    assert_eq!(report.material.pressure, 157);
    let pressures: Vec<u16> = sim
        .state()
        .received
        .iter()
        .filter(|c| c.opcode == op::SET_MACHINE_STATE)
        .map(|c| u16::from_be_bytes([c.payload[10], c.payload[11]]))
        .collect();
    assert_eq!(pressures, vec![145, 157]);
}

// ---- pacing: the controller offers frames to a FIFO that drains over time ----

/// 300 vectors a tenth of a millimetre apart, zigzagging, which is 23 frames.
fn tiny_vectors_job() -> Job {
    let mut src = String::from("G21\nG0 X10 Y10\n");
    for i in 1..=300 {
        writeln!(
            src,
            "G1 X{} Y{}",
            10 + (i % 2) * 5,
            10.0 + f64::from(i) * 0.1
        )
        .unwrap();
    }
    let opts = GcodeOptions {
        origin: Origin::TopLeft,
        ..Default::default()
    };
    Job::from_gcode(&src, &opts).unwrap()
}

/// Runs the job against the slow FIFO of the fill run (34 slots, 20 ms a vector); the time the
/// machine had nothing to do.
async fn starved(pacing: Pacing) -> Duration {
    let sim = SimulatedMachine::new();
    sim.use_tokio_clock();
    sim.set_fifo(Some(FifoModel {
        capacity: 34,
        entry_time: Duration::from_millis(20),
    }));
    let config = Config {
        pacing,
        ..Config::default()
    };
    let cricut = Cricut::connect(sim.link(), config).await.unwrap();
    let ops = operator(&sim, &cricut);
    cricut.run_job(tiny_vectors_job()).await.unwrap();
    ops.await.unwrap();
    assert_eq!(sim.state().bad_frames, 0);
    sim.state().fifo.as_ref().unwrap().starved()
}

#[tokio::test(start_paused = true)]
async fn the_controllers_legacy_pacing_starves_a_slow_fifo_and_quick_pacing_does_not() {
    let legacy = starved(Pacing::LEGACY).await;
    let quick = starved(Pacing::quick(Duration::from_millis(40))).await;
    assert!(legacy > Duration::from_secs(5), "legacy {legacy:?}");
    assert!(quick < Duration::from_millis(500), "quick {quick:?}");
    assert!(quick < legacy / 10);
}

#[tokio::test(start_paused = true)]
async fn the_controller_sends_frames_of_the_size_the_pacing_asks_for() {
    let sim = SimulatedMachine::new();
    let cricut = Cricut::connect(
        sim.link(),
        Config {
            pacing: Pacing::LEGACY.with_frame_entries(6).unwrap(),
            ..Config::default()
        },
    )
    .await
    .unwrap();
    let ops = operator(&sim, &cricut);
    cricut.run_job(job()).await.unwrap();
    ops.await.unwrap();
    let sizes: Vec<usize> = sim
        .state()
        .received
        .iter()
        .filter(|c| c.opcode == op::MULTI_DRAW_VECTOR)
        .map(|c| c.payload.len() / 9)
        .collect();
    // 20 points: 6, 6, 6, 2, then the park frame of 2.
    assert_eq!(sizes, vec![6, 6, 6, 2, 2]);
}
