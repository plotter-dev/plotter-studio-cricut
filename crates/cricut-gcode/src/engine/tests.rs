//! The engine is checked against the simulated machine, and against the async controller: the same
//! cut must produce the same encrypted command stream whichever one sends it.

use std::fmt::Write as _;
use std::time::Duration;

use super::*;
use crate::job;
use crate::protocol::op;
use crate::transport::{MockLink, SimulatedMachine};
use crate::{Config, Cricut, Event, GcodeOptions, Job, JobStatus, MaterialSelection, Origin};

/// A simulated machine behind the engine's blocking port, with a clock that only moves when the
/// engine waits.
struct SimPort {
    link: MockLink,
    sim: SimulatedMachine,
    clock: Duration,
    seed: u8,
}

impl Port for SimPort {
    fn write(&mut self, data: &[u8]) -> Result<()> {
        // The machine's FIFO drains with the engine's time.
        self.sim.set_time(self.clock);
        self.link.write_now(data)
    }
    fn read(&mut self, max: usize, timeout: Duration) -> Result<Vec<u8>> {
        let bytes = self.link.read_now(max)?;
        if bytes.is_empty() {
            self.clock += timeout;
        }
        Ok(bytes)
    }
    fn sleep(&mut self, duration: Duration) {
        self.clock += duration;
    }
    fn now(&mut self) -> Duration {
        self.clock
    }
    fn random(&mut self) -> u8 {
        self.seed = self.seed.wrapping_mul(37).wrapping_add(11);
        self.seed
    }
}

fn connect(sim: &SimulatedMachine) -> Engine<SimPort> {
    let port = SimPort {
        link: sim.link(),
        sim: sim.clone(),
        clock: Duration::ZERO,
        seed: 1,
    };
    Engine::connect(port, MachineType::ExploreAir2).expect("the simulated machine answers")
}

const INCH: i64 = 25_400;

/// One stroke of `points` points 0.1 in apart, starting 1 in from the mat's corner.
fn stroke(force: u32, points: i64) -> Vec<Op> {
    let mut ops = vec![Op::ToolDown(force), Op::Travel((INCH, INCH))];
    for i in 1..points {
        ops.push(Op::Line((INCH + i * INCH / 10, INCH)));
    }
    ops.push(Op::ToolUp);
    ops
}

/// Plays the operator and runs the engine until the job is over: loads the mat and confirms when
/// asked, presses Go when it blinks. Returns every update.
fn run(engine: &mut Engine<SimPort>, sim: &SimulatedMachine, ops: &[Op]) -> Vec<Update> {
    let mut updates = Vec::new();
    let mut next = 0;
    for _ in 0..200_000 {
        while next < ops.len() && engine.feed(&ops[next]).expect("a valid op") {
            next += 1;
        }
        if engine.activity() == Activity::WaitingForGo {
            sim.press_go();
        }
        if let Step::Update(update) = engine.step().expect("the machine answers") {
            if let Update::Prompt { id, .. } = &update {
                sim.load_mat();
                engine.confirm(*id);
            }
            let over = matches!(update, Update::Finished { .. } | Update::Unloaded);
            updates.push(update);
            if over {
                return updates;
            }
        }
    }
    panic!("the job did not finish; updates so far: {updates:?}");
}

fn encrypted_ops(sim: &SimulatedMachine) -> Vec<u8> {
    sim.state().received.iter().map(|c| c.opcode).collect()
}

fn pressures(sim: &SimulatedMachine) -> Vec<u16> {
    sim.state()
        .received
        .iter()
        .filter(|c| c.opcode == op::SET_MACHINE_STATE)
        .map(|c| u16::from_be_bytes([c.payload[10], c.payload[11]]))
        .collect()
}

/// Every vector the machine received, as `(kind, x, y)` in inches.
fn vectors(sim: &SimulatedMachine) -> Vec<(u8, f64, f64)> {
    sim.state()
        .received
        .iter()
        .filter(|c| c.opcode == op::MULTI_DRAW_VECTOR)
        .flat_map(|c| c.payload.as_chunks::<9>().0.to_vec())
        .map(|e| {
            (
                e[0],
                f64::from(f32::from_le_bytes(e[1..5].try_into().unwrap())),
                f64::from(f32::from_le_bytes(e[5..9].try_into().unwrap())),
            )
        })
        .collect()
}

#[test]
fn a_stroke_is_cut_as_the_async_controller_cuts_it() {
    // The same 20-point stroke through both controllers, on separate simulated machines.
    let sim_blocking = SimulatedMachine::new();
    let mut engine = connect(&sim_blocking);
    engine.begin_job().unwrap();
    let mut ops = stroke(157, 20);
    ops.push(Op::End);
    let updates = run(&mut engine, &sim_blocking, &ops);
    assert!(matches!(
        updates.last(),
        Some(Update::Finished { parked: true })
    ));

    let rt = tokio::runtime::Builder::new_current_thread()
        .enable_time()
        .start_paused(true)
        .build()
        .unwrap();
    let sim_async = SimulatedMachine::new();
    rt.block_on(async {
        let cricut = Cricut::connect(sim_async.link(), Config::default())
            .await
            .unwrap();
        let mut src = String::from("G21\nG0 X25.4 Y25.4\n");
        for i in 1..20 {
            let _ = writeln!(src, "G1 X{:.2} Y25.4", 25.4 + f64::from(i) * 2.54);
        }
        let options = GcodeOptions {
            origin: Origin::TopLeft,
            ..Default::default()
        };
        let job = Job::from_gcode(&src, &options)
            .unwrap()
            .material(MaterialSelection::Pressure(157));
        let mut events = cricut.subscribe();
        let sim = sim_async.clone();
        let operator = tokio::spawn(async move {
            while let Ok(event) = events.recv().await {
                match event {
                    Event::JobStatusChanged(JobStatus::WaitingForMat) => sim.load_mat(),
                    Event::JobStatusChanged(JobStatus::WaitingForGo) => sim.press_go(),
                    Event::JobStatusChanged(JobStatus::Done) => break,
                    _ => {}
                }
            }
        });
        cricut.run_job(job).await.unwrap();
        operator.await.unwrap();
    });

    assert_eq!(
        sim_blocking.state().received,
        sim_async.state().received,
        "the blocking engine must send exactly what the async controller sends"
    );
}

#[test]
fn nothing_is_sent_to_the_machine_until_the_operator_confirms() {
    let sim = SimulatedMachine::new();
    let mut engine = connect(&sim);
    assert!(engine.begin_job().unwrap(), "no mat is in yet");
    for op in stroke(157, 20) {
        assert!(engine.feed(&op).unwrap());
    }
    let mut prompt = None;
    for _ in 0..50 {
        if let Step::Update(Update::Prompt { id, message }) = engine.step().unwrap() {
            prompt = Some((id, message));
            break;
        }
    }
    let (_, message) = prompt.expect("a cutting section asks the operator first");
    assert!(message.contains("Load"), "{message}");
    for _ in 0..50 {
        let _ = engine.step().unwrap();
    }
    assert_eq!(sim.state().received.len(), 0, "no command before Go");
    assert_eq!(engine.activity(), Activity::AwaitingConfirmation);
}

#[test]
fn a_mat_that_is_already_in_is_not_asked_for_again() {
    let sim = SimulatedMachine::new();
    sim.load_mat();
    let mut engine = connect(&sim);
    assert!(!engine.begin_job().unwrap());
    for op in stroke(157, 14) {
        engine.feed(&op).unwrap();
    }
    engine.feed(&Op::End).unwrap();
    let mut message = None;
    for _ in 0..50 {
        if let Step::Update(Update::Prompt { message: m, .. }) = engine.step().unwrap() {
            message = Some(m);
            break;
        }
    }
    let message = message.unwrap();
    assert!(!message.contains("Load"), "{message}");
    assert!(message.contains("Go"), "{message}");
}

#[test]
fn a_change_of_pressure_starts_a_new_section_after_its_own_prompt() {
    let sim = SimulatedMachine::new();
    let mut engine = connect(&sim);
    engine.begin_job().unwrap();
    let mut ops = stroke(120, 14);
    ops.extend(stroke(350, 14));
    ops.push(Op::End);
    let updates = run(&mut engine, &sim, &ops);
    assert_eq!(pressures(&sim), vec![120, 350]);
    let prompts = updates
        .iter()
        .filter(|u| matches!(u, Update::Prompt { .. }))
        .count();
    assert_eq!(prompts, 2, "one prompt per section");
}

#[test]
fn a_pause_in_the_program_is_the_next_sections_prompt() {
    let sim = SimulatedMachine::new();
    let mut engine = connect(&sim);
    engine.begin_job().unwrap();
    let mut ops = vec![
        Op::Pause("Load pen: Fine liner into Clamp A".into()),
        Op::ToolChange,
    ];
    ops.extend(stroke(120, 14));
    ops.push(Op::Pause("Load blade into Clamp B".into()));
    ops.push(Op::ToolChange);
    ops.extend(stroke(350, 14));
    ops.push(Op::End);
    let updates = run(&mut engine, &sim, &ops);
    let messages: Vec<&str> = updates
        .iter()
        .filter_map(|u| match u {
            Update::Prompt { message, .. } => Some(message.as_str()),
            _ => None,
        })
        .collect();
    assert_eq!(messages.len(), 2, "{messages:?}");
    assert!(messages[0].contains("Fine liner"), "{}", messages[0]);
    assert!(messages[1].contains("blade"), "{}", messages[1]);
    assert_eq!(pressures(&sim), vec![120, 350]);
}

#[test]
fn a_program_with_no_cuts_finishes_without_touching_the_blade() {
    let sim = SimulatedMachine::new();
    sim.load_mat();
    let mut engine = connect(&sim);
    engine.begin_job().unwrap();
    let updates = run(
        &mut engine,
        &sim,
        &[Op::Pause("nothing".into()), Op::ToolUp, Op::End],
    );
    assert!(matches!(updates.last(), Some(Update::Finished { .. })));
    assert_eq!(sim.state().received.len(), 0);
}

#[test]
fn travel_moves_never_draw() {
    let sim = SimulatedMachine::new();
    let mut engine = connect(&sim);
    engine.begin_job().unwrap();
    let ops = [
        Op::Travel((INCH, INCH)),
        Op::Line((2 * INCH, INCH)),
        Op::ToolDown(157),
        Op::Travel((3 * INCH, INCH)),
        Op::Line((3 * INCH, 2 * INCH)),
        Op::ToolUp,
        Op::Line((4 * INCH, 2 * INCH)),
        Op::End,
    ];
    run(&mut engine, &sim, &ops);
    let v = vectors(&sim);
    // One MOVE to (3.25, 1.25 in), one LINE to (3.25, 2.25 in), then the park frame's two MOVEs.
    assert_eq!(v.len(), 4, "{v:?}");
    assert_eq!(v[0].0, 2);
    assert!((v[0].1 - 3.25).abs() < 1e-6 && (v[0].2 - 1.25).abs() < 1e-6);
    assert_eq!(v[1].0, 0);
    assert!((v[1].2 - 2.25).abs() < 1e-6);
}

#[test]
fn a_frame_the_machine_drops_is_sent_again() {
    let sim = SimulatedMachine::new();
    sim.state().drop_vectors = 3;
    let mut engine = connect(&sim);
    engine.begin_job().unwrap();
    let mut ops = stroke(157, 30);
    ops.push(Op::End);
    run(&mut engine, &sim, &ops);
    assert_eq!(sim.state().dropped, 3);
    let frames: usize = encrypted_ops(&sim)
        .iter()
        .map(|o| usize::from(*o == op::MULTI_DRAW_VECTOR))
        .sum();
    assert_eq!(frames, 3 + 1, "30 points are 3 frames, then the park frame");
}

#[test]
fn an_arc_is_cut_as_chords_that_end_exactly_on_its_target() {
    let sim = SimulatedMachine::new();
    let mut engine = connect(&sim);
    engine.begin_job().unwrap();
    // A quarter circle of 20 mm radius, from (30, 10) mm to (10, 30) mm about (10, 10) mm.
    let ops = [
        Op::ToolDown(157),
        Op::Travel((30_000, 10_000)),
        Op::Arc {
            to: (10_000, 30_000),
            centre_offset: (-20_000, 0),
            ccw: true,
        },
        Op::End,
    ];
    run(&mut engine, &sim, &ops);
    let points = vectors(&sim);
    let cut = &points[..points.len() - 2];
    assert!(
        cut.len() > 8,
        "a 20 mm arc needs many chords: {}",
        cut.len()
    );
    let centre = 10.0 / 25.4 + 0.25;
    let radius = 20.0 / 25.4;
    for (_, x, y) in cut {
        let r = ((x - centre).powi(2) + (y - centre).powi(2)).sqrt();
        assert!(
            (r - radius).abs() < 0.001,
            "({x}, {y}) is {r} from the centre"
        );
    }
    let (_, x, y) = cut[cut.len() - 1];
    assert!((x - centre).abs() < 1e-5 && (y - (30.0 / 25.4 + 0.25)).abs() < 1e-5);
}

#[test]
fn a_point_off_the_mat_is_refused() {
    let sim = SimulatedMachine::new();
    let mut engine = connect(&sim);
    engine.begin_job().unwrap();
    engine.feed(&Op::ToolDown(157)).unwrap();
    engine.feed(&Op::Travel((INCH, INCH))).unwrap();
    let err = engine.feed(&Op::Line((13 * INCH, INCH))).unwrap_err();
    assert!(matches!(err, Error::OutOfBounds { .. }), "{err}");
}

#[test]
fn input_is_held_back_when_the_buffer_is_full() {
    let sim = SimulatedMachine::new();
    let mut engine = connect(&sim);
    engine.begin_job().unwrap();
    engine.feed(&Op::ToolDown(157)).unwrap();
    engine.feed(&Op::Travel((INCH, INCH))).unwrap();
    let mut accepted = 0;
    while engine
        .feed(&Op::Line((INCH + accepted, INCH + (accepted % 2) * 100)))
        .unwrap()
    {
        accepted += 1;
        assert!(accepted < 10_000, "the buffer never filled");
    }
    assert!(accepted >= 13, "at least one frame is buffered: {accepted}");
}

#[test]
fn cancelling_aborts_the_machine_and_parks_the_head() {
    let sim = SimulatedMachine::new();
    let mut engine = connect(&sim);
    engine.begin_job().unwrap();
    for op in stroke(157, 60) {
        engine.feed(&op).unwrap();
    }
    // Start cutting, then cancel with the stroke half sent.
    for _ in 0..400 {
        sim.load_mat();
        if let Step::Update(Update::Prompt { id, .. }) = engine.step().unwrap() {
            engine.confirm(id);
        }
        if engine.activity() == Activity::WaitingForGo {
            sim.press_go();
        }
        if engine.activity() == Activity::Cutting && sim.state().received.len() > 12 {
            break;
        }
    }
    assert_eq!(engine.activity(), Activity::Cutting);
    let aborts = |sim: &SimulatedMachine| {
        sim.state()
            .plain_received
            .iter()
            .map(|o| usize::from(*o == op::ABORT))
            .sum::<usize>()
    };
    let before = aborts(&sim);
    engine.cancel().unwrap();
    assert!(aborts(&sim) > before, "cancel flushes the machine's queue");
    let mut unloaded = false;
    for _ in 0..2_000 {
        if matches!(engine.step().unwrap(), Step::Update(Update::Unloaded)) {
            unloaded = true;
            break;
        }
    }
    assert!(unloaded, "the head is parked after a cancel");
    assert_eq!(engine.activity(), Activity::Idle);
    assert_eq!(sim.state().lights_cmd, Some((0x46, 0x00)));
    // The last frame is the park frame: two MOVEs to the machine's rest position.
    let last = sim
        .state()
        .received
        .iter()
        .rfind(|c| c.opcode == op::MULTI_DRAW_VECTOR)
        .cloned()
        .unwrap();
    assert_eq!(last.payload, job::park_frame());
}

#[test]
fn a_second_job_can_follow_the_first() {
    let sim = SimulatedMachine::new();
    let mut engine = connect(&sim);
    for _ in 0..2 {
        engine.begin_job().unwrap();
        let mut ops = stroke(157, 14);
        ops.push(Op::End);
        run(&mut engine, &sim, &ops);
    }
    assert_eq!(pressures(&sim), vec![157, 157]);
}

#[test]
fn feeding_with_no_job_is_an_error() {
    let sim = SimulatedMachine::new();
    let mut engine = connect(&sim);
    assert!(matches!(engine.feed(&Op::End), Err(Error::NoJob)));
    engine.begin_job().unwrap();
    assert!(matches!(engine.begin_job(), Err(Error::Busy)));
}

// ---- clamp A (pens) ----
//
// Design Space starts a pen job differently from a blade job (captured 2026-11-19, USB): another
// `SetMachineState`, and no `OrientBlade`. The machine has no clamp-select command, so these start
// bytes are how a job gets clamp A.

/// The `SetMachineState` payloads the machine received, in order.
fn machine_states(sim: &SimulatedMachine) -> Vec<Vec<u8>> {
    sim.state()
        .received
        .iter()
        .filter(|c| c.opcode == op::SET_MACHINE_STATE)
        .map(|c| c.payload.clone())
        .collect()
}

#[test]
fn a_pen_in_clamp_a_starts_like_design_space_starts_a_pen_job() {
    let sim = SimulatedMachine::new();
    let mut engine = connect(&sim);
    engine.begin_job().unwrap();
    let mut ops = vec![
        Op::Pause("Load pen: Fine liner into Clamp A".into()),
        Op::ToolChange,
        Op::Clamp(Clamp::A),
    ];
    ops.extend(stroke(120, 14));
    ops.push(Op::End);
    run(&mut engine, &sim, &ops);

    assert_eq!(
        machine_states(&sim),
        vec![job::machine_state_payload(Clamp::A, 120)]
    );
    let ops = encrypted_ops(&sim);
    assert!(
        !ops.contains(&op::ORIENT_BLADE),
        "a pen is never oriented like a blade: {ops:02x?}"
    );
    // EndDraw, SetMachineState, Geometry, Speed x4, BeginDraw, then the vectors.
    assert_eq!(
        ops[..9],
        [
            0x22,
            0x30,
            0x31,
            0xA1,
            0xA1,
            0xA1,
            0xA1,
            0x21,
            op::MULTI_DRAW_VECTOR
        ]
    );
}

#[test]
fn a_blade_after_a_pen_gets_its_own_start() {
    let sim = SimulatedMachine::new();
    let mut engine = connect(&sim);
    engine.begin_job().unwrap();
    let mut ops = vec![Op::ToolChange, Op::Clamp(Clamp::A)];
    ops.extend(stroke(120, 14));
    ops.extend([
        Op::Pause("Load blade into Clamp B".into()),
        Op::ToolChange,
        Op::Clamp(Clamp::B),
    ]);
    ops.extend(stroke(350, 14));
    ops.push(Op::End);
    run(&mut engine, &sim, &ops);

    assert_eq!(
        machine_states(&sim),
        vec![
            job::machine_state_payload(Clamp::A, 120),
            job::machine_state_payload(Clamp::B, 350),
        ]
    );
    let oriented = sim
        .state()
        .received
        .iter()
        .filter(|c| c.opcode == op::ORIENT_BLADE)
        .count();
    assert_eq!(oriented, 1, "only the blade section orients");
}

#[test]
fn a_program_that_never_names_a_clamp_is_cut_with_the_blade() {
    let sim = SimulatedMachine::new();
    let mut engine = connect(&sim);
    engine.begin_job().unwrap();
    let mut ops = stroke(157, 14);
    ops.push(Op::End);
    run(&mut engine, &sim, &ops);
    assert_eq!(
        machine_states(&sim),
        vec![job::machine_state_payload(Clamp::B, 157)]
    );
    assert!(encrypted_ops(&sim).contains(&op::ORIENT_BLADE));
}

#[test]
fn naming_a_different_clamp_mid_section_starts_a_new_section() {
    let sim = SimulatedMachine::new();
    let mut engine = connect(&sim);
    engine.begin_job().unwrap();
    // No tool change between them: the clamp alone splits the cut.
    let mut ops = vec![Op::Clamp(Clamp::A)];
    ops.extend(stroke(120, 14));
    ops.push(Op::Clamp(Clamp::B));
    ops.extend(stroke(120, 14));
    ops.push(Op::End);
    run(&mut engine, &sim, &ops);
    assert_eq!(
        machine_states(&sim),
        vec![
            job::machine_state_payload(Clamp::A, 120),
            job::machine_state_payload(Clamp::B, 120),
        ]
    );
}
// ---- failures the machine or the link can cause ----

type LinkSwitch = std::rc::Rc<std::cell::Cell<bool>>;

/// A port whose link can be cut and restored from the test.
struct FlakyPort {
    inner: SimPort,
    down: LinkSwitch,
}

impl Port for FlakyPort {
    fn write(&mut self, data: &[u8]) -> Result<()> {
        if self.down.get() {
            return Err(Error::Disconnected);
        }
        self.inner.write(data)
    }
    fn read(&mut self, max: usize, timeout: Duration) -> Result<Vec<u8>> {
        if self.down.get() {
            return Err(Error::Disconnected);
        }
        self.inner.read(max, timeout)
    }
    fn sleep(&mut self, duration: Duration) {
        self.inner.sleep(duration);
    }
    fn now(&mut self) -> Duration {
        self.inner.now()
    }
    fn random(&mut self) -> u8 {
        self.inner.random()
    }
}

fn connect_flaky(sim: &SimulatedMachine) -> (Engine<FlakyPort>, LinkSwitch) {
    let down = LinkSwitch::default();
    let port = FlakyPort {
        inner: SimPort {
            link: sim.link(),
            sim: sim.clone(),
            clock: Duration::ZERO,
            seed: 1,
        },
        down: down.clone(),
    };
    let engine =
        Engine::connect(port, MachineType::ExploreAir2).expect("the simulated machine answers");
    (engine, down)
}

/// Plays the operator until `until` says stop; panics if it never does.
fn drive<P: Port>(
    engine: &mut Engine<P>,
    sim: &SimulatedMachine,
    mut until: impl FnMut(&Engine<P>) -> bool,
) {
    for _ in 0..50_000 {
        sim.load_mat();
        if engine.activity() == Activity::WaitingForGo {
            sim.press_go();
        }
        if let Step::Update(Update::Prompt { id, .. }) = engine.step().expect("the machine answers")
        {
            engine.confirm(id);
        }
        if until(engine) {
            return;
        }
    }
    panic!("never got there; activity is {:?}", engine.activity());
}

#[test]
fn a_command_the_machine_never_answers_is_sent_again() {
    let cut = |lost: u32| {
        let sim = SimulatedMachine::new();
        let mut engine = connect(&sim);
        engine.begin_job().unwrap();
        let mut ops = stroke(157, 14);
        ops.push(Op::End);
        // The first encrypted command of the job goes missing.
        sim.state().silent_next = lost;
        run(&mut engine, &sim, &ops);
        encrypted_ops(&sim)
    };
    assert_eq!(
        cut(1),
        cut(0),
        "the lost setup command was sent again, so the machine saw the whole job"
    );
}

#[test]
fn a_vector_frame_the_machine_never_answers_is_sent_again() {
    let sim = SimulatedMachine::new();
    let mut engine = connect(&sim);
    engine.begin_job().unwrap();
    let mut ops = stroke(157, 14);
    ops.push(Op::End);
    for op in &ops {
        assert!(engine.feed(op).unwrap());
    }
    // Go on until the setup is sent, then lose the first vector frame.
    drive(&mut engine, &sim, |_| {
        sim.state()
            .received
            .iter()
            .filter(|c| c.opcode == op::BEGIN_DRAW)
            .count()
            == 1
            && sim
                .state()
                .received
                .last()
                .is_some_and(|c| c.opcode == op::BEGIN_DRAW)
    });
    sim.state().silent_next = 1;
    run(&mut engine, &sim, &[]);
    // A leading MOVE and 13 lines, then the park frame's two MOVEs; nothing is missing.
    assert_eq!(vectors(&sim).len(), 14 + 2);
}

#[test]
fn a_machine_that_stays_silent_is_an_error_not_a_hang() {
    let sim = SimulatedMachine::new();
    let mut engine = connect(&sim);
    engine.begin_job().unwrap();
    for op in stroke(157, 14) {
        engine.feed(&op).unwrap();
    }
    sim.state().silent_next = 1_000;
    let mut failure = None;
    for _ in 0..2_000 {
        sim.load_mat();
        if engine.activity() == Activity::WaitingForGo {
            sim.press_go();
        }
        match engine.step() {
            Ok(Step::Update(Update::Prompt { id, .. })) => {
                engine.confirm(id);
            }
            Ok(_) => {}
            Err(e) => {
                failure = Some(e);
                break;
            }
        }
    }
    assert!(
        matches!(failure, Some(Error::MachineDropping { .. })),
        "{failure:?}"
    );
}

#[test]
fn a_failed_position_read_while_parking_does_not_end_the_wait() {
    let sim = SimulatedMachine::new();
    let (mut engine, down) = connect_flaky(&sim);
    engine.begin_job().unwrap();
    let mut ops = stroke(157, 14);
    ops.push(Op::End);
    for op in &ops {
        assert!(engine.feed(op).unwrap());
    }
    drive(&mut engine, &sim, |e| e.activity() == Activity::Finishing);
    down.set(true);
    assert!(engine.step().is_err(), "the link is down");
    down.set(false);
    let mut finished = None;
    for _ in 0..100 {
        if let Step::Update(Update::Finished { parked }) = engine.step().unwrap() {
            finished = Some(parked);
            break;
        }
    }
    assert_eq!(
        finished,
        Some(true),
        "the carriage is read again and seen at rest, not written off as unparked"
    );
}

#[test]
fn a_cancel_whose_abort_fails_still_parks_the_head_afterwards() {
    let sim = SimulatedMachine::new();
    let (mut engine, down) = connect_flaky(&sim);
    engine.begin_job().unwrap();
    for op in stroke(157, 60) {
        engine.feed(&op).unwrap();
    }
    drive(&mut engine, &sim, |e| {
        e.activity() == Activity::Cutting && sim.state().received.len() > 12
    });
    down.set(true);
    assert!(engine.cancel().is_err(), "the abort cannot be sent");
    assert_eq!(
        engine.activity(),
        Activity::Unloading,
        "the job was not forgotten: the head is still to be parked"
    );
    down.set(false);
    let mut unloaded = false;
    for _ in 0..2_000 {
        if matches!(engine.step().unwrap(), Step::Update(Update::Unloaded)) {
            unloaded = true;
            break;
        }
    }
    assert!(unloaded);
    let last = sim
        .state()
        .received
        .iter()
        .rfind(|c| c.opcode == op::MULTI_DRAW_VECTOR)
        .cloned()
        .unwrap();
    assert_eq!(last.payload, job::park_frame());
}

#[test]
fn abandoning_drops_the_work_and_leaves_the_engine_usable() {
    let sim = SimulatedMachine::new();
    let mut engine = connect(&sim);
    engine.begin_job().unwrap();
    assert_eq!(engine.activity(), Activity::Preparing);
    engine.abandon();
    assert_eq!(engine.activity(), Activity::Idle);
    assert!(engine.begin_job().is_ok());
}

#[test]
fn no_single_step_waits_long_enough_for_the_host_to_give_up() {
    // The host cuts a call off after two seconds: a step must stay well inside that, even while
    // a mat is loaded, Go pressed and the mat ejected.
    let sim = SimulatedMachine::new();
    let mut engine = connect(&sim);
    engine.begin_job().unwrap();
    let mut ops = stroke(157, 14);
    ops.push(Op::End);
    for op in &ops {
        assert!(engine.feed(op).unwrap());
    }
    let mut worst = Duration::ZERO;
    let mut unloaded = false;
    for _ in 0..50_000 {
        sim.load_mat();
        if engine.activity() == Activity::WaitingForGo {
            sim.press_go();
        }
        let before = engine.link.port.clock;
        let step = engine.step().unwrap();
        worst = worst.max(engine.link.port.clock.saturating_sub(before));
        match step {
            Step::Update(Update::Prompt { id, .. }) => {
                engine.confirm(id);
            }
            Step::Update(Update::Finished { .. }) => engine.unload().unwrap(),
            Step::Update(Update::Unloaded) => {
                unloaded = true;
                break;
            }
            _ => {}
        }
    }
    assert!(unloaded);
    assert!(
        worst <= Duration::from_millis(1300),
        "the longest step took {worst:?}"
    );
    assert!(!engine.mat_in(), "the mat was ejected");
}

#[test]
fn the_machine_is_given_time_to_eject_the_mat_before_the_unload_is_over() {
    let sim = SimulatedMachine::new();
    let mut engine = connect(&sim);
    engine.begin_job().unwrap();
    let mut ops = stroke(157, 14);
    ops.push(Op::End);
    run(&mut engine, &sim, &ops);
    engine.unload().unwrap();
    let start = engine.link.port.clock;
    loop {
        if matches!(engine.step().unwrap(), Step::Update(Update::Unloaded)) {
            break;
        }
    }
    let took = engine.link.port.clock.saturating_sub(start);
    assert!(
        took >= Duration::from_millis(3000),
        "the unload finished after {took:?}"
    );
}

// ---- a pen left down across a section break ----

#[test]
fn a_tool_change_with_the_pen_down_goes_on_cutting_at_the_same_pressure() {
    let sim = SimulatedMachine::new();
    let mut engine = connect(&sim);
    engine.begin_job().unwrap();
    let mut ops = stroke(157, 3);
    ops.pop(); // no `ToolUp`: the pen is still down at the tool change
    ops.push(Op::ToolChange);
    ops.push(Op::Line((INCH + 5 * INCH / 10, 2 * INCH)));
    ops.push(Op::End);
    run(&mut engine, &sim, &ops);
    assert_eq!(pressures(&sim), vec![157, 157], "both sections are cut");
    // The second section starts with a MOVE to where the first stopped, then the line.
    let v = vectors(&sim);
    let moves: Vec<_> = v.iter().filter(|e| e.0 == 2).collect();
    assert!(
        moves.len() >= 3,
        "a leading MOVE in each section and the parks: {v:?}"
    );
}

/// The `Geometry` payload of every job setup the machine received, in order.
fn geometries(sim: &SimulatedMachine) -> Vec<Vec<u8>> {
    sim.state()
        .received
        .iter()
        .filter(|c| c.opcode == op::GEOMETRY)
        .map(|c| c.payload.clone())
        .collect()
}

fn geometry_of(speed: Speed) -> Vec<u8> {
    setup_commands_with(Clamp::B, speed, 120)[2].payload.clone()
}

fn prompts(updates: &[Update]) -> usize {
    updates
        .iter()
        .filter(|u| matches!(u, Update::Prompt { .. }))
        .count()
}

#[test]
fn a_speed_the_program_names_is_sent_with_the_setup() {
    let sim = SimulatedMachine::new();
    let mut engine = connect(&sim);
    engine.begin_job().unwrap();
    let fast = Speed::FAST;
    let mut ops = vec![Op::Speed(fast)];
    ops.extend(stroke(120, 14));
    ops.push(Op::End);
    run(&mut engine, &sim, &ops);
    assert_eq!(geometries(&sim), vec![geometry_of(fast)]);
}

#[test]
fn a_program_that_names_no_speed_runs_at_normal_speed() {
    let sim = SimulatedMachine::new();
    let mut engine = connect(&sim);
    engine.begin_job().unwrap();
    let mut ops = stroke(120, 14);
    ops.push(Op::End);
    run(&mut engine, &sim, &ops);
    assert_eq!(geometries(&sim), vec![geometry_of(Speed::NORMAL)]);
}

#[test]
fn naming_the_same_speed_again_changes_nothing() {
    let sim = SimulatedMachine::new();
    let mut engine = connect(&sim);
    engine.begin_job().unwrap();
    let speed = Speed::from_mm_s(76.2, 76.2);
    let mut ops = vec![Op::Speed(speed)];
    ops.extend(stroke(120, 14));
    ops.push(Op::Speed(speed));
    ops.extend(stroke(120, 14));
    ops.push(Op::End);
    let updates = run(&mut engine, &sim, &ops);
    assert_eq!(geometries(&sim), vec![geometry_of(speed)]);
    assert_eq!(prompts(&updates), 1, "one section, one prompt");
}

#[test]
fn a_different_speed_after_something_was_cut_starts_a_new_section() {
    // The speed is part of the setup, so, like a change of pressure, it begins a new section with
    // its own prompt.
    let sim = SimulatedMachine::new();
    let mut engine = connect(&sim);
    engine.begin_job().unwrap();
    let (slow, fast) = (Speed::NORMAL, Speed::FAST);
    let mut ops = vec![Op::Speed(slow)];
    ops.extend(stroke(120, 14));
    ops.push(Op::Speed(fast));
    ops.extend(stroke(120, 14));
    ops.push(Op::End);
    let updates = run(&mut engine, &sim, &ops);
    assert_eq!(geometries(&sim), vec![geometry_of(slow), geometry_of(fast)]);
    assert_eq!(pressures(&sim), vec![120, 120]);
    assert_eq!(prompts(&updates), 2);
}

#[test]
fn a_speed_named_before_anything_is_cut_just_sets_the_sections_speed() {
    let sim = SimulatedMachine::new();
    let mut engine = connect(&sim);
    engine.begin_job().unwrap();
    let mut ops = vec![Op::Speed(Speed::NORMAL), Op::Speed(Speed::FAST)];
    ops.extend(stroke(120, 14));
    ops.push(Op::End);
    let updates = run(&mut engine, &sim, &ops);
    assert_eq!(geometries(&sim), vec![geometry_of(Speed::FAST)]);
    assert_eq!(prompts(&updates), 1);
}

#[test]
fn a_speed_change_with_the_pen_down_goes_on_cutting_in_the_same_clamp_and_pressure() {
    let sim = SimulatedMachine::new();
    let mut engine = connect(&sim);
    engine.begin_job().unwrap();
    let mut ops = vec![Op::Clamp(Clamp::A), Op::Speed(Speed::NORMAL)];
    ops.extend(stroke(120, 3));
    ops.pop(); // no `ToolUp`: the pen is still down when the speed changes
    ops.push(Op::Speed(Speed::FAST));
    ops.push(Op::Line((INCH + 5 * INCH / 10, 2 * INCH)));
    ops.push(Op::End);
    run(&mut engine, &sim, &ops);
    assert_eq!(geometries(&sim).len(), 2);
    assert_eq!(pressures(&sim), vec![120, 120]);
    // Clamp A both times: its setup starts `a9`, not the blade's `28`.
    assert!(machine_states(&sim).iter().all(|m| m[0] == 0xA9));
    // The second section starts with a MOVE to where the first stopped, then the line.
    let moves = vectors(&sim).iter().filter(|e| e.0 == 2).count();
    assert!(moves >= 3, "a leading MOVE in each section and the parks");
}

// ---- pacing: how frames are offered to a FIFO that drains over time ----

/// The machine of the fill run: a 34-slot FIFO, and 20 ms over each vector.
fn slow_fifo(sim: &SimulatedMachine) {
    sim.set_fifo(Some(crate::transport::FifoModel {
        capacity: 34,
        entry_time: Duration::from_millis(20),
    }));
}

/// One stroke of `vectors` vectors a tenth of a millimetre apart, zigzagging 5 mm across.
fn tiny_stroke(force: u32, vectors: i64) -> Vec<Op> {
    let mut ops = vec![Op::ToolDown(force), Op::Travel((INCH, INCH))];
    for i in 1..=vectors {
        ops.push(Op::Line((INCH + (i % 2) * 5_000, INCH + i * 100)));
    }
    ops.push(Op::ToolUp);
    ops
}

/// Cuts `vectors` tiny vectors in one stroke with the given pacing against the slow FIFO; the time
/// the machine had nothing to do and how many frames it refused.
fn pace(pacing: Pacing, vectors: i64) -> (Duration, u32) {
    let sim = SimulatedMachine::new();
    slow_fifo(&sim);
    let mut engine = connect(&sim);
    engine.set_pacing(pacing);
    engine.begin_job().unwrap();
    let mut ops = tiny_stroke(120, vectors);
    ops.push(Op::End);
    run(&mut engine, &sim, &ops);
    let st = sim.state();
    assert_eq!(st.bad_frames, 0);
    (st.fifo.as_ref().unwrap().starved(), st.dropped)
}

#[test]
fn the_legacy_pacing_leaves_the_machine_with_nothing_to_do() {
    // 300 vectors are 23 frames. Told it has 4 slots free after a second frame, the legacy pacing
    // sleeps 1.1 s although the machine needs 260 ms to finish a frame, so the FIFO runs dry.
    let (starved, _) = pace(Pacing::LEGACY, 300);
    assert!(
        starved > Duration::from_secs(5),
        "the machine was idle for only {starved:?}"
    );
}

#[test]
fn quick_pacing_keeps_the_machine_busy() {
    let (legacy, _) = pace(Pacing::LEGACY, 300);
    let (quick, _) = pace(Pacing::quick(Duration::from_millis(40)), 300);
    // The machine can have a frame queued behind the one it is working on, so a 40 ms poll never
    // lets it run dry for long: well under a tenth of what the legacy pacing allows.
    assert!(quick < legacy / 10, "quick {quick:?}, legacy {legacy:?}");
    assert!(quick < Duration::from_millis(500), "{quick:?}");
}

#[test]
fn smaller_frames_are_sent_when_asked_for() {
    let sim = SimulatedMachine::new();
    let mut engine = connect(&sim);
    engine.set_pacing(Pacing::LEGACY.with_frame_entries(6).unwrap());
    engine.begin_job().unwrap();
    let mut ops = stroke(120, 20);
    ops.push(Op::End);
    run(&mut engine, &sim, &ops);
    let sizes: Vec<usize> = sim
        .state()
        .received
        .iter()
        .filter(|c| c.opcode == op::MULTI_DRAW_VECTOR)
        .map(|c| c.payload.len() / 9)
        .collect();
    // 19 lines and the leading MOVE are 20 entries: 6, 6, 6, 2, and then the park frame of 2.
    assert_eq!(sizes, vec![6, 6, 6, 2, 2]);
}

#[test]
fn quick_pacing_gives_up_on_a_machine_that_never_makes_room() {
    // The machine takes an hour over each vector, so after the first two frames nothing is ever
    // accepted. Quick pacing stops 30 s after the last frame it got in; legacy would keep trying
    // for 300 attempts of up to 3 s.
    let sim = SimulatedMachine::new();
    sim.set_fifo(Some(crate::transport::FifoModel {
        capacity: 34,
        entry_time: Duration::from_secs(3_600),
    }));
    let mut engine = connect(&sim);
    engine.set_pacing(Pacing::quick(Duration::from_millis(40)));
    engine.begin_job().unwrap();
    let mut ops = tiny_stroke(120, 100);
    ops.push(Op::End);
    let mut next = 0;
    let mut failure = None;
    for _ in 0..200_000 {
        while next < ops.len() && engine.feed(&ops[next]).expect("a valid op") {
            next += 1;
        }
        if engine.activity() == Activity::WaitingForGo {
            sim.press_go();
        }
        match engine.step() {
            Ok(Step::Update(Update::Prompt { id, .. })) => {
                sim.load_mat();
                engine.confirm(id);
            }
            Ok(_) => {}
            Err(e) => {
                failure = Some(e);
                break;
            }
        }
    }
    assert!(
        matches!(failure, Some(Error::MachineDropping { .. })),
        "{failure:?}"
    );
    let clock = sim.state().clock;
    assert!(
        clock >= Duration::from_secs(30) && clock < Duration::from_secs(40),
        "gave up at {clock:?}"
    );
}
