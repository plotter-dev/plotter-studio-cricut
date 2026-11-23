//! Cut a G-code file.
//!
//! ```text
//! cargo run --example cut_gcode -- <file.gcode> --dry-run              # parse and show the plan
//! cargo run --example cut_gcode -- <file.gcode> --simulate             # run against the simulator
//! cargo run --example cut_gcode -- <file.gcode> COM5 [material] [less|more]
//! ```
//!
//! `material` is a dial name such as "Vinyl" or "Cardstock"; without it the machine's dial is used.

#![expect(
    clippy::print_stdout,
    clippy::print_stderr,
    reason = "example program: output is its interface"
)]

use cricut_gcode::job;
use cricut_gcode::transport::{Link, SerialLink, SimulatedMachine};
use cricut_gcode::{
    Config, Cricut, Event, GcodeOptions, Job, JobStatus, Material, MaterialSelection,
    PressureAdjust,
};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let [file, target, rest @ ..] = args.as_slice() else {
        anyhow::bail!(
            "usage: cut_gcode <file.gcode> (--dry-run | --simulate | <port> [material] [less|more])"
        );
    };

    let src = std::fs::read_to_string(file)?;
    let mut job = Job::from_gcode(&src, &GcodeOptions::default())?;
    for w in &job.plan.warnings {
        eprintln!("warning: {w}");
    }
    let (lo, hi) = job
        .plan
        .bounds()
        .ok_or_else(|| anyhow::anyhow!("nothing to cut"))?;
    println!(
        "{} paths, {} vectors, {} frames; machine-inch bounds ({:.3}, {:.3}) – ({:.3}, {:.3})",
        job.plan.polylines.len(),
        job.plan.vector_count(),
        job::vector_frames(&job.plan.polylines).len(),
        lo.x,
        lo.y,
        hi.x,
        hi.y
    );

    let adjust = match rest.get(1).map(|s| s.to_lowercase()) {
        Some(s) if s == "less" => PressureAdjust::Less,
        Some(s) if s == "more" => PressureAdjust::More,
        _ => PressureAdjust::Default,
    };
    if let Some(name) = rest.first() {
        let m =
            Material::from_name(name).ok_or_else(|| anyhow::anyhow!("unknown material {name}"))?;
        job = job.material(MaterialSelection::Material(m, adjust));
    } else {
        job = job.material(MaterialSelection::FromDial(adjust));
    }

    match target.as_str() {
        "--dry-run" => Ok(()),
        "--simulate" => {
            let sim = SimulatedMachine::new();
            let auto = sim.clone();
            run(sim.link(), job, move |ev| match ev {
                Event::JobStatusChanged(JobStatus::WaitingForMat) => auto.load_mat(),
                Event::JobStatusChanged(JobStatus::WaitingForGo) => auto.press_go(),
                _ => {}
            })
            .await
        }
        port => run(SerialLink::open(port).await?, job, |_| {}).await,
    }
}

async fn run<L: Link>(
    link: L,
    job: Job,
    mut on_event: impl FnMut(&Event) + Send + 'static,
) -> anyhow::Result<()> {
    let cricut = Cricut::connect(link, Config::default()).await?;
    let mut events = cricut.subscribe();
    tokio::spawn(async move {
        while let Ok(ev) = events.recv().await {
            match &ev {
                Event::JobStatusChanged(JobStatus::WaitingForMat) => println!("Load the mat."),
                Event::JobStatusChanged(JobStatus::WaitingForGo) => println!("Press Go."),
                Event::MaterialSelected(m) => {
                    println!("material: {:?}, pressure {}", m.material, m.pressure);
                }
                Event::Progress(p) => println!("frame {}/{}", p.frames_sent, p.frames_total),
                Event::JobStatusChanged(s) => println!("status: {s:?}"),
                Event::Warning(w) => println!("warning: {w}"),
                _ => {}
            }
            on_event(&ev);
        }
    });

    let result = tokio::select! {
        r = cricut.run_job(job) => r,
        _ = tokio::signal::ctrl_c() => {
            cricut.cancel();
            Err(cricut_gcode::Error::Cancelled)
        }
    };
    match &result {
        Ok(report) => println!("done: {report:?}"),
        Err(e) => println!("job ended: {e}"),
    }
    println!("unloading…");
    cricut.unload().await?;
    cricut.shutdown();
    result.map(|_| ()).map_err(Into::into)
}
