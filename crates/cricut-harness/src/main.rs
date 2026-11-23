//! Hardware test harness. Run the stages in order against a real machine:
//!
//! ```text
//! cargo run -p cricut-harness -- ports
//! cargo run -p cricut-harness -- probe COM5          # read-only checks + encrypted session
//! cargo run -p cricut-harness -- watch COM5 [secs]   # live events: turn the dial, load a mat
//! cargo run -p cricut-harness -- cut COM5 [material] [--less|--more] [--pressure=N] [--no-go]
//! cargo run -p cricut-harness -- ladder COM5 [--from=145] [--step=12] [--count=7] [--row=0]
//! cargo run -p cricut-harness -- offsets COM5 [--pressure=169] [--row=4] [--overcut-mm=0] [--offsets-mm=0.1,0.2,...] [--right-to-left]
//! cargo run -p cricut-harness -- firstshape COM5 [--pressure=157] [--row=8] [--overcuts-mm=0,0.5,1,2]
//! cargo run -p cricut-harness -- speed COM5 [--plan] [--gcode=FILE] [--pacing=legacy|quick[:MS]] [--frame-entries=N] [--simplify=MM] [--turns=N] [--pressure=120] [--length-mm=100] [--passes=5] [--profiles=0:n,250:n,500:n,750:n,1000:n,1000:f]
//! ```
//!
//! `speed` draws one line with a pen in clamp A at several speed settings, each as a single pass
//! and as several, and times every job. The extra time of the long run over the short one is the
//! extra passes alone, so setup, moving in and parking cancel out. A setting is `PERMILLE:n|f`:
//! how far the four speed registers are from Normal (0) to Fast Mode's values (1000), and whether
//! the job sends Fast Mode's `Geometry` block (`f`) or Normal's (`n`). Values between the two
//! profiles have never been seen on the wire; they stay inside the range Design Space sends.
//! `--plan` prints the rows and the settings and connects to nothing. `--gcode=FILE` runs compiled
//! G-code as it is, once per setting (default: Normal, then 4 in/s with the second `Geometry` float
//! at 0, 500 and 1000), and compares the total times.
//!
//! `ladder` cuts a row of 0.6-inch squares, left to right at increasing pressure, to find where
//! the blade cuts through. Each square is its own job (the known-good way to set pressure) on a
//! single mat load and a single Go press.
//!
//! `cut` draws a 1-inch square and a ½-inch circle near the top-left of the sheet. Without a
//! material name it uses the machine's dial. The machine waits for you to load the mat and
//! press Go (unless `--no-go`), so nothing moves until you act. Ctrl+C cancels and ejects.
//!
//! Every run writes a full wire log (tx/rx bytes, decrypted encrypted traffic) to
//! `target/harness/<stage>-<unix time>.log`.

#![expect(
    clippy::print_stdout,
    reason = "a console tool: stdout is its user interface; diagnostics go to the wire log"
)]

use std::fmt::Write as _;
use std::path::PathBuf;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::{Context, bail};
use cricut_gcode::Pacing;
use cricut_gcode::job::Clamp;
use cricut_gcode::protocol::{self, MachineType, op};
use cricut_gcode::session::Session;
use cricut_gcode::transport::{SerialLink, find_usb_ports};
use cricut_gcode::{
    Config, Cricut, DialSetting, Event, GcodeOptions, Job, JobStatus, Material, MaterialSelection,
    PressureAdjust,
};

mod speed;

/// 1" square at (0.5", 0.5") from the sheet's top-left, and a ½"-diameter circle beside it.
/// Bottom-left origin on a 12" sheet, millimetres.
const TEST_GCODE: &str = "\
G21 G90
M5
G0 X12.7 Y292.1
M3
G1 X38.1 Y292.1
G1 X38.1 Y266.7
G1 X12.7 Y266.7
G1 X12.7 Y292.1
M5
G0 X63.5 Y279.4
M3
G2 X63.5 Y279.4 I-6.35 J0
M5
";

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let stage = args.first().map_or("help", String::as_str);
    let port = args.get(1).cloned();
    let need_port = || {
        port.clone()
            .context("missing PORT (e.g. COM5); try `harness ports`")
    };

    let log = init_log(stage)?;
    let result = match stage {
        "ports" => ports(),
        "probe" => probe(&need_port()?).await,
        "watch" => {
            let secs = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(30);
            watch(&need_port()?, secs).await
        }
        "cut" => cut(&need_port()?, &args[2..]).await,
        "ladder" => ladder(&need_port()?, &args[2..]).await,
        "corners" => corners(&need_port()?, &args[2..]).await,
        "offsets" => offsets(&need_port()?, &args[2..]).await,
        "firstshape" => first_shape(&need_port()?, &args[2..]).await,
        "speed" => speed_stage(&need_port()?, &args[2..]).await,
        _ => {
            println!(
                "usage: harness (ports | probe PORT | watch PORT [secs] | cut PORT [material] [--less|--more] [--pressure=N] [--no-go] | ladder PORT [--from=N] [--step=N] [--count=N] [--row=N] | offsets PORT [--pressure=N] [--row=N] [--overcut-mm=X] [--offsets-mm=A,B,...] [--right-to-left] | firstshape PORT [--pressure=N] [--row=N] [--overcuts-mm=A,B,...] | speed PORT [--plan] [--gcode=FILE] [--pacing=legacy|quick[:MS]] [--frame-entries=N] [--simplify=MM] [--turns=N] [--pressure=N] [--length-mm=N] [--passes=N] [--profiles=PERMILLE:n|f,...])"
            );
            return Ok(());
        }
    };
    println!("\nwire log: {}", log.display());
    result
}

fn init_log(stage: &str) -> anyhow::Result<PathBuf> {
    let dir = PathBuf::from("target/harness");
    std::fs::create_dir_all(&dir)?;
    let secs = SystemTime::now().duration_since(UNIX_EPOCH)?.as_secs();
    let path = dir.join(format!("{stage}-{secs}.log"));
    let file = std::fs::File::create(&path)?;
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::TRACE)
        .with_ansi(false)
        .with_writer(std::sync::Mutex::new(file))
        .init();
    Ok(path)
}

fn ports() -> anyhow::Result<()> {
    let ports = find_usb_ports()?;
    if ports.is_empty() {
        bail!("no Cricut USB ports found");
    }
    for p in ports {
        println!(
            "{}  {}  serial={}",
            p.port_name,
            p.product.unwrap_or_default(),
            p.serial_number.unwrap_or_default()
        );
    }
    Ok(())
}

/// Step-by-step checks, lowest layer first, so a failure points at the broken piece.
async fn probe(port: &str) -> anyhow::Result<()> {
    let mut checks = Checks::default();

    let link = SerialLink::open(port)
        .await
        .with_context(|| format!("opening {port}"))?;
    checks.pass("open serial port", port);
    let mut s = Session::new(link, MachineType::ExploreAir2);

    match s.machine_status().await? {
        Some(body) => checks.pass("GetMachineStatus 0x60", hex(&body)),
        None => checks.fail("GetMachineStatus 0x60", "no reply"),
    }
    match s.interface_state().await? {
        Some(i) => checks.pass(
            "GetInterfaceState 0x73",
            format!(
                "lights={:02x} buttons={:02x} mat_loaded={} dial={} ({})",
                i.lights,
                i.buttons,
                i.mat_loaded(),
                i.dial,
                DialSetting::from_raw(i.dial)
            ),
        ),
        None => checks.fail("GetInterfaceState 0x73", "no/unparseable reply"),
    }
    match s.dial().await? {
        Some(d) => checks.pass(
            "GetMaterialDial 0x38",
            format!("{d} = {}", DialSetting::from_raw(d)),
        ),
        None => checks.fail("GetMaterialDial 0x38", "no reply"),
    }
    match s.position().await? {
        Some((x, y)) => checks.pass("ReadPosition 0xB9", format!("({x:.3}, {y:.3}) in")),
        None => checks.fail("ReadPosition 0xB9", "no reply"),
    }

    let t = Instant::now();
    match s.handshake().await {
        Ok(()) => checks.pass(
            "NewAES handshake",
            format!("session key established in {:?}", t.elapsed()),
        ),
        Err(e) => {
            checks.fail("NewAES handshake", e.to_string());
            return checks.finish();
        }
    }

    // EndDraw with nothing open is what every Design Space job sends first: harmless, and its
    // ACK proves the session key is right (a wrong key yields garbage or silence).
    match s
        .send_enc(op::END_DRAW, &[0, 0, 0], Duration::from_millis(1000))
        .await?
    {
        Some(r) if r.body.get(2) == Some(&op::END_DRAW) => checks.pass(
            "encrypted EndDraw 0x22",
            format!("hdr={:02x} decrypted ACK: {}", r.header, hex(&r.body)),
        ),
        Some(r) => checks.fail(
            "encrypted EndDraw 0x22",
            format!(
                "hdr={:02x} unexpected reply: {} (wrong key?)",
                r.header,
                hex(&r.body)
            ),
        ),
        None => checks.fail("encrypted EndDraw 0x22", "no reply"),
    }
    match s.interface_state().await? {
        Some(_) => checks.pass("plaintext after session", "GetInterfaceState still answers"),
        None => checks.fail("plaintext after session", "no reply"),
    }
    drop(s);

    // The high-level controller over a fresh connection.
    let t = Instant::now();
    let cricut = Cricut::connect(SerialLink::open(port).await?, Config::default()).await?;
    let st = cricut.state();
    checks.pass(
        "Cricut::connect",
        format!(
            "{:?} in {:?}; mat_loaded={:?} dial={:?}",
            st.connection,
            t.elapsed(),
            st.mat_loaded,
            st.dial.map(|d| d.to_string())
        ),
    );
    cricut.shutdown();
    checks.finish()
}

async fn watch(port: &str, secs: u64) -> anyhow::Result<()> {
    let cricut = Cricut::connect(SerialLink::open(port).await?, Config::default()).await?;
    let st = cricut.state();
    println!(
        "connected; mat_loaded={:?} dial={:?}",
        st.mat_loaded,
        st.dial.map(|d| d.to_string())
    );
    println!("watching for {secs}s: turn the dial, load/unload a mat, press buttons…");
    let mut events = cricut.subscribe();
    let t0 = Instant::now();
    let deadline = tokio::time::sleep(Duration::from_secs(secs));
    tokio::pin!(deadline);
    loop {
        tokio::select! {
            ev = events.recv() => println!("[{:>6.2}s] {}", t0.elapsed().as_secs_f32(), describe(&ev?)),
            () = &mut deadline => break,
            _ = tokio::signal::ctrl_c() => break,
        }
    }
    cricut.shutdown();
    Ok(())
}

async fn cut(port: &str, opts: &[String]) -> anyhow::Result<()> {
    let mut no_go = false;
    let mut adjust = PressureAdjust::Default;
    let mut pressure: Option<u16> = None;
    let mut name: Option<&str> = None;
    for o in opts {
        match o.as_str() {
            "--no-go" => no_go = true,
            "--less" => adjust = PressureAdjust::Less,
            "--more" => adjust = PressureAdjust::More,
            p if p.starts_with("--pressure=") => {
                pressure = Some(p["--pressure=".len()..].parse().context("--pressure=N")?);
            }
            // Refuse unknown flags rather than silently running a real job.
            f if f.starts_with("--") => bail!("unknown option {f}"),
            n => name = Some(n),
        }
    }
    let material = match (pressure, name) {
        (Some(p), _) => MaterialSelection::Pressure(p),
        (None, Some(name)) => {
            let m = Material::from_name(name).with_context(|| {
                let names: Vec<_> = Material::ALL.iter().map(|m| m.name()).collect();
                format!("unknown material {name:?}; one of {names:?}")
            })?;
            MaterialSelection::Material(m, adjust)
        }
        (None, None) => MaterialSelection::FromDial(adjust),
    };

    let job = Job::from_gcode(TEST_GCODE, &GcodeOptions::default())?
        .material(material)
        .wait_for_go(!no_go);
    let (lo, hi) = job.plan.bounds().context("empty plan")?;
    println!(
        "test job: {} paths, {} vectors; machine-inch bounds ({:.2}, {:.2})–({:.2}, {:.2})",
        job.plan.polylines.len(),
        job.plan.vector_count(),
        lo.x,
        lo.y,
        hi.x,
        hi.y
    );

    let cricut = Cricut::connect(SerialLink::open(port).await?, Config::default()).await?;
    let mut events = cricut.subscribe();
    let t0 = Instant::now();
    tokio::spawn(async move {
        while let Ok(ev) = events.recv().await {
            println!("[{:>6.2}s] {}", t0.elapsed().as_secs_f32(), describe(&ev));
        }
    });

    let result = tokio::select! {
        r = cricut.run_job(job) => r,
        _ = tokio::signal::ctrl_c() => {
            println!("Ctrl+C: cancelling");
            cricut.cancel();
            Err(cricut_gcode::Error::Cancelled)
        }
    };
    match &result {
        Ok(r) => println!("\nPASS job: {r:?}"),
        Err(e) => println!("\nFAIL job: {e}"),
    }
    println!("unloading (park + eject)…");
    cricut.unload().await?;
    tokio::time::sleep(Duration::from_millis(200)).await;
    cricut.shutdown();
    result.map(|_| ()).map_err(Into::into)
}

/// Corner diagnostic: four 0.6" squares in one job, each drawn differently, so the location of
/// any uncut corner tells drag-knife lag (every corner) apart from a first-corner effect.
async fn corners(port: &str, opts: &[String]) -> anyhow::Result<()> {
    let mut pressure = 169u16;
    for o in opts {
        match o.as_str() {
            p if p.starts_with("--pressure=") => {
                pressure = p["--pressure=".len()..].parse().context("--pressure=N")?;
            }
            f => bail!("unknown option {f}"),
        }
    }
    if pressure > LADDER_MAX_PRESSURE {
        bail!("pressure {pressure} exceeds the safety cap of {LADDER_MAX_PRESSURE}");
    }

    // Corners of a 0.6" square whose top-left is at (x, 0.5), sheet inches, Y down.
    let sq = |x: f64| {
        let (l, r, t, b) = (x, x + 0.6, 0.5, 1.1);
        [(l, t), (r, t), (r, b), (l, b)] // TL, TR, BR, BL
    };
    let path = |pts: &[(f64, f64)]| {
        let mut s = format!("G0 X{} Y{}\n", pts[0].0, pts[0].1);
        for p in &pts[1..] {
            let _ = writeln!(s, "G1 X{} Y{}", p.0, p.1);
        }
        s
    };
    let a = sq(0.5);
    let b = sq(1.5);
    let c = sq(2.5);
    let d = sq(3.5);
    let figures = [
        (
            "A",
            "clockwise from top-left (same as before)",
            path(&[a[0], a[1], a[2], a[3], a[0]]),
        ),
        (
            "B",
            "counter-clockwise from top-left (first corner: bottom-left)",
            path(&[b[0], b[3], b[2], b[1], b[0]]),
        ),
        (
            "C",
            "clockwise from bottom-right (first corner: bottom-left)",
            path(&[c[2], c[3], c[0], c[1], c[2]]),
        ),
        (
            "D",
            "clockwise from top-left, overcut 0.2\" past the start",
            path(&[d[0], d[1], d[2], d[3], d[0], (d[0].0 + 0.2, d[0].1)]),
        ),
    ];

    let mut src = String::from("G20 G90\n");
    println!("corner diagnostic at pressure {pressure}, one job, left to right:");
    for (i, (name, desc, p)) in figures.iter().enumerate() {
        println!(
            "  {name}  x = {:.1}–{:.1} in   {desc}",
            column_x(i),
            column_x(i) + 0.6
        );
        src.push_str(p);
    }
    println!();
    let opts = GcodeOptions {
        origin: cricut_gcode::Origin::TopLeft,
        ..GcodeOptions::default()
    };
    let job = Job::from_gcode(&src, &opts)?.material(MaterialSelection::Pressure(pressure));
    run_jobs(port, vec![("corner diagnostic (squares A-D)".into(), job)]).await
}

/// Connect, then run labelled jobs in order on one mat load, printing events as they happen;
/// finally park and eject. Each job's own `wait_for_go` decides whether it waits for Go.
/// Ctrl+C cancels the running job (the mat is still ejected).
async fn run_jobs(port: &str, jobs: Vec<(String, Job)>) -> anyhow::Result<()> {
    run_jobs_with(port, jobs, Config::default())
        .await
        .map(|_| ())
}

/// Runs the jobs in order on one connection and returns how long each took, from the call to the
/// carriage being back at park. Stops at the first job that fails.
async fn run_jobs_with(
    port: &str,
    jobs: Vec<(String, Job)>,
    config: Config,
) -> anyhow::Result<Vec<Duration>> {
    let cricut = Cricut::connect(SerialLink::open(port).await?, config).await?;
    let mut events = cricut.subscribe();
    let t0 = Instant::now();
    tokio::spawn(async move {
        while let Ok(ev) = events.recv().await {
            if !matches!(ev, Event::PositionChanged(_) | Event::Progress(_)) {
                println!("[{:>6.2}s] {}", t0.elapsed().as_secs_f32(), describe(&ev));
            }
        }
    });
    {
        let c = cricut.clone();
        tokio::spawn(async move {
            if tokio::signal::ctrl_c().await.is_ok() {
                println!("Ctrl+C: cancelling");
                c.cancel();
            }
        });
    }

    let mut results = Vec::new();
    let mut times = Vec::new();
    for (label, job) in jobs {
        println!("--- {label} ---");
        let started = Instant::now();
        let result = cricut.run_job(job).await;
        let took = started.elapsed();
        if result.is_ok() {
            println!("    took {:.2} s", took.as_secs_f64());
            times.push(took);
        }
        let stop = result.is_err();
        results.push((label, result));
        if stop {
            break;
        }
    }

    println!("unloading (park + eject)…");
    cricut.unload().await?;
    tokio::time::sleep(Duration::from_millis(200)).await;
    cricut.shutdown();

    println!("\nresults:");
    let mut failed = 0;
    for (label, result) in &results {
        match result {
            Ok(r) => println!(
                "  PASS  {label}: {} frames, {} vectors, {} retries, parked={}",
                r.frames, r.vectors, r.retries, r.parked
            ),
            Err(e) => {
                failed += 1;
                println!("  FAIL  {label}: {e}");
            }
        }
    }
    if failed > 0 {
        bail!("{failed} job(s) did not complete");
    }
    Ok(times)
}

/// First-shape test: one 0.6" square per job, all on one mat load and one Go press, so every
/// square is the first shape of its job. Each job uses a different overcut.
async fn first_shape(port: &str, opts: &[String]) -> anyhow::Result<()> {
    let (mut pressure, mut row) = (157u16, 8u32);
    let mut overcuts_mm: Vec<f64> = vec![0.0, 0.5, 1.0, 2.0];
    for o in opts {
        let val = |prefix: &str| o[prefix.len()..].to_string();
        match o.as_str() {
            s if s.starts_with("--pressure=") => {
                pressure = val("--pressure=").parse().context("--pressure=N")?;
            }
            s if s.starts_with("--row=") => row = val("--row=").parse().context("--row=N")?,
            s if s.starts_with("--overcuts-mm=") => {
                overcuts_mm = val("--overcuts-mm=")
                    .split(',')
                    .map(|v| v.trim().parse::<f64>())
                    .collect::<Result<_, _>>()
                    .context("--overcuts-mm=A,B,C (millimetres)")?;
            }
            f => bail!("unknown option {f}"),
        }
    }
    if pressure > LADDER_MAX_PRESSURE {
        bail!("pressure {pressure} exceeds the safety cap of {LADDER_MAX_PRESSURE}");
    }
    if row > 9 {
        bail!("--row must be 0..=9 to stay on a letter-size sheet");
    }
    if !(1..=7).contains(&overcuts_mm.len()) || overcuts_mm.iter().any(|v| !(0.0..=5.0).contains(v))
    {
        bail!("--overcuts-mm needs 1..=7 values, each 0..=5 mm");
    }

    let gopts = GcodeOptions {
        origin: cricut_gcode::Origin::TopLeft,
        ..GcodeOptions::default()
    };
    let y = 0.5 + f64::from(row);
    println!(
        "first-shape test at pressure {pressure}, row {row} (top edge {y:.1} in below the sheet \
         top); one job per square, left to right:"
    );
    let mut jobs = Vec::new();
    for (i, mm) in overcuts_mm.iter().enumerate() {
        let x = column_x(i);
        let (x2, y2) = (x + 0.6, y + 0.6);
        let src = format!(
            "G20 G90\nG0 X{x} Y{y}\nG1 X{x2} Y{y}\nG1 X{x2} Y{y2}\nG1 X{x} Y{y2}\nG1 X{x} Y{y}\n"
        );
        let knife = cricut_gcode::DragKnife::new(0.0).overcut(mm / 25.4);
        let job = Job::from_gcode(&src, &gopts)?
            .drag_knife(knife)
            .material(MaterialSelection::Pressure(pressure))
            .wait_for_go(i == 0);
        let label = if *mm == 0.0 {
            "no overcut (control)".to_string()
        } else {
            format!("overcut {mm} mm")
        };
        println!("  #{}  x = {x:.1}–{x2:.1} in   {label}", i + 1);
        jobs.push((format!("square #{} ({label})", i + 1), job));
    }
    println!();
    run_jobs(port, jobs).await
}

/// Drag-knife calibration: one job, six identical 0.6" squares (clockwise from top-left),
/// compensated with blade offsets 0.1–0.5 mm, plus an uncompensated control at the right.
async fn offsets(port: &str, opts: &[String]) -> anyhow::Result<()> {
    let (mut pressure, mut row, mut overcut_mm) = (169u16, 4u32, 0.0f64);
    let mut offsets_mm: Vec<f64> = vec![0.1, 0.2, 0.3, 0.4, 0.5, 0.0];
    let mut right_to_left = false;
    for o in opts {
        let val = |prefix: &str| o[prefix.len()..].to_string();
        match o.as_str() {
            "--right-to-left" => right_to_left = true,
            s if s.starts_with("--offsets-mm=") => {
                offsets_mm = val("--offsets-mm=")
                    .split(',')
                    .map(|v| v.trim().parse::<f64>())
                    .collect::<Result<_, _>>()
                    .context("--offsets-mm=A,B,C (millimetres)")?;
            }
            s if s.starts_with("--pressure=") => {
                pressure = val("--pressure=").parse().context("--pressure=N")?;
            }
            s if s.starts_with("--row=") => row = val("--row=").parse().context("--row=N")?,
            s if s.starts_with("--overcut-mm=") => {
                overcut_mm = val("--overcut-mm=").parse().context("--overcut-mm=X")?;
            }
            f => bail!("unknown option {f}"),
        }
    }
    if pressure > LADDER_MAX_PRESSURE {
        bail!("pressure {pressure} exceeds the safety cap of {LADDER_MAX_PRESSURE}");
    }
    if row > 9 {
        bail!("--row must be 0..=9 to stay on a letter-size sheet");
    }
    if !(0.0..=5.0).contains(&overcut_mm) {
        bail!("--overcut-mm must be 0..=5");
    }
    if !(1..=7).contains(&offsets_mm.len()) || offsets_mm.iter().any(|v| !(0.0..=2.0).contains(v)) {
        bail!("--offsets-mm needs 1..=7 values, each 0..=2 mm, to fit on a letter-size sheet");
    }

    let gopts = GcodeOptions {
        origin: cricut_gcode::Origin::TopLeft,
        ..GcodeOptions::default()
    };
    let y = 0.5 + f64::from(row);
    let mut plan = cricut_gcode::CutPlan::default();
    println!(
        "drag-knife calibration at pressure {pressure}, overcut {overcut_mm} mm, row {row} \
         (top edge {y:.1} in below the sheet top), left to right:"
    );
    for (i, mm) in offsets_mm.iter().enumerate() {
        let x = column_x(i);
        let (x2, y2) = (x + 0.6, y + 0.6);
        let src = format!(
            "G20 G90\nG0 X{x} Y{y}\nG1 X{x2} Y{y}\nG1 X{x2} Y{y2}\nG1 X{x} Y{y2}\nG1 X{x} Y{y}\n"
        );
        let knife = cricut_gcode::DragKnife::from_mm(*mm).overcut(overcut_mm / 25.4);
        let sq = knife.compensate_plan(&cricut_gcode::gcode::parse_with(&src, &gopts)?);
        let label = if *mm == 0.0 {
            "no compensation (control)".to_string()
        } else {
            format!("offset {mm:.1} mm")
        };
        println!(
            "  #{}  x = {x:.1}–{x2:.1} in   {label}  ({} vectors)",
            i + 1,
            sq.vector_count()
        );
        plan.polylines.extend(sq.polylines);
    }
    if right_to_left {
        plan.polylines.reverse();
        println!("cut order: right to left (#{} first)", offsets_mm.len());
    } else {
        println!("cut order: left to right (#1 first)");
    }
    println!();
    let job = Job::new(plan).material(MaterialSelection::Pressure(pressure));
    run_jobs(port, vec![("calibration squares".into(), job)]).await
}

/// Left edge, in sheet inches, of test figure `i` in a row: 0.5" margin, 1" pitch.
fn column_x(i: usize) -> f64 {
    // Rows hold at most 8 figures (validated by every stage), so the index always fits.
    0.5 + f64::from(u32::try_from(i).unwrap_or(u32::MAX))
}

/// Highest pressure the ladder will use (Poster Board + is 307).
const LADDER_MAX_PRESSURE: u16 = 320;

async fn ladder(port: &str, opts: &[String]) -> anyhow::Result<()> {
    let (mut from, mut step, mut count, mut row) = (145u16, 12u16, 7u16, 0u32);
    for o in opts {
        // Parse straight into the target type: out-of-range input is an error, never a wrap.
        if let Some(v) = o.strip_prefix("--from=") {
            from = v.parse().context("--from=N (0..=65535)")?;
        } else if let Some(v) = o.strip_prefix("--step=") {
            step = v.parse().context("--step=N (0..=65535)")?;
        } else if let Some(v) = o.strip_prefix("--count=") {
            count = v.parse().context("--count=N")?;
        } else if let Some(v) = o.strip_prefix("--row=") {
            row = v.parse().context("--row=N")?;
        } else {
            bail!("unknown option {o}");
        }
    }
    // Rows are 1" apart; row 9 ends at 10.1", inside a letter-size sheet.
    if row > 9 {
        bail!("--row must be 0..=9 to stay on a letter-size sheet");
    }
    // 1-inch pitch from 0.5": 8 squares end at 8.1", inside a letter-size sheet.
    if !(1..=8).contains(&count) {
        bail!("--count must be 1..=8 so the row fits on a letter-size sheet");
    }
    let pressures = (0..count)
        .map(|i| {
            i.checked_mul(step)
                .and_then(|d| from.checked_add(d))
                .filter(|p| *p <= LADDER_MAX_PRESSURE)
        })
        .collect::<Option<Vec<u16>>>()
        .with_context(|| {
            format!("pressures must stay within the ladder's safety cap of {LADDER_MAX_PRESSURE}")
        })?;

    // Top-left origin, inches: the row sits 0.5" below the sheet's top edge.
    let opts = GcodeOptions {
        origin: cricut_gcode::Origin::TopLeft,
        ..GcodeOptions::default()
    };
    let mut jobs = Vec::new();
    for (i, p) in pressures.iter().enumerate() {
        let x = column_x(i);
        let y = 0.5 + f64::from(row);
        let (x2, y2) = (x + 0.6, y + 0.6);
        let src = format!(
            "G20 G90\nG0 X{x} Y{y}\nG1 X{x2} Y{y}\nG1 X{x2} Y{y2}\nG1 X{x} Y{y2}\nG1 X{x} Y{y}\n"
        );
        let job = Job::from_gcode(&src, &opts)?
            .material(MaterialSelection::Pressure(*p))
            .wait_for_go(i == 0);
        jobs.push((x, *p, job));
    }

    println!(
        "pressure ladder, row {row} (top edge {:.1} in below the sheet top), left to right:",
        0.5 + f64::from(row)
    );
    for (i, (x, p, _)) in jobs.iter().enumerate() {
        println!("  #{}  x = {x:.1}–{:.1} in   pressure {p}", i + 1, x + 0.6);
    }
    println!();
    let jobs = jobs
        .into_iter()
        .enumerate()
        .map(|(i, (_, p, job))| (format!("figure #{} at pressure {p}", i + 1), job))
        .collect();
    run_jobs(port, jobs).await
}

/// The highest pen pressure the speed stage accepts: it draws on paper and must never cut it.
const SPEED_MAX_PRESSURE: u16 = 200;

/// What the `speed` stage was asked to do.
struct SpeedPlan {
    workload: speed::Workload,
    pressure: u16,
    /// Sizes of the small and the large figure, in the workload's units.
    sizes: (u32, u32),
    profiles: Vec<speed::Profile>,
    /// A G-code file to run as it is, once per setting, instead of the workload's figures.
    program: Option<speed::Program>,
    /// How vector frames are offered to the machine's FIFO.
    pacing: Pacing,
    /// Only print the layout; connect to nothing.
    plan_only: bool,
}

/// Parses `legacy`, `quick` (a 40 ms poll) or `quick:MS` (5 to 500 ms).
fn parse_pacing(text: &str) -> Result<Pacing, String> {
    match text {
        "legacy" => Ok(Pacing::LEGACY),
        "quick" => Ok(Pacing::quick(Duration::from_millis(40))),
        other => {
            let ms = other
                .strip_prefix("quick:")
                .and_then(|n| n.parse::<u64>().ok())
                .filter(|n| (5..=500).contains(n))
                .ok_or_else(|| {
                    format!("{other:?}: expected legacy, quick or quick:MS with MS 5 to 500")
                })?;
            Ok(Pacing::quick(Duration::from_millis(ms)))
        }
    }
}

/// Where a G-code file is drawn: this far from the sheet's top-left corner, in millimetres.
const PROGRAM_OFFSET_MM: f64 = 10.0;

/// The `speed` stage's options as typed, before they are checked against each other.
struct SpeedOptions {
    workload: Option<speed::Workload>,
    pressure: u16,
    long: Option<u32>,
    profiles: Option<Vec<speed::Profile>>,
    gcode: Option<String>,
    pacing: Pacing,
    /// Tolerance for simplifying the file's drawn path, in millimetres; 0 for none.
    simplify_mm: f64,
    /// Chords in each turn of the `rounded` workload, when asked for.
    turns: Option<u32>,
    plan_only: bool,
}

/// Reads the `speed` stage's options and checks each on its own.
fn parse_speed_options(opts: &[String]) -> anyhow::Result<SpeedOptions> {
    let mut o = SpeedOptions {
        workload: None,
        pressure: 120,
        long: None,
        profiles: None,
        gcode: None,
        pacing: Pacing::LEGACY,
        simplify_mm: 0.0,
        turns: None,
        plan_only: false,
    };
    let mut frame_entries = None;
    for arg in opts {
        let val = |prefix: &str| arg[prefix.len()..].to_string();
        match arg.as_str() {
            s if s.starts_with("--workload=") => {
                o.workload = Some(
                    speed::Workload::parse(&val("--workload="))
                        .context("--workload=line|zigzag|curve|rounded")?,
                );
            }
            s if s.starts_with("--gcode=") => o.gcode = Some(val("--gcode=")),
            s if s.starts_with("--pressure=") => {
                o.pressure = val("--pressure=").parse().context("--pressure=N")?;
            }
            s if s.starts_with("--long=") => {
                o.long = Some(val("--long=").parse::<u32>().context("--long=N")?);
            }
            s if s.starts_with("--profiles=") => {
                o.profiles = Some(
                    speed::parse_profiles(&val("--profiles=")).map_err(|e| anyhow::anyhow!(e))?,
                );
            }
            s if s.starts_with("--pacing=") => {
                o.pacing = parse_pacing(&val("--pacing=")).map_err(|e| anyhow::anyhow!(e))?;
            }
            s if s.starts_with("--frame-entries=") => {
                frame_entries = Some(
                    val("--frame-entries=")
                        .parse::<usize>()
                        .context("--frame-entries=N")?,
                );
            }
            s if s.starts_with("--simplify=") => {
                o.simplify_mm = val("--simplify=")
                    .parse::<f64>()
                    .ok()
                    .filter(|v| (0.001..=0.5).contains(v))
                    .context("--simplify=MM with MM 0.001 to 0.5")?;
            }
            s if s.starts_with("--turns=") => {
                o.turns = Some(
                    val("--turns=")
                        .parse::<u32>()
                        .ok()
                        .filter(|n| (2..=24).contains(n))
                        .context("--turns=N with N 2 to 24")?,
                );
            }
            "--plan" => o.plan_only = true,
            _ => bail!("unknown option {arg}"),
        }
    }
    if o.pressure > SPEED_MAX_PRESSURE {
        bail!("--pressure must be at most {SPEED_MAX_PRESSURE}: this stage draws, it never cuts");
    }
    if let Some(entries) = frame_entries {
        o.pacing = o
            .pacing
            .with_frame_entries(entries)
            .context("--frame-entries must be 1 to 13")?;
    }
    Ok(o)
}

/// Reads and checks the `speed` stage's options; nothing is sent to the machine.
fn speed_plan(opts: &[String]) -> anyhow::Result<SpeedPlan> {
    let SpeedOptions {
        workload,
        pressure,
        long,
        profiles,
        gcode,
        pacing,
        simplify_mm,
        turns,
        plan_only,
    } = parse_speed_options(opts)?;
    if let Some(path) = gcode {
        if workload.is_some() || long.is_some() || turns.is_some() {
            bail!("--gcode runs a file as it is: it takes no --workload, --long or --turns");
        }
        let text = std::fs::read_to_string(&path).with_context(|| format!("reading {path}"))?;
        let program = speed::Program::from_gcode_simplified(
            &text,
            PROGRAM_OFFSET_MM,
            PROGRAM_OFFSET_MM,
            simplify_mm,
        )
        .map_err(|e| anyhow::anyhow!("{path}: {e}"))?;
        let profiles = profiles.unwrap_or_else(speed::file_profiles);
        if profiles.is_empty() {
            bail!("--profiles needs at least one entry");
        }
        return Ok(SpeedPlan {
            workload: speed::Workload::Line,
            pressure,
            sizes: (0, 0),
            profiles,
            program: Some(program),
            pacing,
            plan_only,
        });
    }
    let workload = match (workload.unwrap_or(speed::Workload::Line), turns) {
        (speed::Workload::Rounded(_), Some(n)) => speed::Workload::Rounded(n),
        (other, None) => other,
        (_, Some(_)) => bail!("--turns goes with --workload=rounded"),
    };
    let (small, default_large) = workload.default_sizes();
    let large = long.unwrap_or(default_large);
    if large <= small || large > workload.max_size() {
        bail!(
            "--long must be more than {small} and at most {} {}s for the {} workload",
            workload.max_size(),
            workload.unit(),
            workload.name()
        );
    }
    let profiles = profiles.unwrap_or_else(|| workload.default_profiles());
    if profiles.is_empty() {
        bail!("--profiles needs at least one entry");
    }
    let bottom = speed::band_top_mm(workload, profiles.len() - 1) + workload.band_mm();
    if bottom > speed::LAST_ROW_LIMIT_MM {
        bail!(
            "{} settings would reach {bottom:.0} mm; at most {} mm fits the sheet: use fewer --profiles",
            profiles.len(),
            speed::LAST_ROW_LIMIT_MM
        );
    }
    Ok(SpeedPlan {
        workload,
        pressure,
        sizes: (small, large),
        profiles,
        program: None,
        pacing,
        plan_only,
    })
}

/// The jobs of the `speed` stage: an untimed warm-up that carries the Load and Go presses, then
/// either the file once per setting or, for every setting, a small and a large figure side by side
/// on the same rows.
fn speed_jobs(plan: &SpeedPlan) -> anyhow::Result<Vec<(String, Job)>> {
    let opts = GcodeOptions {
        origin: cricut_gcode::Origin::TopLeft,
        ..GcodeOptions::default()
    };
    let material = MaterialSelection::Pressure(plan.pressure);
    let w = plan.workload;
    let warm_up = format!(
        "G21 G90\nG0 X{0:.3} Y{1:.3}\nM3\nG1 X{2:.3} Y{1:.3}\nM5\n",
        speed::SMALL_X_MM,
        speed::FIRST_ROW_MM,
        speed::SMALL_X_MM + 10.0
    );
    let mut jobs = vec![(
        "warm-up (not timed)".to_owned(),
        Job::from_gcode(&warm_up, &opts)?
            .material(material)
            .clamp(Clamp::A),
    )];
    println!(
        "  warm-up: 10 mm at {:.0} mm from the top edge",
        speed::FIRST_ROW_MM
    );
    if let Some(program) = &plan.program {
        println!(
            "speed test on a G-code file: pen in clamp A on paper, pressure {}; {} drawn moves ({} before simplifying), {:.0} mm of drawing, reaching ({:.0}, {:.0}) mm; each setting draws it once, over the last",
            plan.pressure,
            program.drawn_moves,
            program.source_moves,
            program.drawn_mm,
            program.max_x_mm,
            program.max_y_mm
        );
        for profile in &plan.profiles {
            println!("  {}", profile.label());
            let job = Job::from_gcode(&program.text, &opts)?
                .material(material)
                .clamp(Clamp::A)
                .speed(profile.speed())
                .wait_for_go(false);
            jobs.push((profile.label(), job));
        }
        return Ok(jobs);
    }
    println!(
        "speed test, {} workload: pen in clamp A on paper, pressure {}; each setting draws {} and {} {}s",
        w.name(),
        plan.pressure,
        plan.sizes.0,
        plan.sizes.1,
        w.unit()
    );
    for (i, profile) in plan.profiles.iter().enumerate() {
        let y = speed::band_top_mm(w, i);
        println!(
            "  band {} ({y:.0} to {:.0} mm from the top edge): {}",
            i + 1,
            y + w.band_mm(),
            profile.label()
        );
        for (x, size) in [
            (speed::SMALL_X_MM, plan.sizes.0),
            (speed::SMALL_X_MM + w.large_offset_mm(), plan.sizes.1),
        ] {
            let job = Job::from_gcode(&w.gcode(x, y, size), &opts)?
                .material(material)
                .clamp(Clamp::A)
                .speed(profile.speed())
                .wait_for_go(false);
            jobs.push((format!("{}, {size} {}(s)", profile.label(), w.unit()), job));
        }
    }
    Ok(jobs)
}

/// Prints the table of a G-code file run at several settings, each against the first.
fn program_report(plan: &SpeedPlan, times: &[Duration]) -> String {
    let mut csv = String::from("draw_in_s,travel_in_s,corner_b,corner_d,seconds,times_as_fast\n");
    println!("\nthe file at each setting, against the first:");
    println!("  {:<34} {:>9} {:>14}", "setting", "time", "x as fast");
    let first = times.get(1).copied();
    for (profile, time) in plan.profiles.iter().zip(times.iter().skip(1)) {
        let ratio = first.map_or(1.0, |f| speed::speedup(f, *time));
        println!(
            "  {:<34} {:>8.1}s {:>13.2}x",
            profile.label(),
            time.as_secs_f64(),
            ratio
        );
        let (b, d) = profile
            .corner_mix
            .map_or((String::new(), String::new()), |(b, d)| {
                (b.to_string(), d.to_string())
            });
        let _ = writeln!(
            csv,
            "{},{},{b},{d},{:.3},{ratio:.3}",
            profile.draw_in_s,
            profile.travel_in_s,
            time.as_secs_f64()
        );
    }
    csv
}

/// Prints the table of the `speed` stage and writes it as CSV; returns the file's path.
fn speed_report(plan: &SpeedPlan, times: &[Duration]) -> anyhow::Result<PathBuf> {
    let w = plan.workload;
    let (small, large) = plan.sizes;
    let csv = if plan.program.is_some() {
        program_report(plan, times)
    } else {
        let (small_mm, large_mm) = (w.length_mm(small), w.length_mm(large));
        let mut csv = format!(
            "workload,draw_in_s,travel_in_s,corner_b,corner_d,small_{0}s,large_{0}s,small_s,large_s,mm_per_s,overhead_s\n",
            w.unit()
        );
        println!(
            "\nspeed on the {} workload ({small_mm:.0} mm and {large_mm:.0} mm of path):",
            w.name()
        );
        println!(
            "  {:<34} {:>9} {:>9} {:>8} {:>10}",
            "setting", "small", "large", "mm/s", "overhead"
        );
        for (i, profile) in plan.profiles.iter().enumerate() {
            let (Some(first), Some(second)) = (times.get(1 + 2 * i), times.get(2 + 2 * i)) else {
                break;
            };
            let t = speed::timing(*first, *second, small_mm, large_mm);
            let cell = |v: Option<f64>| v.map_or("-".to_owned(), |v| format!("{v:.2}"));
            let (mm_s, overhead) = (
                cell(t.map(|t| t.mm_per_second)),
                cell(t.map(|t| t.overhead_seconds)),
            );
            println!(
                "  {:<34} {:>8.1}s {:>8.1}s {mm_s:>8} {overhead:>9}s",
                profile.label(),
                first.as_secs_f64(),
                second.as_secs_f64(),
            );
            let (b, d) = profile
                .corner_mix
                .map_or((String::new(), String::new()), |(b, d)| {
                    (b.to_string(), d.to_string())
                });
            let _ = writeln!(
                csv,
                "{},{},{},{b},{d},{small},{large},{:.3},{:.3},{mm_s},{overhead}",
                w.name(),
                profile.draw_in_s,
                profile.travel_in_s,
                first.as_secs_f64(),
                second.as_secs_f64(),
            );
        }
        csv
    };
    let secs = SystemTime::now().duration_since(UNIX_EPOCH)?.as_secs();
    let path = PathBuf::from(format!("target/harness/speed-results-{secs}.csv"));
    std::fs::write(&path, format!("# pacing: {}\n{csv}", plan.pacing))?;
    Ok(path)
}

/// Speed test: one kind of figure drawn at several speed settings, small and large, with the time
/// of every job measured. See `speed.rs` for how the numbers are derived.
///
/// Uses a pen in clamp A on paper, so nothing is cut. Every setting is tried in the order given
/// (the safest first), and the run stops at the first job that fails.
async fn speed_stage(port: &str, opts: &[String]) -> anyhow::Result<()> {
    let plan = speed_plan(opts)?;
    let jobs = speed_jobs(&plan)?;
    if plan.plan_only {
        println!("\n--plan: nothing was sent.");
        return Ok(());
    }
    println!(
        "\nThe machine waits for you to load the mat and press Go; nothing moves before that."
    );
    println!("Stop at any time with Ctrl+C (the head parks and the mat ejects).\n");
    let config = Config {
        poll_interval: Duration::from_millis(100),
        pacing: plan.pacing,
        ..Config::default()
    };
    println!("pacing: {}", plan.pacing);
    let times = run_jobs_with(port, jobs, config).await?;
    let path = speed_report(&plan, &times)?;
    println!("\nresults: {}", path.display());
    Ok(())
}

fn describe(ev: &Event) -> String {
    match ev {
        Event::JobStatusChanged(JobStatus::WaitingForMat) => {
            "status: WaitingForMat  >>> load the mat (press the Load button) <<<".into()
        }
        Event::JobStatusChanged(JobStatus::WaitingForGo) => {
            "status: WaitingForGo   >>> press Go when ready <<<".into()
        }
        Event::InterfaceChanged(i) => format!(
            "interface: lights={:02x} buttons={:02x} progress={} dial={}",
            i.lights, i.buttons, i.progress, i.dial
        ),
        Event::DialChanged(d) => format!("dial: {d}"),
        Event::MaterialSelected(m) => format!(
            "material: {} pressure={}",
            m.material.map_or("raw".into(), |m| m.to_string()),
            m.pressure
        ),
        Event::PositionChanged(p) => format!("position: ({:.3}, {:.3})", p.x, p.y),
        other => format!("{other:?}"),
    }
}

#[derive(Debug, Default)]
struct Checks {
    passed: usize,
    failed: usize,
}

impl Checks {
    fn pass(&mut self, name: &str, detail: impl std::fmt::Display) {
        self.passed += 1;
        println!("PASS  {name:<28} {detail}");
    }

    fn fail(&mut self, name: &str, detail: impl std::fmt::Display) {
        self.failed += 1;
        println!("FAIL  {name:<28} {detail}");
    }

    fn finish(self) -> anyhow::Result<()> {
        if self.failed > 0 {
            bail!("{} check(s) failed", self.failed);
        }
        println!("\nall {} checks passed", self.passed);
        Ok(())
    }
}

fn hex(b: &[u8]) -> String {
    let _ = protocol::hex; // keep the protocol module referenced for docs
    b.iter()
        .map(|x| format!("{x:02x}"))
        .collect::<Vec<_>>()
        .join(" ")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn plan(opts: &[&str]) -> anyhow::Result<SpeedPlan> {
        speed_plan(&opts.iter().map(|o| (*o).to_owned()).collect::<Vec<_>>())
    }

    #[test]
    fn the_default_plan_is_a_line_workload_that_fits_the_sheet() {
        let p = plan(&[]).expect("the defaults are valid");
        assert_eq!(p.workload, speed::Workload::Line);
        assert_eq!(p.profiles.len(), 5);
        assert_eq!(p.sizes, (1, 5));
        assert!(!p.plan_only);
    }

    #[test]
    fn every_workload_has_defaults_that_fit_the_sheet() {
        for w in ["line", "zigzag", "curve", "rounded"] {
            assert!(plan(&[&format!("--workload={w}")]).is_ok(), "{w}");
        }
        assert_eq!(plan(&["--workload=zigzag"]).unwrap().sizes, (10, 50));
        assert_eq!(plan(&["--workload=curve"]).unwrap().sizes, (1, 4));
    }

    #[test]
    fn too_many_settings_for_the_sheet_are_refused() {
        // A zigzag band is 42 mm: n settings reach 10 + 42 (n + 1) mm, so five fit under the 280 mm
        // limit (262) and six do not (304).
        let five = "--profiles=1:3,1.5:3,2:3,2.5:3,3:3";
        let six = "--profiles=1:3,1.5:3,2:3,2.5:3,3:3,3.5:3";
        assert!(plan(&["--workload=zigzag", five]).is_ok());
        assert!(plan(&["--workload=zigzag", six]).is_err());
        // Line bands are 15 mm: 17 settings reach 10 + 15 x 18 = 280 mm, 18 reach 295.
        let many = |n: usize| format!("--profiles={}", vec!["2:3"; n].join(","));
        assert!(plan(&[&many(17)]).is_ok());
        assert!(plan(&[&many(18)]).is_err());
    }

    #[test]
    fn the_speed_stage_never_draws_hard_enough_to_cut() {
        assert!(plan(&["--pressure=200"]).is_ok());
        assert!(plan(&["--pressure=201"]).is_err());
    }

    #[test]
    fn the_large_figure_must_be_larger_than_the_small_one_and_fit() {
        assert!(plan(&["--long=2"]).is_ok());
        assert!(plan(&["--long=9"]).is_ok());
        assert!(
            plan(&["--long=1"]).is_err(),
            "not larger than the small one"
        );
        assert!(
            plan(&["--long=10"]).is_err(),
            "too many passes for the band"
        );
        assert!(plan(&["--workload=zigzag", "--long=60"]).is_ok());
        assert!(plan(&["--workload=zigzag", "--long=61"]).is_err());
    }

    #[test]
    fn unknown_options_and_bad_settings_are_refused() {
        assert!(plan(&["--faster"]).is_err());
        assert!(plan(&["--workload=spiral"]).is_err());
        assert!(plan(&["--profiles=5:3"]).is_err());
        assert!(plan(&["--plan"]).expect("--plan is valid").plan_only);
    }

    #[test]
    fn the_jobs_are_a_warm_up_and_a_small_and_a_large_figure_per_setting() {
        let p = plan(&["--workload=curve"]).unwrap();
        let jobs = speed_jobs(&p).unwrap();
        assert_eq!(jobs.len(), 1 + 2 * p.profiles.len());
        assert!(jobs[0].0.contains("warm-up"));
    }
}

#[cfg(test)]
mod file_tests {
    use super::*;

    /// A tiny compiled program in a file of its own, for the options that read one.
    fn program_file(name: &str) -> String {
        let path = std::env::temp_dir().join(format!(
            "cricut-harness-{name}-{}.gcode",
            std::process::id()
        ));
        std::fs::write(
            &path,
            "G21\nG90\nM0 (Load pen)\nT1 M6\nG0 X10.000 Y20.000\nM3 S120\nG1 X30.000 Y20.000 F6000\nG1 X30.000 Y21.000 F6000\nM5\nM2\n",
        )
        .unwrap();
        path.display().to_string()
    }

    fn plan(opts: &[String]) -> anyhow::Result<SpeedPlan> {
        speed_plan(opts)
    }

    #[test]
    fn a_file_is_run_once_per_setting_after_the_warm_up() {
        let file = program_file("run");
        let p = plan(&[format!("--gcode={file}")]).expect("a valid file");
        let program = p.program.as_ref().expect("the file's program");
        assert_eq!(program.drawn_moves, 2);
        assert_eq!(p.profiles, speed::file_profiles());
        assert_eq!(speed_jobs(&p).unwrap().len(), 1 + p.profiles.len());
    }

    #[test]
    fn a_file_is_placed_ten_millimetres_in_from_the_corner() {
        let file = program_file("place");
        let p = plan(&[format!("--gcode={file}")]).unwrap();
        // The file's own corner (30, 21) plus the offset on both axes.
        let program = p.program.unwrap();
        assert!((program.max_x_mm - 40.0).abs() < 1e-9 && (program.max_y_mm - 31.0).abs() < 1e-9);
    }

    #[test]
    fn a_file_takes_no_workload_and_a_missing_file_is_an_error() {
        let file = program_file("conflict");
        assert!(plan(&[format!("--gcode={file}"), "--workload=curve".to_owned()]).is_err());
        assert!(plan(&[format!("--gcode={file}"), "--long=3".to_owned()]).is_err());
        assert!(plan(&["--gcode=/no/such/file.gcode".to_owned()]).is_err());
    }

    #[test]
    fn a_files_settings_can_be_chosen_and_are_still_checked() {
        let file = program_file("settings");
        let p = plan(&[format!("--gcode={file}"), "--profiles=3:3".to_owned()]).unwrap();
        assert_eq!(p.profiles.len(), 1);
        assert!(plan(&[format!("--gcode={file}"), "--profiles=9:3".to_owned()]).is_err());
    }
}

#[cfg(test)]
mod pacing_tests {
    use super::*;

    fn plan(opts: &[&str]) -> anyhow::Result<SpeedPlan> {
        speed_plan(&opts.iter().map(|o| (*o).to_owned()).collect::<Vec<_>>())
    }

    #[test]
    fn the_default_is_the_pacing_verified_on_hardware() {
        assert_eq!(plan(&[]).unwrap().pacing, Pacing::LEGACY);
    }

    #[test]
    fn quick_pacing_polls_every_forty_milliseconds_unless_told_otherwise() {
        assert_eq!(
            plan(&["--pacing=quick"]).unwrap().pacing,
            Pacing::quick(Duration::from_millis(40))
        );
        assert_eq!(
            plan(&["--pacing=quick:25"]).unwrap().pacing,
            Pacing::quick(Duration::from_millis(25))
        );
    }

    #[test]
    fn a_poll_outside_five_to_five_hundred_milliseconds_is_refused() {
        for bad in [
            "--pacing=quick:4",
            "--pacing=quick:501",
            "--pacing=quick:x",
            "--pacing=fast",
        ] {
            assert!(plan(&[bad]).is_err(), "{bad}");
        }
    }

    #[test]
    fn frames_of_one_to_thirteen_vectors_can_be_asked_for() {
        let p = plan(&["--pacing=quick", "--frame-entries=6"]).unwrap();
        assert_eq!(p.pacing.frame_entries(), 6);
        assert!(plan(&["--frame-entries=0"]).is_err());
        assert!(plan(&["--frame-entries=14"]).is_err());
    }
}

#[cfg(test)]
mod simplify_tests {
    use super::*;

    fn file(name: &str) -> String {
        let path = std::env::temp_dir().join(format!(
            "cricut-harness-{name}-{}.gcode",
            std::process::id()
        ));
        std::fs::write(
            &path,
            "G0 X10 Y10\nM3\nG1 X15 Y10\nG1 X20 Y10\nG1 X25 Y10\nG1 X25 Y15\nM5\n",
        )
        .unwrap();
        path.display().to_string()
    }

    fn plan(opts: &[String]) -> anyhow::Result<SpeedPlan> {
        speed_plan(opts)
    }

    #[test]
    fn a_file_is_simplified_only_when_asked() {
        let f = file("plain");
        let plain = plan(&[format!("--gcode={f}")]).unwrap().program.unwrap();
        assert_eq!(plain.drawn_moves, 4);
        let simple = plan(&[format!("--gcode={f}"), "--simplify=0.01".to_owned()])
            .unwrap()
            .program
            .unwrap();
        assert_eq!((simple.drawn_moves, simple.source_moves), (2, 4));
    }

    #[test]
    fn the_tolerance_must_be_between_a_micrometre_and_half_a_millimetre() {
        let f = file("range");
        for ok in ["0.001", "0.01", "0.5"] {
            assert!(
                plan(&[format!("--gcode={f}"), format!("--simplify={ok}")]).is_ok(),
                "{ok}"
            );
        }
        for bad in ["0", "0.0005", "0.6", "x"] {
            assert!(
                plan(&[format!("--gcode={f}"), format!("--simplify={bad}")]).is_err(),
                "{bad}"
            );
        }
    }
}

#[cfg(test)]
mod rounded_tests {
    use super::*;

    fn plan(opts: &[&str]) -> anyhow::Result<SpeedPlan> {
        speed_plan(&opts.iter().map(|o| (*o).to_owned()).collect::<Vec<_>>())
    }

    #[test]
    fn the_rounded_workload_has_six_chords_a_turn_unless_told_otherwise() {
        assert_eq!(
            plan(&["--workload=rounded"]).unwrap().workload,
            speed::Workload::Rounded(6)
        );
        assert_eq!(
            plan(&["--workload=rounded", "--turns=3"]).unwrap().workload,
            speed::Workload::Rounded(3)
        );
    }

    #[test]
    fn turns_are_two_to_twenty_four_and_only_for_the_rounded_workload() {
        assert!(plan(&["--workload=rounded", "--turns=2"]).is_ok());
        assert!(plan(&["--workload=rounded", "--turns=24"]).is_ok());
        assert!(plan(&["--workload=rounded", "--turns=1"]).is_err());
        assert!(plan(&["--workload=rounded", "--turns=25"]).is_err());
        assert!(plan(&["--workload=zigzag", "--turns=6"]).is_err());
        assert!(plan(&["--turns=6"]).is_err());
    }

    #[test]
    fn a_rounded_plan_fits_the_sheet_and_makes_the_same_jobs_as_the_zigzag() {
        let rounded = plan(&["--workload=rounded"]).unwrap();
        let zigzag = plan(&["--workload=zigzag"]).unwrap();
        assert_eq!(rounded.profiles, zigzag.profiles);
        assert_eq!(
            speed_jobs(&rounded).unwrap().len(),
            speed_jobs(&zigzag).unwrap().len()
        );
    }
}
