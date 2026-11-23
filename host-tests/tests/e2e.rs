//! The real Cricut Explore Air 2 plugin, built for `wasm32-wasip2`, runs through the Plotter
//! Studio host against the simulated machine from `cricut-gcode`. The simulator decrypts and
//! checks every frame, so a job that finishes here was spoken in the machine's own protocol.
//!
//! These tests need a checkout of the host next to this repository; see `Cargo.toml`.

use std::path::PathBuf;
use std::process::Command;
use std::sync::{Arc, Mutex, Once};
use std::time::Duration;

use cricut_gcode::protocol::op;
use cricut_gcode::transport::{MockLink, SimulatedMachine};
use ps_device::*;
use tokio::sync::mpsc;

/// A fixed set of buses for plugin discovery tests.
#[derive(Default)]
struct FakeBuses {
    serial: Vec<DiscoveredDevice>,
}

impl Buses for FakeBuses {
    fn serial_ports(&self) -> Vec<DiscoveredDevice> {
        self.serial.clone()
    }
    fn usb_devices(&self) -> Vec<DiscoveredDevice> {
        Vec::new()
    }
    fn ble_devices(&self, _: Duration) -> Vec<DiscoveredDevice> {
        Vec::new()
    }
    fn mdns_services(&self, _: &str, _: Duration) -> Vec<DiscoveredDevice> {
        Vec::new()
    }
}

/// A serial port as the system lists it.
fn port(name: &str, usb: Option<(u16, u16)>, product: Option<&str>) -> DiscoveredDevice {
    DiscoveredDevice {
        id: DeviceId(format!("serial:{name}")),
        name: product.unwrap_or(name).to_owned(),
        transport: TransportKind::Serial,
        usb,
        address: Some(name.to_owned()),
        serial: None,
    }
}

/// The plugin component, built for `wasm32-wasip2` on first use.
fn plugin() -> Vec<u8> {
    static BUILD: Once = Once::new();
    BUILD.call_once(|| {
        let status = Command::new(env!("CARGO"))
            .args([
                "build",
                "--manifest-path",
                concat!(env!("CARGO_MANIFEST_DIR"), "/../Cargo.toml"),
                "-p",
                "cricut-plugin",
                "--target",
                "wasm32-wasip2",
                "--release",
            ])
            .status()
            .expect("cargo runs");
        assert!(status.success(), "building cricut-plugin for wasm32-wasip2");
    });
    let dir = std::env::var_os("CARGO_TARGET_DIR").map_or_else(
        || PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../target"),
        PathBuf::from,
    );
    std::fs::read(dir.join("wasm32-wasip2/release/cricut_plugin.wasm")).expect("the built plugin")
}

/// The simulated machine behind a blocking serial link. Like the real one it is lit by the Load
/// prompt and waits for Go; the transport plays the operator by loading the mat when asked and
/// pressing Go when the light blinks.
#[derive(Clone)]
struct Cricut {
    sim: SimulatedMachine,
    link: Arc<Mutex<MockLink>>,
}

impl Cricut {
    fn new() -> Self {
        let sim = SimulatedMachine::new();
        let link = Arc::new(Mutex::new(sim.link()));
        Self { sim, link }
    }
}

impl BlockingTransport for Cricut {
    fn kind(&self) -> TransportKind {
        TransportKind::Serial
    }

    fn write(&mut self, data: &[u8]) -> Result<(), String> {
        self.link
            .lock()
            .unwrap()
            .write_now(data)
            .map_err(|e| e.to_string())
    }

    fn read(&mut self, max: usize, timeout: Duration) -> Result<Vec<u8>, String> {
        let (prompt, go_blinks) = {
            let state = self.sim.state();
            (state.load_prompt, state.lights_cmd == Some((2, 8)))
        };
        if prompt {
            self.sim.load_mat();
        }
        if go_blinks {
            self.sim.press_go();
        }
        let bytes = self
            .link
            .lock()
            .unwrap()
            .read_now(max)
            .map_err(|e| e.to_string())?;
        if bytes.is_empty() {
            std::thread::sleep(timeout.min(Duration::from_millis(5)));
        }
        Ok(bytes)
    }
}

/// A transport that never answers.
#[derive(Default)]
struct Silent;

impl BlockingTransport for Silent {
    fn kind(&self) -> TransportKind {
        TransportKind::Serial
    }
    fn write(&mut self, _: &[u8]) -> Result<(), String> {
        Ok(())
    }
    fn read(&mut self, _: usize, timeout: Duration) -> Result<Vec<u8>, String> {
        std::thread::sleep(timeout.min(Duration::from_millis(5)));
        Ok(Vec::new())
    }
}

/// A program as the compiler writes it: a pause for the tool, a tool change, then a square with a
/// rounded corner.
const PROGRAM: &str = "G21\nG90\nM0 (Load blade: Fine-point blade into Clamp B)\nT1 M6\n\
G0 X25.400 Y25.400\nM3 S157\nG1 X50.800 Y25.400 F6000\nG1 X50.800 Y50.800\n\
G3 X25.400 Y76.200 I-25.400 J0.000 F6000\nG1 X25.400 Y25.400\nM5\nM2\n";

#[tokio::test]
async fn the_plugin_recognises_the_machine_by_its_status_reply() {
    let info = WasmHost::new(WasmLimits::default())
        .unwrap()
        .identify(&plugin(), Box::new(Cricut::new()))
        .await
        .expect("an Explore Air 2 is recognised");
    assert_eq!(info.model, "Cricut Explore Air 2");
    assert_eq!(info.profile_id.as_deref(), Some("cricut.explore-air-2"));
}

#[tokio::test]
async fn a_serial_device_that_is_silent_is_not_taken_for_a_cricut() {
    let info = WasmHost::new(WasmLimits::default())
        .unwrap()
        .identify(&plugin(), Box::new(Silent))
        .await;
    assert!(info.is_none(), "{info:?}");
}

#[tokio::test]
async fn discovery_offers_only_the_ports_that_look_like_the_machine() {
    let buses = ScopedBuses::new(
        Arc::new(FakeBuses {
            serial: vec![
                port("COM3", Some((0x20D3, 7)), Some("USB Serial Device")),
                port("COM4", Some((0x0403, 1)), Some("FT232R USB UART")),
                port("COM5", None, None),
                port("COM6", Some((0x1234, 1)), Some("Cricut Explore Air 2")),
            ],
        }),
        vec![DeviceFilter {
            transport: TransportKind::Serial,
            usb: None,
        }],
    );
    let found = WasmHost::new(WasmLimits::default())
        .unwrap()
        .discover(&plugin(), buses)
        .await
        .unwrap();
    let mut addresses: Vec<_> = found.iter().filter_map(|d| d.address.clone()).collect();
    addresses.sort();
    assert_eq!(addresses, ["COM3", "COM6"]);
}

/// Runs `program` through the host's runner and the plugin against a fresh simulated machine.
async fn run(program: &str, tools: Vec<(u8, String, String)>) -> (JobOutcome, SimulatedMachine) {
    let machine = Cricut::new();
    let sim = machine.sim.clone();
    let mut driver = WasmHost::new(WasmLimits::default())
        .unwrap()
        .open(&plugin(), Box::new(machine))
        .await
        .expect("the plugin connects");
    let (tx, mut rx) = mpsc::unbounded_channel();
    let mut observer = |event: RunEvent| {
        if let RunEvent::UserActionRequired(action) = &event {
            let _ = tx.send(Control::Ack(action.id));
        }
    };
    let header = JobHeader {
        job_id: "job".into(),
        mat_index: 1,
        profile_id: "cricut.explore-air-2".into(),
        tools,
    };
    let outcome = run_job(
        &mut driver,
        program,
        header,
        &mut rx,
        &mut observer,
        &RunConfig::default(),
    )
    .await
    .unwrap();
    (outcome, sim)
}

/// The first ten bytes of every `SetMachineState` payload the machine accepted: they say which
/// clamp the job started in (the pressure follows).
fn machine_state_prefixes(sim: &SimulatedMachine) -> Vec<Vec<u8>> {
    sim.state()
        .received
        .iter()
        .filter(|c| c.opcode == op::SET_MACHINE_STATE)
        .map(|c| c.payload[..10].to_vec())
        .collect()
}

#[tokio::test]
async fn a_whole_job_runs_through_the_runner_and_the_wasm_plugin() {
    let (outcome, sim) = run(PROGRAM, vec![]).await;
    assert!(
        matches!(outcome, JobOutcome::Completed { .. }),
        "{outcome:?}"
    );
    let opcodes: Vec<u8> = sim.state().received.iter().map(|c| c.opcode).collect();
    assert!(
        opcodes.contains(&op::MULTI_DRAW_VECTOR),
        "the job drew vectors: {opcodes:?}"
    );
    assert_eq!(sim.state().bad_frames, 0, "every frame was well formed");
}

#[tokio::test]
async fn a_tool_in_holder_b_starts_the_job_as_a_blade_job() {
    let tools = vec![(1, "cricut.fine-point-blade".into(), "generic.holder-b".into())];
    let (outcome, sim) = run(PROGRAM, tools).await;
    assert!(
        matches!(outcome, JobOutcome::Completed { .. }),
        "{outcome:?}"
    );
    let expected = cricut_gcode::job::machine_state_payload(cricut_gcode::job::Clamp::B, 0);
    assert_eq!(machine_state_prefixes(&sim), vec![expected[..10].to_vec()]);
}

#[tokio::test]
async fn a_tool_in_holder_a_starts_the_job_as_a_pen_job() {
    let program = PROGRAM.replace(
        "Load blade: Fine-point blade into Clamp B",
        "Load pen: Fine liner into Clamp A",
    );
    let tools = vec![(1, "pen.fineliner-black-0.4".into(), "generic.holder-a".into())];
    let (outcome, sim) = run(&program, tools).await;
    assert!(
        matches!(outcome, JobOutcome::Completed { .. }),
        "{outcome:?}"
    );
    let expected = cricut_gcode::job::machine_state_payload(cricut_gcode::job::Clamp::A, 0);
    assert_eq!(machine_state_prefixes(&sim), vec![expected[..10].to_vec()]);
}

/// Profiles written for earlier versions of the plugin name the clamps `cricut.clamp-a` and
/// `cricut.clamp-b`; jobs for them must keep starting as before.
#[tokio::test]
async fn the_earlier_clamp_ids_still_select_the_clamp() {
    let program = PROGRAM.replace(
        "Load blade: Fine-point blade into Clamp B",
        "Load pen: Fine liner into Clamp A",
    );
    let tools = vec![(1, "pen.fineliner-black-0.4".into(), "cricut.clamp-a".into())];
    let (outcome, sim) = run(&program, tools).await;
    assert!(
        matches!(outcome, JobOutcome::Completed { .. }),
        "{outcome:?}"
    );
    let expected = cricut_gcode::job::machine_state_payload(cricut_gcode::job::Clamp::A, 0);
    assert_eq!(machine_state_prefixes(&sim), vec![expected[..10].to_vec()]);
}
