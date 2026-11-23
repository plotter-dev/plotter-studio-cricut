//! The Cricut Explore Air 2 device plugin, a WebAssembly component.
//!
//! The protocol, the AES session and the job sequence live in `cricut-gcode`'s blocking
//! [`engine`]; this crate only connects it to the host. It turns each step the
//! host sends into an engine [`Op`], hands the engine the host's connection, and drives it a little
//! at a time from `wait-event`, so no call runs long enough for the host to give up on it.
//!
//! The plugin never parses G-code: every line arrives already parsed.

use std::borrow::Borrow;
use std::cell::RefCell;
use std::collections::{HashMap, VecDeque};
use std::hash::{BuildHasher, RandomState};
use std::time::Duration;

use cricut_gcode::engine::{self, Activity, Engine, Op, Port, Step, Update};
use cricut_gcode::job::{Clamp, Speed};
use cricut_gcode::{Error, MachineType};

wit_bindgen::generate!({
    path: "../wit",
    world: "device-plugin",
});

use exports::plotter::device::driver::{Guest, GuestSession};
use plotter::device::host;
use plotter::device::transport::Connection;
use plotter::device::types::{
    Capabilities, DeviceError, DeviceInfo, Event, GcodeLine, JobHeader, MachineState, PowerState,
    Status, Step as GcodeStep, UserAction,
};

/// Cricut's USB vendor id, as the machine reports it (confirmed on an Explore Air 2).
const CRICUT_USB_VENDOR: u16 = 0x20D3;
/// The machine profile this plugin drives.
const PROFILE: &str = "cricut.explore-air-2";
/// How fast the carriage moves between shapes, in millimetres per second: Design Space's normal
/// travel speed (3.0 in/s). Every Cricut setting known to move the machine is a drawing speed;
/// this one stays put until it has been measured to matter. The profile's
/// `max_travel_speed_mm_s` says the same for the estimate.
const TRAVEL_MM_S: f64 = 76.2;
/// Lines the host may offer at once; the engine takes what fits and the host offers the rest again.
const BATCH_LINES: u32 = 256;
/// The longest the plugin sleeps when the engine has nothing to do.
const IDLE_NAP_MS: u64 = 20;
/// How long `wait-event` goes on starting engine steps. The host cuts a call off after two
/// seconds and one step can take over a second when the machine is slow to answer, so the rest
/// of the host's timeout is left to the next call.
const WAIT_BUDGET_MS: u64 = 500;

struct Component;

/// The host's connection as the engine's [`Port`].
struct HostPort<C: Borrow<Connection>> {
    conn: C,
    /// Keyed from the host's random source (std seeds it from `wasi:random`), for the handshake's
    /// nonce.
    noise: RandomState,
    counter: u64,
}

impl<C: Borrow<Connection>> HostPort<C> {
    fn new(conn: C) -> Self {
        Self {
            conn,
            noise: RandomState::new(),
            counter: 0,
        }
    }
}

fn io_error(message: &str) -> Error {
    Error::Io(std::io::Error::other(message.to_owned()))
}

impl<C: Borrow<Connection>> Port for HostPort<C> {
    fn write(&mut self, data: &[u8]) -> cricut_gcode::Result<()> {
        self.conn
            .borrow()
            .write(data)
            .map_err(|e| io_error(&e.message))
    }

    fn read(&mut self, max: usize, timeout: Duration) -> cricut_gcode::Result<Vec<u8>> {
        let max = u32::try_from(max).unwrap_or(u32::MAX);
        let timeout_ms = u32::try_from(timeout.as_millis()).unwrap_or(u32::MAX);
        self.conn
            .borrow()
            .read(max, timeout_ms)
            .map_err(|e| io_error(&e.message))
    }

    fn sleep(&mut self, duration: Duration) {
        host::sleep(u32::try_from(duration.as_millis()).unwrap_or(u32::MAX));
    }

    fn now(&mut self) -> Duration {
        Duration::from_millis(host::now_ms())
    }

    fn random(&mut self) -> u8 {
        self.counter = self.counter.wrapping_add(1);
        self.noise.hash_one(self.counter).to_le_bytes()[0]
    }
}

fn error(code: &str, message: impl Into<String>) -> DeviceError {
    DeviceError {
        code: code.to_owned(),
        message: message.into(),
        recoverable: false,
    }
}

/// The host's error for an engine error.
fn device_error(e: &Error) -> DeviceError {
    let code = match e {
        Error::Io(_) | Error::Disconnected => "io",
        Error::HandshakeFailed { .. } | Error::NoSession => "handshake",
        Error::MachineDropping { .. } => "machine-dropping",
        Error::OutOfBounds { .. } => "out-of-bounds",
        Error::Busy | Error::NoJob => "busy",
        _ => "cricut",
    };
    error(code, e.to_string())
}

fn device_info(transport: plotter::device::types::TransportKind) -> DeviceInfo {
    DeviceInfo {
        model: "Cricut Explore Air 2".to_owned(),
        serial_number: None,
        firmware: None,
        profile_id: Some(PROFILE.to_owned()),
        transport,
    }
}

/// The feed, in millimetres a minute, of a step that cuts.
fn feed_of(step: &GcodeStep) -> Option<u32> {
    match step {
        GcodeStep::Line(l) => Some(l.feed_mm_min),
        GcodeStep::Arc(a) => Some(a.feed_mm_min),
        _ => None,
    }
}

/// The speed a cut's feed asks of this machine: the feed is the drawing speed, held to what
/// Design Space has sent (4 in/s), and travel is [`TRAVEL_MM_S`].
fn speed_of(feed_mm_min: u32) -> Speed {
    Speed::from_mm_s(f64::from(feed_mm_min) / 60.0, TRAVEL_MM_S)
}

/// The engine op for a step, if the machine has a use for it.
fn op_of(step: &GcodeStep) -> Option<Op> {
    Some(match step {
        GcodeStep::Travel(p) => Op::Travel((p.x, p.y)),
        GcodeStep::Line(l) => Op::Line((l.to.x, l.to.y)),
        GcodeStep::Arc(a) => Op::Arc {
            to: (a.to.x, a.to.y),
            centre_offset: (a.centre_offset.x, a.centre_offset.y),
            ccw: a.ccw,
        },
        GcodeStep::ToolDown(force) => Op::ToolDown(*force),
        GcodeStep::ToolUp => Op::ToolUp,
        GcodeStep::ToolChange(_) => Op::ToolChange,
        GcodeStep::Pause(message) => Op::Pause(message.clone()),
        GcodeStep::End => Op::End,
        // The machine has no heater, and the dialect is always millimetres, absolute, as written
        // in the file. A cut's feed is its speed; `send` hands that to the engine separately.
        GcodeStep::Comment(_)
        | GcodeStep::Units(_)
        | GcodeStep::Absolute
        | GcodeStep::Temperature(_)
        | GcodeStep::Dwell(_) => return None,
    })
}

/// The clamp a holder of the profile is: `generic.holder-a` takes pens, `generic.holder-b` the
/// blade. These are the ids the generic tools list, so a pen from the host's built-in library fits
/// the machine. The ids older profiles used, `cricut.clamp-a` and `cricut.clamp-b`, still work.
/// `None` for a holder this plugin does not know, which is then cut as the blade.
fn clamp_of(holder: &str) -> Option<Clamp> {
    let holder = holder.to_ascii_lowercase();
    if holder.ends_with("holder-a") || holder.ends_with("clamp-a") {
        Some(Clamp::A)
    } else if holder.ends_with("holder-b") || holder.ends_with("clamp-b") {
        Some(Clamp::B)
    } else {
        None
    }
}

fn idle_status(state: MachineState) -> Status {
    Status {
        power: PowerState::On,
        state,
        progress: None,
        mat_loaded: None,
        holders: Vec::new(),
        position_mm: None,
        temperature_c: None,
        battery_percent: None,
        alerts: Vec::new(),
    }
}

struct State {
    /// `Err` holds why the session could not be opened.
    engine: Result<Engine<HostPort<Connection>>, DeviceError>,
    events: VecDeque<Event>,
    /// The latest prompt, shown as the machine's state until it is confirmed.
    prompt: Option<UserAction>,
    /// The clamp of each tool slot of the job, from the header's tool table.
    clamps: HashMap<u8, Clamp>,
    transport: plotter::device::types::TransportKind,
}

struct CricutSession {
    state: RefCell<State>,
}

impl State {
    fn engine(&mut self) -> Result<&mut Engine<HostPort<Connection>>, DeviceError> {
        self.engine.as_mut().map_err(|e| e.clone())
    }

    /// One engine step; the event it produces, if any. A failure stops the machine safely first.
    fn step(&mut self) -> Option<(bool, Option<Event>)> {
        let result = self.engine.as_mut().ok()?.step();
        match result {
            Ok(Step::Idle) => Some((false, None)),
            Ok(Step::Worked) => Some((true, None)),
            Ok(Step::Update(Update::Prompt { id, message })) => {
                let action = UserAction { id, message };
                self.prompt = Some(action.clone());
                Some((true, Some(Event::UserActionRequired(action))))
            }
            Ok(Step::Update(Update::Finished { .. })) => {
                self.prompt = None;
                Some((true, Some(Event::JobFinished)))
            }
            Ok(Step::Update(_)) => Some((true, None)),
            Err(e) => {
                self.prompt = None;
                if let Ok(engine) = &mut self.engine {
                    if engine.activity() == Activity::Unloading {
                        // The tidying up itself failed; trying it again would only fail again and
                        // fault again on every call.
                        engine.abandon();
                    } else {
                        // Flush the machine's queue and send the carriage home; never leave it
                        // mid-cut.
                        let _ = engine.cancel();
                    }
                }
                Some((true, Some(Event::Fault(device_error(&e)))))
            }
        }
    }
}

impl Guest for Component {
    type Session = CricutSession;

    /// Only ports that look like a Cricut are offered, so no other port is ever opened: Cricut's
    /// USB vendor id, or a product string that names it.
    fn discover() -> Vec<String> {
        plotter::device::discovery::list_serial_ports()
            .into_iter()
            .filter(|c| {
                c.usb.is_some_and(|(vendor, _)| vendor == CRICUT_USB_VENDOR)
                    || c.name.to_lowercase().contains("cricut")
            })
            .map(|c| c.id)
            .collect()
    }

    fn identify(conn: &Connection) -> Option<DeviceInfo> {
        let mut port = HostPort::new(conn);
        // Only a Cricut answers the status query with a reply that starts `60`.
        engine::probe(&mut port).ok()??;
        Some(device_info(conn.kind()))
    }
}

impl GuestSession for CricutSession {
    fn new(conn: Connection) -> Self {
        let transport = conn.kind();
        let engine = Engine::connect(HostPort::new(conn), MachineType::ExploreAir2)
            .map_err(|e| device_error(&e));
        Self {
            state: RefCell::new(State {
                engine,
                events: VecDeque::new(),
                prompt: None,
                clamps: HashMap::new(),
                transport,
            }),
        }
    }

    fn info(&self) -> DeviceInfo {
        device_info(self.state.borrow().transport)
    }

    fn capabilities(&self) -> Capabilities {
        Capabilities {
            // The machine queues about 80 vectors ahead and runs them; there is no way to hold
            // it, so pause is not offered. Cancel flushes the queue and parks the carriage.
            pause: false,
            resume: false,
            cancel: true,
            jog: false,
            home: false,
            load_mat: false,
            unload_mat: false,
            tool_detection: false,
            reports_position: false,
            reports_temperature: false,
            max_inflight_lines: BATCH_LINES,
        }
    }

    fn status(&self) -> Status {
        let mut s = self.state.borrow_mut();
        // A cancel finishes by parking the carriage; the host polls the status, so it moves on. A
        // failure is held for the next `wait-event` so the host hears of it.
        let stepped = matches!(
            s.engine.as_ref().map(Engine::activity),
            Ok(Activity::Unloading)
        );
        if stepped && let Some((_, Some(event))) = s.step() {
            s.events.push_back(event);
        }
        let prompt = s.prompt.clone();
        let engine = match s.engine() {
            Ok(engine) => engine,
            Err(e) => return idle_status(MachineState::Fault(e)),
        };
        let state = match engine.activity() {
            Activity::Idle => MachineState::Idle,
            Activity::AwaitingConfirmation | Activity::WaitingForMat | Activity::WaitingForGo => {
                prompt.map_or(MachineState::RunningJob, MachineState::WaitingForUser)
            }
            Activity::Preparing | Activity::Cutting | Activity::Finishing => {
                MachineState::RunningJob
            }
            _ => MachineState::Busy,
        };
        let mat_loaded = match engine.activity() {
            // Reading the interface costs a round trip; a call that has just stepped has had its share.
            Activity::Idle if !stepped => engine
                .interface_state()
                .ok()
                .flatten()
                .map(|i| i.mat_loaded()),
            _ => Some(engine.mat_in()),
        };
        Status {
            mat_loaded,
            ..idle_status(state)
        }
    }

    fn wait_event(&self, timeout_ms: u32) -> Option<Event> {
        let mut s = self.state.borrow_mut();
        if let Some(event) = s.events.pop_front() {
            return Some(event);
        }
        let start = host::now_ms();
        let limit = u64::from(timeout_ms).min(WAIT_BUDGET_MS);
        loop {
            let (worked, event) = s.step()?;
            if event.is_some() {
                return event;
            }
            let elapsed = host::now_ms().saturating_sub(start);
            if elapsed >= limit {
                return None;
            }
            if !worked {
                let nap = (limit - elapsed).min(IDLE_NAP_MS);
                host::sleep(u32::try_from(nap).unwrap_or(u32::MAX));
            }
        }
    }

    fn begin_job(&self, header: JobHeader) -> Result<(), DeviceError> {
        let mut s = self.state.borrow_mut();
        s.events.clear();
        s.prompt = None;
        // The machine has no command that selects a clamp; the way a job starts decides, so each
        // tool's clamp comes from where the compiler put it.
        s.clamps = header
            .tools
            .iter()
            .filter_map(|t| Some((t.slot, clamp_of(&t.holder)?)))
            .collect();
        s.engine()?
            .begin_job()
            .map(|_| ())
            .map_err(|e| device_error(&e))
    }

    fn send(&self, lines: Vec<GcodeLine>) -> Result<u32, DeviceError> {
        let mut s = self.state.borrow_mut();
        let clamps = s.clamps.clone();
        let engine = s.engine()?;
        let mut accepted = 0_u32;
        for line in &lines {
            // The speed is part of a section's setup; the engine starts a new section when it
            // changes after something was cut, and ignores it when it is the same.
            if let Some(feed) = feed_of(&line.step)
                && let Err(e) = engine.feed(&Op::Speed(speed_of(feed)))
            {
                return Err(device_error(&e));
            }
            if let Some(op) = op_of(&line.step) {
                match engine.feed(&op) {
                    Ok(true) => {}
                    Ok(false) => break,
                    Err(e) => return Err(device_error(&e)),
                }
            }
            // A tool change also tells the engine which clamp the tool is in. The engine always
            // takes a tool change, so the line is never half-accepted.
            if let GcodeStep::ToolChange(slot) = &line.step
                && let Some(clamp) = clamps.get(slot)
                && let Err(e) = engine.feed(&Op::Clamp(*clamp))
            {
                return Err(device_error(&e));
            }
            accepted += 1;
        }
        Ok(accepted)
    }

    fn end_job(&self) -> Result<(), DeviceError> {
        Ok(())
    }

    fn pause(&self) -> Result<(), DeviceError> {
        Err(error(
            "unsupported",
            "the Cricut cannot be paused once it has queued the cut",
        ))
    }

    fn resume(&self) -> Result<(), DeviceError> {
        Err(error("unsupported", "the Cricut cannot be paused"))
    }

    fn cancel(&self) -> Result<(), DeviceError> {
        let mut s = self.state.borrow_mut();
        s.events.clear();
        s.prompt = None;
        s.engine()?.cancel().map_err(|e| device_error(&e))
    }

    fn user_action_done(&self, id: u32) -> Result<(), DeviceError> {
        let mut s = self.state.borrow_mut();
        if s.engine()?.confirm(id) {
            s.prompt = None;
        }
        Ok(())
    }

    fn jog(&self, _dx_mm: f64, _dy_mm: f64) -> Result<(), DeviceError> {
        Err(error("unsupported", "the Cricut has no jog command"))
    }

    fn disconnect(&self) {
        if let Ok(engine) = self.state.borrow_mut().engine() {
            engine.disconnect();
        }
    }
}

export!(Component);
