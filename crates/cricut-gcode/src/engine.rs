//! A blocking, step-driven Explore Air 2 job engine with no async runtime.
//!
//! [`crate::Cricut`] runs a job as one long `async` function. A device plugin cannot: it is a
//! WebAssembly component whose host calls it a little at a time (feed some lines, wait for an
//! event, ask for the status) and gives up on a call that takes more than a few seconds. The
//! [`Engine`] is the same job cut into small steps that each finish quickly:
//!
//! - the program arrives as [`Op`]s through [`Engine::feed`], which refuses more when its buffer is
//!   full, so the host's cursor follows the cutting;
//! - [`Engine::step`] does one unit of work (one command, one frame, one poll) and says when the
//!   operator is needed ([`Update::Prompt`]) or the job is over;
//! - the bytes on the wire are the same as the async controller sends, in the same order. A test
//!   runs both against the simulated machine and compares them.
//!
//! The machine takes one blade pressure per cut, so a program is split into *sections* wherever
//! the tool or the pressure changes. Each section is its own cut with its own prompt, Go press,
//! setup and park, exactly as Design Space runs a job with several tools.

use std::collections::VecDeque;
use std::f64::consts::TAU;
use std::time::Duration;

use crate::error::{Error, Result};
use crate::gcode::Point;
use crate::job::{
    self, Clamp, EncCommand, Speed, VECTOR_LINE, VECTOR_MOVE, VECTORS_PER_FRAME,
    setup_commands_with, vector_entry,
};
use crate::pacing::Pacing;
use crate::protocol::{self, InterfaceState, MachineType, Reply, body_len, op};

const MS: fn(u64) -> Duration = Duration::from_millis;

/// Give up on a command the machine keeps dropping after this many attempts.
const MAX_ATTEMPTS: u32 = 300;
/// Give up on a command the machine keeps leaving unanswered after this many attempts. Far fewer
/// than [`MAX_ATTEMPTS`]: a NAK says the machine is alive and busy, silence says it may be gone.
const MAX_SILENT: u32 = 3;
/// Vector entries held before [`Engine::feed`] asks for a pause: eight frames.
const BUFFER_ENTRIES: usize = VECTORS_PER_FRAME * 8;
/// How long to wait for the carriage to reach its rest position at the end of a section.
const PARK_TIMEOUT: Duration = Duration::from_secs(180);
/// How long to wait for it after a cancel.
const UNLOAD_PARK_TIMEOUT: Duration = Duration::from_secs(15);
/// The longest [`Engine::disconnect`] waits before starting another step. A plugin call is cut off
/// after two seconds and one step, or the abort that comes first, can take about a second.
const DISCONNECT_LIMIT: Duration = Duration::from_millis(700);
/// How long the mat is given to settle in the rollers after it is loaded.
const MAT_SETTLE: Duration = Duration::from_millis(1000);
/// How long the machine is given to eject a mat after the command.
const EJECT_TIME: Duration = Duration::from_millis(3000);
/// Largest machine coordinate `(x, y)` in inches: a 12 x 24 in mat with the 0.25 in offset.
const MAT_EXTENT_IN: (f64, f64) = (12.25, 24.25);
/// The sheet's top-left corner on the mat, in inches.
const MAT_OFFSET_IN: f64 = 0.25;
const UM_PER_INCH: f64 = 25_400.0;
/// Largest gap between an arc and its chords, in µm.
const ARC_TOLERANCE_UM: f64 = 10.0;

/// What the engine needs from a link: a byte stream with timeouts, a clock and a little luck.
///
/// The device host provides all of these to a plugin; tests provide a simulated machine.
pub trait Port {
    /// Writes all of `data`.
    ///
    /// # Errors
    ///
    /// When the link is gone.
    fn write(&mut self, data: &[u8]) -> Result<()>;

    /// Reads up to `max` bytes, waiting at most `timeout` for the first; empty means none came.
    ///
    /// # Errors
    ///
    /// When the link is gone.
    fn read(&mut self, max: usize, timeout: Duration) -> Result<Vec<u8>>;

    /// Waits for `duration`.
    fn sleep(&mut self, duration: Duration);

    /// A monotonic clock: time since some fixed moment.
    fn now(&mut self) -> Duration;

    /// A random byte, for the `NewAES` handshake's nonce.
    fn random(&mut self) -> u8;
}

impl<T: Port + ?Sized> Port for &mut T {
    fn write(&mut self, data: &[u8]) -> Result<()> {
        (**self).write(data)
    }
    fn read(&mut self, max: usize, timeout: Duration) -> Result<Vec<u8>> {
        (**self).read(max, timeout)
    }
    fn sleep(&mut self, duration: Duration) {
        (**self).sleep(duration);
    }
    fn now(&mut self) -> Duration {
        (**self).now()
    }
    fn random(&mut self) -> u8 {
        (**self).random()
    }
}

/// One step of a `ps-gcode/1` program, as the engine needs it. Coordinates are integer
/// micrometres in the program's frame (top-left origin, +Y towards the user).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Op {
    /// `M0`: the program asks the operator to do something before it goes on.
    Pause(String),
    /// `T<n> M6`: another tool is wanted. It is taken to be the blade in clamp B until a
    /// [`Op::Clamp`] says otherwise.
    ToolChange,
    /// The clamp the current tool sits in, from the host's tool table. A pen in clamp A starts a
    /// job differently from the blade in clamp B; see [`Clamp`].
    Clamp(Clamp),
    /// How fast the carriage moves from here on, from the feed of the program's cuts. The speed
    /// is part of a section's setup, so naming a different one after something was cut begins a
    /// new section, as a different pressure does.
    Speed(Speed),
    /// `M3 S<force>`: lower the tool; the force is the blade pressure.
    ToolDown(u32),
    /// `M5`: raise the tool.
    ToolUp,
    /// `G0`: move with the tool up.
    Travel((i64, i64)),
    /// `G1`: move in a line, cutting if the tool is down.
    Line((i64, i64)),
    /// `G2`/`G3`: move along an arc, cutting if the tool is down.
    Arc {
        /// Where the arc ends.
        to: (i64, i64),
        /// The arc's centre relative to where it starts.
        centre_offset: (i64, i64),
        /// Counter-clockwise (`G3`).
        ccw: bool,
    },
    /// `M2`: the program is over.
    End,
}

/// What the engine is doing, for a status display.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum Activity {
    /// Nothing.
    Idle,
    /// A job is open and waiting for something to cut.
    Preparing,
    /// The operator has been asked to do something and has not confirmed.
    AwaitingConfirmation,
    /// The Load light is on and no mat is in yet.
    WaitingForMat,
    /// The Go light is blinking.
    WaitingForGo,
    /// Cutting.
    Cutting,
    /// Everything is sent; the carriage is finishing and parking.
    Finishing,
    /// Parking the carriage after a cancel.
    Unloading,
}

/// What [`Engine::step`] reports.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum Update {
    /// The operator is needed: do what it says, then [`Engine::confirm`] the prompt.
    Prompt {
        /// Pass this to [`Engine::confirm`].
        id: u32,
        /// What to tell the operator.
        message: String,
    },
    /// The whole program was cut. `parked` is whether the carriage was seen back at rest.
    Finished {
        /// The carriage reached its rest position.
        parked: bool,
    },
    /// A cancel or [`Engine::unload`] finished.
    Unloaded,
}

/// The result of one [`Engine::step`].
#[derive(Debug, Clone, PartialEq, Eq)]
#[must_use]
pub enum Step {
    /// Nothing to do until more input, or a confirmation, arrives.
    Idle,
    /// Something was done; step again.
    Worked,
    /// Something the caller should act on.
    Update(Update),
}

// ---- the blocking link ----

/// A machine link with protocol state: the blocking twin of [`crate::session::Session`].
struct Link<P: Port> {
    port: P,
    kek: [u8; 32],
    key: Option<[u8; 32]>,
    /// Received bytes, already un-stuffed.
    rx: Vec<u8>,
    /// A trailing 0x5E that may be the first half of an escape pair.
    pending_escape: bool,
}

impl<P: Port> Link<P> {
    fn new(port: P, machine: MachineType) -> Self {
        Self {
            port,
            kek: machine.kek(),
            key: None,
            rx: Vec::new(),
            pending_escape: false,
        }
    }

    fn read_chunk(&mut self, timeout: Duration) -> Result<usize> {
        let buf = self.port.read(512, timeout)?;
        if buf.is_empty() {
            return Ok(0);
        }
        let mut raw = Vec::with_capacity(buf.len() + 1);
        if std::mem::take(&mut self.pending_escape) {
            raw.push(0x5E);
        }
        raw.extend_from_slice(&buf);
        if raw.last() == Some(&0x5E) {
            raw.pop();
            self.pending_escape = true;
        }
        self.rx.extend(protocol::unescape(&raw));
        Ok(buf.len())
    }

    fn flush_pending_escape(&mut self) {
        if std::mem::take(&mut self.pending_escape) {
            self.rx.push(0x5E);
        }
    }

    /// Reads one reply frame, waiting up to `timeout`; `None` on timeout.
    fn read_reply(&mut self, timeout: Duration) -> Result<Option<Reply>> {
        let deadline = self.port.now() + timeout;
        loop {
            if self.rx.is_empty() && self.pending_escape {
                // A lone 0x5E is the whole buffer: it may be a header or half an escape pair.
                if self.read_chunk(MS(30))? == 0 {
                    self.flush_pending_escape();
                }
                continue;
            }
            if let Some(&header) = self.rx.first() {
                let need = 1 + body_len(header);
                if self.rx.len() + 1 == need && self.pending_escape {
                    // The held-back 0x5E is all that is missing. If no 0xA2 follows promptly it
                    // was a literal byte ending the frame.
                    if self.read_chunk(MS(30))? == 0 {
                        self.flush_pending_escape();
                    }
                    continue;
                }
                if self.rx.len() >= need {
                    let frame: Vec<u8> = self.rx.drain(..need).collect();
                    let mut body = frame[1..].to_vec();
                    if header & 0x80 != 0
                        && !body.is_empty()
                        && body.len().is_multiple_of(16)
                        && let Some(key) = &self.key
                    {
                        protocol::aes_decrypt(&mut body, key);
                    }
                    return Ok(Some(Reply { header, body }));
                }
            }
            let now = self.port.now();
            if now >= deadline {
                self.flush_pending_escape();
                return Ok(None);
            }
            self.read_chunk(deadline.saturating_sub(now))?;
        }
    }

    /// Discards any buffered or in-flight input.
    fn drain(&mut self) -> Result<()> {
        while self.read_chunk(MS(150))? > 0 {}
        self.rx.clear();
        self.pending_escape = false;
        Ok(())
    }

    fn write(&mut self, frame: &[u8]) -> Result<()> {
        // Request/reply is strict: anything still buffered is stale.
        self.rx.clear();
        self.pending_escape = false;
        self.port.write(frame)
    }

    fn query(&mut self, opcode: u8, payload: &[u8], timeout: Duration) -> Result<Option<Reply>> {
        let frame = protocol::plain_frame(opcode, payload)?;
        self.write(&frame)?;
        self.read_reply(timeout)
    }

    fn machine_status(&mut self) -> Result<Option<Vec<u8>>> {
        Ok(self
            .query(op::MACHINE_STATUS, &[0, 0, 0], MS(1000))?
            .map(|r| r.body))
    }

    fn interface_state(&mut self) -> Result<Option<InterfaceState>> {
        let reply = self.query(op::INTERFACE_STATE, &[], MS(1000))?;
        Ok(reply.and_then(|r| InterfaceState::parse(&r.body)))
    }

    fn set_lights(&mut self, a: u8, b: u8) -> Result<()> {
        self.query(op::SET_LIGHTS, &[a, b], MS(600))?;
        Ok(())
    }

    fn clear_buttons(&mut self) -> Result<()> {
        self.query(op::CLEAR_BUTTONS, &[0x0F], MS(600))?;
        Ok(())
    }

    /// Starts the eject. The reply is not waited for beyond a second; the caller gives the machine
    /// the rest of [`EJECT_TIME`] in short steps.
    fn unload_mat(&mut self) -> Result<()> {
        self.query(op::UNLOAD_MAT, &[0x03, 0, 0], MS(1000))?;
        Ok(())
    }

    fn abort(&mut self) -> Result<()> {
        self.query(op::ABORT, &[], MS(1000))?;
        Ok(())
    }

    fn position(&mut self) -> Result<Option<(f32, f32)>> {
        let reply = self.query(op::READ_POSITION, &[], MS(600))?;
        Ok(reply.and_then(|r| protocol::parse_position(&r.body)))
    }

    /// Establishes the AES session (key slot 7, as Design Space uses).
    fn handshake(&mut self) -> Result<()> {
        const ATTEMPTS: u32 = 3;
        const SLOT: u8 = 7;
        for _ in 0..ATTEMPTS {
            // Sync the machine's frame parser (and our buffer) with a cheap read-only query.
            let _ = self.query(op::MACHINE_STATUS, &[0, 0, 0], MS(800))?;
            self.drain()?;

            let seed: [u8; 3] = std::array::from_fn(|_| self.port.random());
            self.write(&protocol::plain_frame(op::RESET_AES, &seed)?)?;
            self.read_chunk(MS(600))?;
            for _ in 0..6 {
                if self.read_chunk(MS(150))? == 0 {
                    break;
                }
            }
            self.drain()?;

            let random: [u8; 47] = std::array::from_fn(|_| self.port.random());
            self.write(&protocol::new_aes_request(SLOT, &random))?;

            let deadline = self.port.now() + MS(2500);
            while self.port.now() < deadline {
                self.read_chunk(MS(400))?;
                let mut view = self.rx.clone();
                if self.pending_escape {
                    view.push(0x5E);
                }
                if let Some(key) = protocol::find_session_key(&view, &self.kek) {
                    self.key = Some(key);
                    self.rx.clear();
                    self.pending_escape = false;
                    return Ok(());
                }
            }
        }
        self.key = None;
        Err(Error::HandshakeFailed { attempts: ATTEMPTS })
    }

    /// Sends one encrypted command and waits for its decrypted reply; `None` on timeout.
    fn send_enc(&mut self, opcode: u8, payload: &[u8], timeout: Duration) -> Result<Option<Reply>> {
        let key = self.key.ok_or(Error::NoSession)?;
        let frame = protocol::enc_frame(opcode, payload, &key)?;
        self.write(&frame)?;
        self.read_reply(timeout)
    }
}

/// Asks whatever is on `port` whether it is an Explore Air 2: its status reply starts `0x60`.
///
/// Sends one harmless read-only query. `Ok(None)` when nothing answers like a Cricut.
///
/// # Errors
///
/// When the link is gone.
pub fn probe<P: Port>(port: P) -> Result<Option<Vec<u8>>> {
    let mut link = Link::new(port, MachineType::ExploreAir2);
    link.drain()?;
    Ok(link
        .machine_status()?
        .filter(|body| body.first() == Some(&op::MACHINE_STATUS)))
}

// ---- the program, cut into sections ----

/// What is wanted from a pass over one section's frames.
#[derive(Debug, Clone)]
enum Item {
    Cmd(EncCommand),
    Frame {
        body: Vec<u8>,
        /// Counted in [`Engine::frames_sent`]: the park frame is not.
        counts: bool,
    },
}

/// A run of the program cut at one pressure, between operator prompts.
#[derive(Debug, Default)]
struct Section {
    pressure: Option<u16>,
    /// The clamp the tool is in; the blade's, unless the program says otherwise.
    clamp: Clamp,
    /// How fast the carriage moves; Normal unless the program says otherwise.
    speed: Speed,
    /// What the program asked of the operator before this section.
    gates: Vec<String>,
    entries: VecDeque<[u8; 9]>,
    /// Entries ever added; a section with none has nothing to cut.
    pushed: usize,
    /// No more entries will be added.
    closed: bool,
}

#[derive(Debug)]
struct Watch {
    start: Duration,
    stable_since: Duration,
    last: Option<(f32, f32)>,
    limit: Duration,
    /// The machine does not report its position: give up at this time instead.
    blind_until: Option<Duration>,
}

#[derive(Debug)]
enum Phase {
    /// Decide what the next section needs.
    Next,
    AwaitConfirm,
    WaitMat,
    /// The mat is in; giving it a moment to settle.
    MatSettle,
    /// Lighting Go, in two steps (`0` clears the buttons, `1` lights it).
    EnterGo(u8),
    WaitGo,
    /// Go was pressed: the commands that precede the setup, one per step.
    Starting(u8),
    /// Sending the setup commands.
    Setup,
    Cut,
    /// Sending the end of the draw section and the park section.
    Closing,
    Parking(Watch),
    UnloadStart,
    UnloadClosing,
    UnloadParking(Watch),
    /// Turning the lights off (`0`), looking for a mat (`1`), sending the eject (`2`) and tidying
    /// up (`3`), one per step.
    UnloadFinish(u8),
    /// The machine is ejecting the mat.
    UnloadSettle,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Kind {
    Job,
    Unload { eject: bool },
}

/// Where the program's pen is, as the lines of it arrive.
#[derive(Debug, Default)]
struct Pen {
    at: (i64, i64),
    down: bool,
    /// The next cut starts with a blade-up move to `at`.
    need_move: bool,
}

#[derive(Debug)]
#[expect(
    clippy::struct_excessive_bools,
    reason = "independent flags of one small state machine; grouping them would hide that"
)]
struct Work {
    kind: Kind,
    phase: Phase,
    sections: VecDeque<Section>,
    pen: Pen,
    input_done: bool,
    needs_mat: bool,
    confirmed: bool,
    prompt_id: u32,
    queue: VecDeque<Item>,
    not_before: Duration,
    attempts: u32,
    /// Attempts in a row the machine did not answer at all.
    silent: u32,
    paced: bool,
    /// When a frame was first refused and not accepted since, for [`Pacing::give_up`].
    refused_since: Option<Duration>,
    fifo: Option<u8>,
    frames_sent: usize,
    /// The machine has been sent something for this work, so it needs parking if cancelled.
    touched: bool,
    done: bool,
}

impl Work {
    fn new(kind: Kind, phase: Phase) -> Self {
        Self {
            kind,
            phase,
            sections: VecDeque::new(),
            pen: Pen::default(),
            input_done: false,
            needs_mat: false,
            confirmed: false,
            prompt_id: 0,
            queue: VecDeque::new(),
            not_before: Duration::ZERO,
            attempts: 0,
            silent: 0,
            paced: false,
            refused_since: None,
            fifo: None,
            frames_sent: 0,
            touched: false,
            done: false,
        }
    }

    fn buffered(&self) -> usize {
        self.sections.iter().map(|s| s.entries.len()).sum()
    }

    /// Opens a section, unless the last one has nothing in it yet; then it only gains the gate.
    fn start_section(&mut self, gate: Option<String>) {
        let gate = gate.filter(|g| !g.trim().is_empty());
        match self.sections.back_mut() {
            Some(last) if last.pushed == 0 => last.gates.extend(gate),
            other => {
                if let Some(last) = other {
                    last.closed = true;
                }
                self.sections.push_back(Section {
                    gates: gate.into_iter().collect(),
                    ..Section::default()
                });
            }
        }
    }

    fn set_pressure(&mut self, pressure: u16) {
        if self.sections.is_empty() {
            self.start_section(None);
        }
        let last = self.sections.back_mut().expect("a section was just added");
        match last.pressure {
            Some(p) if p != pressure && last.pushed > 0 => {
                // One pressure per cut: the next stroke is a new section, in the same clamp.
                let clamp = last.clamp;
                self.start_section(None);
                if let Some(next) = self.sections.back_mut() {
                    next.pressure = Some(pressure);
                    next.clamp = clamp;
                }
            }
            _ => last.pressure = Some(pressure),
        }
    }

    /// The speed of what is cut next. The speed is part of the setup, so naming another one after
    /// something was already cut begins a new section, in the same clamp and at the same pressure.
    fn set_speed(&mut self, speed: Speed) {
        if self.sections.is_empty() {
            self.start_section(None);
        }
        let Some(last) = self.sections.back_mut() else {
            return;
        };
        if last.speed == speed {
            return;
        }
        if last.pushed == 0 {
            last.speed = speed;
            return;
        }
        let (clamp, pressure) = (last.clamp, last.pressure);
        self.start_section(None);
        self.carry_pen_down();
        if let Some(next) = self.sections.back_mut() {
            next.clamp = clamp;
            next.pressure = next.pressure.or(pressure);
            next.speed = speed;
        }
    }

    /// The clamp of what is cut next. A clamp is part of how a cut starts, so naming another one
    /// after something was already cut begins a new section.
    fn set_clamp(&mut self, clamp: Clamp) {
        if self.sections.is_empty() {
            self.start_section(None);
        }
        let Some(last) = self.sections.back_mut() else {
            return;
        };
        if last.clamp == clamp {
            return;
        }
        if last.pushed > 0 {
            self.start_section(None);
        }
        if let Some(next) = self.sections.back_mut() {
            next.clamp = clamp;
        }
    }

    #[expect(
        clippy::cast_precision_loss,
        reason = "micrometre coordinates on a mat are far below 2^53, where i64 -> f64 is exact"
    )]
    fn push_entry(&mut self, kind: u8, um: (i64, i64)) -> Result<()> {
        let x_in = um.0 as f64 / UM_PER_INCH + MAT_OFFSET_IN;
        let y_in = um.1 as f64 / UM_PER_INCH + MAT_OFFSET_IN;
        if !(0.0..=MAT_EXTENT_IN.0).contains(&x_in) || !(0.0..=MAT_EXTENT_IN.1).contains(&y_in) {
            return Err(Error::OutOfBounds { x_in, y_in });
        }
        if self.sections.is_empty() {
            self.start_section(None);
        }
        let section = self.sections.back_mut().expect("a section was just added");
        section
            .entries
            .push_back(vector_entry(kind, Point::new(x_in, y_in)));
        section.pushed += 1;
        Ok(())
    }

    fn cut_to(&mut self, to: (i64, i64)) -> Result<()> {
        if std::mem::take(&mut self.pen.need_move) {
            self.push_entry(VECTOR_MOVE, self.pen.at)?;
        }
        self.push_entry(VECTOR_LINE, to)
    }

    /// A new section opened while the pen is down (a pause or tool change with no `M5` before it)
    /// goes on cutting at the pressure it was cutting at, and starts with a blade-up move to the
    /// pen. Without that, its first stroke would have neither pressure nor a starting point.
    fn carry_pen_down(&mut self) {
        if !self.pen.down {
            return;
        }
        self.pen.need_move = true;
        let n = self.sections.len();
        let inherited = n
            .checked_sub(2)
            .and_then(|i| self.sections.get(i))
            .and_then(|s| s.pressure);
        if let Some(next) = self.sections.back_mut()
            && next.pressure.is_none()
        {
            next.pressure = inherited;
        }
    }

    /// Takes one op; false when the buffer is full and the op was not taken.
    fn feed(&mut self, op: &Op) -> Result<bool> {
        if self.input_done {
            return Ok(true);
        }
        match op {
            Op::Pause(message) => {
                self.start_section(Some(message.clone()));
                self.carry_pen_down();
            }
            Op::ToolChange => {
                self.start_section(None);
                // A tool that names no clamp is the blade; a `Clamp` op may follow.
                if let Some(next) = self.sections.back_mut() {
                    next.clamp = Clamp::default();
                }
                self.carry_pen_down();
            }
            Op::Clamp(clamp) => self.set_clamp(*clamp),
            Op::Speed(speed) => self.set_speed(*speed),
            Op::ToolDown(force) => {
                self.set_pressure(u16::try_from(*force).unwrap_or(u16::MAX));
                self.pen.down = true;
                self.pen.need_move = true;
            }
            Op::ToolUp => self.pen.down = false,
            Op::Travel(to) => {
                self.pen.at = *to;
                self.pen.need_move = true;
            }
            Op::Line(to) => {
                if self.pen.down {
                    if self.buffered() >= BUFFER_ENTRIES {
                        return Ok(false);
                    }
                    self.cut_to(*to)?;
                }
                self.pen.at = *to;
            }
            Op::Arc {
                to,
                centre_offset,
                ccw,
            } => {
                if self.pen.down {
                    if self.buffered() >= BUFFER_ENTRIES {
                        return Ok(false);
                    }
                    for point in arc_points(self.pen.at, *to, *centre_offset, *ccw) {
                        self.cut_to(point)?;
                    }
                }
                self.pen.at = *to;
            }
            Op::End => {
                self.input_done = true;
                if let Some(last) = self.sections.back_mut() {
                    last.closed = true;
                }
            }
        }
        Ok(true)
    }
}

/// The chords that replace an arc, ending exactly at `to`. A degenerate arc is one line.
#[expect(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    reason = "micrometre coordinates are exact in f64, and a chord count is small and positive"
)]
fn arc_points(from: (i64, i64), to: (i64, i64), offset: (i64, i64), ccw: bool) -> Vec<(i64, i64)> {
    let centre = (
        from.0 as f64 + offset.0 as f64,
        from.1 as f64 + offset.1 as f64,
    );
    let (fx, fy) = (from.0 as f64 - centre.0, from.1 as f64 - centre.1);
    let radius = fx.hypot(fy);
    if radius < 1.0 {
        return vec![to];
    }
    let start = fy.atan2(fx);
    let end = (to.1 as f64 - centre.1).atan2(to.0 as f64 - centre.0);
    let mut sweep = if ccw { end - start } else { start - end }.rem_euclid(TAU);
    if from == to || sweep == 0.0 {
        sweep = TAU;
    }
    // A chord subtending `step` has sagitta r (1 - cos(step / 2)).
    let ratio = (1.0 - ARC_TOLERANCE_UM / radius).clamp(-1.0, 1.0);
    let chord_angle = 2.0 * ratio.acos();
    let chords = if chord_angle <= 0.0 {
        1
    } else {
        ((sweep / chord_angle).ceil() as usize).max(1)
    };
    let mut points = Vec::with_capacity(chords);
    for i in 1..chords {
        let t = i as f64 / chords as f64;
        let angle = if ccw {
            start + sweep * t
        } else {
            start - sweep * t
        };
        points.push((
            (centre.0 + radius * angle.cos()).round() as i64,
            (centre.1 + radius * angle.sin()).round() as i64,
        ));
    }
    points.push(to);
    points
}

// ---- the engine ----

/// An open link to a machine and the job running on it.
pub struct Engine<P: Port> {
    link: Link<P>,
    /// How frames are offered to the machine's FIFO; see [`Pacing`].
    pacing: Pacing,
    raw_status: Vec<u8>,
    /// A mat was loaded in a job and has not been ejected.
    mat_in: bool,
    /// The last section ended with the carriage confirmed at its rest position.
    parked: bool,
    work: Option<Work>,
}

/// Deliberately omits the key material so it never reaches a log.
impl<P: Port> std::fmt::Debug for Engine<P> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Engine")
            .field("has_session_key", &self.link.key.is_some())
            .field("mat_in", &self.mat_in)
            .field("activity", &self.activity())
            .finish_non_exhaustive()
    }
}

impl<P: Port> Engine<P> {
    /// Opens a session: reads the machine's status and performs the `NewAES` handshake.
    ///
    /// # Errors
    ///
    /// [`Error::HandshakeFailed`] when the machine does not answer as an Explore Air 2 does, or a
    /// link error.
    pub fn connect(port: P, machine: MachineType) -> Result<Self> {
        let mut link = Link::new(port, machine);
        link.drain()?;
        let raw_status = link.machine_status()?.unwrap_or_default();
        link.handshake()?;
        Ok(Self {
            link,
            pacing: Pacing::default(),
            raw_status,
            mat_in: false,
            parked: false,
            work: None,
        })
    }

    /// The machine's reply to its status query when the session was opened.
    #[must_use]
    pub fn raw_status(&self) -> &[u8] {
        &self.raw_status
    }

    /// What the engine is doing now.
    #[must_use]
    pub fn activity(&self) -> Activity {
        let Some(work) = &self.work else {
            return Activity::Idle;
        };
        match work.phase {
            Phase::Next => Activity::Preparing,
            Phase::AwaitConfirm => Activity::AwaitingConfirmation,
            Phase::WaitMat | Phase::MatSettle | Phase::EnterGo(_) => Activity::WaitingForMat,
            Phase::WaitGo => Activity::WaitingForGo,
            Phase::Starting(_) | Phase::Setup | Phase::Cut | Phase::Closing => Activity::Cutting,
            Phase::Parking(_) => Activity::Finishing,
            Phase::UnloadStart
            | Phase::UnloadClosing
            | Phase::UnloadParking(_)
            | Phase::UnloadFinish(_)
            | Phase::UnloadSettle => Activity::Unloading,
        }
    }

    /// Vector frames sent for the job so far.
    #[must_use]
    pub fn frames_sent(&self) -> usize {
        self.work.as_ref().map_or(0, |w| w.frames_sent)
    }

    /// Whether a mat is in the machine, as far as the engine knows.
    #[must_use]
    pub fn mat_in(&self) -> bool {
        self.mat_in
    }

    /// Reads the lights, buttons and dial.
    ///
    /// # Errors
    ///
    /// When the link is gone.
    pub fn interface_state(&mut self) -> Result<Option<InterfaceState>> {
        self.link.interface_state()
    }

    /// Reads the carriage position in machine inches.
    ///
    /// # Errors
    ///
    /// When the link is gone.
    pub fn position(&mut self) -> Result<Option<(f32, f32)>> {
        self.link.position()
    }

    /// Chooses how frames are offered to the machine's FIFO; takes effect from the next frame.
    /// [`Pacing::LEGACY`], the default, is the one verified on hardware.
    pub fn set_pacing(&mut self, pacing: Pacing) {
        self.pacing = pacing;
    }

    /// Opens a job. Returns whether the operator will be asked to load a mat: false when one is
    /// already in.
    ///
    /// # Errors
    ///
    /// [`Error::Busy`] when a job or an unload is under way, or a link error.
    pub fn begin_job(&mut self) -> Result<bool> {
        if self.work.is_some() {
            return Err(Error::Busy);
        }
        // Bit 0x10 is the "mat loaded" *light*, not a sensor: lighting the Load prompt clears it,
        // and pressing Load with a mat in ejects it. So when a mat is known to be in, the Load
        // prompt is skipped entirely.
        let seen = self.link.interface_state()?.is_some_and(|i| i.mat_loaded());
        let needs_mat = !(self.mat_in || seen);
        if needs_mat {
            self.link.clear_buttons()?;
            self.link.set_lights(0x46, 0x20)?; // Load
        }
        let mut work = Work::new(Kind::Job, Phase::Next);
        work.needs_mat = needs_mat;
        self.work = Some(work);
        Ok(needs_mat)
    }

    /// Gives the job the next op; false when its buffer is full and the op must be offered again.
    ///
    /// # Errors
    ///
    /// [`Error::NoJob`] with no job open, or [`Error::OutOfBounds`] for a point off the mat.
    pub fn feed(&mut self, op: &Op) -> Result<bool> {
        match &mut self.work {
            Some(work) if work.kind == Kind::Job => work.feed(op),
            _ => Err(Error::NoJob),
        }
    }

    /// The operator did what a [`Update::Prompt`] asked. False when `id` is not the open prompt.
    pub fn confirm(&mut self, id: u32) -> bool {
        match &mut self.work {
            Some(work) if matches!(work.phase, Phase::AwaitConfirm) && work.prompt_id == id => {
                work.confirmed = true;
                true
            }
            _ => false,
        }
    }

    /// Stops the job: flushes the machine's queue, then parks the carriage (see [`Engine::step`]
    /// for the finish, [`Update::Unloaded`]). The mat is left in.
    ///
    /// # Errors
    ///
    /// When the link is gone. The parking is installed first, so a failed abort still leaves the
    /// engine parking the carriage on later steps instead of forgetting the job.
    pub fn cancel(&mut self) -> Result<()> {
        let Some(old) = self.work.take() else {
            return Ok(());
        };
        let mut work = Work::new(Kind::Unload { eject: false }, Phase::UnloadStart);
        work.touched = old.touched || matches!(old.kind, Kind::Unload { .. });
        self.parked = false;
        self.work = Some(work);
        self.link.abort()
    }

    /// Gives up on whatever is under way without touching the machine, for when the link has
    /// failed even the tidying up that [`Engine::cancel`] set going. The carriage is then in an
    /// unknown place, and the mat is taken to be in.
    pub fn abandon(&mut self) {
        self.work = None;
        self.parked = false;
    }

    /// Parks the carriage, turns the lights off and ejects the mat, step by step.
    ///
    /// # Errors
    ///
    /// [`Error::Busy`] when a job is under way.
    pub fn unload(&mut self) -> Result<()> {
        if self.work.is_some() {
            return Err(Error::Busy);
        }
        let mut work = Work::new(Kind::Unload { eject: true }, Phase::UnloadStart);
        work.touched = true;
        self.work = Some(work);
        Ok(())
    }

    /// Stops anything under way and tidies up, for at most a few seconds. Call before dropping the
    /// link.
    pub fn disconnect(&mut self) {
        let start = self.link.port.now();
        let _ = self.cancel();
        while self.work.is_some() && self.link.port.now().saturating_sub(start) < DISCONNECT_LIMIT {
            match self.step() {
                Ok(Step::Idle | Step::Worked | Step::Update(_)) => {}
                Err(_) => break,
            }
        }
        self.work = None;
    }

    /// Does one unit of work: one command, one frame, one poll. Each call returns within a few
    /// seconds of the machine's replies.
    ///
    /// # Errors
    ///
    /// A link error, or a machine that keeps dropping a command. The job stays open so that
    /// [`Engine::cancel`] can flush the machine.
    pub fn step(&mut self) -> Result<Step> {
        let Some(mut work) = self.work.take() else {
            return Ok(Step::Idle);
        };
        let result = self.advance(&mut work);
        if !work.done {
            self.work = Some(work);
        }
        result
    }

    fn advance(&mut self, w: &mut Work) -> Result<Step> {
        match &mut w.phase {
            Phase::Next => self.next_section(w),
            Phase::AwaitConfirm => {
                if !w.confirmed {
                    return Ok(Step::Idle);
                }
                w.confirmed = false;
                w.phase = if w.needs_mat {
                    Phase::WaitMat
                } else {
                    Phase::EnterGo(0)
                };
                Ok(Step::Worked)
            }
            Phase::WaitMat => {
                if self.link.interface_state()?.is_some_and(|i| i.mat_loaded()) {
                    w.needs_mat = false;
                    w.not_before = self.link.port.now() + MAT_SETTLE;
                    w.phase = Phase::MatSettle;
                } else {
                    self.link.port.sleep(MS(150));
                }
                Ok(Step::Worked)
            }
            Phase::MatSettle => {
                let now = self.link.port.now();
                if now < w.not_before {
                    self.link
                        .port
                        .sleep(w.not_before.saturating_sub(now).min(MS(250)));
                } else {
                    w.phase = Phase::EnterGo(0);
                }
                Ok(Step::Worked)
            }
            Phase::EnterGo(stage) => {
                let stage = *stage;
                self.enter_go(w, stage)?;
                Ok(Step::Worked)
            }
            Phase::WaitGo => {
                if self.link.interface_state()?.is_some_and(|i| i.go_pressed()) {
                    w.phase = Phase::Starting(0);
                } else {
                    self.link.port.sleep(MS(150));
                }
                Ok(Step::Worked)
            }
            Phase::Starting(stage) => {
                let stage = *stage;
                self.start_cutting(w, stage)?;
                Ok(Step::Worked)
            }
            Phase::Setup => {
                if w.queue.is_empty() {
                    w.phase = Phase::Cut;
                    return Ok(Step::Worked);
                }
                self.pump(w)
            }
            Phase::Cut => self.cut(w),
            Phase::Closing => {
                if w.queue.is_empty() {
                    w.phase = Phase::Parking(self.watch(PARK_TIMEOUT));
                    return Ok(Step::Worked);
                }
                self.pump(w)
            }
            Phase::Parking(watch) => {
                // The watch stays in the phase, so a failed poll leaves it as it was.
                if let Some(parked) = self.watch_step(watch)? {
                    self.parked = parked;
                    w.sections.pop_front();
                    w.phase = Phase::Next;
                }
                Ok(Step::Worked)
            }
            Phase::UnloadStart
            | Phase::UnloadClosing
            | Phase::UnloadParking(_)
            | Phase::UnloadFinish(_)
            | Phase::UnloadSettle => self.advance_unload(w),
        }
    }

    /// The steps of parking the carriage and tidying up after a cancel or an unload.
    fn advance_unload(&mut self, w: &mut Work) -> Result<Step> {
        match &mut w.phase {
            Phase::UnloadStart => {
                if !w.touched || self.parked || self.link.key.is_none() {
                    w.phase = Phase::UnloadFinish(0);
                    return Ok(Step::Worked);
                }
                w.queue.extend([
                    Item::Cmd(EncCommand::begin_draw()),
                    Item::Frame {
                        body: job::park_frame(),
                        counts: false,
                    },
                    Item::Cmd(EncCommand::end_draw()),
                ]);
                w.phase = Phase::UnloadClosing;
                Ok(Step::Worked)
            }
            Phase::UnloadClosing => {
                if w.queue.is_empty() {
                    w.phase = Phase::UnloadParking(self.watch(UNLOAD_PARK_TIMEOUT));
                    return Ok(Step::Worked);
                }
                self.pump(w)
            }
            Phase::UnloadParking(watch) => {
                if let Some(parked) = self.watch_step(watch)? {
                    self.link.abort()?;
                    self.parked = parked;
                    w.phase = Phase::UnloadFinish(0);
                }
                Ok(Step::Worked)
            }
            Phase::UnloadFinish(stage) => {
                let stage = *stage;
                self.finish_unload(w, stage)
            }
            Phase::UnloadSettle => {
                let now = self.link.port.now();
                if now < w.not_before {
                    self.link
                        .port
                        .sleep(w.not_before.saturating_sub(now).min(MS(250)));
                    return Ok(Step::Worked);
                }
                self.finish_unload(w, 3)
            }
            _ => Ok(Step::Idle),
        }
    }

    /// The last of an unload, one machine exchange per step so none runs long.
    fn finish_unload(&mut self, w: &mut Work, stage: u8) -> Result<Step> {
        match stage {
            0 => {
                self.link.set_lights(0x46, 0x00)?;
                w.phase = if matches!(w.kind, Kind::Unload { eject: true }) {
                    Phase::UnloadFinish(1)
                } else {
                    Phase::UnloadFinish(3)
                };
                Ok(Step::Worked)
            }
            1 => {
                let in_machine =
                    self.mat_in || self.link.interface_state()?.is_some_and(|i| i.mat_loaded());
                w.phase = Phase::UnloadFinish(if in_machine { 2 } else { 3 });
                Ok(Step::Worked)
            }
            2 => {
                let sent = self.link.port.now();
                self.link.unload_mat()?;
                w.not_before = sent + EJECT_TIME;
                w.phase = Phase::UnloadSettle;
                Ok(Step::Worked)
            }
            _ => {
                if matches!(w.kind, Kind::Unload { eject: true }) {
                    self.mat_in = false;
                    self.parked = false;
                }
                w.done = true;
                Ok(Step::Update(Update::Unloaded))
            }
        }
    }

    fn watch(&mut self, limit: Duration) -> Watch {
        let now = self.link.port.now();
        Watch {
            start: now,
            stable_since: now,
            last: None,
            limit,
            blind_until: None,
        }
    }

    /// Decides what the next section needs: a prompt, or nothing if the job is over.
    fn next_section(&mut self, w: &mut Work) -> Result<Step> {
        loop {
            match w.sections.front() {
                None => {
                    if w.input_done {
                        w.done = true;
                        return Ok(Step::Update(Update::Finished {
                            parked: self.parked,
                        }));
                    }
                    return Ok(Step::Idle);
                }
                Some(s) if s.pushed == 0 && !s.closed && !w.input_done => return Ok(Step::Idle),
                Some(s) if s.pushed == 0 => {
                    w.sections.pop_front();
                }
                Some(s) => {
                    let mut parts: Vec<String> = Vec::new();
                    if w.needs_mat {
                        parts.push("Load the mat: put it in the machine and press Load.".into());
                    }
                    parts.extend(s.gates.iter().cloned());
                    parts.push(
                        "Then confirm here and press the blinking Go button on the machine to \
                         start cutting."
                            .into(),
                    );
                    w.prompt_id += 1;
                    w.confirmed = false;
                    w.phase = Phase::AwaitConfirm;
                    return Ok(Step::Update(Update::Prompt {
                        id: w.prompt_id,
                        message: parts.join("\n"),
                    }));
                }
            }
        }
    }

    /// The mat is in: light Go and wait for the operator to press it, one exchange per step.
    fn enter_go(&mut self, w: &mut Work, stage: u8) -> Result<()> {
        if stage == 0 {
            self.mat_in = true;
            self.link.clear_buttons()?;
            w.phase = Phase::EnterGo(1);
        } else {
            self.link.set_lights(0x02, 0x08)?; // Go blinking
            w.phase = Phase::WaitGo;
        }
        Ok(())
    }

    /// Go was pressed: the setup sequence Design Space sends before any vectors, one exchange per
    /// step so none runs long.
    fn start_cutting(&mut self, w: &mut Work, stage: u8) -> Result<()> {
        match stage {
            0 => {
                // Refuse before touching the machine.
                if w.sections.front().and_then(|s| s.pressure).is_none() {
                    return Err(Error::Gcode {
                        line: 0,
                        message: "the program cuts without setting a pressure (M3 S...)".into(),
                    });
                }
                self.link.clear_buttons()?;
                w.phase = Phase::Starting(1);
            }
            1 => {
                self.link.set_lights(0x02, 0x04)?; // Go solid
                w.phase = Phase::Starting(2);
            }
            2 => {
                self.link.abort()?; // reset the cut state before drawing, as Design Space does
                w.phase = Phase::Starting(3);
            }
            _ => {
                self.link.set_lights(0x01, 0x01)?;
                let front = w.sections.front();
                let clamp = front.map(|s| s.clamp).unwrap_or_default();
                let speed = front.map(|s| s.speed).unwrap_or_default();
                let pressure = front.and_then(|s| s.pressure).unwrap_or_default();
                self.parked = false;
                w.touched = true;
                w.fifo = None;
                w.queue.extend(
                    setup_commands_with(clamp, speed, pressure)
                        .into_iter()
                        .map(Item::Cmd),
                );
                w.phase = Phase::Setup;
            }
        }
        Ok(())
    }

    /// Cutting: frames of 13 vectors as they fill, then the end of the section.
    fn cut(&mut self, w: &mut Work) -> Result<Step> {
        if !w.queue.is_empty() {
            return self.pump(w);
        }
        let Some(section) = w.sections.front_mut() else {
            w.phase = Phase::Next;
            return Ok(Step::Worked);
        };
        let per_frame = self.pacing.frame_entries();
        if section.entries.len() >= per_frame || (section.closed && !section.entries.is_empty()) {
            let take = section.entries.len().min(per_frame);
            let body: Vec<u8> = section.entries.drain(..take).flatten().collect();
            w.queue.push_back(Item::Frame { body, counts: true });
            return Ok(Step::Worked);
        }
        if section.closed {
            // Park in its own draw section, as Design Space ends every job. Without it the head
            // stays at the last cut point and repeated jobs creep toward the end stop.
            w.queue.extend([
                Item::Cmd(EncCommand::end_draw()),
                Item::Cmd(EncCommand::begin_draw()),
                Item::Frame {
                    body: job::park_frame(),
                    counts: false,
                },
                Item::Cmd(EncCommand::end_draw()),
            ]);
            w.phase = Phase::Closing;
            return Ok(Step::Worked);
        }
        Ok(Step::Idle)
    }

    /// One attempt at the command or frame at the front of the queue.
    fn pump(&mut self, w: &mut Work) -> Result<Step> {
        let now = self.link.port.now();
        if now < w.not_before {
            self.link
                .port
                .sleep(w.not_before.saturating_sub(now).min(MS(250)));
            return Ok(Step::Worked);
        }
        let Some(item) = w.queue.front().cloned() else {
            return Ok(Step::Idle);
        };
        match item {
            Item::Cmd(cmd) => {
                let reply = self
                    .link
                    .send_enc(cmd.opcode, &cmd.payload, cmd.reply_timeout)?;
                let dropped = reply.as_ref().is_some_and(|r| {
                    r.body.len() > 2 && r.body[1] == 0x01 && r.body[2] == cmd.opcode
                });
                if reply.is_none() {
                    self.unanswered(w, cmd.label)?;
                } else if dropped {
                    w.attempts += 1;
                    if w.attempts >= MAX_ATTEMPTS {
                        return Err(Error::MachineDropping {
                            label: cmd.label.into(),
                            attempts: MAX_ATTEMPTS,
                        });
                    }
                    w.not_before = self.link.port.now() + MS(500);
                } else {
                    w.attempts = 0;
                    w.silent = 0;
                    w.queue.pop_front();
                }
            }
            Item::Frame { body, counts } => {
                let need = job::fifo_needed(&body);
                if !std::mem::replace(&mut w.paced, true) {
                    let wait = self.pacing.before_frame(w.fifo, need);
                    if !wait.is_zero() {
                        w.not_before = now + wait;
                        return Ok(Step::Worked);
                    }
                }
                let reply = self.link.send_enc(op::MULTI_DRAW_VECTOR, &body, MS(1000))?;
                let dropped = reply.as_ref().is_some_and(Reply::is_drop_nak);
                if let Some(free) = reply.as_ref().and_then(Reply::free_fifo)
                    && (!dropped || self.pacing.reads_free_from_refusals())
                {
                    w.fifo = Some(free);
                }
                if reply.is_none() {
                    self.unanswered(w, "MultiDrawVector")?;
                } else if dropped {
                    w.attempts += 1;
                    let since = now.saturating_sub(*w.refused_since.get_or_insert(now));
                    if self.pacing.give_up(w.attempts, since) {
                        return Err(Error::MachineDropping {
                            label: "MultiDrawVector".into(),
                            attempts: w.attempts,
                        });
                    }
                    w.not_before = self.link.port.now() + self.pacing.after_refusal(w.fifo, need);
                } else {
                    w.attempts = 0;
                    w.silent = 0;
                    w.paced = false;
                    w.refused_since = None;
                    w.queue.pop_front();
                    if counts {
                        w.frames_sent += 1;
                    }
                }
            }
        }
        Ok(Step::Worked)
    }

    /// The machine said nothing to the item at the front of the queue: it may never have arrived,
    /// so it is sent again, but not for ever.
    fn unanswered(&mut self, w: &mut Work, label: &str) -> Result<()> {
        w.silent += 1;
        if w.silent >= MAX_SILENT {
            return Err(Error::MachineDropping {
                label: label.into(),
                attempts: MAX_SILENT,
            });
        }
        w.not_before = self.link.port.now() + MS(500);
        Ok(())
    }

    /// One poll of the carriage: `Some(true)` once it is parked, `Some(false)` when it stops
    /// elsewhere, cannot be read, or time runs out.
    fn watch_step(&mut self, watch: &mut Watch) -> Result<Option<bool>> {
        let now = self.link.port.now();
        if now.saturating_sub(watch.start) >= watch.limit {
            return Ok(Some(false));
        }
        match self.link.position()? {
            None => match watch.blind_until {
                Some(until) if now >= until => return Ok(Some(false)),
                Some(_) => {}
                None => {
                    if watch.last.is_none() && now.saturating_sub(watch.start) > MS(2000) {
                        // The machine does not report its position: wait a fixed time instead.
                        watch.blind_until = Some(now + MS(3000));
                    }
                }
            },
            Some((x, y)) => {
                // y (the roller axis) settles a little short of 0 on real hardware, e.g. 0.066
                if (f64::from(x) - job::PARK.x).abs() < 0.05
                    && (f64::from(y) - job::PARK.y).abs() < 0.1
                {
                    return Ok(Some(true));
                }
                let moved = watch
                    .last
                    .is_none_or(|(lx, ly)| (x - lx).abs() > 0.001 || (y - ly).abs() > 0.001);
                if moved {
                    watch.last = Some((x, y));
                    watch.stable_since = now;
                } else if now.saturating_sub(watch.stable_since) > MS(3000) {
                    return Ok(Some(false));
                }
            }
        }
        self.link.port.sleep(MS(250));
        Ok(None)
    }
}

#[cfg(all(test, feature = "runtime"))]
mod tests;
