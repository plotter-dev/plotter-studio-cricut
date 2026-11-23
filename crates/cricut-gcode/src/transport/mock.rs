//! An in-memory simulated Explore Air 2, for tests and for developing without hardware.
//!
//! The simulation answers status queries, performs the `NewAES` handshake with the real key
//! derivation, decrypts and verifies every encrypted frame, and records what it received.
//! Tests drive the physical side (loading a mat, pressing Go) through [`SimulatedMachine`].

use std::collections::VecDeque;
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::time::Duration;

use super::Link;
use crate::error::{Error, Result};
use crate::protocol::{self, MachineType, body_len, checksum, op};

/// An encrypted command the simulated machine accepted.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReceivedCommand {
    pub opcode: u8,
    pub payload: Vec<u8>,
}

/// Observable and controllable state of the simulated machine.
#[derive(Debug, Clone)]
pub struct SimState {
    pub mat_loaded: bool,
    pub buttons: u8,
    pub dial: u8,
    pub position: (f32, f32),
    /// Last `(a, b)` pair sent with `SetLights`.
    pub lights_cmd: Option<(u8, u8)>,
    /// The Load prompt light is on. Like the real machine, this replaces the "mat loaded"
    /// light (bit 0x10) in the interface state even when a mat is physically in.
    pub load_prompt: bool,
    pub session_key: Option<[u8; 32]>,
    /// Free FIFO slots reported in ACKs, when there is no [`FifoModel`].
    pub fifo_free: u8,
    /// A FIFO that fills and drains over time; without one the machine always has room.
    pub fifo: Option<FifoState>,
    /// The machine's clock, set by whoever drives the simulation (a test's own clock).
    pub clock: Duration,
    /// Read the clock from tokio's (paused or not) instead, from this moment on.
    pub tokio_epoch: Option<tokio::time::Instant>,
    /// Drop (NAK) the next N encrypted commands.
    pub drop_next: u32,
    /// Drop (NAK) the next N `MultiDrawVector` frames.
    pub drop_vectors: u32,
    /// Leave the next N encrypted commands unanswered, and unprocessed, as a lost frame is.
    pub silent_next: u32,
    /// Encrypted commands accepted, in order.
    pub received: Vec<ReceivedCommand>,
    /// Plaintext opcodes received, in order.
    pub plain_received: Vec<u8>,
    /// Number of encrypted frames that were dropped.
    pub dropped: u32,
    /// Encrypted frames whose checksum or framing was wrong.
    pub bad_frames: u32,
    /// Reply to `MachineStatus` (0x60), body only.
    pub status_body: Vec<u8>,
}

impl Default for SimState {
    fn default() -> Self {
        Self {
            mat_loaded: false,
            buttons: 0,
            dial: 3, // Paper
            position: (4.06, 0.0),
            lights_cmd: None,
            load_prompt: false,
            session_key: None,
            fifo_free: 60,
            fifo: None,
            clock: Duration::ZERO,
            tokio_epoch: None,
            drop_next: 0,
            drop_vectors: 0,
            silent_next: 0,
            received: Vec::new(),
            plain_received: Vec::new(),
            dropped: 0,
            bad_frames: 0,
            status_body: vec![0x60, 0x6a, 0, 0, 0, 0, 0, 0, 0],
        }
    }
}

/// A FIFO that fills and drains over time, as the real machine's does (see [`crate::pacing`]).
///
/// It holds `capacity` slots and a frame of `n` vectors takes `n + 2` of them; the machine works
/// through the frames one after another, `entry_time` per vector, and a vector's slot is free once
/// it has been worked through. The machine reports the slots free after taking a frame and drops
/// one that does not fit. Observed on the machine (harness `speed` on a fill, 2026-11-21): capacity
/// 34, since a full frame into an empty FIFO reports 19, a 5-vector one 27 and a 3-vector one 29.
#[derive(Debug, Clone)]
pub struct FifoModel {
    /// Slots in the FIFO.
    pub capacity: usize,
    /// Time the machine takes over one vector.
    pub entry_time: Duration,
}

/// What the FIFO holds and how long the machine has had nothing to do.
#[derive(Debug, Clone)]
pub struct FifoState {
    model: FifoModel,
    /// `(when the machine starts the frame, vectors in it)`, oldest first.
    frames: VecDeque<(Duration, usize)>,
    /// When the last frame will be done.
    busy_until: Duration,
    /// Total time between one frame being finished and the next arriving.
    starved: Duration,
}

impl FifoState {
    /// An empty FIFO.
    pub fn new(model: FifoModel) -> Self {
        Self {
            model,
            frames: VecDeque::new(),
            busy_until: Duration::ZERO,
            starved: Duration::ZERO,
        }
    }

    /// Slots taken at `now`.
    pub fn occupied(&self, now: Duration) -> usize {
        self.frames
            .iter()
            .map(|&(start, entries)| {
                let done = if self.model.entry_time.is_zero() {
                    entries
                } else if now <= start {
                    0
                } else {
                    let worked =
                        now.saturating_sub(start).as_nanos() / self.model.entry_time.as_nanos();
                    usize::try_from(worked).unwrap_or(usize::MAX).min(entries)
                };
                let left = entries - done;
                if left == 0 { 0 } else { left + 2 }
            })
            .sum()
    }

    /// Takes a frame of `entries` vectors at `now`; the slots still free, or `None` when it does
    /// not fit.
    pub fn accept(&mut self, now: Duration, entries: usize) -> Option<usize> {
        let need = entries + 2;
        let occupied = self.occupied(now);
        if occupied + need > self.model.capacity {
            return None;
        }
        let start = self.busy_until.max(now);
        if !self.frames.is_empty() && self.busy_until < now {
            self.starved += now.saturating_sub(self.busy_until);
        }
        let count = u32::try_from(entries).unwrap_or(u32::MAX);
        self.busy_until = start + self.model.entry_time * count;
        self.frames.push_back((start, entries));
        // Frames the machine is entirely done with are forgotten.
        while self
            .frames
            .front()
            .is_some_and(|&(s, e)| s + self.model.entry_time * u32::try_from(e).unwrap_or(0) <= now)
        {
            self.frames.pop_front();
        }
        Some(self.model.capacity - occupied - need)
    }

    /// Total time the machine sat idle between frames of the same job.
    pub fn starved(&self) -> Duration {
        self.starved
    }
}

#[cfg(test)]
mod fifo_tests {
    use super::*;

    const MS: fn(u64) -> Duration = Duration::from_millis;

    fn fifo() -> FifoState {
        FifoState::new(FifoModel {
            capacity: 34,
            entry_time: MS(20),
        })
    }

    #[test]
    fn the_reported_free_count_matches_what_the_real_machine_reported() {
        // 34 slots less the frame's vectors and two: 19, 27 and 29, as in the wire log.
        assert_eq!(fifo().accept(MS(0), 13), Some(19));
        assert_eq!(fifo().accept(MS(0), 5), Some(27));
        assert_eq!(fifo().accept(MS(0), 3), Some(29));
    }

    #[test]
    fn two_full_frames_fit_and_a_third_does_not() {
        let mut f = fifo();
        assert_eq!(f.accept(MS(0), 13), Some(19));
        assert_eq!(f.accept(MS(0), 13), Some(4));
        assert_eq!(f.accept(MS(0), 13), None, "15 slots wanted, 4 free");
    }

    #[test]
    fn a_vectors_slot_is_free_once_the_machine_has_worked_through_it() {
        let mut f = fifo();
        f.accept(MS(0), 13);
        f.accept(MS(0), 13);
        // The first frame takes 13 x 20 ms = 260 ms. After 200 ms 10 of its vectors are done, so 3
        // + 2 slots remain of it, 15 of the second: 20 taken, 14 free: not enough for 15.
        assert_eq!(f.occupied(MS(200)), 20);
        assert_eq!(f.accept(MS(200), 13), None);
        // After 220 ms 11 are done: 2 + 2 + 15 = 19 taken, 15 free: just enough.
        assert_eq!(f.occupied(MS(220)), 19);
        assert_eq!(f.accept(MS(220), 13), Some(0));
    }

    #[test]
    fn a_frame_the_machine_has_finished_takes_no_slots() {
        let mut f = fifo();
        f.accept(MS(0), 13);
        assert_eq!(f.occupied(MS(260)), 0);
        assert_eq!(f.accept(MS(260), 13), Some(19));
    }

    #[test]
    fn the_machine_is_starved_for_the_time_it_has_nothing_queued() {
        // The first frame is done at 260 ms; the next arrives at 300 ms: 40 ms with nothing to do.
        let mut f = fifo();
        f.accept(MS(0), 13);
        f.accept(MS(300), 13);
        assert_eq!(f.starved(), MS(40));
        // A frame that arrives while the machine is busy costs nothing.
        f.accept(MS(310), 13);
        assert_eq!(f.starved(), MS(40));
    }

    #[test]
    fn a_machine_with_no_entry_time_is_never_full() {
        let mut f = FifoState::new(FifoModel {
            capacity: 34,
            entry_time: Duration::ZERO,
        });
        for _ in 0..10 {
            assert_eq!(f.accept(MS(0), 13), Some(19));
        }
    }
}

/// Handle to a simulated machine's state, shared with its [`MockLink`].
#[derive(Debug, Clone, Default)]
pub struct SimulatedMachine {
    state: Arc<Mutex<SimState>>,
}

impl SimulatedMachine {
    pub fn new() -> Self {
        Self::default()
    }

    /// Create a link to this machine.
    pub fn link(&self) -> MockLink {
        MockLink {
            sim: self.clone(),
            rx: VecDeque::new(),
            closed: false,
        }
    }

    /// Inspect or modify the simulated state.
    pub fn state(&self) -> MutexGuard<'_, SimState> {
        self.state.lock().unwrap_or_else(PoisonError::into_inner)
    }

    /// Gives the machine a FIFO that fills and drains over time (`None` for one that never fills).
    pub fn set_fifo(&self, model: Option<FifoModel>) {
        self.state().fifo = model.map(FifoState::new);
    }

    /// Sets the machine's clock; for tests that keep their own.
    pub fn set_time(&self, now: Duration) {
        self.state().clock = now;
    }

    /// Makes the machine read tokio's clock, which tests pause, instead of its own.
    pub fn use_tokio_clock(&self) {
        self.state().tokio_epoch = Some(tokio::time::Instant::now());
    }

    /// The user loads a mat.
    pub fn load_mat(&self) {
        let mut st = self.state();
        st.mat_loaded = true;
        st.load_prompt = false;
    }

    /// The user presses Load: loads a mat, or ejects one that is already in.
    pub fn press_load(&self) {
        let mut st = self.state();
        st.buttons |= protocol::InterfaceState::BUTTON_LOAD;
        st.mat_loaded = !st.mat_loaded;
        st.load_prompt = false;
    }

    /// The user presses Go.
    pub fn press_go(&self) {
        self.state().buttons |= protocol::InterfaceState::BUTTON_GO;
    }

    pub fn set_dial(&self, raw: u8) {
        self.state().dial = raw;
    }

    fn handle(&self, frame: &[u8]) -> Vec<u8> {
        let mut st = self.state();
        if frame[0] & 0x80 != 0 {
            return handle_encrypted(&mut st, frame);
        }
        let opcode = frame[1];
        let payload = &frame[2..];
        st.plain_received.push(opcode);
        match opcode {
            op::MACHINE_STATUS => framed(&st.status_body),
            op::INTERFACE_STATE => {
                let lights = if st.load_prompt {
                    0x20
                } else if st.mat_loaded {
                    0x10
                } else {
                    0x00
                };
                framed(&[op::INTERFACE_STATE, lights, st.buttons, 0x03, 0, st.dial])
            }
            op::MATERIAL_DIAL => framed(&[op::MATERIAL_DIAL, st.dial]),
            op::SET_LIGHTS => {
                st.lights_cmd = Some((payload[0], payload[1]));
                st.load_prompt = (payload[0], payload[1]) == (0x46, 0x20);
                framed(&[opcode])
            }
            op::CLEAR_BUTTONS => {
                st.buttons = 0;
                framed(&[opcode])
            }
            op::UNLOAD_MAT => {
                st.mat_loaded = false;
                framed(&[opcode])
            }
            op::READ_POSITION => {
                let mut b = vec![op::READ_POSITION];
                b.extend_from_slice(&st.position.0.to_be_bytes());
                b.extend_from_slice(&st.position.1.to_be_bytes());
                framed(&b)
            }
            op::RESET_AES => {
                st.session_key = None;
                framed(&[opcode])
            }
            op::NEW_AES => {
                let key: [u8; 32] = rand::random();
                st.session_key = Some(key);
                let mut block = [0u8; 48];
                block[0] = 0x01;
                block[1..33].copy_from_slice(&key);
                protocol::aes_encrypt(&mut block, &MachineType::ExploreAir2.kek());
                let mut out = vec![0x30];
                out.extend_from_slice(&block);
                escape(&out)
            }
            _ => framed(&[opcode]),
        }
    }
}

fn handle_encrypted(st: &mut SimState, frame: &[u8]) -> Vec<u8> {
    let Some(key) = st.session_key else {
        st.bad_frames += 1;
        return Vec::new();
    };
    let mut inner = frame[1..].to_vec();
    protocol::aes_decrypt(&mut inner, &key);
    let len = inner[0] as usize;
    if len == 0 || len + 1 > inner.len() - 1 || inner[inner.len() - 1] != checksum(&inner[..=len]) {
        st.bad_frames += 1;
        return Vec::new();
    }
    if st.silent_next > 0 {
        st.silent_next -= 1;
        return Vec::new();
    }
    let opcode = inner[1];
    let payload = inner[2..=len].to_vec();

    let drop_vector = opcode == op::MULTI_DRAW_VECTOR && st.drop_vectors > 0;
    // A frame the FIFO has no room for is dropped like any other.
    let now = match st.tokio_epoch {
        Some(epoch) => epoch.elapsed(),
        None => st.clock,
    };
    let mut free_after = None;
    let no_room = if opcode == op::MULTI_DRAW_VECTOR && !drop_vector && st.fifo.is_some() {
        let entries = payload.len() / 9;
        match st.fifo.as_mut().and_then(|f| f.accept(now, entries)) {
            Some(free) => {
                free_after = Some(u8::try_from(free).unwrap_or(u8::MAX));
                false
            }
            None => true,
        }
    } else {
        false
    };
    let mut body = if st.drop_next > 0 || drop_vector || no_room {
        if drop_vector {
            st.drop_vectors -= 1;
        } else if !no_room {
            st.drop_next -= 1;
        }
        st.dropped += 1;
        // The real machine's NAK is `07 01 <op> 80 08` and then padding: it carries no free count.
        vec![0x07, 0x01, opcode, 0x80, 0x08, 0x07]
    } else {
        if opcode == op::MULTI_DRAW_VECTOR
            && let Some(last) = payload.as_chunks::<9>().0.iter().last()
        {
            let x = f32::from_le_bytes(last[1..5].try_into().unwrap());
            let y = f32::from_le_bytes(last[5..9].try_into().unwrap());
            // the real machine settles slightly short of 0 on the roller axis
            st.position = if (x, y) == (4.0, 0.0) {
                (4.0, 0.066)
            } else {
                (x, y)
            };
        }
        st.received.push(ReceivedCommand { opcode, payload });
        vec![0x05, 0x00, opcode, 0, 0, free_after.unwrap_or(st.fifo_free)]
    };
    body.resize(16, 0);
    protocol::aes_encrypt(&mut body, &key);
    let mut out = vec![0x90];
    out.extend_from_slice(&body);
    escape(&out)
}

fn framed(body: &[u8]) -> Vec<u8> {
    let len = u8::try_from(body.len()).expect("simulated reply bodies are under 128 bytes");
    let mut out = vec![len];
    out.extend_from_slice(body);
    escape(&out)
}

/// The machine's byte stuffing: `0x24` → `0x5E 0xA2`.
fn escape(bytes: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(bytes.len());
    for &b in bytes {
        if b == 0x24 {
            out.extend_from_slice(&[0x5E, 0xA2]);
        } else {
            out.push(b);
        }
    }
    out
}

/// The host side of a [`SimulatedMachine`].
#[derive(Debug)]
pub struct MockLink {
    sim: SimulatedMachine,
    rx: VecDeque<u8>,
    closed: bool,
}

impl MockLink {
    /// Simulate the machine going away (USB unplugged / powered off).
    pub fn close(&mut self) {
        self.closed = true;
    }

    /// Writes to the machine without an async runtime: the replies are queued at once.
    ///
    /// # Errors
    ///
    /// [`Error::Disconnected`] after [`MockLink::close`].
    pub fn write_now(&mut self, data: &[u8]) -> Result<()> {
        self.deliver(data)
    }

    /// Takes up to `max` queued reply bytes, none when the machine has said nothing.
    ///
    /// # Errors
    ///
    /// [`Error::Disconnected`] after [`MockLink::close`].
    pub fn read_now(&mut self, max: usize) -> Result<Vec<u8>> {
        if self.closed {
            return Err(Error::Disconnected);
        }
        let n = max.min(self.rx.len());
        Ok(self.rx.drain(..n).collect())
    }

    /// Hand each frame in `data` to the simulator and queue its replies.
    fn deliver(&mut self, mut data: &[u8]) -> Result<()> {
        if self.closed {
            return Err(Error::Disconnected);
        }
        while let Some(&hdr) = data.first() {
            let body = if hdr & 0x80 != 0 {
                body_len(hdr)
            } else {
                usize::from(hdr)
            };
            let n = (1 + body).min(data.len());
            let reply = self.sim.handle(&data[..n]);
            self.rx.extend(reply);
            data = &data[n..];
        }
        Ok(())
    }
}

impl Link for MockLink {
    // The simulator replies synchronously, so writing never has to wait.
    fn write(&mut self, data: &[u8]) -> impl Future<Output = Result<()>> + Send {
        std::future::ready(self.deliver(data))
    }

    async fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        if self.closed {
            return Err(Error::Disconnected);
        }
        if self.rx.is_empty() {
            tokio::time::sleep(timeout).await;
            return Ok(0);
        }
        let n = buf.len().min(self.rx.len());
        for (slot, b) in buf.iter_mut().zip(self.rx.drain(..n)) {
            *slot = b;
        }
        Ok(n)
    }
}
