//! One open link to a machine: request/reply framing, plaintext queries, the `NewAES` handshake
//! and encrypted commands. This is the low-level layer; most users want [`crate::Cricut`].

use std::time::Duration;

use tokio::time::Instant;

use crate::error::{Error, Result};
use crate::protocol::{self, InterfaceState, MachineType, Reply, body_len, op};
use crate::transport::Link;

const MS: fn(u64) -> Duration = Duration::from_millis;

/// A machine link with protocol state (session key, receive buffer).
pub struct Session<L: Link> {
    link: L,
    machine: MachineType,
    kek: [u8; 32],
    key: Option<[u8; 32]>,
    /// Received bytes, already un-stuffed.
    rx: Vec<u8>,
    /// A trailing 0x5E that may be the first half of an escape pair.
    pending_escape: bool,
}

/// Deliberately omits the key material (KEK and session key) so it never reaches logs.
impl<L: Link> std::fmt::Debug for Session<L> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Session")
            .field("machine", &self.machine)
            .field("has_session_key", &self.key.is_some())
            .field("buffered_bytes", &self.rx.len())
            .finish_non_exhaustive()
    }
}

impl<L: Link> Session<L> {
    pub fn new(link: L, machine: MachineType) -> Self {
        Self {
            link,
            machine,
            kek: machine.kek(),
            key: None,
            rx: Vec::new(),
            pending_escape: false,
        }
    }

    pub fn machine_type(&self) -> MachineType {
        self.machine
    }

    pub fn has_session_key(&self) -> bool {
        self.key.is_some()
    }

    pub fn into_link(self) -> L {
        self.link
    }

    // ---- raw I/O ----

    async fn read_chunk(&mut self, timeout: Duration) -> Result<usize> {
        let mut buf = [0u8; 512];
        let n = self.link.read(&mut buf, timeout).await?;
        if n == 0 {
            return Ok(0);
        }
        tracing::trace!(bytes = %Hex(&buf[..n]), "rx");
        let mut raw = Vec::with_capacity(n + 1);
        if std::mem::take(&mut self.pending_escape) {
            raw.push(0x5E);
        }
        raw.extend_from_slice(&buf[..n]);
        if raw.last() == Some(&0x5E) {
            raw.pop();
            self.pending_escape = true;
        }
        self.rx.extend(protocol::unescape(&raw));
        Ok(n)
    }

    fn flush_pending_escape(&mut self) {
        if std::mem::take(&mut self.pending_escape) {
            self.rx.push(0x5E);
        }
    }

    /// Read one reply frame, waiting up to `timeout`. `Ok(None)` on timeout.
    async fn read_reply(&mut self, timeout: Duration) -> Result<Option<Reply>> {
        let deadline = Instant::now() + timeout;
        loop {
            if self.rx.is_empty() && self.pending_escape {
                // A lone 0x5E is the whole buffer: it may be a header or half an escape pair.
                if self.read_chunk(MS(30)).await? == 0 {
                    self.flush_pending_escape();
                }
                continue;
            }
            if let Some(&hdr) = self.rx.first() {
                let need = 1 + body_len(hdr);
                if self.rx.len() + 1 == need && self.pending_escape {
                    // The held-back 0x5E is all that's missing. If no 0xA2 follows promptly it
                    // was a literal byte ending the frame.
                    if self.read_chunk(MS(30)).await? == 0 {
                        self.flush_pending_escape();
                    }
                    continue;
                }
                if self.rx.len() >= need {
                    let frame: Vec<u8> = self.rx.drain(..need).collect();
                    let header = frame[0];
                    let mut body = frame[1..].to_vec();
                    if header & 0x80 != 0
                        && !body.is_empty()
                        && body.len().is_multiple_of(16)
                        && let Some(k) = &self.key
                    {
                        protocol::aes_decrypt(&mut body, k);
                    }
                    return Ok(Some(Reply { header, body }));
                }
            }
            let now = Instant::now();
            if now >= deadline {
                self.flush_pending_escape();
                if !self.rx.is_empty() {
                    tracing::debug!(bytes = ?self.rx, "incomplete reply at timeout");
                }
                return Ok(None);
            }
            self.read_chunk(deadline - now).await?;
        }
    }

    /// Discard any buffered or in-flight input.
    pub async fn drain(&mut self) -> Result<()> {
        while self.read_chunk(MS(150)).await? > 0 {}
        self.rx.clear();
        self.pending_escape = false;
        Ok(())
    }

    async fn write(&mut self, frame: &[u8]) -> Result<()> {
        // Request/reply is strict: anything still buffered is stale.
        self.rx.clear();
        self.pending_escape = false;
        tracing::trace!(bytes = %Hex(frame), "tx");
        self.link.write(frame).await
    }

    // ---- plaintext ----

    /// Send a plaintext command and wait for its reply.
    pub async fn query(
        &mut self,
        opcode: u8,
        payload: &[u8],
        timeout: Duration,
    ) -> Result<Option<Reply>> {
        let frame = protocol::plain_frame(opcode, payload)?;
        self.write(&frame).await?;
        self.read_reply(timeout).await
    }

    /// `GetMachineStatus` (0x60). Returns the raw reply body.
    pub async fn machine_status(&mut self) -> Result<Option<Vec<u8>>> {
        Ok(self
            .query(op::MACHINE_STATUS, &[0, 0, 0], MS(1000))
            .await?
            .map(|r| r.body))
    }

    /// `GetInterfaceState` (0x73): lights, buttons, progress, dial.
    pub async fn interface_state(&mut self) -> Result<Option<InterfaceState>> {
        let reply = self.query(op::INTERFACE_STATE, &[], MS(1000)).await?;
        Ok(reply.and_then(|r| InterfaceState::parse(&r.body)))
    }

    /// `GetMaterialDial` (0x38): 1 = Custom, 2..=16 = material stops. Retries a few times.
    pub async fn dial(&mut self) -> Result<Option<u8>> {
        for _ in 0..3 {
            if let Some(r) = self.query(op::MATERIAL_DIAL, &[0, 0, 0], MS(1000)).await?
                && r.body.len() >= 2
                && r.body[0] == op::MATERIAL_DIAL
            {
                return Ok(Some(r.body[1]));
            }
            self.drain().await?;
        }
        Ok(None)
    }

    /// Set the button lights. Known pairs: Load solid `46 20`, Go blink `02 08`, Go solid
    /// `02 04`, running `01 01`, all off `46 00`.
    pub async fn set_lights(&mut self, a: u8, b: u8) -> Result<()> {
        self.query(op::SET_LIGHTS, &[a, b], MS(600)).await?;
        Ok(())
    }

    /// Clear latched button presses.
    pub async fn clear_buttons(&mut self) -> Result<()> {
        self.query(op::CLEAR_BUTTONS, &[0x0F], MS(600)).await?;
        Ok(())
    }

    /// Eject the mat.
    pub async fn unload_mat(&mut self) -> Result<()> {
        self.query(op::UNLOAD_MAT, &[0x03, 0, 0], MS(3000)).await?;
        Ok(())
    }

    /// Abort / reset the cut state (also flushes queued motion).
    pub async fn abort(&mut self) -> Result<()> {
        self.query(op::ABORT, &[], MS(1000)).await?;
        Ok(())
    }

    /// Carriage position in machine inches; about (4.06, 0) at rest.
    pub async fn position(&mut self) -> Result<Option<(f32, f32)>> {
        let reply = self.query(op::READ_POSITION, &[], MS(600)).await?;
        Ok(reply.and_then(|r| protocol::parse_position(&r.body)))
    }

    // ---- NewAES handshake ----

    /// Establish the AES session (key slot 7, as Design Space uses).
    pub async fn handshake(&mut self) -> Result<()> {
        self.handshake_slot(7).await
    }

    /// Establish the AES session using key slot `slot` (2..=15).
    pub async fn handshake_slot(&mut self, slot: u8) -> Result<()> {
        const ATTEMPTS: u32 = 3;
        for attempt in 1..=ATTEMPTS {
            // Sync the machine's frame parser (and our buffer) with a cheap read-only query;
            // without this the NewAES read picks up stray bytes, especially over Bluetooth.
            let _ = self.query(op::MACHINE_STATUS, &[0, 0, 0], MS(800)).await?;
            self.drain().await?;

            let seed: [u8; 3] = rand::random();
            self.write(&protocol::plain_frame(op::RESET_AES, &seed)?)
                .await?;
            self.read_chunk(MS(600)).await?;
            for _ in 0..6 {
                if self.read_chunk(MS(150)).await? == 0 {
                    break;
                }
            }
            self.drain().await?;

            let random: [u8; 47] = std::array::from_fn(|_| rand::random());
            self.write(&protocol::new_aes_request(slot, &random))
                .await?;

            let deadline = Instant::now() + MS(2500);
            while Instant::now() < deadline {
                self.read_chunk(MS(400)).await?;
                let mut view = self.rx.clone();
                if self.pending_escape {
                    view.push(0x5E);
                }
                if let Some(key) = protocol::find_session_key(&view, &self.kek) {
                    self.key = Some(key);
                    self.rx.clear();
                    self.pending_escape = false;
                    tracing::debug!(attempt, "NewAES session established");
                    return Ok(());
                }
            }
            tracing::debug!(attempt, "NewAES handshake attempt failed");
        }
        self.key = None;
        Err(Error::HandshakeFailed { attempts: ATTEMPTS })
    }

    // ---- encrypted ----

    /// Send one encrypted command and wait for its (decrypted) reply. `Ok(None)` on timeout.
    pub async fn send_enc(
        &mut self,
        opcode: u8,
        payload: &[u8],
        timeout: Duration,
    ) -> Result<Option<Reply>> {
        let key = self.key.ok_or(Error::NoSession)?;
        let frame = protocol::enc_frame(opcode, payload, &key)?;
        tracing::debug!(op = format_args!("{opcode:02X}"), payload = %Hex(payload), "enc tx");
        self.write(&frame).await?;
        let reply = self.read_reply(timeout).await?;
        if let Some(r) = &reply {
            tracing::debug!(body = %Hex(&r.body), "enc rx");
        } else {
            tracing::debug!("enc rx: no reply");
        }
        Ok(reply)
    }
}

/// Hex formatting for wire logs.
struct Hex<'a>(&'a [u8]);

impl std::fmt::Display for Hex<'_> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        for (i, b) in self.0.iter().enumerate() {
            if i > 0 {
                f.write_str(" ")?;
            }
            write!(f, "{b:02x}")?;
        }
        Ok(())
    }
}
