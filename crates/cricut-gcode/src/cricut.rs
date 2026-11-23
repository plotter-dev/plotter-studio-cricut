//! High-level async machine controller: connect, watch state, run G-code jobs, unload.

use std::sync::{Arc, Mutex as StdMutex, PoisonError};
use std::time::Duration;

use tokio::sync::{Mutex, broadcast, watch};
use tokio::time::Instant;
use tokio_util::sync::CancellationToken;

use crate::error::{Error, Result};
use crate::events::{self, ConnectionState, Event, JobStatus, MachineInfo, MachineState, Progress};
use crate::gcode::{self, CutPlan, GcodeOptions, Point};
use crate::job::{self, Clamp, EncCommand, PARK, Speed};
use crate::materials::{DialSetting, MaterialSelection, ResolvedMaterial};
use crate::pacing::Pacing;
use crate::protocol::{InterfaceState, MachineType, op};
use crate::session::Session;
use crate::transport::Link;

const MS: fn(u64) -> Duration = Duration::from_millis;

/// Give up on a frame the machine keeps dropping after this many attempts.
const MAX_ATTEMPTS: u32 = 300;

/// Controller configuration.
#[derive(Debug, Clone)]
pub struct Config {
    pub machine_type: MachineType,
    /// How often to poll lights/buttons/dial while no job is running.
    pub poll_interval: Duration,
    /// Capacity of the event broadcast channel; slow subscribers miss old events (lagged).
    pub event_capacity: usize,
    /// How long to wait for the carriage to park at the end of a job.
    pub park_timeout: Duration,
    /// How vector frames are offered to the machine's FIFO. The default is the pacing verified
    /// on hardware; see [`Pacing`].
    pub pacing: Pacing,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            machine_type: MachineType::ExploreAir2,
            poll_interval: MS(500),
            event_capacity: 256,
            park_timeout: Duration::from_secs(180),
            pacing: Pacing::default(),
        }
    }
}

/// A cut job.
#[derive(Debug, Clone)]
pub struct Job {
    pub plan: CutPlan,
    pub material: MaterialSelection,
    /// Wait for the user to press Go (default). If false, cutting starts as soon as a mat is in.
    pub wait_for_go: bool,
    /// The clamp the tool is in; decides how the job starts. The blade's (clamp B) by default.
    pub clamp: Clamp,
    /// How fast the carriage moves; Design Space's normal speed by default.
    pub speed: Speed,
}

impl Job {
    pub fn new(plan: CutPlan) -> Self {
        Self {
            plan,
            material: MaterialSelection::default(),
            wait_for_go: true,
            clamp: Clamp::default(),
            speed: Speed::default(),
        }
    }

    /// Parse G-code into a job.
    pub fn from_gcode(src: &str, opts: &GcodeOptions) -> Result<Self> {
        Ok(Self::new(gcode::parse_with(src, opts)?))
    }

    #[must_use]
    pub fn material(mut self, material: MaterialSelection) -> Self {
        self.material = material;
        self
    }

    #[must_use]
    pub fn wait_for_go(mut self, wait: bool) -> Self {
        self.wait_for_go = wait;
        self
    }

    /// The clamp the tool is in (see [`Clamp`]).
    #[must_use]
    pub fn clamp(mut self, clamp: Clamp) -> Self {
        self.clamp = clamp;
        self
    }

    /// How fast the carriage moves (see [`Speed`]).
    #[must_use]
    pub fn speed(mut self, speed: Speed) -> Self {
        self.speed = speed;
        self
    }

    /// Apply drag-knife compensation to the job's paths (see [`crate::knife`]).
    #[must_use]
    pub fn drag_knife(mut self, knife: crate::knife::DragKnife) -> Self {
        self.plan = knife.compensate_plan(&self.plan);
        self
    }
}

/// Summary of a completed job.
#[derive(Debug, Clone, PartialEq)]
pub struct JobReport {
    pub material: ResolvedMaterial,
    pub frames: usize,
    pub vectors: usize,
    /// Frames the machine dropped (FIFO full) and that were re-sent.
    pub retries: u32,
    /// Whether the carriage was confirmed back at the park position.
    pub parked: bool,
}

struct Inner<L: Link> {
    config: Config,
    session: Mutex<Session<L>>,
    state: watch::Sender<MachineState>,
    events: broadcast::Sender<Event>,
    job_token: StdMutex<Option<CancellationToken>>,
    shutdown: CancellationToken,
    /// We saw a mat load in a job and haven't ejected it (the machine can misreport after a
    /// job dies mid-draw).
    mat_in: StdMutex<bool>,
    /// The last job ended with the head confirmed at the park position.
    parked: StdMutex<bool>,
}

/// A connected machine. Cheap to clone; all clones share one connection.
pub struct Cricut<L: Link> {
    inner: Arc<Inner<L>>,
}

impl<L: Link> std::fmt::Debug for Cricut<L> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Cricut")
            .field("config", &self.inner.config)
            .field("state", &*self.inner.state.borrow())
            .finish_non_exhaustive()
    }
}

impl<L: Link> Clone for Cricut<L> {
    fn clone(&self) -> Self {
        Self {
            inner: self.inner.clone(),
        }
    }
}

impl<L: Link> Cricut<L> {
    /// Open a session over `link`: read machine status, perform the `NewAES` handshake and start
    /// background state polling. Must be called inside a Tokio runtime.
    pub async fn connect(link: L, config: Config) -> Result<Self> {
        let (events, _) = broadcast::channel(config.event_capacity.max(1));
        let (state, _) = watch::channel(MachineState::default());
        let mut session = Session::new(link, config.machine_type);

        let raw_status = session.machine_status().await?.unwrap_or_default();
        session.handshake().await?;
        let interface = session.interface_state().await?;

        let inner = Arc::new(Inner {
            session: Mutex::new(session),
            state,
            events,
            job_token: StdMutex::new(None),
            shutdown: CancellationToken::new(),
            mat_in: StdMutex::new(false),
            parked: StdMutex::new(false),
            config,
        });
        let cricut = Self { inner };
        cricut.update(|s| {
            s.connection = ConnectionState::Connected;
            s.machine = Some(MachineInfo {
                machine_type: cricut.inner.config.machine_type,
                raw_status,
            });
        });
        if let Some(i) = interface {
            cricut.apply_interface(i);
        }
        tokio::spawn(poll_loop(cricut.inner.clone()));
        Ok(cricut)
    }

    /// Subscribe to change events.
    pub fn subscribe(&self) -> broadcast::Receiver<Event> {
        self.inner.events.subscribe()
    }

    /// A receiver that always holds the latest [`MachineState`].
    pub fn watch_state(&self) -> watch::Receiver<MachineState> {
        self.inner.state.subscribe()
    }

    /// The current state snapshot.
    pub fn state(&self) -> MachineState {
        self.inner.state.borrow().clone()
    }

    /// Request cancellation of the running job (if any). The job's `run_job` future returns
    /// [`Error::Cancelled`]. Call [`Cricut::unload`] afterwards to park and eject.
    pub fn cancel(&self) {
        if let Some(t) = self
            .inner
            .job_token
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .as_ref()
        {
            t.cancel();
        }
    }

    /// Stop background polling. The link closes when the last clone is dropped.
    pub fn shutdown(&self) {
        self.cancel();
        self.inner.shutdown.cancel();
    }

    /// Read the material dial now.
    pub async fn read_dial(&self) -> Result<DialSetting> {
        let raw = self.inner.session.lock().await.dial().await;
        let raw = self.io(raw)?.ok_or(Error::DialUnreadable)?;
        let dial = DialSetting::from_raw(raw);
        self.update(|s| s.dial = Some(dial));
        Ok(dial)
    }

    /// Read the carriage position now (machine inches).
    pub async fn read_position(&self) -> Result<Option<Point>> {
        let pos = self.inner.session.lock().await.position().await;
        let pos = self
            .io(pos)?
            .map(|(x, y)| Point::new(f64::from(x), f64::from(y)));
        if let Some(p) = pos {
            self.update(|s| s.position = Some(p));
        }
        Ok(pos)
    }

    /// Run a job to completion: Load prompt → (Go) → cut → park.
    ///
    /// Only one job runs at a time; a second call returns [`Error::Busy`].
    pub async fn run_job(&self, job: Job) -> Result<JobReport> {
        let token = {
            let mut slot = self
                .inner
                .job_token
                .lock()
                .unwrap_or_else(PoisonError::into_inner);
            if slot.is_some() {
                return Err(Error::Busy);
            }
            let t = self.inner.shutdown.child_token();
            *slot = Some(t.clone());
            t
        };
        let mut session = self.inner.session.lock().await;
        *self
            .inner
            .parked
            .lock()
            .unwrap_or_else(PoisonError::into_inner) = false;
        self.update(|s| s.progress = None);

        let result = self.run_job_locked(&mut session, &job, &token).await;
        let result = self.io(result);
        match &result {
            Ok(_) => self.update(|s| s.job = JobStatus::Done),
            Err(Error::Cancelled) => {
                // Flush anything still queued in the machine.
                let _ = session.abort().await;
                self.update(|s| s.job = JobStatus::Cancelled);
            }
            Err(e) => {
                let reason = e.to_string();
                self.update(|s| s.job = JobStatus::Failed { reason });
            }
        }
        drop(session);
        *self
            .inner
            .job_token
            .lock()
            .unwrap_or_else(PoisonError::into_inner) = None;
        result
    }

    async fn run_job_locked(
        &self,
        s: &mut Session<L>,
        job: &Job,
        token: &CancellationToken,
    ) -> Result<JobReport> {
        let material = self.resolve_material(s, job.material).await?;
        self.update(|st| st.material = Some(material));

        // Bit 0x10 is the "mat loaded" *light*, not a sensor: lighting the Load prompt clears it,
        // and pressing Load with a mat in ejects it. So if a mat is already in (we loaded it
        // and haven't ejected it, or the machine shows it), skip the Load prompt entirely.
        let already_in = *self
            .inner
            .mat_in
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            || s.interface_state().await?.is_some_and(|i| i.mat_loaded());
        if already_in {
            tracing::debug!("mat already loaded; skipping the Load prompt");
        } else {
            s.clear_buttons().await?;
            s.set_lights(0x46, 0x20).await?; // Load
            self.update(|st| st.job = JobStatus::WaitingForMat);
            self.wait_interface(s, token, InterfaceState::mat_loaded)
                .await?;
            sleep(token, MS(1000)).await?;
        }
        *self
            .inner
            .mat_in
            .lock()
            .unwrap_or_else(PoisonError::into_inner) = true;
        self.update(|st| st.mat_loaded = Some(true));

        s.clear_buttons().await?;
        if job.wait_for_go {
            s.set_lights(0x02, 0x08).await?; // Go blinking
            self.update(|st| st.job = JobStatus::WaitingForGo);
            self.wait_interface(s, token, InterfaceState::go_pressed)
                .await?;
            s.clear_buttons().await?;
        }
        s.set_lights(0x02, 0x04).await?; // Go solid
        s.abort().await?; // reset the cut state before drawing, as Design Space does
        s.set_lights(0x01, 0x01).await?;
        self.update(|st| st.job = JobStatus::Cutting);

        let mut retries = 0;
        for cmd in job::setup_commands_with(job.clamp, job.speed, material.pressure) {
            retries += self.send_cmd(s, token, &cmd).await?;
        }

        let pacing = self.inner.config.pacing;
        let frames = job::vector_frames_of(&job.plan.polylines, pacing.frame_entries());
        let total = frames.len();
        let mut last_fifo: Option<u8> = None;
        self.update(|st| {
            st.progress = Some(Progress {
                frames_sent: 0,
                frames_total: total,
            });
        });
        for (i, frame) in frames.iter().enumerate() {
            let wait = pacing.before_frame(last_fifo, job::fifo_needed(frame));
            if !wait.is_zero() {
                sleep(token, wait).await?;
            }
            let (fifo, r) = self.send_frame(s, token, frame, last_fifo).await?;
            last_fifo = fifo;
            retries += r;
            self.update(|st| {
                st.progress = Some(Progress {
                    frames_sent: i + 1,
                    frames_total: total,
                });
            });
        }
        self.send_cmd(s, token, &EncCommand::end_draw()).await?;

        // Park in its own draw section, as Design Space ends every job. Without it the head stays
        // at the last cut point and repeated jobs creep toward the end stop until the machine faults.
        retries += self.send_park(s, token, last_fifo).await?;
        self.update(|st| st.job = JobStatus::Finishing);
        let parked = self
            .wait_parked(s, token, self.inner.config.park_timeout)
            .await?;
        *self
            .inner
            .parked
            .lock()
            .unwrap_or_else(PoisonError::into_inner) = parked;

        Ok(JobReport {
            material,
            frames: total,
            vectors: job.plan.vector_count(),
            retries,
            parked,
        })
    }

    /// Stop any running job, send the head home, clear the lights and eject the mat. With no mat
    /// in, only clears the lights.
    pub async fn unload(&self) -> Result<()> {
        self.cancel();
        // Acquiring the session waits for a cancelled job to finish unwinding.
        let mut s = self.inner.session.lock().await;
        let r = self.unload_locked(&mut s).await;
        self.io(r)
    }

    async fn unload_locked(&self, s: &mut Session<L>) -> Result<()> {
        let was_parked = *self
            .inner
            .parked
            .lock()
            .unwrap_or_else(PoisonError::into_inner);
        let seen_in = *self
            .inner
            .mat_in
            .lock()
            .unwrap_or_else(PoisonError::into_inner);
        let loaded = match s.interface_state().await? {
            Some(i) => {
                self.apply_interface(i);
                seen_in || i.mat_loaded()
            }
            None => seen_in,
        };
        s.abort().await?;
        if loaded && !was_parked && s.has_session_key() {
            let none = CancellationToken::new();
            self.send_park(s, &none, None).await?;
            self.wait_parked(s, &none, Duration::from_secs(15)).await?;
            s.abort().await?;
        }
        s.set_lights(0x46, 0x00).await?;
        if loaded {
            s.unload_mat().await?;
        }
        *self
            .inner
            .mat_in
            .lock()
            .unwrap_or_else(PoisonError::into_inner) = false;
        *self
            .inner
            .parked
            .lock()
            .unwrap_or_else(PoisonError::into_inner) = false;
        self.update(|st| {
            st.job = JobStatus::Idle;
            st.mat_loaded = Some(false);
        });
        Ok(())
    }

    // ---- helpers ----

    async fn resolve_material(
        &self,
        s: &mut Session<L>,
        sel: MaterialSelection,
    ) -> Result<ResolvedMaterial> {
        Ok(match sel {
            MaterialSelection::Pressure(p) => ResolvedMaterial {
                material: None,
                pressure: p,
            },
            MaterialSelection::Material(m, adj) => ResolvedMaterial {
                material: Some(m),
                pressure: m.pressure(adj),
            },
            MaterialSelection::FromDial(adj) => {
                let dial = DialSetting::from_raw(s.dial().await?.ok_or(Error::DialUnreadable)?);
                self.update(|st| st.dial = Some(dial));
                match dial {
                    DialSetting::Material(m) => ResolvedMaterial {
                        material: Some(m),
                        pressure: m.pressure(adj),
                    },
                    DialSetting::Custom => return Err(Error::DialOnCustom),
                    DialSetting::Unknown(_) => return Err(Error::DialUnreadable),
                }
            }
        })
    }

    async fn wait_interface(
        &self,
        s: &mut Session<L>,
        token: &CancellationToken,
        cond: fn(&InterfaceState) -> bool,
    ) -> Result<()> {
        loop {
            if let Some(i) = s.interface_state().await? {
                self.apply_interface(i);
                if cond(&i) {
                    return Ok(());
                }
            }
            sleep(token, MS(150)).await?;
        }
    }

    /// Send one encrypted command, re-sending while the machine's queue is full. Returns retries.
    async fn send_cmd(
        &self,
        s: &mut Session<L>,
        token: &CancellationToken,
        cmd: &EncCommand,
    ) -> Result<u32> {
        for attempt in 1..=MAX_ATTEMPTS {
            check(token)?;
            let reply = s
                .send_enc(cmd.opcode, &cmd.payload, cmd.reply_timeout)
                .await?;
            let dropped = reply
                .as_ref()
                .is_some_and(|r| r.body.len() > 2 && r.body[1] == 0x01 && r.body[2] == cmd.opcode);
            if reply.is_none() {
                tracing::debug!(label = cmd.label, "no reply");
            }
            if !dropped {
                return Ok(attempt - 1);
            }
            self.warn(format!("{} dropped by machine; retry {attempt}", cmd.label));
            sleep(token, MS(500)).await?;
        }
        Err(Error::MachineDropping {
            label: cmd.label.into(),
            attempts: MAX_ATTEMPTS,
        })
    }

    /// Send one `MultiDrawVector` frame, pacing to the machine's FIFO. Returns (free FIFO, retries).
    async fn send_frame(
        &self,
        s: &mut Session<L>,
        token: &CancellationToken,
        body: &[u8],
        mut last_fifo: Option<u8>,
    ) -> Result<(Option<u8>, u32)> {
        let need = job::fifo_needed(body);
        let pacing = self.inner.config.pacing;
        let started = Instant::now();
        let mut attempt = 0;
        loop {
            attempt += 1;
            check(token)?;
            let reply = s.send_enc(op::MULTI_DRAW_VECTOR, body, MS(1000)).await?;
            let dropped = reply
                .as_ref()
                .is_some_and(super::protocol::Reply::is_drop_nak);
            if let Some(f) = reply.as_ref().and_then(super::protocol::Reply::free_fifo)
                && (!dropped || pacing.reads_free_from_refusals())
            {
                last_fifo = Some(f);
            }
            if !dropped {
                return Ok((last_fifo, attempt - 1));
            }
            tracing::debug!(attempt, fifo = ?last_fifo, "vector frame dropped; retrying");
            if pacing.give_up(attempt, started.elapsed()) {
                return Err(Error::MachineDropping {
                    label: "MultiDrawVector".into(),
                    attempts: attempt,
                });
            }
            sleep(token, pacing.after_refusal(last_fifo, need)).await?;
        }
    }

    async fn send_park(
        &self,
        s: &mut Session<L>,
        token: &CancellationToken,
        fifo: Option<u8>,
    ) -> Result<u32> {
        let mut retries = self.send_cmd(s, token, &EncCommand::begin_draw()).await?;
        retries += self.send_frame(s, token, &job::park_frame(), fifo).await?.1;
        retries += self.send_cmd(s, token, &EncCommand::end_draw()).await?;
        Ok(retries)
    }

    /// Poll the carriage until it reaches [`PARK`]. Gives up early if it stops elsewhere, and
    /// falls back to a fixed wait if the machine doesn't report its position.
    async fn wait_parked(
        &self,
        s: &mut Session<L>,
        token: &CancellationToken,
        timeout: Duration,
    ) -> Result<bool> {
        let t0 = Instant::now();
        let mut last: Option<(f32, f32)> = None;
        let mut stable_since = t0;
        while t0.elapsed() < timeout {
            let pos = s.position().await?;
            let now = Instant::now();
            match pos {
                None => {
                    if last.is_none() && now - t0 > MS(2000) {
                        self.warn("carriage position not readable; waiting a fixed 3 s".into());
                        sleep(token, MS(3000)).await?;
                        return Ok(false);
                    }
                }
                Some((x, y)) => {
                    self.update(|st| st.position = Some(Point::new(f64::from(x), f64::from(y))));
                    // y (the roller axis) settles a little short of 0 on real hardware, e.g. 0.066
                    if (f64::from(x) - PARK.x).abs() < 0.05 && (f64::from(y) - PARK.y).abs() < 0.1 {
                        return Ok(true);
                    }
                    let moved = last
                        .is_none_or(|(lx, ly)| (x - lx).abs() > 0.001 || (y - ly).abs() > 0.001);
                    if moved {
                        last = Some((x, y));
                        stable_since = now;
                    } else if now - stable_since > MS(3000) {
                        self.warn(format!("carriage stopped at ({x:.3}, {y:.3}), not at park"));
                        return Ok(false);
                    }
                }
            }
            sleep(token, MS(250)).await?;
        }
        Ok(false)
    }

    fn apply_interface(&self, i: InterfaceState) {
        self.update(|s| {
            s.interface = Some(i);
            s.mat_loaded = Some(i.mat_loaded());
            s.dial = Some(DialSetting::from_raw(i.dial));
        });
    }

    fn update(&self, f: impl FnOnce(&mut MachineState)) {
        update(&self.inner, f);
    }

    fn warn(&self, msg: String) {
        tracing::warn!("{msg}");
        let _ = self.inner.events.send(Event::Warning(msg));
    }

    /// Mark the connection lost on link errors, passing the result through.
    fn io<T>(&self, r: Result<T>) -> Result<T> {
        if let Err(e @ (Error::Io(_) | Error::Serial(_) | Error::Disconnected)) = &r {
            let reason = e.to_string();
            self.update(|s| s.connection = ConnectionState::Disconnected { reason });
            self.inner.shutdown.cancel();
        }
        r
    }
}

fn update<L: Link>(inner: &Inner<L>, f: impl FnOnce(&mut MachineState)) {
    let mut emitted = Vec::new();
    inner.state.send_modify(|s| {
        let old = s.clone();
        f(s);
        emitted = events::diff(&old, s);
    });
    for e in emitted {
        let _ = inner.events.send(e);
    }
}

/// Background poll of lights/buttons/dial while idle.
async fn poll_loop<L: Link>(inner: Arc<Inner<L>>) {
    loop {
        tokio::select! {
            () = inner.shutdown.cancelled() => return,
            () = tokio::time::sleep(inner.config.poll_interval) => {}
        }
        // A running job holds the session and reports state itself.
        let Ok(mut s) = inner.session.try_lock() else {
            continue;
        };
        match s.interface_state().await {
            Ok(Some(i)) => update(&inner, |st| {
                st.interface = Some(i);
                st.mat_loaded = Some(i.mat_loaded());
                st.dial = Some(DialSetting::from_raw(i.dial));
            }),
            Ok(None) => {}
            Err(e) => {
                let reason = e.to_string();
                update(&inner, |st| {
                    st.connection = ConnectionState::Disconnected { reason }
                });
                inner.shutdown.cancel();
                return;
            }
        }
    }
}

fn check(token: &CancellationToken) -> Result<()> {
    if token.is_cancelled() {
        Err(Error::Cancelled)
    } else {
        Ok(())
    }
}

async fn sleep(token: &CancellationToken, d: Duration) -> Result<()> {
    tokio::select! {
        () = token.cancelled() => Err(Error::Cancelled),
        () = tokio::time::sleep(d) => Ok(()),
    }
}
