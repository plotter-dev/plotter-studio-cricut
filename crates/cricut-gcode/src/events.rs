//! Machine state snapshots and change events.

use crate::gcode::Point;
use crate::materials::{DialSetting, ResolvedMaterial};
use crate::protocol::{InterfaceState, MachineType};

/// Link state.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ConnectionState {
    Connecting,
    Connected,
    Disconnected { reason: String },
}

/// What a job is doing.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub enum JobStatus {
    /// No job running.
    #[default]
    Idle,
    /// Load light on; waiting for the user to load a mat.
    WaitingForMat,
    /// Go light blinking; waiting for the user to press Go.
    WaitingForGo,
    /// Sending cut vectors.
    Cutting,
    /// Everything sent; waiting for the carriage to finish and park.
    Finishing,
    Done,
    Cancelled,
    Failed {
        reason: String,
    },
}

impl JobStatus {
    pub fn is_active(&self) -> bool {
        matches!(
            self,
            JobStatus::WaitingForMat
                | JobStatus::WaitingForGo
                | JobStatus::Cutting
                | JobStatus::Finishing
        )
    }
}

/// Frame-level progress of the cut.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Progress {
    pub frames_sent: usize,
    pub frames_total: usize,
}

/// Identity of the connected machine.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MachineInfo {
    pub machine_type: MachineType,
    /// Raw `GetMachineStatus` (0x60) reply body; its layout is not fully decoded yet.
    pub raw_status: Vec<u8>,
}

/// A full snapshot of everything known about the machine. Available via
/// [`crate::Cricut::state`] / [`crate::Cricut::watch_state`].
#[derive(Debug, Clone, PartialEq)]
pub struct MachineState {
    pub connection: ConnectionState,
    pub machine: Option<MachineInfo>,
    pub interface: Option<InterfaceState>,
    pub mat_loaded: Option<bool>,
    pub dial: Option<DialSetting>,
    pub job: JobStatus,
    /// Material and pressure of the current or most recent job.
    pub material: Option<ResolvedMaterial>,
    pub progress: Option<Progress>,
    /// Last known carriage position, machine inches.
    pub position: Option<Point>,
}

impl Default for MachineState {
    fn default() -> Self {
        Self {
            connection: ConnectionState::Connecting,
            machine: None,
            interface: None,
            mat_loaded: None,
            dial: None,
            job: JobStatus::Idle,
            material: None,
            progress: None,
            position: None,
        }
    }
}

/// A change notification. Subscribe with [`crate::Cricut::subscribe`].
#[derive(Debug, Clone, PartialEq)]
pub enum Event {
    ConnectionChanged(ConnectionState),
    /// Raw lights/buttons/dial changed.
    InterfaceChanged(InterfaceState),
    MatLoadedChanged(bool),
    /// The physical material dial was turned.
    DialChanged(DialSetting),
    /// A job resolved its material and pressure.
    MaterialSelected(ResolvedMaterial),
    JobStatusChanged(JobStatus),
    Progress(Progress),
    PositionChanged(Point),
    /// Something non-fatal worth surfacing (e.g. a dropped frame being retried).
    Warning(String),
}

/// Compute the events implied by a state transition.
#[cfg(feature = "runtime")]
pub(crate) fn diff(old: &MachineState, new: &MachineState) -> Vec<Event> {
    let mut ev = Vec::new();
    if old.connection != new.connection {
        ev.push(Event::ConnectionChanged(new.connection.clone()));
    }
    if let Some(i) = new.interface.filter(|i| old.interface.as_ref() != Some(i)) {
        ev.push(Event::InterfaceChanged(i));
    }
    if let Some(m) = new.mat_loaded.filter(|m| old.mat_loaded != Some(*m)) {
        ev.push(Event::MatLoadedChanged(m));
    }
    if let Some(d) = new.dial.filter(|d| old.dial != Some(*d)) {
        ev.push(Event::DialChanged(d));
    }
    if let Some(m) = new.material.filter(|m| old.material != Some(*m)) {
        ev.push(Event::MaterialSelected(m));
    }
    if old.job != new.job {
        ev.push(Event::JobStatusChanged(new.job.clone()));
    }
    if let Some(p) = new.progress.filter(|p| old.progress != Some(*p)) {
        ev.push(Event::Progress(p));
    }
    if let Some(p) = new.position.filter(|p| old.position != Some(*p)) {
        ev.push(Event::PositionChanged(p));
    }
    ev
}
