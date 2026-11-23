//! # cricut-gcode
//!
//! Drive a Cricut Explore Air 2 directly from G-code: no Design Space, no cloud.
//!
//! The crate is layered so that everything except the transport is pure and testable:
//!
//! | module | role |
//! |---|---|
//! | [`gcode`] | G-code → polylines in machine inches |
//! | [`materials`] | dial materials and pressures |
//! | [`job`] | deterministic encoding of a cut job into encrypted commands |
//! | [`protocol`] | framing, checksums, AES, key derivation, reply parsing |
//! | [`transport`] | byte links: USB serial, and an in-memory simulated machine |
//! | [`session`] | request/reply, `NewAES` handshake, encrypted commands |
//! | [`Cricut`] | async controller with state ([`MachineState`]) and [`Event`]s |
//! | [`engine`] | the same job as a blocking, step-driven machine with no async runtime |
//!
//! The `runtime` feature (on by default) adds [`Cricut`], [`session`] and [`transport`]; without it
//! the crate builds for WebAssembly, which is how the device plugin uses [`engine`].
//!
//! ```no_run
//! use cricut_gcode::{Config, Cricut, Event, GcodeOptions, Job, transport::SerialLink};
//!
//! # async fn demo() -> cricut_gcode::Result<()> {
//! let link = SerialLink::open("COM5").await?;
//! let cricut = Cricut::connect(link, Config::default()).await?;
//!
//! let mut events = cricut.subscribe();
//! tokio::spawn(async move {
//!     while let Ok(ev) = events.recv().await {
//!         println!("{ev:?}");
//!     }
//! });
//!
//! let job = Job::from_gcode("G21\nG0 X10 Y10\nG1 X60 Y10\nG1 X60 Y60\n", &GcodeOptions::default())?;
//! let report = cricut.run_job(job).await?; // Load mat, press Go
//! cricut.unload().await?;
//! # Ok(()) }
//! ```
//!
//! **Safety:** opcodes that write firmware or persistent memory are rejected at the framing
//! layer ([`protocol::FORBIDDEN_OPCODES`]).

#[cfg(feature = "runtime")]
pub mod cricut;
pub mod engine;
pub mod error;
pub mod events;
pub mod gcode;
pub mod job;
pub mod knife;
pub mod materials;
pub mod pacing;
pub mod protocol;
#[cfg(feature = "runtime")]
pub mod session;
#[cfg(feature = "runtime")]
pub mod transport;

#[cfg(feature = "runtime")]
pub use cricut::{Config, Cricut, Job, JobReport};
pub use error::{Error, Result};
pub use events::{ConnectionState, Event, JobStatus, MachineInfo, MachineState, Progress};
pub use gcode::{CutPlan, GcodeOptions, Origin, Point, Polyline};
pub use knife::DragKnife;
pub use materials::{DialSetting, Material, MaterialSelection, PressureAdjust, ResolvedMaterial};
pub use pacing::Pacing;
pub use protocol::{InterfaceState, MachineType};
