//! Byte transports to the machine.
//!
//! The byte stream is identical over USB (CDC serial) and Bluetooth (RFCOMM channel 6). USB is
//! implemented in [`serial`]; Bluetooth is not yet. [`mock`] is an in-memory simulated machine
//! for tests and development without hardware.

use std::future::Future;
use std::time::Duration;

use crate::error::Result;

pub mod mock;
pub mod serial;

pub use mock::{FifoModel, FifoState, MockLink, SimulatedMachine};
pub use serial::{SerialLink, find_usb_ports};

/// A raw, bidirectional byte link to a machine.
pub trait Link: Send + 'static {
    /// Write all bytes.
    fn write(&mut self, data: &[u8]) -> impl Future<Output = Result<()>> + Send;

    /// Read whatever is available into `buf`, waiting at most `timeout` for the first byte.
    /// Returns `Ok(0)` on timeout. A closed link is an error.
    fn read(
        &mut self,
        buf: &mut [u8],
        timeout: Duration,
    ) -> impl Future<Output = Result<usize>> + Send;
}
