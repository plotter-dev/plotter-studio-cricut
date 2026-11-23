use std::time::Duration;

/// Everything that can go wrong in this crate.
#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("I/O error on machine link: {0}")]
    Io(#[from] std::io::Error),

    #[cfg(feature = "runtime")]
    #[error("serial port error: {0}")]
    Serial(#[from] tokio_serial::Error),

    #[error("the machine link is closed")]
    Disconnected,

    #[error("no reply from the machine within {0:?}")]
    Timeout(Duration),

    #[error("NewAES handshake did not complete after {attempts} attempts")]
    HandshakeFailed { attempts: u32 },

    #[error("no encrypted session; the handshake has not been performed")]
    NoSession,

    #[error("refusing to send opcode 0x{0:02X}: it can brick or mis-calibrate the machine")]
    ForbiddenOpcode(u8),

    #[error("frame too large: {0} bytes (max 128 encrypted / 255 plaintext)")]
    FrameTooLarge(usize),

    #[error("machine kept dropping {label} after {attempts} attempts")]
    MachineDropping { label: String, attempts: u32 },

    #[error("G-code error on line {line}: {message}")]
    Gcode { line: usize, message: String },

    #[error("material dial is set to Custom; choose a material explicitly")]
    DialOnCustom,

    #[error("could not read the material dial")]
    DialUnreadable,

    #[error("the program moves to ({x_in:.2}, {y_in:.2}) in, outside the cutting mat")]
    OutOfBounds { x_in: f64, y_in: f64 },

    #[error("no job is running")]
    NoJob,

    #[error("a job is already running")]
    Busy,

    #[error("the job was cancelled")]
    Cancelled,
}

pub type Result<T, E = Error> = std::result::Result<T, E>;
