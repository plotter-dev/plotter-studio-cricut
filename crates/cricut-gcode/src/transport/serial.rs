//! USB CDC serial transport (115200 8N1).

use std::time::Duration;

use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio_serial::{SerialPortBuilderExt, SerialPortType, SerialStream};

use super::Link;
use crate::error::Result;

/// Cricut's USB vendor id.
pub const CRICUT_USB_VID: u16 = 0x20D3;

/// A machine attached over USB.
pub struct SerialLink {
    port: SerialStream,
    name: String,
}

impl std::fmt::Debug for SerialLink {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SerialLink")
            .field("port", &self.name)
            .finish_non_exhaustive()
    }
}

impl SerialLink {
    /// Open a serial port (e.g. `COM5` or `/dev/ttyACM0`) at 115200 baud.
    ///
    /// Retries for up to 2 s: right after a previous handle is dropped, Windows can still
    /// report the port as in use ("Access is denied").
    pub async fn open(port_name: &str) -> Result<Self> {
        let builder = tokio_serial::new(port_name, 115_200)
            .data_bits(tokio_serial::DataBits::Eight)
            .stop_bits(tokio_serial::StopBits::One)
            .parity(tokio_serial::Parity::None)
            .flow_control(tokio_serial::FlowControl::None);
        let deadline = tokio::time::Instant::now() + Duration::from_secs(2);
        let port = loop {
            match builder.clone().open_native_async() {
                Ok(p) => break p,
                Err(e) if tokio::time::Instant::now() < deadline => {
                    tracing::debug!(error = %e, "open {port_name} failed; retrying");
                    tokio::time::sleep(Duration::from_millis(100)).await;
                }
                Err(e) => return Err(e.into()),
            }
        };
        // Let the CDC device settle, then drop anything it sent while opening.
        tokio::time::sleep(Duration::from_millis(200)).await;
        let mut link = Self {
            port,
            name: port_name.to_string(),
        };
        let mut scratch = [0u8; 512];
        while link.read(&mut scratch, Duration::from_millis(50)).await? > 0 {}
        Ok(link)
    }

    pub fn port_name(&self) -> &str {
        &self.name
    }
}

impl Link for SerialLink {
    async fn write(&mut self, data: &[u8]) -> Result<()> {
        self.port.write_all(data).await?;
        self.port.flush().await?;
        Ok(())
    }

    async fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        // A 0-byte read is not end-of-file on a serial port: on Windows the port is configured
        // so reads complete immediately with whatever is buffered, possibly nothing. Keep
        // trying until data arrives or the timeout passes; a vanished device surfaces as an
        // I/O error instead.
        let deadline = tokio::time::Instant::now() + timeout;
        loop {
            match tokio::time::timeout_at(deadline, self.port.read(buf)).await {
                Err(_) => return Ok(0),
                Ok(Ok(0)) => {
                    if tokio::time::Instant::now() >= deadline {
                        return Ok(0);
                    }
                    tokio::time::sleep(Duration::from_millis(2)).await;
                }
                Ok(Ok(n)) => return Ok(n),
                Ok(Err(e)) => return Err(e.into()),
            }
        }
    }
}

/// A Cricut found on a serial port.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UsbPortInfo {
    pub port_name: String,
    pub product: Option<String>,
    pub serial_number: Option<String>,
}

/// List serial ports that look like a Cricut (vendor id 0x20D3, or "cricut" in the description).
pub fn find_usb_ports() -> Result<Vec<UsbPortInfo>> {
    Ok(tokio_serial::available_ports()?
        .into_iter()
        .filter_map(|p| match p.port_type {
            SerialPortType::UsbPort(info) => {
                let text = format!(
                    "{} {}",
                    info.product.as_deref().unwrap_or(""),
                    info.manufacturer.as_deref().unwrap_or("")
                )
                .to_lowercase();
                (info.vid == CRICUT_USB_VID || text.contains("cricut")).then_some(UsbPortInfo {
                    port_name: p.port_name,
                    product: info.product,
                    serial_number: info.serial_number,
                })
            }
            _ => None,
        })
        .collect())
}
