//! List Cricut USB ports, or connect to one and print live state changes.
//!
//! ```text
//! cargo run --example status            # list ports
//! cargo run --example status -- COM5    # connect and watch (Ctrl+C to quit)
//! ```

#![expect(
    clippy::print_stdout,
    reason = "example program: output is its interface"
)]

use cricut_gcode::transport::{SerialLink, find_usb_ports};
use cricut_gcode::{Config, Cricut};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let Some(port) = std::env::args().nth(1) else {
        let ports = find_usb_ports()?;
        if ports.is_empty() {
            println!("no Cricut USB ports found");
        }
        for p in ports {
            println!("{}  {}", p.port_name, p.product.unwrap_or_default());
        }
        return Ok(());
    };

    let cricut = Cricut::connect(SerialLink::open(&port).await?, Config::default()).await?;
    println!("connected: {:#?}", cricut.state());
    let mut events = cricut.subscribe();
    loop {
        tokio::select! {
            ev = events.recv() => println!("{:?}", ev?),
            _ = tokio::signal::ctrl_c() => break,
        }
    }
    cricut.shutdown();
    Ok(())
}
