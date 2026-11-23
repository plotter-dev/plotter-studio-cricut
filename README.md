# Cricut Explore Air 2 plugin for Plotter Studio

A Plotter Studio device plugin that drives a
**Cricut Explore Air 2** directly over USB: no Design Space, no cloud. It speaks the machine's
native (encrypted) cut protocol, which was worked out by
[OpenCutter](https://github.com/jpechart/opencutter) and ported here to Rust.

> This project is independent. It is **not affiliated with, authorised or endorsed by Cricut,
> Inc.** "Cricut", "Explore" and "Design Space" are trademarks of their owner and are used only
> to describe compatibility. See [NOTICE.md](NOTICE.md), including the section on key material,
> before you redistribute anything.

> **Hardware warning.** This software moves a machine with a blade. Read the safety notes, start
> with scrap material, keep a hand near the Pause/Power button, and never leave a running job
> unattended. Use at your own risk (MIT licence, no warranty).

## What is in this repository

| Path | What |
|---|---|
| [`plugin/`](plugin) | The WebAssembly device plugin (`cricut-plugin`) and its `manifest.json` |
| [`crates/cricut-gcode`](crates/cricut-gcode) | The protocol library: framing, AES session, job encoding, G-code interpreter, a blocking `engine` (what the plugin runs), an async controller and a simulated machine |
| [`crates/cricut-harness`](crates/cricut-harness) | Hardware test harness: staged probes and test cuts on a real machine |
| [`library/`](library) | Plotter Studio machine profiles and tools for the Explore family; import them in the app |
| [`wit/`](wit) | A copy of the host's plugin contract (`ps-device.wit`) this plugin is built against |
| [`host-tests/`](host-tests) | End-to-end tests of the built plugin through the Plotter Studio host (needs a host checkout) |
| [`xtask/`](xtask) | `cargo xtask package` |

## Install

1. Build the package (needs the `wasm32-wasip2` target: `rustup target add wasm32-wasip2`):

   ```sh
   cargo xtask package          # writes target/packages/cricut.psdevice
   ```

2. Copy `cricut.psdevice` into Plotter Studio's plugins folder (on Windows
   `%APPDATA%\PlotterStudio\data\plugins\`) and press refresh in the app.
3. Import the profiles and tools from [`library/`](library) through the app's library window.
   The plugin reports the machine profile `cricut.explore-air-2`; without it the app cannot
   compile jobs for the machine.
4. Connect the machine by USB, press **Load**/**Go** when the app asks, and approve the device
   the first time.

On Linux your user needs permission to open the serial port (usually membership of the
`dialout` or `uucp` group, or a udev rule for USB vendor `20d3`).

## Using the library without the app

`cricut-gcode` is a normal Rust library (`publish = false` for now):

```sh
cargo run -p cricut-gcode --example cut_gcode -- crates/cricut-gcode/examples/sample.gcode --dry-run
cargo run -p cricut-gcode --example cut_gcode -- crates/cricut-gcode/examples/sample.gcode --simulate
```

The last form without `--simulate` and with a serial port (`COM5`) **cuts for real**.

## Development

```sh
cargo fmt --all --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace
cargo build -p cricut-plugin --target wasm32-wasip2 --release
# with the host checked out next to this repo as ../plotter-studio:
cargo test --manifest-path host-tests/Cargo.toml
```

The verified behaviour of the machine (acknowledgement formats, the mat-light quirk, speeds,
first-shape undercut, clamp mapping) is recorded in
[`crates/cricut-gcode/CLAUDE.md`](crates/cricut-gcode/CLAUDE.md). The simulator models it; when
hardware teaches something new, update the simulator and add a regression test.

Firmware and EEPROM opcodes are refused by `protocol::check_opcode`. Do not remove that guard or
add raw-byte escape hatches.

## Licence

MIT, see [LICENSE](LICENSE). Protocol knowledge and test vectors derive from OpenCutter (MIT);
see [NOTICE.md](NOTICE.md).
