//! `cargo xtask package`: builds the plugin for `wasm32-wasip2` and packs it, with its manifest,
//! into `target/packages/cricut.psdevice` (a zip of `manifest.json` and `plugin.wasm`), the
//! format Plotter Studio installs.

#![expect(
    clippy::print_stdout,
    clippy::print_stderr,
    reason = "a command-line tool: output is its interface"
)]

use std::io::Write as _;
use std::path::{Path, PathBuf};
use std::process::{Command, ExitCode};

use anyhow::{Context, Result, bail};

fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .map_or_else(|| PathBuf::from("."), Path::to_path_buf)
}

fn package() -> Result<PathBuf> {
    let root = root();
    let status = Command::new(env!("CARGO"))
        .current_dir(&root)
        .args([
            "build",
            "-p",
            "cricut-plugin",
            "--target",
            "wasm32-wasip2",
            "--release",
            "--locked",
        ])
        .status()
        .context("running cargo")?;
    if !status.success() {
        bail!("building the plugin failed (is the wasm32-wasip2 target installed?)");
    }
    let wasm = std::fs::read(root.join("target/wasm32-wasip2/release/cricut_plugin.wasm"))
        .context("reading the built plugin")?;
    let manifest = std::fs::read_to_string(root.join("plugin/manifest.json"))
        .context("reading plugin/manifest.json")?;
    if wasm.is_empty() {
        bail!("the plugin component is empty");
    }

    let mut cursor = std::io::Cursor::new(Vec::new());
    let mut zip = zip::ZipWriter::new(&mut cursor);
    let options =
        zip::write::SimpleFileOptions::default().last_modified_time(zip::DateTime::default());
    for (name, bytes) in [
        ("manifest.json", manifest.as_bytes()),
        ("plugin.wasm", wasm.as_slice()),
    ] {
        zip.start_file(name, options)?;
        zip.write_all(bytes)?;
    }
    zip.finish()?;

    let dir = root.join("target/packages");
    std::fs::create_dir_all(&dir)?;
    let file = dir.join("cricut.psdevice");
    std::fs::write(&file, cursor.into_inner())?;
    Ok(file)
}

fn main() -> ExitCode {
    if std::env::args().nth(1).as_deref() != Some("package") {
        eprintln!("usage: cargo xtask package");
        return ExitCode::FAILURE;
    }
    match package() {
        Ok(file) => {
            println!("wrote {}", file.display());
            ExitCode::SUCCESS
        }
        Err(error) => {
            eprintln!("error: {error:#}");
            ExitCode::FAILURE
        }
    }
}
