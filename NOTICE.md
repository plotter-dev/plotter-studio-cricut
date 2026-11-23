# Notices

## Interoperability statement

This project is independent software created to interoperate with a Cricut Explore Air 2
die-cutting machine that its users own. It is **not affiliated with, authorised, or endorsed by
Cricut, Inc.** "Cricut", "Explore", "Design Space" and related marks belong to their owner and are
used here only to describe compatibility. No Cricut software is bundled, and none is required.

The communication protocol was established by [OpenCutter](https://github.com/jpechart/opencutter)
(MIT licence, Copyright (c) 2026 OpenCutter contributors) by observing a machine its author owns,
for the purpose of making that hardware work with independent software. This repository ports that
work to Rust and extends it with further observations of an owned machine. Where the
interoperability exception for reverse engineering applies (for example 17 U.S.C. §1201(f) in the
United States, or Article 6 of the EU Software Directive), this is the use it was meant for. This
is not legal advice; check your own jurisdiction.

Golden byte vectors in the tests come from OpenCutter's own tests.

## Machine key material

The Explore Air 2 authenticates motion commands with an AES session key. The key is derived from a
64-byte, per-machine-type value (the "base buffer", see `MachineType::base_buffer` in
`crates/cricut-gcode/src/protocol.rs`) that originates inside Cricut's own software. As in
OpenCutter, **this repository ships that value in source form** so the plugin works with no setup.

That is a deliberate choice and it carries legal exposure: the published code redistributes key
material derived from a vendor's software, and vendors have historically objected to this kind of
thing. If you fork or redistribute, make your own decision. The lower-exposure alternative is to
remove the literal and load the value at build or start-up time from a local, git-ignored file that
each user supplies from their own installation. Never commit protocol captures or extracted
vendor binaries.

## Third-party components

Rust crate dependencies keep their own licences; see `Cargo.lock` and run `cargo deny check`.
