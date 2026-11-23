# cricut-gcode (library)

G-code in, Cricut Explore Air 2 protocol out, with async state and events. Every layer except
the transport is pure and testable without hardware.

## Layers (top depends on bottom)

| Module | Role | Pure? |
|---|---|---|
| `cricut` | `Cricut` controller: connect, `run_job`, `cancel`, `unload`, background polling | no |
| `engine` | the same job as a blocking, step-driven `Engine` over a `Port` (no tokio); what the WASM plugin runs | no |
| `events` | `MachineState` snapshot (`watch`) and `Event` stream (`broadcast`); `diff()` derives events | yes |
| `session` | request/reply framing over a `Link`, `NewAES` handshake, encrypted commands | no |
| `job` | deterministic encoding of a job: setup commands, 13-vector frames, park frame | yes |
| `pacing` | how frames are offered to the machine's FIFO: legacy or quick waits, frame size | yes |
| `knife` | optional drag-knife compensation / overcut (off by default) | yes |
| `gcode` | interprets blocks parsed by the `gcode` crate → `CutPlan` of polylines, machine inches | yes |
| `materials` | dial stops, pressures, Less/Default/More | yes |
| `protocol` | framing, checksum, AES-256-ECB, KEK derivation, unstuffing, reply parsing, opcode guard | yes |
| `transport` | `Link` trait; `SerialLink` (USB, 115200 8N1); `SimulatedMachine`/`MockLink` | — |

The `runtime` feature (default) gates `cricut`, `session`, `transport` and tokio. Without it the
crate is `protocol`, `job`, `gcode`, `materials`, `knife` and `engine`, and builds for
`wasm32-wasip2`: `plugin/` is the device plugin built on `engine` (see below). Keep that
build working (`cargo build -p cricut-gcode --no-default-features --target wasm32-wasip2`).

Public API surface is re-exported from `lib.rs`. Treat it as a semver contract once published:
don't rename/remove public items casually; add `#[non_exhaustive]` to enums/structs that may grow.

## Invariants (do not break)

- `protocol::check_opcode` rejects `FORBIDDEN_OPCODES` for every frame, plaintext and encrypted.
- Encrypted frames: body padded to the next 16-byte multiple that leaves a trailing checksum byte;
  header `0x80 | len`, where a 128-byte body is `0x80`. Max 13 vectors (119 → 128 bytes) per frame.
- Every job is: (Load prompt unless mat already in) → (Go unless `wait_for_go(false)`) → `0x68`
  abort → setup sequence → vector frames paced on free FIFO → `EndDraw` → park section → poll
  position until parked. Matches opencutter / Design Space; keep the order.
- Key material never appears in `Debug` output or logs (`Session`'s `Debug` is hand-written).
- `SimulatedMachine` must keep modelling verified hardware behaviour; when hardware teaches us
  something new, update the simulator **and** add a regression test that would have caught it.

## Verified on real hardware (Explore Air 2, USB, 2026-11-16)

- Encrypted ACK body: `05 00 <op> xx xx <free FIFO>`; drop NAK: `07 01 <op> …` (retry after a
  wait). `SetMachineState` (0x30) and speed (0xA1) are answered with an *echo* of the command.
- FIFO starts around 80 free; after a long job `EndDraw` may be NAKed until the queue drains.
- Interface bit `0x10` is the **mat-loaded light**, not a sensor: lighting the Load prompt
  (`72 46 20`) clears it, and pressing Load with a mat in **ejects** it. Hence a job skips the
  Load prompt when the mat is already in (`mat_in` tracking).
- Paper cuts through at pressure ≥ 157; 145 does not.
- The **first shape of every job** shows an undercut at its closing seam (top of the left wall),
  independent of position, pressure and blade; later shapes are clean. A 0.5 mm overcut
  (`DragKnife::new(0.0).overcut(0.5 / 25.4)`) fixes it. Root cause unknown.
- Software drag-knife *offset* compensation made corners worse → firmware likely compensates.
  Keep `DragKnife` offset at 0 unless a hardware test proves otherwise.
- Windows serial reads can return 0 bytes without EOF; reopening a port right after closing can
  fail with "Access is denied" (handled by retry in `SerialLink::open`).

## Pens: clamp A (from a Design Space capture; confirmed on hardware 2026-11-19)

The machine has no clamp-select command; how a job *starts* decides the clamp. Decrypted from a
Design Space pen job (USB capture, 2026-11-19, session key from the capture's own handshake) the
pen start differs from the blade start (`job::Clamp`, `setup_commands_for`) in exactly two ways:

- `SetMachineState` (0x30) is `a9 00 00 0d 0c 03 e8 03 e8 02 <pressure>` instead of
  `28 00 00 0d 0c 03 e8 03 e8 00 <pressure>`. What the changed bits mean is unknown.
- No `EndDraw, OrientBlade (0x76)` before the vectors: `…Speed×4, BeginDraw`, then vectors.

Geometry, the four speed registers and the end of the job (`EndDraw, BeginDraw, park, EndDraw`)
are identical. Design Space's pen pressure was 120, the same `M3 S` the compiler writes for a pen.
On hardware the pen job uses clamp A. Only one capture exists, so which of the two changed bytes
selects the clamp is not separated (it was a single small pen shape). The plugin picks the clamp from the header's tool table
(`generic.holder-a` / `generic.holder-b`; the older `cricut.clamp-a` / `cricut.clamp-b` still work); a tool it cannot place is cut as the blade. Design Space
also sends `0x12` (CheckFirmware) at connect; we never do (forbidden).

## Speed: what Design Space sends (capture 2026-11-21, decrypted)

Two cuts of the same 2-inch rounded square in one session, Fast Mode off then on. Decoded with
`crates/cricut-gcode/tools/decode_usb_trace.py` (the machine's `NewAES` reply gives the key). Both
jobs are byte-identical to ours up to the speed commands, including `SetMachineState` (pressure
135, unchanged by Fast Mode). They differ in exactly `Geometry` and the four `SpeedRegister`s:

| | Geometry (8 big-endian f32) | SpeedRegister 0x61 / 0x86 / 0x5e / 0x83 |
|---|---|---|
| Normal | 2.0, 0.4, 2.0, 0.4, 3.0, 38.6, 3.0, 38.6 | 300 / 300 / 150 / 150 |
| Fast | 4.0, 0.6, 4.0, 0.6, 5.0, 30.6, 5.0, 30.6 | 10 / 10 / 10 / 10 |

- **Measured on the machine (harness `speed`, 2026-11-21, pen on paper, 100 mm line):** the speed
  registers do **not** change the speed. Drawing speed was 50.6 mm/s for every normal-geometry
  setting, with the registers at 300/150, 227/115, 155/80, 82/45 and 10 alike (carriage position
  polls, `target/harness/speed-1795280526.log`). Fast `Geometry` gave 101 mm/s (118 at its peak).
  A pass of the line took 2.2 s Normal and 1.24 s Fast. What the registers do is not known.
- **`Geometry` holds literal speeds in inches per second.** Floats 0 and 2 are the drawing speed:
  2.0 in/s is 50.8 mm/s (measured 50.6), 4.0 is 101.6 (measured 101). Floats 4 and 6 are the
  travel (blade-up) speed per axis: the carriage's x and y limits measured 3.00 in/s Normal and
  5.0 in/s Fast. Floats 1 and 3 (0.4 -> 0.6) and 5 and 7 (38.6 -> 30.6) are not yet understood;
  acceleration or corner handling is the likely reading. They matter for short segments: a fill
  job of 2562 mm in 9,214 entries averaged only 14 mm/s against a 50 mm/s cruise.
- The first draft of this section said the 2-inch square ran at roughly 20 and 29 mm/s: that
  included the move in, the park move and whole-second timestamps, and is superseded by the above.
- The `speed` stage's "overhead" column grows down the table only because each job sits one row
  (15 mm) further from the park position; it is not an effect of the settings.
- **How speed is set now.** `job::Speed` is a drawing and a travel speed (`Speed::from_mm_s`),
  held to what Design Space has sent (1..=4 in/s drawing, 1..=5 in/s travel); `Speed::NORMAL` and
  `Speed::FAST` reproduce the two captured profiles byte for byte. The second and sixth
  `Geometry` floats and the registers, whose meaning is unknown, are mixed between Normal's and
  Fast's in step with the drawing speed (so 3.0 in/s gets 0.5 and 34.6). `with_corner_mix` sets
  the two floats apart from the speed, for experiments. A job (`Job::speed`) or an engine section
  (`Op::Speed`) carries one; a different speed after something was cut starts a new section, as
  a different pressure does.
- **Per machine.** The Explore Air 2 plugin maps each cut's `F` to the drawing speed
  (`speed_of`: mm/min / 60 -> mm/s) and travels at a fixed 76.2 mm/s; its profile
  (`cricut.explore-air-2.json`) says `max_speed_mm_s` 101.6 and `max_travel_speed_mm_s` 76.2,
  so the compiler holds a tool's `speed` parameter (and the G-code `F`) to what the machine
  does, and the estimate uses the same travel speed. A machine with other speeds (a Silhouette,
  say) gets its own plugin mapping and its own profile limits. `max_accel_mm_s2` (4000) is still
  an invented number, so the estimate for jobs of short segments is far too low until the
  `zigzag` and `curve` workloads below calibrate it.
- **Harness `speed` stage** (`cargo harness speed PORT --workload=line|zigzag|curve`): pen in
  clamp A on paper. Each setting draws a small and a large figure side by side on the same rows;
  the extra path over the extra time is the speed achieved, so setup, moving in and parking
  cancel. `line` checks that drawing speed follows the setting (1.0 to 4.0 in/s); `zigzag`
  (30 mm hatch rows 0.8 mm apart, joined as the compiler links a fill) and `curve` (0.2 mm
  segments round a circle) vary the unexplained `Geometry` floats at Fast's drawing speed.
  Settings are `DRAW:TRAVEL[:B:D]`. `--plan` prints the layout and connects to nothing.
- **Harness results (2026-11-21, pen, 100 mm line / 30 mm hatch rows 0.8 mm apart / 0.2 mm segments):**

  | setting | `line` mm/s | `zigzag` mm/s | `curve` mm/s |
  |---|---|---|---|
  | draw 1.0 in/s | 22.7 | | |
  | draw 2.0 in/s (Normal) | 41.6 | 35.4 | 44.4 |
  | draw 3.0 in/s | 58.2 | | |
  | draw 3.5 in/s | 67.8 | | |
  | draw 4.0 in/s, B 0 / D 0 | 77.8 (mixed B) | 47.1 | 46.2 |
  | draw 4.0 in/s, B 1000 / D 0 | | 59.7 | 17.8 |
  | draw 4.0 in/s, B 0 / D 1000 | | 47.1 | 45.8 |
  | draw 4.0 in/s, B 1000 / D 1000 | | 60.0 | 18.5 |

  (B and D are the second and sixth `Geometry` floats, 0 = Normal's 0.4 / 38.6, 1000 = Fast's
  0.6 / 30.6. The figures are path length over the extra time of the large figure, so reversals
  are included: a pass of the line ends in a stop.)
- **Drawing speed follows the setting.** Nominal 25.4, 50.8, 76.2, 88.9 and 101.6 mm/s came out
  as 22.7, 41.6, 58.2, 67.8 and 77.8 mm/s on the line; the shortfall is the turnaround of each
  pass, which takes 0.46, 0.44, 0.40, 0.35 and 0.30 s (acceleration 55, 115, 190, 255, 335 mm/s^2
  with the planner's trapezoids). It is stored as `accel_by_speed` in the profile.
- **D (the sixth float) does nothing to drawing.** Its rows match B's within noise.
- **B (the second float) trades hairpins against tiny segments.** At 4.0 in/s it cuts the cost of
  a hairpin reversal from 0.35 s to 0.21 s (zigzag 47 -> 60 mm/s), but the machine then handles
  0.2 mm segments at only about 82 entries/s (12 ms each: the FIFO sat full, 2-4 slots free), so a
  curve of them drops from 46 to 18 mm/s. Neither value is best for everything; a fill (hairpins
  and tiny segments) needs a measurement of its own.
- **The host caps tiny segments at about 217 entries/s.** Each 13-vector frame is an exchange of
  30-45 ms over the USB serial link, so 0.2 mm segments top out near 43-46 mm/s whatever the
  drawing speed; that is why `curve` at B 0 reads 44-46 for both Normal and Fast drawing. The
  machine's own limit at B 0 is therefore not known.
- **The planner reproduces the machine.** `ps-compiler`'s trapezoid planner with the acceleration
  table and a junction deviation of 0.5 mm (now in the profile; it was 0.05) predicts the
  `zigzag` rows to within 3 % and, replayed over the 9,254 moves of the pen fill, 183 s at
  50.8 mm/s against about 180 s measured on 2026-11-20. The estimate for that job as the machine
  runs it today (100 mm/s) is 116 s; it was 49 s.
- **Running a real job through the harness.** `cargo harness speed PORT --gcode=FILE` takes
  compiled G-code (comments, pauses, tool changes and `F` dropped, moved 10 mm in from the corner),
  draws it once per setting (default: Normal, then 4 in/s with the second `Geometry` float B at 0,
  500 and 1000) over the same place, and compares the total times. The pen fill of 2026-11-20
  (9,214 drawn moves, 2,562 mm, 712 frames) is the intended file; `target/harness/fill-job.gcode`
  is a copy exported with the current limits.
- **The host's feed rate is the device's latency, not ours.** A 13-vector frame is one request and
  one reply, 30-45 ms; even a bare status query takes 30 ms, our serial read loop polls every 2 ms,
  and Design Space also strictly alternates request and reply. The only lever would be sending
  several frames before reading their replies, safe by construction only while the last reported
  free FIFO covers all of them (the machine only frees space between replies), but nobody has
  checked that the machine accepts back-to-back frames, so it needs a hardware test first. It does
  not bind yet: 9,254 entries at 217 entries/s is 43 s, against 100 s or more for the fill.
- **The real fill, through the harness controller (2026-11-21, 9,254 vectors, 712 frames):**
  Normal 331 s; 4 in/s with B 0: 331 s; B 500: 275 s; B 1000: 236 s. The planner says 183 s at
  Normal and about 110 s at 4 in/s, so the machine was not the limit: **the host's pacing was.**
  The wire log (`speed-1795288860.log`) shows 87 % of the 323 s of the Normal run spent waiting
  between frames, and that 389 of 851 frames were sent into an empty FIFO (reported free 19, the
  largest value ever seen in this job), i.e. the machine had run dry. After a reply with free < 15
  the controller sleeps `(15 - free) * 100 ms` (up to 1.5 s) although the machine drains a frame
  in a few hundred ms; the same formula is in `Engine::pump`. A refused frame (138, 16 %) is
  retried after about 600 ms and refused again 75 % of the time.
- **The FIFO is shallow.** Reported free never exceeded 19 here (27 and 29 in the `curve` runs),
  and a 13-vector frame needs 15, so the machine holds about one frame: whenever the host is
  slower than the machine at emptying a frame, the queue runs dry and the carriage stops. Smaller
  frames, and retrying within one exchange (30-45 ms) instead of sleeping by that formula, are the
  untested remedies. A NAK's reply carries no free count (`07 01 43 ...`), but the code reads byte
  5 of it as one.
- B's effect on the total (331 / 275 / 236 s) is therefore a pacing artefact, not a measure of B.
- **The FIFO, read from the wire logs of the fill run (2026-11-21).** 34 slots; a frame of `n`
  vectors takes `n + 2`; the ACK reports the slots free *after* taking it (a full frame into an
  empty FIFO reports 19, a 5-vector one 27, a 3-vector one 29); a frame that does not fit is
  dropped, and the machine's NAK (`07 01 <op> 80 08` and padding) carries no free count. Two full
  frames fit, so a host that sends the next frame within one exchange (30-45 ms) can keep the
  machine fed. A frame sent after a reply of free >= 15 was never refused (463 of 463 times).
- **`pacing` (new, not yet verified on hardware).** `Pacing::LEGACY` is the old behaviour and still
  the default of the engine, the controller (`Config::pacing`) and the plugin. `Pacing::quick(poll)`
  retries after `poll` (40 ms) instead of sleeping 100 ms per slot short, ignores the junk in a NAK,
  and gives up after 30 s without a frame accepted instead of 300 attempts; `with_frame_entries`
  sends smaller frames. `SimulatedMachine::set_fifo(FifoModel { capacity: 34, entry_time })` models
  the FIFO draining over time (with `set_time` or `use_tokio_clock`) and reports `starved()`: on
  300 tiny vectors at 20 ms each legacy leaves it idle for more than 5 s and quick for under 0.5 s.
  Flip the default only after `cargo harness speed PORT --gcode=FILE --pacing=quick` beats the
  331 s legacy run of the same fill and the wire log shows few refusals and no stalls.
- **Quick pacing on the same fill (2026-11-21, `speed-1795292469.log`):** Normal 321 s, 4 in/s B 0
  320 s, B 500 259 s, B 1000 221 s, against 331 / 331 / 275 / 236 s with legacy pacing: 3-7 %
  better. The FIFO sat full (reported free median 3, about 3,000 refusals a job): **the machine,
  not the host, is the limit.** The earlier reading that waiting starved it was only a small part.
- **For this fill the machine's time is a fixed cost per drawn vector, whatever the drawing
  speed.** Per vector (setup and parking, about 10 s, taken off): 33.6 ms at B 0.4, 26.9 ms at
  B 0.5, 22.9 ms at B 0.6; time per vector times B is 13.46, 13.46 and 13.71. So the job takes about
  `vectors * 0.0135 / B` seconds, which is why Normal and 4 in/s took the same time and why B
  (never above Fast Mode's 0.6 so far) is the only setting that helped. No trapezoid planner
  reproduces it: the planner predicts 183 s at Normal and about 110 s at 4 in/s, so its estimate
  for fills of many tiny vectors is far too low (the earlier "183 s vs 180 s measured" was a
  coincidence with a rough stopwatch figure).
- **The fill has far more vectors than it needs.** 9,214 drawn vectors; simplified within 10 um
  (invisible to a 0.4-1 mm pen, 0.3 mm of 2,562 mm) 2,414 remain, within 20 um 1,957. At the
  measured per-vector cost that is about 80 s instead of 320 s at Normal, if long vectors cost no
  more. `cargo harness speed PORT --gcode=FILE --simplify=0.01` tests it on the machine.
- **The simplified fill on the machine (2026-11-21, `speed-1795294368.log`, `--simplify=0.01`, legacy
  pacing):** Normal 220 s, 4 in/s B 0: 215 s, 4 in/s B 1000: 157 s, against 321 / 320 / 221 s for the
  same fill as 9,214 vectors: **29-32 % faster** with 2,414 vectors. The prediction of 4 times was
  wrong: the vectors that went (tiny ones along boundary links) cost only about 15 ms each, while
  the 2,414 that remain are real corners and cost about 72 ms each. What is left is mostly corner
  time, and B (up to Fast Mode's 0.6) is still the only setting that moves it.
- **The compiler now simplifies every stroke** (`ps-compiler/src/simplify.rs`, Douglas-Peucker from
  the `simplify-polyline` crate, tolerance the profile's chord tolerance, 10 um): runs of straight
  moves become the fewest within tolerance, arcs are untouched and end a run. The same project
  compiles to 2,415 drawn moves instead of 9,214.
- **The estimator** now charges the machine for vectors: the planner (acceleration by speed,
  junction deviation 0.05 mm) plus `vector_ms_by_speed` (5.0 ms divided by Fast Mode's B: 12.5 ms
  at 50.8 mm/s and below, 10.0 at 76.2, 8.33 at 101.6). Against the four measured totals it is
  within 15 % (9-15 % high at Normal, 1-2 % at the speeds jobs run at); the same project now
  estimates 168 s at 100 mm/s, where the machine took 157 s at the same settings, and the original
  estimate was 49 s.
- **Harness `rounded` workload (built and run 2026-11-21, quick pacing, 6 chords a turn):** the `zigzag` rows (30 mm, 0.8 mm pitch)
  with each hairpin made a semicircle of radius 0.4 mm drawn as `--turns=N` chords (2 to 24, default 6),
  to measure what corner sharpness costs. Compare its mm/s with `zigzag`'s (35.4 Normal, 47.1 at 4 in/s
  B 0, 59.7 at B 1000). Its turns are many short vectors, so run it with `--pacing=quick`: legacy pacing
  and the 30-45 ms a frame exchange cap short vectors near 217 a second.
- **Rounded hairpins are much cheaper (`speed-1795297931.log`).** Per 30.8 mm row: Normal 0.87 s sharp,
  0.68 s rounded; 4 in/s B 0: 0.65 -> 0.54 s; 4 in/s B 1000: 0.52 -> 0.37 s (17-28 % less). The corner
  loss beyond cruising falls from 0.21-0.27 s to about 0.06 s, which is what the turn's 7 vectors cost
  (about 9 ms each), so rounding removes nearly all the corner penalty and what is left is vector
  count. The FIFO stayed full (free median 2-3): machine-bound. Next: fewer chords per turn (2, 3).
- **Finer turns are better (`--turns` 2, 3, 6, 12 at 4 in/s B 1000, 2026-11-21):** time per row 0.463,
  0.430, 0.371, 0.336 s against 0.516 for a sharp hairpin; corner loss 0.156, 0.123, 0.064, 0.029 s,
  i.e. about 0.35 s divided by the chords in the turn. Twelve chords cut a row's time by 35 %.
- **The compiler now rounds fill turns** where the profile sets `fill_turn_step_deg` (Explore Air 2:
  15): `ps-compiler/src/link.rs` `round_hairpin` replaces the sharp join between two rows that run back
  to back about one pitch apart with a semicircle of chords turning at most that many degrees, cutting
  both rows back by the radius so the turn stays inside the shape and checking each chord against the
  region (otherwise the join stays sharp). Your project compiles to 4,412 vectors (2,415 sharp) over
  2,822 mm (2,562): most vertices now turn under 30 degrees. Not yet run on the machine; the 10 um
  simplification that follows may thin the 3.4 um-sagitta chords of a small turn to about 7. The
  estimator's per-vector cost was fitted on sharp vectors and over-estimates smooth ones (162 s for the
  rounded project), to be refitted from the next run.
- The `F6000` the compiler writes is 100 mm/s, which the plugin now sends as 3.937 in/s: jobs
  drawn before 2026-11-21 ran at 2.0 in/s whatever the tool's `speed` was.

## The Plotter Studio plugin

`plugin/` (a `wasm32-wasip2` component, packed by `cargo xtask package` into
`cricut.psdevice`, installed into `%APPDATA%\PlotterStudio\data\plugins`) drives the machine for
the app. It is thin: it maps the host's parsed steps to `engine::Op` and runs `Engine::step` from
`wait-event`. Things that follow from the host's contract:

- The host cuts every plugin call off after 2 s, so every `Engine::step` must return well within
  that (long waits are split into 250 ms steps, and each machine exchange is its own step, so no
  step runs much over a second; `wait-event` stops starting steps after 500 ms). `connect` (the
  handshake) is not stepped. A command the machine leaves unanswered is sent again up to 3 times,
  then fails with `MachineDropping`; if tidying up after a fault fails too, the plugin calls
  `Engine::abandon` and reports the fault once.
- One blade pressure per cut: a tool change, a pause, or a different `M3 S<force>` starts a new
  *section*: its own prompt, Go press, setup and park. The host's runner blocks on a prompt until
  the user confirms it, so there is one prompt per section, and Go is pressed on the machine.
- Pause is not offered (the machine runs what it has queued); cancel aborts and parks, and the
  host's status polls finish the parking. The mat is never ejected automatically.
- Tests: `engine/tests.rs` (equivalence with the async controller on the simulator) and
  `host-tests/tests/e2e.rs` (the real component through the host's runner).

## Coordinates

G-code defaults to millimetres, absolute, origin **bottom-left** of a 12" sheet (+Y up);
converted to machine inches with a top-left origin, +Y towards the user, plus a (0.25", 0.25")
mat offset. Cut points outside `mat_extent_in` are rejected at parse time.

## Testing

- Unit tests next to the code (`#[cfg(test)] mod tests`); golden byte vectors come from
  opencutter's Java tests — don't "fix" them to match new code.
- `tests/simulated_job.rs` runs full jobs against `SimulatedMachine` with paused Tokio time and
  asserts the exact decrypted command stream.
- Hardware verification happens only through `crates/cricut-harness` (see `/hardware-test`).
