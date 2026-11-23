//! Deterministic cut-job encoding: the exact encrypted command sequence Design Space (and
//! opencutter) send, built from polylines and a pressure. No I/O here.

use std::time::Duration;

use crate::gcode::{Point, Polyline};
use crate::protocol::{hex, op};

/// At most 13 vectors per frame: 13 × 9 B + len + op = 119 B → one 128-byte encrypted body.
pub const VECTORS_PER_FRAME: usize = 13;

/// End-of-job carriage park position, machine inches (the machine's own rest position).
pub const PARK: Point = Point::new(4.0, 0.0);

/// Vector entry type: blade up.
pub const VECTOR_MOVE: u8 = 2;
/// Vector entry type: blade down.
pub const VECTOR_LINE: u8 = 0;

const DRAW_PAYLOAD: [u8; 3] = [0, 0, 0];

/// One encrypted command in a job.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EncCommand {
    pub opcode: u8,
    pub payload: Vec<u8>,
    pub label: &'static str,
    /// How long to wait for the reply. `OrientBlade` takes ~1.2 s, and a late reply would shift
    /// every later ACK.
    pub reply_timeout: Duration,
}

impl EncCommand {
    fn new(opcode: u8, payload: Vec<u8>, label: &'static str) -> Self {
        Self {
            opcode,
            payload,
            label,
            reply_timeout: Duration::from_millis(1000),
        }
    }

    pub fn begin_draw() -> Self {
        Self::new(op::BEGIN_DRAW, DRAW_PAYLOAD.to_vec(), "BeginDraw")
    }

    pub fn end_draw() -> Self {
        Self::new(op::END_DRAW, DRAW_PAYLOAD.to_vec(), "EndDraw")
    }
}

/// The clamp a tool sits in. The machine has no command that selects a clamp: the job's start
/// sequence decides which one it uses.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[non_exhaustive]
pub enum Clamp {
    /// Clamp A: pens (and the scoring stylus). The start sequence was captured from Design Space
    /// drawing with a pen.
    A,
    /// Clamp B: the blade. The sequence opencutter uses, verified on hardware. The default.
    #[default]
    B,
}

/// Millimetres in the inch the machine counts in.
const MM_PER_INCH: f64 = 25.4;

/// How fast the carriage moves. The machine has one tool carriage, so this is a property of the
/// job, not of the tool: a pen and a blade take the same profiles.
///
/// What sets the speed is the `Geometry` block of the setup. Measured on the machine (harness
/// `speed`, 2026-11-21), its first and third floats are the drawing speed and its fifth and
/// seventh the travel (blade-up) speed, in inches per second. Design Space sends 2.0 and 3.0 in
/// Normal and 4.0 and 5.0 in Fast Mode; those are the only profiles ever captured, and
/// [`Speed::NORMAL`] and [`Speed::FAST`] reproduce them byte for byte. The four speed registers
/// that go with them (300/150 and 10) changed neither speed, from 300 down to 10.
///
/// Every other speed is made by [`Speed::from_mm_s`] and stays inside the range Design Space has
/// sent: 1 to 4 in/s drawing and 1 to 5 in/s travel. The values the profiles differ in beyond the
/// speeds (two more `Geometry` floats and the registers, whose meaning is unknown) are mixed
/// between Normal's and Fast's in step with the drawing speed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Speed {
    /// Drawing speed, thousandths of an inch per second.
    draw_milli: u16,
    /// Travel speed, thousandths of an inch per second.
    travel_milli: u16,
    /// How far the second `Geometry` float is from Normal's (0) to Fast's (1000).
    corner_b_permille: u16,
    /// How far the sixth `Geometry` float is from Normal's (0) to Fast's (1000).
    corner_d_permille: u16,
}

impl Default for Speed {
    fn default() -> Self {
        Self::NORMAL
    }
}

impl Speed {
    /// Design Space's default: drawing at 2.0 in/s (50.8 mm/s), travel at 3.0 in/s (76.2 mm/s).
    pub const NORMAL: Self = Self {
        draw_milli: 2000,
        travel_milli: 3000,
        corner_b_permille: 0,
        corner_d_permille: 0,
    };

    /// Design Space's Fast Mode: drawing at 4.0 in/s (101.6 mm/s), travel at 5.0 in/s (127 mm/s).
    pub const FAST: Self = Self {
        draw_milli: 4000,
        travel_milli: 5000,
        corner_b_permille: 1000,
        corner_d_permille: 1000,
    };

    const DRAW_RANGE: (u16, u16) = (1000, 4000);
    const TRAVEL_RANGE: (u16, u16) = (1000, 5000);

    /// A speed of `draw_mm_s` while drawing and `travel_mm_s` while moving between shapes, held
    /// to 1..=4 in/s (25.4..=101.6 mm/s) drawing and 1..=5 in/s (25.4..=127 mm/s) travel. A value
    /// that is not a number gives the [`Speed::NORMAL`] one.
    #[must_use]
    pub fn from_mm_s(draw_mm_s: f64, travel_mm_s: f64) -> Self {
        let draw_milli = Self::milli(draw_mm_s, Self::DRAW_RANGE, Self::NORMAL.draw_milli);
        let travel_milli = Self::milli(travel_mm_s, Self::TRAVEL_RANGE, Self::NORMAL.travel_milli);
        let along = Self::along(draw_milli);
        Self {
            draw_milli,
            travel_milli,
            corner_b_permille: along,
            corner_d_permille: along,
        }
    }

    /// The same speed with the second and sixth `Geometry` floats `b_permille` and `d_permille`
    /// thousandths of the way from Normal's to Fast's. `None` when either is above 1000.
    ///
    /// For experiments: what those floats do is not known (the fill-style test in the harness
    /// exists to find out).
    #[must_use]
    pub fn with_corner_mix(self, b_permille: u16, d_permille: u16) -> Option<Self> {
        (b_permille <= 1000 && d_permille <= 1000).then_some(Self {
            corner_b_permille: b_permille,
            corner_d_permille: d_permille,
            ..self
        })
    }

    /// The drawing speed in millimetres per second.
    #[must_use]
    pub fn draw_mm_s(self) -> f64 {
        f64::from(self.draw_milli) / 1000.0 * MM_PER_INCH
    }

    /// The travel speed in millimetres per second.
    #[must_use]
    pub fn travel_mm_s(self) -> f64 {
        f64::from(self.travel_milli) / 1000.0 * MM_PER_INCH
    }

    #[expect(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        reason = "the value is rounded and held inside a u16 range on the line above"
    )]
    fn milli(mm_s: f64, (low, high): (u16, u16), fallback: u16) -> u16 {
        if !mm_s.is_finite() {
            return fallback;
        }
        (mm_s / MM_PER_INCH * 1000.0)
            .round()
            .clamp(f64::from(low), f64::from(high)) as u16
    }

    /// How far a drawing speed is from Normal's (0) to Fast's (1000).
    fn along(draw_milli: u16) -> u16 {
        draw_milli
            .saturating_sub(Self::NORMAL.draw_milli)
            .min(Self::FAST.draw_milli - Self::NORMAL.draw_milli)
            / 2
    }

    /// The `Geometry` payload: eight big-endian `f32`s, `[draw, B, draw, B, travel, D, travel, D]`.
    #[expect(
        clippy::cast_possible_truncation,
        reason = "the wire format is f32; the mix is between two f32 values"
    )]
    fn geometry(self) -> Vec<u8> {
        let mix = |normal: f32, fast: f32, permille: u16| -> f32 {
            match permille {
                0 => normal,
                1000 => fast,
                p => {
                    (f64::from(normal)
                        + (f64::from(fast) - f64::from(normal)) * f64::from(p) / 1000.0)
                        as f32
                }
            }
        };
        let draw = f32::from(self.draw_milli) / 1000.0;
        let travel = f32::from(self.travel_milli) / 1000.0;
        let b = mix(0.4, 0.6, self.corner_b_permille);
        let d = mix(38.6, 30.6, self.corner_d_permille);
        [draw, b, draw, b, travel, d, travel, d]
            .iter()
            .flat_map(|f| f.to_be_bytes())
            .collect()
    }

    /// The four `SpeedRegister` payloads: register number (`u16`), then its value (`u32`), both
    /// big-endian. They change with the drawing speed because Design Space's two profiles differ
    /// in them; they do not change how fast the machine goes.
    fn registers(self) -> Vec<Vec<u8>> {
        let permille = u32::from(Self::along(self.draw_milli));
        REGISTERS
            .iter()
            .map(|&(register, normal)| {
                // Round to nearest, halves up, in integers: normal - round((normal - 10) * p / 1000).
                let toward_fast = ((normal - FAST_REGISTER) * permille + 500) / 1000;
                let mut payload = register.to_be_bytes().to_vec();
                payload.extend_from_slice(&(normal - toward_fast).to_be_bytes());
                payload
            })
            .collect()
    }
}

/// The four speed registers and their Normal values; every register is 10 in Fast Mode.
const REGISTERS: [(u16, u32); 4] = [(0x61, 300), (0x86, 300), (0x5e, 150), (0x83, 150)];
const FAST_REGISTER: u32 = 10;

/// `SetMachineState` payload: a fixed prefix followed by the pressure, big-endian.
///
/// Clamp B's is opencutter's (`28 00 00 0d 0c 03 e8 03 e8 00 <pressure>`). Clamp A's is from a
/// Design Space pen capture (`a9 00 00 0d 0c 03 e8 03 e8 02 00 78`, pressure 120): the same bytes
/// except the first (`0x28` → `0xA9`) and the tenth (`0x00` → `0x02`). What those bits mean is
/// not known; only that Design Space sends them for a pen.
pub fn machine_state_payload(clamp: Clamp, pressure: u16) -> Vec<u8> {
    let mut p = match clamp {
        Clamp::A => hex("a900000d0c03e803e802"),
        Clamp::B => hex("2800000d0c03e803e800"),
    };
    p.extend_from_slice(&pressure.to_be_bytes());
    p
}

/// The `SetMachineState` payload for the blade in clamp B; see [`machine_state_payload`].
pub fn set_machine_state_payload(pressure: u16) -> Vec<u8> {
    machine_state_payload(Clamp::B, pressure)
}

/// The per-job setup for the blade in clamp B, sent before the first vector frame:
/// `EndDraw, SetMachineState, Geometry, Speed×4, BeginDraw, EndDraw, OrientBlade, BeginDraw`.
pub fn setup_commands(pressure: u16) -> Vec<EncCommand> {
    setup_commands_for(Clamp::B, pressure)
}

/// The per-job setup for a tool in `clamp`.
///
/// Both start `EndDraw, SetMachineState, Geometry, Speed×4, BeginDraw`. The blade then ends the
/// draw section, orients itself (`OrientBlade`, about a second) and begins another. Design Space
/// does none of that for a pen: its vectors follow the first `BeginDraw`.
pub fn setup_commands_for(clamp: Clamp, pressure: u16) -> Vec<EncCommand> {
    setup_commands_with(clamp, Speed::NORMAL, pressure)
}

/// The per-job setup for a tool in `clamp` at `speed`: [`setup_commands_for`] with the profile's
/// `Geometry` and speed registers.
pub fn setup_commands_with(clamp: Clamp, speed: Speed, pressure: u16) -> Vec<EncCommand> {
    let mut cmds = vec![
        EncCommand::end_draw(),
        EncCommand::new(
            op::SET_MACHINE_STATE,
            machine_state_payload(clamp, pressure),
            "SetMachineState",
        ),
        EncCommand::new(op::GEOMETRY, speed.geometry(), "Geometry"),
    ];
    for payload in speed.registers() {
        cmds.push(EncCommand::new(
            op::SPEED_REGISTER,
            payload,
            "SpeedRegister",
        ));
    }
    cmds.push(EncCommand::begin_draw());
    if clamp == Clamp::B {
        cmds.push(EncCommand::end_draw());
        cmds.push(EncCommand {
            reply_timeout: Duration::from_millis(3000),
            ..EncCommand::new(op::ORIENT_BLADE, hex("0000003e00000080"), "OrientBlade")
        });
        cmds.push(EncCommand::begin_draw());
    }
    cmds
}

/// Encode one 9-byte vector entry: `[type][x f32 LE][y f32 LE]`, inches.
#[expect(
    clippy::cast_possible_truncation,
    reason = "the wire format is f32; mat coordinates are far inside its exact range"
)]
pub fn vector_entry(kind: u8, p: Point) -> [u8; 9] {
    let mut e = [0u8; 9];
    e[0] = kind;
    e[1..5].copy_from_slice(&(p.x as f32).to_le_bytes());
    e[5..9].copy_from_slice(&(p.y as f32).to_le_bytes());
    e
}

/// `MultiDrawVector` frame bodies: a blade-up MOVE to the first point of each polyline, then
/// blade-down LINEs, packed 13 vectors per frame.
pub fn vector_frames(polylines: &[Polyline]) -> Vec<Vec<u8>> {
    vector_frames_of(polylines, VECTORS_PER_FRAME)
}

/// [`vector_frames`] with `per_frame` vectors in a frame (1 to 13).
pub fn vector_frames_of(polylines: &[Polyline], per_frame: usize) -> Vec<Vec<u8>> {
    let per_frame = per_frame.clamp(1, VECTORS_PER_FRAME);
    let entries: Vec<[u8; 9]> = polylines
        .iter()
        .flat_map(|pl| {
            pl.points
                .iter()
                .enumerate()
                .map(|(i, p)| vector_entry(if i == 0 { VECTOR_MOVE } else { VECTOR_LINE }, *p))
        })
        .collect();
    entries.chunks(per_frame).map(<[[u8; 9]]>::concat).collect()
}

/// The end-of-job park frame: two blade-up MOVEs to [`PARK`].
pub fn park_frame() -> Vec<u8> {
    let e = vector_entry(VECTOR_MOVE, PARK);
    [e, e].concat()
}

/// Number of FIFO slots a frame needs: one per vector plus two for framing.
pub fn fifo_needed(frame: &[u8]) -> usize {
    frame.len() / 9 + 2
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn machine_state_pressure() {
        assert_eq!(
            set_machine_state_payload(157),
            hex("2800000d0c03e803e800009d")
        );
        assert_eq!(
            set_machine_state_payload(307),
            hex("2800000d0c03e803e8000133")
        );
    }

    #[test]
    fn setup_sequence() {
        let ops: Vec<u8> = setup_commands(157).iter().map(|c| c.opcode).collect();
        assert_eq!(
            ops,
            vec![
                0x22, 0x30, 0x31, 0xA1, 0xA1, 0xA1, 0xA1, 0x21, 0x22, 0x76, 0x21
            ]
        );
        assert_eq!(setup_commands(157)[2].payload[4..8], hex("3ecccccd")[..]);
    }

    // The clamp A bytes below come from a Design Space pen job captured on 2026-11-19 over USB and
    // decrypted with the session key from the capture's own handshake. They are wire ground truth.

    #[test]
    fn clamp_a_machine_state_is_what_design_space_sent_for_a_pen() {
        assert_eq!(
            machine_state_payload(Clamp::A, 120),
            hex("a900000d0c03e803e8020078")
        );
    }

    #[test]
    fn clamp_b_machine_state_is_unchanged() {
        assert_eq!(
            machine_state_payload(Clamp::B, 157),
            hex("2800000d0c03e803e800009d")
        );
        assert_eq!(
            set_machine_state_payload(157),
            machine_state_payload(Clamp::B, 157)
        );
    }

    #[test]
    fn clamp_a_setup_never_orients_a_blade() {
        let ops: Vec<u8> = setup_commands_for(Clamp::A, 120)
            .iter()
            .map(|c| c.opcode)
            .collect();
        // The capture: EndDraw, SetMachineState, Geometry, Speed x4, BeginDraw, then vectors.
        assert_eq!(ops, vec![0x22, 0x30, 0x31, 0xA1, 0xA1, 0xA1, 0xA1, 0x21]);
        assert_eq!(setup_commands(157), setup_commands_for(Clamp::B, 157));
    }

    #[test]
    fn clamp_a_uses_the_same_geometry_and_speeds_as_the_blade() {
        // Identical in the capture and in the blade sequence.
        let pen = setup_commands_for(Clamp::A, 120);
        let blade = setup_commands(120);
        assert_eq!(pen[2], blade[2]);
        assert_eq!(pen[3..7], blade[3..7]);
    }

    // The profile bytes below come from a Design Space session captured on 2026-11-21 over USB
    // (two cuts of the same square, Fast Mode off then on), decrypted with the session key from the
    // capture's own handshake. They are wire ground truth: do not edit them to fit new code.

    const NORMAL_GEOMETRY: &str =
        "400000003ecccccd400000003ecccccd40400000421a666640400000421a6666";
    const FAST_GEOMETRY: &str = "408000003f19999a408000003f19999a40a0000041f4cccd40a0000041f4cccd";

    fn speed_payloads(cmds: &[EncCommand]) -> Vec<Vec<u8>> {
        cmds.iter()
            .filter(|c| c.opcode == op::SPEED_REGISTER)
            .map(|c| c.payload.clone())
            .collect()
    }

    /// The eight big-endian floats of a `Geometry` payload.
    fn floats(cmds: &[EncCommand]) -> Vec<f32> {
        cmds[2]
            .payload
            .chunks(4)
            .map(|c| f32::from_be_bytes(c.try_into().expect("four bytes")))
            .collect()
    }

    fn assert_floats(got: &[f32], want: &[f32]) {
        assert_eq!(got.len(), want.len());
        for (g, w) in got.iter().zip(want) {
            assert!((g - w).abs() < 1e-5, "got {got:?}, want {want:?}");
        }
    }

    #[test]
    fn normal_speed_is_what_design_space_sends_with_fast_mode_off() {
        let cmds = setup_commands_with(Clamp::B, Speed::NORMAL, 135);
        assert_eq!(cmds[2].payload, hex(NORMAL_GEOMETRY));
        assert_eq!(
            speed_payloads(&cmds),
            vec![
                hex("00610000012c"),
                hex("00860000012c"),
                hex("005e00000096"),
                hex("008300000096")
            ]
        );
    }

    #[test]
    fn fast_speed_is_what_design_space_sends_with_fast_mode_on() {
        let cmds = setup_commands_with(Clamp::B, Speed::FAST, 135);
        assert_eq!(cmds[2].payload, hex(FAST_GEOMETRY));
        assert_eq!(
            speed_payloads(&cmds),
            vec![
                hex("00610000000a"),
                hex("00860000000a"),
                hex("005e0000000a"),
                hex("00830000000a")
            ]
        );
    }

    #[test]
    fn fast_mode_changes_only_geometry_and_the_speed_registers() {
        // In the capture, pressure, the command order and everything after the speeds were equal.
        let normal = setup_commands_with(Clamp::B, Speed::NORMAL, 135);
        let fast = setup_commands_with(Clamp::B, Speed::FAST, 135);
        assert_eq!(normal.len(), fast.len());
        for (i, (n, f)) in normal.iter().zip(&fast).enumerate() {
            assert_eq!(n.opcode, f.opcode, "command {i}");
            let speed_related = n.opcode == op::GEOMETRY || n.opcode == op::SPEED_REGISTER;
            assert_eq!(n.payload == f.payload, !speed_related, "command {i}");
        }
    }

    #[test]
    fn a_pen_takes_the_fast_profile_too() {
        // The machine has one tool carriage; speed is not a property of the tool.
        let pen = setup_commands_with(Clamp::A, Speed::FAST, 120);
        assert_eq!(pen[2].payload, hex(FAST_GEOMETRY));
        assert_eq!(pen[0..2], setup_commands_for(Clamp::A, 120)[0..2]);
    }

    // Measured on the machine (harness `speed`, 2026-11-21): the first and third Geometry floats
    // are the drawing speed and the fifth and seventh the travel speed, in inches per second:
    // 2.0 and 3.0 Normal, 4.0 and 5.0 Fast. 2.0 in/s is 50.8 mm/s (50.6 measured).

    #[test]
    fn the_captured_profiles_are_speeds_in_millimetres_per_second() {
        assert_eq!(Speed::from_mm_s(50.8, 76.2), Speed::NORMAL);
        assert_eq!(Speed::from_mm_s(101.6, 127.0), Speed::FAST);
        assert!((Speed::NORMAL.draw_mm_s() - 50.8).abs() < 1e-9);
        assert!((Speed::NORMAL.travel_mm_s() - 76.2).abs() < 1e-9);
        assert!((Speed::FAST.draw_mm_s() - 101.6).abs() < 1e-9);
        assert!((Speed::FAST.travel_mm_s() - 127.0).abs() < 1e-9);
    }

    #[test]
    fn a_requested_speed_never_leaves_what_design_space_has_sent() {
        // Above Fast Mode's speeds: held at them. Far below Normal: held at 1 in/s (25.4 mm/s).
        assert_eq!(Speed::from_mm_s(500.0, 500.0), Speed::FAST);
        let slow = Speed::from_mm_s(5.0, 5.0);
        assert!((slow.draw_mm_s() - 25.4).abs() < 1e-9);
        assert!((slow.travel_mm_s() - 25.4).abs() < 1e-9);
        // Numbers that are not numbers give the default speed.
        assert_eq!(Speed::from_mm_s(f64::NAN, f64::INFINITY), Speed::NORMAL);
    }

    #[test]
    fn a_speed_halfway_between_the_profiles_has_hand_computed_bytes() {
        // 3.0 in/s is halfway from 2.0 to 4.0, so everything the profiles differ in is halfway:
        // the corner floats 0.5 and 34.6, and the registers 300 - 145 = 155 and 150 - 70 = 80.
        let cmds = setup_commands_with(Clamp::A, Speed::from_mm_s(76.2, 76.2), 120);
        assert_floats(&floats(&cmds), &[3.0, 0.5, 3.0, 0.5, 3.0, 34.6, 3.0, 34.6]);
        assert_eq!(
            speed_payloads(&cmds),
            vec![
                hex("00610000009b"),
                hex("00860000009b"),
                hex("005e00000050"),
                hex("008300000050")
            ]
        );
    }

    #[test]
    fn a_speed_below_normal_keeps_normals_other_values() {
        let cmds = setup_commands_with(Clamp::A, Speed::from_mm_s(25.4, 76.2), 120);
        assert_floats(&floats(&cmds), &[1.0, 0.4, 1.0, 0.4, 3.0, 38.6, 3.0, 38.6]);
        assert_eq!(
            speed_payloads(&cmds),
            speed_payloads(&setup_commands_with(Clamp::A, Speed::NORMAL, 120))
        );
    }

    #[test]
    fn the_corner_floats_can_be_set_apart_from_the_speed() {
        // Fast drawing with Normal's first corner float and Fast's second: B 0.4, D 30.6.
        let speed = Speed::from_mm_s(101.6, 127.0)
            .with_corner_mix(0, 1000)
            .expect("a valid mix");
        let cmds = setup_commands_with(Clamp::A, speed, 120);
        assert_floats(&floats(&cmds), &[4.0, 0.4, 4.0, 0.4, 5.0, 30.6, 5.0, 30.6]);
        assert_eq!(Speed::FAST.with_corner_mix(1001, 0), None);
        assert_eq!(Speed::FAST.with_corner_mix(0, 1001), None);
    }

    #[test]
    fn every_speed_stays_inside_the_range_design_space_has_sent() {
        for mm_s in (0..=2000).map(|i| f64::from(i) / 10.0) {
            let cmds = setup_commands_with(Clamp::A, Speed::from_mm_s(mm_s, mm_s), 120);
            let f = floats(&cmds);
            assert!(
                (1.0..=4.0).contains(&f[0]) && f[0].to_bits() == f[2].to_bits(),
                "{mm_s}: {f:?}"
            );
            assert!(
                (1.0..=5.0).contains(&f[4]) && f[4].to_bits() == f[6].to_bits(),
                "{mm_s}: {f:?}"
            );
            assert!(
                (0.4..=0.6).contains(&f[1]) && f[1].to_bits() == f[3].to_bits(),
                "{mm_s}: {f:?}"
            );
            assert!(
                (30.6..=38.6).contains(&f[5]) && f[5].to_bits() == f[7].to_bits(),
                "{mm_s}: {f:?}"
            );
            for payload in speed_payloads(&cmds) {
                let value = u32::from_be_bytes(payload[2..6].try_into().expect("four bytes"));
                assert!((10..=300).contains(&value), "{mm_s}: {value}");
            }
        }
    }

    #[test]
    fn the_default_speed_is_normal_and_existing_setups_are_unchanged() {
        assert_eq!(Speed::default(), Speed::NORMAL);
        assert_eq!(
            setup_commands_for(Clamp::B, 157),
            setup_commands_with(Clamp::B, Speed::NORMAL, 157)
        );
        assert_eq!(
            setup_commands_for(Clamp::A, 120),
            setup_commands_with(Clamp::A, Speed::NORMAL, 120)
        );
    }

    #[test]
    fn vectors_chunk_and_encode() {
        let pts: Vec<Point> = (0..20).map(|i| Point::new(f64::from(i), 1.5)).collect();
        let frames = vector_frames(&[
            Polyline { points: pts },
            Polyline {
                points: vec![Point::new(0.0, 0.0), Point::new(1.0, 1.0)],
            },
        ]);
        assert_eq!(frames.len(), 2);
        assert_eq!(frames[0].len(), 13 * 9);
        assert_eq!(frames[1].len(), 9 * 9);
        assert_eq!(frames[0][0], VECTOR_MOVE);
        assert_eq!(frames[0][9], VECTOR_LINE);
        assert_eq!(&frames[0][5..9], &1.5f32.to_le_bytes());
        // the second polyline's first entry (index 20 overall -> frame 1, entry 7) is a MOVE
        assert_eq!(frames[1][7 * 9], VECTOR_MOVE);
        assert_eq!(fifo_needed(&frames[0]), 15);
    }

    #[test]
    fn vectors_can_be_packed_into_smaller_frames() {
        // 20 points are 20 entries (a MOVE and 19 LINEs): 6, 6, 6 and 2 at six to a frame.
        let pts: Vec<Point> = (0..20).map(|i| Point::new(f64::from(i), 1.5)).collect();
        let frames = vector_frames_of(
            &[Polyline {
                points: pts.clone(),
            }],
            6,
        );
        let sizes: Vec<usize> = frames.iter().map(|f| f.len() / 9).collect();
        assert_eq!(sizes, vec![6, 6, 6, 2]);
        // The ordinary frames are the same entries, thirteen at a time.
        assert_eq!(
            vector_frames(&[Polyline {
                points: pts.clone()
            }]),
            vector_frames_of(&[Polyline { points: pts }], VECTORS_PER_FRAME)
        );
    }

    #[test]
    fn park() {
        let f = park_frame();
        assert_eq!(f.len(), 18);
        assert_eq!(f[0], 2);
        assert_eq!(&f[1..5], &4.0f32.to_le_bytes());
        assert_eq!(&f[9..18], &f[0..9]);
    }
}
