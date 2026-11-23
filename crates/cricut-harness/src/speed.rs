//! Planning and arithmetic of the `speed` stage, kept free of I/O so it can be tested.
//!
//! The stage draws the same kind of figure at several speed settings, once small and once large.
//! Everything that does not depend on the figure's size (setup commands, moving in from the park
//! position, parking again) is in both runs, so the extra time of the large run over the small
//! one is the extra path alone, and its length over that time is the speed the machine achieved
//! on that kind of path. Each pair sits on the same rows of the sheet, so the distance from the
//! park position is the same for both.

use std::f64::consts::TAU;
use std::fmt::Write as _;
use std::time::Duration;

use cricut_gcode::job::Speed;

/// Millimetres in an inch; the machine's speeds are inches a second.
const MM_PER_INCH: f64 = 25.4;
/// Where the first band of figures starts, millimetres from the sheet's top edge.
pub(crate) const FIRST_ROW_MM: f64 = 10.0;
/// The lowest row the sheet is allowed to reach (a 12 inch mat is 304.8 mm).
pub(crate) const LAST_ROW_LIMIT_MM: f64 = 280.0;
/// Where every figure of the small column starts, millimetres from the left edge.
pub(crate) const SMALL_X_MM: f64 = 10.0;

/// One speed setting to measure: speeds in inches a second, and optionally how far the two
/// `Geometry` floats whose meaning is unknown are from Normal's (0) to Fast's (1000).
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Profile {
    pub(crate) draw_in_s: f64,
    pub(crate) travel_in_s: f64,
    pub(crate) corner_mix: Option<(u16, u16)>,
}

impl Profile {
    /// The [`Speed`] to send.
    pub(crate) fn speed(self) -> Speed {
        let speed = Speed::from_mm_s(self.draw_in_s * MM_PER_INCH, self.travel_in_s * MM_PER_INCH);
        match self.corner_mix {
            Some((b, d)) => speed.with_corner_mix(b, d).unwrap_or(speed),
            None => speed,
        }
    }

    /// A short name such as `draw 4.0, travel 3.0, corners 0/1000`.
    pub(crate) fn label(self) -> String {
        let mut label = format!("draw {:.1}, travel {:.1}", self.draw_in_s, self.travel_in_s);
        if let Some((b, d)) = self.corner_mix {
            let _ = write!(label, ", corners {b}/{d}");
        }
        label
    }
}

const fn profile(draw: f64, travel: f64, corner_mix: Option<(u16, u16)>) -> Profile {
    Profile {
        draw_in_s: draw,
        travel_in_s: travel,
        corner_mix,
    }
}

/// What the `line` workload measures by default: how drawing speed follows the setting.
pub(crate) const LINE_PROFILES: [Profile; 5] = [
    profile(1.0, 3.0, None),
    profile(2.0, 3.0, None),
    profile(3.0, 3.0, None),
    profile(3.5, 3.0, None),
    profile(4.0, 3.0, None),
];

/// What the short-segment workloads measure by default: Normal, then Fast's drawing speed with
/// each unexplained `Geometry` float at Normal's or Fast's value.
pub(crate) const CORNER_PROFILES: [Profile; 5] = [
    profile(2.0, 3.0, None),
    profile(4.0, 3.0, Some((0, 0))),
    profile(4.0, 3.0, Some((1000, 0))),
    profile(4.0, 3.0, Some((0, 1000))),
    profile(4.0, 3.0, Some((1000, 1000))),
];

/// Parses `2:3,4:3:0:1000`: the drawing and travel speeds in inches a second, then optionally the
/// two corner floats in thousandths of the way from Normal's value to Fast's.
///
/// # Errors
///
/// A message naming the part that is not understood or leaves the range Design Space has sent
/// (1 to 4 in/s drawing, 1 to 5 in/s travel, 0 to 1000 for the floats).
pub(crate) fn parse_profiles(text: &str) -> Result<Vec<Profile>, String> {
    text.split(',')
        .map(|part| {
            let part = part.trim();
            let fields: Vec<&str> = part.split(':').collect();
            let number = |s: &str| -> Result<f64, String> {
                s.parse::<f64>()
                    .ok()
                    .filter(|v| v.is_finite())
                    .ok_or_else(|| format!("{part:?}: {s:?} is not a number"))
            };
            let (draw, travel, mix) = match fields.as_slice() {
                [draw, travel] => (number(draw)?, number(travel)?, None),
                [draw, travel, b, d] => {
                    let permille = |s: &str| {
                        s.parse::<u16>()
                            .ok()
                            .filter(|p| *p <= 1000)
                            .ok_or_else(|| format!("{part:?}: {s:?} must be from 0 to 1000"))
                    };
                    (
                        number(draw)?,
                        number(travel)?,
                        Some((permille(b)?, permille(d)?)),
                    )
                }
                _ => return Err(format!("{part:?}: expected DRAW:TRAVEL or DRAW:TRAVEL:B:D")),
            };
            if !(1.0..=4.0).contains(&draw) {
                return Err(format!("{part:?}: drawing speed must be 1 to 4 in/s"));
            }
            if !(1.0..=5.0).contains(&travel) {
                return Err(format!("{part:?}: travel speed must be 1 to 5 in/s"));
            }
            Ok(profile(draw, travel, mix))
        })
        .collect()
}

/// The kind of figure drawn.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Workload {
    /// A 100 mm line drawn out and back: long moves, one reversal at the end of each.
    Line,
    /// Hatch rows joined into one stroke, as the compiler links a fill: 30 mm rows 0.8 mm apart,
    /// so a sharp reversal every 30 mm and a 0.8 mm step between.
    Zigzag,
    /// A circle of 15 mm radius made of 0.2 mm segments: many tiny moves, no sharp corners.
    Curve,
    /// The `Zigzag` rows with each hairpin made a semicircle of radius half the row pitch, drawn as
    /// this many chords: the same path with gentler corners, to see what the corners cost.
    Rounded(u32),
}

const LINE_MM: f64 = 100.0;
const ZIGZAG_WIDTH_MM: f64 = 30.0;
const ZIGZAG_PITCH_MM: f64 = 0.8;
const CURVE_RADIUS_MM: f64 = 15.0;
const CURVE_SEGMENT_MM: f64 = 0.2;
/// Chords in each turn of the rounded zigzag unless told otherwise.
const DEFAULT_TURN_CHORDS: u32 = 6;

impl Workload {
    /// Parses the `--workload=` value.
    pub(crate) fn parse(text: &str) -> Option<Self> {
        match text {
            "line" => Some(Self::Line),
            "zigzag" => Some(Self::Zigzag),
            "curve" => Some(Self::Curve),
            "rounded" => Some(Self::Rounded(DEFAULT_TURN_CHORDS)),
            _ => None,
        }
    }

    pub(crate) fn name(self) -> &'static str {
        match self {
            Self::Line => "line",
            Self::Zigzag => "zigzag",
            Self::Curve => "curve",
            Self::Rounded(_) => "rounded",
        }
    }

    /// The settings measured when none are given.
    pub(crate) fn default_profiles(self) -> Vec<Profile> {
        match self {
            Self::Line => LINE_PROFILES.to_vec(),
            Self::Zigzag | Self::Curve | Self::Rounded(_) => CORNER_PROFILES.to_vec(),
        }
    }

    /// What one unit of size is: a pass, a row or a loop.
    pub(crate) fn unit(self) -> &'static str {
        match self {
            Self::Line => "pass",
            Self::Zigzag | Self::Rounded(_) => "row",
            Self::Curve => "loop",
        }
    }

    /// The sizes of the small and the large figure, in units.
    pub(crate) fn default_sizes(self) -> (u32, u32) {
        match self {
            Self::Line => (1, 5),
            Self::Zigzag | Self::Rounded(_) => (10, 50),
            Self::Curve => (1, 4),
        }
    }

    /// The largest size of the large figure that fits its band.
    pub(crate) fn max_size(self) -> u32 {
        match self {
            Self::Line => 9,
            Self::Zigzag | Self::Rounded(_) => 60,
            Self::Curve => 8,
        }
    }

    /// Height of the band of the sheet one setting uses, in millimetres.
    pub(crate) fn band_mm(self) -> f64 {
        match self {
            Self::Line => 15.0,
            Self::Zigzag | Self::Rounded(_) => 42.0,
            Self::Curve => 35.0,
        }
    }

    /// Where the large figure starts, to the right of the small one.
    pub(crate) fn large_offset_mm(self) -> f64 {
        match self {
            Self::Line => 140.0,
            Self::Zigzag | Self::Rounded(_) => 42.0,
            Self::Curve => 45.0,
        }
    }

    #[expect(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        reason = "a small positive whole number of segments, rounded up on the line above"
    )]
    fn curve_points() -> u32 {
        // 0.2 mm segments round a 94.2 mm circle: 472 of them.
        (TAU * CURVE_RADIUS_MM / CURVE_SEGMENT_MM).ceil() as u32
    }

    /// The length of the path of `size` units, in millimetres.
    pub(crate) fn length_mm(self, size: u32) -> f64 {
        let n = f64::from(size);
        match self {
            Self::Line => LINE_MM * n,
            Self::Zigzag => ZIGZAG_WIDTH_MM * n + ZIGZAG_PITCH_MM * (n - 1.0).max(0.0),
            Self::Rounded(turn) => {
                // Each turn is `turn` chords of a semicircle of radius half the pitch.
                let chord =
                    ZIGZAG_PITCH_MM * (std::f64::consts::PI / (2.0 * f64::from(turn))).sin();
                ZIGZAG_WIDTH_MM * n + f64::from(turn) * chord * (n - 1.0).max(0.0)
            }
            Self::Curve => {
                let m = f64::from(Self::curve_points());
                n * m * 2.0 * CURVE_RADIUS_MM * (std::f64::consts::PI / m).sin()
            }
        }
    }

    /// G-code for a figure of `size` units whose top-left corner is at (`x`, `y`) millimetres from
    /// the sheet's top-left corner.
    pub(crate) fn gcode(self, x: f64, y: f64, size: u32) -> String {
        let mut code = String::from("G21 G90\n");
        match self {
            Self::Line => {
                let _ = writeln!(code, "G0 X{x:.3} Y{y:.3}");
                for pass in 0..size {
                    let at = if pass % 2 == 0 { x + LINE_MM } else { x };
                    let _ = writeln!(code, "G1 X{at:.3} Y{y:.3}");
                }
            }
            Self::Zigzag => {
                let _ = writeln!(code, "G0 X{x:.3} Y{y:.3}");
                for row in 0..size {
                    let at = if row % 2 == 0 { x + ZIGZAG_WIDTH_MM } else { x };
                    let row_y = y + ZIGZAG_PITCH_MM * f64::from(row);
                    let _ = writeln!(code, "G1 X{at:.3} Y{row_y:.3}");
                    if row + 1 < size {
                        let _ = writeln!(code, "G1 X{at:.3} Y{:.3}", row_y + ZIGZAG_PITCH_MM);
                    }
                }
            }
            Self::Rounded(turn) => {
                let _ = writeln!(code, "G0 X{x:.3} Y{y:.3}");
                let radius = ZIGZAG_PITCH_MM / 2.0;
                for row in 0..size {
                    let rightwards = row % 2 == 0;
                    let at = if rightwards { x + ZIGZAG_WIDTH_MM } else { x };
                    let row_y = y + ZIGZAG_PITCH_MM * f64::from(row);
                    let _ = writeln!(code, "G1 X{at:.3} Y{row_y:.3}");
                    if row + 1 < size {
                        // A semicircle about the point half a pitch below the row's end, out to the
                        // right after a rightward row and to the left after a leftward one.
                        let sweep = if rightwards { 1.0 } else { -1.0 };
                        for k in 1..=turn {
                            let angle = -std::f64::consts::FRAC_PI_2
                                + sweep * std::f64::consts::PI * f64::from(k) / f64::from(turn);
                            let _ = writeln!(
                                code,
                                "G1 X{:.3} Y{:.3}",
                                at + radius * angle.cos(),
                                row_y + radius + radius * angle.sin()
                            );
                        }
                    }
                }
            }
            Self::Curve => {
                let (cx, cy) = (x + CURVE_RADIUS_MM, y + CURVE_RADIUS_MM);
                let m = Self::curve_points();
                let _ = writeln!(code, "G0 X{:.3} Y{cy:.3}", cx + CURVE_RADIUS_MM);
                for k in (1..=m * size).map(|k| k % m) {
                    let angle = TAU * f64::from(k) / f64::from(m);
                    let _ = writeln!(
                        code,
                        "G1 X{:.3} Y{:.3}",
                        cx + CURVE_RADIUS_MM * angle.cos(),
                        cy + CURVE_RADIUS_MM * angle.sin()
                    );
                }
            }
        }
        code
    }
}

/// The settings measured when a G-code file is run as it is: Normal, then Fast's drawing speed
/// with the second `Geometry` float (B) at Normal's value, halfway, and at Fast's. The sixth float
/// (D) made no difference to drawing in the `zigzag` and `curve` runs, so it stays at Normal's.
pub(crate) fn file_profiles() -> Vec<Profile> {
    vec![
        profile(2.0, 3.0, None),
        profile(4.0, 3.0, Some((0, 0))),
        profile(4.0, 3.0, Some((500, 0))),
        profile(4.0, 3.0, Some((1000, 0))),
    ]
}

/// How many times as fast `second` is as `first`.
pub(crate) fn speedup(first: Duration, second: Duration) -> f64 {
    first.as_secs_f64() / second.as_secs_f64()
}

/// A G-code file reduced to what the harness's controller understands, and what is in it.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Program {
    /// Millimetres, absolute: travels, lines and arcs, the tool down and up.
    pub(crate) text: String,
    /// Moves made with the tool down.
    pub(crate) drawn_moves: usize,
    /// The same before any simplifying.
    pub(crate) source_moves: usize,
    /// Their length, arcs counted by their chord.
    pub(crate) drawn_mm: f64,
    /// The farthest the program reaches, millimetres from the top-left corner.
    pub(crate) max_x_mm: f64,
    pub(crate) max_y_mm: f64,
}

/// The mat's usable size, with a margin, in millimetres.
const SHEET_MM: f64 = 295.0;

impl Program {
    /// Keeps the `G0`-`G3`, `M3` and `M5` lines of compiled G-code (dropping comments, pauses,
    /// tool changes and the feed, which the harness's settings replace) and moves everything by
    /// (`dx`, `dy`) millimetres.
    ///
    /// # Errors
    ///
    /// A message when nothing is drawn or the program leaves the sheet.
    #[cfg(test)]
    pub(crate) fn from_gcode(source: &str, dx: f64, dy: f64) -> Result<Self, String> {
        Self::from_gcode_simplified(source, dx, dy, 0.0)
    }

    /// The program with every run of straight moves with the tool down replaced by the
    /// fewest that stay within `simplify_mm` of it (none for 0).
    ///
    /// # Errors
    ///
    /// A message when nothing is drawn or the program leaves the sheet.
    pub(crate) fn from_gcode_simplified(
        source: &str,
        dx: f64,
        dy: f64,
        simplify_mm: f64,
    ) -> Result<Self, String> {
        let mut out = Rendered::default();
        let (mut at_x, mut at_y) = (0.0_f64, 0.0_f64);
        let mut down = false;
        // Straight moves with the tool down, as positions on the sheet, waiting to be simplified.
        let mut run: Vec<(f64, f64)> = Vec::new();
        let (mut max_x, mut max_y) = (0.0_f64, 0.0_f64);
        for line in source.lines() {
            let line = line.split(';').next().unwrap_or_default();
            let mut words = line.split_whitespace();
            let Some(code) = words.next() else { continue };
            match code {
                "M3" | "M5" => {
                    out.flush(&mut run, simplify_mm);
                    down = code == "M3";
                    let _ = writeln!(out.text, "{code}");
                }
                "G0" | "G1" | "G2" | "G3" => {
                    let (mut nx, mut ny) = (at_x, at_y);
                    let (mut arc_i, mut arc_j) = (None, None);
                    for word in words {
                        let (letter, value) = word.split_at(1);
                        let Ok(number) = value.parse::<f64>() else {
                            return Err(format!("{line:?}: {word:?} is not a number"));
                        };
                        match letter {
                            "X" => nx = number,
                            "Y" => ny = number,
                            "I" => arc_i = Some(number),
                            "J" => arc_j = Some(number),
                            _ => {}
                        }
                    }
                    if code == "G1" && down {
                        if run.is_empty() {
                            run.push((at_x + dx, at_y + dy));
                        }
                        run.push((nx + dx, ny + dy));
                        out.source_moves += 1;
                    } else {
                        out.flush(&mut run, simplify_mm);
                        let _ = write!(out.text, "{code} X{:.3} Y{:.3}", nx + dx, ny + dy);
                        if let Some(i) = arc_i {
                            let _ = write!(out.text, " I{i:.3}");
                        }
                        if let Some(j) = arc_j {
                            let _ = write!(out.text, " J{j:.3}");
                        }
                        out.text.push('\n');
                        if down && code != "G0" {
                            out.source_moves += 1;
                            out.drawn_moves += 1;
                            out.drawn_mm += (nx - at_x).hypot(ny - at_y);
                        }
                    }
                    (at_x, at_y) = (nx, ny);
                    max_x = max_x.max(at_x + dx);
                    max_y = max_y.max(at_y + dy);
                }
                _ => {}
            }
        }
        out.flush(&mut run, simplify_mm);
        if out.drawn_moves == 0 {
            return Err("the program draws nothing".to_owned());
        }
        if max_x > SHEET_MM || max_y > SHEET_MM {
            return Err(format!(
                "the program reaches ({max_x:.0}, {max_y:.0}) mm, beyond the {SHEET_MM:.0} mm the sheet allows"
            ));
        }
        Ok(Self {
            text: format!("G21 G90\n{}", out.text),
            drawn_moves: out.drawn_moves,
            source_moves: out.source_moves,
            drawn_mm: out.drawn_mm,
            max_x_mm: max_x,
            max_y_mm: max_y,
        })
    }
}

/// The program as it is written out, and what has been counted so far.
#[derive(Debug, Default)]
struct Rendered {
    text: String,
    drawn_moves: usize,
    source_moves: usize,
    drawn_mm: f64,
}

impl Rendered {
    /// Writes the pending run of straight moves, simplified, and empties it.
    fn flush(&mut self, run: &mut Vec<(f64, f64)>, tolerance_mm: f64) {
        if run.len() >= 2 {
            let points = if tolerance_mm > 0.0 {
                simplify(run, tolerance_mm)
            } else {
                run.clone()
            };
            for pair in points.windows(2) {
                self.drawn_mm += (pair[1].0 - pair[0].0).hypot(pair[1].1 - pair[0].1);
            }
            for p in points.iter().skip(1) {
                let _ = writeln!(self.text, "G1 X{:.3} Y{:.3}", p.0, p.1);
                self.drawn_moves += 1;
            }
        }
        run.clear();
    }
}

/// Douglas-Peucker: the points that keep the path within `tolerance_mm` of the original, first and
/// last always kept. A path that returns to its start is judged against its farthest point, so its
/// shape survives.
pub(crate) fn simplify(points: &[(f64, f64)], tolerance_mm: f64) -> Vec<(f64, f64)> {
    if points.len() < 3 {
        return points.to_vec();
    }
    let mut keep = vec![false; points.len()];
    keep[0] = true;
    keep[points.len() - 1] = true;
    let mut spans = vec![(0, points.len() - 1)];
    while let Some((first, last)) = spans.pop() {
        let (a, b) = (points[first], points[last]);
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        let length = dx.hypot(dy);
        let mut farthest = (0.0_f64, first);
        for (i, p) in points.iter().enumerate().take(last).skip(first + 1) {
            let distance = if length == 0.0 {
                (p.0 - a.0).hypot(p.1 - a.1)
            } else {
                (dy * (p.0 - a.0) - dx * (p.1 - a.1)).abs() / length
            };
            if distance > farthest.0 {
                farthest = (distance, i);
            }
        }
        if farthest.0 > tolerance_mm {
            keep[farthest.1] = true;
            spans.push((first, farthest.1));
            spans.push((farthest.1, last));
        }
    }
    points
        .iter()
        .zip(keep)
        .filter_map(|(p, k)| k.then_some(*p))
        .collect()
}

/// The top edge of the band of the `index`th setting (0 is the first measured one); the warm-up
/// has its own band above them.
pub(crate) fn band_top_mm(workload: Workload, index: usize) -> f64 {
    // A handful of bands, so the index always converts exactly.
    FIRST_ROW_MM + workload.band_mm() * (1.0 + f64::from(u32::try_from(index).unwrap_or(u32::MAX)))
}

/// What a pair of timed runs says about one speed setting.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Timing {
    /// The path's length over the extra time of the large run, in millimetres per second.
    pub(crate) mm_per_second: f64,
    /// Seconds a run takes before any path is counted: setup, moving in and parking.
    pub(crate) overhead_seconds: f64,
}

/// The timing from a small run of `small_mm` of path and a large one of `large_mm`, or `None`
/// when the large run was not longer in both length and time (nothing can be said then).
pub(crate) fn timing(
    small: Duration,
    large: Duration,
    small_mm: f64,
    large_mm: f64,
) -> Option<Timing> {
    let extra_mm = large_mm - small_mm;
    let extra_s = large.as_secs_f64() - small.as_secs_f64();
    if extra_mm <= 0.0 || extra_s <= 0.0 {
        return None;
    }
    let mm_per_second = extra_mm / extra_s;
    Some(Timing {
        mm_per_second,
        overhead_seconds: small.as_secs_f64() - small_mm / mm_per_second,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn secs(s: u64) -> Duration {
        Duration::from_secs(s)
    }

    fn coordinates(code: &str) -> Vec<(f64, f64)> {
        code.lines()
            .filter(|l| l.starts_with("G0 ") || l.starts_with("G1 "))
            .map(|l| {
                let v: Vec<f64> = l[3..]
                    .split_whitespace()
                    .map(|w| w[1..].parse().expect("a number"))
                    .collect();
                (v[0], v[1])
            })
            .collect()
    }

    fn path_length(code: &str) -> f64 {
        let pts = coordinates(code);
        pts.windows(2)
            .map(|w| (w[1].0 - w[0].0).hypot(w[1].1 - w[0].1))
            .sum()
    }

    // ---- running a G-code file as it is ----

    const COMPILED: &str = "; plotter-studio 0.1.0\nG21\nG90\nM0 (Load pen: Fine liner into Clamp A)\n\
T1 M6\nG0 X10.000 Y20.000\nM3 S120\nG1 X30.000 Y20.000 F6000\nG1 X30.000 Y21.000\n\
G3 X40.000 Y31.000 I5.000 J0.000\nM5\nM0 (Unload mat)\nM2\n";

    #[test]
    fn a_compiled_program_keeps_only_what_the_machine_draws_and_is_moved_by_the_offset() {
        let p = Program::from_gcode(COMPILED, 10.0, 5.0).expect("a valid program");
        assert_eq!(
            p.text,
            "G21 G90\nG0 X20.000 Y25.000\nM3\nG1 X40.000 Y25.000\nG1 X40.000 Y26.000\n\
G3 X50.000 Y36.000 I5.000 J0.000\nM5\n"
        );
    }

    #[test]
    fn a_program_knows_how_many_moves_it_draws_and_how_far() {
        // A 20 mm line, a 1 mm step and an arc counted by its chord (10 mm, 10 mm): 3 drawn moves
        // and 20 + 1 + 14.14 mm of them; the first move is a travel.
        let p = Program::from_gcode(COMPILED, 0.0, 0.0).unwrap();
        assert_eq!(p.drawn_moves, 3);
        assert!((p.drawn_mm - (20.0 + 1.0 + 200.0_f64.sqrt())).abs() < 1e-6);
        assert!((p.max_x_mm - 40.0).abs() < 1e-9 && (p.max_y_mm - 31.0).abs() < 1e-9);
    }

    #[test]
    fn a_program_that_leaves_the_sheet_is_refused() {
        let far = "G0 X10 Y10\nM3\nG1 X296 Y10\nM5\n";
        assert!(Program::from_gcode(far, 0.0, 0.0).is_err());
        assert!(Program::from_gcode(far, -10.0, 0.0).is_ok());
    }

    #[test]
    fn a_program_with_nothing_to_draw_is_refused() {
        assert!(Program::from_gcode("G21\nG90\nM0 (nothing)\nM2\n", 0.0, 0.0).is_err());
        assert!(Program::from_gcode("G0 X1 Y1\nG0 X2 Y2\n", 0.0, 0.0).is_err());
    }

    #[test]
    fn the_file_settings_start_with_normal_and_then_vary_only_the_second_float() {
        let profiles = file_profiles();
        assert_eq!(profiles[0], profile(2.0, 3.0, None));
        assert_eq!(
            profiles[1..],
            [
                profile(4.0, 3.0, Some((0, 0))),
                profile(4.0, 3.0, Some((500, 0))),
                profile(4.0, 3.0, Some((1000, 0))),
            ]
        );
    }

    #[test]
    fn a_setting_is_compared_with_the_first_as_a_percentage() {
        // 180 s then 120 s: a third less time, which is 1.5 times as fast.
        assert!((speedup(secs(180), secs(120)) - 1.5).abs() < 1e-9);
        assert!((speedup(secs(180), secs(180)) - 1.0).abs() < 1e-9);
        assert!((speedup(secs(180), secs(240)) - 0.75).abs() < 1e-9);
    }

    // ---- simplifying the drawn path ----

    #[test]
    fn a_point_on_the_line_between_its_neighbours_is_dropped() {
        let pts = [(0.0, 0.0), (5.0, 0.0), (10.0, 0.0)];
        assert_eq!(simplify(&pts, 0.01), vec![(0.0, 0.0), (10.0, 0.0)]);
    }

    #[test]
    fn a_point_within_the_tolerance_of_the_line_is_dropped_and_one_beyond_it_is_kept() {
        // 1 um off the line is inside 10 um; 20 um off is outside.
        let near = [(0.0, 0.0), (5.0, 0.001), (10.0, 0.0)];
        assert_eq!(simplify(&near, 0.01), vec![(0.0, 0.0), (10.0, 0.0)]);
        let far = [(0.0, 0.0), (5.0, 0.02), (10.0, 0.0)];
        assert_eq!(simplify(&far, 0.01), far.to_vec());
    }

    #[test]
    fn a_corner_is_always_kept() {
        let pts = [(0.0, 0.0), (5.0, 0.0), (5.0, 5.0)];
        assert_eq!(simplify(&pts, 0.01), pts.to_vec());
        // Even a very small one, if it is larger than the tolerance: a 0.8 mm hairpin step.
        let hairpin = [(0.0, 0.0), (30.0, 0.0), (30.0, 0.8), (0.0, 0.8)];
        assert_eq!(simplify(&hairpin, 0.01), hairpin.to_vec());
    }

    #[test]
    fn a_run_of_nearly_straight_points_becomes_one_vector() {
        // Eleven points along a line with 2 um of wobble: one vector from end to end.
        let pts: Vec<(f64, f64)> = (0..=10)
            .map(|i| (f64::from(i), if i % 2 == 0 { 0.0 } else { 0.002 }))
            .collect();
        assert_eq!(simplify(&pts, 0.01), vec![(0.0, 0.0), (10.0, 0.0)]);
    }

    #[test]
    fn a_path_that_returns_to_its_start_keeps_its_shape() {
        // A closed square with extra points on its edges: four corners and the return remain.
        let square = [
            (0.0, 0.0),
            (5.0, 0.0),
            (10.0, 0.0),
            (10.0, 5.0),
            (10.0, 10.0),
            (5.0, 10.0),
            (0.0, 10.0),
            (0.0, 5.0),
            (0.0, 0.0),
        ];
        assert_eq!(
            simplify(&square, 0.01),
            vec![
                (0.0, 0.0),
                (10.0, 0.0),
                (10.0, 10.0),
                (0.0, 10.0),
                (0.0, 0.0)
            ]
        );
    }

    #[test]
    fn two_points_or_fewer_are_left_alone() {
        assert_eq!(simplify(&[], 0.01), vec![]);
        assert_eq!(simplify(&[(1.0, 1.0)], 0.01), vec![(1.0, 1.0)]);
        assert_eq!(
            simplify(&[(1.0, 1.0), (2.0, 2.0)], 0.01),
            vec![(1.0, 1.0), (2.0, 2.0)]
        );
    }

    #[test]
    fn a_program_is_simplified_stroke_by_stroke_and_reports_both_counts() {
        // A stroke of three collinear lines (5 mm each) and a corner: 4 vectors, 3 after.
        let src = "G0 X10 Y10\nM3\nG1 X15 Y10\nG1 X20 Y10\nG1 X25 Y10\nG1 X25 Y15\nM5\n";
        let p = Program::from_gcode_simplified(src, 0.0, 0.0, 0.01).unwrap();
        assert_eq!(p.source_moves, 4);
        assert_eq!(p.drawn_moves, 2);
        assert!((p.drawn_mm - 20.0).abs() < 1e-9);
        assert_eq!(
            p.text,
            "G21 G90\nG0 X10.000 Y10.000\nM3\nG1 X25.000 Y10.000\nG1 X25.000 Y15.000\nM5\n"
        );
    }

    #[test]
    fn simplifying_by_nothing_changes_nothing() {
        let src = "G0 X10 Y10\nM3\nG1 X15 Y10\nG1 X20 Y10\nM5\n";
        let plain = Program::from_gcode(src, 0.0, 0.0).unwrap();
        let none = Program::from_gcode_simplified(src, 0.0, 0.0, 0.0).unwrap();
        assert_eq!(plain, none);
        assert_eq!(plain.drawn_moves, 2);
    }

    #[test]
    fn an_arc_ends_a_run_and_is_kept_as_it_is() {
        let src = "G0 X10 Y10\nM3\nG1 X15 Y10\nG1 X20 Y10\nG3 X25 Y15 I0 J5\nG1 X30 Y15\nG1 X35 Y15\nM5\n";
        let p = Program::from_gcode_simplified(src, 0.0, 0.0, 0.01).unwrap();
        assert!(p.text.contains("G3 X25.000 Y15.000 I0.000 J5.000"));
        // 5 drawn moves before; the two collinear lines on each side become one each.
        assert_eq!(p.source_moves, 5);
        assert_eq!(p.drawn_moves, 3);
    }

    // ---- the rounded zigzag: the same rows, with each hairpin turned as a semicircle ----

    #[test]
    fn a_two_chord_turn_goes_out_to_the_side_and_back_to_the_next_row() {
        // Rows 0.8 mm apart are joined by a semicircle of radius 0.4 mm. With two chords its middle
        // point is level with the centre and 0.4 mm beyond the end of the row.
        assert_eq!(
            Workload::Rounded(2).gcode(10.0, 20.0, 2),
            "G21 G90\nG0 X10.000 Y20.000\nG1 X40.000 Y20.000\nG1 X40.400 Y20.400\nG1 X40.000 Y20.800\nG1 X10.000 Y20.800\n"
        );
    }

    #[test]
    fn the_turn_at_the_left_goes_out_to_the_left() {
        // Three rows: the second turn is at the left end, so its middle point is at x = 9.6.
        let code = Workload::Rounded(2).gcode(10.0, 20.0, 3);
        assert!(code.contains("G1 X9.600 Y21.200"), "{code}");
        assert!(code.ends_with("G1 X40.000 Y21.600\n"), "{code}");
    }

    #[test]
    fn the_length_of_a_rounded_zigzag_is_its_rows_plus_its_chords() {
        // Two rows of 30 mm and one turn of two chords of hypot(0.4, 0.4) = 0.5657 mm each.
        let want = 60.0 + 2.0 * 0.4_f64.hypot(0.4);
        assert!((Workload::Rounded(2).length_mm(2) - want).abs() < 1e-9);
        // More chords follow the semicircle more closely, towards 0.4 pi = 1.2566 mm.
        let fine = Workload::Rounded(24).length_mm(2) - 60.0;
        assert!((fine - 0.4 * std::f64::consts::PI).abs() < 0.005, "{fine}");
    }

    #[test]
    fn a_generated_rounded_zigzag_has_the_length_it_claims() {
        for turn in [2, 6, 12] {
            let code = Workload::Rounded(turn).gcode(10.0, 20.0, 10);
            assert!(
                (path_length(&code) - Workload::Rounded(turn).length_mm(10)).abs() < 0.05,
                "{turn}"
            );
        }
    }

    #[test]
    fn a_rounded_zigzag_has_as_many_vectors_as_its_turns_need() {
        // Each of the 9 turns of a 10-row figure is `turn` vectors, besides the 10 rows.
        let code = Workload::Rounded(6).gcode(10.0, 20.0, 10);
        assert_eq!(code.matches("G1 ").count(), 10 + 9 * 6);
    }

    #[test]
    fn the_rounded_workload_is_chosen_by_name_with_six_chords_unless_told_otherwise() {
        assert_eq!(Workload::parse("rounded"), Some(Workload::Rounded(6)));
        assert_eq!(Workload::Rounded(6).name(), "rounded");
        assert_eq!(
            Workload::Rounded(6).default_sizes(),
            Workload::Zigzag.default_sizes()
        );
        assert_eq!(
            Workload::Rounded(6).default_profiles(),
            Workload::Zigzag.default_profiles()
        );
    }

    #[test]
    fn four_hundred_extra_millimetres_in_forty_seconds_are_ten_millimetres_a_second() {
        // 20 s for 100 mm and 60 s for 500 mm: 40 s for the extra 400 mm, so 10 mm/s, and the
        // first 100 mm take 10 s of the short run's 20, leaving 10 s of overhead.
        let t = timing(secs(20), secs(60), 100.0, 500.0).expect("a longer run");
        assert!((t.mm_per_second - 10.0).abs() < 1e-9);
        assert!((t.overhead_seconds - 10.0).abs() < 1e-9);
    }

    #[test]
    fn a_run_that_is_not_longer_says_nothing() {
        assert_eq!(timing(secs(30), secs(30), 100.0, 500.0), None);
        assert_eq!(timing(secs(30), secs(20), 100.0, 500.0), None);
        assert_eq!(timing(secs(20), secs(60), 500.0, 500.0), None);
        assert_eq!(timing(secs(20), secs(60), 500.0, 100.0), None);
    }

    #[test]
    fn a_line_is_drawn_out_and_back() {
        assert_eq!(
            Workload::Line.gcode(10.0, 25.0, 3),
            "G21 G90\nG0 X10.000 Y25.000\nG1 X110.000 Y25.000\nG1 X10.000 Y25.000\nG1 X110.000 Y25.000\n"
        );
        assert!((Workload::Line.length_mm(5) - 500.0).abs() < 1e-9);
    }

    #[test]
    fn a_zigzag_alternates_rows_and_steps_down_between_them() {
        // Two 30 mm rows 0.8 mm apart, joined at the right: out, down, back.
        assert_eq!(
            Workload::Zigzag.gcode(10.0, 20.0, 2),
            "G21 G90\nG0 X10.000 Y20.000\nG1 X40.000 Y20.000\nG1 X40.000 Y20.800\nG1 X10.000 Y20.800\n"
        );
        // 2 rows of 30 and 1 step of 0.8; 10 rows are 300 + 9 * 0.8.
        assert!((Workload::Zigzag.length_mm(2) - 60.8).abs() < 1e-9);
        assert!((Workload::Zigzag.length_mm(10) - 307.2).abs() < 1e-9);
    }

    #[test]
    fn a_generated_zigzag_has_the_length_it_claims() {
        let code = Workload::Zigzag.gcode(10.0, 20.0, 10);
        assert!((path_length(&code) - Workload::Zigzag.length_mm(10)).abs() < 1e-6);
    }

    #[test]
    fn a_curve_is_472_tiny_segments_a_loop_that_close_on_themselves() {
        // 2 pi * 15 mm = 94.25 mm, in 0.2 mm segments: 471.2, so 472.
        let one = Workload::Curve.gcode(10.0, 20.0, 1);
        assert_eq!(one.matches("G1 ").count(), 472);
        assert_eq!(
            Workload::Curve.gcode(10.0, 20.0, 4).matches("G1 ").count(),
            4 * 472
        );
        let pts = coordinates(&one);
        assert_eq!(pts.first(), pts.last(), "a loop ends where it started");
        // Every coordinate is rounded to a micrometre, so the sum is close but not exact.
        assert!((path_length(&one) - Workload::Curve.length_mm(1)).abs() < 0.1);
        assert!((Workload::Curve.length_mm(1) - 94.2).abs() < 0.1);
    }

    #[test]
    fn every_figure_stays_inside_its_band_and_the_sheet() {
        for workload in [
            Workload::Line,
            Workload::Zigzag,
            Workload::Curve,
            Workload::Rounded(6),
        ] {
            let (_, large) = workload.default_sizes();
            assert!(large <= workload.max_size());
            let code = workload.gcode(SMALL_X_MM + workload.large_offset_mm(), 0.0, large);
            let pts = coordinates(&code);
            let (low, high) = pts.iter().fold((f64::MAX, f64::MIN), |(lo, hi), p| {
                (lo.min(p.1), hi.max(p.1))
            });
            assert!(
                low >= -1e-9 && high <= workload.band_mm(),
                "{workload:?}: {low}..{high}"
            );
            let right = pts.iter().map(|p| p.0).fold(f64::MIN, f64::max);
            assert!(right <= 290.0, "{workload:?}: reaches x = {right}");
        }
    }

    #[test]
    fn bands_follow_each_other_from_the_first_row() {
        assert!((band_top_mm(Workload::Zigzag, 0) - 52.0).abs() < 1e-9);
        assert!((band_top_mm(Workload::Zigzag, 1) - 94.0).abs() < 1e-9);
        assert!((band_top_mm(Workload::Line, 2) - 55.0).abs() < 1e-9);
    }

    #[test]
    fn profiles_parse_with_speeds_and_optional_corner_floats() {
        assert_eq!(
            parse_profiles("2:3, 4:3:0:1000").unwrap(),
            vec![profile(2.0, 3.0, None), profile(4.0, 3.0, Some((0, 1000)))]
        );
    }

    #[test]
    fn profiles_that_leave_the_range_design_space_has_sent_are_refused() {
        for bad in [
            "0.5:3",
            "5:3",
            "2:6",
            "2:0.5",
            "2",
            "2:3:1001:0",
            "2:3:5",
            "x:3",
            "2:3:0:x",
            "",
        ] {
            assert!(parse_profiles(bad).is_err(), "{bad:?} should be refused");
        }
    }

    #[test]
    fn the_captured_profiles_are_what_design_space_sends() {
        assert_eq!(profile(2.0, 3.0, None).speed(), Speed::NORMAL);
        assert_eq!(profile(4.0, 5.0, Some((1000, 1000))).speed(), Speed::FAST);
    }

    #[test]
    fn the_default_settings_are_all_valid_and_fit_the_sheet() {
        for workload in [
            Workload::Line,
            Workload::Zigzag,
            Workload::Curve,
            Workload::Rounded(6),
        ] {
            let profiles = workload.default_profiles();
            let count = f64::from(u32::try_from(profiles.len()).unwrap());
            let bottom = FIRST_ROW_MM + workload.band_mm() * (1.0 + count);
            assert!(
                bottom <= LAST_ROW_LIMIT_MM,
                "{workload:?} needs {bottom} mm"
            );
        }
    }
}
