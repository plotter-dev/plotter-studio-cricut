//! G-code → cut paths in machine inches.
//!
//! Parsing is done by the [`gcode`](https://crates.io/crates/gcode) crate and arc geometry by
//! [`lyon_geom`](https://crates.io/crates/lyon_geom). This module only *interprets* the
//! parsed blocks: modal state, units, coordinate mapping and blade up/down.
//!
//! Supported subset:
//! - `G0` rapid (blade up), `G1` linear, `G2`/`G3` arcs (`I`/`J` centre offsets or `R`), XY plane
//! - `G20` inches / `G21` millimetres (default mm), `G90` absolute / `G91` relative (default absolute)
//! - Blade state from `M3`/`M4` (down) and `M5` (up), or from `Z` (≤ `z_cut_threshold` = down)
//! - `F`, `S`, `T`, `G4`, `G17`, `G54`, `G64`, `G94`, `M0`–`M2`, `M30` etc. are accepted and
//!   ignored (with a warning where they would change the result)
//!
//! Not supported: `*NN` line checksums (rejected by the parser) and `#` variables.
//!
//! A `G1`/`G2`/`G3` move cuts when the blade is down. The blade starts *down*, so a plain
//! `G0`/`G1` program works without any `M3` or `Z` words.

use gcode::{Block, Code, Value};
use lyon_geom::{Angle, Arc, ArcFlags, SvgArc, point, vector};

use crate::error::{Error, Result};

/// A point in machine inches (top-left origin, Y towards the user, mat offset applied).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point {
    pub x: f64,
    pub y: f64,
}

impl Point {
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }
}

/// A connected cut: the blade travels (up) to `points[0]`, then cuts through the rest.
#[derive(Debug, Clone, PartialEq)]
pub struct Polyline {
    pub points: Vec<Point>,
}

/// Where the G-code coordinate origin sits on the sheet.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Origin {
    /// Usual CNC/CAM convention: origin at the sheet's bottom-left, +Y up (away from the user).
    /// Y is flipped using the sheet height, in inches.
    BottomLeft { sheet_height_in: f64 },
    /// Machine convention: origin at the sheet's top-left, +Y down (towards the user).
    TopLeft,
}

/// Options controlling how G-code maps to the machine.
#[derive(Debug, Clone, PartialEq)]
pub struct GcodeOptions {
    pub origin: Origin,
    /// Machine-inch offset of the sheet's top-left corner on the mat. Default (0.25, 0.25), a
    /// deliberate safe-travel margin.
    pub mat_offset_in: (f64, f64),
    /// Max deviation, in inches, when flattening arcs into line segments.
    pub arc_tolerance_in: f64,
    /// `Z` at or below this (in the program's units) lowers the blade.
    pub z_cut_threshold: f64,
    /// Largest allowed machine coordinate `(x, y)` in inches. Cut points outside
    /// `0..=extent` are rejected. Default: a 12×12 in mat (12.25, 12.25 with the offset);
    /// use `(12.25, 24.25)` for a 12×24 mat.
    pub mat_extent_in: (f64, f64),
}

impl Default for GcodeOptions {
    fn default() -> Self {
        Self {
            origin: Origin::BottomLeft {
                sheet_height_in: 12.0,
            },
            mat_offset_in: (0.25, 0.25),
            arc_tolerance_in: 0.002,
            z_cut_threshold: 0.0,
            mat_extent_in: (12.25, 12.25),
        }
    }
}

/// The result of parsing a program.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct CutPlan {
    pub polylines: Vec<Polyline>,
    /// Non-fatal notes, e.g. ignored commands that may matter.
    pub warnings: Vec<String>,
}

impl CutPlan {
    /// Machine-inch bounding box `(min, max)` of all cut points, if any.
    pub fn bounds(&self) -> Option<(Point, Point)> {
        let mut it = self.polylines.iter().flat_map(|p| p.points.iter());
        let first = *it.next()?;
        Some(it.fold((first, first), |(lo, hi), p| {
            (
                Point::new(lo.x.min(p.x), lo.y.min(p.y)),
                Point::new(hi.x.max(p.x), hi.y.max(p.y)),
            )
        }))
    }

    /// Total number of vector entries the plan will send.
    pub fn vector_count(&self) -> usize {
        self.polylines.iter().map(|p| p.points.len()).sum()
    }
}

/// Parse G-code with default options.
pub fn parse(src: &str) -> Result<CutPlan> {
    parse_with(src, &GcodeOptions::default())
}

/// Parse G-code into machine-inch polylines.
pub fn parse_with(src: &str, opts: &GcodeOptions) -> Result<CutPlan> {
    let program = gcode::parse(src).map_err(|diags| {
        let first = diags.iter().next();
        Error::Gcode {
            line: first.map_or(0, |d| d.span.line + 1),
            message: first.map_or_else(|| "parse error".into(), |d| d.kind.to_string()),
        }
    })?;

    let mut st = State::new(opts);
    for block in &program.blocks {
        st.line_no = block.span.line + 1;
        let words = st.words(block)?;
        st.exec(&words)?;
    }
    st.finish_polyline();
    Ok(CutPlan {
        polylines: st.out,
        warnings: st.warnings,
    })
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Motion {
    Rapid,
    Linear,
    ArcCw,
    ArcCcw,
}

/// One block flattened into G/M codes (`letter`, major, minor) and parameter words.
struct Words {
    codes: Vec<(char, u32, u32)>,
    params: Vec<(char, f64)>,
}

impl Words {
    fn get(&self, letter: char) -> Option<f64> {
        self.params
            .iter()
            .rev()
            .find(|(l, _)| *l == letter)
            .map(|(_, v)| *v)
    }
}

struct State<'a> {
    opts: &'a GcodeOptions,
    line_no: usize,
    inches: bool,
    absolute: bool,
    motion: Option<Motion>,
    blade_down: bool,
    /// Current position in program coordinates, inches.
    pos: (f64, f64),
    current: Vec<Point>,
    out: Vec<Polyline>,
    warnings: Vec<String>,
}

impl<'a> State<'a> {
    fn new(opts: &'a GcodeOptions) -> Self {
        Self {
            opts,
            line_no: 0,
            inches: false,
            absolute: true,
            motion: None,
            blade_down: true,
            pos: (0.0, 0.0),
            current: Vec::new(),
            out: Vec::new(),
            warnings: Vec::new(),
        }
    }

    fn err(&self, message: impl Into<String>) -> Error {
        Error::Gcode {
            line: self.line_no,
            message: message.into(),
        }
    }

    fn warn(&mut self, message: impl Into<String>) {
        self.warnings
            .push(format!("line {}: {}", self.line_no, message.into()));
    }

    /// Flatten a parsed block. The parser attaches parameters to the preceding G/M code;
    /// G-code semantics are per block, so they are pooled here.
    fn words(&self, block: &Block) -> Result<Words> {
        let mut w = Words {
            codes: Vec::new(),
            params: Vec::new(),
        };
        let mut push_param = |letter: char, value: &Value| -> Result<()> {
            let v = match value {
                // The parser stores f32; go through its shortest decimal form so that e.g.
                // `25.4` becomes exactly 25.4_f64 rather than 25.399999618…
                Value::Literal(f) => f.to_string().parse::<f64>().unwrap_or(f64::from(*f)),
                _ => return Err(self.err(format!("{letter}: variables are not supported"))),
            };
            w.params.push((letter.to_ascii_uppercase(), v));
            Ok(())
        };
        for code in &block.codes {
            let (letter, number, args) = match code {
                Code::General(g) => ('G', g.number, &g.args),
                Code::Miscellaneous(m) => ('M', m.number, &m.args),
                Code::ToolChange(t) => ('T', t.number, &t.args),
                _ => continue,
            };
            let minor = number.minor().map_or(0, std::num::NonZero::get);
            w.codes.push((letter, number.major(), minor));
            for a in args {
                push_param(a.letter, &a.value)?;
            }
        }
        for a in &block.word_addresses {
            push_param(a.letter, &a.value)?;
        }
        Ok(w)
    }

    fn scale(&self) -> f64 {
        if self.inches { 1.0 } else { 1.0 / 25.4 }
    }

    fn to_machine(&self, (x, y): (f64, f64)) -> Point {
        let (ox, oy) = self.opts.mat_offset_in;
        let my = match self.opts.origin {
            Origin::BottomLeft { sheet_height_in } => sheet_height_in - y,
            Origin::TopLeft => y,
        };
        Point::new(ox + x, oy + my)
    }

    fn exec(&mut self, words: &Words) -> Result<()> {
        let mut new_motion = None;

        // Modal / non-motion codes first, so `G21 G1 X…` on one block works.
        for &(letter, major, minor) in &words.codes {
            match (letter, major, minor) {
                ('G', 0, 0) => new_motion = Some(Motion::Rapid),
                ('G', 1, 0) => new_motion = Some(Motion::Linear),
                ('G', 2, 0) => new_motion = Some(Motion::ArcCw),
                ('G', 3, 0) => new_motion = Some(Motion::ArcCcw),
                ('G', 20, 0) => self.inches = true,
                ('G', 21, 0) => self.inches = false,
                ('G', 90, 0) => self.absolute = true,
                ('G', 91, 0) => self.absolute = false,
                // Accepted and ignored: no effect on a 2-D cut path (dwell, plane XY, offsets
                // off, work offsets, path modes, feed mode; program stop/end, tool change).
                ('G', 4 | 17 | 40 | 49 | 54..=59 | 61 | 64 | 80 | 94, 0)
                | ('M', 0 | 1 | 2 | 6 | 30, 0)
                | ('T', _, _) => {}
                ('G', 18 | 19, 0) => return Err(self.err("only the XY plane (G17) is supported")),
                ('G', 28, 0) => self.warn("G28 (home) ignored"),
                ('G', 92, 0) => return Err(self.err("G92 coordinate offsets are not supported")),
                ('M', 3 | 4, 0) => self.set_blade(true),
                ('M', 5, 0) => self.set_blade(false),
                (l, n, m) => {
                    let sub = if m == 0 {
                        String::new()
                    } else {
                        format!(".{m}")
                    };
                    self.warn(format!("unsupported {l}{n}{sub} ignored"));
                }
            }
        }
        if new_motion.is_some() {
            self.motion = new_motion;
        }

        if let Some(z) = words.get('Z') {
            self.set_blade(z <= self.opts.z_cut_threshold);
        }

        let x = words.get('X');
        let y = words.get('Y');
        if x.is_none() && y.is_none() {
            return Ok(());
        }
        let Some(motion) = self.motion else {
            return Err(self.err("coordinates given before any G0/G1/G2/G3"));
        };

        let s = self.scale();
        let target = if self.absolute {
            (
                x.map_or(self.pos.0, |v| v * s),
                y.map_or(self.pos.1, |v| v * s),
            )
        } else {
            (
                self.pos.0 + x.unwrap_or(0.0) * s,
                self.pos.1 + y.unwrap_or(0.0) * s,
            )
        };

        match motion {
            Motion::Linear if self.blade_down => self.cut_to(target)?,
            Motion::ArcCw | Motion::ArcCcw if self.blade_down => {
                for p in self.arc_points(target, motion == Motion::ArcCw, words)? {
                    self.cut_to(p)?;
                }
            }
            // Rapids, and any move with the blade up, are travel.
            Motion::Rapid | Motion::Linear | Motion::ArcCw | Motion::ArcCcw => self.travel(target),
        }
        Ok(())
    }

    fn set_blade(&mut self, down: bool) {
        if self.blade_down && !down {
            self.finish_polyline();
        }
        self.blade_down = down;
    }

    fn travel(&mut self, target: (f64, f64)) {
        self.finish_polyline();
        self.pos = target;
    }

    fn cut_to(&mut self, target: (f64, f64)) -> Result<()> {
        if self.current.is_empty() {
            let start = self.checked(self.to_machine(self.pos))?;
            self.current.push(start);
        }
        self.pos = target;
        let p = self.checked(self.to_machine(target))?;
        if self.current.last() != Some(&p) {
            self.current.push(p);
        }
        Ok(())
    }

    fn checked(&self, p: Point) -> Result<Point> {
        let (mx, my) = self.opts.mat_extent_in;
        let ok = |v: f64, max: f64| (-1e-9..=max + 1e-9).contains(&v);
        if ok(p.x, mx) && ok(p.y, my) {
            Ok(p)
        } else {
            Err(self.err(format!(
                "cut point ({:.3}, {:.3}) in is outside the mat (0..{mx}, 0..{my}); check units and origin",
                p.x, p.y
            )))
        }
    }

    fn finish_polyline(&mut self) {
        let pts = std::mem::take(&mut self.current);
        if pts.len() >= 2 {
            self.out.push(Polyline { points: pts });
        }
    }

    /// Flatten an XY arc (program coordinates, inches, +Y up) into points after the start.
    #[expect(
        clippy::many_single_char_names,
        reason = "s/i/j/r/c mirror the G-code words and arc notation"
    )]
    fn arc_points(
        &self,
        end: (f64, f64),
        clockwise: bool,
        words: &Words,
    ) -> Result<Vec<(f64, f64)>> {
        let s = self.scale();
        let start = self.pos;
        let (i, j, r) = (words.get('I'), words.get('J'), words.get('R'));

        let arc: Arc<f64> = if i.is_some() || j.is_some() {
            let c = (
                start.0 + i.unwrap_or(0.0) * s,
                start.1 + j.unwrap_or(0.0) * s,
            );
            let radius = (start.0 - c.0).hypot(start.1 - c.1);
            if radius < 1e-9 {
                return Ok(vec![end]);
            }
            // G-code sweep rule: always the given direction; equal endpoints = full circle.
            let a0 = (start.1 - c.1).atan2(start.0 - c.0);
            let a1 = (end.1 - c.1).atan2(end.0 - c.0);
            let tau = std::f64::consts::TAU;
            let mut sweep = a1 - a0;
            if clockwise && sweep >= -1e-12 {
                sweep -= tau;
            } else if !clockwise && sweep <= 1e-12 {
                sweep += tau;
            }
            Arc {
                center: point(c.0, c.1),
                radii: vector(radius, radius),
                start_angle: Angle::radians(a0),
                sweep_angle: Angle::radians(sweep),
                x_rotation: Angle::zero(),
            }
        } else if let Some(r) = r {
            let r_in = r * s;
            let chord = (end.0 - start.0).hypot(end.1 - start.1);
            // lyon silently enlarges a too-small radius (SVG rules); G-code treats it as an error.
            if chord > 2.0 * r_in.abs() + 1e-6 {
                return Err(self.err("arc radius too small for its endpoints"));
            }
            let svg = SvgArc {
                from: point(start.0, start.1),
                to: point(end.0, end.1),
                radii: vector(r_in.abs(), r_in.abs()),
                x_rotation: Angle::zero(),
                // Negative R selects the long way round; positive sweep = counter-clockwise (+Y up).
                flags: ArcFlags {
                    large_arc: r_in < 0.0,
                    sweep: !clockwise,
                },
            };
            if svg.is_straight_line() {
                return Ok(vec![end]);
            }
            svg.to_arc()
        } else {
            return Err(self.err("arc needs I/J or R"));
        };

        let mut pts: Vec<(f64, f64)> = arc
            .flattened(self.opts.arc_tolerance_in.max(1e-6))
            .map(|p| (p.x, p.y))
            .collect();
        // Land exactly on the programmed end point.
        match pts.last_mut() {
            Some(last) => *last = end,
            None => pts.push(end),
        }
        Ok(pts)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn top_left() -> GcodeOptions {
        GcodeOptions {
            origin: Origin::TopLeft,
            mat_offset_in: (0.0, 0.0),
            ..Default::default()
        }
    }

    fn approx(a: Point, x: f64, y: f64) {
        assert!(
            (a.x - x).abs() < 1e-9 && (a.y - y).abs() < 1e-9,
            "{a:?} != ({x}, {y})"
        );
    }

    #[test]
    fn square_in_mm_default_origin() {
        let plan = parse("G21 G90\nG0 X0 Y0\nG1 X25.4 Y0\nG1 X25.4 Y25.4\nG1 X0 Y25.4\nG1 X0 Y0\n")
            .unwrap();
        assert_eq!(plan.polylines.len(), 1);
        let p = &plan.polylines[0].points;
        assert_eq!(p.len(), 5);
        // bottom-left origin on a 12in sheet, +0.25in offset
        approx(p[0], 0.25, 12.25);
        approx(p[1], 1.25, 12.25);
        approx(p[2], 1.25, 11.25);
    }

    #[test]
    fn inches_relative_and_comments() {
        let src = "%\nN10 G20 (inches) ; comment\nG91\ng0 x1 y1\nG1X1\nG1 Y1\n%\n";
        let plan = parse_with(src, &top_left()).unwrap();
        let p = &plan.polylines[0].points;
        approx(p[0], 1.0, 1.0);
        approx(p[1], 2.0, 1.0);
        approx(p[2], 2.0, 2.0);
    }

    #[test]
    fn modal_motion_and_spindle() {
        let src = "G20\nM5\nG1 X1 Y1\nM3\nG1 X2\nX3\nM5\nG1 X4\nM3\nG1 Y2";
        let plan = parse_with(src, &top_left()).unwrap();
        assert_eq!(plan.polylines.len(), 2);
        assert_eq!(plan.polylines[0].points.len(), 3);
        approx(plan.polylines[1].points[0], 4.0, 1.0);
    }

    #[test]
    fn z_controls_blade() {
        let src = "G20\nG0 Z0.1\nG0 X1 Y1\nG1 Z-0.01\nG1 X2\nG0 Z0.1\nG1 X3";
        let plan = parse_with(src, &top_left()).unwrap();
        assert_eq!(plan.polylines.len(), 1);
        assert_eq!(plan.polylines[0].points.len(), 2);
    }

    #[test]
    fn full_circle_ij() {
        // All arc tests are centred on (2, 2) so they stay on the mat.
        let src = "G20\nG0 X3 Y2\nG2 X3 Y2 I-1 J0";
        let opts = GcodeOptions {
            arc_tolerance_in: 0.001,
            ..top_left()
        };
        let plan = parse_with(src, &opts).unwrap();
        let pts = &plan.polylines[0].points;
        assert!(pts.len() > 20);
        for p in pts {
            assert!(((p.x - 2.0).hypot(p.y - 2.0) - 1.0).abs() < 1e-9);
        }
        approx(*pts.last().unwrap(), 3.0, 2.0);
        // clockwise from (3,2) about (2,2): y decreases first in program space
        assert!(pts[1].y < 2.0);
    }

    #[test]
    fn quarter_arc_radius_form() {
        let src = "G20\nG0 X3 Y2\nG3 X2 Y3 R1";
        let plan = parse_with(src, &top_left()).unwrap();
        for p in &plan.polylines[0].points {
            assert!(((p.x - 2.0).hypot(p.y - 2.0) - 1.0).abs() < 1e-6, "{p:?}");
        }
    }

    #[test]
    fn radius_arcs_pick_correct_centre() {
        // CW from (2,3) to (3,2) with R1: the short arc, centred on (2,2)
        let plan = parse_with("G20\nG0 X2 Y3\nG2 X3 Y2 R1", &top_left()).unwrap();
        for p in &plan.polylines[0].points {
            assert!(((p.x - 2.0).hypot(p.y - 2.0) - 1.0).abs() < 1e-6, "{p:?}");
        }
        // negative R selects the long way round, centred on (3,3)
        let plan = parse_with("G20\nG0 X2 Y3\nG2 X3 Y2 R-1", &top_left()).unwrap();
        for p in &plan.polylines[0].points {
            assert!(((p.x - 3.0).hypot(p.y - 3.0) - 1.0).abs() < 1e-6, "{p:?}");
        }
    }

    #[test]
    fn errors_and_warnings() {
        assert!(matches!(parse("X1 Y1"), Err(Error::Gcode { line: 1, .. })));
        assert!(matches!(
            parse("G0 X1\nG1 Xabc"),
            Err(Error::Gcode { line: 2, .. })
        ));
        assert!(parse("G18").is_err());
        // line checksums and variables are rejected, with the right line number
        assert!(matches!(
            parse("G21\nG1 X1*55"),
            Err(Error::Gcode { line: 2, .. })
        ));
        assert!(matches!(parse("G0 X#1"), Err(Error::Gcode { .. })));
        let plan = parse("G28\nM8").unwrap();
        assert_eq!(plan.warnings.len(), 2);
    }

    #[test]
    fn out_of_bounds_rejected() {
        // Y above the 12in sheet with a bottom-left origin -> negative machine Y
        assert!(matches!(
            parse("G20\nG0 X1 Y13\nG1 X2 Y13"),
            Err(Error::Gcode { line: 3, .. })
        ));
        assert!(parse("G20\nG0 X1 Y1\nG1 X13 Y1").is_err());
        let long_mat = GcodeOptions {
            mat_extent_in: (12.25, 24.25),
            ..top_left()
        };
        assert!(parse_with("G20\nG0 X1 Y20\nG1 X2 Y20", &long_mat).is_ok());
    }

    #[test]
    fn bounds() {
        let plan = parse_with("G20 G0 X1 Y2\nG1 X3 Y0.5", &top_left()).unwrap();
        let (lo, hi) = plan.bounds().unwrap();
        approx(lo, 1.0, 0.5);
        approx(hi, 3.0, 2.0);
    }
}
