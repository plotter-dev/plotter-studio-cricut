//! Drag-knife compensation.
//!
//! A Cricut blade swivels freely, and its cutting tip sits a small distance `offset` behind the
//! swivel axis, trailing it in the direction of travel. The machine positions the *axis*, so an
//! uncompensated path cuts short at the end of every edge (e.g. a closed shape's last edge stops
//! `offset` before the start point) and rounds every corner.
//!
//! Compensation moves the axis so the *tip* follows the drawn path:
//! - every edge is extended by `offset` in its direction of travel, so the trailing tip reaches
//!   the corner;
//! - at a corner the axis sweeps an arc of radius `offset` around the corner point, swivelling
//!   the blade in place to the new direction;
//! - closed shapes can be overcut: the path continues past its start for `overcut` along the
//!   first edge, so the start/end seam is cut twice.
//!
//! Arcs are flattened with `lyon_geom`.

use lyon_geom::{Angle, Arc, point, vector};

use crate::gcode::{CutPlan, Point, Polyline};

/// Drag-knife parameters, in inches.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DragKnife {
    /// Distance from the swivel axis to the cutting tip.
    pub offset_in: f64,
    /// Extra distance to re-cut past the start of closed shapes.
    pub overcut_in: f64,
    /// Corners turning less than this many degrees get no swivel arc.
    pub min_turn_deg: f64,
    /// Max deviation when flattening swivel arcs.
    pub arc_tolerance_in: f64,
}

impl DragKnife {
    /// Compensation for a blade with the given tip offset (inches); no overcut.
    pub fn new(offset_in: f64) -> Self {
        Self {
            offset_in,
            overcut_in: 0.0,
            min_turn_deg: 10.0,
            arc_tolerance_in: 0.0005,
        }
    }

    pub fn from_mm(offset_mm: f64) -> Self {
        Self::new(offset_mm / 25.4)
    }

    #[must_use]
    pub fn overcut(mut self, overcut_in: f64) -> Self {
        self.overcut_in = overcut_in;
        self
    }

    /// Compensate one polyline.
    pub fn compensate(&self, pl: &Polyline) -> Polyline {
        let mut pts: Vec<Point> = Vec::with_capacity(pl.points.len());
        for p in &pl.points {
            if pts.last().is_none_or(|q| dist(*q, *p) > 1e-9) {
                pts.push(*p);
            }
        }
        let (Some(&first), Some(&last)) = (pts.first(), pts.last()) else {
            return Polyline { points: pts };
        };
        if pts.len() < 2 {
            return Polyline { points: pts };
        }
        let closed = dist(first, last) < 1e-6;
        if closed && self.overcut_in > 0.0 {
            pts.extend(walk(&pts, self.overcut_in));
        }
        let d = self.offset_in;
        if d <= 0.0 {
            return Polyline { points: pts };
        }

        let dirs: Vec<(f64, f64)> = pts.windows(2).map(|w| unit(w[0], w[1])).collect();
        let min_turn = self.min_turn_deg.to_radians();
        let mut out = vec![shift(pts[0], dirs[0], d)];
        for (i, &u) in dirs.iter().enumerate() {
            let corner = pts[i + 1];
            out.push(shift(corner, u, d));
            let Some(&next) = dirs.get(i + 1) else { break };
            let a0 = u.1.atan2(u.0);
            let turn = wrap(next.1.atan2(next.0) - a0);
            if turn.abs() > min_turn {
                // Swivel around the corner: the tip stays on it while the axis sweeps.
                let arc = Arc {
                    center: point(corner.x, corner.y),
                    radii: vector(d, d),
                    start_angle: Angle::radians(a0),
                    sweep_angle: Angle::radians(turn),
                    x_rotation: Angle::zero(),
                };
                out.extend(
                    arc.flattened(self.arc_tolerance_in.max(1e-6))
                        .map(|p| Point::new(p.x, p.y)),
                );
            }
        }
        Polyline { points: out }
    }

    /// Compensate every polyline of a plan.
    pub fn compensate_plan(&self, plan: &CutPlan) -> CutPlan {
        CutPlan {
            polylines: plan.polylines.iter().map(|p| self.compensate(p)).collect(),
            warnings: plan.warnings.clone(),
        }
    }
}

fn dist(a: Point, b: Point) -> f64 {
    (b.x - a.x).hypot(b.y - a.y)
}

fn unit(a: Point, b: Point) -> (f64, f64) {
    let l = dist(a, b);
    ((b.x - a.x) / l, (b.y - a.y) / l)
}

fn shift(p: Point, u: (f64, f64), d: f64) -> Point {
    Point::new(p.x + u.0 * d, p.y + u.1 * d)
}

/// Wrap an angle into (-π, π].
fn wrap(a: f64) -> f64 {
    let tau = std::f64::consts::TAU;
    let a = a.rem_euclid(tau);
    if a > std::f64::consts::PI { a - tau } else { a }
}

/// Points along `pts` from its start, covering `len` (the last one possibly mid-edge).
fn walk(pts: &[Point], mut len: f64) -> Vec<Point> {
    let mut out = Vec::new();
    for w in pts.windows(2) {
        let seg = dist(w[0], w[1]);
        if seg >= len {
            out.push(shift(w[0], unit(w[0], w[1]), len));
            return out;
        }
        out.push(w[1]);
        len -= seg;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pl(pts: &[(f64, f64)]) -> Polyline {
        Polyline {
            points: pts.iter().map(|&(x, y)| Point::new(x, y)).collect(),
        }
    }

    fn near(p: Point, x: f64, y: f64) {
        assert!(
            (p.x - x).abs() < 1e-9 && (p.y - y).abs() < 1e-9,
            "{p:?} != ({x}, {y})"
        );
    }

    #[test]
    fn zero_offset_is_identity() {
        let p = pl(&[(1.0, 1.0), (2.0, 1.0), (2.0, 2.0)]);
        assert_eq!(DragKnife::new(0.0).compensate(&p), p);
    }

    #[test]
    fn straight_edge_is_extended_by_offset() {
        let out = DragKnife::new(0.01).compensate(&pl(&[(1.0, 1.0), (2.0, 1.0)]));
        assert_eq!(out.points.len(), 2);
        near(out.points[0], 1.01, 1.0);
        near(out.points[1], 2.01, 1.0);
    }

    #[test]
    fn corner_gets_swivel_arc_around_corner_point() {
        let d = 0.01;
        let out = DragKnife::new(d).compensate(&pl(&[(1.0, 1.0), (2.0, 1.0), (2.0, 2.0)]));
        let p = &out.points;
        near(p[1], 2.01, 1.0); // overshoot the corner
        // swivel arc: every point is `d` from the corner, ending pointing down the next edge
        let arc_end = p.len() - 2;
        for q in &p[1..=arc_end] {
            assert!((dist(*q, Point::new(2.0, 1.0)) - d).abs() < 1e-9, "{q:?}");
        }
        near(p[arc_end], 2.0, 1.01);
        assert!(arc_end >= 3, "arc should have intermediate points");
        near(*p.last().unwrap(), 2.0, 2.01);
    }

    #[test]
    fn shallow_turn_has_no_arc() {
        let out = DragKnife::new(0.01).compensate(&pl(&[(0.0, 0.0), (1.0, 0.0), (2.0, 0.05)]));
        assert_eq!(out.points.len(), 3);
    }

    #[test]
    fn closed_square_overcut_retraces_first_edge() {
        let sq = pl(&[(1.0, 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 2.0), (1.0, 1.0)]);
        let out = DragKnife::new(0.0).overcut(0.25).compensate(&sq);
        near(*out.points.last().unwrap(), 1.25, 1.0);
        // with offset: the closing corner at the start also swivels, then the tip runs on
        let out = DragKnife::new(0.01).overcut(0.25).compensate(&sq);
        near(*out.points.last().unwrap(), 1.26, 1.0);
        let swivels = out
            .points
            .iter()
            .filter(|q| (dist(**q, Point::new(1.0, 1.0)) - 0.01).abs() < 1e-9)
            .count();
        assert!(swivels > 2, "closing corner should swivel");
    }

    #[test]
    fn duplicate_points_are_ignored() {
        let out = DragKnife::new(0.01).compensate(&pl(&[(1.0, 1.0), (1.0, 1.0), (2.0, 1.0)]));
        assert_eq!(out.points.len(), 2);
    }
}
