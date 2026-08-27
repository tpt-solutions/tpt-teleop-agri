//! AB-line and curve (polyline) guidance math.
//!
//! All positions are in the local ENU frame (metres). The hot-path methods
//! (`cross_track`, `along_track`) are allocation-free and return the signed
//! perpendicular error the controller converts into a steering setpoint.

/// A 2D vector in the local ENU plane (x = east, y = north).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Vec2 {
    /// East component (metres).
    pub x: f64,
    /// North component (metres).
    pub y: f64,
}

impl Vec2 {
    /// Construct a vector.
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    /// Vector subtraction.
    #[allow(clippy::should_implement_trait)]
    pub fn sub(self, o: Vec2) -> Vec2 {
        Vec2::new(self.x - o.x, self.y - o.y)
    }

    /// Vector addition.
    #[allow(clippy::should_implement_trait)]
    pub fn add(self, o: Vec2) -> Vec2 {
        Vec2::new(self.x + o.x, self.y + o.y)
    }

    /// Dot product.
    pub fn dot(self, o: Vec2) -> f64 {
        self.x * o.x + self.y * o.y
    }

    /// Scale by `s`.
    pub fn scale(self, s: f64) -> Vec2 {
        Vec2::new(self.x * s, self.y * s)
    }

    /// Euclidean length.
    pub fn len(self) -> f64 {
        (self.x * self.x + self.y * self.y).sqrt()
    }
}

/// An AB guidance line from `a` to `b` (extended infinitely along `a→b`).
#[derive(Debug, Clone, Copy)]
pub struct AbLine {
    a: Vec2,
    dir: Vec2,
    length: f64,
}

impl AbLine {
    /// Build a line from endpoint `a` to `b`. Degenerate (zero-length) lines
    /// fall back to an east-pointing direction.
    pub fn new(a: Vec2, b: Vec2) -> Self {
        let ab = b.sub(a);
        let length = ab.len();
        let dir = if length > 0.0 {
            ab.scale(1.0 / length)
        } else {
            Vec2::new(1.0, 0.0)
        };
        Self { a, dir, length }
    }

    /// Signed cross-track error (metres): positive = left of the line.
    pub fn cross_track(&self, p: Vec2) -> f64 {
        let ap = p.sub(self.a);
        self.dir.x * ap.y - self.dir.y * ap.x
    }

    /// Along-track distance (metres) of the projection of `p` onto the line,
    /// measured from `a` (may be negative or exceed `length`).
    pub fn along_track(&self, p: Vec2) -> f64 {
        let ap = p.sub(self.a);
        ap.dot(self.dir)
    }

    /// Heading of the line (radians, atan2(north, east)).
    pub fn heading(&self) -> f64 {
        self.dir.y.atan2(self.dir.x)
    }

    /// Length of the AB segment (metres).
    pub fn length(&self) -> f64 {
        self.length
    }
}

/// A curved guidance path represented as a polyline (e.g. a contour or
/// headland arc sampled into segments).
pub struct Polyline {
    pts: Vec<Vec2>,
}

impl Polyline {
    /// Build a path from ordered vertices.
    pub fn new(pts: Vec<Vec2>) -> Self {
        Self { pts }
    }

    /// Signed cross-track error to the closest segment (positive = left).
    pub fn cross_track(&self, p: Vec2) -> f64 {
        let mut best = f64::INFINITY;
        let mut best_sign = 0.0;
        for w in self.pts.windows(2) {
            let seg = w[1].sub(w[0]);
            let len2 = seg.dot(seg);
            let t = if len2 > 0.0 {
                (p.sub(w[0]).dot(seg) / len2).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let proj = w[0].add(seg.scale(t));
            let d = p.sub(proj).len();
            if d < best {
                best = d;
                let seg_unit = if len2 > 0.0 {
                    seg.scale(1.0 / len2.sqrt())
                } else {
                    Vec2::new(1.0, 0.0)
                };
                let cross = seg_unit.x * (p.sub(proj).y) - seg_unit.y * (p.sub(proj).x);
                best_sign = if cross >= 0.0 { 1.0 } else { -1.0 };
            }
        }
        best * best_sign
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ab_cross_track_sign() {
        let line = AbLine::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 0.0));
        assert!((line.cross_track(Vec2::new(5.0, 2.0)) - 2.0).abs() < 1e-9);
        assert!((line.cross_track(Vec2::new(5.0, -2.0)) + 2.0).abs() < 1e-9);
        assert!((line.along_track(Vec2::new(5.0, 2.0)) - 5.0).abs() < 1e-9);
        assert!((line.heading()).abs() < 1e-9);
    }

    #[test]
    fn ab_degenerate_fallback() {
        let line = AbLine::new(Vec2::new(0.0, 0.0), Vec2::new(0.0, 0.0));
        assert_eq!(line.heading(), 0.0);
    }

    #[test]
    fn polyline_closest_segment() {
        let path = Polyline::new(vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(10.0, 0.0),
            Vec2::new(10.0, 10.0),
        ]);
        assert!((path.cross_track(Vec2::new(5.0, 1.0)) - 1.0).abs() < 1e-9);
        assert!((path.cross_track(Vec2::new(12.0, 5.0)) + 2.0).abs() < 1e-9);
    }
}
