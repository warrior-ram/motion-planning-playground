//! 2D geometry: obstacles, capsule collision, angle wrapping.

use std::f64::consts::PI;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct V2 {
    pub x: f64,
    pub y: f64,
}

impl V2 {
    pub fn new(x: f64, y: f64) -> Self {
        V2 { x, y }
    }
    pub fn sub(self, o: V2) -> V2 {
        V2::new(self.x - o.x, self.y - o.y)
    }
    pub fn dot(self, o: V2) -> f64 {
        self.x * o.x + self.y * o.y
    }
    pub fn cross(self, o: V2) -> f64 {
        self.x * o.y - self.y * o.x
    }
    pub fn len(self) -> f64 {
        self.dot(self).sqrt()
    }
}

#[derive(Clone, Copy, Debug)]
pub enum Shape {
    /// Axis-aligned rectangle, normalised so x0 <= x1, y0 <= y1.
    Rect { x0: f64, y0: f64, x1: f64, y1: f64 },
    Circle { c: V2, r: f64 },
}

impl Shape {
    /// Rect from contract form (x,y = top-left, w,h); negative sizes are normalised.
    pub fn rect(x: f64, y: f64, w: f64, h: f64) -> Shape {
        let (x0, x1) = if w >= 0.0 { (x, x + w) } else { (x + w, x) };
        let (y0, y1) = if h >= 0.0 { (y, y + h) } else { (y + h, y) };
        Shape::Rect { x0, y0, x1, y1 }
    }
    pub fn circle(x: f64, y: f64, r: f64) -> Shape {
        Shape::Circle { c: V2::new(x, y), r: r.abs() }
    }
}

/// Wrap an angle to (-pi, pi].
pub fn wrap_angle(a: f64) -> f64 {
    let two_pi = 2.0 * PI;
    let r = a.rem_euclid(two_pi); // [0, 2pi)
    if r > PI {
        r - two_pi
    } else {
        r
    }
}

/// Shortest signed angular difference b - a, in (-pi, pi].
pub fn angle_diff(a: f64, b: f64) -> f64 {
    wrap_angle(b - a)
}

/// Distance from point p to segment ab.
pub fn point_seg_dist(p: V2, a: V2, b: V2) -> f64 {
    let ab = b.sub(a);
    let l2 = ab.dot(ab);
    let t = if l2 <= 1e-18 {
        0.0
    } else {
        (p.sub(a).dot(ab) / l2).clamp(0.0, 1.0)
    };
    let q = V2::new(a.x + ab.x * t, a.y + ab.y * t);
    p.sub(q).len()
}

/// True if segments ab and cd properly cross (touching cases are caught by distance).
fn segs_cross(a: V2, b: V2, c: V2, d: V2) -> bool {
    let d1 = b.sub(a).cross(c.sub(a));
    let d2 = b.sub(a).cross(d.sub(a));
    let d3 = d.sub(c).cross(a.sub(c));
    let d4 = d.sub(c).cross(b.sub(c));
    ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0))
        && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))
}

/// Minimum distance between segments ab and cd.
pub fn seg_seg_dist(a: V2, b: V2, c: V2, d: V2) -> f64 {
    if segs_cross(a, b, c, d) {
        return 0.0;
    }
    point_seg_dist(a, c, d)
        .min(point_seg_dist(b, c, d))
        .min(point_seg_dist(c, a, b))
        .min(point_seg_dist(d, a, b))
}

fn point_in_rect(p: V2, x0: f64, y0: f64, x1: f64, y1: f64) -> bool {
    p.x >= x0 && p.x <= x1 && p.y >= y0 && p.y <= y1
}

/// Does a capsule (segment ab, radius `radius`) overlap the shape?
pub fn capsule_hits(a: V2, b: V2, radius: f64, shape: &Shape) -> bool {
    match *shape {
        Shape::Circle { c, r } => point_seg_dist(c, a, b) <= r + radius,
        Shape::Rect { x0, y0, x1, y1 } => {
            if point_in_rect(a, x0, y0, x1, y1) || point_in_rect(b, x0, y0, x1, y1) {
                return true;
            }
            let c = [
                V2::new(x0, y0),
                V2::new(x1, y0),
                V2::new(x1, y1),
                V2::new(x0, y1),
            ];
            (0..4).any(|i| seg_seg_dist(a, b, c[i], c[(i + 1) % 4]) <= radius)
        }
    }
}
