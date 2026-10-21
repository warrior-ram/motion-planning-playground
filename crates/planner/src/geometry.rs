//! Geometry primitives and exact disc-robot collision checks.

use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Point {
    pub x: f64,
    pub y: f64,
}

impl Point {
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    pub fn dist(self, o: Point) -> f64 {
        (self.x - o.x).hypot(self.y - o.y)
    }

    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite()
    }
}

/// Obstacle shape shared with the frontend. Rect `x,y` = top-left; circle `x,y` = centre.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "lowercase")]
pub enum Obstacle {
    Rect { x: f64, y: f64, w: f64, h: f64 },
    Circle { x: f64, y: f64, r: f64 },
}

impl Obstacle {
    pub fn validate(&self) -> Result<(), String> {
        match *self {
            Obstacle::Rect { x, y, w, h } => {
                if !(x.is_finite() && y.is_finite() && w.is_finite() && h.is_finite()) {
                    return Err("rect obstacle has non-finite values".into());
                }
                if w < 0.0 || h < 0.0 {
                    return Err("rect obstacle has negative size".into());
                }
            }
            Obstacle::Circle { x, y, r } => {
                if !(x.is_finite() && y.is_finite() && r.is_finite()) {
                    return Err("circle obstacle has non-finite values".into());
                }
                if r < 0.0 {
                    return Err("circle obstacle has negative radius".into());
                }
            }
        }
        Ok(())
    }

    /// Axis-aligned bounding box as (min x, min y, max x, max y).
    pub fn bounds(&self) -> (f64, f64, f64, f64) {
        match *self {
            Obstacle::Rect { x, y, w, h } => (x, y, x + w, y + h),
            Obstacle::Circle { x, y, r } => (x - r, y - r, x + r, y + r),
        }
    }

    /// Distance from point to the shape (0 if inside).
    pub fn point_dist(&self, p: Point) -> f64 {
        match *self {
            Obstacle::Rect { x, y, w, h } => point_rect_dist(p, x, y, w, h),
            Obstacle::Circle { x, y, r } => (p.dist(Point::new(x, y)) - r).max(0.0),
        }
    }

    /// Exact distance from segment ab to the shape (0 if they intersect).
    pub fn segment_dist(&self, a: Point, b: Point) -> f64 {
        match *self {
            Obstacle::Rect { x, y, w, h } => {
                if seg_intersects_rect(a, b, x, y, w, h) {
                    return 0.0;
                }
                let corners = [
                    Point::new(x, y),
                    Point::new(x + w, y),
                    Point::new(x, y + h),
                    Point::new(x + w, y + h),
                ];
                let mut d = point_rect_dist(a, x, y, w, h).min(point_rect_dist(b, x, y, w, h));
                for c in corners {
                    d = d.min(seg_point_dist(a, b, c));
                }
                d
            }
            Obstacle::Circle { x, y, r } => (seg_point_dist(a, b, Point::new(x, y)) - r).max(0.0),
        }
    }
}

pub fn point_rect_dist(p: Point, x: f64, y: f64, w: f64, h: f64) -> f64 {
    let dx = (x - p.x).max(0.0).max(p.x - (x + w));
    let dy = (y - p.y).max(0.0).max(p.y - (y + h));
    dx.hypot(dy)
}

pub fn seg_point_dist(a: Point, b: Point, p: Point) -> f64 {
    let abx = b.x - a.x;
    let aby = b.y - a.y;
    let len2 = abx * abx + aby * aby;
    if len2 < 1e-18 {
        return a.dist(p);
    }
    let t = (((p.x - a.x) * abx + (p.y - a.y) * aby) / len2).clamp(0.0, 1.0);
    p.dist(Point::new(a.x + t * abx, a.y + t * aby))
}

/// Liang-Barsky segment vs axis-aligned rect intersection test.
pub fn seg_intersects_rect(a: Point, b: Point, x: f64, y: f64, w: f64, h: f64) -> bool {
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    let mut t0 = 0.0f64;
    let mut t1 = 1.0f64;
    let checks = [
        (-dx, a.x - x),
        (dx, x + w - a.x),
        (-dy, a.y - y),
        (dy, y + h - a.y),
    ];
    for (p, q) in checks {
        if p == 0.0 {
            if q < 0.0 {
                return false;
            }
        } else {
            let r = q / p;
            if p < 0.0 {
                if r > t1 {
                    return false;
                }
                if r > t0 {
                    t0 = r;
                }
            } else {
                if r < t0 {
                    return false;
                }
                if r < t1 {
                    t1 = r;
                }
            }
        }
    }
    true
}

/// Planning world for a disc robot of radius `radius`.
#[derive(Clone, Debug)]
pub struct World {
    pub width: f64,
    pub height: f64,
    pub radius: f64,
    pub obstacles: Vec<Obstacle>,
}

impl World {
    pub fn new(width: f64, height: f64, radius: f64, obstacles: Vec<Obstacle>) -> Self {
        Self { width, height, radius, obstacles }
    }

    /// Clearance threshold; a tiny epsilon so a zero-radius robot still collides when inside.
    fn clearance(&self) -> f64 {
        self.radius.max(1e-6)
    }

    pub fn in_bounds(&self, p: Point) -> bool {
        let r = self.radius;
        p.x >= r && p.x <= self.width - r && p.y >= r && p.y <= self.height - r
    }

    /// True if the robot disc centred at `p` is collision-free.
    pub fn point_free(&self, p: Point) -> bool {
        if !self.in_bounds(p) {
            return false;
        }
        let c = self.clearance();
        self.obstacles.iter().all(|o| o.point_dist(p) >= c)
    }

    /// True if the robot disc swept along segment ab is collision-free (exact test).
    pub fn segment_free(&self, a: Point, b: Point) -> bool {
        // Bounds region is convex, so checking the endpoints suffices.
        if !self.in_bounds(a) || !self.in_bounds(b) {
            return false;
        }
        let c = self.clearance();
        let (sx0, sx1) = (a.x.min(b.x), a.x.max(b.x));
        let (sy0, sy1) = (a.y.min(b.y), a.y.max(b.y));
        self.obstacles.iter().all(|o| {
            // The gap between bounding boxes is a lower bound on the true distance, so an
            // obstacle whose box is at least `c` away can never collide; skip the exact test.
            let (ox0, oy0, ox1, oy1) = o.bounds();
            let gap = (ox0 - sx1).max(sx0 - ox1).max(oy0 - sy1).max(sy0 - oy1);
            gap >= c || o.segment_dist(a, b) >= c
        })
    }
}

pub fn path_length(path: &[Point]) -> f64 {
    path.windows(2).map(|w| w[0].dist(w[1])).sum()
}
