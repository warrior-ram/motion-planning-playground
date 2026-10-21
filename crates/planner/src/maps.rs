//! The fixed, deterministic set of 20 benchmark maps.

use serde::Serialize;

use crate::astar::astar;
use crate::events::EventLog;
use crate::geometry::{Obstacle, Point, World};
use crate::rng::SplitMix64;

pub const WIDTH: f64 = 1000.0;
pub const HEIGHT: f64 = 640.0;
/// Robot radius the maps are designed (and validated) for; matches the `plan` default.
pub const DESIGN_RADIUS: f64 = 12.0;
/// A* grid size the maps are validated with; matches the `plan` default.
pub const DESIGN_GRID: f64 = 10.0;

#[derive(Clone, Debug, Serialize)]
pub struct BenchMap {
    pub id: String,
    pub name: String,
    pub width: f64,
    pub height: f64,
    pub start: Point,
    pub goal: Point,
    pub obstacles: Vec<Obstacle>,
}

impl BenchMap {
    pub fn world(&self, radius: f64) -> World {
        World::new(self.width, self.height, radius, self.obstacles.clone())
    }
}

fn rect(x: f64, y: f64, w: f64, h: f64) -> Obstacle {
    Obstacle::Rect { x, y, w, h }
}

fn circle(x: f64, y: f64, r: f64) -> Obstacle {
    Obstacle::Circle { x, y, r }
}

fn map(n: usize, name: &str, start: (f64, f64), goal: (f64, f64), obstacles: Vec<Obstacle>) -> BenchMap {
    BenchMap {
        id: format!("m{:02}", n),
        name: name.to_string(),
        width: WIDTH,
        height: HEIGHT,
        start: Point::new(start.0, start.1),
        goal: Point::new(goal.0, goal.1),
        obstacles,
    }
}

#[derive(Clone, Copy, PartialEq)]
enum Side {
    Left,
    Right,
}

/// Rectangular ring of wall thickness `t` centred at (cx, cy) with half extents (hx, hy),
/// with a gap of `gap` units centred on the given side.
fn ring(cx: f64, cy: f64, hx: f64, hy: f64, t: f64, gap_side: Side, gap: f64) -> Vec<Obstacle> {
    let (x0, x1, y0, y1) = (cx - hx, cx + hx, cy - hy, cy + hy);
    let mut v = vec![rect(x0, y0, 2.0 * hx, t), rect(x0, y1 - t, 2.0 * hx, t)];
    let side_wall = |x: f64, gapped: bool| -> Vec<Obstacle> {
        if gapped {
            let g0 = cy - gap / 2.0;
            let g1 = cy + gap / 2.0;
            vec![rect(x, y0, t, g0 - y0), rect(x, g1, t, y1 - g1)]
        } else {
            vec![rect(x, y0, t, 2.0 * hy)]
        }
    };
    v.extend(side_wall(x0, gap_side == Side::Left));
    v.extend(side_wall(x1 - t, gap_side == Side::Right));
    v
}

fn maze(seed: u64) -> Vec<Obstacle> {
    const COLS: usize = 10;
    const ROWS: usize = 6;
    let cw = WIDTH / COLS as f64;
    let ch = HEIGHT / ROWS as f64;
    let t = 10.0;
    // walls[cell] = (east wall present, south wall present)
    let mut east = vec![true; COLS * ROWS];
    let mut south = vec![true; COLS * ROWS];
    let mut visited = vec![false; COLS * ROWS];
    let mut rng = SplitMix64::new(seed);
    let mut stack = vec![0usize];
    visited[0] = true;
    while let Some(&cur) = stack.last() {
        let (i, j) = (cur % COLS, cur / COLS);
        let mut nb: Vec<usize> = Vec::with_capacity(4);
        if i > 0 && !visited[cur - 1] {
            nb.push(cur - 1);
        }
        if i + 1 < COLS && !visited[cur + 1] {
            nb.push(cur + 1);
        }
        if j > 0 && !visited[cur - COLS] {
            nb.push(cur - COLS);
        }
        if j + 1 < ROWS && !visited[cur + COLS] {
            nb.push(cur + COLS);
        }
        if nb.is_empty() {
            stack.pop();
            continue;
        }
        let next = nb[rng.below(nb.len())];
        if next == cur + 1 {
            east[cur] = false;
        } else if next + 1 == cur {
            east[next] = false;
        } else if next == cur + COLS {
            south[cur] = false;
        } else {
            south[next] = false;
        }
        visited[next] = true;
        stack.push(next);
    }
    let mut obs = Vec::new();
    for j in 0..ROWS {
        for i in 0..COLS {
            let c = j * COLS + i;
            if east[c] && i + 1 < COLS {
                let x = (i + 1) as f64 * cw - t / 2.0;
                obs.push(rect(x, j as f64 * ch - t / 2.0, t, ch + t));
            }
            if south[c] && j + 1 < ROWS {
                let y = (j + 1) as f64 * ch - t / 2.0;
                obs.push(rect(i as f64 * cw - t / 2.0, y, cw + t, t));
            }
        }
    }
    // Clip to world (cosmetic only; collision against out-of-world parts is irrelevant).
    obs.into_iter()
        .map(|o| match o {
            Obstacle::Rect { x, y, w, h } => {
                let x0 = x.max(0.0);
                let y0 = y.max(0.0);
                let x1 = (x + w).min(WIDTH);
                let y1 = (y + h).min(HEIGHT);
                rect(x0, y0, x1 - x0, y1 - y0)
            }
            o => o,
        })
        .collect()
}

fn solvable(start: Point, goal: Point, obstacles: &[Obstacle]) -> bool {
    let world = World::new(WIDTH, HEIGHT, DESIGN_RADIUS, obstacles.to_vec());
    let mut log = EventLog::new(false);
    astar(&world, start, goal, DESIGN_GRID, &mut log).found
}

/// Seeded random clutter. Obstacles that would touch the start/goal (plus margin) are rejected;
/// if the result is not solvable by A*, the generator deterministically retries with the next
/// sub-seed, so the output is fixed for a given `seed`.
fn clutter(seed: u64, count: usize, rect_frac: f64, size_lo: f64, size_hi: f64, start: Point, goal: Point) -> Vec<Obstacle> {
    let margin = DESIGN_RADIUS + 25.0;
    for attempt in 0..64u64 {
        let mut rng = SplitMix64::new(seed.wrapping_mul(1_000_003).wrapping_add(attempt));
        let mut obs = Vec::with_capacity(count);
        let mut tries = 0;
        while obs.len() < count && tries < count * 50 {
            tries += 1;
            let o = if rng.next_f64() < rect_frac {
                let w = rng.range(size_lo, size_hi).round();
                let h = rng.range(size_lo, size_hi).round();
                let x = rng.range(0.0, WIDTH - w).round();
                let y = rng.range(0.0, HEIGHT - h).round();
                rect(x, y, w, h)
            } else {
                let r = (rng.range(size_lo, size_hi) / 2.0).round();
                let x = rng.range(r, WIDTH - r).round();
                let y = rng.range(r, HEIGHT - r).round();
                circle(x, y, r)
            };
            if o.point_dist(start) < margin || o.point_dist(goal) < margin {
                continue;
            }
            obs.push(o);
        }
        if solvable(start, goal, &obs) {
            return obs;
        }
    }
    Vec::new()
}

/// Build the 20 benchmark maps. Pure and deterministic.
pub fn benchmark_maps() -> Vec<BenchMap> {
    let mut maps = Vec::with_capacity(20);
    let w = WIDTH;
    let h = HEIGHT;

    // m01 Open field
    maps.push(map(1, "Open field", (80.0, 320.0), (920.0, 320.0), vec![]));

    // m02 Single wall with gaps at the ends
    maps.push(map(
        2,
        "Single wall",
        (80.0, 320.0),
        (920.0, 320.0),
        vec![rect(490.0, 90.0, 20.0, 460.0)],
    ));

    // m03 Serpentine corridors
    maps.push(map(
        3,
        "Serpentine corridors",
        (60.0, 70.0),
        (60.0, 570.0),
        vec![
            rect(0.0, 150.0, 850.0, 15.0),
            rect(150.0, 310.0, 850.0, 15.0),
            rect(0.0, 470.0, 850.0, 15.0),
        ],
    ));

    // m04 Maze (seeded DFS maze, 10 x 6 cells)
    maps.push(map(4, "Maze", (50.0, 53.0), (950.0, 587.0), maze(4)));

    // m05 Bug trap: U-shaped pocket around the start, opening away from the goal
    maps.push(map(
        5,
        "Bug trap",
        (310.0, 320.0),
        (900.0, 320.0),
        vec![
            rect(380.0, 180.0, 20.0, 280.0),
            rect(180.0, 180.0, 220.0, 20.0),
            rect(180.0, 440.0, 220.0, 20.0),
        ],
    ));

    // m06 Narrow passage: full-height wall with a single 40-unit gap
    maps.push(map(
        6,
        "Narrow passage",
        (120.0, 120.0),
        (880.0, 520.0),
        vec![rect(490.0, 0.0, 20.0, 300.0), rect(490.0, 340.0, 20.0, 300.0)],
    ));

    // m07 Spiral: concentric rings with alternating gaps; start in the centre
    {
        let mut obs = Vec::new();
        // Three rings with ~100-unit (x) and ~80-unit (y) corridors between them, so a radius-12
        // robot always has at least 56 units of free width. Tighter rings made the outer corridors
        // 6-26 units wide, which no uniform sampler gets through in any reasonable budget. The
        // outermost gap faces the goal so the route never needs the thin strip at the world border.
        let rings = [(420.0, 260.0, Side::Right), (320.0, 180.0, Side::Left), (220.0, 100.0, Side::Right)];
        for (hx, hy, side) in rings {
            obs.extend(ring(500.0, 320.0, hx, hy, 10.0, side, 60.0));
        }
        maps.push(map(7, "Spiral", (500.0, 320.0), (970.0, 320.0), obs));
    }

    // m08 Slalom of large discs
    maps.push(map(
        8,
        "Slalom",
        (60.0, 320.0),
        (940.0, 320.0),
        vec![
            circle(200.0, 220.0, 110.0),
            circle(350.0, 420.0, 110.0),
            circle(500.0, 220.0, 110.0),
            circle(650.0, 420.0, 110.0),
            circle(800.0, 220.0, 110.0),
        ],
    ));

    // m09 Rooms: 2x2 rooms; doors force a top-left -> top-right -> bottom-right route
    maps.push(map(
        9,
        "Rooms",
        (120.0, 120.0),
        (880.0, 540.0),
        vec![
            // vertical divider, door at y 90..160 (top half only)
            rect(495.0, 0.0, 10.0, 90.0),
            rect(495.0, 160.0, 10.0, h - 160.0),
            // horizontal divider, door at x 760..830 (right half only)
            rect(0.0, 315.0, 760.0, 10.0),
            rect(830.0, 315.0, w - 830.0, 10.0),
            // furniture
            rect(200.0, 200.0, 140.0, 60.0),
            rect(620.0, 60.0, 80.0, 160.0),
            rect(150.0, 420.0, 200.0, 120.0),
            circle(700.0, 480.0, 50.0),
        ],
    ));

    // m10 Double narrow passage
    maps.push(map(
        10,
        "Double narrow passage",
        (80.0, 540.0),
        (920.0, 100.0),
        vec![
            rect(320.0, 0.0, 20.0, 100.0),
            rect(320.0, 140.0, 20.0, h - 140.0),
            rect(660.0, 0.0, 20.0, 500.0),
            rect(660.0, 540.0, 20.0, h - 540.0),
        ],
    ));

    // m11 Zigzag walls (alternating from top and bottom)
    maps.push(map(
        11,
        "Zigzag walls",
        (60.0, 320.0),
        (940.0, 320.0),
        vec![
            rect(180.0, 0.0, 20.0, 480.0),
            rect(340.0, 160.0, 20.0, 480.0),
            rect(500.0, 0.0, 20.0, 480.0),
            rect(660.0, 160.0, 20.0, 480.0),
            rect(820.0, 0.0, 20.0, 480.0),
        ],
    ));

    // m12 Pillar lattice
    {
        let start = Point::new(40.0, 320.0);
        let goal = Point::new(960.0, 320.0);
        let mut obs = Vec::new();
        let mut row = 0;
        let mut y = 50.0;
        while y < h {
            let offset = if row % 2 == 0 { 0.0 } else { 45.0 };
            let mut x = 110.0 + offset;
            while x < w - 60.0 {
                let c = circle(x, y, 22.0);
                if c.point_dist(start) > 40.0 && c.point_dist(goal) > 40.0 {
                    obs.push(c);
                }
                x += 90.0;
            }
            y += 90.0;
            row += 1;
        }
        maps.push(map(12, "Pillar lattice", (start.x, start.y), (goal.x, goal.y), obs));
    }

    // m13 Warehouse shelves
    {
        let mut obs = Vec::new();
        for r in 0..4 {
            for c in 0..3 {
                let x = 160.0 + c as f64 * 240.0;
                let y = 90.0 + r as f64 * 130.0;
                obs.push(rect(x, y, 190.0, 40.0));
            }
        }
        obs.push(rect(40.0, 250.0, 40.0, 140.0));
        obs.push(rect(920.0, 250.0, 40.0, 140.0));
        maps.push(map(13, "Warehouse shelves", (40.0, 40.0), (960.0, 600.0), obs));
    }

    // m14 Dead ends: lanes that look promising but only the bottom one reaches the gap
    maps.push(map(
        14,
        "Dead ends",
        (300.0, 240.0),
        (900.0, 320.0),
        vec![
            rect(600.0, 0.0, 20.0, 560.0),
            rect(150.0, 155.0, 450.0, 10.0),
            rect(150.0, 315.0, 450.0, 10.0),
            rect(150.0, 475.0, 450.0, 10.0),
        ],
    ));

    // m15..m20 seeded random clutter
    let s = Point::new(50.0, 320.0);
    let g = Point::new(950.0, 320.0);
    let random_specs: [(&str, u64, usize, f64, f64, f64); 6] = [
        ("Sparse clutter", 15, 18, 0.5, 30.0, 110.0),
        ("Scattered boulders", 16, 30, 0.0, 30.0, 90.0),
        ("Mixed clutter", 17, 40, 0.5, 25.0, 90.0),
        ("Dense blocks", 18, 55, 1.0, 20.0, 80.0),
        ("Dense mixed clutter", 19, 75, 0.5, 20.0, 65.0),
        ("Forest", 20, 140, 0.0, 14.0, 40.0),
    ];
    for (name, seed, count, rect_frac, lo, hi) in random_specs {
        let obs = clutter(seed, count, rect_frac, lo, hi, s, g);
        maps.push(map(seed as usize, name, (s.x, s.y), (g.x, g.y), obs));
    }

    maps
}
