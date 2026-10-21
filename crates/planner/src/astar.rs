//! A* on an 8-connected occupancy grid with an octile heuristic.
//!
//! Cells are `grid` world units wide; cell (i, j) is free when the robot disc centred at the
//! cell centre is collision-free. Every move (including start -> first cell and last cell -> goal)
//! is additionally checked as a swept segment, so the returned path is exactly collision-free.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use crate::events::{Event, EventLog, PlanOutput};
use crate::geometry::{Point, World};

const SQRT2: f64 = std::f64::consts::SQRT_2;
/// Upper bound on `frontier` events emitted per A* run.
pub const MAX_FRONTIER_EVENTS: usize = 4_000;

#[derive(Clone, Copy, Debug, PartialEq)]
struct OpenItem {
    f: f64,
    g: f64,
    idx: usize,
}

impl Eq for OpenItem {}

impl Ord for OpenItem {
    fn cmp(&self, other: &Self) -> Ordering {
        // Min-heap on f; tie-break prefers larger g (deeper nodes), then lower index.
        other
            .f
            .total_cmp(&self.f)
            .then_with(|| self.g.total_cmp(&other.g))
            .then_with(|| other.idx.cmp(&self.idx))
    }
}

impl PartialOrd for OpenItem {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

struct Grid<'a> {
    world: &'a World,
    size: f64,
    cols: usize,
    rows: usize,
    /// 0 = unknown, 1 = free, 2 = blocked (lazy cache).
    occ: Vec<u8>,
}

impl<'a> Grid<'a> {
    fn new(world: &'a World, size: f64) -> Self {
        let cols = ((world.width / size).ceil() as usize).max(1);
        let rows = ((world.height / size).ceil() as usize).max(1);
        Self { world, size, cols, rows, occ: vec![0; cols * rows] }
    }

    fn center(&self, idx: usize) -> Point {
        let i = idx % self.cols;
        let j = idx / self.cols;
        Point::new((i as f64 + 0.5) * self.size, (j as f64 + 0.5) * self.size)
    }

    fn free(&mut self, idx: usize) -> bool {
        if self.occ[idx] == 0 {
            let c = self.center(idx);
            self.occ[idx] = if self.world.point_free(c) { 1 } else { 2 };
        }
        self.occ[idx] == 1
    }

    fn cell_of(&self, p: Point) -> (i64, i64) {
        ((p.x / self.size).floor() as i64, (p.y / self.size).floor() as i64)
    }

    /// Nearest free cell whose centre is reachable from `p` by a collision-free segment.
    fn snap(&mut self, p: Point) -> Option<usize> {
        let (ci, cj) = self.cell_of(p);
        let mut candidates: Vec<(f64, usize)> = Vec::new();
        for dj in -2i64..=2 {
            for di in -2i64..=2 {
                let i = ci + di;
                let j = cj + dj;
                if i < 0 || j < 0 || i >= self.cols as i64 || j >= self.rows as i64 {
                    continue;
                }
                let idx = j as usize * self.cols + i as usize;
                candidates.push((self.center(idx).dist(p), idx));
            }
        }
        candidates.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        for (_, idx) in candidates {
            if self.free(idx) && self.world.segment_free(p, self.center(idx)) {
                return Some(idx);
            }
        }
        None
    }
}

fn octile(a: Point, b: Point) -> f64 {
    let dx = (a.x - b.x).abs();
    let dy = (a.y - b.y).abs();
    let (lo, hi) = if dx < dy { (dx, dy) } else { (dy, dx) };
    hi + (SQRT2 - 1.0) * lo
}

pub fn astar(world: &World, start: Point, goal: Point, grid_size: f64, log: &mut EventLog) -> PlanOutput {
    let mut out = PlanOutput::default();
    if !world.point_free(start) || !world.point_free(goal) {
        return out;
    }
    let mut grid = Grid::new(world, grid_size);
    let (s, g) = match (grid.snap(start), grid.snap(goal)) {
        (Some(s), Some(g)) => (s, g),
        _ => return out,
    };

    let n = grid.cols * grid.rows;
    let goal_c = grid.center(g);
    let mut gcost = vec![f64::INFINITY; n];
    let mut parent = vec![usize::MAX; n];
    let mut closed = vec![false; n];
    let mut heap = BinaryHeap::new();

    gcost[s] = 0.0;
    parent[s] = s;
    heap.push(OpenItem { f: octile(grid.center(s), goal_c), g: 0.0, idx: s });
    // Frontier events are optional; cap their volume so expansions dominate the replay log.
    let mut frontier_budget = MAX_FRONTIER_EVENTS;
    let sc = grid.center(s);
    log.push(Event::Frontier { x: sc.x, y: sc.y });
    frontier_budget -= 1;

    const NB: [(i64, i64, f64); 8] = [
        (1, 0, 1.0),
        (-1, 0, 1.0),
        (0, 1, 1.0),
        (0, -1, 1.0),
        (1, 1, SQRT2),
        (1, -1, SQRT2),
        (-1, 1, SQRT2),
        (-1, -1, SQRT2),
    ];

    let mut found = false;
    while let Some(item) = heap.pop() {
        out.iterations += 1;
        let cur = item.idx;
        if closed[cur] || item.g > gcost[cur] {
            continue;
        }
        closed[cur] = true;
        out.nodes_explored += 1;
        let c = grid.center(cur);
        let pc = grid.center(parent[cur]);
        log.push(Event::Expand { x: c.x, y: c.y, px: pc.x, py: pc.y });
        if cur == g {
            found = true;
            break;
        }
        let ci = (cur % grid.cols) as i64;
        let cj = (cur / grid.cols) as i64;
        for (di, dj, w) in NB {
            let ni = ci + di;
            let nj = cj + dj;
            if ni < 0 || nj < 0 || ni >= grid.cols as i64 || nj >= grid.rows as i64 {
                continue;
            }
            let nidx = nj as usize * grid.cols + ni as usize;
            if closed[nidx] || !grid.free(nidx) {
                continue;
            }
            let ng = gcost[cur] + w * grid_size;
            if ng >= gcost[nidx] {
                continue;
            }
            let ncent = grid.center(nidx);
            if !world.segment_free(c, ncent) {
                continue;
            }
            let first_time = gcost[nidx].is_infinite();
            gcost[nidx] = ng;
            parent[nidx] = cur;
            heap.push(OpenItem { f: ng + octile(ncent, goal_c), g: ng, idx: nidx });
            if first_time && frontier_budget > 0 {
                frontier_budget -= 1;
                log.push(Event::Frontier { x: ncent.x, y: ncent.y });
            }
        }
    }

    if found {
        let mut cells = vec![g];
        let mut cur = g;
        while cur != s {
            cur = parent[cur];
            cells.push(cur);
        }
        cells.reverse();
        let mut path = Vec::with_capacity(cells.len() + 2);
        path.push(start);
        for idx in cells {
            let p = grid.center(idx);
            if path.last().map_or(true, |l: &Point| l.dist(p) > 1e-9) {
                path.push(p);
            }
        }
        if path.last().map_or(true, |l| l.dist(goal) > 1e-9) {
            path.push(goal);
        }
        out.found = true;
        out.path = path;
    }
    out
}
