//! RRT and RRT* in continuous 2-D space for a disc robot.

use crate::events::{Event, EventLog, PlanOutput};
use crate::geometry::{Point, World};
use crate::rng::SplitMix64;

#[derive(Clone, Copy, Debug)]
pub struct RrtParams {
    pub max_iterations: usize,
    pub step_size: f64,
    pub goal_bias: f64,
    pub goal_tolerance: f64,
    pub rewire_radius: f64,
}

impl Default for RrtParams {
    fn default() -> Self {
        Self {
            max_iterations: 20000,
            step_size: 25.0,
            goal_bias: 0.05,
            goal_tolerance: 20.0,
            rewire_radius: 60.0,
        }
    }
}

fn dist2(a: Point, b: Point) -> f64 {
    (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y)
}

/// Upper bound on grid cells, so very large worlds stay cheap in memory.
const MAX_GRID_CELLS: f64 = 65_536.0;

/// Uniform bucket grid over the world, used for nearest / radius queries.
/// Queries return exactly what a linear scan over node indices would (same ties, same order).
struct Grid {
    cell: f64,
    cols: usize,
    rows: usize,
    buckets: Vec<Vec<u32>>,
}

impl Grid {
    fn new(width: f64, height: f64, cell: f64) -> Self {
        let cell = cell.max((width * height / MAX_GRID_CELLS).sqrt()).max(1.0);
        let cols = ((width / cell).ceil() as usize).max(1);
        let rows = ((height / cell).ceil() as usize).max(1);
        Self { cell, cols, rows, buckets: vec![Vec::new(); cols * rows] }
    }

    fn col(&self, x: f64) -> usize {
        ((x / self.cell).floor().max(0.0) as usize).min(self.cols - 1)
    }

    fn row(&self, y: f64) -> usize {
        ((y / self.cell).floor().max(0.0) as usize).min(self.rows - 1)
    }

    fn insert(&mut self, p: Point, idx: usize) {
        let (c, r) = (self.col(p.x), self.row(p.y));
        self.buckets[r * self.cols + c].push(idx as u32);
    }
}

struct Tree {
    pts: Vec<Point>,
    parent: Vec<usize>,
    cost: Vec<f64>,
    children: Vec<Vec<usize>>,
    grid: Grid,
}

impl Tree {
    fn new(root: Point, world: &World, cell: f64) -> Self {
        let mut grid = Grid::new(world.width, world.height, cell);
        grid.insert(root, 0);
        Self { pts: vec![root], parent: vec![0], cost: vec![0.0], children: vec![Vec::new()], grid }
    }

    fn add(&mut self, p: Point, parent: usize) -> usize {
        let idx = self.pts.len();
        let c = self.cost[parent] + self.pts[parent].dist(p);
        self.pts.push(p);
        self.parent.push(parent);
        self.cost.push(c);
        self.children.push(Vec::new());
        self.children[parent].push(idx);
        self.grid.insert(p, idx);
        idx
    }

    /// Nearest node to `q`; on equal distance the lowest index wins (as a linear scan would).
    /// Searches square rings of cells outward until no unvisited cell can hold anything closer.
    fn nearest(&self, q: Point) -> usize {
        let g = &self.grid;
        let (cx, cy) = (g.col(q.x) as isize, g.row(q.y) as isize);
        let mut best = 0;
        let mut bd = f64::INFINITY;
        let max_ring = g.cols.max(g.rows) as isize;
        for k in 0..=max_ring {
            for y in (cy - k)..=(cy + k) {
                if y < 0 || y >= g.rows as isize {
                    continue;
                }
                let on_edge_row = y == cy - k || y == cy + k;
                let step = if on_edge_row { 1 } else { (2 * k).max(1) };
                let mut x = cx - k;
                while x <= cx + k {
                    if x >= 0 && x < g.cols as isize {
                        for &i in &g.buckets[y as usize * g.cols + x as usize] {
                            let i = i as usize;
                            let d = dist2(self.pts[i], q);
                            if d < bd || (d == bd && i < best) {
                                bd = d;
                                best = i;
                            }
                        }
                    }
                    x += step;
                }
            }
            // Distance from q to the edge of the (2k+1)^2 block already searched.
            let x0 = (cx - k) as f64 * g.cell;
            let x1 = (cx + k + 1) as f64 * g.cell;
            let y0 = (cy - k) as f64 * g.cell;
            let y1 = (cy + k + 1) as f64 * g.cell;
            let bound = (q.x - x0).min(x1 - q.x).min(q.y - y0).min(y1 - q.y);
            if bound > 0.0 && bd < bound * bound {
                break;
            }
        }
        best
    }

    /// All nodes within `r` of `q`, in ascending index order.
    fn near(&self, q: Point, r: f64) -> Vec<usize> {
        let g = &self.grid;
        let r2 = r * r;
        let (c0, c1) = (g.col(q.x - r), g.col(q.x + r));
        let (r0, r1) = (g.row(q.y - r), g.row(q.y + r));
        let mut out = Vec::new();
        for row in r0..=r1 {
            for col in c0..=c1 {
                for &i in &g.buckets[row * g.cols + col] {
                    let i = i as usize;
                    if dist2(self.pts[i], q) <= r2 {
                        out.push(i);
                    }
                }
            }
        }
        out.sort_unstable();
        out
    }

    /// Change `node`'s parent and propagate the cost change to all descendants.
    fn reparent(&mut self, node: usize, new_parent: usize) {
        let old = self.parent[node];
        if let Some(pos) = self.children[old].iter().position(|&c| c == node) {
            self.children[old].swap_remove(pos);
        }
        self.parent[node] = new_parent;
        self.children[new_parent].push(node);
        let new_cost = self.cost[new_parent] + self.pts[new_parent].dist(self.pts[node]);
        let delta = new_cost - self.cost[node];
        let mut stack = vec![node];
        while let Some(n) = stack.pop() {
            self.cost[n] += delta;
            stack.extend(self.children[n].iter().copied());
        }
    }

    fn path_to(&self, node: usize) -> Vec<Point> {
        let mut out = vec![self.pts[node]];
        let mut cur = node;
        while cur != 0 {
            cur = self.parent[cur];
            out.push(self.pts[cur]);
        }
        out.reverse();
        out
    }
}

fn steer(from: Point, to: Point, step: f64) -> Point {
    let d = from.dist(to);
    if d <= step {
        to
    } else {
        let t = step / d;
        Point::new(from.x + (to.x - from.x) * t, from.y + (to.y - from.y) * t)
    }
}

fn sample(world: &World, goal: Point, bias: f64, rng: &mut SplitMix64) -> Point {
    if rng.next_f64() < bias {
        return goal;
    }
    let r = world.radius;
    let (x0, x1) = (r, (world.width - r).max(r));
    let (y0, y1) = (r, (world.height - r).max(r));
    Point::new(rng.range(x0, x1), rng.range(y0, y1))
}

/// Finish a path: tree path to `node`, then the goal (if not already identical).
fn finish(tree: &Tree, node: usize, goal: Point) -> Vec<Point> {
    let mut path = tree.path_to(node);
    if path.last().map_or(true, |l| l.dist(goal) > 1e-9) {
        path.push(goal);
    }
    path
}

/// Plain RRT: stops as soon as a node within `goal_tolerance` connects to the goal.
pub fn rrt(world: &World, start: Point, goal: Point, p: &RrtParams, seed: u64, log: &mut EventLog) -> PlanOutput {
    let mut out = PlanOutput::default();
    if !world.point_free(start) || !world.point_free(goal) {
        return out;
    }
    let mut rng = SplitMix64::new(seed);
    let mut tree = Tree::new(start, world, p.step_size);
    log.push(Event::Expand { x: start.x, y: start.y, px: start.x, py: start.y });

    if start.dist(goal) <= p.goal_tolerance && world.segment_free(start, goal) {
        out.found = true;
        out.path = finish(&tree, 0, goal);
        out.nodes_explored = 1;
        return out;
    }

    for _ in 0..p.max_iterations {
        out.iterations += 1;
        let q = sample(world, goal, p.goal_bias, &mut rng);
        let ni = tree.nearest(q);
        let from = tree.pts[ni];
        let new = steer(from, q, p.step_size);
        if from.dist(new) < 1e-9 || !world.segment_free(from, new) {
            continue;
        }
        let idx = tree.add(new, ni);
        log.push(Event::Expand { x: new.x, y: new.y, px: from.x, py: from.y });
        if new.dist(goal) <= p.goal_tolerance && world.segment_free(new, goal) {
            out.found = true;
            out.path = finish(&tree, idx, goal);
            break;
        }
    }
    out.nodes_explored = tree.pts.len();
    out
}

/// RRT*: choose-parent + rewiring within `rewire_radius`. Runs the full iteration budget,
/// continually improving the best goal connection; emits `rewire` events on parent changes.
pub fn rrt_star(world: &World, start: Point, goal: Point, p: &RrtParams, seed: u64, log: &mut EventLog) -> PlanOutput {
    let mut out = PlanOutput::default();
    if !world.point_free(start) || !world.point_free(goal) {
        return out;
    }
    let radius = p.rewire_radius.max(p.step_size);
    let mut rng = SplitMix64::new(seed);
    let mut tree = Tree::new(start, world, radius);
    log.push(Event::Expand { x: start.x, y: start.y, px: start.x, py: start.y });

    // Nodes that can connect straight to the goal; best one chosen at the end (costs change).
    let mut goal_nodes: Vec<usize> = Vec::new();
    if start.dist(goal) <= p.goal_tolerance && world.segment_free(start, goal) {
        goal_nodes.push(0);
    }

    for _ in 0..p.max_iterations {
        out.iterations += 1;
        let q = sample(world, goal, p.goal_bias, &mut rng);
        let ni = tree.nearest(q);
        let from = tree.pts[ni];
        let new = steer(from, q, p.step_size);
        if from.dist(new) < 1e-9 || !world.segment_free(from, new) {
            continue;
        }

        // Choose the lowest-cost collision-free parent among neighbours.
        let near = tree.near(new, radius);
        let mut best_parent = ni;
        let mut best_cost = tree.cost[ni] + from.dist(new);
        for &j in &near {
            if j == ni {
                continue;
            }
            let c = tree.cost[j] + tree.pts[j].dist(new);
            if c + 1e-9 < best_cost && world.segment_free(tree.pts[j], new) {
                best_cost = c;
                best_parent = j;
            }
        }
        let idx = tree.add(new, best_parent);
        let pp = tree.pts[best_parent];
        log.push(Event::Expand { x: new.x, y: new.y, px: pp.x, py: pp.y });

        // Rewire neighbours through the new node when it shortens their path.
        for &j in &near {
            if j == best_parent || j == 0 {
                continue;
            }
            let c = tree.cost[idx] + new.dist(tree.pts[j]);
            if c + 1e-9 < tree.cost[j] && world.segment_free(new, tree.pts[j]) {
                let op = tree.pts[tree.parent[j]];
                tree.reparent(j, idx);
                let pj = tree.pts[j];
                log.push(Event::Rewire { x: pj.x, y: pj.y, px: new.x, py: new.y, opx: op.x, opy: op.y });
            }
        }

        if new.dist(goal) <= p.goal_tolerance && world.segment_free(new, goal) {
            goal_nodes.push(idx);
        }
    }

    out.nodes_explored = tree.pts.len();
    let best = goal_nodes
        .iter()
        .copied()
        .min_by(|&a, &b| {
            let ca = tree.cost[a] + tree.pts[a].dist(goal);
            let cb = tree.cost[b] + tree.pts[b].dist(goal);
            ca.total_cmp(&cb).then(a.cmp(&b))
        });
    if let Some(b) = best {
        out.found = true;
        out.path = finish(&tree, b, goal);
    }
    out
}

#[cfg(test)]
mod grid_tests {
    use super::*;

    #[test]
    fn grid_queries_match_linear_scan() {
        let world = World::new(1000.0, 640.0, 12.0, Vec::new());
        for &cell in &[25.0, 60.0] {
            let mut rng = SplitMix64::new(11);
            let mut tree = Tree::new(Point::new(500.0, 320.0), &world, cell);
            for _ in 0..4000 {
                let p = Point::new(rng.range(0.0, 1000.0), rng.range(0.0, 640.0));
                tree.add(p, 0);
            }
            for _ in 0..3000 {
                // Include queries outside the world, as RRT never makes but the grid must handle.
                let q = Point::new(rng.range(-100.0, 1100.0), rng.range(-100.0, 740.0));
                let mut best = 0;
                let mut bd = f64::INFINITY;
                for (i, p) in tree.pts.iter().enumerate() {
                    let d = dist2(*p, q);
                    if d < bd {
                        bd = d;
                        best = i;
                    }
                }
                assert_eq!(tree.nearest(q), best);
                let want: Vec<usize> = (0..tree.pts.len()).filter(|&i| dist2(tree.pts[i], q) <= cell * cell).collect();
                assert_eq!(tree.near(q, cell), want);
            }
        }
    }
}
