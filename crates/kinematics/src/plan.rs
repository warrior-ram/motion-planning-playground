//! Joint-space RRT / RRT* for the planar arm.

use crate::arm::Arm;
use crate::geom::{angle_diff, wrap_angle, Shape};
use crate::rng::SplitMix64;
use std::f64::consts::PI;

pub const MAX_EVENTS: usize = 20_000;
/// Max per-joint step between collision checks along an edge (rad).
pub const EDGE_CHECK_STEP: f64 = 0.02;
/// Max per-joint step in the densified output path (rad).
pub const PATH_DENSIFY_STEP: f64 = 0.05;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Algorithm {
    Rrt,
    RrtStar,
}

#[derive(Clone, Debug)]
pub struct PlanParams {
    pub max_iterations: usize,
    pub step_size: f64,
    pub goal_bias: f64,
    pub goal_tolerance: f64,
    pub rewire_radius: f64,
}

impl Default for PlanParams {
    fn default() -> Self {
        PlanParams {
            max_iterations: 10000,
            step_size: 0.15,
            goal_bias: 0.1,
            goal_tolerance: 0.1,
            rewire_radius: 0.5,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum Event {
    Expand { q: Vec<f64>, pq: Vec<f64> },
    Rewire { q: Vec<f64>, pq: Vec<f64>, opq: Vec<f64> },
}

#[derive(Clone, Debug)]
pub struct PlanResult {
    pub found: bool,
    pub path: Vec<Vec<f64>>,
    pub nodes_explored: usize,
    pub iterations: usize,
    pub events: Vec<Event>,
    /// Set when the planner refused to run (e.g. start/goal in collision).
    pub note: Option<String>,
}

pub struct World<'a> {
    pub arm: &'a Arm,
    pub obstacles: &'a [Shape],
    pub radius: f64,
}

impl<'a> World<'a> {
    pub fn valid(&self, q: &[f64]) -> bool {
        self.arm.config_valid(q, self.obstacles, self.radius)
    }

    /// Check the joint-space edge a -> b (shortest angular way) at <= EDGE_CHECK_STEP per joint.
    pub fn edge_valid(&self, a: &[f64], b: &[f64]) -> bool {
        let span = max_abs_diff(a, b);
        let steps = (span / EDGE_CHECK_STEP).ceil().max(1.0) as usize;
        let mut q = vec![0.0; a.len()];
        for s in 1..=steps {
            interp_into(a, b, s as f64 / steps as f64, &mut q);
            if !self.valid(&q) {
                return false;
            }
        }
        true
    }
}

/// Largest absolute per-joint shortest angular difference.
pub fn max_abs_diff(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).map(|(x, y)| angle_diff(*x, *y).abs()).fold(0.0, f64::max)
}

/// Euclidean distance on the torus (shortest angular differences).
pub fn dist(a: &[f64], b: &[f64]) -> f64 {
    a.iter()
        .zip(b)
        .map(|(x, y)| {
            let d = angle_diff(*x, *y);
            d * d
        })
        .sum::<f64>()
        .sqrt()
}

pub fn interp_into(a: &[f64], b: &[f64], t: f64, out: &mut [f64]) {
    for i in 0..a.len() {
        out[i] = wrap_angle(a[i] + angle_diff(a[i], b[i]) * t);
    }
}

pub fn interp(a: &[f64], b: &[f64], t: f64) -> Vec<f64> {
    let mut out = vec![0.0; a.len()];
    interp_into(a, b, t, &mut out);
    out
}

/// Densify so consecutive configs differ by <= `step` rad on every joint.
pub fn densify(path: &[Vec<f64>], step: f64) -> Vec<Vec<f64>> {
    let mut out = Vec::new();
    if path.is_empty() {
        return out;
    }
    out.push(path[0].clone());
    for w in path.windows(2) {
        let span = max_abs_diff(&w[0], &w[1]);
        // Small safety margin so floating error never exceeds `step`.
        let n = (span / (step * 0.999)).ceil().max(1.0) as usize;
        for s in 1..=n {
            if s == n {
                out.push(w[1].clone());
            } else {
                out.push(interp(&w[0], &w[1], s as f64 / n as f64));
            }
        }
    }
    out
}

struct Node {
    q: Vec<f64>,
    parent: usize,
    cost: f64,
    children: Vec<usize>,
}

fn add_node(nodes: &mut Vec<Node>, q: Vec<f64>, parent: usize, cost: f64) -> usize {
    let idx = nodes.len();
    nodes.push(Node { q, parent, cost, children: Vec::new() });
    if idx != 0 {
        nodes[parent].children.push(idx);
    }
    idx
}

fn reparent(nodes: &mut [Node], i: usize, new_parent: usize) {
    let old = nodes[i].parent;
    nodes[old].children.retain(|&c| c != i);
    nodes[i].parent = new_parent;
    nodes[new_parent].children.push(i);
}

struct Events {
    list: Vec<Event>,
}

impl Events {
    fn push(&mut self, e: Event) {
        if self.list.len() < MAX_EVENTS {
            self.list.push(e);
        }
    }
}

/// Uniform bucket grid over the joint-space torus [-PI, PI)^dim, used for nearest / radius
/// queries. Queries return exactly what a linear scan over node indices would (same ties,
/// same order), so planner output does not depend on whether the grid is used.
struct TorusGrid {
    dim: usize,
    /// Cells per axis.
    n: usize,
    cell: f64,
    buckets: Vec<Vec<u32>>,
}

/// Total cell budget; keeps the grid small for 3+ joints.
const MAX_TORUS_CELLS: usize = 32_768;

impl TorusGrid {
    fn new(dim: usize, min_cell: f64) -> Self {
        let tau = 2.0 * PI;
        // Cells at least `min_cell` wide (with a margin against rounding), within the budget.
        let mut n = ((tau / (min_cell * 1.001)).floor() as usize).max(1);
        while n > 1 && n.checked_pow(dim as u32).map_or(true, |c| c > MAX_TORUS_CELLS) {
            n -= 1;
        }
        let total = n.pow(dim as u32);
        TorusGrid { dim, n, cell: tau / n as f64, buckets: vec![Vec::new(); total] }
    }

    fn axis_cell(&self, a: f64) -> usize {
        (((wrap_angle(a) + PI) / self.cell).floor().max(0.0) as usize).min(self.n - 1)
    }

    fn index(&self, cells: &[usize]) -> usize {
        cells.iter().fold(0, |acc, &c| acc * self.n + c)
    }

    fn insert(&mut self, q: &[f64], idx: usize) {
        let cells: Vec<usize> = q.iter().map(|&a| self.axis_cell(a)).collect();
        let i = self.index(&cells);
        self.buckets[i].push(idx as u32);
    }

    /// Distinct wrapped cells within `k` of `c` along one axis.
    fn axis_span(&self, c: usize, k: usize) -> Vec<usize> {
        if 2 * k + 1 >= self.n {
            return (0..self.n).collect();
        }
        let n = self.n as isize;
        (-(k as isize)..=k as isize).map(|o| (c as isize + o).rem_euclid(n) as usize).collect()
    }

    /// Visit every bucket whose cell offset from `c` is within `k` on each axis; when `shell`
    /// is set, only buckets at offset exactly `k` on at least one axis.
    fn visit(&self, c: &[usize], k: usize, shell: bool, f: &mut impl FnMut(&[u32])) {
        let full = 2 * k + 1 >= self.n;
        let spans: Vec<Vec<usize>> = c.iter().map(|&ci| self.axis_span(ci, k)).collect();
        let mut pick = vec![0usize; self.dim];
        let mut cells = vec![0usize; self.dim];
        loop {
            let mut on_shell = false;
            for d in 0..self.dim {
                cells[d] = spans[d][pick[d]];
                if !full && (pick[d] == 0 || pick[d] == spans[d].len() - 1) {
                    on_shell = true;
                }
            }
            if !shell || full || on_shell || k == 0 {
                f(&self.buckets[self.index(&cells)]);
            }
            let mut d = 0;
            loop {
                if d == self.dim {
                    return;
                }
                pick[d] += 1;
                if pick[d] < spans[d].len() {
                    break;
                }
                pick[d] = 0;
                d += 1;
            }
        }
    }

    /// Lower bound on the distance from `q` to any point outside the searched block of
    /// half-width `k` cells (per axis, wrapped). Infinite once the block covers the torus.
    fn outside_bound(&self, q: &[f64], c: &[usize], k: usize) -> f64 {
        if 2 * k + 1 >= self.n {
            return f64::INFINITY;
        }
        let mut b = f64::INFINITY;
        for (d, &a) in q.iter().enumerate() {
            let x = wrap_angle(a) + PI;
            let lo = (c[d] as f64 - k as f64) * self.cell;
            let hi = (c[d] as f64 + k as f64 + 1.0) * self.cell;
            b = b.min(x - lo).min(hi - x);
        }
        // Margin against rounding in cell assignment.
        b - 1e-9
    }
}

/// Nearest node to `q`; on equal distance the lowest index wins (as a linear scan would).
fn nearest(nodes: &[Node], grid: &TorusGrid, q: &[f64]) -> usize {
    let c: Vec<usize> = q.iter().map(|&a| grid.axis_cell(a)).collect();
    let mut best = 0;
    let mut bd = f64::INFINITY;
    for k in 0..=grid.n {
        grid.visit(&c, k, true, &mut |bucket| {
            for &i in bucket {
                let i = i as usize;
                let d = dist(&nodes[i].q, q);
                if d < bd || (d == bd && i < best) {
                    bd = d;
                    best = i;
                }
            }
        });
        let bound = grid.outside_bound(q, &c, k);
        if bound == f64::INFINITY || (bound > 0.0 && bd < bound) {
            break;
        }
    }
    best
}

/// All nodes within `r` of `q`, in ascending index order. Requires `grid.cell >= r`.
fn near_nodes(nodes: &[Node], grid: &TorusGrid, q: &[f64], r: f64) -> Vec<usize> {
    let c: Vec<usize> = q.iter().map(|&a| grid.axis_cell(a)).collect();
    let mut out = Vec::new();
    grid.visit(&c, 1, false, &mut |bucket| {
        for &i in bucket {
            let i = i as usize;
            if dist(&nodes[i].q, q) <= r {
                out.push(i);
            }
        }
    });
    out.sort_unstable();
    out
}

fn steer(from: &[f64], to: &[f64], step: f64) -> Vec<f64> {
    let d = dist(from, to);
    if d <= step {
        to.iter().map(|a| wrap_angle(*a)).collect()
    } else {
        interp(from, to, step / d)
    }
}

/// Plan from `start` to `goal` in joint space.
pub fn plan(
    world: &World,
    start: &[f64],
    goal: &[f64],
    algorithm: Algorithm,
    seed: u64,
    params: &PlanParams,
) -> PlanResult {
    let start: Vec<f64> = start.iter().map(|a| wrap_angle(*a)).collect();
    let goal: Vec<f64> = goal.iter().map(|a| wrap_angle(*a)).collect();
    let mut ev = Events { list: Vec::new() };
    let fail = |note: &str, ev: Events, nodes: usize, it: usize| PlanResult {
        found: false,
        path: Vec::new(),
        nodes_explored: nodes,
        iterations: it,
        events: ev.list,
        note: Some(note.to_string()),
    };
    if !world.valid(&start) {
        return fail("start configuration is in collision", ev, 0, 0);
    }
    if !world.valid(&goal) {
        return fail("goal configuration is in collision", ev, 0, 0);
    }

    let dim = start.len();
    let mut rng = SplitMix64::new(seed);
    let step = params.step_size.max(1e-3);
    let goal_tol = params.goal_tolerance.max(0.0);
    let r = params.rewire_radius.max(step);
    let mut grid = TorusGrid::new(dim, if algorithm == Algorithm::RrtStar { r } else { step });
    let mut nodes: Vec<Node> = Vec::new();
    add_node(&mut nodes, start.clone(), 0, 0.0);
    grid.insert(&start, 0);
    ev.push(Event::Expand { q: start.clone(), pq: start.clone() });

    // Trivial case: direct connection.
    if world.edge_valid(&start, &goal) && algorithm == Algorithm::Rrt {
        ev.push(Event::Expand { q: goal.clone(), pq: start.clone() });
        let path = densify(&[start.clone(), goal.clone()], PATH_DENSIFY_STEP);
        return PlanResult {
            found: true,
            path,
            nodes_explored: 2,
            iterations: 0,
            events: ev.list,
            note: None,
        };
    }

    let mut goal_node: Option<usize> = None;
    let mut iterations = 0;
    let mut sample = vec![0.0; dim];

    for it in 0..params.max_iterations {
        iterations = it + 1;
        if rng.next_f64() < params.goal_bias {
            sample.copy_from_slice(&goal);
        } else {
            for s in sample.iter_mut() {
                *s = wrap_angle(-PI + rng.next_f64() * 2.0 * PI);
            }
        }
        let ni = nearest(&nodes, &grid, &sample);
        let qnew = steer(&nodes[ni].q, &sample, step);
        if !world.valid(&qnew) || !world.edge_valid(&nodes[ni].q, &qnew) {
            continue;
        }

        let mut parent = ni;
        let mut cost = nodes[ni].cost + dist(&nodes[ni].q, &qnew);
        let mut near: Vec<usize> = Vec::new();
        if algorithm == Algorithm::RrtStar {
            near = near_nodes(&nodes, &grid, &qnew, r);
            for &i in &near {
                if i == ni {
                    continue;
                }
                let c = nodes[i].cost + dist(&nodes[i].q, &qnew);
                if c < cost && world.edge_valid(&nodes[i].q, &qnew) {
                    cost = c;
                    parent = i;
                }
            }
        }
        ev.push(Event::Expand { q: qnew.clone(), pq: nodes[parent].q.clone() });
        let new_idx = add_node(&mut nodes, qnew.clone(), parent, cost);
        grid.insert(&qnew, new_idx);

        if algorithm == Algorithm::RrtStar {
            for &i in &near {
                if i == parent {
                    continue;
                }
                let c = cost + dist(&qnew, &nodes[i].q);
                if c + 1e-12 < nodes[i].cost && world.edge_valid(&qnew, &nodes[i].q) {
                    let old = nodes[i].parent;
                    ev.push(Event::Rewire {
                        q: nodes[i].q.clone(),
                        pq: qnew.clone(),
                        opq: nodes[old].q.clone(),
                    });
                    let delta = nodes[i].cost - c;
                    reparent(&mut nodes, i, new_idx);
                    // Propagate the cost decrease to descendants.
                    propagate(&mut nodes, i, delta);
                }
            }
        }

        // Goal connection.
        let dg = dist(&qnew, &goal);
        if dg <= goal_tol.max(1e-9) || (dg <= step && world.edge_valid(&qnew, &goal)) {
            match algorithm {
                Algorithm::Rrt => {
                    let gi = attach_goal(&mut nodes, &mut ev, new_idx, &goal);
                    goal_node = Some(gi);
                    break;
                }
                Algorithm::RrtStar => {
                    let c = nodes[new_idx].cost + dg;
                    let better = goal_node.map_or(true, |g| c < nodes[g].cost);
                    if better {
                        match goal_node {
                            None => goal_node = Some(attach_goal(&mut nodes, &mut ev, new_idx, &goal)),
                            Some(g) => {
                                let old = nodes[g].parent;
                                ev.push(Event::Rewire {
                                    q: goal.clone(),
                                    pq: nodes[new_idx].q.clone(),
                                    opq: nodes[old].q.clone(),
                                });
                                let delta = nodes[g].cost - c;
                                reparent(&mut nodes, g, new_idx);
                                propagate(&mut nodes, g, delta);
                            }
                        }
                    }
                }
            }
        }
    }

    match goal_node {
        None => PlanResult {
            found: false,
            path: Vec::new(),
            nodes_explored: nodes.len(),
            iterations,
            events: ev.list,
            note: None,
        },
        Some(g) => {
            // Recompute goal cost via parent chain (rewires may have improved ancestors).
            let mut raw = Vec::new();
            let mut i = g;
            let mut guard = 0;
            loop {
                raw.push(nodes[i].q.clone());
                if i == 0 || guard > nodes.len() {
                    break;
                }
                i = nodes[i].parent;
                guard += 1;
            }
            raw.reverse();
            if let Some(last) = raw.last_mut() {
                *last = goal.clone();
            }
            PlanResult {
                found: true,
                path: densify(&raw, PATH_DENSIFY_STEP),
                nodes_explored: nodes.len(),
                iterations,
                events: ev.list,
                note: None,
            }
        }
    }
}

fn attach_goal(nodes: &mut Vec<Node>, ev: &mut Events, from: usize, goal: &[f64]) -> usize {
    let c = nodes[from].cost + dist(&nodes[from].q, goal);
    ev.push(Event::Expand { q: goal.to_vec(), pq: nodes[from].q.clone() });
    add_node(nodes, goal.to_vec(), from, c)
}

fn propagate(nodes: &mut [Node], root: usize, delta: f64) {
    let mut stack = vec![root];
    let mut visited = 0usize;
    while let Some(i) = stack.pop() {
        visited += 1;
        if visited > nodes.len() {
            break; // cycle guard (cannot happen: rewires only decrease cost)
        }
        nodes[i].cost -= delta;
        stack.extend(nodes[i].children.iter().copied());
    }
}

#[cfg(test)]
mod grid_tests {
    use super::*;

    fn linear_nearest(nodes: &[Node], q: &[f64]) -> usize {
        let mut best = 0;
        let mut bd = f64::INFINITY;
        for (i, n) in nodes.iter().enumerate() {
            let d = dist(&n.q, q);
            if d < bd {
                bd = d;
                best = i;
            }
        }
        best
    }

    #[test]
    fn grid_queries_match_linear_scan() {
        for dim in [2usize, 3] {
            for &cell in &[0.15, 0.5] {
                let mut rng = SplitMix64::new(7 + dim as u64);
                let mut grid = TorusGrid::new(dim, cell);
                let mut nodes: Vec<Node> = Vec::new();
                for _ in 0..3000 {
                    let q: Vec<f64> = (0..dim).map(|_| wrap_angle(-PI + rng.next_f64() * 2.0 * PI)).collect();
                    let idx = add_node(&mut nodes, q.clone(), 0, 0.0);
                    grid.insert(&q, idx);
                }
                for _ in 0..2000 {
                    let q: Vec<f64> = (0..dim).map(|_| wrap_angle(-PI + rng.next_f64() * 2.0 * PI)).collect();
                    assert_eq!(nearest(&nodes, &grid, &q), linear_nearest(&nodes, &q));
                    let want: Vec<usize> = (0..nodes.len()).filter(|&i| dist(&nodes[i].q, &q) <= cell).collect();
                    assert_eq!(near_nodes(&nodes, &grid, &q, cell), want);
                }
            }
        }
    }
}
