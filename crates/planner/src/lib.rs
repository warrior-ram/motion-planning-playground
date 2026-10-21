//! Mobile-robot motion planners (A*, RRT, RRT*) with a thin JSON / wasm-bindgen layer.
//!
//! The algorithms live in plain Rust modules so the native benchmark binary reuses them.

pub mod astar;
pub mod events;
pub mod geometry;
pub mod maps;
pub mod rng;
pub mod rrt;

#[cfg(test)]
mod tests;

use serde::{Deserialize, Serialize};
use wasm_bindgen::prelude::*;

use events::{Event, EventLog, PlanOutput};
use geometry::{path_length, Obstacle, Point, World};
use rrt::RrtParams;

/// Hard limits that keep a single call bounded (and panic/OOM free) inside the browser.
pub const MAX_WORLD_DIM: f64 = 100_000.0;
pub const MAX_GRID_CELLS: f64 = 4_000_000.0;
pub const MAX_ITERATIONS: usize = 1_000_000;
pub const MAX_OBSTACLES: usize = 10_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Algorithm {
    AStar,
    Rrt,
    RrtStar,
}

impl Algorithm {
    pub fn parse(s: &str) -> Option<Self> {
        match s.to_ascii_lowercase().as_str() {
            "astar" => Some(Self::AStar),
            "rrt" => Some(Self::Rrt),
            "rrtstar" => Some(Self::RrtStar),
            _ => None,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Self::AStar => "astar",
            Self::Rrt => "rrt",
            Self::RrtStar => "rrtstar",
        }
    }
}

#[derive(Deserialize, Default)]
#[serde(rename_all = "camelCase")]
struct RawParams {
    grid_size: Option<f64>,
    max_iterations: Option<f64>,
    step_size: Option<f64>,
    goal_bias: Option<f64>,
    goal_tolerance: Option<f64>,
    rewire_radius: Option<f64>,
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct RawRequest {
    algorithm: String,
    width: Option<f64>,
    height: Option<f64>,
    #[serde(default)]
    obstacles: Vec<Obstacle>,
    start: Point,
    goal: Point,
    robot_radius: Option<f64>,
    seed: Option<f64>,
    #[serde(default)]
    params: Option<RawParams>,
}

/// Fully validated planning request.
#[derive(Clone, Debug)]
pub struct PlanRequest {
    pub algorithm: Algorithm,
    pub world: World,
    pub start: Point,
    pub goal: Point,
    pub seed: u64,
    pub grid_size: f64,
    pub rrt: RrtParams,
}

#[derive(Serialize, Debug)]
#[serde(rename_all = "camelCase")]
pub struct PlanResponse {
    pub found: bool,
    pub path: Vec<Point>,
    pub path_length: f64,
    pub nodes_explored: usize,
    pub iterations: usize,
    pub events: Vec<Event>,
}

fn finite_pos(name: &str, v: f64, allow_zero: bool) -> Result<f64, String> {
    if !v.is_finite() || v < 0.0 || (!allow_zero && v == 0.0) {
        let want = if allow_zero { ">= 0" } else { "> 0" };
        return Err(format!("{name} must be a finite number {want}"));
    }
    Ok(v)
}

pub fn parse_request(req: &str) -> Result<PlanRequest, String> {
    let raw: RawRequest = serde_json::from_str(req).map_err(|e| format!("invalid request JSON: {e}"))?;
    let algorithm = Algorithm::parse(&raw.algorithm)
        .ok_or_else(|| format!("unknown algorithm '{}' (expected astar, rrt or rrtstar)", raw.algorithm))?;

    let width = finite_pos("width", raw.width.unwrap_or(1000.0), false)?;
    let height = finite_pos("height", raw.height.unwrap_or(640.0), false)?;
    if width > MAX_WORLD_DIM || height > MAX_WORLD_DIM {
        return Err(format!("width/height must be <= {MAX_WORLD_DIM}"));
    }
    let radius = finite_pos("robotRadius", raw.robot_radius.unwrap_or(12.0), true)?;
    if raw.obstacles.len() > MAX_OBSTACLES {
        return Err(format!("too many obstacles (max {MAX_OBSTACLES})"));
    }
    for o in &raw.obstacles {
        o.validate()?;
    }
    if !raw.start.is_finite() || !raw.goal.is_finite() {
        return Err("start/goal must have finite coordinates".into());
    }
    let seed_f = raw.seed.unwrap_or(1.0);
    if !seed_f.is_finite() {
        return Err("seed must be a finite number".into());
    }
    let seed = (seed_f.trunc() as i64) as u64;

    let p = raw.params.unwrap_or_default();
    let grid_size = finite_pos("params.gridSize", p.grid_size.unwrap_or(10.0), false)?;
    if (width / grid_size).ceil() * (height / grid_size).ceil() > MAX_GRID_CELLS {
        return Err(format!("params.gridSize too small: grid would exceed {MAX_GRID_CELLS} cells"));
    }
    let max_it = finite_pos("params.maxIterations", p.max_iterations.unwrap_or(20000.0), true)?;
    if max_it > MAX_ITERATIONS as f64 {
        return Err(format!("params.maxIterations must be <= {MAX_ITERATIONS}"));
    }
    let step_size = finite_pos("params.stepSize", p.step_size.unwrap_or(25.0), false)?;
    let goal_bias = p.goal_bias.unwrap_or(0.05);
    if !goal_bias.is_finite() || !(0.0..=1.0).contains(&goal_bias) {
        return Err("params.goalBias must be in [0, 1]".into());
    }
    let goal_tolerance = finite_pos("params.goalTolerance", p.goal_tolerance.unwrap_or(20.0), true)?;
    let rewire_radius = finite_pos("params.rewireRadius", p.rewire_radius.unwrap_or(60.0), true)?;

    Ok(PlanRequest {
        algorithm,
        world: World::new(width, height, radius, raw.obstacles),
        start: raw.start,
        goal: raw.goal,
        seed,
        grid_size,
        rrt: RrtParams {
            max_iterations: max_it as usize,
            step_size,
            goal_bias,
            goal_tolerance,
            rewire_radius,
        },
    })
}

/// Run a validated request. `record_events = false` skips the replay log (used by the bench).
pub fn run(req: &PlanRequest, record_events: bool) -> PlanResponse {
    let mut log = EventLog::new(record_events);
    let out: PlanOutput = match req.algorithm {
        Algorithm::AStar => astar::astar(&req.world, req.start, req.goal, req.grid_size, &mut log),
        Algorithm::Rrt => rrt::rrt(&req.world, req.start, req.goal, &req.rrt, req.seed, &mut log),
        Algorithm::RrtStar => rrt::rrt_star(&req.world, req.start, req.goal, &req.rrt, req.seed, &mut log),
    };
    let path_length = if out.found { path_length(&out.path) } else { 0.0 };
    PlanResponse {
        found: out.found,
        path: if out.found { out.path } else { Vec::new() },
        path_length,
        nodes_explored: out.nodes_explored,
        iterations: out.iterations,
        events: log.events,
    }
}

fn error_json(msg: &str) -> String {
    serde_json::json!({ "error": msg }).to_string()
}

/// Plain-Rust entry point behind the wasm export.
pub fn plan_json(req: &str) -> String {
    match parse_request(req) {
        Ok(r) => serde_json::to_string(&run(&r, true)).unwrap_or_else(|e| error_json(&e.to_string())),
        Err(e) => error_json(&e),
    }
}

pub fn benchmark_maps_json() -> String {
    serde_json::to_string(&maps::benchmark_maps()).unwrap_or_else(|e| error_json(&e.to_string()))
}

#[wasm_bindgen]
pub fn plan(req: &str) -> String {
    plan_json(req)
}

#[wasm_bindgen]
pub fn benchmark_maps() -> String {
    benchmark_maps_json()
}
