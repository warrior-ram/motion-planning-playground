//! Planar 2/3-link arm kinematics + joint-space planning, exported to WASM.
//!
//! All exports take a JSON string and return a JSON string (camelCase), per CONTRACT.md.
//! Bad input returns `{"error": "..."}`; nothing panics across the wasm boundary.

pub mod arm;
pub mod geom;
pub mod plan;
pub mod rng;

use arm::{solve_ik, Arm, IkOptions};
use geom::{Shape, V2};
use plan::{Algorithm, Event, PlanParams, World};
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use wasm_bindgen::prelude::*;

// ---------- JSON types ----------

#[derive(Deserialize, Serialize, Clone, Copy, Debug)]
pub struct Point {
    pub x: f64,
    pub y: f64,
}

#[derive(Deserialize, Clone, Debug)]
#[serde(tag = "kind", rename_all = "lowercase")]
pub enum Obstacle {
    Rect { x: f64, y: f64, w: f64, h: f64 },
    Circle { x: f64, y: f64, r: f64 },
}

impl Obstacle {
    fn shape(&self) -> Shape {
        match *self {
            Obstacle::Rect { x, y, w, h } => Shape::rect(x, y, w, h),
            Obstacle::Circle { x, y, r } => Shape::circle(x, y, r),
        }
    }
}

#[derive(Deserialize)]
struct FkReq {
    base: Point,
    links: Vec<f64>,
    angles: Vec<f64>,
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct IkReq {
    base: Point,
    links: Vec<f64>,
    angles: Vec<f64>,
    target: Point,
    #[serde(default)]
    limits: Option<Vec<[f64; 2]>>,
    #[serde(default)]
    max_iterations: Option<usize>,
    #[serde(default)]
    tolerance: Option<f64>,
}

#[derive(Deserialize, Default)]
#[serde(rename_all = "camelCase")]
struct PlanParamsReq {
    max_iterations: Option<usize>,
    step_size: Option<f64>,
    goal_bias: Option<f64>,
    goal_tolerance: Option<f64>,
    rewire_radius: Option<f64>,
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct PlanReq {
    base: Point,
    links: Vec<f64>,
    #[serde(default)]
    obstacles: Vec<Obstacle>,
    #[serde(default)]
    link_thickness: Option<f64>,
    start: Vec<f64>,
    goal: Vec<f64>,
    #[serde(default)]
    algorithm: Option<String>,
    #[serde(default)]
    seed: Option<u64>,
    #[serde(default)]
    params: Option<PlanParamsReq>,
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct ValidReq {
    base: Point,
    links: Vec<f64>,
    #[serde(default)]
    obstacles: Vec<Obstacle>,
    #[serde(default)]
    link_thickness: Option<f64>,
    angles: Vec<f64>,
}

const DEFAULT_THICKNESS: f64 = 8.0;
const MAX_ITERATIONS_CAP: usize = 200_000;

// ---------- helpers ----------

fn err(msg: impl Into<String>) -> String {
    json!({ "error": msg.into() }).to_string()
}

fn finite(v: f64) -> bool {
    v.is_finite()
}

fn check_point(p: &Point, name: &str) -> Result<V2, String> {
    if finite(p.x) && finite(p.y) {
        Ok(V2::new(p.x, p.y))
    } else {
        Err(format!("{name} must have finite x and y"))
    }
}

fn check_arm(base: &Point, links: &[f64]) -> Result<Arm, String> {
    let b = check_point(base, "base")?;
    if links.len() < 2 || links.len() > 3 {
        return Err(format!("links must have 2 or 3 entries (got {})", links.len()));
    }
    if links.iter().any(|l| !finite(*l) || *l <= 0.0) {
        return Err("link lengths must be finite and > 0".into());
    }
    Ok(Arm::new(b, links.to_vec()))
}

fn check_angles(a: &[f64], n: usize, name: &str) -> Result<(), String> {
    if a.len() != n {
        return Err(format!("{name} must have {n} entries (one per link), got {}", a.len()));
    }
    if a.iter().any(|v| !finite(*v)) {
        return Err(format!("{name} must be finite numbers"));
    }
    Ok(())
}

fn check_obstacles(obs: &[Obstacle]) -> Result<Vec<Shape>, String> {
    for o in obs {
        let ok = match *o {
            Obstacle::Rect { x, y, w, h } => [x, y, w, h].iter().all(|v| finite(*v)),
            Obstacle::Circle { x, y, r } => [x, y, r].iter().all(|v| finite(*v)),
        };
        if !ok {
            return Err("obstacle fields must be finite numbers".into());
        }
    }
    Ok(obs.iter().map(Obstacle::shape).collect())
}

fn check_thickness(t: Option<f64>) -> Result<f64, String> {
    let t = t.unwrap_or(DEFAULT_THICKNESS);
    if !finite(t) || t < 0.0 {
        return Err("linkThickness must be finite and >= 0".into());
    }
    Ok(t)
}

fn pts(v: &[V2]) -> Vec<Point> {
    v.iter().map(|p| Point { x: p.x, y: p.y }).collect()
}

fn parse<T: for<'de> Deserialize<'de>>(req: &str) -> Result<T, String> {
    serde_json::from_str(req).map_err(|e| format!("invalid request: {e}"))
}

/// Run `f`, converting any (unexpected) panic into an error JSON. On wasm32 with
/// panic=abort this cannot catch, so all code paths are written to be panic-free anyway.
fn guarded(f: impl FnOnce() -> Result<String, String> + std::panic::UnwindSafe) -> String {
    match std::panic::catch_unwind(f) {
        Ok(Ok(s)) => s,
        Ok(Err(e)) => err(e),
        Err(_) => err("internal error"),
    }
}

// ---------- pure (native-testable) API ----------

pub fn fk_json(req: &str) -> String {
    let req = req.to_owned();
    guarded(move || {
        let r: FkReq = parse(&req)?;
        let arm = check_arm(&r.base, &r.links)?;
        check_angles(&r.angles, r.links.len(), "angles")?;
        Ok(json!({ "joints": pts(&arm.fk(&r.angles)) }).to_string())
    })
}

pub fn ik_json(req: &str) -> String {
    let req = req.to_owned();
    guarded(move || {
        let r: IkReq = parse(&req)?;
        let arm = check_arm(&r.base, &r.links)?;
        let n = r.links.len();
        check_angles(&r.angles, n, "angles")?;
        let target = check_point(&r.target, "target")?;
        let limits = match r.limits {
            None => None,
            Some(l) => {
                if l.len() != n {
                    return Err(format!("limits must have {n} entries, got {}", l.len()));
                }
                if l.iter().any(|[a, b]| !finite(*a) || !finite(*b) || a > b) {
                    return Err("each limit must be [min, max] with finite min <= max".into());
                }
                Some(l.into_iter().map(|[a, b]| (a, b)).collect())
            }
        };
        let tolerance = r.tolerance.unwrap_or(0.5);
        if !finite(tolerance) || tolerance < 0.0 {
            return Err("tolerance must be finite and >= 0".into());
        }
        let opts = IkOptions {
            limits,
            max_iterations: r.max_iterations.unwrap_or(100).min(10_000),
            tolerance,
        };
        let res = solve_ik(&arm, &r.angles, target, &opts);
        Ok(json!({
            "angles": res.angles,
            "joints": pts(&res.joints),
            "reached": res.reached,
            "error": res.error,
            "iterations": res.iterations,
        })
        .to_string())
    })
}

fn event_json(e: &Event) -> Value {
    match e {
        Event::Expand { q, pq } => json!({ "k": "expand", "q": q, "pq": pq }),
        Event::Rewire { q, pq, opq } => json!({ "k": "rewire", "q": q, "pq": pq, "opq": opq }),
    }
}

pub fn plan_arm_json(req: &str) -> String {
    let req = req.to_owned();
    guarded(move || {
        let r: PlanReq = parse(&req)?;
        let arm = check_arm(&r.base, &r.links)?;
        let n = r.links.len();
        check_angles(&r.start, n, "start")?;
        check_angles(&r.goal, n, "goal")?;
        let shapes = check_obstacles(&r.obstacles)?;
        let thickness = check_thickness(r.link_thickness)?;
        let algorithm = match r.algorithm.as_deref().unwrap_or("rrt") {
            "rrt" => Algorithm::Rrt,
            "rrtstar" => Algorithm::RrtStar,
            other => return Err(format!("unknown algorithm '{other}' (expected rrt or rrtstar)")),
        };
        let d = PlanParams::default();
        let p = r.params.unwrap_or_default();
        let params = PlanParams {
            max_iterations: p.max_iterations.unwrap_or(d.max_iterations).min(MAX_ITERATIONS_CAP),
            step_size: p.step_size.unwrap_or(d.step_size),
            goal_bias: p.goal_bias.unwrap_or(d.goal_bias),
            goal_tolerance: p.goal_tolerance.unwrap_or(d.goal_tolerance),
            rewire_radius: p.rewire_radius.unwrap_or(d.rewire_radius),
        };
        if !(finite(params.step_size) && params.step_size > 0.0) {
            return Err("params.stepSize must be > 0".into());
        }
        if !(finite(params.goal_bias) && (0.0..=1.0).contains(&params.goal_bias)) {
            return Err("params.goalBias must be in [0, 1]".into());
        }
        if !(finite(params.goal_tolerance) && params.goal_tolerance >= 0.0) {
            return Err("params.goalTolerance must be >= 0".into());
        }
        if !(finite(params.rewire_radius) && params.rewire_radius >= 0.0) {
            return Err("params.rewireRadius must be >= 0".into());
        }
        let world = World { arm: &arm, obstacles: &shapes, radius: thickness / 2.0 };
        let res = plan::plan(&world, &r.start, &r.goal, algorithm, r.seed.unwrap_or(1), &params);
        let mut out = json!({
            "found": res.found,
            "path": res.path,
            "nodesExplored": res.nodes_explored,
            "iterations": res.iterations,
            "events": res.events.iter().map(event_json).collect::<Vec<_>>(),
        });
        if let Some(note) = res.note {
            out["reason"] = Value::String(note);
        }
        Ok(out.to_string())
    })
}

pub fn arm_config_valid_json(req: &str) -> String {
    let req = req.to_owned();
    guarded(move || {
        let r: ValidReq = parse(&req)?;
        let arm = check_arm(&r.base, &r.links)?;
        check_angles(&r.angles, r.links.len(), "angles")?;
        let shapes = check_obstacles(&r.obstacles)?;
        let thickness = check_thickness(r.link_thickness)?;
        let valid = arm.config_valid(&r.angles, &shapes, thickness / 2.0);
        Ok(json!({ "valid": valid }).to_string())
    })
}

// ---------- thin wasm-bindgen layer ----------

#[wasm_bindgen]
pub fn fk(req: &str) -> String {
    fk_json(req)
}

#[wasm_bindgen]
pub fn ik(req: &str) -> String {
    ik_json(req)
}

#[wasm_bindgen]
pub fn plan_arm(req: &str) -> String {
    plan_arm_json(req)
}

#[wasm_bindgen]
pub fn arm_config_valid(req: &str) -> String {
    arm_config_valid_json(req)
}

#[cfg(test)]
mod tests;
