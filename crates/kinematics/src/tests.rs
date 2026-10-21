use super::*;
use crate::geom::{angle_diff, capsule_hits, wrap_angle};
use crate::plan::{max_abs_diff, PATH_DENSIFY_STEP};
use std::f64::consts::PI;
use std::time::Instant;

fn v(s: &str) -> Value {
    serde_json::from_str(s).expect("valid json output")
}

fn close(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol
}

// ---------- FK ----------

#[test]
fn fk_known_angles() {
    let arm = Arm::new(V2::new(500.0, 600.0), vec![100.0, 50.0]);
    let j = arm.fk(&[0.0, 0.0]);
    assert_eq!(j.len(), 3);
    assert!(close(j[1].x, 600.0, 1e-9) && close(j[1].y, 600.0, 1e-9));
    assert!(close(j[2].x, 650.0, 1e-9) && close(j[2].y, 600.0, 1e-9));

    // +pi/2 rotates clockwise on screen => +y (down).
    let j = arm.fk(&[PI / 2.0, 0.0]);
    assert!(close(j[2].x, 500.0, 1e-9) && close(j[2].y, 750.0, 1e-9));

    // Relative angles: -pi/2 then +pi/2 => first link up, second link along +x.
    let j = arm.fk(&[-PI / 2.0, PI / 2.0]);
    assert!(close(j[1].x, 500.0, 1e-9) && close(j[1].y, 500.0, 1e-9));
    assert!(close(j[2].x, 550.0, 1e-9) && close(j[2].y, 500.0, 1e-9));
}

#[test]
fn fk_json_contract_example() {
    let out = v(&fk_json(r#"{"base":{"x":500,"y":600},"links":[180,140,90],"angles":[-1.2,0.5,0.3]}"#));
    let joints = out["joints"].as_array().unwrap();
    assert_eq!(joints.len(), 4);
    assert_eq!(joints[0]["x"], 500.0);
    let (mut x, mut y, mut phi) = (500.0f64, 600.0f64, 0.0f64);
    for (l, a) in [(180.0, -1.2), (140.0, 0.5), (90.0, 0.3)] {
        phi += a;
        x += l * f64::cos(phi);
        y += l * f64::sin(phi);
    }
    assert!(close(joints[3]["x"].as_f64().unwrap(), x, 1e-9));
    assert!(close(joints[3]["y"].as_f64().unwrap(), y, 1e-9));
}

// ---------- IK ----------

fn ik_reaches(links: Vec<f64>, seed: Vec<f64>, targets: &[(f64, f64)]) {
    let arm = Arm::new(V2::new(500.0, 600.0), links);
    let opts = IkOptions::default();
    for &(tx, ty) in targets {
        let r = solve_ik(&arm, &seed, V2::new(tx, ty), &opts);
        let e = arm.end_effector(&r.angles);
        let d = ((e.x - tx).powi(2) + (e.y - ty).powi(2)).sqrt();
        assert!(r.reached, "target ({tx},{ty}) not reached, error {}", r.error);
        assert!(d <= 0.5 + 1e-9, "distance {d}");
        assert!(close(d, r.error, 1e-6));
    }
}

#[test]
fn ik_two_link_reaches() {
    ik_reaches(
        vec![180.0, 140.0],
        vec![-1.2, 0.5],
        &[(650.0, 450.0), (400.0, 400.0), (700.0, 600.0), (500.0, 350.0), (560.0, 560.0)],
    );
}

#[test]
fn ik_three_link_reaches() {
    ik_reaches(
        vec![180.0, 140.0, 90.0],
        vec![-1.2, 0.5, 0.3],
        &[(650.0, 400.0), (350.0, 380.0), (800.0, 600.0), (500.0, 250.0), (540.0, 580.0)],
    );
}

#[test]
fn ik_reaches_from_straight_seed() {
    // Fully straight seed is a Jacobian singularity for targets along the arm.
    ik_reaches(vec![180.0, 140.0], vec![0.0, 0.0], &[(600.0, 600.0)]);
    ik_reaches(vec![180.0, 140.0, 90.0], vec![0.0, 0.0, 0.0], &[(650.0, 600.0)]);
}

#[test]
fn ik_unreachable_points_at_target() {
    for links in [vec![180.0, 140.0], vec![180.0, 140.0, 90.0]] {
        let arm = Arm::new(V2::new(500.0, 600.0), links.clone());
        let seed = vec![-1.0; links.len()];
        let target = V2::new(100.0, 100.0);
        let r = solve_ik(&arm, &seed, target, &IkOptions::default());
        assert!(!r.reached);
        let reach: f64 = links.iter().sum();
        let dist_t = ((target.x - 500.0).powi(2) + (target.y - 600.0).powi(2)).sqrt();
        assert!(close(r.error, dist_t - reach, 0.5), "error {} vs {}", r.error, dist_t - reach);
        // End effector direction from base matches target direction.
        let e = arm.end_effector(&r.angles);
        let a_e = (e.y - 600.0).atan2(e.x - 500.0);
        let a_t = (target.y - 600.0).atan2(target.x - 500.0);
        assert!(angle_diff(a_e, a_t).abs() < 1e-2);
        // Arm is (nearly) fully stretched.
        let de = ((e.x - 500.0).powi(2) + (e.y - 600.0).powi(2)).sqrt();
        assert!(de > reach - 0.5);
    }
}

#[test]
fn ik_respects_limits() {
    let arm = Arm::new(V2::new(500.0, 600.0), vec![180.0, 140.0]);
    let opts = IkOptions { limits: Some(vec![(-PI, PI), (0.0, 0.3)]), ..Default::default() };
    let r = solve_ik(&arm, &[-1.0, 0.1], V2::new(500.0, 350.0), &opts);
    assert!(r.angles[1] >= 0.0 && r.angles[1] <= 0.3);
}

#[test]
fn ik_dragging_keeps_elbow_sense() {
    // Small steps along a path: elbow sign must never flip and joints move smoothly.
    let arm = Arm::new(V2::new(500.0, 600.0), vec![180.0, 140.0, 90.0]);
    let mut q = vec![-1.2, 0.5, 0.3];
    let opts = IkOptions::default();
    let e0 = arm.end_effector(&q);
    for i in 0..200 {
        // Pointer drag: ~3 world units per move, starting at the current end effector.
        let t = i as f64 / 199.0;
        // Sweep right-to-left across the workspace (stays well inside the 410 reach).
        let target = V2::new(e0.x - 400.0 * t, e0.y + 60.0 * (t * 6.0).sin());
        let r = solve_ik(&arm, &q, target, &opts);
        assert!(r.reached, "step {i}: target {target:?} not reached, error {}", r.error);
        assert!(r.angles[1] > 0.0, "elbow flipped at step {i}");
        assert!(max_abs_diff(&q, &r.angles) < 0.5, "jump at step {i}");
        q = r.angles;
    }
}

#[test]
fn ik_is_sub_millisecond() {
    let arm = Arm::new(V2::new(500.0, 600.0), vec![180.0, 140.0, 90.0]);
    let opts = IkOptions::default();
    let mut q = vec![-1.2, 0.5, 0.3];
    let n = 2000;
    let start = Instant::now();
    for i in 0..n {
        let a = i as f64 * 0.01;
        let target = V2::new(500.0 + 250.0 * a.cos(), 600.0 - 200.0 - 100.0 * a.sin());
        q = solve_ik(&arm, &q, target, &opts).angles;
    }
    let per_call = start.elapsed().as_secs_f64() * 1000.0 / n as f64;
    // Also measure through the JSON layer (what wasm actually calls).
    let req = r#"{"base":{"x":500,"y":600},"links":[180,140,90],"angles":[0,0,0],"target":{"x":200,"y":420},"maxIterations":100,"tolerance":0.5}"#;
    let s2 = Instant::now();
    for _ in 0..n {
        std::hint::black_box(ik_json(req));
    }
    let per_json = s2.elapsed().as_secs_f64() * 1000.0 / n as f64;
    println!("IK: {per_call:.5} ms/call (core), {per_json:.5} ms/call (JSON, cold seed)");
    assert!(per_call < 1.0 && per_json < 1.0);
}

#[test]
fn ik_json_shape_and_errors() {
    let out = v(&ik_json(
        r#"{"base":{"x":500,"y":600},"links":[180,140],"angles":[-1,1],"target":{"x":600,"y":450}}"#,
    ));
    for k in ["angles", "joints", "reached", "error", "iterations"] {
        assert!(out.get(k).is_some(), "missing {k}");
    }
    assert_eq!(out["joints"].as_array().unwrap().len(), 3);
    assert!(v(&ik_json("not json"))["error"].is_string());
    assert!(v(&ik_json(r#"{"base":{"x":0,"y":0},"links":[1],"angles":[0],"target":{"x":1,"y":1}}"#))["error"].is_string());
    assert!(v(&ik_json(r#"{"base":{"x":0,"y":0},"links":[1,1],"angles":[0],"target":{"x":1,"y":1}}"#))["error"].is_string());
}

// ---------- geometry ----------

#[test]
fn angle_wrapping() {
    assert!(close(wrap_angle(PI), PI, 1e-12));
    assert!(close(wrap_angle(-PI), PI, 1e-12));
    assert!(close(wrap_angle(3.0 * PI / 2.0), -PI / 2.0, 1e-12));
    assert!(close(wrap_angle(-3.0 * PI / 2.0), PI / 2.0, 1e-12));
    assert!(close(wrap_angle(7.0 * PI), PI, 1e-9));
    assert!(close(wrap_angle(0.3), 0.3, 1e-12));
    for i in -100..100 {
        let w = wrap_angle(i as f64 * 0.37);
        assert!(w > -PI && w <= PI);
    }
    // Shortest difference crosses the seam.
    assert!(close(angle_diff(3.0, -3.0), 2.0 * PI - 6.0, 1e-12));
    assert!(close(angle_diff(-3.0, 3.0), -(2.0 * PI - 6.0), 1e-12));
}

#[test]
fn capsule_rect_collision() {
    let r = Shape::rect(100.0, 100.0, 50.0, 50.0);
    // Segment passes through.
    assert!(capsule_hits(V2::new(50.0, 125.0), V2::new(200.0, 125.0), 1.0, &r));
    // Segment fully inside.
    assert!(capsule_hits(V2::new(110.0, 110.0), V2::new(120.0, 120.0), 0.0, &r));
    // Near miss beyond radius.
    assert!(!capsule_hits(V2::new(50.0, 90.0), V2::new(200.0, 90.0), 4.0, &r));
    // Within radius of the edge.
    assert!(capsule_hits(V2::new(50.0, 90.0), V2::new(200.0, 90.0), 10.5, &r));
    // Corner proximity: endpoint near the corner, diagonal distance ~ 7.07.
    assert!(!capsule_hits(V2::new(95.0, 95.0), V2::new(80.0, 80.0), 7.0, &r));
    assert!(capsule_hits(V2::new(95.0, 95.0), V2::new(80.0, 80.0), 7.2, &r));
    // Negative width/height normalised.
    let rn = Shape::rect(150.0, 150.0, -50.0, -50.0);
    assert!(capsule_hits(V2::new(50.0, 125.0), V2::new(200.0, 125.0), 1.0, &rn));
}

#[test]
fn capsule_circle_collision() {
    let c = Shape::circle(100.0, 100.0, 20.0);
    assert!(capsule_hits(V2::new(0.0, 100.0), V2::new(200.0, 100.0), 0.0, &c));
    assert!(!capsule_hits(V2::new(0.0, 125.0), V2::new(200.0, 125.0), 4.0, &c));
    assert!(capsule_hits(V2::new(0.0, 125.0), V2::new(200.0, 125.0), 5.5, &c));
    // Closest point is the segment endpoint.
    assert!(!capsule_hits(V2::new(130.0, 100.0), V2::new(200.0, 100.0), 9.0, &c));
    assert!(capsule_hits(V2::new(130.0, 100.0), V2::new(200.0, 100.0), 10.5, &c));
}

#[test]
fn arm_config_valid_json_works() {
    let base = r#""base":{"x":500,"y":600},"links":[180,140],"linkThickness":8"#;
    let hit = format!(r#"{{{base},"obstacles":[{{"kind":"rect","x":600,"y":580,"w":40,"h":40}}],"angles":[0,0]}}"#);
    let miss = format!(r#"{{{base},"obstacles":[{{"kind":"circle","x":600,"y":400,"r":30}}],"angles":[0,0]}}"#);
    assert_eq!(v(&arm_config_valid_json(&hit))["valid"], false);
    assert_eq!(v(&arm_config_valid_json(&miss))["valid"], true);
    assert!(v(&arm_config_valid_json("{}"))["error"].is_string());
}

// ---------- planning ----------

/// Arm points right; goal points up-left. A pin straight above the base blocks the short
/// (counter-clockwise on screen) sweep, so the planner must swing the arm down and around.
fn plan_scene(algo: &str, seed: u64) -> (Arm, Vec<Shape>, Value) {
    let req = format!(
        r#"{{"base":{{"x":500,"y":400}},"links":[150,110],"linkThickness":8,
        "obstacles":[{{"kind":"circle","x":500,"y":280,"r":25}},{{"kind":"rect","x":640,"y":470,"w":60,"h":50}}],
        "start":[0.0,0.3],"goal":[-2.6,-0.4],"algorithm":"{algo}","seed":{seed},
        "params":{{"maxIterations":4000}}}}"#
    );
    let out = v(&plan_arm_json(&req));
    let arm = Arm::new(V2::new(500.0, 400.0), vec![150.0, 110.0]);
    let shapes = vec![Shape::circle(500.0, 280.0, 25.0), Shape::rect(640.0, 470.0, 60.0, 50.0)];
    (arm, shapes, out)
}

fn check_path(arm: &Arm, shapes: &[Shape], out: &Value, start: &[f64], goal: &[f64]) {
    assert!(out.get("error").is_none(), "{out}");
    assert_eq!(out["found"], true, "{}", out);
    let path: Vec<Vec<f64>> = serde_json::from_value(out["path"].clone()).unwrap();
    assert!(path.len() >= 2);
    assert!(max_abs_diff(&path[0], start) < 1e-9);
    assert!(max_abs_diff(path.last().unwrap(), goal) < 1e-9);
    for q in &path {
        assert!(q.iter().all(|a| *a > -PI - 1e-12 && *a <= PI + 1e-12));
        assert!(arm.config_valid(q, shapes, 4.0), "invalid config on path: {q:?}");
    }
    for w in path.windows(2) {
        assert!(max_abs_diff(&w[0], &w[1]) <= PATH_DENSIFY_STEP + 1e-9);
    }
}

#[test]
fn plan_arm_rrt_finds_valid_path_around_obstacle() {
    let (arm, shapes, out) = plan_scene("rrt", 1);
    // The straight joint-space interpolation must actually collide (so planning is needed).
    let world = World { arm: &arm, obstacles: &shapes, radius: 4.0 };
    assert!(!world.edge_valid(&[0.0, 0.3], &[-2.6, -0.4]));
    check_path(&arm, &shapes, &out, &[0.0, 0.3], &[-2.6, -0.4]);
    assert!(out["events"].as_array().unwrap().iter().all(|e| e["k"] == "expand"));
}

#[test]
fn plan_arm_rrtstar_finds_valid_path_and_rewires() {
    let (arm, shapes, out) = plan_scene("rrtstar", 3);
    check_path(&arm, &shapes, &out, &[0.0, 0.3], &[-2.6, -0.4]);
    let evs = out["events"].as_array().unwrap();
    assert!(evs.iter().any(|e| e["k"] == "rewire"));
    let rw = evs.iter().find(|e| e["k"] == "rewire").unwrap();
    assert!(rw["q"].is_array() && rw["pq"].is_array() && rw["opq"].is_array());
    assert!(evs.len() <= 20_000);
}

#[test]
fn plan_arm_three_links() {
    let req = r#"{"base":{"x":500,"y":600},"links":[180,140,90],"linkThickness":8,
        "obstacles":[{"kind":"rect","x":420,"y":260,"w":160,"h":40}],
        "start":[-2.2,0.6,0.4],"goal":[-0.9,-0.5,-0.4],"algorithm":"rrt","seed":7}"#;
    let out = v(&plan_arm_json(req));
    let arm = Arm::new(V2::new(500.0, 600.0), vec![180.0, 140.0, 90.0]);
    let shapes = vec![Shape::rect(420.0, 260.0, 160.0, 40.0)];
    check_path(&arm, &shapes, &out, &[-2.2, 0.6, 0.4], &[-0.9, -0.5, -0.4]);
}

#[test]
fn plan_arm_deterministic() {
    for algo in ["rrt", "rrtstar"] {
        let (_, _, a) = plan_scene(algo, 42);
        let (_, _, b) = plan_scene(algo, 42);
        assert_eq!(a.to_string(), b.to_string());
    }
    let (_, _, a) = plan_scene("rrt", 1);
    let (_, _, b) = plan_scene("rrt", 2);
    assert_ne!(a["events"].to_string(), b["events"].to_string());
}

#[test]
fn plan_arm_wraps_across_seam() {
    // Start near +pi, goal near -pi: shortest route crosses the seam (0.2 rad apart).
    let req = r#"{"base":{"x":500,"y":300},"links":[100,80],"start":[3.04,0.2],"goal":[-3.04,0.2],"algorithm":"rrt","seed":1}"#;
    let out = v(&plan_arm_json(req));
    assert_eq!(out["found"], true);
    let path: Vec<Vec<f64>> = serde_json::from_value(out["path"].clone()).unwrap();
    assert!(path.len() <= 6, "path took the long way: {} configs", path.len());
    for w in path.windows(2) {
        assert!(max_abs_diff(&w[0], &w[1]) <= PATH_DENSIFY_STEP + 1e-9);
    }
}

#[test]
fn plan_arm_start_in_collision_and_bad_input() {
    let req = r#"{"base":{"x":500,"y":400},"links":[150,110],"linkThickness":8,
        "obstacles":[{"kind":"circle","x":600,"y":400,"r":20}],
        "start":[0,0],"goal":[2,0],"algorithm":"rrt","seed":1}"#;
    let out = v(&plan_arm_json(req));
    assert_eq!(out["found"], false);
    assert_eq!(out["path"].as_array().unwrap().len(), 0);
    assert!(v(&plan_arm_json(r#"{"base":{"x":0,"y":0},"links":[1,1],"start":[0,0],"goal":[1,1],"algorithm":"prm"}"#))["error"].is_string());
    assert!(v(&plan_arm_json(r#"{"base":{"x":0,"y":0},"links":[1,1],"start":[0,0],"goal":[1,1],"obstacles":[{"kind":"blob"}]}"#))["error"].is_string());
    assert!(v(&plan_arm_json(""))["error"].is_string());
}

#[test]
fn plan_arm_event_cap() {
    // Two pins above and below the base block link 1 from ever crossing +-pi/2, so start
    // (pointing right) and goal (pointing left) are in disconnected free-space components.
    // RRT* (expand + rewire events) over many iterations overflows the 20000-event cap.
    let req = r#"{"base":{"x":500,"y":400},"links":[100,80],"linkThickness":4,
        "obstacles":[{"kind":"circle","x":500,"y":350,"r":5},{"kind":"circle","x":500,"y":450,"r":5}],
        "start":[0,0],"goal":[3.0,0],"algorithm":"rrtstar","seed":1,
        "params":{"maxIterations":30000,"rewireRadius":0.2}}"#;
    let out = v(&plan_arm_json(req));
    assert_eq!(out["found"], false);
    let evs = out["events"].as_array().unwrap();
    assert_eq!(evs.len(), 20_000);
    // Earliest events are kept: the first is the start node.
    assert_eq!(evs[0]["k"], "expand");
    assert_eq!(evs[0]["q"], serde_json::json!([0.0, 0.0]));
}

#[test]
fn plan_arm_default_params_timing() {
    let (_, _, _) = plan_scene("rrt", 1);
    let t = Instant::now();
    let (_, _, out) = plan_scene("rrtstar", 1);
    println!(
        "plan_arm rrtstar 4000 iters: {:.1} ms, {} nodes, {} events",
        t.elapsed().as_secs_f64() * 1000.0,
        out["nodesExplored"],
        out["events"].as_array().unwrap().len()
    );
    assert_eq!(out["found"], true);
}
