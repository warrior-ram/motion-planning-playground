use crate::events::{Event, MAX_EVENTS};
use crate::geometry::{path_length, Obstacle, Point, World};
use crate::maps::{benchmark_maps, DESIGN_GRID, DESIGN_RADIUS};
use crate::{parse_request, plan_json, run, Algorithm, PlanRequest};

fn req(algorithm: &str, obstacles: serde_json::Value, extra: serde_json::Value) -> PlanRequest {
    let mut v = serde_json::json!({
        "algorithm": algorithm,
        "width": 1000, "height": 640,
        "obstacles": obstacles,
        "start": {"x": 80, "y": 320}, "goal": {"x": 920, "y": 320},
        "robotRadius": 12,
        "seed": 1
    });
    if let (Some(o), Some(e)) = (v.as_object_mut(), extra.as_object()) {
        for (k, val) in e {
            o.insert(k.clone(), val.clone());
        }
    }
    parse_request(&v.to_string()).expect("valid request")
}

fn assert_path_valid(world: &World, path: &[Point], start: Point, goal: Point) {
    assert!(path.len() >= 2);
    assert_eq!(path[0], start);
    assert_eq!(*path.last().unwrap(), goal);
    for w in path.windows(2) {
        assert!(world.segment_free(w[0], w[1]), "segment {:?} -> {:?} collides", w[0], w[1]);
    }
}

#[test]
fn collision_point_rect_and_circle() {
    let w = World::new(
        1000.0,
        640.0,
        10.0,
        vec![Obstacle::Rect { x: 100.0, y: 100.0, w: 50.0, h: 50.0 }, Obstacle::Circle { x: 500.0, y: 300.0, r: 40.0 }],
    );
    assert!(!w.point_free(Point::new(120.0, 120.0)), "inside rect");
    assert!(!w.point_free(Point::new(95.0, 120.0)), "within radius of rect edge");
    assert!(w.point_free(Point::new(89.0, 120.0)), "just clear of rect");
    assert!(!w.point_free(Point::new(155.0, 155.0)), "near rect corner (dist ~7.07)");
    assert!(w.point_free(Point::new(158.0, 158.0)), "clear of rect corner (dist ~11.3)");
    assert!(!w.point_free(Point::new(545.0, 300.0)), "within radius of circle");
    assert!(w.point_free(Point::new(551.0, 300.0)), "clear of circle");
    // world bounds
    assert!(!w.point_free(Point::new(5.0, 300.0)));
    assert!(!w.point_free(Point::new(500.0, 635.0)));
    assert!(w.point_free(Point::new(10.0, 10.0)));
}

#[test]
fn collision_segment_checks_whole_edge() {
    let w = World::new(1000.0, 640.0, 5.0, vec![Obstacle::Rect { x: 495.0, y: 0.0, w: 10.0, h: 400.0 }]);
    let a = Point::new(100.0, 200.0);
    let b = Point::new(900.0, 200.0);
    assert!(w.point_free(a) && w.point_free(b));
    assert!(!w.segment_free(a, b), "segment crossing a thin wall must collide");
    // passes below the wall with clearance
    assert!(w.segment_free(Point::new(100.0, 420.0), Point::new(900.0, 420.0)));
    // grazes the bottom of the wall within the robot radius
    assert!(!w.segment_free(Point::new(100.0, 403.0), Point::new(900.0, 403.0)));
    // diagonal past a circle
    let wc = World::new(1000.0, 640.0, 5.0, vec![Obstacle::Circle { x: 500.0, y: 320.0, r: 50.0 }]);
    assert!(!wc.segment_free(Point::new(400.0, 220.0), Point::new(600.0, 420.0)));
    assert!(wc.segment_free(Point::new(400.0, 100.0), Point::new(600.0, 150.0)));
}

#[test]
fn astar_empty_map_near_straight_line() {
    let r = req("astar", serde_json::json!([]), serde_json::json!({}));
    let out = run(&r, true);
    assert!(out.found);
    let straight = r.start.dist(r.goal);
    assert!(out.path_length >= straight - 1e-6);
    assert!(out.path_length <= straight * 1.02, "A* length {} vs straight {}", out.path_length, straight);
    assert_path_valid(&r.world, &out.path, r.start, r.goal);
    assert!(out.nodes_explored > 0);
    // first expand event is the start cell with itself as parent
    match &out.events.iter().find(|e| matches!(e, Event::Expand { .. })).unwrap() {
        Event::Expand { x, y, px, py } => {
            assert_eq!((x, y), (px, py));
        }
        _ => unreachable!(),
    }
}

#[test]
fn astar_diagonal_is_octile_optimal() {
    let r = req(
        "astar",
        serde_json::json!([]),
        serde_json::json!({"start": {"x": 105, "y": 105}, "goal": {"x": 505, "y": 405}}),
    );
    let out = run(&r, false);
    assert!(out.found);
    let straight = r.start.dist(r.goal);
    // grid path is octile-optimal: bounded by octile distance (~1.082x straight for 4:3)
    assert!(out.path_length <= straight * 1.09, "{}", out.path_length);
}

fn simple_map() -> serde_json::Value {
    serde_json::json!([
        {"kind": "rect", "x": 480, "y": 100, "w": 40, "h": 440},
        {"kind": "circle", "x": 250, "y": 200, "r": 60},
        {"kind": "circle", "x": 750, "y": 450, "r": 60}
    ])
}

#[test]
fn all_planners_solve_simple_map() {
    for alg in ["astar", "rrt", "rrtstar"] {
        let r = req(alg, simple_map(), serde_json::json!({}));
        let out = run(&r, true);
        assert!(out.found, "{alg} failed");
        assert_path_valid(&r.world, &out.path, r.start, r.goal);
        assert!((path_length(&out.path) - out.path_length).abs() < 1e-9);
        assert!(!out.events.is_empty());
    }
}

#[test]
fn rrtstar_emits_rewire_events() {
    let r = req("rrtstar", simple_map(), serde_json::json!({}));
    let out = run(&r, true);
    assert!(out.events.iter().any(|e| matches!(e, Event::Rewire { .. })));
    let rr = req("rrt", simple_map(), serde_json::json!({}));
    let out = run(&rr, true);
    assert!(!out.events.iter().any(|e| matches!(e, Event::Rewire { .. })));
}

#[test]
fn rrtstar_shorter_than_rrt_on_average() {
    let mut sum_rrt = 0.0;
    let mut sum_star = 0.0;
    let seeds = 10;
    for seed in 1..=seeds {
        let extra = serde_json::json!({"seed": seed});
        let a = run(&req("rrt", simple_map(), extra.clone()), false);
        let b = run(&req("rrtstar", simple_map(), extra), false);
        assert!(a.found && b.found, "seed {seed}");
        sum_rrt += a.path_length;
        sum_star += b.path_length;
    }
    assert!(sum_star <= sum_rrt, "RRT* avg {} > RRT avg {}", sum_star / seeds as f64, sum_rrt / seeds as f64);
}

#[test]
fn deterministic_with_same_seed() {
    for alg in ["rrt", "rrtstar", "astar"] {
        let a = plan_json(&serde_json::json!({
            "algorithm": alg, "width": 1000, "height": 640, "obstacles": simple_map(),
            "start": {"x": 80, "y": 320}, "goal": {"x": 920, "y": 320}, "robotRadius": 12, "seed": 42
        }).to_string());
        let b = plan_json(&serde_json::json!({
            "algorithm": alg, "width": 1000, "height": 640, "obstacles": simple_map(),
            "start": {"x": 80, "y": 320}, "goal": {"x": 920, "y": 320}, "robotRadius": 12, "seed": 42
        }).to_string());
        assert_eq!(a, b, "{alg} not deterministic");
    }
    // different seeds give different RRT trees
    let x = run(&req("rrt", simple_map(), serde_json::json!({"seed": 1})), true);
    let y = run(&req("rrt", simple_map(), serde_json::json!({"seed": 2})), true);
    assert_ne!(x.events, y.events);
    // benchmark maps are deterministic too
    assert_eq!(crate::benchmark_maps_json(), crate::benchmark_maps_json());
}

#[test]
fn events_capped() {
    // Large iteration budget, no goal reachable (goal walled in) -> many tree nodes.
    let obs = serde_json::json!([
        {"kind": "rect", "x": 860, "y": 260, "w": 120, "h": 10},
        {"kind": "rect", "x": 860, "y": 370, "w": 120, "h": 10},
        {"kind": "rect", "x": 860, "y": 260, "w": 10, "h": 120},
        {"kind": "rect", "x": 970, "y": 260, "w": 10, "h": 120}
    ]);
    let r = req("rrtstar", obs.clone(), serde_json::json!({"params": {"maxIterations": 40000}}));
    let out = run(&r, true);
    assert!(!out.found);
    assert!(out.path.is_empty() && out.path_length == 0.0);
    assert_eq!(out.events.len(), MAX_EVENTS);
    assert!(out.nodes_explored > MAX_EVENTS / 2);
    // A* with a tiny grid on the same unreachable map
    let r = req("astar", obs, serde_json::json!({"params": {"gridSize": 3}}));
    let out = run(&r, true);
    assert!(!out.found);
    assert!(out.events.len() <= MAX_EVENTS);
    assert_eq!(out.events.len(), MAX_EVENTS);
}

#[test]
fn bad_input_returns_error() {
    let cases = [
        "not json",
        "{}",
        r#"{"algorithm":"dijkstra","start":{"x":1,"y":1},"goal":{"x":2,"y":2}}"#,
        r#"{"algorithm":"astar","start":{"x":1,"y":1},"goal":{"x":2,"y":2},"params":{"gridSize":0}}"#,
        r#"{"algorithm":"astar","start":{"x":1,"y":1},"goal":{"x":2,"y":2},"params":{"gridSize":0.001}}"#,
        r#"{"algorithm":"rrt","start":{"x":1,"y":1},"goal":{"x":2,"y":2},"params":{"goalBias":2}}"#,
        r#"{"algorithm":"rrt","start":{"x":1,"y":1},"goal":{"x":2,"y":2},"width":-5}"#,
        r#"{"algorithm":"rrt","start":{"x":1,"y":1},"goal":{"x":2,"y":2},"obstacles":[{"kind":"hexagon","x":1,"y":1}]}"#,
        r#"{"algorithm":"rrt","start":{"x":1,"y":1},"goal":{"x":2,"y":2},"obstacles":[{"kind":"circle","x":1,"y":1,"r":-3}]}"#,
    ];
    for c in cases {
        let v: serde_json::Value = serde_json::from_str(&plan_json(c)).unwrap();
        assert!(v.get("error").and_then(|e| e.as_str()).is_some(), "no error for {c}");
    }
}

#[test]
fn colliding_start_reports_not_found() {
    for alg in ["astar", "rrt", "rrtstar"] {
        let r = req(alg, serde_json::json!([{"kind":"circle","x":80,"y":320,"r":30}]), serde_json::json!({}));
        let out = run(&r, true);
        assert!(!out.found && out.path.is_empty() && out.path_length == 0.0);
    }
}

#[test]
fn response_json_shape() {
    let s = plan_json(
        &serde_json::json!({
            "algorithm": "rrtstar", "obstacles": simple_map(),
            "start": {"x": 80, "y": 320}, "goal": {"x": 920, "y": 320}
        })
        .to_string(),
    );
    let v: serde_json::Value = serde_json::from_str(&s).unwrap();
    for k in ["found", "path", "pathLength", "nodesExplored", "iterations", "events"] {
        assert!(v.get(k).is_some(), "missing {k}");
    }
    let ev = v["events"].as_array().unwrap();
    assert_eq!(ev[0]["k"], "expand");
    assert!(ev[0].get("px").is_some());
    let rw = ev.iter().find(|e| e["k"] == "rewire").unwrap();
    for k in ["x", "y", "px", "py", "opx", "opy"] {
        assert!(rw.get(k).is_some());
    }
    let fr = plan_json(
        &serde_json::json!({"algorithm": "astar", "start": {"x": 80, "y": 320}, "goal": {"x": 920, "y": 320}}).to_string(),
    );
    let v: serde_json::Value = serde_json::from_str(&fr).unwrap();
    assert!(v["events"].as_array().unwrap().iter().any(|e| e["k"] == "frontier"));
}

#[test]
fn benchmark_maps_valid_and_solvable() {
    let maps = benchmark_maps();
    assert_eq!(maps.len(), 20);
    let mut ids = std::collections::HashSet::new();
    for m in &maps {
        assert!(ids.insert(m.id.clone()), "duplicate id {}", m.id);
        let w = m.world(DESIGN_RADIUS);
        assert!(w.point_free(m.start), "{} start collides", m.id);
        assert!(w.point_free(m.goal), "{} goal collides", m.id);
        let r = PlanRequest {
            algorithm: Algorithm::AStar,
            world: w.clone(),
            start: m.start,
            goal: m.goal,
            seed: 1,
            grid_size: DESIGN_GRID,
            rrt: Default::default(),
        };
        let out = run(&r, false);
        assert!(out.found, "{} ({}) not solvable by A*", m.id, m.name);
        assert_path_valid(&w, &out.path, m.start, m.goal);
    }
    let json: serde_json::Value = serde_json::from_str(&crate::benchmark_maps_json()).unwrap();
    let first = &json[0];
    for k in ["id", "name", "width", "height", "start", "goal", "obstacles"] {
        assert!(first.get(k).is_some(), "missing {k}");
    }
    assert_eq!(first["id"], "m01");
    // clutter maps really are cluttered
    assert!(maps[14..].iter().all(|m| !m.obstacles.is_empty()));
}
