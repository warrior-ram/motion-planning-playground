//! Native benchmark: all 3 planners over the 20 fixed maps.
//! A* runs once per map (it is deterministic); RRT / RRT* are averaged over 10 seeds.
//! Writes `bench/results.json` and `bench/results.md` at the repo root.

use std::fmt::Write as _;
use std::path::PathBuf;
use std::time::Instant;

use planner::maps::{benchmark_maps, BenchMap, DESIGN_GRID, DESIGN_RADIUS};
use planner::rrt::RrtParams;
use planner::{run, Algorithm, PlanRequest};
use serde::Serialize;

const SEEDS: u64 = 10;
const ALGS: [Algorithm; 3] = [Algorithm::AStar, Algorithm::Rrt, Algorithm::RrtStar];

#[derive(Serialize)]
#[serde(rename_all = "camelCase")]
struct RunRecord {
    seed: u64,
    found: bool,
    time_ms: f64,
    nodes_explored: usize,
    iterations: usize,
    path_length: f64,
}

#[derive(Serialize)]
#[serde(rename_all = "camelCase")]
struct MapResult {
    map_id: String,
    map_name: String,
    algorithm: String,
    runs: usize,
    successes: usize,
    /// Mean over all runs.
    mean_time_ms: f64,
    /// Mean over all runs.
    mean_nodes_explored: f64,
    /// Mean over successful runs only; null when no run succeeded.
    mean_path_length: Option<f64>,
    records: Vec<RunRecord>,
}

#[derive(Serialize)]
#[serde(rename_all = "camelCase")]
struct Summary {
    algorithm: String,
    runs: usize,
    successes: usize,
    success: String,
    mean_time_ms: f64,
    mean_nodes_explored: f64,
    mean_path_length: Option<f64>,
    /// Mean of (path length / A* path length) over successful runs on maps A* solved.
    mean_path_ratio_vs_astar: Option<f64>,
}

#[derive(Serialize)]
#[serde(rename_all = "camelCase")]
struct Report {
    robot_radius: f64,
    grid_size: f64,
    seeds: u64,
    params: serde_json::Value,
    maps: usize,
    results: Vec<MapResult>,
    summary: Vec<Summary>,
}

fn mean(v: impl Iterator<Item = f64>) -> Option<f64> {
    let (s, n) = v.fold((0.0, 0usize), |(s, n), x| (s + x, n + 1));
    if n == 0 {
        None
    } else {
        Some(s / n as f64)
    }
}

fn bench_map(m: &BenchMap, alg: Algorithm) -> MapResult {
    let base = PlanRequest {
        algorithm: alg,
        world: m.world(DESIGN_RADIUS),
        start: m.start,
        goal: m.goal,
        seed: 1,
        grid_size: DESIGN_GRID,
        rrt: RrtParams::default(),
    };
    let seeds: Vec<u64> = if alg == Algorithm::AStar { vec![1] } else { (1..=SEEDS).collect() };
    let records: Vec<RunRecord> = seeds
        .into_iter()
        .map(|seed| {
            let req = PlanRequest { seed, ..base.clone() };
            let t = Instant::now();
            // Record events too, so timings reflect what the browser call does.
            let out = run(&req, true);
            let time_ms = t.elapsed().as_secs_f64() * 1000.0;
            RunRecord {
                seed,
                found: out.found,
                time_ms,
                nodes_explored: out.nodes_explored,
                iterations: out.iterations,
                path_length: out.path_length,
            }
        })
        .collect();
    let successes = records.iter().filter(|r| r.found).count();
    MapResult {
        map_id: m.id.clone(),
        map_name: m.name.clone(),
        algorithm: alg.name().to_string(),
        runs: records.len(),
        successes,
        mean_time_ms: mean(records.iter().map(|r| r.time_ms)).unwrap_or(0.0),
        mean_nodes_explored: mean(records.iter().map(|r| r.nodes_explored as f64)).unwrap_or(0.0),
        mean_path_length: mean(records.iter().filter(|r| r.found).map(|r| r.path_length)),
        records,
    }
}

fn fmt_opt(v: Option<f64>, prec: usize) -> String {
    v.map_or_else(|| "-".to_string(), |x| format!("{x:.prec$}"))
}

fn repo_root() -> PathBuf {
    // crates/planner -> repo root
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("..").join("..")
}

fn main() {
    let maps = benchmark_maps();
    // Warm-up so the first timed run is not penalised by cold caches.
    for m in maps.iter().take(3) {
        for alg in ALGS {
            let _ = bench_map(m, alg);
        }
    }

    let mut results = Vec::new();
    for m in &maps {
        for alg in ALGS {
            let r = bench_map(m, alg);
            eprintln!(
                "{} {:<24} {:<8} {}/{}  {:>8.2} ms",
                r.map_id, r.map_name, r.algorithm, r.successes, r.runs, r.mean_time_ms
            );
            results.push(r);
        }
    }

    let astar_len = |map_id: &str| -> Option<f64> {
        results
            .iter()
            .find(|r| r.map_id == map_id && r.algorithm == "astar")
            .and_then(|r| r.mean_path_length)
    };

    let summary: Vec<Summary> = ALGS
        .iter()
        .map(|alg| {
            let rs: Vec<&MapResult> = results.iter().filter(|r| r.algorithm == alg.name()).collect();
            let recs: Vec<&RunRecord> = rs.iter().flat_map(|r| r.records.iter()).collect();
            let runs = recs.len();
            let successes = recs.iter().filter(|r| r.found).count();
            let ratios = rs.iter().flat_map(|r| {
                let base = astar_len(&r.map_id);
                r.records.iter().filter(|x| x.found).filter_map(move |x| base.map(|b| x.path_length / b))
            });
            Summary {
                algorithm: alg.name().to_string(),
                runs,
                successes,
                success: format!("{successes} of {runs}"),
                mean_time_ms: mean(recs.iter().map(|r| r.time_ms)).unwrap_or(0.0),
                mean_nodes_explored: mean(recs.iter().map(|r| r.nodes_explored as f64)).unwrap_or(0.0),
                mean_path_length: mean(recs.iter().filter(|r| r.found).map(|r| r.path_length)),
                mean_path_ratio_vs_astar: mean(ratios),
            }
        })
        .collect();

    let p = RrtParams::default();
    let report = Report {
        robot_radius: DESIGN_RADIUS,
        grid_size: DESIGN_GRID,
        seeds: SEEDS,
        params: serde_json::json!({
            "gridSize": DESIGN_GRID,
            "maxIterations": p.max_iterations,
            "stepSize": p.step_size,
            "goalBias": p.goal_bias,
            "goalTolerance": p.goal_tolerance,
            "rewireRadius": p.rewire_radius,
        }),
        maps: maps.len(),
        results,
        summary,
    };

    // ---- Markdown ----
    let mut md = String::new();
    let _ = writeln!(md, "# Motion planner benchmark\n");
    let _ = writeln!(
        md,
        "Generated by `cargo run -p planner --release --bin bench`. {} maps, robot radius {}, \
         default params (gridSize {}, maxIterations {}, stepSize {}, goalBias {}, goalTolerance {}, rewireRadius {}). \
         A* is deterministic and runs once per map; RRT and RRT* run with seeds 1..={} per map. \
         Time and nodes are means over all runs; path length is the mean over successful runs only. \
         Native release build (`std::time::Instant`), event logging enabled; browser/WASM timings will differ.\n",
        maps.len(),
        DESIGN_RADIUS,
        DESIGN_GRID,
        p.max_iterations,
        p.step_size,
        p.goal_bias,
        p.goal_tolerance,
        p.rewire_radius,
        SEEDS
    );
    let _ = writeln!(md, "## Summary per algorithm\n");
    let _ = writeln!(md, "| Algorithm | Success | Mean time (ms) | Mean nodes explored | Mean path length | Path length vs A* |");
    let _ = writeln!(md, "|---|---|---:|---:|---:|---:|");
    for s in &report.summary {
        let _ = writeln!(
            md,
            "| {} | {} | {:.2} | {:.0} | {} | {} |",
            s.algorithm,
            s.success,
            s.mean_time_ms,
            s.mean_nodes_explored,
            fmt_opt(s.mean_path_length, 1),
            fmt_opt(s.mean_path_ratio_vs_astar, 3)
        );
    }
    let _ = writeln!(md, "\n## Per map\n");
    let _ = writeln!(md, "| Map | Algorithm | Success | Time (ms) | Nodes explored | Path length |");
    let _ = writeln!(md, "|---|---|---|---:|---:|---:|");
    for r in &report.results {
        let _ = writeln!(
            md,
            "| {} {} | {} | {} of {} | {:.2} | {:.0} | {} |",
            r.map_id,
            r.map_name,
            r.algorithm,
            r.successes,
            r.runs,
            r.mean_time_ms,
            r.mean_nodes_explored,
            fmt_opt(r.mean_path_length, 1)
        );
    }

    let out_dir = repo_root().join("bench");
    std::fs::create_dir_all(&out_dir).expect("create bench dir");
    let json = serde_json::to_string_pretty(&report).expect("serialize report");
    std::fs::write(out_dir.join("results.json"), json + "\n").expect("write results.json");
    std::fs::write(out_dir.join("results.md"), md).expect("write results.md");
    println!("{}", report.summary.iter().map(|s| format!("{}: {} ({:.2} ms)", s.algorithm, s.success, s.mean_time_ms)).collect::<Vec<_>>().join("\n"));
    println!("wrote {}", out_dir.display());
}
