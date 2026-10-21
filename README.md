# Motion planning playground

An interactive page for comparing robot motion planners. Draw walls, drag the start and the goal, and race A*, RRT and RRT* on the same map while their search trees grow on the canvas. A second tab plans collision-free motion for a 2 or 3 joint arm, which you can drag by its end effector.

All of the planning code is Rust, compiled to WebAssembly, and runs in the browser. There is no server.

## What it does

- **Mobile robot:** plans a path for a disc-shaped robot with A*, RRT or RRT*, or races all three. Planning time, nodes explored and path length appear side by side for each planner.
- **Robot arm:** forward and inverse kinematics for a planar 2 or 3 joint arm mounted at the centre of the canvas, so it can reach in every direction. Drag the end effector and the joints follow. RRT or RRT* plans in joint space to move the arm between two poses without touching obstacles. On the default scene both planners succeed on 20 of 20 seeds, for both 2 and 3 joints.
- **Benchmark:** every planner over 20 fixed maps, shown on the page and reproducible from the command line.

## The algorithms

**A\*** searches a grid of 10 unit cells with 8-connected moves and an octile-distance heuristic. Cells are blocked when the robot's disc would touch an obstacle. A\* always finds the shortest path the grid allows, if one exists, and gives the same answer every run.

**RRT** (rapidly-exploring random tree) grows a tree from the start. Each step samples a random point, finds the nearest tree node, and extends the tree 25 units toward the sample if the move is collision-free. It stops as soon as the tree reaches the goal. It is fast and needs no grid, but its paths are jagged and longer than necessary.

**RRT\*** grows the tree the same way, with two additions. A new node connects to whichever nearby node gives it the cheapest path from the start, and nearby nodes are rewired through the new node when that shortens their paths. It keeps improving the path for the full iteration budget, so it is the slowest of the three, but its paths converge toward the shortest.

**Arm planning** runs RRT and RRT\* in joint space, where each point is a set of joint angles. Each link is a capsule, and every edge is checked for collisions at most 0.02 rad apart on each joint.

**Inverse kinematics** uses damped least squares on the arm's Jacobian, so it stays stable near singular poses and returns the closest reachable pose when the target is out of reach.

## Benchmark

20 fixed maps, robot radius 12. A\* runs once per map because it is deterministic. RRT and RRT\* each run with 10 seeds per map. Native release build. Times in the browser are slower.

| Planner | Success | Mean time | Mean nodes explored | Path length vs A\* |
|---|---|---:|---:|---:|
| A\* | 20 of 20 | 0.55 ms | 1,399 | 1.000 |
| RRT | 200 of 200 | 0.80 ms | 618 | 1.225 |
| RRT\* | 200 of 200 | 72.40 ms | 14,063 | 0.939 |

RRT\* finds paths 6% shorter than A\*, because A\* is confined to grid moves and RRT\* is not. RRT finds paths 22% longer. The per-map results are in [bench/results.md](bench/results.md).

### Performance

The tree planners answer every nearest-node and nearby-node query from a uniform bucket grid instead of scanning the whole tree. Collision checks skip any obstacle whose bounding box is out of reach. Neither change alters the results: all 420 benchmark runs give identical paths and node counts before and after, and a test checks the grid queries against a linear scan.

| | Before | After |
|---|---:|---:|
| RRT\*, mean per map (native) | 307.5 ms | 72.4 ms |
| RRT\*, mean per map (browser) | about 440 ms | 98 ms |
| RRT\*, worst map (browser) | about 870 ms | 131 ms |
| Arm RRT\*, 3 joints, 10,000 iterations (native) | 1,114 ms | 212 ms |

The arm timings were measured on the original arm scene, before the arm moved to the centre of the canvas.

## Run it

You need Rust with the `wasm32-unknown-unknown` target, [wasm-pack](https://rustwasm.github.io/wasm-pack/) and Node.js.

```bash
wasm-pack build crates/planner --target web --release --out-dir ../../web/src/wasm/planner
wasm-pack build crates/kinematics --target web --release --out-dir ../../web/src/wasm/kinematics
cd web
npm install
npm run dev
```

The compiled WebAssembly is committed in `web/src/wasm/`, so the two `wasm-pack` steps are only needed after changing the Rust code.

Run the tests and the benchmark from the repository root:

```bash
cargo test --release
cargo run -p planner --release --bin bench
```

The benchmark writes `bench/results.json` and `bench/results.md`. Copy `bench/results.json` to `web/public/bench-results.json` to update the page.

## Layout

| Path | Contents |
|---|---|
| `crates/planner` | A\*, RRT, RRT\*, collision checks, the 20 benchmark maps and the benchmark binary |
| `crates/kinematics` | Forward and inverse kinematics, and joint-space RRT and RRT\* for the arm |
| `web` | React, TypeScript and Vite frontend that draws on canvas |
| `bench` | Benchmark output |
| `CONTRACT.md` | The JSON interface between the Rust crates and the frontend |
