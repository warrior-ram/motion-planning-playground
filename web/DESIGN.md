# Design: Motion planning playground

Code-led build, chosen and recorded without asking the owner (they asked for no questions). The direction contract lives in `.impeccable/surfaces/src-app-tsx.md` (seed bc0843b3); product facts, all inferred from the brief, live in `PRODUCT.md`.

## Direction
Motorsport timing graphics applied to a working tool. The canvas is the circuit, the planners are three teams racing it, and a timing tower on the right re-sorts as results land. The shell itself is a standard operate layout: header, mode tabs, a canvas work area with its tools, a control column, and a map rail.

## Tokens
| Role | Value |
|---|---|
| Ground / panel / panel-2 | `#121417` / `#1A1D22` / `#20242A` |
| Lines | `#2B3038`, strong `#3B424D` |
| Ink / soft / faint | `#ECEEF1` / `#A3ACB7` / `#7D8793` |
| A* livery | `#FF7A45` |
| RRT livery | `#34D1C4` |
| RRT* livery | `#E8E15A` |
| Best in column | `#B072FF` (the purple "fastest sector" convention) |
| Failure / collision | `#FF4D5E` |
| Type | Barlow Condensed 500-700 for names and tabular numerals; Barlow 400-600 for UI and body (both self-hosted via @fontsource) |
| Radii | 8px controls, 10-14px panels |
| Easing | `--ease-out cubic-bezier(.23,1,.32,1)`, `--ease-in-out cubic-bezier(.77,0,.175,1)`, `--ease-drawer cubic-bezier(.32,.72,0,1)` |

Each planner keeps one colour everywhere: tree, path, robot, tower swatch, benchmark bars and the explainer rule. Purple is reserved for the best value in a column and red for failure.

## Motion inventory
- Page load: the brand bars rise, then the header, claim, tabs and work area enter with a staggered fade, lift and blur. The field's survey dots sweep in left to right, obstacles pop in with a short stagger, and preset tiles cascade.
- An opening race starts automatically 0.7 s after load.
- Search replay: the `events` log is replayed with requestAnimationFrame on a per-run offscreen layer. A* cells fill over a visible grid armature that fades in only while A* is on the map. RRT and RRT* edges grow from a pulsing head, and RRT* rewires flash white.
- Path draw: the found path traces itself with ease-in-out behind a white leading dot. The robot fades in at the start, then drives the path at the same speed for every planner, so the shortest path visibly arrives first.
- Timing tower: rows slide in staggered, re-sort with a spring layout animation, and the phase label (Exploring, Path found, Driving, Arrived, No path) animates on each change. A progress bar under each row tracks the replay, numbers count up, and the best values light purple once the race settles.
- Status band: the message swaps with a vertical slide and blur, and the band turns red on failure or collision.
- Start and goal markers follow drags and preset swaps on a spring. Deleted obstacles fade out with a red flash. An invalid placement pulses the canvas border red and raises a toast.
- Arm: the end effector target is spring-smoothed and IK is solved every frame, so the joints follow smoothly. Joint angles ease between poses on a critically damped spring. The arm colour blends to red while `arm_config_valid` is false. Plans replay the joint-space tree projected through FK, trace the end-effector path, then animate the arm along the densified path.
- Controls: tabs, segmented controls and tools use a shared-layout sliding pill. Every pressable scales to 0.97 on press. Hover effects are gated behind `(hover: hover)`. The preset rail lifts the hovered tile and dims the rest.
- Benchmark: summary rows rise in when the section scrolls into view, success bars fill, means count up, the 20-map bar matrix grows in staggered, and the metric switch re-scales the bars.
- Toasts enter from below on the drawer curve and leave faster than they arrive.
- Reduced motion: `MotionConfig reducedMotion="user"`, searches jump straight to their final result, springs are disabled, and fades replace movement.

## Data
Benchmark data is loaded from `public/bench-results.json`, a copy of `bench/results.json`. Re-copy it after re-running `cargo run -p planner --release --bin bench`.
