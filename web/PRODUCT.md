# Product

<!-- impeccable:product-schema 1 -->

All facts below are inferred from the written brief (demos.md "Demo 1" and CONTRACT.md); the owner asked for no interview.

## Platform

web

## Users
(inferred) Recruiters, hiring engineers and robotics peers who open a portfolio link on a laptop or phone, and spend ten seconds to a few minutes poking at it. Secondary: the author, recording a 30 to 45 second screen capture including a failure.

## Product Purpose
(inferred) "Motion planning playground": an interactive page where visitors draw obstacles, set a start and goal, and watch A*, RRT and RRT* find a path for a mobile robot, and drag a 2 or 3 joint arm whose joints follow via inverse kinematics. Success: a visitor sees a search tree grow live and understands the trade-offs between planners within one session.

## Positioning
(inferred) The planners are real Rust compiled to WebAssembly, run in the visitor's browser, and every number shown (time, nodes explored, path length) is measured live on the visitor's own map.

## Operating Context
(inferred) Opened from a LinkedIn post, a README GIF, or a portfolio. Used with mouse or touch. Screen recordings of it are a primary distribution artifact.

## Capabilities and Constraints
- Algorithms: A* (8-connected grid, octile), RRT, RRT*; arm: FK, DLS IK, joint-space RRT/RRT*, collision check.
- 20 fixed benchmark maps from `benchmark_maps()`; benchmark results from `bench/results.json`.
- Static hosting (Cloudflare Pages / GitHub Pages); Vite base `./`.
- Must work on laptop and phone (touch drawing and dragging).

## Evidence on Hand
- Live planner output only. Benchmark numbers come exclusively from `bench/results.json`; none may be invented.

## Product Principles
- Show the algorithm working, not a description of it.
- Every number is measured, never staged.
- Failure is allowed to show (a planner that does not find a path says so).
- Motion everywhere, but each motion explains state.

## Accessibility & Inclusion
(inferred) Respect prefers-reduced-motion; keyboard-operable controls; WCAG AA contrast.
