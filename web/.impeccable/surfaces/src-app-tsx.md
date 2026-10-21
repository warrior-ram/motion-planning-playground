---
version: 1
slug: "src-app-tsx"
primary_target: "src/App.tsx"
related_targets: []
---

Scope: the single page of the motion planning playground (web/src/App.tsx and its components). Visitor mode: Operate.
Audience and job: recruiters and robotics engineers on a laptop or phone, in a ten-second to few-minute visit. They draw obstacles, move the start and goal, run A*, RRT and RRT*, compare the numbers, drag a planar arm, and look at the 20-map benchmark.
Constraints: the owner asked for no questions; every choice was made in-thread. Code-led build (no image generation available). Motion everywhere, but each motion explains state. prefers-reduced-motion is respected.

## Direction contract

THESIS: Comparing three planners is a race, so the page reads like a motorsport timing broadcast: a live circuit (the canvas) and a timing tower that re-sorts as results land. It refuses the category default, which is a grey control panel with a lone canvas and a raw stats table.

OWN-WORLD: Carbon ground (#121417) with panels a half-step lighter. Barlow Condensed for names and tabular numerals, Barlow for body text. Three series colours act as team liveries: A* orange #FF7A45, RRT teal #34D1C4, RRT* yellow-lime #E8E15A. Purple #B072FF marks the best value in a column (the fastest-sector convention), red #FF4D5E marks a collision or failure. The goal is a chequered disc, the start a white grid-slot disc. Primary buttons are solid white with carbon text.

STORY: The visitor sees a race already running, understands that each colour is a planner, then draws their own walls and races again. Failure is shown honestly ("No path in 5000 iterations").

FIRST VIEWPORT: The header carries the name, a one-line claim and the mode tabs. Below it, the canvas holds about 68% of the width, with a control column to the right: tools, algorithm choice, Plan and Race buttons, then the timing tower. The map preset rail runs under the canvas. The primary action, "Race all three", sits at the top of the control column.

FORM: Motorsport timing graphics, my third-ranked candidate (1 orienteering map, 2 transit diagram, 3 motorsport timing graphics, 4 drafting sheet, 5 chess problem diagram, 6 lab graph paper, 7 topographic survey). Seed key bc0843b3. Raises, each named for the challenger that donated it: from Ikeda datamatics, data-density courage in the tower's tabular numerals; from Airport wayfinding, a single status band that repaints instantly on state change; from Streaming wall, a preset rail where the focused tile lifts and its neighbours dim; from Crouwel grid, the A* grid drawn as a real visible armature only while A* is in play; from Botanical folio, the three planners registered at identical scale on one circuit.

Signature move: the timing tower re-sorts with layout animation while three robots drive their found paths at equal speed, so the shortest path visibly crosses the line first.

FINISH: unreviewed and undocumented is unfinished; this build ends with the finish review, the verdict, DESIGN.md, and every shipping raster carrying its provenance
