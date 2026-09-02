repo: HadeonRobotics/HadeonRobotics.github.io
branch: main

## Last sync
date: 2026-08-30T15:30:00Z

### Updated in this project
- Recreated the live home page as `Current Site.dc.html` (before-state baseline)
- Built `Hadeon Robotics.dc.html` — full upgraded site on the Industry design system
- Hero solver rewritten as a direct port of BLAST's formulation (see below)
- Copied the Hadeon logo and five team headshots from `images/`

## Screen map
| Screen | Repo files |
| --- | --- |
| Current Site.dc.html | index.html, css/styles.css, js/components.js, images/HadeonLogo.svg, images/*_headshot_compressed.jpeg |
| Hadeon Robotics.dc.html | index.html, solutions.html, about.html, news.html, careers.html, js/components.js, css/styles.css, images/ |

## Related repositories
Dynamium-Lab/blast @ main — read `readme.md`, `examples/example_02_trajectory_optimization.cpp`,
`examples/example_03_collision_avoidance.cpp` on 2026-08-29. The hero demo's problem setup
On 2026-08-30 also read `blast/trajectory/bspline.hpp`, `blast/optimization/initial_guess.hpp`,
`blast/optimization/objective.hpp`, `blast/blast_optimization.hpp`, `blast/blast_task.hpp`,
`blast/blast_world.hpp`, `blast/manipulator/UR5e.hpp`.

The hero planner is now a line-by-line JS port of BLAST's formulation:
`Bspline::compute_basis` (uniform clamped knots), `Bspline::compute_control`
(three clamped control points per end from Task::stop_to_stop), objective = T
(`Objective::time_weight`), `with_segments` constraint reduction (worst case per
segment for position / velocity / acceleration / external collision),
`guess_shot_mean` shotgun initial guess, `max_tries` retry loop, and UR5e.hpp
limits verbatim. BLAST has no WASM or JS build (header-only C++ plus Python
bindings), so its native SQP is replaced in the browser by penalty descent with
finite-difference gradients and a backtracking line search — the only substituted
piece. No BLAST source is vendored.
