# Handoff — running BLAST in the browser

For Claude Code (or whoever picks up the port). Two independent halves that meet at
one interface. Read "The contract" first; everything else is detail.

---

## Why

`Hadeon Robotics.dc.html` has a live 3D hero: a UR5e plans stop-to-stop between two
draggable poses around draggable obstacles, and the readouts show solve time,
function evaluations, clearance and trajectory duration.

The solver behind it today is a **JS port of BLAST's formulation**, not BLAST. It is
faithful where it matters and honest about where it isn't:

| Piece | Status |
| --- | --- |
| `Bspline::compute_basis` (uniform clamped knots) | ported line-by-line |
| `Bspline::compute_control` (3 clamped ctrl pts per end) | ported |
| Objective = T (`Objective::time_weight`) | ported |
| `with_segments` constraint reduction | ported (worst case per segment × 4 classes) |
| `guess_shot_mean` shotgun initial guess | ported, plus a detour-seeded variant |
| `max_tries` retry loop | ported |
| UR5e limits (`UR5e.hpp`) | verbatim |
| **Native SQP** | **substituted** — penalty descent, FD gradients, backtracking line search |
| Robot collision model | simplified — swept segment envelope, not `UR5e.hpp`'s 7 capsules |
| Obstacles | spheres only (BLAST supports boxes/capsules/dynamic too) |

Current numbers in-browser: 9–25 ms, ~450–1800 evaluations, 37-dim x, feasible on
essentially all random scenes. Replacing the substituted solver with the real one
should be **faster and strictly more correct**.

There is no WASM or JS build of BLAST today — header-only C++ plus Python bindings.
That's the gap this handoff closes.

---

## The contract

Both halves build against this. Define it first, don't renegotiate it mid-way.

```c
// bindings.cpp — flat arrays, no JSON, no Embind. Called from JS via ccall/direct heap.

// Returns 0 on success, negative on error. All arrays are caller-allocated in the
// WASM heap; the JS side owns them.
int blast_solve(
    const double* task,      // 6 joints x 6 [p,v,a start | p,v,a goal], row-major = 36
    const double* limits,    // 6*3: pos_max[6], vel_max[6], acc_max[6]
    const double* spheres,   // n_spheres x 4: cx, cy, cz, r
    int           n_spheres,
    const double* boxes,     // n_boxes x 15: cx,cy,cz, ex,ey,ez, R[9]  (0 for v1)
    int           n_boxes,
    int           n_ctrl,    // 12
    int           n_points,  // 28 for interactive; 110 matches example_03
    int           degree,    // 5
    int           max_tries, // 4
    double        success_tolerance, // 0.01
    double*       out_q,     // OUT n_points x 6 joint positions, row-major
    double*       out_meta   // OUT 5: T, compute_time_ms, num_eval, max_constraint_value, success(0|1)
);
```

Notes on the shape, and why:

- **Flat doubles, not JSON.** The hero re-solves on a 140 ms drag throttle; string
  marshalling per solve is wasted budget. JS writes into the heap, calls, reads out.
- **`out_q` is sampled joint positions, not control points.** The web side renders
  and interpolates; it does not need the spline or the basis. Keep the spline inside.
- **`out_meta` mirrors `blast::Result`** — the readouts already display exactly these
  five values, so nothing on the web side has to change when the real solver lands.
- **Boxes are in the signature but unused in v1.** The hero uses spheres. Wiring the
  field now avoids a signature change later; pass `n_boxes = 0`.
- **Collision model.** v1 may keep using the simplified envelope so the JS and WASM
  paths are comparable. Switching to `UR5e.hpp`'s real capsule list is a v2 upgrade
  and will change clearance numbers — expect that, don't treat it as a regression.

---

## Half one — the port (needs a shell; this is the Claude Code half)

### Step 0, before anything else (1–2 h)

```
emcmake cmake -B build-wasm -DBLAST_USE_NATIVE_SQP=ON
cmake --build build-wasm --target example_03_collision_avoidance
```

This single probe decides the whole estimate. `blast/` is header-only, so if
`blast/extern` (293 files) compiles clean, the rest is routine plumbing. **Report
back after this step before committing to the rest.**

### Then

1. **Emscripten preset** in `CMakePresets.json`. Flags:
   `-O3 -flto -msimd128 -sMODULARIZE -sEXPORT_ES6 -sENVIRONMENT=web
   -sEXPORTED_FUNCTIONS=_blast_solve,_malloc,_free
   -sEXPORTED_RUNTIME_METHODS=ccall,HEAPF64`
   Fixed heap (`-sINITIAL_MEMORY=64MB`, no `ALLOW_MEMORY_GROWTH`) — predictable, no
   realloc stalls mid-drag.
2. **`bindings.cpp`** implementing the contract. `python/src` already defines a
   working FFI boundary over the same API — copy its shape, skip the design work.
3. **Verification fixtures.** Same inputs through native and WASM; assert `out_q`
   agrees to ~1e-9 and `success` matches. This is the step that catches a bad float
   build, and it's worth the 4–6 h.

### Known constraints

- **`BLAST_USE_NATIVE_SQP=ON` is required** if you ever want the float build —
  external NLopt's C API is double-only, and `blast_optimization.hpp` has an
  `#error` guarding exactly this. v1 should be doubles anyway.
- **Skip the threadpool for v1.** `-pthread` needs `SharedArrayBuffer`, which needs
  COOP/COEP headers on the serving origin. GitHub Pages cannot set those — and the
  site is served from `HadeonRobotics.github.io`. That single fact probably settles
  it: single-threaded, or move hosting.
- **`blast/gpu` does not cross over.** No CUDA equivalent in WASM.
- Budget: happy path ~2 days, realistic 4–5, worst case (portability fights in
  `extern`) 1.5–2 weeks. The probe tells you which.

### Expected performance (estimates, not measurements)

- Scalar C++ → WASM: 1.3–2× native; with `-msimd128` on hand-written kernels,
  closer to 1.2–1.5×.
- Single-threaded costs whatever the threadpool buys — plausibly 2–4× on multicore.
- Net guess: if native single-threaded `example_03` is 5–15 ms, expect 8–30 ms in
  the browser. Comfortably inside the 140 ms drag throttle.
- Payload 300 KB – 1 MB gzipped; 50–150 ms to fetch and instantiate with streaming
  compilation.

---

## Half two — the web side (my half)

Not yet written; waiting on the probe. When it goes in:

1. `blast-wasm.js` — ES-module wrapper: instantiate, allocate the six heap buffers
   once and reuse them, `solve(task, limits, spheres) → {Q, T, ms, evals, viol, ok}`.
2. `Hadeon Robotics.dc.html` — `plan()` calls the WASM solver when the module is
   ready, keeping the existing JS port as the fallback. Two rules:
   - **The demo must never break on a failed load.** If the module 404s or throws,
     the JS port keeps running and the page looks identical.
   - **The caption must say which solver is live.** Right now it reads "Ported from
     BLAST's own formulation; the solver itself is C++." When the real solver is
     running, that becomes a claim about the actual library and the copy changes to
     match. Don't ship WASM without updating it, and don't claim it before.
3. Readouts need no change — `out_meta` is already their shape.

---

## What's in this project

| File | What |
| --- | --- |
| `Hadeon Robotics.dc.html` | the upgraded site, all pages, hero solver in the logic class |
| `Current Site.dc.html` | recreation of the live site, before-state baseline |
| `github.md` | repo association, sync record, screen map |
| `images/` | logo + team headshots, copied from the site repo |
| `_ds/industry-…/` | the Industry design system bundle the site is built on |

The solver lives in `Hadeon Robotics.dc.html`'s logic class. Start at `plan()` and
read down: `guess()` → `solveFrom()` → `merit()` → `evalX()` → `clearance()`.
`buildSpline()` and `control()` are the ported BLAST pieces and carry comments
naming their C++ counterparts.
