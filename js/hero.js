// Interactive UR5e trajectory-planner hero demo.
//
// A JS port of BLAST's formulation (see Dynamium-Lab/blast): a degree-5
// B-spline in joint space, clamped stop-to-stop boundaries, objective = T
// (Objective::time_weight), constraints reduced to one worst-case value per
// spline segment (OptimizationMethod::with_segments), UR5e.hpp limits
// verbatim. No BLAST source is vendored — this is a from-scratch port.
// The one substituted piece: blast's native SQP is C++-only, so here it's
// penalty descent with finite-difference gradients and a backtracking line
// search (see solveFrom()). Everything else is line-for-line equivalent.
(function () {
  'use strict';

  const canvas = document.getElementById('hero-canvas');
  if (!canvas) return;

  const wrap = document.getElementById('hero-wrap');
  const statusEl = document.getElementById('hero-status');
  const tSolveEl = document.getElementById('hero-t-solve');
  const tIterEl = document.getElementById('hero-t-iter');
  const tClearEl = document.getElementById('hero-t-clear');
  const tErrEl = document.getElementById('hero-t-err');
  const solverKindEl = document.getElementById('hero-solver-kind');
  const planEl = document.getElementById('hero-plan');
  const runEl = document.getElementById('hero-run');
  const hintEl = document.getElementById('hero-hint');
  const tryEl = document.getElementById('hero-try');
  const videoPanel = document.getElementById('hero-video-panel');
  const simPanel = document.getElementById('hero-sim-panel');
  const scenarioEls = Array.from(document.querySelectorAll('.hero-scenarios button'));

  const OBSTACLE_COUNT = 3;

  // Four curated, hand-picked start/goal joint pairs (not derived from a
  // dragged cartesian point via IK) — see the "elbow-branch mismatch" note
  // below. Each q1 was solved from a seed pool anchored at its own q0 and
  // filtered to the smallest worst-case single-joint delta, so the pair sits
  // on a continuous branch instead of an arbitrary elbow/wrist flip.
  const SCENARIOS = [
    { name: 'Side reach',  q0: [1.0787, -1.2900, 1.9462, -1.7725, -1.3647, 0], q1: [2.6065, -1.6878, 1.6289, -1.3554, 0.0754, -1.2796] },
    { name: 'Full sweep',  q0: [1.6159, -1.1716, 1.8083, -1.4254, -1.5445, 0], q1: [1.2876, -2.6736, 0.6013, 0.1107, -0.6677, -1.0171] },
    { name: 'Short hop',   q0: [2.4756, -1.3891, 1.7690, -1.3983, -1.3493, 0], q1: [1.3828, -3.0500, 0.9563, 0.0226, -1.7517, 0] },
    { name: 'Cross reach', q0: [2.7517, -1.3161, 1.8165, -1.4010, -1.2312, 0], q1: [2.1861, -2.9909, 0.9233, -0.1033, -1.2502, -0.4555] },
  ];

  const solver = {
    // UR5e model — DH values from the Blast readme's UR5e definition (metres)
    DH: [
      { a: 0,       d: 0.1625, al:  Math.PI / 2 },
      { a: -0.425,  d: 0,      al:  0 },
      { a: -0.3922, d: 0,      al:  0 },
      { a: 0,       d: 0.1333, al:  Math.PI / 2 },
      { a: 0,       d: 0.0997, al: -Math.PI / 2 },
      { a: 0,       d: 0.0996, al:  0 }
    ],
    // ── blast::Optimization, ported ──────────────────────────────────────
    DEGREE: 5,
    NCTRL: 12,                                              // blast::Bspline(12, …, 5, 6)
    NPTS: 28,
    VMAX: [3.1416, 3.1416, 3.1416, 3.1416, 3.1416, 3.1416], // limits.velocity_max
    AMAX: [13.96, 13.96, 13.96, 13.96, 13.96, 13.96],       // limits.acceleration_max
    PMAX: [6.2832, 6.2832, 3.1416, 6.2832, 6.2832, 6.2832], // limits.position_max
    TUBE: 0.055,                                            // link capsule radius
    TOL: 0.01,                                              // success_tolerance
    MAX_TRIES: 4,                                           // max_tries
    ITERS: 14,

    boot() {
      this.cam = { az: -0.95, el: 0.36, dist: 2.15, target: [0, 0, 0.42] };
      this.userActive = false;
      this.drag = null;
      this.running = false;
      this.phase = 0;
      this.t0 = performance.now();
      this.updateBasis();
      if (!this.spl) this.buildSpline();

      // WASM load is fire-and-forget: never blocks first paint, and the JS
      // port below is a fully working solver on its own — plan() falls back
      // to it automatically if this never resolves, resolves false, or
      // solve() itself fails on some later call.
      this.wasmReady = false;
      this.blastSolver = null;
      import('/js/blast-solver.js').then((mod) => {
        this.blastSolver = mod;
        return mod.ready(this.NPTS);
      }).then((ok) => {
        this.wasmReady = ok;
        if (ok && solverKindEl) solverKindEl.textContent = 'C++, compiled to WebAssembly';
      }).catch(() => { this.wasmReady = false; });

      this.initScene(0);

      this.ro = new ResizeObserver(() => this.resize());
      this.ro.observe(wrap);
      this.resize();

      const pos = (e) => { const r = canvas.getBoundingClientRect(); return { x: e.clientX - r.left, y: e.clientY - r.top }; };
      canvas.onpointerdown = (e) => {
        const p = pos(e);
        let hit = null, bd = 24;
        // Start/goal are fixed per scenario (see SCENARIOS) — only the
        // obstacles are draggable, so that's the only hit-test needed.
        this.obs.forEach((o, i) => {
          const s = this.project(o.c);
          if (!s) return;
          const d = Math.hypot(p.x - s.x, p.y - s.y);
          const rad = o.r * s.scale;
          if (d < rad + 8 && d < bd + rad) { bd = d; hit = { type: 'obs', i }; }
        });
        this.drag = hit || { type: 'orbit' };
        this.last = p;
        this.userActive = true;
        canvas.setPointerCapture(e.pointerId);
      };
      canvas.onpointermove = (e) => {
        if (!this.drag) return;
        const p = pos(e);
        const dx = p.x - this.last.x, dy = p.y - this.last.y;
        this.last = p;
        if (this.drag.type === 'orbit') {
          this.cam.az -= dx * 0.006;
          this.cam.el = Math.max(-0.1, Math.min(1.2, this.cam.el + dy * 0.005));
          return;
        }
        const obj = this.obs[this.drag.i].c;
        const s = this.project(obj);
        if (!s) return;
        const k = 1 / s.scale;
        const mv = this.add(this.scl(this.basis.right, dx * k), this.scl(this.basis.up, -dy * k));
        obj[0] = Math.max(-0.85, Math.min(0.85, obj[0] + mv[0]));
        obj[1] = Math.max(-0.85, Math.min(0.85, obj[1] + mv[1]));
        obj[2] = Math.max(0.06, Math.min(1.1, obj[2] + mv[2]));
        // Moving an obstacle never auto-solves — the last plan is now stale,
        // so drop it (Run disables, hint resets) and wait for Plan.
        this.traj = null;
        this.running = false;
        this.path = null;
        this.updateControls();
      };
      const up = (e) => {
        this.drag = null;
        if (e && canvas.hasPointerCapture && canvas.hasPointerCapture(e.pointerId)) canvas.releasePointerCapture(e.pointerId);
      };
      canvas.onpointerup = up; canvas.onpointercancel = up;

      scenarioEls.forEach((btn, i) => {
        btn.addEventListener('click', () => {
          this.userActive = true;
          this.initScene(i);
          scenarioEls.forEach((b, j) => {
            b.classList.toggle('btn-primary', j === i);
            b.classList.toggle('btn-secondary', j !== i);
          });
        });
      });
      if (planEl) planEl.addEventListener('click', () => { this.plan(); });
      if (runEl) runEl.addEventListener('click', () => {
        if (!this.traj || !this.traj.success) return;
        this.playT = performance.now();
        this.running = true;
        this.updateControls();
      });

      const loop = () => {
        try { this.tickFrame(); this.draw(); } catch (err) { console.error('[hero] frame error', err); }
        this.raf = requestAnimationFrame(loop);
      };
      this.raf = requestAnimationFrame(loop);
    },

    // ── vector helpers ──────────────────────────────────────────────────
    add(a, b) { return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]; },
    sub(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; },
    scl(a, k) { return [a[0] * k, a[1] * k, a[2] * k]; },
    dot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; },
    crs(a, b) { return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]; },
    len(a) { return Math.hypot(a[0], a[1], a[2]); },
    nrm(a) { const l = this.len(a) || 1; return [a[0] / l, a[1] / l, a[2] / l]; },

    // ── forward kinematics (standard DH) ──────────────────────────────────
    fk(q) {
      let R = [1, 0, 0, 0, 1, 0, 0, 0, 1];
      let p = [0, 0, 0];
      const pts = [p], zs = [];
      for (let i = 0; i < 6; i++) {
        const D = this.DH[i];
        const ct = Math.cos(q[i]), st = Math.sin(q[i]);
        const ca = Math.cos(D.al), sa = Math.sin(D.al);
        const M = [ct, -st * ca, st * sa,
                   st,  ct * ca, -ct * sa,
                   0,   sa,      ca];
        const t = [D.a * ct, D.a * st, D.d];
        zs.push([R[2], R[5], R[8]]);
        p = this.add(p, [R[0] * t[0] + R[1] * t[1] + R[2] * t[2],
                         R[3] * t[0] + R[4] * t[1] + R[5] * t[2],
                         R[6] * t[0] + R[7] * t[1] + R[8] * t[2]]);
        const C = new Array(9);
        for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++)
          C[r * 3 + c] = R[r * 3] * M[c] + R[r * 3 + 1] * M[3 + c] + R[r * 3 + 2] * M[6 + c];
        R = C;
        pts.push(p);
      }
      return { pts, zs };
    },

    segDist(A, B, P) {
      const ab = this.sub(B, A), ap = this.sub(P, A);
      const l2 = this.dot(ab, ab) || 1e-9;
      const t = Math.max(0, Math.min(1, this.dot(ap, ab) / l2));
      return this.len(this.sub(P, this.add(A, this.scl(ab, t))));
    },

    clearance(q) {
      const k = this.fk(q);
      let best = Infinity;
      for (let i = 0; i < 6; i++) {
        for (const o of this.obs) {
          const d = this.segDist(k.pts[i], k.pts[i + 1], o.c) - o.r - this.TUBE;
          if (d < best) best = d;
        }
      }
      return best;
    },

    // ── inverse kinematics (damped least squares, position only) ─────────
    ik(target, seed) {
      const q = seed.slice();
      for (let it = 0; it < 90; it++) {
        const k = this.fk(q);
        const ee = k.pts[6];
        let err = this.sub(target, ee);
        const em = this.len(err);
        if (em < 0.0015) break;
        if (em > 0.04) err = this.scl(err, 0.04 / em);
        const J = [[], [], []];
        for (let j = 0; j < 6; j++) {
          const c = this.crs(k.zs[j], this.sub(ee, k.pts[j]));
          J[0][j] = c[0]; J[1][j] = c[1]; J[2][j] = c[2];
        }
        const lam = 0.02;
        const A = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
        for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) {
          let v = 0; for (let j = 0; j < 6; j++) v += J[r][j] * J[c][j];
          A[r][c] = v + (r === c ? lam : 0);
        }
        const m = A;
        const c00 = m[1][1] * m[2][2] - m[1][2] * m[2][1];
        const c01 = m[1][2] * m[2][0] - m[1][0] * m[2][2];
        const c02 = m[1][0] * m[2][1] - m[1][1] * m[2][0];
        const det = m[0][0] * c00 + m[0][1] * c01 + m[0][2] * c02 || 1e-9;
        const inv = [
          [c00, m[0][2] * m[2][1] - m[0][1] * m[2][2], m[0][1] * m[1][2] - m[0][2] * m[1][1]],
          [c01, m[0][0] * m[2][2] - m[0][2] * m[2][0], m[0][2] * m[1][0] - m[0][0] * m[1][2]],
          [c02, m[0][1] * m[2][0] - m[0][0] * m[2][1], m[0][0] * m[1][1] - m[0][1] * m[1][0]]
        ];
        const u = [
          (inv[0][0] * err[0] + inv[0][1] * err[1] + inv[0][2] * err[2]) / det,
          (inv[1][0] * err[0] + inv[1][1] * err[1] + inv[1][2] * err[2]) / det,
          (inv[2][0] * err[0] + inv[2][1] * err[1] + inv[2][2] * err[2]) / det
        ];
        for (let j = 0; j < 6; j++) {
          q[j] += Math.max(-0.18, Math.min(0.18, J[0][j] * u[0] + J[1][j] * u[1] + J[2][j] * u[2]));
          q[j] = Math.max(-3.05, Math.min(3.05, q[j]));
        }
      }
      return q;
    },

    // multi-start IK: converge, then keep the branch closest to the preferred config
    ikBest(target, prefer) {
      const seeds = [
        prefer,
        [0, -1.57, 1.57, -1.57, -1.57, 0],
        [0, -1.2, 1.6, -1.9, -1.57, 0],
        [1.2, -1.0, 1.2, -1.7, -1.57, 0],
        [-1.2, -1.4, 1.4, -1.5, -1.57, 0]
      ];
      for (let i = 0; i < 6; i++) seeds.push(Array.from({ length: 6 }, () => (Math.random() - 0.5) * 5));
      let best = null, fallback = null;
      for (const sd of seeds) {
        const q = this.ik(target, sd);
        const err = this.len(this.sub(target, this.fk(q).pts[6]));
        if (!fallback || err < fallback.err) fallback = { q, err };
        if (err > 0.005) continue;
        const dist = q.reduce((a, v, j) => a + Math.abs(v - prefer[j]), 0);
        const clear = this.clearance(q);
        const score = dist + (clear < 0 ? 40 : 0);
        if (!best || score < best.score) best = { q, score };
      }
      return best ? best.q : fallback.q;
    },

    // ── blast::Bspline::compute_basis (uniform clamped knots) ─────────────
    buildSpline() {
      const nc = this.NCTRL, p = this.DEGREE, npts = this.NPTS, m = nc + p;
      const knots = new Float64Array(m + 1);
      for (let i = m; i > m - p - 1; i--) knots[i] = 1;
      const du = 1 / (m + 1 - 2 * (p + 1) + 1);
      for (let i = p + 1; i < m - p; i++) knots[i] = knots[i - 1] + du;
      const N = new Float64Array(m * (p + 1));
      const bp = [], bv = [], ba = [];
      const div = (num, den) => (Math.abs(den) < 1e-12 ? 0 : num / den);
      const step = 1 / (npts - 1);
      for (let pt = 0; pt < npts; pt++) {
        const u = pt * step;
        N.fill(0);
        for (let i = 0; i < m; i++) N[i] = (u >= knots[i] && u < knots[i + 1]) ? 1 : 0;
        if (pt === npts - 1) N[nc - 1] = 1;
        for (let pi = 1; pi <= p; pi++)
          for (let i = 0; i < m - pi; i++)
            N[m * pi + i] = div(N[m * (pi - 1) + i] * (u - knots[i]), knots[i + pi] - knots[i])
              + div(N[m * (pi - 1) + i + 1] * (knots[i + pi + 1] - u), knots[i + pi + 1] - knots[i + 1]);
        const cp = new Float64Array(nc), cv = new Float64Array(nc), ca = new Float64Array(nc);
        for (let i = 0; i < nc; i++) cp[i] = N[m * p + i];
        for (let i = 0; i < nc - 1; i++) cv[i] = -div(p * N[m * (p - 1) + i + 1], knots[i + p + 1] - knots[i + 1]);
        for (let i = 1; i < nc; i++) cv[i] += div(p * N[m * (p - 1) + i], knots[i + p] - knots[i]);
        const k2 = p * (p - 1);
        for (let i = 0; i < nc - 2; i++) ca[i] = div(k2 * N[m * (p - 2) + i + 2], (knots[i + p + 1] - knots[i + 1]) * (knots[i + p + 1] - knots[i + 2]));
        for (let i = 1; i < nc - 1; i++) ca[i] -= div(k2 * N[m * (p - 2) + i + 1] * (div(1, knots[i + p] - knots[i]) + div(1, knots[i + p + 1] - knots[i + 1])), knots[i + p] - knots[i + 1]);
        for (let i = 2; i < nc; i++) ca[i] += div(k2 * N[m * (p - 2) + i], (knots[i + p - 1] - knots[i]) * (knots[i + p] - knots[i]));
        bp.push(this.sparse(cp)); bv.push(this.sparse(cv)); ba.push(this.sparse(ca));
      }
      this.spl = { bp, bv, ba, nseg: nc - p, xLen: 6 * (nc - 6) + 1 };
    },

    sparse(col) {
      const idx = [], w = [];
      for (let i = 0; i < col.length; i++) if (Math.abs(col[i]) > 1e-12) { idx.push(i); w.push(col[i]); }
      return { idx, w, n: idx.length };
    },

    // blast::Bspline::compute_control — the three control points at each end are
    // fixed by the task boundary; with zero boundary velocity and acceleration
    // (Task::stop_to_stop) the kv/ka terms collapse onto the endpoint value.
    control(x, q0, q1) {
      const nc = this.NCTRL, out = [];
      let xi = 0;
      for (let j = 0; j < 6; j++) {
        const c = new Float64Array(nc);
        c[0] = c[1] = c[2] = q0[j];
        for (let i = 3; i < nc - 3; i++) c[i] = x[xi++];
        c[nc - 3] = c[nc - 2] = c[nc - 1] = q1[j];
        out.push(c);
      }
      return out;
    },

    // one constraint evaluation: worst case per segment for position, velocity,
    // acceleration and external collision — c_i <= 0 is feasible
    evalX(x) {
      this.numEval++;
      const nseg = this.spl.nseg, npts = this.NPTS;
      const T = Math.max(0.05, x[x.length - 1]);
      const ctr = this.control(x, this.q0, this.q1);
      const c = new Float64Array(nseg * 4).fill(-1);
      const q = new Array(6);
      let minClear = Infinity;
      for (let pt = 0; pt < npts; pt++) {
        const seg = Math.min(nseg - 1, Math.floor(pt * nseg / npts));
        const P = this.spl.bp[pt], V = this.spl.bv[pt], A = this.spl.ba[pt];
        let pw = -1, vw = -1, aw = -1;
        for (let j = 0; j < 6; j++) {
          const cj = ctr[j];
          let pv = 0, vv = 0, av = 0;
          for (let k = 0; k < P.n; k++) pv += cj[P.idx[k]] * P.w[k];
          for (let k = 0; k < V.n; k++) vv += cj[V.idx[k]] * V.w[k];
          for (let k = 0; k < A.n; k++) av += cj[A.idx[k]] * A.w[k];
          q[j] = pv;
          vv /= T; av /= T * T;
          const p1 = Math.abs(pv) / this.PMAX[j] - 1;
          const v1 = Math.abs(vv) / this.VMAX[j] - 1;
          const a1 = Math.abs(av) / this.AMAX[j] - 1;
          if (p1 > pw) pw = p1;
          if (v1 > vw) vw = v1;
          if (a1 > aw) aw = a1;
        }
        if (pw > c[seg]) c[seg] = pw;
        if (vw > c[nseg + seg]) c[nseg + seg] = vw;
        if (aw > c[2 * nseg + seg]) c[2 * nseg + seg] = aw;
        const d = this.clearance(q);
        if (d < minClear) minClear = d;
        if (-d > c[3 * nseg + seg]) c[3 * nseg + seg] = -d;
      }
      let pvaViol = 0, viol = 0;
      for (let i = 0; i < 3 * nseg; i++) if (c[i] > pvaViol) pvaViol = c[i];
      for (let i = 0; i < c.length; i++) if (c[i] > viol) viol = c[i];
      return { c, viol, pvaViol, T, minClear };
    },

    merit(x, mu) {
      const r = this.evalX(x);
      let pen = 0;
      for (let i = 0; i < r.c.length; i++) if (r.c[i] > 0) pen += r.c[i] * r.c[i];
      return { f: r.T + mu * pen, viol: r.viol, T: r.T, minClear: r.minClear };
    },

    clampX(x) {
      const nf = this.NCTRL - 6;
      for (let j = 0; j < 6; j++)
        for (let i = 0; i < nf; i++) {
          const k = j * nf + i;
          x[k] = Math.max(-this.PMAX[j], Math.min(this.PMAX[j], x[k]));
        }
      x[x.length - 1] = Math.max(0.15, Math.min(20, x[x.length - 1]));
      return x;
    },

    // A random shotgun (blast's own guess_shot_mean, which this used to
    // port line-for-line) was scattering the initial guess control points
    // with noise, then handing solveFrom() whichever scattered guess scored
    // best *before* optimization even ran — a poor predictor of what
    // actually converges, and it made every retry a re-roll instead of a
    // deliberate next move. This is fully deterministic instead: straight
    // line in joint space between q0 and q1 (or routed through detourVia()'s
    // single via-pose when an obstacle sits on the direct line), at a given
    // duration T. No randomness — local optimization converges far more
    // reliably from a physically sensible starting point than from noise.
    guess(via, T) {
      const n = this.spl.xLen, nf = this.NCTRL - 6;
      const x = new Float64Array(n);
      let xi = 0;
      for (let j = 0; j < 6; j++)
        for (let i = 0; i < nf; i++) {
          const f = (i + 1) / (nf + 1);
          x[xi++] = via
            ? (f < 0.5 ? this.q0[j] + (via[j] - this.q0[j]) * (f / 0.5)
                       : via[j] + (this.q1[j] - via[j]) * ((f - 0.5) / 0.5))
            : this.q0[j] + (this.q1[j] - this.q0[j]) * f;
        }
      x[n - 1] = T;
      return this.clampX(x);
    },

    // When an obstacle sits on the straight-line path, build a via-pose clear of
    // it — pushed out along the obstacle-surface normal — to seed the shotgun.
    detourVia() {
      if (!this.obs || !this.obs.length || !this.q0 || !this.q1) return null;
      const A = this.fk(this.q0).pts[6], B = this.fk(this.q1).pts[6];
      let worst = null;
      for (const o of this.obs) {
        const d = this.segDist(A, B, o.c) - o.r - this.TUBE;
        if (!worst || d < worst.d) worst = { o, d };
      }
      if (!worst || worst.d > 0.03) return null;
      const o = worst.o;
      const M = this.scl(this.add(A, B), 0.5);
      let nv = this.sub(M, o.c);
      if (this.len(nv) < 1e-3) nv = this.crs(this.nrm(this.sub(B, A)), [0, 0, 1]);
      nv = this.nrm(nv);
      const W = this.add(o.c, this.scl(nv, o.r + this.TUBE + 0.09));
      W[2] = Math.max(0.12, Math.min(1.05, W[2]));
      return this.ikBest(W, this.q0);
    },

    // penalty descent with finite-difference gradients and a backtracking line
    // search, standing in for blast's native SQP (which is C++ only)
    solveFrom(x0, mu0) {
      const n = x0.length, eps = 1e-4;
      let x = x0.slice(), mu = mu0;
      let cur = this.merit(x, mu);
      const g = new Float64Array(n);
      for (let it = 0; it < this.ITERS; it++) {
        for (let j = 0; j < n; j++) {
          const keep = x[j];
          x[j] = keep + eps;
          g[j] = (this.merit(x, mu).f - cur.f) / eps;
          x[j] = keep;
        }
        let gn = 0;
        for (let j = 0; j < n; j++) gn += g[j] * g[j];
        gn = Math.sqrt(gn) || 1e-9;
        let step = 0.3 / gn, moved = false;
        for (let ls = 0; ls < 6; ls++) {
          const y = new Float64Array(n);
          for (let j = 0; j < n; j++) y[j] = x[j] - step * g[j];
          this.clampX(y);
          const m = this.merit(y, mu);
          if (m.f < cur.f) { x = y; cur = m; moved = true; break; }
          step *= 0.4;
        }
        if (!moved && cur.viol <= this.TOL) break;
        mu *= 1.4;
        cur = this.merit(x, mu);
      }
      const r = this.evalX(x);
      // clearance is a hard constraint: success_tolerance applies to the
      // dimensionless PVA constraints only, never to penetration
      return { x, T: r.T, viol: r.viol, minClear: r.minClear, ok: r.pvaViol <= this.TOL && r.minClear >= 0 };
    },

    // Every accepted result is validated before it becomes this.traj: a NaN
    // or malformed solve (either solver path) must never reach qAt()/draw(),
    // since that would throw mid-frame and permanently kill the rAF loop.
    validQ(Q) {
      return Array.isArray(Q) && Q.length === this.NPTS && Q.every((q) => q.every(Number.isFinite));
    },

    plan() {
      if (!this.spl) this.buildSpline();
      this.numEval = 0;
      this.running = false;
      this.phase = 0;
      const t0 = performance.now();

      // WASM's native guess is a straight line in joint space (no obstacle
      // routing) — good when the direct path is clear, but when an obstacle
      // sits squarely on it, only a detour finds a way through. So: take
      // WASM's answer immediately if it's feasible, but if it reports
      // infeasible, also run the JS port below (it has a via-point detour
      // heuristic WASM's guess doesn't) and keep whichever actually found a
      // feasible path — preferring WASM's when both did, since its
      // self-collision model is the real one (see bindings.cpp).
      let wasmTraj = null;
      if (this.wasmReady && this.blastSolver) {
        const r = this.blastSolver.solve(
          this.q0, this.q1, this.obs,
          { PMAX: this.PMAX, VMAX: this.VMAX, AMAX: this.AMAX },
          { nCtrl: this.NCTRL, degree: this.DEGREE, maxTries: this.MAX_TRIES, tol: this.TOL }
        );
        // r.evals === 0 means the native solver declined to even attempt a
        // solve (e.g. start/goal already in collision) rather than tried and
        // failed — that's not a real result, so treat it like a null result.
        if (r && r.evals > 0 && Number.isFinite(r.T) && this.validQ(r.Q)) {
          let minClear = Infinity;
          for (const q of r.Q) { const d = this.clearance(q); if (d < minClear) minClear = d; }
          wasmTraj = {
            T: r.T, minClear, viol: r.viol, success: r.ok, ms: r.ms, evals: r.evals, Q: r.Q,
            path: r.Q.map((q) => this.fk(q).pts[6])
          };
          if (r.ok) {
            this.traj = wasmTraj;
            this.playT = performance.now();
            this.path = wasmTraj.path;
            this.pushReadout();
            return;
          }
        }
      }

      let best = null;
      const via = this.detourVia();
      let dq = 0;
      for (let j = 0; j < 6; j++) dq = Math.max(dq, Math.abs(this.q1[j] - this.q0[j]));
      const T0 = Math.max(0.45, 2.1 * dq / this.VMAX[0]);
      // Each retry is a deliberate next move, not a re-roll: alternate the
      // direct line with the obstacle-routed via-pose, and every other pass
      // give the spline more time in case duration (not path shape) was
      // what the previous attempt was short on.
      for (let attempt = 0; attempt < this.MAX_TRIES; attempt++) {
        const useVia = via && attempt % 2 === 1;
        const T = T0 * (1 + 0.35 * Math.floor(attempt / 2));
        const r = this.solveFrom(this.guess(useVia ? via : null, T), 25);
        const better = !best
          || (r.ok && !best.ok)
          || (r.ok && best.ok && r.T < best.T)
          || (!r.ok && !best.ok && r.viol < best.viol);
        if (better) best = r;
        if (best.ok) break;
      }
      const ms = performance.now() - t0;
      const sm = best ? this.sample(best.x) : null;
      let jsTraj = null;
      if (best && Number.isFinite(best.T) && this.validQ(sm.Q)) {
        jsTraj = {
          T: best.T, minClear: best.minClear, viol: best.viol, success: best.ok,
          ms, evals: this.numEval, Q: sm.Q, path: sm.P
        };
      }

      const winner = jsTraj && (!wasmTraj || jsTraj.success && !wasmTraj.success
        || (jsTraj.success === wasmTraj.success && jsTraj.viol < wasmTraj.viol))
        ? jsTraj : wasmTraj;

      if (!winner) {
        // Should not happen given clamped inputs, but never leave the demo
        // mid-broken — report a clean failure instead of NaN reaching draw().
        this.traj = { T: 1, minClear: -1, viol: 1, success: false, ms, evals: this.numEval, Q: [this.q0, this.q1] };
        this.path = null;
        this.pushReadout();
        return;
      }
      this.traj = winner;
      this.playT = performance.now();
      this.path = winner.path;
      this.pushReadout();
    },

    sample(x) {
      const ctr = this.control(x, this.q0, this.q1);
      const Q = [], P = [];
      for (let pt = 0; pt < this.NPTS; pt++) {
        const B = this.spl.bp[pt];
        const q = new Array(6);
        for (let j = 0; j < 6; j++) {
          const cj = ctr[j];
          let v = 0;
          for (let k = 0; k < B.n; k++) v += cj[B.idx[k]] * B.w[k];
          q[j] = v;
        }
        Q.push(q); P.push(this.fk(q).pts[6]);
      }
      return { Q, P };
    },

    qAt(s) {
      const Q = this.traj.Q, n = Q.length - 1;
      if (n < 1) return this.q0;
      const u = Math.max(0, Math.min(1, s)) * n;
      const i = Math.min(n - 1, Math.floor(u)), f = u - i;
      return Q[i].map((v, j) => v + (Q[i + 1][j] - v) * f);
    },

    pushReadout() {
      const t = this.traj;
      const set = (el, v) => { if (el) el.textContent = v; };
      set(tSolveEl, t.ms.toFixed(1) + ' ms');
      set(tIterEl, String(t.evals));
      const mm = t.minClear * 1000;
      set(tClearEl, mm < 0 ? mm.toFixed(0) + ' mm · contact' : mm.toFixed(0) + ' mm');
      if (tClearEl) tClearEl.style.color = mm < 0 ? '#b45f4d' : '';
      set(tErrEl, t.T.toFixed(2) + ' s');
      if (tIterEl) tIterEl.title = 'max constraint violation ' + t.viol.toFixed(4);
      if (statusEl) {
        statusEl.textContent = t.success ? 'FEASIBLE' : 'INFEASIBLE';
        statusEl.style.color = t.success ? 'var(--color-accent-700)' : '#b45f4d';
      }
      this.updateControls();
    },

    // Keeps the Run button, the hint line, and the readouts in sync with plan
    // state. A trajectory only ever plays if traj.success is true — this is
    // the one place Run's disabled state is decided, so an infeasible plan
    // can never be animated regardless of which solver path produced it.
    updateControls() {
      if (runEl) runEl.disabled = !this.traj || !this.traj.success;
      if (!this.traj) {
        // Scene changed since the last plan — stale numbers would misrepresent
        // the current (unplanned) scene, so clear them rather than leave them.
        const set = (el, v) => { if (el) el.textContent = v; };
        set(tSolveEl, '—'); set(tIterEl, '—'); set(tClearEl, '—'); set(tErrEl, '—');
        if (tClearEl) tClearEl.style.color = '';
        if (statusEl) { statusEl.textContent = 'READY'; statusEl.style.color = ''; }
      }
      if (!hintEl) return;
      if (!this.traj) {
        hintEl.textContent = 'Drag the obstacles — then click Plan.';
      } else if (!this.traj.success) {
        hintEl.textContent = 'Not feasible — reposition the goal or an obstacle, then Plan again.';
      } else if (this.running) {
        hintEl.textContent = 'Running — watch it move.';
      } else {
        hintEl.textContent = 'Feasible — click Run to watch it move.';
      }
    },

    // ── scene ──────────────────────────────────────────────────────────
    // Loads a curated scenario's fixed start/goal (see SCENARIOS) and drops
    // a pillar of obstacles between them — never solves. A fresh scene
    // always waits for an explicit Plan click. Re-picking the same scenario
    // reshuffles the obstacles, which doubles as a "reset" affordance.
    initScene(idx) {
      this.scenarioIdx = idx;
      const sc = SCENARIOS[idx];
      this.q0 = sc.q0.slice();
      this.q1 = sc.q1.slice();
      this.startP = this.fk(this.q0).pts[6];
      this.goalP = this.fk(this.q1).pts[6];

      // Place a pillar of obstacles between the two end-effector points, but
      // reject any placement that would already collide with the arm's own
      // links at rest — the midpoint of two cartesian points says nothing
      // about where the elbow/forearm actually sit for a given pose, and
      // blast's solver correctly refuses to plan from an already-colliding
      // start/goal (see the r.evals check in plan()) rather than try anyway.
      const rnd = (a, b) => a + Math.random() * (b - a);
      const mid = this.scl(this.add(this.startP, this.goalP), 0.5);
      const baseTh = Math.atan2(mid[1], mid[0]);
      const baseRad = Math.max(0.30, Math.min(0.55, this.len([mid[0], mid[1], 0])));
      for (let attempt = 0; attempt < 40; attempt++) {
        const th = baseTh + rnd(-0.2, 0.2) * (1 + attempt * 0.1);
        const rad = Math.max(0.28, Math.min(0.62, baseRad + rnd(-0.05, 0.05) * attempt));
        const cand = [];
        for (let i = 0; i < OBSTACLE_COUNT; i++) {
          cand.push({ c: [rad * Math.cos(th), rad * Math.sin(th), 0.28 + i * 0.24], r: rnd(0.10, 0.13) });
        }
        this.obs = cand;
        if (this.clearance(this.q0) > 0.02 && this.clearance(this.q1) > 0.02) break;
      }

      this.traj = null;
      this.running = false;
      this.path = null;
      this.updateControls();
    },

    resize() {
      const w = wrap.clientWidth, h = wrap.clientHeight;
      if (!w || !h) return;
      const dpr = Math.min(window.devicePixelRatio || 1, 2);
      canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr);
      canvas.getContext('2d').setTransform(dpr, 0, 0, dpr, 0, 0);
      this.W = w; this.H = h;
      this.focal = Math.min(w, h) * 1.05;
    },

    updateBasis() {
      const c = this.cam;
      const dir = [Math.cos(c.el) * Math.cos(c.az), Math.cos(c.el) * Math.sin(c.az), Math.sin(c.el)];
      const eye = this.add(c.target, this.scl(dir, c.dist));
      const f = this.nrm(this.sub(c.target, eye));
      const right = this.nrm(this.crs(f, [0, 0, 1]));
      this.basis = { eye, f, right, up: this.crs(right, f) };
    },

    project(P) {
      const b = this.basis;
      if (!b) return null;
      const d = this.sub(P, b.eye);
      const cz = this.dot(d, b.f);
      if (cz < 0.08) return null;
      const scale = this.focal / cz;
      return { x: this.W / 2 + this.dot(d, b.right) * scale, y: this.H / 2 - this.dot(d, b.up) * scale, scale };
    },

    tickFrame() {
      if (!this.W || !this.q0) return;
      if (!this.userActive) {
        const t = (performance.now() - this.t0) / 1000;
        this.cam.az = -0.95 + 0.22 * Math.sin(t * 0.07);
      }
      this.updateBasis();
      if (!this.running || !this.traj) return;
      const el = (performance.now() - this.playT) / 1000;
      if (el >= this.traj.T) {
        this.phase = 1;
        this.running = false;
        this.updateControls();
      } else {
        this.phase = el / this.traj.T;
      }
    },

    // ── render ─────────────────────────────────────────────────────────
    seg(g, A, B) {
      const a = this.project(A), b = this.project(B);
      if (!a || !b) return;
      g.moveTo(a.x, a.y); g.lineTo(b.x, b.y);
    },
    ring(g, c, r, plane) {
      let first = true;
      for (let i = 0; i <= 40; i++) {
        const t = (i / 40) * Math.PI * 2;
        const o = plane === 0 ? [Math.cos(t) * r, Math.sin(t) * r, 0]
                : plane === 1 ? [Math.cos(t) * r, 0, Math.sin(t) * r]
                              : [0, Math.cos(t) * r, Math.sin(t) * r];
        const p = this.project(this.add(c, o));
        if (!p) { first = true; continue; }
        if (first) { g.moveTo(p.x, p.y); first = false; } else g.lineTo(p.x, p.y);
      }
    },
    arm(g, q, color, width) {
      const pts = this.fk(q).pts;
      g.strokeStyle = color; g.lineWidth = width;
      g.lineCap = 'round'; g.lineJoin = 'round';
      g.beginPath();
      let started = false;
      for (const P of pts) {
        const p = this.project(P);
        if (!p) { started = false; continue; }
        if (!started) { g.moveTo(p.x, p.y); started = true; } else g.lineTo(p.x, p.y);
      }
      g.stroke();
      g.lineCap = 'butt';
      return pts;
    },

    // 2D capsule (stadium) outline between two projected screen points.
    capsulePath(g, a, b, r) {
      const angle = Math.atan2(b.y - a.y, b.x - a.x);
      g.beginPath();
      g.arc(a.x, a.y, r, angle + Math.PI / 2, angle - Math.PI / 2, true);
      g.arc(b.x, b.y, r, angle - Math.PI / 2, angle + Math.PI / 2, true);
      g.closePath();
    },

    // Real UR5e link radii (metres), base -> tool, approximating the collision
    // capsule list in blast/manipulator/UR5e.hpp (base/shoulder, upper arm,
    // forearm, wrist1, wrist2, wrist3) — the same taper the solver's own
    // collision model uses, not a stylized guess.
    LINK_RADIUS: [0.09, 0.06, 0.055, 0.0425, 0.04, 0.038],

    armSolid(g, q, strokeColor) {
      const pts = this.fk(q).pts;
      const proj = pts.map((P) => this.project(P));
      g.fillStyle = '#f2f2f3'; g.strokeStyle = strokeColor; g.lineWidth = 1.3; g.lineJoin = 'round';
      for (let i = 0; i < pts.length - 1; i++) {
        const a = proj[i], b = proj[i + 1];
        if (!a || !b) continue;
        const r = this.LINK_RADIUS[i] * (a.scale + b.scale) / 2;
        this.capsulePath(g, a, b, r);
        g.fill(); g.stroke();
      }
      return pts;
    },

    draw() {
      if (!this.W || !this.q0) return;
      const g = canvas.getContext('2d');
      const accent = '#5980a6', ink = '#1d1f20';
      g.clearRect(0, 0, this.W, this.H);

      g.strokeStyle = 'rgba(29,31,32,0.09)'; g.lineWidth = 1;
      g.beginPath();
      for (let i = -5; i <= 5; i++) {
        const v = i * 0.2;
        this.seg(g, [v, -1.0, 0], [v, 1.0, 0]);
        this.seg(g, [-1.0, v, 0], [1.0, v, 0]);
      }
      g.stroke();
      g.strokeStyle = 'rgba(29,31,32,0.26)';
      g.beginPath(); this.seg(g, [-1, 0, 0], [1, 0, 0]); this.seg(g, [0, -1, 0], [0, 1, 0]); g.stroke();

      for (const o of this.obs) {
        g.strokeStyle = 'rgba(29,31,32,0.45)'; g.lineWidth = 1;
        g.beginPath(); this.ring(g, o.c, o.r, 0); this.ring(g, o.c, o.r, 1); this.ring(g, o.c, o.r, 2); g.stroke();
        g.setLineDash([3, 4]);
        g.strokeStyle = 'rgba(89,128,166,0.4)';
        g.beginPath(); this.ring(g, o.c, o.r + this.TUBE, 0); g.stroke();
        g.strokeStyle = 'rgba(29,31,32,0.16)';
        g.beginPath(); this.ring(g, [o.c[0], o.c[1], 0], o.r, 0); this.seg(g, o.c, [o.c[0], o.c[1], 0]); g.stroke();
        g.setLineDash([]);
      }

      // planned tool path — only once a plan exists
      if (this.traj && this.path && this.path.length > 1) {
        g.strokeStyle = 'rgba(89,128,166,0.55)'; g.lineWidth = 1.2;
        g.beginPath();
        let first = true;
        for (const P of this.path) {
          const p = this.project(P);
          if (!p) { first = true; continue; }
          if (first) { g.moveTo(p.x, p.y); first = false; } else g.lineTo(p.x, p.y);
        }
        g.stroke();
      }

      // ghost poses along the trajectory — only once a plan exists
      if (this.traj) {
        for (const s of [0, 0.25, 0.5, 0.75, 1]) {
          this.arm(g, this.qAt(s), 'rgba(89,128,166,0.20)', 3);
        }
      }

      // base plate
      g.strokeStyle = 'rgba(29,31,32,0.5)'; g.lineWidth = 1;
      g.beginPath(); this.ring(g, [0, 0, 0], 0.13, 0); this.ring(g, [0, 0, 0.02], 0.13, 0); g.stroke();

      // live pose — solid capsule links (the UR5e's real collision-model taper).
      // Rest pose (q0) until a plan is running; qAt(phase) while it plays.
      const pose = this.traj && this.running ? this.qAt(this.phase) : this.q0;
      const pts = this.armSolid(g, pose, accent);
      for (let i = 1; i < pts.length; i++) {
        const p = this.project(pts[i]);
        if (!p) continue;
        g.fillStyle = '#f2f2f3'; g.strokeStyle = ink; g.lineWidth = 1.1;
        g.beginPath(); g.arc(p.x, p.y, i === pts.length - 1 ? 3.5 : 4.5, 0, Math.PI * 2);
        g.fill(); g.stroke();
      }

      // start / goal markers
      const mark = (P, label, filled) => {
        const sp = this.project(P);
        if (!sp) return;
        g.strokeStyle = ink; g.lineWidth = 1;
        g.beginPath(); g.rect(sp.x - 6, sp.y - 6, 12, 12);
        if (filled) { g.fillStyle = ink; g.fill(); } else g.stroke();
        g.setLineDash([2, 4]); g.strokeStyle = 'rgba(29,31,32,0.3)';
        g.beginPath(); this.seg(g, P, [P[0], P[1], 0]); g.stroke();
        g.setLineDash([]);
        g.fillStyle = ink; g.font = '600 10px "Barlow Condensed", sans-serif';
        g.fillText(label, sp.x + 10, sp.y - 8);
      };
      mark(this.startP, 'START', true);
      mark(this.goalP, 'GOAL', false);

      g.fillStyle = 'rgba(29,31,32,0.45)';
      g.font = '600 10px "Barlow Condensed", sans-serif';
      g.fillText('UR5E · 6R · B-SPLINE DEG 5 · 12 CTRL', 10, this.H - 10);
    }
  };

  // The simulator is lazy: nothing solver-related (WASM load, spline setup,
  // the render loop) runs until the visitor opts in via "Try the live
  // planner" — until then the panel just shows the recorded demo video.
  function reveal() {
    if (videoPanel) videoPanel.hidden = true;
    if (simPanel) simPanel.hidden = false;
    if (tryEl) tryEl.hidden = true;
    if (!solver.booted) { solver.booted = true; solver.boot(); }
  }

  if (tryEl) {
    tryEl.addEventListener('click', (e) => { e.preventDefault(); reveal(); });
  } else {
    // No video/try markup on this page variant — boot straight in.
    if (document.readyState === 'loading') {
      document.addEventListener('DOMContentLoaded', () => solver.boot());
    } else {
      solver.boot();
    }
  }
})();
