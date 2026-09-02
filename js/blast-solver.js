// Thin wrapper over the WASM build of BLAST's real solver (native SQP,
// real self- and external-collision constraints) — see
// github.com/HadeonRobotics/blast, branch nikos/emsdk, wasm/bindings.cpp.
//
// Loading is fire-and-forget: if the module 404s, throws, or the browser
// can't run WASM, ready() resolves false and the caller keeps using its own
// solver. Nothing here ever throws out to the caller.

let modulePromise = null;
let heap = null; // allocated once, lazily, and reused across solves

async function loadModule() {
  if (!modulePromise) {
    modulePromise = import('/js/wasm/blast-wasm.js')
      .then((m) => m.default())
      .catch(() => null);
  }
  return modulePromise;
}

const MAX_SPHERES = 8;
const D = 8; // bytes per double

function allocateHeap(Module, nPoints) {
  return {
    Module,
    nPoints,
    taskPtr: Module._malloc(36 * D),
    limitsPtr: Module._malloc(18 * D),
    spheresPtr: Module._malloc(MAX_SPHERES * 4 * D),
    outQPtr: Module._malloc(nPoints * 6 * D),
    outMetaPtr: Module._malloc(5 * D),
  };
}

// Kicks off (or reuses) the module load. Resolves true once solve() is safe
// to call, false if WASM isn't available here.
export async function ready(nPoints) {
  try {
    const Module = await loadModule();
    if (!Module) return false;
    if (!heap || heap.nPoints !== nPoints) heap = allocateHeap(Module, nPoints);
    return true;
  } catch {
    return false;
  }
}

// q0, q1: 6-element joint arrays (start/goal, stop-to-stop).
// obstacles: [{c:[x,y,z], r}, ...] (spheres only, up to MAX_SPHERES).
// limits: {PMAX, VMAX, AMAX} — each a 6-element array.
// opts: {nCtrl, degree, maxTries, tol}.
// Returns {Q, T, ms, evals, viol, ok} or null if WASM isn't ready / the call failed.
export function solve(q0, q1, obstacles, limits, opts) {
  if (!heap) return null;
  try {
    const { Module, taskPtr, limitsPtr, spheresPtr, outQPtr, outMetaPtr, nPoints } = heap;

    const task = new Float64Array(36);
    for (let j = 0; j < 6; j++) {
      task[j * 6 + 0] = q0[j];
      task[j * 6 + 3] = q1[j];
    }
    Module.HEAPF64.set(task, taskPtr / D);

    const flatLimits = new Float64Array([...limits.PMAX, ...limits.VMAX, ...limits.AMAX]);
    Module.HEAPF64.set(flatLimits, limitsPtr / D);

    const nSpheres = Math.min(obstacles.length, MAX_SPHERES);
    if (nSpheres) {
      const flatSpheres = new Float64Array(nSpheres * 4);
      for (let i = 0; i < nSpheres; i++) {
        const o = obstacles[i];
        flatSpheres.set([o.c[0], o.c[1], o.c[2], o.r], i * 4);
      }
      Module.HEAPF64.set(flatSpheres, spheresPtr / D);
    }

    Module.ccall(
      'blast_solve', 'number',
      ['number', 'number', 'number', 'number', 'number', 'number', 'number', 'number', 'number', 'number', 'number', 'number', 'number'],
      [taskPtr, limitsPtr, spheresPtr, nSpheres, 0, 0, opts.nCtrl, nPoints, opts.degree, opts.maxTries, opts.tol, outQPtr, outMetaPtr]
    );

    const meta = Module.HEAPF64.subarray(outMetaPtr / D, outMetaPtr / D + 5);
    const qFlat = Module.HEAPF64.subarray(outQPtr / D, outQPtr / D + nPoints * 6);
    const Q = [];
    for (let p = 0; p < nPoints; p++) Q.push(Array.from(qFlat.subarray(p * 6, p * 6 + 6)));

    return { Q, T: meta[0], ms: meta[1], evals: meta[2], viol: meta[3], ok: meta[4] === 1 };
  } catch {
    return null;
  }
}
