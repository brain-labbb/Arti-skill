# Infinite Mobility Table 4: Collision and Mechanical Clearance

Run classification: **FORMAL**; status: **BLOCKED**.

Frozen cohort: N_eval=720, J_eval=4723; rest q=0; single-joint K=21; Sobol R=64 (seed 20260813); penetration threshold 1e-06 m.

| Metric | Result |
|---|---:|
| Rest All-pair CF | N/E |
| Rest Non-adjacent CF | N/E |
| Single-joint Sweep CF | N/E |
| Multi-joint Sobol CF | N/E |
| Collision-state Rate | N/E |
| AOR | N/E |
| Max Penetration | N/E |
| Collision-free Range | N/E |
| Strict Collision Pass | N/E |

The preflight found 0 native URDF collision elements across 720 assets. Collision queries were not executed: `BLOCKED_NATIVE_COLLISION_GEOMETRY_ABSENT`. Treating empty contact queries as passes would be a vacuous result, not mechanical-clearance evidence.

Intent-to-evaluate state plan: rest 720, single 99183, Sobol 42560, total 142463. No state receipt is emitted because the collision oracle is inapplicable. If a future release adds native collision geometry, the full state schedule must be rerun under a new protocol snapshot.

AOR and max penetration remain N/E; no visual or bounding-box fallback was used.
