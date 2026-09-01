# Ours per-class N=5 (supplementary) Table 4: Collision and Mechanical Clearance

Run classification: **SMOKE**.

Frozen cohort: N_eval=3 (frozen per-class N=5 cohort order with max-joint fence/Ferris-wheel overrides); rest q=0; single-joint K=21; Sobol R=64 (seed 20260813); penetration threshold 1e-06 m; scale protocol `pybullet_q0_collision_shape_union_aabb_v1`.

| Metric | Result |
|---|---:|
| Rest All-pair CF | 0 / 3 (0.000%) |
| Rest Non-adjacent CF | 1 / 3 (33.333%) |
| Single-joint Sweep CF | 1 / 3 (33.333%) |
| Multi-joint Sobol CF | 1 / 3 (33.333%) |
| Collision-state Rate | 134 / 300 (44.667%) |
| AOR | N/E |
| Max Penetration | 0.265094 (3 / 3 measured; COMPLETE) |
| Collision-free Range | 41 / 105 (39.048%) |
| Strict Collision Pass | 1 / 3 (33.333%) |

Collision-state Rate uses the fail-closed denominator: unexecuted states count as non-free.
AOR is N/E because no stable exact overlap-volume implementation was run; bounding-box overlap is not substituted.
Discrete sweeps do not constitute CCD, joint semantic correctness, or dynamics validity.
