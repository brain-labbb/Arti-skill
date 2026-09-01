# Ours-500K Table 4: Collision and Mechanical Clearance

Run classification: **FORMAL**.

Frozen cohort: N_eval=500 (full acquired roster, Table 2 manifest order); rest q=0; single-joint K=21; Sobol R=64 (seed 20260813); penetration threshold 1e-06 m; scale protocol `pybullet_q0_collision_shape_union_aabb_v1`.

| Metric | Result |
|---|---:|
| Rest All-pair CF | 7 / 500 (1.400%) |
| Rest Non-adjacent CF | 487 / 500 (97.400%) |
| Single-joint Sweep CF | 487 / 500 (97.400%) |
| Multi-joint Sobol CF | 485 / 500 (97.000%) |
| Collision-state Rate | 2447 / 84307 (2.902%) |
| AOR | N/E |
| Max Penetration | 0.297148 (500 / 500 measured; COMPLETE) |
| Collision-free Range | 49577 / 51807 (95.696%) |
| Strict Collision Pass | 485 / 500 (97.000%) |

Collision-state Rate uses the fail-closed denominator: unexecuted states count as non-free.
AOR is N/E because no stable exact overlap-volume implementation was run; bounding-box overlap is not substituted.
Discrete sweeps do not constitute CCD, joint semantic correctness, or dynamics validity.
