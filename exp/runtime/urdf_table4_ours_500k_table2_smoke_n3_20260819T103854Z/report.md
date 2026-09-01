# Ours-500K Table 4: Collision and Mechanical Clearance

Run classification: **SMOKE**.

Frozen cohort: N_eval=3 (full acquired roster, Table 2 manifest order); rest q=0; single-joint K=21; Sobol R=64 (seed 20260813); penetration threshold 1e-06 m; scale protocol `pybullet_q0_collision_shape_union_aabb_v1`.

| Metric | Result |
|---|---:|
| Rest All-pair CF | 0 / 3 (0.000%) |
| Rest Non-adjacent CF | 3 / 3 (100.000%) |
| Single-joint Sweep CF | 3 / 3 (100.000%) |
| Multi-joint Sobol CF | 3 / 3 (100.000%) |
| Collision-state Rate | 0 / 300 (0.000%) |
| AOR | N/E |
| Max Penetration | 0.202047 (3 / 3 measured; COMPLETE) |
| Collision-free Range | 105 / 105 (100.000%) |
| Strict Collision Pass | 3 / 3 (100.000%) |

Collision-state Rate uses the fail-closed denominator: unexecuted states count as non-free.
AOR is N/E because no stable exact overlap-volume implementation was run; bounding-box overlap is not substituted.
Discrete sweeps do not constitute CCD, joint semantic correctness, or dynamics validity.
