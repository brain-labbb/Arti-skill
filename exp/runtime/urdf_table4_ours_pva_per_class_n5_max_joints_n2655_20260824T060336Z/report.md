# Ours per-class N=5 (supplementary) Table 4: Collision and Mechanical Clearance

Run classification: **FORMAL**.

Frozen cohort: N_eval=2655 (frozen per-class N=5 cohort order with max-joint fence/Ferris-wheel overrides); rest q=0; single-joint K=21; Sobol R=64 (seed 20260813); penetration threshold 1e-06 m; scale protocol `pybullet_q0_collision_shape_union_aabb_v1`.

| Metric | Result |
|---|---:|
| Rest All-pair CF | 17 / 2655 (0.640%) |
| Rest Non-adjacent CF | 1811 / 2655 (68.211%) |
| Single-joint Sweep CF | 1637 / 2655 (61.657%) |
| Multi-joint Sobol CF | 1639 / 2655 (61.733%) |
| Collision-state Rate | 191232 / 486903 (39.275%) |
| AOR | N/E |
| Max Penetration | 0.712460 (2648 / 2655 measured; PARTIAL) |
| Collision-free Range | 175144 / 314328 (55.720%) |
| Strict Collision Pass | 1613 / 2655 (60.753%) |

Collision-state Rate uses the fail-closed denominator: unexecuted states count as non-free.
AOR is N/E because no stable exact overlap-volume implementation was run; bounding-box overlap is not substituted.
Discrete sweeps do not constitute CCD, joint semantic correctness, or dynamics validity.
