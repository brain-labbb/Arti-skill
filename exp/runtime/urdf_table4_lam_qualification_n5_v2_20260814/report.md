# LAM released outputs qualification N=5: URDF Sim-Ready Table 4

Status: **COMPLETE_WITH_RETAINED_FAILURES**

This is the exact frozen N=800 cohort identified by the supplied Table 3 asset_records.jsonl, reconstructed in selection_rank order from the 3,217-asset LAM release universe. It is not a full-release or category-balanced result.

Observed authoritative categories: 5. Unweighted category macro: collision_free_range=20.000%, collision_state_rate=80.000%, multi_joint_sobol_cf=20.000%, rest_all_pair_cf=0.000%, rest_non_adjacent_cf=20.000%, single_joint_sweep_cf=20.000%, strict_collision_pass=20.000%.

| Metric | Result |
|---|---:|
| Rest All-pair CF | 0/5 (0.000%) |
| Rest Non-adjacent CF | 1/5 (20.000%) |
| Single-joint Sweep CF | 1/5 (20.000%) |
| Multi-joint Sobol CF | 1/5 (20.000%) |
| Collision-state Rate | 449/598 (75.084%) |
| AOR | N/E |
| Max Penetration | 0.14281719699129822 (fully measured 3/5; observed 3/5; PARTIAL) |
| Collision-free Range | 84/273 (30.769%) |
| Strict Collision Pass | 1/5 (20.000%) |

Collision-state Rate is fail-closed. Unexecuted configurations remain in the frozen denominator and count as non-free.

AOR is N/E because no stable exact overlap-volume calculation was run. Sweeps are discrete; no CCD, semantic-joint, or physical-dynamics claim is made.
