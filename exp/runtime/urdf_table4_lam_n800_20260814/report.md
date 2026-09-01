# LAM released outputs fixed Table 3 N=800 cohort: URDF Sim-Ready Table 4

Status: **COMPLETE_WITH_RETAINED_FAILURES**

This is the exact frozen N=800 cohort identified by the supplied Table 3 asset_records.jsonl, reconstructed in selection_rank order from the 3,217-asset LAM release universe. It is not a full-release or category-balanced result.

Observed authoritative categories: 305. Unweighted category macro: collision_free_range=25.076%, collision_state_rate=74.596%, multi_joint_sobol_cf=21.512%, rest_all_pair_cf=0.000%, rest_non_adjacent_cf=24.986%, single_joint_sweep_cf=22.761%, strict_collision_pass=21.512%.

| Metric | Result |
|---|---:|
| Rest All-pair CF | 0/800 (0.000%) |
| Rest Non-adjacent CF | 113/800 (14.125%) |
| Single-joint Sweep CF | 101/800 (12.625%) |
| Multi-joint Sobol CF | 91/800 (11.375%) |
| Collision-state Rate | 88097/100631 (87.545%) |
| AOR | N/E |
| Max Penetration | 0.782640066199297 (fully measured 321/800; observed 321/800; PARTIAL) |
| Collision-free Range | 4832/50295 (9.607%) |
| Strict Collision Pass | 91/800 (11.375%) |

Collision-state Rate is fail-closed. Unexecuted configurations remain in the frozen denominator and count as non-free.

AOR is N/E because no stable exact overlap-volume calculation was run. Sweeps are discrete; no CCD, semantic-joint, or physical-dynamics claim is made.
