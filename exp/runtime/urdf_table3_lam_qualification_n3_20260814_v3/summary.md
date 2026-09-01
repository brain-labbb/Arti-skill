# LAM Table 3 Kinematic Executability

Run classification: **NON_FORMAL_SMOKE**.

Frozen full-release random cohort: N_eval=3 from N_release=3217, seed=20260813; J_eval=7. No quality-tier filtering was applied.

| Metric | Result |
|---|---:|
| valid_range | 7 / 7 (100.00%) |
| joint_sweep_success | 7 / 7 (100.00%) |
| non_degenerate_motion | 7 / 7 (100.00%) |
| subtree_consistency | 7 / 7 (100.00%) |
| joint_level_pass | 7 / 7 (100.00%) |
| fk_roundtrip_error | translation=0.0; rotation_rad=0.0; coverage=7 / 7 (COMPLETE) |
| strict_kinematic_pass | 3 / 3 (100.00%) |

Category macro average over 3 observed categories (3 with at least one declared movable joint):

| Metric | Category macro |
|---|---:|
| valid_range | 100.00% (categories=3) |
| joint_sweep_success | 100.00% (categories=3) |
| non_degenerate_motion | 100.00% (categories=3) |
| subtree_consistency | 100.00% (categories=3) |
| joint_level_pass | 100.00% (categories=3) |
| strict_kinematic_pass | 100.00% (categories=3) |

This evaluation checks executable declared kinematics only. It does not validate semantic joint correctness, collision-free motion, dynamics, or real-world fidelity.
