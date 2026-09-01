# Articraft-10K Table 3 Kinematic Executability

Run classification: **NON_FORMAL_SMOKE**.

Exact Table 2 manifest cohort: N_eval=3 from N_release=9996, seed=20260813; J_eval=12. Existing package order was preserved without resampling.

| Metric | Result |
|---|---:|
| valid_range | 12 / 12 (100.00%) |
| joint_sweep_success | 12 / 12 (100.00%) |
| non_degenerate_motion | 12 / 12 (100.00%) |
| subtree_consistency | 12 / 12 (100.00%) |
| joint_level_pass | 12 / 12 (100.00%) |
| fk_roundtrip_error | translation=0.0; rotation_rad=0.0; coverage=12 / 12 (COMPLETE) |
| strict_kinematic_pass | 3 / 3 (100.00%) |

Category macro average over 3 observed raw categories (3 with at least one declared movable joint):

| Metric | Category macro |
|---|---:|
| valid_range | 100.00% (categories=3) |
| joint_sweep_success | 100.00% (categories=3) |
| non_degenerate_motion | 100.00% (categories=3) |
| subtree_consistency | 100.00% (categories=3) |
| joint_level_pass | 100.00% (categories=3) |
| strict_kinematic_pass | 100.00% (categories=3) |

This evaluates executable declared kinematics only, not semantic joints, collision safety, dynamics, or real-world fidelity.
