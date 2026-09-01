# Artiverse Table 3 Kinematic Executability

Run classification: **FORMAL**.

Exact Table 1 manifest cohort: N_eval=800 from N_release=3544, seed=20260813; J_eval=3875. Existing manifest order was preserved without resampling.

| Metric | Result |
|---|---:|
| valid_range | 3875 / 3875 (100.00%) |
| joint_sweep_success | 3854 / 3875 (99.46%) |
| non_degenerate_motion | 3782 / 3875 (97.60%) |
| subtree_consistency | 3854 / 3875 (99.46%) |
| joint_level_pass | 3782 / 3875 (97.60%) |
| fk_roundtrip_error | translation=0.0; rotation_rad=0.0; coverage=3854 / 3875 (PARTIAL) |
| strict_kinematic_pass | 762 / 800 (95.25%) |

Category macro average over 67 observed raw categories (67 with at least one declared movable joint):

| Metric | Category macro |
|---|---:|
| valid_range | 100.00% (categories=67) |
| joint_sweep_success | 99.85% (categories=67) |
| non_degenerate_motion | 95.66% (categories=67) |
| subtree_consistency | 99.85% (categories=67) |
| joint_level_pass | 95.66% (categories=67) |
| strict_kinematic_pass | 91.09% (categories=67) |

This evaluates executable declared kinematics only, not semantic joints, collision safety, dynamics, or real-world fidelity.
