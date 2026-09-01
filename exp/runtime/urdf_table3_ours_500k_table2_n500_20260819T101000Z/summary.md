# Ours-500K Table 3 Kinematic Executability

Run classification: **FORMAL**.

Exact Table 2 manifest cohort: N_eval=500 from N_release=500; J_eval=2467. Existing asset order was preserved without resampling.

| Metric | Result |
|---|---:|
| valid_range | 2467 / 2467 (100.00%) |
| joint_sweep_success | 2467 / 2467 (100.00%) |
| non_degenerate_motion | 2467 / 2467 (100.00%) |
| subtree_consistency | 2467 / 2467 (100.00%) |
| joint_level_pass | 2467 / 2467 (100.00%) |
| fk_roundtrip_error | translation=0.0; rotation_rad=0.0; coverage=2467 / 2467 (COMPLETE) |
| strict_kinematic_pass | 500 / 500 (100.00%) |

Category macro average over 12 observed raw categories (12 with at least one declared movable joint):

| Metric | Category macro |
|---|---:|
| valid_range | 100.00% (categories=12) |
| joint_sweep_success | 100.00% (categories=12) |
| non_degenerate_motion | 100.00% (categories=12) |
| subtree_consistency | 100.00% (categories=12) |
| joint_level_pass | 100.00% (categories=12) |
| strict_kinematic_pass | 100.00% (categories=12) |

This evaluates executable declared kinematics only, not semantic joints, collision safety, dynamics, or real-world fidelity.
