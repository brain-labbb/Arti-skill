# PartNet-Mobility Table 3 Kinematic Executability

Run classification: **NON_FORMAL_SMOKE**.

Exact Table 4 frozen cohort: N_eval=1 from N_release=2347; J_eval=2. Existing manifest item order was preserved without resampling.

| Metric | Result |
|---|---:|
| valid_range | 2 / 2 (100.00%) |
| joint_sweep_success | 2 / 2 (100.00%) |
| non_degenerate_motion | 2 / 2 (100.00%) |
| subtree_consistency | 2 / 2 (100.00%) |
| joint_level_pass | 2 / 2 (100.00%) |
| fk_roundtrip_error | translation=0.0; rotation_rad=0.0; coverage=2 / 2 (COMPLETE) |
| strict_kinematic_pass | 1 / 1 (100.00%) |

Category macro average over 1 observed raw categories (1 with at least one declared movable joint):

| Metric | Category macro |
|---|---:|
| valid_range | 100.00% (categories=1) |
| joint_sweep_success | 100.00% (categories=1) |
| non_degenerate_motion | 100.00% (categories=1) |
| subtree_consistency | 100.00% (categories=1) |
| joint_level_pass | 100.00% (categories=1) |
| strict_kinematic_pass | 100.00% (categories=1) |

Frozen input inventory records 79 missing collision-mesh references across 13 assets; these assets remain in the frozen denominators.

This evaluates executable declared kinematics only, not semantic joints, collision safety, dynamics, or real-world fidelity.
