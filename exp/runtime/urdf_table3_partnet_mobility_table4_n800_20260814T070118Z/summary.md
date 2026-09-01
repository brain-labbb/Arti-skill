# PartNet-Mobility Table 3 Kinematic Executability

Run classification: **FORMAL**.

Exact Table 4 frozen cohort: N_eval=800 from N_release=2347; J_eval=4078. Existing manifest item order was preserved without resampling.

| Metric | Result |
|---|---:|
| valid_range | 4078 / 4078 (100.00%) |
| joint_sweep_success | 4078 / 4078 (100.00%) |
| non_degenerate_motion | 4069 / 4078 (99.78%) |
| subtree_consistency | 4076 / 4078 (99.95%) |
| joint_level_pass | 4066 / 4078 (99.71%) |
| fk_roundtrip_error | translation=0.0; rotation_rad=2.1073424255447017e-08; coverage=4078 / 4078 (COMPLETE) |
| strict_kinematic_pass | 793 / 800 (99.12%) |

Category macro average over 46 observed raw categories (46 with at least one declared movable joint):

| Metric | Category macro |
|---|---:|
| valid_range | 100.00% (categories=46) |
| joint_sweep_success | 100.00% (categories=46) |
| non_degenerate_motion | 99.77% (categories=46) |
| subtree_consistency | 99.89% (categories=46) |
| joint_level_pass | 99.60% (categories=46) |
| strict_kinematic_pass | 99.44% (categories=46) |

Frozen input inventory records 79 missing collision-mesh references across 13 assets; these assets remain in the frozen denominators.

This evaluates executable declared kinematics only, not semantic joints, collision safety, dynamics, or real-world fidelity.
