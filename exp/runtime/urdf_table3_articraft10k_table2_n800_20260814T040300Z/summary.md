# Articraft-10K Table 3 Kinematic Executability

Run classification: **FORMAL**.

Exact Table 2 manifest cohort: N_eval=800 from N_release=9996, seed=20260813; J_eval=2865. Existing package order was preserved without resampling.

| Metric | Result |
|---|---:|
| valid_range | 2865 / 2865 (100.00%) |
| joint_sweep_success | 2865 / 2865 (100.00%) |
| non_degenerate_motion | 2865 / 2865 (100.00%) |
| subtree_consistency | 2855 / 2865 (99.65%) |
| joint_level_pass | 2845 / 2865 (99.30%) |
| fk_roundtrip_error | translation=0.0; rotation_rad=2.9802322387695312e-08; coverage=2865 / 2865 (COMPLETE) |
| strict_kinematic_pass | 795 / 800 (99.38%) |

Category macro average over 222 observed raw categories (222 with at least one declared movable joint):

| Metric | Category macro |
|---|---:|
| valid_range | 100.00% (categories=222) |
| joint_sweep_success | 100.00% (categories=222) |
| non_degenerate_motion | 100.00% (categories=222) |
| subtree_consistency | 99.68% (categories=222) |
| joint_level_pass | 99.19% (categories=222) |
| strict_kinematic_pass | 99.27% (categories=222) |

This evaluates executable declared kinematics only, not semantic joints, collision safety, dynamics, or real-world fidelity.
