# PV-A per-class N=5 Table 3 Kinematic Executability

Run classification: **FORMAL**.

Exact frozen PV-A cohort: N_eval=2655 from N_release=302440; J_eval=14968; K=21.

| Metric | Result |
|---|---:|
| valid_range | 14874 / 14968 (99.37%) |
| joint_sweep_success | 14874 / 14968 (99.37%) |
| non_degenerate_motion | 14874 / 14968 (99.37%) |
| subtree_consistency | 14670 / 14968 (98.01%) |
| joint_level_pass | 14638 / 14968 (97.80%) |
| fk_roundtrip_error | translation=0.0; rotation_rad=2.9802322387695312e-08; coverage=14874 / 14968 (PARTIAL) |
| strict_kinematic_pass | 2622 / 2655 (98.76%) |

Category macro average over 531 observed raw categories (531 with declared movable joints):

| Metric | Category macro |
|---|---:|
| valid_range | 99.91% (categories=531) |
| joint_sweep_success | 99.91% (categories=531) |
| non_degenerate_motion | 99.91% (categories=531) |
| subtree_consistency | 98.99% (categories=531) |
| joint_level_pass | 98.75% (categories=531) |
| strict_kinematic_pass | 98.76% (categories=531) |

This evaluates executable declared kinematics only.
