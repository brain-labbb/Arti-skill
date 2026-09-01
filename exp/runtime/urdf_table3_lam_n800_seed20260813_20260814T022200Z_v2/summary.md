# LAM Table 3 Kinematic Executability

Run classification: **FORMAL**.

Frozen full-release random cohort: N_eval=800 from N_release=3217, seed=20260813; J_eval=2395. No quality-tier filtering was applied.

| Metric | Result |
|---|---:|
| valid_range | 2378 / 2395 (99.29%) |
| joint_sweep_success | 2005 / 2395 (83.72%) |
| non_degenerate_motion | 2000 / 2395 (83.51%) |
| subtree_consistency | 2005 / 2395 (83.72%) |
| joint_level_pass | 2000 / 2395 (83.51%) |
| fk_roundtrip_error | translation=0.0; rotation_rad=0.0; coverage=2005 / 2395 (PARTIAL) |
| strict_kinematic_pass | 692 / 800 (86.50%) |

Category macro average over 305 observed categories (302 with at least one declared movable joint):

| Metric | Category macro |
|---|---:|
| valid_range | 95.86% (categories=302) |
| joint_sweep_success | 81.43% (categories=302) |
| non_degenerate_motion | 80.11% (categories=302) |
| subtree_consistency | 81.43% (categories=302) |
| joint_level_pass | 80.11% (categories=302) |
| strict_kinematic_pass | 79.32% (categories=305) |

This evaluation checks executable declared kinematics only. It does not validate semantic joint correctness, collision-free motion, dynamics, or real-world fidelity.
