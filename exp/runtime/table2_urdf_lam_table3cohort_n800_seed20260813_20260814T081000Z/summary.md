# LAM released outputs Table 2 URDF audit

Run classification: **FORMAL**.

Frozen cohort: N=800, seed=20260813.

| Metric | Result |
|---|---:|
| parse_rate | 719 / 800 (89.88%) |
| resource_resolution | 777 / 800 (97.12%) |
| finite_fields | 800 / 800 (100.00%) |
| valid_tree | 733 / 800 (91.62%) |
| valid_joint_spec | 759 / 800 (94.88%) |
| collision_coverage | 372 / 800 (46.50%) |
| inertial_coverage | 25 / 800 (3.12%) |
| inertia_validity | 25 / 800 (3.12%) |
| strict_urdf_pass | 24 / 800 (3.00%) |

Category macro average: evaluated over 305 observed category groups using an unweighted mean. This global fixed cohort is not category-balanced or a full release.

| Metric | Category macro |
|---|---:|
| parse_rate | 80.17% |
| resource_resolution | 92.76% |
| finite_fields | 100.00% |
| valid_tree | 86.40% |
| valid_joint_spec | 88.99% |
| collision_coverage | 74.04% |
| inertial_coverage | 0.83% |
| inertia_validity | 0.83% |
| strict_urdf_pass | 0.79% |
