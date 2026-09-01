# Artiverse Table 2 URDF audit

Run classification: **FORMAL**.

Frozen cohort: N=800, seed=20260813.

| Metric | Result |
|---|---:|
| parse_rate | 797 / 800 (99.62%) |
| resource_resolution | 800 / 800 (100.00%) |
| finite_fields | 800 / 800 (100.00%) |
| valid_tree | 797 / 800 (99.62%) |
| valid_joint_spec | 800 / 800 (100.00%) |
| collision_coverage | 777 / 800 (97.12%) |
| inertial_coverage | 800 / 800 (100.00%) |
| inertia_validity | 800 / 800 (100.00%) |
| strict_urdf_pass | 774 / 800 (96.75%) |

Category macro average: evaluated over 67 observed raw-category groups using an unweighted mean. This global fixed cohort is not category-balanced or a full release.

| Metric | Category macro |
|---|---:|
| parse_rate | 99.79% |
| resource_resolution | 100.00% |
| finite_fields | 100.00% |
| valid_tree | 99.79% |
| valid_joint_spec | 100.00% |
| collision_coverage | 94.30% |
| inertial_coverage | 100.00% |
| inertia_validity | 100.00% |
| strict_urdf_pass | 94.09% |
