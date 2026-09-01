# PartNet-Mobility Table 2 URDF audit

Run classification: **NON_FORMAL_SMOKE**.

Frozen cohort: N=5, exact frozen items order; SHA256(salt + NUL + numeric dataset_id), ascending by (digest, numeric ID); salt=urdf-sim-ready-table4-partnet-mobility-n800-v1:20260813.

| Metric | Result |
|---|---:|
| parse_rate | 1 / 5 (20.00%) |
| resource_resolution | 5 / 5 (100.00%) |
| finite_fields | 5 / 5 (100.00%) |
| valid_tree | 5 / 5 (100.00%) |
| valid_joint_spec | 5 / 5 (100.00%) |
| collision_coverage | 0 / 5 (0.00%) |
| inertial_coverage | 0 / 5 (0.00%) |
| inertia_validity | 0 / 5 (0.00%) |
| strict_urdf_pass | 0 / 5 (0.00%) |

Category macro average: evaluated over 4 observed category groups using an unweighted mean. This global fixed cohort is not category-balanced or a full release.

| Metric | Category macro |
|---|---:|
| parse_rate | 25.00% |
| resource_resolution | 100.00% |
| finite_fields | 100.00% |
| valid_tree | 100.00% |
| valid_joint_spec | 100.00% |
| collision_coverage | 0.00% |
| inertial_coverage | 0.00% |
| inertia_validity | 0.00% |
| strict_urdf_pass | 0.00% |
