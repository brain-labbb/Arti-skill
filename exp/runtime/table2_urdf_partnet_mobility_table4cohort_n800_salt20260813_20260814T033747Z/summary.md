# PartNet-Mobility Table 2 URDF audit

Run classification: **FORMAL**.

Frozen cohort: N=800, exact frozen items order; SHA256(salt + NUL + numeric dataset_id), ascending by (digest, numeric ID); salt=urdf-sim-ready-table4-partnet-mobility-n800-v1:20260813.

| Metric | Result |
|---|---:|
| parse_rate | 95 / 800 (11.88%) |
| resource_resolution | 787 / 800 (98.38%) |
| finite_fields | 800 / 800 (100.00%) |
| valid_tree | 800 / 800 (100.00%) |
| valid_joint_spec | 800 / 800 (100.00%) |
| collision_coverage | 0 / 800 (0.00%) |
| inertial_coverage | 0 / 800 (0.00%) |
| inertia_validity | 0 / 800 (0.00%) |
| strict_urdf_pass | 0 / 800 (0.00%) |

Category macro average: evaluated over 46 observed category groups using an unweighted mean. This global fixed cohort is not category-balanced or a full release.

| Metric | Category macro |
|---|---:|
| parse_rate | 12.00% |
| resource_resolution | 98.15% |
| finite_fields | 100.00% |
| valid_tree | 100.00% |
| valid_joint_spec | 100.00% |
| collision_coverage | 0.00% |
| inertial_coverage | 0.00% |
| inertia_validity | 0.00% |
| strict_urdf_pass | 0.00% |
