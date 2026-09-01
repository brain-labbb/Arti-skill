# PV-A per-class N=5 Table 2 URDF audit

Run classification: **SMOKE**.

Custom cohort: `CATEGORY_STRATIFIED_N5_WITH_FENCE_FERRIS_MAX_JOINT_OVERRIDES`; N_release=302440, N_eval=5, 531 observed categories.

| Metric | Result |
|---|---:|
| parse_rate | 5 / 5 (100.00%) |
| resource_resolution | 5 / 5 (100.00%) |
| finite_fields | 5 / 5 (100.00%) |
| valid_tree | 5 / 5 (100.00%) |
| valid_joint_spec | 5 / 5 (100.00%) |
| collision_coverage | 5 / 5 (100.00%) |
| inertial_coverage | 5 / 5 (100.00%) |
| inertia_validity | 5 / 5 (100.00%) |
| strict_urdf_pass | 5 / 5 (100.00%) |

Category macro: unweighted mean over 1 raw categories; all frozen assets and failures retained.

| Metric | Category macro |
|---|---:|
| parse_rate | 100.00% |
| resource_resolution | 100.00% |
| finite_fields | 100.00% |
| valid_tree | 100.00% |
| valid_joint_spec | 100.00% |
| collision_coverage | 100.00% |
| inertial_coverage | 100.00% |
| inertia_validity | 100.00% |
| strict_urdf_pass | 100.00% |

Wall time: 4.716898 seconds.
Status counts: `{"completed": 5}`.
