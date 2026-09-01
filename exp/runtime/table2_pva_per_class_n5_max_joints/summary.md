# PV-A per-class N=5 Table 2 URDF audit

Run classification: **CUSTOM_COHORT_EVIDENCE**.

Custom cohort: `CATEGORY_STRATIFIED_N5_WITH_FENCE_FERRIS_MAX_JOINT_OVERRIDES`; N_release=302440, N_eval=2655, 531 observed categories.

| Metric | Result |
|---|---:|
| parse_rate | 2655 / 2655 (100.00%) |
| resource_resolution | 2655 / 2655 (100.00%) |
| finite_fields | 2655 / 2655 (100.00%) |
| valid_tree | 2655 / 2655 (100.00%) |
| valid_joint_spec | 2650 / 2655 (99.81%) |
| collision_coverage | 2629 / 2655 (99.02%) |
| inertial_coverage | 975 / 2655 (36.72%) |
| inertia_validity | 975 / 2655 (36.72%) |
| strict_urdf_pass | 961 / 2655 (36.20%) |

Category macro: unweighted mean over 531 raw categories; all frozen assets and failures retained.

| Metric | Category macro |
|---|---:|
| parse_rate | 100.00% |
| resource_resolution | 100.00% |
| finite_fields | 100.00% |
| valid_tree | 100.00% |
| valid_joint_spec | 99.81% |
| collision_coverage | 99.02% |
| inertial_coverage | 36.72% |
| inertia_validity | 36.72% |
| strict_urdf_pass | 36.20% |

Wall time: 616.308474 seconds.
Status counts: `{"completed": 2655}`.
