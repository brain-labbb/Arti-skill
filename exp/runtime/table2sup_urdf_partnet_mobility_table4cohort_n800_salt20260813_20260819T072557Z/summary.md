# Table 2 supplementary — PartNet-Mobility (frozen Table 4 cohort, N=800)

- Protocol ID: `table2_supplementary_partnet_mobility_table4cohort_n800_v1`
- Run directory: `/mnt/zsn/lyb/arti-skill/exp/runtime/table2sup_urdf_partnet_mobility_table4cohort_n800_salt20260813_20260819T072557Z`
- Cohort source: `/mnt/zsn/lyb/arti-skill/exp/runtime/urdf_table4_partnet_mobility_n800_20260813/frozen_manifest.json` (SHA256 `2ff015ee6bb377ce693126b52dd632a7565a3eaa9f0007e26122a1bb4ab99900`)
- N_eval = 800, J_eval = 4078, observed categories = 46
- Status: completed = 800, error = 0

## Overall micro averages

| Dataset / Outputs | Visual-bearing Collision Coverage ↑ | Joint-limit Portability ↑ | Joint Dynamics Coverage ↑ | Placeholder-mass Incidence ↓ |
|---|---:|---:|---:|---:|
| PartNet-Mobility | 800 / 800 (100.00%) | 0 / 4078 (0.00%) | 0 / 4078 (0.00%) | N/E (placeholder_registry_empty) |

Companion detail:

- Visual-bearing link-micro coverage: 4807 / 4807 (100.00%); link extraction complete on 800 / 800 assets; zero visual-bearing link assets (completed): 0.
- Joint extraction: portability extracted 4078 / intended 4078; dynamics extracted 4078 / intended 4078.
- Placeholder-mass: complete-inertial links 0 / measured dynamic links 5678 (0.00%).

## Category macro (unweighted mean of per-category rates)

- Visual-bearing asset rate mean: 100.0
- Portability joint rate mean: 0.0
- Dynamics joint rate mean: 0.0

