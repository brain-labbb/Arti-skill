# Table 2 Supplementary Full-Release Results

Collision, Joint, and Inertial Diagnostics for eight full-release comparison cohorts plus the unchanged Ours rows.

- Combined receipt: `/mnt/zsn/lyb/arti-skill/exp/runtime/table2sup_full_release_20260826/full_release_receipt.json`
- Source protocol: [URDF-Sim-Ready-Automatic-Evaluation.md](../../URDF-Sim-Ready-Automatic-Evaluation.md)
- This renderer is read-only and does not run an evaluator.

| Dataset / Outputs | N_eval | J_eval | Visual-bearing Collision Coverage | Joint-limit Portability | Joint Dynamics Coverage | Placeholder-mass Incidence | Status | Evidence |
|---|---:|---:|---:|---:|---:|---:|---|---|
| Ours-500K | 500 | 2,467 | 500 / 500 (100.00%) | 2,467 / 2,467 (100.00%) | 266 / 2,467 (10.78%) | N/E | source | source |
| Ours per-class N=5 (supplementary) | 2,655 | 14,968 | 2,655 / 2,655 (100.00%) | 14,943 / 14,968 (99.83%) | 1,137 / 14,968 (7.60%) | N/E | source | source |
| Articraft-10K | 9,996 | 37,144 | 3,059 / 9,996 (30.60%) | 37,143 / 37,144 (100.00%) | 829 / 37,144 (2.23%) | N/E | complete | [`summary`](articraft/summary.json) |
| LAM released outputs | 3,217 | 10,381 | 1,479 / 3,217 (45.97%) | 8,480 / 10,381 (81.69%) | 0 / 10,381 (0.00%) | N/E | complete | [`summary`](lam/summary.json) |
| Artiverse | 3,544 | 16,332 | 3,542 / 3,544 (99.94%) | 15,555 / 16,332 (95.24%) | 0 / 16,332 (0.00%) | N/E | complete | [`summary`](artiverse/summary.json) |
| PartNet-Mobility | 2,347 | 11,971 | 2,347 / 2,347 (100.00%) | 0 / 11,971 (0.00%) | 0 / 11,971 (0.00%) | N/E | complete | [`summary`](partnet/summary.json) |
| PhysX-Mobility | 2,024 | 9,883 | 0 / 2,024 (0.00%) | 9,883 / 9,883 (100.00%) | 0 / 9,883 (0.00%) | N/E | complete | [`summary`](physx/summary.json) |
| SketchMobility | 4,956 | 11,009 | 2,779 / 4,956 (56.07%) | 6,452 / 11,009 (58.61%) | 2,303 / 11,009 (20.92%) | N/E | complete | [`summary`](sketch/summary.json) |
| Infinite Mobility | 720 | 4,723 | 0 / 720 (0.00%) | 4,687 / 4,723 (99.24%) | 0 / 4,723 (0.00%) | N/E | complete | [`summary`](infinite/summary.json) |
| Infinigen-Sim | 8,226 | 31,975 | 8,226 / 8,226 (100.00%) | 0 / 31,975 (0.00%) | 31,975 / 31,975 (100.00%) | N/E | complete | [`summary`](infinigen/summary.json) |

N/E denotes not estimable under the frozen placeholder registry. Asset-level errors and incomplete records remain in the published denominators.
