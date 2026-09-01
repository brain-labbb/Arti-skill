# Table 1/2/3 Full-Release Results

This handoff contains the verified primary Table 1, Table 2, and Table 3 rows for the complete local comparison releases. Ours/Brain rows are unchanged and historical N=800 evidence remains separate.

- Source protocol: [URDF-Sim-Ready-Automatic-Evaluation.md](../../URDF-Sim-Ready-Automatic-Evaluation.md)
- Compact receipt: [full_release_receipt.md](full_release_receipt.md)
- Read-only acceptance report: [automation_check_auto.json](automation_check_auto.json)

## Table 1. Dataset Scale and Structural Diversity

> **Full-release update (2026-08-25).** The eight comparison rows below use the complete local rosters (`N_eval` and `J_eval` are shown by the receipt-bound denominators), not the historical N=800 samples. Ours/Brain rows are intentionally unchanged. Paragraphs or receipt paths containing `n800` elsewhere in this document are retained as historical audit evidence and do not override these primary-table values.
>
> **PV-A formal full-release backfill (2026-08-27).** The PV-A rows in Tables 1, 2, Table 2 supplementary, Table 3, and Table 4 use the complete frozen release: `N_eval=302,440`, `J_eval=1,453,516`, and 531 / 531 generator classes. All manifest assets remain in the denominators; terminal worker status is 301,986 completed, 448 recovered, 5 parent errors, and 1 timeout. The run is classified `FORMAL_FULL_RELEASE`, and the read-only automation check reports `all_pass=true`, database integrity `ok`, and 5 / 5 tables checked. Evidence: [final results](runtime/pva_table1234_full_release_20260826/evaluation/pva_table1234_full_release_results.md), [full-release receipt](runtime/pva_table1234_full_release_20260826/evaluation/full_release_receipt.json), and [automation check](runtime/pva_table1234_full_release_20260826/evaluation/automation_check.json).

| Dataset / Outputs | N_release | N_eval | Observed Labels (release / eval) | Links/Asset | Movable Joints/Asset | Multi-joint Assets (%) ↑ | Pooled Raw-tree Support (%) (descriptive) | Exact Duplicate Rate (%) ↓ |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Ours-500K | 500 | 500 | 12 / 12 | 6.25 / 5 / 13 | 4.93 / 4 / 12 | 92.20 | 13.40 (n=500) | 0.00 (n=500) |
| Ours per-class N=5 (supplementary) | 302,440 | 2,655 | 531 / 531 | 7.31 / 5 / 13 | 5.64 / 3 / 11 | 82.86 | 34.99 (n=2,655) | 0.00 (n=2,655) |
| Ours / PV-A | 302,440 | 302,440 | 531 / 531 | 6.56 / 5 / 12 (n=302,435) | 4.81 / 3 / 10 (n=302,435) | 84.55% (n=302,440) | 5,356 / 302,435 = 1.77% (pooled diagnostic only) | 0.42% (n=302,435) |
| Articraft-10K | 9,996 | 9,996 | 240 / 240 | 5.11 / 4 / 8 (n=9,996) | 3.72 / 3 / 6 (n=9,996) | 82.86% (n=9,996) | 18.50% (n=9,996) | 0.00% (n=9,996) |
| LAM released outputs | 3,217 | 3,217 | 787 / 787 | 6.09 / 4 / 11 (n=3,005) | 3.00 / 2 / 5 (n=3,005) | 53.59% (n=3,217) | 32.72% (n=2,968) | 2.56% (n=2,924) |
| Artiverse | 3,544 | 3,544 | 84 / 84 | 8.22 / 5 / 16 (n=3,526) | 4.60 / 2.5 / 7 (n=3,526) | 77.65% (n=3,544) | 22.69% (n=3,526) | 0.00% (n=3,526) |
| PartNet-Mobility | 2,347 | 2,347 | 46 / 46 | 7.10 / 4 / 11 (n=2,347) | 5.10 / 2 / 9 (n=2,347) | 59.86% (n=2,347) | 10.40% (n=2,347) | 0.00% (n=2,314) |
| PhysX-Mobility | 2,024 | 2,024 | 132 / 132 | 12.85 / 6 / 19 (n=2,024) | 4.89 / 2 / 7 (n=2,024) | 55.09% (n=2,024) | 15.46% (n=2,024) | 0.00% (n=2,024) |
| SketchMobility | 4,956 | 4,956 | 70 / 70 | 3.38 / 3 / 5 (n=4,949) | 2.22 / 1 / 4 (n=4,949) | 48.31% (n=4,956) | 3.13% (n=4,949) | 0.02% (n=4,901) |
| Infinite Mobility (supplementary generated cohort) | 720 | 720 | 20 / 20 | 15.04 / 8 / 41 (n=720) | 6.56 / 3 / 16 (n=720) | 76.39% (n=720) | 21.81% (n=720) | 0.00% (n=720) |
| Infinigen-Sim | 8,226 | 8,226 | 17 / 17 | 5.89 / 5 / 11 (n=8,226) | 3.89 / 3 / 9 (n=8,226) | 73.86% (n=8,226) | 0.89% (n=8,226) | 0.00% (n=6,726) |

| Ours / PV-A category macro | Multi-joint Assets ↑ | Raw-tree Signature Support (descriptive) | Exact Duplicate Rate ↓ |
|---|---:|---:|---:|
| 531 generator classes, unweighted mean | 82.80% | 9.55% | 0.26% |

> **Topology metric correction (2026-08-27).** The former `1.77% ↑` presentation was not a valid cross-dataset diversity score. It is the legacy pooled ratio `5,356 distinct rooted-joint-tree-v1 hashes / 302,435 valid trees`; `unique/N` necessarily falls as the release grows and it also merges the same raw tree across unrelated categories. It is retained above only as a support diagnostic, without an upward arrow. The full-release within-category views are `7,617 / 302,435 = 2.52%` when `(generator class, hash)` is the identity and `9.55%` when the per-class rates are macro-averaged. Neither controls the unequal per-class sample counts (7--3,200).
>
> Exact category-stratified rarefaction of the same frozen full-release records at `k=5` gives an expected within-class signature rate of `62.92%` over 531 / 531 classes. The frozen per-class N=5 cohort realizes `1,675 / 2,655 = 63.09%` within classes, while its legacy cross-class pooled value is `929 / 2,655 = 34.99%`. The read-only audit at `exp/scripts/audit_pva_table1_topologies.py` streams all 302,440 frozen records, verifies the summary self-hash and category aggregates, and reproduces these views. These `k=5` values are supplementary diagnostics, not the preregistered cross-method headline: the N=5 cohort includes two max-joint overrides, and the shared-category `n=20`, five-resample protocol in `div_v2.md` remains to be run for every method.

| PV-A topology view | Pooled raw-tree support | Category-conditioned support | Category macro / fixed-budget view |
|---|---:|---:|---:|
| Full release (302,435 valid) | 5,356 / 302,435 = 1.77% | 7,617 / 302,435 = 2.52% | 9.55% macro; 62.92% exact rarefaction at k=5 |
| Frozen per-class N=5 supplementary | 929 / 2,655 = 34.99% | 1,675 / 2,655 = 63.09% | 63.09% macro (equal five-per-class denominator) |

---

## Table 2. URDF Validity and Structural Integrity

> **Full-release update (2026-08-27).** The comparison rows use the same complete rosters as Table 1. Parser/resource failures and retained terminal failures remain in the denominators; no failed asset was removed or replaced. Ours Brain/per-class rows are unchanged; the PV-A row uses the formal full-release receipt described in Table 1.

| Dataset / Outputs | Parse Rate ↑ | Resource Resolution ↑ | Finite Fields ↑ | Valid Tree ↑ | Valid Joint Spec. ↑ | Collision Coverage ↑ | Strict URDF Pass ↑ |
|---|---:|---:|---:|---:|---:|---:|---:|
| Ours-500K | 500 / 500 (100.00%) | 500 / 500 (100.00%) | 500 / 500 (100.00%) | 500 / 500 (100.00%) | 500 / 500 (100.00%) | 500 / 500 (100.00%) | 4 / 500 (0.80%) |
| Ours per-class N=5 (supplementary) | 2,655 / 2,655 (100.00%) | 2,655 / 2,655 (100.00%) | 2,655 / 2,655 (100.00%) | 2,655 / 2,655 (100.00%) | 2,650 / 2,655 (99.81%) | 2,629 / 2,655 (99.02%) | 961 / 2,655 (36.20%) |
| Ours / PV-A | 302,435 / 302,440 (100.00%) | 302,435 / 302,440 (100.00%) | 302,435 / 302,440 (100.00%) | 302,435 / 302,440 (100.00%) | 302,221 / 302,440 (99.93%) | 300,390 / 302,440 (99.32%) | 103,399 / 302,440 (34.19%) |
| Articraft-10K | 9,996 / 9,996 (100.00%) | 9,996 / 9,996 (100.00%) | 9,996 / 9,996 (100.00%) | 9,996 / 9,996 (100.00%) | 9,996 / 9,996 (100.00%) | 3,049 / 9,996 (30.50%) | 127 / 9,996 (1.27%) |
| LAM released outputs | 2,928 / 3,217 (91.02%) | 3,125 / 3,217 (97.14%) | 3,217 / 3,217 (100.00%) | 2,968 / 3,217 (92.26%) | 3,078 / 3,217 (95.68%) | 1,479 / 3,217 (45.97%) | 95 / 3,217 (2.95%) |
| Artiverse | 3,526 / 3,544 (99.49%) | 3,543 / 3,544 (99.97%) | 3,543 / 3,544 (99.97%) | 3,526 / 3,544 (99.49%) | 3,540 / 3,544 (99.89%) | 3,458 / 3,544 (97.57%) | 3,443 / 3,544 (97.15%) |
| PartNet-Mobility | 316 / 2,347 (13.46%) | 2,314 / 2,347 (98.59%) | 2,343 / 2,347 (99.83%) | 2,347 / 2,347 (100.00%) | 2,342 / 2,347 (99.79%) | 0 / 2,347 (0.00%) | 0 / 2,347 (0.00%) |
| PhysX-Mobility | 2,024 / 2,024 (100.00%) | 2,024 / 2,024 (100.00%) | 2,024 / 2,024 (100.00%) | 2,024 / 2,024 (100.00%) | 2,024 / 2,024 (100.00%) | 0 / 2,024 (0.00%) | 0 / 2,024 (0.00%) |
| SketchMobility | 4,227 / 4,956 (85.29%) | 4,908 / 4,956 (99.03%) | 4,956 / 4,956 (100.00%) | 4,949 / 4,956 (99.86%) | 4,954 / 4,956 (99.96%) | 1,913 / 4,956 (38.60%) | 1 / 4,956 (0.02%) |
| Infinite Mobility (supplementary generated cohort) | 720 / 720 (100.00%) | 720 / 720 (100.00%) | 720 / 720 (100.00%) | 720 / 720 (100.00%) | 445 / 720 (61.81%) | 0 / 720 (0.00%) | 0 / 720 (0.00%) |
| Infinigen-Sim | 94 / 8,226 (1.14%) | 6,722 / 8,226 (81.72%) | 8,226 / 8,226 (100.00%) | 8,226 / 8,226 (100.00%) | 8,226 / 8,226 (100.00%) | 0 / 8,226 (0.00%) | 0 / 8,226 (0.00%) |

| Ours / PV-A category macro | Parse Rate ↑ | Resource Resolution ↑ | Finite Fields ↑ | Valid Tree ↑ | Valid Joint Spec. ↑ | Collision Coverage ↑ | Strict URDF Pass ↑ |
|---|---:|---:|---:|---:|---:|---:|---:|
| 531 generator classes, unweighted mean | 99.99% | 99.99% | 99.99% | 99.99% | 99.84% | 98.89% | 36.23% |

---

## Table 3. Kinematic Executability

> **Full-release update (2026-08-27).** The eight comparison rows below and the PV-A row use complete-release `N_eval` and `J_eval` denominators. Unsupported joints, parser/runtime errors, and partial FK coverage remain fail-closed; Ours Brain/per-class rows are unchanged. The PV-A row uses the formal full-release receipt documented in Table 1.

| Dataset / Outputs | Valid Range ↑ | Joint Sweep Success ↑ | Non-degenerate Motion ↑ | Subtree Consistency ↑ | FK Round-trip Error ↓ | Joint-level Pass ↑ | Strict Kinematic Pass ↑ |
|---|---:|---:|---:|---:|---:|---:|---:|
| Ours-500K | 2,467 / 2,467 (100.00%) | 2,467 / 2,467 (100.00%) | 2,467 / 2,467 (100.00%) | 2,467 / 2,467 (100.00%) | 0.000000 normalized translation / 0.000000 rad rotation (2,467 / 2,467 measured; COMPLETE) | 2,467 / 2,467 (100.00%) | 500 / 500 (100.00%) |
| Ours per-class N=5 (supplementary) | 14,874 / 14,968 (99.37%) | 14,874 / 14,968 (99.37%) | 14,874 / 14,968 (99.37%) | 14,670 / 14,968 (98.01%) | 0.000000 normalized translation / 2.980232e-8 rad rotation (14,874 / 14,968 measured; PARTIAL) | 14,638 / 14,968 (97.80%) | 2,622 / 2,655 (98.76%) |
| Ours / PV-A | 1,452,330 / 1,453,516 (99.92%) | 1,452,330 / 1,453,516 (99.92%) | 1,452,330 / 1,453,516 (99.92%) | 1,429,688 / 1,453,516 (98.36%) | 0.000000 normalized translation / 9.424322e-08 rad rotation (1,452,330 / 1,453,516 measured; PARTIAL) | 1,426,812 / 1,453,516 (98.16%) | 299,924 / 302,440 (99.17%) |
| Articraft-10K | 37,143 / 37,144 (100.00%) | 37,137 / 37,144 (99.98%) | 37,137 / 37,144 (99.98%) | 36,897 / 37,144 (99.34%) | 0.000000 normalized translation / 2.980232e-08 rad rotation (37,137 / 37,144 measured; PARTIAL) | 36,842 / 37,144 (99.19%) | 9,950 / 9,996 (99.54%) |
| LAM released outputs | 10,328 / 10,381 (99.49%) | 8,886 / 10,381 (85.60%) | 8,855 / 10,381 (85.30%) | 8,882 / 10,381 (85.56%) | 0.000000 normalized translation / 2.107342e-08 rad rotation (8,886 / 10,381 measured; PARTIAL) | 8,849 / 10,381 (85.24%) | 2,822 / 3,217 (87.72%) |
| Artiverse | 16,320 / 16,332 (99.93%) | 16,220 / 16,332 (99.31%) | 15,837 / 16,332 (96.97%) | 16,220 / 16,332 (99.31%) | 0.000000 normalized translation / 0.000000e+00 rad rotation (16,220 / 16,332 measured; PARTIAL) | 15,837 / 16,332 (96.97%) | 3,333 / 3,544 (94.05%) |
| PartNet-Mobility | 11,923 / 11,971 (99.60%) | 11,923 / 11,971 (99.60%) | 11,898 / 11,971 (99.39%) | 11,919 / 11,971 (99.57%) | 0.000000 normalized translation / 2.980232e-08 rad rotation (11,923 / 11,971 measured; PARTIAL) | 11,888 / 11,971 (99.31%) | 2,326 / 2,347 (99.11%) |
| PhysX-Mobility | 9,883 / 9,883 (100.00%) | 9,883 / 9,883 (100.00%) | 2,633 / 9,883 (26.64%) | 9,883 / 9,883 (100.00%) | 0.000000 normalized translation / 2.107342e-08 rad rotation (9,883 / 9,883 measured; COMPLETE) | 2,632 / 9,883 (26.63%) | 1,198 / 2,024 (59.19%) |
| SketchMobility | 11,007 / 11,009 (99.98%) | 10,997 / 11,009 (99.89%) | 9,059 / 11,009 (82.29%) | 10,992 / 11,009 (99.85%) | 0.000000 normalized translation / 2.107342e-08 rad rotation (10,997 / 11,009 measured; PARTIAL) | 9,047 / 11,009 (82.18%) | 3,566 / 4,956 (71.95%) |
| Infinite Mobility (supplementary generated cohort) | 4,687 / 4,723 (99.24%) | 4,687 / 4,723 (99.24%) | 4,537 / 4,723 (96.06%) | 4,687 / 4,723 (99.24%) | 0.000000 normalized translation / 0.000000e+00 rad rotation (4,687 / 4,723 measured; PARTIAL) | 4,537 / 4,723 (96.06%) | 541 / 720 (75.14%) |
| Infinigen-Sim | 31,975 / 31,975 (100.00%) | 31,975 / 31,975 (100.00%) | 25,750 / 31,975 (80.53%) | 31,975 / 31,975 (100.00%) | 0.000000 normalized translation / 0.000000e+00 rad rotation (31,975 / 31,975 measured; COMPLETE) | 25,750 / 31,975 (80.53%) | 5,712 / 8,226 (69.44%) |

| Ours / PV-A category macro | Valid Range ↑ | Joint Sweep Success ↑ | Non-degenerate Motion ↑ | Subtree Consistency ↑ | FK Round-trip Error ↓ | Joint-level Pass ↑ | Strict Kinematic Pass ↑ |
|---|---:|---:|---:|---:|---:|---:|---:|
| 531 generator classes, unweighted mean | 99.95% | 99.95% | 99.95% | 98.80% | N/E | 98.49% | 98.59% |

---

## Receipt Status

All eight rosters and all 24 table checkpoints are complete. Asset-level errors and incomplete fingerprints are retained in the published denominators.

| Dataset | N_eval | J_eval | Table 1 | Table 2 | Table 3 | Roster |
|---|---:|---:|---|---|---|---|
| Articraft-10K | 9,996 | 37,144 | `complete` (EVALUATED=9996) | `complete` (completed=9996) | `complete` (completed=9995, error=1) | [`manifest.json`](articraft/full_release_manifest.json) |
| LAM released outputs | 3,217 | 10,381 | `complete` (EVALUATED=2924, EVALUATED_FINGERPRINT_INCOMPLETE=81, error=212) | `complete` (completed=3217) | `complete` (completed=3205, error=12) | [`manifest.json`](lam/full_release_manifest.json) |
| Artiverse | 3,544 | 16,332 | `complete` (EVALUATED=3526, error=18) | `complete` (completed=3543, error=1) | `complete` (completed=3543, error=1) | [`manifest.json`](artiverse/full_release_manifest.json) |
| PartNet-Mobility | 2,347 | 11,971 | `complete` (EVALUATED=2314, EVALUATED_FINGERPRINT_INCOMPLETE=33) | `complete` (completed=2347) | `complete` (completed=2343, error=4) | [`manifest.json`](partnet/full_release_manifest.json) |
| PhysX-Mobility | 2,024 | 9,883 | `complete` (EVALUATED=2024) | `complete` (completed=2024) | `complete` (completed=2024) | [`manifest.json`](physx/full_release_manifest.json) |
| SketchMobility | 4,956 | 11,009 | `complete` (EVALUATED=4901, EVALUATED_FINGERPRINT_INCOMPLETE=48, error=7) | `complete` (completed=4956) | `complete` (completed=4956) | [`manifest.json`](sketch/full_release_manifest.json) |
| Infinite Mobility | 720 | 4,723 | `complete` (EVALUATED=720) | `complete` (completed=720) | `complete` (completed=720) | [`manifest.json`](infinite/full_release_manifest.json) |
| Infinigen-Sim | 8,226 | 31,975 | `complete` (EVALUATED=6726, EVALUATED_FINGERPRINT_INCOMPLETE=1500) | `complete` (completed=8226) | `complete` (completed=8226) | [`manifest.json`](infinigen/full_release_manifest.json) |

Per-table evidence is under each dataset directory: `summary.json`, `records.jsonl` or `asset_records.jsonl`, `checkpoint.json`, and `artifact_manifest.json`.

Generated from the already verified receipts; this renderer does not run an evaluator.
