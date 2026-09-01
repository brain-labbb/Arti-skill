# Ours / PV-A Full-Release Results

Classification: **FORMAL_FULL_RELEASE**. Frozen evaluation: **302,440 assets**, **1,453,516 movable joints**, **531 generator classes**.

All manifest assets remain in the denominator. Parser errors, native crashes, and timeouts are retained as failures. Table 4 uses K=21 single-joint states and R=64 Sobol states with seed 20260813.

## Table 1. Dataset Scale and Structural Diversity

> The legacy pooled raw-tree ratio is a cohort-size-dependent support descriptor, not a higher-is-better diversity score. Cross-method topology claims require a shared category set and equal per-category budget.

| Dataset | N_release | N_eval | Raw categories (release / eval) | Links / asset (mean / median / P90) | Movable joints / asset (mean / median / P90) | Multi-joint assets | Pooled raw-tree support (descriptive) | Exact duplicate rate |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Ours / PV-A | 302,440 | 302,440 | 531 / 531 | 6.56 / 5 / 12 (n=302,435) | 4.81 / 3 / 10 (n=302,435) | 84.55% (n=302,440) | 5,356 / 302,435 (1.77%; pooled diagnostic) | 0.42% (n=302,435) |

### Table 1 topology interpretation

| Cohort | Pooled raw-tree support | Category-conditioned support | Category-macro support |
|---|---:|---:|---:|
| Ours / PV-A full release | 5,356 / 302,435 (1.77%) | 7,617 / 302,435 (2.52%) | 9.55% (531 categories) |

`rooted-joint-tree-v1` ignores names, geometry, and numerical parameters, includes fixed joints, and does not encode mimic dependencies. These values therefore describe raw rooted-URDF-tree signature support; they are not mechanism-level or geometry diversity scores. Exact category-stratified rarefaction at `k=5` is 62.92%; reproduce it with `exp/scripts/audit_pva_table1_topologies.py`.

---

## Table 2. URDF Validity and Structural Integrity

| Dataset | Parse Rate | Resource Resolution | Finite Fields | Valid Tree | Valid Joint Spec. | Collision Coverage | Inertial Coverage | Inertia Validity | Strict URDF Pass |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Ours / PV-A | 302,435 / 302,440 (100.00%) | 302,435 / 302,440 (100.00%) | 302,435 / 302,440 (100.00%) | 302,435 / 302,440 (100.00%) | 302,221 / 302,440 (99.93%) | 300,390 / 302,440 (99.32%) | 105,322 / 302,440 (34.82%) | 105,322 / 302,440 (34.82%) | 103,399 / 302,440 (34.19%) |

---

## Table 2 Supplementary. Collision, Joint, and Inertial Diagnostics

| Dataset | Visual-bearing Collision Coverage | Joint-limit Portability | Joint Dynamics Coverage | Placeholder-mass Incidence |
|---|---:|---:|---:|---:|
| Ours / PV-A | 302,435 / 302,440 (100.00%) | 1,452,399 / 1,453,516 (99.92%) | 169,681 / 1,453,516 (11.67%) | N/E |

---

## Table 3. Kinematic Executability

| Dataset | Valid Range | Joint Sweep Success | Non-degenerate Motion | Subtree Consistency | FK Round-trip Error | Joint-level Pass | Strict Kinematic Pass |
|---|---:|---:|---:|---:|---:|---:|---:|
| Ours / PV-A | 1,452,330 / 1,453,516 (99.92%) | 1,452,330 / 1,453,516 (99.92%) | 1,452,330 / 1,453,516 (99.92%) | 1,429,688 / 1,453,516 (98.36%) | 0.000000 normalized translation / 9.424322e-08 rad rotation (1,452,330 / 1,453,516 measured; PARTIAL) | 1,426,812 / 1,453,516 (98.16%) | 299,924 / 302,440 (99.17%) |

---

## Table 4. Collision and Mechanical Clearance

| Dataset | Rest All-pair CF | Rest Non-adjacent CF | Single-joint Sweep CF | Multi-joint Sobol CF | Collision-state Rate | AOR | Max Penetration | Collision-free Range | Strict Collision Pass |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Ours / PV-A | 3,335 / 302,440 (1.103%) | 217,607 / 302,440 (71.950%) | 196,427 / 302,440 (64.947%) | 194,467 / 302,440 (64.299%) | 16,297,421 / 50,182,436 (32.476%) | N/E | 0.845122 (302,157 / 302,440 measured; PARTIAL) | 19,600,773 / 30,523,836 (64.215%) | 192,352 / 302,440 (63.600%) |

---

## Evidence

- Full receipt: `/mnt/zsn/lyb/arti-skill/exp/runtime/pva_table1234_full_release_20260826/evaluation/full_release_receipt.json`
- Read-only Table 1 topology audit: `/mnt/zsn/lyb/arti-skill/exp/scripts/audit_pva_table1_topologies.py`
- Automation check: `/mnt/zsn/lyb/arti-skill/exp/runtime/pva_table1234_full_release_20260826/evaluation/automation_check.json`
- Source roster: `/mnt/zsn/lyb/arti-skill/exp/runtime/pva_table1234_full_release_20260826/roster/roster_manifest.json`
- Result database: `/mnt/zsn/lyb/arti-skill/exp/runtime/pva_table1234_full_release_20260826/evaluation/results.sqlite3`
