# Ours / PV-A SMOKE Results

Classification: **SMOKE**. Frozen evaluation: **128 assets**, **907 movable joints**, **3 generator classes**.

All manifest assets remain in the denominator. Parser errors, native crashes, and timeouts are retained as failures. Table 4 uses K=21 single-joint states and R=64 Sobol states with seed 20260813.

## Table 1. Dataset Scale and Structural Diversity

| Dataset | N_release | N_eval | Raw categories (release / eval) | Links / asset (mean / median / P90) | Movable joints / asset (mean / median / P90) | Multi-joint assets | Unique topologies | Exact duplicate rate |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Ours / PV-A | 302,440 | 128 | 531 / 3 | 8.38 / 3.0 / 25 (n=128) | 7.09 / 2.0 / 23 (n=128) | 85.94% (n=128) | 25.78% (n=128) | 0.00% (n=128) |

---

## Table 2. URDF Validity and Structural Integrity

| Dataset | Parse Rate | Resource Resolution | Finite Fields | Valid Tree | Valid Joint Spec. | Collision Coverage | Inertial Coverage | Inertia Validity | Strict URDF Pass |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Ours / PV-A | 128 / 128 (100.00%) | 128 / 128 (100.00%) | 128 / 128 (100.00%) | 128 / 128 (100.00%) | 128 / 128 (100.00%) | 128 / 128 (100.00%) | 90 / 128 (70.31%) | 90 / 128 (70.31%) | 90 / 128 (70.31%) |

---

## Table 2 Supplementary. Collision, Joint, and Inertial Diagnostics

| Dataset | Visual-bearing Collision Coverage | Joint-limit Portability | Joint Dynamics Coverage | Placeholder-mass Incidence |
|---|---:|---:|---:|---:|
| Ours / PV-A | 128 / 128 (100.00%) | 907 / 907 (100.00%) | 0 / 907 (0.00%) | N/E |

---

## Table 3. Kinematic Executability

| Dataset | Valid Range | Joint Sweep Success | Non-degenerate Motion | Subtree Consistency | FK Round-trip Error | Joint-level Pass | Strict Kinematic Pass |
|---|---:|---:|---:|---:|---:|---:|---:|
| Ours / PV-A | 907 / 907 (100.00%) | 907 / 907 (100.00%) | 907 / 907 (100.00%) | 907 / 907 (100.00%) | 0.000000 normalized translation / 0.000000e+00 rad rotation (907 / 907 measured; COMPLETE) | 907 / 907 (100.00%) | 128 / 128 (100.00%) |

---

## Table 4. Collision and Mechanical Clearance

| Dataset | Rest All-pair CF | Rest Non-adjacent CF | Single-joint Sweep CF | Multi-joint Sobol CF | Collision-state Rate | AOR | Max Penetration | Collision-free Range | Strict Collision Pass |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Ours / PV-A | 0 / 128 (0.000%) | 67 / 128 (52.344%) | 33 / 128 (25.781%) | 18 / 128 (14.062%) | 17,705 / 27,367 (64.695%) | N/E | 0.265281 (128 / 128 measured; COMPLETE) | 5,521 / 19,047 (28.986%) | 18 / 128 (14.062%) |

---

## Evidence

- Full receipt: `/mnt/zsn/lyb/arti-skill/exp/runtime/pva_table1234_full_release_20260826/smoke/full_release_receipt.json`
- Automation check: `/mnt/zsn/lyb/arti-skill/exp/runtime/pva_table1234_full_release_20260826/smoke/automation_check.json`
- Source roster: `/mnt/zsn/lyb/arti-skill/exp/runtime/pva_table1234_full_release_20260826/roster/roster_manifest.json`
- Result database: `/mnt/zsn/lyb/arti-skill/exp/runtime/pva_table1234_full_release_20260826/smoke/results.sqlite3`
