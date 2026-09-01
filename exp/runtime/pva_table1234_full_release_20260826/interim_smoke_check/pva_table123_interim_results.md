# Ours / PV-A Table 1/2/3 Interim Results

Classification: **INTERIM_ORDERED_PREFIX_NOT_FULL_RELEASE**.

Frozen committed prefix: **N=128 assets**, **J=907 movable joints**, **3 / 531 generator classes observed**. Formal intent remains N=128, J=907.

Worker status at cutoff: `completed=128`. Cutoff UTC: `2026-08-26T14:39:39.089759Z`.

This is not a full-release result and must not replace the final Ours / PV-A row. The roster is category-ordered, so this prefix is compositionally biased; all values will be replaced after the complete formal receipt passes verification.

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

## Evidence

- Interim manifest: `/mnt/zsn/lyb/arti-skill/exp/runtime/pva_table1234_full_release_20260826/interim_smoke_check/interim_manifest.json`
- Formal execution manifest: `/mnt/zsn/lyb/arti-skill/exp/runtime/pva_table1234_full_release_20260826/smoke/manifest.json`
- Frozen protocol snapshot: `/mnt/zsn/lyb/arti-skill/exp/runtime/pva_table1234_full_release_20260826/smoke/protocol_snapshot.md`
- Append-only result database: `/mnt/zsn/lyb/arti-skill/exp/runtime/pva_table1234_full_release_20260826/smoke/results.sqlite3`
