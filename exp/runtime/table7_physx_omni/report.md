# Table 7 PhysX-Omni Production Readiness audit

Status: **SUPERSEDED_AVAILABILITY_PREFLIGHT / TABLE7_NOT_EVALUATED**  
Interpretation: **Table 6 now has seven attributable exploratory packages; no Table 7 production-readiness run has been performed**  
Local Table 7 evaluated final packages: **N=0**

> This 2026-08-11 availability preflight is superseded only with respect to model/output availability. The later Table 6 pilot downloaded the pinned weights, used a disclosed adapter, and produced seven attributable packages. It did not execute the Table 7 topology, portability, deterministic rebuild, semantic, kinematic, or physical-completeness protocol, so all Table 7 cells remain `not_evaluable` rather than inheriting Table 6 values.

Official code is locally available at source commit
`46fa1cd0b6883d4d14431d51c3326ef80a85ef64` (tree
`68c9717f55c809998ecc5ded95d067af9351578d`). The fixed source archive hash matches
the Table 6 pin, and `18` local metadata
files are inventoried with byte counts and SHA-256 values in `manifest.json`.

This code-availability evidence is distinct from production-output evidence. The
prior CPU tiny smoke exercised benchmark manifest/aggregation plumbing only; it
did not run PhysX-Omni inference and did not produce a final asset package.

## Pinned model inputs and blockers

| Input | Frozen revision | Local state |
|---|---|---|
| PhysX-Omni 8B checkpoint | `765cd275839f88333cb754f1c6c0b8d3887a3b2c` | now present for the later Table 6 pilot |
| TRELLIS-image-large | `25e0d31ffbebe4b5a97464dd851910efc3002d96` | now present for the later Table 6 pilot |
| Method-generated final package | later Table 6 exploratory scope | 7 attributable packages present; not Table 7-evaluated |
| Common Table 7 output adapter | frozen local harness | not implemented/run |

No network, download, GPU, inference, or generation was performed by this
historical Table 7 runner. Its local `N=0` is now only the Table 7 evaluation
denominator, not the workspace output-availability denominator.

## Common-protocol row

| Method | Watertight | Manifold | Open Edges | Degenerate Faces | Self-Intersection | Source KB | URDF KB | Mesh KB | Portable Package | Deterministic Build | Semantic Complete | Kinematic Complete | Physical Complete |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| PhysX-Omni (7 exploratory packages available; Table 7 formal audit N=0) | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable |

All 13 cells are `not_evaluable`. Geometry, size, portability, rebuild, and
completeness tests require a dedicated Table 7 execution. The seven later
Table 6 packages were not retroactively converted into Table 7 scores.

Protocol SHA-256: `5fc86932f35f8b66514d5747be732b5c75fef7215c987628f5dd28522f710a7c`  
Manifest SHA-256: `529d093d1bec4970e81d6a849c141633344540ddcf2eb4755a516ff58bf6a6e1`
