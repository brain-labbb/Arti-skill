# Original Articraft Table 7 production-readiness audit

## Scope

The audit retains all 143 frozen rating>=4 records. The strata are 126 strict compile successes and 17 strict compile failures with recovered URDFs. Recovered exports are auditable, but none is promoted to a strict compile success.

## Table 7 row

| Method | Watertight | Manifold | Open Edges | Degenerate Faces | Self-Intersection | Source KB | URDF KB | Mesh KB | Portable Package | Deterministic Build | Semantic Complete | Kinematic Complete | Physical Complete |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Original Articraft (frozen N=143) | 0.683345 mean/asset; 298/515 geometries | 0.996633 mean/asset; 514/515 geometries | 2103.525/asset; 208249 total | 50.909/asset; 5040 total | N/A | 13.08/asset; 1871.07 total | 18.70/asset; 2674.60 total | 621.47/asset; 61525.17 total | 143/143 | N/A (no two fresh builds) | N/A (field proxy separate) | 125/143 | 0/143 |

Manifold is the frozen edge-manifold proxy (every undirected edge has at most two incident faces); vertex-manifold is not claimed.
Geometry denominators contain only packaged or URDF-referenced readable triangle mesh payloads. The evaluator does not triangulate primitive-only URDF geometry.
Geometry not evaluable: 44; no mesh payload: 44; mesh load errors: 0.

## Cohort and evidence states

| Cohort | Requested | Available | Geometry evaluable | Portable pass | Kinematic pass | Physical pass |
|---|---:|---:|---:|---:|---:|---:|
| All | 143 | 143 | 99 | 143/143 | 125/143 | 0/143 |
| strict_compile_success | 126 | 126 | 88 | 126/126 | 113/126 | 0/126 |
| strict_compile_failure_recovered_urdf | 17 | 17 | 11 | 17/17 | 12/17 | 0/17 |

## Non-ranking field proxy

The semantic named-tree field proxy passes 143/143 assets. This is not semantic completeness or semantic correctness; strict Semantic Complete remains not_evaluable for all assets.

## Completeness field gates

| Axis | Gate | Passing assets |
|---|---|---:|
| kinematic_complete | explicit_finite_joint_origins | 125/143 |
| kinematic_complete | movable_one_axis_joint_axes | 143/143 |
| kinematic_complete | parent_child_references | 143/143 |
| kinematic_complete | recognized_joint_types | 143/143 |
| kinematic_complete | revolute_prismatic_limits | 143/143 |
| physical_complete | movable_joints_have_native_damping_and_friction | 2/143 |
| physical_complete | native_contact_material_or_friction_metadata | 0/143 |
| physical_complete | physical_links_have_positive_definite_native_inertia | 58/143 |
| physical_complete | physical_links_have_positive_native_mass | 58/143 |
| physical_complete | visual_links_have_native_collision | 143/143 |

## Fail-closed exclusions

- Self-intersection: not_evaluable; no exact backend excluding adjacent faces was run.
- Deterministic build: not_evaluable; re-hashing an existing artifact was not treated as a rebuild.
- Semantic completeness: not_evaluable; no output-independent required-part and role specification exists.
- Physical metadata: simulator defaults from Table 6 do not count as native package fields.

## Reproduction

```bash
python exp/scripts/run_table7_articraft.py
python exp/scripts/run_table7_articraft.py --verify-only
```

Evidence: `protocol_snapshot.json`, `manifest.json`, `asset_records.json`, `summary.json`, and `self_check.json` in this directory.
