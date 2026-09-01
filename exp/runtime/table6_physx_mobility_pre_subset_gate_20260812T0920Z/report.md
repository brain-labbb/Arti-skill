# Table 6: PhysX-Mobility release reference

Status: **PARTIAL_RETAINED_FAILURES_COLLISION_NA**. This is a same-ID PartNet-Mobility derivative dataset reference, not PhysX-Omni or PhysX-Anything generated-method output.

## Provenance and scope

- Official HF revision: `d0768ee9e1415f6be8db78d6389ba018b85134c0`; archive SHA-256: `88308cc2a4cc6177c59e32c2de51e881e6b961737295e5082d7ed01cca221908`; license: CC-BY-NC-4.0.
- Main denominator: 2024 numeric URDF/JSON/partseg IDs present in the official archive. The 28 local `_collision`/`_sim` variants are excluded.
- Archive binding: URDF 2024/2024 and finaljson 2024/2024 are byte-exact by size+SHA-256. Partseg relative-path inventory is NOT exact (295978/291305 local/archive files; missing 0, extra 4673).
- All 2024/2024 IDs exist in the local PartNet-Mobility release. Dataset-card lineage and same-ID reuse make PartNet agreement export fidelity, not independent accuracy.

## Static release audit

- Static package pass: 2024/2024; valid URDF trees: 2024/2024.
- Movable joints: 9896 ({"floating": 13, "prismatic": 7250, "revolute": 2633}), mean 4.889328/asset; functional-motion 9883; zero-width 0; unsupported/floating 13.
- Metadata proxies: parent/child structurally valid 9896/9896; axis metadata valid 9896/9896; origin metadata valid 9896/9896; bounded limits 9883/9883.
- Visual mesh references: 91855; missing: 0. Collision elements: 0; collision mesh references: 0.
- Inertial tags are syntactically positive on 26016/26016 links, but 26016/26016 are the uniform mass=1 / unit-diagonal placeholders and are not physical mass fidelity.
- `finaljson.group_info` to released URDF export fidelity: names 3242/9896; all encoded fields 2898/9896. This is same-release serialization fidelity.
- Exact child-mesh matched PartNet/PhysX pairs: 9733; exact type 8764; rotational class 9733; axis line 9714. These are same-source preservation diagnostics.

## PyBullet v3 reset/readback

The frozen cohort contains one outcome-independent SHA-256 winner from each of 45 exact PartNet categories. Load/reset outcomes never affect selection and failures are not replaced.
- Load / joint-map / complete: 45/45 / 45/45 / 44/45.
- Declared/functionally swept/zero-width/unsupported joints: 273 / 273 / 0 / 0.
- Executed states: 1848/3003 single-joint and 1600/1664 Sobol; max reset/readback error 0 (tol 1e-09).
- Self-collision protocol smoke: PASS; contacts by flag {"exclude_parent_only": 0, "none": 0, "self_collision": 4, "self_collision_exclude_parent": 4}. This fixture validates flags only and is excluded from dataset metrics.
- Motors are disabled. Each pose uses resetJointState -> performCollisionDetection -> getJointState with no stepSimulation. Each asset runs in an isolated subprocess.
- Contact/penetration rates are intentionally not collected: zero official collision elements make them vacuous. This is not CCD or a geometry-validity test.

## Table 6 candidate row

| Method | Articulable | Joints/Asset | Native Joint Exposure | Joint Type Accuracy | Joint Recall | Parent-Child Accuracy | Axis Valid | Origin Valid | Limit Valid | Joint Geom. Valid | Asset Geom. Valid | Full-Range Collision-Free | Generic Range |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| PhysX-Mobility (PartNet-derived dataset reference; supplementary) | 2021/2024 expose movable URDF joints* | 4.889 (9896/2024)* | 9896/9896 declared joints* | N/A (same-source derivative) | N/A (same-source derivative) | N/A (9896/9896 structurally valid*) | N/A (9896/9896 metadata-valid*) | N/A (9896/9896 metadata-valid*) | N/A (9883/9883 bounded metadata-valid*) | N/A (0 collision elements/no independent axis gold) | N/A (0 collision elements/no independent gold) | N/A (0 collision elements) | 41/2633 revolute >=300 deg*; 0 continuous* |

`*` denotes package exposure, metadata, or same-source export fidelity, never independent articulation accuracy. Keep this row supplementary and out of generated-method rankings.
