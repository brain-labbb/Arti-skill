# PhysX-Mobility 12836 PyBullet Diagnosis

Status: **confirmed PyBullet multibody state-interface limitation; uniform workaround validated**.

This was a read-only, isolated diagnosis. It did not change the formal runner, frozen cohort, or formal result artifacts.

## Input and environment

- Dataset asset: `12836` (`Keyboard`)
- Frozen URDF SHA-256: `2b492feff73de2386998c4450b51467f8c23c7b0ce31e825267c8c6c48710b47`
- URDF topology: 211 joints total, comprising 105 movable and 106 fixed joints
- Native collision elements: 0
- Python: `exp/.venv_low_medium/bin/python`
- PyBullet API: `202010061`; build time: `Aug 4 2026 09:39:21`
- All five thread caps were fixed at 1.

## Isolated failure matrix

| Load case | Integer flags | Loaded joints | `performCollisionDetection` | `getJointState` |
|---|---:|---:|---|---|
| Formal protocol | 1048602 | 211 | PASS | FAIL |
| Without `URDF_IGNORE_VISUAL_SHAPES` | 26 | 211 | PASS | FAIL |
| `URDF_USE_INERTIA_FROM_FILE` only | 2 | 211 | PASS | FAIL |
| No flags | 0 | 211 | PASS | FAIL |
| `URDF_USE_SELF_COLLISION` only | 8 | 211 | PASS | FAIL |

The first movable joint is index 1 (`joint_prismatic_l_0_abstract_0_1`). Its state read fails immediately after loading, after `resetJointState`, and after `performCollisionDetection`. `getLinkState` and `getBasePositionAndOrientation` also fail, while `performCollisionDetection` itself succeeds. The failure therefore is not caused by a particular joint index, self-collision flags, ignoring visuals, collision detection, or call order.

## Capacity evidence

Among available dataset assets tested around the boundary, all assets through 125 joints returned body/joint/link state successfully; the next available size is 155 joints, and all tested assets at 155 or more joints failed the state APIs. There are no dataset assets between 126 and 154 joints, so the URDF-loader threshold is bracketed rather than claimed as an exact dataset-derived threshold.

An independent synthetic `createMultiBody` probe accepted and reported 127 joints. Requests for 129 and 211 joints were silently truncated to 127. This exposes a 127-joint multibody cap in the installed PyBullet client. Combined with the whole-body state failure for a URDF-reported 211-joint body, the evidence supports a PyBullet command/status serialization capacity limitation.

## Validated workaround

Adding `URDF_MERGE_FIXED_LINKS` (`524288`) uniformly to the formal flags changes the loaded topology from 211 to 105 joints while retaining all 105 movable joints. For asset 12836:

- movable joint names are unique and match XML exactly and in order;
- joint type, limits, and axis have 0 mismatches;
- all 105 reset/readback calls succeed with maximum error 0;
- collision detection succeeds.

Because this release contains zero native collision tags, merging fixed links does not remove collision geometry used by the current functional collision proxy. It does change the runtime topology, so it must be declared and audited rather than applied silently.

## Recommendation

Add `URDF_MERGE_FIXED_LINKS` to the functional PyBullet load flags for the **entire frozen PhysX-Mobility cohort**, and rerun all 45 assets without changing sample selection. Fail closed unless the post-load movable count, unique names/order, joint types, limits, and axes match the frozen XML records. Do not special-case 12836 and do not count the pre-merge failure as a native asset failure.
