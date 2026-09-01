# Table 6 LAM articulation baseline

Status: **PARTIAL_COMPLETE**

This is a local audit of the official LAM release, not a same-prompt generation
rerun. No paper metric was copied into the local row. The official code is fixed
at commit `0b3a87beb8c35273a5acf8681221791aff746d8e` (Apache-2.0); the official
dataset is fixed at revision `28cec4f5be7e34fd4d586879ecfcb67f7c5e4cc0` (MIT).

## Completed scope

- Full structured audit: 3,217/3,217 inline URDFs parsed. Tiers are 2,533 viable,
  299 loads-only, and 385 broken.
- Viable primary cohort: 2,533/2,533 have movable joints; 7,613 movable joints,
  mean 3.006 per asset.
- Native metadata: 7,613/7,613 joints have structurally valid parent/child,
  nonzero finite axis, finite/default origin, and required bounded limits.
- Bounded limits: 6,345/6,345 metadata-valid. Explicit origin is 4,610/7,613;
  absent URDF origins use the standard identity default.
- Generic range: 194/3,747 revolute joints (5.18%) span at least 300 degrees;
  1,267 continuous joints are reported separately.
- Collision availability: 1,198/2,533 viable assets (47.30%) expose collision
  elements in the release URDF.

The 1,185,271,461-byte viable archive matches official SHA-256
`a582ef0aa0f3073749adcc73d289a12200e500c1a5762a4ee1530eefc2c4920d`.
It contains 204,029 members, with no links or unsafe paths. A deterministic
100-category cohort was frozen before outcomes; 100/100 extracted URDFs exactly
match the corresponding parquet row and every mesh reference resolves.

## Functional proxy

- Correction history (2026-08-12): functional protocol v1 passed
  `URDF_USE_SELF_COLLISION_EXCLUDE_PARENT` alone (flag 16), which does not enable
  self-collision. Its 100% collision-free result is invalid. Protocol v2 enabled
  self-collision but treated any reported contact as failure. Protocol v3 uses
  the same combined flag 24, explicitly disables all 311 movable-joint motors,
  performs collision detection directly after each reset without simulation
  stepping, and separates contact from penetration. The frozen cohort and
  sampled motion states are unchanged.
- PyBullet load: 100/100; drive sampling: 100/100; failed/timeout assets: 0.
- All 100 assets declare 311 movable joints. No bounded zero-width joints occur
  in this cohort (0/311), so all 311 are motion-sweep eligible and execute 7,069
  states. The evaluator retains zero-width joints in declared counts but excludes
  them from motion joint/state/asset denominators.
- Collision-equipped subset: 90 assets, 279 declared and motion-eligible joints,
  3,069 single-joint states, and 3,328 Sobol multi-joint states. It contains 0/279
  bounded zero-width joints.
- Primary penetration-free proxy (`max penetration <= 1e-6 m`): 1,750/6,397
  states (27.36%), 75/279 complete single-joint sweeps (26.88%), and 37/90
  assets (41.11%). By state family, 922/3,069 single-joint states (30.04%) and
  828/3,328 Sobol multi-joint states (24.88%) pass.
- Strict contact-free companion metric: 1,694/6,397 states (26.48%), 75/279
  joint sweeps (26.88%), and 36/90 assets (40.00%). By state family, 915/3,069
  single-joint states (29.81%) and 779/3,328 Sobol states (23.41%) pass.
- All 311 motors were disabled with zero force. Collision checking used
  `resetJointState` then `performCollisionDetection`, with zero calls to
  `stepSimulation`; observed maximum reset-pose error was exactly 0.
- The 10 assets without collision elements are collision `N/A`; they are not
  included in the 6,397-state, 279-joint, or 90-asset denominators, and their
  per-asset collision fields are JSON `null`.

This is discrete PyBullet checking, not CCD. PyBullet emitted 554 missing-inertial
warnings and used default mass/inertia, so the result demonstrates kinematic
load/drive and a collision-geometry proxy, not physical readiness.

## Table 6 row

| Method | Articulable | Joints/Asset | Native Joint Exposure | Joint Type Accuracy | Joint Recall | Parent-Child Accuracy | Axis Valid | Origin Valid | Limit Valid | Joint Geom. Valid | Asset Geom. Valid | Full-Range Collision-Free | Generic Range |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| LAM official release audit | 2,533/2,533 metadata; 100/100 local load+drive* | 3.006 (7,613/2,533) | 7,613/7,613* | N/A | N/A | N/A; 7,613/7,613 structural* | N/A; 7,613/7,613 finite nonzero* | N/A; 7,613/7,613 finite/default* | N/A; 6,345/6,345 bounded metadata* | N/A; 75/279 penetration-free proxy* | N/A; 37/90 penetration-free proxy* | 1,750/6,397 states at 1e-6 m tolerance on 90 collision-bearing assets* | 194/3,747 revolute (5.18%); 1,267 continuous* |

`*` denotes a direct local metadata or collision proxy, not semantic accuracy.
Type accuracy, joint recall, semantic parent-child/axis/origin/limit validity,
joint/asset geometric validity, and rest-pose preservation remain `N/A` because
there is no independent frozen joint gold or pre-articulation artifact pair.
The shared PV-A prompt manifest and provider API credentials were unavailable;
credentials were not inspected, so a fair same-prompt generation rerun remains blocked.

## Artifacts

- `metadata_summary.json`: full 3,217-row aggregate.
- `metadata_asset_records.json`: per-row static audit.
- `frozen_cohort.json`: outcome-independent 100-category selection.
- `package_preflight.json`: package/hash/mesh resolution checks.
- `functional_summary.json`: protocol-v3 motor/reset/contact/penetration policy,
  corrected results, and explicit denominators.
- `functional_asset_records.json`: per-asset load/drive/sweep records.
- `sweep_stdout.log`: PyBullet runtime warnings.
- `evaluate_release.py`: reproducible evaluator; released JS/code is never executed.
