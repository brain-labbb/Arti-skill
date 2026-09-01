# Table 6: Infinite Mobility articulation

- Status: COMPLETE_STATIC_AND_DRIVE; collision metrics BLOCKED
- Frozen cohort: 20 factories x 36 seeds = 720 requested cases
- Generated/evaluated: 713/720; generation timeouts: 7
- Evaluator outcomes: {'DRIVE_FAIL': 1, 'INPUT_NOT_GENERATED': 7, 'PASS': 712}
- Articulable: 657/720 intent-to-run; 657/713 conditional on generated
- Movable joints: 4514; mean 6.269/requested case, 6.331/generated package
- Joint load/drive: 48906 single-joint states and 34688 multi-joint Sobol configurations verified
- Metadata proxies: parent-child 4514/4514; axis 4514/4514; origin 4514/4514; bounded limits 3226/3226
- Generic-range proxy: 150/1201 revolute joints >= 300 degrees; continuous joints: 1288
- Collision geometry: 0 elements across 713 generated packages
- Drive failure: WindowFactory seed 13 loads and maps 68 movable joints, but getJointState fails for its 140-link/139-joint body
- Integrity scope: frozen generation manifest/records are SHA-256 pinned; full mesh-package rehash was skipped in this evaluator invocation

## Table 6 candidate row

| Method | Articulable | Joints/Asset | Native Joint Exposure | Joint Type Accuracy | Joint Recall | Parent-Child Accuracy | Axis Valid | Origin Valid | Limit Valid | Joint Geom. Valid | Asset Geom. Valid | Full-Range Collision-Free | Generic Range |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Infinite Mobility (20 public factories x 36 seeds) | 657/720 = 91.2% intent-to-run; 657/713 = 92.1% generated-only | 6.269 requested; 6.331 generated-only | 658/720 assets; 4514/4514 declared joints* | N/A | N/A | N/A (4514/4514 structurally valid*) | N/A (4514/4514 metadata-valid*) | N/A (4514/4514 metadata-valid*) | N/A (3226/3226 bounded metadata-valid*) | N/A (no collision geometry/gold) | N/A (no collision geometry/gold) | N/A (0 collision elements) | 150/1201 revolute >=300 deg*; 1288 continuous* |

`*` denotes a structural/metadata operational proxy, not semantic joint correctness.
The collision-related columns are N/A because these exports contain no native collision geometry; reporting zero contacts would be vacuous.
The primary denominator is all 720 requested factory-seed cases. The generated-only denominator is included only to separate seven generation timeouts from 55 valid but non-articulated packages.
