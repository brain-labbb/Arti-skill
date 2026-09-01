# LAM Naming baseline: official viable release

Status: **COMPLETE**

Protocol: `nano3d_table2_baseline_naming_v1.1`. This is an offline local evaluation
of the official LAM viable release, not a transcription of paper values.

## Coverage

- Frozen requested cohort: 2533 viable assets across 597 categories.
- URDF artifacts found: 2533/2533.
- URDF parse success: 2533/2533.
- Naming-evaluable artifacts: 2533/2533.
- Manifest/parsed link-count mismatches: 0.

## Direct Naming results

- Parts: 15009 renderable URDF links; 5.925385 mean per evaluable asset (median 4).
- Named / Nameability: 14909/15009 = 0.993337 micro; asset-macro 0.996193.
- Placeholder renderable parts: 100.
- Assets containing placeholder parts: 12/2533; fully nameable assets: 2521/2533.
- Representation: URDF renderable-link only; do not merge this row with GLB-node counts.

This is an official-release cohort audit, not a shared prompt/category-matched
rerun against the other methods; it should be labeled separately in Table 2.

The archive was streamed without extraction. It contains 204029
unique members and 175144 regular files; unsafe names=0,
duplicates=0, links=0, special members=0.
The 818 additional `pipeline_logs/.../generated.urdf`
members are intermediate feedback iterations; they are recorded but excluded
from the canonical final-artifact cohort and were not evaluated.

## Fail-closed metrics

Cross-seed consistency is `N/A`: LAM is evaluated as independent per-asset
generation and the release has no reusable factory/template plus seed identity.
Semantic Precision/Recall, Naming Richness, Functional Core Coverage, Instance
Discriminability, and Over-Segmentation remain `N/A`: no LAM-linked
output-independent role gold or three complete independent blind judges exist.

## Provenance

- Official code: `https://github.com/gaoypeng/LAM.git` at `0b3a87beb8c35273a5acf8681221791aff746d8e`.
- Manifest SHA256: `70216593ec02b71d596e456498ff9863ad0f8e519d5d27d2cf4f58792d412412`.
- Viable tar SHA256: `a582ef0aa0f3073749adcc73d289a12200e500c1a5762a4ee1530eefc2c4920d`.
- Evaluator SHA256: `863295216d0e8e6b11511e82ee74df6f261d2e850df7b1112d955c15b9a0d022`.
- Records SHA256: `0ec93a27881e4941ed2792277f7e762c1aeee1f6d13ee2b7a1afc11c4e22251c`.
