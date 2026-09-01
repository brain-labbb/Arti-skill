# Infinite Mobility Table 3 Kinematic Executability

Run classification: **FORMAL**.

This is a supplementary full generated cohort, not an official finite release. It preserves 720 frozen factory/seed identities, including 7 recoveries, in the exact Table 2 manifest order.

N_eval=720; J_eval=4723; K=21 states per joint.

Expected/observed joint denominator: 4723 / 4723.

Zero-joint assets: 55 / 720 ({"TableDiningFactory": 19, "VaseFactory": 36}); these assets fail Strict Kinematic Pass by protocol.

| Metric | Result |
|---|---:|
| Valid Range | 4687 / 4723 (99.24%) |
| Joint Sweep Success | 4687 / 4723 (99.24%) |
| Non-degenerate Motion | 4537 / 4723 (96.06%) |
| Subtree Consistency | 4687 / 4723 (99.24%) |
| Joint-level Pass | 4537 / 4723 (96.06%) |
| FK Round-trip Error | translation=0.0; rotation_rad=0.0; coverage=4687 / 4723 (PARTIAL) |
| Strict Kinematic Pass | 541 / 720 (75.14%) |

Category macro over 20 factories (19 with movable joints):

| Metric | Category macro |
|---|---:|
| Valid Range | 99.80% (categories=19) |
| Joint Sweep Success | 99.80% (categories=19) |
| Non-degenerate Motion | 95.38% (categories=19) |
| Subtree Consistency | 99.80% (categories=19) |
| Joint-level Pass | 95.38% (categories=19) |
| Strict Kinematic Pass | 75.14% (categories=20) |

Worker status counts: `{"completed": 720}`.

Parse/tree: 720 / 720 parsed; 720 / 720 valid trees.

Provenance: 713 original PASS and 7 recovery overlays.

Attestation: package before/after 720 / 720; runtime matches 720 / 720.

Hash evidence:

- manifest content: `28ac7cec9b80221786c14dca2e546e7ecca73c813a6ad9a101dc43d3d4a6335b`
- adapter: `50f11da87296046323f9d6d1330f62b023be70084452da9151e013d10740bb2d`
- core evaluator: `0da075f077ce13c78bb6b4ee66b0abe77668ccf7bb3c105660b321e667fc2acf`
- protocol: `b05a4edbe61037f2dc4bc1bc1580e66f13f852fc24a3b979e130e8a7aa30ef00`
- environment: `76e794ab8a092763461b854b220ae7d0273508722139df04ee45d9ba7865b558`
- cohort manifest: `cfd9c06ea35dcec57c53d44dbf52903ecba6f33321075495c97c58fe30d23c08`
- Table 2 manifest: `3dce6436aac2d25507d7843a3e0e5cbee130e83e0c24c2bfbfb08467ca356290`

Artifact closure: `manifest.json`, `asset_records.jsonl`, `summary.json`, `report.md`, `environment.json`, `protocol_snapshot.md`, `checkpoint.json`, and `artifact_manifest.json`.

All timeout, exception, malformed-result, and package-drift cases remain in the asset and declared-joint denominators.

This discrete FK evaluation does not establish semantic joint correctness, collision safety, dynamics, or real-world fidelity.
