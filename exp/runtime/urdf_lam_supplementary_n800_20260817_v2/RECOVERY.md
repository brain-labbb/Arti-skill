# Recovery finalization — urdf_lam_supplementary_n800_20260817_v2

Frozen protocol: `urdf_lam_supplementary_n800_genesis_v1` (`genesis_contact_penetration_v1`).

## Why recovery was required

1. The frozen `run_scope` aggregation crashed on rank 46: the child receipt serialized
   `asset_record.intra_link_redundancy` with `status="N/E"` but `measured_link_count=1`,
   while the frozen verifier (`_redundancy_measurement`) requires `measured == 0` for N/E.
   Root cause: `lam_supplementary_geometry.collision_redundancy_measurement` early-return
   paths keep the pre-computed measured counter. No metric value is affected (the atom
   stays N/E with the same reason and null numeric fields).
2. `lam_supplementary_static.py` drifted from the frozen `code_identity` on 2026-08-19
   (frozen `1c2fdc2c3d9f8ebcb3ab6b0bf8144b307c86b4b44790cf3182c2395ab37267ff` -> observed `ac77a014a513cd7d0fa675e7aa46dcaf14433dbb7f01a47895c0010ea1bc3a73`), AFTER all child receipts were written (2026-08-17/18). The frozen execution-binding gate re-hashes current files and
   can therefore never pass for this completed run.

## Repair applied (single receipt, single field)

- Receipt `/mnt/zsn/lyb/arti-skill/exp/runtime/urdf_lam_supplementary_n800_20260817_v2/children/rank_0046.json`:
  `asset_record.intra_link_redundancy.measured_link_count` changed `1 -> 0`.
- SHA-256 after repair: `797708b779057f194e6a1eb79811c81e888e983dc1094656408867eecf3ef760`.
- Original buggy bytes preserved in `/mnt/zsn/lyb/arti-skill/exp/runtime/urdf_lam_supplementary_n800_20260817_v2/child_attempts/rank_0046.json`
  (SHA-256 `9ec40522c8158a81c2615819cb4c4fed76e45875bb5c7a6c1701cdbec290bbee`), untouched.
- Metric impact: none (N/E before and after; identical reason and null numeric fields).

## Binding comparisons at finalization

- runtime_binding matches freeze: True

| component | frozen sha256 | observed sha256 | matches |
|---|---|---|---|
| runner | `c43f3047553e4fbc9dfeefcbb7308bc42df8c6f0aab24f2a85f412c5efe12df5` | `c43f3047553e4fbc9dfeefcbb7308bc42df8c6f0aab24f2a85f412c5efe12df5` | True |
| static | `1c2fdc2c3d9f8ebcb3ab6b0bf8144b307c86b4b44790cf3182c2395ab37267ff` | `ac77a014a513cd7d0fa675e7aa46dcaf14433dbb7f01a47895c0010ea1bc3a73` | False |
| geometry | `55877794c911b5b760bca407ee0db9f5b4f562cfdd21aea5b513d3d8335f5f55` | `55877794c911b5b760bca407ee0db9f5b4f562cfdd21aea5b513d3d8335f5f55` | True |
| verifier | `834f9c92bfc17ce899bd3a36391ed94109763ade0d1ddb734ae2a972a620b830` | `834f9c92bfc17ce899bd3a36391ed94109763ade0d1ddb734ae2a972a620b830` | True |

## Aggregation evidence

- verifier._validate_strict_state_records: PASS (50336 strict rows)
- verifier.aggregate_records: PASS (800 assets, 2395 joints)
- verification aggregates SHA256: `86ef90bbf33d948809bbe2d444602e7a3e253100a6a4b274bcef88a9e43f09f4`
- summary status: PARTIAL (fail-closed assets retained in every denominator)

Final artifacts were written exactly once by the recovery finalizer
(`exp/scripts/finalize_urdf_lam_supplementary_recovery.py`) using the frozen runner's
row construction, sort orders, summary schema and report template.
