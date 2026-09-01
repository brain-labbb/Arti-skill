# PhysX-Omni Table 6 exploratory method pilot

## Positioning

Status: `EXPLORATORY_PARTIAL`; `publication_gate=false`. This is an attributable exploratory pilot on eight identities from a PhysX-Anything mobility test list. It is **not** a held-out claim: the released PhysX-Omni training configuration names PhysX-Mobility, and public evidence does not establish that these IDs were excluded. It also has no independent semantic joint gold.

The session record fixed all eight IDs before outputs were observed and retained the failed ID without replacement. However, no pre-run local selection artifact survives, so `selection_reproducibility=NOT_EVIDENCED` and the reproducibility check is `FAIL`. Do not describe this as a reproducible salt-argmin or formal outcome-independent benchmark cohort.

## Result

- Intent: 8 assets / 8 PartNet categories.
- Attributable complete outputs: 7/8; retained incomplete: 1/8 (`102001`, Stage1 interrupted after nine part outputs without `allind.npy` or a success marker).
- Static XML: 7/7 complete outputs parse as valid trees.
- Articulation: 13 movable joints, mean 1.857 per complete asset.
- Structural metadata: parent/child 13/13; axis 13/13; origin 13/13; bounded/continuous limit metadata 13/13.
- PyBullet reset/readback: 7/7 complete outputs; 39 states; max error 0.0 m/rad-equivalent joint coordinate units. Protocol disables motors, then uses `resetJointState -> performCollisionDetection -> readback`, with no `stepSimulation`.
- Collision: 0 collision elements, therefore penetration/full-range/CCD are `N/A`, not a vacuous pass.
- Inertia: 47/47 syntactically valid inertials are uniform placeholders; physical inertia fidelity is `N/A`.
- Semantic diagnostic: upstream `102187` is TrashCan, but its generated output calls itself `Double Door Refrigerator` / `Appliance`. This is retained as a structural generation success and an observed semantic mismatch, never as semantic accuracy.
- Independent semantic type/recall/parent/axis/origin/limit accuracy: `N/A`.

## Provenance

- Source commit: `46fa1cd0b6883d4d14431d51c3326ef80a85ef64`. Execution used a disclosed five-file operational adapter for deterministic ordering/seeds, explicit local dependency routing, checked subprocesses, and fail-fast evidence; this was not an untouched-checkout execution.
- PhysX-Omni model revision: `765cd275839f88333cb754f1c6c0b8d3887a3b2c`; TRELLIS revision: `25e0d31ffbebe4b5a97464dd851910efc3002d96`; Qwen processor revision: `cc594898137f460bfe9f0759e9844b3ce807cfb5`.
- Dependency integrity: original manifest 45/45 `PASS`. It omits the invoked Qwen processor/tokenizer directory; the independent audit separately verifies 11/11 files at `cc594898137f460bfe9f0759e9844b3ce807cfb5`, so primary-manifest coverage remains a formal `FAIL`.
- Per-stage explicit times are retained when present. Missing historical start/finish/wall fields remain `unknown`; method-log timestamps and file mtimes are separate evidence fields and are not promoted into exact wall times.
- Terminal evidence chain: independent audit JSON SHA-256 `18f9d60ac22f6b3cd754d6ef9ba3cb1a35010feaaf6cb2312726d69ce6c7a889`; audit Markdown SHA-256 `f1367ef57cd47ac159dcc26ce0f45ebfb1cff048e6d1a89a31b44b5d0d21cd04`; `manifest.json` SHA-256 `d833f74fbaf91412cf62d62d6ceae230b03b20bf58593dec86d37cd4607370d8`; `summary.json` SHA-256 `05414ad7d52a27299ae11ea79e9914179d625eaf682250b05c84bca1a8f01464`. The independent audit pins `stage_records.json` SHA-256 `7c434e7673172f4e905da4b847d1b405af70f342b106c33f2d7b86dcad4111ef` and `asset_records.json` SHA-256 `b96ee50f80f338d365e287b3642c833dfade43bb79460894bdbb66dfb3ff206b`.

See `manifest.json`, `stage_records.json`, `asset_records.json`, `summary.json`, and `self_check.json` for machine-readable denominators, hashes, and limitations.
