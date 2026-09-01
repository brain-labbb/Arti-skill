# Infinite Mobility Table 1 read-only review

## Historical pre-formal verdict

**NOT READY at the time of the initial review.** The findings below describe
the pre-fix implementation and are superseded by the actual publication
acceptance at the end of this file.

The five submitted tests pass, but the implementation does not yet satisfy the formal 720-asset contract. I found **4 Critical**, **5 Important**, and **1 Minor** issue.

## Critical

### 1. A single package-binding failure aborts the entire run instead of becoming a failed record in the 720 denominator

- Location: `scripts/run_table1_infinite_mobility.py:109-123`, especially the preflight loop at `:116-117`; call site at `:153`.
- Evidence: `evaluate_cohort()` calls `_verify_row()` for every selected row before it enters `_evaluate_row()`. `_evaluate_row()` has the intended fail-closed exception conversion at `:74-106`, but binding/hash/path failures never reach it. One missing or drifted package therefore produces no `asset_records.jsonl`, summary, report, or 720-row denominator.
- Test evidence: `tests/test_infinite_mobility_table123.py:169-171` explicitly expects the whole evaluation to raise on one URDF drift, so the test codifies behavior opposite to the requirement.
- Fix: validate only global manifest/cohort invariants as fatal. Run every selected asset through `_evaluate_row()` and convert all asset-local path, binding, parse, and fingerprint failures into a `FAILED` record. Assert `len(records) == N_eval` before aggregation and publication. Add a test with two assets where one drifts and verify two records, `status_counts={"COMPLETED": 1, "FAILED": 1}`, and every coverage/multi-joint denominator remains 2.

### 2. The required protocol snapshot is replaced with a two-line placeholder and the report is not a Table 1 result report

- Location: `scripts/run_table1_infinite_mobility.py:177-184`.
- Evidence: `protocol_snapshot.md` is generated from a hard-coded heading rather than byte-copying `exp/URDF-Sim-Ready-Automatic-Evaluation.md` (the live protocol is approximately 263 KB). `report.md` contains only `N_release` and `N_eval`; it omits the shared evaluator's Table 1 metrics, denominators, coverage, status diagnostics, provenance, and formal/supplementary qualifications.
- Fix: freeze the real protocol bytes with no-follow regular-file reads, record both live-source-at-freeze and snapshot SHA-256, verify byte equality before publication, and include the snapshot in the artifact manifest. Adapt the shared Table 1 report renderer so the Infinite Mobility report includes the complete metric table and diagnostics, with the supplementary-cohort wording and recovery counts.

### 3. The formal run is not source- or evaluator-bound and its run manifest has no self-hash

- Location: `scripts/run_table1_infinite_mobility.py:157-170`; source freeze at `scripts/infinite_mobility_table123_common.py:186-205` and `:258-275`.
- Evidence: the run manifest binds only the cohort manifest. It omits runner/evaluator SHA-256, shared evaluator protocol identifiers, Python/environment binding, real protocol source/snapshot hashes, and a `manifest_content_sha256`. The cohort manifest in turn omits byte hashes for the primary `manifest.json`, primary `records.json`, recovery manifest/summary, and the preparation runner. Consequently the same nominal formal command can silently use changed source records or evaluator code while still producing a manifest marked `formal: true`.
- Fix: add canonical self-hashes to both cohort and run manifests; freeze and verify hashes for every identity/provenance source, the preparation script, the Infinite Mobility runner, the imported shared Table 1 evaluator, and the real protocol snapshot. Record the evaluator protocol constants and relevant deterministic environment metadata. Validate all bindings before evaluation and again before publish.

### 4. `--formal` accepts an arbitrary self-hashed list of 720 unique asset IDs, not the confirmed 20 x 36 frozen identity matrix

- Location: `scripts/run_table1_infinite_mobility.py:33-49`, `:147-150`; preparation check at `scripts/infinite_mobility_table123_common.py:300-305`.
- Evidence: formal validation checks only `N_release == 720` and the selected list length. It does not validate the cohort's declared `N_eval`, exact factory order/set, seeds `0..35`, one row for every factory-seed pair, row `asset_id/factory/seed/raw_category` consistency, 713 original `PASS` plus exactly 7 original `TIMEOUT` recoveries, or an approved cohort manifest hash. Preparation similarly accepts any 20 distinct factory names from a modified source manifest.
- Fix: define and validate the exact approved factory list and seed sequence, derive the 720 expected asset IDs, compare rows one-for-one, require declared `N_release == N_eval == len(assets) == 720`, verify the 713/7 provenance split and recovery identity set, and bind formal execution to the approved frozen cohort content hash (or an independently frozen source-universe hash).

## Important

### 5. Recovery records can be redirected anywhere inside the repository, and original TIMEOUT records are not hash-bound

- Location: `scripts/infinite_mobility_table123_common.py:119-153`, especially `:143`; retained row fields at `:243-246`.
- Evidence: `_contained(repo_root, recovery_record, ...)` proves only repository containment. It does not require the record to be under the supplied `recovery_root`, nor at `cases/<factory>/seed_<seed>/record.json`. A recovery manifest can therefore reference any repository JSON object with a matching identity/status/hash. The row copies the recovery case but records only the original status string; it does not bind the corresponding original TIMEOUT record or the primary records file, so the preserved provenance is not independently auditable.
- Fix: resolve the manifest path relative to a documented base, require it to equal (or be contained beneath) the expected recovery case path under `recovery_root`, reject symlink components, and hash-bind the recovery manifest/summary and original records source. Store an original-record content hash (and stable source locator) plus the verified recovery record hash/path in each recovery row. Cross-check elapsed time/status/package hash fields against both source records.

### 6. Package-root symlinks and source-root escapes are accepted because symlink checks occur after `resolve()`

- Location: `scripts/infinite_mobility_table123_common.py:157-176`, especially `:164-170`; evaluation mirror at `scripts/run_table1_infinite_mobility.py:53-64`.
- Evidence: both functions overwrite the raw package path with `resolve(strict=True)` and then call `is_symlink()` on the resolved target, which cannot detect that the original package leaf was a symlink. `_selected_package()` also never proves that the resolved package remains beneath `source_root`. Nested entries are checked by the Table 2 package binder, but the package root and its path components are not.
- Fix: retain raw and resolved paths separately; reject a symlink at the package leaf and every component from the trusted source root using no-follow checks; require the resolved package to be under the resolved source root and at the exact expected case path. Apply the same no-follow path validation during evaluation before opening any asset.

### 7. Every large package is fully hashed repeatedly, including twice before evaluation and twice again in the worker

- Location: preparation at `scripts/infinite_mobility_table123_common.py:232-250`; evaluation at `scripts/run_table1_infinite_mobility.py:65-70`, `:116-123`.
- Evidence: preparation calls `baseline_package_sha256()` and then `package_binding()`, each reading every regular file. A successful evaluation repeats both full scans in the preflight loop and then repeats both in `_evaluate_row()`: four full-package scans per formal run before the shared fingerprint reads its referenced closure. This is unsuitable for large packages and adds avoidable I/O for all 720 assets.
- Fix: perform one no-follow file-manifest scan per package, retain per-file hashes, and derive both the Table 2 binding and legacy baseline digest from that single manifest. Remove the duplicate preflight. During evaluation, compute one current binding per row, compare it once, and reuse its URDF file digest; then run the shared evaluator. Add call-count/read-byte tests that fail if a package binding is recomputed for the same row.

### 8. Publication is not concurrency-safe and can overwrite an output directory created after the early existence check

- Location: cohort publication at `scripts/infinite_mobility_table123_common.py:297-320`; Table 1 publication at `scripts/run_table1_infinite_mobility.py:151-152`, `:171-190`.
- Evidence: overwrite refusal is checked before long hashing/evaluation, with no output lock or atomic no-replace reservation. `Path.replace()` can replace an existing empty destination directory, so an output appearing between the check and publish is not unconditionally refused. Staged artifacts are also not independently verified against `artifact_manifest.json` before rename.
- Fix: acquire an exclusive output lock/reservation before work, revalidate at publish time, and use a no-replace publication strategy (or immutable version directory plus atomically created pointer) that never replaces any pre-existing path. Verify every staged artifact's byte count/hash and manifest self-hash, fsync files/directories as required, then publish. Add concurrent-writer and late-created-output tests for both cohort and Table 1 paths.

### 9. Tests do not exercise the formal/output contract and assert the wrong failure behavior

- Location: `tests/test_infinite_mobility_table123.py:101-180`.
- Evidence: all five tests use direct helper calls; none calls `publish_cohort()` or `run()`. There is no test for the exact 720 identity/provenance split, source hashes, recovery path redirection, package-root symlinks, manifest/run self-hash, real protocol byte snapshot, full report, artifact verification, overwrite races, atomic cleanup, failed-record denominator behavior, or hash call counts. The only drift test expects global abort.
- Fix: replace the drift assertion with fail-closed record/denominator assertions and add end-to-end small-fixture publication tests plus a synthetic 20 x 36 formal manifest test. Mock only expensive evaluator operations, not binding/path/output behavior; compare real artifact bytes and hashes.

## Minor

### 10. Cohort `protocol_snapshot.json` is only a source-selection dictionary and is mislabeled as a protocol snapshot

- Location: `scripts/infinite_mobility_table123_common.py:311-317`.
- Evidence: the file contains only `manifest["source_selection"]`; it does not snapshot the generation/freeze protocol, source manifest protocol, source hashes, identity rules, or recovery overlay rules. This makes the cohort artifact set appear more complete than it is.
- Fix: either rename it to `source_selection.json`, or emit a real canonical cohort protocol snapshot containing the primary protocol, exact identity matrix policy, recovery overlay contract, source bindings, and preparation implementation binding; include and verify it in the artifact manifest.

## Verification performed

`PYTHONDONTWRITEBYTECODE=1 pytest -q -p no:cacheprovider tests/test_infinite_mobility_table123.py` completed with **5 passed in 14.41s**. No real cohort preparation or formal evaluation was started.

## Actual formal publication acceptance (2026-08-25)

Verdict: **ACCEPTED - FINAL READY FOR RELEASE.** The canonical publication at
`runtime/table1_infinite_mobility_720` passed an independent read-only
acceptance after the formal single-worker run completed in the persistent
`tmux` session. No asset was regenerated or replaced during acceptance.

The output contains exactly the five declared artifacts plus
`artifact_manifest.json`; every byte count and SHA-256 receipt matches. The
formal run manifest self-hash, exact 20-factory x 36-seed order, 713 original
`PASS` rows, and seven approved `TIMEOUT` recovery identities all match the
frozen cohort. All 720 records echo the frozen source rows and are
`COMPLETED` with `error = null`, XML parse/tree/fingerprint coverage complete,
and internally consistent package bindings.

Independent recomputation produced:

```text
N_release = N_eval = 720; raw factories = 20 / 20
Links/Asset = 15.0416666667 / 8 / 41
Movable Joints/Asset = 6.5597222222 / 3 / 16
Multi-joint Assets = 550 / 720 (76.3888888889%)
Unique Topologies = 157 / 720 (21.8055555556%; coverage 720 / 720)
Exact Duplicate Rate = 0 / 720 (0.0%; coverage 720 / 720)
Factory macro = multi-joint 76.3888888889%; topology 22.0833333333%; duplicate 0.0%
```

Formal artifact hashes are:

```text
manifest.json          ef1c4355129063d923c799486e6abe3b41add479fddf492b30d0adee00332403
asset_records.jsonl    6ad769c42657898ff5befdf054140c8b13f58f39c783ba649feb8532c0eb17cc
summary.json           f7e047d0d92b4aec81a71d0d451e085654a4c0dbe23436ac8b8d784d1cfd3501
report.md              23ec1807f7fe072443a558446ce4f053662f6792f743bcba808e7d43cb2ad132
artifact_manifest.json c56bb280fb3fb3c4b9322961313fa831094810aecdce02c34e0797a150cfc425
protocol_snapshot.md   b05a4edbe61037f2dc4bc1bc1580e66f13f852fc24a3b979e130e8a7aa30ef00
```

The protocol snapshot is intentionally the pre-writeback protocol used by the
formal run. The live document now contains the supplementary Infinite Mobility
rows and therefore has a different current hash; a future formal rerun must
freeze a new cohort/protocol binding rather than silently reusing this receipt.
