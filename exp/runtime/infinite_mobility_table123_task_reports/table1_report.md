# Infinite Mobility Table 1 Task Report

## Scope

Implemented the shared supplementary full generated cohort freezer and the
Infinite Mobility Table 1 runner. The cohort is explicitly labeled as a
supplementary full generated cohort, not an official finite release.

The freezer retains every factory/seed identity in source-manifest order. PASS
packages come from `runtime/infinite_mobility_v1`; original TIMEOUT identities
come only from recovery cases enumerated in `recovery_manifest.json`. It checks
the individual recovery record SHA-256 before loading it, cross-checks the
recovery summary, preserves `original_status=TIMEOUT` and `recovery_used=true`,
uses each selected record's `validation.urdf_path`, and refuses path escapes,
symlinks, missing files, duplicate identities, source binding drift, and missing
recoveries.

## TDD Evidence

RED command:

```bash
../arti-template/.venv/bin/python -m pytest -q tests/test_infinite_mobility_table123.py
```

RED result: `5 failed in 0.18s`; each failure was a `FileNotFoundError` for the
new shared freezer or Table 1 runner, confirming the requested behavior did not
yet exist.

GREEN command:

```bash
../arti-template/.venv/bin/python -m pytest -q tests/test_infinite_mobility_table123.py
```

GREEN result: `5 passed in 1.78s`.

Fixture smoke command:

```bash
../arti-template/.venv/bin/python -m pytest -q \
  tests/test_infinite_mobility_table123.py::test_table1_analyzes_nested_seed_package_and_rejects_manifest_drift
```

Fixture smoke result: `1 passed in 2.85s`. It exercises an actual nested
`<seed>/scene.urdf` package with an `objs/` relative resource, then modifies the
URDF and confirms freeze-binding drift is rejected.

## Environment

Used `../arti-template/.venv/bin/python`, the historical frozen Table 2
environment. `exp/.venv_low_medium` was not used because it lacks `pygltflib`;
the selected environment provides the Table 2 binder dependencies.

## Non-Run

Started a read-only all-source freezer preflight but stopped it on instruction
before completion. It published no cohort and ran no Table 1 evaluation. No
formal 720-asset preparation or evaluation was started.

## Second Review Fixes

The freezer now performs one no-follow package scan per asset and derives both
the Table 2-compatible package binding and the legacy baseline package hash from
that scan. Frozen rows additionally carry `selection_index`, `source`, and an
XML-derived `declared_joint_count_hint`. Formal validation requires the fixed
20-factory x 36-seed matrix, matching identity fields, 713 original PASS rows,
the seven approved original TIMEOUT identities, and complete original/recovery
record provenance.

Source files, freezer/preparer, Table 1 runner/evaluator, and the live protocol
are SHA-256 bound in the cohort/run manifests. Both manifests self-hash. The
Table 1 runner copies the exact bytes of
`URDF-Sim-Ready-Automatic-Evaluation.md` into `protocol_snapshot.md`, verifies
the copy, and renders the full Table 1 metric/coverage/status report with the
supplementary-cohort qualification.

Asset-local path, hash, binding, parse, and fingerprint errors now become
`FAILED` records. They remain in the frozen `N_eval` denominator. Global
cohort/source/evaluator invariants remain fatal before evaluation. Staged output
is checked against an artifact manifest and is published under an exclusive
output lock as a newly reserved real directory; any existing output is refused.

Second-round RED command:

```bash
PYTHONDONTWRITEBYTECODE=1 ../arti-template/.venv/bin/python -m pytest -q \
  -p no:cacheprovider tests/test_infinite_mobility_table123.py
```

Second-round RED result: `4 failed, 4 passed in 3.05s`. The failures showed the
old global drift abort, redirected recovery acceptance, missing selection
artifact, and absent formal constants/API. A later targeted RED also showed
missing `source`/`evaluation` bindings, and the report regression showed the
incorrect Artiverse `5,402`/salted-selection text.

Second-round GREEN command:

```bash
PYTHONDONTWRITEBYTECODE=1 ../arti-template/.venv/bin/python -m pytest -q \
  -p no:cacheprovider tests/test_infinite_mobility_table123.py
```

Second-round GREEN result: `8 passed in 0.22s`.

Final publication regression added after the initial GREEN verifies Linux
`renameat2(RENAME_NOREPLACE)` directory publication: the completed staging
directory is atomically made visible only when the target is absent, and an
existing target is refused. Final command/result:

```bash
PYTHONDONTWRITEBYTECODE=1 ../arti-template/.venv/bin/python -m pytest -q \
  -p no:cacheprovider tests/test_infinite_mobility_table123.py
```

`9 passed in 0.23s`.

Latest final verification after the report update: syntax check exit 0 and
`9 passed in 0.17s`.

Syntax check:

```bash
PYTHONDONTWRITEBYTECODE=1 ../arti-template/.venv/bin/python -m py_compile \
  scripts/infinite_mobility_table123_common.py \
  scripts/prepare_infinite_mobility_table123_cohort.py \
  scripts/run_table1_infinite_mobility.py
```

Syntax check result: exit 0.

No real 720-asset cohort preparation or formal Table 1 evaluation was run in
this second round.

## CPFS Follow-up Fixes

The legacy package digest now sorts the complete file manifest by each item's
global relative `path` before deriving the baseline hash. The Table 2 binding
file list remains in its existing traversal/protocol order. A regression fixture
with both package-root and nested files binds a non-empty recorded
`package_sha256` and verifies the sorted derivation.

CPFS does not support the prior `renameat2(RENAME_NOREPLACE)` call (it returned
`EINVAL`, errno 22). Publication now reserves the real output directory with
atomic `mkdir`, records its device/inode, moves staged children with ordinary
same-filesystem `rename`, and checks the reservation inode before and after each
move. A late-created target is refused and an altered reservation is never
removed. The published default path remains a real, directly consumable
directory.

Follow-up RED command (before these fixes):

```bash
PYTHONDONTWRITEBYTECODE=1 ../arti-template/.venv/bin/python -m pytest -q \
  -p no:cacheprovider \
  tests/test_infinite_mobility_table123.py::test_legacy_package_hash_uses_global_relative_path_order \
  tests/test_infinite_mobility_table123.py::test_runtime_mount_publish_run_and_late_target_are_no_replace_safe
```

RED result: `2 failed`. The digest test reported the recorded/derived package
hash mismatch; the runtime-mounted publish failed with `OSError: [Errno 22]
Invalid argument` from `renameat2`.

Follow-up GREEN and final target-file command:

```bash
PYTHONDONTWRITEBYTECODE=1 ../arti-template/.venv/bin/python -m pytest -q \
  -p no:cacheprovider tests/test_infinite_mobility_table123.py
```

Result: `11 passed in 0.58s`.

The runtime-mounted end-to-end publish/run and late-target test is included in
that result. Syntax verification also passed with exit 0:

```bash
PYTHONDONTWRITEBYTECODE=1 ../arti-template/.venv/bin/python -m py_compile \
  scripts/infinite_mobility_table123_common.py \
  scripts/prepare_infinite_mobility_table123_cohort.py \
  scripts/run_table1_infinite_mobility.py
```

No formal 720 preparation or evaluation was started.

An additional inode-replacement regression was then added. Its RED command was:

```bash
PYTHONDONTWRITEBYTECODE=1 ../arti-template/.venv/bin/python -m pytest -q \
  -p no:cacheprovider \
  tests/test_infinite_mobility_table123.py::test_publish_detects_replaced_reservation_without_deleting_new_target
```

RED result: `1 failed in 0.10s`; the prior cleanup branch silently returned
after an inode change. The final current verification command was:

```bash
PYTHONDONTWRITEBYTECODE=1 ../arti-template/.venv/bin/python -m pytest -q \
  -p no:cacheprovider tests/test_infinite_mobility_table123.py
PYTHONDONTWRITEBYTECODE=1 ../arti-template/.venv/bin/python -m py_compile \
  scripts/infinite_mobility_table123_common.py \
  scripts/prepare_infinite_mobility_table123_cohort.py \
  scripts/run_table1_infinite_mobility.py
```

Final result: `12 passed in 0.62s`; syntax check exit 0. A final fresh target
test run after this report update also completed with `12 passed in 0.79s`.

One read-only actual-package spot check also completed without starting a cohort
or evaluation: `OvenFactory/seed_000` recorded and sorted-derived legacy hash
both equal `eeb0c6533bc488dd99ca1e73c8ad8cda9794bb4d22797a68464ff66ec089575f`.

## Formal 720-asset run completion (2026-08-25)

The canonical single-worker formal run completed in the persistent tmux
watcher and published `runtime/table1_infinite_mobility_720`. Independent
read-only acceptance confirmed the exact 20-factory x 36-seed matrix, 713
original PASS rows plus 7 identity-preserving recovery overlays, 720 completed
records, artifact closure, manifest self-hash, and independent summary
recomputation. The formal result is accepted for publication; see
`table1_review.md` for the detailed acceptance receipt and hashes.
