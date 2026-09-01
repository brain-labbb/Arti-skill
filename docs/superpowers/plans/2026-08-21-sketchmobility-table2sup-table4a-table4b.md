# SketchMobility Table 2 Supplementary, Table 4a, and Table 4b Plan

> **Execution rule:** Follow test-driven development for every behavior change,
> and use fresh independent verification before publishing any result.

**Goal:** Evaluate and backfill the three pending SketchMobility rows using the
exact frozen N=800 cohort.

**Architecture:** Three small SketchMobility adapters bind the existing formal
Table 2/3/4 receipts to the established static, Genesis, and exact-geometry
evaluators. Each emits a write-once resumable receipt and is checked by a
standalone verifier before document publication.

**Spec:** `docs/superpowers/specs/2026-08-21-sketchmobility-table2sup-table4a-table4b-design.md`

## Global Frozen Inputs

- Table 2: `exp/runtime/table2_urdf_sketch_mobility_table1cohort_n800_20260821T035015Z`
- Table 3: `exp/runtime/urdf_table3_sketch_mobility_table1cohort_n800_20260821T062050Z`
- Table 4: `exp/runtime/urdf_table4_sketch_mobility_table1cohort_n800_20260821T090554Z`
- Dataset root: `exp/SketchMobility`
- Ordered asset IDs SHA256:
  `a88506e1da8e7e8b61a740965dea2faba4e9ab8280f47417e17550024b6dde17`
- N=800, J=1,824; no resampling, replacement, filtering, or reordering.

## Task 1: Shared SketchMobility Receipt Contracts

**Files:**

- Create: `exp/scripts/sketchmobility_supplementary_common.py`
- Create: `exp/scripts/test_sketchmobility_supplementary_common.py`

- [ ] Write tests that reject changed cohort order, identity, URDF bytes,
  recursive package bytes, unsafe paths/symlinks, and mismatched upstream rows.
- [ ] Run the focused test and observe RED because the common contract is absent.
- [ ] Implement frozen input loading, canonical hashing, package closure,
  write-once output creation, atomic rank journals, strict resume validation,
  source snapshots, artifact manifests, and whole-tree receipt digesting.
- [ ] Run the focused test and observe GREEN.

## Task 2: Table 2 Supplementary Adapter

**Files:**

- Create: `exp/scripts/run_table2sup_urdf_sketch_mobility.py`
- Create: `exp/scripts/verify_table2sup_urdf_sketch_mobility.py`
- Create: `exp/scripts/test_run_table2sup_urdf_sketch_mobility.py`

- [ ] Write RED tests for N=800/J=1,824 intent, static metric semantics,
  empty placeholder registry rendering, fail-closed child records, aggregate
  recomputation, live-source drift rejection, and verifier tamper detection.
- [ ] Implement the minimal adapter around `lam_supplementary_static`, with a
  fresh child per asset and exact current evaluator source/version pins.
- [ ] Implement an import-independent verifier that recomputes upstream and
  package bindings, all aggregates, artifact closure, and formal configuration.
- [ ] Run the full Table 2 supplementary suite GREEN, then syntax and diff checks.
- [ ] Obtain independent review and resolve all Critical/Important findings.
- [ ] Run fixed N=5 smoke, replay its frozen verifier, and record its immutable
  path in the formal manifest.
- [ ] Run formal N=800, replay the published verifier read-only, and confirm the
  whole receipt digest is unchanged.
- [ ] Backfill only the Table 2 supplementary SketchMobility row and evidence
  text, then verify every value/hash against persisted bytes.

## Task 3: Table 4a Adapter

**Files:**

- Create: `exp/scripts/run_table4a_urdf_sketch_mobility.py`
- Create: `exp/scripts/verify_table4a_urdf_sketch_mobility.py`
- Create: `exp/scripts/test_run_table4a_urdf_sketch_mobility.py`

- [ ] Write RED tests for Table 2 fail-closed pre-gating, Table 3 joint joins,
  Table 4 K=21 state identity, missing-reference regeneration, intended
  denominators, DoF bins, journal/resume, write-once publication, smoke-gate
  replay, and semantic tamper rejection.
- [ ] Implement the minimal Genesis adapter using the exact existing collision
  oracle and frozen 16-worker/3,600-second/1.5-second launch configuration.
- [ ] Emit 489 Table 2 collision-incomplete terminal records without Genesis,
  while keeping their assets, joints, and states in every formal denominator.
- [ ] Implement the standalone verifier, including positional identity joins,
  state-hash checks, aggregate recomputation, historical strict bin values,
  source snapshots, environment pins, and artifact closure.
- [ ] Run focused real-Genesis integration tests and the full suite GREEN, then
  syntax and diff checks.
- [ ] Obtain independent review and resolve all Critical/Important findings.
- [ ] Run fixed N=5 smoke and frozen-verifier replay.
- [ ] Run formal N=800 with strict resume available; monitor until terminal.
- [ ] Replay the published verifier read-only and prove receipt bytes unchanged.
- [ ] Backfill only the Table 4a SketchMobility row, its declared-DoF bin table,
  and evidence paragraph; verify all values/hashes against receipt bytes.

## Task 4: Table 4b Adapter

**Files:**

- Create: `exp/scripts/run_table4b_urdf_sketch_mobility.py`
- Create: `exp/scripts/verify_table4b_urdf_sketch_mobility.py`
- Create: `exp/scripts/test_run_table4b_urdf_sketch_mobility.py`

- [ ] Write RED tests for exact cohort/package binding, all seven metrics,
  `PARTIAL`/`N/E` denominators, timing protocol, fail-closed child records,
  journal/resume, source snapshots, write-once publication, smoke gating, and
  semantic tamper rejection.
- [ ] Implement the minimal adapter around `lam_supplementary_geometry` and the
  established isolated one-thread timing harness, frozen at 16 workers and a
  900-second per-asset timeout.
- [ ] Implement the standalone verifier to recompute all aggregates and validate
  cohort identity, environment/source pins, artifact closure, and output hashes.
- [ ] Run focused exact-geometry integration tests and the full suite GREEN, then
  syntax and diff checks.
- [ ] Obtain independent review and resolve all Critical/Important findings.
- [ ] Run fixed N=5 smoke and frozen-verifier replay.
- [ ] Run formal N=800, replay the published verifier read-only, and prove the
  receipt digest is unchanged.
- [ ] Backfill only the Table 4b SketchMobility row and evidence paragraph, then
  verify every value/hash against persisted bytes.

## Task 5: Final Cross-Table Audit

- [ ] Re-run all new unit/integration suites and compile the six runner/verifier
  entry points.
- [ ] Verify all three formal receipts report the same ordered N=800 asset IDs
  hash and package/URDF identities.
- [ ] Recompute the three Markdown rows directly from each `summary.json` and
  confirm all evidence links resolve within the immutable receipt directories.
- [ ] Confirm no unrelated `TBD` row or existing result was modified.
- [ ] Run `git diff --check`, inspect the scoped diff, and update durable session
  state with commands, receipt paths, metrics, hashes, and remaining risks.
