# Table 1/2/3 Full-Release Evaluation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Freeze and evaluate the complete local release roster for the seven named finite datasets plus Infinite Mobility and Infinigen-Sim in Tables 1, 2, and 3, without changing the historical N=800 receipts or running Ours.

**Architecture:** Add a shared full-release roster contract and dataset-specific discovery adapters. Three new full-release runners consume the same ordered roster and reuse the existing structural, URDF-audit, and FK metric cores; they publish independent, resumable receipts under a new runtime namespace. The Markdown protocol is updated only after all receipts pass independent artifact and reaggregation checks.

**Tech Stack:** Python 3.12; `arti-template/.venv` for Table 2; `exp/.venv_low_medium` for Tables 1 and 3; pytest; existing `run_table1_artiverse.py`, `run_table2_urdf_articraft.py`, and `run_urdf_table3_lam.py` cores; JSON/JSONL and SHA-256 artifact receipts.

**Spec:** `docs/superpowers/specs/2026-08-25-table123-full-release-design.md`

## Global Constraints

- Ours-500K and PV-A are out of scope; do not launch or alter their runners.
- Full-release rosters retain every locally available source-bound asset and every failure in the denominator.
- LAM source root is exactly `exp/Articulated-Object-Code`; all manifest tiers, including `broken`, are included.
- Infinite Mobility is the existing 720-asset local operational universe, not an official finite release.
- Infinigen-Sim is the 17-archive URDF release at revision `2dea6d2ca7a7f99d273e9e7437de5caaee261c24`.
- New output root is `exp/runtime/table123_full_release_20260825`; never overwrite historical N=800 directories.
- All writes use atomic publication and all published files are bound by an artifact manifest.
- No destructive cleanup, source archive edits, or termination of unrelated user processes.

### Task 1: Shared Full-Release Roster Contract

**Files:**
- Create: `exp/scripts/table123_full_release_common.py`
- Test: `exp/tests/test_table123_full_release_common.py`

**Interfaces:**
- `canonical_sha256(value: Any) -> str`
- `sha256_file(path: Path) -> str`
- `freeze_roster(rows: Iterable[dict[str, Any]], output: Path, *, dataset: str, source_bindings: list[dict[str, str]]) -> dict[str, Any]`
- `load_roster(path: Path, *, expected_dataset: str | None = None) -> dict[str, Any]`
- `verify_roster(path: Path) -> dict[str, Any]`
- `write_checkpoint(path: Path, payload: dict[str, Any]) -> None`
- `verify_artifacts(output: Path) -> None`

- [x] **Step 1: Write failing tests** for deterministic ordering, duplicate IDs, path escape/symlink rejection, primary URDF hash drift, dynamic `N_eval`/`J_eval`, and canonical self-hash.
- [x] **Step 2: Run the focused tests and verify they fail** because the new module and contract are absent.
- [x] **Step 3: Implement the minimal schema and verification functions** using canonical JSON (`sort_keys=True`, compact separators, `ensure_ascii=True`) and atomic writes.
- [x] **Step 4: Run the focused tests and verify they pass**; then run existing receipt-verification tests as a regression check.

### Task 2: Dataset Roster Builders And Infinigen Extraction

**Files:**
- Create: `exp/scripts/build_table123_full_release_rosters.py`
- Create: `exp/scripts/extract_infinigen_sim_full_release.py`
- Test: `exp/tests/test_table123_full_release_rosters.py`
- Test: `exp/tests/test_infinigen_sim_full_release_extraction.py`

**Interfaces:**
- `build_roster(dataset: str, *, source_root: Path, output: Path) -> Path`
- `discover_articraft(source_root: Path) -> list[dict[str, Any]]`
- `discover_lam(source_root: Path) -> list[dict[str, Any]]`
- `discover_artiverse(source_root: Path) -> list[dict[str, Any]]`
- `discover_partnet(source_root: Path) -> list[dict[str, Any]]`
- `discover_physx(source_root: Path) -> list[dict[str, Any]]`
- `discover_sketch(source_root: Path) -> list[dict[str, Any]]`
- `discover_infinite(cohort_manifest: Path) -> list[dict[str, Any]]`
- `extract_archives_securely(archive_root: Path, destination: Path) -> dict[str, Any]`
- `discover_infinigen(source_root: Path) -> list[dict[str, Any]]`

- [x] **Step 1: Write failing fixture tests** covering all eight source layouts, LAM broken-tier retention, Artiverse malformed XML retention, PhysX exact raw labels, Infinite 720 identity order, and Infinigen traversal/link rejection.
- [x] **Step 2: Run the focused tests and verify the expected failures.**
- [x] **Step 3: Implement source-specific discovery** with exact release counts: 9,996; 3,217; 3,544; 2,347; 2,024; 4,956; 720; and 8,226.
- [x] **Step 4: Implement secure Infinigen extraction** into `exp/runtime/table123_full_release_20260825/inputs/infinigen_sim`, validate all 17 LFS archive hashes, then freeze package bindings.
- [x] **Step 5: Run focused tests and a read-only roster census**; abort if any expected count or source hash differs.

### Task 3: Full-Release Table 1 Runner

**Files:**
- Create: `exp/scripts/run_table1_full_release.py`
- Test: `exp/tests/test_run_table1_full_release.py`

**Interfaces:**
- `evaluate_row(row: dict[str, Any]) -> dict[str, Any]`
- `aggregate_full_release(records: Iterable[dict[str, Any]], roster: dict[str, Any]) -> dict[str, Any]`
- `run_full_release(roster_path: Path, output: Path, *, workers: int) -> Path`

- [x] **Step 1: Write failing tests** proving the runner accepts all roster rows, preserves failed rows, computes dynamic denominators, and rejects an old N=800-only manifest.
- [x] **Step 2: Run focused tests and verify RED.**
- [x] **Step 3: Implement the runner** by reusing `analyze_urdf`, `fingerprint_package`, and the shared aggregation protocol; add deterministic shards and resumable checkpoints.
- [x] **Step 4: Run fixture smoke tests** with one valid, one malformed, and one resource-failing package; verify artifact closure.
- [x] **Step 5: Run Table 1 for all eight full rosters** with bounded worker counts and publish only complete receipts.

### Task 4: Full-Release Table 2 Runner

**Files:**
- Create: `exp/scripts/run_table2_full_release.py`
- Test: `exp/tests/test_run_table2_full_release.py`

**Interfaces:**
- `audit_row(row: dict[str, Any], *, run_standard_parser: bool) -> dict[str, Any]`
- `aggregate_full_release(records: Iterable[dict[str, Any]], roster: dict[str, Any]) -> dict[str, Any]`
- `run_full_release(roster_path: Path, output: Path, *, workers: int, timeout_seconds: float) -> Path`

- [x] **Step 1: Write failing tests** for dynamic asset/joint denominators, child timeout/error records, package drift fail-closed behavior, and preservation of zero-joint assets.
- [x] **Step 2: Run focused tests and verify RED.**
- [x] **Step 3: Implement fresh-child execution** around `audit_asset_package`, with frozen BLAS variables, bounded process groups, deterministic run tokens, and checkpoint resume.
- [x] **Step 4: Run fixture smoke tests** in the pinned Table 2 environment and independently reaggregate all nine metrics.
- [x] **Step 5: Run full-release Table 2 in dataset-sized shards**, beginning with the smallest rosters and publishing each dataset only after artifact verification.

### Task 5: Full-Release Table 3 Runner

**Files:**
- Create: `exp/scripts/run_table3_full_release.py`
- Test: `exp/tests/test_run_table3_full_release.py`

**Interfaces:**
- `evaluate_row(row: dict[str, Any], *, samples: int = 21) -> dict[str, Any]`
- `aggregate_full_release(records: Iterable[dict[str, Any]], roster: dict[str, Any]) -> dict[str, Any]`
- `run_full_release(roster_path: Path, output: Path, *, workers: int, timeout_seconds: float, samples: int = 21) -> Path`

- [x] **Step 1: Write failing tests** for dynamic `J_eval`, unsupported joint fail-closed records, zero-joint assets, exact K=21 states, and resume order.
- [x] **Step 2: Run focused tests and verify RED.**
- [x] **Step 3: Implement the runner** around `run_urdf_table3_lam.evaluate_urdf`, removing old formal N/J/category constants from the new adapter while retaining the shared FK semantics.
- [x] **Step 4: Run fixture smoke tests** for revolute, prismatic, continuous, malformed, floating, and zero-joint cases.
- [x] **Step 5: Run full-release Table 3 for all eight rosters**, checkpointing by deterministic row ranges and publishing complete artifact manifests.

### Task 6: Independent Receipt Verification And Markdown Update

**Files:**
- Create: `exp/scripts/verify_table123_full_release.py`
- Test: `exp/tests/test_verify_table123_full_release.py`
- Modify: `exp/URDF-Sim-Ready-Automatic-Evaluation.md` Table 1, Table 2, and Table 3 sections plus their evidence paragraphs.

**Interfaces:**
- `verify_dataset_receipts(dataset_output: Path) -> dict[str, Any]`
- `reaggregate_table(table_output: Path) -> dict[str, Any]`
- `render_full_release_rows(results: dict[str, Any]) -> str`

- [x] **Step 1: Write failing verifier tests** for artifact size/hash drift, record-count mismatch, summary mismatch, and stale protocol snapshots.
- [x] **Step 2: Run focused tests and verify RED.**
- [x] **Step 3: Implement independent verification** without importing the production aggregation functions for the final comparison.
- [x] **Step 4: Verify all 24 new table receipts** (eight datasets × three tables), including source bindings and dynamic denominators.
- [x] **Step 5: Update the Markdown rows and evidence text** from verified summaries; retain historical N=800 values in an appendix and explicitly exclude Ours.
- [x] **Step 6: Run the complete Table 1/2/3 contract suite plus the new full-release tests** and perform a final artifact closure check before claiming completion.
- [x] **Post-plan acceptance gate:** add `exp/scripts/check_table123_full_release.py` as a read-only, fail-closed recheck for all eight rosters, 24 receipts, Markdown denominators/metrics, and frozen Ours/Brain row fingerprints; `--pytest` runs the focused contract suite without launching a full-release evaluator.

## Execution Order

Tasks 1 and 2 are prerequisites. Tasks 3, 4, and 5 share only the frozen
rosters and can run in parallel by dataset/table after smoke tests pass. Task 6
is last and cannot begin until all 24 receipts are complete and independently
verified.

## Capacity Estimates

The finite full-release panel contains 26,084 assets excluding Infinigen and
Infinite (and excluding Ours); Table 3 is expected to cover roughly 90,000
movable joints and about 1.9 million K=21 states. Infinigen adds 8,226 assets
and 31,975 joints. Runs
must be resumable and may take many hours; no partial result is written into a
headline table.
