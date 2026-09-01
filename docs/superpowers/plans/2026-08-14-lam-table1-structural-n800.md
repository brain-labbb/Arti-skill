# LAM Table 1 Structural N=800 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Evaluate the exact frozen LAM N=800 cohort with the shared Table 1 structural metrics and write the verified result into the LAM row.

**Architecture:** A thin LAM adapter validates the completed Table 3 manifest and JSONL receipt, reconstructs manifest rank order, binds every selected package before and after evaluation, and calls the existing shared Table 1 analyzer, fingerprint, and aggregation functions. It publishes a sealed output directory atomically and keeps every frozen asset in all applicable denominators.

**Tech Stack:** Python 3.12, standard library, pytest, existing `run_table1_artiverse.py` shared evaluator.

**Spec:** `exp/URDF-Sim-Ready-Automatic-Evaluation.md` Table 1 plus `exp/runtime/urdf_table3_lam_n800_seed20260813_20260814T022400Z_v3/manifest.json`

## Global Constraints

- Use exactly 800 frozen records and ranks from the supplied runtime directory; never redraw or replace an asset.
- Treat `manifest.json` as the ordering authority and `asset_records.jsonl` as keyed completion evidence.
- Use only `<source_root>/<rel_path>/generated.urdf` as the primary URDF.
- Preserve the two prior Table 3 errors as source metadata; Table 1 determines its own structural status.
- Bind all regular files in each selected package before evaluation and verify the same binding afterward.
- Use explicit release and evaluation categories from `manifest.csv`: 787 and 305.
- Do not modify shared metric definitions or unrelated experiment files.
- Do not create a git commit unless separately requested.

---

### Task 1: Frozen cohort adapter

**Files:**

- Create: `exp/tests/test_run_table1_lam.py`
- Create: `exp/scripts/run_table1_lam.py`

**Interfaces:**

- Consumes: Table 3 `manifest.json`, `asset_records.jsonl`, release `manifest.csv`, `dataset_api.json`, and `released_outputs`.
- Produces: `load_frozen_cohort(records_path: Path, *, dataset_root: Path, expected_n: int, formal: bool) -> dict[str, Any]`.

- [x] **Step 1: Write failing loader tests**

```python
def test_loader_restores_manifest_rank_order(tmp_path):
    fixture = make_frozen_fixture(tmp_path, jsonl_order=(2, 1))
    cohort = runner.load_frozen_cohort(
        fixture.records, dataset_root=fixture.root, expected_n=2, formal=False
    )
    assert [row["selection_rank"] for row in cohort["assets"]] == [1, 2]

def test_loader_rejects_missing_or_mismatched_completion_record(tmp_path):
    fixture = make_frozen_fixture(tmp_path, omit_rank=2)
    with pytest.raises(ValueError, match="completion records"):
        runner.load_frozen_cohort(
            fixture.records, dataset_root=fixture.root, expected_n=2, formal=False
        )
```

- [x] **Step 2: Run the focused tests and verify the missing runner causes RED**

Run: `exp/.venv_low_medium/bin/python -m pytest -q exp/tests/test_run_table1_lam.py -k 'loader'`

Expected: collection failure because `exp/scripts/run_table1_lam.py` does not exist.

- [x] **Step 3: Implement the minimum validating loader**

The loader must verify manifest self-hash, unique ranks 1..N, selected-key hash, exact JSONL keyed join, primary URDF identity/hash, release CSV/API hashes, release size, candidate-pool hash, categories, and package containment. In formal mode it must also require the frozen N, seed, source revision, manifest content hash, and input JSONL file hash.

- [x] **Step 4: Add drift tests and make the loader suite GREEN**

Cover duplicate/missing JSONL keys, mismatched rank/hash/category/path, unsafe paths, wrong primary hash, source hash drift, and package symlinks.

Run: `exp/.venv_low_medium/bin/python -m pytest -q exp/tests/test_run_table1_lam.py -k 'loader or cohort or package'`

Expected: all selected tests pass.

### Task 2: Structural evaluation and sealed artifacts

**Files:**

- Modify: `exp/tests/test_run_table1_lam.py`
- Modify: `exp/scripts/run_table1_lam.py`

**Interfaces:**

- Consumes: ordered identities from `load_frozen_cohort()`.
- Produces: `evaluate_package(identity: dict[str, Any]) -> dict[str, Any]`, `aggregate_lam_records(...) -> dict[str, Any]`, and CLI output artifacts.

- [x] **Step 1: Write failing evaluation tests**

```python
def test_prior_table3_error_is_metadata_not_table1_failure(tmp_path):
    identity = make_identity(tmp_path, source_status="error")
    record = runner.evaluate_package(identity)
    assert record["source_table3_status"] == "error"
    assert record["parse_success"] is True
    assert record["status"].startswith("EVALUATED")

def test_package_mutation_fails_in_place(tmp_path):
    identity = make_identity(tmp_path)
    Path(identity["package"], "added.bin").write_bytes(b"drift")
    record = runner.evaluate_package(identity)
    assert record["status"] == "EVALUATION_FAILED"
    assert record["asset_key"] == identity["asset_key"]
```

- [x] **Step 2: Run the focused tests and verify RED**

Run: `exp/.venv_low_medium/bin/python -m pytest -q exp/tests/test_run_table1_lam.py -k 'table3_error or mutation or aggregate'`

Expected: failures because evaluation and aggregation are not implemented.

- [x] **Step 3: Implement evaluation by composing shared functions**

Call `analyze_urdf(generated.urdf)`, `fingerprint_package(generated.urdf)`, and `aggregate_records(...)`. Preserve source Table 3 fields separately, write ordered results with `executor.map`, and fail the same record in place if its package binding changes before or during evaluation.

- [x] **Step 4: Implement atomic publication and artifact sealing**

Write `manifest.json`, `asset_records.jsonl`, `summary.json`, `report.md`, and `artifact_manifest.json` in staging; verify the seal and atomically publish the completed directory with the shared output helpers.

- [x] **Step 5: Run the complete adapter test suite and make it GREEN**

Run: `exp/.venv_low_medium/bin/python -m pytest -q exp/tests/test_run_table1_lam.py`

Expected: all tests pass with no warnings or errors.

### Task 3: Formal N=800 run and Table 1 update

**Files:**

- Create: `exp/runtime/table1_lam_released_outputs/`
- Modify: `exp/URDF-Sim-Ready-Automatic-Evaluation.md`

**Interfaces:**

- Consumes: the tested adapter and exact supplied JSONL.
- Produces: sealed formal artifacts and one populated Table 1 row.

- [x] **Step 1: Run the formal evaluation**

```bash
exp/.venv_low_medium/bin/python exp/scripts/run_table1_lam.py \
  --dataset-root exp/Articulated-Object-Code \
  --input-records exp/runtime/urdf_table3_lam_n800_seed20260813_20260814T022400Z_v3/asset_records.jsonl \
  --output exp/runtime/table1_lam_released_outputs \
  --expected-n 800 --formal --workers 4
```

- [x] **Step 2: Independently verify the formal artifacts**

Check artifact hashes, 800 unique ordered identities, frozen source hashes, current package bindings, summary denominators, finite reported metrics, and rerun determinism of metric-bearing records/summary fields.

- [x] **Step 3: Write only the verified LAM values into Table 1**

Keep `Paper-reported Assets` as `N/R`; write `N_release=3,217`, `N_eval=800`, categories `787 / 305`, the three link/joint statistics, multi-joint rate, topology rate with denominator, and duplicate rate with denominator. Add one concise sentence identifying the frozen Table 3 cohort and incomplete-resource duplicate denominator.

- [x] **Step 4: Run final verification**

Run the full adapter tests, inspect the exact Markdown row, validate all artifact hashes, and request an independent read-only code/data review before reporting completion.
