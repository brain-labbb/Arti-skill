# Articraft Data Pipeline Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and package a clean, code-only Articraft data pipeline with batch preparation, model-backed generation, validation, and deterministic JSONL export.

**Architecture:** Add a small `pipeline` orchestration package inside `articraft_data` and keep the existing `agent`, `articraft`, `sdk`, `storage`, and `cli` packages as the compatibility implementation. Add a safe archive builder in `articraft_data/tools/build_zip.py` that copies an explicit source allowlist into `articraft_data_pipeline/`, excludes data and local state, and verifies the resulting archive.

**Tech Stack:** Python 3.11+, standard-library `csv`/`json`/`zipfile`, existing Articraft runtime, pytest, Ruff.

**Spec:** `docs/superpowers/specs/2026-08-25-articraft-data-pipeline-design.md`

## Global Constraints

- The archive root is `articraft_data_pipeline/`.
- Public stages are `prepare`, `generate`, `validate`, and `export`.
- Runtime data paths must be explicit; no API key or secret may be written to output files.
- The archive must exclude raw records, generated assets, caches, virtual environments, frontend files, historical scripts, and `.env` files.
- Tests use temporary directories and do not require network access or provider credentials.

---

### Task 1: Add The Public Pipeline Contracts

**Files:**
- Create: `articraft_data/pipeline/__init__.py`
- Create: `articraft_data/pipeline/config.py`
- Create: `articraft_data/pipeline/prepare.py`
- Create: `articraft_data/tests/pipeline/test_prepare.py`

**Interfaces:**
- Produces `BatchPreparationError`, `PreparedBatch`, and `prepare_batch(batch_path: Path, output_dir: Path) -> Path`.
- `prepare_batch` reads the canonical required columns, trims string fields, supplies stable `row_id` values when absent, rejects unsupported columns, rejects missing required values, rejects non-positive `max_turns`, rejects duplicate `row_id` values, and writes `output_dir/batch.csv` plus `output_dir/prepare_report.json`.

- [ ] **Step 1: Write the failing tests**

```python
def test_prepare_batch_normalizes_rows_and_writes_report(tmp_path):
    source = tmp_path / "input.csv"
    source.write_text(
        "category_slug,prompt,provider,model_id,thinking_level,max_turns\n"
        " lamp ,  Make a lamp  , openai , gpt-5 , high , 4\n",
        encoding="utf-8",
    )

    output = prepare_batch(source, tmp_path / "run")

    assert output.read_text(encoding="utf-8").splitlines()[1].startswith(
        "row_0001,lamp,Make a lamp,openai,gpt-5,high,4"
    )
    assert json.loads((tmp_path / "run" / "prepare_report.json").read_text())[
        "row_count"
    ] == 1


def test_prepare_batch_reports_row_number_for_invalid_turns(tmp_path):
    source = tmp_path / "input.csv"
    source.write_text(
        "category_slug,prompt,provider,model_id,thinking_level,max_turns\n"
        "lamp,Make a lamp,openai,gpt-5,high,0\n",
        encoding="utf-8",
    )

    with pytest.raises(BatchPreparationError, match="row 2.*max_turns"):
        prepare_batch(source, tmp_path / "run")
```

- [ ] **Step 2: Run the focused tests to verify they fail**

Run: `uv run --directory articraft_data pytest tests/pipeline/test_prepare.py -q`

Expected: FAIL because `pipeline.prepare` does not exist.

- [ ] **Step 3: Implement the minimal preparation contract**

Use `csv.DictReader`, preserve the input header order only for known fields, emit a fixed canonical header, and write JSON with `sort_keys=True`. Raise `BatchPreparationError` with the CSV line number in every validation error. Do not import the model runner from this module.

- [ ] **Step 4: Run the focused tests to verify they pass**

Run: `uv run --directory articraft_data pytest tests/pipeline/test_prepare.py -q`

Expected: PASS with two tests passing.

- [ ] **Step 5: Commit the pipeline contract**

```bash
git -C articraft_data add pipeline tests/pipeline
git -C articraft_data commit -m "feat: add batch preparation pipeline"
```

### Task 2: Add Validation And Deterministic Export

**Files:**
- Create: `articraft_data/pipeline/validate.py`
- Create: `articraft_data/pipeline/export.py`
- Create: `articraft_data/tests/pipeline/test_validate_export.py`

**Interfaces:**
- Produces `validate_repository(repo_root: Path, report_path: Path) -> bool`.
- Produces `export_dataset(repo_root: Path, output_path: Path) -> int`.
- Validation delegates to `storage.data_validation.validate_data_format` and writes counts plus all errors to the report.
- Export reads canonical dataset entries/index rows, sorts by `dataset_id` then `record_id`, writes compact JSONL, and returns the number of exported rows.

- [ ] **Step 1: Write failing tests with a synthetic repository fixture**

```python
def test_validate_repository_writes_machine_readable_success_report(tmp_path):
    repo = make_valid_fixture_repo(tmp_path / "repo")

    assert validate_repository(repo, tmp_path / "validation.json") is True
    report = json.loads((tmp_path / "validation.json").read_text())
    assert report["ok"] is True
    assert report["errors"] == []


def test_export_dataset_is_sorted_and_returns_count(tmp_path):
    repo = make_dataset_fixture_repo(tmp_path / "repo")
    output = tmp_path / "dataset.jsonl"

    assert export_dataset(repo, output) == 2
    rows = [json.loads(line) for line in output.read_text().splitlines()]
    assert [row["dataset_id"] for row in rows] == ["lamp_000001", "lamp_000002"]
```

- [ ] **Step 2: Run the focused tests to verify they fail**

Run: `uv run --directory articraft_data pytest tests/pipeline/test_validate_export.py -q`

Expected: FAIL because the new public functions do not exist.

- [ ] **Step 3: Implement validation and export**

Construct `StorageRepo(repo_root)`, call the existing validator without requiring hydrated records for the smoke fixture, serialize the full result counts and errors, and return `False` on any validation error. For export, prefer `records_index.jsonl` when present, fall back to dataset entry sidecars, omit rows missing `dataset_id`, and write one sorted JSON object per line with deterministic separators and key ordering.

- [ ] **Step 4: Run the focused tests to verify they pass**

Run: `uv run --directory articraft_data pytest tests/pipeline/test_validate_export.py -q`

Expected: PASS with all validation and export tests passing.

- [ ] **Step 5: Commit validation and export**

```bash
git -C articraft_data add pipeline tests/pipeline
git -C articraft_data commit -m "feat: add dataset validation and export"
```

### Task 3: Add Generation Wrapper And CLI

**Files:**
- Create: `articraft_data/pipeline/generate.py`
- Create: `articraft_data/pipeline/cli.py`
- Modify: `articraft_data/pipeline/__init__.py`
- Create: `articraft_data/tests/pipeline/test_cli.py`

**Interfaces:**
- Produces `GenerationOptions` and `run_batch(repo_root: Path, batch_path: Path, options: GenerationOptions) -> int`.
- `run_batch` calls `agent.batch_runner.build_batch_config` and `run_dataset_batch` via `asyncio.run`, forwarding concurrency, local work concurrency, prompt path, max cost, resume, and pause options.
- `python -m pipeline.cli` exposes `prepare`, `generate`, `validate`, and `export` subcommands and returns non-zero for invalid arguments or failed validation.

- [ ] **Step 1: Write failing CLI tests**

```python
def test_cli_help_lists_pipeline_stages():
    result = subprocess.run(
        [sys.executable, "-m", "pipeline.cli", "--help"],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0
    assert "prepare" in result.stdout
    assert "generate" in result.stdout
    assert "validate" in result.stdout
    assert "export" in result.stdout
```

- [ ] **Step 2: Run the CLI test to verify it fails**

Run: `uv run --directory articraft_data pytest tests/pipeline/test_cli.py -q`

Expected: FAIL because `pipeline.cli` is missing.

- [ ] **Step 3: Implement the generation wrapper and CLI**

Keep provider/model/thinking-level values in the CSV and pass the existing batch runner's configuration options through unchanged. The CLI must create parent output directories, print the run summary as JSON, catch `ValueError`/`FileNotFoundError` and return status 2, and never print environment variable values.

- [ ] **Step 4: Run the CLI and focused test suite**

Run: `uv run --directory articraft_data pytest tests/pipeline -q`

Expected: PASS with all pipeline tests passing.

- [ ] **Step 5: Commit the public CLI**

```bash
git -C articraft_data add pipeline tests/pipeline
git -C articraft_data commit -m "feat: expose clean data pipeline CLI"
```

### Task 4: Build The Clean Archive

**Files:**
- Create: `articraft_data/tools/build_zip.py`
- Create: `articraft_data/tests/pipeline/test_build_zip.py`
- Create: `articraft_data_pipeline/README.md`
- Create: `articraft_data_pipeline/pyproject.toml`
- Create: `articraft_data_pipeline/.env.example`
- Create: `articraft_data_pipeline/examples/batch.csv`
- Create: `articraft_data_pipeline/LICENSE`
- Create: `articraft_data_pipeline/NOTICE`

**Interfaces:**
- `build_archive(source_root: Path, output_zip: Path) -> ArchiveReport`.
- The builder copies `pipeline`, `agent`, `articraft`, `sdk`, `storage`, and `cli` Python files plus package metadata, README, example, and focused tests.
- It rejects any selected path containing `.env`, `.venv`, `data/`, `cache`, `node_modules`, `viewer/web`, `logs`, `scripts`, or generated media, and verifies the zip member list after writing.

- [ ] **Step 1: Write failing archive tests**

```python
def test_build_archive_excludes_runtime_state(tmp_path):
    archive = tmp_path / "articraft_data_pipeline.zip"
    report = build_archive(REPO_ROOT, archive)
    names = set(report.members)

    assert "articraft_data_pipeline/pipeline/cli.py" in names
    assert not any("/.env" in name or name.endswith("/.env") for name in names)
    assert not any("/data/" in name for name in names)
    assert not any("/.venv/" in name or "/node_modules/" in name for name in names)
    assert not any(name.startswith("articraft_data_pipeline/viewer/") for name in names)
```

- [ ] **Step 2: Run the archive test to verify it fails**

Run: `uv run --directory articraft_data pytest tests/pipeline/test_build_zip.py -q`

Expected: FAIL because the builder and archive source files do not exist.

- [ ] **Step 3: Implement the allowlisted builder and package metadata**

Use `zipfile.ZipFile` with `ZIP_DEFLATED`, sorted relative paths, and normalized archive member names. Copy code from the nested repository without following its `.git` directory. Copy tests only from `tests/pipeline`. Include runtime dependencies from the source project in the clean package's `pyproject.toml`, omitting viewer/frontend-only dependencies only when imports remain valid. Exclude sample records and all caches.

- [ ] **Step 4: Run archive tests and inspect the member list**

Run: `uv run --directory articraft_data pytest tests/pipeline/test_build_zip.py -q`

Expected: PASS and no forbidden member is present.

- [ ] **Step 5: Build the requested zip**

```bash
python articraft_data/tools/build_zip.py \
  --source-root articraft_data \
  --output articraft_data_pipeline.zip
```

Expected: the command prints the archive path, member count, and byte size, then exits 0.

- [ ] **Step 6: Commit the builder and package metadata**

```bash
git -C articraft_data add tools/build_zip.py tests/pipeline
git -C articraft_data commit -m "build: package clean Articraft pipeline archive"
```

### Task 5: Verify The Extracted Deliverable

**Files:**
- Modify: `articraft_data_pipeline.zip`

- [ ] **Step 1: Re-run the full focused pipeline suite**

Run: `uv run --directory articraft_data pytest tests/pipeline -q`

Expected: exit code 0 with zero failures.

- [ ] **Step 2: Verify the archive structurally**

Run: `unzip -t articraft_data_pipeline.zip` and a Python member-list check for forbidden names.

Expected: archive integrity passes and no forbidden path is present.

- [ ] **Step 3: Verify extracted CLI help without network access**

Extract into a temporary directory under `.tmp`, then run:

```bash
PYTHONPATH=.tmp/articraft_data_pipeline \
  python -m pipeline.cli --help
```

Expected: exit code 0 and all four stages listed.

- [ ] **Step 4: Run formatting and linting for new code**

Run: `uv run --directory articraft_data ruff format --check pipeline tools/build_zip.py tests/pipeline` and `uv run --directory articraft_data ruff check pipeline tools/build_zip.py tests/pipeline`.

Expected: both commands exit 0.

- [ ] **Step 5: Report the final artifact**

Record the absolute zip path, `stat -c '%s'` byte size, member count, test result, and any source-repository changes. Do not claim completion until each command above has fresh exit-0 evidence.
