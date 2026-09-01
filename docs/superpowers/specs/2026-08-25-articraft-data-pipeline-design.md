# Articraft Data Pipeline Design

**Date:** 2026-08-25

**Status:** Approved

## Goal

Create a clean, independently runnable zip distribution of the Articraft data
pipeline. The distribution must support batch preparation, model-backed record
generation, canonical record storage, format validation, and dataset export,
while excluding the source repository's raw dataset, caches, local environments,
secrets, viewer, and historical one-off scripts.

## Scope

The archive will be named `articraft_data_pipeline.zip` and will contain a
source tree rooted at `articraft_data_pipeline/`. It will include the smallest
working closure of the existing `agent`, `articraft`, `sdk`, `storage`, and
`cli` packages needed by the pipeline, plus a new `pipeline` package that gives
the workflow a stable, narrow entrypoint.

Included capabilities:

- Read and normalize a batch CSV with the existing Articraft batch schema.
- Run a generation batch through the existing provider/agent implementation.
- Store records in the canonical `data/records` layout.
- Validate categories, batch specs, records, and index data.
- Export a compact JSONL dataset manifest from valid records.
- Run the complete workflow through a documented CLI.

Excluded content:

- `data/records`, hydrated assets, generated URDFs, rendered media, and large
  indexes.
- `data/cache`, `data/local`, `.pytest_cache`, `.ruff_cache`, `.uv-cache`, and
  other generated state.
- `.venv`, `.env`, provider credentials, upload state, and logs containing
  potentially sensitive runtime information.
- `viewer`, frontend dependencies, historical batch scripts, and unrelated
  development-only commands.

The archive will include only a tiny synthetic example batch and a README that
explains how to point the pipeline at an external data root.

## Architecture

The new `pipeline` package is an orchestration layer, not a second storage
implementation. It will keep the existing Articraft record and batch formats as
the compatibility boundary and delegate generation, storage, and validation to
the copied core modules.

The public stages are:

1. `prepare`: validate a batch CSV and create a normalized run input report.
2. `generate`: invoke the existing batch runner with explicit repository and
   runtime options.
3. `validate`: run canonical format checks and write a machine-readable report.
4. `export`: scan valid dataset entries and write deterministic JSONL output.

Each stage accepts an explicit repository root and output path. No stage relies
on the current working directory for data discovery, and no stage writes outside
the configured root or output directory.

## Package Layout

```text
articraft_data_pipeline/
  README.md
  LICENSE
  NOTICE
  pyproject.toml
  .env.example
  pipeline/
    __init__.py
    config.py
    prepare.py
    generate.py
    validate.py
    export.py
    cli.py
  articraft/
  agent/
  sdk/
  storage/
  cli/
  examples/
    batch.csv
  tests/
  tools/
    build_zip.py
```

The copied packages may retain internal files required by imports, but the
archive builder will select files from an explicit allowlist and reject known
secret, cache, data, and frontend paths. The archive builder itself is kept in
the outer workspace so the deliverable can be rebuilt without adding packaging
machinery to the runtime package.

## Interfaces

The pipeline modules will expose small typed functions:

- `pipeline.prepare.prepare_batch(batch_path, output_dir) -> Path`
- `pipeline.generate.run_batch(repo_root, batch_path, options) -> int`
- `pipeline.validate.validate_repository(repo_root, report_path) -> bool`
- `pipeline.export.export_dataset(repo_root, output_path) -> int`

The CLI will expose four subcommands with `--repo-root` and explicit paths:

```text
python -m pipeline.cli prepare --batch examples/batch.csv --output run
python -m pipeline.cli generate --repo-root run/repo --batch run/batch.csv
python -m pipeline.cli validate --repo-root run/repo --report run/validation.json
python -m pipeline.cli export --repo-root run/repo --output run/dataset.jsonl
```

The generation command will preserve existing provider/model/thinking-level
options and will not add credentials to generated files or reports.

## Error Handling

Invalid CSV rows fail preparation with row numbers and field names. Generation
returns a non-zero status for unrecoverable configuration or storage errors and
retains the existing batch runner's per-row result reporting. Validation returns
false and records every discovered issue in a JSON report. Export refuses to
silently include invalid or incomplete entries and reports skipped records with
their reason.

## Testing

Tests will use temporary directories and synthetic JSON/CSV fixtures. They will
cover batch normalization, invalid required fields, safe repository path
handling, deterministic export ordering, validation report creation, CLI help,
and archive exclusion rules. The copied core tests will not be bundled wholesale;
the focused pipeline tests will verify the public wrapper contracts, while the
source repository's existing test suite remains the reference for the copied
core implementation.

## Acceptance Criteria

- A clean archive is produced at `articraft_data_pipeline.zip`.
- The archive contains no `.env`, credentials, cache directory, virtual
  environment, raw records, rendered assets, frontend, or historical scripts.
- `python -m pipeline.cli --help` works from the extracted package after its
  declared dependencies are installed.
- The prepare, validate, and export stages pass against the included synthetic
  fixture without network access.
- The archive can be rebuilt deterministically enough for content inspection;
  generated zip timestamps may vary, but the selected file set and file content
  must be stable.
- The final response identifies the zip path, its size, and verification results.
