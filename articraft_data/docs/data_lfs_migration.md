# Data Records In Git LFS

`data/records/**` is LFS-backed so code and lightweight dataset metadata clone quickly. The normal-Git source of truth for unhydrated browsing is `data/records_index.jsonl`.

## What Stays In Normal Git

- `data/categories/**`
- `data/batch_specs/**`
- `data/system_prompts/**`
- top-level data manifests/docs
- `data/records_index.jsonl`

## Hydration

Use targeted hydration for normal development:

```bash
uv run articraft data hydrate --record rec_...
uv run articraft data hydrate --category washing_machine
uv run articraft data hydrate --time-from 2026-04-01 --time-to 2026-04-07
uv run articraft data hydrate --last 7d
uv run articraft data hydrate --from-file record_ids.txt
uv run articraft data hydrate --all
```

Date filters use `created_at` from `data/records_index.jsonl`. `YYYY-MM-DD` values are inclusive local-day bounds; ISO timestamps are exact. Category and time filters are intersections.

## Validation

`uv run articraft data check` validates normal-Git metadata, the record index, and any hydrated records. Unhydrated records warn through the skipped count rather than failing.

Use stricter modes when needed:

```bash
uv run articraft data check --record rec_...
uv run articraft data check --require-records
```

## Record Index

Regenerate the index after adding, promoting, rating, or editing dataset records:

```bash
uv run articraft data build-record-index
```

The builder updates rows for hydrated records and preserves existing rows for records that are still pointer-only.

## History Rewrite Runbook

The source migration branch should land this tooling first. The actual repository rewrite is an operator step:

1. Freeze merges to `main`.
2. Create an offline mirror or bundle backup of the pre-rewrite repository.
3. In a fresh mirror clone, strip historical `data/records/**` blobs from `main`.
4. Re-add the current `data/records/**` snapshot under Git LFS.
5. Push the rewritten `main` and LFS objects.
6. Ask contributors to reclone or hard-reset to the rewritten branch.

Do not push archival refs containing old record blobs back to the primary origin.
