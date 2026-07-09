# Picture Expansion Ledger

This directory is a small ledger for picture-based variant expansion.
It is not a second Articraft data store.

- `picture/` remains the source image taxonomy.
- the per-record `picture.json` sidecar (aggregated into `data/index/subcat/` shards) maps
  images to original records — the legacy `external_assets_map.json` is **retired** (and the
  `build_manifests.py` helper that read it was removed).
- `data/records/` remains the canonical location for asset records.
- `picture_expansion/generated_assets.jsonl` records variants generated for the picture taxonomy.

Files:

- `batch_sources.json`: expansion batch summaries to include.
- `generated_assets.jsonl`: one row per generated picture variant.
- `validation_report.json`: counts and validation issues from the latest rebuild.
- `scripts/build_manifests.py`: rebuilds the ledger from existing Articraft files.

Rebuild:

```bash
python picture_expansion/scripts/build_manifests.py
```

To add a new expansion batch, add its `summary.jsonl` to `batch_sources.json`,
then rerun the script.
