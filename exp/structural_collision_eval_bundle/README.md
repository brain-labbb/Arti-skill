# Articulated Structural and Collision Evaluation Bundle

This portable bundle evaluates the six structural-integrity metrics and three
motion-induced collision metrics agreed for the full eight-dataset experiment.
It uses URDF, mesh geometry, FK, trimesh proximity, and FCL; no physics simulator
is required.

## Contents

- `METRICS.md`: frozen metric definitions, exclusions, aggregation, limitations.
- `RESULTS_TEMPLATE.md`: current Core-200 structural values and final table shape.
- `structural_integrity_eval.py`: URDF, mesh, FK, support, gap, and axis core.
- `full_eval.py`: parallel atomic workers with resume and deterministic sharding.
- `aggregate.py`: fail-closed coverage audit plus JSON/CSV/Markdown table output.
- `manifest_tool.py`: normalize, rebase, scan, filter, and validate manifests.
- `protocol.json`: frozen K=9 evaluation parameters.
- `datasets.example.json`: editable scan patterns for all eight datasets.
- `smoke_test.py`: self-contained end-to-end test.
- `run_full.sh`: single-node or sharded launcher.

## Environment

Python 3.10 or newer is recommended.

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python smoke_test.py
```

The smoke test must finish before launching a real cohort. It exercises URDF
parsing, primitive meshes, FK, proximity, FCL contacts, parent-child ROI masking,
atomic record writing, and all nine aggregate fields.

## Prepare the Full Manifest

The evaluator accepts normalized JSONL with one asset per line:

```json
{"dataset_slug":"pva","asset_id":"category/seed","urdf_path":"/data/.../model.urdf","package_root":"/data/.../seed"}
```

### Option A: rebase an existing frozen manifest

```bash
python manifest_tool.py normalize \
  --input /path/to/old_manifest.json \
  --out manifest.all.jsonl \
  --path-map /old/server/data=/new/server/data \
  --check-paths
```

Both nested `datasets[].rows[]` JSON and JSONL inputs are supported.

### Option B: scan dataset roots

Edit every root and glob in `datasets.example.json`, then run:

```bash
python manifest_tool.py scan \
  --config datasets.example.json \
  --out manifest.all.jsonl
```

The example globs are starting points, not frozen evidence. Inspect duplicate
URDF variants and dataset-specific package roots before accepting the roster.

### Freeze the eligible articulated cohort

```bash
python manifest_tool.py filter-eligible \
  --input manifest.all.jsonl \
  --out manifest.eligible.jsonl \
  --rejects manifest.rejected.jsonl \
  --min-movable 1 \
  --max-movable 20

python manifest_tool.py validate \
  --input manifest.eligible.jsonl \
  --check-paths
```

Expected counts from the current full-release plan are a useful audit target:

| Dataset | Candidate | Eligible |
|---|---:|---:|
| Articraft | 10,787 | 10,690 |
| LAM | 3,217 | 3,074 |
| Artiverse | 3,544 | 3,472 |
| PartNet-Mobility | 2,347 | 2,266 |
| PhysX-Mobility | 2,024 | 1,941 |
| SketchMobility | 4,956 | 4,955 |
| Infinigen-Sim | 8,225 | 8,131 |
| PV-A | 302,440 | 293,385 |
| Total | 337,540 | 327,914 |

Different source releases may legitimately change these counts. If they differ,
freeze and report the exact manifest hash instead of forcing the old totals.

## Pilot Before Full Release

Run 50 or 200 assets first and inspect error/coverage records:

```bash
MAX_ASSETS=50 WORKERS=8 bash run_full.sh manifest.eligible.jsonl pilot_results
python aggregate.py \
  --manifest manifest.eligible.jsonl \
  --results pilot_results \
  --out pilot_results/aggregate
```

`MAX_ASSETS` limits execution, but aggregation against the full manifest will
correctly show low coverage. For a standalone pilot table, create a separate
50-row manifest.

## Full Single-Node Run

```bash
WORKERS=32 TASK=both bash run_full.sh manifest.eligible.jsonl results_full
```

Each asset writes one deterministic JSON file under `results_full/records/`.
Rerunning the identical command resumes completed terminal records. A mismatched
manifest, protocol, task, or shard count is rejected unless `--overwrite` is
explicitly passed; using a fresh output directory is safer.

Do not set worker count equal to CPU count without measuring memory and mesh I/O.
Start at 16-32 workers and inspect throughput and resident memory.

## Multi-Server Sharding

Use the identical manifest, protocol, task, and `SHARD_COUNT` on every server:

```bash
SHARD_COUNT=8 SHARD_INDEX=0 WORKERS=32 bash run_full.sh manifest.eligible.jsonl results_s0
SHARD_COUNT=8 SHARD_INDEX=1 WORKERS=32 bash run_full.sh manifest.eligible.jsonl results_s1
```

Continue through shard index 7. After completion, copy every shard's `records/`
subdirectories into one empty result directory. Record filenames are based on
asset identity, so properly disjoint shards do not collide. Retain all run and
completion manifests for provenance.

## Aggregate the Final Table

```bash
python aggregate.py \
  --manifest manifest.eligible.jsonl \
  --results results_full \
  --out results_full/aggregate
```

Outputs:

- `summary.json`: exact denominators, coverage, and nine metrics.
- `table.csv`: spreadsheet-ready final table.
- `table.md`: Markdown final table.

Binary asset metrics use the full manifest denominator. Continuous metrics use
asset-balanced evaluable means/P95 with explicit structural and collision
coverage. Do not publish a continuous score without its coverage.

## Recommended Execution Order

1. Freeze and hash the full eligible manifest.
2. Freeze `protocol.json`; do not tune thresholds on PV-A versus baselines.
3. Pass the smoke test.
4. Run a labeled valid/broken calibration and audit parent-child masked contacts.
5. Run an N=200 pilot for every dataset.
6. Launch the full sharded evaluation.
7. Aggregate once all shards are present and archive manifests, logs, and hashes.
