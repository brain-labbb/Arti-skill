# Artiverse Table 2: Ours / PV-A automated evaluation

This evaluation follows the articulation-statistics table in the [Artiverse
paper](https://arxiv.org/abs/2605.24403), Table 2 (PDF page 6).  The frozen
column order and counting policy are stored in
[`artiverse_table2_pva_protocol_v1.json`](reference/artiverse_table2_pva_protocol_v1.json).

## Run

The runner is [`run_artiverse_table2_pva.py`](scripts/run_artiverse_table2_pva.py).
It reads the frozen full-release roster, evaluates every primary `model.urdf`,
and writes ordered per-asset evidence plus CSV/Markdown/JSON summaries.

Smoke test:

```bash
cd /mnt/zsn/lyb/arti-skill
python exp/scripts/run_artiverse_table2_pva.py \
  --limit 5 \
  --workers 2 \
  --strict \
  --output /tmp/artiverse_table2_pva_smoke
```

Full PV-A release:

```bash
cd /mnt/zsn/lyb/arti-skill
python exp/scripts/run_artiverse_table2_pva.py \
  --roster exp/runtime/pva_table1234_full_release_20260826/roster/roster_manifest.json \
  --workers 32 \
  --output exp/runtime/artiverse_table2_pva_full_release_20260827
```

The path above is the completed evidence directory in this workspace.  For a
fresh rerun, keep the same roster/protocol arguments but choose a new output
directory; existing output is never overwritten.

The output directory contains:

* `asset_records.jsonl`: one record per roster row, in frozen roster order;
* `summary.json`: machine-readable totals, coverage, diagnostics, and input
  bindings;
* `table2.csv` and `table2.md`: the Table 2-shaped row;
* `run_manifest.json`: runner/protocol/roster hashes and artifact hashes.

The runner refuses to overwrite an existing output directory.  `--strict`
publishes the evidence and then exits non-zero if an asset, roster count, or
annotation coverage check fails.

## Current result

The 2026-08-27 full-release run evaluated 302,440 / 302,440 assets across 531
categories.  Its Table 2-shaped output is:

| Dataset | # obj | Category total | Avg # obj | # Func. Parts total | Avg | # Arti. Parts total | Avg | # Joints 1-DoF | # Joints 2-DoF |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Ours / PV-A | 302,440 | 531 | 569.6 | 1,979,929 | 6.5 | 1,453,516 | 4.8 | 1,453,516 | 0 |

Evidence:

* [`summary.json`](runtime/artiverse_table2_pva_full_release_20260827/summary.json)
* [`table2.md`](runtime/artiverse_table2_pva_full_release_20260827/table2.md)
* [`run_manifest.json`](runtime/artiverse_table2_pva_full_release_20260827/run_manifest.json)

The run found no failures in this runner's XML structural gate and no roster
joint-count mismatch.  This is a separate annotation-statistics scan; it does
not replace the stricter resource, inertial, collision, or runtime gates in the
existing sim-ready Table 1--4 receipts.
The release roster binding reports `J_eval = 1,453,516`; the XML type counts
are 766,554 revolute, 382,066 prismatic, and 304,896 continuous joints.

## Important scope

The current PV-A package contains URDF and physical/appearance metadata, but no
semantic functional-part or articulation annotation sidecar.  Therefore the
default result is explicitly classified `STRUCTURAL_PROXY` and is not a
paper-comparable semantic gold row:

* functional parts = URDF links with at least one `<visual>` element;
* articulated parts = unique child links of non-fixed XML joints;
* joints = non-fixed XML joint elements, mapped to DoF buckets by XML type.

For reference, PV-A has 1,983,527 total XML links; 1,979,929 of them carry a
visual element and are used by the functional-part proxy.  The distinction is
intentional and recorded per asset.

## Semantic mode

When semantic annotations become available, run:

```bash
python exp/scripts/run_artiverse_table2_pva.py \
  --mode semantic \
  --annotations /path/to/pva_table2_annotations.json \
  --workers 32 \
  --output /path/to/new-output
```

The sidecar must declare schema
`artiverse_table2_pva_annotation_v1`.  It may be a JSON object with an
`assets` list/map, or JSONL rows.  Each row has this shape:

```json
{
  "schema_version": "artiverse_table2_pva_annotation_v1",
  "asset_id": "PV-A/<category>/seed_<n>",
  "functional_parts": [{"id": "base"}],
  "articulated_parts": [{"id": "door"}],
  "joints": [{"pid": "door_joint", "type": "revolute"}]
}
```

Repeated `pid`/`id` records are grouped into one logical joint.  Composite
semantic types are mapped as follows: revolute/continuous/prismatic/screw are
1-DoF, cylindrical/universal are 2-DoF, and 3-DoF or unsupported types remain
in diagnostics.  Fixed-only groups are excluded; a `free` placeholder is
ignored when a concrete motion record exists in the same group.

All selected roster rows remain in the denominator.  Averages use
`n_obj / category_total` for the category average and `part total / n_obj` for
the part averages, rounded to one decimal; incomplete runs report averages as
`null` rather than silently changing the denominator.
