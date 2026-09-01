# Articraft-10K Supplementary Table S1 Evaluator Design

## Objective

Produce a frozen, fail-closed Supplementary Table S1 result for the existing
Articraft-10K N=800 cohort. The evaluator audits evidence that was published
with each asset; it does not create new mechanical receipts and then count
those receipts as source evidence.

## Frozen Cohort And Sources

The cohort is exactly `.records[]`, in stored order, from:

`exp/runtime/table2_urdf_articraft10k_n800_seed20260813_20260813T145915Z/manifest.json`

The runner must bind the following identities before evaluating any metric:

- Table 2 cohort manifest file SHA256
  `13c47e2b2affadb951a01cab826bae139852fca5769e99ec081cc916ffa6373d`.
- Table 2 manifest content SHA256
  `576852cb6da00775e1c51360b82b4be40e0a614e4fb0cfb1bae066912eed56a3`.
- Ordered asset IDs SHA256
  `79c44441600077513d3cde1cda8fef38324e1a0ee660730b860d5313f0ae9784`.
- Frozen Table 4 manifest file SHA256
  `6b4275cf3da29244af70c04acecd87094f0c158dee992db20b04e90c05292c20`
  and content SHA256
  `1c6ba7d9e19818580fe8573cf95bb1d065bf2235d0699070516888520f86d7b6`.
- Frozen Table 4 asset records SHA256
  `b732a53a464a8aeebb74799d5ec737de75f3cca377c9a5b274a5dd35adbe301b`
  and state records SHA256
  `6efd4031ecebf74f30f8d3ec3c312ae2faf1b521322b5d4a8b57bb732177ac8b`.
- Each selected package's recursive resource-closure binding and
  `model.urdf` SHA256 from the Table 2 manifest.

No asset may be resampled, replaced, or omitted after an audit failure.

## Evidence Boundary

Only evidence distributed with the selected asset package before this S1
evaluation qualifies as source mechanical evidence. A later unified evaluator
record, including the frozen Table 4 record, may be used as an independent
strict-result source but may not be reclassified as a published receipt.

An asset has a qualifying mechanical receipt only when a machine-readable
published artifact binds all of the following:

1. asset identity and recursive resource-closure hash;
2. mechanical protocol and runner identity;
3. pair policy and contact or penetration threshold;
4. the asset-level mechanical conclusion.

A compile report containing only compile success, visual/URDF compilation, or
unbound authored checks does not qualify. The current Articraft-10K cohort's
compile reports are expected to fail this qualification, but the runner must
derive that result from each bound package rather than hard-code the count.

## Metric Operationalization

### Receipt-bound Assets

For every package, parse `compile_report.json` fail-closed and evaluate the
qualification contract above. Report qualifying assets over all 800 assets.
Missing, malformed, drifted, or insufficient reports count as not bound.

### Receipt Replay Pass

Only a qualifying published receipt can be replayed. An independent replay
must bind the same input closure and protocol, execute the stated protocol,
and reproduce the full conclusion. Missing or unqualified receipts count as
failures in the full N=800 denominator. The runner must not replay the later
Table 4 record as though it were a source receipt.

### Deterministic Rebuild Match

Rebuild eligibility requires a published model recipe, complete external
inputs, and a content-addressed build environment or SDK identity sufficient
to recreate the original materializer. A source file plus an unavailable git
commit or null SDK fingerprint is not sufficient.

The current cohort has model source, but its provenance commits are not
resolvable in the available official history and all SDK fingerprints are
null. Therefore the expected formal status is `N/E` with rebuild eligibility
coverage `0/800`. The evaluator audits and records the reason per asset. It
must not execute the source files against the current SDK or report those
results as formal rebuilds.

### Allowance Density

Only a machine-readable allowance registry published in the asset package and
bound to concrete source-URDF link pairs qualifies. Calls present only in
unexecuted `model.py` source do not qualify because the release compile reports
did not materialize or bind them.

For audit provenance, the runner statically counts intended eligible
non-adjacent source-link pairs as unordered pairs of distinct links that each
declare collision geometry, excluding direct parent-child pairs from the
source URDF graph. Malformed URDFs retain fail-closed asset records. Under the
protocol's explicit no-registered-allowance rule, an empty registry reports
allowance density zero; the numerator and audited eligible-pair denominator
are both retained in the summary.

### Strict Pass Without Method-specific Allowance

Use the frozen Articraft-10K Table 4 `strict_collision_pass` conclusion. Join
all records by both selection order and asset identity, then verify model URDF
and package-closure hashes against the Table 2 cohort. Recompute the pass count
from the 800 per-asset records. Do not rerun with a changed simulator or pair
policy under the same cell.

### Registered-allowance Gain

Because no qualifying method-specific allowance registry is published, the
registered-allowance result equals the no-allowance result and gain is exactly
`0 pp`. Source-code allowance calls are reported only as non-qualifying
diagnostics and cannot affect this metric.

## Runner And Outputs

Add one dataset-specific runner:

`exp/scripts/run_s1_articraft10k.py`

It supports `--mode smoke` and `--mode formal`. Both modes audit source bytes;
formal mode requires all 800 assets and runs the complete verifier. Each run
directory contains:

- `frozen_config.json`
- `protocol_snapshot.md`
- `asset_records.jsonl`
- `summary.json`
- `summary.md`
- `manifest.json`

The output manifest binds the runner, this design/protocol snapshot, all
inputs, and all produced artifacts by SHA256. Formal verification checks
cohort identity, order, record count, per-asset input hashes, Table 4 join
identity, aggregate recomputation, metric denominator rules, and output
hashes.

## Failure Handling

All parse, schema, path, hash, join, and evidence-qualification failures are
recorded per asset and retained in metric denominators. Source manifest or
frozen Table 4 identity drift is a run-level error because it invalidates the
entire frozen protocol. The evaluator never executes released `model.py`, does
not access the network, and does not mutate source packages.

## Testing And Execution

Development follows test-driven development:

1. cohort order and package-binding tests;
2. positive and negative mechanical-receipt qualification fixtures;
3. rebuild-eligibility fixtures, including null and unavailable identities;
4. source-URDF eligible-pair counting fixtures;
5. Table 4 order/hash join and fail-closed mismatch fixtures;
6. aggregate and `N/E` rendering tests;
7. smoke execution followed by the frozen N=800 formal audit;
8. independent recomputation and output-hash verification.

This task does not update the paper table. Formal runtime artifacts remain in
an independent `exp/runtime/s1_articraft10k_*` directory for later review.
