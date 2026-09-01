# Table 1/2/3 Full-Release Evaluation Design

## Status

Approved scope in chat on 2026-08-25: exclude Ours from this migration; evaluate
the user-named releases at their complete local release boundary. This design
is the authority for the implementation plan and the new receipts.

## Goal

Replace the current N=800 Table 1/2/3 comparison panels with independently
frozen full-release panels for Articraft-10K, LAM released outputs, Artiverse,
PartNet-Mobility, PhysX-Mobility, SketchMobility, Infinite Mobility, and
Infinigen-Sim. Existing N=800 receipts remain immutable historical artifacts;
the new full-release receipts are published beside them and become the values
reported in the live protocol tables.

Ours-500K and the PV-A release are explicitly out of scope for this migration.
The local Brain-500 cohort is not rerun.

## Fixed Cohort Boundaries

The full-release boundary is the complete, locally available, source-bound
roster at the pinned revision. No outcome-dependent filtering, replacement,
subsampling, or reselection is permitted. Every row remains in the asset
denominator even when parsing, resource loading, conversion, FK, or worker
execution fails.

| Dataset | Local source | Full asset denominator | Full movable-joint denominator | Boundary note |
|---|---|---:|---:|---|
| Articraft-10K | `exp/Articraft-10K/released_urdf` | 9,996 | computed during roster freeze | 240 exact local labels; all primary URDFs present |
| LAM released outputs | `exp/Articulated-Object-Code/released_outputs` | 3,217 | 10,381 | all manifest tiers, including 385 `broken` rows |
| Artiverse | `exp/artiverse/data` | 3,544 | 16,332 | 84 exact local labels; malformed XML stays in denominator |
| PartNet-Mobility | `exp/PartNet-Mobility/data/dataset` | 2,347 | 11,971 | 46 labels; local complete extracted roster |
| PhysX-Mobility | `exp/PhysX-Mobility/extracted/PhysX_mobility` | 2,024 | 9,883 supported movable joints | 132 exact raw labels; unsupported `floating` joints fail closed |
| SketchMobility | `exp/SketchMobility/data` | 4,956 | 11,009 | 70 exact source/category labels |
| Infinite Mobility | `exp/runtime/infinite_mobility_urdf_table123_cohort` | 720 | 4,723 | local 20-factory operational universe, not an official finite release |
| Infinigen-Sim | `exp/Infinigen-Sim/urdf` at HF revision `2dea6d2ca7a7f99d273e9e7437de5caaee261c24` | 8,226 | 31,975 | 17 URDF archives; secure extraction required before scoring |

The declared joint counts above are inventory expectations. The frozen roster
is authoritative; a mismatch aborts scoring before any result record is
published.

## Common Roster Contract

Each dataset gets one immutable `full_release_manifest.json` and one ordered
`full_release_roster.jsonl`. A row contains:

- stable `asset_id`, exact source/category label, and deterministic ordinal;
- absolute source path plus repository-relative portable path;
- primary URDF relative path, byte size, and SHA-256;
- recursive package file list, sizes, SHA-256 values, and package-binding hash;
- declared link/joint inventory and the ordered non-fixed joint list;
- release archive/revision bindings and the roster self-hash.

The same ordered roster is consumed by all three tables. Table 1 and Table 2
use the asset count as `N_eval`; Table 3 uses the same asset denominator and
the roster's declared non-fixed joint count as `J_eval`. Zero-joint assets are
retained and fail the strict asset-level kinematic conjunction according to
the existing protocol semantics.

Infinigen archives are extracted into a separate runtime staging directory,
never over the downloaded archives. Before extraction, every tar member is
checked for absolute paths, `..` traversal, links, and special files. The
17 archive LFS hashes and the extracted roster are bound into the manifest.

## Table Semantics

### Table 1

Reuse the shared structural evaluator and fingerprint protocol. Evaluate every
roster row in deterministic ordinal order. Report release/evaluation labels,
link and movable-joint distributions, multi-joint rate, topology coverage,
duplicate rate, and all parse/resource failures against the full denominator.

### Table 2

Reuse the common URDF audit core with the full roster as input. Run each asset
in a fresh child process under the pinned Table 2 environment, with frozen
thread variables and bounded timeout. Preserve terminal status records and
aggregate all nine existing metrics over the complete asset/joint denominators.

### Table 3

Reuse the analytic FK core and K=21 sweep semantics. Remove all old N=800,
selected-ID, category-count, and fixed-J contracts from the new full-release
adapters. Evaluate every declared supported movable joint; unsupported joint
types and child failures are explicit fail-closed records. Checkpoint after
each completed shard so an interrupted run resumes without changing roster
order or denominators.

## Output And Documentation Boundary

New receipts live below:

`exp/runtime/table123_full_release_20260825/<dataset>/table{1,2,3}/`

Each table directory contains the roster binding, protocol snapshot, runtime
environment, ordered records, summary, report, checkpoint, and artifact
manifest. No existing N=800 directory is overwritten.

The live Markdown Table 1/2/3 rows are replaced with full-release values and
the surrounding prose states the new denominators. Historical N=800 values and
links remain in a clearly labeled appendix for reproducibility. Infinite
Mobility and Infinigen-Sim are labeled supplementary/context rows and are not
silently merged into any shared-category aggregate.

## Failure, Resume, And Resource Policy

- A failed asset is a record, never a reason to shrink `N_eval` or `J_eval`.
- A missing or changed source hash aborts the run before publication.
- Shards are deterministic prefixes/ranges of the frozen roster; retries reuse
  the same row and run token.
- Workers are capped separately per table to avoid exhausting the shared host;
  no GPU is required for Tables 1--3.
- Long runs publish only after artifact closure and independent reaggregation
  pass. Partial checkpoints are not reported as table results.
- Existing unrelated processes and user changes are not stopped or reverted.

## Test Strategy

Before production scoring, tests must fail first for:

1. full-roster loaders rejecting a missing/duplicate row or changed URDF hash;
2. dynamic denominator calculation and zero-joint retention;
3. safe Infinigen tar extraction and path/link rejection;
4. Table 1/2/3 adapters refusing old N=800-only manifests;
5. checkpoint resume preserving exact order and denominator;
6. artifact manifest self-hash and independent summary reaggregation.

The full existing Table 1/2/3 contract suite remains a regression gate. Smoke
runs use at least one normal, one malformed/resource-failing, and one
zero-joint or unsupported-joint fixture where the dataset provides them.

## Non-Goals

This migration does not add Table 4/5 simulation, semantic correctness
judgment, collision-policy changes, physics parameter imputation, or a new
shared-category balanced panel. It does not reinterpret upstream quality tiers
as evaluation outcomes.
