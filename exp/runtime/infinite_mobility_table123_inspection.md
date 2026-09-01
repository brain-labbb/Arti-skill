# Infinite Mobility: URDF Table 1--3 Read-only Inspection

Inspection date: 2026-08-23. This report is a read-only inventory; it did not
start an evaluation or change an evaluator.

## Decision

Use the existing **720 requested factory x seed cohort** as requested, but do
not cite the existing Infinite Mobility Nano3D reports as the URDF-Sim-Ready
Table 1--3 result. A small, dataset-specific adapter is required to freeze the
720 identities and call the existing shared Table 1 / Table 2 / Table 3 cores.

The intended Table 1--3 result must retain all 720 requested identities in its
asset denominator. The seven original generation timeouts must be represented
as terminal failures; their later 900-second packages may be inspected, but
must not replace the original outcomes in a headline 720-cohort run. This is
the only interpretation consistent with the document's no outcome-based
deletion / replacement rule.

## 1. Existing Cohort and Its Suitability

### What is available

- `runtime/infinite_mobility_v1/manifest.json` freezes 20 public factories and
  seeds `0..35`, thus 720 requested cases. Its source records are
  `runtime/infinite_mobility_v1/records.json`.
- The source summary records `713 / 720` structural package passes, seven
  timeouts, 4,514 declared movable joints, portable relative paths for all 713
  original passes, and **zero native `<collision>` elements**.
- A recovery run is present at
  `runtime/infinite_mobility_timeout_recovery_v1/`; its
  `recovery_manifest.json` declares seven recovered packages. The naming run
  confirms that a 713-original-pass plus 7-recovery overlay makes 720
  parseable `scene.urdf` packages available for descriptive analysis.
- Original successful packages are rooted below
  `runtime/infinite_mobility_v1/cases/<Factory>/seed_<NNN>/package/` and have
  a single case-relative URDF `<seed>/scene.urdf`; recovery packages are
  rooted below
  `runtime/infinite_mobility_timeout_recovery_v1/cases/<Factory>/seed_<NNN>/package/`.
  The exact path and package digest should always come from the record /
  recovery manifest, not be reconstructed by string convention.

### What has already been measured, but is not the requested Table 1--3

- `run_table1_infinite_mobility_reliability.py` audits compile and structural
  package reliability. Its published summary reports `713 / 720` original
  structural passes and correctly marks its Full-QC metrics N/E because it did
  not run the URDF simulation protocol and all original passing exports have
  zero collision geometry.
- `run_infinite_mobility_naming.py` is a direct naming/part-label analysis. It
  intentionally evaluates only generated packages, overlays recovery, and
  reports 720 naming-evaluable assets. It is not URDF-Sim-Ready Table 2.
- `run_infinite_mobility_hierarchy.py` and
  `run_infinite_mobility_hierarchy_paper.py` compute named-link hierarchy and
  topology stability. The paper runner's 30 case headline is five selected
  factories x seeds 0--5; its 180 case supplement is five factories x all 36
  seeds. Neither is the URDF-Sim-Ready Table 3 kinematic-executability
  protocol.

### Consequence for the three requested tables

| Table | Existing usable asset cohort? | Existing Infinite Mobility number reusable as result? | Expected limiting fact |
|---|---|---|---|
| 1, scale and structural diversity | Yes: all 720 identities; 713 original packages plus seven recovery packages are physically available. | No. The current reliability/naming definitions differ and omit or overlay failures. | Full 720 denominator needs explicit failure records for the seven original timeouts. |
| 2, URDF validity and structural integrity | Yes, with the same freeze policy. | No. No existing Table 2 audit result. | Zero native collision elements will make collision coverage fail; parser/resource/inertial results must be measured. |
| 3, kinematic executability | Yes, with the same freeze policy. | No. Existing hierarchy Table 3 is a different metric. | The FK core can assess kinematics, but zero-collision packages and seven failures remain in asset/joint denominators. |

The cohort is a public-factory supplementary cohort, not a shared-category
balanced comparison with the seven datasets in the protocol. The existing
five-category mapping (cabinet, table, refrigerator, dishwasher, oven) may be
used only for a separately labelled supplementary matched panel; it must not
quietly turn the 720-cohort result into the document's seven-method balanced
cohort.

## 2. Recommended Minimal Reuse Plan

### Cohort adapter: required once, shared by all tables

Create a new Infinite Mobility-specific adapter, for example
`scripts/prepare_infinite_mobility_urdf_table123_cohort.py`. It should be a
read-only freezer and produce one manifest before any table runner executes.
It must read only these authoritative inputs:

- `runtime/infinite_mobility_v1/manifest.json`
- `runtime/infinite_mobility_v1/records.json`
- `runtime/infinite_mobility_timeout_recovery_v1/recovery_manifest.json`
- the package directory and `scene.urdf` bytes named by each source record

For every one of the 720 ordered `(factory, seed)` pairs, write one entry. The
entry must retain the original terminal status and may additionally describe a
recovery package, but it must never overwrite `original_status: TIMEOUT` with
`PASS`.

Suggested adapter manifest contract (JSON):

```json
{
  "schema_version": 1,
  "dataset": "Infinite Mobility",
  "classification": "FORMAL",
  "protocol_source": "exp/URDF-Sim-Ready-Automatic-Evaluation.md",
  "cohort_id": "infinite-mobility-public-factories-20x36-v1",
  "selection": {
    "kind": "fixed_cartesian_factory_seed_grid",
    "factories": ["OfficeChairFactory"],
    "seeds": [0],
    "expected_n": 720,
    "outcome_based_reselection": false,
    "ordered_asset_ids_sha256": "<sha256>"
  },
  "assets": [
    {
      "selection_index": 0,
      "asset_id": "OfficeChairFactory__seed_000",
      "factory": "OfficeChairFactory",
      "seed": 0,
      "raw_category": "OfficeChairFactory",
      "original_status": "PASS",
      "primary_package": "<absolute package path>",
      "primary_urdf_relative_path": "0/scene.urdf",
      "primary_urdf_sha256": "<sha256>",
      "package_binding": {
        "file_count": 0,
        "total_bytes": 0,
        "files": [{"path": "0/scene.urdf", "bytes": 0, "sha256": "<sha256>"}],
        "content_manifest_sha256": "<sha256>"
      },
      "declared_joint_count_hint": 0,
      "recovery": {
        "available": false,
        "status": null,
        "package": null,
        "package_sha256": null
      }
    }
  ],
  "manifest_content_sha256": "<canonical sha256 excluding this field>"
}
```

For original timeouts, `primary_package` / primary URDF fields must be null and
`declared_joint_count_hint` must be zero unless a frozen pre-run declaration
supports a nonzero value. Each table then emits a fail-closed asset record.
Recovery fields may support a clearly labelled `available-recovery-only`
supplement, not replacement of the main record.

The adapter must reject symlinks, verify that all referenced files remain
within the package root, calculate the recursive package binding, verify the
recorded package SHA-256 where applicable, and snapshot the exact protocol
Markdown into the output. These are established local patterns in the Table 2
and Table 3 runners.

### Table 1: reuse the structural-analysis core, not a dataset-specific CLI

Best source: `run_table1_partnet_mobility.py`, whose shared module functions
are `SHARED.analyze_urdf(urdf_path)`, `SHARED.fingerprint_package(urdf_path)`,
and `SHARED.aggregate_records(records, ...)`. They calculate the document's
declared link/joint counts, valid rooted tree, normalized topology hash, and
resource-closure fingerprint.

Why not invoke `run_table1_partnet_mobility.py` directly: its CLI requires a
PartNet-Mobility Table 4 frozen manifest, package layout `mobility.urdf`,
`meta.json`, the PartNet root, and an exact N=800 contract. The Articraft Table
1 runner is likewise locked to `model.urdf`, its release root, metadata Git
records, and N=800. Neither accepts an arbitrary manifest.

Minimal adapter work: a wrapper calls the shared analysis/fingerprint functions
for every non-null manifest entry, emits a Table 1 failure record for the seven
timeouts, aggregates with `N_eval=720`, and macro-aggregates `raw_category`
over 20 factory names. It must record the 713/720 resource/analysis coverage
alongside all full-denominator rates. Do not use the recovery overlay for the
main denominator.

### Table 2: reuse the audit core directly

Best source: `run_table2_urdf_articraft.py`:

- `package_binding(package)` and `manifest_self_hash(manifest)` provide the
  binding primitives.
- `freeze_protocol_snapshot(output)` snapshots the exact evaluation protocol.
- `audit_asset_package(package, asset_id, primary_urdf_relative_path=...)`
  performs the nine Table 2 metric audits with `urdfpy 0.0.22`, resource
  closure, finite/tree/joint checks, collision coverage, inertial coverage,
  and inertia validity.
- `failed_record(...)` and `aggregate_records(...)` preserve fail-closed
  denominators and aggregate the normal Table 2 metrics.

Why not invoke `run_table2_urdf_articraft.py` directly: the command's formal
mode is hard-pinned to Articraft roots, 9,996 release IDs, N=800 and that
method's category-record checkout. Its other dataset profiles likewise impose
their own fixed source/cohort contracts. `run_table2_urdf_physx_mobility.py`
is a concrete staging example, but it is bound to a PhysX Table 5 receipt set
and has a deliberately specialised mesh relocation path.

Minimal adapter work: for every package entry, call `audit_asset_package` with
`primary_urdf_relative_path` equal to the record's `0/scene.urdf`-style path,
verify the package binding before and after the audit, and invoke
`failed_record` for original timeouts. Use `raw_category=factory` for macro
averages. One interpreter per asset and the existing owned-process-group
timeout design from the PhysX runner are advisable because `urdfpy` / mesh
loading is not assumed safe in-process.

The zero-collision fact is not a reason to skip Table 2. It should result in
collision coverage failures under the frozen definition, while other metrics
remain evaluated normally.

### Table 3: reuse the FK core directly

Best source: `run_urdf_table3_lam.py`:

- `evaluate_urdf(urdf_path, asset_key, samples=21,
  declared_joint_count_hint=...)` is the shared frozen FK evaluator.
- `failed_record(asset_key, declared_joint_count, reason)` creates exact
  fail-closed per-joint failures.
- `aggregate_records(records, expected_n)` calculates all Table 3 rates using
  declared non-fixed joints as the joint denominator.

It performs the required 21-state single-joint sweep; continuous joints use
the frozen `[-pi, pi]` interval. The core reads visual/collision mesh
references to derive its q0 scale, so it needs the package-relative `scene.urdf`
and all referenced meshes intact. The lack of `<collision>` tags alone is not
a blocker because visual geometry can supply scale; a missing/unreadable mesh
will fail that record or its relevant measured metric according to the core.

Why not invoke a Table 3 script directly: `run_urdf_table3_articraft10k.py`,
`run_urdf_table3_partnet_mobility.py`, and
`run_table3_urdf_physx_mobility.py` hard-code source roots, release/cohort
hashes, expected sample sizes, metadata categories, and sometimes a staging
rewrite. The existing `run_infinite_mobility_hierarchy*.py` scripts have a
similar name but are hierarchy evaluators, not this FK metric.

Minimal adapter work: use fresh child interpreters plus a timeout; call
`evaluate_urdf` on each available package; bind every result to the shared
manifest hash; create `failed_record(asset_id, 0, "original generation timeout")`
for the seven unavailable original assets; then aggregate over `expected_n=720`.
For Table 3 headline strict pass, a zero-joint asset cannot pass because the
core defines `strict_kinematic_pass` as a nonempty set of passing movable
joints. Report the joint denominator and zero-joint assets explicitly.

## 3. Proposed CLI After Adding Only This Adapter

The following are recommended implementation/run commands. They are **not
currently runnable** because the Infinite Mobility Table 1--3 adapter/wrappers
do not yet exist; running a PartNet, Articraft, or PhysX command with fabricated
arguments would violate those runners' formal contracts.

```bash
cd /mnt/zsn/lyb/arti-skill

python exp/scripts/prepare_infinite_mobility_urdf_table123_cohort.py \
  --source-runtime exp/runtime/infinite_mobility_v1 \
  --recovery-runtime exp/runtime/infinite_mobility_timeout_recovery_v1 \
  --policy original-720-fail-closed \
  --output exp/runtime/infinite_mobility_urdf_table123/cohort_manifest.json

python exp/scripts/run_table1_infinite_mobility_urdf.py \
  --cohort-manifest exp/runtime/infinite_mobility_urdf_table123/cohort_manifest.json \
  --protocol exp/URDF-Sim-Ready-Automatic-Evaluation.md \
  --workers 4 \
  --output exp/runtime/table1_infinite_mobility_720

python exp/scripts/run_table2_infinite_mobility_urdf.py \
  --cohort-manifest exp/runtime/infinite_mobility_urdf_table123/cohort_manifest.json \
  --protocol exp/URDF-Sim-Ready-Automatic-Evaluation.md \
  --workers 4 \
  --asset-timeout-seconds 120 \
  --output exp/runtime/table2_infinite_mobility_720

python exp/scripts/run_table3_infinite_mobility_urdf.py \
  --cohort-manifest exp/runtime/infinite_mobility_urdf_table123/cohort_manifest.json \
  --protocol exp/URDF-Sim-Ready-Automatic-Evaluation.md \
  --samples 21 \
  --workers 4 \
  --asset-timeout-seconds 120 \
  --output exp/runtime/table3_infinite_mobility_720
```

Each table output should contain at least `protocol_snapshot.md`,
`manifest.json` (with self-hash and complete ordered assets),
`asset_records.jsonl`, `summary.json`, a readable report, and an artifact
manifest with SHA-256 entries. Table 3 should also preserve a checkpoint /
resume file and child result binding as the existing Articraft runner does.

## 4. Critical Blockers and Risks

1. **No direct compatible formal runner exists.** Existing runners are not
   generic CLIs: they deliberately reject a different release root, cohort
   hash, layout, N, or dataset profile. A small wrapper is required; hacking
   the manifest into an Articraft/PartNet/PhysX shape would create false
   provenance.
2. **720 requested versus 720 physical packages is a protocol decision.** The
   recovery overlay makes packages available but changes the source-generation
   time budget. For Table 1--3 the conservative, document-consistent main run
   is original 720 fail-closed. A recovery-only analysis must be separately
   labelled and must not replace the seven terminal failures.
3. **Collision coverage is structurally zero.** The recorded 713 original
   passes and the independent reliability audit both find zero native collision
   elements. Table 2 collision coverage should therefore be expected to fail
   across the main cohort; no automatic visual-to-collision conversion is
   permitted because it changes the released URDF.
4. **The protocol's cohort panels do not fit this baseline without disclosure.**
   This is a generated public-factory grid rather than a fixed release asset
   corpus with broadly comparable category labels. It can be reported as an
   Infinite Mobility supplementary / Full generated cohort. A shared-category
   panel needs a separately frozen five-category mapping and is only a
   limited comparison, not automatically the document's seven-method common
   category set.
5. **Result classification must be new and explicit.** The document's current
   seven-row tables do not list Infinite Mobility. Add it as a labelled
   supplementary row/panel or revise the frozen evaluation plan before
   inserting it into a headline cross-method table.
6. **Table 3 can have a large declared joint workload.** The source records
   report 4,514 movable joints for the 713 original passes. At 21 states per
   bounded joint, per-asset process isolation, timeout enforcement, and
   checkpointing are necessary. Do not infer FK pass from the structural
   generation pass.
7. **Mesh/path correctness remains an actual evaluation gate.** The source
   preliminary checker found all 713 original successful packages to have
   relative mesh paths and valid references, but Table 2/3 must independently
   verify the exact resource closure and package hash. The recovery tree also
   contains an `attempts/orphan_staging_*` artifact; use only paths named by
   `recovery_manifest.json`.
8. **Environment pinning is required.** Table 2 depends on its frozen `urdfpy`
   parser behavior; Table 3 uses the documented FK thresholds and K=21. Record
   Python/dependency versions, runner/core SHA-256 values, protocol snapshot,
   timeout policy, and all package hashes before publishing any result.

## Sources Inspected

- `URDF-Sim-Ready-Automatic-Evaluation.md`
- `scripts/run_table1_infinite_mobility_reliability.py`
- `scripts/run_infinite_mobility_naming.py`
- `scripts/run_infinite_mobility_hierarchy.py`
- `scripts/run_infinite_mobility_hierarchy_paper.py`
- `scripts/run_table1_partnet_mobility.py`
- `scripts/run_table2_urdf_articraft.py`
- `scripts/run_table2_urdf_physx_mobility.py`
- `scripts/run_urdf_table3_lam.py`
- `scripts/run_table3_urdf_physx_mobility.py`
- `baselines/Infinite-Mobility-official/README.md`
- `runtime/infinite_mobility_v1/*`
- `runtime/infinite_mobility_timeout_recovery_v1/*`
- `runtime/table1_reliability/infinite_mobility/*`
- `runtime/infinite_mobility_naming_v1/*`
- `runtime/nano3d_hierarchy_paper/infinite_mobility/*`
