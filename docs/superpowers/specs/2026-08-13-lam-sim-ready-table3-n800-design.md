# LAM Sim-Ready Table 3 N=800 Design

## Status

Approved in chat on 2026-08-13. This document freezes the design before any
LAM N=800 Table 3 result is generated or inspected.

## Goal

Run the Kinematic Executability evaluation defined by Table 3 of
`exp/URDF-Sim-Ready-Automatic-Evaluation.md` on a deterministic sample of 800
assets from the recommended (`viable`) tier of the official
Articulated-Object-Code release.

The experiment measures whether the released URDFs expose finite, executable
kinematics under a frozen discrete sweep. It does not establish that joint
types, axes, origins, limits, geometry, or dynamics match real objects.

## Scope And Claim Boundary

- Method label: `LAM released outputs (viable sample, N=800)`.
- Source release: `YipengGao/Articulated-Object-Code` at revision
  `28cec4f5be7e34fd4d586879ecfcb67f7c5e4cc0`.
- Source manifest: `exp/Articulated-Object-Code/manifest.csv` with SHA-256
  `70216593ec02b71d596e456498ff9863ad0f8e519d5d27d2cf4f58792d412412`.
- Release pool: exactly the 2,533 manifest rows whose `tier` is `viable`.
- This is a sampled viable-tier panel, not a full-release result and not the
  shared-category balanced panel described elsewhere in the protocol.
- Upstream `viable` labels are selection metadata only. They are not reused as
  Table 3 outcomes.
- No asset is regenerated, repaired, converted, or edited for evaluation.
- Table 2 validity, collision freedom, runtime dynamics, semantic articulation
  correctness, and physical realism are outside this run's claims.

## Frozen Cohort

### Identity And Ranking

The unique sample identity is the manifest `rel_path`, not
`object_release_id`, because the latter is not unique across all release rows.

Use this exact selection namespace:

```text
urdf-sim-ready-table3-lam-viable-n800-v1
```

For each viable row, compute:

```text
selection_hash = SHA256(
    UTF8("urdf-sim-ready-table3-lam-viable-n800-v1")
    || byte(0)
    || UTF8(rel_path)
)
```

Sort by `(selection_hash hexadecimal ascending, rel_path bytewise ascending)`
and select the first 800 rows. There is no stratification, replacement, or
outcome-dependent reselection.

The result-blind preflight currently predicts:

- `N_release = 2,533`
- `N_eval = 800`
- 280 represented release categories
- `J_eval = 2,243` non-fixed joints
- selected joint types: 1,140 revolute, 768 prismatic, and 335 continuous
- selected link count: 4,585

The freeze command must independently reproduce these values. A mismatch is
input drift and must stop the run before scoring.

### Frozen Input Records

The frozen manifest contains exactly one row per selected asset with:

- selection rank and selection hash;
- manifest row index, `rel_path`, `object_release_id`, category, and tier;
- absolute source path recorded for provenance and repository-relative path
  used for portable receipts;
- URDF byte size and SHA-256;
- the canonical ordered list of non-fixed joint names, types, axes, declared
  limits, and frozen neutral values;
- referenced geometry paths, byte sizes, and SHA-256 values;
- a digest over the ordered geometry resource list.

All referenced resources used to compute the bounding box are frozen before
the first scored worker starts. A later hash mismatch is a terminal input
validation failure, not a reason to refresh the manifest.

## Kinematic Semantics

### URDF Transform Convention

The analytic evaluator parses the final released `generated.urdf` directly.
For a joint `j`:

```text
T_world(child) = T_world(parent) * T_origin(j) * T_motion(j, q)
```

`T_origin` uses URDF fixed-axis RPY order
`Rz(yaw) * Ry(pitch) * Rx(roll)` followed by the declared translation.
Missing `<origin>` is the URDF identity transform. Missing one-axis `<axis>`
uses the URDF default `(1, 0, 0)`. A declared axis must contain three finite
values and have norm greater than `1e-12`; it is normalized for FK while its
raw norm is retained as evidence.

Motion transforms are:

- revolute and continuous: Rodrigues rotation about the normalized local axis;
- prismatic: translation along the normalized local axis;
- fixed: identity;
- planar, floating, spherical, malformed, and unknown joint types:
  unsupported by this scalar-sweep protocol and failed closed.

The selected frozen cohort contains no planar, floating, spherical, mimic, or
unknown non-fixed joints. The evaluator still implements explicit fail-closed
records for those cases so future input drift cannot silently become identity
motion.

### Neutral State

URDF has no standard field for a declared initial scalar joint position. The
frozen operational neutral state is therefore:

```text
q0 = min(max(0, lower), upper)  for bounded revolute/prismatic joints
q0 = 0                         for continuous joints
```

When one joint is swept, every other non-fixed joint stays at its frozen `q0`.

### Sweep States

`K = 21` means 21 total states including both interval endpoints:

- bounded revolute/prismatic: `linspace(lower, upper, 21)`;
- continuous: `linspace(-pi, pi, 21)`.

The declared ranges are not clipped, even when unusually large. Finite ranges
such as a revolute span above `2*pi` or a prismatic span above 10 native meters
are recorded as diagnostics and executed as declared. Non-finite, missing,
empty, or reversed bounded ranges fail `Valid Range`.

## Geometry Scale

The object bounding-box diagonal is computed at the frozen neutral state from
the union of all readable final visual geometry. Collision geometry is used
only when an asset has no visual geometry.

Mesh vertices are loaded without repair or geometry processing, with the URDF
mesh scale and element origin applied before the neutral link transform.
Primitive geometry is expanded analytically. Scene subgeometry transforms are
preserved. The diagonal is:

```text
d = norm(aabb_max - aabb_min)
```

All coordinates and `d` must be finite, and `d` must exceed `1e-9` meters. An
unreadable geometry payload or invalid diagonal is retained as a failed asset
and failed planned joints; no default object scale is substituted.

## Engines And Isolation

### Analytic Engine

The primary structural FK implementation uses Python XML parsing plus NumPy.
It must not import the Articraft SDK or reuse author-side geometry-QC FK code.
It produces a world transform for every URDF link at every planned state.

### PyBullet Execution Engine

PyBullet is the independent execution engine and is run through
`arti-template/.venv/bin/python` in `DIRECT` mode. The frozen API version is
`202010061` at design time.

The loader uses:

- `useFixedBase=True`;
- loader flags exactly `0`;
- no `URDF_MAINTAIN_LINK_ORDER`;
- no dynamics step and no collision result in any Table 3 metric;
- motors disabled before state resets;
- binding by exact URDF joint and child-link names, never by assumed index
  order.

`URDF_MAINTAIN_LINK_ORDER` is prohibited because the installed PyBullet build
was result-blindly shown to terminate with signal 11 on a viable LAM laptop
that loads successfully with flags `0`.

Each asset executes in a fresh subprocess. The controller launches at most
four subprocesses concurrently. Each asset has one attempt and a 180-second
wall-clock timeout. A timeout kills the complete subprocess group. Nonzero
exit, signal termination, timeout, malformed output, or missing output becomes
a terminal failed record for the asset and all affected planned joints. There
is no retry and no replacement sampling.

## Pose Comparison

PyBullet world link frames are reconstructed by exact link names and aligned
to the URDF root frame before comparison. For analytic transform `A` and
PyBullet transform `B`:

```text
translation_error_normalized = norm(t_A - t_B) / d
rotation_error_rad = acos(clamp((trace(R_A^T R_B) - 1) / 2, -1, 1))
```

The frozen tolerances are:

- translation: `norm(t_A - t_B) <= max(1e-8 meters, 1e-6 * d)`;
- rotation: `rotation_error_rad <= 1e-5`;
- joint reset/readback: `abs(q_actual - q_requested) <=
  max(1e-9, 1e-12 * abs(q_requested))` in the joint coordinate's native unit.

Every raw error and its contributing asset, joint, state, and link are stored.

## Table 3 Metrics

### Valid Range

A joint passes when its type is supported, its effective axis is finite and
nonzero, and its frozen interval is finite and nonempty. Bounded joints require
`lower < upper`. Continuous joints use the frozen `[-pi, pi]` interval.

### Joint Sweep Success

A joint passes when all 21 states:

- execute in both engines;
- produce finite transforms for every declared link;
- bind and reset the intended joint with finite readback;
- keep all other joint readbacks at their neutral values; and
- satisfy the analytic-versus-PyBullet translation and rotation tolerances for
  every link.

### Non-Degenerate Motion

For every descendant link, compare each sampled pose to the neutral pose. Let
`delta_t` be the largest normalized translation and `delta_r` the largest
rotation geodesic over all descendants and all 21 samples. The joint passes
when:

```text
delta_t > 1e-6 OR delta_r > 1e-5 radians
```

Using the maximum over the entire interval, rather than comparing only its two
endpoints, prevents a continuous joint at `-pi` and `pi` from being
incorrectly called degenerate because those endpoint orientations coincide.
Both engines must agree on the pass disposition.

### Subtree Consistency

The descendant subtree is derived only from the frozen URDF graph. For every
state, every non-descendant link must remain within the frozen translation and
rotation tolerances of its neutral pose in both engines. Any out-of-subtree
movement or incomplete link mapping is a failure.

### FK Round-Trip Error

For every sample, PyBullet executes `q0 -> q_sample -> q0`. The joint record
stores the maximum post-return normalized translation error and rotation error
over all links and samples. It passes round-trip when both are within the
frozen pose tolerances.

Analytic `q0` recomputation is stored as an implementation-integrity check but
is not presented as independent asset evidence, because the analytic engine is
stateless. The Table 3 numeric cell reports normalized translation and
rotation separately as `translation / rotation`, together with the count of
joints that produced numeric measurements. Failed joints remain failures in
`J_eval`; they are never silently removed from the pass-rate denominator.

### Joint-Level And Asset-Level Pass

`Joint-level Pass` requires Valid Range, Joint Sweep Success, Non-Degenerate
Motion, Subtree Consistency, and FK Round-Trip pass for the same joint.

`Strict Kinematic Pass` requires every planned non-fixed joint in the asset to
reach Joint-level Pass. An asset-level parse, resource, bounding-box, engine,
timeout, or subprocess failure makes Strict Kinematic Pass false.

## Denominators And Aggregation

- Asset metrics use the frozen `N_eval = 800` denominator.
- Joint metrics use the frozen `J_eval = 2,243` denominator.
- Every planned asset and joint receives a terminal pass/fail disposition.
- Runtime crashes, timeouts, binding failures, and unavailable numeric errors
  contribute failures to pass rates.
- Numeric error distributions report their explicit contributing count over
  `J_eval`; unavailable errors are not imputed as zero or removed without a
  disclosed coverage count.
- Micro rates are direct numerators over 800 or 2,243.
- Category macro rates first compute each metric within each of the 280 frozen
  categories and then take the unweighted mean across categories. Joint-level
  category rates use the category's planned-joint denominator; asset-level
  rates use its selected-asset denominator.
- Summary statistics for numeric errors include mean, median, P90, maximum,
  contributing numerator, and planned denominator.
- Percentages are formatted from integer counts; JSON retains full precision
  and Markdown uses two decimal places.

## Output Contract

The exclusive formal output root is:

```text
exp/runtime/urdf_sim_ready_table3_lam_viable_n800_v1/
```

It contains:

```text
protocol.json
manifest.jsonl
joint_manifest.jsonl
resource_manifest.jsonl
input_receipt.json
environment.json
command.txt
cases/<sample_key>/result.json
cases/<sample_key>/result_seal.json
logs/<sample_key>.stdout.log
logs/<sample_key>.stderr.log
state_records.jsonl.gz
joint_records.jsonl
asset_records.jsonl
summary.json
report.md
self_check.json
hashes.sha256
```

`state_records.jsonl.gz` stores every planned asset/joint/state identity,
requested and read-back values, analytic and PyBullet link poses as translation
plus quaternion, finite flags, engine errors, subtree errors, and round-trip
errors. This preserves enough evidence to recompute every joint metric without
rerunning FK.

Each terminal case result is written atomically and sealed with the protocol,
manifest, joint-manifest, input, runner, result, stdout, and stderr hashes.
Resume skips a case only when all bindings and its seal validate exactly.
Partial or stale files are never treated as completed evidence.

The formal output directory must not already contain a different protocol or
manifest. Smoke tests use a separate root and are permanently labelled
`SMOKE_NOT_A_PAPER_RESULT`.

## Completion And Self-Checks

The formal run status is `COMPLETE_PAPER_ELIGIBLE` only when all of these hold:

- the source release revision and all frozen source hashes match;
- manifest has exactly 800 unique `rel_path` identities and ranks 0 through
  799;
- joint manifest has exactly 2,243 unique asset/joint identities;
- every asset has one valid terminal case seal;
- asset records match the manifest one-to-one;
- joint records match the joint manifest one-to-one;
- all planned state identities are present exactly once or have a sealed
  terminal asset failure that deterministically expands to failed joint
  records;
- every rate can be recomputed from integer numerators and denominators;
- category counts sum to the global denominators;
- `summary.json` reproduces exactly from records;
- every Markdown table value matches `summary.json`;
- all declared evidence files exist, parse, remain inside the authorized
  workspace, and match `hashes.sha256`.

An experiment can be complete while containing failed assets or joints. Missing
terminal evidence makes the run `INCOMPLETE_NOT_A_PAPER_RESULT`; inconsistent
or drifted evidence makes it `FAILED_VALIDATION`.

## Implementation Shape

The implementation adds:

- `exp/reference/urdf_sim_ready_table3_lam_viable_n800_v1.json`: machine-readable
  frozen protocol;
- `exp/scripts/run_urdf_sim_ready_table3_lam.py`: freeze, worker, run,
  aggregate, and verify commands;
- `exp/tests/test_urdf_sim_ready_table3_lam.py`: selection, FK, metric,
  isolation, aggregation, and integrity tests.

The controller invokes the same script in worker mode through the frozen
Python executable. Worker subprocesses never mutate the released dataset.

## Verification Sequence

1. Run unit and synthetic-URDF integration tests under
   `arti-template/.venv/bin/python`.
2. Freeze the N=800 manifest and verify all input hashes and predicted counts
   without scoring.
3. Run a separate fixed first-eight-manifest smoke cohort and verify every
   artifact, record, aggregate, and hash receipt.
4. Recheck current CPU, memory, disk, and unrelated process ownership.
5. Start the formal four-worker N=800 run in a detached process with a PID and
   append-only controller log.
6. Monitor terminal case growth and failure classes without changing the
   frozen protocol, retry policy, worker count, or cohort.
7. Aggregate only after all 800 terminal cases exist, then run the independent
   self-check before reporting any Table 3 value.
