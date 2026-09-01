# Infinite Mobility Table 2 adapter report

Date: 2026-08-24

## Second review outcome

The second review blockers are corrected. The adapter now constrains native
thread pools before importing NumPy, pins the published formal cohort bytes,
independently validates exact ordered/provenance semantics, safely quarantines
stale worker scratch on resume, and rejects symlinks in every component of the
raw expected package path. No formal 720-asset evaluation was started.

## Outcome

Implemented the Infinite Mobility Table 2 adapter in
`scripts/run_table2_infinite_mobility.py` with focused coverage in
`tests/test_infinite_mobility_table2.py`. No 720-asset formal evaluation was
started.

The adapter treats the 20 factory x 36 seed dataset as a supplementary full
generated cohort, not an official finite release or shared-category balanced
panel. The seven pre-freeze recovery packages remain identified by
`original_status=TIMEOUT`, `recovery_used=true`, and their frozen recovery
provenance.

## Shared-core reuse

The adapter does not implement Table 2 audit semantics. It delegates to
`run_table2_urdf_articraft.py` for:

- the nine frozen Table 2 metric definitions and `audit_asset_package`;
- recursive no-follow `package_binding` checks before and after every audit;
- `failed_record` and `aggregate_records` fail-closed behavior;
- exact protocol snapshot creation and validation;
- fresh-interpreter job execution, process-group timeout termination, and
  child runtime attestation;
- atomic artifact writes and output locking.

The wrapper validates and binds the Infinite Mobility cohort, converts its rows
to the shared job schema, preserves dataset provenance, checkpoints result
records, and publishes the dataset-specific report/artifact closure. It also
acts as the controlled fresh-child bootstrap, then calls the unchanged shared
`audit_frozen_job`; it does not reimplement audit semantics.

## Contracts

Formal mode requires:

- the common freezer's canonical cohort manifest;
- manifest file SHA-256
  `cfd9c06ea35dcec57c53d44dbf52903ecba6f33321075495c97c58fe30d23c08`,
  content SHA-256
  `f5e29f1becd47cae991f5d238dff3f86b2b009365738df3e46cdbea297032c23`,
  and artifact-manifest SHA-256
  `ac31de70d50ed7153178482bb5283659be94fb5945cc2b7157754ac61dfc5439`;
- exact `N_release=N_eval=720`, validated as the approved 20 x 36 identity
  matrix in factory-major, seed-minor order with the exact 713 primary / 7
  recovery split, `J=4723`, and 55 zero-joint assets;
- `/mnt/zsn/lyb/arti-skill/arti-template/.venv/bin/python` and `urdfpy==0.0.22`;
- 8 workers, 300 seconds per asset, standard parser enabled, and no limit.

The run manifest binds the cohort file and self-hash, the complete cohort
artifact manifest, all upstream source/evaluator bindings, the adapter and
shared core, protocol snapshot, environment, evaluator configs, ordered asset
IDs, primary URDF hashes, and complete package bindings. Table 2 records retain
the 1-based frozen order and the Table 3 inputs (`source`,
`declared_joint_count_hint`, category, original/recovery provenance, package,
URDF hash, and package binding).

Every pending asset runs in a fresh adapter-bootstrap child that invokes the
shared core. The four thread variables are forced to `1` before the core imports
NumPy. Runtime attestation stores the actual observed values and queries the
actual NumPy and SciPy OpenBLAS pool sizes; missing or non-1 pools are fatal.
Source drift before or during audit, child exception, and timeout produce a
bound fail-closed record without changing the denominator.

Resume accepts only records whose manifest, package, primary URDF, runtime,
protocol, checkpoint, completion order, and source metadata bindings validate.
Before loading a resume checkpoint, the shared ownership-checked recovery
quarantines stale scratch, terminates only proven owned process groups, and
persists the recovery evidence in the checkpoint. Raw package paths are checked
component-by-component without following symlinks before and after each child
audit, including the package root. Non-resume runs atomically reserve a new
directory with `mkdir(exist_ok=false)` and refuse an existing output; resume is
serialized by the shared nonblocking output lock.

Completed output contains:

- `manifest.json`
- `protocol_snapshot.md`
- `environment.json`
- `records.jsonl`
- `summary.json`
- `report.md`
- `checkpoint.json`
- `artifact_manifest.json`

The result JSONL stores the package content digest rather than duplicating the
potentially large per-file list. The authoritative complete package binding
remains in `manifest.json` for Table 3 reuse.

## TDD evidence

Initial RED, before the adapter existed:

```text
../arti-template/.venv/bin/python -m pytest -q tests/test_infinite_mobility_table2.py
ERROR tests/test_infinite_mobility_table2.py
ModuleNotFoundError: No module named 'run_table2_infinite_mobility'
```

Initial feature GREEN reached 9 passing adapter tests. An independent run then
exposed a runtime-binding defect when Python was invoked through a path
containing `..`: six child-backed tests failed with
`worker runtime binding drift: environment, environment_sha256`.

Root cause: the parent receipt captured the unnormalized `sys.executable`,
while a fresh child normalized that path. The scheduler also forces
`OMP_NUM_THREADS`, `OPENBLAS_NUM_THREADS`, `MKL_NUM_THREADS`, and
`NUMEXPR_NUM_THREADS` to `1` for children. The regression test deliberately
uses a non-normalized interpreter path and non-1 parent thread values.

Targeted RED:

```text
../arti-template/.venv/bin/python -m pytest -q -p no:cacheprovider \
  --basetemp=runtime/.pytest_table2_thread_red \
  tests/test_infinite_mobility_table2.py::test_child_runtime_binding_uses_child_thread_environment_and_normalized_interpreter
1 failed: child runtime binding drift: environment, environment_sha256
```

The fix uses one scoped runtime context for manifest freeze, parent preflight,
and child launch: it normalizes the interpreter path, installs the exact child
thread environment, and restores all parent values on exit. Child attestation
remains strict.

Targeted GREEN:

```text
1 passed in 3.50s
```

Full GREEN using the independent command shape:

```text
../arti-template/.venv/bin/python -m pytest -q -p no:cacheprovider \
  --basetemp=runtime/.pytest_table2_root_fixed \
  tests/test_infinite_mobility_table2.py
10 passed in 21.37s
```

Final verification also forced non-child parent thread values and confirmed the
scoped runtime restoration/binding behavior across the complete adapter suite:

```text
OMP_NUM_THREADS=7 OPENBLAS_NUM_THREADS=8 MKL_NUM_THREADS=9 NUMEXPR_NUM_THREADS=10 \
  ../arti-template/.venv/bin/python -m pytest -q -p no:cacheprovider \
  --basetemp=runtime/.pytest_table2_final tests/test_infinite_mobility_table2.py
10 passed in 19.63s
```

The shared cohort dependency suite passed `12 passed in 2.43s` using
`tests/test_infinite_mobility_table123.py`. `py_compile` and `git diff --check`
also exited successfully for the scoped Table 2 files.

## Second review TDD evidence

Targeted RED:

```text
../arti-template/.venv/bin/python -m pytest -q -p no:cacheprovider \
  --basetemp=runtime/.pytest_table2_review_red \
  tests/test_infinite_mobility_table2.py \
  -k 'formal_contract_pins or formal_validator or subprocess_bootstrap or package_root_symlink or resume_quarantines or resume_never_kills or resume_terminates_only'
8 failed, 9 deselected
```

The subprocess test reproduced OpenBLAS `pthread_create failed` with inherited
non-1 settings. The other failures demonstrated the absent hash pins and exact
formal validator, acceptance of a package-root symlink, and non-recovering stale
scratch. The identical targeted selection passed `8 passed, 9 deselected in
9.92s` after implementation.

Final target-filesystem GREEN deliberately inherited non-1 values:

```text
OMP_NUM_THREADS=7 OPENBLAS_NUM_THREADS=8 MKL_NUM_THREADS=9 \
NUMEXPR_NUM_THREADS=10 \
../arti-template/.venv/bin/python -m pytest -q -p no:cacheprovider \
  --basetemp=runtime/.pytest_table2_root_20260824_review2 \
  tests/test_infinite_mobility_table2.py
17 passed in 28.49s
```

This suite includes a fresh subprocess that reports both discovered OpenBLAS
pools at one thread, exact formal negative probes against the frozen 720-row
manifest, dead/unproven/live-owned scratch recovery, and package-root symlink
redirection. `py_compile` passed for both scoped Python files. Ruff passed with
`RAYON_NUM_THREADS=1`, and `git diff --check` passed.

## Final shared-core binding review

The final review found that the manifest displayed the shared-core identity but
the worker's static runtime binding did not cover it. The adapter environment
now includes `shared_core_path` and `shared_core_sha256`; because the complete
environment and its canonical hash are already fields in the shared core's
strict runtime binding, parent and child compare the shared core without any
change or relaxation to shared attestation semantics. The adapter recomputes
the same environment at parent preflight and immediately before artifact
closure, so drift after manifest freeze also prevents publication.

The new negative test freezes a temporary drifted core identity in the parent
while the fresh child imports the real core. Before the correction it completed
the asset and failed the assertion:

```text
tests/test_infinite_mobility_table2.py::test_child_runtime_binding_rejects_shared_core_drift
1 failed: DID NOT RAISE FatalRuntimeBindingError
```

After the correction the targeted test passed. Final target-filesystem
verification, again with inherited thread variables set to non-1 values, was:

```text
OMP_NUM_THREADS=7 OPENBLAS_NUM_THREADS=8 MKL_NUM_THREADS=9 \
NUMEXPR_NUM_THREADS=10 \
../arti-template/.venv/bin/python -m pytest -q -p no:cacheprovider \
  --basetemp=runtime/.pytest_table2_root_20260824_core_binding \
  tests/test_infinite_mobility_table2.py
18 passed in 28.06s
```

`py_compile`, Ruff with `RAYON_NUM_THREADS=1`, and `git diff --check` all passed
after this final change.

## Spawn EAGAIN recovery

Two external formal attempts encountered machine-wide cgroup thread pressure:
`subprocess.Popen` raised `BlockingIOError: [Errno 11]`. Those attempts are not
valid results because the previous exception factory converted the spawn
failure into ordinary asset metric rows. No formal run was started while
implementing this correction.

The correction is adapter-local; the shared core file remains unchanged. Only
while the Infinite Mobility scheduler is running, the adapter replaces the
core's module-local `subprocess` reference with a proxy. The proxy delegates all
behavior except `Popen` raising `BlockingIOError(errno=11)`, which retries the
same spawn in place. Already active children remain alive, so a host that can
run fewer than eight simultaneous children still makes progress as children
finish. The original job object and run token are retained.

The fixed policy is present in both `adapter_config` and the runtime-bound
environment:

- retry only a `BlockingIOError` with `errno=11` thrown directly by the
  adapter-controlled `subprocess.Popen` call;
- exponential backoff starting at 1 second, capped at 30 seconds;
- at most 1800 seconds of cumulative planned backoff;
- never checkpoint the retry condition as an asset metric row;
- on budget exhaustion, exit nonzero after shared-core cleanup while preserving
  the running checkpoint for `--resume`;
- keep every failure outside that exact Popen scope, including an errno 11 from
  an ownership write after successful spawn, on the existing fail-closed path.

The exception factory recognizes only the adapter's private
`SpawnEagainRetryBudgetExhausted` marker. It does not classify ordinary error
strings by an EAGAIN prefix. Popen EAGAIN therefore stays inside the proxy and
does not abort the batch or repeatedly terminate healthy active children.

TDD first showed that transient EAGAIN became parent-synthesized metric rows and
that exhaustion returned success (`2 failed, 1 passed`). A capacity regression
then started one healthy child before the next spawn failed; the whole-batch
implementation incorrectly launched that first identity four times. With the
in-place proxy, the original three focused cases pass: transient recovery yields exactly
one child-attested row per identity, exhaustion leaves no EAGAIN record and
resumes successfully, and PermissionError still yields one fail-closed row.

A follow-up RED injected `BlockingIOError(errno=11)` into the second child's
final `ownership.json` write after both Popen calls had succeeded. Prefix-based
classification restarted both identities, so each was spawned twice. The
strictly scoped correction removes the generic EAGAIN sentinel/batch loop. The
healthy identity now spawns once and completes child-attested; the affected
identity spawns once and receives exactly one parent-synthesized fail-closed
row; scratch is removed.

Final verification:

```text
OMP_NUM_THREADS=7 OPENBLAS_NUM_THREADS=8 MKL_NUM_THREADS=9 \
NUMEXPR_NUM_THREADS=10 \
../arti-template/.venv/bin/python -m pytest -q -p no:cacheprovider \
  --basetemp=runtime/.pytest_table2_root_20260824_popen_scope_rerun \
  tests/test_infinite_mobility_table2.py
22 passed in 31.98s
```

Targeted shared-core scheduler/runtime/scratch tests passed `4 passed, 146
deselected in 2.35s`. `py_compile`, Ruff with `RAYON_NUM_THREADS=1`, and
`git diff --check` passed. No shared-core source change was required.

## Remaining risk

The test suite uses small real URDF/package fixtures and real fresh child
interpreters; it does not establish the wall-clock behavior or completion rate
of the 720 large generated packages. Formal execution and post-run artifact
verification are still required. Native collision declarations are expected to
be absent in this cohort, but the adapter does not infer that result and will
run every frozen metric normally.
