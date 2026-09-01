# Infinite Mobility Table 3 adapter report

Date: 2026-08-24

## Outcome

Implemented the Infinite Mobility Table 3 adapter in
`scripts/run_table3_infinite_mobility.py` and its focused regression suite in
`tests/test_infinite_mobility_table3.py`. No formal 720-asset Table 3 run was
started.

The adapter reuses `evaluate_urdf`, `failed_record`, and `aggregate_records`
from `scripts/run_urdf_table3_lam.py`; it does not redefine FK, joint sweep, or
metric semantics. It evaluates the supplementary full generated cohort in the
exact completed Table 2 order, including all seven recovery overlays with their
original TIMEOUT provenance.

## Frozen formal contract

Formal mode requires the canonical cohort, Table 2 result, and protocol paths;
the `.venv_low_medium` interpreter; Python 3.12.3, NumPy 2.5.1, and Trimesh
5.0.0; 21 states per joint; 4 workers; a 120-second per-asset timeout; no
limit; and exactly 720 frozen records.

The canonical cohort is pinned to file SHA-256
`cfd9c06ea35dcec57c53d44dbf52903ecba6f33321075495c97c58fe30d23c08`
and self/content SHA-256
`f5e29f1becd47cae991f5d238dff3f86b2b009365738df3e46cdbea297032c23`.
Formal validation independently proves the exact 20 factory x 36 seed order,
713 original PASS plus 7 recovery identities, 4,723 declared movable joints,
and 55 zero-joint assets. The frozen zero-joint breakdown is 36 VaseFactory
and 19 TableDiningFactory assets. Zero-joint assets remain in the 720-asset
strict denominator and fail strict kinematic pass by protocol.

Formal Table 3 also requires a canonical, complete Table 2 publication. It
checks the exact eight-file artifact closure plus required `.run.lock`
sidecar, Table 2 manifest self-hash and registered file/content hashes, formal
evaluator configuration, 720 deeply validated terminal records in frozen
order, package/source/runtime bindings, completed checkpoint, status counts,
and summary. The clean canonical Table 2 publication is now pinned to manifest
file SHA-256
`3dce6436aac2d25507d7843a3e0e5cbee130e83e0c24c2bfbfb08467ca356290`
and manifest content/self SHA-256
`f1cc7c062767ec6e6cb8d05caea122f17baf42d1a9b12e5bc40d16648c3306c3`.
Its verified artifact-manifest file SHA-256 is
`085930ee671e2267db8ab36fef1fc52f176be5dffaf2dac58e1d9d8f572b6da7`.

## Runtime and recovery evidence

Every asset runs in a fresh child interpreter. A frozen per-asset run token and
runtime binding covers the run manifest, adapter, shared core, protocol,
evaluation config, Python/package versions, and the four child thread
variables. The child returns its actual observations; the parent accepts a
successful record only when runtime, package-before, and package-after
attestations match. Timeout, child exception, malformed result, evaluator
drift, URDF drift, and package drift all produce retained fail-closed records
with the original declared-joint denominator.

Checkpoint publication atomically rewrites manifest-ordered records and then
writes a receipt containing record byte count/SHA-256, manifest file/content
hashes, completed identities/tokens, and completion order. Resume deeply
validates every row and accepts a records-ahead crash window only when exactly
one additional complete row can be proven from the prior checkpoint receipt
and contiguous completion order. Every reusable package and URDF is re-scanned;
drifted rows are removed and rerun. Existing real scratch is moved to an
output-external quarantine, while symlinked or non-directory scratch is
rejected without traversal.

Final publication contains exactly:

- `manifest.json`
- `asset_records.jsonl`
- `summary.json`
- `report.md`
- `environment.json`
- `protocol_snapshot.md`
- `checkpoint.json`
- `artifact_manifest.json`

The generated report includes all seven requested Table 3 cells, category
macro values, parse/tree counts, expected/observed joint denominator,
zero-joint disclosure, 713/7 provenance, package/runtime attestation counts,
and evaluator/protocol/source hash evidence.

## TDD evidence

First implementation RED, before the adapter existed:

```text
12 failed
```

The initial implementation reached `15 passed in 7.44s`; an independent
low-medium-environment run reached `15 passed in 8.38s`.

The read-only second review identified formal configuration, checkpoint,
runtime attestation, completed Table 2 closure, 4,723/55 denominator, exact
artifact closure, and report-evidence gaps. The focused second-round RED was:

```text
12 failed, 10 passed in 9.75s
```

After the fixes, the full adapter suite passed:

```text
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
NUMEXPR_NUM_THREADS=1 TMPDIR=runtime/test_tmp \
.venv_low_medium/bin/python -m pytest -q -p no:cacheprovider \
  --basetemp=runtime/test_tmp/table3_green_full5 \
  tests/test_infinite_mobility_table3.py
22 passed in 11.55s
```

A subsequent formal Table 2 positive/tamper test increased the suite to 23
tests. The second-round verification passed:

```text
23 passed in 9.60s
```

`py_compile` and `git diff --check` also passed for the scoped files.

The third read-only review compared the adapter against the actual Table 2
publication schema and found three remaining gaps: Table 2 result rows were
incorrectly expected to repeat the full package binding, their result-origin
runtime attestations were not validated, and a crash after writing a complete
checkpoint but before sealing the artifact manifest could not resume. The
third-round tests were added before implementation. The focused RED evidence
was:

```text
real-schema/attestation: 1 passed, 4 failed
seal-only recovery:      6 failed, 27 deselected
```

The adapter now validates the real compact Table 2 result schema using
`expected_package_path` plus the package-binding digest, enforces the same
`child_attested` and fail-closed `parent_synthesized` runtime-origin rules as
Table 2, and supports a narrowly constrained seal-only resume. Seal-only
resume revalidates all seven pre-seal artifacts, recomputes and compares the
summary and report, re-scans every package/URDF binding, rejects extra files,
and writes only `artifact_manifest.json`.

Focused third-round GREEN evidence is:

```text
real-schema/attestation: 5 passed, 23 deselected in 1.63s
seal-only recovery:      1 passed in 2.50s
seal-only corruption:    5 passed in 4.13s
```

A complete 33-test run attempted while the formal Table 2 job was using eight
child processes was interrupted by the host thread limit after 25 tests had
passed (`RuntimeError: can't start new thread`). It was not counted as GREEN.
The run coordinator held the final complete rerun until Table 2 had exited and
released its workers.

After Table 2 exited successfully, the single deferred complete run passed on
the target filesystem:

```text
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
NUMEXPR_NUM_THREADS=1 TMPDIR=runtime/test_tmp \
.venv_low_medium/bin/python -m pytest -q -p no:cacheprovider \
  --basetemp=runtime/test_tmp/table3_round3_final \
  tests/test_infinite_mobility_table3.py
33 passed in 13.79s
```

Fresh `py_compile` and scoped `git diff --check` verification also passed. At
that stage the two formal Table 2 pin constants remained `None`; no formal
Table 3 run was started.

A follow-up static review found that seal-only resume still accepted a
complete checkpoint whose `completion_order` exceeded `N`, and could also
accept a records receipt re-signed around a non-contiguous completion-order
set. A new parameterized negative test covers both mutations. Complete
checkpoint creation and resume now both require `completion_order == N` and
record completion orders exactly equal to the unique contiguous set `1..N`;
the incomplete append-window rules are unchanged. At the run coordinator's
request, no dynamic test or compile command was run immediately after this
follow-up patch while host resources were constrained. It is covered by the
later final 43-test result rather than the preceding 33-test result.

## Spawn-capacity retry follow-up

A later formal Table 2 run demonstrated that host thread exhaustion can make
the child `Popen` call raise `BlockingIOError(errno.EAGAIN)`. Treating that
capacity event as an asset failure would pollute both the asset and declared
joint denominators. Table 3 now retries only the `Popen` expression for the
same frozen job and run token. Per-job delays are exponential from 1 second to
a 30-second cap; all four workers share a locked 1,800-second cumulative
budget and an exhaustion event. The lock covers budget checking and delay
reservation, not sleeping. A successful `Popen` transfers ownership to the
existing child/result path, so an EAGAIN from `communicate` or any later step
is never retried and remains fail closed.

On shared-budget exhaustion, EAGAIN jobs produce no asset or joint rows.
Already spawned children are allowed to finish and attested results are still
checkpointed. After every future and child has settled and scratch is safely
removed, the runner exits nonzero with a `running` checkpoint. Resume uses the
original manifest and frozen run tokens. Other spawn and child errors retain
the existing parent-synthesized fail-closed behavior.

The complete retry policy is bound into Table 3's manifest config and
environment, the child-observed runtime config, and parent live validation.
Formal Table 2 publication validation also now requires the real upstream
`spawn_eagain_retry_policy` in both `adapter_config` and `environment` to
exactly match Table 2's frozen policy. Its fixture mirrors those current
fields.

The focused RED was:

```text
6 failed, 1 passed, 35 deselected in 4.85s
```

The passing case was the non-EAGAIN characterization. A separate post-Popen
EAGAIN characterization also passed before implementation, proving that the
existing fail-closed boundary was already correct. After implementation, the
focused suite passed:

```text
8 passed, 35 deselected in 5.12s
```

The first complete run reached `41 passed, 2 failed in 16.62s`. It exposed an
incomplete synthetic attestation fixture and a real interaction between
drift recovery and the newly exact terminal completion-order receipt. The
fixture now includes the real environment binding. When package drift
invalidates retained records, their trusted completion order is compacted to
`1..k` before new work continues, so the final receipt can remain exactly
`1..N`.

A verification attempt after those two fixes could not start the first
`ThreadPoolExecutor` thread because the host had 16,187 threads and raised
`RuntimeError: can't start new thread`. Per coordinator instruction, no
further dynamic reruns were attempted while the Table 2 clean formal job was
active.

After that job completed, canonical Table 2 preflight validated all 720 rows,
`{"completed": 720}` status counts, exact manifest file/content pins, the
artifact-manifest hash above, the current upstream spawn policy, runtime
attestations, checkpoint, and artifact closure. No Table 3 evaluation was
started by this preflight. The single deferred complete suite then passed:

```text
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
NUMEXPR_NUM_THREADS=1 TMPDIR=runtime/test_tmp \
.venv_low_medium/bin/python -m pytest -q -p no:cacheprovider \
  --basetemp=runtime/test_tmp/table3_spawn_final_full \
  tests/test_infinite_mobility_table3.py
43 passed in 25.11s
```

Fresh final `py_compile` and scoped `git diff --check` verification passed.
The two clean Table 2 pins are populated; no formal Table 3 run was started.

## Owned child lifecycle follow-up

Final review found that a post-`Popen` exception from `communicate()` could be
converted into a parent-synthesized failure while the owned child remained
active or unreaped. The worker job directory was also deleted in that path.
The negative tests first reproduced all affected outcomes:

```text
4 failed, 41 deselected in 4.15s
```

Table 3 now applies one bounded process-group cleanup protocol after every
timeout or exceptional `communicate()`: inspect the owned process, send
`SIGTERM`, wait for at most 2 seconds, then send `SIGKILL` and wait for at most
another 2 seconds. A successfully reaped child produces a fail-closed timeout
or error row with explicit termination evidence. This cleanup is strictly
post-spawn and never re-enters the EAGAIN spawn retry path; the frozen job and
run token are spawned exactly once. Normal completed and child-attested
runtime-failure behavior is unchanged.

If the child still cannot be reaped after `SIGKILL`, the worker raises an
`OwnedProcessLifecycleError`. Both scheduling entry points preserve that
fatal condition rather than synthesizing a metric row. The publication stays
unsealed with a `running` checkpoint, and `.worker_scratch/job_*` is retained
with `job.json` plus `lifecycle_failure.json` for ownership and termination
diagnosis. Already-exited and `ChildProcessError` already-reaped states are
accepted.

The focused GREEN covered TERM-only exit, KILL-required exit, post-Popen
EAGAIN and non-EAGAIN exceptions, the unreapable fatal path, and the existing
real timeout path:

```text
4 passed, 41 deselected in 3.79s
```

The single final complete run passed:

```text
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
NUMEXPR_NUM_THREADS=1 TMPDIR=runtime/test_tmp \
.venv_low_medium/bin/python -m pytest -q -p no:cacheprovider \
  --basetemp=runtime/test_tmp/table3_owned_final_full \
  tests/test_infinite_mobility_table3.py
45 passed in 17.91s
```

Fresh `py_compile` passed for the adapter and test module. Scoped no-index
whitespace checks emitted no findings for all three allowed files. No formal
Table 3 evaluation was started.

## Fatal admission and resume ownership follow-up

The adjacent lifecycle review found that all 720 futures were submitted at
once, while the fatal state was retained only by the parent result loop. A
single-worker reproduction therefore spawned the next queued job after an
unreapable child (`2` spawns instead of `1`), and a coordinated three-worker
reproduction eventually spawned all eight jobs instead of stopping at the
three already active. Resume also quarantined a preserved lifecycle failure
without checking whether its old process group still existed. The clean
resume RED was:

```text
3 failed, 47 deselected in 4.62s
```

Table 3 now passes one thread-safe fatal lifecycle controller to every worker
in a run. The controller uses the same lock for publishing the first
`OwnedProcessLifecycleError` and admitting each exact `Popen` call. A job
whose package scan has not started exits at the initial gate; a scan already
in progress is checked again before scratch/Popen; and the locked Popen gate
closes the remaining race. Fatal publication also wakes any EAGAIN backoff.
The parent cancels pending futures where possible and ignores canceled or
pre-ownership aborted futures without creating metric rows. Children that
already acquired ownership still settle normally, and any valid results are
checkpointed before the run exits nonzero. Fatal scratch is retained based on
the shared controller state rather than parent future-observation order.

Every preserved `lifecycle_failure.json` now binds the asset, frozen run
token, and process-group ID at both the marker and termination-evidence
levels. Before resume creates an executor, the adapter walks existing scratch
without following symlinks, validates each marker against its `job.json` and
pending frozen job, and probes the old group only with `killpg(pgid, 0)`. A
successful probe means the group still exists; permission and all other
non-ESRCH failures mean disappearance cannot be proved. Both cases abort
without Popen or scratch mutation. Only explicit `ProcessLookupError`/ESRCH
allows the recorded scratch directory to be quarantined and the same frozen
run token to resume. No signal is sent to a potentially reused PGID.

Focused GREEN covered the prior unreapable case, the single-worker queue, the
coordinated concurrent queue, and existing/unknown/gone resume states:

```text
6 passed, 44 deselected in 5.97s
```

The single final complete run passed:

```text
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
NUMEXPR_NUM_THREADS=1 TMPDIR=runtime/test_tmp \
.venv_low_medium/bin/python -m pytest -q -p no:cacheprovider \
  --basetemp=runtime/test_tmp/table3_lifecycle_gate_final_full \
  tests/test_infinite_mobility_table3.py
50 passed in 21.70s
```

Fresh `py_compile` passed for the adapter and test module, and scoped
no-index whitespace checks emitted no findings for all three allowed files.
No formal Table 3 evaluation was started.

## Remaining risk

The suite uses small real URDF/package fixtures and real fresh child
interpreters. It does not establish runtime or completion behavior for all 720
generated packages. The canonical Table 2 publication and its registered pins
now pass real 720-row preflight. The formal Table 3 720 run and post-run
artifact verification remain required.
