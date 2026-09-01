# Infinite Mobility Table 2 adapter review

## Verdict

**NOT READY / FORMAL 720 BLOCKED.**

The currently published cohort bytes are internally valid and have the intended 720-row order and 713/7 provenance split. The blocker is the adapter contract: a formal run can still accept a different self-consistent cohort, import the numerical stack before applying its thread limits, and become non-resumable after a hard interruption.

## Findings

### Critical 1: the thread policy is applied after NumPy/OpenBLAS has already initialized, and the child attestation does not contain observed thread values

- `scripts/run_table2_infinite_mobility.py:17-18` imports the shared core at module import time.
- `scripts/run_table2_urdf_articraft.py:132-135` imports NumPy/trimesh immediately.
- The adapter does not set `OMP_NUM_THREADS`, `OPENBLAS_NUM_THREADS`, `MKL_NUM_THREADS`, and `NUMEXPR_NUM_THREADS` until `child_runtime_environment()` at `scripts/run_table2_infinite_mobility.py:209-220`, entered only at `scripts/run_table2_infinite_mobility.py:731-733`.
- With all four variables unset, the process had `Threads: 127` immediately after importing the adapter. Inside `child_runtime_environment()`, all four environment variables read `1`, but `/proc/self/status` still reported `Threads: 127`; changing the variables after native-library initialization does not shrink the initialized pools.
- A concurrent pair of shared-core imports reproduced `OpenBLAS blas_thread_init: pthread_create failed` and one verification process exited 130; an earlier concurrent core test printed all eight assertions as passed but exited 139. The identical core test run alone exited 0, isolating the failure to native thread initialization/resource pressure rather than pytest assertions.
- The manifest only records the declared constant at `scripts/run_table2_infinite_mobility.py:283`. `core.environment_metadata()` does not read the four environment variables, and `current_worker_runtime_binding()` builds its evidence from that metadata. Thus `worker_runtime_binding` proves the static config and interpreter/dependencies, but not the values actually observed by the worker.
- `tests/test_infinite_mobility_table2.py:341-363` misses both defects: module import has already happened before the test mutates the environment, and the assertion compares the worker environment object to the same parent object without asserting observed thread variables.

Required correction: establish the four `=1` values before any import of the shared core/numerical stack (for example, through a bootstrap entry point or an import-safe restructuring), and include the fresh child's observed values in the runtime binding. Add a subprocess test launched with non-1 inherited values that proves both the pre-import constraint and child attestation.

### Critical 2: formal mode pins only the cohort path, not the already-frozen cohort identity

- The formal constants at `scripts/run_table2_infinite_mobility.py:21-29` contain no cohort manifest file hash, manifest content hash, or cohort artifact-manifest hash.
- `load_cohort()` computes a receipt from whichever valid bytes are currently present at `scripts/run_table2_infinite_mobility.py:96-104`.
- Formal validation at `scripts/run_table2_infinite_mobility.py:120-140` checks N=720 and the canonical path, but never compares the computed receipt to a trusted frozen digest.
- Self-hash, source hash, and artifact closure checks detect uncoordinated drift. They do not prevent replacing the cohort and its source/artifact receipts with a newly recomputed, internally consistent set and then labeling it `FORMAL`.
- `tests/test_infinite_mobility_table2.py:284-300` demonstrates the gap indirectly: a synthetic object with only N declarations and 720 placeholder rows passes `validate_contract()` once the interpreter is monkeypatched.

The current canonical cohort is:

- `manifest.json` file SHA-256: `cfd9c06ea35dcec57c53d44dbf52903ecba6f33321075495c97c58fe30d23c08`
- manifest content SHA-256: `f5e29f1becd47cae991f5d238dff3f86b2b009365738df3e46cdbea297032c23`
- `artifact_manifest.json` SHA-256: `ac31de70d50ed7153178482bb5283659be94fb5945cc2b7157754ac61dfc5439`

Required correction: freeze these identities as formal constants (or an equivalent independently trusted receipt) and reject any mismatch before output creation. Keep dynamic hashes in the run manifest as evidence, but do not use them as the trust anchor.

### Critical 3: the formal validator accepts a reordered 20 x 36 matrix and invalid PASS/recovery provenance

- `scripts/infinite_mobility_table123_common.py:248-254` checks the approved factory/seed declarations and the *set* of 720 asset IDs, not their list order.
- `scripts/infinite_mobility_table123_common.py:262-265` checks only that each row's `selection_index` matches its current list position and that its own factory/seed fields agree. A permutation with rewritten indices passes.
- `scripts/infinite_mobility_table123_common.py:255-261` validates the seven TIMEOUT rows and total PASS count, but does not enforce `PASS -> recovery_used=False, source=primary, recovery_provenance=None` for all 713 rows.
- The adapter adds only `source == (recovery if recovery_used else primary)` at `scripts/run_table2_infinite_mobility.py:166-172`; it does not connect `recovery_used` to `original_status`.
- Read-only in-memory negative probes against the real cohort confirmed both defects: swapping the first two rows and rewriting indices printed `reordered_formal_rows=ACCEPTED`; marking a PASS row as a recovery row printed `pass_marked_recovery=ACCEPTED` after both the common formal validator and `_freeze_records()` ran.
- The current artifact itself is ordered correctly (`OfficeChairFactory/seed_000` through `WindowFactory/seed_035`) and has 713 PASS plus seven approved TIMEOUT recoveries. This finding concerns the fail-closed contract.
- Table 3's current consumer has an independent exact-order/provenance validator and would reject such a manifest, but that does not make a wrongly labeled Table 2 `FORMAL` result acceptable.

Required correction: add an adapter-local comparison against the exact ordered list `[(factory, seed) for APPROVED_FACTORIES for seed in 0..35]`, require exactly the approved seven recovery IDs, and enforce both directions of the status/source/provenance relation. This can be fixed without changing the already source-bound common freezer.

### Important 4: a hard-interrupted run cannot use the advertised resume path because stale worker scratch is rejected instead of safely recovered

- On any remaining work, `scripts/run_table2_infinite_mobility.py:667-670` aborts when `.worker_scratch` exists.
- A SIGKILL, host loss, or client failure can leave exactly that directory even though per-result checkpoints are durable.
- The shared core already provides ownership-checked quarantine and process-group recovery in `recover_stale_worker_scratch()` and calls it at `scripts/run_table2_urdf_articraft.py:4064-4065`. The adapter does not call it.
- The adapter tests cover ordinary partial-record resume and lock contention, but no stale-scratch hard-interruption state.

Required correction: under the output lock, invoke the shared safe recovery path before loading/scheduling resumed jobs, persist its result or otherwise bind the quarantine state, and add dead/unproven/live-owned scratch tests. Without this, a multi-hour 720 run requires unrecorded manual intervention to resume.

### Important 5: replacing a frozen package directory with a symlink to identical bytes passes both package-binding gates

- `package_file_manifest()` resolves the package root before testing entries at `scripts/run_table2_urdf_articraft.py:399-420`; it rejects symlinks *inside* a package, but not a symlink used as the package root.
- Resume reuse at `scripts/run_table2_infinite_mobility.py:615-624` and the fresh child pre/post checks use this function.
- A temporary diagnostic froze a real package, renamed the directory, and replaced the frozen path with a directory symlink. It observed `root_is_symlink=True`, `binding_matches=True`, and `primary_resolves_to_moved=True`.
- The content metrics would be unchanged, but the frozen source path/containment identity has been redirected. Table 3 later rejects this, while Table 2 currently accepts it.

Required correction: reject a symlink package root and symlink path components before `.resolve()` in every resume and child pre/post binding check. Add a root-redirection regression test in addition to the existing added-file drift test.

## Verified behavior

The following requested areas are otherwise implemented coherently:

- formal CLI freezes 8 workers, 300 s per-asset timeout, no limit, enabled `urdfpy 0.0.22`, the canonical interpreter, and the canonical cohort path;
- the run manifest is self-hashed and binds the cohort receipt, source/evaluator/adapter hashes, protocol snapshot, environment, config, ordered records, and recovery metadata;
- each non-fatal asset failure produces one fail-closed record, and aggregation retains the complete selected denominator;
- fresh children perform package-content binding before and after audit, with runtime drift treated as fatal evaluator drift;
- checkpoint rows are ordered by frozen identity, runtime-token checked, source-bound, and atomically rewritten;
- final artifacts include `manifest.json`, `protocol_snapshot.md`, `environment.json`, `records.jsonl`, `summary.json`, `report.md`, and `checkpoint.json`, all bound by `artifact_manifest.json`;
- the frozen Table 2 manifest schema supplies the fields consumed by `run_table3_infinite_mobility.py` (order, package binding, URDF hash/path, declared joint hint, source and recovery provenance).

## Verification evidence

- Full adapter target under non-1 inherited settings:
  `OMP_NUM_THREADS=7 OPENBLAS_NUM_THREADS=8 MKL_NUM_THREADS=9 NUMEXPR_NUM_THREADS=10 ../arti-template/.venv/bin/python -m pytest -q tests/test_infinite_mobility_table2.py`
  -> `10 passed in 16.80s`, exit 0.
- Shared-core targeted checks, rerun serially after isolating the native import issue:
  `../arti-template/.venv/bin/python -m pytest -q tests/test_table2_urdf_articraft.py -k 'child_runtime_preflight_rejects_post_freeze_drift or child_source_failure_is_runtime_attested or runtime_binding_drift_aborts_scheduler or stale_worker_scratch_dead_job_is_quarantined or resume_record_validation_rejects_strict_and_schema_inconsistency'`
  -> `8 passed, 142 deselected in 1.76s`, exit 0.
- `../arti-template/.venv/bin/python -m py_compile scripts/run_table2_infinite_mobility.py tests/test_infinite_mobility_table2.py` -> exit 0.
- Actual formal read-only preflight (cohort load, artifact/source/evaluator/protocol validation, contract validation, record freeze) -> `formal_preflight=PASS n=720`, with no output directory created.
- Canonical cohort read-only verification -> N=720, exact order true, 713 original PASS, seven TIMEOUT recovery overlays; common formal validator and artifact verification both returned successfully.
- No formal 720 evaluation was run as part of this review.

## Final re-review verdict (round 2)

**NOT READY / FORMAL 720 REMAINS BLOCKED.**

All five findings from the first review are closed by the second-round changes. One new critical runtime-binding gap remains: fresh children attest the adapter but not the shared audit core that they import and execute.

### Closure of the five original findings

1. **Closed: pre-import thread policy and observed child runtime.** The adapter installs all four thread limits before importing the numerical stack at `scripts/run_table2_infinite_mobility.py:21-32`; actual OpenBLAS pools and environment values are observed and fail-closed at `scripts/run_table2_infinite_mobility.py:462-530`. The shared scheduler is redirected through the adapter bootstrap at `scripts/run_table2_infinite_mobility.py:554-566`, and internal children use that path at `scripts/run_table2_infinite_mobility.py:1219-1230`. Tests at `tests/test_infinite_mobility_table2.py:491-546` cover the fresh child binding, normalized interpreter, inherited non-1 values, and actual OpenBLAS pool counts.
2. **Closed: independent formal cohort identity.** The three trusted cohort digests are frozen at `scripts/run_table2_infinite_mobility.py:51-59`; `load_cohort()` binds the manifest and artifact receipt at `scripts/run_table2_infinite_mobility.py:117-153`; formal validation rejects any digest mismatch before output creation at `scripts/run_table2_infinite_mobility.py:355-397`. Tests at `tests/test_infinite_mobility_table2.py:323-360` exercise each pin and the canonical interpreter/count constraints.
3. **Closed: exact 720 order and PASS/recovery provenance.** The adapter independently enforces the exact factory-major 20 x 36 order, row identities, 713 primary PASS rows, the exact seven TIMEOUT recoveries, complete source-bound recovery records, J=4723, and 55 zero-joint assets at `scripts/run_table2_infinite_mobility.py:201-352`. Negative tests at `tests/test_infinite_mobility_table2.py:363-460` cover reorder, PASS/source drift, identity/joint drift, incomplete provenance, wrong path, and wrong hash.
4. **Closed: stale scratch recovery and durable evidence.** Recovery runs under the output lock before resume processing at `scripts/run_table2_infinite_mobility.py:1007-1031`; recovery evidence is checkpointed and validated at `scripts/run_table2_infinite_mobility.py:856-949`. Tests at `tests/test_infinite_mobility_table2.py:570-692` cover dead scratch quarantine, preservation of an unproven live process, and termination of only a proven owned adapter child.
5. **Closed: package-root and ancestor symlink redirection.** Raw frozen path equality and every path component are checked without following symlinks at `scripts/run_table2_infinite_mobility.py:757-778`; the gate wraps child audit before and after at `scripts/run_table2_infinite_mobility.py:790-807` and wraps resumed package reuse at `scripts/run_table2_infinite_mobility.py:1057-1072`. The child fail-closed denominator regression is at `tests/test_infinite_mobility_table2.py:549-567`.

### New critical finding: the fresh child runtime binding does not attest the shared core

- The run manifest binds both the adapter and the shared core at `scripts/run_table2_infinite_mobility.py:631-636`.
- Redirecting `core.SCRIPT_PATH` to the adapter at `scripts/run_table2_infinite_mobility.py:554-566` is necessary for the pre-import bootstrap, but it also makes the runtime binding's `evaluator_path`/`evaluator_sha256` describe only the adapter.
- The shared core's static child binding contains only evaluator, protocol, config, and environment fields at `scripts/run_table2_urdf_articraft.py:344-353`. The adapter environment extension at `scripts/run_table2_infinite_mobility.py:526-530` adds observed thread evidence but no shared-core path or hash. Consequently, `current_worker_runtime_binding()` and `frozen_worker_runtime_binding()` at `scripts/run_table2_urdf_articraft.py:3192-3262` cannot detect shared-core drift.
- Resume validates the current shared-core hash once at `scripts/run_table2_infinite_mobility.py:727-744`, before jobs are scheduled. There is no shared-core hash check in each child or before final publication at `scripts/run_table2_infinite_mobility.py:1116-1175`. A core file change after manifest freeze and before/during child execution can therefore be used by a child and accepted into the checkpoint while its runtime binding still matches the frozen adapter hash. Restoring the core bytes later would also evade the downstream path/hash check.
- The existing test at `tests/test_infinite_mobility_table2.py:212` verifies only that the manifest records a shared-core hash; the child binding tests at `tests/test_infinite_mobility_table2.py:491-546` do not exercise shared-core drift. A read-only structural probe confirmed that neither `RUNTIME_BINDING_STATIC_FIELDS` nor the adapter environment contains `shared_core_path` or `shared_core_sha256`.

Required correction: include the canonical shared-core path and SHA-256 in data that is compared independently in every fresh child (adding them to the adapter's bound environment is one compatible route), and add a negative fresh-child regression that changes the expected/observed shared-core identity and proves fatal runtime rejection. The final publication path should retain the same binding.

### Round 2 verification evidence

- Full adapter target with non-1 inherited settings:
  `OMP_NUM_THREADS=7 OPENBLAS_NUM_THREADS=8 MKL_NUM_THREADS=9 NUMEXPR_NUM_THREADS=10 ../arti-template/.venv/bin/python -m pytest -q tests/test_infinite_mobility_table2.py`
  -> `17 passed in 21.01s`, exit 0.
- `../arti-template/.venv/bin/python -m py_compile scripts/run_table2_infinite_mobility.py tests/test_infinite_mobility_table2.py` -> exit 0.
- Current real formal cohort read-only preflight passed with N=720, first ID `OfficeChairFactory/seed_000`, last ID `WindowFactory/seed_035`, and the three pinned hashes `cfd9c06e...3c08`, `f5e29f1b...c23`, and `ac31de70...439`. All four observed thread variables were `1`; both discovered NumPy/SciPy OpenBLAS pools reported one thread.
- The formal preflight did not call `run()`: the sentinel output path remained absent. No formal 720 evaluation was started.

## Final re-review verdict (round 3)

**READY FOR FORMAL 720.**

No blocking findings remain. The last shared-core child-runtime issue is closed, all five original findings remain closed, and the current real formal cohort passes the complete read-only contract preflight.

### Closure of the last critical finding

- `_environment()` now records the canonical shared-core path and its current SHA-256 at `scripts/run_table2_infinite_mobility.py:526-532`.
- The environment object is frozen into the evaluation manifest and independently hashed at `scripts/run_table2_infinite_mobility.py:630-654`. The shared core remains separately exposed as `shared_core_path`/`shared_core_sha256`, so the publication has both direct and child-runtime bindings.
- The shared core includes the complete environment object and hash in every frozen/current worker binding through `RUNTIME_BINDING_STATIC_FIELDS` and its binding validators at `scripts/run_table2_urdf_articraft.py:344-353` and `scripts/run_table2_urdf_articraft.py:3192-3262`. Since the adapter replaces `core.environment_metadata` with `_environment()` at `scripts/run_table2_infinite_mobility.py:574-586`, every fresh child independently re-hashes the shared-core file before accepting its job.
- Parent resume and pre-schedule gates recompute the same environment at `scripts/run_table2_infinite_mobility.py:747-770` and `scripts/run_table2_infinite_mobility.py:1080-1085`. The final artifact seal repeats it after all rows are checkpointed and before publication at `scripts/run_table2_infinite_mobility.py:1207-1213`.
- The regression at `tests/test_infinite_mobility_table2.py:555-569` freezes a different parent-side shared-core identity while the fresh child observes the real core and requires fatal runtime-binding rejection. Publication assertions at `tests/test_infinite_mobility_table2.py:193-218` also verify the direct and environment-embedded core bindings.

This closes the previously identified window: shared-core drift after manifest freeze is now detected by each fresh child, and drift before final publication is independently detected by the artifact-seal gate.

### Final independent verification

- Full target with deliberately non-1 inherited thread settings:
  `OMP_NUM_THREADS=7 OPENBLAS_NUM_THREADS=8 MKL_NUM_THREADS=9 NUMEXPR_NUM_THREADS=10 ../arti-template/.venv/bin/python -m pytest -q tests/test_infinite_mobility_table2.py`
  -> `18 passed in 22.76s`, exit 0.
- Last-critical negative test, run separately with the same inherited settings:
  `../arti-template/.venv/bin/python -m pytest -q tests/test_infinite_mobility_table2.py::test_child_runtime_binding_rejects_shared_core_drift`
  -> `1 passed in 2.41s`, exit 0.
- `../arti-template/.venv/bin/python -m py_compile scripts/run_table2_infinite_mobility.py tests/test_infinite_mobility_table2.py` -> exit 0.
- Current real cohort formal preflight -> PASS with N=720, exact endpoints `OfficeChairFactory/seed_000` and `WindowFactory/seed_035`, 713 PASS plus seven approved TIMEOUT recovery rows, workers=8, timeout=300.0 s, standard parser enabled, no limit, and canonical Python `/mnt/zsn/lyb/arti-skill/arti-template/.venv/bin/python`.
- Current trusted cohort hashes matched exactly: manifest file `cfd9c06ea35dcec57c53d44dbf52903ecba6f33321075495c97c58fe30d23c08`, manifest content `f5e29f1becd47cae991f5d238dff3f86b2b009365738df3e46cdbea297032c23`, and artifact manifest `ac31de70d50ed7153178482bb5283659be94fb5945cc2b7157754ac61dfc5439`.
- The preflight environment bound the current shared core at `/mnt/zsn/lyb/arti-skill/exp/scripts/run_table2_urdf_articraft.py` with SHA-256 `296f2af6fa73721a586a3e4b60459533b4656ed4068d06935828aab61c074d75`; all four thread variables and both discovered OpenBLAS pools reported one thread.
- A separate negative gate matrix rejected drift in each of the three cohort hashes, workers, timeout, limit, standard-parser flag, canonical cohort path, and canonical Python. Its sentinel output path remained absent.
- Neither preflight called `run()`. No formal 720 evaluation was launched by this review.

## EAGAIN recovery review (round 4)

**NOT READY / FORMAL 720 BLOCKED.**

The Popen-bound EAGAIN recovery itself is correctly in-place, bounded, and resumable. One additional non-Popen retry branch violates the required boundary and can launch an already healthy identity twice with the same run token.

### Blocking finding: non-Popen EAGAIN aborts and replays the whole incomplete batch

- `_RetryingPopen` at `scripts/run_table2_infinite_mobility.py:117-157` is correctly scoped: it catches only `BlockingIOError` with errno 11 and retries the exact delegate call in place. Other exception types and other errno values are re-raised.
- However, `_scheduler_exception_record()` at `scripts/run_table2_infinite_mobility.py:1136-1150` also recognizes any `child_spawn_failed: BlockingIOError: [Errno 11]` produced elsewhere in the shared core's much larger spawn `try` block. It converts that error into the same no-record sentinel even though it did not originate at the Popen boundary.
- `checkpoint_result()` turns that sentinel into `RetryableSpawnEagain` at `scripts/run_table2_infinite_mobility.py:1246-1254`. The outer loop catches it at `scripts/run_table2_infinite_mobility.py:1275-1302`, after the shared scheduler's `finally` has terminated all active children and removed their scratch directories at `scripts/run_table2_urdf_articraft.py:972-986`. It then rebuilds every identity not yet checkpointed with the original in-memory job and token.
- A read-only-code dynamic smoke probe injected one `BlockingIOError(11)` into the second job's final `ownership.json` write, after its Popen had already succeeded. Both the first healthy identity and the second identity were observed in two Popen calls; each pair reused one run token. The final records were child-attested and scratch was empty, so the duplicate execution is invisible in the publication. This is not an environmental false positive: the injection point and attempt counts were deterministic.
- The manifest policy explicitly advertises this batch behavior as `other_spawn_step_action` at `scripts/run_table2_infinite_mobility.py:100-114`, so the policy is hash-bound but does not meet the requested contract of retrying errno 11 only at the Popen boundary while preserving active identities.
- `tests/test_infinite_mobility_table2.py:285-325` proves the normal Popen case does not duplicate the already active job. It does not cover an errno 11 raised immediately after a successful Popen. `tests/test_infinite_mobility_table2.py:360-387` covers PermissionError, not this boundary distinction.

Required correction: retain the in-place `_RetryingPopen` loop and the budget-exhaustion no-record abort, but remove the ordinary `SPAWN_EAGAIN_REASON_PREFIX` batch-retry path. A `BlockingIOError(11)` from any non-Popen spawn step must follow the shared core's existing fail-closed record path, leaving active jobs intact. Add a regression that injects errno 11 after a successful Popen and asserts the healthy identity is spawned once, the affected identity yields one parent-synthesized failure, no token is shared by multiple real child processes, and scratch cleanup remains complete.

### Behavior verified correct

- The Popen retry passes identical arguments on every attempt, so the job path and run token are unchanged; active children remain in the shared scheduler while the wrapper sleeps.
- Backoff is global to the scheduler context, starts at 1 s, doubles through 2/4/8/16 s, caps at 30 s, and does not reset its accumulated budget after a successful spawn. A direct probe exhausted after 63 admitted sleeps totaling 1771 s; the next 30 s sleep was rejected because it would exceed the 1800 s maximum.
- Budget exhaustion produces no metric row, exits nonzero, leaves the checkpoint in `running` state, removes empty worker scratch, and succeeds through `--resume` once spawning is available.
- `BlockingIOError(errno=12)`, `PermissionError(errno=13)`, and unrelated `RuntimeError` were each propagated by `_RetryingPopen` after one delegate call and zero backoff. The scheduler PermissionError regression confirms the normal fail-closed metric record.
- The complete retry policy is present identically in `evaluation.adapter_config` and the runtime-bound environment at `scripts/run_table2_infinite_mobility.py:627-634` and `scripts/run_table2_infinite_mobility.py:691-706`; their canonical hashes remain part of resume and child runtime validation.

### Round 4 verification evidence

- `OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1 ../arti-template/.venv/bin/python -m pytest -q tests/test_infinite_mobility_table2.py` -> `21 passed in 35.18s`, exit 0.
- Four unchanged shared-core scheduler/runtime/scratch checks -> `4 passed, 146 deselected in 2.06s`, exit 0.
- `OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1 ../arti-template/.venv/bin/python -m py_compile scripts/run_table2_infinite_mobility.py tests/test_infinite_mobility_table2.py` -> exit 0.
- The direct boundary probe observed attempt counts `{Factory0/seed_000: 2, Factory1/seed_001: 2}` with one unique token per identity after a single post-Popen errno 11 injection; final scratch was absent.
- No formal evaluation or formal preflight was run in this round.

## EAGAIN recovery final re-review (round 5)

**READY FOR FORMAL 720.**

No blocking findings remain. The retry boundary is now limited to a direct `subprocess.Popen` errno 11, and the round 4 whole-batch replay path has been removed.

### Closure of the round 4 finding

- The published policy now states `scope = subprocess.Popen only` and sends every other spawn-step failure to a fail-closed metric record at `scripts/run_table2_infinite_mobility.py:95-107`.
- `_RetryingPopen` at `scripts/run_table2_infinite_mobility.py:110-150` retries only a directly caught `BlockingIOError` whose errno is 11. It retains the same call arguments, job file, run token, stderr handle, and shared-core active-child state.
- `_scheduler_exception_record()` at `scripts/run_table2_infinite_mobility.py:1129-1138` now reserves its no-record sentinel exclusively for the private `SpawnEagainRetryBudgetExhausted` exception. An ordinary errno 11 from any other spawn step is handled by `core.bound_job_failure()`.
- `run_locked()` makes one shared scheduler call under the Popen proxy at `scripts/run_table2_infinite_mobility.py:1261-1277`. `RetryableSpawnEagain`, the ordinary errno 11 prefix, the pending-job reconstruction loop, and the batch retry are absent.
- The regression at `tests/test_infinite_mobility_table2.py:390-440` injects errno 11 into the second job's post-Popen ownership write. It requires exactly one Popen for each identity, one completed child-attested record for the healthy identity, one parent-synthesized all-fail record for the affected identity, and complete scratch cleanup.

The original independent dynamic probe was rerun without modification to its injection boundary. It now observed `{Factory0/seed_000: 1, Factory1/seed_001: 1}` Popen calls. The first row was `completed/child_attested`; the second was `error/parent_synthesized`, contained the errno 11 spawn failure, had every metric set to fail, and left no worker scratch.

### Final verification evidence

- `OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1 ../arti-template/.venv/bin/python -m pytest -q tests/test_infinite_mobility_table2.py` -> `22 passed in 25.08s`, exit 0.
- Four shared-core scheduler/runtime/scratch checks -> `4 passed, 146 deselected in 1.79s`, exit 0.
- `OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1 ../arti-template/.venv/bin/python -m py_compile scripts/run_table2_infinite_mobility.py tests/test_infinite_mobility_table2.py` -> exit 0.
- A direct wrapper probe observed `[1, 2, 4, 1]` across intermittent failures and successes while retaining a global accumulated wait of 8 s. Persistent EAGAIN admitted 63 sleeps totaling 1771 s and rejected the next 30 s delay before crossing the 1800 s maximum.
- The same probe confirmed `BlockingIOError(errno=12)`, `PermissionError`, and unrelated `RuntimeError` are not retried; the removed batch-retry type and ordinary errno 11 prefix are absent; the bound policy reports `subprocess.Popen only`.
- Temporary probe directories were removed and no internal Table 2 child remained. No formal evaluation or formal preflight was run in this round.

## Actual publication acceptance

**READY / ACCEPTED FOR PUBLICATION.**

The completed canonical artifact at `runtime/table2_infinite_mobility_720` passed an independent read-only acceptance with 48,560 assertions. No formal evaluation, resume, or output-lock acquisition was performed during acceptance.

- Publication closure is exact: the artifact manifest binds the seven expected content files and every recorded byte count/SHA-256 matches. The only additional root entry is the runner's regular operational `.run.lock`; there are no scratch, temporary, symlink, or undeclared content artifacts. All publication hashes were identical before and after acceptance.
- Run-manifest self-hash is valid (`f1cc7c062767ec6e6cb8d05caea122f17baf42d1a9b12e5bc40d16648c3306c3`). Artifact-manifest SHA-256 is `085930ee671e2267db8ab36fef1fc52f176be5dffaf2dac58e1d9d8f572b6da7`; manifest-file SHA-256 is `3dce6436aac2d25507d7843a3e0e5cbee130e83e0c24c2bfbfb08467ca356290`; records SHA-256 is `d488501734a41d4b814c294f7ad94ed529df72b0f99cfaed8b1d19a3bf1c2ada`; summary SHA-256 is `ec071d395b7578c1042b7106a401d94685a47f7566d5214dcad2cdc6a9df51c6`.
- Checkpoint is complete and fully bound: `completed=720`, `n_eval=720`, `remaining=0`, `completion_order=720`; manifest-content, manifest-file, and records hashes match. Its single startup recovery observation reports no quarantine and no terminated owned process group.
- The run manifest equals the freshly verified canonical cohort freeze in exact factory-major 20 x 36 order, from `OfficeChairFactory/seed_000` through `WindowFactory/seed_035`. Manifest and JSONL order, selection ranks/hashes, cohort-row hashes, and the completion-order permutation all validate. Provenance is exactly 713 primary `PASS` plus the seven approved original `TIMEOUT` recovery identities, with no replacement.
- All 720 results are `completed`, `child_attested`, and uniquely bound to 720 run tokens and 720 fresh worker PIDs. Every job/worker runtime binding agrees with the frozen adapter, shared core, protocol, config, normalized canonical interpreter, one-thread environment, and observed one-thread OpenBLAS pools. All workers returned zero with no termination and zero stderr bytes.
- Every result's package-content evidence matches its frozen package binding. All 720 stored recursive file manifests, content-manifest hashes, byte/file counts, legacy package digests, and current primary-URDF hashes validate. No result-level EAGAIN, error, evaluation timeout, or worker stderr evidence exists; the seven `TIMEOUT` strings are source provenance only.
- The Popen-only errno-11 retry policy is identical in adapter config, runtime environment, environment artifact, and every child runtime binding; all corresponding canonical hashes match.

| Metric | Asset result | Factory macro |
|---|---:|---:|
| parse_rate | 720 / 720 (100.00%) | 100.00% |
| resource_resolution | 720 / 720 (100.00%) | 100.00% |
| finite_fields | 720 / 720 (100.00%) | 100.00% |
| valid_tree | 720 / 720 (100.00%) | 100.00% |
| valid_joint_spec | 445 / 720 (61.81%) | 61.81% |
| collision_coverage | 0 / 720 (0.00%) | 0.00% |
| inertial_coverage | 720 / 720 (100.00%) | 100.00% |
| inertia_validity | 720 / 720 (100.00%) | 100.00% |
| strict_urdf_pass | 0 / 720 (0.00%) | 0.00% |

The nine asset-level aggregates and all 20 per-factory denominators/metric counts were independently recomputed from `records.jsonl` and match `summary.json` exactly. Because every factory contributes exactly 36 identities, each unweighted factory-macro rate equals its corresponding asset-level rate.
