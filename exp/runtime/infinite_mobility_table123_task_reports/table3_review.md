# Infinite Mobility Table 3 read-only review

Date: 2026-08-24

Verdict: **NOT READY for the formal 720-asset run.**

Scope reviewed:

- `scripts/run_table3_infinite_mobility.py`
- `tests/test_infinite_mobility_table3.py`
- frozen cohort contract in `scripts/infinite_mobility_table123_common.py`
- intended Table 2 manifest/artifact contract in `scripts/run_table2_infinite_mobility.py`
- shared FK core `scripts/run_urdf_table3_lam.py`
- Table 3 protocol in `URDF-Sim-Ready-Automatic-Evaluation.md`

No formal 720-asset evaluation was run.

## Findings

### Critical 1: formal mode does not freeze the outcome-affecting timeout

- Location: `scripts/run_table3_infinite_mobility.py:1386-1409`, defaults at
  `:43-45`, CLI at `:1419-1423`; missing tests around
  `tests/test_infinite_mobility_table3.py:417-449`.
- `validate_contract()` freezes `samples=21`, the protocol path, and the Python
  path, but accepts any positive `workers` and `asset_timeout_seconds`. The
  timeout is part of fail-closed metric semantics: changing it can change both
  asset and joint pass counts while the run is still labelled `FORMAL`.
- Read-only probe: formal validation accepted `workers=999` and
  `asset_timeout_seconds=0.001`.
- Fix: require exactly 4 workers and 120 seconds in formal mode (or freeze a
  separately approved formal value before the run), and reject every override.
  Add positive and negative tests for both fields. Also freeze an expected
  environment fingerprint/version contract rather than relying only on a
  mutable virtual-environment path.

### Critical 2: resume accepts stale or tampered result rows

- Location: shallow result validation at
  `scripts/run_table3_infinite_mobility.py:953-979`; resume loading at
  `:1195-1212`; checkpoint writes at `:1271-1280` and `:1330-1343`; pending-row
  selection at `:1283-1299`; the current test deliberately reconstructs an
  unhashed checkpoint at `tests/test_infinite_mobility_table3.py:551-578`.
- `checkpoint.json` does not bind the bytes/hash of `asset_records.jsonl`, the
  manifest file hash, completion order, or per-job tokens. `_validate_record()`
  checks only a few identity fields, the joint-list length, a boolean strict
  field, and `fresh_interpreter=True`. It does not validate status, the joint
  metric schema/invariants, source fields, the child PID/executable, or that
  successful child evidence has both package checks true.
- Read-only probe: `_validate_record()` accepted `status="forged"`, one empty
  joint object, `strict_kinematic_pass=true`, and both package-binding evidence
  flags false.
- A resumed completed row is also reused without re-scanning its package and
  URDF. Package drift after the original child completed can therefore retain a
  successful row in the final headline metrics. Table 2 explicitly rechecks
  reusable rows; Table 3 does not.
- Fix: bind checkpoint to manifest-file and records-file hashes; give each job a
  unique frozen runtime token and completion order; deeply validate every core
  result and source/runtime field; and re-scan package plus URDF before reusing
  any row. Drifted rows must be rerun or replaced with a fail-closed record, not
  silently reused. Add tests that mutate a metric/status and that mutate a
  package between interruption and resume.

### Critical 3: the manifest does not prove that every child used the frozen evaluator

- Location: live core import at `scripts/run_table3_infinite_mobility.py:86-95`;
  start-time evaluator hashes at `:790-802`; child launch from the live adapter
  path at `:992-1005`; incomplete worker evidence/validation at `:821-829` and
  `:953-979`; publication at `:1346-1376`.
- The manifest hashes the adapter and shared core once, but every fresh child
  imports the live files again. Child evidence contains no observed adapter/core
  hashes, config hash, environment hash, or thread-environment attestation, and
  publication does not re-hash the evaluator. An edit during a long run can
  produce mixed evaluator versions under one start-time hash.
- Fix: execute children from immutable frozen source copies or pass a strict
  runtime binding and require each child to attest the adapter, core, config,
  Python/environment, and thread settings. Re-verify those hashes before final
  artifact publication. Add an evaluator-drift test that fails closed.

### Critical 4: formal Table 3 does not establish that the upstream Table 2 run completed

- Location: generic artifact check at
  `scripts/run_table3_infinite_mobility.py:316-332`; Table 2 evaluation checks at
  `:400-451`; Table 2 load/selection at `:547-590`; formal gate at `:679-689`;
  minimal two-file upstream fixture at
  `tests/test_infinite_mobility_table3.py:256-258`.
- Formal mode accepts any self-consistent Table 2 manifest labelled
  `mode=formal` / `classification=FORMAL`. It does not require the canonical
  Table 2 path, exact frozen Table 2 formal config (8 workers, 300 seconds,
  urdfpy enabled), or exact completed artifact set. The artifact verifier only
  requires `manifest.json` and validates whatever keys are listed; it never
  reads Table 2 `summary.json`, `records.jsonl`, or `checkpoint.json`.
- Consequently, a synthetic two-file artifact closure can satisfy the loader
  without proving that 720 Table 2 audits reached terminal records. This breaks
  the intended Table 2 -> Table 3 evidence chain even though row order and
  package bindings inside the two manifests are compared carefully.
- Fix: in formal mode require the canonical Table 2 publication (or an exact
  pre-registered file/content hash), its exact artifact key set, completed
  checkpoint and summary, 720 deeply validated result records, and the exact
  Table 2 formal evaluator config/environment. Add a formal fixture that lacks
  each completion artifact in turn and must be rejected.

### Important 1: the 4,723-joint denominator and zero-joint semantics are not frozen in the adapter

- Location: formal asset validation at
  `scripts/run_table3_infinite_mobility.py:487-510`; result/aggregation at
  `:968-971` and `:1346-1359`; report at `:1088-1137`.
- The current frozen cohort does contain exactly 4,723 declared movable joints.
  It also contains **55 zero-joint assets**, not 36: 36 `VaseFactory` and 19
  `TableDiningFactory` rows. The shared core correctly makes a zero-joint asset
  fail strict via `bool(joints)`, and per-row validation keeps each declared
  hint in the joint denominator.
- However, formal validation never asserts `sum(declared_joint_count_hint) ==
  4723`, the expected zero-joint count/identities, or final `j_eval == 4723`.
  The report does not disclose the 55 strict-failing zero-joint assets, despite
  the inspection contract requiring them to be explicit.
- Fix: freeze and validate the 4,723 denominator and the zero-joint roster (or
  an ordered per-row joint-count hash), assert the final aggregate denominator,
  and report `55 / 720` with the factory breakdown. Add a formal-contract test
  that changes one hint while preserving the 720 identity matrix.

### Important 2: checkpoint publication is not crash-consistent and the final closure is open-ended

- Location: scratch setup at `scripts/run_table3_infinite_mobility.py:1232-1236`;
  JSONL append followed by a separate checkpoint replacement at `:1324-1343`;
  artifact construction at `:1170-1192`; generic verification at `:316-332`.
- A crash after fsync of the appended record but before checkpoint replacement
  leaves valid extra JSONL data with stale counts; resume rejects the mismatch
  instead of safely recognizing/recovering the committed prefix. On resume,
  pre-existing `.worker_scratch` is accepted with `exist_ok=True`, including a
  directory symlink, rather than rejected like the Table 2 adapter does.
- Final verification accepts undeclared extra files and does not require an
  exact output key set. The lock and existing-output checks protect cooperating
  writers, but they do not close these crash/filesystem integrity gaps.
- Fix: use a checkpoint generation with atomically rewritten ordered records
  and a bound digest, safely recover an unambiguous committed prefix, reject any
  pre-existing/symlinked scratch root, and validate an exact final artifact
  closure. Add crash-window, scratch-symlink, and undeclared-file tests.

### Important 3: the report omits evidence needed for reliable document writeback

- Location: `scripts/run_table3_infinite_mobility.py:1088-1137` and
  `:1348-1359`.
- The main micro-average cells are renderable, but the report omits category
  macro values already computed by the shared core, parse/tree counts,
  zero-joint counts, original-PASS/recovery split, package-binding outcomes,
  and evaluator/protocol/environment/artifact hashes. It also does not state an
  asserted expected-versus-observed 4,723 joint denominator.
- Fix: render those fields (or provide a dedicated writeback receipt) and add a
  report-schema test that checks all seven Table 3 cells plus provenance,
  denominator, zero-joint, category-macro, and artifact-link fields.

## Confirmed behavior

- Cohort and Table 2 manifest rows are compared in the same exact order, with
  identity, recovery provenance, URDF, and complete package-binding equality.
- The shared core is called with `samples=21`; continuous joints use the frozen
  `[-pi, pi]` interval; ordinary completed records preserve declared joint
  denominators; zero-joint strict semantics are fail-closed.
- Normal child execution uses a fresh interpreter, checks the complete package
  before and after FK evaluation, and turns timeout/child failure into retained
  fail-closed records.
- The output lock, existing-output refusal, protocol snapshot, manifest
  self-hash, and basic artifact byte/hash checks are present.

These positives do not remove the four formal-evidence blockers above.

## Verification evidence

Commands were run with temporary/cache paths under the authorized repository;
no formal 720 run was started.

```text
exp/.venv_low_medium/bin/python -m pytest -q -p no:cacheprovider \
  --basetemp=exp/runtime/table3_review_pytest_tmp \
  exp/tests/test_infinite_mobility_table3.py
15 passed in 6.22s
```

```text
exp/.venv_low_medium/bin/python -m py_compile \
  exp/scripts/run_table3_infinite_mobility.py \
  exp/tests/test_infinite_mobility_table3.py
exit 0
```

The passing suite does not test the four critical cases above, and one current
resume test codifies the weak unhashed-checkpoint behavior.

## Final re-review verdict (2026-08-24)

Verdict: **NOT READY.** This is not yet `READY AFTER PIN`: one critical
Table 2 / Table 3 schema-and-attestation incompatibility remains in code, in
addition to the intentionally empty Table 2 publication pins. One lower-severity
final-sealing crash window also remains.

No formal 720-asset evaluation was run during this re-review.

### Finding closure status

1. **Critical 1 - CLOSED.** Formal mode now freezes `K=21`, `workers=4`,
   `asset_timeout_seconds=120`, the canonical cohort/Table 2/protocol paths,
   the low-medium Python prefix, and the Python 3.12.3 / NumPy 2.5.1 /
   trimesh 5.0.0 fingerprint (`scripts/run_table3_infinite_mobility.py:63-70`,
   `:2339-2387`). Negative tests cover sample, worker, timeout, interpreter,
   runtime, and cohort-path drift.
2. **Critical 2 - CLOSED for the original finding.** Checkpoints now bind the
   manifest file, ordered record bytes/hash, completed identities, run tokens,
   and completion order (`:1900-1938`). Resume deeply validates records,
   recognizes only the single atomic append window, rechecks package/URDF
   bytes, and reruns drifted rows (`:1962-2069`). Tests cover malformed result
   fields, crash-window recovery, package drift, and symlinked scratch.
3. **Critical 3 - CLOSED.** Every job now binds adapter/core/protocol/config,
   manifest file, child runtime, thread environment, and run token
   (`:1043-1245`). Children attest the observed binding, successful records
   require before/after package checks, and the parent revalidates live bindings
   throughout evaluation and before sealing (`:1338-1423`, `:1543-1589`,
   `:2146`, `:2182-2214`, `:2224`, `:2324`).
4. **Critical 4 - OPEN.** Exact Table 2 closure, terminal summary/checkpoint,
   result count, formal config, and optional pins were added, but the validator
   is not compatible with the real Table 2 result schema and does not require
   the real worker attestation. Details follow below.
5. **Important 1 - CLOSED.** The exact ordered per-row joint-count hash,
   `J_eval=4723`, the exact 55 zero-joint roster, and zero-joint strict-fail
   semantics are validated before and after evaluation (`:45-59`, `:737-778`,
   `:2234-2258`). The current frozen values independently recompute to 4,723
   joints and 55 zero-joint assets.
6. **Important 2 - PARTIALLY CLOSED.** Exact artifact key/root closure,
   no-follow artifact validation, append-window recovery, package recheck, and
   scratch quarantine/rejection are present. The final complete-checkpoint to
   artifact-manifest crash window remains open; details follow below.
7. **Important 3 - CLOSED.** The report now renders all seven headline cells,
   category macro, parse/tree counts, expected/observed joint denominator,
   zero-joint breakdown, source provenance, runtime/package attestation, hash
   evidence, and artifact closure (`:1717-1842`).

### Remaining Critical: Table 3 rejects real Table 2 records and accepts records without worker attestation

- Location: `scripts/run_table3_infinite_mobility.py:562-615`, especially
  required `package_binding` at `:568-575`; misleading fixture construction at
  `tests/test_infinite_mobility_table3.py:331-347`.
- The production Table 2 adapter deliberately does **not** copy the potentially
  large full `package_binding` into `records.jsonl`; it stores only
  `package_content_manifest_sha256`. This is an explicit tested contract at
  `tests/test_infinite_mobility_table2.py:217-225` and is also documented in
  `runtime/infinite_mobility_table123_task_reports/table2_report.md:72-74`.
- `_validate_formal_table2_result()` nevertheless compares
  `record.get("package_binding")` with the manifest source row. Therefore every
  real Table 2 formal result will fail with `formal Table 2 result source/order
  mismatch`, even after the two Table 2 manifest pins are filled.
- The Table 3 fixture hides the incompatibility by creating each fake result as
  `{**source, ...}`, which adds the full binding that production explicitly
  omits.
- The same validator checks only `job_runtime_binding`; it does not require or
  validate production `result_origin` and `worker_runtime_binding`. A read-only
  probe confirmed that a `completed` all-pass row with neither field is
  accepted. Thus the fixture is simultaneously stricter than production on the
  package field and weaker than production on execution attestation.
- Fix: remove `package_binding` from result-row equality and continue validating
  its digest against the authoritative manifest binding at `:596-599`; add
  `expected_package_path` to source equality; require the same
  `result_origin` / `worker_runtime_binding` rules as Table 2's
  `validate_resume_record()` (`run_table2_urdf_articraft.py:3751-3764`). Change
  the fixture to mirror `SOURCE_RECORD_FIELDS` exactly and assert that no full
  `package_binding` exists in Table 2 result rows.

### Remaining Important: a crash after the complete checkpoint strands sealing

- Location: completed-output refusal at
  `scripts/run_table3_infinite_mobility.py:2113-2119`; final writes at
  `:2322-2335`.
- The runner writes `checkpoint.json` with `state="complete"` before writing
  `artifact_manifest.json`. A crash between those two atomic writes leaves all
  result artifacts and a complete checkpoint but no formal artifact closure.
  `--resume` then refuses the directory as already complete, so the runner
  cannot validate and finish sealing it.
- Fix: when the checkpoint is complete but the artifact manifest is absent,
  allow a seal-only resume that revalidates manifest, protocol, environment,
  all records, summary/report presence, live evaluator, and exact package
  bindings before generating and verifying the artifact manifest. Add a test
  for this exact crash point.

### Table 2 pin status

`FORMAL_TABLE2_MANIFEST_FILE_SHA256` and
`FORMAL_TABLE2_MANIFEST_CONTENT_SHA256` remain `None` by design. A direct probe
confirmed `verify_formal_table2_publication()` raises
`RuntimeError: formal Table 2 manifest hashes are not pinned` before reading its
path. This is correct fail-closed behavior.

After the real Table 2 formal publication completes, fill both pins (preferably
also pin the completed Table 2 artifact-manifest/records receipt), then rerun
the full tests and this cross-adapter schema check. Filling the pins alone is
not sufficient until the remaining Critical incompatibility is fixed.

### Final verification evidence

All requested thread variables were set to 1. No 720 run was started.

```text
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
NUMEXPR_NUM_THREADS=1 exp/.venv_low_medium/bin/python -m pytest -q \
  -p no:cacheprovider \
  --basetemp=exp/runtime/table3_final_review_pytest_tmp \
  exp/tests/test_infinite_mobility_table3.py
23 passed in 9.98s
```

```text
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
NUMEXPR_NUM_THREADS=1 exp/.venv_low_medium/bin/python -m py_compile \
  exp/scripts/run_table3_infinite_mobility.py \
  exp/tests/test_infinite_mobility_table3.py
exit 0
```

The 23 passing tests do not exercise the production Table 2 result schema or
the complete-checkpoint/pre-artifact crash point, so they do not override the
remaining findings.

## Round 3 static re-review verdict (2026-08-24)

Verdict: **READY AFTER PIN** as a static candidate. The two blockers from the
previous re-review are closed in the current code, and the additional complete
checkpoint completion-order gap found during this round was also closed before
this verdict. The only launch gate now visible is the intentionally unset pair
of hashes for the clean canonical Table 2 publication, followed by the deferred
post-pin dynamic verification described below.

No Table 3 formal run was started. Dynamic tests were deliberately not started
during this review because the canonical Table 2 rerun was active while the
host was close to its prior process/thread `EAGAIN` threshold. This section is
therefore a static verdict, not a claim that the latest test revision has
already been executed independently.

### Round 3 closure evidence

1. **Production Table 2 record compatibility - CLOSED.** Table 3 now compares
   the exact compact source-field projection emitted by Table 2, including
   `expected_package_path` and excluding the full `package_binding`
   (`scripts/run_table3_infinite_mobility.py:578-587`; production definition at
   `scripts/run_table2_infinite_mobility.py:78-98`). It independently compares
   `package_content_manifest_sha256` with the authoritative full binding held in
   the Table 2 manifest (`scripts/run_table3_infinite_mobility.py:604-609`). The
   fixture now uses the same compact projection and explicitly asserts that no
   result row contains `package_binding`
   (`tests/test_infinite_mobility_table3.py:30-50`, `:719-744`).
2. **Table 2 execution attestation - CLOSED.** A `child_attested` result must
   carry a `worker_runtime_binding` exactly equal to its fully frozen job
   binding. A `parent_synthesized` result must omit worker attestation, have
   `error` or `timeout` status, and leave every Table 2 metric false
   (`scripts/run_table3_infinite_mobility.py:610-634`). These rules match the
   production Table 2 resume validator
   (`scripts/run_table2_urdf_articraft.py:3739-3764`). Negative tests cover a
   missing worker attestation, parent-completed forgery, worker binding drift,
   and a partially passing parent failure
   (`tests/test_infinite_mobility_table3.py:775-840`).
3. **Complete-checkpoint seal-only recovery - CLOSED.** When all terminal
   files and a complete checkpoint exist but `artifact_manifest.json` does not,
   resume rechecks the fresh input/manifest/protocol/environment/runtime
   bindings, the exact checkpoint receipt, every record, every package and
   selected URDF, the exact pre-seal file set, recomputed summary, and rendered
   report before writing and exactly verifying the artifact manifest
   (`scripts/run_table3_infinite_mobility.py:2011-2250`, `:2253-2334`). Missing,
   tampered, drifted, and undeclared terminal states are covered by negative
   tests (`tests/test_infinite_mobility_table3.py:1087-1178`).
4. **Complete completion-order receipt - CLOSED.** Complete checkpoints now
   require `completion_order == N`, exactly `N` records, and record completion
   orders equal to the unique contiguous set `1..N`, both when writing and when
   loading (`scripts/run_table3_infinite_mobility.py:1923-1964`, `:2094-2106`).
   The new tests exercise both a checkpoint-only `N+1` mutation and a changed
   record with a recomputed byte/hash receipt and an order gap
   (`tests/test_infinite_mobility_table3.py:1120-1146`).
5. **Frozen formal denominator and runtime contracts - RECONFIRMED.** Formal
   mode still fixes 720 assets, 4,723 declared joints, the exact 55 zero-joint
   roster with fail-closed strict semantics, 21 states per joint, 4 workers,
   and 120 seconds per asset. Evaluator/core/protocol/config/environment and
   fresh-child bindings remain checked before, during, and after evaluation.

### Remaining launch gate

`FORMAL_TABLE2_MANIFEST_FILE_SHA256` and
`FORMAL_TABLE2_MANIFEST_CONTENT_SHA256` remain `None` at
`scripts/run_table3_infinite_mobility.py:61-62`, as required while the clean
Table 2 publication is still being produced. The verifier checks both pins at
`:646-652` before attempting to accept a formal Table 2 publication, so formal
Table 3 remains fail closed.

After the clean canonical Table 2 output completes, fill both hashes from that
publication and then rerun the full current Table 3 test file (now 35 collected
cases) with `OMP_NUM_THREADS`, `OPENBLAS_NUM_THREADS`, `MKL_NUM_THREADS`, and
`NUMEXPR_NUM_THREADS` all set to `1`, run `py_compile`, and validate the actual
canonical Table 2 publication through this adapter. `READY AFTER PIN` becomes
the final dynamic readiness verdict only after those checks pass.

## Final dynamic re-review verdict (2026-08-24)

Verdict: **NOT READY.** One blocking post-`Popen` process-lifecycle finding
remains. The clean Table 2 pins and publication now validate, and the focused
EAGAIN suite passed independently, but the formal Table 3 run must not start
until the child cleanup invariant below is implemented and tested.

No formal 720-asset Table 3 run was started.

### Important (blocking): post-`Popen` exceptions can orphan a live child

- Location: `scripts/run_table3_infinite_mobility.py:1808-1854`, especially
  `process.communicate()` at `:1822-1823` and the unconditional job-directory
  removal at `:1853-1854`; incomplete regression at
  `tests/test_infinite_mobility_table3.py:1253-1297`.
- The retry loop is correctly limited to `subprocess.Popen()` at `:1738-1770`.
  However, after `Popen` succeeds, a `BlockingIOError(EAGAIN)` or any other
  unexpected exception from `communicate()` escapes `_execute_job()`. The
  parent converts it to a fail-closed metric record, but `_execute_job()` first
  deletes `job_root` without terminating and reaping the already-created child.
- The child can therefore continue consuming processes/threads after its asset
  has been checkpointed as a parent failure, or become unreaped. This is
  especially unsafe in the resource-exhaustion condition that motivated the
  EAGAIN policy, and it permits the parent to remove a job directory while its
  child may still be using it.
- The current post-`Popen` regression uses a fake object with only `pid` and
  `communicate()`. It proves one spawn, no retry sleep, and a fail-closed record,
  but cannot prove termination or reaping and therefore misses the lifecycle
  defect.
- Required invariant: once `Popen` returns, every exit path must prove the child
  is stopped and reaped before deleting the job directory, emitting the parent
  failure record, or advancing the checkpoint. A generic post-spawn exception
  must **not** re-enter the spawn retry loop; terminate the process group, allow
  a short bounded reap, escalate to `SIGKILL` if necessary, and complete
  `wait()`/`communicate()` reaping. Cleanup errors should remain fail closed and
  must not silently certify that the child is gone.
- Required test: return a controllable fake live process whose first
  `communicate()` raises `EAGAIN`; assert exactly one `Popen`, zero retry waits,
  process-group terminate/kill plus final reap as appropriate, no live child at
  record/checkpoint time, and job-directory removal only after reaping.

### Independently verified items

- Pinned Table 2 manifest file hash:
  `3dce6436aac2d25507d7843a3e0e5cbee130e83e0c24c2bfbfb08467ca356290`.
- Pinned Table 2 manifest content hash:
  `f1cc7c062767ec6e6cb8d05caea122f17baf42d1a9b12e5bc40d16648c3306c3`.
- Canonical Table 2 closure contains exactly the seven declared artifacts plus
  the required `.run.lock` sidecar. Its checkpoint is complete with
  `N=720`, `remaining=0`, and `completion_order=720`; all 720 records are
  completed, child-attested, uniquely tokened, and have unique completion
  orders spanning 1 through 720. Manifest, records, summary, checkpoint, and
  artifact receipts agree.
- The exact upstream Table 2 spawn-EAGAIN policy is present in both adapter
  config and environment. Table 3 binds its own retry policy into both config
  and environment, freezes per-asset run tokens in the manifest, preserves the
  running checkpoint on retry-budget exhaustion, and retains formal gates for
  21 samples, 4 workers, 120 seconds, 4,723 joints, and the exact 55 zero-joint
  assets.
- Current reviewed hashes:
  `run_table3_infinite_mobility.py = 5057e86467d0cb1dba2c636c3404f79766952c2738e8ae483575c9c272e78aa8`;
  `test_infinite_mobility_table3.py = f085b9a5b217b999810a1b91d325425708686608a47855557d044c9b100b7448`.

### Dynamic evidence

All four thread variables were fixed to `1`. The focused eight-case command
completed successfully:

```text
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
NUMEXPR_NUM_THREADS=1 exp/.venv_low_medium/bin/python -m pytest -q \
  -p no:cacheprovider \
  --basetemp=exp/runtime/table3_final_dynamic_targeted_tmp \
  [two upstream/live policy tests and four spawn lifecycle tests]
........                                                                 [100%]
8 passed in 5.27s
```

The full suite and `py_compile` were intentionally not started after confirming
the blocking lifecycle defect, to avoid consuming additional process/thread
headroom. After the fix, rerun this eight-case subset, the full current suite,
`py_compile`, and the canonical Table 2 preflight before changing the verdict
to `READY`.

## Lifecycle-fix re-review verdict (2026-08-24)

Verdict: **NOT READY.** The direct post-`Popen` orphaning defect from the prior
section is closed, but one adjacent run-wide lifecycle gate remains missing.
An unreapable owned child is recorded locally yet does not stop queued work and
does not prevent an unsafe resume while that child may still exist.

No formal Table 3 run was started.

### Prior lifecycle finding - CLOSED

`_execute_job()` now treats a successfully created process as owned until it is
proven reaped. On timeout or any other post-`Popen` exception it sends `SIGTERM`,
waits up to two seconds, escalates to `SIGKILL`, waits another two seconds, and
only emits a parent-synthesized failure after successful reaping
(`scripts/run_table3_infinite_mobility.py:1781-1876`, `:1921-2053`). The failure
record carries the termination receipt. If reaping still cannot be proven, it
raises `OwnedProcessLifecycleError`, emits no metric row, and retains the job
directory plus `lifecycle_failure.json` diagnostic. The spawn retry remains
strictly scoped to `Popen`.

The independent four-case lifecycle subset passed and covers timeout reaping,
post-`Popen` EAGAIN reaped after TERM, another post-`Popen` error reaped after
TERM/KILL, and the unreapable fatal path:

```text
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
NUMEXPR_NUM_THREADS=1 exp/.venv_low_medium/bin/python -m pytest -q \
  -p no:cacheprovider \
  --basetemp=exp/runtime/table3_lifecycle_final_targeted_tmp \
  [timeout + post-Popen lifecycle node IDs]
....                                                                     [100%]
4 passed in 3.75s
```

### Important (blocking): unreapable-child failure is not a run-wide or resume gate

- Locations: the production pool submission/catch/raise flow at
  `scripts/run_table3_infinite_mobility.py:2758-2828`; the analogous helper at
  `:2068-2104`; unconditional stale-scratch quarantine at `:2514-2525`; and the
  single-job-only regression at
  `tests/test_infinite_mobility_table3.py:1358-1442`.
- The runner submits every pending job to the executor up front. When one future
  raises `OwnedProcessLifecycleError`, the parent only stores the exception and
  continues the `as_completed()` loop. Exiting the executor context waits for
  queued futures, so workers can keep spawning and evaluating the remaining
  assets while an owned child is explicitly not proven dead. With 720 pending
  assets, the fatal exception may not reach the caller until nearly the entire
  run drains, adding processes and threads in exactly the unsafe condition that
  should stop new spawns.
- The retained `lifecycle_failure.json` is also not a resume gate. On
  `--resume`, `_prepare_worker_scratch()` silently moves the preserved scratch
  tree to a quarantine path and starts a new child with the same frozen token.
  It never proves that the old process group is gone. This can create a second
  evaluation while the first child remains active, defeating the purpose of
  refusing to certify an unreaped child.
- The current test uses `limit=1`, so there is no later queued job whose spawn
  can expose the first issue, and it does not attempt resume, so the second
  issue is untested.
- Required invariant: `OwnedProcessLifecycleError` must set a shared fatal
  event checked before every subsequent `Popen`; pending futures must be
  cancelled or prevented from spawning, while already-created children are
  still driven through bounded cleanup. The run must then return nonzero with
  its last running checkpoint and lifecycle evidence intact.
- Required resume invariant: a preserved lifecycle-failure marker must make
  resume fail closed before quarantine or `Popen` unless an explicit recovery
  procedure has established that the recorded process group no longer exists.
  Silent quarantine is insufficient evidence.
- Required tests: use at least two pending jobs and assert no later `Popen`
  occurs after the first unreapable failure; then invoke `--resume` on the
  retained output and assert rejection before any child spawn while the marker
  is unresolved.

### Reconfirmed bindings

The current Table 2 pins still exactly match the canonical clean publication:
file hash
`3dce6436aac2d25507d7843a3e0e5cbee130e83e0c24c2bfbfb08467ca356290`
and content hash
`f1cc7c062767ec6e6cb8d05caea122f17baf42d1a9b12e5bc40d16648c3306c3`.
The canonical checkpoint remains complete at 720/720; all 720 result tokens and
completion orders are unique, orders span exactly 1 through 720, and both
upstream EAGAIN policy copies remain exact.

Current reviewed hashes are
`run_table3_infinite_mobility.py = a8cb86443f1f44d43223c9c42822520f41358298cbfde518cca622c559658632`
and
`test_infinite_mobility_table3.py = a737d5c59f5b8114b66f36acd017979e71e1f9d8640662d4985af6f93c097a43`.

The full 45-case suite, `py_compile`, and canonical preflight were intentionally
not rerun after confirming this blocker. After the fatal and resume gates are
implemented, rerun the lifecycle subset, all EAGAIN policy/retry cases, the
full suite, `py_compile`, and canonical preflight before issuing `READY`.

## Final readiness verdict (2026-08-24)

Verdict: **FINAL READY.** No blocking findings remain in the reviewed Table 3
runner or its test suite. This latest section supersedes every earlier
`NOT READY` / conditional verdict in this review.

No formal 720-asset Table 3 evaluation was started during review.

### Final finding closure

1. **Run-wide lifecycle fatal gate - CLOSED.** `FatalLifecycleController`
   publishes an unreapable-child failure from the worker under the same lock
   used for actual `Popen` admission. The fatal event also wakes EAGAIN backoff
   waiters. Workers check the gate at entry, after preflight, and at the atomic
   spawn boundary; after fatal publication no further child can be admitted
   (`scripts/run_table3_infinite_mobility.py:179-215`, `:1789-1827`,
   `:1973-2030`).
2. **Scheduler cancellation and owned-child cleanup - CLOSED.** Both the helper
   and production schedulers cancel queued futures, skip cancelled/fatal-gated
   jobs without manufacturing metric rows, and allow only children admitted
   before the fatal linearization point to complete bounded lifecycle cleanup
   (`scripts/run_table3_infinite_mobility.py:2120-2176`, `:2932-3017`). A
   single-worker regression proves no second spawn, while a coordinated
   multi-worker regression proves spawns stop at the already-active worker
   count (`tests/test_infinite_mobility_table3.py:1446-1575`).
3. **Lifecycle-marker resume gate - CLOSED.** Preserved markers expose the
   owned process-group ID at top level and bind it to the pending asset, frozen
   token, job receipt, child PID, and unreaped termination evidence. Scratch is
   walked without following symlinks. Resume probes each PGID with signal zero
   before quarantine: only `ESRCH` proves absence; an existing group, `EPERM`,
   or any unknown probe result rejects resume before `Popen`
   (`scripts/run_table3_infinite_mobility.py:1943-1970`, `:2586-2705`). Tests
   cover existing, unknown, and gone process groups and assert zero spawns for
   both refusal cases (`tests/test_infinite_mobility_table3.py:1578-1680`).
4. **Earlier contracts - RECONFIRMED.** EAGAIN retry remains limited to
   `subprocess.Popen`, uses one cumulative run budget, preserves the same
   manifest-frozen token, wakes and aborts on lifecycle fatal, and retains a
   running checkpoint on exhaustion. Post-`Popen` failures never retry and are
   recorded only after TERM/KILL/reap succeeds. Table 2 compact schema,
   execution attestation, exact closure, complete receipt, Table 3 checkpoint
   sealing, completion-order continuity, 21-state evaluation, 4 workers,
   120-second timeout, 4,723-joint denominator, and exact 55 zero-joint roster
   remain enforced.

### Final dynamic evidence

Every command used `OMP_NUM_THREADS=1`, `OPENBLAS_NUM_THREADS=1`,
`MKL_NUM_THREADS=1`, and `NUMEXPR_NUM_THREADS=1`.

Focused fatal/resume subset:

```text
......                                                                   [100%]
6 passed in 6.20s
```

Full Table 3 suite:

```text
..................................................                       [100%]
50 passed in 21.09s
```

Compilation:

```text
exp/.venv_low_medium/bin/python -m py_compile \
  exp/scripts/run_table3_infinite_mobility.py \
  exp/tests/test_infinite_mobility_table3.py
exit 0
```

The direct read-only canonical preflight ran `validate_contract()` with the
actual low-medium interpreter and `load_inputs(..., formal=True)`. It passed
with Python 3.12.3, NumPy 2.5.1, trimesh 5.0.0, `N=720`, `J=4723`, and 55
zero-joint assets. It validated the clean Table 2 publication and returned:

```text
manifest file:     3dce6436aac2d25507d7843a3e0e5cbee130e83e0c24c2bfbfb08467ca356290
manifest content:  f1cc7c062767ec6e6cb8d05caea122f17baf42d1a9b12e5bc40d16648c3306c3
artifact manifest: 085930ee671e2267db8ab36fef1fc52f176be5dffaf2dac58e1d9d8f572b6da7
records:           d488501734a41d4b814c294f7ad94ed529df72b0f99cfaed8b1d19a3bf1c2ada
summary:           ec071d395b7578c1042b7106a401d94685a47f7566d5214dcad2cdc6a9df51c6
checkpoint:        dda7cd7bf70e99389688684d9123662e1f694725a2e28b9e18c215a5f7b39086
status counts:     completed=720
```

The reviewed files remained unchanged across final verification:

```text
run_table3_infinite_mobility.py  50f11da87296046323f9d6d1330f62b023be70084452da9151e013d10740bb2d
test_infinite_mobility_table3.py d4db86335e3483afa1aeee81421fd322214e6e3cb75de1b89a45946bc3a7b565
```

## Actual formal publication acceptance (2026-08-24)

Verdict: **ACCEPTED - FINAL READY FOR RELEASE.** The completed canonical
publication at `runtime/table3_infinite_mobility_720` passed an independent,
read-only acceptance. No finding remains. This acceptance covers the actual
720-asset output, not only the runner and tests reviewed above. No formal job
was started or resumed during acceptance.

### Exact artifact closure and receipts

The output contains exactly eight top-level regular files, with no directory,
symlink, worker scratch, lifecycle marker, or unlisted artifact. The artifact
manifest has exactly the other seven files and every stored byte count and
SHA-256 receipt matches the current bytes. Independent hashes are:

```text
artifact_manifest.json  b282912a3f4eb3d8288c1664ae52c61e6ba7c8bfde3af31a12d7aedced561313
asset_records.jsonl     e1ebf268e6839869e9d7e8d98e2ae0411e4ed17dea28b5c7692bef326b6f4113
checkpoint.json         d0ae61598e5123ade8659889f3d083217f856f401cadd40a0bfbdc63de519823
environment.json        7eb09c2f7f260b5a8b9b01788febfd1958697f0062495b2e26b4b936339d43e9
manifest.json           52d03061d150e23f5f97e0227931047379969a5518c5448a14e7062a3ed6d611
protocol_snapshot.md    b05a4edbe61037f2dc4bc1bc1580e66f13f852fc24a3b979e130e8a7aa30ef00
report.md               d397bbedf087d29d1e0bf27774b1b648fc256194bd9dada68d169c21f018729f
summary.json            794145ba8962b0320e8e0ef10ecbb9d5405f8e38bbd4cfada4344216f4501e27
```

The manifest self-hash independently recomputes to
`28ac7cec9b80221786c14dca2e546e7ecca73c813a6ad9a101dc43d3d4a6335b`.
The summary carries that same content binding. `protocol_snapshot.md` is
byte-for-byte equal to the live protocol, `environment.json` exactly equals
the frozen environment object, and `report.md` exactly equals a fresh render
from the accepted manifest and summary.

### Input, runtime, and record closure

Formal `load_inputs(..., formal=True)`, contract validation, a fresh manifest
rebuild, and live runtime verification all passed against the current canonical
inputs. Rebuilt source, selection, evaluation, and all 720 frozen manifest rows
are exact. The accepted pins are:

```text
cohort manifest file     cfd9c06ea35dcec57c53d44dbf52903ecba6f33321075495c97c58fe30d23c08
cohort manifest content  f5e29f1becd47cae991f5d238dff3f86b2b009365738df3e46cdbea297032c23
cohort artifact manifest ac31de70d50ed7153178482bb5283659be94fb5945cc2b7157754ac61dfc5439
Table 2 manifest file    3dce6436aac2d25507d7843a3e0e5cbee130e83e0c24c2bfbfb08467ca356290
Table 2 manifest content f1cc7c062767ec6e6cb8d05caea122f17baf42d1a9b12e5bc40d16648c3306c3
Table 2 artifact manifest 085930ee671e2267db8ab36fef1fc52f176be5dffaf2dac58e1d9d8f572b6da7
protocol                   b05a4edbe61037f2dc4bc1bc1580e66f13f852fc24a3b979e130e8a7aa30ef00
adapter                    50f11da87296046323f9d6d1330f62b023be70084452da9151e013d10740bb2d
core evaluator             0da075f077ce13c78bb6b4ee66b0abe77668ccf7bb3c105660b321e667fc2acf
```

The Table 2 completion receipt also remains exact: records
`d488501734a41d4b814c294f7ad94ed529df72b0f99cfaed8b1d19a3bf1c2ada`,
summary
`ec071d395b7578c1042b7106a401d94685a47f7566d5214dcad2cdc6a9df51c6`,
checkpoint
`dda7cd7bf70e99389688684d9123662e1f694725a2e28b9e18c215a5f7b39086`,
with `completed=720`.

The publication preserves the exact Table 2/cohort order for all 720 assets.
Every run token independently re-derives from the frozen protocol, Table 2,
selection, asset, and package binding; all tokens are unique. Runner-level
resume validation rechecked every current package and selected URDF and
validated all 720 records without dropping or recovering a row. All 720 rows
are `completed`, have `error=null`, and originate from `child_attested`; their
720 child PIDs are unique and differ from the parent PID. Before/after package
receipts and expected/observed runtime bindings match on every row. The child
runtime is Python 3.12.3, NumPy 2.5.1, and trimesh 5.0.0 with all four thread
variables set to 1; formal configuration remains 21 samples, 4 workers, and a
120-second asset timeout.

There is no operational error, timeout, EAGAIN failure, termination receipt,
or lifecycle failure in the result rows, and exact output closure proves there
is no retained lifecycle/EAGAIN scratch. The only EAGAIN text in normal
attestation is the required frozen retry-policy description.

### Checkpoint and independent metric recomputation

The checkpoint is `complete`, `completed=720`, `remaining=0`, and
`completion_order=720`. Its manifest file/content bindings, exact JSONL byte
length/hash, ordered IDs, keys, and run tokens all match the publication.
Record completion orders are unique and span exactly `1..720`.

Independent standard-JSON aggregation, without calling the shared core,
reproduced all seven headline cells exactly:

```text
valid_range             4687 / 4723 = 0.9923777260215965
joint_sweep_success     4687 / 4723 = 0.9923777260215965
non_degenerate_motion   4537 / 4723 = 0.9606182511115816
subtree_consistency     4687 / 4723 = 0.9923777260215965
joint_level_pass        4537 / 4723 = 0.9606182511115816
fk_roundtrip_error      measured=4687, passed=4687, denominator=4723,
                        max translation=0.0, max rotation=0.0, status=PARTIAL
strict_kinematic_pass    541 /  720 = 0.7513888888888889
```

The declared and observed joint denominator is exactly `J=4723`. Every joint
binds `sample_count_expected=21`: all 4,687 valid-range joints executed all 21
states, while the 36 invalid-range joints executed zero and remain counted as
failures. There are exactly 55 zero-joint assets (19 TableDining and 36 Vase),
all with an empty joint list and strict failure; the ordered zero-joint roster
hash is
`9b58c190e65e742f36bfe03fcf3a3ead49dc147b9b20230c65cd39bc331c83cf`.
All 720 assets parsed and passed tree validation. Source provenance is 713
original-pass plus 7 recovery-overlay assets.

The category macro was independently regrouped from records and also matches
exactly: 20 asset categories, 19 categories with joints; valid range, sweep,
and subtree rates are `0.9980385746976136`; non-degenerate motion and joint
level rates are `0.9538063565874492`; strict asset rate across 20 categories is
`0.7513888888888889`.

Two independent read-only checks supplied the evidence: the production
runner's exact artifact/input/runtime/resume validation (including live
package rehashing of all 720 rows), and a separate standard-library parser that
recomputed closure, hashes, tokens, attestations, denominators, headline cells,
zero-joint semantics, and category macro. Both exited 0.
