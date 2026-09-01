# T2–T5 execution status — 2026-08-07

> **Superseded:** this was an early fail-closed readiness snapshot. The subsequent real runs and
> final interpretation are recorded in `exp/T2-T5-真实实验报告-20260807.md`; do not use the
> `blocked`/`not_run` statements below as the final experiment status.

## Bottom line

The experiment directions are workable, but the current repository supports a mixture of
reconstruction pilots, operational proxies, and incomplete formal protocols. The runs below were
kept fail-closed: missing independent gold, semantic partitions, reviewers, simulators, or a frozen
authoring model remain `N/A`/blocked rather than being inferred from generated outputs.

## T2 — Template Authoring and Distribution Reliability

- Frozen development cohort: 12 tasks, balanced 4 simple / 4 medium / 4 complex.
- Arms: naive same-LLM, without SourceMap, without TemplateDesign, and Full Ours.
- Repeats: 3; total authoring packets: 144.
- Every task's SourceMap, TemplateDesign, hidden historical template, and referenced raw records
  resolve successfully.
- Raw records and shared rules are identical across arms. Withheld task evidence and the historical
  target template are explicitly forbidden, and the executor is required to enforce an allowlist.
- Packets are resumable and result telemetry includes first-shot/final success, repairs, human
  intervention, wall time, tokens, and API cost.
- Status: `execution_ready=false` until an exact same-model backend and isolated authoring executor
  are frozen. This is a reconstruction/infrastructure pilot, not unseen-task paper evidence.

Relevant files:

- `exp/t2_authoring_pilot/protocol.json`
- `exp/scripts/run_t2_authoring_pilot.py`
- `exp/runtime/t2_authoring_pilot/dev_v2/`

Existing Panel B evidence remains 1,188/1,188 seed compile and Full-QC, 33/33 templates at 36/36,
and 231/231 project-native corners. It evaluates existing templates, not newly authored arms.

## T3 — Semantic Structure and Frozen Constraints

The source-derived constraint audit was rerun from the frozen 33×36 packages. All headline values
exactly match the previous run:

| Metric | Reproduced result |
|---|---:|
| Constraints | 17,706 |
| Coverage | 17,706/17,706 = 1.0000 |
| Satisfaction / conditional pass | 17,125/17,706 = 0.9672 |
| Count pass | 3,345/3,620 = 0.9240 |
| Numeric pass | 6,402/6,402 = 1.0000 |
| Relational pass | 3,814/4,120 = 0.9257 |
| Interface / kinematic / valid-config compatibility | 1,188/1,188 each |
| All-pass assets | 960/1,188 = 0.8081 |
| All-seeds-pass templates | 25/33 |

Reproduced output: `exp/runtime/nano3d_constraints_repro_20260807/`.

This remains source-derived and operational. Independent semantic precision/recall judges,
parent-child/hierarchy gold, and a frozen invalid-combination manifest are still absent. Therefore
the result is suitable as a local audit or supplementary proxy, not the final independent T3 row.

## T5 — Articulation, Collision, and Simulation Readiness

The original selected-export paths had decayed: 10 of 33 packages no longer existed. The rerun uses
the preserved 33-package GLB input manifest and verifies every frozen URDF hash before evaluation.
The functional metrics exactly reproduce the old values:

| Metric | Reproduced result |
|---|---:|
| Assets / movable joints | 33 / 186 |
| Single-joint states (11 per joint) | 2,046/2,046 collision-free |
| Multi-joint Sobol states (64 × 24 assets) | 1,536/1,536 collision-free |
| Combined swept states | 3,582/3,582 collision-free |
| Joint single-sweep pass | 186/186 |
| Asset collision-free proxy | 33/33 |

Reproduced output: `exp/runtime/nano3d_articulation_repro_v2_20260807/`.

The collision result is a PyBullet discrete reset-and-step proxy with direct parent-child
self-collision excluded; CCD and low-clearance adaptive refinement are not run. Joint type/recall,
parent-child correctness, axis error, origin error, and limit error remain `N/A` without frozen gold.

Panel C is not ready. Of 241 links, 239 have collision geometry, 41 have a positive finite inertial
record, and only 39/241 (16.18%) have both; only 8/33 assets are complete under this minimum
definition. MuJoCo/Genesis/PyBullet/Isaac L5, rest stability, and worst-state stability are unrun.

## T4 — Distributional Editability

The 18-task single-seed slice was rerun and exactly reproduces the old deterministic summary:

- artifact valid: 18/18;
- source + GLB changed: 18/18;
- hierarchy preserved: 18/18;
- token-based target handle: 14/18;
- all deterministic gates: 14/18;
- failed target-handle tasks: E001, E002, E003, E017;
- human review and adjudication: not submitted.

Reproduced output: `exp/runtime/nano3d_paper_editability_repro_20260807/`.

The desired 18×16 extension is now frozen as 288 unique valid edits. No case is a no-op; additive
integer edits are exactly `+1`, and replacement seeds already containing the target are excluded.
Manifest: `exp/runtime/t4_distributional_protocol_v1/cases.jsonl`.

Execution remains blocked until an independent party freezes target roles, allowed dependents, and
true non-target roles for all 18 tasks. A pre-edit regression manifest and two blinded reviewers plus
adjudication are also required. Compiling 288 artifacts before those definitions exist would not make
Non-Target Preservation, Locality, Regression Preservation, or Final Pass measurable.

## Recommended next gates

1. Freeze the exact T2 model/executor and run a small 1-task × 4-arm smoke before launching 144
   authoring calls.
2. Create one shared independent annotation pass covering T3 semantic/hierarchy gold, T5 joint gold,
   and T4 semantic role partitions; version and hash it before scoring.
3. Regenerate a physics-complete T5 cohort, then run simulator availability/L1–L5 preflight before
   committing to 4-simulator results.
4. Only after T4 semantic partitions and regression inputs are frozen, compile and score the 288
   distributional edit cases and send the blinded packet to reviewers.
