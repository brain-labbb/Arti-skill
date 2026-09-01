# T2 authoring development pilot

This directory freezes the minimum viable T2 Panel A experiment: 12 tasks, four method arms,
and three independent repeats (144 authoring runs). It also freezes the downstream 32 coverage
seeds plus four explicit boundary configurations used by Panel B.

## Claim boundary

The 12 tasks already have completed SourceMaps, TemplateDesigns, and templates. The historical
template is hidden from the author and retained only for evaluator-side leakage checks. This makes
the cohort useful for reconstruction, infrastructure, variance, and ablation debugging, but not for
the paper's final unseen-task success-rate claim.

The final paper cohort still needs freshly selected source categories, frozen SourceMaps/Designs
where appropriate, and authoring runs performed before any target template exists. Candidate source
pools already identified in `articraft_data` include `flip_phone`, `glove_compartment_door`,
`flatbed_scanner_with_hinged_lid`, `clamp_meter_with_hinged_jaw_and_rotary_selector`,
`adjustable_weight_bench_with_hinged_backrest`, `bicycle_dropper_seatpost_assembly`,
`folding_kick_scooter`, `garden_gate`, `extension_ladder`, `dock_loading_ramp`,
`dualcolumn_lift_carriage`, and `dualrail_gantry_axis`. These names are a candidate pool, not a
frozen paper cohort.

## Method isolation

Every arm receives the identical raw record pool and shared SDK/authoring rules.

| Arm | SourceMap | TemplateDesign |
|---|---:|---:|
| `naive_same_llm` | no | no |
| `without_source_map` | no | yes |
| `without_template_design` | yes | no |
| `full_ours` | yes | yes |

The target template, historical outputs, evaluator state, and other arms' outputs are forbidden.
This is an allowlist contract: an executor must materialize only `allowed_inputs` in an isolated
worktree or sandbox. Merely telling an agent not to open an accessible file is insufficient.

## Prepare and resume

Preparing packets is local and makes no provider/API calls:

```bash
python exp/scripts/run_t2_authoring_pilot.py prepare --run-id dev_v2
```

This creates 144 immutable `packet.json` files below
`exp/runtime/t2_authoring_pilot/dev_v2/runs/`. Re-running the command is resumable: it verifies the
protocol, task manifest, and packets and refuses changed inputs.

Before a real paid run, freeze one exact model identifier for every arm:

```bash
python exp/scripts/run_t2_authoring_pilot.py prepare \
  --run-id <formal-run-id> \
  --model-id <provider/model-version>
```

This command only marks the manifest ready; it does not invoke the model. The authoring executor is
intentionally external because the repository currently has no canonical automated
template-authoring model entrypoint. It must enforce packet allowlists, record usage/cost telemetry,
run the same hidden evaluator for each arm, and write `authoring_result.json` according to
`schemas/authoring_result.schema.json`.

Validate and aggregate any completed results with:

```bash
python exp/scripts/run_t2_authoring_pilot.py status \
  exp/runtime/t2_authoring_pilot/<run-id>
```

Invalid result files make the command exit nonzero. Missing files remain `pending`, so interrupted
runs can resume without rewriting completed results.

## Metrics and gates

First-shot and final authoring success use the same hidden contract/domain/random-16 evaluator;
final success permits at most three automated repair turns. Distribution reliability is evaluated
only after authoring using random-36, explicit corners, and regression retention. Time, input/output
tokens, and API cost are mandatory result fields, allowing T7 telemetry to be collected during T2.
