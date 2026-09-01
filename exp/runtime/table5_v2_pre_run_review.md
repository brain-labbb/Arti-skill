# Table 5 revision-2 pre-run review

Status: **HOLD - Infinigen Genesis r2 smoke complete; formal approval/prepared manifest pending**  
Updated: 2026-08-29  
Formal simulator jobs started: **No**

## Frozen Core-200 cohort

The operative selection is:

```text
exp/runtime/table5_v2_core200_five_full_release_articraft10787_infinigen_paired_official/cohort_manifest.json

cohort SHA256   = 660dbeeb01fcfa379c6ec33a572c78553bc06571d586407f92f52dc852781a60
manifest SHA256 = 3085bb408276f71e8c00173405f859c902cfb840645f12b6b5f9ebad83b69ebf
protocol SHA256 = c6f6e4ff3d07a3d925e17685c9693b9c11849b0065057f53d66f6204100ef62b
```

All eight datasets have 200 selected assets. LAM, Artiverse, PartNet-Mobility,
PhysX-Mobility, and SketchMobility now use their complete frozen original-release
rosters, not the superseded N=800 parent rosters. The common eligibility rule is
`1 <= movable_joint_count <= 20`. The `fence`, `sofa_bed`, and `public_toilet`
semantic exclusions apply only to PV-A.

Infinigen-Sim uses the 8,225-identity intersection of the official URDF and MJCF
releases. Genesis and PyBullet use the official URDF; MuJoCo uses the official
MJCF with the same asset ID. No converted replacement format is allowed.

## Revision-2 metric contract

`Import Success` is recorded immediately when the manifest-bound source returns
successfully from the simulator's native load operation:

```text
Genesis:  Scene.add_entity + Scene.build
PyBullet: loadURDF
MuJoCo:   MjModel.from_xml_path
```

Mapping, physics readback/application, first step, FK, and dynamics are separate
diagnostics. A failure after native load does not reverse Import Success.

`Stable Rollout` has a fixed asset denominator of 200. Every imported asset must
actually complete the frozen number of zero-applied-force passive steps under the
frozen gravity, timestep, and per-engine solver configuration. Every mapped joint
state and observed rigid-body pose must remain finite, and mapping must remain
unchanged. No eligible or mapped DoF is required, but a zero-DoF asset must still
execute every step; an empty joint list is not itself a pass.

Physics policy is released-first/native-fallback. Valid released inertial fields
are consumed by the simulator. PV-A's frozen `physics.json` is compiled into
mass, COM, and inertia only where source inertial data is absent or invalid. If a
field is unavailable, the simulator handles it by its native rules; missing
physics metadata alone is not an import or preflight failure. Dynamic metrics and
physics receipts expose the resulting behavior.

All failures remain in the fixed N=200 denominator. Assets are never deleted,
replaced, repaired, renamed, structurally rewritten, or supplemented after a
failure.

## Bound implementation

```text
protocol schema  = table5_v2_runtime_protocol_v2
protocol ID      = table5-v2-readiness-portability-v2
metric semantics = table5-v2-native-import-passive-stability-r2
computed protocol SHA256 = 18fa96094dc0c3ffff16e003f5a9f13199fd026e8fe429ac17fc9978c66f3be5
passive rollout steps = 240

prepare binder   = exp/scripts/table5_v2_prepare_r2.py
runtime          = exp/scripts/table5_v2_runtime_r2.py
aggregate        = exp/scripts/table5_v2_aggregate_r2.py
formal launcher  = exp/scripts/run_table5_v2_formal_tmux.sh
```

The computed protocol hash is the expected value for the current source tree; it
is not formally frozen until an r2 prepared manifest containing that exact hash
exists and passes verification.

The runtime explicitly binds the frozen core runtime, the Genesis coincident
fixed-root coordinate-mapping compatibility layer, the evaluator, physics code,
and r2 aggregate by source hash. The compatibility layer neither edits an asset
nor invents an observed link.

## Current blocker

The required formal prepared manifest does not exist:

```text
exp/runtime/table5_v2_core200_prepared_five_full_release_articraft10787_infinigen_paired_official_metrics_r2/manifest.json
```

The old prepared manifest belongs to the superseded parent-roster cohort and old
metric protocol and must not be used. Under the current instruction not to rerun
prepare, no r2 simulator job can be started. The formal launcher verifies the
exact r2 prepared manifest before creating a tmux session and verifies it again
inside the session, so it fails closed at this boundary.

## Smoke result and boundary

The completed PyBullet and MuJoCo smoke runs are retained and were not rerun. The
original seven datasets' completed Genesis smoke runs are also retained.

The only requested new smoke, Infinigen-Sim on Genesis under r2, completed on the
two frozen smoke assets:

| Asset ID | Native Import | DoF Mapping | Passive Stable Rollout | Physics |
|---|---:|---:|---:|---|
| `dishwasher/1196` | pass | 7 / 7 | pass, 240 / 240 steps | Genesis native fallback |
| `microwave/1208` | pass | 5 / 5 | pass, 240 / 240 steps | Genesis native fallback |

Both terminal records are `completed`; every mapped state and observed link pose
was finite for all 240 steps, mapping was unchanged, and both worker stderr logs
are empty. The r2 smoke manifest is smoke-only and cannot contribute to formal
N=200 results:

```text
exp/runtime/table5_v2_eight_full_release_three_sim_smoke_r2/prepared/manifest.json
manifest SHA256 = 1c973877c2752016e4f212cc9ff5cf57de85d613810ff50f4ee9cfc9ac0901af
protocol SHA256 = 18fa96094dc0c3ffff16e003f5a9f13199fd026e8fe429ac17fc9978c66f3be5
formal_result_eligible = false
```

The generic runtime summary still exposes legacy boolean fields such as `load`;
they are not r2 Import Success. The hash-bound r2 receipts above are operative.
Stop here and do not start formal N=200 without separate approval.

## Formal launch after separate approval

Only after the smoke report and explicit formal approval:

```bash
bash exp/scripts/run_table5_v2_formal_tmux.sh
```

The launcher creates the `table5-v2-r2-formal` tmux session and runs all three
simulators with exactly 5 workers. Its target output is:

```text
exp/runtime/table5_v2_r2_formal_eight_datasets
```
