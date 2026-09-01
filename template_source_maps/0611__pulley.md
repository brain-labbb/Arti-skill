# 0611 / pulley — template source map
pattern: mixed / multiplicity
parents: rec_use-the-attached-reference-image-as-the-primary-_20260712_064522_904114_797df9d4 — pictureY/0611/pulley/001.png
canonical_baselines: none
status: P1 complete; rejected coaxial swing-side-plate v2 deleted and rear-hinged side-gate v3 converged

## Slot Candidates

| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints | status |
|---|---|---|---|---|---|---|
| sheave_count | N=1 | N | origin_anchor | rec_use-the-attached-reference-image-as-the-primary-_20260712_064522_904114_797df9d4 | sheave / sheave_axle | converged |
| sheave_count | N=2 | N | forked_anchor | rec_0611_pulley_var_sheave_count_2 | sheave_i / sheave_axle_i / `for i in range(sheave_count)` | converged |
| sheave_count | N=3 | N | forked_anchor | rec_0611_pulley_var_sheave_count_3 | sheave_i / sheave_axle_i / `for i in range(SHEAVE_COUNT)` | converged |
| frame_construction | open twin cheeks | ① / ③ | origin_anchor | rec_use-the-attached-reference-image-as-the-primary-_20260712_064522_904114_797df9d4 | mount_frame / _mount_frame_shape | converged |
| frame_construction | rear-hinged side gate / snatch block | ① / ② | forked_anchor | rec_0611_pulley_var_frame_hinged_side_gate_v3 | gate_cheek / gate_hinge / gate_plate_slot_barrel_lug / hinge_pin / quick_release_latch | converged |
| frame_construction | enclosed shell block | ③ | forked_anchor | rec_0611_pulley_var_frame_enclosed_shell | mount_frame / enclosed_pulley_shell / sheave_axle | converged |
| mount_interface | fixed two-hole plate | ① | origin_anchor | rec_use-the-attached-reference-image-as-the-primary-_20260712_064522_904114_797df9d4 | mount_plate_cheeks_axle | converged |
| mount_interface | closed eye | ① | forked_anchor | rec_0611_pulley_var_mount_closed_eye | closed_eye_frame / closed_eye_cheeks_axle / sheave_axle | converged |
| mount_interface | swivel hook | ① / ② | forked_anchor | rec_0611_pulley_var_mount_swivel_hook | mount_frame / load_hook / hook_swivel | converged |
| mount_interface | clevis/shackle | ① | forked_anchor | rec_0611_pulley_var_mount_clevis | mount_interface / clevis_ears_bridge / attachment_pin | converged |
| secondary_module | none | ① | origin_anchor | rec_use-the-attached-reference-image-as-the-primary-_20260712_064522_904114_797df9d4 | no secondary anchor | converged |
| secondary_module | lower becket eye | ① | forked_anchor | rec_0611_pulley_var_secondary_becket | mount_frame / becket_eye / becket_neck | converged |
| sheave_profile | deep U-groove rope sheave | ③ | origin_anchor | rec_use-the-attached-reference-image-as-the-primary-_20260712_064522_904114_797df9d4 | _grooved_sheave_geometry / grooved_sheave | converged |
| sheave_profile | narrow V-groove cable sheave | ③ | forked_anchor | rec_0611_pulley_var_sheave_profile_narrow_v | _grooved_sheave_geometry / narrow_v_groove / sheave_axle | converged |

## Multiplicity / Copy Logic

- count_param: `sheave_count`
- N samples: N=1 origin; N=2 and N=3 converged forks
- suggested N_range: 1–4
- copied object / naming / placement / joint policy: `sheave_i` + `sheave_axle_i`; uniform X-axis spacing; independent CONTINUOUS joint per sheave; loop-emitted with `for i in range(sheave_count)`

## Six-Axis Diversity Record

| axis | treatment | values / range / reason |
|---|---|---|
| ① skeleton / structural topology | source-backed | fixed plate/open cheeks; swing-open cheek; enclosed shell; closed eye; swivel hook; clevis; becket |
| ② joint / mechanism type | source-backed | continuous sheave axle; hinged snatch cheek; continuous hook swivel |
| ③ primary form family | source-backed | planar bent-sheet frame; volumetric shell; deep U and narrow V sheave profiles |
| ④ surface decoration | record_only / world_knowledge_extrapolation | smooth stampings, shallow reinforcing beads, stamped markings; host-conformal only |
| ⑤ proportion / size / travel | record_only | 20–80 mm sheave diameter; 0.3–2 mm cheek clearance; realistic light-duty through utility scale |
| ⑥ material / palette / finish | record_only / companion | brushed stainless, galvanized steel, black anodized aluminium, marine bronze, restrained safety-red hook |

## Compatibility Probes

| probe | record_id | combined axes | risk tested | result |
|---|---|---|---|---|
| none | — | — | ordinary variants preserve all non-target interfaces | not required |

## Blocked / Excluded

- plain wheel/caster: lacks rope groove and supported pulley-block identity.
- gear/toothed sprocket: changes the load medium and neighboring category.
- cable reel/winch drum: stores rope instead of redirecting it across a sheave.
- material-only, color-only, or scale-only forks: axes ⑤/⑥ are record/companion ranges, not anchors.
- rejected `rec_0611_pulley_var_frame_swing_open_cheek`: human visual review rejected the off-axis lower-pedestal flap design; record deleted.
- rejected `rec_0611_pulley_var_frame_swing_side_plate_v2`: human visual review rejected the coaxial in-plane swing-plate design; record deleted.

## Verification Notes

- Nine retained variants were generated from the single origin parent with `provider=openai`, `model_id=gpt-5.6-sol`: eight use `thinking_level=xhigh`, and the human-requested frame gate replacement uses `thinking_level=medium`.
- All nine retained variants were recompiled with `articraft compile --target full --validate --strict-geom-qc`; all reports have `status=success`. The v3 gate report has 0 failures and 0 warnings, and explicitly checks its rear-edge Z-axis `gate_hinge`, full-size cheek, closed axle slot/latch contact, 15 mm lateral travel, 8 mm sheave clearance, and 10 mm exposed rope path.
- Every record remains in `collections=['workbench']`, carries the origin as `parent_record_id`, and reconciles to `pictureY/0611/pulley/001.png` in `data/index/subcat/0611__pulley.jsonl`.
- Expected warning class: moving sheaves (and the swivel hook) use explicit `allow_isolated_part` justifications because their visible shafts/bearings retain real running clearance rather than intersecting the child geometry.
- No ordinary variant combines multiple candidate-anchor axes; the hinged gate and swivel hook include only the localized joint required by their named primary structural interface.
