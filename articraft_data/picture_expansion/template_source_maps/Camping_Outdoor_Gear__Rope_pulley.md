<!--
subcategory_contract:
  category: Camping_Outdoor Gear
  subcategory: Rope pulley
  core_identity: a rope block/pulley whose grooved sheave (wheel) turns freely on an axle to redirect or gain mechanical advantage on a rope
  must_keep:
    - at least one grooved sheave as a real turning child (CONTINUOUS/REVOLUTE joint about the axle, axis (0,1,0))
    - a side-plate/shell body that captures the sheave on an axle
    - a rope-attachment interface (eye/hook/swivel/shackle/becket) to a mounting point
  must_not_become:
    - carabiner / snap-hook / rappel ring with no wheel
    - chain hoist / winch / gin wheel / capstan
    - bare pulley wheel with no housing, or a cleat with no sheave
  image_evidence:
    - 003.png: compact DOUBLE block, stacked silver cheek plates + dark inner plates, two grooved sheaves, swivel hooks top & bottom, coiled black rope; inset shows a bearing sheave
    - 002.png: single stainless block "STAINLESS 15", swivel eye + carabiner, exposed sheave, threaded rope; kit inset shows swivel pulleys + snap hooks + carabiners
    - 001.png: lineup of single-sheave pulleys - SRLF light weight, SRLF with side plates, SG rope glider (tube), SRLKG ring/tube pulley
  parent_evidence:
    - o3 (ad2de3): parts frame/sheave; helpers _side_plate_geometry (pear cheek with top eye slot + axle bore), _grooved_sheave_geometry; frame_to_sheave CONTINUOUS; fixed integral eye
    - o2 (8db781): parts housing/sheave/upper_swivel/rope; helper _pulley_cheek_mesh (oval cheek w/ window); housing_to_sheave CONTINUOUS, housing_to_upper_swivel CONTINUOUS (swivel eye + carabiner clip)
    - o1 (3508f0): parts frame/upper_sheave/lower_sheave/top_hook/bottom_hook/rope; helpers _plate_mesh,_sheave_mesh,_hook_mesh; two CONTINUOUS sheave joints + two REVOLUTE swivel hooks
-->

# Camping_Outdoor Gear / Rope pulley — template source map
pattern: mixed (parallel_children for attachment/mechanism slots + multiplicity on sheave count)
parents:
- rec_camping_outdoor_gear__rope_pulley_ad2de311f270426e89efa649f8081484 (o3) — picture/Camping_Outdoor Gear/Rope pulley/001.png — single sheave, fixed integral eye, open cheek plates
- rec_camping_outdoor_gear__rope_pulley_8db7819e58414e1f9df89ef1651224ab (o2) — picture/Camping_Outdoor Gear/Rope pulley/002.png — single sheave, swivel eye + carabiner
- rec_camping_outdoor_gear__rope_pulley_3508f00573934a1e82b506b8bf7688c2 (o1) — picture/Camping_Outdoor Gear/Rope pulley/003.png — double sheave, swivel hooks top & bottom
canonical_baselines: none
underfilled_reason: none (normal richness; 3 origins + 10 forks = 13 counted anchors)

## Slot Candidates
| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints | status |
|---|---|---|---|---|---|---|
| sheave_multiplicity | single (N=1) | N | origin_anchor | o3, o2 | sheave / frame_to_sheave (CONTINUOUS) | converged |
| sheave_multiplicity | double (N=2) | N | origin_anchor | o1 | upper_sheave, lower_sheave / frame_to_*_sheave | converged |
| sheave_multiplicity | triple (N=3) | N | forked_anchor | rec_rope_pulley_var_triple_block (from o1) | 3x sheave loop / frame_to_sheave_i CONTINUOUS | converged |
| side_plate_construction | fixed twin cheek plates | ① | origin_anchor | o3, o1 (also o2 oval cheeks) | front_plate, rear_plate / _side_plate_geometry | converged |
| side_plate_construction | swinging/hinged side plate (snatch block) | ① | forked_anchor | rec_rope_pulley_var_snatch_block (from o3) | swing_plate / frame_to_swing_plate REVOLUTE | converged |
| body_form | open side-plate block (planar) | ③ | origin_anchor | o3, o1 | front_plate/rear_plate | converged |
| body_form | solid enclosed shell / mortise block (volumetric) | ③ | forked_anchor | rec_rope_pulley_var_shell_block (from o3) | shell body + internal axle_pin / frame_to_sheave | converged |
| body_form | tubular fairlead / rope-glider shell | ③ | forked_anchor | rec_rope_pulley_var_tube_fairlead (from o2) | tube housing / housing_to_sheave | converged |
| attachment | fixed integral eye | ② static | origin_anchor | o3 top_slot | eye cut in _side_plate_geometry (FIXED) | converged |
| attachment | swivel eye | ② | origin_anchor | o2 upper_swivel | housing_to_upper_swivel CONTINUOUS | converged |
| attachment | swivel hook | ② | origin_anchor | o1 top_hook/bottom_hook | frame_to_top_hook / frame_to_bottom_hook REVOLUTE | converged |
| attachment | carabiner gate clip | ② | origin_anchor | o2 top_clip_loop | gate_hinge_knuckle on upper_swivel | converged |
| attachment | rigid fixed hook | ② | forked_anchor | rec_rope_pulley_var_fixed_hook (from o1) | top_hook / frame_to_top_hook FIXED | converged |
| attachment | snap shackle / spring gate | ② | forked_anchor | rec_rope_pulley_var_snap_shackle (from o2) | snap head + gate REVOLUTE on upper_swivel | converged |
| attachment | captive U-bail / shackle head | ③ | forked_anchor | rec_rope_pulley_var_bail_shackle (from o3) | U-bail straddling cheeks (FIXED) | converged |
| attachment | becket (fixed lower rope-anchor eye) | ③ | forked_anchor | rec_rope_pulley_var_becket (from o1) | becket eye at bottom_neck (FIXED) | converged |
| rope_control | plain sheave | ② | origin_anchor | o1, o2, o3 | sheave / *_to_sheave CONTINUOUS | converged |
| rope_control | cam cleat / rope ratchet (self-locking) | ② | forked_anchor | rec_rope_pulley_var_cam_cleat (from o2) | cam_jaw / housing_to_cam REVOLUTE | converged |
| rope_control | progress-capture / prusik-minding | ② | forked_anchor | rec_rope_pulley_var_progress_capture (from o3) | capture_cam / frame_to_capture_cam REVOLUTE | converged |

## Multiplicity / Copy Logic
- count_param: sheave list, from o1's `for z, label in [(0.035,"upper_sheave"),(-0.035,"lower_sheave")]` (loop-emitted already)
- N samples: N=1 (o3, o2), N=2 (o1), N=3 (rec_rope_pulley_var_triple_block)
- suggested N_range: 1–4 (single up to quad block; block-and-tackle rarely exceeds 4 sheaves per block)
- copied object: grooved sheave (_sheave_mesh / _grooved_sheave_geometry)
- naming: indexed sheave_0 / sheave_1 / sheave_2
- placement: evenly spaced coaxial stack along z, all on the block axle
- joint policy: each sheave gets its own frame_to_sheave_i CONTINUOUS joint, axis (0,1,0); side plates widened to capture the stack

## Six-Axis Diversity Record
| axis | treatment | values / range / reason |
|---|---|---|
| ① skeleton / structural topology | source-backed | fixed twin cheek plates (origins) vs swinging/hinged cheek (snatch block, forked); axle-captured sheave in all |
| ② joint / mechanism type | source-backed | CONTINUOUS sheave spin (all); swivel eye CONTINUOUS (o2); swivel hook REVOLUTE (o1); fixed eye/hook FIXED; added self-locking cam REVOLUTE (cam cleat, progress capture); snap-shackle gate REVOLUTE |
| ③ primary form family | source-backed | planar open side-plate block (origins); volumetric solid shell/mortise block (forked); tubular fairlead shell (forked); attachment-interface families: eye / hook / swivel / bail / becket |
| ④ surface decoration | record_only / world_knowledge_extrapolation | stamped load rating ("15" on o2), rotation witness mark (o3), rivet/collar heads; host-conformal stamps only |
| ⑤ proportion / size / travel | record_only | sheave dia ~16–26 mm; single vs double block height; groove width to rope dia; shell fullness — ride-along only |
| ⑥ material / palette / finish | record_only | brushed/polished stainless (o2), silver + dark inner steel (o1), anodized orange (o3), black braided rope; colorway companions (anodized red/blue/black, hi-vis, varnished wood) allowed as ride-alongs |

## Compatibility Probes
| probe | record_id | combined axes | risk tested | result |
|---|---|---|---|---|
| triple + becket | rec_rope_pulley_probe_triple_becket | N multiplicity + becket attachment | becket clearance against widened 3-sheave stack; rope re-route without collision | converged |

## Blocked / Excluded
- pure carabiner / snap hook / rappel ring (kit insets in 002/001): neighbor category, no turning sheave — excluded (must_not_become)
- aluminium sleeve / rope thimble (kit inset in 002): not a pulley — excluded
- chain hoist / winch / gin wheel: neighbor category — excluded
- ④/⑤/⑥-only variants (colorway, groove-width, stamped rating): audit-only, never standalone — recorded, not forked
