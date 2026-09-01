<!--
subcategory_contract:
  category: Emergency Equipment
  subcategory: Defibrillator case
  core_identity: A dedicated protective case/enclosure that holds one AED/defibrillator and is opened by a real hinged door/lid (or equivalent opening mechanism) to retrieve the device.
  must_keep:
    - protective enclosure body sized to one AED device
    - at least one real non-fixed opening joint (hinged door/lid, or slide/shutter that opens the case)
    - an AED module/device retained inside (cradle, shelf, or slide-out)
  must_not_become:
    - [general storage cabinet, mailbox/meter box, toolbox/camera case, first-aid soft bag without AED identity, safe/lockbox, display case]
  image_evidence:
    - 001.png: soft red/black molded AED carry case, open clamshell lid with printed instruction panels, front black velcro flap, side D-rings, AED device + electrode cable inside
    - 002.png: white powder-coated wall AED cabinet, side-hinged front door with clear viewing window, red DEFIBRILLATOR title + green AED cross signage, side vent slots, "ALARM WILL SOUND" note, yellow HeartSine AED inside on a shelf
  parent_evidence:
    - origin1 (carry case): parts case_base/lid/aed_device/front_flap; joints base_to_lid REVOLUTE, base_to_front_flap REVOLUTE, base_to_aed FIXED; helpers _rounded_plate/_rounded_ring/_lid_origin; loop red_icon_panel_{0..2}
    - origin2 (wall cabinet): parts cabinet/door/aed_module; joints cabinet_to_door REVOLUTE (vertical axis), cabinet_to_aed PRISMATIC; helpers _cabinet_shell/_door_frame/_aed_case; loops side_vent_{0..4}, hinge_knuckle_{0..2}
-->

# Emergency Equipment / Defibrillator case — template source map
pattern: mixed (parallel_children per origin + multiplicity on ventilation slats)
parents:
- origin1 = rec_emergency_equipment__defibrillator_case_4aa24e0337dd40a396b6e21015f06fc2 — picture: picture/Emergency Equipment/Defibrillator case/001.png (soft molded AED carry case, clamshell lid)
- origin2 = rec_emergency_equipment__defibrillator_case_ef2187bc75d84ba68fccbb2378af9e33 — picture: picture/Emergency Equipment/Defibrillator case/002.png (wall-mounted AED cabinet, hinged front door)
canonical_baselines: none
underfilled_reason: none (14 candidate anchors = 2 origins + 12 planned forks, within normal 12–18)

## Slot Candidates
| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints | status |
|---|---|---|---|---|---|---|
| body_form | wall_cabinet_box | ③ | origin_anchor | origin2 / 002.png | cabinet, shell, cabinet_to_door | converged |
| body_form | molded_soft_carry_case | ③ | origin_anchor | origin1 / 001.png | case_base, lid, base_to_lid | converged |
| body_form | pole_mount_enclosure | ③ | forked_anchor | rec_defibrillator_case_var_pole_mount (fork origin2) | cabinet, dark_back_panel + clamp collar/arms | converged |
| body_form | outdoor_weatherproof_cabinet | ③ | forked_anchor | rec_defibrillator_case_var_outdoor_cabinet (fork origin2) | shell + sloped roof cap, hooded side_vent | converged |
| body_form | rigid_hard_shell_case | ③ | forked_anchor | rec_defibrillator_case_var_hardshell_case (fork origin1) | case_base, lid ribbed shell, snap latch | converged |
| body_form | soft_shoulder_bag_pouch | ③ | forked_anchor | rec_defibrillator_case_var_shoulder_pouch (fork origin1) | case_base pouch, lid fold-over flap, side_d_ring strap | converged |
| opening_or_motion | front_hinged_door_vertical_axis | ② | origin_anchor | origin2 | door, cabinet_to_door REVOLUTE axis(0,0,1) | converged |
| opening_or_motion | clamshell_top_hinged_lid | ② | origin_anchor | origin1 | lid, base_to_lid REVOLUTE | converged |
| opening_or_motion | drop_down_flap_door | ② | forked_anchor | rec_defibrillator_case_var_dropdown_flap (fork origin2) | door, cabinet_to_door re-axed (1,0,0) bottom hinge | converged |
| opening_or_motion | clear_flip_up_cover | ② | forked_anchor | rec_defibrillator_case_var_clear_flip_cover (fork origin2) | door transparent cover, top-hinged cabinet_to_door | converged |
| opening_or_motion | roll_up_shutter | ① | forked_anchor (probe) | rec_defibrillator_case_var_roll_shutter (fork origin2) | shutter_slat_i loop + PRISMATIC opening | converged |
| latch_or_handle | pull_handle | ②/record | origin_anchor | origin2 | pull_handle, handle_*_post (static) | converged |
| latch_or_handle | velcro_strap_flap | ② | origin_anchor | origin1 | front_flap, base_to_front_flap REVOLUTE | converged |
| latch_or_handle | rotary_knob_latch | ② | forked_anchor | rec_defibrillator_case_var_rotary_latch (fork origin2) | knob body + CONTINUOUS/REVOLUTE face-normal latch | converged |
| alarm_signage_module | alarm_strobe_beacon | ① | forked_anchor | rec_defibrillator_case_var_alarm_strobe (fork origin2) | strobe_beacon fixed to shell, status_led | converged |
| aed_retention | fixed_cradle | ②/record | origin_anchor | origin1 | aed_device, base_to_aed FIXED | converged |
| aed_retention | prismatic_slideout_shelf | ② | origin_anchor | origin2 | aed_module, cabinet_to_aed PRISMATIC | converged |
| aed_retention | swing_out_revolute_bracket | ② | forked_anchor | rec_defibrillator_case_var_swingout_bracket (fork origin2) | cradle arm, cabinet_to_aed → REVOLUTE vertical | converged |
| multiplicity(vents) | vent_slots N=5 | N | origin_anchor | origin2 | side_vent_{0..4} loop | converged |
| multiplicity(vents) | vent_slots N=9 (dense) | N | forked_anchor | rec_defibrillator_case_var_vent_dense (fork origin2) | side_vent_i loop, matching shell cut | converged |
| multiplicity(vents) | vent_slots N=3 (sparse) | N | forked_anchor | rec_defibrillator_case_var_vent_sparse (fork origin2) | side_vent_i loop, matching shell cut | converged |

## Multiplicity / Copy Logic
- count_param: number of `side_vent_i` slats (with matching cut in `_cabinet_shell`)
- N samples: 5 (origin2), 9 (dense fork), 3 (sparse fork)
- suggested N_range: 3–12 evenly spaced louver slats
- copied object / naming / placement / joint policy: copied object = a single vent slat box + matching shell cut-through; naming = stable indexed `side_vent_i`; placement = regular vertical spacing on the right side wall; joint policy = static fixed decoration with cut-throughs (no articulation). Secondary loop available for `hinge_knuckle_i` (origin2, N=3) and `red_icon_panel_i` (origin1, N=3) but not sampled as its own axis.

## Six-Axis Diversity Record
| axis | treatment | values / range / reason |
|---|---|---|
| ① skeleton / structural topology | source-backed | wall cabinet vs carry case vs pole-mount vs pouch; added alarm-strobe subassembly; roll-shutter slat topology |
| ② joint / mechanism type | source-backed | side-hinged door (axis 0,0,1), top clamshell lid, bottom drop-flap (axis 1,0,0), top flip-up cover, prismatic shutter, prismatic slide-out AED, revolute swing-out AED cradle, rotary latch, velcro flap |
| ③ primary form family | source-backed | volumetric wall cabinet box; molded clamshell tray; hard-shell ribbed case; soft shoulder pouch; pole-mount enclosure; sloped-roof outdoor cabinet |
| ④ surface decoration | record_only / world_knowledge_extrapolation | DEFIBRILLATOR red title, green AED heart-cross signage, "TRAINED RESPONDERS / ALARM WILL SOUND" text blocks, printed CPR instruction panels/icons — host-conformal only, not standalone variants |
| ⑤ proportion / size / travel | record_only | cabinet ~0.46×0.58×0.16 m; carry case ~0.36×0.28×0.10 m; door swing 0–1.75 rad; lid open ~103°; AED slide travel ~0.22 m — ride-along companions only (e.g. shallower depth on clear cover) |
| ⑥ material / palette / finish | record_only | white powder-coat metal, red/black EVA fabric, yellow AED, smoky window, green signage; companion colorways: safety-yellow pole box, green outdoor box, orange/charcoal hard case, black/red EMS pouch |

## Compatibility Probes
| probe | record_id | combined axes | risk tested | result |
|---|---|---|---|---|
| roll-up shutter front | rec_defibrillator_case_var_roll_shutter | ① slat-multiplicity + ② prismatic opening replacing revolute door | shutter slats stacking/clearing the front aperture in open pose without colliding roof/AED | converged |

## Blocked / Excluded
- floor_stand_column (freestanding AED tower): deferred — low incremental coverage vs pole_mount + wall_cabinet; add only if budget underfills.
- key_lock_barrel latch: excluded to avoid over-density; rotary_knob_latch already covers the "actuated latch mechanism" candidate beyond pull_handle/velcro_flap.
- Standalone ④/⑤/⑥ variants: excluded per rules (audit-only; ride along as companion variations).
