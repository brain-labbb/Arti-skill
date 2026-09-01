<!--
subcategory_contract:
  category: Emergency Equipment
  subcategory: Stretcher
  core_identity: a load-bearing patient-carrying surface used to move an injured/immobile person
  must_keep:
    - a rigid or tensioned full-body patient support surface
    - a means to lift/carry/roll it (wheeled undercarriage, carry poles/handles, or basket rim)
    - at least one real non-fixed joint unless explicitly static (fold, hinge, telescoping, or caster spin)
  must_not_become:
    - hospital examination table / adjustable exam couch
    - wheelchair or stair chair
    - hospital electric bed frame
    - hand cart / gurney-shaped furniture
  image_evidence:
    - 001.png: wheeled ambulance cot, orange padded mattress, X-frame scissor undercarriage, 4 swivel casters, folding tubular side rails, backrest raised, red foot release lever, foot push hoop
    - 002.png: wheeled ambulance cot, black mattress on yellow frame, folding X drop-legs, 4 casters, side rails, raised backrest, tall IV pole, red brake tabs, foot push handle
  parent_evidence:
    - o1 (39031c58): lower_carriage root, litter_frame on PRISMATIC height_slide, backrest on REVOLUTE backrest_hinge, side_rail_0/1 REVOLUTE, scissor_arm_0/1 REVOLUTE scissor_hinge, caster_0-3 CONTINUOUS caster_spin; 4 casters; _tube/_box helpers; caster_locations list
    - o2 (c7b79ba9): deck_frame root, backrest REVOLUTE deck_to_backrest, side_rail_0/1 REVOLUTE, head_leg/foot_leg REVOLUTE folding legs, caster_0-3 CONTINUOUS; 4 casters; IV pole; _add_leg_geometry/_add_side_rail_geometry/_make_caster_meshes helpers; caster_specs list
-->

# Emergency Equipment / Stretcher — template source map
pattern: mixed (linear_chain lift + parallel_children rails/legs/casters + multiplicity casters)
parents:
- rec_emergency_equipment__stretcher_39031c58fb4f4885963ff67e361ff07c (o1) — picture/Emergency Equipment/Stretcher/001.png — wheeled scissor-lift ambulance cot, orange pads, 4 casters
- rec_emergency_equipment__stretcher_c7b79ba913ea474991d52db15c0f9e7e (o2) — picture/Emergency Equipment/Stretcher/002.png — wheeled folding-leg ambulance cot, black mattress on yellow frame, IV pole, 4 casters
canonical_baselines: none
underfilled_reason: none (12 candidate anchors + 1 probe within the normal 12–18 band; coverage-first, not padded)

## Slot Candidates
| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints | status |
|---|---|---|---|---|---|---|
| support_or_base (undercarriage) | wheeled scissor-lift X-frame cot | ③/② | origin_anchor | o1 001.png | lower_carriage, scissor_arm_0/1, scissor_hinge_0/1, height_slide | converged (origin) |
| support_or_base | wheeled folding drop-leg cot | ③/② | origin_anchor | o2 002.png | deck_frame, head_leg, foot_leg, deck_to_head_leg/foot_leg | converged (origin) |
| support_or_base | four telescoping tube-in-tube legs | ② | forked_anchor | rec_stretcher_var_telescoping_legs (o1) | new telescoping leg parts + PRISMATIC, replaces scissor | converged |
| support_or_base | non-wheeled carry poles + folding spreaders | ③ | forked_anchor | rec_stretcher_var_pole_canvas (o2) | carry poles, spreader REVOLUTE (reuse deck_to_head_leg/foot_leg) | converged |
| support_or_base | rigid basket / Stokes shell (clamshell fold) | ③ | forked_anchor | rec_stretcher_var_basket_litter (o2) | basket shell halves, center fold REVOLUTE | converged |
| support_or_base | flat rigid spine backboard (no legs) | ③ | forked_anchor | rec_stretcher_var_spine_board (o2) | flat board deck_frame, hinged head blocks | converged |
| collapse_lift_mechanism | scissor X-frame revolute + prismatic height | ② | origin_anchor | o1 | scissor_hinge_0/1, height_slide | converged (origin) |
| collapse_lift_mechanism | folding drop-leg revolute | ② | origin_anchor | o2 | deck_to_head_leg, deck_to_foot_leg | converged (origin) |
| collapse_lift_mechanism | telescoping-leg prismatic | ② | forked_anchor | rec_stretcher_var_telescoping_legs (o1) | leg PRISMATIC | converged |
| collapse_lift_mechanism | whole-deck Trendelenburg tilt revolute | ② | forked_anchor | rec_stretcher_var_trendelenburg_tilt (o1) | tilt_frame, deck_tilt REVOLUTE | converged |
| collapse_lift_mechanism | scoop split blades + telescoping length prismatic | ② | forked_anchor | rec_stretcher_var_scoop_split (o2) | deck_half_left/right, separation + length PRISMATIC | converged |
| patient_surface | segmented padded foam mattress | ③ | origin_anchor | o1/o2 | foot_pad, seat_pad, back_pad / main_mattress | converged (origin) |
| patient_surface | tensioned canvas/vinyl bed sheet | ③ | forked_anchor | rec_stretcher_var_canvas_deck (o1) | canvas panels between upper_side_tube | converged |
| patient_surface | perforated basket floor pan | ③ | forked_anchor | rec_stretcher_var_basket_litter (o2) | mesh floor pan | converged |
| patient_surface | hard flat HDPE board | ③ | forked_anchor | rec_stretcher_var_spine_board (o2) | rigid board deck_frame | converged |
| restraint_or_side_rails | folding tubular side rails | ① | origin_anchor | o1/o2 | side_rail_0/1, side_rail_hinge_* / deck_to_side_rail_* | converged (origin) |
| restraint_or_side_rails | webbing restraint strap harness (no rails) | ① | forked_anchor | rec_stretcher_var_strap_harness (o1) | cross straps + shoulder harness visuals | converged |
| restraint_or_side_rails | hinged head-immobilizer blocks | ① | forked_anchor | rec_stretcher_var_spine_board (o2) | head blocks REVOLUTE (reuse deck_to_side_rail_0/1) | converged |
| articulation_topology | single backrest hinge | ① | origin_anchor | o1/o2 | backrest_hinge / deck_to_backrest | converged (origin) |
| articulation_topology | backrest + foot/knee-gatch hinge | ① | forked_anchor | rec_stretcher_var_foot_gatch (o1) | foot_section, foot_gatch_hinge REVOLUTE | converged |
| handle_or_grip | foot push hoop / handle | ② | origin_anchor | o1/o2 | foot_push_crossbar / foot_push_handle | converged (origin) |
| handle_or_grip | carry poles / basket rope handles | ③ | forked_anchor | rec_stretcher_var_pole_canvas, rec_stretcher_var_basket_litter | pole ends, rope grabs | converged |
| handle_or_grip | fold-out telescoping carry handles | ② | forked_anchor | rec_stretcher_var_telescoping_handles (o2) | head/foot handle PRISMATIC | converged |
| multiplicity_casters | 4 casters | N | origin_anchor | o1/o2 | caster_0-3, caster_spin_* / *_leg_to_caster_* | converged (origin) |
| multiplicity_casters | 6 casters | N | forked_anchor | rec_stretcher_var_casters_six (o1) | caster_0-5, caster_spin_0-5 | converged |
| multiplicity_casters | 2 casters (foot end only) | N | forked_anchor | rec_stretcher_var_casters_two (o2) | caster_0-1, foot_leg_to_caster_0/1 | converged |

## Multiplicity / Copy Logic
- count_param: length of caster_locations (o1) / caster_specs (o2)
- N samples: 2 (rec_stretcher_var_casters_two), 4 (both origins), 6 (rec_stretcher_var_casters_six)
- suggested N_range: 2–6 swivel casters (2, 4, 6 realistic; odd counts atypical)
- copied object / naming / placement / joint policy:
  - copied_object: caster wheel + fork/yoke subassembly
  - naming: caster_i / spin joint caster_spin_i or *_leg_to_caster_i, i in 0..N-1
  - placement: two longitudinal rows on a shared track; mid pair added symmetrically for N=6; single foot pair for N=2
  - joint_policy: one CONTINUOUS spin joint per wheel about the ±Y/local axle; generated in a loop, never hand-written

## Six-Axis Diversity Record
| axis | treatment | values / range / reason |
|---|---|---|
| ① skeleton / structural topology | source-backed | strap-only vs folding rails; single backrest vs backrest+foot-gatch; wheeled cot vs pole vs basket vs board undercarriage topologies |
| ② joint / mechanism type | source-backed | scissor REVOLUTE+PRISMATIC (o1), folding-leg REVOLUTE (o2), telescoping-leg PRISMATIC, Trendelenburg tilt REVOLUTE, scoop split+length PRISMATIC, telescoping handle PRISMATIC, caster CONTINUOUS |
| ③ primary form family | source-backed | wheeled cot / folding pole-and-canvas / basket-Stokes litter / rigid spine board; surface: padded mattress / canvas / perforated pan / hard board |
| ④ surface decoration | record_only + world_knowledge_extrapolation | restraint strap layouts, hand-hole cutouts, mattress seams, brake/release decals — host-conformal only, not standalone variants |
| ⑤ proportion / size / travel | record_only | deck length ~1.9–2.0 m; height travel ~0.24 m (o1 height_slide); rail fold 0–1.55 rad; backrest ±0.55 rad; bariatric-wide and pediatric-short are proportion-only, not forked |
| ⑥ material / palette / finish | record_only + companion | safety-orange vinyl (o1), safety-yellow frame + black mattress (o2); companions: EMS navy-blue, olive/OD canvas, international-orange/red rescue, hi-vis HDPE — ride-along only |

## Compatibility Probes
| probe | record_id | combined axes | risk tested | result |
|---|---|---|---|---|
| basket shell on wheeled folding-leg carriage | rec_stretcher_var_wheeled_basket_probe (o2) | ③ basket body + wheeled folding-leg undercarriage | basket floor pan vs deck side tubes clearance; basket rim vs folded-leg swing envelope | converged |

## Blocked / Excluded
- soft roll-up flexible litter (SKED-style): excluded — would be effectively static_only with no faithful non-fixed joint; not worth a fork here.
- powered hydraulic single-column bariatric lift: excluded as redundant with telescoping-leg and scissor lift mechanisms (would be padding).
- wheelchair / stair-chair conversion: blocked — drifts to neighbor category (seated patient transport), violates must_not_become.
- exam-table / hospital-bed multi-motor deck: blocked — neighbor category drift; foot-gatch fork already covers multi-section articulation within stretcher identity.
