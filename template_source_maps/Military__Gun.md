# Military / Gun — template source map

pattern: mixed

parents:
- rec_model-a-classic-double-action-revolver-colt-pyth_20260610_081456_135585_9e7d2f05 ← picture/Military/Gun/001.png (revolver spine: fills Slot A=revolver_swingout, Slot B=mid barrel, Slot C=fixed sight, Slot D=square grip; carries the chamber multiplicity)
- rec_model-a-modern-striker-fired-semi-automatic-pist_20260610_081229_238472_7155f244 ← picture/Military/Gun/002.png (semi-auto spine: fills Slot A=semi_auto_slide, Slot B=mid barrel, Slot C=fixed sight, Slot D=straight grip)

## Slot 候选覆盖

### Slot A:action
| 候选(module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| revolver_swingout | rec_model-a-classic-double-action-revolver-colt-pyth_20260610_081456_135585_9e7d2f05 | parts frame / crane / cylinder / ejector_rod / trigger / hammer / grip; joints crane_swing(revolute, axis -X) / cylinder_spin(continuous, axis +X) / ejector_push(prismatic) / trigger_pull(revolute) / hammer_cock(revolute); grip_mount(fixed) | 5 nonfixed DOF; crane-mounted spinning cylinder swings out left, ejector star rides crane arbor bore; frame carries trigger_pin/hammer_pin pivots | converged (parent) |
| semi_auto_slide | rec_model-a-modern-striker-fired-semi-automatic-pist_20260610_081229_238472_7155f244 | parts frame / slide / trigger / takedown_lever / magazine; joints frame_to_slide(prismatic, axis -X) / frame_to_trigger(revolute) / frame_to_takedown_lever(revolute) / frame_to_magazine(prismatic, raked); helpers _build_frame_solid / _build_slide_solid | 4 nonfixed DOF; reciprocating slide on frame rails over hollow bore, raked magwell drop, open-loop trigger guard | converged (parent) |

### Slot B:barrel-length
| 候选(module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| revolver_mid | rec_model-a-classic-double-action-revolver-colt-pyth_20260610_081456_135585_9e7d2f05 | _barrel_solid (barrel_assembly); MUZZLE_X=0.126, LUG_X1=0.122 | 6-inch full-length vented-rib barrel + full underlug shrouding the ejector rod | converged (parent) |
| revolver_snub | rec_handgun_var_revsnub | _barrel_solid (barrel_assembly); MUZZLE_X=0.025, LUG_X1=0.020 | 2-inch snub-nose: short barrel + shortened underlug, name=snubnose_double_action_revolver | converged (workbench, rating pending sync) |
| pistol_mid | rec_model-a-modern-striker-fired-semi-automatic-pist_20260610_081229_238472_7155f244 | _build_slide_solid (slide_body) / barrel_block; SLIDE_X1=0.105 (length 0.21 m) | standard slide length, barrel_block visible through ejection port | converged (parent) |
| pistol_long | rec_handgun_var_pistlong | _build_slide_solid (slide_body) / barrel_block; SLIDE_X1=0.130 (length 0.235 m) | longer 5-inch slide, bore exit pushed forward, name=striker_fired_pistol_5in | converged (workbench, rating pending sync) |

### Slot C:sights
| 候选(module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| revolver_fixed | rec_model-a-classic-double-action-revolver-colt-pyth_20260610_081456_135585_9e7d2f05 | _rear_sight_solid (rear_sight) | low fixed notch sight milled into the top strap | converged (parent) |
| pistol_fixed | rec_model-a-modern-striker-fired-semi-automatic-pist_20260610_081229_238472_7155f244 | rear_sight / front_sight (Box) | low fixed iron sights on the slide deck | converged (parent) |
| revolver_adjustable | rec_handgun_var_revadjsight | _rear_sight_solid (rear_sight): base / bridge / housing union + windage_stem / windage_head / elev_screw | tall fully-adjustable assembly (blade tip > z=0.137) with windage knob + elevation screw | converged (workbench, rating pending sync) |
| pistol_optic_cut | rec_handgun_var_pistoptic | slide elems optic_sight_block / optic_lens_window / optic_screw_{i}; _build_slide_solid milled pocket cut | optic-ready milled top deck with low-profile red-dot housing + lens window + mount screws | converged (workbench, rating pending sync) |

### Slot D:grip
| 候选(module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| revolver_square | rec_model-a-classic-double-action-revolver-colt-pyth_20260610_081456_135585_9e7d2f05 | _grip_solid (grip_body) + left/right_grip_panel + left/right_grip_screw; grip_mount(fixed) | square target-style butt with flat heel, walnut panels | converged (parent) |
| revolver_roundbutt | rec_handgun_var_revroundbutt | _grip_solid (grip_body, spline outline); grip_mount(fixed) | rounded smooth-heel butt (spline, no flat butt surface), smaller/curved | converged (workbench, rating pending sync) |
| pistol_straight | rec_model-a-modern-striker-fired-semi-automatic-pist_20260610_081229_238472_7155f244 | _build_frame_solid grip outline (BUTT_F/BUTT_G) + left/right_grip_panel; mag drop 0.10 | full-size raked polymer grip, standard magwell depth | converged (parent) |
| pistol_compact | rec_handgun_var_pistcompact | _build_frame_solid grip outline (BUTT_F=(-0.046,0.0)/BUTT_G); MAG_TRAVEL=0.07 | grip shortened ~25 mm along rake, shorter flush magazine (capacity folded into grip length) | converged (workbench, rating pending sync) |

## Multiplicity / Copy Logic
- count_param: CHAMBER_COUNT (revolver cylinder chambers; with derived CHAMBER_ANGLE_STEP = 360.0 / CHAMBER_COUNT)
- N 样本已覆盖: {6} → rec_model-a-classic-double-action-revolver-colt-pyth_20260610_081456_135585_9e7d2f05 (parent, originally hard-coded 6) / {5} → rec_handgun_var_rev5shot / {8} → rec_handgun_var_rev8shot
- 模板建议 N_range: [5, 8]
- copied object / naming / placement / joint policy: each copy = one chamber bore (_chamber_position) + flute (_flute_position) + dark chamber_liner_{k}; equiangular about the cylinder axis (top chamber on the bore axis at q=0); rigid inside the single cylinder part (no per-chamber joint — they ride the one cylinder_spin continuous joint). Refactor note: the parent's two hard-coded `for k in range(6)` loops (chamber/flute cuts + liners) plus the polygon(6) ejector star were rewritten to `range(CHAMBER_COUNT)` / `polygon(CHAMBER_COUNT, ...)` with the _chamber_position / _flute_position helpers so the chamber-count variants are pure N-parameter changes.

## 排除项(未来 compatibility matrix 素材)
- cross-family hybrids (revolver action × pistol slide / mixed grip-spine parts) — excluded: the two spines are disjoint part/joint sets; no candidate crosses revolver↔semi_auto and none is expected to converge (出类目 / would not be a single coherent gun).
- magazine capacity as a looped multiplicity — excluded: not modeled as a per-round copy loop; capacity is folded into the pistol_compact grip/magazine-length axis (Slot D) rather than a count_param.
- color / scale as axes — excluded: material palette and overall size are fixed per spine, not template slots.
