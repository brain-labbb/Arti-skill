# Structure / Elevator — template source map

pattern: mixed (named functional slots + a multiplicity axis on door leaves)

parents:
- rec_a-passenger-elevator-landing-entrance-in-a-dark-_20260608_165157_614797_f45c0537 ← picture/Structure/Elevator/001.png
  (covers: Slot A center_opening_2leaf, Slot B flush_stone_wall, Slot C bare_dark_shaft, Slot D digit_indicator+call_plate)
- rec_a-lobby-elevator-with-polished-brass-center-open_20260608_165157_970371_a14558c5 ← picture/Structure/Elevator/002.png
  (covers: Slot A center_opening_2leaf, Slot B flush_stone_wall, Slot C furnished_cab, Slot D illuminated_indicator+call_plate)

Object identity: an **elevator landing entrance** — a wall surround with a doorway opening,
center/side sliding door leaves, a floor-position indicator above the opening, a hall call
control beside it, a grooved threshold sill at the floor, and an interior reveal behind the
doors (bare shaft ↔ furnished cab). NOT a freestanding hoistway/shaft machine.

Coordinate convention inherited from parents (Z-up, meters): +X wall width / door slide,
+Y wall depth into the shaft, +Z height, floor at z=0. Doorway is cut through the wall;
door leaves sit in a shallow front pocket y∈[-0.040, 0]; shaft/cab recess behind y∈[0, ~0.35].

## Slot 候选覆盖

### Slot A: door_mechanism (the moving leaves; PRISMATIC along X)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| center_opening_2leaf | (both parents) | left_door/right_door, wall_to_left_door/wall_to_right_door (prismatic ±X) | two leaves part at center seam, mirror travel | converged |
| side_opening_telescopic_2leaf | rec_elevator_var_side_telescopic | left_door/right_door, both prismatic -X, different travel limits | both leaves slide same side, nested/telescoping | converged |
| single_slide_1leaf | rec_elevator_var_single_slide | single door leaf, one prismatic -X joint | one slab door into wall pocket (service/freight) | converged |
| center_opening_telescopic_4leaf | rec_elevator_var_center_four_leaf | door_leaf_{i} loop ×4, per-leaf prismatic, inner travels farther | 4 telescoping leaves, 2 per side (wide opening) | converged |

### Slot B: surround_facade (the fixed wall the opening sits in)
| 候选 | record_id | 关键 part/visual 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| flush_stone_wall | (both parents) | wall_surround / granite_slab|marble_surround (opening cut through flat slab) | flat wall, opening is a through-cut | converged |
| proud_architrave_portal | rec_elevator_var_proud_architrave | wall_surround + architrave band visuals proud toward -Y | raised molded frame standing proud of the wall | converged |
| recessed_alcove | rec_elevator_var_recessed_alcove | wall_surround + alcove reveal visuals (L/R/top reveals) | opening sunk into a niche, stepped reveals | converged |
| metal_framed_pylon | rec_elevator_var_metal_pylon | slim steel mullions + head beam, thin infill | narrow metal door-frame (observation/glass hoistway) | converged |

### Slot C: interior_reveal (what shows behind the open doors)
| 候选 | record_id | 关键 part/visual 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| bare_dark_shaft | rec_a-passenger...f45c0537 (P1) | wall_surround.shaft_recess (5 thin panels, dark) | shallow dark 5-panel shaft, no furniture | converged |
| furnished_cab | rec_a-lobby...a14558c5 (P2) | cab_interior (shell+back panels), handrail, ceiling, floor | cab shell + colored wall panels + handrail | converged |
| mirror_panel_cab | rec_elevator_var_mirror_cab | cab_interior with mirrored rear wall, no handrail | cab with reflective back wall, handrail removed | converged |

### Slot D: landing_fixtures (indicator + hall call controls; FIXED to wall)
| 候选 | record_id | 关键 part/visual 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| digit_indicator + call_plate | (both parents) | indicator (box+digit), call_panel (plate+up/down buttons) | 7-seg red digit + small up/down plate | converged |
| arrow_lantern + large_panel | rec_elevator_var_arrow_lantern | indicator→arrow lantern cluster, call_panel→larger panel+button | up/down arrow lanterns + bigger call panel | converged |
| lcd_strip + touch_call | rec_elevator_var_lcd_strip | indicator→LCD bar, call_panel→single touch plate | wide LCD display bar + flush touch button | converged |
| minimal_none | rec_elevator_var_minimal | indicator & call_panel removed | no indicator, no call plate (back-of-house) | converged |

## Multiplicity / Copy Logic
- count_param: door_leaf_count — coupled to Slot A mechanism (1 single_slide, 2 center/side, 4 telescopic-4leaf).
- N 样本已覆盖: {1, 2, 4} → rec_elevator_var_single_slide / parents+side_telescopic / rec_elevator_var_center_four_leaf
- 模板建议 N_range: leaf count is a discrete enum tied to the door mechanism, not a free N sweep; treat each mechanism as one topology (the 4 Slot-A candidates already give 4 distinct leaf-count/joint topologies).
- copied object: a door leaf (shared _leaf_shape helper). naming: door_leaf_{i} / left_door/right_door.
  placement: symmetric about X=0 (center-opening) or stacked toward one jamb (telescopic).
  joint policy: each leaf its own PRISMATIC joint along X; travel limit per leaf (telescopic = graded limits).

## 排除项(未来 compatibility matrix 素材)
- Slot C furnished/mirror cab × Slot B metal_framed_pylon: a glass/metal observation pylon usually
  pairs with a bare or glazed shaft, not a fully furnished opaque cab — flag as low-priority combo
  (not built; template may gate furnished_cab to stone/portal surrounds).
- Slot D minimal_none × Slot C furnished_cab is allowed but semantically odd (a furnished cab with no
  hall fixtures) — permitted, not a structural conflict.
