# Headwear / Racing helmet - template source map

pattern: mixed helmet shell
parents:
- rec_build-a-realistic-articulated-3d-model-of-a-raci_20260609_215058_284953_f750bd51 <- picture/Headwear/Racing helmet/001.png (red racing helmet with shell, visor, pivot studs, chin structure, vents). Covers Slot A=full_face_shell, Slot B=outer_clear_visor, Slot C=parent_venting.

Racing helmet family with shell/opening, visor/chin mechanism, and aero/vent accessory layer.
Variants isolate visor system, chin/shell opening, rear spoiler, and vent cluster while keeping visor
articulation or another real joint.

## Slot 候选覆盖

### Slot A:shell / face opening
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| full_face_shell | rec_build-a-realistic-articulated-3d-model-of-a-raci_20260609_215058_284953_f750bd51 (parent) | helmet shell, chin trim | full-face racing helmet shell | converged |
| half_open_face | rec_racing_helmet_var_half_open_face | open face shell, retained visor joint | half open-face helmet shell with larger face opening | converged |

### Slot B:visor / chin articulation
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| outer_clear_visor | parent | visor, pivot studs, visor hinge | single articulated outer visor | converged |
| dual_sun_visor | rec_racing_helmet_var_dual_sun_visor | inner sun visor hinge plus outer visor | inner drop-down tinted sun visor behind clear visor | converged |
| modular_chin_bar | rec_racing_helmet_var_modular_chin_bar | chin bar hinge | flip-up modular chin bar with visible side hinge | converged |

### Slot C:aero / ventilation layer
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| parent_venting | parent | vent visuals | inherited vent/trim layer | converged |
| rear_detail_mesh | rec_racing_helmet_var_peak_visor (record slug `peak_visor` is legacy; content is the rear detail mesh) | `rear_occipital_panel`, `rear_vent_slit_{i}`, `rear_chevron_ridge_{i}` | refined rear occipital mesh/texture panel with raised vent slits and mirrored chevrons | converged |
| aero_rear_spoiler | rec_racing_helmet_var_aero_rear_spoiler | rear spoiler visual | rear aero spoiler | converged |
| top_air_vents | rec_racing_helmet_var_top_air_vents | vent cluster loop | top vent cluster | converged |

## Multiplicity / Copy Logic
- count_param: local `vent_count` for top vent cluster; visor/chin candidates are one module each.
- N 样本已覆盖: vent_count sampled as repeated top vent cluster; side pivot count fixed at 2.
- 模板建议 N_range: side pivots fixed 2; vent cluster count should remain candidate-local until more counts are sampled.
- copied object / naming / placement / joint policy: mirrored side pivots/hinges should be left/right named; repeated vents use `vent_{i}` loop with regular spacing.

## 组合数预审
Slot A(2) x Slot B(3) x Slot C(4) = 24 >= 10 ✓.

## 排除项(未来 compatibility matrix 素材)
- No blocked cells in this batch; all planned racing helmet variants converged.
- modular_chin_bar and half_open_face both touch lower-face geometry; combine only if explicitly designed as a compatible modular/open-face hybrid.
- top_air_vents, rear_detail_mesh, and aero_rear_spoiler are accessory-layer candidates and should not change visor/chin mechanics.
