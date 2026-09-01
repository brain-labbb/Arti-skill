# Handtools / dial caliper — template source map

pattern: parallel_children
parents:
- rec_build-a-realistic-articulated-3d-model-of-a-dial_20260609_154006_366045_b08cf719 ← picture/Handtools/dial caliper/001.png (analog dial readout, dual inside/outside jaws, depth rod + knurled thumb roller, flat graduated beam)

Single parent — variants populate every slot. Shared kinematics: a `beam` (root) carries a
`slider` carriage on a PRISMATIC joint (`beam_to_slider`, axis +X) — the defining measuring
mechanism, present in EVERY candidate. The analog readout adds a CONTINUOUS `slider_to_needle`
joint (mimic-coupled to slider travel); digital/vernier readouts drop it, leaving the
prismatic slide as the sole joint. Four independent structural slots: readout, jaw config,
fine-adjust accessory, beam profile.

## Slot 候选覆盖

### Slot A:measurement readout
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| analog_dial | rec_..._dial_..._b08cf719 (parent) | `dial_bezel`, `dial_face`, `dial_ticks` (`for i in range(n)`), `needle`, `slider_to_needle` (CONTINUOUS) | round dial + rotating needle | converged |
| digital_lcd | rec_caliper_var_digital | flat LCD panel on slider; dial/needle removed (prismatic only) | digital caliper readout | converged |
| vernier_scale | rec_caliper_var_vernier (primary) | beam main scale + slider vernier window via tick loop; dial/needle removed | vernier caliper readout | converged |

### Slot B:jaw configuration
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| dual_inside_outside | rec_..._dial_..._b08cf719 (parent) | `fixed_jaw` + slider jaws (lower outside + upper inside) | universal inside/outside jaws | converged |
| depth_flat | rec_caliper_var_depthjaw (primary) | lower jaws → flat registration feet, depth_rod emphasized | depth/step measuring config | converged |
| internal_only | rec_caliper_var_internaljaw | only upper inside knife-edge tips, lower blades removed | internal-bore jaws, slim head | converged |
| step_jaw | rec_caliper_var_stepjaw (primary) | lower jaw compound: external blade + stepped shoulder ledge | step/shoulder measuring jaws | converged |

### Slot C:fine-adjust / accessory
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| rod_and_roller | rec_..._dial_..._b08cf719 (parent) | `depth_rod` + `thumb_roller` (`for i in range(18)` flutes) | depth rod + knurled fine-adjust wheel | converged |
| lock_screw | rec_caliper_var_lockscrew | thumb_roller → knurled/hex lock screw (optional small revolute) | lock screw clamps slider to beam | converged |
| simplified_no_rod | rec_caliper_var_norod | depth_rod + thumb_roller removed, molded thumb lip | simplified, no depth rod | converged |

### Slot D:beam profile
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| flat_bar | rec_..._dial_..._b08cf719 (parent) | `beam_body` flat rectangular bar, rounded edges | flat graduated bar | converged |
| channel_beam | rec_dial_caliper_var_channel_beam | channel / ribbed beam cross-section | channel-profile beam | converged |

## Multiplicity / Copy Logic
- count_param: 部件内复制(非小类轴): `dial_ticks` 用 `for i in range(n)`(parent n=50,major every 5th),
  `thumb_roller` flutes `for i in range(18)`。tick/flute 数是 module 内部参数,不作独立 slot 轴。
  vernier_scale 候选的主尺/游标刻度同样循环发射。
- 无小类级 multiplicity 轴(核心是固定 named slots)。

## 组合数预审
Slot A(3) × Slot B(4) × Slot C(3) × Slot D(2) = 72 ≥ 10 ✓。每个 slot ≥2 候选。pattern = parallel_children。

## 排除项 / 重复格子(未来 compatibility matrix 素材)
- 重复格子(prior 批次同格备样,workbench 保留但不作 primary、不同步):
  - vernier_scale: rec_dial_caliper_var_vernier_scale — dup of rec_caliper_var_vernier
  - depth_flat: rec_dial_caliper_var_depth_rod_jaws — dup of rec_caliper_var_depthjaw
  - step_jaw: rec_dial_caliper_var_step_jaws — dup of rec_caliper_var_stepjaw
- digital_lcd / vernier_scale 去掉 needle 后仅剩 prismatic 滑动关节(仍满足 ≥1 non-fixed joint)。
- 纯尺寸(beam 长度/量程)是模板连续参数,不入 slot。
