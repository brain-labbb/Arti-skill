# Playground / playground chair swing carousel — template source map

slug: `playground_chair_swing_carousel`
pattern: radial (one central CONTINUOUS spin column/rotor + N loop-emitted REVOLUTE swinging seats on radial arms, 90/60/180-deg even angular spacing)

parents (2 — a powered chair-swing carousel: a center column spins; arms carry seats that swing out on revolute pivots; all loop-clean for the seat ring):
- rec_model-an-old-four-seat-playground-chair-swing-ca_20260610_085340_162128_e48c2551 ← picture/Playground/playground chair swing carousel — "old" four-seat; round splayed-leg Cylinder base; 8 splayed spline-tube X-lattice arms (`pivot_bar_i` + `arm_tube_i_k`); slatted bucket seats + wraparound bent-tube guard rail; rust_red/cream; uses mesh_from_cadquery/tube_from_spline_points. fills SlotA `splayed_leg_base`, SlotB `spline_tube_lattice`, SlotC `slatted_bucket_rail`. converged (parent)
- rec_model-a-weathered-four-seat-playground-chair-swi_20260610_085325_366519_82f97e28 ← picture/Playground/playground chair swing carousel — "weathered" four-seat; square Box base_plate + 4 anchor_bolts; white column + 2 rust bands; 4 straight radial `arm_i` (blue/yellow) + 8 `brace_n` X-truss + clevis tips; flat platform seat + simple backrest; only Box/Cylinder. fills SlotA `square_slab_base`, SlotB `straight_radial_arm`, SlotC `flat_platform_seat`. converged (parent)

## Slot 候选覆盖

### Slot A:base_form(地面底座 — 固定,承 CONTINUOUS spin column)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| splayed_leg_base | rec_model-an-old-four-seat-playground-chair-swing-ca_20260610_085340_162128_e48c2551 | base_plate / splayed legs | 圆盘 + 外撇腿底座,自然落地 | converged (parent) |
| square_slab_base | rec_model-a-weathered-four-seat-playground-chair-swi_20260610_085325_366519_82f97e28 | base_plate / anchor_bolt_i | 方板底座 + 四地脚螺栓 | converged (parent) |
| pedestal_column | rec_pcsc_var_pedestal | base_pedestal / foot_disc | 单中央粗柱外张成大圆盘底脚 | workbench (pending sync) — EMPTY cell |
| tripod_stand | rec_pcsc_var_tripod | tripod_leg_i(loop3) / hub | 三外撇腿 + 中央毂底座 | workbench (pending sync) — EMPTY cell |

### Slot B:arm_structure(承座臂 — 连 rotor → 每座 REVOLUTE swing pivot)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| spline_tube_lattice | rec_model-an-old-four-seat-playground-chair-swing-ca_20260610_085340_162128_e48c2551 | pivot_bar_i / arm_tube_i_k | 8 撇出 spline-tube X 桁臂,曲管 mesh | converged (parent) |
| straight_radial_arm | rec_model-a-weathered-four-seat-playground-chair-swi_20260610_085325_366519_82f97e28 | arm_i / brace_n / clevis | 4 直辐射臂 + 8 X 撑 + clevis 端 | converged (parent) |
| cantilever_arm | rec_pcsc_var_cantilever | arm_i(loop) | 单根锥形悬臂/座,无桁架 | workbench (pending sync) — EMPTY cell |
| overhead_chain_hung | rec_pcsc_var_chainhung | top_hub_ring / hanger_i(loop) | 座由顶毂吊链垂挂(钟摆),仍每座 revolute | workbench (pending sync) — EMPTY cell |

### Slot C:seat_type(座面 — fixed 到对应 swing pivot 件,整座随 revolute 摆)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| slatted_bucket_rail | rec_model-an-old-four-seat-playground-chair-swing-ca_20260610_085340_162128_e48c2551 | seat_i / guard_rail_i | 板条桶座 + 环抱弯管护栏 | converged (parent) |
| flat_platform_seat | rec_model-a-weathered-four-seat-playground-chair-swi_20260610_085325_366519_82f97e28 | seat_i / backrest_i | 平板座 + 简单靠背 | converged (parent) |
| deep_bucket_seat | rec_pcsc_var_bucket | seat_i(loft/cadquery)/ lap_bar_i | 深包围曲壁桶座 + 前安全杆 | workbench (pending sync) — EMPTY cell |

## Multiplicity / Copy Logic
- count_param: `seat_count`(座/臂 N) — parents 覆盖 N=4(both);variant 扩 N ∈ {2, 6}
- N 样本已覆盖:{2(var_n2), 4(parents), 6(var_n6)};模板建议 N_range [2, 8] even angular spacing
- copied object / naming / placement / joint policy:
  - copied object:臂 `arm_i` / 座 `seat_i` / 护栏 / 撑杆 `brace_n` / 锚栓 `anchor_bolt_i`
  - naming:`for i in range(seat_count)` + `f"arm_{i}"`/`f"seat_{i}"`;角度 `2*pi*i/seat_count`
  - placement:绕 rotor 等角分布,半径固定;座挂在臂端 swing pivot
  - joint policy:1 个 column→rotor CONTINUOUS spin(轴 Z);每座 1 REVOLUTE swing(轴 ~ (0,-1,0) 切向,±~30deg);座 fixed 到 swing pivot 件

## 排除项(未来 compatibility matrix 素材)
- seat_count N 不专门多 fork:parents N=4 + var_n2/var_n6 已铺三档 → 计数轴交给模板采样器。
- 跨轴组合(如 pedestal_column × spline_tube_lattice × deep_bucket × N6)交给模板采样器,不专造组合变体。
- color / material / 纯比例不是结构轴。

---
7 个 variant 填格:
- var_pedestal → SlotA `pedestal_column`(EMPTY)
- var_tripod → SlotA `tripod_stand`(EMPTY)
- var_cantilever → SlotB `cantilever_arm`(EMPTY)
- var_chainhung → SlotB `overhead_chain_hung`(EMPTY)
- var_bucket → SlotC `deep_bucket_seat`(EMPTY)
- var_n2 → seat_count N=2
- var_n6 → seat_count N=6
