# Container / Box — template source map

pattern: mixed（主体为固定 named slots:closure 主机构 + interior + walls;interior 槽内含一根 multiplicity 轴 divider_count × N）

parents:
- rec_closed-kraft-cardboard-shipping-box-with-foldabl_20260606_074530_030261_0c476a9b ← picture/Container/Box/001.png（闭合 kraft 瓦楞运输箱:四顶盖 flap REVOLUTE，`for name,... in flap_defs` 干净循环发射 flap_n/s/e/w;占格 closure=four_top_flaps / interior=plain / walls=solid_corrugated。**flap 复制逻辑最干净的 parent**。）
- rec_kraft-cardboard-gift-box-with-a-separate-lift-of_20260606_074543_611960_3386fca8 ← picture/Container/Box/002.png（kraft 礼盒 + 分体望远镜式升降盖 PRISMATIC;占格 closure=liftoff_telescoping_lid / interior=plain / walls=solid。）
- rec_wooden-keepsake-box-with-a-rear-hinged-lid-and-d_20260606_074551_960283_c87d6c10 ← picture/Container/Box/003.png（胡桃木 keepsake 箱:后铰平盖 REVOLUTE + 黄铜 knuckle + 指接角 + maple accent spline;占格 closure=rear_hinged_flat_lid / interior=plain / walls=solid_wood_fingerjoint。）
- rec_open-corrugated-cardboard-box-with-four-fold-out_20260606_074600_560733_ef4e9e5a ← picture/Container/Box/004.png（开口瓦楞箱:四 fold-out 顶盖 REVOLUTE，long_flap_0/1 + short_flap_0/1 手写 4 件;占格 closure=four_fold_out_flaps（与 P1 同 flap 家族，开态）/ interior=plain / walls=solid_corrugated。）

## 组合数预审

组合数预审: Π(closure 5 × interior 4 × walls 3) × N(divider 2 样本) = 5×4×3 = 60，再含 interior 槽内 N 轴 2 样本 → ≥ 10 ✓

## Slot 候选覆盖

### Slot A:closure_mechanism（主机构槽——箱体的开合动作）
| 候选(未来 module) | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| four_top_flaps（基线） | rec_closed-kraft-cardboard-shipping-box-with-foldabl_20260606_074530_030261_0c476a9b | flap_{n,s,e,w} / box_to_flap_* (REVOLUTE) / `_flap_slab` helper + `flap_defs` 循环 | 四顶盖各绕顶边 REVOLUTE 翻折 | converged(parent) |
| liftoff_telescoping_lid | rec_kraft-cardboard-gift-box-with-a-separate-lift-of_20260606_074543_611960_3386fca8 | lid / box_to_lid (PRISMATIC +Z) / `_lid_solid` `_box_base_solid` | 分体盖沿 +Z 直升脱离（望远镜套口） | converged(parent) |
| rear_hinged_flat_lid | rec_wooden-keepsake-box-with-a-rear-hinged-lid-and-d_20260606_074551_960283_c87d6c10 | lid / base_to_lid (REVOLUTE -X) / lid_knuckle_i + base_knuckle_i 黄铜铰 | 后沿平盖绕铰线 REVOLUTE 上掀 | converged(parent) |
| sliding_drawer | rec_container_box_var_sliding_drawer | drawer / box_to_drawer (PRISMATIC，前向 -Y) + `_drawer_solid` helper | 内抽屉前抽 PRISMATIC，顶为固定面（火柴盒式） | converged |
| swing_double_door | rec_container_box_var_swing_double_door | door_left/door_right / box_to_door_i (REVOLUTE 立轴 ×2) | 前壁双开门，各立轴 REVOLUTE 外摆 | converged |
| front_drop_door | rec_container_box_var_roll_top_tambour | front_door / box_to_front_door (REVOLUTE 底边) | 单前壁面板沿底边 REVOLUTE 下翻（前装料桶式） | converged |

### Slot B:interior_structure（内部机构 / 内胆）
| 候选(未来 module) | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| plain（基线） | 全部 parent | —（无内部机构，仅 keepsake 有 lid_panel 装饰） | 空腔 | converged(parent) |
| liftout_tray | rec_container_box_var_liftout_tray | inner_tray / box_to_tray (PRISMATIC +Z) + 内壁 ledge visual | 浅内托盘落于内 ledge，垂直 PRISMATIC 提出 | converged |
| stacking_lip | rec_container_box_var_stacking_lip | stack_lip + foot_ring 作 box parent visual（无 joint）；保留 lift-off lid 为唯一活动关节 | 顶内缘凸唇 + 底凹脚环，可叠箱注册 | converged |
| compartment_dividers_N（multiplicity） | rec_container_box_var_n2_dividers / rec_container_box_var_n4_dividers | divider_i / box_to_divider_i (PRISMATIC ×N) + `_divider_solid` helper | N 块插槽隔板，循环发射，统一 PRISMATIC 提出 | converged |

### Slot C:wall_surface（壁面 / 表面结构）
| 候选(未来 module) | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| solid_corrugated / solid_wood（基线） | P1/P4（瓦楞）, P3（指接木）, P2（实壁） | `_shell_solid` / `_box_shell` / `_box_base_solid` | 实心箱壁（瓦楞纸 / 木 / 指接角） | converged(parent) |
| slatted_walls | rec_container_box_var_slatted_walls | slat_i（四壁横板条）/ `_slat_solid` helper，for 循环发射 | 开口板条箱壁（横木板 + 间隙） | converged |
| perforated_walls | rec_container_box_var_perforated_walls | shell（CadQuery boolean 钻孔阵）/ `_perforated_shell` helper | 规则圆孔通风壁（布尔挖孔） | converged |

## Multiplicity / Copy Logic
- count_param: `divider_count`（interior 槽内的隔板复制数）。closure / walls 主结构为固定 named slots，无 N 复制；walls=slatted 的 slat_i 是壁面内部复制细节，复制数随尺寸自适应，不作为独立 multiplicity 采样轴（属 controlled local parameterization）。
- N 样本已覆盖: {2, 4} → rec_container_box_var_n2_dividers / rec_container_box_var_n4_dividers
- 模板建议 N_range: divider_count ∈ [1, 8]（采样域可远大于 {2,4} 样本;箱体宽度上限处自然封顶）
- copied object / naming / placement / joint policy: copied = 单块竖隔板（`_divider_solid` 共享 helper）;naming = `divider_i`；placement = 沿 +X 等距插槽，落在底面 ledge；joint policy = 每块独立 `box_to_divider_i` PRISMATIC 垂直提出（统一上下限）。

## 格子覆盖与批次规模
- closure 3 空格(sliding_drawer / swing_double_door / front_drop_door) + interior 3 空格(liftout_tray / stacking_lip / dividers)其中 dividers 含 N={2,4} 两样本 + walls 2 空格(slatted / perforated)。
- 待填 cells = closure 3 + interior(liftout 1 + stacking 1 + divider 主格 1) + walls 2 = 共 8 个结构格，divider 格再加 1 个 N 样本 → **9 个计划变体**。落点干净、无连续尺寸虚胖，主机构 slot(closure)满配 6 候选。

## 排除项(未来 compatibility matrix 素材)
- `roll_top_tambour`（真正的卷帘/百叶卷盖）:真实物体需 N 段链式铰接板条沿弯曲导轨滑动，SDK 无曲线 rail prismatic，造价高且易出类目/不收敛 → 降级为单 `front_drop_door`（单铰下翻门）作该格的可收敛真实形态，记此处。卷帘形态留作模板侧未来扩展候选。
- `magnetic_clasp` / `friction_press_fit` 等无关节闭合:0 活动关节，违反 §3 拒收，不立格。
- 跨轴组合(如 圆形箱体 × 折叠把手)不单独造:组合由模板采样器免费产出，非样本池义务。
- handle/grip 轴本批未单列:四个 parent 均无独立提手机构（仅 keepsake 黄铜铰为装饰），真实 Box 小类提手词汇稀薄，暂折入未来 hardware 槽，不强造空候选。
