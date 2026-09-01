# Chair / Folding chair — template source map

pattern: parallel_children(固定 named slots:fold 主机构 + seat_panel + frame + accessory;无 multiplicity)

parents:
- rec_metal-folding-chair-a-gray-powder-coated-tubular_20260606_120809_950609_cc9b62cf ← picture/Chair/Folding chair/001.png（灰色粉末喷涂钢管折叠椅;helper `_leg_tube` / `_cross_tube` / `_pad_mesh` / `_floor_footprint`;3 REVOLUTE 折叠铰;基线 = fold:scissor_cross × seat:padded × frame:tubular_steel × accessory:plain;**全批 fork 基线**）

20 个变体来自 `chair_folding_chair_gpt55_20260611` 批次(openai / gpt-5.5 / med),全部 rc=0、均有 URDF、非 fixed joint 2–5 个;完整 record id 见 `picture_expansion/generated_assets.jsonl`(此处用 variant_index 标注)。

**关键结构信号(给模板作者):** 全 20 个变体 **共享** `_leg_tube` / `_cross_tube` / `_floor_footprint` 三个 helper —— 这就是模板的固定骨架(管腿 + 折叠交叉 + 落地足印);**座/背板**则按变体各换一个 `_*_mesh` helper(`_sling_mesh` / `_canvas_mesh` / `_perforated_plastic_panel` / `_plastic_panel_mesh` / `_cane_panel_mesh` / `_fabric_bucket_mesh` …)。seat_panel 槽是最干净的可换模块,直接对应这些 helper。

## 组合数预审


## Slot 候选覆盖

### Slot A:fold_mechanism（主机构槽——椅子的折叠动作)
| 候选(未来 module) | variant | 关键 joint / 结构 | 状态 |
|---|---|---|---|
| scissor_cross_brace（基线） | parent / v01 | `_cross_tube` 交叉撑,3 REVOLUTE | parent / converged |
| rear_legs_fold_under | v02 | 后腿向架下折叠,5 REVOLUTE | converged |
| pivoting_rear_support_legs | v03 / v17 | 后支撑腿绕销 REVOLUTE 翻转 | converged |
| triangular_folding_frame | v07 | 三角折叠腿架(stool 高) | converged |
| reinforced_double_cross_brace | v10 | 双交叉撑加固 + 橡胶脚帽 | converged |
| collapsing_side_hinge_links | v20 | 侧铰链折叠链 + 四细腿,5 REVOLUTE | converged |

### Slot B:seat_back_panel（座/背板;每候选一个 `_*_mesh` helper)
| 候选 | variant | helper | 状态 |
|---|---|---|---|
| padded_pad（基线） | parent | `_pad_mesh` | parent |
| fabric_sling | v01 | `_sling_mesh` | converged |
| molded_plastic | v02 / v14 | `_plastic_panel_mesh` | converged |
| wood_slat | v03 | `_slat_y_positions` | converged |
| padded_vinyl_cushion | v05 | `_pad_mesh`(厚垫) | converged |
| canvas | v06 / v12 | `_canvas_mesh` | converged |
| perforated_plastic | v09 | `_perforated_plastic_panel` | converged |
| polycarbonate_translucent | v11 | `_panel_mesh` | converged |
| woven_cane | v17 | `_cane_panel_mesh` + `_arched_back_rail` | converged |
| fabric_bucket | v20 | `_fabric_bucket_mesh` / `_fabric_back_mesh` | converged |

### Slot C:frame_style（管材/边框样式)
| 候选 | variant | 结构特征 | 状态 |
|---|---|---|---|
| tubular_steel（基线） | parent | 圆钢管 `_leg_tube` | parent |
| aluminum_tube | v01 | 轻量铝管 | converged |
| chrome_tube_hinge_plates | v05 | 镀铬管 + 可见铰板 | converged |
| thin_wire_frame | v11 | 细线材框 | converged |
| wood_side_frame | v12 | 深色木折叠边框 + 销 | converged |
| rectangular_director_side_frame | v06 | director 式矩形侧框 | converged |
| flat_bar_angular | v18 | 扁条角形侧框(`_flat_bar_between` / `_flat_bar_path` / `_cross_flat_bar`)+ 圆 pivot hub | converged |

### Slot D:accessory（附加机构;各自带一个独立活动 joint)
| 候选 | variant | 关键 joint / 结构 | 状态 |
|---|---|---|---|
| plain（基线） | parent | 无附件 | parent |
| pivoting_tablet_arm | v08 | 写字板臂 REVOLUTE 旋转(`_tablet_steel_mesh`),5 REVOLUTE | converged |
| reclining_backrest_2nd_hinge | v13 | 靠背第二锁定铰位,5 REVOLUTE | converged |
| carry_handle_cutout | v16 | 靠背提手挖孔(`_handle_backrest_mesh`)+ 座铰,4 REVOLUTE | converged |
| flip_down_footrest | v19 | 前腿联动翻下脚踏杆(`_footrest_loop`),5 REVOLUTE | converged |

## Multiplicity / Copy Logic

- count_param: **无**,核心结构为固定 named slots(单椅)。按 SPEC §8 写"无复制逻辑"。

## 连续尺寸参数(非候选;模板侧缩放,勿当 slot 候选)

stool 高(v07) / child 小(v14) / heavy-duty 宽(v15) / beach 低斜(v04) / tall director 高(v06) / narrow 窄(v16) —— 这些是 **比例/姿态** 差异,写 spec 时作连续尺寸参数(座高/座宽/靠背角),不要当 Slot 候选(FORK_VARIANTS §2 line 65)。

## 格子覆盖(全部 converged;parent 基线计入)

fold 5 空格(v02/v03/v07/v10/v20) + seat_panel 9(v01/v02/v03/v05/v06/v09/v11/v17/v20) + frame 6(v01/v05/v06/v11/v12/v18) + accessory 4(v08/v13/v16/v19)。20 个变体已铺满规划格子,**Folding chair 小类样本池就绪**。

## 排除项(未来 compatibility matrix 素材)

- (暂无;全 20 收敛。)多数变体带次级轴(如 v06 = director 框 + canvas 座 + 高比例),写 spec 时按 headline 轴归 module,其余作 seat/frame 交叉或连续比例参数,勿当独立候选。
