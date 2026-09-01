# Fence / Cascade fences (MORE THAN 1) — template source map

pattern: mixed(panel_style 槽 + 链式 multiplicity)
parents:
- rec_build-a-realistic-articulated-3d-model-of-a-casc_20260609_215015_537295_1b7c235f ← picture/Fence/Cascade fences (MORE THAN 1)/001.png(CadQuery 实体;`_build_panel(x_shift)` 共享 helper;picket 循环发射;眼板+插销联接;**fork 基线首选**)
- rec_build-a-realistic-articulated-3d-model-of-a-casc_20260609_215018_605061_87cb6f73 ← picture/Fence/Cascade fences (MORE THAN 1)/002.png(管件 helper 风格;与 001 占同一格子,留作对照样本)

两个 parent 均为:管式竖杆板(picket)× 拱形脚 × N=2(`barrier_panel`/`linked_panel` 手写命名,未循环化)。

## 组合数预审

3(panel_style)× 2(feet_style)× 3(N 样本,保守下界)= 18 ≥ 10 ✓

## Slot 候选覆盖

### Slot A:panel_style(主机构槽——板体即被复制与铰接的主体)
| 候选(未来 module) | record_id | 关键 part/joint/helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| picket_tubular(基线) | rec_..._1b7c235f | `_frame_solid` / `_infill_solid`(21 竖杆循环) | 管框 + 竖杆 picket + 眼板插销 | parent(现成) |
| mesh_infill | rec_..._849ff922 | `_mesh_h_rods` / `_mesh_v_rods`(横竖杆循环,MESH_SPACING_X/Z) | 管框 + 横竖焊接网格 | converged |
| solid_half_panel | rec_..._5333dc9b | `_lower_plate_solid` / `_upper_pickets_solid` | 管框 + 下半实心板/上半短 picket | converged |

### Slot B:feet_style
| 候选(未来 module) | record_id | 关键 part/joint/helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| bridge_feet(基线) | rec_..._1b7c235f | `_feet_solid`(A 字撑管) | splayed A-frame 撑脚 | parent(现成) |
| flat_feet | rec_..._862eeff1 | `_feet_solid`(box plates,PLATE_LEN_X/Y) | 平板矩形配重脚,贴地 | converged |

## Multiplicity / Copy Logic

- count_param: `panel_count`(变体用 `N_PANELS`)
- N 样本已覆盖: {2(parent 基线), 4, 6} → rec_..._1b7c235f / rec_..._fc8fd294(n4) / rec_..._eac791d6(n6)
- 模板建议 N_range: **[2, 100]**(模板采样域;sweep 建议加权采样,小 N 高频、大 N 长尾,并为 sweep 设 N 上限以控编译时长)
- copied object: 整块 panel(含框、infill、脚、联接件),由共享 helper 发射
- naming: `panel_{i}`,循环 `for i in range(N_PANELS)`(n4/n6 变体已用此结构,可直接作 module 源码)
- placement: 沿链方向等距,首块为根
- joint policy: `hinge_{i}_{i+1}`,`panel_{i}→panel_{i+1}` REVOLUTE,竖直 Z 轴,原点在眼板/插销联接处,等限位
- **注意:两个 parent 的 N=2 是手写 `barrier_panel`/`linked_panel`,未循环化。n4/n6 变体已重写为 `panel_{i}` 循环链,模板应以变体(而非 parent)作为 multiplicity 的 copy-logic 源码。**

## 格子覆盖(cells = 5,全部 converged)

| # | 格子 | record_id |
|---|---|---|
| 1 | panel_style = mesh_infill | rec_..._849ff922 |
| 2 | panel_style = solid_half_panel | rec_..._5333dc9b |
| 3 | feet_style = flat_feet | rec_..._862eeff1 |
| 4 | panel_count = 4 | rec_..._fc8fd294 |
| 5 | panel_count = 6 | rec_..._eac791d6 |

完整 record id 见 `picture_expansion/generated_assets.jsonl`。

## 排除项(未来 compatibility matrix 素材)

- (暂无)
