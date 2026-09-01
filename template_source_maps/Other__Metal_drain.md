# Other / Metal drain — template source map

pattern: mixed（固定 named slots:flange_shape + grate_pattern + grate_mechanism(主机构);grate_pattern 含 slot_count 连续多重性）

parents（1 个母资产,方形不锈钢浴室地漏）:
- rec_model-a-square-stainless-steel-bathroom-floor-dr_20260610_084452_893664_1f470a4a ← picture/Other/Metal drain/001.png（基线 = flange:square × pattern:pinwheel_slots × mechanism:twist_lock + lift_out（REVOLUTE twist + PRISMATIC lift））

批次：other_metal_drain_qwen37max_20260617（dashscope qwen3.7-max / medium）。6 变体全部 compile=success、workbench-only、≥1 非 fixed joint。

## 组合数预审

flange 3 × pattern 3 × mechanism 3 = 27 ≫ 10。mechanism 含 twist+lift(REVOLUTE+PRISMATIC)/hinged_flip(REVOLUTE)/popup(PRISMATIC) 多拓扑。

## Slot 候选覆盖

### Slot A:flange_shape（外法兰足迹）
| 候选 | record_id | 结构特征 | 状态 |
|---|---|---|---|
| square（基线） | parent | 方形法兰 + 锥杯 | parent |
| round_circular | rec_variant-flange-shape-round-circular-change-the-s_20260617_114250_756469_b8d32e6f | 圆形法兰(lathe) | converged(2) |
| hexagonal | rec_variant-flange-shape-hexagonal-change-the-square_20260617_114250_757888_4b1ebd24 | 六边形法兰 | converged(2) |

### Slot B:grate_pattern（篦子穿孔图样;含 slot_count/ring_count 连续多重性）
| 候选 | record_id | 结构特征 | 状态 |
|---|---|---|---|
| pinwheel_slots（基线） | parent | 四象限风车槽(`for group/for slot` 循环) | parent |
| concentric_rings | rec_variant-grate-pattern-concentric-rings-replace-t_20260617_114250_752376_d68de668 | 同心环槽(循环 rings) | converged(2) |
| square_grid_holes | rec_variant-grate-pattern-square-grid-holes-replace-_20260617_114250_759791_e214b2de | 方格孔阵(嵌套循环) | converged(2) |

### Slot C:grate_mechanism（**主机构槽**——篦子动作）
| 候选 | record_id | 关键 joint | 状态 |
|---|---|---|---|
| twist_lock_lift_out（基线） | parent | `grate_lift` PRISMATIC(+Z) + `grate_twist` REVOLUTE(Z) | parent |
| hinged_flip_grate | rec_variant-grate-mechanism-hinged-flip-grate-replac_20260617_114250_750354_3cfbbefa | 单边翻起 REVOLUTE(水平) | converged(1) |
| popup_center_plug | rec_variant-grate-mechanism-popup-center-plug-add-a-_20260617_114543_389051_a0a6918d | 中心 pop-up 塞 PRISMATIC(+Z) | converged(3) |

## Multiplicity / Copy Logic
- count_param: **`slot_count` / `ring_count` / `hole_grid`**（篦子穿孔单元数,grate_pattern 槽内的连续复制逻辑）
  - 母资产: `N_GROUPS=4 × SLOTS_PER_GROUP`,风车槽循环发射;环槽/方格孔变体各自循环
  - 模板建议 N_range: slot/ring/hole 数 [4, 60]（视盘径连续缩放,sweep 友好）
  - copied object: 单槽/单环/单孔(CadQuery cut);naming 循环 index;placement 等角/等径/网格;joint policy 全部随 grate 体动(无独立关节)
  - 主结构(flange/grate/mechanism)为固定 named slots。

## 格子覆盖

| 槽 | 候选数(含基线) | 空格已填 |
|---|---|---|
| A flange_shape | 3 | round / hexagonal(2) |
| B grate_pattern | 3 | concentric / square_grid(2) |
| C grate_mechanism(主机构) | 3 | hinged_flip / popup(2) |

每槽 ≥2 → 满足 §8。**Metal drain 小类样本池就绪。**

## 排除项
- rectangular_linear（长条淋浴槽地漏）需同时改 flange + grate 形状(twist-lock 不适用长条篦),为耦合双轴,留作 compatibility matrix 素材,未纳入单轴格子。
