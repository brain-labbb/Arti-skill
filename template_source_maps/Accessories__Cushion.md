# Accessories / Cushion — template source map

> 注意:此处 "Cushion" = 化妆**气垫粉盒 (cosmetic cushion powder compact)**,非抱枕。结构家族 =
> 翻盖式粉盒:中空 base 碗 + 粉盘 + 翻盖(盖内镜子/标签)+ 后铰链 + 前卡扣。

pattern: mixed（固定 named slots:case_footprint + lid_mechanism + interior;外加 compartment_count 多重性轴）

parents（2 个母资产,均 base+lid+1 REVOLUTE 翻盖粉盒;基线 = footprint × rear_flip_hinge × single_powder_pan × inner_mirror）:
- rec_create-a-highly-detailed-articulated-round-cushi_20260605_133628_646636_81989983 ← picture/Accessories/Cushion/002.png（圆形,中空 translucent 碗 + 粉盘 + 后翻盖含镜子/花徽标签;**round footprint 基线**）
- rec_create-a-highly-detailed-articulated-square-luxu_20260605_133604_487371_876ccf2a ← picture/Accessories/Cushion/001.png（方形 luxury,黑底白盖 + chrome 分模线 + 盖内镜子;**square footprint 基线**）

## 组合数预审

case_footprint 3 × lid_mechanism 3 × interior 3 × compartment_count N∈{1,2,3} = 81 ≫ 10 ✓

## Slot 候选覆盖

### Slot A:case_footprint（外形/足迹;连续比例由模板缩放,这里列结构不同的足迹形态）
| 候选（未来 module） | record_id | 结构特征 | 状态 |
|---|---|---|---|
| round（基线） | P_round | 圆形中空碗 + 圆盖 | parent |
| square（基线） | P_square | 圆角方形碗 + 方盖 | parent |
| oval | rec_cushion_var_oval | 椭圆拉长足迹（cadquery profile 重写,base+lid 部件树不变） | converged |

### Slot B:lid_mechanism（开合机构 —— 主机构槽）
| 候选 | record_id | 关键 joint | 状态 |
|---|---|---|---|
| rear_flip_hinge（基线） | parents | 后单铰翻盖 REVOLUTE | parent |
| clamshell_two_leaf | rec_cushion_var_clamshell | leaf_rear/leaf_front 双叶蛤壳 base_to_leaf_* REVOLUTE ×2 | converged |
| slide_lid | rec_cushion_var_slide_lid | base_to_lid 横向滑盖 PRISMATIC | converged |

### Slot C:interior（内部机构）
| 候选 | record_id | 关键 joint | 状态 |
|---|---|---|---|
| single_powder_pan（基线） | parents | 单粉盘座 | parent |
| removable_refill_cartridge | rec_cushion_var_refill_cartridge | base_to_refill PRISMATIC 提起 + base_to_lid REVOLUTE | converged |
| flip_puff_tray | rec_cushion_var_puff_tray | base_to_puff_tray REVOLUTE + base_to_lid REVOLUTE | converged |

### Slot D（多重性）:compartment_count
| N | record_id | 结构 | 状态 |
|---|---|---|---|
| 1（基线） | parents | 单粉盘 | parent |
| 2 | rec_cushion_var_dual_pan | 双粉盘并排,for i in range(2) loop 发射 | converged |
| 3 | rec_cushion_var_triple_pan | 三粉盘并排,for i in range(3) loop 发射 | converged |

## Multiplicity / Copy Logic
- count_param: **`pan_count`**（base 内并排粉盘数）
- N 样本计划覆盖: {1, 2, 3}
- 模板建议 N_range: [1, 4]（粉盒内粉盘数现实上限小）
- copied object: 单只粉盘 (`_pan_mesh`),共享 helper 发射
- naming: `pan_{i}`,`for i in range(N)`
- placement: 沿一轴等距并排
- joint policy: 粉盘随 base（非移动则 inline 为 base visual,Rule 1）;翻盖 REVOLUTE 提供活动关节

## 格子覆盖（P1 完成）

| 槽 | 候选数(含基线) | converged 变体 |
|---|---|---|
| A case_footprint | 3 | oval（round/square 为 parent 基线） |
| B lid_mechanism（主机构） | 3 | clamshell / slide_lid |
| C interior | 3 | refill_cartridge / puff_tray |
| D compartment_count | N∈{1,2,3} | dual_pan(N=2) / triple_pan(N=3) |

7 个变体全部 compile=success、≥1 非fixed joint、collections=['workbench']（未进 dataset collection）、单轴 diff。每槽 ≥2 候选 + multiplicity ≥2 个 N → 满足 FORK_VARIANTS §8 完成定义。**Cushion 小类样本池就绪。**

注:7 个变体 `category_slug=accessories` 系**继承自两个 parent**（parent 本身 workbench-only 但带此 slug 标签）；`'dataset' not in collections`,即未 promote 进 curated 数据集,符合"不污染论文数据集"的真实意图。

## 排除项（不收敛/出类目项）
- 无。本批 7/7 收敛,无出类目。
