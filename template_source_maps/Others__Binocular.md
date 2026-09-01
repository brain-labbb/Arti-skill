# Others / Binocular — template source map

pattern: mixed（固定 named slots:hinge_bridge 根 → left/right barrel 平行 fold 子件 + 中央 focus_wheel;diopter / focus ring / twist eyecup 为 barrel 上的链式子件。无模板级复制数量轴——barrel 恒为 ×2）

parents（1 个母资产,中央铰链 Porro-prism 双筒）:
- P1 rec_model-a-pair-of-classic-porro-prism-binoculars-2_20260610_085123_938411_a1874ba2 ← picture/Others/Binocular/001.png（经典 20x50 Porro:offset stepped barrels + 中央 hinge bridge + 中央 focus wheel + 右目镜 diopter ring;4 非 fixed joint:fold×2 REVOLUTE + focus CONTINUOUS + diopter REVOLUTE;**全批 fork 母资产**）

批次：others_binocular_qwen37max_20260620（dashscope qwen3.7-max / medium）。5 变体全部 compile=success、workbench-only、≥1 非 fixed joint、单轴控制变量、仍明确读作双筒望远镜。

## 组合数预审

barrel_prism_layout 3 × focus_mechanism 3 × eyecup_style 2 = **18 ≥ 10** ✓。

## Slot 候选覆盖

### Slot A:barrel_prism_layout（主 footprint 槽——光学筒/棱镜排布）
| 候选(未来 module) | record_id | 关键 part/joint/helper | 结构特征 | 状态 |
|---|---|---|---|---|
| porro_offset（基线） | P1 | `_add_barrel` / `_objective_tube_mesh` / `_housing_mesh`;objective 轴外侧(OBJ_Y≈0.065)、eyepiece 轴内侧(EYE_Y≈0.032),stepped prism housing 跨接 | 经典阶梯式偏置 Porro 双筒(宽 W 形) | parent |
| roof_straight | rec_binocular_var_roof_prism | `_barrel_body_mesh`(等径直筒);每筒 objective 与 eyepiece **同轴**直通,两筒近距平行(IPD≈0.064),保留 4 关节拓扑 | 现代纤细直筒 roof-prism 双筒 | converged（已同步） |
| reverse_porro_compact | rec_binocular_var_reverse_porro | `_add_barrel` 互换横向偏置:objective 轴内移(≈0.026)、eyepiece 轴外展(≈0.038),整体收小 | 紧凑 reverse-Porro 双筒 | converged |

### Slot B:focus_mechanism（主机构槽——调焦机构）
| 候选 | record_id | 关键 part/joint | 结构特征 | 状态 |
|---|---|---|---|---|
| center_wheel_diopter（基线） | P1 | `focus_wheel` part + `bridge_to_focus_wheel`(CONTINUOUS, +X);`diopter_ring` + `right_barrel_to_diopter_ring`(REVOLUTE, +X, ±60°) | 中央调焦轮 + 右目镜屈光度环 | parent |
| individual_focus | rec_binocular_var_individual_focus | `_focus_ring_mesh` helper;`focus_ring_{i}`(i∈0..1,`for i in range(2)`)+ `barrel_to_focus_ring_{i}`(REVOLUTE 每目镜);**无中央轮、无 diopter** | 左右目镜各自独立调焦环(IF/海军式) | converged |
| fixed_focus | rec_binocular_var_fixed_focus | **无 focus_wheel / 无 diopter** part 或 joint;仅 `bridge_to_left/right_barrel` fold;run_tests 显式断言两者缺失 | 免调焦/密封定焦双筒 | converged |

### Slot C:eyecup_style（目镜罩形态）
| 候选 | record_id | 关键 part/joint/helper | 结构特征 | 状态 |
|---|---|---|---|---|
| rubber_fold（基线） | P1 | `_eyecup_mesh`(软橡胶 lathe 罩,作为 barrel visual,无 joint) | 固定折叠橡胶眼罩 | parent |
| twist_up | rec_binocular_var_twist_eyecup | `_twist_up_eyecup_collar_mesh`;`eyecup_collar_{i}`(i∈0..1 循环)+ `{left,right}_barrel_to_eyecup_collar_{i}`(PRISMATIC ≈0.008 沿视轴) | 旋升/伸缩眼罩(戴镜者用) | converged |

> Slot C 仅 2 候选:眼罩的真实结构词汇表本质就是「固定折叠」vs「旋升伸缩」两族(翼形/卷边只是折叠族的外观微变,非新拓扑)。下游 spec 若需第 3 候选可加 `winged_fold`,但按 SPEC_TEMPLATE §4 「样本池不足降到 2 并说明理由」处置即可。

## Multiplicity / Copy Logic
- **无模板级复制数量逻辑(无 `*_count`)**:核心结构由固定 named slots 表达。barrel 恒为 ×2(双筒定义),不是可变 N。
- module-local 固定循环(非模板轴):每筒 hinge sleeve/arm 经 `zip(...)` 循环发射;`individual_focus` 的 `focus_ring_{i}` 与 `twist_eyecup` 的 `eyecup_collar_{i}` 均为 `for i in range(2)` 循环(固定 2,左右各一,共享 helper + 统一 joint policy),不暴露为模板 count 参数。
- 模板建议 N_range:无(barrel count 固定 2)。

## copied object / naming / placement / joint policy
- copied object:左右对称的 barrel 子件(经 `_add_barrel(model, name, side, ...)`,side=±1 镜像);focus_ring / eyecup_collar 子件。
- naming:`left_barrel` / `right_barrel`;循环子件 `focus_ring_{i}` / `eyecup_collar_{i}`。
- placement:沿 ±Y 镜像偏置(side·offset),视轴沿 +X。
- joint policy:fold = 两个 REVOLUTE 绕中央 +X hinge 轴(对向折叠 ±25°);focus ring = REVOLUTE 绕各自视轴;eyecup collar = PRISMATIC 沿视轴。

## 排除项（未来 compatibility matrix 素材）
- 无。5 个计划格子全部一次收敛。母资产已满足 §4 可读性契约(barrel 经 `_add_barrel` helper 镜像发射、hinge lug 经 zip 循环),无需 multiplicity loop-rewrite。
- 兼容性提示(供下游 spec compatibility matrix):`fixed_focus` 与 `twist_up` 可共存(定焦也能配旋升眼罩);`individual_focus` 的目镜调焦环与 `twist_up` 眼罩同在目镜端,组合时需注意 PRISMATIC eyecup 与 REVOLUTE focus ring 的轴向间隙(下游模板做 clearance 校验)。
