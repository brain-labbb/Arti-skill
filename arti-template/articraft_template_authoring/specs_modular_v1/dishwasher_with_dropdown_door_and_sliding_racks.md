## 元信息

| 项 | 值 |
|---|---|
| slug | `dishwasher_with_dropdown_door_and_sliding_racks` |
| template path | `agent/templates/dishwasher_with_dropdown_door_and_sliding_racks.py` |
| test path (optional) | `tests/agent/test_dishwasher_with_dropdown_door_and_sliding_racks_template.py` |
| stage | `TEMPLATE_AFTER_REVIEW` |
| status | `approved` |
| __modular__ | `True` |
| pattern | `mixed` |

## 5 星样本阅读摘要

| 项 | 值 |
|---|---|
| five_star_total | 18 |
| read_count | 18 |
| read_scope | 类别内全部 18 个 5 星样本；每条均读取 `record.json`、`revision.json`、`prompt.txt`、`model.py` |
| source_index_policy | 仅索引下方采纳 module source |

结构族分布：

| 结构族 | 样本数 | 说明 |
|---|---:|---|
| Box 薄壁开放前腔 tub | 8 | 侧/后/底/顶/前框拼合 |
| CadQuery 布尔中空壳 | 8 | `mesh_from_cadquery` 一体掏空 |
| 外箱 + 内胆分离 | 2 | `body` 外壳 + `tub_*` 衬里 |
| 双轨搁架（lower + upper） | 9 | 2× PRISMATIC slide |
| 三轨搁架（+ cutlery/utensil tray） | 9 | 3× tub-bearing slide |
| 下拉门底铰 REVOLUTE | 16 | 轴多为 ±X；少数 ±Y 或前缘铰 |
| 门顶/机身/独立 pod 控制区 | 3 挂载族 | door strip / body fascia / control_pod |

## 核心身份

**落地洗涤腔（tub/body）** 承载 **1–3 个 PRISMATIC 滑动搁架**（沿导轨向用户方向抽出），外加 **REVOLUTE 下拉前门**（底铰链或前下缘铰链，关闭封腔、打开向下/向外摆开）。腔内通常有 **CONTINUOUS 旋转喷淋臂**；控制旋钮/按钮多在门顶缘、机身前面板或独立 `control_pod`。

类别定义性关节：**`door_hinge` + `rack_slide[*]` + `spray_spin[*]`**。

边界：
- 不是 `drawer_cabinet_with_sliding_drawers`（水平抽屉盒，非洗涤腔+搁架篮）。
- 不是 `chest_freezer_with_hinged_lid`（顶盖翻转，非底铰门下摆）。
- 不是 `display_freezer_with_sliding_glass_lids`（玻璃滑盖冷柜）。

## 槽位 + 候选模块表

### Slot A：chassis_shell

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| `box_open_tub` | rec_dishwasher_with_dropdown_door_and_sliding_racks_0e1506ab39624ec6b0cde75e775225db | L92-L112 | eligible if compatible | 多块 Box 拼薄壁五面+前框；侧轨焊在 tub |
| `cq_hollow_shell` | rec_dishwasher_with_dropdown_door_and_sliding_racks_2fcd8639e5af4667a778aefdf4456d61 | L26-L46, L151-L156 | eligible if compatible | CQ 布尔掏空单 mesh + 导轨 Box |
| `case_plus_liner` | rec_dishwasher_with_dropdown_door_and_sliding_racks_6031139969f5409cbb18a74a9ff4f52d | L198-L286 | eligible if compatible | 外 `body` 壳 + 嵌套 `tub_*` 衬里片 |

### Slot B：door_dropdown

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| `hinge_x_standard` | rec_dishwasher_with_dropdown_door_and_sliding_racks_51172c0a32d641098b31e50c5fcae5b7 | L223-L231 | eligible if compatible | `tub_to_door` REVOLUTE axis `(1,0,0)` @ 底铰 |
| `hinge_y_depth_cabinet` | rec_dishwasher_with_dropdown_door_and_sliding_racks_11149fda5e9f4bc48c6a31d416e25a14 | L502-L515 | eligible if compatible | `body_to_door` axis `(0,-1,0)`；滑架沿 X |
| `hinge_inverted_pose` | rec_dishwasher_with_dropdown_door_and_sliding_racks_aece8961933f436dae93467c4bf49035 | L144-L154 | eligible if compatible | limits `lower=-1.45, upper=0`；q=0 为 service-open |
| `hinge_front_edge_drop` | rec_dishwasher_with_dropdown_door_and_sliding_racks_9dd15c0aefb940f2acd9fd2596b80ced | L360-L368 | eligible if compatible | 前缘铰 `@ BODY_D/2`，axis `(-1,0,0)` |

### Slot C：rack_multiplicity（joint-bearing）

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| `2_racks` | rec_dishwasher_with_dropdown_door_and_sliding_racks_73bc836aa4d14912946b5cd580c15331 | L234-L251 | eligible if compatible | `lower_rack` + `upper_rack` 双 PRISMATIC |
| `3_racks` | rec_dishwasher_with_dropdown_door_and_sliding_racks_f64b47dba9c54168b352c390fca47511 | L182-L242 | eligible if compatible | lower + upper + `cutlery_tray` 第三滑轨 |

### Slot D：rack_geometry

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| `wire_cylinder` | rec_dishwasher_with_dropdown_door_and_sliding_racks_0e1506ab39624ec6b0cde75e775225db | L39-L79 | eligible if compatible | Cylinder 杆 + runner Box 网格 |
| `wire_box_grid` | rec_dishwasher_with_dropdown_door_and_sliding_racks_65b09535f3d149529a3a068b0f2f2164 | L41-L90 | eligible if compatible | 细 Box 十字网格 + 固定 tine |
| `cq_fused_basket` | rec_dishwasher_with_dropdown_door_and_sliding_racks_aece8961933f436dae93467c4bf49035 | L36-L86, L206-L228 | eligible if compatible | `_rack_geometry` → `mesh_from_cadquery` 熔丝篮 |
| `shallow_cutlery_tray_layer` | rec_dishwasher_with_dropdown_door_and_sliding_racks_f64b47dba9c54168b352c390fca47511 | L67-L75, L223-L233 | eligible if compatible | `shallow=True` 矮壁分隔篮（第三层） |

### Slot E：control_cluster

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| `door_top_strip` | rec_dishwasher_with_dropdown_door_and_sliding_racks_0e1506ab39624ec6b0cde75e775225db | L146-L181 | eligible if compatible | 门顶旋钮 CONTINUOUS + 按钮 PRISMATIC |
| `body_fascia` | rec_dishwasher_with_dropdown_door_and_sliding_racks_11149fda5e9f4bc48c6a31d416e25a14 | L617-L645 | eligible if compatible | `body_to_selector_knob` + `body_to_button_*` |
| `fixed_control_pod` | rec_dishwasher_with_dropdown_door_and_sliding_racks_22e171c2d3a6417697e69e5386d27ec4 | L193-L342 | eligible if compatible | `chassis_to_control_pod` FIXED；pod 上旋钮/rocker |

## 槽位图（slot graph）

pattern: `mixed`（multiplicity + parallel children）

```text
[chassis_shell] (grounded tub/body)
    |-- REVOLUTE --> [door_dropdown]
    |-- PRISMATIC -Y/X --> [rack_0]
    |-- PRISMATIC -Y/X --> [rack_1]
    |-- PRISMATIC (optional) --> [rack_2 / cutlery_tray]
    |-- CONTINUOUS --> [lower_spray_arm]
    |-- CONTINUOUS --> [upper_spray_arm]
[door_dropdown] -- optional child controls --> [door_top_strip parts]
[chassis_shell] -- optional --> [body_fascia] or [fixed_control_pod]
```

- `rack_multiplicity` 决定 tub 直接 bearing 的 slide 关节数（2 或 3）。
- `rack_geometry` 定义每个 rack part 的 wire/cq/box 建造方式；第三层可与 `shallow_cutlery_tray_layer` 绑定。
- `control_cluster` 与门/机身互斥：同一前面不可双主控区。

## 每槽位 Module Emits / Interfaces

### Slot A / chassis_shell

| module | emits | 描述 | 来源 |
|---|---|---|
| `box_open_tub` | parts | fixed `tub`；floor/ceiling/sides/front gasket | 0e15 / L92-L112 |
| `box_open_tub` | downstream | side rails `lower_rail_*`/`upper_rail_*` @ distinct Z | 0e15 / L109-L112 |
| `cq_hollow_shell` | parts | `cabinet` + `thin_wall_tub` mesh | 2fcd / L151-L156 |
| `case_plus_liner` | parts | `body` 外壳 + `tub_wall_*` 衬里 | 6031 / L255-L286 |

### Slot B / door_dropdown

| module | emits | 描述 | 来源 |
|---|---|---|
| `hinge_x_standard` | parts + joint | `door` panel + `tub_to_door` REVOLUTE X | 5117 / L223-L231 |
| `hinge_y_depth_cabinet` | parts + joint | 深度向机箱铰链；改变 rack slide 轴系 | 1114 / L502-L515 |
| `hinge_inverted_pose` | joint policy | service-open 为 rest pose | aece / L144-L154 |
| `hinge_front_edge_drop` | parts + joint | 前下缘铰链摆门 | 9dd1 / L360-L368 |

### Slot C / rack_multiplicity

| module | emits | 描述 | 来源 |
|---|---|---|
| `2_racks` | joints | `*_to_lower_rack`, `*_to_upper_rack` PRISMATIC | 73bc / L234-L251 |
| `3_racks` | joints | 上述 + `*_to_cutlery_tray`；需第三对导轨 | f64b / L223-L242 |

### Slot D / rack_geometry

| module | emits | 描述 | 来源 |
|---|---|---|
| `wire_cylinder` | visuals | Cylinder 杆篮 + runner | 0e15 / L39-L79 |
| `wire_box_grid` | visuals | Box 焊丝网格 + tine | 65b0 / L41-L90 |
| `cq_fused_basket` | visuals | CQ union 熔丝篮 mesh | aece / L36-L86 |
| `shallow_cutlery_tray_layer` | visuals | 矮分隔篮（第三层专用） | f64b / L67-L75 |

### Slot E / control_cluster

| module | emits | 描述 | 来源 |
|---|---|---|
| `door_top_strip` | parts + joints | door 子件旋钮/按钮 | 0e15 / L146-L181 |
| `body_fascia` | parts + joints | body 前面板控件 | 1114 / L617-L645 |
| `fixed_control_pod` | fixed + joints | 独立 pod 控件 | 22e1 / L193-L342 |

## 参数范围汇总

| 参数 | 类型 | 取值范围 / 候选值 | 默认 | 派生关系 | 来源 |
|---|---|---|---|---|---|
| `chassis_shell` | enum | 3 modules | sampled | procedural | Slot A |
| `door_dropdown` | enum | 4 modules | sampled | gated by slide axis family | Slot B |
| `rack_multiplicity` | enum | `2_racks`, `3_racks` | sampled | joint-bearing count | Slot C |
| `rack_geometry` | enum | 4 modules | sampled | 3_racks 可强制 shallow 第三层 | Slot D |
| `control_cluster` | enum | 3 modules | sampled | 与门/机身挂载互斥 | Slot E |
| `tub_width` | float | [0.50, 0.76] | sampled | 派生 rack 宽 | aece / L101-L102 |
| `tub_depth` | float | [0.54, 0.80] | sampled | 派生 slide travel | aece / L213-L216 |
| `rack_travel_scale` | float | [0.80, 1.15] | sampled | clamp slide upper limit | 73bc / L234-L251 |

## Multiplicity / Copy Logic

- `count_param`: `rack_count`（tub-bearing PRISMATIC slides）
- `N_range`: `2` 或 `3`；仅统计 tub/body 直接 parent 的 rack slide 关节。
- sampling domain: `2_racks` 或 `3_racks`；`3_racks` 须 chassis 具备第三对导轨 @ distinct Z。
- copied object: `lower_rack`, `upper_rack`, 可选 `cutlery_tray`/`utensil_tray`。
- naming: `lower_rack`/`upper_rack`/`cutlery_tray` 或 `rack_0`/`rack_1`/`rack_2`。
- placement: slide origin @ tub 内导轨 Z；axis 与 `door_dropdown` 坐标系一致（多数 `(0,-1,0)`，Y-hinge 族为 `(−1,0,0)`）。
- joint policy: 每个 rack 一对 PRISMATIC；caddy FIXED 到 lower 不计入 tub-level count。
- source/gating: 第三层需 `shallow_cutlery_tray_layer` 或等效 shallow 几何；双轨禁止第三 slide。

## 拓扑多样性审计

总组合数：3 × 4 × 2 × 4 × 3 = 288 raw；gating 后估计 **40–60** 合法组合。


理由：壳体、门铰语义、搁架层数、篮几何、控制挂载五维真实差异；18 样本覆盖 ~12–15 独立原型。

seed_domain_policy：procedural_first

Procedural Sampling / Sweep Plan：
- sampler：`chassis_shell` → legal `door_dropdown` → `rack_multiplicity` → `rack_geometry` → `control_cluster`。
- `seed=0` 不特殊。
- gating：`hinge_y_depth` 须配 slide_axis `neg_x`；`control_pod` ⊥ `door_top_strip`；`3_racks` 须 tub 高度足够。
- rack 须保留源样本 mesh/cadquery 复杂度，不得统一降级 Box。
- sweep 0-49 初验；viewer 检查关门姿态、搁架抽出、门下摆。

Topology target：1000-seed distinct 建议 ≥80。（统一口径：富类别建议 ≥300；低于 300 记录真实组合空间或兼容约束原因；report-only，不设门。）

| item | policy | validator / viewer focus |
|---|---|---|
| sampler | chassis → door → rack count → geometry → controls | slot_choices 一致 |
| compatibility matrix | 见下表 | slide 轴与铰链坐标系一致 |
| controlled local variation | tub 尺寸、travel scale | 不破坏导轨对齐 |
| regression overrides | none 初版 | — |
| random sweep | 0-49 | diversity + hinge/rack motion |

| slot | candidate_count | 是否 ≥2 | 是否 ≥3 | 备注 |
|---|---:|---|---|---|
| chassis_shell | 3 | yes | yes | box/cq/case-liner |
| door_dropdown | 4 | yes | yes | X/Y/inverted/front |
| rack_multiplicity | 2 | yes | no | joint-bearing 2/3 |
| rack_geometry | 4 | yes | yes | cyl/box/cq/shallow |
| control_cluster | 3 | yes | yes | door/body/pod |

### Compatibility matrix / gating

| chassis_shell | legal door | legal rack_count | gating |
|---|---|---|---|
| `box_open_tub` | hinge_x, inverted | 2, 3 | 标准族 |
| `cq_hollow_shell` | hinge_x, inverted, front_edge | 2, 3 | 需 CQ tub 高度 |
| `case_plus_liner` | hinge_x, front_edge | 2 | body fascia 优先 |

| door_dropdown | slide_axis | legal control | gating |
|---|---|---|---|
| `hinge_x_standard` | `(0,-1,0)` | door_top, body_fascia | 默认 |
| `hinge_y_depth_cabinet` | `(-1,0,0)` | body_fascia | 禁止 door_top 条 |
| `hinge_front_edge_drop` | `(0,1,0)` | control_pod | pod 族 |

| rack_multiplicity | required geometry | gating |
|---|---|---|
| `3_racks` | shallow 第三层或 cutlery tray | 须第三对 rail |

## Validator

- `__modular__ = True`
- 存在 `REVOLUTE` dropdown door + ≥2 `PRISMATIC` rack slides
- `rack_count` 与 joint 数一致
- cq_fused_basket 使用 `mesh_from_cadquery`，非纯 Box 替代
- hinge pin/knuckle 允许 element-scoped `allow_overlap`
- rack 在腔内滑动，closed pose 不与 door 穿模（除允许铰链区）

## Reject cases

- 无下拉门底铰/前缘铰
- 搁架无 PRISMATIC slide
- 仅换不锈钢颜色/尺寸
- 抽屉盒替代开放篮搁架
- 顶盖冰箱式开门
- 无洗涤腔喷淋语义
- `control_pod` 与 `door_top_strip` 双主控叠放
- 第三轨无对应导轨 Z

## 与相邻类别的边界

- 不该混入：`drawer_cabinet_with_sliding_drawers`（封闭抽屉 vs 开放篮+洗涤腔）。
- 不该混入：`chest_freezer_with_hinged_lid`（顶盖 vs 底铰门）。
- 不该混入：`elevator` / `threestage_telescoping_slide`（套筒伸缩 vs 搁架滑轨）。
- 不该混入：纯柜体无 rack slide。

## 审核记录

| 项 | 结论 |
|---|---|
| reviewer status | approved |
| reviewer notes | 用户 2026-06-09 确认通过；进入 TEMPLATE_AFTER_REVIEW |

## 模板实现备注（可选）

- `cq_fused_basket` 必须保留 `_rack_geometry` CQ union 路径（aece/f64b/22e1）。
- `hinge_inverted_pose` 的 rest pose 与测试断言须与源样本一致。
- spray arm 建议作为 module-local fixed/continuous children，不单独占 slot。
- 附件（mug_shelf、tine_bank、caddy slide）初版可作为 module-local optional，不进入主 slot 域。

## Module Source Index

| source_id | slot | module | sample_id | model.py 来源 | 采纳用途 |
|---|---|---|---|---|---|
| S1 | A | `box_open_tub` | rec_…_0e1506ab | L92-L112 | thin-wall tub |
| S2 | A | `cq_hollow_shell` | rec_…_2fcd8639 | L26-L46, L151-L156 | CQ hollow shell |
| S3 | A | `case_plus_liner` | rec_…_60311399 | L198-L286 | body + liner |
| S4 | B | `hinge_x_standard` | rec_…_51172c0a | L223-L231 | standard X hinge |
| S5 | B | `hinge_y_depth_cabinet` | rec_…_11149fda | L502-L515 | Y-axis door |
| S6 | B | `hinge_inverted_pose` | rec_…_aece8961 | L144-L154 | inverted limits |
| S7 | B | `hinge_front_edge_drop` | rec_…_9dd15c0a | L360-L368 | front-edge hinge |
| S8 | C | `2_racks` | rec_…_73bc836a | L234-L251 | dual slide |
| S9 | C | `3_racks` | rec_…_f64b47db | L182-L242 | triple slide |
| S10 | D | `wire_cylinder` | rec_…_0e1506ab | L39-L79 | cyl wire rack |
| S11 | D | `wire_box_grid` | rec_…_65b09535 | L41-L90 | box grid rack |
| S12 | D | `cq_fused_basket` | rec_…_aece8961 | L36-L86, L206-L228 | CQ basket |
| S13 | D | `shallow_cutlery_tray_layer` | rec_…_f64b47db | L67-L75, L223-L233 | shallow 3rd |
| S14 | E | `door_top_strip` | rec_…_0e1506ab | L146-L181 | door controls |
| S15 | E | `body_fascia` | rec_…_11149fda | L617-L645 | body controls |
| S16 | E | `fixed_control_pod` | rec_…_22e171c2 | L193-L342 | control pod |
