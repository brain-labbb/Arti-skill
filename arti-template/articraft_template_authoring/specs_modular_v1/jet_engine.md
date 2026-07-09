# Jet Engine Spec

## 元信息
| 项 | 值 |
|---|---|
| slug | `jet_engine` |
| template path | `agent/templates/jet_engine.py` |
| test path | `tests/agent/test_jet_engine_template.py` |
| stage | `TEMPLATE_AFTER_REVIEW` |
| status | `approved` |
| __modular__ | `True` |
| pattern | `mixed` |

## 5 星样本阅读摘要
| 项 | 值 |
|---|---|
| five_star_total | 20 |
| read_count | 20 |
| read_scope | all 5-star samples in category `jet_engine`; each record's `record.json` (rating==5 confirmed), active `revision.json`/`rev_000001/model.py`, and `prompt.txt` were read in full |
| samples_adopted_as_module_sources | 20 |
| samples_read_but_not_adopted | 0 |
| source_index_policy | only adopted module sources are indexed below (see Module Source Index) |

枚举方式：扫描 `data/records/*/record.json`，过滤 `category_slug == "jet_engine"` 且 `rating == 5`，得到 20 个样本，全部读取，无抽样。类别评分分布 `{3:1, 4:19, 5:20}`。

### 关键结论（决定本 spec 的拓扑轴）

- **唯一普适运动**：20/20 样本都有「转子绕发动机中轴线 CONTINUOUS 自转」这一主自由度。被删除的旧模板用两条 FIXED 串联三段、零可动关节，正是违反这一类别身份的根本错误。
- 转子配置存在 3 种真实拓扑：单前风扇（front fan，1×CONTINUOUS，最常见）/ 涡喷压气机转子（compressor rotor，无风扇，1×CONTINUOUS）/ 双轴（fan + core spool，2×独立 CONTINUOUS）。
- 尾段存在 2 种真实拓扑：静态排气锥（0 额外关节）/ 可变喷口花瓣（N×REVOLUTE，部分用 `Mimic` 联动），外加一种静态 chevron 变体。
- 安装/接地存在 3 种真实拓扑：裸发动机模块（无外部支撑）/ 地面展示支架（独立 part + FIXED 链）/ 顶部挂架 pylon。
- 可选第二类铰接：检修口盖（belly fold-down 或 side bay door，1×REVOLUTE）。
- 轴向约定不统一：多数样本以世界 X 为中轴（几何沿局部 Z 建再 `rotate_y(π/2)` / `Origin(rpy=(0,π/2,0))`），少数（4f9be8、ad0120、e07fdb、efe964）直接以 Z 为中轴。模板必须统一为单一约定（见下：world X）。

## 核心身份

Jet engine 是航空燃气涡轮发动机：一个静态外罩（涡扇 nacelle 短舱 / 涡喷 cylindrical casing），内部一个或两个绕发动机纵轴连续旋转的转子（前风扇盘 / 压气机转子 / fan+core 双轴），加上进气唇口、中心 spinner、静子/支撑叶片、排气锥或可变喷口等模块。核心运动恒为转子的 CONTINUOUS 自转；可选的第二类运动是可变喷口花瓣（REVOLUTE，N 复制）和检修口盖（REVOLUTE）。发动机可裸装、坐在展示支架上、或挂在 pylon 下。

成熟域：单台完整发动机本体（含其 spinner / 静子 / 排气段）+ 至多一个安装结构 + 至多一组可变喷口 + 至多一个检修口盖。

边界（不该混入）：
- 不是 `ceiling_fan` / `axial_fan_rotor` / `box_fan`：那些是裸风扇，没有发动机短舱/核心机/排气段身份；jet engine 的转子必须封装在 nacelle/casing 内并有进气-核心-排气轴向叙事。
- 不是 `wind_turbine` / `traditional_windmill`：那些是塔架上的少叶大转子，无短舱/核心机。
- 不是 `single_rotor_helicopter` / `drone`：jet engine 没有机身/尾梁/起落架，转子轴是发动机纵向中轴而非垂直升力轴。
- 不是 `rocket_engine` / `cannon`：jet engine 必须有可见的旋转转子，而非纯静态喷管/身管。

## 采用源码索引（Adopted Source Index）

见文末「Module Source Index」（20 个源较多，集中放在末尾）。下文 slot table 引用 `S#` 与 `model.py:Lx-Ly`。

## 槽位 + 候选模块表

### Slot A：mount_platform（接地/安装结构，根部）

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| `bare_inline` | S5 `rec_jet_engine_268a98e6...` | L153-L208（nacelle 作为根 part，无外部支撑） | eligible if compatible | 无独立支撑 part；engine_core 的外罩直接作为根；最少 part、最少 FIXED 关节 |
| `ground_display_stand` | S19 `rec_jet_engine_ec6fd583...` | L186-L238（`stand_base`+`display_support` 多 part + FIXED 链） | eligible if compatible | 独立支架 part(s)（base/plinth + column/pylon_arm + saddle），经 FIXED 关节托住外罩；可单 part 或 base→support 两级链 |
| `top_pylon_mount` | S10 `rec_jet_engine_5abca3da...` | L247-L258（`pylon_mount` 独立 part + FIXED） | eligible if compatible | 顶部挂架（mount_pad + pylon_fairing），经 FIXED 接到外罩上表面；机翼吊装姿态 |

补充来源：`ground_display_stand` 另见 S3 `0003` L205-L233、S16 `ccaecd76` L150-L174、S6 `278c04` L109-L152（涡喷维护 cradle）。`top_pylon_mount` 另见 S15 `ad0120` L243-L260、S2 `0001` L368-L381（pylon 融入 nacelle，作为 module-local fused 变体）。

### Slot B：engine_core（外罩 + 转子，紧耦合的发动机本体）

> 外罩样式与转子配置在 5 星样本中强相关（涡喷 casing 必配压气机转子、不配风扇；cutaway 壳体专为暴露双轴而开）。按 workflow「变化不足以独立成 slot 时折入相邻 module」的规则，将 shell 与 rotor 合为一个 `engine_core` 模块族，候选之间是结构不同的本体家族，避免非法组合（如 smooth nacelle + 压气机面）。

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| `smooth_turbofan` | S2 `rec_jet_engine_0001` | L266-L508（封闭 nacelle 短舱 + 单前风扇 1×CONTINUOUS） | eligible if compatible | 封闭中空 lathe nacelle（进气唇 round cap）+ 中心 spinner + 单 fan_rotor part；1 个 CONTINUOUS 主轴；静态核心机/静子/排气锥作为外罩 visual |
| `turbojet_module` | S6 `rec_jet_engine_278c04c0...` | L159-L344（圆柱 casing + 前支撑蜘蛛 + 压气机转子） | eligible if compatible | 圆柱外壳 + 可见压气机面（无风扇/无 spinner）+ compressor_rotor part；1 个 CONTINUOUS 主轴；前 strut 蜘蛛把转子轴托在 casing 内 |
| `cutaway_twospool` | S14 `rec_jet_engine_a602a9de...` | L173-L375（开切壳体暴露 fan + 多级 core spool，2×CONTINUOUS） | eligible if compatible | 部分角度 C 形/开笼壳体暴露内部；fan_rotor + core_spool 两个独立 CONTINUOUS 同轴转子；最多 part/最复杂内部细节 |

补充来源：`smooth_turbofan` 另见 S5/S9/S10/S11/S16/S17/S19/S3/S4（封闭涡扇主力家族，见 Module Source Index）。`turbojet_module` 另见 S12 `5e2bb7` L70-L248（casing→support→rotor 两级链）。`cutaway_twospool` 另见 S1 `ec62c1e0` L211-L516（fan_spin + core_spin 双 CONTINUOUS）、S15 `ad0120` L86-L411（open cage + fan + core_spool 双 CONTINUOUS + chevrons）。

### Slot C：aft_nozzle（尾段/喷口）

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| `static_exhaust_cone` | S19 `rec_jet_engine_ec6fd583...` | L289-L312（`rear_exhaust` 锥/plug，FIXED 或并入外罩） | eligible if compatible | 静态排气锥/中心 plug；0 个额外可动关节；可作独立 FIXED part 或并入外罩 visual |
| `variable_petal_nozzle` | S18 `rec_jet_engine_e2123c10...` | L246-L272（petal 几何）, L371-L409（N 个 petal part + N×REVOLUTE） | eligible if compatible | 后部 `nozzle_ring`（FIXED 到外罩）+ N 个 petal part，每个一条 REVOLUTE（局部切向轴）；收敛/张开变面积喷口 |
| `static_chevron_ring` | S15 `rec_jet_engine_ad0120c0...` | L183-L214（`chevron_petals` 14 长+14 短，静态径向阵列） | eligible if compatible | 高涵道锯齿 chevron 静态环；0 个额外可动关节；与 `static_exhaust_cone` 拓扑同为静态尾段但视觉/边缘几何不同 |

补充来源：`variable_petal_nozzle` 另见 S8 `4f9be8` L203-L255（8 petal，用 `Mimic(joint="petal_0_hinge")` 把 7 个从动 hinge 联动到主 hinge）。`static_exhaust_cone` 另见 S3 `0003` L86-L99/L247-L251、S17 `e07fdb` L165-L176、S5 `268a98` L187-L191。

### Slot D：access_door（可选检修铰接）

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| `none` | —（多数样本无门，如 S2/S5/S14/S17） | n/a | eligible if compatible | 无检修门；外罩封闭，仅转子（+可选喷口）运动 |
| `belly_fold_down_panel` | S7 `rec_jet_engine_3ad5d74c...` | L230-L264（`access_panel` part + 下侧 REVOLUTE hinge） | eligible if compatible | 进气下方矩形检修盖，下铰链 REVOLUTE（轴 Y，lower=0→upper≈1.35）；外罩侧有固定 hinge 五金 |
| `side_equipment_bay_door` | S18 `rec_jet_engine_e2123c10...` | L334-L369（`bay_door` part + 侧 REVOLUTE hinge） | eligible if compatible | 侧面设备舱门，侧铰链 REVOLUTE（轴 Z）；外罩侧有 bay_housing + bay_equipment 静态内构 |

补充来源：`belly_fold_down_panel` 另见 S13 `6d9ed3b3` L307-L367。

> Reviewer note：Slot C 的 `variable_petal_nozzle` 与 Slot D 的 door 候选各自只有 1-2 个源家族（数据集里可变喷口仅 4f9be8/e2123c，门仅 3ad5d7/6d9ed3/e2123c）。这是允许的降级情形：每个候选都有真实可回溯源码，且都是结构不同的拓扑（额外 part + 额外关节）。不要在没有新 5 星样本的情况下臆造第三个门/喷口家族。

## 槽位图（slot graph）

pattern: `mixed`（root 接地 + 串联主轴 + 尾段 multiplicity + 并联可选门）

```text
[Slot A mount_platform]                      (bare_inline 时本 slot 为空, engine_core 外罩即根)
   └──FIXED (saddle/pylon_pad 接触面, 无 axis)──> [Slot B engine_core.outer_shell] (root if bare)
                                                     │
   engine_core 内部:                                 │
     outer_shell ──CONTINUOUS axis=(1,0,0)──> rotor_primary (fan_rotor | compressor_rotor)
     outer_shell ──CONTINUOUS axis=(1,0,0)──> core_spool        (仅 cutaway_twospool, 第二独立轴)
                                                     │
     outer_shell ──[Slot C aft_nozzle]──>           │
        static_exhaust_cone / static_chevron_ring : FIXED 或并入 outer_shell visual (0 DOF)
        variable_petal_nozzle :
            outer_shell ──FIXED──> nozzle_ring
            nozzle_ring ──REVOLUTE axis≈local Y──> nozzle_petal_i  (i=0..N-1, 可 Mimic 联动到 petal_0)
                                                     │
     outer_shell ──[Slot D access_door]──>          │
        none : 无
        belly_fold_down_panel : outer_shell ──REVOLUTE axis=(0,1,0), 下铰线──> access_panel
        side_equipment_bay_door : outer_shell ──REVOLUTE axis=(0,0,1), 侧铰线──> bay_door
```

接口点位说明：
- **平台→外罩**：FIXED；接触面是支架 saddle 上缘 / pylon mount_pad 下缘与外罩外表面的贴合面；`Origin.xyz` 把外罩抬到中轴高度（参考 S19 `support_to_nacelle` z=1.01、S16 `stand_to_nacelle` z=ENGINE_CENTER_Z=1.05、S6 `stand_to_engine` z=0.41）。
- **外罩→主转子**：CONTINUOUS，axis=world X `(1,0,0)`，origin 在转子盘前方中轴点（参考 S2 FAN_JOINT_X、S18 FAN_X、S19 `Origin(xyz=(-0.40,0,0))`）；转子 hub/shaft 经轴承 collar/socket 捕获在外罩中轴上（captured-shaft，需要 element-scoped allow_overlap）。
- **外罩→core_spool**（仅双轴）：第二条独立 CONTINUOUS，axis 同 X，origin 多为零点（参考 S14 `casing_to_rotor` origin=Origin()、S15 `nacelle_to_core_spool`、S1 `core_spin`）；与主转子同轴但独立转速/effort。
- **外罩→nozzle_ring→petal_i**：nozzle_ring FIXED 到外罩后端；每 petal 一条 REVOLUTE，hinge 在半径 R 的后环上，`Origin(xyz=(R·cosθ, R·sinθ, hinge_x), rpy=(θ,0,0))` 把局部铰轴绕到该 petal 切向（参考 S18 L389-L409、S8 L243-L255）。
- **外罩→access_door**：单条 REVOLUTE，belly 为下铰线（轴 Y），side bay 为侧铰线（轴 Z）；外罩侧需有可见 hinge 五金（knuckle/bracket/pin）作为支撑路径（参考 S7 L174-L194、S18 bay_housing L133-L186）。

## 每槽位 Module Emits / Interfaces

### Slot A / mount_platform
| emits | 描述 | 来源 |
|---|---|---|
| parts | `bare_inline`: 无（外罩即根）; `ground_display_stand`: `stand`/`stand_base`(+`display_support`); `top_pylon_mount`: `pylon_mount` | S5 / S19,S3,S16,S6 / S10,S15 |
| internal joints | stand 两级时 `base_to_support` FIXED；其余无内部关节 | S19 L351-L364 |
| upstream interface | 接地面（base plinth 底面 / pylon 顶）——作为整模型根 | S19 L186-L202 |
| downstream interface | saddle/mount_pad 贴合外罩外表面，FIXED 托举到中轴高度 | S19 L358-L364 |

### Slot B / engine_core
| emits | 描述 | 来源 |
|---|---|---|
| parts | `outer_shell`（静态短舱/casing/cutaway 壳，含进气唇、静子、轴承 collar、核心机外壳等静态 visual）+ `rotor_primary`（fan 或 compressor）+（仅双轴）`core_spool` | S2 L266-L508 / S6 L159-L344 / S14 L173-L375 |
| internal joints | `rotor_spin` CONTINUOUS axis X（必有）；`core_spin` CONTINUOUS axis X（仅 cutaway_twospool） | S2 L500-L508 / S14 L366-L375, S15 L392-L411 |
| upstream interface | outer_shell 外表面接收平台 FIXED；bare 时 outer_shell 为根 | S5 L153-L208 |
| downstream interface | 后端环面供 aft_nozzle 装配；外罩面供 access_door hinge 五金 | S18 L83-L130 |

### Slot C / aft_nozzle
| emits | 描述 | 来源 |
|---|---|---|
| parts | static: 0-1（`rear_exhaust` 或并入外罩）; variable: `nozzle_ring` + `nozzle_petal_0..N-1` | S19 L289-L312 / S18 L371-L388 |
| internal joints | static: 无（FIXED/并入）; variable: N×`petal_i_hinge` REVOLUTE（可 Mimic 联动） | S18 L389-L409 / S8 L243-L255 |
| upstream interface | nozzle_ring/cone FIXED 接外罩后端中轴环 | S8 L210-L216 |
| downstream interface | petal 自由端张开/收敛（末端，无下游） | S18 L246-L272 |

### Slot D / access_door
| emits | 描述 | 来源 |
|---|---|---|
| parts | `none`: 无; `access_panel`（belly）/`bay_door`（side），各含 panel/leaf/stiffener/latch/hinge 几个 visual | S7 L230-L254 / S18 L234-L243 |
| internal joints | 单条 REVOLUTE（belly 轴 Y / side 轴 Z），bounded lower/upper | S7 L256-L264 / S18 L356-L369 |
| upstream interface | 外罩侧固定 hinge 五金（knuckle/bracket/pin）作为可见支撑 | S7 L174-L194 |
| downstream interface | 门自由端张开（末端，无下游） | S7 L230-L254 |

## 参数范围汇总
| 参数 | 类型 | 取值范围 / 候选值 | 默认 | 派生关系 | 来源 |
|---|---|---|---|---|---|
| `mount_platform` | enum | `bare_inline` / `ground_display_stand` / `top_pylon_mount` | sampled | deterministic procedural sampler | Slot A table |
| `engine_core` | enum | `smooth_turbofan` / `turbojet_module` / `cutaway_twospool` | sampled | deterministic procedural sampler | Slot B table |
| `aft_nozzle` | enum | `static_exhaust_cone` / `variable_petal_nozzle` / `static_chevron_ring` | sampled | 受 compatibility gate（见审计） | Slot C table |
| `access_door` | enum | `none` / `belly_fold_down_panel` / `side_equipment_bay_door` | sampled | 受 compatibility gate（turbojet 一般 `none`） | Slot D table |
| `petal_count` | int | `6..18` | 12 | multiplicity M1（joint-bearing，N×REVOLUTE）；仅 `variable_petal_nozzle`；reviewer-gated 外推 | 实测 S8/S18=8，外推到真实收敛-扩散喷口 12–20 |
| `petal_gang_mode` | enum | `mimic` / `independent` | `independent` | petal hinge 是否联动到 petal_0 | S8 (mimic) / S18 (independent) |
| `fan_blade_count` | int | `12..28` | 20 | multiplicity M2（jointless，转子盘内叶片复制）；reviewer-gated 外推 | 实测 12–24（0003/ccaecd/…/efe964），外推到真实风扇 18–28 |
| `compressor_stage_count` | int | `3..8` | 5 | multiplicity M3（jointless，仅 `cutaway_twospool` 的 core_spool 内复制）；reviewer-gated 外推 | 实测 S1/S15=4、S14=5，外推到展示尺度短 core 上限 8 |
| `turbine_stage_count` | int | `2..5` | 2 | multiplicity M4（jointless，仅 `cutaway_twospool` 的 core_spool 内复制）；reviewer-gated 外推 | 实测 S1/S14=2、S15=3，外推到展示尺度上限 5 |
| `engine_length_scale` | float | `[0.85, 1.20]` | 1.0 | 受控局部 scale，clamp in `resolve_config` | 全样本比例域 |
| `nacelle_diameter_scale` | float | `[0.85, 1.20]` | 1.0 | 受控局部 scale，与转子/喷口半径联动 clamp | 全样本比例域 |
| `stand_height_scale` | float | `[0.80, 1.25]` | 1.0 | 仅 stand/pylon；clamp 使外罩到中轴高度合理 | S19/S16/S6 |
| `palette_theme` | enum | service / military / carrier / test-cell 等 | sampled | 仅配色，不改拓扑 | 多样本材质 |

> 轴向约定：模板统一以 **世界 X** 为发动机中轴（几何沿局部 Z 构建再 `rotate_y(π/2)` 或在 `Origin(rpy=(0,π/2,0))` 中旋转）。所有 CONTINUOUS 转子 `axis=(1,0,0)`。这是 20 个样本里的多数约定；少数 Z-轴样本（4f9be8/ad0120/e07fdb/efe964）在改编时把几何与关节轴一并旋转到 X。

## Multiplicity / Copy Logic

模板有 4 处由 count 参数驱动的同构复制逻辑。它们都是 multiplicity；区别只在 `joint policy`：

- **joint-bearing 复制**（每份新增一条关节）：`petal_count`。
- **jointless 复制**（每份只是某个旋转 part 内部的 visual，共用该 part 的单条 spin 关节，不新增关节）：`fan_blade_count`、`compressor_stage_count`、`turbine_stage_count`。jointless 仍属 multiplicity——它改变重复 visual 的数量，N 进入拓扑多样性采样统计（与范例 `single_rotor_helicopter` 的 `main_blade_count`/`tail_blade_count` 写法一致）。

> **范围策略（reviewer-gated extrapolation）**：M1-M4 的 `N_range` 上限超出 20 个五星样本的实测值，按真实发动机物理 + 展示尺度几何拥挤度外推（实测值偏小，因样本是简化展示模型）。这与范例 helicopter 把 `main_blade_count` 实测 5 外推到 `3..8` 同理。所有 N 在 `resolve_config` clamp；**真正的上限以 sweep-pipeline 为准**：模板实现的 sweep 闭环若发现某高 N 档稳定触发 collision / island 失败，再 clamp 回来并记录原因，spec 阶段不预先收死。

### M1 `petal_count`（joint-bearing）
- `count_param`: `petal_count`
- `N_range`: `6..18`（默认 12；实测 S8/S18 都用 8，上限按真实收敛-扩散喷口 12–20 片外推；seed 0 不特殊）
- sampling domain: 仅当 `aft_nozzle == variable_petal_nozzle` 时采样整数 N；其他尾段无此参数
- copied object: 一个 `nozzle_petal` part（petal_skin + hinge_barrel）+ 其 `petal_i_hinge` REVOLUTE 关节
- naming: part `nozzle_petal_{i:02d}`，joint `petal_{i:02d}_hinge`（i=0..N-1）
- placement: 绕后部 `nozzle_ring` 半径 R 均布，第 i 个相位 `θ_i = i·2π/N`，`Origin(xyz=(R·cosθ_i, R·sinθ_i, hinge_x), rpy=(θ_i,0,0))`；每个 petal 根部必须搭到 nozzle_ring 上（可见支撑）
- joint policy: **每 petal 一条 bounded REVOLUTE**（局部切向轴，参考 lower≈-20°/0、upper≈+22°/0.48）；`petal_gang_mode=mimic` 时 petal_1..N-1 用 `Mimic(joint="petal_00_hinge")` 联动到主 hinge（S8），`independent` 时各自独立（S18）
- source/gating: S8 L243-L255（Mimic 阵列）, S18 L371-L409（独立阵列）

### M2 `fan_blade_count`（jointless）
- `count_param`: `fan_blade_count`
- `N_range`: `12..28`（默认 20；实测 0003=12 / ccaecd·ec6fd5=14 / 6d9ed3·e07fdb·e2123c=16 / ec62c1e0·0001·5abca3=18 / efe964=24，上限按真实风扇 18–28 外推）
- sampling domain: 所有本体都有风扇/压气机转子盘，恒采样
- copied object: `rotor_primary` part（`fan_rotor` 或 `compressor_rotor`）**内部**的一片叶片几何
- naming: 内部 visual `fan_blade_i` / `rotor_blade_i`（i=0..N-1）；不暴露为独立 part
- placement: 绕 hub 径向均布，相位 `θ_i = i·2π/N`，每片根部搭到 hub/spinner
- joint policy: **无 per-blade 关节**；整盘共用 `rotor_primary` 的那条 `rotor_spin` CONTINUOUS
- source/gating: 全 turbofan/turbojet 样本（FanRotorGeometry blade_count 或手搓 `_radial_pattern`）

### M3 `compressor_stage_count`（jointless，仅 `cutaway_twospool`）
- `count_param`: `compressor_stage_count`
- `N_range`: `3..8`（默认 5；实测 S1 ec62c1e0=4 / S15 ad0120=4 / S14 a602a9=5，上限按展示尺度短 core 可容纳级数外推）
- sampling domain: 仅当 `engine_core == cutaway_twospool` 时采样
- copied object: `core_spool` part **内部**的一个压气机级盘（带该级叶片环）
- naming: 内部 visual `compressor_stage_i`（i=0..N-1），沿轴向递进
- placement: 沿 core 轴等距站位，每级盘半径沿轴渐变；全部 merge 进 `core_spool`
- joint policy: **无 per-stage 关节**；所有级共用 `core_spin` CONTINUOUS（参考 S1 L382-L389 把 4 级 merge 进 `core_rotor`）
- source/gating: S1 L382-L389 / S14 L322-L346 / S15

### M4 `turbine_stage_count`（jointless，仅 `cutaway_twospool`）
- `count_param`: `turbine_stage_count`
- `N_range`: `2..5`（默认 2；实测 S1 ec62c1e0=2 / S14 a602a9=2 / S15 ad0120=3，上限按展示尺度尾段可容纳级数外推）
- sampling domain: 仅当 `engine_core == cutaway_twospool` 时采样
- copied object: `core_spool` part **内部**的一个涡轮级盘（带该级叶片环）
- naming: 内部 visual `turbine_stage_i`（i=0..N-1），位于压气机级之后
- placement: 沿 core 轴尾段等距站位；全部 merge 进 `core_spool`
- joint policy: **无 per-stage 关节**；与压气机级同享 `core_spin` CONTINUOUS（参考 S1 L392-L397）
- source/gating: S1 L392-L397 / S14 L348-L364 / S15

> 注意：转子/spool 的**数量**（1 vs 2）不是 multiplicity，而是 `engine_core` slot 选择（`cutaway_twospool` 才有第二条 `core_spin`），上限 2，不暴露为 `*_count`。M2-M4 改的是某个旋转 part **内部**画几个盘/几片叶，不改关节拓扑。

## 拓扑多样性审计

总组合数（标称）：slot 组合 `mount_platform(3) × engine_core(3) × aft_nozzle(3) × access_door(3) = 81`，再乘 multiplicity 采样数：M1 `petal_count∈{6..18}`（仅 variable 分支，+N×REVOLUTE，改 joint 拓扑）、M2 `fan_blade_count∈{12..28}`、M3 `compressor_stage_count∈{3..8}`、M4 `turbine_stage_count∈{2..5}`（仅 cutaway 分支，jointless，改重复 visual 数）。扣除 compatibility gate 后的合法 slot 组合约 50-70 个。

按 **joint 拓扑等价类**计（multiplicity 中只有 M1 改 joint 数，M2-M4 jointless 不改 joint 拓扑）：core 3（fan 1cont / compressor 1cont / dual 2cont）× 尾段 {static, petals-N×(N=6..18 共 13 档)} = 14 × door 3 × platform 3 ≈ 上百。1000-seed slot choice tuple distinct 富类别建议 ≥300（report-only） 可达（M2-M4 进一步增加 distinct 几何配置，但若只按 joint 拓扑去重不计入）。


seed_domain_policy：`procedural_first`。`config_from_seed(seed)` 对所有普通 seed 用 deterministic procedural sampling；`seed=0` 不特殊、不作 anchor。采样顺序：先 mount_platform → engine_core → aft_nozzle（受 core gate）→ access_door（受 core gate）→ petal_count/gang_mode（仅 variable）→ 受控局部 scale。所有非法组合在 sampler 或 `resolve_config` 中降级/重采样/拒绝，不让 builder 后期失败。

Topology target：1000-seed slot choice tuple distinct 富类别建议 ≥300（report-only）；若低于 300，是 compatibility gate（turbojet 排除 chevron/door）收窄所致，需在实现时记录。

若使用 regression overrides：默认无。后续 sweep 若发现稳定失败组合或 reviewer 指定回归样本，可加少量显式 regression seed 并写明原因；不得用小型 curated/modulo 表作为主 seed domain。

Controlled local parameterization：初版应包含 `engine_length_scale`、`nacelle_diameter_scale`、`stand_height_scale`、`petal_count`、`fan_blade_count` 等关键 scale；全部在 `resolve_config` 中 clamp / 派生（如 nozzle_ring 半径、转子半径随 nacelle_diameter_scale 联动，stand saddle 高度随 stand_height_scale 抬到中轴）。这些只改安全比例/clearance/细节尺寸与复制数，不改未声明拓扑、不绕过 compatibility matrix、不破坏 captured-shaft / hinge 接口。

| item | policy | validator / viewer focus |
|---|---|---|
| sampler | slot 顺序 A→B→C→D→multiplicity→scale；weighted choice + compatibility gate | `slot_choices_for_seed` 与实际 build choices 一致 |
| compatibility matrix | `turbojet_module` 不配 `static_chevron_ring`、一般不配 door（裸涡喷无短舱腹部），优先 `none`；`variable_petal_nozzle` 与 `static_chevron_ring` 互斥（都是尾段，只能选一，本就同槽）；`cutaway_twospool`/`smooth_turbofan` 兼容全部尾段与 door | 无 floating、无穿模、轴/range 正确、闭合姿态、max petal multiplicity、bulky cutaway、optional door 失败 |
| controlled local variation | safe continuous scale（length/diameter/stand height）+ clamp | 比例变化不破坏接口/clearance/支撑/joint origin/类别身份 |
| regression overrides | none（初版） | 仅限已知失败回归或 reviewer 指定 |
| random sweep | 初轮 seeds 0-49；成熟审计 0-999 | 与 MatingContract 失败 |

| slot | candidate_count | 是否 ≥2 | 是否 ≥3 | 备注 |
|---|---:|---|---|---|
| mount_platform | 3 | yes | yes | bare / stand / pylon 都有多源 |
| engine_core | 3 | yes | yes | turbofan 源众多；turbojet 2 源；cutaway 3 源 |
| aft_nozzle | 3 | yes | yes | static cone（多源）/ variable petals（2 源）/ chevron（1 源） |
| access_door | 3 | yes | yes | 含 `none`；belly 2 源、side bay 1 源 |

## Validator（`run_jet_engine_tests` 必须覆盖的点）

- `slot_choices_for_seed` 返回已实现的 module 名称，且与实际 build choices 一致。
- `config_from_seed` 对所有普通 seed 用 deterministic procedural sampling；seed=0 不特殊。
- compatibility matrix / gating 阻止非法组合（turbojet+chevron、turbojet+door、双尾段）。
- **必有且恰有正确数量的 CONTINUOUS 转子主轴**：单转子本体 1 条、`cutaway_twospool` 2 条；axis 都为 `(1,0,0)`；类型为 CONTINUOUS（无 lower/upper）。这是类别身份的硬检查（旧模板正是在此失败）。
- 转子 hub/shaft 被外罩轴承 collar/socket 捕获并保持同轴；spin 到非零 pose 后仍居中（`expect_origin_distance` yz 近 0，参考 S6/S19）。
- `variable_petal_nozzle`：恰好 N=`petal_count` 个 petal part 与 N 条 REVOLUTE，均布相位正确，张开方向向外（posed AABB 检查），petal 根搭在 nozzle_ring 上；`mimic` 模式下从动 hinge 正确联动。
- `access_door`：door REVOLUTE 轴/方向正确（belly 下折、side 侧开），hinge 五金可见且门搭在铰线上。
- `mount_platform` 非 bare 时：支架/pylon 为独立 part，FIXED 托住外罩到中轴高度，无 floating；接地面成立。
- captured-shaft / hinge-pin overlap 用 element-scoped `allow_overlap` + reason 声明，并配 `expect_within`/`expect_overlap`/`expect_contact` 证明啮合；不得用 blanket `allow_overlap` 掩盖（参考 efe964 的反面：blanket allow + 弱 QC）。
- 默认 QC 栈：`fail_if_isolated_parts`、`warn_if_part_contains_disconnected_geometry_islands`、`fail_if_parts_overlap_in_current_pose`、`fail_if_articulation_overlaps`（多 pose 采样）。

## Reject cases（必须能识别并拒绝）

- 零可动关节 / 全 FIXED 链（旧模板的核心错误）——发动机必须有转子 CONTINUOUS 主轴。
- 转子做成 REVOLUTE bounded 或加了 lower/upper（应为 CONTINUOUS 自转）。
- 转子轴方向错误（不沿发动机纵轴）、或多个转子不同轴。
- 转子盘脱离外罩中轴 / 悬空 / 未被轴承捕获，spin 后平移漂出。
- `variable_petal_nozzle` 的 petal 悬空、不搭 nozzle_ring、或张开方向朝内。
- 检修门悬在外罩之外、铰链五金缺失、轴向错误。
- 用 blanket `allow_overlap` 掩盖精度穿模而非 element-scoped 声明。
- 把裸风扇/吊扇/风力机当 jet engine（无短舱-核心-排气身份）。
- turbojet 配 chevron 高涵道尾段或同时出现两种尾段。
- stand/pylon 融进同一 mesh 却声称是接地结构却无支撑路径（参考 efe964 反面）。

## 与相邻类别的边界

- `ceiling_fan` / `axial_fan_rotor` / `box_fan`：裸风扇，无短舱/核心机/排气段；jet engine 转子必须封装在外罩内并有进气-核心-排气轴向叙事。
- `wind_turbine` / `traditional_windmill`：塔架少叶大转子，无短舱/核心机。
- `single_rotor_helicopter` / `drone`：有机身/尾梁/起落架；其转子轴是垂直升力轴，jet engine 是纵向中轴。
- `rocket_engine` / `cannon`：纯静态喷管/身管，无旋转转子。

## 模板实现备注（可选）

- 共享 helper：`LatheGeometry.from_shell_profiles`（中空外罩 + 进气唇 round cap + lip_samples）是 turbofan/turbojet 外罩主力写法；`section_loft`+`repair_loft` + 4-点 `_blade_section` 翼型是叶片/静子/petal 主力写法；SDK 原生 `FanRotorGeometry`/`FanRotorBlade`(scimitar)/`FanRotorHub` 是更紧凑的风扇写法（S4/S5/S10/S15/S18/efe964 用过），可作为 `front_fan_rotor` 的备选实现。`cutaway` 壳体用 partial-θ 角度 lathe / 手搓 `add_vertex`+`add_face` C 壳（S1/S14）或 open cage longeron（S15）。
- 统一轴向到 world X；改编 Z-轴源（4f9be8/ad0120/e07fdb/efe964）时把几何与 `axis` 一并旋转。
- captured-shaft（转子 hub 略大于轴承孔）与 hinge-pin-in-barrel 需要 element-scoped `allow_overlap`（参考 S4 L265-L271、S7 allow_overlap+expect_within、S3 用纯 expect_* 无 allow 的更干净写法）。
- `cutaway_twospool` 的开切壳体是有意暴露内部，不是缺陷；但要确保切口不破坏 island/支撑检查（参考 S1/S14/S15）。
- 高质量 modular 参考（实现阶段从 MATURE_TEMPLATE_METHOD.md reference map 深读 1-3 个）：优先 `single_rotor_helicopter`（CONTINUOUS 转子 + 可选门）、`traditional_windmill`/`overshot_waterwheel`（连续旋转主轴 + radial 复制叶片）、以及带 multiplicity REVOLUTE 阵列的模板（用于 petal 阵列与 Mimic 联动）。

## Module Source Index

| source_id | sample_id | model.py 来源 | 采纳用途 |
|---|---|---|---|
| S1 | `rec_jet_engine_ec62c1e0`（`rec_an-extremely-realistic-model-of-a-jet-engine-wit_20260318_232012_ec62c1e0`） | `…/rev_000001/model.py:L211-L516` | cutaway 双轴（fan_spin + core_spin 两条独立 CONTINUOUS），C 壳暴露内部 |
| S2 | `rec_jet_engine_0001` | `…/model.py:L266-L508` | smooth_turbofan 封闭 nacelle + 单前风扇 1×CONTINUOUS + 融入 pylon |
| S3 | `rec_jet_engine_0003` | `…/model.py:L205-L325` | turbofan + 独立 display stand part + FIXED（最干净的支架/外罩/转子三段分离）；静态 exhaust_cone |
| S4 | `rec_jet_engine_202515e1...` | `…/model.py:L39-L254` | turbofan + 融入 stand；`FanRotorGeometry` scimitar 风扇写法 |
| S5 | `rec_jet_engine_268a98e6...` | `…/model.py:L153-L249` | bare_inline 区域涡扇（nacelle 作根）+ 单风扇；CadQuery 短舱 |
| S6 | `rec_jet_engine_278c04c0...` | `…/model.py:L109-L344` | turbojet_module + 维护 cradle（FIXED）+ compressor_rotor，captured-shaft 严谨 QC |
| S7 | `rec_jet_engine_3ad5d74c...` | `…/model.py:L230-L264` | belly_fold_down_panel 检修盖 REVOLUTE（+ fan CONTINUOUS） |
| S8 | `rec_jet_engine_4f9be877...` | `…/model.py:L203-L255` | variable_petal_nozzle 8 petal，`Mimic` 联动到 petal_0 |
| S9 | `rec_jet_engine_596a0163...` | `…/model.py:L80-L204` | geared turbofan + display pylon，单风扇 |
| S10 | `rec_jet_engine_5abca3da...` | `…/model.py:L213-L275` | high-bypass turbofan + top_pylon_mount 独立 part FIXED |
| S11 | `rec_jet_engine_5ac6b19c...` | `…/model.py:L82-L235` | high-bypass turbofan，pylon 融入 nacelle 变体 |
| S12 | `rec_jet_engine_5e2bb735...` | `…/model.py:L70-L248` | turbojet_module，casing→support→rotor 两级链（Z 轴源，需旋转到 X） |
| S13 | `rec_jet_engine_6d9ed3b3...` | `…/model.py:L307-L367` | belly_fold_down_panel 检修盖第二源 |
| S14 | `rec_jet_engine_a602a9de...` | `…/model.py:L173-L375` | cutaway_twospool（fan + 5 压气机 + 2 涡轮级 spool）1 条整体 CONTINUOUS |
| S15 | `rec_jet_engine_ad0120c0...` | `…/model.py:L86-L411` | turbofan fan + core_spool 两条 CONTINUOUS + static_chevron_ring + open cage + pylon |
| S16 | `rec_jet_engine_ccaecd76...` | `…/model.py:L150-L269` | geared turbofan + display_stand（root）→nacelle→fan 三段链 |
| S17 | `rec_jet_engine_e07fdb1f...` | `…/model.py:L82-L322` | 区域涡扇单风扇（Z 轴源，需旋转到 X） |
| S18 | `rec_jet_engine_e2123c10...` | `…/model.py:L234-L409` | 军用涡扇 + variable_petal_nozzle 8 petal（独立）+ side_equipment_bay_door REVOLUTE |
| S19 | `rec_jet_engine_ec6fd583...` | `…/model.py:L186-L380` | turbofan + 多 part display stand 链 FIXED + 独立 rear_exhaust（static cone）；最强 QC harness |
| S20 | `rec_jet_engine_efe9644b...` | `…/model.py:L23-L102` | turbofan + 融入 stand；CadQuery 短舱 + `FanRotorGeometry`（反面参考：blanket allow_overlap + 弱 QC，不要照抄 QC） |

## 审核记录
| 项 | 结论 |
|---|---|
| reviewer status | approved |
| reviewer notes | 用户经多轮迭代审核（修正 multiplicity 定义=含 jointless 复制；range 外推 fan_blade 12-28 / petal 6-18 / compressor_stage 3-8 / turbine_stage 2-5，reviewer-gated，sweep 再 clamp），并明确指示进入第二阶段。stator/strut 根数暂保持写死（不开成 multiplicity）。核心要求 = 强制转子 CONTINUOUS 主轴。 |

## 实现记录（TEMPLATE_AFTER_REVIEW 完成）
| 项 | 结论 |
|---|---|
| status | implemented，`sweep-pipeline verdict=pass`（0-49 全过，pass_rate=1.0；=pass`，0-49 distinct=34，0-999 distinct=199成熟目标） |
| 坐标约定 | 统一 world X 中轴；`outer_shell` 轴向居中（x∈[-EL/2,EL/2]），中轴高度由 `axis_z` 决定（bare=0 / stand=+R 腹部为 local 原点 / pylon=-R 顶冠为 local 原点），使 FIXED 挂载关节两侧都落在真实几何上 |
| 叶片 primitive | 风扇/压气机/各级盘统一用 `section_loft`(4 点翼型)×`_radial_pattern` 纯 python 叶排（spec 认可写法），**未用 cadquery `FanRotorGeometry`**——后者每 seed 多达 13 个 boolean union 触发 60s compile 超时；section_loft 既保真又快（worst cutaway 单 seed≈13s） |
| compatibility gate（实现版，需登记） | turbojet→无 chevron、door 强制 `none`；stand+belly→改 side（saddle 占腹部）；**cutaway→尾段强制 `static_exhaust_cone`**：开放笼/暴露核心机收尾于轴上 plug，chevron/petal 需要封闭尾罩故保留给 turbofan/turbojet。此 gate 收窄了 cutaway 的尾段轴（spec 原列其兼容全部尾段，含 S15=cutaway+chevron），为鲁棒性与机构连贯性做的实现决定 |
| 视觉目检 | 已按根协议 preview 目检：进气口开放、风扇/压气机面正面可见（轴向 head-on 视角），cutaway 暴露双轴核心，variable petal 在尾部张开，stand/pylon/bare 姿态连贯；转子 CONTINUOUS 自转 + petal/door REVOLUTE 经 posed-AABB 断言验证方向正确。注意：闭合涵道 turbofan 在斜 3/4 视角呈短舱筒身（符合真实短舱外观），运动件（风扇）从进气口正视可见，满足"运动件必须可见"要求 |
| QC 实现要点 | 全部 captured-shaft / hinge-pin / 腹部门 overlap 用 element-scoped `allow_overlap`（按真实 visual 名遍历），无 blanket allow；rotor 居中自转、petal 外张、door 开向用 `ctx.pose` + AABB 断言；无 disconnected island（无 `allow_disconnected_islands` 逃生口可用，全部 part visual 互相接触保证单连通） |
