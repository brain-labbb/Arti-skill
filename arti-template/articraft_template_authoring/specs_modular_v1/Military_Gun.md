## 元信息

| 项 | 值 |
|---|---|
| slug | `handgun` |
| template path | `agent/templates/Military_Gun.py` |
| test path (optional) | `tests/agent/test_handgun_template.py` |
| stage | `SPEC_ONLY_DRAFT` |
| status | `pending` |
| __modular__ | `True` |
| pattern | `mixed` |

`mixed`：上游 `action` slot 决定一整条 spine（revolver swing-out cylinder vs semi-auto reciprocating slide），其余 slot 在该 spine 的固定 mating face / pivot 上挂 part；revolver spine 还带一根 multiplicity 轴（cylinder chamber 复制）。

## 5 星样本阅读摘要

| 项 | 值 |
|---|---|
| five_star_total | 10 |
| read_count | 10 |
| read_scope | all 5-star samples in this category（2 parents + 8 variants，rating=5，synced at `data/records/<id>/`） |
| source_index_policy | only adopted module sources are indexed below |

结构族分布：

| 结构族 (action spine) | 样本数 | 说明 |
|---|---:|---|
| revolver swing-out cylinder（crane + 旋转 cylinder + ejector rod + hammer） | 6 | parent + revsnub/revadjsight/revroundbutt/rev5shot/rev8shot；携带 chamber multiplicity |
| semi-auto reciprocating slide（frame rails + 往复 slide + magazine + takedown lever） | 4 | parent + pistlong/pistcompact/pistoptic |

读取的两条 spine 是**完全 disjoint 的 part/joint 集合**：

- revolver：`frame / crane / cylinder / ejector_rod / trigger / hammer / grip`；joints `crane_swing`(REVOLUTE,-X) / `cylinder_spin`(CONTINUOUS,+X) / `ejector_push`(PRISMATIC,-X) / `trigger_pull`(REVOLUTE,+Y) / `hammer_cock`(REVOLUTE,-Y) / `grip_mount`(FIXED)。5 个非固定 DOF。
- semi-auto：`frame / slide / trigger / takedown_lever / magazine`；joints `frame_to_slide`(PRISMATIC,-X) / `frame_to_trigger`(REVOLUTE,+Y) / `frame_to_takedown_lever`(REVOLUTE,+Y) / `frame_to_magazine`(PRISMATIC,沿 raked grip 轴)。4 个非固定 DOF。

关键观测：rev5shot / rev8shot 两个 5 星变体**已经**把 parent 硬编码的 `for k in range(6)` chamber/flute/liner 循环 + `polygon(6, ...)` ejector star 重写成 `CHAMBER_COUNT` 参数（`CHAMBER_ANGLE_STEP = 360.0 / CHAMBER_COUNT` + `_chamber_position(k)` / `_flute_position(k)` helpers），证明 chamber-count 是纯 N 参数轴，refactor 已被 5 星样本验证过。模板必须采纳这套 helper 形式作为 multiplicity 契约。

## 核心身份

handgun（pistol / 手枪）：单手握持、grip-butt 接地、bore 轴沿 +X 指向 muzzle 的小型枪械。两大成熟 action 家族：

1. **revolver**：弹巢（cylinder）多腔旋转 + 摆出装填（crane swing-out），击锤外露（hammer），ejector rod 退壳。
2. **semi-automatic pistol**：套筒（slide）沿 frame 导轨往复，弹匣（magazine）从 grip 底插入，takedown lever 拆解。

默认成熟域：整体长 ~0.21–0.30 m，宽 ~0.034–0.046 m，高 ~0.13–0.16 m；grip butt 或 magazine baseplate 接地于 z≈0；bore 轴 +X。核心身份特征是 grip + trigger(+guard) + barrel/bore + 一条 action spine。

不该混入相邻类别见末节（步枪/冲锋枪/纳枪类、玩具/水枪、枪套配件）。

## 槽位 + 候选模块表

四个 slot：`action`（决定 spine，主轴）、`barrel_length`、`sights`、`grip`。每个 candidate 都来自被采纳的 5 星样本片段，结构（part tree / joint / primitive）互不相同。所有 `Lx-Ly` 为真实 model.py 行号。

### Slot A：action（spine 选择，决定整套 part/joint topology + 是否带 chamber multiplicity）

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| `revolver_swingout` | rec_model-a-classic-double-action-revolver-colt-pyth_20260610_081456_135585_9e7d2f05 | L88-L196, L272-L488 (frame/crane/cylinder/rod build + assembly), L334-L416 (crane_swing/cylinder_spin/ejector_push joints) | eligible if compatible | 5 非固定 DOF；crane(REVOLUTE -X) 摆出，crane 上挂 cylinder(CONTINUOUS +X 自旋) + ejector_rod(PRISMATIC -X)；frame 持 hammer(REVOLUTE -Y)/trigger(REVOLUTE +Y)/pivot pins；crane arbor 过盈插入 cylinder center bore；携带 `CHAMBER_COUNT` 轴 |
| `semi_auto_slide` | rec_model-a-modern-striker-fired-semi-automatic-pist_20260610_081229_238472_7155f244 | L80-L147 (frame/rail/slide build), L171-L331 (assembly), L247-L331 (slide/trigger/lever/magazine joints) | eligible if compatible | 4 非固定 DOF；slide(PRISMATIC -X) 在 frame 导轨往复，bore 在 slide 内 hollow；frame 持 trigger(REVOLUTE +Y) + takedown_lever(REVOLUTE +Y) + magazine(PRISMATIC 沿 raked grip 轴)；open-loop trigger guard；无 multiplicity 轴 |

> **single-pair slot 降级说明**：`action` slot 只有 2 个 candidate，低于 3-6 目标。理由：handgun 类目真实成熟 action 只有这两个 disjoint spine 家族（5 星样本无第三族，cross-family hybrid 出类目，见排除项）。本 slot 是模板的 spine 选择器；它的 2 个值各自驱动 Slot B/C/D 的不同子候选池，整体拓扑多样性由四 slot 的兼容组合（见审计）撑起，不靠单一 slot 凑数。保留 2 candidate 合法（SPEC_TEMPLATE §4 允许样本池不足时降到 2 并说明理由）。

### Slot B：barrel_length（沿 +X 的 muzzle/bore 出口长度；候选按 action spine 分两池）

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| `revolver_mid` | rec_model-a-classic-double-action-revolver-colt-pyth_..._9e7d2f05 | L31 (MUZZLE_X=0.126), L53 (LUG_X1=0.122), L109-L126 (`_barrel_solid`) | eligible if action=revolver_swingout | 6 in 满长 vented-rib barrel + 全长 underlug 包裹 ejector rod；4 个 vent slot 循环 (L118-120) |
| `revolver_snub` | rec_handgun_var_revsnub | L31 (MUZZLE_X=0.025), L53 (LUG_X1=0.020), L109-L124 (`_barrel_solid`) | eligible if action=revolver_swingout | 2 in snub-nose：短 barrel + 缩短 underlug + 单 vent slot；front sight 移到 snub muzzle |
| `pistol_mid` | rec_model-a-modern-striker-fired-semi-automatic-pist_..._7155f244 | L45 (SLIDE_X1=0.105，长 0.21 m), L128-L147 (`_build_slide_solid`), L214-L220 (barrel_block) | eligible if action=semi_auto_slide | 标准 slide 长度，barrel_block 透过 ejection port 可见 |
| `pistol_long` | rec_handgun_var_pistlong | L51 (SLIDE_X1=0.130，长 0.235 m), L79-L80 (SLIDE_MID_X/SLIDE_LENGTH), L147-L175 (`_build_slide_solid` 用 SLIDE_MID_X 重定位特征) | eligible if action=semi_auto_slide | 5 in 长 slide：bore 出口前推 (bore_world_x=SLIDE_X1-0.015)，所有 slide 特征以 SLIDE_MID_X 派生 |

### Slot C：sights（rear sight 结构层；候选按 action spine 分两池）

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| `revolver_fixed` | rec_model-a-classic-double-action-revolver-colt-pyth_..._9e7d2f05 | L142-L145 (`_rear_sight_solid`), L306-L312 (rear_sight visual) | eligible if action=revolver_swingout | 低 fixed notch sight 铣在 top strap 上；单 box + notch cut |
| `revolver_adjustable` | rec_handgun_var_revadjsight | L146-L179 (`_rear_sight_solid`：base/bridge/housing union + windage_stem/windage_head + elev_screw) | eligible if action=revolver_swingout | 高 fully-adjustable 组件，blade 顶 > z=0.137；windage 旋钮 (L158-168) + elevation screw (L170-175) |
| `pistol_fixed` | rec_model-a-modern-striker-fired-semi-automatic-pist_..._7155f244 | L222-L233 (rear_sight + front_sight Box) | eligible if action=semi_auto_slide | 低 fixed iron sights 在 slide deck 上，两个 Box |
| `pistol_optic_cut` | rec_handgun_var_pistoptic | L148-L168 (`_build_slide_solid` 铣 optic pocket + 螺孔), L245-L274 (optic_sight_block + optic_lens_window + optic_screw_{i}) | eligible if action=semi_auto_slide | optic-ready 铣槽 top deck：red-dot housing + lens window + 2 mount screw；slide solid 真的被 cut 出 pocket |

### Slot D：grip（grip/backstrap/magwell 几何层；候选按 action spine 分两池）

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| `revolver_square` | rec_model-a-classic-double-action-revolver-colt-pyth_..._9e7d2f05 | L248-L269 (`_grip_solid` polyline，平 butt heel), L458-L486 (grip part + panel/screw + `grip_mount` FIXED) | eligible if action=revolver_swingout | 方形 target-style butt，平 heel，walnut 木 panel + screw；separate `grip` part 经 FIXED `grip_mount` |
| `revolver_roundbutt` | rec_handgun_var_revroundbutt | L248-L275 (`_grip_solid` 用 spline outline，圆 heel，单点 kiss z=0) | eligible if action=revolver_swingout | 圆 smooth-heel butt（spline，无平 butt 面），更小更弯 |
| `pistol_straight` | rec_model-a-modern-striker-fired-semi-automatic-pist_..._7155f244 | L54-L57 (BUTT_F/BUTT_G/BUTT_CX/BUTT_CZ), L80-L116 (`_build_frame_solid` grip outline + magwell), L308-L331 (magazine + `frame_to_magazine` drop 0.10) | eligible if action=semi_auto_slide | 全尺寸 raked polymer grip，标准 magwell 深度，mag drop 0.10 m |
| `pistol_compact` | rec_handgun_var_pistcompact | L58-L59 (BUTT_F=(-0.046,0.0)/BUTT_G=(-0.108,0.021)), L65 (MAG_TRAVEL=0.07), L85-L123 (`_build_frame_solid` 缩短 grip outline) | eligible if action=semi_auto_slide | grip 沿 rake 缩短 ~25 mm，更短 flush magazine（容量折进 grip 长度），mag drop 0.07 m |

硬约束满足检查：除 `action`（2，已说明降级）外，B/C/D 每 slot 4 candidates（每 spine 池 2，共 4）。所有 candidate 结构差异真实（part tree / joint / primitive 不同），非仅尺寸/颜色/装饰。

## 槽位图（slot graph）

pattern: `mixed`

```
                 [Slot A: action  ← spine selector, 决定 frame part tree + 整套 joint topology]
                          │
        ┌─────────────────┼─────────────────────────────┬───────────────────────────┐
        │ FIXED/union 到   │ FIXED/union 到               │ PRISMATIC/REVOLUTE child   │
        │ frame muzzle 端  │ frame/slide top deck         │ 经 spine 内部 joint         │
        ▼                  ▼                              ▼                            ▼
 [Slot B: barrel_length]  [Slot C: sights]        (spine-internal moving parts)   [Slot D: grip]
   muzzle/bore +X 出口      rear sight on top strap/slide  cylinder/slide/trigger/    grip + magwell
                                                          hammer/magazine/lever       backstrap
```

跨 slot 连接与接口点位：

- **A→B（barrel）**：
  - revolver spine：barrel 是 `frame` part 上的一个 visual（`_barrel_solid` union 进 frame），与 frame front face 在 `FRAME_FRONT=-0.026` 处共体；接口 = barrel root 的 underlug 座 + forcing-cone tuck（barrel 不是独立 part，无 joint，FIXED union 到 frame）。`MUZZLE_X` / `LUG_X1` 是该接口的可变参数。
  - semi-auto spine：barrel length = `slide` part 的 `_build_slide_solid` 长度（`SLIDE_X1`）；slide 是 PRISMATIC child，barrel 出口随 slide 长度前推。接口 = slide box 前端 + muzzle bore。
- **A→C（sights）**：
  - revolver：rear sight 是 `frame` 上 visual，座在 top strap z≈0.126；接口 = top-strap dovetail 平面。fixed=低 box，adjustable=高 union 组件（blade tip 抬到 >z=0.137）。
  - semi-auto：sights 在 `slide` part 顶面 deck；optic_cut 在 `_build_slide_solid` 里真切出 pocket（接口 = milled pocket floor + screw holes），fixed=两个 Box deck sight。
- **A→D（grip）**：
  - revolver：`grip` 是独立 part，经 `grip_mount`(FIXED, origin=Origin()) 接到 frame 后下方 tang；接口 = grip 顶 frame-tang 平面（grip 与 frame 在 L626 `expect_contact`）。square=polyline 平 butt，roundbutt=spline 圆 butt。
  - semi-auto：grip 是 `frame` part 自身 outline 的一部分（`BUTT_F/BUTT_G` 角点 + magwell cut），不是独立 part；`magazine` 经 `frame_to_magazine`(PRISMATIC，沿 raked grip 轴) 从 butt 面插入。接口 = magwell 内壁 + butt center mag joint frame。straight/compact 改 grip outline 角点 + mag travel。

互斥/派生规则：

- **revolver_* 与 pistol_* 候选互斥**：Slot B/C/D 的候选池由 Slot A 决定（revolver action 只能配 revolver_mid/snub、revolver_fixed/adjustable、revolver_square/roundbutt；semi_auto action 只能配 pistol_* 一族）。compatibility matrix 强制此 gate，禁止 cross-family（revolver action × pistol slide / mixed grip）。
- multiplicity 轴 `CHAMBER_COUNT` 仅当 `action=revolver_swingout` 时存在；semi_auto spine 无复制轴。

## 每槽位 Module Emits / Interfaces

### Slot A / module `revolver_swingout`

| emits | 描述 | 来源 |
|---|---|---|
| parts | `frame`（含 barrel/guard/rear_sight/latch/pins visual）, `crane`, `cylinder`, `ejector_rod`, `trigger`, `hammer`；`grip` 由 Slot D 提供 | S1 / model.py:L272-L488 |
| internal joints | `crane_swing`(REVOLUTE, axis -X, 0..0.785), `cylinder_spin`(CONTINUOUS, axis +X, 无限), `ejector_push`(PRISMATIC, axis -X, 0..0.020), `trigger_pull`(REVOLUTE, axis +Y, 0..0.44), `hammer_cock`(REVOLUTE, axis -Y, 0..0.52) | S1 / model.py:L343-L456 |
| upstream interface | 此 module 是 root（frame 是 model root）；无 upstream | S1 / model.py:L284 |
| downstream interface | frame front face (FRAME_FRONT=-0.026) → barrel union (Slot B)；top strap z≈0.126 → rear sight (Slot C)；frame rear tang → grip FIXED mount (Slot D)；crane center bore → cylinder multiplicity host | S1 / model.py:L88-L106, L142, L480-L486 |

### Slot A / module `semi_auto_slide`

| emits | 描述 | 来源 |
|---|---|---|
| parts | `frame`（含 accessory_rail/grip panels/slide_stop visual + grip outline + magwell）, `slide`（含 barrel_block/sights/serrations）, `trigger`, `takedown_lever`, `magazine` | S2 / model.py:L171-L331 |
| internal joints | `frame_to_slide`(PRISMATIC, axis -X, 0..0.045), `frame_to_trigger`(REVOLUTE, axis +Y, 0..25°), `frame_to_takedown_lever`(REVOLUTE, axis +Y, 0..90°), `frame_to_magazine`(PRISMATIC, axis -Z 沿 RAKE, 0..0.10) | S2 / model.py:L247-L331 |
| upstream interface | root（frame 是 model root）；无 upstream | S2 / model.py:L184 |
| downstream interface | slide box（barrel 长度 = Slot B）；slide top deck (sights/optic pocket = Slot C)；frame grip outline BUTT_F/BUTT_G + magwell (grip = Slot D) | S2 / model.py:L45, L54-L57, L80-L116 |

### Slot B / module `revolver_mid` & `revolver_snub`

| emits | 描述 | 来源 |
|---|---|---|
| parts | 无新 part：barrel 作为 `frame` 的 visual（`barrel_assembly`）union 进 frame solid | S1 / model.py:L109-L126, L292-L298 |
| internal joints | 无（FIXED union 到 frame） | — |
| upstream interface | frame front (FRAME_FRONT) + underlug 座 + ejector-rod channel（穿 underlug，cut `_xcyl`） | S1 / model.py:L112, L125 |
| downstream interface | muzzle bore at MUZZLE_X；front sight blade 在 muzzle | S1 / model.py:L116, L122 (mid) ; revsnub L116-120 (snub 单 vent + snub front sight) |

### Slot B / module `pistol_mid` & `pistol_long`

| emits | 描述 | 来源 |
|---|---|---|
| parts | 无新 part：barrel 长度 = `slide` part 的 `_build_slide_solid` 范围 + `barrel_block` cylinder visual | S2 / model.py:L128-L147, L214-L220 ; pistlong L147-L175 |
| internal joints | 无（barrel 随 slide 的 `frame_to_slide` PRISMATIC 移动） | — |
| upstream interface | slide rails（与 frame top FRAME_TOP=SLIDE_Z0 共面） | S2 / model.py:L48 |
| downstream interface | muzzle bore（slide 前端 hollow）；pistol_long 把 bore 出口推到 SLIDE_X1-0.015，所有 feature 经 SLIDE_MID_X 重定位 | S4 / model.py:L158, L153-L175 |

### Slot C / module `revolver_fixed` & `revolver_adjustable`

| emits | 描述 | 来源 |
|---|---|---|
| parts | 无新 part：`rear_sight` 作为 `frame` 的 visual | S1 / model.py:L142-L145, L306-L312 ; revadjsight L146-L179 |
| internal joints | 无（FIXED 在 top strap） | — |
| upstream interface | top strap dovetail 平面 z≈0.126 | S1 / model.py:L143 |
| downstream interface | fixed=低 notch；adjustable=高 union（base/bridge/housing + windage knob + elev screw），blade tip >z=0.137 | S5 / model.py:L146-L179 |

### Slot C / module `pistol_fixed` & `pistol_optic_cut`

| emits | 描述 | 来源 |
|---|---|---|
| parts | 无新 part：sights 作为 `slide` 的 visual（fixed=rear+front Box；optic=optic_sight_block + optic_lens_window + optic_screw_{i}） | S2 / model.py:L222-L233 ; pistoptic L245-L274 |
| internal joints | 无（随 slide 往复） | — |
| upstream interface | slide top deck；optic_cut 额外在 `_build_slide_solid` 切 milled pocket + 螺孔 | S6 / model.py:L148-L168 |
| downstream interface | optic lens window / red-dot housing 高出 deck（用于 overall height 检查） | S6 / model.py:L252-L265 |

### Slot D / module `revolver_square` & `revolver_roundbutt`

| emits | 描述 | 来源 |
|---|---|---|
| parts | `grip`（独立 part，square 含 walnut panel + screw；roundbutt 仅 grip_body spline） | S1 / model.py:L458-L486 ; revroundbutt L248-L275 |
| internal joints | `grip_mount`(FIXED, origin=Origin()) → frame | S1 / model.py:L480-L486 |
| upstream interface | frame rear tang 平面（grip 顶接 frame，`expect_contact`） | S1 / model.py:L626 |
| downstream interface | butt heel 接地 z≈0（square=平 butt，roundbutt=单点 kiss） | S1 L255 ; S7 model.py:L259 |

### Slot D / module `pistol_straight` & `pistol_compact`

| emits | 描述 | 来源 |
|---|---|---|
| parts | grip 是 `frame` outline 的一部分（非独立 part）；`magazine`（mag_body + mag_baseplate）是独立 part | S2 / model.py:L80-L116, L308-L321 |
| internal joints | `frame_to_magazine`(PRISMATIC, 沿 raked grip 轴, 0..MAG_TRAVEL) | S2 / model.py:L323-L331 |
| upstream interface | frame grip outline 角点 BUTT_F/BUTT_G + magwell cut + butt center mag joint frame (BUTT_CX/BUTT_CZ, RAKE) | S2 / model.py:L54-L57, L109-L116 |
| downstream interface | magazine baseplate 接地 z≈0；compact 缩短 grip + flush mag（MAG_TRAVEL=0.07 vs straight 0.10） | S8 / model.py:L58-L59, L65 |

## 参数范围汇总

| 参数 | 类型 | 取值范围 / 候选值 | 标称默认 | 约束类型 | 约束 / 函数 | 来源 |
|---|---|---|---|---|---|---|
| `action` | enum | `revolver_swingout`, `semi_auto_slide` | — | choice | 由 deterministic procedural sampler 选择；决定 B/C/D 候选池与是否有 `chamber_count` 轴 | Slot A table |
| `barrel_length` | enum | revolver: `revolver_mid`,`revolver_snub` ; semi-auto: `pistol_mid`,`pistol_long` | — | conditional | 合法值依赖 `action`（compatibility matrix） | Slot B table |
| `sights` | enum | revolver: `revolver_fixed`,`revolver_adjustable` ; semi-auto: `pistol_fixed`,`pistol_optic_cut` | — | conditional | 合法值依赖 `action` | Slot C table |
| `grip` | enum | revolver: `revolver_square`,`revolver_roundbutt` ; semi-auto: `pistol_straight`,`pistol_compact` | — | conditional | 合法值依赖 `action` | Slot D table |
| `chamber_count` | int | `[5, 8]` | 6 | conditional | 仅 `action=revolver_swingout` 时存在；`chamber_angle_step = 360.0 / chamber_count` | S1 L169-182 / rev5shot L40-41 |
| `barrel_len_scale` | float | `[0.85, 1.15]` | 1.0 | independent | 在所选 barrel module 标称 `MUZZLE_X`/`SLIDE_X1` 上独立缩放后 clamp（不跨越邻近 barrel module 的语义档） | S1 L31 / S2 L45 |
| `grip_height_scale` | float | `[0.92, 1.08]` | 1.0 | independent | 缩放 grip/backstrap 高度（revolver `_grip_solid` 纵向；semi-auto BUTT_F/BUTT_G 沿 rake） | S1 L248-269 / S2 L54-57 |
| `mag_travel` | float | derived | 0.10 (straight) / 0.07 (compact) | equation | `= grip_height_scale · base_mag_travel(grip)`；仅 semi-auto；随 grip 模块标称值派生 | S2 L66 / S8 L65 |
| `cyl_radius_scale` | float | derived | 1.0 | equation | `= f(chamber_count)`：chamber 数越多需略大 CYL_R 保持 chamber 间壁厚；不独立采样 | S1 L36, L40-42 |
| (—) | constraint | — | — | inequality | grip butt / mag baseplate 最低点 ≤ z=0.004（接地）；违反时沿 grip 轴回缩 `grip_height_scale` | S1 L568 / S2 L447-451 |
| (—) | constraint | — | — | inequality | revolver：chamber circle clearance `chamber_count · 2·CHAMBER_R < 2π·CHAMBER_CIRCLE_R·cyl_radius_scale`（chamber 不互穿）；违反时拒绝重采或抬 `cyl_radius_scale` | S1 L40-41, L169-173 |
| (—) | constraint | — | — | inequality | semi-auto：seated magazine 必须留在 grip footprint 内（`expect_within` x margin），且 retained ≥0.05 in magwell；违反时回缩 mag/ grip scale | S2 L462-475 |
| `palette_style` | enum | `blued_steel`, `stainless`, `two_tone`, `walnut_panel`, `polymer_olive`, `optic_black` | (按 spine 选默认) | choice | 见下表；纯材质/颜色，不改拓扑 | S1 L275-281 / S2 L174-181 |

### `palette_style`（≥3，目标 4–6；全部从 5 星样本材质提取）

| palette_style | 主体配色 | 来源样本材质 | 适用 spine |
|---|---|---|---|
| `blued_steel` | 深蓝黑钢 frame/slide + 黑细节 | S2 `slide_black`(0.10,0.10,0.11) / `sight_black` | both（revolver blued 经典） |
| `stainless` | 亮不锈钢 frame/barrel/cylinder | S1 `stainless_steel`(0.78,0.79,0.81)/`stainless_mid`/`stainless_bright` | revolver 优先 |
| `two_tone` | 黑 slide / olive 或亮 frame 对比 | S2 `slide_black` + `frame_olive`(0.42,0.43,0.36) | semi-auto 优先 |
| `walnut_panel` | stainless/blued 主体 + 胡桃木 grip panel | S1 `walnut`(0.45,0.28,0.15)/`walnut_dark`(0.28,0.16,0.09) | revolver（square grip） |
| `polymer_olive` | olive-drab polymer frame + graphite mag | S2 `frame_olive`/`mag_graphite`(0.20,0.20,0.21)/`baseplate_black` | semi-auto |
| `optic_black` | matte 黑 slide + 深绿 optic lens | pistoptic `optic_lens`(0.15,0.22,0.18) + S2 sight_black | semi-auto（optic_cut sights） |

palette gating：`walnut_panel` 需 grip 为 `revolver_square`（有木 panel）；`optic_black` 偏好 `sights=pistol_optic_cut`；其余 palette 与任意兼容 spine 组合自由。palette 不进入 topology distinct 计数。

## Multiplicity / Copy Logic

**0/1/K 轴说明**：本模板有 **1 根 multiplicity 轴**，且**条件存在**（仅当 `action=revolver_swingout`）。`semi_auto_slide` spine 无任何模板级复制轴（弹匣容量不建模为 per-round 复制，已折入 `grip` 的 compact/straight 长度档 + `mag_travel`，见排除项）。

### 轴 1：`chamber_count`（revolver cylinder 弹巢腔）

- `count_param`: `chamber_count`（派生 `chamber_angle_step = 360.0 / chamber_count`）
- `N_range`: `[5, 8]`（本小类本轴产品域；测试偏小档，产品全程 5–8）。5 星样本已覆盖 N∈{5(rev5shot), 6(parent), 8(rev8shot)}。
- sampling domain（权重档，按轴加权）：6 最常见（典型左轮）权重最高；5/7 次之；8 稀有（高容量 magnum/小口径）。下游模板对此轴做一次加权采样、编进 `slot_choices`、各自 clamp 到 [5,8]、sweep 上限设 8。
- copied object: 每个 copy = 一个 chamber bore（`_chamber_position(k)` 的 `_xcyl` cut 穿过 cylinder）+ 一个 flute（`_flute_position(k)`，位于相邻 chamber 之间）+ 一个 dark `chamber_liner_{k}` visual（off-axis 见证 continuous spin）。ejector star = `polygon(chamber_count, 0.0116)`。
- naming: `chamber_liner_{k}`（k=0..chamber_count-1）；k=0 在 bore 轴（q=0 顶腔）。
- placement: 等角分布于 cylinder 轴四周，`chamber_position(k)`：`a = radians(90 + chamber_angle_step·k)`，半径 `CHAMBER_CIRCLE_R`；顶腔（k=0）落在 bore 轴上。flute 位于腔间：`a = radians(90 + 0.5·step + step·k)`，半径 0.0215。
- joint policy: **无 per-chamber joint**。所有 chamber/flute/liner rigid 嵌在**单个** `cylinder` part 内，整体只骑一个 `cylinder_spin`(CONTINUOUS, +X) joint。复制只发生在 cylinder solid 的 cut/visual 层，不增 part/joint 数。
- source/gating: parent 原本硬编码 `for k in range(6)` × 2 + `polygon(6, ...)`（S1 L169-182, L189-194, L366-375）；rev5shot（S7=rec_handgun_var_rev5shot, L40-41, L165-206, L376-...）/rev8shot（S8b=rec_handgun_var_rev8shot, L40-44）**已**重写为 `range(CHAMBER_COUNT)` / `polygon(CHAMBER_COUNT, ...)` + `_chamber_position`/`_flute_position` helper。**模板契约**：采纳 rev5shot 的 helper 形式，所有三处循环（chamber cut / flute cut / liner visual）+ ejector polygon 用 `chamber_count` 参数化；CYL_R 经 `cyl_radius_scale=f(chamber_count)` 派生以保 chamber 间壁厚（inequality 兜底）。仅 revolver spine 暴露此轴；semi-auto 不暴露 `*_count`。

## 拓扑多样性审计

总组合数（按 spine 分支后求和）：

- revolver spine：barrel(2) × sights(2) × grip(2) × chamber_count(4: N∈{5,6,7,8}) = **32**
- semi-auto spine：barrel(2) × sights(2) × grip(2) = **8**（无 multiplicity 轴）
- 合计 raw distinct 拓扑 = 32 + 8 = **40**（palette / 连续 scale 不计入 distinct）


理由：40 个合法拓扑组合远超 10；两条 disjoint spine + 各自 barrel/sights/grip 真实结构差异 + revolver chamber multiplicity（5–8）共同撑起。5 星样本本身就覆盖了 10 个独立结构点（2 spine baseline + 8 单轴变体），证明每个 slot 的候选确实产生不同 part tree / joint / primitive。

seed_domain_policy：procedural_first

Procedural Sampling / Sweep Plan：
- deterministic procedural sampling 顺序：`action`（spine，加权：revolver/semi-auto 约均衡）→ 按 spine 解析 `barrel_length`/`sights`/`grip` 的 conditional 候选池并加权采样 → 若 revolver 则对 `chamber_count` 做 [5,8] 加权采样（6 高频，8 稀有）→ `palette_style`（受 grip/sights gating）→ 连续 scale（先 independent `barrel_len_scale`/`grip_height_scale`，再 equation 派生 `mag_travel`/`cyl_radius_scale`，最后 inequality 投影接地与 chamber clearance）。
- `seed=0` 不特殊；全 seed 走 procedural sampling。
- compatibility matrix 强制：revolver action 永不配 pistol_* 候选，反之亦然；`chamber_count` 轴仅 revolver；palette gating 见上。
- 初始 sweep seeds `0-49`；成熟度审计 `0-999`；viewer 预览须覆盖两条 spine + 各 barrel/sights/grip + chamber_count 端点（5 与 8）。
- random sweep 关注：grip/mag 接地、chamber 不互穿、cylinder swing-out 与 ejector push 不撞 underlug、slide 往复 clearance、optic pocket 不破 bore。

Topology target：1000-seed slot choice tuple distinct 上限受类目真实结构约束 = 40（spine×barrel×sights×grip×chamber）。低于富类别建议 ≥300，原因：handgun 真实成熟 action 仅 2 disjoint 家族，每 spine 的功能层各只有 2 个结构候选，chamber 轴只有 4 档；这是类目内在结构上限，不是采样不足。多样性靠 spine 切换 + 条件候选 + multiplicity 充分表达，连续 scale 提供形态变化但不计入 distinct。（统一口径：富类别建议 ≥300；低于 300 记录真实组合空间或兼容约束原因；report-only，不设门。）

Controlled local parameterization（初版应含的关键连续 scale）：
- `barrel_len_scale` `[0.85,1.15]` independent，clamp 到不跨邻近 barrel 语义档（mid↔snub / mid↔long 之间留 buffer）。
- `grip_height_scale` `[0.92,1.08]` independent，受接地 inequality 回缩。
- `mag_travel` = `grip_height_scale · base_mag_travel(grip)`（equation，semi-auto）。
- `cyl_radius_scale` = `f(chamber_count)`（equation，revolver；保 chamber 间壁厚）。
所有 scale 在 `resolve_config` 内 clamp/派生/投影，不破 InterfaceSpec（barrel union 座、top-strap dovetail、grip FIXED 接触、magwell 内壁）、MatingContract（crane arbor↔cylinder bore 过盈、ejector rod↔arbor bore、slide↔rail gap）、multiplicity（chamber 等角 + clearance）。

| item | policy | validator / viewer focus |
|---|---|---|
| sampler | action → (conditional barrel/sights/grip) → chamber_count(rev) → palette → scales | `slot_choices_for_seed` 反映实际 build choices |
| compatibility matrix | 见下表；revolver/semi-auto 候选池互斥；chamber 轴仅 revolver | 无 floating barrel/sight/grip、无 cross-family、无穿模、接地正确 |
| controlled local variation | `barrel_len_scale`, `grip_height_scale`（+ derived `mag_travel`/`cyl_radius_scale`） | 比例变化不破 interface/clearance/接地/joint origin/identity |
| regression overrides | none（初版） | 仅已知失败回归或审核指定样本 |
| random sweep | seeds 0-49 初验，0-999 成熟度 | contract 失败 |

### Compatibility matrix / gating

| action (spine) | legal barrel_length | legal sights | legal grip | chamber_count 轴 | gating notes |
|---|---|---|---|---|---|
| `revolver_swingout` | `revolver_mid`, `revolver_snub` | `revolver_fixed`, `revolver_adjustable` | `revolver_square`, `revolver_roundbutt` | 有，[5,8] | crane swing-out clearance；ejector rod 行程不撞 underlug；`walnut_panel` palette 需 square grip |
| `semi_auto_slide` | `pistol_mid`, `pistol_long` | `pistol_fixed`, `pistol_optic_cut` | `pistol_straight`, `pistol_compact` | 无 | slide↔rail gap 维持；optic_cut pocket 不破 bore；mag 留 grip footprint 内；compact grip 配 0.07 mag travel |

| 互斥 / fallback | 规则 |
|---|---|
| cross-family | 禁止 revolver action 配任何 pistol_* 候选（反之亦然）——出类目 |
| chamber_count | 仅 revolver 暴露；semi-auto 选中时该轴不存在（不写 `*_count`） |
| palette | `walnut_panel`→需 revolver_square；`optic_black`→偏好 pistol_optic_cut；不满足时回退 spine 默认 palette |

| slot | candidate_count | 是否 ≥2 | 是否 ≥3 | 备注 |
|---|---:|---|---|---|
| action | 2 | yes | no | spine 选择器；类目真实只有 2 disjoint action 家族，已说明降级；驱动下游条件候选池 |
| barrel_length | 4 (2/spine) | yes | yes | revolver_mid/snub + pistol_mid/long |
| sights | 4 (2/spine) | yes | yes | revolver_fixed/adjustable + pistol_fixed/optic_cut |
| grip | 4 (2/spine) | yes | yes | revolver_square/roundbutt + pistol_straight/compact |

## Validator

- `__modular__ = True`
- `slot_choices_for_seed` 返回四个 slot 的实际 module 名 + （revolver 时）`chamber_count`
- `config_from_seed` 全 seed 走 deterministic procedural sampling；`seed=0` 不特殊
- compatibility matrix / gating 阻止非法组合（cross-family revolver×pistol；chamber_count 出现在 semi-auto）
- regression overrides 稀疏且有理由（初版 none）
- 最终模板不靠小型 curated/modulo 表循环作主 seed domain
- controlled local scale（`barrel_len_scale`/`grip_height_scale`）被 clamp，且不破 interface/clearance/joint origin/multiplicity
- cross-part scale 依赖（`mag_travel` equation、`cyl_radius_scale` equation、接地与 chamber clearance inequality）在 `resolve_config` 内求解，不留到 builder 失败
- 关键 InterfaceSpec / MatingContract 点存在：crane arbor↔cylinder center bore 过盈（`allow_overlap` + `expect_overlap`）、ejector rod↔crane arbor bore、closed crane 座 frame rail、slide↔frame rail gap（`expect_gap` z）、grip↔frame 接触、magazine 在 magwell 内
- 关键 joint 类型/轴/range：revolver `cylinder_spin`(CONTINUOUS,+X)、`crane_swing`(REVOLUTE,-X,0..~0.785)、`ejector_push`(PRISMATIC,-X,0..0.02)、`trigger_pull`/`hammer_cock`(REVOLUTE,±Y)；semi-auto `frame_to_slide`(PRISMATIC,-X,0..0.045)、`frame_to_magazine`(PRISMATIC,沿 RAKE)、`frame_to_trigger`/`frame_to_takedown_lever`(REVOLUTE,+Y)
- 复制对象遵循命名/放置策略：`chamber_liner_{k}`（k=0..chamber_count-1），等角，顶腔在 bore 轴，rigid 在单个 cylinder part；`len(chamber_liner)==chamber_count`
- off-axis 见证：half-turn spin 把顶腔（liner_0）带到底部（z 下降 >0.02）

## Reject cases

- cross-family hybrid：revolver action 配 pistol slide / pistol grip，或 mixed spine part（两 spine 是 disjoint part/joint 集，出类目）
- `chamber_count` 出现在 semi-auto spine，或暴露任何 magazine `round_count` 复制轴
- chamber 数超 [5,8]，或 chamber 互穿（clearance inequality 失效 / `cyl_radius_scale` 未派生）
- cylinder 上挂 per-chamber joint（应 rigid 在单 cylinder part，只骑一个 `cylinder_spin`）
- grip butt / magazine baseplate 不接地（最低点远离 z≈0），或 grip 漂浮（revolver grip FIXED mount 未接触 frame）
- swing-out 摆出时 cylinder/ejector 撞 underlug，或 ejector push 行程穿 barrel shroud；slide 往复无 rail clearance（`expect_gap` 失败）
- bore 不 hollow（muzzle probe 命中实体），或 optic pocket cut 破穿 bore
- barrel/sight/grip module 与所选 action 不兼容却被采样（conditional gate 失效）
- `barrel_len_scale`/`grip_height_scale` 越界改变 module 语义档（mid 缩成 snub / long 缩成 mid），或破 InterfaceSpec/接地

## 与相邻类别的边界

- 不该混入：**长枪（rifle / shotgun / submachine gun / carbine）**——它们有肩托（stock）、长 barrel、双手前后握持，整体 >0.5 m；handgun 单手 grip-butt 接地、barrel 短、无肩托。
- 不该混入：**玩具枪 / 水枪 / nerf / cap gun**——非真实 firearm 机构（无 cylinder spin / slide reciprocate / hammer），多为玩具配色与中空塑壳；handgun 必须有真实 action spine 的活动语义。
- 不该混入：**枪械配件 / 单独零件（弹匣、枪套 holster、suppressor、瞄具单品）**——它们不是完整 handgun，缺 grip+trigger+action+barrel 的核心身份组合。
- 不该混入：**电钻 / 钉枪 / 胶枪 等 "gun-shaped" 工具（caulking_gun 等已有独立模板）**——它们有 trigger-grip 外形但功能是工具，无弹巢/套筒/击锤 firearm 机构。

## 审核记录

| 项 | 结论 |
|---|---|
| reviewer status | pending |
| reviewer notes | （待人工审核） |

## 模板实现备注（可选）

- revolver / semi-auto 两条 spine 各自的 helper 集 disjoint（`_xcyl/_bbox/_frame_body_solid/_barrel_solid/_crane_solid/_cylinder_solid/_rod_solid` vs `_build_frame_solid/_build_rail_solid/_build_slide_solid/_build_trigger_solid`）；模板按 `action` 分派两套 build 路径，不共享几何 helper。
- `chamber_count` 重构必须采纳 rev5shot 的 helper 形式：`CHAMBER_ANGLE_STEP = 360.0 / CHAMBER_COUNT` + `_chamber_position(k)` / `_flute_position(k)`，并把 parent 的三处 `for k in range(6)`（chamber cut L169-173 / flute cut L177-181 / liner visual L366-375）+ `polygon(6, ...)` ejector star（L189-194）全部参数化。
- captured-pin / 过盈 overlap 需 element-scoped `allow_overlap`：revolver 的 crane_body↔cylinder_body、crane_body↔ejector_rod_body、crane_body↔frame_body（closed crane 座 rail 0.5 mm）；semi-auto 的 frame_body↔trigger_blade、frame_body↔lever_boss、frame_body↔mag_baseplate。复合/缩放后这些 overlap 必须随 module 选择复现。
- 暂不进入 seed domain 的组合：任何 cross-family 混装（compatibility matrix 排除）。

## Module Source Index

| source_id | slot | module | sample_id | model.py 来源 | 采纳用途 |
|---|---|---|---|---|---|
| S1 | A/B/C/D | revolver_swingout + revolver_mid + revolver_fixed + revolver_square | rec_model-a-classic-double-action-revolver-colt-pyth_20260610_081456_135585_9e7d2f05 | L88-L488（含 multiplicity baseline L169-194, L366-375） | revolver spine part tree + joints + barrel/sight/grip baseline + chamber 复制原型 |
| S2 | A/B/C/D | semi_auto_slide + pistol_mid + pistol_fixed + pistol_straight | rec_model-a-modern-striker-fired-semi-automatic-pist_20260610_081229_238472_7155f244 | L80-L331 | semi-auto spine part tree + joints + slide/sight/grip+magazine baseline |
| S3 | B | revolver_snub | rec_handgun_var_revsnub | L31, L53, L109-L124 | 短 barrel + 短 underlug barrel module |
| S4 | B | pistol_long | rec_handgun_var_pistlong | L51, L79-L80, L147-L175 | 长 slide barrel module（SLIDE_MID_X 重定位） |
| S5 | C | revolver_adjustable | rec_handgun_var_revadjsight | L146-L179 | 高 adjustable rear sight 组件（windage+elevation） |
| S6 | C | pistol_optic_cut | rec_handgun_var_pistoptic | L148-L168, L245-L274 | optic-ready milled pocket + red-dot housing + lens + screws |
| S7 | D + multiplicity | revolver_roundbutt（grip）/ rev5shot（chamber refactor） | rec_handgun_var_revroundbutt ; rec_handgun_var_rev5shot | roundbutt L248-L275 ; rev5shot L40-41, L165-206, L376 | spline 圆 butt grip ; `CHAMBER_COUNT` helper 重构原型（N=5） |
| S8 | D + multiplicity | pistol_compact（grip）/ rev8shot（chamber N=8） | rec_handgun_var_pistcompact ; rec_handgun_var_rev8shot | compact L58-59, L65, L85-L123 ; rev8shot L40-44 | 缩短 grip + 短 mag travel ; chamber N=8 端点验证 |
