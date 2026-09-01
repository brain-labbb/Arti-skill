# first_aid_cabinet (wall-mounted first-aid cabinet) — Modular Spec

> 来源小类：`picture/Science/First aid cabinet`（articraft_data 上游 Science/First aid cabinet fork-variant pool）。
> 源 source map：`articraft_data/picture_expansion/template_source_maps/Science__First_aid_Other_Cabinet.md`。
> 1 母资产 + 6 个 converged fork 变体 = 7 个 5★ 样本，全部读 `model.py`（见 §5 摘要）。
> 引用 `model.py:Lx-Ly` 来自各样本 `articraft_data/data/records/<id>/revisions/rev_000001/model.py`，
> 以 part / joint / helper **名字** 为准（`cabinet_body` / `cabinet_door` / `body_to_door` /
> `door_{i}` / `body_to_door_{i}` / `drawer` / `drawer_{i}` / `body_to_drawer` / `body_to_drawer_{i}` /
> `shelf_{i}` / `body_hinge_barrel` / `door_hinge_barrel` / `bottom_front_panel` / `build_shelf` /
> `build_drawer_tray` / `build_hinge_knuckles` 等），行号仅作定位（按各样本读取时锚定，重排后以名字为准）。

## 元信息
| 项 | 值 |
|---|---|
| slug | `first_aid_cabinet` |
| template path | `agent/templates/Science_First_aid_cabinet.py` |
| test path (optional) | `tests/agent/test_first_aid_cabinet_template.py`（不写，sweep 为唯一验收）|
| stage | `SPEC_ONLY_DRAFT` |
| status | `pending` |
| __modular__ | `True` |
| pattern | `mixed`（固定 named slots：`door`(主机构) + `interior_fitment`；外加两根 multiplicity 轴——`shelf_count` FIXED `shelf_{i}` 视觉栈 + `drawer_count` 各自 +Y PRISMATIC `drawer_{i}` 栈）|

## 5 星样本阅读摘要
| 项 | 值 |
|---|---|
| five_star_total | 1 parent + 6 converged fork 变体 = 7 |
| read_count | 7（全部读 `model.py` 全文 / 关键机构段）|
| read_scope | all 5-star samples in this category（combinatorial fork pool：parent 全读 + 每个变体读其差异层：door type / interior fitment / shelf-count loop / drawer-stack loop）|
| source_index_policy | 仅被采纳为 module source 的样本进入下方 slot 表与 §14 |

逐样本要点（采纳归属）：
- **P1 parent**（`rec_build-...-firs_20260609_183625_780787_99727092`）：白色钣金壁挂柜。`cabinet_body` 为单根（`build_body_shell` 前开口空腔壳 + 顶部 `carry_handle` 提手 + `body_hinge_barrel` 铰链桶 + 内部 `shelf_{i}` + 码放 `supply_*` 补给）；唯一活动子件 `cabinet_door`，经 `body_to_door` REVOLUTE 绕左前竖边 +Z 轴 0..150° 外摆；门面 `door_frame`(开窗) + `door_glass` + `door_emblem`(红十字) + `door_banner`(FIRST AID) + `door_knob` + `door_hinge_barrel`。**采纳为 door=glass_front_hinged 基线 + interior_fitment=open_shelves 基线 + shelf_count 复制契约 N=2 + carcass 共享件（body shell / handle / hinge barrels / supplies）+ palette white+red-cross 基线**。`model.py:L77-L399`（geometry+assembly），`shelf` 循环 `L267-L275`，hinge `L325-L399`。
- **solid_door**（`rec_first_aid_cabinet_var_solid_door`）：门面换为 `build_door_panel` 实心不开窗钣金板（无 `door_glass`，红十字+banner 改为 `DECAL_T` 印刷贴 `door_emblem`/`door_banner` 贴附面板表面），其余（body / hinge / shelf 循环 / supplies / `body_to_door` REVOLUTE）与 parent 同。**采纳为 door=solid_panel_hinged**。`build_door_panel` `L131-L140`，无 glass 断言 `L386-L392`。
- **double_doors**（`rec_first_aid_cabinet_var_double_doors`）：单门拆为两窄扇 `door_0`/`door_1`（`build_door_frame(side)`/`build_door_glass(side)`/... 按 `side=±1` 参数化），各自 `body_to_door_0`(axis +Z) / `body_to_door_1`(axis -Z) REVOLUTE 从中线对开；body 侧 `body_hinge_barrel_{i}` 双角铰；闭合时两扇在 Z 全高相接（`L494-L498` overlap 断言）。**采纳为 door=double_doors**。`door_sides=[1,-1]` 循环 `L317-L353`，双 hinge `L370-L388`。
- **drawer_base**（`rec_first_aid_cabinet_var_drawer_base`）：parent + 在底部隔间加一只抽屉。`cabinet_body` 增 `bottom_front_panel`(开抽屉口) + `slide_rail_l/r`；`drawer` 部件（`build_drawer_tray` 开顶托盘 + `drawer_front` 面板 + `drawer_pull` 拉手 + `drawer_rail_{i}` + 内置 `drawer_supply_{i}`）经 `body_to_drawer` PRISMATIC 沿 +Y 拉出 0..0.080。门仍为 glass_front_hinged。**采纳为 interior_fitment=shelves_plus_drawer（drawer_count=1 端点）+ 单抽屉 prismatic 接口契约 + bottom_front_panel / slide_rail 共享件**。drawer geometry `L280-L344`，`body_to_drawer` `L561-L571`。
- **drawer_stack**（`rec_first_aid_cabinet_var_drawer_stack`）：上半门区（`HINGE_Z_BODY` 抬升的 `body_to_door` REVOLUTE）+ 横向 `divider` 隔板 + 下半 3 只抽屉栈。`build_body_shell` 自带 `divider`(SPLIT_Z) + 下半 `lower_panel` 在 `for i in range(N_DRAWERS)` 里铣 3 个抽屉口；`drawer_{i}` 部件（共享 `build_drawer_tray` 含一体化 front+pull）经 `body_to_drawer_{i}` 各自 +Y PRISMATIC 0..0.085 拉出；`drawer_slot_z(i)= -INNER_H/2 + DRAWER_SLOT_H·(i+0.5)`（下半区等距）。**采纳为 interior_fitment=shelves_plus_drawer_stack + drawer_count 复制契约（copied object / 命名 / placement / +Y PRISMATIC joint policy）N=3 样本 + upper/lower split + divider 共享件**。drawer loop `L463-L513`，`drawer_slot_z` `L80-L82`，prismatic 循环 `L500-L513`。
- **one_shelf**（`rec_first_aid_cabinet_var_one_shelf`）：parent 结构，`shelf_z = [0.0]`（单层居中）。**采纳为 shelf_count multiplicity N=1 端点 + 居中单层 placement**。shelf 段 `L268-L274`。
- **three_shelf**（`rec_first_aid_cabinet_var_three_shelf`）：parent 结构，`n_shelves=3`，`shelf_z = [(-INNER_H/2)+(k+1)·INNER_H/(n_shelves+1) for k in range(n_shelves)]`（N+1 等分）+ `support_z = [floor_top]+[z+SHELF_T/2 for z in shelf_z]` 每隔间一行补给。**采纳为 shelf_count multiplicity 的规范等距公式 + copied-object 命名/placement/支撑契约 N=3 样本**。shelf 循环 `L268-L277`，等距公式 `L269-L270`，supply 支撑 `L294`。

## 核心身份

壁挂式急救药品柜（first-aid cabinet）：一只白色钣金箱体 `cabinet_body` 作为**单一接地根**（前开口空腔壳 + 顶部提手 + 红十字/FIRST AID 身份标志），背面贴墙；正面由一扇或两扇绕竖边外摆的 **REVOLUTE 门** 封闭，内部由 **FIXED 横隔板 `shelf_{i}`** 分层、可选叠加 **+Y PRISMATIC 抽屉 `drawer_{i}`** 栈，码放纱布/药盒补给。默认成熟域：小尺度壁挂柜（~0.34W × 0.13D × 0.40H m），门为单轴竖铰，抽屉为单轴前后滑出。

核心运动身份 = `cabinet_body`(root) → 至少一个 **REVOLUTE 门**（竖 Z 轴，0..~150°，绑左/右/双前缘 hinge barrel）；内部 fitment 决定是否再叠加一组 **PRISMATIC 抽屉**（+Y，单轴前拉）。任何 seed 必须保留：① 单根接地 body；② ≥1 个 REVOLUTE 门暴露内腔。

不该混入：
- 自立 / 带腿 / 落地药柜（free-standing / legged）——本类是壁挂，不出脚、不带底座支腿。
- 卷帘门 / tambour / roll-up（无样本提供干净单轴铰接，排除）。
- 把 `door_knob` / `drawer_pull` / `carry_handle` / 红十字贴 当作独立 slot——它们是 module-local 固定 visual / 装饰，不是 slot。

## 槽位 + 候选模块表

### Slot A：door（封闭机构，主 REVOLUTE 机构，绑 body 前缘 hinge barrel）

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| glass_front_hinged | rec_build-...-firs_..._99727092 (parent) | L145-L226（door geometry）, L336-L399（door part + `body_to_door` REVOLUTE） | eligible if compatible | 单扇 `cabinet_door`：`door_frame`(开窗) + `door_glass` 玻璃 + `door_emblem` 红十字 + `door_banner` + `door_knob` + `door_hinge_barrel`；1 个 `body_to_door` REVOLUTE 绕左前竖边 +Z 0..150° |
| solid_panel_hinged | rec_first_aid_cabinet_var_solid_door | L131-L187（`build_door_panel` 实心板 + 印刷贴）, L285-L337（door part + `body_to_door` REVOLUTE） | eligible if compatible | 单扇实心钣金门 `door_panel`（无 `door_glass`）；红十字/banner 为 `DECAL_T` 印刷贴贴附面板；同 1 个 `body_to_door` REVOLUTE +Z；视觉计数与 glass 版不同（少 glass、贴更薄） |
| double_doors | rec_first_aid_cabinet_var_double_doors | L145-L228（`build_door_*(side)` 参数化窄扇）, L315-L388（`door_{i}` 循环 + 双 `body_to_door_{i}` REVOLUTE） | eligible if compatible | 两窄扇 `door_0`/`door_1`（各窗+玻璃+红十字+banner+knob+hinge），从中线对开：`body_to_door_0` axis +Z、`body_to_door_1` axis -Z；body 侧 `body_hinge_barrel_{i}` 双角铰；闭合时两扇 Z 全高相接 |

### Slot B：interior_fitment（内腔分层 / 抽屉，决定是否叠加 PRISMATIC 抽屉栈）

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| open_shelves | rec_build-...-firs_..._99727092 (parent) | L104-L108（`build_shelf`）, L267-L323（`shelf_{i}` 循环 + supplies） | eligible if compatible | 纯 FIXED 横隔板 `shelf_{i}`（无抽屉），按 `shelf_count` 轴等距堆叠；每隔间码放 `supply_*`；无 PRISMATIC 子件 |
| shelves_plus_drawer | rec_first_aid_cabinet_var_drawer_base | L128-L142（`bottom_front_panel` 开口）, L280-L344（drawer geom）, L475-L531（drawer part）, L558-L571（`body_to_drawer` PRISMATIC） | eligible if compatible | 上部 `shelf_{i}` 保留 + 底隔间加 **1** 只 `drawer`（`drawer_count` 轴 = 1 端点）：`build_drawer_tray` 开顶托盘 + `drawer_front` + `drawer_pull` + `drawer_rail_{i}` + body 侧 `bottom_front_panel`/`slide_rail_l/r`；`body_to_drawer` +Y PRISMATIC 0..0.080 |
| shelves_plus_drawer_stack | rec_first_aid_cabinet_var_drawer_stack | L60-L82（upper/lower split + `drawer_slot_z`）, L123-L164（`build_body_shell` divider+lower_panel 铣孔循环）, L212-L264（`build_drawer_tray`）, L463-L513（`drawer_{i}` 循环 + `body_to_drawer_{i}` PRISMATIC 循环） | eligible if compatible | 上半门区（`HINGE_Z_BODY` 抬升铰）+ 横 `divider` + 下半 **N** 只 `drawer_{i}` 栈（`drawer_count` 轴 ≥2）：`build_body_shell` 自铣 N 个抽屉口；各 `drawer_{i}` 共享托盘几何，各自 `body_to_drawer_{i}` +Y PRISMATIC 0..0.085；`drawer_slot_z(i)` 下半区等距 |

> 单候选降级说明：无。Slot A=3 candidates、Slot B=3 candidates，均 ≥3，均来自结构不同的 5★ 样本（door 三种 part-tree/joint-count，fitment 三种 joint-topology：0 PRISMATIC / 1 PRISMATIC / N PRISMATIC + upper-lower split），无需折叠或降级到 2。

## 槽位图（slot graph）

pattern: mixed（parallel children + 双 multiplicity 轴）

```
                 cabinet_body (root, grounded, 单一接地根)
                 ├─[REVOLUTE +Z @ 前缘 hinge barrel, 0..150°]──> Slot A: door
                 │     · glass_front_hinged: 1× cabinet_door  (单 body_to_door)
                 │     · solid_panel_hinged: 1× cabinet_door  (单 body_to_door)
                 │     · double_doors:       2× door_{i}      (body_to_door_0 +Z / _1 −Z, 中线对开)
                 │
                 ├─[FIXED 视觉, shelf_count 轴]──> shelf_{i}   (Slot B 的 open-shelf 层, 无 joint)
                 │     · z_i = −INNER_H/2 + (i+1)·INNER_H/(N_shelf+1)   (规范等距, FIXED visual)
                 │
                 └─[PRISMATIC +Y, drawer_count 轴]──> Slot B: drawer 栈 (仅 fitment 含抽屉时)
                       · shelves_plus_drawer:       1× drawer     (body_to_drawer,   0..0.080)
                       · shelves_plus_drawer_stack: N× drawer_{i} (body_to_drawer_{i}, 0..0.085)
```

接口点位与 joint 策略：
- **body→door（跨 slot, REVOLUTE）**：mating = body 前缘 `body_hinge_barrel(_i)` 竖桶 ↔ door `door_hinge_barrel(_i)` 竖桶（barrel 互嵌 knuckle 交错，captured-pin overlap，grandfather mating）。joint 原点 = 前缘 hinge 线（x = ±(BODY_W/2−0.006)，y = BODY_D/2+HINGE_INSET，z = body 中或上半区中 `HINGE_Z_BODY`）。axis 单门/左扇 = (0,0,+1)、右扇 = (0,0,−1)。range 0..radians(150)。闭合姿态：门面 +Y 盖住 body 前面（`expect_overlap axes="xz"`），door_max_y > body_max_y。
- **body→shelf（multiplicity 轴, FIXED visual）**：shelf 是 `cabinet_body` 内的 FIXED 视觉（非独立 part、无 joint），oversized X 嵌入两侧壁（`SHELF_EMBED`），Y 跨度从背壁内到闭门面后（不撞门）。等距公式见 §8。element-scoped `allow_overlap(body, body, shelf_i, body_shell, ...)`。
- **body→drawer（multiplicity 轴, PRISMATIC）**：每 `drawer_{i}` 是 body 的独立 PRISMATIC 子件。mating = drawer `drawer_front` 后面 ↔ body `bottom_front_panel`/`lower_panel` 开口缘（闭合 q=0 抽屉缩入腔内，front 面盖住开口）。joint 原点 = body 前面 (y = BODY_D/2)，z = 该抽屉槽中心。axis = (0,1,0)，range 0..~0.08。多抽屉时各自独立轴（一个隔间一只，等距堆叠）。

互斥 / 派生 / 可选：
- Slot A 三候选互斥（一个 seed 仅一种门型）。
- Slot B 三候选互斥；`open_shelves` 不发射任何 drawer（`drawer_count`=0）；`shelves_plus_drawer` 强制 `drawer_count`=1；`shelves_plus_drawer_stack` 强制 `drawer_count`∈[2,N]。
- `shelf_count` 轴在 `open_shelves` 与 `shelves_plus_drawer` 下用整腔/上腔等距；在 `shelves_plus_drawer_stack` 下 shelf 仅占上半门区（下半被抽屉栈占用），shelf_count 受上半区高度约束（见 §8 conditional）。

## 每槽位 Module Emits / Interfaces

### Slot A / module glass_front_hinged
| emits | 描述 | 来源 |
|---|---|---|
| parts | `cabinet_door`（`door_frame` 开窗 + `door_glass` + `door_emblem` + `door_banner` + `door_knob` + `door_hinge_barrel`） | parent / L336-L368 |
| internal joints | 无（门内部全 FIXED 视觉） | parent |
| upstream interface | door `door_hinge_barrel` 竖桶（local x=0 pin），消费 REVOLUTE +Z；闭合时门面 +Y 盖 body 前面 | parent / L362-L368, L389-L399 |
| downstream interface | 无（门是终端活动件） | parent |

### Slot A / module solid_panel_hinged
| emits | 描述 | 来源 |
|---|---|---|
| parts | `cabinet_door`（`door_panel` 实心 + 印刷 `door_emblem`/`door_banner` + `door_knob` + `door_hinge_barrel`；无 `door_glass`） | solid_door / L286-L313 |
| internal joints | 无 | solid_door |
| upstream interface | 同 glass 版：`door_hinge_barrel` ↔ body barrel，REVOLUTE +Z | solid_door / L307-L313, L327-L337 |
| downstream interface | 无 | solid_door |

### Slot A / module double_doors
| emits | 描述 | 来源 |
|---|---|---|
| parts | `door_0`、`door_1`（各 `frame_{i}`/`glass_{i}`/`emblem_{i}`/`banner_{i}`/`knob_{i}`/`hinge_barrel_{i}`） | double_doors / L319-L353 |
| internal joints | `body_to_door_0`(REVOLUTE axis +Z)、`body_to_door_1`(REVOLUTE axis −Z) | double_doors / L370-L388 |
| upstream interface | 两套 `hinge_barrel_{i}` ↔ body `body_hinge_barrel_{i}` 双角铰；闭合两扇 Z 全高相接 | double_doors / L301-L313, L494-L498 |
| downstream interface | 无 | double_doors |

### Slot B / module open_shelves
| emits | 描述 | 来源 |
|---|---|---|
| parts | 无独立 part；向 `cabinet_body` 追加 FIXED `shelf_{i}` 视觉 + `supply_*` 补给 | parent / L267-L323 |
| internal joints | 无 | parent |
| upstream interface | shelf 嵌入 body 侧/背壁（`allow_overlap shelf_i/body_shell`） | parent / L539-L545 |
| downstream interface | 无（fitment 不向下游传递）；供 shelf_count 轴消费 | parent |

### Slot B / module shelves_plus_drawer
| emits | 描述 | 来源 |
|---|---|---|
| parts | `drawer`（`drawer_tray`+`drawer_front`+`drawer_pull`+`drawer_rail_{i}`+`drawer_supply_{i}`）；body 追加 `bottom_front_panel`+`slide_rail_l/r`；上部保留 `shelf_{i}` | drawer_base / L361-L531 |
| internal joints | `body_to_drawer`（PRISMATIC +Y，0..0.080） | drawer_base / L558-L571 |
| upstream interface | drawer `drawer_front` 后面 ↔ body `bottom_front_panel` 开口缘；闭合 front 盖口（`expect_contact drawer_front/bottom_front_panel`） | drawer_base / L811-L822 |
| downstream interface | 无；drawer_count 轴固定为 1 | drawer_base |

### Slot B / module shelves_plus_drawer_stack
| emits | 描述 | 来源 |
|---|---|---|
| parts | `drawer_{i}`（i=0..N−1，共享 `build_drawer_tray` 一体托盘+front+pull）；body `build_body_shell` 自带 `divider`+`lower_panel`(铣 N 口)；上半保留 `shelf_{i}` | drawer_stack / L123-L164, L463-L471 |
| internal joints | `body_to_drawer_{i}`（各 PRISMATIC +Y，0..0.085，统一策略） | drawer_stack / L500-L513 |
| upstream interface | 每 `drawer_{i}` front ↔ `lower_panel` 第 i 口（`drawer_slot_z(i)` 等距）；闭合缩入腔内 | drawer_stack / L154-L163, L755-L759 |
| downstream interface | 无；drawer_count 轴 ≥2 | drawer_stack |

## 参数范围汇总
| 参数 | 类型 | 取值范围 / 候选值 | 标称默认 | 约束类型 | 约束 / 函数 | 来源 |
|---|---|---|---|---|---|---|
| door_choice | enum | glass_front_hinged / solid_panel_hinged / double_doors | — | choice | deterministic procedural sampler 选择 | Slot A 表 |
| fitment_choice | enum | open_shelves / shelves_plus_drawer / shelves_plus_drawer_stack | — | choice | deterministic procedural sampler 选择 | Slot B 表 |
| shelf_count | int | [1, 6]（产品域；测试偏小，见 §8） | 2 | independent | per-N 加权抽样后 clamp；FIXED `shelf_{i}` 等距 | one_shelf L268 / parent L267-L275 / three_shelf L269-L270 |
| drawer_count | int | open_shelves→0；shelves_plus_drawer→1（固定）；shelves_plus_drawer_stack→[2,4] | 由 fitment 派生 | conditional | 范围随 fitment_choice 解析；≥2 时各自 +Y PRISMATIC | drawer_base / drawer_stack L67,L500-L513 |
| palette_style | enum | clinical_white_redcross / stainless_steel / emergency_green / industrial_grey / vintage_enamel | clinical_white_redcross | choice | per-seed 采样；只改 material rgba，不改拓扑 | 见 §palette |
| body_width_scale | float | [0.85, 1.20] | 1.0 | independent | 在 [min,max] 均匀采样后 clamp；缩放 BODY_W / INNER_W / DOOR_W；门面与抽屉宽随动派生 | parent BODY_W L37 |
| body_height_scale | float | [0.85, 1.25] | 1.0 | independent | 缩放 BODY_H / INNER_H / Z_LIFT；shelf/drawer 槽 z 随 INNER_H 派生 | parent BODY_H L39 |
| door_thickness_scale | float | [0.85, 1.15] | 1.0 | independent | 缩放 DOOR_T；不改门高/宽（仅 Y 厚度，保持闭合 seat） | parent DOOR_T L45 |
| drawer_travel_scale | float | [0.85, 1.15] | 1.0 | independent | 缩放 PRISMATIC upper（~0.080/0.085） | drawer_base L569 / drawer_stack L77 |
| (—) shelf 等距 | constraint | — | — | equation | `shelf_z[i] = −INNER_H/2 + (i+1)·INNER_H/(N_shelf+1)`，shelf 不独立给 z | three_shelf L269-L270 |
| (—) drawer 槽等距 | constraint | — | — | equation | `drawer_slot_z(i) = lower_floor + DRAWER_SLOT_H·(i+0.5)`，`DRAWER_SLOT_H = LOWER_H/N_drawer` | drawer_stack L68,L80-L82 |
| (—) 门宽随 body | constraint | — | — | equation | `DOOR_W = k·BODY_W`（单门）/ `DOOR_LEAF_W = BODY_W/2 − 0.006 − PANEL_X0`（双扇）；保形随 body_width_scale | parent L43 / double_doors L57 |
| (—) shelf-vs-drawer 区高 | constraint | — | — | inequality | stack 模式：shelf 仅占上半门区 `[SPLIT_Z, +INNER_H/2]`，下半 `[−INNER_H/2, SPLIT_Z]` 全给抽屉栈；`N_shelf·SHELF_T + 间隙 ≤ UPPER_H`，超界回缩 shelf_count | drawer_stack L60-L82 |
| (—) 抽屉宽 clear 铰 | constraint | — | — | inequality | `DRAWER_W ≤ INNER_W − 2·hinge_clearance`（抽屉避开左/双侧 hinge barrel）；single-drawer 用 0.260 窄盘，stack 用 INNER_W−0.008（无侧铰下落区） | drawer_base L82 |

连续尺寸采样契约（写进 `config_from_seed`/`resolve_config`）：
1. 先采 `independent`：body_width_scale、body_height_scale、door_thickness_scale、drawer_travel_scale（均匀采样后 clamp）。
2. 按 `equation` 派生：DOOR_W/DOOR_LEAF_W（随 body_width_scale 保形）、shelf_z（随 INNER_H 与 N_shelf）、drawer_slot_z（随 LOWER_H 与 N_drawer）。
3. 用 `inequality` 投影/回缩：shelf-vs-drawer 区高（stack 模式回缩 shelf_count）、抽屉宽 clear hinge（回缩 DRAWER_W），无法满足则拒绝重采。
4. `conditional` 范围先于采样按 fitment_choice 解析 drawer_count 域；shelf_count 上限在 stack 模式按上半区高度解析。

## Multiplicity / Copy Logic

本模板有 **2 根独立 multiplicity 轴**，各自加权采样、各自 clamp、sweep 各自设上限。

### 轴 1：shelf_count（内部 FIXED 隔板）
- `count_param`: `shelf_count`
- `N_range`: **[1, 6]**（本小类本轴产品域；测试偏小、产品全程。样本覆盖 {1,2,3} → one_shelf / parent / three_shelf）
- sampling domain（权重档）：小 N 高频（N∈{1,2,3} 占大头），N∈{4,5,6} 稀有尾部下采样。
- copied object: `cabinet_body` 内的 FIXED `shelf_{i}` 视觉（同一 `build_shelf` mesh，oversized X 嵌入两侧/背壁）；每隔间叠一行 `supply_*` 补给（可选装饰）。
- naming: `shelf_{i}`（i=0..N−1，自下而上）。supply 命名 module-local，不暴露为 count。
- placement: 规范等距 `shelf_z[i] = −INNER_H/2 + (i+1)·INNER_H/(N+1)`（整腔，open_shelves / shelves_plus_drawer 上腔）；stack 模式只在上半门区等距（`SPLIT_Z..+INNER_H/2`）。
- joint policy: **FIXED 视觉，无 joint**（隔板焊死，靠 `allow_overlap(shelf_i, body_shell)` 表达嵌入支撑）。
- source/gating: parent L267-L275（N=2）、one_shelf L268（N=1）、three_shelf L269-L270（N=3 等距公式）。stack 模式 N_shelf 受上半区高 inequality 回缩。

### 轴 2：drawer_count（底/下半区前拉抽屉栈，各自 PRISMATIC）
- `count_param`: `drawer_count`
- `N_range`: **[1, 4]**（本小类本轴产品域；测试偏小。样本覆盖 {1,3} → drawer_base / drawer_stack；N=2/4 由 stack 循环按 copy 逻辑构造性安全）
- sampling domain（权重档）：由 fitment_choice 派生（conditional）——open_shelves→0；shelves_plus_drawer→1；shelves_plus_drawer_stack→[2,4] 小 N 偏多。
- copied object: `drawer_{i}` 独立 part（共享 `build_drawer_tray` 一体托盘+front+pull）；body `lower_panel` 在 `for i in range(N)` 里铣 N 个抽屉口。
- naming: `drawer_{i}` + joint `body_to_drawer_{i}`（i=0..N−1，自下而上）；单抽屉端点退化为 `drawer` + `body_to_drawer`（无后缀，drawer_base 形态）。
- placement: 下半区等距 `drawer_slot_z(i) = −INNER_H/2 + DRAWER_SLOT_H·(i+0.5)`，`DRAWER_SLOT_H = LOWER_H/N`；joint 原点在 body 前面 (y=BODY_D/2)，该槽 z。
- joint policy: 每 `drawer_{i}` 各自 **+Y PRISMATIC**（axis (0,1,0)，lower=0、upper≈0.080~0.085·drawer_travel_scale，统一 effort/velocity 策略）。
- source/gating: drawer_base L475-L571（N=1）、drawer_stack L463-L513（N≥2 循环 + 铣口循环 + 统一 PRISMATIC 策略）。drawer_count>0 仅在 shelves_plus_drawer(_stack) 下，互斥 gate 由 fitment_choice 强制。

## 拓扑多样性审计

总组合数：door(3) × fitment(3) × shelf_count(N∈[1,6]→测试取 ≥3 档) × drawer_count(由 fitment 派生：open=0 / single=1 / stack∈[2,4]→≥2 档)
= 3 × [ open_shelves(shelf:≥3 档, drawer:1 档=0)  +  shelves_plus_drawer(shelf:≥3 档, drawer:1 档=1)  +  shelves_plus_drawer_stack(shelf:≥2 档, drawer:3 档∈{2,3,4}) ]
= 3 × [ 3 + 3 + 2×3 ] = 3 × 12 = **36 ≥ 10 ✓**（与 source map 预审 27 一致，含两 multiplicity 轴后更高）。

理由：door 3 × fitment 3 已 9 个基底拓扑；叠加 shelf_count（FIXED `shelf_{i}` 数进 slot_choice 命名，如 `shelf_x2`）与 drawer_count（`drawer_{i}` 数进命名，如 `drawer_x3`）后 distinct slot_choice 元组远超 10。50-seed sweep 轻松覆盖。

seed_domain_policy：procedural_first
Procedural Sampling / Sweep Plan：`config_from_seed` 对所有普通 seed（含 seed 0）deterministic 加权采样：① 先抽 door_choice、fitment_choice（均匀/轻加权）；② 按 fitment 解析 drawer_count 域（conditional）并加权抽（小 N 偏多）；③ 抽 shelf_count（小 N 偏多，stack 模式按上半区高 clamp）；④ 抽连续 scale（independent → equation 派生 → inequality 投影）。compatibility matrix 强制：drawer_count>0 ⇔ fitment∈{shelves_plus_drawer, shelves_plus_drawer_stack}；shelves_plus_drawer 锁 drawer_count=1；shelf-vs-drawer 区高与抽屉避铰由 inequality gate 拦非法。无须 curated/modulo 主表。少量 regression overrides 仅在已知失败回归时加（暂无）。
Topology target：1000-seed slot choice tuple distinct 富类别建议 ≥300（report-only）；本类受 door(3)×fitment(3) 基底限制，加 shelf_count(1-6)×drawer_count(0-4) 后 distinct ≈ 3×(6 + 6 + 5×3) ≈ 81 量级，低于 300 属类别天然约束（壁挂柜门型与内腔机构有限），可接受。
Controlled local parameterization：初版含 body_width_scale[0.85,1.20]、body_height_scale[0.85,1.25]、door_thickness_scale[0.85,1.15]、drawer_travel_scale[0.85,1.15]。门宽随 body_width_scale 保形（equation），shelf/drawer 槽 z 随 INNER_H 派生（equation），shelf-vs-drawer 区高与抽屉避铰为 inequality，均在 `resolve_config` clamp/派生/投影，不破坏 hinge mating、drawer slide 接口、shelf 嵌壁支撑或 multiplicity 命名。

| item | policy | validator / viewer focus |
|---|---|---|
| sampler | door/fitment 均匀轻加权；drawer_count 由 fitment 派生加权；shelf_count 小 N 偏多 | slot_choices_for_seed 与 build 选择一致（含 shelf/drawer 计数命名）|
| compatibility matrix | drawer_count>0 ⇔ fitment 含 drawer；single 锁 1；stack ∈[2,4]；shelf-vs-drawer 区高 / 抽屉避铰 inequality fallback 回缩 | no floating（shelf 嵌壁、drawer front 盖口、hinge barrel 接触），no collision（抽屉避铰、门闭合 seat），axis（门 +Z、抽屉 +Y），max multiplicity（shelf≤6 / drawer≤4）|
| controlled local variation | body_width/height_scale、door_thickness_scale、drawer_travel_scale + clamp/equation/inequality | 比例变化不破坏 hinge mating、drawer slide、shelf 嵌壁、joint 原点、类别 identity |
| regression overrides | none | — |
| random sweep | seeds 0-49 初轮；0-999 成熟审计 | 与 contract failures |

| slot | candidate_count | 是否 ≥2 | 是否 ≥3 | 备注 |
|---|---:|---|---|---|
| A (door) | 3 | yes | yes | glass / solid / double |
| B (interior_fitment) | 3 | yes | yes | open_shelves / +drawer / +drawer_stack |
| shelf_count 轴 | N∈[1,6] | yes | yes | FIXED `shelf_{i}` 等距 |
| drawer_count 轴 | N∈[0,4] | yes | yes | +Y PRISMATIC `drawer_{i}`，由 fitment 派生 |

## Validator

- slot_choices_for_seed 返回已实现的 module 名（含 shelf_count / drawer_count 计数编码，如 `(door, glass_front_hinged)`、`(fitment, shelves_plus_drawer_stack)`、`(shelf_count, x3)`、`(drawer_count, x2)`）
- config_from_seed 对所有普通 seed 用 deterministic procedural sampling（seed 0 不特殊）
- compatibility matrix / gating 阻止非法组合（drawer 仅在含 drawer 的 fitment；single 锁 1；stack≥2；shelf-vs-drawer 区高、抽屉避铰 inequality）
- optional regression overrides 稀少且有据（暂无）
- 不无限轮换小型 curated 表作为主 seed domain
- 局部 scale（body_width/height、door_thickness、drawer_travel）被 clamp/派生/投影，不破坏 hinge mating、drawer slide、shelf 嵌壁、joint 原点、multiplicity
- equation/inequality/conditional 约束在 resolve_config 求解（门宽随 body、shelf/drawer 槽 z、区高、避铰、drawer_count 域），不留到 builder 才失败
- 关键 InterfaceSpec / MatingContract 存在：body↔door hinge barrel（captured-pin，grandfather mating）、drawer_front↔front_panel 开口（PRISMATIC seat）
- 关键 joint 类型/轴/range：`body_to_door(_i)` REVOLUTE axis ±Z 0..radians(150)；`body_to_drawer(_i)` PRISMATIC axis +Y 0..~0.08
- copied objects 遵循命名/placement：`shelf_{i}` 等距 FIXED；`drawer_{i}`+`body_to_drawer_{i}` 下半等距 PRISMATIC
- allow_overlap 局部 element-scoped：shelf↔body_shell、drawer_front↔panel、hinge barrel 互嵌、glass/decal 贴附

## Reject cases

- 任一 seed 缺少 `cabinet_body` 单根，或 body 不接地（出现自立腿/底座支腿）→ free-standing，非本类。
- 任一 seed 缺少 ≥1 个 REVOLUTE 门（门退化为 FIXED 或无门）→ 失去封闭机构身份。
- 门轴非竖直 Z（出现水平铰 / 上翻门 / 卷帘）→ 非样本支持的机构，拒绝。
- 抽屉非 +Y PRISMATIC（出现旋转抽屉 / 多轴）→ 偏离样本 joint 策略。
- shelf 或 drawer 漂浮（shelf 不嵌壁、drawer front 不盖口、hinge barrel 不接触）→ 支撑断裂。
- 闭合姿态门面不盖 body 前面，或抽屉闭合不缩入腔内 → 闭合姿态失效。
- stack 模式 shelf_count 溢出上半区 / 抽屉撞 hinge barrel / 抽屉互撞 → inequality 未回缩。
- 把 `door_knob`/`drawer_pull`/`carry_handle`/红十字贴 当独立 slot 或独立 part → 装饰误升级。
- double_doors 两扇不在中线对开（同向 / 不相接）→ 双铰 axis 配置错。

## 与相邻类别的边界

- 不该混入：free-standing / legged medicine cabinet（落地药柜）——本类是**壁挂**，body 贴墙、不出脚不带底座；出现支腿即出类。
- 不该混入：bathroom mirror cabinet / 普通 `cabinet`（通用柜）——本类有 first-aid 身份标志（红十字 + FIRST AID + 急救补给），且默认小尺度壁挂；通用柜无此身份且尺度/语义更宽。
- 不该混入：`container_locker`（金属储物柜组）——locker 是多 bay 并排 + 键盘/挂锁/密码盘 latch 机构 + 通风百叶门面；本类是单箱体、无 latch slot、门面为玻璃/实心/红十字，内腔为药品分层而非储物 bay。
- 不该混入：roll-up / tambour-door 柜——无干净单轴铰接样本，排除。

## 审核记录
| 项 | 结论 |
|---|---|
| reviewer status | pending |
| reviewer notes | （待人工审核：确认 door 3 / fitment 3 候选与两 multiplicity 轴的拆分；确认 shelf_count[1,6] FIXED + drawer_count 由 fitment 派生的 conditional 域；确认壁挂边界排除 free-standing/legged）|

## Module Source Index

| source_id | slot | module | sample_id | model.py 来源 | 采纳用途 |
|---|---|---|---|---|---|
| S1 | A door + B open_shelves | glass_front_hinged + open_shelves | rec_build-...-firs_..._99727092 (parent) | L77-L399 | door 玻璃门基线 + open-shelf 基线 + carcass 共享件（body/handle/hinge/supplies）+ shelf_count N=2 + palette 基线 |
| S2 | A door | solid_panel_hinged | rec_first_aid_cabinet_var_solid_door | L131-L337 | 实心门 part-tree（无 glass，印刷贴）|
| S3 | A door | double_doors | rec_first_aid_cabinet_var_double_doors | L145-L388 | 双扇对开（`door_{i}` 循环 + 双 REVOLUTE ±Z）|
| S4 | B interior_fitment | shelves_plus_drawer | rec_first_aid_cabinet_var_drawer_base | L128-L571 | 单抽屉 PRISMATIC + bottom_front_panel/slide_rail 共享件 + drawer_count=1 端点 |
| S5 | B interior_fitment | shelves_plus_drawer_stack | rec_first_aid_cabinet_var_drawer_stack | L60-L513 | 抽屉栈（upper/lower split + divider + 铣口循环 + `drawer_{i}` 各 +Y PRISMATIC）+ drawer_count 复制契约 |
| S6 | shelf_count 轴 | open_shelves (N=1) | rec_first_aid_cabinet_var_one_shelf | L268-L274 | shelf_count N=1 端点 + 居中单层 placement |
| S7 | shelf_count 轴 | open_shelves (N=3) | rec_first_aid_cabinet_var_three_shelf | L268-L294 | shelf_count 规范等距公式 + copied-object 命名/placement/支撑契约 N=3 |

## 模板实现备注（可选）

- door 三 module 与 fitment 三 module 共享同一组 carcass helper（`build_body_shell` / `build_shelf` / `build_handle` / `build_hinge_knuckles` / `build_supply_block`），按 fitment 决定 body shell 是否带 `divider`+`lower_panel` 铣口（stack 模式）。
- captured-pin overlaps（element-scoped allow_overlap）：body↔door hinge barrel 互嵌 knuckle（grandfather mating，barrel knuckle 交错站位 DOOR/BODY_KNUCKLE_Z）；double_doors 两套各一组。
- drawer slide：`drawer_front`↔`bottom_front_panel`/`lower_panel` 开口缘 element-scoped allow_overlap + `expect_contact`（闭合 seat）；drawer 整体 vs body cavity 用 `allow_overlap(body, drawer_{i})`（抽屉滑入腔内，drawer_stack L755-L759 形态）。
- shelf 嵌壁、supply 落座、glass/decal 贴附均为 element-scoped allow_overlap（按各样本 run_tests 复刻；复制时所有 `shelf_{i}`/`drawer_{i}` 的 allow_overlap 在循环里逐 i 声明）。
- single-drawer（shelves_plus_drawer）退化命名为 `drawer`/`body_to_drawer`（无后缀），与 stack 的 `drawer_{i}`/`body_to_drawer_{i}` 在 slot_choice 命名上统一为 `drawer_x1`/`drawer_xN`。
- stack 模式抽屉宽用 INNER_W−0.008（无侧铰落区，hinge 在上半门区）；single 模式用 0.260 窄盘避开整高左铰——避铰 inequality 按 fitment 分支解析。

## palette_style 颜色方案（per-seed 采样，≥3 目标 4-6，源自 5★ 样本 material）

| palette_style | body 主体 | 门/前板 | 玻璃/trim | 红十字/banner | 五金（handle/hinge/knob/pull）| 来源样本 material |
|---|---|---|---|---|---|---|
| clinical_white_redcross（默认）| 白漆钢 WHITE_METAL (0.92,0.92,0.93) | 白 WHITE_METAL | 淡蓝玻璃 GLASS (0.62,0.74,0.80,0.45) | 急救红 RED (0.78,0.10,0.12) | 亮钢 STEEL (0.70,0.72,0.75) | parent L63-L71 全套 |
| stainless_steel | 拉丝不锈钢 (0.74,0.75,0.77) | 不锈钢 | 灰玻 (0.55,0.60,0.64,0.45) | 急救红 RED | 暗钢 (0.55,0.57,0.60) | STEEL 基重映射 |
| emergency_green | 急救绿柜体 (0.16,0.45,0.30) | 绿 + 白 trim | 淡蓝玻璃 GLASS | 白十字 WHITE_TRIM (0.97,0.97,0.97) | 亮钢 STEEL | RED/绿急救色域重映射 |
| industrial_grey | 哑光灰 (0.55,0.56,0.58) | 灰 | 灰玻 | 急救红 RED | 暗钢 | SHELF_GREY/STEEL 基重映射 |
| vintage_enamel | 米白搪瓷 (0.90,0.88,0.82) | 米白 + 红边 | 老玻璃 (0.66,0.72,0.70,0.45) | 暗红十字 (0.62,0.10,0.12) | 黄铜 (0.72,0.60,0.32) | 基于 5★ 结构 + 现实搪瓷急救箱色域重映射 |

palette_style 仅改 material rgba，不改拓扑（按 §7 choice 类型 per-seed 采样）。stainless_steel / emergency_green / industrial_grey / vintage_enamel 为基于 5★ 结构（WHITE_METAL/RED/GLASS/STEEL/SHELF_GREY 色域）的现实重映射，凑足 5 档真实壁挂急救柜配色（white+red-cross / stainless / green-emergency / grey + 搪瓷）。
