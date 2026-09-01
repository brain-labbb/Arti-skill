# Modular Spec — Military / Rifle (`rifle`)

## 元信息
| 项 | 值 |
|---|---|
| slug | `rifle` |
| template path | `agent/templates/Military_Rifle.py` |
| test path (optional) | `tests/agent/test_rifle_template.py` |
| stage | `SPEC_ONLY_DRAFT` |
| status | `pending` |
| __modular__ | `True` |
| pattern | `mixed` (parallel-children slots off a fixed receiver spine + handguard-rail multiplicity) |

## 5 星样本阅读摘要
| 项 | 值 |
|---|---|
| five_star_total | 13 |
| read_count | 13 |
| read_scope | all 5-star samples in this category (1 parent M4 carbine + 12 sweep variants) |
| source_index_policy | only adopted module sources are indexed below |

13 个 5★ 样本全部读完（无抽样）。母资产 `rec_model-an-m4-style-military-carbine-rifle-all-mat...497c83bd` 是唯一独立全机母件，单文件覆盖四个 slot 的 base candidate（collapsible buttstock / quad-Picatinny handguard / red-dot reflex sight / birdcage muzzle）。其余 12 个是从母件派生的 single-slot swap 变体，每个只替换一个 slot，其余 spine + 三个 slot 与母件逐行一致。因此每个 candidate 的真实 model.py:Lx-Ly 都来自其对应记录的 revision rev_000001/model.py。

**不变的结构脊（spine，所有样本逐行一致，模板里写成固定结构而非 slot）**：`receiver` part 内含 `upper_receiver`/`rail_base`/`rail_ribs`/`lower_receiver`/`magwell`/`guard_bar`+`guard_front_post`+`guard_rear_post`/`buffer_tube`+`castle_nut`/`grip_plate`+`grip_body`/`delta_ring`（母件 L66-L194）；固定结构 `front_sight_tower`（L292-L321）与 `vertical_foregrip`（L367-L392）。脊上挂 5 个不变的非固定关节：`trigger_pull` REVOLUTE +Y ~25°（L487-L496）、`charging_handle_slide` PRISMATIC -X 0.07 m（L458-L467）、`magazine_release` PRISMATIC 沿前倾 magwell 轴 0.06 m（L550-L559）、`safety_selector_rotate` REVOLUTE -Y 90°（L582-L593）、以及 buttstock slot 自己的关节（随 module 变）。Bore 轴 = world +X，muzzle 朝 +X，buttstock 朝 -X；+Z 向上；BORE_Z=0.175。

## 核心身份

`rifle` = 单兵肩射军用步枪 / 卡宾枪（M4/AR 系传统直枪身布局）。物理含义：一根贯穿全长的 bore 轴（world +X）承载 receiver（上下机匣 + magwell + 扳机护圈 + 握把 + buffer tube）→ barrel → handguard 三段；可活动功能件包括 trigger（扣动）、charging handle（拉机柄前后滑动）、magazine（弹匣插拔）、safety selector（保险/快慢机旋转）以及 buttstock（伸缩/折叠）。默认成熟域是标准全长直枪身卡宾枪：弹匣在扳机**前方**插进 magwell，机匣后方接 buffer tube/枪托，机匣顶面 + handguard 是模块化挂载面（Picatinny / M-LOK / KeyMod）。

模板要保留的 articulation 身份：**trigger_pull、charging_handle_slide、magazine_release、safety_selector_rotate 这 4 个脊关节恒在**，外加 buttstock slot 至少一个（slide / fold）或 fixed，optic slot 视 module 可能再加 0-2 个翻转关节（iron sights）。

不该混入的相邻类别：
- **bullpup**（弹匣/枪机在扳机后方、机匣反转布局）——破坏 receiver/magwell/buffer-tube 脊的拓扑，出 `rifle` 默认域。
- **belt-fed / box-feed 机枪（LMG）**（供弹托盘 + 弹链替代 magwell 弹匣）——出类别。
- **handgun / pistol**（无 buttstock、无 buffer tube、无全长 barrel+handguard）——属手枪类。
- **shotgun pump / lever-action**（泵动前握把往复供弹、杠杆供弹）——供弹机构拓扑不同，不在本模板默认域（pump 的往复前护木 ≠ 本模板的 fixed foregrip）。

## 槽位 + 候选模块表

四个 slot 全部以 receiver/barrel 脊为共同 parent（parallel-children），其中 handguard slot 内含一根 Picatinny 顶轨 multiplicity 轴。

### Slot A：buttstock（后方抵肩件，挂在 receiver buffer-tube 轴上）

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| `collapsible` | rec_model-an-m4-style-military-carbine-rifle-all-mat_...497c83bd | L394-L436 | eligible if compatible | part `buttstock`(`stock_body`) + 关节 `stock_slide` PRISMATIC axis(1,0,0) 0.09 m travel；六位聚合物伸缩托套在整根 `buffer_tube` 上前后滑（母件 base） |
| `fixed_A2` | rec_rifle_var_fixedstock | L395-L455 | eligible if compatible | part `buttstock`(`stock_body`/`butt_pad`/`sling_swivel`) + 关节 `receiver_to_stock` FIXED；一体式 A2 实心托壳完全包住 buffer tube + castle nut，尾部橡胶 butt pad，无行程（脊降到 4 个非固定关节） |
| `side_folding` | rec_rifle_var_foldstock | buttstock L415-L473；额外 receiver hinge visuals `buffer_tube_stub`/`hinge_bracket`/`hinge_pin` L146-L179 | eligible if compatible | parts `buttstock`(`stock_knuckle`/`stock_body`) + 关节 `stock_fold` REVOLUTE axis(0,0,-1) ~175°；左壁 hinge bracket 上的骨架臂折平贴 +Y 机匣，buffer tube 缩成 stub。**改动 receiver**：换 stub + 加铰链三件 |
| `pdw_wire` | rec_rifle_var_pdwstock | buttstock L407-L472；helper `_pdw_wire_rail()` L56-L66 | eligible if compatible | part `buttstock`(`stock_collar`/`stock_pad`/`stock_rail_0`/`stock_rail_1`) + 关节 `stock_slide` PRISMATIC axis(1,0,0) 0.09 m；紧凑骨架双钢丝托 + 抵肩垫，clearance-bore 套环在 buffer tube 上滑 |

### Slot B：handguard-surface（前护木挂载面，part `handguard`，关节 `receiver_to_handguard` FIXED）

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| `quad_picatinny` | rec_model-an-m4-style-military-carbine-rifle-all-mat_...497c83bd | L231-L290 | eligible if compatible | `handguard_tube` + `hg_rail_top`/`hg_rail_bottom`/`hg_rail_left`/`hg_rail_right`（rib loop `hg_xs` len N=11）；四面全 Picatinny 肋轨；底轨在前握把夹钳区 `if not (0.178<x<0.222)` 跳格。**承载 multiplicity 轴**（见第 8 节） |
| `smooth_tube` | rec_rifle_var_smoothtube | L232-L250 | eligible if compatible | 仅 `handguard_tube`（圆管 OD 0.027 / 内孔 0.019，无任何 rail 元素，rib loop 移除）；光滑自由浮动管，0 槽 0 轨 |
| `mlok` | rec_rifle_var_mlokrail | handguard L232-L324；helpers `_mlok_slot_cutter`/`_mlok_slot_inset` L252-L269 | eligible if compatible | `handguard_tube` + `mlok_slot_0..11`（3 行 ×4 = 12 个 M-LOK 椭圆槽，行角 90/-90/180°）+ 保留 `hg_rail_top`（顶轨 rib loop len 11）；只有顶 Picatinny 轨幸存 |
| `keymod` | rec_rifle_var_keymod | handguard L250-L371；helper `_keymod_slot()` L56-L71 | eligible if compatible | `handguard_tube` + `keymod_slot_0..~18`（7/行 ×3 面，底行跳前握把区）+ `hg_rail_top_base`/`hg_rail_top_ribs`（顶轨 rib loop len 11）；圆角方管 KeyMod 钥匙孔槽；只有顶轨幸存 |

### Slot C：optic（瞄准系统，挂在 receiver 顶平台轨 / handguard 上）

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| `red_dot` | rec_model-an-m4-style-military-carbine-rifle-all-mat_...497c83bd | L323-L365 | eligible if compatible | part `reflex_sight`(`optic_mount`/`optic_body`/`optic_hood`/`optic_rear_housing`/`optic_knob`) + 关节 `receiver_to_reflex_sight` FIXED；紧凑红点反射镜夹在机匣顶轨上 |
| `scope` | rec_rifle_var_scope | L326-L468；helper `_ring_mount()` inline L389-L429 | eligible if compatible | part `scope`(`scope_body`/`ring_mount_0`/`ring_mount_1`/`elevation_turret`/`windage_turret`) + 关节 `receiver_to_scope` FIXED；长倍率镜：融合镜身 + 物镜钟罩 + 目镜，双环座承载，KnobGeometry 滚花高低/风偏炮塔 |
| `iron_sights` | rec_rifle_var_ironsights | rear_sight L375-L414；front_sight L417-L451 | eligible if compatible | parts `rear_sight`(`rear_sight_frame`/`rear_grip_0..1`) + `front_sight`(`front_sight_body`/`front_knob_0..1`) + 关节 `rear_sight_flip` REVOLUTE axis(0,-1,0) ~90°（parent=receiver）+ `front_sight_flip` REVOLUTE axis(0,-1,0) ~90°（parent=handguard）；翻起式 BUIS 对：后照门觇孔 + 前准星；**无 reflex_sight**；加 2 个翻转关节 |

### Slot D：muzzle（枪口装置，作为 visual 加在 part `barrel` 上，关节 `receiver_to_barrel` FIXED）

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| `birdcage` | rec_model-an-m4-style-military-carbine-rifle-all-mat_...497c83bd | barrel `flash_hider` visual L208-L223 | eligible if compatible | barrel visual `flash_hider`；带槽鸟笼消焰器（圆筒切两条十字缝 + 中孔），无新关节 |
| `suppressor` | rec_rifle_var_suppressor | barrel `suppressor` visual L211-L248；helper `_suppressor_ring()` L214-L223 | eligible if compatible | barrel visual `suppressor`（替换 flash_hider）；~0.19 m 长消音器罐：螺纹 collar + 光滑外管 + endcap + 内部 baffle 环，无新关节 |
| `brake` | rec_rifle_var_brake | barrel `muzzle_brake` visual L266-L275；helper `_muzzle_brake_body()` L56-L110 | eligible if compatible | barrel visual `muzzle_brake`（替换 flash_hider）；~0.054 m 多孔补偿制退器：通膛体 + 螺纹 collar + 6 侧孔/baffle 切口，无新关节 |

> 所有 slot 都有 ≥3 个结构不同的 candidate，无 single-candidate 降级。

## 槽位图（slot graph）

pattern: mixed（parallel-children off fixed spine + 1 multiplicity axis inside Slot B）

```
                         [FIXED SPINE: receiver(脊) ── receiver_to_barrel FIXED ──> barrel]
                                 │                                    │
     ┌───────────────┬──────────┴───────────┬──────────────┐         │ (barrel 末端)
     │               │                       │              │         │
 receiver_to_      receiver_to_         receiver_to_     buffer-tube  │
 handguard FIXED   reflex_sight/scope   <selector/trig/  axis 后端    │
     │             FIXED (red_dot,      charging/mag      │           │
     ▼             scope) │ rear/front  脊关节，恒在)      ▼           ▼
  Slot B           _sight_flip REVOLUTE                Slot A      Slot D
  handguard        (iron_sights)                       buttstock   muzzle
  ─ multiplicity:  │                                    │           (barrel 上的 visual,
    hg_rail_top    ▼                                    ▼            无关节)
    N ribs         Slot C                          stock_slide PRISMATIC /
                   optic                           stock_fold REVOLUTE /
                                                   receiver_to_stock FIXED
```

接口点位与跨 slot joint：
- **Slot B（handguard）→ receiver**：`receiver_to_handguard` FIXED。mating face = handguard 管根抵 `delta_ring` 环（母件 L182-L194，bore 轴 +X 方向 contact plane）。upstream interface = handguard 管根的 -X face（normal 轴 X）；anchor 法向分量 0。
- **Slot C（optic）→ receiver / handguard**：
  - `red_dot` / `scope`：`receiver_to_reflex_sight` / `receiver_to_scope` FIXED，mating face = optic mount 底面坐落机匣顶 `rail_base`/`rail_ribs`（z 法向，contact plane at z≈0.205）。
  - `iron_sights`：`rear_sight_flip` REVOLUTE axis(0,-1,0) parent=receiver（pivot 在机匣顶轨后段）+ `front_sight_flip` REVOLUTE axis(0,-1,0) parent=handguard（pivot 在 handguard 前端顶轨）。两关节 pivot 都落在真实铰链硬件几何上。
- **Slot A（buttstock）→ receiver**：挂在 buffer-tube 轴（world -X 后端，z=BORE_Z）。
  - `collapsible`/`pdw_wire`：`stock_slide` PRISMATIC axis(1,0,0)，origin=(-0.240,0,0.170)，lower=0 upper=0.09。
  - `fixed_A2`：`receiver_to_stock` FIXED。
  - `side_folding`：`stock_fold` REVOLUTE axis(0,0,-1)，origin=(-0.115,0.024,0.170)（左壁 hinge pin），lower=0 upper=175°；**该 module 还重写 receiver 后端**（buffer_tube→buffer_tube_stub + hinge_bracket + hinge_pin）。
- **Slot D（muzzle）→ barrel**：纯 visual 加在 barrel part 末端（world +X ≈0.40-0.59），无新关节，随 `receiver_to_barrel` FIXED 一起固定。

互斥/派生：
- Slot C `iron_sights` 与 `red_dot`/`scope` 互斥（前者无 `reflex_sight`/`scope` part，且额外引入 2 个翻转关节，joint 计数随之变化）。
- Slot A `side_folding` 派生 receiver 改动（stub + 铰链三件），其余 Slot A module 用全长 buffer_tube + castle_nut。模板需在 receiver builder 里按 Slot A choice 分支这一段。
- Slot B 的 multiplicity（顶轨 rib 数 N）仅在 `hg_rail_top` 存在的 module（quad_picatinny / mlok / keymod）生效；`smooth_tube` 无顶轨 → N 轴对它不适用（conditional，见第 7/8 节）。

## 每槽位 Module Emits / Interfaces

### Slot A / module collapsible
| emits | 描述 | 来源 |
|---|---|---|
| parts | `buttstock`(`stock_body`) | 母件 / model.py:L394-L426 |
| internal joints | `stock_slide` PRISMATIC axis(1,0,0) lower=0 upper=0.09 | 母件 / model.py:L427-L436 |
| upstream interface | receiver buffer-tube 轴后端，origin(-0.240,0,0.170)，face 法向 X | 母件 / model.py:L432 |
| downstream interface | 无（链尾） | — |

### Slot A / module fixed_A2
| emits | 描述 | 来源 |
|---|---|---|
| parts | `buttstock`(`stock_body`/`butt_pad`/`sling_swivel`) | rec_rifle_var_fixedstock / model.py:L398-L449 |
| internal joints | `receiver_to_stock` FIXED（脊降到 4 个非固定关节） | rec_rifle_var_fixedstock / model.py:L450-L455 |
| upstream interface | 套住整根 buffer_tube + castle_nut；FIXED support | rec_rifle_var_fixedstock / model.py:L450-L455 |
| downstream interface | 无 | — |

### Slot A / module side_folding
| emits | 描述 | 来源 |
|---|---|---|
| parts | `buttstock`(`stock_knuckle`/`stock_body`)；并改写 receiver 后端 `buffer_tube_stub`/`hinge_bracket`/`hinge_pin` | rec_rifle_var_foldstock / model.py:L415-L463、L146-L179 |
| internal joints | `stock_fold` REVOLUTE axis(0,0,-1) origin(-0.115,0.024,0.170) lower=0 upper=175° | rec_rifle_var_foldstock / model.py:L464-L473 |
| upstream interface | 左壁 hinge bracket/pin（pivot 落在 hinge_pin 几何上） | rec_rifle_var_foldstock / model.py:L161-L179 |
| downstream interface | 无 | — |

### Slot A / module pdw_wire
| emits | 描述 | 来源 |
|---|---|---|
| parts | `buttstock`(`stock_collar`/`stock_pad`/`stock_rail_0`/`stock_rail_1`)；helper `_pdw_wire_rail()` | rec_rifle_var_pdwstock / model.py:L407-L461、L56-L66 |
| internal joints | `stock_slide` PRISMATIC axis(1,0,0) lower=0 upper=0.09 | rec_rifle_var_pdwstock / model.py:L463-L472 |
| upstream interface | clearance-bore collar 套 buffer_tube，origin(-0.240,0,0.170) | rec_rifle_var_pdwstock / model.py:L463-L472 |
| downstream interface | 无 | — |

### Slot B / module quad_picatinny
| emits | 描述 | 来源 |
|---|---|---|
| parts | `handguard`(`handguard_tube`/`hg_rail_top`/`hg_rail_bottom`/`hg_rail_left`/`hg_rail_right`) | 母件 / model.py:L231-L284 |
| internal joints | 无（part 内全 visual） | — |
| upstream interface | `receiver_to_handguard` FIXED，管根抵 delta_ring（-X face） | 母件 / model.py:L285-L290 |
| downstream interface | front sight tower 抵接面（barrel 上的固定结构）+ foregrip 夹底轨 | 母件 / model.py:L292-L321、L367-L392 |
| multiplicity | `hg_rail_top` 顶轨 rib 数 N（见第 8 节） | 母件 / model.py:L245-L253 |

### Slot B / module smooth_tube
| emits | 描述 | 来源 |
|---|---|---|
| parts | `handguard`(`handguard_tube` 仅) | rec_rifle_var_smoothtube / model.py:L232-L244 |
| internal joints | 无 | — |
| upstream interface | `receiver_to_handguard` FIXED | rec_rifle_var_smoothtube / model.py:L245-L250 |
| downstream interface | 无顶轨 → multiplicity N 轴不适用 | rec_rifle_var_smoothtube / model.py:L232-L244 |

### Slot B / module mlok
| emits | 描述 | 来源 |
|---|---|---|
| parts | `handguard`(`handguard_tube`/`mlok_slot_0..11`/`hg_rail_top`)；helpers `_mlok_slot_cutter`/`_mlok_slot_inset` | rec_rifle_var_mlokrail / model.py:L232-L317、L252-L269 |
| internal joints | 无 | — |
| upstream interface | `receiver_to_handguard` FIXED | rec_rifle_var_mlokrail / model.py:L319-L324 |
| downstream interface | 顶轨 `hg_rail_top` 存在 → 承载 multiplicity N | rec_rifle_var_mlokrail / model.py:L304-L317 |

### Slot B / module keymod
| emits | 描述 | 来源 |
|---|---|---|
| parts | `handguard`(`handguard_tube`/`keymod_slot_0..~18`/`hg_rail_top_base`/`hg_rail_top_ribs`)；helper `_keymod_slot()` | rec_rifle_var_keymod / model.py:L250-L371、L56-L71 |
| internal joints | 无 | — |
| upstream interface | `receiver_to_handguard` FIXED | rec_rifle_var_keymod / model.py:L373-L378 |
| downstream interface | 顶轨 `hg_rail_top_ribs` 存在 → 承载 multiplicity N | rec_rifle_var_keymod / model.py:L361-L371 |

### Slot C / module red_dot
| emits | 描述 | 来源 |
|---|---|---|
| parts | `reflex_sight`(`optic_mount`/`optic_body`/`optic_hood`/`optic_rear_housing`/`optic_knob`) | 母件 / model.py:L323-L359 |
| internal joints | 无 | — |
| upstream interface | `receiver_to_reflex_sight` FIXED，mount 底面坐机匣顶轨（z 法向 z≈0.205） | 母件 / model.py:L360-L365 |
| downstream interface | 无 | — |

### Slot C / module scope
| emits | 描述 | 来源 |
|---|---|---|
| parts | `scope`(`scope_body`/`ring_mount_0`/`ring_mount_1`/`elevation_turret`/`windage_turret`)；helper `_ring_mount()` | rec_rifle_var_scope / model.py:L326-L461、L389-L429 |
| internal joints | 无（KnobGeometry turrets 是 visual，不是关节） | — |
| upstream interface | `receiver_to_scope` FIXED，双环座坐机匣顶轨 | rec_rifle_var_scope / model.py:L463-L468 |
| downstream interface | 无 | — |

### Slot C / module iron_sights
| emits | 描述 | 来源 |
|---|---|---|
| parts | `rear_sight`(`rear_sight_frame`/`rear_grip_0..1`) + `front_sight`(`front_sight_body`/`front_knob_0..1`)；无 reflex_sight | rec_rifle_var_ironsights / model.py:L375-L402、L417-L440 |
| internal joints | `rear_sight_flip` REVOLUTE axis(0,-1,0) lower=0 upper=π/2 parent=receiver；`front_sight_flip` REVOLUTE axis(0,-1,0) lower=0 upper=π/2 parent=handguard | rec_rifle_var_ironsights / model.py:L403-L414、L441-L451 |
| upstream interface | rear pivot 在机匣顶轨后段、front pivot 在 handguard 前端顶轨（pivot 落在铰链几何上） | rec_rifle_var_ironsights / model.py:L403-L414、L441-L451 |
| downstream interface | 无 | — |

### Slot D / module birdcage
| emits | 描述 | 来源 |
|---|---|---|
| parts | barrel visual `flash_hider`（不独立成 part） | 母件 / model.py:L208-L223 |
| internal joints | 无（随 `receiver_to_barrel` FIXED） | 母件 / model.py:L224-L229 |
| upstream interface | barrel 末端 muzzle 面（world +X ≈0.40） | 母件 / model.py:L208-L223 |
| downstream interface | 无 | — |

### Slot D / module suppressor
| emits | 描述 | 来源 |
|---|---|---|
| parts | barrel visual `suppressor`（替换 flash_hider）；helper `_suppressor_ring()` | rec_rifle_var_suppressor / model.py:L242-L248、L214-L223 |
| internal joints | 无 | — |
| upstream interface | barrel 末端 muzzle 面 | rec_rifle_var_suppressor / model.py:L242-L248 |
| downstream interface | 无 | — |

### Slot D / module brake
| emits | 描述 | 来源 |
|---|---|---|
| parts | barrel visual `muzzle_brake`（替换 flash_hider）；helper `_muzzle_brake_body()` | rec_rifle_var_brake / model.py:L271-L275、L56-L110 |
| internal joints | 无 | — |
| upstream interface | barrel 末端 muzzle 面 | rec_rifle_var_brake / model.py:L271-L275 |
| downstream interface | 无 | — |

## 参数范围汇总
| 参数 | 类型 | 取值范围 / 候选值 | 标称默认 | 约束类型 | 约束 / 函数 | 来源 |
|---|---|---|---|---|---|---|
| `buttstock_choice` | enum | `collapsible` / `fixed_A2` / `side_folding` / `pdw_wire` | — | choice | deterministic procedural sampler；`side_folding` 同时触发 receiver 后端 stub+hinge 分支 | Slot A table |
| `handguard_choice` | enum | `quad_picatinny` / `smooth_tube` / `mlok` / `keymod` | — | choice | sampler | Slot B table |
| `optic_choice` | enum | `red_dot` / `scope` / `iron_sights` | — | choice | sampler；`iron_sights` 加 2 翻转关节、删 reflex/scope | Slot C table |
| `muzzle_choice` | enum | `birdcage` / `suppressor` / `brake` | — | choice | sampler | Slot D table |
| `top_rail_rib_count` | int (N) | [6, 30] | 11 | conditional | 仅当 `handguard_choice ∈ {quad_picatinny, mlok, keymod}` 时有效；`smooth_tube` 时忽略。等距：start x=0.112，首末跨距固定 0.178，step=0.178/(N-1) | 母件 L245 + rails7/rails18 |
| `palette_style` | enum | `black` / `fde` / `od_green` / `two_tone` / `gray_wolf` | `black` | choice | 仅改 material rgba，不改拓扑 | 母件 L59-L64 |
| `stock_travel_scale` | float | [0.85, 1.15] | 1.0 | independent | 仅对 `collapsible`/`pdw_wire` 的 `stock_slide` upper（0.09×scale）；clamp 到 [0.06,0.11]；`fixed_A2`/`side_folding` 无效 | 母件 L435 |
| `handguard_len_scale` | float | [0.92, 1.08] | 1.0 | independent | handguard 管长 0.192×scale；驱动等距 rib span（见 inequality） | 母件 L233-L239 |
| `optic_height_scale` | float | [0.95, 1.10] | 1.0 | independent | optic mount/body z 偏移微调；clamp 保持总高 ≤0.27 | 母件 L327-L353 |
| (—) | constraint | — | — | inequality | rib span ≤ handguard_len（`0.178·handguard_len_scale ≤ 0.192·handguard_len_scale − 2·端部余量`）；违反则按比例回缩 rib span，不缩 N | 接口 / clearance |
| (—) | constraint | — | — | inequality | optic 顶 z（受 optic_height_scale）+ scope 环座 ≤ 0.27 m 总高上限；违反则回缩 optic_height_scale | 母件测试 L698-L702 |
| (—) | constraint | — | — | inequality | `side_folding` 折叠位 stock 不得穿 magazine/optic（fold pose AABB 不重叠 magazine/optic part）；违反则降级到 `collapsible` 并记日志 | rec_rifle_var_foldstock L464-L473 |

连续尺度默认相互独立；唯一耦合通过上面三条 inequality 显式声明。采样契约：先采 `stock_travel_scale`/`handguard_len_scale`/`optic_height_scale`（independent，范围内均匀）→ 无 equation 派生 → 用 inequality 把 rib span / 总高 / 折叠 clearance 投影回可行域（违反时回缩或对 fold 降级）→ `top_rail_rib_count` 的有效性按 `handguard_choice`（conditional）在采样前解析。

## Multiplicity / Copy Logic

单一 multiplicity 轴：**handguard 顶轨 Picatinny rib 数 N**（loop 已存在于母件 L245 与 rails7/rails18）。

- `count_param`: `top_rail_rib_count`（即 `hg_xs` / `hg_rib_xs` 顶轨 rib loop 的长度 N）
- `N_range`（本小类本轴产品域）: **[6, 30]**。已覆盖样本 {7, 11, 18}：rec_rifle_var_rails7 (N=7) / 母件 (N=11) / rec_rifle_var_rails18 (N=18)。
- sampling domain（权重档）: 小 N 偏多，大 N 稀有。建议加权 ~ N∈[9,15] 高频（真实卡宾枪典型轨长），N∈[6,8] 与 N∈[16,30] 各占小尾部；模板做 per-轴加权采样、clamp 到 [6,30]、sweep 上限设 30。
- copied object: 一根 Picatinny rib（`_box_compound` 的一个 box；top/bottom `(0.0075,0.034,0.0075)`，left/right `(0.0075,0.0075,0.034)`；rails18 把 rib 缩到 `(0.005,0.034,0.006)` 以保大 N 视觉可分——模板可按 N 派生 rib 厚度：N>15 时缩窄到 ~0.005）。
- naming: rib **不是**每根一个 named element——它们按面打包成单个 compound mesh（`hg_rail_top` / `hg_rail_bottom` / `hg_rail_left` / `hg_rail_right`，keymod 是 `hg_rail_top_base`+`hg_rail_top_ribs`）。loop 只改 `hg_xs` 的 x-center 列表 `[0.112 + i*(0.178/(N-1)) for i in range(N)]`。
- placement: 沿 bore +X 等距，固定起点 x=0.112、固定首末跨距 0.178（step=0.178/(N-1)）；底轨复制跳过前握把夹钳区 `if not (0.178<x<0.222)`。FIXED（顶/底/左/右轨随 handguard part 一起 `receiver_to_handguard` FIXED 到 receiver）。
- joint policy: 整个 `handguard` part（及所有 rib 复制）刚性 FIXED 到 receiver；rib 自身不带关节。
- source/gating: 该轴**仅对** `handguard_choice ∈ {quad_picatinny, mlok, keymod}` 生效（顶轨存在）；`smooth_tube` 时 N 轴被 gate 掉（conditional），module_topology 仍由 4 个 enum slot 提供。

> 排除项（不做 multiplicity / 不做独立 slot）：magazine straight↔curve 是连续 cant 参数，不枚举为变体；bullpup / belt-fed 出类别（见核心身份）。

## 拓扑多样性审计

总组合数（不含 N、不含连续 scale）：A × B × C × D = 4 × 4 × 3 × 3 = **144** 个 enum 组合。
把 handguard N 轴（N∈[6,30]，但仅在 3/4 个 handguard module 上有效）算进去：有顶轨的组合每个再乘 ~25 个 distinct N → 远超门槛。

理由：仅 enum 组合就有 144 个 distinct slot_choice 元组；slot_choices_for_seed 把 (buttstock, handguard, optic, muzzle, top_rail_rib_count) 编进元组，N 变化也算 distinct。即便只采样 50 个 seed 也极易跨过 10 distinct；1000-seed 按 ≥300 富类别建议线观察，report-only。

seed_domain_policy：procedural_first
Procedural Sampling / Sweep Plan：`config_from_seed(seed)` 用 deterministic procedural sampling——对每个 slot 做加权 enum 抽样（4 个 slot 各自独立），handguard N 轴按上面权重档加权抽（仅当 handguard 有顶轨）。compatibility gating：(1) `iron_sights` 与 `red_dot`/`scope` 天然互斥（同一 enum slot 的不同 candidate，sampler 只选一个，无非法并存）；(2) `smooth_tube` 选中时 N 轴不采样（conditional 解析），slot_choices 里 top_rail_rib_count 记为 0 / N/A；(3) `side_folding` 选中时 receiver builder 走 stub+hinge 分支，fold pose clearance 不过则降级 `collapsible`（sparse，记日志）。无小型 curated/modulo 主表。Topology target：1000-seed distinct 富类别建议 ≥300（report-only）（144 enum 组合 × N 变化足以达到）。regression overrides：仅在某 (buttstock=side_folding, optic=scope/iron_sights) 组合出现 fold-clearance 回归时加 sparse override seed，并注明审核理由。

Controlled local parameterization：初版应含 `stock_travel_scale`[0.85,1.15]、`handguard_len_scale`[0.92,1.08]、`optic_height_scale`[0.95,1.10] 三个连续 scale，全部在 `resolve_config` clamp/投影，受第 7 节三条 inequality 约束（rib span ≤ handguard 长、总高 ≤0.27、fold clearance），不破坏 InterfaceSpec / FIXED support / N multiplicity。主多样性来自 4 enum slot + N 轴，不把每个小零件做自由随机。

| item | policy | validator / viewer focus |
|---|---|---|
| sampler | 4 个 enum slot 独立加权抽 + handguard N 轴加权（conditional on 顶轨存在） | slot_choices_for_seed 与 build choices 一致（含 N） |
| compatibility matrix | iron_sights⊕(red_dot/scope) 互斥（同 slot）；smooth_tube→N 轴 off；side_folding→receiver stub+hinge 分支 + fold-clearance gate→fallback collapsible | 无 floating、无穿模（fold pose vs mag/optic）、关节轴/range 正确、N≤30、bulky scope/suppressor 不撞 optic |
| controlled local variation | stock_travel_scale / handguard_len_scale / optic_height_scale，全 clamp+投影 | 比例变化不破坏 FIXED 接口、clearance、关节 origin（脊关节 origin 落在真实硬件上）、类别 identity |
| regression overrides | none（除非 fold×scope/iron clearance 回归，届时 sparse + 注明） | 仅已知失败回归 |
| random sweep | seeds 0-49 初轮；0-999 成熟审计 | 与 contract failures |

| slot | candidate_count | 是否 ≥2 | 是否 ≥3 | 备注 |
|---|---:|---|---|---|
| A buttstock | 4 | yes | yes | |
| B handguard-surface | 4 | yes | yes | 承载 N multiplicity 轴 |
| C optic | 3 | yes | yes | iron_sights 引入额外关节 |
| D muzzle | 3 | yes | yes | barrel 上的 visual，无关节 |

## Validator

- slot_choices_for_seed 返回已实现的 module names（含 (buttstock, handguard, optic, muzzle, top_rail_rib_count) 元组）
- config_from_seed 对所有普通 seed（含 seed 0）用 deterministic procedural sampling
- compatibility gating 阻止非法组合：iron_sights 不与 red_dot/scope 并存；smooth_tube 时不采 N；side_folding 走 receiver 分支且 fold-clearance fallback
- optional regression overrides 稀疏且有理由（仅 fold-clearance 回归）
- 不靠小型 curated 表当主 seed domain
- 受控连续 scale 全部 clamp/投影，不破坏接口、clearance、关节 origin、N multiplicity
- 跨部件 scale 依赖（rib span ≤ handguard 长、总高 ≤0.27、fold clearance）在 `resolve_config` 求解，不留到 builder 失败
- 关键 InterfaceSpec / MatingContract 点位存在：handguard 抵 delta_ring、optic mount 坐顶轨、buffer-tube 轴 stock 接口、barrel muzzle 面
- 关键关节类型/轴/range 正确：trigger_pull REVOLUTE +Y 0~25°；charging_handle_slide PRISMATIC -X 0~0.07；magazine_release PRISMATIC 前倾轴 0~0.06（axis[2]<-0.9, axis[0]>0.1）；safety_selector_rotate REVOLUTE -Y 0~90°；stock_slide PRISMATIC +X 0~0.09（collapsible/pdw）或 stock_fold REVOLUTE (0,0,-1) 0~175°（fold）或 FIXED（A2）；iron_sights rear/front_sight_flip REVOLUTE (0,-1,0) 0~90°
- copied object（顶轨 rib）遵守 naming（compound-per-face，不是每 rib 一 element）与等距 placement（start 0.112, span 0.178, step 0.178/(N-1)，底轨跳 0.178<x<0.222）
- element-scoped allow_overlap：buffer_tube↔stock_body（滑套）、upper_receiver↔handle_shaft（拉机柄滑槽）、magwell↔mag_top + lower_receiver↔mag_top（弹匣插座）；fold variant 加 hinge_pin↔stock_knuckle

## Reject cases

- bullpup 布局：弹匣/枪机移到扳机后方、机匣反转 → 破坏 receiver/magwell/buffer-tube 脊，超出 `rifle` identity。
- belt-fed / box-feed LMG：供弹托盘 + 弹链替代 magwell → 出类别。
- 把 magazine straight↔curve 当 slot/multiplicity 枚举（它是连续 cant 参数）。
- `smooth_tube` 仍然采 N 顶轨 rib（无顶轨却复制 rib → floating islands）。
- `iron_sights` 与 `red_dot`/`scope` 同时出现（双重 optic 占同一顶轨、joint 计数混乱）。
- N 超出 [6,30] 或 rib span 超过 handguard 管长（rib 悬空 / 穿出管端 → island / clearance fail）。
- `side_folding` 用全长 buffer_tube（应换 stub）或折叠位 stock 穿 magazine/optic（fold-clearance）。
- 把每个连续 scale 当独立自由变量乱抽，导致总高 >0.27 或脊关节 origin 偏离真实硬件 >15 mm（baseline articulation-origin tol）。
- muzzle device 做成独立 part 加新关节（应是 barrel 上的 visual，随 `receiver_to_barrel` FIXED）。

## 与相邻类别的边界

- 不该混入：**bullpup rifle**（弹匣/action 在扳机后、机匣反转拓扑破坏 receiver/buffer-tube/magwell 脊）。
- 不该混入：**belt-fed / box-feed LMG**（feed tray + 弹链替代 magwell magazine，供弹拓扑不同）。
- 不该混入：**handgun / pistol**（无 buttstock、无 buffer tube、无全长 barrel+handguard）。
- 不该混入：**pump shotgun / lever-action**（往复前护木 / 杠杆供弹机构，与本模板 fixed foregrip + magwell 拓扑不同）。

## 审核记录
| 项 | 结论 |
|---|---|
| reviewer status | pending |
| reviewer notes | — |

## 模板实现备注（可选）

- 共享 helper：`_box_compound`（rib/rail 复制，母件 L48-L53）；顶轨 rib loop 在 quad_picatinny/mlok/keymod 三个 module 间共享同一 `hg_xs` 生成器（按 N + handguard_len_scale 解析）。
- muzzle slot 三个 candidate 共享「barrel 上的单一 visual + 可选 helper」模式：birdcage 内联、suppressor `_suppressor_ring()`、brake `_muzzle_brake_body()`；都不发关节。
- `side_folding` 是唯一会改写 upstream（receiver）的 Slot A module：receiver builder 必须按 buttstock_choice 在 buffer_tube（全长 + castle_nut）与 buffer_tube_stub + hinge_bracket + hinge_pin 之间分支；hinge_pin↔stock_knuckle 需 element-scoped allow_overlap，stock_fold 关节 origin 落在 hinge_pin 几何上（满足 15 mm baseline）。
- captured-pin / 滑套 overlap（element-scoped allow_overlap，写进 run_rifle_tests）：buffer_tube↔stock_body、upper_receiver↔handle_shaft、magwell↔mag_top、lower_receiver↔mag_top。
- mlok/keymod 槽是 cut 出来的开口 + inset visual，注意每行槽自身的 island（inset 必须贴管壁，否则 per-part island）；keymod 底行在前握把区跳格要与底轨跳格区一致。
- scope 的 KnobGeometry 高低/风偏炮塔是 visual（参考 MEMORY: 轴对称 KnobGeometry 易 fail AABB spin check，但此处是 FIXED visual 不旋转，无 spin 检查风险）。
