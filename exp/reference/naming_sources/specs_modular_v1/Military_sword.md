# Modular Spec — Military / sword

## 元信息
| 项 | 值 |
|---|---|
| slug | `sword` |
| template path | `agent/templates/Military_sword.py` |
| test path (optional) | `tests/agent/test_sword_template.py` |
| stage | `SPEC_ONLY_DRAFT` |
| status | `pending` |
| __modular__ | `True` |
| pattern | `mixed` (parallel_children blade+hilt over a shared scabbard chassis + multiplicity on suspension rings) |

`pattern=mixed`：核心是一把插在鞘里的剑。`scabbard`（鞘）是接地 chassis，`sword`（剑体 = blade+hilt 同一刚体 part）通过 **`sword_draw` PRISMATIC**（沿 +X 抽出）挂在鞘上；blade-profile 与 hilt 是装到同一 `sword` part 上的两个可替换功能层（parallel children，共用同一 root part frame，不串成链）；suspension rings 是鞘上 N 个同构复制的 REVOLUTE 吊环（multiplicity）。

---

## 5 星样本阅读摘要
| 项 | 值 |
|---|---|
| five_star_total | 8 |
| read_count | 8 |
| read_scope | all 5-star samples in this category (1 parent + 7 variants) |
| source_index_policy | only adopted module sources are indexed below |

阅读结论（全部 8 个样本逐行读完）：

- **共享不变量（所有 8 个样本完全一致）**：`scabbard` 接地 chassis（throat + chape 底面坐落在 z≈0，整鞘沿 +X 水平躺）；鞘体由 `_loft` 在 YZ 平面沿 +X ruled-loft 出截面，中心高度 `ZC≈0.0147`，鞘嘴 `MOUTH_X=0.50`；鞘是真正中空 —— 一条 blade cavity（`_loft(CAV_SECS)`）被 boolean-cut 穿过 burl-wood `body`、brass `chape`、brass `throat`；`sword` part 含 blade + blade_spine + guard + grip(core/lower/upper) + gold collar beads + pommel(+finial)；唯一移动主结构 = `sword_draw` PRISMATIC (+X, lower=0 upper=0.50) + 每个吊环一个 REVOLUTE pivot（+Y 轴，±60°）。整鞘 ~0.75 m。每个模型 **恰好一个 prismatic**（已逐个核对）。
- **blade-profile 轴（4 个结构不同的来源）**：leaf-shaped 双刃（parent，polyline 经 shoulder 收尖）、straight 平行双刃（直边到近尖才收）、curved single-edged saber（抛物线侧弯 `_sword_bow`，刃在 +Y、脊在 −Y，**鞘体/cavity 同弯 `_scabbard_bow`**）、broad triangular 单直收尖 + 凸起中脊 `_blade_spine_solid`。
- **hilt 轴（3 个结构不同的来源）**：box_guard + 扁椭球 amber bead_pommel + brass 球 finial（parent）；cruciform 十字格 `_crossguard_solid` + 扁圆轮 disc_pommel `_disc_pommel_solid` + peening collar；slim crossguard + 扫掠 knuckle_guard 管 (`tube_from_spline_points`) + 高八面体 scent-stopper pommel `_scent_stopper_pommel`。
- **multiplicity 轴（3 个 N 样本）**：N=2（parent，手工成对 front/rear，反侧挂载）、N=4（`band_{i}` 循环 + `_lug_positions` helper，全 +Y 侧，带 band wrap）、N=6（`ring_{i}` 循环 + `_ring_positions()`，交替 +Y/−Y 侧）。三者的吊环都是 REVOLUTE +Y ±60°、`RING_HANG` 偏轴下挂（让旋转可验证）、torus 套在 pin 上（captured contact）。

---

## 核心身份

`sword` = 一把带鞘（scabbard）的冷兵器剑。物理含义：刚性钢刃 + 装饰握柄（hilt = 护手 guard + 握把 grip + 柄头 pommel）组成的 `sword` 主体，**滑动插入/抽出**一个接地的中空鞘；鞘外侧带若干可摆动的悬挂吊环（suspension rings）用于佩挂。默认成熟域：单手短剑/直剑/军刀级别，整体 ~0.75 m，水平陈放。

核心运动语义（必须保留，不可退化）：
1. **`sword_draw` PRISMATIC** 是这个类别的身份关节 —— 剑能从鞘里被抽出来。所有 8 个样本都恰好携带一个 prismatic，模板必须对每个 seed 都 emit 它。
2. **blade DRAW from scabbard** —— blade 在鞘内 nests（`expect_within` yz + `expect_overlap` x），半抽时仍居中，全抽（q=0.50）时刃完全越过鞘嘴。
3. **中空鞘 cavity 必须容纳所选 blade profile** —— cavity 截面要逐站宽于 blade，这是本模板的**首要兼容风险**（见兼容矩阵，尤其曲刃 saber）。
4. 吊环 REVOLUTE，偏轴下挂以使摆动几何可见。

不该混入的相邻类别：见末节"与相邻类别的边界"。

---

## 槽位 + 候选模块表

### Slot A：blade_profile（剑刃截面族；决定 `sword` part 的刃几何 + 鞘 cavity/body/throat 的内腔尺寸与是否侧弯）

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| `leaf_double_edge` | rec_model-a-roman-gladius-style-short-sword-sheathed_20260610_080621_578517_42984874 | blade `_blade_solid` L109-L118；直 cavity `CAV_SECS` L54-L58；直 `_loft` L99-L106 | eligible if compatible | 叶形双刃：polyline base 半宽 0.023 → shoulder 半宽 0.015 → 收尖；平直挤出 |
| `straight_double_edge` | rec_sword_var_straightblade | `_blade_solid` L111-L120；uniform cavity `CAV_SECS` L56-L60 | eligible if compatible | 平行双刃：均匀半宽 0.020，仅在 `BLADE_TAPER_X=−0.39` 之后收尖；cavity 均匀宽 |
| `curved_saber` | rec_sword_var_saber | `_curved_blade_solid` L133-L156；`_sword_bow`/`_scabbard_bow` L98-L103；`_offset_loft` L107-L122；弯 cavity L201；弯 body L204-L212；弯 lug L278-L307 | eligible if compatible (gated：仅与直 hilt 组合，见兼容矩阵) | 单刃军刀：抛物线侧弯 `BOW_MAX=0.018`，刃在 +Y、脊在 −Y；**鞘 body+chape+throat+cavity+lug 全部按 `_scabbard_bow(x)` 同步侧弯**以匹配 |
| `broad_triangular` | rec_sword_var_broadsword | `_blade_solid` L117-L127；`_blade_spine_solid` L130-L144；宽 cavity `CAV_SECS` L61-L66；宽 throat L69-L71 | eligible if compatible | 宽三角：guard 端全宽 ~0.076，单段直收到尖；带凸起三角中脊 `blade_spine`；鞘 body/throat/cavity 加宽 |

### Slot B：hilt（护手 + 握把 + 柄头；装到 `sword` part 正 X 半侧）

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| `box_bead` | rec_model-a-roman-gladius-style-short-sword-sheathed_20260610_080621_578517_42984874 | guard Box + relief L237-L249；grip core/lower/upper + gold beads L250-L278；`_pommel_solid` 扁椭球 L126-L134, L279-L284；finial L285-L296 | eligible if compatible | 矩形 box 护手 + relief 板；扁椭球 amber pommel；gold 螺旋珠 collar；brass 球 finial |
| `cruciform_disc` | rec_sword_var_crossguard_disc | `_crossguard_solid` L126-L145, L277-L283；guard grooves L284-L291；quillon tips L292-L299；`_disc_pommel_solid` L148-L171, L336-L342；`pommel_collar` L330-L335 | eligible if compatible | 宽十字格（横杆 + 中央块 + quillon 端帽）；扁圆轮 disc pommel（中央 boss）+ brass peening collar |
| `knuckle_scentstop` | rec_sword_var_basket_scentstop | slim crossguard + relief L278-L291；`tube_from_spline_points` knuckle_guard L294-L305（`KNUCKLE_POINTS` L89-L95）；`_scent_stopper_pommel` 八面体 L161-L173, L339-L344；finial L347-L361 | eligible if compatible | 细 crossguard + brass 扫掠护拳管（拱过握把）+ 高八面体 scent-stopper pommel + finial |

降级说明：blade_profile 取得 **4 个** candidate、hilt 取得 **3 个**，均满足 3-6 目标，无单 candidate slot。`palette_style` 不作为 slot（材质不改拓扑），作为独立 enum 参数。

---

## 槽位图（slot graph）

pattern: `mixed`

```
                            scabbard (grounded chassis：body⊃cavity / chape / throat / N×lug)
                              │
              ┌───────────────┴───────────────────────────────┐
       [sword_draw PRISMATIC +X, 0..0.50]          [ring_i_pivot REVOLUTE +Y ±60°]  × N
              │  (mating: scabbard mouth plane @ x=MOUTH_X)         │  (pin axis @ each lug)
              ▼                                                     ▼
            sword  ◄── Slot A (blade_profile) ┐                  ring_i  (multiplicity copies)
              ▲                               ├ both emit visuals onto the SAME `sword` part
              └── Slot B (hilt) ──────────────┘   (parallel children of the sword root frame,
                                                    no inter-slot chain joint)
```

说明：

- **scabbard ↔ sword**：唯一接口是鞘嘴平面（`MOUTH_X=0.50` 处的 yz 平面），`sword_draw` PRISMATIC，axis=(1,0,0)，origin=(MOUTH_X,0,ZC)，limits lower=0 upper=0.50。Slot A 决定 cavity 内腔 + 鞘嘴 `throat` 通孔尺寸（以及是否侧弯）；Slot A、B 的可见几何都画进同一个 `sword` part（不是串链，是同 part 的两个功能层），所以 A↔B 之间没有跨 part joint，只有几何共址约束（guard 内面坐落在鞘嘴处，blade 向 −X 插入鞘内）。
- **scabbard ↔ ring_i**：每个吊环一个 REVOLUTE，axis=(0,1,0)，origin 在对应 lug pin 上，limits ±RING_LIMIT(60°)。ring 偏轴下挂 `RING_HANG`。
- **互斥/gating**：`curved_saber`（Slot A）会让整鞘侧弯；与 `cruciform_disc` / `knuckle_scentstop` 的宽横向硬件组合时弯鞘嘴对位风险高 → 见兼容矩阵（saber 优先只配 `box_bead`，或弯量降一档）。
- **N（吊环数）** 由 multiplicity 轴独立加权采样，与 Slot A/B 正交。

---

## 每槽位 Module Emits / Interfaces

### Slot A / module `leaf_double_edge`
| emits | 描述 | 来源 |
|---|---|---|
| parts | `sword.blade`（叶形双刃 mesh）、`sword.blade_spine`（细 Box 脊） | S_parent / L109-L118, L230-L236 |
| internal joints | 无（blade 是 `sword` part 的 visual） | — |
| upstream interface | 鞘嘴 mating 面（x=MOUTH_X yz 平面）；blade 向 −X 插入 cavity | S_parent / L298-L306 |
| downstream interface | cavity 截面契约（直、半宽 base≈0.023）传给 scabbard cavity 生成 | S_parent / `CAV_SECS` L54-L58 |

### Slot A / module `straight_double_edge`
| emits | 描述 | 来源 |
|---|---|---|
| parts | `sword.blade`（平行双刃）、`sword.blade_spine` | S_straight / L111-L120, L232-L238 |
| internal joints | 无 | — |
| upstream interface | 同上鞘嘴 mating | S_straight / L300-L308 |
| downstream interface | 均匀宽 cavity 契约（半宽≈0.020 等宽到鞘嘴） | S_straight / `CAV_SECS` L56-L60 |

### Slot A / module `curved_saber`
| emits | 描述 | 来源 |
|---|---|---|
| parts | `sword.blade`（侧弯单刃，刃 +Y/脊 −Y；无独立 blade_spine） | S_saber / `_curved_blade_solid` L133-L156, L312-L316 |
| internal joints | 无 | — |
| upstream interface | 鞘嘴 mating（弯鞘的 origin 仍在 x=MOUTH_X，y=0；blade 在 sword-local y=0 base） | S_saber / L386-L394 |
| downstream interface | **弯 cavity + 弯 body/chape/throat + 弯 lug 契约**：scabbard 必须用 `_scabbard_bow(x)` 把所有截面沿 +Y 偏移，cavity 逐站宽于弯刃 | S_saber / `_offset_loft` L107-L122 / `_scabbard_bow` L98-L99 / cavity L201 / lug L278-L307 |

### Slot A / module `broad_triangular`
| emits | 描述 | 来源 |
|---|---|---|
| parts | `sword.blade`（宽三角）、`sword.blade_spine`（凸起三角中脊 mesh） | S_broad / `_blade_solid` L117-L127 / `_blade_spine_solid` L130-L144, L256-L261 |
| internal joints | 无 | — |
| upstream interface | 鞘嘴 mating（宽 throat 通孔） | S_broad / L323-L331 |
| downstream interface | 加宽 cavity（base 全宽 ~0.076）+ 加宽 body W1 + 加宽 throat 契约 | S_broad / `CAV_SECS` L61-L66 / body L50 / throat L69-L71 |

### Slot B / module `box_bead`
| emits | 描述 | 来源 |
|---|---|---|
| parts (visual on `sword`) | `guard`(Box) + `guard_relief_{0,1}`、`grip_core/lower/upper`、`grip_collar_bead_{0..3}`、`pommel`(扁椭球) + `finial_stem/ball` | S_parent / L237-L296 |
| internal joints | 无（全是 `sword` part visual） | — |
| upstream interface | guard 内面坐落鞘嘴（sheathed 时 `expect_gap` guard↔throat 0..0.003） | S_parent / L424-L433 |
| downstream interface | hilt 末端 finial（柄头 +X 端，x≈0.74 world） | S_parent / L478-L483 |

### Slot B / module `cruciform_disc`
| emits | 描述 | 来源 |
|---|---|---|
| parts (visual on `sword`) | `guard`(`_crossguard_solid` 十字) + `guard_groove_{0,1}` + `guard_tip_{0,1}`、grip core/lower/upper + beads、`pommel_collar`、`pommel`(`_disc_pommel_solid` 圆轮) | S_cross / L277-L342 |
| internal joints | 无 | — |
| upstream interface | crossguard 中央块内面坐落鞘嘴（`expect_gap` 放宽到 min_gap=−0.016 容纳中央块厚度） | S_cross / L470-L479 |
| downstream interface | disc pommel 末端（薄圆轮，半径≤0.014 离地） | S_cross / L510-L519 |

### Slot B / module `knuckle_scentstop`
| emits | 描述 | 来源 |
|---|---|---|
| parts (visual on `sword`) | `crossguard`(slim Box) + `crossguard_relief_{0,1}`、`knuckle_guard`(扫掠管 mesh)、grip core/lower/upper + beads、`pommel`(`_scent_stopper_pommel` 八面体) + `finial_stem/ball` | S_knuckle / L278-L361 |
| internal joints | 无（knuckle_guard 是固定 visual，拱过握把但两端坐落 crossguard 与 pommel base） | S_knuckle / `KNUCKLE_POINTS` L89-L95 |
| upstream interface | crossguard 内面坐落鞘嘴（`expect_gap` 0..0.005） | S_knuckle / L489-L498 |
| downstream interface | scent-stopper pommel + finial（+X 端） | S_knuckle / L549-L571 |

### multiplicity / ring_i（每根吊环一份；详见 Multiplicity 节）
| emits | 描述 | 来源 |
|---|---|---|
| parts | `scabbard` 上 `band_{i}_wrap`/`ring_{i}_band`(可选 band 包带) + `{i}_flange`+`{i}_pin`+`{i}_head`(lug 三件)；`ring_{i}`/`band_{i}_ring` part 内 torus `ring` | S_bands4 L267-L308, L391-L410 / S_bands6 L246-L282, L366-L384 |
| internal joints | `ring_{i}_pivot` REVOLUTE +Y ±60° per ring | S_bands4 L400-L410 / S_bands6 L374-L384 |
| upstream interface | lug pin（captured：torus 套 pin，`expect_contact` ring↔pin） | S_bands4 L588-L595 |
| downstream interface | 无（叶子节点） | — |

---

## 参数范围汇总
| 参数 | 类型 | 取值范围 / 候选值 | 标称默认 | 约束类型 | 约束 / 函数 | 来源 |
|---|---|---|---|---|---|---|
| `blade_profile` | enum | `leaf_double_edge` / `straight_double_edge` / `curved_saber` / `broad_triangular` | — | choice | deterministic procedural sampler；`curved_saber` 受 hilt-gate（见兼容矩阵） | Slot A table |
| `hilt` | enum | `box_bead` / `cruciform_disc` / `knuckle_scentstop` | — | choice | deterministic procedural sampler | Slot B table |
| `ring_count` | int | `[1, 8]` | 2 | choice (multiplicity) | 加权采样（小 N 高频）；clamp 到 `[1,8]`；详见 Multiplicity 节 | S_parent N=2 / S_bands4 N=4 / S_bands6 N=6 |
| `palette_style` | enum | `steel_bronze` / `blackened_iron` / `brass_leather` / `silver` | `steel_bronze` | choice | 仅改材质 rgba，不改拓扑/尺寸 | 全样本 `model.material(...)` L140-L147 |
| `length_scale` | float | `[0.92, 1.10]` | 1.0 | independent | 整体 x 长度缩放；`sword_draw.upper` 与 blade 长度同比缩放 | S_parent overall ~0.75 m / L382-L386 |
| `blade_thick_scale` | float | `[0.85, 1.20]` | 1.0 | independent | 刃厚 `BLADE_THICK` 缩放 | S_parent L71 |
| `ring_x_spacing_scale` | float | `[0.85, 1.10]` | 1.0 | independent | 吊环沿 x 等距间距缩放（不改首尾边界 clearance） | S_bands6 `RING_X_START/END` L76-L77 |
| `cavity_clearance` | float | derived | — | equation | `= max(blade_half_w(profile)+0.0010, …)` 逐站；cavity 半宽 = blade 半宽 + clearance margin（直刃 0.0010，曲刃额外 +0.0008 余量） | S_saber CAV vs blade / S_broad CAV L61-L66 |
| `throat_hole_w` | float | derived | — | equation | `= cavity_w(at MOUTH_X) + 0.004`（鞘嘴通孔随 blade 宽派生：broad 用 0.086，常规 0.0555） | S_broad L71 / S_parent L63 |
| `sword_draw_upper` | float | derived | 0.50 | equation | `= 0.50 · length_scale`（抽出行程随整体长度派生，保证全抽越过鞘嘴） | S_parent L305 |
| (—) | constraint | — | — | inequality | `blade_outer_aabb(yz) + margin ≤ cavity_inner(yz)` 逐站（鞘内 blade 不穿模）；违反 → 增大该站 cavity 宽或拒绝重采 | `expect_within` S_parent L406-L414 |
| (—) | constraint | — | — | inequality | `pommel/disc/knuckle 最低点 z ≥ 0`（柄头硬件离地）；违反 → 缩半径或抬 grip 中心 | S_cross disc r≤0.014 注释 L153-L156 |
| (—) | constraint | — | — | inequality | `ring_x_i ∈ [chape_end+pad, throat_x0−pad]` 且 `Σ 等距 ≤ 可用 body 长`（N 大时压缩间距）；违反 → 按 N clamp 间距 | S_bands6 `RING_X_START/END` L76-L77 |
| (—) | constraint | — | — | conditional | `curved_saber` 选中时：若 hilt ∈ {cruciform_disc, knuckle_scentstop} 则 `BOW_MAX` 上限降到 0.010（或 gate 改选 box_bead），避免弯鞘嘴与宽横向硬件穿模 | 兼容矩阵 |

连续尺寸采样契约：先采 `length_scale`/`blade_thick_scale`/`ring_x_spacing_scale`（independent，均匀）→ 派生 `cavity_clearance`/`throat_hole_w`/`sword_draw_upper`（equation）→ 用 inequality 把 cavity-nesting、离地、吊环边界投影/回缩 → 解析 `curved_saber` 的 `BOW_MAX` conditional 上限。全部在 `resolve_config` 求解。

---

## Multiplicity / Copy Logic

**一根 multiplicity 轴：suspension rings（吊环站）。** Slot A/B 无 multiplicity（固定 named 结构）。

- `count_param`: `ring_count`
- `N_range`: `[1, 8]`（产品域；本小类样本覆盖 N∈{2,4,6}，模板建议 `[1,8]`。测试偏小 N，产品全程）
- sampling domain（权重档）：小 N 高频、大 N 稀有。建议权重 `{1:0.10, 2:0.34, 3:0.18, 4:0.16, 5:0.08, 6:0.07, 7:0.04, 8:0.03}`（N=2 是 parent 的成对标准式，权重最高；N≥7 稀有，靠构造安全、稀疏采样）。每个 seed 对这根轴做一次加权抽样、clamp 到 `[1,8]`、编进 `slot_choices`、sweep 各自设上限 8。
- copied object：一个 **suspension-ring station** = 鞘上一组 lug（`flange` + `pin` + `head` 三个 Cylinder，可选 `band_{i}_wrap` 包带）+ 一个 `ring_{i}` part（torus `ring` visual，偏轴下挂 `RING_HANG`）。
- naming：`scabbard` 上 visual `band_{i}_wrap` / `band_{i}_flange` / `band_{i}_pin` / `band_{i}_head`（i=0..N−1）；ring part `band_{i}_ring`（含 visual `ring`）；joint `band_{i}_pivot`。**采用 bands4 的 `band_{i}` 循环命名为模板规范**（统一、与 `_lug_positions` helper 配套）。
- placement：沿鞘 body 等距 x（`RING_X_START=0.13` … `RING_X_END=0.42`，clear of chape ball 与 throat band）；`ring_x_spacing_scale` 缩放间距。侧别策略二选一（作为 module-local 固定策略，非新 slot）：**全 +Y 侧**（bands4 风格，配 band wrap）或 **交替 +Y/−Y**（bands6 `_ring_positions` 风格，N≥4 时更均衡）。初版默认：N≤3 全 +Y（贴 parent 单侧观感），N≥4 交替侧。
- joint policy：**统一** —— 每环 REVOLUTE about +Y（`SWING_AXIS_Y=(0,1,0)`），limits `±RING_LIMIT=60°`，origin 在该环 lug pin 上，ring 偏轴下挂 `RING_HANG` 使旋转 AABB 可见（`xmin` 减小、`zmax` 增大）。torus 套 pin 为 captured contact（`expect_contact` ring↔pin，需 element-scoped `allow_overlap`）。
- source/gating：parent 手工成对（`front`/`rear`，反侧）→ **band_i loop-rewrite 是契约的一部分**：模板必须把 parent 的 hand-paired 2-tuple 改写为 `for i in range(ring_count)` 循环（沿用 bands4 的 `_lug_positions(half_w)` helper 算 flange/pin/head/joint 的 y 中心）。`sword_draw` PRISMATIC 在所有 N 下都保留（恰好一个 prismatic）。

跨轴共享 sampling helper 暂不抽象（这是本小类唯一 multiplicity 轴）。

---

## 拓扑多样性审计

总组合数（拓扑等价类）：blade_profile(4) × hilt(3) × ring_count N(8 个 distinct N) = **96**（兼容矩阵会 gate 掉少量 saber×宽硬件组合，仍 ≫ 阈值）。`palette_style` / 连续 scale 不计入拓扑等价类。

理由：仅 blade(4)×hilt(3)=12 个 (A,B) 组合就 ≥10；再叠加 8 个 N，distinct slot_choice tuple 上限约 96。50-seed sweep 必然覆盖 ≥10 distinct；1000-seed 低于 300 时记录该小词汇上限。

- `seed_domain_policy`: `procedural_first`（seed=0 不特殊）。
- Topology target：1000-seed slot choice tuple distinct 富类别建议 ≥300（report-only）；4×3×8=96 个基础类，低于 300 的原因是该类源支持结构空间封顶。
- regression overrides：none（首版不用 curated/modulo 表作为主 seed domain）。
- Controlled local parameterization：`length_scale[0.92,1.10]`、`blade_thick_scale[0.85,1.20]`、`ring_x_spacing_scale[0.85,1.10]` 为 independent；`cavity_clearance`/`throat_hole_w`/`sword_draw_upper` 为 equation 派生；cavity-nesting/离地/吊环边界为 inequality；`curved_saber` 的 `BOW_MAX` 上限为 conditional。全部在 `resolve_config` clamp/派生，保证不破坏 PRISMATIC mating、cavity 嵌套、REVOLUTE origin、N×吊环与类别 identity。

| item | policy | validator / viewer focus |
|---|---|---|
| sampler | 先抽 ring_count（加权小 N），再均匀抽 blade/hilt，再 gate，再 palette/scale | slot_choices_for_seed 与 build 选择一致 |
| compatibility matrix | saber × {cruciform_disc, knuckle_scentstop}：降 BOW_MAX 到 ≤0.010 或回退 box_bead；其余全合法 | 弯鞘嘴 vs 宽横向硬件无穿模；cavity 逐站包住所选 blade；无悬空吊环；N≤8 |
| controlled local variation | length/blade_thick/ring_spacing scale + clamp/派生 | 比例变化不破坏 prismatic 行程、cavity clearance、吊环 captured contact、joint origin、类别 identity |
| regression overrides | none | — |
| random sweep | 初版 seeds 0-49；成熟审计 0-999 | cavity nesting / draw / ring-swing 契约失败 |

| slot | candidate_count | 是否 ≥2 | 是否 ≥3 | 备注 |
|---|---:|---|---|---|
| A blade_profile | 4 | yes | yes | |
| B hilt | 3 | yes | yes | |
| multiplicity rings (N) | 8 (N∈[1,8]) | yes | yes | 一根 multiplicity 轴 |

---

## Validator

- `slot_choices_for_seed` 返回已实现 module 名（含 `f"{N}_ring_set"`）。
- `config_from_seed` 对所有普通 seed 用 deterministic procedural sampling（seed=0 不特殊）。
- 兼容矩阵 gate 阻止 saber × 宽横向 hilt 的弯鞘穿模组合（降 BOW_MAX 或回退 box_bead）。
- 无 regression overrides（不循环小型 curated 表）。
- 受控 scale 在 `resolve_config` clamp/派生；cavity_clearance/throat_hole_w/sword_draw_upper(equation)、cavity-nesting/离地/吊环边界(inequality)、saber BOW_MAX(conditional) 全在 resolve_config 求解，不留到 builder 才失败。
- 关键 InterfaceSpec / MatingContract 存在：鞘嘴 PRISMATIC mating 面（x=MOUTH_X）；每环 lug pin captured contact。
- 关键 joints 类型/轴/行程：`sword_draw` PRISMATIC axis=(1,0,0) lower=0 upper≈0.50；每个 `band_{i}_pivot` REVOLUTE axis=(0,1,0) ±60°；每个模型恰好一个 prismatic。
- copied object 遵循命名/placement：`band_{i}_*` lug + `band_{i}_ring` + `band_{i}_pivot`，沿 x 等距、clear of chape/throat。
- captured-pin overlap 用 element-scoped `allow_overlap(ring_part, scabbard, elem_a="ring", elem_b="band_{i}_pin", ...)`，不可用 broad/floating 允许。

## Reject cases

- blade outer 截面在任一站超出 cavity inner（鞘内 blade 穿模 / `expect_within` 失败）—— 尤其 broad_triangular 未加宽 cavity，或 curved_saber 的弯刃越出未同弯的 cavity。
- 鞘体/chape/throat/cavity 在 saber 下未用 `_scabbard_bow` 同步侧弯（弯刃配直鞘）。
- 全抽（q=upper）时刃未完全越过鞘嘴，或 sheathed 时刃未藏入鞘内（draw 语义断裂）。
- 模型缺失或多于一个 prismatic（`sword_draw` 必须恰好一个）。
- 吊环 lug pin 与 torus 未接触（ring 漂浮）、或偏轴下挂被去掉导致 REVOLUTE 旋转 AABB 不可见（swing 不可验证）。
- 吊环站落在 chape ball 或 throat band 上（x 越界），或 N=8 时间距压到 lug 互相穿模。
- 柄头硬件（disc pommel / scent-stopper / knuckle 顶）最低点低于地面 z<0，或 knuckle_guard 两端不坐落 crossguard/pommel（悬空 island）。
- crossguard/box guard 内面没坐落鞘嘴（sheathed gap 超界）。

---

## 与相邻类别的边界

- 不该混入：**knife / dagger（Tools 或 Military/knife）** —— 短刃匕首无鞘抽出 PRISMATIC 主语义、无悬挂吊环 multiplicity；sword 的身份是"带鞘可抽出的长刃 + 吊环"。
- 不该混入：**spear / polearm / staff** —— 长杆刺击武器主轴是杆而非可抽出的鞘+刃组合，无 scabbard cavity 嵌套。
- 不该混入：**scabbard / sheath 作为独立容器类** —— 本模板鞘是 chassis 的一部分，不是独立可开合容器；不要把鞘建成有铰链盖的 box/container。
- 不该混入：**sliding-sash / drawer 等纯 PRISMATIC 抽屉类** —— 虽同为 prismatic，但 sword 的 prismatic 语义绑定"刃从鞘嘴抽出 + 刃在鞘内嵌套"，且叠加 REVOLUTE 吊环 multiplicity 与刃/柄拓扑，不是单纯滑轨。

---

## 审核记录
| 项 | 结论 |
|---|---|
| reviewer status | pending |
| reviewer notes | 待人工审核。重点核对：(1) blade-profile × cavity 兼容矩阵是否充分（尤其 curved_saber 弯鞘 + 宽横向 hilt 的 gate）；(2) band_i loop-rewrite 命名规范（采 bands4 `band_{i}_*`）；(3) N_range[1,8] 权重档；(4) palette_style 4 档是否够。 |

## 模板实现备注（可选）

- 共享 helper：`_loft`（直）/ `_offset_loft`+`_scabbard_bow`（弯，saber 专用）；`_lerp_sections`；`_lug_positions(half_w)`（bands4，算 flange/pin/head/joint y）；`_ring_solid`。直 vs 弯鞘体走两条 build 路径，由 `blade_profile==curved_saber` 选择。
- 关键 InterfaceSpec / MatingContract：鞘嘴 PRISMATIC（mating 可用 yz 面契约）；吊环 pin-through-torus 属 captured-pin，**omit `mating=`（grandfather）** 并用 element-scoped `allow_overlap`。
- captured-pin overlap：每环 `allow_overlap(band_{i}_ring, scabbard, elem_a="ring", elem_b="band_{i}_pin", reason="ring threaded on lug pin")`。
- 暂不进入 seed domain 的组合：`curved_saber` × `cruciform_disc`/`knuckle_scentstop` 的高弯量（BOW_MAX>0.010）档由兼容矩阵 gate 掉，留待审核后视目检决定是否放宽。
- 参考实现深读建议（TEMPLATE_AFTER_REVIEW 阶段）：`monitor_mount`（multiplicity N-link + captured-pin allow_overlap 范式）+ `retractable_utility_knife`（housing→mechanism→blade 链 + PRISMATIC 抽出语义最接近本类的 draw）。

## Module Source Index

| source_id | slot | module | sample_id | model.py 来源 | 采纳用途 |
|---|---|---|---|---|---|
| S_parent | A/B/mult | leaf_double_edge / box_bead / N=2 | rec_model-a-roman-gladius-style-short-sword-sheathed_20260610_080621_578517_42984874 | L99-L329 | 直鞘 chassis + 叶刃 + box/bead hilt + 成对吊环（loop-rewrite 起点） |
| S_straight | A | straight_double_edge | rec_sword_var_straightblade | L111-L120, L56-L60 | 平行双刃 + 均匀 cavity |
| S_saber | A | curved_saber | rec_sword_var_saber | L98-L156, L201-L307 | 弯刃 + 弯鞘体/cavity/throat/lug（`_offset_loft`/`_scabbard_bow`） |
| S_broad | A | broad_triangular | rec_sword_var_broadsword | L117-L144, L50-L71 | 宽三角刃 + 中脊 + 加宽 cavity/throat |
| S_cross | B | cruciform_disc | rec_sword_var_crossguard_disc | L126-L171, L277-L342 | 十字格 + 圆轮 disc pommel + peening collar |
| S_knuckle | B | knuckle_scentstop | rec_sword_var_basket_scentstop | L89-L173, L278-L361 | slim crossguard + 扫掠 knuckle 管 + 八面体 scent-stopper pommel |
| S_bands4 | mult | 4_ring_set | rec_sword_var_bands4 | L143-L198, L267-L410 | `band_{i}` 循环命名规范 + `_lug_positions` + band wrap（全 +Y） |
| S_bands6 | mult | 6_ring_set | rec_sword_var_bands6 | L90-L108, L246-L282, L366-L384 | `_ring_positions()` 交替 +Y/−Y 侧策略 |
