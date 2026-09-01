# Modular Spec — Astronomy / Satellite

## 元信息
| 项 | 值 |
|---|---|
| slug | `Astronomy_Satellite` |
| template path | `agent/templates/Astronomy_Satellite.py` |
| test path (optional) | `tests/agent/test_Astronomy_Satellite_template.py` |
| stage | `SPEC_ONLY_DRAFT` |
| status | `pending` |
| __modular__ | `True` |
| pattern | `mixed` (root bus + parallel-children solar-power + parallel-children antenna + 2 multiplicity axes) |
| function stem | `astronomy_satellite` (exports `build_astronomy_satellite`, `config_from_seed`, `run_astronomy_satellite_tests`) |

`pattern = mixed`: a single root `bus` part carries two parallel-children slots
(solar power appendages on the ±Y walls; antenna reflectors on the deck faces),
each of which manually parents its own articulations to the bus (no serial chain
joint). Two multiplicity axes ride on top: `panel_count` (panels per solar wing)
and `dish_count` (number of reflector stations).

## 5 星样本阅读摘要
| 项 | 值 |
|---|---|
| five_star_total | 10 |
| read_count | 10 |
| read_scope | all 5-star samples in this subclass (origin 母本 + 9 forked variants) |
| source_index_policy | only adopted module sources are indexed below; all 10 were read in full |

Samples (all `collections=["workbench"]`, `rating=5`, synced from `articraft_data`):

- `rec_a-large-esa-style-geostationary-telecommunicatio_20260708_115617_705266_8d718b55` — ORIGIN 母本 (box bus, 2 rigid CONTINUOUS solar wings ×3 panels, 2 saucer dishes on REVOLUTE gimbals).
- `rec_satellite_var_panels5` — multiplicity: `WING_PANEL_COUNT=5`.
- `rec_satellite_var_dishes4` — multiplicity: `DISH_SIDES` 2→4 (upper/lower/forward/aft).
- `rec_satellite_var_wing_revolute` — joint type: wing drive CONTINUOUS→REVOLUTE (bounded slew).
- `rec_satellite_var_dish_prismatic` — joint type: dish gimbal REVOLUTE→PRISMATIC (telescoping deploy).
- `rec_satellite_var_flatpanel_antenna` — form family (aperture): parabolic saucer→flat phased-array Box panel.
- `rec_satellite_var_hex_bus` — form family (bus): box→hexagonal prism (`ExtrudeGeometry`).
- `rec_satellite_var_single_boom_dish` — skeleton: two side dishes→one dish on a central mast boom.
- `rec_satellite_var_folding_wing_chain` — skeleton: rigid wing→accordion REVOLUTE fold chain.
- `rec_satellite_var_solardrum` — form family (power): flat panel wing→cylindrical body-mounted solar drum on CONTINUOUS X-spin drive.

> **Confirmed-pool note (reviewer):** the upstream human-confirmed
> `variant_pool_checklist.md` (batch `variant_fork_4cats_20260710`) marks **8**
> Satellite variants OK and does **not** list `solardrum`. `solardrum` is a
> valid *direct* origin fork (`edit_mode=copy`, parent=origin), compiles clean
> (0 warnings), created one day before the confirmed batch. The PATHFINDER task
> brief enumerates it as part of the confirmed pool, so it is adopted here as
> the source anchor for the `cylinder_bus` ③ form candidate and recorded
> explicitly rather than silently dropped. Flagged for reviewer reconciliation.

## 核心身份

An **orbital telecommunications / relay satellite (spacecraft bus + deployed
appendages)**: a compact central spacecraft **bus** (box / hexagonal-prism /
cylindrical body, tiled with radiator + solar-cell plates, gold MLI foil
patches, a small sensor/camera barrel) that carries, on articulated deployable
appendages, (a) **solar power** — long multi-segment solar array wings or
body-mounted solar drums on rotary **array-drive** joints, and (b) the identity
feature **antenna reflectors** — large dish saucers or flat phased-array panels
on steerable **gimbal / deploy** joints. At least one real non-fixed joint is
always present (an array drive and/or a dish gimbal). Default mature domain:
1–2 m class bus with 2 wings and 1–4 reflectors.

Not to be confused with the neighbouring picture subclass **Astronomy / Antenna
dish** (a *ground-station* parabolic dish on an az-el pedestal/tripod planted on
the earth) — the Satellite bus is a free-flying body whose dishes are secondary
deployed appendages, not the whole object on a ground mount.

## 槽位 + 候选模块表

### Slot A：bus_body (root · ③ Primary Form Family)

The root spacecraft body. Same part tree across candidates: one body visual +
3×3 `front_radiator_tile` grid + 4 `mli_*` gold-foil patches + `camera_barrel` +
`camera_lens` (all fused as `bus` part visuals, Rule 1). Only the body envelope
prototype changes; all three expose the identical mounting envelope
(`bus_half_y` for ±Y wings, `bus_half_z` for ±Z decks, `bus_front_x` for the +X
Earth face) so downstream slots are form-independent.

| module_name | source_type | source evidence | model.py:Lx-Ly | sampling eligibility | 结构特征 / form_subtype |
|---|---|---|---|---|---|
| `box_bus` | forked_anchor (origin) | `rec_a-large-esa-...8d718b55` | L91-L133 | eligible | rectangular `Box` bus body + tile grid + MLI + camera. **Volumetric Envelope Form** |
| `hex_bus` | forked_anchor | `rec_satellite_var_hex_bus` | L46-L58, L72-L79, L117, L123-L166 | eligible | hexagonal-prism body via `ExtrudeGeometry(_hex_profile(R), BUS_Z)`; flat +X face at apothem; tiles/MLI re-anchored to hex faces. **Volumetric Envelope Form** |
| `cylinder_bus` | world_knowledge_extrapolation (③) | anchors: `rec_satellite_var_solardrum` drum lathe L142-L160 + `rec_a-...8d718b55` box part tree; reviewer | n/a (LatheGeometry cylinder profile, reuses drum lathe idiom) | eligible | spin-stabilized cylindrical body (`LatheGeometry` closed cylinder + flat end caps), same tile/MLI/camera visual set. **Volumetric Envelope Form** (real HS-376/early-Intelsat bus form) |

### Slot B：solar_power (parallel children on bus ±Y · ① skeleton + ② joint + multiplicity `panel_count`)

Two mirror appendages on opposite ±Y bus walls, each on a rotary array drive.
Array drive is a captured-boom pivot socketed into the bus wall → joint
grandfathered (no MatingContract), element-scoped `allow_overlap(bus_body, boom)`
(Rule 2 captured-pivot exception, exactly as every source declares).

| module_name | source_type | source evidence | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|---|
| `rigid_wings` | forked_anchor (origin) | `rec_a-...8d718b55` | L135-L196 | eligible | 2 wings; `yoke_boom` + 2 `yoke_strut_k` + N rigid `array_panel_i` boxes + `panel_hinge_spar_i_k`; `solar_wing_drive_{side}` **CONTINUOUS** axis Y. panel_count multiplicity. |
| `revolute_wings` | forked_anchor | `rec_satellite_var_wing_revolute` | L136, L185-L197 | eligible | identical wing part tree; drive **REVOLUTE** axis Y with bounded slew `MotionLimits(lower=-0.55, upper=0.55)`. panel_count multiplicity. |
| `folding_wings` | forked_anchor | `rec_satellite_var_folding_wing_chain` | L136-L246 | eligible | ① skeleton change: each wing is a chain — root segment (yoke + `array_panel_0`) → `array_panel_{side}_i` linked by `panel_fold_{side}_i` **REVOLUTE** accordion joints (alternating z axis); CONTINUOUS root drive. panel_count multiplicity = chain length. |
| `solar_drums` | forked_anchor | `rec_satellite_var_solardrum` | L33-L37, L135-L235 | eligible | ③/① change: 2 cylindrical `solar_drum_{side}` modules (`boom_shaft` + `hub_collar` + `drum_body` LatheGeometry + `endcap_k` + cell-ring dividers) on `solar_drum_drive_{side}` **CONTINUOUS** axis X. No panel_count (monolithic drum). |

### Slot C：antenna (parallel children on bus deck/faces · ① skeleton + ③ aperture form + ② joint + multiplicity `dish_count`)

The identity feature. Each reflector station is a `reflector*` part carrying
`shoulder_barrel` (captured gimbal barrel, embedded → grandfathered joint +
`allow_overlap`), `deploy_arm` (tube mesh), `dish_hub`, the aperture, and a
`feed_aperture`. Steered/deployed on a gimbal joint parented to the bus.

| module_name | source_type | source evidence | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|---|
| `dual_dish_revolute` | forked_anchor (origin, dishes4) | `rec_a-...8d718b55` L198-L251; `rec_satellite_var_dishes4` L42-L64, L221-L310 | L198-L251 | eligible | K saucer reflectors (`LatheGeometry` parabolic profile) on curved arms; `dish_gimbal_{side}` **REVOLUTE** axis Y ±0.35. K=2 upper/lower; K=4 adds forward/aft on ±X faces (DISH_CONFIG). dish_count multiplicity. |
| `dual_dish_prismatic` | forked_anchor | `rec_satellite_var_dish_prismatic` | L235-L254 | eligible | same saucer reflectors; `dish_gimbal_{side}` **PRISMATIC** telescoping along arm axis `(0.4906,0.2818,0.8245)`, `MotionLimits(lower=0, upper=0.35)`. dish_count multiplicity. |
| `dual_flatpanel` | forked_anchor | `rec_satellite_var_flatpanel_antenna` | L179-L219 | eligible | ③ aperture form: flat rectangular phased-array `array_face` Box (0.04×1.50×1.30) + Box feed patch replaces the parabolic saucer; REVOLUTE gimbal retained. **Planar Boundary Form**. dish_count multiplicity. |
| `single_boom_dish` | forked_anchor | `rec_satellite_var_single_boom_dish` | L42, L74-L90, L201-L245 | eligible | ① skeleton: ONE saucer reflector on a long central `deploy_arm` mast (`MAST_HEIGHT≈2.2`) rising from the top deck; single `dish_gimbal` REVOLUTE axis Y. dish_count fixed = 1. |

硬约束满足：每个 slot ≥3 结构不同 candidate（A=3, B=4, C=4）；每个普通
candidate 有 forked_anchor + `model.py:Lx-Ly`；唯一 `world_knowledge_extrapolation`
是 `cylinder_bus`（③ Primary Form Family 例外，form_subtype=Volumetric Envelope
Form，几何锚定在 solardrum 的 LatheGeometry 圆柱轮廓 + origin 的 box part tree，
保持同一 part tree / primitive 家族 / mounting interface）。

## 槽位图（slot graph）

pattern: `mixed` (root + parallel children + multiplicity)

```
bus_body (root; box / hex / cylinder)
   ├─[±Y wall · array-drive CONTINUOUS|REVOLUTE(Y) or drum-drive CONTINUOUS(X); captured boom socket]→ solar_power  (×2 mirror appendages, ×N panels)
   └─[deck/face · gimbal REVOLUTE(Y)|PRISMATIC(arm-axis); captured barrel socket]→ antenna      (×K reflector stations)
```

- **slot 顺序 / parent**：`bus_body` 是 root，唯一被复用的 parent。`solar_power`
  与 `antenna` 都直接把各自 joint 的 `parent=bus`，互不串联（parallel children）。
  两者均只声明 `downstream` 接口（re-export bus），不声明 `upstream`，因此
  assembler 不发射自动 chain joint（各模块自己发原始 joint，与 5 星源一致）。
- **接口点位**：solar → bus 侧壁面 `(0, ±bus_half_y, 0)`（+Y/-Y face，captured
  boom）；antenna → bus 顶/底 deck `(0.25, ±0.30, ±bus_half_z)`，K=4 追加 +X/-X
  face `(±bus_front_x, ±0.35, ±0.40)`（captured barrel）。single_boom → 顶 deck
  `(0,0,bus_half_z)`。
- **跨 slot joint type/axis/range**：array drive CONTINUOUS(Y) / REVOLUTE(Y,
  ±0.55) / drum CONTINUOUS(X)；gimbal REVOLUTE(Y, ±0.35) / PRISMATIC(arm-axis,
  0..0.35)；fold chain REVOLUTE(Z, 0..fold_upper)。
- **互斥/派生**：`single_boom_dish` 强制 `dish_count=1`；`solar_drums` 无
  `panel_count`。bus 形态与 solar/antenna 完全正交（挂点用共享 half-extent 派生，
  与 bus 形态无关），可自由组合。

## 每槽位 Module Emits / Interfaces

### Slot A / module box_bus | hex_bus | cylinder_bus
| emits | 描述 | 来源 |
|---|---|---|
| parts | `bus` (single root part) | origin L91 |
| visuals | `bus_body` (Box/hex-mesh/lathe-cyl) + `front_radiator_tile_r{r}_c{c}` (3×3) + `mli_*`×4 + `camera_barrel` + `camera_lens` | origin L91-L133; hex L117,L123-L166 |
| internal joints | none (root, static body) | — |
| downstream interface | `bus` part, `bus_body` visual, face `positive_z`, anchor `(0,0,bus_half_z)`, face_extents `(2·bus_front_x, 2·bus_half_y)` (informational; children wire manually) | — |

### Slot B / module rigid_wings | revolute_wings | folding_wings | solar_drums
| emits | 描述 | 来源 |
|---|---|---|
| parts | `solar_wing_{pos_y,neg_y}` (+ `array_panel_{side}_i` chain parts for folding) or `solar_drum_{pos_y,neg_y}` | origin L137-L139; folding L149-L194; drum L162 |
| visuals | `yoke_boom` + `yoke_strut_k` + `array_panel_i` + `panel_hinge_spar_i_k`; drum: `boom_shaft`+`hub_collar`+`drum_body`+`endcap_k`+cell rings | origin L143-L183; drum L164-L221 |
| internal joints | `solar_wing_drive_{side}` (CONTINUOUS/REVOLUTE Y) or `solar_drum_drive_{side}` (CONTINUOUS X); folding adds `panel_fold_{side}_i` (REVOLUTE Z) | origin L185-L196; revolute L185-L197; folding L206-L246; drum L224-L235 |
| upstream interface | **none declared** (parallel-children; parents joints directly to `bus`) | — |
| downstream interface | re-export bus downstream (passthrough) | — |

### Slot C / module dual_dish_revolute | dual_dish_prismatic | dual_flatpanel | single_boom_dish
| emits | 描述 | 来源 |
|---|---|---|
| parts | `reflector_{upper,lower[,forward,aft]}` (K parts) or `reflector` (single boom) | origin L204; dishes4 L221; single L203 |
| visuals | `shoulder_barrel` + `deploy_arm`(tube/mast mesh) + `dish_hub` + (`dish_saucer` lathe \| `array_face` box) + `feed_aperture` | origin L207-L234; flat L203-L219; single L201-L235 |
| internal joints | `dish_gimbal_{side}` REVOLUTE(Y,±0.35) \| PRISMATIC(arm-axis,0..0.35); single `dish_gimbal` REVOLUTE | origin L236-L251; prismatic L236-L254; single L237-L245 |
| upstream interface | **none declared** (parallel-children; parents joints directly to `bus`) | — |
| downstream interface | re-export bus downstream (passthrough) | — |

活动件语义：array drive 旋转/展开太阳翼；gimbal 指向/展开天线；fold joint 折叠
手风琴翼。不动细节（tiles/MLI/camera/struts/feed）写成宿主 part visual，非独立
part（Rule 1）。captured boom/barrel socket 用 element-scoped allow_overlap（Rule 2
例外），gimbal/drive 原点落在 bus 真实 face 几何（origin honesty）。

## 参数范围汇总
| 参数 | 类型 | 取值范围 / 候选值 | 标称默认 | 约束类型 | 约束 / 函数 | 来源 |
|---|---|---|---|---|---|---|
| `bus_form` | enum | box_bus / hex_bus / cylinder_bus | box_bus | choice | procedural sampler | Slot A |
| `power_module` | enum | rigid_wings / revolute_wings / folding_wings / solar_drums | rigid_wings | choice | procedural sampler | Slot B |
| `antenna_module` | enum | dual_dish_revolute / dual_dish_prismatic / dual_flatpanel / single_boom_dish | dual_dish_revolute | choice | procedural sampler | Slot C |
| `panel_count` | int | {3,4,5} (obs: 3 origin, 5 panels5; 4 interp within range) | 3 | conditional | only for panel-based power modules; `solar_drums`→n/a | origin L33, panels5 L33 |
| `dish_count` | int | {2,4} (obs: 2 origin, 4 dishes4) | 2 | conditional | forced to 1 iff `antenna_module==single_boom_dish` | origin L42, dishes4 L42 |
| `bus_scale` | float | [0.85, 1.20] | 1.0 | independent | uniform, clamp; scales bus envelope (half_y/half_z/front_x co-derive) | origin L29-L31 |
| `bus_half_y` | float | derived | — | equation | `= 0.85·bus_scale` (box) / `HEX_RADIUS·bus_scale` (hex) / `R_cyl·bus_scale` (cyl) | origin L30 |
| `bus_half_z` | float | derived | — | equation | `= 0.95·bus_scale` | origin L31 |
| `bus_front_x` | float | derived | — | equation | `= 0.65·bus_scale` (box) / `apothem·bus_scale` (hex) / `R_cyl·bus_scale` (cyl) | origin L29 |
| `panel_span_scale` | float | [0.85, 1.15] | 1.0 | independent | uniform, clamp; panel length/height | origin L34-L35 |
| `dish_radius_scale` | float | [0.85, 1.15] | 1.0 | independent | uniform, clamp; saucer/array footprint | origin L39 |
| `gimbal_range` | float | [0.20, 0.45] | 0.35 | independent | revolute gimbal ±range (rad) | origin L250 |
| `wing_slew` | float | [0.35, 0.60] | 0.55 | conditional | revolute_wings slew ±range; else n/a | wing_revolute L195 |
| `fold_upper` | float | [0.9, 1.4] | 1.2 | conditional | folding_wings per-fold REVOLUTE upper (rad); capped so accordion panels stay clear | folding L242 |
| (—) | constraint | — | — | inequality | `dish_count==1` when `single_boom_dish`; K=4 forward/aft only when bus provides a +X/-X mount → always true (all forms expose `bus_front_x`) | dishes4 L42-L64 |

所有 equation/inequality/conditional 在 `resolve_config` 内求解；builder 不失败。

## 参数范围汇总 → 7.5 编译预算 / compile budget（必填）

**Per-seed compile budget: ≤ 20 s** (hang-guard `--compile-timeout 60`).
Geometry is dominated by a few meshes: `dish_saucer` LatheGeometry (72 seg),
`deploy_arm` tube (16 radial), hex/cylinder bus lathe/extrude (≤48 seg). All K
reflectors share ONE `dish_mesh` / one `arm_mesh`; both wings share one panel
box geometry; drum shell shared. Tessellation tiers: saucer/hex/cyl ≤48–72 seg
(hero body), arm tube 16 radial, small cylinders (hub/barrel/camera) default.
No boolean sculpting. Expect 5–12 s/seed; downgrade seg counts first if over.

## Multiplicity / Copy Logic

**两根独立 multiplicity 轴**（各自加权采样、各自编入 `slot_choices`、各自 clamp、
sweep 各自设上限）：

### 轴 1 — `panel_count`（每翼太阳能板段数）
- `count_param`: `panel_count`; `N_range` product `[3,5]`, test `[3,5]`; sampling
  domain 加权：`{3: 0.5, 4: 0.2, 5: 0.3}`（小 N 偏多）。
- copied object: `array_panel_i` box（`for i in range(panel_count)`）+
  `panel_hinge_spar_i_k` 桥接每个 inter-panel gap；folding 时每段是独立 chain part
  `array_panel_{side}_i` + `panel_fold_{side}_i` REVOLUTE。
- naming: `array_panel_{i}` / `panel_hinge_spar_{i}_{k}` / (folding)
  `array_panel_{side}_{i}` / `panel_fold_{side}_{i}`。placement: uniform
  `panel_pitch = PANEL_LEN_Y + PANEL_GAP` 沿 +Y。joint policy: rigid/revolute =
  no per-panel joint（一整块翼）；folding = 每段一个 REVOLUTE fold。
- source/gating: origin (N=3) L164-L183, panels5 (N=5) L33；`solar_drums` 不参与
  （drum 无 panel），采样时该轴写 `n/a`。
- 数量变化不改主体形态/机制（folding 仍是 folding，rigid 仍是 rigid）。

### 轴 2 — `dish_count`（反射面/天线站数）
- `count_param`: `dish_count`; `N_range` `[2,4]`, test `{2,4}`; sampling domain
  加权：`{2: 0.7, 4: 0.3}`（大 N 稀有）。
- copied object: `reflector_{side}` 整个天线装配（barrel+arm+hub+aperture+feed）+
  各自 `dish_gimbal_{side}`。K=2 → upper/lower（±Z deck）；K=4 → +forward/aft
  （±X face），per-side DISH_CONFIG origin/rpy。
- naming: `reflector_{upper,lower,forward,aft}` / `dish_gimbal_{side}`。placement:
  real bus faces。joint policy: 每站独立 gimbal（REVOLUTE 或 PRISMATIC）。
- source/gating: origin (K=2) L202-L251, dishes4 (K=4) L42-L64,L221-L310。
  `single_boom_dish` 强制 K=1（gated）。**非-box bus + 深抛物面 saucer（dual_dish_
  revolute / dual_dish_prismatic）门控为 K=2**（六棱柱 apothem / 圆柱曲面无法为大口径
  aft saucer 提供足够 standoff；薄平板相控阵 dual_flatpanel 贴合任意 hull，K=4 保留）。
- K=4 forward barrel 穿过 +X tile 层 → element-scoped allow_overlap（dishes4 L301-L310）。
- 每个 revolute dish gimbal 的转角由 `clamp_joint_limits`（clearance solver，
  keepout=["bus"]，豁免 captured barrel / tile overlap）求解，替代手调角度上限，
  跨 bus 形态与口径自适应保证转动全程反射面不扫入本体。

## 视觉多样性 6 轴考察

| 轴 | 怎么判断 | 有/无 | 取值/范围 + source_type / 来源 · 或理由 |
|---|---|---|---|
| ① 骨架图 | 加/减会动的 part 或一条边 | 有 | 结构骨架 candidate：rigid wing（origin, forked_anchor）／accordion fold-chain（folding_wing_chain, forked_anchor，多 REVOLUTE fold part）／solar drum（solardrum, forked_anchor，drum part 替换翼）；antenna：two-limb dual dish（origin）／single central-mast boom（single_boom_dish, forked_anchor，part/joint 计数 2→1）。全部 source-backed。 |
| └ multiplicity | 同构件 ×N | 有 | 见 §8：panel_count {3,4,5}（origin/panels5），dish_count {2,4}（origin/dishes4）。 |
| ② 关节类型 | 图不变，边换 type/轴 | 有 | array drive CONTINUOUS(Y)（origin）↔ REVOLUTE(Y,±slew)（wing_revolute）↔ drum CONTINUOUS(X)（solardrum）；dish gimbal REVOLUTE(Y)（origin）↔ PRISMATIC(arm-axis)（dish_prismatic）；fold REVOLUTE(Z)（folding）。全部 forked_anchor；每种类型都在 sweep 中出现。 |
| ③ 主体形态家族 | 图&关节不变，换核心 part 可识别形态原型 | 有 | **两处登记进 slot_choices**：(A) bus 形态 slot — box（origin）/ hex prism（hex_bus）/ cylinder（solardrum-anchored world_knowledge_extrap）；form_subtype = Volumetric Envelope Form ×3。(C) 天线孔径形态 — parabolic saucer（origin, Volumetric Envelope）/ flat phased-array panel（flatpanel_antenna, Planar Boundary Form）。 |
| ④ 表面装饰 | 原型不变叠加表面细节 | 有 | `front_radiator_tile` 3×3 网格、`mli_*` 金箔贴片、`panel_hinge_spar`、drum cell-ring 分隔环、`feed_aperture` — 均为宿主 part visual，随 ③（bus 面）/⑤（缩放）派生位置（tiles 落在 `bus_front_x`/apothem 上，随 bus_scale 移动）。source_type=record_only（origin/hex_bus/solardrum）。 |
| ⑤ 尺寸/行程 | 只连续改尺寸/比例/行程 | 有 | §7 连续 scale：bus_scale[0.85,1.20]、panel_span_scale[0.85,1.15]、dish_radius_scale[0.85,1.15]。关节运动包络（每个非-continuous joint）：gimbal REVOLUTE axis Y，open 双向，[闭合 0, 可行 ±gimbal_range≤0.45]；prismatic deploy axis=arm_dir，[0, 0.35] m；fold REVOLUTE axis Z，[0, fold_upper≤1.4]。`motion_test_plan`：跑 `fail_if_parts_overlap_in_sampled_poses(max_pose_samples=64)`；targeted `ctx.pose` — array drive 转 π/2 出平面、gimbal 转 gimbal_range 位移反射面、prismatic 平移 dish、fold 折叠 chain panel。continuous drive 采 {0,±90°,180°}。 |
| ⑥ 涂装 | 只改材质/颜色 | 有 | 材质大类 painted/metal；配色 ≥5 colorway：`esa_gold`（暗 bus + 金箔 + 蓝板 + 白盘）、`commercial_white`、`military_dark`、`bare_metal_silver`、`copper_bronze`、`deep_space_black`。材质大类覆盖 ≥ ceil(0.5×6)=3。 |

**收尾自检**：0–9 seed 渲染须肉眼见到 box/hex/cyl 三种 bus、saucer 与 flat panel
两种孔径、rigid/folding/drum 三种 power、材质配色多样、fold/gimbal 全程不穿模。

## 采样与覆盖审计

总组合数（distinct slot-choice tuple 上界）：
- bus 3 × power(panel-based 3×panel_count 3 = 9, + drum 1 = 10) × antenna(multi 3×dish_count 2 = 6, + single_boom 1 = 7) = **3 × 10 × 7 = 210**。

理由：210 < 富类别建议 300，因为真实结构词汇在此收敛——所有样本共享同一
「bus + 太阳能驱动 + 可动天线」cell，可动轴只有三根离散槽 + 两根小 multiplicity。
不硬凑组合空间（质量红线：不反推上游变体数量）。report-only，不设 gate。

seed_domain_policy：`procedural_first`。

Procedural Sampling / Sweep Plan：`config_from_seed(seed)` 用
`random.Random(seed)` 依次抽 bus_form、power_module、antenna_module，再按
compatibility 抽 panel_count（power 为 panel-based 时）/ dish_count（antenna 非
single_boom 时，否则 1）、palette、连续 scale。seed 0 pinned 到 origin 母本组合
（box_bus + rigid_wings×3 + dual_dish_revolute×2, esa_gold）作为 documented
regression anchor（sparse override，其余 seed 全 procedural）。random sweep
`0-15`（fast）→ `0-35`（final）→ corner。

Topology target：1000-seed slot-choice tuple 覆盖用于成熟度观察；真实上界 210
（见上），低于 300 的原因为结构词汇收敛，已记录。report-only。

Controlled local parameterization：`bus_scale`（half_y/half_z/front_x 由其
equation 派生，保 mounting 一致）、`panel_span_scale`、`dish_radius_scale`、
`gimbal_range`、`wing_slew`（conditional）、`fold_upper`（conditional）。全部在
`resolve_config` clamp / 派生；不破坏 captured-socket 接口、gimbal 原点、
multiplicity。连续尺寸契约：先采 independent（bus_scale/panel_span/dish_radius/
gimbal_range）→ equation 派生 bus half-extents → conditional 解析 wing_slew/
fold_upper/dish_count。

| item | policy | validator / viewer focus |
|---|---|---|
| sampler | slot 顺序 bus→power→antenna，加权 choice；multiplicity 各自加权 | slot_choices_for_seed 与 build 选择一致 |
| compatibility matrix | single_boom_dish → dish_count=1（gate）；solar_drums → 无 panel_count；bus×power×antenna 正交自由组合 | 无 floating / collision / 轴错误 / max-N / bulky / 可选子件失败 |
| controlled local variation | 6 个 clamp 连续 scale | 比例变化不破坏接口/clearance/support/joint 原点/类别身份 |
| regression overrides | seed 0 = origin 母本（documented anchor）；无其它 | 仅母本 canonical 预览 |
| random sweep | seeds 0-15 fast, 0-35 final, + corner | contract failures; axis_realization; viewer focus |

| slot | candidate_count | ≥2 | ≥3 | 备注 |
|---|---:|---|---|---|
| bus_body | 3 | yes | yes | box/hex/cylinder |
| solar_power | 4 | yes | yes | rigid/revolute/folding/drum |
| antenna | 4 | yes | yes | dish-rev/dish-pris/flatpanel/single-boom |

## Validator

- `slot_choices_for_seed` returns implemented module names (+ panel_count/dish_count axes)
- `config_from_seed` uses deterministic procedural sampling for all ordinary seeds (seed 0 = documented母本 override only)
- compatibility gating prevents illegal combos (single_boom→K=1; drum→no panel_count) in `resolve_config`
- controlled local scales clamped; cannot break captured-socket interfaces, gimbal/drive origin honesty, or multiplicity
- cross-part scale dependencies (bus half-extents) derived in `resolve_config`
- captured boom/barrel overlaps are element-scoped `allow_overlap` (not broad part-level)
- key joints have expected type/axis/range: array drive CONTINUOUS/REVOLUTE(Y) or drum CONTINUOUS(X); gimbal REVOLUTE(Y)/PRISMATIC(arm-axis); fold REVOLUTE(Z)
- copied `array_panel_i` / `reflector_{side}` follow naming + placement policy
- `run_astronomy_satellite_tests` calls `fail_if_parts_overlap_in_sampled_poses` + ≥1 targeted `ctx.pose` per mechanism

## Reject cases

- Wing/dish steered pose collides with the bus at gimbal min/max → shrink `gimbal_range` or move mount off the deck edge.
- folding_wings fully-folded accordion self-intersects (adjacent panels stack) → cap `fold_upper`, element-scoped allow_overlap on hinge-spar↔panel-edge only, never mask panel-body overlap.
- K=4 forward/aft dish barrel floats off a curved cylinder +X face → socket the barrel into the face (allow_overlap) or inset the mount to the apothem/radius.
- Bus form swap leaves tiles/MLI floating off the new face (constant-radius decoration on hex/cyl) → re-anchor tiles to `bus_front_x`/apothem derived from the realized form (Rule 4).
- `single_boom` mast steered past the bus with an over-wide range → clamp; the mast must clear the wings.
- Downgrading `dish_saucer` LatheGeometry / drum lathe / hex ExtrudeGeometry to crude Box/Cylinder (Rule 3 violation).

## 与相邻类别的边界

- 不该混入：**Astronomy / Antenna dish**（地面站抛物面天线在 az-el 基座/三脚架上，整体是地面对象；Satellite 的 dish 只是自由飞行 bus 上的次级可动附件）。
- 不该混入：**Astronomy / Space shuttle / Return capsule**（有翼/钝头再入体，非 bus+附件拓扑）。
- 不该混入：一个只旋转的 az-el 抛物面（无 bus、无太阳翼、无 MLI/tile 身份特征）。

## 审核记录
| 项 | 结论 |
|---|---|
| reviewer status | pending |
| reviewer notes | solardrum 未在上游 `variant_pool_checklist.md`（8 变体）内，但被 PATHFINDER 任务书列为确认池并作为 cylinder_bus ③ 锚点采纳；待人工与上游 checklist 对齐。 |

## 模板实现备注（可选）

- bus half-extents（half_y/half_z/front_x）single-sourced in `ResolvedConfig`（Contract 3c），solar/antenna 挂点全部从中派生，bus 形态正交。
- captured boom/barrel socket → 原始 joint（no MatingContract, grandfathered）+ element-scoped `allow_overlap`，与全部 5 星源一致（Rule 2 例外）。
- 所有 K 个反射面共享一个 `dish_mesh` / `arm_mesh`；两翼共享 panel box 几何；drum 壳共享 —— 保编译预算。
- folding_wings 优先考虑 `_mechanisms.coupled_chain`（concertina 惯用）；首版用 capped `fold_upper` + hinge-spar allow_overlap + targeted fold pose test，若 sampled-pose 穿模再切换 coupled_chain / clamp_joint_limits。
- 组装走 `_modular.assemble(..., selection_mode="anchor_choices")`：bus root 声明 downstream；solar/antenna 只声明 downstream（re-export bus）→ 无自动 chain joint，各模块发原始 joint 到 bus（parallel-children，同 Tipping_Barrow 惯用）。

## Module Source Index

| source_id | slot | module | sample_id | model.py 来源 | 采纳用途 |
|---|---|---|---|---|---|
| S1 | A/B/C | box_bus + rigid_wings + dual_dish_revolute | `rec_a-large-esa-...8d718b55` (origin 母本) | L48-L362 | bus part tree, 翼 part tree + CONTINUOUS drive, saucer 反射面 + REVOLUTE gimbal, 全部 test 语义 |
| S2 | B mult | panel_count=5 | `rec_satellite_var_panels5` | L33 | panel_count multiplicity 上界 |
| S3 | C mult | dish_count=4 | `rec_satellite_var_dishes4` | L42-L64, L221-L310 | dish_count=4 forward/aft DISH_CONFIG + allow_overlap |
| S4 | B ② | revolute_wings | `rec_satellite_var_wing_revolute` | L185-L197 | REVOLUTE array drive + 有界 slew |
| S5 | C ② | dual_dish_prismatic | `rec_satellite_var_dish_prismatic` | L236-L254 | PRISMATIC 伸缩展开 + arm-axis |
| S6 | C ③ | dual_flatpanel | `rec_satellite_var_flatpanel_antenna` | L179-L219 | 平板相控阵孔径（Planar Boundary Form） |
| S7 | A ③ | hex_bus | `rec_satellite_var_hex_bus` | L46-L58, L72-L79, L117-L166 | 六棱柱 bus ExtrudeGeometry + 面重锚 tiles/MLI |
| S8 | C ① | single_boom_dish | `rec_satellite_var_single_boom_dish` | L42, L74-L90, L201-L245 | 单中央桅杆天线骨架 |
| S9 | B ① | folding_wings | `rec_satellite_var_folding_wing_chain` | L136-L246 | 手风琴折叠翼 REVOLUTE chain |
| S10 | A ③ / B | cylinder_bus anchor + solar_drums | `rec_satellite_var_solardrum` | L33-L37, L135-L235 | 圆柱 lathe 几何（cylinder_bus 锚）+ solar drum 模块 CONTINUOUS X drive |
