# Grenade Spec

## 元信息
| 项 | 值 |
|---|---|
| slug | `grenade` |
| template path | `agent/templates/Military_Granade.py` |
| test path (optional) | `tests/agent/test_grenade_template.py` |
| stage | `SPEC_ONLY_DRAFT` |
| status | `pending` |
| __modular__ | `True` |
| pattern | `mixed` |

`mixed`: a linear root chain `body_form → fuze` plus a `body_surface` overlay slot
parented into the body, plus a multiplicity axis (fragmentation rows) that lives
inside the `frag_knob_grid` surface module. The body spine (ovoid lathe vs
cylinder/hex tube) is the master choice that gates which surface and fuze
candidates are compatible.

## 5 星样本阅读摘要
| 项 | 值 |
|---|---|
| five_star_total | 10 |
| read_count | 10 |
| read_scope | all 5-star samples in this category (2 converged parents + 8 converged workbench variants); each record's `model.py` revision `rev_000001` read in full |
| source_index_policy | only adopted module sources are indexed below |

阅读摘要（10/10 读完，全部 rating=5）：

- **两条互不兼容的 body spine.** 所有样本归为两族。
  - **mk2-ovoid spine**（mk2 parent / taperegg / smoothovoid / ventovoid / frag3row /
    frag7row）：身体是 `cq.Workplane("XZ")` lathe `revolve` 出来的 ovoid/teardrop
    mesh（`_lathe(BODY_PROFILE)`），grounded z=0，BODY_TOP_Z≈0.085，整体高≈0.115 m，
    dia≈0.060 m。fuze 是一个 `mesh_from_cadquery(_fuze_shape())` Compound（threaded
    collar + rectangular pivot housing + lug + axle），lever 是 lathe-strip mesh，
    pin 是 cotter mesh，ring 是 torus mesh。所有 fuze/lever/pin/ring helper 在
    mk2 / taperegg / smoothovoid / ventovoid / frag3row / frag7row 之间**逐字相同**。
  - **m84-cylinder spine**（m84 parent / smoothcyl / fragcyl / bailfuze）：身体是
    bottom hex cap + annular/solid shell + top hex cap + 一串 primitive visuals
    （`Cylinder`/`Box`），全部挂在单个 `body` part 上，grounded z=0，高≈0.131 m，
    tube dia≈0.048，hex across-corners≈0.056。fuze 是 silver `fuze_collar`+`fuze_body`
    primitive cylinders + `lever_lug_*` boxes，全部是 `body` 的 visuals（不是独立
    part）。lever/pin/ring 用 primitive `Box`/`Cylinder` + bent-wire `tube_from_spline_points`。
- **不变运动契约（跨全部 10 个样本验证）.** 每个样本都有 lever REVOLUTE（axis -Y，
  0..~1.57）、pull pin PRISMATIC（axis +Y，0..0.020 m）、ring/bail REVOLUTE。pin 的
  父子链有两种：ring-on-pin（mk2 族 + m84 perforated/smooth/frag：ring 是 pin 的
  child，pin extract 时 ring 跟着走）和 bail-on-body（bailfuze：单 pin + 一个三角
  bail，bail 是 body 的 child，revolute swivel）。type profile 始终是 rev/pris/rev。
- **Surface 是独立可替换层.** 同一个 body spine 可以配不同表面：mk2-ovoid 可以
  frag-knob-grid（fused mesh）、smooth-shell（无纹理）、perforated-vent（boolean
  孔 + 内 charge tube）；m84-cylinder 可以 perforated-vent（boolean 孔）、smooth、
  frag-knob-grid（per-knob FIXED visuals）。fragcyl 和 ventovoid 是显式的 cross
  组合抽检（frag-on-cyl、vent-on-ovoid），证明 surface 可跨 spine 移植。
- **Multiplicity = fragmentation 行数.** frag3row/parent(5)/frag7row 给出
  KNOB_ROWS_Z 长度 {3,5,7}（mk2-spine），fragcyl 给出 N_KNOB_ROWS=6（cyl-spine）。
  列固定 8。knob 全部 FIXED 进 body，无 articulation。
- **未单独成 slot 的差异**：palette（olive/sand/black 等只换 rgba）、整体尺寸、
  vent 孔数（HOLE_COLS 8↔10）、装饰（neck band / stencil / mid band / charge tube /
  red insert）都是 module-local 装饰或 palette，不是独立 candidate。

## 核心身份

Grenade 是一种手投爆炸物（hand grenade）：一个 grounded、直立的小型 body
（高 ~0.11–0.13 m，dia ~0.05–0.06 m），顶部一个 fuze 总成，外面挂着一套
**safety train**：一根 safety lever（spoon/striker lever）压在 body 侧面、绕水平
轴 REVOLUTE 释放；一根（或两根）safety pull pin 沿自身轴 PRISMATIC 抽出；一个
pull ring（或 bail）绕 pin 轴 / body 轴 REVOLUTE 摆动。body 表面可能是光面铸体、
fragmentation 凸纹（pineapple 波纹格）或穿孔散气壳。

成熟域：单 body + 顶置 fuze，lever/pin/ring 三件套始终存在且保持
**lever REVOLUTE / pin PRISMATIC / ring(或 bail) REVOLUTE** 的不变运动契约。

不该混入的相邻类别见「与相邻类别的边界」。

## 采用源码索引（Adopted Source Index）
| source_id | sample_id | model.py 来源 | 采纳用途 |
|---|---|---|---|
| S1 | `rec_model-an-mk2-style-pineapple-fragmentation-hand-_20260610_080226_230556_a8628c01` | `data/records/rec_model-an-mk2-style-pineapple-fragmentation-hand-_20260610_080226_230556_a8628c01/revisions/rev_000001/model.py:L56-L457` | ovoid_frag body spine; frag_knob_grid surface (fused mesh); mk2 single-pin/single-ring fuze train |
| S2 | `rec_model-an-m84-style-stun-flashbang-grenade-about-_20260610_080253_506027_b1beda97` | `data/records/rec_model-an-m84-style-stun-flashbang-grenade-about-_20260610_080253_506027_b1beda97/revisions/rev_000001/model.py:L48-L532` | cylindrical_tube body spine (hex caps); perforated_vent surface; m84 twin-pin/twin-ring fuze train |
| S3 | `rec_grenade_var_taperegg` | `data/records/rec_grenade_var_taperegg/revisions/rev_000001/model.py:L57-L113` | tapered_ovoid body profile (teardrop egg, narrow base tip) |
| S4 | `rec_grenade_var_smoothovoid` | `data/records/rec_grenade_var_smoothovoid/revisions/rev_000001/model.py:L89-L99` | smooth_shell surface on ovoid spine (lathe only, no knobs) |
| S5 | `rec_grenade_var_smoothcyl` | `data/records/rec_grenade_var_smoothcyl/revisions/rev_000001/model.py:L91-L94` | smooth_shell surface on cylinder spine (solid cylinder, no holes) |
| S6 | `rec_grenade_var_fragcyl` | `data/records/rec_grenade_var_fragcyl/revisions/rev_000001/model.py:L111-L260` | frag_knob_grid surface on cylinder spine (per-knob FIXED visuals, N_KNOB_ROWS×N_KNOB_COLS) |
| S7 | `rec_grenade_var_ventovoid` | `data/records/rec_grenade_var_ventovoid/revisions/rev_000001/model.py:L124-L207` | perforated_vent surface on ovoid spine (hollow shell + radial bore + inner charge tube) |
| S8 | `rec_grenade_var_bailfuze` | `data/records/rec_grenade_var_bailfuze/revisions/rev_000001/model.py:L136-L350` | single-pin triangular bail fuze train (bail child of body, revolute swivel) |
| S9 | `rec_grenade_var_frag3row` | `data/records/rec_grenade_var_frag3row/revisions/rev_000001/model.py:L87` | frag-row multiplicity N=3 (KNOB_ROWS_Z length 3) |
| S10 | `rec_grenade_var_frag7row` | `data/records/rec_grenade_var_frag7row/revisions/rev_000001/model.py:L87-L93` | frag-row multiplicity N=7 (KNOB_ROWS_Z length 7, knob height reduced) |

## 槽位 + 候选模块表

### Slot A：body_form（root spine — master choice）

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| ovoid_frag | S1 `rec_model-an-mk2-...-a8628c01` | L56-L162 (`BODY_PROFILE` + `_lathe` + `_interp_radius`/`_surface_normal`) | eligible if compatible | `cq.Workplane("XZ")` lathe revolve of ovoid `BODY_PROFILE`; widest mid-body r≈0.030; flat rounded base at z=0; BODY_TOP_Z=0.085; emits single `grenade_body` part as a mesh visual; mk2 fuze family attaches at neck |
| cylindrical_tube | S2 `rec_model-an-m84-...-b1beda97` | L48-L213 (`_hex_prism` + hex-cap/shell emits in `body`) | eligible if compatible | straight ~0.048 dia annular/solid tube with hex bottom+top caps (`_hex_prism`, across-corners 0.056); ~0.131 m tall; all visuals (caps, shell, fuze, stencil) on one `body` part; m84 fuze family attaches at top cap |
| tapered_ovoid | S3 `rec_grenade_var_taperegg` | L57-L113 (`BODY_PROFILE` teardrop + reused mk2 lathe) | eligible if compatible | steeper teardrop `BODY_PROFILE`: narrow rounded base tip r=0.006, widest ~73% up; `base_dia < 0.55·belly_dia`; same lathe pipeline as ovoid_frag → a distinct profile family, NOT just a size tweak (different part silhouette + lever clearance polyline) |

理由：3 candidates，覆盖两条 spine（lathe vs hex-tube）+ lathe 家族内的 teardrop
profile 变体。ovoid_frag 和 tapered_ovoid 共享 lathe pipeline 但 silhouette /
base-taper / lever clearance polyline 结构不同。

### Slot B：body_surface（overlay layer parented into the body)

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| frag_knob_grid | S1 `rec_model-an-mk2-...-a8628c01` (ovoid) / S6 `rec_grenade_var_fragcyl` (cyl) | S1 L140-L162 (`_body_shape` knob loop, fused into mesh); S6 L111-L260 (`_fragmentation_knob` + per-knob FIXED visual loop) | eligible if compatible | KNOB_ROWS×KNOB_COLS beveled knobs over recessed core (waffle grid). On ovoid spine: knobs fused into the `grenade_body` Compound. On cyl spine: per-knob named visuals `frag_knob_{i_row}_{i_col}` placed radially. Carries the frag-row multiplicity axis. |
| smooth_shell | S4 `rec_grenade_var_smoothovoid` (ovoid) / S5 `rec_grenade_var_smoothcyl` (cyl) | S4 L89-L99 (`_body_shape` = lathe only); S5 L91-L94 (`_smooth_shell` solid cylinder) | eligible if compatible | unbroken cast shell, no surface texture, no holes. Ovoid: `_lathe(BODY_PROFILE)` only. Cyl: solid `circle().extrude()`. Zero knob/hole multiplicity. |
| perforated_vent_shell | S2 `rec_model-an-m84-...-b1beda97` (cyl) / S7 `rec_grenade_var_ventovoid` (ovoid) | S2 L100-L132 (`_perforated_shell`/`_mid_band`, 3×5 boolean cuts); S7 L124-L207 (`_radial_hole_cutter` + hollow shell + inner `_charge_tube_shape`) | eligible if compatible | annular shell with HOLE_ROWS×HOLE_COLS boolean-cut radial vent holes revealing an inner charge tube; brown mid-band wraps a row; optional red flash insert. Holes are boolean cuts (not parts) → no part-level multiplicity. |

理由：3 candidates，每个都有 ovoid+cyl 两条 spine 的来源（cross 组合抽检由 fragcyl /
ventovoid 提供），证明 surface 层可跨 spine 移植。

### Slot C：fuze（safety train — chained on top of the body)

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| mk2_single_pin_ring | S1 `rec_model-an-mk2-...-a8628c01` | L165-L351 (`_fuze_shape`/`_lever_shape`/`_pin_shape`/`_ring_shape` + 3 articulations L319-L349) | eligible if compatible | threaded collar + rect pivot housing/lug/axle mesh; safety_lever(spoon) REVOLUTE -Y; single cotter safety_pin PRISMATIC +Y; single torus pull_ring REVOLUTE, child of pin. 4 parts (body+lever+pin+ring), 3 joints. |
| m84_twin_pin_ring | S2 `rec_model-an-m84-...-b1beda97` | L156-L340 (`_add_pin_visuals` + lever/twin-pin/twin-ring emits + 5 articulations L268-L339) | eligible if compatible | silver collar+body cylinders + lever lugs; safety_lever REVOLUTE -Y; primary_pin + secondary_pin PRISMATIC on two horizontal axes ~75° apart; each pin carries a bent-wire pull ring REVOLUTE child. 6 parts, 5 joints. |
| single_pin_bail | S8 `rec_grenade_var_bailfuze` | L136-L350 (`_bail_ring_mesh` + lever/pin/bail emits + 3 articulations L290-L350) | eligible if compatible | collar+body + lever lugs + bail lugs; safety_lever REVOLUTE -Y; single safety_pin PRISMATIC +Y; one triangular bent-wire bail REVOLUTE swivel whose parent is `body` (axis -X, 0..2.8). 4 parts, 3 joints — distinct ring-on-body topology vs ring-on-pin. |

理由：3 candidates；joint count/topology 都不同（3 vs 5 joints；ring-on-pin vs
twin ring-on-pin vs bail-on-body），全部保持 rev/pris/rev type profile。

## 槽位图（slot graph）

pattern: mixed (root chain + parented overlay + intra-module multiplicity)

```
body_form (Slot A, root, grounded z=0)
    │
    ├─[FIXED / parented overlay: surface visuals laid on the body lathe/tube
    │   normal; on ovoid spine fused into the body mesh, on cyl spine emitted
    │   as FIXED child visuals of the body]──> body_surface (Slot B)
    │                                              └─[multiplicity: frag rows]
    │
    └─[chain mount at fuze neck/top cap, FIXED-into-body collar]──> fuze (Slot C)
            ├─ safety_lever  REVOLUTE  axis -Y   @ top pivot lug   0..~1.571
            ├─ safety_pin(s) PRISMATIC axis +Y   @ fuze bore       0..0.020
            └─ pull_ring/bail REVOLUTE            @ pin shaft (or body)  ~±1.05 / 0..2.8
```

说明：

- **顺序 / parent**：`body_form` 是 root，grounded 在 z=0。`body_surface` 不是串接
  child——它是叠加在 body 上的表面层（ovoid spine 直接 fuse 进 body mesh；cyl spine
  作为 body 的 FIXED child visuals 发射）。`fuze` 总成的 collar/housing/lugs 都是
  FIXED 进 body 的 visuals/mesh；只有 lever/pin/ring/bail 是独立 articulated parts。
- **接口点位**：
  - body↔surface：body 的 outward surface normal（ovoid: `_surface_normal(z)` 沿
    lathe profile；cyl: 径向 `BODY_R_OUT`）。surface 元素坐在该法线上、轻微 embed。
  - body↔fuze：ovoid 在 neck（z≈BODY_TOP_Z，collar at COLLAR_Z0）；cyl 在 top cap
    上方（FUZE_COLLAR_Z0≈0.112）。fuze collar 是 FIXED-into-body anchor。
  - fuze 内部：lever pivot @ top lug（axle 水平 -Y）；pin bore @ housing/fuze_body；
    ring swivel @ pin head eye（ring-on-pin）或 bail swivel @ body bail-lug（bail-on-body）。
- **joint type / axis / range**：lever REVOLUTE axis (0,-1,0) lower=0 upper≈1.571；
  pin PRISMATIC axis (0,+1,0) lower=0 upper=0.020（twin 时第二根 yaw≈75°）；ring
  REVOLUTE axis (0,+1,0) span≈2.1（±1.05）；bail REVOLUTE axis (-1,0,0) 0..2.8。
- **互斥 / 派生**：Slot A 的 spine 选择**门控** Slot B 与 Slot C（compatibility
  matrix）。tapered_ovoid 与 ovoid_frag 共用 mk2 fuze 家族；cylindrical_tube 用
  m84 fuze 家族。frag-row 多样性是 Slot B `frag_knob_grid` 的 module-local 轴。

## 每槽位 Module Emits / Interfaces

### Slot A / module ovoid_frag
| emits | 描述 | 来源 |
|---|---|---|
| parts | `grenade_body`（root，lathe mesh）；neck_band 装饰 visual | S1 / L278-L294 |
| internal joints | 无（root 不可动） | S1 |
| upstream interface | 无（root，grounded z=0） | S1 / L398-L401 |
| downstream interface | surface overlay on lathe normal；fuze collar FIXED at neck (z≈0.085) | S1 / L165-L192 |

### Slot A / module cylindrical_tube
| emits | 描述 | 来源 |
|---|---|---|
| parts | `body`（root；hex caps + shell + stencil + fuze visuals all FIXED here） | S2 / L176-L244 |
| internal joints | 无 | S2 |
| upstream interface | 无（root，grounded z=0） | S2 / L401-L411 |
| downstream interface | surface overlay radial @ BODY_R_OUT；fuze stack FIXED above top cap (z≈0.112) | S2 / L224-L244 |

### Slot A / module tapered_ovoid
| emits | 描述 | 来源 |
|---|---|---|
| parts | `grenade_body`（teardrop lathe mesh）；neck_band | S3 / L57-L113, reused emits as ovoid_frag |
| internal joints | 无 | S3 |
| upstream interface | 无（root，grounded z=0；narrow base tip r=0.006） | S3 / L407-L419 |
| downstream interface | surface overlay；fuze collar FIXED at neck；lever clearance polyline tracks the steeper taper | S3 / L101-L113 |

### Slot B / module frag_knob_grid
| emits | 描述 | 来源 |
|---|---|---|
| parts | ovoid: fused into `grenade_body` mesh (no new part). cyl: FIXED child visuals `frag_knob_{i_row}_{i_col}` on `body` | S1 / L140-L162; S6 / L201-L223 |
| internal joints | knobs are FIXED into body (no articulation) | S6 / L208-L223 |
| upstream interface | seats on body surface normal, slight embed (KNOB_EMBED≈0.001) | S6 / L213-L220 |
| downstream interface | none (terminal overlay) | — |

### Slot B / module smooth_shell
| emits | 描述 | 来源 |
|---|---|---|
| parts | ovoid: `grenade_body` = lathe(BODY_PROFILE) only. cyl: `smooth_shell` visual on `body` | S4 / L97-L99; S5 / L91-L94, L145-L150 |
| internal joints | 无 | — |
| upstream interface | covers body silhouette, no protrusions | S4 / L337-L342 |
| downstream interface | none | — |

### Slot B / module perforated_vent_shell
| emits | 描述 | 来源 |
|---|---|---|
| parts | ovoid: `perforated_shell` mesh + inner `charge_tube` mesh (hollow). cyl: `perforated_shell` + `charge_tube` + `mid_band` + `flash_insert` visuals | S7 / L313-L344; S2 / L183-L207 |
| internal joints | 无（holes are boolean cuts） | — |
| upstream interface | shell wraps body; charge tube centered within shell (xy/z within margin) | S7 / L452-L468 |
| downstream interface | none | — |

### Slot C / module mk2_single_pin_ring
| emits | 描述 | 来源 |
|---|---|---|
| parts | `safety_lever`(spoon), `safety_pin`(cotter_pin), `pull_ring`(ring_torus); fuze collar/housing/lug/axle fused into body mesh | S1 / L296-L315, L165-L192 |
| internal joints | `lever_pivot` REVOLUTE -Y 0..1.571; `pin_slide` PRISMATIC +Y 0..0.020; `ring_swivel` REVOLUTE +Y span 2.1 (child of pin) | S1 / L319-L349 |
| upstream interface | collar FIXED into body @ neck (COLLAR_Z0≈0.087) | S1 / L167-L169 |
| downstream interface | none (terminal) | — |

### Slot C / module m84_twin_pin_ring
| emits | 描述 | 来源 |
|---|---|---|
| parts | `safety_lever`, `primary_pin`, `secondary_pin`, `primary_pull_ring`, `secondary_pull_ring`; fuze collar/body/lugs visuals on body | S2 / L249-L331 |
| internal joints | `lever_pivot` REVOLUTE -Y; `primary_pin_slide` + `secondary_pin_slide` PRISMATIC (2nd yaw≈75°); `primary_ring_swing` + `secondary_ring_swing` REVOLUTE (children of pins) | S2 / L268-L339 |
| upstream interface | fuze stack FIXED above top cap (FUZE_COLLAR_Z0≈0.112) | S2 / L224-L244 |
| downstream interface | none | — |

### Slot C / module single_pin_bail
| emits | 描述 | 来源 |
|---|---|---|
| parts | `safety_lever`, `safety_pin`, `bail_ring`(bail_wire + pivot_pin); fuze collar/body/lever-lugs/bail-lugs visuals on body | S8 / L271-L350 |
| internal joints | `lever_pivot` REVOLUTE -Y; `safety_pin_slide` PRISMATIC +Y; `bail_swivel` REVOLUTE -X 0..2.8, parent=body (NOT pin) | S8 / L290-L350 |
| upstream interface | fuze + bail-lug FIXED above top cap; bail pivot @ z≈0.122 above collar | S8 / L259-L266, L342-L350 |
| downstream interface | none | — |

## 参数范围汇总
| 参数 | 类型 | 取值范围 / 候选值 | 标称默认 | 约束类型 | 约束 / 函数 | 来源 |
|---|---|---|---|---|---|---|
| `body_form_choice` | enum | `ovoid_frag` / `cylindrical_tube` / `tapered_ovoid` | — | choice | deterministic procedural sampler; master spine choice | Slot A table |
| `body_surface_choice` | enum | `frag_knob_grid` / `smooth_shell` / `perforated_vent_shell` | — | choice | sampler, gated by spine compatibility matrix | Slot B table |
| `fuze_choice` | enum | `mk2_single_pin_ring` / `m84_twin_pin_ring` / `single_pin_bail` | — | choice | sampler, gated by spine compatibility matrix | Slot C table |
| `frag_row_count` | int | `[3, 7]` | 5 | conditional | only sampled when `body_surface_choice == frag_knob_grid`; per-N weighted draw; copied into `slot_choices` | S1/S9/S10, S6 |
| `palette_style` | enum | `olive_drab` / `flat_black` / `desert_sand` / `od_with_ochre_band` / `weathered_silver_fuze` (≥3; target 4-6) | `olive_drab` | choice | recolors materials only; no topology change | S1 L272-L276, S2 L80-L88, S8 L79-L87 |
| `body_height_scale` | float | [0.92, 1.08] | 1.0 | independent | uniform sample, clamp; scales body z-extent + fuze/lever/pin/ring z-anchors together (keeps grounding + height in spec band) | S1 L37-L68, S2 L52-L65 |
| `body_dia_scale` | float | [0.92, 1.08] | 1.0 | independent | uniform sample, clamp; scales radial body profile + surface-overlay radii together | S1 L56-L68, S2 L48-L52 |
| `knob_height_scale` | float | derived | 1.0 | equation | `= clamp(1.0 · (5 / frag_row_count)^0.5, 0.7, 1.1)` — denser rows ⇒ shorter knobs (matches frag7row L90 reducing KNOB_H to 0.0082) | S10 L87-L93 |
| `fuze_mount_z` | float | derived | — | equation | `= f(body_height_scale, spine)`: ovoid → neck (BODY_TOP_Z·hscale); cyl → CAP_TOP top + fuze stack | S1 L37, S2 L55-L65 |
| (—) | constraint | — | — | inequality | grounding: `body_aabb.zmin ∈ [-1e-6, 0.001]`; overall height after scales ∈ ovoid [0.110,0.120] / cyl [0.120,0.140]; violate ⇒ rescale-clamp | S1 L398-L401, S2 L401-L411 |
| (—) | constraint | — | — | inequality | lever clearance: spoon polyline must hug body flank with 1–3 mm running gap across the scaled silhouette; violate ⇒ re-offset polyline outward | S1 L97-L109, S3 L101-L113 |

## Multiplicity / Copy Logic

存在 1 根 multiplicity 轴：fragmentation rows（仅在 `frag_knob_grid` surface 下生效）。

- `count_param`: `frag_row_count` = `len(KNOB_ROWS_Z)`（mk2-spine）或 `N_KNOB_ROWS`（cyl-spine）。
- `N_range`: `[3, 7]`（本小类本轴产品域；测试偏小，产品全程）。来源 N 样本：
  3 = `rec_grenade_var_frag3row`（KNOB_ROWS_Z 长 3）、5 = mk2 parent、7 =
  `rec_grenade_var_frag7row`（KNOB_ROWS_Z 长 7，KNOB_H 减小）；cross frag-on-cyl
  6 = `rec_grenade_var_fragcyl`（N_KNOB_ROWS=6）。
- sampling domain（权重档）：小 N 高频，大 N 稀有 —— 加权偏向 {3,4,5}，{6,7} 稀疏。
  default/typical N=5（来自 parent），seed 0 不特殊。
- copied object: 一个 beveled fragmentation knob（mk2-spine：chamfered box fused
  进 body Compound；cyl-spine：tapered `_fragmentation_knob` loft solid）。
- naming: 概念上每行 `frag_row_i`。mk2-spine 把整圈 knobs fuse 进单个 `frag_body`
  mesh Compound（不暴露独立命名 visual）；cyl-spine 发射 per-knob 命名 visual
  `frag_knob_{i_row}_{i_col}`。
- placement: 等 z 行（KNOB_ROWS_Z，沿 body 高均匀）× 等角列（KNOB_COLS=8，45° 间距，
  mk2 偏移 22.5° 让 lever 子午线落在两列之间），坐在 body lathe/tube 法线上、轻微 embed。
- joint policy: 所有 knobs **FIXED 进 body，无 articulation**；只有 fuze 的
  lever/pin/ring/bail 是 articulated parts。
- source/gating: 该轴只在 `body_surface_choice == frag_knob_grid` 时存在；smooth_shell
  和 perforated_vent_shell 没有 part-level 复制（vent 孔是 HOLE_ROWS×HOLE_COLS 的
  boolean 切割，不是 parts，因此不进 multiplicity 轴）。列固定 8，不作为独立 count 轴。

## 拓扑多样性审计

总组合数（受 compatibility matrix 约束后）：

- spine-gated 合法 (A,B,C) 三元组：
  - ovoid_frag / tapered_ovoid（2 个 lathe spine）× {frag_knob_grid, smooth_shell,
    perforated_vent_shell}（3）× {mk2_single_pin_ring, single_pin_bail*}
  - cylindrical_tube（1）× {frag_knob_grid, smooth_shell, perforated_vent_shell}（3）
    × {m84_twin_pin_ring, single_pin_bail}（2）
- 名义 A×B×C = 3×3×3 = 27；compatibility matrix 砍掉非法 fuze↔spine 组合后，
  legal (A,B,C) ≈ 2(lathe)×3×2 + 1(cyl)×3×2 = 12 + 6 = 18 个 distinct slot 三元组。
- 加上 frag_knob_grid 下的 frag_row_count {3,4,5,6,7}（5 档）只对带 frag 的三元组
  额外乘开：frag 三元组有 (2 lathe + 1 cyl)×(可用 fuze) ≈ 3 spine-fuze 组合 × 5 N
  = 15 个额外 distinct（slot_choices 把 N 编进 module 名 `frag_{n}row_grid`）。
- distinct slot_choice tuples 估计 ≈ 18(non-frag/各-frag base) + 增量 ≈ **30+**，

理由：18 个 spine-gated 合法三元组本身已 >10；frag-row 多样性再把 frag 子集乘开。
即便 sampler 只命中其中一半，也稳过 10。

seed_domain_policy：procedural_first

Procedural Sampling / Sweep Plan：`config_from_seed` 对每个 seed：
1. 加权抽 `body_form_choice`（spine = master）。
2. 由 spine 解析 Slot B / Slot C 的合法候选集（compatibility matrix），在合法集内加权抽。
3. 若 `body_surface_choice == frag_knob_grid`，对 `frag_row_count` 做 per-N 加权抽
   （小 N 偏多），编进 `slot_choices`（module 名带 N）。
4. 抽 `palette_style`；抽 `body_height_scale`/`body_dia_scale`（independent），
   派生 `knob_height_scale`/`fuze_mount_z`（equation），用 grounding/height/clearance
   inequality 投影回缩，无法满足则拒绝重采。
`slot_choices_for_seed` 返回 `[(body_form, mod), (body_surface, mod[+N]), (fuze, mod)]`。
连续 scale 默认不进 slot_choices（不改拓扑等价类）。

Topology target：1000-seed slot choice tuple distinct 富类别建议 ≥300（report-only）。本类别因 spine-gated 合法
三元组只有 ~18 个 + frag-row 增量，distinct 上限受类别结构约束，预计落在 30–40；
低于 300 的原因是 grenade 拓扑空间本身有限（单 body + 顶 fuze + 固定 safety train
契约），不是采样不足 —— 这是类别 identity 约束。

若使用 regression overrides：none（除非 sweep 暴露具体失败 seed，届时按理由稀疏登记）。

Controlled local parameterization：初版应含 `body_height_scale`、`body_dia_scale`
（independent, [0.92,1.08] clamp）、派生 `knob_height_scale`（equation,
clamp[0.7,1.1]）和 `fuze_mount_z`（equation）。这些只改安全比例 / knob 长度 /
fuze 高度 anchor，不改 part tree、joint topology、frag-row 数或接口语义；grounding /
overall-height / lever-clearance inequality 在 `resolve_config` 内求解，违反则回缩或拒绝。

| item | policy | validator / viewer focus |
|---|---|---|
| sampler | spine master → compat-gated B/C → frag_row weighted (small-N biased) → palette → scales | slot_choices_for_seed matches build choices |
| compatibility matrix | mk2_single_pin_ring ↔ {ovoid_frag, tapered_ovoid} only; m84_twin_pin_ring ↔ cylindrical_tube only; single_pin_bail ↔ all spines; frag_row_count only when surface=frag_knob_grid; vent/smooth carry no part multiplicity | no illegal fuze-on-wrong-spine, no floating fuze, no frag-rows on smooth/vent |
| controlled local variation | body_height_scale / body_dia_scale independent; knob_height_scale / fuze_mount_z derived; all clamped to grounding + height + clearance bands | proportions vary without breaking grounding, fuze seat, lever clearance, joint origin, identity |
| regression overrides | none | only previously-failed or reviewer-selected seeds |
| random sweep | seeds 0-49 initial pass; 0-999 maturity audit | contract failures (lever/pin/ring joint types, grounding, fuze seat) |

| slot | candidate_count | 是否 ≥2 | 是否 ≥3 | 备注 |
|---|---:|---|---|---|
| A body_form | 3 | yes | yes | 2 lathe spines + 1 hex-tube spine |
| B body_surface | 3 | yes | yes | each with ovoid+cyl sources |
| C fuze | 3 | yes | yes | 3/5/3 joints; rev/pris/rev profile invariant |

## Validator

- `slot_choices_for_seed` returns implemented module names (frag module name encodes N when surface=frag_knob_grid)
- `config_from_seed` uses deterministic procedural sampling for all ordinary seeds; seed=0 not special
- compatibility matrix prevents illegal fuze↔spine combos and frag-rows on non-frag surfaces
- optional regression overrides are sparse and justified (currently none)
- final template does not endlessly cycle a small curated table as the main seed domain
- controlled local scale params (`body_height_scale`, `body_dia_scale`, derived `knob_height_scale`/`fuze_mount_z`) are clamped and cannot break grounding, fuze seat, lever clearance, joint origin, or frag-row multiplicity
- cross-part scale dependencies (knob_height ← frag_row_count; fuze_mount_z ← body_height_scale) resolved in `resolve_config`
- critical InterfaceSpec / MatingContract points exist: body↔fuze collar seat, body↔surface normal embed, lever↔lug pivot, pin↔bore, ring↔pin-eye (or bail↔body-lug)
- key joints have expected type / axis / range: lever REVOLUTE axis -Y 0..~1.571; pin(s) PRISMATIC axis +Y 0..0.020; ring REVOLUTE axis +Y span ~2.1 (child of pin) OR bail REVOLUTE axis -X 0..2.8 (child of body)
- copied objects (frag knobs) follow naming/placement policy (equal-z rows × equal-angle cols, FIXED into body, no articulation)
- intentional captured-pin overlaps declared element-scoped: pin-in-bore, lever-ear-on-axle/lug, ring-through-eye, bail-pin-through-lug, charge-tube-ring-on-shell

## Reject cases

- fuze articulated as anything other than lever REVOLUTE / pin PRISMATIC / ring(or bail) REVOLUTE — breaks the invariant motion contract.
- m84 twin-pin fuze placed on an ovoid lathe spine, or mk2 single-pin-housing fuze on a hex-tube spine — illegal spine↔fuze combo (must be gated out).
- frag_row_count exposed/sampled when surface is smooth_shell or perforated_vent_shell (those have no part-level knob copy; vent holes are boolean cuts).
- body not grounded (zmin far from 0) or overall height outside the spine band after scaling — proportions/grounding broken.
- fuze collar floating above the body neck/top-cap instead of FIXED-seated (gap at the mount).
- pull ring NOT a child of the pin in ring-on-pin fuzes (would not extract with the pin) — except `single_pin_bail`, where the bail is intentionally a child of the body.
- frag knobs emitted as articulated parts instead of FIXED into the body.
- lever clearance polyline collides with / floats off the scaled body flank (no 1–3 mm running gap).
- treating palette/size as topology candidates (recolor or rescale is not a new module).

## 与相邻类别的边界

- 不该混入：Military/Mine 或 IED（无 hand-held lever/pin/ring safety train；触发机构不同，通常无顶置 fuze 总成 + spoon lever）。
- 不该混入：Military/Bomb 或 Artillery_shell（弹体带尾翼/引信螺纹但无 spoon lever + pull-ring 手投 safety train；尺度远大于手投 grenade ~0.13 m）。
- 不该混入：Tools/Spray_can 或 Container/Gas_cylinder（外形可能相似的直立罐体，但无 lever REVOLUTE + pin PRISMATIC + ring REVOLUTE 运动契约，顶部是阀/喷嘴而非 fuze 安全机构）。
- 不该混入：40mm grenade-launcher round / rifle grenade（无手投 spoon lever + pull ring；通过发射器击发，结构是带 cartridge/projectile 的弹药）。

## 审核记录
| 项 | 结论 |
|---|---|
| reviewer status | pending |
| reviewer notes | — |

## 模板实现备注（可选）

- mk2-spine 的 fuze/lever/pin/ring helper（`_fuze_shape`/`_lever_shape`/`_pin_shape`/
  `_ring_shape`）在 S1/S3/S4/S7/S9/S10 之间逐字相同 → 抽成共享 mk2 fuze factory；
  cyl-spine 的 `_add_pin_visuals`/`_ring_mesh` 在 S2/S5/S6 之间逐字相同 → 共享 m84
  fuze factory。bail 的 `_bail_ring_mesh` 是 single_pin_bail 专属。
- frag_knob_grid 有两套发射路径（ovoid: fuse 进 body mesh Compound；cyl: per-knob
  FIXED visual）；surface factory 需按 spine 分支选择路径，共享 KNOB_ROWS_Z 计算。
- perforated_vent_shell 在 ovoid spine 需要 hollow shell + inner charge tube +
  support rings（S7），charge-tube-ring↔shell-inner-wall 用 element-scoped allow_overlap
  （S7 L435-L441）；cyl spine 只需 boolean 孔 + solid charge tube cylinder（S2）。
- captured-pin overlaps 需 element-scoped allow_overlap（不要 broad part-level）：
  pin-shaft↔fuze-bore、lever-ear↔axle/lug、ring↔pin-eye、bail-pivot-pin↔bail-lug、
  charge-tube-ring↔shell-inner-wall。参见 S1 L366-L382、S2 L362-L398、S8 L370-L393。
- 暂不进入 seed domain 的组合：full A×B×C 矩阵中被 compatibility matrix 标为非法的
  fuze↔spine 组合（m84-twin-on-ovoid、mk2-housing-on-cyl）不实现、不采样。
- ovoid spine 把 surface+fuze 都 fuse 进单个 `grenade_body` mesh part；cyl spine 把
  caps/shell/fuze/stencil 都作为单个 `body` part 的 visuals —— 两条 spine 的 root
  part 名不同（`grenade_body` vs `body`），实现时统一抽象成 root body part handle。
