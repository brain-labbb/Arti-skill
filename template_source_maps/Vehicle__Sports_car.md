# Vehicle / Sports car — template source map

pattern: 固定 named slots，无模板级复制数量轴（轮恒 ×4、门恒 ×2、转向节恒 ×2 —— 是「跑车」的定义而非可变 N）。共享 kinematic 骨架（前轮 steer+spin、后轮 spin、双门 swing；基线 **4 REVOLUTE + 4 CONTINUOUS**，循环发射）在全部变体中**完全一致**，骨架不拆。模板差异由「车身剖面族 + 门机构 + 大灯 + 尾部 + 涂装」五个 named slot 表达。车身为 `superellipse_side_loft` 截面表（tuple `(y, z_min, z_max, width)`，+Y 为长轴），**主 footprint 槽 = body_profile_family**：不同剖面表（非缩放）才能跨「老爷车 ↔ 楔形超跑」，单缩放只会得到窄楔形。

parents（2 个母资产 —— 本小类是双母族，分别锚定 vintage 与 wedge 两个 profile family）:
- P1 rec_classic-mercedes-300sl-gullwing-sports-car-in-de_20260608_164442_671467_88ed264e ← picture/Vehicle/Sports car_/001.png（经典 Mercedes 300SL Gullwing；vintage_pontoon 剖面表 sect=962db44d：窄车身 width≈1.63、窄轮距 track≈1.49、短轴距 wb≈2.40、直立 greenhouse 占比≈0.35、独立鼓包前后翼子板 + 收腰门槛；**独立 roof part**；Y 轴 gullwing 门）
- P2 rec_bright-yellow-lamborghini-diablo-style-wedge-sup_20260612_133511_811503_655801c5 ← picture/Vehicle/Sports car_/002.png（Lamborghini Diablo wedge；modern_wedge 剖面表 sect=954ba18f：宽体 width≈2.02、宽轮距 track≈1.78、长轴距 wb≈2.65、低矮一体 greenhouse、整合 fastback 车顶无独立 roof part；X 轴微仰 scissor 门）

批次：sports_car（混合：claude-opus 外部 fork + dashscope qwen3.7-max + codex 接管编辑）。24 变体全部 compile=success、`compile --validate` 24/24 通过（0 error，统一 2 warning = allowed-overlap + disconnected-island note）、workbench-only、均含 ≥1 非 fixed joint、仍明确读作跑车。**全部 rating=null（未评级 / 未 sync 5★）。**

## 组合数预审

body_profile_family 4 × door_mechanism 3 × headlight_style 4 × rear_treatment 4 = **192 ≥ 20** ✓（用户口径 door×light×rear×livery = 3×4×4×4 = 192 同样 ≥20）。
无 multiplicity 轴（轮/门/转向节计数固定）。


## Slot 候选覆盖

### Slot A：body_profile_family（主 footprint 槽 —— 车身剖面族，决定老爷车 vs 超跑的真实比例）
| 候选（未来 module） | record_id | 关键 part/helper | 结构特征 | 状态 |
|---|---|---|---|---|
| modern_wedge（基线） | P2 | `superellipse_side_loft(LOWER_SECTIONS)` sect=954ba18f；宽体低矮、一体 fastback 顶、无独立 roof part | 楔形超跑（width 2.02 / track 1.78 / wb 2.65 / GH 0.35） | parent |
| vintage_pontoon | P1 | sect=962db44d；窄身、独立鼓包前后翼子板、收腰门槛、**独立直立 roof part** + windshield/rear_window shell | 老爷车（width 1.63 / track 1.49 / wb 2.40 / GH 0.35，直立高顶）；**证明 loft 能撑老爷车比例（非缩放，是剖面 shape 差异）** | converged（已 6× 复制；但 6 个 car001 截面表 md5 全同=纯换肤，shape 仅 P1 一份） |
| fastback | **rec_sportscar_fastback_body_v01** | 干净单轴 fork（off Diablo P2，仅改 LOWER_SECTIONS→fastback；门=scissor、骨架 7rev+2cont 原样）；body hash c243031a | 911 式后置溜背 | **converged ✓ CLEAN（compile --validate 通过）** |
| organic_teardrop | **rec_sportscar_teardrop_body_v01** | 干净单轴 fork（off Diablo P2，仅改车身→有机水滴；门=scissor、骨架原样）；body hash 8f2fa2e8 | Huayra 式有机曲面 | **converged ✓ CLEAN（compile --validate 通过）** |
| (可选) rounded_hyper | rec_bugatti_chiron_v01 | sect=18394813；圆润 W16、双色 C 线 | Chiron 式圆胖 hypercar | 备选；与 teardrop 拓扑近似，可并入 |

### Slot B：door_mechanism（结构槽 —— 门铰链拓扑）
| 候选 | record_id | 关键 joint | 结构特征 | 状态 |
|---|---|---|---|---|
| scissor_butterfly（基线） | P2 | `DOOR_AXIS=(-1/_AX_NORM, 0, ±_AX_TILT/_AX_NORM)`（横向 X 为主 + Z 微仰）；REVOLUTE 上掀 | 剪刀/蝴蝶门（前铰上掀） | parent |
| gullwing | P1 | `DOOR_AXIS=(0, ±1, 0)`（纵向 Y 轴）；hinge 在顶 HINGE_Z≈0.88；REVOLUTE 上掀 | 鸥翼门（顶脊铰上掀） | converged（仅在 vintage 车身上演示，**与 body 绑定**） |
| dihedral | rec_koenigsegg_regera_v01 / rec_bugatti_chiron_v01 / rec_porsche_911_v01 | `DOOR_AXIS=_DH 归一化`自定义矢量；REVOLUTE 上掀外展 | 二面角/上旋外展门 | 需单轴收敛（仅在 3 个 reskin 车身上演示） |
| (可补) conventional | —— | 竖轴 REVOLUTE 侧开 | 常规侧开门（B 柱铰） | 需补 fork（语料缺；老爷车实际多为侧开门，补此项可去掉 gullwing⟺vintage 的强绑定） |

> ⚠ **门 ⊥ 车身可分离性（已修正 2026-06-21，SPECS 段复核更正）**：原稿误把 Pagani 记为 gullwing——实测 Pagani `DOOR_AXIS` 是横向-X（scissor 族，注释虽写 gullwing 但铰链轴是 scissor）。修正后实况：
> - **scissor** ⊥ body ✓：Diablo(wedge) + Ferrari(sharp-wedge) + Pagani(teardrop) + 两个 body fork（fastback/teardrop 车身上保留 scissor）→ 跨多种车身，已证。
> - **dihedral** ⊥ body ✓：Porsche(fastback) + Koenigsegg(teardrop) + Bugatti(rounded) → 跨 3 车身，已证。
> - **gullwing** ⊥ body ✗ **未证**：唯一来源是 Mercedes(vintage)，单源单车身，从未在别的车身上演示。
>
> 故 scissor / dihedral 可放心作独立 slot；**gullwing 仍 vintage-bound，其与车身的可分离性尚无语料佐证**——是否接受 gullwing 单源、或补一个 external-edit 的 gullwing-on-wedge 数据点，留 spec-review 决定。
>
> ⛔ **流程约束（重要，影响如何造门变体）**：用 `articraft fork`(qwen agent) 造 gullwing-on-wedge / scissor-on-vintage / sidedoor 单轴门变体，**两轮（max-turns 80 / 90）全部失败**。根因非几何（scissor-on-vintage 几何已成：clearance ✓ overlaps ✓），而是 fork-agent harness 把父资产的**门机构 contract（铰链轴 + articulation 名）作为不可改的 scaffold / immutable infrastructure 注入**，改门机构必然违约、agent 无法从可编辑段清除（原话："injected by the compile scaffold... immutable infrastructure code... cannot be cleared"）。**结论：改门机构的变体不能走 `articraft fork` agent 路线，必须走 `articraft external`(external_edit_draft, 直接编辑 model.py) 路线**——现有 gullwing/dihedral reskin（Pagani 等）正是此路线所造。换车身/加部件不受此限（body 不在 contract 内，故 fastback/teardrop fork 成功）。

### Slot C：headlight_style（大灯）
| 候选 | record_id | 关键 part/joint | 结构特征 | 状态 |
|---|---|---|---|---|
| exposed_rect（基线） | P2 | 车头前缘平嵌矩形/条灯（visual，无 joint） | 现代平嵌条灯 | parent |
| pop_up_cover | rec_codex_car002_v07 | **+`cover_panel` + `cover_front_seam` + `cover_outer_seam` + `hinge_pin`**；翻盖 REVOLUTE | 翻灯（带铰活动盖）**改拓扑** | **converged ✓ CLEAN（纯单轴 fork，仅加翻灯盖）** |
| round_fixed | P1 / rec_porsche_911_v01 | 车头圆形灯碗/凸透镜（visual） | 圆灯 | converged（多轴提取） |
| quad_cluster | rec_pagani_huayra_v01 / rec_bugatti_chiron_v01 | 四点圆/LED 灯组（`tail_light_bar` 同族尾灯） | 四灯组/quad-LED | converged（多轴提取） |

### Slot D：rear_treatment（尾部 = 尾翼 × 排气，可拆 2 子槽）
| 候选 | record_id | 关键 part/joint | 结构特征 | 状态 |
|---|---|---|---|---|
| integrated（基线） | P2 | 一体整合尾，无独立大翼 | 整合尾 | parent |
| big_gt_wing | rec_ferrari_f40_v01 / rec_bugatti_chiron_v01 | `wing_blade` + `wing_gurney`（独立 part） | 大 GT 尾翼 | converged（多轴提取） |
| ducktail | rec_porsche_911_v01 | `ducktail_blade` + `ducktail_lip` | 鸭尾扰流 | converged（多轴提取） |
| engine_louver_cover | rec_codex_car002_v09 | **+`cover_panel` + `cover_hinge_pin`**；后舱盖翻板（带铰） | 发动机舱百叶翻盖 **改拓扑** | **converged ✓ CLEAN（纯单轴 fork，仅加后舱翻盖）** |
| 排气子轴 exhaust{side / central_quad} | central_quad: rec_pagani_huayra_v01(`exhaust_surround`) / rec_ferrari_f40_v01(`exhaust_center`+`exhaust_ring_center`) / rec_koenigsegg_regera_v01(`exhaust_ring`) | 中置四出 vs 两侧出 | 可作 D 的正交子槽或并入 | converged（多轴提取） |

### Slot E：livery_palette（涂装 / 材质族 —— 纯外观）
| 候选 | record_id | 结构特征 | 状态 |
|---|---|---|---|
| gloss_solid（基线） | P2 | 单色高光漆 | parent |
| two_tone | rec_bugatti_chiron_v01 | 双色分色（C 线分界） | converged |
| bare_carbon_accent | rec_pagani_huayra_v01 / rec_koenigsegg_regera_v01 | 裸碳纤 + 抛光铝/银饰条 | converged |
| metallic_chrome | P1 / rec_porsche_911_v01 | 金属漆 + 镀铬饰条 | converged（trivial，任意变体可切） |

## Multiplicity / Copy Logic
- **无模板级复制数量逻辑（无 `*_count`）**：核心结构由固定 named slots 表达。轮恒 ×4（前 steer+spin、后 spin）、门恒 ×2、转向节恒 ×2 —— 均为跑车定义而非可变 N。
- module-local 固定循环（非模板轴）：左右镜像子件（门、转向节、轮）经 side=±1 镜像/`zip` 循环发射，不暴露为 count 参数。
- 模板建议 N_range：无（计数全固定）。

## copied object / naming / placement / joint policy
- copied object：左右对称的 wheel/steer_knuckle/door 子件（side=±1 镜像）；body 为单体 loft + boolean 挖腔（cabin cavity + door aperture 切穿侧面）。
- naming：`wheel_{front,rear}_{left,right}_{tire,disc}`、`steer_knuckle_{l,r}`、`door_{left,right}`、`axle_*`；车身随族重命名（`diablo_wedge_supercar` / vintage body / 各 reskin 名）。
- placement：轮触地 z=0；车身沿 +Y（车头 +Y）；左右沿 ±X 镜像。
- joint policy：前轮 = steer(REVOLUTE 竖直 king-pin) ⊕ spin(CONTINUOUS 横向)；后轮 = spin(CONTINUOUS)；门 = REVOLUTE（轴随机构槽：scissor=X 微仰 / gullwing=Y / dihedral=_DH）；直轴杆 hub-to-hub 穿 bored channel。pop_up 翻灯 / engine_louver 后盖 = 各自 REVOLUTE 翻板。

## 排除项（未来 compatibility matrix 素材）+ 完成定义(§8) 现状
**§8 状态（2026-06-21 定稿，variant-review gate 已决定：接受 reskin 证据收尾）：本小类判定为可进入 SPECS。** 语料 24/24 编译通过 + 2 个定向单轴 body fork 后，槽覆盖：
- ✅ **body_profile_family 4 候选全 converged**：modern_wedge(P2) / vintage_pontoon(P1) 两份原生 shape + fastback / teardrop 两个干净单轴 fork（off P2，统一 door=scissor 基线上纯换 body 表，干净隔离车身轴）。
- ✅ **2 个干净单轴结构特征变体**：`v07`(pop_up_cover) + `v09`(engine_louver_cover)，各在 Diablo 基线只加一个翻板。
- ⚖ **门机构 / 灯 / 尾 的其余候选 = 多轴 reskin 提取**（pagani/porsche/ferrari/bugatti/koenigsegg 同时改多项）。scissor + dihedral 的 ⊥车身可分离性已由 reskin **跨车身佐证**（scissor∈{wedge,sharp-wedge,teardrop,fastback}；dihedral∈{fastback,teardrop,rounded}，见 Slot B 修正）→ 这两类接受 reskin 收敛证据。**gullwing 仍单源 vintage-bound、未跨车身佐证**（见 Slot B），spec-review 决定是否补 external-edit 数据点。agent-fork 对改门机构结构性阻塞，干净门数据点须走 external-edit。
- ⚠ **下游 spec author 须注意**：纯换肤变体（6×car001 / 13×car002 共 body 表）不贡献额外 shape；从多轴 reskin 取候选时**只取目标轴、把同时变化的其余特征当噪声过滤**。

**定向单轴 fork 进展（2026-06-20）：**
- ✅ **body_profile_family 已补齐**：`rec_sportscar_fastback_body_v01`(fastback) + `rec_sportscar_teardrop_body_v01`(teardrop) 干净单轴 fork 成功 + `compile --validate` 通过。本槽 4 候选现全部 converged（modern_wedge=P2 / vintage_pontoon=P1 / fastback / teardrop），且 fastback/teardrop 是在统一 door=scissor 基线上纯换 body 表——干净隔离了车身轴。
- ⛔ **门机构 cross-fork 走 `articraft fork` 结构性阻塞两轮失败**（见 Slot B 流程约束）。门 ⊥ 车身可分离性已由现有 reskin 佐证，§8 可不强制这三个显式数据点。如仍要 gullwing-on-wedge / scissor-on-vintage / conventional 三个干净单轴门变体，**须改走 `articraft external fork` → 直接编辑 model.py（改 DOOR_AXIS+铰座+门皮+重写 run_tests+改 articulation 名）→ `external check` → `external finalize`**，非 agent fork。

**兼容性提示（供下游 spec compatibility matrix）：**
- 共享骨架（轮/转向/轴）与全部槽自由组合，骨架不随槽变。
- pop_up_cover（前）与 engine_louver_cover（后）可共存（各自独立翻板，互不干涉）。
- gullwing 门在低矮 wedge 车身上需校验顶铰高度（vintage roof 高 z≈1.17，wedge 一体顶低）——下游做开门 swept-volume / 顶部铰位 clearance 校验。
- livery_palette 与所有结构槽完全正交（纯材质），不参与 clearance。
