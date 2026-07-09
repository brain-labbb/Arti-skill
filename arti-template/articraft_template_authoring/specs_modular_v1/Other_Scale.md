# scale (decorative two-pan balance scale / scales-of-justice) — Modular Spec

> 来源小类：`picture/Other/Scale`（articraft_data 上游小类样本池）。
> 上游 source map：`picture_expansion/template_source_maps/Other__Scale.md`。
> **"Scale" 在此 = 装饰性双盘天平（decorative two-pan balance scale, scales-of-justice style），不是体重秤 / 厨房电子秤 / 弹簧秤 / 鱼鳞片 / 比例尺。**
> 结构家族 = 立柱天平：静止 `pedestal`（root，落地立柱 + 顶部 clevis 枢叉）+ 横梁 `beam`（绕 +Y REVOLUTE 在柱顶称重倾摆）+ 左右两只秤盘吊挂 `pan_0`/`pan_1`（各绕 +Y REVOLUTE 在梁端独立摆，使盘保持水平）。固定 3 关节（1 梁 + 2 盘），**pan_count 恒为 2**（天平定义，不可作 multiplicity 轴）。
>
> **同步状态**：本 spec 引用的 8 个 5 星样本（1 个 parent + 5 个 fork 槽位变体 + 2 个 chain_count 多重性变体）已同步进本仓库 `data/records/`，rating=5。行号按各样本本仓库 `revisions/rev_000001/model.py` 实际行号计（已逐一 AST + 人工核对）。引用以 helper / part / joint **名字** 为准（`_build_plinth`/`_build_column`/`_build_pivot_head`/`_build_hanger_set`/`_build_dish`/`pedestal`/`beam`/`pan_{idx}`/`pedestal_to_beam`/`beam_to_pan_{idx}`/`_rod_solid`/`ATTACH_ANGLES`），行号仅作定位。

## 元信息
| 项 | 值 |
|---|---|
| slug | `scale` |
| template path | `agent/templates/Other_Scale.py` |
| test path (optional) | `tests/agent/test_scale_template.py`（不写，sweep 为唯一验收）|
| stage | `SPEC_ONLY_DRAFT` |
| status | `pending` |
| __modular__ | `True` |
| pattern | `mixed`（固定 named slots: pedestal_column + pan_hanger + beam_form 挂到共同 `pedestal`/`beam` parent，**外加** `chain_count` 每盘吊链多重性轴）|

## 5 星样本阅读摘要
| 项 | 值 |
|---|---|
| five_star_total | 8（1 parent + 5 fork 槽位变体 + 2 chain_count 多重性变体；均 converged，compile success、≥3 非 fixed joint（1 beam REVOLUTE + 2 pan REVOLUTE）、workbench-only）|
| read_count | 8（**全部读完整 `model.py`**，不抽样；含每个样本的 build helpers、part 树、articulation 与 run_tests）|
| read_scope | all 5-star samples in this category |
| source_index_policy | 仅被采纳为 module source 的样本进入下方 source 表与 §14；本批 8/8 全部被采纳为真实 scale，无未采用样本、无 contamination |

阅读要点（用于槽位分解）：
- **核心机构在全样本恒定**：`pedestal`（root，静止）── `pedestal_to_beam` **REVOLUTE axis=(0,1,0)** origin=(0,0,PIVOT_Z=0.357)，lower=-15°/upper=+15° ──> `beam`；`beam` ── `beam_to_pan_{idx}` **REVOLUTE axis=(0,1,0)** origin=(±0.150,0,HANG_Z=-0.0215)，lower=-15°/upper=+15° ──> `pan_{idx}`（idx∈{0,1}）。3 个 joint、part 树 `pedestal→beam→{pan_0,pan_1}` 在**所有 8 个样本中完全一致**。变化只在三个 named slot 的 mesh / part 内容 + 每盘吊链复制数。
- **pedestal_column 轴**（Slot A）：`pedestal` part 的 visual 组成换形。parent = 方阶基 plinth + flute_ring + 车工 baluster column + pivot_head（4 visual）；tripod = turned hub + 3 条 splayed leg（`for i in range(LEG_COUNT)` 复制，落地于 ball feet）+ pivot_head；round_drum = lathe 圆鼓 drum + flute_ring + column + pivot_head。三者**都把 `pivot_head` clevis 枢叉送到同一 PIVOT_Z 柱顶接口**，beam/pan/joint 拓扑不变 → pedestal_column 只换 `pedestal` 内 visual 组合（tripod 多一个 leg 复制循环，但 leg 是 pedestal-local non-moving visual，不增 part/joint）。
- **pan_hanger 轴**（Slot B）：`pan_{idx}` part 内的吊挂 + 盛器换形。three_chain_dished（parent）= 三链 hanger_set（top ring + hub + 3 converging rod + rim ball）+ 浅碟 dished pan；flat_plate_single_rod = 单中央 rod hanger + 平盘 disc + rim；deep_bucket_scoop = 三链 hanger_set + lathe 深桶 hollow bucket。三者**都通过同一 hanger top ring 套在 beam 端 hang_ball 上**（hook-on-ring captured joint），`pan_{idx}` part 数 / `beam_to_pan_{idx}` REVOLUTE 拓扑不变 → pan_hanger 换 `pan` 内 visual（hanger 形态 + 盛器形态）。
- **beam_form 轴**（Slot C）：`beam` part 的横梁 visual 换形。straight_rect（parent）= `Box` 直矩形 bar + pivot_hub + pivot_axle + 端 knob/pin/ball；ornate_scroll_beam = 两条 swept S-curve `scroll_arm_{idx}`（`sweep_profile_along_spline`）+ tube `scroll_volute_{idx}` + center_rosette + 同样的 pivot_hub/axle + 端 hang 硬件。两者**都是单 `beam` part、同一 pedestal_to_beam REVOLUTE pivot + 同一端 hang_ball 接口** → beam_form 只换 `beam` bar 的 mesh，不改 part/joint。
- **chain_count 轴**（Slot D 多重性）：每盘吊链（rod chain）数。chain_count=2（`ATTACH_ANGLES=(90,270)`，`for chain_idx, ang_deg in enumerate(ATTACH_ANGLES)` 把每链发为独立 visual `chain_{chain_idx}`）/ parent=3（`ATTACH_ANGLES=(90,210,330)`，三链 merge 进 `_build_hanger_set`）/ chain_count=4（`ATTACH_ANGLES=(0,90,180,270)`，四链 merge）→ 同构吊链沿 rim 等角 N 次复制。吊链是**非移动 visual**（Rule 1，随 pan 体动，无独立 joint）。pan_count 恒 2 不作轴。

## 核心身份

一只装饰性**双盘天平**（decorative two-pan balance scale，scales-of-justice 风格，常见 matte black cast iron）：一根静止落地立柱（方阶基 + 车工柱 / 三脚撑 / 圆鼓基，柱顶一只 clevis 枢叉 + dome finial），叉中横穿一根**横梁**（直矩形或涡卷曲梁），梁绕 +Y 轴在柱顶 **REVOLUTE** 称重倾摆（±15°）；梁两端各挂一只**秤盘吊挂**（顶 ring 套在梁端 hang_ball 上，2-4 根细 rod 吊链收束到盛器，盛器为浅碟 / 平盘 / 深桶），每盘各绕 +Y **REVOLUTE** 独立摆动（±15°，使盘随梁倾时仍保持水平）。默认 +Z 朝上、ground z=0、beam 沿 X、所有 pivot axis 沿 +Y。活动语义 = **梁的称重倾摆**（1 REVOLUTE）+ **两盘的独立水平摆**（2 REVOLUTE），固定 3 关节。默认成熟域：pedestal_column × pan_hanger × beam_form × 每盘吊链数 N∈[2,5] 的台式装饰天平。

不该混入：
- **体重秤 / 厨房电子秤 / 浴室秤（bathroom/kitchen platform scale）**——单一踏板 + 数字 / 指针表盘，无横梁双盘机构，是完全不同的结构家族。
- **弹簧秤 / 挂钩秤 / 鱼鳞秤（spring / hanging hook scale）**——单挂钩 + 弹簧位移，无 beam pivot、无双盘。
- **比例尺 / 标尺 / 渐变色阶（measuring scale / ruler / gradient）**——纯刻度无机构，名字同语义全异。
- **单盘 / 三盘 / N≠2 盘**——出"天平"类目；pan_count 恒为 2 是本类身份定义（见 §排除项）。

## 槽位 + 候选模块表

> **建模注记**：三个 named slot 都是**改 visual 组合 / mesh 形态、不增减 part 或 joint** 的轴——pedestal_column 换 `pedestal` 内立柱 visual（tripod 多一个 pedestal-local leg 复制循环）、pan_hanger 换 `pan` 内吊挂+盛器 visual、beam_form 换 `beam` bar mesh。三者笛卡尔积 × chain_count 共同撑开多样性（见 §9）。固定核心机构（pedestal REVOLUTE beam + 2 pan REVOLUTE）在全样本恒定，是类别身份不变量。

### Slot A：pedestal_column（底座 + 立柱支撑形式 —— `pedestal` part 内 visual 组合）

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| turned_baluster_square_plinth（基线） | rec_model-a-...-58c45b51（parent）| `_build_plinth` L79-95 / `_build_column` L98-129 / `_build_flute_ring` L132-140 / `_build_pivot_head` L143-162 / pedestal 装配 L205-225 | eligible if compatible | 方阶基 plinth（box step×2 + square→round loft cap）+ flute_ring（14 斜肋）+ 车工 baluster column（revolve）+ clevis pivot_head；圆对称柱身，方足迹基线 |
| tripod_legs | rec_variant-pedestal-column-tripod-legs-...-5bfc3418 | `_build_hub` L85-98 / `_build_one_leg` L101-123 / `_build_positioned_leg` L126-137 / `_build_pivot_head` L140-159 / pedestal 装配（含 `for i in range(LEG_COUNT)` L208-213）| eligible if compatible | turned hub + **3 条 splayed leg**（120° 等角，各带 ball foot 落地）+ pivot_head；leg 是 pedestal-local non-moving visual 复制（LEG_COUNT=3 固定），落地点改为三脚 |
| round_drum_base | rec_variant-pedestal-column-round-drum-base-...-9e45efb9 | `_build_drum_base` L77-92 / `_build_column` L95-126 / `_build_flute_ring` L129-137 / `_build_pivot_head` L140-159 / pedestal 装配 L202-222 | eligible if compatible | lathe 圆鼓 drum base（revolve 带 molding 环）+ flute_ring + column + pivot_head；圆鼓落地足迹替方阶基，柱身 / 枢叉同 parent |

### Slot B：pan_hanger（秤盘吊挂 + 盛器形式 —— `pan_{idx}` part 内 visual）

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 关键结构特征 |
|---|---|---|---|---|
| three_chain_dished（基线） | rec_model-a-...-58c45b51（parent）| `_build_hanger_set` L165-178（top ring + hub + 3 rod + ball）/ `_build_dish` L181-192 / pan 装配 L278-289 | eligible if compatible | 三链 hanger_set（torus top ring 套 hang_ball + hub + 3 converging rod + rim ball）+ 浅碟 dished pan（spherical-cap shell，0.11m 径）；hanger merge 成单 `hanger_rods` visual |
| flat_plate_single_rod | rec_variant-pan-hanger-flat-plate-single-rod-...-54123985 | `_build_single_rod` L167-178（ring + hub + 单中央 rod）/ `_build_flat_plate` L181-201 / pan 装配 L288-298 | eligible if compatible | **单中央 rod** hanger（torus ring 套 hang_ball + hub + 1 竖直 rod，无 converging 链）+ 平盘 disc（flat cylinder + 抬起 rim ring）；hanger 退化为单杆，盛器为平盘 |
| deep_bucket_scoop | rec_variant-pan-hanger-deep-bucket-scoop-...-2f16a2b1 | `_build_hanger_set` L171-184（同 parent 三链）/ `_build_bucket` L187-221（lathe hollow，BUCKET_* 常量 L61-68）/ pan 装配 L308-318 | eligible if compatible | 三链 hanger_set + **lathe 深桶 hollow bucket**（revolve outer cut inner，tapered wall + flat bottom + 外翻 rim lip）替浅碟；盛器从碟变深斗 |

### Slot C：beam_form（横梁形态 —— `beam` part bar mesh）

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 关键结构特征 |
|---|---|---|---|---|
| straight_rect（基线） | rec_model-a-...-58c45b51（parent）| beam 装配 L228-265（`Box` bar L229-234 + pivot_hub/pivot_axle Cylinder + 端 end_knob/hang_pin/hang_ball `for idx, sx` L247-265）| eligible if compatible | 直矩形 `Box` bar（0.30m 长）+ 中央 pivot_hub + 横 pivot_axle + 两端 knob/drop pin/hang_ball；单 `beam` part，primitive 直梁 |
| ornate_scroll_beam | rec_variant-beam-form-ornate-scroll-beam-...-c2faa4ca | `_scroll_arm_path` L202-217 / `_build_scroll_arm_mesh` L220-229（`sweep_profile_along_spline`）/ `_scroll_volute_path` L232-248 / `_build_scroll_volute_mesh` L251-260（`tube_from_spline_points`）/ `_build_center_rosette_mesh` L263-272 / beam 装配 L308-356 | eligible if compatible | **两条 swept S-curve `scroll_arm_{idx}`**（rounded-rect profile 沿样条扫掠）+ tube `scroll_volute_{idx}` 涡卷 + center_rosette torus + 同 pivot_hub/axle + 端 hang 硬件；单 `beam` part，涡卷曲梁 mesh 替直 Box |

## 槽位图（slot graph）

pattern: mixed（固定 named slots: pedestal_column 决定 `pedestal`（root，坐地）；beam_form 决定 `beam`（pedestal 的 REVOLUTE child）；pan_hanger 决定两只 `pan_{idx}`（beam 的 REVOLUTE children）；外加 `chain_count` 在每只 `pan` 内沿 rim N 次复制吊链 visual）

```
pedestal (root, 坐地; 由 pedestal_column 决定 plinth/hub+legs/drum + flute/column + pivot_head clevis)
  │
  └──[pedestal_to_beam: REVOLUTE axis=(0,1,0), origin=(0,0,PIVOT_Z=0.357), lower=-15° upper=+15°]
        │
        beam (由 beam_form 决定 straight_rect Box / ornate_scroll swept arms; 单 part; 端 hang_ball 接口固定在 ±TIP_X=±0.150)
          │
          ├──[beam_to_pan_0: REVOLUTE axis=(0,1,0), origin=(+0.150,0,HANG_Z=-0.0215), lower=-15° upper=+15°]
          │     └─ pan_0 (由 pan_hanger 决定 hanger 形态 + 盛器; 顶 ring 套 +X 端 hang_ball)
          │           └─ [chain_count multiplicity 轴]  chain_{j} / 等角 rod，j∈range(N)，沿 ATTACH_ANGLES 等角分布
          │
          └──[beam_to_pan_1: REVOLUTE axis=(0,1,0), origin=(-0.150,0,HANG_Z=-0.0215), lower=-15° upper=+15°]
                └─ pan_1 (镜像; 顶 ring 套 -X 端 hang_ball)
                      └─ [chain_count multiplicity 轴]  同 pan_0，N 一致
```

接口点位与 joint 语义：
- **pedestal_column 接口（互斥三选一）**：三候选都把 `pivot_head` clevis 枢叉送到柱顶 PIVOT_Z（origin Z=0.357 固定）。pivot_head（plate_a/plate_b fork + bridge + neck + dome）在三者中**完全相同**（同一 `_build_pivot_head`），是 pedestal→beam 接口的不变硬件。pedestal 坐地（zmin≈0），无父。
- **beam_form 接口（互斥二选一）**：两候选都是单 `beam` part，绕 `pedestal_to_beam` REVOLUTE +Y 倾摆；都带 `pivot_axle`（横穿 clevis fork，captured）+ 中央 `pivot_hub` + 两端 `hang_ball`（在 ±TIP_X=±0.150, z=HANG_Z=-0.0215）。端 hang_ball 是 beam→pan 接口的不变锚点。
- **pan_hanger 接口（互斥三选一）**：三候选的 `pan_{idx}` 都绕 `beam_to_pan_{idx}` REVOLUTE +Y（origin 在对应端 hang_ball），顶 ring（torus）套在 beam 端 hang_ball + drop pin 上（hook-on-ring captured-pin）。盛器（dished / flat plate / bucket）悬于 hanger 下端，盘心在 pan 局部原点下方约 0.18-0.19m。
- **chain_count 接口**：吊链（converging rod + rim ball）为**非移动 visual**（Rule 1，随 pan 体动，无独立 joint），沿 rim 按 `ATTACH_ANGLES` 等角分布、收束到 pan 顶 hub。chain_count=2 把每链发为独立 visual `chain_{j}`（`for chain_idx, ang_deg in enumerate(ATTACH_ANGLES)`，c2 L287-291）；parent(3) / chain_count=4 把全链 merge 进 `_build_hanger_set` 单 mesh。模板统一采**独立 visual 风格**（每链 `chain_{j}`）以保 N 可变（见 §8）。flat_plate_single_rod 是 N=1 退化（单中央 rod，不沿 ATTACH_ANGLES 分布）。
- **mating policy**：beam pivot_axle↔clevis fork = pin-in-fork captured（`allow_overlap(beam.pivot_axle, pedestal.pivot_head)`）；pan 顶 ring↔beam 端 hang_ball/drop pin = hook-on-ring captured（`allow_overlap(pan.hanger_*, beam.hang_ball_{idx}/hang_pin_{idx})`）。几何为套环 / 穿轴非两轴对齐面对接 → **省略 MatingContract（grandfather）**，由 `fail_if_articulation_origin_far_from_geometry` 守 origin + element-scoped `allow_overlap` 守 captured overlap（照搬各样本 run_tests 的 allow_overlap 段，parent L321-342）。
- **rest pose**：所有 3 个 REVOLUTE q=0（梁水平、两盘水平垂吊），lower=-TILT / upper=+TILT（TILT=15°）。
- **互斥 / 可选 / 派生**：pedestal_column 三候选互斥；beam_form 二候选互斥；pan_hanger 三候选互斥；flat_plate_single_rod 隐含 chain_count=1（单杆），故 §9 gate（flat_plate 强制 N=1）。pan_count 恒 2，非轴。

## 每槽位 Module Emits / Interfaces

### Slot A / pedestal_column — turned_baluster_square_plinth（基线）
| emits | 描述 | 来源 |
|---|---|---|
| parts | `pedestal`（root；visual: `plinth` + `flute_ring` + `column` + `pivot_head`）| parent L205-225 / `_build_plinth` L79-95 / `_build_column` L98-129 |
| internal joints | 无（pedestal 是 root，内部无活动件）| — |
| upstream interface | root（坐地 zmin≈0，无父）| parent run_tests L344-350 |
| downstream interface | 柱顶 `pivot_head` clevis fork（供 beam 的 pivot_axle 穿入），PIVOT_Z=0.357 | `_build_pivot_head` L143-162 |

### Slot A / pedestal_column — tripod_legs
| emits | 描述 | 来源 |
|---|---|---|
| parts | `pedestal`（visual: `hub` + `leg_{i}`×3（`for i in range(LEG_COUNT)`）+ `pivot_head`）| tripod L202-218 / `_build_positioned_leg` L126-137 |
| internal joints | 无（3 leg 是 pedestal-local non-moving visual，非独立 part/joint，Rule 1）| — |
| upstream interface | root（三脚 ball feet 坐地 zmin≈ball_r）| tripod run_tests L340-343 |
| downstream interface | 同 turned_baluster：柱顶 `pivot_head` clevis（同一 helper）| `_build_pivot_head` L140-159 |

### Slot A / pedestal_column — round_drum_base
| emits | 描述 | 来源 |
|---|---|---|
| parts | `pedestal`（visual: `drum` + `flute_ring` + `column` + `pivot_head`）| drum L202-222 / `_build_drum_base` L77-92 |
| internal joints | 无 | — |
| upstream interface | root（圆鼓坐地 zmin≈0）| drum run_tests |
| downstream interface | 同上：柱顶 `pivot_head` clevis | `_build_pivot_head` L140-159 |

### Slot C / beam_form — straight_rect（基线）
| emits | 描述 | 来源 |
|---|---|---|
| parts | `beam`（visual: `bar` Box + `pivot_hub` + `pivot_axle` + 端 `end_knob_{idx}`/`hang_pin_{idx}`/`hang_ball_{idx}`）| parent L228-265 |
| internal joints | `pedestal_to_beam` REVOLUTE axis=(0,1,0)，origin=(0,0,0.357)，lower=-TILT/upper=+TILT | parent L267-275 |
| upstream interface | `pivot_axle` 横穿 pedestal `pivot_head` clevis fork（captured）| parent L321-327 |
| downstream interface | 两端 `hang_ball_{idx}`（±0.150, 0, -0.0215）供 pan 顶 ring 套入 | parent L260-265 |

### Slot C / beam_form — ornate_scroll_beam
| emits | 描述 | 来源 |
|---|---|---|
| parts | `beam`（visual: `scroll_arm_{idx}`×2 swept + `scroll_volute_{idx}`×2 + `center_rosette` + `pivot_hub` + `pivot_axle` + 端 `end_knob/hang_pin/hang_ball_{idx}`）| scroll L308-356 |
| internal joints | `pedestal_to_beam` REVOLUTE axis=(0,1,0)，origin=(0,0,0.357)，lower=-TILT/upper=+TILT | scroll L356-... |
| upstream interface | `pivot_axle` 横穿 clevis（同 straight）| scroll allow_overlap 段 |
| downstream interface | 两端 `hang_ball_{idx}`（同 ±0.150 锚点）| scroll L350-352 |

### Slot B / pan_hanger — three_chain_dished（基线）
| emits | 描述 | 来源 |
|---|---|---|
| parts | `pan_{idx}`（visual: `hanger_rods`（top ring + hub + 3 rod + ball）+ `dish`）| parent L278-289 / `_build_hanger_set` L165-178 / `_build_dish` L181-192 |
| internal joints | `beam_to_pan_{idx}` REVOLUTE axis=(0,1,0)，origin=(±0.150,0,-0.0215)，lower=-TILT/upper=+TILT | parent L290-298 |
| upstream interface | 顶 ring 套 beam `hang_ball_{idx}`/`hang_pin_{idx}`（hook-on-ring captured）| parent L328-342 |

### Slot B / pan_hanger — flat_plate_single_rod
| emits | 描述 | 来源 |
|---|---|---|
| parts | `pan_{idx}`（visual: `hanger_rod`（ring + hub + 单中央 rod）+ `plate`（flat disc + rim ring））| flat L288-298 / `_build_single_rod` L167-178 / `_build_flat_plate` L181-201 |
| internal joints | `beam_to_pan_{idx}` REVOLUTE axis=(0,1,0)，origin=(±0.150,0,-0.0215) | flat L300-... |
| upstream interface | 顶 ring 套 hang_ball/drop pin（captured）；隐含 chain_count=1（单杆）| flat L330-345 |

### Slot B / pan_hanger — deep_bucket_scoop
| emits | 描述 | 来源 |
|---|---|---|
| parts | `pan_{idx}`（visual: `hanger_rods`（三链，同 parent）+ `bucket`（lathe hollow 深桶））| bucket L308-318 / `_build_hanger_set` L171-184 / `_build_bucket` L187-221 |
| internal joints | `beam_to_pan_{idx}` REVOLUTE axis=(0,1,0)，origin=(±0.150,0,-0.0215) | bucket L320-... |
| upstream interface | 顶 ring 套 hang_ball/drop pin（captured）| bucket L346-... |

### chain_count multiplicity（吊链复制；non-moving visual）
| emits | 描述 | 来源 |
|---|---|---|
| parts | 无独立 part；`chain_{j}`（converging rod + rim ball）作 `pan_{idx}` 的 visual | chain2 `_build_single_chain` L172-178 + pan loop L287-291 |
| joints | 无（Rule 1，吊链随 pan 体动）| — |
| placement | `for j in range(N)`，沿 rim 按 `ATTACH_ANGLES`（N 等角）分布，收束到 pan 顶 hub | chain2 L287-291（N=2）/ chain4 `_build_hanger_set` L165-178（N=4 merge）/ parent N=3 |

## 参数范围汇总
| 参数 | 类型 | 取值范围 / 候选值 | 标称默认 | 约束类型 | 约束 / 函数 | 来源 |
|---|---|---|---|---|---|---|
| pedestal_column | enum | turned_baluster_square_plinth / tripod_legs / round_drum_base | turned_baluster_square_plinth | choice | deterministic procedural sampler 选；决定 `pedestal` visual + 落地足迹 | module table |
| pan_hanger | enum | three_chain_dished / flat_plate_single_rod / deep_bucket_scoop | three_chain_dished | choice | sampler 选；决定 `pan` 吊挂 + 盛器（互斥）| module table |
| beam_form | enum | straight_rect / ornate_scroll_beam | straight_rect | choice | sampler 选；决定 `beam` bar mesh（互斥）| module table |
| chain_count (N) | int | 声明域 [2,5]；sweep 采样域 [2,5]（偏小加权：2/3 高频、4 常见、5 长尾）；flat_plate_single_rod 强制 N=1 | 3 | conditional→slot_choice | 编入 slot_choice 为 `n{N}`（拓扑维度）；N 与 pan_hanger 联动（flat_plate→1，见下不等式 + §8）| chain2 / parent / chain4 |
| palette_style | enum | matte_black_cast_iron / antique_brass / polished_chrome_silver / aged_bronze_verdigris / weathered_pewter | matte_black_cast_iron | palette | palette only，**不计入 slot_choice**；按 seed 采一种 colorway 应用到 base/beam/chain/pan 四材质 | 各样本材质（parent L199-202）|
| column_height_scale | float | [0.90, 1.12] | 1.0 | independent | 缩放立柱高 → PIVOT_Z 与 pivot_head Z 同步派生（保枢叉落柱顶），clamp | resolve clamp |
| base_footprint_scale | float | [0.90, 1.15] | 1.0 | independent | 缩放底座足迹（plinth W / drum R / 三脚 spread），保稳定支撑半径，clamp | resolve clamp |
| beam_len_scale | float | [0.88, 1.12] | 1.0 | independent | 缩放 BEAM_LEN → TIP_X = beam_len/2 同步（端 hang_ball/pan origin 派生），clamp | resolve clamp |
| pan_size_scale | float | [0.88, 1.12] | 1.0 | independent | 缩放盛器径（DISH_R / PLATE_R / BUCKET_TOP_R），clamp | resolve clamp |
| tilt_angle_scale | float | [0.80, 1.20] | 1.0 | independent | 缩放 3 个 REVOLUTE 的 ±TILT（保 ≤25°，不让盘撞柱），clamp | resolve clamp |
| (—) | derived | — | — | equation | `PIVOT_Z = COLUMN_TOP·column_height_scale + clevis_offset`；`pivot_head Z = f(column_height_scale)`；`TIP_X = BEAM_LEN·beam_len_scale / 2`；pan origin x=±TIP_X、z=HANG_Z 随之派生 | 接口（pivot/hang_ball 锚） |
| (—) | constraint | — | — | inequality | 两盘不互撞 / 不撞柱：`pan_radius·pan_size_scale + clearance ≤ TIP_X − pedestal_footprint/2`；违反时回缩 pan_size_scale 或拒绝重采 | 接口 / clearance |
| (—) | constraint | — | — | inequality | 倾摆时盘不触地 / 不撞底座：满倾（pivot=+TILT, swing=∓TILT）下 pan zmin > base_top；违反时缩 tilt_angle_scale | 接口 / clearance（parent run_tests L443-450）|
| (—) | constraint | — | — | conditional | pan_hanger=flat_plate_single_rod ⇒ chain_count 锁 N=1（单中央杆无 converging 链）；其余 pan_hanger ⇒ N∈[2,5] | 接口 / §8 |

所有连续 scale 在 `resolve_config` 中 clamp / 派生；每个 build 解析一次。scale 只动安全比例 / clearance / 行程 / 角度，**绝不改变 pedestal_column / pan_hanger / beam_form / N 的拓扑、也绝不改 pan_count（恒 2）或 3 个固定 REVOLUTE**。

## Multiplicity / Copy Logic

**1 根 multiplicity 轴**（每盘吊链数；pan_count 恒 2，**不是** multiplicity 轴）：

- **count_param**：`chain_count`（模板内变量 N；每只 `pan` 的吊链 / converging rod 数；两盘 N 一致）。
- **N_range**：声明产品域 **[2, 5]**（装饰天平每盘吊链现实上 2-4 链最常见，取上界 5 略放宽；source map 建议 [2,6]，本 spec 收窄为 **[2,5]** 因为只有 N=2/3/4 有直接样本，N=5 由 ATTACH_ANGLES 等角公式安全外推、N=6 下采样稀有故取上界 5 保守 → 见 §9 sweep plan）。`config_from_seed` 的 sweep 采样域 **[2, 5]**（偏小加权：N=2/3 高频、N=4 常见、N=5 稀疏）。**特例**：pan_hanger=flat_plate_single_rod 时 N 锁 1（单中央杆，无 converging 链，见 §8 gate）。
- **sampling domain**：`config_from_seed` 先采 pan_hanger；若 flat_plate_single_rod → N=1；否则 `rng.choices((2,3,4,5), weights=偏小)`。`resolve_config` 把任意外部 config 的 N（非 flat_plate）clamp 到 [2,5]。
- **copied object**：单根吊链单元——`chain_{j}`（一条 converging rod `_rod_solid` + rim ball），共享 helper 发射（`_build_single_chain(ang_deg)`，chain2 L172-178 已是此结构，可直接作 copy-logic 源）；N 条复用同一 helper。
- **naming**：`chain_{j}`（pan-local visual 名）/ mesh id `pan_chain_{j}_{idx}`（含 pan idx 去重），`for j in range(N)`（chain2 L287-291 已用此结构）。
- **placement**：沿 rim **等角**分布——`ATTACH_ANGLES = [base_offset + j·360/N for j in range(N)]`（绝对式，每 j 的角由 N 解析，不累加漂移），收束到 pan 顶 hub（hub 与 top ring 与 N 无关，固定）。绝对式（角由 N 与 base_offset 解析）是 N-不变前提。
- **joint policy**：吊链是**非移动件**（Rule 1）→ inline 为 `pan` 的 visual，**不发射独立 joint**；活动关节恒为 1 beam REVOLUTE + 2 pan REVOLUTE。两盘 N 一致（对称天平）。
- **source/gating**：copy-logic 源取 chain2 L172-178 + L287-291（`_build_single_chain` + `for j in enumerate(ATTACH_ANGLES)` 独立 visual 风格，N 可变）；N=3 取 parent 的 merged `_build_hanger_set`（等价 3 链）、N=4 取 chain4 `_build_hanger_set`（等价 4 链）作几何对照。模板统一采**每链独立 visual** 风格以保 N 任意可变。N≥2 与 pan_hanger 的兼容见 §9 矩阵（flat_plate 锁 N=1）。

## 拓扑多样性审计

总组合数：pedestal_column(3) × pan_hanger(3) × beam_form(2) × chain_count 采样数 = **基数 3×3×2 = 18**，乘 chain_count：
- pan_hanger ∈ {three_chain_dished, deep_bucket_scoop}（2 种）× N∈{2,3,4,5}（4 档）= 8；
- pan_hanger = flat_plate_single_rod（N 锁 1，1 档）= 1；
即每个 (pedestal_column × beam_form)=6 组合下 pan_hanger×N 合法子组合 = 8+1 = **9**；
**总合法组合数 = 6 × 9 = 54**（与 source map 预审 54 一致）。

理由：pedestal_column(3) × beam_form(2) 即 6 个底座/梁拓扑外观类；叠 pan_hanger(3，含单杆/三链/深桶不同 visual 组合)即 18 ≫ 10 已稳过；再叠 chain_count（N 编入 slot_choice）后总 54 distinct。**N 必须编入 `slot_choices_for_seed` 的 tuple**（`("chain_count", f"n{N}")`，对齐 cushion/shopping_bucket/fence_cascade），否则不同吊链数在 slot_choice 上无法区分，损失一整根拓扑维度。注意核心 3-REVOLUTE 机构在全样本恒定 → 拓扑差异来自 named slot 的 visual 组合 + N，**不是** joint 拓扑差异（本类身份就是固定双盘机构）。

seed_domain_policy：procedural_first（`config_from_seed(seed)` 对所有 seed 用 `random.Random(seed)` 采样；seed=0 不特殊）。

Procedural Sampling / Sweep Plan：sampler 先 `rng.choice` 三个 named slot（pedestal_column / pan_hanger / beam_form），经兼容矩阵合法化，再按 pan_hanger 解析 chain_count（flat_plate→1，否则 `rng.choices` 加权 N∈[2,5]），再 uniform 各连续 scale（column_height/base_footprint/beam_len/pan_size/tilt_angle）。compatibility matrix 排除 / 降级非法组合（见下表）。无 regression overrides（首版纯 procedural）。random sweep seeds 0-49 初轮 + 0-999 成熟审计；viewer 目检 seeds 0-9。


Controlled local parameterization：见 §参数表的 column_height_scale / base_footprint_scale / beam_len_scale / pan_size_scale / tilt_angle_scale（全 independent，部分 equation 派生 PIVOT_Z/TIP_X，两条 clearance inequality）。采样契约：先采 named slot → 按 pan_hanger 解析 chain_count（conditional：flat_plate→1）→ 采 independent scale → 派生（PIVOT_Z/pivot_head Z 随 column_height、TIP_X 随 beam_len、pan origin 随 TIP_X）→ 用两条 clearance inequality（两盘不互撞/撞柱、满倾不触地）投影 / 回缩。跨部件依赖（PIVOT_Z↔column height、TIP_X↔beam len、pan origin↔TIP_X、盘径↔beam 半跨）显式落在 §7 equation/inequality，在 `resolve_config` 内求解。这些 scale 不破坏 pivot/hang_ball 接口、captured-pin/hook-on-ring、N 复制逻辑或类别身份（固定 3-REVOLUTE 双盘）。

| item | policy | validator / viewer focus |
|---|---|---|
| sampler | 先 `rng.choice` 三 named slot（经兼容矩阵），再按 pan_hanger 解析 chain_count（flat_plate→1，否则 `rng.choices` 加权 N∈[2,5]），再 uniform 各 scale | slot_choices_for_seed 含 `("chain_count", f"n{N}")` 且与 build 一致 |
| compatibility matrix | (1) **pan_hanger=flat_plate_single_rod × chain_count**：单中央杆无 converging 链 → N 锁 1（slot_choice 记 `n1`）；three_chain_dished / deep_bucket_scoop → N∈[2,5]。 (2) **pedestal_column 与 beam_form / pan_hanger 正交**：三底座（plinth/tripod/drum）均可配任意梁形 + 任意盘形（pivot_head 接口同一），全允许。 (3) **beam_form × pan_hanger 正交**：直梁 / 涡卷梁端 hang_ball 锚点相同，均可挂任意盘，全允许。 (4) **pan_count 恒 2**：不暴露为参数，sampler 不采。 | 无 floating / collision / 两盘互撞或撞柱 / 满倾触地 / 单杆配多链 / pan_count≠2 |
| controlled local variation | 5 个 clamped scale（column_height、base_footprint、beam_len、pan_size、tilt_angle），每 build 统一；PIVOT_Z/TIP_X/pan origin 为 equation 派生 | 比例变化不破坏 pivot/hang_ball origin、captured 接口、两盘 clearance、坐地、固定 3-REVOLUTE 与类别身份 |
| regression overrides | none（首版纯 procedural）| — |
| random sweep | seeds 0-49 初轮；0-999 成熟审计 | 逐组合 QC（倾摆 / 双盘水平摆语义）|

| slot | candidate_count | 是否 ≥2 | 是否 ≥3 | 备注 |
|---|---:|---|---|---|
| pedestal_column | 3 | yes | yes | turned_baluster（parent）/ tripod / drum（互斥底座）|
| pan_hanger | 3 | yes | yes | 三链碟 / 单杆平盘 / 深桶（互斥盘形）|
| beam_form | 2 | yes | no | 直梁（parent）/ 涡卷梁（互斥）；**降级理由**：source map 仅 ornate_scroll 一个 beam fork（其余样本沿用直梁），样本池只支撑 2 个结构不同的梁形，无第 3 个真实梁拓扑可采 → 保 2，不发明结构（符合 SPEC_TEMPLATE §4 "样本池不足可降到 2，须说明理由"）|
| chain_count (N) | 4（采样域 {2,3,4,5}，2/3 高频 / 5 长尾；flat_plate 特例 n1）| yes | yes | 拓扑维度，编入 slot_choice |

## Validator

- `slot_choices_for_seed` 返回已实现的 module 名，且含 `("chain_count", f"n{N}")`（flat_plate 时 `n1`）
- `config_from_seed` 对所有 seed 用 deterministic procedural sampling；非 flat_plate 时 N 采样域 ⊆ [2,5]，flat_plate 时 N=1
- `resolve_config` 把 chain_count clamp 到 [2,5]（flat_plate 锁 1），各 scale clamp 到声明范围；PIVOT_Z/TIP_X/pan origin 为 equation 派生；两条 clearance inequality 在 resolve 内投影 / 回缩
- compatibility matrix / gating 阻止非法组合（flat_plate 锁 N=1；pan_count 恒 2 不暴露；两盘不互撞/撞柱；满倾不触地）
- 连续 scale clamp 后不破坏 pivot/hang_ball origin / captured-pin/hook-on-ring 接口 / 两盘 clearance / 坐地 / N 复制 / 固定 3-REVOLUTE
- 关键 joint：`pedestal_to_beam` REVOLUTE axis≈(0,1,0)（abs(axis[1])>0.99）origin Z≈PIVOT_Z；`beam_to_pan_0`/`beam_to_pan_1` 2×REVOLUTE axis≈(0,1,0) origin x≈±TIP_X、z≈HANG_Z；三 joint lower=-TILT/upper=+TILT 对称
- 称重语义：pivot +TILT 时一盘降一盘升（parent run_tests L419-429）；单盘 swing +TILT 时盘心 X 偏移（off-axis proof，L431-441）
- captured 接口：element-scoped `allow_overlap`（beam `pivot_axle`↔pedestal `pivot_head`；pan `hanger_*`↔beam `hang_ball_{idx}`/`hang_pin_{idx}`），照搬各样本 run_tests 的 allow_overlap 段（parent L321-342）
- copied object 遵循 `chain_{j}` 命名 + 沿 ATTACH_ANGLES 等角 placement + Rule 1（无独立 joint）；两盘 N 一致
- grandfather：所有 pivot / hook-on-ring captured 接口省略 MatingContract，由 origin 检查 + allow_overlap 守
- `pan_count` 恒 2（part `pan_0`/`pan_1` 各 1 个 beam_to_pan REVOLUTE），不暴露为参数、不进 multiplicity

## Reject cases

- 把 chain_count 当普通 int 参数、不进 slot_choice → 不同吊链数 slot_choice 同形，损失拓扑维度（违反 §8/§9 硬要求）。
- 把 pan_count 做成 multiplicity 轴（单盘 / 三盘 / N 盘）→ 出"天平"类目；pan_count 恒 2 是身份定义，必须固定。
- flat_plate_single_rod 配 chain_count>1（单中央杆补多条 converging 链）→ 无样本支持、语义矛盾；必须 gate（flat_plate 锁 N=1）。
- 把吊链 / 三脚 leg 当独立活动 part 加 joint → 违反 Rule 1（非移动件，应 inline 为承载 part visual）。
- 3 个 REVOLUTE rest pose 设成倾摆角而非 q=0（梁水平、盘垂吊）→ current-pose 与 viewer 目检不符（所有样本 q=0 水平，lower=-TILT/upper=+TILT 对称）。
- pivot / pan-swing origin 放在腔中心或任意点而非真实柱顶 clevis / 梁端 hang_ball → `fail_if_articulation_origin_far_from_geometry`（0.015）FAIL。
- 给 pivot_axle↔clevis 或 ring↔hang_ball 补 MatingContract 硬对接 → 套环 / 穿轴几何对不上，mating-gap FAIL；应 grandfather + allow_overlap。
- pan_size / beam_len scale 致两盘互撞或撞柱 / 满倾触地 → §7 两条 inequality FAIL；须按比例回缩。
- 把连续尺寸 / 颜色 / 材质（palette_style / *_scale）当新 candidate 塞进 slot → 不是结构差异。
- 把"体重秤 / 弹簧秤 / 比例尺"语义混入（单踏板 / 单挂钩 / 无机构）→ 出类，本类是立柱双盘倾摆天平。

## 与相邻类别的边界

- 不该混入：**体重秤 / 厨房 / 浴室电子秤（platform / digital scale）**——单踏板 + 表盘 / 数显，无横梁双盘倾摆机构，主运动 spine 完全不同。
- 不该混入：**弹簧秤 / 挂钩秤 / 鱼鳞秤（spring / hanging scale）**——单挂钩 + 弹簧线性位移，无 beam pivot、无对称双盘。
- 不该混入：**比例尺 / 标尺 / 渐变色阶 / 鱼鳞片（ruler / measuring scale / gradient / fish scale）**——纯刻度或表面纹理，无任何机构（名字同形语义全异，见顶部注记）。
- 不该混入：**单盘 / 三盘 / N≠2 盘秤**——pan_count 恒 2 是天平身份；N≠2 出类，如需可作单独 slug。

## 审核记录
| 项 | 结论 |
|---|---|
| reviewer status | pending |
| reviewer notes | （待人工审核：确认 (1) beam_form 仅 2 candidate（样本池仅 1 个梁 fork，已说明降级理由，是否接受）；(2) chain_count N_range 取 [2,5]（保守，N=5 由等角公式外推、N=6 无源故收窄；source map 建议 [2,6]）还是按 source map [2,6]；(3) flat_plate_single_rod 锁 N=1 的兼容降级策略；(4) Topology target 54<300 的说明是否接受（本小类核心机构固定，真实结构上限）；(5) 吊链 / 三脚 leg Rule 1 inline 无独立 joint 是否符合 multiplicity 审计期望；(6) palette_style 5 个 colorway（matte_black / antique_brass / polished_chrome / aged_bronze_verdigris / weathered_pewter）是否覆盖现实装饰天平材质）|（统一口径：富类别建议 ≥300；低于 300 记录真实组合空间或兼容约束原因；report-only，不设门。）

## 模板实现备注（可选）

- 共享 helper：`_rod_solid`（所有 rod / chain / flute 通用，全样本一致 L71-76）、`_build_pivot_head`（pedestal_column 三候选共享同一 clevis helper，是 pedestal→beam 接口不变硬件）、`_build_column`/`_build_flute_ring`（baluster/drum 共享柱身 + 斜肋）、`_build_hanger_set`/`_build_single_chain`（pan_hanger 三链 / 复制吊链共享）、`_build_dish`/`_build_flat_plate`/`_build_bucket`（盛器 mesh，按 pan_hanger 切换）、`sweep_profile_along_spline`/`tube_from_spline_points`（仅 ornate_scroll_beam）。
- captured 接口 allow_overlap：`run_scale_tests` 里逐接口补 element-scoped `allow_overlap`（pivot_axle↔pivot_head；pan hanger↔hang_ball/hang_pin），照搬 parent run_tests L321-342（5 个 fork 与 2 个 chain_count 变体 allow_overlap 段同构）。
- conditional 范围解析顺序：先采 pedestal_column / pan_hanger / beam_form → 按 pan_hanger 解析 chain_count（flat_plate→1，否则 N∈[2,5]）→ 采 column_height/base_footprint/beam_len/pan_size/tilt_angle independent scale → 派生 PIVOT_Z（随 column_height）/ TIP_X（随 beam_len）/ pan origin（随 TIP_X）→ 投影两条 clearance inequality（两盘不撞、满倾不触地）。
- chain_count 复制：统一用每链独立 visual 风格（`for j in range(N): pan.visual(_build_single_chain(ATTACH_ANGLES[j]), name=f"chain_{j}")`，照搬 chain2 L287-291）；ATTACH_ANGLES 由 N 等角生成（不硬编码）；N=1（flat_plate）退化为单中央 rod（不进链循环）。
- pan_count 恒 2：`for idx, sx in ((0,1.0),(1,-1.0))` 固定两盘对称，不参数化（全样本 L247/L278 此结构一致）。
- 参考模板：`agent/templates/Accessories_Cushion.py`（同为 mixed pattern：固定 named slots + `("count", f"n{N}")` 进 slot_choice + 绝对式 placement + 共享 mesh helper + 兼容矩阵 gating + captured-pin allow_overlap + grandfather MatingContract 骨架，本类可同构改编；区别：scale 核心 3-REVOLUTE 机构在全样本恒定，named slot 只换 visual 不换 joint 拓扑）。`agent/templates/Bag_Suitcase_Shopping_bucket.py`（绝对式沿轴等距 placement + count 进 slot_choice）。

## Module Source Index

| source_id | slot | module | sample_id | model.py 来源 | 采纳用途 |
|---|---|---|---|---|---|
| S1 | A / B / C（parent 基线）| turned_baluster + three_chain_dished + straight_rect + N=3 | rec_model-a-...-58c45b51 | `_build_plinth` L79-95 / `_build_column` L98-129 / `_build_pivot_head` L143-162 / `_build_hanger_set` L165-178 / `_build_dish` L181-192 / 核心机构装配 L205-298 / allow_overlap L321-342 | 方阶基底座 + 三链碟盘 + 直梁基线 + 核心 3-REVOLUTE 机构 + captured 范式 + N=3 吊链对照 |
| S2 | A | tripod_legs | rec_variant-pedestal-column-tripod-legs-...-5bfc3418 | `_build_hub` L85-98 / `_build_one_leg` L101-123 / `_build_positioned_leg` L126-137 / `for i in range(LEG_COUNT)` L208-213 | 三脚底座 mesh helper（pivot_head 接口不变，beam/pan 树不变）|
| S3 | A | round_drum_base | rec_variant-pedestal-column-round-drum-base-...-9e45efb9 | `_build_drum_base` L77-92 / pedestal 装配 L202-222 | 圆鼓 lathe 底座（drum + flute + column + pivot_head）|
| S4 | B | flat_plate_single_rod | rec_variant-pan-hanger-flat-plate-single-rod-...-54123985 | `_build_single_rod` L167-178 / `_build_flat_plate` L181-201 / pan 装配 L288-298 | 单中央杆 + 平盘（隐含 chain_count=1）|
| S5 | B | deep_bucket_scoop | rec_variant-pan-hanger-deep-bucket-scoop-...-2f16a2b1 | `_build_bucket` L187-221（BUCKET_* L61-68）/ `_build_hanger_set` L171-184 / pan 装配 L308-318 | lathe 深桶盛器（三链 hanger 不变，盘形换深斗）|
| S6 | C | ornate_scroll_beam | rec_variant-beam-form-ornate-scroll-beam-...-c2faa4ca | `_scroll_arm_path` L202-217 / `_build_scroll_arm_mesh` L220-229 / `_scroll_volute_path` L232-248 / `_build_scroll_volute_mesh` L251-260 / `_build_center_rosette_mesh` L263-272 / beam 装配 L308-356 | 涡卷曲梁 swept mesh（单 beam part + pivot/hang 接口不变）|
| S7 | D（multiplicity）| chain_count N=2 | rec_variant-chain-count-2-...-7935158d | `_build_hanger_top` L165-170 / `_build_single_chain` L172-178 / `for chain_idx, ang_deg in enumerate(ATTACH_ANGLES)` L287-291（`ATTACH_ANGLES=(90,270)` L65）| 双链 copy-logic 源（**每链独立 visual** `chain_{j}` 风格，N 可变的权威结构）|
| S8 | D（multiplicity）| chain_count N=4 | rec_variant-chain-count-4-...-c5c8338c | `_build_hanger_set` L165-178（`ATTACH_ANGLES=(0,90,180,270)` L65，四链等角 merge）| 四链 copy-logic 对照（等角分布上界示例）|
