# Pencil sharpener — Modular Spec

> 来源小类：`picture/Stationary/Pencil sharpener`（articraft_data 上游小类样本池；对象身份为 desk hand-crank pencil sharpener，参考图 `picture/Stationary/Pencil sharpener/001.png`，Caran d'Ache 风格台式削笔器）。
> 上游 source map：建议回填 `picture_expansion/template_source_maps/Stationary__Pencil_sharpener.md`（当前尚未建立；本 spec 已逐一内联全部 record_id + module 来源，source map 缺失不影响来源完整性）。
> **同步前置**：本 spec 引用的 `model.py:Lx-Ly` 来自该小类的 workbench-only 样本（1 个 parent + 10 个单轴 fork 变体），目前仍在 `articraft_data` 仓库，**尚未同步进本仓库 `data/records/`，且上游 `rating` 当前为 `null`**。进入 TEMPLATE_AFTER_REVIEW 前需先把这 11 个 record 目录 + 物化缓存同步进本仓库并批量写 `rating=5`（FORK_VARIANTS §7：收敛即入池——11 个样本均 compile rc=0、均含 ≥1 非 fixed joint、均不出类目）。本 spec 行号按各样本 `articraft_data` 当前 `revisions/rev_000001/model.py` 计；同步后按本仓库行号 rebase。引用以 part/joint/helper **名字** 为准（`_build_housing` / `_build_cutter` / `_build_crank` / `_build_drawer` / `_build_bin_lid` / `_build_canister` / `_build_blade` / `_build_crank_grip` / `_build_clamp_frame` / `_build_thumbscrew` / `_cut_port` / `housing_to_crank` / `housing_to_drawer` / `bin_to_lid` / `housing_to_canister` / `arm_to_grip` / `frame_to_thumbscrew` 等），行号仅作定位。

## 元信息
| 项 | 值 |
|---|---|
| slug | `pencil_sharpener` |
| template path | `agent/templates/Stationary_Pencil_sharpener.py` |
| test path (optional) | `tests/agent/test_pencil_sharpener_template.py`（不写，sweep 为唯一验收）|
| stage | `TEMPLATE_BUILT` |
| __modular__ | `True` |
| pattern | `mixed`（固定 root housing + parallel-children 槽位：body_form 形态 + sharpening 机构 + shavings 容器 + crank handle + mount，**外加** sharpening-hole 的 `n_holes` 多重性轴）|

## 5 星样本阅读摘要
| 项 | 值 |
|---|---|
| five_star_total | 11（1 parent + 10 单轴 fork 变体；均 converged，compile rc=0、均有 ≥1 非 fixed joint、workbench-only）|
| read_count | 11（全部样本 `model.py` 全文逐行读，含 build_object_model + run_tests）|
| read_scope | all 5-star samples in this category（小类只有这 11 个，无抽样）|
| source_index_policy | only adopted module sources are indexed below（11 个样本全部提供 module 来源，无未采用样本）|

样本与采纳分工：
- **S0 parent**（`rec_build-a-realistic-articulated-3d-model-of-a-penc_..._38af6c5b`）：charcoal 圆角矩形 housing（`_rounded_box`+shoulder 块），前面 1 个 conical pencil port + 内部 helical cutter（FIXED），右侧 hand crank（CONTINUOUS 绕 +Y），顶部 2 根 clamp post（FIXED）。**全批基线**：提供 housing 壳、cutter、crank 装配、clamp posts、port/throat funnel 与坐标约定（+X 前、+Y 右、+Z 上）。
- **S1 drawer_shavings**（`rec_sharpener_var_drawer_shavings`）：lower-front 增 PRISMATIC 抽屉 `drawer`（沿 +X 滑出，`_build_drawer` L220-273，housing cut drawer cavity + rails L137-167，joint L455-465）。**shavings=sliding_drawer（PRISMATIC）槽位来源**。
- **S2 lid_hinged_bin**（`rec_sharpener_var_lid_hinged_bin`）：lower-rear 拆成 fixed `bin_shell`（FIXED）+ 顶盖 `bin_lid`（REVOLUTE 绕 -Y 沿后边翻开，`_build_bin_shell` L215-272、`_build_bin_lid` L275-311、joint L487-493）。**shavings=hinged_bin（REVOLUTE）槽位来源**。
- **S3 mechanism_wedge**（`rec_sharpener_var_mechanism_wedge`）：删 crank，前 port 改 fixed 单片 `_build_blade`（FIXED 钢片），运动转到 shavings 侧 `shavings_lid`（REVOLUTE 绕 +Y，`_build_shavings_lid` L199-252、joint L328-334）。**sharpening=fixed_wedge_blade 槽位来源 + shavings hinged-lid 互补来源**。
- **S4 canister_twist**（`rec_sharpener_var_canister_twist`）：cutter 下方挂 twist-off `shavings_canister`（CONTINUOUS 绕 +Z 拧下，revolved cup + threaded collar，`_build_canister` L303-379、joint L469-475）。**shavings=twist_canister（CONTINUOUS）槽位来源**。
- **S5 holes_dual**（`rec_sharpener_var_holes_dual`）：前面 2 个不同孔径 port + 2 个 cutter，由 `_cut_port` L81-103 + `_build_cutter(y_offset)` L139-171 + `for i in range(2)` L301-323 发射。**hole-count multiplicity N=2 + 循环范式来源**。
- **S6 holes_triple**（`rec_sharpener_var_holes_triple`）：前面 3 个 graduated port + 3 cutter，同 `_cut_port` + `for` 范式（L296-323）。**hole-count multiplicity N=3 来源**。
- **S7 body_cylindrical**（`rec_sharpener_var_body_cylindrical`）：housing 改 revolved 圆桶 barrel（`_build_barrel` L85-160，`revolve(360)` + dome 顶 + foot ring + band）。**body_form=cylindrical_barrel 槽位来源**。
- **S8 body_teardrop**（`rec_sharpener_var_body_teardrop`）：housing 改 lofted 流线 teardrop 壳（`_build_teardrop_shell` L78-161，后宽前窄）。**body_form=teardrop_shell 槽位来源**。
- **S9 handle_folding**（`rec_sharpener_var_handle_folding`）：crank 拆成 `crank_arm`（CONTINUOUS 绕 +Y）+ `crank_grip`（REVOLUTE 绕 -X 在臂端 knuckle 折叠，`_build_crank_arm` L191-246、`_build_crank_grip` L247-298、arm_to_grip joint L412-418）。**crank_handle=folding（嵌套 REVOLUTE）槽位来源**。
- **S10 clamp_gclamp**（`rec_sharpener_var_clamp_gclamp`）：删 2 clamp post，body 下挂 C 形 `clamp_frame`（FIXED）+ `thumbscrew`（PRISMATIC 绕 +Z 顶住桌面，`_build_clamp_frame` L199-289、`_build_thumbscrew` L290-355、joint L501-507）。**mount=under_body_gclamp（PRISMATIC 螺杆）槽位来源**。

冗余说明：S0/S1/S2/S4/S5/S6/S7/S8/S9/S10 的 crank（或 crank_arm）均为同一 CONTINUOUS 绕 +Y 机构（共享 `_build_crank` 基线）；只有 S3 用 fixed wedge blade 取代 crank。各 fork 各自只改 1 根结构轴，其余层与 parent 同构——干净的单轴控制变量 fork 池，每个轴恰好 1-2 个收敛候选。

## 核心身份

台式手摇削笔器（desk hand-crank pencil sharpener）：一只实心壳体 housing（charcoal 圆角矩形 / 圆桶 / 流线 teardrop，外形 ~0.09×0.08×0.10 m），平躺于 z=0；正面（+X）有 1 个或多个 conical pencil port（漏斗 throat 通向内部 helical cutter），右侧（+Y）伸出 hand crank（手柄绕水平 +Y 轴 **CONTINUOUS 旋转**驱动 cutter），下部/后部有 shavings 容器（屑盒），顶部有 clamp post 或下挂 G-clamp 固定到桌沿。**主用户机构 = 侧面手摇曲柄的旋转**（CONTINUOUS 绕 +Y）；当机构改为 fixed 钢片 wedge 时，主运动转为 shavings 侧的 REVOLUTE 翻盖。

默认成熟域：一只 box/barrel/teardrop housing，crank 或 wedge 削笔机构，1-3 个削笔孔，屑盒为 captured-cavity / sliding-drawer / hinged-bin-lid / twist-canister 之一，曲柄为一体或 folding，固定为顶部 clamp post 或 under-body G-clamp。活动语义恒含 ≥1 非 fixed joint（crank CONTINUOUS / drawer PRISMATIC / lid REVOLUTE / canister CONTINUOUS / thumbscrew PRISMATIC / folding grip REVOLUTE）。cutter / clamp post / front plate / wedge blade / bin shell 等不动件为 fixed 装饰（cutter/blade/bin/post 做 FIXED part 是上游样本既有约定，grandfathered 接受；front plate inline body visual）。

不该混入：电动削笔器 electric sharpener（无手摇曲柄、无桌沿夹，按钮启动马达）、刀片式小削笔器 prism/wedge pocket sharpener（无 housing/曲柄/屑盒机构、纯一块塑料 + 刀片，体量过小且无削笔孔阵列以外机构）、卷笔刀玩具 / 文具收纳座（无削笔 port + cutter 身份）。Stationary 大类内区别于 Calculator / Clip / Stapler 等无削笔孔 + 曲柄文具。

## 槽位 + 候选模块表

> **建模注记（重要）**：pencil_sharpener 是 **root housing + 一组 parallel children**——crank（CONTINUOUS）/ shavings 容器（drawer PRISMATIC / lid REVOLUTE / canister CONTINUOUS / 或 captured 无 part）/ mount（clamp posts FIXED 或 g-clamp PRISMATIC）/ cutter（FIXED）各自独立挂到 housing 的不同真实面，**不串成链**。下面 5 个 slot 是 housing 的并联可替换层；sharpening-hole 的 N 由 §8 多重性轴描述。cutter primitive 随 n_holes 多重性循环复制。

### Slot A：body_form（壳体 footprint 形态）

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| box_charcoal（基线） | rec_..._38af6c5b（S0；多数 fork 同形） | `_build_housing` L74-122 + `_rounded_box` L64-71 | eligible if compatible | 圆角矩形 slab housing（`box`+`fillet("|Z")`）+ shoulder 块；前面平面 port、顶部平 deck |
| cylindrical_barrel | rec_..._body_cylindrical（S7） | `_build_barrel` L85-160（`revolve(360)` L98-107 + dome 顶 + foot ring + band）| eligible if compatible | 旋转圆桶 housing（revolved 圆截面 + chamfer dome 顶 + foot ring + 装饰 band），圆 footprint |
| teardrop_shell | rec_..._body_teardrop（S8） | `_build_teardrop_shell` L78-161（loft 后宽前窄流线壳）| eligible if compatible | lofted 流线 teardrop 壳（后部宽、向前 port 收窄），compound 曲面 footprint |

> 3 candidate（满足 ≥3 目标）：box / barrel / teardrop 三个真实收敛形态，footprint primitive（box vs revolve vs loft）成对结构差异。

### Slot B：sharpening_mechanism（削笔机构）

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| crank_helical（基线） | rec_..._38af6c5b（S0；多数 fork） | `_build_crank` L195-264 + CONTINUOUS `housing_to_crank` L351-359 + `_build_cutter` L125-158（FIXED）| eligible if compatible | 侧面 hand crank（轴+臂+grip），CONTINUOUS 绕 +Y 驱动；内部 helical cutter（FIXED part）|
| fixed_wedge_blade | rec_..._mechanism_wedge（S3） | `_build_blade` L143-164（FIXED 钢片）+ 无 crank；运动转到 shavings lid | eligible if compatible | 无 crank，port 横跨一片 fixed 钢刀片（手转笔），主运动改由 shavings hinged lid 提供 |

> 2 candidate（降级理由）：单 parent 小类，sharpening 机构只有 crank（powered helical）与 fixed wedge（manual）两个真实收敛形态。**互斥门控**：fixed_wedge ⇒ 无 crank part ⇒ 必须强制 shavings=hinged_bin（保证 ≥1 非 fixed joint），见 §9 兼容矩阵。

### Slot C：shavings_container（屑盒/收屑机构）

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| captured_cavity（基线） | rec_..._38af6c5b（S0） | （无独立 shavings part；屑留在 housing 内腔）| eligible if compatible | 屑盒为 housing 内腔（无活动收屑件；baseline）|
| sliding_drawer | rec_..._drawer_shavings（S1） | `_build_drawer` L220-273 + housing drawer cavity/rails L137-167 + PRISMATIC `housing_to_drawer` L455-465 | eligible if compatible | lower-front 开口 + open-top tray，PRISMATIC 沿 +X 滑出（range 0..DRAWER_TRAVEL），关闭面板 flush |
| hinged_bin_lid | rec_..._lid_hinged_bin（S2）/ rec_..._mechanism_wedge（S3 互补） | `_build_bin_shell` L215-272（FIXED）+ `_build_bin_lid` L275-311 + REVOLUTE `bin_to_lid` L487-493 | eligible if compatible | lower-rear fixed bin shell + 顶盖 REVOLUTE 绕 -Y 沿后边翻开（range 0..~1.6），origin 在盖-唇接缝 |
| twist_canister | rec_..._canister_twist（S4） | `_build_canister` L303-379（revolved cup + threaded collar）+ CONTINUOUS `housing_to_canister` L469-475 | eligible if compatible | cutter 下方挂 revolved cup + 螺纹 collar，CONTINUOUS 绕 +Z 拧下，cup 朝 -Z 收屑 |

> 4 candidate（满足 ≥3 目标）：captured（无件）/ drawer（PRISMATIC）/ bin lid（REVOLUTE）/ canister（CONTINUOUS）四个真实拓扑，joint type 各异。

### Slot D：crank_handle（曲柄手柄机构；conditional 仅 crank 存在时）

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| one_piece（基线） | rec_..._38af6c5b（S0） | `_build_crank` L195-264（一体臂+grip）| eligible if compatible | 一体曲柄臂 + grip 旋钮（仅 1 个 CONTINUOUS joint）|
| folding_grip | rec_..._handle_folding（S9） | `_build_crank_arm` L191-246 + `_build_crank_grip` L247-298 + REVOLUTE `arm_to_grip` L412-418 | eligible if compatible | 曲柄臂端 knuckle 上 grip 段 REVOLUTE 绕 -X 折叠（部署/收纳），整体仍 CONTINUOUS 旋转（+1 嵌套 joint）|

> 2 candidate（降级理由 + conditional）：crank handle 仅 one-piece / folding 两个真实形态；且本 slot **conditional**——仅 sharpening=crank_helical 时存在（fixed_wedge 无 crank，本 slot 强制 one_piece 占位、不发件），见 §9。

### Slot E：mount（桌面固定机构）

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| top_clamp_posts（基线） | rec_..._38af6c5b（S0；多数 fork） | `_build_clamp_post` L174-192 + `for idx` 2 posts FIXED L314-333 | eligible if compatible | 顶部 deck 上 2 根 cylindrical clamp post（FIXED part 对，via `for i in range(2)`）|
| under_body_gclamp | rec_..._clamp_gclamp（S10） | `_build_clamp_frame` L199-289（FIXED C 框）+ `_build_thumbscrew` L290-355 + PRISMATIC `frame_to_thumbscrew` L501-507 | eligible if compatible | body 下挂 C 形 clamp frame（FIXED）+ thumbscrew pad PRISMATIC 绕 +Z 顶住桌底（夹桌沿）|

> 2 candidate（降级理由）：mount 只有 top posts（FIXED 对）与 under-body G-clamp（FIXED 框 + PRISMATIC 螺杆）两个真实形态。

## 槽位图（slot graph）

```
pattern: mixed（root housing + parallel children + hole multiplicity）

                         housing (root, body_form ∈ {box, barrel, teardrop})
                          │   坐标：footprint 居中 XY，底面 z=0，前面 +X，右侧 +Y，上 +Z
        ┌─────────────────┼──────────────┬───────────────────┬──────────────────┐
        │                 │              │                   │                  │
  cutter[N] +        crank /          shavings_container   crank_handle        mount
  pencil_port[N]     wedge blade      (Slot C)             (Slot D, cond.)     (Slot E)
  (mult: Slot B)     (Slot B)
        │                 │              │                   │                  │
  port cut + cutter  crank: CONTINUOUS  captured: 无件      one_piece: 无件     posts: 2×FIXED
  FIXED @ port 深处  绕 +Y @ 右壁       drawer: PRISMATIC   folding: REVOLUTE   gclamp: FIXED 框
  (front face +X)    wedge: FIXED 钢片  +X @ 前下           绕 -X @ 臂端 knuckle + PRISMATIC
                     @ port (无 crank)  lid: REVOLUTE -Y                        螺杆绕 +Z @ 框
                     ⇒主运动转 shavings @ 后上接缝
                                        canister: CONTINUOUS
                                        绕 +Z @ cutter 下
```

接口点位（每条 housing→child 连接）：
- **housing → cutter_{i}**：mating = port throat 深处（origin 在 housing 局部 (0,0,0)，cutter mesh 自带 port 中心定位），joint = FIXED，cutter 前端 recessed 在前面后方。grandfathered（cutter 捕获在 throat 内）。
- **housing → crank**：mating = 右壁 axle 孔（`origin=(0, RIGHT_WALL_Y, AXLE_Z)`），joint = CONTINUOUS，axis `(0,1,0)`；crank axle stub 捕获在 housing axle bore（element-scoped allow_overlap）。
- **housing → drawer**：mating = 前下 drawer cavity 底（`origin=(drawer_closed_x, 0, DRAWER_CENTER_Z−DRAWER_H/2)`），joint = PRISMATIC，axis `(1,0,0)`，range `[0, DRAWER_TRAVEL]`；drawer tray 在 cavity 内（allow_overlap）。
- **housing → bin_shell（FIXED）→ bin_lid（REVOLUTE）**：bin_shell FIXED 在后下；bin_lid mating = 盖-唇后边接缝（`origin=(−BIN_L/2, 0, BIN_H)`），joint = REVOLUTE，axis `(0,−1,0)`，range `[0, ~1.6]`。
- **housing → canister**：mating = cutter 下 housing 底面 socket（`origin=(0,0,0)` 在 canister socket 中心），joint = CONTINUOUS，axis `(0,0,1)`；collar 旋入 socket（allow_overlap）。
- **crank → crank_grip（folding）**：mating = 臂端 knuckle（`origin=(0, KNUCKLE_Y, KNUCKLE_Z)`），joint = REVOLUTE，axis `(−1,0,0)`；grip knuckle 捕获在臂端（allow_overlap）。
- **housing → clamp_post_{i}（FIXED）**：mating = 顶 deck（`origin=(x_off, 0.012, 0)`），joint = FIXED ×2。
- **housing → clamp_frame（FIXED）→ thumbscrew（PRISMATIC）**：frame FIXED 在 body 下；thumbscrew mating = 框下横梁顶（`origin=(SCREW_X, 0, CF_LOWER_TOP)`），joint = PRISMATIC，axis `(0,0,1)`，pad 上行夹桌。
- **互斥/可选/派生**：Slot B 的 fixed_wedge ⇒ 无 crank ⇒ Slot D 强制 one_piece（占位、不发件）且 Slot C 强制 hinged_bin_lid（保运动）；Slot D 仅在 crank_helical 时真实发件（conditional）；cutter 个数由 §8 n_holes **派生**；各 mount/shavings/handle 挂不同面，互不干涉。

## 每槽位 Module Emits / Interfaces

### Slot A / module box_charcoal
| emits | 描述 | 来源 |
|---|---|---|
| parts | `housing`（root，`housing_shell` visual：圆角矩形 slab + shoulder + port/throat/axle cut）+ inline `front_plate` visual | S0 / `_build_housing` L74-122、front_plate L161-171 |
| internal joints | 无（root）| — |
| downstream interface | 前面 +X（port/cutter）、右壁 +Y（crank axle）、顶 deck（posts）、底/后/前下（shavings/mount）| S0 / L74-122 |

### Slot A / module cylindrical_barrel
| emits | 描述 | 来源 |
|---|---|---|
| parts | `housing`（revolved 圆桶 + dome 顶 + foot ring + band + port/axle cut）| S7 / `_build_barrel` L85-160 |
| downstream interface | 同 box 但 footprint 为圆（port 在桶壁 +X，crank 在 +Y 桶壁）| S7 / L128-158 |

### Slot A / module teardrop_shell
| emits | 描述 | 来源 |
|---|---|---|
| parts | `housing`（lofted 流线壳，后宽前窄 + port/axle cut）| S8 / `_build_teardrop_shell` L78-161 |
| downstream interface | 同上但 footprint 为 teardrop（前部窄收向 port）| S8 / L78-161 |

### Slot B / module crank_helical
| emits | 描述 | 来源 |
|---|---|---|
| parts | `crank`（axle+flange+hub+arm+grip）；`cutter_{i}` helical cutter（FIXED，随 n_holes）| S0 / crank L195-264、cutter L125-158 |
| internal joints | `housing_to_crank` CONTINUOUS axis (0,1,0)；`housing_to_cutter_{i}` FIXED | S0 / L351-359、L305-311 |
| upstream interface | 右壁 axle bore（crank）/ port throat 深处（cutter）| S0 / axle_bore L113-120 |

### Slot B / module fixed_wedge_blade
| emits | 描述 | 来源 |
|---|---|---|
| parts | `blade`（横跨 port 的 fixed 钢片，FIXED）；无 crank；cutter 省略（blade 取代）| S3 / `_build_blade` L143-164 |
| internal joints | `housing_to_blade` FIXED；主运动改由 Slot C 强制 hinged_bin_lid 提供 | S3 / L313-323 |
| upstream interface | port 跨面（blade）| S3 / L143-164 |

### Slot C / module captured_cavity
| emits | 描述 | 来源 |
|---|---|---|
| parts | 无独立 part（屑留 housing 内腔）| S0 |
| internal joints | 无 | — |

### Slot C / module sliding_drawer
| emits | 描述 | 来源 |
|---|---|---|
| parts | `drawer`（open-top tray + faceplate + grip）；housing 增 drawer cavity/rails cut | S1 / `_build_drawer` L220-273、cavity L137-167 |
| internal joints | `housing_to_drawer` PRISMATIC axis (1,0,0) range [0,TRAVEL] | S1 / L455-465 |
| upstream interface | 前下 drawer cavity 底（rails 承托）| S1 / L137-167 |

### Slot C / module hinged_bin_lid
| emits | 描述 | 来源 |
|---|---|---|
| parts | `bin_shell`（FIXED open-top tray）+ `bin_lid`（顶盖，hinge 边在 part frame x=0）| S2 / shell L215-272、lid L275-311 |
| internal joints | `housing_to_bin_shell` FIXED；`bin_to_lid` REVOLUTE axis (0,−1,0) range [0,~1.6] | S2 / L464-469、L487-493 |
| upstream interface | 后下 housing 内（shell）；盖-唇后边接缝（lid hinge）| S2 / L487-493 |

### Slot C / module twist_canister
| emits | 描述 | 来源 |
|---|---|---|
| parts | `shavings_canister`（revolved cup + 螺纹 collar，collar 朝 +Z 入 socket、cup 朝 −Z）| S4 / `_build_canister` L303-379 |
| internal joints | `housing_to_canister` CONTINUOUS axis (0,0,1) | S4 / L469-475 |
| upstream interface | cutter 下 housing 底面 socket | S4 / L455-475 |

### Slot D / module one_piece
| emits | 描述 | 来源 |
|---|---|---|
| parts | 无独立 part（grip 已 union 进 crank）| S0 / L241-259 |
| internal joints | 无 | — |

### Slot D / module folding_grip
| emits | 描述 | 来源 |
|---|---|---|
| parts | crank 拆 `crank_arm`（不含 grip）+ `crank_grip`（独立段）| S9 / arm L191-246、grip L247-298 |
| internal joints | `arm_to_grip` REVOLUTE axis (−1,0,0)（绕臂端 knuckle 折叠）| S9 / L412-418 |
| upstream interface | 臂端 knuckle（grip 捕获）| S9 / L412-418 |

### Slot E / module top_clamp_posts
| emits | 描述 | 来源 |
|---|---|---|
| parts | `clamp_post_{1,2}`（2 根 cylindrical post，via `for i in range(2)`）| S0 / `_build_clamp_post` L174-192、loop L314-333 |
| internal joints | `housing_to_clamp_post_{1,2}` FIXED ×2 | S0 / L327-333 |
| upstream interface | 顶 deck | S0 / L314-333 |

### Slot E / module under_body_gclamp
| emits | 描述 | 来源 |
|---|---|---|
| parts | `clamp_frame`（FIXED C 框）+ `thumbscrew`（pad）| S10 / frame L199-289、screw L290-355 |
| internal joints | `housing_to_clamp_frame` FIXED；`frame_to_thumbscrew` PRISMATIC axis (0,0,1) | S10 / L479-484、L501-507 |
| upstream interface | body 下面（frame）；框下横梁顶（screw） | S10 / L501-507 |

### hole multiplicity / module cutter+port_{i}（见 §8）
| emits | 描述 | 来源 |
|---|---|---|
| parts | `cutter_{i}`（helical cutter visual）× N；housing 上对应 port cut × N | S5/S6 循环 L301-323 / L296-323 |
| internal joints | `housing_to_cutter_{i}` FIXED × N | S5 / L315-320 |
| upstream interface | 各 port throat 深处（housing 前面对应 cut）| S5 / `_cut_port` L81-103 |

## 参数范围汇总
| 参数 | 类型 | 取值范围 / 候选值 | 标称默认 | 约束类型 | 约束 / 函数 | 来源 |
|---|---|---|---|---|---|---|
| body_form | enum | {box_charcoal, cylindrical_barrel, teardrop_shell} | box_charcoal | choice | deterministic procedural sampler 选择 | Slot A 表 |
| sharpening_mechanism | enum | {crank_helical, fixed_wedge_blade} | crank_helical | choice | sampler 选择 | Slot B 表 |
| shavings_container | enum | {captured_cavity, sliding_drawer, hinged_bin_lid, twist_canister} | captured_cavity | choice | sampler 选择；fixed_wedge 时强制 hinged_bin_lid | Slot C 表 |
| crank_handle | enum | {one_piece, folding_grip} | one_piece | conditional | 仅 crank_helical 时真实发件；fixed_wedge ⇒ 强制 one_piece 占位 | Slot D 表 |
| mount | enum | {top_clamp_posts, under_body_gclamp} | top_clamp_posts | choice | sampler 选择 | Slot E 表 |
| n_holes | int | [1, 3] | 1 | independent | 加权采样（1 偏多）后 clamp | §8 / S5/S6 |
| body_scale | float | [0.88, 1.15] | 1.0 | independent | 缩放 housing footprint（BODY_W/D 或 barrel R 或 teardrop 包络），clamp | S0 L34-37 |
| crank_arm_scale | float | [0.85, 1.15] | 1.0 | independent | 缩放 CRANK_ARM_LEN；clamp 保曲柄不越出 housing 高度 | S0 L53 |
| drawer_travel_scale | float | [0.85, 1.15] | 1.0 | conditional | sliding_drawer 存在时缩放 DRAWER_TRAVEL；clamp 使全开仍部分保留在 housing | S1 L76 |
| (—) | constraint | — | — | inequality | n_holes 个 port 横向等距须落在前面可用宽度内（`n_holes·port_pitch ≤ usable_face_w·body_scale`）；越界回缩 pitch/孔径 | §8 / S5/S6 footprint |
| (—) | constraint | — | — | inequality | drawer/canister/bin 容器包络 ≤ housing lower 体量（容器尺寸随 body_scale 缩放并 clamp，避免穿出壳体）| 接口 / clearance |

连续 scale 默认独立采样 → conditional（crank_handle/ drawer_travel 按上游 enum 解析）→ inequality 把 port 阵列/容器包络投影回可行域。全部在 `resolve_config` 内求解。

## Multiplicity / Copy Logic

sharpening-hole 是唯一多重性来源，含 **1 根 count 轴**：

- `count_param`: **`n_holes`**（前面削笔孔 + 对应 cutter 的个数）
- `N_range`: `n_holes ∈ [1, 3]`（产品域；1=标准单孔、2=标准+jumbo 双孔、3=small/medium/large 三孔）；已覆盖样本：S0=1、S5=2、S6=3
- sampling domain（权重档）：1 孔高频（标准削笔器），2/3 孔渐稀（多孔少见长尾）
- copied object: 单个 `cutter_{i}` part（helical cutter visual）+ 其 FIXED joint；几何由共享 helper `_build_cutter(y_offset, throat_r)` 复用；housing 上对应 port 由 `_cut_port(y_offset, outer_r, throat_r)` cut
- naming: `cutter_{i}` / `housing_to_cutter_{i}`，`for i in range(n_holes)`（S5/S6 已用此结构，直接作 module 源码）
- placement: 前面（+X）沿 Y 横向等距排列，孔径可渐变（S6 small/medium/large）；中心 `port_y[i]`、孔径 `throat_r[i]` 按 n_holes 居中分布并经 body_scale 缩放、clamp 进可用面宽
- joint policy: 每 cutter **独立** FIXED，挂 housing；cutter 捕获在各自 port throat 内（grandfathered + allow_overlap）
- source/gating: 循环范式 S5 L301-323 / S6 L296-323；`_cut_port` + `_build_cutter` 共享 helper；fixed_wedge 机构下 cutter 省略（blade 取代），但 port cut 仍按 n_holes 保留以读身份

## 拓扑多样性审计

总组合数（离散槽，扣互斥）：body_form(3) × sharpening(2) × shavings(4) × crank_handle(cond) × mount(2)。
- crank_helical 分支：3 × 1 × 4 × 2(handle) × 2(mount) = 48
- fixed_wedge 分支：3 × 1 × 1(强制 hinged_bin) × 1(handle 占位) × 2(mount) = 6
→ 离散组合 ≈ **54**，再叠 n_holes(3 值) ≈ **160+ distinct 拓扑**（多数 n_holes 产生不同 cutter 计数 = 不同 part/joint 拓扑等价类）。

理由：仅离散槽组合即 54 >10；n_holes 轴再乘 3，distinct 拓扑充裕。

seed_domain_policy：procedural_first
Procedural Sampling / Sweep Plan：`config_from_seed(seed)` 对所有普通 seed 做 deterministic procedural sampling——加权选 5 个离散 slot（shavings 偏 captured/drawer，body 偏 box，mechanism 偏 crank，handle 偏 one_piece，mount 偏 posts）、加权采 n_holes（1 偏多）、采连续 scale，经 `resolve_config` 的兼容矩阵 + inequality 求解。`seed=0` 不特殊。无 regression overrides（若 sweep 暴露特定 seed 失败再稀疏加并注明）。
Topology target：1000-seed slot choice tuple distinct 富类别建议 ≥300（report-only）（本类 ~160 上界可达；若实测低于 300，多因 n_holes 或 handle 碰撞，可调宽权重）。
Controlled local parameterization：初版含 `body_scale` / `crank_arm_scale` / `drawer_travel_scale`，全部 clamp/conditional，受 footprint 包络、容器嵌合、crank 高度约束，不改拓扑或类别身份。

| item | policy | validator / viewer focus |
|---|---|---|
| sampler | slot 顺序：body→mechanism→shavings→handle→mount→n_holes→scales；加权 | slot_choices_for_seed 与 build choices 一致 |
| compatibility matrix | fixed_wedge ⇒ 无 crank ⇒ shavings 强制 hinged_bin_lid + handle 强制 one_piece；crank_helical 时 handle 自由；twist_canister + under_body_gclamp 空间互斥 ⇒ mount 退 top_clamp_posts；sliding_drawer + cylindrical_barrel（圆面无法容纳深抽屉腔）⇒ shavings 退 captured_cavity；n_holes port 落前面可用宽；容器（drawer/bin）包络随 body footprint clamp（`_drawer_dims` / `_bin_dims`），不穿壳 | 无穿模/悬空/越界 port、crank 不越壳、容器不穿壳/不severing shell、≥1 非 fixed joint |
| controlled local variation | body_scale / crank_arm_scale / drawer_travel_scale + clamp | 比例变化不破坏 port 捕获、crank 接触、容器嵌合、joint origin、身份 |
| regression overrides | none（初版无）| 仅 sweep 暴露的具体失败 seed |
| random sweep | 初轮 seeds 0-49，成熟 0-999 | 与 contract 失败 |

| slot | candidate_count | 是否 ≥2 | 是否 ≥3 | 备注 |
|---|---:|---|---|---|
| A body_form | 3 | yes | yes | box / barrel / teardrop |
| B sharpening_mechanism | 2 | yes | no | crank（CONTINUOUS）/ fixed_wedge（运动转 shavings）|
| C shavings_container | 4 | yes | yes | captured / drawer / bin_lid / canister |
| D crank_handle | 2 | yes | no | one_piece / folding；conditional（仅 crank）|
| E mount | 2 | yes | no | top posts / under-body g-clamp |
| (mult) n_holes | [1-3] | — | — | 多重性轴，cutter/port 计数乘子 |

## Validator

- slot_choices_for_seed returns implemented module names（body_form / sharpening_mechanism / shavings_container / crank_handle / mount + n_holes）
- config_from_seed uses deterministic procedural sampling for all ordinary seeds（seed=0 不特殊）
- compatibility matrix / gating：fixed_wedge ⇒ no crank + 强制 hinged_bin_lid + one_piece handle；crank_handle conditional；n_holes port 落前面可用宽；容器包络 ≤ housing
- 始终 ≥1 非 fixed joint（crank CONTINUOUS / drawer PRISMATIC / lid REVOLUTE / canister CONTINUOUS / thumbscrew PRISMATIC / folding grip REVOLUTE）
- optional regression overrides 初版为空
- controlled local scale params 全部 clamp，且不破坏 port 捕获 / crank 接触 / 容器嵌合 / joint origin / 身份
- cross-part scale 依赖（port 阵列 vs 前面宽、容器 vs housing、crank vs 高度）在 `resolve_config` 内求解
- critical captured-pin / seated overlap：cutter-throat、crank axle-bore、drawer-cavity、canister collar-socket、folding grip knuckle、thumbscrew-frame
- key joints：crank CONTINUOUS +Y；drawer PRISMATIC +X；bin lid REVOLUTE -Y；canister CONTINUOUS +Z；folding grip REVOLUTE -X；thumbscrew PRISMATIC +Z；cutter/post/blade/frame FIXED
- copied objects 遵循 `cutter_{i}` 命名 + 横向等距 placement + 统一 FIXED joint policy
- front plate inline body visual（不建 FIXED 装饰 part）；cutter/blade/bin shell/clamp post/clamp frame 做 FIXED part 为上游既有约定（grandfathered 接受）

## Reject cases

- fixed_wedge 机构却仍发 crank，或 fixed_wedge 下 shavings 不强制 hinged_bin（导致全 fixed joint、无运动）。
- crank_handle=folding 却在 fixed_wedge（无 crank）下发件（孤立 grip）。
- n_holes 个 port 越出前面可用宽度（穿出 housing 侧壁或互相重叠）。
- 容器（drawer/canister/bin/g-clamp）尺寸随 scale 涨穿出 housing 壳体或悬空不接触。
- crank axle stub 不入 housing axle bore（曲柄悬浮），或 folding grip knuckle 不捕获在臂端。
- drawer 全开完全脱出 housing（未保留部分插入），或 bin lid hinge origin 不在盖-唇接缝（幻影铰）。
- 用连续尺寸/颜色冒充拓扑：只改 body 尺寸而不换 body_form/mechanism/shavings/mount/n_holes 就当新拓扑。
- cutter 用手写命名的 2-3 个代替 `for i in range(n_holes)` 循环（多重性退化）。
- config_from_seed 采样到未实现组合（如 fixed_wedge+canister、folding 在 wedge 下）。

## 与相邻类别的边界

- 不该混入：电动削笔器 electric sharpener（无手摇曲柄 + 桌沿夹，按钮启动马达；本类 = 手摇曲柄 + 削笔孔 + 屑盒）。
- 不该混入：袖珍刀片削笔器 prism/wedge pocket sharpener（纯一块塑料 + 刀片，无 housing/曲柄/屑盒机构，体量过小）。
- 不该混入：卷笔刀玩具 / 文具收纳座 / 订书机（无削笔 port + cutter + 曲柄身份）。
- Stationary 大类内：区别于 Calculator / Clip / Stapler / Pen 等无削笔孔 + 曲柄文具。

## 审核记录
| 项 | 结论 |
|---|---|
| reviewer status | pending |
| reviewer notes | 待人工审核。降级点：①Slot B/D/E 各 2 candidate（单 parent 小类，sharpening/handle/mount 真实形态各仅 2 个，扩容须回 fork 池补造）；②fixed_wedge 的互斥门控（强制 hinged_bin + one_piece）是否认可，或改为 fixed_wedge 不进 seed domain；③n_holes [1-3] N_range 与权重档。|

## 模板实现备注（可选）

- 共享 helper：`_build_housing_mesh`（按 body_form 分 box/barrel/teardrop 三路）、`_build_cutter_mesh`（按 n_holes 循环 + `_cut_port` cut）、`_build_crank_mesh`（one_piece：grip union；folding：arm + 独立 grip）、容器各自 helper（drawer/bin/canister/gclamp）。
- captured-pin / seated overlap 需 element-scoped allow_overlap：cutter-throat、crank axle-bore、drawer-cavity、canister collar-socket、folding grip knuckle、thumbscrew-frame、bin lid-shell 接缝。
- 派生与门控集中在 `resolve_config`：crank_handle conditional（wedge ⇒ one_piece）、shavings 强制（wedge ⇒ hinged_bin）、n_holes port 阵列投影、容器包络 clamp。
- 模板实现前先深读 `calculator.py`（root chassis + parallel children + 多重性按键阵列，运动拓扑相近）与 `shopping_bucket.py`（parallel children + 多容器/handle 槽 + captured-pin bail），不按类别名相似选。

## Module Source Index

| source_id | slot | module | sample_id | model.py 来源 | 采纳用途 |
|---|---|---|---|---|---|
| S0 | A/B/D/E/mult | box / crank / one_piece / posts / 单孔基线 | rec_..._38af6c5b | `_build_housing` L74-122、`_build_crank` L195-264、`_build_cutter` L125-158、`_build_clamp_post` L174-192、CONTINUOUS crank L351-359 | box 壳 + crank + cutter + 2 posts + 单孔 + 坐标基线 |
| S1 | C | sliding_drawer | rec_sharpener_var_drawer_shavings | `_build_drawer` L220-273、cavity/rails L137-167、PRISMATIC `housing_to_drawer` L455-465 | 抽屉 PRISMATIC + cavity 嵌合 |
| S2 | C | hinged_bin_lid | rec_sharpener_var_lid_hinged_bin | `_build_bin_shell` L215-272、`_build_bin_lid` L275-311、REVOLUTE `bin_to_lid` L487-493 | bin shell（FIXED）+ 翻盖 REVOLUTE |
| S3 | B | fixed_wedge_blade | rec_sharpener_var_mechanism_wedge | `_build_blade` L143-164、`_build_shavings_lid` L199-252、REVOLUTE lid L328-334 | fixed 钢片 + 运动转 shavings lid |
| S4 | C | twist_canister | rec_sharpener_var_canister_twist | `_build_canister` L303-379、CONTINUOUS `housing_to_canister` L469-475 | 旋拧 canister CONTINUOUS |
| S5 | mult | n_holes=2 | rec_sharpener_var_holes_dual | `_cut_port` L81-103、`_build_cutter(y)` L139-171、`for i` L301-323 | 双孔循环范式 |
| S6 | mult | n_holes=3 | rec_sharpener_var_holes_triple | `_cut_port` L110-132、`_build_cutter` L134-168、`for` L296-323 | 三孔渐变循环 |
| S7 | A | cylindrical_barrel | rec_sharpener_var_body_cylindrical | `_build_barrel` L85-160（revolve + dome + foot + band）| 圆桶旋转壳 |
| S8 | A | teardrop_shell | rec_sharpener_var_body_teardrop | `_build_teardrop_shell` L78-161（loft 流线壳）| teardrop loft 壳 |
| S9 | D | folding_grip | rec_sharpener_var_handle_folding | `_build_crank_arm` L191-246、`_build_crank_grip` L247-298、REVOLUTE `arm_to_grip` L412-418 | 折叠 grip 嵌套 REVOLUTE |
| S10 | E | under_body_gclamp | rec_sharpener_var_clamp_gclamp | `_build_clamp_frame` L199-289、`_build_thumbscrew` L290-355、PRISMATIC `frame_to_thumbscrew` L501-507 | G-clamp 框 + 螺杆 PRISMATIC |
