# sentry_turret — modular spec

## 元信息
| 项 | 值 |
|---|---|
| slug | `sentry_turret` |
| template path | `agent/templates/Military_Turret.py` |
| test path (optional) | `tests/agent/test_sentry_turret_template.py` |
| stage | `SPEC_ONLY_DRAFT` |
| status | `pending` |
| __modular__ | `True` |
| pattern | `mixed`（base→yaw→pitch→recoil 串链 + barrel-count 与 leg-count 两根 multiplicity 轴）|

## 5 星样本阅读摘要
| 项 | 值 |
|---|---|
| five_star_total | 9 |
| read_count | 9 |
| read_scope | all 5-star samples in Military/Turret (1 parent + 8 variants), all rating=5 |
| source_index_policy | only adopted module sources are indexed below |

读取结果（全部 9 个 model.py 完整读取，record.json `rating=5` 全部确认）：

- **parent** `rec_model-a-stylized-...-abo_..._d996dd05` — 主结构：4-leg splayed base → hex yaw collar (continuous +Z) → boxy receiver (revolute pitch, -Y trunnion axis, -10..+60°) → 2×2 barrel cluster (prismatic recoil -X, 0..0.06 m)。这是整套链的标准蓝本：`_segment_tube`/`_rect_tube`/`_perforated_shell`/`_receiver_shell`/`_round_tube` helpers + trunnion captured-pin overlap + barrel snug-bore overlap。
- **tripod3** `rec_sentry_turret_var_tripod3` — 与 parent 几乎逐行相同，仅 `LEG_ANGLES=(0,120,240)`；leg-count 轴 N=3 证据。
- **legs6** `rec_sentry_turret_var_legs6` — 同上，`LEG_ANGLES=(0,60,120,180,240,300)`；leg-count 轴 N=6 证据；footprint 测试放宽到 1.0–1.65 m。
- **pedestal** `rec_sentry_turret_var_pedestal` — base 换成 lathe-revolved 实心鼓形柱（`_pedestal_drum` profile.revolve）+ shaft band + base rim + N=8 flange bolt ring；**无 LEG_ANGLES**；yaw joint 重命名 `pedestal_to_collar_yaw`。
- **ceiling** `rec_sentry_turret_var_ceiling` — base 换成倒挂 ceiling plate（`_ceiling_plate_mesh` 带 cable bore + bolt holes）+ 下垂 mounting boss + `_boss_shroud_mesh` 穿孔罩 + shroud rings；collar **倒挂**（hex 向下 extrude、cheeks/discs 负 z）；turret 悬于 plate 下方；yaw `plate_to_collar_yaw`；**无 LEG_ANGLES**。
- **single1** `rec_sentry_turret_var_single1` — barrel-count N=1：`BARREL_OFFSETS=[(0.0,0.0)]`；**parent 的嵌套 `for sz/for sy` 2×2 循环被重写成单层 `for i,(dy,dz) in enumerate(BARREL_OFFSETS)` offset-list 循环**，receiver 前壁 bore 也改成 `for dy,dz in BARREL_OFFSETS`；carriage 缩小到 (0.10,0.12,0.12)。
- **twin2** `rec_sentry_turret_var_twin2` — barrel-count N=2：`BARREL_OFFSETS=[(-BARREL_DY,0),(+BARREL_DY,0)]`，水平并排；同一 offset-list 代码路径。
- **cameraoptic** `rec_sentry_turret_var_cameraoptic` — Slot C optics=camera：receiver top deck 上加 `_camera_sight_body`（mounting base + 圆角主体 + 镜头凹槽 + visor hood）+ `sight_lens` 玻璃盘 + `sight_led` 红色状态灯；随 pitch 一起俯仰。
- **laserpod** `rec_sentry_turret_var_laserpod` — Slot C optics=laser_pod：receiver 侧上方 `sensor_pod_body` 圆柱 pod + `sensor_lens`（暗红玻璃）+ `sensor_rear_cap` + `sensor_mount_bracket`（侧壁支架，桥接到 receiver 侧壁保证连通）；随 pitch 一起俯仰。

结构变化轴（真实拓扑差异，非纯尺寸/配色）：(A) base 安装形态 4 种 part-tree 完全不同；(B) barrel 数量 N（offset-list 多重复制 + receiver bore 数量随之变化）；(C) optics 是否存在及形态（0 / camera / laser_pod，不同 visual 子树）。leg-count 是 Slot A 内 leg-type base 的次级 N 轴。

## 核心身份

一台 **automated sentry gun turret（自动哨戒炮塔）**：一个接地（或天花板倒挂）的安装座承载一个 **continuous yaw（连续平转）** 旋转台，旋转台上是一对 trunnion 耳轴承托的 **revolute pitch（俯仰，-10..+60°）** receiver（机匣/炮身壳），机匣前方伸出一束 **prismatic recoil（后坐，沿炮管轴 0..0.06 m）** 的炮管簇。核心运动语义 = **2-DOF 瞄准链（pan 连续偏航 + tilt 俯仰）+ 后坐**，三者在所有 base / barrel-count / optics 候选中**必须全部保留**。默认成熟域：~1.1–1.3 m 高，industrial / safety-yellow 风格的固定式自动机炮。

不该混入：
- **手持/单兵枪械、便携 SMG**：哨戒炮塔是**固定安装、自动平转俯仰**的台座武器，无握把/枪托/扳机人因结构，不是手持物。
- **坦克/装甲车炮塔（Tank turret）**：那是带装甲壳体、车体集成、大口径单管的载具子系统；本类是裸露三脚/柱座 + 多管小炮 + 外露 trunnion 的独立机器人哨戒单元。
- **监控摄像头云台 / PTZ 球机**：同样 pan+tilt，但 sentry_turret 的标识是**炮管簇 + 后坐 prismatic + trunnion 俯仰耳轴**；纯光学云台没有炮管与后坐，camera 在本类只是 Slot C optics 配件而非主体。
- **天文望远镜赤道仪 / 三脚架仪器**：虽有 tripod + 俯仰，但望远镜镜筒沿光轴、无后坐、无炮管簇、无机匣机炮语义。

## 槽位 + 候选模块表

### Slot A：base_mount（root + yaw stage carrier）

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| splayed_quad_legs | rec_model-a-stylized-...-abo_..._d996dd05 | L186-L261（leg loop L198-223 + column/platform L226-261；helpers `_rect_tube` L87-94, `_segment_tube` L97-110, `_perforated_shell` L119-137）| eligible if compatible | 4 根 box-section splayed 撑腿（`LEG_ANGLES=(45,135,225,315)`）落在圆 foot pad 上 + 中央 gunmetal 立柱 + perforated 黄罩 + ring platform；接地 z≈0，~1.2 m footprint；leg-count N 轴宿主 |
| tripod_3legs | rec_sentry_turret_var_tripod3 | L186-L261（同上结构，`LEG_ANGLES=(0,120,240)` L36）| eligible if compatible | 同撑腿系统但 3 腿 120° 等距；与 quad 共享 leg/column/platform 全部 helper；leg-count N 轴宿主 |
| solid_pedestal | rec_sentry_turret_var_pedestal | L149-L187（drum L152-156 + band/rim L159-173 + bolt ring L178-187；helper `_pedestal_drum` L79-94）| eligible if compatible | lathe-revolved 实心鼓形柱（base flange→shaft→top flange，`profile.revolve`）+ shaft band + dark base rim + 地面 flange 上 N=8 bolt 环；**无腿、无 LEG_ANGLES**；圆形 footprint ≥0.7 m |
| ceiling_mount | rec_sentry_turret_var_ceiling | L221-L278（plate L224-229 + boss L235-243 + shroud L246-251 + rings L254-278；helpers `_ceiling_plate_mesh` L104-121, `_boss_shroud_mesh` L124-146, `_shroud_ring_mesh` L149-160）| eligible if compatible | 平圆 ceiling plate（cable bore + 周边 N=8 bolt holes）+ 下垂 mounting boss + 穿孔 shroud + annular rings；turret 整体悬于 plate 下方；**无 LEG_ANGLES**；倒挂坐标（plate 顶面在 CEILING_Z_TOP=1.15） |

degrade note：Slot A 有 4 个结构互异候选，满足 3-6 目标，无降级。

### Slot B：barrel_count（recoiling muzzle multiplicity）

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| single_barrel{1} | rec_sentry_turret_var_single1 | cluster loop L337-371 + receiver bore loop L158-166 + `BARREL_OFFSETS` L76 | eligible if compatible | `BARREL_OFFSETS=[(0,0)]`，单根居中炮管 + shroud + cap；**offset-list 单层循环 `for i,(dy,dz) in enumerate(BARREL_OFFSETS)`**；receiver 前壁按 offset 数量切 bore；carriage 缩小 (0.10,0.12,0.12) |
| twin_barrel{2} | rec_sentry_turret_var_twin2 | cluster loop L339-371 + bore loop L160 + `BARREL_OFFSETS` L75 | eligible if compatible | `BARREL_OFFSETS=[(-BARREL_DY,0),(+BARREL_DY,0)]`，水平并排双管；**同一 offset-list 代码路径**（正是它逼出 2×2→list 重写） |
| quad_barrel{4} | rec_model-a-stylized-...-abo_..._d996dd05 | cluster loop L353-375 + receiver bore loop L157-166 | eligible if compatible | parent 的 2×2 grid（`±BARREL_DY × ±BARREL_DZ`，嵌套 `for sz/for sy`）；模板中**改写为 4-entry offset-list** `[(±DY,±DZ)]` 走统一路径 |

degrade note：Slot B 直接来源只覆盖 N∈{1,2,4} 三个结构样本（3 candidate，满足最低 3）。模板侧把 N 暴露为 **`barrel_count` multiplicity 参数 N_range [1,6]**，由统一 offset-list placement 规则（见 §Multiplicity 轴 1）程序化生成 N=3/5/6 的 offset 表，无需新增样本——这是 multiplicity 轴而非新 candidate。

### Slot C：optics（sensor/sight mounted on receiver，rides with pitch）

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| none | rec_model-a-stylized-...-abo_..._d996dd05 | receiver block L305-323（`top_panel_{i}` only，无 optics elem）| eligible if compatible | 裸 receiver 顶面，仅双 recessed `top_panel_{i}`，无瞄具；基线 |
| camera | rec_sentry_turret_var_cameraoptic | helper `_camera_sight_body` L195-(~231) + emit `sight_body`/`sight_lens`/`sight_led` L401-438 | eligible if compatible | recessed-lens 摄像瞄准头（mounting base + 圆角主体 + 镜头凹槽 + visor hood）+ 玻璃镜盘 + 红色状态 LED；bolt 在 receiver top deck，随 pitch 俯仰 |
| laser_pod | rec_sentry_turret_var_laserpod | sensor consts L88-96 + emit `sensor_pod_body`/`sensor_lens`/`sensor_rear_cap`/`sensor_mount_bracket` L338-379 | eligible if compatible | receiver 侧上方圆柱 sensor/激光 pod（+X 对齐）+ 前暗红玻璃 lens + rear cap + 侧壁 mount bracket（桥接 receiver 侧壁保连通），随 pitch 俯仰 |

degrade note：Slot C 有 3 个结构互异候选（无瞄具 / 顶置摄像 / 侧置激光 pod），满足 3-6 目标，无降级。

## 槽位图（slot graph）

pattern: mixed（serial chain + 两根 slot-level multiplicity）

```
[Slot A base_mount] --(base_to_collar_yaw: CONTINUOUS axis +Z, origin@座顶面 YAW_Z)-->
[support_collar (固定结构层，hex collar + 2× trunnion cheek/flange/disc)]
  --(collar_to_receiver_pitch: REVOLUTE axis -Y, origin@pivot, lower=-10° upper=+60°)-->
[Slot C optics 挂在 receiver 上 (FIXED-on-receiver visual 子树, 随 pitch)]
[receiver]
  --(receiver_to_barrels_recoil: PRISMATIC axis -X, origin@receiver frame, 0..RECOIL_TRAVEL)-->
[Slot B barrel_count (N 根炮管簇, 共享单一 recoil prismatic)]
```

接口点位与装配：
- **A→collar（yaw）**：collar 的 `hex_collar` 下/上面贴座顶；接口面 = base 顶面（leg-type=ring_platform 顶 z=0.78；pedestal=top flange z=0.78；ceiling=mounting boss 底 z=1.05 倒挂）。yaw joint origin 落在该接触面 (0,0,YAW_Z)，axis 恒为 +Z CONTINUOUS。`support_collar` 是模板内**固定结构层（非 slot）**，所有 base 候选都用同一 collar（leg/pedestal 直立、ceiling 倒挂镜像 z）。upstream 接口 anchor 法向分量为 0（child collar 局部原点落在 yaw 轴上）。
- **collar→receiver（pitch）**：collar 的 `trunnion_cheek_{0,1}` / `trunnion_disc_{0,1}` 抱 receiver 的 `trunnion_pin_{0,1}`（captured-pin，沿 ±Y）。pitch joint origin 在 pivot（leg/pedestal world z≈1.02；ceiling world z≈0.81），axis=(0,-1,0)，REVOLUTE lower=-10° upper=+60°。
- **optics→receiver**：Slot C 的瞄具 visual 直接 emit 到 `receiver` part（非独立 part、非新 joint），随 pitch 一起动；camera 坐 receiver 顶 deck，laser_pod 经 `sensor_mount_bracket` 桥到 receiver 侧壁（保 part 内连通）。
- **receiver→barrels（recoil）**：`barrel_{i}` 穿过 receiver 前壁 snug bore（captured slide，沿 +X），`carriage` 嵌在 receiver 腔内。recoil joint origin = receiver frame 原点，axis=(-1,0,0)，PRISMATIC 0..RECOIL_TRAVEL。N 根炮管**共享这一个** prismatic。

互斥/派生：
- Slot A 是 root，三链 joint 名前缀随 base（leg/pedestal=`base_to_collar_yaw`/`pedestal_to_collar_yaw`；ceiling=`plate_to_collar_yaw`）。模板统一为一个 yaw articulation，parent part 名随 base 候选。
- **leg-count N 轴仅对 leg-type base（splayed_quad_legs / tripod_3legs）生效**；solid_pedestal / ceiling_mount 无 `LEG_ANGLES`，忽略该参数。
- Slot C 与 Slot A/B 正交（任意 base × 任意 barrel_count × 任意 optics 合法）。

## 每槽位 Module Emits / Interfaces

### Slot A / module splayed_quad_legs（及 tripod_3legs 同构）
| emits | 描述 | 来源 |
|---|---|---|
| parts | `leg_strut_{i}` / `knee_gusset_{i}` / `foot_pad_{i}`（i over LEG_ANGLES）、`column_core`、`column_base_cap`、`perforated_wrap`、`hub_ring`、`ring_platform`、`platform_lip`（同一 `leg_base` part）| parent L186-261 |
| internal joints | 无（legs 在 leg_base 内刚性，无 per-leg joint）| parent L198-223 |
| upstream interface | 接地：foot pad 底 z≈0；leg_base 为 root（无 upstream joint）| parent L218-223 |
| downstream interface | ring_platform 顶面 z=0.78（+Z face）→ 供 yaw joint mate | parent L250-255 |

### Slot A / module solid_pedestal
| emits | 描述 | 来源 |
|---|---|---|
| parts | `pedestal_drum`、`shaft_band`、`base_rim`、`flange_bolt_{i}`（i<8，同一 `pedestal` part）| pedestal L149-187 |
| internal joints | 无 | — |
| upstream interface | 接地：base flange 底 z≈0；root | pedestal L152-156 |
| downstream interface | top flange 顶面 z=0.78（+Z face）→ yaw joint mate | pedestal L88-94 |

### Slot A / module ceiling_mount
| emits | 描述 | 来源 |
|---|---|---|
| parts | `plate_body`、`mounting_boss`、`boss_shroud`、`shroud_ring_top`、`shroud_ring_bot`、`bolt_head_{i}`（同一 `ceiling_plate` part）| ceiling L221-278 |
| internal joints | 无 | — |
| upstream interface | 固定于天花板：plate 顶 z=CEILING_Z_TOP=1.15；root | ceiling L224-229 |
| downstream interface | mounting_boss 底面 z=1.05（-Z face，倒挂）→ yaw joint mate；collar 与 turret 悬于其下 | ceiling L240-243 |

### support_collar（模板内固定结构层，所有 base 共用）
| emits | 描述 | 来源 |
|---|---|---|
| parts | `hex_collar`、`trunnion_cheek_{0,1}`、`cheek_flange_{0,1}`、`trunnion_disc_{0,1}`（同一 `support_collar` part）| parent L264-292 / ceiling L281-321（倒挂镜像）|
| upstream interface | hex_collar 贴座顶/座底（CONTINUOUS +Z yaw，effort=80 vel=2）| parent L294-302 |
| downstream interface | trunnion cheek/disc 抱 receiver pin（REVOLUTE -Y pitch）；captured-pin overlap | parent L273-292 |

### receiver（含 Slot C optics 子树）
| emits | 描述 | 来源 |
|---|---|---|
| parts | `receiver_shell`（mesh，含 N 个 barrel bore + 2 top pocket）、`top_panel_{0,1}`、`trunnion_pin_{0,1}`；+optics: camera→`sight_body`/`sight_lens`/`sight_led`，laser_pod→`sensor_pod_body`/`sensor_lens`/`sensor_rear_cap`/`sensor_mount_bracket`（同一 `receiver` part）| parent L305-323 / camera L401-438 / laser L338-379 |
| internal joints | 无（optics 随 receiver 刚性，无独立 joint）| — |
| upstream interface | trunnion_pin（pitch revolute，axis -Y，lower=-10° upper=+60°，effort=60 vel=1.5）| parent L325-334 |
| downstream interface | receiver 前壁 snug bore（PRISMATIC -X recoil，0..RECOIL_TRAVEL，effort=50 vel=1.0）| parent L377-386 |

### Slot B / barrel_cluster（N 根，统一 offset-list）
| emits | 描述 | 来源 |
|---|---|---|
| parts | `carriage`、`barrel_{i}`、`muzzle_shroud_{i}`、`muzzle_cap_{i}`（i over BARREL_OFFSETS，同一 `barrel_cluster` part）| single1 L337-371 |
| internal joints | 无（N 根炮管刚性同簇）| — |
| upstream interface | barrel 穿 receiver 前壁 bore（snug captured slide）+ carriage 嵌 receiver 腔 | single1 L373-382 |
| downstream interface | muzzle cap 端（炮口，链终点）| single1 L366-371 |

## 参数范围汇总
| 参数 | 类型 | 取值范围 / 候选值 | 标称默认 | 约束类型 | 约束 / 函数 | 来源 |
|---|---|---|---|---|---|---|
| base_choice | enum | splayed_quad_legs / tripod_3legs / solid_pedestal / ceiling_mount | — | choice | deterministic procedural sampler；进 slot_choice | Slot A 表 |
| barrel_count | int(N) | [1, 6] | 4 | independent(N) | offset-list 长度；偏小加权采样后 clamp；进 slot_choice（按 N 命名）| Slot B / §Multiplicity 轴1 |
| optics_choice | enum | none / camera / laser_pod | none | choice | deterministic procedural sampler；进 slot_choice | Slot C 表 |
| leg_count | int(N) | {3, 4, 6} | 4 | conditional(N) | 仅 leg-type base 用（`LEG_ANGLES` 长度）；非 leg base 忽略；进 slot_choice（leg-type base 时）| §Multiplicity 轴2 |
| palette_style | enum | 见 §palette_style（5 个 colorway，各含 finish）| safety_yellow_gunmetal | palette | palette only（含 finish 维度），**不计入 slot_choice** | §palette_style |
| pitch_range_scale | float | [0.85, 1.15] | 1.0 | independent | 缩放 pitch upper（+60°·scale），lower 固定 -10°；clamp upper∈[+45°,+70°] | parent L84 |
| recoil_travel_scale | float | [0.85, 1.15] | 1.0 | independent | 缩放 RECOIL_TRAVEL（0.06·scale），clamp∈[0.045,0.075]，保 carriage 后坐后仍 ≥0.05 m 嵌 receiver | parent L83/L568-577 |
| barrel_len_scale | float | [0.90, 1.12] | 1.0 | independent | 缩放炮管/shroud/cap 长（BARREL_X1, SHROUD_X0, CAP_X0 沿 +X），clamp，保 bore engage ≥0.10 m | parent L77-81 |
| support_scale | float | [0.92, 1.10] | 1.0 | independent | 缩放 base 高度族（leg HUB/KNEE/FOOT z 或 PEDESTAL_TOP_Z 或 CEILING_Z_TOP），同比抬 yaw/pitch 接口高度，clamp 保接地/接顶 | parent L37-52 / pedestal L37-44 |
| footprint_scale | float | [0.90, 1.15] | 1.0 | conditional | 仅 leg/pedestal base 用：缩放 FOOT_PT 半径 / BASE_PLATE_R，clamp 保 footprint∈类别域 1.0–1.7 m；ceiling 忽略 | parent L39 / pedestal L40 |
| (—) | constraint | — | — | inequality | 接口闭合可行域：(a) yaw 接口面高度随 support_scale 缩放后 collar/pivot/receiver 链整体平移跟随，base 仍接地（leg/pedestal）或接顶（ceiling）；(b) barrel snug-bore engage = (barrel front − receiver front wall) ≥0.10 m，barrel_len_scale 不得使炮管缩到脱出 bore；(c) 后坐到底 carriage ∩ receiver ≥0.05 m；(d) N 根 barrel offset 落在 receiver 前壁内（offset 半径 ≤ RECV_HALF_Y/HALF_Z − BORE_R），违反则回缩 BARREL_DY/DZ 或拒绝重采 | 接口 / clearance / §Multiplicity |

连续尺寸采样契约：先采 independent（pitch_range / recoil_travel / barrel_len / support）→ conditional（footprint_scale 仅 leg/pedestal、leg_count 仅 leg-type、barrel offset 表随 N）按上游 choice 解析 → inequality 把 barrel engage / 后坐残留 / offset-在前壁内 投影回缩或拒绝重采。所有约束在 `resolve_config` 求解，不留到 builder。

### palette_style（colorway，5 个：1 源自 5★ 实测 + 4 realistic-for-the-class 推断）

每 colorway = 一组协调配色（body/safety accent + metal hardware + barrel + lens）外加一个 `finish` 描述子（palette 内显式材质-表面维度，**非独立 slot / 非 slot_choice**，驱动模板侧 material metallic/roughness）。`finish` 取值域 `{satin_painted_steel, semigloss_painted_steel, matte_military_paint, anodized_gunmetal, glossy_drone_plastic}`。base colorway 沿用 parent 实测 material 名/rgba（`safety_yellow`/`gunmetal`/`dark_grey_steel`/`barrel_grey`）；其余 4 色族为该类目（自动机炮常见涂装：安全黄、军绿、纯枪铁灰、白色无人机哨戒）realistic-for-the-class 推断。

| palette_style | finish | body / safety accent | metal hardware | barrel / lens | 来源样本 |
|---|---|---|---|---|---|
| safety_yellow_gunmetal（基线）| satin_painted_steel | `safety_yellow` (0.95,0.76,0.10) | `gunmetal` (0.15,0.16,0.18) / `dark_grey_steel` (0.24,0.25,0.27) | `barrel_grey` (0.46,0.47,0.50) | parent L30-33 |
| industrial_yellow_black | semigloss_painted_steel | `safety_yellow` (0.96,0.74,0.08) | `near_black` (0.09,0.09,0.10) | `barrel_grey` (0.44,0.45,0.48) | inferred（工业机黄+黑五金）|
| military_green | matte_military_paint | `olive_drab` (0.27,0.31,0.18) | `gunmetal` (0.15,0.16,0.18) | `barrel_grey` (0.40,0.41,0.43) | inferred（军用橄榄绿哨戒炮）|
| gunmetal_mono | anodized_gunmetal | `gunmetal` (0.20,0.21,0.23) | `dark_grey_steel` (0.13,0.13,0.15) | `barrel_grey` (0.46,0.47,0.50) | inferred（全枪铁灰一体涂装）|
| white_drone | glossy_drone_plastic | `drone_white` (0.90,0.91,0.92) | `light_grey` (0.55,0.56,0.58) | `barrel_grey` (0.48,0.49,0.52) | inferred（白色无人哨戒单元）|

> palette_style 每 seed 采样（`rng.choice(PALETTE_STYLES)`），保证 swept 输出 color-diverse；**仍为 palette-only，不计入 slot_choice、不改任何 slot / candidate / multiplicity / joint / dimension / topology**。`finish` 仅作 palette 内显式材质-表面维度，不引入新关节/新件/新拓扑等价类。camera/laser lens 玻璃色（红/暗红）由 optics module 固定发射，不随 palette_style 变（瞄具光学件标识色）。

## Multiplicity / Copy Logic

本小类有 **2 根独立 multiplicity 轴**：(1) barrel count（所有 base 通用），(2) leg count（仅 leg-type base）。每根轴各自加权采样、各自编进 slot_choice、各自 clamp、sweep 各自设上限。

### 轴 1：barrel_count — Slot B 多重复制（所有 base 通用）
- **count_param**：`barrel_count`（= `len(BARREL_OFFSETS)`，offset 列表 `[(dy,dz), ...]`，相对 pitch 轴）
- **N_range**：声明产品域 **[1, 6]**；`config_from_seed` sweep 采样域 [1, 6] 全程（N 小、编译轻，无需缩窄）。
- **sampling domain**：`rng.choices((1,2,3,4,5,6), weights=偏小，4 略加权（parent 基线）)`；`resolve_config` 把任意外部 N clamp 到 [1,6]。
- **copied object**：一根炮管三件（`barrel_{i}` 圆管 + `muzzle_shroud_{i}` + `muzzle_cap_{i}`）+ receiver 前壁一个对应 guide bore。
- **naming**：单索引 `{i}` over `enumerate(BARREL_OFFSETS)`；统一 `for i,(dy,dz) in enumerate(BARREL_OFFSETS)`（**parent 嵌套 2×2 循环须重写成单层 offset-list 循环**——single1/twin2 已证；模板对 N=4 也用 4-entry offset-list，不保留嵌套）。
- **placement**：N 根的 offset 表程序化生成（`BARREL_OFFSETS_FOR_N`）——N=1 居中 `[(0,0)]`；N=2 水平并排 `[(∓DY,0)]`；N=4 用 `±DY×±DZ` 2×2；N=3/5/6 沿 pitch 轴对称环/列排（落在 receiver 前壁内：offset 半径 ≤ RECV_HALF_Y/HALF_Z − BORE_R）。每根在 `(BARREL_X0/SHROUD_X0/CAP_X0, dy, dz)`，receiver 前壁切一个匹配 bore。
- **joint policy**：N 根**共享单一** `receiver_to_barrels_recoil` prismatic（-X，0..RECOIL_TRAVEL）；barrel count **永不新增 joint**。
- **source/gating**：copy-logic 源取 single1 L337-371（offset-list 循环）+ receiver bore loop L158-166；**不取 parent**（parent 是未循环化的嵌套 2×2）。

### 轴 2：leg_count — Slot A 次级多重复制（仅 leg-type base）
- **count_param**：`leg_count`（= `len(LEG_ANGLES)`，Z 偏航角元组）
- **N_range**：声明产品域 **{3, 4, 6}**（已覆盖：tripod3=3 / parent=4 / legs6=6；样本无 5 故 N=5 不入域，避免奇对称落脚不均）；sweep 域同 {3,4,6}。
- **sampling domain**：`rng.choice((3,4,6))`（4 略加权，parent 基线）；仅当 base_choice∈{splayed_quad_legs, tripod_3legs} 时采样，否则该参数无效。
- **copied object**：一条腿组（`leg_strut_{i}`（共享 `leg_mesh`/`_segment_tube`）+ `knee_gusset_{i}` + `foot_pad_{i}`）。
- **naming**：索引 `{i}` over `enumerate(LEG_ANGLES)`；`LEG_ANGLES = tuple(round(k*360/N) for k in range(N))`。
- **placement**：每条腿绕 Z `yaw=radians(ang)`，knee/foot 落在公共地面圆 `(FOOT_PT.r·cos/sin, …)`，feet 接地 z≈0、等角分布。
- **joint policy**：legs 在 `leg_base` 内刚性，**无 per-leg joint**。
- **source/gating**：copy-logic 源取 parent L198-223（leg loop）；tripod3 L36 + legs6 L36 证明 `LEG_ANGLES` 即 count_param。**仅 leg-type base 生效**——solid_pedestal / ceiling_mount 无 `LEG_ANGLES`，slot_choice 不带 leg 维。

## 拓扑多样性审计

总组合数：
- Slot A base = 4
- Slot B barrel_count N ∈ [1,6] = 6
- Slot C optics = 3
- leg_count（仅 leg-type 2 个 base 各 ×3，pedestal/ceiling 各 ×1）

合法 (base, leg/none, barrel_N, optics) tuple 数 = `[(splayed×3 legN) + (tripod×3 legN) + pedestal×1 + ceiling×1] × 6 barrelN × 3 optics` = `(3+3+1+1) × 6 × 3 = 8 × 18 = 144` 个 distinct slot_choice 拓扑组合。

理由：144 个合法 slot_choice 组合，远超 10 distinct 下限；单 50-seed sweep 即可轻松覆盖 ≥10（base×barrelN×optics×legN 四维交叉）。

seed_domain_policy：procedural_first
Procedural Sampling / Sweep Plan：`config_from_seed(seed)` 用 `rng = Random(seed)` 顺序采样 base_choice → optics_choice → barrel_count（偏小加权）→ leg_count（仅 leg-type base，否则跳过）→ palette_style → 连续 scale。`slot_choices_for_seed` 返回 `[("base_mount", base), ("barrel_count", f"{N}_barrel"), ("optics", optics)]`（leg-type base 时追加 `("leg_count", f"{L}_leg")`）。compatibility matrix：base × barrel × optics 全正交（无非法组合）；leg_count gate 仅在 leg-type base 解锁；ceiling base 走倒挂坐标分支（collar/pivot 负 z 镜像）。少量 regression overrides：none（首版不预置）。random sweep / viewer 目检：seeds 0-49 初轮，0-999 成熟审计。
Topology target：1000-seed slot choice tuple distinct 富类别建议 ≥300（report-only）；144 个合法组合下可达，无类别约束压低。
Controlled local parameterization：关键连续 scale = `support_scale`（base 高度族 + 链平移）、`footprint_scale`（leg/pedestal 落脚半径，conditional）、`barrel_len_scale`（炮管长 + bore engage 保持）、`recoil_travel_scale`（行程 + 后坐残留嵌合）、`pitch_range_scale`（俯仰上限）。全部在 `resolve_config` clamp / 派生，按 §7 约束类型声明（independent / conditional / inequality）并遵守采样契约；跨部件依赖（barrel engage、后坐残留、offset 在前壁内、接地/接顶）显式落 inequality 行，不当独立自由变量。它们只改安全比例/clearance，不破 InterfaceSpec / MatingContract / 两根 multiplicity。

| item | policy | validator / viewer focus |
|---|---|---|
| sampler | base→optics→barrel_count(偏小)→leg_count(条件)→palette→scales；slot_choice 含 base/barrel_N/optics(+leg_N) | slot_choices_for_seed matches build choices |
| compatibility matrix | base×barrel×optics 全正交合法；leg_count 仅 leg-type base 解锁；ceiling 走倒挂坐标分支 | no floating, collision, axis, max multiplicity, optics 漂浮, ceiling 倒挂接顶 |
| controlled local variation | support/footprint/barrel_len/recoil/pitch scale + clamp；barrel offset 表随 N 生成并约束在前壁内 | proportions vary；barrel 仍 engage bore、carriage 仍嵌 receiver、base 仍接地/接顶、pitch/yaw/recoil 语义不变 |
| regression overrides | none | — |
| random sweep | seeds 0-49 初轮，0-999 成熟审计 | 与 contract failures |

| slot | candidate_count | 是否 ≥2 | 是否 ≥3 | 备注 |
|---|---:|---|---|---|
| A base_mount | 4 | yes | yes | splayed_quad / tripod / pedestal / ceiling |
| B barrel_count | 3 直接源（N∈{1,2,4}）→ 模板 multiplicity [1,6] | yes | yes | offset-list 多重复制轴 |
| C optics | 3 | yes | yes | none / camera / laser_pod |
| (A 次轴) leg_count | 3（N∈{3,4,6}）| yes | yes | 仅 leg-type base |

## Validator

- slot_choices_for_seed returns implemented module names（base / `{N}_barrel` / optics，leg-type 时含 `{L}_leg`）
- config_from_seed uses deterministic procedural sampling for all ordinary seeds（含 seed 0）
- compatibility matrix / gating prevents illegal combinations（leg_count 仅 leg-type base；ceiling 倒挂分支）
- optional regression overrides are sparse and justified（首版 none）
- final templates do not endlessly cycle a small curated table as the main seed domain
- controlled local scale params clamped；cross-part deps（barrel engage / 后坐残留 / offset 在前壁内 / 接地接顶）resolved in `resolve_config`
- critical InterfaceSpec / MatingContract exist：yaw 座顶接触、pitch trunnion captured-pin、recoil snug-bore
- key joints have expected type/axis/range：yaw CONTINUOUS +Z；pitch REVOLUTE -Y lower=-10° upper=+60°；recoil PRISMATIC -X 0..RECOIL_TRAVEL
- copied objects follow naming/placement：`barrel_{i}`/`muzzle_shroud_{i}`/`muzzle_cap_{i}` over BARREL_OFFSETS；`leg_strut_{i}`/`knee_gusset_{i}`/`foot_pad_{i}` over LEG_ANGLES
- N 根 barrel 共享单一 recoil prismatic（barrel count 不加 joint）；N 条 leg 在 leg_base 内刚性（leg count 不加 joint）

## Reject cases

- yaw 非 CONTINUOUS +Z，或 pitch 非 REVOLUTE -Y / 限位偏离 -10..+60° / recoil 非 PRISMATIC -X —— 破坏 2-DOF 瞄准 + 后坐核心身份。
- barrel-count 用嵌套 2×2 循环（未重写成 offset-list），导致 N≠1/4 时炮管放置/命名错乱或 bore 数对不上。
- barrel offset 落到 receiver 前壁外（offset 半径 > RECV_HALF − BORE_R），炮管悬空或穿出壳体。
- barrel_len_scale 把炮管缩到脱出 receiver guide bore（engage <0.10 m），或后坐到底 carriage 脱出 receiver（残留 <0.05 m）。
- leg_count 被施加到 solid_pedestal / ceiling_mount（这些 base 无 LEG_ANGLES），产生悬空腿或重复结构。
- ceiling_mount 未走倒挂坐标分支（collar/pivot/receiver 仍朝上），turret 不悬于 plate 下方或不接顶。
- support_scale 缩放后 base 脱离接地（leg/pedestal foot 离地）或 ceiling plate 脱离接顶，链整体未跟随平移。
- optics 子树漂浮（camera 不坐 receiver 顶 deck / laser_pod bracket 不桥 receiver 侧壁），未随 pitch 俯仰。
- palette_style 被误计入 slot_choice，或 finish 引入新关节/新拓扑等价类。

## 与相邻类别的边界

- 不该混入：**Tank turret（坦克炮塔）**——载具集成、装甲壳体、大口径单管、无外露三脚/柱座与裸 trunnion；本类是独立固定式机器人哨戒单元（多管小炮 + 外露俯仰耳轴 + 后坐）。
- 不该混入：**手持枪械 / 单兵武器**——本类是固定安装、自动 pan+tilt 台座武器，无握把/枪托/扳机人因结构。
- 不该混入：**PTZ 监控云台 / 球机**——虽同 pan+tilt，但缺炮管簇与后坐 prismatic；camera 在本类仅为 Slot C 配件，非主体。
- 不该混入：**天文望远镜赤道仪 / 三脚架仪器**——镜筒沿光轴、无后坐、无炮管簇与机匣机炮语义。

## 审核记录
| 项 | 结论 |
|---|---|
| reviewer status | pending |
| reviewer notes | — |

## 模板实现备注（可选）

- 共享 helper：`_rect_tube`/`_segment_tube`（leg struts，splayed/tripod 共用）、`_perforated_shell`（leg-base 立柱罩）、`_round_tube`（barrel/shroud/cap，所有 N 共用）、`_receiver_shell`（按 BARREL_OFFSETS 切 bore）、`_pedestal_drum`（pedestal）、`_ceiling_plate_mesh`/`_boss_shroud_mesh`/`_shroud_ring_mesh`（ceiling）、`_camera_sight_body`（camera optics）。
- 关键 InterfaceSpec / MatingContract：yaw 座顶接触（leg/pedestal +Z；ceiling -Z 倒挂）grandfather-friendly 平面贴合；pitch trunnion 是 captured-pin（pin-through-cheek，**omit mating**，grandfather）；recoil 是 snug captured slide（barrel-through-bore，**omit mating**，grandfather）。
- captured-pin overlap 需 element-scoped `allow_overlap`：`trunnion_pin_{i}` ↔ `trunnion_cheek_{i}`/`trunnion_disc_{i}`（parent L404-412）；`barrel_{i}` ↔ `receiver_shell`（snug bore，每根 N，parent L426-433）。
- ceiling base 倒挂：collar hex 向下 extrude、cheeks/discs/pivot 取负 z（ceiling L283-321）；pitch/yaw axis 不变，仅几何镜像 + 接口高度上移。
- 暂不入 seed domain：barrel N=5/6 与 N≥3 的 offset 表需在实现时校验 receiver 前壁容纳上限（`offset_radius ≤ RECV_HALF − BORE_R`）；若某 N 的对称 offset 排不下，clamp 回缩 BARREL_DY/DZ 或在该 base 下降级该 N。

## Module Source Index

| source_id | slot | module | sample_id | model.py 来源 | 采纳用途 |
|---|---|---|---|---|---|
| S1 | Slot A | splayed_quad_legs | rec_model-a-stylized-...-abo_..._d996dd05 | L186-261（helpers L87-137）| leg loop part tree + downstream platform interface + leg-count 轴 |
| S2 | Slot A | tripod_3legs | rec_sentry_turret_var_tripod3 | L36, L186-261 | leg-count N=3（LEG_ANGLES 证据）|
| S3 | Slot A | (leg-count N=6) | rec_sentry_turret_var_legs6 | L36, L480-505 | leg-count N=6（LEG_ANGLES + footprint 放宽）|
| S4 | Slot A | solid_pedestal | rec_sentry_turret_var_pedestal | L79-94, L149-187 | lathe drum part tree + top-flange yaw interface |
| S5 | Slot A | ceiling_mount | rec_sentry_turret_var_ceiling | L104-160, L221-321 | 倒挂 plate/boss/shroud part tree + 倒挂 collar 镜像 |
| S6 | Slot B | single_barrel{1} | rec_sentry_turret_var_single1 | L76, L141-166, L337-382 | offset-list 单层循环重写（copy-logic 主源）+ receiver bore loop |
| S7 | Slot B | twin_barrel{2} | rec_sentry_turret_var_twin2 | L75, L160, L339-371 | offset-list N=2 placement 证据 |
| S8 | Slot B | quad_barrel{4} | rec_model-a-stylized-...-abo_..._d996dd05 | L73-81, L353-375 | 2×2 grid offset 值（改写为 4-entry list）|
| S9 | Slot C | camera | rec_sentry_turret_var_cameraoptic | L195-231, L401-438 | camera sight 子树 + lens + LED emit |
| S10 | Slot C | laser_pod | rec_sentry_turret_var_laserpod | L88-96, L338-379 | sensor pod 子树 + bracket-to-sidewall 连通 |
| S11 | (collar/receiver) | support_collar + receiver | rec_model-a-stylized-...-abo_..._d996dd05 | L264-334（collar/pitch）, L377-386（recoil）| 固定结构层 collar + trunnion captured-pin + recoil prismatic |
