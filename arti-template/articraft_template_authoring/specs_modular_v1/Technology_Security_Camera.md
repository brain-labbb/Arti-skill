# Technology / Security_Camera — modular spec

## 元信息
| 项 | 值 |
|---|---|
| slug | `Technology_Security_Camera` |
| template path | `agent/templates/Technology_Security_Camera.py` |
| test path (optional) | (none; sweep-pipeline is the acceptance signal) |
| stage | `SPEC_ONLY_DRAFT` |
| status | `pending` |
| __modular__ | `True` |
| pattern | `linear_chain`（mount → [pan] → carriage → [tilt] → camera_head；multiplicity on IR ring + lens）|

## 5 星样本阅读摘要
| 项 | 值 |
|---|---|
| five_star_total | 10（4 origin + 6 fork）|
| read_count | 10 |
| read_scope | all synced rating-5 samples in this picture 小类 (4 origins + 6 forks) |
| source_index_policy | only adopted module sources are indexed（§14）|

## 核心身份

一台**2 自由度（pan+tilt）铰接式监控摄像机** = 刚性安装座 + 云台（pan/tilt）+ 摄像头。
全部 10 个样本共享同一条抽象云台脊柱

    rigid_mount --[pan REVOLUTE]--> carriage --[tilt REVOLUTE]--> camera_head

pan 轴取安装面法向、经过 pan 轴承（轴对称 knuckle）；tilt 轴水平（+Y），经过被
yoke 夹持的横销。默认成熟域：室内/室外定点监控（bullet / box / speed_dome /
turret_eyeball 四种外壳），装在桌面座 / 壁挂臂 / 壁挂直装 / 吸顶四种安装座上。

**不该混入**：webcam（无铰接安装座、无云台，直接夹在屏幕上）；floodlight 安防灯
（有云台臂但主体是灯板、无镜头/成像头）；PTZ 云台**空**支架（无摄像头）。判据：
必须同时具备"可动安装云台 + 带镜头的成像头"。

## 槽位 + 候选模块表

### Slot A ①：mount_form（安装座骨架，4 锚全源）

| module_name | source_type | source evidence | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|---|
| desk | forked_anchor | S1 `…1da3777f` | L38-59（base/stand）| eligible if compatible | 圆底盘 base_plate + 立柱 pedestal → 顶部 pan 座；pan 轴 +Z |
| wall_arm | forked_anchor | S2 `…03a3be03` / wall_arm_pan | L31-59 | eligible if compatible | 壁板 Box(0.015,0.14,0.16) + 水平 support_arm(Cyl,X)；pan 轴 +X（bullet/box）/+Z（dome/turret）|
| wall_direct | forked_anchor | S3 `…4e6dbba3` | L99-148 | eligible if compatible | 壁板 + 短 stub 直接托 pan 座（无臂）；pan 轴 +X（bullet/box）|
| ceiling | forked_anchor | S4 `…1892c604` / ceiling_bullet | L58-90 | eligible if compatible | 吸顶盘 ceiling_plate + 下垂 drop_stem → 座在下方；pan 轴 +Z |

### Slot B ②：gimbal_dof（云台自由度 = 骨架图差异）

| module_name | source_type | source evidence | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|---|
| full_pan_tilt | forked_anchor | S1/S3/S4 | S1 L248-265 / S4 L199-216 | eligible if compatible | 3 part / 2 REVOLUTE：mount→carriage(pan)→camera_head(tilt) |
| tilt_only | forked_anchor | S2 `…03a3be03` | L178-186 | eligible if compatible | 2 part / 1 REVOLUTE：yoke 折入 mount，mount→camera_head(tilt)，无 pan part/joint |

### Slot C ③：housing_form（主体形态家族 / Primary Form Family）

| module_name | source_type | source evidence | model.py:Lx-Ly | form_subtype | 结构特征 |
|---|---|---|---|---|---|
| bullet | forked_anchor | S1/S2/S3 | S1 L117-141 | Volumetric Envelope Form | 水平圆柱体 Cyl(r≈0.045,L≈0.17,轴X)+后盖，前脸镜头 |
| box | forked_anchor | box_housing@S3 | L176-246 | Volumetric Envelope Form | 长方体 Box(0.185,0.10,0.10)+后板，矩形化外壳 |
| speed_dome | forked_anchor | S4 / wall_arm_dome | S4 L31-90 | Macro Surface Construction | 上截锥罩(canopy Cyl)+透明球罩 Sphere(dome_cover) + 内 camera_core |
| turret_eyeball | forked_anchor | turret_eyeball@S4 | L35-163 | Volumetric Envelope Form | 浅座圈 socket_ring + 半球/球眼 eyeball Sphere(r≈0.05) |

四个 candidate 同 part tree（都是 camera_head 单件 + tilt 轴销）、同 primitive 家族
（Box/Cylinder/Sphere）、同接口（tilt 横销 + 前脸镜头/IR 环），只换核心 part 的可识别
几何形态原型 → 合法结构差异（AUTHORING §A Rule 3 / SPEC §8.5 ③）。

### Slot D：sunshield（遮阳罩，仅 bullet/box）

| module_name | source_type | source evidence | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|---|
| none | forked_anchor | S2/S4 | — | eligible if compatible | 无 |
| short_lip | forked_anchor | S1 `…1da3777f` | L211-216 | eligible if compatible | 短平顶 visor Box(0.075,·,0.005)贴体顶 |
| long_hood | forked_anchor | S3/box_housing/ceiling_bullet | S3 L33-60 | eligible if compatible | 长顶板 + 两侧板 U 型罩，前伸过镜头（保持在 pivot 之前）|

> gate：dome/turret 不佩遮阳罩（§9）；sunshield candidate 为宿主表面派生的 host-conformal 装饰几何（Rule 4）。

## 槽位图（slot graph）

pattern: linear_chain（+ multiplicity on camera_head）

```
mount --[pan REVOLUTE, axis=mount_normal, origin=seat, 经 pan_knuckle 轴承]--> carriage
      --[tilt REVOLUTE, axis=+Y, origin=seat+(_TILT_DX,0,0), 经 yoke 横销 tilt_pin]--> camera_head
（gimbal_dof=tilt_only 时：删去 carriage 与 pan joint，yoke 建在 mount 上，mount --[tilt]--> camera_head）
```

- **接口点位**：pan = mount 的 `seat_boss`（轴对称座）↔ carriage 的 `pan_knuckle`（轴对称 barrel），
  两者同轴过盈（captured bearing）；tilt = carriage/mount 的 `yoke_cheek_{0,1}`+`tilt_pin` 夹持
  camera_head 的 `tilt_trunnion`（同轴过盈）。
- **joint type/axis/range**：均 REVOLUTE；pan 轴=mount 法向（bullet/box：desk/ceiling→+Z，wall→+X；
  dome/turret：恒 +Z 吊装），range 由 `clamp_joint_limits` 对 mount keepout 求解；tilt 轴 +Y（`(0,-1,0)`），
  候选 [−1.0, 0.7]，由 solver 对 yoke/mount keepout 收缩。
- **互斥/派生**：carriage 仅在 full_pan_tilt 存在；speed_dome 强制 full_pan_tilt（无 pan 即废）。

## 每槽位 Module Emits / Interfaces

### Slot A / mount
| emits | 描述 | 来源 |
|---|---|---|
| parts | `mount`（base/pedestal 或 wall_plate/arm 或 ceiling_plate/drop_stem + `seat_boss`）| S1 L38-111 / S2 L31-92 / S3 L99-148 / S4 L58-90 |
| internal joints | 无（全为 parent visual）| — |
| downstream interface | `seat`（xyz）+ pan_axis（法向）+ `seat_boss` 座 | S1 L248-256 / S4 L199-207 |

### Slot B / carriage（full_pan_tilt）
| emits | 描述 | 来源 |
|---|---|---|
| parts | `carriage`（`pan_knuckle` + `yoke_bridge` + `yoke_cheek_{0,1}` + `tilt_pin`）| S1 L63-111 stand-yoke / S4 L92-122 pan_carriage |
| upstream interface | pan REVOLUTE，origin=seat，axis=pan_axis | S1 L248-256 |
| downstream interface | tilt pivot = seat+(_TILT_DX,0,0)，yoke 横销 | S1 L257-265 |

### Slot C / camera_head
| emits | 描述 | 来源 |
|---|---|---|
| parts | `camera_head`（housing + `front_bezel` + `lens_barrel_j`/`front_glass_j`×N + `ir_led_i`×N + sunshield + `tilt_trunnion`/`mount_ear`）| S1 L117-228 / S4 L124-197 |
| internal joints | 无（IR/lens/sunshield 均 parent visual，Rule 1）| — |
| upstream interface | tilt REVOLUTE 的 child，`tilt_trunnion` 含 (0,0,0) | S2 L178-186 |

## 参数范围汇总
| 参数 | 类型 | 取值范围 / 候选值 | 标称默认 | 约束类型 | 约束 / 函数 | 来源 |
|---|---|---|---|---|---|---|
| mount_form | enum | desk/wall_arm/wall_direct/ceiling | — | choice | 采样器 | Slot A |
| gimbal_dof | enum | tilt_only/full_pan_tilt | — | conditional | speed_dome→full_pan_tilt | Slot B |
| housing_form | enum | bullet/box/speed_dome/turret_eyeball | — | conditional | 依 mount 合法集（§9）| Slot C |
| sunshield | enum | none/short_lip/long_hood | — | conditional | 仅 bullet/box，否则 none | Slot D |
| palette_style | enum | silver/white/gray/black | — | choice | 采样器 | §8.5 ⑥ |
| ir_count | int(mult) | {4,8,20} | 8 | choice | 加权（4/8 高频，20 稀有）| S2/S4/S1 |
| lens_count | int(mult) | {1,2} | 1 | choice | 加权（1 主，2 稀有）| dual_lens@S1 |
| body_len_scale | float | [0.85,1.18]→clamp[0.8,1.25] | 1.0 | independent | 独立采样 clamp | S1-S4 body L |
| body_radius_scale | float | [0.88,1.12]→clamp[0.82,1.18] | 1.0 | independent | 独立采样 clamp | S1-S4 body r |
| mount_scale | float | [0.9,1.15]→clamp[0.85,1.2] | 1.0 | independent | 独立采样 clamp | mount 尺寸 |
| (—) | constraint | — | — | inequality | pan/tilt range = `clamp_joint_limits` 对 mount/yoke keepout 求解，exempt captured 轴承对 | 接口/clearance |

所有 equation/inequality/conditional 均在 `resolve_config`（enum gate）与 build 内 `_solve_gimbal_limits`
（joint range）求解，不留到 builder 失败。

### 7.5 编译预算 / compile budget
每-seed 预算 **≤ 12s**（实测 0.1–0.3s/seed）。全部 Box/Cylinder/Sphere，无 cadquery / lathe / 布尔；
Sphere/Cylinder 默认分档；IR 环 N 个相同 LED 复用同一 helper。远低于预算，`--compile-timeout 120` 为 10× 看门狗。

## Multiplicity / Copy Logic

两根独立 multiplicity 轴，各自加权采样、各自编进 `slot_choices`、各自 clamp：

- **IR-LED 环 N**：`count_param=ir_count`，`N_range={4,8,20}`（三档全源：S2=4, S4=8, S1=20）。
  sampling domain：权重 (0.42,0.42,0.16)（小 N 高频、20 稀有）。copied object = `ir_led_i`
  Cylinder，`for i in range(N)` 沿前脸环形（`angle=2πi/N+π/N`，半径 host-conformal 贴前脸），
  parent visual（不动→非 part），嵌入 `front_bezel`。source/gating：全源，免 fork。
- **lens_module N**：`count_param=lens_count`，`N_range={1,2}`（1=多源，2=dual_lens@S1 fork）。
  sampling domain：权重 (0.78,0.22)。copied object = `lens_barrel_j`+`front_glass_j`，`for j`
  在 y 偏移 (0 或 ±0.024) 发射，parent visual。模板可外推 3（未启用）。

## 视觉多样性 6 轴考察

| 轴 | 怎么判断 | 有/无 | 取值/范围 + source_type / 来源 · 或理由 |
|---|---|---|---|
| ① 骨架图 | 加/减会动的 part 或一条边 | 有 | gimbal_dof：full_pan_tilt（3 part/2 revolute，S1/S3/S4）vs tilt_only（2 part/1 revolute，S2）——增/删 pan part+joint；forked_anchor |
| └ multiplicity | 同构件 ×N | 有 | 见 §8：IR N∈{4,8,20}、lens N∈{1,2} |
| ② 关节类型 | 图不变换 type/轴 | 有 | pan/tilt 均 REVOLUTE；pan 轴随 mount 法向变（+Z desk/ceiling & dome/turret；+X wall bullet/box）——轴变体；每种在 sweep 出现（axis_realization）；forked_anchor |
| ③ 主体形态家族 | 换核心 part 可识别形态原型 | 有 | 4 candidate：bullet/box（Volumetric Envelope）、speed_dome（Macro Surface）、turret_eyeball（Volumetric Envelope），均登记进 `slot_choices`，forked_anchor；见 Slot C |
| ④ 表面装饰 | 叠加表面细节/改装饰数 | 有 | sunshield{none/short_lip/long_hood}（宿主体顶派生，随 ③⑤ 共形）+ IR 环装饰数{4,8,20}；record_only + forked_anchor |
| ⑤ 尺寸/行程 | 只连续改尺寸/行程 | 有 | body_len/radius_scale、mount_scale（§7）；tilt 关节包络：轴+Y，[闭合 0, 可行下界≈−1.0 / 上界≈0.7]，solver 收缩；pan：轴=法向，range solver 求解；motion_test_plan：`fail_if_parts_overlap_in_sampled_poses(64)` + targeted `ctx.pose` 证 pan/tilt 使 off-axis LED 位移>0.01 + solved-bounded clearance 复核；全程不穿模 |
| ⑥ 涂装 | 只改材质/颜色 | 有 | 材质大类 painted-metal/plastic + glass（lens）+ led；配色 4：silver/white/gray/black（覆盖 ≥ ceil(0.5×4)=2 ✓）|

**收尾自检**：0-9 seed batch 目检——四种外壳形态拉得开、四种配色都现、sunshield 贴体不悬空、
pan/tilt 开合全程不穿模。

## 拓扑多样性审计

总组合数：mount(4) × gimbal(2) × housing(4) × sunshield(3) × palette(4) × ir(3) × lens(2)
= 4×2×4×3×4×3×2 ≈ 4608（去掉 gate 后仍数千），1000-seed slot choice tuple distinct 按 ≥300 富类别建议线观察；3 根连续 scale 只补视觉/比例多样性。

理由：7 根 slot 轴每根 ≥2 candidate，48-seed sweep 已实现每根全值覆盖（见下）。

seed_domain_policy：procedural_first（seed=0 不特殊）。
Procedural Sampling / Sweep Plan：`config_from_seed` 先按 mount 采样 → 从该 mount 合法 housing 集采样
（compatibility gate 内建，`_LEGAL_HOUSINGS`）→ gimbal（dome 强制 full_pan_tilt）→ sunshield（仅 bullet/box）
→ ir/lens 加权 → palette → 3 连续 scale。`resolve_config` 用 `_coerce_legal` 把任意（corner）组合投影回合法域，
故 corner seed 始终合法。无 regression override。random sweep 0-35 + corner（per-field 极值 + 未实现组合）。
Topology target：富类别建议 ≥300（report-only）（组合空间数千）。

Controlled local parameterization：body_len_scale/body_radius_scale/mount_scale（§7，independent，clamp）；
joint range 为 inequality（solver 求解，不破坏接口/clearance/joint origin/multiplicity）。

| item | policy | validator / viewer focus |
|---|---|---|
| sampler | mount→housing(合法集)→gimbal→sunshield→ir→lens→palette→scale；加权 | slot_choices_for_seed 与 build 一致 |
| compatibility matrix | 见下矩阵；非法组合由 `_LEGAL_HOUSINGS` + `_coerce_legal` gate | no floating/collision/axis/multiplicity 失败 |
| controlled local variation | 3 连续 scale + solver joint range，全 clamp/求解 | 比例变化不破接口/clearance/身份 |
| regression overrides | none | — |
| random sweep | seeds 0-35 初过 + corner；0-999 成熟审计 | contract failures；axis_realization |

**兼容矩阵（①×③；COPY 自 source map gates）**

| mount×housing | bullet | speed_dome | turret_eyeball | box |
|---|---|---|---|---|
| desk | ✓源S1 | gate | gate | ✓外推 |
| wall_arm | ✓源S2 | ✓fork | ✓外推 | ✓外推 |
| wall_direct | ✓源S3 | gate | ✓外推 | ✓外推 |
| ceiling | ✓fork | ✓源S4 | ✓外推 | gate |

其他 gate：dome/turret 不佩 sunshield；tilt_only 不配 dome（无 pan 即废 → dome 强制 full_pan_tilt）。

| slot | candidate_count | 是否 ≥2 | 是否 ≥3 | 备注 |
|---|---:|---|---|---|
| A mount_form | 4 | yes | yes | |
| B gimbal_dof | 2 | yes | no | 骨架图轴，2 态即结构差异 |
| C housing_form | 4 | yes | yes | ③ Primary Form Family |
| D sunshield | 3 | yes | yes | 仅 bullet/box |

## Validator

- slot_choices_for_seed 返回已实现 module 名（含 ir/lens multiplicity 档）
- config_from_seed 对全部 seed（含 0）用 deterministic procedural sampling
- `_LEGAL_HOUSINGS` + `_coerce_legal` 阻止非法 mount×housing / tilt_only×dome / dome-turret×sunshield
- 无 regression override；主 seed domain 非 curated 表
- 连续 scale 全 clamp；joint range 由 `_solve_gimbal_limits` 求解，不破接口/clearance/joint origin
- 关键接口存在：pan 经 `pan_knuckle`（轴对称）、tilt 经 `tilt_pin`+`yoke_cheek`（captured）
- 关节 type/axis/range：pan/tilt 均 REVOLUTE，pan 轴=mount 法向，tilt 轴 +Y
- 复制件命名/放置：`ir_led_i`（环）/`lens_barrel_j`,`front_glass_j`（y 偏移）loop 发射

## Reject cases

- pan 轴不过 pan 轴承 / tilt 轴不过横销（origin-honesty 失败）
- 外壳几何伸到 pivot 之后（camera-x<0）撞进 yoke → 静态穿模
- sunshield long_hood 后端探到 pivot 之后 → tilt 收缩为 0
- pan 先于 tilt 求解（用未 clamp 的深俯仰）→ pan 假性坍缩为 0
- 把 IR/lens/sunshield 建成 FIXED 关节独立 part（应为 parent visual，Rule 1）
- dome/turret 用 roll-about-boresight 当 pan（球体旋转不可见 → 假 DOF）
- 非法组合（desk×dome、ceiling×box、tilt_only×dome）未 gate

## 与相邻类别的边界

- 不该混入：webcam（无铰接安装座/云台，主体是小夹持成像模块，语义是"桌面/屏幕视频输入"而非"安防定点监控"）
- 不该混入：floodlight 安防灯（有云台臂但主体为 LED 灯板、无镜头/成像头；本类必须有带镜头的 camera_head）
- 不该混入：PTZ 空支架 / 云台头（无 camera_head 成像件）

## 审核记录
| 项 | 结论 |
|---|---|
| reviewer status | pending |
| reviewer notes | sweep-pipeline verdict=pass（0-35 pass_rate 1.0 + corner clean，48 seed distinct=48，逐 slot 全值覆盖）；batch 目检待人工 |

## Module Source Index
| source_id | slot | module | sample_id | model.py 来源 | 采纳用途 |
|---|---|---|---|---|---|
| S1 | A/C/D/mult | desk / bullet / short_lip / IR N=20 | `rec_a-silver-bullet-cctv-security-camera-a-short-cyl_20260624_124946_096010_1da3777f` | base L38-59, stand-yoke L63-111, bullet L117-141, IR loop L173-189, sun-lip L211-216, joints L248-265 | mount+yoke+bullet+IR环+short_lip+pan/tilt |
| S2 | A/B | wall_arm / tilt_only | `rec_a-white-bullet-security-camera-a-cylindrical-bod_20260624_125234_670793_03a3be03` | wall_bracket L31-92, camera L96-176, LED N=4 L157-169, tilt joint L178-186 | wall_arm mount + tilt_only 单铰 + IR N=4 |
| S3 | A/D | wall_direct / long_hood | `rec_gray-cctv-bullet-security-camera-with-a-cylindri_20260624_134448_028125_4e6dbba3` | wall_plate L99-148, pan_knuckle L152-176, camera L180-250, hood L33-60, joints L252-269 | wall_direct mount + long_hood + pan(X)/tilt |
| S4 | A/C/mult | ceiling / speed_dome / IR N=8 | `rec_a-white-ptz-speed-dome-security-camera-a-truncat_20260624_125044_036850_1892c604` | housing L58-90, pan_carriage L92-122, camera_ball L124-197, IR loop L174-197, joints L199-216 | ceiling mount + speed_dome + IR N=8 + pan(Z)/tilt |
| F1 | C | box_housing@S3 | `rec_security_camera_var_box_housing` | camera_body Box L176-246 | box 外壳形态 |
| F2 | C | turret_eyeball@S4 | `rec_security_camera_var_turret_eyeball` | housing/eyeball L35-163 | turret_eyeball 外壳形态 |
| F3 | mult | dual_lens@S1 | `rec_security_camera_var_dual_lens` | _add_lens_module L147-198 | lens N=2 |
| F4 | A×② | wall_arm_pan@S2 | `rec_security_camera_var_wall_arm_pan` | pan_collar L61-112 | wall_arm + full_pan_tilt |
| F5 | A×③ | ceiling_bullet@S3 | `rec_security_camera_var_ceiling_bullet` | ceiling_plate/stem L101-180 | ceiling + bullet |
| F6 | A×③ | wall_arm_dome@S4 | `rec_security_camera_var_wall_arm_dome` | wall-arm+cap L58-109 | wall_arm + speed_dome |
