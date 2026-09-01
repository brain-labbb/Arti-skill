# military_knife (tactical / combat folding-deploy knife) — Modular Spec

> 来源小类：`picture/Military/Military knife`（articraft_data 上游小类样本池）。
> 上游 source map：`picture_expansion/template_source_maps/Military__Military_Handtools_Knife.md`。
> **"Military knife" 在此 = 战术 / 军用刀，核心身份是"刀身可受控展开 / 收纳的机构刀"**（OTF 前向滑出 / 侧开折刀 / 滑鞘罩护），不是纯固定刀（fixed blade，零关节，出类）、不是蝴蝶刀 balisong（双柄双枢轴，拓扑超本模板）、也不是 bayonet 刺刀挂枪卡座 / sword 剑 / machete 砍刀。
> 结构家族 = 机构战术刀：一只 `handle`（root，金属 shell 前体 + tan grip 后块 + 橙色 accent + 脊背锯齿轴）+ 一个**展开机构**（OTF 滑出 blade / 侧开 pivot blade / 滑动 sheath 罩固定刀）；机构内**至少一个非固定关节**（PRISMATIC 或 REVOLUTE）。
>
> **同步状态**：本 spec 引用的 10 个 5 星样本（1 个 parent OTF + 9 个 fork 槽位变体）**已同步进本仓库 `data/records/<id>/`，rating=5**（逐一核对 `record.json` rating=5）。行号按各样本本仓库 `revisions/rev_000001/model.py` 实际行号计（已逐一逐行核对）。引用以 part / joint / helper **名字** 为准（`handle`/`blade`/`thumb_slider`/`sheath` part；`handle_to_blade`/`handle_to_thumb_slider`/`handle_to_sheath` joint；`_build_shell_shape`/`_build_grip_shape`/`_channel_cut`/`_build_blade_shape`/`_build_slider_shape`/`_build_guard_shape`/`_build_sheath_shape`/`_build_glass_breaker`/`_build_pocket_clip` helper），行号仅作定位。
>
> **坐标约定（统一全候选）**：世界轴一致 —— **+X = 刀尖 / 展开方向**（handle 前端出刀）；handle 跨 x[0,0.14]、y±0.015、z[0,0.02]，**坐地 z=0**；脊背在 **+Y**（锯齿轴），主切刃在 **−Y**；blade 厚 ~0.003m。所有 10 个样本共用此约定，模板无需 rebase。

## 元信息
| 项 | 值 |
|---|---|
| slug | `military_knife` |
| template path | `agent/templates/Military_Military_knife.py` |
| test path (optional) | `tests/agent/test_military_knife_template.py`（不写，sweep 为唯一验收）|
| stage | `SPEC_ONLY_DRAFT` |
| status | `pending` |
| __modular__ | `True` |
| pattern | `mixed`（固定 root `handle` + 互斥的展开机构 slot（决定 part 树 / 主关节 spine）+ blade-profile mesh 维度 + pommel/clip handle-visual 维度 + 脊背锯齿 multiplicity 轴）|

## 5 星样本阅读摘要
| 项 | 值 |
|---|---|
| five_star_total | 10（1 parent OTF + 9 fork 槽位变体；均 converged、compile success、≥1 非 fixed joint、rating=5）|
| read_count | 10（**全部读完整 `model.py`**，不抽样；含每个样本的 build helpers、part 树、articulation 与 run_tests 段）|
| read_scope | all 5-star samples in this category |
| source_index_policy | 仅被采纳为 module source 的样本进入下方 source 表与 §14；本批 10/10 全部被采纳，无未采用样本 |

阅读要点（用于槽位分解）：
- **共享基线拓扑（全部 10 个样本）**：`handle`（root，坐地 z=0；visual：`metal_body` 倒角金属 shell + `grip_block` tan 后块 + `grip_ridge_{i}`×4 防滑棱 + `accent_top_{j}`/`accent_{side}_{j}` 橙色 accent）。`_chamfered_bar`、`_build_shell_shape`、`_build_grip_shape`、`_channel_cut` 是共享 helper。所有样本 handle 尺寸 ~0.14×0.03×0.02m、橙 accent 在前部（x>0.10）、tan grip 在后。
- **Slot A 展开机构轴（决定 part 树 + 主关节 spine + root 坐标语义）** —— 真正的拓扑变化轴：
  - **otf_prismatic_slide**（parent）：3 part（`handle`/`blade`/`thumb_slider`），2 个 PRISMATIC joint（`handle_to_blade` +X 0→0.115、`handle_to_thumb_slider` +X 0→0.04 **Mimic** of blade）；blade 藏于 `_channel_cut` 中空 channel，沿前槽滑出；thumb slider 在 recessed top track 同轴随动（reduced mimic ratio）。
  - **side_folding_pivot**（foldpivot）：4 part（`handle`/`blade`/`secondary_blade_1`/`secondary_blade_2`），3 个 REVOLUTE joint（各 blade 绕近前端共享 PIVOT_X≈0.133 的 Z 轴 0→π）；三把刀片在柄宽/厚度方向错层收纳，主刀较短以保证合拢时不穿出柄尾/底边，两个副刀更短更窄也可独立转出；handle 加 `pivot_pin`（Cylinder）visual、主 blade 加 `thumb_stud` visual。
  - **sliding_sheath_fixedblade**（sheathslide）：2 part（`handle`/`sheath`），1 个 PRISMATIC joint（`handle_to_sheath` +X 0→0.05）；**刀身是 handle 上的 `tanto_blade` fixed visual（非独立 part）**，全龙骨固定刀身；管状 `sheath_tube` collar 沿刀轴前滑罩住刀尖；handle 加 `guard` 板（宽于鞘作后退止挡）visual。**靠外加滑鞘 prismatic 关节才纳入 articulated 类目**（纯固定刀本身出类）。
- **Slot B blade-profile 轴（刀刃轮廓 mesh，不改 part/joint 拓扑）** —— `_build_blade_shape` 的 2D 轮廓变化：
  - **tanto**（parent）：直主刃 + 角折 tanto 次刃斜上到尖、clipped tip 折回脊；纯 polyline 折线（5 点）。
  - **drop_point**（droppoint）：凸脊平滑下弯 + 缓上翘 belly 在居中点相交（无角折）；用 `spline` 双曲线收敛到尖。
  - **dagger**（dagger）：对称双刃匕首，y=0 两侧均切刃收敛到居中矛尖；顶面中央 `top_ridge` union spine 凸条；无锯齿（对称轮廓，锯齿轴需 gate 关闭，见 §兼容矩阵）。
  - **clip_point**（clippoint）：直主刃近尖微上翘、脊直行 ~2/3 后凹弧 clip 下削到细前尖；`n_clip=8` 折线近似凹弧。
- **Slot D pommel/clip 轴（尾端附件，全是 `handle` 的 parent visual，无独立 part / 无 joint）**：
  - **plain**（parent）：仅 tan grip block + `grip_ridge_{i}` 防滑棱 + 前部橙 accent；无尾椎 / 夹。
  - **glassbreak**（glassbreak）：`_build_glass_breaker` → `glass_breaker_spike` handle visual（`makeCone` 短锥破窗椎 base_r0.004→tip0.0004，旋 −90° 指 −X，藏 grip 后端）+ mounting collar 圈。
  - **pocketclip**（pocketclip）：`_build_pocket_clip` → `pocket_clip` handle visual（+Y 侧面薄簧片：anchor + bend + arm + return lip，以 `CLIP_STANDOFF`=0.002 离面立起，向前延伸）。
- **Multiplicity 轴（脊背锯齿数）**：`_build_blade_shape` 内 `for i in range(N)` 沿刀脊 +Y 等距切矩形 box 缺口（`blade.cut(notch)`，纯几何减材，**不引入关节、不计 part**）。N 已覆盖 {3, 6, 10}（serr3 / parent / serr10）；serr10 提为 `SERRATION_COUNT`/`SERRATION_X0`/`SERRATION_SPACING` 常量。grip-ridge 防滑棱（固定 N=4）**不另设 multiplicity 轴**（与锯齿轴重复，折叠进锯齿轴）。

## 核心身份

一只**战术 / 军用机构刀**：一只金属 + tan grip 的 `handle`（root，坐地于侧躺姿态；前体 gunmetal 倒角 shell、后块 tan grip + 防滑棱、前部橙色 accent 条），配一个**刀身展开 / 收纳机构**（三选一互斥）：(a) **OTF 前向滑出** —— blade 沿前槽 +X PRISMATIC 滑出 0→0.115m、thumb slider mimic 随动；(b) **侧开折刀** —— blade 绕近前端 Z 轴 REVOLUTE 翻 0→π，合藏入 channel ↔ 开与柄共线；(c) **滑鞘固定刀** —— 全龙骨固定刀身（handle 的 fixed visual）+ 管状 sheath collar 沿刀轴 +X PRISMATIC 0→0.05m 罩护刀尖。刀刃轮廓四选一（tanto / drop_point / dagger / clip_point），尾端附件三选一（plain / glassbreak / pocketclip handle visual），脊背锯齿数可变（N∈[2,20]）。默认成熟域：deployment(3) × blade_profile(4) × pommel_clip(3) × 锯齿 N 的小型战术刀。

**身份硬约束**：展开机构内**至少一个非固定关节**（PRISMATIC 或 REVOLUTE）—— 这是本类区别于纯固定刀的核心。sliding_sheath 的刀身虽固定，但靠 sheath PRISMATIC 纳入；纯零关节固定刀**出类**（reject）。

不该混入：
- **纯固定刀（fixed-blade，零关节）**——无任何活动结构，不满足 articulated 类目要求（sliding_sheath 通过外加滑鞘 prismatic 才纳入；不得退化成无 joint 的纯固定刀）。
- **蝴蝶刀（balisong）**——双柄绕双枢轴翻转、刃居中夹持，拓扑超出本 named-slot 模板（双 REVOLUTE + 中夹刃，root 坐标不同）。
- **弹簧辅助 / 弹簧自动（spring-assist / automatic）**——需弹簧蓄能-触发联动，无法用纯静态 PRISMATIC/REVOLUTE 表达。
- **刺刀（bayonet）/ 剑（sword）/ 砍刀（machete）/ 多功能折叠工具（multitool）**——bayonet 带挂枪卡座、sword/machete 是长固定刃、multitool 是多工具绕同 pivot 阵列展开，结构家族不同。

## 槽位 + 候选模块表

> **建模注记**：`deployment`（Slot A）是改 part 数 / joint 拓扑 / root 运动 spine 的主轴（OTF=3part/2PRISMATIC、folding=2part/1REVOLUTE、sheath=2part/1PRISMATIC+固定刀 visual）。`blade_profile`（Slot B）是 `_build_blade_shape` mesh-profile 维度，**不改 part/joint 拓扑**，列为候选轴以对齐 schema、与 deployment 笛卡尔积撑开多样性（见 §9）。`pommel_clip`（Slot D）全是 `handle` 的 parent visual（无 part / 无 joint），亦 mesh/装饰维度。`blade_profile` 与 `pommel_clip` 都正交于 deployment（除 dagger×sheath 的锯齿 gate，见兼容矩阵）。

### Slot A：deployment（刀身展开 / 收纳机构 —— 决定 part 树、主关节 spine 与 root 运动语义）

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 关键 joint / 结构特征 |
|---|---|---|---|---|
| otf_prismatic_slide（基线）| rec_model-a-futuristic-military-otf-out-the-front-kn_...fae188ee（parent）| `_build_shell_shape`（含 channel + track cut）L86-95 / `_channel_cut` L70-83 / `_build_slider_shape` L131-155 / `blade` part L209-214 / `thumb_slider` part L217-222 / `handle_to_blade` **PRISMATIC** +X L225-233 / `handle_to_thumb_slider` **PRISMATIC** +X **Mimic** L236-245 | eligible if compatible | OTF 前向滑出：**3 part**（handle/blade/thumb_slider），2 个 PRISMATIC（blade 0→0.115、slider 0→0.04 mimic ratio=SLIDER_TRAVEL/BLADE_TRAVEL）；blade 藏中空 channel 沿前槽滑出，slider 在 recessed track 同轴随动；rest 刀全藏（`expect_within`）、deploy 全长 ~0.26m |
| side_folding_pivot | rec_military_knife_var_foldpivot | `_build_shell_shape`（含 channel + groove）L83-96 / `_build_blade_shape`（pivot at origin，q=0 沿 −X，q=π 沿 +X）L107-136 / `pivot_pin` visual L191-197 / `blade` part + `thumb_stud` visual L200-215 / `handle_to_blade` **REVOLUTE** 绕 Z 0→π L217-228 | eligible if compatible | 侧开多刀折叠：**4 part**（handle + 主刀 + 2 把副刀），3 个 REVOLUTE（共享前端 pivot pin，柄宽/厚度方向 y/z-offset 错层；lower=0/upper=π）；主刀缩短并与副刀一起折藏入 side pocket，q=π 时各自可独立转出；handle 加 `pivot_pin`、主 blade 加 `thumb_stud` |
| sliding_sheath_fixedblade | rec_military_knife_var_sheathslide | `_build_shell_shape`（实心无 channel）L70-74 / `_build_blade_shape`（世界坐标固定刀，extrude 后 `tanto_blade` handle visual）L84-110 / `_build_guard_shape` L113-124 / `_build_sheath_shape`（矩管 collar + tab + ridge）L127-168 / `guard` visual L202-206 / `sheath` part L235-240 / `handle_to_sheath` **PRISMATIC** +X L248-258 | eligible if compatible | 滑鞘固定刀：**2 part**（handle/sheath），1 个 PRISMATIC（sheath 0→0.05）；**刀身是 handle 的 `tanto_blade` fixed visual**（全龙骨，非独立 part）；管状 sheath collar 沿刀轴前滑罩刀尖，`guard` 板宽于鞘作后退止挡；run_tests 断言无独立 blade/thumb_slider part（sheathslide L330-336）|

### Slot B：blade-profile（刀刃轮廓 —— `_build_blade_shape` 2D mesh 维度，不改 part/joint 拓扑）

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| tanto（基线）| rec_model-a-futuristic-military-otf-out-the-front-kn_...fae188ee（parent）| `_build_blade_shape` polyline 5 点（直主刃 + 角折 tanto 次刃 + clipped tip）L104-128 | eligible if compatible | 直主刃 + 角折式 tanto 次刃斜上到尖、clipped tip 折回脊；纯 polyline 折线轮廓；脊背可锯齿（非对称） |
| drop_point | rec_military_knife_var_droppoint | `_build_blade_shape` 用 `spline` 双曲线（凸脊下弯 + 缓 belly 上翘，居中点相交）L104-144 | eligible if compatible | 凸脊平滑下弯与缓上翘 belly 在居中点相交（无角折）；用 spline 双曲线收敛到尖；straight 段（rear，x<−0.048）保留锯齿 |
| dagger | rec_military_knife_var_dagger | `_build_blade_shape` 对称 polyline + `top_ridge` union（中央 spine 凸条）L107-139 | eligible if compatible | 对称双刃匕首，y=0 两侧均切刃收敛到居中矛尖；顶面中央 spine ridge 凸条；**对称轮廓两侧皆刃 → 无脊背锯齿轴**（锯齿 gate 关闭，见兼容矩阵）|
| clip_point | rec_military_knife_var_clippoint | `_build_blade_shape`（直主刃微上翘 + `n_clip=8` 折线凹弧 clip）L104-154 | eligible if compatible | 直主刃近尖微上翘、脊直行 ~2/3 后凹弧 clip 下削到细前尖；n_clip=8 折线近似凹弧；straight 段（rear）保留锯齿 |

### Slot D：pommel/clip（尾端附件 —— 全是 `handle` 的 parent visual，无独立 part / 无 joint）

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| plain（基线）| rec_model-a-futuristic-military-otf-out-the-front-kn_...fae188ee（parent）| `grip_block` visual L177-180 / `grip_ridge_{i}`×4 L182-189 / `accent_top_{j}`+`accent_{side}_{j}` L191-206 | eligible if compatible | 朴素尾端：仅 tan grip block + 防滑 grip ridge + 前部橙 accent；无尾椎 / 夹（pommel_clip 不加额外 visual）|
| glassbreak | rec_military_knife_var_glassbreak | `_build_glass_breaker`（makeCone 短锥 + collar 圈）L131-157 / `glass_breaker_spike` handle visual L239-243 | eligible if compatible | grip 后端 makeCone 短锥破窗椎（base_r0.004→tip0.0004，旋 −90° 指 −X）+ mounting collar 圈；作 handle visual（material `tungsten`），无独立 part / joint |
| pocketclip | rec_military_knife_var_pocketclip | `_build_pocket_clip`（anchor+bend+arm+lip）L142-195 / clip 尺寸常量 L60-69 / `pocket_clip` handle visual L277-279 | eligible if compatible | +Y 侧面薄簧片口袋夹：后端 anchor 平板 + bend 过渡 + arm 簧臂（离面 CLIP_STANDOFF=0.002 立起、向前延伸）+ 自由端 return lip；作 handle visual，无独立 part / joint |

> 候选数：deployment(3) / blade_profile(4) / pommel_clip(3)。无单 candidate slot（最低 3）。multiplicity 锯齿轴是第 4 个多样性轴（见 §8）。

## 槽位图（slot graph）

pattern: mixed（固定 root `handle`；deployment 决定挂 handle 的活动件 part 树 + 主关节；blade_profile 只换 blade mesh；pommel_clip 只加 handle visual；锯齿轴只在 blade mesh 上 cut）

```
handle (root, 坐地 z=0; 共享 visual: metal_body shell + grip_block + grip_ridge_{i}×4 + accent_{top/side}_{j})
  │   handle shell 是否含 _channel_cut 由 deployment 决定（OTF/folding 有 channel；sheath 实心）
  │
  ├── [deployment slot]  (三选一，互斥；决定主关节 spine)
  │     ├─ otf_prismatic_slide :
  │     │     ├─ blade (独立 part) ──[handle_to_blade: PRISMATIC axis=(1,0,0), origin=(SHELL_X1,0,CHANNEL_Z0), 0→0.115]
  │     │     │      （blade 藏中空 channel；rest 全藏 expect_within；deploy 保留插入重叠 expect_overlap）
  │     │     │      blade visual = _build_blade_shape (由 blade_profile 决定; 脊背锯齿 由 N cut)
  │     │     └─ thumb_slider (独立 part) ──[handle_to_thumb_slider: PRISMATIC axis=(1,0,0), origin=(0.050,0,TRACK_FLOOR_Z), 0→0.04, Mimic(handle_to_blade, ratio=0.04/0.115)]
  │     │
  │     ├─ side_folding_pivot :
  │     │     └─ blade (独立 part) ──[handle_to_blade: REVOLUTE axis=(0,0,1), origin=(PIVOT_X≈0.133,0,HANDLE_T/2), 0→π]
  │     │            （blade local frame: pivot at origin; q=0 折藏 channel, q=π 开与柄共线）
  │     │            handle 加 pivot_pin Cylinder visual; blade 加 thumb_stud Cylinder visual
  │     │            blade visual = _build_blade_shape (pivot-at-origin 版; 脊背锯齿 由 N cut)
  │     │
  │     └─ sliding_sheath_fixedblade :
  │           ├─ (刀身 = handle 的 tanto_blade FIXED visual, 非 part; _build_blade_shape 世界坐标版; 脊背锯齿 由 N cut)
  │           ├─ (guard = handle 的 guard FIXED visual, 宽于鞘作止挡)
  │           └─ sheath (独立 part) ──[handle_to_sheath: PRISMATIC axis=(1,0,0), origin=(guard_front_x,0,BLADE_CENTER_Z), 0→0.05]
  │                  （矩管 collar 沿刀轴前滑罩刀尖；rest 退缩近 handle、deploy 罩过刀尖）
  │
  ├── [blade_profile slot]  (四选一，只换 blade mesh 轮廓; dagger 关锯齿)
  │     tanto / drop_point / dagger / clip_point  → 决定 _build_blade_shape 的 2D 轮廓
  │
  └── [pommel_clip slot]  (三选一，只加 handle parent visual; 无 joint)
        plain (无额外 visual) / glassbreak (glass_breaker_spike) / pocketclip (pocket_clip)
```

接口点位与 joint 语义：
- **handle → blade（deployment=otf_prismatic_slide）**：mating = 前槽 channel floor。PRISMATIC axis=(1,0,0)，origin=(SHELL_X1=0.14, 0, CHANNEL_Z0=0.0085)（parent L230），lower=0/upper=BLADE_TRAVEL=0.115；blade local x=0 在前槽面（joint frame），body 沿 −X 藏 channel；rest 时 blade 全藏 handle（`expect_within` blade⊂handle，parent L319-325），blade 坐 channel floor（`expect_contact` parent L327-332）；deploy q=upper 时 blade 保留 ≥0.008 插入重叠（`expect_overlap` parent L373-379）。
- **handle → thumb_slider（仅 otf_prismatic_slide）**：mating = recessed top track floor。PRISMATIC axis=(1,0,0)，origin=(0.050,0,TRACK_FLOOR_Z=0.017)（parent L241），lower=0/upper=SLIDER_TRAVEL=0.04，**Mimic(joint=handle_to_blade, multiplier=0.04/0.115)**（parent L244）；slider 坐 track floor（`expect_contact`）、button 凸出 top face（parent L356-361）。
- **handle → blade（deployment=side_folding_pivot）**：mating = 近前端 Z 枢轴。REVOLUTE axis=(0,0,1)，origin=(PIVOT_X≈0.133, 0, HANDLE_T/2=0.01)（foldpivot L223），lower=0/upper=π；blade local frame pivot 在 origin（blade body 含 (0,0,0)，满足 chain-joint child-frame-origin 约束）；handle 加 `pivot_pin` Cylinder（穿 handle 厚度，foldpivot L191-197）、blade 加 `thumb_stud` Cylinder（spine 近 pivot，foldpivot L207-215）。closed q=0 blade 藏 handle XY/Z 包络（`expect_within` foldpivot L293-306）；open q=π 全长 ~0.25-0.27m、保留插入重叠（foldpivot L308-323）。
- **handle → sheath（deployment=sliding_sheath_fixedblade）**：mating = guard 前面、blade 中高。PRISMATIC axis=(1,0,0)，origin=(guard_front_x=HANDLE_LEN+GUARD_THICK, 0, BLADE_CENTER_Z=0.01)（sheathslide L253），lower=0/upper=SHEATH_TRAVEL=0.05；sheath 矩管 collar bore 套住固定刀（blade ⊂ sheath bore on y/z，`expect_within` sheathslide L373-381）；rest q=0 退缩近 handle（刀尖露），deploy q=upper 罩过刀尖（sheathslide L351-361）。**刀身是 handle 的 `tanto_blade` fixed visual**（非 part，无 joint），靠 sheath PRISMATIC 满足 articulated。
- **blade mesh 接口**：blade 的 `_build_blade_shape` 由 blade_profile 决定轮廓、由锯齿 N cut 缺口。OTF/folding 的 blade 是独立 part 的 visual；sheath 的刀身是 handle 的 visual（坐标系不同：OTF blade local x=0 在前槽、folding blade pivot at origin、sheath blade 世界坐标固定）。模板按 deployment 选 blade 坐标系版本。
- **pommel_clip 接口**：glass_breaker_spike / pocket_clip 是 `handle` 的 parent visual（FIXED 语义，captured 嵌入 grip/side，element-scoped `allow_overlap(grip_block, glass_breaker_spike)` / `(metal_body, pocket_clip)` 守捕入过盈），无独立 part / 无 joint。
- **mating policy**：blade-in-channel（OTF）、blade-pivot-pin（folding）、sheath-over-blade、pommel-captured 等接口多为滑配 / 捕入 / 套筒，**非两轴对齐面硬对接** → 省略 MatingContract（grandfather），由 `fail_if_articulation_origin_far_from_geometry`（0.015）守 origin + element-scoped `allow_overlap` 守 captured/sliding overlap（照搬各样本 run_tests 段）。
- **rest pose**：OTF blade q=0（全藏）；folding blade q=0（折藏）；sheath q=0（退缩、刀露）—— 全部"收纳 / 待用"姿态，viewer 目检以此为准。
- **互斥 / 可选 / 派生**：deployment 三候选互斥（决定 part 数 / 关节类型 / handle shell 是否 channel）；blade_profile 四候选互斥；pommel_clip 三候选互斥。`handle_to_thumb_slider` + thumb_slider part **仅 otf_prismatic_slide 有**；`pivot_pin`/`thumb_stud` visual 仅 folding 有；`guard`/`sheath`/固定刀 visual 仅 sheath 有。脊背锯齿轴在 blade mesh 上 cut，**dagger 关闭**（对称轮廓两侧皆刃，无单脊背可锯齿）。

## 每槽位 Module Emits / Interfaces

### root / handle（共享基线，全候选共有）
| emits | 描述 | 来源 |
|---|---|---|
| parts | `handle`（root，坐地；visual：`metal_body` 倒角 shell + `grip_block` tan 后块 + `grip_ridge_{i}`×4 防滑棱 + `accent_top_{j}`/`accent_{side}_{j}` 橙 accent）| parent L170-206 |
| internal joints | 无（handle 是 root；deployment 的活动件挂在 handle 上）| — |
| upstream interface | root（坐地，无父）| — |
| downstream interface | 前槽 channel（OTF/folding：blade 滑出 / 枢轴）/ 前面 + guard（sheath：固定刀 + sheath 滑轨）/ 顶 track（OTF：slider）| parent `_channel_cut` L70-83 / sheathslide L113-124 |

### Slot A / deployment — otf_prismatic_slide
| emits | 描述 | 来源 |
|---|---|---|
| parts | `blade`（visual `tanto_blade`=`_build_blade_shape`）+ `thumb_slider`（visual `slider_button`）| parent L209-222 |
| internal joints | `handle_to_blade` PRISMATIC axis=(1,0,0) 0→0.115（parent L225-233）+ `handle_to_thumb_slider` PRISMATIC axis=(1,0,0) 0→0.04 Mimic（parent L236-245）| parent L225-245 |
| upstream interface | blade local x=0 在前槽面 = joint origin（含 (0,0,0)）；slider 坐 track floor | parent L104-128, L131-155 |

### Slot A / deployment — side_folding_pivot
| emits | 描述 | 来源 |
|---|---|---|
| parts | `blade`（visual `tanto_blade` pivot-at-origin 版 + `thumb_stud` Cylinder）| foldpivot L200-215 |
| internal joints | `handle_to_blade` REVOLUTE axis=(0,0,1) origin=(PIVOT_X,0,HANDLE_T/2) 0→π | foldpivot L217-228 |
| upstream interface | blade pivot 在 local origin（含 (0,0,0)，满足 chain child-frame origin）；handle 加 `pivot_pin` 落在 origin 真实硬件 | foldpivot L107-136, L191-197 |

### Slot A / deployment — sliding_sheath_fixedblade
| emits | 描述 | 来源 |
|---|---|---|
| parts | `sheath`（visual `sheath_tube` 矩管 collar）；**刀身 `tanto_blade` + `guard` 是 handle 的 fixed visual（非 part）**| sheathslide L196-206, L235-240 |
| internal joints | `handle_to_sheath` PRISMATIC axis=(1,0,0) origin=(guard_front_x,0,BLADE_CENTER_Z) 0→0.05 | sheathslide L248-258 |
| upstream interface | sheath rear-face center = joint origin；guard 前面作 mating 参考；sheath bore 套固定刀（`expect_within` blade⊂bore on y/z）| sheathslide L127-168, L373-381 |

### Slot B / blade_profile（四候选共用接口；只换 `_build_blade_shape` 2D 轮廓）
| emits | 描述 | 来源 |
|---|---|---|
| parts | 无新 part（决定既有 blade visual 的 mesh 轮廓）；tanto/drop_point/clip_point 非对称（+Y 脊可锯齿），dagger 对称（含 `top_ridge` union，关锯齿）| tanto parent L104-128 / droppoint L104-144 / dagger L107-139 / clippoint L104-154 |
| internal joints | 无 | — |
| upstream interface | blade 轮廓必须含 deployment 选定的 joint-frame 参考点（OTF: x=0 前槽；folding: pivot origin；sheath: 世界坐标固定刀）| 各 `_build_blade_shape` |

### Slot D / pommel_clip — plain
| emits | 描述 | 来源 |
|---|---|---|
| parts | 无（不加额外 visual；仅基线 grip_block + ridge + accent）| parent L177-206 |
| internal joints | 无 | — |
| upstream interface | grip 后端裸（无尾端附件）| — |

### Slot D / pommel_clip — glassbreak
| emits | 描述 | 来源 |
|---|---|---|
| parts | 无独立 part（`glass_breaker_spike` 为 `handle` visual）| glassbreak L239-243 |
| internal joints | 无 | — |
| upstream interface | 短锥 base + collar 捕入 grip 后面（`allow_overlap(grip_block, glass_breaker_spike)`）| glassbreak L131-157 |

### Slot D / pommel_clip — pocketclip
| emits | 描述 | 来源 |
|---|---|---|
| parts | 无独立 part（`pocket_clip` 为 `handle` visual）| pocketclip L277-279 |
| internal joints | 无 | — |
| upstream interface | anchor 平板贴 +Y 侧面（`allow_overlap(metal_body/grip_block, pocket_clip)`）；arm 离面 CLIP_STANDOFF 立起 | pocketclip L142-195 |

## 参数范围汇总
| 参数 | 类型 | 取值范围 / 候选值 | 标称默认 | 约束类型 | 约束 / 函数 | 来源 |
|---|---|---|---|---|---|---|
| deployment | enum | otf_prismatic_slide / side_folding_pivot / sliding_sheath_fixedblade | otf_prismatic_slide | choice | sampler 选；决定 part 数 / 主关节 / handle shell 是否 channel | Slot A 表 |
| blade_profile | enum | tanto / drop_point / dagger / clip_point | tanto | choice | sampler 选；只换 blade mesh 轮廓；dagger 关锯齿 | Slot B 表 |
| pommel_clip | enum | plain / glassbreak / pocketclip | plain | choice | sampler 选；只加 handle visual（无 joint）| Slot D 表 |
| serration_count | int | [2, 20]（per-N 加权，见 §8）| 6 | conditional | 脊背锯齿数；**dagger 时强制 0（gate 关闭）**；进 slot_choice 区分拓扑等价类 | parent L120 / serr3 L120 / serr10 L52 |
| palette_style | enum | gunmetal_orange（默认）/ blacked_out / od_green / stonewash / desert_tan / multicam_grip | gunmetal_orange | palette | palette only，**不计入 slot_choice**；每 seed 采一套（见下表）| 各样本材质 |
| blade_len_scale | float | [0.88, 1.12] | 1.0 | independent | 缩放 BLADE_LEN（刀身长）→ 联动 channel 深 / sheath travel / deploy 全长，clamp | parent L44 |
| blade_width_scale | float | [0.90, 1.12] | 1.0 | independent | 缩放 BLADE_Y_HALF（刀宽）→ 须 < CHANNEL_Y_HALF（OTF/folding 藏 channel），clamp | parent L48 / L38 |
| handle_len_scale | float | [0.90, 1.10] | 1.0 | independent | 缩放 HANDLE_LEN（柄长）→ 联动 grip/shell 分界、accent x 位置，clamp | parent L27 |
| handle_thick_scale | float | [0.92, 1.10] | 1.0 | independent | 缩放 HANDLE_T（柄厚）→ 联动 channel z 高、pivot z、sheath bore z，clamp（保坐地 z=0）| parent L29 |
| deploy_travel_scale | float | [0.85, 1.10] | 1.0 | conditional | OTF: 缩放 BLADE_TRAVEL（slider mimic 同步缩）；sheath: 缩放 SHEATH_TRAVEL（保罩过刀尖）；folding: 不适用（行程固定 π）| parent L49 / sheathslide L53 |
| pivot_x_scale | float | [0.95, 1.04] | 1.0 | conditional | 仅 folding 有效；缩放 PIVOT_X（保 pivot 在前端 channel 内、blade 折藏不超 handle 后端）| foldpivot L45 |
| spike_len_scale | float | [0.85, 1.20] | 1.0 | conditional | 仅 glassbreak 有效；缩放破窗椎长（保不超 grip 后端过远 < 0.018）| glassbreak L139 |
| clip_arm_scale | float | [0.88, 1.12] | 1.0 | conditional | 仅 pocketclip 有效；缩放 CLIP_ARM_LEN（保 arm 不超 handle 前端、standoff 不变）| pocketclip L67 |
| (—) | constraint | — | — | inequality | blade 藏 channel：`BLADE_Y_HALF·blade_width_scale ≤ CHANNEL_Y_HALF − 0.0005`（OTF/folding）；违反则缩 blade_width_scale | parent L48, L38 |
| (—) | constraint | — | — | inequality | deploy 保留插入：OTF `BLADE_LEN·blade_len_scale − BLADE_TRAVEL·deploy_travel_scale ≥ 0.008`（≥8mm retained）；违反则缩 travel | parent L373-379 |
| (—) | constraint | — | — | inequality | sheath 罩过刀尖：`SHEATH_TRAVEL·deploy_travel_scale ≥ (blade_tip_x − sheath_rest_front_x)`；违反则增 travel 或缩 blade_len | sheathslide L351-361 |
| (—) | constraint | — | — | inequality | folding 全藏：`BLADE_LEN·blade_len_scale ≤ PIVOT_X·pivot_x_scale + 0.005`（blade 折藏不戳出 handle 后端）；违反则缩 blade_len 或前移 pivot | foldpivot L293-306 |
| (—) | constraint | — | — | conditional | dagger ⇒ serration_count=0（对称双刃无单脊背锯齿）；其余 profile serration_count∈[2,20] | dagger L107-139 |

palette_style 候选（每 seed 采一套，**不计入 slot_choice**，跨 5★ 样本观察的真实材质 + 战术配色外推）：
| palette_style | metal_body shell | blade_steel | grip_block | accent / pommel | 来源样本 |
|---|---|---|---|---|---|
| gunmetal_orange（默认）| gunmetal (0.27,0.29,0.32) | blade_steel (0.74,0.76,0.78) | grip_tan (0.56,0.40,0.24) | accent_orange (0.95,0.45,0.10) | parent / 全样本 |
| blacked_out | gunmetal_dark (0.15,0.16,0.18) | 黑 PVD 刃 (0.18,0.19,0.21) | 黑 G10 (0.10,0.10,0.12) | 暗红 accent (0.45,0.10,0.08) | parent `gunmetal_dark` 外推 |
| od_green | OD 绿 frame (0.30,0.34,0.22) | stonewash 灰刃 (0.55,0.57,0.58) | OD 绿 grip (0.26,0.30,0.18) | 黑 accent (0.12,0.12,0.13) | sheathslide `sheath_olive` 外推 |
| stonewash | 钢灰 (0.58,0.60,0.62) | stonewash (0.62,0.64,0.66) | 灰 micarta (0.45,0.43,0.40) | 钢 accent | blade_steel 族外推 |
| desert_tan | FDE 棕褐 (0.62,0.52,0.36) | stonewash 灰刃 | desert tan grip (0.70,0.58,0.40) | 黑 accent | grip_tan 外推 |
| multicam_grip | gunmetal frame | blade_steel | 多色迷彩 grip (0.42,0.40,0.28) | 黑 accent + tungsten spike | parent + glassbreak `tungsten` 混 |

所有连续 scale 在 `resolve_config` 中 clamp / 派生；每个 build 解析一次。scale 只动安全比例 / 行程 / 尺寸 / clearance，**绝不改变 deployment / blade_profile / pommel_clip 的拓扑或锯齿等价类**。

## Multiplicity / Copy Logic

存在 **1 根可变 multiplicity 轴**：**脊背锯齿数（spine serration notch count）**。

- `count_param`：`serration_count`
- `N_range`（本小类本轴产品域）：**[2, 20]**（测试偏小、产品全程）。样本已覆盖 {3, 6, 10}（serr3 / parent / serr10）。
- sampling domain（权重档）：小 N 高频、大 N 稀有。建议加权草案 —— N∈[3,8] 约 70%（含基线 6）、N∈[2] 与 [9,12] 约 25%、N∈[13,20] 约 5%（细齿 ≤ 真实可切空间）。具体权重以人工审核后取值为准。
- copied object：**刀脊矩形锯齿缺口（box notch）** —— `cq.Workplane.box(SERRATION_W≈0.005, SERRATION_D≈0.006, BLADE_T·4)`（serr3 用更大齿 0.008×0.008、serr10 用 0.005 细齿）。
- naming：**无单独 visual** —— 齿是 `blade.cut(notch)` 对刀身的减材，不计 part / 不计独立 visual（命名归并入 blade 自身）。
- placement：`for i in range(N): cx = SERRATION_X0 + i·SERRATION_SPACING`，沿刀脊 +Y **等距**排列（equidistant pitch；parent SPACING=0.012、serr3=0.030 大齿、serr10=0.010 细齿）；落在脊背 straight 段（drop_point/clip_point 须避开前端曲线 clip 区，仅在 rear straight 段 cut）。
- joint policy：**纯几何 cut，不引入关节**（`blade.cut(notch)`，减材，无 part / 无 joint）。
- source/gating：parent L120（N=6）/ serr3 L120（N=3）/ serr10 L52-54, L127-134（N=10，提为常量）。**gating**：(1) `blade_profile=dagger` ⇒ serration_count=0（对称双刃无单脊背，关锯齿轴）；(2) serration spacing × N 须 ≤ 脊背 straight 段可用长度（drop_point/clip_point straight 段更短，N 上限随之收窄，conditional）。


## 拓扑多样性审计

总组合数：deployment(3) × blade_profile(4) × pommel_clip(3) × serration_N(多档) = 36 × N_buckets。
仅取 serration 离散为 ~6 个采样桶（{2,3,5,6,8,10,15} 之类近似），**36 × ~6 ≈ 200+ distinct module-tuple 组合**（dagger 关锯齿略减，见下）。

仅 deployment(3) × blade_profile(4) × pommel_clip(3) = **36 ≥ 10**（已远超机械门控）；其中 joint 拓扑差异来自 deployment 的三种 spine：{2 PRISMATIC + mimic（OTF，3 part）/ 1 REVOLUTE（folding，2 part）/ 1 PRISMATIC + 固定刀 visual（sheath，2 part）}。

理由：deployment × blade_profile × pommel_clip 单独即 36 distinct，含 3 类真实主关节 spine（OTF 双 PRISMATIC+mimic / folding REVOLUTE / sheath PRISMATIC）与 part 数差异（3 part vs 2 part；blade 是否独立 part）。叠 serration_count 的可变 multiplicity（不同 N = 不同 module 名）后 ~200+ distinct，远超 ≥10。deployment / blade_profile / pommel_clip / serration 的差异天然进 `slot_choices_for_seed` 的 tuple（`("deployment",m)`、`("blade_profile",m)`、`("pommel_clip",m)`、`("serration",f"serr_{N}")`）。

seed_domain_policy：procedural_first（`config_from_seed(seed)` 对所有 seed 用 `random.Random(seed)` 采样；seed=0 不特殊）。

Procedural Sampling / Sweep Plan：sampler 先 `rng.choice` 三个 named slot（deployment / blade_profile / pommel_clip），经兼容矩阵合法化（dagger ⇒ serration=0、drop_point/clip_point 收窄 serration 上限），再对 serration_count 做 per-N 加权抽样（小 N 偏多），再 uniform 各连续 scale + 采 palette_style。无 regression overrides（首版纯 procedural）。random sweep seeds 0-49 初轮 + 0-999 成熟审计；viewer 目检 seeds 0-9（重点看 OTF 滑出 + slider mimic 随动、folding 翻 0→π 折藏↔展开、sheath 罩刀尖、rest 收纳姿态、dagger 对称刃无锯齿）。

Topology target：1000-seed slot choice tuple distinct 预计 ≥300（36 named-slot 组合 × serration 多档），符合建议。serration 可变 multiplicity 提供超线性细分；即使保守只算 36 named 组合 × 6 serration 桶 ≈ 200，按 ≥300 富类别口径观察。若审核要求更高，serration N_range 可向 [2,20] 全程展开。（统一口径：富类别建议 ≥300；低于 300 记录真实组合空间或兼容约束原因；report-only，不设门。）

Controlled local parameterization：见 §参数表的 blade_len_scale / blade_width_scale / handle_len_scale / handle_thick_scale（independent）+ deploy_travel_scale / pivot_x_scale（@folding）/ spike_len_scale（@glassbreak）/ clip_arm_scale（@pocketclip）（conditional）。全部 `resolve_config` clamp + 每 build 统一应用。采样契约：先采 named slot（解析 conditional：deploy_travel 随 deployment、pivot_x 仅 folding、spike_len 仅 glassbreak、clip_arm 仅 pocketclip；dagger ⇒ serration=0）→ 采 serration_count（per-N 加权，受 blade_profile straight 段长 conditional 上限）→ 采 independent blade/handle scale → 派生（channel 深随 blade_len、sheath travel 随 blade_len）→ 用四条 inequality（blade 藏 channel、deploy 保留插入、sheath 罩刀尖、folding 全藏）投影 / 回缩。跨部件依赖（blade 宽 vs channel 宽、travel vs blade 长、pivot vs blade 长）显式落在 §7 inequality，在 `resolve_config` 内求解。这些 scale 不破坏 blade-in-channel / blade-pivot / sheath-over-blade / pommel-captured 接口、PRISMATIC/REVOLUTE joint origin、收纳姿态、锯齿等价类或类别身份。

| item | policy | validator / viewer focus |
|---|---|---|
| sampler | 先 `rng.choice` 三 named slot（deployment/blade_profile/pommel_clip），解析兼容（dagger⇒serration=0、drop/clip 收窄 serration 上限），采 serration_count（per-N 加权），再 uniform 各 conditional/independent scale，采 palette_style | slot_choices_for_seed 含 `("deployment",m)`/`("blade_profile",m)`/`("pommel_clip",m)`/`("serration",serr_N)` 且与 build 一致 |
| compatibility matrix | deployment × blade_profile × pommel_clip **三轴正交合法**（任意组合可装配）。gate：(1) **dagger ⇒ serration_count=0**（对称双刃无单脊背锯齿）；(2) drop_point/clip_point 的 serration 仅在 rear straight 段，N·spacing ≤ straight 段长 → N 上限收窄（conditional clamp）；(3) deploy_travel_scale 解析随 deployment（OTF/sheath 缩 travel、folding 行程固定 π）；(4) pivot_x_scale 仅 folding、spike_len_scale 仅 glassbreak、clip_arm_scale 仅 pocketclip 生效；(5) **任何组合都保证 ≥1 非固定关节**（不退化成纯固定刀）。 | 无 floating / collision / blade 戳穿 channel 壁 / sheath 罩不过刀尖 / folding 折藏戳出后端 / serration 切穿曲线区 / dagger 误开锯齿 / 零关节固定刀 |
| controlled local variation | 4 independent + 4 conditional clamped scale + serration multiplicity，每 build 统一；conditional 随 slot 解析 | 比例变化不破坏 blade-in-channel/blade-pivot/sheath-over-blade/pommel captured、PRISMATIC/REVOLUTE origin、收纳姿态、锯齿等价类、类别身份 |
| regression overrides | none（首版纯 procedural）| — |
| random sweep | seeds 0-49 初轮；0-999 成熟审计 | 逐 deployment 机构 QC（OTF 滑出 + slider mimic / folding 翻转 / sheath 罩护）+ 收纳/展开姿态 |

| slot | candidate_count | 是否 ≥2 | 是否 ≥3 | 备注 |
|---|---:|---|---|---|
| deployment | 3 | yes | yes | otf(3part/2PRISMATIC+mimic) / folding(2part/1REVOLUTE) / sheath(2part/1PRISMATIC+固定刀 visual) |
| blade_profile | 4 | yes | yes | tanto / drop_point / dagger(对称,关锯齿) / clip_point（mesh-profile 维度）|
| pommel_clip | 3 | yes | yes | plain / glassbreak / pocketclip（handle visual 维度，无 joint）|
| serration（multiplicity）| N∈[2,20] | yes | yes | 可变锯齿数轴；不同 N = 不同 module 名（dagger 时关闭）|

## Validator

- `slot_choices_for_seed` 返回已实现的 module 名，含 `("deployment",m)`/`("blade_profile",m)`/`("pommel_clip",m)`/`("serration",serr_N)`
- `config_from_seed` 对所有 seed 用 deterministic procedural sampling（seed=0 不特殊）
- `resolve_config` 把各 scale clamp 到声明范围；deploy_travel/pivot_x/spike_len/clip_arm 为 conditional 随 deployment/pommel 解析；四条 inequality（blade 藏 channel、deploy 保留插入、sheath 罩刀尖、folding 全藏）在 resolve 内投影 / 回缩
- compatibility matrix 三轴正交合法；**dagger ⇒ serration_count=0**；drop_point/clip_point serration N 上限随 straight 段长收窄；conditional scale 仅在对应 module 生效
- **每个采样组合保证 ≥1 非固定关节**（OTF/folding 的独立 blade joint、sheath 的 sheath joint）；**绝不退化成零关节纯固定刀**
- 连续 scale clamp 后不破坏 blade-in-channel / blade-pivot / sheath-over-blade / pommel-captured 接口、PRISMATIC/REVOLUTE joint origin、收纳姿态、锯齿等价类
- 关键 joint：
  - otf_prismatic_slide：`handle_to_blade` PRISMATIC axis=(1,0,0)（abs(axis[0])>0.99）0→0.115·scale + `handle_to_thumb_slider` PRISMATIC axis=(1,0,0) **Mimic(handle_to_blade)** 0→0.04·scale
  - side_folding_pivot：`handle_to_blade` REVOLUTE axis=(0,0,1)（abs(axis[2])>0.99）0→π；run_tests 断言**恰一个非 fixed joint**
  - sliding_sheath_fixedblade：`handle_to_sheath` PRISMATIC axis=(1,0,0) 0→0.05·scale；刀身是 handle fixed visual（无独立 blade/thumb_slider part）
- captured / sliding 过盈：element-scoped `allow_overlap`（OTF: blade↔handle channel、slider↔track；folding: pivot_pin↔handle、thumb_stud↔blade；sheath: sheath_tube↔fixed blade、guard↔handle；pommel: glass_breaker_spike↔grip_block、pocket_clip↔metal_body/grip_block），照搬各样本 run_tests 段
- 脊背锯齿：`for i in range(N)` 等距 box cut（无 part / 无 joint）；dagger 时 N=0；命名归并 blade
- grip_ridge / accent 固定阵列 visual 遵循 `grip_ridge_{i}`/`accent_{top/side}_{j}` 命名 + 绝对式 placement + Rule 1（无独立 joint，FIXED 装饰）
- grandfather：blade-in-channel / blade-pivot / sheath-over-blade / pommel-captured 接口省略 MatingContract，由 origin 检查（0.015）+ allow_overlap 守
- deployment=sliding_sheath 时断言无 `blade`/`thumb_slider` part（照搬 sheathslide L330-336）；deployment=folding 时断言恰一非 fixed joint（照搬 foldpivot L272-282）

## Reject cases

- 任一采样组合退化成**零关节纯固定刀**（无 PRISMATIC / REVOLUTE）→ 出 articulated 类目，本类核心身份要求 ≥1 非固定展开关节。
- deployment=sliding_sheath_fixedblade 仍把刀身做成独立 `blade` part 加 joint，或 OTF/folding 把刀身做成 handle fixed visual → 违反各 deployment 的 part 树拓扑（sheathslide 刀身是 visual、OTF/folding 刀身是独立 part）。
- deployment=otf_prismatic_slide 缺 `handle_to_thumb_slider` Mimic 或让 slider 独立平移不随 blade → 违反 OTF mimic 联动（thumb slider 必 mimic blade at reduced ratio）。
- blade_profile=dagger 仍切脊背锯齿 → 对称双刃两侧皆刃无单脊背可锯齿，须 serration_count=0。
- 把脊背锯齿缺口当独立活动 part 加 joint → 违反 multiplicity copy 策略（纯 `blade.cut` 减材，无 part / 无 joint）。
- blade rest pose 设成展开（q=upper）而非收纳（q=0）→ current-pose 与 viewer 目检不符（所有样本 rest = OTF 全藏 / folding 折藏 / sheath 退缩刀露）。
- `handle_to_blade`（OTF/folding）/ `handle_to_sheath` origin 放在任意点而非前槽 channel floor / pivot pin / guard 前面真实硬件 → `fail_if_articulation_origin_far_from_geometry`（0.015）FAIL。
- blade 宽超 channel（`BLADE_Y_HALF·scale > CHANNEL_Y_HALF`）致 OTF/folding 收纳穿模 channel 壁 → §7 第一条 inequality FAIL；缩 blade_width。
- OTF deploy 行程过大致 blade 失去插入重叠（< 8mm retained）飞出 → §7 第二条 inequality FAIL；缩 travel。
- folding 刀身过长致折藏戳出 handle 后端 → §7 第四条 inequality FAIL；缩 blade_len 或前移 pivot。
- sheath travel 不足罩不过刀尖 → §7 第三条 inequality FAIL；增 travel 或缩 blade_len。
- 给滑配 / 枢轴 / 套筒接口补 MatingContract 硬对接 → 几何对不上，mating-gap FAIL；应 grandfather + allow_overlap。
- 把连续尺寸 / 颜色 / 材质（palette_style / blade scale）当新 candidate 塞进 slot → 不是结构差异。
- 把蝴蝶刀 / 弹簧自动刀 / 刺刀 / 剑语义混入（双枢轴中夹刃 / 弹簧触发 / 挂枪卡座 / 长固定刃）→ 出类。

## 与相邻类别的边界

- 不该混入：**纯固定刀（fixed-blade knife，零关节）**——无活动展开机构，不满足 articulated 类目；本类 deployment 三候选每个都带 ≥1 非固定关节（sliding_sheath 靠滑鞘 PRISMATIC 纳入）。
- 不该混入：**蝴蝶刀（balisong）**——双柄绕双枢轴翻转、刃居中夹持，双 REVOLUTE + 中夹刃 root 坐标，拓扑超本 named-slot 模板。
- 不该混入：**弹簧辅助 / 自动刀（spring-assist / automatic）**——需弹簧蓄能-触发联动，无法用纯静态 PRISMATIC/REVOLUTE 表达（本类是用户手动 pose 的纯运动学关节）。
- 不该混入：**刺刀 / 剑 / 砍刀 / 多功能折叠工具（bayonet / sword / machete / multitool）**——bayonet 带挂枪卡座、sword/machete 是长固定刃、multitool 是多工具绕同 pivot 阵列展开，结构家族 / 尺度 / root spine 不同。

## 审核记录
| 项 | 结论 |
|---|---|
| reviewer status | pending |
| reviewer notes | （待人工审核：确认 (1) **deployment 三候选 root 坐标 spine 不同**（OTF blade local x=0 在前槽 / folding blade pivot at origin / sheath 刀身世界坐标固定 visual），模板按 deployment 选 blade 坐标系版本 + handle shell 是否 channel；是否接受单模板覆盖三 spine，还是 sheath（固定刀 + 滑鞘）须拆 slug；(2) **dagger ⇒ serration_count=0** 的 gate（对称双刃无单脊背锯齿）是否接受，还是 dagger 直接从 blade_profile 候选移除以简化；(3) serration multiplicity N_range [2,20] + per-N 加权（小 N 偏多）是否合适，权重草案（[3,8]~70% / [2]+[9,12]~25% / [13,20]~5%）待定；(4) drop_point/clip_point 的 serration 仅在 rear straight 段、N 上限随段长收窄，是否需特别 QC；(5) OTF 的 thumb_slider Mimic 联动模板是否实现（源是真实 Mimic at ratio=0.04/0.115）；(6) palette_style 6 套是否合适，blacked_out/od_green/stonewash/desert_tan/multicam_grip 五套为样本配色（gunmetal_dark/sheath_olive/blade_steel/grip_tan/tungsten）外推；(7) pommel_clip 三候选全是 handle visual（无 part/joint），是否接受作为独立 slot 还是折入 handle module-local variant。）|

## 模板实现备注（可选）

- **deployment 坐标分支**：三候选 root 运动 spine 不同，模板按 `deployment` 选：
  - otf_prismatic_slide：handle shell 含 `_channel_cut`（中空前槽）+ recessed top track；blade 是独立 part，local x=0 在前槽面（含 (0,0,0) 满足 chain child-frame origin）；thumb_slider 独立 part + Mimic。
  - side_folding_pivot：handle shell 含 `_channel_cut`（blade 折藏空间）；blade 是独立 part，pivot 在 local origin；handle 加 `pivot_pin`、blade 加 `thumb_stud`。
  - sliding_sheath_fixedblade：handle shell **实心**（`_build_shell_shape` 不 cut channel）；刀身 + guard 是 handle fixed visual（世界坐标）；sheath 独立 part，rear-face center = joint origin。
- **共享 helper**：`_chamfered_bar`（handle bar 倒角）、`_build_shell_shape`（按 deployment 决定是否 channel/track）、`_build_grip_shape`、`_channel_cut`（OTF/folding 共用）、`_build_blade_shape`（按 blade_profile 切轮廓 + 按 deployment 选坐标系版本 + 按 serration_count cut 锯齿）、`_build_slider_shape`（OTF）、`_build_guard_shape`+`_build_sheath_shape`（sheath）、`_build_glass_breaker`/`_build_pocket_clip`（pommel）。
- **blade mesh 三坐标系版本**：OTF 用 parent 的 x=0-在前槽版（body 沿 −X）；folding 用 foldpivot 的 pivot-at-origin 版（含 narrow heel 近 pivot + translate −BLADE_T/2 居中）；sheath 用 sheathslide 的世界坐标固定版（x0=HANDLE_LEN）。同一 blade_profile 轮廓逻辑（tanto/drop/dagger/clip）须能映射到三坐标系（共用 2D 轮廓函数 + 坐标偏移参数）。
- **serration cut**：`_build_blade_shape` 末尾 `for i in range(serration_count): cx=SERRATION_X0+i·SERRATION_SPACING; blade=blade.cut(box_notch)`；dagger 时 skip（serration_count=0）；drop_point/clip_point 时 SERRATION_X0/SPACING 须落 rear straight 段（避开前端曲线 clip 区，参 droppoint L134-143 注释 "before the drop-point curve begins at ~x=-0.048"、clippoint L144-145 "only on the straight portion, rearward of the clip start"）。
- **captured / sliding allow_overlap**：`run_military_knife_tests` 里逐 module 补 element-scoped `allow_overlap`，照搬各样本：OTF blade-in-channel + slider-in-track（parent `expect_within`/`expect_contact` L319-355，captured 处用 allow_overlap）；folding pivot_pin↔handle + thumb_stud↔blade；sheath sheath_tube↔fixed_blade + guard↔handle（sheathslide `expect_within` L373-381）；pommel glass_breaker_spike↔grip_block（glassbreak collar 嵌入）、pocket_clip↔metal_body/grip_block（pocketclip anchor 贴面）。
- **conditional 范围解析顺序**：先采 deployment / blade_profile / pommel_clip → 解析 dagger⇒serration=0、drop/clip serration 上限随 straight 段长、deploy_travel 随 deployment、pivot_x 仅 folding、spike_len 仅 glassbreak、clip_arm 仅 pocketclip → 采 serration_count（per-N 加权）→ 采 independent blade/handle scale → 派生 channel 深 / sheath travel → 投影四条 inequality。
- **mimic 实现**：OTF 的 `handle_to_thumb_slider` 用 `Mimic(joint="handle_to_blade", multiplier=SLIDER_TRAVEL/BLADE_TRAVEL)`（parent L244）；scale 缩 travel 时 mimic ratio 同步重算（slider travel 与 blade travel 等比）。
- **参考模板**：选运动拓扑相近的 —— 单 root + 互斥展开机构 + PRISMATIC/REVOLUTE child 的小尺度模板（如 retractable_utility_knife 的 housing→mechanism→blade 链 + slider PRISMATIC、container_dispenser 的滑/翻盖机构）。military_knife 尺度小（handle ~0.14m、blade ~0.12m、blade 厚 ~0.003m），joint origin 须精确落真实硬件面（≤0.015m baseline）；OTF channel floor / folding pivot pin / sheath guard 前面是三个关键 origin 参考点。

## Module Source Index

| source_id | slot | module | sample_id | model.py 来源 | 采纳用途 |
|---|---|---|---|---|---|
| S1 | A/B/D（parent 基线）| otf_prismatic_slide + tanto + plain | rec_model-a-futuristic-military-otf-out-the-front-kn_...fae188ee | `_build_shell_shape`(channel+track) L86-95 / `_channel_cut` L70-83 / `_build_blade_shape`(tanto) L104-128 / `_build_slider_shape` L131-155 / handle visuals L170-206 / blade+slider part L209-222 / `handle_to_blade` PRISMATIC L225-233 / `handle_to_thumb_slider` PRISMATIC Mimic L236-245 / 锯齿 `for i in range(6)` L120 | OTF 双 PRISMATIC + mimic spine + tanto 刃 + plain 尾端 + handle 共享基线 + 锯齿 N=6 基线 + blade-in-channel captured 范式 |
| S2 | A | side_folding_pivot | rec_military_knife_var_foldpivot | `_build_shell_shape`(channel) L83-96 / `_build_blade_shape`(pivot-at-origin) L107-136 / `pivot_pin` visual L191-197 / `blade`+`thumb_stud` L200-215 / `handle_to_blade` REVOLUTE 绕 Z 0→π L217-228 / 断言恰一非 fixed joint L272-282 | 侧开折刀 REVOLUTE spine（2 part，blade pivot-at-origin，pivot_pin/thumb_stud visual）|
| S3 | A | sliding_sheath_fixedblade | rec_military_knife_var_sheathslide | `_build_shell_shape`(实心) L70-74 / `_build_blade_shape`(世界坐标固定刀) L84-110 / `_build_guard_shape` L113-124 / `_build_sheath_shape` L127-168 / `tanto_blade`+`guard` handle visual L196-206 / `sheath` part L235-240 / `handle_to_sheath` PRISMATIC L248-258 / 断言无 blade/slider part L330-336 | 滑鞘固定刀 PRISMATIC spine（刀身是 handle fixed visual，sheath collar 罩护，guard 止挡）|
| S4 | B | drop_point | rec_military_knife_var_droppoint | `_build_blade_shape` spline 双曲线（凸脊下弯 + belly 上翘居中相交）L104-144 / 锯齿仅 rear straight 段 L134-143 | drop_point 刃轮廓（spline 平滑收尖，无角折）|
| S5 | B | dagger | rec_military_knife_var_dagger | `_build_blade_shape` 对称 polyline + `top_ridge` union（中央 spine 凸条）L107-139 | dagger 对称双刃 + 中央 spine ridge（关锯齿轴）|
| S6 | B | clip_point | rec_military_knife_var_clippoint | `_build_blade_shape`（直主刃微上翘 + n_clip=8 折线凹弧 clip）L104-154 / 锯齿仅 straight 段 L144-153 | clip_point 刃轮廓（凹弧 clip 下削细尖）|
| S7 | D | glassbreak | rec_military_knife_var_glassbreak | `_build_glass_breaker`(makeCone 锥 + collar) L131-157 / `glass_breaker_spike` handle visual L239-243 | 破窗椎尾端（makeCone 短锥 + mounting collar，handle visual material tungsten）|
| S8 | D | pocketclip | rec_military_knife_var_pocketclip | `_build_pocket_clip`(anchor+bend+arm+lip) L142-195 / clip 尺寸常量 L60-69 / `pocket_clip` handle visual L277-279 | 口袋夹尾端（+Y 侧薄簧片 anchor/bend/arm/lip，CLIP_STANDOFF 离面立起，handle visual）|
| S9 | serration | serr_3（multiplicity 小 N）| rec_military_knife_var_serr3 | `for i in range(3): cx=-0.100+i·0.030; blade.cut(0.008×0.008 box)` L119-128 / 断言恰 3 齿 L398-417 | 锯齿 N=3 大齿端（spacing 0.030，验 multiplicity 小 N）|
| S10 | serration | serr_10（multiplicity 大 N + 常量化）| rec_military_knife_var_serr10 | `SERRATION_COUNT=10`/`SERRATION_X0=-0.115`/`SERRATION_SPACING=0.010` L52-54 / `for i in range(SERRATION_COUNT): cut` L127-134 / 断言 count=10/spacing=0.010 L325-335 | 锯齿 N=10 细齿端（spacing 0.010，验 multiplicity 大 N，提为常量供模板参数化）|
