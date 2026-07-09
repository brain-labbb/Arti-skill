# tipping_barrow — modular spec (SPEC_ONLY_DRAFT)

## 元信息
| 项 | 值 |
|---|---|
| slug | `tipping_barrow` |
| template path | `agent/templates/Urban_Environment_Tipping_Barrow.py` |
| test path (optional) | `tests/agent/test_tipping_barrow_template.py` |
| stage | `SPEC_ONLY_DRAFT` |
| status | `pending` |
| __modular__ | `True` |
| pattern | `mixed`（parallel_children: tub/wheels/support 都挂到 frame root；wheel_count 是一根 multiplicity 轴；tip_mechanism 可在 frame→tub 之间插入一个 lift_arm 链节 → 局部 linear_chain） |

`pattern` 说明：steel **frame 是 root**，载荷 **tub/hopper**、**ground wheels** 与 **support**（legs / casters）
都是挂到 frame 的 parallel children。`wheel_count` 是一根 `for ... in WHEEL_LAYOUT` 的 multiplicity 轴
（共享 `_wheel_part` helper）。`tip_mechanism` 决定 frame→tub 之间是直接 REVOLUTE（front_pivot_tip）还是
插入一个 `lift_arm` 中间 part 形成 frame→lift_arm→hopper 的两跳 REVOLUTE 链（lift_and_tip）。support 的
legs / caster sub-wheels 是 module-local 的 `for side in (1,-1)` 次级复制循环，不暴露为独立 N 轴。

## 5 星样本阅读摘要
| 项 | 值 |
|---|---|
| five_star_total | 9 |
| read_count | 9 |
| read_scope | all 5-star samples in this category（2 parents + 7 single-axis variants） |
| source_index_policy | only adopted module sources are indexed below |

读到的 9 个源（全部为本小类 retained 5★，已 sync 进 `data/records/`；每个都读了
`model.py` + prompt + revision/record metadata）：

- **S1** parent `rec_single-wheel-garden-wheelbarrow-a-gray-sheet-met_20260608_164456_401819_ffd09fd0`
  — 单轮园艺 wheelbarrow：lofted sheet-metal 浅 bowl tray **FIXED 在 frame** 上、tubular 管架、2 后立腿 +
  foot rail、1 前 pneumatic 轮 CONTINUOUS、2 黄 grip。tip 靠人抬整车（tray 无 tip joint，wheel 是唯一非
  fixed joint）。
- **S2** `rec_tipping_barrow_var_wheel2` — **两轮 tipping barrow**：sheet-metal bowl **tub 是独立 part，经
  `frame_to_tub` REVOLUTE 前倾倾倒**；2 轮共享后轴 CONTINUOUS（`for i,side in enumerate([1,-1])` +
  `_make_wheel_part` helper）；前立腿 + foot rail。**这是同时具备真 tip joint + 2 轮 + barrow 身份的规范
  根模板。**
- **S3** parent `rec_heavy-duty-plastic-tilt-truck-dump-cart-a-large-_20260608_164439_693743_747a4fc9`
  — 重型塑料 tilt-truck：tapered ribbed PE hopper 经 `frame_to_hopper` REVOLUTE 前倾倾倒；2 大后轮
  CONTINUOUS（`for sy,wheel_name,joint_name`）+ 2 前 swivel caster（yoke REVOLUTE yaw + wheel CONTINUOUS
  roll，`for i,sy in ((0,1),(1,-1))`）；low steel base frame；hopper trunnion saddle 抱轴。
- **S4** `rec_tipping_barrow_var_wheel4` — 四轮 tilt-truck：`for i,axle_x,sy,has_swivel in WHEEL_LAYOUT`
  单循环 emit 全部 4 轮（共享 `_wheel_mesh`/`_tire_mesh`），rear pair 直接 CONTINUOUS、front pair 经 yoke
  REVOLUTE+CONTINUOUS；frame 抬高让前 caster 自由 swivel。
- **S5** `rec_tipping_barrow_var_tubdeep` — 深 tub tilt-truck：仅改 `_OUTER_SECS` 截面 + `RIM_Z=1.38`，直壁深
  bin，其余拓扑 = S3。
- **S6** `rec_tipping_barrow_var_tubround` — 单轮 rounded-pan barrow：tub 改用 `_ellipse_loop` 椭圆截面
  lofted 圆盘 pan + 卷边 rim，near-circular 平面；其余拓扑 = S1（tub FIXED、单前轮、2 立腿）。
- **S7** `rec_tipping_barrow_var_lifttip` — **lift-and-tip 连杆**：frame→`lift_arm`（REVOLUTE 抬升，axis -Y）→
  `hopper`（REVOLUTE 前倾，axis +Y）两跳链；U 形 lift_arm 由 `for i in range(2)` 镜像 `_one_lift_arm_mesh`
  helper 拼出；其余（2 大轮 + 2 caster）= S3。
- **S8** `rec_tipping_barrow_var_legs` — 单轮 barrow，把 2 立腿 + foot rail 从 frame 大 mesh 里**拆成独立
  named visual**，经 `for i,side in enumerate((1,-1))` + `_leg_mesh`/`_foot_rail_mesh` helper emit。
- **S9** `rec_tipping_barrow_var_caster` — tilt-truck，把 caster 装配抽成 `_build_caster(model,frame,i,sy,...)`
  helper 经 `for i in range(2)` emit（yoke REVOLUTE + wheel CONTINUOUS）；其余 = S3。

两条来源**家族 / spine**：
- **barrow spine**（S1/S2/S6/S8）：tubular 管架 + sheet-metal lofted bowl/pan + 后/前立腿 + 2 黄 grip +
  单或双轮；`tube_from_spline_points` 管架、`_rrect_loop`/`_ellipse_loop` 截面、`WHEEL_R≈0.18`。
- **tilt-truck spine**（S3/S4/S5/S7/S9）：low box base frame + tapered ribbed PE hopper（hollow loft +
  inner inset + rim ring + trunnion saddle）+ 2 大轮 + 2 swivel caster；`WHEEL_R≈0.205`、`BoxGeometry` 框架、
  hopper `_seat(-pivot)` 把 world-coord 几何坐回 pivot 子帧。

两 spine 共享同一抽象拓扑：**frame(root) → {tub REVOLUTE 倾倒（或 FIXED + 抬车）, N×wheel CONTINUOUS,
support}**。模板把两 spine 统一成一组 slot，frame/tub 几何按所选 tub_shape + support family 走对应分支。

## 核心身份

tipping_barrow = **手推 / 拖拽式翻斗车（wheelbarrow / tilt-truck dump cart）**：一个 **tub / tray / hopper**
载体装在 **wheeled steel frame** 上，**绕 lateral（Y）轴的 REVOLUTE pivot 向前（+X）翻转倾倒载荷**——这条
tip REVOLUTE 是类别定义关节。地面靠 **1 或 2（少数 4）个 front/axle 轮 CONTINUOUS 滚动** + 静止时落在
**handle/leg frame 或 axle cradle 或 swivel caster** 上。坐标约定 **Z-up、floor z=0、+X = 前进/倾倒方向、
+Y = 横向轴线、centerline y=0**，所有重复件镜像跨 y=0。核心成熟域 = 园艺独轮/双轮 barrow + 工业塑料
tilt-truck，二者都保留至少一个真倾倒/滚动非 fixed 关节。

**至少一个非 fixed 关节恒为真**：要么 tub 自身有 tip REVOLUTE（front_pivot_tip / lift_and_tip），要么
（lift_off_then_lift，tub FIXED 抬车倾倒）由 wheel 的 CONTINUOUS roll 担当真关节——绝不允许全 fixed 的纯
静态模型。

不该混入（见末节边界）：**Caster_Trolley**（纯搬运台车，无倾倒 tip 关节、平台不翻）、**Draft_Wagon**
（畜力/平板四轮拖车，有转向前桥 + 拉杆、无前倾倾倒 hopper）。

## 槽位 + 候选模块表

模板分 4 个 slot：A=wheel_count（multiplicity 轴）、B=tub_shape、C=tip_mechanism（含定义性 tip REVOLUTE）、
D=support。每个 candidate 都结构不同且有 5★ 来源。

### Slot A：wheel_count（multiplicity 轴：地面轮总数 N + front/rear 分布）

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| one_front_wheel (N=1) | S1 parent | wheel part+joint L282-L302（`frame_to_wheel` CONTINUOUS axis Y，origin `(WHEEL_X,0,AXLE_Z)`）；fork stubs L199-L204 | eligible if compatible | 单前轮居中（`abs(y)<0.006`），fork stub 抱 hub；barrow spine 基线 |
| two_axle_wheels (N=2) | S2 wheel2 | `for i,side in enumerate([1,-1])` L397-L399；`_make_wheel_part` L296-L327（`frame_to_wheel_{i}` CONTINUOUS axis Y，y=`side*TRACK/2`）；axle tube L194-L199 | eligible if compatible | 镜像一对共享后轴 `TRACK=0.56`，wheel_0/wheel_1，stub L242-L247 |
| two_big_wheels (N=2, tilt-truck) | S3 parent | `for sy,wheel_name,joint_name` L326-L351（`frame_to_wheel_l/r` CONTINUOUS axis Y，y=`sy*WHEEL_Y`）；axle L223-L227 | eligible if compatible | tilt-truck 后轴一对大轮 `WHEEL_Y=0.345`；命名 wheel_l/wheel_r |
| four_wheels (N=4, 2f+2r) | S4 wheel4 | `WHEEL_LAYOUT` L80-L86 + `for i,axle_x,sy,has_swivel` 单循环 L344-L408；rear→`frame_to_wheel_{i}` CONTINUOUS、front→`frame_to_yoke_{i}` REVOLUTE+`yoke_{i}_to_wheel_{i}` CONTINUOUS | eligible if compatible | 单 layout 表驱动全部 4 轮，rear 直接滚、front 带 swivel；frame 抬高 `FRAME_CZ=0.33` |

N candidates = {1, 2, 4} → **3 distinct N**。N=2 有两种实现（barrow 后轴 / tilt-truck 大轮），由 tub_shape +
support family 选定，不算独立 N。

### Slot B：tub_shape（载荷体横截面）

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| shallow_tray | S1/S2 | `_bowl_secs` S1 L92-L101 / S2 L98-L109 + `_tray_shell/inner/rim` S2 L112-L134 | eligible if compatible | rrect superellipse 浅 bowl，4 截面 `TRAY_DEPTH=0.27`；hollow outer+inner+卷边 rim；barrow spine |
| tapered_hopper | S3 | `_OUTER_SECS` L114-L120 + `_hopper_shell/inner/ribs/rim/saddle` L123-L200 | eligible if compatible | rrect 锥形 ribbed PE hopper，5 截面前壁陡 rear 直，3 道 rib + rim ring + trunnion saddle；tilt-truck spine |
| deep_dump_tub | S5 tubdeep | `_OUTER_SECS` L118-L124（直壁 0.52→1.0）+ `RIM_Z=1.38` L60；ribs L165-L169 | eligible if compatible | tapered_hopper 的直壁深 bin 变体：底宽 ≥55% rim、cavity depth ≥0.95m；拓扑同 hopper，仅截面/高度参数 |
| rounded_pan | S6 tubround | `_ellipse_loop` L84-L90 + `_pan_secs`（7 截面 power-curve）L96-L112 + `_tray_shell/inner/rim` L115-L134 | eligible if compatible | 椭圆截面 lofted 圆盘 pan，near-circular 平面（width/length>0.70）+ 卷边 rim；barrow spine |

注：deep_dump_tub vs tapered_hopper 属同一 loft 拓扑、不同截面参数，但**剖面族（直壁 vs 锥壁）+ 高度行程**
形成可见结构家族差异，按源 S5 收为独立 candidate（其 run_tests 增加了 cavity-depth/upright-wall 断言）。

### Slot C：tip_mechanism（如何倾倒；恒含 ≥1 个真非 fixed 关节）

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| front_pivot_tip | S2 / S3 | S2 `frame_to_tub` REVOLUTE axis `(0,-1,0)` origin `(AXLE_X,0,PIVOT_Z)` L385-L395；S3 `frame_to_hopper` REVOLUTE axis `(0,1,0)` origin pivot L418-L426 + `_seat` L399-L401 | eligible if compatible | tub/hopper 是独立 part，单 REVOLUTE 绕轴线前倾倒（lower=0），pivot 在 axle 高度；**定义关节** |
| lift_off_then_lift | S1 / S6 | S1 单 `frame_to_wheel` CONTINUOUS L294-L302（tray FIXED 在 frame L270-L275） | eligible if compatible（仅 barrow spine + tub FIXED） | tub FIXED 在 frame，靠抬整车倾倒；真非 fixed 关节由 wheel CONTINUOUS roll 担当；barrow 基线 |
| lift_and_tip | S7 lifttip | `lift_arm` part + `frame_to_lift_arm` REVOLUTE axis `(0,-1,0)` L503-L511 + `lift_arm_to_hopper` REVOLUTE axis `(0,1,0)` L546-L554；`_one_lift_arm_mesh` L295-L316 + `for i in range(2)` L323-L326；frame pivot bracket L266-L286；hopper cradle bracket L190-L229 | eligible if compatible（仅 tilt-truck spine + N∈{2,4}） | 插入 U 形 lift_arm 中间 part：先抬升再前倾的两跳 REVOLUTE 链（raise-then-tip）；hopper part frame 坐在 `_TUB_TIP_WORLD` |

### Slot D：support（静止落地支撑）

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| stand_legs | S1 / S8 | S8 `_leg_mesh`/`_leg_path` L211-L225 + `_foot_rail_mesh` L228-L233 + `for i,side in enumerate((1,-1))` L290-L299；S2 前立腿 L169-L184 | eligible if compatible（barrow spine） | 2 镜像立腿落地 + 横 foot rail，三点站姿（2 腿 + 轮）；barrow spine |
| axle_cradle | S3 / S5 | `_base_frame_mesh` rails+rear_cross+axle+blocks L207-L239（无落地腿，靠 axle 大轮 + 前 caster 站立） | eligible if compatible（tilt-truck spine） | low box base frame，载荷由后轴大轮 + 前 caster 支撑（无独立立腿）；tilt-truck 基线 |
| front_swivel_casters | S3 / S9 | S9 `_build_caster(model,frame,i,sy,...)` L307-L365 + `for i in range(2)` L416-L418（`frame_to_caster_yoke_{i}` REVOLUTE axis Z + `caster_yoke_{i}_to_wheel` CONTINUOUS axis Y）；frame mount plate L235-L238 | eligible if compatible（tilt-truck spine） | 2 前 swivel caster：king-pin REVOLUTE yaw + 小轮 CONTINUOUS roll，可转向；与 axle_cradle 叠加构成 N=4-类站姿 |

> Slot D 三 candidate 都 ≥2 来源、结构不同（落地腿 / 纯 axle 大轮 / 可转向 caster），无 single-candidate slot。
> stand_legs ↔ barrow spine，axle_cradle/front_swivel_casters ↔ tilt-truck spine，按 spine 兼容门控选取。

## 槽位图（slot graph）

pattern: mixed（root parallel_children + 1 multiplicity 轴 + 可选 lift_arm 链节）

```
                         frame (root, steel)
        ┌────────────────────┼───────────────────────┬─────────────────────┐
        │                    │                       │                     │
 [Slot C tip]          [Slot A wheels ×N]      [Slot D support]      （grips / rim / saddle
        │              for i in WHEEL_LAYOUT          │              = frame/tub local visuals）
   front_pivot_tip:    each wheel:                 stand_legs:
   frame --REVOLUTE--> tub      frame --CONTINUOUS--> wheel_i      2× _leg + foot_rail visuals on frame
   (axis ±Y, lower=0,           (axis Y, effort 40-60)            axle_cradle: frame box rails only
    origin @ axle line)                                          front_swivel_casters:
                                front swivel (N=4 / caster):       frame --REVOLUTE(Z)--> yoke_i
   lift_and_tip:                 frame --REVOLUTE(Z)--> yoke_i        --CONTINUOUS(Y)--> caster_wheel_i
   frame --REVOLUTE(-Y)-->        --CONTINUOUS(Y)--> wheel_i
     lift_arm --REVOLUTE(+Y)--> hopper
   (raise then tip, 2-hop chain)

   lift_off_then_lift:
   tub FIXED on frame（真关节 = wheel CONTINUOUS roll）
```

跨 slot 接口点位与策略：

- **frame→tub（tip，定义关节）**：mating = **lateral axle/pivot line**（origin `(AXLE_X,0,PIVOT_Z)`，PIVOT_Z 在
  tub 底/axle 高度）。`frame_to_tub` / `frame_to_hopper` REVOLUTE，**axis ±Y**，`lower=0, upper∈[0.7,1.4]`。
  几何用 `_seat(-pivot)` 把 world-coord 体坐回 pivot 子帧，q=0 时回到 authored world pose。tub 经 trunnion
  saddle（hopper）或 cross-tube 支座（barrow）抱轴、`allow_overlap`/`allow_isolated_part` + `expect_contact`
  防 floating。
- **frame→wheel_i（roll）**：mating = **axle line**（origin `(axle_x, sy*half_track, AXLE_Z)`，`AXLE_Z=WHEEL_R`
  保证轮触地 z≈0）。CONTINUOUS，**axis (0,1,0)**；轮 mesh 先 `rotate_z(π/2)` 把 WheelGeometry 的 local-X 自转
  轴转到 local Y。axle tube / fork stub `allow_overlap` 抱 hub。
- **frame→yoke_i→caster_wheel_i（swivel+roll）**：king-pin mount plate 在 frame 顶（`CASTER_X, sy*CASTER_Y`），
  yoke REVOLUTE axis Z（`lower=-π,upper=π`），caster wheel CONTINUOUS axis Y，origin 在 yoke local trailing
  offset。boss `allow_overlap` 抱 mount plate、wheel `allow_overlap` 抱 fork cheek。
- **frame→lift_arm→hopper（lift_and_tip）**：frame pivot bracket pin（`LIFT_PX,0,LIFT_PZ`）→ lift_arm 抬升
  REVOLUTE axis -Y；lift_arm tub-end pin（`TUB_TIP_DX,0,TUB_TIP_DZ`）→ hopper 前倾 REVOLUTE axis +Y。两个
  captured-pin 接口各自 element-scoped `allow_overlap`（`lift_arm_body`↔`frame`，`hopper_cradle`↔
  `lift_arm_body`）+ `expect_contact`。
- **frame→support**：stand_legs 是 frame 上的 named visual（脚 z≈0、foot rail 跨 centerline），不引入新 joint；
  axle_cradle 无独立件；front_swivel_casters 引入上面的 yoke+wheel 子树。

互斥 / 派生：
- `lift_off_then_lift`（tub FIXED）只在 **barrow spine**（shallow_tray / rounded_pan + stand_legs + N∈{1,2}）合法。
- `lift_and_tip`（lift_arm 链）只在 **tilt-truck spine**（tapered_hopper / deep_dump_tub + axle_cradle/caster
  + N∈{2,4}）合法。
- `front_swivel_casters` 与 `axle_cradle` 同属 tilt-truck spine（N=4 = axle_cradle + casters 升级）；`stand_legs`
  只配 barrow spine。
- tub_shape 决定 spine：{shallow_tray, rounded_pan}=barrow；{tapered_hopper, deep_dump_tub}=tilt-truck。spine
  反向门控 wheel_count（barrow→N∈{1,2}，tilt-truck→N∈{2,4}）与 support / tip 合法集（见兼容矩阵）。

## 每槽位 Module Emits / Interfaces

### Slot A / module one_front_wheel (N=1)
| emits | 描述 | 来源 |
|---|---|---|
| parts | `wheel`（disc + tire 双 visual） | S1 / L283-L289 |
| internal joints | `frame_to_wheel` CONTINUOUS axis (0,1,0) effort40 vel30 | S1 / L294-L302 |
| upstream interface | frame fork stub 抱 hub（`allow_overlap`+`expect_contact`） | S1 / L199-L204,L408-L412 |
| downstream interface | 触地 z≈0（`AXLE_Z=WHEEL_R`），居中 y≈0 | S1 / L57-L58 |

### Slot A / module two_axle_wheels / two_big_wheels (N=2)
| emits | 描述 | 来源 |
|---|---|---|
| parts | `wheel_0/wheel_1`（S2）或 `wheel_l/wheel_r`（S3），共享 mesh helper | S2 L296-L327 / S3 L326-L342 |
| internal joints | 每轮 `frame_to_wheel_*` CONTINUOUS axis Y，镜像 `±half_track` | S2 L318-L326 / S3 L343-L351 |
| upstream interface | 共享 axle tube + stub `allow_overlap` 抱双 hub | S2 L194-L199,L552-L561 / S3 L223-L227 |
| downstream interface | 镜像跨 y=0、同尺寸、触地 | S2 L426-L473 |

### Slot A / module four_wheels (N=4)
| emits | 描述 | 来源 |
|---|---|---|
| parts | `wheel_0..3`（+ front `yoke_2/yoke_3`），单 layout 表循环 | S4 / L80-L86,L344-L408 |
| internal joints | rear `frame_to_wheel_{0,1}` CONTINUOUS；front `frame_to_yoke_{2,3}` REVOLUTE(Z)+`yoke_*_to_wheel_*` CONTINUOUS(Y) | S4 / L376-L408 |
| upstream interface | rear axle 抱 hub；front yoke kingpin 抱 mount plate（captured-pin allow_overlap） | S4 / L619-L645 |
| downstream interface | 4 轮触地，rear/front 各镜像对，frame 抬离地面 | S4 / L499-L580 |

### Slot B / module shallow_tray / rounded_pan（barrow tub）
| emits | 描述 | 来源 |
|---|---|---|
| parts | tub `tray_outer`+`tray_inner`+`tray_rim` 三 visual（front_pivot_tip 时为独立 `tub` part；lift_off 时挂 frame） | S2 L364-L376 / S1 L270-L275 |
| internal joints | 无（tub 本身刚体；倾倒关节归 Slot C） | — |
| upstream interface | front_pivot_tip：tub part frame 在 pivot；lift_off：FIXED 在 frame，cross-tube 支座支撑 | S2 L100,L385-L395 / S1 L179-L192 |
| downstream interface | hollow cavity（inner within outer）+ 卷边 rim 在顶；floor 在 frame 上方 z>0.25 | S6 L401-L425 |

### Slot B / module tapered_hopper / deep_dump_tub（tilt-truck hopper）
| emits | 描述 | 来源 |
|---|---|---|
| parts | hopper `hopper_outer/inner/rim/ribs/saddle` 五 visual（独立 `hopper` part） | S3 L402-L412 |
| internal joints | 无（倾倒关节归 Slot C） | — |
| upstream interface | trunnion saddle 从 body 底降到 axle 抱轴；`_seat(-pivot)` 坐回 pivot 子帧 | S3 L176-L200,L397-L401 |
| downstream interface | 大锥/深直 hollow body，rim 在 `RIM_Z`，body 底 z>0.20 | S3/S5 L489-L528 |

### Slot C / module front_pivot_tip（定义关节）
| emits | 描述 | 来源 |
|---|---|---|
| parts | 复用 Slot B 的 tub/hopper part | — |
| internal joints | `frame_to_tub`/`frame_to_hopper` REVOLUTE axis ±Y，origin axle 线，lower=0 upper 0.7-1.4 | S2 L385-L395 / S3 L418-L426 |
| upstream interface | pivot @ `(AXLE_X,0,PIVOT_Z)`，saddle/支座抱轴 | S3 L397,L606-L613 |
| downstream interface | 正 q 时 tub front（+X）下沉、rear 上抬（倾倒动作可检） | S2 L497-L509 |

### Slot C / module lift_and_tip
| emits | 描述 | 来源 |
|---|---|---|
| parts | `lift_arm`（U 形，2 镜像臂 + 横梁 + tub-end pin）+ 复用 hopper part | S7 L295-L341,L492-L496 |
| internal joints | `frame_to_lift_arm` REVOLUTE axis -Y（抬升 lower0 upper1.0）+ `lift_arm_to_hopper` REVOLUTE axis +Y（前倾 lower0 upper1.2） | S7 L503-L511,L546-L554 |
| upstream interface | frame pivot bracket pin 抱 lift_arm 后 eye（`lift_arm_body`↔`frame` allow_overlap） | S7 L266-L286,L763-L773 |
| downstream interface | lift_arm tub-end pin 抱 hopper cradle bracket（`hopper_cradle`↔`lift_arm_body` allow_overlap）；组合 q 抬升+前倾 | S7 L190-L229,L775-L786 |

### Slot D / module stand_legs
| emits | 描述 | 来源 |
|---|---|---|
| parts | frame 上 `leg_0`/`leg_1` + `foot_rail` named visual（非独立 part） | S8 L286-L299 |
| internal joints | 无（静态支撑） | — |
| upstream interface | leg 顶接 handle tube；foot rail 跨 centerline 连双脚 | S8 L211-L233 |
| downstream interface | 双脚 z≈0，与 wheel 构成三点站姿 | S8 L430-L437 |

### Slot D / module axle_cradle
| emits | 描述 | 来源 |
|---|---|---|
| parts | frame box rails + rear_cross + axle + blocks（无独立支撑件） | S3 L207-L239 |
| internal joints | 无 | — |
| upstream interface | 载荷由后轴大轮承担，frame 底近地 z≤0.04 | S3 L481-L486 |
| downstream interface | 与 axle 大轮 + 前 caster 共同站立 | S3 L470-L486 |

### Slot D / module front_swivel_casters
| emits | 描述 | 来源 |
|---|---|---|
| parts | `caster_yoke_{i}` + `caster_wheel_{i}`（`for i in range(2)` + `_build_caster` helper） | S9 L307-L365,L416-L418 |
| internal joints | `frame_to_caster_yoke_{i}` REVOLUTE axis Z（lower-π upper π）+ `caster_yoke_{i}_to_wheel` CONTINUOUS axis Y | S9 L331-L365 |
| upstream interface | kingpin boss 抱 frame mount plate（captured-pin allow_overlap） | S9 L235-L238,L697-L708 |
| downstream interface | 小轮触地，可绕 kingpin yaw（XY swing） | S9 L587-L601 |

## 参数范围汇总

| 参数 | 类型 | 取值范围 / 候选值 | 标称默认 | 约束类型 | 约束 / 函数 | 来源 |
|---|---|---|---|---|---|---|
| tub_shape | enum | shallow_tray / tapered_hopper / deep_dump_tub / rounded_pan | — | choice | deterministic sampler 选；决定 spine | Slot B 表 |
| tip_mechanism | enum | front_pivot_tip / lift_off_then_lift / lift_and_tip | — | conditional | 合法集随 spine（兼容矩阵）；恒含 ≥1 非 fixed 关节 | Slot C 表 |
| support_family | enum | stand_legs / axle_cradle / front_swivel_casters | — | conditional | barrow→stand_legs；tilt-truck→{axle_cradle, front_swivel_casters} | Slot D 表 |
| wheel_count | int(N) | {1, 2, 4} | — | conditional | barrow→{1,2}；tilt-truck→{2,4}（N=4 蕴含 caster） | Slot A 表 |
| tub_depth_scale | float | [0.85, 1.30] | 1.0 | independent | barrow `TRAY_DEPTH=0.27`、hopper `RIM_Z-BOTTOM_Z` 高度缩放后 clamp | S5 L60 / S2 L67 |
| tub_len_scale | float | [0.90, 1.15] | 1.0 | independent | `TRAY_LEN`/`BODY_LEN` 缩放 | S3 L58 / S2 L60 |
| tub_wid_scale | float | derived | 1.0 | equation | `= tub_len_scale`（保形，避免极端长宽比破坏 rrect r-clamp） | S3 L59 |
| wheel_radius_scale | float | [0.90, 1.15] | 1.0 | independent | `WHEEL_R` 缩放；`AXLE_Z=WHEEL_R*scale` 联动保证触地 | S1 L55 / S3 L64 |
| track_scale | float | [0.90, 1.15] | 1.0 | independent（N≥2） | `TRACK`/`WHEEL_Y` 横距缩放（N=1 无效） | S2 L60 / S3 L66 |
| tip_upper | float | [0.7, 1.4] | 1.1 | independent | tip REVOLUTE upper 行程；lower=0 锁定 | S2 L394 / S3 L425 |
| handle_len_scale | float | [0.90, 1.15] | 1.0 | independent（barrow spine） | `GRIP_X` 后伸长度缩放 | S1 L67 |
| palette_style | enum | garden_steel / industrial_gray / safety_orange / agri_green / municipal_blue / galv_zinc | garden_steel | choice | ≥3（目标 6），见下 palette 节 | S1 L254-L259 / S3 L310-L314 |
| (—) | constraint | — | — | inequality | `tub_wall_z_min`（body 底）> `support_top` 余隙；违反则回缩 tub_depth_scale | S2 L491-L495 |
| (—) | constraint | — | — | inequality | tilt-truck：`tub_bottom_half_y < WHEEL_Y` 保证 body 底不撞轮；违反回缩 tub_wid | S5 注释 L114-L117 |
| (—) | constraint | — | — | inequality | `AXLE_Z = WHEEL_R·wheel_radius_scale`（轮心高=半径）保证触地 z≈0；非自由变量 | S1 L58 |

**palette_style 候选（≥3，目标 6 colorways）**：
- `garden_steel`：frame 银灰 `(0.62,0.63,0.66)` + tray 钢灰 `(0.46,0.47,0.49)` + grip 黄 `(0.86,0.78,0.20)` + 黑胎（S1 基线）
- `industrial_gray`：hopper 灰塑 `(0.62,0.63,0.64)` + steel `(0.34,0.35,0.37)` + wheel_gray（S3 基线）
- `safety_orange`：tub `(0.85,0.40,0.10)` + 黑 frame + 黄 grip（市政/工地）
- `agri_green`：tub `(0.20,0.45,0.22)` + 灰 frame（农用）
- `municipal_blue`：hopper `(0.20,0.35,0.62)` + steel frame（环卫 tilt-truck）
- `galv_zinc`：tub/frame 镀锌亮灰 `(0.70,0.72,0.74)` 单色金属 + 黑胎

palette 只改 material rgba，不改拓扑 / 几何 / 关节。

## Multiplicity / Copy Logic

本模板有 **1 根 multiplicity 轴**（wheel_count）。其余重复件（2 legs、2 grips、2 casters、2 lift arms、ribs、
saddle plates）是**固定 K=2/3 的 module-local 次级循环**，不暴露为模板级 `*_count` N 轴。

**轴 1 — wheel_count（地面轮总数 N）**

- `count_param`：`wheel_count`（int，由 `slot_choices_for_seed` 决定，conditional on spine）。
- `N_range`：本小类本轴产品域 = **{1, 2, 4}**（无大 N 尾部——翻斗车物理上 ≤4 轮；测试与产品同域，无需大 N
  采样）。
- sampling domain（权重档）：N=2 最常见（barrow 双轮 + tilt-truck 双大轮）权重最高；N=1（独轮园艺 barrow）次之；
  N=4（带 caster 工业车）较稀。建议加权 `{2:0.5, 1:0.3, 4:0.2}`，再被 spine 门控裁剪（barrow→只 {1,2}，
  tilt-truck→只 {2,4}）后归一化。
- copied object：`wheel`（barrow N=1）/ `wheel_0,wheel_1` / `wheel_l,wheel_r`（N=2）/ `wheel_0..3` + front
  `yoke_2,yoke_3`（N=4），全部经单一 `for ... in WHEEL_LAYOUT` 循环 + 共享 `_wheel_mesh`/`_tire_mesh` helper。
- naming：N=1 → `wheel`/`frame_to_wheel`；N≥2 → `wheel_{i}`/`frame_to_wheel_{i}`（front swivel → `yoke_{i}` +
  `frame_to_yoke_{i}` + `yoke_{i}_to_wheel_{i}`）。统一索引化命名。
- placement：rear axle `(AXLE_X, sy*half_track, AXLE_Z)`；front axle（N=4）`(FRONT_AXLE_X, sy*half_track,
  YOKE_MOUNT_Z)`。镜像跨 y=0，`AXLE_Z=WHEEL_R·scale` 保证触地。
- joint policy：每轮一个 CONTINUOUS roll axis (0,1,0) effort 40-60 vel 30；front swivel 额外一个 REVOLUTE yaw
  axis (0,0,1) lower-π upper π。
- source/gating：S4 `WHEEL_LAYOUT` L80-L86 单循环 L344-L408。N=4 蕴含 front swivel（has_swivel=True）。spine
  门控见兼容矩阵。

**次级固定循环（K 固定，非 N 轴）**：legs `for i,side in enumerate((1,-1))`（S8 L290）、casters
`for i in range(2)`（S9 L416）、lift arms `for i in range(2)`（S7 L323）、grips（S2 L350-L356）、ribs（S3
L161-L173）、saddle plates `for sy in (1,-1)`（S3 L186）——均 K=2/3 module-local，不参与 multiplicity 采样。

## 拓扑多样性审计

总组合数（spine-gated，剔除非法组合后）：

- **barrow spine**：tub∈{shallow_tray, rounded_pan}(2) × tip∈{front_pivot_tip, lift_off_then_lift}(2) ×
  support=stand_legs(1) × N∈{1,2}(2) = **8** 组合。
- **tilt-truck spine**：tub∈{tapered_hopper, deep_dump_tub}(2) × tip∈{front_pivot_tip, lift_and_tip}(2) ×
  support∈{axle_cradle, front_swivel_casters}(2) × N∈{2,4}(2，N=4 ⇒ caster) = ~**12** 组合（去掉
  N=2+caster-only 冗余约 ~10 distinct）。
- 合计 **≈18 distinct topology 组合**（含 3 distinct N），远超机械门槛。

理由：18 个合法 slot/module/N 组合在 part-tree / joint-topology / chain-depth（front_pivot_tip 单跳 vs
lift_and_tip 双跳 + lift_arm part vs lift_off 零 tip-joint）/ wheel-count(1/2/4) / support 子树（legs-visual vs
caster-yoke-subtree）上都拓扑可分；模板 sweep 应产 ≥10 distinct module-topology。

seed_domain_policy：**procedural_first**。

Procedural Sampling / Sweep Plan：`config_from_seed` 先 deterministic 选 tub_shape（定 spine），再 conditional
裁剪并采 wheel_count / tip_mechanism / support_family / palette_style（按上面权重 + 兼容矩阵 gate），再采
independent 连续 scale（tub_depth/len/wheel_radius/track/tip_upper/handle_len）→ 派生 tub_wid（=len）→ 用
inequality 把 tub 余隙 / body-底-不撞轮 / 触地高度投影回缩。`slot_choices_for_seed` 返回与 build 完全一致的
module name 集。无非法组合泄漏（spine 门控在采样阶段解析）。少量 regression overrides 仅用于已知边界（见下）。

Topology target：1000-seed slot choice tuple distinct 目标 ~18（受类别物理约束——翻斗车结构家族有限，<300 合理，（统一口径：富类别建议 ≥300；低于 300 记录真实组合空间或兼容约束原因；report-only，不设门。）
因为合法 spine×tub×tip×support×N 组合本就 ≈18；多样性主要来自 18 拓扑 × 6 palette × 连续 scale 抖动，而非
更多拓扑）。说明：本类别拓扑天花板低是类别本质（不是缺陷），主 seed domain 用全 18 组合 + palette + scale，
不靠 modulo 表轮换。

Controlled local parameterization：初版模板应含 `tub_depth_scale`(independent [0.85,1.30])、
`tub_len_scale`(independent [0.90,1.15])、`tub_wid_scale`(equation =len)、`wheel_radius_scale`(independent
[0.90,1.15]，联动 AXLE_Z)、`track_scale`(independent [0.90,1.15], N≥2)、`tip_upper`(independent [0.7,1.4])、
`handle_len_scale`(independent [0.90,1.15], barrow)。全部在 `resolve_config` 内 clamp / 派生 / 投影，遵循
采样契约（先 independent → 派生 equation → inequality 投影回缩 → conditional 范围按 spine 解析），不破坏
pivot 接口、轮触地、tub 余隙、support 三点站姿与类别 identity。

| item | policy | validator / viewer focus |
|---|---|---|
| sampler | tub_shape→spine→(wheel_count, tip, support, palette) weighted+gated→continuous scales | slot_choices_for_seed == build choices |
| compatibility matrix | barrow spine: tub∈{shallow,round}, tip∈{front_pivot, lift_off}, support=legs, N∈{1,2}；tilt-truck spine: tub∈{hopper,deep}, tip∈{front_pivot, lift_and_tip}, support∈{axle_cradle, caster}, N∈{2,4}(N4⇒caster)；非法组合采样阶段排除 | no floating tub, no全fixed, axle 触地, lift chain origin, caster swivel, body-不撞轮 |
| controlled local variation | 上述 7 个 scale，clamp+derive+inequality 投影 | 比例变化不破坏 pivot/clearance/support/joint range/identity |
| regression overrides | sparse：barrow N=2 + front_pivot_tip（S2 规范根）、tilt-truck lift_and_tip N=4（最深链）、deep_dump_tub cavity 边界（S5）——仅审核指定边界 case | previously-failed/reviewer cases only |
| random sweep | seeds 0-49 初筛，0-999 成熟度审计 | contract failures |

| slot | candidate_count | 是否 ≥2 | 是否 ≥3 | 备注 |
|---|---:|---|---|---|
| A wheel_count | 3 (N∈{1,2,4}; N=2 双实现) | yes | yes | multiplicity 轴 |
| B tub_shape | 4 | yes | yes | 2 barrow + 2 tilt-truck |
| C tip_mechanism | 3 | yes | yes | 含定义性 tip REVOLUTE |
| D support | 3 | yes | yes | spine-gated |

## Validator

- slot_choices_for_seed returns implemented module names（tub_shape/tip/support/wheel_count/palette）
- config_from_seed uses deterministic procedural sampling for all ordinary seeds（spine-first）
- compatibility matrix / gating prevents illegal combos（barrow↔stand_legs↔N{1,2}↔{front_pivot,lift_off}；
  tilt-truck↔{axle_cradle,caster}↔N{2,4}↔{front_pivot,lift_and_tip}）
- **定义性关节恒在**：front_pivot_tip/lift_and_tip 有 tub/hopper tip REVOLUTE axis ±Y lower=0；
  lift_off_then_lift 时 tub FIXED 但 wheel CONTINUOUS roll 担当真关节（绝不全 fixed）
- key joints type/axis/range：tip REVOLUTE axis |y|>0.9, lower=0；wheel CONTINUOUS axis (0,1,0)；caster yaw
  REVOLUTE axis (0,0,1)；lift_arm REVOLUTE axis -Y
- copied wheels follow naming（wheel/wheel_i/wheel_l/r）+ placement（镜像跨 y=0，AXLE_Z=WHEEL_R·scale 触地）
- controlled scales clamped；cross-part deps（tub_wid=len equation；tub 余隙 / body-不撞轮 / 触地 inequality）
  resolved in resolve_config
- critical InterfaceSpec/MatingContract exist：pivot 抱轴（saddle/cross-tube）、fork stub 抱 hub、caster
  kingpin 抱 mount plate、lift captured pins
- optional regression overrides sparse & justified；不靠小 curated/modulo 表轮换主 seed domain

## Reject cases

1. **tub 不翻 / 无 tip 关节 且 wheel 也 FIXED**（全 fixed 静态模型）——丢失倾倒+滚动两个真关节，违反类别身份。
2. **floating tub / hopper**：tub part 与 frame 无 saddle/cross-tube 接触（缺 `expect_contact`/`allow_overlap`），
   tip pivot 悬空。
3. **wheel 不触地**：`AXLE_Z ≠ WHEEL_R·scale` 导致轮 z_min 偏离 0（穿地或悬空）。
4. **tip 轴错误**：tip REVOLUTE 轴非 ±Y（如绕 X/Z），或 lower≠0（休止位不在水平），倾倒方向错。
5. **spine 串味**：barrow 管架配 tilt-truck hopper、或 stand_legs 配 N=4 caster（非法跨 spine 组合泄漏到
   seed domain）。
6. **lift_and_tip 链断**：lift_arm 抬升后 hopper 不随之上升，或 captured-pin 接口缺 element-scoped allow_overlap
   导致穿模 / 漂浮。
7. **body 撞轮**：tilt-truck tub 底半宽 ≥ WHEEL_Y（深/宽 tub 缩放后侧壁穿大轮）。
8. **caster 不能 swivel / 不触地**：kingpin REVOLUTE 轴非 Z 或 yoke 几何使小轮悬空。

## 与相邻类别的边界

- **不该混入：Caster_Trolley**（搬运台车/手推车）——trolley 是平台 + 4 caster 纯搬运，**没有前倾倾倒的 tip
  REVOLUTE**（其唯一关节是 caster swivel/roll，平台刚性不翻）。tipping_barrow 的**定义关节是 tub 绕 lateral
  轴前倾倒**；若一个模型平台不翻、只靠 caster 移动，应归 Caster_Trolley 而非本类。
- **不该混入：Draft_Wagon**（畜力/平板四轮拖车）——wagon 是四轮平板车 + **转向前桥（front bogie / steering
  axle）+ 拉杆/辕（draft pole）**，靠牵引前进、载货平板**不前倾倾倒**。tipping_barrow 无转向前桥/拉杆，靠人
  手推 handle 或拖拽，且必须有前倾 dump 机构。若模型有转向前桥 + 牵引杆且车斗固定不翻，应归 Draft_Wagon。

## 审核记录
| 项 | 结论 |
|---|---|
| reviewer status | pending |
| reviewer notes | — |

## 模板实现备注（可选）

- barrow spine 与 tilt-truck spine 各有一套管架/箱架 helper；建议共享 `_rrect_loop`、`_wheel_part`
  （`rotate_z(π/2)` 把自转轴转到 Y）、`_seat(-pivot)` 倾倒子帧坐回逻辑。
- tub/hopper 用 `_seat(-pivot)` 把 world-coord loft 坐回 tip 子帧，q=0 回到 authored world pose——front_pivot_tip
  与 lift_and_tip 都依赖此模式（lift_and_tip 用 `_TUB_TIP_WORLD` 作 offset）。
- captured-pin overlap 需 element-scoped allow_overlap：fork-stub↔hub、caster-kingpin↔mount-plate、
  lift `lift_arm_body`↔`frame` 与 `hopper_cradle`↔`lift_arm_body`、hopper saddle↔frame-axle。
- lift_off_then_lift 时 tub 是 frame 上的 FIXED visual（非独立 part），用 `allow_isolated_part` +
  cross-tube `expect_contact`（contact_tol≈0.025）容许休止小余隙（S2 L565-L575）。
- WheelHub BoltPattern 用全正参数（count≥4, circle_diameter>0），避免 axisymmetric spin-check 退化。
- 暂不进入 seed domain 的组合：barrow spine + caster、tilt-truck + stand_legs（跨 spine，物理不合理）。
