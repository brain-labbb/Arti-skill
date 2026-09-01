# Searchlight Tower Spec

## 元信息
| 项 | 值 |
|---|---|
| slug | `searchlight_tower` |
| template path | `agent/templates/searchlight_tower.py` |
| test path | `tests/agent/test_searchlight_tower_template.py` |
| stage | `REVIEWED` |
| status | `approved` |
| __modular__ | `True` |
| pattern | `linear_chain` |

## 5 星样本阅读摘要
| 项 | 值 |
|---|---|
| five_star_total | 34 |
| read_count | 34 |
| read_scope | all 5-star samples in this category: `model.py` / `revision.json` / `record.json` / `prompt.txt` were read |
| samples_adopted_as_module_sources | 6 |
| samples_read_but_not_adopted | 28 |
| source_index_policy | only adopted reusable snippets are indexed below |

全量阅读后的结构族分布：

| 结构族 | 样本数 | 说明 |
|---|---:|---|
| pole/tube-mast -> pan yoke -> spotlight head | 14 | 单立柱（圆/方）+ 顶部 pan bearing + 2-arm trunnion yoke 抱住 barrel 灯头；pan(Z) + tilt(horizontal) |
| lattice/braced-frame tower -> pan yoke -> head | 7 | 角柱 + 横撑 + 斜撑 + 服务平台/爬梯/护栏的桁架塔，运动链相同 |
| tripod/legged compact support -> yoke -> lamp | 4 | 短中心柱 + 3-4 条外撑腿，便携感，运动链相同 |
| foundation + mast + 独立 FIXED service platform | 3 | 在 pan stage 之前多出 FIXED 结构件（平台/爬梯） |
| desktop / portable folding searchlight | 2 | base->mast 增加 fold(Y) 关节，再 pan + tilt |
| 精密仪器 / 额外 DOF 变体 | 3 | pan+tilt 之外增加 knob/handwheel/switch/lock-pin 等附加旋钮或锁紧件 |
| lamp head shell construction variants | 34 | primitive cylinder barrel / `LatheGeometry.from_shell_profiles` 中空灯壳 / cadquery tube；恒有 translucent front lens |

被采纳样本逐条标注：
- `rec_searchlight_tower_df462b4bd099451784e6bf4402deba8c` — adopted：最干净的 canonical pole-mast，3 part（base_tower / yoke_stage / lamp_head），2 joint（pan REVOLUTE Z + tilt REVOLUTE (0,-1,0)），全 primitive。
- `rec_searchlight_tower_0001` — adopted：premium pole+turret，tube-frame mast helper、`LatheGeometry` 灯壳、head carry-handle、turret pan stage。
- `rec_searchlight_tower_a99f7fe231a84b2d87c6c29bfc2b8324` — adopted：`TrunnionYokeGeometry` 单 mesh yoke + tripod 腿 root + reflector。
- `rec_searchlight_tower_0003` — adopted：lattice/服务塔，strut/member helpers、mast loft、perforated deck、swept yoke arms、ladder/railing。
- `rec_searchlight_tower_d502cc5a2eed4557ad6007b93841e3c0` — adopted：exposed-pole + tube-spline 斜撑 + 可见 bearing collars + split yoke arms + Lathe 灯壳。
- `rec_searchlight_tower_b5336452915842dbb1abcfc3324fa716` — adopted：desktop folding 变体，base->mast fold(Y) + pan + tilt。
- Remaining 28 samples — read but not adopted：同一 support -> pan yoke -> head 链的 axis 符号 / 材质 / 尺度 / cooling fins / cable gland 等装饰差异，已被上述模块覆盖。

## 核心身份
`searchlight_tower` 是一个可定向的探照灯塔：静态支撑结构（pole mast / lattice tower / tripod / desktop fold base）顶部承载一个绕竖直 Z 轴方位旋转（pan）的 stage，再通过一对 trunnion 抱住 barrel/can 形灯头，灯头绕水平轴俯仰（tilt）。灯头必须有朝光束方向的 translucent front lens + bezel，并常带 reflector/bulb。默认成熟域是 two-DOF pan/tilt 链；service platform/ladder、carry handle、cooling fins、lock pin/knob/handwheel、fold 都是可选辅助。

边界：
- 不包括固定不动的泛光灯：必须有 pan + tilt articulation。
- 不混入 parabolic dish/天线：灯头是 barrel + 凸透镜 lens，不是 concave reflector dish。
- 不混入 CCTV pan-tilt camera head：身份件是带 lens 的 spotlight barrel，不是 camera + 镜头模组。
- 不混入 crane/observation tower：必须有灯头，不能只剩塔身。

## 采用源码索引（Adopted Source Index）
| source_id | sample_id | model.py 来源 | 采纳用途 |
|---|---|---|---|
| S1 | `rec_searchlight_tower_df462b4bd099451784e6bf4402deba8c` | `data/records/rec_searchlight_tower_df462b4bd099451784e6bf4402deba8c/revisions/rev_000001/model.py:L47-L210` | canonical 3-part pole skeleton：base_tower L47-L83、yoke_stage L85-L132、lamp_head L134-L181、pan/tilt joints L183-L210 |
| S2 | `rec_searchlight_tower_0001` | `data/records/rec_searchlight_tower_0001/revisions/rev_000001/model.py:L42-L201,L269-L441` | tube-frame mast helper L42-L165、Lathe 灯壳 L168-L184、carry-handle L187-L201、turret pan stage L269-L329、lamp head L331-L412、joints L414-L441 |
| S3 | `rec_searchlight_tower_a99f7fe231a84b2d87c6c29bfc2b8324` | `data/records/rec_searchlight_tower_a99f7fe231a84b2d87c6c29bfc2b8324/revisions/rev_000001/model.py:L20-L227` | `_cylinder_between` L20-L40、legged/tripod support L53-L124、`TrunnionYokeGeometry` 单 mesh yoke L126-L155、lamp + trunnion + reflector L157-L205、joints L207-L227 |
| S4 | `rec_searchlight_tower_0003` | `data/records/rec_searchlight_tower_0003/revisions/rev_000001/model.py:L47-L194,L306-L399,L601-L623` | strut/member helpers L47-L94、mast loft L101-L111、perforated deck L114-L123、swept yoke arms L126-L140、shell from_shell_profiles L143-L194、ladder/rail builders L306-L399、joints L601-L623 |
| S5 | `rec_searchlight_tower_d502cc5a2eed4557ad6007b93841e3c0` | `data/records/rec_searchlight_tower_d502cc5a2eed4557ad6007b93841e3c0/revisions/rev_000001/model.py:L25-L301` | lamp shell helper L25-L47、mast + tube-spline braces L60-L130、split yoke arms + webs + bearing collars L132-L224、lamp head L226-L282、joints L284-L301 |
| S6 | `rec_searchlight_tower_b5336452915842dbb1abcfc3324fa716` | `data/records/rec_searchlight_tower_b5336452915842dbb1abcfc3324fa716/revisions/rev_000001/model.py:L31-L273` | desktop folding：base L31-L80、folding mast L82-L116、pan_carriage L118-L177、head L179-L232、fold(Y)+pan(Z)+tilt(-Y) joints L234-L273 |

## 槽位 + 候选模块表

### Slot A：root_support
| module_name | 5_star_source | model.py:Lx-Ly | seed=0 anchor | 结构特征 |
|---|---|---|---|---|
| `pole_mast` | S1 | `model.py:L47-L83` | **yes** | 立柱 + 底板 + 少量斜撑，顶部 pan bearing seat |
| `lattice_tower` | S4 | `model.py:L47-L123` | | 角柱 + 横撑 + 斜撑 + perforated 服务平台 |
| `tripod_legged` | S3 | `model.py:L53-L124` | | 短中心柱 + 3-4 外撑腿 + spreader |
| `exposed_pole_braced` | S5 | `model.py:L60-L130` | | 立柱 + tube-spline 斜撑，可见 bearing collars |
| `desktop_fold_base` | S6 | `model.py:L31-L116` | | 小底座 + fold(Y) mast，便携折叠 |

### Slot B：pan_stage（yoke）
| module_name | 5_star_source | model.py:Lx-Ly | seed=0 anchor | 结构特征 |
|---|---|---|---|---|
| `two_arm_box_yoke` | S1 | `model.py:L85-L132` | **yes** | turntable disk + 2 box yoke arms + cylinder bearings |
| `trunnion_yoke_mesh` | S3 | `model.py:L126-L155` | | 单 `TrunnionYokeGeometry` mesh yoke |
| `split_arm_collar_yoke` | S5 | `model.py:L132-L224` | | split yoke arms + webs + 明显 bearing collars |
| `turret_pan_stage` | S2 | `model.py:L269-L329` | | drum/turret pan stage + drive box |
| `fold_pan_carriage` | S6 | `model.py:L118-L177` | | desktop fold 后的小 pan carriage |

### Slot C：spotlight_head
| module_name | 5_star_source | model.py:Lx-Ly | seed=0 anchor | 结构特征 |
|---|---|---|---|---|
| `primitive_barrel_head` | S1 | `model.py:L134-L181` | **yes** | barrel + bezel + translucent lens + rear housing + 2 trunnions（全 primitive） |
| `lathe_shell_head` | S2 | `model.py:L331-L412` | | `LatheGeometry.from_shell_profiles` 中空灯壳 + reflector + bulb + carry handle |
| `tripod_reflector_head` | S3 | `model.py:L157-L205` | | barrel + reflector bowl + bulb + trunnion shaft |
| `serviced_shell_head` | S4 | `model.py:L143-L194` | | from_shell_profiles 灯壳 + 冷却带 + guard，配 lattice 塔尺度 |

### Slot D：auxiliary_mechanism
| module_name | 5_star_source | model.py:Lx-Ly | seed=0 anchor | 结构特征 |
|---|---|---|---|---|
| `none` | S1 | `model.py:L183-L210` | **yes** | 只有 pan + tilt |
| `service_structure` | S4 | `model.py:L306-L399` | | ladder + perforated deck + guardrail（高塔/桁架） |
| `mast_fold` | S6 | `model.py:L234-L273` | | base->mast fold(Y) 关节，便携折叠 |
| `adjust_knobs_handwheel` | S2 | `model.py:L269-L329` | | pan/tilt 调节 knob / elevation handwheel（CONTINUOUS/REVOLUTE 小旋钮） |

## 槽位图（slot graph）
pattern = `linear_chain`

```
[Slot A root_support]
      -- (optional) mast fold REVOLUTE, axis (0,1,0)  [Slot D mast_fold] -->
      -- pan CONTINUOUS/REVOLUTE, axis (0,0,1) -->
[Slot B pan_stage / yoke]
      -- tilt REVOLUTE, horizontal axis (0,-1,0) -->
[Slot C spotlight_head]
      -- (optional) FIXED service platform / small knob joints [Slot D] -->
[Slot D service_structure / adjust knobs]
```

主约束基准是 pan vertical axis 与 tilt trunnion line。yoke arm span、trunnion 高度、lamp head 长度/半径、lens 位置都从这两条基准派生。

## 部件（Parts）

### Slot A / root_support
| part | visual_count | 描述 | 来源 |
|---|---:|---|---|
| `base_tower` / `mast` / `tower_support` / `tripod` | ~3-80 | 立柱/桁架/三脚架/折叠底座静态根，顶部 pan bearing seat | S1 / S3 / S4 / S5 / S6 |

### Slot B / pan_stage
| part | visual_count | 描述 | 来源 |
|---|---:|---|---|
| `yoke_stage` / `pan_carriage` / `turntable` | ~3-20 | turntable disk + 2 yoke arms（或单 mesh yoke）+ trunnion bearings | S1 / S2 / S3 / S5 |

### Slot C / spotlight_head
| part | visual_count | 描述 | 来源 |
|---|---:|---|---|
| `lamp_head` / `spotlight_head` | ~4-30 | barrel/can 灯头 + front lens + bezel + reflector/bulb + rear housing + 2 trunnions | S1-S4 |

### Slot D / auxiliary_mechanism
| part | visual_count | 描述 | 来源 |
|---|---:|---|---|
| `service_platform` / `ladder` / `guardrail` | ~5-40 | 高塔服务平台/爬梯/护栏（可挂为本体 visual 或独立 FIXED part） | S4 |
| `mast`（fold 段） | ~3-6 | desktop 折叠 mast，base 与 pan stage 之间的 fold 件 | S6 |
| `pan_knob` / `tilt_knob` / `elevation_wheel` | ~2-6 | 可选方位/俯仰调节旋钮或手轮（小 REVOLUTE/CONTINUOUS） | S2 |

非可动装饰（cooling fins、cable gland、front guard bars、carry handle、铭牌）默认用 `parent.visual(...)` 挂在 lamp_head / yoke / mast 上，不建独立 part。

## 关节（Joints）
| 关节 | 类型 | parent_slot.part | child_slot.part | axis | range | 描述 | 来源 |
|---|---|---|---|---|---|---|---|
| `pan` / `azimuth` | CONTINUOUS or REVOLUTE | A.root_support | B.pan_stage | `(0,0,1)` | unlimited（CONT）或 `[-pi, pi]` / `±150°`（REV） | 方位角旋转，竖直轴，origin 在 mast 顶 bearing seat | S1-S6 |
| `tilt` / `elevation` | REVOLUTE | B.pan_stage | C.spotlight_head | `(0,-1,0)`（主），也有 `(1,0,0)`/`(0,1,0)` | `lower∈[-0.75,-0.20]`，`upper∈[0.65,1.10]`（上仰更大） | 灯头俯仰，axis 穿过两 yoke arm 的 trunnion line | S1-S6 |
| `mast_fold` | REVOLUTE | A.base | A.mast（fold 段） | `(0,1,0)` | `[0, 1.25]` | desktop 折叠 mast | S6 |
| `platform_fixed` | FIXED | A.mast | D.service_platform | n/a | n/a | 独立服务平台/爬梯结构件（仅当需要独立 frame 时） | S4 |
| `pan_knob_spin` / `tilt_knob_spin` | CONTINUOUS or REVOLUTE | B.pan_stage | D.knob/wheel | 水平或竖直 | 小幅或 unlimited | 可选方位/俯仰调节旋钮/手轮 | S2 |

## 参数范围汇总
| 参数 | 类型 | 取值范围 / 候选值 | 默认值 | 派生关系 | 来源 |
|---|---|---|---|---|---|
| `root_style` | enum | `pole_mast`, `lattice_tower`, `tripod_legged`, `exposed_pole_braced`, `desktop_fold` | `pole_mast` | Slot A module | S1/S3/S4/S5/S6 |
| `mast_section` | enum | `round`, `square` | `round` | pole/lattice 立柱截面 | S1/S5 |
| `pan_joint_type` | enum | `revolute`, `continuous` | `revolute` | 大型工业塔多 continuous | S1/S2 |
| `yoke_style` | enum | `two_arm_box`, `trunnion_yoke_mesh`, `split_arm_collar`, `turret`, `fold_carriage` | `two_arm_box` | Slot B module，受 root_style 约束 | S1/S2/S3/S5 |
| `head_shell_style` | enum | `primitive_cylinder`, `lathe_shell`, `cadquery_tube` | `primitive_cylinder` | Slot C 灯壳构造层级 | S1/S2/S4 |
| `head_aim_axis` | enum | `plus_x`, `plus_y` | `plus_x` | 决定 tilt axis 符号与 trunnion/lens 方向 | S1/S3 |
| `service_structure` | enum | `none`, `ladder`, `platform_deck`, `ladder_deck_rail` | `none` | Slot D，高塔/桁架 | S4 |
| `extra_mechanism` | enum | `none`, `mast_fold`, `pan_lock_pin`, `adjust_knobs`, `handwheel` | `none` | Slot D 附加 DOF | S2/S6 |
| `tower_height` | float | `[0.22, 5.2]` | `2.4` | base 顶到 pan 轴的高度 | S1/S4 |
| `mast_radius` | float | `[0.033, 0.30]` | `0.09` | round/square 立柱半径或半宽 | S1/S5 |
| `yoke_span` | float | `[0.18, 0.92]` | `0.30` | 两 arm 间距 = 灯头宽，trunnion line 长度 | S1/S5 |
| `pan_stage_height` | float | `[0.07, 0.98]` | `0.28` | turntable 顶到 trunnion 的高度 | S1/S5 |
| `head_length` | float | `[0.10, 0.92]` | `0.38` | barrel 长度 | S1/S2 |
| `head_radius` | float | `[0.034, 0.46]` | `0.12` | barrel 半径 | S1/S2 |
| `lens_radius` | float | `[0.037, 0.34]` | `0.10` | 前镜片半径，略小于 bezel | S1/S2 |
| `lens_alpha` | float | `[0.32, 0.80]` | `0.45` | lens 玻璃材质透明度 | S1/S2 |
| `tilt_lower` | float | `[-0.75, -0.20]` | `-0.45` | tilt 下限 | S1/S5 |
| `tilt_upper` | float | `[0.65, 1.10]` | `0.85` | tilt 上限 | S1/S5 |
| `has_reflector` | bool | `True/False` | `True` | 灯头内 reflector bowl + bulb | S3 |
| `has_carry_handle` | bool | `True/False` | `False` | 灯头顶 carry handle（spline tube） | S2 |
| `has_front_guard` | bool | `True/False` | `False` | 前 guard ring/bars | S4 |
| `cooling_fin_count` | int | `[0, 12]` | `0` | 灯壳冷却带数量 | S4 |

## Multiplicity / Copy Logic
- 模板级核心是 one root support, one pan stage, one spotlight head, optional one auxiliary structure/knob：无 template 级可变 N 复制核心件。
- Module-local 的 lattice 角柱（3-4）、tripod 腿（3-4）、yoke arms（2）、trunnion bearings（2）、cooling fins、guard bars、ladder rungs、bolt circles 是 helper 内固定/受 root_style 约束的循环，不暴露独立 `leg_count`/`fin_count` 之外的 template 级 N（`cooling_fin_count` 仅装饰）。
- tripod 腿固定 3-4 条结构腿，由 root_style 决定，不作为 sampled template-level N。

## 拓扑多样性审计
总组合数（gating 前）：5 root × 5 yoke × 3 head_shell × 4 auxiliary ≈ 300。


| slot | candidate_count | 是否 >=3 | 备注 |
|---|---:|---|---|
| root_support | 5 | yes | pole/lattice/tripod/exposed/desktop |
| pan_stage | 5 | yes | box yoke / trunnion mesh / split collar / turret / fold carriage |
| spotlight_head | 4 | yes | primitive / lathe / tripod-reflector / serviced shell |
| auxiliary | 4 | yes | none / service structure / fold / knobs-handwheel |

## 部件多样性审计（Part Diversity Audit）
| 部件类型 | observed_variation | 连续参数是否足够 | 是否需要离散参数 | 推荐参数 | 说明 |
|---|---|---|---|---|---|
| root support | pole(round/square) / lattice / tripod / exposed-braced / desktop-fold | no | yes | `root_style`, `mast_section` | contact plane 与 part tree 差异大 |
| pan stage / yoke | two-arm box / single trunnion mesh / split-collar / turret / fold carriage | no | yes | `yoke_style` | trunnion 支撑几何不同 |
| spotlight head shell | primitive cylinder / lathe hollow can / cadquery tube | no | yes | `head_shell_style` | 灯壳拓扑/截面不同 |
| lens / reflector | lens 恒有（透明），reflector/bulb 可选 | mostly | reflector/bulb = bool | `lens_radius`, `lens_alpha`, `has_reflector` | lens 是最强身份信号 |
| tilt 行程 | 连续上下限，上仰更大 | yes | no | `tilt_lower`, `tilt_upper` | 同拓扑角度变化 |
| service / handle / fins | 高塔有 ladder/deck/rail；fins/handle 数量级变化 | no（结构）/ yes（数量） | yes（service）/ bool | `service_structure`, `has_carry_handle`, `cooling_fin_count` | 高塔结构件改变 part tree |
| extra mechanism | none / fold / lock pin / knobs / handwheel | no | yes | `extra_mechanism` | 增加 part/joint |

## 组合逻辑（Composition Logic）
1. `root_style` 确定地面/桌面 contact plane、立柱截面与 pan 竖直轴位置；`tower_height` 给出 pan 轴高度。
2. `pan` 关节 origin 落在 mast 顶 bearing seat（xy=0），`pan_joint_type` 决定 CONTINUOUS 还是限幅 REVOLUTE。
3. `yoke_style` 在 pan stage 上按 `yoke_span` 对称放两 arm（或单 mesh yoke），`pan_stage_height` 决定 trunnion line 高度。
4. `tilt` 关节 origin 落在 trunnion line 上（两 arm 之间），axis 与 `head_aim_axis` 一致（aim +X -> tilt (0,-1,0)）。
5. `lamp_head` 的 barrel/lens/bezel/reflector 沿 aim 方向从 trunnion 派生；front lens 在 barrel 前端，bezel 略大包住 lens。
6. trunnion shaft/pins 与 yoke bearings 之间的嵌入用 scoped `allow_overlap`（near-universal idiom）。
7. service platform/ladder 从 mast 高度派生并贴塔身；fold mast 在 base 与 pan stage 之间插入 fold(Y) 关节；调节 knob/handwheel 贴 pan stage 侧面。

## 已有模板写法参考
`cctv_mast_with_pantilt_camera_head` / `parabolic_dish_on_azimuth_elevation_mount` / `monitor_mount` / `rotary_post` / `revolute_hinge`（仅参考代码组织、joint metadata 与 trunnion `allow_overlap` 写法）

## 约束
- 必须有 pan（竖直 Z 轴）与 tilt（水平轴）两级运动，链序 root -> pan_stage -> head，不允许灯头直接漂浮。
- 灯头必须有朝光束方向的 translucent front lens + bezel；reflector/bulb 可选但推荐。
- tilt 关节 origin 必须落在两 yoke arm 的 trunnion line 上，不在空中。
- `head_aim_axis` 与 tilt axis 符号、trunnion 朝向、lens 位置必须一致（正向 tilt 抬升光束）。
- root support 必须按 root_style 接触地面/桌面平面。
- lattice/桁架塔的 ladder/deck/rail 必须贴塔身并从 mast 高度派生。

## Validator
| 检查项 | 标准 |
|---|---|
| identity | spotlight barrel + translucent lens + bezel + mount 同时存在 |
| pan joint | root -> pan_stage 绕 `(0,0,1)`，CONTINUOUS 或 REVOLUTE |
| tilt joint | pan_stage -> head 绕水平轴 REVOLUTE，origin 在 trunnion line |
| chain order | root -> pan_stage -> head，无直接漂浮灯头 |
| hinge placement | tilt origin 介于两 yoke arm 之间 |
| lens | lamp_head 含半透明 lens visual，aim 方向正确 |
| diversity | `root_style` / `yoke_style` / `head_shell_style` 分支均被覆盖 |
| aux | service structure / fold / knobs 正确挂载且不替换主 pan/tilt 关节 |

## Reject cases
- 灯头固定不动，没有 pan/tilt 两级运动。
- 没有 translucent lens / bezel，灯头退化成普通 box/cylinder，失去类别身份。
- tilt axis 竖直或 pan axis 水平（轴向错误）。
- 灯头脱离 yoke 漂浮，或 trunnion 不在两 arm 之间。
- 退化成 parabolic dish / camera pan-tilt / 纯塔身（身份丢失）。
- lattice/桁架 ladder/deck 悬空，不接触塔身。
- desktop fold mast 折叠后灯头穿模或脱离 pan carriage。

## 审核记录
| 项 | 结论 |
|---|---|
| reviewer status | approved |
| reviewer notes | 用户审核通过，进入 TEMPLATE_AFTER_REVIEW |
