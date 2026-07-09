# Drone Spec

## 元信息
| 项 | 值 |
|---|---|
| slug | `drone` |
| template path | `agent/templates/drone.py` |
| test path | `tests/agent/test_drone_template.py` |
| stage | `SPEC_ONLY_DRAFT` |
| status | `SPEC_ONLY_DRAFT` |

## 5 星样本阅读摘要
| 项 | 值 |
|---|---|
| five_star_total | 28 |
| read_count | 28 |
| read_scope | all 5-star samples in this category |
| source_index_policy | only adopted reusable snippets are indexed below |

## 核心身份
无人机类别必须表现为带推进器的无人载具：中心机身 / airframe、对称布置的臂或翼、多个 motor pod / propeller，以及 propeller 的连续旋转关节。5 星样本横跨 folding multirotor、hexacopter、tilt-rotor VTOL、带云台相机的 quadcopter、fixed-wing VTOL；这些主 topology 差异很大，成熟模板阶段应优先收窄到 `multirotor_drone`，或拆分为 multirotor / tiltrotor / fixed-wing VTOL 子模板。

采纳策略：默认模板应以 multirotor 为稳定 seed domain，把 folding arm、hex radial layout、landing gear、camera gimbal、propeller rotor 做成可兼容 helper。tiltrotor VTOL 和 fixed-wing VTOL 的 5 星样本可保留为 evidence 和 future split source；除非 reviewer 明确批准，否则不进入默认 seed。

## 采用源码索引（Adopted Source Index）
| source_id | sample_id | model.py 来源 | 采纳用途 |
|---|---|---|---|
| S1 | `rec_drone_456cea3ca6b74cfe9d77a06909bb21d2` | `data/records/rec_drone_456cea3ca6b74cfe9d77a06909bb21d2/revisions/rev_000001/model.py:L28-L47` | propeller blade mesh suitable for continuous rotor parts |
| S2 | `rec_drone_456cea3ca6b74cfe9d77a06909bb21d2` | `data/records/rec_drone_456cea3ca6b74cfe9d77a06909bb21d2/revisions/rev_000001/model.py:L50-L317` | folding arm visuals, body, arm / propeller parts, arm fold revolute joints and propeller spin joints |
| S3 | `rec_drone_456cea3ca6b74cfe9d77a06909bb21d2` | `data/records/rec_drone_456cea3ca6b74cfe9d77a06909bb21d2/revisions/rev_000001/model.py:L356-L477` | axis tests, propeller hub seating and folded-arm clearance checks for later validator mapping |
| S4 | `rec_drone_098da8da96264548b77a178b65dc143c` | `data/records/rec_drone_098da8da96264548b77a178b65dc143c/revisions/rev_000001/model.py:L27-L133` | hex frame constants, hex body profile, arm / prop meshes and radial arm specs |
| S5 | `rec_drone_098da8da96264548b77a178b65dc143c` | `data/records/rec_drone_098da8da96264548b77a178b65dc143c/revisions/rev_000001/model.py:L147-L338` | center frame, hex plates, skids, six folding arms, propellers and joints |
| S6 | `rec_drone_098da8da96264548b77a178b65dc143c` | `data/records/rec_drone_098da8da96264548b77a178b65dc143c/revisions/rev_000001/model.py:L393-L418` | validator pattern for arm hinge at plate rim and propeller tip radius |
| S7 | `rec_drone_5889264cec8a4b45931e9ccedb5c4a50` | `data/records/rec_drone_5889264cec8a4b45931e9ccedb5c4a50/revisions/rev_000001/model.py:L31-L190` | airframe rails, yokes, hinge points, tilt pods, propeller axles, tilt revolute joints and propeller continuous joints |
| S8 | `rec_drone_5889264cec8a4b45931e9ccedb5c4a50` | `data/records/rec_drone_5889264cec8a4b45931e9ccedb5c4a50/revisions/rev_000001/model.py:L212-L226` | joint axis tests for tilt pods and propellers |
| S9 | `rec_drone_5ae12ee0a7154e21baf462db7f024d39` | `data/records/rec_drone_5ae12ee0a7154e21baf462db7f024d39/revisions/rev_000001/model.py:L34-L109` | arm, propeller and landing gear tube meshes |
| S10 | `rec_drone_5ae12ee0a7154e21baf462db7f024d39` | `data/records/rec_drone_5ae12ee0a7154e21baf462db7f024d39/revisions/rev_000001/model.py:L111-L428` | body shell, nose, landing gear, arms, propellers, yaw / tilt / roll camera gimbal and joints |
| S11 | `rec_drone_d180b0d71c9e41199c73fc532f19fbf6` | `data/records/rec_drone_d180b0d71c9e41199c73fc532f19fbf6/revisions/rev_000001/model.py:L85-L394` | fixed-wing VTOL fuselage, wing, tail, nacelles, propellers, nacelle tilt and propeller spin joints |

## 部件（Parts）
| 部件 | 描述 | 参数 | 来源 |
|---|---|---|---|
| `airframe_body` / `central_hub` | 必需；中心机身、frame plate、fuselage 或主承载壳体 | `airframe_family`, `body_profile`, `body_length`, `body_width`, `body_height` | S2 / `model.py:L119-L205`; S4 / `model.py:L27-L133`; S5 / `model.py:L147-L222`; S7 / `model.py:L31-L110`; S10 / `model.py:L111-L160`; S11 / `model.py:L157-L213` |
| `arm_i` / `boom_i` | multirotor 条件必需；从中心 hub 径向伸出的承载臂，可固定或折叠 | `rotor_count`, `arm_layout`, `arm_length`, `arm_profile`, `folding_enabled` | S2 / `model.py:L50-L317`; S4 / `model.py:L92-L133`; S5 / `model.py:L227-L338`; S9 / `model.py:L34-L63`; S10 / `model.py:L190-L252` |
| `motor_pod_i` / `nacelle_i` | 必需；电机舱、rotor pod、tilt nacelle 或 wing-mounted pod | `motor_pod_style`, `pod_radius`, `tiltrotor_enabled` | S2 / `model.py:L207-L317`; S5 / `model.py:L227-L338`; S7 / `model.py:L126-L190`; S10 / `model.py:L190-L252`; S11 / `model.py:L215-L287` |
| `propeller_i` | 必需；连续旋转的螺旋桨 rotor，含 hub 和多片 blade | `propeller_radius`, `propeller_blade_count`, `propeller_style`, `propeller_clearance` | S1 / `model.py:L28-L47`; S2 / `model.py:L207-L317`; S4 / `model.py:L92-L133`; S5 / `model.py:L227-L338`; S7 / `model.py:L111-L190`; S9 / `model.py:L34-L63`; S10 / `model.py:L190-L252`; S11 / `model.py:L289-L394` |
| `landing_gear` / `skid_i` | optional but common；脚架、skid、tube gear 或 short feet | `landing_gear_style`, `ground_clearance`, `skid_span` | S5 / `model.py:L147-L222`; S9 / `model.py:L89-L109`; S10 / `model.py:L162-L188` |
| `camera_gimbal` / `payload` | optional；前下方云台相机，含 yaw / tilt / roll 关节 | `gimbal_enabled`, `gimbal_style`, `camera_size` | S10 / `model.py:L254-L428` |
| `wing` / `tail` | fixed-wing / VTOL 条件必需；机翼、尾翼、pylon | `wing_enabled`, `wing_span`, `tail_style` | S11 / `model.py:L85-L394` |
| `tilt_yoke` / `pod_yoke` | tiltrotor 条件必需；允许 nacelle 俯仰的支架或 yoke | `tiltrotor_enabled`, `tilt_axis_style`, `nacelle_tilt_range` | S7 / `model.py:L31-L190`; S11 / `model.py:L215-L394` |
| `fold_hinge_hardware` | folding multirotor 条件必需；臂根 hinge plates、pins、stops | `fold_hinge_style`, `fold_angle_limit` | S2 / `model.py:L50-L317`; S5 / `model.py:L227-L338`; S6 / `model.py:L393-L418` |
| `nav_lights` / `antenna` | visual；LED、GPS dome、antenna、labels | `detail_style`, `nav_light_count` | S2 / `model.py:L119-L205`; S5 / `model.py:L147-L222`; S10 / `model.py:L111-L160` |

## 关节（Joints）
| 关节 | 类型 | axis | origin | range | 描述 | 来源 |
|---|---|---|---|---|---|---|
| `propeller_spin_i` | continuous | rotor shaft axis, usually `(0, 0, 1)` for horizontal multirotor | motor pod center | unbounded | 每个 propeller 绕电机轴连续旋转 | S2 / `model.py:L263-L317`; S5 / `model.py:L227-L338`; S7 / `model.py:L182-L190`; S10 / `model.py:L349-L428`; S11 / `model.py:L349-L394` |
| `arm_fold_i` | revolute | local vertical or hinge pin axis at arm root | arm root hinge on body rim | `[0, fold_angle_limit]` | folding quad / hex 的臂从飞行展开位折到机身侧 | S2 / `model.py:L263-L317`; S3 / `model.py:L416-L477`; S5 / `model.py:L227-L338`; S6 / `model.py:L393-L418`; S10 / `model.py:L349-L428` |
| `nacelle_tilt_i` / `motor_tilt_i` | revolute | `(0, 1, 0)` or wing/pod lateral axis | yoke trunnion through nacelle | `[-1.57, 1.57]` or constrained VTOL range | tiltrotor nacelle 从垂直升力方向过渡到前飞方向 | S7 / `model.py:L159-L190`; S8 / `model.py:L212-L226`; S11 / `model.py:L349-L394` |
| `gimbal_yaw` | revolute | `(0, 0, 1)` | gimbal mount under nose | `[-1.2, 1.2]` | camera yaw around vertical mount | S10 / `model.py:L254-L428` |
| `gimbal_tilt` | revolute | `(1, 0, 0)` or camera pitch axis | camera side trunnions | `[-0.9, 0.35]` | camera tilts downward / forward | S10 / `model.py:L254-L428` |
| `gimbal_roll` | revolute | `(0, 1, 0)` or optical axis roll | camera barrel center | `[-0.45, 0.45]` | camera roll stabilization, optional | S10 / `model.py:L254-L428` |

## 参数范围汇总
| 参数 | 类型 | 取值范围 / 候选值 | 默认值 | 派生关系 | 来源 |
|---|---|---|---|---|---|
| `airframe_family` | enum | `quad_folding` / `hex_multirotor` / `flat_quad_gimbal` / `tiltrotor_vtol` / `fixed_wing_vtol` | `quad_folding` | 决定主承载件、joint chain and legal rotor layout | S2 / `model.py:L50-L317`; S5 / `model.py:L147-L338`; S7 / `model.py:L31-L190`; S10 / `model.py:L111-L428`; S11 / `model.py:L85-L394` |
| `seed_domain` | enum | `multirotor_core` / `tiltrotor_review_gated` / `fixed_wing_split_candidate` | `multirotor_core` | default seed 只采样 quad/hex/flat camera multirotor；VTOL / fixed-wing 需要审核或拆分 | S2 / `model.py:L50-L317`; S5 / `model.py:L147-L338`; S7 / `model.py:L31-L190`; S10 / `model.py:L111-L428`; S11 / `model.py:L85-L394` |
| `body_profile` | enum | `rounded_shell` / `hex_plate_stack` / `fuselage` / `rail_frame` | `rounded_shell` | constrained by `airframe_family` | S2 / `model.py:L119-L205`; S4 / `model.py:L40-L47`; S5 / `model.py:L147-L222`; S7 / `model.py:L31-L110`; S11 / `model.py:L157-L213` |
| `rotor_count` | int | `2 / 4 / 6 / 8` | `4` | fixed by family except experimental review variants; arms and propellers count must match | S4 / `model.py:L117-L133`; S5 / `model.py:L227-L338`; S10 / `model.py:L190-L252`; S11 / `model.py:L215-L394` |
| `arm_layout` | enum | `x_quad` / `plus_quad` / `hex_radial` / `wing_pylons` / `tilt_pods` | `x_quad` | determines rotor angles and symmetry plane | S2 / `model.py:L207-L317`; S4 / `model.py:L117-L133`; S5 / `model.py:L227-L338`; S7 / `model.py:L31-L190`; S11 / `model.py:L215-L394` |
| `body_length` | float | `0.22-1.15` | `0.38` | multirotor smaller, fixed-wing uses upper range | S2 / `model.py:L119-L205`; S5 / `model.py:L147-L222`; S10 / `model.py:L111-L160`; S11 / `model.py:L157-L213` |
| `rotor_ring_radius` | float | `0.32-1.2` | `0.55` | motor center radius from body center; drives arm length and prop clearance | S2 / `model.py:L207-L317`; S5 / `model.py:L227-L338`; S10 / `model.py:L190-L252` |
| `arm_profile` | enum | `rectangular_tube` / `tapered_boom` / `truss_rail` / `wing_pylon` | `rectangular_tube` | affects arm visual and hinge hardware | S2 / `model.py:L50-L117`; S7 / `model.py:L31-L110`; S9 / `model.py:L34-L63`; S11 / `model.py:L215-L287` |
| `folding_enabled` | bool | `true` / `false` | `true` | if true, add `arm_fold_i` and hinge hardware; otherwise arms fixed visuals | S2 / `model.py:L50-L317`; S3 / `model.py:L416-L477`; S5 / `model.py:L227-L338`; S10 / `model.py:L349-L428` |
| `fold_angle_limit` | float | `0.65-2.35` rad | `1.45` | clamped by body radius, adjacent arm clearance and folded footprint | S3 / `model.py:L416-L477`; S6 / `model.py:L393-L418`; S10 / `model.py:L349-L428` |
| `propeller_radius` | float | `0.08-0.32` | `0.16` | `<= 0.45 * nearest_motor_spacing - clearance` | S1 / `model.py:L28-L47`; S4 / `model.py:L27-L34`; S5 / `model.py:L227-L338`; S10 / `model.py:L190-L252`; S11 / `model.py:L289-L394` |
| `propeller_blade_count` | int | `2 / 3 / 4` | `2` | affects prop mesh; all propeller_i share blade count unless styled | S1 / `model.py:L28-L47`; S7 / `model.py:L111-L190`; S9 / `model.py:L34-L63` |
| `motor_pod_style` | enum | `round_motor_cap` / `ductless_pod` / `tilt_nacelle` / `wing_nacelle` | `round_motor_cap` | derived from family and tiltrotor flag | S2 / `model.py:L207-L317`; S7 / `model.py:L126-L190`; S11 / `model.py:L215-L394` |
| `landing_gear_style` | enum | `none` / `skid_pair` / `tube_legs` / `short_feet` | `skid_pair` | landing gear bottom plane defines ground contact | S5 / `model.py:L147-L222`; S9 / `model.py:L89-L109`; S10 / `model.py:L162-L188` |
| `gimbal_enabled` | bool | `true` / `false` | `true` | only for multirotor / camera drone variants | S10 / `model.py:L254-L428` |
| `tiltrotor_enabled` | bool | `true` / `false` | `false` | if true, nacelle tilt joints replace fixed multirotor motor pods | S7 / `model.py:L126-L190`; S11 / `model.py:L215-L394` |
| `wing_enabled` | bool | `true` / `false` | `false` | fixed-wing VTOL only; should be split or guarded by seed domain | S11 / `model.py:L85-L394` |

## 部件多样性审计（Part Diversity Audit）
| 部件类型 | observed_variation | 连续参数是否足够 | 是否需要离散参数 | 推荐参数 | 说明 |
|---|---|---|---|---|---|
| airframe body | rounded quad shell / hex plate stack / rail frame / fixed-wing fuselage | no | yes | `airframe_family`, `body_profile` | 主承载 spine 和坐标基准不同，需要离散 topology |
| arms / booms | folding X arms / radial hex arms / wing pylons / truss rails | no | yes | `arm_layout`, `arm_profile`, `folding_enabled` | 臂数量、折叠语义和父子链不同 |
| motor pods | fixed round motor / tilt nacelle / wing nacelle | no | yes | `motor_pod_style`, `tiltrotor_enabled` | nacelle 是否可 tilt 会改变 joint chain |
| propellers | two-blade / three-blade / wing prop / multirotor prop | no | yes | `propeller_blade_count`, `propeller_style` | blade count and shaft axis are visually / mechanically important |
| landing gear | skid pair / tube legs / short feet / none | no | yes | `landing_gear_style` | 接地支撑方式影响 no-floating validator |
| camera payload | no camera / fixed camera / 2-axis or 3-axis gimbal | no | yes | `gimbal_enabled`, `gimbal_style` | gimbal adds independent joint chain |
| wing / tail | none / fixed wing with tail / VTOL pylons | no | yes | `wing_enabled`, `tail_style` | fixed-wing VTOL 与 multirotor 不同，建议拆分 |
| propeller shaft geometry | same topology but radius/height changes | yes | no | `propeller_radius`, `pod_radius` | 同一 family 内可用连续参数派生 |

## 组合逻辑（Composition Logic）
1. 先由 `airframe_family` 决定主约束基准：multirotor 使用中心 hub + radial symmetry plane；tiltrotor 使用 rail/yoke + pod trunnion axis；fixed-wing 使用 fuselage spine + wing station。
2. `quad_folding` / `hex_multirotor`：采样 body envelope 与 `rotor_ring_radius`，从 body center 按 `rotor_count` 均匀派生 motor centers；`arm_length = rotor_ring_radius - body_mount_radius - pod_radius`。
3. `propeller_radius` 必须按最近 motor spacing 夹紧，保证 propeller tip 不与相邻 prop 或 body 相交。
4. folding arms 的 hinge origin 放在 body rim / plate edge；arm geometry 从 hinge point 指向 motor center，fold range 用 body clearance 和相邻 arm clearance 约束。
5. 每个 motor pod 的 propeller joint origin 必须与 pod shaft center 一致；propeller visual 以该 shaft 为局部轴，不得偏心。
6. landing gear 先确定 ground contact plane，再派生 skid/leg top attachment points，确保 body ground clearance 合理。
7. gimbal 挂在 nose / underside mount，yaw -> tilt -> roll -> camera 的父子链固定；camera 不得独立漂浮。
8. tiltrotor / fixed-wing VTOL 分支中，nacelle tilt axis 从 yoke/trunnion 派生，propeller shaft 随 nacelle 子件运动。
9. nav lights、antenna、screws、logos 作为 body 或 arm visual；不创建无语义 fixed parts。
10. 如果审核不拆分，`config_from_seed` 只能覆盖 `quad_folding`, `hex_multirotor`, `flat_quad_gimbal` 这些同族 multirotor；tiltrotor / fixed-wing 分支必须显式 feature-gated。
11. `resolve_config` 必须执行 airframe compatibility matrix：`quad_folding` 允许 `x_quad` / folding arms / gimbal / skid legs；`hex_multirotor` 允许 `hex_radial` / six arms / skid or short feet，不允许 wing/tail；`flat_quad_gimbal` 允许 fixed arms + gimbal，不允许 nacelle tilt；`tiltrotor_vtol` 需要 yokes + nacelle tilt，不允许 arm folding unless explicitly implemented；`fixed_wing_vtol` 需要 fuselage + wing/tail + wing nacelles，应默认 split。
12. adopted source 只能填充对应 topology helper；S11 fixed-wing fuselage 不得替换 multirotor body helper，S7 tilt pod 不得混入普通 quad motor pod，除非 family 已切换到 review-gated VTOL。

## 已有模板写法参考
radial_array / continuous_rotor / revolute_hinge / folding_link_chain / telescoping_tube / caster_wheel

## 约束
- 每个 drone 必须至少有 2 个 propeller spin joints；multirotor 默认 4 或 6 个。
- `rotor_count`、arm count、motor pod count、propeller count 必须一致，fixed-wing VTOL 的 wing nacelle count 也必须可解释。
- propeller spin axis 必须穿过 motor pod center，且与 propeller visual 局部轴一致。
- folding arm hinge 必须在 body rim / plate edge，不能在臂中段随机旋转。
- propeller disk 不得切入 body、相邻 propeller 或 landing gear。
- landing gear 若出现，必须接触 ground plane 并连接到 body / arms。
- gimbal 若出现，必须挂在 body underside / nose，yaw/tilt/roll axis 语义正确。
- fixed-wing / tiltrotor 分支应由 reviewer 批准或拆分，否则不要进入默认 seed domain。
- `seed_domain=multirotor_core` 时，`airframe_family` 只能是 `quad_folding`, `hex_multirotor`, `flat_quad_gimbal`。
- 不得为了覆盖所有 5 星样本，把 wing/tail、folding quad arms、tilt nacelles 和 camera gimbal 同时拼到一个默认 multirotor 上。

## Validator
| 检查项 | 标准 |
|---|---|
| identity | airframe body + symmetric propulsors + propeller continuous joints present |
| count consistency | `rotor_count == len(motor_pods) == len(propellers)` for multirotor families |
| seed domain | default seeds remain inside `multirotor_core`; review-gated VTOL/fixed-wing branches are not sampled accidentally |
| compatibility matrix | `airframe_family`, `arm_layout`, `motor_pod_style`, `tiltrotor_enabled`, `wing_enabled` are legal before build |
| radial symmetry | multirotor motor centers lie on configured radial layout around body center |
| propeller axis | each `propeller_spin_i` origin equals motor shaft center and axis matches shaft |
| propeller clearance | tip radius within nearest-neighbor and body clearance limits |
| arm hinge | folding arms hinge at body rim and folded range avoids body penetration |
| landing contact | landing gear bottom touches common ground plane; body remains above plane |
| gimbal chain | if enabled, yaw parent -> tilt parent -> roll/camera chain exists with expected axes |
| tiltrotor semantics | if enabled, nacelle tilt axis goes through yoke trunnions and propeller is child of nacelle |
| part diversity | `airframe_family`, `body_profile`, `arm_layout`, `motor_pod_style`, `landing_gear_style`, `gimbal_enabled` drive geometry |
| no floating | arms、pods、props、gear、gimbal、wing/tail all have clear parent/support |

## Reject cases
- 没有 propeller continuous joints，只是固定风扇装饰。
- propeller 数量与 arms / pods 数不一致。
- arm hinge 在臂中间或 body 内部，折叠时穿过机身。
- propeller 轴偏离 motor center，旋转像偏心风车。
- propeller radius 过大，与相邻 propeller 或 body 严重相交。
- landing gear 悬空或不连接 body。
- camera gimbal 漂浮在机身前方，没有 yaw / tilt parent chain。
- 把 fixed-wing、quad、ROV、speaker-like device 混成一个不可信无人机。
- S11 fixed-wing 或 S7 tiltrotor 片段没有经过 topology switch，就被强行适配成普通 quadcopter helper。

## 审核记录
| 项 | 结论 |
|---|---|
| reviewer status | pending |
| reviewer notes | SPEC_ONLY_DRAFT; waiting for human review. 建议审核时优先收窄为 multirotor 或拆分 VTOL 子模板。未进入模板实现阶段。 |
