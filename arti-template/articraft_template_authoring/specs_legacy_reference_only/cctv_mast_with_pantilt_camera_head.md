# Cctv Mast With Pantilt Camera Head Spec

## 元信息
| 项 | 值 |
|---|---|
| slug | `cctv_mast_with_pantilt_camera_head` |
| template path | `agent/templates/cctv_mast_with_pantilt_camera_head.py` |
| test path | `tests/agent/test_cctv_mast_with_pantilt_camera_head_template.py` |
| stage | `SPEC_ONLY_DRAFT` |
| status | `SPEC_ONLY_DRAFT` |

## 5 星样本阅读摘要
| 项 | 值 |
|---|---|
| five_star_total | 40 |
| read_count | 40 |
| read_scope | all 5-star samples in this category |
| source_index_policy | only adopted reusable snippets are indexed below |

## 核心身份
CCTV mast with pan-tilt camera head 必须有固定支撑结构（mast、pole、wall bracket、telescoping column 或 trailer mast）、pan head / yoke，以及带 lens 的 camera head；相机绕竖直 pan 轴转动，并绕水平 tilt 轴俯仰。

## 采用源码索引（Adopted Source Index）
| source_id | sample_id | model.py 来源 | 采纳用途 |
|---|---|---|---|
| S1 | `rec_cctv_mast_with_pantilt_camera_head_f99aa949bd6048e7b7b8c106cd62edd0` | `data/records/rec_cctv_mast_with_pantilt_camera_head_f99aa949bd6048e7b7b8c106cd62edd0/revisions/rev_000001/model.py:L26-L194` | freestanding mast, base plate, pan bearing, yoke, bullet camera, pan and tilt joints |
| S2 | `rec_cctv_mast_with_pantilt_camera_head_e57e41780fa24427841c699cba49c568` | `data/records/rec_cctv_mast_with_pantilt_camera_head_e57e41780fa24427841c699cba49c568/revisions/rev_000001/model.py:L67-L363` | telescoping column, side arm, pan head, bullet camera, extension / pan / tilt joints |
| S3 | `rec_cctv_mast_with_pantilt_camera_head_b26b755db716458ebef3c328eeb0a6c3` | `data/records/rec_cctv_mast_with_pantilt_camera_head_b26b755db716458ebef3c328eeb0a6c3/revisions/rev_000001/model.py:L25-L200` | corner wall bracket, triangular plate, extension arm, compact bullet PT head |
| S4 | `rec_cctv_mast_with_pantilt_camera_head_e0091eb9a1434d71ae71c6392fd01a33` | `data/records/rec_cctv_mast_with_pantilt_camera_head_e0091eb9a1434d71ae71c6392fd01a33/revisions/rev_000001/model.py:L54-L465` | mobile trailer base, multi-section telescoping mast, pan yoke and box camera |

## 部件（Parts）
| 部件 | 描述 | 参数 | 来源 |
|---|---|---|---|
| `support` / `mast` / `wall_plate` / `base_frame` | 必需；承载相机的固定支撑，可以是立杆、墙角支架、伸缩柱或拖车底盘 | `mount_layout`, `mast_profile`, `mast_height`, `base_style` | S1 / `model.py:L26-L75`; S2 / `model.py:L95-L181`; S3 / `model.py:L77-L116`; S4 / `model.py:L168-L244` |
| `side_arm` / `drop_rod` / `extension_arm` | optional / conditional；非顶装 mast 使用的横臂、吊杆或支架臂 | `arm_style`, `arm_length`, `brace_style` | S2 / `model.py:L182-L214`; S3 / `model.py:L83-L105`; S4 / `model.py:L168-L210` |
| `telescoping_stage_i` | optional；伸缩立柱或拖车 mast 的多级 prismatic stage | `telescoping_stage_count`, `extension_travel`, `mast_profile` | S2 / `model.py:L157-L181`; S2 / `model.py:L313-L326`; S4 / `model.py:L54-L156`; S4 / `model.py:L228-L303`; S4 / `model.py:L399-L440` |
| `pan_head` | 必需；位于支撑顶端或臂端的旋转底座、bearing、motor can、yoke bridge | `pan_head_style`, `yoke_width`, `bearing_radius` | S1 / `model.py:L77-L132`; S2 / `model.py:L215-L250`; S3 / `model.py:L117-L143`; S4 / `model.py:L305-L336` |
| `camera_head` / `camera` | 必需；带 lens 的 bullet、box 或 dome-ish camera | `camera_style`, `lens_style`, `sunshield_style` | S1 / `model.py:L133-L175`; S2 / `model.py:L252-L311`; S3 / `model.py:L144-L181`; S4 / `model.py:L337-L377` |
| `base_fasteners` / `bolts` / `gussets` | optional visual；基础螺栓、焊接加强板、支架三角板 | `fastener_count`, `brace_style` | S1 / `model.py:L51-L75`; S2 / `model.py:L108-L150`; S3 / `model.py:L25-L65`; S3 / `model.py:L107-L116` |
| `wheels` / `equipment_cabinet` | optional；mobile trailer layout 的固定附件 | `trailer_features` | S4 / `model.py:L168-L227`; S4 / `model.py:L378-L398` |

## 关节（Joints）
| 关节 | 类型 | axis | origin | range | 描述 | 来源 |
|---|---|---|---|---|---|---|
| `pan_joint` | revolute / continuous | `(0, 0, 1)` | mast top, arm tip, or pan socket center | `[-pi, pi]`, `[-2.6, 2.6]`, or continuous | pan head 绕竖直轴旋转 | S1 / `model.py:L177-L185`; S2 / `model.py:L334-L347`; S3 / `model.py:L183-L191`; S4 / `model.py:L441-L449` |
| `tilt_joint` | revolute | `(0, ±1, 0)` or `(1, 0, 0)` depending local yoke orientation | yoke trunnion center | approx `[-1.15, 0.90]` | camera head 在 yoke 内俯仰 | S1 / `model.py:L186-L194`; S2 / `model.py:L348-L361`; S3 / `model.py:L192-L200`; S4 / `model.py:L450-L463` |
| `mast_extension_i` | prismatic | `(0, 0, 1)` | parent tube opening | `0-0.96` by stage | telescoping mast 各级向上伸出 | S2 / `model.py:L313-L326`; S4 / `model.py:L399-L440` |
| `support_to_arm` | fixed / revolute | fixed or `(0,0,1)` when adjustable arm observed | pole collar or bracket hinge | fixed or limited swing | 支架臂连接到 mast / wall plate | S2 / `model.py:L327-L333`; S3 / `model.py:L77-L116` |
| `wheel_joint_i` | fixed | n/a | trailer axle | n/a | trailer 轮子作为固定可视附件，不作为核心运动 | S4 / `model.py:L378-L390` |

## 参数范围汇总
| 参数 | 类型 | 取值范围 / 候选值 | 默认值 | 派生关系 | 来源 |
|---|---|---|---|---|---|
| `mount_layout` | enum | `freestanding_pole` / `telescoping_column` / `corner_wall_bracket` / `mobile_trailer_mast` / `overhead_drop` / `side_arm_pole` | `freestanding_pole` | 决定 root support tree and optional extension joints | S1 / `model.py:L26-L194`; S2 / `model.py:L67-L363`; S3 / `model.py:L25-L200`; S4 / `model.py:L54-L465` |
| `mast_profile` | enum | `round_pole` / `square_tube` / `lattice` / `wall_plate` / `trailer_tube_stack` | `round_pole` | constrained by `mount_layout` | S1 / `model.py:L26-L75`; S2 / `model.py:L85-L181`; S4 / `model.py:L54-L156` |
| `camera_style` | enum | `bullet` / `box` / `dome` / `ptz_pod` | `bullet` | determines camera body, lens, sunshield / dome | S1 / `model.py:L133-L175`; S2 / `model.py:L252-L311`; S3 / `model.py:L144-L181`; S4 / `model.py:L337-L377` |
| `pan_head_style` | enum | `bearing_can` / `compact_socket` / `yoke_bridge` / `slew_bearing` | `bearing_can` | determines pan_head visual stack | S1 / `model.py:L77-L132`; S2 / `model.py:L215-L250`; S3 / `model.py:L117-L143`; S4 / `model.py:L305-L336` |
| `mast_height` | float | `0.45-4.5` | `1.5` | telescoping layouts add stage travel to collapsed height | S1 / `model.py:L33-L49`; S2 / `model.py:L78-L83`; S4 / `model.py:L35-L51` |
| `telescoping_stage_count` | int | `0-4` | `0` | only telescoping / trailer layouts create prismatic stages | S2 / `model.py:L157-L181`; S4 / `model.py:L228-L303` |
| `extension_travel` | float | `0.25-2.5` total | `0.55` | distributed over telescoping stages | S2 / `model.py:L313-L326`; S4 / `model.py:L399-L440` |
| `arm_style` | enum | `none` / `side_arm` / `triangular_corner_arm` / `drop_rod` / `braced_arm` | `none` | constrained by mount layout | S2 / `model.py:L182-L214`; S3 / `model.py:L83-L105` |
| `pan_range_mode` | enum | `limited` / `continuous` | `limited` | selects revolute limits or continuous pan | S1 / `model.py:L177-L185`; S4 / `model.py:L441-L449` |
| `tilt_range` | float pair | lower `-1.2--0.4`, upper `0.5-0.9` | `[-0.6, 0.55]` | tilt joint motion limits | S1 / `model.py:L186-L194`; S2 / `model.py:L348-L361`; S3 / `model.py:L192-L200` |
| `sunshield_style` | enum | `none` / `top_plate` / `three_sided` / `dome_cover` | `top_plate` | depends on `camera_style` | S1 / `model.py:L152-L157`; S2 / `model.py:L295-L300`; S3 / `model.py:L169-L181` |

## 部件多样性审计（Part Diversity Audit）
| 部件类型 | observed_variation | 连续参数是否足够 | 是否需要离散参数 | 推荐参数 | 说明 |
|---|---|---|---|---|---|
| support / mast | freestanding pole / telescoping column / corner wall bracket / mobile trailer / overhead mount / lattice mast | no | yes | `mount_layout`, `mast_profile` | 支撑拓扑完全不同，不能由高度连续变化表达 |
| pan head | bearing can / compact socket / yoke bridge / slew bearing | no | yes | `pan_head_style` | pan 基座形态影响连接和识别 |
| camera head | bullet / box / dome / PTZ pod | no | yes | `camera_style` | lens body 和外罩轮廓是核心类别差异 |
| arm / bracket | none / side arm / triangular corner arm / drop rod / braced arm | no | yes | `arm_style`, `brace_style` | 支架连接方式和力学形态不同 |
| telescoping mast | none / two-stage / four-stage square tube | no | yes | `telescoping_stage_count`, `mast_profile` | 有无伸缩链和 stage 数影响 part tree / joints |
| fasteners / gussets | bolts / welded webs / triangular gussets / guide pads | no | yes | `brace_style`, `fastener_count` | 主要作为支撑风格识别件；数量可连续 / int，形态需枚举 |
| lens / sunshield | bare lens / top shield / side shields / dome cover | no | yes | `sunshield_style`, `lens_style` | 摄像机头识别差异显著 |

## 组合逻辑（Composition Logic）
1. 先根据 `mount_layout` 生成 root support：pole、wall plate、base column、trailer frame 或 overhead bracket。
2. 若 `telescoping_stage_count > 0`，从 root support 向上生成 nested mast stages，每级 prismatic axis 为 `(0,0,1)`，保留插入长度和 guide pads。
3. 若需要 side arm / drop rod，作为 fixed child 或 root visual 连接到 mast / bracket 末端。
4. 在支撑顶端或臂端创建 `pan_head`，pan joint origin 放在 bearing / socket 中心，axis `(0,0,1)`。
5. 在 `pan_head` 中生成 yoke cheeks / bridge，在 trunnion center 放置 `camera_head`，tilt axis 水平。
6. 根据 `camera_style` 生成 bullet / box / dome geometry，lens 必须朝外，sunshield / dome cover 为 camera visual。
7. bolts、weld webs、guide strips、trailer cabinet、fixed wheels 等非核心活动件作为 visual 或 fixed accessory，不影响 pan-tilt 链。

## 已有模板写法参考
telescoping_tube / revolute_hinge / rotary_post / button_slider / prismatic_slide

## 约束
- 必须有 pan joint 和 tilt joint；pan 在 tilt 的父链上。
- pan axis 必须竖直 `(0,0,1)`；tilt axis 必须水平。
- camera head 必须包含 lens / front glass；lens 方向应远离支撑件。
- yoke cheeks / trunnions 必须与 tilt origin 对齐。
- telescoping stages 必须保持嵌套，不得伸出后脱离父 tube。
- wall / corner bracket layout 不应生成落地 pole；freestanding layout 不应漂浮在墙外。
- mobile trailer layout 的轮子可 fixed，但 mast 必须连接到 trailer deck。

## Validator
| 检查项 | 标准 |
|---|---|
| identity | support + pan_head + camera_head + visible lens 同时存在 |
| pan/tilt joints | 恰好至少 1 个 pan revolute/continuous 和 1 个 tilt revolute |
| axis check | pan `(0,0,1)`；tilt horizontal `(1,0,0)` 或 `(0,±1,0)` |
| joint chain | camera child 连接到 pan_head，pan_head 连接到 support / arm / top mast |
| yoke alignment | camera trunnion 位于 yoke cheeks 之间，tilt origin 靠近 trunnion center |
| telescoping retention | extended pose 下每级 mast 仍保留插入 overlap |
| part diversity | `mount_layout`, `mast_profile`, `camera_style`, `pan_head_style`, `arm_style` 存在 |
| no floating | bolts / arms / pan bearing / camera 均连接或接触父件 |

## Reject cases
- 没有 pan 或 tilt 其中任一关节。
- pan axis 不是竖直，或 tilt axis 不是水平。
- camera 没有 lens，像普通盒子。
- pan_head 漂浮在 mast / arm 外。
- telescoping mast 伸出后完全脱离父级。
- wall bracket、trailer、pole 等支撑布局混乱，无法识别 CCTV mast。
- 只做杆子加摄像头，没有 yoke / bearing / trunnion 等 pan-tilt 结构。

## 审核记录
| 项 | 结论 |
|---|---|
| reviewer status | pending |
| reviewer notes | 待人工审核；当前仅为 SPEC_ONLY_DRAFT，未进入模板实现阶段。 |
