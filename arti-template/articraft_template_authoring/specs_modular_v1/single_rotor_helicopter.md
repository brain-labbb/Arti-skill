# Single Rotor Helicopter Spec

## 元信息
| 项 | 值 |
|---|---|
| slug | `single_rotor_helicopter` |
| template path | `agent/templates/single_rotor_helicopter.py` |
| test path | `tests/agent/test_single_rotor_helicopter_template.py` |
| stage | `SPEC_ONLY_DRAFT` |
| status | `pending` |
| __modular__ | `True` |
| pattern | `mixed` |

## 5 星样本阅读摘要
| 项 | 值 |
|---|---|
| five_star_total | 2 |
| read_count | 2 |
| read_scope | all 5-star samples in this category; each record's `record.json`, active `revision.json`, `prompt.txt`, and `model.py` were read/scanned |
| samples_adopted_as_module_sources | 2 |
| samples_read_but_not_adopted | 0 |
| source_index_policy | only two 5-star sources exist; every slot falls back to 2 candidates with explicit reviewer caveat |

- adopted as module sources: `rec_single_rotor_helicopter_39863ea3a843412f8349168b699859ce`, `rec_single_rotor_helicopter_90637aef635c4429a1f1b432e4449e0a`.
- read but not adopted: none.

## 核心身份

Single rotor helicopter is an aircraft with one main rotor on a mast above the fuselage, a tail boom carrying a tail rotor, and landing gear/skids/wheels. Core motions are main rotor continuous spin about vertical mast axis and tail rotor continuous spin about a horizontal tail axis. Optional articulation includes cockpit/service/cabin doors, including hinged and sliding door families.

边界：
- 不包括 coaxial/multirotor/drone categories: exactly one main rotor disk and one tail anti-torque rotor family.
- 不混入 airplane: must have vertical-lift main rotor and tail rotor.

## 采用源码索引（Adopted Source Index）
| source_id | sample_id | model.py 来源 | 采纳用途 |
|---|---|---|---|
| S1 | `rec_single_rotor_helicopter_39863ea3a843412f8349168b699859ce` | `data/records/rec_single_rotor_helicopter_39863ea3a843412f8349168b699859ce/revisions/rev_000001/model.py:L66-L387` | fire utility skid airframe, five-blade main rotor, tail rotor, service and crew hinged doors |
| S2 | `rec_single_rotor_helicopter_90637aef635c4429a1f1b432e4449e0a` | `data/records/rec_single_rotor_helicopter_90637aef635c4429a1f1b432e4449e0a/revisions/rev_000001/model.py:L184-L587` | offshore transport wheeled airframe, main/tail rotors, cockpit hinges and sliding cabin door |

## 槽位 + 候选模块表

### Slot A：airframe_landing
| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| `fire_utility_skid_airframe` | `rec_single_rotor_helicopter_39863ea3a843412f8349168b699859ce` | L66-L197 | eligible if compatible | robust fuselage, skid landing gear, service-door frames |
| `offshore_wheeled_airframe` | `rec_single_rotor_helicopter_90637aef635c4429a1f1b432e4449e0a` | L184-L472 | eligible if compatible | long fuselage, wheeled landing gear, cockpit/cabin door cutouts |

### Slot B：main_rotor
| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| `five_blade_fire_rotor` | `rec_single_rotor_helicopter_39863ea3a843412f8349168b699859ce` | L198-L249, L328-L336 | eligible if compatible | five-blade rotor with mast/hub; CONTINUOUS vertical spin |
| `tall_mast_transport_rotor` | `rec_single_rotor_helicopter_90637aef635c4429a1f1b432e4449e0a` | L473-L500, L570-L578 | eligible if compatible | taller mast and rotor disk for transport helicopter |

### Slot C：tail_rotor
| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| `compact_tail_rotor` | `rec_single_rotor_helicopter_39863ea3a843412f8349168b699859ce` | L250-L279, L337-L345 | eligible if compatible | tail rotor near boom tip, horizontal axis |
| `fin_mounted_tail_rotor` | `rec_single_rotor_helicopter_90637aef635c4429a1f1b432e4449e0a` | L501-L527, L579-L587 | eligible if compatible | tail rotor on fin with transport-airframe proportions |

### Slot D：doors_service
| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| `service_and_crew_hinges` | `rec_single_rotor_helicopter_39863ea3a843412f8349168b699859ce` | L280-L387 | eligible if compatible | paired service doors plus crew access door, all REVOLUTE |
| `cockpit_hinges_and_cabin_slide` | `rec_single_rotor_helicopter_90637aef635c4429a1f1b432e4449e0a` | L459-L569 | eligible if compatible | two cockpit hinge doors plus PRISMATIC sliding cabin door |

> Reviewer caveat: all slots have only 2 candidates because the category has only two 5-star samples. This is the allowed fallback case; do not invent a third source family without new 5-star records.

## 槽位图（slot graph）

pattern: `mixed`

```text
[Slot A airframe_landing]
  ├── main_rotor_spin CONTINUOUS, axis Z --> [Slot B main_rotor]
  ├── tail_rotor_spin CONTINUOUS, axis X --> [Slot C tail_rotor]
  └── parallel REVOLUTE/PRISMATIC doors --> [Slot D doors_service]
```

## 部件（Parts）
| part | slot | visual_count | 描述 | 来源 |
|---|---|---:|---|---|
| `fuselage` / `airframe` | A | ~15-40 | cabin, tail boom, skids/wheels, doors cutouts, stabilizer details | S1/S2 |
| `main_rotor` | B | variable | central hub and 3-8 main blades | S1/S2 + reviewer-gated count extrapolation |
| `tail_rotor` | C | variable | fin/boom anti-torque rotor with 2-5 blades | S1/S2 + reviewer-gated count extrapolation |
| `service_door` / `cockpit_door` / `cabin_door` | D | ~0-3 each | hinged or sliding access doors | S1/S2 |

## 关节（Joints）
| 关节 | 类型 | parent_slot.part | child_slot.part | axis | range | 描述 |
|---|---|---|---|---|---|---|
| `main_rotor_spin` | CONTINUOUS | A.airframe | B.main_rotor | `(0,0,1)` | unbounded | main rotor disk spin |
| `tail_rotor_spin` | CONTINUOUS | A.airframe | C.tail_rotor | `(1,0,0)` | unbounded | tail rotor spin |
| `door_hinge_i` | REVOLUTE | A.airframe | D.door | vertical `(0,0,+/-1)` | bounded | cockpit/service/crew door |
| `cabin_door_slide` | PRISMATIC | A.airframe | D.cabin_door | longitudinal | short travel | sliding passenger cabin door |

## 每槽位 Module Emits / Interfaces

本 spec 是 modular v1 早期写法；每个 slot/module 的 emitted parts、internal joints、upstream/downstream interface 已在「槽位 + 候选模块表」「槽位图」「部件（Parts）」「关节（Joints）」和 adopted source index 中给出。模板实现阶段必须把这些信息逐 module 显式落成 `ModuleBuild`、`InterfaceSpec` 和 `MatingContract`，不能只按全局部件清单拼装。

| emits | 描述 | 来源 |
|---|---|---|
| parts / visuals | 以各 slot candidate 的结构特征和「部件（Parts）」表为准；不动装饰优先作为 parent visual | adopted source index + slot table |
| internal joints | 以「关节（Joints）」表和 slot graph 中的 joint type / axis / range 为准 | adopted source index + joints table |
| upstream interface | 来自 slot graph 中的 parent face、hinge line、socket、rail、axis、contact plane 或 support policy | slot graph + source snippets |
| downstream interface | 消费相邻 slot 的 mating point / axis / face；必须在实现中转成真实 `InterfaceSpec` | slot graph + source snippets |
| mating contracts | 每个 separate moving child 和跨 slot 连接必须有可见支撑路径；captured pin / shaft / bearing overlap 需要局部 allow-overlap 理由 | validator + reject cases |

## 参数范围汇总
| 参数 | 类型 | 取值范围 | 默认 | 派生关系 | 来源 |
|---|---|---|---|---|---|
| `airframe_style` | enum | `fire_utility_skid_airframe` / `offshore_wheeled_airframe` | `fire_utility_skid_airframe` | Slot A | S1/S2 |
| `main_rotor_style` | enum | `five_blade_fire_rotor` / `tall_mast_transport_rotor` | `five_blade_fire_rotor` | Slot B | S1/S2 |
| `tail_rotor_style` | enum | `compact_tail_rotor` / `fin_mounted_tail_rotor` | `compact_tail_rotor` | Slot C | S1/S2 |
| `door_style` | enum | `service_and_crew_hinges` / `cockpit_hinges_and_cabin_slide` / `none` | `service_and_crew_hinges` | Slot D | S1/S2 |
| `main_blade_count` | int | `3..8` | 5 | sampled blade multiplicity; default is source-backed but seed-independent | S1/S2 + reviewer-gated count extrapolation |
| `tail_blade_count` | int | `2..5` | 4 | sampled tail blade multiplicity; default is source-backed but seed-independent | S1/S2 + reviewer-gated count extrapolation |

## Multiplicity / Copy Logic

- `main_blade_count`: `N_range=3..8`, sampling domain=`all integers`; default / typical value is `N=5` from source-backed examples, but seed 0 is not special.
- The copied object is main-rotor blade geometry inside the Slot B `main_rotor` part. Blades do not get per-blade pitch joints in the current source-backed domain.
- Naming should use `main_blade_i` for named visuals when exposed. The single moving articulation remains `main_rotor_spin`.
- Placement is radial around the mast/hub in the rotor disk: blade `i` uses phase `i * 360° / main_blade_count`, and every blade root must connect to a visible hub/root socket.
- `tail_blade_count`: `N_range=2..5`, sampling domain=`all integers`; default / typical value is `N=4`. The copied object is tail-rotor blade/cuff/link visual geometry inside the Slot C `tail_rotor` part. The single moving articulation remains `tail_rotor_spin`, with radial placement around the horizontal tail axis.

## 拓扑多样性审计


| slot | candidate_count | 是否 >=3 | 备注 |
|---|---:|---|---|
| airframe_landing | 2 | no | only two 5-star samples exist |
| main_rotor | 2 | no | only two 5-star samples exist |
| tail_rotor | 2 | no | only two 5-star samples exist |
| doors_service | 3 | yes | includes `none`, but only two source families |

### Procedural Sampling / Sweep Plan

seed_domain_policy：procedural_first。`config_from_seed(seed)` 对普通 seed 使用 deterministic procedural sampling；`seed=0` 不特殊，不作为 anchor 或 reference seed。Sampling 先选择上游结构槽，再从 compatible 下游 candidate 集合中选择 module / multiplicity / module-local variant。

Compatibility matrix / gating：以「槽位图」「每槽位 Module Emits / Interfaces」「Validator」中定义的接口、joint 轴、支撑面、range 和互斥关系为准；不兼容组合必须在 sampler 或 `resolve_config` 中降级、重采样或拒绝，不能让 builder 后期失败。

Regression overrides：默认无。若未来 sweep 发现稳定失败组合，或 reviewer 指定固定回归样本，可以添加少量显式 regression seed，但必须写明 seed、组合和原因；不得用小型 curated / modulo 表作为主 seed domain。

Random sweep / viewer plan：首次模板验收跑 `uv run articraft template sweep-pipeline <slug>`，依赖 0、0-4、0-19、0-49 的 cumulative random seeds 检查 build、MatingContract、joint origin / axis / range、support、collision 和 。机械通过后 viewer 目检一小批随机 seed，重点看类别身份、比例、closed pose、bulky module、optional moving child、max multiplicity、captured-pin / bearing / hinge / rail 接口。


## Validator（author_run_tests 必须覆盖的点）
- Exactly one main rotor assembly with vertical continuous spin.
- Tail rotor must be mounted on tail boom/fin with horizontal spin axis.
- Airframe must carry landing gear/skids/wheels and tail boom; no floating rotors.
- Doors, if enabled, must sit in cabin/service cutouts and use correct hinge/slide axes.
- Main rotor blades must be evenly distributed and attached to hub.

## Reject cases（必须能识别并拒绝）
- Multirotor/drone layout or coaxial twin main rotors.
- Missing tail rotor or tail rotor axis vertical.
- Main rotor detached from mast or blades not attached to hub.
- Sliding/hinged doors floating outside fuselage.

## 与相邻类别的边界
- `drone`：multiple small rotors around a frame; no tail boom/tail rotor pair.
- `wind_turbine` / `ceiling_fan`：rotor-only devices without fuselage and tail boom.
- `airplane`：fixed-wing aircraft without main helicopter rotor.

## 审核记录
| 项 | 结论 |
|---|---|
| reviewer status | pending |
| reviewer notes | SPEC_ONLY_DRAFT with 2-source fallback; waiting for human review |
