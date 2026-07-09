# Satellite With Articulated Solar Panels Spec

## 元信息
| 项 | 值 |
|---|---|
| slug | `satellite_with_articulated_solar_panels` |
| template path | `agent/templates/satellite_with_articulated_solar_panels.py` |
| test path | `tests/agent/test_satellite_with_articulated_solar_panels_template.py` |
| stage | `IMPLEMENTED` |
| status | `built_source_limited` |
| __modular__ | `True` |
| pattern | `mixed` |

## 5 星样本阅读摘要
| 项 | 值 |
|---|---|
| five_star_total | 0 |
| read_count | 0 |
| read_scope | all records in this category were enumerated; there are no rating-5 records available |
| samples_adopted_as_module_sources | 0 |
| samples_read_but_not_adopted | 0 |
| blocking_reason | SPEC_ONLY requires real 5-star module sources; this category currently has only one rating-4 dataset record |

Non-5-star reference found but **not adopted as 5-star source**: `rec_a-realistic-model-of-a-satellite_20260402_161121_286240_c5af0a24` (rating 4), with `model.py` at `data/records/rec_a-realistic-model-of-a-satellite_20260402_161121_286240_c5af0a24/revisions/rev_000001/model.py`. It contains `bus`, `left_solar_wing`, `right_solar_wing`, and `high_gain_antenna`; solar wings are REVOLUTE on bus, antenna is FIXED. Because it is not 5-star and there is only one structural family, it cannot satisfy the normal SPEC_ONLY module-source contract.

## 核心身份

Target identity, pending source approval: a spacecraft bus with articulated solar-array wings. The main bus carries avionics/radiators/antenna hardware; one or more solar panels deploy from hinges on bus sides; optional high-gain antenna, sensor mast, engine/nozzle, or secondary booms are attached to the bus. The core motion is solar panel deployment/retraction around bus-mounted hinges; optional antenna articulation may be added only when supported by future 5-star sources.

边界：
- 不包括 ground parabolic dish mounts; those belong to `parabolic_dish_on_azimuth_elevation_mount`.
- 不混入 drone/aircraft: this is an orbital spacecraft bus, not an aerodynamic vehicle.

## Provisional Non-Adopted Source Index
| source_id | sample_id | rating | model.py 来源 | observed structure |
|---|---|---:|---|---|
| R4-1 | `rec_a-realistic-model-of-a-satellite_20260402_161121_286240_c5af0a24` | 4 | `data/records/rec_a-realistic-model-of-a-satellite_20260402_161121_286240_c5af0a24/revisions/rev_000001/model.py:L86-L360` | `bus` with two `solar_wing` parts, two REVOLUTE deployment hinges, and fixed high-gain antenna |

## 槽位 + 候选模块表

Normal candidate tables are intentionally **not filled** because every candidate must cite a real 5-star source. A future approved spec should target at least:

| intended slot | minimum candidate families needed | notes |
|---|---|---|
| `bus_body` | 3 | boxy communications bus, cylindrical/hexagonal bus, truss bus |
| `solar_array` | 3 | single-panel wing, multi-panel foldout, bilateral long wing |
| `antenna_or_payload` | 3 | fixed high-gain dish, articulated antenna boom, sensor mast/payload module |
| `deployment_topology` | 2-3 | bilateral hinges, four-wing cross, nested fold panel chain |

## 槽位图（slot graph）

pattern target: `mixed`

```text
[bus_body]
  ├── REVOLUTE deploy hinge(s) --> [solar_array_i]
  ├── optional REVOLUTE/CONTINUOUS --> [antenna_or_payload]
  └── fixed visuals: radiators, decks, engine nozzle, star tracker
```

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
| 参数 | 类型 | intended range | 默认 | 来源状态 |
|---|---|---|---|---|
| `bus_style` | enum | pending 5-star sources | pending | blocked |
| `solar_wing_count` | int | 2 / 4 / maybe 1 | 2 | blocked |
| `panel_segments_per_wing` | int | 1-4 | 2 | blocked |
| `antenna_style` | enum | none / fixed / articulated | pending | blocked |

## Multiplicity / Copy Logic

Blocked/provisional only; do not implement from the current source set.

- Future `solar_wing_count` would control how many solar-array wings are attached to the bus. The current non-adopted rating-4 reference shows bilateral `N=2`; `N=1` or `N=4` needs 5-star backing before it becomes a normal sampling option.
- Future `panel_segments_per_wing` would control repeated panel segments within each wing. A single-panel wing uses one REVOLUTE bus hinge; a segmented foldout wing would be a serial chain of `wing_i_segment_j` parts/joints only if future sources support that topology.
- Naming should distinguish wing-level and segment-level multiplicity, for example `solar_wing_i`, `solar_wing_i_deploy`, `solar_wing_i_segment_j`, and `solar_wing_i_segment_j_fold`.
- Placement must keep solar wings attached to visible bus-side hinge barrels or yokes. Bilateral wings should mirror across the bus; four-wing topology should distribute roots on supported bus faces.

## 拓扑多样性审计


| slot | candidate_count | 是否 >=3 | 备注 |
|---|---:|---|---|
| bus_body | 0 | no | no 5-star sources |
| solar_array | 0 | no | no 5-star sources |
| antenna_or_payload | 0 | no | no 5-star sources |

### Stage 1 / Stage 2 seed-domain plan

seed_domain_stage：stage1_coverage。当前 spec 的组合空间以「拓扑多样性审计」中的兼容 slot/module 组合为准；Stage 1 seed domain 应优先覆盖 seed=0 anchor、每个主要 slot candidate、最大 part/joint 数组合、bulky module、可选 moving child、captured-pin / bearing / hinge / rail 接口、以及最容易出现悬空、穿模、joint 轴错或 closed pose 不合理的组合。

Stage 1 high-risk coverage seed plan：

| seed/range | covered combo | risk type | viewer / validator focus |
|---|---|---|---|
| 0 | spec 标注的 seed=0 anchor module combination | regression anchor | 类别身份、baseline part tree、主 joint 语义 |
| 1-N | 覆盖各 slot 的非 anchor candidate 和 gated optional moving child | interface / axis / support | 悬空、穿模、joint origin、axis、range、closed pose |
| N+ | 覆盖最大 part count、bulky module、captured-pin / bearing / hinge / rail 组合 | clearance / mating contract | visible support path、allow-overlap 局部理由、viewer 比例 |

Stage 2 procedural target：所有 Stage-1 模板完成并通过 sweep/viewer 后，主体 `seed>0` 逻辑迁移为 unbounded deterministic procedural sampling；除 anchor、coverage 和 regression overrides 外，不得无限轮换少数 curated / modulo 组合表来冒充 dataset-scale seed domain。

### Stage 1 / Stage 2 seed-domain plan

seed_domain_stage：stage1_coverage。当前 spec 的组合空间以「拓扑多样性审计」中的兼容 slot/module 组合为准；Stage 1 seed domain 应优先覆盖 seed=0 anchor、每个主要 slot candidate、最大 part/joint 数组合、bulky module、可选 moving child、captured-pin / bearing / hinge / rail 接口、以及最容易出现悬空、穿模、joint 轴错或 closed pose 不合理的组合。

Stage 1 high-risk coverage seed plan：

| seed/range | covered combo | risk type | viewer / validator focus |
|---|---|---|---|
| 0 | spec 标注的 seed=0 anchor module combination | regression anchor | 类别身份、baseline part tree、主 joint 语义 |
| 1-N | 覆盖各 slot 的非 anchor candidate 和 gated optional moving child | interface / axis / support | 悬空、穿模、joint origin、axis、range、closed pose |
| N+ | 覆盖最大 part count、bulky module、captured-pin / bearing / hinge / rail 组合 | clearance / mating contract | visible support path、allow-overlap 局部理由、viewer 比例 |

Stage 2 procedural target：所有 Stage-1 模板完成并通过 sweep/viewer 后，主体 `seed>0` 逻辑迁移为 unbounded deterministic procedural sampling；除 anchor、coverage 和 regression overrides 外，不得无限轮换少数 curated / modulo 组合表来冒充 dataset-scale seed domain。

## Validator（future approved spec should cover）
- Bus must be the root spacecraft body and carry all non-moving radiators/decks/nozzles as visuals.
- Solar arrays must be separate articulated parts with visible hinge barrels/clevises on the bus.
- Deployment hinge axes must align with the panel root and keep panels clear of the bus.
- Optional antenna/payload must not be mistaken for ground dish infrastructure.

## Reject cases（必须能识别并拒绝）
- No articulated solar panels.
- Solar panels as floating slabs with no hinge/root yoke.
- Ground pedestal dish or drone/aircraft silhouette.
- A one-source template claiming module diversity without 5-star backing.

## 与相邻类别的边界
- `parabolic_dish_on_azimuth_elevation_mount`：ground/roof/wall az-el mount, not spacecraft bus.
- `drone` / `single_rotor_helicopter`：aerodynamic vehicles with rotors/landing gear, not orbital bus/solar arrays.

## 审核记录
| 项 | 结论 |
|---|---|
| reviewer status | implemented (source-limited) |
| reviewer notes | Originally `SOURCE_BLOCKED` (0 five-star sources, single rating-4 reference). Template was subsequently authored as a modular object anyway with 4 slots (`bus_body` / `solar_array` / `payload` / `thruster_pack`). `agent/templates/satellite_with_articulated_solar_panels.py` is registered in `cli/template.py` and `template sweep-pipeline` PASSes (4 stages / 50 seeds). 2026-06-28: fixed seed-0 disconnected-geometry islands in `bus` — pulled the 8 corner frame rails inward (`±0.56`→`±0.53`) to bite the deck edge and moved `star_tracker_stem`/`dome` into the `sensor_plinth` footprint `(-0.12, -0.12)`. Source coverage remains below the normal 5-star module-source contract; revisit slot diversity if/when rating-5 records appear. |
