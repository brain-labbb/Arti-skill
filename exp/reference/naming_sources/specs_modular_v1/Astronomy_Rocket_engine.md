# Modular Spec - Astronomy / Rocket engine

## 元信息
| 项 | 值 |
|---|---|
| slug | `Astronomy_Rocket_engine` |
| template path | `agent/templates/Astronomy_Rocket_engine.py` |
| test path (optional) | `tests/agent/test_Astronomy_Rocket_engine_template.py` |
| stage | `SPEC_ONLY_DRAFT` |
| status | `pending` |
| __modular__ | `True` |
| pattern | `linear_chain` (root thrust structure -> gimbal -> engine) + 3 multiplicity/feature axes |
| function stem | `astronomy_rocket_engine` (exports `build_astronomy_rocket_engine`, `config_from_seed`, `run_astronomy_rocket_engine_tests`) |

`pattern = linear_chain`: a serial mechanical spine `thrust_structure (root) ->
gimbal -> engine`, but joints are emitted MANUALLY per slot (parallel-children
idiom: each slot declares only a `downstream` re-export, no auto chain joint),
because the gimbal choice changes both the engine's PARENT part and its mount
joint (two-axis inserts an intermediate `gimbal_cross`; single-axis parents the
engine directly to the bulkhead). Three axes ride on top: nozzle multiplicity
`chamber_count` (1 vs 4-chamber cluster), a `prismatic_feature`
(none / deployable nozzle skirt / TVC jack), and decoration counts
(`strut_count`, `actuator_count`).

## 5 星样本阅读摘要
| 项 | 值 |
|---|---|
| five_star_total | 11 |
| read_count | 11 |
| read_scope | all 5-star samples in this subclass (origin 母本 + 10 forked variants) |
| source_index_policy | only adopted module sources are indexed below; all 11 were read in full |

Samples (all `collections=["workbench"]`, `rating=5`):

- `rec_a-stylized-liquid-fuel-rocket-engine-assembly-sh_...1fe2d6b0` - ORIGIN 母本 (solid disc bulkhead, two-axis gimbal cross, egg chamber + bell nozzle, 4 struts, 3 actuators).
- `rec_rocket_engine_var_truss_thrust_frame` - ③ root form: solid disc -> open TorusGeometry ring + radial spokes + diagonal braces.
- `rec_rocket_engine_var_single_axis_gimbal` - ①+②: two-axis cross (2 REVOLUTE) -> single-axis trunnion yoke (1 REVOLUTE), yoke fused as bulkhead visuals, engine parents directly to bulkhead.
- `rec_rocket_engine_var_conical_nozzle` - ③ nozzle form: curved bell lathe -> straight linear-taper cone lathe.
- `rec_rocket_engine_var_aerospike_nozzle` - ③ nozzle form: bell -> truncated annular aerospike (central spike plug lathe + annular shroud + cooling ridges).
- `rec_rocket_engine_var_cylindrical_chamber` - ③ chamber form: egg ellipsoid -> cylindrical barrel with domed injector head (lathe profile).
- `rec_rocket_engine_var_nozzle_cluster` - ① multiplicity: single chamber+bell -> 4 scaled chamber+throat+bell subunits in a radial cluster (all visuals in engine_assembly).
- `rec_rocket_engine_var_deploy_nozzle_prismatic` - ①+② prismatic: adds a `nozzle_extension` skirt part on a PRISMATIC `nozzle_deploy` joint (telescoping deploy).
- `rec_rocket_engine_var_prismatic_tvc_actuator` - ②+① prismatic: actuator 0 becomes a real translating `tvc_actuator` rod part inside a fixed barrel on a PRISMATIC `tvc_extend` joint.
- `rec_rocket_engine_var_strut_count` - multiplicity: `STRUT_ANGLES` 4 -> 6 gimbal struts.
- `rec_rocket_engine_var_actuator_count` - multiplicity: `ACTUATOR_ANGLES` 3 -> 2 actuators.

## 核心身份

A **stylized liquid-fuel rocket engine on a thrust-vectoring gimbal mount**,
built vertically nozzle-down: a **thrust structure** at the top (a solid gray
bulkhead disc, or an open truss ring frame) with rim bolts, riveted converging
struts and spring TVC actuators, from which the engine **powerhead** hangs on a
**gimbal** (either a two-axis gimbal cross giving pitch+yaw, or a single-axis
trunnion yoke giving pitch only). The powerhead is a white ellipsoid or
cylindrical **combustion chamber** with a riveted seam and clamp collar, a black
corrugated **throat**, a clamp ring, and a big black ribbed **nozzle** (bell /
conical / annular aerospike), plus a small turbopump and feed pipe. The
category-defining joint is ALWAYS the gimbal (a REVOLUTE that vectors the
nozzle); an optional PRISMATIC deployable nozzle skirt or TVC jack may be
present. Multi-chamber (RD-170 style) clusters carry N powerhead subunits under
one gimbal. Default mature domain: a 1-2 m-tall single engine, gimbal +/-0.12 rad.

Not to be confused with the neighbouring **whole launch vehicle / rocket stage**
(the engine is only the thrust chamber + mount, not a fueled airframe), nor with
a generic **industrial gimbal / pan-tilt head** (this object reads as a rocket
nozzle assembly with a chamber, throat and bell, not a camera mount).

## 槽位 + 候选模块表

### Slot A: thrust_structure (root · ③ Primary Form Family)

The root part `thrust_bulkhead`. Same part tree/interface across candidates: a
top cap (solid disc or open ring) + 8 `rim_bolt` + `gimbal_hub_stub` + N
`gimbal_strut` + M actuators, all fused as `thrust_bulkhead` visuals (Rule 1).
Only the top-cap prototype changes; both expose the identical gimbal-pivot
mounting datum at `(0,0,PIVOT_Z)` so the gimbal slot is form-independent.

| module_name | source_type | source evidence | model.py:Lx-Ly | sampling eligibility | 结构特征 / form_subtype |
|---|---|---|---|---|---|
| `disc_bulkhead` | forked_anchor (origin) | `rec_a-...1fe2d6b0` | L84-L168 | eligible | solid `Cylinder` bulkhead disc + rim lip + rim bolts + hub stub + struts + actuators. **Volumetric Envelope Form** |
| `truss_frame` | forked_anchor | `rec_rocket_engine_var_truss_thrust_frame` | L94-L192 | eligible | open `TorusGeometry` structural ring + N radial spokes + N diagonal braces + hub stub reaching the ring plane; same bolts/struts/actuators re-anchored to the ring. **Macro Surface Construction** (open lattice vs solid disc) |

Slot A has 2 candidates: the confirmed pool contains exactly two source-backed
thrust-structure forms (solid disc, open truss). Degrade-to-2 justified per
SPEC_TEMPLATE 4; no fabricated 3rd form.

### Slot B: gimbal (① skeleton + ② joint type)

The category-defining mechanism. Determines how many parts/joints sit between
the bulkhead and the engine.

| module_name | source_type | source evidence | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|---|
| `two_axis_gimbal` | forked_anchor (origin) | `rec_a-...1fe2d6b0` | L171-L193 (cross+pitch), L334-L342 (yaw) | eligible | emits `gimbal_cross` part (2 crossed pins) + `gimbal_pitch` REVOLUTE (bulkhead->cross, axis X); the engine mounts to the cross via a `gimbal_yaw` REVOLUTE (axis Y). 3-part spine, 2 revolute joints. |
| `single_axis_gimbal` | forked_anchor | `rec_rocket_engine_var_single_axis_gimbal` | L171-L210 (yoke visuals), L351-L360 (single joint) | eligible | trunnion yoke (collar + 2 arms + 2 bearings + pin) fused as bulkhead visuals (Rule 1, non-moving); engine mounts DIRECTLY to the bulkhead via one `gimbal_pitch` REVOLUTE (axis X). 2-part spine, 1 revolute joint. |

Slot B has 2 candidates: the pool contains exactly two source-backed gimbal
topologies. Degrade-to-2 justified; no fabricated 3rd.

### Slot C: engine (③ nozzle form + carries chamber/cluster/prismatic axes)

The powerhead `engine_assembly`. Candidate discriminator = **nozzle form** (the
most recognizable ③ prototype). Chamber form, cluster count and the deployable
skirt ride as config the engine factory reads.

| module_name | source_type | source evidence | model.py:Lx-Ly | sampling eligibility | 结构特征 / form_subtype |
|---|---|---|---|---|---|
| `bell_nozzle` | forked_anchor (origin) | `rec_a-...1fe2d6b0` | L262-L293 | eligible | hollow curved `LatheGeometry` bell shell (closed wall loop, open exit) + cooling ribs + exit band. **Volumetric Envelope Form** |
| `conical_nozzle` | forked_anchor | `rec_rocket_engine_var_conical_nozzle` | L54-L55, L262-L274 | eligible | straight linear-taper cone `LatheGeometry` (2-point outer profile) replacing the curved bell; same ribs/band. **Volumetric Envelope Form** (straight vs curved skirt母线) |
| `aerospike_nozzle` | forked_anchor | `rec_rocket_engine_var_aerospike_nozzle` | L55-L63, L271-L330 | eligible | ① skeleton change of the exhaust end: central tapered `spike_plug` LatheGeometry + `annular_shroud` LatheGeometry ring + spike cooling ridges + bands, replacing the bell. **Macro Surface Construction** (plug/aerospike vs enclosed bell) |

硬约束满足: Slot C has 3 structurally distinct source-backed candidates;
Slots A/B degrade to 2 with justification (pool exhausted, no fabrication). All
candidates are `forked_anchor` with real `model.py:Lx-Ly`; no
`world_knowledge_extrapolation` candidate is used (all forms are source-backed).

## 槽位图（slot graph）

pattern: `linear_chain` (manual joints; parallel-children re-export idiom)

```
thrust_structure (root; disc_bulkhead / truss_frame)
   |
   |--[gimbal_pitch REVOLUTE(axis X, +/-gimbal_range) @ (0,0,PIVOT_Z); captured pin bearing]
   v
 gimbal          two_axis: gimbal_cross part ---[gimbal_yaw REVOLUTE(axis Y) @ cross origin]--> engine
                 single_axis: (no part; yoke = bulkhead visuals) --[gimbal_pitch REVOLUTE(axis X)]--> engine
   |
   v
 engine (engine_assembly; bell / conical / aerospike nozzle)
   +--[optional] nozzle_extension --[nozzle_deploy PRISMATIC(axis -Z, 0..0.30)]--> (telescoping skirt)
```

必须说明:
- slot 顺序/parent: `thrust_structure` is root. `gimbal` runs next; `two_axis`
  emits `gimbal_cross` + pitch joint (parent=bulkhead). `engine` runs last and
  emits its mount joint parenting to the gimbal-designated part (cross for
  two_axis via yaw REVOLUTE axis Y; bulkhead for single_axis via pitch REVOLUTE
  axis X). All slots declare only `downstream` (re-export the bulkhead pivot),
  so the assembler emits NO auto chain joint; each slot emits raw joints (same
  as every 5-star source, and the Satellite gold idiom).
- 接口点位: gimbal pivot at `(0,0,PIVOT_Z)` on the hub stub (disc) / hub stub at
  ring plane (truss). Engine mount hub / cross pins straddle that pivot
  (captured-pin bearing). nozzle_extension slides along -Z from the bell exit.
  tvc_actuator rod translates along the tilted actuator-0 axis on the bulkhead.
- 跨 slot joint type/axis/range: gimbal_pitch REVOLUTE(X, +/-gimbal_range<=0.20);
  gimbal_yaw REVOLUTE(Y, +/-gimbal_range) [two_axis only]; nozzle_deploy
  PRISMATIC(-Z, 0..0.30); tvc_extend PRISMATIC(actuator-axis, 0..0.08).
- 互斥/派生: `chamber_count=4` (cluster) forces `nozzle_form=bell_nozzle`,
  `chamber_form=egg`, and forbids `nozzle_extension` (source cluster is
  egg+bell, no skirt). `nozzle_extension` requires `nozzle_form in {bell,
  conical}` and `chamber_count=1` (a skirt telescopes over a bell/cone exit,
  not a spike). gimbal x thrust_structure x nozzle are otherwise orthogonal.

## 每槽位 Module Emits / Interfaces

### Slot A / module disc_bulkhead | truss_frame
| emits | 描述 | 来源 |
|---|---|---|
| parts | `thrust_bulkhead` (single root part) | origin L83 |
| visuals | disc: `bulkhead_disc`+`bulkhead_rim`; truss: `frame_ring`+`frame_radial_i`+`frame_diagonal_i`; both: `rim_bolt_i`x8 + `gimbal_hub_stub` + `gimbal_strut_i`x`strut_count` + `actuator_piston_i`/`actuator_spring_i_k`/`actuator_collar_i` x`actuator_count` (or tvc barrel) | origin L84-L168; truss L94-L192 |
| internal joints | none in `none`/`tvc`; `tvc_extend` PRISMATIC when prismatic_feature=tvc_actuator | tvc L225-L233 |
| downstream interface | `thrust_bulkhead` part, `gimbal_hub_stub` visual, face `negative_z`, anchor `(0,0,PIVOT_Z)` (informational; gimbal wires manually) | origin L107-L112 |

### Slot B / module two_axis_gimbal | single_axis_gimbal
| emits | 描述 | 来源 |
|---|---|---|
| parts | two_axis: `gimbal_cross`; single_axis: none (yoke fused into bulkhead) | origin L171; single L171-L210 |
| visuals | two_axis: `gimbal_pin_x`+`gimbal_pin_y`; single_axis: `yoke_collar`+`yoke_arm_i`+`yoke_bearing_i`+`trunnion_pin` on bulkhead | origin L172-L183; single L181-L210 |
| internal joints | two_axis: `gimbal_pitch` REVOLUTE(X) bulkhead->cross | origin L185-L193 |
| downstream interface | re-export bulkhead pivot (passthrough) | - |

### Slot C / module bell_nozzle | conical_nozzle | aerospike_nozzle
| emits | 描述 | 来源 |
|---|---|---|
| parts | `engine_assembly`; optional `nozzle_extension` | origin L196; deploy L369-L370 |
| visuals | `mount_hub`+`mounting_plate`+chamber(egg/cyl)+`chamber_seam`+`throat_collar`+`throat_corrugation_i`x6+clamp ring+nozzle(bell/cone/spike+shroud+ridges)+bands+turbopump(bracket/body/manifold)+`feed_pipe`; cluster: 4x scaled `chamber_i`/`throat_*_i`/`nozzle_bell_i` subunits | origin L199-L332; cyl L213-L229; aerospike L271-L330; cluster L95-L185 |
| internal joints | engine mount joint (`gimbal_yaw` REVOLUTE Y to cross, or `gimbal_pitch` REVOLUTE X to bulkhead); optional `nozzle_deploy` PRISMATIC | origin L334-L342; single L351-L360; deploy L431-L442 |
| upstream interface | none declared (manual joint to gimbal-designated parent) | - |
| downstream interface | re-export bulkhead pivot (passthrough) | - |

活动件语义: gimbal REVOLUTE vectors the nozzle; nozzle_deploy PRISMATIC
telescopes the skirt down; tvc_extend PRISMATIC jacks the actuator rod. All
non-moving detail (bolts, struts, static actuators, yoke, ribs, seams, bands,
turbopump, feed pipe, cluster subunits) are host part visuals, not separate
parts (Rule 1). Captured gimbal pin / trunnion / telescoping slider overlaps
use element-scoped `allow_overlap` (Rule 2 grandfathered exceptions, exactly as
every source declares).

## 参数范围汇总
| 参数 | 类型 | 取值范围 / 候选值 | 标称默认 | 约束类型 | 约束 / 函数 | 来源 |
|---|---|---|---|---|---|---|
| `thrust_form` | enum | disc_bulkhead / truss_frame | disc_bulkhead | choice | procedural sampler | Slot A |
| `gimbal_form` | enum | two_axis_gimbal / single_axis_gimbal | two_axis_gimbal | choice | procedural sampler | Slot B |
| `nozzle_form` | enum | bell_nozzle / conical_nozzle / aerospike_nozzle | bell_nozzle | choice | procedural sampler | Slot C |
| `chamber_form` | enum | egg / cylindrical | egg | conditional | forced to `egg` when `chamber_count==4` | origin L213, cyl L213-L223 |
| `chamber_count` | int | {1,4} (obs: 1 origin, 4 cluster) | 1 | conditional | 4 -> bell+egg, no extension | origin, cluster L57-L58 |
| `prismatic_feature` | enum | none / nozzle_extension / tvc_actuator | none | conditional | extension gated to bell/cone + count 1; else -> none/tvc | deploy L431, tvc L225 |
| `strut_count` | int | {4,6} (obs: 4 origin, 6 strut_count) | 4 | conditional | radial gimbal struts (decoration) | origin L37, strut_count L37 |
| `actuator_count` | int | {2,3} (obs: 3 origin, 2 actuator_count) | 3 | conditional | radial spring actuators (decoration) | origin L38, actuator_count L38 |
| `gimbal_range` | float | [0.08, 0.20] | 0.12 | independent | uniform, clamp; revolute gimbal +/- range (rad) | origin L192 |
| `disc_radius_scale` | float | [0.85, 1.15] | 1.0 | independent | uniform, clamp; scales top-cap radius | origin L34 |
| `engine_scale` | float | [0.85, 1.15] | 1.0 | independent | uniform, clamp; scales chamber+nozzle radial profile (保形; z-positions fixed) | origin L213-L277 |
| (-) | constraint | - | - | inequality | cluster subunit radius x offset must clear the thrust axis; `engine_scale` clamped so a K=4 cluster stays within the plate footprint | cluster L54-L58 |

所有 equation/inequality/conditional 在 `resolve_config` 内求解；builder 不失败。

## 7.5 编译预算 / compile budget（必填）

**Per-seed compile budget: <= 20 s** (hang-guard `--compile-timeout 60`).
Meshes: chamber lathe (56 seg), bell/cone/spike lathe (64 seg), turbopump feed
pipe tube (14 radial), truss torus ring (20x48), optional extension shells (48-64
seg). Cluster reuses ONE profile per subunit at 40-48 seg. Tessellation tiers:
hero lathe bodies <=64 seg, small rings/ribs default cylinders, truss torus <=48.
No boolean sculpting. Expect 4-10 s/seed; downgrade seg counts first if over.

## Multiplicity / Copy Logic

**两根 multiplicity 轴 + 一根 optional-feature 轴**（各自加权采样、各自编入
`slot_choices`、各自 clamp）：

### 轴 1 - `chamber_count`（powerhead 燃烧室/喷管数）
- `count_param`: `chamber_count`; `N_range` product `{1,4}`, test `{1,4}`;
  sampling 加权 `{1: 0.75, 4: 0.25}`（单室常见，集束稀有）。
- copied object: 整个 chamber+seam+collar+corrugation+clamp+bell 子装配（scaled by
  `SUB_SR/SUB_SZ`），naming `chamber_{i}`/`throat_corrugation_{i}_{j}`/
  `nozzle_bell_{i}`/... placement: `CLUSTER_R` 半径、`CLUSTER_ANGLES` 均布。joint
  policy: 无新 joint（子件全是 engine_assembly 的 visual，整块随 gimbal 转）。
- source/gating: single origin L199-L332; cluster L95-L185。`chamber_count=4`
  强制 bell nozzle + egg chamber + no extension。

### 轴 2 - `strut_count` / `actuator_count`（推力结构装饰件数）
- `strut_count` {4,6} 加权 `{4:0.6, 6:0.4}`; `actuator_count` {2,3} 加权
  `{3:0.6, 2:0.4}`. copied object: `gimbal_strut_i` cylinders (radial, hub-ward)
  / actuator (piston+3 springs+collar) radial groups on the bulkhead. naming
  `gimbal_strut_{i}` / `actuator_piston_{i}` etc. placement: even angular spacing.
  joint policy: none（装饰 visual，除非 actuator 0 被 tvc 替换）。
- source/gating: origin (struts=4, act=3), strut_count (6) L37, actuator_count (2) L38。

### 轴 3 - `prismatic_feature`（可选 PRISMATIC 机构，② 覆盖）
- values `{none, nozzle_extension, tvc_actuator}` 加权 `{none:0.6,
  nozzle_extension:0.2, tvc_actuator:0.2}`。
- nozzle_extension: adds `nozzle_extension` part + `nozzle_deploy` PRISMATIC(-Z,
  0..0.30); gated to bell/cone + count 1 (deploy L369-L442).
- tvc_actuator: actuator 0 -> fixed barrel + `tvc_actuator` rod part on
  PRISMATIC `tvc_extend` (actuator axis, 0..0.08) (tvc L147-L233).

## 视觉多样性 6 轴考察

| 轴 | 怎么判断 | 有/无 | 取值/范围 + source_type / 来源 · 或理由 |
|---|---|---|---|
| ① 骨架图 | 加/减会动的 part 或一条边 | 有 | gimbal skeleton: two-axis cross (origin, +cross part +2 REVOLUTE) vs single-axis trunnion (single_axis_gimbal, no cross part, 1 REVOLUTE); nozzle end: enclosed bell/cone vs open aerospike plug+shroud (aerospike); optional `nozzle_extension` part (deploy) / `tvc_actuator` part (tvc). 全部 forked_anchor。 |
| └ multiplicity | 同构件 xN | 有 | 见 8: chamber_count {1,4} (origin/cluster); strut_count {4,6}; actuator_count {2,3}. |
| ② 关节类型 | 图不变，边换 type/轴 | 有 | REVOLUTE gimbal pitch(X)/yaw(Y) (origin), single trunnion REVOLUTE(X) (single_axis); PRISMATIC nozzle_deploy(-Z) (deploy) / tvc_extend (tvc). 全部 forked_anchor；每种类型都在 sweep 中出现。 |
| ③ 主体形态家族 | 图&关节不变，换核心 part 可识别形态原型 | 有 | **三处登记进 slot_choices**: (A) thrust structure - solid disc (origin, Volumetric Envelope) / open truss ring (truss_frame, Macro Surface Construction). (C) nozzle - curved bell / straight cone (Volumetric Envelope) / annular aerospike plug (aerospike, Macro Surface Construction). (chamber) egg ellipsoid / cylindrical barrel (cylindrical_chamber, Volumetric Envelope). |
| ④ 表面装饰 | 原型不变叠加表面细节 | 有 | `rim_bolt`x8, `chamber_seam`, `throat_corrugation`x6, `bell_rib`/`spike_ridge`, `exit_rim_band`/`shroud_band`, `gimbal_strut` count, `actuator` count - all host part visuals derived from the realized top-cap/nozzle surface (radius profile) so they hug the body across ③/⑤. source_type=record_only (origin/truss/aerospike/strut_count/actuator_count). |
| ⑤ 尺寸/行程 | 只连续改尺寸/比例/行程 | 有 | 连续 scale: disc_radius_scale[0.85,1.15], engine_scale[0.85,1.15], gimbal_range[0.08,0.20]. 关节运动包络: gimbal_pitch REVOLUTE axis X, 双向 open, [闭合 0, +/-gimbal_range<=0.20]; gimbal_yaw REVOLUTE axis Y same (two_axis only); nozzle_deploy PRISMATIC axis -Z, [0, 0.30] m (slides down/away, never toward body); tvc_extend PRISMATIC actuator-axis, [0, 0.08] m. `motion_test_plan`: `fail_if_parts_overlap_in_sampled_poses(max_pose_samples=48)`; targeted `ctx.pose` - pitch swings nozzle >0.05, yaw swings nozzle (two_axis), deploy slides skirt below bell exit, tvc extends rod. |
| ⑥ 涂装 | 只改材质/颜色 | 有 | 材质大类 metal(struct/disc/pipe gray) + painted(shell white / accent blue / bell black); >=6 colorway: `classic_blue_white`, `nasa_grey`, `soviet_green`, `copper_regen`, `matte_black_ops`, `test_stand_orange`. 材质大类覆盖 >= ceil(0.5x6)=3. |

**收尾自检**: 0-9 seed 渲染须肉眼见到 disc 与 truss 两种推力结构、bell/cone/aerospike
三种喷管、egg/cyl 两种燃烧室、单室与集束、two-axis 与 single-axis gimbal、材质配色多样、
gimbal/prismatic 全程不穿模。

## 采样与覆盖审计

总组合数（distinct slot-choice tuple 上界，去 N 不计连续）:
- thrust 2 x gimbal 2 x nozzle 3 = 12 base topology cells;
- x chamber_form 2 (single only) x chamber_count 2 x prismatic 3 (gated) approx 12 x ~8 = ~100 distinct realized tuples (report-only).

理由: 真实结构词汇收敛（所有样本共享同一「thrust structure + gimbal + 悬挂
powerhead」cell）；不硬凑组合空间（质量红线：不反推上游变体数量）。report-only，
不设 gate。

seed_domain_policy: `procedural_first`。

Procedural Sampling / Sweep Plan: `config_from_seed(seed)` 用 `random.Random(seed)`
依次抽 thrust_form、gimbal_form、nozzle_form、chamber_form、chamber_count、
prismatic_feature、strut_count、actuator_count、palette、连续 scale，再在
`resolve_config` 里按 compatibility gating 收敛非法组合。seed 0 pinned 到 origin
母本组合（disc_bulkhead + two_axis_gimbal + bell_nozzle + egg + 1 chamber +
none + 4 struts + 3 act, classic_blue_white）作为 documented regression anchor
（sparse override；其余 seed 全 procedural）。random sweep `0-15`(fast) ->
`0-35`(final) -> corner。

Topology target: 1000-seed slot-choice tuple 覆盖用于成熟度观察；真实上界约 100
（见上），低于 300 的原因为结构词汇收敛，已记录。report-only。

Controlled local parameterization: `disc_radius_scale`, `engine_scale`,
`gimbal_range`; 全部在 `resolve_config` clamp / 派生；不破坏 captured-pivot 接口、
gimbal 原点、multiplicity。连续尺寸契约: 先采 independent (disc/engine/gimbal) ->
无 equation 从属 -> conditional 解析 chamber_form/prismatic gating。

| item | policy | validator / viewer focus |
|---|---|---|
| sampler | slot 顺序 thrust->gimbal->nozzle，加权 choice；multiplicity/feature 各自加权 | slot_choices_for_seed 与 build 选择一致 |
| compatibility matrix | cluster->bell+egg+no extension；extension->bell/cone+count1；否则正交自由组合 | 无 floating / collision / 轴错误 / max-N / 可选子件失败 |
| controlled local variation | 3 个 clamp 连续 scale | 比例变化不破坏接口/clearance/support/joint 原点/类别身份 |
| regression overrides | seed 0 = origin 母本（documented anchor）；无其它 | 仅母本 canonical 预览 |
| random sweep | seeds 0-15 fast, 0-35 final, + corner | contract failures; axis_realization; viewer focus |

| slot | candidate_count | 是否 >=2 | 是否 >=3 | 备注 |
|---|---:|---|---|---|
| thrust_structure | 2 | yes | no | disc/truss；池仅 2 源形态，degrade 已说明 |
| gimbal | 2 | yes | no | two_axis/single_axis；池仅 2 源拓扑，degrade 已说明 |
| engine (nozzle) | 3 | yes | yes | bell/conical/aerospike |

## Validator

- `slot_choices_for_seed` returns implemented module names (+ chamber_form/chamber_count/prismatic/strut/actuator axes)
- `config_from_seed` uses deterministic procedural sampling for all ordinary seeds (seed 0 = documented 母本 override only)
- compatibility gating prevents illegal combos (cluster->bell+egg+no extension; extension->bell/cone+count1) in `resolve_config`
- controlled local scales clamped; cannot break captured-pivot interfaces, gimbal origin honesty, or multiplicity
- captured gimbal-pin / trunnion / telescoping-slider overlaps are element-scoped `allow_overlap` (not broad part-level)
- key joints have expected type/axis/range: gimbal REVOLUTE(X)[+Y two_axis]; nozzle_deploy/tvc_extend PRISMATIC
- copied `chamber_i` / `gimbal_strut_i` / `actuator_i` follow naming + placement policy
- `run_astronomy_rocket_engine_tests` calls `fail_if_parts_overlap_in_sampled_poses` + >=1 targeted `ctx.pose` per mechanism

## Reject cases

- Gimbal steered pose swings the nozzle bell up into the bulkhead/hub -> keep `gimbal_range<=0.20` and the pivot well above the chamber (nozzle hangs far below, low risk).
- 4-chamber cluster subunits overlap each other or overflow the mounting plate -> clamp `engine_scale`, keep `SUB_SR`/`CLUSTER_R` so subunits stay separated (radial offset > subunit radius).
- Deployable skirt is a disconnected floating part -> grandfather as a telescoping slider (`allow_isolated_part`/element-scoped `allow_overlap`) exactly per the deploy source; only for bell/cone count-1.
- Single-axis trunnion yoke arms/pin float off the bulkhead -> fuse them as bulkhead visuals contacting the hub stub (Rule 1); allow_overlap the pin<->mount_hub bearing.
- Truss frame swap leaves rim bolts/struts floating off the missing disc -> re-anchor them to the realized ring plane (Rule 4).
- Downgrading chamber/nozzle `LatheGeometry` or the truss `TorusGeometry` to crude Box/Cylinder placeholders (Rule 3 violation).

## 与相邻类别的边界

- 不该混入: **whole launch vehicle / rocket stage**（有燃料箱、级间段、整流罩的整机；本类别只是推力室+喷管+gimbal 安装）。
- 不该混入: **industrial pan-tilt head / camera gimbal**（无燃烧室/喉部/喷管钟形身份特征）。
- 不该混入: **jet turbine engine**（有风扇/压气机盘、进气道；火箭发动机为钟形喷管+燃烧室，无进气）。

## 审核记录
| 项 | 结论 |
|---|---|
| reviewer status | pending |
| reviewer notes | 11 confirmed samples (origin + 10 variants), all source-backed, no world_knowledge_extrapolation candidate. Slots A/B degrade to 2 candidates (pool exhausted); justified per SPEC 4. |

## 模板实现备注（可选）

- gimbal pivot height `PIVOT_Z` + hub-stub bottom single-sourced; extend the hub
  stub down to `<=PIVOT_Z` so the gimbal joint origin sits in real hardware
  (origin honesty, tol 0.020).
- Engine parent is resolved from `gimbal_form`: `gimbal_cross` (two_axis, yaw
  axis Y) or `thrust_bulkhead` (single_axis, pitch axis X). Emitted manually.
- All K cluster subunits share ONE scaled lathe profile per element to keep the
  compile budget; the whole cluster is a single `engine_assembly` part.
- Captured gimbal-pin / trunnion overlaps + telescoping slider -> element-scoped
  `allow_overlap` (Rule 2 exception), consistent with every 5-star source.
- 组装走 `_modular.assemble(..., selection_mode="anchor_choices")`: thrust root
  declares downstream; gimbal/engine declare only downstream (re-export) -> no
  auto chain joint, each slot emits raw joints (parallel-children idiom, same as
  Satellite gold + Tipping_Barrow).

## Module Source Index

| source_id | slot | module | sample_id | model.py 来源 | 采纳用途 |
|---|---|---|---|---|---|
| S1 | A/B/C | disc_bulkhead + two_axis_gimbal + bell_nozzle + egg | `rec_a-...1fe2d6b0` (origin 母本) | L83-L444 | root part tree, two-axis gimbal cross + 2 REVOLUTE, egg chamber + bell nozzle, all test semantics |
| S2 | A ③ | truss_frame | `rec_rocket_engine_var_truss_thrust_frame` | L94-L192 | open TorusGeometry ring + radial/diagonal truss |
| S3 | B ①② | single_axis_gimbal | `rec_rocket_engine_var_single_axis_gimbal` | L171-L210, L351-L360 | trunnion yoke (bulkhead visuals) + single REVOLUTE |
| S4 | C ③ | conical_nozzle | `rec_rocket_engine_var_conical_nozzle` | L54-L55, L262-L274 | straight linear-taper cone lathe |
| S5 | C ③ | aerospike_nozzle | `rec_rocket_engine_var_aerospike_nozzle` | L55-L63, L271-L330 | spike plug + annular shroud + ridges |
| S6 | C chamber ③ | cylindrical_chamber | `rec_rocket_engine_var_cylindrical_chamber` | L213-L223 | cylindrical barrel + domed head lathe |
| S7 | C mult | nozzle_cluster | `rec_rocket_engine_var_nozzle_cluster` | L53-L71, L95-L185, L332-L341 | 4x scaled chamber+throat+bell subunits |
| S8 | C prismatic | deploy_nozzle_prismatic | `rec_rocket_engine_var_deploy_nozzle_prismatic` | L56-L60, L299-L318, L369-L442 | deployable skirt part + PRISMATIC nozzle_deploy |
| S9 | A prismatic | prismatic_tvc_actuator | `rec_rocket_engine_var_prismatic_tvc_actuator` | L39, L132-L233 | TVC barrel + rod part + PRISMATIC tvc_extend |
| S10 | A mult | strut_count | `rec_rocket_engine_var_strut_count` | L37 | 6-strut multiplicity |
| S11 | A mult | actuator_count | `rec_rocket_engine_var_actuator_count` | L38 | 2-actuator multiplicity |
