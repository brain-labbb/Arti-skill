# Modular Spec — Law Enforcement_Protective Gear / Riot shield

## 元信息
| 项 | 值 |
|---|---|
| slug | `riot_shield` |
| template path | `agent/templates/riot_shield.py` |
| test path (optional) | — (tests inline in `run_riot_shield_tests`) |
| stage | `SPEC_ONLY_DRAFT` |
| status | `pending` |
| __modular__ | `True` |
| pattern | `mixed` (linear fold-panel chain via revolute hinge(s) + parallel handle/shutter children on a shared body) |

## 5 星样本阅读摘要
| 项 | 值 |
|---|---|
| five_star_total | 8 |
| read_count | 8 |
| read_scope | all 5-star samples in this subcategory (2 origins + 6 verified forks) |
| source_index_policy | only adopted module sources are indexed below |

## 核心身份

A riot shield is a hand-carried (or self-standing) body-shielding barrier used by
police / security for crowd control: a broad rigid or soft-armor panel presented
between the officer and a threat, larger than the torso footprint, held by a
bolt-on carry handle / forearm grip or self-standing on an A-frame, and — for the
panel forms — folding through one or more revolute fold hinges so it collapses for
transport. The mature domain covers four primary form families (rigid molded
polymer plate, soft ballistic-fabric slab, curved polycarbonate shell, round
convex dished disc), a fold-panel multiplicity (monolithic → quad-fold), an
optional gun-/vision-port shutter, and a small family of holds.

不该混入：ballistic vest / body-armor plate (worn, not carried; no shield panel);
medieval tower shield / buckler prop (decorative, no fold/grip hardware, wrong
proportions); tactical backpack; tent / display panel board.

## 槽位 + 候选模块表

### Slot A：body_form（③ Primary Form Family，登记进 `slot_choices`）

| module_name | source_type | source evidence | model.py:Lx-Ly | form_subtype | sampling eligibility | 结构特征 |
|---|---|---|---|---|---|---|
| `rigid_polymer_panel` | forked_anchor | rec_black-…4985a0ef (origin A) | L52-101 (`_front_panel_solid`), L161-226 (front part) | Planar Boundary Form | eligible if compatible | Bent molded plate authored as overlapping rotated `Box` visuals: mid plate + rearward top deflector bend + forward kick flare + 3 stiffening ribs; steel clamp hinge brackets + hinge pins as parent visuals; self-stands (no handle). |
| `soft_fabric_panel` | forked_anchor | rec_folding-…d961ac50 (origin B) | L123-174 (front part), L90-108 (`_add_binding`) | Planar Boundary Form | eligible if compatible | Thin fabric slab `Box` + charcoal edge binding (side/side/end) + fabric fold sleeve (`Cylinder`) wrapping the top hinge line; logo patch + grommets as parent visuals. |
| `curved_polycarbonate_shell` | forked_anchor | rec_riot_shield_var_form_curved | L101-193 (`_build_curved_shell_mesh`), L234-287 (front part) | Macro Surface Construction | eligible if compatible | Triangulated thin cylindrically-bowed `MeshGeometry` shell (double-walled outer/inner + caps); integral smoke viewport band + edge trim + molded ribs conformal to the curved surface; translucent. |
| `round_convex_disc` | forked_anchor | rec_riot_shield_var_form_round | L59-91 (`_build_disc_shell`), L133-169 (front part) | Volumetric Envelope Form | eligible if compatible | Shallow spherical-cap `LatheGeometry.from_shell_profiles` dished disc; dark rubber rim `TorusGeometry` band + painted unit ring; translucent. |

四个候选都是结构不同的可识别主体形态原型（planar plate / planar fabric slab / macro
curved shell / volumetric dished disc），不是缩放/换色。

### Slot B：fold_topology（① 骨架 / N multiplicity + ② gun-port 关节）

| module_name | source_type | source evidence | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|---|
| `monolithic` | forked_anchor | rec_riot_shield_var_form_curved / _form_round | L221-378 / L123-226 | eligible if compatible | Single rigid body, no fold structure (static_only); only the grip joint. n_panels=1. |
| `bifold` | forked_anchor | rec_black-…4985a0ef (rigid) · rec_folding-…d961ac50 (soft) | L253-265 (rigid fold) · L229-241 (soft fold) | eligible if compatible | Two panels + one revolute fold hinge (pin+clamp-bracket for rigid, fabric fold-sleeve for soft). n_panels=2. |
| `bifold_gunport` | forked_anchor | rec_riot_shield_var_mechanism_gunport | L181-204 (`_shutter_solid`), L311-323 (shutter_hinge) | eligible if compatible (rigid only) | bifold + a second revolute joint: a hinged gun-/vision-port shutter flap over the rear window (replaces the fixed perforated screen). |
| `trifold` | forked_anchor | rec_riot_shield_var_n3 | L112-158 (`_add_panel_body`), L258-273 (hinge loop) | eligible if compatible (soft only) | Three loop-emitted panels + two revolute fold hinges, accordion. n_panels=3. |
| `quadfold` | forked_anchor | rec_riot_shield_var_n4 | L129-183 (`_build_panel`), L295-316 (hinge loop) | eligible if compatible (soft only) | Four loop-emitted panels + three revolute fold hinges, accordion. n_panels=4. |

### Slot C：grip（① handle sub-tree + ② grip 关节）

| module_name | source_type | source evidence | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|---|
| `free_standing_none` | forked_anchor | rec_black-…4985a0ef | L151-265 (no handle part) | eligible if compatible (rigid only) | No grip; the rigid A-frame self-stands on its bent plate + folding rear support. |
| `bolt_on_carry_handle` | forked_anchor | rec_folding-…d961ac50 | L175-212 (`carry_handle` + FIXED `panel_to_handle`) | eligible if compatible | Cast-aluminium bolt-on bracket: mount plate + open rectangular grip loop (4 rails) + 4 dome carriage bolts (`Sphere`), FIXED to the body face. |
| `forearm_cradle_grip_bar` | forked_anchor | rec_riot_shield_var_grip_forearm | L85-118 (`_forearm_cradle`), L119-150 (`_grip_bar_assembly`) | eligible if compatible (soft only) | Forearm cradle cuff (`Cylinder` ring) + horizontal grip bar on standoffs, FIXED to the mount plate (two-point hold). |
| `rotating_grip` | forked_anchor | rec_riot_shield_var_form_round | L170-226 (`carry_handle` + REVOLUTE `panel_to_handle`) | eligible if compatible (round only) | Same bolt-on bracket but mounted on a REVOLUTE central boss (axis = shield normal) so the grip rotates. |
| `forearm_strap` | forked_anchor | rec_riot_shield_var_form_curved | L332-378 (`forearm_strap` part + REVOLUTE `strap_pivot`) | eligible if compatible (curved only) | Bolt-on bracket + a pivoting forearm retention strap (`strap_pivot` REVOLUTE) hanging from the grip frame, with buckle. |

硬约束满足：每个 slot ≥3 candidates（B=5, C=5, A=4），每个 candidate 结构不同、有真实
5 星来源。非活动细节（POLICE band、logo patch、ribs、grommets、bolts、viewport band、
rim/unit ring、bindings）全部是宿主 `.visual(...)`（Rule 1）；唯一 FIXED-jointed child 是
真实的 bolted-on cast-aluminium carry-handle 子装配（所有源都独立建模它）。

## 槽位图（slot graph）

pattern: `mixed`

```
Slot A body_form  (root part `front_panel`)
   │
   ├─[Slot B fold_topology]  front_panel --[REVOLUTE fold_hinge_0..k, axis=+Y, at panel top edge]--> fold_panel_1..k
   │        (soft: fabric fold-sleeve seam; rigid: pin-through-clamp-bracket clevis)
   │        └─[② gunport]  fold_panel_1 --[REVOLUTE shutter_hinge, axis=+Y, at window top]--> viewport_shutter
   │
   └─[Slot C grip]  front_panel --[FIXED | REVOLUTE panel_to_handle, at body mount face]--> carry_handle
            └─[forearm_strap only]  carry_handle --[REVOLUTE strap_pivot, axis=-Y, at grip frame bottom]--> forearm_strap
```

接口点位：
- fold hinge: shared fold-line axis `+Y` at each panel's top edge (`Origin xyz=(0,0,h)` local); pin-through-sleeve (rigid clamp bracket clevis captures `hinge_pin_*`; soft fabric fold sleeve wraps the child edge). These pin/sleeve/seam joints cannot be expressed as two axis-aligned `MatingContract` faces → grandfathered (no `mating=`), kept honest with element-scoped seam exemptions mirrored in `run_tests`.
- shutter hinge: `+Y` at window top; shutter plate + latch seat flush against the rear panel back when closed.
- grip mount: FIXED (bolt-on / cradle) or REVOLUTE (rotating axis=body normal `±X`) on the body mount face; the flat mount plate seats against the (possibly curved) shield face (`expect_contact` / `allow_overlap` for the curved/round shells).
- strap pivot: `-Y` at the grip frame bottom rail; the pin is captured inside the frame.

互斥 / 派生：fold_topology 与 grip 由 body_form 经 compatibility matrix 门控（见 §9）；
gunport 只在 rigid，trifold/quadfold 只在 soft，rotating_grip 只在 round，forearm_strap
只在 curved。curved/round 是 monolithic（无 fold 结构）。

## 每槽位 Module Emits / Interfaces

### Slot A / module rigid_polymer_panel
| emits | 描述 | 来源 |
|---|---|---|
| parts | `front_panel` (front_mid + front_bend + front_flare + ribs + police band + clamp brackets + hinge pins as visuals) | A / L161-226 |
| internal joints | none (fold hinge belongs to Slot B) | — |
| upstream interface | root grounded body; self-standing (no grip) | A / L151-160 |
| downstream interface | top-bend hinge line `RIGID_HINGE_XYZ` captures the rear panel pins; no handle mount | A / L253-265 |

### Slot A / module soft_fabric_panel
| emits | 描述 | 来源 |
|---|---|---|
| parts | `front_panel` (front_slab + side/side/end binding + hinge_sleeve + logo patch + grommets) | B / L123-174 |
| internal joints | none | — |
| upstream interface | root grounded body | B / L109-122 |
| downstream interface | top hinge sleeve at `z=h` → fold seam; mount face at `+X, z=SOFT_HANDLE_Z` for handle | B / L130, L204-212 |

### Slot A / module curved_polycarbonate_shell
| emits | 描述 | 来源 |
|---|---|---|
| parts | `front_panel` (`shield_shell` mesh + viewport band + edge trim + molded ribs) | form_curved / L234-287 |
| internal joints | none (monolithic) | — |
| downstream interface | convex outer face point `_shell_surface_x(CURVED_HANDLE_Z)` for handle mount | form_curved / L323-331 |

### Slot A / module round_convex_disc
| emits | 描述 | 来源 |
|---|---|---|
| parts | `front_panel` (`disc_shell` lathe + rim torus + unit ring) | form_round / L133-169 |
| internal joints | none (monolithic) | — |
| downstream interface | concave rear centre for the rotating central grip | form_round / L209-226 |

### Slot B / module bifold · trifold · quadfold (soft)
| emits | 描述 | 来源 |
|---|---|---|
| parts | `fold_panel_1..k` (loop-emitted `slab` + binding + hinge_sleeve via shared `_emit_soft_fold_panel`) | n3 L112-158 / n4 L129-183 |
| internal joints | `fold_hinge_0..k-1` REVOLUTE axis `+Y`; interior hinges mimic-coupled (`coupled_chain`, alternating ±2.0) for n≥3 | n3 L258-273 / n4 L295-316 |
| upstream interface | each panel's local frame on its bottom hinge edge; slab extends +Z (deployed-flat rest) | B / L229-241 |
| downstream interface | fabric fold-sleeve seam wraps the next hinge line | B / L130 |

### Slot B / module bifold · bifold_gunport (rigid)
| emits | 描述 | 来源 |
|---|---|---|
| parts | `fold_panel_1` (`rear_panel_body` + hinge tongues + perforated `vision_mesh` OR gun-port); `viewport_shutter` (gunport) | A L102-150, L227-252 / gunport L110-204 |
| internal joints | `fold_hinge_0` REVOLUTE axis `+Y` (rear hangs at deploy tilt); `shutter_hinge` REVOLUTE axis `+Y` (gunport) | A L253-265 / gunport L298-323 |
| upstream interface | hinge tongues rise into the clamp-bracket clevis; pins captured | A / L253-265 |
| downstream interface | closed shutter seats over the rear window | gunport / L181-204 |

### Slot C / module carry-handle family
| emits | 描述 | 来源 |
|---|---|---|
| parts | `carry_handle` (mount_plate + grip frame rails / cradle+bar + 4 dome bolts); `forearm_strap` (strap variant) | B L175-203 / grip_forearm L85-150 / form_curved L332-361 |
| internal joints | `panel_to_handle` FIXED (bolt-on/cradle) or REVOLUTE (rotating, axis=body normal); `strap_pivot` REVOLUTE axis `-Y` | B L204-212 / form_round L209-226 / form_curved L362-378 |
| upstream interface | mount plate seats flush on the body mount face | B / L204-212 |
| downstream interface | strap pin captured inside the grip frame bottom rail | form_curved / L362-378 |

活动件全部有 articulation 语义；不动细节写成 parent visual（Rule 1）。

## 参数范围汇总
| 参数 | 类型 | 取值范围 / 候选值 | 标称默认 | 约束类型 | 约束 / 函数 | 来源 |
|---|---|---|---|---|---|---|
| `body_form` | enum | rigid_polymer_panel / soft_fabric_panel / curved_polycarbonate_shell / round_convex_disc | — | choice | deterministic sampler picks first | Slot A |
| `fold_topology` | enum | monolithic / bifold / bifold_gunport / trifold / quadfold | — | conditional | 从 `ALLOWED_TOPOLOGY[body_form]` 采样 | Slot B |
| `grip` | enum | free_standing_none / bolt_on_carry_handle / forearm_cradle_grip_bar / rotating_grip / forearm_strap | — | conditional | 从 `ALLOWED_GRIP[body_form]` 采样 | Slot C |
| `n_panels` | int | 1..4 | — | equation | `= N_PANELS_BY_TOPOLOGY[fold_topology]` (不独立采样) | Slot B |
| `palette_style` | enum | tactical_black / ballistic_gray / olive_tan / smoke_clear | tactical_black | choice | 独立采样；translucent 形态叠加 alpha | ⑥ |
| `width_scale` | float | [0.90, 1.10] | 1.0 | independent | 均匀采样后 clamp | ⑤ |
| `height_scale` | float | [0.90, 1.12] | 1.0 | independent | 均匀采样后 clamp | ⑤ |
| (—) | constraint | — | — | inequality | fold_hinge / shutter_hinge / strap_pivot 行程由 `clamp_joint_limits`（seam/capture 对 exempt）解出可行上界；违反时收缩 | 接口 / clearance |

所有 conditional / equation / inequality 在 `resolve_config` + builder 的 clamp 内求解，不留到 fail。

## 7.5 编译预算 / compile budget
每-seed 编译预算 **≤8s**。依据：主体是薄壳/薄板 —— curved shell `MeshGeometry` 6×24 网格、
round disc lathe 48 段、perforated vision panel、torus 48 段，均为轻量 tessellation（小特征
≤48 段，主体英雄面 ≤96 段）；N 个相同 fold 面复用同一 `_emit_soft_fold_panel` helper。实测
40-seed 三段 pipeline ≈61s（含 fast/final/corner + clearance solves），单-seed 编译远低于预算。

## Multiplicity / Copy Logic

- 有一根 multiplicity 轴：`n_panels`（折叠面链）。
- `count_param`: `n_panels`；`N_range`（产品域）**[1, 4]**（monolithic=1 / bifold=2 / trifold=3 / quadfold=4）。sampling domain：由 `fold_topology` enum 派生（非独立 N 采样）——小 N（monolithic/bifold）高频，trifold/quadfold 稀有，符合权重档；仅 soft_fabric_panel 达到 3/4。
- copied object: 折叠面 slab + 边 binding (`_add_binding`) + fabric fold sleeve；naming `fold_panel_{idx}` / `fold_hinge_{idx}`（stable indexed，loop-emitted via 共享 helper）。
- placement: 沿 fold 轴在每个面顶边链式串接；accordion 交替折向（interior hinges mimic-coupled ±2.0）使各段折叠成栈。
- joint policy: 每对相邻面之间恰好一个 REVOLUTE fold hinge，共享 `+Y` fold 轴；handle 始终在 `front_panel`。
- source / gating: bifold=A,B；trifold=n3；quadfold=n4；仅 soft 到达 3/4（rigid 到 bifold+gunport）。
- 次级重复特征（grommets×3、ribs×3、dome bolts×4）保持 parametric / `record_only`，不是 fork。

## 视觉多样性 6 轴考察

| 轴 | 怎么判断 | 有/无 | 取值 / 理由 |
|---|---|---|---|
| ① 骨架图 | 加/减会动的 part 或一条边 | 有 | monolithic(1) / bifold(2) / trifold(3) / quadfold(4) 面链；rigid A-frame 折叠鞍撑；grip 子树 open-loop vs cradle+bar；均 forked_anchor（A,B,n3,n4,grip_forearm）。 |
| └ multiplicity | 同构件 ×N | 有 | 见 §8：`n_panels` N∈[1,4]，小 N 高频。 |
| ② 关节类型 | 图不变，换 type/轴 | 有 | REVOLUTE fold hinge (fabric fold-sleeve B / pin+clamp-bracket A)；REVOLUTE gun-port shutter_hinge(gunport)；grip 关节 FIXED vs REVOLUTE (rotating, axis=body normal)；REVOLUTE strap_pivot。每种都在 sweep 出现。 |
| ③ 主体形态家族 | 换核心 part 的可识别几何原型 | 有（登记进 slot_choices） | rigid bent polymer plate(A, Planar Boundary) / soft fabric slab(B, Planar Boundary) / curved polycarbonate shell(form_curved, Macro Surface) / round convex dished disc(form_round, Volumetric Envelope)；各 source-backed。 |
| ④ 表面装饰 | 叠加表面细节 | 有 (`record_only` + 宿主派生) | POLICE band(A) / G-FOLD logo patch + 白条(B) / 3× stiffening ribs / perforation pattern / viewport band(curved) / rim + unit ring(round) / edge binding + grommets；全部宿主 `.visual`，随 ③⑤ 逐-面/逐-z 共形（curved/round 用 `_shell_surface_x` 派生 x）。 |
| ⑤ 尺寸/行程 | 只连续改尺寸/行程 | 有 | width_scale [0.90,1.10]、height_scale [0.90,1.12]；关节运动包络（`clamp_joint_limits` 全程 solver-clamp）：fold_hinge_0 soft +Y [0, ≤1.4]（seam 对 exempt）、rigid [−0.5, +0.1]；interior fold hinges mimic-coupled ±2.0；shutter_hinge +Y [0, ≤1.4]（closed→open）；strap_pivot −Y clamp 到 [≈−0.30, 0.55]（禁止摆入壳体）；grip REVOLUTE(rotating) [−0.7,0.7]。motion_test_plan：`fail_if_parts_overlap_in_sampled_poses(max_pose_samples=64)` + 每机构一条 targeted `ctx.pose`（fold mid、shutter open、strap tilt、rotate）。seam / captured-pin / closed-shutter / strap-pin 接触写 sampled-pose exemption（见 §13）。 |
| ⑥ 涂装 | 只改材质/颜色 | 有 | 材质大类：painted polymer / soft fabric / translucent polycarbonate(glass-like) / cast-aluminium + steel hardware。配色 4：tactical_black / ballistic_gray / olive_tan / smoke_clear（translucent 形态叠加 alpha=0.6）。 |

## 采样与覆盖审计

总组合数（compatibility-gated body_form × fold_topology × grip）：
- rigid: 2 topo × 1 grip = 2
- soft: 3 topo × 2 grip = 6
- curved: 1 topo × 2 grip = 2
- round: 1 topo × 2 grip = 2
= **12 reachable slot-choice combos**（× palette 4 × 连续 width/height）。

理由：riot shields 的结构词汇天然有限（宽面板 + 可选折叠 + 一种握持 + 可选观察口），超出
form-family / fold multiplicity / gun-port / grip topology 后诚实候选耗尽；以低端广度覆盖而非
用 ④/⑤/⑥、缩放或涂装凑数。compatibility matrix 保证每个 seed 都是类别忠实的 shield。

seed_domain_policy：procedural_first（`config_from_seed` 对所有 seed 用 deterministic
`random.Random(seed)` 采样；seed 0 不特殊）。
Procedural Sampling / Sweep Plan：先选 body_form，再从该形态的 `ALLOWED_TOPOLOGY` /
`ALLOWED_GRIP` 采 fold_topology 和 grip（compatibility gating 避免非法组合，如 curved+trifold、
soft+free_standing、round+strap）；`n_panels` 由 topology 派生。无 regression overrides。
random sweep：seeds 0-35 初判 + corner 512-seed probe。
Topology target：真实 slot-choice tuple 空间仅 12（× palette 48），远低于 300 —— 由上述有限
结构词汇 + compatibility 门控决定；report-only，不作 gate。
Controlled local parameterization：width_scale [0.90,1.10]、height_scale [0.90,1.12]，
独立采样后 clamp；不破坏接口/clearance/joint origin（fold/shutter/strap 行程均 solver-clamp，
seam 与 bracket-clevis / bracket / bindings 随 width_scale 同步缩放，捕获接触对随之 exempt）。

| item | policy | validator / viewer focus |
|---|---|---|
| sampler | body_form → gated (fold_topology, grip) 加权采样，n_panels 派生 | `slot_choices_for_seed` == build choices（`slot_choice_errors=0`） |
| compatibility matrix | `ALLOWED_TOPOLOGY` / `ALLOWED_GRIP` per body_form；无 fallback（每形态自带合法默认 [0]） | 无 floating / 穿模 / 轴错 / closed-pose / max-N / bulky / optional-child 失败 |
| controlled local variation | width/height scale clamp；hinge/shutter/strap 行程 solver-clamp | 比例变化不破坏接口、clearance、joint origin、类别 identity |
| regression overrides | none | — |
| random sweep | seeds 0-35 初判；corner 512-probe 覆盖成熟度 | contract failures；axis_realization；viewer focus |

| slot | candidate_count | 是否 ≥2 | 是否 ≥3 | 备注 |
|---|---:|---|---|---|
| A body_form | 4 | yes | yes | ③ 主体形态家族，登记进 slot_choices |
| B fold_topology | 5 | yes | yes | 含 N multiplicity + ② gun-port |
| C grip | 5 | yes | yes | 含 ② FIXED/REVOLUTE grip 关节 |

## Validator

- `slot_choices_for_seed` 返回已实现的 module names（body_form / fold_topology / grip）
- `config_from_seed` 对所有 seed 用 deterministic procedural sampling（seed 0 不特殊）
- compatibility matrix (`ALLOWED_TOPOLOGY` / `ALLOWED_GRIP`) 阻止非法组合
- 无 regression overrides；不循环小型 curated 表
- width/height scale 在 `resolve_config` clamp；hinge/shutter/strap 行程在 builder 内 solver-clamp，不留到 fail
- 关键 joint 语义正确：fold_hinge_* REVOLUTE +Y；shutter_hinge REVOLUTE；panel_to_handle FIXED（或 rotating 时 REVOLUTE，轴=body normal）；strap_pivot REVOLUTE
- 复制面遵守 naming/placement（`fold_panel_{i}` / `fold_hinge_{i}`，共享 helper）
- captured/seam overlap 用 element-scoped `allow_overlap` 精确豁免（见 §13）

## Reject cases

- fold hinge clamp 收缩到 [0,0]（seam / bracket-clevis 接触未 exempt 或 margin 过大）→ 面不动
- 相邻面 seam 面对（slab / binding / sleeve）被当作 clearance 违规
- rigid 折叠 rear panel / tongue 与 clamp bracket cheek / hinge pin 捕获接触未豁免
- gun-port shutter 或 perforated vision_mesh 悬空（未贴合 rear 面）→ isolated / disconnected island
- forearm strap 摆入 curved 壳体（行程未 solver-clamp）→ 穿模
- 把 POLICE band / logo / ribs / bindings / bolts 做成 FIXED joint part（违反 Rule 1）
- 用 width/height 缩放或涂装冒充 ③ 主体形态多样性

## 与相邻类别的边界

- 不该混入：body armor / ballistic vest —— 穿戴式护甲板，无独立 shield 面板与握持/自立硬件。
- 不该混入：medieval tower shield / buckler prop —— 装饰盾，无折叠/铰链/工业握持，且比例/材质不符。
- 不该混入：tent / display panel board —— 无护体语义、无 carry handle / A-frame 支撑逻辑。

## 审核记录
| 项 | 结论 |
|---|---|
| reviewer status | pending |
| reviewer notes | Template `verdict=pass`（fast 16/16, final 36/36, corner 40/40, threshold 0.90）；4 body_form × 5 fold_topology × 5 grip 全部在 axis_realization 出现，slot_choice_errors=0；12 reachable combos 中 11 由 base seeds 实现，1 稀有 combo report-only 未覆盖（corner 门控通过）。 |

## 模板实现备注（可选）

- 共享 helper：`_emit_soft_fold_panel`（loop-emitted fold 面）、`_box/_cyl/_sphere`、`_add_binding`、`_emit_mount_plate/_emit_grip_frame/_emit_bolts`（grip 硬件）。
- 无 `MatingContract`：fold / shutter / grip-pivot 都是 pin-through-sleeve / fabric-fold-sleeve，无法用两个轴对齐面表达 → grandfathered（省略 `mating=`），用 element-scoped `allow_overlap` + `expect_contact` 在 `run_tests` 镜像。
- captured / seam element-scoped `allow_overlap`（solver `allowed_pairs` 与 QC 精确一致，避免 masked 穿模）：
  - soft 每个 fold seam：parent {slab, hinge_sleeve, 3× binding} × child {slab, 3× binding}（`_soft_seam_pairs` 全枚举，稳健于 ±height_scale 使常高 binding 滑过 seam）。
  - rigid 捕获铰：`hinge_pin_i` 与两个 `bracket_i_{inner,outer}_cheek` × {rear_panel_body, tongue_i}（`_rigid_capture_pairs`）；gunport 追加 rear_panel_body × {shutter_plate, shutter_latch}。
  - curved/round：平 mount_plate + bolt shanks/domes 座在弯面上；strap：pivot_pin + strap_body 座在 grip frame 上（`_strap_capture_pairs`）。
- 求解 margin：soft fold 0.004、rigid fold/shutter 0.006、strap 0.002（面/铰紧贴，seam 接触本身已 exempt；margin 只约束 child 对模型其余部分）。
- 不进入 seed domain 的组合：由 compatibility matrix 直接排除（curved/round 非 monolithic、soft 的 free_standing/gunport/rotating、rigid 的 handle 变体等）。
