# foosball_table — Modular Spec

## 元信息
| 项 | 值 |
|---|---|
| slug | `foosball_table` |
| template path | `agent/templates/Sports_Table_football.py` |
| test path (optional) | `tests/agent/test_foosball_table_template.py` |
| stage | `SPEC_ONLY_DRAFT` |
| status | `pending` |
| __modular__ | `True` |
| pattern | `mixed` (fixed cabinet/markings parent + 2 independent multiplicity axes: rod_count × figures_per_rod; leg & grip are fixed named slots) |

## 5 星样本阅读摘要
| 项 | 值 |
|---|---|
| five_star_total | 9 |
| read_count | 9 |
| read_scope | all 5-star samples in this category (parent + 8 forked variants) |
| source_index_policy | only adopted module sources are indexed below |

Sources read in full (all `revisions/rev_000001/model.py`):
- S1 `rec_foosball-table-black-cabinet-with-rounded-corner_20260611_160945_143790_5586ac02` (parent: 8 rods, splayed legs, plain grip, mixed figures)
- S2 `rec_foosball_table_var_rods6`
- S3 `rec_foosball_table_var_rods4`
- S4 `rec_foosball_table_var_panellegs`
- S5 `rec_foosball_table_var_crosslegs`
- S6 `rec_foosball_table_var_tubelegs`
- S7 `rec_foosball_table_var_contourgrip`
- S8 `rec_foosball_table_var_ballgrip`
- S9 `rec_foosball_table_var_figures3`

## 核心身份

A foosball / table-football table: a closed rectangular play cabinet (black side panels with rounded corners + lighter gray top rim) standing on a leg structure, holding a green pitch floor with raised white markings (halfway line, center circle/spot, goal boxes), an open goal slot in each end wall, and a bead score counter rail above each end. The defining articulated content is a set of horizontal steel **player rods** crossing the cabinet width (Y axis): each rod has TWO DOF — a **prismatic slide** along its own axis (massless carrier link) and a **continuous spin** (kick) — and carries a colored team-side handle grip plus N molded player figures (torso + head + legs + foot block) that hang along −Z at rest. The two-DOF rod spine (slide + spin) is the immutable identity of the category and must survive every slot swap.

Mature domain: a recognizable foosball table with a player-rod field, team colors (red vs blue), goal mouths at both ends, and a stable freestanding support. Not a generic game table; not a single-axis spinner toy.

## 槽位 + 候选模块表

### Slot A：player_rod_count (PRIMARY multiplicity — rod assemblies replicated by ROD_CONFIGS)

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| rods_8 | S1 | L78-L87 (ROD_CONFIGS table); rod loop L293-L378 | eligible if compatible | full eight-rod table, per-team GK1/D2/M5/A3 mixed layout, rods at x=±0.525..±0.075 (spacing 0.15) |
| rods_6 | S2 | L80-L102 (procedural ROD_CONFIGS, ROD_SPAN=1.05, N_RODS=6, spacing 0.210); rod loop L308-L386 | eligible if compatible | standard six-rod home table, per-team GK1/D2/A3, even spacing across span |
| rods_4 | S3 | L80-L86 (ROD_CONFIGS, ROD_COUNT=4); rod loop ~L290+ | eligible if compatible | compact four-rod mini table, per-team GK1 + combined-outfield rod(3), wide spacing (x=±0.36, ±0.12), larger travel 0.14/0.12 |

This is a multiplicity axis (Slot A *is* the count_param `rod_count`); the three sources are the N={8,6,4} witnesses. The rod-assembly module body itself (carrier + steel shaft + grip + figures + slide + spin) is structurally identical across N; only count, X-placement, per-rod n_fig/spacing/travel differ. See §8.

### Slot B：cabinet_leg_form (under-cabinet support structure — fixed named slot)

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| splayed_legs_4 | S1 | L249-L264 (placement loop); constants L60-L62 (LEG_TILT/LEG_LEN/LEG_SX/LEG_SY) | eligible if compatible | four white flat rectangular legs, each tilted by LEG_TILT≈0.13 rad outward about Y, inlined cab visuals, feet land at z≈0 |
| side_panels_full_height | S4 | `_ground_panel_mesh` L108-L133; placement L167-L177 | eligible if compatible | the two side panels themselves drop full-height (height=WALL_TOP) to the floor; arcade-style solid sides, NO separate legs |
| cross_x_trestles | S5 | `_add_x_trestle` L129-L167; constants L61-L64; placement L292-L295 | eligible if compatible | two crossed square-bar X-frame trestles (one per cabinet end at x=±END_X) + short horizontal cross brace; bars tilt between TOP_SPREAD 0.24 and FOOT_SPREAD 0.26 |
| tubular_legs_4 | S6 | `_tubular_leg_mesh` L131-L150; constants L61-L66; placement L275-L284 | eligible if compatible | four plumb round chromed tube legs (LEG_TUBE_R 0.025) each with a leveling foot disc (LEG_FOOT_R 0.035); inset LEG_INSET_X/Y=0.06 from corners |

### Slot C：rod_handle_grip_form (grip slipped over team-side rod end — fixed named slot, per-rod)

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| plain_cylinder_grip | S1 | handle emit L310-L318 (Cylinder r=0.0175, len=0.115) | eligible if compatible | plain straight cylindrical grip, single primitive `handle` |
| contoured_waisted_grip | S7 | `_ergonomic_grip_mesh` L112-L145; handle emit L347-L353 | eligible if compatible | lathe-revolved waisted barrel: flared ends + pinched center (spline profile), single mesh `handle` (long axis along local Z, rod's rpy aligns to Y) |
| ball_knob_grip | S8 | `_knob_grip_mesh` L101-L112 (GRIP_STEM_LEN 0.040 / STEM_R 0.013 / BALL_R 0.024); handle emit L330-L338 | eligible if compatible | classic stem+sphere knob, unioned into ONE solid `handle` (NOTE: source map said `handle_stem + handle_ball`, but the built model.py unions them into a single visual named `handle`); built along +Y, flipped via rpy for the −Y team |

Slot C has exactly 3 structurally-distinct candidates (≥3 met). All three keep the same downstream contract: a single `handle` visual on the team side, fully outside the cabinet wall.

### Slot D：figures_per_rod (nested SECONDARY multiplicity — per-rod inner figure loop)

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| mixed_figures (1/2/3/5) | S1 | per-rod n_fig from ROD_CONFIGS L78-L87; figure loop L330-L359 | eligible if compatible | realistic positional counts: GK=1, D=2, M=5, A=3 per row type; n_fig & spacing come from each ROD_CONFIGS entry |
| uniform_figures (constant) | S9 | FIGURES_PER_ROD L76-L77; ROD_CONFIGS L83-L92; figure loop L335-L359 | eligible if compatible | every rod carries the same FIGURES_PER_ROD count (=3 witness), uniform spacing 0.185 & travel 0.110 |

Slot D is degraded to **2 candidates** (justified): figures_per_rod is fundamentally a *count policy* (mixed-by-row-type vs uniform), not a topology with many distinct shapes — the figure module body (torso/head/legs/foot) is identical; only the per-rod count assignment differs. The 2 candidates are the only two structurally-meaningful distribution policies, and the underlying N sweep (1..5 figures) supplies the real multiplicity diversity. This is the source-map-blessed pair; reviewer note flagged below.

## 槽位图（slot graph）

pattern: mixed (fixed parent cabinet + parallel-children leg slot + per-rod multiplicity with nested figure multiplicity)

```
cabinet (FIXED parent: base + green pitch + side panels w/ rod bores + end walls w/ goal slots + gray rim + markings + score rails)
  │
  ├─[Slot B: cabinet_leg_form] — fixed support to floor
  │     splayed_legs_4 / tubular_legs_4 : inlined leg visuals on `cabinet`, mount face = cabinet underside (z=BASE_BOT), feet at z≈0
  │     side_panels_full_height : leg slot FOLDS INTO the side panels (panels themselves drop to floor; no separate leg visuals)
  │     cross_x_trestles : inlined trestle visuals on `cabinet` at x=±END_X, must clear the goal mouths/score posts
  │
  └─[Slot A: player_rod_count] ×N rod assemblies, evenly spaced along X
        cabinet --[rod_{idx}_slide: PRISMATIC, axis=(0,1,0), origin=(x,0,ROD_Z), limits=±travel]--> rod_{idx}_carrier (massless)
        rod_{idx}_carrier --[rod_{idx}_spin: CONTINUOUS, axis=(0,1,0), origin=(0,0,0)]--> rod_{idx}
        rod_{idx} body:
          - shaft (steel cylinder along Y, passes through bearing bores in both side panels — overlap allowed)
          - [Slot C: rod_handle_grip_form] handle (team side, outside wall)
          - end_cap (black, opposite end)
          - [Slot D: figures_per_rod] ×n_fig player figures (torso/head/legs/foot), spaced along Y, hang −Z at q=0
```

Interface notes:
- **Slot B ↔ cabinet**: mount at cabinet underside; downstream = floor contact (feet/panel bottom at z≈0). `side_panels_full_height` is the one case where the side-panel slot and leg slot are the SAME geometry (panels replace both the rod-bore wall AND the legs); the rod-bore bearing holes must still be present in the full-height panel.
- **Slot A ↔ cabinet**: the slide joint origin is on the cabinet at (x, 0, ROD_Z); rod bores at each `cfg[1]` x-position must exist in both side panels (the bore point set is derived from ROD_CONFIGS, so it tracks Slot A's count/placement).
- **Slot C ↔ rod**: grip is a fixed visual on `rod_{idx}` at the team-side rod end (y = side·end_y), no joint; downstream identity = single `handle` AABB beyond |CAB_W/2|.
- **Slot D ↔ rod**: figures are fixed visuals on `rod_{idx}`, spaced along the rod axis; they ride the spin DOF.

## 每槽位 Module Emits / Interfaces

### cabinet (FIXED parent — not a swappable slot)
| emits | 描述 | 来源 |
|---|---|---|
| parts | `cabinet` part: cabinet_base, pitch (green), side_panel_0/1 (rounded, rod-bored), end_wall_{0,1}_post_{a,b} + _lintel (goal slot), side_rim/end_rim (gray), markings (halfway_line, center_circle, center_spot, goal_box_*), score_post/score_rail/score_bead_* | S1 / L143-L290 |
| internal joints | none (single rigid cabinet) | — |
| upstream interface | world root (base link) | S1 / L143 |
| downstream interface | underside mount for Slot B (z=BASE_BOT); rod-slide origins at (x,0,ROD_Z); rod bores in side panels | S1 / L112-L124, L361-L369 |

### Slot A / module rods_{N}
| emits | 描述 | 来源 |
|---|---|---|
| parts | per rod: `rod_{idx}_carrier` (massless, Inertial 1e-4), `rod_{idx}` (shaft + handle + end_cap + figures) | S1 / L299-L359 |
| internal joints | `rod_{idx}_slide` PRISMATIC axis Y ±travel (cabinet→carrier); `rod_{idx}_spin` CONTINUOUS axis Y (carrier→rod) | S1 / L361-L378 |
| upstream interface | slide joint origin on cabinet at (x, 0, ROD_Z) | S1 / L366 |
| downstream interface | rod axis hosts Slot C grip + Slot D figures; shaft seated in side-panel bores (allow_overlap + expect_contact) | S1 / L302-L308, L429-L447 |

### Slot B / module <leg form>
| emits | 描述 | 来源 |
|---|---|---|
| parts | inlined visuals on `cabinet`: `leg_{i}` (splayed/tubular) or `trestle_{i}_bar_{a,b}`+`trestle_{i}_brace` or `ground_panel_{i}` | S1 L249-L264 / S6 L275-L284 / S5 L292-L295 / S4 L167-L177 |
| internal joints | none (fixed support) | — |
| upstream interface | cabinet underside / side-panel geometry | per source |
| downstream interface | floor contact at z≈0 (feet, panel bottom, or bar ends) | S1 L536-L543 / S5 L567-L584 / S6 L556-L573 |

### Slot C / module <grip>
| emits | 描述 | 来源 |
|---|---|---|
| parts | single `handle` visual on `rod_{idx}` (Cylinder, revolved mesh, or unioned stem+ball mesh) | S1 L310-L318 / S7 L347-L353 / S8 L330-L338 |
| internal joints | none (fixed to rod, rides spin) | — |
| upstream interface | team-side rod end at y = side·end_y (red=−Y, blue=+Y) | S1 / L312-L316 |
| downstream interface | handle AABB lies fully outside the cabinet wall (|y| > CAB_W/2) | S1 / L461-L470 |

### Slot D / module <figures policy>
| emits | 描述 | 来源 |
|---|---|---|
| parts | per figure: `player_{j+1}_torso` (Box), `_head` (Sphere), `_legs` (Box), `_foot` (Box tilted) | S1 / L330-L359 |
| internal joints | none (fixed to rod, ride spin) | — |
| upstream interface | rod axis; spaced yj = (j−(n_fig−1)/2)·spacing | S1 / L331 |
| downstream interface | feet hang above pitch at rest (expect_gap), clear pitch at all spin angles | S1 / L449-L458, L505-L516 |

## 参数范围汇总
| 参数 | 类型 | 取值范围 / 候选值 | 标称默认 | 约束类型 | 约束 / 函数 | 来源 |
|---|---|---|---|---|---|---|
| leg_form | enum | {splayed_legs_4, side_panels_full_height, cross_x_trestles, tubular_legs_4} | splayed_legs_4 | choice | procedural sampler | Slot B table |
| grip_form | enum | {plain_cylinder_grip, contoured_waisted_grip, ball_knob_grip} | plain_cylinder_grip | choice | procedural sampler | Slot C table |
| figures_policy | enum | {mixed_figures, uniform_figures} | mixed_figures | choice | procedural sampler | Slot D table |
| rod_count | int (multiplicity) | even ∈ [2,8] (test {4,6,8}; product full) | 8 | conditional | weighted draw §8; even-only (per-team symmetry); X-placement derived from count | Slot A / S1 L78-87, S2 L80-102 |
| figures_per_rod | int (multiplicity) | per-rod n_fig ∈ [1,5] | mixed (GK1/D2/M5/A3) | conditional | uniform-policy → constant N; mixed-policy → per-row-type assignment | Slot D / S1, S9 |
| palette_style | enum | {classic_black_red_blue, arcade_blue_yellow_white, natural_wood_red_blue, pro_charcoal_silver, retro_green_orange} | classic_black_red_blue | choice | per-seed colorway sample; remaps cabinet/leg/rim/team materials | materials across S1–S9 |
| cabinet_length_scale | float | [0.92, 1.10] | 1.0 | independent | scales CAB_L; rod X-positions scale with it | S1 / L34 |
| rod_spacing_scale | float | derived | 1.0 | equation | `= cabinet_length_scale` (rods stay evenly spread over scaled span; spacing = span/(N−1)) | S2 / L80-L82 |
| leg_splay_scale | float | [0.7, 1.3] | 1.0 | independent | scales LEG_TILT (splayed) / spread (trestle); clamp so feet footprint < cabinet footprint+margin | S1 L60 / S5 L62-63 |
| grip_radius_scale | float | [0.85, 1.15] | 1.0 | independent | scales grip outer radius; clamp ≤ rod_spacing/2 to avoid neighbor-grip clash | S1 L311 / S7 / S8 |
| slide_travel_scale | float | [0.85, 1.10] | 1.0 | conditional | scales per-rod travel; upper bound conditional on rod_count (denser rods → less travel) | S1 L78-87 / S3 L80-86 |
| (—) | constraint | — | — | inequality | figure_swing_radius < (ROD_Z − PITCH_TOP) − 0.002 AND 2·swing_radius < rod_spacing; violate → shrink figure/spacing or reject | S1 / L389-L398 |
| (—) | constraint | — | — | inequality | every figure stays inside interior walls at both slide extremes (|y| ≤ INNER_Y − 0.002) | S1 / L472-L486 |
| (—) | constraint | — | — | inequality | (cross_x_trestles only) trestle bars + brace must clear goal mouth/score posts at x=±END_X | source map watch / S5 |

palette_style colorways (drawn from materials observed across S1–S9; cabinet_black/rim_gray/pitch_green/leg_white/chrome + team_red/team_blue):
- **classic_black_red_blue** — black cabinet, gray rim, white/chrome legs, red vs blue teams (parent S1)
- **arcade_blue_yellow_white** — dark-blue cabinet, white rim, white legs, yellow vs white teams (arcade-style)
- **natural_wood_red_blue** — warm wood-tone cabinet/legs, green pitch, red vs blue teams (home/wood table)
- **pro_charcoal_silver** — charcoal cabinet, brushed-silver/chrome tube legs + rim, red vs blue teams (pro/tournament; pairs with tubular_legs_4)
- **retro_green_orange** — muted green cabinet, cream rim, wood legs, orange vs green teams (retro)

## Multiplicity / Copy Logic

This template has **TWO independent multiplicity axes** (rod_count is primary; figures_per_rod is a nested secondary axis carried inside each rod assembly).

### Axis 1 (PRIMARY): rod_count
- `count_param`: `rod_count` = len(ROD_CONFIGS)
- `N_range`: product **even ∈ [2, 8]** (real foosball is per-team symmetric, so counts are even and almost always 4/6/8); **test-small** = {4, 6} with 8 in the maturity sweep
- sampling domain (weighted): heavy on {6, 8} (canonical full tables), moderate {4} (mini table), rare {2} (degenerate goalie-only); odd counts excluded
- copied object: one rod assembly = massless `rod_{idx}_carrier` + steel `rod_{idx}` (shaft + handle + end_cap + n_fig figures) + slide joint + spin joint
- naming: `rod_{idx}` / `rod_{idx}_carrier` (idx = 1..N); joints `rod_{idx}_slide` / `rod_{idx}_spin`
- placement: rods evenly spaced along X across the rod span; per-team role assignment (GK at ends, outfield inward) derived from N (N=4 → GK+combined; N=6 → GK/D/A; N=8 → GK/D/M/A)
- joint policy: EVERY rod keeps the immutable two-DOF spine — `rod_{idx}_slide` PRISMATIC axis Y ±travel on the carrier + `rod_{idx}_spin` CONTINUOUS axis Y on the rod; uniform across all N; survives every Slot B/C/D swap
- source/gating: rod bores in side panels are generated from the same ROD_CONFIGS x-list so bores track count; figure clearance inequalities re-checked at the sampled N

### Axis 2 (SECONDARY, nested): figures_per_rod
- `count_param`: `figures_per_rod` = per-rod `n_fig`
- `N_range`: **per rod ∈ [1, 5]** (GK=1, D=2, A=3, M=5 are the real foosball row counts); test-small = {1,2,3}, full includes 5
- sampling domain: gated by `figures_policy` enum — `mixed_figures` assigns n_fig by row type (positionally realistic); `uniform_figures` sets all rods to one constant ∈ {2,3} (3 is the canonical uniform witness)
- copied object: one molded player figure = `player_{j+1}_torso` (Box) + `_head` (Sphere) + `_legs` (Box) + `_foot` (tilted Box)
- naming: `player_{j+1}_{torso,head,legs,foot}` (j = 0..n_fig−1)
- placement: figures evenly spaced along the rod axis (Y), centered (yj = (j−(n_fig−1)/2)·spacing), hanging along −Z at q=0
- joint policy: figures are FIXED visuals on `rod_{idx}` (no own joint); they ride the rod's spin + slide
- source/gating: spacing must satisfy the slide-extreme interior-wall inequality and the figure-swing clearance; denser figures (n_fig=5) use tighter spacing 0.12

## 拓扑多样性审计

总组合数：Slot B(4) × Slot C(3) × Slot D-policy(2) × rod_count N-samples(≈3: {4,6,8}) × figures_per_rod nested(uniform constant ∈{2,3} ≈2, mixed=1) ≈ 4×3×2×3 = **72 base topology combos** (before nested figure-count variation), well over the multiplicity diversity floor.

理由：leg_form(4) × grip_form(3) alone = 12 distinct leg/grip topologies; multiply by 2 figure policies and 3 rod-count samples and distinct topologies far exceed 10. The slot/multiplicity space — not continuous scales — is the diversity source.

seed_domain_policy：procedural_first
Procedural Sampling / Sweep Plan: `config_from_seed` does a deterministic weighted draw per axis — (1) sample leg_form, grip_form, figures_policy from uniform/lightly-weighted enums; (2) weighted draw rod_count from {4,6,8} (heavy 6/8); (3) resolve per-rod n_fig from figures_policy (mixed→row-type table, uniform→constant); (4) sample independent continuous scales (cabinet_length, leg_splay, grip_radius, slide_travel) then derive rod_spacing_scale = cabinet_length_scale; (5) project onto the figure-clearance + interior-wall inequalities, retracting spacing/figure size or rejecting+resampling on violation; (6) resolve trestle-clearance conditional for cross_x_trestles. `slot_choices_for_seed` returns the same enum picks the builder uses. Random sweep seeds 0–49 for the initial pass, 0–999 for maturity.
Topology target: 1000-seed slot choice tuple distinct expected ≥ ~40–70 (bounded by 72 base combos × nested figure counts; below the generic ≥300 because the category genuinely has a small fixed structural vocabulary — only leg/grip/figure-policy/rod-count vary; the cabinet, pitch, goal slots and two-DOF rod spine are invariant identity). This bounded ceiling is the category-compatibility reason for <300.（统一口径：富类别建议 ≥300；低于 300 记录真实组合空间或兼容约束原因；report-only，不设门。）
若使用 regression overrides：none planned (the parent's mixed-1/2/3/5 layout is reproducible via figures_policy=mixed + rod_count=8, not an override).
Controlled local parameterization: cabinet_length_scale [0.92,1.10] (independent), rod_spacing_scale (= cabinet_length_scale, equation), leg_splay_scale [0.7,1.3] (independent, clamp feet footprint), grip_radius_scale [0.85,1.15] (independent, clamp ≤ rod_spacing/2), slide_travel_scale [0.85,1.10] (conditional on rod_count). All resolved in `resolve_config`; figure-swing and interior-wall constraints are inequalities applied after derivation; none may break the slide/spin joint origins, the rod-bore interface, or the goal-slot identity.

| item | policy | validator / viewer focus |
|---|---|---|
| sampler | weighted enum draws (leg/grip/figures_policy) + weighted rod_count {4,6,8} + nested n_fig; continuous scales independent→derived→projected | slot_choices_for_seed matches build choices |
| compatibility matrix | all leg×grip×figures×N legal; cross_x_trestles gated by trestle-clearance inequality (fallback: single-crossbar trestle); rod_count even-only; figure spacing clamped at high n_fig & dense rods | no floating legs, no goal-mouth foul, no neighbor-rod/grip clash, slide-extreme wall containment, spin-pose pitch clearance |
| controlled local variation | cabinet_length/leg_splay/grip_radius/slide_travel scales with clamps + derived rod_spacing | proportions vary without breaking rod-bore interface, joint origins, figure clearance, or category identity |
| regression overrides | none | — |
| random sweep | seeds 0–49 initial, 0–999 maturity | contract failures |

| slot | candidate_count | 是否 ≥2 | 是否 ≥3 | 备注 |
|---|---:|---|---|---|
| A (rod_count) | 3 (N witnesses {8,6,4}) | yes | yes | multiplicity axis |
| B (leg_form) | 4 | yes | yes | |
| C (grip_form) | 3 | yes | yes | |
| D (figures_policy) | 2 | yes | no | count-policy axis; degraded to 2 (justified above) — real diversity from nested n_fig sweep |

## Validator

- slot_choices_for_seed returns implemented module names for leg_form / grip_form / figures_policy and the sampled rod_count / figures_per_rod
- config_from_seed uses deterministic procedural sampling for all ordinary seeds
- compatibility matrix / gating prevents illegal combos (odd rod_count, trestle goal-mouth foul, over-dense figures)
- no regression overrides; seed domain is procedural, not a curated table
- continuous scales clamped/derived in resolve_config (rod_spacing = cabinet_length; grip_radius ≤ rod_spacing/2; figure-swing & interior-wall inequalities); never left to fail in builder
- every rod has both `rod_{idx}_slide` (PRISMATIC, axis Y, ±travel) and `rod_{idx}_spin` (CONTINUOUS, axis Y)
- rod shaft seated in both side-panel bores (allow_overlap + expect_contact), retained insertion at slide extremes
- handle (Slot C) AABB lies fully outside the cabinet wall on the team side
- figures (Slot D) hang above pitch at rest and clear pitch at all sampled spin angles
- legs/panels/trestles reach the floor (z≈0); goal slot open in both end walls; score rail above each end
- copied rod & figure objects follow naming (`rod_{idx}` / `player_{j+1}_*`) and even-spacing placement

## Reject cases

- A rod missing one of its two DOF (slide or spin), or a slide that is not PRISMATIC / spin not CONTINUOUS (breaks category identity).
- Odd `rod_count`, or a rod_count whose figures violate the swing-radius/neighbor-clearance inequality at the sampled spacing.
- Handle grip that intersects the cabinet wall instead of sitting fully outside it (wrong team side or grip too long).
- Figures that swing into the pitch at some spin angle, or escape the interior walls at a slide extreme.
- Leg/panel/trestle that does not reach z≈0 (floating table) or whose footprint collides with the cabinet bores/goal mouth (cross_x_trestles fouling the goal slot).
- Side panels lose the rod clearance bores (rod can't pass / disconnected), or full-height panels drop the bores when leg_form=side_panels_full_height.
- Goal slot in an end wall closed off (post/lintel geometry seals the mouth).
- Continuous scale left unclamped so cabinet_length and rod_spacing desync (rods bunch or overrun the cabinet ends).

## 与相邻类别的边界

- 不该混入：generic **game/dining table** (Furniture) — foosball requires the player-rod field + goal mouths + two-DOF kicking rods; a plain tabletop is not this category.
- 不该混入：**air hockey / pool table** — those have a flat play surface with pucks/balls but no through-cabinet articulated player rods; the rod-spin kick mechanism is the identity.
- 不该混入：single-axis **spinner / pinball** toys — foosball's defining content is the *replicated* multi-rod field (multiplicity), not one actuated lever.

## 审核记录
| 项 | 结论 |
|---|---|
| reviewer status | pending |
| reviewer notes | (1) Slot D degraded to 2 candidates (count-policy axis, justified — nested n_fig sweep carries real multiplicity). Confirm acceptable. (2) ball_knob_grip emits a single unioned `handle` visual, NOT separate `handle_stem`+`handle_ball` as the source map names suggested — spec follows the built model.py. (3) cross_x_trestles needs a trestle-clearance gate vs goal mouth/score posts (source-map watch item) with single-crossbar fallback. (4) Topology distinct ceiling ~40–70 (<300) is intrinsic to the category's small structural vocabulary; flagged for reviewer acceptance. |
