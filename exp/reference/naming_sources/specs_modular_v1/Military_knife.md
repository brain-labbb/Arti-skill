# Modular Spec — katana (Military / knife → Japanese KATANA display set)

## 元信息

| 项 | 值 |
|---|---|
| slug | `katana` |
| template path | `agent/templates/Military_knife.py` |
| test path (optional) | `tests/agent/test_katana_template.py` |
| stage | `SPEC_ONLY_DRAFT` |
| status | `pending` |
| __modular__ | `True` |
| pattern | `mixed` (stand-style root chooses 1 of N parallel mount stations; each station carries a 2-link `saya → blade` chain; sword count is a `multiplicity` axis) |

> 小类 label is `knife`, but the parent asset and all 9 variants are a **Japanese katana display set** (slug `katana`): one or more sheathed katanas mounted on a stand / wall panel. The articulated unit is a sheathed katana whose blade assembly draws out of its scabbard. The rack is the structural spine.

## 5 星样本阅读摘要

| 项 | 值 |
|---|---|
| five_star_total | 10 |
| read_count | 10 |
| read_scope | all 5-star samples in this 小类 (1 parent + 9 `rec_katana_var_*`); each read in full: `model.py` + `record.json` (rating=5 confirmed) |
| source_index_policy | only adopted module sources are indexed below |

**Reading summary.** Every sample shares the same per-sword physical contract: a hollow white **saya** (scabbard, `_saya_tube_shape`, mouth at local x=0, tail at x=−0.73, bore radius 0.0145) holding a lofted tapered **blade_assembly** (`_blade_shape` + habaki collar + tsuba + fuchi + tsuka grip + kashira pommel). Each sword attaches to the rack by a **FIXED** `saya_mount` and the blade draws via a **PRISMATIC** `blade_draw` (axis (1,0,0), origin (0,0,0), `MotionLimits(lower=0, upper=0.70, effort=25, velocity=0.6)`). Saya/blade meshes are built once and shared across all swords. The 10 samples vary along exactly three structural axes plus one multiplicity axis:

- **Stand style (rack spine, root part):** parent = two black-lacquer **pillar planks with twin forward crescent cradles** on a base box (tiered_crescent_rack); `var_vpost` = three upright **U-channel posts** holding near-vertical swords (FIXED mount gets a `rpy=(0,PITCH,0)` lean); `var_wallmount` = flat **wall panel with 6 L-bracket hooks**, no base box, swords float above ground.
- **Tsuba (guard, on blade_assembly):** parent = plain **round red disc** (Cylinder r=0.041 rotated π/2 about Y) + gold annular rim; `var_tsubasq` = **mokkō-gata** four-lobe rounded-square plate (`_mokko_tsuba_shape`) + square rim; `var_tsubapierced` = **openwork sukashi** disc (`_tsuba_sukashi_shape`, boolean-cut tang slot + 4 radial slots + 4 cardinal holes).
- **Tsuka wrap (grip, on blade_assembly):** parent = **pink ito diamond-wrap** (pink grip + dark `wrap_diamond_top_*`/`wrap_diamond_front_*` lozenge Boxes); `var_gripsmooth` = bare **smooth samegawa** grip (no diamonds, samegawa material); `var_gripcord` = **cord ring bands** (`GRIP_BAND_COUNT` evenly-spaced dark `_ring_shape` bands).
- **Sword count (multiplicity):** parent {3} via a hand-tuned 3-key `SWORD_MOUNTS` dict (`top`/`middle`/`bottom`); `var_n1` {1}, `var_n2` {2} use a clean `N_SWORDS` int with a `_sword_mount_*` helper and **regular tier spacing** (`TIER_BASE_Z + i·TIER_SPACING`); `var_n3loop` {3} **rewrites the parent dict into the loop form** (`TIER_X/TIER_Y/TIER_Z` arrays + `_sword_mount_xyz(i)` + `_mount_sword(...)`, single `for i in range(N_SWORDS)`).

The for-i sword loop-rewrite (replacing the parent's literal 3-key dict) is therefore part of the template contract; the clean source of that rewrite is `var_n3loop` (+ `var_n1`/`var_n2` for the regular-spacing N≠3 generalization).

## 核心身份

A **displayed Japanese sword set**: a decorative stand (or wall panel) holding one or more **sheathed katanas**, each of which keeps a **FIXED saya mount** to the rack and a **PRISMATIC blade draw** that slides the blade assembly (blade + habaki + tsuba + tsuka + kashira) out of its scabbard along the sword long axis (travel 0..0.70 m). The mature domain is a single rack object: a base-box-and-pillars stand, an upright-post stand, or a wall-mount board. Each katana is ~1.0 m long with the recognizable parts — white saya with deep-pink mouth ring / bands / sakura blossoms, gold habaki, a tsuba guard wider than the saya, a wrapped tsuka grip, and a dark kashira pommel.

**Neighbor boundary.** Each sword is **sheathed-mountable** (a scabbard the blade nests inside) and **rack-displayed** — this distinguishes it from the other `knife` families that share the 小类 label but are NOT part of this template:

- NOT `retractable_utility_knife` / `military_knife` (folding/fixed combat or utility knives with blade-pivot or slider mechanisms, handheld, no rack, no scabbard-draw).
- NOT a single in-hand katana with no display rack (this template's identity is the **display set** — a rack carrying N sheathed swords).

## 槽位 + 候选模块表

### Slot A：stand_style（rack spine / root part — what carries the swords）

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| tiered_crescent_rack | rec_model-a-decorative-japanese-katana-display-set-a_…_ac7923b8 (parent) | `_pillar_shape` L88-L105; base box `display_stand` + plinth/body/cap + 2× `pillar_{i}` L124-L152; kanji plaque + frame + strokes L154-L182 | eligible if compatible | base box (≈0.30×0.18×0.12) + two black-lacquer pillar planks, each with two forward crescent cradle notches (lower z≈0.32, upper z≈0.46); extra sword seats on the box top; gold-framed kanji plaque on −Y face |
| vertical_post | rec_katana_var_vpost | `_post_shape` L103-L131 (U-channel: back panel + 2 side walls + foot); base box L152-L167; `post_{i}` placed at `POST_XS` L170-L177; mount lean `rpy=(0,PITCH,0)` L323-L338 | eligible if compatible | base box + N upright U-channel posts on the box top; each post holds a **near-vertical** sheathed katana with a 2° lean; saya seats 0.5 mm into the channel back panel; mount xyz = (post_x + lean offset, MOUTH_Y, MOUTH_Z=0.87) |
| wall_mount | rec_katana_var_wallmount | `wall_panel` root + `panel_body` L170-L177; gold frame L179-L192; plaque L193-L224; `_hook_shape` L98-L150 (L-bracket + cradle notch + rib); hooks placed L226-L237; mount L360-L373 | eligible if compatible | flat black-lacquer wall board (≈0.30 W × 0.58 H × 0.018 T), **no base box**, bottom z≈0.08 above ground; 6 L-bracket cradle hooks in tiers; swords float above ground; plaque below bottom tier |

All 3 candidates structurally distinct (different root part, different part tree, different saya support geometry, different mount frame). Slot A ≥3 ✔.

### Slot B：tsuba（guard plate on the blade_assembly — off-axis disc/plate wider than the saya）

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| round_disc | parent …_ac7923b8 | `tsuba_disc` Cylinder r=0.041 rotated π/2 about Y L249-L254 + `tsuba_rim` (`_ring_shape` mesh) L255-L260; rim mesh built L191-L192 | eligible if compatible | plain round red disc guard (∅≈0.082 > 2.2·SAYA_R) with gold annular rim; cheapest topology |
| square_mokko | rec_katana_var_tsubasq | `_mokko_tsuba_shape` L82-L98 (rounded-square plate + center tang slot) + `_mokko_tsuba_rim_shape` L101-L125; emitted as `tsuba_plate` + `tsuba_rim` L302-L313 | eligible if compatible | mokkō-gata four-lobe rounded-square guard **plate** (filleted rect, vertical tang slot) + matching square gold rim; element name is `tsuba_plate` not `tsuba_disc` |
| pierced_sukashi | rec_katana_var_tsubapierced | `_tsuba_sukashi_shape` L79-L133 (round plate boolean-cut: nakago-ana + 4×45° radial slots + 4 cardinal holes) + `tsuba_rim` L308-L321 | eligible if compatible | openwork sukashi guard: round disc with central tang slot, four 45° radial petal slots, four cardinal circular through-holes |

All 3 distinct (round solid vs lobed-square plate vs pierced openwork). Slot B ≥3 ✔.

### Slot C：tsuka_wrap（grip wrap on the blade_assembly）

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| pink_diamond_ito | parent …_ac7923b8 | `tsuka_grip` pink Cylinder L267-L273 + `wrap_diamond_top_{0..2}` L282-L289 + `wrap_diamond_front_{0..1}` L290-L296 | eligible if compatible | pink grip with 5 dark diamond-wrap (ito) lozenge Box accents on top and front faces |
| smooth_samegawa | rec_katana_var_gripsmooth | `tsuka_grip` Cylinder with `material="samegawa"` L267-L273; **no** `wrap_diamond_*` elements; samegawa material defined L119 | eligible if compatible | bare smooth samegawa-style grip cylinder, all diamond-wrap accents removed; grip material `samegawa` (off-white ray skin) not pink |
| cord_ring_bands | rec_katana_var_gripcord | `tsuka_grip` pink Cylinder L274-L280 + `grip_band_{i}` loop L289-L300; `grip_band_mesh` (`_ring_shape`) L197-L198; `GRIP_BAND_COUNT=12` L42 | eligible if compatible | pink grip wrapped with `GRIP_BAND_COUNT` evenly-spaced dark cord **ring bands** (annular `_ring_shape` rings) instead of diamonds |

All 3 distinct (lozenge Boxes vs bare cylinder vs annular ring bands; different part-list cardinality on the blade). Slot C ≥3 ✔.

## 槽位图（slot graph）

pattern: `mixed` (parallel mount stations on a chosen root spine; each station hosts a 2-link draw chain; station count is a multiplicity axis)

```
[Slot A stand_style: root part]
  tiered_crescent_rack ─ base box + pillar cradle notches (z≈0.32 / 0.46) + box-top seat
  vertical_post        ─ base box + U-channel posts at POST_XS, MOUTH_Z=0.87, lean PITCH
  wall_mount           ─ wall panel (root, NO base box) + L-bracket cradle hooks
        │
        │  for i in range(N_SWORDS):   ← multiplicity axis (sword count)
        │      saya_i = build_saya(...)            (shared saya/blade meshes)
        │      blade_i = build_blade_assembly(..., tsuba=Slot B, wrap=Slot C)
        │
        ├── FIXED  {prefix|sword_i}_saya_mount
        │     parent = root spine part  →  child = saya_i
        │     origin xyz = mount_xyz(i)  (rack-style-specific seat: cradle z / post mouth / hook tier)
        │     rpy = (0,0,0)  [crescent_rack, wall_mount]  |  (0,PITCH,0)  [vertical_post lean]
        │
        └── PRISMATIC  {prefix|sword_i}_blade_draw
              parent = saya_i  →  child = blade_assembly_i
              origin (0,0,0); axis (1,0,0); MotionLimits(lower=0, upper=TRAVEL=0.70, effort=25, vel=0.6)
              [Slot B tsuba + Slot C tsuka_wrap live inside blade_assembly_i; no extra joints]
```

- **Slot A → swords interface:** the chosen root spine provides one mount frame (xyz, and rpy for the lean) per sword index. The mount xyz is the **saya mouth** position in the stand frame (cradle notch center, post channel mouth, or hook cradle center).
- **Saya → blade interface:** the blade nests inside the hollow saya bore; the PRISMATIC joint origin is at the saya mouth (0,0,0 local); at q=0 the blade is fully sheathed, at q=TRAVEL it has translated 0.70 m along +X while retaining ≥0.010 m insertion overlap.
- **Slot B / Slot C are module-local to blade_assembly** — they change only the guard plate and grip-wrap visuals; they emit no joints and do not change the mount or draw interface. They are mutually independent and compatible with every Slot A choice.
- **Mutual exclusion / derivation:** exactly one Slot A, one Slot B, one Slot C per seed. `vertical_post` derives a non-zero mount `rpy` (lean) and a near-vertical saya pose; the other two racks keep horizontal swords (rpy=0). All three Slot A choices accept any N in range.

## 每槽位 Module Emits / Interfaces

### Slot A / tiered_crescent_rack
| emits | 描述 | 来源 |
|---|---|---|
| parts | `display_stand` (root): `base_plinth`/`base_body`/`base_cap` boxes, 2× `pillar_{i}` (mesh `_pillar_shape`), `plaque_panel` + 4 `frame_*` + 4 `kanji_stroke_*` | parent L124-L182 |
| internal joints | none (rigid root) | parent |
| upstream interface | ground plane (base box rests at z≈0) | parent L331 (test) |
| downstream interface | per-sword FIXED mount frame: 2 cradle notches (z≈0.32 lower, 0.46 upper) + box-top seat (z = BOX_TOP_Z + SAYA_R − 0.5mm) | parent `SWORD_MOUNTS` L53-L57 |

### Slot A / vertical_post
| emits | 描述 | 来源 |
|---|---|---|
| parts | `display_stand` (root): base box + N× `post_{i}` (mesh `_post_shape`, U-channel) + plaque/frame/kanji | var_vpost L152-L206 |
| internal joints | none (rigid root) | var_vpost |
| upstream interface | base box rests at z≈0 | var_vpost L363 (test) |
| downstream interface | per-sword FIXED mount with lean: xyz=(post_x + lean offset, MOUTH_Y, MOUTH_Z=0.87), rpy=(0,PITCH,0), PITCH=−(π/2−LEAN), LEAN=2° | var_vpost L323-L338 |

### Slot A / wall_mount
| emits | 描述 | 来源 |
|---|---|---|
| parts | `wall_panel` (root, NO base box): `panel_body`, 4 `frame_*`, `plaque_panel` + 4 `plaque_frame_*` + 4 `kanji_stroke_*`, 6 `hook_{i}` (mesh `_hook_shape`, L-bracket + notch + rib) | var_wallmount L170-L237 |
| internal joints | none (rigid root) | var_wallmount |
| upstream interface | panel bottom z≈0.08 (wall-mounted, floats above ground) | var_wallmount L408 (test) |
| downstream interface | per-sword FIXED mount at hook cradle center: xyz=(SWORD_MOUTH_X=0.28, CRADLE_Y=−ARM_REACH, tier_z − SEAT_DROP), rpy=(0,0,0) | var_wallmount L360-L373 |

### swords (the copied object — applies under every Slot A)
| emits | 描述 | 来源 |
|---|---|---|
| parts | `{prefix|sword_i}_saya` (mesh `_saya_tube_shape` + mouth_ring + 2 bands + 3 blossoms + 3 centers + 3 dots) and `{prefix|sword_i}_blade_assembly` (blade_body + habaki_collar + tsuba(SlotB) + tsuba_rim + fuchi_collar + tsuka_grip(SlotC) + kashira_pommel) | parent `_build_saya` L198-L237, `_build_blade_assembly` L239-L297 |
| internal joints | per sword: FIXED `saya_mount` (stand→saya) + PRISMATIC `blade_draw` (saya→blade) | parent L300-L318 |
| upstream interface | saya_mount.origin = mount_xyz(i) from Slot A's downstream interface | parent L303-L309 |
| downstream interface | blade_draw: axis (1,0,0), origin (0,0,0), limits lower=0 upper=TRAVEL=0.70 | parent L310-L318 |

### Slot B / round_disc · square_mokko · pierced_sukashi
| emits | 描述 | 来源 |
|---|---|---|
| parts | one off-axis guard visual on blade_assembly: `tsuba_disc` (Cylinder, round/sukashi mesh) **or** `tsuba_plate` (mokko mesh) + `tsuba_rim` | parent L249-L260 / tsubasq L302-L313 / tsubapierced L308-L321 |
| internal joints | none (rides with blade_assembly) | — |
| interface | concentric with the blade axis at the tsuba station (x≈0.0035), ∅ > 2.2·SAYA_R; no joint, no mount change | parent test L430-L434 |

### Slot C / pink_diamond_ito · smooth_samegawa · cord_ring_bands
| emits | 描述 | 来源 |
|---|---|---|
| parts | `tsuka_grip` cylinder (material pink or samegawa) ± wrap accents: `wrap_diamond_top_{0..2}`+`wrap_diamond_front_{0..1}` (ito) / none (smooth) / `grip_band_{0..K-1}` (cord) | parent L267-L296 / gripsmooth L267-L273 / gripcord L274-L300 |
| internal joints | none | — |
| interface | grip spans GRIP_X0..GRIP_X1 (≈0.25 m) on blade_assembly; wrap accents are module-local visuals only | parent L267-L273 |

## 参数范围汇总

| 参数 | 类型 | 取值范围 / 候选值 | 标称默认 | 约束类型 | 约束 / 函数 | 来源 |
|---|---|---|---|---|---|---|
| `stand_style` | enum | `tiered_crescent_rack` / `vertical_post` / `wall_mount` | — | choice | deterministic procedural sampler (or regression override) | Slot A table |
| `tsuba` | enum | `round_disc` / `square_mokko` / `pierced_sukashi` | — | choice | sampler | Slot B table |
| `tsuka_wrap` | enum | `pink_diamond_ito` / `smooth_samegawa` / `cord_ring_bands` | — | choice | sampler | Slot C table |
| `palette_style` | enum | `black_lacquer_red` / `black_lacquer_pink` / `natural_wood_gold` / `crimson_gold` / `charcoal_steel` | `black_lacquer_red` | choice | reskin only; does not change topology | parent materials L112-L122 |
| `n_swords` | int | `[1, 5]` | 3 | multiplicity | weighted per-N draw (see Multiplicity) | var_n1/n2/n3loop |
| `tier_spacing_scale` | float | `[0.85, 1.20]` | 1.0 | independent | scales the vertical pitch between sword stations; clamp | var_n1 `TIER_SPACING` L58 / var_n2 L55 |
| `saya_length_scale` | float | `[0.90, 1.10]` | 1.0 | independent | uniform-scales SAYA_LEN and the blade loft / travel together (shape-locked) | parent `SAYA_LEN` L32 |
| `blade_travel` | float | derived | 0.70 | equation | `= 0.70 · saya_length_scale · 0.96` (keeps ≥0.010 m retained insertion at full draw) | parent `TRAVEL` L36 |
| `tsuba_diameter_scale` | float | `[0.95, 1.20]` | 1.0 | independent | scales guard diameter; clamp so ∅ stays > 2.2·SAYA_R | parent test L434 |
| `stand_width_scale` | float | `[0.90, 1.15]` | 1.0 | independent | scales base box / panel width to fit N stations; clamp | parent box L133 / vpost POST_XS L69 |
| (—) | constraint | — | — | inequality | `N stations must fit the spine`: station_pitch·(N−1) ≤ usable_span(stand_style)·stand_width_scale; else shrink `tier_spacing_scale` (rack) or pack post/hook X (vpost/wall) | InterfaceSpec, var_vpost POST_XS / var_wallmount HOOK_X |
| (—) | constraint | — | — | inequality | `retained insertion`: blade_travel ≤ SAYA_LEN·saya_length_scale − 0.020; violation → reduce blade_travel | parent draw test L409-L412 |
| (—) | constraint | — | — | conditional | `tiered_crescent_rack` has only 2 cradle tiers + 1 box-top seat (3 native stations); for N>3 fall back to the regular-tier-spacing column layout (var_n1/n2 form) so tier z = TIER_BASE_Z + i·TIER_SPACING; `wall_mount` adds hook tiers; `vertical_post` adds posts along POST_XS | var_n1 L57-L65 |

Sampling contract (resolve_config): sample `independent` scales first; derive `blade_travel`; project N-fit + retained-insertion inequalities (shrink spacing / travel, else reject & resample); resolve the `conditional` station layout from the chosen `stand_style` and N before building.

## Multiplicity / Copy Logic

Single multiplicity axis: **sword count**.

- `count_param`: `n_swords` (template int). Parent uses the hand-tuned 3-key `SWORD_MOUNTS` dict (`top`/`middle`/`bottom`, L53-L57); the template **rewrites that dict into the for-i loop form** taken from `var_n3loop` (`_sword_mount_xyz(i)` over `TIER_X/TIER_Y/TIER_Z` arrays L67-L77, `_mount_sword(...)` L128-L146, single `for i in range(N_SWORDS)` L344-L346) generalized to arbitrary N via the **regular tier spacing** of `var_n1`/`var_n2` (`TIER_BASE_Z + i·TIER_SPACING`, var_n1 L57-L65 / var_n2 L54-L61).
- `N_range`: `[1, 5]` (product domain). Samples cover {1, 2, 3}: var_n1 {1}, var_n2 {2}, var_n3loop {3}, parent {3}. N=4,5 are safe by construction (regular tier spacing + N-fit inequality) and sparsely swept.
- sampling domain (weights): small N high-frequency, larger N rare. Suggested per-N weights over [1,5]: N=1→0.18, N=2→0.27, N=3→0.33, N=4→0.15, N=5→0.07 (mode at the canonical 3-sword set, tail downweighted). Tests bias small N; product domain covers the full range.
- copied object: one full sheathed katana = `{prefix|sword_i}_saya` (built by `_build_saya`) + `{prefix|sword_i}_blade_assembly` (built by `_build_blade_assembly`). **Saya and blade meshes are shared** (built once via `mesh_from_cadquery`, reused per sword); only positions/joints differ.
- naming: index naming `sword_{i}_saya` / `sword_{i}_blade_assembly`, joints `sword_{i}_saya_mount` / `sword_{i}_blade_draw` (var_n3loop form L132/L139). The parent's literal `top`/`middle`/`bottom` prefixes are kept **only** as an optional regression override that reproduces the curated 3-sword layout; the default seed domain uses index names.
- placement: per-stand-style station layout with **regular tier spacing** for the generalizable axis:
  - `tiered_crescent_rack`: stations stack vertically at `tier_z = TIER_BASE_Z + i·TIER_SPACING·tier_spacing_scale` (cradle column); the curated 3-key override reproduces the exact parent `top`/`middle`/`bottom` seats (2 cradles + box top).
  - `vertical_post`: stations along `POST_XS` (X-spread posts), mount xyz = (post_x + lean offset, MOUTH_Y, MOUTH_Z), evenly packed for N.
  - `wall_mount`: stations on hook tiers (`TIER_Z` × `HOOK_X` pairs), evenly stacked for N.
- joint policy (uniform per sword, identical across all stand styles): `sword_{i}_saya_mount` = **FIXED** (stand/panel → saya at mount xyz; rpy=(0,PITCH,0) only for vertical_post lean); `sword_{i}_blade_draw` = **PRISMATIC** (saya → blade_assembly, axis (1,0,0), origin (0,0,0), `MotionLimits(lower=0, upper=blade_travel, effort=25, velocity=0.6)`). Each sword is fully independent (its own FIXED mount + own PRISMATIC draw).
- source/gating: N is clamped to `[1,5]`; the N-fit inequality shrinks station spacing / packs stations before resampling; `tiered_crescent_rack` past its 3 native stations falls back to the regular-spacing cradle column (conditional, var_n1/n2 form).

## 拓扑多样性审计

总组合数：Slot A (3) × Slot B (3) × Slot C (3) × N (5 distinct sword counts) = **135** topology classes (palette excluded as a reskin, not a topology axis).

理由：3×3×3 = 27 distinct slot triples already clear the ≥10 bar without counting N; adding the 5-way multiplicity gives 135 distinct topology equivalence classes. All slot choices are mutually compatible (B and C are blade-local; every A accepts every N).

seed_domain_policy：`procedural_first`.

**Procedural Sampling / Sweep Plan.** `config_from_seed(seed)` seeds a per-seed RNG and: (1) weighted-draws `n_swords` over [1,5] (mode at 3, tail rare); (2) uniform-draws `stand_style`, `tsuba`, `tsuka_wrap`, `palette_style`; (3) samples the `independent` scales, derives `blade_travel`, projects the N-fit + retained-insertion inequalities, and resolves the `conditional` station layout. `slot_choices_for_seed(seed)` returns `[("stand_style", A), ("tsuba", B), ("tsuka_wrap", C)]` plus the resolved `n_swords` (continuous scales are NOT recorded as slot choices since they don't change topology class). No compatibility gate excludes any triple; the only gates are the N-fit / insertion inequalities and the crescent-rack >3 conditional fallback. A small set of regression overrides (≤4 seeds) reproduces the curated parent (3-sword `tiered_crescent_rack` × `round_disc` × `pink_diamond_ito` with the literal `top/middle/bottom` layout) and the three single-axis variant families; these are justified as reviewer-selected canonical cases, NOT the main seed domain.

Topology target：1000-seed slot choice tuple distinct 富类别建议 ≥300（report-only）；with 135 classes and weighted N (downweighting the rare 4/5-sword tail), low results should record the discrete-space ceiling rather than fail a gate.

**Controlled local parameterization.** Key continuous scales: `tier_spacing_scale` [0.85,1.20] (station pitch), `saya_length_scale` [0.90,1.10] (sword length, shape-locked to blade loft + travel via equation), `tsuba_diameter_scale` [0.95,1.20] (guard ∅, clamped > 2.2·SAYA_R), `stand_width_scale` [0.90,1.15] (spine width to fit N). All clamped/derived in `resolve_config`; cross-part dependencies declared as equation (`blade_travel`) and inequality (N-fit, retained-insertion). None changes a declared topology class, the FIXED/PRISMATIC interfaces, or category identity.

| item | policy | validator / viewer focus |
|---|---|---|
| sampler | weighted N over [1,5] (mode 3); uniform over A/B/C/palette; scales sampled then clamped/derived | slot_choices_for_seed matches build choices; resolved n_swords matches emitted sword count |
| compatibility matrix | all A×B×C triples legal; B/C are blade-local; every A accepts every N; only gates = N-fit + retained-insertion inequalities + crescent-rack>3 conditional fallback | no floating sword, no station collision, draw axis/range correct, sheathed pose closes, N stations fit the spine |
| controlled local variation | tier_spacing_scale / saya_length_scale / tsuba_diameter_scale / stand_width_scale, all clamped/derived | proportions vary without breaking saya_mount seat, blade_draw origin/range, tsuba-wider-than-saya, or ~1.0 m sword identity |
| regression overrides | ≤4 seeds: curated parent (3-sword crescent rack, round disc, pink ito, literal top/middle/bottom) + n1/n2/n3loop canonical layouts | previously-authored canonical cases only |
| random sweep | seeds 0-49 initial pass; 0-999 maturity audit | contract failures (mount seat, draw range, sheathed closure, N-fit) |

| slot | candidate_count | 是否 ≥2 | 是否 ≥3 | 备注 |
|---|---:|---|---|---|
| A stand_style | 3 | yes | yes | tiered_crescent_rack / vertical_post / wall_mount |
| B tsuba | 3 | yes | yes | round_disc / square_mokko / pierced_sukashi |
| C tsuka_wrap | 3 | yes | yes | pink_diamond_ito / smooth_samegawa / cord_ring_bands |
| (mult) n_swords | 5 | yes | yes | N ∈ [1,5], weighted mode at 3 |

## Validator

- `slot_choices_for_seed` returns implemented module names for `(stand_style, tsuba, tsuka_wrap)` and the resolved `n_swords`.
- `config_from_seed` uses deterministic procedural sampling (weighted N + uniform slot draws) for all ordinary seeds; `seed=0` is not special.
- compatibility: every A×B×C triple legal; only N-fit / retained-insertion inequalities and the crescent-rack>3 conditional fallback gate the build.
- optional regression overrides are sparse (≤4) and justified (curated parent + n1/n2/n3loop canonical layouts).
- controlled local scales (tier_spacing_scale, saya_length_scale→blade_travel, tsuba_diameter_scale, stand_width_scale) are clamped/derived in `resolve_config` and cannot break the saya_mount seat, blade_draw origin/axis/range, tsuba clearance, or N-station fit.
- critical InterfaceSpec/MatingContract points exist: saya seated on the chosen spine (`expect_contact` saya↔stand), blade nested in saya bore at q=0, blade retains ≥0.010 m insertion at full draw.
- key joints: per sword, FIXED `sword_{i}_saya_mount` + PRISMATIC `sword_{i}_blade_draw` (axis (1,0,0), lower=0, upper=blade_travel).
- copied objects follow naming (`sword_{i}_saya` / `sword_{i}_blade_assembly`) and per-stand-style placement; saya/blade meshes shared (built once).

## Reject cases

- A sword floats off the rack (saya not seated in any cradle / post channel / hook notch — `expect_contact` saya↔stand fails).
- The blade is not retained at full draw (blade_travel ≥ saya bore length, blade exits the scabbard — retained-insertion inequality violated).
- `blade_draw` authored as REVOLUTE / wrong axis / non-zero origin, or the saya_mount made movable (must be FIXED).
- N stations overrun the spine (station pitch·(N−1) > usable span → posts/hooks/cradles collide or hang off the base) — N-fit inequality not applied.
- tsuba authored on-axis or ∅ ≤ 2.2·SAYA_R (guard no wider than the saya — loses the off-axis-disc identity test).
- Crescent-rack used for N>3 without the regular-tier-spacing fallback (only 2 cradles + 1 box-top seat natively → swords with no station).
- A single in-hand katana with no rack, or a folding/utility combat knife — wrong identity (no display set, no scabbard-draw).
- Slot enum offers an unimplemented module, or `config_from_seed` cycles a small curated/modulo table instead of weighted procedural sampling.

## 与相邻类别的边界

- 不该混入：`retractable_utility_knife` / `military_knife`（折叠或固定战术/工具刀，blade-pivot 或 slider 机构，手持，无 rack、无 scabbard-draw — 与本模板的 sheathed-saya + rack-display identity 冲突）。
- 不该混入：单把无展示架的 in-hand katana（本模板 identity 是 **display set** — rack 承载 N 把 sheathed 刀；缺 rack 即失去 FIXED saya_mount 接口与多样性轴）。
- 不该混入：纯装饰刀架雕塑或无 articulated draw 的静态摆件（每把刀必须有真实 PRISMATIC blade_draw）。

## 审核记录

| 项 | 结论 |
|---|---|
| reviewer status | pending |
| reviewer notes | — |

## 模板实现备注（可选）

- Shared helpers: `_saya_tube_shape`, `_ring_shape`, `_blade_shape` are identical across all 10 sources — build once, reuse. `_build_saya` is stand-style-independent. `_build_blade_assembly` takes the Slot B (tsuba) + Slot C (tsuka_wrap) selections and branches the guard mesh (`tsuba_disc` Cylinder / `_tsuba_sukashi_shape` mesh / `_mokko_tsuba_shape` `tsuba_plate`) and grip accents (diamonds / none / `grip_band_{i}` rings).
- The for-i sword loop-rewrite (replacing the parent's literal 3-key `SWORD_MOUNTS` dict) is mandatory; adopt `var_n3loop`'s `_sword_mount_xyz(i)` + `_mount_sword(...)` structure, but generalize `TIER_*` to N via `var_n1`/`var_n2`'s regular tier spacing so N=4,5 are well-defined.
- `vertical_post` is the only stand_style with a non-zero mount rpy (the 2° lean, `PITCH=−(π/2−LEAN)`); its sheathed pose is **near-vertical** (Z extent dominates) — pose tests must branch on stand_style (horizontal vs near-vertical sword).
- allow_overlap declarations are element-scoped and per-stand-style: saya_tube ↔ {pillar_i | post_i | hook_j} seating overlap; blade_body ↔ saya_tube sheathed-insertion overlap; vertical_post additionally allows band_i ↔ post_i. Replicate all of these per sword when implementing tests.
- `wall_mount` has NO base box and floats above ground (panel bottom z≈0.08); its grounding test asserts the panel/saya are above ground, opposite to the base-box racks. Validator must branch grounding on stand_style.
