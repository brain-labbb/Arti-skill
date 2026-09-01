# Kitchen / Knife set — template source map

pattern: multiplicity (per-N knife stack) with fixed named slots layered on
(effectively mixed: the dominant axis is `knife_count`, with block-form, holding-
mechanism, and base/stand slots layered on)

parents:
- rec_model-a-kitchen-knife-block-set-a-slanted-light-_20260610_080923_805616_1d3b3a0e ← picture/Kitchen/Knife set/001.png (slanted light-oak block leaning back 12°, on four dark rubber feet, engraved logo on the front pocket face, 6 knives in angled top slots + a kitchen shears in a wide front pocket)

Kitchen knife block set. Shared kinematics for every candidate: a `knife_block`
root (block_shell + four `foot_{i}` + `logo_seal`/`logo_text` decorations) that
holds N knives, each a removable child on its own **PRISMATIC** slot-slide, plus a
two-part kitchen shears that slides out (`shears_slide` PRISMATIC) and opens at its
central rivet (`shears_pivot` REVOLUTE between `shears_inner_half` and
`shears_outer_half`). The block form, the holding mechanism, and the base/stand are
the three independent structural slots below; the knife count is the multiplicity
axis. The shears slide+pivot is preserved in every source as a second
non-fixed-joint anchor, but the per-knife PRISMATIC slot-slide is the multiplicity
copy unit and the guaranteed ≥1 non-fixed joint.

> Coordinate / readability note: the PARENT emits its six knives from a **hand-
> written `KNIVES` dict** (keys `chef_knife` / `bread_knife` / `santoku_knife` /
> `utility_knife` / `paring_knife_0` / `paring_knife_1`), iterated with
> `for kname, spec in KNIVES.items()` and named after the dict key — it is NOT a
> `knife_{i}` index loop. Per FORK_VARIANTS.md §2.0 the multiplicity template must
> use the refactored **loop form `for i in range(N): knife_{i}`** with a shared
> knife geometry helper and a uniform per-knife PRISMATIC policy. The
> loop-emitted variants below (count_4, count_8, block_upright, block_horizontal_bar,
> base_pedestal) are the canonical copy-logic samples; base_metal_stand and the
> parent keep the dict form and should be read for structure only, not copied as
> the emission template.

## Slot 候选覆盖

### Slot A:block form / silhouette (`knife_block` body + slot axis)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| slanted_block [P] | parent (…_1d3b3a0e) | `knife_block` part / `block_shell` visual (`_build_block_solid` extruded XZ wedge, `_tilted_box` slot cutters) / `{knife}_slide` PRISMATIC axis (0,0,1) with `rpy=(0,-TILT,0)` 12° lean | leaning-back oak block, knives ride angled top slots parallel to the block axis; front pocket holds the shears | converged |
| upright_block | rec_knife_set_var_block_upright | `knife_block` / `block_shell` (axis-aligned `box(BLOCK_D,BLOCK_W,BLOCK_H)`, vertical slot cutters) / `knife_{i}_slide` PRISMATIC axis (0,0,1) no tilt; `shears_slide` axis (1,0,0) out the front face | upright vertical block, knives draw straight up; shears slides out a horizontal front slot (`_shears_steel_h` rotated 90°) | converged |
| horizontal_bar | rec_knife_set_var_block_horizontal_bar | `holder` / `holder_shell` (long YZ wedge `extrude(HALF_L, both)`, 0.46 m long, 15° slots via `_make_slot_cutter`) / `knife_{i}_slide` PRISMATIC axis (0,0,1) with `JOINT_ROLL` 75° roll | long low countertop bar (~0.46×0.22 m), knives lie at a shallow 15° in a single row; `_knife_x(i)` even X spacing | converged |

### Slot B:holding mechanism (knife retention inside the block)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| angled_prismatic_slots [P] | parent (…_1d3b3a0e) | `block_shell` cut by `_tilted_box(slot_w, SLOT_THICK, …)` per knife / each `{knife}_slide` PRISMATIC | solid-block individual milled slots, one tight rectangular slot per blade; knife held by clearance fit + gravity | converged |
| bristle_insert | rec_knife_set_var_mech_bristle | `knife_block` = `housing_shell` (hollow smoked-acrylic, `_build_housing`+`_build_front_pocket`) + `bristle_{i}` grid (loop `for i in range(N_BRISTLES)`, `_build_*`/`Cylinder` rods) / single `chef_knife` part on `chef_slide` PRISMATIC; other 5 knives inline FIXED block visuals (`{name}_steel`/`{name}_grip` from `FIXED_KNIVES`) | universal bristle block: dense flexible-rod grid the blades push between; one knife stays articulated (chef_slide) for the ≥1-joint floor | converged |

### Slot C:base / stand (what the block sits on)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| rubber_feet [P] | parent (…_1d3b3a0e) | `foot_{i}` Cylinder visuals on `knife_block` (loop `for i,(fx,fy) in enumerate(foot_xy)`); block is the root, sits at z=0 | four dark rubber feet directly under the block; no separate base part | converged |
| sculptural_pedestal | rec_knife_set_var_base_pedestal | `pedestal` part / `pedestal_column` (`_build_pedestal` CadQuery `revolve(360)` turned bronze column, base plinth→entasis shaft→capital) + `base_pad_{i}` / `pedestal_to_block` FIXED (origin z=PEDESTAL_TOP) | block FIXED-mounted atop a tall turned pedestal column; raises the set ~0.13 m | converged |
| cast_metal_stand | rec_knife_set_var_base_metal_stand | `cast_metal_stand` part / `stand_frame` (`_build_stand`: filleted base plate + tilted `left`/`right` side walls + tilted `back` support + cast `gusset` ribs) + `stand_front_lip` / `stand_to_block` FIXED (origin z=0.012) | block cradled in a heavy cast-metal angled stand; `expect_contact` of `foot_{i}` on the plate | converged |

## Multiplicity / Copy Logic
- count_param: `knife_count` (sources call it `NUM_KNIVES` / `N_KNIVES`; parent has a
  hand-written 6-entry `KNIVES` dict). This is the dominant multiplicity axis: each
  knife is one independent PRISMATIC slot-slide, so non-fixed joint counts run N+2
  across variants (N knife slides + `shears_slide` + `shears_pivot`); never 0.
- N 样本已覆盖: {4, 6, 8} → rec_knife_set_var_count_4 (NUM_KNIVES=4: chef/bread/utility/paring, two rows of two) / parent (6: 3 back + 3 front) / rec_knife_set_var_count_8 (N_KNIVES=8: 4 back + 4 front, block widened to BLOCK_WIDTH=0.175 to fit).
- 模板建议 N_range: [3, 8] (small 3-knife starter set up to a full 8-slot block;
  block width / row layout auto-fits — count_8 widens BLOCK_WIDTH and respaces the
  `y` positions, cheeks/floor follow). Sample coverage {4,6,8} only demonstrates the
  copy logic; the sampler fills 3,5,7. Each different N is one topology.
- copied object: one knife = a `knife_{i}` part with `{knife}_steel` (tapered blade +
  bolster, `_knife_steel`), `{knife}_grip` (lofted walnut handle, `_knife_grip`), and
  two `{knife}_rivet_{j}` Cylinders, built by the shared `_knife_steel` / `_knife_grip`
  / `_knife_rivet_x` helpers (count_4 wraps them in `_build_knife_part`; count_8 in
  `_build_knife`).
- naming: `knife_{i}` part, `knife_{i}_steel` / `knife_{i}_grip` / `knife_{i}_rivet_{j}`
  visuals, `knife_{i}_slide` joint (0-based i). PARENT instead names parts after the
  `KNIVES` dict key (chef_knife / paring_knife_0 …) — template must use the `knife_{i}`
  index form (see readability note).
- placement: knives sit in 1–2 rows of slots on the block top; per-knife (x, y) slot
  center drives both the cut cutter and the joint origin (slanted: `BACK_ROW_M` /
  `FRONT_ROW_M` mouth points + per-knife `y`; upright: `KNIFE_POSITIONS`; horizontal:
  `_knife_x(i)` even spacing). Blade lengths/widths taper down the set (chef largest →
  paring smallest), carried in `KNIFE_SPECS`.
- joint policy: each knife is an INDEPENDENT PRISMATIC joint, parent=`knife_block`,
  axis=(0,0,1) in the knife local frame (the joint `rpy` rotates that local +Z onto the
  withdrawal direction — slanted block uses `rpy=(0,-TILT,0)`, upright uses no tilt,
  horizontal uses `JOINT_ROLL`≈75° about X), `MotionLimits(lower=0.0,
  upper=spec["travel"]≈0.10–0.18, effort=20, velocity=0.5)`. Not chained, not on a
  shared hub — every knife slides out of its own slot independently. Travel scales with
  blade length.
- emission confirmed:
  - count_4 = `for i in range(NUM_KNIVES): _build_knife_part(model, block, i, KNIFE_SPECS[i], …)` → `knife_{i}` (loop ✓)
  - count_8 = `for i in range(N_KNIVES): _build_knife(model, block, i, KNIFE_SPECS[i], …)` → `knife_{i}` (loop ✓)
  - block_upright = `for i in range(NUM_KNIVES):` inline `knife_{i}` (loop ✓)
  - block_horizontal_bar = `for i in range(N_KNIVES):` inline `knife_{i}` (loop ✓)
  - base_pedestal = `for i, spec in enumerate(KNIFE_SPECS):` named from `spec["name"]` (loop over list ✓, but parts named chef_knife/… not knife_{i})
  - base_metal_stand = `for kname, spec in KNIVES.items():` (dict form, hand-named — like parent, NOT the index loop)
  - PARENT = `for kname, spec in KNIVES.items():` (dict form, hand-named — NOT a loop over a count)
  → Canonical copy-logic samples for the template are **count_4 / count_8** (and
  block_upright / block_horizontal_bar for the cross-form loop): they prove the
  `for i in range(N)` + `knife_{i}` + shared `_knife_steel`/`_knife_grip` helper +
  uniform PRISMATIC policy contract. The dict-form sources (parent, base_metal_stand)
  must be refactored to the index loop when authored as a template.

Secondary (non-knife) copy loops present for the template author to reuse:
- feet: `for i, (fx,fy) in enumerate(foot_xy)` → `foot_{i}` (4 corners, inline FIXED visuals).
- bristle grid (bristle_insert): `for i in range(N_BRISTLES)` → `bristle_{i}` inline rods
  (a count axis on the holding-mechanism candidate, not jointed).
- pedestal pads / stand gussets: `for i in range(4)` → `base_pad_{i}`; nested
  `for y_sign … for x_off …` cast `gusset` ribs (inline FIXED).
- shears: two halves (`shears_inner_half` / `shears_outer_half`), each with grip rivets
  emitted via `for j, (…)` — a fixed 2-part assembly, not the multiplicity axis.

## 组合数预审
Slot A (3: slanted_block / upright_block / horizontal_bar) × Slot B
(2: angled_prismatic_slots / bristle_insert) × Slot C (3: rubber_feet /
sculptural_pedestal / cast_metal_stand) = 18 base topologies.
× knife_count N samples (3, e.g. {4,6,8}) → 18 already clears the
full [3,8] range. Every slot has ≥2 candidates (A=3, B=2, C=3). pattern = multiplicity
with fixed named slots layered on. Even ignoring multiplicity, 3×2×3=18 ≥ 10 ✓.

## 排除项(未来 compatibility matrix 素材)
- bristle_insert (Slot B) was forked off the parent at N=6 but only ONE knife stays
  articulated (`chef_slide`); the other five are inline FIXED block visuals because
  loose rods cannot each anchor an independent prismatic child. So bristle_insert
  does NOT carry the full per-N PRISMATIC copy logic — the knife_count multiplicity
  axis is exercised on the angled_prismatic_slots mechanism (count_4 / count_8) only.
  Note for the compatibility matrix: bristle_insert × large N means many fixed visual
  knives + one slide, not N slides.
- horizontal_bar lays knives in a single row (`_knife_x(i)`), while slanted/upright
  use two rows (`row="back"/"front"`); the block-form × row-layout interaction is a
  template detail (single-row vs two-row placement is a function of N and block_form),
  left to the authoring helper, not separately sampled.
- base_pedestal / cast_metal_stand are pure base swaps (block + knives + shears kept at
  the parent 6-knife baseline, mounted via a FIXED `*_to_block` joint); they are
  compatible with the full N range and any Slot A/B since the base only changes what is
  under the block.
- No converged variant combines a non-parent Slot A with a non-feet Slot C, or a
  bristle mechanism with a pedestal/stand base — single-axis control means those combos
  (e.g. upright_block × cast_metal_stand, bristle_insert × pedestal) are left to the
  sampler.
- Pure dimensional knobs (block W/D/H, slot/travel sizes, tilt angle, blade taper) are
  NOT slots — they are the template's continuous parameters (controlled local
  parameterization). The 12° lean and 15° shallow angle are the slanted vs horizontal
  block-form difference, captured in Slot A, not as a separate angle slot.

---
note: 母资产 picture/Kitchen/Knife set/001.png covers the slanted_block ×
angled_prismatic_slots × rubber_feet × N=6 baseline. All variants are fork children of
the parent (workbench-only), each single-axis except the count_4/count_8 multiplicity
pair (which only change knife_count). Every candidate keeps ≥1 non-fixed joint: the
per-knife PRISMATIC slides (N of them) plus the shears `shears_slide` PRISMATIC +
`shears_pivot` REVOLUTE; bristle_insert keeps `chef_slide` + the two shears joints.
