# Kitchen / Dish washer — template source map

pattern: mixed (defining motion is the opening mechanism — either the drop-down DOOR
REVOLUTE or the dish-DRAWER PRISMATIC — with a per-N stack of sliding racks layered on as
the multiplicity axis; the control+handle layer is a third fixed named slot)

parents:
- rec_model-a-freestanding-stainless-steel-dishwasher-_20260610_080821_417246_c2d628fd ← picture/Kitchen/Dish washer/001.png (freestanding stainless dishwasher: bottom-hinged drop-down door, dark top-edge control strip with recessed pocket handle, two sliding chrome wire racks). Covers the baseline cell: Slot A=drop_down_door × Slot B=wire_grid × Slot C=front_fascia × rack_count=2.

Freestanding dishwasher. Shared spine for the door-based candidates: a hollow `cabinet`
(root) — recessed `kick_plinth`, brushed shell (`side_wall_{0,1}` / `back_wall` /
`cabinet_floor` / `cabinet_ceiling`), polished tub liner (`tub_wall_{0,1}` /
`tub_back_panel` / `tub_floor`), side `rack_rail_{i}` glide rails, and an overhanging
`top_slab`. The opening mechanism, the rack basket geometry, and the control+handle are
the three independent structural slots; the sliding-rack stack is the multiplicity axis.
Coordinate convention shared by all sources: X = width, Y = depth (front = +Y), Z = up;
grounded at z = 0. Helpers `_build_cabinet` / `_build_door` / `_populate_rack` (+
`_linspace`) recur across every door-based source; the drawer candidate refactors the same
parts into one moving body.

## Slot 候选覆盖

### Slot A:opening mechanism (defining motion)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| drop_down_door | parent (…_c2d628fd) | `front_door` part (`door_panel`); `door_hinge` REVOLUTE (parent=`cabinet`, axis ≈ -X, 0→π/2) at bottom front edge `(0, BODY_FRONT_Y, PLINTH_H)`; racks are SEPARATE parts on their own PRISMATIC slides | bottom-hinged door folds forward/down to horizontal; tub stays fixed; racks slide out independently over the folded door | converged |
| dish_drawer | rec_dish_washer_var_door_drawer_dishdrawer | `drawer` part (`drawer_panel` + `tub_wall_{i}` + `tub_floor` + `drawer_runner_{i}` + `rack_*` wires) carried as ONE body; `drawer_slide` PRISMATIC (parent=`cabinet`, axis +Y, 0→DRAWER_TRAVEL≈0.42) at `(0, BODY_FRONT_Y, DRAWER_BOTTOM_Z)`; cabinet adds `lower_front_panel` + `cabinet_rail_{i}` channels; `allow_overlap` runner-in-rail telescope | the whole tub assembly (front panel + liner + rack + runners) pulls out as a single drawer; NO revolute, NO separate sliding racks (the rack is fixed inside the drawer body) | converged |

### Slot B:rack basket geometry
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| wire_grid | parent (…_c2d628fd) | `_populate_rack(...)`: `floor_rod_y_{i}` / `floor_rod_x_{i}` / `top_rail_xy_{i}` / `top_rail_side_{i}` / `side_wire_{k}_{j}` / `face_wire_{k}_{j}`, all Box rods; support arm = `wheel_{idx}` (lower) or `side_runner_{k}` (upper) | open chrome wire-frame basket built from many thin Box rods; lands every wire on another wire | converged |
| fused_basket | rec_dish_washer_var_rack_geom_fused_basket | `_build_basket_mesh(w,d,h)` → single CadQuery solid `basket_body` (tray floor + 4 perimeter walls + `rarray` cylindrical tine grid) via `mesh_from_cadquery`; same `wheel_{idx}` / `side_runner_{k}` supports | molded plastic-coated basket as one fused mesh with integrated tine prongs instead of a wire lattice; uses real CadQuery geometry (primitive upgrade) | converged |

### Slot C:control + handle (door/front-panel upper strip)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| front_fascia | parent (…_c2d628fd) | `control_strip` (front-proud strip) + recessed `pocket_handle` + `display_lcd` + `button_{i}` (loop), all on the door front face | dark control strip across the door top edge, proud of the front face, with an inset pocket-handle groove; controls visible head-on when closed | converged |
| top_hidden | rec_dish_washer_var_door_top_control | `control_panel` recessed into the door TOP edge (z≈DOOR_H face) + `pocket_handle` + `display_lcd` + `button_{i}` on the top face; tests assert controls below the slab underside when closed, exposed forward when open | hidden top-lip control panel: clean brushed front when closed, controls rotate into view when the door opens | converged |
| proud_bar_handle | rec_dish_washer_var_control_body_fascia_handle | keeps `control_strip` + `display_lcd` + `button_{i}`; replaces the pocket groove with a full-width `bar_handle` Cylinder on two `handle_standoff_{i}` brackets standing off the face (>0.40 m span, >0.02 m standoff) | proud tubular bar handle (the grab-bar fascia look) instead of a recessed pocket; adds standoff brackets | converged |

## Multiplicity / Copy Logic
- count_param: `rack_count` (the door-based sources call it `NUM_RACKS` over `RACK_CONFIGS`,
  or iterate `RACK_SPECS`). This is the dominant multiplicity axis on the **drop_down_door**
  candidate: each rack = one independent PRISMATIC slide, so non-fixed joint count = 1 (door
  hinge) + rack_count. (The **dish_drawer** candidate has rack_count effectively 1 — a single
  fixed rack rides inside the one pull-out drawer body — so its multiplicity is the trivial
  endpoint; the per-N rack stack belongs to the door family.)
- N 样本已覆盖: {2, 3} →
  - N=2 (loop form): rec_dish_washer_var_door_handle_recessed_tall — `RACK_SPECS` list of 2 dicts, `for i, spec in enumerate(RACK_SPECS)` emits `rack_0` (tall deep lower, wheels) + `rack_1` (removable upper half-rack, runners), each on a `{name}_slide` PRISMATIC. THIS is the loop-emit reference for N=2.
  - N=3: rec_dish_washer_var_racks_3_cutlery — `RACK_CONFIGS` list of 3 + `NUM_RACKS`, `for i in range(NUM_RACKS)` emits `rack_{i}` parts (rack_0 lower/wheels, rack_1 upper/runners, rack_2 shallow cutlery tray/runners) + `rack_slide_{i}` PRISMATIC; runner rails also loop-emitted as `rack_rail_{rail_idx}`.
  - The PARENT itself is N=2 but **hand-written** (`upper_rack` / `lower_rack` via two separate `model.part()` + `_populate_rack` calls, joints `upper_rack_slide` / `lower_rack_slide`) — NOT a loop. **The template must use the loop form** (the two variants above), not the parent's two named racks.
- 模板建议 N_range: [2, 3] (real freestanding dishwashers run a lower rack + upper rack,
  optionally a third shallow cutlery tray on top; >3 stacked sliding racks is not a real
  configuration, so this axis is deliberately narrow). Sample coverage {2,3} already spans
  the full suggested range.
- copied object: one rack = a `rack_{i}` part holding the basket geometry (Slot B module) plus
  its support arm (`wheel_{idx}` ×4 for the bottom wheeled rack, `side_runner_{k}` ×2 for the
  upper runner racks), built by the shared `_populate_rack` helper.
- naming: `rack_{i}` part, `rack_slide_{i}` joint (racks_3 form) or `rack_{i}` + `{name}_slide`
  (handle_recessed_tall form); the template should standardize on `rack_{i}` / `rack_slide_{i}`.
- placement: stacked bottom→top; rack_0 sits on the tub floor (wheels), each higher rack on its
  own pair of side rails at increasing z; the prismatic origin z = each rack's resting height,
  x = 0, y = small fixed inset (`rack_yc ≈ -0.01`).
- joint policy: each rack is an INDEPENDENT PRISMATIC slide, parent=`cabinet`, axis=(0,1,0)
  (+Y pull-out), `MotionLimits(lower=0.0, upper=RACK_TRAVEL≈0.45, effort=20, velocity=0.5)`.
  Not chained, not on a shared hub — every rack slides out of the tub on its own. The bottom
  rack rolls on `wheel_{idx}`; the upper racks glide on `side_runner_{k}` over `rack_rail_*`.

Secondary copy loops present for the template author to reuse:
- buttons: `for i, bx in enumerate((...))` emits `button_{i}` on the control strip (all Slot C variants).
- side walls / rails: `for sx, tag in ((-1.0,...),(1.0,...))` symmetric pairs (`side_wall_{0,1}`, `tub_wall_{0,1}`, `rack_rail_{0,1}`, `handle_standoff_{i}`, `drawer_runner_{i}`, `cabinet_rail_{i}`).
- wire lattice / tine grid / runner rails: inline loops inside `_populate_rack` / `_build_basket_mesh` / `_build_cabinet`.

## 组合数预审
Slot A (2: drop_down_door / dish_drawer) × Slot B (2: wire_grid / fused_basket) ×
Slot C (3: front_fascia / top_hidden / proud_bar_handle) = 12 base topologies ≥ 10 ✓.
× rack_count N ∈ {2,3} on the door family further multiplies the door-based column. Every
slot has ≥2 candidates (A=2, B=2, C=3). pattern = mixed (opening-mechanism slot + rack
multiplicity axis + control/handle slot).

## 排除项(未来 compatibility matrix 素材)
- The MULTIPLICITY (per-N sliding rack stack) only applies to the **drop_down_door** Slot A
  candidate. The **dish_drawer** candidate carries a single fixed rack inside the one drawer
  body, so rack_count on a drawer is structurally the trivial endpoint (N=1) — note for the
  compatibility matrix that rack_count × dish_drawer is not a meaningful combination, while
  rack_count ∈ [2,3] is the normal domain for drop_down_door.
- The **top_hidden** Slot C control candidate depends on the drop-down door rotating to expose
  the controls (its tests assert controls hidden under the slab when closed, forward when the
  door opens). On a dish_drawer front panel the controls are simply on the proud panel face, so
  top_hidden × dish_drawer would need re-anchoring — left to the sampler / matrix.
- All single-axis variants were forked off the parent baseline (door × wire_grid × front_fascia
  × N=2). No converged variant combines two non-baseline slots at once (e.g. dish_drawer ×
  fused_basket, or top_hidden × N=3) — those cross-axis combinations are sampler products, not
  separately forked here.
- Pure dimensional knobs (cabinet W/D/H, rack travel, basket height/depth, tine density,
  handle bar diameter) are NOT slots — they are the template's continuous parameters
  (controlled local parameterization). The "tall lower rack + shallow half-rack" sizing in
  handle_recessed_tall is incidental dimensional variety; that record's structural contribution
  is the N=2 LOOP EMISSION form, not its proportions.

---
note: parent picture/Kitchen/Dish washer/001.png covers the drop_down_door × wire_grid ×
front_fascia × rack_count=2 baseline. The six variants are single-axis (or N-only) fork
children of the parent (workbench-only), each holding ≥1 non-fixed joint. The template must
emit racks via the `for i` loop form (racks_3_cutlery / handle_recessed_tall), NOT the
parent's two hand-written `upper_rack` / `lower_rack` parts.
