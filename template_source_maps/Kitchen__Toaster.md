# Kitchen / Toaster — template source map

pattern: multiplicity (slot/shelf count is the dominant copy axis) + named slots
(browning-control / lever-placement / crumb-tray / body-silhouette are fixed
structural slots layered on the per-N bread-slot stack → effectively mixed)

parents:
- rec_model-a-compact-two-slice-pop-up-toaster-amazonb_20260610_081022_324275_d8fcb465 ← picture/Kitchen/Toaster/001.png (compact 2-slice AmazonBasics-style pop-up toaster: square matte-gray box, front control panel, push-down carriage lever, rotary browning dial, three push buttons; covers slot_count=2 × rotary-dial × front-lever × no-tray × square-body baseline)

Pop-up toaster. Shared spine for every candidate: a single fused `body` (root)
carcass — `shell` (CadQuery hollow box: internal `cavity` cut + per-slot top
openings + recessed `slot_rim_plate` + front-wall lever slot / button holes /
control holes), a brushed-silver `control_panel` visual on the +X end face, four
`foot_{i}` cylinders, and `dial_mark_{i+1}` + `brand_strip` panel decorations
(all inline `parent.visual(...)`, no FIXED-joint decoration parts). The defining
motion is the **`carriage_lever` PRISMATIC** joint (`body_to_carriage_lever`,
axis (0,0,-1), travel 0.070 m straight DOWN, `bread_shelf_*` shelves visible
through the top slots) — identical in all 8 sources. The browning control, the
lever placement, the crumb tray, and the body silhouette are the four named
structural slots; the bread-slot/shelf count is the multiplicity axis.
Coordinate convention shared by all sources: +Z up, body long axis along X,
brushed-silver control panel on the +X front face; looking at the panel (along
-X), +Y is the viewer's right, -Y the viewer's left.

## Slot 候选覆盖

### Slot A：browning control (mounted on the +X `control_panel`)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| rotary_dial | parent (…_d8fcb465) | `browning_dial` part (`dial_cap` KnobGeometry + `dial_shaft` + off-axis `dial_pointer_nub`); `body_to_browning_dial` REVOLUTE axis (1,0,0), ~270°; panel `dial_mark_{i+1}` ticks | dark-gray knob rotating about the horizontal +X panel normal; nub sweeps to prove continuous rotation | converged |
| slider | rec_toaster_var_browning_slider | `browning_slider` part (`slider_tab` + `slider_stem` + `slider_carriage_plate`); `body_to_browning_slider` PRISMATIC axis (0,0,1), travel 0.044; vertical `track_slot` cut in panel/wall; `slider_mark_{i+1}` ticks via `for i in range(4)` | dark tab on a vertical track, slides UP = high browning | converged |
| digital | rec_toaster_var_browning_digital | `browning_button_{i}` parts (i∈{0,1} = UP/DOWN) with `browning_pad_{i}` / `browning_stem_{i}` / `browning_indicator_{i}`; `body_to_browning_button_{i}` PRISMATIC axis (-1,0,0), travel 0.003; inline `digital_display` + `display_bezel` panel visuals; loop `for i in range(2)` | dark LCD inset flanked by UP/DOWN push pads (2 prismatic buttons) instead of an analog control | converged |

### Slot B：carriage-lever placement (the defining push-down mechanism)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| front_lever | parent (…_d8fcb465) | `carriage_lever` part (`lever_knob` + `lever_stem` + `carriage_crossbar` + `bread_shelf_*`); `body_to_carriage_lever` PRISMATIC axis (0,0,-1); lever slot cut in front `control_panel` + front wall; knob seats on `control_panel` | push-down knob rides the front-panel slot | converged |
| side_lever | rec_toaster_var_lever_side | same `carriage_lever` / `body_to_carriage_lever` PRISMATIC axis (0,0,-1) UNCHANGED, but slot + knob relocated to the +Y side wall; new `lever_guide_plate` body visual; `_side_guide_shape()` helper; knob seats on `lever_guide_plate` | lever moved from front face to the +Y side wall; panel loses its lever slot | converged |

> Slot B is the **defining-motion slot** (carriage PRISMATIC). Only the mount face
> moves (front-panel slot ↔ +Y-side-wall slot + guide plate); the joint type, axis
> (0,0,-1), and 0.070 m travel are invariant across both candidates.

### Slot C：crumb tray (mounted in the shell floor pocket)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| none | parent (…_d8fcb465) | (no tray part; sealed shell floor) | baseline — no removable tray | converged |
| pullout | rec_toaster_var_crumb_tray_pullout | `crumb_tray` part / `tray_body` visual (`_crumb_tray_shape()`: base plate + 3 side lips + pull-handle tab); `body_to_crumb_tray` PRISMATIC axis (-1,0,0), travel 0.180; shell-floor `tray_pocket` cut; retains insertion when extended | flat crumb tray slides out the -X end below the cavity | converged |

> Slot C is a 1→2 candidate axis (none vs pull-out). For SPEC_TEMPLATE.md
> (no single-candidate slots), either fold `none` in as the disabled state of the
> pull-out module, or treat tray-present/absent as a boolean feature flag of Slot C.

### Slot D：body silhouette (`shell` primitive family)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| square_box | parent (…_d8fcb465) | `_shell_shape()` = `cq.Workplane.box(...)` + filleted vertical/top/bottom edges; `body_gray` matte material | classic modern squared box with rounded corners | converged |
| retro_round | rec_toaster_var_body_retro_round | `_shell_shape()` = 3-section `slot2D` (base→barrel→dome) `.loft()`; `chrome_body` polished material; cavity/slots/recess/holes boolean-cut from the lofted solid | curved barrel-dome retro chrome silhouette (a real lathe/loft primitive, not a boxy placeholder) | converged |

## Multiplicity / Copy Logic
- count_param: `slot_count` (sources express it as `NUM_SLOTS` in the 4-slice
  variant; the parent has 2 hand-tupled slots; long-single is the N=1 degenerate).
  Each bread slot is one shelf on the SINGLE shared carriage — the carriage itself
  is one PRISMATIC joint regardless of N (the shelves are visuals on it, not extra
  joints), so this is a **visual/cut multiplicity**, not a per-N joint multiplicity.
- N 样本已覆盖: {2, 4, 1} →
  - 2 = parent (…_d8fcb465): shell/rim slot cuts via `for yc in (SLOT_YC, -SLOT_YC)`; shelves via `for tag, yc in (("right", SLOT_YC), ("left", -SLOT_YC))` → `bread_shelf_right` / `bread_shelf_left`.
  - 4 = rec_toaster_var_slot_4slice: `NUM_SLOTS=4`; `SLOT_POSITIONS` built by a nested loop over `SLOT_PAIR_XC × (±SLOT_YC)`; shell + rim cuts and shelves all emitted via `for i in range(NUM_SLOTS)` → `bread_shelf_{i}`; body lengthened to 0.396 m; added X-spanning `side_rail_{side_tag}` spine.
  - 1 = rec_toaster_var_slot_long_single: single wide baguette slot; one `bread_shelf` (no loop); narrower cavity.
- 模板建议 N_range: **[2, 4]** (suggested model domain — covers the real toaster
  range: 2-slice common, 4-slice common, and a single long-slot/baguette form as
  N=1; the sampler fills the rest). The long-single N=1 is a real product but reads
  as a distinct silhouette; the template can model it either as N=1 or as a separate
  "long_slot" form flag.
- copied object: one bread slot = a top `shell`/`rim` opening cut + one
  `bread_shelf_*` carriage visual under it (no extra joint per slot).
- naming: range-indexed `bread_shelf_{i}` (4-slice) is the readable form; parent's
  `bread_shelf_right`/`_left` 2-tuple loop and long-single's bare `bread_shelf`
  are the N=2 / N=1 endpoints. Template should standardize on `bread_shelf_{i}`.
- placement: slots laid out on the top deck (2-slice: ±SLOT_YC pair; 4-slice:
  2 X-pairs each ±SLOT_YC); each shelf centered under its slot at the carriage
  rest height, inside the cavity.
- joint policy: ALL shelves ride the ONE shared `carriage_lever` (single
  `body_to_carriage_lever` PRISMATIC, axis (0,0,-1), travel 0.070,
  `MotionLimits(effort=15, velocity=0.3, lower=0, upper=0.070)`). The carriage is
  NOT chained or per-slot-jointed — every shelf moves together with the single
  push-down carriage. Slot count changes the visual/cut count, not the joint count.
- emission confirmed: **slots/shelves ARE loop-emitted.** 4-slice = real
  `for i in range(NUM_SLOTS)` over a `SLOT_POSITIONS` list (shell cuts, rim cuts,
  shelves all looped); parent = a 2-element `for ... in (tuple)` loop (semi-loop,
  `_right`/`_left` tagged); long-single = N=1 single statement. The readability
  contract holds in the 4-slice source (loop + `bread_shelf_{i}` + shared `_box`
  cut geometry + single uniform carriage joint). NOTE for the template author:
  fold the parent's `("right"/"left")` tuple and the long-single's bare shelf into
  one `for i in range(slot_count)` form so Multiplicity/Copy Logic reads cleanly.

Secondary (non-slot) copy loops present for the template author to reuse:
- feet: `for i, (fx, fy) in enumerate([...4 corners...])` → `foot_{i}` (FIXED inline visuals).
- dial ticks: `for i, ang_deg in enumerate((235,180,125,70))` → `dial_mark_{i+1}`;
  slider variant `for i in range(4)` → `slider_mark_{i+1}`.
- function buttons: `for name, z in zip(("cancel_button","frozen_button","bagel_button"), BTN_Z)`
  → 3 independent PRISMATIC `body_to_{name}` presses (axis (-1,0,0), travel 0.003).
- digital browning buttons: `for i in range(2)` → `browning_button_{i}` PRISMATIC pair.

## 组合数预审
Slot A (3: rotary_dial / slider / digital) × Slot B (2: front_lever / side_lever)
× Slot C (2: none / pullout) × Slot D (2: square_box / retro_round) = 24 base
topologies. × slot_count multiplicity N samples {1,2,4} (model N_range [2,4] plus
(Π(slot) = 24 ≥ 10 BEFORE N; A×B alone = 6, A×B×D = 12). Every slot has ≥2
candidates after folding (A=3, B=2, C=2 with none-as-disabled, D=2). pattern =
multiplicity (slot_count) + named slots. ✓

## 排除项(未来 compatibility matrix 素材)
- Each variant is single-axis off the parent baseline (only the targeted slot
  changes; everything else stays at parent values). No converged variant combines
  two non-baseline slots (e.g. side_lever × retro_round, or digital × pullout) —
  those cross-slot combos are sampler products, not separately built here.
- slot_count interactions: the 4-slice variant lengthens the body to 0.396 m and
  adds `side_rail_{side_tag}` carriage spine; the long-single narrows the cavity.
  Note for the compatibility matrix that retro_round's lofted dome was tuned for
  the 2-slice body (cavity shortened, recess/panel Z lowered for the taper) — a
  retro_round × 4-slice (long body) combo would need the loft re-proportioned, and
  the dome taper limits top-slot count along Y. side_lever assumes a flat side
  wall (square_box); side_lever × retro_round would need the slot/guide projected
  onto the curved barrel face.
- Slot C `none` is a single-candidate state (tray absent) — fold it into the
  pull-out module as its disabled/absent flag rather than a standalone slot value.
- Pure dimensional knobs (body L/W/H, lever travel, dial range, tray travel,
  slot pitch) are NOT slots — they are the template's continuous parameters
  (controlled local parameterization).

---
note: parent picture/Kitchen/Toaster/001.png covers the slot_count=2 × rotary_dial
× front_lever × no-tray × square_box baseline; the 7 variants are fork children of
the parent (workbench-only), each single-axis (one named slot OR the slot_count
multiplicity). All 8 sources compile-success, are workbench-only, and carry ≥1
non-fixed joint (every source keeps the carriage PRISMATIC plus its slot's
mechanism). This map targets a fresh modular Kitchen/Toaster template; sync to
arti-template is by record-dir + materialization copy (rating=5), staying
workbench-only on both sides.
