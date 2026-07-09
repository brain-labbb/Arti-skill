# Handtools / Tool cart — template source map

pattern: multiplicity (also has fixed named slots → effectively mixed; the dominant
axis is the per-N drawer stack, with a storage-module slot and a caster slot layered on)

parents:
- rec_build-a-realistic-articulated-3d-model-of-a-tool_20260609_163949_304573_1e586d5a ← picture/Handtools/Tool cart/001.png (red-and-black rolling roller cabinet: 5-drawer stack [3 shallow + 2 deep], side pegboard, molded top tray, rear push handle, 4 swivel casters)

Rolling roller cabinet / mobile tool chest. Shared spine for every candidate: a single
fused `cabinet_frame` carcass (root, `carcass_shell` + `side_pegboard` visuals) standing
on four casters, with a fixed rear `push_handle` (`handle_frame` + `handle_grip`, joint
`frame_to_handle` FIXED). The front storage face, the top deck, and the wheel type are the
independent structural slots below. The drawer stack is the multiplicity axis. Coordinate
convention is shared by all sources: +Z up, +Y toward the front (drawer face / handle grip),
+X right; carcass floor sits at `FLOOR_Z = CASTER_GAP` so wheels rest at z=0.

## Slot 候选覆盖

### Slot A:storage module (front face + top deck)
The drawers candidate is itself the multiplicity axis (see below); the other candidates
swap the front face and/or top deck while keeping a parent-baseline drawer stack.

| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| drawer_stack | parent (…_1e586d5a) | `drawer_{i}` part / `drawer_face_{i}` visual / `frame_to_drawer_{i}` (PRISMATIC +Y), loop `for i in enumerate(_drawer_face_centers())` | red drawer fronts with recessed finger pull, black hollow box body sliding straight out; THE multiplicity axis | converged |
| cabinet_door | rec_tool_cart_var_door | `cabinet_door` part / `door_panel` visual / `frame_to_door` (REVOLUTE +Z, 0→DOOR_OPEN_ANGLE) | swings a hinged cabinet door over the lower bay; 3 shallow drawers retained above it | converged |
| open_shelf | rec_tool_cart_var_openshelf | `bay_divider` inline visual + `shelf_{i}` inline visuals on `cabinet_frame` (FIXED, loop `for i in range(SHELF_COUNT)` via `_shelf_z_positions()`); helpers `_shelf_mesh` / `_bay_divider_mesh` | open lower bay with fixed shelf boards instead of a closed front; 3 drawers retained above | converged |
| worktop | rec_tool_cart_var_worktop | flat slab + raised lip fused into `_carcass_mesh` (replaces the recessed tray basin); no extra part | top deck becomes a flat solid worktop with a thin perimeter lip; full 5-drawer parent stack retained below | converged |
| rail_shelf | rec_tool_cart_var_railshelf | `guard_rail` inline visual on `cabinet_frame`; helper `_guard_rail_mesh` (4 corner posts via `for i in range(4)` + horizontal bars) | tubular perimeter guard rail around the top utility shelf; full 5-drawer parent stack retained | converged |

### Slot B:caster / wheel module (mounted under `cabinet_frame` floor)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| all_swivel | parent (…_1e586d5a) | `caster_fork_{i}` / `caster_wheel_{i}`; `frame_to_caster_{i}` (CONTINUOUS Z) + `caster_to_wheel_{i}` (CONTINUOUS X); helpers `_caster_fork_mesh` / `_caster_wheel_mesh` | four identical trailing-yoke swivel casters; 4×(swivel+roll) joints | converged |
| mixed_swivel_fixed | rec_tool_cart_var_mixedcaster | front 2: `swivel_fork_{i}`/`swivel_wheel_{i}` (`frame_to_swivel_{i}` CONTINUOUS Z + `swivel_to_wheel_{i}` CONTINUOUS X); rear 2: `fixed_bracket_{i}`/`fixed_wheel_{i}` (`frame_to_bracket_{i}` FIXED + `bracket_to_wheel_{i}` CONTINUOUS X); helper `_fixed_bracket_mesh` | two swivel + two rigid (non-steering) casters, the common shop layout; two `for i in range(2)` loops | converged |
| brake_caster | rec_tool_cart_var_brakecaster | `caster_fork_{i}`/`caster_wheel_{i}` (rim+tire visuals) + `brake_lever_{i}`; `frame_to_caster_{i}` (CONTINUOUS Z) + `caster_to_wheel_{i}` (CONTINUOUS X) + `fork_to_brake_{i}` (REVOLUTE X, 0→BRAKE_ENGAGE_ANGLE); helpers `_brake_lever_mesh` / `_caster_wheel_meshes` | swivel casters with a foot brake lever per wheel; adds a 3rd joint per caster | converged |

## Multiplicity / Copy Logic
- count_param: `drawer_count` (sources call it `NUM_DRAWERS` / `N_DRAWERS`; parent has a fixed
  `DRAWER_HEIGHTS` 5-tuple). This is the dominant multiplicity axis: each drawer is one
  PRISMATIC joint, so non-fixed joint counts run 11–17 across variants.
- N 样本已覆盖: {3, 5, 7} → rec_tool_cart_var_n3 (NUM_DRAWERS=3, uniform 0.150) / parent (5: three 0.072 shallow + two 0.150 deep) / rec_tool_cart_var_n7 (N_DRAWERS=7, uniform 0.098, CAB_H raised to 0.820 to fit the taller stack).
- 模板建议 N_range: [2, 9] (small rolling cart 2 drawers up to a tall roller cabinet ~9;
  band height auto-fits via `CAB_H` / `_drawer_band_top` / `_drawer_face_centers`). Sample
  coverage {3,5,7} only demonstrates the copy logic; the sampler fills the rest.
- copied object: one drawer = a `drawer_{i}` part (red `drawer_face_{i}` panel with finger
  pull + hollow black box body) built by the shared `_drawer_mesh(index)` helper.
- naming: `drawer_{i}` part, `drawer_face_{i}` visual, `frame_to_drawer_{i}` joint (0-based i, top→bottom).
- placement: stacked top→bottom; `_drawer_face_centers()` walks down from `_drawer_band_top()`
  subtracting each `DRAWER_HEIGHTS[i]` + `DRAWER_GAP`; the joint origin z = each drawer's
  resting center, x=y=0.
- joint policy: each drawer is an INDEPENDENT PRISMATIC joint, parent=`cabinet_frame`,
  axis=(0,1,0) (+Y pull-out), `MotionLimits(lower=0.0, upper=DRAWER_TRAVEL≈0.30, effort=60, velocity=0.30)`.
  Not chained, not on a shared hub — every drawer slides out of the carcass independently.
- emission confirmed: n3 = `for i in range(NUM_DRAWERS)`; n7 = `DRAWER_HEIGHTS = tuple([0.098]*N_DRAWERS)`
  then `for i, zc in enumerate(_drawer_face_centers())`; parent loops `enumerate(face_centers)`
  over a hand-written 5-tuple. The readability contract (loop + `drawer_{i}` naming + shared
  `_drawer_mesh` helper + uniform prismatic policy) holds in every drawer source.

Secondary (non-drawer) copy loops present for the template author to reuse:
- casters: `for i, (cx,cy) in enumerate(caster_positions)` (4 corners; mixedcaster splits into
  two `for i in range(2)` loops for swivel vs fixed).
- open_shelf boards: `for i in range(SHELF_COUNT)` inline FIXED visuals (a count axis on the
  open_shelf candidate, not jointed).
- pegboard holes / rail posts / wheel spokes: inline geometry loops inside mesh helpers.

## 组合数预审
Slot A (5: drawer_stack / cabinet_door / open_shelf / worktop / rail_shelf) × Slot B
(3: all_swivel / mixed_swivel_fixed / brake_caster) = 15 base topologies.
× N samples for the drawer_stack candidate (3, e.g. {3,5,7}) easily clears the
spans the full [2,9] range). Every slot has ≥2 candidates (A=5, B=3). pattern = multiplicity
with fixed named slots layered on.

## 排除项(未来 compatibility matrix 素材)
- worktop / rail_shelf / open_shelf candidates were forked off the 5-drawer parent baseline,
  while cabinet_door was forked at 3 drawers (door needs vertical room for the lower bay). The
  drawer_count × storage-module interaction (e.g. a worktop over only 2 drawers, or a door over
  7) is a sampler product, not separately sampled here — note for the compatibility matrix that
  cabinet_door / open_shelf assume a reduced drawer band, while worktop / rail_shelf are pure
  top-deck swaps compatible with the full N range.
- No converged variant combines a non-drawer Slot A with a non-swivel Slot B (single-axis
  control means combos like cabinet_door × brake_caster are left to the sampler).
- Pure dimensional knobs (carcass W/D/H, drawer travel, throat/lip sizes) are NOT slots —
  they are the template's continuous parameters (controlled local parameterization).

---
note: 母资产 picture/Handtools/Tool cart/001.png covers the drawer_stack × all_swivel
baseline. Variants are fork children of the parent (workbench-only); each is single-axis
(one storage module OR one caster type, except the n3/n7 multiplicity pair which only change
drawer_count). Downstream non-modular files rolling_toolbox_with_telescoping_handle.py and
platform_cart.py are unrelated and ignored — this map targets a fresh modular Tool_cart template.
