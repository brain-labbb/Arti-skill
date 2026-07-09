from __future__ import annotations

# Variant: Three-panel horizontal sliding window with TWO sliding sashes on
# separate tracks.  Center sash slides RIGHT (+X), right sash slides LEFT (-X)
# — opposite directions.  Left lite is FIXED with NO muntin grille.  Both
# sliding sashes carry colonial divided-lite grilles.  Deep track grooves are
# cut into the head and sill rails.  Rubber gasket strips wrap every glass pane.
#
# Coordinate convention:
#   +Z is up, window stands vertically.
#     width  -> X,  height -> Z (sill near z=0),  depth / slide-normal -> Y
#   Glass plane is X-Z.  Window reads SHUT at q=0 for both prismatic joints.
#   Positive q on either joint opens the window (sashes move toward each other
#   and stack in the centre on separate Y tracks, revealing the side openings).
#
# Structure:
#   frame (root)     – outer vinyl shell with track-groove channels
#   left_lite (FIXED)– sash ring + glass + gasket, NO grille
#   center_sash      – sash ring + grille + glass + gasket, PRISMATIC +X
#   right_sash       – sash ring + grille + glass + gasket, PRISMATIC -X

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Absolute dimensions (meters)
# ---------------------------------------------------------------------------

TOTAL_W = 3.00
TOTAL_H = 1.50

FRAME_FACE = 0.070        # outer frame member face width
MULLION_FACE = 0.060      # intermediate mullion face width
FRAME_DEPTH = 0.120       # deeper frame to accommodate two slider tracks

# Three lite columns: left fixed | center slider | right slider
LEFT_LITE_W = 0.85
CENTER_SASH_W = 1.04
RIGHT_SASH_W = 0.85

# Sash construction
SASH_FACE = 0.055         # sash perimeter rail/stile face width
SASH_DEPTH = 0.035        # sash depth along Y (thinner for sliders)
GLASS_T = 0.008           # glazing thickness

# Colonial grille (on sliders only)
GRILLE_COLS = 4
GRILLE_ROWS = 5
MUNTIN_T = 0.020
MUNTIN_DEPTH = 0.020

# Track layout (Y positions).  Fixed lite is in the rear; two sliders are on
# separate tracks toward the front, with enough Y gap that the sashes never
# touch when they pass each other.
FIXED_LITE_Y = -0.030
CENTER_TRACK_Y = 0.012
RIGHT_TRACK_Y = 0.048

# Track groove dimensions (channels cut into sill top and head bottom)
GROOVE_DEPTH = 0.015      # 15 mm deep channels
GROOVE_WIDTH = 0.022      # 22 mm wide channels

# Gasket dimensions
GASKET_FACE = 0.007       # 7 mm visible rubber strip width
GASKET_DEPTH = 0.010      # 10 mm depth (wraps glass edge)

REBATE = 0.005            # glass tucks under sash lip by this much

# ---------------------------------------------------------------------------
# Derived layout
# ---------------------------------------------------------------------------

HALF_W = TOTAL_W / 2.0

INNER_X0 = -HALF_W + FRAME_FACE
INNER_X1 = HALF_W - FRAME_FACE
INNER_Z0 = FRAME_FACE
INNER_Z1 = TOTAL_H - FRAME_FACE

LEFT_X0 = INNER_X0
LEFT_X1 = LEFT_X0 + LEFT_LITE_W
MUL0_X0 = LEFT_X1
MUL0_X1 = MUL0_X0 + MULLION_FACE
CENTER_X0 = MUL0_X1
CENTER_X1 = CENTER_X0 + CENTER_SASH_W
MUL1_X0 = CENTER_X1
MUL1_X1 = MUL1_X0 + MULLION_FACE
RIGHT_X0 = MUL1_X1
RIGHT_X1 = INNER_X1

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

VINYL_RGBA = (0.94, 0.95, 0.96, 1.0)
GLASS_RGBA = (0.50, 0.58, 0.64, 0.32)
GASKET_RGBA = (0.12, 0.12, 0.12, 1.0)   # dark rubber


# ---------------------------------------------------------------------------
# Geometry helpers (CadQuery, meters, world frame)
# ---------------------------------------------------------------------------

def _slab(x0: float, x1: float, z0: float, z1: float,
          y_center: float, depth: float) -> cq.Workplane:
    """Axis-aligned box in the X-Z plane centred on y_center."""
    w = x1 - x0
    h = z1 - z0
    cx = (x0 + x1) / 2.0
    cz = (z0 + z1) / 2.0
    return (
        cq.Workplane("XY")
        .transformed(offset=(cx, y_center, cz))
        .box(w, depth, h)
    )


def _build_frame_shape() -> cq.Workplane:
    """Outer frame slab with three lite openings AND deep track grooves cut
    into the sill top face and head bottom face for each slider track."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)
    cut_d = FRAME_DEPTH + 0.02

    left_cut = _slab(LEFT_X0, LEFT_X1, INNER_Z0, INNER_Z1, 0.0, cut_d)
    center_cut = _slab(CENTER_X0, CENTER_X1, INNER_Z0, INNER_Z1, 0.0, cut_d)
    right_cut = _slab(RIGHT_X0, RIGHT_X1, INNER_Z0, INNER_Z1, 0.0, cut_d)
    frame = outer.cut(left_cut).cut(center_cut).cut(right_cut)

    # Track grooves: for each slider track, cut a channel into the sill top
    # and head bottom.  The grooves span the full inner width of the frame.
    groove_x_span = INNER_X1 - INNER_X0 + 0.02  # slightly wider than openings
    groove_x0 = INNER_X0 - 0.01
    groove_x1 = INNER_X1 + 0.01
    for ty in (CENTER_TRACK_Y, RIGHT_TRACK_Y):
        # Sill groove: cut downward from sill top face (z = FRAME_FACE)
        sill_gr = _slab(
            groove_x0, groove_x1,
            FRAME_FACE - GROOVE_DEPTH, FRAME_FACE + 0.001,
            ty, GROOVE_WIDTH,
        )
        frame = frame.cut(sill_gr)
        # Head groove: cut upward from head bottom face (z = TOTAL_H - FRAME_FACE)
        head_gr = _slab(
            groove_x0, groove_x1,
            TOTAL_H - FRAME_FACE - 0.001, TOTAL_H - FRAME_FACE + GROOVE_DEPTH,
            ty, GROOVE_WIDTH,
        )
        frame = frame.cut(head_gr)

    return frame


def _build_sash_frame_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """Sash ring (perimeter frame only, no grille) in sash-local frame."""
    ow = opening_w
    oh = opening_h
    out_w = ow + 2 * SASH_FACE
    out_h = oh + 2 * SASH_FACE
    outer = _slab(-out_w / 2, out_w / 2, -out_h / 2, out_h / 2, 0.0, SASH_DEPTH)
    inner = _slab(-ow / 2, ow / 2, -oh / 2, oh / 2, 0.0, SASH_DEPTH + 0.02)
    return outer.cut(inner)


def _build_sash_with_grille_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """Sash ring with colonial muntin grille bars across the opening."""
    ow = opening_w
    oh = opening_h
    out_w = ow + 2 * SASH_FACE
    out_h = oh + 2 * SASH_FACE
    outer = _slab(-out_w / 2, out_w / 2, -out_h / 2, out_h / 2, 0.0, SASH_DEPTH)
    inner = _slab(-ow / 2, ow / 2, -oh / 2, oh / 2, 0.0, SASH_DEPTH + 0.02)
    ring = outer.cut(inner)

    bars = None
    # Vertical muntins
    for c in range(1, GRILLE_COLS):
        x = -ow / 2 + (c / GRILLE_COLS) * ow
        bar = _slab(x - MUNTIN_T / 2, x + MUNTIN_T / 2,
                     -oh / 2, oh / 2, 0.0, MUNTIN_DEPTH)
        bars = bar if bars is None else bars.union(bar)
    # Horizontal muntins
    for r in range(1, GRILLE_ROWS):
        z = -oh / 2 + (r / GRILLE_ROWS) * oh
        bar = _slab(-ow / 2, ow / 2,
                     z - MUNTIN_T / 2, z + MUNTIN_T / 2,
                     0.0, MUNTIN_DEPTH)
        bars = bar if bars is None else bars.union(bar)

    return ring if bars is None else ring.union(bars)


def _build_glass_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """Clear glass pane, rebated under the sash lip."""
    ow = opening_w + 2 * REBATE
    oh = opening_h + 2 * REBATE
    return _slab(-ow / 2, ow / 2, -oh / 2, oh / 2, 0.0, GLASS_T)


def _build_gasket_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """Thin rubber gasket frame wrapping the glass pane perimeter."""
    glass_hw = (opening_w + 2 * REBATE) / 2.0
    glass_hh = (opening_h + 2 * REBATE) / 2.0
    outer_hw = glass_hw + GASKET_FACE
    outer_hh = glass_hh + GASKET_FACE
    outer = _slab(-outer_hw, outer_hw, -outer_hh, outer_hh, 0.0, GASKET_DEPTH)
    inner = _slab(-glass_hw, glass_hw, -glass_hh, glass_hh, 0.0, GASKET_DEPTH + 0.002)
    return outer.cut(inner)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    span = LEFT_LITE_W + MULLION_FACE + CENTER_SASH_W + MULLION_FACE + RIGHT_SASH_W
    inner_w = INNER_X1 - INNER_X0
    assert abs(span - inner_w) < 1e-6, f"layout {span} != inner {inner_w}"

    model = ArticulatedObject(name="dual_slider_sliding_window")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("rubber", rgba=GASKET_RGBA)

    # --- Frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame_shell"),
        material="vinyl", name="frame_shell",
    )

    opening_h = INNER_Z1 - INNER_Z0

    # --- Left lite (FIXED, no grille) ---
    left = model.part("left_lite")
    left.visual(
        mesh_from_cadquery(_build_sash_frame_shape(LEFT_LITE_W, opening_h), "left_lite_frame"),
        material="vinyl", name="left_lite_frame",
    )
    left.visual(
        mesh_from_cadquery(_build_glass_shape(LEFT_LITE_W, opening_h), "left_lite_glass"),
        material="glass", name="left_lite_glass",
    )
    left.visual(
        mesh_from_cadquery(_build_gasket_shape(LEFT_LITE_W, opening_h), "left_lite_gasket"),
        material="rubber", name="left_lite_gasket",
    )

    # --- Center sash (SLIDING +X, with grille) ---
    ctr = model.part("center_sash")
    ctr.visual(
        mesh_from_cadquery(_build_sash_with_grille_shape(CENTER_SASH_W, opening_h), "center_sash_vinyl"),
        material="vinyl", name="center_sash_vinyl",
    )
    ctr.visual(
        mesh_from_cadquery(_build_glass_shape(CENTER_SASH_W, opening_h), "center_sash_glass"),
        material="glass", name="center_sash_glass",
    )
    ctr.visual(
        mesh_from_cadquery(_build_gasket_shape(CENTER_SASH_W, opening_h), "center_sash_gasket"),
        material="rubber", name="center_sash_gasket",
    )

    # --- Right sash (SLIDING -X, with grille) ---
    rgt = model.part("right_sash")
    rgt.visual(
        mesh_from_cadquery(_build_sash_with_grille_shape(RIGHT_SASH_W, opening_h), "right_sash_vinyl"),
        material="vinyl", name="right_sash_vinyl",
    )
    rgt.visual(
        mesh_from_cadquery(_build_glass_shape(RIGHT_SASH_W, opening_h), "right_sash_glass"),
        material="glass", name="right_sash_glass",
    )
    rgt.visual(
        mesh_from_cadquery(_build_gasket_shape(RIGHT_SASH_W, opening_h), "right_sash_gasket"),
        material="rubber", name="right_sash_gasket",
    )

    # World-space centres for each lite opening
    left_cx = (LEFT_X0 + LEFT_X1) / 2.0
    center_cx = (CENTER_X0 + CENTER_X1) / 2.0
    right_cx = (RIGHT_X0 + RIGHT_X1) / 2.0
    mid_cz = (INNER_Z0 + INNER_Z1) / 2.0

    # FIXED left lite
    model.articulation(
        "frame_to_left_lite",
        ArticulationType.FIXED,
        parent="frame", child="left_lite",
        origin=Origin(xyz=(left_cx, FIXED_LITE_Y, mid_cz)),
    )

    slide_travel = LEFT_LITE_W * 0.90  # ~one side-panel width

    # Center sash: PRISMATIC along +X (positive q slides it RIGHT)
    model.articulation(
        "frame_to_center_sash",
        ArticulationType.PRISMATIC,
        parent="frame", child="center_sash",
        origin=Origin(xyz=(center_cx, CENTER_TRACK_Y, mid_cz)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.5,
                                   lower=0.0, upper=slide_travel),
    )

    # Right sash: PRISMATIC along -X (positive q slides it LEFT)
    model.articulation(
        "frame_to_right_sash",
        ArticulationType.PRISMATIC,
        parent="frame", child="right_sash",
        origin=Origin(xyz=(right_cx, RIGHT_TRACK_Y, mid_cz)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.5,
                                   lower=0.0, upper=slide_travel),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    left = object_model.get_part("left_lite")
    center = object_model.get_part("center_sash")
    right = object_model.get_part("right_sash")

    center_slide = object_model.get_articulation("frame_to_center_sash")
    right_slide = object_model.get_articulation("frame_to_right_sash")

    # ---- Intentional-overlap allowances ----

    # Helper: register per-sash glass/gasket/frame overlaps
    def _allow_sash_overlaps(nm: str, frame_vis: str):
        # Glass captured under sash lip
        ctx.allow_overlap(
            nm, nm, elem_a=f"{nm}_glass", elem_b=frame_vis,
            reason="Glass pane rebated under sash lip (captured glazing).",
        )
        # Gasket wraps glass edge
        ctx.allow_overlap(
            nm, nm, elem_a=f"{nm}_gasket", elem_b=f"{nm}_glass",
            reason="Rubber gasket wraps around glass pane perimeter.",
        )
        # Gasket contacts sash frame
        ctx.allow_overlap(
            nm, nm, elem_a=f"{nm}_gasket", elem_b=frame_vis,
            reason="Gasket compressed between glass edge and sash frame.",
        )

    _allow_sash_overlaps("left_lite", "left_lite_frame")
    _allow_sash_overlaps("center_sash", "center_sash_vinyl")
    _allow_sash_overlaps("right_sash", "right_sash_vinyl")

    # Frame-to-sash overlaps
    ctx.allow_overlap(
        "frame", "left_lite", elem_a="frame_shell", elem_b="left_lite_frame",
        reason="Left fixed lite rebated into frame opening (seated capture).",
    )
    ctx.allow_overlap(
        "frame", "left_lite", elem_a="frame_shell", elem_b="left_lite_glass",
        reason="Left lite glass captured by frame rebate lip.",
    )
    ctx.allow_overlap(
        "frame", "left_lite", elem_a="frame_shell", elem_b="left_lite_gasket",
        reason="Left lite gasket seated against frame rebate.",
    )

    ctx.allow_overlap(
        "frame", "center_sash", elem_a="frame_shell", elem_b="center_sash_vinyl",
        reason="Center sash rides head/sill track; laps frame face along track.",
    )
    ctx.allow_overlap(
        "frame", "center_sash", elem_a="frame_shell", elem_b="center_sash_glass",
        reason="Center sash glass laps track lip.",
    )
    ctx.allow_overlap(
        "frame", "center_sash", elem_a="frame_shell", elem_b="center_sash_gasket",
        reason="Center sash gasket laps track lip.",
    )

    ctx.allow_overlap(
        "frame", "right_sash", elem_a="frame_shell", elem_b="right_sash_vinyl",
        reason="Right sash rides head/sill track; laps frame face along track.",
    )
    ctx.allow_overlap(
        "frame", "right_sash", elem_a="frame_shell", elem_b="right_sash_glass",
        reason="Right sash glass laps track lip.",
    )
    ctx.allow_overlap(
        "frame", "right_sash", elem_a="frame_shell", elem_b="right_sash_gasket",
        reason="Right sash gasket laps track lip.",
    )

    # ---- Prompt-specific structural checks ----

    # Two prismatic joints
    ctx.check(
        "center_sash has prismatic joint",
        center_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={center_slide.articulation_type}",
    )
    ctx.check(
        "right_sash has prismatic joint",
        right_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={right_slide.articulation_type}",
    )

    # Opposite slide directions (one +X, one -X)
    ctx.check(
        "sliders move in opposite X directions",
        center_slide.axis[0] * right_slide.axis[0] < 0,
        details=f"center axis={center_slide.axis}, right axis={right_slide.axis}",
    )

    # Rubber gasket visuals present on all sash parts
    for nm, part_obj in (("left_lite", left), ("center_sash", center), ("right_sash", right)):
        gasket_name = f"{nm}_gasket"
        ctx.check(
            f"{nm} has rubber gasket strip",
            any(v.name == gasket_name for v in part_obj.visuals),
            details=f"visuals={[v.name for v in part_obj.visuals]}",
        )

    # Sliding sashes have colonial grille (vinyl visual with muntin bars)
    ctx.check(
        "center_sash has colonial grille vinyl",
        any(v.name == "center_sash_vinyl" for v in center.visuals),
        details="center sash should carry vinyl+grille visual",
    )
    ctx.check(
        "right_sash has colonial grille vinyl",
        any(v.name == "right_sash_vinyl" for v in right.visuals),
        details="right sash should carry vinyl+grille visual",
    )

    # Fixed left lite does NOT carry a grille (its vinyl is a plain frame ring)
    ctx.check(
        "left_lite has no grille (plain sash frame only)",
        any(v.name == "left_lite_frame" for v in left.visuals)
        and not any(v.name == "left_lite_grille" for v in left.visuals),
        details="left fixed lite should have plain frame, no grille bars",
    )

    # ---- Closed pose (both at q=0) ----
    with ctx.pose({center_slide: 0.0, right_slide: 0.0}):
        f_aabb = ctx.part_world_aabb(frame)
        l_aabb = ctx.part_world_aabb(left)
        c_aabb = ctx.part_world_aabb(center)
        r_aabb = ctx.part_world_aabb(right)

        # Frame spans the full window width
        frame_w = f_aabb[1][0] - f_aabb[0][0]
        ctx.check(
            "frame spans full window width",
            abs(frame_w - TOTAL_W) < 0.02,
            details=f"frame_w={frame_w:.3f}",
        )

        # Sill near z=0, head near z=TOTAL_H
        ctx.check(
            "sill near z=0",
            abs(f_aabb[0][2]) < 0.02,
            details=f"zmin={f_aabb[0][2]:.4f}",
        )
        ctx.check(
            "head near full height",
            abs(f_aabb[1][2] - TOTAL_H) < 0.02,
            details=f"zmax={f_aabb[1][2]:.4f}",
        )

        # Lites ordered left -> center -> right
        lx = (l_aabb[0][0] + l_aabb[1][0]) / 2.0
        cx = (c_aabb[0][0] + c_aabb[1][0]) / 2.0
        rx = (r_aabb[0][0] + r_aabb[1][0]) / 2.0
        ctx.check(
            "lites ordered left-center-right",
            lx < cx < rx,
            details=f"lx={lx:.3f} cx={cx:.3f} rx={rx:.3f}",
        )

        # All lites seated within frame height
        for nm, ab in (("left", l_aabb), ("center", c_aabb), ("right", r_aabb)):
            ctx.check(
                f"{nm} lite within frame height",
                ab[0][2] > f_aabb[0][2] - 1e-4 and ab[1][2] < f_aabb[1][2] + 1e-4,
                details=f"{nm} z=[{ab[0][2]:.3f},{ab[1][2]:.3f}]",
            )

        # Sliders are on separate Y tracks (proud of fixed lite)
        l_y = (l_aabb[0][1] + l_aabb[1][1]) / 2.0
        c_y = (c_aabb[0][1] + c_aabb[1][1]) / 2.0
        r_y = (r_aabb[0][1] + r_aabb[1][1]) / 2.0
        ctx.check(
            "center sash proud of fixed lite",
            c_y > l_y + 0.02,
            details=f"center_y={c_y:.3f} fixed_y={l_y:.3f}",
        )
        ctx.check(
            "right sash proud of fixed lite",
            r_y > l_y + 0.02,
            details=f"right_y={r_y:.3f} fixed_y={l_y:.3f}",
        )
        ctx.check(
            "sliders on separate Y tracks",
            abs(r_y - c_y) > 0.02,
            details=f"center_y={c_y:.3f} right_y={r_y:.3f}",
        )

        # Fixed lite seated in frame opening (projected overlap)
        ctx.expect_overlap(
            left, frame, axes="xz", min_overlap=0.03,
            name="left fixed lite seated in frame opening",
        )

        rest_cx = cx
        rest_rx = rx
        rest_cz = (c_aabb[0][2] + c_aabb[1][2]) / 2.0
        rest_rz = (r_aabb[0][2] + r_aabb[1][2]) / 2.0

    # ---- Open pose: both sliders driven to max travel ----
    travel_c = center_slide.motion_limits.upper
    travel_r = right_slide.motion_limits.upper

    with ctx.pose({center_slide: travel_c, right_slide: travel_r}):
        c_open = ctx.part_world_aabb(center)
        r_open = ctx.part_world_aabb(right)
        f_open = ctx.part_world_aabb(frame)

        open_cx = (c_open[0][0] + c_open[1][0]) / 2.0
        open_rx = (r_open[0][0] + r_open[1][0]) / 2.0

        # Center sash moved RIGHT (+X) by ~travel
        ctx.check(
            "center sash slides RIGHT (+X)",
            (open_cx - rest_cx) > travel_c * 0.85,
            details=f"rest_cx={rest_cx:.3f} open_cx={open_cx:.3f} travel={travel_c:.3f}",
        )

        # Right sash moved LEFT (-X) by ~travel
        ctx.check(
            "right sash slides LEFT (-X)",
            (rest_rx - open_rx) > travel_r * 0.85,
            details=f"rest_rx={rest_rx:.3f} open_rx={open_rx:.3f} travel={travel_r:.3f}",
        )

        # Pure horizontal slide (no Z drift)
        c_open_z = (c_open[0][2] + c_open[1][2]) / 2.0
        r_open_z = (r_open[0][2] + r_open[1][2]) / 2.0
        ctx.check(
            "center slide is horizontal (no Z drift)",
            abs(c_open_z - rest_cz) < 0.02,
            details=f"rest_z={rest_cz:.3f} open_z={c_open_z:.3f}",
        )
        ctx.check(
            "right slide is horizontal (no Z drift)",
            abs(r_open_z - rest_rz) < 0.02,
            details=f"rest_z={rest_rz:.3f} open_z={r_open_z:.3f}",
        )

        # Retained insertion: both sashes remain within frame X span
        ctx.check(
            "center sash retained in frame X at max travel",
            c_open[1][0] < f_open[1][0] + 1e-4 and c_open[0][0] > f_open[0][0] - 1e-4,
            details=f"sash x=[{c_open[0][0]:.3f},{c_open[1][0]:.3f}] frame x=[{f_open[0][0]:.3f},{f_open[1][0]:.3f}]",
        )
        ctx.check(
            "right sash retained in frame X at max travel",
            r_open[1][0] < f_open[1][0] + 1e-4 and r_open[0][0] > f_open[0][0] - 1e-4,
            details=f"sash x=[{r_open[0][0]:.3f},{r_open[1][0]:.3f}] frame x=[{f_open[0][0]:.3f},{f_open[1][0]:.3f}]",
        )

        # Vertical track engagement retained
        ctx.expect_overlap(
            center, frame, axes="z", min_overlap=0.10,
            name="center sash retains head/sill track engagement",
        )
        ctx.expect_overlap(
            right, frame, axes="z", min_overlap=0.10,
            name="right sash retains head/sill track engagement",
        )

    return ctx.report()


object_model = build_object_model()
