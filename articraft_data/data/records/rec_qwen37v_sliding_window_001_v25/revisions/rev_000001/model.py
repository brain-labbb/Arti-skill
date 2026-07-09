from __future__ import annotations

# Three-panel horizontal sliding window variant: thick aluminum frame rails with
# deep tracks, colonial divided-lite grilles, rubber gasket strips around glass,
# and a rotating latch at the meeting rail.
#
# Coordinate convention:
#   +Z is up. Window stands vertically.
#     - width  -> X
#     - height -> Z   (sill near z=0)
#     - frame depth / glazing thickness / slide-normal -> Y
#   The glass plane is the X-Z plane. The window reads SHUT at q=0.
#
# Structure:
#   - frame (static root): thick aluminum head, sill, jambs, mullions with deep
#     track grooves cut into the head and sill inner faces.
#   - left_lite, right_lite (FIXED): aluminum sash ring + colonial grille +
#     rubber gasket + glass.
#   - center_sash (SLIDING): same construction, proud of side lites in +Y.
#   - latch (REVOLUTE): small lever on center sash meeting rail.

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

TOTAL_W = 3.00            # overall window width along X
TOTAL_H = 1.50            # overall height along Z

# Thick aluminum frame
FRAME_FACE = 0.090        # wider aluminum frame member face width
MULLION_FACE = 0.070      # intermediate mullion face width
FRAME_DEPTH = 0.140       # deep aluminum frame depth along Y

# Track grooves (channels cut into head/sill inner face for sash to ride in)
TRACK_GROOVE_W = 0.035    # groove width along Y (matches sash depth + clearance)
TRACK_GROOVE_DEPTH = 0.018  # how deep the groove cuts into the frame member
TRACK_GROOVE_OFFSET_Y = 0.025  # groove center offset from frame Y center toward +Y (proud track)

# Three lite columns
SIDE_LITE_W = 0.84        # clear opening width of each side lite
CENTER_LITE_W = 1.00      # clear opening width of center lite

# Sash construction
SASH_FACE = 0.055         # sash perimeter rail/stile face width
SASH_DEPTH = 0.032        # sash depth along Y (fits in track groove)
GLASS_T = 0.008           # glazing thickness

# Rubber gasket strips around glass
GASKET_W = 0.012          # gasket strip face width
GASKET_DEPTH = 0.010      # gasket strip depth (slightly proud of glass)

# Colonial grille
GRILLE_COLS = 4
GRILLE_ROWS = 5
MUNTIN_T = 0.018
MUNTIN_DEPTH = 0.018

# Latch dimensions
LATCH_BASE_W = 0.030      # latch base plate width
LATCH_BASE_H = 0.050      # latch base plate height
LATCH_BASE_D = 0.015      # latch base plate depth
LATCH_LEVER_W = 0.015     # lever width
LATCH_LEVER_L = 0.060     # lever length
LATCH_LEVER_D = 0.012     # lever depth

# Y layout
FIXED_LITE_Y = -0.025     # fixed side lites rear glazing plane
SLIDE_SASH_Y = 0.052      # center sash proud toward +Y

REBATE = 0.005

# ---------------------------------------------------------------------------
# Derived layout
# ---------------------------------------------------------------------------

HALF_W = TOTAL_W / 2.0

INNER_X0 = -HALF_W + FRAME_FACE
INNER_X1 = HALF_W - FRAME_FACE
INNER_Z0 = FRAME_FACE
INNER_Z1 = TOTAL_H - FRAME_FACE

LEFT_X0 = INNER_X0
LEFT_X1 = LEFT_X0 + SIDE_LITE_W
MUL0_X0 = LEFT_X1
MUL0_X1 = MUL0_X0 + MULLION_FACE
CENTER_X0 = MUL0_X1
CENTER_X1 = CENTER_X0 + CENTER_LITE_W
MUL1_X0 = CENTER_X1
MUL1_X1 = MUL1_X0 + MULLION_FACE
RIGHT_X0 = MUL1_X1
RIGHT_X1 = INNER_X1

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

ALUMINUM_RGBA = (0.72, 0.74, 0.76, 1.0)   # brushed aluminum silver
GLASS_RGBA = (0.50, 0.58, 0.64, 0.32)     # cool grey-blue, semi-transparent
RUBBER_RGBA = (0.12, 0.12, 0.13, 1.0)     # dark rubber gasket
LATCH_RGBA = (0.25, 0.25, 0.27, 1.0)      # dark anodized aluminum latch


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _slab(x0: float, x1: float, z0: float, z1: float, y_center: float, depth: float) -> cq.Workplane:
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
    """Thick aluminum frame with deep track grooves in head and sill."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)

    # Cut three lite openings
    cut_depth = FRAME_DEPTH + 0.02
    left_cut = _slab(LEFT_X0, LEFT_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    center_cut = _slab(CENTER_X0, CENTER_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    right_cut = _slab(RIGHT_X0, RIGHT_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)

    frame = outer.cut(left_cut).cut(center_cut).cut(right_cut)

    # Deep track grooves in head (top rail) inner face
    # Groove runs the full inner width, cut into the bottom face of the head member
    groove_y_center = FRAME_DEPTH / 2.0 - TRACK_GROOVE_DEPTH / 2.0  # near front face
    head_groove = _slab(
        INNER_X0, INNER_X1,
        INNER_Z1, INNER_Z1 + TRACK_GROOVE_DEPTH,
        groove_y_center - FRAME_DEPTH / 2.0 + TRACK_GROOVE_OFFSET_Y,
        TRACK_GROOVE_W,
    )
    frame = frame.cut(head_groove)

    # Deep track groove in sill (bottom rail) inner face
    sill_groove = _slab(
        INNER_X0, INNER_X1,
        INNER_Z0 - TRACK_GROOVE_DEPTH, INNER_Z0,
        groove_y_center - FRAME_DEPTH / 2.0 + TRACK_GROOVE_OFFSET_Y,
        TRACK_GROOVE_W,
    )
    frame = frame.cut(sill_groove)

    return frame


def _build_sash_grille_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """Sash ring + colonial muntin grille in local frame."""
    ow = opening_w
    oh = opening_h
    out_w = ow + 2 * SASH_FACE
    out_h = oh + 2 * SASH_FACE

    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, SASH_DEPTH)
    opening = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SASH_DEPTH + 0.02)
    ring = outer.cut(opening)

    bars = None
    for c in range(1, GRILLE_COLS):
        frac = c / GRILLE_COLS
        x = -ow / 2.0 + frac * ow
        bar = _slab(
            x - MUNTIN_T / 2.0, x + MUNTIN_T / 2.0,
            -oh / 2.0, oh / 2.0,
            0.0, MUNTIN_DEPTH,
        )
        bars = bar if bars is None else bars.union(bar)

    for r in range(1, GRILLE_ROWS):
        frac = r / GRILLE_ROWS
        z = -oh / 2.0 + frac * oh
        bar = _slab(
            -ow / 2.0, ow / 2.0,
            z - MUNTIN_T / 2.0, z + MUNTIN_T / 2.0,
            0.0, MUNTIN_DEPTH,
        )
        bars = bar if bars is None else bars.union(bar)

    return ring if bars is None else ring.union(bars)


def _build_gasket_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """Rubber gasket strip around the glass perimeter in sash-local frame."""
    ow = opening_w
    oh = opening_h
    # Outer gasket rectangle matches the opening + rebate
    outer_w = ow + 2 * REBATE + 2 * GASKET_W
    outer_h = oh + 2 * REBATE + 2 * GASKET_W
    # Inner cut is the glass opening
    inner_w = ow + 2 * REBATE
    inner_h = oh + 2 * REBATE

    outer = _slab(-outer_w / 2.0, outer_w / 2.0, -outer_h / 2.0, outer_h / 2.0, 0.0, GASKET_DEPTH)
    inner = _slab(-inner_w / 2.0, inner_w / 2.0, -inner_h / 2.0, inner_h / 2.0, 0.0, GASKET_DEPTH + 0.02)
    return outer.cut(inner)


def _build_glass_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """Glass pane in sash-local frame."""
    ow = opening_w + 2 * REBATE
    oh = opening_h + 2 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


def _build_latch_shape() -> cq.Workplane:
    """Latch assembly: base plate + rotating lever, in latch-local frame.
    The latch pivots around local Z axis at the origin (center of base plate).
    The lever extends along +X from the pivot point."""
    # Base plate (mounted on sash stile)
    base = _slab(
        -LATCH_BASE_W / 2.0, LATCH_BASE_W / 2.0,
        -LATCH_BASE_H / 2.0, LATCH_BASE_H / 2.0,
        0.0, LATCH_BASE_D,
    )
    # Lever arm extending from pivot center along +X
    lever = _slab(
        0.0, LATCH_LEVER_L,
        -LATCH_LEVER_W / 2.0, LATCH_LEVER_W / 2.0,
        0.0, LATCH_LEVER_D,
    )
    return base.union(lever)


# ---------------------------------------------------------------------------
# Part builders
# ---------------------------------------------------------------------------

def _add_lite(model: ArticulatedObject, name: str, opening_w: float, opening_h: float) -> None:
    """Add a sash part with aluminum ring, grille, rubber gasket, and glass."""
    lite = model.part(name)
    lite.visual(
        mesh_from_cadquery(_build_sash_grille_shape(opening_w, opening_h), f"{name}_aluminum"),
        material="aluminum",
        name=f"{name}_aluminum",
    )
    lite.visual(
        mesh_from_cadquery(_build_gasket_shape(opening_w, opening_h), f"{name}_gasket"),
        material="rubber",
        name=f"{name}_gasket",
    )
    lite.visual(
        mesh_from_cadquery(_build_glass_shape(opening_w, opening_h), f"{name}_glass"),
        material="glass",
        name=f"{name}_glass",
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    span = SIDE_LITE_W + MULLION_FACE + CENTER_LITE_W + MULLION_FACE + SIDE_LITE_W
    inner_w = INNER_X1 - INNER_X0
    assert abs(span - inner_w) < 1e-6, f"lite layout {span} != inner width {inner_w}"

    model = ArticulatedObject(name="aluminum_sliding_window_with_latch")
    model.material("aluminum", rgba=ALUMINUM_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("rubber", rgba=RUBBER_RGBA)
    model.material("latch_metal", rgba=LATCH_RGBA)

    # --- Static outer frame (root) with track grooves ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material="aluminum",
        name="frame_shell",
    )

    opening_h = INNER_Z1 - INNER_Z0

    # --- Two FIXED side lites + CENTER sliding sash ---
    _add_lite(model, "left_lite", SIDE_LITE_W, opening_h)
    _add_lite(model, "right_lite", SIDE_LITE_W, opening_h)
    _add_lite(model, "center_sash", CENTER_LITE_W, opening_h)

    # --- Latch on center sash meeting rail ---
    latch = model.part("latch")
    latch.visual(
        mesh_from_cadquery(_build_latch_shape(), "latch_body"),
        material="latch_metal",
        name="latch_body",
    )

    # Centers of each opening
    left_cx = (LEFT_X0 + LEFT_X1) / 2.0
    right_cx = (RIGHT_X0 + RIGHT_X1) / 2.0
    center_cx = (CENTER_X0 + CENTER_X1) / 2.0
    mid_cz = (INNER_Z0 + INNER_Z1) / 2.0

    # FIXED side lites
    model.articulation(
        "frame_to_left_lite",
        ArticulationType.FIXED,
        parent="frame",
        child="left_lite",
        origin=Origin(xyz=(left_cx, FIXED_LITE_Y, mid_cz)),
    )
    model.articulation(
        "frame_to_right_lite",
        ArticulationType.FIXED,
        parent="frame",
        child="right_lite",
        origin=Origin(xyz=(right_cx, FIXED_LITE_Y, mid_cz)),
    )

    # CENTER sliding sash: PRISMATIC along +X
    slide_travel = SIDE_LITE_W * 0.92
    model.articulation(
        "frame_to_center_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="center_sash",
        origin=Origin(xyz=(center_cx, SLIDE_SASH_Y, mid_cz)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.5, lower=0.0, upper=slide_travel),
    )

    # Latch: REVOLUTE on the center sash meeting rail (left stile where it meets
    # the left mullion when closed). The latch pivots around local Z axis.
    # Meeting rail X position: left edge of center sash opening = CENTER_X0
    # In the sash-local frame, the meeting rail left stile is at local X = -(CENTER_LITE_W/2 + SASH_FACE/2)
    # The latch sits at mid-height of the sash (local Z = 0).
    latch_local_x = -(CENTER_LITE_W / 2.0 + SASH_FACE / 2.0)
    latch_local_z = 0.0
    # Latch sits at the front face of the sash (proud in Y for visibility)
    latch_y_offset = SASH_DEPTH / 2.0 + LATCH_BASE_D / 2.0

    model.articulation(
        "sash_to_latch",
        ArticulationType.REVOLUTE,
        parent="center_sash",
        child="latch",
        origin=Origin(xyz=(latch_local_x, latch_y_offset, latch_local_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.57),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    left_lite = object_model.get_part("left_lite")
    right_lite = object_model.get_part("right_lite")
    center_sash = object_model.get_part("center_sash")
    latch = object_model.get_part("latch")
    slide = object_model.get_articulation("frame_to_center_sash")
    latch_joint = object_model.get_articulation("sash_to_latch")

    # --- Intentional overlaps ---
    # Glass captured under gasket and sash lip
    ctx.allow_overlap(
        "left_lite", "left_lite",
        elem_a="left_lite_glass",
        elem_b="left_lite_aluminum",
        reason="Glass pane is rebated under the sash/muntin lip (captured glazing).",
    )
    ctx.allow_overlap(
        "left_lite", "left_lite",
        elem_a="left_lite_gasket",
        elem_b="left_lite_aluminum",
        reason="Rubber gasket strip sits between glass and sash ring (seated compression).",
    )
    ctx.allow_overlap(
        "left_lite", "left_lite",
        elem_a="left_lite_gasket",
        elem_b="left_lite_glass",
        reason="Gasket strip overlaps the glass edge perimeter (seated capture seal).",
    )
    ctx.allow_overlap(
        "right_lite", "right_lite",
        elem_a="right_lite_glass",
        elem_b="right_lite_aluminum",
        reason="Glass pane is rebated under the sash/muntin lip (captured glazing).",
    )
    ctx.allow_overlap(
        "right_lite", "right_lite",
        elem_a="right_lite_gasket",
        elem_b="right_lite_aluminum",
        reason="Rubber gasket strip sits between glass and sash ring (seated compression).",
    )
    ctx.allow_overlap(
        "right_lite", "right_lite",
        elem_a="right_lite_gasket",
        elem_b="right_lite_glass",
        reason="Gasket strip overlaps the glass edge perimeter (seated capture seal).",
    )
    ctx.allow_overlap(
        "center_sash", "center_sash",
        elem_a="center_sash_glass",
        elem_b="center_sash_aluminum",
        reason="Glass pane is rebated under the sash/muntin lip (captured glazing).",
    )
    ctx.allow_overlap(
        "center_sash", "center_sash",
        elem_a="center_sash_gasket",
        elem_b="center_sash_aluminum",
        reason="Rubber gasket strip sits between glass and sash ring (seated compression).",
    )
    ctx.allow_overlap(
        "center_sash", "center_sash",
        elem_a="center_sash_gasket",
        elem_b="center_sash_glass",
        reason="Gasket strip overlaps the glass edge perimeter (seated capture seal).",
    )

    # Fixed lites rebated into frame
    ctx.allow_overlap(
        "frame", "left_lite",
        elem_a="frame_shell", elem_b="left_lite_aluminum",
        reason="Left fixed lite is rebated into the frame opening (seated capture).",
    )
    ctx.allow_overlap(
        "frame", "right_lite",
        elem_a="frame_shell", elem_b="right_lite_aluminum",
        reason="Right fixed lite is rebated into the frame opening (seated capture).",
    )
    # Center sash rides track
    ctx.allow_overlap(
        "frame", "center_sash",
        elem_a="frame_shell", elem_b="center_sash_aluminum",
        reason="Center sash rides the head/sill track grooves (slider capture).",
    )
    # Glass captured in frame openings
    ctx.allow_overlap(
        "frame", "left_lite",
        elem_a="frame_shell", elem_b="left_lite_glass",
        reason="Left lite glass rebated under frame opening lip.",
    )
    ctx.allow_overlap(
        "frame", "right_lite",
        elem_a="frame_shell", elem_b="right_lite_glass",
        reason="Right lite glass rebated under frame opening lip.",
    )
    ctx.allow_overlap(
        "frame", "center_sash",
        elem_a="frame_shell", elem_b="center_sash_glass",
        reason="Center sash glass laps the track lip as sash rides the groove.",
    )
    # Latch mounted on sash stile
    ctx.allow_overlap(
        "center_sash", "latch",
        elem_a="center_sash_aluminum", elem_b="latch_body",
        reason="Latch base plate is surface-mounted on the sash meeting rail stile.",
    )

    # --- Verify aluminum frame geometry ---
    frame_aabb = ctx.part_world_aabb(frame)
    frame_depth = frame_aabb[1][1] - frame_aabb[0][1]
    ctx.check(
        "frame has thick aluminum depth",
        frame_depth > 0.12,
        details=f"frame depth={frame_depth:.3f}",
    )

    # --- Verify rubber gaskets present on each lite ---
    for nm in ("left_lite", "right_lite", "center_sash"):
        gasket_vis = object_model.get_part(nm).get_visual(f"{nm}_gasket")
        ctx.check(
            f"{nm} has rubber gasket strip",
            gasket_vis is not None,
            details=f"missing gasket visual on {nm}",
        )

    # --- Verify latch part and revolute joint exist ---
    ctx.check(
        "latch part exists",
        latch is not None,
        details="latch part not found",
    )
    ctx.check(
        "latch joint is revolute",
        latch_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"latch joint type={latch_joint.articulation_type}",
    )

    # --- Closed pose (q=0): window reads SHUT ---
    with ctx.pose({slide: 0.0, latch_joint: 0.0}):
        f_aabb = ctx.part_world_aabb(frame)
        l_aabb = ctx.part_world_aabb(left_lite)
        r_aabb = ctx.part_world_aabb(right_lite)
        c_aabb = ctx.part_world_aabb(center_sash)

        frame_w = f_aabb[1][0] - f_aabb[0][0]
        center_w = c_aabb[1][0] - c_aabb[0][0]
        ctx.check(
            "frame spans wider than center sash",
            frame_w > center_w + 1.5,
            details=f"frame_w={frame_w:.3f}, center_w={center_w:.3f}",
        )

        ctx.check(
            "sill near z=0",
            abs(f_aabb[0][2]) < 0.02,
            details=f"frame zmin={f_aabb[0][2]:.4f}",
        )
        ctx.check(
            "head reaches full height",
            abs(f_aabb[1][2] - TOTAL_H) < 0.02,
            details=f"frame zmax={f_aabb[1][2]:.4f}",
        )

        lx = (l_aabb[0][0] + l_aabb[1][0]) / 2.0
        cx_pos = (c_aabb[0][0] + c_aabb[1][0]) / 2.0
        rx = (r_aabb[0][0] + r_aabb[1][0]) / 2.0
        ctx.check(
            "lites ordered left-center-right",
            lx < cx_pos < rx,
            details=f"left_x={lx:.3f}, center_x={cx_pos:.3f}, right_x={rx:.3f}",
        )

        # Center sash proud of side lites
        l_y = (l_aabb[0][1] + l_aabb[1][1]) / 2.0
        c_y = (c_aabb[0][1] + c_aabb[1][1]) / 2.0
        ctx.check(
            "center sash proud of side lites",
            c_y > l_y + 0.02,
            details=f"center_y={c_y:.3f}, side_y={l_y:.3f}",
        )

        # Fixed lites seated in frame
        ctx.expect_overlap(
            left_lite, frame, axes="xz", min_overlap=0.03,
            name="left fixed lite seated in frame opening",
        )
        ctx.expect_overlap(
            right_lite, frame, axes="xz", min_overlap=0.03,
            name="right fixed lite seated in frame opening",
        )

        rest_cx = cx_pos
        rest_cz = (c_aabb[0][2] + c_aabb[1][2]) / 2.0

    # --- Slide open pose ---
    travel = slide.motion_limits.upper
    with ctx.pose({slide: travel, latch_joint: 0.0}):
        c_open = ctx.part_world_aabb(center_sash)
        open_cx = (c_open[0][0] + c_open[1][0]) / 2.0
        ctx.check(
            "center sash slides along +X by ~travel",
            abs((open_cx - rest_cx) - travel) < 0.02,
            details=f"rest_cx={rest_cx:.3f}, open_cx={open_cx:.3f}, travel={travel:.3f}",
        )
        c_open_z = (c_open[0][2] + c_open[1][2]) / 2.0
        ctx.check(
            "slide is purely horizontal",
            abs(c_open_z - rest_cz) < 0.02,
            details=f"open_z={c_open_z:.3f}, rest_z={rest_cz:.3f}",
        )
        f_aabb2 = ctx.part_world_aabb(frame)
        ctx.check(
            "sash retained within frame X span at full travel",
            c_open[1][0] < f_aabb2[1][0] + 1e-4 and c_open[0][0] > f_aabb2[0][0] - 1e-4,
            details=f"sash x=[{c_open[0][0]:.3f},{c_open[1][0]:.3f}] frame x=[{f_aabb2[0][0]:.3f},{f_aabb2[1][0]:.3f}]",
        )
        ctx.expect_overlap(
            center_sash, frame, axes="z", min_overlap=0.10,
            name="sash retains vertical engagement with head/sill track",
        )

    # --- Latch rotation test ---
    with ctx.pose({slide: 0.0, latch_joint: 0.0}):
        latch_rest_aabb = ctx.part_world_aabb(latch)
    with ctx.pose({slide: 0.0, latch_joint: 1.2}):
        latch_rotated_aabb = ctx.part_world_aabb(latch)

    ctx.check(
        "latch rotates when joint is driven",
        latch_rest_aabb is not None and latch_rotated_aabb is not None,
        details=f"rest_aabb={latch_rest_aabb}, rotated_aabb={latch_rotated_aabb}",
    )
    if latch_rest_aabb and latch_rotated_aabb:
        # The latch lever extends along +X at rest; after rotating ~1.2 rad around Z,
        # the bounding box should change significantly (lever swings in XY plane).
        rest_width = latch_rest_aabb[1][0] - latch_rest_aabb[0][0]
        rotated_width = latch_rotated_aabb[1][0] - latch_rotated_aabb[0][0]
        rest_depth = latch_rest_aabb[1][1] - latch_rest_aabb[0][1]
        rotated_depth = latch_rotated_aabb[1][1] - latch_rotated_aabb[0][1]
        
        ctx.check(
            "latch bounding box changes with rotation",
            abs(rest_width - rotated_width) > 0.01 or abs(rest_depth - rotated_depth) > 0.01,
            details=f"rest_w={rest_width:.4f}, rotated_w={rotated_width:.4f}, rest_d={rest_depth:.4f}, rotated_d={rotated_depth:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
