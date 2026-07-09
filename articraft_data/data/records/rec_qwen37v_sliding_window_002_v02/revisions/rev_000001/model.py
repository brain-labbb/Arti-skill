from __future__ import annotations

# Three-panel sliding window variant: white vinyl frame with two fixed panes
# (left narrow, center wide) and a vertically sliding lower sash on the right
# column. Two roller blocks at the sash bottom; visible overlap stile where
# the sliding sash top rail extends past the meeting rail.
#
# Coordinate convention:
#   +Z is up. Window stands vertically.
#     width  -> X,  height -> Z (sill near z=0),  depth -> Y
#   q=0 reads SHUT. Driving the prismatic joint slides the lower sash UPWARD
#   (+Z) to open, staying retained in the frame tracks.

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Absolute dimensions (meters)
# ---------------------------------------------------------------------------

TOTAL_W = 1.52
TOTAL_H = 1.72

FRAME_FACE = 0.085        # outer frame member face width (chunky vinyl)
FRAME_DEPTH = 0.140       # deep box section along Y

MULLION_W = 0.050         # vertical mullion bar width
HORIZ_RAIL_H = 0.060      # horizontal meeting rail height in right column

SASH_FACE = 0.055         # sash perimeter rail/stile face width
SASH_DEPTH = 0.060        # sash depth along Y
GLASS_T = 0.008           # glazing thickness
REBATE = 0.005            # glass tucks under sash lip

OVERLAP_STILE_H = 0.040   # sliding sash top rail extends past meeting rail

# Roller blocks at sash bottom
ROLLER_W = 0.025
ROLLER_H = 0.015
ROLLER_D = 0.030

# Latch hardware
LATCH_PLATE_W = 0.028
LATCH_PLATE_H = 0.060
LATCH_PLATE_T = 0.010
LATCH_LEVER_LEN = 0.040
LATCH_LEVER_R = 0.005

# ---------------------------------------------------------------------------
# Derived layout
# ---------------------------------------------------------------------------

HALF_W = TOTAL_W / 2.0
INNER_X0 = -HALF_W + FRAME_FACE
INNER_X1 = HALF_W - FRAME_FACE
INNER_Z0 = FRAME_FACE
INNER_Z1 = TOTAL_H - FRAME_FACE
INNER_W = INNER_X1 - INNER_X0
INNER_H = INNER_Z1 - INNER_Z0
MID_CZ = (INNER_Z0 + INNER_Z1) / 2.0

# Three columns: left (narrow), center (wider), right (narrow)
AVAIL_W = INNER_W - 2 * MULLION_W
LEFT_COL_W = AVAIL_W * 0.30
RIGHT_COL_W = AVAIL_W * 0.30
CENTER_COL_W = AVAIL_W - LEFT_COL_W - RIGHT_COL_W

# Column X boundaries
LEFT_X0 = INNER_X0
LEFT_X1 = LEFT_X0 + LEFT_COL_W
MULLION1_X0 = LEFT_X1
MULLION1_X1 = MULLION1_X0 + MULLION_W
CENTER_X0 = MULLION1_X1
CENTER_X1 = CENTER_X0 + CENTER_COL_W
MULLION2_X0 = CENTER_X1
MULLION2_X1 = MULLION2_X0 + MULLION_W
RIGHT_X0 = MULLION2_X1
RIGHT_X1 = INNER_X1

# Column centers
LEFT_CX = (LEFT_X0 + LEFT_X1) / 2.0
CENTER_CX = (CENTER_X0 + CENTER_X1) / 2.0
RIGHT_CX = (RIGHT_X0 + RIGHT_X1) / 2.0

# Right column vertical split: upper transom + meeting rail + lower sliding sash
LOWER_SASH_H = INNER_H * 0.58          # lower portion height
MEETING_RAIL_Z = INNER_Z0 + LOWER_SASH_H  # center of horizontal rail
UPPER_TRANSOM_H = INNER_H - LOWER_SASH_H - HORIZ_RAIL_H

# Opening heights (clear areas in the frame)
RIGHT_LOWER_OPEN_TOP = MEETING_RAIL_Z - HORIZ_RAIL_H / 2.0
RIGHT_LOWER_OPENING_H = RIGHT_LOWER_OPEN_TOP - INNER_Z0
RIGHT_UPPER_OPEN_BOT = MEETING_RAIL_Z + HORIZ_RAIL_H / 2.0
RIGHT_UPPER_OPENING_H = INNER_Z1 - RIGHT_UPPER_OPEN_BOT

# Sliding sash outer dimensions (includes overlap stile at top)
SLIDER_OUTER_H = RIGHT_LOWER_OPENING_H + OVERLAP_STILE_H
SLIDER_OUTER_W = RIGHT_COL_W

# Glass dims within sliding sash
SLIDER_GLASS_W = SLIDER_OUTER_W - 2 * SASH_FACE + 2 * REBATE
SLIDER_GLASS_H = SLIDER_OUTER_H - 2 * SASH_FACE - OVERLAP_STILE_H + 2 * REBATE
# Glass center is shifted down from sash center by OVERLAP_STILE_H/2
SLIDER_GLASS_CZ = -OVERLAP_STILE_H / 2.0

# Articulation origin for sliding sash: center of sash outer in closed position
SLIDER_ORIGIN_Z = INNER_Z0 + SLIDER_OUTER_H / 2.0

# Upper transom center Z
UPPER_TRANSOM_CZ = (RIGHT_UPPER_OPEN_BOT + INNER_Z1) / 2.0

# Travel limit: sash slides up but stays retained in frame
SLIDE_TRAVEL = min(RIGHT_LOWER_OPENING_H * 0.55, INNER_Z1 - SLIDER_ORIGIN_Z - SLIDER_OUTER_H / 2.0 + FRAME_FACE * 0.5)

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

VINYL_RGBA = (0.94, 0.95, 0.96, 1.0)
GLASS_RGBA = (0.52, 0.60, 0.66, 0.30)
METAL_RGBA = (0.74, 0.76, 0.79, 1.0)

# ---------------------------------------------------------------------------
# Geometry helpers (CadQuery)
# ---------------------------------------------------------------------------

def _slab(x0: float, x1: float, z0: float, z1: float, y_center: float, depth: float) -> cq.Workplane:
    """Axis-aligned box in X-Z plane, centered on y_center with given Y depth."""
    return (
        cq.Workplane("XY")
        .transformed(offset=((x0 + x1) / 2.0, y_center, (z0 + z1) / 2.0))
        .box(x1 - x0, depth, z1 - z0)
    )


def _build_frame_shape() -> cq.Workplane:
    """Outer frame: thick slab with three column openings, two mullions, and
    one horizontal meeting rail in the right column."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)
    cut_d = FRAME_DEPTH + 0.02

    # Left column opening (full inner height)
    left_cut = _slab(LEFT_X0, LEFT_X1, INNER_Z0, INNER_Z1, 0.0, cut_d)
    # Center column opening (full inner height, wider)
    center_cut = _slab(CENTER_X0, CENTER_X1, INNER_Z0, INNER_Z1, 0.0, cut_d)
    # Right column: upper transom opening
    right_upper_cut = _slab(RIGHT_X0, RIGHT_X1, RIGHT_UPPER_OPEN_BOT, INNER_Z1, 0.0, cut_d)
    # Right column: lower sliding sash opening
    right_lower_cut = _slab(RIGHT_X0, RIGHT_X1, INNER_Z0, RIGHT_LOWER_OPEN_TOP, 0.0, cut_d)

    return outer.cut(left_cut).cut(center_cut).cut(right_upper_cut).cut(right_lower_cut)


def _build_sash_ring(opening_w: float, opening_h: float) -> cq.Workplane:
    """Symmetric sash ring in local frame (centered at origin)."""
    out_w = opening_w
    out_h = opening_h
    outer = _slab(-out_w / 2, out_w / 2, -out_h / 2, out_h / 2, 0.0, SASH_DEPTH)
    cut_w = opening_w - 2 * SASH_FACE
    cut_h = opening_h - 2 * SASH_FACE
    inner = _slab(-cut_w / 2, cut_w / 2, -cut_h / 2, cut_h / 2, 0.0, SASH_DEPTH + 0.02)
    return outer.cut(inner)


def _build_glass_pane(w: float, h: float) -> cq.Workplane:
    """Glass pane centered at local origin."""
    return _slab(-w / 2, w / 2, -h / 2, h / 2, 0.0, GLASS_T)


def _build_sliding_sash_shape() -> cq.Workplane:
    """Sliding lower sash with overlap stile at top (asymmetric top rail)."""
    ow = SLIDER_OUTER_W
    oh = SLIDER_OUTER_H
    # Outer box centered at origin
    outer = _slab(-ow / 2, ow / 2, -oh / 2, oh / 2, 0.0, SASH_DEPTH)
    # Inner cut: offset downward by OVERLAP_STILE_H/2 (top rail is taller)
    cut_w = ow - 2 * SASH_FACE
    cut_h = oh - 2 * SASH_FACE - OVERLAP_STILE_H
    cz = SLIDER_GLASS_CZ  # -OVERLAP_STILE_H / 2
    inner = _slab(-cut_w / 2, cut_w / 2, cz - cut_h / 2, cz + cut_h / 2, 0.0, SASH_DEPTH + 0.02)
    return outer.cut(inner)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_panel_sliding_window")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("metal", rgba=METAL_RGBA)

    # --- Frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame_shell"),
        material="vinyl",
        name="frame_shell",
    )

    # --- Left fixed sash (narrow) ---
    left = model.part("left_fixed")
    left.visual(
        mesh_from_cadquery(_build_sash_ring(LEFT_COL_W, INNER_H), "left_sash_vinyl"),
        material="vinyl",
        name="left_sash_vinyl",
    )
    left.visual(
        mesh_from_cadquery(
            _build_glass_pane(LEFT_COL_W - 2 * SASH_FACE + 2 * REBATE,
                              INNER_H - 2 * SASH_FACE + 2 * REBATE),
            "left_sash_glass"),
        material="glass",
        name="left_sash_glass",
    )

    # --- Center fixed sash (wider) ---
    center = model.part("center_fixed")
    center.visual(
        mesh_from_cadquery(_build_sash_ring(CENTER_COL_W, INNER_H), "center_sash_vinyl"),
        material="vinyl",
        name="center_sash_vinyl",
    )
    center.visual(
        mesh_from_cadquery(
            _build_glass_pane(CENTER_COL_W - 2 * SASH_FACE + 2 * REBATE,
                              INNER_H - 2 * SASH_FACE + 2 * REBATE),
            "center_sash_glass"),
        material="glass",
        name="center_sash_glass",
    )

    # --- Upper transom (fixed, right column upper) ---
    upper = model.part("upper_transom")
    upper.visual(
        mesh_from_cadquery(
            _build_sash_ring(RIGHT_COL_W, RIGHT_UPPER_OPENING_H), "upper_sash_vinyl"),
        material="vinyl",
        name="upper_sash_vinyl",
    )
    upper.visual(
        mesh_from_cadquery(
            _build_glass_pane(RIGHT_COL_W - 2 * SASH_FACE + 2 * REBATE,
                              RIGHT_UPPER_OPENING_H - 2 * SASH_FACE + 2 * REBATE),
            "upper_sash_glass"),
        material="glass",
        name="upper_sash_glass",
    )

    # --- Sliding lower sash (right column lower, PRISMATIC +Z) ---
    slider = model.part("sliding_sash")
    slider.visual(
        mesh_from_cadquery(_build_sliding_sash_shape(), "sliding_sash_vinyl"),
        material="vinyl",
        name="sliding_sash_vinyl",
    )
    slider.visual(
        mesh_from_cadquery(
            _build_glass_pane(SLIDER_GLASS_W, SLIDER_GLASS_H),
            "sliding_sash_glass"),
        origin=Origin(xyz=(0.0, 0.0, SLIDER_GLASS_CZ)),
        material="glass",
        name="sliding_sash_glass",
    )

    # Roller blocks: two small metal blocks at the sash bottom rail
    roller_z = -SLIDER_OUTER_H / 2.0 + ROLLER_H * 0.4  # half-embedded in bottom rail
    roller_x_off = SLIDER_OUTER_W / 2.0 - ROLLER_W / 2.0 - 0.025
    for i, rx in enumerate([-roller_x_off, roller_x_off]):
        slider.visual(
            Box((ROLLER_W, ROLLER_D, ROLLER_H)),
            origin=Origin(xyz=(rx, 0.0, roller_z)),
            material="metal",
            name=f"roller_{i}",
        )

    # Latch handle on left stile of sliding sash (toward center of window)
    stile_x = -(SLIDER_OUTER_W / 2.0 - SASH_FACE / 2.0)
    face_y = SASH_DEPTH / 2.0
    plate_y = face_y + LATCH_PLATE_T / 2.0
    slider.visual(
        Box((LATCH_PLATE_W, LATCH_PLATE_T, LATCH_PLATE_H)),
        origin=Origin(xyz=(stile_x, plate_y, 0.05)),
        material="metal",
        name="latch_plate",
    )
    lever_y = face_y + LATCH_PLATE_T + LATCH_LEVER_LEN / 2.0
    slider.visual(
        Cylinder(radius=LATCH_LEVER_R, length=LATCH_LEVER_LEN),
        origin=Origin(xyz=(stile_x, lever_y, 0.042), rpy=(1.5707963, 0.0, 0.0)),
        material="metal",
        name="latch_lever",
    )

    # --- Articulations ---

    # Left fixed sash
    model.articulation(
        "frame_to_left_fixed",
        ArticulationType.FIXED,
        parent="frame",
        child="left_fixed",
        origin=Origin(xyz=(LEFT_CX, 0.0, MID_CZ)),
    )

    # Center fixed sash
    model.articulation(
        "frame_to_center_fixed",
        ArticulationType.FIXED,
        parent="frame",
        child="center_fixed",
        origin=Origin(xyz=(CENTER_CX, 0.0, MID_CZ)),
    )

    # Upper transom (fixed)
    model.articulation(
        "frame_to_upper_transom",
        ArticulationType.FIXED,
        parent="frame",
        child="upper_transom",
        origin=Origin(xyz=(RIGHT_CX, 0.0, UPPER_TRANSOM_CZ)),
    )

    # Sliding sash: PRISMATIC along +Z (slides upward to open)
    model.articulation(
        "frame_to_sliding_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="sliding_sash",
        origin=Origin(xyz=(RIGHT_CX, 0.0, SLIDER_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.3, lower=0.0, upper=SLIDE_TRAVEL),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    left_fixed = object_model.get_part("left_fixed")
    center_fixed = object_model.get_part("center_fixed")
    upper_transom = object_model.get_part("upper_transom")
    sliding_sash = object_model.get_part("sliding_sash")
    slide = object_model.get_articulation("frame_to_sliding_sash")

    # --- Intentional overlaps ---
    # Glass rebated under each sash lip (captured glazing)
    for nm, vinyl_nm, glass_nm in [
        ("left_fixed", "left_sash_vinyl", "left_sash_glass"),
        ("center_fixed", "center_sash_vinyl", "center_sash_glass"),
        ("upper_transom", "upper_sash_vinyl", "upper_sash_glass"),
        ("sliding_sash", "sliding_sash_vinyl", "sliding_sash_glass"),
    ]:
        ctx.allow_overlap(
            nm, nm,
            elem_a=glass_nm,
            elem_b=vinyl_nm,
            reason=f"Glass rebated under {nm} sash lip (captured glazing).",
        )

    # Sashes seated in frame openings
    for nm, vinyl_nm in [
        ("left_fixed", "left_sash_vinyl"),
        ("center_fixed", "center_sash_vinyl"),
        ("upper_transom", "upper_sash_vinyl"),
    ]:
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=vinyl_nm,
            reason=f"{nm} sash ring seated in frame opening.",
        )
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_glass" if nm != "left_fixed" else "left_sash_glass",
            reason=f"{nm} glass within frame opening.",
        )

    # Sliding sash seated in frame track
    ctx.allow_overlap(
        "frame", "sliding_sash",
        elem_a="frame_shell",
        elem_b="sliding_sash_vinyl",
        reason="Sliding sash ring seated in frame track with overlap stile extending into meeting rail.",
    )
    ctx.allow_overlap(
        "frame", "sliding_sash",
        elem_a="frame_shell",
        elem_b="sliding_sash_glass",
        reason="Sliding sash glass within frame opening.",
    )
    # Roller blocks protrude into the sill track (seated rollers)
    for i in range(2):
        ctx.allow_overlap(
            "frame", "sliding_sash",
            elem_a="frame_shell",
            elem_b=f"roller_{i}",
            reason=f"Roller {i} sits in the frame sill track (seated roller, not floating).",
        )

    # Roller blocks mounted on sash bottom
    for i in range(2):
        ctx.allow_overlap(
            "sliding_sash", "sliding_sash",
            elem_a=f"roller_{i}",
            elem_b="sliding_sash_vinyl",
            reason=f"Roller block {i} half-seated in sliding sash bottom rail.",
        )

    # Latch hardware mounted on sash stile
    ctx.allow_overlap(
        "sliding_sash", "sliding_sash",
        elem_a="latch_plate",
        elem_b="sliding_sash_vinyl",
        reason="Latch plate seated on sliding sash stile face.",
    )

    # --- Three-panel structure checks ---
    left_aabb = ctx.part_world_aabb(left_fixed)
    center_aabb = ctx.part_world_aabb(center_fixed)
    slider_aabb = ctx.part_world_aabb(sliding_sash)
    upper_aabb = ctx.part_world_aabb(upper_transom)

    left_w = left_aabb[1][0] - left_aabb[0][0]
    center_w = center_aabb[1][0] - center_aabb[0][0]
    ctx.check(
        "center pane wider than left pane",
        center_w > left_w + 0.05,
        details=f"center_w={center_w:.3f}, left_w={left_w:.3f}",
    )

    # Three columns ordered left to right
    left_cx = (left_aabb[0][0] + left_aabb[1][0]) / 2.0
    center_cx = (center_aabb[0][0] + center_aabb[1][0]) / 2.0
    slider_cx = (slider_aabb[0][0] + slider_aabb[1][0]) / 2.0
    ctx.check(
        "three columns ordered left to right",
        left_cx < center_cx < slider_cx,
        details=f"left={left_cx:.3f}, center={center_cx:.3f}, right={slider_cx:.3f}",
    )

    # Sliding sash below upper transom in closed position
    upper_cz = (upper_aabb[0][2] + upper_aabb[1][2]) / 2.0
    slider_cz = (slider_aabb[0][2] + slider_aabb[1][2]) / 2.0
    ctx.check(
        "sliding sash below upper transom",
        slider_cz < upper_cz,
        details=f"slider_z={slider_cz:.3f}, upper_z={upper_cz:.3f}",
    )

    # --- Joint mechanism checks ---
    ctx.check(
        "slide joint is prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={slide.articulation_type}",
    )
    ctx.check(
        "slide axis is vertical (+Z)",
        abs(slide.axis[2]) > 0.99 and abs(slide.axis[0]) < 0.01 and abs(slide.axis[1]) < 0.01,
        details=f"axis={slide.axis}",
    )
    ctx.check(
        "slide has positive travel",
        slide.motion_limits.upper is not None and slide.motion_limits.upper > 0.1,
        details=f"upper={slide.motion_limits.upper}",
    )

    # --- Closed pose (q=0) ---
    with ctx.pose({slide: 0.0}):
        s_closed = ctx.part_world_aabb(sliding_sash)
        rest_cz = (s_closed[0][2] + s_closed[1][2]) / 2.0

        # Sash seated within frame
        frame_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "closed sash within frame bounds",
            s_closed[0][2] > frame_aabb[0][2] - 0.01 and s_closed[1][2] < frame_aabb[1][2] + 0.01,
            details=f"sash z=[{s_closed[0][2]:.3f},{s_closed[1][2]:.3f}]",
        )

        # Roller blocks at sash bottom
        for i in range(2):
            r_aabb = ctx.part_element_world_aabb(sliding_sash, elem=f"roller_{i}")
            r_cz = (r_aabb[0][2] + r_aabb[1][2]) / 2.0
            ctx.check(
                f"roller_{i} at sash bottom",
                r_cz < rest_cz - 0.05,
                details=f"roller_z={r_cz:.3f}, sash_center_z={rest_cz:.3f}",
            )

        # Overlap stile: sliding sash top extends past the meeting rail zone
        # (the sash top should be near or above the meeting rail center)
        sash_top = s_closed[1][2]
        ctx.check(
            "overlap stile extends above lower opening",
            sash_top > RIGHT_LOWER_OPEN_TOP,
            details=f"sash_top={sash_top:.3f}, lower_opening_top={RIGHT_LOWER_OPEN_TOP:.3f}",
        )

    # --- Open pose: sash slides upward ---
    travel = slide.motion_limits.upper
    with ctx.pose({slide: travel}):
        s_open = ctx.part_world_aabb(sliding_sash)
        open_cz = (s_open[0][2] + s_open[1][2]) / 2.0

        ctx.check(
            "sash moves upward when opened",
            open_cz > rest_cz + 0.05,
            details=f"rest_z={rest_cz:.3f}, open_z={open_cz:.3f}, travel={travel:.3f}",
        )

        # Retained within frame at max travel
        f_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "sash retained in frame at max travel",
            s_open[0][2] > f_aabb[0][2] - 0.02 and s_open[1][2] < f_aabb[1][2] + 0.02,
            details=f"sash z=[{s_open[0][2]:.3f},{s_open[1][2]:.3f}]",
        )

        # No horizontal drift
        open_cx = (s_open[0][0] + s_open[1][0]) / 2.0
        ctx.check(
            "pure vertical slide (no X drift)",
            abs(open_cx - slider_cx) < 0.01,
            details=f"closed_x={slider_cx:.3f}, open_x={open_cx:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
