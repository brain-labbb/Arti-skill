from __future__ import annotations

# Variant 07: Two-panel horizontal sliding window with BOTH sashes sliding in
# opposite directions on separate prismatic joints. Muntin grid bars on the
# right sash only. Deep track grooves along top and bottom frame rails.
# Rubber gasket strips around glass panes on both sashes.
#
# Coordinate convention:
#   +Z is up. Window stands vertically.
#     - width  -> X
#     - height -> Z   (sill near z=0)
#     - frame depth / glazing thickness / slide-normal -> Y
#   Glass plane is the X-Z plane. q=0 reads SHUT for both sashes.
#   Left sash slides toward +X (right) to open.
#   Right sash slides toward -X (left) to open.
#
# Structure:
#   - frame (static root): head, sill, two jambs with deep box profile.
#     Track grooves are visible channels in the head and sill rails.
#   - left_sash (PRISMATIC +X): vinyl sash ring + glass + rubber gasket
#   - right_sash (PRISMATIC -X): vinyl sash ring + glass + rubber gasket
#     + muntin grid bars
#   - Latch handle on right sash's meeting stile.

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

TOTAL_W = 1.52            # overall window width along X
TOTAL_H = 1.72            # overall height along Z (sill at z=0, head at z=TOTAL_H)

FRAME_FACE = 0.085        # outer frame member face width (chunky vinyl)
FRAME_DEPTH = 0.140       # deep box section along Y (thick patio-slider profile)

MEETING_OVERLAP = 0.040   # the two sash stiles overlap by this much at center

SASH_FACE = 0.075         # sash perimeter rail/stile face width (chunky)
SASH_DEPTH = 0.060        # sash depth along Y
GLASS_T = 0.008           # glazing thickness along Y

# Y layout: frame box centered on y=0. Both sashes sit in separate tracks.
# Left sash is in the rear track; right sash is in the front track.
LEFT_SASH_Y = -0.028      # rear track center (Y)
RIGHT_SASH_Y = 0.044      # front track center (Y), proud toward +Y

REBATE = 0.005            # glass tucks under the sash lip by this much

# Track grooves
TRACK_GROOVE_W = 0.018    # groove width along Y
TRACK_GROOVE_D = 0.025    # groove depth (how deep into head/sill)

# Rubber gasket
GASKET_W = 0.006          # gasket strip width (visible face)
GASKET_T = 0.004          # gasket thickness along Y

# Muntin bars (right sash only)
MUNTIN_W = 0.018          # muntin bar face width
MUNTIN_T = 0.012          # muntin bar thickness along Y

# Latch (cam lock) hardware
LATCH_PLATE_W = 0.028
LATCH_PLATE_H = 0.075
LATCH_PLATE_T = 0.010
LATCH_LEVER_LEN = 0.045
LATCH_LEVER_R = 0.006

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

SASH_OPENING_W = (INNER_W + MEETING_OVERLAP) / 2.0
SASH_OPENING_H = INNER_H

# Opening centers (world X) of each sash at rest (closed, q=0)
LEFT_OPEN_CX = INNER_X0 + SASH_OPENING_W / 2.0
RIGHT_OPEN_CX = INNER_X1 - SASH_OPENING_W / 2.0
MID_CZ = (INNER_Z0 + INNER_Z1) / 2.0

# Muntin grid: 2 columns x 3 rows on right sash
MUNTIN_COLS = 2
MUNTIN_ROWS = 3

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

VINYL_RGBA = (0.94, 0.95, 0.96, 1.0)
GLASS_RGBA = (0.52, 0.60, 0.66, 0.30)
GASKET_RGBA = (0.12, 0.12, 0.13, 1.0)     # dark rubber
GROOVE_RGBA = (0.18, 0.18, 0.20, 1.0)     # dark channel
METAL_RGBA = (0.74, 0.76, 0.79, 1.0)


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _slab(x0: float, x1: float, z0: float, z1: float, y_center: float, depth: float) -> cq.Workplane:
    """Axis-aligned box spanning [x0,x1] x [z0,z1] in the X-Z plane, centered on
    y_center with the given Y depth."""
    return (
        cq.Workplane("XY")
        .transformed(offset=((x0 + x1) / 2.0, y_center, (z0 + z1) / 2.0))
        .box(x1 - x0, depth, z1 - z0)
    )


def _build_frame_shape() -> cq.Workplane:
    """Static outer frame: thick slab with the sash opening cut through."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)
    cut_depth = FRAME_DEPTH + 0.02
    opening = _slab(INNER_X0, INNER_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    return outer.cut(opening)


def _build_track_grooves() -> cq.Workplane:
    """Deep track grooves in the head and sill rails. Two parallel channels
    (one for each sash track) running the full inner width along X."""
    # Grooves sit on the inner face of head and sill, cut into the vinyl.
    # Each groove is a thin box recessed into the rail face.
    groove_y_positions = [LEFT_SASH_Y, RIGHT_SASH_Y]
    rail_z_positions = [
        (INNER_Z0 - TRACK_GROOVE_D / 2.0),           # sill groove
        (INNER_Z1 + TRACK_GROOVE_D / 2.0),           # head groove
    ]
    result = None
    for gz in rail_z_positions:
        for gy in groove_y_positions:
            g = _slab(
                INNER_X0, INNER_X1,
                gz - TRACK_GROOVE_D / 2.0, gz + TRACK_GROOVE_D / 2.0,
                gy, TRACK_GROOVE_W,
            )
            result = g if result is None else result.union(g)
    return result


def _build_sash_shape() -> cq.Workplane:
    """One sash ring in its own local frame, centered on local origin."""
    ow = SASH_OPENING_W
    oh = SASH_OPENING_H
    out_w = ow + 2 * SASH_FACE
    out_h = oh + 2 * SASH_FACE
    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, SASH_DEPTH)
    opening = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SASH_DEPTH + 0.02)
    return outer.cut(opening)


def _build_glass_shape() -> cq.Workplane:
    """Single clear pane filling the sash opening, rebated under the sash lip."""
    ow = SASH_OPENING_W + 2 * REBATE
    oh = SASH_OPENING_H + 2 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


def _build_gasket_frame() -> cq.Workplane:
    """Rubber gasket strip around the glass pane perimeter (sash-local frame).
    A thin frame sitting between glass edge and sash inner lip."""
    ow = SASH_OPENING_W + 2 * REBATE
    oh = SASH_OPENING_H + 2 * REBATE
    outer_w = ow + 2 * GASKET_W
    outer_h = oh + 2 * GASKET_W
    # Outer solid
    outer = _slab(-outer_w / 2.0, outer_w / 2.0, -outer_h / 2.0, outer_h / 2.0, 0.0, GASKET_T)
    # Cut inner opening (same as glass size)
    inner = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GASKET_T + 0.002)
    return outer.cut(inner)


def _build_muntin_bars() -> cq.Workplane:
    """Muntin grid bars for the right sash: (MUNTIN_COLS-1) vertical bars +
    (MUNTIN_ROWS-1) horizontal bars, forming a COLS x ROWS grid over the glass.
    Authored in sash-local frame."""
    ow = SASH_OPENING_W
    oh = SASH_OPENING_H
    result = None
    # Vertical bars (divide into MUNTIN_COLS columns)
    for i in range(1, MUNTIN_COLS):
        bx = -ow / 2.0 + i * (ow / MUNTIN_COLS)
        bar = _slab(
            bx - MUNTIN_W / 2.0, bx + MUNTIN_W / 2.0,
            -oh / 2.0, oh / 2.0,
            0.0, MUNTIN_T,
        )
        result = bar if result is None else result.union(bar)
    # Horizontal bars (divide into MUNTIN_ROWS rows)
    for j in range(1, MUNTIN_ROWS):
        bz = -oh / 2.0 + j * (oh / MUNTIN_ROWS)
        bar = _slab(
            -ow / 2.0, ow / 2.0,
            bz - MUNTIN_W / 2.0, bz + MUNTIN_W / 2.0,
            0.0, MUNTIN_T,
        )
        result = result.union(bar) if result is not None else bar
    return result


# ---------------------------------------------------------------------------
# Part builders
# ---------------------------------------------------------------------------

def _add_sash_with_gasket(model: ArticulatedObject, name: str) -> None:
    """Add a sash part with vinyl ring, glass pane, and rubber gasket."""
    sash = model.part(name)
    sash.visual(
        mesh_from_cadquery(_build_sash_shape(), f"{name}_vinyl"),
        material="vinyl",
        name=f"{name}_vinyl",
    )
    sash.visual(
        mesh_from_cadquery(_build_glass_shape(), f"{name}_glass"),
        material="glass",
        name=f"{name}_glass",
    )
    sash.visual(
        mesh_from_cadquery(_build_gasket_frame(), f"{name}_gasket"),
        material="gasket",
        name=f"{name}_gasket",
    )


def _add_latch(model: ArticulatedObject, sash_name: str) -> None:
    """Add cam-latch hardware on the right sash's meeting (inner/left) stile."""
    sash = model.get_part(sash_name)
    stile_x = -SASH_OPENING_W / 2.0 - SASH_FACE / 2.0
    face_y = SASH_DEPTH / 2.0
    plate_y = face_y + LATCH_PLATE_T / 2.0

    sash.visual(
        Box((LATCH_PLATE_W, LATCH_PLATE_T, LATCH_PLATE_H)),
        origin=Origin(xyz=(stile_x, plate_y, 0.0)),
        material="metal",
        name=f"{sash_name}_latch_plate",
    )
    lever_y = face_y + LATCH_PLATE_T + LATCH_LEVER_LEN / 2.0
    sash.visual(
        Cylinder(radius=LATCH_LEVER_R, length=LATCH_LEVER_LEN),
        origin=Origin(xyz=(stile_x, lever_y, -0.008), rpy=(1.5707963, 0.0, 0.0)),
        material="metal",
        name=f"{sash_name}_latch_lever",
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_slide_window_muntin")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("gasket", rgba=GASKET_RGBA)
    model.material("groove", rgba=GROOVE_RGBA)
    model.material("metal", rgba=METAL_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame_shell"),
        material="vinyl",
        name="frame_shell",
    )
    # Track grooves (dark channels in head and sill)
    frame.visual(
        mesh_from_cadquery(_build_track_grooves(), "track_grooves"),
        material="groove",
        name="track_grooves",
    )

    # --- Left sash (rear track, slides +X to open) ---
    _add_sash_with_gasket(model, "left_sash")

    # --- Right sash (front track, slides -X to open, has muntin bars) ---
    _add_sash_with_gasket(model, "right_sash")
    # Muntin grid bars on the right sash only
    right_sash = model.get_part("right_sash")
    right_sash.visual(
        mesh_from_cadquery(_build_muntin_bars(), "right_sash_muntins"),
        material="vinyl",
        name="right_sash_muntins",
    )
    _add_latch(model, "right_sash")

    # --- Articulations ---
    slide_travel = SASH_OPENING_W * 0.85

    # Left sash: prismatic along +X. Positive q slides right (toward right sash) to open.
    model.articulation(
        "frame_to_left_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="left_sash",
        origin=Origin(xyz=(LEFT_OPEN_CX, LEFT_SASH_Y, MID_CZ)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.5, lower=0.0, upper=slide_travel),
    )

    # Right sash: prismatic along -X. Positive q slides left (toward left sash) to open.
    model.articulation(
        "frame_to_right_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="right_sash",
        origin=Origin(xyz=(RIGHT_OPEN_CX, RIGHT_SASH_Y, MID_CZ)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.5, lower=0.0, upper=slide_travel),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    left_sash = object_model.get_part("left_sash")
    right_sash = object_model.get_part("right_sash")
    left_slide = object_model.get_articulation("frame_to_left_sash")
    right_slide = object_model.get_articulation("frame_to_right_sash")

    # --- Intentional overlaps ---
    # Glass + gasket tuck under the vinyl sash lip (captured glazing).
    for nm in ("left_sash", "right_sash"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass",
            elem_b=f"{nm}_vinyl",
            reason="Glass pane is rebated under the sash lip so it reads captured.",
        )
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_gasket",
            elem_b=f"{nm}_vinyl",
            reason="Rubber gasket sits between glass edge and sash inner lip (seated compression).",
        )
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_gasket",
            elem_b=f"{nm}_glass",
            reason="Rubber gasket wraps the glass perimeter edge (captured glazing seal).",
        )
    # Sash rings lap the frame opening edge (seated in track).
    for nm in ("left_sash", "right_sash"):
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_vinyl",
            reason=f"{nm} ring is rebated into the frame head-sill track (seated capture).",
        )
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_glass",
            reason=f"{nm} glass is rebated under the frame opening lip.",
        )
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_gasket",
            reason=f"{nm} gasket contacts the frame track (seated seal).",
        )
        ctx.allow_overlap(
            "frame", nm,
            elem_a="track_grooves",
            elem_b=f"{nm}_vinyl",
            reason=f"{nm} sash rides in the track groove channel.",
        )
        ctx.allow_overlap(
            "frame", nm,
            elem_a="track_grooves",
            elem_b=f"{nm}_glass",
            reason=f"{nm} glass passes through the track groove region when seated in the sash opening.",
        )
        ctx.allow_overlap(
            "frame", nm,
            elem_a="track_grooves",
            elem_b=f"{nm}_gasket",
            reason=f"{nm} gasket passes through the track groove region when seated.",
        )
    # Muntin bars sit on the right sash glass surface.
    ctx.allow_overlap(
        "right_sash", "right_sash",
        elem_a="right_sash_muntins",
        elem_b="right_sash_glass",
        reason="Muntin grid bars are mounted on the glass surface of the right sash.",
    )
    ctx.allow_overlap(
        "right_sash", "right_sash",
        elem_a="right_sash_muntins",
        elem_b="right_sash_vinyl",
        reason="Muntin bars connect to the sash vinyl frame at their ends.",
    )
    # Muntin bars extend to the sash opening edges which overlap with the frame
    # rebate region (intentional seated configuration).
    ctx.allow_overlap(
        "frame", "right_sash",
        elem_a="frame_shell",
        elem_b="right_sash_muntins",
        reason="Muntin bars extend to the sash opening edges which overlap with the frame rebate region.",
    )
    # Latch keeper plate seated on the right sash stile face.
    ctx.allow_overlap(
        "right_sash", "right_sash",
        elem_a="right_sash_latch_plate",
        elem_b="right_sash_vinyl",
        reason="Latch keeper plate is seated onto the right sash meeting-stile face.",
    )

    # --- Structural checks ---
    # Both sashes have prismatic (non-fixed) joints.
    ctx.check(
        "left_sash joint is prismatic",
        left_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={left_slide.articulation_type}",
    )
    ctx.check(
        "right_sash joint is prismatic",
        right_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={right_slide.articulation_type}",
    )
    # Opposite slide axes.
    left_axis = left_slide.axis
    right_axis = right_slide.axis
    ctx.check(
        "sash joints slide in opposite X directions",
        (left_axis[0] > 0 and right_axis[0] < 0) or (left_axis[0] < 0 and right_axis[0] > 0),
        details=f"left_axis={left_axis}, right_axis={right_axis}",
    )
    ctx.check(
        "slide axes are purely horizontal (no Y or Z component)",
        abs(left_axis[1]) < 1e-6 and abs(left_axis[2]) < 1e-6
        and abs(right_axis[1]) < 1e-6 and abs(right_axis[2]) < 1e-6,
        details=f"left_axis={left_axis}, right_axis={right_axis}",
    )

    # --- Muntin bars exist only on right sash ---
    right_visuals = [v.name for v in right_sash.visuals]
    left_visuals = [v.name for v in left_sash.visuals]
    ctx.check(
        "right sash has muntin bars",
        "right_sash_muntins" in right_visuals,
        details=f"right_visuals={right_visuals}",
    )
    ctx.check(
        "left sash has no muntin bars",
        "left_sash_muntins" not in left_visuals,
        details=f"left_visuals={left_visuals}",
    )

    # --- Gasket strips exist on both sashes ---
    ctx.check(
        "left sash has rubber gasket",
        "left_sash_gasket" in left_visuals,
        details=f"left_visuals={left_visuals}",
    )
    ctx.check(
        "right sash has rubber gasket",
        "right_sash_gasket" in right_visuals,
        details=f"right_visuals={right_visuals}",
    )

    # --- Track grooves exist on the frame ---
    frame_visuals = [v.name for v in frame.visuals]
    ctx.check(
        "frame has track grooves",
        "track_grooves" in frame_visuals,
        details=f"frame_visuals={frame_visuals}",
    )

    # --- Closed pose (q=0): window reads SHUT ---
    with ctx.pose({left_slide: 0.0, right_slide: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        l_aabb = ctx.part_world_aabb(left_sash)
        r_aabb = ctx.part_world_aabb(right_sash)

        # Frame spans the full width.
        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        ctx.check(
            "frame spans full window width",
            frame_w > 1.40,
            details=f"frame_w={frame_w:.3f}",
        )
        # Left sash center is left of right sash center.
        lx = (l_aabb[0][0] + l_aabb[1][0]) / 2.0
        rx = (r_aabb[0][0] + r_aabb[1][0]) / 2.0
        ctx.check(
            "left sash is left of right sash at rest",
            lx < rx,
            details=f"left_x={lx:.3f}, right_x={rx:.3f}",
        )
        # Right sash sits proud (+Y) of left sash.
        ly = (l_aabb[0][1] + l_aabb[1][1]) / 2.0
        ry = (r_aabb[0][1] + r_aabb[1][1]) / 2.0
        ctx.check(
            "right sash proud of left sash in Y",
            ry > ly + 0.02,
            details=f"right_y={ry:.3f}, left_y={ly:.3f}",
        )
        # Both sashes seated in frame.
        ctx.expect_overlap(left_sash, frame, axes="xz", min_overlap=0.03,
                           name="left sash seated in frame opening")
        ctx.expect_overlap(right_sash, frame, axes="xz", min_overlap=0.03,
                           name="right sash seated in frame opening")

        rest_lx = lx
        rest_rx = rx
        rest_lz = (l_aabb[0][2] + l_aabb[1][2]) / 2.0
        rest_rz = (r_aabb[0][2] + r_aabb[1][2]) / 2.0

    # --- Open pose: both sashes slide toward center (opposite directions) ---
    travel = left_slide.motion_limits.upper
    with ctx.pose({left_slide: travel, right_slide: travel}):
        l_open = ctx.part_world_aabb(left_sash)
        r_open = ctx.part_world_aabb(right_sash)
        open_lx = (l_open[0][0] + l_open[1][0]) / 2.0
        open_rx = (r_open[0][0] + r_open[1][0]) / 2.0

        # Left sash moved in +X direction.
        ctx.check(
            "left sash opens toward +X (slides right)",
            open_lx > rest_lx + 0.20,
            details=f"rest_lx={rest_lx:.3f}, open_lx={open_lx:.3f}",
        )
        # Right sash moved in -X direction.
        ctx.check(
            "right sash opens toward -X (slides left)",
            open_rx < rest_rx - 0.20,
            details=f"rest_rx={rest_rx:.3f}, open_rx={open_rx:.3f}",
        )
        # Pure horizontal slide (no Z change).
        open_lz = (l_open[0][2] + l_open[1][2]) / 2.0
        open_rz = (r_open[0][2] + r_open[1][2]) / 2.0
        ctx.check(
            "left sash slide is purely horizontal",
            abs(open_lz - rest_lz) < 0.02,
            details=f"rest_z={rest_lz:.3f}, open_z={open_lz:.3f}",
        )
        ctx.check(
            "right sash slide is purely horizontal",
            abs(open_rz - rest_rz) < 0.02,
            details=f"rest_z={rest_rz:.3f}, open_z={open_rz:.3f}",
        )
        # Retained insertion: sashes stay within frame X span.
        f_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "left sash retained within frame at full travel",
            l_open[0][0] > f_aabb[0][0] - 1e-4 and l_open[1][0] < f_aabb[1][0] + 1e-4,
            details=f"sash x=[{l_open[0][0]:.3f},{l_open[1][0]:.3f}]",
        )
        ctx.check(
            "right sash retained within frame at full travel",
            r_open[0][0] > f_aabb[0][0] - 1e-4 and r_open[1][0] < f_aabb[1][0] + 1e-4,
            details=f"sash x=[{r_open[0][0]:.3f},{r_open[1][0]:.3f}]",
        )

    return ctx.report()


object_model = build_object_model()
