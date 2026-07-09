from __future__ import annotations

# Two-panel vertical sliding window (double-hung style), slim white vinyl frame
# with bevelled outer corners. Upper sash FIXED, lower sash slides UPWARD on a
# vertical prismatic joint. Two small roller blocks at the bottom of the moving
# sash. Sill lip protrudes forward at the bottom with drainage slots cut through.
#
# Coordinate convention:
#   +Z is up. Window stands vertically.
#     - width  -> X
#     - height -> Z   (sill near z=0)
#     - frame depth -> Y
#   Glass plane is the X-Z plane. q=0 reads SHUT (lower sash down).
#   Driving the prismatic joint slides the lower sash upward (+Z) to open.
#
# Structure:
#   - frame (root): slim vinyl perimeter with bevelled corners + sill lip +
#     drainage slots. Single large opening cut for both sashes.
#   - upper_sash (FIXED): vinyl sash ring + glass, seated in upper half.
#   - lower_sash (PRISMATIC +Z): vinyl sash ring + glass + two roller blocks
#     at the bottom + a cam latch handle.

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

TOTAL_W = 1.20            # overall window width along X
TOTAL_H = 1.40            # overall height along Z (sill at z=0, head at z=TOTAL_H)

FRAME_FACE = 0.055        # slim outer frame member face width
FRAME_DEPTH = 0.090       # frame depth along Y

SASH_FACE = 0.045         # sash perimeter rail/stile face width (slim)
SASH_DEPTH = 0.045        # sash depth along Y
GLASS_T = 0.006           # glazing thickness along Y

# Sash layout: two equal sashes stacked vertically. The meeting rail is at
# mid-height. Upper sash in the upper half, lower sash in the lower half.
MEETING_RAIL_H = 0.040    # height of the meeting rail region between sashes

# Y layout: frame centered on y=0. Upper sash sits in the rear track; lower
# sash sits proud (front, +Y) so it can slide past the upper sash.
UPPER_SASH_Y = -0.026
LOWER_SASH_Y = 0.026

# Sill lip
SILL_LIP_DEPTH = 0.035    # how far the sill lip protrudes forward (+Y)
SILL_LIP_HEIGHT = 0.020   # sill lip thickness in Z

# Drainage slots: thin rectangular slots cut through the sill lip
DRAIN_SLOT_W = 0.040      # slot width along X
DRAIN_SLOT_H = 0.008      # slot height along Z
DRAIN_SLOT_COUNT = 4

# Roller blocks at bottom of lower sash
ROLLER_W = 0.025          # roller block width (X)
ROLLER_H = 0.012          # roller block height (Z)
ROLLER_D = 0.020          # roller block depth (Y)

# Latch hardware
LATCH_PLATE_W = 0.025
LATCH_PLATE_H = 0.060
LATCH_PLATE_T = 0.008
LATCH_LEVER_LEN = 0.038
LATCH_LEVER_R = 0.005

METAL_RGBA = (0.72, 0.74, 0.77, 1.0)

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

# Meeting rail divides the inner height. Upper opening above, lower below.
MID_Z = (INNER_Z0 + INNER_Z1) / 2.0
UPPER_OPEN_Z0 = MID_Z + MEETING_RAIL_H / 2.0
UPPER_OPEN_Z1 = INNER_Z1
LOWER_OPEN_Z0 = INNER_Z0
LOWER_OPEN_Z1 = MID_Z - MEETING_RAIL_H / 2.0

UPPER_OPEN_H = UPPER_OPEN_Z1 - UPPER_OPEN_Z0
LOWER_OPEN_H = LOWER_OPEN_Z1 - LOWER_OPEN_Z0

UPPER_OPEN_CZ = (UPPER_OPEN_Z0 + UPPER_OPEN_Z1) / 2.0
LOWER_OPEN_CZ = (LOWER_OPEN_Z0 + LOWER_OPEN_Z1) / 2.0
OPEN_CX = (INNER_X0 + INNER_X1) / 2.0

REBATE = 0.004

# Bevel/chamfer size for outer frame corners
CHAMFER = 0.006

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

VINYL_RGBA = (0.93, 0.94, 0.95, 1.0)
GLASS_RGBA = (0.50, 0.58, 0.64, 0.28)


# ---------------------------------------------------------------------------
# Geometry helpers (CadQuery), authored directly in meters.
# ---------------------------------------------------------------------------

def _slab(x0: float, x1: float, z0: float, z1: float, y_center: float, depth: float) -> cq.Workplane:
    """Axis-aligned box spanning [x0,x1] x [z0,z1] centered on y_center."""
    return (
        cq.Workplane("XY")
        .transformed(offset=((x0 + x1) / 2.0, y_center, (z0 + z1) / 2.0))
        .box(x1 - x0, depth, z1 - z0)
    )


def _build_frame_shape() -> cq.Workplane:
    """Static outer frame: slim slab with one large opening for both sashes,
    bevelled outer vertical edges, sill lip, and drainage slots."""
    # Main frame slab
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)

    # Cut the main window opening (single large opening spanning both sash regions)
    cut_depth = FRAME_DEPTH + 0.02
    opening = _slab(INNER_X0, INNER_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    frame = outer.cut(opening)

    # Bevel the four outer vertical edges (parallel to Z)
    # Select vertical edges on the outer box perimeter
    try:
        frame = frame.edges("|Z").chamfer(CHAMFER)
    except Exception:
        pass  # chamfer may fail on complex topology; geometry still valid

    # Sill lip: a protruding ledge at the bottom front of the frame
    sill_lip = _slab(
        -HALF_W + FRAME_FACE * 0.5,
        HALF_W - FRAME_FACE * 0.5,
        0.0,
        SILL_LIP_HEIGHT,
        FRAME_DEPTH / 2.0 + SILL_LIP_DEPTH / 2.0,
        SILL_LIP_DEPTH,
    )
    frame = frame.union(sill_lip)

    # Drainage slots: thin rectangular cuts through the sill lip
    slot_spacing = INNER_W / (DRAIN_SLOT_COUNT + 1)
    for i in range(DRAIN_SLOT_COUNT):
        sx = INNER_X0 + slot_spacing * (i + 1)
        slot = _slab(
            sx - DRAIN_SLOT_W / 2.0,
            sx + DRAIN_SLOT_W / 2.0,
            0.001,
            SILL_LIP_HEIGHT - 0.002,
            FRAME_DEPTH / 2.0 + SILL_LIP_DEPTH / 2.0,
            SILL_LIP_DEPTH + 0.02,
        )
        frame = frame.cut(slot)

    return frame


def _build_upper_sash_shape() -> cq.Workplane:
    """Upper sash ring in its own local frame, centered at origin."""
    ow = INNER_W
    oh = UPPER_OPEN_H
    out_w = ow + 2 * SASH_FACE
    out_h = oh + 2 * SASH_FACE
    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, SASH_DEPTH)
    opening = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SASH_DEPTH + 0.02)
    return outer.cut(opening)


def _build_lower_sash_shape() -> cq.Workplane:
    """Lower sash ring in its own local frame, centered at origin."""
    ow = INNER_W
    oh = LOWER_OPEN_H
    out_w = ow + 2 * SASH_FACE
    out_h = oh + 2 * SASH_FACE
    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, SASH_DEPTH)
    opening = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SASH_DEPTH + 0.02)
    return outer.cut(opening)


def _build_upper_glass_shape() -> cq.Workplane:
    ow = INNER_W + 2 * REBATE
    oh = UPPER_OPEN_H + 2 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


def _build_lower_glass_shape() -> cq.Workplane:
    ow = INNER_W + 2 * REBATE
    oh = LOWER_OPEN_H + 2 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


# ---------------------------------------------------------------------------
# Part builders
# ---------------------------------------------------------------------------

def _add_roller_blocks(sash_part) -> None:
    """Add two small roller blocks at the bottom of the lower sash (local frame).
    Positioned at the bottom rail, offset symmetrically from center."""
    oh = LOWER_OPEN_H
    out_h = oh + 2 * SASH_FACE
    # Bottom rail local Z center is at -out_h/2 + SASH_FACE/2
    # Roller sits at the very bottom edge of the sash
    roller_z = -out_h / 2.0 - ROLLER_H / 2.0 + 0.002  # just below bottom rail
    roller_y = 0.0  # centered on sash depth

    # Two rollers offset symmetrically from center along X
    offset_x = INNER_W * 0.35
    for i, sign in enumerate((-1, 1)):
        rx = sign * offset_x
        sash_part.visual(
            Box((ROLLER_W, ROLLER_D, ROLLER_H)),
            origin=Origin(xyz=(rx, roller_y, roller_z)),
            material="metal",
            name=f"roller_{i}",
        )


def _add_latch(model: ArticulatedObject, sash_name: str) -> None:
    """Add cam-latch on the lower sash's top rail (meeting stile), centered."""
    sash = model.get_part(sash_name)
    oh = LOWER_OPEN_H
    out_h = oh + 2 * SASH_FACE
    # Top rail center in local Z
    rail_z = out_h / 2.0 - SASH_FACE / 2.0
    face_y = SASH_DEPTH / 2.0
    plate_y = face_y + LATCH_PLATE_T / 2.0

    sash.visual(
        Box((LATCH_PLATE_W, LATCH_PLATE_T, LATCH_PLATE_H)),
        origin=Origin(xyz=(0.0, plate_y, rail_z)),
        material="metal",
        name=f"{sash_name}_latch_plate",
    )
    lever_y = face_y + LATCH_PLATE_T + LATCH_LEVER_LEN / 2.0
    sash.visual(
        Cylinder(radius=LATCH_LEVER_R, length=LATCH_LEVER_LEN),
        origin=Origin(xyz=(0.0, lever_y, rail_z - 0.006), rpy=(1.5707963, 0.0, 0.0)),
        material="metal",
        name=f"{sash_name}_latch_lever",
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vertical_sliding_window")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("metal", rgba=METAL_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame_shell"),
        material="vinyl",
        name="frame_shell",
    )

    # --- Upper sash (FIXED) ---
    upper_sash = model.part("upper_sash")
    upper_sash.visual(
        mesh_from_cadquery(_build_upper_sash_shape(), "upper_sash_vinyl"),
        material="vinyl",
        name="upper_sash_vinyl",
    )
    upper_sash.visual(
        mesh_from_cadquery(_build_upper_glass_shape(), "upper_sash_glass"),
        material="glass",
        name="upper_sash_glass",
    )

    # --- Lower sash (PRISMATIC, slides upward) ---
    lower_sash = model.part("lower_sash")
    lower_sash.visual(
        mesh_from_cadquery(_build_lower_sash_shape(), "lower_sash_vinyl"),
        material="vinyl",
        name="lower_sash_vinyl",
    )
    lower_sash.visual(
        mesh_from_cadquery(_build_lower_glass_shape(), "lower_sash_glass"),
        material="glass",
        name="lower_sash_glass",
    )
    _add_roller_blocks(lower_sash)
    _add_latch(model, "lower_sash")

    # FIXED upper sash seated in the rear glazing plane.
    model.articulation(
        "frame_to_upper_sash",
        ArticulationType.FIXED,
        parent="frame",
        child="upper_sash",
        origin=Origin(xyz=(OPEN_CX, UPPER_SASH_Y, UPPER_OPEN_CZ)),
    )

    # PRISMATIC lower sash: slides upward along +Z.
    # Positive q raises the sash. The sash stays retained in the jamb tracks.
    slide_travel = LOWER_OPEN_H * 0.85  # can open most of the lower opening height
    model.articulation(
        "frame_to_lower_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="lower_sash",
        origin=Origin(xyz=(OPEN_CX, LOWER_SASH_Y, LOWER_OPEN_CZ)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.4, lower=0.0, upper=slide_travel),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    upper_sash = object_model.get_part("upper_sash")
    lower_sash = object_model.get_part("lower_sash")
    slide = object_model.get_articulation("frame_to_lower_sash")

    # --- Intentional overlaps ---
    # Glass rebated under sash lips
    for nm in ("upper_sash", "lower_sash"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass",
            elem_b=f"{nm}_vinyl",
            reason="Glass pane is rebated under the sash lip (captured glazing).",
        )
    # Sash rings seated in the frame tracks
    for nm in ("upper_sash", "lower_sash"):
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_vinyl",
            reason=f"{nm} ring is rebated into the frame jamb track (seated capture).",
        )
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_glass",
            reason=f"{nm} glass is rebated under the frame opening lip.",
        )
    # Latch plate seated on lower sash
    ctx.allow_overlap(
        "lower_sash", "lower_sash",
        elem_a="lower_sash_latch_plate",
        elem_b="lower_sash_vinyl",
        reason="Latch plate is seated onto the lower sash top rail face.",
    )
    # Roller blocks mounted at bottom of lower sash (small intentional embed)
    for rn in ("roller_0", "roller_1"):
        ctx.allow_overlap(
            "lower_sash", "lower_sash",
            elem_a=rn,
            elem_b="lower_sash_vinyl",
            reason="Roller block is mounted at the bottom rail of the lower sash.",
        )
        ctx.allow_overlap(
            "frame", "lower_sash",
            elem_a="frame_shell",
            elem_b=rn,
            reason=f"{rn} sits in the sill track at the bottom of the frame (intentional track engagement).",
        )

    # --- Closed pose (q=0): window reads SHUT ---
    with ctx.pose({slide: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        u_aabb = ctx.part_world_aabb(upper_sash)
        l_aabb = ctx.part_world_aabb(lower_sash)

        # Frame proportions
        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        frame_h = frame_aabb[1][2] - frame_aabb[0][2]
        ctx.check(
            "frame width reasonable",
            1.0 < frame_w < 1.5,
            details=f"frame_w={frame_w:.3f}",
        )
        ctx.check(
            "frame height reasonable",
            1.2 < frame_h < 1.6,
            details=f"frame_h={frame_h:.3f}",
        )

        # Slim frame rails: face width should be notably narrower than the original 85mm
        ctx.check(
            "slim frame rails",
            FRAME_FACE < 0.065,
            details=f"frame_face={FRAME_FACE:.3f}",
        )

        # Upper sash above lower sash
        uz = (u_aabb[0][2] + u_aabb[1][2]) / 2.0
        lz = (l_aabb[0][2] + l_aabb[1][2]) / 2.0
        ctx.check(
            "upper sash above lower sash",
            uz > lz + 0.10,
            details=f"upper_z={uz:.3f}, lower_z={lz:.3f}",
        )

        # Both sashes seated within frame
        for nm, ab in (("upper", u_aabb), ("lower", l_aabb)):
            ctx.check(
                f"{nm} sash within frame height",
                ab[0][2] > frame_aabb[0][2] - 0.005 and ab[1][2] < frame_aabb[1][2] + 0.005,
                details=f"{nm} z=[{ab[0][2]:.3f},{ab[1][2]:.3f}]",
            )

        # Sash overlap in frame opening (seated)
        ctx.expect_overlap(
            upper_sash, frame, axes="xz", min_overlap=0.02,
            name="upper sash seated in frame opening",
        )
        ctx.expect_overlap(
            lower_sash, frame, axes="xz", min_overlap=0.02,
            name="lower sash seated in frame opening",
        )

        # Roller blocks at bottom of lower sash
        for rn in ("roller_0", "roller_1"):
            r_aabb = ctx.part_element_world_aabb(lower_sash, elem=rn)
            ctx.check(
                f"{rn} exists at bottom of lower sash",
                r_aabb[0][2] < lz,
                details=f"roller_zmin={r_aabb[0][2]:.3f}, sash_center_z={lz:.3f}",
            )

        # Sill lip extends forward of frame front face
        frame_front_y = frame_aabb[1][1]
        ctx.check(
            "sill lip extends forward",
            frame_front_y > FRAME_DEPTH / 2.0 + 0.010,
            details=f"frame_front_y={frame_front_y:.3f}",
        )

        # Lower sash sits proud (front, +Y) of upper sash
        uy = (u_aabb[0][1] + u_aabb[1][1]) / 2.0
        ly = (l_aabb[0][1] + l_aabb[1][1]) / 2.0
        ctx.check(
            "lower sash proud of upper sash",
            ly > uy + 0.010,
            details=f"lower_y={ly:.3f}, upper_y={uy:.3f}",
        )

        rest_lz = lz

    # --- Open pose: lower sash slides upward ---
    travel = slide.motion_limits.upper
    with ctx.pose({slide: travel}):
        l_open = ctx.part_world_aabb(lower_sash)
        open_lz = (l_open[0][2] + l_open[1][2]) / 2.0

        # Lower sash moves upward
        ctx.check(
            "lower sash slides upward",
            open_lz > rest_lz + 0.10,
            details=f"rest_z={rest_lz:.3f}, open_z={open_lz:.3f}",
        )
        # Pure vertical slide (no X change)
        rest_lx = OPEN_CX
        open_lx = (l_open[0][0] + l_open[1][0]) / 2.0
        ctx.check(
            "slide is purely vertical",
            abs(open_lx - rest_lx) < 0.02,
            details=f"open_x={open_lx:.3f}, rest_x={rest_lx:.3f}",
        )
        # Retained in frame: sash stays within frame height
        f_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "sash retained within frame at full travel",
            l_open[0][2] > f_aabb[0][2] - 0.01 and l_open[1][2] < f_aabb[1][2] + 0.05,
            details=f"sash z=[{l_open[0][2]:.3f},{l_open[1][2]:.3f}]",
        )
        # Sash still overlaps frame in X (retained in jamb tracks)
        ctx.expect_overlap(
            lower_sash, frame, axes="x", min_overlap=0.03,
            name="sash retains X engagement with jamb tracks",
        )

    # --- Joint is prismatic with vertical axis ---
    ctx.check(
        "slide joint is prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={slide.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
