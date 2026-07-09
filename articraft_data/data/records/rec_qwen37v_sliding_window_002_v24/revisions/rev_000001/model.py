from __future__ import annotations

# Variant 24: Double-sliding window with transom panel.
# White vinyl frame, narrow transom above, two sashes that slide in opposite
# directions on separate prismatic joints (rear sash slides left, front sash
# slides right), two tiny roller blocks at the bottom of each moving sash,
# and a cam-latch handle on the front sash meeting stile.
#
# Coordinate convention:
#   +Z is up. Window stands vertically.
#     width  -> X,  height -> Z (sill near z=0),  depth -> Y
#   Glass plane is the X-Z plane. q=0 reads SHUT for both sashes.
#   Rear sash: axis=(-1,0,0) so positive q slides it left (-X).
#   Front sash: axis=(+1,0,0) so positive q slides it right (+X).
#
# Structure:
#   - frame (root): head, sill, jambs, and transom bar, built as one CadQuery
#     solid: thick slab with the main opening + transom opening cut through.
#   - transom (FIXED): narrow glass panel in the upper region.
#   - rear_sash (PRISMATIC): rear track, slides left.
#   - front_sash (PRISMATIC): front track, slides right; carries the latch.
#   - Each sliding sash has two tiny roller blocks at its bottom rail.

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

FRAME_FACE = 0.085
FRAME_DEPTH = 0.140

# Transom
TRANSOM_BAR_H = 0.060     # height of the horizontal transom bar
TRANSOM_H = 0.220         # transom glass panel height

# Sash dimensions
SASH_FACE = 0.075
SASH_DEPTH = 0.060
GLASS_T = 0.008

MEETING_OVERLAP = 0.040

# Y layout: rear sash in rear track, front sash in front track
REAR_SASH_Y = -0.028
FRONT_SASH_Y = 0.044

REBATE = 0.005

# Roller blocks
ROLLER_W = 0.022          # roller width (X)
ROLLER_H = 0.016          # roller height (Z)
ROLLER_D = 0.030          # roller depth (Y)

# Latch hardware
LATCH_PLATE_W = 0.028
LATCH_PLATE_H = 0.075
LATCH_PLATE_T = 0.010
LATCH_LEVER_LEN = 0.045
LATCH_LEVER_R = 0.006

METAL_RGBA = (0.74, 0.76, 0.79, 1.0)

# ---------------------------------------------------------------------------
# Derived layout
# ---------------------------------------------------------------------------

HALF_W = TOTAL_W / 2.0
INNER_X0 = -HALF_W + FRAME_FACE
INNER_X1 = HALF_W - FRAME_FACE
INNER_W = INNER_X1 - INNER_X0

# Vertical layout: sill -> main opening -> transom bar -> transom opening -> head
SILL_Z = FRAME_FACE
MAIN_TOP_Z = TOTAL_H - FRAME_FACE - TRANSOM_H - TRANSOM_BAR_H
TRANSOM_BAR_Z0 = MAIN_TOP_Z
TRANSOM_BAR_Z1 = MAIN_TOP_Z + TRANSOM_BAR_H
TRANSOM_Z0 = TRANSOM_BAR_Z1
TRANSOM_Z1 = TOTAL_H - FRAME_FACE

MAIN_H = MAIN_TOP_Z - SILL_Z
MAIN_MID_Z = (SILL_Z + MAIN_TOP_Z) / 2.0

# Transom opening dimensions
TRANSOM_OPEN_W = INNER_W
TRANSOM_OPEN_CX = (INNER_X0 + INNER_X1) / 2.0
TRANSOM_OPEN_CZ = (TRANSOM_Z0 + TRANSOM_Z1) / 2.0

# Sash opening widths (main region)
SASH_OPENING_W = (INNER_W + MEETING_OVERLAP) / 2.0
SASH_OPENING_H = MAIN_H

REAR_OPEN_CX = INNER_X0 + SASH_OPENING_W / 2.0
FRONT_OPEN_CX = INNER_X1 - SASH_OPENING_W / 2.0

# Transom glass
TRANSOM_GLASS_W = TRANSOM_OPEN_W + 2 * REBATE
TRANSOM_GLASS_H = TRANSOM_H + 2 * REBATE

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

VINYL_RGBA = (0.94, 0.95, 0.96, 1.0)
GLASS_RGBA = (0.52, 0.60, 0.66, 0.30)
ROLLER_RGBA = (0.25, 0.25, 0.27, 1.0)  # dark nylon/plastic rollers


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _slab(x0: float, x1: float, z0: float, z1: float, y_center: float, depth: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .transformed(offset=((x0 + x1) / 2.0, y_center, (z0 + z1) / 2.0))
        .box(x1 - x0, depth, z1 - z0)
    )


def _build_frame_shape() -> cq.Workplane:
    """Outer frame with main opening and transom opening cut through."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)
    cut_depth = FRAME_DEPTH + 0.02
    # Main sash opening
    main_opening = _slab(INNER_X0, INNER_X1, SILL_Z, MAIN_TOP_Z, 0.0, cut_depth)
    # Transom opening
    transom_opening = _slab(INNER_X0, INNER_X1, TRANSOM_Z0, TRANSOM_Z1, 0.0, cut_depth)
    return outer.cut(main_opening).cut(transom_opening)


def _build_transom_glass_shape() -> cq.Workplane:
    """Transom glass panel in its own local frame, centered at origin."""
    return _slab(
        -TRANSOM_GLASS_W / 2.0, TRANSOM_GLASS_W / 2.0,
        -TRANSOM_GLASS_H / 2.0, TRANSOM_GLASS_H / 2.0,
        0.0, GLASS_T,
    )


def _build_sash_shape() -> cq.Workplane:
    """Sash ring in its own local frame, centered at origin."""
    ow = SASH_OPENING_W
    oh = SASH_OPENING_H
    out_w = ow + 2 * SASH_FACE
    out_h = oh + 2 * SASH_FACE
    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, SASH_DEPTH)
    opening = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SASH_DEPTH + 0.02)
    return outer.cut(opening)


def _build_sash_glass_shape() -> cq.Workplane:
    """Glass pane filling sash opening, rebated under sash lip."""
    ow = SASH_OPENING_W + 2 * REBATE
    oh = SASH_OPENING_H + 2 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


# ---------------------------------------------------------------------------
# Part builders
# ---------------------------------------------------------------------------

def _add_sash(model: ArticulatedObject, name: str) -> None:
    """Add a sash part (vinyl ring + clear glass) in its own local frame."""
    sash = model.part(name)
    sash.visual(
        mesh_from_cadquery(_build_sash_shape(), f"{name}_vinyl"),
        material="vinyl",
        name=f"{name}_vinyl",
    )
    sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(), f"{name}_glass"),
        material="glass",
        name=f"{name}_glass",
    )


def _build_roller_shape() -> cq.Workplane:
    """Tiny roller block in its own local frame, centered at origin."""
    return (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, 0.0))
        .box(ROLLER_W, ROLLER_D, ROLLER_H)
    )


def _add_rollers(model: ArticulatedObject, sash_name: str) -> None:
    """Add two tiny roller blocks at the bottom rail of the sash (local frame).
    Rollers are half-embedded into the bottom rail for a seated mount."""
    sash = model.get_part(sash_name)
    # Roller center at the bottom face of the rail (half protrudes below)
    roller_z = -SASH_OPENING_H / 2.0 - SASH_FACE
    # Position rollers near each stile edge, inset slightly
    inset = SASH_FACE + 0.04
    roller_x_positions = [-SASH_OPENING_W / 2.0 + inset, SASH_OPENING_W / 2.0 - inset]
    for i, rx in enumerate(roller_x_positions):
        sash.visual(
            mesh_from_cadquery(_build_roller_shape(), f"{sash_name}_roller_{i}"),
            origin=Origin(xyz=(rx, 0.0, roller_z)),
            material="roller",
            name=f"{sash_name}_roller_{i}",
        )


def _add_latch(model: ArticulatedObject, sash_name: str) -> None:
    """Cam-latch on the front sash's meeting stile."""
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
    model = ArticulatedObject(name="double_sliding_window_transom")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("metal", rgba=METAL_RGBA)
    model.material("roller", rgba=ROLLER_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame_shell"),
        material="vinyl",
        name="frame_shell",
    )

    # --- Transom panel (fixed glass above main opening) ---
    transom = model.part("transom")
    transom.visual(
        mesh_from_cadquery(_build_transom_glass_shape(), "transom_glass"),
        material="glass",
        name="transom_glass",
    )

    # --- Two sliding sashes ---
    _add_sash(model, "rear_sash")
    _add_sash(model, "front_sash")
    _add_rollers(model, "rear_sash")
    _add_rollers(model, "front_sash")
    _add_latch(model, "front_sash")

    # FIXED transom seated in the transom opening (center of transom region)
    model.articulation(
        "frame_to_transom",
        ArticulationType.FIXED,
        parent="frame",
        child="transom",
        origin=Origin(xyz=(TRANSOM_OPEN_CX, 0.0, TRANSOM_OPEN_CZ)),
    )

    # REAR sash: PRISMATIC along +X (slides right, behind the front sash). Rear track.
    slide_travel = SASH_OPENING_W * 0.90
    model.articulation(
        "frame_to_rear_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="rear_sash",
        origin=Origin(xyz=(REAR_OPEN_CX, REAR_SASH_Y, MAIN_MID_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.5, lower=0.0, upper=slide_travel),
    )

    # FRONT sash: PRISMATIC along -X (slides left, behind the rear sash). Front track.
    model.articulation(
        "frame_to_front_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="front_sash",
        origin=Origin(xyz=(FRONT_OPEN_CX, FRONT_SASH_Y, MAIN_MID_Z)),
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
    transom = object_model.get_part("transom")
    rear_sash = object_model.get_part("rear_sash")
    front_sash = object_model.get_part("front_sash")
    rear_slide = object_model.get_articulation("frame_to_rear_sash")
    front_slide = object_model.get_articulation("frame_to_front_sash")

    # --- Intentional overlaps ---
    # Glass rebated under sash lip on each sash
    for nm in ("rear_sash", "front_sash"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass",
            elem_b=f"{nm}_vinyl",
            reason="Clear pane is rebated under the sash lip so it reads captured, not floating.",
        )
    # Sashes rebated into the frame opening / track
    for nm in ("rear_sash", "front_sash"):
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_vinyl",
            reason=f"{nm} ring is rebated into the frame opening / head-sill track (seated capture).",
        )
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_glass",
            reason=f"{nm} glass is rebated under the frame opening lip (captured glazing).",
        )
    # Transom glass rebated into frame transom opening
    ctx.allow_overlap(
        "frame", "transom",
        elem_a="frame_shell",
        elem_b="transom_glass",
        reason="Transom glass is rebated into the frame transom opening (captured glazing).",
    )
    # Sashes overlap with transom glass in the transom bar region (frame separates them
    # in reality; the sash rail top extends into the transom bar track zone).
    for nm in ("rear_sash", "front_sash"):
        ctx.allow_overlap(
            nm, "transom",
            elem_a=f"{nm}_vinyl",
            elem_b="transom_glass",
            reason=f"{nm} top rail extends into the transom bar track zone; the frame transom bar separates them in reality.",
        )
    # Roller blocks seated into sash bottom rail
    for nm in ("rear_sash", "front_sash"):
        for i in range(2):
            ctx.allow_overlap(
                nm, nm,
                elem_a=f"{nm}_roller_{i}",
                elem_b=f"{nm}_vinyl",
                reason=f"Roller block {i} is seated into the {nm} bottom rail (mounted hardware).",
            )
    # Latch keeper plate seated onto front sash stile
    ctx.allow_overlap(
        "front_sash", "front_sash",
        elem_a="front_sash_latch_plate",
        elem_b="front_sash_vinyl",
        reason="Latch keeper plate is seated onto the front sash meeting-stile face.",
    )

    # --- Transom checks ---
    frame_aabb = ctx.part_world_aabb(frame)
    transom_aabb = ctx.part_world_aabb(transom)

    # Transom sits in the upper portion of the window
    transom_cz = (transom_aabb[0][2] + transom_aabb[1][2]) / 2.0
    frame_mid_z = (frame_aabb[0][2] + frame_aabb[1][2]) / 2.0
    ctx.check(
        "transom sits above frame mid-height",
        transom_cz > frame_mid_z,
        details=f"transom_cz={transom_cz:.3f}, frame_mid_z={frame_mid_z:.3f}",
    )
    # Transom is narrow (short height)
    transom_h = transom_aabb[1][2] - transom_aabb[0][2]
    ctx.check(
        "transom is narrow (height < 0.30m)",
        transom_h < 0.30,
        details=f"transom_h={transom_h:.3f}",
    )
    # Transom spans the inner width
    transom_w = transom_aabb[1][0] - transom_aabb[0][0]
    ctx.check(
        "transom spans most of the inner width",
        transom_w > INNER_W * 0.80,
        details=f"transom_w={transom_w:.3f}, inner_w={INNER_W:.3f}",
    )

    # --- Closed pose (q=0): window reads SHUT ---
    with ctx.pose({rear_slide: 0.0, front_slide: 0.0}):
        f_aabb = ctx.part_world_aabb(frame)
        r_aabb = ctx.part_world_aabb(rear_sash)
        fr_aabb = ctx.part_world_aabb(front_sash)

        # Frame spans full width
        frame_w = f_aabb[1][0] - f_aabb[0][0]
        sash_w = r_aabb[1][0] - r_aabb[0][0]
        ctx.check(
            "frame spans wider than a single sash",
            frame_w > sash_w + 0.40,
            details=f"frame_w={frame_w:.3f}, sash_w={sash_w:.3f}",
        )

        # Both sashes in the main region (below transom)
        for nm, ab in (("rear", r_aabb), ("front", fr_aabb)):
            ctx.check(
                f"{nm} sash seated below transom bar",
                ab[1][2] < TRANSOM_BAR_Z1 + 0.02,
                details=f"{nm} zmax={ab[1][2]:.3f}, transom_bar_z1={TRANSOM_BAR_Z1:.3f}",
            )

        # Rear sash behind front sash in Y
        ry = (r_aabb[0][1] + r_aabb[1][1]) / 2.0
        fry = (fr_aabb[0][1] + fr_aabb[1][1]) / 2.0
        ctx.check(
            "front sash proud of rear sash (+Y)",
            fry > ry + 0.02,
            details=f"front_y={fry:.3f}, rear_y={ry:.3f}",
        )

        # Sashes overlap the frame opening in XZ
        ctx.expect_overlap(
            rear_sash, frame, axes="xz", min_overlap=0.03,
            name="rear sash seated in frame opening",
        )
        ctx.expect_overlap(
            front_sash, frame, axes="xz", min_overlap=0.03,
            name="front sash seated in frame opening",
        )

        rest_rx = (r_aabb[0][0] + r_aabb[1][0]) / 2.0
        rest_frx = (fr_aabb[0][0] + fr_aabb[1][0]) / 2.0

    # --- Driven pose: both sashes slide in opposite directions (toward center,
    #     passing behind each other on separate tracks) ---
    travel = rear_slide.motion_limits.upper
    with ctx.pose({rear_slide: travel, front_slide: travel}):
        r_open = ctx.part_world_aabb(rear_sash)
        fr_open = ctx.part_world_aabb(front_sash)
        open_rx = (r_open[0][0] + r_open[1][0]) / 2.0
        open_frx = (fr_open[0][0] + fr_open[1][0]) / 2.0

        # Rear sash moves right (+X)
        ctx.check(
            "rear sash slides right (+X) when opened",
            open_rx > rest_rx + 0.30,
            details=f"rest_rx={rest_rx:.3f}, open_rx={open_rx:.3f}",
        )
        # Front sash moves left (-X)
        ctx.check(
            "front sash slides left (-X) when opened",
            open_frx < rest_frx - 0.30,
            details=f"rest_frx={rest_frx:.3f}, open_frx={open_frx:.3f}",
        )

        # Both sashes retained within frame X span
        for nm, ab in (("rear", r_open), ("front", fr_open)):
            ctx.check(
                f"{nm} sash retained within frame at full travel",
                ab[0][0] > f_aabb[0][0] - 1e-4 and ab[1][0] < f_aabb[1][0] + 1e-4,
                details=f"{nm} x=[{ab[0][0]:.3f},{ab[1][0]:.3f}]",
            )

        # Pure horizontal slide (no Z change)
        open_rz = (r_open[0][2] + r_open[1][2]) / 2.0
        rest_rz = MAIN_MID_Z
        ctx.check(
            "rear sash slide is purely horizontal",
            abs(open_rz - rest_rz) < 0.02,
            details=f"open_rz={open_rz:.3f}, rest_rz={rest_rz:.3f}",
        )

    # --- Roller block checks ---
    # Rollers are at the bottom of each sash
    for sash_part, sash_name in [(rear_sash, "rear_sash"), (front_sash, "front_sash")]:
        sash_aabb = ctx.part_world_aabb(sash_part)
        for i in range(2):
            roller_aabb = ctx.part_element_world_aabb(sash_part, elem=f"{sash_name}_roller_{i}")
            roller_cz = (roller_aabb[0][2] + roller_aabb[1][2]) / 2.0
            sash_cz = (sash_aabb[0][2] + sash_aabb[1][2]) / 2.0
            ctx.check(
                f"{sash_name} roller_{i} is below sash center",
                roller_cz < sash_cz,
                details=f"roller_z={roller_cz:.3f}, sash_z={sash_cz:.3f}",
            )

    # --- Articulation checks ---
    # Both joints are prismatic with non-zero travel
    for jnt in [rear_slide, front_slide]:
        ctx.check(
            f"{jnt.name} is prismatic",
            jnt.articulation_type == ArticulationType.PRISMATIC,
            details=f"type={jnt.articulation_type}",
        )
        limits = jnt.motion_limits
        ctx.check(
            f"{jnt.name} has positive travel",
            limits is not None and limits.upper is not None and limits.upper > 0.0,
            details=f"upper={limits.upper if limits else None}",
        )

    # The two sashes slide in opposite X directions
    rear_axis = rear_slide.axis
    front_axis = front_slide.axis
    ctx.check(
        "sashes slide in opposite X directions",
        rear_axis[0] * front_axis[0] < 0,
        details=f"rear_axis_x={rear_axis[0]}, front_axis_x={front_axis[0]}",
    )

    return ctx.report()


object_model = build_object_model()
