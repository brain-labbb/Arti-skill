from __future__ import annotations

# Three-panel horizontal sliding window, white vinyl frame.
# Left fixed sash, wider fixed center sash, right sliding sash.
# Revolute latch at the meeting rail, two roller blocks on the slider,
# sill lip with drainage slots.
#
# Coordinate convention:
#   +Z up, window stands vertically.
#     width  -> X
#     height -> Z   (sill near z=0)
#     depth  -> Y   (+Y is front / exterior)
#   Glass plane is X-Z. Slide q=0 is SHUT; positive q slides the right sash
#   toward center (-X). Latch q=0 is locked; positive q rotates the lever.

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
# Dimensions (meters)
# ---------------------------------------------------------------------------

TOTAL_W = 1.80
TOTAL_H = 1.72

FRAME_FACE = 0.085       # outer frame member face width
FRAME_DEPTH = 0.140      # deep box section along Y

MULLION_W = 0.040        # vertical mullion bar width between panels

# Three panel openings: left + center (wider) + right
LEFT_OPEN_W = 0.425
CENTER_OPEN_W = 0.700
RIGHT_OPEN_W = 0.425

SASH_FACE = 0.075        # sash rail/stile face width (slider sash)
SASH_DEPTH = 0.060       # sash depth along Y (slider sash)
FIXED_SASH_FACE = 0.018  # thin glazing-bead frame for fixed panels
FIXED_SASH_DEPTH = 0.030 # thin fixed panel depth
GLASS_T = 0.008
REBATE = 0.005

# Y layout: fixed sashes in rear track, slider in front track
FIXED_SASH_Y = -0.028
SLIDE_SASH_Y = 0.044

# Sill lip
SILL_LIP_EXTEND = 0.025  # protrusion forward from frame front face
SILL_LIP_T = 0.012       # lip plate thickness

# Drainage slots
DRAIN_W = 0.025
DRAIN_H = 0.012
DRAIN_COUNT = 4

# Latch hardware
LATCH_LEVER_LEN = 0.045
LATCH_LEVER_R = 0.006
LATCH_BASE_W = 0.025
LATCH_BASE_H = 0.050
LATCH_BASE_T = 0.008

# Roller blocks
ROLLER_W = 0.022
ROLLER_H = 0.014
ROLLER_D = 0.018

# ---------------------------------------------------------------------------
# Derived layout
# ---------------------------------------------------------------------------

HALF_W = TOTAL_W / 2.0
INNER_X0 = -HALF_W + FRAME_FACE
INNER_X1 = HALF_W - FRAME_FACE
INNER_Z0 = FRAME_FACE
INNER_Z1 = TOTAL_H - FRAME_FACE
INNER_H = INNER_Z1 - INNER_Z0
SASH_OPENING_H = INNER_H
MID_CZ = (INNER_Z0 + INNER_Z1) / 2.0

# Opening X ranges
LEFT_X0 = INNER_X0
LEFT_X1 = INNER_X0 + LEFT_OPEN_W
MULL1_X0 = LEFT_X1
MULL1_X1 = MULL1_X0 + MULLION_W
CENTER_X0 = MULL1_X1
CENTER_X1 = CENTER_X0 + CENTER_OPEN_W
MULL2_X0 = CENTER_X1
MULL2_X1 = MULL2_X0 + MULLION_W
RIGHT_X0 = MULL2_X1
RIGHT_X1 = INNER_X1

# Opening centers
LEFT_CX = (LEFT_X0 + LEFT_X1) / 2.0
CENTER_CX = (CENTER_X0 + CENTER_X1) / 2.0
RIGHT_CX = (RIGHT_X0 + RIGHT_X1) / 2.0

# Sash outer dimensions
def _sash_outer_w(opening_w: float) -> float:
    return opening_w + 2 * SASH_FACE

SASH_OUTER_H = SASH_OPENING_H + 2 * SASH_FACE

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

VINYL_RGBA = (0.94, 0.95, 0.96, 1.0)
GLASS_RGBA = (0.52, 0.60, 0.66, 0.30)
METAL_RGBA = (0.74, 0.76, 0.79, 1.0)
DARK_RGBA = (0.18, 0.18, 0.20, 1.0)

# ---------------------------------------------------------------------------
# Geometry helpers (CadQuery, meters)
# ---------------------------------------------------------------------------

def _slab(x0: float, x1: float, z0: float, z1: float,
          y_center: float, depth: float) -> cq.Workplane:
    """Axis-aligned box in X-Z plane centered on y_center."""
    return (
        cq.Workplane("XY")
        .transformed(offset=((x0 + x1) / 2.0, y_center, (z0 + z1) / 2.0))
        .box(x1 - x0, depth, z1 - z0)
    )


def _build_frame_shape() -> cq.Workplane:
    """Outer frame: thick slab with three sash openings (two mullion bars remain),
    plus a sill lip and drainage slots."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)
    cut_depth = FRAME_DEPTH + 0.02

    # Three rectangular openings
    left_cut = _slab(LEFT_X0, LEFT_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    center_cut = _slab(CENTER_X0, CENTER_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    right_cut = _slab(RIGHT_X0, RIGHT_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    frame = outer.cut(left_cut).cut(center_cut).cut(right_cut)

    # Sill lip: thin plate extending forward from bottom front of frame
    lip_y = FRAME_DEPTH / 2.0 + SILL_LIP_EXTEND / 2.0 - 0.005
    lip_dy = SILL_LIP_EXTEND + 0.01
    sill_lip = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, lip_y, SILL_LIP_T / 2.0))
        .box(TOTAL_W, lip_dy, SILL_LIP_T)
    )
    frame = frame.union(sill_lip)

    # Drainage slots: small rectangular cuts through the sill front face
    slot_z = FRAME_FACE * 0.35
    slot_y_depth = 0.05
    spacing = TOTAL_W / (DRAIN_COUNT + 1)
    for i in range(DRAIN_COUNT):
        sx = -HALF_W + spacing * (i + 1)
        slot = (
            cq.Workplane("XY")
            .transformed(offset=(sx, FRAME_DEPTH / 2.0, slot_z))
            .box(DRAIN_W, slot_y_depth, DRAIN_H)
        )
        frame = frame.cut(slot)

    return frame


def _build_sash_shape(opening_w: float, face: float = SASH_FACE,
                     depth: float = SASH_DEPTH) -> cq.Workplane:
    """Hollow sash ring in its own local frame, centered at origin."""
    ow = opening_w
    oh = SASH_OPENING_H
    out_w = ow + 2 * face
    out_h = oh + 2 * face
    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0,
                  0.0, depth)
    opening = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0,
                    0.0, depth + 0.02)
    return outer.cut(opening)


def _build_sash_glass(opening_w: float) -> cq.Workplane:
    """Clear pane filling the sash opening (local frame), rebated under lip."""
    ow = opening_w + 2 * REBATE
    oh = SASH_OPENING_H + 2 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_panel_sliding_window")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("metal", rgba=METAL_RGBA)
    model.material("dark", rgba=DARK_RGBA)

    # --- Frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame_shell"),
        material="vinyl",
        name="frame_shell",
    )

    # --- Left fixed sash ---
    left_fixed = model.part("left_fixed")
    left_fixed.visual(
        mesh_from_cadquery(
            _build_sash_shape(LEFT_OPEN_W, FIXED_SASH_FACE, FIXED_SASH_DEPTH),
            "left_fixed_vinyl"),
        material="vinyl",
        name="left_fixed_vinyl",
    )
    left_fixed.visual(
        mesh_from_cadquery(_build_sash_glass(LEFT_OPEN_W), "left_fixed_glass"),
        material="glass",
        name="left_fixed_glass",
    )

    # --- Center fixed sash (wider) ---
    center_fixed = model.part("center_fixed")
    center_fixed.visual(
        mesh_from_cadquery(
            _build_sash_shape(CENTER_OPEN_W, FIXED_SASH_FACE, FIXED_SASH_DEPTH),
            "center_fixed_vinyl"),
        material="vinyl",
        name="center_fixed_vinyl",
    )
    center_fixed.visual(
        mesh_from_cadquery(_build_sash_glass(CENTER_OPEN_W), "center_fixed_glass"),
        material="glass",
        name="center_fixed_glass",
    )

    # --- Right sliding sash ---
    right_slider = model.part("right_slider")
    right_slider.visual(
        mesh_from_cadquery(_build_sash_shape(RIGHT_OPEN_W), "right_slider_vinyl"),
        material="vinyl",
        name="right_slider_vinyl",
    )
    right_slider.visual(
        mesh_from_cadquery(_build_sash_glass(RIGHT_OPEN_W), "right_slider_glass"),
        material="glass",
        name="right_slider_glass",
    )

    # Roller blocks: two small blocks at the bottom of the sliding sash
    roller_z = -(SASH_OUTER_H / 2.0 - ROLLER_H / 2.0)
    roller_lx = -RIGHT_OPEN_W / 2.0 + ROLLER_W / 2.0 + 0.015
    roller_rx = RIGHT_OPEN_W / 2.0 - ROLLER_W / 2.0 - 0.015
    right_slider.visual(
        Box((ROLLER_W, ROLLER_D, ROLLER_H)),
        origin=Origin(xyz=(roller_lx, 0.0, roller_z)),
        material="dark",
        name="roller_left",
    )
    right_slider.visual(
        Box((ROLLER_W, ROLLER_D, ROLLER_H)),
        origin=Origin(xyz=(roller_rx, 0.0, roller_z)),
        material="dark",
        name="roller_right",
    )

    # Keeper plate on the meeting stile (left stile of right slider)
    stile_x = -RIGHT_OPEN_W / 2.0 - SASH_FACE / 2.0
    face_y = SASH_DEPTH / 2.0
    plate_y = face_y + LATCH_BASE_T / 2.0
    right_slider.visual(
        Box((LATCH_BASE_W, LATCH_BASE_T, LATCH_BASE_H)),
        origin=Origin(xyz=(stile_x, plate_y, 0.0)),
        material="metal",
        name="keeper_plate",
    )

    # --- Latch (separate part, revolute) ---
    latch = model.part("latch")
    # Lever arm: cylinder along +Z from pivot (default cylinder axis is +Z)
    latch.visual(
        Cylinder(radius=LATCH_LEVER_R, length=LATCH_LEVER_LEN),
        origin=Origin(xyz=(0.0, 0.0, LATCH_LEVER_LEN / 2.0)),
        material="metal",
        name="latch_lever",
    )
    # Small pivot boss at the base
    latch.visual(
        Cylinder(radius=0.009, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material="metal",
        name="latch_boss",
    )

    # --- Articulations ---

    # Fixed left sash
    model.articulation(
        "frame_to_left_fixed",
        ArticulationType.FIXED,
        parent="frame",
        child="left_fixed",
        origin=Origin(xyz=(LEFT_CX, FIXED_SASH_Y, MID_CZ)),
    )

    # Fixed center sash
    model.articulation(
        "frame_to_center_fixed",
        ArticulationType.FIXED,
        parent="frame",
        child="center_fixed",
        origin=Origin(xyz=(CENTER_CX, FIXED_SASH_Y, MID_CZ)),
    )

    # Sliding right sash: prismatic along -X
    slide_travel = RIGHT_OPEN_W * 0.85
    model.articulation(
        "frame_to_right_slider",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="right_slider",
        origin=Origin(xyz=(RIGHT_CX, SLIDE_SASH_Y, MID_CZ)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.5,
                                   lower=0.0, upper=slide_travel),
    )

    # Latch: revolute, parented to right_slider at the meeting rail
    # Pivot is on the meeting stile front face, mid-height
    latch_pivot_y = face_y + LATCH_BASE_T + 0.003
    model.articulation(
        "slider_to_latch",
        ArticulationType.REVOLUTE,
        parent="right_slider",
        child="latch",
        origin=Origin(xyz=(stile_x, latch_pivot_y, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0,
                                   lower=0.0, upper=1.5708),
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
    right_slider = object_model.get_part("right_slider")
    latch = object_model.get_part("latch")

    slide = object_model.get_articulation("frame_to_right_slider")
    latch_joint = object_model.get_articulation("slider_to_latch")

    # --- Intentional overlaps ---
    # Glass rebated under sash lips
    for nm in ("left_fixed", "center_fixed", "right_slider"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass", elem_b=f"{nm}_vinyl",
            reason="Glass pane rebated under sash lip (captured glazing).",
        )
    # Sashes seated in frame tracks
    for nm in ("left_fixed", "center_fixed", "right_slider"):
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell", elem_b=f"{nm}_vinyl",
            reason=f"{nm} ring seated in frame track (rebate capture).",
        )
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell", elem_b=f"{nm}_glass",
            reason=f"{nm} glass captured in frame opening.",
        )
    # Keeper plate mounted on meeting stile
    ctx.allow_overlap(
        "right_slider", "right_slider",
        elem_a="keeper_plate", elem_b="right_slider_vinyl",
        reason="Keeper plate mounted on meeting stile face.",
    )
    # Latch hardware at pivot
    ctx.allow_overlap(
        "right_slider", "latch",
        elem_a="keeper_plate", elem_b="latch_boss",
        reason="Latch pivot boss seated against keeper plate (mounted hardware).",
    )
    ctx.allow_overlap(
        "right_slider", "latch",
        elem_a="keeper_plate", elem_b="latch_lever",
        reason="Latch lever passes through keeper plate pivot region.",
    )
    # Roller blocks in sill track
    for rn in ("roller_left", "roller_right"):
        ctx.allow_overlap(
            "right_slider", "frame",
            elem_a=rn, elem_b="frame_shell",
            reason=f"{rn} sits in the sill track (roller in track).",
        )

    # --- Three-panel structure ---
    center_aabb = ctx.part_world_aabb(center_fixed)
    left_aabb = ctx.part_world_aabb(left_fixed)
    right_aabb = ctx.part_world_aabb(right_slider)

    center_w = center_aabb[1][0] - center_aabb[0][0]
    left_w = left_aabb[1][0] - left_aabb[0][0]
    right_w = right_aabb[1][0] - right_aabb[0][0]

    ctx.check(
        "center panel wider than left panel",
        center_w > left_w + 0.10,
        details=f"center_w={center_w:.3f}, left_w={left_w:.3f}",
    )
    ctx.check(
        "right slider similar opening to left fixed",
        abs(right_w - left_w) < 2 * (SASH_FACE - FIXED_SASH_FACE) + 0.02,
        details=f"right_w={right_w:.3f}, left_w={left_w:.3f}",
    )

    # Panel ordering: left < center < right in X
    left_cx = (left_aabb[0][0] + left_aabb[1][0]) / 2.0
    center_cx = (center_aabb[0][0] + center_aabb[1][0]) / 2.0
    right_cx = (right_aabb[0][0] + right_aabb[1][0]) / 2.0
    ctx.check(
        "panels ordered left < center < right",
        left_cx < center_cx < right_cx,
        details=f"left={left_cx:.3f}, center={center_cx:.3f}, right={right_cx:.3f}",
    )

    # --- Sill lip: frame extends further in +Y at bottom than at top ---
    frame_aabb = ctx.part_world_aabb(frame)
    frame_y_extent = frame_aabb[1][1] - frame_aabb[0][1]
    ctx.check(
        "frame has sill lip protruding forward",
        frame_y_extent > FRAME_DEPTH + SILL_LIP_EXTEND * 0.4,
        details=f"frame_y={frame_y_extent:.3f}, expected>{FRAME_DEPTH + SILL_LIP_EXTEND * 0.4:.3f}",
    )

    # --- Roller blocks at bottom of right slider ---
    roller_l_aabb = ctx.part_element_world_aabb(right_slider, elem="roller_left")
    roller_r_aabb = ctx.part_element_world_aabb(right_slider, elem="roller_right")
    slider_bottom = right_aabb[0][2]

    for rn, raabb in (("left", roller_l_aabb), ("right", roller_r_aabb)):
        rz = (raabb[0][2] + raabb[1][2]) / 2.0
        ctx.check(
            f"roller_{rn} near bottom of slider",
            rz < slider_bottom + 0.06,
            details=f"roller_z={rz:.3f}, slider_bottom={slider_bottom:.3f}",
        )

    # --- Latch revolute joint ---
    ctx.check(
        "latch joint is revolute",
        latch_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={latch_joint.articulation_type}",
    )

    # Latch lever rotates visibly about Y axis
    with ctx.pose({latch_joint: 0.0}):
        lever_locked = ctx.part_element_world_aabb(latch, elem="latch_lever")
    with ctx.pose({latch_joint: 1.2}):
        lever_unlocked = ctx.part_element_world_aabb(latch, elem="latch_lever")

    locked_x_span = lever_locked[1][0] - lever_locked[0][0]
    unlocked_x_span = lever_unlocked[1][0] - lever_unlocked[0][0]
    ctx.check(
        "latch lever X-span changes on rotation",
        abs(locked_x_span - unlocked_x_span) > 0.005,
        details=f"locked_span={locked_x_span:.4f}, unlocked_span={unlocked_x_span:.4f}",
    )

    # --- Slide joint: prismatic, moves slider toward center ---
    ctx.check(
        "slide joint is prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={slide.articulation_type}",
    )

    with ctx.pose({slide: 0.0}):
        rest_pos = ctx.part_world_aabb(right_slider)
        rest_cx = (rest_pos[0][0] + rest_pos[1][0]) / 2.0

    travel = slide.motion_limits.upper
    with ctx.pose({slide: travel}):
        open_pos = ctx.part_world_aabb(right_slider)
        open_cx = (open_pos[0][0] + open_pos[1][0]) / 2.0

    ctx.check(
        "slider opens toward center (-X)",
        open_cx < rest_cx - 0.10,
        details=f"rest_x={rest_cx:.3f}, open_x={open_cx:.3f}",
    )

    # Retained: slider stays within frame X span at full travel
    frame_x = ctx.part_world_aabb(frame)
    ctx.check(
        "slider retained within frame X at full travel",
        open_pos[0][0] > frame_x[0][0] - 0.01 and open_pos[1][0] < frame_x[1][0] + 0.01,
        details=f"slider x=[{open_pos[0][0]:.3f},{open_pos[1][0]:.3f}], "
                f"frame x=[{frame_x[0][0]:.3f},{frame_x[1][0]:.3f}]",
    )

    # Non-fixed joints
    non_fixed = [
        a for a in object_model.articulations
        if a.articulation_type != ArticulationType.FIXED
    ]
    ctx.check(
        "at least one non-fixed joint",
        len(non_fixed) >= 1,
        details=f"non_fixed_count={len(non_fixed)}",
    )

    return ctx.report()


object_model = build_object_model()
