from __future__ import annotations

# Three-panel horizontal sliding window (variant 12), white vinyl frame.
# Left FIXED sash (narrow), wider center FIXED sash, right SLIDING sash (narrow).
# A tilt-in latch pair on the sliding sash meeting stile pivots on small revolute
# joints. Two tiny roller blocks at the bottom of the sliding sash ride on the
# sill track. A small metal cam-latch keeper plate remains as a visual on the
# sliding sash meeting stile.
#
# Coordinate convention:
#   +Z is up. Window stands vertically.
#     width  -> X,  height -> Z (sill near z=0),  frame depth -> Y
#   Glass plane is the X-Z plane. q=0 reads SHUT.
#   Driving the prismatic joint slides the right sash toward -X to open,
#   retained in the head/sill track.
#
# Y layout: left + center fixed sashes in the rear track (-Y);
#           sliding sash proud (+Y) so it passes in front of the center sash.

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
TOTAL_H = 1.72            # overall height along Z (sill at z=0)

FRAME_FACE = 0.085        # outer frame member face width (chunky vinyl)
FRAME_DEPTH = 0.140       # deep box section along Y

SASH_FACE = 0.055         # sash rail/stile face width
SASH_DEPTH = 0.060        # sash depth along Y
GLASS_T = 0.008           # glazing thickness
REBATE = 0.005            # glass tucks under the sash lip

# Three-panel layout: glass opening widths (the clear pane size per sash)
LEFT_SASH_OPENING_W = 0.25     # left fixed sash glass width
CENTER_SASH_OPENING_W = 0.54   # center fixed sash glass width (~2x wider)
RIGHT_SASH_OPENING_W = 0.25    # right sliding sash glass width

SASH_OPENING_H = 0.0           # set below from INNER_H

# Y layout: rear (fixed) and front (sliding) tracks
FIXED_SASH_Y = -0.028     # rear glazing plane center
SLIDE_SASH_Y = 0.044      # front track center (proud toward +Y)

# Cam latch keeper plate (non-articulated visual on sliding sash)
LATCH_PLATE_W = 0.028
LATCH_PLATE_H = 0.065
LATCH_PLATE_T = 0.010

# Tilt-in latch tabs (articulated parts)
TILT_LATCH_W = 0.012      # tab width (X)
TILT_LATCH_T = 0.006      # tab thickness (Y)
TILT_LATCH_H = 0.040      # tab length (Z)

# Roller blocks (visuals on the sliding sash, move with it)
ROLLER_W = 0.025          # roller width (X)
ROLLER_D = 0.030          # roller depth (Y)
ROLLER_H = 0.010          # roller height (Z)

# Materials
VINYL_RGBA = (0.94, 0.95, 0.96, 1.0)
GLASS_RGBA = (0.52, 0.60, 0.66, 0.30)
METAL_RGBA = (0.74, 0.76, 0.79, 1.0)
NYLON_RGBA = (0.22, 0.22, 0.24, 1.0)   # dark nylon roller blocks

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
SASH_OPENING_H = INNER_H
MID_CZ = (INNER_Z0 + INNER_Z1) / 2.0

# Sash outer widths (glass opening + 2*SASH_FACE)
LEFT_SASH_OUTER_W = LEFT_SASH_OPENING_W + 2 * SASH_FACE
CENTER_SASH_OUTER_W = CENTER_SASH_OPENING_W + 2 * SASH_FACE
RIGHT_SASH_OUTER_W = RIGHT_SASH_OPENING_W + 2 * SASH_FACE

# Sash center X positions (world frame):
# Left sash starts at the inner jamb; center is at window center;
# right sash ends near the right jamb.
LEFT_SASH_CX = INNER_X0 + LEFT_SASH_OUTER_W / 2.0
CENTER_SASH_CX = 0.0
RIGHT_SASH_CX = INNER_X1 - RIGHT_SASH_OUTER_W / 2.0

# Slide travel: the right sash slides left, passing in front of the center sash.
slide_travel = 0.35


# ---------------------------------------------------------------------------
# Geometry helpers (CadQuery)
# ---------------------------------------------------------------------------

def _slab(x0, x1, z0, z1, y_center, depth):
    """Axis-aligned box spanning [x0,x1] x [z0,z1], centered on y_center."""
    return (
        cq.Workplane("XY")
        .transformed(offset=((x0 + x1) / 2.0, y_center, (z0 + z1) / 2.0))
        .box(x1 - x0, depth, z1 - z0)
    )


def _build_frame_shape():
    """Static outer frame: thick slab with one large rectangular opening."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)
    cut_depth = FRAME_DEPTH + 0.02
    opening = _slab(INNER_X0, INNER_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    return outer.cut(opening)


def _build_sash_shape(opening_w):
    """Hollow sash ring for the given glass opening width."""
    ow = opening_w
    oh = SASH_OPENING_H
    out_w = ow + 2 * SASH_FACE
    out_h = oh + 2 * SASH_FACE
    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, SASH_DEPTH)
    opening = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SASH_DEPTH + 0.02)
    return outer.cut(opening)


def _build_glass_shape(opening_w):
    """Clear pane for the given glass opening width, rebated under sash lip."""
    ow = opening_w + 2 * REBATE
    oh = SASH_OPENING_H + 2 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


# ---------------------------------------------------------------------------
# Part builders
# ---------------------------------------------------------------------------

def _add_sash(model, name, opening_w):
    """Add a sash part (vinyl ring + glass) centered at its own local origin."""
    sash = model.part(name)
    sash.visual(
        mesh_from_cadquery(_build_sash_shape(opening_w), f"{name}_vinyl"),
        material="vinyl",
        name=f"{name}_vinyl",
    )
    sash.visual(
        mesh_from_cadquery(_build_glass_shape(opening_w), f"{name}_glass"),
        material="glass",
        name=f"{name}_glass",
    )
    return sash


def _add_cam_latch_visuals(sash, sash_name, opening_w):
    """Non-articulated cam latch keeper plate on the sliding sash meeting stile."""
    stile_x = -opening_w / 2.0 - SASH_FACE / 2.0
    face_y = SASH_DEPTH / 2.0
    plate_y = face_y + LATCH_PLATE_T / 2.0
    sash.visual(
        Box((LATCH_PLATE_W, LATCH_PLATE_T, LATCH_PLATE_H)),
        origin=Origin(xyz=(stile_x, plate_y, 0.0)),
        material="metal",
        name=f"{sash_name}_latch_plate",
    )


def _add_roller_blocks(sash, sash_name, outer_w):
    """Two small nylon roller blocks at the bottom rail of the sliding sash."""
    roller_z = -(SASH_OPENING_H / 2.0 + SASH_FACE + ROLLER_H / 2.0)
    left_x = -(outer_w / 2.0 - ROLLER_W / 2.0)
    right_x = +(outer_w / 2.0 - ROLLER_W / 2.0)
    sash.visual(
        Box((ROLLER_W, ROLLER_D, ROLLER_H)),
        origin=Origin(xyz=(left_x, 0.0, roller_z)),
        material="nylon",
        name=f"{sash_name}_roller_0",
    )
    sash.visual(
        Box((ROLLER_W, ROLLER_D, ROLLER_H)),
        origin=Origin(xyz=(right_x, 0.0, roller_z)),
        material="nylon",
        name=f"{sash_name}_roller_1",
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_panel_sliding_window")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("metal", rgba=METAL_RGBA)
    model.material("nylon", rgba=NYLON_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material="vinyl",
        name="frame_shell",
    )

    # --- Three sashes ---
    _add_sash(model, "fixed_sash_0", LEFT_SASH_OPENING_W)
    _add_sash(model, "center_sash", CENTER_SASH_OPENING_W)
    sliding_sash = _add_sash(model, "sliding_sash", RIGHT_SASH_OPENING_W)

    # Cam latch + rollers on sliding sash
    _add_cam_latch_visuals(sliding_sash, "sliding_sash", RIGHT_SASH_OPENING_W)
    _add_roller_blocks(sliding_sash, "sliding_sash", RIGHT_SASH_OUTER_W)

    # --- Tilt-in latch pair (articulated parts on the sliding sash) ---
    stile_x = -(RIGHT_SASH_OPENING_W / 2.0 + SASH_FACE / 2.0)
    face_y = SASH_DEPTH / 2.0
    upper_z = SASH_OPENING_H / 2.0 - 0.12
    lower_z = -(SASH_OPENING_H / 2.0 - 0.12)

    for idx, lz in enumerate((upper_z, lower_z)):
        latch = model.part(f"tilt_latch_{idx}")
        # Tab extends along +Z from pivot origin, slightly proud of sash face
        latch.visual(
            Box((TILT_LATCH_W, TILT_LATCH_T, TILT_LATCH_H)),
            origin=Origin(xyz=(0.0, TILT_LATCH_T / 2.0, TILT_LATCH_H / 2.0)),
            material="metal",
            name=f"tilt_latch_{idx}_body",
        )

    # --- Articulations ---

    # Fixed left sash: seated in rear glazing plane
    model.articulation(
        "frame_to_fixed_sash",
        ArticulationType.FIXED,
        parent="frame",
        child="fixed_sash_0",
        origin=Origin(xyz=(LEFT_SASH_CX, FIXED_SASH_Y, MID_CZ)),
    )

    # Fixed center sash: seated in rear glazing plane
    model.articulation(
        "frame_to_center_sash",
        ArticulationType.FIXED,
        parent="frame",
        child="center_sash",
        origin=Origin(xyz=(CENTER_SASH_CX, FIXED_SASH_Y, MID_CZ)),
    )

    # Sliding right sash: PRISMATIC along X.
    # axis=(-1,0,0) so positive q slides left (opens).
    model.articulation(
        "frame_to_sliding_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="sliding_sash",
        origin=Origin(xyz=(RIGHT_SASH_CX, SLIDE_SASH_Y, MID_CZ)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.5, lower=0.0, upper=slide_travel),
    )

    # Tilt-in latch pair: REVOLUTE pivots on the sliding sash meeting stile.
    # Axis (0,1,0): rotation in the sash X-Z plane (tab flips outward from stile).
    # Positive q rotates the tab from vertical (locked) toward horizontal (unlocked).
    for idx, lz in enumerate((upper_z, lower_z)):
        model.articulation(
            f"sash_to_latch_{idx}",
            ArticulationType.REVOLUTE,
            parent="sliding_sash",
            child=f"tilt_latch_{idx}",
            origin=Origin(xyz=(stile_x, face_y, lz)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=1.2),
        )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    fixed_sash_0 = object_model.get_part("fixed_sash_0")
    center_sash = object_model.get_part("center_sash")
    sliding_sash = object_model.get_part("sliding_sash")
    tilt_latch_0 = object_model.get_part("tilt_latch_0")
    tilt_latch_1 = object_model.get_part("tilt_latch_1")

    slide = object_model.get_articulation("frame_to_sliding_sash")
    latch_joint_0 = object_model.get_articulation("sash_to_latch_0")
    latch_joint_1 = object_model.get_articulation("sash_to_latch_1")

    # --- Intentional overlaps ---
    # Glass rebated under sash lips (captured glazing)
    for nm, ow in (("fixed_sash_0", LEFT_SASH_OPENING_W),
                   ("center_sash", CENTER_SASH_OPENING_W),
                   ("sliding_sash", RIGHT_SASH_OPENING_W)):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass",
            elem_b=f"{nm}_vinyl",
            reason=f"Clear pane rebated under {nm} sash lip (captured, not floating).",
        )
    # Sashes rebated into the frame opening (seated capture)
    for nm in ("fixed_sash_0", "center_sash", "sliding_sash"):
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_vinyl",
            reason=f"{nm} ring rebated into the frame head/sill track.",
        )
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_glass",
            reason=f"{nm} glass rebated under the frame opening lip.",
        )
    # Left and center fixed sashes are in the same rear track and their meeting
    # stiles interlock (small intentional overlap at the meeting stile).
    ctx.allow_overlap(
        "fixed_sash_0", "center_sash",
        elem_a="fixed_sash_0_vinyl",
        elem_b="center_sash_vinyl",
        reason="Meeting stile interlock between adjacent rear-track fixed sashes.",
    )
    # Cam latch plate seated on sliding sash stile face
    ctx.allow_overlap(
        "sliding_sash", "sliding_sash",
        elem_a="sliding_sash_latch_plate",
        elem_b="sliding_sash_vinyl",
        reason="Latch keeper plate seated on sliding-sash meeting stile.",
    )
    # Tilt latch tabs seated on the sash stile face (pivot mount)
    for idx in range(2):
        ctx.allow_overlap(
            "sliding_sash", f"tilt_latch_{idx}",
            elem_a="sliding_sash_vinyl",
            elem_b=f"tilt_latch_{idx}_body",
            reason=f"Tilt latch {idx} pivot-mounted on the sliding sash meeting stile face.",
        )
    # Roller blocks seated at the sash bottom rail
    for idx in range(2):
        ctx.allow_overlap(
            "sliding_sash", "sliding_sash",
            elem_a=f"sliding_sash_roller_{idx}",
            elem_b="sliding_sash_vinyl",
            reason=f"Roller block {idx} seated at the sliding sash bottom rail.",
        )

    # --- Closed pose (q=0): window reads SHUT ---
    with ctx.pose({slide: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        f0_aabb = ctx.part_world_aabb(fixed_sash_0)
        c_aabb = ctx.part_world_aabb(center_sash)
        s_aabb = ctx.part_world_aabb(sliding_sash)

        # Frame spans full width and height
        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        ctx.check(
            "frame spans full window width",
            frame_w > 1.30,
            details=f"frame_w={frame_w:.3f}",
        )
        ctx.check(
            "sill near z=0",
            abs(frame_aabb[0][2]) < 0.02,
            details=f"frame zmin={frame_aabb[0][2]:.4f}",
        )
        ctx.check(
            "head near full height",
            abs(frame_aabb[1][2] - TOTAL_H) < 0.02,
            details=f"frame zmax={frame_aabb[1][2]:.4f}",
        )

        # Three sashes left-to-right: fixed_0, center, sliding
        f0_x = (f0_aabb[0][0] + f0_aabb[1][0]) / 2.0
        c_x = (c_aabb[0][0] + c_aabb[1][0]) / 2.0
        s_x = (s_aabb[0][0] + s_aabb[1][0]) / 2.0
        ctx.check(
            "fixed_sash_0 left of center_sash",
            f0_x < c_x,
            details=f"fixed_x={f0_x:.3f}, center_x={c_x:.3f}",
        )
        ctx.check(
            "center_sash left of sliding_sash",
            c_x < s_x,
            details=f"center_x={c_x:.3f}, sliding_x={s_x:.3f}",
        )

        # Center sash is wider than each side sash
        f0_w = f0_aabb[1][0] - f0_aabb[0][0]
        c_w = c_aabb[1][0] - c_aabb[0][0]
        s_w = s_aabb[1][0] - s_aabb[0][0]
        ctx.check(
            "center sash wider than left sash",
            c_w > f0_w + 0.10,
            details=f"center_w={c_w:.3f}, left_w={f0_w:.3f}",
        )
        ctx.check(
            "center sash wider than right sash",
            c_w > s_w + 0.10,
            details=f"center_w={c_w:.3f}, right_w={s_w:.3f}",
        )

        # Sliding sash proud of fixed sashes in Y
        f0_y = (f0_aabb[0][1] + f0_aabb[1][1]) / 2.0
        s_y = (s_aabb[0][1] + s_aabb[1][1]) / 2.0
        ctx.check(
            "sliding sash proud of fixed sashes",
            s_y > f0_y + 0.02,
            details=f"sliding_y={s_y:.3f}, fixed_y={f0_y:.3f}",
        )

        # All sashes seated within frame height
        for nm, ab in (("fixed_sash_0", f0_aabb), ("center_sash", c_aabb), ("sliding_sash", s_aabb)):
            ctx.check(
                f"{nm} within frame height",
                ab[0][2] > frame_aabb[0][2] - 1e-4 and ab[1][2] < frame_aabb[1][2] + 1e-4,
                details=f"{nm} z=[{ab[0][2]:.3f},{ab[1][2]:.3f}]",
            )

        # Roller blocks at the bottom of the sliding sash
        for idx in range(2):
            r_aabb = ctx.part_element_world_aabb(sliding_sash, elem=f"sliding_sash_roller_{idx}")
            ctx.check(
                f"roller_{idx} near sash bottom",
                r_aabb[1][2] < s_aabb[0][2] + 0.02,
                details=f"roller_top_z={r_aabb[1][2]:.4f}, sash_bottom_z={s_aabb[0][2]:.4f}",
            )

        rest_sx = s_x
        rest_sz = (s_aabb[0][2] + s_aabb[1][2]) / 2.0

    # --- Open pose: sliding sash slides left (-X) ---
    travel = slide.motion_limits.upper
    with ctx.pose({slide: travel}):
        s_open = ctx.part_world_aabb(sliding_sash)
        open_sx = (s_open[0][0] + s_open[1][0]) / 2.0
        ctx.check(
            "sliding sash opens toward -X",
            open_sx < rest_sx - 0.15,
            details=f"rest_x={rest_sx:.3f}, open_x={open_sx:.3f}",
        )
        # Pure horizontal slide
        open_sz = (s_open[0][2] + s_open[1][2]) / 2.0
        ctx.check(
            "slide is purely horizontal",
            abs(open_sz - rest_sz) < 0.02,
            details=f"open_z={open_sz:.3f}, rest_z={rest_sz:.3f}",
        )
        # Retained within frame
        f_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "sash retained within frame at full travel",
            s_open[0][0] > f_aabb[0][0] - 1e-4 and s_open[1][0] < f_aabb[1][0] + 1e-4,
            details=f"sash x=[{s_open[0][0]:.3f},{s_open[1][0]:.3f}]",
        )

    # --- Tilt latch pivot check ---
    # At q=0 latches are vertical (locked). At upper limit they flip outward.
    latch_upper = latch_joint_0.motion_limits.upper
    with ctx.pose({latch_joint_0: 0.0}):
        l0_closed = ctx.part_world_aabb(tilt_latch_0)
    with ctx.pose({latch_joint_0: latch_upper}):
        l0_open = ctx.part_world_aabb(tilt_latch_0)
    # The latch AABB should change when pivoted (tab moves from vertical to angled)
    closed_span_z = l0_closed[1][2] - l0_closed[0][2]
    open_span_z = l0_open[1][2] - l0_open[0][2]
    ctx.check(
        "tilt_latch_0 pivots (AABB changes)",
        abs(closed_span_z - open_span_z) > 0.002,
        details=f"closed_z_span={closed_span_z:.4f}, open_z_span={open_span_z:.4f}",
    )

    # Verify both latch joints are revolute with valid limits
    for jnt in (latch_joint_0, latch_joint_1):
        ctx.check(
            f"{jnt.name} is revolute",
            jnt.articulation_type == ArticulationType.REVOLUTE,
            details=f"type={jnt.articulation_type}",
        )
        ctx.check(
            f"{jnt.name} has positive range",
            jnt.motion_limits.upper > jnt.motion_limits.lower,
            details=f"lower={jnt.motion_limits.lower}, upper={jnt.motion_limits.upper}",
        )

    # Verify slide joint is prismatic
    ctx.check(
        "slide joint is prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={slide.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
