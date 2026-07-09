from __future__ import annotations

# Corner-lift sliding window variant: three-panel horizontal sliding window
# with white vinyl frame and colonial divided-lite grilles.
#
# Changes from parent:
#   - Small vent panel at upper-left corner with REVOLUTE tilt joint
#   - Two roller blocks at the bottom of the sliding sash
#   - Center sash still slides left-right on PRISMATIC joint
#
# Coordinate convention:
#   +Z is up. Window stands vertically.
#     - width  -> X
#     - height -> Z   (sill near z=0)
#     - frame depth / glazing thickness / slide-normal -> Y
#   The glass plane is the X-Z plane.

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
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

FRAME_FACE = 0.070
MULLION_FACE = 0.060
FRAME_DEPTH = 0.110

SIDE_LITE_W = 0.85
CENTER_LITE_W = 1.04

SASH_FACE = 0.055
SASH_DEPTH = 0.055
GLASS_T = 0.008

GRILLE_COLS = 4
GRILLE_ROWS = 5
MUNTIN_T = 0.020
MUNTIN_DEPTH = 0.020

FIXED_LITE_Y = -0.020
SLIDE_SASH_Y = 0.052

REBATE = 0.005

# Vent panel dimensions
VENT_W = 0.28
VENT_H = 0.22
VENT_DEPTH = 0.035
VENT_FRAME_FACE = 0.030

# Roller block dimensions
ROLLER_W = 0.040
ROLLER_D = 0.025
ROLLER_H = 0.018

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

# Vent panel position: upper-left corner of the window, on the front face
VENT_CX = LEFT_X0 + VENT_W / 2.0 + 0.06  # offset from left jamb
VENT_Y = FRAME_DEPTH / 2.0 + VENT_DEPTH / 2.0  # back face touches frame front
# Hinge at the bottom edge; panel extends upward to touch the head member bottom.
VENT_HINGE_Z = INNER_Z1 - VENT_H  # top of panel lands at INNER_Z1 (head bottom)

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

VINYL_RGBA = (0.94, 0.95, 0.96, 1.0)
GLASS_RGBA = (0.50, 0.58, 0.64, 0.32)
ROLLER_RGBA = (0.25, 0.25, 0.27, 1.0)  # dark grey nylon/delrin rollers


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _slab(x0, x1, z0, z1, y_center, depth):
    w = x1 - x0
    h = z1 - z0
    cx = (x0 + x1) / 2.0
    cz = (z0 + z1) / 2.0
    return (
        cq.Workplane("XY")
        .transformed(offset=(cx, y_center, cz))
        .box(w, depth, h)
    )


def _build_frame_shape():
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)
    cut_depth = FRAME_DEPTH + 0.02
    left_cut = _slab(LEFT_X0, LEFT_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    center_cut = _slab(CENTER_X0, CENTER_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    right_cut = _slab(RIGHT_X0, RIGHT_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    return outer.cut(left_cut).cut(center_cut).cut(right_cut)


def _build_sash_grille_shape(opening_w, opening_h):
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


def _build_sash_glass_shape(opening_w, opening_h):
    ow = opening_w + 2 * REBATE
    oh = opening_h + 2 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


def _build_vent_panel_shape():
    """Small vent panel: vinyl frame ring with glass pane, in its own local frame.
    Local origin at the bottom-center of the panel (hinge line)."""
    out_w = VENT_W
    out_h = VENT_H
    # Panel slab extends upward from local z=0
    outer = _slab(-out_w / 2.0, out_w / 2.0, 0.0, out_h, 0.0, VENT_DEPTH)
    # Cut the glass opening (inset by frame face)
    glass_w = out_w - 2 * VENT_FRAME_FACE
    glass_h = out_h - 2 * VENT_FRAME_FACE
    opening = _slab(
        -glass_w / 2.0, glass_w / 2.0,
        VENT_FRAME_FACE, VENT_FRAME_FACE + glass_h,
        0.0, VENT_DEPTH + 0.02,
    )
    return outer.cut(opening)


def _build_vent_glass_shape():
    """Glass pane for the vent panel."""
    glass_w = VENT_W - 2 * VENT_FRAME_FACE + 2 * REBATE
    glass_h = VENT_H - 2 * VENT_FRAME_FACE + 2 * REBATE
    return _slab(
        -glass_w / 2.0, glass_w / 2.0,
        VENT_FRAME_FACE - REBATE, VENT_FRAME_FACE + glass_h - REBATE,
        0.0, GLASS_T,
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    span = SIDE_LITE_W + MULLION_FACE + CENTER_LITE_W + MULLION_FACE + SIDE_LITE_W
    inner_w = INNER_X1 - INNER_X0
    assert abs(span - inner_w) < 1e-6, f"lite layout {span} != inner width {inner_w}"

    model = ArticulatedObject(name="corner_lift_sliding_window")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("roller", rgba=ROLLER_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material="vinyl",
        name="frame_shell",
    )

    opening_h = INNER_Z1 - INNER_Z0

    # --- Fixed side lites ---
    for name, cx_val in [("left_lite", (LEFT_X0 + LEFT_X1) / 2.0),
                          ("right_lite", (RIGHT_X0 + RIGHT_X1) / 2.0)]:
        lite = model.part(name)
        lite.visual(
            mesh_from_cadquery(_build_sash_grille_shape(SIDE_LITE_W, opening_h), f"{name}_vinyl"),
            material="vinyl",
            name=f"{name}_vinyl",
        )
        lite.visual(
            mesh_from_cadquery(_build_sash_glass_shape(SIDE_LITE_W, opening_h), f"{name}_glass"),
            material="glass",
            name=f"{name}_glass",
        )

    # --- Center sliding sash with roller blocks ---
    center_sash = model.part("center_sash")
    center_sash.visual(
        mesh_from_cadquery(_build_sash_grille_shape(CENTER_LITE_W, opening_h), "center_sash_vinyl"),
        material="vinyl",
        name="center_sash_vinyl",
    )
    center_sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(CENTER_LITE_W, opening_h), "center_sash_glass"),
        material="glass",
        name="center_sash_glass",
    )
    # Two roller blocks at the bottom of the sliding sash.
    # Sash local frame: centered at origin, extends +/- (opening_w/2 + SASH_FACE) in X,
    # +/- (opening_h/2 + SASH_FACE) in Z. Bottom rail at z = -(opening_h/2 + SASH_FACE).
    sash_bottom_z = -(opening_h / 2.0 + SASH_FACE)
    roller_inset = 0.08  # distance from sash edge to roller center
    roller_x_left = -(CENTER_LITE_W / 2.0 + SASH_FACE) + roller_inset
    roller_x_right = (CENTER_LITE_W / 2.0 + SASH_FACE) - roller_inset
    roller_z_center = sash_bottom_z - ROLLER_H / 2.0

    center_sash.visual(
        Box((ROLLER_W, ROLLER_D, ROLLER_H)),
        origin=Origin(xyz=(roller_x_left, 0.0, roller_z_center)),
        material="roller",
        name="roller_left",
    )
    center_sash.visual(
        Box((ROLLER_W, ROLLER_D, ROLLER_H)),
        origin=Origin(xyz=(roller_x_right, 0.0, roller_z_center)),
        material="roller",
        name="roller_right",
    )

    # --- Vent panel ---
    vent_panel = model.part("vent_panel")
    vent_panel.visual(
        mesh_from_cadquery(_build_vent_panel_shape(), "vent_panel_vinyl"),
        material="vinyl",
        name="vent_panel_vinyl",
    )
    vent_panel.visual(
        mesh_from_cadquery(_build_vent_glass_shape(), "vent_panel_glass"),
        material="glass",
        name="vent_panel_glass",
    )

    # --- Articulations ---
    mid_cz = (INNER_Z0 + INNER_Z1) / 2.0
    left_cx = (LEFT_X0 + LEFT_X1) / 2.0
    right_cx = (RIGHT_X0 + RIGHT_X1) / 2.0
    center_cx = (CENTER_X0 + CENTER_X1) / 2.0

    # Fixed side lites
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

    # Center sliding sash: PRISMATIC along +X
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

    # Vent panel: REVOLUTE at its bottom edge (hinge line along X).
    # The vent panel local frame has origin at its bottom-center.
    # Joint origin in world: at the vent panel hinge line.
    # Positive rotation tilts the top of the panel outward (+Y direction).
    # Axis: +X so positive q tilts the top edge outward (right-hand rule around +X).
    vent_hinge_world_x = VENT_CX
    vent_hinge_world_y = VENT_Y
    vent_hinge_world_z = VENT_HINGE_Z  # bottom edge of vent panel = hinge line
    vent_tilt_max = 0.55  # ~31 degrees max tilt

    model.articulation(
        "frame_to_vent_panel",
        ArticulationType.REVOLUTE,
        parent="frame",
        child="vent_panel",
        origin=Origin(xyz=(vent_hinge_world_x, vent_hinge_world_y, vent_hinge_world_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.0, lower=0.0, upper=vent_tilt_max),
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
    vent_panel = object_model.get_part("vent_panel")
    slide = object_model.get_articulation("frame_to_center_sash")
    vent_tilt = object_model.get_articulation("frame_to_vent_panel")

    # --- Intentional overlaps ---
    for nm in ("left_lite", "right_lite", "center_sash"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass",
            elem_b=f"{nm}_vinyl",
            reason="Clear pane is rebated under the sash/muntin lip so it reads captured.",
        )

    ctx.allow_overlap(
        "frame", "left_lite",
        elem_a="frame_shell", elem_b="left_lite_vinyl",
        reason="Left fixed lite is rebated into the frame opening.",
    )
    ctx.allow_overlap(
        "frame", "right_lite",
        elem_a="frame_shell", elem_b="right_lite_vinyl",
        reason="Right fixed lite is rebated into the frame opening.",
    )
    ctx.allow_overlap(
        "frame", "center_sash",
        elem_a="frame_shell", elem_b="center_sash_vinyl",
        reason="Center sash rides the head/sill track and laps the frame face.",
    )
    ctx.allow_overlap(
        "frame", "left_lite",
        elem_a="frame_shell", elem_b="left_lite_glass",
        reason="Left lite glass is rebated under the frame opening lip.",
    )
    ctx.allow_overlap(
        "frame", "right_lite",
        elem_a="frame_shell", elem_b="right_lite_glass",
        reason="Right lite glass is rebated under the frame opening lip.",
    )
    ctx.allow_overlap(
        "frame", "center_sash",
        elem_a="frame_shell", elem_b="center_sash_glass",
        reason="Center sash glass laps the head/sill track lip.",
    )
    # Vent panel glass captured in vent frame
    ctx.allow_overlap(
        "vent_panel", "vent_panel",
        elem_a="vent_panel_glass", elem_b="vent_panel_vinyl",
        reason="Vent panel glass is rebated under the vent frame lip.",
    )
    # Vent panel sits proud of the frame front face; small overlap at the hinge region
    ctx.allow_overlap(
        "frame", "vent_panel",
        elem_a="frame_shell", elem_b="vent_panel_vinyl",
        reason="Vent panel hinge region seats against the frame front face.",
    )
    # Roller blocks at the bottom of the center sash ride in the sill track groove.
    for roller_name in ("roller_left", "roller_right"):
        ctx.allow_overlap(
            "center_sash", "frame",
            elem_a=roller_name, elem_b="frame_shell",
            reason=f"{roller_name} rides in the sill track groove at the bottom of the sliding sash.",
        )

    # --- Structural checks at rest pose (q=0) ---
    with ctx.pose({slide: 0.0, vent_tilt: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        c_aabb = ctx.part_world_aabb(center_sash)

        # Frame spans full width
        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        center_w = c_aabb[1][0] - c_aabb[0][0]
        ctx.check(
            "frame spans wider than the center sash",
            frame_w > center_w + 1.5,
            details=f"frame_w={frame_w:.3f}, center_w={center_w:.3f}",
        )

        # Sill near z=0
        ctx.check(
            "sill sits near z=0",
            abs(frame_aabb[0][2]) < 0.02,
            details=f"frame zmin={frame_aabb[0][2]:.4f}",
        )

        # Vent panel exists and is near the top of the window
        vent_aabb = ctx.part_world_aabb(vent_panel)
        ctx.check(
            "vent panel is near the top of the window",
            vent_aabb[1][2] > TOTAL_H * 0.7,
            details=f"vent zmax={vent_aabb[1][2]:.3f}, threshold={TOTAL_H * 0.7:.3f}",
        )
        ctx.check(
            "vent panel is small relative to window",
            (vent_aabb[1][0] - vent_aabb[0][0]) < 0.5 and (vent_aabb[1][2] - vent_aabb[0][2]) < 0.4,
            details=f"vent size=({vent_aabb[1][0] - vent_aabb[0][0]:.3f}, {vent_aabb[1][2] - vent_aabb[0][2]:.3f})",
        )

        # Vent panel sits proud of (or at) the frame front face
        frame_front_y = frame_aabb[1][1]
        vent_center_y = (vent_aabb[0][1] + vent_aabb[1][1]) / 2.0
        ctx.check(
            "vent panel sits at or beyond frame front face",
            vent_center_y > frame_front_y - 0.02,
            details=f"vent_y={vent_center_y:.3f}, frame_front_y={frame_front_y:.3f}",
        )

        rest_cx = (c_aabb[0][0] + c_aabb[1][0]) / 2.0
        rest_cz = (c_aabb[0][2] + c_aabb[1][2]) / 2.0

    # --- Sliding sash moves along +X ---
    travel = slide.motion_limits.upper
    with ctx.pose({slide: travel, vent_tilt: 0.0}):
        c_open = ctx.part_world_aabb(center_sash)
        open_cx = (c_open[0][0] + c_open[1][0]) / 2.0
        ctx.check(
            "center sash slides along +X by ~travel",
            abs((open_cx - rest_cx) - travel) < 0.02,
            details=f"rest_cx={rest_cx:.3f}, open_cx={open_cx:.3f}, travel={travel:.3f}",
        )
        # Pure horizontal slide
        c_open_z = (c_open[0][2] + c_open[1][2]) / 2.0
        ctx.check(
            "slide is purely horizontal",
            abs(c_open_z - rest_cz) < 0.02,
            details=f"open_z={c_open_z:.3f}, rest_z={rest_cz:.3f}",
        )
        # Retained in frame
        f_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "sash retained within frame X span at full travel",
            c_open[1][0] < f_aabb[1][0] + 1e-4 and c_open[0][0] > f_aabb[0][0] - 1e-4,
            details=f"sash x=[{c_open[0][0]:.3f},{c_open[1][0]:.3f}] frame x=[{f_aabb[0][0]:.3f},{f_aabb[1][0]:.3f}]",
        )

    # --- Vent panel tilts outward ---
    with ctx.pose({slide: 0.0, vent_tilt: 0.0}):
        vent_rest_aabb = ctx.part_world_aabb(vent_panel)
        vent_rest_y_max = vent_rest_aabb[1][1]

    with ctx.pose({slide: 0.0, vent_tilt: vent_tilt.motion_limits.upper}):
        vent_open_aabb = ctx.part_world_aabb(vent_panel)
        vent_open_y_max = vent_open_aabb[1][1]
        ctx.check(
            "vent panel tilts outward (top edge moves in +Y)",
            vent_open_y_max > vent_rest_y_max + 0.01,
            details=f"rest_y_max={vent_rest_y_max:.4f}, open_y_max={vent_open_y_max:.4f}",
        )
        # The vent panel bottom (hinge) should not move much
        vent_open_z_min = vent_open_aabb[0][2]
        vent_rest_z_min = vent_rest_aabb[0][2]
        ctx.check(
            "vent panel hinge stays near its rest position",
            abs(vent_open_z_min - vent_rest_z_min) < 0.03,
            details=f"rest_z_min={vent_rest_z_min:.4f}, open_z_min={vent_open_z_min:.4f}",
        )

    # --- Roller blocks exist on the sliding sash ---
    ctx.check(
        "roller_left visual exists on center_sash",
        center_sash.get_visual("roller_left") is not None,
        details="Expected roller_left visual on center_sash",
    )
    ctx.check(
        "roller_right visual exists on center_sash",
        center_sash.get_visual("roller_right") is not None,
        details="Expected roller_right visual on center_sash",
    )

    # Roller blocks are at the bottom of the sash, near the frame sill track.
    with ctx.pose({slide: 0.0, vent_tilt: 0.0}):
        ctx.expect_contact(
            center_sash, frame,
            elem_a="roller_left", elem_b="frame_shell",
            contact_tol=0.025,
            name="roller_left contacts the sill track region",
        )
        ctx.expect_contact(
            center_sash, frame,
            elem_a="roller_right", elem_b="frame_shell",
            contact_tol=0.025,
            name="roller_right contacts the sill track region",
        )

    # Verify both joints are non-fixed
    ctx.check(
        "slide joint is prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"slide type={slide.articulation_type}",
    )
    ctx.check(
        "vent tilt joint is revolute",
        vent_tilt.articulation_type == ArticulationType.REVOLUTE,
        details=f"vent_tilt type={vent_tilt.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
