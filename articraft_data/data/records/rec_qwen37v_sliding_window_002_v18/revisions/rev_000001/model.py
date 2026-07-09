from __future__ import annotations

# Corner-lift sliding window variant: white vinyl frame with horizontal mullion
# creating a transom area. One fixed sash (left-lower) + one sliding sash
# (right-lower, PRISMATIC) + small vent panel (upper-right, FIXED) + corner-lift
# handle (REVOLUTE on sliding sash) + two roller blocks at sliding sash bottom.
#
# Coordinate convention:
#   +Z is up. Window stands vertically.
#     width  -> X, height -> Z (sill near z=0), depth -> Y
#   Glass plane is X-Z. q=0 reads SHUT. Driving the prismatic joint slides the
#   right sash sideways toward the fixed left sash (-X) to open.
#
# Structure:
#   - frame (root): head, sill, jambs + horizontal mullion + vertical mullion
#   - fixed_sash (lower-left, FIXED): vinyl sash ring + clear glass
#   - sliding_sash (lower-right, PRISMATIC): vinyl sash ring + glass + rollers
#   - vent_panel (upper-right, FIXED): small sash ring + clear glass
#   - lift_handle (REVOLUTE on sliding_sash): corner-lift lever

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
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

MEETING_OVERLAP = 0.040

SASH_FACE = 0.050
SASH_DEPTH = 0.060
GLASS_T = 0.008
REBATE = 0.005

# Mullion dimensions
MULLION_FACE = 0.055
MULLION_DEPTH = FRAME_DEPTH

# Vent panel sash
VENT_SASH_FACE = 0.040

# Y layout: fixed sash rear, sliding sash proud (front track)
FIXED_SASH_Y = -0.028
SLIDE_SASH_Y = 0.044

# Roller blocks
ROLLER_W = 0.028
ROLLER_D = 0.020
ROLLER_H = 0.014

# Lift handle
HANDLE_BASE_W = 0.030
HANDLE_BASE_T = 0.010
HANDLE_BASE_H = 0.050
HANDLE_LEVER_LEN = 0.050
HANDLE_LEVER_R = 0.005
HANDLE_GRIP_R = 0.008

# Latch plate (kept on sliding sash for realism)
LATCH_PLATE_W = 0.028
LATCH_PLATE_H = 0.070
LATCH_PLATE_T = 0.010

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

# Horizontal mullion at ~72% height
H_MULLION_Z = INNER_Z0 + 0.72 * INNER_H

# Vertical mullion in upper area (separating vent panel from transom)
V_MULLION_X = INNER_X1 - 0.32

# Lower sash area
SASH_BOTTOM = INNER_Z0
SASH_TOP = H_MULLION_Z - MULLION_FACE / 2.0
SASH_OPENING_H = SASH_TOP - SASH_BOTTOM
SASH_OPENING_W = (INNER_W + MEETING_OVERLAP) / 2.0

SASH_CENTER_Z = (SASH_BOTTOM + SASH_TOP) / 2.0

# Sash outer dimensions
sash_out_w = SASH_OPENING_W + 2.0 * SASH_FACE
sash_out_h = SASH_OPENING_H + 2.0 * SASH_FACE

# Opening centers (world X) of each sash
FIXED_OPEN_CX = INNER_X0 + SASH_OPENING_W / 2.0
SLIDE_OPEN_CX = INNER_X1 - SASH_OPENING_W / 2.0

# Vent panel opening (upper-right)
VENT_X0 = V_MULLION_X + MULLION_FACE / 2.0
VENT_X1 = INNER_X1
VENT_Z0 = H_MULLION_Z + MULLION_FACE / 2.0
VENT_Z1 = INNER_Z1
VENT_W = VENT_X1 - VENT_X0
VENT_H = VENT_Z1 - VENT_Z0
VENT_CX = (VENT_X0 + VENT_X1) / 2.0
VENT_CZ = (VENT_Z0 + VENT_Z1) / 2.0

# Transom opening (upper-left) - for fixed transom glass
TRANSOM_X0 = INNER_X0
TRANSOM_X1 = V_MULLION_X - MULLION_FACE / 2.0
TRANSOM_Z0 = VENT_Z0
TRANSOM_Z1 = INNER_Z1
TRANSOM_CX = (TRANSOM_X0 + TRANSOM_X1) / 2.0
TRANSOM_CZ = (TRANSOM_Z0 + TRANSOM_Z1) / 2.0
TRANSOM_W = TRANSOM_X1 - TRANSOM_X0
TRANSOM_H = TRANSOM_Z1 - TRANSOM_Z0

# Slide travel
slide_travel = SASH_OPENING_W * 0.85

# Roller positions in sliding sash local frame
roller_bottom_z = -sash_out_h / 2.0
roller_cz = roller_bottom_z - ROLLER_H / 2.0 + 0.004  # recessed 4mm
left_roller_x = -(SASH_OPENING_W / 2.0 + SASH_FACE / 2.0) + 0.040
right_roller_x = (SASH_OPENING_W / 2.0 + SASH_FACE / 2.0) - 0.040

# Handle pivot in sliding sash local frame
HANDLE_PIVOT_X = -(SASH_OPENING_W / 2.0 + SASH_FACE / 2.0)
HANDLE_PIVOT_Y = SASH_DEPTH / 2.0 + 0.005
HANDLE_PIVOT_Z = 0.08

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

VINYL_RGBA = (0.94, 0.95, 0.96, 1.0)
GLASS_RGBA = (0.52, 0.60, 0.66, 0.30)
METAL_RGBA = (0.74, 0.76, 0.79, 1.0)
NYLON_RGBA = (0.18, 0.18, 0.20, 1.0)


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _slab(x0: float, x1: float, z0: float, z1: float, y_center: float, depth: float) -> cq.Workplane:
    """Axis-aligned box in the X-Z plane, centered on y_center."""
    return (
        cq.Workplane("XY")
        .transformed(offset=((x0 + x1) / 2.0, y_center, (z0 + z1) / 2.0))
        .box(x1 - x0, depth, z1 - z0)
    )


def _build_frame_shape() -> cq.Workplane:
    """Outer frame with horizontal and vertical mullions."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)
    cut_depth = FRAME_DEPTH + 0.02
    # Cut the entire inner opening
    opening = _slab(INNER_X0, INNER_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    frame = outer.cut(opening)

    # Add horizontal mullion bar (spanning full inner width)
    h_mullion = _slab(
        INNER_X0, INNER_X1,
        H_MULLION_Z - MULLION_FACE / 2.0, H_MULLION_Z + MULLION_FACE / 2.0,
        0.0, MULLION_DEPTH,
    )
    frame = frame.union(h_mullion)

    # Add vertical mullion bar (upper area only, separating vent panel)
    v_mullion = _slab(
        V_MULLION_X - MULLION_FACE / 2.0, V_MULLION_X + MULLION_FACE / 2.0,
        H_MULLION_Z + MULLION_FACE / 2.0, INNER_Z1,
        0.0, MULLION_DEPTH,
    )
    frame = frame.union(v_mullion)

    return frame


def _build_sash_shape() -> cq.Workplane:
    """Main sash ring in its own local frame (centered at origin)."""
    ow = SASH_OPENING_W
    oh = SASH_OPENING_H
    out_w = ow + 2.0 * SASH_FACE
    out_h = oh + 2.0 * SASH_FACE
    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, SASH_DEPTH)
    opening = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SASH_DEPTH + 0.02)
    return outer.cut(opening)


def _build_sash_glass_shape() -> cq.Workplane:
    """Glass pane for main sash (local frame, centered)."""
    ow = SASH_OPENING_W + 2.0 * REBATE
    oh = SASH_OPENING_H + 2.0 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


def _build_vent_sash_shape() -> cq.Workplane:
    """Vent panel sash ring (local frame, centered)."""
    ow = VENT_W
    oh = VENT_H
    out_w = ow + 2.0 * VENT_SASH_FACE
    out_h = oh + 2.0 * VENT_SASH_FACE
    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, SASH_DEPTH)
    opening = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SASH_DEPTH + 0.02)
    return outer.cut(opening)


def _build_vent_glass_shape() -> cq.Workplane:
    """Glass pane for vent panel (local frame, centered)."""
    gw = VENT_W + 2.0 * REBATE
    gh = VENT_H + 2.0 * REBATE
    return _slab(-gw / 2.0, gw / 2.0, -gh / 2.0, gh / 2.0, 0.0, GLASS_T)


def _build_transom_glass_shape() -> cq.Workplane:
    """Transom glass pane (upper-left fixed light, world-positioned).
    Sized with 3mm margin into frame/mullion edges for connectivity."""
    margin = 0.003
    gw = TRANSOM_W + 2.0 * margin
    gh = TRANSOM_H + 2.0 * margin
    return _slab(
        TRANSOM_CX - gw / 2.0, TRANSOM_CX + gw / 2.0,
        TRANSOM_CZ - gh / 2.0, TRANSOM_CZ + gh / 2.0,
        0.0, GLASS_T,
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="corner_lift_sliding_window")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("metal", rgba=METAL_RGBA)
    model.material("nylon", rgba=NYLON_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame_shell"),
        material="vinyl",
        name="frame_shell",
    )
    # Transom glass (upper-left fixed light, mounted in frame)
    frame.visual(
        mesh_from_cadquery(_build_transom_glass_shape(), "transom_glass"),
        material="glass",
        name="transom_glass",
    )

    # --- Fixed (lower-left) sash ---
    fixed_sash = model.part("fixed_sash")
    fixed_sash.visual(
        mesh_from_cadquery(_build_sash_shape(), "fixed_sash_vinyl"),
        material="vinyl",
        name="fixed_sash_vinyl",
    )
    fixed_sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(), "fixed_sash_glass"),
        material="glass",
        name="fixed_sash_glass",
    )

    # --- Sliding (lower-right) sash with roller blocks ---
    sliding_sash = model.part("sliding_sash")
    sliding_sash.visual(
        mesh_from_cadquery(_build_sash_shape(), "sliding_sash_vinyl"),
        material="vinyl",
        name="sliding_sash_vinyl",
    )
    sliding_sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(), "sliding_sash_glass"),
        material="glass",
        name="sliding_sash_glass",
    )
    # Latch plate on meeting stile
    stile_x = -(SASH_OPENING_W / 2.0 + SASH_FACE / 2.0)
    face_y = SASH_DEPTH / 2.0
    plate_y = face_y + LATCH_PLATE_T / 2.0
    sliding_sash.visual(
        Box((LATCH_PLATE_W, LATCH_PLATE_T, LATCH_PLATE_H)),
        origin=Origin(xyz=(stile_x, plate_y, -0.02)),
        material="metal",
        name="sliding_sash_latch_plate",
    )
    # Two roller blocks at the bottom of the sliding sash
    sliding_sash.visual(
        Box((ROLLER_W, ROLLER_D, ROLLER_H)),
        origin=Origin(xyz=(left_roller_x, 0.0, roller_cz)),
        material="nylon",
        name="roller_left",
    )
    sliding_sash.visual(
        Box((ROLLER_W, ROLLER_D, ROLLER_H)),
        origin=Origin(xyz=(right_roller_x, 0.0, roller_cz)),
        material="nylon",
        name="roller_right",
    )

    # --- Vent panel (upper-right, FIXED) ---
    vent_panel = model.part("vent_panel")
    vent_panel.visual(
        mesh_from_cadquery(_build_vent_sash_shape(), "vent_panel_vinyl"),
        material="vinyl",
        name="vent_panel_vinyl",
    )
    vent_panel.visual(
        mesh_from_cadquery(_build_vent_glass_shape(), "vent_panel_glass"),
        material="glass",
        name="vent_panel_glass",
    )

    # --- Corner-lift handle (REVOLUTE on sliding sash) ---
    lift_handle = model.part("lift_handle")
    # Base plate at the pivot origin
    lift_handle.visual(
        Box((HANDLE_BASE_W, HANDLE_BASE_T, HANDLE_BASE_H)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="metal",
        name="handle_base",
    )
    # Lever arm extending along +X from pivot (horizontal at q=0)
    # Cylinder is along local +Z, rotate to lie along +X: rpy=(0, pi/2, 0)
    lift_handle.visual(
        Cylinder(radius=HANDLE_LEVER_R, length=HANDLE_LEVER_LEN),
        origin=Origin(xyz=(HANDLE_LEVER_LEN / 2.0, 0.0, 0.0), rpy=(0.0, 1.5708, 0.0)),
        material="metal",
        name="handle_lever",
    )
    # Grip ball at the end of the lever
    lift_handle.visual(
        Sphere(radius=HANDLE_GRIP_R),
        origin=Origin(xyz=(HANDLE_LEVER_LEN, 0.0, 0.0)),
        material="metal",
        name="handle_grip",
    )

    # -----------------------------------------------------------------------
    # Articulations
    # -----------------------------------------------------------------------

    # Fixed sash: FIXED, seated in rear glazing plane
    model.articulation(
        "frame_to_fixed_sash",
        ArticulationType.FIXED,
        parent="frame",
        child="fixed_sash",
        origin=Origin(xyz=(FIXED_OPEN_CX, FIXED_SASH_Y, SASH_CENTER_Z)),
    )

    # Sliding sash: PRISMATIC along X. Positive q slides toward fixed sash (-X).
    model.articulation(
        "frame_to_sliding_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="sliding_sash",
        origin=Origin(xyz=(SLIDE_OPEN_CX, SLIDE_SASH_Y, SASH_CENTER_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.5, lower=0.0, upper=slide_travel),
    )

    # Vent panel: FIXED, seated in upper-right frame opening
    model.articulation(
        "frame_to_vent_panel",
        ArticulationType.FIXED,
        parent="frame",
        child="vent_panel",
        origin=Origin(xyz=(VENT_CX, FIXED_SASH_Y, VENT_CZ)),
    )

    # Corner-lift handle: REVOLUTE on sliding sash
    # Pivot at the meeting stile, axis along -Y so positive q rotates lever
    # from horizontal (+X) upward (+Z).
    model.articulation(
        "sash_to_handle",
        ArticulationType.REVOLUTE,
        parent="sliding_sash",
        child="lift_handle",
        origin=Origin(xyz=(HANDLE_PIVOT_X, HANDLE_PIVOT_Y, HANDLE_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=0.0, upper=1.5708),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    fixed_sash = object_model.get_part("fixed_sash")
    sliding_sash = object_model.get_part("sliding_sash")
    vent_panel = object_model.get_part("vent_panel")
    lift_handle = object_model.get_part("lift_handle")
    slide = object_model.get_articulation("frame_to_sliding_sash")
    handle_joint = object_model.get_articulation("sash_to_handle")

    # --- Intentional overlaps ---
    # Glass rebated under sash lip on each main sash
    for nm in ("fixed_sash", "sliding_sash"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass",
            elem_b=f"{nm}_vinyl",
            reason="Clear pane is rebated under the sash lip (captured glazing).",
        )
    # Main sashes seated in frame opening / track
    for nm in ("fixed_sash", "sliding_sash"):
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_vinyl",
            reason=f"{nm} ring is rebated into the frame track (seated capture).",
        )
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_glass",
            reason=f"{nm} glass is rebated under the frame opening lip.",
        )
    # Latch plate seated on sliding sash
    ctx.allow_overlap(
        "sliding_sash", "sliding_sash",
        elem_a="sliding_sash_latch_plate",
        elem_b="sliding_sash_vinyl",
        reason="Latch plate is seated on the sliding sash meeting stile face.",
    )
    # Roller blocks recessed into sash bottom rail
    ctx.allow_overlap(
        "sliding_sash", "sliding_sash",
        elem_a="roller_left",
        elem_b="sliding_sash_vinyl",
        reason="Left roller is recessed into the sliding sash bottom rail (mounted hardware).",
    )
    ctx.allow_overlap(
        "sliding_sash", "sliding_sash",
        elem_a="roller_right",
        elem_b="sliding_sash_vinyl",
        reason="Right roller is recessed into the sliding sash bottom rail (mounted hardware).",
    )
    # Handle base seated on sash stile
    ctx.allow_overlap(
        "sliding_sash", "lift_handle",
        elem_a="sliding_sash_vinyl",
        elem_b="handle_base",
        reason="Lift handle base is seated onto the sliding sash meeting stile (mounted hardware).",
    )
    # Vent panel glass rebated under vent sash lip
    ctx.allow_overlap(
        "vent_panel", "vent_panel",
        elem_a="vent_panel_glass",
        elem_b="vent_panel_vinyl",
        reason="Vent glass is rebated under the vent panel sash lip.",
    )
    # Vent panel seated in frame opening
    ctx.allow_overlap(
        "frame", "vent_panel",
        elem_a="frame_shell",
        elem_b="vent_panel_vinyl",
        reason="Vent panel sash ring is seated in the frame upper-right opening.",
    )
    ctx.allow_overlap(
        "frame", "vent_panel",
        elem_a="frame_shell",
        elem_b="vent_panel_glass",
        reason="Vent panel glass is within the frame opening rebate.",
    )
    # Transom glass seated in frame
    ctx.allow_overlap(
        "frame", "frame",
        elem_a="transom_glass",
        elem_b="frame_shell",
        reason="Transom glass is seated in the frame upper-left opening (captured glazing).",
    )

    # --- Closed pose (q=0): window reads SHUT ---
    with ctx.pose({slide: 0.0, handle_joint: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        f_aabb = ctx.part_world_aabb(fixed_sash)
        s_aabb = ctx.part_world_aabb(sliding_sash)
        v_aabb = ctx.part_world_aabb(vent_panel)

        # Frame spans the full width
        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        sash_w = s_aabb[1][0] - s_aabb[0][0]
        ctx.check(
            "frame spans wider than a single sash",
            frame_w > sash_w + 0.40,
            details=f"frame_w={frame_w:.3f}, sash_w={sash_w:.3f}",
        )
        # Sill near z=0, head at full height
        ctx.check(
            "sill sits near z=0",
            abs(frame_aabb[0][2]) < 0.02,
            details=f"frame zmin={frame_aabb[0][2]:.4f}",
        )
        ctx.check(
            "head reaches full height",
            abs(frame_aabb[1][2] - TOTAL_H) < 0.02,
            details=f"frame zmax={frame_aabb[1][2]:.4f}",
        )
        # Fixed sash left of sliding sash
        fx = (f_aabb[0][0] + f_aabb[1][0]) / 2.0
        sx = (s_aabb[0][0] + s_aabb[1][0]) / 2.0
        ctx.check(
            "fixed sash left of sliding sash",
            fx < sx,
            details=f"fixed_x={fx:.3f}, sliding_x={sx:.3f}",
        )
        # Both main sashes within frame height
        for nm, ab in (("fixed", f_aabb), ("sliding", s_aabb)):
            ctx.check(
                f"{nm} sash seated within frame height",
                ab[0][2] > frame_aabb[0][2] - 1e-4 and ab[1][2] < frame_aabb[1][2] + 1e-4,
                details=f"{nm} z=[{ab[0][2]:.3f},{ab[1][2]:.3f}]",
            )
        # Sliding sash proud of fixed sash
        fy = (f_aabb[0][1] + f_aabb[1][1]) / 2.0
        sy = (s_aabb[0][1] + s_aabb[1][1]) / 2.0
        ctx.check(
            "sliding sash proud of fixed sash",
            sy > fy + 0.02,
            details=f"sliding_y={sy:.3f}, fixed_y={fy:.3f}",
        )
        # Sashes seated in frame opening
        ctx.expect_overlap(
            fixed_sash, frame, axes="xz", min_overlap=0.03,
            name="fixed sash seated in frame opening",
        )
        ctx.expect_overlap(
            sliding_sash, frame, axes="xz", min_overlap=0.03,
            name="sliding sash seated in frame opening",
        )

        # --- Vent panel checks ---
        vent_cz = (v_aabb[0][2] + v_aabb[1][2]) / 2.0
        sash_top = s_aabb[1][2]
        ctx.check(
            "vent panel above main sashes",
            vent_cz > sash_top - 0.05,
            details=f"vent_cz={vent_cz:.3f}, sash_top={sash_top:.3f}",
        )
        # Vent panel is small (narrower than a main sash)
        vent_w = v_aabb[1][0] - v_aabb[0][0]
        ctx.check(
            "vent panel narrower than main sash",
            vent_w < sash_w * 0.60,
            details=f"vent_w={vent_w:.3f}, sash_w={sash_w:.3f}",
        )
        # Vent panel on the right side of the window
        vent_cx = (v_aabb[0][0] + v_aabb[1][0]) / 2.0
        ctx.check(
            "vent panel on right side",
            vent_cx > 0.0,
            details=f"vent_cx={vent_cx:.3f}",
        )

        # --- Roller block checks ---
        roller_l_aabb = ctx.part_element_world_aabb(sliding_sash, elem="roller_left")
        roller_r_aabb = ctx.part_element_world_aabb(sliding_sash, elem="roller_right")
        # Rollers are near the bottom of the sliding sash
        roller_l_z = (roller_l_aabb[0][2] + roller_l_aabb[1][2]) / 2.0
        roller_r_z = (roller_r_aabb[0][2] + roller_r_aabb[1][2]) / 2.0
        sash_bottom = s_aabb[0][2]
        ctx.check(
            "left roller near sash bottom",
            abs(roller_l_z - sash_bottom) < 0.04,
            details=f"roller_z={roller_l_z:.3f}, sash_bottom={sash_bottom:.3f}",
        )
        ctx.check(
            "right roller near sash bottom",
            abs(roller_r_z - sash_bottom) < 0.04,
            details=f"roller_z={roller_r_z:.3f}, sash_bottom={sash_bottom:.3f}",
        )
        # Rollers are separated in X (left and right positions)
        roller_l_x = (roller_l_aabb[0][0] + roller_l_aabb[1][0]) / 2.0
        roller_r_x = (roller_r_aabb[0][0] + roller_r_aabb[1][0]) / 2.0
        ctx.check(
            "rollers separated left-right",
            roller_r_x - roller_l_x > 0.30,
            details=f"left_x={roller_l_x:.3f}, right_x={roller_r_x:.3f}",
        )

        rest_sx = sx
        rest_sz = (s_aabb[0][2] + s_aabb[1][2]) / 2.0

    # --- Open pose: sliding sash slides toward fixed sash (-X) ---
    travel = slide.motion_limits.upper
    with ctx.pose({slide: travel, handle_joint: 0.0}):
        s_open = ctx.part_world_aabb(sliding_sash)
        open_sx = (s_open[0][0] + s_open[1][0]) / 2.0
        # Positive q moves sash in -X
        ctx.check(
            "sliding sash opens toward fixed sash (-X)",
            abs((rest_sx - open_sx) - travel) < 0.02 and open_sx < rest_sx - 0.30,
            details=f"rest_x={rest_sx:.3f}, open_x={open_sx:.3f}, travel={travel:.3f}",
        )
        # Pure horizontal slide (no Z change)
        open_sz = (s_open[0][2] + s_open[1][2]) / 2.0
        ctx.check(
            "slide is purely horizontal",
            abs(open_sz - rest_sz) < 0.02,
            details=f"open_z={open_sz:.3f}, rest_z={rest_sz:.3f}",
        )
        # Retained insertion: sash stays within frame X span
        f_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "sash retained within frame X span at full travel",
            s_open[0][0] > f_aabb[0][0] - 1e-4 and s_open[1][0] < f_aabb[1][0] + 1e-4,
            details=f"sash x=[{s_open[0][0]:.3f},{s_open[1][0]:.3f}] frame x=[{f_aabb[0][0]:.3f},{f_aabb[1][0]:.3f}]",
        )
        ctx.expect_overlap(
            sliding_sash, frame, axes="z", min_overlap=0.10,
            name="sash retains vertical engagement with head/sill track",
        )

    # --- Handle rotation check ---
    with ctx.pose({slide: 0.0, handle_joint: 0.0}):
        handle_rest = ctx.part_element_world_aabb(lift_handle, elem="handle_grip")
        grip_rest_z = (handle_rest[0][2] + handle_rest[1][2]) / 2.0
        grip_rest_x = (handle_rest[0][0] + handle_rest[1][0]) / 2.0

    with ctx.pose({slide: 0.0, handle_joint: 1.5708}):
        handle_rot = ctx.part_element_world_aabb(lift_handle, elem="handle_grip")
        grip_rot_z = (handle_rot[0][2] + handle_rot[1][2]) / 2.0
        grip_rot_x = (handle_rot[0][0] + handle_rot[1][0]) / 2.0

    ctx.check(
        "handle grip rises when rotated (corner-lift)",
        grip_rot_z > grip_rest_z + 0.02,
        details=f"rest_z={grip_rest_z:.3f}, rotated_z={grip_rot_z:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
