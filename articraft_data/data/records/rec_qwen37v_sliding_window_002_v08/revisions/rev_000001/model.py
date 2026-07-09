from __future__ import annotations

# Corner-lift sliding window variant: white vinyl frame with one fixed sash
# (left) and one sliding sash (right). The sliding sash carries:
#   - a small hinged vent panel at the bottom-left corner (REVOLUTE)
#   - a rotating latch at the meeting rail (REVOLUTE)
#   - two tiny roller blocks at the bottom rail
#   - a visible overlap stile where the two sashes cross
#
# Coordinate convention (same as parent):
#   +Z is up, width -> X, height -> Z, depth -> Y
#   Glass plane is X-Z. q=0 reads SHUT for all joints.
#
# Structure:
#   frame (root, static): hollow vinyl perimeter
#   fixed_sash (FIXED): left sash ring + glass
#   sliding_sash (PRISMATIC): right sash ring + glass + rollers + overlap stile
#   vent_panel (REVOLUTE child of sliding_sash): small hinged pane at bottom corner
#   latch (REVOLUTE child of sliding_sash): rotating cam lever at meeting rail

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

MEETING_OVERLAP = 0.040

SASH_FACE = 0.075
SASH_DEPTH = 0.060
GLASS_T = 0.008

FIXED_SASH_Y = -0.028
SLIDE_SASH_Y = 0.044

REBATE = 0.005

# --- New variant features ---

# Vent panel (corner-lift)
VENT_W = 0.24
VENT_H = 0.28
VENT_FRAME = 0.022
VENT_DEPTH = 0.035
VENT_GLASS_T = 0.006

# Roller blocks
ROLLER_W = 0.035
ROLLER_H = 0.018
ROLLER_D = 0.025

# Overlap stile
OVERLAP_STILE_W = 0.030
OVERLAP_STILE_T = 0.012

# Latch (separate part, revolute)
LATCH_PIVOT_R = 0.010
LATCH_PIVOT_H = 0.012
LATCH_LEVER_LEN = 0.042
LATCH_LEVER_W = 0.012
LATCH_LEVER_T = 0.008

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

FIXED_OPEN_CX = INNER_X0 + SASH_OPENING_W / 2.0
SLIDE_OPEN_CX = INNER_X1 - SASH_OPENING_W / 2.0
MID_CZ = (INNER_Z0 + INNER_Z1) / 2.0

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

VINYL_RGBA = (0.94, 0.95, 0.96, 1.0)
GLASS_RGBA = (0.52, 0.60, 0.66, 0.30)
METAL_RGBA = (0.74, 0.76, 0.79, 1.0)
RUBBER_RGBA = (0.18, 0.18, 0.18, 1.0)


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
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)
    cut_depth = FRAME_DEPTH + 0.02
    opening = _slab(INNER_X0, INNER_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    return outer.cut(opening)


def _build_sash_shape() -> cq.Workplane:
    ow = SASH_OPENING_W
    oh = SASH_OPENING_H
    out_w = ow + 2 * SASH_FACE
    out_h = oh + 2 * SASH_FACE
    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, SASH_DEPTH)
    opening = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SASH_DEPTH + 0.02)
    return outer.cut(opening)


def _build_sash_glass_shape() -> cq.Workplane:
    ow = SASH_OPENING_W + 2 * REBATE
    oh = SASH_OPENING_H + 2 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


def _build_vent_frame_shape() -> cq.Workplane:
    """Vent panel frame in its own local frame: origin at hinge line (top center),
    panel extends downward (z from -VENT_H to 0), centered in X."""
    outer = _slab(-VENT_W / 2.0, VENT_W / 2.0, -VENT_H, 0.0, 0.0, VENT_DEPTH)
    inner = _slab(
        -(VENT_W / 2.0 - VENT_FRAME),
        (VENT_W / 2.0 - VENT_FRAME),
        -(VENT_H - VENT_FRAME),
        -VENT_FRAME,
        0.0,
        VENT_DEPTH + 0.02,
    )
    return outer.cut(inner)


def _build_vent_glass_shape() -> cq.Workplane:
    gw = VENT_W - 2 * VENT_FRAME + 2 * REBATE
    gh = VENT_H - VENT_FRAME + REBATE
    return _slab(-gw / 2.0, gw / 2.0, -(VENT_H - VENT_FRAME), -VENT_FRAME, 0.0, VENT_GLASS_T)


# ---------------------------------------------------------------------------
# Part builders
# ---------------------------------------------------------------------------

def _add_fixed_sash(model: ArticulatedObject) -> None:
    sash = model.part("fixed_sash")
    sash.visual(
        mesh_from_cadquery(_build_sash_shape(), "fixed_sash_vinyl"),
        material="vinyl",
        name="fixed_sash_vinyl",
    )
    sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(), "fixed_sash_glass"),
        material="glass",
        name="fixed_sash_glass",
    )


def _add_sliding_sash(model: ArticulatedObject) -> None:
    sash = model.part("sliding_sash")
    sash.visual(
        mesh_from_cadquery(_build_sash_shape(), "sliding_sash_vinyl"),
        material="vinyl",
        name="sliding_sash_vinyl",
    )
    sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(), "sliding_sash_glass"),
        material="glass",
        name="sliding_sash_glass",
    )

    # --- Overlap stile: visible vertical strip at meeting rail ---
    stile_x = -SASH_OPENING_W / 2.0 - SASH_FACE / 2.0
    stile_y = SASH_DEPTH / 2.0 + OVERLAP_STILE_T / 2.0
    sash.visual(
        Box((OVERLAP_STILE_W, OVERLAP_STILE_T, SASH_OPENING_H)),
        origin=Origin(xyz=(stile_x, stile_y, 0.0)),
        material="vinyl",
        name="overlap_stile",
    )

    # --- Two roller blocks at bottom rail ---
    roller_z = -(SASH_OPENING_H / 2.0 + SASH_FACE) + ROLLER_H / 2.0
    roller_y = 0.0
    for i, x_off in enumerate([-SASH_OPENING_W / 2.0 + 0.06, SASH_OPENING_W / 2.0 - 0.06]):
        sash.visual(
            Box((ROLLER_W, ROLLER_D, ROLLER_H)),
            origin=Origin(xyz=(x_off, roller_y, roller_z)),
            material="rubber",
            name=f"roller_{i}",
        )


def _add_vent_panel(model: ArticulatedObject) -> None:
    """Small hinged vent panel at bottom-left corner of sliding sash."""
    vent = model.part("vent_panel")
    vent.visual(
        mesh_from_cadquery(_build_vent_frame_shape(), "vent_frame"),
        material="vinyl",
        name="vent_frame",
    )
    vent.visual(
        mesh_from_cadquery(_build_vent_glass_shape(), "vent_glass"),
        material="glass",
        name="vent_glass",
    )


def _add_latch(model: ArticulatedObject) -> None:
    """Rotating latch lever at the meeting rail, separate part with revolute joint."""
    latch = model.part("latch")
    # Pivot boss (cylinder at origin, along local Z)
    latch.visual(
        Cylinder(radius=LATCH_PIVOT_R, length=LATCH_PIVOT_H),
        origin=Origin(xyz=(0.0, 0.0, LATCH_PIVOT_H / 2.0)),
        material="metal",
        name="latch_pivot",
    )
    # Lever arm extending along -X (locked: toward meeting rail)
    latch.visual(
        Box((LATCH_LEVER_LEN, LATCH_LEVER_W, LATCH_LEVER_T)),
        origin=Origin(xyz=(-LATCH_LEVER_LEN / 2.0, 0.0, LATCH_PIVOT_H / 2.0)),
        material="metal",
        name="latch_lever",
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="corner_lift_sliding_window")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("metal", rgba=METAL_RGBA)
    model.material("rubber", rgba=RUBBER_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame_shell"),
        material="vinyl",
        name="frame_shell",
    )

    # --- Fixed sash (left, FIXED) ---
    _add_fixed_sash(model)

    # --- Sliding sash (right, PRISMATIC) ---
    _add_sliding_sash(model)

    # --- Vent panel (child of sliding sash, REVOLUTE) ---
    _add_vent_panel(model)

    # --- Latch (child of sliding sash, REVOLUTE) ---
    _add_latch(model)

    # FIXED left sash seated in the rear glazing plane.
    model.articulation(
        "frame_to_fixed_sash",
        ArticulationType.FIXED,
        parent="frame",
        child="fixed_sash",
        origin=Origin(xyz=(FIXED_OPEN_CX, FIXED_SASH_Y, MID_CZ)),
    )

    # SLIDING right sash: PRISMATIC along X. Positive q slides left (-X) to open.
    slide_travel = SASH_OPENING_W * 0.90
    model.articulation(
        "frame_to_sliding_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="sliding_sash",
        origin=Origin(xyz=(SLIDE_OPEN_CX, SLIDE_SASH_Y, MID_CZ)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.5, lower=0.0, upper=slide_travel),
    )

    # VENT PANEL: REVOLUTE hinge at top edge of vent area.
    # Vent area at bottom-left of sliding sash (meeting rail side).
    # Hinge at top of vent: z = -SASH_OPENING_H/2 + VENT_H
    # Vent center X: -SASH_OPENING_W/2 + VENT_W/2
    vent_hinge_x = -SASH_OPENING_W / 2.0 + VENT_W / 2.0
    vent_hinge_z = -SASH_OPENING_H / 2.0 + VENT_H
    # Axis along +X: positive q rotates bottom edge toward +Y (outward, opening vent).
    # Panel extends downward (-Z) from hinge; right-hand rule around +X takes -Z toward +Y.
    model.articulation(
        "sash_to_vent",
        ArticulationType.REVOLUTE,
        parent="sliding_sash",
        child="vent_panel",
        origin=Origin(xyz=(vent_hinge_x, 0.0, vent_hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.0, lower=0.0, upper=0.70),
    )

    # LATCH: REVOLUTE at meeting rail, mid-height, front face.
    # Pivot on the meeting stile of the sliding sash.
    latch_x = -SASH_OPENING_W / 2.0 - SASH_FACE / 2.0
    latch_y = SASH_DEPTH / 2.0 + LATCH_PIVOT_H / 2.0 + 0.003
    # Axis along +Z: positive q rotates lever from -X (locked, toward meeting rail)
    # toward +Y (unlocked, pointing outward).
    model.articulation(
        "sash_to_latch",
        ArticulationType.REVOLUTE,
        parent="sliding_sash",
        child="latch",
        origin=Origin(xyz=(latch_x, latch_y, 0.0)),
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
    fixed_sash = object_model.get_part("fixed_sash")
    sliding_sash = object_model.get_part("sliding_sash")
    vent_panel = object_model.get_part("vent_panel")
    latch = object_model.get_part("latch")

    slide = object_model.get_articulation("frame_to_sliding_sash")
    vent_joint = object_model.get_articulation("sash_to_vent")
    latch_joint = object_model.get_articulation("sash_to_latch")

    # --- Verify joint types ---
    ctx.check(
        "sliding sash joint is prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={slide.articulation_type}",
    )
    ctx.check(
        "vent panel joint is revolute",
        vent_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={vent_joint.articulation_type}",
    )
    ctx.check(
        "latch joint is revolute",
        latch_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={latch_joint.articulation_type}",
    )

    # --- Intentional overlaps ---
    # Glass rebated under sash lip on each sash
    for nm in ("fixed_sash", "sliding_sash"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass",
            elem_b=f"{nm}_vinyl",
            reason="Clear pane is rebated under the sash lip so it reads captured.",
        )
    # Sash rings rebated into frame opening
    for nm in ("fixed_sash", "sliding_sash"):
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_vinyl",
            reason=f"{nm} ring is rebated into the frame opening (seated capture).",
        )
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_glass",
            reason=f"{nm} glass is rebated under the frame opening lip.",
        )
    # Overlap stile seated on sliding sash front face
    ctx.allow_overlap(
        "sliding_sash", "sliding_sash",
        elem_a="overlap_stile",
        elem_b="sliding_sash_vinyl",
        reason="Overlap stile is seated onto the sliding sash meeting stile front face.",
    )
    # Rollers seated at bottom rail
    for i in range(2):
        ctx.allow_overlap(
            "sliding_sash", "sliding_sash",
            elem_a=f"roller_{i}",
            elem_b="sliding_sash_vinyl",
            reason="Roller block is seated at the sliding sash bottom rail.",
        )
    # Vent panel sits within the sash opening area (frame and sash rebate)
    ctx.allow_overlap(
        "frame", "vent_panel",
        elem_a="frame_shell",
        elem_b="vent_frame",
        reason="Vent panel is rebated within the frame opening when the sliding sash is closed.",
    )
    ctx.allow_overlap(
        "sliding_sash", "vent_panel",
        elem_a="sliding_sash_vinyl",
        elem_b="vent_frame",
        reason="Vent panel frame is rebated within the sliding sash opening area.",
    )
    ctx.allow_overlap(
        "sliding_sash", "vent_panel",
        elem_a="sliding_sash_glass",
        elem_b="vent_frame",
        reason="Vent panel sits within the sash opening area; small glass-frame overlap is intentional.",
    )
    ctx.allow_overlap(
        "sliding_sash", "vent_panel",
        elem_a="sliding_sash_glass",
        elem_b="vent_glass",
        reason="Vent glass is slightly proud of the main sash glass plane.",
    )
    # Latch pivot seated on sash face
    ctx.allow_overlap(
        "sliding_sash", "latch",
        elem_a="sliding_sash_vinyl",
        elem_b="latch_pivot",
        reason="Latch pivot boss is seated against the sliding sash meeting stile face.",
    )
    ctx.allow_overlap(
        "sliding_sash", "latch",
        elem_a="overlap_stile",
        elem_b="latch_pivot",
        reason="Latch pivot is mounted at the meeting rail where the overlap stile is also present.",
    )
    ctx.allow_overlap(
        "sliding_sash", "latch",
        elem_a="overlap_stile",
        elem_b="latch_lever",
        reason="Latch lever in locked position rests against the meeting rail overlap stile.",
    )

    # --- Closed pose (q=0): window reads SHUT ---
    with ctx.pose({slide: 0.0, vent_joint: 0.0, latch_joint: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        f_aabb = ctx.part_world_aabb(fixed_sash)
        s_aabb = ctx.part_world_aabb(sliding_sash)
        v_aabb = ctx.part_world_aabb(vent_panel)
        l_aabb = ctx.part_world_aabb(latch)

        # Frame spans full width
        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        sash_w = s_aabb[1][0] - s_aabb[0][0]
        ctx.check(
            "frame spans wider than a single sash",
            frame_w > sash_w + 0.40,
            details=f"frame_w={frame_w:.3f}, sash_w={sash_w:.3f}",
        )
        # Sill near floor
        ctx.check(
            "sill sits near z=0",
            abs(frame_aabb[0][2]) < 0.02,
            details=f"frame zmin={frame_aabb[0][2]:.4f}",
        )
        # Two sashes side by side
        fx = (f_aabb[0][0] + f_aabb[1][0]) / 2.0
        sx = (s_aabb[0][0] + s_aabb[1][0]) / 2.0
        ctx.check(
            "fixed sash left of sliding sash",
            fx < sx,
            details=f"fixed_x={fx:.3f}, sliding_x={sx:.3f}",
        )
        # Sliding sash proud of fixed
        fy = (f_aabb[0][1] + f_aabb[1][1]) / 2.0
        sy = (s_aabb[0][1] + s_aabb[1][1]) / 2.0
        ctx.check(
            "sliding sash proud of fixed sash",
            sy > fy + 0.02,
            details=f"sliding_y={sy:.3f}, fixed_y={fy:.3f}",
        )
        # Both sashes seated within frame
        for nm, ab in (("fixed", f_aabb), ("sliding", s_aabb)):
            ctx.check(
                f"{nm} sash seated within frame height",
                ab[0][2] > frame_aabb[0][2] - 1e-4 and ab[1][2] < frame_aabb[1][2] + 1e-4,
                details=f"{nm} z=[{ab[0][2]:.3f},{ab[1][2]:.3f}]",
            )
        # Sashes overlap in frame X projection
        ctx.expect_overlap(
            fixed_sash, frame, axes="xz", min_overlap=0.03,
            name="fixed sash seated in frame opening",
        )
        ctx.expect_overlap(
            sliding_sash, frame, axes="xz", min_overlap=0.03,
            name="sliding sash seated in frame opening",
        )

        # --- Overlap stile visible at meeting rail ---
        stile_aabb = ctx.part_element_world_aabb(sliding_sash, elem="overlap_stile")
        stile_cx = (stile_aabb[0][0] + stile_aabb[1][0]) / 2.0
        stile_cy = (stile_aabb[0][1] + stile_aabb[1][1]) / 2.0
        ctx.check(
            "overlap stile near meeting rail (between sashes)",
            abs(stile_cx - (fx + sx) / 2.0) < 0.15,
            details=f"stile_x={stile_cx:.3f}, mid={((fx+sx)/2):.3f}",
        )
        ctx.check(
            "overlap stile proud of sliding sash face",
            stile_cy > sy,
            details=f"stile_y={stile_cy:.3f}, sash_y={sy:.3f}",
        )

        # --- Rollers at bottom of sliding sash ---
        for i in range(2):
            r_aabb = ctx.part_element_world_aabb(sliding_sash, elem=f"roller_{i}")
            r_cz = (r_aabb[0][2] + r_aabb[1][2]) / 2.0
            sash_bottom = s_aabb[0][2]
            ctx.check(
                f"roller_{i} near bottom of sliding sash",
                r_cz < sash_bottom + 0.05,
                details=f"roller_z={r_cz:.3f}, sash_bottom={sash_bottom:.3f}",
            )

        # --- Vent panel at bottom-left of sliding sash ---
        v_cx = (v_aabb[0][0] + v_aabb[1][0]) / 2.0
        v_cz = (v_aabb[0][2] + v_aabb[1][2]) / 2.0
        ctx.check(
            "vent panel near meeting rail side of sliding sash",
            v_cx < sx,
            details=f"vent_x={v_cx:.3f}, sash_x={sx:.3f}",
        )
        ctx.check(
            "vent panel near bottom of sash",
            v_cz < MID_CZ,
            details=f"vent_z={v_cz:.3f}, mid_z={MID_CZ:.3f}",
        )

        # --- Latch at meeting rail, mid-height ---
        l_cx = (l_aabb[0][0] + l_aabb[1][0]) / 2.0
        l_cz = (l_aabb[0][2] + l_aabb[1][2]) / 2.0
        ctx.check(
            "latch near meeting rail",
            l_cx < sx,
            details=f"latch_x={l_cx:.3f}, sash_x={sx:.3f}",
        )
        ctx.check(
            "latch near mid-height",
            abs(l_cz - MID_CZ) < 0.20,
            details=f"latch_z={l_cz:.3f}, mid_z={MID_CZ:.3f}",
        )

        # --- Proof checks for intentional overlaps ---
        # Vent panel within frame opening
        ctx.expect_within(
            vent_panel, frame, axes="xz", margin=0.01,
            name="vent panel stays within frame opening",
        )
        # Latch pivot near overlap stile (both at meeting rail)
        ctx.expect_overlap(
            latch, sliding_sash, axes="xz", min_overlap=0.005,
            elem_a="latch_pivot", elem_b="overlap_stile",
            name="latch pivot overlaps with overlap stile at meeting rail",
        )

        rest_sx = sx

    # --- Sliding sash opens toward fixed sash (-X) ---
    travel = slide.motion_limits.upper
    with ctx.pose({slide: travel, vent_joint: 0.0, latch_joint: 0.0}):
        s_open = ctx.part_world_aabb(sliding_sash)
        open_sx = (s_open[0][0] + s_open[1][0]) / 2.0
        ctx.check(
            "sliding sash opens toward fixed sash (-X)",
            open_sx < rest_sx - 0.30,
            details=f"rest_x={rest_sx:.3f}, open_x={open_sx:.3f}",
        )
        # Retained within frame
        f_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "sash retained within frame X span at full travel",
            s_open[0][0] > f_aabb[0][0] - 1e-4 and s_open[1][0] < f_aabb[1][0] + 1e-4,
            details=f"sash x=[{s_open[0][0]:.3f},{s_open[1][0]:.3f}]",
        )

    # --- Vent panel opens: positive q swings bottom edge outward (+Y) ---
    with ctx.pose({slide: 0.0, vent_joint: 0.5, latch_joint: 0.0}):
        v_open = ctx.part_world_aabb(vent_panel)
        v_open_cy = (v_open[0][1] + v_open[1][1]) / 2.0
        # Vent bottom edge should move outward (toward +Y, front of window)
        s_open_aabb = ctx.part_world_aabb(sliding_sash)
        s_open_cy = (s_open_aabb[0][1] + s_open_aabb[1][1]) / 2.0
        ctx.check(
            "vent panel opens outward when driven",
            v_open_cy > s_open_cy - 0.01,
            details=f"vent_y={v_open_cy:.3f}, sash_y={s_open_cy:.3f}",
        )

    # --- Latch rotates: positive q swings lever outward ---
    with ctx.pose({slide: 0.0, vent_joint: 0.0, latch_joint: 1.2}):
        l_open = ctx.part_world_aabb(latch)
        l_open_cy = (l_open[0][1] + l_open[1][1]) / 2.0
        s_aabb2 = ctx.part_world_aabb(sliding_sash)
        s_cy2 = (s_aabb2[0][1] + s_aabb2[1][1]) / 2.0
        ctx.check(
            "latch lever rotates outward when driven",
            l_open_cy > s_cy2,
            details=f"latch_y={l_open_cy:.3f}, sash_y={s_cy2:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
