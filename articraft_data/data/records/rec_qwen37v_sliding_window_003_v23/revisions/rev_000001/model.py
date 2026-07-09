from __future__ import annotations

# Variant 23: Single-hung vertical sliding window — one movable lower sash,
# fixed upper sash, tilt-in latch pair on revolute joints, deep track grooves
# on head/sill/jambs, recessed pull cup on the lower sash bottom rail.
#
# Coordinate convention (same as parent):
#   +Z is up. Window stands vertically: height along +Z, width along X, frame
#   depth along Y. Sill at z=0, head at z=WIN_H.
#
# Articulation:
#   - UPPER sash: FIXED (stationary in the head track).
#   - LOWER sash: PRISMATIC axis (0,0,1): positive q slides it UP (opens).
#   - LEFT latch: REVOLUTE on lower sash left stile, axis Y, pivots inward.
#   - RIGHT latch: REVOLUTE on lower sash right stile, axis Y, pivots inward.

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

WIN_W = 0.92          # overall window width (X)
WIN_H = 1.52          # overall window height (Z), sill at z=0
FRAME_FACE = 0.060    # outer frame member face width (X/Z)
FRAME_DEPTH = 0.110   # outer frame jamb depth (Y)

# Clear opening inside the outer frame
OPEN_W = WIN_W - 2 * FRAME_FACE
OPEN_H = WIN_H - 2 * FRAME_FACE
OPEN_X0 = -WIN_W / 2.0 + FRAME_FACE
OPEN_X1 = WIN_W / 2.0 - FRAME_FACE
OPEN_Z0 = FRAME_FACE
OPEN_Z1 = WIN_H - FRAME_FACE

# Sash geometry
SASH_W = OPEN_W + 0.016      # stiles extend into jamb grooves (tongue-and-groove fit)
SASH_RAIL = 0.052
SASH_DEPTH = 0.034
SASH_H = OPEN_H * 0.535      # each sash ~53.5% of opening for meeting rail overlap
GLASS_T = 0.006

# Y planes — upper sash slightly exterior, lower sash slightly interior
# Faces touch at Y=0 for meeting rail contact
SASH_Y_GAP = 0.017
LOWER_SASH_Y = -SASH_Y_GAP
UPPER_SASH_Y = +SASH_Y_GAP

# Sash positions (world Z of bottom edge)
# Lower sash enters sill groove; upper sash positioned so meeting rails align
LOWER_BOTTOM_Z = OPEN_Z0 - 0.006
UPPER_BOTTOM_Z = LOWER_BOTTOM_Z + SASH_H - SASH_RAIL  # meeting rails overlap by one rail

# Muntin grid: 3 columns x 2 rows
MUNTIN_W = 0.022
N_COLS = 3
N_ROWS = 2

# Deep track grooves (head, sill, jambs)
TRACK_GROOVE_W = 0.038   # groove width in Y (wider than sash for clearance)
TRACK_GROOVE_DEPTH = 0.048  # how far the groove cuts into the frame member (deep tracks)
JAMB_GROOVE_X = 0.020   # how far jamb grooves reach into the jamb (X)

# Tilt-in latches
LATCH_W = 0.028   # tab length along X (protrusion into track)
LATCH_H = 0.032   # tab height along Z
LATCH_D = 0.012   # tab depth along Y

# Pull cup on lower sash bottom rail
CUP_DIAMETER = 0.042
CUP_DEPTH = 0.010

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

FRAME_RGBA = (0.945, 0.945, 0.945, 1.0)   # white painted frame
SASH_RGBA = (0.965, 0.965, 0.965, 1.0)    # white sash
GLASS_RGBA = (0.30, 0.36, 0.42, 0.34)     # cool dark-tinted glass
LATCH_RGBA = (0.78, 0.80, 0.82, 1.0)      # brushed metal latch
CUP_RGBA = (0.88, 0.88, 0.90, 1.0)        # slightly darker recess


# ---------------------------------------------------------------------------
# Frame geometry with deep track grooves
# ---------------------------------------------------------------------------

def _build_frame_shape() -> cq.Workplane:
    """White outer frame with deep track grooves on all four rails.

    Grooves:
    - Two vertical channels per jamb (one per sash Y-plane)
    - Horizontal channel in the head rail (upper sash track)
    - Horizontal channel in the sill rail (lower sash track)
    """
    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, WIN_H / 2.0))
        .box(WIN_W, FRAME_DEPTH, WIN_H)
    )

    # Cut the clear central opening
    opening = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, (OPEN_Z0 + OPEN_Z1) / 2.0))
        .box(OPEN_W, FRAME_DEPTH + 0.02, OPEN_H)
    )
    frame = outer.cut(opening)

    # --- Side jamb grooves (two per jamb, one per sash track) ---
    for sign, edge_x in ((+1.0, OPEN_X0), (-1.0, OPEN_X1)):
        cx = edge_x - sign * JAMB_GROOVE_X / 2.0
        for track_y in (LOWER_SASH_Y, UPPER_SASH_Y):
            groove = (
                cq.Workplane("XY")
                .transformed(offset=(cx, track_y, (OPEN_Z0 + OPEN_Z1) / 2.0))
                .box(JAMB_GROOVE_X, TRACK_GROOVE_W, OPEN_H)
            )
            frame = frame.cut(groove)

    # --- Head rail groove (horizontal channel for upper sash track) ---
    # Cuts upward from the opening top edge into the head member.
    head_groove_z = OPEN_Z1 + TRACK_GROOVE_DEPTH / 2.0
    head_groove = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, UPPER_SASH_Y, head_groove_z))
        .box(OPEN_W, TRACK_GROOVE_W, TRACK_GROOVE_DEPTH)
    )
    frame = frame.cut(head_groove)

    # --- Sill rail groove (horizontal channel for lower sash track) ---
    # Cuts downward from the opening bottom edge into the sill member.
    sill_groove_z = OPEN_Z0 - TRACK_GROOVE_DEPTH / 2.0
    sill_groove = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, LOWER_SASH_Y, sill_groove_z))
        .box(OPEN_W, TRACK_GROOVE_W, TRACK_GROOVE_DEPTH)
    )
    frame = frame.cut(sill_groove)

    return frame


# ---------------------------------------------------------------------------
# Sash frame geometry (CadQuery): perimeter ring + 6-lite muntin grid
# ---------------------------------------------------------------------------

def _build_sash_frame_shape() -> cq.Workplane:
    """One sash: perimeter ring plus a 3x2 muntin grid.

    Local frame: X in [-SASH_W/2, +SASH_W/2], Z in [0, SASH_H], Y centered at 0.
    """
    w = SASH_W
    h = SASH_H
    r = SASH_RAIL
    d = SASH_DEPTH

    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, h / 2.0))
        .box(w, d, h)
    )

    in_x0, in_x1 = -w / 2.0 + r, w / 2.0 - r
    in_z0, in_z1 = r, h - r
    inner_w = in_x1 - in_x0
    inner_h = in_z1 - in_z0

    col_lines = [in_x0 + (i + 1) * inner_w / N_COLS for i in range(N_COLS - 1)]
    row_lines = [in_z0 + (j + 1) * inner_h / N_ROWS for j in range(N_ROWS - 1)]

    x_edges = [in_x0] + col_lines + [in_x1]
    z_edges = [in_z0] + row_lines + [in_z1]
    half_m = MUNTIN_W / 2.0
    sash = outer
    for ci in range(N_COLS):
        for ri in range(N_ROWS):
            lx0 = x_edges[ci] + (half_m if ci > 0 else 0.0)
            lx1 = x_edges[ci + 1] - (half_m if ci < N_COLS - 1 else 0.0)
            lz0 = z_edges[ri] + (half_m if ri > 0 else 0.0)
            lz1 = z_edges[ri + 1] - (half_m if ri < N_ROWS - 1 else 0.0)
            lite = (
                cq.Workplane("XY")
                .transformed(offset=((lx0 + lx1) / 2.0, 0.0, (lz0 + lz1) / 2.0))
                .box(lx1 - lx0, d + 0.02, lz1 - lz0)
            )
            sash = sash.cut(lite)

    return sash


def _build_sash_glass_shape() -> cq.Workplane:
    """Six thin glass panes filling the lite openings."""
    w = SASH_W
    h = SASH_H
    r = SASH_RAIL
    rebate = 0.005
    d = SASH_DEPTH

    in_x0, in_x1 = -w / 2.0 + r, w / 2.0 - r
    in_z0, in_z1 = r, h - r
    inner_w = in_x1 - in_x0
    inner_h = in_z1 - in_z0
    col_lines = [in_x0 + (i + 1) * inner_w / N_COLS for i in range(N_COLS - 1)]
    row_lines = [in_z0 + (j + 1) * inner_h / N_ROWS for j in range(N_ROWS - 1)]
    x_edges = [in_x0] + col_lines + [in_x1]
    z_edges = [in_z0] + row_lines + [in_z1]
    half_m = MUNTIN_W / 2.0

    panes = None
    for ci in range(N_COLS):
        for ri in range(N_ROWS):
            lx0 = x_edges[ci] + (half_m if ci > 0 else 0.0) - rebate
            lx1 = x_edges[ci + 1] - (half_m if ci < N_COLS - 1 else 0.0) + rebate
            lz0 = z_edges[ri] + (half_m if ri > 0 else 0.0) - rebate
            lz1 = z_edges[ri + 1] - (half_m if ri < N_ROWS - 1 else 0.0) + rebate
            pane = (
                cq.Workplane("XY")
                .transformed(offset=((lx0 + lx1) / 2.0, 0.0, (lz0 + lz1) / 2.0))
                .box(lx1 - lx0, GLASS_T, lz1 - lz0)
            )
            panes = pane if panes is None else panes.union(pane)
    return panes


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hung_sliding_window")

    model.material("frame", rgba=FRAME_RGBA)
    model.material("sash", rgba=SASH_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("latch", rgba=LATCH_RGBA)
    model.material("cup", rgba=CUP_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material="frame",
        name="frame_shell",
    )

    # --- Fixed upper sash ---
    upper = model.part("upper_sash")
    upper.visual(
        mesh_from_cadquery(_build_sash_frame_shape(), "upper_sash_frame"),
        material="sash",
        name="upper_sash_frame",
    )
    upper.visual(
        mesh_from_cadquery(_build_sash_glass_shape(), "upper_sash_glass"),
        material="glass",
        name="upper_sash_glass",
    )

    # --- Movable lower sash ---
    lower = model.part("lower_sash")
    lower.visual(
        mesh_from_cadquery(_build_sash_frame_shape(), "lower_sash_frame"),
        material="sash",
        name="lower_sash_frame",
    )
    lower.visual(
        mesh_from_cadquery(_build_sash_glass_shape(), "lower_sash_glass"),
        material="glass",
        name="lower_sash_glass",
    )

    # Pull cup visual: short cylinder recessed into the bottom rail interior face.
    # Oriented along Y axis (perpendicular to glass), centered in the recess.
    cup_y = -(SASH_DEPTH / 2.0 - CUP_DEPTH / 2.0)
    lower.visual(
        Cylinder(CUP_DIAMETER / 2.0, CUP_DEPTH),
        origin=Origin(xyz=(0.0, cup_y, SASH_RAIL / 2.0), rpy=(1.5708, 0.0, 0.0)),
        material="cup",
        name="pull_cup",
    )

    # --- Tilt-in latches ---
    # Left latch: on left stile, pivots around Y axis
    left_latch = model.part("left_latch")
    left_latch.visual(
        Box((LATCH_W, LATCH_D, LATCH_H)),
        origin=Origin(xyz=(-LATCH_W / 2.0, 0.0, 0.0)),
        material="latch",
        name="left_latch_tab",
    )

    # Right latch: on right stile, pivots around Y axis
    right_latch = model.part("right_latch")
    right_latch.visual(
        Box((LATCH_W, LATCH_D, LATCH_H)),
        origin=Origin(xyz=(LATCH_W / 2.0, 0.0, 0.0)),
        material="latch",
        name="right_latch_tab",
    )

    # ----- Articulations -----

    # Upper sash: FIXED (stationary in the head track)
    model.articulation(
        "frame_to_upper_sash",
        ArticulationType.FIXED,
        parent="frame",
        child="upper_sash",
        origin=Origin(xyz=(0.0, UPPER_SASH_Y, UPPER_BOTTOM_Z)),
    )

    # Lower sash: PRISMATIC, slides UP. axis (0,0,1), positive q opens.
    model.articulation(
        "frame_to_lower_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="lower_sash",
        origin=Origin(xyz=(0.0, LOWER_SASH_Y, LOWER_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=0.25, lower=0.0, upper=SASH_H * 0.42
        ),
    )

    # Left latch: REVOLUTE, pivots inward (tab swings from -X toward center)
    # Origin at the left stile edge, mid-height of sash, on the lower_sash.
    latch_z_local = SASH_H / 2.0
    left_latch_x = -SASH_W / 2.0
    model.articulation(
        "lower_sash_to_left_latch",
        ArticulationType.REVOLUTE,
        parent="lower_sash",
        child="left_latch",
        origin=Origin(xyz=(left_latch_x, 0.0, latch_z_local)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=0.0, upper=1.2
        ),
    )

    # Right latch: REVOLUTE, pivots inward (tab swings from +X toward center)
    right_latch_x = SASH_W / 2.0
    model.articulation(
        "lower_sash_to_right_latch",
        ArticulationType.REVOLUTE,
        parent="lower_sash",
        child="right_latch",
        origin=Origin(xyz=(right_latch_x, 0.0, latch_z_local)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=0.0, upper=1.2
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    lower = object_model.get_part("lower_sash")
    upper = object_model.get_part("upper_sash")
    left_latch = object_model.get_part("left_latch")
    right_latch = object_model.get_part("right_latch")

    j_lower = object_model.get_articulation("frame_to_lower_sash")
    j_upper = object_model.get_articulation("frame_to_upper_sash")
    j_left = object_model.get_articulation("lower_sash_to_left_latch")
    j_right = object_model.get_articulation("lower_sash_to_right_latch")

    # --- Intentional overlaps ---
    # Glass panes are rebated under sash rails/muntins (captured glass).
    for sash_name in ("lower_sash", "upper_sash"):
        ctx.allow_overlap(
            sash_name, sash_name,
            elem_a=f"{sash_name}_glass",
            elem_b=f"{sash_name}_frame",
            reason="Glass panes are rebated under the sash rails/muntins so they read as captured, not floating.",
        )

    # Sashes ride in jamb track grooves cut into the frame.
    ctx.allow_overlap(
        "frame", "lower_sash",
        reason="Lower sash stiles ride in the jamb track grooves (retained insertion).",
    )
    ctx.allow_overlap(
        "frame", "upper_sash",
        reason="Upper sash is seated in the head/jamb track grooves (fixed insertion).",
    )

    # Pull cup is seated into the bottom rail recess.
    ctx.allow_overlap(
        "lower_sash", "lower_sash",
        elem_a="pull_cup",
        elem_b="lower_sash_frame",
        reason="Pull cup is recessed into the lower sash bottom rail.",
    )

    # Latches are mounted on sash stiles and their tabs protrude into the
    # frame jamb track grooves (engaged with the track at rest).
    ctx.allow_overlap(
        "frame", "left_latch",
        elem_a="frame_shell",
        elem_b="left_latch_tab",
        reason="Left tilt latch tab protrudes into the frame jamb track groove when at rest.",
    )
    ctx.allow_overlap(
        "frame", "right_latch",
        elem_a="frame_shell",
        elem_b="right_latch_tab",
        reason="Right tilt latch tab protrudes into the frame jamb track groove when at rest.",
    )
    # Latches contact the lower sash stile surface (mounted).
    ctx.allow_overlap(
        "lower_sash", "left_latch",
        reason="Left tilt latch is mounted on the lower sash left stile.",
    )
    ctx.allow_overlap(
        "lower_sash", "right_latch",
        reason="Right tilt latch is mounted on the lower sash right stile.",
    )

    # --- Structural checks ---
    with ctx.pose({j_lower: 0.0}):
        f_aabb = ctx.part_world_aabb(frame)
        lo_aabb = ctx.part_world_aabb(lower)
        up_aabb = ctx.part_world_aabb(upper)

        # Frame is the widest/tallest element
        frame_w = f_aabb[1][0] - f_aabb[0][0]
        sash_w = lo_aabb[1][0] - lo_aabb[0][0]
        ctx.check(
            "frame spans wider than a sash",
            frame_w > sash_w + 0.05,
            details=f"frame_w={frame_w:.3f}, sash_w={sash_w:.3f}",
        )
        # Sill near z=0, head above 1m
        ctx.check(
            "frame sill near z=0",
            abs(f_aabb[0][2]) < 0.01 and f_aabb[1][2] > 1.0,
            details=f"frame z=({f_aabb[0][2]:.3f}, {f_aabb[1][2]:.3f})",
        )
        # Lower sash below upper sash (stacked)
        lo_cz = (lo_aabb[0][2] + lo_aabb[1][2]) / 2.0
        up_cz = (up_aabb[0][2] + up_aabb[1][2]) / 2.0
        ctx.check(
            "lower sash below upper sash",
            lo_cz < up_cz - 0.2,
            details=f"lower_cz={lo_cz:.3f}, upper_cz={up_cz:.3f}",
        )
        # Upper sash is fixed near the head of the opening
        ctx.check(
            "upper sash seated near head",
            up_aabb[1][2] > OPEN_Z1 - 0.05,
            details=f"upper_top={up_aabb[1][2]:.3f}, OPEN_Z1={OPEN_Z1:.3f}",
        )
        # Lower sash rests near sill
        ctx.check(
            "lower sash rests near sill",
            lo_aabb[0][2] < OPEN_Z0 + 0.02,
            details=f"lower_bottom={lo_aabb[0][2]:.3f}, OPEN_Z0={OPEN_Z0:.3f}",
        )

        rest_lo_cz = lo_cz

    # --- Lower sash slides UP (opens) ---
    travel = SASH_H * 0.40
    with ctx.pose({j_lower: travel}):
        op = ctx.part_world_aabb(lower)
        op_cz = (op[0][2] + op[1][2]) / 2.0
        ctx.check(
            "lower sash slides up when opened",
            op_cz > rest_lo_cz + travel * 0.8,
            details=f"rest_cz={rest_lo_cz:.3f}, opened_cz={op_cz:.3f}, travel={travel:.3f}",
        )
        # Retained in frame
        ctx.expect_overlap(
            lower, frame, axes="x", min_overlap=0.05,
            name="lower sash retained in frame when open",
        )

    # --- Tilt latches pivot inward ---
    with ctx.pose({j_left: 0.8, j_right: 0.8}):
        ll_aabb = ctx.part_world_aabb(left_latch)
        rl_aabb = ctx.part_world_aabb(right_latch)
        # Both latches should still be near the sash (not far away)
        lo_aabb = ctx.part_world_aabb(lower)
        ctx.check(
            "left latch stays near lower sash when pivoted",
            ll_aabb[0][0] > lo_aabb[0][0] - 0.05 and ll_aabb[1][0] < lo_aabb[1][0] + 0.05,
            details=f"latch_x=({ll_aabb[0][0]:.3f},{ll_aabb[1][0]:.3f}), sash_x=({lo_aabb[0][0]:.3f},{lo_aabb[1][0]:.3f})",
        )
        ctx.check(
            "right latch stays near lower sash when pivoted",
            rl_aabb[0][0] > lo_aabb[0][0] - 0.05 and rl_aabb[1][0] < lo_aabb[1][0] + 0.05,
            details=f"latch_x=({rl_aabb[0][0]:.3f},{rl_aabb[1][0]:.3f}), sash_x=({lo_aabb[0][0]:.3f},{lo_aabb[1][0]:.3f})",
        )

    # --- Pull cup is on the lower sash ---
    cup_aabb = ctx.part_element_world_aabb(lower, elem="pull_cup")
    if cup_aabb is not None:
        cup_cz = (cup_aabb[0][2] + cup_aabb[1][2]) / 2.0
        lo_aabb = ctx.part_world_aabb(lower)
        # Cup should be near the bottom of the sash (on the bottom rail)
        ctx.check(
            "pull cup on lower sash bottom rail",
            cup_cz < lo_aabb[0][2] + SASH_RAIL * 1.5,
            details=f"cup_cz={cup_cz:.3f}, sash_bottom={lo_aabb[0][2]:.3f}",
        )
        # Cup is centered in X
        cup_cx = (cup_aabb[0][0] + cup_aabb[1][0]) / 2.0
        ctx.check(
            "pull cup centered on sash",
            abs(cup_cx) < 0.05,
            details=f"cup_cx={cup_cx:.3f}",
        )

    # --- Non-fixed joints exist ---
    ctx.check(
        "lower sash has prismatic joint",
        j_lower is not None,
        details="frame_to_lower_sash articulation must exist",
    )
    ctx.check(
        "left latch has revolute joint",
        j_left is not None,
        details="lower_sash_to_left_latch articulation must exist",
    )
    ctx.check(
        "right latch has revolute joint",
        j_right is not None,
        details="lower_sash_to_right_latch articulation must exist",
    )

    # --- Latches engage frame tracks at rest (proof for overlap allowance) ---
    with ctx.pose({j_lower: 0.0, j_left: 0.0, j_right: 0.0}):
        ctx.expect_overlap(
            left_latch, frame, axes="xy", min_overlap=0.005,
            name="left latch engages frame track at rest",
        )
        ctx.expect_overlap(
            right_latch, frame, axes="xy", min_overlap=0.005,
            name="right latch engages frame track at rest",
        )

    return ctx.report()


object_model = build_object_model()
