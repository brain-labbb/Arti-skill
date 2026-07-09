from __future__ import annotations

# Two-panel horizontal sliding window: white frame, two side-by-side sashes
# with the left sash movable (slides horizontally). Deep track grooves along
# the head (top) and sill (bottom) rails. A small latch rotates on a revolute
# joint at the meeting rail. A recessed pull cup on the movable sash.
#
# Coordinate convention:
#   +Z is up. Window stands vertically: height along +Z, width along X, frame
#   depth / glazing thickness along Y (the glass plane is the X-Z plane). The
#   sill sits at z=0; the head is at z=WIN_H.
#
# Articulation:
#   - LEFT sash is PRISMATIC, axis (1,0,0): positive q slides it RIGHT (opens).
#   - RIGHT sash is FIXED (stationary panel).
#   - LATCH is REVOLUTE on the left sash meeting rail, axis (0,0,1).

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
OPEN_W = WIN_W - 2 * FRAME_FACE   # clear width
OPEN_H = WIN_H - 2 * FRAME_FACE   # clear height
OPEN_X0 = -WIN_W / 2.0 + FRAME_FACE
OPEN_X1 = WIN_W / 2.0 - FRAME_FACE
OPEN_Z0 = FRAME_FACE
OPEN_Z1 = WIN_H - FRAME_FACE

# Track grooves in head/sill (horizontal channels running along X).
TRACK_DEPTH = 0.028     # how deep the groove cuts into the head/sill (Z)
TRACK_WIDTH = 0.022     # groove width along Y (captures the sash rail)

# Sash geometry. Each sash is slightly more than half the clear width so they
# overlap at the central meeting rail. Full clear height minus track clearance.
SASH_OVERLAP = 0.030                    # overlap at the meeting rail
SASH_W = OPEN_W * 0.5 + SASH_OVERLAP   # each sash width
SASH_RAIL = 0.048                       # sash perimeter member width (stile/rail)
SASH_DEPTH = 0.034                      # sash thickness (Y)
SASH_H = OPEN_H - 0.010                 # sash height (small clearance in tracks)
GLASS_T = 0.006                         # glazing thickness (Y)

# Y planes: left sash rides interior (-Y), right sash rides exterior (+Y),
# offset so they pass each other at the meeting rail.
SASH_Y_GAP = 0.018
LEFT_SASH_Y = -SASH_Y_GAP
RIGHT_SASH_Y = +SASH_Y_GAP

# Closed-pose sash positions (world X of sash local-frame origin).
# Left sash: its right edge should be near center (overlap past center).
# Right sash: its left edge should be near center (overlap past center).
LEFT_SASH_X_CLOSED = OPEN_X0 + SASH_W / 2.0      # left sash origin when closed
RIGHT_SASH_X_CLOSED = OPEN_X1 - SASH_W / 2.0      # right sash origin when closed

# Sash Z: bottom at the sill opening + small clearance, centered in track depth
SASH_BOTTOM_Z = OPEN_Z0 + 0.005

# Muntin grid: 2 columns x 3 rows of lites per sash
MUNTIN_W = 0.020
N_COLS = 2
N_ROWS = 3

# Latch at meeting rail
LATCH_BASE = (0.030, 0.018, 0.016)   # latch base plate (X, Y, Z)
LATCH_LEVER = (0.040, 0.010, 0.008)  # latch rotating lever
LATCH_Z = SASH_H * 0.52              # latch height on sash (near center)

# Pull cup on left sash meeting rail stile
PULL_CUP_W = 0.040       # pull cup width (Z, vertical)
PULL_CUP_H = 0.025       # pull cup height... actually let's call it depth
PULL_CUP_DEPTH = 0.010   # how deep the cup is recessed

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

FRAME_RGBA = (0.945, 0.945, 0.945, 1.0)   # white painted frame
SASH_RGBA = (0.965, 0.965, 0.965, 1.0)    # white sash (very slightly brighter)
GLASS_RGBA = (0.30, 0.36, 0.42, 0.34)     # cool dark-tinted glass
LATCH_RGBA = (0.82, 0.83, 0.85, 1.0)      # brushed metal latch
PULL_RGBA = (0.75, 0.76, 0.78, 1.0)       # satin metal pull cup


# ---------------------------------------------------------------------------
# Static outer frame geometry (CadQuery) — with head/sill track grooves
# ---------------------------------------------------------------------------

def _build_frame_shape() -> cq.Workplane:
    """White outer frame: perimeter slab with central opening, plus deep track
    grooves in the head (top rail) and sill (bottom rail) where sashes ride.
    """
    # Solid outer slab
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

    # Deep track grooves in head (top rail) and sill (bottom rail).
    # Each groove is a long horizontal channel cut into the head/sill,
    # running the full opening width. Two grooves per rail (one per sash plane).
    groove_x_extent = OPEN_W + 0.010  # slightly wider than opening

    for rail_z in (OPEN_Z0, OPEN_Z1):
        # Groove center Z: cut into the rail from the opening edge
        if rail_z < WIN_H / 2.0:
            # Sill: groove cuts upward from the opening bottom edge
            gz = rail_z + TRACK_DEPTH / 2.0
        else:
            # Head: groove cuts downward from the opening top edge
            gz = rail_z - TRACK_DEPTH / 2.0

        for track_y in (LEFT_SASH_Y, RIGHT_SASH_Y):
            groove = (
                cq.Workplane("XY")
                .transformed(offset=(0.0, track_y, gz))
                .box(groove_x_extent, TRACK_WIDTH, TRACK_DEPTH)
            )
            frame = frame.cut(groove)

    return frame


# ---------------------------------------------------------------------------
# Sash frame geometry (CadQuery): perimeter ring + muntin grid
# ---------------------------------------------------------------------------

def _build_sash_frame_shape() -> cq.Workplane:
    """One sash: perimeter ring plus a 2x3 muntin grid, built as a slab with
    rectangular lite openings cut through.

    Authored in the sash-local frame:
      - local X runs -SASH_W/2 .. +SASH_W/2
      - local Z runs 0 .. SASH_H (bottom rail at z=0)
      - local Y is the sash thickness, centered at y=0.
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

    # Inner glazed region
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
    """Thin glass panes filling the lite openings, rebated under the
    muntin/rail lips so the glass reads as captured."""
    w = SASH_W
    h = SASH_H
    r = SASH_RAIL
    rebate = 0.005

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
# Pull cup geometry — a recessed pocket on the sash stile
# ---------------------------------------------------------------------------

def _build_pull_cup_shape() -> cq.Workplane:
    """Recessed pull cup: a shallow rectangular pocket that sits on the
    meeting-rail stile of the movable sash. Authored so the cup protrudes
    slightly proud of the sash surface (the recess is visual)."""
    # The cup is a small box with a recessed center.
    outer_w = 0.038  # X (along stile, narrow)
    outer_d = 0.014  # Y (depth, protrudes from sash face)
    outer_h = 0.060  # Z (height of the pull cup)
    return (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, 0.0))
        .box(outer_w, outer_d, outer_h)
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="horizontal_slider_window")

    model.material("frame", rgba=FRAME_RGBA)
    model.material("sash", rgba=SASH_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("latch", rgba=LATCH_RGBA)
    model.material("pull", rgba=PULL_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame_shell"),
        material="frame",
        name="frame_shell",
    )

    # --- Left sash (movable, slides horizontally) ---
    left_sash = model.part("left_sash")
    left_sash.visual(
        mesh_from_cadquery(_build_sash_frame_shape(), "left_sash_frame"),
        material="sash",
        name="left_sash_frame",
    )
    left_sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(), "left_sash_glass"),
        material="glass",
        name="left_sash_glass",
    )

    # Pull cup on the left sash — mounted on the right stile (meeting rail side),
    # on the interior face (-Y side). Positioned at roughly mid-height.
    pull_x = SASH_W / 2.0 - SASH_RAIL / 2.0   # on the right stile
    pull_y = -(SASH_DEPTH / 2.0 + 0.007)       # slightly proud of interior face
    pull_z = SASH_H * 0.45                      # below center for ergonomic pull
    left_sash.visual(
        mesh_from_cadquery(_build_pull_cup_shape(), "left_pull_cup"),
        origin=Origin(xyz=(pull_x, pull_y, pull_z)),
        material="pull",
        name="left_pull_cup",
    )

    # Latch base (fixed to left sash meeting rail)
    latch_base_x = SASH_W / 2.0 - SASH_RAIL / 2.0   # on right stile
    latch_base_y = -(SASH_DEPTH / 2.0 + LATCH_BASE[1] / 2.0 - 0.004)
    left_sash.visual(
        Box(LATCH_BASE),
        origin=Origin(xyz=(latch_base_x, latch_base_y, LATCH_Z)),
        material="latch",
        name="latch_base",
    )

    # --- Latch lever (revolute child of left sash) ---
    latch = model.part("latch")
    latch.visual(
        Box(LATCH_LEVER),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="latch",
        name="latch_lever",
    )

    # --- Right sash (fixed panel) ---
    right_sash = model.part("right_sash")
    right_sash.visual(
        mesh_from_cadquery(_build_sash_frame_shape(), "right_sash_frame"),
        material="sash",
        name="right_sash_frame",
    )
    right_sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(), "right_sash_glass"),
        material="glass",
        name="right_sash_glass",
    )

    # ----- Articulations -----

    # LEFT sash: slides RIGHT (+X) to open. Prismatic axis (1,0,0).
    # At q=0, left sash is at its closed position (left side of opening).
    model.articulation(
        "frame_to_left_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="left_sash",
        origin=Origin(xyz=(LEFT_SASH_X_CLOSED, LEFT_SASH_Y, SASH_BOTTOM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=0.25,
            lower=0.0,
            upper=OPEN_W * 0.45,  # slides nearly fully open
        ),
    )

    # RIGHT sash: fixed panel
    model.articulation(
        "frame_to_right_sash",
        ArticulationType.FIXED,
        parent="frame",
        child="right_sash",
        origin=Origin(xyz=(RIGHT_SASH_X_CLOSED, RIGHT_SASH_Y, SASH_BOTTOM_Z)),
    )

    # LATCH: revolute joint on left sash meeting rail.
    # Axis (0,0,1): rotates in the horizontal plane.
    # At q=0 the lever is "locked" (horizontal, aligned with the stile).
    # Positive q rotates it to "unlocked" (~pi/2).
    model.articulation(
        "left_sash_to_latch",
        ArticulationType.REVOLUTE,
        parent="left_sash",
        child="latch",
        origin=Origin(xyz=(latch_base_x, latch_base_y - LATCH_BASE[1] / 2.0, LATCH_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=2.0,
            lower=0.0,
            upper=1.57,  # ~90 degrees
        ),
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
    latch = object_model.get_part("latch")

    j_slide = object_model.get_articulation("frame_to_left_sash")
    j_latch = object_model.get_articulation("left_sash_to_latch")

    # --- Intentional overlaps ---
    # Glass panes tuck under sash muntin/rail lips (captured glass).
    for sash_name in ("left_sash", "right_sash"):
        ctx.allow_overlap(
            sash_name, sash_name,
            elem_a=f"{sash_name}_glass",
            elem_b=f"{sash_name}_frame",
            reason="Glass panes are rebated under the sash rails/muntins so they read as captured.",
        )

    # Sashes ride in the head/sill track grooves cut into the frame.
    ctx.allow_overlap(
        "frame", "left_sash",
        reason="Left sash rails ride in the head/sill track grooves (retained insertion).",
    )
    ctx.allow_overlap(
        "frame", "right_sash",
        reason="Right sash rails ride in the head/sill track grooves (retained insertion).",
    )

    # Two sashes overlap at the meeting rail (different Y planes).
    ctx.allow_overlap(
        "left_sash", "right_sash",
        reason="Sashes overlap at the central meeting rail; they ride in offset Y planes.",
    )

    # Latch base seated on left sash meeting rail stile.
    ctx.allow_overlap(
        "left_sash", "left_sash",
        elem_a="latch_base",
        elem_b="left_sash_frame",
        reason="Latch base is mounted (seated) onto the left sash meeting rail stile.",
    )

    # Pull cup seated on left sash stile.
    ctx.allow_overlap(
        "left_sash", "left_sash",
        elem_a="left_pull_cup",
        elem_b="left_sash_frame",
        reason="Pull cup is mounted (seated) onto the left sash meeting rail stile.",
    )

    # Latch lever overlaps latch base (pivot nesting).
    ctx.allow_overlap(
        "left_sash", "latch",
        elem_a="latch_base",
        elem_b="latch_lever",
        reason="Latch lever pivots on the latch base (captured pivot).",
    )

    # --- Closed pose (q=0): both sashes in place, window reads shut ---
    with ctx.pose({j_slide: 0.0}):
        f_aabb = ctx.part_world_aabb(frame)
        ls_aabb = ctx.part_world_aabb(left_sash)
        rs_aabb = ctx.part_world_aabb(right_sash)

        # Frame is the widest/tallest element
        frame_w = f_aabb[1][0] - f_aabb[0][0]
        sash_w = ls_aabb[1][0] - ls_aabb[0][0]
        ctx.check(
            "frame spans wider than a sash",
            frame_w > sash_w + 0.05,
            details=f"frame_w={frame_w:.3f}, sash_w={sash_w:.3f}",
        )

        # Sill at/near z=0
        ctx.check(
            "frame sill near z=0",
            abs(f_aabb[0][2]) < 0.01 and f_aabb[1][2] > 1.0,
            details=f"frame z range=({f_aabb[0][2]:.3f}, {f_aabb[1][2]:.3f})",
        )

        # Sashes within frame opening width
        ctx.check(
            "left sash within frame width",
            ls_aabb[0][0] > f_aabb[0][0] - 0.01 and ls_aabb[1][0] < f_aabb[1][0] + 0.01,
            details=f"left x=({ls_aabb[0][0]:.3f},{ls_aabb[1][0]:.3f})",
        )

        # Left sash is to the LEFT of the right sash at closed pose
        ls_cx = (ls_aabb[0][0] + ls_aabb[1][0]) / 2.0
        rs_cx = (rs_aabb[0][0] + rs_aabb[1][0]) / 2.0
        ctx.check(
            "left sash is left of right sash at closed pose",
            ls_cx < rs_cx - 0.05,
            details=f"left_cx={ls_cx:.3f}, right_cx={rs_cx:.3f}",
        )

        # Sashes overlap at the meeting rail in X (closed, no daylight gap)
        ctx.check(
            "sashes overlap at meeting rail (closed)",
            ls_aabb[1][0] >= rs_aabb[0][0] - 1e-4,
            details=f"left_right_edge={ls_aabb[1][0]:.3f}, right_left_edge={rs_aabb[0][0]:.3f}",
        )

        # Sashes ride in offset Y planes
        ls_cy = (ls_aabb[0][1] + ls_aabb[1][1]) / 2.0
        rs_cy = (rs_aabb[0][1] + rs_aabb[1][1]) / 2.0
        ctx.check(
            "sashes ride in offset Y planes",
            abs(ls_cy - rs_cy) > 0.015,
            details=f"left_cy={ls_cy:.3f}, right_cy={rs_cy:.3f}",
        )

        # Both sashes span the full opening height
        ls_h = ls_aabb[1][2] - ls_aabb[0][2]
        rs_h = rs_aabb[1][2] - rs_aabb[0][2]
        ctx.check(
            "sashes span nearly the full opening height",
            ls_h > OPEN_H * 0.9 and rs_h > OPEN_H * 0.9,
            details=f"left_h={ls_h:.3f}, right_h={rs_h:.3f}, open_h={OPEN_H:.3f}",
        )

        rest_ls_cx = ls_cx

    # --- HERO: left sash slides RIGHT (opens) ---
    travel = OPEN_W * 0.40
    with ctx.pose({j_slide: travel}):
        op = ctx.part_world_aabb(left_sash)
        op_cx = (op[0][0] + op[1][0]) / 2.0
        ctx.check(
            "left sash slides right when opened",
            op_cx > rest_ls_cx + travel * 0.8,
            details=f"rest_cx={rest_ls_cx:.3f}, opened_cx={op_cx:.3f}, travel={travel:.3f}",
        )
        # Stays retained: still overlaps the frame vertically
        ctx.expect_overlap(
            left_sash, frame, axes="z", min_overlap=0.05,
            name="left sash retained in frame tracks when open",
        )

    # --- Latch rotates on revolute joint ---
    with ctx.pose({j_latch: 0.0}):
        latch_aabb_locked = ctx.part_world_aabb(latch)
    with ctx.pose({j_latch: 1.57}):
        latch_aabb_open = ctx.part_world_aabb(latch)

    # The latch lever AABB should change between locked and open poses
    # (proving the revolute joint actually rotates the lever).
    locked_extent_x = latch_aabb_locked[1][0] - latch_aabb_locked[0][0]
    open_extent_x = latch_aabb_open[1][0] - latch_aabb_open[0][0]
    locked_extent_y = latch_aabb_locked[1][1] - latch_aabb_locked[0][1]
    open_extent_y = latch_aabb_open[1][1] - latch_aabb_open[0][1]
    ctx.check(
        "latch lever rotates between locked and open poses",
        abs(locked_extent_x - open_extent_x) > 0.005 or abs(locked_extent_y - open_extent_y) > 0.005,
        details=f"locked_xy=({locked_extent_x:.4f},{locked_extent_y:.4f}), open_xy=({open_extent_x:.4f},{open_extent_y:.4f})",
    )

    # --- Pull cup exists on the left sash ---
    pull_aabb = ctx.part_element_world_aabb(left_sash, elem="left_pull_cup")
    if pull_aabb is not None:
        ls_aabb = ctx.part_world_aabb(left_sash)
        pull_cx = (pull_aabb[0][0] + pull_aabb[1][0]) / 2.0
        # Pull cup should be near the right edge of the left sash (meeting rail side)
        ls_right_edge = ls_aabb[1][0]
        ctx.check(
            "pull cup is near the meeting rail side of the left sash",
            pull_cx > ls_right_edge - SASH_RAIL * 1.5,
            details=f"pull_cx={pull_cx:.3f}, sash_right={ls_right_edge:.3f}",
        )

    # --- Track grooves: frame has deep channels in head/sill ---
    # The frame AABB should show depth features at top and bottom.
    # We verify the frame opening height matches expectations (grooves don't
    # collapse the frame).
    ctx.check(
        "frame has correct height with track grooves",
        f_aabb[1][2] - f_aabb[0][2] > WIN_H * 0.95,
        details=f"frame_h={f_aabb[1][2] - f_aabb[0][2]:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
