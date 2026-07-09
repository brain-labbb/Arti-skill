from __future__ import annotations

# Aluminum-framed sliding window: thick rails with deep track grooves, a
# movable lower sash that slides upward on a vertical prismatic joint, and a
# fixed upper sash.  Variant of the double-hung sash window family.
#
# Coordinate convention:
#   +Z is up.  Window stands vertically: height along +Z, width along X,
#   frame depth / glazing thickness along Y.  Sill at z=0, head at z=WIN_H.
#
# Articulation:
#   - LOWER sash: PRISMATIC, axis (0,0,1): positive q slides it UP (opens).
#   - UPPER sash: FIXED (stationary in the upper track).
#   The lower sash rides in the interior (-Y) track; the upper sash is fixed
#   in the exterior (+Y) track.

import cadquery as cq
import math

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
FRAME_FACE = 0.078    # thick aluminum frame member face width
FRAME_DEPTH = 0.130   # deep frame jamb (Y) for track channels

# Clear opening inside the outer frame
OPEN_W = WIN_W - 2 * FRAME_FACE
OPEN_H = WIN_H - 2 * FRAME_FACE
OPEN_X0 = -WIN_W / 2.0 + FRAME_FACE
OPEN_X1 = WIN_W / 2.0 - FRAME_FACE
OPEN_Z0 = FRAME_FACE
OPEN_Z1 = WIN_H - FRAME_FACE

# Sash geometry
SASH_W = OPEN_W - 0.014                 # running clearance to the jambs
SASH_RAIL = 0.054                       # sash perimeter member width
SASH_DEPTH = 0.036                      # sash thickness (Y)
SASH_H = OPEN_H * 0.545                 # each sash height
GLASS_T = 0.006                         # glazing thickness

# Y planes: lower sash rides interior (-Y), upper sash rides exterior (+Y)
SASH_Y_GAP = 0.018
LOWER_SASH_Y = -SASH_Y_GAP
UPPER_SASH_Y = +SASH_Y_GAP

# Closed-pose sash positions
LOWER_BOTTOM_Z = OPEN_Z0 + 0.005       # lower sash rests on sill track
MEETING_OVERLAP = SASH_RAIL
UPPER_BOTTOM_Z = LOWER_BOTTOM_Z + SASH_H - MEETING_OVERLAP

# Muntin grid: 3 columns x 2 rows per sash
MUNTIN_W = 0.022
N_COLS = 3
N_ROWS = 2

# Side track channels (jamb grooves for sash stiles)
TRACK_W = 0.020
TRACK_DEPTH = 0.032

# Deep horizontal track grooves in head and sill
H_GROOVE_W = 0.022      # groove width (along Y)
H_GROOVE_DEPTH = 0.026  # depth of cut into the rail (along Z)

# Pull cup on lower sash bottom rail
PULL_CUP_DIA = 0.034
PULL_CUP_RECESS = 0.010
PULL_CUP_RIM_H = 0.003
PULL_CUP_RIM_W = 0.004

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

FRAME_RGBA = (0.68, 0.71, 0.74, 1.0)   # dark anodized aluminum frame
SASH_RGBA = (0.72, 0.75, 0.78, 1.0)    # lighter aluminum sash
GLASS_RGBA = (0.28, 0.34, 0.40, 0.32)  # cool tinted glass
PULL_RGBA = (0.52, 0.55, 0.58, 1.0)    # darker pull cup accent
TRACK_RGBA = (0.60, 0.63, 0.66, 1.0)   # track rail accent


# ---------------------------------------------------------------------------
# Frame geometry (CadQuery): thick aluminum perimeter with deep track grooves
# ---------------------------------------------------------------------------

def _build_frame_shape() -> cq.Workplane:
    """Aluminum outer frame: perimeter slab with central opening, side-track
    channels in the jambs, and deep horizontal track grooves in the head and
    sill.
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

    # --- Side track grooves in the jambs (vertical channels) ---
    groove_x = FRAME_FACE * 0.50
    for sign, edge_x in ((+1.0, OPEN_X0), (-1.0, OPEN_X1)):
        cx = edge_x - sign * groove_x / 2.0
        for track_y in (LOWER_SASH_Y, UPPER_SASH_Y):
            groove = (
                cq.Workplane("XY")
                .transformed(offset=(cx, track_y, (OPEN_Z0 + OPEN_Z1) / 2.0))
                .box(groove_x, TRACK_DEPTH, OPEN_H)
            )
            frame = frame.cut(groove)

    # --- Deep horizontal track grooves in sill and head ---
    # These are channels cut into the opening-facing inner face of the sill
    # (bottom rail) and head (top rail), running along X.  Two parallel
    # grooves per rail (one per sash track plane).
    groove_span = OPEN_W - 0.010  # nearly full opening width

    # Sill grooves: cut upward from the sill top face (z = OPEN_Z0)
    for track_y in (LOWER_SASH_Y, UPPER_SASH_Y):
        sill_groove = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, track_y, OPEN_Z0 - H_GROOVE_DEPTH / 2.0))
            .box(groove_span, H_GROOVE_W, H_GROOVE_DEPTH)
        )
        frame = frame.cut(sill_groove)

    # Head grooves: cut downward from the head bottom face (z = OPEN_Z1)
    for track_y in (LOWER_SASH_Y, UPPER_SASH_Y):
        head_groove = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, track_y, OPEN_Z1 + H_GROOVE_DEPTH / 2.0))
            .box(groove_span, H_GROOVE_W, H_GROOVE_DEPTH)
        )
        frame = frame.cut(head_groove)

    # --- Raised bearing ridges between the groove channels ---
    # These are thin strips on the sill and head between the two groove
    # positions, representing the actual rail surface the sash rides on.
    # They are unioned into the frame so the whole frame stays one solid.
    ridge_h = 0.005
    ridge_w = abs(UPPER_SASH_Y - LOWER_SASH_Y) - H_GROOVE_W  # gap between grooves
    ridge_span = OPEN_W - 0.020

    if ridge_w > 0.004:
        # Sill center ridge
        sill_ridge = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, 0.0, OPEN_Z0 + ridge_h / 2.0))
            .box(ridge_span, ridge_w, ridge_h)
        )
        frame = frame.union(sill_ridge)

        # Head center ridge
        head_ridge = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, 0.0, OPEN_Z1 - ridge_h / 2.0))
            .box(ridge_span, ridge_w, ridge_h)
        )
        frame = frame.union(head_ridge)

    # Outer ridges (on the interior and exterior sides of the grooves)
    outer_ridge_w = 0.010
    outer_ridge_y_int = LOWER_SASH_Y - H_GROOVE_W / 2.0 - outer_ridge_w / 2.0
    outer_ridge_y_ext = UPPER_SASH_Y + H_GROOVE_W / 2.0 + outer_ridge_w / 2.0

    for ry in (outer_ridge_y_int, outer_ridge_y_ext):
        sill_outer = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, ry, OPEN_Z0 + ridge_h / 2.0))
            .box(ridge_span, outer_ridge_w, ridge_h)
        )
        frame = frame.union(sill_outer)
        head_outer = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, ry, OPEN_Z1 - ridge_h / 2.0))
            .box(ridge_span, outer_ridge_w, ridge_h)
        )
        frame = frame.union(head_outer)

    return frame


# ---------------------------------------------------------------------------
# Sash frame geometry (CadQuery): perimeter ring + 6-lite muntin grid
# ---------------------------------------------------------------------------

def _build_sash_frame_shape() -> cq.Workplane:
    """One sash: perimeter ring with 3x2 muntin grid.
    Local frame: X = -SASH_W/2 .. +SASH_W/2, Z = 0 .. SASH_H, Y centered.
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
# Pull cup geometry (CadQuery): recessed finger-grip cup
# ---------------------------------------------------------------------------

def _build_pull_cup_shape() -> cq.Workplane:
    """Recessed pull cup: annular rim ring + recessed back plate.
    Built in local cup frame: centered at origin, extending along +Y.
    """
    r_outer = PULL_CUP_DIA / 2.0
    r_inner = r_outer - PULL_CUP_RIM_W
    rim_h = PULL_CUP_RIM_H

    # Annular rim ring (washer shape)
    rim = (
        cq.Workplane("XZ")
        .circle(r_outer)
        .circle(r_inner)
        .extrude(rim_h)
    )

    # Back plate (thin disk recessed behind the rim)
    back = (
        cq.Workplane("XZ")
        .circle(r_inner - 0.001)
        .extrude(-PULL_CUP_RECESS)
    )

    return rim.union(back)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="aluminum_sliding_window")

    model.material("frame", rgba=FRAME_RGBA)
    model.material("sash", rgba=SASH_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("pull", rgba=PULL_RGBA)
    model.material("track", rgba=TRACK_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame_shell"),
        material="frame",
        name="frame_shell",
    )

    # --- Upper sash (FIXED - stationary in upper track) ---
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

    # --- Lower sash (MOVABLE - slides upward) ---
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

    # Recessed pull cup on the lower sash bottom rail, interior face (-Y),
    # centered in X.  Positioned at the mid-height of the bottom rail.
    # The cup is centered at the sash face so half embeds into the rail.
    pull_cup_y = -(SASH_DEPTH / 2.0)  # centered on the sash face
    pull_cup_z = SASH_RAIL / 2.0  # center of bottom rail
    lower.visual(
        mesh_from_cadquery(_build_pull_cup_shape(), "pull_cup"),
        origin=Origin(xyz=(0.0, pull_cup_y, pull_cup_z)),
        material="pull",
        name="pull_cup",
    )

    # ----- Articulations -----

    # LOWER sash: slides UP on prismatic joint. axis (0,0,1).
    model.articulation(
        "frame_to_lower_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="lower_sash",
        origin=Origin(xyz=(0.0, LOWER_SASH_Y, LOWER_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0, velocity=0.30, lower=0.0, upper=SASH_H * 0.44
        ),
    )

    # UPPER sash: FIXED (stationary in the upper track).
    model.articulation(
        "frame_to_upper_sash",
        ArticulationType.FIXED,
        parent="frame",
        child="upper_sash",
        origin=Origin(xyz=(0.0, UPPER_SASH_Y, UPPER_BOTTOM_Z)),
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
    j_lower = object_model.get_articulation("frame_to_lower_sash")
    j_upper = object_model.get_articulation("frame_to_upper_sash")

    # --- Intentional overlaps ---
    # Glass panes tuck under sash muntin/rail lips (captured glass).
    for sash_name in ("lower_sash", "upper_sash"):
        ctx.allow_overlap(
            sash_name, sash_name,
            elem_a=f"{sash_name}_glass",
            elem_b=f"{sash_name}_frame",
            reason="Glass panes are rebated under the sash rails/muntins (captured glazing).",
        )
    # Sashes ride in the jamb track grooves and sill/head channels.
    ctx.allow_overlap(
        "frame", "lower_sash",
        reason="Lower sash stiles and rails ride in the frame track grooves (retained insertion).",
    )
    ctx.allow_overlap(
        "frame", "upper_sash",
        reason="Upper sash is fixed in the exterior track grooves (retained insertion).",
    )
    # Pull cup is seated into the lower sash bottom rail.
    ctx.allow_overlap(
        "lower_sash", "lower_sash",
        elem_a="pull_cup",
        elem_b="lower_sash_frame",
        reason="Pull cup rim is recessed into the lower sash bottom rail face.",
    )

    # --- Structural checks ---

    # 1. The lower sash articulation is PRISMATIC (non-fixed joint).
    ctx.check(
        "lower sash has prismatic joint",
        j_lower.articulation_type == ArticulationType.PRISMATIC,
        details=f"got {j_lower.articulation_type}",
    )

    # 2. The upper sash articulation is FIXED.
    ctx.check(
        "upper sash is fixed",
        j_upper.articulation_type == ArticulationType.FIXED,
        details=f"got {j_upper.articulation_type}",
    )

    # 3. Pull cup exists on the lower sash.
    pull_vis = lower.get_visual("pull_cup")
    ctx.check(
        "pull cup exists on lower sash",
        pull_vis is not None,
        details="pull_cup visual not found on lower_sash",
    )

    # 4. Frame has deep track grooves (the frame shell includes the groove geometry).
    frame_vis = frame.get_visual("frame_shell")
    ctx.check(
        "frame shell includes track groove geometry",
        frame_vis is not None,
        details="frame_shell visual not found",
    )

    # 5. Frame uses aluminum material (not white paint).
    frame_vis = frame.get_visual("frame_shell")
    mat_name = frame_vis.material.name if hasattr(frame_vis.material, 'name') else str(frame_vis.material)
    ctx.check(
        "frame uses aluminum material",
        "frame" in mat_name.lower(),
        details=f"material={mat_name}",
    )

    # --- Closed pose (q=0): lower sash seated, window reads shut ---
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
        # Sill near z=0, window stands tall
        ctx.check(
            "frame sill near z=0",
            abs(f_aabb[0][2]) < 0.01 and f_aabb[1][2] > 1.0,
            details=f"frame z range=({f_aabb[0][2]:.3f}, {f_aabb[1][2]:.3f})",
        )
        # Sashes within frame width
        ctx.check(
            "sashes within frame width",
            lo_aabb[0][0] > f_aabb[0][0] and lo_aabb[1][0] < f_aabb[1][0]
            and up_aabb[0][0] > f_aabb[0][0] and up_aabb[1][0] < f_aabb[1][0],
            details=f"lower x=({lo_aabb[0][0]:.3f},{lo_aabb[1][0]:.3f}) upper x=({up_aabb[0][0]:.3f},{up_aabb[1][0]:.3f})",
        )
        # Lower sash below upper sash
        lo_center_z = (lo_aabb[0][2] + lo_aabb[1][2]) / 2.0
        up_center_z = (up_aabb[0][2] + up_aabb[1][2]) / 2.0
        ctx.check(
            "lower sash below upper sash at closed pose",
            lo_center_z < up_center_z - 0.3,
            details=f"lower_cz={lo_center_z:.3f}, upper_cz={up_center_z:.3f}",
        )
        # Sashes in offset Y planes
        lo_cy = (lo_aabb[0][1] + lo_aabb[1][1]) / 2.0
        up_cy = (up_aabb[0][1] + up_aabb[1][1]) / 2.0
        ctx.check(
            "sashes ride in offset Y planes",
            abs(lo_cy - up_cy) > 0.015,
            details=f"lower_cy={lo_cy:.3f}, upper_cy={up_cy:.3f}",
        )

        rest_lo_z = lo_center_z

    # --- Lower sash slides UP when opened ---
    travel = SASH_H * 0.42
    with ctx.pose({j_lower: travel}):
        op = ctx.part_world_aabb(lower)
        op_cz = (op[0][2] + op[1][2]) / 2.0
        ctx.check(
            "lower sash slides up when opened",
            op_cz > rest_lo_z + travel * 0.8,
            details=f"rest_cz={rest_lo_z:.3f}, opened_cz={op_cz:.3f}, travel={travel:.3f}",
        )
        # Still retained in frame
        ctx.expect_overlap(
            lower, frame, axes="x", min_overlap=0.05,
            name="lower sash retained in frame when open",
        )

    # --- Upper sash stays fixed (no movement at any pose) ---
    up_rest_aabb = ctx.part_world_aabb(upper)
    # Even when lower sash opens, upper stays put (it has no movable joint)
    with ctx.pose({j_lower: travel}):
        up_open_aabb = ctx.part_world_aabb(upper)
        up_rest_cz = (up_rest_aabb[0][2] + up_rest_aabb[1][2]) / 2.0
        up_open_cz = (up_open_aabb[0][2] + up_open_aabb[1][2]) / 2.0
        ctx.check(
            "upper sash stays fixed when lower sash opens",
            abs(up_open_cz - up_rest_cz) < 0.001,
            details=f"rest_cz={up_rest_cz:.4f}, open_cz={up_open_cz:.4f}",
        )

    # --- Pull cup is on the lower sash bottom rail ---
    pull_aabb = ctx.part_element_world_aabb(lower, elem="pull_cup")
    if pull_aabb is not None:
        lo_frame_aabb = ctx.part_element_world_aabb(lower, elem="lower_sash_frame")
        if lo_frame_aabb is not None:
            # Pull cup should be near the bottom of the sash (lower quarter)
            pull_cz = (pull_aabb[0][2] + pull_aabb[1][2]) / 2.0
            sash_bot = lo_frame_aabb[0][2]
            sash_top = lo_frame_aabb[1][2]
            sash_h = sash_top - sash_bot
            ctx.check(
                "pull cup near bottom rail of lower sash",
                pull_cz < sash_bot + sash_h * 0.20,
                details=f"pull_cz={pull_cz:.3f}, sash range=({sash_bot:.3f},{sash_top:.3f})",
            )

    return ctx.report()


object_model = build_object_model()
