from __future__ import annotations

# Horizontal sliding window variant: white frame, one fixed sash (single-lite)
# and one horizontally sliding sash with 3x2 muntin grid bars. Independent
# insect screen on a shallow prismatic track. Deep track grooves along the
# top (head) and bottom (sill) frame rails. Recessed pull cup on the movable
# sash meeting stile.
#
# Coordinate convention:
#   +Z is up. Window stands vertically: height along +Z, width along X,
#   frame depth / glazing thickness along Y. The sill sits at z=0; the head
#   is at z=WIN_H.
#
# Articulation:
#   - FIXED sash is FIXED to the frame (right half, exterior track).
#   - SLIDING sash is PRISMATIC, axis (1,0,0): positive q slides it RIGHT,
#     stacking behind the fixed sash and revealing the left opening.
#   - INSECT screen is PRISMATIC, axis (1,0,0): slides independently on a
#     shallower interior track.

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

WIN_W = 0.92
WIN_H = 1.52
FRAME_FACE = 0.060
FRAME_DEPTH = 0.120

# Clear opening inside the outer frame.
OPEN_W = WIN_W - 2 * FRAME_FACE
OPEN_H = WIN_H - 2 * FRAME_FACE
OPEN_X0 = -WIN_W / 2.0 + FRAME_FACE
OPEN_X1 = WIN_W / 2.0 - FRAME_FACE
OPEN_Z0 = FRAME_FACE
OPEN_Z1 = WIN_H - FRAME_FACE

# Sash geometry. Each sash is slightly wider than half the opening so they
# overlap at the central meeting stile.
SASH_W = OPEN_W * 0.52
SASH_H = OPEN_H - 0.006
SASH_RAIL = 0.052
SASH_DEPTH = 0.034
GLASS_T = 0.006

# Y-track planes: sliding sash rides on the interior (-Y), fixed sash on the
# exterior (+Y), separated so they pass each other at the meeting stile.
SASH_Y_GAP = 0.020
SLIDING_SASH_Y = -SASH_Y_GAP
FIXED_SASH_Y = +SASH_Y_GAP

# Insect screen on the innermost (most interior) track.
SCREEN_Y = -0.048
SCREEN_W = OPEN_W * 0.48
SCREEN_H = OPEN_H - 0.010
SCREEN_FRAME_W = 0.025
SCREEN_DEPTH = 0.012
SCREEN_MESH_T = 0.002

# Deep track grooves cut into the head and sill.
GROOVE_Y_WIDTH = SASH_DEPTH + 0.002
GROOVE_Z_DEPTH = 0.025
SCREEN_GROOVE_Y_WIDTH = SCREEN_DEPTH + 0.002
SCREEN_GROOVE_Z_DEPTH = 0.018

# Muntin grid on sliding sash: 3 columns x 2 rows.
MUNTIN_W = 0.022
N_COLS = 3
N_ROWS = 2

# Recessed pull cup on the sliding sash meeting stile.
PULL_CUP_SIZE = (0.038, 0.008, 0.026)

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

FRAME_RGBA = (0.945, 0.945, 0.945, 1.0)
SASH_RGBA = (0.965, 0.965, 0.965, 1.0)
GLASS_RGBA = (0.30, 0.36, 0.42, 0.34)
SCREEN_FRAME_RGBA = (0.58, 0.58, 0.56, 1.0)
SCREEN_MESH_RGBA = (0.50, 0.50, 0.48, 0.55)
PULL_RGBA = (0.72, 0.73, 0.75, 1.0)


# ---------------------------------------------------------------------------
# Frame geometry (CadQuery)
# ---------------------------------------------------------------------------

def _build_frame_shape() -> cq.Workplane:
    """White outer frame with central opening and deep horizontal track
    grooves in the sill and head for the two sash tracks plus one screen
    track."""
    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, WIN_H / 2.0))
        .box(WIN_W, FRAME_DEPTH, WIN_H)
    )
    opening = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, (OPEN_Z0 + OPEN_Z1) / 2.0))
        .box(OPEN_W, FRAME_DEPTH + 0.02, OPEN_H)
    )
    frame = outer.cut(opening)

    # Deep horizontal grooves in sill and head for each sash track.
    for sash_y in (SLIDING_SASH_Y, FIXED_SASH_Y):
        # Sill groove (cuts upward from opening bottom into the sill).
        sill_groove = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, sash_y, OPEN_Z0 - GROOVE_Z_DEPTH / 2.0))
            .box(OPEN_W + 0.01, GROOVE_Y_WIDTH, GROOVE_Z_DEPTH)
        )
        frame = frame.cut(sill_groove)
        # Head groove (cuts downward from opening top into the head).
        head_groove = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, sash_y, OPEN_Z1 + GROOVE_Z_DEPTH / 2.0))
            .box(OPEN_W + 0.01, GROOVE_Y_WIDTH, GROOVE_Z_DEPTH)
        )
        frame = frame.cut(head_groove)

    # Screen track grooves (shallower, interior-most).
    for z_center in (
        OPEN_Z0 - SCREEN_GROOVE_Z_DEPTH / 2.0,
        OPEN_Z1 + SCREEN_GROOVE_Z_DEPTH / 2.0,
    ):
        screen_groove = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, SCREEN_Y, z_center))
            .box(OPEN_W + 0.01, SCREEN_GROOVE_Y_WIDTH, SCREEN_GROOVE_Z_DEPTH)
        )
        frame = frame.cut(screen_groove)

    return frame


# ---------------------------------------------------------------------------
# Sash frame geometry (CadQuery)
# ---------------------------------------------------------------------------

def _build_sash_frame_shape(with_muntins: bool) -> cq.Workplane:
    """Sash perimeter frame. With muntins: 3x2 lite grid. Without: single
    large lite opening. Local frame: X centered, Z from 0 to SASH_H, Y
    centered."""
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

    if not with_muntins:
        lite = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, 0.0, h / 2.0))
            .box(inner_w, d + 0.02, inner_h)
        )
        return outer.cut(lite)

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


def _build_sash_glass_shape(with_muntins: bool) -> cq.Workplane:
    """Glass pane(s) for the sash, rebated under the rails/muntins."""
    w = SASH_W
    h = SASH_H
    r = SASH_RAIL
    rebate = 0.005

    in_x0, in_x1 = -w / 2.0 + r, w / 2.0 - r
    in_z0, in_z1 = r, h - r
    inner_w = in_x1 - in_x0
    inner_h = in_z1 - in_z0

    if not with_muntins:
        return (
            cq.Workplane("XY")
            .transformed(offset=(0.0, 0.0, h / 2.0))
            .box(inner_w + 2 * rebate, GLASS_T, inner_h + 2 * rebate)
        )

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
# Insect screen geometry (CadQuery)
# ---------------------------------------------------------------------------

def _build_screen_frame_shape() -> cq.Workplane:
    """Thin aluminium screen frame: perimeter ring with central opening."""
    w = SCREEN_W
    h = SCREEN_H
    f = SCREEN_FRAME_W
    d = SCREEN_DEPTH
    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, h / 2.0))
        .box(w, d, h)
    )
    inner = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, h / 2.0))
        .box(w - 2 * f, d + 0.01, h - 2 * f)
    )
    return outer.cut(inner)


def _build_screen_mesh_shape() -> cq.Workplane:
    """Thin semi-transparent mesh panel captured inside the screen frame."""
    w = SCREEN_W
    h = SCREEN_H
    f = SCREEN_FRAME_W
    return (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, h / 2.0))
        .box(w - 2 * f + 0.004, SCREEN_MESH_T, h - 2 * f + 0.004)
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_window")

    model.material("frame", rgba=FRAME_RGBA)
    model.material("sash", rgba=SASH_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("screen_frame", rgba=SCREEN_FRAME_RGBA)
    model.material("screen_mesh", rgba=SCREEN_MESH_RGBA)
    model.material("pull", rgba=PULL_RGBA)

    # --- Outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame_shell"),
        material="frame",
        name="frame_shell",
    )

    # --- Fixed sash (right half, exterior track, single lite) ---
    fixed = model.part("fixed_sash")
    fixed.visual(
        mesh_from_cadquery(_build_sash_frame_shape(False), "fixed_sash_frame"),
        material="sash",
        name="fixed_sash_frame",
    )
    fixed.visual(
        mesh_from_cadquery(_build_sash_glass_shape(False), "fixed_sash_glass"),
        material="glass",
        name="fixed_sash_glass",
    )

    # --- Sliding sash (left half, interior track, 3x2 muntin grid) ---
    sliding = model.part("sliding_sash")
    sliding.visual(
        mesh_from_cadquery(_build_sash_frame_shape(True), "sliding_sash_frame"),
        material="sash",
        name="sliding_sash_frame",
    )
    sliding.visual(
        mesh_from_cadquery(_build_sash_glass_shape(True), "sliding_sash_glass"),
        material="glass",
        name="sliding_sash_glass",
    )

    # Recessed pull cup on the meeting stile (right stile) of the sliding sash.
    pull_x = SASH_W / 2.0 - SASH_RAIL / 2.0
    pull_y = -(SASH_DEPTH / 2.0)
    pull_z = SASH_H * 0.45
    sliding.visual(
        Box(PULL_CUP_SIZE),
        origin=Origin(xyz=(pull_x, pull_y, pull_z)),
        material="pull",
        name="pull_cup",
    )

    # --- Insect screen (interior-most track) ---
    screen = model.part("screen")
    screen.visual(
        mesh_from_cadquery(_build_screen_frame_shape(), "screen_frame_shell"),
        material="screen_frame",
        name="screen_frame",
    )
    screen.visual(
        mesh_from_cadquery(_build_screen_mesh_shape(), "screen_mesh_panel"),
        material="screen_mesh",
        name="screen_mesh",
    )

    # ----- Articulations -----

    # Fixed sash: rigidly attached to the frame (right half).
    fixed_x = OPEN_X1 - SASH_W / 2.0
    fixed_z = OPEN_Z0 + 0.003
    model.articulation(
        "frame_to_fixed_sash",
        ArticulationType.FIXED,
        parent="frame",
        child="fixed_sash",
        origin=Origin(xyz=(fixed_x, FIXED_SASH_Y, fixed_z)),
    )

    # Sliding sash: prismatic, slides right to open (axis +X).
    sliding_x = OPEN_X0 + SASH_W / 2.0
    sliding_z = OPEN_Z0 + 0.003
    model.articulation(
        "frame_to_sliding_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="sliding_sash",
        origin=Origin(xyz=(sliding_x, SLIDING_SASH_Y, sliding_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=0.25, lower=0.0, upper=0.25,
        ),
    )

    # Insect screen: prismatic, slides independently on the interior track.
    screen_x = OPEN_X0 + SCREEN_W / 2.0
    screen_z = OPEN_Z0 + 0.005
    model.articulation(
        "frame_to_screen",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="screen",
        origin=Origin(xyz=(screen_x, SCREEN_Y, screen_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0, velocity=0.15, lower=0.0, upper=0.15,
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    fixed = object_model.get_part("fixed_sash")
    sliding = object_model.get_part("sliding_sash")
    screen = object_model.get_part("screen")
    j_slide = object_model.get_articulation("frame_to_sliding_sash")
    j_screen = object_model.get_articulation("frame_to_screen")

    # --- Intentional overlaps ---

    # Glass panes rebated under sash rails/muntins.
    for sash_name, glass_elem, frame_elem in (
        ("fixed_sash", "fixed_sash_glass", "fixed_sash_frame"),
        ("sliding_sash", "sliding_sash_glass", "sliding_sash_frame"),
    ):
        ctx.allow_overlap(
            sash_name, sash_name,
            elem_a=glass_elem, elem_b=frame_elem,
            reason="Glass panes are rebated under the sash rails/muntins so they read as captured.",
        )

    # Sashes ride in the frame track grooves (retained insertion).
    ctx.allow_overlap(
        "frame", "fixed_sash",
        reason="Fixed sash stiles are retained in the top/bottom frame track grooves.",
    )
    ctx.allow_overlap(
        "frame", "sliding_sash",
        reason="Sliding sash stiles ride in the top/bottom frame track grooves.",
    )
    ctx.allow_overlap(
        "frame", "screen",
        reason="Screen frame rides in the interior screen track grooves.",
    )

    # Sashes overlap at the meeting stile (different Y track planes).
    ctx.allow_overlap(
        "fixed_sash", "sliding_sash",
        reason="Sashes overlap at the central meeting stile; they ride in offset Y track planes.",
    )

    # Pull cup recessed into the sliding sash meeting stile.
    ctx.allow_overlap(
        "sliding_sash", "sliding_sash",
        elem_a="pull_cup", elem_b="sliding_sash_frame",
        reason="Pull cup is recessed into (seated on) the sliding sash meeting stile.",
    )

    # Screen mesh captured inside the screen frame.
    ctx.allow_overlap(
        "screen", "screen",
        elem_a="screen_mesh", elem_b="screen_frame",
        reason="Screen mesh panel is captured inside the screen frame.",
    )

    # --- Closed pose (q=0) ---
    with ctx.pose({j_slide: 0.0, j_screen: 0.0}):
        f_aabb = ctx.part_world_aabb(frame)
        sl_aabb = ctx.part_world_aabb(sliding)
        fx_aabb = ctx.part_world_aabb(fixed)
        sc_aabb = ctx.part_world_aabb(screen)

        # Frame is the widest element.
        frame_w = f_aabb[1][0] - f_aabb[0][0]
        sash_w = sl_aabb[1][0] - sl_aabb[0][0]
        ctx.check(
            "frame spans wider than a sash",
            frame_w > sash_w + 0.05,
            details=f"frame_w={frame_w:.3f}, sash_w={sash_w:.3f}",
        )

        # Sill near z=0, head above 1m.
        ctx.check(
            "frame sill near z=0",
            abs(f_aabb[0][2]) < 0.01 and f_aabb[1][2] > 1.0,
            details=f"frame z=({f_aabb[0][2]:.3f}, {f_aabb[1][2]:.3f})",
        )

        # Sashes are side by side: sliding on the left, fixed on the right.
        sl_cx = (sl_aabb[0][0] + sl_aabb[1][0]) / 2.0
        fx_cx = (fx_aabb[0][0] + fx_aabb[1][0]) / 2.0
        ctx.check(
            "sliding sash left of fixed sash",
            sl_cx < fx_cx - 0.05,
            details=f"sliding_cx={sl_cx:.3f}, fixed_cx={fx_cx:.3f}",
        )

        # Sashes overlap at the meeting stile in X.
        ctx.check(
            "sashes overlap at meeting stile (closed)",
            sl_aabb[1][0] >= fx_aabb[0][0] - 1e-4,
            details=f"sliding_right={sl_aabb[1][0]:.3f}, fixed_left={fx_aabb[0][0]:.3f}",
        )

        # Sashes ride in offset Y planes.
        sl_cy = (sl_aabb[0][1] + sl_aabb[1][1]) / 2.0
        fx_cy = (fx_aabb[0][1] + fx_aabb[1][1]) / 2.0
        ctx.check(
            "sashes ride in offset Y track planes",
            abs(sl_cy - fx_cy) > 0.025,
            details=f"sliding_cy={sl_cy:.3f}, fixed_cy={fx_cy:.3f}",
        )

        # Screen is on the interior side of the sliding sash.
        sc_cy = (sc_aabb[0][1] + sc_aabb[1][1]) / 2.0
        ctx.check(
            "screen is on interior side of sliding sash",
            sc_cy < sl_cy - 0.010,
            details=f"screen_cy={sc_cy:.3f}, sliding_cy={sl_cy:.3f}",
        )

        rest_sl_cx = sl_cx
        rest_sc_cx = (sc_aabb[0][0] + sc_aabb[1][0]) / 2.0

    # --- HERO: sliding sash opens (slides right) ---
    travel = 0.20
    with ctx.pose({j_slide: travel}):
        op = ctx.part_world_aabb(sliding)
        op_cx = (op[0][0] + op[1][0]) / 2.0
        ctx.check(
            "sliding sash slides right when opened",
            op_cx > rest_sl_cx + travel * 0.8,
            details=f"rest_cx={rest_sl_cx:.3f}, opened_cx={op_cx:.3f}, travel={travel:.3f}",
        )
        # Retained: still overlaps the frame in Z (stays in tracks).
        ctx.expect_overlap(
            sliding, frame, axes="z", min_overlap=0.05,
            name="sliding sash retained in frame tracks when open",
        )

    # --- HERO: screen slides independently ---
    with ctx.pose({j_screen: 0.10}):
        sc_op = ctx.part_world_aabb(screen)
        sc_op_cx = (sc_op[0][0] + sc_op[1][0]) / 2.0
        ctx.check(
            "screen slides independently from sashes",
            sc_op_cx > rest_sc_cx + 0.06,
            details=f"rest_sc_cx={rest_sc_cx:.3f}, moved_sc_cx={sc_op_cx:.3f}",
        )
        ctx.expect_overlap(
            screen, frame, axes="z", min_overlap=0.05,
            name="screen retained in frame screen track when moved",
        )

    # --- Pull cup exists on sliding sash meeting stile ---
    pull_aabb = ctx.part_element_world_aabb(sliding, elem="pull_cup")
    if pull_aabb is not None:
        pull_cx = (pull_aabb[0][0] + pull_aabb[1][0]) / 2.0
        sl_aabb_rest = ctx.part_world_aabb(sliding)
        sl_right = sl_aabb_rest[1][0] if sl_aabb_rest else 0.0
        ctx.check(
            "pull cup near the meeting stile (right edge of sliding sash)",
            pull_cx > (sl_aabb_rest[0][0] + sl_right) / 2.0 if sl_aabb_rest else False,
            details=f"pull_cx={pull_cx:.3f}, sash right={sl_right:.3f}",
        )

    # --- Joint types ---
    ctx.check(
        "sliding sash joint is prismatic",
        j_slide.articulation_type == ArticulationType.PRISMATIC,
    )
    ctx.check(
        "screen joint is prismatic",
        j_screen.articulation_type == ArticulationType.PRISMATIC,
    )

    # --- Muntin grid only on sliding sash ---
    # The sliding sash frame has muntin bars (more complex geometry); the
    # fixed sash frame has a single lite opening (no muntins). We verify
    # both sashes exist and have distinct glass visual names.
    sl_glass = sliding.get_visual("sliding_sash_glass")
    fx_glass = fixed.get_visual("fixed_sash_glass")
    ctx.check(
        "sliding sash has glass visual",
        sl_glass is not None,
    )
    ctx.check(
        "fixed sash has glass visual",
        fx_glass is not None,
    )

    return ctx.report()


object_model = build_object_model()
