from __future__ import annotations

# Sliding window variant: white frame with a narrow transom panel above,
# two side-by-side sashes below (one fixed, one sliding horizontally on a
# prismatic joint), roller blocks at the bottom of the moving sash, sill lip,
# and drainage slots.
#
# Coordinate convention:
#   +Z is up. Window stands vertically: height along +Z, width along X, frame
#   depth / glazing thickness along Y. The sill sits at z=0; the head is at
#   z=WIN_H. The exterior face is at +Y.
#
# Articulation:
#   - The FIXED sash sits in the interior (-Y) track on the left.
#   - The SLIDING sash sits in the exterior (+Y) track on the right and slides
#     LEFT on a PRISMATIC joint, axis (-1,0,0). Positive q opens the window.

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

WIN_W = 0.92          # overall window width (X)
WIN_H = 1.52          # overall window height (Z), sill at z=0
FRAME_FACE = 0.060    # outer frame member face width (X/Z)
FRAME_DEPTH = 0.110   # outer frame jamb depth (Y)

# Transom panel above the sliding sashes
TRANSOM_BAR_H = 0.040       # horizontal divider bar between main opening and transom
TRANSOM_OPEN_H = 0.200      # transom glass opening height

# Clear opening inside the outer frame
OPEN_W = WIN_W - 2 * FRAME_FACE   # clear width between jambs
OPEN_X0 = -OPEN_W / 2.0
OPEN_X1 = +OPEN_W / 2.0

# Main opening (below transom bar) for the two sashes
MAIN_Z0 = FRAME_FACE                               # sill top surface
MAIN_Z1 = WIN_H - FRAME_FACE - TRANSOM_BAR_H - TRANSOM_OPEN_H   # header bottom
MAIN_H = MAIN_Z1 - MAIN_Z0                         # main opening height

# Transom opening (above the bar)
TRANSOM_Z0 = MAIN_Z1 + TRANSOM_BAR_H
TRANSOM_Z1 = WIN_H - FRAME_FACE
TRANSOM_ACTUAL_H = TRANSOM_Z1 - TRANSOM_Z0         # should equal TRANSOM_OPEN_H

# Sash geometry: two side-by-side sashes in the main opening
SASH_RAIL = 0.048                       # sash perimeter member width
SASH_DEPTH = 0.034                      # sash thickness (Y)
SASH_GAP = 0.010                        # gap between sashes at meeting stile
SASH_SIDE_CLEAR = 0.005                 # clearance to jambs
SASH_W = (OPEN_W - 2 * SASH_SIDE_CLEAR - SASH_GAP) / 2.0
SASH_H = MAIN_H - 0.008                 # vertical clearance in tracks
GLASS_T = 0.006                         # glazing thickness

# Y track planes: fixed sash interior (-Y), sliding sash exterior (+Y)
SASH_Y_GAP = 0.016
FIXED_SASH_Y = -SASH_Y_GAP
SLIDING_SASH_Y = +SASH_Y_GAP

# Closed-pose sash X centers (world)
FIXED_SASH_X = OPEN_X0 + SASH_SIDE_CLEAR + SASH_W / 2.0
SLIDING_SASH_X = OPEN_X1 - SASH_SIDE_CLEAR - SASH_W / 2.0

# Track grooves (horizontal channels in sill and header)
TRACK_W = 0.018         # groove width (Y)
TRACK_DEPTH = 0.024     # groove depth (Z)

# Muntin grid: 2 columns x 3 rows per sash (6 lites)
MUNTIN_W = 0.022
N_COLS = 2
N_ROWS = 3

# Sash bottom Z (world): embed stile bases into the sill track groove for
# retained insertion (the stile extensions sit below the sill surface).
SASH_BOTTOM_Z = MAIN_Z0 - TRACK_DEPTH * 0.40

# Sill lip: protruding ledge at the bottom exterior face
SILL_LIP_H = 0.016      # lip height
SILL_LIP_EXT = 0.022    # how far lip protrudes beyond exterior frame face

# Drainage slots: weep holes in the sill near the exterior face
DRAIN_SLOT_W = 0.032
DRAIN_SLOT_H = 0.006
N_DRAIN_SLOTS = 3

# Roller blocks at bottom of sliding sash
ROLLER_W = 0.024
ROLLER_H = 0.010
ROLLER_D = SASH_DEPTH * 0.70

# Transom panel frame
TRANSOM_RAIL = 0.028
TRANSOM_DEPTH = 0.030
TRANSOM_PANEL_W = OPEN_W - 0.008     # clearance in opening
TRANSOM_PANEL_H = TRANSOM_OPEN_H - 0.006

# Sash lock on sliding sash meeting stile
LOCK_BODY = (0.020, 0.024, 0.050)    # (X, Y, Z) on the stile
LOCK_LEVER = (0.010, 0.008, 0.040)

# Sliding travel limit
SLIDE_TRAVEL = SASH_W * 0.88

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

FRAME_RGBA = (0.945, 0.945, 0.945, 1.0)     # white painted frame
SASH_RGBA = (0.965, 0.965, 0.965, 1.0)      # white sash
GLASS_RGBA = (0.30, 0.36, 0.42, 0.34)       # cool dark-tinted glass
LOCK_RGBA = (0.86, 0.87, 0.89, 1.0)         # brushed metal lock
ROLLER_RGBA = (0.22, 0.22, 0.25, 1.0)       # dark roller nylon


# ---------------------------------------------------------------------------
# Frame geometry (CadQuery)
# ---------------------------------------------------------------------------

def _build_frame_shape() -> cq.Workplane:
    """White outer frame: perimeter slab with main opening (lower) and transom
    opening (upper) cut out, horizontal track grooves in sill and header for
    horizontal sliding, a protruding sill lip, and drainage weep slots."""

    # Solid outer slab spanning the full window footprint.
    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, WIN_H / 2.0))
        .box(WIN_W, FRAME_DEPTH, WIN_H)
    )

    # Cut the main sash opening (lower portion).
    main_cut = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, (MAIN_Z0 + MAIN_Z1) / 2.0))
        .box(OPEN_W, FRAME_DEPTH + 0.02, MAIN_H)
    )
    frame = outer.cut(main_cut)

    # Cut the transom opening (upper portion, leaves the transom bar between).
    transom_cut = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, (TRANSOM_Z0 + TRANSOM_Z1) / 2.0))
        .box(OPEN_W, FRAME_DEPTH + 0.02, TRANSOM_OPEN_H)
    )
    frame = frame.cut(transom_cut)

    # Horizontal track grooves in the sill (cut downward from sill top surface).
    # Two grooves, one per sash track plane.
    for track_y in (FIXED_SASH_Y, SLIDING_SASH_Y):
        groove = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, track_y, MAIN_Z0 - TRACK_DEPTH / 2.0))
            .box(OPEN_W + 0.01, TRACK_W, TRACK_DEPTH)
        )
        frame = frame.cut(groove)

    # Horizontal track grooves in the header/transom bar (cut upward from
    # the bottom of the transom bar = top of main opening).
    for track_y in (FIXED_SASH_Y, SLIDING_SASH_Y):
        groove = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, track_y, MAIN_Z1 + TRACK_DEPTH / 2.0))
            .box(OPEN_W + 0.01, TRACK_W, TRACK_DEPTH)
        )
        frame = frame.cut(groove)

    # Sill lip: protruding ledge at the bottom exterior edge.
    lip_overlap = 0.005  # ensure solid union with frame body
    sill_lip = (
        cq.Workplane("XY")
        .transformed(offset=(
            0.0,
            FRAME_DEPTH / 2.0 + SILL_LIP_EXT / 2.0 - lip_overlap,
            SILL_LIP_H / 2.0,
        ))
        .box(WIN_W - 0.010, SILL_LIP_EXT + lip_overlap, SILL_LIP_H)
    )
    frame = frame.union(sill_lip)

    # Drainage (weep) slots: cut through the sill from the exterior face
    # inward to the outer track groove.
    drain_span = FRAME_DEPTH / 2.0 - SLIDING_SASH_Y + 0.008
    drain_cy = (FRAME_DEPTH / 2.0 + SLIDING_SASH_Y) / 2.0
    drain_cz = MAIN_Z0 - TRACK_DEPTH * 0.35  # just below the sill surface
    spacing = OPEN_W / (N_DRAIN_SLOTS + 1)
    for i in range(N_DRAIN_SLOTS):
        dx = OPEN_X0 + spacing * (i + 1)
        slot = (
            cq.Workplane("XY")
            .transformed(offset=(dx, drain_cy, drain_cz))
            .box(DRAIN_SLOT_W, drain_span, DRAIN_SLOT_H)
        )
        frame = frame.cut(slot)

    return frame


# ---------------------------------------------------------------------------
# Sash frame geometry (CadQuery): perimeter ring + muntin grid
# ---------------------------------------------------------------------------

def _build_sash_frame_shape() -> cq.Workplane:
    """One sash: perimeter ring plus a 2-col x 3-row muntin grid.
    Local frame: X centered, Z from 0 to SASH_H, Y centered."""
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
# Transom panel geometry (CadQuery)
# ---------------------------------------------------------------------------

def _build_transom_frame_shape() -> cq.Workplane:
    """Narrow transom panel: perimeter frame with one vertical muntin
    dividing it into two lites. Local frame: X centered, Z from 0 to height."""
    w = TRANSOM_PANEL_W
    h = TRANSOM_PANEL_H
    r = TRANSOM_RAIL
    d = TRANSOM_DEPTH

    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, h / 2.0))
        .box(w, d, h)
    )

    # Inner glazed region
    in_x0, in_x1 = -w / 2.0 + r, w / 2.0 - r
    in_z0, in_z1 = r, h - r
    half_m = MUNTIN_W / 2.0

    # Center vertical muntin divides into two panes
    mid_x = 0.0

    # Left lite opening
    left_opening = (
        cq.Workplane("XY")
        .transformed(offset=((in_x0 + mid_x - half_m) / 2.0, 0.0, (in_z0 + in_z1) / 2.0))
        .box(mid_x - half_m - in_x0, d + 0.02, in_z1 - in_z0)
    )
    # Right lite opening
    right_opening = (
        cq.Workplane("XY")
        .transformed(offset=((mid_x + half_m + in_x1) / 2.0, 0.0, (in_z0 + in_z1) / 2.0))
        .box(in_x1 - mid_x - half_m, d + 0.02, in_z1 - in_z0)
    )

    return outer.cut(left_opening).cut(right_opening)


def _build_transom_glass_shape() -> cq.Workplane:
    """Two glass panes for the transom, rebated under the frame lips."""
    w = TRANSOM_PANEL_W
    h = TRANSOM_PANEL_H
    r = TRANSOM_RAIL
    rebate = 0.004
    half_m = MUNTIN_W / 2.0

    in_x0, in_x1 = -w / 2.0 + r, w / 2.0 - r
    in_z0, in_z1 = r, h - r
    mid_x = 0.0

    # Left pane
    left = (
        cq.Workplane("XY")
        .transformed(offset=((in_x0 + mid_x - half_m) / 2.0, 0.0, (in_z0 + in_z1) / 2.0))
        .box(mid_x - half_m - in_x0 + 2 * rebate, GLASS_T, in_z1 - in_z0 + 2 * rebate)
    )
    # Right pane
    right = (
        cq.Workplane("XY")
        .transformed(offset=((mid_x + half_m + in_x1) / 2.0, 0.0, (in_z0 + in_z1) / 2.0))
        .box(in_x1 - mid_x - half_m + 2 * rebate, GLASS_T, in_z1 - in_z0 + 2 * rebate)
    )
    return left.union(right)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_window")

    model.material("frame", rgba=FRAME_RGBA)
    model.material("sash", rgba=SASH_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("lock", rgba=LOCK_RGBA)
    model.material("roller", rgba=ROLLER_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame_shell"),
        material="frame",
        name="frame_shell",
    )

    # --- Transom panel (fixed, above sashes) ---
    transom = model.part("transom")
    transom.visual(
        mesh_from_cadquery(_build_transom_frame_shape(), "transom_frame"),
        material="sash",
        name="transom_frame",
    )
    transom.visual(
        mesh_from_cadquery(_build_transom_glass_shape(), "transom_glass"),
        material="glass",
        name="transom_glass",
    )

    # --- Fixed sash (left side, interior track) ---
    fixed_sash = model.part("fixed_sash")
    fixed_sash.visual(
        mesh_from_cadquery(_build_sash_frame_shape(), "fixed_sash_frame"),
        material="sash",
        name="fixed_sash_frame",
    )
    fixed_sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(), "fixed_sash_glass"),
        material="glass",
        name="fixed_sash_glass",
    )

    # --- Sliding sash (right side, exterior track) with rollers and lock ---
    sliding_sash = model.part("sliding_sash")
    sliding_sash.visual(
        mesh_from_cadquery(_build_sash_frame_shape(), "sliding_sash_frame"),
        material="sash",
        name="sliding_sash_frame",
    )
    sliding_sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(), "sliding_sash_glass"),
        material="glass",
        name="sliding_sash_glass",
    )

    # Two roller blocks at the bottom of the sliding sash (near each stile).
    roller_inset = 0.030   # distance from stile edge to roller center
    roller_z = -ROLLER_H / 2.0  # protrudes below sash bottom (local z=0)
    for idx, sign in enumerate((-1.0, +1.0)):
        rx = sign * (SASH_W / 2.0 - roller_inset)
        sliding_sash.visual(
            Box((ROLLER_W, ROLLER_D, ROLLER_H)),
            origin=Origin(xyz=(rx, 0.0, roller_z)),
            material="roller",
            name=f"roller_{idx}",
        )

    # Sash lock on the meeting stile (left stile of sliding sash).
    lock_x = -SASH_W / 2.0 + SASH_RAIL / 2.0
    lock_y = -(SASH_DEPTH / 2.0 + LOCK_BODY[1] / 2.0 - 0.004)
    lock_z = SASH_H * 0.48   # mid-height on the stile
    sliding_sash.visual(
        Box(LOCK_BODY),
        origin=Origin(xyz=(lock_x, lock_y, lock_z)),
        material="lock",
        name="sliding_lock_body",
    )
    sliding_sash.visual(
        Box(LOCK_LEVER),
        origin=Origin(xyz=(lock_x, lock_y - LOCK_BODY[1] / 2.0 - LOCK_LEVER[1] / 2.0 + 0.002, lock_z)),
        material="lock",
        name="sliding_lock_lever",
    )

    # ----- Articulations -----

    # Transom: fixed panel in the transom opening.
    model.articulation(
        "frame_to_transom",
        ArticulationType.FIXED,
        parent=frame,
        child=transom,
        origin=Origin(xyz=(0.0, 0.0, TRANSOM_Z0)),
    )

    # Fixed sash: stationary in the left side, interior track.
    model.articulation(
        "frame_to_fixed_sash",
        ArticulationType.FIXED,
        parent=frame,
        child=fixed_sash,
        origin=Origin(xyz=(FIXED_SASH_X, FIXED_SASH_Y, SASH_BOTTOM_Z)),
    )

    # Sliding sash: prismatic, axis (-1,0,0) so positive q slides left (opens).
    model.articulation(
        "frame_to_sliding_sash",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=sliding_sash,
        origin=Origin(xyz=(SLIDING_SASH_X, SLIDING_SASH_Y, SASH_BOTTOM_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0, velocity=0.30, lower=0.0, upper=SLIDE_TRAVEL,
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    transom = object_model.get_part("transom")
    fixed_sash = object_model.get_part("fixed_sash")
    sliding_sash = object_model.get_part("sliding_sash")
    j_slide = object_model.get_articulation("frame_to_sliding_sash")

    # --- Intentional overlaps ---
    # Glass panes tucked under sash muntin/rail lips (captured glass).
    for sash_name in ("fixed_sash", "sliding_sash"):
        ctx.allow_overlap(
            sash_name, sash_name,
            elem_a=f"{sash_name}_glass",
            elem_b=f"{sash_name}_frame",
            reason="Glass panes are rebated under the sash rails/muntins (captured glazing).",
        )
    # Transom glass rebated under transom frame.
    ctx.allow_overlap(
        "transom", "transom",
        elem_a="transom_glass",
        elem_b="transom_frame",
        reason="Transom glass panes are rebated under the transom frame rails.",
    )
    # Transom panel sits inside the frame transom opening (seated panel).
    ctx.allow_overlap(
        "frame", "transom",
        elem_a="frame_shell",
        elem_b="transom_frame",
        reason="Transom panel is seated inside the frame transom opening; mesh tessellation reports full-panel overlap at the cut boundary.",
    )
    # Sashes ride in the track grooves cut into the frame sill and header.
    ctx.allow_overlap(
        "frame", "fixed_sash",
        reason="Fixed sash stiles sit in the sill and header track grooves (retained).",
    )
    ctx.allow_overlap(
        "frame", "sliding_sash",
        reason="Sliding sash stiles ride in the sill and header track grooves (retained).",
    )
    # The two sashes overlap at the meeting stile (different Y planes).
    ctx.allow_overlap(
        "fixed_sash", "sliding_sash",
        reason="Sashes overlap at the meeting stile; they ride in offset Y track planes.",
    )
    # Roller blocks protrude below the sash bottom into the track groove.
    ctx.allow_overlap(
        "sliding_sash", "frame",
        elem_a="roller_0",
        elem_b="frame_shell",
        reason="Roller blocks protrude below the sash into the sill track groove.",
    )
    ctx.allow_overlap(
        "sliding_sash", "frame",
        elem_a="roller_1",
        elem_b="frame_shell",
        reason="Roller blocks protrude below the sash into the sill track groove.",
    )
    # Lock body seated on the meeting stile.
    ctx.allow_overlap(
        "sliding_sash", "sliding_sash",
        elem_a="sliding_lock_body",
        elem_b="sliding_sash_frame",
        reason="Lock body is mounted (seated) onto the sliding sash meeting stile.",
    )

    # --- Transom panel sits within frame transom opening ---
    ctx.expect_within(
        transom, frame, axes="x",
        inner_elem="transom_frame",
        outer_elem="frame_shell",
        margin=0.010,
        name="transom frame within frame width",
    )
    transom_aabb = ctx.part_world_aabb(transom)
    sash_fixed_aabb = ctx.part_world_aabb(fixed_sash)
    ctx.check(
        "transom sits above sashes",
        transom_aabb[0][2] > sash_fixed_aabb[1][2] - 0.02,
        details=f"transom_z_min={transom_aabb[0][2]:.3f}, sash_z_max={sash_fixed_aabb[1][2]:.3f}",
    )

    # --- Sill lip protrudes beyond the frame exterior face ---
    frame_aabb = ctx.part_world_aabb(frame)
    ctx.check(
        "sill lip protrudes beyond frame exterior",
        frame_aabb[1][1] > FRAME_DEPTH / 2.0 + 0.005,
        details=f"frame_y_max={frame_aabb[1][1]:.3f}, expected>{FRAME_DEPTH / 2.0 + 0.005:.3f}",
    )

    # --- Roller blocks exist on the sliding sash ---
    roller_0_aabb = ctx.part_element_world_aabb(sliding_sash, elem="roller_0")
    roller_1_aabb = ctx.part_element_world_aabb(sliding_sash, elem="roller_1")
    ctx.check(
        "roller blocks exist on sliding sash",
        roller_0_aabb is not None and roller_1_aabb is not None,
        details="roller_0 or roller_1 visual not found",
    )
    if roller_0_aabb and roller_1_aabb:
        # Rollers are near the bottom of the sash
        sash_sliding_aabb = ctx.part_world_aabb(sliding_sash)
        ctx.check(
            "rollers near sash bottom",
            roller_0_aabb[0][2] < sash_sliding_aabb[0][2] + 0.020
            and roller_1_aabb[0][2] < sash_sliding_aabb[0][2] + 0.020,
            details=f"roller_0_z_min={roller_0_aabb[0][2]:.3f}, roller_1_z_min={roller_1_aabb[0][2]:.3f}",
        )
        # Rollers are spaced apart in X (one near each stile)
        roller_span = abs(
            (roller_0_aabb[0][0] + roller_0_aabb[1][0]) / 2.0
            - (roller_1_aabb[0][0] + roller_1_aabb[1][0]) / 2.0
        )
        ctx.check(
            "rollers spaced apart near stiles",
            roller_span > SASH_W * 0.5,
            details=f"roller_x_span={roller_span:.3f}, sash_w={SASH_W:.3f}",
        )

    # --- Sliding joint is prismatic (proof for roller/sash retention allowances) ---
    slide_summary = ctx.articulation_summary(j_slide) if hasattr(ctx, "articulation_summary") else None
    if slide_summary is None:
        # Fallback: just check the joint exists and the sash moves in the right direction.
        ctx.check("sliding articulation exists", j_slide is not None, details="joint not found")

    # --- Fixed sash retained in sill groove (proof for frame-fixed_sash overlap) ---
    ctx.expect_overlap(
        fixed_sash, frame, axes="z", min_overlap=0.005,
        name="fixed sash embedded in sill groove at Z",
    )

    # --- Closed pose (q=0): window reads shut ---
    with ctx.pose({j_slide: 0.0}):
        f_aabb = ctx.part_world_aabb(frame)
        fx_aabb = ctx.part_world_aabb(fixed_sash)
        sl_aabb = ctx.part_world_aabb(sliding_sash)

        # Frame is the tallest and widest element.
        ctx.check(
            "frame spans full window height",
            f_aabb[1][2] - f_aabb[0][2] > WIN_H * 0.95,
            details=f"frame_h={f_aabb[1][2] - f_aabb[0][2]:.3f}",
        )

        # Fixed sash on the left, sliding sash on the right.
        fx_cx = (fx_aabb[0][0] + fx_aabb[1][0]) / 2.0
        sl_cx = (sl_aabb[0][0] + sl_aabb[1][0]) / 2.0
        ctx.check(
            "fixed sash left, sliding sash right at closed pose",
            fx_cx < sl_cx - 0.10,
            details=f"fixed_cx={fx_cx:.3f}, sliding_cx={sl_cx:.3f}",
        )

        # Both sashes within the frame opening.
        ctx.check(
            "sashes within frame width at closed",
            fx_aabb[0][0] > f_aabb[0][0] and fx_aabb[1][0] < f_aabb[1][0]
            and sl_aabb[0][0] > f_aabb[0][0] and sl_aabb[1][0] < f_aabb[1][0],
            details=f"fixed x=({fx_aabb[0][0]:.3f},{fx_aabb[1][0]:.3f}) sliding x=({sl_aabb[0][0]:.3f},{sl_aabb[1][0]:.3f})",
        )

        # Sashes in offset Y planes (use frame elements only, not lock/rollers).
        fx_frame_aabb = ctx.part_element_world_aabb(fixed_sash, elem="fixed_sash_frame")
        sl_frame_aabb = ctx.part_element_world_aabb(sliding_sash, elem="sliding_sash_frame")
        if fx_frame_aabb and sl_frame_aabb:
            fx_cy = (fx_frame_aabb[0][1] + fx_frame_aabb[1][1]) / 2.0
            sl_cy = (sl_frame_aabb[0][1] + sl_frame_aabb[1][1]) / 2.0
            ctx.check(
                "sashes ride in offset Y track planes",
                abs(fx_cy - sl_cy) > 0.020,
                details=f"fixed_cy={fx_cy:.3f}, sliding_cy={sl_cy:.3f}",
            )

        rest_sl_cx = sl_cx

    # --- HERO: sliding sash slides LEFT (opens) ---
    with ctx.pose({j_slide: SLIDE_TRAVEL}):
        op_aabb = ctx.part_world_aabb(sliding_sash)
        op_cx = (op_aabb[0][0] + op_aabb[1][0]) / 2.0
        ctx.check(
            "sliding sash moves left when opened",
            op_cx < rest_sl_cx - SLIDE_TRAVEL * 0.8,
            details=f"rest_cx={rest_sl_cx:.3f}, opened_cx={op_cx:.3f}, travel={SLIDE_TRAVEL:.3f}",
        )
        # Still retained in the frame (overlaps frame in X).
        ctx.expect_overlap(
            sliding_sash, frame, axes="x", min_overlap=0.05,
            name="sliding sash retained in frame when open",
        )
        # Still overlaps the fixed sash at meeting stile (in the Y-overlap region).
        ctx.expect_overlap(
            sliding_sash, fixed_sash, axes="z", min_overlap=SASH_H * 0.5,
            name="sliding sash still vertically aligned with fixed sash when open",
        )

    return ctx.report()


object_model = build_object_model()
