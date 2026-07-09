from __future__ import annotations

# Corner-lift horizontal sliding window: white frame, two side-by-side sashes
# (one fixed, one sliding), a small vent panel, a rotating latch at the
# meeting stile, and two roller blocks on the moving sash.
#
# Variant 08 of the double-hung sash window family, forked into a sliding
# window sibling. Remains a framed sliding window — not a door or cabinet.
#
# Coordinate convention:
#   +Z is up. Window stands vertically: height along +Z, width along X,
#   frame depth along Y. The sill sits at z=0; the head is at z=WIN_H.
#   The glass plane is the X-Z plane.
#
# Articulation:
#   - Moving sash: PRISMATIC, axis (-1, 0, 0): positive q slides it LEFT (opens).
#   - Latch: REVOLUTE, axis (0, 1, 0): positive q rotates latch lever open.
#   - Fixed sash: FIXED to frame.
#   - Vent panel: FIXED to fixed_sash.

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

WIN_W = 1.10           # overall window width (X)
WIN_H = 1.20           # overall window height (Z), sill at z=0
FRAME_FACE = 0.058     # outer frame member face width (X/Z)
FRAME_DEPTH = 0.100    # outer frame jamb depth (Y)

# Clear opening inside the outer frame
OPEN_W = WIN_W - 2 * FRAME_FACE   # clear width
OPEN_H = WIN_H - 2 * FRAME_FACE   # clear height
OPEN_X0 = -WIN_W / 2.0 + FRAME_FACE
OPEN_X1 = WIN_W / 2.0 - FRAME_FACE
OPEN_Z0 = FRAME_FACE
OPEN_Z1 = WIN_H - FRAME_FACE

# Sash geometry. Each sash is slightly more than half the opening width so
# they overlap at the meeting stile when closed.
SASH_W = OPEN_W * 0.53 + 0.010      # sash width with overlap
SASH_RAIL = 0.048                    # horizontal rail width
SASH_STILE = 0.044                   # regular vertical stile width
SASH_DEPTH = 0.032                   # sash thickness (Y)
SASH_H = OPEN_H + 0.008             # sash extends into sill/head tracks for retention
OVERLAP_STILE_W = 0.058             # wider stile at meeting edge

GLASS_T = 0.005                      # glazing thickness

# Y planes: fixed sash rides rear (+Y), moving sash rides front (-Y)
SASH_Y_GAP = 0.018
FIXED_SASH_Y = +SASH_Y_GAP
MOVING_SASH_Y = -SASH_Y_GAP

# Sash bottom Z (extends into sill track groove for retention)
SASH_BOTTOM_Z = OPEN_Z0 - 0.004

# Muntin grid: 2 columns x 2 rows of lites per sash
MUNTIN_W = 0.020
N_COLS = 2
N_ROWS = 2

# Track channels in sill and head
TRACK_W = 0.016         # track groove width (Z)
TRACK_DEPTH = 0.024     # track groove depth (Y) — narrower than sash for retention contact

# Closed-pose sash center X positions
FIXED_SASH_CX = OPEN_X0 + SASH_W / 2.0 + 0.003
MOVING_SASH_CX = OPEN_X1 - SASH_W / 2.0 - 0.003

# Maximum slide travel
SLIDE_TRAVEL = MOVING_SASH_CX - FIXED_SASH_CX - 0.02

# Roller blocks at the bottom of the moving sash
ROLLER_SIZE = (0.028, 0.024, 0.014)   # (X, Y, Z)

# Latch at the meeting stile
LATCH_BASE_SIZE = (0.036, 0.014, 0.024)   # (X, Y, Z)
LATCH_LEVER_SIZE = (0.030, 0.008, 0.012)  # (X, Y, Z)

# Vent panel (in the lower-left lite of the fixed sash)
VENT_BORDER = 0.010   # frame border width around the vent opening

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

FRAME_RGBA = (0.945, 0.945, 0.945, 1.0)    # white painted frame
SASH_RGBA = (0.965, 0.965, 0.965, 1.0)     # white sash (slightly brighter)
GLASS_RGBA = (0.30, 0.36, 0.42, 0.34)      # cool dark-tinted glass
LATCH_RGBA = (0.72, 0.73, 0.76, 1.0)       # brushed metal latch
ROLLER_RGBA = (0.18, 0.18, 0.20, 1.0)      # dark nylon roller
VENT_RGBA = (0.90, 0.91, 0.92, 1.0)        # vent panel frame (slight grey)


# ---------------------------------------------------------------------------
# Static outer frame geometry (CadQuery)
# ---------------------------------------------------------------------------

def _build_frame_shape() -> cq.Workplane:
    """White outer frame: perimeter slab with central opening cut out,
    plus horizontal track grooves in the sill and head for the sashes."""
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

    # Horizontal track grooves in sill (bottom) and head (top).
    # Each track plane gets a groove cut into the inner face of the member.
    # Sill: groove starts at z=OPEN_Z0 and cuts downward into sill material.
    # Head: groove starts at z=OPEN_Z1 and cuts upward into head material.
    for z_center in (OPEN_Z0 - TRACK_W / 2.0, OPEN_Z1 + TRACK_W / 2.0):
        for track_y in (FIXED_SASH_Y, MOVING_SASH_Y):
            groove = (
                cq.Workplane("XY")
                .transformed(offset=(0.0, track_y, z_center))
                .box(OPEN_W + 0.02, TRACK_DEPTH, TRACK_W + 0.002)
            )
            frame = frame.cut(groove)

    return frame


# ---------------------------------------------------------------------------
# Sash frame geometry (CadQuery): perimeter ring + muntin grid
# ---------------------------------------------------------------------------

def _build_sash_frame_shape(
    width: float,
    height: float,
    depth: float,
    rail: float,
    stile: float,
    n_cols: int,
    n_rows: int,
    muntin: float,
    left_stile_override: float | None = None,
) -> cq.Workplane:
    """Sash frame with perimeter rails/stiles and muntin grid openings.

    Local frame: X from -width/2 to +width/2, Z from 0 to height,
    Y centered at 0.
    """
    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, height / 2.0))
        .box(width, depth, height)
    )

    ls = left_stile_override if left_stile_override is not None else stile
    rs = stile

    in_x0 = -width / 2.0 + ls
    in_x1 = width / 2.0 - rs
    in_z0 = rail
    in_z1 = height - rail
    inner_w = in_x1 - in_x0
    inner_h = in_z1 - in_z0

    col_lines = [in_x0 + (i + 1) * inner_w / n_cols for i in range(n_cols - 1)]
    row_lines = [in_z0 + (j + 1) * inner_h / n_rows for j in range(n_rows - 1)]
    x_edges = [in_x0] + col_lines + [in_x1]
    z_edges = [in_z0] + row_lines + [in_z1]
    half_m = muntin / 2.0

    sash = outer
    for ci in range(n_cols):
        for ri in range(n_rows):
            lx0 = x_edges[ci] + (half_m if ci > 0 else 0.0)
            lx1 = x_edges[ci + 1] - (half_m if ci < n_cols - 1 else 0.0)
            lz0 = z_edges[ri] + (half_m if ri > 0 else 0.0)
            lz1 = z_edges[ri + 1] - (half_m if ri < n_rows - 1 else 0.0)
            lite = (
                cq.Workplane("XY")
                .transformed(offset=((lx0 + lx1) / 2.0, 0.0, (lz0 + lz1) / 2.0))
                .box(lx1 - lx0, depth + 0.02, lz1 - lz0)
            )
            sash = sash.cut(lite)
    return sash


def _compute_lite_bounds(
    width: float,
    height: float,
    rail: float,
    stile: float,
    n_cols: int,
    n_rows: int,
    muntin: float,
    left_stile_override: float | None = None,
) -> list[list[tuple[float, float, float, float]]]:
    """Return lite opening bounds [(x0, x1, z0, z1)] indexed [col][row]."""
    ls = left_stile_override if left_stile_override is not None else stile
    rs = stile
    in_x0 = -width / 2.0 + ls
    in_x1 = width / 2.0 - rs
    in_z0 = rail
    in_z1 = height - rail
    inner_w = in_x1 - in_x0
    inner_h = in_z1 - in_z0
    col_lines = [in_x0 + (i + 1) * inner_w / n_cols for i in range(n_cols - 1)]
    row_lines = [in_z0 + (j + 1) * inner_h / n_rows for j in range(n_rows - 1)]
    x_edges = [in_x0] + col_lines + [in_x1]
    z_edges = [in_z0] + row_lines + [in_z1]
    half_m = muntin / 2.0

    bounds = []
    for ci in range(n_cols):
        col_bounds = []
        for ri in range(n_rows):
            lx0 = x_edges[ci] + (half_m if ci > 0 else 0.0)
            lx1 = x_edges[ci + 1] - (half_m if ci < n_cols - 1 else 0.0)
            lz0 = z_edges[ri] + (half_m if ri > 0 else 0.0)
            lz1 = z_edges[ri + 1] - (half_m if ri < n_rows - 1 else 0.0)
            col_bounds.append((lx0, lx1, lz0, lz1))
        bounds.append(col_bounds)
    return bounds


def _build_glass_shape(
    width: float,
    height: float,
    depth: float,
    rail: float,
    stile: float,
    n_cols: int,
    n_rows: int,
    muntin: float,
    glass_t: float,
    left_stile_override: float | None = None,
    skip_lites: set[tuple[int, int]] | None = None,
) -> cq.Workplane:
    """Glass panes for the sash lite openings."""
    bounds = _compute_lite_bounds(
        width, height, rail, stile, n_cols, n_rows, muntin, left_stile_override
    )
    rebate = 0.004
    skip = skip_lites or set()

    panes = None
    for ci in range(n_cols):
        for ri in range(n_rows):
            if (ci, ri) in skip:
                continue
            lx0, lx1, lz0, lz1 = bounds[ci][ri]
            pane = (
                cq.Workplane("XY")
                .transformed(
                    offset=((lx0 + lx1) / 2.0, 0.0, (lz0 + lz1) / 2.0)
                )
                .box(lx1 - lx0 + rebate, glass_t, lz1 - lz0 + rebate)
            )
            panes = pane if panes is None else panes.union(pane)
    return panes


# ---------------------------------------------------------------------------
# Vent panel geometry
# ---------------------------------------------------------------------------

def _build_vent_panel_shape(lite_w: float, lite_h: float, depth: float, border: float) -> cq.Workplane:
    """Small vent panel frame with louver slits. Built centered at origin.
    Sized to match the lite opening so it seats flush against the sash muntins."""
    vw = lite_w
    vh = lite_h
    vd = depth * 0.90  # slightly thinner than sash to sit within the opening
    # Outer frame plate
    outer = (
        cq.Workplane("XY")
        .box(vw, vd, vh)
    )
    # Cut the inner opening (leaving the border frame)
    inner = (
        cq.Workplane("XY")
        .box(vw - 2 * border, vd + 0.02, vh - 2 * border)
    )
    panel = outer.cut(inner)
    # Add 3 horizontal louver slats inside the opening
    n_slats = 3
    open_h = vh - 2 * border
    slat_h = 0.006
    for i in range(n_slats):
        frac = (i + 1) / (n_slats + 1)
        sz = -open_h / 2.0 + frac * open_h
        slat = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, 0.0, sz))
            .box(vw - 2 * border - 0.004, vd * 0.5, slat_h)
        )
        panel = panel.union(slat)
    return panel


def _build_vent_glass(lite_w: float, lite_h: float, border: float, glass_t: float) -> cq.Workplane:
    """Thin glass pane for the vent panel opening."""
    return (
        cq.Workplane("XY")
        .box(lite_w - 2 * border + 0.002, glass_t, lite_h - 2 * border + 0.002)
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="corner_lift_sliding_window")

    model.material("frame", rgba=FRAME_RGBA)
    model.material("sash", rgba=SASH_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("latch", rgba=LATCH_RGBA)
    model.material("roller", rgba=ROLLER_RGBA)
    model.material("vent", rgba=VENT_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material="frame",
        name="frame_shell",
    )

    # --- Fixed sash (left side, rear track) ---
    fixed_sash = model.part("fixed_sash")
    fixed_sash.visual(
        mesh_from_cadquery(
            _build_sash_frame_shape(
                SASH_W, SASH_H, SASH_DEPTH,
                SASH_RAIL, SASH_STILE, N_COLS, N_ROWS, MUNTIN_W,
            ),
            "fixed_sash_frame",
        ),
        material="sash",
        name="fixed_sash_frame",
    )
    # Glass: skip the lower-left lite (vent panel goes there)
    fixed_sash.visual(
        mesh_from_cadquery(
            _build_glass_shape(
                SASH_W, SASH_H, SASH_DEPTH,
                SASH_RAIL, SASH_STILE, N_COLS, N_ROWS, MUNTIN_W, GLASS_T,
                skip_lites={(0, 0)},
            ),
            "fixed_sash_glass",
        ),
        material="glass",
        name="fixed_sash_glass",
    )

    # --- Vent panel (FIXED child of fixed_sash) ---
    vent_panel = model.part("vent_panel")
    # Compute vent lite position in the fixed sash local frame
    lite_bounds = _compute_lite_bounds(
        SASH_W, SASH_H, SASH_RAIL, SASH_STILE, N_COLS, N_ROWS, MUNTIN_W,
    )
    vent_lx0, vent_lx1, vent_lz0, vent_lz1 = lite_bounds[0][0]
    vent_cx = (vent_lx0 + vent_lx1) / 2.0
    vent_cz = (vent_lz0 + vent_lz1) / 2.0
    vent_lite_w = vent_lx1 - vent_lx0
    vent_lite_h = vent_lz1 - vent_lz0

    vent_panel.visual(
        mesh_from_cadquery(
            _build_vent_panel_shape(vent_lite_w, vent_lite_h, SASH_DEPTH, VENT_BORDER),
            "vent_panel_frame",
        ),
        material="vent",
        name="vent_panel_frame",
    )
    vent_panel.visual(
        mesh_from_cadquery(
            _build_vent_glass(vent_lite_w, vent_lite_h, VENT_BORDER, GLASS_T),
            "vent_panel_glass",
        ),
        material="glass",
        name="vent_panel_glass",
    )

    # --- Moving sash (right side, front track, slides left to open) ---
    moving_sash = model.part("moving_sash")
    moving_sash.visual(
        mesh_from_cadquery(
            _build_sash_frame_shape(
                SASH_W, SASH_H, SASH_DEPTH,
                SASH_RAIL, SASH_STILE, N_COLS, N_ROWS, MUNTIN_W,
                left_stile_override=OVERLAP_STILE_W,
            ),
            "moving_sash_frame",
        ),
        material="sash",
        name="moving_sash_frame",
    )
    moving_sash.visual(
        mesh_from_cadquery(
            _build_glass_shape(
                SASH_W, SASH_H, SASH_DEPTH,
                SASH_RAIL, SASH_STILE, N_COLS, N_ROWS, MUNTIN_W, GLASS_T,
                left_stile_override=OVERLAP_STILE_W,
            ),
            "moving_sash_glass",
        ),
        material="glass",
        name="moving_sash_glass",
    )

    # Roller blocks at the bottom of the moving sash (two, near corners)
    roller_y = -(SASH_DEPTH / 2.0 + ROLLER_SIZE[1] / 2.0 - 0.004)
    roller_z = ROLLER_SIZE[2] / 2.0  # sitting on the bottom rail
    for i, roller_x in enumerate((-SASH_W / 2.0 + 0.060, SASH_W / 2.0 - 0.060)):
        moving_sash.visual(
            Box(ROLLER_SIZE),
            origin=Origin(xyz=(roller_x, roller_y, roller_z)),
            material="roller",
            name=f"roller_{i}",
        )

    # Latch base (static, mounted on moving sash meeting stile)
    latch_x = -SASH_W / 2.0 + OVERLAP_STILE_W / 2.0
    latch_y = -(SASH_DEPTH / 2.0 + LATCH_BASE_SIZE[1] / 2.0 - 0.003)
    latch_z = SASH_H / 2.0
    moving_sash.visual(
        Box(LATCH_BASE_SIZE),
        origin=Origin(xyz=(latch_x, latch_y, latch_z)),
        material="latch",
        name="latch_base",
    )

    # --- Latch lever (REVOLUTE child of moving_sash) ---
    latch = model.part("latch")
    latch.visual(
        Box(LATCH_LEVER_SIZE),
        origin=Origin(xyz=(LATCH_LEVER_SIZE[0] / 2.0, 0.0, 0.0)),
        material="latch",
        name="latch_lever",
    )

    # ----- Articulations -----

    # Fixed sash: FIXED to frame, positioned on the left side in the rear track
    model.articulation(
        "frame_to_fixed_sash",
        ArticulationType.FIXED,
        parent="frame",
        child="fixed_sash",
        origin=Origin(xyz=(FIXED_SASH_CX, FIXED_SASH_Y, SASH_BOTTOM_Z)),
    )

    # Vent panel: FIXED to fixed_sash, at the vent lite position
    model.articulation(
        "sash_to_vent",
        ArticulationType.FIXED,
        parent="fixed_sash",
        child="vent_panel",
        origin=Origin(xyz=(vent_cx, 0.0, vent_cz)),
    )

    # Moving sash: PRISMATIC, slides left (axis -X) to open
    model.articulation(
        "frame_to_moving_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="moving_sash",
        origin=Origin(xyz=(MOVING_SASH_CX, MOVING_SASH_Y, SASH_BOTTOM_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=50.0, velocity=0.30, lower=0.0, upper=SLIDE_TRAVEL,
        ),
    )

    # Latch: REVOLUTE, rotates around Y axis at the meeting stile
    # Origin is in the moving_sash local frame, at the latch pivot point
    model.articulation(
        "sash_to_latch",
        ArticulationType.REVOLUTE,
        parent="moving_sash",
        child="latch",
        origin=Origin(xyz=(latch_x, latch_y - LATCH_BASE_SIZE[1] / 2.0, latch_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=4.0, lower=0.0, upper=1.57,
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    fixed_sash = object_model.get_part("fixed_sash")
    moving_sash = object_model.get_part("moving_sash")
    vent_panel = object_model.get_part("vent_panel")
    latch = object_model.get_part("latch")

    j_slide = object_model.get_articulation("frame_to_moving_sash")
    j_latch = object_model.get_articulation("sash_to_latch")

    # --- Intentional overlaps ---
    # Glass panes are captured under sash rails/muntins
    for sash_name in ("fixed_sash", "moving_sash"):
        ctx.allow_overlap(
            sash_name, sash_name,
            elem_a=f"{sash_name}_glass",
            elem_b=f"{sash_name}_frame",
            reason="Glass panes are rebated under sash rails/muntins (captured glazing).",
        )

    # Moving sash rides in the frame track grooves
    ctx.allow_overlap(
        "frame", "moving_sash",
        reason="Moving sash stiles ride in the front jamb track grooves (retained insertion).",
    )
    # Fixed sash sits in the rear track
    ctx.allow_overlap(
        "frame", "fixed_sash",
        reason="Fixed sash stiles sit in the rear jamb track grooves (retained).",
    )

    # Sashes overlap at the meeting stile (different Y planes)
    ctx.allow_overlap(
        "fixed_sash", "moving_sash",
        reason="Sashes overlap at the meeting stile; they ride in offset Y planes.",
    )

    # Latch base seated on moving sash
    ctx.allow_overlap(
        "moving_sash", "moving_sash",
        elem_a="latch_base",
        elem_b="moving_sash_frame",
        reason="Latch base is mounted (seated) onto the moving sash meeting stile.",
    )

    # Rollers seated on moving sash bottom rail
    for roller_name in ("roller_0", "roller_1"):
        ctx.allow_overlap(
            "moving_sash", "moving_sash",
            elem_a=roller_name,
            elem_b="moving_sash_frame",
            reason="Roller block is seated on the moving sash bottom rail.",
        )

    # Vent panel glass captured under vent frame
    ctx.allow_overlap(
        "vent_panel", "vent_panel",
        elem_a="vent_panel_glass",
        elem_b="vent_panel_frame",
        reason="Vent glass is captured under the vent panel frame border.",
    )

    # Vent panel seated inside the fixed sash lite opening
    ctx.allow_overlap(
        "fixed_sash", "vent_panel",
        reason="Vent panel frame is seated inside the fixed sash lower-left lite opening.",
    )
    # Proof: vent panel is within the fixed sash footprint
    ctx.expect_within(
        vent_panel, fixed_sash, axes="xz",
        margin=0.005,
        name="vent panel stays within fixed sash bounds",
    )

    # --- Closed pose (q=0): sashes in place, window reads shut ---
    with ctx.pose({j_slide: 0.0}):
        f_aabb = ctx.part_world_aabb(frame)
        fix_aabb = ctx.part_world_aabb(fixed_sash)
        mov_aabb = ctx.part_world_aabb(moving_sash)

        # Frame is the widest/tallest element
        frame_w = f_aabb[1][0] - f_aabb[0][0]
        sash_w = mov_aabb[1][0] - mov_aabb[0][0]
        ctx.check(
            "frame spans wider than a sash",
            frame_w > sash_w + 0.05,
            details=f"frame_w={frame_w:.3f}, sash_w={sash_w:.3f}",
        )

        # Sill at/near z=0
        ctx.check(
            "frame sill near z=0",
            abs(f_aabb[0][2]) < 0.01 and f_aabb[1][2] > 0.8,
            details=f"frame z range=({f_aabb[0][2]:.3f}, {f_aabb[1][2]:.3f})",
        )

        # Fixed sash is on the left, moving sash on the right
        fix_cx = (fix_aabb[0][0] + fix_aabb[1][0]) / 2.0
        mov_cx = (mov_aabb[0][0] + mov_aabb[1][0]) / 2.0
        ctx.check(
            "fixed sash left of moving sash",
            fix_cx < mov_cx - 0.10,
            details=f"fixed_cx={fix_cx:.3f}, moving_cx={mov_cx:.3f}",
        )

        # Sashes overlap in X at the meeting stile (closed)
        ctx.check(
            "sashes overlap in X when closed",
            fix_aabb[1][0] > mov_aabb[0][0] + 0.005,
            details=f"fixed_right={fix_aabb[1][0]:.3f}, moving_left={mov_aabb[0][0]:.3f}",
        )

        # Sashes are in offset Y planes
        fix_cy = (fix_aabb[0][1] + fix_aabb[1][1]) / 2.0
        mov_cy = (mov_aabb[0][1] + mov_aabb[1][1]) / 2.0
        ctx.check(
            "sashes ride in offset Y planes",
            abs(fix_cy - mov_cy) > 0.015,
            details=f"fixed_cy={fix_cy:.3f}, moving_cy={mov_cy:.3f}",
        )

        rest_mov_cx = mov_cx

    # --- HERO: moving sash slides LEFT (opens) ---
    travel = SLIDE_TRAVEL * 0.80
    with ctx.pose({j_slide: travel}):
        op_aabb = ctx.part_world_aabb(moving_sash)
        op_cx = (op_aabb[0][0] + op_aabb[1][0]) / 2.0
        ctx.check(
            "moving sash slides left when opened",
            op_cx < rest_mov_cx - travel * 0.8,
            details=f"rest_cx={rest_mov_cx:.3f}, opened_cx={op_cx:.3f}, travel={travel:.3f}",
        )
        # Still retained in the frame
        ctx.expect_overlap(
            moving_sash, frame, axes="z", min_overlap=0.10,
            name="moving sash retained in frame when open",
        )

    # --- Latch rotates (revolute joint works) ---
    with ctx.pose({j_latch: 0.0}):
        latch_rest = ctx.part_world_aabb(latch)
    with ctx.pose({j_latch: 1.2}):
        latch_open = ctx.part_world_aabb(latch)
    if latch_rest is not None and latch_open is not None:
        rest_cx = (latch_rest[0][0] + latch_rest[1][0]) / 2.0
        open_cx = (latch_open[0][0] + latch_open[1][0]) / 2.0
        # The latch lever should move when rotated
        ctx.check(
            "latch lever rotates on its joint",
            abs(rest_cx - open_cx) > 0.005 or abs(
                (latch_rest[0][2] + latch_rest[1][2]) / 2.0
                - (latch_open[0][2] + latch_open[1][2]) / 2.0
            ) > 0.005,
            details=f"rest_cx={rest_cx:.4f}, open_cx={open_cx:.4f}",
        )

    # --- Roller blocks exist at the bottom of the moving sash ---
    for roller_name in ("roller_0", "roller_1"):
        roller_aabb = ctx.part_element_world_aabb(moving_sash, elem=roller_name)
        if roller_aabb is not None:
            mov_aabb = ctx.part_world_aabb(moving_sash)
            # Rollers should be near the bottom of the sash
            ctx.check(
                f"{roller_name} near bottom of moving sash",
                roller_aabb[0][2] < mov_aabb[0][2] + 0.06,
                details=f"roller_min_z={roller_aabb[0][2]:.3f}, sash_min_z={mov_aabb[0][2]:.3f}",
            )

    # --- Vent panel exists ---
    vent_aabb = ctx.part_world_aabb(vent_panel)
    if vent_aabb is not None:
        fix_aabb = ctx.part_world_aabb(fixed_sash)
        # Vent panel is within the fixed sash footprint
        ctx.check(
            "vent panel within fixed sash width",
            vent_aabb[0][0] > fix_aabb[0][0] - 0.01 and vent_aabb[1][0] < fix_aabb[1][0] + 0.01,
            details=f"vent x=({vent_aabb[0][0]:.3f},{vent_aabb[1][0]:.3f}), fixed x=({fix_aabb[0][0]:.3f},{fix_aabb[1][0]:.3f})",
        )

    # --- Overlap stile: moving sash has a wider left stile ---
    # The moving sash left edge should show a wider stile (the overlap stile).
    # We check that the moving sash glass is offset from the left edge more than
    # a regular stile width would suggest.
    mov_glass_aabb = ctx.part_element_world_aabb(moving_sash, elem="moving_sash_glass")
    mov_frame_aabb = ctx.part_element_world_aabb(moving_sash, elem="moving_sash_frame")
    if mov_glass_aabb is not None and mov_frame_aabb is not None:
        stile_width_left = mov_glass_aabb[0][0] - mov_frame_aabb[0][0]
        ctx.check(
            "moving sash overlap stile wider than regular stile",
            stile_width_left > SASH_STILE + 0.005,
            details=f"left_stile_width={stile_width_left:.4f}, regular_stile={SASH_STILE:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
