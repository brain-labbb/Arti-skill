from __future__ import annotations

# Three-panel sliding window variant: white frame, three panels across the
# width with a wider fixed center pane. The right-side sash slides vertically
# upward on a prismatic joint. Two roller blocks at the sash bottom and a
# visible overlap stile where the sliding sash crosses the center pane.
#
# Coordinate convention:
#   +Z is up. Window stands vertically: height along +Z, width along X,
#   frame depth along Y. Sill at z=0, head at z=WIN_H.
#
# Panel layout (left to right):
#   LEFT fixed pane | mullion | CENTER wider fixed pane | mullion | RIGHT sliding sash
#
# Articulation:
#   RIGHT sash is PRISMATIC, axis (0,0,1): positive q slides it UP (opens).

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

WIN_W = 1.10          # overall window width (X)
WIN_H = 1.52          # overall window height (Z), sill at z=0
FRAME_FACE = 0.058    # outer frame member face width (X/Z)
FRAME_DEPTH = 0.100   # outer frame jamb depth (Y)

# Clear opening inside the outer frame
OPEN_W = WIN_W - 2 * FRAME_FACE   # clear width ~0.984
OPEN_H = WIN_H - 2 * FRAME_FACE   # clear height ~1.404
OPEN_X0 = -WIN_W / 2.0 + FRAME_FACE
OPEN_X1 = WIN_W / 2.0 - FRAME_FACE
OPEN_Z0 = FRAME_FACE
OPEN_Z1 = WIN_H - FRAME_FACE

# Vertical mullion bars separating the three panel sections
MULLION_W = 0.038     # mullion face width (X)

# Panel widths (within the clear opening, between mullions)
# Total available = OPEN_W - 2*MULLION_W
AVAIL_W = OPEN_W - 2 * MULLION_W   # ~0.908
CENTER_W = AVAIL_W * 0.50          # center panel: 50% (~0.454)
SIDE_W = (AVAIL_W - CENTER_W) / 2  # side panels: 25% each (~0.227)

# Panel X positions (left edges)
LEFT_X0 = OPEN_X0
LEFT_X1 = LEFT_X0 + SIDE_W
MUL1_X0 = LEFT_X1
MUL1_X1 = MUL1_X0 + MULLION_W
CENTER_X0 = MUL1_X1
CENTER_X1 = CENTER_X0 + CENTER_W
MUL2_X0 = CENTER_X1
MUL2_X1 = MUL2_X0 + MULLION_W
RIGHT_X0 = MUL2_X1
RIGHT_X1 = OPEN_X1

# Glass/sash depth
GLASS_T = 0.006       # glazing thickness
FIXED_FRAME_D = 0.030 # fixed pane frame depth (Y)

# Sliding sash dimensions
SASH_RAIL = 0.048     # sash perimeter member width
SASH_STILE = 0.048    # sash vertical member width
SASH_DEPTH = 0.034    # sash thickness (Y)
SASH_W = SIDE_W - 0.006  # slight running clearance
SASH_H = OPEN_H - 0.008  # sash height (full opening height minus clearance)

# Overlap stile: a vertical strip on the left edge of the sliding sash that
# extends past the sash frame to overlap the center pane
OVERLAP_STILE_W = 0.028   # width of the overlap extension
OVERLAP_STILE_T = 0.018   # thickness (Y) of overlap stile

# Roller blocks at bottom of sliding sash
ROLLER_W = 0.032      # roller block width (X)
ROLLER_H = 0.014      # roller block height (Z)
ROLLER_D = 0.022      # roller block depth (Y)

# Side track for sliding sash
TRACK_W = 0.016
TRACK_DEPTH = 0.028

# Y plane for the sliding sash (slightly interior)
SASH_Y = -0.012
FIXED_Y = 0.0         # fixed panes centered in frame depth

# Sliding sash closed position (bottom of sash near sill)
SASH_BOTTOM_Z = OPEN_Z0 + 0.004

# Muntin grid for sliding sash: 2 columns x 2 rows
MUNTIN_W = 0.020
SASH_COLS = 2
SASH_ROWS = 2

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

FRAME_RGBA = (0.945, 0.945, 0.945, 1.0)   # white painted frame
SASH_RGBA = (0.965, 0.965, 0.965, 1.0)    # white sash
GLASS_RGBA = (0.30, 0.36, 0.42, 0.34)     # cool dark-tinted glass
ROLLER_RGBA = (0.25, 0.25, 0.27, 1.0)     # dark gray roller blocks
MULLION_RGBA = (0.93, 0.93, 0.93, 1.0)    # white mullion


# ---------------------------------------------------------------------------
# Frame geometry (CadQuery) - outer perimeter with opening + track groove
# ---------------------------------------------------------------------------

def _build_frame_shape() -> cq.Workplane:
    """Outer frame: perimeter slab with central opening cut out,
    plus a vertical track groove on the right jamb for the sliding sash."""
    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, WIN_H / 2.0))
        .box(WIN_W, FRAME_DEPTH, WIN_H)
    )
    # Cut central opening
    opening = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, (OPEN_Z0 + OPEN_Z1) / 2.0))
        .box(OPEN_W, FRAME_DEPTH + 0.02, OPEN_H)
    )
    frame = outer.cut(opening)

    # Vertical track groove on right jamb for sliding sash
    groove_x = FRAME_FACE * 0.5
    cx = OPEN_X1 + groove_x / 2.0
    groove = (
        cq.Workplane("XY")
        .transformed(offset=(cx, SASH_Y, (OPEN_Z0 + OPEN_Z1) / 2.0))
        .box(groove_x, TRACK_DEPTH, OPEN_H)
    )
    frame = frame.cut(groove)

    return frame


# ---------------------------------------------------------------------------
# Vertical mullion bars (part of frame)
# ---------------------------------------------------------------------------

def _build_mullions_shape() -> cq.Workplane:
    """Two vertical mullion bars spanning the opening height, separating
    the three panel sections."""
    m1_cx = (MUL1_X0 + MUL1_X1) / 2.0
    m2_cx = (MUL2_X0 + MUL2_X1) / 2.0
    mid_z = (OPEN_Z0 + OPEN_Z1) / 2.0

    mul1 = (
        cq.Workplane("XY")
        .transformed(offset=(m1_cx, 0.0, mid_z))
        .box(MULLION_W, FRAME_DEPTH * 0.85, OPEN_H)
    )
    mul2 = (
        cq.Workplane("XY")
        .transformed(offset=(m2_cx, 0.0, mid_z))
        .box(MULLION_W, FRAME_DEPTH * 0.85, OPEN_H)
    )
    return mul1.union(mul2)


# ---------------------------------------------------------------------------
# Fixed pane geometry (simple frame + glass for left and center panels)
# ---------------------------------------------------------------------------

def _build_fixed_frame_shape(width: float) -> cq.Workplane:
    """Thin perimeter frame for a fixed pane. Local frame: centered on X,
    Z from 0 to OPEN_H, Y centered at 0."""
    rail = 0.032
    depth = FIXED_FRAME_D
    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, OPEN_H / 2.0))
        .box(width, depth, OPEN_H)
    )
    # Cut inner opening
    inner_w = width - 2 * rail
    inner_h = OPEN_H - 2 * rail
    inner = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, OPEN_H / 2.0))
        .box(inner_w, depth + 0.02, inner_h)
    )
    return outer.cut(inner)


def _build_fixed_glass_shape(width: float) -> cq.Workplane:
    """Single glass pane for a fixed panel."""
    rail = 0.032
    rebate = 0.004
    inner_w = width - 2 * rail + 2 * rebate
    inner_h = OPEN_H - 2 * rail + 2 * rebate
    pane = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, OPEN_H / 2.0))
        .box(inner_w, GLASS_T, inner_h)
    )
    return pane


# ---------------------------------------------------------------------------
# Sliding sash geometry (frame + muntin grid + glass + overlap stile)
# ---------------------------------------------------------------------------

def _build_sash_frame_shape() -> cq.Workplane:
    """Sliding sash: perimeter frame with 2x2 muntin grid.
    Local frame: centered X, Z from 0 to SASH_H, Y centered at 0."""
    w = SASH_W
    h = SASH_H
    rail = SASH_RAIL
    stile = SASH_STILE
    d = SASH_DEPTH

    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, h / 2.0))
        .box(w, d, h)
    )

    # Inner glazed region
    in_x0 = -w / 2.0 + stile
    in_x1 = w / 2.0 - stile
    in_z0 = rail
    in_z1 = h - rail
    inner_w = in_x1 - in_x0
    inner_h = in_z1 - in_z0

    # Muntin grid: 2 cols x 2 rows
    col_line = (in_x0 + in_x1) / 2.0
    row_line = (in_z0 + in_z1) / 2.0

    x_edges = [in_x0, col_line, in_x1]
    z_edges = [in_z0, row_line, in_z1]
    half_m = MUNTIN_W / 2.0

    sash = outer
    for ci in range(SASH_COLS):
        for ri in range(SASH_ROWS):
            lx0 = x_edges[ci] + (half_m if ci > 0 else 0.0)
            lx1 = x_edges[ci + 1] - (half_m if ci < SASH_COLS - 1 else 0.0)
            lz0 = z_edges[ri] + (half_m if ri > 0 else 0.0)
            lz1 = z_edges[ri + 1] - (half_m if ri < SASH_ROWS - 1 else 0.0)
            lite = (
                cq.Workplane("XY")
                .transformed(offset=((lx0 + lx1) / 2.0, 0.0, (lz0 + lz1) / 2.0))
                .box(lx1 - lx0, d + 0.02, lz1 - lz0)
            )
            sash = sash.cut(lite)

    return sash


def _build_sash_glass_shape() -> cq.Workplane:
    """Four glass panes for the 2x2 lite grid."""
    w = SASH_W
    h = SASH_H
    rail = SASH_RAIL
    stile = SASH_STILE
    rebate = 0.004

    in_x0 = -w / 2.0 + stile
    in_x1 = w / 2.0 - stile
    in_z0 = rail
    in_z1 = h - rail
    col_line = (in_x0 + in_x1) / 2.0
    row_line = (in_z0 + in_z1) / 2.0

    x_edges = [in_x0, col_line, in_x1]
    z_edges = [in_z0, row_line, in_z1]
    half_m = MUNTIN_W / 2.0

    panes = None
    for ci in range(SASH_COLS):
        for ri in range(SASH_ROWS):
            lx0 = x_edges[ci] + (half_m if ci > 0 else 0.0) - rebate
            lx1 = x_edges[ci + 1] - (half_m if ci < SASH_COLS - 1 else 0.0) + rebate
            lz0 = z_edges[ri] + (half_m if ri > 0 else 0.0) - rebate
            lz1 = z_edges[ri + 1] - (half_m if ri < SASH_ROWS - 1 else 0.0) + rebate
            pane = (
                cq.Workplane("XY")
                .transformed(offset=((lx0 + lx1) / 2.0, 0.0, (lz0 + lz1) / 2.0))
                .box(lx1 - lx0, GLASS_T, lz1 - lz0)
            )
            panes = pane if panes is None else panes.union(pane)
    return panes


def _build_overlap_stile_shape() -> cq.Workplane:
    """Overlap stile: a vertical strip on the left edge of the sliding sash
    that extends past the sash frame to overlap the adjacent center pane.
    Local frame matches sash local frame."""
    # Position: at the left edge of sash, extending further left
    stile_x = -SASH_W / 2.0 - OVERLAP_STILE_W / 2.0
    stile_z = SASH_H / 2.0
    # Slightly proud on the interior face
    stile_y = -(SASH_DEPTH / 2.0 + OVERLAP_STILE_T / 2.0 - 0.004)
    stile = (
        cq.Workplane("XY")
        .transformed(offset=(stile_x, stile_y, stile_z))
        .box(OVERLAP_STILE_W, OVERLAP_STILE_T, SASH_H - 0.010)
    )
    return stile


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_panel_sliding_window")

    model.material("frame", rgba=FRAME_RGBA)
    model.material("sash", rgba=SASH_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("roller", rgba=ROLLER_RGBA)
    model.material("mullion", rgba=MULLION_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame_shell"),
        material="frame",
        name="frame_shell",
    )
    # Vertical mullions as part of the frame
    frame.visual(
        mesh_from_cadquery(_build_mullions_shape(), "mullions"),
        material="mullion",
        name="mullions",
    )

    # --- Left fixed pane ---
    left_pane = model.part("left_pane")
    left_cx = (LEFT_X0 + LEFT_X1) / 2.0
    # Geometry is built in local coords: X centered, Z from 0 to OPEN_H.
    # Articulation places part frame at (left_cx, 0, OPEN_Z0).
    left_pane.visual(
        mesh_from_cadquery(_build_fixed_frame_shape(SIDE_W), "left_frame"),
        material="frame",
        name="left_frame",
    )
    left_pane.visual(
        mesh_from_cadquery(_build_fixed_glass_shape(SIDE_W), "left_glass"),
        material="glass",
        name="left_glass",
    )

    # --- Center fixed pane (wider) ---
    center_pane = model.part("center_pane")
    center_cx = (CENTER_X0 + CENTER_X1) / 2.0
    center_pane.visual(
        mesh_from_cadquery(_build_fixed_frame_shape(CENTER_W), "center_frame"),
        material="frame",
        name="center_frame",
    )
    center_pane.visual(
        mesh_from_cadquery(_build_fixed_glass_shape(CENTER_W), "center_glass"),
        material="glass",
        name="center_glass",
    )

    # --- Fixed articulations: attach fixed panes to frame ---
    model.articulation(
        "frame_to_left_pane",
        ArticulationType.FIXED,
        parent="frame",
        child="left_pane",
        origin=Origin(xyz=(left_cx, FIXED_Y, OPEN_Z0)),
    )
    model.articulation(
        "frame_to_center_pane",
        ArticulationType.FIXED,
        parent="frame",
        child="center_pane",
        origin=Origin(xyz=(center_cx, FIXED_Y, OPEN_Z0)),
    )

    # --- Right sliding sash ---
    sash = model.part("sliding_sash")
    sash.visual(
        mesh_from_cadquery(_build_sash_frame_shape(), "sash_frame"),
        material="sash",
        name="sash_frame",
    )
    sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(), "sash_glass"),
        material="glass",
        name="sash_glass",
    )
    # Overlap stile on the left edge of the sash
    sash.visual(
        mesh_from_cadquery(_build_overlap_stile_shape(), "overlap_stile"),
        material="sash",
        name="overlap_stile",
    )
    # Two roller blocks at the bottom of the sash
    roller_y = -(SASH_DEPTH / 2.0 + ROLLER_D / 2.0 - 0.003)
    roller_z = ROLLER_H / 2.0
    roller_x_offset = SASH_W / 2.0 - ROLLER_W / 2.0 - 0.010
    for i, sign in enumerate((-1.0, +1.0)):
        sash.visual(
            Box((ROLLER_W, ROLLER_D, ROLLER_H)),
            origin=Origin(xyz=(sign * roller_x_offset, roller_y, roller_z)),
            material="roller",
            name=f"roller_{i}",
        )

    # ----- Articulation: sliding sash moves UP on prismatic joint -----
    right_cx = (RIGHT_X0 + RIGHT_X1) / 2.0
    model.articulation(
        "frame_to_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="sliding_sash",
        origin=Origin(xyz=(right_cx, SASH_Y, SASH_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=50.0, velocity=0.20, lower=0.0, upper=SASH_H * 0.45
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    left_pane = object_model.get_part("left_pane")
    center_pane = object_model.get_part("center_pane")
    sash = object_model.get_part("sliding_sash")
    j_sash = object_model.get_articulation("frame_to_sash")

    # --- Intentional overlaps ---
    # Glass panes tuck under the sash muntin lips (captured glass)
    ctx.allow_overlap(
        "sliding_sash", "sliding_sash",
        elem_a="sash_glass", elem_b="sash_frame",
        reason="Glass panes rebated under sash rails/muntins (captured glass).",
    )
    # Overlap stile is part of the sash, extends past to overlap center pane region
    ctx.allow_overlap(
        "sliding_sash", "sliding_sash",
        elem_a="overlap_stile", elem_b="sash_frame",
        reason="Overlap stile is mounted on the sash frame left edge.",
    )
    # Roller blocks seated into the sash bottom rail
    for roller_name in ("roller_0", "roller_1"):
        ctx.allow_overlap(
            "sliding_sash", "sliding_sash",
            elem_a=roller_name, elem_b="sash_frame",
            reason="Roller block seated into the sash bottom rail.",
        )
    # Sliding sash rides in the track groove cut into the right jamb
    ctx.allow_overlap(
        "frame", "sliding_sash",
        reason="Sliding sash stiles ride in the jamb track groove (retained insertion).",
    )
    # Fixed panes are seated within the frame opening
    ctx.allow_overlap(
        "frame", "left_pane",
        reason="Left fixed pane is seated within the frame opening.",
    )
    ctx.allow_overlap(
        "frame", "center_pane",
        reason="Center fixed pane is seated within the frame opening.",
    )
    # Overlap stile may overlap center pane when closed (it's designed to)
    ctx.allow_overlap(
        "sliding_sash", "center_pane",
        elem_a="overlap_stile", elem_b="center_frame",
        reason="Overlap stile extends past the sash to overlap the center pane edge (sliding window seal).",
    )

    # --- Closed pose (q=0): sash seated, window reads shut ---
    with ctx.pose({j_sash: 0.0}):
        f_aabb = ctx.part_world_aabb(frame)
        sash_aabb = ctx.part_world_aabb(sash)

        # Frame spans wider than the sash
        frame_w = f_aabb[1][0] - f_aabb[0][0]
        sash_w = sash_aabb[1][0] - sash_aabb[0][0]
        ctx.check(
            "frame spans wider than sliding sash",
            frame_w > sash_w + 0.3,
            details=f"frame_w={frame_w:.3f}, sash_w={sash_w:.3f}",
        )
        # Frame sill near z=0
        ctx.check(
            "frame sill near z=0",
            abs(f_aabb[0][2]) < 0.01 and f_aabb[1][2] > 1.0,
            details=f"frame z=({f_aabb[0][2]:.3f}, {f_aabb[1][2]:.3f})",
        )
        # Sash is within frame width
        ctx.check(
            "sash within frame width at rest",
            sash_aabb[0][0] > f_aabb[0][0] and sash_aabb[1][0] < f_aabb[1][0],
            details=f"sash x=({sash_aabb[0][0]:.3f},{sash_aabb[1][0]:.3f})",
        )
        # Sash is to the right of center pane
        cp_aabb = ctx.part_world_aabb(center_pane)
        sash_cx = (sash_aabb[0][0] + sash_aabb[1][0]) / 2.0
        cp_cx = (cp_aabb[0][0] + cp_aabb[1][0]) / 2.0
        ctx.check(
            "sliding sash right of center pane",
            sash_cx > cp_cx + 0.1,
            details=f"sash_cx={sash_cx:.3f}, center_cx={cp_cx:.3f}",
        )
        # Center pane is wider than the sliding sash
        cp_w = cp_aabb[1][0] - cp_aabb[0][0]
        ctx.check(
            "center pane wider than sliding sash",
            cp_w > sash_w + 0.05,
            details=f"center_w={cp_w:.3f}, sash_w={sash_w:.3f}",
        )
        rest_sash_cz = (sash_aabb[0][2] + sash_aabb[1][2]) / 2.0

    # --- Opened pose: sash slides UP ---
    travel = SASH_H * 0.40
    with ctx.pose({j_sash: travel}):
        op_aabb = ctx.part_world_aabb(sash)
        op_cz = (op_aabb[0][2] + op_aabb[1][2]) / 2.0
        ctx.check(
            "sliding sash moves up when opened",
            op_cz > rest_sash_cz + travel * 0.8,
            details=f"rest_cz={rest_sash_cz:.3f}, opened_cz={op_cz:.3f}, travel={travel:.3f}",
        )
        # Sash still retained in frame
        ctx.expect_overlap(
            sash, frame, axes="x", min_overlap=0.05,
            name="sash retained in frame when open",
        )

    # --- Roller blocks exist at bottom of sash ---
    roller_0_aabb = ctx.part_element_world_aabb(sash, elem="roller_0")
    roller_1_aabb = ctx.part_element_world_aabb(sash, elem="roller_1")
    ctx.check(
        "two roller blocks present",
        roller_0_aabb is not None and roller_1_aabb is not None,
        details="roller_0 or roller_1 missing",
    )
    if roller_0_aabb is not None and roller_1_aabb is not None:
        # Rollers are near the bottom of the sash
        sash_aabb = ctx.part_world_aabb(sash)
        r0_bot = roller_0_aabb[0][2]
        r1_bot = roller_1_aabb[0][2]
        ctx.check(
            "rollers near sash bottom",
            r0_bot < sash_aabb[0][2] + 0.04 and r1_bot < sash_aabb[0][2] + 0.04,
            details=f"r0_bot={r0_bot:.3f}, r1_bot={r1_bot:.3f}, sash_bot={sash_aabb[0][2]:.3f}",
        )

    # --- Overlap stile exists on sash left edge ---
    stile_aabb = ctx.part_element_world_aabb(sash, elem="overlap_stile")
    ctx.check(
        "overlap stile present on sash",
        stile_aabb is not None,
        details="overlap_stile visual not found",
    )
    if stile_aabb is not None:
        # Stile is to the left of the sash center
        stile_cx = (stile_aabb[0][0] + stile_aabb[1][0]) / 2.0
        sash_aabb = ctx.part_world_aabb(sash)
        sash_cx = (sash_aabb[0][0] + sash_aabb[1][0]) / 2.0
        ctx.check(
            "overlap stile on left side of sash",
            stile_cx < sash_cx - 0.02,
            details=f"stile_cx={stile_cx:.3f}, sash_cx={sash_cx:.3f}",
        )

    # --- Three panels exist ---
    lp_aabb = ctx.part_world_aabb(left_pane)
    ctx.check(
        "left fixed pane exists",
        lp_aabb is not None and (lp_aabb[1][0] - lp_aabb[0][0]) > 0.1,
        details="left pane missing or too narrow",
    )
    ctx.check(
        "center fixed pane exists and is wider",
        cp_aabb is not None and (cp_aabb[1][0] - cp_aabb[0][0]) > 0.35,
        details="center pane missing or too narrow",
    )

    # --- Articulation is prismatic (non-fixed joint) ---
    ctx.check(
        "sliding sash has prismatic joint",
        j_sash.articulation_type == ArticulationType.PRISMATIC,
        details=f"joint type={j_sash.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
