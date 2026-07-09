from __future__ import annotations

# Sliding window variant: slim vinyl frame with bevelled corners, one fixed
# upper sash and one lower sash that slides upward on a prismatic joint.
# Two roller blocks at the bottom of the moving sash. Sill lip with drainage
# slots cut through it.
#
# Coordinate convention:
#   +Z is up. Window stands vertically: height along +Z, width along X, frame
#   depth along Y. The sill sits at z=0; the head is at z=WIN_H.
#   Interior is -Y, exterior is +Y.

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

WIN_W = 0.92           # overall window width (X)
WIN_H = 1.52           # overall window height (Z), sill at z=0
FRAME_FACE = 0.042     # slim vinyl rail face width
FRAME_DEPTH = 0.078    # slim frame jamb depth (Y)

# Clear opening inside the outer frame
OPEN_W = WIN_W - 2 * FRAME_FACE
OPEN_H = WIN_H - 2 * FRAME_FACE
OPEN_X0 = -WIN_W / 2.0 + FRAME_FACE
OPEN_X1 = WIN_W / 2.0 - FRAME_FACE
OPEN_Z0 = FRAME_FACE
OPEN_Z1 = WIN_H - FRAME_FACE

# Sash geometry
SASH_W = OPEN_W - 0.008                # running clearance to jambs
SASH_RAIL = 0.040                      # sash perimeter member width
SASH_DEPTH = 0.026                     # sash thickness (Y)
SASH_H = OPEN_H * 0.545               # each sash height
GLASS_T = 0.005                        # glazing thickness

# Y planes: lower sash on interior (-Y) track, upper sash fixed on exterior (+Y)
SASH_Y = -0.010                        # lower sash track center
UPPER_SASH_Y = +0.010                  # upper sash fixed position

# Closed-pose positions
LOWER_BOTTOM_Z = OPEN_Z0 + 0.004      # lower sash on sill, small clearance
UPPER_BOTTOM_Z = LOWER_BOTTOM_Z + SASH_H - SASH_RAIL  # overlap at meeting rail

# Muntin grid: 3 columns x 2 rows per sash
MUNTIN_W = 0.020
N_COLS = 3
N_ROWS = 2

# Track groove dimensions
TRACK_GROOVE_W = 0.016
TRACK_GROOVE_DEPTH = 0.022

# Frame corner chamfer
FRAME_CHAMFER = 0.005

# Sill lip: protrudes outward (+Y) from the frame bottom
SILL_LIP_EXT = 0.032                   # how far the sill extends outward
SILL_LIP_H = 0.015                     # sill lip thickness (Z)

# Drainage slots in sill lip
DRAIN_W = 0.036                        # slot width (X)
DRAIN_N = 3                            # number of slots

# Roller blocks at bottom of lower sash
ROLLER_W = 0.026                       # roller width (X)
ROLLER_D = 0.016                       # roller depth (Y)
ROLLER_H = 0.009                       # roller height (Z)

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

FRAME_RGBA = (0.90, 0.90, 0.88, 1.0)    # vinyl off-white frame
SASH_RGBA = (0.93, 0.93, 0.91, 1.0)     # slightly brighter vinyl sash
GLASS_RGBA = (0.28, 0.34, 0.40, 0.34)   # cool tinted glass
ROLLER_RGBA = (0.14, 0.14, 0.14, 1.0)   # dark nylon roller


# ---------------------------------------------------------------------------
# Frame geometry (CadQuery): slim vinyl with bevelled corners, sill lip,
# drainage slots, and track grooves
# ---------------------------------------------------------------------------

def _build_frame_shape() -> cq.Workplane:
    """Slim vinyl outer frame with:
    - Bevelled (chamfered) outer vertical edges
    - Central opening for the sashes
    - Track grooves in jambs for the sliding sash
    - Sill lip protruding outward (+Y) at the bottom
    - Drainage slots cut through the sill lip
    """
    # 1. Outer box
    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, WIN_H / 2.0))
        .box(WIN_W, FRAME_DEPTH, WIN_H)
    )

    # 2. Chamfer the 4 outer vertical edges (bevelled corners)
    outer = outer.edges("|Z").chamfer(FRAME_CHAMFER)

    # 3. Cut the central opening
    opening = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, (OPEN_Z0 + OPEN_Z1) / 2.0))
        .box(OPEN_W, FRAME_DEPTH + 0.02, OPEN_H)
    )
    frame = outer.cut(opening)

    # 4. Track grooves in jambs for both sash planes
    groove_x = FRAME_FACE * 0.45
    for sign, edge_x in ((+1.0, OPEN_X0), (-1.0, OPEN_X1)):
        cx = edge_x - sign * groove_x / 2.0
        for track_y in (SASH_Y, UPPER_SASH_Y):
            groove = (
                cq.Workplane("XY")
                .transformed(offset=(cx, track_y, (OPEN_Z0 + OPEN_Z1) / 2.0))
                .box(groove_x, TRACK_GROOVE_DEPTH, OPEN_H)
            )
            frame = frame.cut(groove)

    # 5. Sill lip: thin shelf protruding outward (+Y) at the bottom
    sill_y_center = FRAME_DEPTH / 2.0 + SILL_LIP_EXT / 2.0
    sill_z_center = SILL_LIP_H / 2.0
    sill = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, sill_y_center, sill_z_center))
        .box(WIN_W - 4 * FRAME_CHAMFER, SILL_LIP_EXT, SILL_LIP_H)
    )
    frame = frame.union(sill)

    # 6. Drainage slots: rectangular cuts through the sill lip
    drain_spacing = OPEN_W / (DRAIN_N + 1)
    for i in range(DRAIN_N):
        dx = OPEN_X0 + (i + 1) * drain_spacing
        slot = (
            cq.Workplane("XY")
            .transformed(offset=(dx, sill_y_center, sill_z_center))
            .box(DRAIN_W, SILL_LIP_EXT + 0.01, SILL_LIP_H + 0.006)
        )
        frame = frame.cut(slot)

    return frame


# ---------------------------------------------------------------------------
# Sash frame geometry (CadQuery): perimeter ring + 6-lite muntin grid
# ---------------------------------------------------------------------------

def _build_sash_frame_shape() -> cq.Workplane:
    """One sash: perimeter ring with 3x2 muntin grid.
    Local frame: X = -SASH_W/2..+SASH_W/2, Z = 0..SASH_H, Y centered at 0.
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
    """Six thin glass panes filling the lite openings, rebated under the
    muntin/rail lips so the glass reads as captured."""
    w = SASH_W
    h = SASH_H
    r = SASH_RAIL
    rebate = 0.004

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
    model = ArticulatedObject(name="sliding_window")

    model.material("frame", rgba=FRAME_RGBA)
    model.material("sash", rgba=SASH_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("roller", rgba=ROLLER_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material="frame",
        name="frame_shell",
    )

    # --- Upper sash (fixed in place) ---
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

    # --- Lower sash (slides upward) ---
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

    # Roller blocks at bottom of lower sash (near left and right stiles)
    roller_x_off = SASH_W / 2.0 - ROLLER_W / 2.0 - 0.024
    for side, sx in (("left", -roller_x_off), ("right", roller_x_off)):
        lower.visual(
            Box((ROLLER_W, ROLLER_D, ROLLER_H)),
            origin=Origin(xyz=(sx, 0.0, -ROLLER_H * 0.35)),
            material="roller",
            name=f"roller_{side}",
        )

    # ----- Articulations -----

    # Upper sash: FIXED (does not move in a single-slider window)
    model.articulation(
        "frame_to_upper_sash",
        ArticulationType.FIXED,
        parent="frame",
        child="upper_sash",
        origin=Origin(xyz=(0.0, UPPER_SASH_Y, UPPER_BOTTOM_Z)),
    )

    # Lower sash: PRISMATIC, axis (0,0,1), positive q slides UP
    max_travel = SASH_H * 0.42
    model.articulation(
        "frame_to_lower_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="lower_sash",
        origin=Origin(xyz=(0.0, SASH_Y, LOWER_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0, velocity=0.3, lower=0.0, upper=max_travel
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
    j_lower = object_model.get_articulation("frame_to_lower_sash")
    j_upper = object_model.get_articulation("frame_to_upper_sash")

    # --- Intentional overlaps ---
    # Glass panes tuck under sash muntin/rail lips (captured glazing)
    for sash_name in ("lower_sash", "upper_sash"):
        ctx.allow_overlap(
            sash_name, sash_name,
            elem_a=f"{sash_name}_glass",
            elem_b=f"{sash_name}_frame",
            reason="Glass panes are rebated under the sash rails/muntins so they read as captured, not floating.",
        )

    # Lower sash rides in jamb track grooves
    ctx.allow_overlap(
        "frame", "lower_sash",
        reason="Lower sash stiles ride in the jamb track grooves (retained insertion).",
    )

    # Upper sash is fixed in the frame; top rail retained by the frame head
    ctx.allow_overlap(
        "frame", "upper_sash",
        reason="Upper sash is seated in the frame opening with top rail retained by the frame head.",
    )

    # Two sashes overlap at the meeting rail (different Y planes)
    ctx.allow_overlap(
        "lower_sash", "upper_sash",
        reason="Sashes overlap by one rail at the central meeting rail; they ride in offset Y planes.",
    )

    # Roller blocks are mounted into the lower sash bottom rail
    for roller_name in ("roller_left", "roller_right"):
        ctx.allow_overlap(
            "lower_sash", "lower_sash",
            elem_a=roller_name,
            elem_b="lower_sash_frame",
            reason=f"Roller block ({roller_name}) is mounted into the lower sash bottom rail.",
        )

    # --- Frame geometry: slim vinyl with sill lip ---
    f_aabb = ctx.part_world_aabb(frame)
    frame_w = f_aabb[1][0] - f_aabb[0][0]
    frame_h = f_aabb[1][2] - f_aabb[0][2]
    frame_y_depth = f_aabb[1][1] - f_aabb[0][1]

    ctx.check(
        "frame has window-scale proportions",
        frame_w > 0.80 and frame_h > 1.40,
        details=f"w={frame_w:.3f}, h={frame_h:.3f}",
    )

    # Sill lip extends beyond the main frame body in +Y
    ctx.check(
        "sill lip protrudes beyond frame depth",
        frame_y_depth > FRAME_DEPTH + SILL_LIP_EXT * 0.4,
        details=f"frame_y_depth={frame_y_depth:.4f}, expected>{FRAME_DEPTH + SILL_LIP_EXT * 0.4:.4f}",
    )

    # Frame slim rails: face width is noticeably less than the parent (0.060)
    ctx.check(
        "slim vinyl frame rails",
        FRAME_FACE < 0.050,
        details=f"FRAME_FACE={FRAME_FACE:.3f}",
    )

    # --- Roller blocks exist on lower sash ---
    roller_left = lower.get_visual("roller_left")
    roller_right = lower.get_visual("roller_right")
    ctx.check("roller_left visual exists", roller_left is not None, "roller_left not found")
    ctx.check("roller_right visual exists", roller_right is not None, "roller_right not found")

    # --- Joint type checks ---
    ctx.check(
        "lower sash joint is prismatic",
        j_lower.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={j_lower.articulation_type}",
    )
    ctx.check(
        "upper sash joint is fixed",
        j_upper.articulation_type == ArticulationType.FIXED,
        details=f"type={j_upper.articulation_type}",
    )

    # --- Closed pose (q=0): lower sash seated, window reads shut ---
    with ctx.pose({j_lower: 0.0}):
        lo_aabb = ctx.part_world_aabb(lower)
        up_aabb = ctx.part_world_aabb(upper)

        lo_cz = (lo_aabb[0][2] + lo_aabb[1][2]) / 2.0
        up_cz = (up_aabb[0][2] + up_aabb[1][2]) / 2.0

        ctx.check(
            "lower sash below upper sash at rest",
            lo_cz < up_cz - 0.20,
            details=f"lo_cz={lo_cz:.3f}, up_cz={up_cz:.3f}",
        )

        # Both sashes within frame width
        ctx.check(
            "sashes within frame opening width",
            lo_aabb[0][0] > f_aabb[0][0] and lo_aabb[1][0] < f_aabb[1][0]
            and up_aabb[0][0] > f_aabb[0][0] and up_aabb[1][0] < f_aabb[1][0],
            details=f"lower x=({lo_aabb[0][0]:.3f},{lo_aabb[1][0]:.3f}) upper x=({up_aabb[0][0]:.3f},{up_aabb[1][0]:.3f})",
        )

        # Upper sash retained in frame (X overlap with frame jambs)
        ctx.expect_overlap(
            upper, frame, axes="x", min_overlap=0.05,
            name="upper sash retained in frame",
        )

        # Meeting rail: sashes overlap in Z at rest
        ctx.check(
            "sashes overlap at meeting rail",
            lo_aabb[1][2] >= up_aabb[0][2] - 1e-4,
            details=f"lower_top={lo_aabb[1][2]:.3f}, upper_bottom={up_aabb[0][2]:.3f}",
        )

        # Frame sill near z=0
        ctx.check(
            "frame sill near z=0",
            abs(f_aabb[0][2]) < 0.01 and f_aabb[1][2] > 1.0,
            details=f"frame z=({f_aabb[0][2]:.3f},{f_aabb[1][2]:.3f})",
        )

        rest_cz = lo_cz

    # --- HERO: lower sash slides UP when opened ---
    travel = SASH_H * 0.38
    with ctx.pose({j_lower: travel}):
        op_aabb = ctx.part_world_aabb(lower)
        op_cz = (op_aabb[0][2] + op_aabb[1][2]) / 2.0

        ctx.check(
            "lower sash slides upward when opened",
            op_cz > rest_cz + travel * 0.8,
            details=f"rest_cz={rest_cz:.3f}, opened_cz={op_cz:.3f}, travel={travel:.3f}",
        )

        # Sash still retained in frame
        ctx.expect_overlap(
            lower, frame, axes="x", min_overlap=0.05,
            name="lower sash retained in frame when open",
        )

    return ctx.report()


object_model = build_object_model()
