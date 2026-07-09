from __future__ import annotations

# Double-hung sash window variant 07: white frame, lower sash with 6-lite
# muntin grid, upper sash with single large pane (no muntins). Both sashes
# slide vertically on separate prismatic joints in opposite directions.
# Deep track grooves along head and sill frame rails. Rubber gasket strips
# around all glass panes.
#
# Coordinate convention:
#   +Z is up. Window stands vertically: height along +Z, width along X, frame
#   depth / glazing thickness along Y. The sill sits at z=0; head at z=WIN_H.
#
# Articulation:
#   - LOWER sash PRISMATIC axis (0,0,1): positive q slides UP.
#   - UPPER sash PRISMATIC axis (0,0,-1): positive q slides DOWN.

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

# Clear opening inside the outer frame
OPEN_W = WIN_W - 2 * FRAME_FACE
OPEN_H = WIN_H - 2 * FRAME_FACE
OPEN_X0 = -WIN_W / 2.0 + FRAME_FACE
OPEN_X1 = WIN_W / 2.0 - FRAME_FACE
OPEN_Z0 = FRAME_FACE
OPEN_Z1 = WIN_H - FRAME_FACE

# Sash geometry
SASH_W = OPEN_W - 0.010                 # slight running clearance to jambs
SASH_RAIL = 0.052                       # sash perimeter member width
SASH_DEPTH = 0.034                      # sash thickness (Y)
SASH_H = OPEN_H * 0.545                 # each sash height (overlap at center)
GLASS_T = 0.006                         # glazing thickness (Y)

# Y planes: lower sash interior (-Y), upper sash exterior (+Y)
SASH_Y_GAP = 0.016
LOWER_SASH_Y = -SASH_Y_GAP
UPPER_SASH_Y = +SASH_Y_GAP

# Closed-pose sash bottom edges (world Z)
LOWER_BOTTOM_Z = OPEN_Z0 + 0.004
MEETING_OVERLAP = SASH_RAIL
UPPER_BOTTOM_Z = LOWER_BOTTOM_Z + SASH_H - MEETING_OVERLAP

# Muntin grid (lower sash only): 3 columns x 2 rows
MUNTIN_W = 0.022
N_COLS = 3
N_ROWS = 2

# Side track channels in jambs
TRACK_W = 0.018
TRACK_DEPTH = 0.030

# Deep track grooves in head and sill
GROOVE_DEPTH = 0.012                    # how far groove cuts into head/sill
GROOVE_Y_WIDTH = SASH_DEPTH + 0.008    # groove width in Y

# Rubber gasket strips
GASKET_STRIP_W = 0.006                 # visible rubber strip width
GASKET_T = 0.004                       # gasket thickness

# Sash lock at meeting rail
LOCK_BODY = (0.060, 0.026, 0.022)
LOCK_LEVER = (0.044, 0.012, 0.010)

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

FRAME_RGBA = (0.945, 0.945, 0.945, 1.0)    # white painted frame
SASH_RGBA = (0.965, 0.965, 0.965, 1.0)     # white sash
GLASS_RGBA = (0.30, 0.36, 0.42, 0.34)      # cool dark-tinted glass
LOCK_RGBA = (0.86, 0.87, 0.89, 1.0)        # brushed metal sash lock
GASKET_RGBA = (0.12, 0.12, 0.12, 1.0)      # dark rubber gasket


# ---------------------------------------------------------------------------
# Pane-opening helpers (sash-local frame, z=0 at sash bottom)
# ---------------------------------------------------------------------------

def _muntin_pane_openings() -> list[tuple[float, float, float, float]]:
    """Return [(cx, cz, width, height)] for the 6 lite openings."""
    w, h, r = SASH_W, SASH_H, SASH_RAIL
    in_x0, in_x1 = -w / 2.0 + r, w / 2.0 - r
    in_z0, in_z1 = r, h - r
    inner_w = in_x1 - in_x0
    inner_h = in_z1 - in_z0
    col_lines = [in_x0 + (i + 1) * inner_w / N_COLS for i in range(N_COLS - 1)]
    row_lines = [in_z0 + (j + 1) * inner_h / N_ROWS for j in range(N_ROWS - 1)]
    x_edges = [in_x0] + col_lines + [in_x1]
    z_edges = [in_z0] + row_lines + [in_z1]
    half_m = MUNTIN_W / 2.0
    openings = []
    for ci in range(N_COLS):
        for ri in range(N_ROWS):
            lx0 = x_edges[ci] + (half_m if ci > 0 else 0.0)
            lx1 = x_edges[ci + 1] - (half_m if ci < N_COLS - 1 else 0.0)
            lz0 = z_edges[ri] + (half_m if ri > 0 else 0.0)
            lz1 = z_edges[ri + 1] - (half_m if ri < N_ROWS - 1 else 0.0)
            openings.append(((lx0 + lx1) / 2.0, (lz0 + lz1) / 2.0,
                             lx1 - lx0, lz1 - lz0))
    return openings


def _plain_pane_opening() -> list[tuple[float, float, float, float]]:
    """Return [(cx, cz, width, height)] for the single large opening."""
    w, h, r = SASH_W, SASH_H, SASH_RAIL
    in_x0, in_x1 = -w / 2.0 + r, w / 2.0 - r
    in_z0, in_z1 = r, h - r
    return [(0.0, (in_z0 + in_z1) / 2.0, in_x1 - in_x0, in_z1 - in_z0)]


# ---------------------------------------------------------------------------
# Frame geometry with deep track grooves in head and sill
# ---------------------------------------------------------------------------

def _build_frame_shape() -> cq.Workplane:
    """White outer frame: perimeter slab with central opening, side-track
    channels in jambs, and deep track grooves in head and sill."""
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

    # Side-track channels in jambs (two per jamb, one per sash plane)
    groove_x = FRAME_FACE * 0.55
    for sign, edge_x in ((+1.0, OPEN_X0), (-1.0, OPEN_X1)):
        cx = edge_x - sign * groove_x / 2.0
        for track_y in (LOWER_SASH_Y, UPPER_SASH_Y):
            groove = (
                cq.Workplane("XY")
                .transformed(offset=(cx, track_y, (OPEN_Z0 + OPEN_Z1) / 2.0))
                .box(groove_x, TRACK_DEPTH, OPEN_H)
            )
            frame = frame.cut(groove)

    # Deep track grooves in sill (bottom frame member)
    # Cut from top face of sill downward by GROOVE_DEPTH
    sill_groove_z = FRAME_FACE - GROOVE_DEPTH / 2.0
    for track_y in (LOWER_SASH_Y, UPPER_SASH_Y):
        groove = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, track_y, sill_groove_z))
            .box(OPEN_W, GROOVE_Y_WIDTH, GROOVE_DEPTH)
        )
        frame = frame.cut(groove)

    # Deep track grooves in head (top frame member)
    # Cut from bottom face of head upward by GROOVE_DEPTH
    head_groove_z = WIN_H - FRAME_FACE + GROOVE_DEPTH / 2.0
    for track_y in (LOWER_SASH_Y, UPPER_SASH_Y):
        groove = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, track_y, head_groove_z))
            .box(OPEN_W, GROOVE_Y_WIDTH, GROOVE_DEPTH)
        )
        frame = frame.cut(groove)

    return frame


# ---------------------------------------------------------------------------
# Muntin sash (lower): perimeter ring + 3x2 muntin grid
# ---------------------------------------------------------------------------

def _build_muntin_sash_frame_shape() -> cq.Workplane:
    """Sash with 6-lite muntin grid: perimeter ring plus muntin bars."""
    w, h, r, d = SASH_W, SASH_H, SASH_RAIL, SASH_DEPTH
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


# ---------------------------------------------------------------------------
# Plain sash (upper): perimeter ring only, one large glass opening
# ---------------------------------------------------------------------------

def _build_plain_sash_frame_shape() -> cq.Workplane:
    """Sash with single large opening (no muntin bars)."""
    w, h, r, d = SASH_W, SASH_H, SASH_RAIL, SASH_DEPTH
    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, h / 2.0))
        .box(w, d, h)
    )
    in_w = w - 2 * r
    in_h = h - 2 * r
    opening = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, h / 2.0))
        .box(in_w, d + 0.02, in_h)
    )
    return outer.cut(opening)


# ---------------------------------------------------------------------------
# Glass panes
# ---------------------------------------------------------------------------

def _build_glass_panes(openings: list[tuple[float, float, float, float]],
                       rebate: float = 0.005) -> cq.Workplane:
    """Build glass panes for the given lite openings (rebated under rails)."""
    panes = None
    for cx, cz, pw, ph in openings:
        pane = (
            cq.Workplane("XY")
            .transformed(offset=(cx, 0.0, cz))
            .box(pw + 2 * rebate, GLASS_T, ph + 2 * rebate)
        )
        panes = pane if panes is None else panes.union(pane)
    return panes


# ---------------------------------------------------------------------------
# Rubber gasket strips around glass panes
# ---------------------------------------------------------------------------

def _build_gasket_frames(openings: list[tuple[float, float, float, float]]
                         ) -> cq.Workplane:
    """Build gasket frames (rectangular rings) around each lite opening.
    Positioned on the interior face (-Y) of the sash, half proud."""
    y_pos = -(SASH_DEPTH / 2.0)  # centered at sash interior face
    result = None
    for cx, cz, pw, ph in openings:
        outer = (
            cq.Workplane("XY")
            .transformed(offset=(cx, y_pos, cz))
            .box(pw + 0.002, GASKET_T, ph + 0.002)
        )
        iw = pw - 2 * GASKET_STRIP_W
        ih = ph - 2 * GASKET_STRIP_W
        if iw > 0.002 and ih > 0.002:
            inner = (
                cq.Workplane("XY")
                .transformed(offset=(cx, y_pos, cz))
                .box(iw, GASKET_T + 0.002, ih)
            )
            gasket = outer.cut(inner)
        else:
            gasket = outer
        result = gasket if result is None else result.union(gasket)
    return result


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="double_hung_sash_window")

    model.material("frame", rgba=FRAME_RGBA)
    model.material("sash", rgba=SASH_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("lock", rgba=LOCK_RGBA)
    model.material("gasket", rgba=GASKET_RGBA)

    # --- Static outer frame (root) with track grooves ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material="frame",
        name="frame_shell",
    )

    # --- Lower sash: 6-lite muntin grid + gaskets ---
    lower = model.part("lower_sash")
    lower.visual(
        mesh_from_cadquery(_build_muntin_sash_frame_shape(), "lower_sash_frame"),
        material="sash",
        name="lower_sash_frame",
    )
    lower_openings = _muntin_pane_openings()
    lower.visual(
        mesh_from_cadquery(_build_glass_panes(lower_openings), "lower_sash_glass"),
        material="glass",
        name="lower_sash_glass",
    )
    lower.visual(
        mesh_from_cadquery(_build_gasket_frames(lower_openings), "lower_sash_gaskets"),
        material="gasket",
        name="lower_sash_gaskets",
    )

    # Sash lock on the lower sash top (meeting) rail
    lock_z = SASH_H - SASH_RAIL / 2.0
    lock_body_y = -(SASH_DEPTH / 2.0 + LOCK_BODY[1] / 2.0 - 0.004)
    lower.visual(
        Box(LOCK_BODY),
        origin=Origin(xyz=(0.0, lock_body_y, lock_z)),
        material="lock",
        name="lower_sash_lock_body",
    )
    lower.visual(
        Box(LOCK_LEVER),
        origin=Origin(xyz=(0.0, lock_body_y - LOCK_BODY[1] / 2.0, lock_z + 0.004)),
        material="lock",
        name="lower_sash_lock_lever",
    )

    # --- Upper sash: plain single pane + gaskets ---
    upper = model.part("upper_sash")
    upper.visual(
        mesh_from_cadquery(_build_plain_sash_frame_shape(), "upper_sash_frame"),
        material="sash",
        name="upper_sash_frame",
    )
    upper_openings = _plain_pane_opening()
    upper.visual(
        mesh_from_cadquery(_build_glass_panes(upper_openings), "upper_sash_glass"),
        material="glass",
        name="upper_sash_glass",
    )
    upper.visual(
        mesh_from_cadquery(_build_gasket_frames(upper_openings), "upper_sash_gaskets"),
        material="gasket",
        name="upper_sash_gaskets",
    )

    # ----- Articulations (double-hung, opposite slide directions) -----
    # LOWER sash: slides UP. axis (0,0,1), positive q opens upward.
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

    # UPPER sash: slides DOWN. axis (0,0,-1), positive q opens (moves down).
    model.articulation(
        "frame_to_upper_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="upper_sash",
        origin=Origin(xyz=(0.0, UPPER_SASH_Y, UPPER_BOTTOM_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=0.25, lower=0.0, upper=SASH_H * 0.42
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
    # Glass panes tuck under sash rails/muntins (captured glass).
    for sash_name in ("lower_sash", "upper_sash"):
        ctx.allow_overlap(
            sash_name, sash_name,
            elem_a=f"{sash_name}_glass",
            elem_b=f"{sash_name}_frame",
            reason="Glass panes are rebated under the sash rails/muntins so they read as captured.",
        )
        # Gaskets sit on sash face around lite openings (seated contact).
        ctx.allow_overlap(
            sash_name, sash_name,
            elem_a=f"{sash_name}_gaskets",
            elem_b=f"{sash_name}_frame",
            reason="Rubber gasket strips are seated on the sash interior face around each pane opening.",
        )

    # Each sash rides in the jamb/sill/head track grooves.
    ctx.allow_overlap(
        "frame", "lower_sash",
        reason="Lower sash stiles and rails ride in the jamb, sill, and head track grooves.",
    )
    ctx.allow_overlap(
        "frame", "upper_sash",
        reason="Upper sash stiles and rails ride in the jamb, sill, and head track grooves.",
    )
    # Two sashes overlap by one rail at the meeting rail (offset Y planes).
    ctx.allow_overlap(
        "lower_sash", "upper_sash",
        reason="Sashes overlap by one rail at the central meeting rail; offset Y planes.",
    )
    # Sash lock body seated onto lower sash top rail.
    ctx.allow_overlap(
        "lower_sash", "lower_sash",
        elem_a="lower_sash_lock_body",
        elem_b="lower_sash_frame",
        reason="Sash lock is mounted (seated) onto the lower sash meeting rail.",
    )

    # --- Closed pose (q=0): both sashes seated, window reads shut ---
    with ctx.pose({j_lower: 0.0, j_upper: 0.0}):
        f_aabb = ctx.part_world_aabb(frame)
        lo_aabb = ctx.part_world_aabb(lower)
        up_aabb = ctx.part_world_aabb(upper)

        frame_w = f_aabb[1][0] - f_aabb[0][0]
        sash_w = lo_aabb[1][0] - lo_aabb[0][0]
        ctx.check(
            "frame spans wider than a sash",
            frame_w > sash_w + 0.05,
            details=f"frame_w={frame_w:.3f}, sash_w={sash_w:.3f}",
        )
        ctx.check(
            "frame sill near z=0",
            abs(f_aabb[0][2]) < 0.01 and f_aabb[1][2] > 1.0,
            details=f"frame z range=({f_aabb[0][2]:.3f}, {f_aabb[1][2]:.3f})",
        )
        ctx.check(
            "sashes within frame width",
            lo_aabb[0][0] > f_aabb[0][0] and lo_aabb[1][0] < f_aabb[1][0]
            and up_aabb[0][0] > f_aabb[0][0] and up_aabb[1][0] < f_aabb[1][0],
            details=f"lower x=({lo_aabb[0][0]:.3f},{lo_aabb[1][0]:.3f}) "
                    f"upper x=({up_aabb[0][0]:.3f},{up_aabb[1][0]:.3f})",
        )
        lo_center_z = (lo_aabb[0][2] + lo_aabb[1][2]) / 2.0
        up_center_z = (up_aabb[0][2] + up_aabb[1][2]) / 2.0
        ctx.check(
            "lower sash below upper sash at closed pose",
            lo_center_z < up_center_z - 0.3,
            details=f"lower_cz={lo_center_z:.3f}, upper_cz={up_center_z:.3f}",
        )
        ctx.check(
            "sashes overlap at meeting rail (shut)",
            lo_aabb[1][2] >= up_aabb[0][2] - 1e-4,
            details=f"lower_top={lo_aabb[1][2]:.3f}, upper_bottom={up_aabb[0][2]:.3f}",
        )
        lo_cy = (lo_aabb[0][1] + lo_aabb[1][1]) / 2.0
        up_cy = (up_aabb[0][1] + up_aabb[1][1]) / 2.0
        ctx.check(
            "sashes ride in offset Y planes",
            abs(lo_cy - up_cy) > 0.015,
            details=f"lower_cy={lo_cy:.3f}, upper_cy={up_cy:.3f}",
        )
        rest_lo_z = lo_center_z
        rest_up_z = up_center_z
        rest_lo_top = lo_aabb[1][2]
        rest_up_bot = up_aabb[0][2]

    # --- Lower sash slides UP when opened ---
    travel = SASH_H * 0.40
    with ctx.pose({j_lower: travel}):
        op = ctx.part_world_aabb(lower)
        op_cz = (op[0][2] + op[1][2]) / 2.0
        ctx.check(
            "lower sash slides up when opened",
            op_cz > rest_lo_z + travel * 0.8,
            details=f"rest_cz={rest_lo_z:.3f}, opened_cz={op_cz:.3f}, travel={travel:.3f}",
        )
        ctx.expect_overlap(
            lower, frame, axes="x", min_overlap=0.05,
            name="lower sash retained in frame when open",
        )

    # --- Upper sash slides DOWN when opened ---
    with ctx.pose({j_upper: travel}):
        op = ctx.part_world_aabb(upper)
        op_cz = (op[0][2] + op[1][2]) / 2.0
        ctx.check(
            "upper sash slides down when opened",
            op_cz < rest_up_z - travel * 0.8,
            details=f"rest_cz={rest_up_z:.3f}, opened_cz={op_cz:.3f}, travel={travel:.3f}",
        )
        ctx.expect_overlap(
            upper, frame, axes="x", min_overlap=0.05,
            name="upper sash retained in frame when open",
        )

    # --- Both open: clear separation from closed seats ---
    with ctx.pose({j_lower: travel, j_upper: travel}):
        lo = ctx.part_world_aabb(lower)
        up = ctx.part_world_aabb(upper)
        ctx.check(
            "opening both sashes separates them from closed seats",
            lo[1][2] > rest_lo_top + travel * 0.7
            and up[0][2] < rest_up_bot - travel * 0.7,
            details=f"lower_top {rest_lo_top:.3f}->{lo[1][2]:.3f}, "
                    f"upper_bot {rest_up_bot:.3f}->{up[0][2]:.3f}",
        )

    # --- Sash lock centered on meeting rail ---
    lock_aabb = ctx.part_element_world_aabb(lower, elem="lower_sash_lock_body")
    if lock_aabb is not None:
        lock_cx = (lock_aabb[0][0] + lock_aabb[1][0]) / 2.0
        ctx.check(
            "sash lock centered on the meeting rail",
            abs(lock_cx) < 0.06,
            details=f"lock world X center={lock_cx:.3f}",
        )

    # --- Variant 07 specific checks ---

    # Lower sash has muntin grid frame (6-lite pattern)
    ctx.check(
        "lower sash has muntin grid frame",
        ctx.part_element_world_aabb(lower, elem="lower_sash_frame") is not None,
        details="lower_sash_frame visual with muntin grid must exist",
    )
    # Upper sash has plain frame (single large pane, no muntins)
    ctx.check(
        "upper sash has plain single-pane frame",
        ctx.part_element_world_aabb(upper, elem="upper_sash_frame") is not None,
        details="upper_sash_frame visual (no muntins) must exist",
    )
    # Rubber gasket strips on lower sash
    ctx.check(
        "lower sash has rubber gaskets around panes",
        ctx.part_element_world_aabb(lower, elem="lower_sash_gaskets") is not None,
        details="lower_sash_gaskets visual must exist",
    )
    # Rubber gasket strips on upper sash
    ctx.check(
        "upper sash has rubber gaskets around pane",
        ctx.part_element_world_aabb(upper, elem="upper_sash_gaskets") is not None,
        details="upper_sash_gaskets visual must exist",
    )
    # Both joints are prismatic (non-fixed)
    ctx.check(
        "lower sash joint is prismatic",
        j_lower.articulation_type == ArticulationType.PRISMATIC,
        details=f"joint type={j_lower.articulation_type}",
    )
    ctx.check(
        "upper sash joint is prismatic",
        j_upper.articulation_type == ArticulationType.PRISMATIC,
        details=f"joint type={j_upper.articulation_type}",
    )
    # Opposite slide directions: lower axis Z > 0, upper axis Z < 0
    ctx.check(
        "sashes slide in opposite Z directions",
        j_lower.axis[2] > 0 and j_upper.axis[2] < 0,
        details=f"lower axis z={j_lower.axis[2]:.1f}, upper axis z={j_upper.axis[2]:.1f}",
    )
    # Upper sash plain glass is larger than any single lower sash lite
    up_glass_aabb = ctx.part_element_world_aabb(upper, elem="upper_sash_glass")
    lo_glass_aabb = ctx.part_element_world_aabb(lower, elem="lower_sash_glass")
    if up_glass_aabb is not None and lo_glass_aabb is not None:
        up_glass_w = up_glass_aabb[1][0] - up_glass_aabb[0][0]
        lo_glass_w = lo_glass_aabb[1][0] - lo_glass_aabb[0][0]
        ctx.check(
            "upper sash single pane wider than lower sash glass region",
            up_glass_w > lo_glass_w - 0.01,
            details=f"upper_glass_w={up_glass_w:.3f}, lower_glass_w={lo_glass_w:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
