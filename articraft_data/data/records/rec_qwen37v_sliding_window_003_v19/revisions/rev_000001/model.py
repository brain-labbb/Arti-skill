from __future__ import annotations

# Sliding window variant: white frame, two stacked six-lite sashes that slide
# vertically, plus an outer insect screen panel in a separate exterior track.
#
# Variant 19 structural changes from the parent double-hung sash window:
#   1. Outer insect screen panel in a separate exterior (+Y) track.
#   2. Lower sash slides upward on a vertical prismatic joint (retained).
#   3. Deep track grooves along the top (head) and bottom (sill) frame rails.
#   4. Rubber gasket strips around every glass pane.
#
# Coordinate convention:
#   +Z is up. Window stands vertically: height along +Z, width along X, frame
#   depth / glazing thickness along Y. The sill sits at z=0; head at z=WIN_H.
#
# Articulation:
#   - LOWER sash: PRISMATIC, axis (0,0,1): positive q slides UP (opens).
#   - UPPER sash: PRISMATIC, axis (0,0,-1): positive q slides DOWN (opens).
#   - SCREEN:     PRISMATIC, axis (0,0,1): positive q slides UP (retracts).
#   All three ride in separate Y-plane tracks within the frame depth.

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
FRAME_DEPTH = 0.140   # outer frame jamb depth (Y) — wider to fit 3 tracks

# Clear opening inside the outer frame
OPEN_W = WIN_W - 2 * FRAME_FACE   # clear width
OPEN_H = WIN_H - 2 * FRAME_FACE   # clear height
OPEN_X0 = -WIN_W / 2.0 + FRAME_FACE
OPEN_X1 = WIN_W / 2.0 - FRAME_FACE
OPEN_Z0 = FRAME_FACE
OPEN_Z1 = WIN_H - FRAME_FACE

# Sash geometry
SASH_W = OPEN_W - 0.010                 # slight running clearance to the jambs
SASH_RAIL = 0.052                       # sash perimeter member width (stile/rail)
SASH_DEPTH = 0.034                      # sash thickness (Y)
SASH_H = OPEN_H * 0.545                 # each sash height (overlap at center)
GLASS_T = 0.006                         # glazing thickness (Y)

# Y planes: three tracks from interior (-Y) to exterior (+Y).
# Lower sash: interior track. Upper sash: middle track. Screen: exterior track.
SASH_Y_GAP = 0.018
LOWER_SASH_Y = -SASH_Y_GAP - 0.012      # interior track
UPPER_SASH_Y = 0.0                       # middle track
SCREEN_Y = SASH_Y_GAP + 0.018           # exterior track

# Closed-pose sash bottom edges (world Z)
LOWER_BOTTOM_Z = OPEN_Z0 + 0.004
MEETING_OVERLAP = SASH_RAIL
UPPER_BOTTOM_Z = LOWER_BOTTOM_Z + SASH_H - MEETING_OVERLAP

# Muntin grid: 3 columns x 2 rows of lites per sash
MUNTIN_W = 0.022
N_COLS = 3
N_ROWS = 2

# Side track channels (jamb grooves for sash stiles)
TRACK_W = 0.018
TRACK_DEPTH = 0.030

# Deep head/sill track groove dimensions
HEAD_SILL_GROOVE_DEPTH = 0.025   # how far the groove cuts into the frame member
HEAD_SILL_GROOVE_HEIGHT = 0.036  # groove opening width in Z (matches sash depth + clearance)

# Screen panel dimensions
SCREEN_FRAME_W = 0.024           # screen frame member width
SCREEN_DEPTH = 0.014             # screen frame thickness (Y)
SCREEN_W = OPEN_W - 0.008        # screen width with running clearance
# Screen extends BEYOND the head/sill track grooves so the top and bottom
# rails overlap solid frame material (real screen retention in tracks).
# Sill groove: OPEN_Z0 - HEAD_SILL_GROOVE_DEPTH to OPEN_Z0 (0.035 to 0.060)
# Head groove: OPEN_Z1 to OPEN_Z1 + HEAD_SILL_GROOVE_DEPTH (1.46 to 1.485)
SCREEN_BOTTOM_Z = OPEN_Z0 - HEAD_SILL_GROOVE_DEPTH - 0.005  # below sill groove
SCREEN_H = (OPEN_Z1 + HEAD_SILL_GROOVE_DEPTH + 0.005) - SCREEN_BOTTOM_Z

# Rubber gasket dimensions
GASKET_W = 0.004                 # gasket strip face width
GASKET_T = 0.003                 # gasket strip thickness (Y)

# Sash lock at the meeting rail
LOCK_BODY = (0.060, 0.026, 0.022)
LOCK_LEVER = (0.044, 0.012, 0.010)

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

FRAME_RGBA = (0.945, 0.945, 0.945, 1.0)    # white painted frame
SASH_RGBA = (0.965, 0.965, 0.965, 1.0)     # white sash
GLASS_RGBA = (0.30, 0.36, 0.42, 0.34)      # cool dark-tinted glass
LOCK_RGBA = (0.86, 0.87, 0.89, 1.0)        # brushed metal sash lock
GASKET_RGBA = (0.12, 0.12, 0.13, 1.0)      # dark charcoal rubber gasket
SCREEN_FRAME_RGBA = (0.72, 0.72, 0.73, 1.0) # silver aluminum screen frame
SCREEN_MESH_RGBA = (0.25, 0.26, 0.27, 0.45) # semi-transparent dark mesh


# ---------------------------------------------------------------------------
# Static outer frame geometry (CadQuery)
# ---------------------------------------------------------------------------

def _build_frame_shape() -> cq.Workplane:
    """White outer frame: perimeter slab with central opening, side-track
    channels in the jambs, deep track grooves in the head and sill, and a
    third (exterior) track groove for the insect screen.
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

    # --- Side jamb track grooves (two per jamb: lower sash + upper sash + screen) ---
    groove_x = FRAME_FACE * 0.55
    for sign, edge_x in ((+1.0, OPEN_X0), (-1.0, OPEN_X1)):
        cx = edge_x - sign * groove_x / 2.0
        for track_y in (LOWER_SASH_Y, UPPER_SASH_Y, SCREEN_Y):
            groove = (
                cq.Workplane("XY")
                .transformed(offset=(cx, track_y, (OPEN_Z0 + OPEN_Z1) / 2.0))
                .box(groove_x, TRACK_DEPTH, OPEN_H)
            )
            frame = frame.cut(groove)

    # --- Deep track grooves along head (top) and sill (bottom) rails ---
    # These are horizontal channels cut into the head and sill frame members
    # where the sash top/bottom rails ride. One groove per track plane.
    groove_z_depth = HEAD_SILL_GROOVE_DEPTH  # how far into the member (Z)
    groove_y_width = HEAD_SILL_GROOVE_HEIGHT  # opening in Y

    for track_y in (LOWER_SASH_Y, UPPER_SASH_Y, SCREEN_Y):
        # Sill groove (bottom frame member): cut upward from the opening bottom edge
        sill_cz = OPEN_Z0 - groove_z_depth / 2.0
        sill_groove = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, track_y, sill_cz))
            .box(OPEN_W, groove_y_width, groove_z_depth)
        )
        frame = frame.cut(sill_groove)

        # Head groove (top frame member): cut downward from the opening top edge
        head_cz = OPEN_Z1 + groove_z_depth / 2.0
        head_groove = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, track_y, head_cz))
            .box(OPEN_W, groove_y_width, groove_z_depth)
        )
        frame = frame.cut(head_groove)

    return frame


# ---------------------------------------------------------------------------
# Sash frame geometry (CadQuery): perimeter ring + 6-lite muntin grid
# ---------------------------------------------------------------------------

def _build_sash_frame_shape() -> cq.Workplane:
    """One sash: perimeter ring plus a 3x2 muntin grid."""
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
# Rubber gasket strips around glass panes
# ---------------------------------------------------------------------------

def _build_sash_gasket_shape() -> cq.Workplane:
    """Rubber gasket strips forming a thin border around each of the six lite
    openings. Each gasket is a flat rectangular ring (four strips per lite)
    that sits on the glass surface, slightly proud.

    Authored in the sash-local frame (same as sash frame and glass).
    """
    w = SASH_W
    h = SASH_H
    r = SASH_RAIL
    rebate = 0.003   # gasket tucks slightly under the muntin/rail lip

    in_x0, in_x1 = -w / 2.0 + r, w / 2.0 - r
    in_z0, in_z1 = r, h - r
    inner_w = in_x1 - in_x0
    inner_h = in_z1 - in_z0
    col_lines = [in_x0 + (i + 1) * inner_w / N_COLS for i in range(N_COLS - 1)]
    row_lines = [in_z0 + (j + 1) * inner_h / N_ROWS for j in range(N_ROWS - 1)]
    x_edges = [in_x0] + col_lines + [in_x1]
    z_edges = [in_z0] + row_lines + [in_z1]
    half_m = MUNTIN_W / 2.0

    gaskets = None
    gw = GASKET_W
    gt = GASKET_T

    for ci in range(N_COLS):
        for ri in range(N_ROWS):
            # Lite opening bounds (with small rebate under the muntins)
            lx0 = x_edges[ci] + (half_m if ci > 0 else 0.0) - rebate
            lx1 = x_edges[ci + 1] - (half_m if ci < N_COLS - 1 else 0.0) + rebate
            lz0 = z_edges[ri] + (half_m if ri > 0 else 0.0) - rebate
            lz1 = z_edges[ri + 1] - (half_m if ri < N_ROWS - 1 else 0.0) + rebate
            lite_w = lx1 - lx0
            lite_h = lz1 - lz0
            cx = (lx0 + lx1) / 2.0
            cz = (lz0 + lz1) / 2.0

            # Four strips per lite: top, bottom, left, right
            strips = [
                # Bottom strip (full width)
                (cx, 0.0, lz0 + gw / 2.0, lite_w, gt, gw),
                # Top strip (full width)
                (cx, 0.0, lz1 - gw / 2.0, lite_w, gt, gw),
                # Left strip (height minus corners)
                (lx0 + gw / 2.0, 0.0, cz, gw, gt, lite_h - 2 * gw),
                # Right strip (height minus corners)
                (lx1 - gw / 2.0, 0.0, cz, gw, gt, lite_h - 2 * gw),
            ]
            for sx, sy, sz, dx, dy, dz in strips:
                if dx <= 0 or dz <= 0:
                    continue
                strip = (
                    cq.Workplane("XY")
                    .transformed(offset=(sx, sy, sz))
                    .box(dx, dy, dz)
                )
                gaskets = strip if gaskets is None else gaskets.union(strip)

    return gaskets


# ---------------------------------------------------------------------------
# Insect screen panel geometry
# ---------------------------------------------------------------------------

def _build_screen_frame_shape() -> cq.Workplane:
    """Screen panel frame: a thin rectangular aluminum frame (four rails) with
    the central region open for the mesh. Authored in screen-local frame:
      - local X: -SCREEN_W/2 .. +SCREEN_W/2
      - local Z: 0 .. SCREEN_H
      - local Y: centered at 0, thickness SCREEN_DEPTH
    """
    w = SCREEN_W
    h = SCREEN_H
    r = SCREEN_FRAME_W
    d = SCREEN_DEPTH

    # Outer slab
    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, h / 2.0))
        .box(w, d, h)
    )

    # Cut the central opening (leaving the frame rails/stiles)
    inner_w = w - 2 * r
    inner_h = h - 2 * r
    opening = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, h / 2.0))
        .box(inner_w, d + 0.02, inner_h)
    )
    return outer.cut(opening)


def _build_screen_mesh_shape() -> cq.Workplane:
    """Thin semi-transparent mesh panel filling the screen frame opening."""
    w = SCREEN_W
    h = SCREEN_H
    r = SCREEN_FRAME_W
    mesh_t = 0.002  # very thin mesh panel

    inner_w = w - 2 * r + 0.006  # slight overlap with frame for capture
    inner_h = h - 2 * r + 0.006

    return (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, h / 2.0))
        .box(inner_w, mesh_t, inner_h)
    )


# ---------------------------------------------------------------------------
# Sash part builder
# ---------------------------------------------------------------------------

def _add_sash(model: ArticulatedObject, name: str) -> None:
    sash = model.part(name)
    sash.visual(
        mesh_from_cadquery(_build_sash_frame_shape(), f"{name}_frame"),
        material="sash",
        name=f"{name}_frame",
    )
    sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(), f"{name}_glass"),
        material="glass",
        name=f"{name}_glass",
    )
    # Rubber gasket strips around every glass pane
    sash.visual(
        mesh_from_cadquery(_build_sash_gasket_shape(), f"{name}_gaskets"),
        material="gasket",
        name=f"{name}_gaskets",
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_window_with_screen")

    model.material("frame", rgba=FRAME_RGBA)
    model.material("sash", rgba=SASH_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("lock", rgba=LOCK_RGBA)
    model.material("gasket", rgba=GASKET_RGBA)
    model.material("screen_frame", rgba=SCREEN_FRAME_RGBA)
    model.material("screen_mesh", rgba=SCREEN_MESH_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material="frame",
        name="frame_shell",
    )

    # --- Two sashes with glass and gaskets ---
    _add_sash(model, "lower_sash")
    _add_sash(model, "upper_sash")

    # Sash lock on the lower sash top (meeting) rail
    lower = model.get_part("lower_sash")
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

    # --- Insect screen panel in exterior track ---
    screen = model.part("screen")
    screen.visual(
        mesh_from_cadquery(_build_screen_frame_shape(), "screen_frame"),
        material="screen_frame",
        name="screen_frame",
    )
    screen.visual(
        mesh_from_cadquery(_build_screen_mesh_shape(), "screen_mesh"),
        material="screen_mesh",
        name="screen_mesh",
    )

    # ----- Articulations -----

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

    # SCREEN: slides UP in exterior track. axis (0,0,1), positive q retracts up.
    model.articulation(
        "frame_to_screen",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="screen",
        origin=Origin(xyz=(0.0, SCREEN_Y, SCREEN_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0, velocity=0.30, lower=0.0, upper=SCREEN_H * 0.40
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
    screen = object_model.get_part("screen")
    j_lower = object_model.get_articulation("frame_to_lower_sash")
    j_upper = object_model.get_articulation("frame_to_upper_sash")
    j_screen = object_model.get_articulation("frame_to_screen")

    # --- Intentional overlaps ---
    # Glass panes tuck under the sash muntin/rail lips (captured glass).
    for sash_name in ("lower_sash", "upper_sash"):
        ctx.allow_overlap(
            sash_name, sash_name,
            elem_a=f"{sash_name}_glass",
            elem_b=f"{sash_name}_frame",
            reason="Glass panes are rebated under the sash rails/muntins so they read as captured, not floating.",
        )
        # Gasket strips sit on the glass surface, slightly overlapping both glass and frame
        ctx.allow_overlap(
            sash_name, sash_name,
            elem_a=f"{sash_name}_gaskets",
            elem_b=f"{sash_name}_glass",
            reason="Rubber gasket strips are seated against the glass panes (compression fit).",
        )
        ctx.allow_overlap(
            sash_name, sash_name,
            elem_a=f"{sash_name}_gaskets",
            elem_b=f"{sash_name}_frame",
            reason="Rubber gasket strips tuck under the sash rail/muntin lips (captured seal).",
        )

    # Each sash rides in the jamb side-track grooves cut into the frame.
    ctx.allow_overlap(
        "frame", "lower_sash",
        reason="Lower sash stiles ride in the interior jamb track grooves (retained insertion).",
    )
    ctx.allow_overlap(
        "frame", "upper_sash",
        reason="Upper sash stiles ride in the middle jamb track grooves (retained insertion).",
    )
    # The two sashes overlap by one rail at the meeting rail (different Y planes).
    ctx.allow_overlap(
        "lower_sash", "upper_sash",
        reason="Sashes overlap by one rail at the central meeting rail; they ride in offset Y planes.",
    )
    # Sash lock body is seated onto the lower sash top rail.
    ctx.allow_overlap(
        "lower_sash", "lower_sash",
        elem_a="lower_sash_lock_body",
        elem_b="lower_sash_frame",
        reason="Sash lock is mounted (seated) onto the lower sash meeting rail.",
    )
    # Screen panel rides in the exterior track groove of the frame.
    ctx.allow_overlap(
        "frame", "screen",
        reason="Screen panel rides in the exterior track grooves (retained insertion).",
    )
    # Screen mesh overlaps the screen frame (captured in the frame)
    ctx.allow_overlap(
        "screen", "screen",
        elem_a="screen_mesh",
        elem_b="screen_frame",
        reason="Screen mesh is captured inside the screen frame rails.",
    )

    # --- Verify prompt-specific geometry ---

    # 1. Screen part exists with frame and mesh visuals
    screen_vis = [v.name for v in screen.visuals]
    ctx.check(
        "screen has frame visual",
        "screen_frame" in screen_vis,
        details=f"screen visuals: {screen_vis}",
    )
    ctx.check(
        "screen has mesh visual",
        "screen_mesh" in screen_vis,
        details=f"screen visuals: {screen_vis}",
    )

    # 2. Gasket visuals exist on both sashes
    for sash_name in ("lower_sash", "upper_sash"):
        sash_part = object_model.get_part(sash_name)
        sash_vis = [v.name for v in sash_part.visuals]
        ctx.check(
            f"{sash_name} has rubber gasket strips",
            f"{sash_name}_gaskets" in sash_vis,
            details=f"{sash_name} visuals: {sash_vis}",
        )

    # 3. Three distinct track Y planes (lower sash, upper sash, screen)
    # Screen is on the exterior side, distinct from both sash planes.
    ctx.check(
        "screen track is exterior to upper sash track",
        SCREEN_Y > UPPER_SASH_Y + 0.010,
        details=f"screen_y={SCREEN_Y:.3f}, upper_sash_y={UPPER_SASH_Y:.3f}",
    )

    # --- Closed pose (q=0): all panels seated, window reads shut ---
    with ctx.pose({j_lower: 0.0, j_upper: 0.0, j_screen: 0.0}):
        f_aabb = ctx.part_world_aabb(frame)
        lo_aabb = ctx.part_world_aabb(lower)
        up_aabb = ctx.part_world_aabb(upper)
        sc_aabb = ctx.part_world_aabb(screen)

        # Frame is the widest/tallest element
        frame_w = f_aabb[1][0] - f_aabb[0][0]
        sash_w = lo_aabb[1][0] - lo_aabb[0][0]
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
        # Sashes overlap at meeting rail
        ctx.check(
            "sashes overlap at meeting rail (shut)",
            lo_aabb[1][2] >= up_aabb[0][2] - 1e-4,
            details=f"lower_top={lo_aabb[1][2]:.3f}, upper_bottom={up_aabb[0][2]:.3f}",
        )
        # Screen is in the exterior Y plane
        sc_cy = (sc_aabb[0][1] + sc_aabb[1][1]) / 2.0
        lo_cy = (lo_aabb[0][1] + lo_aabb[1][1]) / 2.0
        up_cy = (up_aabb[0][1] + up_aabb[1][1]) / 2.0
        ctx.check(
            "screen rides in exterior track (distinct Y plane)",
            sc_cy > up_cy + 0.010,
            details=f"screen_cy={sc_cy:.3f}, upper_cy={up_cy:.3f}, lower_cy={lo_cy:.3f}",
        )
        # Screen covers most of the opening height
        sc_h = sc_aabb[1][2] - sc_aabb[0][2]
        ctx.check(
            "screen panel covers most of the opening",
            sc_h > OPEN_H * 0.85,
            details=f"screen_h={sc_h:.3f}, open_h={OPEN_H:.3f}",
        )

        rest_lo_z = lo_center_z
        rest_up_z = up_center_z
        rest_lo_top = lo_aabb[1][2]
        rest_up_bot = up_aabb[0][2]
        rest_sc_z = (sc_aabb[0][2] + sc_aabb[1][2]) / 2.0

    # --- HERO: lower sash slides UP (opens) ---
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

    # --- HERO: upper sash slides DOWN (opens) ---
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

    # --- HERO: screen slides UP (retracts) ---
    screen_travel = SCREEN_H * 0.35
    with ctx.pose({j_screen: screen_travel}):
        op = ctx.part_world_aabb(screen)
        op_cz = (op[0][2] + op[1][2]) / 2.0
        ctx.check(
            "screen panel slides up when retracted",
            op_cz > rest_sc_z + screen_travel * 0.8,
            details=f"rest_cz={rest_sc_z:.3f}, retracted_cz={op_cz:.3f}, travel={screen_travel:.3f}",
        )
        ctx.expect_overlap(
            screen, frame, axes="x", min_overlap=0.05,
            name="screen retained in frame when retracted",
        )

    # --- Both sashes open: clear central gap ---
    with ctx.pose({j_lower: travel, j_upper: travel}):
        lo = ctx.part_world_aabb(lower)
        up = ctx.part_world_aabb(upper)
        ctx.check(
            "opening both sashes separates them from closed seats",
            lo[1][2] > rest_lo_top + travel * 0.7
            and up[0][2] < rest_up_bot - travel * 0.7,
            details=f"lower_top {rest_lo_top:.3f}->{lo[1][2]:.3f}, upper_bot {rest_up_bot:.3f}->{up[0][2]:.3f}",
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

    # --- Articulation type checks: at least one non-fixed joint ---
    ctx.check(
        "lower sash joint is prismatic (vertical slide)",
        j_lower.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={j_lower.articulation_type}",
    )
    ctx.check(
        "screen joint is prismatic (vertical slide)",
        j_screen.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={j_screen.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
