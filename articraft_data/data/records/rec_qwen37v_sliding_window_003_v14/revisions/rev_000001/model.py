from __future__ import annotations

# Sliding window variant of the double-hung sash window: white frame, two
# stacked six-lite sashes that slide vertically, plus a narrow transom panel
# above, an independently-sliding insect screen, roller blocks on the lower
# sash, and a visible overlap stile where the sashes cross.
#
# Coordinate convention:
#   +Z is up. Window stands vertically: height along +Z, width along X, frame
#   depth / glazing thickness along Y (the glass plane is the X-Z plane). The
#   sill sits at z=0; the head is at z=WIN_H.
#
# Articulation:
#   - LOWER sash is PRISMATIC, axis (0,0,1): positive q slides it UP (opens).
#   - UPPER sash is PRISMATIC, axis (0,0,-1): positive q slides it DOWN (opens).
#   - INSECT SCREEN is PRISMATIC, axis (0,0,1): positive q slides it UP.
#   Both sashes stay retained in the side tracks at full travel. The lower sash
#   rides in the interior (-Y) track plane; the upper sash rides in the exterior
#   (+Y) track plane, so they pass each other at the meeting rail.

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

# Transom: fixed glass panel above the sliding sashes
TRANSOM_H = 0.200         # transom panel clear height
TRANSOM_BAR = 0.040       # horizontal transom bar thickness (Z)

# Clear opening inside the outer frame (below the transom)
OPEN_W = WIN_W - 2 * FRAME_FACE   # clear width
# Sash opening height: total minus frame top/bottom minus transom bar minus transom panel
SASH_OPEN_H = WIN_H - 2 * FRAME_FACE - TRANSOM_BAR - TRANSOM_H
OPEN_X0 = -WIN_W / 2.0 + FRAME_FACE
OPEN_X1 = WIN_W / 2.0 - FRAME_FACE
OPEN_Z0 = FRAME_FACE                    # sill top
OPEN_Z1_SASH = FRAME_FACE + SASH_OPEN_H # top of sash opening (bottom of transom bar)

# Transom bar center Z
TRANSOM_BAR_Z = OPEN_Z1_SASH + TRANSOM_BAR / 2.0
# Transom glass panel center and extent
TRANSOM_GLASS_Z0 = TRANSOM_BAR_Z + TRANSOM_BAR / 2.0
TRANSOM_GLASS_Z1 = WIN_H - FRAME_FACE

# Sash geometry. Each sash is the same width and a bit over half the sash opening
# height; they overlap at the meeting rail in the middle.
SASH_W = OPEN_W - 0.010                 # slight running clearance to the jambs
SASH_RAIL = 0.052                       # sash perimeter member width (stile/rail)
SASH_DEPTH = 0.034                      # sash thickness (Y)
SASH_H = SASH_OPEN_H * 0.545           # each sash height (overlap at center)
GLASS_T = 0.006                         # glazing thickness (Y)

# Y planes: lower sash rides interior (-Y), upper sash rides exterior (+Y),
# offset from the frame depth center so they clear each other at the meeting rail.
SASH_Y_GAP = 0.016                       # half the gap between the two sash planes
LOWER_SASH_Y = -SASH_Y_GAP
UPPER_SASH_Y = +SASH_Y_GAP

# Closed-pose sash bottom edges (world Z). They overlap by one rail height at
# the meeting rail.
LOWER_BOTTOM_Z = OPEN_Z0 + 0.004        # lower sash rests on the sill, small clearance
MEETING_OVERLAP = SASH_RAIL             # one rail of overlap at the meeting rail
UPPER_BOTTOM_Z = LOWER_BOTTOM_Z + SASH_H - MEETING_OVERLAP

# Muntin grid: 3 columns x 2 rows of lites per sash -> 2 vertical + 1 horizontal bar.
MUNTIN_W = 0.022        # muntin bar face width
N_COLS = 3
N_ROWS = 2

# Side track channels (so each sash visibly rides in a groove in the jambs).
TRACK_W = 0.018
TRACK_DEPTH = 0.030

# Sash lock at the meeting rail.
LOCK_BODY = (0.060, 0.026, 0.022)   # (X, Y, Z)
LOCK_LEVER = (0.044, 0.012, 0.010)

# Roller blocks at bottom of lower sash
ROLLER_W = 0.030       # roller block width (X)
ROLLER_D = 0.018       # roller block depth (Y)
ROLLER_H = 0.012       # roller block height (Z)
ROLLER_INSET = 0.060   # how far in from each stile edge

# Overlap stile: a visible vertical member at the meeting rail crossing
OVERLAP_STILE_W = 0.026    # width of the overlap stile (X)
OVERLAP_STILE_D = 0.008    # depth/thickness (Y)
OVERLAP_STILE_H = SASH_RAIL * 1.6  # extends above and below meeting rail

# Insect screen
SCREEN_FRAME_W = 0.018     # screen frame member width
SCREEN_DEPTH = 0.014       # screen thickness (Y, very thin)
SCREEN_W = OPEN_W - 0.012  # slightly narrower than opening
SCREEN_H = SASH_OPEN_H * 0.50  # about half the sash opening height
SCREEN_Y = LOWER_SASH_Y - SASH_DEPTH / 2.0 - SCREEN_DEPTH / 2.0 - 0.003  # interior side

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

FRAME_RGBA = (0.945, 0.945, 0.945, 1.0)   # white painted frame
SASH_RGBA = (0.965, 0.965, 0.965, 1.0)    # white sash (very slightly brighter)
GLASS_RGBA = (0.30, 0.36, 0.42, 0.34)     # cool dark-tinted glass
LOCK_RGBA = (0.86, 0.87, 0.89, 1.0)       # brushed metal sash lock
ROLLER_RGBA = (0.25, 0.25, 0.28, 1.0)     # dark nylon/plastic rollers
SCREEN_FRAME_RGBA = (0.80, 0.80, 0.78, 1.0)  # aluminum screen frame
SCREEN_MESH_RGBA = (0.35, 0.38, 0.35, 0.55)  # dark insect screen mesh
STILE_RGBA = (0.93, 0.93, 0.93, 1.0)     # white overlap stile


# ---------------------------------------------------------------------------
# Static outer frame geometry (CadQuery)
# ---------------------------------------------------------------------------

def _build_frame_shape() -> cq.Workplane:
    """White outer frame: a perimeter slab with the central sash opening cut out,
    plus a transom bar and transom opening, plus side-track channels in the jambs.
    """
    # Solid outer slab spanning the full window footprint.
    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, WIN_H / 2.0))
        .box(WIN_W, FRAME_DEPTH, WIN_H)
    )

    # Cut the sash opening (lower portion, below transom bar).
    sash_opening = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, (OPEN_Z0 + OPEN_Z1_SASH) / 2.0))
        .box(OPEN_W, FRAME_DEPTH + 0.02, SASH_OPEN_H)
    )
    frame = outer.cut(sash_opening)

    # Cut the transom opening (upper portion, above transom bar).
    transom_opening = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, (TRANSOM_GLASS_Z0 + TRANSOM_GLASS_Z1) / 2.0))
        .box(OPEN_W, FRAME_DEPTH + 0.02, TRANSOM_GLASS_Z1 - TRANSOM_GLASS_Z0)
    )
    frame = frame.cut(transom_opening)

    # Two side-track channels per jamb: shallow grooves for the sash stiles.
    groove_x = FRAME_FACE * 0.55
    for sign, edge_x in ((+1.0, OPEN_X0), (-1.0, OPEN_X1)):
        cx = edge_x - sign * groove_x / 2.0
        for track_y in (LOWER_SASH_Y, UPPER_SASH_Y):
            groove = (
                cq.Workplane("XY")
                .transformed(offset=(cx, track_y, (OPEN_Z0 + OPEN_Z1_SASH) / 2.0))
                .box(groove_x, TRACK_DEPTH, SASH_OPEN_H)
            )
            frame = frame.cut(groove)

    return frame


def _build_transom_bar_shape() -> cq.Workplane:
    """Horizontal transom bar separating sash opening from transom panel."""
    return (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, TRANSOM_BAR_Z))
        .box(OPEN_W, FRAME_DEPTH * 0.75, TRANSOM_BAR)
    )


def _build_transom_glass_shape() -> cq.Workplane:
    """Fixed glass panel in the transom opening.
    
    The glass extends slightly into the frame rebate on all edges so it reads
    as captured/seated rather than floating.
    """
    rebate = 0.008  # glass extends into frame rebate
    glass_w = OPEN_W - 0.010 + 2 * rebate
    glass_h = TRANSOM_GLASS_Z1 - TRANSOM_GLASS_Z0 + 2 * rebate
    glass_cz = (TRANSOM_GLASS_Z0 + TRANSOM_GLASS_Z1) / 2.0
    return (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, glass_cz))
        .box(glass_w, GLASS_T, glass_h)
    )


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

    # Inner glazed region (inside the perimeter rails).
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
# Overlap stile geometry
# ---------------------------------------------------------------------------

def _build_overlap_stile_shape() -> cq.Workplane:
    """Visible vertical stile at the meeting rail where the two sashes cross.
    Authored in the lower sash local frame: positioned at the top rail area
    where the sashes overlap, slightly proud of the sash face on the interior
    side so it reads as a distinct crossing member.
    """
    # Position: centered at x=0, at the meeting rail overlap zone, on interior face
    stile_z = SASH_H - SASH_RAIL / 2.0  # at the top rail center
    stile_y = -(SASH_DEPTH / 2.0 + OVERLAP_STILE_D / 2.0 - 0.003)  # slightly proud
    return (
        cq.Workplane("XY")
        .transformed(offset=(0.0, stile_y, stile_z))
        .box(OVERLAP_STILE_W, OVERLAP_STILE_D, OVERLAP_STILE_H)
    )


# ---------------------------------------------------------------------------
# Insect screen geometry
# ---------------------------------------------------------------------------

def _build_screen_frame_shape() -> cq.Workplane:
    """Insect screen: thin rectangular frame with mesh infill.
    Authored in screen-local frame: centered at origin, height along +Z from 0.
    """
    w = SCREEN_W
    h = SCREEN_H
    d = SCREEN_DEPTH
    fw = SCREEN_FRAME_W

    # Outer frame slab
    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, h / 2.0))
        .box(w, d, h)
    )

    # Cut out the interior to leave a frame ring
    inner_w = w - 2 * fw
    inner_h = h - 2 * fw
    inner = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, h / 2.0))
        .box(inner_w, d + 0.02, inner_h)
    )
    return outer.cut(inner)


def _build_screen_mesh_shape() -> cq.Workplane:
    """Thin mesh panel filling the screen frame opening."""
    w = SCREEN_W - 2 * SCREEN_FRAME_W + 0.004
    h = SCREEN_H - 2 * SCREEN_FRAME_W + 0.004
    mesh_t = 0.002  # very thin mesh
    return (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, SCREEN_H / 2.0))
        .box(w, mesh_t, h)
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


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_window_with_transom")

    model.material("frame", rgba=FRAME_RGBA)
    model.material("sash", rgba=SASH_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("lock", rgba=LOCK_RGBA)
    model.material("roller", rgba=ROLLER_RGBA)
    model.material("screen_frame", rgba=SCREEN_FRAME_RGBA)
    model.material("screen_mesh", rgba=SCREEN_MESH_RGBA)
    model.material("stile", rgba=STILE_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material="frame",
        name="frame_shell",
    )
    # Transom bar
    frame.visual(
        mesh_from_cadquery(_build_transom_bar_shape(), "transom_bar"),
        material="frame",
        name="transom_bar",
    )
    # Transom glass (fixed panel)
    frame.visual(
        mesh_from_cadquery(_build_transom_glass_shape(), "transom_glass"),
        material="glass",
        name="transom_glass",
    )

    # --- Two sashes ---
    _add_sash(model, "lower_sash")
    _add_sash(model, "upper_sash")

    # Sash lock on the lower sash top (meeting) rail, center.
    lower = model.get_part("lower_sash")
    lock_z = SASH_H - SASH_RAIL / 2.0   # on the top rail of the lower sash
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

    # --- Overlap stile on lower sash at the meeting rail ---
    lower.visual(
        mesh_from_cadquery(_build_overlap_stile_shape(), "overlap_stile"),
        material="stile",
        name="overlap_stile",
    )

    # --- Roller blocks at bottom of lower sash ---
    roller_z = -ROLLER_H / 2.0  # below the sash bottom rail (sits on sill track)
    roller_y = 0.0  # centered in sash depth
    left_x = -SASH_W / 2.0 + ROLLER_INSET + ROLLER_W / 2.0
    right_x = SASH_W / 2.0 - ROLLER_INSET - ROLLER_W / 2.0
    lower.visual(
        Box((ROLLER_W, ROLLER_D, ROLLER_H)),
        origin=Origin(xyz=(left_x, roller_y, roller_z)),
        material="roller",
        name="roller_left",
    )
    lower.visual(
        Box((ROLLER_W, ROLLER_D, ROLLER_H)),
        origin=Origin(xyz=(right_x, roller_y, roller_z)),
        material="roller",
        name="roller_right",
    )

    # --- Insect screen (independently sliding) ---
    screen = model.part("insect_screen")
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

    # INSECT SCREEN: slides UP independently. axis (0,0,1), positive q slides up.
    # Screen rests at bottom of sash opening, can slide up to near the transom.
    screen_bottom_z = OPEN_Z0 + 0.006  # just above sill
    model.articulation(
        "frame_to_screen",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="insect_screen",
        origin=Origin(xyz=(0.0, SCREEN_Y, screen_bottom_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0, velocity=0.3, lower=0.0, upper=SASH_OPEN_H - SCREEN_H - 0.010
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
    screen = object_model.get_part("insect_screen")
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
    # Each sash rides in the jamb side-track grooves cut into the frame.
    ctx.allow_overlap(
        "frame", "lower_sash",
        reason="Lower sash stiles ride in the interior jamb track grooves (retained insertion).",
    )
    ctx.allow_overlap(
        "frame", "upper_sash",
        reason="Upper sash stiles ride in the exterior jamb track grooves (retained insertion).",
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
    # Overlap stile is mounted on the lower sash at the meeting rail.
    ctx.allow_overlap(
        "lower_sash", "lower_sash",
        elem_a="overlap_stile",
        elem_b="lower_sash_frame",
        reason="Overlap stile is mounted proud of the lower sash at the meeting rail crossing.",
    )
    # Roller blocks are seated on the lower sash bottom rail.
    ctx.allow_overlap(
        "lower_sash", "lower_sash",
        elem_a="roller_left",
        elem_b="lower_sash_frame",
        reason="Roller block is mounted on the lower sash bottom rail.",
    )
    ctx.allow_overlap(
        "lower_sash", "lower_sash",
        elem_a="roller_right",
        elem_b="lower_sash_frame",
        reason="Roller block is mounted on the lower sash bottom rail.",
    )
    # Screen mesh is captured inside screen frame.
    ctx.allow_overlap(
        "insect_screen", "insect_screen",
        elem_a="screen_mesh",
        elem_b="screen_frame",
        reason="Screen mesh is captured inside the screen frame.",
    )
    # Screen rides in the frame track area.
    ctx.allow_overlap(
        "frame", "insect_screen",
        reason="Insect screen rides in a track channel on the interior side of the frame.",
    )
    # Transom glass is seated in the frame transom opening.
    ctx.allow_overlap(
        "frame", "frame",
        elem_a="transom_glass",
        elem_b="frame_shell",
        reason="Transom glass panel is seated in the frame transom opening.",
    )
    # Transom bar connects to frame shell.
    ctx.allow_overlap(
        "frame", "frame",
        elem_a="transom_bar",
        elem_b="frame_shell",
        reason="Transom bar is structurally part of the frame assembly.",
    )

    # --- Transom panel exists above the sashes ---
    transom_bar_aabb = ctx.part_element_world_aabb(frame, elem="transom_bar")
    if transom_bar_aabb is not None:
        lower_aabb = ctx.part_world_aabb(lower)
        ctx.check(
            "transom bar sits above the lower sash",
            transom_bar_aabb[0][2] > lower_aabb[0][2] + SASH_H * 0.5,
            details=f"transom_bar_min_z={transom_bar_aabb[0][2]:.3f}, lower_min_z={lower_aabb[0][2]:.3f}",
        )

    transom_glass_aabb = ctx.part_element_world_aabb(frame, elem="transom_glass")
    if transom_glass_aabb is not None:
        upper_aabb_closed = ctx.part_world_aabb(upper)
        ctx.check(
            "transom glass sits above the upper sash",
            transom_glass_aabb[0][2] > upper_aabb_closed[0][2] + SASH_H * 0.3,
            details=f"transom_glass_min_z={transom_glass_aabb[0][2]:.3f}, upper_min_z={upper_aabb_closed[0][2]:.3f}",
        )

    # --- Closed pose (q=0): both sashes seated, window reads shut ---
    with ctx.pose({j_lower: 0.0, j_upper: 0.0, j_screen: 0.0}):
        f_aabb = ctx.part_world_aabb(frame)
        lo_aabb = ctx.part_world_aabb(lower)
        up_aabb = ctx.part_world_aabb(upper)

        # Frame is the widest/tallest element.
        frame_w = f_aabb[1][0] - f_aabb[0][0]
        sash_w = lo_aabb[1][0] - lo_aabb[0][0]
        ctx.check(
            "frame spans wider than a sash",
            frame_w > sash_w + 0.05,
            details=f"frame_w={frame_w:.3f}, sash_w={sash_w:.3f}",
        )
        # Sill at/near z=0 (window stands upright).
        ctx.check(
            "frame sill near z=0",
            abs(f_aabb[0][2]) < 0.01 and f_aabb[1][2] > 1.0,
            details=f"frame z range=({f_aabb[0][2]:.3f}, {f_aabb[1][2]:.3f})",
        )
        # Sashes are inside the frame opening width.
        ctx.check(
            "sashes within frame width",
            lo_aabb[0][0] > f_aabb[0][0] and lo_aabb[1][0] < f_aabb[1][0]
            and up_aabb[0][0] > f_aabb[0][0] and up_aabb[1][0] < f_aabb[1][0],
            details=f"lower x=({lo_aabb[0][0]:.3f},{lo_aabb[1][0]:.3f}) upper x=({up_aabb[0][0]:.3f},{up_aabb[1][0]:.3f})",
        )
        # Lower sash sits below the upper sash.
        lo_center_z = (lo_aabb[0][2] + lo_aabb[1][2]) / 2.0
        up_center_z = (up_aabb[0][2] + up_aabb[1][2]) / 2.0
        ctx.check(
            "lower sash below upper sash at closed pose",
            lo_center_z < up_center_z - 0.2,
            details=f"lower_cz={lo_center_z:.3f}, upper_cz={up_center_z:.3f}",
        )

        # Roller blocks are at the bottom of the lower sash
        roller_l_aabb = ctx.part_element_world_aabb(lower, elem="roller_left")
        roller_r_aabb = ctx.part_element_world_aabb(lower, elem="roller_right")
        if roller_l_aabb is not None and roller_r_aabb is not None:
            ctx.check(
                "roller blocks at bottom of lower sash",
                roller_l_aabb[0][2] < lo_aabb[0][2] + 0.010
                and roller_r_aabb[0][2] < lo_aabb[0][2] + 0.010,
                details=f"roller_l_min_z={roller_l_aabb[0][2]:.3f}, roller_r_min_z={roller_r_aabb[0][2]:.3f}, sash_bottom={lo_aabb[0][2]:.3f}",
            )
            # Two distinct rollers separated in X
            roller_l_cx = (roller_l_aabb[0][0] + roller_l_aabb[1][0]) / 2.0
            roller_r_cx = (roller_r_aabb[0][0] + roller_r_aabb[1][0]) / 2.0
            ctx.check(
                "two rollers separated horizontally",
                abs(roller_r_cx - roller_l_cx) > 0.3,
                details=f"left_x={roller_l_cx:.3f}, right_x={roller_r_cx:.3f}",
            )

        # Overlap stile at the meeting rail
        stile_aabb = ctx.part_element_world_aabb(lower, elem="overlap_stile")
        if stile_aabb is not None:
            # Stile should be near the top of the lower sash (meeting rail area)
            stile_cz = (stile_aabb[0][2] + stile_aabb[1][2]) / 2.0
            ctx.check(
                "overlap stile at meeting rail zone",
                stile_cz > lo_aabb[1][2] - SASH_RAIL * 2.0,
                details=f"stile_cz={stile_cz:.3f}, sash_top={lo_aabb[1][2]:.3f}",
            )

        # Screen at rest position (bottom of opening)
        scr_aabb = ctx.part_world_aabb(screen)
        ctx.check(
            "screen at rest is near bottom of sash opening",
            scr_aabb[0][2] < OPEN_Z0 + 0.05,
            details=f"screen_min_z={scr_aabb[0][2]:.3f}",
        )

        rest_lo_z = lo_center_z
        rest_up_z = up_center_z
        rest_scr_z = (scr_aabb[0][2] + scr_aabb[1][2]) / 2.0

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

    # --- HERO: insect screen slides UP independently ---
    screen_travel = SASH_OPEN_H * 0.40
    with ctx.pose({j_screen: screen_travel}):
        scr_op = ctx.part_world_aabb(screen)
        scr_op_cz = (scr_op[0][2] + scr_op[1][2]) / 2.0
        ctx.check(
            "insect screen slides up independently",
            scr_op_cz > rest_scr_z + screen_travel * 0.8,
            details=f"rest_scr_cz={rest_scr_z:.3f}, opened_scr_cz={scr_op_cz:.3f}, travel={screen_travel:.3f}",
        )
        # Screen stays retained in frame X footprint
        ctx.expect_overlap(
            screen, frame, axes="x", min_overlap=0.05,
            name="screen retained in frame when slid up",
        )

    # --- Screen joint is prismatic (non-fixed) ---
    ctx.check(
        "screen articulation is prismatic",
        j_screen.articulation_type == ArticulationType.PRISMATIC,
        details=f"screen joint type={j_screen.articulation_type}",
    )

    # --- Sash lock sits at the meeting rail, centered in X ---
    lock_aabb = ctx.part_element_world_aabb(lower, elem="lower_sash_lock_body")
    if lock_aabb is not None:
        lock_cx = (lock_aabb[0][0] + lock_aabb[1][0]) / 2.0
        ctx.check(
            "sash lock centered on the meeting rail",
            abs(lock_cx) < 0.06,
            details=f"lock world X center={lock_cx:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
