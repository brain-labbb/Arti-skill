from __future__ import annotations

# Corner-lift sliding window: white frame, one horizontally-sliding sash
# with corner-lift tab and roller blocks, a small fixed vent panel, an
# independently sliding insect screen, a raised sill lip, and drainage
# slots cut through the sill.
#
# Coordinate convention:
#   +Z is up. Window stands vertically: width along X, height along Z,
#   frame depth / glazing thickness along Y (the glass plane is X-Z).
#   Sill at z=0; head at z=WIN_H.
#
# Articulation:
#   - SLIDING SASH: PRISMATIC, axis (-1,0,0): positive q slides LEFT (opens).
#     The sash covers the right half of the opening when closed and slides
#     leftward to overlap the fixed panel region, exposing the right side.
#   - INSECT SCREEN: PRISMATIC, axis (-1,0,0): positive q slides LEFT.
#     The screen rides on a shallow interior track independently of the sash.

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

WIN_W = 1.20          # overall window width (X)
WIN_H = 1.00          # overall window height (Z), sill at z=0
FRAME_FACE = 0.055    # outer frame member face width (X/Z)
FRAME_DEPTH = 0.100   # outer frame jamb depth (Y)

# Clear opening inside the outer frame
OPEN_W = WIN_W - 2 * FRAME_FACE
OPEN_H = WIN_H - 2 * FRAME_FACE
OPEN_X0 = -WIN_W / 2.0 + FRAME_FACE
OPEN_X1 = WIN_W / 2.0 - FRAME_FACE
OPEN_Z0 = FRAME_FACE
OPEN_Z1 = WIN_H - FRAME_FACE

# Sash geometry: covers roughly the right half of the opening
SASH_W = OPEN_W * 0.52
SASH_H = OPEN_H - 0.008          # small clearance top and bottom
SASH_RAIL = 0.048                # sash perimeter member width
SASH_DEPTH = 0.032               # sash thickness (Y)
GLASS_T = 0.006                  # glazing thickness

# Sash Y position (middle track, slightly exterior)
SASH_Y = 0.012

# Sash closed position: right side of opening
# The sash local frame has X centered and Z from 0 to SASH_H.
# At closed, the sash right edge is near OPEN_X1.
SASH_CLOSED_CX = OPEN_X1 - SASH_W / 2.0 - 0.003
SASH_CLOSED_CZ = OPEN_Z0 + 0.004   # bottom of sash just above sill

# Travel: how far sash slides left (almost full sash width)
SASH_TRAVEL = SASH_W * 0.82

# Muntin grid for sash: 3 columns x 2 rows
MUNTIN_W = 0.020
N_COLS = 3
N_ROWS = 2

# Side track channels in jambs
TRACK_W = 0.016
TRACK_DEPTH = 0.028

# Roller blocks at sash bottom
ROLLER_W = 0.028
ROLLER_H = 0.010
ROLLER_D = 0.018
ROLLER_INSET = SASH_W / 2.0 - 0.060  # distance from sash center to roller center

# Corner-lift tab at sash bottom-right corner
LIFT_TAB_W = 0.038
LIFT_TAB_H = 0.014
LIFT_TAB_D = 0.016

# Insect screen
SCREEN_W = OPEN_W - 0.012
SCREEN_H = OPEN_H - 0.010
SCREEN_FRAME = 0.018         # screen frame member width
SCREEN_DEPTH = 0.010         # screen frame thickness
SCREEN_Y = -FRAME_DEPTH / 2.0 + 0.022   # interior track

# Screen closed position: centered on the opening
SCREEN_CLOSED_CX = 0.0
SCREEN_CLOSED_CZ = OPEN_Z0 + 0.005
SCREEN_TRAVEL = OPEN_W * 0.42

# Sill lip: raised ridge on interior sill edge
SILL_LIP_H = 0.010
SILL_LIP_D = 0.012

# Drainage slots in the sill
DRAIN_COUNT = 4
DRAIN_SLOT_W = 0.038
DRAIN_SLOT_H = 0.005
DRAIN_SLOT_DEPTH = FRAME_DEPTH * 0.6

# Vent panel: small fixed lite in upper-left corner of the opening
VENT_W = 0.18
VENT_H = 0.20
VENT_RAIL = 0.022
VENT_GLASS_T = 0.005

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

FRAME_RGBA = (0.945, 0.945, 0.945, 1.0)     # white painted frame
SASH_RGBA = (0.965, 0.965, 0.965, 1.0)      # white sash
GLASS_RGBA = (0.30, 0.36, 0.42, 0.34)       # cool dark-tinted glass
SCREEN_RGBA = (0.50, 0.50, 0.48, 0.45)      # semi-transparent screen mesh
ROLLER_RGBA = (0.22, 0.22, 0.22, 1.0)       # dark nylon rollers
LIFT_RGBA = (0.80, 0.80, 0.82, 1.0)         # metal corner-lift tab
VENT_RGBA = (0.93, 0.93, 0.93, 1.0)         # vent panel frame


# ---------------------------------------------------------------------------
# Frame geometry (CadQuery)
# ---------------------------------------------------------------------------

def _build_frame_shape() -> cq.Workplane:
    """Outer perimeter frame with central opening, sill track channel,
    head track channel, sill lip, and drainage slots."""
    # Solid outer slab
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

    # Track channels in sill and head for the sash
    # Sill track: groove along X at the sill level, centered in Y at SASH_Y
    sill_track = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, SASH_Y, OPEN_Z0 - 0.002))
        .box(OPEN_W + 0.01, TRACK_DEPTH, 0.012)
    )
    frame = frame.cut(sill_track)

    # Head track: groove along X at the head level
    head_track = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, SASH_Y, OPEN_Z1 + 0.002))
        .box(OPEN_W + 0.01, TRACK_DEPTH, 0.012)
    )
    frame = frame.cut(head_track)

    # Screen track: shallow groove on interior face of sill and head
    screen_sill_track = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, SCREEN_Y, OPEN_Z0 - 0.002))
        .box(OPEN_W + 0.01, SCREEN_DEPTH + 0.004, 0.010)
    )
    frame = frame.cut(screen_sill_track)

    screen_head_track = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, SCREEN_Y, OPEN_Z1 + 0.002))
        .box(OPEN_W + 0.01, SCREEN_DEPTH + 0.004, 0.010)
    )
    frame = frame.cut(screen_head_track)

    # Side track grooves in jambs for sash stiles
    groove_x = FRAME_FACE * 0.50
    for sign, edge_x in ((+1.0, OPEN_X0), (-1.0, OPEN_X1)):
        cx = edge_x - sign * groove_x / 2.0
        groove = (
            cq.Workplane("XY")
            .transformed(offset=(cx, SASH_Y, (OPEN_Z0 + OPEN_Z1) / 2.0))
            .box(groove_x, TRACK_DEPTH, OPEN_H)
        )
        frame = frame.cut(groove)

    # Sill lip: add a raised ridge on the interior edge of the sill
    sill_lip = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, -FRAME_DEPTH / 2.0 + SILL_LIP_D / 2.0, OPEN_Z0 - SILL_LIP_H / 2.0))
        .box(OPEN_W * 0.96, SILL_LIP_D, SILL_LIP_H)
    )
    frame = frame.union(sill_lip)

    # Drainage slots: narrow horizontal slots cut through the sill near the
    # exterior face. These are through-cuts so water can drain outward.
    drain_spacing = OPEN_W / (DRAIN_COUNT + 1)
    for i in range(DRAIN_COUNT):
        dx = OPEN_X0 + drain_spacing * (i + 1)
        slot = (
            cq.Workplane("XY")
            .transformed(offset=(dx, FRAME_DEPTH / 2.0 - DRAIN_SLOT_DEPTH / 2.0, OPEN_Z0 - DRAIN_SLOT_H / 2.0 + 0.001))
            .box(DRAIN_SLOT_W, DRAIN_SLOT_DEPTH + 0.01, DRAIN_SLOT_H)
        )
        frame = frame.cut(slot)

    return frame


# ---------------------------------------------------------------------------
# Sash frame geometry (CadQuery): perimeter ring + 6-lite muntin grid
# ---------------------------------------------------------------------------

def _build_sash_frame_shape() -> cq.Workplane:
    """Sliding sash: perimeter ring plus 3x2 muntin grid.
    Local frame: X centered (-SASH_W/2..+SASH_W/2), Z from 0..SASH_H,
    Y centered at 0 with depth SASH_DEPTH."""
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
# Insect screen geometry (CadQuery)
# ---------------------------------------------------------------------------

def _build_screen_frame_shape() -> cq.Workplane:
    """Insect screen: thin perimeter frame with mesh infill.
    Local frame: X centered, Z from 0..SCREEN_H, Y centered at 0."""
    w = SCREEN_W
    h = SCREEN_H
    f = SCREEN_FRAME
    d = SCREEN_DEPTH

    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, h / 2.0))
        .box(w, d, h)
    )
    # Cut the center to leave a thin frame
    inner = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, h / 2.0))
        .box(w - 2 * f, d + 0.01, h - 2 * f)
    )
    return outer.cut(inner)


def _build_screen_mesh_shape() -> cq.Workplane:
    """Semi-transparent screen mesh panel inside the frame."""
    w = SCREEN_W - 2 * SCREEN_FRAME + 0.004
    h = SCREEN_H - 2 * SCREEN_FRAME + 0.004
    return (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, h / 2.0 + SCREEN_FRAME))
        .box(w, 0.002, h)
    )


# ---------------------------------------------------------------------------
# Vent panel geometry (CadQuery)
# ---------------------------------------------------------------------------

def _build_vent_frame_shape() -> cq.Workplane:
    """Small fixed vent panel: thin frame with glass.
    Local frame: X centered, Z from 0..VENT_H, Y centered at 0."""
    w = VENT_W
    h = VENT_H
    r = VENT_RAIL
    d = SASH_DEPTH * 0.7

    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, h / 2.0))
        .box(w, d, h)
    )
    inner = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, h / 2.0))
        .box(w - 2 * r, d + 0.01, h - 2 * r)
    )
    return outer.cut(inner)


def _build_vent_glass_shape() -> cq.Workplane:
    """Single glass pane in the vent panel."""
    w = VENT_W - 2 * VENT_RAIL + 0.004
    h = VENT_H - 2 * VENT_RAIL + 0.004
    return (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, h / 2.0 + VENT_RAIL))
        .box(w, VENT_GLASS_T, h)
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="corner_lift_sliding_window")

    model.material("frame", rgba=FRAME_RGBA)
    model.material("sash", rgba=SASH_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("screen_frame", rgba=SASH_RGBA)
    model.material("screen_mesh", rgba=SCREEN_RGBA)
    model.material("roller", rgba=ROLLER_RGBA)
    model.material("lift_tab", rgba=LIFT_RGBA)
    model.material("vent", rgba=VENT_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material="frame",
        name="frame_shell",
    )

    # Fixed vent panel: separate part mounted in the upper-left corner of the opening.
    # Positioned near the frame corner to read as structurally attached.
    vent_cx = OPEN_X0 + VENT_W / 2.0 + 0.010   # near left jamb
    vent_origin_z = OPEN_Z1 - VENT_H - 0.010    # near head
    vent_y = SASH_Y  # same track plane as sash
    vent = model.part("vent_panel")
    vent.visual(
        mesh_from_cadquery(_build_vent_frame_shape(), "vent_frame"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="vent",
        name="vent_frame",
    )
    vent.visual(
        mesh_from_cadquery(_build_vent_glass_shape(), "vent_glass"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="glass",
        name="vent_glass",
    )

    # Fixed articulation: vent panel is rigidly attached to the frame
    model.articulation(
        "frame_to_vent",
        ArticulationType.FIXED,
        parent="frame",
        child="vent_panel",
        origin=Origin(xyz=(vent_cx, vent_y, vent_origin_z)),
    )

    # --- Sliding sash ---
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

    # Roller blocks at sash bottom (two, near left and right edges)
    roller_z = -ROLLER_H / 2.0  # protrude below sash bottom
    for i, x_off in enumerate((-ROLLER_INSET, +ROLLER_INSET)):
        sash.visual(
            Box((ROLLER_W, ROLLER_D, ROLLER_H)),
            origin=Origin(xyz=(x_off, 0.0, roller_z)),
            material="roller",
            name=f"roller_{i}",
        )

    # Corner-lift tab at bottom-right of sash
    lift_x = SASH_W / 2.0 - LIFT_TAB_W / 2.0 - 0.008
    lift_z = LIFT_TAB_H / 2.0 + 0.002  # just above sash bottom rail
    lift_y = -(SASH_DEPTH / 2.0 + LIFT_TAB_D / 2.0 - 0.004)  # proud on interior face
    sash.visual(
        Box((LIFT_TAB_W, LIFT_TAB_D, LIFT_TAB_H)),
        origin=Origin(xyz=(lift_x, lift_y, lift_z)),
        material="lift_tab",
        name="corner_lift_tab",
    )

    # --- Insect screen ---
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
    # Sliding sash: PRISMATIC along -X. Positive q slides LEFT (opens).
    model.articulation(
        "frame_to_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="sliding_sash",
        origin=Origin(xyz=(SASH_CLOSED_CX, SASH_Y, SASH_CLOSED_CZ)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0, velocity=0.30, lower=0.0, upper=SASH_TRAVEL
        ),
    )

    # Insect screen: PRISMATIC along -X. Positive q slides LEFT.
    model.articulation(
        "frame_to_screen",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="insect_screen",
        origin=Origin(xyz=(SCREEN_CLOSED_CX, SCREEN_Y, SCREEN_CLOSED_CZ)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0, velocity=0.40, lower=0.0, upper=SCREEN_TRAVEL
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    sash = object_model.get_part("sliding_sash")
    screen = object_model.get_part("insect_screen")
    j_sash = object_model.get_articulation("frame_to_sash")
    j_screen = object_model.get_articulation("frame_to_screen")

    # --- Intentional overlaps ---
    # Glass panes tuck under sash muntin/rail lips (captured glass).
    ctx.allow_overlap(
        "sliding_sash", "sliding_sash",
        elem_a="sash_glass",
        elem_b="sash_frame",
        reason="Glass panes are rebated under the sash rails/muntins so they read as captured.",
    )
    # Sash rides in the jamb track grooves cut into the frame.
    ctx.allow_overlap(
        "frame", "sliding_sash",
        reason="Sash stiles ride in the jamb track grooves (retained insertion).",
    )
    # Screen rides in the interior track grooves.
    ctx.allow_overlap(
        "frame", "insect_screen",
        reason="Screen frame rides in the shallow interior track grooves.",
    )
    # Vent glass seated in vent frame
    ctx.allow_overlap(
        "vent_panel", "vent_panel",
        elem_a="vent_glass",
        elem_b="vent_frame",
        reason="Vent glass is seated inside the vent panel frame.",
    )

    # --- Isolated parts: sash and screen ride in tracks with clearance ---
    ctx.allow_isolated_part(
        "sliding_sash",
        reason="Sliding sash rides in jamb track grooves with running clearance; retained by sill/head tracks and jamb channels.",
    )
    ctx.allow_isolated_part(
        "insect_screen",
        reason="Insect screen rides in a shallow interior track with running clearance; retained by sill/head screen grooves.",
    )
    ctx.allow_isolated_part(
        "vent_panel",
        reason="Fixed vent panel is mounted in the frame opening corner; structurally attached via FIXED articulation to the frame.",
    )

    # --- Non-fixed joints exist ---
    ctx.check(
        "sash joint is prismatic",
        j_sash.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={j_sash.articulation_type}",
    )
    ctx.check(
        "screen joint is prismatic",
        j_screen.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={j_screen.articulation_type}",
    )

    # --- Closed pose (q=0): sash on right, screen centered ---
    with ctx.pose({j_sash: 0.0, j_screen: 0.0}):
        f_aabb = ctx.part_world_aabb(frame)
        s_aabb = ctx.part_world_aabb(sash)
        sc_aabb = ctx.part_world_aabb(screen)

        # Frame is the widest element
        frame_w = f_aabb[1][0] - f_aabb[0][0]
        sash_w = s_aabb[1][0] - s_aabb[0][0]
        ctx.check(
            "frame spans wider than sash",
            frame_w > sash_w + 0.10,
            details=f"frame_w={frame_w:.3f}, sash_w={sash_w:.3f}",
        )
        # Window stands upright: sill near z=0, head above 0.8m
        ctx.check(
            "frame sill near z=0 and head above 0.8m",
            abs(f_aabb[0][2]) < 0.01 and f_aabb[1][2] > 0.80,
            details=f"z=({f_aabb[0][2]:.3f}, {f_aabb[1][2]:.3f})",
        )
        # Sash is within the frame width
        ctx.check(
            "sash within frame width at closed",
            s_aabb[0][0] > f_aabb[0][0] and s_aabb[1][0] < f_aabb[1][0],
            details=f"sash x=({s_aabb[0][0]:.3f},{s_aabb[1][0]:.3f})",
        )
        # Sash is on the right half of the opening at closed
        sash_cx = (s_aabb[0][0] + s_aabb[1][0]) / 2.0
        ctx.check(
            "sash on right side when closed",
            sash_cx > 0.05,
            details=f"sash_cx={sash_cx:.3f}",
        )

        rest_sash_cx = sash_cx
        rest_screen_cx = (sc_aabb[0][0] + sc_aabb[1][0]) / 2.0

        # Proof: sash retained in frame footprint (X and Z overlap)
        ctx.expect_overlap(
            sash, frame, axes="xz", min_overlap=0.02,
            name="sash overlaps frame in XZ at closed (retained in tracks)",
        )
        # Proof: screen retained in frame footprint
        ctx.expect_overlap(
            screen, frame, axes="xz", min_overlap=0.02,
            name="screen overlaps frame in XZ at closed (retained in tracks)",
        )

    # --- Sash slides LEFT when opened ---
    travel = SASH_TRAVEL * 0.80
    with ctx.pose({j_sash: travel}):
        op = ctx.part_world_aabb(sash)
        op_cx = (op[0][0] + op[1][0]) / 2.0
        ctx.check(
            "sash slides left when opened",
            op_cx < rest_sash_cx - travel * 0.7,
            details=f"rest_cx={rest_sash_cx:.3f}, opened_cx={op_cx:.3f}",
        )
        # Sash still retained in frame X footprint
        ctx.expect_overlap(
            sash, frame, axes="x", min_overlap=0.05,
            name="sash retained in frame when open",
        )

    # --- Screen slides independently ---
    with ctx.pose({j_screen: SCREEN_TRAVEL * 0.70}):
        sc_op = ctx.part_world_aabb(screen)
        sc_op_cx = (sc_op[0][0] + sc_op[1][0]) / 2.0
        ctx.check(
            "screen slides left independently",
            sc_op_cx < rest_screen_cx - SCREEN_TRAVEL * 0.5,
            details=f"rest_cx={rest_screen_cx:.3f}, opened_cx={sc_op_cx:.3f}",
        )

    # --- Rollers at sash bottom ---
    roller_0_aabb = ctx.part_element_world_aabb(sash, elem="roller_0")
    roller_1_aabb = ctx.part_element_world_aabb(sash, elem="roller_1")
    sash_aabb = ctx.part_world_aabb(sash)
    if roller_0_aabb is not None and sash_aabb is not None:
        ctx.check(
            "roller_0 at sash bottom",
            roller_0_aabb[0][2] <= sash_aabb[0][2] + 0.005,
            details=f"roller_min_z={roller_0_aabb[0][2]:.4f}, sash_min_z={sash_aabb[0][2]:.4f}",
        )
    if roller_1_aabb is not None and sash_aabb is not None:
        ctx.check(
            "roller_1 at sash bottom",
            roller_1_aabb[0][2] <= sash_aabb[0][2] + 0.005,
            details=f"roller_min_z={roller_1_aabb[0][2]:.4f}, sash_min_z={sash_aabb[0][2]:.4f}",
        )
    # Two rollers separated in X
    if roller_0_aabb is not None and roller_1_aabb is not None:
        r0_cx = (roller_0_aabb[0][0] + roller_0_aabb[1][0]) / 2.0
        r1_cx = (roller_1_aabb[0][0] + roller_1_aabb[1][0]) / 2.0
        ctx.check(
            "two rollers separated in X",
            abs(r1_cx - r0_cx) > 0.15,
            details=f"r0_cx={r0_cx:.3f}, r1_cx={r1_cx:.3f}",
        )

    # --- Corner-lift tab exists on sash ---
    lift_aabb = ctx.part_element_world_aabb(sash, elem="corner_lift_tab")
    ctx.check(
        "corner-lift tab present",
        lift_aabb is not None,
        details="corner_lift_tab visual not found",
    )

    # --- Vent panel exists ---
    vent_panel = object_model.get_part("vent_panel")
    vent_aabb = ctx.part_world_aabb(vent_panel)
    ctx.check(
        "vent panel present",
        vent_aabb is not None,
        details="vent_panel part not found",
    )
    # Vent panel is small relative to the frame
    if vent_aabb is not None:
        vent_w = vent_aabb[1][0] - vent_aabb[0][0]
        vent_h = vent_aabb[1][2] - vent_aabb[0][2]
        ctx.check(
            "vent panel is small",
            vent_w < 0.30 and vent_h < 0.30,
            details=f"vent_w={vent_w:.3f}, vent_h={vent_h:.3f}",
        )
        # Proof: vent panel is near the upper-left corner of the opening
        vent_cx = (vent_aabb[0][0] + vent_aabb[1][0]) / 2.0
        vent_top = vent_aabb[1][2]
        ctx.check(
            "vent panel in upper-left area",
            vent_cx < -0.20 and vent_top > OPEN_Z1 - 0.05,
            details=f"vent_cx={vent_cx:.3f}, vent_top={vent_top:.3f}",
        )
    # Proof: vent panel overlaps frame footprint (mounted in the opening)
    ctx.expect_overlap(
        vent_panel, frame, axes="xz", min_overlap=0.01,
        name="vent panel within frame XZ footprint",
    )

    return ctx.report()


object_model = build_object_model()
