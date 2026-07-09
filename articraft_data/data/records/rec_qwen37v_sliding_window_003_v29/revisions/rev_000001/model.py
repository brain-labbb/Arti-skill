from __future__ import annotations

# Horizontal sliding window with white frame, two side-by-side six-lite sashes,
# one fixed and one sliding left-right on a prismatic joint. An insect screen
# panel rides in a separate outer track. Deep track grooves run along the top
# and bottom frame rails. A recessed pull cup is on the sliding sash stile.
#
# Coordinate convention:
#   +Z is up. Window stands vertically: height along +Z, width along X, frame
#   depth / glazing thickness along Y. The sill sits at z=0; the head at z=WIN_H.
#
# Articulation:
#   - sliding_sash is PRISMATIC, axis (1,0,0): positive q slides it RIGHT (opens).
#   - insect_screen is PRISMATIC, axis (1,0,0): positive q slides it RIGHT.

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

# Track groove geometry - deep horizontal channels in top/bottom frame rails
TRACK_DEPTH = 0.022     # how deep the groove cuts into the rail (Z direction)
TRACK_WIDTH = 0.030     # groove width in Y (accommodates sash thickness)
SCREEN_TRACK_WIDTH = 0.018  # narrower track for the screen

# Y positions for tracks (3 tracks: screen exterior, fixed sash middle, sliding sash interior)
SCREEN_TRACK_Y = FRAME_DEPTH / 2.0 - SCREEN_TRACK_WIDTH / 2.0 - 0.004  # near exterior
FIXED_SASH_Y = 0.010          # slightly toward exterior
SLIDING_SASH_Y = -0.018       # slightly toward interior

# Sash geometry
SASH_RAIL = 0.050       # sash perimeter member width
SASH_DEPTH = 0.032      # sash thickness (Y)
SASH_H = OPEN_H - 0.008 # sash height (small clearance top/bottom in track)
SASH_W = OPEN_W * 0.51  # each sash is just over half the opening (overlap at center)
GLASS_T = 0.006

# Closed pose X positions (sash center X at rest)
FIXED_SASH_X = OPEN_X0 + SASH_W / 2.0 - 0.003   # left side, small clearance to jamb
SLIDING_SASH_X = OPEN_X1 - SASH_W / 2.0 + 0.003  # right side

# Insect screen dimensions
SCREEN_FRAME_W = 0.012    # screen frame member width
SCREEN_DEPTH = 0.014      # screen frame thickness (thinner than sash)
SCREEN_W = OPEN_W * 0.50  # covers roughly one sash width
SCREEN_H = OPEN_H - 0.006
SCREEN_X_REST = OPEN_X0 + SCREEN_W / 2.0 - 0.003  # starts at left, exterior track

# Pull cup on the sliding sash
PULL_CUP_W = 0.050
PULL_CUP_H = 0.022
PULL_CUP_DEPTH = 0.008

# Muntin grid: 3 columns x 2 rows per sash
MUNTIN_W = 0.020
N_COLS = 3
N_ROWS = 2

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

FRAME_RGBA = (0.945, 0.945, 0.945, 1.0)
SASH_RGBA = (0.965, 0.965, 0.965, 1.0)
GLASS_RGBA = (0.30, 0.36, 0.42, 0.34)
SCREEN_FRAME_RGBA = (0.70, 0.72, 0.70, 1.0)   # aluminum screen frame
SCREEN_MESH_RGBA = (0.45, 0.47, 0.45, 0.60)    # dark screen mesh
PULL_CUP_RGBA = (0.85, 0.86, 0.88, 1.0)        # brushed metal pull


# ---------------------------------------------------------------------------
# Frame geometry with deep track grooves in top/bottom rails
# ---------------------------------------------------------------------------

def _build_frame_shape() -> cq.Workplane:
    """Outer perimeter frame with the central opening cut out, plus deep
    horizontal track grooves in the top (head) and bottom (sill) rails for
    sash and screen tracks."""
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

    # Deep track grooves in the BOTTOM rail (sill) and TOP rail (head).
    # These are horizontal channels running the full opening width, cut into
    # the top/bottom frame rails. Each groove accommodates one track (sash or screen).
    groove_length = OPEN_W + 0.010  # slightly wider than opening for sash travel

    # Track Y positions for the grooves
    track_positions = [
        (SCREEN_TRACK_Y, SCREEN_TRACK_WIDTH),    # screen track (exterior)
        (FIXED_SASH_Y, TRACK_WIDTH),             # fixed sash track (middle)
        (SLIDING_SASH_Y, TRACK_WIDTH),           # sliding sash track (interior)
    ]

    for track_y, track_w in track_positions:
        # Bottom rail groove (cut upward from sill interior face)
        bottom_groove = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, track_y, OPEN_Z0 - TRACK_DEPTH / 2.0))
            .box(groove_length, track_w, TRACK_DEPTH)
        )
        frame = frame.cut(bottom_groove)

        # Top rail groove (cut downward from head interior face)
        top_groove = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, track_y, OPEN_Z1 + TRACK_DEPTH / 2.0))
            .box(groove_length, track_w, TRACK_DEPTH)
        )
        frame = frame.cut(top_groove)

    return frame


# ---------------------------------------------------------------------------
# Sash frame geometry (CadQuery): perimeter ring + 6-lite muntin grid
# ---------------------------------------------------------------------------

def _build_sash_frame_shape() -> cq.Workplane:
    """One sash: perimeter ring plus 3x2 muntin grid.
    Local frame: X centered, Z from 0 to SASH_H, Y centered at 0."""
    w = SASH_W
    h = SASH_H
    r = SASH_RAIL
    d = SASH_DEPTH

    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, h / 2.0))
        .box(w, d, h)
    )

    # Inner glazed region
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
    """Six thin glass panes for the sash openings."""
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
# Insect screen geometry
# ---------------------------------------------------------------------------

def _build_screen_frame_shape() -> cq.Workplane:
    """Screen frame: thin perimeter ring (aluminum), local Z from 0 to SCREEN_H."""
    w = SCREEN_W
    h = SCREEN_H
    f = SCREEN_FRAME_W
    d = SCREEN_DEPTH

    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, h / 2.0))
        .box(w, d, h)
    )
    # Cut the inner opening (leaving just the frame perimeter)
    inner = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, h / 2.0))
        .box(w - 2 * f, d + 0.02, h - 2 * f)
    )
    return outer.cut(inner)


# ---------------------------------------------------------------------------
# Pull cup geometry (recessed handle on sliding sash stile)
# ---------------------------------------------------------------------------

def _build_pull_cup_shape() -> cq.Workplane:
    """Recessed pull cup: a shallow rectangular dish cut into the stile face."""
    # Outer cup body (slightly proud of the stile face)
    cup = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, 0.0))
        .box(PULL_CUP_W, PULL_CUP_DEPTH, PULL_CUP_H)
    )
    # Hollow recess inside
    recess = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, PULL_CUP_DEPTH * 0.3, 0.0))
        .box(PULL_CUP_W - 0.008, PULL_CUP_DEPTH * 0.7, PULL_CUP_H - 0.006)
    )
    return cup.cut(recess)


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
    model.material("pull_cup", rgba=PULL_CUP_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame_shell"),
        material="frame",
        name="frame_shell",
    )

    # --- Fixed sash (left side, stationary) ---
    fixed = model.part("fixed_sash")
    fixed.visual(
        mesh_from_cadquery(_build_sash_frame_shape(), "fixed_sash_frame"),
        material="sash",
        name="fixed_sash_frame",
    )
    fixed.visual(
        mesh_from_cadquery(_build_sash_glass_shape(), "fixed_sash_glass"),
        material="glass",
        name="fixed_sash_glass",
    )

    # --- Sliding sash (right side, slides left-right) ---
    slider = model.part("sliding_sash")
    slider.visual(
        mesh_from_cadquery(_build_sash_frame_shape(), "sliding_sash_frame"),
        material="sash",
        name="sliding_sash_frame",
    )
    slider.visual(
        mesh_from_cadquery(_build_sash_glass_shape(), "sliding_sash_glass"),
        material="glass",
        name="sliding_sash_glass",
    )
    # Recessed pull cup on the interior-facing stile of the sliding sash (left stile)
    # Position: on the left stile (-X side of sash), interior face (-Y), at mid-height
    pull_x = -SASH_W / 2.0 + SASH_RAIL / 2.0
    pull_y = -(SASH_DEPTH / 2.0 + PULL_CUP_DEPTH / 2.0 - 0.003)
    pull_z = SASH_H / 2.0
    slider.visual(
        mesh_from_cadquery(_build_pull_cup_shape(), "pull_cup"),
        origin=Origin(xyz=(pull_x, pull_y, pull_z)),
        material="pull_cup",
        name="pull_cup",
    )

    # --- Insect screen panel (outer track, slides left-right) ---
    screen = model.part("insect_screen")
    screen.visual(
        mesh_from_cadquery(_build_screen_frame_shape(), "screen_frame"),
        material="screen_frame",
        name="screen_frame",
    )
    # Screen mesh: thin semi-transparent panel filling the inner opening.
    # The dark translucent material reads as insect screen mesh.
    mesh_w = SCREEN_W - 2 * SCREEN_FRAME_W
    mesh_h = SCREEN_H - 2 * SCREEN_FRAME_W
    screen.visual(
        Box((mesh_w, 0.001, mesh_h)),
        origin=Origin(xyz=(0.0, 0.0, SCREEN_H / 2.0)),
        material="screen_mesh",
        name="screen_mesh",
    )

    # ----- Articulations -----
    # FIXED SASH: rigid mount (no articulation needed - it's fixed in place)
    # Position it via the joint origin at its closed world position.
    model.articulation(
        "frame_to_fixed_sash",
        ArticulationType.FIXED,
        parent="frame",
        child="fixed_sash",
        origin=Origin(xyz=(FIXED_SASH_X, FIXED_SASH_Y, OPEN_Z0 + 0.004)),
    )

    # SLIDING SASH: prismatic, axis (1,0,0), positive q slides RIGHT (opens).
    # At q=0 it is closed (right side). At q=max it slides left to fully open.
    # Actually, let's make positive q slide LEFT (open) - the sash moves from right to left.
    # axis (1,0,0) with positive q moving +X means it slides right.
    # For a right-side sash sliding left to open: axis (-1,0,0), positive q moves left.
    slide_travel = OPEN_W * 0.44  # how far it can slide
    model.articulation(
        "frame_to_sliding_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="sliding_sash",
        origin=Origin(xyz=(SLIDING_SASH_X, SLIDING_SASH_Y, OPEN_Z0 + 0.004)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0, velocity=0.30, lower=0.0, upper=slide_travel
        ),
    )

    # INSECT SCREEN: prismatic, slides on the outer track
    model.articulation(
        "frame_to_screen",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="insect_screen",
        origin=Origin(xyz=(SCREEN_X_REST, SCREEN_TRACK_Y, OPEN_Z0 + 0.003)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0, velocity=0.25, lower=0.0, upper=OPEN_W * 0.44
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
    slider = object_model.get_part("sliding_sash")
    screen = object_model.get_part("insect_screen")
    j_slider = object_model.get_articulation("frame_to_sliding_sash")
    j_screen = object_model.get_articulation("frame_to_screen")

    # --- Intentional overlaps ---
    # Glass panes sit under sash muntin/rail lips (captured glass)
    for sash_name in ("fixed_sash", "sliding_sash"):
        ctx.allow_overlap(
            sash_name, sash_name,
            elem_a=f"{sash_name}_glass",
            elem_b=f"{sash_name}_frame",
            reason="Glass panes are rebated under the sash rails/muntins so they read as captured.",
        )
    # Pull cup is seated into the sliding sash stile
    ctx.allow_overlap(
        "sliding_sash", "sliding_sash",
        elem_a="pull_cup",
        elem_b="sliding_sash_frame",
        reason="Pull cup is recessed into the sliding sash stile (seated mount).",
    )
    # Sashes ride in frame track grooves
    ctx.allow_overlap(
        "frame", "fixed_sash",
        reason="Fixed sash top/bottom rails seat in the frame track grooves.",
    )
    ctx.allow_overlap(
        "frame", "sliding_sash",
        reason="Sliding sash rides in the frame track grooves (retained insertion).",
    )
    # Screen rides in the outer track
    ctx.allow_overlap(
        "frame", "insect_screen",
        reason="Insect screen panel rides in the outer frame track groove.",
    )
    # Screen mesh sits within the screen frame
    ctx.allow_overlap(
        "insect_screen", "insect_screen",
        elem_a="screen_mesh",
        elem_b="screen_frame",
        reason="Screen mesh is captured within the screen frame perimeter.",
    )

    # --- Closed pose checks ---
    with ctx.pose({j_slider: 0.0, j_screen: 0.0}):
        f_aabb = ctx.part_world_aabb(frame)
        fix_aabb = ctx.part_world_aabb(fixed)
        slide_aabb = ctx.part_world_aabb(slider)
        scr_aabb = ctx.part_world_aabb(screen)

        # Frame is the tallest/widest element
        ctx.check(
            "frame is tallest element",
            f_aabb[1][2] > 1.0 and f_aabb[1][0] - f_aabb[0][0] > 0.8,
            details=f"frame z_max={f_aabb[1][2]:.3f}, width={f_aabb[1][0] - f_aabb[0][0]:.3f}",
        )
        # Sill near z=0
        ctx.check(
            "frame sill near z=0",
            abs(f_aabb[0][2]) < 0.01,
            details=f"frame z_min={f_aabb[0][2]:.3f}",
        )
        # Both sashes are within frame width
        ctx.check(
            "sashes within frame opening width",
            fix_aabb[0][0] > f_aabb[0][0] and fix_aabb[1][0] < f_aabb[1][0]
            and slide_aabb[0][0] > f_aabb[0][0] and slide_aabb[1][0] < f_aabb[1][0],
            details=f"fixed x=({fix_aabb[0][0]:.3f},{fix_aabb[1][0]:.3f}) sliding x=({slide_aabb[0][0]:.3f},{slide_aabb[1][0]:.3f})",
        )
        # Fixed sash is to the LEFT of sliding sash at rest
        fix_cx = (fix_aabb[0][0] + fix_aabb[1][0]) / 2.0
        slide_cx = (slide_aabb[0][0] + slide_aabb[1][0]) / 2.0
        ctx.check(
            "fixed sash left of sliding sash at rest",
            fix_cx < slide_cx - 0.05,
            details=f"fixed_cx={fix_cx:.3f}, sliding_cx={slide_cx:.3f}",
        )
        # Both sashes are roughly the same height (full opening)
        fix_h = fix_aabb[1][2] - fix_aabb[0][2]
        slide_h = slide_aabb[1][2] - slide_aabb[0][2]
        ctx.check(
            "sashes are full-height (same approximate height)",
            abs(fix_h - slide_h) < 0.01 and fix_h > 1.0,
            details=f"fixed_h={fix_h:.3f}, sliding_h={slide_h:.3f}",
        )
        # Screen is in the outer track (different Y from sashes)
        scr_cy = (scr_aabb[0][1] + scr_aabb[1][1]) / 2.0
        slide_cy = (slide_aabb[0][1] + slide_aabb[1][1]) / 2.0
        ctx.check(
            "screen in separate outer track (Y offset from sash)",
            abs(scr_cy - slide_cy) > 0.010,
            details=f"screen_cy={scr_cy:.3f}, sliding_cy={slide_cy:.3f}",
        )

        rest_slide_cx = slide_cx

    # --- HERO: sliding sash slides LEFT (opens) ---
    travel = OPEN_W * 0.40
    with ctx.pose({j_slider: travel}):
        op = ctx.part_world_aabb(slider)
        op_cx = (op[0][0] + op[1][0]) / 2.0
        ctx.check(
            "sliding sash moves left when opened",
            op_cx < rest_slide_cx - travel * 0.8,
            details=f"rest_cx={rest_slide_cx:.3f}, opened_cx={op_cx:.3f}, travel={travel:.3f}",
        )
        # Still retained within frame height
        ctx.expect_overlap(
            slider, frame, axes="z", min_overlap=0.10,
            name="sliding sash retained in frame height when open",
        )

    # --- HERO: screen slides RIGHT ---
    with ctx.pose({j_screen: travel}):
        op = ctx.part_world_aabb(screen)
        op_cx = (op[0][0] + op[1][0]) / 2.0
        scr_rest_aabb = ctx.part_world_aabb(screen)
        ctx.check(
            "screen slides right when opened",
            op_cx > SCREEN_X_REST + travel * 0.5,
            details=f"screen_rest_x={SCREEN_X_REST:.3f}, opened_cx={op_cx:.3f}",
        )

    # --- Pull cup exists on sliding sash ---
    pull_aabb = ctx.part_element_world_aabb(slider, elem="pull_cup")
    ctx.check(
        "pull cup visual exists on sliding sash",
        pull_aabb is not None,
        details="pull_cup element not found",
    )

    # --- Prismatic joint axis check ---
    j_info = object_model.get_articulation("frame_to_sliding_sash")
    ctx.check(
        "sliding sash has prismatic joint",
        j_info.articulation_type == ArticulationType.PRISMATIC,
        details=f"joint type={j_info.articulation_type}",
    )

    # --- Screen has non-fixed joint ---
    j_scr = object_model.get_articulation("frame_to_screen")
    ctx.check(
        "insect screen has prismatic joint",
        j_scr.articulation_type == ArticulationType.PRISMATIC,
        details=f"screen joint type={j_scr.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
