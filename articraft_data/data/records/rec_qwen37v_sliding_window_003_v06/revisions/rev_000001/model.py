from __future__ import annotations

# Horizontal sliding window variant: slim vinyl frame with bevelled inner
# corners, two six-lite sashes (one fixed, one sliding horizontally on
# rollers), and an insect screen on an independent shallow prismatic track.
#
# Coordinate convention:
#   +Z is up.  Window stands vertically: height along +Z, width along X,
#   frame depth / glazing thickness along Y.  Sill at z=0, head at z=WIN_H.
#   +Y is exterior, -Y is interior (viewed from inside the room).
#
# Articulation:
#   - SLIDING sash: PRISMATIC along +X, positive q slides it open (right).
#   - INSECT screen: PRISMATIC along +X, independent shallow slide.
#   - FIXED sash: FIXED articulation (secured in the exterior track).

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
WIN_H = 1.22           # overall window height (Z), sill at z=0
FRAME_FACE = 0.045     # slim vinyl frame member face width
FRAME_DEPTH = 0.090    # frame jamb depth (Y)
BEVEL = 0.005          # bevelled inner-corner chamfer

# Clear opening inside the outer frame
OPEN_W = WIN_W - 2 * FRAME_FACE   # 0.830
OPEN_H = WIN_H - 2 * FRAME_FACE   # 1.130
OPEN_X0 = -OPEN_W / 2.0
OPEN_X1 = +OPEN_W / 2.0
OPEN_Z0 = FRAME_FACE
OPEN_Z1 = WIN_H - FRAME_FACE

# Sash geometry
SASH_RAIL = 0.038      # sash perimeter member width
SASH_DEPTH = 0.026     # sash thickness (Y)
SASH_H = OPEN_H - 0.006   # each sash height (small top/bottom clearance)
SASH_W = OPEN_W * 0.50 - 0.004  # each sash width (~half opening)

# Y planes: fixed sash exterior (+Y), sliding sash interior (-Y)
SASH_Y_GAP = 0.016
FIXED_SASH_Y = +SASH_Y_GAP
SLIDING_SASH_Y = -SASH_Y_GAP

# Screen track (further interior, within frame depth)
SCREEN_Y = -0.035

# Glass
GLASS_T = 0.005

# Muntin grid: 3 cols x 2 rows per sash
MUNTIN_W = 0.018
N_COLS = 3
N_ROWS = 2

# Track grooves in sill/head
TRACK_W = 0.014
TRACK_DEPTH = 0.022

# Roller blocks on sliding sash
ROLLER_SIZE = (0.020, 0.010, 0.008)   # (X, Y, Z)

# Insect screen
SCREEN_FRAME_W = 0.022
SCREEN_DEPTH = 0.018   # wider than track groove for frame contact
SCREEN_W = SASH_W - 0.005
SCREEN_H = SASH_H - 0.005

# Sash lock (on meeting stile, protrudes in X toward fixed sash)
LOCK_SIZE = (0.010, 0.024, 0.034)

# Closed-pose world X positions (sash centers)
FIXED_SASH_X = OPEN_W * 0.25         # right quarter
SLIDING_SASH_X = -OPEN_W * 0.25      # left quarter

# Sash bottom Z — seated in the sill track groove for frame contact.
# Groove depth = FRAME_FACE * 0.35 = 0.01575; groove bottom at FRAME_FACE - groove_h.
GROOVE_H = FRAME_FACE * 0.35
SASH_BOTTOM_Z = FRAME_FACE - GROOVE_H + 0.001  # 1 mm above groove bottom

# Travel limits
SLIDE_TRAVEL = SASH_W * 0.88
SCREEN_TRAVEL = SCREEN_W * 0.80

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

FRAME_RGBA = (0.92, 0.92, 0.93, 1.0)       # vinyl white
SASH_RGBA = (0.95, 0.95, 0.96, 1.0)        # slightly brighter white
GLASS_RGBA = (0.30, 0.36, 0.42, 0.34)      # cool dark-tinted glass
ROLLER_RGBA = (0.22, 0.22, 0.24, 1.0)      # dark gray nylon
SCREEN_FRAME_RGBA = (0.88, 0.88, 0.89, 1.0)  # light gray aluminum
SCREEN_MESH_RGBA = (0.18, 0.20, 0.22, 0.50)   # dark semi-transparent mesh
LOCK_RGBA = (0.82, 0.83, 0.85, 1.0)        # brushed metal


# ---------------------------------------------------------------------------
# Frame geometry (CadQuery) — slim vinyl with bevelled corners + track grooves
# ---------------------------------------------------------------------------

def _build_frame_shape() -> cq.Workplane:
    """Slim vinyl perimeter frame with bevelled inner corners and three
    horizontal track grooves in the sill and head."""
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

    # Bevel all vertical edges (inner opening + outer corners).
    # For vinyl frames this gives the characteristic eased profile.
    try:
        frame = frame.edges("|Z").chamfer(BEVEL)
    except Exception:
        pass  # geometry is still valid without bevel

    # Cut horizontal track grooves in the sill and head.
    # Three tracks: fixed sash (+Y), sliding sash (-Y), screen (further -Y).
    groove_h = GROOVE_H
    for z_base in (0.0, WIN_H):
        if z_base == 0.0:
            zc = FRAME_FACE - groove_h / 2.0   # groove cut up from sill top
        else:
            zc = WIN_H - FRAME_FACE + groove_h / 2.0  # groove cut down from head bottom
        for ty, tw, td in [
            (FIXED_SASH_Y, TRACK_W, TRACK_DEPTH),
            (SLIDING_SASH_Y, TRACK_W, TRACK_DEPTH),
            (SCREEN_Y, TRACK_W * 0.7, TRACK_DEPTH * 0.7),
        ]:
            groove = (
                cq.Workplane("XY")
                .transformed(offset=(0.0, ty, zc))
                .box(OPEN_W + 0.01, td, groove_h)
            )
            frame = frame.cut(groove)

    return frame


# ---------------------------------------------------------------------------
# Sash geometry (CadQuery): perimeter ring + 3x2 muntin grid
# ---------------------------------------------------------------------------

def _build_sash_frame_shape() -> cq.Workplane:
    """One sash: perimeter ring plus a 3x2 muntin grid."""
    w, h, r, d = SASH_W, SASH_H, SASH_RAIL, SASH_DEPTH

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
    """Six thin glass panes for one sash, rebated under the muntins."""
    w, h, r = SASH_W, SASH_H, SASH_RAIL
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
# Screen geometry (CadQuery): thin frame + mesh panel
# ---------------------------------------------------------------------------

def _build_screen_frame_shape() -> cq.Workplane:
    """Insect screen perimeter frame with inner cutout."""
    w, h, fw, d = SCREEN_W, SCREEN_H, SCREEN_FRAME_W, SCREEN_DEPTH

    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, h / 2.0))
        .box(w, d, h)
    )
    inner = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, h / 2.0))
        .box(w - 2 * fw, d + 0.02, h - 2 * fw)
    )
    return outer.cut(inner)


def _build_screen_mesh_shape() -> cq.Workplane:
    """Thin semi-transparent mesh panel filling the screen frame opening.
    Extends 3 mm into the frame on each side (captured mesh)."""
    w, h, fw = SCREEN_W, SCREEN_H, SCREEN_FRAME_W
    mesh_w = w - 2 * fw + 0.006
    mesh_h = h - 2 * fw + 0.006
    return (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, h / 2.0))
        .box(mesh_w, 0.004, mesh_h)
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_window")

    model.material("frame", rgba=FRAME_RGBA)
    model.material("sash", rgba=SASH_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("roller", rgba=ROLLER_RGBA)
    model.material("screen_frame", rgba=SCREEN_FRAME_RGBA)
    model.material("screen_mesh", rgba=SCREEN_MESH_RGBA)
    model.material("lock", rgba=LOCK_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame_shell"),
        material="frame",
        name="frame_shell",
    )

    # --- Fixed sash (right side, exterior track) ---
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

    # --- Sliding sash (left side, interior track) ---
    sliding = model.part("sliding_sash")
    sliding.visual(
        mesh_from_cadquery(_build_sash_frame_shape(), "sliding_sash_frame"),
        material="sash",
        name="sliding_sash_frame",
    )
    sliding.visual(
        mesh_from_cadquery(_build_sash_glass_shape(), "sliding_sash_glass"),
        material="glass",
        name="sliding_sash_glass",
    )

    # Two tiny roller blocks at the bottom of the sliding sash
    roller_x_offset = SASH_W / 2.0 - SASH_RAIL
    roller_y = 0.0  # centered in sash depth
    roller_z = -0.002  # protrudes below sash bottom into track
    for i, sx in enumerate((-1.0, +1.0)):
        sliding.visual(
            Box(ROLLER_SIZE),
            origin=Origin(xyz=(sx * roller_x_offset, roller_y, roller_z)),
            material="roller",
            name=f"roller_{i}",
        )

    # Sash lock/latch on the meeting stile of the sliding sash (protrudes in +X)
    lock_x = SASH_W / 2.0 + LOCK_SIZE[0] / 2.0 - 0.003  # on right stile, protruding
    lock_y = 0.0  # centered in sash depth
    lock_z = SASH_H * 0.50
    sliding.visual(
        Box(LOCK_SIZE),
        origin=Origin(xyz=(lock_x, lock_y, lock_z)),
        material="lock",
        name="sash_lock",
    )

    # --- Insect screen (independent track, interior side) ---
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

    # Fixed sash: secured in the exterior track
    model.articulation(
        "frame_to_fixed_sash",
        ArticulationType.FIXED,
        parent="frame",
        child="fixed_sash",
        origin=Origin(xyz=(FIXED_SASH_X, FIXED_SASH_Y, SASH_BOTTOM_Z)),
    )

    # Sliding sash: prismatic along +X (positive q slides right = opens)
    model.articulation(
        "frame_to_sliding_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="sliding_sash",
        origin=Origin(xyz=(SLIDING_SASH_X, SLIDING_SASH_Y, SASH_BOTTOM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0, velocity=0.30, lower=0.0, upper=SLIDE_TRAVEL,
        ),
    )

    # Insect screen: independent shallow prismatic along +X
    screen_origin_x = SLIDING_SASH_X  # same starting X as sliding sash
    model.articulation(
        "frame_to_screen",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="insect_screen",
        origin=Origin(xyz=(screen_origin_x, SCREEN_Y, SASH_BOTTOM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0, velocity=0.40, lower=0.0, upper=SCREEN_TRAVEL,
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
    screen = object_model.get_part("insect_screen")
    j_slide = object_model.get_articulation("frame_to_sliding_sash")
    j_screen = object_model.get_articulation("frame_to_screen")

    # --- Intentional overlap allowances ---
    # Glass rebated under muntins
    for sash_name in ("fixed_sash", "sliding_sash"):
        ctx.allow_overlap(
            sash_name, sash_name,
            elem_a=f"{sash_name}_glass",
            elem_b=f"{sash_name}_frame",
            reason="Glass panes are rebated under the sash rails/muntins (captured glazing).",
        )

    # Sashes ride in frame track grooves
    ctx.allow_overlap(
        "frame", "fixed_sash",
        reason="Fixed sash sits in the exterior jamb track groove.",
    )
    ctx.allow_overlap(
        "frame", "sliding_sash",
        reason="Sliding sash rides in the interior jamb track groove.",
    )
    # Sashes in offset Y planes overlap when slider is open
    ctx.allow_overlap(
        "fixed_sash", "sliding_sash",
        reason="Sashes ride in offset Y planes and pass each other horizontally.",
    )
    # Lock seated on sliding sash meeting stile
    ctx.allow_overlap(
        "sliding_sash", "sliding_sash",
        elem_a="sash_lock", elem_b="sliding_sash_frame",
        reason="Sash lock is seated onto the sliding sash meeting stile.",
    )
    # Rollers embedded in sliding sash bottom rail
    for roller_name in ("roller_0", "roller_1"):
        ctx.allow_overlap(
            "sliding_sash", "sliding_sash",
            elem_a=roller_name, elem_b="sliding_sash_frame",
            reason="Roller block is partially embedded in the sliding sash bottom rail.",
        )
    # Screen rides in its own track
    ctx.allow_overlap(
        "frame", "insect_screen",
        reason="Insect screen rides in its own shallow interior track groove.",
    )
    # Screen mesh captured in screen frame
    ctx.allow_overlap(
        "insect_screen", "insect_screen",
        elem_a="screen_mesh", elem_b="screen_frame",
        reason="Screen mesh panel is captured within the screen frame perimeter.",
    )

    # --- Non-fixed joint checks ---
    ctx.check(
        "sliding sash joint is prismatic",
        j_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={j_slide.articulation_type}",
    )
    ctx.check(
        "screen joint is prismatic (independent)",
        j_screen.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={j_screen.articulation_type}",
    )

    # --- Slim vinyl frame check ---
    f_aabb = ctx.part_world_aabb(frame)
    if f_aabb is not None:
        frame_depth = f_aabb[1][1] - f_aabb[0][1]
        ctx.check(
            "frame has slim vinyl depth (< 0.10 m)",
            frame_depth < 0.100,
            details=f"frame_depth={frame_depth:.3f}",
        )

    # --- Closed pose (q=0): both sashes in place, window reads shut ---
    with ctx.pose({j_slide: 0.0, j_screen: 0.0}):
        f_aabb = ctx.part_world_aabb(frame)
        fix_aabb = ctx.part_world_aabb(fixed)
        slide_aabb = ctx.part_world_aabb(sliding)

        # Frame stands upright
        ctx.check(
            "frame stands upright (sill near z=0, head > 1 m)",
            abs(f_aabb[0][2]) < 0.01 and f_aabb[1][2] > 1.0,
            details=f"frame z=({f_aabb[0][2]:.3f}, {f_aabb[1][2]:.3f})",
        )

        # Sashes are side by side (fixed right, sliding left)
        fix_cx = (fix_aabb[0][0] + fix_aabb[1][0]) / 2.0
        slide_cx = (slide_aabb[0][0] + slide_aabb[1][0]) / 2.0
        ctx.check(
            "sashes side by side (fixed right of sliding)",
            fix_cx > slide_cx + 0.10,
            details=f"fixed_cx={fix_cx:.3f}, sliding_cx={slide_cx:.3f}",
        )

        # Both sashes within frame width
        ctx.check(
            "both sashes within frame width",
            fix_aabb[0][0] > f_aabb[0][0] and fix_aabb[1][0] < f_aabb[1][0]
            and slide_aabb[0][0] > f_aabb[0][0] and slide_aabb[1][0] < f_aabb[1][0],
            details="sash extends outside frame",
        )

        # Sashes in offset Y planes
        fix_cy = (fix_aabb[0][1] + fix_aabb[1][1]) / 2.0
        slide_cy = (slide_aabb[0][1] + slide_aabb[1][1]) / 2.0
        ctx.check(
            "sashes ride in offset Y planes",
            abs(fix_cy - slide_cy) > 0.015,
            details=f"fixed_cy={fix_cy:.3f}, sliding_cy={slide_cy:.3f}",
        )

        # Roller blocks present at the bottom of the sliding sash
        roller_0_aabb = ctx.part_element_world_aabb(sliding, elem="roller_0")
        roller_1_aabb = ctx.part_element_world_aabb(sliding, elem="roller_1")
        ctx.check(
            "two roller blocks present on sliding sash",
            roller_0_aabb is not None and roller_1_aabb is not None,
            details="missing roller visual(s)",
        )
        if roller_0_aabb is not None and roller_1_aabb is not None:
            # Rollers near the sash bottom
            r0_bottom = roller_0_aabb[0][2]
            sash_bottom = slide_aabb[0][2]
            ctx.check(
                "rollers near bottom of sliding sash",
                abs(r0_bottom - sash_bottom) < 0.020,
                details=f"roller_bottom={r0_bottom:.3f}, sash_bottom={sash_bottom:.3f}",
            )
            # Rollers separated in X (near left and right stiles)
            r0_cx = (roller_0_aabb[0][0] + roller_0_aabb[1][0]) / 2.0
            r1_cx = (roller_1_aabb[0][0] + roller_1_aabb[1][0]) / 2.0
            ctx.check(
                "rollers separated near left and right stiles",
                abs(r1_cx - r0_cx) > SASH_W * 0.5,
                details=f"roller_0_cx={r0_cx:.3f}, roller_1_cx={r1_cx:.3f}",
            )

        rest_slide_cx = slide_cx

    # --- Sliding sash opens (slides right along +X) ---
    travel = SLIDE_TRAVEL * 0.85
    with ctx.pose({j_slide: travel}):
        op_aabb = ctx.part_world_aabb(sliding)
        op_cx = (op_aabb[0][0] + op_aabb[1][0]) / 2.0
        ctx.check(
            "sliding sash moves right when opened",
            op_cx > rest_slide_cx + travel * 0.8,
            details=f"rest_cx={rest_slide_cx:.3f}, opened_cx={op_cx:.3f}, travel={travel:.3f}",
        )
        # Still retained in frame height
        ctx.expect_overlap(
            sliding, frame, axes="z", min_overlap=0.05,
            name="sliding sash retained in frame when open",
        )

    # --- Screen slides independently ---
    with ctx.pose({j_slide: 0.0, j_screen: 0.0}):
        screen_rest = ctx.part_world_aabb(screen)
        screen_rest_cx = (screen_rest[0][0] + screen_rest[1][0]) / 2.0

    screen_travel = SCREEN_TRAVEL * 0.60
    with ctx.pose({j_slide: 0.0, j_screen: screen_travel}):
        screen_op = ctx.part_world_aabb(screen)
        screen_op_cx = (screen_op[0][0] + screen_op[1][0]) / 2.0
        ctx.check(
            "insect screen slides independently on prismatic joint",
            screen_op_cx > screen_rest_cx + screen_travel * 0.7,
            details=f"rest_cx={screen_rest_cx:.3f}, opened_cx={screen_op_cx:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
