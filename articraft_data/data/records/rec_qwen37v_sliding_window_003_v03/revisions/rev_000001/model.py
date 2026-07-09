from __future__ import annotations

# Vertical sash-style sliding window with a white frame: one fixed upper sash,
# one movable lower sash that slides vertically, and an independently sliding
# insect screen on the interior track. Deep track grooves run along the head
# (top rail) and sill (bottom rail) of the frame.
#
# Coordinate convention:
#   +Z is up. Window stands vertically: height along +Z, width along X, frame
#   depth / glazing thickness along Y. Sill sits at z=0; head at z=WIN_H.
#
# Articulation:
#   - Upper sash: FIXED (not movable).
#   - Lower sash: PRISMATIC, axis (0,0,1): positive q slides UP (opens).
#   - Insect screen: PRISMATIC, axis (0,0,1): positive q slides UP (shallow travel).

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
FRAME_DEPTH = 0.140   # outer frame jamb depth (Y) — deeper for 3 tracks

# Clear opening inside the outer frame
OPEN_W = WIN_W - 2 * FRAME_FACE
OPEN_H = WIN_H - 2 * FRAME_FACE
OPEN_X0 = -WIN_W / 2.0 + FRAME_FACE
OPEN_X1 = WIN_W / 2.0 - FRAME_FACE
OPEN_Z0 = FRAME_FACE
OPEN_Z1 = WIN_H - FRAME_FACE

# Sash geometry
SASH_W = OPEN_W + 0.006                 # stiles enter jamb grooves (retained contact)
SASH_RAIL = 0.052                       # sash perimeter member width
SASH_DEPTH = 0.034                      # sash thickness (Y)
SASH_H = OPEN_H * 0.520                 # each sash height (meet at center)
GLASS_T = 0.006                         # glazing thickness

# Track Y planes: upper sash exterior, lower sash center, screen interior.
UPPER_SASH_Y = +0.016
LOWER_SASH_Y = -0.010
SCREEN_Y = -0.058                       # far interior to clear sash lock

# Closed-pose sash positions (sash rails enter head/sill grooves for support)
LOWER_BOTTOM_Z = OPEN_Z0 - 0.005        # lower sash bottom enters sill groove
UPPER_BOTTOM_Z = OPEN_Z1 - SASH_H + 0.005  # upper sash top enters head groove

# Insect screen
SCREEN_FRAME_W = 0.022                  # screen frame member width
SCREEN_DEPTH = 0.012                     # screen thickness (much thinner than sash)
SCREEN_H = SASH_H                        # same height as sash
SCREEN_W = SASH_W
SCREEN_BOTTOM_Z = OPEN_Z0 - 0.005       # screen bottom enters sill groove

# Muntin grid: 3 columns x 2 rows of lites per sash
MUNTIN_W = 0.022
N_COLS = 3
N_ROWS = 2

# Side track channels (jamb grooves for sash stiles)
TRACK_W = 0.018
TRACK_DEPTH = 0.030

# Deep track grooves on head and sill
GROOVE_Z_DEPTH = 0.020                  # groove depth cut into rail (Z)
GROOVE_LINER_T = 0.003                  # thin liner strip at groove bottom

# Sash lock at the meeting rail
LOCK_BODY = (0.060, 0.026, 0.022)
LOCK_LEVER = (0.044, 0.012, 0.010)

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

FRAME_RGBA = (0.945, 0.945, 0.945, 1.0)       # white painted frame
SASH_RGBA = (0.965, 0.965, 0.965, 1.0)        # white sash (slightly brighter)
GLASS_RGBA = (0.30, 0.36, 0.42, 0.34)         # cool dark-tinted glass
LOCK_RGBA = (0.86, 0.87, 0.89, 1.0)           # brushed metal sash lock
SCREEN_FRAME_RGBA = (0.72, 0.74, 0.76, 1.0)   # silver aluminum screen frame
SCREEN_MESH_RGBA = (0.48, 0.50, 0.53, 0.38)   # gray semi-transparent mesh
GROOVE_RGBA = (0.35, 0.37, 0.40, 1.0)         # dark groove liner / track insert


# ---------------------------------------------------------------------------
# Static outer frame geometry (CadQuery)
# ---------------------------------------------------------------------------

def _build_frame_shape() -> cq.Workplane:
    """White outer frame: perimeter slab with central opening cut out,
    side-track channels in the jambs, and deep track grooves in head/sill.
    """
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

    # Side-track channels: grooves in the jambs for each track plane
    groove_x = FRAME_FACE * 0.55
    for sign, edge_x in ((+1.0, OPEN_X0), (-1.0, OPEN_X1)):
        cx = edge_x - sign * groove_x / 2.0
        for track_y in (UPPER_SASH_Y, LOWER_SASH_Y, SCREEN_Y):
            groove = (
                cq.Workplane("XY")
                .transformed(offset=(cx, track_y, (OPEN_Z0 + OPEN_Z1) / 2.0))
                .box(groove_x, TRACK_DEPTH, OPEN_H)
            )
            frame = frame.cut(groove)

    # Deep track grooves on SILL (bottom rail): cut downward from sill top face
    # Span full sash width (sash stiles extend beyond clear opening into jambs)
    groove_span = SASH_W + 0.010
    for track_y, gw in (
        (UPPER_SASH_Y, SASH_DEPTH + 0.006),
        (LOWER_SASH_Y, SASH_DEPTH + 0.006),
        (SCREEN_Y, SCREEN_DEPTH + 0.006),
    ):
        groove = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, track_y, OPEN_Z0 - GROOVE_Z_DEPTH / 2.0))
            .box(groove_span, gw, GROOVE_Z_DEPTH)
        )
        frame = frame.cut(groove)

    # Deep track grooves on HEAD (top rail): cut upward from head bottom face
    for track_y, gw in (
        (UPPER_SASH_Y, SASH_DEPTH + 0.006),
        (LOWER_SASH_Y, SASH_DEPTH + 0.006),
        (SCREEN_Y, SCREEN_DEPTH + 0.006),
    ):
        groove = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, track_y, OPEN_Z1 + GROOVE_Z_DEPTH / 2.0))
            .box(groove_span, gw, GROOVE_Z_DEPTH)
        )
        frame = frame.cut(groove)

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
# Insect screen geometry (CadQuery)
# ---------------------------------------------------------------------------

def _build_screen_frame_shape() -> cq.Workplane:
    """Thin insect screen perimeter frame (no muntins)."""
    w = SCREEN_W
    h = SCREEN_H
    r = SCREEN_FRAME_W
    d = SCREEN_DEPTH

    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, h / 2.0))
        .box(w, d, h)
    )

    # Inner opening (leaving just the perimeter frame)
    inner_w = w - 2 * r
    inner_h = h - 2 * r
    inner = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, h / 2.0))
        .box(inner_w, d + 0.02, inner_h)
    )

    return outer.cut(inner)


def _build_screen_mesh_shape() -> cq.Workplane:
    """Thin semi-transparent mesh panel inside the screen frame."""
    w = SCREEN_W - 2 * SCREEN_FRAME_W + 0.006
    h = SCREEN_H - 2 * SCREEN_FRAME_W + 0.006

    return (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, SCREEN_H / 2.0))
        .box(w, 0.002, h)
    )


# ---------------------------------------------------------------------------
# Part builders
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
    model = ArticulatedObject(name="sliding_sash_window")

    model.material("frame", rgba=FRAME_RGBA)
    model.material("sash", rgba=SASH_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("lock", rgba=LOCK_RGBA)
    model.material("screen_frame", rgba=SCREEN_FRAME_RGBA)
    model.material("screen_mesh", rgba=SCREEN_MESH_RGBA)
    model.material("groove", rgba=GROOVE_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame_shell"),
        material="frame",
        name="frame_shell",
    )

    # Deep track groove liner visuals on sill (visible dark inserts in grooves)
    groove_liner_w = SASH_W + 0.008  # slightly narrower than groove channel
    sill_groove_z = OPEN_Z0 - GROOVE_Z_DEPTH + GROOVE_LINER_T / 2.0
    for i, (track_y, gw) in enumerate([
        (UPPER_SASH_Y, SASH_DEPTH + 0.004),
        (LOWER_SASH_Y, SASH_DEPTH + 0.004),
        (SCREEN_Y, SCREEN_DEPTH + 0.004),
    ]):
        frame.visual(
            Box((groove_liner_w, gw - 0.004, GROOVE_LINER_T)),
            origin=Origin(xyz=(0.0, track_y, sill_groove_z)),
            material="groove",
            name=f"sill_track_groove_{i}",
        )

    # Deep track groove liner visuals on head
    head_groove_z = OPEN_Z1 + GROOVE_Z_DEPTH - GROOVE_LINER_T / 2.0
    for i, (track_y, gw) in enumerate([
        (UPPER_SASH_Y, SASH_DEPTH + 0.004),
        (LOWER_SASH_Y, SASH_DEPTH + 0.004),
        (SCREEN_Y, SCREEN_DEPTH + 0.004),
    ]):
        frame.visual(
            Box((groove_liner_w, gw - 0.004, GROOVE_LINER_T)),
            origin=Origin(xyz=(0.0, track_y, head_groove_z)),
            material="groove",
            name=f"head_track_groove_{i}",
        )

    # --- Upper sash (FIXED) ---
    _add_sash(model, "upper_sash")

    # --- Lower sash (movable) ---
    _add_sash(model, "lower_sash")

    # Sash lock on the lower sash top rail
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

    # --- Insect screen ---
    screen = model.part("insect_screen")
    screen.visual(
        mesh_from_cadquery(_build_screen_frame_shape(), "screen_frame"),
        material="screen_frame",
        name="screen_frame",
    )
    screen.visual(
        mesh_from_cadquery(_build_screen_mesh_shape(), "screen_mesh_panel"),
        material="screen_mesh",
        name="screen_mesh_panel",
    )

    # ----- Articulations -----

    # Upper sash: FIXED (not movable in a single-hung window)
    model.articulation(
        "frame_to_upper_sash",
        ArticulationType.FIXED,
        parent="frame",
        child="upper_sash",
        origin=Origin(xyz=(0.0, UPPER_SASH_Y, UPPER_BOTTOM_Z)),
    )

    # Lower sash: PRISMATIC, slides UP. axis (0,0,1), positive q opens.
    model.articulation(
        "frame_to_lower_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="lower_sash",
        origin=Origin(xyz=(0.0, LOWER_SASH_Y, LOWER_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=0.25, lower=0.0, upper=SASH_H * 0.45
        ),
    )

    # Insect screen: PRISMATIC, slides UP independently, shallow travel.
    model.articulation(
        "frame_to_screen",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="insect_screen",
        origin=Origin(xyz=(0.0, SCREEN_Y, SCREEN_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0, velocity=0.30, lower=0.0, upper=SASH_H * 0.30
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    upper = object_model.get_part("upper_sash")
    lower = object_model.get_part("lower_sash")
    screen = object_model.get_part("insect_screen")
    j_upper = object_model.get_articulation("frame_to_upper_sash")
    j_lower = object_model.get_articulation("frame_to_lower_sash")
    j_screen = object_model.get_articulation("frame_to_screen")

    # --- Verify joint types ---
    ctx.check(
        "upper sash is FIXED",
        j_upper.articulation_type == ArticulationType.FIXED,
        details=f"upper sash joint type={j_upper.articulation_type}",
    )
    ctx.check(
        "lower sash is PRISMATIC",
        j_lower.articulation_type == ArticulationType.PRISMATIC,
        details=f"lower sash joint type={j_lower.articulation_type}",
    )
    ctx.check(
        "insect screen is PRISMATIC",
        j_screen.articulation_type == ArticulationType.PRISMATIC,
        details=f"screen joint type={j_screen.articulation_type}",
    )

    # --- Verify screen joint has independent limits ---
    screen_limits = j_screen.motion_limits
    lower_limits = j_lower.motion_limits
    ctx.check(
        "screen travel is shallower than sash travel",
        screen_limits is not None and lower_limits is not None
        and screen_limits.upper < lower_limits.upper,
        details=f"screen_upper={screen_limits.upper if screen_limits else None}, "
                f"sash_upper={lower_limits.upper if lower_limits else None}",
    )

    # --- Intentional overlaps ---
    # Glass panes tuck under the sash muntin/rail lips (captured glass)
    for sash_name in ("lower_sash", "upper_sash"):
        ctx.allow_overlap(
            sash_name, sash_name,
            elem_a=f"{sash_name}_glass",
            elem_b=f"{sash_name}_frame",
            reason="Glass panes are rebated under the sash rails/muntins so they read as captured.",
        )
    # Sashes ride in the jamb side-track grooves
    ctx.allow_overlap(
        "frame", "lower_sash",
        reason="Lower sash stiles ride in the jamb track grooves (retained insertion).",
    )
    ctx.allow_overlap(
        "frame", "upper_sash",
        reason="Upper sash stiles ride in the jamb track grooves (fixed insertion).",
    )
    # Screen rides in the interior jamb track
    ctx.allow_overlap(
        "frame", "insect_screen",
        reason="Insect screen frame rides in the interior jamb track groove.",
    )
    # Sashes overlap at meeting rail (different Y planes)
    ctx.allow_overlap(
        "lower_sash", "upper_sash",
        reason="Sashes overlap by one rail at the meeting rail; they ride in offset Y planes.",
    )
    # Sash lock seated on lower sash
    ctx.allow_overlap(
        "lower_sash", "lower_sash",
        elem_a="lower_sash_lock_body",
        elem_b="lower_sash_frame",
        reason="Sash lock is mounted (seated) onto the lower sash meeting rail.",
    )
    # Screen mesh panel seated in screen frame
    ctx.allow_overlap(
        "insect_screen", "insect_screen",
        elem_a="screen_mesh_panel",
        elem_b="screen_frame",
        reason="Screen mesh panel is captured inside the screen frame perimeter.",
    )
    # Track groove liners sit inside the groove channels cut into the frame
    for i in range(3):
        ctx.allow_overlap(
            "frame", "frame",
            elem_a=f"sill_track_groove_{i}",
            elem_b="frame_shell",
            reason="Track groove liner sits inside the sill groove channel.",
        )
        ctx.allow_overlap(
            "frame", "frame",
            elem_a=f"head_track_groove_{i}",
            elem_b="frame_shell",
            reason="Track groove liner sits inside the head groove channel.",
        )

    # --- Closed pose (q=0): all parts seated ---
    with ctx.pose({j_lower: 0.0}):
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
        # Sill near z=0
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
            details=f"lower x=({lo_aabb[0][0]:.3f},{lo_aabb[1][0]:.3f})",
        )
        # Lower sash sits below the upper sash
        lo_center_z = (lo_aabb[0][2] + lo_aabb[1][2]) / 2.0
        up_center_z = (up_aabb[0][2] + up_aabb[1][2]) / 2.0
        ctx.check(
            "lower sash below upper sash at closed pose",
            lo_center_z < up_center_z - 0.2,
            details=f"lower_cz={lo_center_z:.3f}, upper_cz={up_center_z:.3f}",
        )
        # Sashes overlap at meeting rail
        ctx.check(
            "sashes overlap at meeting rail (shut)",
            lo_aabb[1][2] >= up_aabb[0][2] - 1e-4,
            details=f"lower_top={lo_aabb[1][2]:.3f}, upper_bottom={up_aabb[0][2]:.3f}",
        )
        # Screen is on the interior side (-Y) of the lower sash
        lo_cy = (lo_aabb[0][1] + lo_aabb[1][1]) / 2.0
        sc_cy = (sc_aabb[0][1] + sc_aabb[1][1]) / 2.0
        ctx.check(
            "screen rides on interior track (more -Y than lower sash)",
            sc_cy < lo_cy - 0.010,
            details=f"lower_cy={lo_cy:.3f}, screen_cy={sc_cy:.3f}",
        )
        # Screen within frame footprint
        ctx.check(
            "screen within frame footprint",
            sc_aabb[0][0] > f_aabb[0][0] and sc_aabb[1][0] < f_aabb[1][0],
            details=f"screen x=({sc_aabb[0][0]:.3f},{sc_aabb[1][0]:.3f})",
        )

        rest_lo_z = lo_center_z
        rest_lo_top = lo_aabb[1][2]
        rest_sc_z = (sc_aabb[0][2] + sc_aabb[1][2]) / 2.0

    # --- Deep track grooves: verify named groove visuals exist ---
    for i in range(3):
        sill_vis = frame.get_visual(f"sill_track_groove_{i}")
        head_vis = frame.get_visual(f"head_track_groove_{i}")
        ctx.check(
            f"sill track groove {i} exists",
            sill_vis is not None,
            details=f"visual sill_track_groove_{i}={'found' if sill_vis else 'missing'}",
        )
        ctx.check(
            f"head track groove {i} exists",
            head_vis is not None,
            details=f"visual head_track_groove_{i}={'found' if head_vis else 'missing'}",
        )

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
        # Stays retained in frame
        ctx.expect_overlap(
            lower, frame, axes="x", min_overlap=0.05,
            name="lower sash retained in frame when open",
        )

    # --- HERO: insect screen slides UP independently ---
    screen_travel = SASH_H * 0.25
    with ctx.pose({j_screen: screen_travel}):
        op_sc = ctx.part_world_aabb(screen)
        op_sc_cz = (op_sc[0][2] + op_sc[1][2]) / 2.0
        ctx.check(
            "insect screen slides up when opened",
            op_sc_cz > rest_sc_z + screen_travel * 0.8,
            details=f"rest_sc_cz={rest_sc_z:.3f}, opened_sc_cz={op_sc_cz:.3f}",
        )
        # Screen stays retained in frame
        ctx.expect_overlap(
            screen, frame, axes="x", min_overlap=0.05,
            name="screen retained in frame when open",
        )

    # --- Independence: moving screen does NOT move lower sash ---
    with ctx.pose({j_screen: screen_travel}):
        lo_at_screen_open = ctx.part_world_aabb(lower)
        lo_cz_at_screen_open = (lo_at_screen_open[0][2] + lo_at_screen_open[1][2]) / 2.0
        ctx.check(
            "lower sash stays put when screen slides (independent joints)",
            abs(lo_cz_at_screen_open - rest_lo_z) < 0.005,
            details=f"lower_cz with screen open={lo_cz_at_screen_open:.3f}, rest={rest_lo_z:.3f}",
        )

    # --- Sash lock centered ---
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
