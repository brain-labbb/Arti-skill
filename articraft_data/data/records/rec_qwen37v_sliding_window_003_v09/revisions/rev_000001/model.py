from __future__ import annotations

# Horizontal sliding window variant: white frame, two side-by-side six-lite
# sashes that slide horizontally, with an outer insect screen panel in a
# separate track, tilt-in latches on revolute pivots, and deep track grooves
# along top and bottom rails.
#
# Coordinate convention:
#   +Z is up. Window stands vertically: height along +Z, width along X, frame
#   depth / glazing thickness along Y (the glass plane is the X-Z plane). The
#   sill sits at z=0; the head is at z=WIN_H.
#
# Articulation (horizontal slider):
#   - INNER sash is PRISMATIC, axis (1,0,0): positive q slides it RIGHT (opens).
#   - OUTER sash is PRISMATIC, axis (-1,0,0): positive q slides it LEFT (opens).
#   Both sashes stay retained in the top/bottom tracks at full travel.
#   - Each sash has a tilt-in LATCH on a REVOLUTE joint that pivots around X.
#   - Insect screen panel is FIXED in its own exterior track.

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
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
FRAME_DEPTH = 0.130   # outer frame depth (Y) – deeper for 3 tracks

# Clear opening inside the outer frame
OPEN_W = WIN_W - 2 * FRAME_FACE   # clear width
OPEN_H = WIN_H - 2 * FRAME_FACE   # clear height
OPEN_X0 = -WIN_W / 2.0 + FRAME_FACE
OPEN_X1 = WIN_W / 2.0 - FRAME_FACE
OPEN_Z0 = FRAME_FACE
OPEN_Z1 = WIN_H - FRAME_FACE

# Sash geometry. Each sash covers slightly more than half the clear width so
# they overlap at the meeting stile in the center.
SASH_W = OPEN_W * 0.53              # each sash width (overlap at center stile)
SASH_RAIL = 0.050                   # sash perimeter member width (stile/rail)
SASH_DEPTH = 0.032                  # sash thickness (Y)
SASH_H = OPEN_H - 0.012            # sash height (clearance in top/bottom tracks)
GLASS_T = 0.006                     # glazing thickness (Y)

# Y planes: inner sash rides at interior (-Y), outer sash at center, screen at exterior (+Y)
TRACK_SPACING = 0.036               # center-to-center spacing between tracks
INNER_SASH_Y = -TRACK_SPACING
OUTER_SASH_Y = 0.0
SCREEN_Y = +TRACK_SPACING

# Closed-pose sash positions (world X of sash left edge).
# Inner sash (left half) and outer sash (right half), overlapping at center.
INNER_SASH_CLOSED_X = OPEN_X0 + SASH_W / 2.0 - 0.003  # left sash, centered on left half
OUTER_SASH_CLOSED_X = OPEN_X1 - SASH_W / 2.0 + 0.003  # right sash, centered on right half

# Sash center Z (vertically centered in opening)
SASH_CENTER_Z = (OPEN_Z0 + OPEN_Z1) / 2.0

# Muntin grid: 3 columns x 2 rows of lites per sash -> 2 vertical + 1 horizontal bar.
MUNTIN_W = 0.022
N_COLS = 3
N_ROWS = 2

# Deep track grooves in head and sill (top/bottom rails of the frame)
TRACK_GROOVE_W = 0.020     # width of each groove (X direction - but groove runs along X)
TRACK_GROOVE_DEPTH = 0.028 # depth of groove into the frame member (Z direction)
TRACK_GROOVE_SPACING = TRACK_SPACING  # matches sash track Y planes

# Insect screen dimensions
SCREEN_FRAME_W = 0.012     # screen frame member width
SCREEN_T = 0.010           # screen panel total thickness (Y)
SCREEN_MESH_T = 0.002      # mesh material thickness

# Tilt-in latch dimensions
LATCH_BODY = (0.030, 0.014, 0.010)   # (X, Y, Z)
LATCH_LEVER = (0.040, 0.008, 0.006)  # the pivoting lever

# Sash lock at the meeting stile (optional, on inner sash)
LOCK_BODY = (0.020, 0.026, 0.050)   # (X, Y, Z)
LOCK_LEVER = (0.010, 0.012, 0.040)

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

FRAME_RGBA = (0.945, 0.945, 0.945, 1.0)   # white painted frame
SASH_RGBA = (0.965, 0.965, 0.965, 1.0)    # white sash
GLASS_RGBA = (0.30, 0.36, 0.42, 0.34)     # cool dark-tinted glass
LOCK_RGBA = (0.86, 0.87, 0.89, 1.0)       # brushed metal
SCREEN_FRAME_RGBA = (0.88, 0.88, 0.88, 1.0)  # aluminum screen frame
SCREEN_MESH_RGBA = (0.35, 0.35, 0.35, 0.6)   # dark screen mesh
LATCH_RGBA = (0.80, 0.82, 0.84, 1.0)      # metal latch


# ---------------------------------------------------------------------------
# Static outer frame geometry (CadQuery) – with deep track grooves
# ---------------------------------------------------------------------------

def _build_frame_shape() -> cq.Workplane:
    """White outer frame: perimeter slab with central opening cut out,
    plus deep track grooves in the head (top) and sill (bottom) rails for
    three parallel sash/screen tracks, plus shallow side grooves in jambs.

    World frame: opening centered on X=0, Z from 0 (sill) to WIN_H (head).
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

    # Deep track grooves in the SILL (bottom rail, near z=0).
    # Three parallel grooves cut upward from the sill interior face into the
    # bottom frame member. Each groove matches a sash/screen track Y plane.
    groove_span = OPEN_W + 0.010  # grooves span the full opening width plus a bit
    for track_y in (INNER_SASH_Y, OUTER_SASH_Y, SCREEN_Y):
        sill_groove = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, track_y, FRAME_FACE / 2.0))
            .box(groove_span, TRACK_GROOVE_W, FRAME_FACE + 0.010)
        )
        frame = frame.cut(sill_groove)

    # Deep track grooves in the HEAD (top rail, near z=WIN_H).
    for track_y in (INNER_SASH_Y, OUTER_SASH_Y, SCREEN_Y):
        head_groove = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, track_y, WIN_H - FRAME_FACE / 2.0))
            .box(groove_span, TRACK_GROOVE_W, FRAME_FACE + 0.010)
        )
        frame = frame.cut(head_groove)

    # Shallow side grooves in the jambs for sash stile retention.
    # Two grooves per jamb (inner and outer sash tracks; screen doesn't need
    # deep side grooves since it is fixed).
    groove_x = FRAME_FACE * 0.50
    for sign, edge_x in ((+1.0, OPEN_X0), (-1.0, OPEN_X1)):
        cx = edge_x - sign * groove_x / 2.0
        for track_y in (INNER_SASH_Y, OUTER_SASH_Y):
            jamb_groove = (
                cq.Workplane("XY")
                .transformed(offset=(cx, track_y, (OPEN_Z0 + OPEN_Z1) / 2.0))
                .box(groove_x, TRACK_GROOVE_W, OPEN_H)
            )
            frame = frame.cut(jamb_groove)

    return frame


# ---------------------------------------------------------------------------
# Sash frame geometry (CadQuery): perimeter ring + 6-lite muntin grid
# ---------------------------------------------------------------------------

def _build_sash_frame_shape() -> cq.Workplane:
    """One sash: perimeter ring plus a 3x2 muntin grid.

    Authored in sash-local frame:
      - local X runs -SASH_W/2 .. +SASH_W/2
      - local Z runs -SASH_H/2 .. +SASH_H/2 (centered vertically)
      - local Y is the sash thickness, centered at y=0.
    """
    w = SASH_W
    h = SASH_H
    r = SASH_RAIL
    d = SASH_DEPTH

    outer = (
        cq.Workplane("XY")
        .box(w, d, h)
    )

    # Inner glazed region
    in_x0, in_x1 = -w / 2.0 + r, w / 2.0 - r
    in_z0, in_z1 = -h / 2.0 + r, h / 2.0 - r
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
    in_z0, in_z1 = -h / 2.0 + r, h / 2.0 - r
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
    """Screen frame: a thin perimeter ring (no muntins) that holds the mesh.
    Local frame: centered at origin, X spans screen width, Z spans screen height.
    
    The frame is sized to exactly contact the jamb inner faces on the sides,
    and extends into the head/sill track grooves top and bottom for retention.
    """
    sw = OPEN_W          # screen width - contacts jamb inner faces
    sh = OPEN_H + 0.016  # screen height - extends into head/sill grooves for retention
    f = SCREEN_FRAME_W
    t = SCREEN_T

    outer = (
        cq.Workplane("XY")
        .box(sw, t, sh)
    )
    # Cut the center to leave a frame ring
    inner_cut = (
        cq.Workplane("XY")
        .box(sw - 2 * f, t + 0.02, sh - 2 * f)
    )
    return outer.cut(inner_cut)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_window")

    model.material("frame", rgba=FRAME_RGBA)
    model.material("sash", rgba=SASH_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("lock", rgba=LOCK_RGBA)
    model.material("screen_frame", rgba=SCREEN_FRAME_RGBA)
    model.material("screen_mesh", rgba=SCREEN_MESH_RGBA)
    model.material("latch", rgba=LATCH_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material="frame",
        name="frame_shell",
    )

    # --- Inner sash (left side, slides right to open) ---
    inner_sash = model.part("inner_sash")
    inner_sash.visual(
        mesh_from_cadquery(_build_sash_frame_shape(), "inner_sash_frame"),
        material="sash",
        name="inner_sash_frame",
    )
    inner_sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(), "inner_sash_glass"),
        material="glass",
        name="inner_sash_glass",
    )

    # --- Outer sash (right side, slides left to open) ---
    outer_sash = model.part("outer_sash")
    outer_sash.visual(
        mesh_from_cadquery(_build_sash_frame_shape(), "outer_sash_frame"),
        material="sash",
        name="outer_sash_frame",
    )
    outer_sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(), "outer_sash_glass"),
        material="glass",
        name="outer_sash_glass",
    )

    # --- Insect screen panel (fixed in exterior track) ---
    screen = model.part("insect_screen")
    screen.visual(
        mesh_from_cadquery(_build_screen_frame_shape(), "screen_frame"),
        material="screen_frame",
        name="screen_frame",
    )
    # Screen mesh: simple thin semi-transparent panel representing fine mesh
    screen_mesh_w = OPEN_W - 2 * SCREEN_FRAME_W
    screen_mesh_h = OPEN_H + 0.016 - 2 * SCREEN_FRAME_W
    screen.visual(
        Box((screen_mesh_w, SCREEN_MESH_T, screen_mesh_h)),
        material="screen_mesh",
        name="screen_mesh",
    )

    # --- Tilt-in latches (one per sash) ---
    # Inner sash latch: at the top rail of the inner sash, pivoting around X
    inner_latch = model.part("inner_latch")
    inner_latch.visual(
        Box(LATCH_LEVER),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="latch",
        name="inner_latch_lever",
    )
    # Small pivot pin visual (cylinder)
    inner_latch.visual(
        Cylinder(radius=0.004, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)),
        material="latch",
        name="inner_latch_pin",
    )

    # Outer sash latch
    outer_latch = model.part("outer_latch")
    outer_latch.visual(
        Box(LATCH_LEVER),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="latch",
        name="outer_latch_lever",
    )
    outer_latch.visual(
        Cylinder(radius=0.004, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)),
        material="latch",
        name="outer_latch_pin",
    )

    # --- Sash lock on inner sash meeting stile ---
    lock_x = SASH_W / 2.0 - SASH_RAIL / 2.0  # on the right stile of inner sash
    lock_y = -(SASH_DEPTH / 2.0 + LOCK_BODY[1] / 2.0 - 0.004)
    inner_sash.visual(
        Box(LOCK_BODY),
        origin=Origin(xyz=(lock_x, lock_y, 0.0)),
        material="lock",
        name="inner_sash_lock_body",
    )
    inner_sash.visual(
        Box(LOCK_LEVER),
        origin=Origin(xyz=(lock_x, lock_y - LOCK_BODY[1] / 2.0, 0.0)),
        material="lock",
        name="inner_sash_lock_lever",
    )

    # ----- Articulations -----

    # INNER sash: slides RIGHT (positive X). axis (1,0,0), positive q opens.
    # Joint origin at the closed-pose sash center.
    model.articulation(
        "frame_to_inner_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="inner_sash",
        origin=Origin(xyz=(INNER_SASH_CLOSED_X, INNER_SASH_Y, SASH_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=0.25, lower=0.0, upper=OPEN_W * 0.40
        ),
    )

    # OUTER sash: slides LEFT (negative X). axis (-1,0,0), positive q opens.
    model.articulation(
        "frame_to_outer_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="outer_sash",
        origin=Origin(xyz=(OUTER_SASH_CLOSED_X, OUTER_SASH_Y, SASH_CENTER_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=0.25, lower=0.0, upper=OPEN_W * 0.40
        ),
    )

    # Insect screen: FIXED in its own exterior track.
    model.articulation(
        "frame_to_screen",
        ArticulationType.FIXED,
        parent="frame",
        child="insect_screen",
        origin=Origin(xyz=(0.0, SCREEN_Y, SASH_CENTER_Z)),
    )

    # Inner sash tilt-in latch: REVOLUTE around X axis (pivots in Y-Z plane).
    # Mounted at the top rail of inner sash. Positive q tilts the latch inward.
    # The latch sits at top of the sash, on the interior face.
    latch_z = SASH_H / 2.0 - SASH_RAIL / 2.0  # top rail center
    latch_y = -(SASH_DEPTH / 2.0 + LATCH_LEVER[1] / 2.0)
    model.articulation(
        "inner_sash_to_latch",
        ArticulationType.REVOLUTE,
        parent="inner_sash",
        child="inner_latch",
        origin=Origin(xyz=(0.0, latch_y, latch_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=1.0, lower=0.0, upper=1.2
        ),
    )

    # Outer sash tilt-in latch: same concept, mounted on outer sash top rail.
    model.articulation(
        "outer_sash_to_latch",
        ArticulationType.REVOLUTE,
        parent="outer_sash",
        child="outer_latch",
        origin=Origin(xyz=(0.0, latch_y, latch_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=1.0, lower=0.0, upper=1.2
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    inner_sash = object_model.get_part("inner_sash")
    outer_sash = object_model.get_part("outer_sash")
    screen = object_model.get_part("insect_screen")
    inner_latch = object_model.get_part("inner_latch")
    outer_latch = object_model.get_part("outer_latch")

    j_inner = object_model.get_articulation("frame_to_inner_sash")
    j_outer = object_model.get_articulation("frame_to_outer_sash")
    j_inner_latch = object_model.get_articulation("inner_sash_to_latch")
    j_outer_latch = object_model.get_articulation("outer_sash_to_latch")

    # --- Intentional overlaps ---
    # Glass panes tuck under the sash muntin/rail lips (captured glass).
    for sash_name in ("inner_sash", "outer_sash"):
        ctx.allow_overlap(
            sash_name, sash_name,
            elem_a=f"{sash_name}_glass",
            elem_b=f"{sash_name}_frame",
            reason="Glass panes are rebated under the sash rails/muntins so they read as captured.",
        )

    # Sashes ride in the frame track grooves (retained insertion).
    ctx.allow_overlap(
        "frame", "inner_sash",
        reason="Inner sash rides in the interior track grooves of the frame head and sill.",
    )
    ctx.allow_overlap(
        "frame", "outer_sash",
        reason="Outer sash rides in the exterior track grooves of the frame head and sill.",
    )

    # Screen is fixed in its own track, nested in the frame.
    ctx.allow_overlap(
        "frame", "insect_screen",
        reason="Insect screen panel is fixed in its own exterior track groove.",
    )

    # The two sashes overlap at the meeting stile (different Y planes).
    ctx.allow_overlap(
        "inner_sash", "outer_sash",
        reason="Sashes overlap at the central meeting stile; they ride in offset Y track planes.",
    )

    # Sash lock seated on inner sash stile.
    ctx.allow_overlap(
        "inner_sash", "inner_sash",
        elem_a="inner_sash_lock_body",
        elem_b="inner_sash_frame",
        reason="Sash lock is mounted (seated) onto the inner sash meeting stile.",
    )

    # Latches are mounted onto sash rails (seated overlap).
    ctx.allow_overlap(
        "inner_sash", "inner_latch",
        reason="Inner tilt latch is mounted onto the inner sash top rail.",
    )
    ctx.allow_overlap(
        "outer_sash", "outer_latch",
        reason="Outer tilt latch is mounted onto the outer sash top rail.",
    )

    # Screen mesh overlaps screen frame (captured in frame).
    ctx.allow_overlap(
        "insect_screen", "insect_screen",
        elem_a="screen_mesh",
        elem_b="screen_frame",
        reason="Screen mesh is captured within the screen frame perimeter.",
    )

    # --- Closed pose (q=0): both sashes seated, window reads shut ---
    with ctx.pose({j_inner: 0.0, j_outer: 0.0}):
        f_aabb = ctx.part_world_aabb(frame)
        in_aabb = ctx.part_world_aabb(inner_sash)
        out_aabb = ctx.part_world_aabb(outer_sash)

        # Frame is the widest/tallest element.
        frame_w = f_aabb[1][0] - f_aabb[0][0]
        sash_w = in_aabb[1][0] - in_aabb[0][0]
        ctx.check(
            "frame spans wider than a sash",
            frame_w > sash_w + 0.05,
            details=f"frame_w={frame_w:.3f}, sash_w={sash_w:.3f}",
        )

        # Sill at/near z=0 (window stands vertically).
        ctx.check(
            "frame sill near z=0",
            abs(f_aabb[0][2]) < 0.01 and f_aabb[1][2] > 1.0,
            details=f"frame z range=({f_aabb[0][2]:.3f}, {f_aabb[1][2]:.3f})",
        )

        # Sashes are inside the frame opening width.
        ctx.check(
            "sashes within frame width",
            in_aabb[0][0] > f_aabb[0][0] and in_aabb[1][0] < f_aabb[1][0]
            and out_aabb[0][0] > f_aabb[0][0] and out_aabb[1][0] < f_aabb[1][0],
            details=f"inner x=({in_aabb[0][0]:.3f},{in_aabb[1][0]:.3f}) outer x=({out_aabb[0][0]:.3f},{out_aabb[1][0]:.3f})",
        )

        # Inner sash is to the LEFT of the outer sash (side-by-side).
        in_cx = (in_aabb[0][0] + in_aabb[1][0]) / 2.0
        out_cx = (out_aabb[0][0] + out_aabb[1][0]) / 2.0
        ctx.check(
            "inner sash left of outer sash at closed pose",
            in_cx < out_cx - 0.1,
            details=f"inner_cx={in_cx:.3f}, outer_cx={out_cx:.3f}",
        )

        # Sashes overlap at the meeting stile in X (shut, no daylight gap).
        ctx.check(
            "sashes overlap at meeting stile (shut)",
            in_aabb[1][0] >= out_aabb[0][0] - 1e-4,
            details=f"inner_right={in_aabb[1][0]:.3f}, outer_left={out_aabb[0][0]:.3f}",
        )

        # Sashes sit in offset Y planes so they pass each other.
        in_cy = (in_aabb[0][1] + in_aabb[1][1]) / 2.0
        out_cy = (out_aabb[0][1] + out_aabb[1][1]) / 2.0
        ctx.check(
            "sashes ride in offset Y track planes",
            abs(in_cy - out_cy) > 0.015,
            details=f"inner_cy={in_cy:.3f}, outer_cy={out_cy:.3f}",
        )

        rest_in_x = in_cx
        rest_out_x = out_cx

    # --- Insect screen exists in its own track (exterior side) ---
    scr_aabb = ctx.part_world_aabb(screen)
    ctx.check(
        "insect screen panel exists",
        scr_aabb is not None and (scr_aabb[1][0] - scr_aabb[0][0]) > 0.3,
        details=f"screen width={scr_aabb[1][0] - scr_aabb[0][0]:.3f}" if scr_aabb else "no screen",
    )
    # Screen is on the exterior side (+Y) of both sashes.
    scr_cy = (scr_aabb[0][1] + scr_aabb[1][1]) / 2.0
    ctx.check(
        "screen is in exterior track (positive Y of outer sash)",
        scr_cy > out_cy + 0.010,
        details=f"screen_cy={scr_cy:.3f}, outer_cy={out_cy:.3f}",
    )

    # --- HERO: inner sash slides RIGHT (opens) ---
    travel = OPEN_W * 0.38
    with ctx.pose({j_inner: travel}):
        op = ctx.part_world_aabb(inner_sash)
        op_cx = (op[0][0] + op[1][0]) / 2.0
        ctx.check(
            "inner sash slides right when opened",
            op_cx > rest_in_x + travel * 0.8,
            details=f"rest_cx={rest_in_x:.3f}, opened_cx={op_cx:.3f}, travel={travel:.3f}",
        )
        # Stays retained: still overlaps the frame in Z footprint.
        ctx.expect_overlap(
            inner_sash, frame, axes="z", min_overlap=0.10,
            name="inner sash retained in frame when open",
        )

    # --- HERO: outer sash slides LEFT (opens) ---
    with ctx.pose({j_outer: travel}):
        op = ctx.part_world_aabb(outer_sash)
        op_cx = (op[0][0] + op[1][0]) / 2.0
        ctx.check(
            "outer sash slides left when opened",
            op_cx < rest_out_x - travel * 0.8,
            details=f"rest_cx={rest_out_x:.3f}, opened_cx={op_cx:.3f}, travel={travel:.3f}",
        )
        ctx.expect_overlap(
            outer_sash, frame, axes="z", min_overlap=0.10,
            name="outer sash retained in frame when open",
        )

    # --- Tilt latches pivot on revolute joints ---
    # At rest (q=0), latches are in the locked (horizontal) position.
    # At q=1.0, latches tilt inward (rotated).
    with ctx.pose({j_inner_latch: 0.0}):
        latch_rest = ctx.part_world_aabb(inner_latch)
        rest_z_span = latch_rest[1][2] - latch_rest[0][2]

    with ctx.pose({j_inner_latch: 1.0}):
        latch_tilted = ctx.part_world_aabb(inner_latch)
        tilted_z_span = latch_tilted[1][2] - latch_tilted[0][2]
        tilted_y_span = latch_tilted[1][1] - latch_tilted[0][1]

    ctx.check(
        "inner latch tilts when pivoted (Z span changes)",
        abs(tilted_z_span - rest_z_span) > 0.005 or tilted_y_span > 0.010,
        details=f"rest_z_span={rest_z_span:.4f}, tilted_z_span={tilted_z_span:.4f}",
    )

    # Verify both latch joints are revolute with proper limits.
    ctx.check(
        "inner latch joint is revolute",
        j_inner_latch.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={j_inner_latch.articulation_type}",
    )
    ctx.check(
        "outer latch joint is revolute",
        j_outer_latch.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={j_outer_latch.articulation_type}",
    )

    # --- Deep track grooves exist: frame has more depth than a simple slab ---
    # The frame depth should accommodate 3 tracks.
    f_depth = f_aabb[1][1] - f_aabb[0][1]
    ctx.check(
        "frame depth accommodates three tracks",
        f_depth > 0.10,
        details=f"frame depth={f_depth:.3f}",
    )

    # --- Sash lock sits on inner sash meeting stile ---
    lock_aabb = ctx.part_element_world_aabb(inner_sash, elem="inner_sash_lock_body")
    if lock_aabb is not None:
        lock_cx = (lock_aabb[0][0] + lock_aabb[1][0]) / 2.0
        # Lock should be near the right edge (meeting stile) of the inner sash
        ctx.check(
            "sash lock on the meeting stile side",
            lock_cx > -0.05,
            details=f"lock world X center={lock_cx:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
