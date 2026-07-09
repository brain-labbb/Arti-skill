from __future__ import annotations

# Sliding sash window: white frame with two stacked six-lite sashes. The upper
# sash is fixed; the lower sash slides vertically on a prismatic joint. Two
# small roller blocks at the bottom of the moving sash ride in the sill track.
#
# Variant 30 fork of the double-hung sash window. The window is partially open
# at rest so the sash overlap at the meeting rail is visible.
#
# Coordinate convention:
#   +Z is up. Window stands vertically: height along +Z, width along X, frame
#   depth / glazing thickness along Y (the glass plane is the X-Z plane). The
#   sill sits at z=0; the head is at z=WIN_H.
#
# Articulation:
#   - LOWER sash is PRISMATIC, axis (0,0,1): positive q slides it UP (opens).
#     Rest pose is partially open (origin shifted up by PARTIAL_OPEN).
#   - UPPER sash is FIXED (stationary pane).
#   The lower sash rides in the interior (-Y) track plane; the upper sash sits
#   in the exterior (+Y) track plane.

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
OPEN_W = WIN_W - 2 * FRAME_FACE   # clear width
OPEN_H = WIN_H - 2 * FRAME_FACE   # clear height
OPEN_X0 = -WIN_W / 2.0 + FRAME_FACE
OPEN_X1 = WIN_W / 2.0 - FRAME_FACE
OPEN_Z0 = FRAME_FACE
OPEN_Z1 = WIN_H - FRAME_FACE

# Sash geometry. Each sash is the same width and a bit over half the clear
# height; they overlap at the meeting rail in the middle.
SASH_W = OPEN_W - 0.010                 # slight running clearance to the jambs
SASH_RAIL = 0.052                       # sash perimeter member width (stile/rail)
SASH_DEPTH = 0.034                      # sash thickness (Y)
SASH_H = OPEN_H * 0.545                 # each sash height (overlap at center)
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

# Partial open amount at rest (lower sash slides up by this amount).
PARTIAL_OPEN = SASH_H * 0.18            # about 18% open so overlap is visible

# Roller blocks at the bottom of the moving sash.
ROLLER_SIZE = (0.024, 0.018, 0.012)     # (X, Y, Z) small block
ROLLER_INSET = 0.060                    # inset from each sash stile edge

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

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

FRAME_RGBA = (0.945, 0.945, 0.945, 1.0)   # white painted frame
SASH_RGBA = (0.965, 0.965, 0.965, 1.0)    # white sash (very slightly brighter)
GLASS_RGBA = (0.30, 0.36, 0.42, 0.34)     # cool dark-tinted glass (as in photo)
LOCK_RGBA = (0.86, 0.87, 0.89, 1.0)       # brushed metal sash lock


# ---------------------------------------------------------------------------
# Static outer frame geometry (CadQuery)
# ---------------------------------------------------------------------------

def _build_frame_shape() -> cq.Workplane:
    """White outer frame: a perimeter slab with the central opening cut out,
    plus two shallow side-track channels in the jambs.

    World frame: opening centered on X=0, Z from 0 (sill) to WIN_H (head).
    """
    # Solid outer slab spanning the full window footprint.
    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, WIN_H / 2.0))
        .box(WIN_W, FRAME_DEPTH, WIN_H)
    )

    # Cut the clear central opening (leaves head, sill, two jambs).
    opening = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, (OPEN_Z0 + OPEN_Z1) / 2.0))
        .box(OPEN_W, FRAME_DEPTH + 0.02, OPEN_H)
    )
    frame = outer.cut(opening)

    # Two side-track channels per jamb: shallow grooves notched into the
    # opening-facing inner edge of each jamb where the sash stiles ride. The
    # groove is anchored at the opening edge and cuts only partway into the
    # jamb so the jamb (and the whole perimeter frame) stays one connected
    # solid. Two grooves per jamb (one per sash plane).
    groove_x = FRAME_FACE * 0.55          # how far the groove reaches into the jamb (X)
    for sign, edge_x in ((+1.0, OPEN_X0), (-1.0, OPEN_X1)):
        # edge_x is the opening edge of this jamb; groove reaches outward into
        # the jamb by groove_x. Center the cut box so its opening-side face is
        # at the opening edge and it extends into the jamb material.
        cx = edge_x - sign * groove_x / 2.0
        for track_y in (LOWER_SASH_Y, UPPER_SASH_Y):
            groove = (
                cq.Workplane("XY")
                .transformed(offset=(cx, track_y, (OPEN_Z0 + OPEN_Z1) / 2.0))
                .box(groove_x, TRACK_DEPTH, OPEN_H)
            )
            frame = frame.cut(groove)

    return frame


# ---------------------------------------------------------------------------
# Sash frame geometry (CadQuery): perimeter ring + 6-lite muntin grid
# ---------------------------------------------------------------------------

def _build_sash_frame_shape() -> cq.Workplane:
    """One sash: perimeter ring plus a 3x2 muntin grid, built as a slab with
    six rectangular lite openings cut, leaving a true muntin lattice.

    Authored in the sash-local frame:
      - local X runs -SASH_W/2 .. +SASH_W/2
      - local Z runs 0 .. SASH_H (bottom rail at z=0)
      - local Y is the sash thickness, centered at y=0.
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

    # Inner glazed region (inside the perimeter rails).
    in_x0, in_x1 = -w / 2.0 + r, w / 2.0 - r
    in_z0, in_z1 = r, h - r
    inner_w = in_x1 - in_x0
    inner_h = in_z1 - in_z0

    # Column / row boundaries of the lite grid (muntin centerlines).
    col_lines = [in_x0 + (i + 1) * inner_w / N_COLS for i in range(N_COLS - 1)]
    row_lines = [in_z0 + (j + 1) * inner_h / N_ROWS for j in range(N_ROWS - 1)]

    # Build each lite opening rectangle and cut it.
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
    muntin/rail lips so the glass reads as captured, not floating."""
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
    model = ArticulatedObject(name="double_hung_sash_window")

    model.material("frame", rgba=FRAME_RGBA)
    model.material("sash", rgba=SASH_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("lock", rgba=LOCK_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material="frame",
        name="frame_shell",
    )

    # --- Two sashes ---
    _add_sash(model, "lower_sash")
    _add_sash(model, "upper_sash")

    # Sash lock on the lower sash top (meeting) rail, center.
    lower = model.get_part("lower_sash")
    lock_z = SASH_H - SASH_RAIL / 2.0   # on the top rail of the lower sash
    # Mount it on the interior face (-Y) of the lower sash top rail.
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

    # Two small roller blocks at the bottom of the lower sash (moving sash).
    # They ride in the sill track and support the sash as it slides.
    roller_z = ROLLER_SIZE[2] / 2.0     # rollers sit on the bottom rail
    roller_y = 0.0                       # centered in sash depth
    for i, sign in enumerate([+1.0, -1.0]):
        roller_x = sign * (SASH_W / 2.0 - ROLLER_INSET - ROLLER_SIZE[0] / 2.0)
        lower.visual(
            Box(ROLLER_SIZE),
            origin=Origin(xyz=(roller_x, roller_y, roller_z)),
            material="lock",  # same brushed metal as the lock
            name=f"lower_sash_roller_{i}",
        )

    # ----- Articulations (sliding window) -----
    # LOWER sash: slides UP. axis (0,0,1), positive q opens further upward.
    # Origin is at the partially-open rest pose (LOWER_BOTTOM_Z + PARTIAL_OPEN)
    # so the overlap at the meeting rail is visible at q=0.
    model.articulation(
        "frame_to_lower_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="lower_sash",
        origin=Origin(xyz=(0.0, LOWER_SASH_Y, LOWER_BOTTOM_Z + PARTIAL_OPEN)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=0.25, lower=0.0, upper=SASH_H * 0.42 - PARTIAL_OPEN
        ),
    )

    # UPPER sash: FIXED (stationary pane in a sliding window).
    model.articulation(
        "frame_to_upper_sash",
        ArticulationType.FIXED,
        parent="frame",
        child="upper_sash",
        origin=Origin(xyz=(0.0, UPPER_SASH_Y, UPPER_BOTTOM_Z)),
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

    # --- Intentional overlaps ---
    # Glass panes tuck under the sash muntin/rail lips (captured glass).
    for sash_name in ("lower_sash", "upper_sash"):
        ctx.allow_overlap(
            sash_name, sash_name,
            elem_a=f"{sash_name}_glass",
            elem_b=f"{sash_name}_frame",
            reason="Glass panes are rebated under the sash rails/muntins so they read as captured, not floating.",
        )
    # Lower sash rides in the jamb side-track grooves cut into the frame.
    ctx.allow_overlap(
        "frame", "lower_sash",
        reason="Lower sash stiles ride in the interior jamb track grooves (retained insertion).",
    )
    # Upper sash is fixed in the exterior jamb track.
    ctx.allow_overlap(
        "frame", "upper_sash",
        reason="Upper sash is fixed in the exterior jamb track grooves (stationary pane).",
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
    # Roller blocks are mounted on the lower sash bottom rail.
    for roller_name in ("lower_sash_roller_0", "lower_sash_roller_1"):
        ctx.allow_overlap(
            "lower_sash", "lower_sash",
            elem_a=roller_name,
            elem_b="lower_sash_frame",
            reason="Roller blocks are mounted on the lower sash bottom rail to support sliding.",
        )

    # --- Rest pose (q=0): lower sash is partially open ---
    with ctx.pose({j_lower: 0.0}):
        f_aabb = ctx.part_world_aabb(frame)
        lo_aabb = ctx.part_world_aabb(lower)
        up_aabb = ctx.part_world_aabb(upper)

        # Frame is the widest/tallest element and spans wider than a single sash.
        frame_w = f_aabb[1][0] - f_aabb[0][0]
        sash_w = lo_aabb[1][0] - lo_aabb[0][0]
        ctx.check(
            "frame spans wider than a sash",
            frame_w > sash_w + 0.05,
            details=f"frame_w={frame_w:.3f}, sash_w={sash_w:.3f}",
        )
        # Sill at/near z=0 (window stands on the ground, not sunk/flat).
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
        # Lower sash sits below the upper sash (stacked, overlapping near center).
        lo_center_z = (lo_aabb[0][2] + lo_aabb[1][2]) / 2.0
        up_center_z = (up_aabb[0][2] + up_aabb[1][2]) / 2.0
        ctx.check(
            "lower sash below upper sash at rest pose",
            lo_center_z < up_center_z - 0.3,
            details=f"lower_cz={lo_center_z:.3f}, upper_cz={up_center_z:.3f}",
        )
        # The two sashes overlap at the meeting rail in Z (partially open, overlap visible).
        ctx.check(
            "sashes overlap at meeting rail (partially open)",
            lo_aabb[1][2] >= up_aabb[0][2] - 1e-4,
            details=f"lower_top={lo_aabb[1][2]:.3f}, upper_bottom={up_aabb[0][2]:.3f}",
        )
        # Sashes sit in offset Y planes so they pass each other.
        lo_cy = (lo_aabb[0][1] + lo_aabb[1][1]) / 2.0
        up_cy = (up_aabb[0][1] + up_aabb[1][1]) / 2.0
        ctx.check(
            "sashes ride in offset Y planes",
            abs(lo_cy - up_cy) > 0.015,
            details=f"lower_cy={lo_cy:.3f}, upper_cy={up_cy:.3f}",
        )

        # Lower sash is partially open at rest: its bottom is above the sill.
        ctx.check(
            "lower sash partially open at rest",
            lo_aabb[0][2] > OPEN_Z0 + PARTIAL_OPEN * 0.8,
            details=f"lower_bottom={lo_aabb[0][2]:.3f}, expected>{OPEN_Z0 + PARTIAL_OPEN * 0.8:.3f}",
        )

        rest_lo_z = lo_center_z
        rest_lo_top = lo_aabb[1][2]

    # --- HERO: lower sash slides UP further (opens more) ---
    travel = SASH_H * 0.40 - PARTIAL_OPEN  # remaining travel from partial open
    with ctx.pose({j_lower: travel}):
        op = ctx.part_world_aabb(lower)
        op_cz = (op[0][2] + op[1][2]) / 2.0
        ctx.check(
            "lower sash slides up further when opened",
            op_cz > rest_lo_z + travel * 0.8,
            details=f"rest_cz={rest_lo_z:.3f}, opened_cz={op_cz:.3f}, travel={travel:.3f}",
        )
        # Stays retained: still overlaps the frame jambs in X footprint.
        ctx.expect_overlap(
            lower, frame, axes="x", min_overlap=0.05,
            name="lower sash retained in frame when open",
        )

    # --- Roller blocks exist on the lower sash ---
    for roller_name in ("lower_sash_roller_0", "lower_sash_roller_1"):
        roller_aabb = ctx.part_element_world_aabb(lower, elem=roller_name)
        ctx.check(
            f"{roller_name} exists",
            roller_aabb is not None,
            details=f"roller {roller_name} not found on lower sash",
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
