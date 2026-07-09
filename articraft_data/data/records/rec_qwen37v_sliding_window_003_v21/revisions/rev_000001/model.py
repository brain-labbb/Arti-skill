from __future__ import annotations

# Horizontal sliding window: white frame with two side-by-side six-lite sashes
# that slide horizontally in opposite directions on top/bottom track grooves.
#
# Variant of the double-hung sash window, forked into a horizontal slider.
# Reference: Sliding window picture family (003.png).
#
# Coordinate convention:
#   +Z is up. Window stands vertically: height along +Z, width along X, frame
#   depth / glazing thickness along Y (the glass plane is the X-Z plane). The
#   sill sits at z=0; the head is at z=WIN_H.
#
# Articulation (horizontal slider):
#   - LEFT sash is PRISMATIC, axis (1,0,0): positive q slides it RIGHT (opens).
#   - RIGHT sash is PRISMATIC, axis (-1,0,0): positive q slides it LEFT (opens).
#   Both sashes stay retained in the top/bottom tracks at full travel. The left
#   sash rides in the interior (-Y) track plane; the right sash rides in the
#   exterior (+Y) track plane, so they pass each other at the meeting stile.

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

WIN_W = 1.20          # overall window width (X) -- wider than tall for a slider
WIN_H = 0.92          # overall window height (Z), sill at z=0
FRAME_FACE = 0.060    # outer frame member face width (X/Z)
FRAME_DEPTH = 0.110   # outer frame jamb depth (Y)

# Clear opening inside the outer frame
OPEN_W = WIN_W - 2 * FRAME_FACE   # clear width
OPEN_H = WIN_H - 2 * FRAME_FACE   # clear height
OPEN_X0 = -WIN_W / 2.0 + FRAME_FACE
OPEN_X1 = WIN_W / 2.0 - FRAME_FACE
OPEN_Z0 = FRAME_FACE
OPEN_Z1 = WIN_H - FRAME_FACE

# Sash geometry. Each sash is a bit over half the clear width (they overlap at
# the meeting stile in the center) and spans nearly the full clear height.
SASH_OVERLAP = 0.030               # overlap at the meeting stile
SASH_W = OPEN_W / 2.0 + SASH_OVERLAP   # each sash width
SASH_RAIL = 0.048                  # sash perimeter member width (stile/rail)
SASH_DEPTH = 0.034                 # sash thickness (Y)
SASH_H = OPEN_H - 0.008           # sash height (small top/bottom clearance)
GLASS_T = 0.006                    # glazing thickness (Y)

# Y planes: left sash rides interior (-Y), right sash rides exterior (+Y),
# offset from the frame depth center so they clear each other at the meeting stile.
SASH_Y_GAP = 0.018                 # half the gap between the two sash planes
LEFT_SASH_Y = -SASH_Y_GAP
RIGHT_SASH_Y = +SASH_Y_GAP

# Closed-pose sash center X positions (world X). They overlap at the center.
LEFT_CLOSED_X = -(OPEN_W / 4.0)           # left sash center when closed
RIGHT_CLOSED_X = +(OPEN_W / 4.0)          # right sash center when closed

# Sash bottom edge Z: sits just above the sill track groove.
SASH_BOTTOM_Z = OPEN_Z0 + 0.004

# Muntin grid: 3 columns x 2 rows of lites per sash -> 2 vertical + 1 horizontal bar.
MUNTIN_W = 0.020
N_COLS = 3
N_ROWS = 2

# Track grooves: deep channels in the top and bottom frame rails where the
# sash edges ride. Two grooves per rail (one per sash track), offset in Y.
TRACK_DEPTH_Z = 0.025              # how deep the groove cuts into the rail (Z)
TRACK_WIDTH_Y = SASH_DEPTH + 0.006 # groove width in Y (sash + clearance)

# Sash lock at the meeting stile (on left sash).
LOCK_BODY = (0.026, 0.022, 0.050)   # (X, Y, Z)
LOCK_LEVER = (0.012, 0.010, 0.040)

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

FRAME_RGBA = (0.945, 0.945, 0.945, 1.0)   # white painted frame
SASH_RGBA = (0.965, 0.965, 0.965, 1.0)    # white sash (very slightly brighter)
GLASS_RGBA = (0.30, 0.36, 0.42, 0.34)     # cool dark-tinted glass
LOCK_RGBA = (0.86, 0.87, 0.89, 1.0)       # brushed metal sash lock


# ---------------------------------------------------------------------------
# Static outer frame geometry (CadQuery)
# ---------------------------------------------------------------------------

def _build_frame_shape() -> cq.Workplane:
    """White outer frame: a perimeter slab with the central opening cut out,
    plus deep track grooves in the top and bottom rails.

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

    # Deep track grooves in the top and bottom rails. Two grooves per rail
    # (one per sash track), offset in Y so each sash has its own channel.
    # The grooves run the full opening width along X.
    groove_len_x = OPEN_W + 0.01  # slightly wider than opening for clean cut

    for track_y in (LEFT_SASH_Y, RIGHT_SASH_Y):
        # Bottom rail groove: cuts upward from the sill into the bottom rail.
        bottom_groove = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, track_y, OPEN_Z0 - TRACK_DEPTH_Z / 2.0))
            .box(groove_len_x, TRACK_WIDTH_Y, TRACK_DEPTH_Z)
        )
        frame = frame.cut(bottom_groove)

        # Top rail groove: cuts downward from the head into the top rail.
        top_groove = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, track_y, OPEN_Z1 + TRACK_DEPTH_Z / 2.0))
            .box(groove_len_x, TRACK_WIDTH_Y, TRACK_DEPTH_Z)
        )
        frame = frame.cut(top_groove)

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
    model = ArticulatedObject(name="horizontal_sliding_window")

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
    _add_sash(model, "left_sash")
    _add_sash(model, "right_sash")

    # Sash lock on the left sash right (meeting) stile, near center height.
    left = model.get_part("left_sash")
    lock_x = SASH_W / 2.0 - SASH_RAIL / 2.0  # on the right stile of left sash
    lock_z = SASH_H / 2.0                     # mid-height
    # Mount on the interior face (-Y) of the left sash meeting stile.
    lock_body_y = -(SASH_DEPTH / 2.0 + LOCK_BODY[1] / 2.0 - 0.004)
    left.visual(
        Box(LOCK_BODY),
        origin=Origin(xyz=(lock_x, lock_body_y, lock_z)),
        material="lock",
        name="left_sash_lock_body",
    )
    left.visual(
        Box(LOCK_LEVER),
        origin=Origin(xyz=(lock_x, lock_body_y - LOCK_BODY[1] / 2.0, lock_z)),
        material="lock",
        name="left_sash_lock_lever",
    )

    # ----- Articulations (horizontal slider) -----
    # Both sashes are authored with their bottom rail at local z=0 and centered
    # in X. The joint origin is placed at each sash's closed (seated) world
    # position so q=0 reads as a shut window.

    # LEFT sash: slides RIGHT. axis (1,0,0), positive q opens to the right.
    model.articulation(
        "frame_to_left_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="left_sash",
        origin=Origin(xyz=(LEFT_CLOSED_X, LEFT_SASH_Y, SASH_BOTTOM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=0.25, lower=0.0, upper=SASH_W * 0.70
        ),
    )

    # RIGHT sash: slides LEFT. axis (-1,0,0), positive q opens to the left.
    model.articulation(
        "frame_to_right_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="right_sash",
        origin=Origin(xyz=(RIGHT_CLOSED_X, RIGHT_SASH_Y, SASH_BOTTOM_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=0.25, lower=0.0, upper=SASH_W * 0.70
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    left = object_model.get_part("left_sash")
    right = object_model.get_part("right_sash")
    j_left = object_model.get_articulation("frame_to_left_sash")
    j_right = object_model.get_articulation("frame_to_right_sash")

    # --- Verify articulation types and axes ---
    ctx.check(
        "left sash joint is prismatic",
        j_left.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={j_left.articulation_type}",
    )
    ctx.check(
        "right sash joint is prismatic",
        j_right.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={j_right.articulation_type}",
    )
    ctx.check(
        "left sash slides along X axis",
        abs(j_left.axis[0]) > 0.9 and abs(j_left.axis[2]) < 0.1,
        details=f"axis={j_left.axis}",
    )
    ctx.check(
        "right sash slides along X axis",
        abs(j_right.axis[0]) > 0.9 and abs(j_right.axis[2]) < 0.1,
        details=f"axis={j_right.axis}",
    )
    ctx.check(
        "sashes slide in opposite directions",
        j_left.axis[0] * j_right.axis[0] < 0,
        details=f"left_axis={j_left.axis}, right_axis={j_right.axis}",
    )

    # --- Intentional overlaps ---
    # Glass panes tuck under the sash muntin/rail lips (captured glass).
    for sash_name in ("left_sash", "right_sash"):
        ctx.allow_overlap(
            sash_name, sash_name,
            elem_a=f"{sash_name}_glass",
            elem_b=f"{sash_name}_frame",
            reason="Glass panes are rebated under the sash rails/muntins so they read as captured, not floating.",
        )
    # Each sash rides in the top/bottom track grooves cut into the frame.
    ctx.allow_overlap(
        "frame", "left_sash",
        reason="Left sash rides in the interior top/bottom track grooves (retained insertion).",
    )
    ctx.allow_overlap(
        "frame", "right_sash",
        reason="Right sash rides in the exterior top/bottom track grooves (retained insertion).",
    )
    # The two sashes overlap at the meeting stile (different Y planes).
    ctx.allow_overlap(
        "left_sash", "right_sash",
        reason="Sashes overlap at the central meeting stile; they ride in offset Y planes.",
    )
    # Sash lock body is seated onto the left sash meeting stile.
    ctx.allow_overlap(
        "left_sash", "left_sash",
        elem_a="left_sash_lock_body",
        elem_b="left_sash_frame",
        reason="Sash lock is mounted (seated) onto the left sash meeting stile.",
    )

    # --- Closed pose (q=0): both sashes seated, window reads shut ---
    with ctx.pose({j_left: 0.0, j_right: 0.0}):
        f_aabb = ctx.part_world_aabb(frame)
        lo_aabb = ctx.part_world_aabb(left)
        ri_aabb = ctx.part_world_aabb(right)

        # Frame is the widest element and spans wider than a single sash.
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
            abs(f_aabb[0][2]) < 0.01 and f_aabb[1][2] > 0.5,
            details=f"frame z range=({f_aabb[0][2]:.3f}, {f_aabb[1][2]:.3f})",
        )
        # Window is wider than tall (horizontal slider proportions).
        frame_h = f_aabb[1][2] - f_aabb[0][2]
        ctx.check(
            "window wider than tall (slider proportions)",
            frame_w > frame_h + 0.1,
            details=f"frame_w={frame_w:.3f}, frame_h={frame_h:.3f}",
        )
        # Sashes are inside the frame opening.
        ctx.check(
            "sashes within frame width",
            lo_aabb[0][0] > f_aabb[0][0] and lo_aabb[1][0] < f_aabb[1][0]
            and ri_aabb[0][0] > f_aabb[0][0] and ri_aabb[1][0] < f_aabb[1][0],
            details=f"left x=({lo_aabb[0][0]:.3f},{lo_aabb[1][0]:.3f}) right x=({ri_aabb[0][0]:.3f},{ri_aabb[1][0]:.3f})",
        )
        # Left sash is to the left of the right sash (side by side).
        lo_cx = (lo_aabb[0][0] + lo_aabb[1][0]) / 2.0
        ri_cx = (ri_aabb[0][0] + ri_aabb[1][0]) / 2.0
        ctx.check(
            "left sash is left of right sash at closed pose",
            lo_cx < ri_cx - 0.1,
            details=f"left_cx={lo_cx:.3f}, right_cx={ri_cx:.3f}",
        )
        # Sashes sit at roughly the same height (side by side, not stacked).
        lo_cz = (lo_aabb[0][2] + lo_aabb[1][2]) / 2.0
        ri_cz = (ri_aabb[0][2] + ri_aabb[1][2]) / 2.0
        ctx.check(
            "sashes at similar height (side by side, not stacked)",
            abs(lo_cz - ri_cz) < 0.05,
            details=f"left_cz={lo_cz:.3f}, right_cz={ri_cz:.3f}",
        )
        # The two sashes overlap at the meeting stile in X (shut, no daylight gap).
        ctx.check(
            "sashes overlap at meeting stile (shut)",
            lo_aabb[1][0] >= ri_aabb[0][0] - 1e-4,
            details=f"left_right_edge={lo_aabb[1][0]:.3f}, right_left_edge={ri_aabb[0][0]:.3f}",
        )
        # Sashes sit in offset Y planes so they pass each other.
        lo_cy = (lo_aabb[0][1] + lo_aabb[1][1]) / 2.0
        ri_cy = (ri_aabb[0][1] + ri_aabb[1][1]) / 2.0
        ctx.check(
            "sashes ride in offset Y planes",
            abs(lo_cy - ri_cy) > 0.015,
            details=f"left_cy={lo_cy:.3f}, right_cy={ri_cy:.3f}",
        )

        rest_lo_cx = lo_cx
        rest_ri_cx = ri_cx

    # --- HERO: left sash slides RIGHT (opens) ---
    travel = SASH_W * 0.55
    with ctx.pose({j_left: travel}):
        op = ctx.part_world_aabb(left)
        op_cx = (op[0][0] + op[1][0]) / 2.0
        ctx.check(
            "left sash slides right when opened",
            op_cx > rest_lo_cx + travel * 0.8,
            details=f"rest_cx={rest_lo_cx:.3f}, opened_cx={op_cx:.3f}, travel={travel:.3f}",
        )
        # Stays retained: still overlaps the frame in Z footprint.
        ctx.expect_overlap(
            left, frame, axes="z", min_overlap=0.05,
            name="left sash retained in frame tracks when open",
        )

    # --- HERO: right sash slides LEFT (opens) ---
    with ctx.pose({j_right: travel}):
        op = ctx.part_world_aabb(right)
        op_cx = (op[0][0] + op[1][0]) / 2.0
        ctx.check(
            "right sash slides left when opened",
            op_cx < rest_ri_cx - travel * 0.8,
            details=f"rest_cx={rest_ri_cx:.3f}, opened_cx={op_cx:.3f}, travel={travel:.3f}",
        )
        ctx.expect_overlap(
            right, frame, axes="z", min_overlap=0.05,
            name="right sash retained in frame tracks when open",
        )

    # --- Both open: sashes separate from center, opening the window ---
    with ctx.pose({j_left: travel, j_right: travel}):
        lo = ctx.part_world_aabb(left)
        ri = ctx.part_world_aabb(right)
        # Left sash has moved right and right sash has moved left from closed.
        lo_new_cx = (lo[0][0] + lo[1][0]) / 2.0
        ri_new_cx = (ri[0][0] + ri[1][0]) / 2.0
        ctx.check(
            "both sashes moved from closed positions when both open",
            lo_new_cx > rest_lo_cx + travel * 0.7
            and ri_new_cx < rest_ri_cx - travel * 0.7,
            details=f"left {rest_lo_cx:.3f}->{lo_new_cx:.3f}, right {rest_ri_cx:.3f}->{ri_new_cx:.3f}",
        )

    # --- Sash lock sits on the meeting stile, near center height ---
    lock_aabb = ctx.part_element_world_aabb(left, elem="left_sash_lock_body")
    if lock_aabb is not None:
        lock_cz = (lock_aabb[0][2] + lock_aabb[1][2]) / 2.0
        frame_cz = (f_aabb[0][2] + f_aabb[1][2]) / 2.0
        ctx.check(
            "sash lock near window center height",
            abs(lock_cz - frame_cz) < 0.15,
            details=f"lock world Z center={lock_cz:.3f}, frame center={frame_cz:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
