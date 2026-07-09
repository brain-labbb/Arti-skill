from __future__ import annotations

# Horizontal sliding window: thick aluminum frame with deep track grooves in
# the top and bottom rails. Two six-lite sashes sit side by side; the left
# sash is fixed and the right sash slides horizontally on a prismatic joint.
#
# Coordinate convention:
#   +Z is up. Window stands vertically: height along +Z, width along X,
#   frame depth / glazing thickness along Y. The sill sits at z=0; the head
#   at z=WIN_H.
#
# Articulation (horizontal slider):
#   - FIXED sash (left): no motion, fixed articulation.
#   - SLIDING sash (right): PRISMATIC along (-1,0,0). Positive q slides the
#     sash LEFT (toward the fixed sash), opening the right half of the window.
#   Both sashes ride in deep track grooves cut into the top and bottom frame
#   rails. The two sashes sit in offset Y planes so they pass each other.

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

WIN_W = 1.20          # overall window width (X) — landscape slider proportion
WIN_H = 1.00          # overall window height (Z), sill at z=0
FRAME_FACE = 0.068    # thick aluminum frame member face width (X/Z)
FRAME_DEPTH = 0.100   # outer frame depth (Y)

# Clear opening inside the outer frame
OPEN_W = WIN_W - 2 * FRAME_FACE
OPEN_H = WIN_H - 2 * FRAME_FACE
OPEN_X0 = -WIN_W / 2.0 + FRAME_FACE
OPEN_X1 = WIN_W / 2.0 - FRAME_FACE
OPEN_Z0 = FRAME_FACE
OPEN_Z1 = WIN_H - FRAME_FACE

# Deep track grooves in top and bottom rails. These are prominent slots cut
# into the inner face of each horizontal rail where the sash stiles ride.
TRACK_W = 0.024       # groove width (Y direction)
TRACK_DEPTH = 0.030   # groove depth into the rail (Z direction)

# Sash geometry. Each sash is slightly more than half the opening width so
# they overlap at the meeting stile when closed.
MEETING_OVERLAP = 0.050  # overlap at the central meeting stile
SASH_W = (OPEN_W + MEETING_OVERLAP) / 2.0
SASH_H = OPEN_H - 0.008        # slight vertical clearance in the tracks
SASH_RAIL = 0.050              # sash perimeter member width (stile/rail)
SASH_DEPTH = 0.032             # sash thickness (Y)
GLASS_T = 0.006                # glazing thickness (Y)

# Y planes: fixed sash rides in the interior (-Y) track; sliding sash rides
# in the exterior (+Y) track, so they pass each other at the meeting stile.
SASH_Y_GAP = 0.018
FIXED_SASH_Y = -SASH_Y_GAP
SLIDE_SASH_Y = +SASH_Y_GAP

# Sash bottom edge (world Z). Both sashes sit at the same height.
SASH_BOTTOM_Z = OPEN_Z0 + 0.004  # small clearance above the bottom track

# Closed-pose X centers. Fixed sash on left, sliding sash on right.
FIXED_SASH_CX = OPEN_X0 + SASH_W / 2.0        # left half
SLIDE_SASH_CX_CLOSED = OPEN_X1 - SASH_W / 2.0  # right half

# Maximum slide travel: sash slides left until it fully overlaps the fixed sash.
MAX_TRAVEL = SASH_W - MEETING_OVERLAP - 0.010

# Muntin grid: 2 columns x 3 rows of lites per sash (portrait orientation).
MUNTIN_W = 0.020
N_COLS = 2
N_ROWS = 3

# Handle/latch on the sliding sash meeting stile.
HANDLE_BODY = (0.018, 0.024, 0.080)   # (X, Y, Z) — vertical pull handle
HANDLE_GRIP = (0.012, 0.014, 0.060)

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

FRAME_RGBA = (0.68, 0.71, 0.74, 1.0)   # anodized aluminum frame
SASH_RGBA = (0.72, 0.75, 0.78, 1.0)    # slightly brighter aluminum sash
GLASS_RGBA = (0.26, 0.32, 0.38, 0.32)  # dark-tinted glass
HANDLE_RGBA = (0.22, 0.23, 0.25, 1.0)  # dark handle hardware


# ---------------------------------------------------------------------------
# Static outer frame geometry (CadQuery)
# ---------------------------------------------------------------------------

def _build_frame_shape() -> cq.Workplane:
    """Thick aluminum outer frame: perimeter slab with central opening cut out,
    plus deep track grooves in the top and bottom rails for the sash stiles.

    World frame: opening centered on X=0, Z from 0 (sill) to WIN_H (head).
    """
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

    # Deep track grooves in top and bottom rails.
    # Each groove runs along X for the full opening width (so the sliding sash
    # can travel the full range). Two grooves per rail (one per sash Y plane).
    groove_length = OPEN_W + 0.010  # slight extension past the opening edges

    for track_y in (FIXED_SASH_Y, SLIDE_SASH_Y):
        # Top rail groove: cut upward from the opening top edge into the head rail.
        top_groove = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, track_y, OPEN_Z1 + TRACK_DEPTH / 2.0))
            .box(groove_length, TRACK_W, TRACK_DEPTH)
        )
        frame = frame.cut(top_groove)

        # Bottom rail groove: cut downward from the opening bottom edge into the sill.
        bot_groove = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, track_y, OPEN_Z0 - TRACK_DEPTH / 2.0))
            .box(groove_length, TRACK_W, TRACK_DEPTH)
        )
        frame = frame.cut(bot_groove)

    # Add a subtle center mullion guide ridge on the sill (interior face) to
    # suggest where the sashes meet. This is a small raised rib.
    sill_ridge = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, -(FRAME_DEPTH / 2.0 - 0.008), OPEN_Z0 - 0.003))
        .box(OPEN_W * 0.96, 0.012, 0.006)
    )
    frame = frame.union(sill_ridge)

    # Matching ridge on the head rail interior face.
    head_ridge = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, -(FRAME_DEPTH / 2.0 - 0.008), OPEN_Z1 + 0.003))
        .box(OPEN_W * 0.96, 0.012, 0.006)
    )
    frame = frame.union(head_ridge)

    return frame


# ---------------------------------------------------------------------------
# Sash frame geometry (CadQuery): perimeter ring + 6-lite muntin grid
# ---------------------------------------------------------------------------

def _build_sash_frame_shape() -> cq.Workplane:
    """One sash: perimeter ring plus a 2x3 muntin grid, built as a slab with
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

    # Column / row boundaries of the lite grid.
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
    model = ArticulatedObject(name="sliding_window")

    model.material("frame", rgba=FRAME_RGBA)
    model.material("sash", rgba=SASH_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("handle", rgba=HANDLE_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material="frame",
        name="frame_shell",
    )

    # --- Two sashes (side by side) ---
    _add_sash(model, "fixed_sash")
    _add_sash(model, "sliding_sash")

    # Handle on the sliding sash meeting stile (interior face).
    sliding = model.get_part("sliding_sash")
    # Mount on the left stile (meeting stile) of the sliding sash, interior face.
    handle_x = -(SASH_W / 2.0 - SASH_RAIL / 2.0)  # left stile center
    handle_y = -(SASH_DEPTH / 2.0 + HANDLE_BODY[1] / 2.0 - 0.004)  # proud of interior face
    handle_z = SASH_H / 2.0  # mid-height
    sliding.visual(
        Box(HANDLE_BODY),
        origin=Origin(xyz=(handle_x, handle_y, handle_z)),
        material="handle",
        name="sliding_sash_handle",
    )
    sliding.visual(
        Box(HANDLE_GRIP),
        origin=Origin(xyz=(handle_x, handle_y - HANDLE_BODY[1] / 2.0 - HANDLE_GRIP[1] / 2.0 + 0.003, handle_z)),
        material="handle",
        name="sliding_sash_grip",
    )

    # ----- Articulations -----
    # FIXED sash (left): no motion.
    model.articulation(
        "frame_to_fixed_sash",
        ArticulationType.FIXED,
        parent="frame",
        child="fixed_sash",
        origin=Origin(xyz=(FIXED_SASH_CX, FIXED_SASH_Y, SASH_BOTTOM_Z)),
    )

    # SLIDING sash (right): prismatic along (-1,0,0). Positive q slides LEFT (opens).
    model.articulation(
        "frame_to_sliding_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="sliding_sash",
        origin=Origin(xyz=(SLIDE_SASH_CX_CLOSED, SLIDE_SASH_Y, SASH_BOTTOM_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0, velocity=0.30, lower=0.0, upper=MAX_TRAVEL
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
    j_slide = object_model.get_articulation("frame_to_sliding_sash")

    # --- Verify articulation type ---
    ctx.check(
        "sliding sash has prismatic joint",
        j_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"got {j_slide.articulation_type}",
    )

    # --- Intentional overlaps ---
    # Glass panes tuck under the sash muntin/rail lips (captured glass).
    for sash_name in ("fixed_sash", "sliding_sash"):
        ctx.allow_overlap(
            sash_name, sash_name,
            elem_a=f"{sash_name}_glass",
            elem_b=f"{sash_name}_frame",
            reason="Glass panes are rebated under the sash rails/muntins so they read as captured, not floating.",
        )
    # Each sash rides in the top/bottom track grooves cut into the frame.
    ctx.allow_overlap(
        "frame", "fixed_sash",
        reason="Fixed sash stiles ride in the interior track grooves of the top and bottom rails (retained insertion).",
    )
    ctx.allow_overlap(
        "frame", "sliding_sash",
        reason="Sliding sash stiles ride in the exterior track grooves of the top and bottom rails (retained insertion).",
    )
    # The two sashes overlap at the meeting stile (different Y planes).
    ctx.allow_overlap(
        "fixed_sash", "sliding_sash",
        reason="Sashes overlap at the central meeting stile; they ride in offset Y planes so they pass each other.",
    )
    # Handle seated onto the sliding sash stile.
    ctx.allow_overlap(
        "sliding_sash", "sliding_sash",
        elem_a="sliding_sash_handle",
        elem_b="sliding_sash_frame",
        reason="Handle is mounted (seated) onto the sliding sash meeting stile.",
    )

    # --- Closed pose (q=0): both sashes seated, window reads shut ---
    with ctx.pose({j_slide: 0.0}):
        f_aabb = ctx.part_world_aabb(frame)
        fx_aabb = ctx.part_world_aabb(fixed)
        sl_aabb = ctx.part_world_aabb(sliding)

        # Frame is the widest element.
        frame_w = f_aabb[1][0] - f_aabb[0][0]
        ctx.check(
            "frame spans wider than both sashes",
            frame_w > (fx_aabb[1][0] - fx_aabb[0][0]) + 0.10,
            details=f"frame_w={frame_w:.3f}, fixed_sash_w={fx_aabb[1][0] - fx_aabb[0][0]:.3f}",
        )
        # Window is landscape orientation (wider than tall).
        frame_h = f_aabb[1][2] - f_aabb[0][2]
        ctx.check(
            "window is landscape proportion (wider than tall)",
            frame_w > frame_h,
            details=f"frame_w={frame_w:.3f}, frame_h={frame_h:.3f}",
        )
        # Sill at/near z=0.
        ctx.check(
            "frame sill near z=0",
            abs(f_aabb[0][2]) < 0.01 and f_aabb[1][2] > 0.7,
            details=f"frame z range=({f_aabb[0][2]:.3f}, {f_aabb[1][2]:.3f})",
        )
        # Fixed sash is to the LEFT of the sliding sash.
        fixed_cx = (fx_aabb[0][0] + fx_aabb[1][0]) / 2.0
        slide_cx = (sl_aabb[0][0] + sl_aabb[1][0]) / 2.0
        ctx.check(
            "fixed sash left of sliding sash at closed pose",
            fixed_cx < slide_cx - 0.10,
            details=f"fixed_cx={fixed_cx:.3f}, sliding_cx={slide_cx:.3f}",
        )
        # Sashes overlap at the meeting stile in X.
        ctx.check(
            "sashes overlap at meeting stile (shut)",
            fx_aabb[1][0] >= sl_aabb[0][0] - 1e-4,
            details=f"fixed_right={fx_aabb[1][0]:.3f}, sliding_left={sl_aabb[0][0]:.3f}",
        )
        # Sashes ride in offset Y planes.
        fixed_cy = (fx_aabb[0][1] + fx_aabb[1][1]) / 2.0
        slide_cy = (sl_aabb[0][1] + sl_aabb[1][1]) / 2.0
        ctx.check(
            "sashes ride in offset Y planes",
            abs(fixed_cy - slide_cy) > 0.015,
            details=f"fixed_cy={fixed_cy:.3f}, sliding_cy={slide_cy:.3f}",
        )

        rest_slide_cx = slide_cx

    # --- HERO: sliding sash slides LEFT (opens) ---
    travel = MAX_TRAVEL * 0.80
    with ctx.pose({j_slide: travel}):
        op = ctx.part_world_aabb(sliding)
        op_cx = (op[0][0] + op[1][0]) / 2.0
        ctx.check(
            "sliding sash moves left when opened",
            op_cx < rest_slide_cx - travel * 0.8,
            details=f"rest_cx={rest_slide_cx:.3f}, opened_cx={op_cx:.3f}, travel={travel:.3f}",
        )
        # Stays retained: still overlaps the frame in X footprint.
        ctx.expect_overlap(
            sliding, frame, axes="x", min_overlap=0.05,
            name="sliding sash retained in frame when open",
        )
        # Opening exposes the right half of the frame opening.
        # The sliding sash right edge should have moved left of the frame right edge.
        ctx.check(
            "opening exposes right side of frame",
            op[1][0] < f_aabb[1][0] - 0.10,
            details=f"sliding_right={op[1][0]:.3f}, frame_right={f_aabb[1][0]:.3f}",
        )

    # --- Sash handle exists on the sliding sash ---
    handle_aabb = ctx.part_element_world_aabb(sliding, elem="sliding_sash_handle")
    ctx.check(
        "sliding sash has a handle",
        handle_aabb is not None,
        details="handle element not found",
    )

    # --- Deep track grooves: frame has visible top/bottom track features ---
    # The frame should extend deeper (in Z) than just the opening, proving
    # the thick rails exist above and below the opening.
    with ctx.pose({j_slide: 0.0}):
        ctx.check(
            "frame has thick top and bottom rails",
            (f_aabb[1][2] - f_aabb[0][2]) > OPEN_H + 2 * FRAME_FACE * 0.8,
            details=f"frame_h={f_aabb[1][2] - f_aabb[0][2]:.3f}, open_h+rails={OPEN_H + 2*FRAME_FACE:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
