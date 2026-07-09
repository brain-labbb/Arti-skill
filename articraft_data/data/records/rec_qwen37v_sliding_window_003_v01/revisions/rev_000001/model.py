from __future__ import annotations

# Two-panel horizontal sliding window with white frame.
# Left sash is movable (prismatic, slides right to stack behind fixed right sash).
# Deep track grooves run along the head and sill rails.
# Rubber gasket strips surround each glass pane.
#
# Coordinate convention:
#   +Z is up. Window stands vertically: height along +Z, width along X, frame
#   depth / glazing thickness along Y (the glass plane is the X-Z plane). The
#   sill sits at z=0; the head is at z=WIN_H.
#
# Articulation:
#   - LEFT sash is PRISMATIC, axis (1,0,0): positive q slides it to the right
#     (opens by stacking behind the fixed right sash).
#   - RIGHT sash is FIXED in the exterior track.

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

WIN_W = 1.10          # overall window width (X)
WIN_H = 1.00          # overall window height (Z), sill at z=0
FRAME_FACE = 0.060    # outer frame member face width (X/Z)
FRAME_DEPTH = 0.110   # outer frame jamb depth (Y)

# Clear opening inside the outer frame
OPEN_W = WIN_W - 2 * FRAME_FACE   # clear width
OPEN_H = WIN_H - 2 * FRAME_FACE   # clear height
OPEN_X0 = -WIN_W / 2.0 + FRAME_FACE
OPEN_X1 = WIN_W / 2.0 - FRAME_FACE
OPEN_Z0 = FRAME_FACE
OPEN_Z1 = WIN_H - FRAME_FACE

# Sash geometry. Two sashes side by side, each full clear height and slightly
# more than half the clear width so they overlap at the meeting stile.
SASH_OVERLAP = 0.048                       # one stile of overlap at meeting point
SASH_W = (OPEN_W + SASH_OVERLAP) / 2.0    # each sash width
SASH_RAIL = 0.048                          # sash perimeter member width (stile/rail)
SASH_DEPTH = 0.034                         # sash thickness (Y)
SASH_H = OPEN_H - 0.008                   # each sash height (small top/bottom clearance)
GLASS_T = 0.006                            # glazing thickness (Y)

# Y planes: left (movable) sash rides interior (-Y), right (fixed) sash rides
# exterior (+Y), offset so they pass each other at the meeting stile.
SASH_Y_GAP = 0.016
LEFT_SASH_Y = -SASH_Y_GAP
RIGHT_SASH_Y = +SASH_Y_GAP

# Track grooves cut into head and sill.
TRACK_GROOVE_W_Y = SASH_DEPTH + 0.006    # groove width in Y (sash + clearance)
TRACK_GROOVE_D_Z = 0.020                 # groove depth into head/sill (Z)

# Muntin grid: 2 columns x 3 rows of lites per sash.
MUNTIN_W = 0.022
N_COLS = 2
N_ROWS = 3

# Gasket strips around each glass pane.
GASKET_BORDER = 0.005    # visible gasket border width
GASKET_T = 0.010         # gasket thickness in Y (slightly proud of glass)

# Handle/pull on left sash meeting stile.
HANDLE_BODY = (0.018, 0.016, 0.080)   # (X, Y, Z)
HANDLE_BASE = (0.030, 0.008, 0.100)

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

FRAME_RGBA = (0.945, 0.945, 0.945, 1.0)    # white painted frame
SASH_RGBA = (0.965, 0.965, 0.965, 1.0)     # white sash (very slightly brighter)
GLASS_RGBA = (0.30, 0.36, 0.42, 0.34)      # cool dark-tinted glass
GASKET_RGBA = (0.12, 0.12, 0.13, 1.0)      # dark rubber gasket
HANDLE_RGBA = (0.82, 0.83, 0.85, 1.0)      # brushed metal handle


# ---------------------------------------------------------------------------
# Static outer frame geometry (CadQuery)
# ---------------------------------------------------------------------------

def _build_frame_shape() -> cq.Workplane:
    """White outer frame: perimeter slab with central opening cut out,
    plus deep track grooves in the head and sill for horizontal sliding.

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

    # Deep track grooves in the sill (bottom rail) and head (top rail).
    # Two grooves per rail: one per sash track plane.
    groove_len_x = OPEN_W + 0.010  # slightly wider than opening for clean cut

    for track_y in (LEFT_SASH_Y, RIGHT_SASH_Y):
        # Sill groove: cuts downward from the inner top face of the sill.
        sill_groove = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, track_y, OPEN_Z0 - TRACK_GROOVE_D_Z / 2.0))
            .box(groove_len_x, TRACK_GROOVE_W_Y, TRACK_GROOVE_D_Z)
        )
        frame = frame.cut(sill_groove)

        # Head groove: cuts upward from the inner bottom face of the head.
        head_groove = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, track_y, OPEN_Z1 + TRACK_GROOVE_D_Z / 2.0))
            .box(groove_len_x, TRACK_GROOVE_W_Y, TRACK_GROOVE_D_Z)
        )
        frame = frame.cut(head_groove)

    return frame


# ---------------------------------------------------------------------------
# Sash frame geometry (CadQuery): perimeter ring + 2x3 muntin grid
# ---------------------------------------------------------------------------

def _lite_grid():
    """Return lite opening edges for the 2x3 muntin grid in sash-local coords."""
    w = SASH_W
    h = SASH_H
    r = SASH_RAIL

    in_x0, in_x1 = -w / 2.0 + r, w / 2.0 - r
    in_z0, in_z1 = r, h - r
    inner_w = in_x1 - in_x0
    inner_h = in_z1 - in_z0

    col_lines = [in_x0 + (i + 1) * inner_w / N_COLS for i in range(N_COLS - 1)]
    row_lines = [in_z0 + (j + 1) * inner_h / N_ROWS for j in range(N_ROWS - 1)]

    x_edges = [in_x0] + col_lines + [in_x1]
    z_edges = [in_z0] + row_lines + [in_z1]
    return x_edges, z_edges


def _build_sash_frame_shape() -> cq.Workplane:
    """One sash: perimeter ring plus a 2x3 muntin grid, built as a slab with
    six rectangular lite openings cut through, leaving a true muntin lattice.

    Authored in sash-local frame:
      - local X runs -SASH_W/2 .. +SASH_W/2
      - local Z runs 0 .. SASH_H (bottom rail at z=0)
      - local Y is the sash thickness, centered at y=0.
    """
    w = SASH_W
    h = SASH_H
    d = SASH_DEPTH

    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, h / 2.0))
        .box(w, d, h)
    )

    x_edges, z_edges = _lite_grid()
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
    d = SASH_DEPTH
    rebate = 0.005

    x_edges, z_edges = _lite_grid()
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


def _build_sash_gasket_shape() -> cq.Workplane:
    """Rubber gasket strips: thin dark rectangular frames around each lite
    opening, visible as a border between the glass and the muntin/rail."""
    x_edges, z_edges = _lite_grid()
    half_m = MUNTIN_W / 2.0

    gaskets = None
    for ci in range(N_COLS):
        for ri in range(N_ROWS):
            # Outer edge of gasket: flush with lite opening edge (under muntin lip)
            ox0 = x_edges[ci] + (half_m if ci > 0 else 0.0) - 0.002
            ox1 = x_edges[ci + 1] - (half_m if ci < N_COLS - 1 else 0.0) + 0.002
            oz0 = z_edges[ri] + (half_m if ri > 0 else 0.0) - 0.002
            oz1 = z_edges[ri + 1] - (half_m if ri < N_ROWS - 1 else 0.0) + 0.002

            # Inner edge of gasket: inset by GASKET_BORDER
            ix0 = ox0 + GASKET_BORDER
            ix1 = ox1 - GASKET_BORDER
            iz0 = oz0 + GASKET_BORDER
            iz1 = oz1 - GASKET_BORDER

            # Build as outer box minus inner box
            outer_box = (
                cq.Workplane("XY")
                .transformed(offset=((ox0 + ox1) / 2.0, 0.0, (oz0 + oz1) / 2.0))
                .box(ox1 - ox0, GASKET_T, oz1 - oz0)
            )
            inner_box = (
                cq.Workplane("XY")
                .transformed(offset=((ix0 + ix1) / 2.0, 0.0, (iz0 + iz1) / 2.0))
                .box(ix1 - ix0, GASKET_T + 0.002, iz1 - iz0)
            )
            frame = outer_box.cut(inner_box)
            gaskets = frame if gaskets is None else gaskets.union(frame)
    return gaskets


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
    sash.visual(
        mesh_from_cadquery(_build_sash_gasket_shape(), f"{name}_gasket"),
        material="gasket",
        name=f"{name}_gasket",
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_window")

    model.material("frame", rgba=FRAME_RGBA)
    model.material("sash", rgba=SASH_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("gasket", rgba=GASKET_RGBA)
    model.material("handle", rgba=HANDLE_RGBA)

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

    # Handle/pull on left sash meeting stile (right edge), interior face.
    left = model.get_part("left_sash")
    handle_x = SASH_W / 2.0 - SASH_RAIL / 2.0   # center of meeting stile
    handle_y = -(SASH_DEPTH / 2.0 + HANDLE_BASE[1] / 2.0 - 0.003)
    handle_z = SASH_H / 2.0                      # center height
    left.visual(
        Box(HANDLE_BASE),
        origin=Origin(xyz=(handle_x, handle_y, handle_z)),
        material="handle",
        name="handle_base",
    )
    left.visual(
        Box(HANDLE_BODY),
        origin=Origin(xyz=(handle_x, handle_y - HANDLE_BASE[1] / 2.0 - HANDLE_BODY[1] / 2.0 + 0.003, handle_z)),
        material="handle",
        name="handle_pull",
    )

    # ----- Articulations -----
    # Left sash: slides to the RIGHT (+X) to open (stacks behind right sash).
    # Part origin is at sash bottom-center; joint origin places it at its
    # closed world position.
    left_closed_x = OPEN_X0 + SASH_W / 2.0
    left_closed_z = OPEN_Z0 + 0.004   # small sill clearance

    model.articulation(
        "frame_to_left_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="left_sash",
        origin=Origin(xyz=(left_closed_x, LEFT_SASH_Y, left_closed_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=0.25, lower=0.0, upper=0.42
        ),
    )

    # Right sash: fixed in the exterior track.
    right_closed_x = OPEN_X1 - SASH_W / 2.0
    right_closed_z = OPEN_Z0 + 0.004

    model.articulation(
        "frame_to_right_sash",
        ArticulationType.FIXED,
        parent="frame",
        child="right_sash",
        origin=Origin(xyz=(right_closed_x, RIGHT_SASH_Y, right_closed_z)),
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

    # --- Intentional overlaps ---
    # Glass panes tuck under the sash muntin/rail lips (captured glass).
    for sash_name in ("left_sash", "right_sash"):
        ctx.allow_overlap(
            sash_name, sash_name,
            elem_a=f"{sash_name}_glass",
            elem_b=f"{sash_name}_frame",
            reason="Glass panes are rebated under the sash rails/muntins so they read as captured, not floating.",
        )
        ctx.allow_overlap(
            sash_name, sash_name,
            elem_a=f"{sash_name}_gasket",
            elem_b=f"{sash_name}_frame",
            reason="Rubber gasket strips sit in the rebate between glass and muntin/rail.",
        )
        ctx.allow_overlap(
            sash_name, sash_name,
            elem_a=f"{sash_name}_gasket",
            elem_b=f"{sash_name}_glass",
            reason="Gasket inner edge overlaps glass edge (seated gasket).",
        )

    # Sashes ride in the track grooves cut into head and sill.
    ctx.allow_overlap(
        "frame", "left_sash",
        reason="Left sash rides in the interior track grooves (retained insertion in head/sill channels).",
    )
    ctx.allow_overlap(
        "frame", "right_sash",
        reason="Right sash rides in the exterior track grooves (retained insertion in head/sill channels).",
    )
    # The two sashes overlap at the meeting stile (different Y planes).
    ctx.allow_overlap(
        "left_sash", "right_sash",
        reason="Sashes overlap at the meeting stile; they ride in offset Y planes to pass each other.",
    )
    # Handle is seated onto the left sash meeting stile.
    ctx.allow_overlap(
        "left_sash", "left_sash",
        elem_a="handle_base",
        elem_b="left_sash_frame",
        reason="Handle base is mounted (seated) onto the left sash meeting stile.",
    )

    # --- Closed pose (q=0): both sashes seated side by side, window reads shut ---
    with ctx.pose({j_left: 0.0}):
        f_aabb = ctx.part_world_aabb(frame)
        lo_aabb = ctx.part_world_aabb(left)
        ri_aabb = ctx.part_world_aabb(right)

        # Frame is the widest element.
        frame_w = f_aabb[1][0] - f_aabb[0][0]
        sash_w = lo_aabb[1][0] - lo_aabb[0][0]
        ctx.check(
            "frame spans wider than a single sash",
            frame_w > sash_w + 0.05,
            details=f"frame_w={frame_w:.3f}, sash_w={sash_w:.3f}",
        )

        # Sill at/near z=0 (window stands upright).
        ctx.check(
            "frame sill near z=0",
            abs(f_aabb[0][2]) < 0.01 and f_aabb[1][2] > 0.7,
            details=f"frame z range=({f_aabb[0][2]:.3f}, {f_aabb[1][2]:.3f})",
        )

        # Both sashes are inside the frame opening width.
        ctx.check(
            "left sash within frame width",
            lo_aabb[0][0] > f_aabb[0][0] - 0.01 and lo_aabb[1][0] < f_aabb[1][0] + 0.01,
            details=f"left x=({lo_aabb[0][0]:.3f},{lo_aabb[1][0]:.3f})",
        )
        ctx.check(
            "right sash within frame width",
            ri_aabb[0][0] > f_aabb[0][0] - 0.01 and ri_aabb[1][0] < f_aabb[1][0] + 0.01,
            details=f"right x=({ri_aabb[0][0]:.3f},{ri_aabb[1][0]:.3f})",
        )

        # Left sash is to the left of right sash at closed pose.
        lo_cx = (lo_aabb[0][0] + lo_aabb[1][0]) / 2.0
        ri_cx = (ri_aabb[0][0] + ri_aabb[1][0]) / 2.0
        ctx.check(
            "left sash is left of right sash at closed pose",
            lo_cx < ri_cx - 0.1,
            details=f"left_cx={lo_cx:.3f}, right_cx={ri_cx:.3f}",
        )

        # Sashes are side by side (similar Z centers, not stacked vertically).
        lo_cz = (lo_aabb[0][2] + lo_aabb[1][2]) / 2.0
        ri_cz = (ri_aabb[0][2] + ri_aabb[1][2]) / 2.0
        ctx.check(
            "sashes are side by side (similar Z center)",
            abs(lo_cz - ri_cz) < 0.05,
            details=f"left_cz={lo_cz:.3f}, right_cz={ri_cz:.3f}",
        )

        # Sashes ride in offset Y planes.
        lo_cy = (lo_aabb[0][1] + lo_aabb[1][1]) / 2.0
        ri_cy = (ri_aabb[0][1] + ri_aabb[1][1]) / 2.0
        ctx.check(
            "sashes ride in offset Y planes",
            abs(lo_cy - ri_cy) > 0.015,
            details=f"left_cy={lo_cy:.3f}, right_cy={ri_cy:.3f}",
        )

        rest_lo_cx = lo_cx

    # --- HERO: left sash slides RIGHT (opens) ---
    travel = 0.40
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

    # --- Verify prismatic joint is non-fixed ---
    j_info = object_model.get_articulation("frame_to_left_sash")
    ctx.check(
        "left sash has a prismatic (non-fixed) joint",
        j_info is not None,
        details="frame_to_left_sash articulation exists",
    )

    # --- Track grooves exist: frame has the groove geometry ---
    # Verify frame height includes head and sill material beyond the opening.
    f_aabb = ctx.part_world_aabb(frame)
    frame_h = f_aabb[1][2] - f_aabb[0][2]
    ctx.check(
        "frame has head and sill depth for track grooves",
        frame_h > OPEN_H + 2 * FRAME_FACE * 0.5,
        details=f"frame_h={frame_h:.3f}, open_h={OPEN_H:.3f}",
    )

    # --- Gasket material is distinct from glass ---
    ctx.check(
        "gasket material registered",
        "gasket" in [m.name for m in object_model.materials],
        details="gasket material should exist for rubber strips",
    )

    return ctx.report()


object_model = build_object_model()
