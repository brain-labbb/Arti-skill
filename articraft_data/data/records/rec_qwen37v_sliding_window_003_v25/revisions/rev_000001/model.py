from __future__ import annotations

# Horizontal sliding window: thick aluminum frame with deep track grooves,
# two side-by-side six-lite sashes that slide horizontally, a rotating latch
# at the meeting stile, and rubber gasket strips around each glass pane.
#
# Coordinate convention:
#   +Z is up. Window stands vertically: width along X, height along Z,
#   frame depth along Y. The sill sits at z=0; the head at z=WIN_H.
#   Sashes slide along X (horizontal slider).
#
# Articulation:
#   - LEFT sash: PRISMATIC, axis (-1,0,0): positive q slides it LEFT (opens).
#   - RIGHT sash: PRISMATIC, axis (1,0,0): positive q slides it RIGHT (opens).
#   - LATCH: REVOLUTE on the right sash meeting stile, axis (0,0,1):
#     positive q rotates the latch to unlocked position.

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

WIN_W = 1.20          # overall window width (X)
WIN_H = 0.92          # overall window height (Z), sill at z=0
FRAME_FACE = 0.070    # thick aluminum frame rail/stile face width
FRAME_DEPTH = 0.130   # deep frame jamb depth for dual tracks

# Clear opening inside the outer frame
OPEN_W = WIN_W - 2 * FRAME_FACE
OPEN_H = WIN_H - 2 * FRAME_FACE
OPEN_X0 = -WIN_W / 2.0 + FRAME_FACE
OPEN_X1 = WIN_W / 2.0 - FRAME_FACE
OPEN_Z0 = FRAME_FACE
OPEN_Z1 = WIN_H - FRAME_FACE

# Track grooves in top (head) and bottom (sill) rails.
TRACK_GROOVE_W = 0.028   # groove width (Y direction, across frame depth)
TRACK_GROOVE_DEPTH = 0.035  # how deep the groove cuts into the rail (Z)
TRACK_SPACING = 0.048    # center-to-center Y spacing between the two tracks

# Sash geometry: each sash is full opening height, slightly over half width.
SASH_H = OPEN_H - 0.008          # running clearance top/bottom in tracks
SASH_W = OPEN_W * 0.535          # slight overlap at meeting stile
SASH_RAIL = 0.050                # sash perimeter member width
SASH_STILE = 0.048               # sash vertical member width
SASH_DEPTH = 0.032               # sash thickness (Y)
GLASS_T = 0.006                  # glass pane thickness

# Y planes: left sash rides in interior track (-Y), right in exterior (+Y).
SASH_Y_OFFSET = TRACK_SPACING / 2.0
LEFT_SASH_Y = -SASH_Y_OFFSET
RIGHT_SASH_Y = +SASH_Y_OFFSET

# Closed-pose: sashes meet at center (X≈0), overlapping slightly at meeting stile.
MEETING_X = 0.0
LEFT_SASH_CLOSED_X = MEETING_X - SASH_W / 2.0 + 0.010  # left sash center X
RIGHT_SASH_CLOSED_X = MEETING_X + SASH_W / 2.0 - 0.010  # right sash center X

# Muntin grid: 3 columns x 2 rows per sash = 6 lites.
MUNTIN_W = 0.020
N_COLS = 3
N_ROWS = 2

# Rubber gasket strips around each glass pane.
GASKET_W = 0.005   # gasket strip width visible around each pane
GASKET_T = 0.004   # gasket thickness (Y)

# Latch at meeting stile.
LATCH_BASE = (0.030, 0.018, 0.040)   # base block mounted on sash
LATCH_LEVER = (0.050, 0.010, 0.014)  # rotating lever
LATCH_PIVOT_OFFSET_X = -SASH_W / 2.0 + SASH_STILE / 2.0  # on left stile of right sash
LATCH_PIVOT_Z = SASH_H * 0.50        # mid-height of sash

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

ALUMINUM_RGBA = (0.72, 0.74, 0.76, 1.0)     # brushed aluminum frame
SASH_RGBA = (0.78, 0.80, 0.82, 1.0)         # lighter aluminum sash
GLASS_RGBA = (0.32, 0.38, 0.44, 0.30)       # tinted glass
GASKET_RGBA = (0.12, 0.12, 0.13, 1.0)       # dark rubber gasket
LATCH_RGBA = (0.60, 0.62, 0.64, 1.0)        # metal latch


# ---------------------------------------------------------------------------
# Frame geometry (CadQuery): thick aluminum perimeter with deep track grooves
# ---------------------------------------------------------------------------

def _build_frame_shape() -> cq.Workplane:
    """Aluminum outer frame: perimeter slab with central opening, plus deep
    track grooves in the head and sill for horizontal sash travel."""
    # Solid outer slab.
    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, WIN_H / 2.0))
        .box(WIN_W, FRAME_DEPTH, WIN_H)
    )

    # Cut the clear central opening.
    opening = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, (OPEN_Z0 + OPEN_Z1) / 2.0))
        .box(OPEN_W, FRAME_DEPTH + 0.02, OPEN_H)
    )
    frame = outer.cut(opening)

    # Deep track grooves in head (top rail) and sill (bottom rail).
    # Two grooves per rail (one per sash track), running the full opening width.
    # Each groove is a channel cut into the inner face of the rail.
    for rail_z, cut_dir in ((OPEN_Z1, -1.0), (OPEN_Z0, +1.0)):
        # cut_dir: -1 for head (cut downward from opening top), +1 for sill (cut upward from opening bottom)
        groove_cz = rail_z + cut_dir * TRACK_GROOVE_DEPTH / 2.0
        for track_y in (LEFT_SASH_Y, RIGHT_SASH_Y):
            groove = (
                cq.Workplane("XY")
                .transformed(offset=(0.0, track_y, groove_cz))
                .box(OPEN_W + 0.01, TRACK_GROOVE_W, TRACK_GROOVE_DEPTH)
            )
            frame = frame.cut(groove)

    # Side-track grooves in jambs (so sash stiles are retained at travel limits).
    groove_x_depth = FRAME_FACE * 0.50
    for sign, edge_x in ((+1.0, OPEN_X0), (-1.0, OPEN_X1)):
        cx = edge_x - sign * groove_x_depth / 2.0
        for track_y in (LEFT_SASH_Y, RIGHT_SASH_Y):
            groove = (
                cq.Workplane("XY")
                .transformed(offset=(cx, track_y, (OPEN_Z0 + OPEN_Z1) / 2.0))
                .box(groove_x_depth, TRACK_GROOVE_W, OPEN_H)
            )
            frame = frame.cut(groove)

    return frame


# ---------------------------------------------------------------------------
# Sash frame geometry (CadQuery): perimeter ring + 6-lite muntin grid
# ---------------------------------------------------------------------------

def _build_sash_frame_shape() -> cq.Workplane:
    """One sash frame: perimeter ring with 3x2 muntin grid.
    Local frame: X centered, Z from 0 to SASH_H, Y centered at 0."""
    w = SASH_W
    h = SASH_H
    rs = SASH_STILE
    rr = SASH_RAIL
    d = SASH_DEPTH

    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, h / 2.0))
        .box(w, d, h)
    )

    # Inner glazed region.
    in_x0, in_x1 = -w / 2.0 + rs, w / 2.0 - rs
    in_z0, in_z1 = rr, h - rr
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
    rs = SASH_STILE
    rr = SASH_RAIL
    rebate = 0.004

    in_x0, in_x1 = -w / 2.0 + rs, w / 2.0 - rs
    in_z0, in_z1 = rr, h - rr
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


def _build_sash_gasket_shape() -> cq.Workplane:
    """Rubber gasket strips around each glass pane: thin dark frames that sit
    between the glass and the sash frame, visible as a dark border."""
    w = SASH_W
    h = SASH_H
    rs = SASH_STILE
    rr = SASH_RAIL

    in_x0, in_x1 = -w / 2.0 + rs, w / 2.0 - rs
    in_z0, in_z1 = rr, h - rr
    inner_w = in_x1 - in_x0
    inner_h = in_z1 - in_z0
    col_lines = [in_x0 + (i + 1) * inner_w / N_COLS for i in range(N_COLS - 1)]
    row_lines = [in_z0 + (j + 1) * inner_h / N_ROWS for j in range(N_ROWS - 1)]
    x_edges = [in_x0] + col_lines + [in_x1]
    z_edges = [in_z0] + row_lines + [in_z1]
    half_m = MUNTIN_W / 2.0

    gaskets = None
    for ci in range(N_COLS):
        for ri in range(N_ROWS):
            # Outer extent of gasket (slightly larger than glass, under muntins)
            lx0_out = x_edges[ci] + (half_m if ci > 0 else 0.0) - GASKET_W
            lx1_out = x_edges[ci + 1] - (half_m if ci < N_COLS - 1 else 0.0) + GASKET_W
            lz0_out = z_edges[ri] + (half_m if ri > 0 else 0.0) - GASKET_W
            lz1_out = z_edges[ri + 1] - (half_m if ri < N_ROWS - 1 else 0.0) + GASKET_W
            # Inner extent (the glass opening)
            lx0_in = lx0_out + GASKET_W
            lx1_in = lx1_out - GASKET_W
            lz0_in = lz0_out + GASKET_W
            lz1_in = lz1_out - GASKET_W

            # Build gasket as outer rectangle minus inner cutout
            outer_box = (
                cq.Workplane("XY")
                .transformed(offset=((lx0_out + lx1_out) / 2.0, 0.0, (lz0_out + lz1_out) / 2.0))
                .box(lx1_out - lx0_out, GASKET_T, lz1_out - lz0_out)
            )
            inner_box = (
                cq.Workplane("XY")
                .transformed(offset=((lx0_in + lx1_in) / 2.0, 0.0, (lz0_in + lz1_in) / 2.0))
                .box(lx1_in - lx0_in, GASKET_T + 0.002, lz1_in - lz0_in)
            )
            gasket_frame = outer_box.cut(inner_box)
            gaskets = gasket_frame if gaskets is None else gaskets.union(gasket_frame)
    return gaskets


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_window")

    model.material("frame", rgba=ALUMINUM_RGBA)
    model.material("sash", rgba=SASH_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("gasket", rgba=GASKET_RGBA)
    model.material("latch", rgba=LATCH_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame_shell"),
        material="frame",
        name="frame_shell",
    )

    # --- Left sash ---
    left_sash = model.part("left_sash")
    left_sash.visual(
        mesh_from_cadquery(_build_sash_frame_shape(), "left_sash_frame"),
        material="sash",
        name="left_sash_frame",
    )
    left_sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(), "left_sash_glass"),
        material="glass",
        name="left_sash_glass",
    )
    left_sash.visual(
        mesh_from_cadquery(_build_sash_gasket_shape(), "left_sash_gasket"),
        material="gasket",
        name="left_sash_gasket",
    )

    # --- Right sash ---
    right_sash = model.part("right_sash")
    right_sash.visual(
        mesh_from_cadquery(_build_sash_frame_shape(), "right_sash_frame"),
        material="sash",
        name="right_sash_frame",
    )
    right_sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(), "right_sash_glass"),
        material="glass",
        name="right_sash_glass",
    )
    right_sash.visual(
        mesh_from_cadquery(_build_sash_gasket_shape(), "right_sash_gasket"),
        material="gasket",
        name="right_sash_gasket",
    )

    # --- Latch (separate part with revolute joint on right sash) ---
    latch = model.part("latch")
    # Latch base is fixed relative to latch part frame
    latch.visual(
        Box(LATCH_BASE),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="latch",
        name="latch_base",
    )
    # Latch lever extends from the pivot point
    latch.visual(
        Box(LATCH_LEVER),
        origin=Origin(xyz=(LATCH_LEVER[0] / 2.0, 0.0, 0.0)),
        material="latch",
        name="latch_lever",
    )

    # ----- Articulations -----

    # LEFT sash: slides LEFT to open. axis (-1,0,0), positive q moves it left.
    # Part frame origin at the sash bottom center. Closed pose places it
    # at the left side of center (meeting stile at X≈0).
    model.articulation(
        "frame_to_left_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="left_sash",
        origin=Origin(xyz=(LEFT_SASH_CLOSED_X, LEFT_SASH_Y, OPEN_Z0 + 0.004)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=50.0, velocity=0.3, lower=0.0, upper=SASH_W * 0.75
        ),
    )

    # RIGHT sash: slides RIGHT to open. axis (1,0,0), positive q moves it right.
    model.articulation(
        "frame_to_right_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="right_sash",
        origin=Origin(xyz=(RIGHT_SASH_CLOSED_X, RIGHT_SASH_Y, OPEN_Z0 + 0.004)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=50.0, velocity=0.3, lower=0.0, upper=SASH_W * 0.75
        ),
    )

    # LATCH: revolute joint on right sash at the meeting stile.
    # Pivot at the left stile of the right sash, mid-height.
    # axis (0,0,1): positive q rotates the lever out (unlocked).
    latch_origin_x = RIGHT_SASH_CLOSED_X + LATCH_PIVOT_OFFSET_X
    latch_origin_y = RIGHT_SASH_Y - (SASH_DEPTH / 2.0 + LATCH_BASE[1] / 2.0 - 0.003)
    latch_origin_z = OPEN_Z0 + 0.004 + LATCH_PIVOT_Z

    model.articulation(
        "right_sash_to_latch",
        ArticulationType.REVOLUTE,
        parent="right_sash",
        child="latch",
        origin=Origin(xyz=(LATCH_PIVOT_OFFSET_X, -(SASH_DEPTH / 2.0 + LATCH_BASE[1] / 2.0 - 0.003), LATCH_PIVOT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=0.0, upper=1.57
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
    latch = object_model.get_part("latch")
    j_left = object_model.get_articulation("frame_to_left_sash")
    j_right = object_model.get_articulation("frame_to_right_sash")
    j_latch = object_model.get_articulation("right_sash_to_latch")

    # --- Intentional overlaps ---
    # Glass panes tuck under sash muntins/rails (captured glass).
    for sash_name in ("left_sash", "right_sash"):
        ctx.allow_overlap(
            sash_name, sash_name,
            elem_a=f"{sash_name}_glass",
            elem_b=f"{sash_name}_frame",
            reason="Glass panes are rebated under the sash rails/muntins (captured glazing).",
        )
        ctx.allow_overlap(
            sash_name, sash_name,
            elem_a=f"{sash_name}_gasket",
            elem_b=f"{sash_name}_frame",
            reason="Rubber gasket strips sit between glass and sash frame muntins/rails.",
        )
        ctx.allow_overlap(
            sash_name, sash_name,
            elem_a=f"{sash_name}_gasket",
            elem_b=f"{sash_name}_glass",
            reason="Gasket surrounds glass pane edges for weatherseal representation.",
        )

    # Sashes ride in frame track grooves (retained insertion).
    ctx.allow_overlap(
        "frame", "left_sash",
        reason="Left sash rides in the head/sill track grooves (retained insertion).",
    )
    ctx.allow_overlap(
        "frame", "right_sash",
        reason="Right sash rides in the head/sill track grooves (retained insertion).",
    )

    # Sashes overlap at meeting stile (different Y planes).
    ctx.allow_overlap(
        "left_sash", "right_sash",
        reason="Sashes overlap at the meeting stile; they ride in offset Y track planes.",
    )

    # Latch is seated onto the right sash stile.
    ctx.allow_overlap(
        "right_sash", "latch",
        elem_a="right_sash_frame",
        elem_b="latch_base",
        reason="Latch base is mounted (seated) onto the right sash meeting stile.",
    )

    # --- Closed pose (q=0): both sashes centered, window reads shut ---
    with ctx.pose({j_left: 0.0, j_right: 0.0, j_latch: 0.0}):
        f_aabb = ctx.part_world_aabb(frame)
        lo_aabb = ctx.part_world_aabb(left)
        ri_aabb = ctx.part_world_aabb(right)

        # Frame is the widest element.
        frame_w = f_aabb[1][0] - f_aabb[0][0]
        sash_w = lo_aabb[1][0] - lo_aabb[0][0]
        ctx.check(
            "frame spans wider than a sash",
            frame_w > sash_w + 0.1,
            details=f"frame_w={frame_w:.3f}, sash_w={sash_w:.3f}",
        )
        # Sill near z=0, window stands upright.
        ctx.check(
            "frame sill near z=0",
            abs(f_aabb[0][2]) < 0.01 and f_aabb[1][2] > 0.7,
            details=f"frame z=({f_aabb[0][2]:.3f}, {f_aabb[1][2]:.3f})",
        )
        # Both sashes within frame opening.
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
        # Left sash center is to the left of right sash center (side by side).
        lo_cx = (lo_aabb[0][0] + lo_aabb[1][0]) / 2.0
        ri_cx = (ri_aabb[0][0] + ri_aabb[1][0]) / 2.0
        ctx.check(
            "left sash is left of right sash at closed pose",
            lo_cx < ri_cx - 0.05,
            details=f"left_cx={lo_cx:.3f}, right_cx={ri_cx:.3f}",
        )
        # Sashes overlap at meeting stile in X (no daylight gap when shut).
        ctx.check(
            "sashes overlap at meeting stile (shut)",
            lo_aabb[1][0] >= ri_aabb[0][0] - 1e-4,
            details=f"left_right={lo_aabb[1][0]:.3f}, right_left={ri_aabb[0][0]:.3f}",
        )
        # Sashes in offset Y planes (different tracks).
        lo_cy = (lo_aabb[0][1] + lo_aabb[1][1]) / 2.0
        ri_cy = (ri_aabb[0][1] + ri_aabb[1][1]) / 2.0
        ctx.check(
            "sashes ride in offset Y track planes",
            abs(lo_cy - ri_cy) > 0.02,
            details=f"left_cy={lo_cy:.3f}, right_cy={ri_cy:.3f}",
        )

        rest_lo_cx = lo_cx
        rest_ri_cx = ri_cx

    # --- HERO: left sash slides LEFT (opens) ---
    travel = SASH_W * 0.60
    with ctx.pose({j_left: travel}):
        op = ctx.part_world_aabb(left)
        op_cx = (op[0][0] + op[1][0]) / 2.0
        ctx.check(
            "left sash slides left when opened",
            op_cx < rest_lo_cx - travel * 0.8,
            details=f"rest_cx={rest_lo_cx:.3f}, opened_cx={op_cx:.3f}, travel={travel:.3f}",
        )
        # Still retained in frame.
        ctx.expect_overlap(
            left, frame, axes="z", min_overlap=0.05,
            name="left sash retained in frame tracks when open",
        )

    # --- HERO: right sash slides RIGHT (opens) ---
    with ctx.pose({j_right: travel}):
        op = ctx.part_world_aabb(right)
        op_cx = (op[0][0] + op[1][0]) / 2.0
        ctx.check(
            "right sash slides right when opened",
            op_cx > rest_ri_cx + travel * 0.8,
            details=f"rest_cx={rest_ri_cx:.3f}, opened_cx={op_cx:.3f}, travel={travel:.3f}",
        )
        ctx.expect_overlap(
            right, frame, axes="z", min_overlap=0.05,
            name="right sash retained in frame tracks when open",
        )

    # --- Latch rotates on revolute joint ---
    ctx.check(
        "latch has revolute articulation",
        j_latch.articulation_type == ArticulationType.REVOLUTE,
        details=f"latch joint type={j_latch.articulation_type}",
    )
    # Compare latch lever AABB at locked vs unlocked poses.
    with ctx.pose({j_latch: 0.0, j_left: 0.0, j_right: 0.0}):
        lever_aabb_0 = ctx.part_element_world_aabb(latch, elem="latch_lever")
    with ctx.pose({j_latch: 1.2, j_left: 0.0, j_right: 0.0}):
        lever_aabb_1 = ctx.part_element_world_aabb(latch, elem="latch_lever")
    if lever_aabb_0 is not None and lever_aabb_1 is not None:
        # Lever center should shift when the latch rotates.
        cx0 = (lever_aabb_0[0][0] + lever_aabb_0[1][0]) / 2.0
        cy0 = (lever_aabb_0[0][1] + lever_aabb_0[1][1]) / 2.0
        cx1 = (lever_aabb_1[0][0] + lever_aabb_1[1][0]) / 2.0
        cy1 = (lever_aabb_1[0][1] + lever_aabb_1[1][1]) / 2.0
        ctx.check(
            "latch lever rotates when actuated",
            abs(cx1 - cx0) > 0.002 or abs(cy1 - cy0) > 0.002,
            details=f"lever_center q=0: ({cx0:.4f},{cy0:.4f}), q=1.2: ({cx1:.4f},{cy1:.4f})",
        )

    # --- Gasket strips exist on both sashes ---
    for sash_name in ("left_sash", "right_sash"):
        sash_part = object_model.get_part(sash_name)
        gasket_aabb = ctx.part_element_world_aabb(sash_part, elem=f"{sash_name}_gasket")
        ctx.check(
            f"{sash_name} has rubber gasket strips",
            gasket_aabb is not None,
            details=f"gasket AABB={gasket_aabb}",
        )

    # --- Track grooves exist: frame has substantial depth for deep tracks ---
    f_dims = None
    with ctx.pose({j_left: 0.0, j_right: 0.0}):
        f_aabb = ctx.part_world_aabb(frame)
        if f_aabb is not None:
            f_dims = [f_aabb[1][i] - f_aabb[0][i] for i in range(3)]
    if f_dims is not None:
        ctx.check(
            "frame depth accommodates deep tracks",
            f_dims[1] > 0.10,
            details=f"frame depth (Y)={f_dims[1]:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
