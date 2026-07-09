from __future__ import annotations

# Three-panel horizontal sliding window with white frame.
# Wider fixed center pane flanked by two sliding sashes (2 cols x 3 rows each).
# Each sliding sash has a tilt-in latch (revolute) and two roller blocks at bottom.
#
# Coordinate convention:
#   +Z is up. Window stands vertically: height along +Z, width along X,
#   depth along Y. Sill at z=0, head at z=WIN_H.
#
# Articulation:
#   - LEFT sash: PRISMATIC axis (1,0,0): positive q slides RIGHT (opens).
#   - RIGHT sash: PRISMATIC axis (-1,0,0): positive q slides LEFT (opens).
#   - LEFT latch: REVOLUTE axis (1,0,0): positive q tilts latch forward.
#   - RIGHT latch: REVOLUTE axis (1,0,0): positive q tilts latch forward.
#   The fixed center pane is part of the frame assembly (no separate joint).
#   Sashes ride on the interior (-Y) track plane; the center pane sits on the
#   exterior (+Y) plane, so sliding sashes clear the fixed center pane in Y.

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
WIN_H = 1.10          # overall window height (Z), sill at z=0
FRAME_FACE = 0.055    # outer frame member face width (X/Z)
FRAME_DEPTH = 0.100   # outer frame jamb depth (Y)

# Clear opening inside the outer frame
OPEN_W = WIN_W - 2 * FRAME_FACE   # 1.090
OPEN_H = WIN_H - 2 * FRAME_FACE   # 0.990
OPEN_X0 = -OPEN_W / 2.0           # -0.545
OPEN_X1 = OPEN_W / 2.0            #  0.545
OPEN_Z0 = FRAME_FACE               # 0.055
OPEN_Z1 = WIN_H - FRAME_FACE       # 1.045

# Three-panel layout: center section is wider than side sections.
CENTER_W = 0.44                   # fixed center pane width
SIDE_W = (OPEN_W - CENTER_W) / 2  # 0.325 each side

# Sash geometry (each sliding sash fills one side section).
SASH_W = SIDE_W - 0.008           # running clearance in the track
SASH_H = OPEN_H                   # exact fit: sash contacts sill and head
SASH_RAIL = 0.045                 # sash perimeter member width
SASH_DEPTH = 0.032                # sash thickness (Y)
GLASS_T = 0.005                   # glazing thickness (Y)

# Muntin grid per sash: 2 columns x 3 rows = 6 lites.
SASH_COLS = 2
SASH_ROWS = 3
MUNTIN_W = 0.018

# Center pane (fixed, built into the frame): 3 cols x 2 rows muntin grid.
CP_W = CENTER_W - 0.008           # center pane frame width
CP_H = OPEN_H                     # exact fit between sill and head
CP_RAIL = 0.035                   # center pane rail width (thinner)
CP_DEPTH = 0.028                  # center pane frame depth
CP_COLS = 3
CP_ROWS = 2

# Y planes: sashes ride on the interior (-Y) track, center pane on exterior (+Y).
SASH_Y = -0.020
CENTER_Y = 0.022

# Closed-pose sash center X (world).
LEFT_SASH_X = OPEN_X0 + SIDE_W / 2.0    # -0.3825
RIGHT_SASH_X = OPEN_X1 - SIDE_W / 2.0   #  0.3825

# Bottom Z for sashes and center pane (contact the sill surface).
SASH_BOTTOM_Z = OPEN_Z0
CP_BOTTOM_Z = OPEN_Z0

# Travel distance.
SASH_TRAVEL = SIDE_W * 0.82

# Roller blocks at the bottom of each sash.
ROLLER_SIZE = (0.028, 0.022, 0.013)

# Tilt-in latch body dimensions.
LATCH_BODY = (0.038, 0.013, 0.022)

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

FRAME_RGBA = (0.945, 0.945, 0.945, 1.0)    # white painted frame
SASH_RGBA = (0.965, 0.965, 0.965, 1.0)     # white sash (slightly brighter)
GLASS_RGBA = (0.30, 0.36, 0.42, 0.34)      # cool dark-tinted glass
HARDWARE_RGBA = (0.72, 0.73, 0.76, 1.0)    # brushed metal hardware
ROLLER_RGBA = (0.22, 0.22, 0.24, 1.0)      # dark nylon rollers


# ---------------------------------------------------------------------------
# Static outer frame geometry (CadQuery)
# ---------------------------------------------------------------------------

def _build_frame_shape() -> cq.Workplane:
    """White outer frame: four perimeter members (jambs, sill, head) forming
    a single connected solid with a clear central opening."""
    # Left jamb
    left_jamb = (
        cq.Workplane("XY")
        .transformed(offset=(-(WIN_W / 2.0 - FRAME_FACE / 2.0), 0.0, WIN_H / 2.0))
        .box(FRAME_FACE, FRAME_DEPTH, WIN_H)
    )
    # Right jamb
    right_jamb = (
        cq.Workplane("XY")
        .transformed(offset=((WIN_W / 2.0 - FRAME_FACE / 2.0), 0.0, WIN_H / 2.0))
        .box(FRAME_FACE, FRAME_DEPTH, WIN_H)
    )
    # Sill (spans full width for corner overlap with jambs)
    sill = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, FRAME_FACE / 2.0))
        .box(WIN_W, FRAME_DEPTH, FRAME_FACE)
    )
    # Head
    head = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, WIN_H - FRAME_FACE / 2.0))
        .box(WIN_W, FRAME_DEPTH, FRAME_FACE)
    )
    return left_jamb.union(right_jamb).union(sill).union(head)


# ---------------------------------------------------------------------------
# Parameterized lattice frame and glass geometry (CadQuery)
# ---------------------------------------------------------------------------

def _build_lattice_frame(
    w: float, h: float, rail: float, depth: float,
    n_cols: int, n_rows: int, muntin_w: float,
) -> cq.Workplane:
    """Build a perimeter frame with an n_cols x n_rows muntin grid.
    Local: X from -w/2 to +w/2, Z from 0 to h, Y centered at 0."""
    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, h / 2.0))
        .box(w, depth, h)
    )

    in_x0, in_x1 = -w / 2.0 + rail, w / 2.0 - rail
    in_z0, in_z1 = rail, h - rail
    inner_w = in_x1 - in_x0
    inner_h = in_z1 - in_z0

    col_lines = [in_x0 + (i + 1) * inner_w / n_cols for i in range(n_cols - 1)]
    row_lines = [in_z0 + (j + 1) * inner_h / n_rows for j in range(n_rows - 1)]

    x_edges = [in_x0] + col_lines + [in_x1]
    z_edges = [in_z0] + row_lines + [in_z1]
    half_m = muntin_w / 2.0

    sash = outer
    for ci in range(n_cols):
        for ri in range(n_rows):
            lx0 = x_edges[ci] + (half_m if ci > 0 else 0.0)
            lx1 = x_edges[ci + 1] - (half_m if ci < n_cols - 1 else 0.0)
            lz0 = z_edges[ri] + (half_m if ri > 0 else 0.0)
            lz1 = z_edges[ri + 1] - (half_m if ri < n_rows - 1 else 0.0)
            lite = (
                cq.Workplane("XY")
                .transformed(offset=((lx0 + lx1) / 2.0, 0.0, (lz0 + lz1) / 2.0))
                .box(lx1 - lx0, depth + 0.02, lz1 - lz0)
            )
            sash = sash.cut(lite)
    return sash


def _build_lattice_glass(
    w: float, h: float, rail: float,
    n_cols: int, n_rows: int, muntin_w: float, glass_t: float,
) -> cq.Workplane:
    """Glass panes filling the lite openings, rebated under the muntin/rail
    lips so the glass reads as captured, not floating."""
    rebate = 0.004

    in_x0, in_x1 = -w / 2.0 + rail, w / 2.0 - rail
    in_z0, in_z1 = rail, h - rail
    inner_w = in_x1 - in_x0
    inner_h = in_z1 - in_z0

    col_lines = [in_x0 + (i + 1) * inner_w / n_cols for i in range(n_cols - 1)]
    row_lines = [in_z0 + (j + 1) * inner_h / n_rows for j in range(n_rows - 1)]

    x_edges = [in_x0] + col_lines + [in_x1]
    z_edges = [in_z0] + row_lines + [in_z1]
    half_m = muntin_w / 2.0

    panes = None
    for ci in range(n_cols):
        for ri in range(n_rows):
            lx0 = x_edges[ci] + (half_m if ci > 0 else 0.0) - rebate
            lx1 = x_edges[ci + 1] - (half_m if ci < n_cols - 1 else 0.0) + rebate
            lz0 = z_edges[ri] + (half_m if ri > 0 else 0.0) - rebate
            lz1 = z_edges[ri + 1] - (half_m if ri < n_rows - 1 else 0.0) + rebate
            pane = (
                cq.Workplane("XY")
                .transformed(offset=((lx0 + lx1) / 2.0, 0.0, (lz0 + lz1) / 2.0))
                .box(lx1 - lx0, glass_t, lz1 - lz0)
            )
            panes = pane if panes is None else panes.union(pane)
    return panes


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_panel_slider_window")

    model.material("frame", rgba=FRAME_RGBA)
    model.material("sash", rgba=SASH_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("hardware", rgba=HARDWARE_RGBA)
    model.material("roller", rgba=ROLLER_RGBA)

    # --- Frame (root) with integrated fixed center pane ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame_shell"),
        material="frame",
        name="frame_shell",
    )
    # Fixed center pane is part of the frame assembly.
    frame.visual(
        mesh_from_cadquery(
            _build_lattice_frame(CP_W, CP_H, CP_RAIL, CP_DEPTH, CP_COLS, CP_ROWS, MUNTIN_W),
            "center_pane_frame",
        ),
        origin=Origin(xyz=(0.0, CENTER_Y, CP_BOTTOM_Z)),
        material="sash",
        name="center_pane_frame",
    )
    frame.visual(
        mesh_from_cadquery(
            _build_lattice_glass(CP_W, CP_H, CP_RAIL, CP_COLS, CP_ROWS, MUNTIN_W, GLASS_T),
            "center_pane_glass",
        ),
        origin=Origin(xyz=(0.0, CENTER_Y, CP_BOTTOM_Z)),
        material="glass",
        name="center_pane_glass",
    )

    # --- Left sliding sash ---
    left = model.part("left_sash")
    left.visual(
        mesh_from_cadquery(
            _build_lattice_frame(SASH_W, SASH_H, SASH_RAIL, SASH_DEPTH,
                                 SASH_COLS, SASH_ROWS, MUNTIN_W),
            "left_sash_frame",
        ),
        material="sash",
        name="left_sash_frame",
    )
    left.visual(
        mesh_from_cadquery(
            _build_lattice_glass(SASH_W, SASH_H, SASH_RAIL,
                                 SASH_COLS, SASH_ROWS, MUNTIN_W, GLASS_T),
            "left_sash_glass",
        ),
        material="glass",
        name="left_sash_glass",
    )
    # Two roller blocks at the bottom of the left sash.
    roller_z = ROLLER_SIZE[2] / 2.0 - 0.005
    left.visual(
        Box(ROLLER_SIZE),
        origin=Origin(xyz=(-SASH_W * 0.36, 0.0, roller_z)),
        material="roller",
        name="left_roller_0",
    )
    left.visual(
        Box(ROLLER_SIZE),
        origin=Origin(xyz=(SASH_W * 0.36, 0.0, roller_z)),
        material="roller",
        name="left_roller_1",
    )

    # --- Right sliding sash ---
    right = model.part("right_sash")
    right.visual(
        mesh_from_cadquery(
            _build_lattice_frame(SASH_W, SASH_H, SASH_RAIL, SASH_DEPTH,
                                 SASH_COLS, SASH_ROWS, MUNTIN_W),
            "right_sash_frame",
        ),
        material="sash",
        name="right_sash_frame",
    )
    right.visual(
        mesh_from_cadquery(
            _build_lattice_glass(SASH_W, SASH_H, SASH_RAIL,
                                 SASH_COLS, SASH_ROWS, MUNTIN_W, GLASS_T),
            "right_sash_glass",
        ),
        material="glass",
        name="right_sash_glass",
    )
    # Two roller blocks at the bottom of the right sash.
    right.visual(
        Box(ROLLER_SIZE),
        origin=Origin(xyz=(-SASH_W * 0.36, 0.0, roller_z)),
        material="roller",
        name="right_roller_0",
    )
    right.visual(
        Box(ROLLER_SIZE),
        origin=Origin(xyz=(SASH_W * 0.36, 0.0, roller_z)),
        material="roller",
        name="right_roller_1",
    )

    # --- Tilt-in latches (one per sash) ---
    left_latch = model.part("left_latch")
    left_latch.visual(
        Box(LATCH_BODY),
        origin=Origin(xyz=(0.0, 0.0, LATCH_BODY[2] / 2.0)),
        material="hardware",
        name="left_latch_body",
    )

    right_latch = model.part("right_latch")
    right_latch.visual(
        Box(LATCH_BODY),
        origin=Origin(xyz=(0.0, 0.0, LATCH_BODY[2] / 2.0)),
        material="hardware",
        name="right_latch_body",
    )

    # ----- Articulations -----

    # LEFT sash: PRISMATIC, axis (1,0,0), positive q slides RIGHT (opens).
    model.articulation(
        "frame_to_left_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="left_sash",
        origin=Origin(xyz=(LEFT_SASH_X, SASH_Y, SASH_BOTTOM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=0.25, lower=0.0, upper=SASH_TRAVEL,
        ),
    )

    # RIGHT sash: PRISMATIC, axis (-1,0,0), positive q slides LEFT (opens).
    model.articulation(
        "frame_to_right_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="right_sash",
        origin=Origin(xyz=(RIGHT_SASH_X, SASH_Y, SASH_BOTTOM_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=0.25, lower=0.0, upper=SASH_TRAVEL,
        ),
    )

    # LEFT latch: REVOLUTE on the left sash top rail.
    latch_y_in_sash = -(SASH_DEPTH / 2.0 + 0.003)
    latch_z_in_sash = SASH_H - SASH_RAIL * 0.60
    model.articulation(
        "left_sash_to_latch",
        ArticulationType.REVOLUTE,
        parent="left_sash",
        child="left_latch",
        origin=Origin(xyz=(0.0, latch_y_in_sash, latch_z_in_sash)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=0.0, upper=1.2,
        ),
    )

    # RIGHT latch: REVOLUTE on the right sash top rail.
    model.articulation(
        "right_sash_to_latch",
        ArticulationType.REVOLUTE,
        parent="right_sash",
        child="right_latch",
        origin=Origin(xyz=(0.0, latch_y_in_sash, latch_z_in_sash)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=0.0, upper=1.2,
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
    left_latch = object_model.get_part("left_latch")
    right_latch = object_model.get_part("right_latch")

    j_left = object_model.get_articulation("frame_to_left_sash")
    j_right = object_model.get_articulation("frame_to_right_sash")
    j_l_latch = object_model.get_articulation("left_sash_to_latch")
    j_r_latch = object_model.get_articulation("right_sash_to_latch")

    # --- Intentional overlap allowances ---

    # Glass panes rebated under sash muntin/rail lips.
    for sash_name in ("left_sash", "right_sash"):
        ctx.allow_overlap(
            sash_name, sash_name,
            elem_a=f"{sash_name}_glass",
            elem_b=f"{sash_name}_frame",
            reason="Glass panes are rebated under the sash rails/muntins (captured glazing).",
        )
    # Center pane glass rebated under center pane frame rails/muntins.
    ctx.allow_overlap(
        "frame", "frame",
        elem_a="center_pane_glass",
        elem_b="center_pane_frame",
        reason="Center pane glass is rebated under the center pane frame rails/muntins.",
    )
    # Center pane frame seated between sill and head (fixed installation).
    ctx.allow_overlap(
        "frame", "frame",
        elem_a="center_pane_frame",
        elem_b="frame_shell",
        reason="Fixed center pane is installed (seated) between the sill and head of the frame assembly.",
    )

    # Sashes contact sill and head (face-to-face seated fit).
    ctx.allow_overlap(
        "frame", "left_sash",
        reason="Left sash is seated on the sill and head track surfaces of the frame.",
    )
    ctx.allow_overlap(
        "frame", "right_sash",
        reason="Right sash is seated on the sill and head track surfaces of the frame.",
    )

    # Roller blocks seated into sash bottom rails.
    for sash_name, prefix in [("left_sash", "left"), ("right_sash", "right")]:
        for rname in (f"{prefix}_roller_0", f"{prefix}_roller_1"):
            ctx.allow_overlap(
                sash_name, sash_name,
                elem_a=rname,
                elem_b=f"{sash_name}_frame",
                reason="Roller block is seated into the sash bottom rail.",
            )

    # Tilt-in latches mounted on sash top rails.
    ctx.allow_overlap(
        "left_sash", "left_latch",
        reason="Left tilt-in latch is mounted onto the left sash top rail.",
    )
    ctx.allow_overlap(
        "right_sash", "right_latch",
        reason="Right tilt-in latch is mounted onto the right sash top rail.",
    )

    # --- Structural / layout checks ---

    # Center pane is wider than each side sash.
    ctx.check(
        "center pane wider than side sashes",
        CP_W > SASH_W + 0.05,
        details=f"center_w={CP_W:.3f}, sash_w={SASH_W:.3f}",
    )

    # Joint types: prismatic sashes, revolute latches.
    ctx.check(
        "left sash articulation is prismatic",
        j_left.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={j_left.articulation_type}",
    )
    ctx.check(
        "right sash articulation is prismatic",
        j_right.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={j_right.articulation_type}",
    )
    ctx.check(
        "left latch articulation is revolute",
        j_l_latch.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={j_l_latch.articulation_type}",
    )
    ctx.check(
        "right latch articulation is revolute",
        j_r_latch.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={j_r_latch.articulation_type}",
    )

    # --- Closed pose (q=0): all panels seated, window reads shut ---
    with ctx.pose({j_left: 0.0, j_right: 0.0}):
        f_aabb = ctx.part_world_aabb(frame)
        l_aabb = ctx.part_world_aabb(left)
        r_aabb = ctx.part_world_aabb(right)

        # Frame spans full window width and height.
        frame_w = f_aabb[1][0] - f_aabb[0][0]
        frame_h = f_aabb[1][2] - f_aabb[0][2]
        ctx.check(
            "frame spans full window width",
            frame_w > WIN_W - 0.01,
            details=f"frame_w={frame_w:.3f}",
        )
        ctx.check(
            "window stands upright",
            abs(f_aabb[0][2]) < 0.01 and frame_h > 0.9,
            details=f"frame z=({f_aabb[0][2]:.3f},{f_aabb[1][2]:.3f})",
        )

        # Sashes within frame opening in X.
        ctx.check(
            "left sash within frame width",
            l_aabb[0][0] >= f_aabb[0][0] - 0.001 and l_aabb[1][0] <= f_aabb[1][0] + 0.001,
            details=f"left x=({l_aabb[0][0]:.3f},{l_aabb[1][0]:.3f})",
        )
        ctx.check(
            "right sash within frame width",
            r_aabb[0][0] >= f_aabb[0][0] - 0.001 and r_aabb[1][0] <= f_aabb[1][0] + 0.001,
            details=f"right x=({r_aabb[0][0]:.3f},{r_aabb[1][0]:.3f})",
        )

        # Left sash is to the left of right sash (horizontal layout).
        l_cx = (l_aabb[0][0] + l_aabb[1][0]) / 2.0
        r_cx = (r_aabb[0][0] + r_aabb[1][0]) / 2.0
        ctx.check(
            "left sash is left of right sash",
            l_cx < r_cx - 0.2,
            details=f"left_cx={l_cx:.3f}, right_cx={r_cx:.3f}",
        )

        # Center pane visual exists on the frame and is centered.
        cp_aabb = ctx.part_element_world_aabb(frame, elem="center_pane_frame")
        ctx.check(
            "center pane frame visual exists",
            cp_aabb is not None,
            details="center_pane_frame aabb is None",
        )
        if cp_aabb is not None:
            cp_cx = (cp_aabb[0][0] + cp_aabb[1][0]) / 2.0
            ctx.check(
                "center pane centered in frame",
                abs(cp_cx) < 0.05,
                details=f"center_pane cx={cp_cx:.3f}",
            )
            cp_w = cp_aabb[1][0] - cp_aabb[0][0]
            ctx.check(
                "center pane is the widest panel",
                cp_w > SASH_W + 0.05,
                details=f"cp_w={cp_w:.3f}, sash_w={SASH_W:.3f}",
            )

        # Sashes span the full opening height.
        ctx.expect_overlap(
            left, frame, axes="z", min_overlap=0.8,
            name="left sash spans most of the frame height",
        )

        rest_l_cx = l_cx
        rest_r_cx = r_cx

    # --- HERO: left sash slides RIGHT (opens) ---
    travel = SASH_TRAVEL * 0.85
    with ctx.pose({j_left: travel}):
        op = ctx.part_world_aabb(left)
        op_cx = (op[0][0] + op[1][0]) / 2.0
        ctx.check(
            "left sash slides right when opened",
            op_cx > rest_l_cx + travel * 0.8,
            details=f"rest_cx={rest_l_cx:.3f}, opened_cx={op_cx:.3f}, travel={travel:.3f}",
        )
        ctx.expect_overlap(
            left, frame, axes="z", min_overlap=0.3,
            name="left sash retained in frame when open",
        )

    # --- HERO: right sash slides LEFT (opens) ---
    with ctx.pose({j_right: travel}):
        op = ctx.part_world_aabb(right)
        op_cx = (op[0][0] + op[1][0]) / 2.0
        ctx.check(
            "right sash slides left when opened",
            op_cx < rest_r_cx - travel * 0.8,
            details=f"rest_cx={rest_r_cx:.3f}, opened_cx={op_cx:.3f}, travel={travel:.3f}",
        )
        ctx.expect_overlap(
            right, frame, axes="z", min_overlap=0.3,
            name="right sash retained in frame when open",
        )

    # --- Tilt-in latches pivot ---
    with ctx.pose({j_l_latch: 0.8}):
        ll_aabb = ctx.part_world_aabb(left_latch)
        ctx.check(
            "left latch exists and is posed",
            ll_aabb is not None and (ll_aabb[1][2] - ll_aabb[0][2]) > 0.005,
            details=f"left latch aabb={ll_aabb}",
        )

    with ctx.pose({j_r_latch: 0.8}):
        rl_aabb = ctx.part_world_aabb(right_latch)
        ctx.check(
            "right latch exists and is posed",
            rl_aabb is not None and (rl_aabb[1][2] - rl_aabb[0][2]) > 0.005,
            details=f"right latch aabb={rl_aabb}",
        )

    # --- Roller blocks exist at sash bottoms ---
    for sash_part, r0, r1 in [
        (left, "left_roller_0", "left_roller_1"),
        (right, "right_roller_0", "right_roller_1"),
    ]:
        for rname in (r0, r1):
            raabb = ctx.part_element_world_aabb(sash_part, elem=rname)
            ctx.check(
                f"{rname} exists on sash",
                raabb is not None,
                details=f"{rname} aabb is None",
            )
        # Rollers are near the bottom of the sash.
        sash_aabb = ctx.part_world_aabb(sash_part)
        r0_aabb = ctx.part_element_world_aabb(sash_part, elem=r0)
        if sash_aabb is not None and r0_aabb is not None:
            ctx.check(
                f"{r0} near sash bottom",
                r0_aabb[0][2] < sash_aabb[0][2] + 0.03,
                details=f"roller_bot={r0_aabb[0][2]:.3f}, sash_bot={sash_aabb[0][2]:.3f}",
            )

    return ctx.report()


object_model = build_object_model()
