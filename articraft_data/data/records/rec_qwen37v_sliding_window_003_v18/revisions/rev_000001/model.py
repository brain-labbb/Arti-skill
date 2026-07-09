from __future__ import annotations

# Horizontal sliding window: corner-lift slider with a small vent panel.
# Forked from double-hung sash window — same white frame family, but now one
# fixed sash (left) and one sliding sash (right) that translates horizontally
# on a prismatic joint. A small fixed vent panel sits above the main sash area.
# Two tiny roller blocks at the bottom of the sliding sash ride in the sill
# track. A corner-lift handle sits at the bottom interior corner of the slider.
#
# Coordinate convention:
#   +Z is up. Window stands vertically: height along +Z, width along X, frame
#   depth along Y. The sill sits at z=0; the head is at z=WIN_H.

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

# Vent panel at top of the opening
VENT_H = 0.170        # vent panel opening height
MULLION_H = 0.038     # horizontal mullion between vent and main sash area

# Main sash area (below the mullion)
MAIN_H = OPEN_H - VENT_H - MULLION_H   # main sash opening height
MAIN_Z0 = FRAME_FACE                    # bottom of main area (top of sill)
MAIN_Z1 = MAIN_Z0 + MAIN_H             # top of main area (bottom of mullion)

# Vent area (above the mullion)
VENT_Z0 = MAIN_Z1 + MULLION_H
VENT_Z1 = VENT_Z0 + VENT_H             # should ≈ WIN_H - FRAME_FACE

# Sash geometry (each sash is about half the opening width, full main height)
SASH_W = OPEN_W / 2.0 - 0.012          # sash width with running clearance
SASH_H = MAIN_H - 0.008                # sash height with top/bottom clearance
SASH_RAIL = 0.050                      # sash perimeter member width
SASH_DEPTH = 0.034                     # sash thickness (Y)
GLASS_T = 0.006                        # glazing thickness (Y)

# Y planes: fixed sash is exterior (+Y), sliding sash is interior (-Y)
SASH_Y_GAP = 0.018
FIXED_SASH_Y = +SASH_Y_GAP
SLIDING_SASH_Y = -SASH_Y_GAP

# Closed-pose sash center X positions (world)
FIXED_SASH_X = OPEN_X0 + SASH_W / 2.0           # left half center
SLIDING_SASH_X = OPEN_X1 - SASH_W / 2.0         # right half center (closed)
SASH_BOTTOM_Z = MAIN_Z0 + 0.004                 # small sill clearance

# Muntin grid: 2 columns x 3 rows of lites per sash -> 6 lites each
MUNTIN_W = 0.020
N_COLS = 2
N_ROWS = 3

# Track grooves in sill and mullion (horizontal channels)
TRACK_GROOVE_W = SASH_DEPTH + 0.006   # Y width of groove
TRACK_GROOVE_D = 0.012                 # Z depth of groove into sill/mullion

# Roller blocks at bottom of sliding sash
ROLLER_SIZE = (0.028, 0.020, 0.012)   # (X, Y, Z)

# Corner-lift handle at bottom of sliding sash
HANDLE_SIZE = (0.042, 0.016, 0.032)   # (X, Y, Z)

# Vent panel muntins (3 lites across)
VENT_MUNTIN_W = 0.018
VENT_N_COLS = 3

# Sliding travel (how far the sash slides left, behind the fixed sash)
SLIDE_TRAVEL = SASH_W * 0.88

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

FRAME_RGBA = (0.945, 0.945, 0.945, 1.0)   # white painted frame
SASH_RGBA = (0.965, 0.965, 0.965, 1.0)    # white sash (slightly brighter)
GLASS_RGBA = (0.30, 0.36, 0.42, 0.34)     # cool dark-tinted glass
ROLLER_RGBA = (0.22, 0.22, 0.24, 1.0)     # dark nylon rollers
HANDLE_RGBA = (0.82, 0.83, 0.85, 1.0)     # brushed metal handle


# ---------------------------------------------------------------------------
# Frame geometry (CadQuery)
# ---------------------------------------------------------------------------

def _build_frame_shape() -> cq.Workplane:
    """White outer frame: perimeter slab with main sash opening and vent panel
    opening cut out, leaving head, sill, two jambs, and a horizontal mullion.
    Horizontal track grooves are cut into the sill top and mullion bottom.
    """
    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, WIN_H / 2.0))
        .box(WIN_W, FRAME_DEPTH, WIN_H)
    )

    # Cut main sash opening
    main_cut = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, (MAIN_Z0 + MAIN_Z1) / 2.0))
        .box(OPEN_W, FRAME_DEPTH + 0.02, MAIN_H)
    )
    frame = outer.cut(main_cut)

    # Cut vent panel opening
    vent_cut = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, (VENT_Z0 + VENT_Z1) / 2.0))
        .box(OPEN_W, FRAME_DEPTH + 0.02, VENT_H)
    )
    frame = frame.cut(vent_cut)

    # Horizontal track grooves in sill top face (at MAIN_Z0, cutting downward)
    for track_y in (FIXED_SASH_Y, SLIDING_SASH_Y):
        groove = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, track_y, MAIN_Z0 - TRACK_GROOVE_D / 2.0 + 0.001))
            .box(OPEN_W - 0.010, TRACK_GROOVE_W, TRACK_GROOVE_D + 0.002)
        )
        frame = frame.cut(groove)

    # Horizontal track grooves in mullion bottom face (at MAIN_Z1, cutting upward)
    for track_y in (FIXED_SASH_Y, SLIDING_SASH_Y):
        groove = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, track_y, MAIN_Z1 + TRACK_GROOVE_D / 2.0 - 0.001))
            .box(OPEN_W - 0.010, TRACK_GROOVE_W, TRACK_GROOVE_D + 0.002)
        )
        frame = frame.cut(groove)

    return frame


def _build_vent_glass_shape() -> cq.Workplane:
    """Vent panel glass: three panes divided by vertical muntin bars."""
    rebate = 0.004
    pane_total_w = OPEN_W - 2 * VENT_MUNTIN_W
    pane_w = pane_total_w / VENT_N_COLS
    pane_h = VENT_H - rebate * 2

    panes = None
    for i in range(VENT_N_COLS):
        cx = OPEN_X0 + VENT_MUNTIN_W + pane_w / 2.0 + i * (pane_w + VENT_MUNTIN_W)
        cz = (VENT_Z0 + VENT_Z1) / 2.0
        pane = (
            cq.Workplane("XY")
            .transformed(offset=(cx, 0.0, cz))
            .box(pane_w + rebate, GLASS_T, pane_h + rebate)
        )
        panes = pane if panes is None else panes.union(pane)
    return panes


def _build_vent_muntin_shape() -> cq.Workplane:
    """Two vertical muntin bars dividing the vent panel into three lites."""
    bars = None
    pane_total_w = OPEN_W - 2 * VENT_MUNTIN_W
    pane_w = pane_total_w / VENT_N_COLS
    cz = (VENT_Z0 + VENT_Z1) / 2.0

    for i in range(VENT_N_COLS - 1):
        cx = OPEN_X0 + VENT_MUNTIN_W + pane_w * (i + 1) + VENT_MUNTIN_W / 2.0 * (2 * i + 1)
        # Simpler: center of muntin between pane i and pane i+1
        cx = OPEN_X0 + (i + 1) * pane_w + (i + 0.5) * VENT_MUNTIN_W + VENT_MUNTIN_W / 2.0
        bar = (
            cq.Workplane("XY")
            .transformed(offset=(cx, 0.0, cz))
            .box(VENT_MUNTIN_W, SASH_DEPTH * 0.7, VENT_H)
        )
        bars = bar if bars is None else bars.union(bar)
    return bars


# ---------------------------------------------------------------------------
# Sash frame geometry (CadQuery): perimeter ring + muntin grid
# ---------------------------------------------------------------------------

def _build_sash_frame_shape() -> cq.Workplane:
    """One sash: perimeter ring plus a muntin grid with lite openings cut.

    Local frame: X centered, Z from 0 (bottom rail) to SASH_H, Y centered.
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
    """Glass panes filling the lite openings, rebated under muntin lips."""
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
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_window_corner_lift")

    model.material("frame", rgba=FRAME_RGBA)
    model.material("sash", rgba=SASH_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("roller", rgba=ROLLER_RGBA)
    model.material("handle", rgba=HANDLE_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame_shell"),
        material="frame",
        name="frame_shell",
    )
    # Vent panel glass (fixed, part of frame)
    frame.visual(
        mesh_from_cadquery(_build_vent_glass_shape(), "vent_glass"),
        material="glass",
        name="vent_glass",
    )
    # Vent panel muntin bars
    frame.visual(
        mesh_from_cadquery(_build_vent_muntin_shape(), "vent_muntins"),
        material="frame",
        name="vent_muntins",
    )

    # --- Fixed sash (left) ---
    fixed_sash = model.part("fixed_sash")
    fixed_sash.visual(
        mesh_from_cadquery(_build_sash_frame_shape(), "fixed_sash_frame"),
        material="sash",
        name="fixed_sash_frame",
    )
    fixed_sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(), "fixed_sash_glass"),
        material="glass",
        name="fixed_sash_glass",
    )

    # --- Sliding sash (right) ---
    sliding_sash = model.part("sliding_sash")
    sliding_sash.visual(
        mesh_from_cadquery(_build_sash_frame_shape(), "sliding_sash_frame"),
        material="sash",
        name="sliding_sash_frame",
    )
    sliding_sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(), "sliding_sash_glass"),
        material="glass",
        name="sliding_sash_glass",
    )

    # Two roller blocks at bottom of sliding sash (one near each stile)
    roller_x_offset = SASH_W / 2.0 - ROLLER_SIZE[0] / 2.0 - 0.010
    roller_z = -ROLLER_SIZE[2] / 2.0 + 0.002  # slightly below sash bottom
    for i, sign in enumerate((+1.0, -1.0)):
        sliding_sash.visual(
            Box(ROLLER_SIZE),
            origin=Origin(xyz=(sign * roller_x_offset, 0.0, roller_z)),
            material="roller",
            name=f"roller_{i}",
        )

    # Corner-lift handle at the bottom-left corner (interior face) of sliding sash
    handle_x = -SASH_W / 2.0 + HANDLE_SIZE[0] / 2.0 + 0.015
    handle_y = -(SASH_DEPTH / 2.0 + HANDLE_SIZE[1] / 2.0 - 0.004)
    handle_z = SASH_RAIL / 2.0  # centered on bottom rail
    sliding_sash.visual(
        Box(HANDLE_SIZE),
        origin=Origin(xyz=(handle_x, handle_y, handle_z)),
        material="handle",
        name="corner_lift_handle",
    )

    # ----- Articulations -----

    # Fixed sash: rigidly mounted in the left side of the frame
    model.articulation(
        "frame_to_fixed_sash",
        ArticulationType.FIXED,
        parent="frame",
        child="fixed_sash",
        origin=Origin(xyz=(FIXED_SASH_X, FIXED_SASH_Y, SASH_BOTTOM_Z)),
    )

    # Sliding sash: prismatic, axis (-1,0,0), positive q slides LEFT (opens)
    model.articulation(
        "frame_to_sliding_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="sliding_sash",
        origin=Origin(xyz=(SLIDING_SASH_X, SLIDING_SASH_Y, SASH_BOTTOM_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0, velocity=0.30, lower=0.0, upper=SLIDE_TRAVEL
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
    j_fixed = object_model.get_articulation("frame_to_fixed_sash")
    j_slide = object_model.get_articulation("frame_to_sliding_sash")

    # --- Intentional overlaps ---
    # Glass panes tuck under sash muntin/rail lips (captured glass)
    for sash_name in ("fixed_sash", "sliding_sash"):
        ctx.allow_overlap(
            sash_name, sash_name,
            elem_a=f"{sash_name}_glass",
            elem_b=f"{sash_name}_frame",
            reason="Glass panes are rebated under the sash rails/muntins so they read as captured, not floating.",
        )

    # Sashes ride in the horizontal track grooves (sill + mullion)
    ctx.allow_overlap(
        "frame", "fixed_sash",
        reason="Fixed sash stiles are retained in the sill and mullion track grooves.",
    )
    ctx.allow_overlap(
        "frame", "sliding_sash",
        reason="Sliding sash stiles ride in the sill and mullion track grooves.",
    )

    # Roller blocks are mounted under the sliding sash bottom rail
    ctx.allow_overlap(
        "sliding_sash", "sliding_sash",
        elem_a="roller_0",
        elem_b="sliding_sash_frame",
        reason="Roller blocks are mounted against the sliding sash bottom rail.",
    )
    ctx.allow_overlap(
        "sliding_sash", "sliding_sash",
        elem_a="roller_1",
        elem_b="sliding_sash_frame",
        reason="Roller blocks are mounted against the sliding sash bottom rail.",
    )

    # Corner-lift handle is seated onto the sliding sash
    ctx.allow_overlap(
        "sliding_sash", "sliding_sash",
        elem_a="corner_lift_handle",
        elem_b="sliding_sash_frame",
        reason="Corner-lift handle is mounted (seated) onto the sliding sash bottom rail.",
    )

    # Vent glass is captured in the frame vent opening
    ctx.allow_overlap(
        "frame", "frame",
        elem_a="vent_glass",
        elem_b="frame_shell",
        reason="Vent glass is rebated into the frame vent opening.",
    )
    ctx.allow_overlap(
        "frame", "frame",
        elem_a="vent_muntins",
        elem_b="frame_shell",
        reason="Vent muntin bars span the frame vent opening.",
    )

    # --- Closed pose (q=0): sashes side by side, window reads shut ---
    with ctx.pose({j_slide: 0.0}):
        f_aabb = ctx.part_world_aabb(frame)
        fx_aabb = ctx.part_world_aabb(fixed)
        sl_aabb = ctx.part_world_aabb(slider)

        # Frame is tallest element
        ctx.check(
            "frame spans full window height",
            f_aabb[1][2] > 1.0 and abs(f_aabb[0][2]) < 0.01,
            details=f"frame z=({f_aabb[0][2]:.3f}, {f_aabb[1][2]:.3f})",
        )

        # Fixed sash is to the left of sliding sash at closed pose
        fx_cx = (fx_aabb[0][0] + fx_aabb[1][0]) / 2.0
        sl_cx = (sl_aabb[0][0] + sl_aabb[1][0]) / 2.0
        ctx.check(
            "fixed sash left of sliding sash at rest",
            fx_cx < sl_cx - 0.10,
            details=f"fixed_cx={fx_cx:.3f}, sliding_cx={sl_cx:.3f}",
        )

        # Both sashes are within the frame width
        ctx.check(
            "sashes within frame width at rest",
            fx_aabb[0][0] > f_aabb[0][0] and fx_aabb[1][0] < f_aabb[1][0]
            and sl_aabb[0][0] > f_aabb[0][0] and sl_aabb[1][0] < f_aabb[1][0],
            details=f"fixed x=({fx_aabb[0][0]:.3f},{fx_aabb[1][0]:.3f}), "
                    f"slider x=({sl_aabb[0][0]:.3f},{sl_aabb[1][0]:.3f})",
        )

        # Sashes occupy the main area (below the mullion)
        ctx.check(
            "sashes in main area below mullion",
            fx_aabb[1][2] < MAIN_Z1 + 0.02 and sl_aabb[1][2] < MAIN_Z1 + 0.02,
            details=f"fixed_top={fx_aabb[1][2]:.3f}, slider_top={sl_aabb[1][2]:.3f}, mullion_top={MAIN_Z1:.3f}",
        )

        # The two sashes are in offset Y planes
        fx_cy = (fx_aabb[0][1] + fx_aabb[1][1]) / 2.0
        sl_cy = (sl_aabb[0][1] + sl_aabb[1][1]) / 2.0
        ctx.check(
            "sashes ride in offset Y planes",
            abs(fx_cy - sl_cy) > 0.015,
            details=f"fixed_cy={fx_cy:.3f}, sliding_cy={sl_cy:.3f}",
        )

        rest_sl_cx = sl_cx

    # --- HERO: sliding sash moves LEFT (opens) ---
    travel = SLIDE_TRAVEL * 0.85
    with ctx.pose({j_slide: travel}):
        op = ctx.part_world_aabb(slider)
        op_cx = (op[0][0] + op[1][0]) / 2.0
        ctx.check(
            "sliding sash moves left when opened",
            op_cx < rest_sl_cx - travel * 0.8,
            details=f"rest_cx={rest_sl_cx:.3f}, opened_cx={op_cx:.3f}, travel={travel:.3f}",
        )
        # Sliding sash still overlaps the frame in Z (retained vertically)
        ctx.expect_overlap(
            slider, frame, axes="z", min_overlap=0.5,
            name="sliding sash retained vertically when open",
        )

    # --- Roller blocks exist and are at the bottom of the sliding sash ---
    roller0_aabb = ctx.part_element_world_aabb(slider, elem="roller_0")
    roller1_aabb = ctx.part_element_world_aabb(slider, elem="roller_1")
    slider_aabb = ctx.part_world_aabb(slider)
    if roller0_aabb is not None and roller1_aabb is not None and slider_aabb is not None:
        # Rollers are near the bottom of the sash
        ctx.check(
            "roller blocks near sash bottom",
            roller0_aabb[0][2] < slider_aabb[0][2] + 0.03
            and roller1_aabb[0][2] < slider_aabb[0][2] + 0.03,
            details=f"roller0_bot={roller0_aabb[0][2]:.3f}, roller1_bot={roller1_aabb[0][2]:.3f}, "
                    f"sash_bot={slider_aabb[0][2]:.3f}",
        )
        # Two rollers are separated in X
        r0_cx = (roller0_aabb[0][0] + roller0_aabb[1][0]) / 2.0
        r1_cx = (roller1_aabb[0][0] + roller1_aabb[1][0]) / 2.0
        ctx.check(
            "two roller blocks separated in X",
            abs(r0_cx - r1_cx) > 0.10,
            details=f"roller0_cx={r0_cx:.3f}, roller1_cx={r1_cx:.3f}",
        )

    # --- Corner-lift handle exists on the sliding sash ---
    handle_aabb = ctx.part_element_world_aabb(slider, elem="corner_lift_handle")
    if handle_aabb is not None and slider_aabb is not None:
        # Handle is near the bottom of the sash
        handle_cz = (handle_aabb[0][2] + handle_aabb[1][2]) / 2.0
        ctx.check(
            "corner-lift handle near sash bottom",
            handle_cz < slider_aabb[0][2] + SASH_RAIL + 0.02,
            details=f"handle_cz={handle_cz:.3f}, sash_bot={slider_aabb[0][2]:.3f}",
        )

    # --- Vent panel exists at the top of the frame ---
    vent_aabb = ctx.part_element_world_aabb(frame, elem="vent_glass")
    if vent_aabb is not None:
        ctx.check(
            "vent panel in upper portion of window",
            vent_aabb[0][2] > MAIN_Z1 - 0.01,
            details=f"vent_bot_z={vent_aabb[0][2]:.3f}, mullion_top={MAIN_Z1:.3f}",
        )

    # --- Joint type check: sliding sash has a prismatic joint ---
    slide_info = j_slide
    ctx.check(
        "sliding sash has prismatic articulation",
        slide_info.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={slide_info.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
