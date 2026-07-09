from __future__ import annotations

# Sliding window variant with transom: two-panel horizontal sliding window,
# white vinyl frame, one fixed sash (left) + one sliding sash (right) below a
# narrow fixed transom panel. An insect screen slides independently on a
# shallow prismatic track. Two roller blocks at the bottom of the moving sash.
# A visible overlap stile (astragal) marks where the panes cross.
#
# Coordinate convention:
#   +Z up. Window stands vertically.
#     width  -> X
#     height -> Z   (sill near z=0)
#     frame depth / slide-normal -> Y
#   Glass plane is the X-Z plane. q=0 reads SHUT.

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

TOTAL_W = 1.52
TOTAL_H = 1.72

FRAME_FACE = 0.085         # outer frame member face width
FRAME_DEPTH = 0.140        # deep box section along Y

MEETING_OVERLAP = 0.040    # sash stile overlap at center

SASH_FACE = 0.075          # sash rail/stile face width
SASH_DEPTH = 0.060         # sash depth along Y
GLASS_T = 0.008            # glazing thickness

# Y layout: frame box centered on y=0
FIXED_SASH_Y = -0.028      # rear glazing plane
SLIDE_SASH_Y = 0.044       # sliding sash proud toward +Y (front track)

REBATE = 0.005             # glass tucks under sash lip

# Transom
TRANSOM_H = 0.28           # transom opening height
TRANSOM_RAIL_H = 0.055     # horizontal divider rail thickness
TRANSOM_FRAME = 0.040      # transom frame ring width

# Latch hardware
LATCH_PLATE_W = 0.028
LATCH_PLATE_H = 0.075
LATCH_PLATE_T = 0.010
LATCH_LEVER_LEN = 0.045
LATCH_LEVER_R = 0.006

# Roller blocks
ROLLER_W = 0.025
ROLLER_H = 0.016
ROLLER_D = 0.020

# Overlap stile (astragal)
ASTRAGAL_W = 0.016
ASTRAGAL_D = 0.014

# Insect screen
SCREEN_FRAME_W = 0.028
SCREEN_DEPTH = 0.006
SCREEN_MESH_T = 0.001

# Materials
VINYL_RGBA = (0.94, 0.95, 0.96, 1.0)
GLASS_RGBA = (0.52, 0.60, 0.66, 0.30)
METAL_RGBA = (0.74, 0.76, 0.79, 1.0)
SCREEN_RGBA = (0.30, 0.32, 0.30, 0.45)
ROLLER_RGBA = (0.25, 0.25, 0.27, 1.0)

# ---------------------------------------------------------------------------
# Derived layout
# ---------------------------------------------------------------------------

HALF_W = TOTAL_W / 2.0
INNER_X0 = -HALF_W + FRAME_FACE
INNER_X1 = HALF_W - FRAME_FACE
INNER_Z0 = FRAME_FACE
INNER_Z1 = TOTAL_H - FRAME_FACE
INNER_W = INNER_X1 - INNER_X0
INNER_H = INNER_Z1 - INNER_Z0

# Transom and main zones
MAIN_Z0 = INNER_Z0
MAIN_Z1 = INNER_Z1 - TRANSOM_H - TRANSOM_RAIL_H
MAIN_H = MAIN_Z1 - MAIN_Z0
MID_MAIN_Z = (MAIN_Z0 + MAIN_Z1) / 2.0

TRANSOM_Z0 = INNER_Z1 - TRANSOM_H
TRANSOM_Z1 = INNER_Z1
TRANSOM_CZ = (TRANSOM_Z0 + TRANSOM_Z1) / 2.0

# Sash openings fit within main zone, sash outer = MAIN_H
SASH_OPENING_W = (INNER_W + MEETING_OVERLAP) / 2.0
SASH_OPENING_H = MAIN_H - 2 * SASH_FACE + 0.010  # small track engagement

FIXED_OPEN_CX = INNER_X0 + SASH_OPENING_W / 2.0
SLIDE_OPEN_CX = INNER_X1 - SASH_OPENING_W / 2.0

# Transom opening
TRANSOM_OPEN_W = INNER_W - 2 * TRANSOM_FRAME
TRANSOM_OPEN_H = TRANSOM_H - 2 * TRANSOM_FRAME

# Screen
SASH_OUT_W = SASH_OPENING_W + 2 * SASH_FACE
SCREEN_W = SASH_OPENING_W  # screen covers one opening width
SCREEN_H = MAIN_H - 2 * SASH_FACE  # fits within sash clear height
SCREEN_OPEN_W = SCREEN_W - 2 * SCREEN_FRAME_W
SCREEN_OPEN_H = SCREEN_H - 2 * SCREEN_FRAME_W
SCREEN_Y = FIXED_SASH_Y - SASH_DEPTH / 2.0 - 0.006 - SCREEN_DEPTH / 2.0
SCREEN_CX = SLIDE_OPEN_CX  # starts aligned with sliding sash (right side)
SCREEN_TRAVEL = INNER_W / 2.0 - SCREEN_W / 2.0 - 0.01  # slides to cover left opening


# ---------------------------------------------------------------------------
# Geometry helpers (CadQuery, meters)
# ---------------------------------------------------------------------------

def _slab(x0, x1, z0, z1, y_center, depth):
    return (
        cq.Workplane("XY")
        .transformed(offset=((x0 + x1) / 2.0, y_center, (z0 + z1) / 2.0))
        .box(x1 - x0, depth, z1 - z0)
    )


def _build_frame_shape():
    """Frame with transom: slab cut by transom opening (upper) and main sash
    opening (lower), leaving head, sill, jambs, and transom rail."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)
    cut_d = FRAME_DEPTH + 0.02
    transom_cut = _slab(INNER_X0, INNER_X1, TRANSOM_Z0, TRANSOM_Z1, 0.0, cut_d)
    main_cut = _slab(INNER_X0, INNER_X1, MAIN_Z0, MAIN_Z1, 0.0, cut_d)
    return outer.cut(transom_cut).cut(main_cut)


def _build_sash_shape():
    """Sash ring in local frame, centered at origin."""
    ow = SASH_OPENING_W
    oh = SASH_OPENING_H
    out_w = ow + 2 * SASH_FACE
    out_h = oh + 2 * SASH_FACE
    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, SASH_DEPTH)
    opening = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SASH_DEPTH + 0.02)
    return outer.cut(opening)


def _build_sash_glass_shape():
    ow = SASH_OPENING_W + 2 * REBATE
    oh = SASH_OPENING_H + 2 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


def _build_transom_shape():
    """Transom frame ring, centered at origin."""
    ow = TRANSOM_OPEN_W
    oh = TRANSOM_OPEN_H
    out_w = ow + 2 * TRANSOM_FRAME
    out_h = oh + 2 * TRANSOM_FRAME
    depth = SASH_DEPTH * 0.7
    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, depth)
    opening = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, depth + 0.02)
    return outer.cut(opening)


def _build_transom_glass_shape():
    ow = TRANSOM_OPEN_W + 2 * REBATE
    oh = TRANSOM_OPEN_H + 2 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


def _build_screen_frame_shape():
    """Screen frame ring, centered at origin."""
    ow = SCREEN_OPEN_W
    oh = SCREEN_OPEN_H
    out_w = ow + 2 * SCREEN_FRAME_W
    out_h = oh + 2 * SCREEN_FRAME_W
    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, SCREEN_DEPTH)
    opening = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SCREEN_DEPTH + 0.02)
    return outer.cut(opening)


def _build_screen_mesh_shape():
    ow = SCREEN_OPEN_W
    oh = SCREEN_OPEN_H
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SCREEN_MESH_T)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_window_transom_variant")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("metal", rgba=METAL_RGBA)
    model.material("screen_mat", rgba=SCREEN_RGBA)
    model.material("roller_mat", rgba=ROLLER_RGBA)

    # --- Frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame_shell"),
        material="vinyl",
        name="frame_shell",
    )

    # --- Transom (fixed, upper zone) ---
    transom = model.part("transom")
    transom.visual(
        mesh_from_cadquery(_build_transom_shape(), "transom_vinyl"),
        material="vinyl",
        name="transom_vinyl",
    )
    transom.visual(
        mesh_from_cadquery(_build_transom_glass_shape(), "transom_glass"),
        material="glass",
        name="transom_glass",
    )

    # --- Fixed sash (left) ---
    fixed_sash = model.part("fixed_sash")
    fixed_sash.visual(
        mesh_from_cadquery(_build_sash_shape(), "fixed_sash_vinyl"),
        material="vinyl",
        name="fixed_sash_vinyl",
    )
    fixed_sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(), "fixed_sash_glass"),
        material="glass",
        name="fixed_sash_glass",
    )

    # --- Sliding sash (right) with latch, rollers, overlap stile ---
    sliding_sash = model.part("sliding_sash")
    sliding_sash.visual(
        mesh_from_cadquery(_build_sash_shape(), "sliding_sash_vinyl"),
        material="vinyl",
        name="sliding_sash_vinyl",
    )
    sliding_sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(), "sliding_sash_glass"),
        material="glass",
        name="sliding_sash_glass",
    )

    # Latch keeper plate + lever on meeting stile
    stile_x = -SASH_OPENING_W / 2.0 - SASH_FACE / 2.0
    face_y = SASH_DEPTH / 2.0
    plate_y = face_y + LATCH_PLATE_T / 2.0
    sliding_sash.visual(
        Box((LATCH_PLATE_W, LATCH_PLATE_T, LATCH_PLATE_H)),
        origin=Origin(xyz=(stile_x, plate_y, 0.0)),
        material="metal",
        name="sliding_sash_latch_plate",
    )
    lever_y = face_y + LATCH_PLATE_T + LATCH_LEVER_LEN / 2.0
    sliding_sash.visual(
        Cylinder(radius=LATCH_LEVER_R, length=LATCH_LEVER_LEN),
        origin=Origin(xyz=(stile_x, lever_y, -0.008), rpy=(1.5707963, 0.0, 0.0)),
        material="metal",
        name="sliding_sash_latch_lever",
    )

    # Two roller blocks at the bottom rail of sliding sash (protrude below sash
    # to ride on the sill track, as real window rollers do)
    sash_outer_w = SASH_OPENING_W + 2 * SASH_FACE
    roller_z = -(SASH_OPENING_H / 2.0 + SASH_FACE) - 0.002  # protrudes below sash bottom
    roller_positions = [
        -sash_outer_w / 4.0,   # quarter-width toward left
        sash_outer_w / 4.0,    # quarter-width toward right
    ]
    for i, rx in enumerate(roller_positions):
        sliding_sash.visual(
            Box((ROLLER_W, ROLLER_D, ROLLER_H)),
            origin=Origin(xyz=(rx, 0.0, roller_z)),
            material="roller_mat",
            name=f"roller_block_{i}",
        )

    # Overlap stile (astragal): vertical strip on meeting stile, proud of face
    astragal_x = stile_x
    astragal_h = SASH_OPENING_H + 2 * SASH_FACE  # matches sash outer height
    astragal_y = face_y + ASTRAGAL_D / 2.0       # proud of front face
    sliding_sash.visual(
        Box((ASTRAGAL_W, ASTRAGAL_D, astragal_h)),
        origin=Origin(xyz=(astragal_x, astragal_y, 0.0)),
        material="vinyl",
        name="overlap_stile",
    )

    # --- Insect screen (independent prismatic) ---
    screen = model.part("insect_screen")
    screen.visual(
        mesh_from_cadquery(_build_screen_frame_shape(), "screen_frame"),
        material="vinyl",
        name="screen_frame",
    )
    screen.visual(
        mesh_from_cadquery(_build_screen_mesh_shape(), "screen_mesh"),
        material="screen_mat",
        name="screen_mesh",
    )

    # -----------------------------------------------------------------------
    # Articulations
    # -----------------------------------------------------------------------

    # Fixed transom
    model.articulation(
        "frame_to_transom",
        ArticulationType.FIXED,
        parent="frame",
        child="transom",
        origin=Origin(xyz=(0.0, FIXED_SASH_Y, TRANSOM_CZ)),
    )

    # Fixed sash (left)
    model.articulation(
        "frame_to_fixed_sash",
        ArticulationType.FIXED,
        parent="frame",
        child="fixed_sash",
        origin=Origin(xyz=(FIXED_OPEN_CX, FIXED_SASH_Y, MID_MAIN_Z)),
    )

    # Sliding sash (right): prismatic along -X (positive q opens toward left)
    slide_travel = SASH_OPENING_W * 0.90
    model.articulation(
        "frame_to_sliding_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="sliding_sash",
        origin=Origin(xyz=(SLIDE_OPEN_CX, SLIDE_SASH_Y, MID_MAIN_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.5, lower=0.0, upper=slide_travel),
    )

    # Insect screen: independent prismatic along -X
    model.articulation(
        "frame_to_screen",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="insect_screen",
        origin=Origin(xyz=(SCREEN_CX, SCREEN_Y, MID_MAIN_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.3, lower=0.0, upper=SCREEN_TRAVEL),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    transom = object_model.get_part("transom")
    fixed_sash = object_model.get_part("fixed_sash")
    sliding_sash = object_model.get_part("sliding_sash")
    screen = object_model.get_part("insect_screen")
    slide = object_model.get_articulation("frame_to_sliding_sash")
    screen_slide = object_model.get_articulation("frame_to_screen")

    # --- Intentional overlap allowances ---

    # Glass rebated under sash lips
    for nm in ("fixed_sash", "sliding_sash"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass", elem_b=f"{nm}_vinyl",
            reason=f"Pane rebated under {nm} lip (captured glazing).",
        )
    # Sashes rebated into frame tracks
    for nm in ("fixed_sash", "sliding_sash"):
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell", elem_b=f"{nm}_vinyl",
            reason=f"{nm} ring rebated into frame head/sill track.",
        )
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell", elem_b=f"{nm}_glass",
            reason=f"{nm} glass within frame opening rebate.",
        )
    # Transom glass rebated under transom frame
    ctx.allow_overlap(
        "transom", "transom",
        elem_a="transom_glass", elem_b="transom_vinyl",
        reason="Transom glass rebated under transom frame lip.",
    )
    # Transom seated in frame opening
    ctx.allow_overlap(
        "frame", "transom",
        elem_a="frame_shell", elem_b="transom_vinyl",
        reason="Transom frame seated in upper frame opening.",
    )
    ctx.allow_overlap(
        "frame", "transom",
        elem_a="frame_shell", elem_b="transom_glass",
        reason="Transom glass within frame opening rebate.",
    )
    # Latch seated on sliding sash stile
    ctx.allow_overlap(
        "sliding_sash", "sliding_sash",
        elem_a="sliding_sash_latch_plate", elem_b="sliding_sash_vinyl",
        reason="Latch plate seated on sliding sash meeting stile.",
    )
    # Roller blocks seated on sash bottom rail
    for i in range(2):
        ctx.allow_overlap(
            "sliding_sash", "sliding_sash",
            elem_a=f"roller_block_{i}", elem_b="sliding_sash_vinyl",
            reason=f"Roller block {i} embedded in sliding sash bottom rail.",
        )
    # Overlap stile mounted on sash meeting stile
    ctx.allow_overlap(
        "sliding_sash", "sliding_sash",
        elem_a="overlap_stile", elem_b="sliding_sash_vinyl",
        reason="Overlap stile (astragal) mounted proud of sliding sash meeting stile face.",
    )
    # Roller blocks protrude below sash into sill track (real roller-on-track)
    for i in range(2):
        ctx.allow_overlap(
            "frame", "sliding_sash",
            elem_a="frame_shell", elem_b=f"roller_block_{i}",
            reason=f"Roller block {i} protrudes below sash into sill track (real roller contact).",
        )
    # Screen frame in rear frame track
    ctx.allow_overlap(
        "frame", "insect_screen",
        elem_a="frame_shell", elem_b="screen_frame",
        reason="Screen frame slides in rear frame track.",
    )
    ctx.allow_overlap(
        "frame", "insect_screen",
        elem_a="frame_shell", elem_b="screen_mesh",
        reason="Screen mesh within frame track envelope.",
    )
    # Screen mesh captured in screen frame
    ctx.allow_overlap(
        "insect_screen", "insect_screen",
        elem_a="screen_mesh", elem_b="screen_frame",
        reason="Screen mesh captured within screen frame ring.",
    )

    # --- Transom above sashes ---
    transom_aabb = ctx.part_world_aabb(transom)
    fixed_aabb = ctx.part_world_aabb(fixed_sash)
    sliding_aabb = ctx.part_world_aabb(sliding_sash)

    ctx.check(
        "transom above fixed sash",
        transom_aabb[0][2] > fixed_aabb[1][2] - 0.02,
        details=f"transom_zmin={transom_aabb[0][2]:.3f}, fixed_zmax={fixed_aabb[1][2]:.3f}",
    )
    ctx.check(
        "transom above sliding sash",
        transom_aabb[0][2] > sliding_aabb[1][2] - 0.02,
        details=f"transom_zmin={transom_aabb[0][2]:.3f}, sliding_zmax={sliding_aabb[1][2]:.3f}",
    )

    # --- Roller blocks present on sliding sash ---
    sash_visual_names = [v.name for v in sliding_sash.visuals]
    for i in range(2):
        ctx.check(
            f"roller_block_{i} present on sliding sash",
            f"roller_block_{i}" in sash_visual_names,
        )

    # Roller blocks near bottom of sash and protrude below sash bottom rail
    sash_vinyl_aabb = ctx.part_element_world_aabb(sliding_sash, elem="sliding_sash_vinyl")
    sash_bottom_z = sash_vinyl_aabb[0][2]
    for i in range(2):
        roller_aabb = ctx.part_element_world_aabb(sliding_sash, elem=f"roller_block_{i}")
        ctx.check(
            f"roller_block_{i} near bottom of sash",
            roller_aabb[1][2] < MID_MAIN_Z,
            details=f"roller_top_z={roller_aabb[1][2]:.3f}, mid_z={MID_MAIN_Z:.3f}",
        )
        ctx.check(
            f"roller_block_{i} protrudes below sash bottom",
            roller_aabb[0][2] < sash_bottom_z + 0.001,
            details=f"roller_bottom_z={roller_aabb[0][2]:.4f}, sash_bottom_z={sash_bottom_z:.4f}",
        )

    # --- Overlap stile present on sliding sash ---
    ctx.check(
        "overlap stile present on sliding sash",
        "overlap_stile" in sash_visual_names,
    )

    # Overlap stile is on the meeting (left) edge of the sliding sash
    with ctx.pose({slide: 0.0}):
        stile_aabb = ctx.part_element_world_aabb(sliding_sash, elem="overlap_stile")
        stile_cx = (stile_aabb[0][0] + stile_aabb[1][0]) / 2.0
        sliding_cx = ctx.part_world_position(sliding_sash)[0]
        fixed_cx = ctx.part_world_position(fixed_sash)[0]
        ctx.check(
            "overlap stile on meeting edge between sashes",
            fixed_cx < stile_cx < sliding_cx + 0.05,
            details=f"fixed_cx={fixed_cx:.3f}, stile_cx={stile_cx:.3f}, sliding_cx={sliding_cx:.3f}",
        )

    # --- Screen has independent prismatic joint ---
    ctx.check(
        "screen joint is prismatic",
        screen_slide.articulation_type == ArticulationType.PRISMATIC,
    )
    ctx.check(
        "screen joint is separate from sash joint",
        screen_slide.name != slide.name,
    )
    ctx.check(
        "screen joint has motion limits",
        screen_slide.motion_limits is not None
        and screen_slide.motion_limits.upper is not None
        and screen_slide.motion_limits.upper > 0.01,
    )

    # --- Screen slides independently from sash ---
    with ctx.pose({slide: 0.0, screen_slide: 0.0}):
        screen_rest_x = ctx.part_world_position(screen)[0]
        sash_rest_x = ctx.part_world_position(sliding_sash)[0]

    screen_upper = screen_slide.motion_limits.upper
    with ctx.pose({slide: 0.0, screen_slide: screen_upper}):
        screen_moved_x = ctx.part_world_position(screen)[0]
        sash_still_x = ctx.part_world_position(sliding_sash)[0]

    ctx.check(
        "screen moves independently when slid",
        abs(screen_moved_x - screen_rest_x) > 0.10,
        details=f"screen_dx={screen_moved_x - screen_rest_x:.3f}",
    )
    ctx.check(
        "sash stays put when only screen moves",
        abs(sash_still_x - sash_rest_x) < 0.005,
        details=f"sash_dx={sash_still_x - sash_rest_x:.4f}",
    )

    # --- Sliding sash opens correctly ---
    with ctx.pose({slide: 0.0}):
        rest_sx = ctx.part_world_position(sliding_sash)[0]
        rest_sz = ctx.part_world_position(sliding_sash)[2]

    travel = slide.motion_limits.upper
    with ctx.pose({slide: travel}):
        open_pos = ctx.part_world_position(sliding_sash)
        open_sx = open_pos[0]
        open_sz = open_pos[2]
        ctx.check(
            "sliding sash opens toward fixed sash (-X)",
            abs((rest_sx - open_sx) - travel) < 0.02 and open_sx < rest_sx - 0.20,
            details=f"rest_x={rest_sx:.3f}, open_x={open_sx:.3f}, travel={travel:.3f}",
        )
        ctx.check(
            "slide is purely horizontal (no Z change)",
            abs(open_sz - rest_sz) < 0.01,
            details=f"rest_z={rest_sz:.3f}, open_z={open_sz:.3f}",
        )
        # Retained in frame
        f_aabb = ctx.part_world_aabb(frame)
        s_aabb = ctx.part_world_aabb(sliding_sash)
        ctx.check(
            "sash retained within frame X span at full travel",
            s_aabb[0][0] > f_aabb[0][0] - 1e-4 and s_aabb[1][0] < f_aabb[1][0] + 1e-4,
            details=f"sash x=[{s_aabb[0][0]:.3f},{s_aabb[1][0]:.3f}]",
        )

    # --- Closed pose sanity ---
    with ctx.pose({slide: 0.0}):
        ctx.expect_overlap(
            fixed_sash, frame, axes="xz", min_overlap=0.03,
            name="fixed sash seated in frame opening",
        )
        ctx.expect_overlap(
            sliding_sash, frame, axes="xz", min_overlap=0.03,
            name="sliding sash seated in frame opening",
        )
        # Sliding sash proud of fixed sash in +Y
        fy = ctx.part_world_position(fixed_sash)[1]
        sy = ctx.part_world_position(sliding_sash)[1]
        ctx.check(
            "sliding sash proud of fixed sash",
            sy > fy + 0.02,
            details=f"sliding_y={sy:.3f}, fixed_y={fy:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
