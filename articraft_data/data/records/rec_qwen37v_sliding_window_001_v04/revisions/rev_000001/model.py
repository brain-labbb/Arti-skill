from __future__ import annotations

# Three-panel horizontal sliding window variant with:
# - Narrow transom panel above the sliding panes
# - Center sash slides left-right on a prismatic joint
# - Two tiny roller blocks at the bottom of the moving sash
# - Sill lip with drainage slots
# White vinyl/PVC frame with colonial divided-lite grilles.
#
# Coordinate convention:
#   +Z is up. Window stands vertically.
#     - width  -> X
#     - height -> Z   (sill near z=0)
#     - frame depth / glazing thickness / slide-normal -> Y
#   The glass plane is the X-Z plane. The window reads SHUT at q=0; driving the
#   prismatic joint slides the center sash sideways (+X) by ~one panel width,
#   staying retained in the track.

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Absolute dimensions (meters)
# ---------------------------------------------------------------------------

# Outer opening
TOTAL_W = 3.00            # overall window width along X
TOTAL_H = 1.60            # overall height along Z (including transom, sill at z=0)

FRAME_FACE = 0.070        # outer frame member face width (jamb / head / sill)
MULLION_FACE = 0.060      # intermediate mullion face width
FRAME_DEPTH = 0.110       # outer frame depth along Y (chunky vinyl box section)

# Transom: narrow fixed panel at top
TRANSOM_H = 0.22          # transom clear opening height
TRANSOM_BAR = 0.065       # horizontal transom bar face height

# Three lite columns below the transom. Center is wider (the slider).
SIDE_LITE_W = 0.85
CENTER_LITE_W = 1.04

# Sash construction
SASH_FACE = 0.055         # sash perimeter rail/stile face width (in-plane)
SASH_DEPTH = 0.055        # sash depth along Y
GLASS_T = 0.008           # glazing thickness along Y

# Colonial grille (divided lite): each lite is a grid of small panes.
GRILLE_COLS = 4
GRILLE_ROWS = 5
MUNTIN_T = 0.020
MUNTIN_DEPTH = 0.020

# Y layout (depth). Frame box centered on y=0.
FIXED_LITE_Y = -0.020
SLIDE_SASH_Y = 0.052

REBATE = 0.005

# Roller blocks at bottom of sliding sash
ROLLER_W = 0.040          # roller block width (X)
ROLLER_H = 0.018          # roller block height (Z)
ROLLER_D = 0.030          # roller block depth (Y)

# Sill lip and drainage
SILL_LIP_H = 0.025        # sill lip height (Z)
SILL_LIP_PROUD = 0.040    # how far the lip protrudes forward (+Y) from frame face
DRAIN_SLOT_W = 0.030      # drainage slot width
DRAIN_SLOT_H = 0.012      # drainage slot height (through the lip)
DRAIN_SLOT_COUNT = 4      # number of drainage slots

# ---------------------------------------------------------------------------
# Derived layout
# ---------------------------------------------------------------------------

HALF_W = TOTAL_W / 2.0

# Inner clear region (inside the outer head/sill/jambs)
INNER_X0 = -HALF_W + FRAME_FACE
INNER_X1 = HALF_W - FRAME_FACE
INNER_Z0 = FRAME_FACE
INNER_Z1 = TOTAL_H - FRAME_FACE

# Vertical split: transom at top, main lites below
TRANSOM_OPENING_Z0 = INNER_Z1 - TRANSOM_H           # bottom of transom opening
TRANSOM_BAR_Z0 = TRANSOM_OPENING_Z0 - TRANSOM_BAR   # bottom of transom bar
MAIN_LITE_Z1 = TRANSOM_BAR_Z0                       # top of main lite openings
MAIN_LITE_Z0 = INNER_Z0                             # bottom of main lite openings

# Three lite openings laid left -> center -> right with two mullions between.
LEFT_X0 = INNER_X0
LEFT_X1 = LEFT_X0 + SIDE_LITE_W
MUL0_X0 = LEFT_X1
MUL0_X1 = MUL0_X0 + MULLION_FACE
CENTER_X0 = MUL0_X1
CENTER_X1 = CENTER_X0 + CENTER_LITE_W
MUL1_X0 = CENTER_X1
MUL1_X1 = MUL1_X0 + MULLION_FACE
RIGHT_X0 = MUL1_X1
RIGHT_X1 = INNER_X1

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

VINYL_RGBA = (0.94, 0.95, 0.96, 1.0)
GLASS_RGBA = (0.50, 0.58, 0.64, 0.32)
DARK_RGBA = (0.15, 0.15, 0.15, 1.0)  # roller blocks (dark plastic/metal)


# ---------------------------------------------------------------------------
# Geometry helpers (CadQuery). All authored directly in meters, world frame.
# ---------------------------------------------------------------------------

def _slab(x0: float, x1: float, z0: float, z1: float, y_center: float, depth: float) -> cq.Workplane:
    """Axis-aligned box spanning [x0,x1] x [z0,z1] in the X-Z plane, centered on
    y_center with the given Y depth."""
    w = x1 - x0
    h = z1 - z0
    cx = (x0 + x1) / 2.0
    cz = (z0 + z1) / 2.0
    return (
        cq.Workplane("XY")
        .transformed(offset=(cx, y_center, cz))
        .box(w, depth, h)
    )


def _build_frame_shape() -> cq.Workplane:
    """Static outer frame with transom bar: a full slab cut by the three lite
    openings (lower region) and the transom opening (upper region), plus
    the sill lip with drainage slots."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)

    cut_depth = FRAME_DEPTH + 0.02

    # Three main lite openings (below transom bar)
    left_cut = _slab(LEFT_X0, LEFT_X1, MAIN_LITE_Z0, MAIN_LITE_Z1, 0.0, cut_depth)
    center_cut = _slab(CENTER_X0, CENTER_X1, MAIN_LITE_Z0, MAIN_LITE_Z1, 0.0, cut_depth)
    right_cut = _slab(RIGHT_X0, RIGHT_X1, MAIN_LITE_Z0, MAIN_LITE_Z1, 0.0, cut_depth)

    # Transom opening (full inner width, above transom bar)
    transom_cut = _slab(INNER_X0, INNER_X1, TRANSOM_OPENING_Z0, INNER_Z1, 0.0, cut_depth)

    frame = outer.cut(left_cut).cut(center_cut).cut(right_cut).cut(transom_cut)

    # Sill lip: protrudes forward (+Y) below the sill, with drainage slots
    sill_lip = _slab(
        -HALF_W + FRAME_FACE * 0.5,
        HALF_W - FRAME_FACE * 0.5,
        FRAME_FACE * 0.2,
        FRAME_FACE * 0.2 + SILL_LIP_H,
        FRAME_DEPTH / 2.0 + SILL_LIP_PROUD / 2.0,
        SILL_LIP_PROUD,
    )
    frame = frame.union(sill_lip)

    # Drainage slots: rectangular through-cuts in the sill lip
    lip_x_span = (HALF_W - FRAME_FACE * 0.5) - (-HALF_W + FRAME_FACE * 0.5)
    lip_x_center = 0.0
    slot_spacing = lip_x_span / (DRAIN_SLOT_COUNT + 1)
    lip_y_center = FRAME_DEPTH / 2.0 + SILL_LIP_PROUD / 2.0
    lip_z_center = FRAME_FACE * 0.2 + SILL_LIP_H / 2.0

    for i in range(DRAIN_SLOT_COUNT):
        sx = -lip_x_span / 2.0 + slot_spacing * (i + 1)
        slot = _slab(
            sx - DRAIN_SLOT_W / 2.0,
            sx + DRAIN_SLOT_W / 2.0,
            lip_z_center - DRAIN_SLOT_H / 2.0,
            lip_z_center + DRAIN_SLOT_H / 2.0,
            lip_y_center,
            SILL_LIP_PROUD + 0.02,  # through-cut in Y
        )
        frame = frame.cut(slot)

    return frame


def _build_sash_grille_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """One sash built in its OWN local frame, centered on local origin."""
    ow = opening_w
    oh = opening_h
    out_w = ow + 2 * SASH_FACE
    out_h = oh + 2 * SASH_FACE

    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, SASH_DEPTH)
    opening = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SASH_DEPTH + 0.02)
    ring = outer.cut(opening)

    bars = None

    # Vertical muntins
    for c in range(1, GRILLE_COLS):
        frac = c / GRILLE_COLS
        x = -ow / 2.0 + frac * ow
        bar = _slab(
            x - MUNTIN_T / 2.0, x + MUNTIN_T / 2.0,
            -oh / 2.0, oh / 2.0,
            0.0, MUNTIN_DEPTH,
        )
        bars = bar if bars is None else bars.union(bar)

    # Horizontal muntins
    for r in range(1, GRILLE_ROWS):
        frac = r / GRILLE_ROWS
        z = -oh / 2.0 + frac * oh
        bar = _slab(
            -ow / 2.0, ow / 2.0,
            z - MUNTIN_T / 2.0, z + MUNTIN_T / 2.0,
            0.0, MUNTIN_DEPTH,
        )
        bars = bar if bars is None else bars.union(bar)

    return ring if bars is None else ring.union(bars)


def _build_sash_glass_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """Single clear pane filling the sash opening, in the same sash-local frame."""
    ow = opening_w + 2 * REBATE
    oh = opening_h + 2 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


def _build_roller_shape() -> cq.Workplane:
    """Small roller block: a dark plastic/metal block at the sash bottom rail."""
    return (
        cq.Workplane("XY")
        .box(ROLLER_W, ROLLER_D, ROLLER_H)
    )


def _build_transom_glass_shape() -> cq.Workplane:
    """Transom glass pane spanning the full inner width."""
    tw = INNER_X1 - INNER_X0 + 2 * REBATE
    th = TRANSOM_H + 2 * REBATE
    return _slab(-tw / 2.0, tw / 2.0, -th / 2.0, th / 2.0, 0.0, GLASS_T)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    # Sanity: the three lites + two mullions must fill the inner clear width.
    span = SIDE_LITE_W + MULLION_FACE + CENTER_LITE_W + MULLION_FACE + SIDE_LITE_W
    inner_w = INNER_X1 - INNER_X0
    assert abs(span - inner_w) < 1e-6, f"lite layout {span} != inner width {inner_w}"

    model = ArticulatedObject(name="sliding_window_with_transom")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("dark_plastic", rgba=DARK_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material="vinyl",
        name="frame_shell",
    )

    # Opening heights for the main lites (below transom)
    main_opening_h = MAIN_LITE_Z1 - MAIN_LITE_Z0

    # --- Two FIXED side lites ---
    left_lite = model.part("left_lite")
    left_lite.visual(
        mesh_from_cadquery(_build_sash_grille_shape(SIDE_LITE_W, main_opening_h), "left_lite_vinyl"),
        material="vinyl",
        name="left_lite_vinyl",
    )
    left_lite.visual(
        mesh_from_cadquery(_build_sash_glass_shape(SIDE_LITE_W, main_opening_h), "left_lite_glass"),
        material="glass",
        name="left_lite_glass",
    )

    right_lite = model.part("right_lite")
    right_lite.visual(
        mesh_from_cadquery(_build_sash_grille_shape(SIDE_LITE_W, main_opening_h), "right_lite_vinyl"),
        material="vinyl",
        name="right_lite_vinyl",
    )
    right_lite.visual(
        mesh_from_cadquery(_build_sash_glass_shape(SIDE_LITE_W, main_opening_h), "right_lite_glass"),
        material="glass",
        name="right_lite_glass",
    )

    # --- CENTER sliding sash with roller blocks ---
    center_sash = model.part("center_sash")
    center_sash.visual(
        mesh_from_cadquery(_build_sash_grille_shape(CENTER_LITE_W, main_opening_h), "center_sash_vinyl"),
        material="vinyl",
        name="center_sash_vinyl",
    )
    center_sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(CENTER_LITE_W, main_opening_h), "center_sash_glass"),
        material="glass",
        name="center_sash_glass",
    )

    # Roller blocks at the bottom of the center sash.
    # In the sash-local frame, the bottom rail is at z = -(main_opening_h/2 + SASH_FACE).
    # Place two rollers: one near left edge, one near right edge.
    roller_z_local = -(main_opening_h / 2.0 + SASH_FACE) - ROLLER_H / 2.0
    roller_x_offset = CENTER_LITE_W / 2.0 - ROLLER_W

    # Left roller
    center_sash.visual(
        mesh_from_cadquery(_build_roller_shape(), "roller_left"),
        origin=Origin(xyz=(-roller_x_offset, 0.0, roller_z_local)),
        material="dark_plastic",
        name="roller_left",
    )
    # Right roller
    center_sash.visual(
        mesh_from_cadquery(_build_roller_shape(), "roller_right"),
        origin=Origin(xyz=(roller_x_offset, 0.0, roller_z_local)),
        material="dark_plastic",
        name="roller_right",
    )

    # --- Transom panel (fixed glass above the main lites) ---
    transom = model.part("transom")
    transom.visual(
        mesh_from_cadquery(_build_transom_glass_shape(), "transom_glass"),
        material="glass",
        name="transom_glass",
    )

    # Centers (world) of each clear opening.
    left_cx = (LEFT_X0 + LEFT_X1) / 2.0
    right_cx = (RIGHT_X0 + RIGHT_X1) / 2.0
    center_cx = (CENTER_X0 + CENTER_X1) / 2.0
    main_mid_cz = (MAIN_LITE_Z0 + MAIN_LITE_Z1) / 2.0
    transom_cx = (INNER_X0 + INNER_X1) / 2.0
    transom_cz = (TRANSOM_OPENING_Z0 + INNER_Z1) / 2.0

    # FIXED side lites seated in the rear glazing plane.
    model.articulation(
        "frame_to_left_lite",
        ArticulationType.FIXED,
        parent="frame",
        child="left_lite",
        origin=Origin(xyz=(left_cx, FIXED_LITE_Y, main_mid_cz)),
    )
    model.articulation(
        "frame_to_right_lite",
        ArticulationType.FIXED,
        parent="frame",
        child="right_lite",
        origin=Origin(xyz=(right_cx, FIXED_LITE_Y, main_mid_cz)),
    )

    # Transom panel: FIXED to frame, above the main lites
    model.articulation(
        "frame_to_transom",
        ArticulationType.FIXED,
        parent="frame",
        child="transom",
        origin=Origin(xyz=(transom_cx, FIXED_LITE_Y, transom_cz)),
    )

    # CENTER sliding sash: PRISMATIC along +X.
    slide_travel = SIDE_LITE_W * 0.92
    model.articulation(
        "frame_to_center_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="center_sash",
        origin=Origin(xyz=(center_cx, SLIDE_SASH_Y, main_mid_cz)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.5, lower=0.0, upper=slide_travel),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    left_lite = object_model.get_part("left_lite")
    right_lite = object_model.get_part("right_lite")
    center_sash = object_model.get_part("center_sash")
    transom = object_model.get_part("transom")
    slide = object_model.get_articulation("frame_to_center_sash")

    # --- Intentional overlaps ---
    # Glass panes tuck under the vinyl/muntin lip on each sash (captured glass).
    for nm in ("left_lite", "right_lite", "center_sash"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass",
            elem_b=f"{nm}_vinyl",
            reason="Clear pane is rebated under the sash/muntin lip so it reads captured, not floating.",
        )

    # Fixed side lites rebated into frame openings
    ctx.allow_overlap(
        "frame", "left_lite",
        elem_a="frame_shell",
        elem_b="left_lite_vinyl",
        reason="Left fixed lite is rebated into the frame opening; its sash ring laps the jamb/mullion edge.",
    )
    ctx.allow_overlap(
        "frame", "right_lite",
        elem_a="frame_shell",
        elem_b="right_lite_vinyl",
        reason="Right fixed lite is rebated into the frame opening; its sash ring laps the jamb/mullion edge.",
    )

    # Center sash rides the head/sill track
    ctx.allow_overlap(
        "frame", "center_sash",
        elem_a="frame_shell",
        elem_b="center_sash_vinyl",
        reason="Center sash rides the head/sill track and laps the frame face along the track.",
    )

    # Glass rebated under frame lip
    ctx.allow_overlap(
        "frame", "left_lite",
        elem_a="frame_shell",
        elem_b="left_lite_glass",
        reason="Left lite glass is rebated under the frame opening lip.",
    )
    ctx.allow_overlap(
        "frame", "right_lite",
        elem_a="frame_shell",
        elem_b="right_lite_glass",
        reason="Right lite glass is rebated under the frame opening lip.",
    )
    ctx.allow_overlap(
        "frame", "center_sash",
        elem_a="frame_shell",
        elem_b="center_sash_glass",
        reason="Center sash glass laps the head/sill track lip as the proud sash rides the track.",
    )

    # Transom glass rebated into frame transom opening
    ctx.allow_overlap(
        "frame", "transom",
        elem_a="frame_shell",
        elem_b="transom_glass",
        reason="Transom glass is rebated into the frame transom opening.",
    )

    # Roller blocks sit on the sill track, overlapping the sash bottom rail
    ctx.allow_overlap(
        "center_sash", "center_sash",
        elem_a="roller_left",
        elem_b="center_sash_vinyl",
        reason="Roller block is mounted into the bottom rail of the sliding sash.",
    )
    ctx.allow_overlap(
        "center_sash", "center_sash",
        elem_a="roller_right",
        elem_b="center_sash_vinyl",
        reason="Roller block is mounted into the bottom rail of the sliding sash.",
    )
    # Roller blocks contact/seat on the frame sill track (small local embed)
    ctx.allow_overlap(
        "center_sash", "frame",
        elem_a="roller_left",
        elem_b="frame_shell",
        reason="Roller block seats on the frame sill track; small local embed represents loaded contact.",
    )
    ctx.allow_overlap(
        "center_sash", "frame",
        elem_a="roller_right",
        elem_b="frame_shell",
        reason="Roller block seats on the frame sill track; small local embed represents loaded contact.",
    )

    # --- Transom panel checks ---
    frame_aabb = ctx.part_world_aabb(frame)
    transom_aabb = ctx.part_world_aabb(transom)

    # Transom sits above the main lites (higher Z)
    left_aabb = ctx.part_world_aabb(left_lite)
    ctx.check(
        "transom is above the left lite",
        transom_aabb[0][2] > left_aabb[0][2] + 0.05,
        details=f"transom zmin={transom_aabb[0][2]:.3f}, left_lite zmin={left_aabb[0][2]:.3f}",
    )
    # Transom sits within frame height
    ctx.check(
        "transom within frame height",
        transom_aabb[1][2] < frame_aabb[1][2] + 1e-4 and transom_aabb[0][2] > frame_aabb[0][2] - 1e-4,
        details=f"transom z=[{transom_aabb[0][2]:.3f},{transom_aabb[1][2]:.3f}] frame z=[{frame_aabb[0][2]:.3f},{frame_aabb[1][2]:.3f}]",
    )
    # Transom spans most of the frame width
    transom_w = transom_aabb[1][0] - transom_aabb[0][0]
    frame_w = frame_aabb[1][0] - frame_aabb[0][0]
    ctx.check(
        "transom spans most of frame width",
        transom_w > frame_w * 0.7,
        details=f"transom_w={transom_w:.3f}, frame_w={frame_w:.3f}",
    )

    # --- Sill lip check: frame extends forward (+Y) beyond the main frame depth ---
    # The sill lip protrudes in +Y, so frame AABB in Y should exceed FRAME_DEPTH
    frame_y_span = frame_aabb[1][1] - frame_aabb[0][1]
    ctx.check(
        "frame has sill lip protruding forward",
        frame_y_span > FRAME_DEPTH + SILL_LIP_PROUD * 0.5,
        details=f"frame y span={frame_y_span:.3f}, expected > {FRAME_DEPTH + SILL_LIP_PROUD * 0.5:.3f}",
    )

    # --- Roller blocks exist on center sash ---
    ctx.check(
        "center sash has roller_left visual",
        center_sash.get_visual("roller_left") is not None,
        details="roller_left visual missing from center_sash",
    )
    ctx.check(
        "center sash has roller_right visual",
        center_sash.get_visual("roller_right") is not None,
        details="roller_right visual missing from center_sash",
    )

    # Proof checks for roller-to-frame seating: rollers sit near the sill (z~0)
    # Use custom checks since expect_gap compares against frame.max[z] (the head, not sill)
    roller_left_aabb = ctx.part_element_world_aabb(center_sash, elem="roller_left")
    roller_right_aabb = ctx.part_element_world_aabb(center_sash, elem="roller_right")
    if roller_left_aabb is not None and roller_right_aabb is not None:
        ctx.check(
            "left roller near sill height",
            roller_left_aabb[0][2] < 0.010 and roller_left_aabb[0][2] > -0.010,
            details=f"roller_left zmin={roller_left_aabb[0][2]:.4f}",
        )
        ctx.check(
            "right roller near sill height",
            roller_right_aabb[0][2] < 0.010 and roller_right_aabb[0][2] > -0.010,
            details=f"roller_right zmin={roller_right_aabb[0][2]:.4f}",
        )

    # --- Closed pose (q=0): window reads SHUT ---
    with ctx.pose({slide: 0.0}):
        f_aabb = ctx.part_world_aabb(frame)
        l_aabb = ctx.part_world_aabb(left_lite)
        r_aabb = ctx.part_world_aabb(right_lite)
        c_aabb = ctx.part_world_aabb(center_sash)

        # Frame spans wider than center sash
        center_w = c_aabb[1][0] - c_aabb[0][0]
        ctx.check(
            "frame spans wider than the center sash",
            (f_aabb[1][0] - f_aabb[0][0]) > center_w + 1.5,
            details=f"frame_w={f_aabb[1][0] - f_aabb[0][0]:.3f}, center_w={center_w:.3f}",
        )

        # Sill sits near z=0
        ctx.check(
            "sill sits near z=0",
            abs(f_aabb[0][2]) < 0.02,
            details=f"frame zmin={f_aabb[0][2]:.4f}",
        )

        # Three lites ordered left -> center -> right
        lx = (l_aabb[0][0] + l_aabb[1][0]) / 2.0
        cx = (c_aabb[0][0] + c_aabb[1][0]) / 2.0
        rx = (r_aabb[0][0] + r_aabb[1][0]) / 2.0
        ctx.check(
            "lites ordered left-center-right",
            lx < cx < rx,
            details=f"left_x={lx:.3f}, center_x={cx:.3f}, right_x={rx:.3f}",
        )

        # Center sash sits proud (in +Y) of the fixed side lites
        l_y = (l_aabb[0][1] + l_aabb[1][1]) / 2.0
        c_y = (c_aabb[0][1] + c_aabb[1][1]) / 2.0
        ctx.check(
            "center sash proud of side lites",
            c_y > l_y + 0.02,
            details=f"center_y={c_y:.3f}, side_y={l_y:.3f}",
        )

        # Fixed lites seated in frame
        ctx.expect_overlap(
            left_lite, frame, axes="xz", min_overlap=0.03,
            name="left fixed lite seated in frame opening",
        )
        ctx.expect_overlap(
            right_lite, frame, axes="xz", min_overlap=0.03,
            name="right fixed lite seated in frame opening",
        )

        rest_cx = cx
        rest_cz = (c_aabb[0][2] + c_aabb[1][2]) / 2.0

    # --- Driven/open pose: center sash slides sideways along +X ---
    travel = slide.motion_limits.upper
    with ctx.pose({slide: travel}):
        c_open = ctx.part_world_aabb(center_sash)
        open_cx = (c_open[0][0] + c_open[1][0]) / 2.0
        ctx.check(
            "center sash slides along +X by ~travel",
            abs((open_cx - rest_cx) - travel) < 0.02,
            details=f"rest_cx={rest_cx:.3f}, open_cx={open_cx:.3f}, travel={travel:.3f}",
        )
        # Slide is purely horizontal (no Z movement)
        c_open_z = (c_open[0][2] + c_open[1][2]) / 2.0
        ctx.check(
            "slide is purely horizontal",
            abs(c_open_z - rest_cz) < 0.02,
            details=f"open_z={c_open_z:.3f}, rest_z={rest_cz:.3f}",
        )
        # Retained insertion: sash stays within frame X span
        f_aabb_open = ctx.part_world_aabb(frame)
        ctx.check(
            "sash retained within frame X span at full travel",
            c_open[1][0] < f_aabb_open[1][0] + 1e-4 and c_open[0][0] > f_aabb_open[0][0] - 1e-4,
            details=f"sash x=[{c_open[0][0]:.3f},{c_open[1][0]:.3f}] frame x=[{f_aabb_open[0][0]:.3f},{f_aabb_open[1][0]:.3f}]",
        )
        ctx.expect_overlap(
            center_sash, frame,
            axes="z",
            min_overlap=0.10,
            name="sash retains vertical engagement with head/sill track",
        )

    return ctx.report()


object_model = build_object_model()
