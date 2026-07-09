from __future__ import annotations

# Three-panel horizontal sliding window variant 22: wider fixed center pane,
# left sash slides sideways, cam latch at meeting rail (revolute), two roller
# blocks on sliding sash, sill lip with drainage slots.
#
# Coordinate convention:
#   +Z is up. Window stands vertically.
#     width  -> X,  height -> Z (sill near z=0),  depth -> Y
#   Glass plane is X-Z. At q=0 the slider is closed; driving the prismatic
#   joint slides the left sash along +X. The latch rotates on a revolute Y
#   axis (swings in the window plane).
#
# Structure:
#   - frame (root): head, sill (with lip + drainage slots), jambs, mullions.
#   - center_lite (FIXED, wider), right_lite (FIXED).
#   - left_sash (PRISMATIC +X): sash + grille + glass + two roller blocks.
#   - latch (REVOLUTE Y): small cam lever at meeting rail, child of left_sash.

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

TOTAL_W = 3.00
TOTAL_H = 1.50

FRAME_FACE = 0.070
MULLION_FACE = 0.115  # wide enough for two adjacent sash stiles
FRAME_DEPTH = 0.110

# Three lite columns: left slides, center fixed (wider), right fixed.
# Mullion must be >= 2*SASH_FACE so adjacent fixed-lite stiles don't interpenetrate.
SIDE_LITE_W = 0.74
CENTER_LITE_W = 1.15

SASH_FACE = 0.055
SASH_DEPTH = 0.055
GLASS_T = 0.008

GRILLE_COLS = 4
GRILLE_ROWS = 5
MUNTIN_T = 0.020
MUNTIN_DEPTH = 0.020

# Y layout (depth). Fixed lites rear, sliding sash proud (+Y).
FIXED_LITE_Y = -0.020
SLIDE_SASH_Y = 0.052

REBATE = 0.005

# Sill lip dimensions
SILL_LIP_DEPTH = 0.025
SILL_LIP_HEIGHT = 0.015

# Drainage slot dimensions
DRAIN_SLOT_W = 0.040
DRAIN_SLOT_H = 0.012
DRAIN_SLOT_CUT_DEPTH = 0.030
NUM_DRAIN_SLOTS = 3

# Latch dimensions
LATCH_BASE_R = 0.012
LATCH_BASE_H = 0.018
LATCH_LEVER_W = 0.055
LATCH_LEVER_D = 0.012

# Roller block dimensions
ROLLER_W = 0.040
ROLLER_H = 0.020
ROLLER_D = 0.030

# ---------------------------------------------------------------------------
# Derived layout
# ---------------------------------------------------------------------------

HALF_W = TOTAL_W / 2.0

INNER_X0 = -HALF_W + FRAME_FACE
INNER_X1 = HALF_W - FRAME_FACE
INNER_Z0 = FRAME_FACE
INNER_Z1 = TOTAL_H - FRAME_FACE

# left_sash | mullion | center_lite | mullion | right_lite
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
METAL_RGBA = (0.55, 0.57, 0.60, 1.0)


# ---------------------------------------------------------------------------
# Geometry helpers (CadQuery, meters, world frame)
# ---------------------------------------------------------------------------

def _slab(x0, x1, z0, z1, y_center, depth):
    """Axis-aligned box spanning [x0,x1] x [z0,z1], centered on y_center."""
    w = x1 - x0
    h = z1 - z0
    cx = (x0 + x1) / 2.0
    cz = (z0 + z1) / 2.0
    return (
        cq.Workplane("XY")
        .transformed(offset=(cx, y_center, cz))
        .box(w, depth, h)
    )


def _build_frame_shape():
    """Outer frame: slab cut by three lite openings, plus sill lip and drainage
    slots."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)

    cut_depth = FRAME_DEPTH + 0.02
    left_cut = _slab(LEFT_X0, LEFT_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    center_cut = _slab(CENTER_X0, CENTER_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    right_cut = _slab(RIGHT_X0, RIGHT_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)

    frame = outer.cut(left_cut).cut(center_cut).cut(right_cut)

    # Sill lip: horizontal shelf protruding forward (+Y) at the sill bottom.
    sill_lip = _slab(
        -HALF_W + FRAME_FACE, HALF_W - FRAME_FACE,
        0.0, SILL_LIP_HEIGHT,
        FRAME_DEPTH / 2.0 + SILL_LIP_DEPTH / 2.0,
        SILL_LIP_DEPTH,
    )
    frame = frame.union(sill_lip)

    # Drainage slots: rectangular weep holes cut through the sill lip.
    inner_w = INNER_X1 - INNER_X0
    for i in range(NUM_DRAIN_SLOTS):
        frac = (i + 1) / (NUM_DRAIN_SLOTS + 1)
        sx = INNER_X0 + frac * inner_w
        slot = _slab(
            sx - DRAIN_SLOT_W / 2.0, sx + DRAIN_SLOT_W / 2.0,
            0.002, SILL_LIP_HEIGHT - 0.002,
            FRAME_DEPTH / 2.0 + SILL_LIP_DEPTH / 2.0,
            SILL_LIP_DEPTH + 0.02,
        )
        frame = frame.cut(slot)

    return frame


def _build_sash_grille_shape(opening_w, opening_h):
    """Sash ring + colonial muntin grille, centered on local origin."""
    ow = opening_w
    oh = opening_h
    out_w = ow + 2 * SASH_FACE
    out_h = oh + 2 * SASH_FACE

    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, SASH_DEPTH)
    opening = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SASH_DEPTH + 0.02)
    ring = outer.cut(opening)

    bars = None
    for c in range(1, GRILLE_COLS):
        frac = c / GRILLE_COLS
        x = -ow / 2.0 + frac * ow
        bar = _slab(
            x - MUNTIN_T / 2.0, x + MUNTIN_T / 2.0,
            -oh / 2.0, oh / 2.0,
            0.0, MUNTIN_DEPTH,
        )
        bars = bar if bars is None else bars.union(bar)

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


def _build_sash_with_rollers(opening_w, opening_h):
    """Sliding sash: sash ring + grille + two roller blocks at the bottom rail.
    Rollers overlap the bottom rail slightly to ensure geometric connectivity."""
    sash = _build_sash_grille_shape(opening_w, opening_h)

    out_h = opening_h + 2 * SASH_FACE
    # Rollers sit just below the bottom rail, overlapping 5 mm into it.
    roller_z_top = -out_h / 2.0 + 0.005
    roller_z_bottom = roller_z_top - ROLLER_H

    for x_frac in (-0.35, 0.35):
        x_pos = opening_w * x_frac
        roller = _slab(
            x_pos - ROLLER_W / 2.0, x_pos + ROLLER_W / 2.0,
            roller_z_bottom, roller_z_top,
            0.0, ROLLER_D,
        )
        sash = sash.union(roller)

    return sash


def _build_sash_glass_shape(opening_w, opening_h):
    ow = opening_w + 2 * REBATE
    oh = opening_h + 2 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


def _build_latch_shape():
    """Small cam latch: pivot base block + lever arm extending upward (+Z).
    The latch rotates around the Y axis (swings in the window plane)."""
    # Pivot base block (centered at origin)
    base = _slab(
        -LATCH_BASE_R, LATCH_BASE_R,
        -LATCH_BASE_R, LATCH_BASE_R,
        0.0,
        LATCH_BASE_H * 2,
    )
    # Lever arm: extends from z=0 upward, overlapping the base for connectivity
    lever = _slab(
        -LATCH_LEVER_D / 2.0, LATCH_LEVER_D / 2.0,
        -0.005, LATCH_LEVER_W,
        0.0,
        LATCH_LEVER_D,
    )
    return base.union(lever)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    span = SIDE_LITE_W + MULLION_FACE + CENTER_LITE_W + MULLION_FACE + SIDE_LITE_W
    inner_w = INNER_X1 - INNER_X0
    assert abs(span - inner_w) < 1e-6, f"lite layout {span} != inner width {inner_w}"

    model = ArticulatedObject(name="three_panel_sliding_window_v22")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("metal", rgba=METAL_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material="vinyl",
        name="frame_shell",
    )

    opening_h = INNER_Z1 - INNER_Z0

    # --- Left sliding sash (with roller blocks) ---
    left_sash = model.part("left_sash")
    left_sash.visual(
        mesh_from_cadquery(_build_sash_with_rollers(SIDE_LITE_W, opening_h), "left_sash_vinyl"),
        material="vinyl",
        name="left_sash_vinyl",
    )
    left_sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(SIDE_LITE_W, opening_h), "left_sash_glass"),
        material="glass",
        name="left_sash_glass",
    )

    # --- Center fixed lite (wider) ---
    center_lite = model.part("center_lite")
    center_lite.visual(
        mesh_from_cadquery(_build_sash_grille_shape(CENTER_LITE_W, opening_h), "center_lite_vinyl"),
        material="vinyl",
        name="center_lite_vinyl",
    )
    center_lite.visual(
        mesh_from_cadquery(_build_sash_glass_shape(CENTER_LITE_W, opening_h), "center_lite_glass"),
        material="glass",
        name="center_lite_glass",
    )

    # --- Right fixed lite ---
    right_lite = model.part("right_lite")
    right_lite.visual(
        mesh_from_cadquery(_build_sash_grille_shape(SIDE_LITE_W, opening_h), "right_lite_vinyl"),
        material="vinyl",
        name="right_lite_vinyl",
    )
    right_lite.visual(
        mesh_from_cadquery(_build_sash_glass_shape(SIDE_LITE_W, opening_h), "right_lite_glass"),
        material="glass",
        name="right_lite_glass",
    )

    # --- Latch (child of left_sash, revolute) ---
    latch = model.part("latch")
    latch.visual(
        mesh_from_cadquery(_build_latch_shape(), "latch_body"),
        material="metal",
        name="latch_body",
    )

    # Centers of each lite opening
    left_cx = (LEFT_X0 + LEFT_X1) / 2.0
    right_cx = (RIGHT_X0 + RIGHT_X1) / 2.0
    center_cx = (CENTER_X0 + CENTER_X1) / 2.0
    mid_cz = (INNER_Z0 + INNER_Z1) / 2.0

    # FIXED center lite (wider, rear glazing plane)
    model.articulation(
        "frame_to_center_lite",
        ArticulationType.FIXED,
        parent="frame",
        child="center_lite",
        origin=Origin(xyz=(center_cx, FIXED_LITE_Y, mid_cz)),
    )

    # FIXED right lite (rear glazing plane)
    model.articulation(
        "frame_to_right_lite",
        ArticulationType.FIXED,
        parent="frame",
        child="right_lite",
        origin=Origin(xyz=(right_cx, FIXED_LITE_Y, mid_cz)),
    )

    # LEFT sliding sash: PRISMATIC along +X.
    slide_travel = SIDE_LITE_W * 0.92
    model.articulation(
        "frame_to_left_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="left_sash",
        origin=Origin(xyz=(left_cx, SLIDE_SASH_Y, mid_cz)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.5, lower=0.0, upper=slide_travel),
    )

    # LATCH: REVOLUTE around Y axis, mounted on left_sash at the meeting rail
    # (right stile of sash, front face, mid-height). In the sash local frame:
    # x = SIDE_LITE_W/2 + SASH_FACE/2 (center of right stile)
    # y = SASH_DEPTH/2 (front face)
    # z = 0 (mid-height)
    latch_x_local = SIDE_LITE_W / 2.0 + SASH_FACE / 2.0
    latch_y_local = SASH_DEPTH / 2.0 + 0.002
    model.articulation(
        "sash_to_latch",
        ArticulationType.REVOLUTE,
        parent="left_sash",
        child="latch",
        origin=Origin(xyz=(latch_x_local, latch_y_local, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.57),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    left_sash = object_model.get_part("left_sash")
    center_lite = object_model.get_part("center_lite")
    right_lite = object_model.get_part("right_lite")
    latch = object_model.get_part("latch")

    slide = object_model.get_articulation("frame_to_left_sash")
    latch_joint = object_model.get_articulation("sash_to_latch")

    # --- Intentional overlaps ---
    # Glass panes rebated under sash/muntin lip (captured glazing).
    for nm in ("left_sash", "center_lite", "right_lite"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass" if nm != "left_sash" else "left_sash_glass",
            elem_b=f"{nm}_vinyl" if nm != "left_sash" else "left_sash_vinyl",
            reason="Clear pane is rebated under the sash/muntin lip (captured glazing).",
        )

    # Fixed lites rebated into frame openings.
    ctx.allow_overlap(
        "frame", "center_lite",
        elem_a="frame_shell", elem_b="center_lite_vinyl",
        reason="Center fixed lite is rebated into the frame opening (seated capture).",
    )
    ctx.allow_overlap(
        "frame", "right_lite",
        elem_a="frame_shell", elem_b="right_lite_vinyl",
        reason="Right fixed lite is rebated into the frame opening (seated capture).",
    )

    # Sliding sash rides the head/sill track proud of the fixed lites.
    ctx.allow_overlap(
        "frame", "left_sash",
        elem_a="frame_shell", elem_b="left_sash_vinyl",
        reason="Left sash rides the head/sill track and laps the frame face (slider capture).",
    )

    # Glass rebated under frame lip for all panels.
    ctx.allow_overlap(
        "frame", "center_lite",
        elem_a="frame_shell", elem_b="center_lite_glass",
        reason="Center lite glass rebated under frame opening lip.",
    )
    ctx.allow_overlap(
        "frame", "right_lite",
        elem_a="frame_shell", elem_b="right_lite_glass",
        reason="Right lite glass rebated under frame opening lip.",
    )
    ctx.allow_overlap(
        "frame", "left_sash",
        elem_a="frame_shell", elem_b="left_sash_glass",
        reason="Left sash glass laps the head/sill track lip (captured glazing).",
    )

    # Latch overlaps the sash at the mounting point (seated hardware).
    ctx.allow_overlap(
        "left_sash", "latch",
        elem_a="left_sash_vinyl", elem_b="latch_body",
        reason="Latch pivot base is seated against the sash meeting stile (mounted hardware).",
    )

    # --- Prompt-specific checks ---

    # 1. Center lite is wider than the side lites.
    frame_aabb = ctx.part_world_aabb(frame)
    c_aabb = ctx.part_world_aabb(center_lite)
    l_aabb = ctx.part_world_aabb(left_sash)
    r_aabb = ctx.part_world_aabb(right_lite)

    center_w = c_aabb[1][0] - c_aabb[0][0]
    left_w = l_aabb[1][0] - l_aabb[0][0]
    right_w = r_aabb[1][0] - r_aabb[0][0]

    ctx.check(
        "center lite wider than left sash",
        center_w > left_w + 0.10,
        details=f"center_w={center_w:.3f}, left_w={left_w:.3f}",
    )
    ctx.check(
        "center lite wider than right lite",
        center_w > right_w + 0.10,
        details=f"center_w={center_w:.3f}, right_w={right_w:.3f}",
    )

    # 2. Prismatic joint: left sash slides along +X.
    ctx.check(
        "slide joint is prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={slide.articulation_type}",
    )

    with ctx.pose({slide: 0.0}):
        rest_aabb = ctx.part_world_aabb(left_sash)
        rest_cx = (rest_aabb[0][0] + rest_aabb[1][0]) / 2.0
        rest_cz = (rest_aabb[0][2] + rest_aabb[1][2]) / 2.0

    travel = slide.motion_limits.upper
    with ctx.pose({slide: travel}):
        open_aabb = ctx.part_world_aabb(left_sash)
        open_cx = (open_aabb[0][0] + open_aabb[1][0]) / 2.0
        open_cz = (open_aabb[0][2] + open_aabb[1][2]) / 2.0

        ctx.check(
            "left sash slides along +X by ~travel",
            abs((open_cx - rest_cx) - travel) < 0.02,
            details=f"rest_cx={rest_cx:.3f}, open_cx={open_cx:.3f}, travel={travel:.3f}",
        )
        ctx.check(
            "slide is purely horizontal (no Z drift)",
            abs(open_cz - rest_cz) < 0.02,
            details=f"rest_cz={rest_cz:.3f}, open_cz={open_cz:.3f}",
        )
        # Retained insertion: sash stays within frame X span.
        ctx.check(
            "sash retained within frame X span at full travel",
            open_aabb[1][0] < frame_aabb[1][0] + 1e-4 and open_aabb[0][0] > frame_aabb[0][0] - 1e-4,
            details=f"sash x=[{open_aabb[0][0]:.3f},{open_aabb[1][0]:.3f}] frame x=[{frame_aabb[0][0]:.3f},{frame_aabb[1][0]:.3f}]",
        )

    # 3. Revolute latch joint exists with correct type and non-trivial range.
    ctx.check(
        "latch joint is revolute",
        latch_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={latch_joint.articulation_type}",
    )
    ctx.check(
        "latch has non-trivial rotation range",
        latch_joint.motion_limits.upper > 0.5,
        details=f"upper={latch_joint.motion_limits.upper:.3f}",
    )

    # Verify latch rotates (pose test).
    latch_pos_rest = ctx.part_world_position(latch)
    with ctx.pose({latch_joint: 1.0}):
        latch_pos_rotated = ctx.part_world_position(latch)
    # Latch origin should stay near the same position (it rotates in place).
    if latch_pos_rest is not None and latch_pos_rotated is not None:
        ctx.check(
            "latch rotates in place (origin stays near pivot)",
            abs(latch_pos_rotated[0] - latch_pos_rest[0]) < 0.05
            and abs(latch_pos_rotated[2] - latch_pos_rest[2]) < 0.05,
            details=f"rest={latch_pos_rest}, rotated={latch_pos_rotated}",
        )

    # 4. Roller blocks: left sash extends below the clear opening bottom
    # (rollers protrude below the bottom rail).
    opening_bottom = INNER_Z0
    ctx.check(
        "left sash has roller blocks extending below sash frame",
        rest_aabb[0][2] < opening_bottom + SASH_FACE - ROLLER_H * 0.5,
        details=f"sash zmin={rest_aabb[0][2]:.4f}, expected below ~{opening_bottom + SASH_FACE - ROLLER_H * 0.5:.4f}",
    )

    # 5. Sill lip: frame extends forward (+Y) beyond the main frame depth at
    # the bottom.
    frame_y_max = frame_aabb[1][1]
    main_frame_y_max = FRAME_DEPTH / 2.0
    ctx.check(
        "sill lip protrudes forward beyond main frame depth",
        frame_y_max > main_frame_y_max + SILL_LIP_DEPTH * 0.5,
        details=f"frame_y_max={frame_y_max:.4f}, main_frame_y_max={main_frame_y_max:.4f}",
    )

    # 6. Three-panel ordering: left < center < right in X.
    lx = (l_aabb[0][0] + l_aabb[1][0]) / 2.0
    ccx = (c_aabb[0][0] + c_aabb[1][0]) / 2.0
    rx = (r_aabb[0][0] + r_aabb[1][0]) / 2.0
    ctx.check(
        "panels ordered left-center-right",
        lx < ccx < rx,
        details=f"left_x={lx:.3f}, center_x={ccx:.3f}, right_x={rx:.3f}",
    )

    # 7. Frame spans full window dimensions.
    frame_w = frame_aabb[1][0] - frame_aabb[0][0]
    frame_h = frame_aabb[1][2] - frame_aabb[0][2]
    ctx.check(
        "frame spans full width",
        abs(frame_w - TOTAL_W) < 0.02,
        details=f"frame_w={frame_w:.3f}",
    )
    ctx.check(
        "frame spans full height",
        abs(frame_h - TOTAL_H) < 0.02,
        details=f"frame_h={frame_h:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
