from __future__ import annotations

# Three-panel horizontal sliding window variant (Variant 10):
# White vinyl frame with colonial divided-lite grilles.
# - Left lite: FIXED in the rear track.
# - Center sash: PRISMATIC along +X on the front track, PARTIALLY OPEN at rest
#   (shifted ~0.30 m right) so the overlap with the right panel is visible.
# - Right sash: PRISMATIC along -X on the rear track, closed at rest.
# - Two roller blocks at the bottom of each moving sash.
# - Frame has a protruding sill lip and drainage (weep) slots cut into the sill.

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
MULLION_FACE = 0.060
FRAME_DEPTH = 0.110

SIDE_LITE_W = 0.85
CENTER_LITE_W = 1.04

SASH_FACE = 0.055
SASH_DEPTH = 0.055
GLASS_T = 0.008

GRILLE_COLS = 4
GRILLE_ROWS = 5
MUNTIN_T = 0.020
MUNTIN_DEPTH = 0.020

FIXED_LITE_Y = -0.020       # rear track
SLIDE_SASH_Y = 0.052        # front track (proud)

REBATE = 0.005

# Variant-specific: partial open offset for center sash at rest
PARTIAL_OPEN_OFFSET = 0.30   # center sash starts shifted right at q=0

# Sill lip
SILL_LIP_EXTEND = 0.040     # how far the lip protrudes in +Y
SILL_LIP_THICK = 0.015      # thickness in Z

# Drainage (weep) slots
DRAIN_W = 0.040             # slot width along X
DRAIN_H = 0.010             # slot height along Z
N_DRAIN_SLOTS = 5

# Roller blocks
ROLLER_W = 0.028
ROLLER_D = 0.022
ROLLER_H = 0.014

# ---------------------------------------------------------------------------
# Derived layout
# ---------------------------------------------------------------------------

HALF_W = TOTAL_W / 2.0

INNER_X0 = -HALF_W + FRAME_FACE
INNER_X1 = HALF_W - FRAME_FACE
INNER_Z0 = FRAME_FACE
INNER_Z1 = TOTAL_H - FRAME_FACE

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
ROLLER_RGBA = (0.25, 0.25, 0.27, 1.0)  # dark grey nylon roller


# ---------------------------------------------------------------------------
# Geometry helpers (CadQuery, meters, world frame)
# ---------------------------------------------------------------------------

def _slab(x0: float, x1: float, z0: float, z1: float, y_center: float, depth: float) -> cq.Workplane:
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
    """Static outer frame: slab cut by three lite openings + sill lip + weep slots."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)

    cut_depth = FRAME_DEPTH + 0.02
    left_cut = _slab(LEFT_X0, LEFT_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    center_cut = _slab(CENTER_X0, CENTER_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    right_cut = _slab(RIGHT_X0, RIGHT_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)

    frame = outer.cut(left_cut).cut(center_cut).cut(right_cut)

    # --- Sill lip: protruding shelf on the exterior (+Y) face at the bottom ---
    lip_y_center = FRAME_DEPTH / 2.0 + SILL_LIP_EXTEND / 2.0
    sill_lip = _slab(
        -HALF_W, HALF_W,
        FRAME_FACE * 0.25, FRAME_FACE * 0.25 + SILL_LIP_THICK,
        lip_y_center, SILL_LIP_EXTEND,
    )
    frame = frame.union(sill_lip)

    # --- Drainage (weep) slots cut through the sill front face ---
    for i in range(N_DRAIN_SLOTS):
        frac = (i + 0.5) / N_DRAIN_SLOTS
        x = INNER_X0 + frac * (INNER_X1 - INNER_X0)
        slot_z_lo = FRAME_FACE * 0.35
        slot_z_hi = slot_z_lo + DRAIN_H
        slot_y_center = FRAME_DEPTH / 2.0  # through the front face region
        slot_depth = SILL_LIP_EXTEND + 0.02  # through-cut past the lip
        slot = _slab(
            x - DRAIN_W / 2.0, x + DRAIN_W / 2.0,
            slot_z_lo, slot_z_hi,
            slot_y_center, slot_depth,
        )
        frame = frame.cut(slot)

    return frame


def _build_sash_grille_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """Sash vinyl ring + colonial muntin grid in local frame centered on origin."""
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


def _build_sash_glass_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    ow = opening_w + 2 * REBATE
    oh = opening_h + 2 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


def _build_roller_pair(opening_w: float, opening_h: float) -> cq.Workplane:
    """Two small roller blocks at the bottom of the sash in sash-local frame.
    Positioned near the left and right stiles, half-embedded in the bottom rail
    for connectivity, half protruding below as visible rollers."""
    bottom_z = -(opening_h / 2.0 + SASH_FACE)
    # Roller center is slightly below the sash bottom so top embeds in the rail
    roller_cz = bottom_z - ROLLER_H * 0.25
    left_x = -(opening_w / 2.0 + SASH_FACE * 0.5)
    right_x = (opening_w / 2.0 + SASH_FACE * 0.5)

    left_roller = (
        cq.Workplane("XY")
        .transformed(offset=(left_x, 0.0, roller_cz))
        .box(ROLLER_W, ROLLER_D, ROLLER_H)
    )
    right_roller = (
        cq.Workplane("XY")
        .transformed(offset=(right_x, 0.0, roller_cz))
        .box(ROLLER_W, ROLLER_D, ROLLER_H)
    )
    return left_roller.union(right_roller)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    span = SIDE_LITE_W + MULLION_FACE + CENTER_LITE_W + MULLION_FACE + SIDE_LITE_W
    inner_w = INNER_X1 - INNER_X0
    assert abs(span - inner_w) < 1e-6, f"lite layout {span} != inner width {inner_w}"

    model = ArticulatedObject(name="three_panel_sliding_window_variant")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("roller", rgba=ROLLER_RGBA)

    # --- Static outer frame (root) with sill lip and weep slots ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material="vinyl",
        name="frame_shell",
    )

    opening_h = INNER_Z1 - INNER_Z0

    # --- Left lite: FIXED on rear track ---
    left_lite = model.part("left_lite")
    left_lite.visual(
        mesh_from_cadquery(_build_sash_grille_shape(SIDE_LITE_W, opening_h), "left_lite_vinyl"),
        material="vinyl",
        name="left_lite_vinyl",
    )
    left_lite.visual(
        mesh_from_cadquery(_build_sash_glass_shape(SIDE_LITE_W, opening_h), "left_lite_glass"),
        material="glass",
        name="left_lite_glass",
    )

    # --- Center sash: PRISMATIC on front track, partially open at rest ---
    center_sash = model.part("center_sash")
    center_sash.visual(
        mesh_from_cadquery(_build_sash_grille_shape(CENTER_LITE_W, opening_h), "center_sash_vinyl"),
        material="vinyl",
        name="center_sash_vinyl",
    )
    center_sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(CENTER_LITE_W, opening_h), "center_sash_glass"),
        material="glass",
        name="center_sash_glass",
    )
    center_sash.visual(
        mesh_from_cadquery(_build_roller_pair(CENTER_LITE_W, opening_h), "center_rollers"),
        material="roller",
        name="center_rollers",
    )

    # --- Right sash: PRISMATIC on rear track, slides -X, closed at rest ---
    right_sash = model.part("right_sash")
    right_sash.visual(
        mesh_from_cadquery(_build_sash_grille_shape(SIDE_LITE_W, opening_h), "right_sash_vinyl"),
        material="vinyl",
        name="right_sash_vinyl",
    )
    right_sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(SIDE_LITE_W, opening_h), "right_sash_glass"),
        material="glass",
        name="right_sash_glass",
    )
    right_sash.visual(
        mesh_from_cadquery(_build_roller_pair(SIDE_LITE_W, opening_h), "right_rollers"),
        material="roller",
        name="right_rollers",
    )

    # Centers of each opening
    left_cx = (LEFT_X0 + LEFT_X1) / 2.0
    right_cx = (RIGHT_X0 + RIGHT_X1) / 2.0
    center_cx = (CENTER_X0 + CENTER_X1) / 2.0
    mid_cz = (INNER_Z0 + INNER_Z1) / 2.0

    # FIXED left lite on rear track
    model.articulation(
        "frame_to_left_lite",
        ArticulationType.FIXED,
        parent="frame",
        child="left_lite",
        origin=Origin(xyz=(left_cx, FIXED_LITE_Y, mid_cz)),
    )

    # CENTER sliding sash: PRISMATIC along +X on front track.
    # Origin is shifted right by PARTIAL_OPEN_OFFSET so at q=0 the sash is
    # already partially open, exposing the left portion of the center opening.
    # Travel is limited so the sash stays retained within the frame at max q.
    sash_half_w = CENTER_LITE_W / 2.0 + SASH_FACE
    max_retained_travel = (INNER_X1 - sash_half_w) - (center_cx + PARTIAL_OPEN_OFFSET)
    slide_travel = min(SIDE_LITE_W * 0.92, max_retained_travel - 0.01)
    model.articulation(
        "frame_to_center_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="center_sash",
        origin=Origin(xyz=(center_cx + PARTIAL_OPEN_OFFSET, SLIDE_SASH_Y, mid_cz)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.5, lower=0.0, upper=slide_travel),
    )

    # RIGHT sliding sash: PRISMATIC along -X on rear track.
    # At q=0 it is closed (centered in the right opening).
    # Positive q slides it leftward (behind the center sash on the front track).
    right_slide_travel = SIDE_LITE_W * 0.92
    model.articulation(
        "frame_to_right_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="right_sash",
        origin=Origin(xyz=(right_cx, FIXED_LITE_Y, mid_cz)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.5, lower=0.0, upper=right_slide_travel),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    left_lite = object_model.get_part("left_lite")
    center_sash = object_model.get_part("center_sash")
    right_sash = object_model.get_part("right_sash")
    center_slide = object_model.get_articulation("frame_to_center_sash")
    right_slide = object_model.get_articulation("frame_to_right_sash")

    # --- Verify both sliding joints are non-fixed (prismatic) ---
    ctx.check(
        "center sash joint is prismatic",
        center_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={center_slide.articulation_type}",
    )
    ctx.check(
        "right sash joint is prismatic",
        right_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={right_slide.articulation_type}",
    )

    # --- Verify joints have opposite axes ---
    ctx.check(
        "center sash slides along +X",
        center_slide.axis[0] > 0.5,
        details=f"axis={center_slide.axis}",
    )
    ctx.check(
        "right sash slides along -X",
        right_slide.axis[0] < -0.5,
        details=f"axis={right_slide.axis}",
    )

    # --- Intentional overlaps ---
    # Glass captured under sash/muntin lip on each sash
    for nm, glass_nm, vinyl_nm in [
        ("left_lite", "left_lite_glass", "left_lite_vinyl"),
        ("center_sash", "center_sash_glass", "center_sash_vinyl"),
        ("right_sash", "right_sash_glass", "right_sash_vinyl"),
    ]:
        ctx.allow_overlap(
            nm, nm,
            elem_a=glass_nm,
            elem_b=vinyl_nm,
            reason=f"Glass pane rebated under {nm} sash/muntin lip (captured glazing).",
        )

    # Fixed left lite rebated into frame
    ctx.allow_overlap(
        "frame", "left_lite",
        elem_a="frame_shell",
        elem_b="left_lite_vinyl",
        reason="Left fixed lite rebated into frame opening (seated capture).",
    )
    ctx.allow_overlap(
        "frame", "left_lite",
        elem_a="frame_shell",
        elem_b="left_lite_glass",
        reason="Left lite glass rebated under frame opening lip.",
    )

    # Center sash rides front track, laps frame face
    ctx.allow_overlap(
        "frame", "center_sash",
        elem_a="frame_shell",
        elem_b="center_sash_vinyl",
        reason="Center sash rides head/sill track on front track, laps frame face.",
    )
    ctx.allow_overlap(
        "frame", "center_sash",
        elem_a="frame_shell",
        elem_b="center_sash_glass",
        reason="Center sash glass laps track lip.",
    )

    # Right sash on rear track, laps frame
    ctx.allow_overlap(
        "frame", "right_sash",
        elem_a="frame_shell",
        elem_b="right_sash_vinyl",
        reason="Right sash rides head/sill track on rear track, laps frame face.",
    )
    ctx.allow_overlap(
        "frame", "right_sash",
        elem_a="frame_shell",
        elem_b="right_sash_glass",
        reason="Right sash glass laps track lip.",
    )

    # Roller blocks are half-embedded in sash bottom rail (mounted rollers)
    ctx.allow_overlap(
        "center_sash", "center_sash",
        elem_a="center_rollers",
        elem_b="center_sash_vinyl",
        reason="Roller blocks are mounted into the sash bottom rail (half-embedded capture).",
    )
    ctx.allow_overlap(
        "right_sash", "right_sash",
        elem_a="right_rollers",
        elem_b="right_sash_vinyl",
        reason="Roller blocks are mounted into the sash bottom rail (half-embedded capture).",
    )

    # Roller blocks ride in the sill track; they protrude below the sash bottom
    # rail into the sill member (captured in the track channel).
    ctx.allow_overlap(
        "center_sash", "frame",
        elem_a="center_rollers",
        elem_b="frame_shell",
        reason="Center sash rollers ride in the sill track channel; they are intentionally nested in the sill profile.",
    )
    ctx.allow_overlap(
        "right_sash", "frame",
        elem_a="right_rollers",
        elem_b="frame_shell",
        reason="Right sash rollers ride in the sill track channel; they are intentionally nested in the sill profile.",
    )

    # Center sash (front track) may overlap right sash (rear track) in XY
    # projection when partially open - they are separated in Y.
    ctx.allow_overlap(
        "center_sash", "right_sash",
        reason="Center sash on front track passes in front of right sash on rear track; Y offset prevents real collision.",
    )

    # --- Rest pose (q=0 for both): center sash partially open ---
    with ctx.pose({center_slide: 0.0, right_slide: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        c_aabb = ctx.part_world_aabb(center_sash)
        r_aabb = ctx.part_world_aabb(right_sash)
        l_aabb = ctx.part_world_aabb(left_lite)

        # Frame dimensions
        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        ctx.check(
            "frame spans full window width",
            frame_w > 2.5,
            details=f"frame_w={frame_w:.3f}",
        )
        ctx.check(
            "sill near z=0",
            abs(frame_aabb[0][2]) < 0.02,
            details=f"frame zmin={frame_aabb[0][2]:.4f}",
        )

        # Center sash is partially open: its center is shifted right of the
        # center opening center by approximately PARTIAL_OPEN_OFFSET.
        center_opening_cx = (CENTER_X0 + CENTER_X1) / 2.0
        c_cx = (c_aabb[0][0] + c_aabb[1][0]) / 2.0
        ctx.check(
            "center sash partially open at rest",
            c_cx > center_opening_cx + PARTIAL_OPEN_OFFSET * 0.8,
            details=f"sash_cx={c_cx:.3f}, opening_cx={center_opening_cx:.3f}",
        )

        # Right sash is closed: centered in its opening
        right_opening_cx = (RIGHT_X0 + RIGHT_X1) / 2.0
        r_cx = (r_aabb[0][0] + r_aabb[1][0]) / 2.0
        ctx.check(
            "right sash closed at rest",
            abs(r_cx - right_opening_cx) < 0.05,
            details=f"right_sash_cx={r_cx:.3f}, opening_cx={right_opening_cx:.3f}",
        )

        # Center sash on front track (proud in +Y) vs right sash on rear track
        c_y = (c_aabb[0][1] + c_aabb[1][1]) / 2.0
        r_y = (r_aabb[0][1] + r_aabb[1][1]) / 2.0
        ctx.check(
            "center sash proud of right sash (separate tracks)",
            c_y > r_y + 0.03,
            details=f"center_y={c_y:.3f}, right_y={r_y:.3f}",
        )

        # All sashes within frame height
        for nm, ab in [("center", c_aabb), ("right", r_aabb), ("left", l_aabb)]:
            ctx.check(
                f"{nm} sash within frame height",
                ab[0][2] > frame_aabb[0][2] - 0.01 and ab[1][2] < frame_aabb[1][2] + 0.01,
                details=f"{nm} z=[{ab[0][2]:.3f},{ab[1][2]:.3f}]",
            )

    # --- Center sash slides further right ---
    travel_c = center_slide.motion_limits.upper
    with ctx.pose({center_slide: travel_c, right_slide: 0.0}):
        c_open = ctx.part_world_aabb(center_sash)
        open_cx = (c_open[0][0] + c_open[1][0]) / 2.0
        ctx.check(
            "center sash slides further right when driven",
            open_cx > c_cx + 0.1,
            details=f"rest_cx={c_cx:.3f}, open_cx={open_cx:.3f}",
        )
        # Retained within frame
        f_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "center sash retained in frame at full travel",
            c_open[1][0] < f_aabb[1][0] + 0.05,
            details=f"sash xmax={c_open[1][0]:.3f}, frame xmax={f_aabb[1][0]:.3f}",
        )

    # --- Right sash slides left ---
    travel_r = right_slide.motion_limits.upper
    with ctx.pose({center_slide: 0.0, right_slide: travel_r}):
        r_open = ctx.part_world_aabb(right_sash)
        r_open_cx = (r_open[0][0] + r_open[1][0]) / 2.0
        ctx.check(
            "right sash slides left when driven",
            r_open_cx < r_cx - 0.1,
            details=f"rest_cx={r_cx:.3f}, open_cx={r_open_cx:.3f}",
        )
        # Pure horizontal slide (no Z change)
        r_open_z = (r_open[0][2] + r_open[1][2]) / 2.0
        rest_r_z = (r_aabb[0][2] + r_aabb[1][2]) / 2.0
        ctx.check(
            "right sash slide is purely horizontal",
            abs(r_open_z - rest_r_z) < 0.02,
            details=f"open_z={r_open_z:.3f}, rest_z={rest_r_z:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
