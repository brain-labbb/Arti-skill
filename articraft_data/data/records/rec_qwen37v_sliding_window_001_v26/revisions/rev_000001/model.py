from __future__ import annotations

# Variant 26: Three-panel horizontal sliding window with slim vinyl frame rails,
# bevelled (chamfered) outer corners, tilt-in latch pair on revolute joints,
# two roller blocks at the bottom of the moving sash, and a visible overlap
# stile on the meeting edge where the sliding pane crosses the fixed pane.
#
# Coordinate convention:
#   +Z up, window stands vertically.
#   width  -> X,  height -> Z,  frame depth / glazing thickness -> Y
#   Glass plane is X-Z. Window reads SHUT at q=0.

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

TOTAL_W = 3.00
TOTAL_H = 1.50

# Slim frame rails (reduced from 0.070 / 0.060)
FRAME_FACE = 0.048
MULLION_FACE = 0.045
FRAME_DEPTH = 0.100
CORNER_CHAMFER = 0.008          # bevel on outer vertical edges

# Three lite columns (recalculated for slim frame)
# inner_w = 3.0 - 2*0.048 = 2.904; minus 2*0.045 mullions = 2.814
SIDE_LITE_W = 0.880
CENTER_LITE_W = 1.054           # 2.814 - 2*0.880

# Sash construction
SASH_FACE = 0.044
SASH_DEPTH = 0.055
GLASS_T = 0.008

# Colonial grille
GRILLE_COLS = 4
GRILLE_ROWS = 5
MUNTIN_T = 0.020
MUNTIN_DEPTH = 0.020

# Y layout (depth). Frame centered on y=0.
FIXED_LITE_Y = -0.020
SLIDE_SASH_Y = 0.052

REBATE = 0.005

# Tilt-in latch
LATCH_LEN = 0.045
LATCH_W = 0.014
LATCH_T = 0.008

# Roller blocks
ROLLER_W = 0.022
ROLLER_H = 0.015
ROLLER_D = 0.020

# Overlap stile (wider meeting stile on the sliding sash)
OVERLAP_STILE_EXTRA_W = 0.018

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
HARDWARE_RGBA = (0.18, 0.18, 0.22, 1.0)   # dark grey for latches / rollers


# ---------------------------------------------------------------------------
# Geometry helpers (CadQuery, meters, world frame)
# ---------------------------------------------------------------------------

def _slab(x0: float, x1: float, z0: float, z1: float,
          y_center: float, depth: float) -> cq.Workplane:
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
    """Slim outer frame with bevelled (chamfered) outer corners, then three
    lite openings cut through to leave head, sill, jambs and two mullions."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)
    # Bevel the four outer vertical edges (read as mitred vinyl corners).
    outer = outer.edges("|Z").chamfer(CORNER_CHAMFER)
    cut_depth = FRAME_DEPTH + 0.02
    left_cut = _slab(LEFT_X0, LEFT_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    center_cut = _slab(CENTER_X0, CENTER_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    right_cut = _slab(RIGHT_X0, RIGHT_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    return outer.cut(left_cut).cut(center_cut).cut(right_cut)


def _build_sash_grille_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """Sash ring + colonial muntin grille in sash-local frame (centered)."""
    ow = opening_w
    oh = opening_h
    out_w = ow + 2 * SASH_FACE
    out_h = oh + 2 * SASH_FACE

    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0,
                  0.0, SASH_DEPTH)
    opening = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0,
                    0.0, SASH_DEPTH + 0.02)
    ring = outer.cut(opening)

    bars = None
    for c in range(1, GRILLE_COLS):
        frac = c / GRILLE_COLS
        x = -ow / 2.0 + frac * ow
        bar = _slab(x - MUNTIN_T / 2.0, x + MUNTIN_T / 2.0,
                    -oh / 2.0, oh / 2.0, 0.0, MUNTIN_DEPTH)
        bars = bar if bars is None else bars.union(bar)
    for r in range(1, GRILLE_ROWS):
        frac = r / GRILLE_ROWS
        z = -oh / 2.0 + frac * oh
        bar = _slab(-ow / 2.0, ow / 2.0,
                    z - MUNTIN_T / 2.0, z + MUNTIN_T / 2.0,
                    0.0, MUNTIN_DEPTH)
        bars = bar if bars is None else bars.union(bar)
    return ring if bars is None else ring.union(bars)


def _build_sash_glass_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    ow = opening_w + 2 * REBATE
    oh = opening_h + 2 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


def _build_overlap_stile_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """Wider meeting stile on the right (leading) edge of the sliding sash,
    visible where the sliding pane crosses the adjacent fixed pane."""
    stile_w = SASH_FACE + OVERLAP_STILE_EXTRA_W
    stile_x_center = opening_w / 2.0 + stile_w / 2.0
    stile_h = opening_h + 2 * SASH_FACE
    stile_d = SASH_DEPTH + 0.006       # slightly deeper than sash
    stile_y_center = 0.003             # protrude toward +Y (exterior)
    return (
        cq.Workplane("XY")
        .transformed(offset=(stile_x_center, stile_y_center, 0.0))
        .box(stile_w, stile_d, stile_h)
    )


def _build_roller_shape(x_pos: float, opening_h: float) -> cq.Workplane:
    """One small roller block housing, positioned at the bottom of the sash
    in sash-local coordinates. Protrudes 2 mm below the bottom rail so the
    roller mesh intersects the rail mesh (connected geometry)."""
    rail_bottom = -(opening_h / 2.0 + SASH_FACE)
    roller_z = rail_bottom - 0.002 + ROLLER_H / 2.0
    return (
        cq.Workplane("XY")
        .transformed(offset=(x_pos, 0.0, roller_z))
        .box(ROLLER_W, ROLLER_D, ROLLER_H)
    )


def _build_latch_shape() -> cq.Workplane:
    """Small lever extending along +X from the pivot at the local origin.
    Thin in Z (lies flat on the rail face at q=0)."""
    return (
        cq.Workplane("XY")
        .transformed(offset=(LATCH_LEN / 2.0, 0.0, 0.0))
        .box(LATCH_LEN, LATCH_W, LATCH_T)
    )


# ---------------------------------------------------------------------------
# Part builders
# ---------------------------------------------------------------------------

def _add_fixed_lite(model: ArticulatedObject, name: str,
                    opening_w: float, opening_h: float) -> None:
    lite = model.part(name)
    lite.visual(
        mesh_from_cadquery(
            _build_sash_grille_shape(opening_w, opening_h), f"{name}_vinyl"),
        material="vinyl", name=f"{name}_vinyl",
    )
    lite.visual(
        mesh_from_cadquery(
            _build_sash_glass_shape(opening_w, opening_h), f"{name}_glass"),
        material="glass", name=f"{name}_glass",
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    span = SIDE_LITE_W + MULLION_FACE + CENTER_LITE_W + MULLION_FACE + SIDE_LITE_W
    inner_w = INNER_X1 - INNER_X0
    assert abs(span - inner_w) < 1e-6, f"lite layout {span} != inner width {inner_w}"

    model = ArticulatedObject(name="sliding_window_v26")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("hardware", rgba=HARDWARE_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame_shell"),
        material="vinyl", name="frame_shell",
    )

    opening_h = INNER_Z1 - INNER_Z0

    # --- Two FIXED side lites ---
    _add_fixed_lite(model, "left_lite", SIDE_LITE_W, opening_h)
    _add_fixed_lite(model, "right_lite", SIDE_LITE_W, opening_h)

    # --- CENTER sliding sash (with overlap stile + rollers) ---
    center_sash = model.part("center_sash")
    center_sash.visual(
        mesh_from_cadquery(
            _build_sash_grille_shape(CENTER_LITE_W, opening_h),
            "center_sash_vinyl"),
        material="vinyl", name="center_sash_vinyl",
    )
    center_sash.visual(
        mesh_from_cadquery(
            _build_sash_glass_shape(CENTER_LITE_W, opening_h),
            "center_sash_glass"),
        material="glass", name="center_sash_glass",
    )
    # Overlap stile on the right (meeting) edge
    center_sash.visual(
        mesh_from_cadquery(
            _build_overlap_stile_shape(CENTER_LITE_W, opening_h),
            "overlap_stile"),
        material="vinyl", name="overlap_stile",
    )
    # Two roller blocks at the bottom of the sash
    roller_left_x = -CENTER_LITE_W / 3.0
    roller_right_x = CENTER_LITE_W / 3.0
    center_sash.visual(
        mesh_from_cadquery(
            _build_roller_shape(roller_left_x, opening_h), "roller_left"),
        material="hardware", name="roller_left",
    )
    center_sash.visual(
        mesh_from_cadquery(
            _build_roller_shape(roller_right_x, opening_h), "roller_right"),
        material="hardware", name="roller_right",
    )

    # --- Tilt-in latch pair (separate parts on revolute joints) ---
    tilt_latch_top = model.part("tilt_latch_top")
    tilt_latch_top.visual(
        mesh_from_cadquery(_build_latch_shape(), "tilt_latch_top_body"),
        material="hardware", name="tilt_latch_top_body",
    )

    tilt_latch_bottom = model.part("tilt_latch_bottom")
    tilt_latch_bottom.visual(
        mesh_from_cadquery(_build_latch_shape(), "tilt_latch_bottom_body"),
        material="hardware", name="tilt_latch_bottom_body",
    )

    # --- World centers ---
    left_cx = (LEFT_X0 + LEFT_X1) / 2.0
    right_cx = (RIGHT_X0 + RIGHT_X1) / 2.0
    center_cx = (CENTER_X0 + CENTER_X1) / 2.0
    mid_cz = (INNER_Z0 + INNER_Z1) / 2.0

    # FIXED side lites
    model.articulation(
        "frame_to_left_lite",
        ArticulationType.FIXED,
        parent="frame", child="left_lite",
        origin=Origin(xyz=(left_cx, FIXED_LITE_Y, mid_cz)),
    )
    model.articulation(
        "frame_to_right_lite",
        ArticulationType.FIXED,
        parent="frame", child="right_lite",
        origin=Origin(xyz=(right_cx, FIXED_LITE_Y, mid_cz)),
    )

    # CENTER sliding sash: PRISMATIC along +X
    slide_travel = SIDE_LITE_W * 0.92
    model.articulation(
        "frame_to_center_sash",
        ArticulationType.PRISMATIC,
        parent="frame", child="center_sash",
        origin=Origin(xyz=(center_cx, SLIDE_SASH_Y, mid_cz)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=0.5, lower=0.0, upper=slide_travel),
    )

    # Tilt-in latch TOP: on the top rail, left side of the sash.
    # Pivot axis = Y (perpendicular to glass). At q=0 the lever lies flat
    # along the rail; positive q flips the free end upward (+Z).
    latch_top_origin = Origin(xyz=(
        -CENTER_LITE_W / 3.0,
        SASH_DEPTH / 2.0,                   # on the sash front face
        opening_h / 2.0 + SASH_FACE / 2.0,  # center of top rail
    ))
    model.articulation(
        "sash_to_latch_top",
        ArticulationType.REVOLUTE,
        parent="center_sash", child="tilt_latch_top",
        origin=latch_top_origin,
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=2.0, lower=0.0, upper=1.2),
    )

    # Tilt-in latch BOTTOM: on the top rail, right side of the sash.
    latch_bottom_origin = Origin(xyz=(
        CENTER_LITE_W / 3.0,
        SASH_DEPTH / 2.0,
        opening_h / 2.0 + SASH_FACE / 2.0,
    ))
    model.articulation(
        "sash_to_latch_bottom",
        ArticulationType.REVOLUTE,
        parent="center_sash", child="tilt_latch_bottom",
        origin=latch_bottom_origin,
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=2.0, lower=0.0, upper=1.2),
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
    tilt_latch_top = object_model.get_part("tilt_latch_top")
    tilt_latch_bottom = object_model.get_part("tilt_latch_bottom")

    slide = object_model.get_articulation("frame_to_center_sash")
    latch_top_joint = object_model.get_articulation("sash_to_latch_top")
    latch_bottom_joint = object_model.get_articulation("sash_to_latch_bottom")

    # ---- Intentional overlaps ----

    # Glass rebated under the sash/muntin lip on each sash (captured glazing).
    for nm in ("left_lite", "right_lite", "center_sash"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass", elem_b=f"{nm}_vinyl",
            reason="Clear pane is rebated under the sash/muntin lip.",
        )

    # Fixed lites rebated into the frame opening.
    for nm in ("left_lite", "right_lite"):
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell", elem_b=f"{nm}_vinyl",
            reason=f"{nm} sash ring laps the jamb/mullion edge (seated capture).",
        )
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell", elem_b=f"{nm}_glass",
            reason=f"{nm} glass rebated under frame opening lip.",
        )

    # Center sash rides the head/sill track.
    ctx.allow_overlap(
        "frame", "center_sash",
        elem_a="frame_shell", elem_b="center_sash_vinyl",
        reason="Center sash rides the head/sill track; slider capture.",
    )
    ctx.allow_overlap(
        "frame", "center_sash",
        elem_a="frame_shell", elem_b="center_sash_glass",
        reason="Center sash glass laps the track lip.",
    )

    # Overlap stile at the meeting edge (intentionally in the track zone).
    ctx.allow_overlap(
        "frame", "center_sash",
        elem_a="frame_shell", elem_b="overlap_stile",
        reason="Overlap stile is intentionally nested in the frame track at the meeting edge.",
    )

    # Roller blocks sit in the sill track channel.
    ctx.allow_overlap(
        "frame", "center_sash",
        elem_a="frame_shell", elem_b="roller_left",
        reason="Roller housing sits in the sill track channel.",
    )
    ctx.allow_overlap(
        "frame", "center_sash",
        elem_a="frame_shell", elem_b="roller_right",
        reason="Roller housing sits in the sill track channel.",
    )

    # Tilt latches captured on the sash rail surface (pivot embed).
    ctx.allow_overlap(
        "center_sash", "tilt_latch_top",
        elem_a="center_sash_vinyl", elem_b="tilt_latch_top_body",
        reason="Tilt latch pivot captured on the sash top rail.",
    )
    ctx.allow_overlap(
        "center_sash", "tilt_latch_bottom",
        elem_a="center_sash_vinyl", elem_b="tilt_latch_bottom_body",
        reason="Tilt latch pivot captured on the sash top rail.",
    )

    # ---- Variant-specific checks ----

    with ctx.pose({slide: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        l_aabb = ctx.part_world_aabb(left_lite)
        r_aabb = ctx.part_world_aabb(right_lite)
        c_aabb = ctx.part_world_aabb(center_sash)

        # Slim frame: overall width/height still correct.
        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        frame_h = frame_aabb[1][2] - frame_aabb[0][2]
        frame_d = frame_aabb[1][1] - frame_aabb[0][1]

        ctx.check(
            "frame width matches TOTAL_W",
            abs(frame_w - TOTAL_W) < 0.02,
            details=f"frame_w={frame_w:.3f}",
        )
        ctx.check(
            "frame height matches TOTAL_H",
            abs(frame_h - TOTAL_H) < 0.02,
            details=f"frame_h={frame_h:.3f}",
        )

        # Slim rail check: frame depth is the slim dimension (< original 0.110).
        ctx.check(
            "slim frame depth",
            frame_d < 0.110,
            details=f"frame_d={frame_d:.4f}",
        )
        # The frame face width is slim (< original 0.070). Verify via the
        # gap between the outer frame edge and the nearest lite edge.
        left_lite_xmin = l_aabb[0][0]
        frame_xmin = frame_aabb[0][0]
        rail_visible_gap = left_lite_xmin - frame_xmin
        ctx.check(
            "slim jamb rail face < 0.060",
            rail_visible_gap < 0.060,
            details=f"jamb_gap={rail_visible_gap:.4f}",
        )

        # Sill at z~0, head at z~TOTAL_H.
        ctx.check(
            "sill near z=0",
            abs(frame_aabb[0][2]) < 0.02,
            details=f"zmin={frame_aabb[0][2]:.4f}",
        )
        ctx.check(
            "head near TOTAL_H",
            abs(frame_aabb[1][2] - TOTAL_H) < 0.02,
            details=f"zmax={frame_aabb[1][2]:.4f}",
        )

        # Three lites ordered left -> center -> right.
        lx = (l_aabb[0][0] + l_aabb[1][0]) / 2.0
        cx = (c_aabb[0][0] + c_aabb[1][0]) / 2.0
        rx = (r_aabb[0][0] + r_aabb[1][0]) / 2.0
        ctx.check(
            "lites ordered left-center-right",
            lx < cx < rx,
            details=f"left_x={lx:.3f}, center_x={cx:.3f}, right_x={rx:.3f}",
        )

        # Center sash proud of side lites in +Y.
        l_y = (l_aabb[0][1] + l_aabb[1][1]) / 2.0
        c_y = (c_aabb[0][1] + c_aabb[1][1]) / 2.0
        ctx.check(
            "center sash proud of side lites",
            c_y > l_y + 0.02,
            details=f"center_y={c_y:.3f}, side_y={l_y:.3f}",
        )

        # All lites within frame height.
        for nm, ab in (("left", l_aabb), ("right", r_aabb), ("center", c_aabb)):
            ctx.check(
                f"{nm} lite within frame height",
                ab[0][2] > frame_aabb[0][2] - 1e-4
                and ab[1][2] < frame_aabb[1][2] + 1e-4,
                details=f"{nm} z=[{ab[0][2]:.3f},{ab[1][2]:.3f}]",
            )

        # Fixed lites seated in frame.
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

        # --- Overlap stile exists on the right edge of the center sash ---
        ctx.check(
            "overlap_stile visual exists",
            center_sash.get_visual("overlap_stile") is not None,
            details="overlap_stile not found on center_sash",
        )
        # The overlap stile extends past the regular sash width on the right.
        sash_xmax = c_aabb[1][0]
        ctx.check(
            "sash right edge includes overlap stile extent",
            sash_xmax > cx + CENTER_LITE_W / 2.0 + SASH_FACE,
            details=f"sash_xmax={sash_xmax:.3f}, expected>{cx + CENTER_LITE_W/2 + SASH_FACE:.3f}",
        )

        # --- Roller blocks exist at the bottom of the sash ---
        ctx.check(
            "roller_left visual exists",
            center_sash.get_visual("roller_left") is not None,
            details="roller_left not found on center_sash",
        )
        ctx.check(
            "roller_right visual exists",
            center_sash.get_visual("roller_right") is not None,
            details="roller_right not found on center_sash",
        )

    # ---- Tilt-in latch pair checks ----

    # Latches exist and have revolute joints.
    ctx.check(
        "latch_top joint is revolute",
        latch_top_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={latch_top_joint.articulation_type}",
    )
    ctx.check(
        "latch_bottom joint is revolute",
        latch_bottom_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={latch_bottom_joint.articulation_type}",
    )

    # Latch pivot: at q=0 the lever lies flat; at q=1.0 the free end rises.
    with ctx.pose({slide: 0.0, latch_top_joint: 0.0}):
        latch_rest_aabb = ctx.part_world_aabb(tilt_latch_top)
        rest_z_max = latch_rest_aabb[1][2]

    with ctx.pose({slide: 0.0, latch_top_joint: 1.0}):
        latch_open_aabb = ctx.part_world_aabb(tilt_latch_top)
        open_z_max = latch_open_aabb[1][2]

    ctx.check(
        "top latch pivots upward (z_max increases)",
        open_z_max > rest_z_max + 0.010,
        details=f"rest_zmax={rest_z_max:.4f}, open_zmax={open_z_max:.4f}",
    )

    # Same for the bottom latch.
    with ctx.pose({slide: 0.0, latch_bottom_joint: 0.0}):
        blatch_rest_aabb = ctx.part_world_aabb(tilt_latch_bottom)
        blatch_rest_zmax = blatch_rest_aabb[1][2]

    with ctx.pose({slide: 0.0, latch_bottom_joint: 1.0}):
        blatch_open_aabb = ctx.part_world_aabb(tilt_latch_bottom)
        blatch_open_zmax = blatch_open_aabb[1][2]

    ctx.check(
        "bottom latch pivots upward (z_max increases)",
        blatch_open_zmax > blatch_rest_zmax + 0.010,
        details=f"rest_zmax={blatch_rest_zmax:.4f}, open_zmax={blatch_open_zmax:.4f}",
    )

    # ---- Slide mechanism ----

    travel = slide.motion_limits.upper
    with ctx.pose({slide: travel}):
        c_open = ctx.part_world_aabb(center_sash)
        open_cx = (c_open[0][0] + c_open[1][0]) / 2.0

        ctx.check(
            "center sash slides along +X by ~travel",
            abs((open_cx - rest_cx) - travel) < 0.02,
            details=f"rest_cx={rest_cx:.3f}, open_cx={open_cx:.3f}, travel={travel:.3f}",
        )
        # Pure horizontal slide (no Z motion).
        c_open_z = (c_open[0][2] + c_open[1][2]) / 2.0
        ctx.check(
            "slide is purely horizontal",
            abs(c_open_z - rest_cz) < 0.02,
            details=f"open_z={c_open_z:.3f}, rest_z={rest_cz:.3f}",
        )
        # Retained insertion at full travel.
        f_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "sash retained within frame X span at full travel",
            c_open[1][0] < f_aabb[1][0] + 1e-4
            and c_open[0][0] > f_aabb[0][0] - 1e-4,
            details=f"sash x=[{c_open[0][0]:.3f},{c_open[1][0]:.3f}]"
                    f" frame x=[{f_aabb[0][0]:.3f},{f_aabb[1][0]:.3f}]",
        )
        ctx.expect_overlap(
            center_sash, frame, axes="z", min_overlap=0.10,
            name="sash retains vertical engagement with head/sill track",
        )

    return ctx.report()


object_model = build_object_model()
