from __future__ import annotations

# Three-panel horizontal sliding window variant: white vinyl/PVC frame with
# colonial divided-lite grilles, deep track grooves in head and sill, an outer
# insect screen panel in a separate exterior track, and a tilt-in latch pair
# on the center sliding sash.
#
# Coordinate convention:
#   +Z is up. Window stands vertically.
#     - width  -> X
#     - height -> Z   (sill near z=0)
#     - frame depth / glazing thickness / slide-normal -> Y
#   The glass plane is the X-Z plane. +Y is toward the interior.
#
# Structure:
#   - frame (static root): head, sill, two jambs, two intermediate mullions
#     with deep track grooves cut into head bottom and sill top.
#   - left_lite, right_lite (FIXED): vinyl sash ring + colonial muntin grille
#     + clear glass, seated in the rear glazing plane.
#   - center_sash (SLIDING): same construction, proud of the side lites in +Y;
#     PRISMATIC along +X. Carries the tilt-in latch pair.
#   - screen_panel (FIXED): thin aluminum frame + insect mesh, mounted in the
#     exterior screen track on the -Y side of the frame.
#   - tilt_latch_top, tilt_latch_bottom: small lever latches on the center sash
#     stile, each on a REVOLUTE joint pivoting around X to release the sash
#     for tilt-in cleaning.

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
TOTAL_H = 1.50            # overall height along Z (sill at z=0, head at z=TOTAL_H)

FRAME_FACE = 0.070        # outer frame member face width (jamb / head / sill)
MULLION_FACE = 0.060      # intermediate mullion face width
FRAME_DEPTH = 0.110       # outer frame depth along Y (chunky vinyl box section)

# Three lite columns.
SIDE_LITE_W = 0.85        # clear opening width of each side lite
CENTER_LITE_W = 1.04      # clear opening width of the center lite

# Sash construction
SASH_FACE = 0.055         # sash perimeter rail/stile face width (in-plane)
SASH_DEPTH = 0.055        # sash depth along Y
GLASS_T = 0.008           # glazing thickness along Y

# Colonial grille (divided lite)
GRILLE_COLS = 4           # 4 columns of panes
GRILLE_ROWS = 5           # 5 rows of panes
MUNTIN_T = 0.020          # muntin bar face width
MUNTIN_DEPTH = 0.020      # muntin bar depth along Y

# Y layout (depth). Frame box centered on y=0.
FIXED_LITE_Y = -0.020     # fixed side lites: rear glazing plane center (Y)
SLIDE_SASH_Y = 0.052      # center sash sits proud toward +Y (passes in front)

REBATE = 0.005            # glass tucks under the sash/muntin lip

# --- Deep track grooves ---
TRACK_GROOVE_W = 0.015    # groove width in Y
TRACK_GROOVE_DEPTH = 0.012  # groove depth in Z (into head/sill member)

# Three track groove Y centers:
#   rear track (for fixed lites), front track (for sliding sash),
#   screen track (exterior, for insect screen)
GROOVE_REAR_Y = -0.025
GROOVE_FRONT_Y = 0.025
GROOVE_SCREEN_Y = -0.048

# --- Insect screen panel ---
SCREEN_FRAME_W = 0.022    # screen frame member face width
SCREEN_FRAME_DEPTH = 0.015  # screen frame depth along Y
SCREEN_MESH_T = 0.002     # insect mesh thickness
# Screen sits on the exterior (-Y) side, just outside the frame back face
SCREEN_Y = -FRAME_DEPTH / 2.0 - SCREEN_FRAME_DEPTH / 2.0 + 0.003

# --- Tilt-in latches ---
LATCH_LENGTH = 0.050      # latch lever length (along Z when engaged)
LATCH_WIDTH = 0.014       # latch width (along X)
LATCH_DEPTH = 0.010       # latch thickness (along Y)
LATCH_PIVOT_R = 0.008     # pivot boss radius
LATCH_PIVOT_H = 0.012     # pivot boss height (along X)

# ---------------------------------------------------------------------------
# Derived layout
# ---------------------------------------------------------------------------

HALF_W = TOTAL_W / 2.0

INNER_X0 = -HALF_W + FRAME_FACE
INNER_X1 = HALF_W - FRAME_FACE
INNER_Z0 = FRAME_FACE
INNER_Z1 = TOTAL_H - FRAME_FACE

# Three lite openings laid left -> center -> right with two mullions.
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

VINYL_RGBA = (0.94, 0.95, 0.96, 1.0)     # bright white vinyl/PVC
GLASS_RGBA = (0.50, 0.58, 0.64, 0.32)    # cool grey-blue, semi-transparent
ALUMINUM_RGBA = (0.72, 0.73, 0.74, 1.0)  # silver aluminum
SCREEN_RGBA = (0.18, 0.20, 0.22, 0.55)   # dark insect mesh, semi-transparent
LATCH_RGBA = (0.85, 0.86, 0.87, 1.0)     # light metal latch


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
    """Static outer frame: a full slab cut by the three lite openings, leaving
    head, sill, two jambs and two intermediate mullions as one solid.
    Then deep track grooves are cut into head bottom and sill top faces."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)

    cut_depth = FRAME_DEPTH + 0.02
    left_cut = _slab(LEFT_X0, LEFT_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    center_cut = _slab(CENTER_X0, CENTER_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    right_cut = _slab(RIGHT_X0, RIGHT_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)

    frame = outer.cut(left_cut).cut(center_cut).cut(right_cut)

    # --- Deep track grooves in sill top and head bottom ---
    groove_positions = [GROOVE_REAR_Y, GROOVE_FRONT_Y, GROOVE_SCREEN_Y]
    for gy in groove_positions:
        # Sill groove: cut upward from sill top face (z = INNER_Z0)
        sill_groove = _slab(
            INNER_X0, INNER_X1,
            INNER_Z0 - TRACK_GROOVE_DEPTH, INNER_Z0,
            gy, TRACK_GROOVE_W,
        )
        frame = frame.cut(sill_groove)

        # Head groove: cut downward from head bottom face (z = INNER_Z1)
        head_groove = _slab(
            INNER_X0, INNER_X1,
            INNER_Z1, INNER_Z1 + TRACK_GROOVE_DEPTH,
            gy, TRACK_GROOVE_W,
        )
        frame = frame.cut(head_groove)

    return frame


def _build_sash_grille_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """One sash in its own local frame centered on local origin.
    Returns the vinyl (frame + muntins) workplane."""
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
    """Single clear pane filling the sash opening."""
    ow = opening_w + 2 * REBATE
    oh = opening_h + 2 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


def _build_screen_frame_shape() -> cq.Workplane:
    """Insect screen frame: thin aluminum rectangular frame with a hollow center.
    Built in its own local frame centered on local origin.
    The screen spans the inner opening of the window."""
    # Screen panel overall size: matches the inner frame opening
    sw = INNER_X1 - INNER_X0  # same width as inner opening
    sh = INNER_Z1 - INNER_Z0  # same height as inner opening

    out_w = sw + 2 * SCREEN_FRAME_W
    out_h = sh + 2 * SCREEN_FRAME_W

    # Outer slab
    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, SCREEN_FRAME_DEPTH)
    # Inner cutout (screen mesh area)
    inner_cut = _slab(-sw / 2.0, sw / 2.0, -sh / 2.0, sh / 2.0, 0.0, SCREEN_FRAME_DEPTH + 0.01)
    return outer.cut(inner_cut)


def _build_screen_mesh_shape() -> cq.Workplane:
    """Thin insect mesh spanning the screen frame opening."""
    sw = INNER_X1 - INNER_X0
    sh = INNER_Z1 - INNER_Z0
    return _slab(-sw / 2.0, sw / 2.0, -sh / 2.0, sh / 2.0, 0.0, SCREEN_MESH_T)


def _build_latch_shape() -> cq.Workplane:
    """Tilt-in latch lever in its own local frame: pivot boss at origin + lever arm.
    The lever extends along +Z from the pivot when engaged (q=0).
    Pivot axis is X (the latch swings in Y-Z plane)."""
    # Lever arm: extends from z=0 to z=LATCH_LENGTH, centered on X and Y
    lever = _slab(
        -LATCH_WIDTH / 2.0, LATCH_WIDTH / 2.0,
        0.0, LATCH_LENGTH,
        0.0, LATCH_DEPTH,
    )
    # Pivot boss: small cylinder at origin, axis along X
    boss = (
        cq.Workplane("YZ")
        .transformed(offset=(0.0, 0.0, 0.0))
        .circle(LATCH_PIVOT_R)
        .extrude(LATCH_PIVOT_H, both=True)
    )
    return lever.union(boss)


# ---------------------------------------------------------------------------
# Part builders
# ---------------------------------------------------------------------------

def _add_lite(
    model: ArticulatedObject,
    name: str,
    opening_w: float,
    opening_h: float,
) -> None:
    """Add a sash part (vinyl ring + colonial grille + clear glass)."""
    lite = model.part(name)
    lite.visual(
        mesh_from_cadquery(_build_sash_grille_shape(opening_w, opening_h), f"{name}_vinyl"),
        material="vinyl",
        name=f"{name}_vinyl",
    )
    lite.visual(
        mesh_from_cadquery(_build_sash_glass_shape(opening_w, opening_h), f"{name}_glass"),
        material="glass",
        name=f"{name}_glass",
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    span = (
        SIDE_LITE_W + MULLION_FACE + CENTER_LITE_W + MULLION_FACE + SIDE_LITE_W
    )
    inner_w = INNER_X1 - INNER_X0
    assert abs(span - inner_w) < 1e-6, f"lite layout {span} != inner width {inner_w}"

    model = ArticulatedObject(name="three_panel_sliding_window_variant")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("aluminum", rgba=ALUMINUM_RGBA)
    model.material("screen_mesh", rgba=SCREEN_RGBA)
    model.material("latch_metal", rgba=LATCH_RGBA)

    # --- Static outer frame (root) with deep track grooves ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material="vinyl",
        name="frame_shell",
    )

    opening_h = INNER_Z1 - INNER_Z0

    # --- Two FIXED side lites + CENTER sliding sash ---
    _add_lite(model, "left_lite", SIDE_LITE_W, opening_h)
    _add_lite(model, "right_lite", SIDE_LITE_W, opening_h)
    _add_lite(model, "center_sash", CENTER_LITE_W, opening_h)

    # --- Insect screen panel (exterior track, FIXED to frame) ---
    screen = model.part("screen_panel")
    screen.visual(
        mesh_from_cadquery(_build_screen_frame_shape(), "screen_frame"),
        material="aluminum",
        name="screen_frame",
    )
    screen.visual(
        mesh_from_cadquery(_build_screen_mesh_shape(), "screen_mesh"),
        material="screen_mesh",
        name="screen_mesh",
    )

    # --- Tilt-in latches on center sash ---
    latch_top = model.part("tilt_latch_top")
    latch_top.visual(
        mesh_from_cadquery(_build_latch_shape(), "latch_top_body"),
        material="latch_metal",
        name="latch_top_body",
    )

    latch_bot = model.part("tilt_latch_bottom")
    latch_bot.visual(
        mesh_from_cadquery(_build_latch_shape(), "latch_bot_body"),
        material="latch_metal",
        name="latch_bot_body",
    )

    # Centers (world) of each clear opening
    left_cx = (LEFT_X0 + LEFT_X1) / 2.0
    right_cx = (RIGHT_X0 + RIGHT_X1) / 2.0
    center_cx = (CENTER_X0 + CENTER_X1) / 2.0
    mid_cz = (INNER_Z0 + INNER_Z1) / 2.0

    # FIXED side lites seated in the rear glazing plane
    model.articulation(
        "frame_to_left_lite",
        ArticulationType.FIXED,
        parent="frame",
        child="left_lite",
        origin=Origin(xyz=(left_cx, FIXED_LITE_Y, mid_cz)),
    )
    model.articulation(
        "frame_to_right_lite",
        ArticulationType.FIXED,
        parent="frame",
        child="right_lite",
        origin=Origin(xyz=(right_cx, FIXED_LITE_Y, mid_cz)),
    )

    # CENTER sliding sash: PRISMATIC along +X
    slide_travel = SIDE_LITE_W * 0.92
    model.articulation(
        "frame_to_center_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="center_sash",
        origin=Origin(xyz=(center_cx, SLIDE_SASH_Y, mid_cz)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.5, lower=0.0, upper=slide_travel),
    )

    # Screen panel: FIXED to frame on exterior (-Y) side
    model.articulation(
        "frame_to_screen",
        ArticulationType.FIXED,
        parent="frame",
        child="screen_panel",
        origin=Origin(xyz=(0.0, SCREEN_Y, mid_cz)),
    )

    # Tilt-in latches: REVOLUTE on center sash
    # Top latch: mounted near top of right stile, pivots around X axis
    # At q=0, latch points upward (engaged in track groove)
    # Positive q swings latch toward +Y (interior, released)
    latch_x = center_cx + CENTER_LITE_W / 2.0 + SASH_FACE / 2.0  # right stile
    latch_top_z = mid_cz + opening_h / 2.0 - LATCH_LENGTH / 2.0  # near top
    model.articulation(
        "sash_to_latch_top",
        ArticulationType.REVOLUTE,
        parent="center_sash",
        child="tilt_latch_top",
        origin=Origin(xyz=(latch_x - center_cx, SASH_DEPTH / 2.0 + LATCH_DEPTH / 2.0, latch_top_z - mid_cz)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.4),
    )

    # Bottom latch: mounted near bottom of right stile, pivots around X axis
    # The latch geometry extends along +Z, so for the bottom latch we need to
    # flip it. We use a 180° roll in the origin to point it downward.
    latch_bot_z = mid_cz - opening_h / 2.0 + LATCH_LENGTH / 2.0  # near bottom
    model.articulation(
        "sash_to_latch_bottom",
        ArticulationType.REVOLUTE,
        parent="center_sash",
        child="tilt_latch_bottom",
        origin=Origin(
            xyz=(latch_x - center_cx, SASH_DEPTH / 2.0 + LATCH_DEPTH / 2.0, latch_bot_z - mid_cz),
            rpy=(3.14159265, 0.0, 0.0),  # flip 180° so lever points downward
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.4),
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
    screen = object_model.get_part("screen_panel")
    latch_top = object_model.get_part("tilt_latch_top")
    latch_bot = object_model.get_part("tilt_latch_bottom")
    slide = object_model.get_articulation("frame_to_center_sash")
    latch_top_joint = object_model.get_articulation("sash_to_latch_top")
    latch_bot_joint = object_model.get_articulation("sash_to_latch_bottom")

    # --- Intentional overlaps ---
    # Glass panes tuck under the vinyl/muntin lip on each sash (captured glass).
    for nm in ("left_lite", "right_lite", "center_sash"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass",
            elem_b=f"{nm}_vinyl",
            reason="Clear pane is rebated under the sash/muntin lip so it reads captured, not floating.",
        )
    # Fixed lites rebated into frame opening
    ctx.allow_overlap(
        "frame", "left_lite",
        elem_a="frame_shell", elem_b="left_lite_vinyl",
        reason="Left fixed lite is rebated into the frame opening; sash ring laps jamb/mullion edge.",
    )
    ctx.allow_overlap(
        "frame", "right_lite",
        elem_a="frame_shell", elem_b="right_lite_vinyl",
        reason="Right fixed lite is rebated into the frame opening; sash ring laps jamb/mullion edge.",
    )
    # Center sliding sash rides the track, laps the frame face
    ctx.allow_overlap(
        "frame", "center_sash",
        elem_a="frame_shell", elem_b="center_sash_vinyl",
        reason="Center sash rides the head/sill track and laps the frame face along the track.",
    )
    # Glass rebated under frame opening lip
    ctx.allow_overlap(
        "frame", "left_lite",
        elem_a="frame_shell", elem_b="left_lite_glass",
        reason="Left lite glass is rebated under the frame opening lip.",
    )
    ctx.allow_overlap(
        "frame", "right_lite",
        elem_a="frame_shell", elem_b="right_lite_glass",
        reason="Right lite glass is rebated under the frame opening lip.",
    )
    ctx.allow_overlap(
        "frame", "center_sash",
        elem_a="frame_shell", elem_b="center_sash_glass",
        reason="Center sash glass laps the head/sill track lip.",
    )
    # Screen panel: frame shell overlaps screen frame at the track groove
    ctx.allow_overlap(
        "frame", "screen_panel",
        elem_a="frame_shell", elem_b="screen_frame",
        reason="Screen panel frame sits in the exterior track groove, lapping the frame sill/head groove edge.",
    )
    # Latches overlap the center sash stile where mounted
    ctx.allow_overlap(
        "center_sash", "tilt_latch_top",
        elem_a="center_sash_vinyl", elem_b="latch_top_body",
        reason="Top tilt latch is mounted on the sash stile; pivot boss nests into the stile face.",
    )
    ctx.allow_overlap(
        "center_sash", "tilt_latch_bottom",
        elem_a="center_sash_vinyl", elem_b="latch_bot_body",
        reason="Bottom tilt latch is mounted on the sash stile; pivot boss nests into the stile face.",
    )

    # --- Screen panel tests ---
    with ctx.pose({slide: 0.0}):
        screen_aabb = ctx.part_world_aabb(screen)
        frame_aabb = ctx.part_world_aabb(frame)

        # Screen is on the exterior (-Y) side of the frame
        screen_y_center = (screen_aabb[0][1] + screen_aabb[1][1]) / 2.0
        frame_y_center = (frame_aabb[0][1] + frame_aabb[1][1]) / 2.0
        ctx.check(
            "screen panel is on exterior (-Y) side of frame",
            screen_y_center < frame_y_center - 0.02,
            details=f"screen_y={screen_y_center:.3f}, frame_y={frame_y_center:.3f}",
        )

        # Screen spans most of the frame width
        screen_w = screen_aabb[1][0] - screen_aabb[0][0]
        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        ctx.check(
            "screen panel spans most of frame width",
            screen_w > frame_w * 0.8,
            details=f"screen_w={screen_w:.3f}, frame_w={frame_w:.3f}",
        )

        # Screen spans most of the frame height
        screen_h = screen_aabb[1][2] - screen_aabb[0][2]
        frame_h = frame_aabb[1][2] - frame_aabb[0][2]
        ctx.check(
            "screen panel spans most of frame height",
            screen_h > frame_h * 0.8,
            details=f"screen_h={screen_h:.3f}, frame_h={frame_h:.3f}",
        )

    # --- Tilt latch tests ---
    with ctx.pose({slide: 0.0, latch_top_joint: 0.0, latch_bot_joint: 0.0}):
        lt_aabb = ctx.part_world_aabb(latch_top)
        lb_aabb = ctx.part_world_aabb(latch_bot)
        cs_aabb = ctx.part_world_aabb(center_sash)

        # Latches are near the center sash vertically
        lt_cz = (lt_aabb[0][2] + lt_aabb[1][2]) / 2.0
        lb_cz = (lb_aabb[0][2] + lb_aabb[1][2]) / 2.0
        cs_cz = (cs_aabb[0][2] + cs_aabb[1][2]) / 2.0
        ctx.check(
            "top latch is above sash center",
            lt_cz > cs_cz,
            details=f"top_latch_z={lt_cz:.3f}, sash_z={cs_cz:.3f}",
        )
        ctx.check(
            "bottom latch is below sash center",
            lb_cz < cs_cz,
            details=f"bot_latch_z={lb_cz:.3f}, sash_z={cs_cz:.3f}",
        )

        rest_lt_y = (lt_aabb[0][1] + lt_aabb[1][1]) / 2.0

    # Latch pivot test: driving the top latch should move it in Y (swing out)
    with ctx.pose({slide: 0.0, latch_top_joint: 1.2, latch_bot_joint: 0.0}):
        lt_open_aabb = ctx.part_world_aabb(latch_top)
        open_lt_y = (lt_open_aabb[0][1] + lt_open_aabb[1][1]) / 2.0
        ctx.check(
            "top latch swings outward when pivoted",
            abs(open_lt_y - rest_lt_y) > 0.005,
            details=f"rest_y={rest_lt_y:.4f}, open_y={open_lt_y:.4f}",
        )

    # --- Deep track groove verification ---
    # The frame should have the groove channels visible as geometry features.
    # We verify the frame has the expected Z extent (grooves don't cut through).
    with ctx.pose({slide: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "frame sill sits near z=0",
            abs(frame_aabb[0][2]) < 0.02,
            details=f"frame zmin={frame_aabb[0][2]:.4f}",
        )
        ctx.check(
            "frame head reaches full height",
            abs(frame_aabb[1][2] - TOTAL_H) < 0.02,
            details=f"frame zmax={frame_aabb[1][2]:.4f}",
        )

    # --- Closed pose (q=0): window reads SHUT ---
    with ctx.pose({slide: 0.0, latch_top_joint: 0.0, latch_bot_joint: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        l_aabb = ctx.part_world_aabb(left_lite)
        r_aabb = ctx.part_world_aabb(right_lite)
        c_aabb = ctx.part_world_aabb(center_sash)

        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        center_w = c_aabb[1][0] - c_aabb[0][0]
        ctx.check(
            "frame spans wider than the center sash",
            frame_w > center_w + 1.5,
            details=f"frame_w={frame_w:.3f}, center_w={center_w:.3f}",
        )

        lx = (l_aabb[0][0] + l_aabb[1][0]) / 2.0
        cx = (c_aabb[0][0] + c_aabb[1][0]) / 2.0
        rx = (r_aabb[0][0] + r_aabb[1][0]) / 2.0
        ctx.check(
            "lites ordered left-center-right",
            lx < cx < rx,
            details=f"left_x={lx:.3f}, center_x={cx:.3f}, right_x={rx:.3f}",
        )

        for nm, ab in (("left", l_aabb), ("right", r_aabb), ("center", c_aabb)):
            ctx.check(
                f"{nm} lite seated within frame height",
                ab[0][2] > frame_aabb[0][2] - 1e-4 and ab[1][2] < frame_aabb[1][2] + 1e-4,
                details=f"{nm} z=[{ab[0][2]:.3f},{ab[1][2]:.3f}] frame z=[{frame_aabb[0][2]:.3f},{frame_aabb[1][2]:.3f}]",
            )

        l_y = (l_aabb[0][1] + l_aabb[1][1]) / 2.0
        c_y = (c_aabb[0][1] + c_aabb[1][1]) / 2.0
        ctx.check(
            "center sash proud of side lites",
            c_y > l_y + 0.02,
            details=f"center_y={c_y:.3f}, side_y={l_y:.3f}",
        )

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
    with ctx.pose({slide: travel, latch_top_joint: 0.0, latch_bot_joint: 0.0}):
        c_open = ctx.part_world_aabb(center_sash)
        open_cx = (c_open[0][0] + c_open[1][0]) / 2.0
        ctx.check(
            "center sash slides along +X by ~travel",
            abs((open_cx - rest_cx) - travel) < 0.02,
            details=f"rest_cx={rest_cx:.3f}, open_cx={open_cx:.3f}, travel={travel:.3f}",
        )
        c_open_z = (c_open[0][2] + c_open[1][2]) / 2.0
        ctx.check(
            "slide is purely horizontal",
            abs(c_open_z - rest_cz) < 0.02,
            details=f"open_z={c_open_z:.3f}, rest_z={rest_cz:.3f}",
        )
        f_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "sash retained within frame X span at full travel",
            c_open[1][0] < f_aabb[1][0] + 1e-4 and c_open[0][0] > f_aabb[0][0] - 1e-4,
            details=f"sash x=[{c_open[0][0]:.3f},{c_open[1][0]:.3f}] frame x=[{f_aabb[0][0]:.3f},{f_aabb[1][0]:.3f}]",
        )
        ctx.expect_overlap(
            center_sash, frame,
            axes="z",
            min_overlap=0.10,
            name="sash retains vertical engagement with head/sill track",
        )

    return ctx.report()


object_model = build_object_model()
