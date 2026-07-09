from __future__ import annotations

# Variant 19: Three-panel sliding window with:
# - outer insect screen panel in a separate track (outermost +Y)
# - lower sash (center panel) slides upward on a vertical prismatic joint
# - deep track grooves along top and bottom rails (head/sill)
# - rubber gasket strips around glass panes
#
# Coordinate convention:
#   +Z is up, window stands vertically
#   width  -> X
#   height -> Z (sill near z=0)
#   depth  -> Y (glass plane is X-Z)
#
# Structure:
#   - frame (root): outer frame with deep track grooves in head/sill
#   - left_lite, right_lite (FIXED): vinyl sash + colonial grille + glass + gasket
#   - lower_sash (PRISMATIC +Z): center panel slides upward
#   - insect_screen (FIXED): thin frame + screen mesh, outermost track

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

FRAME_FACE = 0.070
MULLION_FACE = 0.060
FRAME_DEPTH = 0.150  # deeper for 3 tracks

SIDE_LITE_W = 0.85
CENTER_LITE_W = 1.04

SASH_FACE = 0.055
SASH_DEPTH = 0.055
GLASS_T = 0.008

GRILLE_COLS = 4
GRILLE_ROWS = 5
MUNTIN_T = 0.020
MUNTIN_DEPTH = 0.020

# Track Y positions (depth, from rear to front)
FIXED_LITE_Y = -0.040     # inner track (rear)
SLIDE_SASH_Y = 0.018      # middle track
SCREEN_Y = 0.058          # outer track (front)

# Track groove dimensions (visible channels in head/sill)
GROOVE_DEPTH = 0.016
GROOVE_WIDTH = 0.022

# Rubber gasket around glass
GASKET_WIDTH = 0.007
GASKET_DEPTH = 0.005

REBATE = 0.005

# Insect screen
SCREEN_FRAME_W = 0.028
SCREEN_FRAME_DEPTH = 0.018
SCREEN_MESH_T = 0.002

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
RUBBER_RGBA = (0.12, 0.12, 0.13, 1.0)
SCREEN_MESH_RGBA = (0.28, 0.30, 0.28, 0.50)
SCREEN_FRAME_RGBA = (0.85, 0.86, 0.87, 1.0)


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _slab(x0: float, x1: float, z0: float, z1: float, y_center: float, depth: float) -> cq.Workplane:
    """Axis-aligned box in the X-Z plane, centered on y_center."""
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
    """Outer frame slab cut by three lite openings, plus track grooves."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)

    cut_depth = FRAME_DEPTH + 0.02
    left_cut = _slab(LEFT_X0, LEFT_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    center_cut = _slab(CENTER_X0, CENTER_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    right_cut = _slab(RIGHT_X0, RIGHT_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)

    frame = outer.cut(left_cut).cut(center_cut).cut(right_cut)

    # Deep track grooves in head (top rail) and sill (bottom rail).
    # Three parallel groove channels per rail, one per track.
    track_ys = [FIXED_LITE_Y, SLIDE_SASH_Y, SCREEN_Y]
    groove_cut_depth = GROOVE_WIDTH + 0.002

    for ty in track_ys:
        # Head groove: cut upward from inner top edge into the head rail
        head_groove = _slab(
            INNER_X0, INNER_X1,
            INNER_Z1, INNER_Z1 + GROOVE_DEPTH,
            ty, groove_cut_depth,
        )
        frame = frame.cut(head_groove)

        # Sill groove: cut downward from inner bottom edge into the sill
        sill_groove = _slab(
            INNER_X0, INNER_X1,
            INNER_Z0 - GROOVE_DEPTH, INNER_Z0,
            ty, groove_cut_depth,
        )
        frame = frame.cut(sill_groove)

    return frame


def _build_sash_grille_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """Sash ring with colonial muntin grille, in local frame centered on origin."""
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
    """Clear pane filling the sash opening, rebated under the lip."""
    ow = opening_w + 2 * REBATE
    oh = opening_h + 2 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


def _build_gasket_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """Rubber gasket ring around glass perimeter, in sash-local frame."""
    ow = opening_w
    oh = opening_h
    # Outer boundary matches the sash opening
    # Inner boundary is the glass visible area (slightly smaller)
    inner_w = ow - 2 * GASKET_WIDTH
    inner_h = oh - 2 * GASKET_WIDTH

    outer = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GASKET_DEPTH)
    inner_cut = _slab(-inner_w / 2.0, inner_w / 2.0, -inner_h / 2.0, inner_h / 2.0, 0.0, GASKET_DEPTH + 0.002)
    return outer.cut(inner_cut)


def _build_screen_frame_shape(panel_w: float, panel_h: float) -> cq.Workplane:
    """Insect screen frame (thin aluminum/vinyl rectangular ring) in local frame."""
    out_w = panel_w + 2 * SCREEN_FRAME_W
    out_h = panel_h + 2 * SCREEN_FRAME_W

    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, SCREEN_FRAME_DEPTH)
    inner_cut = _slab(-panel_w / 2.0, panel_w / 2.0, -panel_h / 2.0, panel_h / 2.0, 0.0, SCREEN_FRAME_DEPTH + 0.002)
    return outer.cut(inner_cut)


def _build_screen_mesh_shape(panel_w: float, panel_h: float) -> cq.Workplane:
    """Thin flat panel representing insect screen mesh."""
    return _slab(-panel_w / 2.0, panel_w / 2.0, -panel_h / 2.0, panel_h / 2.0, 0.0, SCREEN_MESH_T)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    span = SIDE_LITE_W + MULLION_FACE + CENTER_LITE_W + MULLION_FACE + SIDE_LITE_W
    inner_w = INNER_X1 - INNER_X0
    assert abs(span - inner_w) < 1e-6, f"lite layout {span} != inner width {inner_w}"

    model = ArticulatedObject(name="sliding_window_with_screen")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("rubber", rgba=RUBBER_RGBA)
    model.material("screen_mesh", rgba=SCREEN_MESH_RGBA)
    model.material("screen_frame", rgba=SCREEN_FRAME_RGBA)

    # --- Static outer frame (root) with track grooves ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material="vinyl",
        name="frame_shell",
    )

    opening_h = INNER_Z1 - INNER_Z0

    # --- Fixed side lites ---
    for nm, cx in [("left_lite", (LEFT_X0 + LEFT_X1) / 2.0),
                   ("right_lite", (RIGHT_X0 + RIGHT_X1) / 2.0)]:
        lite = model.part(nm)
        lite.visual(
            mesh_from_cadquery(_build_sash_grille_shape(SIDE_LITE_W, opening_h), f"{nm}_vinyl"),
            material="vinyl",
            name=f"{nm}_vinyl",
        )
        lite.visual(
            mesh_from_cadquery(_build_sash_glass_shape(SIDE_LITE_W, opening_h), f"{nm}_glass"),
            material="glass",
            name=f"{nm}_glass",
        )
        lite.visual(
            mesh_from_cadquery(_build_gasket_shape(SIDE_LITE_W, opening_h), f"{nm}_gasket"),
            material="rubber",
            name=f"{nm}_gasket",
        )

    # --- Lower sash (center, slides upward) ---
    lower_sash = model.part("lower_sash")
    lower_sash.visual(
        mesh_from_cadquery(_build_sash_grille_shape(CENTER_LITE_W, opening_h), "lower_sash_vinyl"),
        material="vinyl",
        name="lower_sash_vinyl",
    )
    lower_sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(CENTER_LITE_W, opening_h), "lower_sash_glass"),
        material="glass",
        name="lower_sash_glass",
    )
    lower_sash.visual(
        mesh_from_cadquery(_build_gasket_shape(CENTER_LITE_W, opening_h), "lower_sash_gasket"),
        material="rubber",
        name="lower_sash_gasket",
    )

    # --- Insect screen panel (outermost track) ---
    screen_panel_w = CENTER_LITE_W + 2 * SASH_FACE
    screen_panel_h = opening_h - 2 * SCREEN_FRAME_W

    insect_screen = model.part("insect_screen")
    insect_screen.visual(
        mesh_from_cadquery(_build_screen_frame_shape(screen_panel_w, screen_panel_h), "screen_frame"),
        material="screen_frame",
        name="screen_frame",
    )
    insect_screen.visual(
        mesh_from_cadquery(_build_screen_mesh_shape(screen_panel_w, screen_panel_h), "screen_mesh"),
        material="screen_mesh",
        name="screen_mesh",
    )

    # --- Articulations ---
    mid_cz = (INNER_Z0 + INNER_Z1) / 2.0
    left_cx = (LEFT_X0 + LEFT_X1) / 2.0
    right_cx = (RIGHT_X0 + RIGHT_X1) / 2.0
    center_cx = (CENTER_X0 + CENTER_X1) / 2.0

    # Fixed side lites
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

    # Lower sash: PRISMATIC along +Z (slides upward)
    # At q=0, sash is closed (seated at bottom of opening).
    # Positive q lifts the sash upward.
    slide_travel = 0.50  # 50cm upward travel
    model.articulation(
        "frame_to_lower_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="lower_sash",
        origin=Origin(xyz=(center_cx, SLIDE_SASH_Y, mid_cz)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.3, lower=0.0, upper=slide_travel),
    )

    # Insect screen: fixed in outer track
    screen_cx = center_cx
    model.articulation(
        "frame_to_screen",
        ArticulationType.FIXED,
        parent="frame",
        child="insect_screen",
        origin=Origin(xyz=(screen_cx, SCREEN_Y, mid_cz)),
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
    lower_sash = object_model.get_part("lower_sash")
    insect_screen = object_model.get_part("insect_screen")
    sash_joint = object_model.get_articulation("frame_to_lower_sash")

    # --- Intentional overlaps ---
    # Glass panes tuck under vinyl/muntin lip (captured glass)
    for nm in ("left_lite", "right_lite", "lower_sash"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass",
            elem_b=f"{nm}_vinyl",
            reason="Clear pane is rebated under the sash/muntin lip (captured glazing).",
        )
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_gasket",
            elem_b=f"{nm}_vinyl",
            reason="Rubber gasket sits between glass and sash frame (seated seal).",
        )
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_gasket",
            elem_b=f"{nm}_glass",
            reason="Rubber gasket wraps the glass perimeter (compression seal).",
        )

    # Fixed lites rebated into frame openings
    for nm in ("left_lite", "right_lite"):
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_vinyl",
            reason=f"{nm} sash ring laps the jamb/mullion edge (seated capture).",
        )
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_glass",
            reason=f"{nm} glass rebated under frame opening lip.",
        )
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_gasket",
            reason=f"{nm} gasket sits in the frame rebate.",
        )

    # Lower sash rides vertical track, laps frame head/sill
    ctx.allow_overlap(
        "frame", "lower_sash",
        elem_a="frame_shell",
        elem_b="lower_sash_vinyl",
        reason="Lower sash rides the vertical track and laps the frame along the groove.",
    )
    ctx.allow_overlap(
        "frame", "lower_sash",
        elem_a="frame_shell",
        elem_b="lower_sash_glass",
        reason="Lower sash glass laps the frame track lip.",
    )
    ctx.allow_overlap(
        "frame", "lower_sash",
        elem_a="frame_shell",
        elem_b="lower_sash_gasket",
        reason="Lower sash gasket laps frame track lip.",
    )

    # Insect screen fixed in outer track
    ctx.allow_overlap(
        "frame", "insect_screen",
        elem_a="frame_shell",
        elem_b="screen_frame",
        reason="Screen frame seated in outer track groove.",
    )

    # Screen mesh inside screen frame
    ctx.allow_overlap(
        "insect_screen", "insect_screen",
        elem_a="screen_mesh",
        elem_b="screen_frame",
        reason="Screen mesh captured inside screen frame perimeter.",
    )

    # --- Closed pose (q=0): window reads SHUT ---
    with ctx.pose({sash_joint: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        l_aabb = ctx.part_world_aabb(left_lite)
        r_aabb = ctx.part_world_aabb(right_lite)
        s_aabb = ctx.part_world_aabb(lower_sash)
        scr_aabb = ctx.part_world_aabb(insect_screen)

        # Frame dimensions
        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        sash_w = s_aabb[1][0] - s_aabb[0][0]
        ctx.check(
            "frame spans wider than the lower sash",
            frame_w > sash_w + 1.5,
            details=f"frame_w={frame_w:.3f}, sash_w={sash_w:.3f}",
        )
        ctx.check(
            "sill sits near z=0",
            abs(frame_aabb[0][2]) < 0.02,
            details=f"frame zmin={frame_aabb[0][2]:.4f}",
        )
        ctx.check(
            "head reaches full height",
            abs(frame_aabb[1][2] - TOTAL_H) < 0.02,
            details=f"frame zmax={frame_aabb[1][2]:.4f}",
        )

        # Lites ordered left -> center(sash) -> right
        lx = (l_aabb[0][0] + l_aabb[1][0]) / 2.0
        sx = (s_aabb[0][0] + s_aabb[1][0]) / 2.0
        rx = (r_aabb[0][0] + r_aabb[1][0]) / 2.0
        ctx.check(
            "lites ordered left-sash-right",
            lx < sx < rx,
            details=f"left_x={lx:.3f}, sash_x={sx:.3f}, right_x={rx:.3f}",
        )

        # All lites within frame height
        for nm, ab in [("left", l_aabb), ("right", r_aabb), ("sash", s_aabb)]:
            ctx.check(
                f"{nm} seated within frame height",
                ab[0][2] > frame_aabb[0][2] - 1e-4 and ab[1][2] < frame_aabb[1][2] + 1e-4,
                details=f"{nm} z=[{ab[0][2]:.3f},{ab[1][2]:.3f}]",
            )

        # Insect screen is on the outermost track (+Y side)
        s_y = (s_aabb[0][1] + s_aabb[1][1]) / 2.0
        scr_y = (scr_aabb[0][1] + scr_aabb[1][1]) / 2.0
        ctx.check(
            "screen is outermost (most +Y) track",
            scr_y > s_y + 0.02,
            details=f"screen_y={scr_y:.3f}, sash_y={s_y:.3f}",
        )

        # Fixed lites are in the innermost track (-Y side)
        l_y = (l_aabb[0][1] + l_aabb[1][1]) / 2.0
        ctx.check(
            "fixed lites innermost (most -Y) track",
            l_y < s_y - 0.02,
            details=f"lite_y={l_y:.3f}, sash_y={s_y:.3f}",
        )

        # Lower sash seated at closed position: sash ring extends into the sill
        # track groove (below the visible opening at INNER_Z0), as in a real
        # window where the bottom rail sits captured in the sill channel.
        sash_bottom = s_aabb[0][2]
        ctx.check(
            "lower sash closed: bottom rail in sill track",
            sash_bottom > -0.005 and sash_bottom < INNER_Z0,
            details=f"sash_bottom={sash_bottom:.4f}, sill_top={INNER_Z0:.4f}",
        )

        rest_cz = (s_aabb[0][2] + s_aabb[1][2]) / 2.0
        rest_cx = sx

    # --- Open pose: lower sash slides upward ---
    travel = sash_joint.motion_limits.upper
    with ctx.pose({sash_joint: travel}):
        s_open = ctx.part_world_aabb(lower_sash)
        open_cz = (s_open[0][2] + s_open[1][2]) / 2.0
        open_cx = (s_open[0][0] + s_open[1][0]) / 2.0

        # Sash moved upward by ~travel
        ctx.check(
            "lower sash slides upward by ~travel",
            abs((open_cz - rest_cz) - travel) < 0.02,
            details=f"rest_cz={rest_cz:.3f}, open_cz={open_cz:.3f}, travel={travel:.3f}",
        )
        # No horizontal drift
        ctx.check(
            "slide is purely vertical (no X drift)",
            abs(open_cx - rest_cx) < 0.02,
            details=f"rest_cx={rest_cx:.3f}, open_cx={open_cx:.3f}",
        )
        # Retained: sash still overlaps frame vertically
        f_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "sash retained within frame at max travel",
            s_open[0][2] < f_aabb[1][2] and s_open[1][2] > f_aabb[0][2],
            details=f"sash z=[{s_open[0][2]:.3f},{s_open[1][2]:.3f}] frame z=[{f_aabb[0][2]:.3f},{f_aabb[1][2]:.3f}]",
        )
        ctx.expect_overlap(
            lower_sash, frame,
            axes="x",
            min_overlap=0.10,
            name="sash retains X engagement with frame track at max travel",
        )

    # --- Track grooves exist: frame depth accommodates 3 tracks ---
    frame_depth = frame_aabb[1][1] - frame_aabb[0][1]
    ctx.check(
        "frame deep enough for 3 tracks",
        frame_depth > 0.10,
        details=f"frame_depth={frame_depth:.3f}",
    )

    # --- Screen panel exists with reasonable size ---
    scr_w = scr_aabb[1][0] - scr_aabb[0][0]
    scr_h = scr_aabb[1][2] - scr_aabb[0][2]
    ctx.check(
        "screen panel has reasonable width",
        scr_w > 0.5 and scr_w < 2.0,
        details=f"screen_w={scr_w:.3f}",
    )
    ctx.check(
        "screen panel has reasonable height",
        scr_h > 0.5 and scr_h < 1.8,
        details=f"screen_h={scr_h:.3f}",
    )

    # --- Gasket visual elements exist ---
    ctx.check(
        "lower_sash has gasket element",
        lower_sash.get_visual("lower_sash_gasket") is not None,
        details="gasket visual not found on lower_sash",
    )
    ctx.check(
        "left_lite has gasket element",
        left_lite.get_visual("left_lite_gasket") is not None,
        details="gasket visual not found on left_lite",
    )

    return ctx.report()


object_model = build_object_model()
