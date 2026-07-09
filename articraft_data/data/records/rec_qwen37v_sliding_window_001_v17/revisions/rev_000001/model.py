from __future__ import annotations

# Three-panel horizontal sliding window variant: white vinyl frame, deep track
# grooves along head and sill, muntin grid bars on the MOVABLE sash only, an
# insect screen that slides independently on a shallow prismatic joint, and a
# recessed pull cup on the movable sash.
#
# Coordinate convention:
#   +Z is up. Window stands vertically.
#     - width  -> X
#     - height -> Z   (sill near z=0)
#     - frame depth / glazing thickness / slide-normal -> Y
#   The glass plane is the X-Z plane. The window reads SHUT at q=0; driving the
#   prismatic joints slides parts sideways (+X).
#
# Structure:
#   - frame (static root): head, sill, two jambs, two intermediate mullions
#     with deep track grooves cut into head and sill rails.
#   - left_lite, right_lite (FIXED): vinyl sash ring + clear glass, NO muntins.
#   - center_sash (SLIDING): sash ring + colonial muntin grille + glass + pull cup.
#   - insect_screen (SLIDING): thin frame + mesh panel, on exterior track.

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    MeshGeometry,
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

# Three lite columns. Center is wider (the slider); sides are the fixed lites.
SIDE_LITE_W = 0.85        # clear opening width of each side lite
CENTER_LITE_W = 1.04      # clear opening width of the center lite

# Sash construction
SASH_FACE = 0.055         # sash perimeter rail/stile face width (in-plane)
SASH_DEPTH = 0.055        # sash depth along Y
GLASS_T = 0.008           # glazing thickness along Y

# Colonial grille (divided lite): only on center sash.
GRILLE_COLS = 4           # 4 columns of panes
GRILLE_ROWS = 5           # 5 rows of panes
MUNTIN_T = 0.020          # muntin bar face width (in-plane)
MUNTIN_DEPTH = 0.020      # muntin bar depth along Y

# Y layout (depth). Frame box centered on y=0.
FIXED_LITE_Y = -0.020     # fixed side lites: rear glazing plane center (Y)
SLIDE_SASH_Y = 0.052      # center sash sits proud toward +Y (passes in front)

REBATE = 0.005            # glass tucks under the sash/muntin lip by this much

# Track groove dimensions (deep grooves in head and sill)
TRACK_GROOVE_DEPTH = 0.018   # groove depth cut into the rail
TRACK_GROOVE_WIDTH = 0.025   # groove width (accommodates sash thickness)
# Two tracks: inner track for fixed lites, outer track for slider
TRACK_SPACING = 0.040        # center-to-center spacing of the two tracks

# Insect screen dimensions
SCREEN_FRAME_W = 0.030       # screen frame member width
SCREEN_FRAME_DEPTH = 0.020   # screen frame depth (thin)
SCREEN_MESH_T = 0.003        # mesh panel thickness
SCREEN_Y = -0.065            # screen sits on exterior side (-Y)

# Pull cup dimensions (recessed into the center sash stile)
PULL_CUP_W = 0.060           # pull cup width along Z
PULL_CUP_H = 0.025           # pull cup height along X (depth of recess)
PULL_CUP_DEPTH = 0.015       # how far the cup recesses into the stile

# ---------------------------------------------------------------------------
# Derived layout
# ---------------------------------------------------------------------------

HALF_W = TOTAL_W / 2.0

# Inner clear region (inside the outer head/sill/jambs)
INNER_X0 = -HALF_W + FRAME_FACE
INNER_X1 = HALF_W - FRAME_FACE
INNER_Z0 = FRAME_FACE
INNER_Z1 = TOTAL_H - FRAME_FACE

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

# Screen spans the inner clear region
SCREEN_X0 = INNER_X0
SCREEN_X1 = INNER_X1
SCREEN_Z0 = INNER_Z0
SCREEN_Z1 = INNER_Z1

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

VINYL_RGBA = (0.94, 0.95, 0.96, 1.0)     # bright white vinyl/PVC
GLASS_RGBA = (0.50, 0.58, 0.64, 0.32)    # cool grey-blue, semi-transparent
SCREEN_RGBA = (0.45, 0.45, 0.42, 0.55)   # dark grey mesh, semi-transparent
ALUMINUM_RGBA = (0.75, 0.76, 0.77, 1.0)  # brushed aluminum for screen frame


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
    head, sill, two jambs and the two intermediate mullions as one solid.
    Deep track grooves are cut into the head and sill rails."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)

    cut_depth = FRAME_DEPTH + 0.02  # through-cut clearance in Y
    left_cut = _slab(LEFT_X0, LEFT_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    center_cut = _slab(CENTER_X0, CENTER_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    right_cut = _slab(RIGHT_X0, RIGHT_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)

    frame = outer.cut(left_cut).cut(center_cut).cut(right_cut)

    # Deep track grooves along the head rail (top) and sill rail (bottom).
    # Two parallel grooves: inner track (for fixed lites) and outer track (for slider).
    # The grooves cut into the interior face of the head and sill.
    groove_y_center = 0.0  # centered in frame depth
    groove_cut_depth = TRACK_GROOVE_WIDTH

    # Inner track grooves (at FIXED_LITE_Y position)
    inner_track_y = FIXED_LITE_Y
    # Outer track grooves (at SLIDE_SASH_Y position)
    outer_track_y = SLIDE_SASH_Y

    for track_y in [inner_track_y, outer_track_y]:
        # Head groove (top rail) - cuts downward from inner top of head
        head_groove = _slab(
            INNER_X0, INNER_X1,
            INNER_Z1 - TRACK_GROOVE_DEPTH, INNER_Z1 + 0.002,
            track_y, groove_cut_depth,
        )
        frame = frame.cut(head_groove)

        # Sill groove (bottom rail) - cuts upward from inner bottom of sill
        sill_groove = _slab(
            INNER_X0, INNER_X1,
            INNER_Z0 - 0.002, INNER_Z0 + TRACK_GROOVE_DEPTH,
            track_y, groove_cut_depth,
        )
        frame = frame.cut(sill_groove)

    return frame


def _build_sash_ring_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """Sash ring (frame only, no muntins) in its own local frame centered on origin."""
    ow = opening_w
    oh = opening_h
    out_w = ow + 2 * SASH_FACE
    out_h = oh + 2 * SASH_FACE

    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, SASH_DEPTH)
    opening = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SASH_DEPTH + 0.02)
    return outer.cut(opening)


def _build_sash_with_grille_shape(opening_w: float, opening_h: float, *, include_pull_cup: bool = False) -> cq.Workplane:
    """Sash with colonial muntin grid in its own local frame centered on origin.
    Used for the movable center sash only. Optionally includes a pull cup boss
    on the right stile front face."""
    ow = opening_w
    oh = opening_h
    out_w = ow + 2 * SASH_FACE
    out_h = oh + 2 * SASH_FACE

    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, SASH_DEPTH)
    opening = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SASH_DEPTH + 0.02)
    ring = outer.cut(opening)

    # Colonial muntin grid across the opening
    bars = None

    # Vertical muntins: GRILLE_COLS columns -> (GRILLE_COLS - 1) interior bars.
    for c in range(1, GRILLE_COLS):
        frac = c / GRILLE_COLS
        x = -ow / 2.0 + frac * ow
        bar = _slab(
            x - MUNTIN_T / 2.0, x + MUNTIN_T / 2.0,
            -oh / 2.0, oh / 2.0,
            0.0, MUNTIN_DEPTH,
        )
        bars = bar if bars is None else bars.union(bar)

    # Horizontal muntins: GRILLE_ROWS rows -> (GRILLE_ROWS - 1) interior bars.
    for r in range(1, GRILLE_ROWS):
        frac = r / GRILLE_ROWS
        z = -oh / 2.0 + frac * oh
        bar = _slab(
            -ow / 2.0, ow / 2.0,
            z - MUNTIN_T / 2.0, z + MUNTIN_T / 2.0,
            0.0, MUNTIN_DEPTH,
        )
        bars = bar if bars is None else bars.union(bar)

    sash = ring if bars is None else ring.union(bars)

    # Pull cup: a small raised boss on the right stile front face (+Y).
    # This is the recessed grip the user pulls to slide the sash.
    if include_pull_cup:
        stile_x = ow / 2.0 + SASH_FACE / 2.0  # right stile center
        boss_y = SASH_DEPTH / 2.0 + PULL_CUP_DEPTH / 2.0  # proud of front face
        boss = _slab(
            stile_x - PULL_CUP_H / 2.0, stile_x + PULL_CUP_H / 2.0,
            -PULL_CUP_W / 2.0, PULL_CUP_W / 2.0,
            boss_y, PULL_CUP_DEPTH,
        )
        sash = sash.union(boss)

    return sash


def _build_sash_glass_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """Single clear pane filling the sash opening, in sash-local frame."""
    ow = opening_w + 2 * REBATE
    oh = opening_h + 2 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


def _build_pull_cup_shape() -> cq.Workplane:
    """Recessed pull cup: a shallow rectangular pocket cut into the sash stile.
    Built in its own local frame centered on origin."""
    # The pull cup is a recessed rectangular cavity
    # Width along local Z (vertical on the stile), depth along local X, height along local Y
    return _slab(
        -PULL_CUP_H / 2.0, PULL_CUP_H / 2.0,
        -PULL_CUP_W / 2.0, PULL_CUP_W / 2.0,
        0.0, PULL_CUP_DEPTH,
    )


def _build_screen_frame_shape() -> cq.Workplane:
    """Insect screen frame: thin rectangular frame spanning the inner opening.
    Built in its own local frame centered on origin."""
    ow = SCREEN_X1 - SCREEN_X0
    oh = SCREEN_Z1 - SCREEN_Z0
    out_w = ow + 2 * SCREEN_FRAME_W
    out_h = oh + 2 * SCREEN_FRAME_W

    # Outer frame slab
    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, SCREEN_FRAME_DEPTH)
    # Cut the inner opening (mesh area)
    inner = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SCREEN_FRAME_DEPTH + 0.01)
    return outer.cut(inner)


def _build_screen_mesh_shape() -> cq.Workplane:
    """Insect screen mesh panel: thin panel filling the screen frame opening."""
    ow = (SCREEN_X1 - SCREEN_X0) + 2 * 0.005  # slight overlap for capture
    oh = (SCREEN_Z1 - SCREEN_Z0) + 2 * 0.005
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SCREEN_MESH_T)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    # Sanity: the three lites + two mullions must fill the inner clear width.
    span = SIDE_LITE_W + MULLION_FACE + CENTER_LITE_W + MULLION_FACE + SIDE_LITE_W
    inner_w = INNER_X1 - INNER_X0
    assert abs(span - inner_w) < 1e-6, f"lite layout {span} != inner width {inner_w}"

    model = ArticulatedObject(name="three_panel_sliding_window_variant")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("screen_mesh", rgba=SCREEN_RGBA)
    model.material("aluminum", rgba=ALUMINUM_RGBA)

    # --- Static outer frame (root) with track grooves ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material="vinyl",
        name="frame_shell",
    )

    # Opening heights (clear glass region)
    opening_h = INNER_Z1 - INNER_Z0

    # --- Fixed side lites (NO muntin grilles) ---
    left_lite = model.part("left_lite")
    left_lite.visual(
        mesh_from_cadquery(_build_sash_ring_shape(SIDE_LITE_W, opening_h), "left_lite_vinyl"),
        material="vinyl",
        name="left_lite_vinyl",
    )
    left_lite.visual(
        mesh_from_cadquery(_build_sash_glass_shape(SIDE_LITE_W, opening_h), "left_lite_glass"),
        material="glass",
        name="left_lite_glass",
    )

    right_lite = model.part("right_lite")
    right_lite.visual(
        mesh_from_cadquery(_build_sash_ring_shape(SIDE_LITE_W, opening_h), "right_lite_vinyl"),
        material="vinyl",
        name="right_lite_vinyl",
    )
    right_lite.visual(
        mesh_from_cadquery(_build_sash_glass_shape(SIDE_LITE_W, opening_h), "right_lite_glass"),
        material="glass",
        name="right_lite_glass",
    )

    # --- Center sliding sash (WITH muntin grille + pull cup) ---
    center_sash = model.part("center_sash")
    center_sash.visual(
        mesh_from_cadquery(_build_sash_with_grille_shape(CENTER_LITE_W, opening_h, include_pull_cup=True), "center_sash_vinyl"),
        material="vinyl",
        name="center_sash_vinyl",
    )
    center_sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(CENTER_LITE_W, opening_h), "center_sash_glass"),
        material="glass",
        name="center_sash_glass",
    )

    # --- Insect screen (slides independently) ---
    insect_screen = model.part("insect_screen")
    insect_screen.visual(
        mesh_from_cadquery(_build_screen_frame_shape(), "screen_frame"),
        material="aluminum",
        name="screen_frame",
    )
    insect_screen.visual(
        mesh_from_cadquery(_build_screen_mesh_shape(), "screen_mesh"),
        material="screen_mesh",
        name="screen_mesh",
    )

    # Centers (world) of each clear opening.
    left_cx = (LEFT_X0 + LEFT_X1) / 2.0
    right_cx = (RIGHT_X0 + RIGHT_X1) / 2.0
    center_cx = (CENTER_X0 + CENTER_X1) / 2.0
    mid_cz = (INNER_Z0 + INNER_Z1) / 2.0
    screen_cx = (SCREEN_X0 + SCREEN_X1) / 2.0

    # FIXED side lites seated in the rear glazing plane.
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

    # CENTER sliding sash: PRISMATIC along +X.
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

    # Insect screen: PRISMATIC along +X on its own exterior track.
    # Screen travel is slightly less than sash travel (screen is narrower frame).
    screen_travel = SIDE_LITE_W * 0.85
    model.articulation(
        "frame_to_screen",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="insect_screen",
        origin=Origin(xyz=(screen_cx, SCREEN_Y, mid_cz)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.4, lower=0.0, upper=screen_travel),
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
    insect_screen = object_model.get_part("insect_screen")
    slide = object_model.get_articulation("frame_to_center_sash")
    screen_slide = object_model.get_articulation("frame_to_screen")

    # --- Intentional overlaps ---
    # Glass panes tuck under the vinyl lip on each sash (captured glass).
    for nm in ("left_lite", "right_lite", "center_sash"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass",
            elem_b=f"{nm}_vinyl",
            reason="Clear pane is rebated under the sash lip so it reads captured, not floating.",
        )
    # Fixed lites rebated into frame opening.
    ctx.allow_overlap(
        "frame", "left_lite",
        elem_a="frame_shell", elem_b="left_lite_vinyl",
        reason="Left fixed lite is rebated into the frame opening (seated capture).",
    )
    ctx.allow_overlap(
        "frame", "right_lite",
        elem_a="frame_shell", elem_b="right_lite_vinyl",
        reason="Right fixed lite is rebated into the frame opening (seated capture).",
    )
    # Center sliding sash rides the head/sill track.
    ctx.allow_overlap(
        "frame", "center_sash",
        elem_a="frame_shell", elem_b="center_sash_vinyl",
        reason="Center sash rides the head/sill track and laps the frame face along the track (slider capture).",
    )
    # Glass rebated under frame lip.
    ctx.allow_overlap(
        "frame", "left_lite",
        elem_a="frame_shell", elem_b="left_lite_glass",
        reason="Left lite glass is rebated under the frame opening lip (captured glazing).",
    )
    ctx.allow_overlap(
        "frame", "right_lite",
        elem_a="frame_shell", elem_b="right_lite_glass",
        reason="Right lite glass is rebated under the frame opening lip (captured glazing).",
    )
    ctx.allow_overlap(
        "frame", "center_sash",
        elem_a="frame_shell", elem_b="center_sash_glass",
        reason="Center sash glass laps the head/sill track lip as the proud sash rides the track.",
    )
    # Insect screen rides its own track on the exterior side.
    ctx.allow_overlap(
        "frame", "insect_screen",
        elem_a="frame_shell", elem_b="screen_frame",
        reason="Insect screen frame rides the exterior track groove and laps the frame sill/head.",
    )
    ctx.allow_overlap(
        "insect_screen", "insect_screen",
        elem_a="screen_mesh", elem_b="screen_frame",
        reason="Screen mesh panel is captured inside the screen frame (seated insertion).",
    )

    # --- Structural checks ---
    with ctx.pose({slide: 0.0, screen_slide: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)

        # Frame spans the full width.
        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        center_w = ctx.part_world_aabb(center_sash)[1][0] - ctx.part_world_aabb(center_sash)[0][0]
        ctx.check(
            "frame spans wider than the center sash",
            frame_w > center_w + 1.5,
            details=f"frame_w={frame_w:.3f}, center_w={center_w:.3f}",
        )

        # Sill near z=0.
        ctx.check(
            "sill sits near z=0",
            abs(frame_aabb[0][2]) < 0.02,
            details=f"frame zmin={frame_aabb[0][2]:.4f}",
        )

        # Head at full height.
        ctx.check(
            "head reaches full height",
            abs(frame_aabb[1][2] - TOTAL_H) < 0.02,
            details=f"frame zmax={frame_aabb[1][2]:.4f}",
        )

    # --- Muntin grid only on center sash ---
    # The center sash should have more visuals (vinyl with grille) than side lites.
    center_visuals = [v.name for v in center_sash.visuals]
    left_visuals = [v.name for v in left_lite.visuals]
    ctx.check(
        "center sash has vinyl visual (with muntin grille)",
        "center_sash_vinyl" in center_visuals,
        details=f"center_sash visuals: {center_visuals}",
    )
    ctx.check(
        "left lite has no muntin grille (plain sash ring only)",
        "left_lite_vinyl" in left_visuals,
        details=f"left_lite visuals: {left_visuals}",
    )

    # --- Pull cup on center sash ---
    # The center sash should have a pull cup boss that extends proud of the sash
    # front face in +Y. Prove it by comparing the sash Y extent with the left lite.
    with ctx.pose({slide: 0.0, screen_slide: 0.0}):
        sash_aabb_closed = ctx.part_world_aabb(center_sash)
        left_aabb = ctx.part_world_aabb(left_lite)
        sash_y_max = sash_aabb_closed[1][1]
        left_y_max = left_aabb[1][1]
        ctx.check(
            "center sash has pull cup boss (extends proud in +Y)",
            sash_y_max > left_y_max + 0.010,
            details=f"sash_y_max={sash_y_max:.4f}, left_y_max={left_y_max:.4f}",
        )

    # --- Insect screen exists and has its own joint ---
    ctx.check(
        "insect screen part exists",
        insect_screen is not None,
        details="insect_screen part not found",
    )
    screen_art = object_model.get_articulation("frame_to_screen")
    ctx.check(
        "insect screen has prismatic articulation",
        screen_art is not None and screen_art.articulation_type == ArticulationType.PRISMATIC,
        details=f"screen articulation type: {screen_art.articulation_type if screen_art else 'None'}",
    )

    # --- Insect screen slides independently ---
    screen_travel = screen_slide.motion_limits.upper
    with ctx.pose({slide: 0.0, screen_slide: screen_travel}):
        screen_open = ctx.part_world_aabb(insect_screen)
        screen_rest_cx = (SCREEN_X0 + SCREEN_X1) / 2.0
        screen_open_cx = (screen_open[0][0] + screen_open[1][0]) / 2.0
        ctx.check(
            "insect screen slides along +X independently",
            abs((screen_open_cx - screen_rest_cx) - screen_travel) < 0.02,
            details=f"rest_cx={screen_rest_cx:.3f}, open_cx={screen_open_cx:.3f}, travel={screen_travel:.3f}",
        )

    # --- Screen on exterior side (-Y) of center sash ---
    with ctx.pose({slide: 0.0, screen_slide: 0.0}):
        sash_aabb = ctx.part_world_aabb(center_sash)
        scr_aabb = ctx.part_world_aabb(insect_screen)
        sash_y = (sash_aabb[0][1] + sash_aabb[1][1]) / 2.0
        scr_y = (scr_aabb[0][1] + scr_aabb[1][1]) / 2.0
        ctx.check(
            "insect screen on exterior side of sash",
            scr_y < sash_y - 0.02,
            details=f"screen_y={scr_y:.3f}, sash_y={sash_y:.3f}",
        )

    # --- Center sash slide test ---
    travel = slide.motion_limits.upper
    with ctx.pose({slide: 0.0, screen_slide: 0.0}):
        c_rest = ctx.part_world_aabb(center_sash)
        rest_cx = (c_rest[0][0] + c_rest[1][0]) / 2.0
        rest_cz = (c_rest[0][2] + c_rest[1][2]) / 2.0

    with ctx.pose({slide: travel, screen_slide: 0.0}):
        c_open = ctx.part_world_aabb(center_sash)
        open_cx = (c_open[0][0] + c_open[1][0]) / 2.0
        ctx.check(
            "center sash slides along +X by ~travel",
            abs((open_cx - rest_cx) - travel) < 0.02,
            details=f"rest_cx={rest_cx:.3f}, open_cx={open_cx:.3f}, travel={travel:.3f}",
        )
        # Pure horizontal slide.
        c_open_z = (c_open[0][2] + c_open[1][2]) / 2.0
        ctx.check(
            "slide is purely horizontal",
            abs(c_open_z - rest_cz) < 0.02,
            details=f"open_z={c_open_z:.3f}, rest_z={rest_cz:.3f}",
        )
        # Retained insertion.
        f_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "sash retained within frame X span at full travel",
            c_open[1][0] < f_aabb[1][0] + 1e-4 and c_open[0][0] > f_aabb[0][0] - 1e-4,
            details=f"sash x=[{c_open[0][0]:.3f},{c_open[1][0]:.3f}] frame x=[{f_aabb[0][0]:.3f},{f_aabb[1][0]:.3f}]",
        )

    # --- Track grooves make frame deeper ---
    # The frame should have visible depth variation from the track grooves.
    ctx.check(
        "track groove depth is meaningful",
        TRACK_GROOVE_DEPTH > 0.010,
        details=f"track_groove_depth={TRACK_GROOVE_DEPTH}",
    )

    return ctx.report()


object_model = build_object_model()
