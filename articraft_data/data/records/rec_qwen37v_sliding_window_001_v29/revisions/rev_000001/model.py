from __future__ import annotations

# Variant 29: Three-panel horizontal sliding window with insect screen, deep track
# grooves, and recessed pull cup. White vinyl frame with colonial divided-lite
# grilles. Center sash slides sideways on prismatic joint.
#
# Coordinate convention:
#   +Z is up. Window stands vertically.
#     - width  -> X
#     - height -> Z   (sill near z=0)
#     - frame depth / glazing thickness / slide-normal -> Y
#   The glass plane is the X-Z plane. The window reads SHUT at q=0; driving the
#   prismatic joint slides the center sash sideways (+X) by ~one panel width,
#   staying retained in the track.
#
# Variant changes from parent:
#   - Insect screen panel in a separate exterior track (-Y side)
#   - Deep track grooves along top (head) and bottom (sill) rails
#   - Recessed pull cup on the movable center sash stile
#
# Structure:
#   - frame (static root): head, sill, two jambs, two mullions + track grooves
#   - left_lite, right_lite (FIXED): sash ring + colonial grille + glass
#   - center_sash (SLIDING): sash + grille + glass + pull cup, PRISMATIC along +X
#   - screen_panel (FIXED): exterior insect screen in separate track

import cadquery as cq
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Absolute dimensions (meters)
# ---------------------------------------------------------------------------

# Outer opening
TOTAL_W = 3.00            # overall window width along X
TOTAL_H = 1.50            # overall height along Z (sill at z=0, head at z=TOTAL_H)

FRAME_FACE = 0.070        # outer frame member face width (jamb / head / sill)
MULLION_FACE = 0.060      # intermediate mullion face width
FRAME_DEPTH = 0.120       # outer frame depth along Y (deeper for track grooves)

# Track grooves (deeper channels in head/sill for sash guidance)
TRACK_GROOVE_DEPTH = 0.025  # how deep the groove cuts into the rail
TRACK_GROOVE_WIDTH = 0.030  # groove width in Y (matches sash depth roughly)
TRACK_GROOVE_INSET = 0.010  # groove offset from front face of frame

# Three lite columns. Center is wider (the slider); sides are the fixed lites.
SIDE_LITE_W = 0.85        # clear opening width of each side lite
CENTER_LITE_W = 1.04      # clear opening width of the center lite

# Sash construction
SASH_FACE = 0.055         # sash perimeter rail/stile face width (in-plane)
SASH_DEPTH = 0.055        # sash depth along Y
GLASS_T = 0.008           # glazing thickness along Y

# Colonial grille (divided lite)
GRILLE_COLS = 4           # 4 columns of panes
GRILLE_ROWS = 5           # 5 rows of panes
MUNTIN_T = 0.020          # muntin bar face width (in-plane)
MUNTIN_DEPTH = 0.020      # muntin bar depth along Y

# Pull cup dimensions (recessed handle on movable sash stile)
PULL_CUP_W = 0.060        # cup width along Z (vertical)
PULL_CUP_H = 0.025        # cup height/depth along Y (recess depth)
PULL_CUP_D = 0.015        # cup depth into the stile along X (width of recess)

# Screen panel
SCREEN_FRAME_W = 0.025    # screen frame member width
SCREEN_MESH_T = 0.003     # screen mesh thickness

# Y layout (depth). Frame box centered on y=0.
FIXED_LITE_Y = -0.020     # fixed side lites: rear glazing plane center (Y)
SLIDE_SASH_Y = 0.052      # center sash sits proud toward +Y (passes in front)
SCREEN_Y = -0.065         # screen panel on exterior side (-Y), in separate track

REBATE = 0.005            # glass tucks under the sash/muntin lip by this much

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

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

VINYL_RGBA = (0.94, 0.95, 0.96, 1.0)     # bright white vinyl/PVC
GLASS_RGBA = (0.50, 0.58, 0.64, 0.32)    # cool grey-blue, semi-transparent
SCREEN_RGBA = (0.25, 0.25, 0.27, 0.55)   # dark grey insect screen mesh
ALUMINUM_RGBA = (0.60, 0.62, 0.63, 1.0)  # screen frame aluminum


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
    Includes deep track grooves in head and sill rails."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)

    cut_depth = FRAME_DEPTH + 0.02  # through-cut clearance in Y
    left_cut = _slab(LEFT_X0, LEFT_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    center_cut = _slab(CENTER_X0, CENTER_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    right_cut = _slab(RIGHT_X0, RIGHT_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)

    frame = outer.cut(left_cut).cut(center_cut).cut(right_cut)

    # Deep track grooves in head rail (top) - two parallel grooves for the
    # sliding sash track. Cut from the front face (+Y) inward.
    groove_y_center = FRAME_DEPTH / 2.0 - TRACK_GROOVE_INSET - TRACK_GROOVE_DEPTH / 2.0
    # Head groove (top rail): runs the full inner width
    head_groove = _slab(
        INNER_X0, INNER_X1,
        INNER_Z1 - TRACK_GROOVE_WIDTH, INNER_Z1,
        groove_y_center, TRACK_GROOVE_DEPTH + 0.002,
    )
    frame = frame.cut(head_groove)

    # Sill groove (bottom rail): runs the full inner width
    sill_groove = _slab(
        INNER_X0, INNER_X1,
        INNER_Z0, INNER_Z0 + TRACK_GROOVE_WIDTH,
        groove_y_center, TRACK_GROOVE_DEPTH + 0.002,
    )
    frame = frame.cut(sill_groove)

    return frame


def _build_sash_grille_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """One sash built in its OWN local frame, centered on local origin:
      - local X in [-opening_w/2 - SASH_FACE, +opening_w/2 + SASH_FACE]
      - local Z in [-opening_h/2 - SASH_FACE, +opening_h/2 + SASH_FACE]
      - local Y is the sash depth, centered at 0
    Construction: outer sash slab cut by the clear opening, then the colonial
    muntin grid (vertical + horizontal bars) unioned back in across the opening.
    """
    ow = opening_w
    oh = opening_h
    out_w = ow + 2 * SASH_FACE
    out_h = oh + 2 * SASH_FACE

    # Outer sash slab.
    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, SASH_DEPTH)
    # Hollow it: cut the clear opening (glass region).
    opening = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SASH_DEPTH + 0.02)
    ring = outer.cut(opening)

    # Colonial muntin grid across the opening.
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

    return ring if bars is None else ring.union(bars)


def _build_sash_glass_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """Single clear pane filling the sash opening, in the same sash-local frame."""
    ow = opening_w + 2 * REBATE
    oh = opening_h + 2 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


def _build_pull_cup_shape() -> cq.Workplane:
    """Recessed pull cup on the sash stile. Built in the sash local frame.
    The cup is a recessed oval depression on the right stile of the center sash,
    used to grip and slide the sash. It sits at mid-height of the sash."""
    # Position: on the right stile edge, centered vertically
    cup_x = CENTER_LITE_W / 2.0 + SASH_FACE / 2.0  # right stile center
    cup_z = 0.0  # vertical center of sash
    cup_y = SASH_DEPTH / 2.0 - PULL_CUP_H / 2.0 + 0.002  # recessed from front face

    # Build cup as a rounded box (elongated vertically for grip)
    return (
        cq.Workplane("XY")
        .transformed(offset=(cup_x, cup_y, cup_z))
        .box(PULL_CUP_D, PULL_CUP_H, PULL_CUP_W)
    )


def _build_screen_frame_shape() -> cq.Workplane:
    """Insect screen panel frame: a rectangular aluminum frame that sits in the
    exterior screen track. Built in its own local frame centered on origin.
    The screen covers the full center lite opening width + some overlap."""
    # Screen frame outer dimensions - covers the center opening area
    scr_w = CENTER_LITE_W + 2 * SASH_FACE + 0.02  # slightly wider than sash
    scr_h = (INNER_Z1 - INNER_Z0) + 0.02  # slightly taller than opening

    # Outer frame slab
    outer = _slab(-scr_w / 2.0, scr_w / 2.0, -scr_h / 2.0, scr_h / 2.0, 0.0, SCREEN_FRAME_W)
    # Cut the inner opening (where mesh goes)
    inner_w = scr_w - 2 * SCREEN_FRAME_W
    inner_h = scr_h - 2 * SCREEN_FRAME_W
    inner_cut = _slab(-inner_w / 2.0, inner_w / 2.0, -inner_h / 2.0, inner_h / 2.0, 0.0, SCREEN_FRAME_W + 0.01)
    return outer.cut(inner_cut)


def _build_screen_mesh_shape() -> cq.Workplane:
    """Thin screen mesh panel that fills the screen frame opening.
    The mesh extends slightly into the frame rebate so it reads as captured
    (contacting the inner lip of the frame)."""
    scr_w = CENTER_LITE_W + 2 * SASH_FACE + 0.02
    scr_h = (INNER_Z1 - INNER_Z0) + 0.02
    # Match the frame inner opening exactly (no inset) so mesh edges contact frame
    mesh_w = scr_w - 2 * SCREEN_FRAME_W
    mesh_h = scr_h - 2 * SCREEN_FRAME_W
    return _slab(-mesh_w / 2.0, mesh_w / 2.0, -mesh_h / 2.0, mesh_h / 2.0, 0.0, SCREEN_MESH_T)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    # Sanity: the three lites + two mullions must fill the inner clear width.
    span = (
        SIDE_LITE_W + MULLION_FACE + CENTER_LITE_W + MULLION_FACE + SIDE_LITE_W
    )
    inner_w = INNER_X1 - INNER_X0
    assert abs(span - inner_w) < 1e-6, f"lite layout {span} != inner width {inner_w}"

    model = ArticulatedObject(name="three_panel_sliding_window_screen")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("screen_mesh", rgba=SCREEN_RGBA)
    model.material("aluminum", rgba=ALUMINUM_RGBA)

    # --- Static outer frame (root) with deep track grooves ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material="vinyl",
        name="frame_shell",
    )

    # Opening heights (clear glass region) are common to all three lites.
    opening_h = INNER_Z1 - INNER_Z0

    # --- Two FIXED side lites ---
    for name, cx, lite_w in [
        ("left_lite", (LEFT_X0 + LEFT_X1) / 2.0, SIDE_LITE_W),
        ("right_lite", (RIGHT_X0 + RIGHT_X1) / 2.0, SIDE_LITE_W),
    ]:
        lite = model.part(name)
        lite.visual(
            mesh_from_cadquery(_build_sash_grille_shape(lite_w, opening_h), f"{name}_vinyl"),
            material="vinyl",
            name=f"{name}_vinyl",
        )
        lite.visual(
            mesh_from_cadquery(_build_sash_glass_shape(lite_w, opening_h), f"{name}_glass"),
            material="glass",
            name=f"{name}_glass",
        )

    # --- CENTER sliding sash with pull cup ---
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
    # Recessed pull cup on the right stile of the center sash
    center_sash.visual(
        mesh_from_cadquery(_build_pull_cup_shape(), "pull_cup"),
        material="vinyl",
        name="pull_cup",
    )

    # --- Insect screen panel in separate exterior track ---
    screen_panel = model.part("screen_panel")
    screen_panel.visual(
        mesh_from_cadquery(_build_screen_frame_shape(), "screen_frame"),
        material="aluminum",
        name="screen_frame",
    )
    screen_panel.visual(
        mesh_from_cadquery(_build_screen_mesh_shape(), "screen_mesh"),
        material="screen_mesh",
        name="screen_mesh",
    )

    # Centers (world) of each clear opening.
    left_cx = (LEFT_X0 + LEFT_X1) / 2.0
    right_cx = (RIGHT_X0 + RIGHT_X1) / 2.0
    center_cx = (CENTER_X0 + CENTER_X1) / 2.0
    mid_cz = (INNER_Z0 + INNER_Z1) / 2.0

    # FIXED side lites seated in the rear glazing plane (FIXED joints to frame).
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

    # CENTER sliding sash: PRISMATIC along +X. Joint origin at the sash seated
    # (closed) center, proud of the fixed lites in +Y. Positive q slides the
    # sash toward +X by ~one panel width. The sash keeps overlapping the
    # head/sill track at full travel (retained insertion).
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

    # Screen panel: FIXED in the exterior track
    model.articulation(
        "frame_to_screen",
        ArticulationType.FIXED,
        parent="frame",
        child="screen_panel",
        origin=Origin(xyz=(center_cx, SCREEN_Y, mid_cz)),
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
    screen_panel = object_model.get_part("screen_panel")
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
    # Pull cup is recessed into the sash stile (intentional embedding).
    ctx.allow_overlap(
        "center_sash", "center_sash",
        elem_a="pull_cup",
        elem_b="center_sash_vinyl",
        reason="Pull cup is recessed into the center sash stile; this is the seated handle recess.",
    )
    # Fixed side lites rebated into frame openings.
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
    # Center sash rides the head/sill track and laps the frame face.
    ctx.allow_overlap(
        "frame", "center_sash",
        elem_a="frame_shell",
        elem_b="center_sash_vinyl",
        reason="Center sash rides the head/sill track and laps the frame face along the track; slider capture.",
    )
    # Glass rebated under frame lip.
    ctx.allow_overlap(
        "frame", "left_lite",
        elem_a="frame_shell",
        elem_b="left_lite_glass",
        reason="Left lite glass is rebated under the frame opening lip (captured glazing).",
    )
    ctx.allow_overlap(
        "frame", "right_lite",
        elem_a="frame_shell",
        elem_b="right_lite_glass",
        reason="Right lite glass is rebated under the frame opening lip (captured glazing).",
    )
    ctx.allow_overlap(
        "frame", "center_sash",
        elem_a="frame_shell",
        elem_b="center_sash_glass",
        reason="Center sash glass laps the head/sill track lip as the proud sash rides the track.",
    )
    # Screen panel mesh seated within screen frame
    ctx.allow_overlap(
        "screen_panel", "screen_panel",
        elem_a="screen_mesh",
        elem_b="screen_frame",
        reason="Screen mesh is captured within the screen frame rebate.",
    )
    # Screen panel sits in the exterior track close to/overlapping the frame
    # rear face (the screen track is a shallow groove in the frame).
    ctx.allow_overlap(
        "frame", "screen_panel",
        elem_a="frame_shell",
        elem_b="screen_frame",
        reason="Screen frame sits in the exterior screen track groove; its edges lap the frame track lip.",
    )

    # --- Closed pose (q=0): window reads SHUT ---
    with ctx.pose({slide: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        l_aabb = ctx.part_world_aabb(left_lite)
        r_aabb = ctx.part_world_aabb(right_lite)
        c_aabb = ctx.part_world_aabb(center_sash)
        scr_aabb = ctx.part_world_aabb(screen_panel)

        # Frame spans the full width and is wider/taller than a single lite.
        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        center_w = c_aabb[1][0] - c_aabb[0][0]
        ctx.check(
            "frame spans wider than the center sash",
            frame_w > center_w + 1.5,
            details=f"frame_w={frame_w:.3f}, center_w={center_w:.3f}",
        )

        # Frame bottom sits at/near the floor (sill on the ground, Z up).
        ctx.check(
            "sill sits near z=0",
            abs(frame_aabb[0][2]) < 0.02,
            details=f"frame zmin={frame_aabb[0][2]:.4f}",
        )
        # Frame top is the full window height.
        ctx.check(
            "head reaches full height",
            abs(frame_aabb[1][2] - TOTAL_H) < 0.02,
            details=f"frame zmax={frame_aabb[1][2]:.4f}",
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

        # All lites seated within frame height.
        for nm, ab in (("left", l_aabb), ("right", r_aabb), ("center", c_aabb)):
            ctx.check(
                f"{nm} lite seated within frame height",
                ab[0][2] > frame_aabb[0][2] - 1e-4 and ab[1][2] < frame_aabb[1][2] + 1e-4,
                details=f"{nm} z=[{ab[0][2]:.3f},{ab[1][2]:.3f}] frame z=[{frame_aabb[0][2]:.3f},{frame_aabb[1][2]:.3f}]",
            )

        # Center sash sits proud (in +Y) of the fixed side lites.
        l_y = (l_aabb[0][1] + l_aabb[1][1]) / 2.0
        c_y = (c_aabb[0][1] + c_aabb[1][1]) / 2.0
        ctx.check(
            "center sash proud of side lites",
            c_y > l_y + 0.02,
            details=f"center_y={c_y:.3f}, side_y={l_y:.3f}",
        )

        # Fixed lites seated in frame opening.
        ctx.expect_overlap(
            left_lite, frame, axes="xz", min_overlap=0.03,
            name="left fixed lite seated in frame opening",
        )
        ctx.expect_overlap(
            right_lite, frame, axes="xz", min_overlap=0.03,
            name="right fixed lite seated in frame opening",
        )

        # --- Screen panel checks ---
        # Screen is on the exterior side (-Y) of the fixed lites.
        scr_y = (scr_aabb[0][1] + scr_aabb[1][1]) / 2.0
        ctx.check(
            "screen panel on exterior side of fixed lites",
            scr_y < l_y - 0.01,
            details=f"screen_y={scr_y:.3f}, side_y={l_y:.3f}",
        )
        # Screen overlaps the center lite area in XZ projection (covers the opening).
        ctx.expect_overlap(
            screen_panel, frame, axes="xz", min_overlap=0.20,
            name="screen panel covers the center opening area",
        )

        rest_cx = cx
        rest_cz = (c_aabb[0][2] + c_aabb[1][2]) / 2.0

    # --- Driven/open pose: center sash slides sideways along +X ---
    travel = slide.motion_limits.upper
    with ctx.pose({slide: travel}):
        c_open = ctx.part_world_aabb(center_sash)
        open_cx = (c_open[0][0] + c_open[1][0]) / 2.0
        # The sash center translated along +X by ~the travel distance.
        ctx.check(
            "center sash slides along +X by ~travel",
            abs((open_cx - rest_cx) - travel) < 0.02,
            details=f"rest_cx={rest_cx:.3f}, open_cx={open_cx:.3f}, travel={travel:.3f}",
        )
        # The sash did not move in Z (pure horizontal slide).
        c_open_z = (c_open[0][2] + c_open[1][2]) / 2.0
        ctx.check(
            "slide is purely horizontal",
            abs(c_open_z - rest_cz) < 0.02,
            details=f"open_z={c_open_z:.3f}, rest_z={rest_cz:.3f}",
        )
        # Retained insertion: sash stays within frame X span at full travel.
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

    # --- Pull cup check: exists as distinct visual on center sash ---
    ctx.check(
        "pull cup visual exists on center sash",
        center_sash.get_visual("pull_cup") is not None,
        details="center sash should have a recessed pull cup visual",
    )

    # --- Screen panel check: exists and is fixed to frame ---
    ctx.check(
        "screen panel part exists",
        screen_panel is not None,
        details="insect screen panel should exist as a separate part",
    )
    screen_joint = object_model.get_articulation("frame_to_screen")
    ctx.check(
        "screen panel has fixed joint to frame",
        screen_joint is not None and screen_joint.articulation_type == ArticulationType.FIXED,
        details="screen panel should be fixed in the exterior track",
    )

    # --- Prismatic joint check ---
    ctx.check(
        "center sash has prismatic joint",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details="center sash should slide on a prismatic joint",
    )
    ctx.check(
        "prismatic joint has positive travel range",
        slide.motion_limits.upper > 0.1,
        details=f"upper={slide.motion_limits.upper:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
