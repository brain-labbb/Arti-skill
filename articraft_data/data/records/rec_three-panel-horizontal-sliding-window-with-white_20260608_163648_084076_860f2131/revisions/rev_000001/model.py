from __future__ import annotations

# Three-panel horizontal sliding window, white vinyl/PVC frame with "colonial"
# divided-lite grilles. One wide outer frame holds three vertical lites: two
# FIXED side lites and a CENTER sliding sash that sits proud of (overlaps) the
# side lites and slides sideways along the head/sill track.
#
# Coordinate convention (per brief):
#   +Z is up. Window stands vertically.
#     - width  -> X
#     - height -> Z   (sill near z=0)
#     - frame depth / glazing thickness / slide-normal -> Y
#   The glass plane is the X-Z plane. The window reads SHUT at q=0; driving the
#   prismatic joint slides the center sash sideways (+X) by ~one panel width,
#   staying retained in the track.
#
# Structure:
#   - frame (static root): head, sill, two jambs, and the two intermediate
#     mullions, built as one CadQuery solid (a slab cut by the three lite
#     openings) -> a true hollow profile, not a box with a painted hole.
#   - left_lite, right_lite (FIXED): each a vinyl sash ring + colonial muntin
#     grille + clear glass, seated in the outer frame plane (FIXED joints).
#   - center_sash (SLIDING): same construction, but proud of the side lites in
#     +Y so it can pass in front of them; PRISMATIC along +X.

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

# Three lite columns. Center is wider (the slider); sides are the fixed lites.
# Three openings inside the outer frame, separated by two mullions.
# inner clear width = TOTAL_W - 2*FRAME_FACE = 2.86; minus two mullions (0.12)
# leaves 2.74 for the three lites: center 1.04 + two sides of 0.85.
SIDE_LITE_W = 0.85        # clear opening width of each side lite
CENTER_LITE_W = 1.04      # clear opening width of the center lite

# Sash construction
SASH_FACE = 0.055         # sash perimeter rail/stile face width (in-plane)
SASH_DEPTH = 0.055        # sash depth along Y
GLASS_T = 0.008           # glazing thickness along Y

# Colonial grille (divided lite): each lite is a grid of small panes.
GRILLE_COLS = 4           # 4 columns of panes
GRILLE_ROWS = 5           # 5 rows of panes
MUNTIN_T = 0.020          # muntin bar face width (in-plane)
MUNTIN_DEPTH = 0.020      # muntin bar depth along Y

# Y layout (depth). Frame box centered on y=0. The two fixed lites sit in the
# rear glazing plane; the center sliding sash sits proud toward +Y so it can
# slide in front of the side lites without colliding.
FIXED_LITE_Y = -0.020     # fixed side lites: rear glazing plane center (Y)
SLIDE_SASH_Y = 0.052      # center sash sits proud toward +Y (passes in front)
# Fixed lite front face = FIXED_LITE_Y + SASH_DEPTH/2 = 0.0075
# Center sash back face  = SLIDE_SASH_Y - SASH_DEPTH/2 = 0.0245
# -> ~17 mm air gap in Y so the proud sash never touches the rear lites.

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
# left lite | mullion | center lite | mullion | right lite
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
# (LEFT/CENTER/RIGHT widths chosen so RIGHT_X1 lands on INNER_X1; assert in build.)

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

VINYL_RGBA = (0.94, 0.95, 0.96, 1.0)     # bright white vinyl/PVC
GLASS_RGBA = (0.50, 0.58, 0.64, 0.32)    # cool grey-blue, semi-transparent


# ---------------------------------------------------------------------------
# Geometry helpers (CadQuery). All authored directly in meters, world frame.
# ---------------------------------------------------------------------------

def _slab(x0: float, x1: float, z0: float, z1: float, y_center: float, depth: float) -> cq.Workplane:
    """Axis-aligned box spanning [x0,x1] x [z0,z1] in the X-Z plane, centered on
    y_center with the given Y depth. Built on the XY workplane where the SDK
    treats local Y as world Y (depth) and local Z as world Z (height)."""
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
    head, sill, two jambs and the two intermediate mullions as one solid."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)

    cut_depth = FRAME_DEPTH + 0.02  # through-cut clearance in Y
    left_cut = _slab(LEFT_X0, LEFT_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    center_cut = _slab(CENTER_X0, CENTER_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    right_cut = _slab(RIGHT_X0, RIGHT_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)

    return outer.cut(left_cut).cut(center_cut).cut(right_cut)


def _build_sash_grille_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """One sash built in its OWN local frame, centered on local origin:
      - local X in [-opening_w/2 - SASH_FACE, +opening_w/2 + SASH_FACE]
      - local Z in [-opening_h/2 - SASH_FACE, +opening_h/2 + SASH_FACE]
      - local Y is the sash depth, centered at 0
    Construction: outer sash slab cut by the clear opening, then the colonial
    muntin grid (vertical + horizontal bars) unioned back in across the opening.

    Returns the vinyl (frame + muntins) workplane. The glass is built separately.
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

    # Colonial muntin grid across the opening. The muntins sit at the front face
    # of the sash depth so they read as applied grilles; we make them span the
    # full sash depth (simple, robust, reads as true divided lites).
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
    """Single clear pane filling the sash opening, in the same sash-local frame.
    The pane tucks slightly under the sash lip (rebate) so it reads captured."""
    ow = opening_w + 2 * REBATE
    oh = opening_h + 2 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


# ---------------------------------------------------------------------------
# Part builders
# ---------------------------------------------------------------------------

def _add_lite(
    model: ArticulatedObject,
    name: str,
    opening_w: float,
    opening_h: float,
) -> None:
    """Add a sash part (vinyl ring + colonial grille + clear glass) authored in
    its own local frame centered on the local origin."""
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
    # Sanity: the three lites + two mullions must fill the inner clear width.
    span = (
        SIDE_LITE_W + MULLION_FACE + CENTER_LITE_W + MULLION_FACE + SIDE_LITE_W
    )
    inner_w = INNER_X1 - INNER_X0
    assert abs(span - inner_w) < 1e-6, f"lite layout {span} != inner width {inner_w}"

    model = ArticulatedObject(name="three_panel_sliding_window")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material="vinyl",
        name="frame_shell",
    )

    # Opening heights (clear glass region) are common to all three lites.
    opening_h = INNER_Z1 - INNER_Z0

    # --- Two FIXED side lites + the CENTER sliding sash ---
    _add_lite(model, "left_lite", SIDE_LITE_W, opening_h)
    _add_lite(model, "right_lite", SIDE_LITE_W, opening_h)
    _add_lite(model, "center_sash", CENTER_LITE_W, opening_h)

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
    # sash toward +X (toward the right lite) by ~one panel width. The sash keeps
    # overlapping the head/sill track and the right mullion at full travel
    # (retained insertion).
    slide_travel = SIDE_LITE_W * 0.92  # ~one side-panel width of opening
    model.articulation(
        "frame_to_center_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="center_sash",
        origin=Origin(xyz=(center_cx, SLIDE_SASH_Y, mid_cz)),
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
    # The two FIXED side lites are rebated into the outer-frame opening: each
    # sash ring laps the jamb/mullion opening edge (the real glazing rebate that
    # captures a fixed lite in a vinyl frame).
    ctx.allow_overlap(
        "frame", "left_lite",
        elem_a="frame_shell",
        elem_b="left_lite_vinyl",
        reason="Left fixed lite is rebated into the frame opening; its sash ring laps the jamb/mullion edge (seated capture).",
    )
    ctx.allow_overlap(
        "frame", "right_lite",
        elem_a="frame_shell",
        elem_b="right_lite_vinyl",
        reason="Right fixed lite is rebated into the frame opening; its sash ring laps the jamb/mullion edge (seated capture).",
    )
    # The center sliding sash sits proud of the frame in +Y and rides the
    # head/sill track; its ring laps the frame face along the track. While
    # sliding it passes in front of the right fixed lite. The sash is offset in
    # +Y so glass faces never interpenetrate the rear lites.
    ctx.allow_overlap(
        "frame", "center_sash",
        elem_a="frame_shell",
        elem_b="center_sash_vinyl",
        reason="Center sash rides the head/sill track and laps the frame face along the track; this is the slider capture.",
    )
    # Each lite's glass pane is rebated past the opening edge and laps the frame
    # opening lip (the glazing rebate that keeps glass captured in the frame).
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
        reason="Center sash glass laps the head/sill track lip as the proud sash rides the track (captured glazing).",
    )

    # --- Closed pose (q=0): window reads SHUT ---
    with ctx.pose({slide: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        l_aabb = ctx.part_world_aabb(left_lite)
        r_aabb = ctx.part_world_aabb(right_lite)
        c_aabb = ctx.part_world_aabb(center_sash)

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

        # Three lites ordered left -> center -> right with no big X gaps.
        lx = (l_aabb[0][0] + l_aabb[1][0]) / 2.0
        cx = (c_aabb[0][0] + c_aabb[1][0]) / 2.0
        rx = (r_aabb[0][0] + r_aabb[1][0]) / 2.0
        ctx.check(
            "lites ordered left-center-right",
            lx < cx < rx,
            details=f"left_x={lx:.3f}, center_x={cx:.3f}, right_x={rx:.3f}",
        )

        # All lites seated inside the head/sill (within the frame Z extent).
        for nm, ab in (("left", l_aabb), ("right", r_aabb), ("center", c_aabb)):
            ctx.check(
                f"{nm} lite seated within frame height",
                ab[0][2] > frame_aabb[0][2] - 1e-4 and ab[1][2] < frame_aabb[1][2] + 1e-4,
                details=f"{nm} z=[{ab[0][2]:.3f},{ab[1][2]:.3f}] frame z=[{frame_aabb[0][2]:.3f},{frame_aabb[1][2]:.3f}]",
            )

        # Center sash sits proud (in +Y) of the fixed side lites (it slides in
        # front of them) -> closed pose still reads as a flat window because the
        # offset is small relative to width/height.
        l_y = (l_aabb[0][1] + l_aabb[1][1]) / 2.0
        c_y = (c_aabb[0][1] + c_aabb[1][1]) / 2.0
        ctx.check(
            "center sash proud of side lites",
            c_y > l_y + 0.02,
            details=f"center_y={c_y:.3f}, side_y={l_y:.3f}",
        )

        # Fixed lites are seated in the frame: their rings lap the frame opening
        # edges (rebate capture), proven as projected overlap in the glass plane.
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
        # Retained insertion: the sliding sash still overlaps the static frame
        # (head/sill track + right jamb/mullion) at full travel; it does not exit
        # the track.
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
