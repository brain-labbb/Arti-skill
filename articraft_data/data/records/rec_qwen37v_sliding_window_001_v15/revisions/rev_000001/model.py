from __future__ import annotations

# Three-panel horizontal sliding window with thick aluminum frame rails and deep
# track grooves along head and sill. One wide outer frame holds three vertical
# lites: two FIXED side lites and a CENTER sliding sash that sits proud of
# (overlaps) the side lites and slides sideways along the head/sill track.
#
# Variant of the white-vinyl parent: this one uses thick aluminum frame rails
# with deep track grooves visible on the inner head/sill faces.
#
# Coordinate convention:
#   +Z is up. Window stands vertically.
#     width  -> X
#     height -> Z   (sill near z=0)
#     frame depth / glazing thickness / slide-normal -> Y
#   The glass plane is the X-Z plane. The window reads SHUT at q=0; driving the
#   prismatic joint slides the center sash sideways (+X) by ~one panel width,
#   staying retained in the track.
#
# Structure:
#   - frame (static root): head, sill, two jambs, and two intermediate mullions
#     built as one CadQuery solid (slab cut by three lite openings + track groove
#     channels on head/sill inner faces). Thick aluminum profile.
#   - Track groove liners: 4 thin dark plates at groove bottoms (2 on sill,
#     2 on head) — separate visuals for testability.
#   - left_lite, right_lite (FIXED): aluminum sash ring + colonial muntin grille
#     + clear glass, seated in the rear glazing plane.
#   - center_sash (SLIDING): same construction, proud toward +Y, PRISMATIC +X.

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
TOTAL_H = 1.50            # overall height along Z (sill near z=0, head at z=TOTAL_H)

# Thick aluminum frame
FRAME_FACE = 0.090        # outer frame member face width (thick aluminum rail)
MULLION_FACE = 0.065      # intermediate mullion face width (thick aluminum)
FRAME_DEPTH = 0.140       # outer frame depth along Y (deep aluminum profile)

# Track grooves (deep channels in head/sill inner faces)
TRACK_GROOVE_W = 0.014    # groove width along Y
TRACK_GROOVE_DEPTH = 0.024  # groove depth into the rail body
TRACK_Y_REAR = -0.028     # rear track groove center Y (fixed lite track)
TRACK_Y_FRONT = 0.028     # front track groove center Y (sliding sash track)

# Three lite columns. Center is wider (the slider); sides are the fixed lites.
SIDE_LITE_W = 0.82        # clear opening width of each side lite
CENTER_LITE_W = 1.05      # clear opening width of the center lite

# Sash construction
SASH_FACE = 0.055         # sash perimeter rail/stile face width (in-plane)
SASH_DEPTH = 0.055        # sash depth along Y
GLASS_T = 0.008           # glazing thickness along Y

# Colonial grille (divided lite)
GRILLE_COLS = 4
GRILLE_ROWS = 5
MUNTIN_T = 0.020          # muntin bar face width
MUNTIN_DEPTH = 0.020      # muntin bar depth along Y

# Y layout (depth). Frame box centered on y=0. The two fixed lites sit in the
# rear glazing plane; the center sliding sash sits proud toward +Y.
FIXED_LITE_Y = -0.028     # fixed side lites: rear track plane center (Y)
SLIDE_SASH_Y = 0.040      # center sash sits proud toward +Y (front track)

REBATE = 0.005            # glass tucks under the sash/muntin lip

# Groove liner plate (thin dark visual at groove bottom for testability)
LINER_THICKNESS = 0.003

# ---------------------------------------------------------------------------
# Derived layout
# ---------------------------------------------------------------------------

HALF_W = TOTAL_W / 2.0

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

ALUMINUM_RGBA = (0.72, 0.74, 0.76, 1.0)   # brushed silver aluminum
GLASS_RGBA = (0.50, 0.58, 0.64, 0.32)     # cool grey-blue, semi-transparent
TRACK_LINER_RGBA = (0.30, 0.32, 0.35, 1.0)  # dark recessed track groove liner


# ---------------------------------------------------------------------------
# Geometry helpers (CadQuery). All in meters, world frame.
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
    """Static outer frame: a full slab cut by the three lite openings, then
    deep track groove channels cut into the inner head/sill faces."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)

    cut_depth = FRAME_DEPTH + 0.02
    left_cut = _slab(LEFT_X0, LEFT_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    center_cut = _slab(CENTER_X0, CENTER_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    right_cut = _slab(RIGHT_X0, RIGHT_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)

    body = outer.cut(left_cut).cut(center_cut).cut(right_cut)

    # Deep track grooves on sill inner face (going downward into the sill body).
    eps = 0.002  # small extension for clean boolean cut
    for y_c in (TRACK_Y_REAR, TRACK_Y_FRONT):
        groove = _slab(
            INNER_X0, INNER_X1,
            FRAME_FACE - TRACK_GROOVE_DEPTH,
            FRAME_FACE + eps,
            y_c, TRACK_GROOVE_W,
        )
        body = body.cut(groove)

    # Deep track grooves on head inner face (going upward into the head body).
    for y_c in (TRACK_Y_REAR, TRACK_Y_FRONT):
        groove = _slab(
            INNER_X0, INNER_X1,
            TOTAL_H - FRAME_FACE - eps,
            TOTAL_H - FRAME_FACE + TRACK_GROOVE_DEPTH,
            y_c, TRACK_GROOVE_W,
        )
        body = body.cut(groove)

    return body


def _build_track_groove_liner(y_center: float, z_bottom: float, z_top: float) -> cq.Workplane:
    """Thin dark plate representing a track groove liner at the groove bottom.
    Spans the inner width, sits inside the groove channel."""
    # Slightly narrower than the groove to avoid wall contact
    liner_w = TRACK_GROOVE_W - 0.004
    return _slab(INNER_X0, INNER_X1, z_bottom, z_top, y_center, liner_w)


def _build_sash_grille_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """One sash built in its OWN local frame, centered on local origin.
    Outer sash slab cut by the clear opening, then colonial muntin grid."""
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
    """Single clear pane filling the sash opening, rebated under the sash lip."""
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
    """Add a sash part (aluminum ring + colonial grille + clear glass)."""
    lite = model.part(name)
    lite.visual(
        mesh_from_cadquery(_build_sash_grille_shape(opening_w, opening_h), f"{name}_aluminum"),
        material="aluminum",
        name=f"{name}_aluminum",
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
    span = SIDE_LITE_W + MULLION_FACE + CENTER_LITE_W + MULLION_FACE + SIDE_LITE_W
    inner_w = INNER_X1 - INNER_X0
    assert abs(span - inner_w) < 1e-6, f"lite layout {span} != inner width {inner_w}"

    model = ArticulatedObject(name="aluminum_sliding_window")
    model.material("aluminum", rgba=ALUMINUM_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("track_liner", rgba=TRACK_LINER_RGBA)

    # --- Static outer frame (root) with deep track grooves ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material="aluminum",
        name="frame_shell",
    )

    # Track groove liner visuals (4 dark plates at groove bottoms)
    opening_h = INNER_Z1 - INNER_Z0

    # Sill grooves: liners sit at the bottom of the sill groove cuts
    sill_groove_bottom = FRAME_FACE - TRACK_GROOVE_DEPTH
    sill_groove_top = sill_groove_bottom + LINER_THICKNESS
    for idx, y_c in enumerate((TRACK_Y_REAR, TRACK_Y_FRONT)):
        suffix = "rear" if idx == 0 else "front"
        frame.visual(
            mesh_from_cadquery(
                _build_track_groove_liner(y_c, sill_groove_bottom, sill_groove_top),
                f"sill_track_{suffix}",
            ),
            material="track_liner",
            name=f"sill_track_{suffix}",
        )

    # Head grooves: liners sit at the top of the head groove cuts
    head_groove_top = TOTAL_H - FRAME_FACE + TRACK_GROOVE_DEPTH
    head_groove_bottom = head_groove_top - LINER_THICKNESS
    for idx, y_c in enumerate((TRACK_Y_REAR, TRACK_Y_FRONT)):
        suffix = "rear" if idx == 0 else "front"
        frame.visual(
            mesh_from_cadquery(
                _build_track_groove_liner(y_c, head_groove_bottom, head_groove_top),
                f"head_track_{suffix}",
            ),
            material="track_liner",
            name=f"head_track_{suffix}",
        )

    # --- Two FIXED side lites + the CENTER sliding sash ---
    _add_lite(model, "left_lite", SIDE_LITE_W, opening_h)
    _add_lite(model, "right_lite", SIDE_LITE_W, opening_h)
    _add_lite(model, "center_sash", CENTER_LITE_W, opening_h)

    # Centers (world) of each clear opening.
    left_cx = (LEFT_X0 + LEFT_X1) / 2.0
    right_cx = (RIGHT_X0 + RIGHT_X1) / 2.0
    center_cx = (CENTER_X0 + CENTER_X1) / 2.0
    mid_cz = (INNER_Z0 + INNER_Z1) / 2.0

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

    # CENTER sliding sash: PRISMATIC along +X on the front track.
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
            elem_b=f"{nm}_aluminum",
            reason="Clear pane is rebated under the sash/muntin lip so it reads captured, not floating.",
        )
    # Fixed side lites rebated into the outer-frame opening.
    ctx.allow_overlap(
        "frame", "left_lite",
        elem_a="frame_shell",
        elem_b="left_lite_aluminum",
        reason="Left fixed lite is rebated into the frame opening; its sash ring laps the jamb/mullion edge.",
    )
    ctx.allow_overlap(
        "frame", "right_lite",
        elem_a="frame_shell",
        elem_b="right_lite_aluminum",
        reason="Right fixed lite is rebated into the frame opening; its sash ring laps the jamb/mullion edge.",
    )
    # Center sliding sash rides the head/sill track, lapping the frame face.
    ctx.allow_overlap(
        "frame", "center_sash",
        elem_a="frame_shell",
        elem_b="center_sash_aluminum",
        reason="Center sash rides the head/sill track and laps the frame face along the track.",
    )
    # Glass rebated under frame opening lip.
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
        reason="Center sash glass laps the head/sill track lip (captured glazing).",
    )
    # Track groove liners sit inside the groove cuts (intentional embedding).
    for liner_name in ("sill_track_rear", "sill_track_front",
                       "head_track_rear", "head_track_front"):
        ctx.allow_overlap(
            "frame", "frame",
            elem_a="frame_shell",
            elem_b=liner_name,
            reason=f"Track groove liner '{liner_name}' sits inside the groove cut in the frame rail.",
        )

    # --- Thick aluminum frame verification ---
    with ctx.pose({slide: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)

        # Frame depth (Y extent) should be thick (>= 0.130m for deep aluminum profile)
        frame_depth = frame_aabb[1][1] - frame_aabb[0][1]
        ctx.check(
            "frame has thick aluminum depth",
            frame_depth >= 0.130,
            details=f"frame_depth={frame_depth:.3f}",
        )

        # Frame face width (X extent of jamb) should be thick (>= 0.080m)
        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        ctx.check(
            "frame spans full window width",
            abs(frame_w - TOTAL_W) < 0.02,
            details=f"frame_w={frame_w:.3f}",
        )

        # Sill sits near z=0
        ctx.check(
            "sill sits near z=0",
            abs(frame_aabb[0][2]) < 0.02,
            details=f"frame zmin={frame_aabb[0][2]:.4f}",
        )
        # Head reaches full height
        ctx.check(
            "head reaches full height",
            abs(frame_aabb[1][2] - TOTAL_H) < 0.02,
            details=f"frame zmax={frame_aabb[1][2]:.4f}",
        )

    # --- Track groove presence (deep grooves on head and sill) ---
    # Sill track liners should be near the sill inner face (low Z)
    sill_rear = ctx.part_element_world_aabb(frame, elem="sill_track_rear")
    sill_front = ctx.part_element_world_aabb(frame, elem="sill_track_front")
    head_rear = ctx.part_element_world_aabb(frame, elem="head_track_rear")
    head_front = ctx.part_element_world_aabb(frame, elem="head_track_front")

    ctx.check(
        "sill track grooves exist near sill",
        sill_rear is not None and sill_front is not None
        and sill_rear[0][2] < FRAME_FACE + 0.01
        and sill_front[0][2] < FRAME_FACE + 0.01,
        details=f"sill_rear_zmin={sill_rear[0][2]:.4f}, sill_front_zmin={sill_front[0][2]:.4f}",
    )
    ctx.check(
        "head track grooves exist near head",
        head_rear is not None and head_front is not None
        and head_rear[1][2] > TOTAL_H - FRAME_FACE - 0.01
        and head_front[1][2] > TOTAL_H - FRAME_FACE - 0.01,
        details=f"head_rear_zmax={head_rear[1][2]:.4f}, head_front_zmax={head_front[1][2]:.4f}",
    )
    # Head grooves should be well above sill grooves
    ctx.check(
        "head grooves above sill grooves",
        head_rear[0][2] > sill_rear[1][2] + 0.5,
        details=f"head_zmin={head_rear[0][2]:.3f}, sill_zmax={sill_rear[1][2]:.3f}",
    )
    # Two parallel grooves on sill: rear and front should be at different Y
    ctx.check(
        "sill has two parallel track grooves at different Y",
        abs((sill_rear[0][1] + sill_rear[1][1]) / 2.0
            - (sill_front[0][1] + sill_front[1][1]) / 2.0) > 0.03,
        details="rear and front grooves are at distinct Y positions",
    )

    # --- Closed pose (q=0): window reads SHUT ---
    with ctx.pose({slide: 0.0}):
        l_aabb = ctx.part_world_aabb(left_lite)
        r_aabb = ctx.part_world_aabb(right_lite)
        c_aabb = ctx.part_world_aabb(center_sash)

        # Three lites ordered left -> center -> right
        lx = (l_aabb[0][0] + l_aabb[1][0]) / 2.0
        cx_val = (c_aabb[0][0] + c_aabb[1][0]) / 2.0
        rx = (r_aabb[0][0] + r_aabb[1][0]) / 2.0
        ctx.check(
            "lites ordered left-center-right",
            lx < cx_val < rx,
            details=f"left_x={lx:.3f}, center_x={cx_val:.3f}, right_x={rx:.3f}",
        )

        # All lites seated within frame height
        for nm, ab in (("left", l_aabb), ("right", r_aabb), ("center", c_aabb)):
            ctx.check(
                f"{nm} lite seated within frame height",
                ab[0][2] > frame_aabb[0][2] - 1e-4 and ab[1][2] < frame_aabb[1][2] + 1e-4,
                details=f"{nm} z=[{ab[0][2]:.3f},{ab[1][2]:.3f}]",
            )

        # Center sash sits proud (in +Y) of the fixed side lites
        l_y = (l_aabb[0][1] + l_aabb[1][1]) / 2.0
        c_y = (c_aabb[0][1] + c_aabb[1][1]) / 2.0
        ctx.check(
            "center sash proud of side lites",
            c_y > l_y + 0.02,
            details=f"center_y={c_y:.3f}, side_y={l_y:.3f}",
        )

        # Fixed lites seated in frame opening
        ctx.expect_overlap(
            left_lite, frame, axes="xz", min_overlap=0.03,
            name="left fixed lite seated in frame opening",
        )
        ctx.expect_overlap(
            right_lite, frame, axes="xz", min_overlap=0.03,
            name="right fixed lite seated in frame opening",
        )

        rest_cx = cx_val
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
        # Pure horizontal slide (no Z movement)
        c_open_z = (c_open[0][2] + c_open[1][2]) / 2.0
        ctx.check(
            "slide is purely horizontal (no Z drift)",
            abs(c_open_z - rest_cz) < 0.02,
            details=f"open_z={c_open_z:.3f}, rest_z={rest_cz:.3f}",
        )
        # Retained insertion: sash stays within frame X span at full travel
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

    # --- Prismatic joint verification ---
    ctx.check(
        "prismatic joint exists for center sash",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"joint type={slide.articulation_type}",
    )
    ctx.check(
        "prismatic axis is horizontal (X direction)",
        abs(slide.axis[0]) > 0.9 and abs(slide.axis[1]) < 0.1 and abs(slide.axis[2]) < 0.1,
        details=f"axis={slide.axis}",
    )
    ctx.check(
        "slide travel is meaningful (> 0.3m)",
        slide.motion_limits.upper > 0.3,
        details=f"upper={slide.motion_limits.upper:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
