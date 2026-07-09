from __future__ import annotations

# Variant 12: Three-panel horizontal sliding window with wider fixed center pane.
# Right sash slides sideways; tilt-in latch pair on revolute joints; two roller
# blocks at the bottom of the moving sash.
#
# Coordinate convention:
#   +Z is up. Window stands vertically.
#     - width  -> X
#     - height -> Z   (sill near z=0)
#     - frame depth / glazing thickness / slide-normal -> Y
#   The glass plane is the X-Z plane. The window reads SHUT at q=0; driving the
#   prismatic joint slides the right sash sideways (-X) by ~one panel width,
#   staying retained in the track. Tilt-in latches pivot outward on revolute
#   joints to release the sash for tilt-in cleaning.

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

# Three lite columns: left fixed | mullion | center fixed (WIDER) | mullion | right slider
LEFT_LITE_W = 0.75
CENTER_LITE_W = 1.24       # wider fixed center pane (picture window)
RIGHT_LITE_W = 0.75        # sliding sash

SASH_FACE = 0.055
SASH_DEPTH = 0.055
GLASS_T = 0.008

GRILLE_COLS = 4
GRILLE_ROWS = 5
MUNTIN_T = 0.020
MUNTIN_DEPTH = 0.020

FIXED_LITE_Y = -0.020
SLIDE_SASH_Y = 0.052

REBATE = 0.005

# Latch dimensions (small tilt-in latch tabs)
LATCH_W = 0.028
LATCH_H = 0.038
LATCH_D = 0.012

# Roller block dimensions (at bottom of sliding sash)
ROLLER_W = 0.025
ROLLER_H = 0.010
ROLLER_D = 0.020

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

VINYL_RGBA = (0.94, 0.95, 0.96, 1.0)
GLASS_RGBA = (0.50, 0.58, 0.64, 0.32)
LATCH_RGBA = (0.70, 0.72, 0.74, 1.0)
ROLLER_RGBA = (0.25, 0.25, 0.27, 1.0)

# ---------------------------------------------------------------------------
# Derived layout
# ---------------------------------------------------------------------------

HALF_W = TOTAL_W / 2.0

INNER_X0 = -HALF_W + FRAME_FACE
INNER_X1 = HALF_W - FRAME_FACE
INNER_Z0 = FRAME_FACE
INNER_Z1 = TOTAL_H - FRAME_FACE

LEFT_X0 = INNER_X0
LEFT_X1 = LEFT_X0 + LEFT_LITE_W
MUL0_X0 = LEFT_X1
MUL0_X1 = MUL0_X0 + MULLION_FACE
CENTER_X0 = MUL0_X1
CENTER_X1 = CENTER_X0 + CENTER_LITE_W
MUL1_X0 = CENTER_X1
MUL1_X1 = MUL1_X0 + MULLION_FACE
RIGHT_X0 = MUL1_X1
RIGHT_X1 = INNER_X1


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
    head, sill, two jambs and the two intermediate mullions as one solid."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)

    cut_depth = FRAME_DEPTH + 0.02
    left_cut = _slab(LEFT_X0, LEFT_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    center_cut = _slab(CENTER_X0, CENTER_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    right_cut = _slab(RIGHT_X0, RIGHT_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)

    return outer.cut(left_cut).cut(center_cut).cut(right_cut)


def _build_sash_grille_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """One sash built in its OWN local frame, centered on local origin.
    Outer sash slab cut by the clear opening, then colonial muntin grid unioned
    back in across the opening."""
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


def _build_latch_shape() -> cq.Workplane:
    """Small tilt-in latch tab in its own local frame. Pivot is at the bottom
    center of the latch; the tab extends upward."""
    return (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, LATCH_H / 2.0))
        .box(LATCH_W, LATCH_D, LATCH_H)
    )


def _build_roller_shape() -> cq.Workplane:
    """Small roller block in its own local frame, centered on origin."""
    return (
        cq.Workplane("XY")
        .box(ROLLER_W, ROLLER_D, ROLLER_H)
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    # Sanity: the three lites + two mullions must fill the inner clear width.
    span = LEFT_LITE_W + MULLION_FACE + CENTER_LITE_W + MULLION_FACE + RIGHT_LITE_W
    inner_w = INNER_X1 - INNER_X0
    assert abs(span - inner_w) < 1e-6, f"lite layout {span} != inner width {inner_w}"

    model = ArticulatedObject(name="three_panel_sliding_window_v12")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("latch_metal", rgba=LATCH_RGBA)
    model.material("roller_dark", rgba=ROLLER_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material="vinyl",
        name="frame_shell",
    )

    # Opening heights (clear glass region) are common to all three lites.
    opening_h = INNER_Z1 - INNER_Z0

    # --- Left fixed lite ---
    left_lite = model.part("left_lite")
    left_lite.visual(
        mesh_from_cadquery(_build_sash_grille_shape(LEFT_LITE_W, opening_h), "left_lite_vinyl"),
        material="vinyl",
        name="left_lite_vinyl",
    )
    left_lite.visual(
        mesh_from_cadquery(_build_sash_glass_shape(LEFT_LITE_W, opening_h), "left_lite_glass"),
        material="glass",
        name="left_lite_glass",
    )

    # --- Center fixed lite (WIDER picture window) ---
    center_fixed = model.part("center_fixed")
    center_fixed.visual(
        mesh_from_cadquery(_build_sash_grille_shape(CENTER_LITE_W, opening_h), "center_fixed_vinyl"),
        material="vinyl",
        name="center_fixed_vinyl",
    )
    center_fixed.visual(
        mesh_from_cadquery(_build_sash_glass_shape(CENTER_LITE_W, opening_h), "center_fixed_glass"),
        material="glass",
        name="center_fixed_glass",
    )

    # --- Right sliding sash ---
    right_sash = model.part("right_sash")
    right_sash.visual(
        mesh_from_cadquery(_build_sash_grille_shape(RIGHT_LITE_W, opening_h), "right_sash_vinyl"),
        material="vinyl",
        name="right_sash_vinyl",
    )
    right_sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(RIGHT_LITE_W, opening_h), "right_sash_glass"),
        material="glass",
        name="right_sash_glass",
    )

    # Roller blocks at bottom of sliding sash (two rollers, spread apart)
    roller_offset_x = RIGHT_LITE_W * 0.30
    roller_z = -(opening_h / 2.0 + SASH_FACE) - ROLLER_H / 2.0  # below bottom rail
    right_sash.visual(
        mesh_from_cadquery(_build_roller_shape(), "roller_left"),
        material="roller_dark",
        name="roller_left",
        origin=Origin(xyz=(-roller_offset_x, 0.0, roller_z)),
    )
    right_sash.visual(
        mesh_from_cadquery(_build_roller_shape(), "roller_right"),
        material="roller_dark",
        name="roller_right",
        origin=Origin(xyz=(roller_offset_x, 0.0, roller_z)),
    )

    # --- Tilt-in latches (separate parts on revolute joints) ---
    top_latch = model.part("top_latch")
    top_latch.visual(
        mesh_from_cadquery(_build_latch_shape(), "top_latch_body"),
        material="latch_metal",
        name="top_latch_body",
    )

    bottom_latch = model.part("bottom_latch")
    bottom_latch.visual(
        mesh_from_cadquery(_build_latch_shape(), "bottom_latch_body"),
        material="latch_metal",
        name="bottom_latch_body",
    )

    # Centers (world) of each clear opening.
    left_cx = (LEFT_X0 + LEFT_X1) / 2.0
    right_cx = (RIGHT_X0 + RIGHT_X1) / 2.0
    center_cx = (CENTER_X0 + CENTER_X1) / 2.0
    mid_cz = (INNER_Z0 + INNER_Z1) / 2.0

    # FIXED left lite seated in the rear glazing plane
    model.articulation(
        "frame_to_left_lite",
        ArticulationType.FIXED,
        parent="frame",
        child="left_lite",
        origin=Origin(xyz=(left_cx, FIXED_LITE_Y, mid_cz)),
    )

    # FIXED center lite (wider picture window) seated in the rear glazing plane
    model.articulation(
        "frame_to_center_fixed",
        ArticulationType.FIXED,
        parent="frame",
        child="center_fixed",
        origin=Origin(xyz=(center_cx, FIXED_LITE_Y, mid_cz)),
    )

    # RIGHT sliding sash: PRISMATIC along -X. Positive q slides the sash toward
    # -X (toward the center/left) to open the window. The sash sits proud in +Y.
    slide_travel = LEFT_LITE_W * 0.90
    model.articulation(
        "frame_to_right_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="right_sash",
        origin=Origin(xyz=(right_cx, SLIDE_SASH_Y, mid_cz)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.5, lower=0.0, upper=slide_travel),
    )

    # Tilt-in latches: REVOLUTE on the right sash. The latch pivot is on the
    # left stile (meeting rail) of the sash, near top and bottom. Positive q
    # tilts the latch outward (away from the sash face, toward +Y).
    # Pivot at the bottom edge of each latch; latch extends upward when locked.
    latch_x_local = -(RIGHT_LITE_W / 2.0 + SASH_FACE / 2.0)  # on meeting stile
    latch_y_local = SASH_DEPTH / 2.0 + LATCH_D / 2.0  # proud of sash face
    top_latch_z_local = opening_h / 2.0 - 0.08  # near top
    bottom_latch_z_local = -(opening_h / 2.0) + 0.08  # near bottom

    model.articulation(
        "sash_to_top_latch",
        ArticulationType.REVOLUTE,
        parent="right_sash",
        child="top_latch",
        origin=Origin(xyz=(latch_x_local, latch_y_local, top_latch_z_local)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.3),
    )

    model.articulation(
        "sash_to_bottom_latch",
        ArticulationType.REVOLUTE,
        parent="right_sash",
        child="bottom_latch",
        origin=Origin(xyz=(latch_x_local, latch_y_local, bottom_latch_z_local)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.3),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    left_lite = object_model.get_part("left_lite")
    center_fixed = object_model.get_part("center_fixed")
    right_sash = object_model.get_part("right_sash")
    top_latch = object_model.get_part("top_latch")
    bottom_latch = object_model.get_part("bottom_latch")

    slide = object_model.get_articulation("frame_to_right_sash")
    top_latch_joint = object_model.get_articulation("sash_to_top_latch")
    bottom_latch_joint = object_model.get_articulation("sash_to_bottom_latch")

    opening_h = INNER_Z1 - INNER_Z0

    # --- Intentional overlaps ---
    # Glass panes tuck under the vinyl/muntin lip on each sash (captured glass).
    for nm in ("left_lite", "center_fixed", "right_sash"):
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
        reason="Left fixed lite is rebated into the frame opening; its sash ring laps the jamb/mullion edge.",
    )
    ctx.allow_overlap(
        "frame", "center_fixed",
        elem_a="frame_shell", elem_b="center_fixed_vinyl",
        reason="Center fixed lite is rebated into the frame opening; its sash ring laps the mullion edge.",
    )
    # Adjacent fixed lites' sash rings overlap at the shared mullion (both are
    # seated in the same rear glazing plane and their perimeter rails extend
    # into the mullion rebate region).
    ctx.allow_overlap(
        "left_lite", "center_fixed",
        elem_a="left_lite_vinyl", elem_b="center_fixed_vinyl",
        reason="Adjacent fixed lite sash rails share the mullion rebate; perimeter rails lap at the mullion (seated capture).",
    )
    ctx.allow_overlap(
        "frame", "left_lite",
        elem_a="frame_shell", elem_b="left_lite_glass",
        reason="Left lite glass is rebated under the frame opening lip (captured glazing).",
    )
    ctx.allow_overlap(
        "frame", "center_fixed",
        elem_a="frame_shell", elem_b="center_fixed_glass",
        reason="Center fixed glass is rebated under the frame opening lip (captured glazing).",
    )

    # Sliding sash rides the track proud of the frame
    ctx.allow_overlap(
        "frame", "right_sash",
        elem_a="frame_shell", elem_b="right_sash_vinyl",
        reason="Right sash rides the head/sill track and laps the frame face along the track; this is the slider capture.",
    )
    ctx.allow_overlap(
        "frame", "right_sash",
        elem_a="frame_shell", elem_b="right_sash_glass",
        reason="Right sash glass laps the head/sill track lip as the proud sash rides the track.",
    )
    # Roller blocks at the sash bottom sit on the sill track and slightly
    # penetrate the frame sill surface (seated contact).
    ctx.allow_overlap(
        "frame", "right_sash",
        elem_a="frame_shell", elem_b="roller_left",
        reason="Left roller block sits on the sill track, penetrating the sill surface (seated contact).",
    )
    ctx.allow_overlap(
        "frame", "right_sash",
        elem_a="frame_shell", elem_b="roller_right",
        reason="Right roller block sits on the sill track, penetrating the sill surface (seated contact).",
    )

    # Latches are mounted on the sash face - small overlap at pivot mount
    ctx.allow_overlap(
        "right_sash", "top_latch",
        elem_a="right_sash_vinyl", elem_b="top_latch_body",
        reason="Top latch pivot is embedded in the sash stile face (captured pivot pin).",
    )
    ctx.allow_overlap(
        "right_sash", "bottom_latch",
        elem_a="right_sash_vinyl", elem_b="bottom_latch_body",
        reason="Bottom latch pivot is embedded in the sash stile face (captured pivot pin).",
    )

    # --- Structural checks ---
    with ctx.pose({slide: 0.0, top_latch_joint: 0.0, bottom_latch_joint: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        c_aabb = ctx.part_world_aabb(center_fixed)

        # Center fixed pane is wider than the side panels
        center_w = c_aabb[1][0] - c_aabb[0][0]
        l_aabb = ctx.part_world_aabb(left_lite)
        r_aabb = ctx.part_world_aabb(right_sash)
        left_w = l_aabb[1][0] - l_aabb[0][0]
        right_w = r_aabb[1][0] - r_aabb[0][0]

        ctx.check(
            "center fixed pane wider than side panels",
            center_w > left_w + 0.20 and center_w > right_w + 0.20,
            details=f"center_w={center_w:.3f}, left_w={left_w:.3f}, right_w={right_w:.3f}",
        )

        # Lites ordered left -> center -> right
        lx = (l_aabb[0][0] + l_aabb[1][0]) / 2.0
        cx = (c_aabb[0][0] + c_aabb[1][0]) / 2.0
        rx = (r_aabb[0][0] + r_aabb[1][0]) / 2.0
        ctx.check(
            "lites ordered left-center-right",
            lx < cx < rx,
            details=f"left_x={lx:.3f}, center_x={cx:.3f}, right_x={rx:.3f}",
        )

        # Frame bottom near z=0
        ctx.check(
            "sill sits near z=0",
            abs(frame_aabb[0][2]) < 0.02,
            details=f"frame zmin={frame_aabb[0][2]:.4f}",
        )

        # Center fixed pane is wider than 1.0m (the picture window)
        ctx.check(
            "center fixed pane exceeds 1.0m width",
            center_w > 1.0,
            details=f"center_w={center_w:.3f}",
        )

        # Latches positioned on the sash (on the meeting stile side)
        tl_aabb = ctx.part_world_aabb(top_latch)
        bl_aabb = ctx.part_world_aabb(bottom_latch)
        ctx.check(
            "top latch near top of sash",
            tl_aabb[1][2] > r_aabb[0][2] + opening_h * 0.6,
            details=f"top_latch zmax={tl_aabb[1][2]:.3f}, sash zmin={r_aabb[0][2]:.3f}",
        )
        ctx.check(
            "bottom latch near bottom of sash",
            bl_aabb[0][2] < r_aabb[0][2] + opening_h * 0.4,
            details=f"bottom_latch zmin={bl_aabb[0][2]:.3f}, sash zmin={r_aabb[0][2]:.3f}",
        )

        # Latches contact/near the sash stile (mounted)
        ctx.expect_contact(
            top_latch, right_sash,
            contact_tol=0.005,
            name="top latch mounted on right sash stile",
        )
        ctx.expect_contact(
            bottom_latch, right_sash,
            contact_tol=0.005,
            name="bottom latch mounted on right sash stile",
        )

        # Adjacent fixed lites share the mullion (projected overlap in XZ plane)
        ctx.expect_overlap(
            left_lite, center_fixed,
            axes="xz",
            min_overlap=0.01,
            name="left and center lites overlap at shared mullion rebate",
        )

        # Roller blocks overlap the frame sill region (seated on track)
        ctx.expect_overlap(
            right_sash, frame,
            axes="x",
            elem_a="roller_left",
            elem_b="frame_shell",
            min_overlap=0.010,
            name="left roller overlaps frame sill in X (seated on track)",
        )

    # --- Sliding sash opens along -X ---
    travel = slide.motion_limits.upper
    with ctx.pose({slide: 0.0}):
        rest_cx = (r_aabb[0][0] + r_aabb[1][0]) / 2.0

    with ctx.pose({slide: travel, top_latch_joint: 0.0, bottom_latch_joint: 0.0}):
        r_open = ctx.part_world_aabb(right_sash)
        open_cx = (r_open[0][0] + r_open[1][0]) / 2.0

        # Sash translates along -X by ~travel distance
        ctx.check(
            "right sash slides along -X by ~travel",
            abs((rest_cx - open_cx) - travel) < 0.02,
            details=f"rest_cx={rest_cx:.3f}, open_cx={open_cx:.3f}, travel={travel:.3f}",
        )

        # Retained within frame X span
        f_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "sash retained within frame X span at full travel",
            r_open[1][0] < f_aabb[1][0] + 1e-4 and r_open[0][0] > f_aabb[0][0] - 1e-4,
            details=f"sash x=[{r_open[0][0]:.3f},{r_open[1][0]:.3f}] frame x=[{f_aabb[0][0]:.3f},{f_aabb[1][0]:.3f}]",
        )

        ctx.expect_overlap(
            right_sash, frame,
            axes="z",
            min_overlap=0.10,
            name="sash retains vertical engagement with head/sill track",
        )

    # --- Latch tilt-in articulation ---
    with ctx.pose({slide: 0.0, top_latch_joint: 0.0, bottom_latch_joint: 0.0}):
        tl_locked = ctx.part_world_aabb(top_latch)
        tl_locked_y = (tl_locked[0][1] + tl_locked[1][1]) / 2.0

    with ctx.pose({slide: 0.0, top_latch_joint: 1.2, bottom_latch_joint: 1.2}):
        tl_tilted = ctx.part_world_aabb(top_latch)
        tl_tilted_y = (tl_tilted[0][1] + tl_tilted[1][1]) / 2.0

        # Top latch Y center shifts when tilted (rotates outward from sash face)
        ctx.check(
            "top latch tilts outward on revolute joint",
            abs(tl_tilted_y - tl_locked_y) > 0.005,
            details=f"locked_y={tl_locked_y:.4f}, tilted_y={tl_tilted_y:.4f}",
        )

        # Tilted latch does not move in Z significantly (rotation about X axis)
        tl_locked_z = (tl_locked[0][2] + tl_locked[1][2]) / 2.0
        tl_tilted_z = (tl_tilted[0][2] + tl_tilted[1][2]) / 2.0
        ctx.check(
            "latch rotation does not significantly shift Z center",
            abs(tl_tilted_z - tl_locked_z) < 0.03,
            details=f"locked_z={tl_locked_z:.4f}, tilted_z={tl_tilted_z:.4f}",
        )

    # --- Roller blocks exist at bottom of sash ---
    with ctx.pose({slide: 0.0, top_latch_joint: 0.0, bottom_latch_joint: 0.0}):
        rs_aabb = ctx.part_world_aabb(right_sash)
        # Rollers are at the bottom of the sash part (included in its AABB)
        ctx.check(
            "right sash bottom includes roller region",
            rs_aabb[0][2] < INNER_Z0 + 0.01,
            details=f"sash zmin={rs_aabb[0][2]:.4f}, inner_z0={INNER_Z0:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
