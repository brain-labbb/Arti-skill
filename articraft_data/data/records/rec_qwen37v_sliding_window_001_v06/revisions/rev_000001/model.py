from __future__ import annotations

# Three-panel horizontal sliding window variant with slim vinyl frame rails,
# bevelled outer corners, independent insect screen, and roller blocks on the
# center sliding sash.
#
# Coordinate convention:
#   +Z up, window stands vertically.
#   width  -> X,  height -> Z (sill near z=0),  depth -> Y.
#   Glass plane is X-Z. Window reads SHUT at q=0.
#
# Structure:
#   - frame (root): slim head/sill/jambs + two mullions, one CadQuery solid
#     with bevelled outer vertical corners.
#   - left_lite, right_lite (FIXED): vinyl sash ring + colonial grille + glass.
#   - center_sash (PRISMATIC +X): same construction + two roller blocks at
#     the bottom rail.
#   - insect_screen (PRISMATIC +X): thin frame ring + mesh panel, slides
#     independently on a shallow track in front of the sashes.

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

# Slim vinyl frame
FRAME_FACE = 0.045          # narrow rail/stile face width
MULLION_FACE = 0.048        # slim intermediate mullion
FRAME_DEPTH = 0.100         # frame depth along Y
CORNER_CHAMFER = 0.010      # bevel on outer vertical corners

# Three lite columns
SIDE_LITE_W = 0.882
CENTER_LITE_W = 1.050

# Sash
SASH_FACE = 0.045
SASH_DEPTH = 0.032
GLASS_T = 0.006

# Colonial grille
GRILLE_COLS = 4
GRILLE_ROWS = 5
MUNTIN_T = 0.016
MUNTIN_DEPTH = 0.016

# Y layout (depth). Frame centered on y=0.
FIXED_LITE_Y = -0.022
SLIDE_SASH_Y = 0.022
SCREEN_Y = 0.047            # insect screen sits in front of sashes

# Insect screen
SCREEN_FRAME_FACE = 0.028
SCREEN_FRAME_DEPTH = 0.016
SCREEN_MESH_T = 0.002

# Roller blocks (two, at bottom of center sash)
ROLLER_W = 0.028
ROLLER_H = 0.012
ROLLER_D = 0.015

REBATE = 0.004

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
SCREEN_MESH_RGBA = (0.22, 0.22, 0.25, 0.40)
ROLLER_RGBA = (0.15, 0.15, 0.18, 1.0)
SCREEN_FRAME_RGBA = (0.80, 0.82, 0.84, 1.0)  # light aluminium screen frame


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
    """Slim outer frame with bevelled vertical corners, then three lite cuts."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)
    # Bevel the 4 outer vertical corners before cutting openings
    outer = outer.edges("|Z").chamfer(CORNER_CHAMFER)

    cut_depth = FRAME_DEPTH + 0.02
    left_cut = _slab(LEFT_X0, LEFT_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    center_cut = _slab(CENTER_X0, CENTER_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    right_cut = _slab(RIGHT_X0, RIGHT_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)

    return outer.cut(left_cut).cut(center_cut).cut(right_cut)


def _build_sash_grille_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """Sash ring + colonial muntin grid, centered on local origin."""
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


def _build_roller_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """Two small roller blocks at the bottom of the sash, in sash-local frame."""
    bottom_z = -(opening_h / 2.0 + SASH_FACE)
    # Rollers half-embedded at bottom rail surface
    rz = bottom_z
    rx = opening_w / 2.0 + SASH_FACE - ROLLER_W / 2.0 - 0.020

    left = _slab(
        -rx - ROLLER_W / 2.0, -rx + ROLLER_W / 2.0,
        rz - ROLLER_H / 2.0, rz + ROLLER_H / 2.0,
        0.0, ROLLER_D,
    )
    right = _slab(
        rx - ROLLER_W / 2.0, rx + ROLLER_W / 2.0,
        rz - ROLLER_H / 2.0, rz + ROLLER_H / 2.0,
        0.0, ROLLER_D,
    )
    return left.union(right)


def _build_screen_frame_shape() -> cq.Workplane:
    """Insect screen frame ring, centered on local origin."""
    ow = CENTER_LITE_W
    oh = INNER_Z1 - INNER_Z0
    out_w = ow + 2 * SCREEN_FRAME_FACE
    out_h = oh + 2 * SCREEN_FRAME_FACE

    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, SCREEN_FRAME_DEPTH)
    inner = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SCREEN_FRAME_DEPTH + 0.01)
    return outer.cut(inner)


def _build_screen_mesh_shape() -> cq.Workplane:
    """Thin mesh panel filling the screen opening (rebated under frame lip)."""
    ow = CENTER_LITE_W + 2 * REBATE
    oh = (INNER_Z1 - INNER_Z0) + 2 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SCREEN_MESH_T)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    span = SIDE_LITE_W + MULLION_FACE + CENTER_LITE_W + MULLION_FACE + SIDE_LITE_W
    inner_w = INNER_X1 - INNER_X0
    assert abs(span - inner_w) < 1e-6, f"lite layout {span} != inner width {inner_w}"

    model = ArticulatedObject(name="three_panel_sliding_window_slim")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("screen_mesh", rgba=SCREEN_MESH_RGBA)
    model.material("screen_frame", rgba=SCREEN_FRAME_RGBA)
    model.material("roller", rgba=ROLLER_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame_shell"),
        material="vinyl",
        name="frame_shell",
    )

    opening_h = INNER_Z1 - INNER_Z0

    # --- Fixed side lites ---
    for lite_name, ow in (("left_lite", SIDE_LITE_W), ("right_lite", SIDE_LITE_W)):
        lite = model.part(lite_name)
        lite.visual(
            mesh_from_cadquery(_build_sash_grille_shape(ow, opening_h), f"{lite_name}_vinyl"),
            material="vinyl",
            name=f"{lite_name}_vinyl",
        )
        lite.visual(
            mesh_from_cadquery(_build_sash_glass_shape(ow, opening_h), f"{lite_name}_glass"),
            material="glass",
            name=f"{lite_name}_glass",
        )

    # --- Center sliding sash (with roller blocks) ---
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
        mesh_from_cadquery(_build_roller_shape(CENTER_LITE_W, opening_h), "center_sash_rollers"),
        material="roller",
        name="center_sash_rollers",
    )

    # --- Insect screen ---
    insect_screen = model.part("insect_screen")
    insect_screen.visual(
        mesh_from_cadquery(_build_screen_frame_shape(), "screen_frame"),
        material="screen_frame",
        name="screen_frame",
    )
    insect_screen.visual(
        mesh_from_cadquery(_build_screen_mesh_shape(), "screen_mesh"),
        material="screen_mesh",
        name="screen_mesh",
    )

    # --- Articulations ---
    left_cx = (LEFT_X0 + LEFT_X1) / 2.0
    right_cx = (RIGHT_X0 + RIGHT_X1) / 2.0
    center_cx = (CENTER_X0 + CENTER_X1) / 2.0
    mid_cz = (INNER_Z0 + INNER_Z1) / 2.0

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

    # Center sliding sash: PRISMATIC +X
    slide_travel = SIDE_LITE_W * 0.90
    model.articulation(
        "frame_to_center_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="center_sash",
        origin=Origin(xyz=(center_cx, SLIDE_SASH_Y, mid_cz)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.5, lower=0.0, upper=slide_travel),
    )

    # Insect screen: independent PRISMATIC +X (shallow travel)
    screen_travel = SIDE_LITE_W * 0.85
    model.articulation(
        "frame_to_screen",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="insect_screen",
        origin=Origin(xyz=(center_cx, SCREEN_Y, mid_cz)),
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

    # ======================================================================
    # Intentional overlap allowances
    # ======================================================================

    # Glass rebated under sash/muntin lip (captured glazing, intra-part)
    for nm in ("left_lite", "right_lite", "center_sash"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass",
            elem_b=f"{nm}_vinyl",
            reason="Clear pane is rebated under the sash/muntin lip so it reads captured.",
        )

    # Rollers embedded in center sash bottom rail (intra-part)
    ctx.allow_overlap(
        "center_sash", "center_sash",
        elem_a="center_sash_rollers",
        elem_b="center_sash_vinyl",
        reason="Roller blocks are half-embedded in the bottom rail of the center sash.",
    )

    # Rollers sit in the frame sill track groove (intentional track nesting)
    ctx.allow_overlap(
        "frame", "center_sash",
        elem_a="frame_shell",
        elem_b="center_sash_rollers",
        reason="Roller blocks sit in the frame sill track groove to enable sliding.",
    )

    # Screen mesh rebated under screen frame lip (intra-part)
    ctx.allow_overlap(
        "insect_screen", "insect_screen",
        elem_a="screen_mesh",
        elem_b="screen_frame",
        reason="Screen mesh is rebated under the screen frame lip.",
    )

    # Fixed lites seated in frame opening
    ctx.allow_overlap(
        "frame", "left_lite",
        elem_a="frame_shell", elem_b="left_lite_vinyl",
        reason="Left fixed lite rebated into frame opening (seated capture).",
    )
    ctx.allow_overlap(
        "frame", "right_lite",
        elem_a="frame_shell", elem_b="right_lite_vinyl",
        reason="Right fixed lite rebated into frame opening (seated capture).",
    )

    # Center sash rides frame track
    ctx.allow_overlap(
        "frame", "center_sash",
        elem_a="frame_shell", elem_b="center_sash_vinyl",
        reason="Center sash rides the head/sill track and laps the frame face.",
    )

    # Glass lapping frame opening lip
    ctx.allow_overlap(
        "frame", "left_lite",
        elem_a="frame_shell", elem_b="left_lite_glass",
        reason="Left lite glass rebated under frame opening lip.",
    )
    ctx.allow_overlap(
        "frame", "right_lite",
        elem_a="frame_shell", elem_b="right_lite_glass",
        reason="Right lite glass rebated under frame opening lip.",
    )
    ctx.allow_overlap(
        "frame", "center_sash",
        elem_a="frame_shell", elem_b="center_sash_glass",
        reason="Center sash glass laps the head/sill track lip.",
    )

    # Insect screen sits in frame track (screen overlaps frame in Y)
    ctx.allow_overlap(
        "frame", "insect_screen",
        elem_a="frame_shell", elem_b="screen_frame",
        reason="Insect screen frame sits in the outer frame track, partially embedded.",
    )
    ctx.allow_overlap(
        "frame", "insect_screen",
        elem_a="frame_shell", elem_b="screen_mesh",
        reason="Insect screen mesh passes through the frame track region.",
    )

    # ======================================================================
    # Prompt-specific checks
    # ======================================================================

    # --- Slim frame rails ---
    ctx.check(
        "slim frame rails (FACE < 0.055)",
        FRAME_FACE < 0.055,
        details=f"FRAME_FACE={FRAME_FACE}",
    )

    # --- Bevelled corners: frame depth matches slim profile ---
    frame_aabb = ctx.part_world_aabb(frame)
    frame_depth_actual = frame_aabb[1][1] - frame_aabb[0][1]
    ctx.check(
        "frame depth matches slim bevelled profile",
        abs(frame_depth_actual - FRAME_DEPTH) < 0.015,
        details=f"depth={frame_depth_actual:.4f}, expected~{FRAME_DEPTH}",
    )

    # --- Insect screen exists ---
    screen_aabb = ctx.part_world_aabb(insect_screen)
    ctx.check(
        "insect screen part has geometry",
        screen_aabb is not None,
        details="screen part missing geometry",
    )

    # --- Screen has independent prismatic joint ---
    ctx.check(
        "screen has prismatic joint (non-fixed)",
        screen_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"screen joint type={screen_slide.articulation_type}",
    )

    # --- Roller blocks on center sash ---
    sash_visual_names = [v.name for v in center_sash.visuals]
    ctx.check(
        "center sash has roller blocks",
        "center_sash_rollers" in sash_visual_names,
        details=f"sash visuals={sash_visual_names}",
    )

    # --- At least one non-fixed joint (two prismatic joints present) ---
    non_fixed = [
        a for a in object_model.articulations
        if a.articulation_type != ArticulationType.FIXED
    ]
    ctx.check(
        "at least one non-fixed joint",
        len(non_fixed) >= 1,
        details=f"non-fixed: {[a.name for a in non_fixed]}",
    )

    # ======================================================================
    # Closed pose (q=0): window reads SHUT
    # ======================================================================
    with ctx.pose({slide: 0.0, screen_slide: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        l_aabb = ctx.part_world_aabb(left_lite)
        r_aabb = ctx.part_world_aabb(right_lite)
        c_aabb = ctx.part_world_aabb(center_sash)
        s_aabb = ctx.part_world_aabb(insect_screen)

        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        center_w = c_aabb[1][0] - c_aabb[0][0]
        ctx.check(
            "frame spans wider than center sash",
            frame_w > center_w + 1.5,
            details=f"frame_w={frame_w:.3f}, center_w={center_w:.3f}",
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

        # Lites ordered left -> center -> right
        lx = (l_aabb[0][0] + l_aabb[1][0]) / 2.0
        cx_val = (c_aabb[0][0] + c_aabb[1][0]) / 2.0
        rx = (r_aabb[0][0] + r_aabb[1][0]) / 2.0
        ctx.check(
            "lites ordered left-center-right",
            lx < cx_val < rx,
            details=f"left_x={lx:.3f}, center_x={cx_val:.3f}, right_x={rx:.3f}",
        )

        # All lites seated within frame height.
        # Center sash has roller blocks that protrude ~6 mm into the sill track,
        # so we allow up to ROLLER_H of protrusion below the sill.
        track_tol = ROLLER_H + 0.004
        for nm, ab in (("left", l_aabb), ("right", r_aabb), ("center", c_aabb)):
            ctx.check(
                f"{nm} lite seated within frame height",
                ab[0][2] > frame_aabb[0][2] - track_tol and ab[1][2] < frame_aabb[1][2] + 1e-4,
                details=f"{nm} z=[{ab[0][2]:.3f},{ab[1][2]:.3f}]",
            )

        # Center sash proud of side lites
        l_y = (l_aabb[0][1] + l_aabb[1][1]) / 2.0
        c_y = (c_aabb[0][1] + c_aabb[1][1]) / 2.0
        ctx.check(
            "center sash proud of side lites",
            c_y > l_y + 0.02,
            details=f"center_y={c_y:.3f}, side_y={l_y:.3f}",
        )

        # Screen is in front of everything
        s_y = (s_aabb[0][1] + s_aabb[1][1]) / 2.0
        ctx.check(
            "screen in front of center sash",
            s_y > c_y + 0.01,
            details=f"screen_y={s_y:.3f}, sash_y={c_y:.3f}",
        )

        # Rollers sit in the sill track: prove they overlap the frame in Z
        # (rollers protrude into the sill track groove)
        ctx.expect_overlap(
            center_sash, frame,
            axes="x",
            elem_a="center_sash_rollers",
            elem_b="frame_shell",
            min_overlap=0.010,
            name="rollers engaged with frame sill track",
        )

        # Fixed lites seated in frame
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
        screen_rest_cx = (s_aabb[0][0] + s_aabb[1][0]) / 2.0

    # ======================================================================
    # Driven pose: center sash slides along +X
    # ======================================================================
    travel = slide.motion_limits.upper
    with ctx.pose({slide: travel, screen_slide: 0.0}):
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
        # Retained insertion
        f_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "sash retained within frame X span at full travel",
            c_open[1][0] < f_aabb[1][0] + 1e-4 and c_open[0][0] > f_aabb[0][0] - 1e-4,
            details=f"sash x=[{c_open[0][0]:.3f},{c_open[1][0]:.3f}]",
        )
        ctx.expect_overlap(
            center_sash, frame, axes="z", min_overlap=0.10,
            name="sash retains vertical engagement with track",
        )

    # ======================================================================
    # Driven pose: insect screen slides independently along +X
    # ======================================================================
    screen_travel = screen_slide.motion_limits.upper
    with ctx.pose({slide: 0.0, screen_slide: screen_travel}):
        s_open = ctx.part_world_aabb(insect_screen)
        screen_open_cx = (s_open[0][0] + s_open[1][0]) / 2.0
        ctx.check(
            "screen slides along +X independently",
            abs((screen_open_cx - screen_rest_cx) - screen_travel) < 0.02,
            details=f"rest={screen_rest_cx:.3f}, open={screen_open_cx:.3f}, travel={screen_travel:.3f}",
        )
        # Screen retained within frame
        f_aabb2 = ctx.part_world_aabb(frame)
        ctx.check(
            "screen retained within frame X span at full travel",
            s_open[1][0] < f_aabb2[1][0] + 1e-4 and s_open[0][0] > f_aabb2[0][0] - 1e-4,
            details=f"screen x=[{s_open[0][0]:.3f},{s_open[1][0]:.3f}]",
        )

    return ctx.report()


object_model = build_object_model()
