from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ── Screen constants (unchanged from parent baseline) ──────────────────────
SCREEN_OUTER_W = 0.585
SCREEN_OUTER_H = 0.375
PANEL_W = 0.512
PANEL_H = 0.320
SCREEN_CENTER_Z = 0.105

# ── Desk-clamp arm constants ───────────────────────────────────────────────
DESK_THICKNESS = 0.032
CLAMP_JAW_W = 0.058
CLAMP_JAW_D = 0.062
CLAMP_JAW_T = 0.010
CLAMP_SPINE_T = 0.014
CLAMP_POST_R = 0.016
CLAMP_POST_H = 0.036
SHOULDER_Z = CLAMP_JAW_T + CLAMP_POST_H  # ≈ 0.046

LOWER_ARM_LENGTH = 0.280
LOWER_ARM_W = 0.038
LOWER_ARM_D = 0.026
LOWER_FILLET = 0.005

UPPER_ARM_LENGTH = 0.250
UPPER_ARM_W = 0.036
UPPER_ARM_D = 0.024
UPPER_FILLET = 0.005
# Upper beam starts at the elbow point; overlap with lower housing is intentional
# (beam enters the bearing housing — captured-shaft pattern)
ELBOW_CLEARANCE = 0.000

PIVOT_R = 0.022
PIVOT_W = 0.048  # width of pivot housing along its axis

VESA_PLATE_W = 0.120
VESA_PLATE_H = 0.120
VESA_PLATE_T = 0.010
VESA_PATTERN = 0.100  # VESA 100×100 mm
BRACKET_DEPTH = 0.022


# ── CadQuery geometry helpers ──────────────────────────────────────────────

def _c_clamp_body() -> cq.Workplane:
    """C-clamp frame that grips the desk edge.  Desk surface is at z = 0."""
    top_jaw = (
        cq.Workplane("XY")
        .box(CLAMP_JAW_W, CLAMP_JAW_D, CLAMP_JAW_T)
        .translate((0.0, 0.0, CLAMP_JAW_T / 2.0))
    )
    bot_jaw = (
        cq.Workplane("XY")
        .box(CLAMP_JAW_W, CLAMP_JAW_D, CLAMP_JAW_T)
        .translate((0.0, 0.0, -(DESK_THICKNESS + CLAMP_JAW_T / 2.0)))
    )
    spine_height = DESK_THICKNESS + 2.0 * CLAMP_JAW_T
    spine = (
        cq.Workplane("XY")
        .box(CLAMP_JAW_W, CLAMP_SPINE_T, spine_height)
        .translate((0.0, -(CLAMP_JAW_D / 2.0 - CLAMP_SPINE_T / 2.0), -DESK_THICKNESS / 2.0))
    )
    # Shoulder post on top
    post = (
        cq.Workplane("XY")
        .cylinder(CLAMP_POST_H, CLAMP_POST_R)
        .translate((0.0, 0.0, CLAMP_JAW_T + CLAMP_POST_H / 2.0))
    )
    return top_jaw.union(bot_jaw).union(spine).union(post)


def _clamp_bolt() -> cq.Workplane:
    """Small tightening bolt with hex-head and threaded shank."""
    head_r = 0.008
    head_h = 0.006
    shank_r = 0.004
    shank_h = DESK_THICKNESS + CLAMP_JAW_T + 0.008
    head = cq.Workplane("XY").cylinder(head_h, head_r).translate((0.0, 0.0, -head_h / 2.0))
    shank = cq.Workplane("XY").cylinder(shank_h, shank_r).translate((0.0, 0.0, shank_h / 2.0))
    return head.union(shank)


def _lower_arm_beam() -> cq.Workplane:
    """Lower arm beam along +Z with filleted rectangular cross-section.

    Shortened by 15 mm so the beam top clears the upper arm crossing at
    the elbow; the elbow housing visually bridges the gap.
    """
    beam_len = LOWER_ARM_LENGTH - 0.015
    seg = (
        cq.Workplane("XY")
        .box(LOWER_ARM_W, LOWER_ARM_D, beam_len)
        .translate((0.0, 0.0, beam_len / 2.0))
    )
    try:
        seg = seg.edges("|Z").fillet(LOWER_FILLET)
    except Exception:
        pass
    return seg


def _upper_arm_beam() -> cq.Workplane:
    """Upper arm beam along -Y with filleted rectangular cross-section.

    Starts at y = -ELBOW_CLEARANCE to clear the lower elbow housing.
    Ends at y = -UPPER_ARM_LENGTH.
    """
    beam_len = UPPER_ARM_LENGTH - ELBOW_CLEARANCE
    seg = (
        cq.Workplane("XY")
        .box(UPPER_ARM_W, beam_len, UPPER_ARM_D)
        .translate((0.0, -(ELBOW_CLEARANCE + beam_len / 2.0), 0.0))
    )
    try:
        seg = seg.edges("|Y").fillet(UPPER_FILLET)
    except Exception:
        pass
    return seg


def _vesa_plate_solid() -> cq.Workplane:
    """Flat VESA plate with four mounting holes (100×100 pattern).

    Plate is from y=0 to y=VESA_PLATE_T; holes go through along Y.
    """
    plate = (
        cq.Workplane("XY")
        .box(VESA_PLATE_W, VESA_PLATE_T, VESA_PLATE_H)
        .translate((0.0, VESA_PLATE_T / 2.0, 0.0))
    )
    hole_r = 0.003
    half = VESA_PATTERN / 2.0
    for dx, dz in [(-half, -half), (-half, half), (half, -half), (half, half)]:
        hole = (
            cq.Workplane("XY")
            .cylinder(VESA_PLATE_T + 0.004, hole_r)
            .rotate((0, 0, 0), (1, 0, 0), 90)
            .translate((dx, VESA_PLATE_T / 2.0, dz))
        )
        plate = plate.cut(hole)
    return plate


def _vesa_bolt() -> cq.Workplane:
    """Small VESA mounting bolt (button-head style)."""
    head_r = 0.005
    head_h = 0.004
    shank_r = 0.0025
    shank_h = 0.010
    head = cq.Workplane("XY").cylinder(head_h, head_r)
    shank = (
        cq.Workplane("XY")
        .cylinder(shank_h, shank_r)
        .translate((0.0, 0.0, -(shank_h / 2.0 + head_h / 2.0)))
    )
    return head.union(shank)


# ── Model ──────────────────────────────────────────────────────────────────

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widescreen_lcd_monitor_arm")

    # Materials
    matte_black = model.material("matte_black", rgba=(0.006, 0.007, 0.007, 1.0))
    soft_black = model.material("soft_black", rgba=(0.018, 0.020, 0.020, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.060, 0.062, 0.060, 1.0))
    lcd_white = model.material("lcd_white", rgba=(0.88, 0.92, 0.95, 1.0))
    arm_grey = model.material("arm_grey", rgba=(0.22, 0.22, 0.22, 1.0))
    bolt_silver = model.material("bolt_silver", rgba=(0.55, 0.56, 0.54, 1.0))

    # ────────────────────────────────────────────────────────────────────
    # desk_clamp  (root part — C-clamp body + tightening bolts)
    # ────────────────────────────────────────────────────────────────────
    desk_clamp = model.part("desk_clamp")
    desk_clamp.visual(
        mesh_from_cadquery(_c_clamp_body(), "clamp_body"),
        material=matte_black,
        name="clamp_body",
    )

    # Two tightening bolts at the bottom of the clamp (for-loop)
    bolt_z = -(DESK_THICKNESS + CLAMP_JAW_T + 0.004)
    for i, dx in enumerate((-0.014, 0.014)):
        desk_clamp.visual(
            mesh_from_cadquery(_clamp_bolt(), f"clamp_bolt_{i}"),
            origin=Origin(xyz=(dx, 0.0, bolt_z)),
            material=bolt_silver,
            name=f"clamp_bolt_{i}",
        )

    # ────────────────────────────────────────────────────────────────────
    # arm_lower  (lower arm segment — shoulder to elbow)
    # ────────────────────────────────────────────────────────────────────
    arm_lower = model.part("arm_lower")

    # Main beam along +Z in arm_lower local frame
    arm_lower.visual(
        mesh_from_cadquery(_lower_arm_beam(), "lower_beam"),
        material=arm_grey,
        name="lower_beam",
    )
    # Shoulder pivot housing (cylinder around Z at base)
    arm_lower.visual(
        Cylinder(radius=PIVOT_R, length=PIVOT_W),
        origin=Origin(xyz=(0.0, 0.0, PIVOT_W / 2.0)),
        material=dark_graphite,
        name="lower_shoulder_housing",
    )
    # Elbow pivot housing (cylinder around X at top of lower arm)
    arm_lower.visual(
        Cylinder(radius=PIVOT_R, length=PIVOT_W),
        origin=Origin(xyz=(0.0, 0.0, LOWER_ARM_LENGTH), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_graphite,
        name="lower_elbow_housing",
    )

    # ────────────────────────────────────────────────────────────────────
    # arm_upper  (upper arm segment — elbow to VESA head, extends in -Y)
    # ────────────────────────────────────────────────────────────────────
    arm_upper = model.part("arm_upper")

    # Main beam along -Y in arm_upper local frame (toward user)
    arm_upper.visual(
        mesh_from_cadquery(_upper_arm_beam(), "upper_beam"),
        material=arm_grey,
        name="upper_beam",
    )
    # Adapter block bridging beam end to VESA head bracket
    adapter_len = 0.008
    arm_upper.visual(
        Box((UPPER_ARM_W + 0.004, adapter_len, UPPER_ARM_D + 0.004)),
        origin=Origin(xyz=(0.0, -(UPPER_ARM_LENGTH - adapter_len / 2.0), 0.0)),
        material=dark_graphite,
        name="upper_vesa_adapter",
    )

    # ────────────────────────────────────────────────────────────────────
    # vesa_head  (VESA plate + bracket, extends in -Y toward monitor)
    # ────────────────────────────────────────────────────────────────────
    vesa_head = model.part("vesa_head")

    # Connecting bracket from arm end toward the monitor (in -Y)
    vesa_head.visual(
        Box((UPPER_ARM_W + 0.006, BRACKET_DEPTH, VESA_PLATE_H * 0.55)),
        origin=Origin(xyz=(0.0, -BRACKET_DEPTH / 2.0, 0.0)),
        material=matte_black,
        name="vesa_bracket",
    )
    # VESA plate behind bracket (further in -Y, with mounting holes)
    vesa_head.visual(
        mesh_from_cadquery(_vesa_plate_solid(), "vesa_plate"),
        origin=Origin(xyz=(0.0, -(BRACKET_DEPTH + VESA_PLATE_T), 0.0)),
        material=arm_grey,
        name="vesa_plate",
    )

    # VESA mounting bolts (4 in a 100×100 pattern) — for-loop
    half = VESA_PATTERN / 2.0
    bolt_positions = [(-half, -half), (-half, half), (half, -half), (half, half)]
    for i, (dx, dz) in enumerate(bolt_positions):
        vesa_head.visual(
            mesh_from_cadquery(_vesa_bolt(), f"vesa_bolt_{i}"),
            origin=Origin(
                xyz=(dx, -BRACKET_DEPTH + 0.001, dz),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=bolt_silver,
            name=f"vesa_bolt_{i}",
        )

    # ────────────────────────────────────────────────────────────────────
    # screen  (identical to parent baseline)
    # ────────────────────────────────────────────────────────────────────
    screen = model.part("screen")
    screen.visual(
        Box((0.568, 0.034, 0.360)),
        origin=Origin(xyz=(0.0, -0.047, SCREEN_CENTER_Z)),
        material=soft_black,
        name="rear_shell",
    )
    screen.visual(
        mesh_from_geometry(
            BezelGeometry(
                (PANEL_W, PANEL_H),
                (SCREEN_OUTER_W, SCREEN_OUTER_H),
                0.020,
                opening_shape="rounded_rect",
                outer_shape="rounded_rect",
                opening_corner_radius=0.004,
                outer_corner_radius=0.018,
            ),
            "screen_bezel",
        ),
        origin=Origin(xyz=(0.0, -0.067, SCREEN_CENTER_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="bezel",
    )
    screen.visual(
        Box((PANEL_W, 0.003, PANEL_H)),
        origin=Origin(xyz=(0.0, -0.076, SCREEN_CENTER_Z)),
        material=lcd_white,
        name="display_panel",
    )
    screen.visual(
        Box((0.074, 0.042, 0.100)),
        origin=Origin(xyz=(0.0, -0.030, 0.0)),
        material=dark_graphite,
        name="rear_mount",
    )
    screen.visual(
        Cylinder(radius=0.014, length=0.086),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="hinge_barrel",
    )

    # ────────────────────────────────────────────────────────────────────
    # Articulations
    # ────────────────────────────────────────────────────────────────────

    # 1. Shoulder: desk_clamp → arm_lower  (revolute about Z)
    model.articulation(
        "clamp_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=desk_clamp,
        child=arm_lower,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.0, lower=-2.60, upper=2.60),
    )

    # 2. Elbow: arm_lower → arm_upper  (revolute about X)
    #    Positive q raises the upper arm (rotates -Y toward +Z by right-hand rule about +X:
    #    Rx(+) takes -Y toward -Z, so with arm going -Y, positive q actually lowers.
    #    To make positive q raise: axis=(−1, 0, 0) or equivalently flip limits.
    #    Actually: right-hand rule about +X: +Y→+Z, -Y→-Z.
    #    So positive q takes the -Y arm toward -Z (downward). To raise, use axis=(-1,0,0).
    model.articulation(
        "lower_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=arm_lower,
        child=arm_upper,
        origin=Origin(xyz=(0.0, 0.0, LOWER_ARM_LENGTH)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=1.0, lower=-0.30, upper=1.80),
    )

    # 3. Upper arm to VESA head  (fixed — rigid connection)
    model.articulation(
        "upper_to_vesa",
        ArticulationType.FIXED,
        parent=arm_upper,
        child=vesa_head,
        origin=Origin(xyz=(0.0, -UPPER_ARM_LENGTH, 0.0)),
    )

    # 4. Panel tilt: vesa_head → screen  (revolute about X)
    #    Screen hinge_barrel sits at the VESA bracket/plate front face.
    #    The plate front face is at y = -(BRACKET_DEPTH + VESA_PLATE_T) in vesa_head frame.
    model.articulation(
        "vesa_to_screen",
        ArticulationType.REVOLUTE,
        parent=vesa_head,
        child=screen,
        origin=Origin(
            xyz=(0.0, -(BRACKET_DEPTH + VESA_PLATE_T), 0.0),
            rpy=(-0.12, 0.0, 0.0),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=9.0, velocity=1.5, lower=-0.22, upper=0.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    desk_clamp = object_model.get_part("desk_clamp")
    arm_lower = object_model.get_part("arm_lower")
    arm_upper = object_model.get_part("arm_upper")
    vesa_head = object_model.get_part("vesa_head")
    screen = object_model.get_part("screen")

    shoulder = object_model.get_articulation("clamp_to_lower_arm")
    elbow = object_model.get_articulation("lower_to_upper_arm")
    tilt = object_model.get_articulation("vesa_to_screen")

    # ── Structural identity checks ─────────────────────────────────────
    ctx.check(
        "desk_clamp is root part",
        desk_clamp in object_model.root_parts(),
        details="desk_clamp should be the single root part",
    )
    ctx.check(
        "arm_lower has lower_beam visual",
        arm_lower.get_visual("lower_beam") is not None,
        details="arm_lower must have the lower_beam arm segment",
    )
    ctx.check(
        "arm_upper has upper_beam visual",
        arm_upper.get_visual("upper_beam") is not None,
        details="arm_upper must have the upper_beam arm segment",
    )
    ctx.check(
        "vesa_head has vesa_plate visual",
        vesa_head.get_visual("vesa_plate") is not None,
        details="vesa_head must have the VESA mounting plate",
    )

    # ── Joint type and axis checks ──────────────────────────────────────
    ctx.check(
        "shoulder joint is revolute about Z",
        shoulder.articulation_type == ArticulationType.REVOLUTE
        and shoulder.axis is not None
        and abs(shoulder.axis[2]) > 0.9,
        details=f"shoulder axis={shoulder.axis}",
    )
    ctx.check(
        "elbow joint is revolute about horizontal X",
        elbow.articulation_type == ArticulationType.REVOLUTE
        and elbow.axis is not None
        and abs(elbow.axis[0]) > 0.9,
        details=f"elbow axis={elbow.axis}",
    )
    ctx.check(
        "panel tilt joint is revolute",
        tilt.articulation_type == ArticulationType.REVOLUTE,
        details="vesa_to_screen must be revolute for panel tilt",
    )

    # ── Screen sits above desk level ────────────────────────────────────
    ctx.expect_gap(
        screen,
        desk_clamp,
        axis="z",
        min_gap=0.10,
        positive_elem="display_panel",
        negative_elem="clamp_body",
        name="display sits well above the desk clamp",
    )

    # ── Panel is 16:10 widescreen ───────────────────────────────────────
    ctx.check(
        "panel is 16:10 widescreen",
        abs((PANEL_W / PANEL_H) - 1.6) < 0.02,
        details=f"panel ratio={PANEL_W / PANEL_H:.3f}",
    )

    # ── Shoulder rotation moves the arm assembly ────────────────────────
    with ctx.pose({shoulder: 0.0}):
        rest_pos = ctx.part_world_position(screen)
    with ctx.pose({shoulder: 1.0}):
        rotated_pos = ctx.part_world_position(screen)

    def _dist(a, b):
        if a is None or b is None:
            return 0.0
        return math.sqrt(sum((ai - bi) ** 2 for ai, bi in zip(a, b)))

    ctx.check(
        "shoulder rotation moves the screen laterally",
        _dist(rest_pos, rotated_pos) > 0.05,
        details=f"rest={rest_pos}, rotated={rotated_pos}",
    )

    # ── Elbow raises the screen ─────────────────────────────────────────
    with ctx.pose({elbow: 0.0}):
        elbow_low = ctx.part_world_position(screen)
    with ctx.pose({elbow: 1.0}):
        elbow_high = ctx.part_world_position(screen)

    ctx.check(
        "elbow positive pose raises screen",
        elbow_low is not None
        and elbow_high is not None
        and elbow_high[2] > elbow_low[2] + 0.02,
        details=f"low={elbow_low}, high={elbow_high}",
    )

    # ── Tilt joint moves screen forward/back ────────────────────────────
    with ctx.pose({tilt: -0.20}):
        back_aabb = ctx.part_element_world_aabb(screen, elem="display_panel")
    with ctx.pose({tilt: 0.18}):
        forward_aabb = ctx.part_element_world_aabb(screen, elem="display_panel")

    def _center_y(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return (lo[1] + hi[1]) / 2.0

    back_y = _center_y(back_aabb)
    forward_y = _center_y(forward_aabb)
    ctx.check(
        "tilt joint moves screen forward and back",
        back_y is not None and forward_y is not None and forward_y < back_y - 0.025,
        details=f"back_y={back_y}, forward_y={forward_y}",
    )

    # ── Intentional overlap: beam enters elbow bearing housing ──────────
    ctx.allow_overlap(
        arm_lower,
        arm_upper,
        elem_a="lower_elbow_housing",
        elem_b="upper_beam",
        reason="The upper arm beam enters the lower arm elbow bearing housing as a captured-shaft pivot.",
    )
    ctx.expect_overlap(
        arm_lower,
        arm_upper,
        axes="z",
        elem_a="lower_elbow_housing",
        elem_b="upper_beam",
        min_overlap=0.010,
        name="upper beam is nested inside the elbow housing along Z",
    )

    # ── Intentional overlap: hinge_barrel seated at VESA bracket/plate ──
    ctx.allow_overlap(
        vesa_head,
        screen,
        elem_a="vesa_bracket",
        elem_b="hinge_barrel",
        reason="The hinge_barrel pivot is intentionally seated against the vesa bracket as the tilt interface.",
    )
    ctx.allow_overlap(
        vesa_head,
        screen,
        elem_a="vesa_plate",
        elem_b="hinge_barrel",
        reason="The hinge_barrel pivot partly nests into the VESA plate as the tilt mechanism.",
    )
    ctx.expect_contact(
        vesa_head,
        screen,
        elem_a="vesa_bracket",
        elem_b="hinge_barrel",
        contact_tol=0.020,
        name="hinge_barrel is near the vesa bracket (tilt interface)",
    )

    return ctx.report()


object_model = build_object_model()
