from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Articraft brief
# - Object: single-toggle jaw ore crusher, ~2.4 m long x 2.3 m wide (over the
#   flywheels) x 2.5 m tall. Blue painted steel frame, manganese-gray jaw
#   plates, cream flywheels with lightening holes.
# - Root/support: welded frame (two bolted side plates + front wall carrying
#   the fixed jaw + rear wall + base rails + top beam + shaft/bearings).
# - Parts: frame (root), swing_jaw (pitman + corrugated moving jaw plate +
#   toggle plate), flywheel_0 / flywheel_1 on the eccentric shaft ends.
# - Articulations:
#   * jaw_swing: REVOLUTE, frame -> swing_jaw, about the eccentric shaft axis
#     (Y). Positive q swings the jaw bottom toward the fixed jaw (+X).
#     Small realistic stroke: -0.015 .. 0.04 rad.
#   * flywheel_0_spin: CONTINUOUS, frame -> flywheel_0 about the shaft axis.
#   * flywheel_1_spin: CONTINUOUS, mimics flywheel_0_spin (same shaft).
# - Intentional overlaps: toggle plate seated into the frame toggle seat.
#   The pitman hub and flywheel hubs are bored with real clearance over the
#   shaft, so no allowance is needed there.
# - Tests: wedge chamber gap between jaw plates, positive q closes the gap,
#   toggle plate seated in seat, flywheels ride the shaft, jaw stays between
#   the side plates.
# ---------------------------------------------------------------------------

SHAFT_X = -0.15
SHAFT_Z = 2.05
SHAFT_RADIUS = 0.09
SHAFT_HALF_LEN = 1.15

FIXED_JAW_PITCH = 0.324  # top of fixed jaw leans forward (+X)
SWING_JAW_PITCH = -0.085  # bottom of moving jaw face leans forward (+X)


def _side_plate_shape() -> cq.Workplane:
    """Heavy side plate silhouette in the XZ plane, extruded symmetric in Y."""
    pts = [
        (-1.15, 0.10),
        (1.15, 0.10),
        (1.15, 1.15),
        (0.95, 2.30),
        (0.05, 2.42),
        (-0.55, 2.42),
        (-1.15, 1.35),
    ]
    return (
        cq.Workplane("XZ")
        .polyline(pts)
        .close()
        .extrude(0.04, both=True)
    )


def _flywheel_shape() -> cq.Workplane:
    """Cream flywheel: heavy rim, web with 3 lightening holes, bored hub."""
    rim = (
        cq.Workplane("XY")
        .circle(0.60)
        .circle(0.44)
        .extrude(0.20)
    )
    web = (
        cq.Workplane("XY")
        .workplane(offset=0.04)
        .circle(0.46)
        .extrude(0.12)
    )
    hub = (
        cq.Workplane("XY")
        .workplane(offset=-0.05)
        .circle(0.15)
        .extrude(0.30)
    )
    wheel = rim.union(web).union(hub)
    for k in range(3):
        ang = 2.0 * math.pi * k / 3.0
        hole = (
            cq.Workplane("XY")
            .workplane(offset=-0.10)
            .center(0.28 * math.cos(ang), 0.28 * math.sin(ang))
            .circle(0.11)
            .extrude(0.40)
        )
        wheel = wheel.cut(hole)
    # Press-fit bore: slightly smaller than the shaft so the hub is captured.
    bore = (
        cq.Workplane("XY")
        .workplane(offset=-0.10)
        .circle(0.080)
        .extrude(0.40)
    )
    return wheel.cut(bore)


def _pitman_hub_shape() -> cq.Workplane:
    """Bored pitman hub ring that rides the eccentric shaft with clearance."""
    return (
        cq.Workplane("XZ")
        .circle(0.17)
        .circle(0.095)
        .extrude(0.50, both=True)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="jaw_ore_crusher")

    steel_blue = model.material("steel_blue", rgba=(0.16, 0.27, 0.48, 1.0))
    deep_blue = model.material("deep_blue", rgba=(0.10, 0.17, 0.32, 1.0))
    cream = model.material("cream_paint", rgba=(0.93, 0.91, 0.85, 1.0))
    manganese = model.material("manganese_steel", rgba=(0.52, 0.53, 0.55, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.23, 0.25, 1.0))

    # ------------------------------------------------------------------ frame
    frame = model.part("frame")

    plate_mesh = mesh_from_cadquery(_side_plate_shape(), "side_plate")
    for i, y in enumerate((0.63, -0.63)):
        frame.visual(
            plate_mesh,
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=steel_blue,
            name=f"side_plate_{i}",
        )

    # Bolt heads scattered over both side plate outer faces.
    bolt_spots = [
        (1.0, 0.35),
        (1.0, 0.9),
        (0.98, 1.45),
        (0.88, 2.0),
        (0.35, 2.25),
        (-0.4, 2.25),
        (-0.95, 1.35),
        (-1.0, 0.7),
        (-0.5, 0.3),
        (0.2, 0.3),
    ]
    for i, y in enumerate((0.685, -0.685)):
        for j, (bx, bz) in enumerate(bolt_spots):
            frame.visual(
                Cylinder(radius=0.024, length=0.035),
                origin=Origin(xyz=(bx, y, bz), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=deep_blue,
                name=f"bolt_{i}_{j}",
            )

    # Front wall backing that carries the fixed jaw plate.
    frame.visual(
        Box((0.14, 1.22, 1.95)),
        origin=Origin(xyz=(0.82, 0.0, 1.28), rpy=(0.0, FIXED_JAW_PITCH, 0.0)),
        material=steel_blue,
        name="front_wall",
    )
    # Fixed corrugated jaw plate on the chamber side of the front wall.
    frame.visual(
        Box((0.10, 1.10, 1.80)),
        origin=Origin(xyz=(0.72, 0.0, 1.30), rpy=(0.0, FIXED_JAW_PITCH, 0.0)),
        material=manganese,
        name="fixed_jaw_plate",
    )
    fixed_n = (-math.cos(FIXED_JAW_PITCH), math.sin(FIXED_JAW_PITCH))
    for j, y in enumerate((-0.45, -0.27, -0.09, 0.09, 0.27, 0.45)):
        frame.visual(
            Box((0.05, 0.06, 1.70)),
            origin=Origin(
                xyz=(0.72 + 0.075 * fixed_n[0], y, 1.30 + 0.075 * fixed_n[1]),
                rpy=(0.0, FIXED_JAW_PITCH, 0.0),
            ),
            material=manganese,
            name=f"fixed_jaw_rib_{j}",
        )

    # Rear wall closing the frame behind the pitman.
    frame.visual(
        Box((0.10, 1.22, 1.30)),
        origin=Origin(xyz=(-0.95, 0.0, 0.75), rpy=(0.0, -0.35, 0.0)),
        material=steel_blue,
        name="rear_wall",
    )
    # Toggle seat block protruding from the rear wall.
    frame.visual(
        Box((0.25, 0.90, 0.30)),
        origin=Origin(xyz=(-0.75, 0.0, 0.45)),
        material=deep_blue,
        name="toggle_seat",
    )

    # Base rails and cross plates.
    for i, y in enumerate((0.64, -0.64)):
        frame.visual(
            Box((2.40, 0.18, 0.12)),
            origin=Origin(xyz=(0.0, y, 0.06)),
            material=deep_blue,
            name=f"base_rail_{i}",
        )
    frame.visual(
        Box((0.40, 1.46, 0.12)),
        origin=Origin(xyz=(0.95, 0.0, 0.06)),
        material=deep_blue,
        name="base_cross_plate_front",
    )
    frame.visual(
        Box((0.40, 1.46, 0.12)),
        origin=Origin(xyz=(-0.90, 0.0, 0.06)),
        material=deep_blue,
        name="base_cross_plate_rear",
    )
    # Lower front panel tying the front wall down to the base.
    frame.visual(
        Box((0.35, 1.20, 0.45)),
        origin=Origin(xyz=(1.0, 0.0, 0.30)),
        material=steel_blue,
        name="front_lower_panel",
    )
    # Top cross beam over the pitman.
    frame.visual(
        Box((0.55, 1.34, 0.12)),
        origin=Origin(xyz=(-0.25, 0.0, 2.36)),
        material=steel_blue,
        name="top_beam",
    )
    frame.visual(
        Box((0.16, 0.05, 0.14)),
        origin=Origin(xyz=(-0.25, 0.0, 2.49)),
        material=deep_blue,
        name="lifting_lug",
    )

    # Eccentric shaft and its bearing housings on both side plates.
    frame.visual(
        Cylinder(radius=SHAFT_RADIUS, length=2.0 * SHAFT_HALF_LEN),
        origin=Origin(xyz=(SHAFT_X, 0.0, SHAFT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="eccentric_shaft",
    )
    for i, y in enumerate((0.74, -0.74)):
        frame.visual(
            Cylinder(radius=0.20, length=0.14),
            origin=Origin(xyz=(SHAFT_X, y, SHAFT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=deep_blue,
            name=f"bearing_housing_{i}",
        )

    # -------------------------------------------------------------- swing jaw
    # Part frame sits on the eccentric shaft axis; geometry hangs downward.
    swing_jaw = model.part("swing_jaw")

    hub_mesh = mesh_from_cadquery(_pitman_hub_shape(), "pitman_hub")
    swing_jaw.visual(
        hub_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=steel_blue,
        name="pitman_hub",
    )
    # Cheek plates connecting the hub ring down to the pitman body, kept
    # clear of the shaft radius.
    swing_jaw.visual(
        Box((0.24, 1.00, 0.40)),
        origin=Origin(xyz=(0.0, 0.0, -0.32)),
        material=steel_blue,
        name="hub_cheek",
    )
    swing_jaw.visual(
        Box((0.28, 1.05, 1.55)),
        origin=Origin(xyz=(0.02, 0.0, -1.05)),
        material=steel_blue,
        name="pitman_body",
    )
    # Horizontal stiffening ribs across the pitman back.
    for j, z in enumerate((-0.45, -0.95, -1.45)):
        swing_jaw.visual(
            Box((0.08, 1.05, 0.10)),
            origin=Origin(xyz=(-0.15, 0.0, z)),
            material=deep_blue,
            name=f"pitman_rib_{j}",
        )
    # Moving corrugated jaw plate on the chamber (+X) face, embedded into the
    # pitman body at its upper end.
    swing_jaw.visual(
        Box((0.09, 1.05, 1.50)),
        origin=Origin(xyz=(0.20, 0.0, -0.85), rpy=(0.0, SWING_JAW_PITCH, 0.0)),
        material=manganese,
        name="jaw_plate",
    )
    swing_n = (math.cos(SWING_JAW_PITCH), -math.sin(SWING_JAW_PITCH))
    for j, y in enumerate((-0.42, -0.252, -0.084, 0.084, 0.252, 0.42)):
        swing_jaw.visual(
            Box((0.05, 0.06, 1.40)),
            origin=Origin(
                xyz=(0.20 + 0.07 * swing_n[0], y, -0.85 + 0.07 * swing_n[1]),
                rpy=(0.0, SWING_JAW_PITCH, 0.0),
            ),
            material=manganese,
            name=f"jaw_plate_rib_{j}",
        )
    # Toggle plate reaching back into the frame toggle seat.
    swing_jaw.visual(
        Box((0.50, 0.80, 0.08)),
        origin=Origin(xyz=(-0.32, 0.0, -1.61)),
        material=dark_steel,
        name="toggle_plate",
    )

    # -------------------------------------------------------------- flywheels
    wheel_mesh = mesh_from_cadquery(_flywheel_shape(), "flywheel")

    flywheel_0 = model.part("flywheel_0")
    flywheel_0.visual(
        wheel_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=cream,
        name="wheel_body",
    )

    flywheel_1 = model.part("flywheel_1")
    flywheel_1.visual(
        wheel_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cream,
        name="wheel_body",
    )

    # ----------------------------------------------------------- articulations
    model.articulation(
        "jaw_swing",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=swing_jaw,
        origin=Origin(xyz=(SHAFT_X, 0.0, SHAFT_Z)),
        # Jaw hangs along -Z; -Y axis makes positive q swing the jaw bottom
        # toward the fixed jaw at +X (crushing stroke).
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=250000.0, velocity=1.5, lower=0.0, upper=0.04),
    )
    model.articulation(
        "flywheel_0_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=flywheel_0,
        origin=Origin(xyz=(SHAFT_X, 0.90, SHAFT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60000.0, velocity=32.0),
    )
    model.articulation(
        "flywheel_1_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=flywheel_1,
        origin=Origin(xyz=(SHAFT_X, -0.90, SHAFT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60000.0, velocity=32.0),
        mimic=Mimic(joint="flywheel_0_spin", multiplier=1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    swing_jaw = object_model.get_part("swing_jaw")
    flywheel_0 = object_model.get_part("flywheel_0")
    flywheel_1 = object_model.get_part("flywheel_1")
    jaw_swing = object_model.get_articulation("jaw_swing")

    # The toggle plate is intentionally seated into the frame toggle seat.
    ctx.allow_overlap(
        swing_jaw,
        frame,
        elem_a="toggle_plate",
        elem_b="toggle_seat",
        reason="The toggle plate end is intentionally seated in the frame toggle seat socket.",
    )
    # Flywheel hubs are press-fit on the eccentric shaft ends.
    for wheel in (flywheel_0, flywheel_1):
        ctx.allow_overlap(
            frame,
            wheel,
            elem_a="eccentric_shaft",
            elem_b="wheel_body",
            reason="The flywheel hub bore is press-fit captured on the eccentric shaft end.",
        )

    # Wedge crushing chamber: fixed jaw plate stands in front (+X) of the
    # moving jaw plate with a real crushing gap between the working faces.
    ctx.expect_gap(
        frame,
        swing_jaw,
        axis="x",
        positive_elem="fixed_jaw_plate",
        negative_elem="jaw_plate",
        min_gap=0.02,
        max_gap=0.40,
        name="crushing chamber gap between fixed and moving jaw plates",
    )
    ctx.expect_overlap(
        frame,
        swing_jaw,
        axes="y",
        elem_a="fixed_jaw_plate",
        elem_b="jaw_plate",
        min_overlap=0.9,
        name="jaw plates face each other across the full chamber width",
    )
    # Both jaw faces span the chamber height together.
    ctx.expect_overlap(
        frame,
        swing_jaw,
        axes="z",
        elem_a="fixed_jaw_plate",
        elem_b="jaw_plate",
        min_overlap=1.0,
        name="jaw plates overlap vertically to form the wedge chamber",
    )

    # Swing jaw stays between the side plates.
    ctx.expect_within(
        swing_jaw,
        frame,
        axes="y",
        inner_elem="pitman_body",
        margin=0.0,
        name="pitman rides between the side plates",
    )

    # Toggle plate is seated in the seat at rest (retained insertion in X).
    ctx.expect_overlap(
        swing_jaw,
        frame,
        axes="x",
        elem_a="toggle_plate",
        elem_b="toggle_seat",
        min_overlap=0.03,
        name="toggle plate remains seated in the toggle seat",
    )

    # Eccentric shaft passes through both flywheel hub bores.
    for wheel, wheel_name in ((flywheel_0, "flywheel_0"), (flywheel_1, "flywheel_1")):
        ctx.expect_within(
            frame,
            wheel,
            axes="xz",
            inner_elem="eccentric_shaft",
            outer_elem="wheel_body",
            margin=0.001,
            name=f"shaft is coaxial with {wheel_name}",
        )
        ctx.expect_overlap(
            frame,
            wheel,
            axes="y",
            elem_a="eccentric_shaft",
            elem_b="wheel_body",
            min_overlap=0.15,
            name=f"shaft reaches through the {wheel_name} hub",
        )

    # Flywheels are the widest elements: they hang outboard of the side plates.
    aabb_wheel = ctx.part_world_aabb(flywheel_0)
    ctx.check(
        "flywheel_0 sits outboard of the side plates",
        aabb_wheel is not None and aabb_wheel[0][1] > 0.70,
        details=f"flywheel_0 aabb={aabb_wheel}",
    )

    # Crushing stroke: positive q moves the moving jaw face toward the fixed
    # jaw (+X) and narrows the chamber, without hitting the fixed jaw.
    rest_aabb = ctx.part_element_world_aabb(swing_jaw, elem="jaw_plate")
    upper = jaw_swing.motion_limits.upper if jaw_swing.motion_limits else 0.04
    with ctx.pose({jaw_swing: upper}):
        closed_aabb = ctx.part_element_world_aabb(swing_jaw, elem="jaw_plate")
        ctx.expect_gap(
            frame,
            swing_jaw,
            axis="x",
            positive_elem="fixed_jaw_plate",
            negative_elem="jaw_plate",
            min_gap=0.005,
            name="jaw plates never collide at full crushing stroke",
        )
    ctx.check(
        "positive jaw_swing narrows the crushing chamber",
        rest_aabb is not None
        and closed_aabb is not None
        and closed_aabb[1][0] > rest_aabb[1][0] + 0.02,
        details=f"rest={rest_aabb}, closed={closed_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
