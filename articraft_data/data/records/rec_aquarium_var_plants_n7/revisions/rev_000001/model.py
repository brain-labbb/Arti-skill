from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _rounded_plate(
    width: float,
    depth: float,
    thickness: float,
    *,
    center_y: float,
    bottom_z: float,
    radius: float,
    cutout: tuple[float, float, float, float] | None = None,
):
    """CadQuery rounded rectangular plate in the part frame.

    The plate is centered on X, centered at ``center_y`` on Y, and has its
    lower face at ``bottom_z``.  ``cutout`` is (center_x, center_y, width,
    depth) and cuts all the way through the plate.
    """

    plate = (
        cq.Workplane("XY")
        .box(width, depth, thickness)
        .translate((0.0, center_y, bottom_z + thickness * 0.5))
        .edges("|Z")
        .fillet(radius)
    )
    if cutout is not None:
        cx, cy, cut_w, cut_d = cutout
        cutter = (
            cq.Workplane("XY")
            .box(cut_w, cut_d, thickness * 4.0)
            .translate((cx, cy, bottom_z + thickness * 0.5))
        )
        plate = plate.cut(cutter)
    return plate


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="small_desktop_aquarium")

    glass = model.material("slightly_blue_glass", rgba=(0.70, 0.88, 1.00, 0.32))
    water = model.material("clear_blue_water", rgba=(0.18, 0.55, 0.90, 0.38))
    black = model.material("black_plastic", rgba=(0.015, 0.017, 0.018, 1.0))
    rubber = model.material("soft_black_rubber", rgba=(0.04, 0.045, 0.05, 1.0))
    dark_marble = model.material("dark_marbled_base", rgba=(0.02, 0.08, 0.11, 1.0))
    gravel = model.material("blue_aquarium_gravel", rgba=(0.08, 0.55, 0.86, 1.0))
    green = model.material("plastic_plant_green", rgba=(0.10, 0.58, 0.18, 1.0))
    bright_green = model.material("light_plant_green", rgba=(0.42, 0.90, 0.25, 1.0))
    magenta = model.material("pink_aquatic_plant", rgba=(0.88, 0.08, 0.45, 1.0))
    rock = model.material("grey_rock", rgba=(0.40, 0.41, 0.39, 1.0))
    warm_light = model.material("aqua_led_lens", rgba=(0.25, 0.90, 1.0, 0.75))

    tank = model.part("tank")

    # Real-world small desktop tank proportions: roughly 50 x 32 cm footprint.
    width = 0.52
    depth = 0.32
    wall = 0.006
    glass_min_z = 0.055
    glass_h = 0.285
    glass_center_z = glass_min_z + glass_h * 0.5
    half_w = width * 0.5
    half_d = depth * 0.5

    # Transparent glass basin, open at the top and seated in a black base trim.
    tank.visual(
        Box((width, wall, glass_h)),
        origin=Origin(xyz=(0.0, -half_d + wall * 0.5, glass_center_z)),
        material=glass,
        name="front_glass",
    )
    tank.visual(
        Box((width, wall, glass_h)),
        origin=Origin(xyz=(0.0, half_d - wall * 0.5, glass_center_z)),
        material=glass,
        name="rear_glass",
    )
    tank.visual(
        Box((wall, depth - 2 * wall, glass_h)),
        origin=Origin(xyz=(-half_w + wall * 0.5, 0.0, glass_center_z)),
        material=glass,
        name="side_glass_0",
    )
    tank.visual(
        Box((wall, depth - 2 * wall, glass_h)),
        origin=Origin(xyz=(half_w - wall * 0.5, 0.0, glass_center_z)),
        material=glass,
        name="side_glass_1",
    )
    tank.visual(
        Box((width - 2 * wall, depth - 2 * wall, wall)),
        origin=Origin(xyz=(0.0, 0.0, glass_min_z + wall * 0.5)),
        material=glass,
        name="bottom_glass",
    )

    # Raised base plinth and top frame lip, as seen in the reference.
    tank.visual(
        Box((0.55, 0.35, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=dark_marble,
        name="base_plinth",
    )
    tank.visual(
        Box((0.55, 0.020, 0.026)),
        origin=Origin(xyz=(0.0, -0.168, 0.342)),
        material=black,
        name="top_rim_front",
    )
    tank.visual(
        Box((0.55, 0.020, 0.026)),
        origin=Origin(xyz=(0.0, 0.168, 0.342)),
        material=black,
        name="top_rim_rear",
    )
    tank.visual(
        Box((0.020, 0.32, 0.026)),
        origin=Origin(xyz=(-0.265, 0.0, 0.342)),
        material=black,
        name="top_rim_side_0",
    )
    tank.visual(
        Box((0.020, 0.32, 0.026)),
        origin=Origin(xyz=(0.265, 0.0, 0.342)),
        material=black,
        name="top_rim_side_1",
    )
    tank.visual(
        Box((0.55, 0.030, 0.036)),
        origin=Origin(xyz=(0.0, -0.174, 0.058)),
        material=black,
        name="lower_front_trim",
    )
    tank.visual(
        Box((0.55, 0.022, 0.030)),
        origin=Origin(xyz=(0.0, 0.172, 0.060)),
        material=black,
        name="lower_rear_trim",
    )

    # Contents are kept structural: water, gravel bed, filter, rocks, and a few
    # rooted plants so the object reads as an aquarium rather than an empty box.
    tank.visual(
        Box((width - 2 * wall, depth - 2 * wall, 0.208)),
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        material=water,
        name="water_volume",
    )
    tank.visual(
        Box((width - 0.035, depth - 0.035, 0.038)),
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        material=gravel,
        name="gravel_bed",
    )
    tank.visual(
        Box((0.085, 0.032, 0.175)),
        origin=Origin(xyz=(0.205, 0.126, 0.172)),
        material=black,
        name="filter_housing",
    )
    tank.visual(
        Box((0.020, 0.018, 0.155)),
        origin=Origin(xyz=(0.158, 0.132, 0.178)),
        material=rubber,
        name="filter_intake",
    )
    tank.visual(
        Cylinder(radius=0.030, length=0.028),
        origin=Origin(xyz=(0.020, -0.030, 0.111), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rock,
        name="front_rock",
    )
    tank.visual(
        Cylinder(radius=0.046, length=0.044),
        origin=Origin(xyz=(0.082, 0.035, 0.120), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rock,
        name="center_rock",
    )
    for i, (x, y, h, mat) in enumerate(
        [
            (-0.170, -0.030, 0.130, magenta),
            (-0.080, -0.015, 0.105, bright_green),
            (0.135, 0.028, 0.150, green),
            (0.175, -0.052, 0.120, magenta),
            (-0.120, 0.080, 0.140, green),
            (0.000, -0.090, 0.115, bright_green),
            (-0.010, 0.065, 0.125, magenta),
        ]
    ):
        tank.visual(
            Cylinder(radius=0.010, length=h),
            origin=Origin(xyz=(x, y, 0.097 + h * 0.5)),
            material=mat,
            name=f"plant_stem_{i}",
        )
        tank.visual(
            Box((0.070, 0.010, 0.035)),
            origin=Origin(xyz=(x + 0.014, y, 0.135 + h), rpy=(0.0, 0.45, 0.65)),
            material=mat,
            name=f"plant_leaf_{i}_0",
        )
        tank.visual(
            Box((0.060, 0.010, 0.032)),
            origin=Origin(xyz=(x - 0.014, y, 0.124 + h * 0.85), rpy=(0.0, -0.40, -0.65)),
            material=mat,
            name=f"plant_leaf_{i}_1",
        )

    # Simple rear hinge supports mounted to the tank frame.
    tank.visual(
        Box((0.105, 0.018, 0.014)),
        origin=Origin(xyz=(-0.170, 0.187, 0.350)),
        material=black,
        name="hinge_mount_0",
    )
    tank.visual(
        Box((0.105, 0.018, 0.014)),
        origin=Origin(xyz=(0.170, 0.187, 0.350)),
        material=black,
        name="hinge_mount_1",
    )

    hood = model.part("hood")
    hood_shell = _rounded_plate(
        0.555,
        0.356,
        0.032,
        center_y=-0.174,
        bottom_z=0.0,
        radius=0.030,
        cutout=(0.0, -0.219, 0.195, 0.118),
    )
    hood.visual(
        mesh_from_cadquery(hood_shell, "rounded_hood_shell", tolerance=0.0008),
        material=black,
        name="hood_shell",
    )
    hood.visual(
        Box((0.135, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, -0.356, 0.018)),
        material=rubber,
        name="front_finger_scoop",
    )
    hood.visual(
        Cylinder(radius=0.007, length=0.460),
        origin=Origin(xyz=(0.0, 0.011, 0.008), rpy=(0.0, pi / 2.0, 0.0)),
        material=black,
        name="hood_hinge_barrel",
    )
    hood.visual(
        Box((0.390, 0.024, 0.010)),
        origin=Origin(xyz=(0.0, -0.100, -0.005)),
        material=warm_light,
        name="led_light_lens",
    )
    hood.visual(
        Box((0.180, 0.010, 0.0025)),
        origin=Origin(xyz=(0.0, -0.160, 0.03325)),
        material=black,
        name="flap_hinge_socket",
    )

    flap = model.part("feeding_flap")
    flap_plate = _rounded_plate(
        0.215,
        0.126,
        0.006,
        center_y=-0.061,
        bottom_z=0.0,
        radius=0.014,
    )
    flap.visual(
        mesh_from_cadquery(flap_plate, "feeding_flap_plate", tolerance=0.0006),
        material=black,
        name="flap_panel",
    )
    flap.visual(
        Cylinder(radius=0.0045, length=0.165),
        origin=Origin(xyz=(0.0, 0.001, 0.005), rpy=(0.0, pi / 2.0, 0.0)),
        material=black,
        name="flap_hinge_pin",
    )
    flap.visual(
        Cylinder(radius=0.014, length=0.005),
        origin=Origin(xyz=(0.0, -0.115, 0.0085)),
        material=rubber,
        name="flap_pull_tab",
    )

    model.articulation(
        "tank_to_hood",
        ArticulationType.REVOLUTE,
        parent=tank,
        child=hood,
        origin=Origin(xyz=(0.0, 0.180, 0.357)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.4, lower=0.0, upper=1.25),
    )
    model.articulation(
        "hood_to_flap",
        ArticulationType.REVOLUTE,
        parent=hood,
        child=flap,
        origin=Origin(xyz=(0.0, -0.162, 0.034)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=2.0, lower=0.0, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tank = object_model.get_part("tank")
    hood = object_model.get_part("hood")
    flap = object_model.get_part("feeding_flap")
    hood_joint = object_model.get_articulation("tank_to_hood")
    flap_joint = object_model.get_articulation("hood_to_flap")

    # Reference/category note: no mismatch suspected; the image shows a compact
    # desktop aquarium with a black hinged hood and transparent glass tank.
    ctx.check(
        "core aquarium parts are modeled",
        tank is not None and hood is not None and flap is not None,
        details="Expected tank, hood, and feeding_flap parts.",
    )
    ctx.expect_within(
        tank,
        tank,
        axes="xy",
        inner_elem="water_volume",
        outer_elem="bottom_glass",
        margin=0.001,
        name="water sits within the glass basin footprint",
    )
    ctx.expect_gap(
        hood,
        tank,
        axis="z",
        positive_elem="hood_shell",
        negative_elem="top_rim_front",
        min_gap=0.0,
        max_gap=0.006,
        name="closed hood seats just above front rim",
    )
    ctx.expect_within(
        flap,
        hood,
        axes="xy",
        inner_elem="flap_panel",
        outer_elem="hood_shell",
        margin=0.010,
        name="feeding flap is nested in hood opening footprint",
    )

    closed_hood_aabb = ctx.part_element_world_aabb(hood, elem="hood_shell")
    with ctx.pose({hood_joint: 0.95}):
        open_hood_aabb = ctx.part_element_world_aabb(hood, elem="hood_shell")
    ctx.check(
        "hood hinge opens upward",
        closed_hood_aabb is not None
        and open_hood_aabb is not None
        and open_hood_aabb[1][2] > closed_hood_aabb[1][2] + 0.10,
        details=f"closed={closed_hood_aabb}, open={open_hood_aabb}",
    )

    closed_flap_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
    hood_world_y = 0.180
    cutout_min = (-0.195 / 2.0, hood_world_y - 0.219 - 0.118 / 2.0)
    cutout_max = (0.195 / 2.0, hood_world_y - 0.219 + 0.118 / 2.0)
    flap_min, flap_max = (
        closed_flap_aabb if closed_flap_aabb is not None else ((0, 0, 0), (0, 0, 0))
    )
    ctx.check(
        "closed feeding flap covers the full hood opening",
        closed_flap_aabb is not None
        and flap_min[0] <= cutout_min[0] - 0.004
        and flap_max[0] >= cutout_max[0] + 0.004
        and flap_min[1] <= cutout_min[1] - 0.004
        and flap_max[1] >= cutout_max[1] - 0.001,
        details=f"cutout_min={cutout_min}, cutout_max={cutout_max}, flap={closed_flap_aabb}",
    )
    with ctx.pose({flap_joint: 0.90}):
        open_flap_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
    ctx.check(
        "feeding flap opens upward",
        closed_flap_aabb is not None
        and open_flap_aabb is not None
        and open_flap_aabb[1][2] > closed_flap_aabb[1][2] + 0.035,
        details=f"closed={closed_flap_aabb}, open={open_flap_aabb}",
    )

    # Verify densely planted aquascape: 7 rooted plants (plant_stem_0..6).
    plant_names = [f"plant_stem_{i}" for i in range(7)]
    all_plants_present = all(
        any(v.name == pn for v in tank.visuals) for pn in plant_names
    )
    ctx.check(
        "seven rooted plants are modeled (plant_stem_0..6)",
        all_plants_present,
        details=f"Expected 7 plant stems, found: {[v.name for v in tank.visuals if v.name.startswith('plant_stem_')]}",
    )
    # Confirm the last plant (index 6) sits within the water footprint on XY.
    ctx.expect_within(
        tank,
        tank,
        axes="xy",
        inner_elem="plant_stem_6",
        outer_elem="water_volume",
        margin=0.005,
        name="plant_stem_6 is within the water volume footprint",
    )

    return ctx.report()


object_model = build_object_model()
