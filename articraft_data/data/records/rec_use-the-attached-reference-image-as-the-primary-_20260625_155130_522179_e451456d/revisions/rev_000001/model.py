from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


STEEL_BLACK = Material("painted_black_steel", color=(0.02, 0.025, 0.03, 1.0))
STEEL_DARK = Material("dark_worn_steel", color=(0.09, 0.095, 0.09, 1.0))
RED_PAINT = Material("harvester_red_paint", color=(0.75, 0.03, 0.02, 1.0))
YELLOW_PAINT = Material("safety_yellow", color=(1.0, 0.78, 0.04, 1.0))
RUBBER = Material("matte_black_rubber", color=(0.005, 0.006, 0.006, 1.0))
GLASS = Material("blue_tinted_polycarbonate", color=(0.12, 0.38, 0.55, 0.55))
CHROME = Material("polished_hydraulic_rod", color=(0.82, 0.84, 0.80, 1.0))
ALUMINUM = Material("galvanized_aluminum", color=(0.62, 0.66, 0.62, 1.0))
SOIL_WORN = Material("soil_worn_cutting_metal", color=(0.42, 0.36, 0.28, 1.0))


def _box(part, name, size, xyz, material, rpy=(0.0, 0.0, 0.0)):
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _cyl(part, name, radius, length, xyz, material, rpy=(0.0, 0.0, 0.0)):
    part.visual(Cylinder(radius, length), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _x_cyl(part, name, radius, length, xyz, material):
    _cyl(part, name, radius, length, xyz, material, rpy=(0.0, math.pi / 2.0, 0.0))


def _y_cyl(part, name, radius, length, xyz, material):
    _cyl(part, name, radius, length, xyz, material, rpy=(math.pi / 2.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="agricultural_harvester_vehicle_arm",
        meta={
            "category": "Agricultural",
            "small_class": "Harvester vehicle (arm)",
            "description": "Tracked forestry-style agricultural harvester vehicle with articulated harvesting boom arm.",
        },
    )

    # Root carrier: continuous tracked undercarriage, red cab/engine panels, turret,
    # access steps, lamps, yokes, and track running gear are one rigid supported
    # chassis assembly.
    chassis = model.part("chassis")

    # Tracks and lower frame.
    for side, y in (("side_0", -0.82), ("side_1", 0.82)):
        _box(chassis, f"{side}_rubber_track", (4.55, 0.58, 0.58), (0.0, y, 0.34), RUBBER)
        _box(chassis, f"{side}_track_top_band", (4.15, 0.52, 0.10), (0.0, y, 0.68), STEEL_DARK)
        _box(chassis, f"{side}_track_bottom_band", (4.15, 0.52, 0.10), (0.0, y, 0.05), STEEL_DARK)
        _y_cyl(chassis, f"{side}_front_idler", 0.30, 0.62, (2.02, y, 0.34), STEEL_DARK)
        _y_cyl(chassis, f"{side}_rear_idler", 0.30, 0.62, (-2.02, y, 0.34), STEEL_DARK)
        for i, x in enumerate((-1.15, -0.38, 0.38, 1.15)):
            _y_cyl(chassis, f"{side}_roller_{i}", 0.18, 0.64, (x, y, 0.25), ALUMINUM)
        for i in range(12):
            x = -2.05 + i * 0.37
            _box(chassis, f"{side}_top_grouser_{i}", (0.18, 0.65, 0.06), (x, y, 0.74), RUBBER)
            _box(chassis, f"{side}_bottom_grouser_{i}", (0.18, 0.65, 0.06), (x, y, -0.02), RUBBER)

    _box(chassis, "track_crossbeam", (4.05, 2.05, 0.24), (-0.08, 0.0, 0.76), STEEL_BLACK)
    _box(chassis, "main_frame", (3.30, 1.28, 0.38), (-0.18, 0.0, 1.03), STEEL_BLACK)
    _box(chassis, "belly_guard", (2.75, 1.45, 0.10), (0.20, 0.0, 0.83), SOIL_WORN)

    # Red body panels and cab like the reference: black frame, red engine cowl,
    # blue-tinted polycarbonate cabin glass, side ladder and work lights.
    _box(chassis, "rear_engine_cowl", (1.08, 1.12, 0.62), (-1.72, 0.04, 1.43), RED_PAINT)
    _box(chassis, "engine_black_grille", (0.08, 0.90, 0.38), (-1.14, 0.04, 1.48), STEEL_BLACK)
    for i, z in enumerate((1.30, 1.46, 1.62)):
        _box(chassis, f"grille_slat_{i}", (0.10, 0.96, 0.035), (-1.24, 0.04, z), ALUMINUM)
    _box(chassis, "cab_red_shell", (0.92, 1.04, 0.96), (-1.45, -0.22, 2.08), RED_PAINT)
    _box(chassis, "cab_front_glass", (0.07, 0.82, 0.62), (-0.96, -0.22, 2.17), GLASS, rpy=(0.0, 0.22, 0.0))
    _box(chassis, "cab_side_glass", (0.62, 0.08, 0.58), (-1.42, -0.75, 2.18), GLASS)
    _box(chassis, "cab_rear_glass", (0.06, 0.72, 0.45), (-1.91, -0.22, 2.16), GLASS)
    _box(chassis, "cab_roof", (1.06, 1.12, 0.13), (-1.45, -0.22, 2.62), STEEL_BLACK)
    _box(chassis, "door_handle", (0.05, 0.05, 0.34), (-1.10, -0.79, 2.04), STEEL_BLACK)
    _box(chassis, "side_step_upper", (0.62, 0.36, 0.08), (-1.20, -1.02, 1.12), ALUMINUM)
    _box(chassis, "side_step_lower", (0.70, 0.34, 0.08), (-1.05, -1.05, 0.80), ALUMINUM)
    _box(chassis, "step_hanger_0", (0.07, 0.07, 0.45), (-1.48, -0.89, 0.98), STEEL_BLACK)
    _box(chassis, "step_hanger_1", (0.07, 0.07, 0.45), (-0.88, -0.89, 0.98), STEEL_BLACK)
    _box(chassis, "front_work_light_0", (0.12, 0.20, 0.10), (-0.82, -0.62, 2.50), YELLOW_PAINT)
    _box(chassis, "front_work_light_1", (0.12, 0.20, 0.10), (-0.82, 0.28, 2.50), YELLOW_PAINT)

    # Turret/pedestal for the arm. The yoke plates leave a visible slot where the
    # lower boom pivot barrel nests, with enough clearance to avoid broad overlap.
    _cyl(chassis, "slew_ring", 0.62, 0.22, (-0.55, 0.0, 1.25), STEEL_DARK)
    _box(chassis, "boom_pedestal", (0.54, 0.64, 0.34), (-0.55, 0.0, 1.10), STEEL_BLACK)
    _box(chassis, "pedestal_crosshead", (1.10, 1.02, 0.18), (-0.55, 0.0, 1.18), STEEL_BLACK)
    _box(chassis, "base_yoke_0", (0.34, 0.12, 0.34), (-1.25, -0.45, 1.25), STEEL_BLACK)
    _box(chassis, "base_yoke_1", (0.34, 0.12, 0.34), (-1.25, 0.45, 1.25), STEEL_BLACK)
    _y_cyl(chassis, "base_pivot_pin", 0.07, 1.12, (-1.25, 0.0, 1.28), ALUMINUM)

    # Rotating upper structure lets the harvesting arm slew like the reference.
    turret = model.part("turret")
    _cyl(turret, "turntable_drum", 0.50, 0.18, (0.0, 0.0, 0.09), STEEL_DARK)
    _cyl(turret, "turntable_neck", 0.22, 0.44, (0.0, 0.0, 0.38), STEEL_DARK)
    _box(turret, "boom_mount_web", (0.36, 0.74, 0.32), (0.0, 0.0, 0.70), STEEL_BLACK)
    _box(turret, "upper_yoke_0", (0.30, 0.10, 0.62), (0.0, -0.39, 1.16), STEEL_BLACK)
    _box(turret, "upper_yoke_1", (0.30, 0.10, 0.62), (0.0, 0.39, 1.16), STEEL_BLACK)
    model.articulation(
        "chassis_to_turret",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=turret,
        origin=Origin(xyz=(-0.55, 0.0, 1.36)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6000.0, velocity=0.45, lower=-1.45, upper=1.45),
    )

    # Lower boom: a heavy boxed arm with pivot barrels, side web plates, warning
    # labels, hydraulic barrel/rod pairs and supported hoses.
    lower_boom = model.part("lower_boom")
    _box(lower_boom, "lower_box_tube", (2.60, 0.34, 0.34), (1.31, 0.0, 0.02), STEEL_BLACK)
    _box(lower_boom, "lower_top_reinforcement", (2.45, 0.20, 0.10), (1.70, 0.0, 0.22), STEEL_DARK)
    _box(lower_boom, "lower_side_plate_0", (2.55, 0.045, 0.54), (1.52, -0.205, 0.02), STEEL_BLACK)
    _box(lower_boom, "lower_side_plate_1", (2.55, 0.045, 0.54), (1.52, 0.205, 0.02), STEEL_BLACK)
    _y_cyl(lower_boom, "root_pivot_barrel", 0.21, 0.68, (0.0, 0.0, 0.0), STEEL_DARK)
    _y_cyl(lower_boom, "tip_pivot_barrel", 0.18, 0.64, (2.95, 0.0, 0.0), STEEL_DARK)
    _box(lower_boom, "tip_yoke_0", (0.38, 0.10, 0.60), (2.95, -0.37, 0.0), STEEL_BLACK)
    _box(lower_boom, "tip_yoke_1", (0.38, 0.10, 0.60), (2.95, 0.37, 0.0), STEEL_BLACK)
    _box(lower_boom, "yellow_capacity_plate", (0.82, 0.03, 0.16), (2.20, -0.235, 0.31), YELLOW_PAINT)
    for i, x in enumerate((1.92, 2.12, 2.32, 2.52)):
        _box(lower_boom, f"capacity_black_mark_{i}", (0.025, 0.035, 0.18), (x, -0.255, 0.315), STEEL_BLACK, rpy=(0.0, 0.0, 0.35))
    for side, y in (("side_0", -0.30), ("side_1", 0.30)):
        _x_cyl(lower_boom, f"{side}_lift_cylinder_body", 0.075, 1.00, (0.78, y, -0.25), STEEL_DARK)
        _x_cyl(lower_boom, f"{side}_lift_cylinder_rod", 0.040, 1.30, (1.85, y, -0.25), CHROME)
        _box(lower_boom, f"{side}_cylinder_mount", (0.18, 0.12, 0.20), (0.28, y, -0.17), STEEL_BLACK)
        _box(lower_boom, f"{side}_rod_clevis", (0.20, 0.13, 0.17), (2.55, y, -0.24), STEEL_BLACK)
    hose = tube_from_spline_points(
        [(0.08, 0.23, 0.18), (0.85, 0.25, 0.27), (1.65, 0.25, 0.28), (2.78, 0.23, 0.14)],
        radius=0.030,
        samples_per_segment=10,
        radial_segments=14,
        cap_ends=True,
    )
    lower_boom.visual(mesh_from_geometry(hose, "lower_boom_hydraulic_hose"), material=RUBBER, name="hydraulic_hose")
    for i, x in enumerate((0.52, 1.42, 2.28)):
        _box(lower_boom, f"hose_clamp_{i}", (0.08, 0.10, 0.08), (x, 0.205, 0.20), ALUMINUM)

    model.articulation(
        "turret_to_lower_boom",
        ArticulationType.REVOLUTE,
        parent=turret,
        child=lower_boom,
        origin=Origin(xyz=(0.0, 0.0, 1.16), rpy=(0.0, -0.88, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=9000.0, velocity=0.35, lower=-0.45, upper=0.55),
    )

    # Stick/outer boom: another boxed section, slightly drooped from the lower
    # boom, with its own visible hydraulic supports and hose bundle.
    stick = model.part("stick")
    _box(stick, "stick_box_tube", (1.65, 0.30, 0.30), (1.18, 0.0, 0.0), STEEL_BLACK)
    _box(stick, "stick_top_wear_plate", (1.85, 0.18, 0.08), (1.38, 0.0, 0.18), STEEL_DARK)
    _box(stick, "stick_side_plate_0", (1.80, 0.04, 0.46), (1.30, -0.17, 0.0), STEEL_BLACK)
    _box(stick, "stick_side_plate_1", (1.80, 0.04, 0.46), (1.30, 0.17, 0.0), STEEL_BLACK)
    _box(stick, "root_neck", (0.22, 0.22, 0.18), (0.265, 0.0, 0.0), STEEL_BLACK)
    _y_cyl(stick, "stick_root_barrel", 0.17, 0.28, (0.0, 0.0, 0.0), STEEL_DARK)
    _y_cyl(stick, "wrist_barrel", 0.14, 0.58, (2.45, 0.0, 0.0), STEEL_DARK)
    _box(stick, "wrist_neck", (0.44, 0.20, 0.16), (2.21, 0.0, 0.0), STEEL_BLACK)
    _box(stick, "wrist_yoke_0", (0.30, 0.10, 0.48), (2.45, -0.30, 0.0), STEEL_BLACK)
    _box(stick, "wrist_yoke_1", (0.30, 0.10, 0.48), (2.45, 0.30, 0.0), STEEL_BLACK)
    _x_cyl(stick, "crowd_cylinder_body", 0.065, 0.85, (0.70, -0.25, -0.22), STEEL_DARK)
    _x_cyl(stick, "crowd_cylinder_rod", 0.034, 1.05, (1.62, -0.25, -0.22), CHROME)
    _box(stick, "crowd_mount_root", (0.18, 0.12, 0.18), (0.50, -0.25, -0.16), STEEL_BLACK)
    _box(stick, "crowd_rod_clevis", (0.18, 0.12, 0.16), (2.18, -0.25, -0.22), STEEL_BLACK)
    stick_hose = tube_from_spline_points(
        [(0.42, 0.21, 0.15), (0.82, 0.23, 0.24), (1.38, 0.22, 0.22), (2.05, 0.22, 0.02)],
        radius=0.024,
        samples_per_segment=10,
        radial_segments=12,
        cap_ends=True,
    )
    stick.visual(mesh_from_geometry(stick_hose, "stick_hydraulic_hose"), material=RUBBER, name="hose_bundle")
    for i, x in enumerate((0.55, 1.45, 2.05)):
        _box(stick, f"hose_clip_{i}", (0.07, 0.08, 0.06), (x, 0.185, 0.17), ALUMINUM)

    model.articulation(
        "lower_boom_to_stick",
        ArticulationType.REVOLUTE,
        parent=lower_boom,
        child=stick,
        origin=Origin(xyz=(2.95, 0.0, 0.0), rpy=(0.0, 0.50, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6500.0, velocity=0.45, lower=-0.75, upper=0.80),
    )

    # Harvesting head: compact felling/processing head with red motor cover,
    # black grapple frame, rollers, feed drum, delimbing knives, saw bar and
    # short hose loop. It hangs from the wrist joint and articulates to follow
    # logs on the ground.
    head = model.part("harvester_head")
    _y_cyl(head, "head_wrist_pin", 0.13, 0.46, (0.0, 0.0, 0.0), ALUMINUM)
    _box(head, "hanging_link", (0.18, 0.34, 0.72), (0.12, 0.0, -0.36), STEEL_BLACK)
    _box(head, "head_backbone", (0.95, 0.50, 0.26), (0.42, 0.0, -0.86), STEEL_BLACK)
    _box(head, "red_motor_cover", (0.72, 0.46, 0.32), (0.24, 0.0, -0.58), RED_PAINT)
    _box(head, "bottom_feed_frame", (0.98, 0.56, 0.16), (0.50, 0.0, -1.12), STEEL_BLACK)
    _y_cyl(head, "feed_roller_0", 0.13, 0.62, (0.35, -0.02, -1.00), RUBBER)
    _y_cyl(head, "feed_roller_1", 0.13, 0.62, (0.68, -0.02, -1.00), RUBBER)
    _box(head, "saw_bar", (0.78, 0.08, 0.08), (0.86, 0.0, -1.20), SOIL_WORN, rpy=(0.0, 0.0, -0.22))
    for side, y, yaw in (("side_0", -0.34, -0.25), ("side_1", 0.34, 0.25)):
        _box(head, f"{side}_grapple_arm", (0.70, 0.09, 0.10), (0.46, y, -1.22), STEEL_BLACK, rpy=(0.0, 0.0, yaw))
        _box(head, f"{side}_delimbing_knife", (0.42, 0.06, 0.08), (0.72, y, -0.90), SOIL_WORN, rpy=(0.0, 0.0, yaw))
    head_hose = tube_from_spline_points(
        [(0.30, 0.20, -0.42), (0.32, 0.31, -0.50), (0.40, 0.26, -0.58)],
        radius=0.022,
        samples_per_segment=12,
        radial_segments=12,
        cap_ends=True,
    )
    head.visual(mesh_from_geometry(head_hose, "head_hose_loop"), material=RUBBER, name="head_hose_loop")

    model.articulation(
        "stick_to_harvester_head",
        ArticulationType.REVOLUTE,
        parent=stick,
        child=head,
        origin=Origin(xyz=(2.45, 0.0, 0.0), rpy=(0.0, 0.12, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2500.0, velocity=0.75, lower=-1.05, upper=1.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    ctx.check(
        "small class is harvester vehicle arm",
        object_model.meta.get("small_class") == "Harvester vehicle (arm)"
        and "harvester" in object_model.name
        and "arm" in object_model.name,
        details=f"name={object_model.name!r}, meta={object_model.meta}",
    )

    required_parts = {"chassis", "turret", "lower_boom", "stick", "harvester_head"}
    ctx.check(
        "major subassemblies are present",
        required_parts.issubset({p.name for p in object_model.parts}),
        details=f"parts={[p.name for p in object_model.parts]}",
    )

    non_fixed = [
        j.name
        for j in object_model.articulations
        if j.articulation_type != ArticulationType.FIXED
    ]
    ctx.check(
        "harvesting arm has multiple non-fixed joints",
        {"chassis_to_turret", "turret_to_lower_boom", "lower_boom_to_stick", "stick_to_harvester_head"}.issubset(
            set(non_fixed)
        ),
        details=f"non_fixed={non_fixed}",
    )

    lower = object_model.get_articulation("turret_to_lower_boom")
    stick_joint = object_model.get_articulation("lower_boom_to_stick")
    head_joint = object_model.get_articulation("stick_to_harvester_head")
    lower_boom = object_model.get_part("lower_boom")
    stick = object_model.get_part("stick")
    head = object_model.get_part("harvester_head")
    chassis = object_model.get_part("chassis")

    ctx.allow_overlap(
        lower_boom,
        stick,
        reason="The stick root barrel and neck are intentionally captured in the lower-boom clevis as the revolute pivot assembly.",
    )
    ctx.allow_overlap(
        stick,
        head,
        reason="The head wrist pin and hanging link are intentionally nested through the wrist clevis as a visible pivot.",
    )

    ctx.expect_origin_distance(
        lower_boom,
        stick,
        axes="xyz",
        min_dist=2.75,
        max_dist=3.15,
        name="outer stick pivot sits at the end of the lower boom",
    )
    ctx.expect_origin_gap(
        head,
        chassis,
        axis="z",
        min_gap=0.20,
        name="harvesting head is suspended above the carrier at rest",
    )
    ctx.expect_overlap(
        stick,
        lower_boom,
        axes="yz",
        min_overlap=0.20,
        name="stick pivot assembly is captured between lower-boom yoke plates",
    )
    ctx.expect_overlap(
        head,
        stick,
        axes="yz",
        min_overlap=0.20,
        name="harvester head wrist assembly is captured at the stick tip",
    )

    rest_head_pos = ctx.part_world_position(head)
    rest_stick_pos = ctx.part_world_position(stick)
    with ctx.pose({lower: 0.45, stick_joint: 0.20, head_joint: -0.25}):
        raised_head_pos = ctx.part_world_position(head)
        raised_stick_pos = ctx.part_world_position(stick)
        ctx.expect_origin_gap(
            head,
            chassis,
            axis="z",
            min_gap=0.75,
            name="raised boom lifts the harvester head clear of the chassis",
        )

    ctx.check(
        "positive boom articulation raises the visible arm",
        rest_head_pos is not None
        and raised_head_pos is not None
        and rest_stick_pos is not None
        and raised_stick_pos is not None
        and raised_head_pos[2] > rest_head_pos[2] + 0.55
        and raised_stick_pos[2] > rest_stick_pos[2] + 0.45,
        details=f"rest_head={rest_head_pos}, raised_head={raised_head_pos}, rest_stick={rest_stick_pos}, raised_stick={raised_stick_pos}",
    )

    return ctx.report()


object_model = build_object_model()
