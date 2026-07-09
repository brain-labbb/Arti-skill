from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


TABLE_LENGTH = 0.62
TABLE_DEPTH = 0.40
TABLE_THICKNESS = 0.035
TABLE_CENTER_X = 0.31
TABLE_CENTER_Z = 0.070
TABLE_TOP_Z = TABLE_CENTER_Z + TABLE_THICKNESS / 2.0
PIVOT_X = 0.17
PIVOT_Z = -0.020
HEIGHT_REST_Z = 0.56


def _rounded_slot_cutter(x: float, y: float, length: float, width: float, height: float, *, angle: float) -> cq.Workplane:
    """Centered through-cutter for a rounded drill-table slot."""
    return (
        cq.Workplane("XY")
        .center(x, y)
        .slot2D(length, width, angle=angle)
        .extrude(height)
        .translate((0.0, 0.0, -height / 2.0))
    )


def _table_plate_shape() -> cq.Workplane:
    """Cast-iron drill-press work table with real through-slots and holes."""
    cut_height = TABLE_THICKNESS + 0.030
    plate = (
        cq.Workplane("XY")
        .box(TABLE_LENGTH, TABLE_DEPTH, TABLE_THICKNESS)
        .edges("|Z")
        .fillet(0.018)
        .edges(">Z")
        .chamfer(0.002)
    )

    # Two long T-slot openings running front-to-back and a central drill relief.
    for x in (-0.155, 0.155):
        plate = plate.cut(_rounded_slot_cutter(x, 0.0, 0.315, 0.030, cut_height, angle=90.0))

    center_hole = (
        cq.Workplane("XY")
        .circle(0.033)
        .extrude(cut_height)
        .translate((0.0, 0.0, -cut_height / 2.0))
    )
    cross_slot = _rounded_slot_cutter(0.0, 0.0, 0.170, 0.026, cut_height, angle=0.0)
    plate = plate.cut(center_hole).cut(cross_slot)

    # Small corner mounting holes like cast drill-press tables.
    for x in (-0.245, 0.245):
        for y in (-0.145, 0.145):
            hole = (
                cq.Workplane("XY")
                .center(x, y)
                .circle(0.012)
                .extrude(cut_height)
                .translate((0.0, 0.0, -cut_height / 2.0))
            )
            plate = plate.cut(hole)
    return plate


def _carriage_shape() -> cq.Workplane:
    """Sliding clamp collar, arm, and bored trunnion yoke around the column."""
    side_strap_a = cq.Workplane("XY").box(0.018, 0.096, 0.095).translate((-0.050, 0.0, 0.0))
    side_strap_b = cq.Workplane("XY").box(0.018, 0.096, 0.095).translate((0.050, 0.0, 0.0))
    rear_bridge = cq.Workplane("XY").box(0.118, 0.018, 0.095).translate((0.0, 0.048, 0.0))
    front_clamp_ear = cq.Workplane("XY").box(0.084, 0.030, 0.045).translate((0.0, -0.063, 0.0))
    collar = side_strap_a.union(side_strap_b).union(rear_bridge).union(front_clamp_ear)

    support_arm = cq.Workplane("XY").box(0.145, 0.070, 0.040).translate((0.103, 0.0, -0.037))
    lower_rib = cq.Workplane("XY").box(0.110, 0.045, 0.020).translate((0.100, 0.0, -0.067))
    clamp_boss = cq.Workplane("XY").box(0.064, 0.044, 0.040).translate((0.0, -0.081, 0.0))
    yoke_bridge = cq.Workplane("XY").box(0.070, 0.185, 0.022).translate((0.150, 0.0, -0.064))

    cheek_a = cq.Workplane("XY").box(0.054, 0.024, 0.095).translate((PIVOT_X, -0.080, PIVOT_Z))
    cheek_b = cq.Workplane("XY").box(0.054, 0.024, 0.095).translate((PIVOT_X, 0.080, PIVOT_Z))

    carriage = collar.union(support_arm).union(lower_rib).union(clamp_boss).union(yoke_bridge).union(cheek_a).union(cheek_b)

    # Bore the yoke for the table trunnion shaft, leaving visible clearance.
    yoke_bore = (
        cq.Workplane("XY")
        .cylinder(0.230, 0.023)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((PIVOT_X, 0.0, PIVOT_Z))
    )
    carriage = carriage.cut(yoke_bore)
    return carriage


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="adjustable_drill_press_table")

    black_iron = model.material("black_cast_iron", rgba=(0.015, 0.014, 0.012, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.62, 0.62, 0.58, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.11, 0.11, 0.10, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.82, 0.80, 0.74, 1.0))
    knob_black = model.material("black_plastic", rgba=(0.01, 0.01, 0.012, 1.0))

    column = model.part("column")
    column.visual(
        Cylinder(radius=0.032, length=1.02),
        origin=Origin(xyz=(0.0, 0.0, 0.51)),
        material=satin_steel,
        name="column_shaft",
    )
    column.visual(
        Cylinder(radius=0.038, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, HEIGHT_REST_Z - 0.085)),
        material=dark_steel,
        name="stop_collar",
    )

    carriage = model.part("carriage")
    for name, x in (("column_jaw_0", -0.050), ("column_jaw_1", 0.050)):
        carriage.visual(
            Box((0.018, 0.096, 0.095)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=dark_steel,
            name=name,
        )
    carriage.visual(
        Box((0.118, 0.018, 0.095)),
        origin=Origin(xyz=(0.0, 0.048, 0.0)),
        material=dark_steel,
        name="rear_clamp_bridge",
    )
    carriage.visual(
        Box((0.084, 0.030, 0.045)),
        origin=Origin(xyz=(0.0, -0.063, 0.0)),
        material=dark_steel,
        name="front_clamp_ear",
    )
    carriage.visual(
        Box((0.115, 0.070, 0.040)),
        origin=Origin(xyz=(0.085, 0.0, -0.037)),
        material=dark_steel,
        name="support_arm",
    )
    carriage.visual(
        Box((0.110, 0.045, 0.020)),
        origin=Origin(xyz=(0.100, 0.0, -0.067)),
        material=dark_steel,
        name="lower_rib",
    )
    carriage.visual(
        Box((0.070, 0.185, 0.022)),
        origin=Origin(xyz=(0.150, 0.0, -0.064)),
        material=dark_steel,
        name="yoke_bridge",
    )
    for side, y in enumerate((-0.080, 0.080)):
        carriage.visual(
            Box((0.054, 0.024, 0.025)),
            origin=Origin(xyz=(PIVOT_X, y, 0.015)),
            material=dark_steel,
            name=f"yoke_upper_{side}",
        )
        carriage.visual(
            Box((0.054, 0.024, 0.025)),
            origin=Origin(xyz=(PIVOT_X, y, -0.055)),
            material=dark_steel,
            name=f"yoke_lower_{side}",
        )
        for post, x in enumerate((PIVOT_X - 0.029, PIVOT_X + 0.029)):
            carriage.visual(
                Box((0.010, 0.024, 0.095)),
                origin=Origin(xyz=(x, y, PIVOT_Z)),
                material=dark_steel,
                name=f"yoke_post_{side}_{post}",
            )
    carriage.visual(
        Cylinder(radius=0.008, length=0.014),
        origin=Origin(xyz=(0.0, -0.083, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="clamp_bolt",
    )

    table = model.part("table")
    table.visual(
        mesh_from_cadquery(_table_plate_shape(), "slotted_table_plate", tolerance=0.0008),
        origin=Origin(xyz=(TABLE_CENTER_X, 0.0, TABLE_CENTER_Z)),
        material=black_iron,
        name="table_plate",
    )

    # Bright T-track style rails and a rear fence rail seated on the slotted table.
    for idx, x in enumerate((0.165, 0.455)):
        table.visual(
            Box((0.024, 0.350, 0.010)),
            origin=Origin(xyz=(x, 0.0, TABLE_TOP_Z + 0.0045)),
            material=aluminum,
            name=f"t_track_{idx}",
        )
        for j, y in enumerate((-0.145, 0.145)):
            table.visual(
                Cylinder(radius=0.010, length=0.006),
                origin=Origin(xyz=(x, y, TABLE_TOP_Z + 0.012)),
                material=satin_steel,
                name=f"track_screw_{idx}_{j}",
            )

    table.visual(
        Box((0.500, 0.020, 0.012)),
        origin=Origin(xyz=(0.330, 0.142, TABLE_TOP_Z + 0.010)),
        material=aluminum,
        name="fence_rail",
    )
    table.visual(
        Box((0.145, 0.150, 0.012)),
        origin=Origin(xyz=(0.115, 0.0, TABLE_CENTER_Z - TABLE_THICKNESS / 2.0 - 0.006)),
        material=dark_steel,
        name="underside_plate",
    )
    table.visual(
        Box((0.112, 0.056, 0.052)),
        origin=Origin(xyz=(0.060, 0.0, 0.024)),
        material=dark_steel,
        name="saddle_block",
    )
    for idx, y in enumerate((-0.052, 0.052)):
        table.visual(
            Box((0.160, 0.015, 0.020)),
            origin=Origin(xyz=(0.115, y, 0.037), rpy=(0.0, -0.30, 0.0)),
            material=dark_steel,
            name=f"gusset_{idx}",
        )
    table.visual(
        Cylinder(radius=0.018, length=0.196),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="trunnion_shaft",
    )
    for idx, y in enumerate((-0.0945, 0.0945)):
        table.visual(
            Cylinder(radius=0.030, length=0.005),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=satin_steel,
            name=f"trunnion_washer_{idx}",
        )

    lever = model.part("locking_lever")
    lever.visual(
        Cylinder(radius=0.015, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="lever_hub",
    )
    lever.visual(
        Cylinder(radius=0.006, length=0.135),
        origin=Origin(xyz=(0.0, -0.074, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="lever_stem",
    )
    lever.visual(
        Sphere(radius=0.021),
        origin=Origin(xyz=(0.0, -0.155, 0.0)),
        material=knob_black,
        name="handle_ball",
    )

    model.articulation(
        "column_to_carriage",
        ArticulationType.PRISMATIC,
        parent=column,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, HEIGHT_REST_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-0.15, upper=0.18, effort=220.0, velocity=0.18),
    )
    model.articulation(
        "carriage_to_table",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=table,
        origin=Origin(xyz=(PIVOT_X, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.55, upper=0.55, effort=90.0, velocity=0.8),
    )
    model.articulation(
        "carriage_to_lever",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=lever,
        origin=Origin(xyz=(0.0, -0.104, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-1.20, upper=1.20, effort=8.0, velocity=2.5),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    column = object_model.get_part("column")
    carriage = object_model.get_part("carriage")
    table = object_model.get_part("table")
    lever = object_model.get_part("locking_lever")
    height = object_model.get_articulation("column_to_carriage")
    tilt = object_model.get_articulation("carriage_to_table")
    lever_joint = object_model.get_articulation("carriage_to_lever")

    ctx.check(
        "visible drill press table parts present",
        all(object_model.get_part(name) is not None for name in ("column", "carriage", "table", "locking_lever")),
        details="Expected column, sliding clamp carriage, slotted table, and locking lever.",
    )
    ctx.check(
        "three requested mechanisms authored",
        all(object_model.get_articulation(name) is not None for name in ("column_to_carriage", "carriage_to_table", "carriage_to_lever")),
        details="Expected height slide, table tilt, and lever rotation joints.",
    )

    ctx.expect_within(
        column,
        carriage,
        axes="xy",
        inner_elem="column_shaft",
        margin=0.010,
        name="split clamp jaws surround the column in plan",
    )
    ctx.expect_overlap(
        carriage,
        column,
        axes="z",
        elem_b="column_shaft",
        min_overlap=0.080,
        name="sliding clamp remains engaged on the column",
    )
    ctx.expect_overlap(
        table,
        carriage,
        axes="y",
        elem_a="trunnion_shaft",
        min_overlap=0.15,
        name="trunnion shaft spans the yoke hardware",
    )

    plate_aabb = ctx.part_element_world_aabb(table, elem="table_plate")
    if plate_aabb is not None:
        (mn, mx) = plate_aabb
        dx = mx[0] - mn[0]
        dy = mx[1] - mn[1]
        ctx.check(
            "rectangular table proportions preserved",
            dx > dy * 1.35 and dx > 0.55 and dy > 0.35,
            details=f"table plate size dx={dx:.3f}, dy={dy:.3f}",
        )

    rest_carriage = ctx.part_world_position(carriage)
    with ctx.pose({height: 0.16}):
        raised_carriage = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            column,
            axes="z",
            elem_b="column_shaft",
            min_overlap=0.080,
            name="raised collar still engaged on the column",
        )
    ctx.check(
        "height slide moves table upward",
        rest_carriage is not None and raised_carriage is not None and raised_carriage[2] > rest_carriage[2] + 0.12,
        details=f"rest={rest_carriage}, raised={raised_carriage}",
    )

    rest_plate = ctx.part_element_world_aabb(table, elem="table_plate")
    with ctx.pose({tilt: 0.40}):
        tilted_plate = ctx.part_element_world_aabb(table, elem="table_plate")
    if rest_plate is not None and tilted_plate is not None:
        ctx.check(
            "table tilt changes work surface angle",
            tilted_plate[0][2] < rest_plate[0][2] - 0.025,
            details=f"rest_min_z={rest_plate[0][2]:.3f}, tilted_min_z={tilted_plate[0][2]:.3f}",
        )

    rest_ball = ctx.part_element_world_aabb(lever, elem="handle_ball")
    with ctx.pose({lever_joint: 0.75}):
        moved_ball = ctx.part_element_world_aabb(lever, elem="handle_ball")
    if rest_ball is not None and moved_ball is not None:
        rest_center_z = (rest_ball[0][2] + rest_ball[1][2]) / 2.0
        moved_center_z = (moved_ball[0][2] + moved_ball[1][2]) / 2.0
        ctx.check(
            "locking lever rotates about clamp boss",
            moved_center_z < rest_center_z - 0.030,
            details=f"rest_ball_z={rest_center_z:.3f}, moved_ball_z={moved_center_z:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
