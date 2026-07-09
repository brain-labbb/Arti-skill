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


TABLE_WIDTH = 0.62
TABLE_DEPTH = 0.42
TABLE_Y = -0.16
SLAB_THICKNESS = 0.038
SLAB_BOTTOM = 0.024
SLAB_TOP = SLAB_BOTTOM + SLAB_THICKNESS


def _slot_cutter_x(x: float, y: float, length: float, width: float) -> cq.Workplane:
    """A vertical through-cutter for an XY rounded slot, with its long axis on X."""
    straight = max(0.001, length - width)
    height = 0.45
    base_z = -0.18
    cutter = cq.Workplane("XY").rect(straight, width).extrude(height)
    cutter = cutter.union(cq.Workplane("XY").circle(width / 2.0).extrude(height).translate((-straight / 2.0, 0, 0)))
    cutter = cutter.union(cq.Workplane("XY").circle(width / 2.0).extrude(height).translate((straight / 2.0, 0, 0)))
    return cutter.translate((x, y, base_z))


def _slot_cutter_y(x: float, y: float, length: float, width: float) -> cq.Workplane:
    """A vertical through-cutter for an XY rounded slot, with its long axis on Y."""
    straight = max(0.001, length - width)
    height = 0.45
    base_z = -0.18
    cutter = cq.Workplane("XY").rect(width, straight).extrude(height)
    cutter = cutter.union(cq.Workplane("XY").circle(width / 2.0).extrude(height).translate((0, -straight / 2.0, 0)))
    cutter = cutter.union(cq.Workplane("XY").circle(width / 2.0).extrude(height).translate((0, straight / 2.0, 0)))
    return cutter.translate((x, y, base_z))


def _cast_iron_table_slab() -> cq.Workplane:
    """Single solid cast-iron table slab with through T-slots and chamfered top edge."""
    slab_t = SLAB_THICKNESS
    slab_top_z = SLAB_TOP
    slab_bot_z = SLAB_BOTTOM
    slab_mid_z = (slab_top_z + slab_bot_z) / 2.0

    # Main slab body — rounded rectangle, same footprint as parent.
    slab = (
        cq.Workplane("XY")
        .box(TABLE_WIDTH, TABLE_DEPTH, slab_t)
        .edges("|Z")
        .fillet(0.035)
        .translate((0.0, TABLE_Y, slab_mid_z))
    )

    # Chamfer top perimeter edges before slot cuts to preserve a clean outer profile.
    slab = slab.edges(">Z").chamfer(0.003)

    # Three milled through T-slots running in X (across the table width).
    # Profile: 14 mm top opening, 24 mm undercut channel below.
    slot_opening_w = 0.014
    slot_undercut_w = 0.024
    slot_opening_d = 0.008
    slot_undercut_d = 0.020
    slot_len = TABLE_WIDTH - 0.080

    for y_off in (-0.120, 0.000, 0.120):
        y_pos = TABLE_Y + y_off
        # Narrow top opening (from surface downward).
        nz = slab_top_z - slot_opening_d / 2.0
        slab = slab.cut(
            cq.Workplane("XY")
            .box(slot_len, slot_opening_w, slot_opening_d + 0.002)
            .translate((0.0, y_pos, nz))
        )
        # Wider undercut channel (T-profile base).
        uz = slab_top_z - slot_opening_d - slot_undercut_d / 2.0
        slab = slab.cut(
            cq.Workplane("XY")
            .box(slot_len, slot_undercut_w, slot_undercut_d + 0.002)
            .translate((0.0, y_pos, uz))
        )

    # Central drill-bit clearance hole (through).
    slab = slab.cut(
        cq.Workplane("XY")
        .circle(0.020)
        .extrude(slab_t + 0.020)
        .translate((0.0, TABLE_Y, slab_bot_z - 0.010))
    )

    # Dog holes for auxiliary clamping.
    for x, y_off, r in (
        (-0.245, -0.050, 0.010),
        (0.245, -0.050, 0.010),
        (-0.245, 0.080, 0.010),
        (0.245, 0.080, 0.010),
    ):
        slab = slab.cut(
            cq.Workplane("XY")
            .circle(r)
            .extrude(slab_t + 0.020)
            .translate((x, TABLE_Y + y_off, slab_bot_z - 0.010))
        )

    # Trunnion bearing lugs — cast blocks extending from the slab underside
    # down to the trunnion bore axis, tying the tilt shaft into the slab.
    lug_w = 0.040
    lug_d = 0.028
    lug_bot = -0.018
    lug_top = slab_bot_z + 0.002  # slight embed into slab bottom
    lug_h = lug_top - lug_bot
    lug_cz = (lug_top + lug_bot) / 2.0
    for x_off in (-0.040, 0.040):
        lug = (
            cq.Workplane("XY")
            .box(lug_w, lug_d, lug_h)
            .translate((x_off, 0.0, lug_cz))
        )
        slab = slab.union(lug)

    return slab


def _base_plate() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(0.36, 0.26, 0.030)
        .edges("|Z")
        .fillet(0.020)
        .translate((0.0, 0.0, 0.015))
    )
    for x in (-0.105, 0.105):
        base = base.cut(_slot_cutter_y(x, 0.045, 0.115, 0.022))
        base = base.cut(_slot_cutter_y(x, -0.085, 0.090, 0.020))
    return base


def _carriage_casting() -> cq.Workplane:
    """Sliding collar plus the forked trunnion support for the tilting table."""
    # A clearanced split collar, built from discrete jaws so the column remains
    # physically open rather than colliding with a simplified solid sleeve.
    collar = cq.Workplane("XY").box(0.160, 0.020, 0.070).translate((0.000, 0.056, 0.000))
    collar = collar.union(cq.Workplane("XY").box(0.020, 0.112, 0.070).translate((-0.056, 0.000, 0.000)))
    collar = collar.union(cq.Workplane("XY").box(0.020, 0.112, 0.070).translate((0.056, 0.000, 0.000)))
    collar = collar.union(cq.Workplane("XY").box(0.050, 0.020, 0.050).translate((0.077, 0.028, 0.000)))
    collar = collar.union(cq.Workplane("XY").box(0.050, 0.020, 0.050).translate((0.077, -0.028, 0.000)))

    # Cast saddle reaches forward from the column to the tilting fork.
    collar = collar.union(cq.Workplane("XY").box(0.160, 0.055, 0.050).translate((0.000, -0.062, 0.006)))
    collar = collar.union(cq.Workplane("XY").box(0.300, 0.030, 0.045).translate((0.000, -0.084, 0.030)))
    collar = collar.union(cq.Workplane("XY").box(0.024, 0.120, 0.040).translate((-0.158, -0.105, 0.038)))
    collar = collar.union(cq.Workplane("XY").box(0.024, 0.120, 0.040).translate((0.158, -0.105, 0.038)))

    return collar



def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="adjustable_drill_press_table",
        meta={"reference_note": "Reference content matches the Industrial / Drill press table category."},
    )

    cast_black = model.material("cast_black", rgba=(0.015, 0.014, 0.013, 1.0))
    cast_iron = model.material("cast_iron", rgba=(0.28, 0.27, 0.26, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.12, 0.12, 0.115, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.61, 0.56, 1.0))
    aluminum = model.material("aluminum", rgba=(0.74, 0.76, 0.74, 1.0))
    plywood = model.material("plywood_edge", rgba=(0.78, 0.58, 0.34, 1.0))
    screw_mat = model.material("screw_heads", rgba=(0.05, 0.06, 0.06, 1.0))

    column = model.part("column")
    column.visual(mesh_from_cadquery(_base_plate(), "slotted_base_plate"), material=cast_black, name="base_plate")
    column.visual(
        Cylinder(radius=0.038, length=0.86),
        origin=Origin(xyz=(0.0, 0.0, 0.455)),
        material=brushed_steel,
        name="column_tube",
    )
    column.visual(
        Cylinder(radius=0.070, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.049)),
        material=cast_black,
        name="base_flange",
    )
    for x, y in ((-0.045, -0.030), (0.045, -0.030), (-0.045, 0.030), (0.045, 0.030)):
        column.visual(
            Cylinder(radius=0.009, length=0.008),
            origin=Origin(xyz=(x, y, 0.070)),
            material=dark_steel,
            name=f"flange_bolt_{x:+.0e}_{y:+.0e}",
        )

    carriage = model.part("carriage")
    carriage.visual(Box((0.160, 0.020, 0.070)), origin=Origin(xyz=(0.000, 0.056, 0.000)), material=cast_black, name="collar_rear")
    carriage.visual(Box((0.020, 0.112, 0.070)), origin=Origin(xyz=(-0.056, 0.000, 0.000)), material=cast_black, name="collar_side_0")
    carriage.visual(Box((0.020, 0.112, 0.070)), origin=Origin(xyz=(0.056, 0.000, 0.000)), material=cast_black, name="collar_side_1")
    carriage.visual(Box((0.050, 0.020, 0.050)), origin=Origin(xyz=(0.077, 0.028, 0.000)), material=cast_black, name="clamp_ear_0")
    carriage.visual(Box((0.050, 0.020, 0.050)), origin=Origin(xyz=(0.077, -0.028, 0.000)), material=cast_black, name="clamp_ear_1")
    carriage.visual(Box((0.160, 0.055, 0.050)), origin=Origin(xyz=(0.000, -0.062, 0.006)), material=cast_black, name="saddle_block")
    carriage.visual(Box((0.085, 0.030, 0.045)), origin=Origin(xyz=(-0.120, -0.065, 0.030)), material=cast_black, name="yoke_rail_0")
    carriage.visual(Box((0.085, 0.030, 0.045)), origin=Origin(xyz=(0.120, -0.065, 0.030)), material=cast_black, name="yoke_rail_1")
    carriage.visual(Box((0.024, 0.120, 0.040)), origin=Origin(xyz=(-0.158, -0.105, 0.038)), material=cast_black, name="yoke_arm_0")
    carriage.visual(Box((0.024, 0.120, 0.040)), origin=Origin(xyz=(0.158, -0.105, 0.038)), material=cast_black, name="yoke_arm_1")
    carriage.visual(
        Cylinder(radius=0.008, length=0.070),
        origin=Origin(xyz=(0.073, 0.0, 0.000), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="clamp_screw",
    )
    table = model.part("table")
    table.visual(
        mesh_from_cadquery(_cast_iron_table_slab(), "cast_iron_t_slot_slab"),
        material=cast_iron,
        name="cast_table_slab",
    )
    table.visual(
        Cylinder(radius=0.014, length=0.350),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="tilt_trunnion",
    )

    # Two-piece fence with metal faces, plywood core exposed along the top, and visible screw heads.
    for suffix, x in (("0", -0.155), ("1", 0.155)):
        table.visual(
            Box((0.300, 0.024, 0.060)),
            origin=Origin(xyz=(x, TABLE_Y, SLAB_TOP + 0.030)),
            material=aluminum,
            name=f"fence_face_{suffix}",
        )
    table.visual(
        Box((0.610, 0.030, 0.012)),
        origin=Origin(xyz=(0.0, TABLE_Y, SLAB_TOP + 0.066)),
        material=plywood,
        name="fence_wood_cap",
    )
    table.visual(
        Box((0.570, 0.008, 0.006)),
        origin=Origin(xyz=(0.0, TABLE_Y - 0.010, SLAB_TOP + 0.075)),
        material=brushed_steel,
        name="fence_top_track",
    )
    table.visual(
        Box((0.560, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, TABLE_Y + 0.013, SLAB_TOP + 0.006)),
        material=brushed_steel,
        name="fence_lower_rail",
    )
    for i, x in enumerate((-0.245, -0.085, 0.085, 0.245)):
        table.visual(
            Cylinder(radius=0.010, length=0.010),
            origin=Origin(xyz=(x, TABLE_Y - 0.0130, SLAB_TOP + 0.030), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=screw_mat,
            name=f"fence_screw_{i}",
        )

    lever = model.part("lock_lever")
    lever.visual(
        Cylinder(radius=0.015, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="lever_hub",
    )
    lever.visual(
        Cylinder(radius=0.006, length=0.128),
        origin=Origin(xyz=(0.0, 0.0, -0.064)),
        material=dark_steel,
        name="lever_stem",
    )
    lever.visual(
        Sphere(radius=0.019),
        origin=Origin(xyz=(0.0, 0.0, -0.139)),
        material=cast_black,
        name="lever_knob",
    )

    model.articulation(
        "height_slide",
        ArticulationType.PRISMATIC,
        parent=column,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.380)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.18, lower=0.0, upper=0.180),
    )
    model.articulation(
        "table_tilt",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=table,
        origin=Origin(xyz=(0.0, -0.105, 0.038)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.7, lower=-0.45, upper=0.45),
    )
    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=lever,
        origin=Origin(xyz=(0.0790, 0.000, -0.034)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=-1.2, upper=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    column = object_model.get_part("column")
    carriage = object_model.get_part("carriage")
    table = object_model.get_part("table")
    lever = object_model.get_part("lock_lever")
    height_slide = object_model.get_articulation("height_slide")
    table_tilt = object_model.get_articulation("table_tilt")
    lever_pivot = object_model.get_articulation("lever_pivot")

    ctx.allow_overlap(
        table,
        carriage,
        elem_a="tilt_trunnion",
        elem_b="yoke_arm_0",
        reason="The trunnion shaft is intentionally captured through the forked yoke bearing.",
    )
    ctx.allow_overlap(
        table,
        carriage,
        elem_a="tilt_trunnion",
        elem_b="yoke_arm_1",
        reason="The trunnion shaft is intentionally captured through the forked yoke bearing.",
    )

    ctx.check(
        "drill press table mechanism parts present",
        all(object_model.get_part(name) is not None for name in ("column", "carriage", "table", "lock_lever")),
        details="Expected column, sliding carriage, slotted table, and locking lever.",
    )
    ctx.check(
        "primary motions present",
        all(object_model.get_articulation(name) is not None for name in ("height_slide", "table_tilt", "lever_pivot")),
        details="Expected height slide, table tilt, and lever pivot joints.",
    )

    ctx.expect_gap(
        column,
        carriage,
        axis="x",
        positive_elem="column_tube",
        negative_elem="collar_side_0",
        min_gap=0.004,
        max_gap=0.014,
        name="left collar jaw clears the column",
    )
    ctx.expect_gap(
        carriage,
        column,
        axis="x",
        positive_elem="collar_side_1",
        negative_elem="column_tube",
        min_gap=0.004,
        max_gap=0.014,
        name="right collar jaw clears the column",
    )
    ctx.expect_overlap(
        column,
        carriage,
        axes="z",
        elem_a="column_tube",
        elem_b="collar_rear",
        min_overlap=0.060,
        name="column remains engaged through the clamp collar",
    )
    ctx.expect_overlap(
        table,
        carriage,
        axes="xyz",
        elem_a="tilt_trunnion",
        elem_b="yoke_arm_0",
        min_overlap=0.010,
        name="trunnion is captured by one fork cheek",
    )
    ctx.expect_overlap(
        table,
        carriage,
        axes="xyz",
        elem_a="tilt_trunnion",
        elem_b="yoke_arm_1",
        min_overlap=0.010,
        name="trunnion is captured by opposite fork cheek",
    )
    ctx.expect_gap(
        column,
        table,
        axis="y",
        positive_elem="column_tube",
        negative_elem="cast_table_slab",
        min_gap=0.010,
        name="work surface sits forward of the column",
    )

    rest_table_pos = ctx.part_world_position(table)
    with ctx.pose({height_slide: 0.160}):
        raised_table_pos = ctx.part_world_position(table)
        ctx.expect_gap(
            column,
            carriage,
            axis="x",
            positive_elem="column_tube",
            negative_elem="collar_side_0",
            min_gap=0.004,
            max_gap=0.014,
            name="raised left collar jaw clears column",
        )
        ctx.expect_gap(
            carriage,
            column,
            axis="x",
            positive_elem="collar_side_1",
            negative_elem="column_tube",
            min_gap=0.004,
            max_gap=0.014,
            name="raised right collar jaw clears column",
        )
        ctx.expect_overlap(
            column,
            carriage,
            axes="z",
            elem_a="column_tube",
            elem_b="collar_rear",
            min_overlap=0.060,
            name="raised collar remains retained on column",
        )
    ctx.check(
        "height slide raises table along column",
        rest_table_pos is not None and raised_table_pos is not None and raised_table_pos[2] > rest_table_pos[2] + 0.145,
        details=f"rest={rest_table_pos}, raised={raised_table_pos}",
    )

    rest_top = ctx.part_element_world_aabb(table, elem="cast_table_slab")
    with ctx.pose({table_tilt: 0.30}):
        tilted_top = ctx.part_element_world_aabb(table, elem="cast_table_slab")
    ctx.check(
        "positive tilt tips the work surface about the trunnion",
        rest_top is not None and tilted_top is not None and tilted_top[0][2] < rest_top[0][2] - 0.020,
        details=f"rest_top={rest_top}, tilted_top={tilted_top}",
    )

    # Variant-specific: cast-iron slab must be a single solid plate of substantial
    # thickness carrying the T-slots, replacing the old plywood+laminate sandwich.
    slab_dims = ctx.part_element_world_aabb(table, elem="cast_table_slab")
    ctx.check(
        "cast_table_slab is a solid cast-iron plate of substantial thickness",
        slab_dims is not None
        and (slab_dims[1][2] - slab_dims[0][2]) > 0.034,
        details=f"cast_table_slab AABB depth (z) must exceed 34 mm (got {slab_dims})",
    )
    table_visual_names = {v.name for v in table.visuals}
    ctx.check(
        "cast_table_slab replaces plywood and laminate layers",
        "worktop_substrate" not in table_visual_names
        and "worktop_laminate" not in table_visual_names
        and "cast_table_slab" in table_visual_names,
        details="The variant must have a single cast_table_slab, not the old plywood/laminate layers.",
    )

    rest_lever = ctx.part_world_aabb(lever)
    with ctx.pose({lever_pivot: 0.90}):
        rotated_lever = ctx.part_world_aabb(lever)
    ctx.check(
        "locking lever rotates about its clamp hub",
        rest_lever is not None
        and rotated_lever is not None
        and abs(rotated_lever[1][1] - rest_lever[1][1]) > 0.080,
        details=f"rest_lever={rest_lever}, rotated_lever={rotated_lever}",
    )

    return ctx.report()


object_model = build_object_model()
