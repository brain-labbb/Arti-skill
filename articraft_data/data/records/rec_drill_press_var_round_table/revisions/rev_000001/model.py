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


# Round cast table geometry
TABLE_RADIUS = 0.20
TABLE_Y = -0.18  # disc center offset from table part origin (forward of column)
SUBSTRATE_BOTTOM = 0.030
SUBSTRATE_THICKNESS = 0.028
LAMINATE_THICKNESS = 0.004
LAMINATE_TOP = SUBSTRATE_BOTTOM + SUBSTRATE_THICKNESS + LAMINATE_THICKNESS


def _slot_cutter_y(x: float, y: float, length: float, width: float) -> cq.Workplane:
    """A vertical through-cutter for an XY rounded slot, with its long axis on Y."""
    straight = max(0.001, length - width)
    height = 0.45
    base_z = -0.18
    cutter = cq.Workplane("XY").rect(width, straight).extrude(height)
    cutter = cutter.union(cq.Workplane("XY").circle(width / 2.0).extrude(height).translate((0, -straight / 2.0, 0)))
    cutter = cutter.union(cq.Workplane("XY").circle(width / 2.0).extrude(height).translate((0, straight / 2.0, 0)))
    return cutter.translate((x, y, base_z))


def _drill_table_plate(thickness: float, z_center: float) -> cq.Workplane:
    """Round cast drill-press work table with central clearance hole, radial T-slots, and dog holes."""
    half_t = thickness / 2.0
    cut_depth = thickness + 0.020
    cut_z0 = -half_t - 0.010

    # Main circular disc
    disc = (
        cq.Workplane("XY")
        .circle(TABLE_RADIUS)
        .extrude(thickness)
        .translate((0.0, 0.0, -half_t))
    )

    # Central drill-bit clearance hole
    disc = disc.cut(
        cq.Workplane("XY")
        .circle(0.022)
        .extrude(cut_depth)
        .translate((0.0, 0.0, cut_z0))
    )

    # Four radial T-slots at 0°, 90°, 180°, 270°
    slot_width = 0.016
    slot_inner_r = 0.055
    slot_outer_r = TABLE_RADIUS - 0.015
    slot_length = slot_outer_r - slot_inner_r
    slot_mid_r = (slot_inner_r + slot_outer_r) / 2.0
    half_len = slot_length / 2.0
    slot_r = slot_width / 2.0

    for angle_deg in (0, 90, 180, 270):
        angle_rad = math.radians(angle_deg)
        # Build slot cutter along X at disc center origin
        cutter = (
            cq.Workplane("XY")
            .rect(slot_length, slot_width)
            .extrude(cut_depth)
            .translate((0.0, 0.0, cut_z0))
        )
        # Rounded ends
        cutter = cutter.union(
            cq.Workplane("XY")
            .circle(slot_r)
            .extrude(cut_depth)
            .translate((-half_len, 0.0, cut_z0))
        )
        cutter = cutter.union(
            cq.Workplane("XY")
            .circle(slot_r)
            .extrude(cut_depth)
            .translate((half_len, 0.0, cut_z0))
        )
        # Rotate about Z and translate to radial position
        dx = math.cos(angle_rad) * slot_mid_r
        dy = math.sin(angle_rad) * slot_mid_r
        cutter = cutter.rotate((0, 0, 0), (0, 0, 1), angle_deg).translate((dx, dy, 0.0))
        disc = disc.cut(cutter)

    # Dog holes at two radii, positioned between the radial slots (45°, 135°, 225°, 315°)
    dog_hole_r = 0.012
    for ring_r in (0.10, 0.16):
        for angle_deg in (45, 135, 225, 315):
            angle_rad = math.radians(angle_deg)
            hx = math.cos(angle_rad) * ring_r
            hy = math.sin(angle_rad) * ring_r
            disc = disc.cut(
                cq.Workplane("XY")
                .circle(dog_hole_r)
                .extrude(cut_depth)
                .translate((hx, hy, cut_z0))
            )

    # Translate finished disc to table-local position
    disc = disc.translate((0.0, TABLE_Y, z_center))
    return disc


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


def _table_underside_bracket() -> cq.Workplane:
    """Cast bracket connecting the trunnion shaft to the round table underside."""
    # Main web plate from trunnion area toward disc center
    bracket = cq.Workplane("XY").box(0.160, 0.190, 0.020).translate((0.0, -0.085, 0.015))
    # Trunnion saddle block (raised to clear carriage saddle_block)
    bracket = bracket.union(
        cq.Workplane("XY").box(0.065, 0.050, 0.020).translate((0.0, 0.0, 0.010))
    )
    # Cross rib under disc center for broad disc support
    bracket = bracket.union(
        cq.Workplane("XY").box(0.300, 0.025, 0.020).translate((0.0, TABLE_Y, SUBSTRATE_BOTTOM - 0.010))
    )
    return bracket


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="adjustable_drill_press_table",
        meta={"reference_note": "Reference content matches the Industrial / Drill press table category."},
    )

    cast_black = model.material("cast_black", rgba=(0.015, 0.014, 0.013, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.12, 0.12, 0.115, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.61, 0.56, 1.0))
    machined_iron = model.material("machined_iron", rgba=(0.38, 0.38, 0.36, 1.0))

    # ── Column (KEEP: base_plate, column_tube, base_flange) ──────────────
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

    # ── Carriage (KEEP: collar_rear, collar_side_0/1, saddle_block, yoke_arm_0/1) ──
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
        origin=Origin(xyz=(0.073, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="clamp_screw",
    )

    # ── Table (CHANGED: round cast disc replaces rectangular plate) ──────
    table = model.part("table")
    table.visual(
        mesh_from_cadquery(
            _drill_table_plate(SUBSTRATE_THICKNESS, SUBSTRATE_BOTTOM + SUBSTRATE_THICKNESS / 2.0),
            "cast_round_substrate",
        ),
        material=dark_steel,
        name="worktop_substrate",
    )
    table.visual(
        mesh_from_cadquery(
            _drill_table_plate(LAMINATE_THICKNESS, SUBSTRATE_BOTTOM + SUBSTRATE_THICKNESS + LAMINATE_THICKNESS / 2.0),
            "machined_round_face",
        ),
        material=machined_iron,
        name="worktop_laminate",
    )
    table.visual(
        mesh_from_cadquery(_table_underside_bracket(), "round_table_bracket"),
        material=cast_black,
        name="underside_bracket",
    )
    # KEEP: tilt_trunnion
    table.visual(
        Cylinder(radius=0.014, length=0.350),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="tilt_trunnion",
    )

    # ── Lock lever (KEEP) ────────────────────────────────────────────────
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

    # ── Articulations (KEEP: height_slide, table_tilt, lever_pivot) ──────
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
        details="Expected column, sliding carriage, round cast table, and locking lever.",
    )
    ctx.check(
        "primary motions present",
        all(object_model.get_articulation(name) is not None for name in ("height_slide", "table_tilt", "lever_pivot")),
        details="Expected height slide, table tilt, and lever pivot joints.",
    )

    # ── Variant-specific: circular worktop geometry claim ────────────────
    substrate_aabb = ctx.part_element_world_aabb(table, elem="worktop_substrate")
    if substrate_aabb is not None:
        dx = substrate_aabb[1][0] - substrate_aabb[0][0]
        dy = substrate_aabb[1][1] - substrate_aabb[0][1]
        ctx.check(
            "worktop_substrate is a circular disc (X and Y extents approximately equal)",
            abs(dx - dy) < 0.030,
            details=f"dx={dx:.4f}, dy={dy:.4f}, expected circular symmetry for round table variant",
        )
        ctx.check(
            "worktop_substrate diameter matches round table (~0.40m)",
            0.36 < dx < 0.44,
            details=f"diameter_x={dx:.4f}, expected ~2*TABLE_RADIUS=0.40m",
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
        negative_elem="worktop_laminate",
        min_gap=0.010,
        name="round work surface clears the column",
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

    rest_top = ctx.part_element_world_aabb(table, elem="worktop_laminate")
    with ctx.pose({table_tilt: 0.30}):
        tilted_top = ctx.part_element_world_aabb(table, elem="worktop_laminate")
    ctx.check(
        "positive tilt tips the round work surface about the trunnion",
        rest_top is not None and tilted_top is not None and tilted_top[0][2] < rest_top[0][2] - 0.020,
        details=f"rest_top={rest_top}, tilted_top={tilted_top}",
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
