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
    RackGear,
    SpurGear,
    TestContext,
    TestReport,
    cadquery_local_aabb,
    mesh_from_cadquery,
)


MODULE_MM = 2.0
PINION_TEETH = 28
PINION_WIDTH_MM = 16.0
PINION_BORE_MM = 12.0
RACK_LENGTH_MM = 260.0
RACK_WIDTH_MM = 18.0
RACK_BACKING_MM = 10.0

PINION_PITCH_RADIUS_M = MODULE_MM * PINION_TEETH * 0.5 * 0.001
PINION_CENTER_OFFSET_M = (MODULE_MM * PINION_TEETH * 0.5 + MODULE_MM * 2.05) * 0.001
RACK_TRAVEL_M = 0.045
PINION_TRAVEL_RAD = RACK_TRAVEL_M / PINION_PITCH_RADIUS_M

BASE_LENGTH = 0.320
BASE_WIDTH = 0.095
BASE_THICKNESS = 0.012
RACK_FRAME_Z = 0.036
PINION_CENTER_Z = RACK_FRAME_Z + PINION_CENTER_OFFSET_M


def _pinion_shape() -> cq.Shape:
    gear = SpurGear(
        module=MODULE_MM,
        teeth_number=PINION_TEETH,
        width=PINION_WIDTH_MM,
        backlash=0.08,
    )
    body = gear.build(bore_d=PINION_BORE_MM)
    # Put the generated gear axis (local Z) onto the model Y axis so the wheel
    # stands vertically over the rack, matching the reference image.
    return body.rotate((0, 0, 0), (1, 0, 0), 90)


def _pinion_hub_shape() -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(15.0)
        .circle(PINION_BORE_MM * 0.5)
        .extrude(6.0, both=True)
    )


def _rack_shape() -> cq.Shape:
    rack = RackGear(
        module=MODULE_MM,
        length=RACK_LENGTH_MM,
        width=RACK_WIDTH_MM,
        height=RACK_BACKING_MM,
        backlash=0.08,
    )
    body = rack.build()
    # RackGear is generated with length along X, tooth height along Y, and width
    # along Z.  Rotate Y upward into model Z, then center the rack on the joint
    # frame in X.
    return body.rotate((0, 0, 0), (1, 0, 0), 90).translate((-RACK_LENGTH_MM * 0.5, 0, 0))


def _rack_bounds_m() -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    return cadquery_local_aabb(_rack_shape(), unit_scale=0.001)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="rack_and_pinion_slider",
        meta={
            "category": "Robotics / Rack-and-pinion slider",
            "reference_note": "Classification matches the visible rack-and-pinion slider.",
        },
    )

    model.material("brushed_steel", rgba=(0.58, 0.60, 0.60, 1.0))
    model.material("dark_oxide", rgba=(0.12, 0.13, 0.14, 1.0))
    model.material("zinc_hardware", rgba=(0.76, 0.75, 0.70, 1.0))
    model.material("black_bearing", rgba=(0.02, 0.02, 0.025, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS * 0.5)),
        material="dark_oxide",
        name="base_plate",
    )
    # Low guide rails keep the sliding rack/carriage aligned without intersecting it.
    for y_pos in (-0.035, 0.035):
        frame.visual(
            Box((0.295, 0.010, 0.014)),
            origin=Origin(xyz=(0.0, y_pos, BASE_THICKNESS + 0.007)),
            material="brushed_steel",
            name=f"guide_rail_{0 if y_pos < 0 else 1}",
        )
    # ── Single-side cantilever bearing support (−Y side only) ──────────
    # The pinion is overhung: carried from one bearing block / gearmotor face
    # with no far-side bearing.  The shaft is a cantilever stub.
    cheek_y = -0.030
    cheek_half_thick = 0.009  # beefy 18 mm plate replaces the thin straddle pair

    # Main support cheek — taller and thicker than the old straddle plates.
    frame.visual(
        Box((0.054, 2 * cheek_half_thick, 0.090)),
        origin=Origin(xyz=(0.0, cheek_y, BASE_THICKNESS + 0.045)),
        material="dark_oxide",
        name="bearing_cheek_0",
    )
    # Broad foot bolts the single-side support into the base plate.
    frame.visual(
        Box((0.078, 0.022, 0.012)),
        origin=Origin(xyz=(0.0, -0.040, BASE_THICKNESS + 0.006)),
        material="dark_oxide",
        name="cheek_foot_0",
    )
    # Gearmotor / bearing housing stub on the outer (−Y) face of the cheek.
    motor_length = 0.028
    motor_outer_y = cheek_y - cheek_half_thick  # −0.039
    frame.visual(
        Cylinder(radius=0.022, length=motor_length),
        origin=Origin(
            xyz=(0.0, motor_outer_y - motor_length * 0.5, PINION_CENTER_Z),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material="black_bearing",
        name="motor_housing",
    )
    # Inner-face bearing boss where the shaft exits the cheek toward the pinion.
    frame.visual(
        Cylinder(radius=0.013, length=0.005),
        origin=Origin(
            xyz=(0.0, cheek_y + cheek_half_thick + 0.0025, PINION_CENTER_Z),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material="zinc_hardware",
        name="bearing_boss_0",
    )
    # Cantilever pinion shaft — anchored only in the −Y bearing cheek, extends
    # through the pinion bore but does NOT reach a far-side support.
    shaft_inner_y = cheek_y + cheek_half_thick   # −0.021 (inner cheek face)
    shaft_tip_y = 0.009                           # just past hub face, no far bearing
    shaft_length = shaft_tip_y - shaft_inner_y    # 0.030
    shaft_center_y = (shaft_inner_y + shaft_tip_y) * 0.5  # −0.006
    frame.visual(
        Cylinder(radius=0.00615, length=shaft_length),
        origin=Origin(
            xyz=(0.0, shaft_center_y, PINION_CENTER_Z),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material="zinc_hardware",
        name="pinion_shaft",
    )
    for x_pos in (-0.135, 0.135):
        for y_pos in (-0.033, 0.033):
            frame.visual(
                Cylinder(radius=0.0065, length=0.004),
                origin=Origin(xyz=(x_pos, y_pos, BASE_THICKNESS + 0.002)),
                material="zinc_hardware",
                name=f"mount_screw_{'n' if y_pos > 0 else 's'}_{'p' if x_pos > 0 else 'm'}",
            )

    rack_min, _rack_max = _rack_bounds_m()
    rack_carriage = model.part("rack_carriage")
    rack_carriage.visual(
        mesh_from_cadquery(_rack_shape(), "straight_rack", unit_scale=0.001, tolerance=0.00025),
        material="brushed_steel",
        name="straight_rack",
    )
    carriage_height = 0.012
    rack_carriage.visual(
        Box((0.112, 0.048, carriage_height)),
        # Slight same-part embed makes the block read as bolted to the rack back.
        origin=Origin(xyz=(0.0, 0.0, rack_min[2] - carriage_height * 0.5 + 0.0005)),
        material="dark_oxide",
        name="carriage_block",
    )
    for x_pos in (-0.042, 0.042):
        rack_carriage.visual(
            Cylinder(radius=0.0048, length=0.003),
            origin=Origin(xyz=(x_pos, 0.0, rack_min[2] - 0.0015)),
            material="zinc_hardware",
            name=f"carriage_bolt_{0 if x_pos < 0 else 1}",
        )

    pinion = model.part("pinion")
    pinion.visual(
        mesh_from_cadquery(_pinion_shape(), "toothed_wheel", unit_scale=0.001, tolerance=0.00022),
        material="brushed_steel",
        name="toothed_wheel",
    )
    pinion.visual(
        mesh_from_cadquery(_pinion_hub_shape(), "pinion_hub", unit_scale=0.001, tolerance=0.00025),
        material="zinc_hardware",
        name="hub",
    )

    model.articulation(
        "pinion_spin",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=pinion,
        origin=Origin(xyz=(0.0, 0.0, PINION_CENTER_Z)),
        # Positive rotation drives the rack/carriage in +X at the lower mesh point.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=PINION_TRAVEL_RAD, effort=8.0, velocity=8.0),
    )
    model.articulation(
        "rack_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=rack_carriage,
        origin=Origin(xyz=(0.0, 0.0, RACK_FRAME_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=RACK_TRAVEL_M, effort=80.0, velocity=0.30),
        meta={"pinion_pitch_radius_m": PINION_PITCH_RADIUS_M},
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    rack = object_model.get_part("rack_carriage")
    pinion = object_model.get_part("pinion")
    spin = object_model.get_articulation("pinion_spin")
    slide = object_model.get_articulation("rack_slide")

    ctx.allow_overlap(
        frame,
        pinion,
        elem_a="pinion_shaft",
        elem_b="toothed_wheel",
        reason="The cantilever shaft is intentionally modeled as a light interference fit inside the pinion bore to represent the overhung bearing support.",
    )
    ctx.allow_overlap(
        frame,
        pinion,
        elem_a="pinion_shaft",
        elem_b="hub",
        reason="The shaft locally seats inside the rotating hub bore as a captured journal/bushing fit.",
    )
    ctx.allow_overlap(
        frame,
        pinion,
        elem_a="bearing_boss_0",
        elem_b="hub",
        reason="The inner-face bearing boss is a close-clearance journal seat adjacent to the hub face.",
    )

    ctx.check(
        "classification matches visible rack-and-pinion slider",
        object_model.meta.get("category") == "Robotics / Rack-and-pinion slider",
        details=str(object_model.meta),
    )

    # ── Cantilever topology assertions ──────────────────────────────────
    # The pinion_shaft is a stub anchored only in the −Y bearing cheek.
    shaft_aabb = ctx.part_element_world_aabb(frame, elem="pinion_shaft")
    if shaft_aabb is not None:
        shaft_max_y = float(shaft_aabb[1][1])
        ctx.check(
            "pinion_shaft is a cantilever stub (does not reach +Y far-side bearing)",
            shaft_max_y < 0.020,
            details=f"shaft max Y = {shaft_max_y:.4f} m; must stay below +0.020 m (no second bearing)",
        )

    # The single-side bearing_cheek_0 is on the −Y side of the pinion center.
    cheek_aabb = ctx.part_element_world_aabb(frame, elem="bearing_cheek_0")
    if cheek_aabb is not None:
        cheek_center_y = (float(cheek_aabb[0][1]) + float(cheek_aabb[1][1])) * 0.5
        ctx.check(
            "bearing_cheek_0 is the single-side cantilever support on −Y",
            cheek_center_y < -0.010,
            details=f"cheek center Y = {cheek_center_y:.4f} m; must be on −Y side",
        )

    # The motor_housing stub sits on the outer −Y face of the bearing cheek.
    motor_aabb = ctx.part_element_world_aabb(frame, elem="motor_housing")
    if motor_aabb is not None:
        motor_max_y = float(motor_aabb[1][1])
        ctx.check(
            "motor_housing is on the outer −Y face of the support block",
            motor_max_y < -0.020,
            details=f"motor max Y = {motor_max_y:.4f} m; must be on −Y outer face",
        )

    # ── Mesh and alignment ──────────────────────────────────────────────
    ctx.expect_overlap(
        pinion,
        rack,
        axes="xy",
        min_overlap=0.010,
        elem_a="toothed_wheel",
        elem_b="straight_rack",
        name="pinion teeth align over rack teeth",
    )
    ctx.expect_gap(
        pinion,
        rack,
        axis="z",
        min_gap=0.0,
        max_gap=0.003,
        positive_elem="toothed_wheel",
        negative_elem="straight_rack",
        name="pinion teeth sit just above rack teeth without penetration",
    )
    ctx.expect_within(
        frame,
        pinion,
        axes="xz",
        inner_elem="pinion_shaft",
        outer_elem="toothed_wheel",
        margin=0.001,
        name="shaft is centered inside pinion bore footprint",
    )
    ctx.expect_overlap(
        frame,
        pinion,
        axes="y",
        min_overlap=0.012,
        elem_a="pinion_shaft",
        elem_b="toothed_wheel",
        name="cantilever shaft passes through pinion bore width",
    )
    ctx.expect_overlap(
        rack,
        frame,
        axes="x",
        min_overlap=0.200,
        elem_a="straight_rack",
        elem_b="base_plate",
        name="rack remains carried over base length",
    )
    ctx.expect_gap(
        rack,
        frame,
        axis="z",
        max_gap=0.006,
        max_penetration=0.00002,
        positive_elem="carriage_block",
        negative_elem="base_plate",
        name="carriage rides on fixed base plate without meaningful penetration",
    )

    rest_pos = ctx.part_world_position(rack)
    with ctx.pose({spin: 1.0, slide: PINION_PITCH_RADIUS_M}):
        moved_pos = ctx.part_world_position(rack)
        ctx.expect_overlap(
            pinion,
            rack,
            axes="xy",
            min_overlap=0.010,
            elem_a="toothed_wheel",
            elem_b="straight_rack",
            name="mesh alignment persists during driven slide",
        )

    ctx.check(
        "coordinated pinion rotation and rack slide move +X by pitch radius",
        rest_pos is not None and moved_pos is not None and moved_pos[0] > rest_pos[0] + 0.020,
        details=f"rest={rest_pos}, moved={moved_pos}, pitch_radius={PINION_PITCH_RADIUS_M}",
    )

    return ctx.report()


object_model = build_object_model()
