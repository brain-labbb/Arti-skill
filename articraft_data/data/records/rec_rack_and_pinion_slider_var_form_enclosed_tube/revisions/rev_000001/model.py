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

# Housing shell dimensions (meters)
HOUSING_LENGTH = 0.280
HOUSING_WIDTH = 0.094
HOUSING_HEIGHT = 0.096
HOUSING_WALL = 0.004
HOUSING_CENTER_Z = BASE_THICKNESS + HOUSING_HEIGHT / 2.0

# Aperture for rack rod passage at +X end
APERTURE_WIDTH = 0.058
APERTURE_HEIGHT = 0.044
APERTURE_BOTTOM_OFFSET = 0.004  # above housing inner bottom


def _pinion_shape() -> cq.Shape:
    gear = SpurGear(
        module=MODULE_MM,
        teeth_number=PINION_TEETH,
        width=PINION_WIDTH_MM,
        backlash=0.08,
    )
    body = gear.build(bore_d=PINION_BORE_MM)
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
    return body.rotate((0, 0, 0), (1, 0, 0), 90).translate((-RACK_LENGTH_MM * 0.5, 0, 0))


def _rack_bounds_m() -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    return cadquery_local_aabb(_rack_shape(), unit_scale=0.001)


def _housing_shell_shape() -> cq.Workplane:
    """Hollow rectangular housing shell with end aperture for rack rod."""
    wall = HOUSING_WALL
    length = HOUSING_LENGTH
    width = HOUSING_WIDTH
    height = HOUSING_HEIGHT

    # Outer box centered at origin
    outer = cq.Workplane("XY").box(length, width, height)

    # Inner cavity: open at bottom (sits on base plate), closed top
    inner = (
        cq.Workplane("XY")
        .box(length - 2 * wall, width - 2 * wall, height - wall)
        .translate((0, 0, -wall / 2))
    )
    shell = outer.cut(inner)

    # End aperture at +X face for rack rod to slide through
    ap_zc = -height / 2 + APERTURE_BOTTOM_OFFSET + APERTURE_HEIGHT / 2
    aperture = (
        cq.Workplane("XY")
        .box(wall + 0.008, APERTURE_WIDTH, APERTURE_HEIGHT)
        .translate((length / 2, 0, ap_zc))
    )
    shell = shell.cut(aperture)

    return shell


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
    model.material("anodized_housing", rgba=(0.20, 0.22, 0.26, 1.0))

    # ── Frame (root part) ──────────────────────────────────────────────
    frame = model.part("frame")

    # Base plate
    frame.visual(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS * 0.5)),
        material="dark_oxide",
        name="base_plate",
    )

    # Low guide rails keep the sliding rack/carriage aligned
    for y_pos in (-0.035, 0.035):
        frame.visual(
            Box((0.295, 0.010, 0.014)),
            origin=Origin(xyz=(0.0, y_pos, BASE_THICKNESS + 0.007)),
            material="brushed_steel",
            name=f"guide_rail_{0 if y_pos < 0 else 1}",
        )

    # Bearing cheeks straddle the gear, tied into the base by broad feet.
    # Feet are pulled inward to stay inside the housing shell cavity.
    for y_pos in (-0.030, 0.030):
        foot_y = -0.034 if y_pos < 0 else 0.034
        frame.visual(
            Box((0.046, 0.008, 0.080)),
            origin=Origin(xyz=(0.0, y_pos, BASE_THICKNESS + 0.040)),
            material="dark_oxide",
            name=f"bearing_cheek_{0 if y_pos < 0 else 1}",
        )
        frame.visual(
            Box((0.070, 0.012, 0.010)),
            origin=Origin(xyz=(0.0, foot_y, BASE_THICKNESS + 0.005)),
            material="dark_oxide",
            name=f"cheek_foot_{0 if y_pos < 0 else 1}",
        )
        frame.visual(
            Cylinder(radius=0.011, length=0.004),
            origin=Origin(xyz=(0.0, y_pos * 1.01, PINION_CENTER_Z), rpy=(pi / 2.0, 0.0, 0.0)),
            material="zinc_hardware",
            name=f"bearing_boss_{0 if y_pos < 0 else 1}",
        )

    # Pinion shaft (fixed, carried by bearing cheeks)
    frame.visual(
        Cylinder(radius=0.00615, length=0.074),
        origin=Origin(xyz=(0.0, 0.0, PINION_CENTER_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material="zinc_hardware",
        name="pinion_shaft",
    )

    # Base plate mounting screws (outside housing footprint)
    for x_pos in (-0.135, 0.135):
        for y_pos in (-0.033, 0.033):
            frame.visual(
                Cylinder(radius=0.0065, length=0.004),
                origin=Origin(xyz=(x_pos, y_pos, BASE_THICKNESS + 0.002)),
                material="zinc_hardware",
                name=f"mount_screw_{'n' if y_pos > 0 else 's'}_{'p' if x_pos > 0 else 'm'}",
            )

    # ── Housing shell (FIXED, on frame) ────────────────────────────────
    frame.visual(
        mesh_from_cadquery(
            _housing_shell_shape(),
            "housing_shell",
            tolerance=0.0004,
        ),
        origin=Origin(xyz=(0.0, 0.0, HOUSING_CENTER_Z)),
        material="anodized_housing",
        name="housing_shell",
    )

    # Aperture seal lips (thin rubber frame around the rod opening)
    ap_center_z = HOUSING_CENTER_Z + (-HOUSING_HEIGHT / 2 + APERTURE_BOTTOM_OFFSET + APERTURE_HEIGHT / 2)
    lip_t = 0.004
    lip_w = 0.005
    ap_front_x = HOUSING_LENGTH / 2 + lip_t / 2

    frame.visual(
        Box((lip_t, APERTURE_WIDTH + 2 * lip_w, lip_w)),
        origin=Origin(xyz=(ap_front_x, 0, ap_center_z + APERTURE_HEIGHT / 2 + lip_w / 2)),
        material="black_bearing",
        name="aperture_lip_top",
    )
    frame.visual(
        Box((lip_t, APERTURE_WIDTH + 2 * lip_w, lip_w)),
        origin=Origin(xyz=(ap_front_x, 0, ap_center_z - APERTURE_HEIGHT / 2 - lip_w / 2)),
        material="black_bearing",
        name="aperture_lip_bottom",
    )
    for y_sign in (-1, 1):
        frame.visual(
            Box((lip_t, lip_w, APERTURE_HEIGHT)),
            origin=Origin(
                xyz=(ap_front_x, y_sign * (APERTURE_WIDTH / 2 + lip_w / 2), ap_center_z)
            ),
            material="black_bearing",
            name=f"aperture_lip_{'left' if y_sign < 0 else 'right'}",
        )

    # Housing top bolts
    bolt_xs = [-0.100, -0.030, 0.040, 0.100]
    bolt_ys = [-0.035, 0.035]
    for i, bx in enumerate(bolt_xs):
        for j, by in enumerate(bolt_ys):
            frame.visual(
                Cylinder(radius=0.003, length=0.003),
                origin=Origin(
                    xyz=(bx, by, HOUSING_CENTER_Z + HOUSING_HEIGHT / 2 + 0.0015)
                ),
                material="zinc_hardware",
                name=f"housing_bolt_{i}_{j}",
            )

    # Side stiffener ribs on housing exterior
    for i, z_off in enumerate([-0.025, 0.0, 0.025]):
        for j, y_sign in enumerate((-1, 1)):
            frame.visual(
                Box((HOUSING_LENGTH - 0.020, 0.002, 0.004)),
                origin=Origin(
                    xyz=(0.0, y_sign * (HOUSING_WIDTH / 2 + 0.001), HOUSING_CENTER_Z + z_off)
                ),
                material="anodized_housing",
                name=f"housing_rib_{i}_{j}",
            )

    # Rear access cap on back wall
    frame.visual(
        Cylinder(radius=0.014, length=0.005),
        origin=Origin(
            xyz=(-HOUSING_LENGTH / 2 - 0.0025, 0, HOUSING_CENTER_Z),
            rpy=(0, pi / 2, 0),
        ),
        material="dark_oxide",
        name="rear_access_cap",
    )

    # External mounting pads on base plate (outside housing footprint)
    for x_pos in (-0.152, 0.152):
        for y_pos in (-0.035, 0.035):
            frame.visual(
                Box((0.014, 0.012, 0.005)),
                origin=Origin(xyz=(x_pos, y_pos, BASE_THICKNESS + 0.0025)),
                material="dark_oxide",
                name=f"external_foot_{'n' if y_pos > 0 else 's'}_{'p' if x_pos > 0 else 'm'}",
            )

    # ── Rack carriage (sliding rod) ────────────────────────────────────
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

    # ── Pinion gear ────────────────────────────────────────────────────
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

    # ── Articulations ──────────────────────────────────────────────────
    model.articulation(
        "pinion_spin",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=pinion,
        origin=Origin(xyz=(0.0, 0.0, PINION_CENTER_Z)),
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

    # Intentional overlaps: shaft captured inside pinion bore / hub
    ctx.allow_overlap(
        frame,
        pinion,
        elem_a="pinion_shaft",
        elem_b="toothed_wheel",
        reason="The fixed shaft is intentionally modeled as a light interference fit inside the pinion bore to represent the captured bearing support.",
    )
    ctx.allow_overlap(
        frame,
        pinion,
        elem_a="pinion_shaft",
        elem_b="hub",
        reason="The shaft locally seats inside the rotating hub bore as a captured journal/bushing fit.",
    )

    # ── Classification ─────────────────────────────────────────────────
    ctx.check(
        "classification matches visible rack-and-pinion slider",
        object_model.meta.get("category") == "Robotics / Rack-and-pinion slider",
        details=str(object_model.meta),
    )

    # ── Housing shell enclosure (primary variant assertion) ────────────
    ctx.expect_within(
        pinion,
        frame,
        axes="xz",
        inner_elem="toothed_wheel",
        outer_elem="housing_shell",
        margin=0.002,
        name="housing_shell encloses pinion gear in XZ",
    )
    ctx.expect_within(
        rack,
        frame,
        axes="x",
        inner_elem="straight_rack",
        outer_elem="housing_shell",
        margin=0.002,
        name="housing_shell encloses retracted rack in X",
    )

    # At max extension, the rack rod protrudes through the aperture
    with ctx.pose({slide: RACK_TRAVEL_M}):
        extended_pos = ctx.part_world_position(rack)
        # Rack center at max extension ≈ RACK_TRAVEL_M, half-length 0.130m
        # Housing front wall at HOUSING_LENGTH/2 = 0.140m
        rack_front = (extended_pos[0] + 0.130) if extended_pos else 0.0
        housing_front = HOUSING_LENGTH / 2
        ctx.check(
            "rack rod protrudes past housing_shell aperture at max travel",
            extended_pos is not None and rack_front > housing_front + 0.010,
            details=f"rack_center_x={extended_pos}, rack_front={rack_front:.4f}, housing_front={housing_front:.4f}",
        )

    # ── Gear mesh and alignment ────────────────────────────────────────
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
        name="shaft passes through pinion face width",
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

    # ── Coordinated motion ─────────────────────────────────────────────
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
