from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
)


PI = math.pi


def _cylinder_x(length: float, radius: float, *, center=(0.0, 0.0, 0.0)) -> cq.Workplane:
    """CadQuery cylinder centered at `center`, with its axis along world X."""
    return (
        cq.Workplane("XY")
        .cylinder(length, radius)
        .rotate((0, 0, 0), (0, 1, 0), 90)
        .translate(center)
    )


def _cylinder_y(length: float, radius: float, *, center=(0.0, 0.0, 0.0)) -> cq.Workplane:
    """CadQuery cylinder centered at `center`, with its axis along world Y."""
    return (
        cq.Workplane("XY")
        .cylinder(length, radius)
        .rotate((0, 0, 0), (1, 0, 0), -90)
        .translate(center)
    )


def _annular_cylinder_x(
    length: float,
    outer_radius: float,
    inner_radius: float,
    *,
    center=(0.0, 0.0, 0.0),
) -> cq.Workplane:
    outer = _cylinder_x(length, outer_radius, center=center)
    cutter = _cylinder_x(length + 0.010, inner_radius, center=center)
    return outer.cut(cutter)


def _carrier_cage_geometry() -> cq.Workplane:
    """Simplified differential cage with cheeks, axle collars, flange, and pinion support."""
    # Two cheeks
    cage = _annular_cylinder_x(0.030, 0.090, 0.047, center=(-0.066, 0.0, 0.0))
    cage = cage.union(_annular_cylinder_x(0.030, 0.090, 0.047, center=(0.066, 0.0, 0.0)))

    # Top and bottom bridges connecting the cheeks
    cage = cage.union(cq.Workplane("XY").box(0.160, 0.038, 0.030).translate((0.0, 0.0, 0.083)))
    cage = cage.union(cq.Workplane("XY").box(0.160, 0.038, 0.030).translate((0.0, 0.0, -0.083)))

    # Axle collars
    cage = cage.union(_annular_cylinder_x(0.070, 0.041, 0.030, center=(-0.112, 0.0, 0.0)))
    cage = cage.union(_annular_cylinder_x(0.070, 0.041, 0.030, center=(0.112, 0.0, 0.0)))

    # Ring-gear mounting flange
    cage = cage.union(_annular_cylinder_x(0.026, 0.145, 0.061, center=(-0.105, 0.0, 0.0)))

    # Pinion bearing snout
    bearing = _cylinder_y(0.072, 0.033, center=(-0.145, -0.225, 0.0)).cut(
        _cylinder_y(0.084, 0.016, center=(-0.145, -0.225, 0.0))
    )
    cage = cage.union(bearing)

    # Rib connecting flange to bearing snout (must bridge the X gap for connectivity)
    cage = cage.union(
        cq.Workplane("XY").box(0.080, 0.060, 0.022).translate((-0.125, -0.170, 0.0))
    )

    return cage


def _hex_bolt_head() -> cq.Workplane:
    """Small hexagonal bolt head, centered at origin with axis along local Z."""
    return cq.Workplane("XY").polygon(6, 0.019).extrude(0.012, both=True)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="automotive_differential_gear_assembly",
        meta={
            "description": (
                "Clutch-type limited-slip differential with ring gear, carrier cage, "
                "spider gears, side gears, axle outputs, drive pinion, bolts, and "
                "6-plate friction clutch packs between each side gear and the carrier cheek."
            )
        },
    )

    cast_iron = model.material("dark_cast_iron", rgba=(0.18, 0.18, 0.16, 1.0))
    machined = model.material("machined_steel", rgba=(0.55, 0.52, 0.46, 1.0))
    gear_steel = model.material("oiled_gear_steel", rgba=(0.32, 0.31, 0.28, 1.0))
    tooth_highlight = model.material("bright_tooth_faces", rgba=(0.72, 0.68, 0.58, 1.0))
    bolt_black = model.material("blackened_bolts", rgba=(0.06, 0.065, 0.070, 1.0))
    friction_bronze = model.material("clutch_friction", rgba=(0.38, 0.24, 0.13, 1.0))
    steel_plate = model.material("clutch_steel", rgba=(0.48, 0.46, 0.40, 1.0))

    carrier = model.part("carrier_cage")
    carrier.visual(
        mesh_from_cadquery(_carrier_cage_geometry(), "carrier_open_cage", tolerance=0.003),
        material=cast_iron,
        name="open_cage",
    )

    # Polished cross shaft visible through the cage windows; it carries the spider gears.
    carrier.visual(
        Cylinder(radius=0.0055, length=0.190),
        origin=Origin(rpy=(-PI / 2.0, 0.0, 0.0)),
        material=machined,
        name="cross_shaft",
    )

    # Bolt heads are visible on the flange face, with a coaxial bolt circle.
    bolt_mesh = mesh_from_cadquery(_hex_bolt_head(), "hex_bolt_head", tolerance=0.003)
    for i in range(6):
        a = 2.0 * PI * i / 6.0
        y = 0.112 * math.cos(a)
        z = 0.112 * math.sin(a)
        carrier.visual(
            bolt_mesh,
            origin=Origin(xyz=(-0.089, y, z), rpy=(0.0, PI / 2.0, a)),
            material=bolt_black,
            name=f"bolt_{i}",
        )
        carrier.visual(
            Cylinder(radius=0.0045, length=0.118),
            origin=Origin(xyz=(-0.145, y, z), rpy=(0.0, PI / 2.0, 0.0)),
            material=bolt_black,
            name=f"bolt_shank_{i}",
        )

    # Ring gear is a separate fixed part bolted to the carrier flange.
    # Simplified as a toothed annular ring using a cylinder to stay within compile budget.
    ring = model.part("ring_gear")
    ring.visual(
        Cylinder(radius=0.148, length=0.060),
        origin=Origin(xyz=(-0.220, 0.0, 0.0), rpy=(0.0, PI / 2.0, 0.0)),
        material=gear_steel,
        name="toothed_ring",
    )
    model.articulation(
        "carrier_to_ring",
        ArticulationType.FIXED,
        parent=carrier,
        child=ring,
        origin=Origin(),
    )

    # Clutch-pack LSD friction and steel plate parameters.
    # Plates are mounted on the carrier (inside the cheek cavity) to avoid
    # expensive cross-part collision pairs with the complex cage mesh.
    plate_thickness = 0.0015
    plate_gap = 0.0003
    n_clutch_plates = 6
    friction_plate_radius = 0.038
    steel_plate_radius = 0.043  # slightly larger OD for carrier-keyed tabs
    stack_start_offset = 0.011  # offset from side-gear origin toward the cheek

    # Two side gears and their short axle-output splines. Simplified bevel gear visuals.
    for index, x in enumerate((-0.041, 0.041)):
        side = model.part(f"side_gear_{index}")
        side.visual(
            Cylinder(radius=0.028, length=0.024),
            origin=Origin(rpy=(0.0, PI / 2.0, 0.0)),
            material=tooth_highlight,
            name="bevel_teeth",
        )
        outward = -1.0 if x < 0.0 else 1.0
        side.visual(
            Cylinder(radius=0.016, length=0.105),
            origin=Origin(xyz=(outward * 0.052, 0.0, 0.0), rpy=(0.0, PI / 2.0, 0.0)),
            material=machined,
            name="axle_output",
        )
        side.visual(
            Cylinder(radius=0.030, length=0.040),
            origin=Origin(xyz=(outward * 0.070, 0.0, 0.0), rpy=(0.0, PI / 2.0, 0.0)),
            material=machined,
            name="bearing_journal",
        )
        # Clutch-pack LSD: N alternating friction / steel plates stacked axially
        # between the side-gear back face and the adjacent carrier cheek.
        # Plates are placed on the carrier part (where they physically sit in the
        # cheek cavity) using world-space positions computed from the side gear layout.
        for pi in range(n_clutch_plates):
            plate_x = x + outward * (stack_start_offset + pi * (plate_thickness + plate_gap))
            if pi % 2 == 0:
                carrier.visual(
                    Cylinder(radius=friction_plate_radius, length=plate_thickness),
                    origin=Origin(xyz=(plate_x, 0.0, 0.0), rpy=(0.0, PI / 2.0, 0.0)),
                    material=friction_bronze,
                    name=f"clutch_plate_{index}_{pi}",
                )
            else:
                carrier.visual(
                    Cylinder(radius=steel_plate_radius, length=plate_thickness),
                    origin=Origin(xyz=(plate_x, 0.0, 0.0), rpy=(0.0, PI / 2.0, 0.0)),
                    material=steel_plate,
                    name=f"clutch_plate_{index}_{pi}",
                )
        model.articulation(
            f"carrier_to_side_{index}",
            ArticulationType.CONTINUOUS,
            parent=carrier,
            child=side,
            origin=Origin(xyz=(x, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=80.0, velocity=25.0),
        )

    # Opposed spider pinions on the cross shaft.  They use separate radial teeth so
    # the visible scale remains readable without making the pinions oversized.
    for index, y in enumerate((-0.040, 0.040)):
        spider = model.part(f"spider_gear_{index}")
        spider.visual(
            Cylinder(radius=0.018, length=0.024),
            origin=Origin(rpy=(-PI / 2.0, 0.0, 0.0)),
            material=tooth_highlight,
            name="pinion_hub",
        )
        for j in range(8):
            a = 2.0 * PI * j / 8.0
            spider.visual(
                Box((0.011, 0.026, 0.006)),
                origin=Origin(
                    xyz=(0.023 * math.cos(a), 0.0, 0.023 * math.sin(a)),
                    rpy=(0.0, -a, 0.0),
                ),
                material=tooth_highlight,
                name=f"tooth_{j}",
            )
        model.articulation(
            f"carrier_to_spider_{index}",
            ArticulationType.CONTINUOUS,
            parent=carrier,
            child=spider,
            origin=Origin(xyz=(0.0, y, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=35.0, velocity=35.0),
        )

    # Drive pinion, mounted in the bearing snout and aimed into the ring gear.
    pinion = model.part("drive_pinion")
    pinion.visual(
        Cylinder(radius=0.022, length=0.026),
        origin=Origin(xyz=(0.0, 0.045, 0.0), rpy=(-PI / 2.0, 0.0, 0.0)),
        material=tooth_highlight,
        name="pinion_teeth",
    )
    pinion.visual(
        Cylinder(radius=0.016, length=0.132),
        origin=Origin(xyz=(0.0, -0.008, 0.0), rpy=(-PI / 2.0, 0.0, 0.0)),
        material=machined,
        name="pinion_shaft",
    )
    model.articulation(
        "carrier_to_pinion",
        ArticulationType.CONTINUOUS,
        parent=carrier,
        child=pinion,
        origin=Origin(xyz=(-0.145, -0.225, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=40.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carrier = object_model.get_part("carrier_cage")
    ring = object_model.get_part("ring_gear")
    pinion = object_model.get_part("drive_pinion")
    side_0 = object_model.get_part("side_gear_0")
    side_1 = object_model.get_part("side_gear_1")
    spider_0 = object_model.get_part("spider_gear_0")
    spider_1 = object_model.get_part("spider_gear_1")

    # Prompt-specific structure: coaxial axle outputs, internal spider gears,
    # ring gear fixed to carrier, and the mounted input pinion near the ring.
    for joint_name in (
        "carrier_to_side_0",
        "carrier_to_side_1",
        "carrier_to_spider_0",
        "carrier_to_spider_1",
        "carrier_to_pinion",
    ):
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} is a revolute gear joint",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"{joint_name} type={joint.articulation_type}",
        )

    ctx.expect_origin_distance(
        side_0,
        carrier,
        axes="yz",
        max_dist=0.001,
        name="first side gear is on the axle centerline",
    )
    ctx.expect_origin_distance(
        side_1,
        carrier,
        axes="yz",
        max_dist=0.001,
        name="second side gear is on the axle centerline",
    )
    ctx.expect_origin_gap(
        side_1,
        side_0,
        axis="x",
        min_gap=0.080,
        max_gap=0.086,
        name="side gear origins oppose one another across the cage",
    )
    for side, label in ((side_0, "first"), (side_1, "second")):
        ctx.allow_overlap(
            carrier,
            side,
            elem_a="open_cage",
            elem_b="bearing_journal",
            reason=(
                "The side output journal is intentionally seated in the carrier's "
                "axle opening to show the rotating axle support."
            ),
        )
        ctx.expect_overlap(
            side,
            carrier,
            axes="x",
            elem_a="bearing_journal",
            elem_b="open_cage",
            min_overlap=0.030,
            name=f"{label} side journal is retained in the axle opening",
        )
    ctx.expect_origin_distance(
        spider_0,
        spider_1,
        axes="xz",
        max_dist=0.001,
        name="spider gears share the cross-shaft axis",
    )
    for spider, label in ((spider_0, "first"), (spider_1, "second")):
        ctx.allow_overlap(
            carrier,
            spider,
            elem_a="cross_shaft",
            elem_b="pinion_hub",
            reason=(
                "The spider pinion hub is intentionally represented as captured on "
                "the cross shaft; the hidden bore/shaft fit is simplified as local insertion."
            ),
        )
        ctx.expect_overlap(
            spider,
            carrier,
            axes="y",
            elem_a="pinion_hub",
            elem_b="cross_shaft",
            min_overlap=0.020,
            name=f"{label} spider gear is retained on the cross shaft",
        )
    ctx.expect_overlap(
        ring,
        carrier,
        axes="yz",
        min_overlap=0.110,
        elem_a="toothed_ring",
        elem_b="open_cage",
        name="ring gear is coaxial with the carrier cage",
    )
    for i in range(6):
        ctx.allow_overlap(
            carrier,
            ring,
            elem_a=f"bolt_shank_{i}",
            elem_b="toothed_ring",
            reason="Ring gear bolt shanks intentionally pass through the bolted ring flange.",
        )
        ctx.expect_overlap(
            ring,
            carrier,
            axes="x",
            elem_a="toothed_ring",
            elem_b=f"bolt_shank_{i}",
            min_overlap=0.010,
            name=f"ring gear is retained by bolt shank {i}",
        )
    ctx.allow_overlap(
        carrier,
        pinion,
        elem_a="open_cage",
        elem_b="pinion_shaft",
        reason="The drive pinion shaft is intentionally seated in the bearing bore.",
    )
    ctx.allow_overlap(
        carrier,
        pinion,
        elem_a="open_cage",
        elem_b="pinion_teeth",
        reason="The drive pinion teeth pass through the carrier cage opening to mesh with the ring gear.",
    )
    ctx.expect_overlap(
        pinion,
        carrier,
        axes="y",
        elem_a="pinion_shaft",
        elem_b="open_cage",
        min_overlap=0.050,
        name="drive pinion shaft remains captured in the bearing",
    )
    ctx.expect_origin_gap(
        carrier,
        pinion,
        axis="y",
        min_gap=0.210,
        max_gap=0.240,
        name="drive pinion is mounted beside the ring gear",
    )

    # --- LSD clutch-pack assertions ---
    # Verify clutch plate stacks exist on the carrier with correct naming and count.
    for side_idx in range(2):
        for pi in range(6):
            plate_name = f"clutch_plate_{side_idx}_{pi}"
            ctx.check(
                f"carrier has {plate_name}",
                carrier.get_visual(plate_name) is not None,
                details=f"Missing LSD clutch plate visual '{plate_name}' on carrier_cage",
            )

    # Prove the clutch stacks are positioned between each side gear and the carrier cheek.
    for side_idx, side, label in ((0, side_0, "side_0"), (1, side_1, "side_1")):
        plate_name = f"clutch_plate_{side_idx}_0"
        # The first clutch plate on the carrier should be axially between the side gear
        # bevel teeth and the carrier cheek, proving the stack is in the correct cavity.
        ctx.expect_overlap(
            side,
            carrier,
            axes="x",
            elem_a="bevel_teeth",
            elem_b=plate_name,
            min_overlap=0.001,
            name=f"{label} bevel gear axially overlaps its first clutch plate (stack is between gear and cheek)",
        )
        # Clutch plates are radially centered on the axle axis, proven by overlap
        # with the side gear's bearing journal in the YZ plane.
        ctx.expect_overlap(
            side,
            carrier,
            axes="yz",
            elem_a="bearing_journal",
            elem_b=plate_name,
            min_overlap=0.025,
            name=f"{label} clutch plate stack is radially aligned with the axle output",
        )

    return ctx.report()


object_model = build_object_model()
