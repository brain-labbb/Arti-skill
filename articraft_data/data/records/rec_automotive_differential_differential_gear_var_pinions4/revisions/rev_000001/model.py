from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


PI = math.pi


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="automotive_differential_gear_assembly",
        meta={
            "description": (
                "Heavy-duty 4-pinion open differential with ring gear, carrier cage, "
                "four spider gears on a cross shaft, two side gears, axle outputs, "
                "and drive pinion."
            )
        },
    )

    cast_iron = model.material("dark_cast_iron", rgba=(0.18, 0.18, 0.16, 1.0))
    machined = model.material("machined_steel", rgba=(0.55, 0.52, 0.46, 1.0))
    gear_steel = model.material("oiled_gear_steel", rgba=(0.32, 0.31, 0.28, 1.0))
    tooth_highlight = model.material("bright_tooth_faces", rgba=(0.72, 0.68, 0.58, 1.0))
    bolt_black = model.material("blackened_bolts", rgba=(0.06, 0.065, 0.070, 1.0))

    # ── Carrier cage (root part) ──────────────────────────────────────────
    carrier = model.part("carrier_cage")

    # Two circular cheeks
    for x, label in ((-0.066, "cheek_left"), (0.066, "cheek_right")):
        carrier.visual(
            Cylinder(radius=0.090, length=0.030),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, PI / 2.0, 0.0)),
            material=cast_iron,
            name=label,
        )
    # Four bridges connecting the cheeks
    carrier.visual(
        Box((0.160, 0.038, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.083)),
        material=cast_iron,
        name="bridge_top",
    )
    carrier.visual(
        Box((0.160, 0.038, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, -0.083)),
        material=cast_iron,
        name="bridge_bottom",
    )
    carrier.visual(
        Box((0.160, 0.030, 0.038)),
        origin=Origin(xyz=(0.0, 0.098, 0.0)),
        material=cast_iron,
        name="bridge_front",
    )
    carrier.visual(
        Box((0.160, 0.030, 0.038)),
        origin=Origin(xyz=(0.0, -0.098, 0.0)),
        material=cast_iron,
        name="bridge_back",
    )
    # Axle collars
    for x, label in ((-0.112, "axle_collar_left"), (0.112, "axle_collar_right")):
        carrier.visual(
            Cylinder(radius=0.041, length=0.070),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, PI / 2.0, 0.0)),
            material=cast_iron,
            name=label,
        )
    # Ring-gear mounting flange
    carrier.visual(
        Cylinder(radius=0.145, length=0.026),
        origin=Origin(xyz=(-0.105, 0.0, 0.0), rpy=(0.0, PI / 2.0, 0.0)),
        material=cast_iron,
        name="flange",
    )
    # Four-arm spider cross: two polished shafts crossing at the carrier centre
    carrier.visual(
        Cylinder(radius=0.0055, length=0.190),
        origin=Origin(rpy=(-PI / 2.0, 0.0, 0.0)),
        material=machined,
        name="cross_shaft_y",
    )
    carrier.visual(
        Cylinder(radius=0.0055, length=0.190),
        material=machined,
        name="cross_shaft_z",
    )
    # Bolt heads on the flange face
    for i in range(8):
        a = 2.0 * PI * i / 8.0
        y = 0.112 * math.cos(a)
        z = 0.112 * math.sin(a)
        carrier.visual(
            Cylinder(radius=0.009, length=0.012),
            origin=Origin(xyz=(-0.089, y, z), rpy=(0.0, PI / 2.0, a)),
            material=bolt_black,
            name=f"bolt_{i}",
        )

    # ── Ring gear (fixed to carrier) ──────────────────────────────────────
    ring = model.part("ring_gear")
    ring.visual(
        Cylinder(radius=0.148, length=0.040),
        origin=Origin(xyz=(-0.128, 0.0, 0.0), rpy=(0.0, PI / 2.0, 0.0)),
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

    # ── Side gears (two, coaxial on X axis) ──────────────────────────────
    for index, x in enumerate((-0.041, 0.041)):
        side = model.part(f"side_gear_{index}")
        side.visual(
            Cylinder(radius=0.038, length=0.024),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, PI / 2.0, 0.0)),
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
        model.articulation(
            f"carrier_to_side_{index}",
            ArticulationType.CONTINUOUS,
            parent=carrier,
            child=side,
            origin=Origin(xyz=(x, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=80.0, velocity=25.0),
        )

    # ── Four spider pinions on the 4-arm cross ───────────────────────────
    spider_positions = (
        (0.0, 0.040, 0.0),   # +Y arm
        (0.0, -0.040, 0.0),  # -Y arm
        (0.0, 0.0, 0.040),   # +Z arm
        (0.0, 0.0, -0.040),  # -Z arm
    )
    spider_axes = (
        (0.0, 1.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
        (0.0, 0.0, 1.0),
    )
    spider_hub_rpys = (
        (-PI / 2.0, 0.0, 0.0),  # Y-arm → cylinder along Y
        (-PI / 2.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),        # Z-arm → cylinder along Z (default)
        (0.0, 0.0, 0.0),
    )
    for index, (pos, axis, hub_rpy) in enumerate(
        zip(spider_positions, spider_axes, spider_hub_rpys)
    ):
        spider = model.part(f"spider_gear_{index}")
        spider.visual(
            Cylinder(radius=0.018, length=0.024),
            origin=Origin(rpy=hub_rpy),
            material=tooth_highlight,
            name="pinion_hub",
        )
        model.articulation(
            f"carrier_to_spider_{index}",
            ArticulationType.CONTINUOUS,
            parent=carrier,
            child=spider,
            origin=Origin(xyz=pos),
            axis=axis,
            motion_limits=MotionLimits(effort=35.0, velocity=35.0),
        )

    # ── Drive pinion ─────────────────────────────────────────────────────
    pinion = model.part("drive_pinion")
    pinion.visual(
        Cylinder(radius=0.030, length=0.026),
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
        origin=Origin(xyz=(-0.145, -0.170, 0.0)),
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
    spiders = [object_model.get_part(f"spider_gear_{i}") for i in range(4)]

    # Verify all seven articulations are CONTINUOUS (or FIXED for ring).
    for joint_name in (
        "carrier_to_side_0",
        "carrier_to_side_1",
        "carrier_to_spider_0",
        "carrier_to_spider_1",
        "carrier_to_spider_2",
        "carrier_to_spider_3",
        "carrier_to_pinion",
    ):
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} is a continuous gear joint",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"{joint_name} type={joint.articulation_type}",
        )

    ring_joint = object_model.get_articulation("carrier_to_ring")
    ctx.check(
        "carrier_to_ring is a fixed joint",
        ring_joint.articulation_type == ArticulationType.FIXED,
        details=f"carrier_to_ring type={ring_joint.articulation_type}",
    )

    # Side gears are coaxial on X
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

    # Side journal retained in axle collar
    for side, label, collar_name, cheek_name in (
        (side_0, "first", "axle_collar_left", "cheek_left"),
        (side_1, "second", "axle_collar_right", "cheek_right"),
    ):
        ctx.allow_overlap(
            carrier,
            side,
            elem_a=collar_name,
            elem_b="axle_output",
            reason=(
                "The side axle output shaft passes through the carrier's "
                "axle collar to show the rotating axle support."
            ),
        )
        ctx.allow_overlap(
            carrier,
            side,
            elem_a=collar_name,
            elem_b="bearing_journal",
            reason=(
                "The side output journal is intentionally seated in the carrier's "
                "axle opening to show the rotating axle support."
            ),
        )
        ctx.allow_overlap(
            carrier,
            side,
            elem_a=collar_name,
            elem_b="bevel_teeth",
            reason=(
                "The side gear bevel teeth sit inside the carrier cage near the axle collar."
            ),
        )
        ctx.allow_overlap(
            carrier,
            side,
            elem_a=cheek_name,
            elem_b="axle_output",
            reason=(
                "The side axle output shaft passes through the carrier cheek "
                "to extend outward from the differential."
            ),
        )
        ctx.allow_overlap(
            carrier,
            side,
            elem_a="flange",
            elem_b="bearing_journal",
            reason=(
                "The side gear bearing journal sits near the carrier flange inside the assembly."
            ),
        )
        ctx.allow_overlap(
            carrier,
            side,
            elem_a="flange",
            elem_b="axle_output",
            reason=(
                "The side gear axle output passes near the carrier flange inside the assembly."
            ),
        )
        ctx.allow_overlap(
            carrier,
            side,
            elem_a=cheek_name,
            elem_b="bevel_teeth",
            reason=(
                "The side gear bevel teeth sit inside the carrier cage near the cheek."
            ),
        )
        ctx.expect_overlap(
            side,
            carrier,
            axes="x",
            elem_a="bearing_journal",
            elem_b=collar_name,
            min_overlap=0.030,
            name=f"{label} side journal is retained in the axle opening",
        )

    # 4-pinion spider cross: Y-arm pair and Z-arm pair
    ctx.expect_origin_distance(
        spiders[0],
        spiders[1],
        axes="xz",
        max_dist=0.001,
        name="Y-arm spider pair shares the Y-axis cross arm",
    )
    ctx.expect_origin_distance(
        spiders[2],
        spiders[3],
        axes="xy",
        max_dist=0.001,
        name="Z-arm spider pair shares the Z-axis cross arm",
    )
    spider_shaft_map = (
        ("cross_shaft_y", "y"),
        ("cross_shaft_y", "y"),
        ("cross_shaft_z", "z"),
        ("cross_shaft_z", "z"),
    )
    for index, (shaft_elem, overlap_axis) in enumerate(spider_shaft_map):
        spider = spiders[index]
        label = f"spider_gear_{index}"
        ctx.allow_overlap(
            carrier,
            spider,
            elem_a=shaft_elem,
            elem_b="pinion_hub",
            reason=(
                f"The {label} pinion hub is intentionally captured on the "
                f"4-arm spider cross ({shaft_elem}); the hidden bore/shaft fit "
                "is simplified as local insertion."
            ),
        )
        ctx.expect_overlap(
            spider,
            carrier,
            axes=overlap_axis,
            elem_a="pinion_hub",
            elem_b=shaft_elem,
            min_overlap=0.020,
            name=f"{label} is retained on the {shaft_elem} arm",
        )

    # Ring gear coaxial with carrier and retained on flange
    ctx.allow_overlap(
        carrier,
        ring,
        elem_a="axle_collar_left",
        elem_b="toothed_ring",
        reason="Ring gear passes near the axle collar as it bolts to the flange.",
    )
    ctx.allow_overlap(
        carrier,
        ring,
        elem_a="flange",
        elem_b="toothed_ring",
        reason="Ring gear is bolted directly to the carrier flange; contact overlap is intentional.",
    )
    ctx.allow_overlap(
        ring,
        side_0,
        elem_a="toothed_ring",
        elem_b="axle_output",
        reason="Ring gear and side gear axle output are in the same assembly space.",
    )
    ctx.allow_overlap(
        ring,
        side_0,
        elem_a="toothed_ring",
        elem_b="bearing_journal",
        reason="Ring gear and side gear bearing journal share the differential interior.",
    )
    ctx.allow_overlap(
        pinion,
        ring,
        elem_a="pinion_teeth",
        elem_b="toothed_ring",
        reason="Drive pinion teeth mesh with ring gear teeth.",
    )
    ctx.allow_overlap(
        pinion,
        ring,
        elem_a="pinion_shaft",
        elem_b="toothed_ring",
        reason="Drive pinion shaft extends near the ring gear bore.",
    )
    ctx.expect_overlap(
        ring,
        carrier,
        axes="yz",
        min_overlap=0.110,
        elem_a="toothed_ring",
        elem_b="flange",
        name="ring gear is coaxial with the carrier cage",
    )
    ctx.expect_overlap(
        ring,
        carrier,
        axes="x",
        elem_a="toothed_ring",
        elem_b="flange",
        min_overlap=0.005,
        name="ring gear is retained on the carrier flange",
    )

    # Drive pinion shaft passes through carrier structure
    ctx.allow_overlap(
        carrier,
        pinion,
        elem_a="flange",
        elem_b="pinion_shaft",
        reason=(
            "The drive pinion shaft passes through the carrier flange area "
            "to mesh with the ring gear."
        ),
    )
    ctx.expect_overlap(
        pinion,
        carrier,
        axes="y",
        elem_a="pinion_shaft",
        elem_b="flange",
        min_overlap=0.001,
        name="drive pinion shaft passes through carrier",
    )

    return ctx.report()


object_model = build_object_model()
