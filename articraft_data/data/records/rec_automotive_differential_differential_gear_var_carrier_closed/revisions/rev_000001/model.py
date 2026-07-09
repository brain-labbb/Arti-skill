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


def _ring_gear_visuals(part, material_tooth, material_body):
    """Add ring gear visuals using SDK primitives: body cylinder + tooth boxes."""
    # Ring gear body at x=-0.220 (matching original placement).
    part.visual(
        Cylinder(radius=0.140, length=0.040),
        origin=Origin(xyz=(-0.220, 0.0, 0.0), rpy=(0.0, PI / 2.0, 0.0)),
        material=material_body,
        name="ring_body",
    )
    # Simplified tooth ring: 46 small boxes arranged around the inner circumference.
    n_teeth = 46
    pitch_r = 0.138
    tooth_width = 0.006
    tooth_height = 0.008
    tooth_depth = 0.040
    for i in range(n_teeth):
        a = 2.0 * PI * i / n_teeth
        y = pitch_r * math.cos(a)
        z = pitch_r * math.sin(a)
        part.visual(
            Box((tooth_depth, tooth_width, tooth_height)),
            origin=Origin(xyz=(-0.220, y, z), rpy=(a, 0.0, 0.0)),
            material=material_tooth,
            name=f"ring_tooth_{i}",
        )


def _bevel_gear_visuals(part, material_tooth, material_body):
    """Add bevel gear visuals: hub cylinder + radial tooth boxes touching the hub surface."""
    # Bevel gear body aligned with X axis (rotated from Z).
    body_r = 0.032
    body_len = 0.024
    part.visual(
        Cylinder(radius=body_r, length=body_len),
        origin=Origin(rpy=(0.0, PI / 2.0, 0.0)),
        material=material_body,
        name="bevel_body",
    )
    # 18 teeth arranged radially, touching the body surface.
    n_teeth = 18
    tooth_w = 0.004
    tooth_h = body_r * 0.28  # tooth height extends slightly beyond body
    tooth_d = body_len  # tooth depth matches body length
    tooth_center_r = body_r + tooth_h / 2.0 - 0.001  # slight embed for connectivity
    for i in range(n_teeth):
        a = 2.0 * PI * i / n_teeth
        y = tooth_center_r * math.cos(a)
        z = tooth_center_r * math.sin(a)
        part.visual(
            Box((tooth_d, tooth_w, tooth_h)),
            origin=Origin(xyz=(0.0, y, z), rpy=(a, 0.0, 0.0)),
            material=material_tooth,
            name=f"bevel_tooth_{i}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="automotive_differential_gear_assembly",
        meta={
            "description": (
                "Automotive differential gear assembly with closed solid one-piece cast "
                "carrier case, ring gear, spider gears, side gears, axle outputs, "
                "mounted drive pinion, inspection ports, and bolts."
            )
        },
    )

    cast_iron = model.material("dark_cast_iron", rgba=(0.18, 0.18, 0.16, 1.0))
    machined = model.material("machined_steel", rgba=(0.55, 0.52, 0.46, 1.0))
    gear_steel = model.material("oiled_gear_steel", rgba=(0.32, 0.31, 0.28, 1.0))
    tooth_highlight = model.material("bright_tooth_faces", rgba=(0.72, 0.68, 0.58, 1.0))
    bolt_black = model.material("blackened_bolts", rgba=(0.06, 0.065, 0.070, 1.0))
    port_dark = model.material("port_recess", rgba=(0.04, 0.04, 0.04, 1.0))

    # ---- Carrier: closed solid one-piece cast differential case ----
    carrier = model.part("carrier_cage")

    # Main barrel shell (axle axis = X; Cylinder is Z-aligned, rotate by pitch=PI/2).
    carrier.visual(
        Cylinder(radius=0.100, length=0.164),
        origin=Origin(rpy=(0.0, PI / 2.0, 0.0)),
        material=cast_iron,
        name="closed_case",
    )
    # Axle collars / bearing bosses on each side.
    for idx, x_sign in enumerate((-1.0, 1.0)):
        carrier.visual(
            Cylinder(radius=0.041, length=0.070),
            origin=Origin(xyz=(x_sign * 0.112, 0.0, 0.0), rpy=(0.0, PI / 2.0, 0.0)),
            material=cast_iron,
            name=f"axle_collar_{idx}",
        )
    # Ring-gear mounting flange.
    carrier.visual(
        Cylinder(radius=0.145, length=0.026),
        origin=Origin(xyz=(-0.105, 0.0, 0.0), rpy=(0.0, PI / 2.0, 0.0)),
        material=cast_iron,
        name="ring_flange",
    )
    # Register shoulder.
    carrier.visual(
        Cylinder(radius=0.102, length=0.010),
        origin=Origin(xyz=(-0.124, 0.0, 0.0), rpy=(0.0, PI / 2.0, 0.0)),
        material=cast_iron,
        name="register_shoulder",
    )
    # Pinion bearing snout.
    carrier.visual(
        Cylinder(radius=0.033, length=0.072),
        origin=Origin(xyz=(-0.145, -0.225, 0.0), rpy=(-PI / 2.0, 0.0, 0.0)),
        material=cast_iron,
        name="pinion_bearing_snout",
    )
    # Support ribs connecting bearing snout to flange.
    carrier.visual(
        Box((0.052, 0.130, 0.022)),
        origin=Origin(xyz=(-0.135, -0.160, 0.044)),
        material=cast_iron,
        name="upper_rib",
    )
    carrier.visual(
        Box((0.052, 0.130, 0.022)),
        origin=Origin(xyz=(-0.135, -0.160, -0.044)),
        material=cast_iron,
        name="lower_rib",
    )

    # Cross shaft through the carrier (carries spider gears).
    carrier.visual(
        Cylinder(radius=0.0055, length=0.190),
        origin=Origin(rpy=(-PI / 2.0, 0.0, 0.0)),
        material=machined,
        name="cross_shaft",
    )

    # Bolt heads on the flange face with coaxial bolt circle.
    for i in range(10):
        a = 2.0 * PI * i / 10.0
        y = 0.112 * math.cos(a)
        z = 0.112 * math.sin(a)
        # Hex bolt head approximated as a small cylinder (6-sided = close enough at this scale).
        carrier.visual(
            Cylinder(radius=0.0095, length=0.012),
            origin=Origin(xyz=(-0.089, y, z), rpy=(0.0, PI / 2.0, a)),
            material=bolt_black,
            name=f"bolt_{i}",
        )
        carrier.visual(
            Cylinder(radius=0.0045, length=0.140),
            origin=Origin(xyz=(-0.155, y, z), rpy=(0.0, PI / 2.0, 0.0)),
            material=bolt_black,
            name=f"bolt_shank_{i}",
        )

    # Inspection ports: 4 small dark recessed discs on the barrel surface.
    for i in range(4):
        angle = 2.0 * PI * i / 4.0
        py = 0.101 * math.sin(angle)
        pz = 0.101 * math.cos(angle)
        carrier.visual(
            Cylinder(radius=0.014, length=0.004),
            origin=Origin(
                xyz=(0.0, py, pz),
                rpy=(-angle, 0.0, 0.0),
            ),
            material=port_dark,
            name=f"inspection_port_{i}",
        )

    # ---- Ring gear (fixed to carrier flange) ----
    ring = model.part("ring_gear")
    _ring_gear_visuals(ring, tooth_highlight, gear_steel)
    model.articulation(
        "carrier_to_ring",
        ArticulationType.FIXED,
        parent=carrier,
        child=ring,
        origin=Origin(),
    )

    # ---- Side gears (two, coaxial on the axle axis) ----
    for index, x in enumerate((-0.041, 0.041)):
        side = model.part(f"side_gear_{index}")
        _bevel_gear_visuals(side, tooth_highlight, gear_steel)
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

    # ---- Spider gears (two, on the cross shaft) ----
    for index, y in enumerate((-0.040, 0.040)):
        spider = model.part(f"spider_gear_{index}")
        spider.visual(
            Cylinder(radius=0.018, length=0.024),
            origin=Origin(rpy=(-PI / 2.0, 0.0, 0.0)),
            material=tooth_highlight,
            name="pinion_hub",
        )
        for j in range(12):
            a = 2.0 * PI * j / 12.0
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

    # ---- Drive pinion (mounted in bearing snout, aimed at ring gear) ----
    pinion = model.part("drive_pinion")
    # Pinion body at the mesh point with the ring gear.
    pinion_body_r = 0.024
    pinion_body_len = 0.026
    pinion.visual(
        Cylinder(radius=pinion_body_r, length=pinion_body_len),
        origin=Origin(xyz=(0.0, 0.045, 0.0), rpy=(-PI / 2.0, 0.0, 0.0)),
        material=gear_steel,
        name="pinion_body",
    )
    # 14 teeth around the pinion body, touching the surface.
    tooth_h = pinion_body_r * 0.25
    tooth_center_r = pinion_body_r + tooth_h / 2.0 - 0.001  # slight embed for connectivity
    for i in range(14):
        a = 2.0 * PI * i / 14.0
        pinion.visual(
            Box((pinion_body_len, 0.004, tooth_h)),
            origin=Origin(
                xyz=(0.0, 0.045 + tooth_center_r * math.cos(a),
                     tooth_center_r * math.sin(a)),
                rpy=(a, 0.0, 0.0),
            ),
            material=tooth_highlight,
            name=f"pinion_tooth_{i}",
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

    # Prompt-specific: all moving joints must be continuous (revolute gear joints).
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

    # Side gears are coaxial on the axle centerline.
    ctx.expect_origin_distance(
        side_0, carrier, axes="yz", max_dist=0.001,
        name="first side gear is on the axle centerline",
    )
    ctx.expect_origin_distance(
        side_1, carrier, axes="yz", max_dist=0.001,
        name="second side gear is on the axle centerline",
    )
    ctx.expect_origin_gap(
        side_1, side_0, axis="x", min_gap=0.080, max_gap=0.086,
        name="side gear origins oppose one another across the case",
    )

    # Side gear journals overlap with axle collars (seated fit) and with the barrel wall.
    collar_for = {0: "axle_collar_0", 1: "axle_collar_1"}
    for index, (side, label) in enumerate(((side_0, "first"), (side_1, "second"))):
        collar = collar_for[index]
        ctx.allow_overlap(
            carrier, side, elem_a=collar, elem_b="bearing_journal",
            reason="Side output journal is seated in the carrier axle collar.",
        )
        ctx.allow_overlap(
            carrier, side, elem_a=collar, elem_b="axle_output",
            reason="Side axle output shaft passes through the carrier axle collar bore.",
        )
        ctx.allow_overlap(
            carrier, side, elem_a="closed_case", elem_b="axle_output",
            reason="Axle output shaft passes through the closed carrier case barrel wall.",
        )
        ctx.allow_overlap(
            carrier, side, elem_a="closed_case", elem_b="bevel_body",
            reason="Side gear body is enclosed inside the closed carrier case barrel.",
        )
        ctx.allow_overlap(
            carrier, side, elem_a="closed_case", elem_b="bearing_journal",
            reason="Side gear bearing journal is enclosed inside the closed carrier case.",
        )
        ctx.allow_overlap(
            carrier, side, elem_a="ring_flange", elem_b="bearing_journal",
            reason="Side gear bearing journal passes near the ring flange mounting area.",
        )
        ctx.allow_overlap(
            carrier, side, elem_a="register_shoulder", elem_b="bearing_journal",
            reason="Side gear bearing journal passes through the register shoulder area.",
        )
        ctx.allow_overlap(
            carrier, side, elem_a="register_shoulder", elem_b="axle_output",
            reason="Side gear axle output passes through the register shoulder area.",
        )
        ctx.allow_overlap(
            carrier, side, elem_a="ring_flange", elem_b="axle_output",
            reason="Side gear axle output passes near the ring flange mounting area.",
        )
        # Side gear bevel teeth are inside the carrier barrel.
        for t in range(18):
            ctx.allow_overlap(
                carrier, side, elem_a="closed_case", elem_b=f"bevel_tooth_{t}",
                reason="Side gear teeth rotate inside the closed carrier case barrel.",
            )
        ctx.expect_overlap(
            side, carrier, axes="x", elem_a="bearing_journal", elem_b=collar,
            min_overlap=0.030,
            name=f"{label} side journal is retained in the axle collar",
        )

    # Spider gears share the cross-shaft axis.
    ctx.expect_origin_distance(
        spider_0, spider_1, axes="xz", max_dist=0.001,
        name="spider gears share the cross-shaft axis",
    )
    for spider, label in ((spider_0, "first"), (spider_1, "second")):
        ctx.allow_overlap(
            carrier, spider, elem_a="cross_shaft", elem_b="pinion_hub",
            reason="Spider pinion hub is captured on the cross shaft.",
        )
        ctx.allow_overlap(
            carrier, spider, elem_a="closed_case", elem_b="pinion_hub",
            reason="Spider gears rotate inside the closed carrier case barrel.",
        )
        # Spider gear teeth rotate inside the barrel; allow each tooth.
        for j in range(12):
            ctx.allow_overlap(
                carrier, spider, elem_a="closed_case", elem_b=f"tooth_{j}",
                reason="Spider gear teeth rotate inside the closed carrier case barrel.",
            )
        ctx.expect_overlap(
            spider, carrier, axes="y", elem_a="pinion_hub", elem_b="cross_shaft",
            min_overlap=0.020,
            name=f"{label} spider gear is retained on the cross shaft",
        )

    # Ring gear coaxial with carrier flange.
    ctx.expect_overlap(
        ring, carrier, axes="yz", min_overlap=0.110,
        elem_a="ring_body", elem_b="ring_flange",
        name="ring gear is coaxial with the carrier flange",
    )
    for i in range(10):
        ctx.allow_overlap(
            carrier, ring, elem_a=f"bolt_shank_{i}", elem_b="ring_body",
            reason="Ring gear bolt shanks pass through the bolted ring flange.",
        )
        ctx.expect_overlap(
            ring, carrier, axes="x", elem_a="ring_body", elem_b=f"bolt_shank_{i}",
            min_overlap=0.020,
            name=f"ring gear is retained by bolt shank {i}",
        )

    # Drive pinion seated in bearing snout.
    ctx.allow_overlap(
        carrier, pinion, elem_a="pinion_bearing_snout", elem_b="pinion_shaft",
        reason="Drive pinion shaft is seated in the bearing snout.",
    )
    # Pinion teeth near the bearing snout end may have small local contact.
    for i in range(14):
        ctx.allow_overlap(
            carrier, pinion, elem_a="pinion_bearing_snout", elem_b=f"pinion_tooth_{i}",
            reason="Pinion tooth tips are near the bearing snout opening; small local overlap at the snout entry.",
        )
    ctx.expect_overlap(
        pinion, carrier, axes="y", elem_a="pinion_shaft", elem_b="pinion_bearing_snout",
        min_overlap=0.050,
        name="drive pinion shaft remains captured in the bearing",
    )
    ctx.expect_origin_gap(
        carrier, pinion, axis="y", min_gap=0.210, max_gap=0.240,
        name="drive pinion is mounted beside the ring gear",
    )

    # Prompt-specific: carrier_cage must be a closed solid one-piece case.
    closed_case_visual = carrier.get_visual("closed_case")
    ctx.check(
        "carrier_cage has closed solid case shell",
        closed_case_visual is not None,
        details="carrier_cage must have a 'closed_case' visual (solid one-piece differential case)",
    )

    # Inspection ports must exist on the closed case surface.
    for i in range(4):
        port = carrier.get_visual(f"inspection_port_{i}")
        ctx.check(
            f"inspection_port_{i} exists on closed case",
            port is not None,
            details=f"inspection_port_{i} visual must exist on the carrier case",
        )

    # Internal gears must be enclosed within the carrier barrel on YZ axes.
    ctx.expect_within(
        side_0, carrier, axes="yz", elem_a="bevel_body", elem_b="closed_case", margin=0.010,
        name="first side gear is enclosed within the closed carrier case",
    )
    ctx.expect_within(
        side_1, carrier, axes="yz", elem_a="bevel_body", elem_b="closed_case", margin=0.010,
        name="second side gear is enclosed within the closed carrier case",
    )
    ctx.expect_within(
        spider_0, carrier, axes="yz", elem_a="pinion_hub", elem_b="closed_case", margin=0.010,
        name="first spider gear is enclosed within the closed carrier case",
    )
    ctx.expect_within(
        spider_1, carrier, axes="yz", elem_a="pinion_hub", elem_b="closed_case", margin=0.010,
        name="second spider gear is enclosed within the closed carrier case",
    )

    return ctx.report()


object_model = build_object_model()
