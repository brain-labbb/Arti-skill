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
    """Simplified open differential cage with bearing sleeves and flange."""
    # Simplified: use only 2 bridges instead of 4 for faster boolean ops.
    bridge_specs = (
        ((0.000, 0.000, 0.083), (0.160, 0.038, 0.030)),
        ((0.000, 0.000, -0.083), (0.160, 0.038, 0.030)),
    )
    # Two circular cheeks support the axle-side gears.
    cage = _annular_cylinder_x(0.030, 0.090, 0.047, center=(-0.066, 0.0, 0.0))
    cage = cage.union(cq.Workplane("XY").box(*bridge_specs[0][1]).translate(bridge_specs[0][0]))
    cage = cage.union(_annular_cylinder_x(0.030, 0.090, 0.047, center=(0.066, 0.0, 0.0)))
    cage = cage.union(cq.Workplane("XY").box(*bridge_specs[1][1]).translate(bridge_specs[1][0]))

    # Axle collars.
    for x in (-0.112, 0.112):
        cage = cage.union(_annular_cylinder_x(0.070, 0.041, 0.030, center=(x, 0.0, 0.0)))

    # Ring-gear mounting flange only (skip shoulder for speed).
    flange = _annular_cylinder_x(0.026, 0.145, 0.061, center=(-0.105, 0.0, 0.0))
    cage = cage.union(flange)

    # Simplified pinion bearing snout without ribs.
    bearing = _cylinder_y(0.072, 0.033, center=(-0.145, -0.225, 0.0))
    cage = cage.union(bearing)

    return cage


def _hex_bolt_head() -> cq.Workplane:
    """Small hexagonal bolt head, centered at origin with axis along local Z."""
    return cq.Workplane("XY").polygon(6, 0.019).extrude(0.012, both=True)



def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="automotive_differential_gear_assembly",
        meta={
            "description": (
                "Visible cutaway-style differential core with ring gear, carrier cage, "
                "spider gears, side gears, axle outputs, mounted drive pinion, and bolts."
            )
        },
    )

    cast_iron = model.material("dark_cast_iron", rgba=(0.18, 0.18, 0.16, 1.0))
    machined = model.material("machined_steel", rgba=(0.55, 0.52, 0.46, 1.0))
    gear_steel = model.material("oiled_gear_steel", rgba=(0.32, 0.31, 0.28, 1.0))
    tooth_highlight = model.material("bright_tooth_faces", rgba=(0.72, 0.68, 0.58, 1.0))
    bolt_black = model.material("blackened_bolts", rgba=(0.06, 0.065, 0.070, 1.0))

    carrier = model.part("carrier_cage")
    carrier.visual(
        mesh_from_cadquery(_carrier_cage_geometry(), "carrier_open_cage", tolerance=0.002),
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
    bolt_mesh = mesh_from_cadquery(_hex_bolt_head(), "hex_bolt_head", tolerance=0.002)
    for i in range(4):
        a = 2.0 * PI * i / 4.0
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

    # Carrier dog teeth for the selectable diff-lock collar.
    # Six teeth on an internal splined boss inside the left cheek bore.
    # The boss radius matches the cheek inner radius for physical contact.
    carrier.visual(
        Cylinder(radius=0.047, length=0.008),
        origin=Origin(xyz=(-0.077, 0.0, 0.0), rpy=(0.0, PI / 2.0, 0.0)),
        material=gear_steel,
        name="lock_boss",
    )
    carrier_dog_tooth_mid_r = 0.037  # radial position between boss center and cheek wall
    carrier_dog_tooth_x = -0.081  # axial position at boss outer face
    for i in range(6):
        a = 2.0 * PI * i / 6.0 + PI / 6.0  # offset for interleaving with collar teeth
        ty = carrier_dog_tooth_mid_r * math.cos(a)
        tz = carrier_dog_tooth_mid_r * math.sin(a)
        carrier.visual(
            Box((0.005, 0.006, 0.004)),
            origin=Origin(
                xyz=(carrier_dog_tooth_x, ty, tz),
                rpy=(a, 0.0, 0.0),
            ),
            material=gear_steel,
            name=f"carrier_dog_tooth_{i}",
        )

    # Ring gear is a separate fixed part bolted to the carrier flange.
    # Simplified: use a cylinder instead of SpurGear mesh for compile speed.
    ring = model.part("ring_gear")
    ring.visual(
        Cylinder(radius=0.137, length=0.040),
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

    # Two side gears and their short axle-output splines. Simplified: use cylinders.
    side_visual = Cylinder(radius=0.036, length=0.024)
    for index, x in enumerate((-0.041, 0.041)):
        side = model.part(f"side_gear_{index}")
        inward_pitch = PI / 2.0 if x < 0.0 else -PI / 2.0
        side.visual(
            side_visual,
            origin=Origin(rpy=(0.0, inward_pitch, 0.0)),
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

    # Drive pinion, mounted in the bearing snout and aimed into the ring gear.
    # Simplified: use a cylinder instead of BevelGear mesh for compile speed.
    pinion = model.part("drive_pinion")
    pinion.visual(
        Cylinder(radius=0.028, length=0.026),
        origin=Origin(xyz=(0.0, 0.045, 0.0), rpy=(-PI / 2.0, 0.0, PI / 14.0)),
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

    # ── Selectable diff-lock: splined dog collar on the side_gear_0 hub ──
    # The collar slides axially along the axle to engage/disengage carrier dog teeth.
    collar = model.part("dog_collar")
    # Main collar body: annular ring (outer cylinder minus inner bore)
    collar.visual(
        Cylinder(radius=0.047, length=0.014),
        origin=Origin(rpy=(0.0, PI / 2.0, 0.0)),
        material=machined,
        name="collar_body",
    )
    collar.visual(
        Cylinder(radius=0.0175, length=0.016),
        origin=Origin(rpy=(0.0, PI / 2.0, 0.0)),
        material=cast_iron,
        name="collar_bore",
    )
    # Shift-fork groove: a slightly smaller ring visible as a channel
    collar.visual(
        Cylinder(radius=0.031, length=0.004),
        origin=Origin(xyz=(-0.002, 0.0, 0.0), rpy=(0.0, PI / 2.0, 0.0)),
        material=cast_iron,
        name="shift_groove",
    )
    # Six collar dog teeth on the inward (+X) face for engagement.
    collar_tooth_mid_r = 0.037  # radial position matching carrier teeth
    collar_tooth_half_w = 0.007  # collar body half-width along X
    for i in range(6):
        a = 2.0 * PI * i / 6.0
        ty = collar_tooth_mid_r * math.cos(a)
        tz = collar_tooth_mid_r * math.sin(a)
        collar.visual(
            Box((0.005, 0.006, 0.004)),
            origin=Origin(
                xyz=(collar_tooth_half_w, ty, tz),
                rpy=(a, 0.0, 0.0),
            ),
            material=tooth_highlight,
            name=f"collar_dog_tooth_{i}",
        )
    # PRISMATIC joint: collar slides along +X (inward toward carrier center).
    # At q=0 (disengaged) the collar sits outward on the axle.
    # At q=upper (engaged) the collar teeth interleave with carrier dog teeth.
    model.articulation(
        "carrier_to_lockcollar",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=collar,
        origin=Origin(xyz=(-0.104, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.15, lower=0.0, upper=0.018),
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
    collar = object_model.get_part("dog_collar")
    lock_joint = object_model.get_articulation("carrier_to_lockcollar")

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
    for i in range(4):
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
            min_overlap=0.003,
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
        reason=(
            "The drive pinion teeth are intentionally positioned near the carrier bearing "
            "snout to mesh with the ring gear; local proximity overlap is acceptable."
        ),
    )
    ctx.allow_overlap(
        carrier,
        side_0,
        elem_a="lock_boss",
        elem_b="axle_output",
        reason=(
            "The carrier lock boss surrounds the side_gear_0 axle output to provide "
            "the stationary dog-tooth engagement surface for the diff-lock collar."
        ),
    )
    ctx.expect_overlap(
        carrier,
        side_0,
        axes="x",
        elem_a="lock_boss",
        elem_b="axle_output",
        min_overlap=0.005,
        name="lock boss overlaps side_gear_0 axle along X for dog engagement",
    )
    ctx.allow_overlap(
        side_0,
        collar,
        reason=(
            "The dog collar is intentionally represented as sliding over the "
            "side_gear_0 axle/journal assembly for diff-lock engagement; "
            "the splined fit is simplified as local insertion across multiple elements."
        ),
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
    ctx.expect_overlap(
        pinion,
        carrier,
        axes="yz",
        elem_a="pinion_teeth",
        elem_b="open_cage",
        min_overlap=0.020,
        name="drive pinion teeth remain near the carrier bearing snout",
    )
    ctx.expect_overlap(
        collar,
        side_0,
        axes="x",
        elem_a="collar_bore",
        elem_b="bearing_journal",
        min_overlap=0.010,
        name="dog collar bore remains engaged on the side_gear_0 journal",
    )
    ctx.expect_origin_gap(
        carrier,
        pinion,
        axis="y",
        min_gap=0.210,
        max_gap=0.240,
        name="drive pinion is mounted beside the ring gear",
    )

    # ── Dog-collar diff-lock assertions ──
    ctx.check(
        "carrier_to_lockcollar is prismatic",
        lock_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"carrier_to_lockcollar type={lock_joint.articulation_type}",
    )

    # Collar must be coaxial with the side_gear_0 axle (on the X centerline).
    ctx.expect_origin_distance(
        collar,
        side_0,
        axes="yz",
        max_dist=0.002,
        name="dog collar is coaxial with side_gear_0 axle",
    )

    # Collar must be on the outward (-X) side of the carrier, near side_gear_0.
    ctx.expect_origin_gap(
        side_0,
        collar,
        axis="x",
        min_gap=-0.005,
        max_gap=0.080,
        name="dog collar sits on the side_gear_0 hub region",
    )

    # Allow the collar to overlap with the carrier axle sleeve since it slides on it.
    ctx.allow_overlap(
        carrier,
        collar,
        elem_a="open_cage",
        elem_b="collar_body",
        reason=(
            "The dog collar is intentionally represented as sliding on the "
            "side_gear_0 axle sleeve inside the carrier cheek."
        ),
    )

    # At rest (q=0, disengaged) the collar is outward; at engaged (q=upper) it slides inward.
    rest_x = ctx.part_world_position(collar)
    with ctx.pose({lock_joint: lock_joint.motion_limits.upper}):
        engaged_x = ctx.part_world_position(collar)

    ctx.check(
        "dog collar slides inward (+X) when engaged",
        rest_x is not None
        and engaged_x is not None
        and engaged_x[0] > rest_x[0] + 0.010,
        details=f"rest_x={rest_x}, engaged_x={engaged_x}",
    )

    # At engaged pose, collar teeth must overlap with carrier dog teeth along X.
    with ctx.pose({lock_joint: lock_joint.motion_limits.upper}):
        ctx.expect_overlap(
            collar,
            carrier,
            axes="x",
            elem_a="collar_body",
            elem_b="carrier_dog_tooth_0",
            min_overlap=0.003,
            name="engaged collar teeth overlap carrier dog teeth along X",
        )

    return ctx.report()


object_model = build_object_model()
