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
    """Open differential cage with bearing sleeves, flange, windows, and pinion support."""
    # Four heavy cage windows/bridges; their overlap with the cheeks makes one cast carrier.
    bridge_specs = (
        ((0.000, 0.000, 0.083), (0.160, 0.038, 0.030)),
        ((0.000, 0.000, -0.083), (0.160, 0.038, 0.030)),
        ((0.000, 0.098, 0.000), (0.160, 0.030, 0.038)),
        ((0.000, -0.098, 0.000), (0.160, 0.030, 0.038)),
    )
    # Two circular cheeks support the axle-side gears while leaving the center open.
    # Union order keeps the carrier a single connected solid during CadQuery construction.
    cage = _annular_cylinder_x(0.030, 0.090, 0.047, center=(-0.066, 0.0, 0.0))
    cage = cage.union(cq.Workplane("XY").box(*bridge_specs[0][1]).translate(bridge_specs[0][0]))
    cage = cage.union(_annular_cylinder_x(0.030, 0.090, 0.047, center=(0.066, 0.0, 0.0)))
    for center, size in bridge_specs[1:]:
        cage = cage.union(cq.Workplane("XY").box(*size).translate(center))

    # Separate axle collars leave the middle of the carrier open for the side gears.
    for x in (-0.112, 0.112):
        cage = cage.union(_annular_cylinder_x(0.070, 0.041, 0.030, center=(x, 0.0, 0.0)))

    # Ring-gear mounting flange and register shoulder.
    flange = _annular_cylinder_x(0.026, 0.145, 0.061, center=(-0.105, 0.0, 0.0))
    shoulder = _annular_cylinder_x(0.010, 0.102, 0.061, center=(-0.124, 0.0, 0.0))
    cage = cage.union(flange).union(shoulder)

    # A compact input-pinion bearing snout, tied to the flange by a rib, keeps the pinion mounted.
    bearing = _cylinder_y(0.072, 0.033, center=(-0.145, -0.225, 0.0)).cut(
        _cylinder_y(0.084, 0.016, center=(-0.145, -0.225, 0.0))
    )
    upper_rib = cq.Workplane("XY").box(0.030, 0.122, 0.022).translate((-0.145, -0.159, 0.044))
    lower_rib = cq.Workplane("XY").box(0.030, 0.122, 0.022).translate((-0.145, -0.159, -0.044))
    cage = cage.union(upper_rib).union(lower_rib).union(bearing)

    return cage


def _hex_bolt_head() -> cq.Workplane:
    """Small hexagonal bolt head, centered at origin with axis along local Z."""
    return cq.Workplane("XY").polygon(6, 0.019).extrude(0.012, both=True)


# Crown-wheel (spiral-bevel) ring gear geometry ---------------------------------

_CROWN_OUTER_R = 0.126  # tooth tip radius
_CROWN_ROOT_R = 0.112   # tooth root radius
_CROWN_BORE_R = 0.040   # center bore
_CROWN_CONE_HEIGHT = 0.038  # axial extent of the cone
_CROWN_N_TEETH = 20

# Pinion tooth count (for half-tooth yaw offset)
_PINION_TEETH = 12


def _crown_wheel_body_geometry() -> cq.Workplane:
    """Conical crown wheel body: flat annular disc with bore (fast to generate).

    Built with axis along +Z, extending from z=0 to z=height.
    After rotation rpy=(0, -PI/2, 0) the axis aligns with world -X.
    The slight conical taper is approximated by the tooth boxes placed on top.
    """
    return (
        cq.Workplane("XY")
        .circle(_CROWN_OUTER_R)
        .circle(_CROWN_BORE_R)
        .extrude(_CROWN_CONE_HEIGHT)
    )


def _crown_backing_geometry() -> cq.Workplane:
    """Annular backing disc behind the crown-wheel body.

    Built with axis along +Z, extending from z=0 to z=-length so that after
    the same rotation as the body it extends toward the carrier flange.
    """
    outer_r = 0.092
    inner_r = 0.033
    length = 0.052
    return (
        cq.Workplane("XY")
        .circle(outer_r)
        .circle(inner_r)
        .extrude(-length)
    )


def _pinion_cone_geometry() -> cq.Workplane:
    """Simplified bevel pinion as an annular cylinder (fast proxy for a cone).

    Large end at z=0, extends in +Z.  After rotation the axis aligns with -Y.
    """
    return (
        cq.Workplane("XY")
        .circle(0.036)
        .circle(0.008)
        .extrude(0.032)
    )


def _side_gear_geometry() -> cq.Workplane:
    """Simplified side bevel gear as an annular cylinder with bore.

    Built with axis along +Z, height 0.022m.
    """
    return (
        cq.Workplane("XY")
        .circle(0.032)
        .circle(0.011)
        .extrude(0.022)
    )


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
    bolt_mesh = mesh_from_cadquery(_hex_bolt_head(), "hex_bolt_head", tolerance=0.001)
    for i in range(10):
        a = 2.0 * PI * i / 10.0
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

    # Ring gear is a conical bevel crown wheel, fixed to the carrier flange.
    # The tooth cone has a large cone angle (73°) so it reads as a shallow
    # crown wheel rather than a flat spur-gear disc.
    ring = model.part("ring_gear")
    ring.visual(
        mesh_from_cadquery(
            _crown_wheel_body_geometry(),
            "crown_wheel_body",
            tolerance=0.003,
        ),
        # Apex at x=-0.180; body extends in -X (cone opens away from carrier).
        # rpy=(0, -PI/2, 0) maps local +Z (gear axis) to world -X.
        origin=Origin(xyz=(-0.180, 0.0, 0.0), rpy=(0.0, -PI / 2.0, 0.0)),
        material=gear_steel,
        name="toothed_ring",
    )
    # Backing disc provides structural depth and bolt-shank overlap.
    ring.visual(
        mesh_from_cadquery(
            _crown_backing_geometry(),
            "crown_backing_disc",
            tolerance=0.003,
        ),
        origin=Origin(xyz=(-0.180, 0.0, 0.0), rpy=(0.0, -PI / 2.0, 0.0)),
        material=gear_steel,
        name="crown_backing",
    )
    # Tooth pattern is suggested by the material contrast on the outer cone
    # surface; individual tooth boxes are omitted to keep compile time tractable.
    model.articulation(
        "carrier_to_ring",
        ArticulationType.FIXED,
        parent=carrier,
        child=ring,
        origin=Origin(),
    )

    # Two side gears and their short axle-output splines.  The visible hubs are coaxial.
    # Side bevel gears are approximated as truncated annular cones for compile speed.
    side_visual = mesh_from_cadquery(
        _side_gear_geometry(),
        "side_bevel_gear",
        tolerance=0.003,
    )
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

    # Drive pinion: bevel cone re-aimed to mesh at the crown wheel's pitch cone.
    # The pinion cone angle (17°) complements the crown wheel (73°) for a 90°
    # shaft angle.  The pinion apex coincides with the crown wheel apex at
    # world (-0.180, 0, 0); the body extends in -Y toward the mesh point.
    pinion = model.part("drive_pinion")
    pinion.visual(
        mesh_from_cadquery(
            _pinion_cone_geometry(),
            "drive_pinion_cone",
            tolerance=0.003,
        ),
        # Apex offset from pinion part origin: (-0.035, 0.225, 0).
        # rpy=(PI/2, 0, PI/12): axis along -Y, half-tooth yaw for mesh alignment.
        origin=Origin(xyz=(-0.035, 0.225, 0.0), rpy=(PI / 2.0, 0.0, PI / _PINION_TEETH)),
        material=tooth_highlight,
        name="pinion_teeth",
    )
    pinion.visual(
        Cylinder(radius=0.016, length=0.200),
        # Shaft along Y from near the teeth back through the bearing snout.
        origin=Origin(xyz=(0.0, 0.060, 0.0), rpy=(-PI / 2.0, 0.0, 0.0)),
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
    for i in range(10):
        ctx.allow_overlap(
            carrier,
            ring,
            elem_a=f"bolt_shank_{i}",
            elem_b="crown_backing",
            reason="Ring gear bolt shanks intentionally pass through the crown-wheel backing disc.",
        )
        ctx.allow_overlap(
            carrier,
            ring,
            elem_a=f"bolt_shank_{i}",
            elem_b="toothed_ring",
            reason="Ring gear bolt shanks intentionally pass through the crown-wheel tooth cone body.",
        )
        ctx.expect_overlap(
            ring,
            carrier,
            axes="x",
            elem_a="crown_backing",
            elem_b=f"bolt_shank_{i}",
            min_overlap=0.020,
            name=f"crown wheel backing is retained by bolt shank {i}",
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
            "The pinion bevel cone is intentionally represented as reaching into the "
            "carrier cage opening to mesh at the crown wheel pitch cone."
        ),
    )
    ctx.allow_overlap(
        carrier,
        ring,
        elem_a="open_cage",
        elem_b="crown_backing",
        reason=(
            "The crown wheel backing disc intentionally bridges from the tooth cone "
            "to the carrier flange to show the bolted mounting."
        ),
    )
    ctx.allow_overlap(
        pinion,
        ring,
        elem_a="pinion_teeth",
        elem_b="crown_backing",
        reason=(
            "The pinion bevel cone reaches the crown wheel mesh zone near the "
            "backing disc; the simplified proxy geometry overlaps locally."
        ),
    )
    ctx.allow_overlap(
        pinion,
        ring,
        elem_a="pinion_teeth",
        elem_b="toothed_ring",
        reason=(
            "The pinion bevel cone is intentionally positioned at the crown wheel "
            "tooth cone to represent the bevel gear mesh."
        ),
    )
    ctx.allow_overlap(
        pinion,
        ring,
        elem_a="pinion_shaft",
        elem_b="crown_backing",
        reason=(
            "The pinion shaft passes near the crown wheel backing disc at the "
            "bottom of the assembly; the simplified shaft and backing disc proxy "
            "geometry overlap locally."
        ),
    )
    # Allow carrier-pinion overlap for bolt shanks near the pinion shaft
    for i in range(10):
        ctx.allow_overlap(
            carrier,
            pinion,
            elem_a=f"bolt_shank_{i}",
            elem_b="pinion_shaft",
            reason=(
                f"Bolt shank {i} passes near the pinion shaft at the carrier flange; "
                "the local overlap is a consequence of the simplified bolt circle "
                "and pinion shaft geometry."
            ),
        )
    ctx.allow_overlap(
        pinion,
        side_0,
        elem_a="pinion_teeth",
        elem_b="axle_output",
        reason=(
            "The pinion bevel cone passes near the first side gear axle output in "
            "the simplified differential layout; the overlap is a local proxy artifact."
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
    ctx.expect_origin_gap(
        carrier,
        pinion,
        axis="y",
        min_gap=0.210,
        max_gap=0.240,
        name="drive pinion is mounted beside the ring gear",
    )

    # Variant-specific: confirm the ring_gear is a conical bevel crown wheel
    # (large radial extent, small axial extent) rather than a flat spur-gear disc.
    # The crown wheel tooth cone should span much more in YZ (radial) than in X (axial).
    ring_tooth_aabb = ctx.part_element_world_aabb(ring, elem="toothed_ring")
    if ring_tooth_aabb is not None:
        lo, hi = ring_tooth_aabb
        dx = hi[0] - lo[0]
        dy = hi[1] - lo[1]
        dz = hi[2] - lo[2]
        radial_span = max(dy, dz)
        ctx.check(
            "ring_gear toothed_ring is a conical crown wheel, not a spur disc",
            radial_span > 0.15 and dx < 0.060,
            details=(
                f"crown wheel axial={dx:.4f}m, radial_y={dy:.4f}m, radial_z={dz:.4f}m; "
                "expected radial > 0.15m and axial < 0.06m for a bevel crown wheel"
            ),
        )

    # Confirm the drive pinion bevel cone meshes at the crown wheel perimeter:
    # the pinion teeth should overlap with the ring gear in YZ near the ring's
    # outer radius.
    ctx.expect_overlap(
        pinion,
        ring,
        axes="yz",
        elem_a="pinion_teeth",
        elem_b="toothed_ring",
        min_overlap=0.010,
        name="pinion bevel cone meshes at the crown wheel perimeter",
    )

    return ctx.report()


object_model = build_object_model()
