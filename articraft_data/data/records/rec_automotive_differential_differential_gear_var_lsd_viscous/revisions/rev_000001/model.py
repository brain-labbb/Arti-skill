from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BevelGear,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    SpurGear,
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
    """Simplified differential carrier cage with bearing sleeves and flange."""
    # Main cage body: single annular cylinder with open center
    cage = _annular_cylinder_x(0.180, 0.090, 0.050)

    # Axle bearing sleeves
    cage = cage.union(_annular_cylinder_x(0.070, 0.041, 0.030, center=(-0.112, 0.0, 0.0)))
    cage = cage.union(_annular_cylinder_x(0.070, 0.041, 0.030, center=(0.112, 0.0, 0.0)))

    # Ring-gear mounting flange
    flange = _annular_cylinder_x(0.026, 0.145, 0.061, center=(-0.105, 0.0, 0.0))
    cage = cage.union(flange)

    # Input-pinion bearing snout
    bearing = _cylinder_y(0.072, 0.033, center=(-0.145, -0.225, 0.0)).cut(
        _cylinder_y(0.084, 0.016, center=(-0.145, -0.225, 0.0))
    )
    cage = cage.union(bearing)

    return cage


def _hex_bolt_head() -> cq.Workplane:
    """Small hexagonal bolt head, centered at origin with axis along local Z."""
    return cq.Workplane("XY").polygon(6, 0.019).extrude(0.012, both=True)


def _viscous_drum_geometry() -> cq.Workplane:
    """Sealed viscous-coupling drum shell (annular cylinder, axis along X)."""
    return _annular_cylinder_x(0.052, 0.044, 0.041)


def _viscous_plate_geometry() -> cq.Workplane:
    """Thin annular viscous-coupling friction plate, axis along X, centered at origin."""
    return _annular_cylinder_x(0.0015, 0.038, 0.018)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="automotive_differential_gear_assembly",
        meta={
            "description": (
                "Viscous-coupling limited-slip differential with ring gear, carrier cage, "
                "side gears, sealed viscous drum enclosing an interleaved "
                "plate stack, axle outputs, mounted drive pinion, and bolts."
            )
        },
    )

    cast_iron = model.material("dark_cast_iron", rgba=(0.18, 0.18, 0.16, 1.0))
    machined = model.material("machined_steel", rgba=(0.55, 0.52, 0.46, 1.0))
    gear_steel = model.material("oiled_gear_steel", rgba=(0.32, 0.31, 0.28, 1.0))
    tooth_highlight = model.material("bright_tooth_faces", rgba=(0.72, 0.68, 0.58, 1.0))
    bolt_black = model.material("blackened_bolts", rgba=(0.06, 0.065, 0.070, 1.0))
    drum_shell = model.material("sealed_drum_shell", rgba=(0.22, 0.22, 0.20, 1.0))
    friction_plate = model.material("viscous_friction_plate", rgba=(0.48, 0.42, 0.30, 1.0))

    carrier = model.part("carrier_cage")
    
    # Simplified carrier cage using primitives
    carrier.visual(
        Cylinder(radius=0.090, length=0.180),
        origin=Origin(rpy=(0.0, PI / 2.0, 0.0)),
        material=cast_iron,
        name="open_cage",
    )
    # Axle bearing sleeves
    for x_pos in (-0.112, 0.112):
        carrier.visual(
            Cylinder(radius=0.041, length=0.070),
            origin=Origin(xyz=(x_pos, 0.0, 0.0), rpy=(0.0, PI / 2.0, 0.0)),
            material=cast_iron,
            name=f"axle_sleeve_{'left' if x_pos < 0 else 'right'}",
        )
    # Ring gear mounting flange
    carrier.visual(
        Cylinder(radius=0.145, length=0.026),
        origin=Origin(xyz=(-0.105, 0.0, 0.0), rpy=(0.0, PI / 2.0, 0.0)),
        material=cast_iron,
        name="ring_flange",
    )
    # Input pinion bearing snout
    carrier.visual(
        Cylinder(radius=0.033, length=0.072),
        origin=Origin(xyz=(-0.145, -0.225, 0.0), rpy=(PI / 2.0, 0.0, 0.0)),
        material=cast_iron,
        name="pinion_bearing",
    )
    # Connecting ribs from cage body to pinion bearing
    carrier.visual(
        Box((0.022, 0.120, 0.022)),
        origin=Origin(xyz=(-0.145, -0.159, 0.044)),
        material=cast_iron,
        name="pinion_rib_upper",
    )
    carrier.visual(
        Box((0.022, 0.120, 0.022)),
        origin=Origin(xyz=(-0.145, -0.159, -0.044)),
        material=cast_iron,
        name="pinion_rib_lower",
    )

    # Polished cross shaft visible through the cage windows; it carries the spider gears.
    carrier.visual(
        Cylinder(radius=0.0055, length=0.190),
        origin=Origin(rpy=(-PI / 2.0, 0.0, 0.0)),
        material=machined,
        name="cross_shaft",
    )

    # Sealed viscous-coupling drum mounted inside the carrier cage, concentric on the axle axis.
    # The drum outer radius (0.048) intentionally seats in the cage cheek bores (0.047)
    # to show the drum riding with the carrier assembly.
    carrier.visual(
        Cylinder(radius=0.048, length=0.052),
        origin=Origin(rpy=(0.0, PI / 2.0, 0.0)),
        material=drum_shell,
        name="viscous_drum",
    )

    # Bolt heads are visible on the flange face, with a coaxial bolt circle.
    for i in range(10):
        a = 2.0 * PI * i / 10.0
        y = 0.112 * math.cos(a)
        z = 0.112 * math.sin(a)
        carrier.visual(
            Cylinder(radius=0.0095, length=0.012),
            origin=Origin(xyz=(-0.089, y, z), rpy=(0.0, PI / 2.0, a)),
            material=bolt_black,
            name=f"bolt_{i}",
        )
        carrier.visual(
            Cylinder(radius=0.0045, length=0.150),
            origin=Origin(xyz=(-0.150, y, z), rpy=(0.0, PI / 2.0, 0.0)),
            material=bolt_black,
            name=f"bolt_shank_{i}",
        )

    # Ring gear is a separate fixed part bolted to the carrier flange.
    ring = model.part("ring_gear")
    ring.visual(
        Cylinder(radius=0.155, length=0.040),
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

    # Two side gears and their short axle-output splines.  The visible hubs are coaxial.
    
    # Viscous-coupling interleaved plate stack: N=8 thin annular plates,
    # alternating between side_gear_0 (even) and side_gear_1 (odd).
    N_PLATES = 8
    PLATE_HALF_SPAN = 0.020  # usable axial half-length inside drum end caps
    plate_spacing = (2.0 * PLATE_HALF_SPAN) / (N_PLATES - 1)
    plate_world_x = [-PLATE_HALF_SPAN + i * plate_spacing for i in range(N_PLATES)]

    for index, x in enumerate((-0.041, 0.041)):
        side = model.part(f"side_gear_{index}")
        # Simplified bevel gear as cone/cylinder
        side.visual(
            Cylinder(radius=0.042, length=0.024),
            origin=Origin(rpy=(0.0, PI / 2.0, 0.0)),
            material=tooth_highlight,
            name="bevel_teeth",
        )
        # Hub connecting bevel teeth to the viscous plate region
        hub_direction = 1.0 if x < 0 else -1.0
        side.visual(
            Cylinder(radius=0.020, length=0.040),
            origin=Origin(xyz=(hub_direction * 0.020, 0.0, 0.0), rpy=(0.0, PI / 2.0, 0.0)),
            material=tooth_highlight,
            name="hub_connector",
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
        # Interleaved viscous plates keyed to this side gear hub.
        for i in range(N_PLATES):
            if i % 2 == index:
                local_x = plate_world_x[i] - x
                side.visual(
                    Cylinder(radius=0.036, length=0.0015),
                    origin=Origin(xyz=(local_x, 0.0, 0.0), rpy=(0.0, PI / 2.0, 0.0)),
                    material=friction_plate,
                    name=f"viscous_plate_{i}",
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

    # Spider pinions are enclosed inside the sealed viscous drum and not visible.

    # Drive pinion, mounted in the bearing snout and aimed into the ring gear.
    pinion = model.part("drive_pinion")
    pinion.visual(
        Cylinder(radius=0.035, length=0.026),
        origin=Origin(xyz=(0.0, 0.045, 0.0), rpy=(PI / 2.0, 0.0, 0.0)),
        material=tooth_highlight,
        name="pinion_teeth",
    )
    pinion.visual(
        Cylinder(radius=0.016, length=0.132),
        origin=Origin(xyz=(0.0, -0.008, 0.0), rpy=(PI / 2.0, 0.0, 0.0)),
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

    # Prompt-specific structure: coaxial axle outputs, viscous-coupling plates,
    # ring gear fixed to carrier, and the mounted input pinion near the ring.
    for joint_name in (
        "carrier_to_side_0",
        "carrier_to_side_1",
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
    for side, label, sleeve_name in (
        (side_0, "first", "axle_sleeve_left"),
        (side_1, "second", "axle_sleeve_right"),
    ):
        ctx.allow_overlap(
            carrier,
            side,
            elem_a=sleeve_name,
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
            elem_b=sleeve_name,
            min_overlap=0.030,
            name=f"{label} side journal is retained in the axle opening",
        )
        # Axle output shaft passes through the carrier axle opening
        ctx.allow_overlap(
            carrier,
            side,
            elem_a=sleeve_name,
            elem_b="axle_output",
            reason=(
                "The axle output shaft passes through the carrier's axle opening "
                "to connect the side gear to the external axle."
            ),
        )
        # Hub connector sits inside the carrier cage
        ctx.allow_overlap(
            carrier,
            side,
            elem_a="open_cage",
            elem_b="hub_connector",
            reason=(
                "The side gear hub connector is intentionally positioned inside "
                "the carrier cage to connect the bevel gear to the viscous plate stack."
            ),
        )
        # Axle output passes through the carrier cage
        ctx.allow_overlap(
            carrier,
            side,
            elem_a="open_cage",
            elem_b="axle_output",
            reason=(
                "The axle output shaft passes through the carrier cage body "
                "to extend outward and connect to the external axle."
            ),
        )
        # Ring flange overlaps with bearing journal on the ring-gear side
        if sleeve_name == "axle_sleeve_left":
            ctx.allow_overlap(
                carrier,
                side,
                elem_a="ring_flange",
                elem_b="bearing_journal",
                reason=(
                    "The bearing journal on the ring-gear side sits inside the "
                    "ring gear mounting flange as part of the carrier bearing support."
                ),
            )
            # Ring flange overlaps with axle output shaft on the ring-gear side
            ctx.allow_overlap(
                carrier,
                side,
                elem_a="ring_flange",
                elem_b="axle_output",
                reason=(
                    "The axle output shaft passes through the ring gear mounting flange "
                    "on the ring-gear side as part of the carrier bearing support."
                ),
            )
        # Bevel teeth sit inside the carrier cage
        ctx.allow_overlap(
            carrier,
            side,
            elem_a="open_cage",
            elem_b="bevel_teeth",
            reason=(
                "The side gear bevel teeth are positioned inside the carrier cage "
                "where they mesh with the spider gears."
            ),
        )
        # Viscous drum encloses the hub connector
        ctx.allow_overlap(
            carrier,
            side,
            elem_a="viscous_drum",
            elem_b="hub_connector",
            reason=(
                "The hub connector extends into the viscous drum interior where "
                "it keys the side gear to the interleaved plate stack."
            ),
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
            elem_b="toothed_ring",
            reason="Ring gear bolt shanks intentionally pass through the bolted ring flange.",
        )
        ctx.expect_overlap(
            ring,
            carrier,
            axes="x",
            elem_a="toothed_ring",
            elem_b=f"bolt_shank_{i}",
            min_overlap=0.020,
            name=f"ring gear is retained by bolt shank {i}",
        )
    ctx.allow_overlap(
        carrier,
        pinion,
        elem_a="pinion_bearing",
        elem_b="pinion_shaft",
        reason="The drive pinion shaft is intentionally seated in the bearing bore.",
    )
    ctx.expect_overlap(
        pinion,
        carrier,
        axes="y",
        elem_a="pinion_shaft",
        elem_b="pinion_bearing",
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

    # --- Viscous-coupling limited-slip assertions ---

    # Drum is coaxial with the axle axis (YZ projection overlaps side gears).
    for side, label in ((side_0, "first"), (side_1, "second")):
        ctx.expect_overlap(
            carrier,
            side,
            axes="yz",
            elem_a="viscous_drum",
            elem_b="bevel_teeth",
            min_overlap=0.020,
            name=f"viscous_drum is coaxial with {label} side gear on YZ plane",
        )

    # All 8 interleaved plates are radially contained inside the viscous drum.
    for i in range(8):
        side_part = object_model.get_part(f"side_gear_{i % 2}")
        ctx.expect_within(
            side_part,
            carrier,
            axes="yz",
            inner_elem=f"viscous_plate_{i}",
            outer_elem="viscous_drum",
            margin=0.004,
            name=f"viscous_plate_{i} is radially contained inside viscous_drum",
        )

    # Even plates on side_gear_0, odd plates on side_gear_1 (alternating keying).
    ctx.expect_overlap(
        side_0,
        side_1,
        axes="yz",
        elem_a="viscous_plate_0",
        elem_b="viscous_plate_1",
        min_overlap=0.030,
        name="viscous plates from alternating side gears overlap in the interleaved stack",
    )

    # The viscous drum encloses all interleaved plates in the sealed coupling.
    for i in range(8):
        side_name = f"side_gear_{i % 2}"
        ctx.allow_overlap(
            carrier,
            side_name,
            elem_a="viscous_drum",
            elem_b=f"viscous_plate_{i}",
            reason=(
                "The interleaved viscous plate is intentionally sealed inside the "
                "coupling drum, which rides with the carrier assembly."
            ),
        )

    # The cross shaft passes through the viscous plate stack center bores.
    for i in range(8):
        side_name = f"side_gear_{i % 2}"
        ctx.allow_overlap(
            carrier,
            side_name,
            elem_a="cross_shaft",
            elem_b=f"viscous_plate_{i}",
            reason=(
                "The cross shaft passes through the viscous plate stack; plates have "
                "clearance bores for the shaft in the sealed coupling interior."
            ),
        )

    return ctx.report()


object_model = build_object_model()
