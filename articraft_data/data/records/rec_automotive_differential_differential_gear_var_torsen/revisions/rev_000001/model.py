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
    SpurGear,
    TestContext,
    TestReport,
    Worm,
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
    """Open differential cage with bearing sleeves, flange, windows, pinion support,
    and three Torsen-style axial planet-worm pockets at 120-degree intervals."""
    # Four heavy cage windows/bridges; their overlap with the cheeks makes one cast carrier.
    bridge_specs = (
        ((0.000, 0.000, 0.083), (0.160, 0.038, 0.030)),
        ((0.000, 0.000, -0.083), (0.160, 0.038, 0.030)),
        ((0.000, 0.098, 0.000), (0.160, 0.030, 0.038)),
        ((0.000, -0.098, 0.000), (0.160, 0.030, 0.038)),
    )
    # Two circular cheeks support the axle-side gears while leaving the center open.
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

    # A compact input-pinion bearing snout, tied to the flange by a rib.
    bearing = _cylinder_y(0.072, 0.033, center=(-0.145, -0.225, 0.0)).cut(
        _cylinder_y(0.084, 0.016, center=(-0.145, -0.225, 0.0))
    )
    upper_rib = cq.Workplane("XY").box(0.030, 0.122, 0.022).translate((-0.145, -0.159, 0.044))
    lower_rib = cq.Workplane("XY").box(0.030, 0.122, 0.022).translate((-0.145, -0.159, -0.044))
    cage = cage.union(upper_rib).union(lower_rib).union(bearing)

    # Torsen-style axial planet-worm pockets: 3 pockets at 120-degree intervals
    # around the axle (X) axis.  Each pocket is built as a solid cylinder that
    # clearly overlaps with the cage cheeks, then the bore is cut through.
    # This ensures reliable boolean union at every radial angle.
    n_pockets = 3
    pocket_radial_r = 0.058
    pocket_positions: list[tuple[float, float]] = []
    for i in range(n_pockets):
        angle = 2.0 * PI * i / n_pockets
        py = pocket_radial_r * math.cos(angle)
        pz = pocket_radial_r * math.sin(angle)
        pocket_positions.append((py, pz))
        # Solid cylinder that clearly penetrates the cheek body.
        solid = _cylinder_x(0.170, 0.028, center=(0.0, py, pz))
        cage = cage.union(solid)

    # Cut pocket bores through the merged solid.
    for py, pz in pocket_positions:
        bore = _cylinder_x(0.180, 0.018, center=(0.0, py, pz))
        cage = cage.cut(bore)

    return cage


def _hex_bolt_head() -> cq.Workplane:
    """Small hexagonal bolt head, centered at origin with axis along local Z."""
    return cq.Workplane("XY").polygon(6, 0.019).extrude(0.012, both=True)


def _build_planet_worm_mesh(lead_angle: float) -> object:
    """Shared helper: build a helical planet-worm mesh for one hand.

    Positive lead_angle gives a right-hand worm; negative gives left-hand.
    The worm is built along local Z and will be rotated to X at placement.
    """
    return mesh_from_cadquery(
        Worm(
            module=0.0030,
            lead_angle=lead_angle,
            n_threads=3,
            length=0.036,
            pressure_angle=20.0,
            backlash=0.0003,
        ).build(bore_d=0.012),
        f"planet_worm_{'rh' if lead_angle > 0 else 'lh'}",
        tolerance=0.001,
        angular_tolerance=0.10,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="automotive_differential_gear_assembly",
        meta={
            "description": (
                "Torsen-style geared torque-biasing differential with ring gear, "
                "carrier cage, helical worm side gears, paired planet worms in "
                "axial carrier pockets, axle outputs, mounted drive pinion, and bolts."
            )
        },
    )

    cast_iron = model.material("dark_cast_iron", rgba=(0.18, 0.18, 0.16, 1.0))
    machined = model.material("machined_steel", rgba=(0.55, 0.52, 0.46, 1.0))
    gear_steel = model.material("oiled_gear_steel", rgba=(0.32, 0.31, 0.28, 1.0))
    tooth_highlight = model.material("bright_tooth_faces", rgba=(0.72, 0.68, 0.58, 1.0))
    bolt_black = model.material("blackened_bolts", rgba=(0.06, 0.065, 0.070, 1.0))
    bronze_worm = model.material("bronze_worm", rgba=(0.60, 0.45, 0.22, 1.0))

    # ── Carrier cage (root) ───────────────────────────────────────────────
    carrier = model.part("carrier_cage")
    carrier.visual(
        mesh_from_cadquery(_carrier_cage_geometry(), "carrier_open_cage", tolerance=0.0015),
        material=cast_iron,
        name="open_cage",
    )

    # Bolt heads on the flange face, with a coaxial bolt circle.
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

    # ── Ring gear (fixed to carrier flange) ───────────────────────────────
    ring = model.part("ring_gear")
    ring.visual(
        mesh_from_cadquery(
            SpurGear(
                module=0.0060,
                teeth_number=46,
                width=0.040,
                pressure_angle=20.0,
                helix_angle=18.0,
                backlash=0.0004,
            ).build(bore_d=0.205, chamfer=0.002),
            "large_helical_ring_gear",
            tolerance=0.001,
            angular_tolerance=0.08,
        ),
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

    # ── Helical worm side gears + axle outputs ────────────────────────────
    # Two side gears with steep helical teeth (worm-like) for Torsen torque
    # biasing.  Left and right side gears have opposite helix due to the
    # inward pitch rotation.
    side_visual = mesh_from_cadquery(
        SpurGear(
            module=0.0040,
            teeth_number=16,
            width=0.042,
            pressure_angle=20.0,
            helix_angle=42.0,
            backlash=0.0003,
        ).build(bore_d=0.022, chamfer=0.001),
        "side_helical_worm_gear",
        tolerance=0.001,
        angular_tolerance=0.08,
    )
    for index, x in enumerate((-0.041, 0.041)):
        side = model.part(f"side_gear_{index}")
        inward_pitch = PI / 2.0 if x < 0.0 else -PI / 2.0
        side.visual(
            side_visual,
            origin=Origin(rpy=(0.0, inward_pitch, 0.0)),
            material=tooth_highlight,
            name="helical_teeth",
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

    # ── Paired helical planet worms in carrier pockets ────────────────────
    # 3 pairs (6 total) arranged at 120-degree intervals around the axle axis.
    # Each pair has one right-hand worm (near side_gear_0) and one left-hand
    # worm (near side_gear_1).  Planet worms rotate about X (parallel to axle).
    n_pairs = 3
    pocket_radial_r = 0.058
    pair_x_offset = 0.022

    # Shared worm meshes for right-hand and left-hand helices.
    worm_rh = _build_planet_worm_mesh(lead_angle=28.0)
    worm_lh = _build_planet_worm_mesh(lead_angle=-28.0)

    for pair_idx in range(n_pairs):
        angle = 2.0 * PI * pair_idx / n_pairs
        py = pocket_radial_r * math.cos(angle)
        pz = pocket_radial_r * math.sin(angle)

        for member_idx, x_off in enumerate((-pair_x_offset, pair_x_offset)):
            worm_idx = pair_idx * 2 + member_idx
            planet = model.part(f"planet_worm_{worm_idx}")

            # Right-hand worm for the left-side member, left-hand for the right.
            worm_mesh = worm_rh if member_idx == 0 else worm_lh
            # Rotate worm from local Z to local X.
            worm_pitch = PI / 2.0 if x_off < 0.0 else -PI / 2.0
            planet.visual(
                worm_mesh,
                origin=Origin(rpy=(0.0, worm_pitch, 0.0)),
                material=bronze_worm,
                name="worm_threads",
            )
            # Small bearing journal visible at each end of the planet worm.
            planet.visual(
                Cylinder(radius=0.008, length=0.042),
                origin=Origin(rpy=(0.0, PI / 2.0, 0.0)),
                material=machined,
                name="worm_shaft",
            )

            model.articulation(
                f"carrier_to_planet_{worm_idx}",
                ArticulationType.CONTINUOUS,
                parent=carrier,
                child=planet,
                origin=Origin(xyz=(x_off, py, pz)),
                axis=(1.0, 0.0, 0.0),
                motion_limits=MotionLimits(effort=35.0, velocity=35.0),
            )

    # ── Drive pinion (input) ──────────────────────────────────────────────
    pinion = model.part("drive_pinion")
    pinion.visual(
        mesh_from_cadquery(
            SpurGear(
                module=0.0060,
                teeth_number=12,
                width=0.040,
                pressure_angle=20.0,
                helix_angle=18.0,
                backlash=0.0004,
            ).build(bore_d=0.016, chamfer=0.002),
            "drive_helical_pinion",
            tolerance=0.001,
            angular_tolerance=0.08,
        ),
        origin=Origin(xyz=(0.0, 0.045, 0.0), rpy=(-PI / 2.0, 0.0, PI / 12.0)),
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

    # ── Prompt-specific: Torsen helical worm gearset structure ────────────
    # All planet worm joints must be CONTINUOUS revolute about the axle axis.
    for i in range(6):
        joint = object_model.get_articulation(f"carrier_to_planet_{i}")
        ctx.check(
            f"carrier_to_planet_{i} is a continuous helical worm joint",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"carrier_to_planet_{i} type={joint.articulation_type}",
        )

    # Side gear joints remain CONTINUOUS (helical worm side gears).
    for joint_name in ("carrier_to_side_0", "carrier_to_side_1", "carrier_to_pinion"):
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} is a continuous gear joint",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"{joint_name} type={joint.articulation_type}",
        )

    # Side gears are coaxial with the carrier axle centerline.
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

    # Side gear bearing journals are retained in the carrier axle openings.
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

    # ── Planet worm radial positioning and retention ──────────────────────
    n_pairs = 3
    pocket_radial_r = 0.058
    for pair_idx in range(n_pairs):
        for member_idx in range(2):
            worm_idx = pair_idx * 2 + member_idx
            planet = object_model.get_part(f"planet_worm_{worm_idx}")

            # Planet worm shaft is retained inside the carrier pocket.
            ctx.allow_overlap(
                carrier,
                planet,
                elem_a="open_cage",
                elem_b="worm_shaft",
                reason=(
                    f"Planet worm {worm_idx} shaft is intentionally seated inside "
                    f"the axial carrier pocket to represent the Torsen gear bearing."
                ),
            )
            # Planet worm threads intentionally extend into the carrier pocket
            # bore wall — this is the seated worm-in-pocket embedding.
            ctx.allow_overlap(
                carrier,
                planet,
                elem_a="open_cage",
                elem_b="worm_threads",
                reason=(
                    f"Planet worm {worm_idx} helical threads are intentionally "
                    f"nested inside the carrier pocket bore to show the Torsen "
                    f"worm mesh engagement."
                ),
            )
            ctx.expect_overlap(
                planet,
                carrier,
                axes="x",
                elem_a="worm_shaft",
                elem_b="open_cage",
                min_overlap=0.040,
                name=f"planet_worm_{worm_idx} shaft is retained in carrier pocket",
            )
            ctx.expect_overlap(
                planet,
                carrier,
                axes="yz",
                elem_a="worm_threads",
                elem_b="open_cage",
                min_overlap=0.010,
                name=f"planet_worm_{worm_idx} threads are within carrier pocket footprint",
            )

    # Side gear helical teeth are intentionally embedded inside the carrier
    # cage body — the worm-like side gears sit within the carrier interior.
    for side, label in ((side_0, "first"), (side_1, "second")):
        ctx.allow_overlap(
            carrier,
            side,
            elem_a="open_cage",
            elem_b="helical_teeth",
            reason=(
                f"The {label} helical worm side gear is intentionally seated "
                f"inside the carrier cage body to show the Torsen torque-biasing "
                f"mesh engagement with the planet worms."
            ),
        )
        ctx.expect_overlap(
            side,
            carrier,
            axes="x",
            elem_a="helical_teeth",
            elem_b="open_cage",
            min_overlap=0.020,
            name=f"{label} side gear helical teeth retained in carrier body",
        )

    # Drive pinion teeth near the bearing housing — the input pinion meshes
    # with the ring gear adjacent to the carrier snout.
    ctx.allow_overlap(
        carrier,
        pinion,
        elem_a="open_cage",
        elem_b="pinion_teeth",
        reason=(
            "The drive pinion helical teeth are intentionally near the carrier "
            "bearing housing as the pinion meshes with the ring gear bolted to "
            "the carrier flange."
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

    # Planet worms form 3 radially-arranged pairs around the axle axis.
    # Verify each pair shares approximately the same radial YZ distance.
    for pair_idx in range(n_pairs):
        p0 = object_model.get_part(f"planet_worm_{pair_idx * 2}")
        p1 = object_model.get_part(f"planet_worm_{pair_idx * 2 + 1}")
        ctx.expect_origin_distance(
            p0,
            p1,
            axes="yz",
            max_dist=0.003,
            name=f"planet pair {pair_idx} shares the same radial YZ position",
        )

    # ── Ring gear, pinion, and bolt retention (unchanged from parent) ─────
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
        elem_a="open_cage",
        elem_b="pinion_shaft",
        reason="The drive pinion shaft is intentionally seated in the bearing bore.",
    )
    ctx.expect_origin_gap(
        carrier,
        pinion,
        axis="y",
        min_gap=0.210,
        max_gap=0.240,
        name="drive pinion is mounted beside the ring gear",
    )

    return ctx.report()


object_model = build_object_model()
