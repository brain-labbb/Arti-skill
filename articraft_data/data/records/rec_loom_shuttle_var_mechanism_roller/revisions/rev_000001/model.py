from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _boat_profile(length: float = 0.36, max_width: float = 0.056) -> list[tuple[float, float]]:
    """Smooth plan-view outline for a pointed wooden shuttle body."""
    half_l = length / 2.0
    half_w = max_width / 2.0
    top: list[tuple[float, float]] = []
    # Dense enough to read as smooth worn wood, but still lightweight.
    for i in range(25):
        x = -half_l + length * i / 24.0
        t = abs(x) / half_l
        if t >= 1.0:
            y = 0.0
        else:
            # Fine pointed ends with fuller mid-body sides, like a boat shuttle.
            y = half_w * (1.0 - t**1.75) ** 0.48
        top.append((x, y))
    bottom = [(x, -y) for x, y in reversed(top[1:-1])]
    return top + bottom


def _rounded_rect_profile(
    length: float,
    width: float,
    radius: float,
    *,
    segments: int = 8,
) -> list[tuple[float, float]]:
    """Centered rounded rectangle in XY, with length along X."""
    r = min(radius, length / 2.0 - 1e-5, width / 2.0 - 1e-5)
    hx = length / 2.0 - r
    hy = width / 2.0 - r
    centers = [(hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy)]
    angle_ranges = [
        (0.0, math.pi / 2.0),
        (math.pi / 2.0, math.pi),
        (math.pi, 3.0 * math.pi / 2.0),
        (3.0 * math.pi / 2.0, 2.0 * math.pi),
    ]
    pts: list[tuple[float, float]] = []
    for (cx, cy), (a0, a1) in zip(centers, angle_ranges):
        for j in range(segments + 1):
            a = a0 + (a1 - a0) * j / segments
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def _make_body_mesh():
    outer = _boat_profile()
    # One carved solid: full boat outline, with a blind rounded slot cut almost
    # to the bottom to leave a real thin wooden floor below the bobbin.
    return (
        cq.Workplane("XY")
        .polyline(outer)
        .close()
        .extrude(0.034)
        .faces(">Z")
        .workplane(centerOption="CenterOfBoundBox")
        .slot2D(0.225, 0.044)
        .cutBlind(-0.030)
    )


def _cone_tip_mesh(name: str, *, toward_positive_x: bool):
    cone = ConeGeometry(0.012, 0.030, radial_segments=32, closed=True)
    cone.rotate_y(math.pi / 2.0 if toward_positive_x else -math.pi / 2.0)
    return mesh_from_geometry(cone, name)


def _torus_mesh(name: str, *, normal: str):
    torus = TorusGeometry(0.0075, 0.0016, radial_segments=20, tubular_segments=36)
    if normal == "x":
        torus.rotate_y(math.pi / 2.0)
    elif normal == "y":
        torus.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(torus, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="loom_shuttle",
        meta={
            "reference_note": (
                "Modeled as the visible core object: a wooden loom shuttle with "
                "a hollow bobbin bay and rotating thread bobbin."
            )
        },
    )

    wood = model.material("worn_oiled_wood", rgba=(0.50, 0.25, 0.08, 1.0))
    dark_wood = model.material("dark_worn_cavity", rgba=(0.13, 0.075, 0.040, 1.0))
    grain = model.material("subtle_dark_grain", rgba=(0.24, 0.12, 0.04, 1.0))
    metal = model.material("bruised_steel", rgba=(0.55, 0.58, 0.56, 1.0))
    black = model.material("blackened_eyelet", rgba=(0.015, 0.018, 0.017, 1.0))
    yarn = model.material("aged_cotton_thread", rgba=(0.86, 0.78, 0.60, 1.0))
    red = model.material("red_thread_end", rgba=(0.85, 0.05, 0.03, 1.0))
    bobbin_wood = model.material("bobbin_end_wood", rgba=(0.34, 0.17, 0.06, 1.0))
    steel_roller = model.material("polished_steel_roller", rgba=(0.72, 0.74, 0.76, 1.0))
    dark_axle = model.material("dark_steel_axle", rgba=(0.30, 0.32, 0.34, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_make_body_mesh(), "carved_wooden_hull", tolerance=0.0007),
        material=wood,
        name="carved_hull",
    )
    # Dark, rubbed bottom of the carved bobbin bay, slightly proud of the floor
    # so the open hollow reads clearly from above.
    body.visual(
        Box((0.210, 0.030, 0.0012)),
        origin=Origin(xyz=(0.0, 0.0, 0.0043)),
        material=dark_wood,
        name="cavity_floor",
    )
    # Metal pointed shuttle noses, partly seated into the wooden ends.
    body.visual(
        _cone_tip_mesh("left_metal_tip", toward_positive_x=False),
        origin=Origin(xyz=(-0.171, 0.0, 0.018)),
        material=metal,
        name="front_metal_tip",
    )
    body.visual(
        _cone_tip_mesh("right_metal_tip", toward_positive_x=True),
        origin=Origin(xyz=(0.171, 0.0, 0.018)),
        material=metal,
        name="rear_metal_tip",
    )
    # Metal bearing eyelets at the ends of the bobbin bay. Their holes are
    # coaxial with the bobbin joint and sit on the cavity floor.
    for x, nm in [(-0.105, "front_bearing"), (0.105, "rear_bearing")]:
        body.visual(
            Box((0.006, 0.004, 0.008)),
            origin=Origin(xyz=(x, 0.0, 0.0076)),
            material=black,
            name=f"{nm}_saddle",
        )
        body.visual(
            _torus_mesh(f"{nm}_ring", normal="x"),
            origin=Origin(xyz=(x, 0.0, 0.020)),
            material=black,
            name=nm,
        )
    # Side eyelets and small dark drilled holes like the reference, with no text.
    for x, y, z, nm in [
        (-0.142, -0.016, 0.018, "side_eyelet_0"),
        (0.145, -0.014, 0.022, "side_eyelet_1"),
    ]:
        body.visual(
            _torus_mesh(f"{nm}_mesh", normal="y"),
            origin=Origin(xyz=(x, y, z)),
            material=black,
            name=nm,
        )
    for i, (x, y) in enumerate([(-0.104, 0.018), (-0.045, 0.023), (0.046, 0.023), (0.125, 0.014)]):
        body.visual(
            Cylinder(radius=0.0027, length=0.0010),
            origin=Origin(xyz=(x, y, 0.0343)),
            material=black,
            name=f"top_hole_{i}",
        )
    for i, (x, y, sx, yaw) in enumerate(
        [
            (-0.095, 0.021, 0.055, 0.05),
            (-0.020, 0.024, 0.070, -0.04),
            (0.065, 0.021, 0.060, 0.06),
            (-0.110, -0.021, 0.040, -0.08),
            (0.030, -0.024, 0.080, 0.03),
            (0.116, -0.016, 0.040, -0.06),
        ]
    ):
        body.visual(
            Box((sx, 0.0020, 0.0008)),
            origin=Origin(xyz=(x, y, 0.0339), rpy=(0.0, 0.0, yaw)),
            material=grain,
            name=f"wood_grain_{i}",
        )

    bobbin = model.part("bobbin")
    # The child frame is exactly on the bobbin axis; all cylinder visuals are
    # rotated from local Z to local X so the revolute axis is lengthwise.
    cyl_to_x = (0.0, math.pi / 2.0, 0.0)
    bobbin.visual(
        Cylinder(radius=0.0061, length=0.220),
        origin=Origin(rpy=cyl_to_x),
        material=bobbin_wood,
        name="bobbin_core",
    )
    bobbin.visual(
        Cylinder(radius=0.0132, length=0.142),
        origin=Origin(rpy=cyl_to_x),
        material=yarn,
        name="cotton_thread",
    )
    for x, nm in [(-0.078, "front_flange"), (0.078, "rear_flange")]:
        bobbin.visual(
            Cylinder(radius=0.0142, length=0.010),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=cyl_to_x),
            material=black,
            name=nm,
        )
        bobbin.visual(
            Cylinder(radius=0.0080, length=0.006),
            origin=Origin(xyz=(x * 1.03, 0.0, 0.0), rpy=cyl_to_x),
            material=bobbin_wood,
            name=f"{nm}_boss",
        )
    # Off-axis red thread tail makes bobbin rotation visually apparent.
    bobbin.visual(
        Box((0.014, 0.004, 0.003)),
        origin=Origin(xyz=(0.071, 0.001, 0.0138), rpy=(0.0, 0.0, 0.08)),
        material=red,
        name="red_thread",
    )

    model.articulation(
        "body_to_bobbin",
        ArticulationType.REVOLUTE,
        parent=body,
        child=bobbin,
        origin=Origin(xyz=(0.0, 0.0, 0.0197)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0, lower=-math.pi, upper=math.pi),
    )

    # --- Underside roller bogie ---
    # N=2 small steel rollers recessed into the keel underside so the shuttle
    # rolls smoothly on the loom race. Each roller has its own part and
    # continuous joint about the transverse Y axis.
    roller_radius = 0.005
    roller_length = 0.018  # along Y (transverse)
    axle_radius = 0.0014
    axle_length = 0.030  # spans across the hull width with some margin
    roller_z = -0.002  # axle center 2 mm below hull bottom; crown proud
    roller_positions = [
        (-0.090, "roller_0"),
        (0.090, "roller_1"),
    ]
    # Cylinder axis is local Z; rotate to align along Y for the roller/axle visuals.
    cyl_to_y = (math.pi / 2.0, 0.0, 0.0)

    def _add_roller(x_pos: float, name: str):
        """Create one roller part with wheel and axle visuals, joined to body."""
        rp = model.part(name)
        # Steel roller wheel
        rp.visual(
            Cylinder(radius=roller_radius, length=roller_length),
            origin=Origin(rpy=cyl_to_y),
            material=steel_roller,
            name=f"{name}_wheel",
        )
        # Dark steel axle through the roller
        rp.visual(
            Cylinder(radius=axle_radius, length=axle_length),
            origin=Origin(rpy=cyl_to_y),
            material=dark_axle,
            name=f"roller_axle_{name.split('_')[1]}",
        )
        # Continuous joint about transverse Y axis for rolling on the race.
        model.articulation(
            f"body_to_{name}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=rp,
            origin=Origin(xyz=(x_pos, 0.0, roller_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.2, velocity=20.0),
        )

    for x_pos, name in roller_positions:
        _add_roller(x_pos, name)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    bobbin = object_model.get_part("bobbin")
    joint = object_model.get_articulation("body_to_bobbin")

    ctx.expect_within(
        bobbin,
        body,
        axes="xy",
        margin=0.002,
        name="bobbin is retained inside the shuttle footprint",
    )
    ctx.expect_overlap(
        bobbin,
        body,
        axes="x",
        min_overlap=0.18,
        name="bobbin spans the central hollow bay lengthwise",
    )
    ctx.expect_gap(
        bobbin,
        body,
        axis="z",
        min_gap=0.0005,
        negative_elem="cavity_floor",
        name="bobbin clears the carved cavity floor",
    )
    ctx.expect_contact(
        bobbin,
        body,
        elem_a="bobbin_core",
        elem_b="front_bearing",
        contact_tol=0.0005,
        name="front bearing supports the bobbin axle",
    )
    ctx.expect_contact(
        bobbin,
        body,
        elem_a="bobbin_core",
        elem_b="rear_bearing",
        contact_tol=0.0005,
        name="rear bearing supports the bobbin axle",
    )

    rest = ctx.part_world_position(bobbin)
    rest_tail = ctx.part_element_world_aabb(bobbin, elem="red_thread")
    with ctx.pose({joint: 1.2}):
        turned = ctx.part_world_position(bobbin)
        turned_tail = ctx.part_element_world_aabb(bobbin, elem="red_thread")

    def _aabb_center_yz(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return ((lo[1] + hi[1]) / 2.0, (lo[2] + hi[2]) / 2.0)

    rest_yz = _aabb_center_yz(rest_tail)
    turned_yz = _aabb_center_yz(turned_tail)
    ctx.check(
        "revolute joint keeps bobbin centered while thread mark turns",
        rest is not None
        and turned is not None
        and abs(rest[0] - turned[0]) < 1e-6
        and rest_yz is not None
        and turned_yz is not None
        and abs(rest_yz[0] - turned_yz[0]) > 0.004,
        details=f"rest={rest}, turned={turned}, rest_tail={rest_yz}, turned_tail={turned_yz}",
    )

    # --- Roller bogie tests ---
    roller_0 = object_model.get_part("roller_0")
    roller_1 = object_model.get_part("roller_1")
    joint_r0 = object_model.get_articulation("body_to_roller_0")
    joint_r1 = object_model.get_articulation("body_to_roller_1")

    # Roller crowns are recessed into the keel with limited penetration into
    # the hull (body is above, roller is below on Z axis).
    ctx.expect_gap(
        body,
        roller_0,
        axis="z",
        min_gap=-0.005,
        max_gap=0.002,
        name="roller_0 crown is near the keel surface",
    )
    ctx.expect_gap(
        body,
        roller_1,
        axis="z",
        min_gap=-0.005,
        max_gap=0.002,
        name="roller_1 crown is near the keel surface",
    )

    # Rollers protrude below the hull bottom — the roller part minimum Z is
    # below the body minimum Z.
    body_aabb = ctx.part_world_aabb(body)
    r0_aabb = ctx.part_world_aabb(roller_0)
    r1_aabb = ctx.part_world_aabb(roller_1)
    ctx.check(
        "rollers protrude below the hull keel for race contact",
        body_aabb is not None
        and r0_aabb is not None
        and r1_aabb is not None
        and r0_aabb[0][2] < body_aabb[0][2] - 0.001
        and r1_aabb[0][2] < body_aabb[0][2] - 0.001,
        details=f"body_min_z={body_aabb[0][2]}, r0_min_z={r0_aabb[0][2]}, r1_min_z={r1_aabb[0][2]}",
    )

    # Rollers are within the body footprint on XY.
    ctx.expect_within(
        roller_0,
        body,
        axes="xy",
        margin=0.005,
        name="roller_0 is within the shuttle footprint",
    )
    ctx.expect_within(
        roller_1,
        body,
        axes="xy",
        margin=0.005,
        name="roller_1 is within the shuttle footprint",
    )

    # Symmetric placement along X.
    r0_pos = ctx.part_world_position(roller_0)
    r1_pos = ctx.part_world_position(roller_1)
    ctx.check(
        "rollers are symmetrically placed along the keel",
        r0_pos is not None
        and r1_pos is not None
        and abs(r0_pos[0] + r1_pos[0]) < 0.005
        and abs(r0_pos[1] - r1_pos[1]) < 0.005,
        details=f"roller_0={r0_pos}, roller_1={r1_pos}",
    )

    # Continuous joints allow free rotation — verify roller positions remain
    # stable under a non-zero joint angle (rollers spin in place).
    r0_rest = ctx.part_world_position(roller_0)
    with ctx.pose({joint_r0: 1.5, joint_r1: 1.5}):
        r0_spun = ctx.part_world_position(roller_0)
        r1_spun = ctx.part_world_position(roller_1)

    ctx.check(
        "roller continuous joints spin in place without translating",
        r0_rest is not None
        and r0_spun is not None
        and r1_spun is not None
        and abs(r0_rest[0] - r0_spun[0]) < 1e-6
        and abs(r0_rest[1] - r0_spun[1]) < 1e-6
        and abs(r0_rest[2] - r0_spun[2]) < 1e-6,
        details=f"rest={r0_rest}, spun_0={r0_spun}, spun_1={r1_spun}",
    )

    return ctx.report()


object_model = build_object_model()
