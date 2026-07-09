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

    # ── Underside roller bogie parameters ─────────────────────────────
    _ROLLER_COUNT = 4
    _ROLLER_SPACING = 0.06
    _ROLLER_X_START = -(_ROLLER_COUNT - 1) * _ROLLER_SPACING / 2.0
    _ROLLER_Z = -0.012  # Roller center below hull bottom
    _AXLE_RADIUS = 0.0018
    _AXLE_LENGTH = 0.026
    _ROLLER_RADIUS = 0.008
    _ROLLER_LENGTH = 0.018
    _BRACKET_HEIGHT = 0.014  # Taller to reach down to axle
    cyl_to_y = (math.pi / 2.0, 0.0, 0.0)

    # Fixed axle + fork bracket visuals on the body keel
    for i in range(_ROLLER_COUNT):
        x = _ROLLER_X_START + i * _ROLLER_SPACING
        # Fork-style brackets at axle ends (outside the roller width)
        fork_y_offset = (_ROLLER_LENGTH / 2.0) + 0.002  # Just outside roller
        for y_sign in [-1.0, 1.0]:
            body.visual(
                Box((0.006, 0.003, _BRACKET_HEIGHT)),
                origin=Origin(xyz=(x, y_sign * fork_y_offset, -_BRACKET_HEIGHT / 2.0)),
                material=metal,
                name=f"roller_bracket_{i}_{'L' if y_sign < 0 else 'R'}",
            )
        # Axle pin through the roller
        body.visual(
            Cylinder(radius=_AXLE_RADIUS, length=_AXLE_LENGTH),
            origin=Origin(xyz=(x, 0.0, _ROLLER_Z), rpy=cyl_to_y),
            material=metal,
            name=f"roller_axle_{i}",
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

    # ── Roller parts with continuous joints ───────────────────────────
    roller_mat = model.material("roller_steel", rgba=(0.45, 0.48, 0.46, 1.0))
    
    def _make_roller_part(i: int, x_pos: float):
        """Create one roller part with wheel visual and continuous joint."""
        roller = model.part(f"roller_{i}")
        # Wheel cylinder oriented along Y (transverse)
        roller.visual(
            Cylinder(radius=_ROLLER_RADIUS, length=_ROLLER_LENGTH),
            origin=Origin(rpy=cyl_to_y),
            material=roller_mat,
            name=f"roller_wheel_{i}",
        )
        # Continuous joint about Y axis (transverse rotation)
        model.articulation(
            f"body_to_roller_{i}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=roller,
            origin=Origin(xyz=(x_pos, 0.0, _ROLLER_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.2, velocity=12.0),
        )
        return roller

    for i in range(_ROLLER_COUNT):
        x = _ROLLER_X_START + i * _ROLLER_SPACING
        _make_roller_part(i, x)

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

    # ── Roller bogie tests (4 rollers with continuous joints) ──────
    roller_count = 4
    for i in range(roller_count):
        roller = object_model.get_part(f"roller_{i}")
        roller_joint = object_model.get_articulation(f"body_to_roller_{i}")
        
        # Verify roller part exists and has wheel visual
        ctx.check(
            f"roller_{i} part exists with wheel visual",
            roller is not None and roller.get_visual(f"roller_wheel_{i}") is not None,
            details=f"roller_{i} missing or lacks roller_wheel_{i} visual",
        )
        
        # Verify continuous joint exists and is about Y axis
        ctx.check(
            f"body_to_roller_{i} is continuous joint about Y axis",
            roller_joint is not None 
            and roller_joint.articulation_type == ArticulationType.CONTINUOUS
            and abs(roller_joint.axis[1]) > 0.99,
            details=f"joint type or axis mismatch for body_to_roller_{i}",
        )
        
        # Verify roller is positioned below the main hull
        ctx.expect_gap(
            body,
            roller,
            axis="z",
            positive_elem="carved_hull",
            negative_elem=f"roller_wheel_{i}",
            min_gap=0.001,
            name=f"roller_{i} hangs below the hull keel",
        )

    return ctx.report()


object_model = build_object_model()
