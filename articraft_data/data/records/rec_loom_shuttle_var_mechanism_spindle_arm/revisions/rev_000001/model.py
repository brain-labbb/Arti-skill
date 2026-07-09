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
    # Small metal latch catch near the front bearing; the spindle arm hook
    # rests just below this when latched down.
    body.visual(
        Box((0.008, 0.010, 0.004)),
        origin=Origin(xyz=(-0.105, 0.0, 0.028)),
        material=metal,
        name="front_latch_notch",
    )

    # --- Pivoting spindle arm (hinged at the rear bearing) ---
    spindle_arm = model.part("spindle_arm")
    arm_length = 0.210
    # Thin steel rod, pivot pin at the rear end, latch hook at the front end.
    spindle_arm.visual(
        Cylinder(radius=0.0025, length=arm_length),
        origin=Origin(xyz=(-arm_length / 2.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="spindle_rod",
    )
    # Pivot pin with a small retention collar that contacts the rear bearing
    # torus, forming a captured-pin hinge. The collar radius exceeds the torus
    # inner bore so the pin is mechanically retained.
    spindle_arm.visual(
        Cylinder(radius=0.004, length=0.018),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="pivot_pin",
    )
    spindle_arm.visual(
        Cylinder(radius=0.0062, length=0.003),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="pivot_collar",
    )
    spindle_arm.visual(
        Box((0.012, 0.008, 0.006)),
        origin=Origin(xyz=(-arm_length + 0.006, 0.0, -0.001)),
        material=metal,
        name="latch_hook",
    )

    model.articulation(
        "body_to_spindle_arm",
        ArticulationType.REVOLUTE,
        parent=body,
        child=spindle_arm,
        # Hinge at the rear bearing saddle, axis Y so positive q swings the
        # free (front) end up out of the cavity for pirn loading.
        origin=Origin(xyz=(0.105, 0.0, 0.020)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0, lower=0.0, upper=1.0),
    )

    bobbin = model.part("bobbin")
    # The child frame sits on the spindle axis, midway along the arm; all
    # cylinder visuals rotate from local Z to local X so the pirn spins
    # lengthwise along the spindle rod.
    cyl_to_x = (0.0, math.pi / 2.0, 0.0)
    bobbin.visual(
        Cylinder(radius=0.0061, length=0.200),
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
        "spindle_arm_to_bobbin",
        ArticulationType.REVOLUTE,
        parent=spindle_arm,
        child=bobbin,
        # Pirn axis: midpoint of the arm rod, along X.
        origin=Origin(xyz=(-arm_length / 2.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0, lower=-math.pi, upper=math.pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    spindle_arm = object_model.get_part("spindle_arm")
    bobbin = object_model.get_part("bobbin")
    arm_hinge = object_model.get_articulation("body_to_spindle_arm")
    pirn_spin = object_model.get_articulation("spindle_arm_to_bobbin")

    # Bobbin stays inside the shuttle footprint when the arm is latched down.
    ctx.expect_within(
        bobbin,
        body,
        axes="xy",
        margin=0.004,
        name="bobbin is retained inside the shuttle footprint when arm is latched",
    )
    ctx.expect_overlap(
        bobbin,
        body,
        axes="x",
        min_overlap=0.12,
        name="bobbin spans the central hollow bay lengthwise",
    )
    ctx.expect_gap(
        bobbin,
        body,
        axis="z",
        min_gap=0.0003,
        negative_elem="cavity_floor",
        name="bobbin clears the carved cavity floor",
    )

    # The pivot pin passes through the hull at the rear bearing location
    # (captured hinge pin) and the retention collar is intentionally wider
    # than the bearing bore. Allow these local embeddings.
    ctx.allow_overlap(
        "body",
        "spindle_arm",
        elem_a="carved_hull",
        elem_b="pivot_pin",
        reason="The pivot pin is a captured hinge pin that passes through the hull "
        "at the rear bearing saddle; this is the hinge's mechanical retention.",
    )
    ctx.allow_overlap(
        "body",
        "spindle_arm",
        elem_a="rear_bearing",
        elem_b="pivot_collar",
        reason="The retention collar is intentionally wider than the bearing bore "
        "so the spindle arm is captured on the hinge pin.",
    )

    # Prove the captured hinge: pin is within the bearing region, collar
    # contacts the bearing, and latch hook seats near the front notch.
    ctx.expect_contact(
        spindle_arm,
        body,
        elem_a="pivot_collar",
        elem_b="rear_bearing",
        contact_tol=0.001,
        name="pivot collar is captured on the rear bearing",
    )
    ctx.expect_contact(
        spindle_arm,
        body,
        elem_a="latch_hook",
        elem_b="front_latch_notch",
        contact_tol=0.012,
        name="spindle_arm latch_hook seats at the front latch notch when closed",
    )

    # Swinging the arm up lifts the bobbin well above the body top.
    rest_pos = ctx.part_world_position(bobbin)
    with ctx.pose({arm_hinge: 1.0}):
        lifted_pos = ctx.part_world_position(bobbin)
    ctx.check(
        "spindle_arm hinge lifts the bobbin up out of the cavity for pirn loading",
        rest_pos is not None
        and lifted_pos is not None
        and lifted_pos[2] > rest_pos[2] + 0.03,
        details=f"rest_z={rest_pos[2] if rest_pos else None}, "
        f"lifted_z={lifted_pos[2] if lifted_pos else None}",
    )

    # Pirn spin: rotating the bobbin on the spindle moves the red thread tail
    # in Y/Z while keeping the bobbin X-center fixed.
    rest_tail = ctx.part_element_world_aabb(bobbin, elem="red_thread")
    with ctx.pose({pirn_spin: 1.2}):
        turned_tail = ctx.part_element_world_aabb(bobbin, elem="red_thread")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return ((lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0, (lo[2] + hi[2]) / 2.0)

    rest_c = _aabb_center(rest_tail)
    turned_c = _aabb_center(turned_tail)
    ctx.check(
        "pirn spins on spindle_arm: thread mark moves while X-center stays fixed",
        rest_c is not None
        and turned_c is not None
        and abs(rest_c[0] - turned_c[0]) < 0.002
        and (abs(rest_c[1] - turned_c[1]) > 0.003 or abs(rest_c[2] - turned_c[2]) > 0.003),
        details=f"rest_tail_center={rest_c}, turned_tail_center={turned_c}",
    )

    return ctx.report()


object_model = build_object_model()
