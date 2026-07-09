from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


SHOULDER_Z = 0.155
LOWER_ELBOW = (0.23, 0.0, 0.36)
UPPER_WRIST = (0.28, 0.0, 0.27)


def _origin_along_xz(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[Origin, float]:
    """Return an origin/length for a cylinder whose local Z axis follows a vector."""
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    beta = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    yaw = math.atan2(dy, dx) if abs(dx) > 1.0e-9 or abs(dy) > 1.0e-9 else 0.0
    return (
        Origin(
            xyz=((start[0] + end[0]) * 0.5, (start[1] + end[1]) * 0.5, (start[2] + end[2]) * 0.5),
            rpy=(0.0, beta, yaw),
        ),
        length,
    )


def _y_axis_origin(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(-math.pi / 2.0, 0.0, 0.0))


def _shade_shell_geometry() -> LatheGeometry:
    # A thin spun-metal frustum with a small rear bushing and a rounded front lip.
    # The local +Z axis is the optical axis; the mesh is rotated when mounted.
    return LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.042, 0.000),
            (0.047, 0.018),
            (0.060, 0.075),
            (0.079, 0.150),
        ],
        inner_profile=[
            (0.013, 0.000),
            (0.030, 0.018),
            (0.052, 0.075),
            (0.071, 0.150),
        ],
        segments=64,
        start_cap="flat",
        end_cap="round",
        lip_samples=8,
    )


def _shade_front_rim_geometry() -> TorusGeometry:
    # The torus is centered on the shade axis at the front opening.
    return TorusGeometry(radius=0.075, tube=0.0045, radial_segments=16, tubular_segments=56)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="articulated_desk_lamp")

    matte_black = model.material("matte_black", rgba=(0.025, 0.027, 0.030, 1.0))
    charcoal = model.material("charcoal", rgba=(0.09, 0.095, 0.105, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.63, 0.60, 0.54, 1.0))
    dark_joint = model.material("dark_joint", rgba=(0.16, 0.15, 0.14, 1.0))
    warm_brass = model.material("warm_brass", rgba=(0.86, 0.66, 0.34, 1.0))
    warm_light = model.material("warm_light", rgba=(1.0, 0.82, 0.40, 0.86))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.145, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=matte_black,
        name="weighted_disk",
    )
    base.visual(
        Cylinder(radius=0.102, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=charcoal,
        name="raised_plinth",
    )
    base.visual(
        Cylinder(radius=0.026, length=0.082),
        origin=Origin(xyz=(0.0, 0.0, 0.079)),
        material=brushed_steel,
        name="upright_post",
    )
    base.visual(
        Box((0.052, 0.132, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=dark_joint,
        name="shoulder_bridge",
    )
    for sign in (-1.0, 1.0):
        base.visual(
            Box((0.038, 0.014, 0.068)),
            origin=Origin(xyz=(0.0, sign * 0.072, SHOULDER_Z)),
            material=dark_joint,
            name=f"shoulder_cheek_{0 if sign < 0.0 else 1}",
        )
    base.visual(
        Cylinder(radius=0.018, length=0.0148),
        origin=_y_axis_origin((0.0, -0.0598, SHOULDER_Z)),
        material=warm_brass,
        name="shoulder_cap_0",
    )
    base.visual(
        Cylinder(radius=0.018, length=0.0148),
        origin=_y_axis_origin((0.0, 0.0598, SHOULDER_Z)),
        material=warm_brass,
        name="shoulder_cap_1",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.145, length=0.034),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        Cylinder(radius=0.023, length=0.105),
        origin=_y_axis_origin((0.0, 0.0, 0.0)),
        material=dark_joint,
        name="shoulder_hub",
    )
    for sign in (-1.0, 1.0):
        rod_origin, rod_length = _origin_along_xz(
            (0.0, sign * 0.046, 0.0),
            (LOWER_ELBOW[0], sign * 0.046, LOWER_ELBOW[2]),
        )
        lower_arm.visual(
            Cylinder(radius=0.008, length=rod_length),
            origin=rod_origin,
            material=brushed_steel,
            name=f"lower_rod_{0 if sign < 0.0 else 1}",
        )
        lower_arm.visual(
            Box((0.044, 0.014, 0.056)),
            origin=Origin(xyz=(LOWER_ELBOW[0], sign * 0.057, LOWER_ELBOW[2])),
            material=dark_joint,
            name=f"elbow_cheek_{0 if sign < 0.0 else 1}",
        )
    lower_arm.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=_y_axis_origin((LOWER_ELBOW[0], -0.0445, LOWER_ELBOW[2])),
        material=warm_brass,
        name="elbow_cap_0",
    )
    lower_arm.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=_y_axis_origin((LOWER_ELBOW[0], 0.0445, LOWER_ELBOW[2])),
        material=warm_brass,
        name="elbow_cap_1",
    )
    lower_arm.inertial = Inertial.from_geometry(
        Box((0.29, 0.12, 0.42)),
        mass=0.45,
        origin=Origin(xyz=(0.12, 0.0, 0.18)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.022, length=0.082),
        origin=_y_axis_origin((0.0, 0.0, 0.0)),
        material=dark_joint,
        name="elbow_hub",
    )
    for sign in (-1.0, 1.0):
        rod_origin, rod_length = _origin_along_xz(
            (0.0, sign * 0.030, 0.0),
            (UPPER_WRIST[0], sign * 0.047, UPPER_WRIST[2]),
        )
        upper_arm.visual(
            Cylinder(radius=0.0075, length=rod_length),
            origin=rod_origin,
            material=brushed_steel,
            name=f"upper_rod_{0 if sign < 0.0 else 1}",
        )
        upper_arm.visual(
            Box((0.038, 0.012, 0.050)),
            origin=Origin(xyz=(UPPER_WRIST[0], sign * 0.039, UPPER_WRIST[2])),
            material=dark_joint,
            name=f"wrist_cheek_{0 if sign < 0.0 else 1}",
        )
    upper_arm.visual(
        Cylinder(radius=0.016, length=0.0095),
        origin=_y_axis_origin((UPPER_WRIST[0], -0.03065, UPPER_WRIST[2])),
        material=warm_brass,
        name="wrist_cap_0",
    )
    upper_arm.visual(
        Cylinder(radius=0.016, length=0.0095),
        origin=_y_axis_origin((UPPER_WRIST[0], 0.03065, UPPER_WRIST[2])),
        material=warm_brass,
        name="wrist_cap_1",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.34, 0.09, 0.34)),
        mass=0.36,
        origin=Origin(xyz=(0.14, 0.0, 0.14)),
    )

    lamp_head = model.part("lamp_head")
    head_axis = (0.120, 0.0, -0.100)
    head_axis_len = math.sqrt(head_axis[0] * head_axis[0] + head_axis[2] * head_axis[2])
    head_dir = (head_axis[0] / head_axis_len, 0.0, head_axis[2] / head_axis_len)
    shade_beta = math.atan2(head_dir[0], head_dir[2])
    shade_rear = (0.075, 0.0, -0.055)
    neck_origin, neck_length = _origin_along_xz((0.0, 0.0, 0.0), shade_rear)
    lamp_head.visual(
        Cylinder(radius=0.021, length=0.052),
        origin=_y_axis_origin((0.0, 0.0, 0.0)),
        material=dark_joint,
        name="wrist_hub",
    )
    lamp_head.visual(
        Cylinder(radius=0.014, length=neck_length),
        origin=neck_origin,
        material=dark_joint,
        name="neck",
    )
    lamp_head.visual(
        mesh_from_geometry(_shade_shell_geometry(), "shade_shell"),
        origin=Origin(xyz=shade_rear, rpy=(0.0, shade_beta, 0.0)),
        material=matte_black,
        name="shade_shell",
    )
    lamp_head.visual(
        mesh_from_geometry(_shade_front_rim_geometry(), "front_rim"),
        origin=Origin(
            xyz=(
                shade_rear[0] + head_dir[0] * 0.150,
                0.0,
                shade_rear[2] + head_dir[2] * 0.150,
            ),
            rpy=(0.0, shade_beta, 0.0),
        ),
        material=warm_brass,
        name="front_rim",
    )
    lamp_head.visual(
        Cylinder(radius=0.022, length=0.085),
        origin=Origin(
            xyz=(
                shade_rear[0] + head_dir[0] * 0.042,
                0.0,
                shade_rear[2] + head_dir[2] * 0.042,
            ),
            rpy=(0.0, shade_beta, 0.0),
        ),
        material=dark_joint,
        name="socket",
    )
    lamp_head.visual(
        Sphere(radius=0.024),
        origin=Origin(
            xyz=(
                shade_rear[0] + head_dir[0] * 0.078,
                0.0,
                shade_rear[2] + head_dir[2] * 0.078,
            )
        ),
        material=warm_light,
        name="bulb",
    )
    lamp_head.inertial = Inertial.from_geometry(
        Box((0.24, 0.16, 0.19)),
        mass=0.42,
        origin=Origin(xyz=(0.12, 0.0, -0.08)),
    )

    model.articulation(
        "base_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=-0.70, upper=0.95),
        motion_properties=MotionProperties(damping=0.08, friction=0.04),
    )
    model.articulation(
        "lower_arm_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=LOWER_ELBOW),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.4, lower=-1.10, upper=1.20),
        motion_properties=MotionProperties(damping=0.07, friction=0.035),
    )
    model.articulation(
        "upper_arm_to_lamp_head",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=lamp_head,
        origin=Origin(xyz=UPPER_WRIST),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=7.5, velocity=1.8, lower=-1.05, upper=1.05),
        motion_properties=MotionProperties(damping=0.05, friction=0.03),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    lamp_head = object_model.get_part("lamp_head")
    shoulder = object_model.get_articulation("base_to_lower_arm")
    elbow = object_model.get_articulation("lower_arm_to_upper_arm")
    wrist = object_model.get_articulation("upper_arm_to_lamp_head")

    ctx.check(
        "desk lamp has base, two arms, and head",
        len(object_model.parts) == 4 and len(object_model.articulations) == 3,
        details=f"parts={len(object_model.parts)}, joints={len(object_model.articulations)}",
    )
    ctx.expect_contact(
        base,
        lower_arm,
        elem_a="shoulder_cap_0",
        elem_b="shoulder_hub",
        contact_tol=0.018,
        name="lower arm is captured in the shoulder yoke",
    )
    ctx.expect_contact(
        lower_arm,
        upper_arm,
        elem_a="elbow_cap_0",
        elem_b="elbow_hub",
        contact_tol=0.018,
        name="upper arm is captured in the elbow yoke",
    )
    ctx.expect_contact(
        upper_arm,
        lamp_head,
        elem_a="wrist_cap_0",
        elem_b="wrist_hub",
        contact_tol=0.016,
        name="lamp head is captured in the wrist yoke",
    )

    elbow_rest = ctx.part_world_position(upper_arm)
    wrist_rest = ctx.part_world_position(lamp_head)
    shade_aabb = ctx.part_element_world_aabb(lamp_head, elem="shade_shell")
    ctx.check(
        "rest pose forms an elevated task-light silhouette",
        elbow_rest is not None
        and wrist_rest is not None
        and shade_aabb is not None
        and elbow_rest[2] > SHOULDER_Z + 0.25
        and wrist_rest[0] > 0.45
        and shade_aabb[0][2] < wrist_rest[2] - 0.08,
        details=f"elbow={elbow_rest}, wrist={wrist_rest}, shade_aabb={shade_aabb}",
    )

    with ctx.pose({shoulder: 0.45, elbow: 0.35}):
        raised_elbow = ctx.part_world_position(upper_arm)
    with ctx.pose({wrist: 0.70}):
        tilted_shade_aabb = ctx.part_element_world_aabb(lamp_head, elem="shade_shell")
    ctx.check(
        "hinges raise the arms and the wrist tilts the shade downward",
        elbow_rest is not None
        and raised_elbow is not None
        and shade_aabb is not None
        and tilted_shade_aabb is not None
        and raised_elbow[2] > elbow_rest[2] + 0.05
        and tilted_shade_aabb[0][2] < shade_aabb[0][2] - 0.03,
        details=f"rest_elbow={elbow_rest}, raised_elbow={raised_elbow}, rest_shade={shade_aabb}, tilted_shade={tilted_shade_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
