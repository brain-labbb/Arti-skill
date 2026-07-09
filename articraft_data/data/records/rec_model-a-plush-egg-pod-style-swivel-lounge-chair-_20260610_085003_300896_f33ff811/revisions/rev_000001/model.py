from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# ----------------------------------------------------------------------------
# Plush pod lounge chair with a compact gas-lift pedestal.
#
# The reference is a continuous black upholstered bowl: a low rounded front lip,
# thick side arms, a high rounded back, leather creases on the inner seat/back,
# and a short chrome stem over a glossy black trumpet disc.
# ----------------------------------------------------------------------------

POD_TILT = 0.00
SOCKET_MOUTH_Z = 0.128
COLUMN_TOP_LOCAL = 0.124
POD_JOINT_LOCAL = 0.108
LIFT_TRAVEL = 0.065

BASE_PROFILE = [
    (0.000, 0.000),
    (0.120, 0.000),
    (0.180, 0.000),
    (0.180, 0.010),
    (0.162, 0.015),
    (0.138, 0.026),
    (0.108, 0.044),
    (0.078, 0.069),
    (0.052, 0.094),
    (0.036, 0.116),
    (0.030, 0.128),
    (0.024, 0.128),
    (0.024, 0.030),
    (0.000, 0.030),
]

def _ellipsoid(
    rx: float,
    ry: float,
    rz: float,
    cx: float,
    cy: float,
    cz: float,
    *,
    width_segments: int = 28,
    height_segments: int = 20,
) -> MeshGeometry:
    geo = SphereGeometry(
        1.0,
        width_segments=width_segments,
        height_segments=height_segments,
    )
    geo.scale(rx, ry, rz)
    geo.translate(cx, cy, cz)
    return geo


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="plush_pod_swivel_lounge_chair")

    leather = model.material("leather_matte_black", rgba=(0.052, 0.052, 0.048, 1.0))
    leather_soft = model.material("leather_inner_soft_black", rgba=(0.070, 0.070, 0.066, 1.0))
    crease_dark = model.material("leather_recess_shadow", rgba=(0.025, 0.025, 0.023, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.84, 0.85, 0.88, 1.0))
    gloss_black = model.material("gloss_black", rgba=(0.038, 0.038, 0.042, 1.0))

    base = model.part("trumpet_base")
    base.visual(
        mesh_from_geometry(LatheGeometry(BASE_PROFILE, segments=96), "trumpet_base"),
        material=gloss_black,
        name="trumpet_shell",
    )

    column = model.part("lift_column")
    shaft_bottom_local = 0.030 - SOCKET_MOUTH_Z
    shaft_len = COLUMN_TOP_LOCAL - shaft_bottom_local
    column.visual(
        Cylinder(radius=0.020, length=shaft_len),
        origin=Origin(xyz=(0.0, 0.0, shaft_bottom_local + shaft_len / 2.0)),
        material=chrome,
        name="lift_shaft",
    )
    column.visual(
        Cylinder(radius=0.030, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, POD_JOINT_LOCAL - 0.009)),
        material=chrome,
        name="top_collar",
    )

    model.articulation(
        "gas_lift",
        ArticulationType.PRISMATIC,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, SOCKET_MOUTH_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.10, lower=0.0, upper=LIFT_TRAVEL),
    )

    pod = model.part("seat_pod")

    shell_geo = _ellipsoid(0.288, 0.322, 0.138, 0.075, 0.000, 0.210)
    shell_geo.merge(_ellipsoid(0.195, 0.300, 0.255, -0.105, 0.000, 0.405))
    shell_geo.merge(_ellipsoid(0.198, 0.112, 0.205, 0.055, 0.258, 0.335))
    shell_geo.merge(_ellipsoid(0.198, 0.112, 0.205, 0.055, -0.258, 0.335))
    shell_geo.merge(_ellipsoid(0.165, 0.300, 0.054, 0.235, 0.000, 0.165))
    shell_geo.merge(_ellipsoid(0.060, 0.060, 0.060, 0.000, 0.000, 0.070))
    pod.visual(
        mesh_from_geometry(shell_geo, "pod_shell"),
        material=leather,
        name="pod_shell",
    )

    inner_geo = _ellipsoid(0.188, 0.215, 0.060, 0.105, 0.000, 0.284)
    inner_geo.merge(_ellipsoid(0.112, 0.160, 0.150, -0.045, 0.000, 0.405))
    inner_geo.merge(_ellipsoid(0.066, 0.054, 0.105, 0.070, 0.182, 0.350))
    inner_geo.merge(_ellipsoid(0.066, 0.054, 0.105, 0.070, -0.182, 0.350))
    inner_geo.merge(_ellipsoid(0.044, 0.136, 0.100, 0.050, 0.000, 0.405))
    inner_geo.merge(_ellipsoid(0.148, 0.144, 0.020, 0.158, 0.000, 0.310))
    pod.visual(
        mesh_from_geometry(inner_geo, "inner_padded_bowl"),
        material=leather_soft,
        name="inner_bowl",
    )

    crease_geo = MeshGeometry()
    for y in (-0.105, -0.070, -0.035, 0.000, 0.035, 0.070, 0.105):
        crease_geo.merge(_ellipsoid(0.018, 0.009, 0.088, 0.050, y, 0.405))
        crease_geo.merge(_ellipsoid(0.122, 0.009, 0.016, 0.132, y, 0.318))
    for y in (-0.105, -0.070, -0.035, 0.000, 0.035, 0.070, 0.105):
        crease_geo.merge(_ellipsoid(0.094, 0.009, 0.014, 0.205, y, 0.292))
    pod.visual(
        mesh_from_geometry(crease_geo, "inner_wrinkle_shadows"),
        material=crease_dark,
        name="wrinkle_shadows",
    )

    model.articulation(
        "pod_swivel",
        ArticulationType.CONTINUOUS,
        parent=column,
        child=pod,
        origin=Origin(xyz=(0.0, 0.0, POD_JOINT_LOCAL)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("trumpet_base")
    column = object_model.get_part("lift_column")
    pod = object_model.get_part("seat_pod")
    lift = object_model.get_articulation("gas_lift")
    swivel = object_model.get_articulation("pod_swivel")

    ctx.allow_overlap(
        pod,
        column,
        elem_a="pod_shell",
        elem_b="lift_shaft",
        reason="Shaft tip intentionally tucks into the thick upholstered shell underside.",
    )
    ctx.allow_overlap(
        pod,
        column,
        elem_a="pod_shell",
        elem_b="top_collar",
        reason="Top collar is meant to seat into the chair underside at the swivel mount.",
    )
    ctx.allow_overlap(
        column,
        base,
        elem_a="lift_shaft",
        elem_b="trumpet_shell",
        reason="Gas-lift column intentionally nests inside the trumpet socket.",
    )

    base_aabb = ctx.part_world_aabb(base)
    assert base_aabb is not None
    base_dx = base_aabb[1][0] - base_aabb[0][0]
    base_dy = base_aabb[1][1] - base_aabb[0][1]
    ctx.check(
        "base reads as a compact round disc resting on the ground",
        0.34 <= base_dx <= 0.38
        and 0.34 <= base_dy <= 0.38
        and abs(base_aabb[0][2]) <= 1e-6
        and 0.11 <= base_aabb[1][2] <= 0.14,
        details=f"base aabb={base_aabb}",
    )

    pod_aabb = ctx.part_world_aabb(pod)
    assert pod_aabb is not None
    pod_dx = pod_aabb[1][0] - pod_aabb[0][0]
    pod_dy = pod_aabb[1][1] - pod_aabb[0][1]
    ctx.check(
        "chair body matches a compact lounge-chair envelope",
        0.58 <= pod_dx <= 0.74 and 0.64 <= pod_dy <= 0.82 and 0.82 <= pod_aabb[1][2] <= 0.92,
        details=f"pod aabb={pod_aabb}",
    )

    ctx.expect_within(
        column,
        base,
        axes="xy",
        inner_elem="lift_shaft",
        outer_elem="trumpet_shell",
        margin=0.0,
        name="column stays centered inside the base socket",
    )
    ctx.expect_overlap(
        column,
        base,
        axes="z",
        elem_a="lift_shaft",
        elem_b="trumpet_shell",
        min_overlap=0.060,
        name="column remains inserted into the base socket at rest",
    )
    ctx.expect_contact(
        column,
        base,
        elem_a="lift_shaft",
        elem_b="trumpet_shell",
        contact_tol=1e-4,
        name="lowered column rests on the socket floor",
    )

    ctx.expect_gap(
        pod,
        column,
        axis="z",
        max_penetration=0.050,
        max_gap=-0.004,
        name="chair body is seated onto the column top",
    )

    shaft_aabb = ctx.part_element_world_aabb(column, elem="lift_shaft")
    assert shaft_aabb is not None
    ctx.check(
        "an exposed chrome stem remains visible between base and shell",
        shaft_aabb[0][2] <= 0.04
        and shaft_aabb[1][2] >= 0.22
        and pod_aabb[0][2] - base_aabb[1][2] >= 0.05,
        details=f"lift_shaft aabb={shaft_aabb}, pod aabb={pod_aabb}, base aabb={base_aabb}",
    )

    shell_aabb = ctx.part_element_world_aabb(pod, elem="pod_shell")
    inner_aabb = ctx.part_element_world_aabb(pod, elem="inner_bowl")
    wrinkle_aabb = ctx.part_element_world_aabb(pod, elem="wrinkle_shadows")
    assert shell_aabb is not None and inner_aabb is not None and wrinkle_aabb is not None

    ctx.check(
        "inner padded bowl sits inside the continuous outer shell",
        inner_aabb[0][0] >= shell_aabb[0][0]
        and inner_aabb[1][0] <= shell_aabb[1][0] + 0.010
        and inner_aabb[0][1] >= shell_aabb[0][1]
        and inner_aabb[1][1] <= shell_aabb[1][1]
        and inner_aabb[1][2] <= shell_aabb[1][2],
        details=f"inner aabb={inner_aabb}, shell aabb={shell_aabb}",
    )
    ctx.check(
        "high rounded back rises well above the low front lip",
        shell_aabb[1][2] >= 0.86 and shell_aabb[1][0] - inner_aabb[1][0] >= 0.04,
        details=f"shell aabb={shell_aabb}, inner aabb={inner_aabb}",
    )
    ctx.check(
        "wrinkle shadows span the inner seat and back",
        wrinkle_aabb[1][2] >= 0.54
        and wrinkle_aabb[0][2] <= 0.52
        and 0.22 <= (wrinkle_aabb[1][1] - wrinkle_aabb[0][1]) <= 0.34,
        details=f"wrinkle_shadows aabb={wrinkle_aabb}",
    )
    ctx.check(
        "shell opening is visibly biased rather than perfectly centered",
        abs(shell_aabb[0][0] + shell_aabb[1][0]) >= 0.02,
        details=f"shell x range=({shell_aabb[0][0]:.4f}, {shell_aabb[1][0]:.4f})",
    )

    with ctx.pose({swivel: math.pi}):
        turned_aabb = ctx.part_world_aabb(pod)
        assert turned_aabb is not None
        ctx.check(
            "half-turn swivel preserves the chair envelope while rotating the pod",
            abs((turned_aabb[1][2] - turned_aabb[0][2]) - (pod_aabb[1][2] - pod_aabb[0][2])) <= 0.01
            and abs((turned_aabb[1][1] - turned_aabb[0][1]) - pod_dy) <= 0.01,
            details=f"turned pod aabb={turned_aabb}",
        )

    rest_top = pod_aabb[1][2]
    with ctx.pose({lift: LIFT_TRAVEL}):
        lifted_aabb = ctx.part_world_aabb(pod)
        assert lifted_aabb is not None
        ctx.check(
            "gas lift raises the chair by the full travel",
            abs(lifted_aabb[1][2] - rest_top - LIFT_TRAVEL) <= 0.004,
            details=f"rest top={rest_top:.4f}, lifted top={lifted_aabb[1][2]:.4f}",
        )
        ctx.expect_overlap(
            column,
            base,
            axes="z",
            elem_a="lift_shaft",
            elem_b="trumpet_shell",
            min_overlap=0.010,
            name="lifted column retains insertion in the socket",
        )
        ctx.expect_within(
            column,
            base,
            axes="xy",
            inner_elem="lift_shaft",
            outer_elem="trumpet_shell",
            margin=0.0,
            name="lifted column stays centered in the socket",
        )

    return ctx.report()


object_model = build_object_model()
