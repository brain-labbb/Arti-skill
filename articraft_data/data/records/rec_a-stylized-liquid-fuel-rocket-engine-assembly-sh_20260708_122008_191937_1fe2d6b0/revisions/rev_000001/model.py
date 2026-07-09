from __future__ import annotations

"""Stylized liquid-fuel rocket engine on a gimbal thrust mount.

Built vertically, nozzle down: a large gray thrust bulkhead disc with rim
bolts at the top, riveted struts converging onto a central gimbal cross, and
three white spring actuators reaching toward the engine mounting plate. The
engine assembly hangs from a two-axis gimbal cross: white round mounting
plate, blue hub, white ellipsoid combustion chamber with a riveted seam,
blue clamp collar, black corrugated throat section, blue clamp ring, and a
big black ribbed nozzle bell with blue bands, plus a small blue turbopump
unit and a gray feed pipe loop.
"""

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------- dimensions
PIVOT_Z = 1.70  # world height of the gimbal cross pivot

DISC_R = 0.85
DISC_Z0, DISC_Z1 = 1.99, 2.09

STRUT_ANGLES = (45.0, 135.0, 225.0, 315.0)
ACTUATOR_ANGLES = (0.0, 120.0, 240.0)

# engine-local (pivot at z=0, nozzle pointing -z)
PLATE_R = 0.52
PLATE_Z0, PLATE_Z1 = -0.21, -0.15
CHAMBER_BASE = -0.76  # chamber egg sits from here up to the plate
COLLAR_Z = -0.79
CORRUG_TOP = -0.85
CORRUG_STEPS = 6
CORRUG_PITCH = 0.05
CLAMP_Z = -1.155
BELL_TOP = -1.19
BELL_EXIT = -1.65
BELL_H = BELL_TOP - BELL_EXIT  # 0.46

# bell outer radius profile (z above exit -> radius)
BELL_OUTER = [(0.00, 0.55), (0.16, 0.45), (0.32, 0.39), (BELL_H, 0.37)]


def _bell_outer_radius(z: float) -> float:
    for (z0, r0), (z1, r1) in zip(BELL_OUTER, BELL_OUTER[1:]):
        if z <= z1:
            t = (z - z0) / (z1 - z0)
            return r0 + t * (r1 - r0)
    return BELL_OUTER[-1][1]


def _radial_dir(theta: float, tilt: float) -> tuple[float, float, float]:
    return (math.sin(tilt) * math.cos(theta), math.sin(tilt) * math.sin(theta), math.cos(tilt))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gimbaled_rocket_engine")

    # ------------------------------------------------------------ materials
    bell_black = model.material("bell_black", rgba=(0.09, 0.09, 0.10, 1.0))
    corrug_dark = model.material("corrug_dark", rgba=(0.14, 0.14, 0.16, 1.0))
    accent_blue = model.material("accent_blue", rgba=(0.16, 0.45, 0.85, 1.0))
    shell_white = model.material("shell_white", rgba=(0.93, 0.94, 0.96, 1.0))
    struct_gray = model.material("struct_gray", rgba=(0.52, 0.54, 0.57, 1.0))
    disc_gray = model.material("disc_gray", rgba=(0.44, 0.46, 0.49, 1.0))
    pipe_gray = model.material("pipe_gray", rgba=(0.62, 0.64, 0.67, 1.0))
    detail_dark = model.material("detail_dark", rgba=(0.15, 0.16, 0.18, 1.0))

    # ------------------------------------------------------------ thrust bulkhead (root)
    bulkhead = model.part("thrust_bulkhead")
    bulkhead.visual(
        Cylinder(radius=DISC_R, length=DISC_Z1 - DISC_Z0),
        origin=Origin(xyz=(0.0, 0.0, (DISC_Z0 + DISC_Z1) / 2.0)),
        material=disc_gray,
        name="bulkhead_disc",
    )
    # thicker rim lip around the disc edge
    bulkhead.visual(
        Cylinder(radius=DISC_R + 0.02, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 2.04)),
        material=disc_gray,
        name="bulkhead_rim",
    )
    # white rim bolts on the back face
    for i in range(8):
        theta = math.radians(i * 45.0 + 22.5)
        bulkhead.visual(
            Cylinder(radius=0.035, length=0.10),
            origin=Origin(xyz=(0.70 * math.cos(theta), 0.70 * math.sin(theta), 2.135)),
            material=shell_white,
            name=f"rim_bolt_{i}",
        )
    # central hub stub the gimbal cross pins seat into
    bulkhead.visual(
        Cylinder(radius=0.07, length=DISC_Z0 - 1.72),
        origin=Origin(xyz=(0.0, 0.0, (1.72 + DISC_Z0) / 2.0)),
        material=struct_gray,
        name="gimbal_hub_stub",
    )
    # riveted struts converging from the disc rim onto the hub stub
    strut_len = math.hypot(0.48, 0.17) + 0.02
    strut_tilt = math.atan2(0.48, 0.17)
    for i, ang in enumerate(STRUT_ANGLES):
        theta = math.radians(ang)
        bulkhead.visual(
            Cylinder(radius=0.05, length=strut_len),
            origin=Origin(
                xyz=(0.31 * math.cos(theta), 0.31 * math.sin(theta), 1.905),
                rpy=(0.0, strut_tilt, theta),
            ),
            material=struct_gray,
            name=f"gimbal_strut_{i}",
        )
    # white spring actuators reaching down toward the mounting plate rim
    act_tilt = math.atan2(0.08, 0.43)
    act_len = math.hypot(0.08, 0.43) + 0.03
    for i, ang in enumerate(ACTUATOR_ANGLES):
        theta = math.radians(ang)
        center = (0.54 * math.cos(theta), 0.54 * math.sin(theta), 1.785)
        d = _radial_dir(theta, act_tilt)
        bulkhead.visual(
            Cylinder(radius=0.032, length=act_len),
            origin=Origin(xyz=center, rpy=(0.0, act_tilt, theta)),
            material=shell_white,
            name=f"actuator_piston_{i}",
        )
        # coil-spring rings around the piston
        for k, off in enumerate((-0.10, -0.045, 0.01)):
            bulkhead.visual(
                Cylinder(radius=0.055, length=0.016),
                origin=Origin(
                    xyz=(
                        center[0] + d[0] * off,
                        center[1] + d[1] * off,
                        center[2] + d[2] * off,
                    ),
                    rpy=(0.0, act_tilt, theta),
                ),
                material=pipe_gray,
                name=f"actuator_spring_{i}_{k}",
            )
        # blue retainer collar near the upper end
        bulkhead.visual(
            Cylinder(radius=0.046, length=0.03),
            origin=Origin(
                xyz=(
                    center[0] + d[0] * 0.13,
                    center[1] + d[1] * 0.13,
                    center[2] + d[2] * 0.13,
                ),
                rpy=(0.0, act_tilt, theta),
            ),
            material=accent_blue,
            name=f"actuator_collar_{i}",
        )

    # ------------------------------------------------------------ gimbal cross
    cross = model.part("gimbal_cross")
    cross.visual(
        Cylinder(radius=0.045, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=struct_gray,
        name="gimbal_pin_y",
    )
    cross.visual(
        Cylinder(radius=0.045, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=struct_gray,
        name="gimbal_pin_x",
    )

    model.articulation(
        "gimbal_pitch",
        ArticulationType.REVOLUTE,
        parent=bulkhead,
        child=cross,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=200.0, velocity=0.5, lower=-0.12, upper=0.12),
    )

    # ------------------------------------------------------------ engine assembly
    engine = model.part("engine_assembly")

    # blue hub seating the cross pins, carrying the mounting plate
    engine.visual(
        Cylinder(radius=0.075, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, -0.07)),
        material=accent_blue,
        name="mount_hub",
    )
    # white round engine mounting plate
    engine.visual(
        Cylinder(radius=PLATE_R, length=PLATE_Z1 - PLATE_Z0),
        origin=Origin(xyz=(0.0, 0.0, (PLATE_Z0 + PLATE_Z1) / 2.0)),
        material=shell_white,
        name="mounting_plate",
    )
    # white ellipsoid combustion chamber pod (revolved egg profile)
    chamber_profile = [
        (0.00, 0.000),
        (0.14, 0.015),
        (0.24, 0.090),
        (0.295, 0.220),
        (0.300, 0.300),
        (0.270, 0.400),
        (0.190, 0.490),
        (0.090, 0.540),
        (0.000, 0.550),
    ]
    engine.visual(
        mesh_from_geometry(LatheGeometry(chamber_profile, segments=56), "chamber_pod"),
        origin=Origin(xyz=(0.0, 0.0, CHAMBER_BASE)),
        material=shell_white,
        name="combustion_chamber",
    )
    # riveted seam ring around the chamber equator
    engine.visual(
        Cylinder(radius=0.303, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, CHAMBER_BASE + 0.30)),
        material=pipe_gray,
        name="chamber_seam",
    )
    # blue clamp collar under the chamber
    engine.visual(
        Cylinder(radius=0.175, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, COLLAR_Z)),
        material=accent_blue,
        name="throat_collar",
    )
    # black corrugated throat: stacked rings widening toward the bell
    for i in range(CORRUG_STEPS):
        t = i / (CORRUG_STEPS - 1)
        radius = 0.18 + t * (0.36 - 0.18) + (0.012 if i % 2 else -0.004)
        engine.visual(
            Cylinder(radius=radius, length=0.055),
            origin=Origin(xyz=(0.0, 0.0, CORRUG_TOP - i * CORRUG_PITCH)),
            material=corrug_dark,
            name=f"throat_corrugation_{i}",
        )
    # blue clamp ring between throat and bell
    engine.visual(
        Cylinder(radius=0.39, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, CLAMP_Z)),
        material=accent_blue,
        name="bell_clamp_ring",
    )
    # hollow black nozzle bell shell (closed lathe wall loop, open at the exit)
    bell_wall = [
        (0.55, 0.00),
        (0.45, 0.16),
        (0.39, 0.32),
        (0.37, BELL_H),
        (0.33, BELL_H),
        (0.35, 0.32),
        (0.41, 0.16),
        (0.51, 0.00),
    ]
    engine.visual(
        mesh_from_geometry(LatheGeometry(bell_wall, segments=64), "nozzle_bell_shell"),
        origin=Origin(xyz=(0.0, 0.0, BELL_EXIT)),
        material=bell_black,
        name="nozzle_bell",
    )
    # circumferential cooling ribs along the bell skirt
    for i in range(7):
        z_loc = 0.06 + i * 0.06
        engine.visual(
            Cylinder(radius=_bell_outer_radius(z_loc) + 0.012, length=0.02),
            origin=Origin(xyz=(0.0, 0.0, BELL_EXIT + z_loc)),
            material=bell_black,
            name=f"bell_rib_{i}",
        )
    # blue band at the nozzle exit rim
    engine.visual(
        Cylinder(radius=0.558, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, BELL_EXIT + 0.018)),
        material=accent_blue,
        name="exit_rim_band",
    )
    # small blue turbopump unit hung under the mounting plate
    engine.visual(
        Box((0.06, 0.05, 0.16)),
        origin=Origin(xyz=(0.30, 0.0, -0.27)),
        material=struct_gray,
        name="turbopump_bracket",
    )
    engine.visual(
        Cylinder(radius=0.075, length=0.22),
        origin=Origin(xyz=(0.34, 0.0, -0.38), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent_blue,
        name="turbopump_body",
    )
    engine.visual(
        Box((0.09, 0.10, 0.09)),
        origin=Origin(xyz=(0.42, 0.0, -0.38)),
        material=detail_dark,
        name="turbopump_manifold",
    )
    # gray feed pipe loop from the collar down into the bell clamp ring
    engine.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.12, 0.06, -0.80),
                    (0.40, 0.14, -0.92),
                    (0.52, 0.10, -1.05),
                    (0.36, 0.04, -1.16),
                ],
                radius=0.028,
                samples_per_segment=12,
                radial_segments=14,
                cap_ends=True,
            ),
            "feed_pipe_tube",
        ),
        material=pipe_gray,
        name="feed_pipe",
    )

    model.articulation(
        "gimbal_yaw",
        ArticulationType.REVOLUTE,
        parent=cross,
        child=engine,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=200.0, velocity=0.5, lower=-0.12, upper=0.12),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bulkhead = object_model.get_part("thrust_bulkhead")
    cross = object_model.get_part("gimbal_cross")
    engine = object_model.get_part("engine_assembly")
    pitch = object_model.get_articulation("gimbal_pitch")
    yaw = object_model.get_articulation("gimbal_yaw")

    # Intentional bearing embeddings: the gimbal cross pins seat into the
    # bulkhead hub stub above and the engine mount hub below.
    for pin in ("gimbal_pin_x", "gimbal_pin_y"):
        ctx.allow_overlap(
            bulkhead,
            cross,
            elem_a="gimbal_hub_stub",
            elem_b=pin,
            reason="Gimbal cross pin seats into the bulkhead hub stub bearing.",
        )
        ctx.allow_overlap(
            engine,
            cross,
            elem_a="mount_hub",
            elem_b=pin,
            reason="Gimbal cross pin seats into the engine mount hub bearing.",
        )

    # --- joints -----------------------------------------------------------
    ctx.check(
        "gimbal is a two-axis revolute pair",
        pitch.articulation_type == ArticulationType.REVOLUTE
        and yaw.articulation_type == ArticulationType.REVOLUTE,
        details=f"{pitch.articulation_type}, {yaw.articulation_type}",
    )

    # --- structure --------------------------------------------------------
    disc_aabb = ctx.part_element_world_aabb(bulkhead, elem="bulkhead_disc")
    ctx.check(
        "thrust bulkhead disc caps the assembly",
        disc_aabb is not None
        and disc_aabb[1][2] > 2.0
        and (disc_aabb[1][0] - disc_aabb[0][0]) > 1.6,
        details=f"aabb={disc_aabb}",
    )
    bell_aabb = ctx.part_element_world_aabb(engine, elem="nozzle_bell")
    ctx.check(
        "nozzle bell hangs at the bottom",
        bell_aabb is not None and bell_aabb[0][2] < 0.10 and bell_aabb[0][2] > -0.05,
        details=f"aabb={bell_aabb}",
    )
    chamber_aabb = ctx.part_element_world_aabb(engine, elem="combustion_chamber")
    ctx.check(
        "chamber pod sits between plate and throat",
        chamber_aabb is not None
        and chamber_aabb[1][2] < PIVOT_Z + PLATE_Z1 + 0.02
        and chamber_aabb[0][2] > PIVOT_Z + COLLAR_Z - 0.06,
        details=f"aabb={chamber_aabb}",
    )
    for i in range(7):
        rib = ctx.part_element_world_aabb(engine, elem=f"bell_rib_{i}")
        ctx.check(f"bell rib {i} present", rib is not None, details=f"aabb={rib}")
    for i in range(CORRUG_STEPS):
        ring = ctx.part_element_world_aabb(engine, elem=f"throat_corrugation_{i}")
        ctx.check(f"throat corrugation {i} present", ring is not None)
    for i in range(len(ACTUATOR_ANGLES)):
        ctx.expect_contact(
            bulkhead,
            bulkhead,
            elem_a=f"actuator_piston_{i}",
            elem_b="bulkhead_disc",
            contact_tol=1e-3,
            name=f"actuator {i} anchored to the bulkhead",
        )

    # --- gimbal actually vectors the nozzle -------------------------------
    def bell_center() -> list[float]:
        aabb = ctx.part_element_world_aabb(engine, elem="nozzle_bell")
        return [(aabb[0][i] + aabb[1][i]) / 2.0 for i in range(3)]

    rest_c = bell_center()
    with ctx.pose({pitch: 0.12}):
        pitch_c = bell_center()
    with ctx.pose({yaw: 0.12}):
        yaw_c = bell_center()
    pitch_moved = math.dist(rest_c, pitch_c)
    yaw_moved = math.dist(rest_c, yaw_c)
    ctx.check(
        "pitch gimbal swings the nozzle",
        pitch_moved > 0.08,
        details=f"displacement={pitch_moved:.4f}",
    )
    ctx.check(
        "yaw gimbal swings the nozzle",
        yaw_moved > 0.08,
        details=f"displacement={yaw_moved:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
