from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BACKREST_ANGLE = math.radians(40.0)


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """Soft rectangular cushion/tray form in meters."""
    return (
        cq.Workplane("XY")
        .box(size[0], size[1], size[2])
        .edges()
        .fillet(radius)
    )


def _mitered_backrest_pad(length: float, width: float, thickness: float, top_cutback: float) -> MeshGeometry:
    """A cushion prism with its hinge end sloped so the raised pad does not interpenetrate the seat pad."""
    y0 = -width * 0.5
    y1 = width * 0.5
    verts = [
        (-length, y0, 0.0),
        (0.0, y0, 0.0),
        (0.0, y1, 0.0),
        (-length, y1, 0.0),
        (-length, y0, thickness),
        (-top_cutback, y0, thickness),
        (-top_cutback, y1, thickness),
        (-length, y1, thickness),
    ]
    faces = [
        (0, 1, 2), (0, 2, 3),  # bottom
        (4, 7, 6), (4, 6, 5),  # top
        (0, 4, 5), (0, 5, 1),  # near side
        (3, 2, 6), (3, 6, 7),  # far side
        (0, 3, 7), (0, 7, 4),  # head end
        (1, 5, 6), (1, 6, 2),  # mitered hinge end
    ]
    return MeshGeometry(vertices=verts, faces=faces)


def _rod_origin(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[Origin, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("rod endpoints must be separated")
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    origin = Origin(
        xyz=((start[0] + end[0]) * 0.5, (start[1] + end[1]) * 0.5, (start[2] + end[2]) * 0.5),
        rpy=(0.0, pitch, yaw),
    )
    return origin, length


def _add_rod(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    material: Material,
    name: str,
) -> None:
    origin, length = _rod_origin(start, end)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def _add_caster(part, steel: Material, tire: Material) -> None:
    # Local caster frame sits at the bottom of a leg; the wheel just touches the floor.
    part.visual(
        Cylinder(radius=0.012, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=steel,
        name="swivel_stem",
    )
    part.visual(
        Cylinder(radius=0.030, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.052)),
        material=steel,
        name="swivel_plate",
    )
    part.visual(
        Box((0.020, 0.008, 0.075)),
        origin=Origin(xyz=(0.0, -0.026, -0.055)),
        material=steel,
        name="fork_side_0",
    )
    part.visual(
        Box((0.020, 0.008, 0.075)),
        origin=Origin(xyz=(0.0, 0.026, -0.055)),
        material=steel,
        name="fork_side_1",
    )
    part.visual(
        Cylinder(radius=0.008, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, -0.085), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="axle",
    )
    part.visual(
        Cylinder(radius=0.045, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, -0.085), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=tire,
        name="wheel",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="adjustable_treatment_couch")

    upholstery = model.material("cream_vinyl", rgba=(0.92, 0.88, 0.76, 1.0))
    steel = model.material("white_painted_steel", rgba=(0.96, 0.96, 0.93, 1.0))
    dark = model.material("dark_rubber", rgba=(0.04, 0.045, 0.05, 1.0))
    metal = model.material("brushed_metal", rgba=(0.70, 0.73, 0.72, 1.0))

    frame = model.part("frame")
    # Slim rectangular white steel underframe: deck side rails, cross rails, legs, and lower braces.
    frame.visual(Box((1.90, 0.035, 0.035)), origin=Origin(xyz=(0.0, -0.3425, 0.520)), material=steel, name="side_rail_0")
    frame.visual(Box((1.90, 0.035, 0.035)), origin=Origin(xyz=(0.0, 0.3425, 0.520)), material=steel, name="side_rail_1")
    for i, x in enumerate((-0.95, -0.35, 0.25, 0.95)):
        z = 0.500 if i == 1 else 0.520
        frame.visual(Box((0.035, 0.720, 0.035)), origin=Origin(xyz=(x, 0.0, z)), material=steel, name=f"cross_rail_{i}")
    frame.visual(Box((0.580, 0.620, 0.024)), origin=Origin(xyz=(-0.050, 0.0, 0.547)), material=steel, name="seat_support_panel")
    frame.visual(Box((0.680, 0.620, 0.024)), origin=Origin(xyz=(0.600, 0.0, 0.547)), material=steel, name="leg_support_panel")

    for i, (x, y) in enumerate(((-0.83, -0.31), (-0.83, 0.31), (0.83, -0.31), (0.83, 0.31))):
        frame.visual(Box((0.040, 0.040, 0.390)), origin=Origin(xyz=(x, y, 0.325)), material=steel, name=f"leg_{i}")
    frame.visual(Box((1.66, 0.026, 0.026)), origin=Origin(xyz=(0.0, -0.31, 0.235)), material=steel, name="lower_side_rail_0")
    frame.visual(Box((1.66, 0.026, 0.026)), origin=Origin(xyz=(0.0, 0.31, 0.235)), material=steel, name="lower_side_rail_1")
    frame.visual(Box((0.026, 0.620, 0.026)), origin=Origin(xyz=(-0.83, 0.0, 0.235)), material=steel, name="lower_end_rail_0")
    frame.visual(Box((0.026, 0.620, 0.026)), origin=Origin(xyz=(0.83, 0.0, 0.235)), material=steel, name="lower_end_rail_1")
    _add_rod(frame, (-0.83, -0.31, 0.235), (-0.35, -0.31, 0.520), 0.012, steel, "diagonal_brace_0")
    _add_rod(frame, (-0.83, 0.31, 0.235), (-0.35, 0.31, 0.520), 0.012, steel, "diagonal_brace_1")
    frame.visual(
        Box((0.050, 0.085, 0.035)),
        origin=Origin(xyz=(-0.780, -0.2775, 0.255)),
        material=metal,
        name="support_arm_socket",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.640),
        origin=Origin(xyz=(-0.35, 0.0, 0.540), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="backrest_hinge_pin",
    )

    seat_pad = model.part("seat_pad")
    seat_pad.visual(
        mesh_from_cadquery(_rounded_box((0.600, 0.680, 0.080), 0.022), "seat_pad"),
        origin=Origin(xyz=(-0.050, 0.0, 0.599)),
        material=upholstery,
        name="seat_cushion",
    )
    model.articulation("frame_to_seat_pad", ArticulationType.FIXED, parent=frame, child=seat_pad)

    leg_pad = model.part("leg_pad")
    leg_pad.visual(
        mesh_from_cadquery(_rounded_box((0.700, 0.680, 0.080), 0.022), "leg_pad"),
        origin=Origin(xyz=(0.600, 0.0, 0.599)),
        material=upholstery,
        name="leg_cushion",
    )
    model.articulation("frame_to_leg_pad", ArticulationType.FIXED, parent=frame, child=leg_pad)

    backrest = model.part("backrest")
    backrest.visual(
        mesh_from_geometry(_mitered_backrest_pad(0.750, 0.680, 0.080, 0.072), "backrest_pad"),
        origin=Origin(),
        material=upholstery,
        name="backrest_cushion",
    )
    # White steel subframe just under the raised cushion, connected to the pad underside.
    _add_rod(backrest, (-0.710, -0.295, -0.008), (-0.050, -0.295, -0.008), 0.010, steel, "underside_rail_0")
    _add_rod(backrest, (-0.710, 0.295, -0.008), (-0.050, 0.295, -0.008), 0.010, steel, "underside_rail_1")
    _add_rod(backrest, (-0.050, -0.295, -0.008), (-0.050, 0.295, -0.008), 0.010, steel, "hinge_cross_tube")
    backrest.visual(Box((0.075, 0.032, 0.026)), origin=Origin(xyz=(-0.025, -0.295, -0.012)), material=metal, name="hinge_plate_0")
    backrest.visual(Box((0.075, 0.032, 0.026)), origin=Origin(xyz=(-0.025, 0.295, -0.012)), material=metal, name="hinge_plate_1")
    backrest.visual(Box((0.060, 0.030, 0.018)), origin=Origin(xyz=(-0.025, 0.0, -0.012)), material=metal, name="hinge_bridge")
    backrest.visual(
        Cylinder(radius=0.014, length=0.540),
        origin=Origin(xyz=(0.0, 0.0, -0.015), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="hinge_knuckle",
    )
    model.articulation(
        "frame_to_backrest",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=backrest,
        origin=Origin(xyz=(-0.350, 0.0, 0.560), rpy=(0.0, BACKREST_ANGLE, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.8, lower=-BACKREST_ANGLE, upper=math.radians(70.0)),
    )

    support_arm = model.part("support_arm")
    # A visible thin metal prop arm reaching from the lower frame toward the raised backrest.
    _add_rod(support_arm, (0.0, 0.0, 0.0), (0.355, 0.0, 0.235), 0.008, metal, "sliding_arm")
    support_arm.visual(Cylinder(radius=0.020, length=0.030), origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)), material=metal, name="lower_pivot")
    support_arm.visual(Cylinder(radius=0.018, length=0.030), origin=Origin(xyz=(0.355, 0.0, 0.235), rpy=(math.pi / 2.0, 0.0, 0.0)), material=metal, name="upper_roller")
    model.articulation(
        "frame_to_support_arm",
        ArticulationType.FIXED,
        parent=frame,
        child=support_arm,
        origin=Origin(xyz=(-0.780, -0.245, 0.255)),
    )

    side_tray = model.part("side_tray")
    side_tray.visual(
        mesh_from_cadquery(_rounded_box((0.420, 0.220, 0.022), 0.010), "side_tray_plate"),
        origin=Origin(),
        material=steel,
        name="tray_plate",
    )
    side_tray.visual(Box((0.420, 0.018, 0.030)), origin=Origin(xyz=(0.0, 0.109, 0.026)), material=steel, name="outer_lip")
    side_tray.visual(Box((0.018, 0.220, 0.030)), origin=Origin(xyz=(-0.209, 0.0, 0.026)), material=steel, name="end_lip_0")
    side_tray.visual(Box((0.018, 0.220, 0.030)), origin=Origin(xyz=(0.209, 0.0, 0.026)), material=steel, name="end_lip_1")
    side_tray.visual(Box((0.030, 0.050, 0.025)), origin=Origin(xyz=(-0.140, -0.109, -0.035)), material=steel, name="mount_arm_0")
    side_tray.visual(Box((0.030, 0.050, 0.025)), origin=Origin(xyz=(0.140, -0.109, -0.035)), material=steel, name="mount_arm_1")
    side_tray.visual(Box((0.030, 0.030, 0.040)), origin=Origin(xyz=(-0.140, -0.094, -0.018)), material=steel, name="downstand_0")
    side_tray.visual(Box((0.030, 0.030, 0.040)), origin=Origin(xyz=(0.140, -0.094, -0.018)), material=steel, name="downstand_1")
    model.articulation(
        "frame_to_side_tray",
        ArticulationType.FIXED,
        parent=frame,
        child=side_tray,
        origin=Origin(xyz=(0.575, 0.494, 0.560)),
    )

    caster_positions = (
        (-0.83, -0.31, 0.130),
        (-0.83, 0.31, 0.130),
        (0.83, -0.31, 0.130),
        (0.83, 0.31, 0.130),
    )
    for i, xyz in enumerate(caster_positions):
        caster = model.part(f"caster_{i}")
        _add_caster(caster, steel, dark)
        model.articulation(f"frame_to_caster_{i}", ArticulationType.FIXED, parent=frame, child=caster, origin=Origin(xyz=xyz))

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    backrest = object_model.get_part("backrest")
    seat_pad = object_model.get_part("seat_pad")
    leg_pad = object_model.get_part("leg_pad")
    side_tray = object_model.get_part("side_tray")
    support_arm = object_model.get_part("support_arm")
    hinge = object_model.get_articulation("frame_to_backrest")

    ctx.allow_overlap(
        frame,
        backrest,
        elem_a="backrest_hinge_pin",
        elem_b="hinge_knuckle",
        reason="The backrest hinge knuckle is intentionally captured around the transverse hinge pin.",
    )
    ctx.expect_overlap(
        frame,
        backrest,
        axes="y",
        min_overlap=0.45,
        elem_a="backrest_hinge_pin",
        elem_b="hinge_knuckle",
        name="backrest hinge knuckle spans the hinge pin",
    )
    ctx.allow_overlap(
        frame,
        support_arm,
        elem_a="support_arm_socket",
        elem_b="lower_pivot",
        reason="The thin support arm is represented as emerging from and being captured inside a small frame socket.",
    )
    ctx.allow_overlap(
        frame,
        support_arm,
        elem_a="support_arm_socket",
        elem_b="sliding_arm",
        reason="The prop arm tube intentionally emerges from the small frame socket as a captured support linkage.",
    )
    ctx.allow_overlap(
        backrest,
        frame,
        elem_a="hinge_plate_0",
        elem_b="backrest_hinge_pin",
        reason="The backrest hinge plate locally wraps the transverse hinge pin.",
    )
    ctx.allow_overlap(
        backrest,
        frame,
        elem_a="hinge_plate_1",
        elem_b="backrest_hinge_pin",
        reason="The opposite backrest hinge plate locally wraps the transverse hinge pin.",
    )
    ctx.allow_overlap(
        backrest,
        frame,
        elem_a="hinge_bridge",
        elem_b="backrest_hinge_pin",
        reason="The central hinge bridge is locally captured around the hinge pin.",
    )
    ctx.expect_overlap(
        backrest,
        frame,
        axes="xyz",
        min_overlap=0.006,
        elem_a="hinge_plate_0",
        elem_b="backrest_hinge_pin",
        name="backrest hinge plate wraps the hinge pin",
    )
    ctx.expect_overlap(
        backrest,
        frame,
        axes="xyz",
        min_overlap=0.006,
        elem_a="hinge_plate_1",
        elem_b="backrest_hinge_pin",
        name="opposite backrest hinge plate wraps the hinge pin",
    )
    ctx.expect_overlap(
        backrest,
        frame,
        axes="xyz",
        min_overlap=0.006,
        elem_a="hinge_bridge",
        elem_b="backrest_hinge_pin",
        name="central hinge bridge is captured on the hinge pin",
    )
    ctx.expect_overlap(
        frame,
        support_arm,
        axes="xyz",
        min_overlap=0.010,
        elem_a="support_arm_socket",
        elem_b="lower_pivot",
        name="support arm is captured in the frame socket",
    )

    ctx.expect_gap(seat_pad, frame, axis="z", max_gap=0.008, max_penetration=0.001, name="seat pad rests on the steel deck support")
    ctx.expect_gap(leg_pad, frame, axis="z", max_gap=0.008, max_penetration=0.001, name="leg pad rests on the steel deck support")
    ctx.expect_overlap(seat_pad, frame, axes="xy", min_overlap=0.50, name="seat pad is supported by the frame footprint")
    ctx.expect_overlap(leg_pad, frame, axes="xy", min_overlap=0.60, name="leg pad is supported by the frame footprint")

    raised_aabb = ctx.part_world_aabb(backrest)
    ctx.check(
        "default backrest is raised about forty degrees",
        raised_aabb is not None and raised_aabb[1][2] > 0.94 and raised_aabb[0][0] < -0.90,
        details=f"raised_aabb={raised_aabb}",
    )

    with ctx.pose({hinge: -BACKREST_ANGLE}):
        flat_aabb = ctx.part_world_aabb(backrest)
        ctx.check(
            "lower limit makes the backrest nearly flat with the deck",
            flat_aabb is not None and flat_aabb[1][2] < 0.67,
            details=f"flat_aabb={flat_aabb}",
        )
        ctx.expect_gap(
            seat_pad,
            backrest,
            axis="x",
            max_gap=0.030,
            max_penetration=0.015,
            name="flat backrest meets the seat at the hinge seam",
        )

    tray_aabb = ctx.part_world_aabb(side_tray)
    ctx.check(
        "side tray is a small foot-end side accessory",
        tray_aabb is not None and tray_aabb[0][0] > 0.30 and tray_aabb[1][1] > 0.60,
        details=f"tray_aabb={tray_aabb}",
    )
    ctx.check(
        "four named caster assemblies are present",
        all(object_model.get_part(f"caster_{i}") is not None for i in range(4)),
    )

    return ctx.report()


object_model = build_object_model()
