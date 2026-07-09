from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _rounded_plate_mesh(name: str, length: float, width: float, thickness: float, radius: float):
    return _mesh(
        name,
        ExtrudeGeometry(
            rounded_rect_profile(length, width, radius, corner_segments=10),
            thickness,
            center=True,
        ),
    )


def _arm_segment_mesh(
    name: str,
    *,
    segment_length: float,
    segment_width: float,
    thickness: float,
    center_along: float,
    center_offset: float,
    link_length: float,
    rise: float,
    run: float,
):
    """One rounded segment of a perforated link, oriented in the arm plane."""
    geom = ExtrudeGeometry(
        rounded_rect_profile(
            segment_length,
            segment_width,
            min(segment_length, segment_width) * 0.45,
            corner_segments=8,
        ),
        thickness,
        center=True,
    )
    # Extrude thickness is along local Z. Rotate so thickness is side-to-side
    # local Y, and the 2D segment width becomes vertical local Z.
    geom.rotate_x(math.pi / 2.0)
    geom.translate(center_along - link_length * 0.5, 0.0, center_offset)
    theta = math.atan2(rise, run)
    geom.rotate_y(-theta)
    geom.translate(run * 0.5, 0.0, rise * 0.5)
    return _mesh(name, geom)


def _arm_plate_meshes(*, length: float, width: float, thickness: float, rise: float, run: float):
    """Segmented side link with two real open slots like the reference image."""
    rail_width = 0.007
    rail_offset = width * 0.5 - rail_width * 0.5
    end_bridge_len = 0.032
    center_bridge_len = 0.020
    usable_length = length - end_bridge_len
    center_bridge_at = length * 0.58

    segments = [
        (
            "upper_rail",
            usable_length,
            rail_width,
            length * 0.5,
            rail_offset,
        ),
        (
            "lower_rail",
            usable_length,
            rail_width,
            length * 0.5,
            -rail_offset,
        ),
        (
            "lower_end_bridge",
            end_bridge_len,
            width,
            end_bridge_len * 0.5,
            0.0,
        ),
        (
            "middle_web",
            center_bridge_len,
            width,
            center_bridge_at,
            0.0,
        ),
        (
            "upper_end_bridge",
            end_bridge_len,
            width,
            length - end_bridge_len * 0.5,
            0.0,
        ),
    ]
    return [
        (
            segment_name,
            _arm_segment_mesh(
                f"perforated_side_arm_{segment_name}",
                segment_length=segment_length,
                segment_width=segment_width,
                thickness=thickness,
                center_along=center_along,
                center_offset=center_offset,
                link_length=length,
                rise=rise,
                run=run,
            ),
        )
        for segment_name, segment_length, segment_width, center_along, center_offset in segments
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="adjustable_laptop_stand")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.017, 1.0))
    soft_black = model.material("soft_black", rgba=(0.055, 0.057, 0.060, 1.0))
    groove_black = model.material("groove_black", rgba=(0.0, 0.0, 0.0, 1.0))
    hardware = model.material("brushed_silver", rgba=(0.76, 0.76, 0.74, 1.0))
    laptop_silver = model.material("thin_laptop_silver", rgba=(0.80, 0.82, 0.83, 1.0))

    base = model.part("base")
    base.visual(
        _rounded_plate_mesh("rounded_base_plate", 0.34, 0.24, 0.016, 0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=matte_black,
        name="base_plate",
    )
    # Shallow top grooves/ribs, slightly seated into the base so they read as
    # manufactured channels rather than loose strips.
    for side, y in enumerate((-0.078, 0.078)):
        base.visual(
            Box((0.240, 0.006, 0.0022)),
            origin=Origin(xyz=(0.000, y, 0.0149)),
            material=groove_black,
            name=f"side_groove_{side}",
        )
    for side, x in enumerate((-0.126, 0.126)):
        base.visual(
            Box((0.006, 0.156, 0.0022)),
            origin=Origin(xyz=(x, 0.0, 0.0149)),
            material=groove_black,
            name=f"end_groove_{side}",
        )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.090, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=soft_black,
        name="outer_rotation_disk",
    )
    turntable.visual(
        Cylinder(radius=0.070, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=matte_black,
        name="upper_turntable_disk",
    )
    turntable.visual(
        Cylinder(radius=0.046, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=matte_black,
        name="short_pedestal",
    )
    # Two yoke lugs carried by the rotating pedestal hold the lower arm shaft.
    for side, y in enumerate((-0.052, 0.052)):
        turntable.visual(
            Box((0.040, 0.020, 0.090)),
            origin=Origin(xyz=(0.030, y, 0.045)),
            material=matte_black,
            name=f"base_lug_{side}",
        )
        turntable.visual(
            Cylinder(radius=0.020, length=0.004),
            origin=Origin(xyz=(0.030, y * 1.02, 0.075), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hardware,
            name=f"base_lug_bushing_{side}",
        )

    run = 0.160
    rise = 0.200
    link_length = math.sqrt(run * run + rise * rise)
    link_arms = model.part("link_arms")
    side_arm_meshes = _arm_plate_meshes(
        length=link_length,
        width=0.038,
        thickness=0.009,
        rise=rise,
        run=run,
    )
    for side, y in enumerate((-0.078, 0.078)):
        for segment_name, segment_mesh in side_arm_meshes:
            visual_name = f"side_arm_{side}" if segment_name == "upper_rail" else f"side_arm_{side}_{segment_name}"
            link_arms.visual(
                segment_mesh,
                origin=Origin(xyz=(0.0, y, 0.0)),
                material=matte_black,
                name=visual_name,
            )
    # Structural cross-shafts keep the paired arms as one supported assembly.
    for name, x, z in (("base_shaft", 0.0, 0.0), ("tray_shaft", run, rise)):
        link_arms.visual(
            Cylinder(radius=0.011, length=0.188),
            origin=Origin(xyz=(x, 0.0, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hardware,
            name=name,
        )
    for side, y in enumerate((-0.098, 0.098)):
        link_arms.visual(
            Cylinder(radius=0.023, length=0.009),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hardware,
            name=f"base_pivot_washer_{side}",
        )
        link_arms.visual(
            Cylinder(radius=0.023, length=0.009),
            origin=Origin(xyz=(run, y, rise), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hardware,
            name=f"tray_pivot_washer_{side}",
        )

    tray = model.part("upper_tray")
    # Pivot lugs are attached to the tray underside and capture the upper shaft.
    for side, y in enumerate((-0.052, 0.052)):
        tray.visual(
            Box((0.038, 0.020, 0.054)),
            origin=Origin(xyz=(0.0, y, 0.020)),
            material=matte_black,
            name=f"tray_lug_{side}",
        )
        tray.visual(
            Cylinder(radius=0.020, length=0.004),
            origin=Origin(xyz=(0.0, y * 1.02, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hardware,
            name=f"tray_lug_bushing_{side}",
        )
    tray.visual(
        Box((0.082, 0.230, 0.014)),
        origin=Origin(xyz=(0.022, 0.0, 0.032)),
        material=matte_black,
        name="front_tray_rail",
    )
    for side, y in enumerate((-0.092, 0.092)):
        tray.visual(
            Box((0.300, 0.018, 0.014)),
            origin=Origin(xyz=(-0.110, y, 0.032)),
            material=matte_black,
            name=f"side_tray_rail_{side}",
        )
    # Thin laptop-shaped plate only provides load context; no screen or graphics.
    tray.visual(
        _rounded_plate_mesh("thin_laptop_plate", 0.300, 0.215, 0.006, 0.014),
        origin=Origin(xyz=(-0.110, 0.0, 0.042)),
        material=laptop_silver,
        name="laptop_plate",
    )
    for side, y in enumerate((-0.066, 0.066)):
        tray.visual(
            Box((0.022, 0.058, 0.066)),
            origin=Origin(xyz=(0.056, y, 0.072)),
            material=soft_black,
            name=f"front_lip_{side}",
        )
        tray.visual(
            Box((0.034, 0.064, 0.014)),
            origin=Origin(xyz=(0.044, y, 0.044)),
            material=soft_black,
            name=f"lip_foot_{side}",
        )

    model.articulation(
        "turntable_yaw",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0),
    )
    model.articulation(
        "arm_pitch",
        ArticulationType.REVOLUTE,
        parent=turntable,
        child=link_arms,
        origin=Origin(xyz=(0.030, 0.0, 0.075)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=-0.45, upper=0.45),
    )
    model.articulation(
        "tray_tilt",
        ArticulationType.REVOLUTE,
        parent=link_arms,
        child=tray,
        origin=Origin(xyz=(run, 0.0, rise)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.2, lower=-0.35, upper=0.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    turntable = object_model.get_part("turntable")
    arms = object_model.get_part("link_arms")
    tray = object_model.get_part("upper_tray")

    # Pivot shafts are intentionally captured inside visible bracket lugs.
    for elem in ("base_lug_0", "base_lug_1"):
        ctx.allow_overlap(
            turntable,
            arms,
            elem_a=elem,
            elem_b="base_shaft",
            reason="The lower silver pivot shaft is intentionally captured through the turntable yoke lug.",
        )
        ctx.expect_overlap(
            turntable,
            arms,
            axes="yz",
            elem_a=elem,
            elem_b="base_shaft",
            min_overlap=0.010,
            name=f"{elem} captures lower shaft",
        )
    for elem in ("tray_lug_0", "tray_lug_1"):
        ctx.allow_overlap(
            tray,
            arms,
            elem_a=elem,
            elem_b="tray_shaft",
            reason="The upper silver pivot shaft is intentionally captured through the tray hinge lug.",
        )
        ctx.expect_overlap(
            tray,
            arms,
            axes="yz",
            elem_a=elem,
            elem_b="tray_shaft",
            min_overlap=0.010,
            name=f"{elem} captures upper shaft",
        )

    ctx.check(
        "broad rounded base plate present",
        base.get_visual("base_plate") is not None and len([v for v in base.visuals if "groove" in (v.name or "")]) >= 4,
        details="Expected rounded base plate and shallow groove visuals.",
    )
    ctx.check(
        "turntable disk and pedestal present",
        turntable.get_visual("upper_turntable_disk") is not None and turntable.get_visual("short_pedestal") is not None,
        details="Expected central rotating disk/pedestal.",
    )
    ctx.check(
        "two perforated side arms present",
        arms.get_visual("side_arm_0") is not None and arms.get_visual("side_arm_1") is not None,
        details="Expected paired side arms with elongated mesh cutouts.",
    )
    ctx.check(
        "visible silver pivot hardware present",
        all(arms.get_visual(name) is not None for name in ("base_pivot_washer_0", "base_pivot_washer_1", "tray_pivot_washer_0", "tray_pivot_washer_1")),
        details="Expected silver washers at lower and upper pivots.",
    )
    ctx.check(
        "front retaining lips present",
        tray.get_visual("front_lip_0") is not None and tray.get_visual("front_lip_1") is not None,
        details="Expected two front clamps/lips.",
    )
    nonfixed = [
        joint for joint in object_model.articulations
        if str(joint.articulation_type).split(".")[-1] != "FIXED"
    ]
    ctx.check(
        "rotation and tilt joints present",
        len(nonfixed) >= 2
        and object_model.get_articulation("turntable_yaw") is not None
        and object_model.get_articulation("arm_pitch") is not None
        and object_model.get_articulation("tray_tilt") is not None,
        details="Expected yaw, arm pitch, and tray tilt articulations.",
    )
    ctx.expect_gap(turntable, base, axis="z", max_gap=0.001, max_penetration=0.001, name="turntable sits on base")
    ctx.expect_overlap(tray, arms, axes="xy", min_overlap=0.018, elem_a="front_tray_rail", elem_b="tray_shaft", name="upper tray is carried by arm shaft")

    arm_joint = object_model.get_articulation("arm_pitch")
    tray_joint = object_model.get_articulation("tray_tilt")
    rest_aabb = ctx.part_world_aabb(tray)
    with ctx.pose({arm_joint: 0.30, tray_joint: -0.20}):
        posed_aabb = ctx.part_world_aabb(tray)
    ctx.check(
        "adjustment joints move upper tray",
        rest_aabb is not None and posed_aabb is not None and abs(posed_aabb[0][2] - rest_aabb[0][2]) > 0.010,
        details=f"rest={rest_aabb}, posed={posed_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
