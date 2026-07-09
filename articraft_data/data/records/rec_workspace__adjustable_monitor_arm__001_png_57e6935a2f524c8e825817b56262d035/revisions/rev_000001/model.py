from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    KnobGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_bar_mesh(
    *,
    name: str,
    length: float,
    width: float,
    height: float,
    radius: float,
    axis: str,
):
    """Rounded rectangular extrusion centered on the requested local axis."""
    if axis == "x":
        profile = rounded_rect_profile(height, width, radius, corner_segments=8)
        geom = ExtrudeGeometry(profile, length, center=True)
        geom.rotate_y(math.pi / 2.0)
    elif axis == "y":
        profile = rounded_rect_profile(width, height, radius, corner_segments=8)
        geom = ExtrudeGeometry(profile, length, center=True)
        geom.rotate_x(-math.pi / 2.0)
    elif axis == "z":
        profile = rounded_rect_profile(width, height, radius, corner_segments=8)
        geom = ExtrudeGeometry(profile, length, center=True)
    else:
        raise ValueError(f"unsupported axis {axis!r}")
    return mesh_from_geometry(geom, name)


def _add_rounded_bar(
    part,
    *,
    name: str,
    length: float,
    width: float,
    height: float,
    radius: float,
    axis: str,
    xyz: tuple[float, float, float],
    material: Material,
) -> None:
    part.visual(
        _rounded_bar_mesh(
            name=name,
            length=length,
            width=width,
            height=height,
            radius=radius,
            axis=axis,
        ),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _cyl_x(part, name, radius, length, xyz, material):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _cyl_y(part, name, radius, length, xyz, material):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="adjustable_monitor_arm",
        meta={"classification_note": "Reference and requested category both indicate an adjustable monitor arm."},
    )

    glossy_black = model.material("glossy_black", rgba=(0.002, 0.002, 0.002, 1.0))
    satin_black = model.material("satin_black", rgba=(0.015, 0.014, 0.013, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.12, 0.12, 0.11, 1.0))
    rubber = model.material("black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.60, 0.55, 1.0))
    screen_glass = model.material("cool_screen_glass", rgba=(0.50, 0.70, 0.72, 0.72))

    clamp = model.part("desk_clamp")
    _add_rounded_bar(
        clamp,
        name="top_plate",
        length=0.035,
        width=0.250,
        height=0.180,
        radius=0.022,
        axis="z",
        xyz=(0.0, 0.0, 0.0175),
        material=glossy_black,
    )
    clamp.visual(
        Box((0.052, 0.110, 0.250)),
        origin=Origin(xyz=(-0.095, 0.0, -0.090)),
        material=glossy_black,
        name="rear_spine",
    )
    clamp.visual(
        Box((0.150, 0.090, 0.026)),
        origin=Origin(xyz=(-0.025, 0.0, -0.205)),
        material=glossy_black,
        name="lower_jaw",
    )
    clamp.visual(
        Box((0.060, 0.092, 0.110)),
        origin=Origin(xyz=(-0.094, 0.0, -0.205)),
        material=glossy_black,
        name="jaw_back_rib",
    )
    _cyl_x(clamp, "jaw_bolt_0", 0.006, 0.060, (-0.094, -0.056, -0.155), steel)
    _cyl_x(clamp, "jaw_bolt_1", 0.006, 0.060, (-0.094, -0.056, -0.205), steel)
    clamp.visual(
        Cylinder(radius=0.010, length=0.155),
        origin=Origin(xyz=(0.018, 0.0, -0.160)),
        material=steel,
        name="clamp_screw",
    )
    clamp.visual(
        Cylinder(radius=0.040, length=0.010),
        origin=Origin(xyz=(0.018, 0.0, -0.078)),
        material=steel,
        name="pressure_pad",
    )
    clamp.visual(
        mesh_from_geometry(
            KnobGeometry(0.085, 0.026, body_style="lobed", top_diameter=0.078, base_diameter=0.052),
            "clamp_knob_mesh",
        ),
        origin=Origin(xyz=(0.018, 0.0, -0.246)),
        material=glossy_black,
        name="clamp_knob",
    )
    _cyl_x(clamp, "clamp_knob_crossbar", 0.012, 0.105, (0.018, 0.0, -0.246), glossy_black)
    clamp.visual(
        Cylinder(radius=0.018, length=0.007),
        origin=Origin(xyz=(0.072, 0.048, 0.0385)),
        material=dark_graphite,
        name="top_cap_screw",
    )

    base_swivel = model.part("base_swivel")
    base_swivel.visual(
        Cylinder(radius=0.038, length=0.130),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=glossy_black,
        name="vertical_post",
    )
    base_swivel.visual(
        Cylinder(radius=0.056, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.052)),
        material=glossy_black,
        name="lower_collar",
    )
    base_swivel.visual(
        Cylinder(radius=0.046, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=glossy_black,
        name="upper_collar",
    )
    base_swivel.visual(
        Box((0.078, 0.088, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=glossy_black,
        name="shoulder_block",
    )
    for idx, y in enumerate((-0.052, 0.052)):
        base_swivel.visual(
            Box((0.062, 0.018, 0.151)),
            origin=Origin(xyz=(0.000, y, 0.1105)),
            material=glossy_black,
            name=f"base_yoke_cheek_{idx}",
        )
    _cyl_y(base_swivel, "base_yoke_pin_cap_0", 0.016, 0.006, (0.0, -0.064, 0.135), steel)
    _cyl_y(base_swivel, "base_yoke_pin_cap_1", 0.016, 0.006, (0.0, 0.064, 0.135), steel)
    base_swivel.visual(
        Cylinder(radius=0.010, length=0.116),
        origin=Origin(xyz=(0.0, 0.0, 0.135), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="base_pivot_pin",
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        Cylinder(radius=0.043, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="base_hub",
    )
    _add_rounded_bar(
        lower_arm,
        name="lower_shell",
        length=0.350,
        width=0.083,
        height=0.054,
        radius=0.019,
        axis="x",
        xyz=(0.205, 0.0, 0.0),
        material=glossy_black,
    )
    _add_rounded_bar(
        lower_arm,
        name="lower_top_spine",
        length=0.280,
        width=0.044,
        height=0.014,
        radius=0.007,
        axis="x",
        xyz=(0.205, 0.0, 0.032),
        material=dark_graphite,
    )
    lower_arm.visual(
        Cylinder(radius=0.051, length=0.072),
        origin=Origin(xyz=(0.420, 0.0, 0.0)),
        material=satin_black,
        name="elbow_outer_ring",
    )
    for idx, z in enumerate((-0.045, 0.045)):
        lower_arm.visual(
            Box((0.080, 0.092, 0.018)),
            origin=Origin(xyz=(0.420, 0.0, z)),
            material=glossy_black,
            name=f"elbow_fork_cheek_{idx}",
        )
    _cyl_x(lower_arm, "lower_gas_spring", 0.011, 0.280, (0.210, 0.0, -0.060), dark_graphite)
    _cyl_x(lower_arm, "lower_spring_rod", 0.006, 0.110, (0.365, 0.0, -0.060), steel)
    for idx, x in enumerate((0.075, 0.350)):
        lower_arm.visual(
            Box((0.018, 0.030, 0.058)),
            origin=Origin(xyz=(x, 0.0, -0.033)),
            material=glossy_black,
            name=f"lower_spring_tab_{idx}",
        )
    lower_arm.visual(
        Box((0.090, 0.026, 0.018)),
        origin=Origin(xyz=(0.210, 0.0, -0.082)),
        material=rubber,
        name="lower_cable_clip",
    )
    lower_arm.visual(
        Box((0.028, 0.016, 0.030)),
        origin=Origin(xyz=(0.165, -0.018, -0.065)),
        material=rubber,
        name="lower_clip_lip_0",
    )
    lower_arm.visual(
        Box((0.028, 0.016, 0.030)),
        origin=Origin(xyz=(0.255, 0.018, -0.065)),
        material=rubber,
        name="lower_clip_lip_1",
    )
    lower_arm.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.420, 0.0, -0.057)),
        material=steel,
        name="elbow_silver_screw_0",
    )
    lower_arm.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.420, 0.0, 0.057)),
        material=steel,
        name="elbow_silver_screw_1",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.044, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=satin_black,
        name="elbow_hub",
    )
    _add_rounded_bar(
        upper_arm,
        name="upper_shell",
        length=0.380,
        width=0.088,
        height=0.060,
        radius=0.022,
        axis="x",
        xyz=(0.275, 0.0, 0.006),
        material=glossy_black,
    )
    upper_arm.visual(
        Box((0.050, 0.050, 0.040)),
        origin=Origin(xyz=(0.065, 0.0, 0.006)),
        material=glossy_black,
        name="upper_elbow_neck",
    )
    _add_rounded_bar(
        upper_arm,
        name="upper_raised_cover",
        length=0.310,
        width=0.050,
        height=0.020,
        radius=0.010,
        axis="x",
        xyz=(0.260, 0.0, 0.044),
        material=dark_graphite,
    )
    _cyl_x(upper_arm, "upper_gas_spring_body", 0.014, 0.300, (0.230, 0.0, -0.052), dark_graphite)
    _cyl_x(upper_arm, "upper_gas_spring_rod", 0.0065, 0.155, (0.405, 0.0, -0.052), steel)
    for idx, x in enumerate((0.070, 0.395)):
        upper_arm.visual(
            Box((0.020, 0.032, 0.062)),
            origin=Origin(xyz=(x, 0.0, -0.022)),
            material=glossy_black,
            name=f"upper_link_tab_{idx}",
        )
    upper_arm.visual(
        Box((0.115, 0.026, 0.018)),
        origin=Origin(xyz=(0.290, 0.0, -0.086)),
        material=rubber,
        name="upper_cable_clip",
    )
    upper_arm.visual(
        Box((0.026, 0.016, 0.030)),
        origin=Origin(xyz=(0.235, -0.018, -0.069)),
        material=rubber,
        name="upper_clip_lip_0",
    )
    upper_arm.visual(
        Box((0.026, 0.016, 0.030)),
        origin=Origin(xyz=(0.345, 0.018, -0.069)),
        material=rubber,
        name="upper_clip_lip_1",
    )
    upper_arm.visual(
        Cylinder(radius=0.045, length=0.070),
        origin=Origin(xyz=(0.480, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="wrist_outer_ring",
    )
    for idx, y in enumerate((-0.052, 0.052)):
        upper_arm.visual(
            Box((0.070, 0.018, 0.088)),
            origin=Origin(xyz=(0.480, y, 0.0)),
            material=glossy_black,
            name=f"wrist_yoke_cheek_{idx}",
        )
    _cyl_y(upper_arm, "wrist_pin_cap_0", 0.014, 0.006, (0.480, -0.064, 0.0), steel)
    _cyl_y(upper_arm, "wrist_pin_cap_1", 0.014, 0.006, (0.480, 0.064, 0.0), steel)

    tilt_head = model.part("tilt_head")
    tilt_head.visual(
        Cylinder(radius=0.039, length=0.068),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="tilt_hub",
    )
    _add_rounded_bar(
        tilt_head,
        name="head_bridge",
        length=0.076,
        width=0.075,
        height=0.058,
        radius=0.015,
        axis="x",
        xyz=(0.077, 0.0, 0.0),
        material=glossy_black,
    )
    tilt_head.visual(
        Cylinder(radius=0.034, length=0.026),
        origin=Origin(xyz=(0.112, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="vesa_bearing_collar",
    )
    tilt_head.visual(
        Box((0.018, 0.054, 0.080)),
        origin=Origin(xyz=(0.102, 0.0, 0.0)),
        material=dark_graphite,
        name="tilt_stop_block",
    )

    vesa_mount = model.part("vesa_mount")
    vesa_mount.visual(
        Cylinder(radius=0.041, length=0.022),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="rotation_boss",
    )
    vesa_mount.visual(
        Box((0.012, 0.158, 0.158)),
        origin=Origin(xyz=(0.024, 0.0, 0.0)),
        material=glossy_black,
        name="vesa_plate",
    )
    for row, z in enumerate((-0.050, 0.050)):
        for col, y in enumerate((-0.050, 0.050)):
            _cyl_x(
                vesa_mount,
                f"vesa_screw_{row}_{col}",
                0.008,
                0.010,
                (0.035, y, z),
                steel,
            )
            _cyl_x(
                vesa_mount,
                f"vesa_standoff_{row}_{col}",
                0.0055,
                0.044,
                (0.057, y, z),
                dark_graphite,
            )
    vesa_mount.visual(
        Box((0.035, 0.360, 0.520)),
        origin=Origin(xyz=(0.086, 0.0, 0.020)),
        material=dark_graphite,
        name="monitor_body",
    )
    vesa_mount.visual(
        Box((0.0025, 0.300, 0.440)),
        origin=Origin(xyz=(0.10475, 0.0, 0.020)),
        material=screen_glass,
        name="screen_inset",
    )
    vesa_mount.visual(
        Box((0.005, 0.350, 0.028)),
        origin=Origin(xyz=(0.106, 0.0, 0.282)),
        material=glossy_black,
        name="top_bezel",
    )
    vesa_mount.visual(
        Box((0.005, 0.350, 0.032)),
        origin=Origin(xyz=(0.106, 0.0, -0.242)),
        material=glossy_black,
        name="bottom_bezel",
    )
    vesa_mount.visual(
        Box((0.005, 0.030, 0.492)),
        origin=Origin(xyz=(0.106, -0.195, 0.020)),
        material=glossy_black,
        name="side_bezel_0",
    )
    vesa_mount.visual(
        Box((0.005, 0.030, 0.492)),
        origin=Origin(xyz=(0.106, 0.195, 0.020)),
        material=glossy_black,
        name="side_bezel_1",
    )
    for idx, y in enumerate((-0.120, 0.0, 0.120)):
        vesa_mount.visual(
            Box((0.003, 0.006, 0.500)),
            origin=Origin(xyz=(0.067, y, 0.020)),
            material=satin_black,
            name=f"rear_rib_{idx}",
        )

    model.articulation(
        "base_swivel",
        ArticulationType.REVOLUTE,
        parent=clamp,
        child=base_swivel,
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.5, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "swivel_to_lower_arm",
        ArticulationType.FIXED,
        parent=base_swivel,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.135), rpy=(0.0, -0.18, 0.0)),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(0.420, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "wrist_tilt",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=tilt_head,
        origin=Origin(xyz=(0.480, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.0, lower=-0.70, upper=0.70),
    )
    model.articulation(
        "vesa_rotation",
        ArticulationType.REVOLUTE,
        parent=tilt_head,
        child=vesa_mount,
        origin=Origin(xyz=(0.120, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-math.pi, upper=math.pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    requested_revolutes = {"base_swivel", "elbow", "wrist_tilt", "vesa_rotation"}
    authored_revolutes = {
        joint.name
        for joint in object_model.articulations
        if joint.articulation_type == ArticulationType.REVOLUTE
    }
    ctx.check(
        "requested revolute joints are present",
        requested_revolutes.issubset(authored_revolutes),
        details=f"authored_revolutes={sorted(authored_revolutes)}",
    )

    clamp = object_model.get_part("desk_clamp")
    base = object_model.get_part("base_swivel")
    vesa = object_model.get_part("vesa_mount")
    lower = object_model.get_part("lower_arm")
    upper = object_model.get_part("upper_arm")
    tilt = object_model.get_part("tilt_head")

    ctx.allow_overlap(
        base,
        lower,
        elem_a="base_pivot_pin",
        elem_b="base_hub",
        reason="The base pivot pin intentionally passes through the lower arm hub to carry the first arm segment.",
    )
    ctx.allow_overlap(
        lower,
        upper,
        reason="The upper arm hub is intentionally captured inside the lower arm elbow bearing housing.",
    )
    ctx.allow_overlap(
        upper,
        tilt,
        reason="The tilt head hub is intentionally captured inside the upper arm wrist bearing housing.",
    )
    ctx.allow_overlap(
        tilt,
        vesa,
        elem_a="vesa_bearing_collar",
        elem_b="rotation_boss",
        reason="The VESA rotation boss is intentionally seated through the tilt-head bearing collar.",
    )

    ctx.expect_gap(
        base,
        clamp,
        axis="z",
        max_penetration=0.010,
        name="swivel assembly stands above the desk clamp",
    )
    ctx.expect_origin_distance(
        lower,
        base,
        axes="y",
        max_dist=0.002,
        name="lower arm hub is centered between the base yoke cheeks",
    )
    ctx.expect_overlap(
        base,
        lower,
        axes="xyz",
        elem_a="base_pivot_pin",
        elem_b="base_hub",
        min_overlap=0.018,
        name="base pivot pin passes through the lower arm hub",
    )
    ctx.expect_overlap(
        lower,
        upper,
        axes="xyz",
        min_overlap=0.040,
        name="elbow bearing housing captures the upper arm hub",
    )
    ctx.expect_overlap(
        upper,
        tilt,
        axes="xyz",
        min_overlap=0.035,
        name="wrist bearing housing captures the tilt hub",
    )
    ctx.expect_overlap(
        tilt,
        vesa,
        axes="xyz",
        min_overlap=0.010,
        name="VESA rotation boss remains seated in the collar",
    )
    ctx.expect_origin_distance(
        lower,
        upper,
        axes="xy",
        min_dist=0.35,
        max_dist=0.48,
        name="elbow spacing matches the lower arm segment length",
    )
    ctx.expect_origin_distance(
        upper,
        tilt,
        axes="xy",
        min_dist=0.42,
        max_dist=0.55,
        name="wrist spacing matches the upper arm segment length",
    )

    elbow = object_model.get_articulation("elbow")
    wrist = object_model.get_articulation("wrist_tilt")
    rotation = object_model.get_articulation("vesa_rotation")
    swivel = object_model.get_articulation("base_swivel")

    rest_vesa = ctx.part_world_position(vesa)
    with ctx.pose({elbow: 0.65}):
        bent_vesa = ctx.part_world_position(vesa)
    ctx.check(
        "elbow swing changes the monitor head lateral position",
        rest_vesa is not None
        and bent_vesa is not None
        and abs(bent_vesa[1] - rest_vesa[1]) > 0.18,
        details=f"rest={rest_vesa}, bent={bent_vesa}",
    )

    with ctx.pose({swivel: 0.75}):
        swiveled_vesa = ctx.part_world_position(vesa)
    ctx.check(
        "base swivel yaws the whole arm",
        rest_vesa is not None
        and swiveled_vesa is not None
        and abs(swiveled_vesa[1] - rest_vesa[1]) > 0.45,
        details=f"rest={rest_vesa}, swiveled={swiveled_vesa}",
    )

    with ctx.pose({wrist: 0.45}):
        tilted_vesa = ctx.part_world_position(vesa)
    ctx.check(
        "wrist tilt pitches the VESA plate",
        rest_vesa is not None
        and tilted_vesa is not None
        and abs(tilted_vesa[2] - rest_vesa[2]) > 0.035,
        details=f"rest={rest_vesa}, tilted={tilted_vesa}",
    )

    rest_aabb = ctx.part_world_aabb(vesa)
    with ctx.pose({rotation: math.pi / 2.0}):
        rotated_aabb = ctx.part_world_aabb(vesa)

    def _yz_size(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return (hi[1] - lo[1], hi[2] - lo[2])

    rest_yz = _yz_size(rest_aabb)
    rotated_yz = _yz_size(rotated_aabb)
    ctx.check(
        "VESA rotation rolls the portrait monitor mount",
        rest_yz is not None
        and rotated_yz is not None
        and rotated_yz[0] > rest_yz[0] + 0.12
        and rotated_yz[1] < rest_yz[1] - 0.12,
        details=f"rest_yz={rest_yz}, rotated_yz={rotated_yz}",
    )

    return ctx.report()


object_model = build_object_model()
