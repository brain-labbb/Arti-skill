from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


OPEN_LID_ANGLE = math.radians(103.0)


def _rounded_plate(width: float, depth: float, thickness: float, radius: float, name: str):
    return mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(width, depth, radius, corner_segments=10),
            thickness,
            center=True,
        ),
        name,
    )


def _rounded_ring(
    outer_x: float,
    outer_y: float,
    inner_x: float,
    inner_y: float,
    thickness: float,
    outer_r: float,
    inner_r: float,
    name: str,
):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(outer_x, outer_y, outer_r, corner_segments=10),
            [list(reversed(rounded_rect_profile(inner_x, inner_y, inner_r, corner_segments=10)))],
            thickness,
            center=True,
        ),
        name,
    )


def _lid_xyz(local_xyz: tuple[float, float, float]) -> tuple[float, float, float]:
    """Position a closed-lid local point in the displayed open-lid pose."""
    x, y, z = local_xyz
    c = math.cos(OPEN_LID_ANGLE)
    s = math.sin(OPEN_LID_ANGLE)
    return (x * c + z * s, y, -x * s + z * c)


def _lid_origin(
    local_xyz: tuple[float, float, float],
    extra_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> Origin:
    return Origin(
        xyz=_lid_xyz(local_xyz),
        rpy=(extra_rpy[0], OPEN_LID_ANGLE + extra_rpy[1], extra_rpy[2]),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="open_portable_defibrillator_carry_case",
        meta={
            "run_notes": (
                "Modeled from the provided reference as an open black-and-red soft AED/"
                "defibrillator carry case with compact device inside; no category "
                "classification conflict was apparent."
            )
        },
    )

    case_red = model.material("case_red_fabric", rgba=(0.82, 0.035, 0.045, 1.0))
    black_fabric = model.material("black_fabric", rgba=(0.015, 0.014, 0.016, 1.0))
    dark_foam = model.material("dark_compartment_foam", rgba=(0.035, 0.038, 0.042, 1.0))
    gray_plastic = model.material("gray_plastic", rgba=(0.50, 0.53, 0.53, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.075, 0.085, 0.090, 1.0))
    screen_black = model.material("screen_black", rgba=(0.005, 0.007, 0.009, 1.0))
    screen_white = model.material("screen_white", rgba=(0.92, 0.95, 0.94, 1.0))
    white_pack = model.material("white_accessory_pack", rgba=(0.92, 0.93, 0.91, 1.0))
    print_red = model.material("simple_red_print", rgba=(0.90, 0.04, 0.035, 1.0))
    orange_button = model.material("orange_button", rgba=(1.0, 0.45, 0.10, 1.0))
    green_light = model.material("green_indicator", rgba=(0.08, 0.85, 0.34, 1.0))
    blue_connector = model.material("blue_connector", rgba=(0.02, 0.40, 0.75, 1.0))
    cable_white = model.material("white_cable", rgba=(0.94, 0.96, 0.96, 1.0))
    pink_key = model.material("pink_safety_key", rgba=(0.96, 0.42, 0.63, 1.0))
    metal = model.material("dull_metal", rgba=(0.55, 0.57, 0.56, 1.0))

    base = model.part("case_base")
    base.visual(
        Box((0.360, 0.030, 0.074)),
        origin=Origin(xyz=(0.0, -0.125, 0.045)),
        material=case_red,
        name="red_side_wall_0",
    )
    base.visual(
        Box((0.360, 0.030, 0.074)),
        origin=Origin(xyz=(0.0, 0.125, 0.045)),
        material=case_red,
        name="red_side_wall_1",
    )
    base.visual(
        Box((0.030, 0.280, 0.074)),
        origin=Origin(xyz=(-0.165, 0.0, 0.045)),
        material=case_red,
        name="red_front_wall",
    )
    base.visual(
        Box((0.030, 0.280, 0.074)),
        origin=Origin(xyz=(0.165, 0.0, 0.045)),
        material=case_red,
        name="red_rear_wall",
    )
    base.visual(
        _rounded_plate(0.350, 0.270, 0.010, 0.036, "red_bottom_skin"),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=case_red,
        name="red_bottom_skin",
    )
    base.visual(
        _rounded_plate(0.292, 0.212, 0.014, 0.020, "inner_floor"),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=dark_foam,
        name="inner_floor",
    )
    base.visual(
        Box((0.300, 0.010, 0.055)),
        origin=Origin(xyz=(0.0, -0.111, 0.052)),
        material=black_fabric,
        name="black_inner_liner_0",
    )
    base.visual(
        Box((0.300, 0.010, 0.055)),
        origin=Origin(xyz=(0.0, 0.111, 0.052)),
        material=black_fabric,
        name="black_inner_liner_1",
    )
    base.visual(
        Box((0.010, 0.222, 0.055)),
        origin=Origin(xyz=(-0.151, 0.0, 0.052)),
        material=black_fabric,
        name="black_inner_liner_2",
    )
    base.visual(
        Box((0.010, 0.222, 0.055)),
        origin=Origin(xyz=(0.151, 0.0, 0.052)),
        material=black_fabric,
        name="black_inner_liner_3",
    )
    base.visual(
        Box((0.374, 0.024, 0.022)),
        origin=Origin(xyz=(0.0, -0.135, 0.093)),
        material=black_fabric,
        name="padded_top_rim_0",
    )
    base.visual(
        Box((0.374, 0.024, 0.022)),
        origin=Origin(xyz=(0.0, 0.135, 0.093)),
        material=black_fabric,
        name="padded_top_rim_1",
    )
    base.visual(
        Box((0.024, 0.294, 0.022)),
        origin=Origin(xyz=(-0.175, 0.0, 0.093)),
        material=black_fabric,
        name="padded_top_rim_2",
    )
    base.visual(
        Box((0.024, 0.294, 0.022)),
        origin=Origin(xyz=(0.175, 0.0, 0.093)),
        material=black_fabric,
        name="padded_top_rim_3",
    )
    base.visual(
        Box((0.190, 0.012, 0.038)),
        origin=Origin(xyz=(-0.030, -0.052, 0.041)),
        material=black_fabric,
        name="foam_divider",
    )
    base.visual(
        Box((0.145, 0.070, 0.008)),
        origin=Origin(xyz=(-0.035, -0.097, 0.026)),
        material=dark_foam,
        name="accessory_pocket_floor",
    )
    base.visual(
        Box((0.115, 0.010, 0.024)),
        origin=Origin(xyz=(-0.055, -0.060, 0.040)),
        material=black_fabric,
        name="pocket_front_lip",
    )
    base.visual(
        Box((0.090, 0.007, 0.080)),
        origin=Origin(xyz=(0.030, -0.111, 0.060), rpy=(0.0, -0.35, 0.0)),
        material=white_pack,
        name="electrode_pack",
    )
    base.visual(
        Box((0.074, 0.008, 0.055)),
        origin=Origin(xyz=(0.026, -0.116, 0.062), rpy=(0.0, -0.35, 0.0)),
        material=screen_white,
        name="electrode_pack_label",
    )
    base.visual(
        Box((0.055, 0.006, 0.012)),
        origin=Origin(xyz=(0.018, -0.121, 0.081), rpy=(0.0, -0.35, 0.0)),
        material=print_red,
        name="simple_pack_mark",
    )
    base.visual(
        Box((0.010, 0.100, 0.030)),
        origin=Origin(xyz=(-0.181, 0.0, 0.060)),
        material=black_fabric,
        name="front_latch_patch",
    )
    base.visual(
        Cylinder(radius=0.006, length=0.032),
        origin=Origin(xyz=(-0.184, -0.128, 0.066), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="side_d_ring_0",
    )
    base.visual(
        Cylinder(radius=0.006, length=0.032),
        origin=Origin(xyz=(-0.184, 0.128, 0.066), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="side_d_ring_1",
    )

    lid = model.part("lid")
    lid.visual(
        _rounded_plate(0.350, 0.280, 0.016, 0.038, "lid_red_outer"),
        origin=_lid_origin((-0.175, 0.0, 0.030)),
        material=case_red,
        name="lid_red_outer",
    )
    lid.visual(
        _rounded_plate(0.328, 0.256, 0.012, 0.030, "lid_inner_panel"),
        origin=_lid_origin((-0.175, 0.0, 0.020)),
        material=black_fabric,
        name="lid_inner_panel",
    )
    lid.visual(
        _rounded_ring(0.338, 0.266, 0.292, 0.214, 0.014, 0.034, 0.020, "lid_padded_rim"),
        origin=_lid_origin((-0.175, 0.0, 0.012)),
        material=black_fabric,
        name="lid_padded_rim",
    )
    lid.visual(
        Box((0.050, 0.205, 0.007)),
        origin=_lid_origin((-0.026, 0.0, 0.015)),
        material=black_fabric,
        name="fabric_hinge_leaf",
    )
    lid.visual(
        Box((0.128, 0.052, 0.004)),
        origin=_lid_origin((-0.214, -0.052, 0.005)),
        material=screen_white,
        name="instruction_strip",
    )
    for index, lx in enumerate((-0.252, -0.214, -0.176)):
        lid.visual(
            Box((0.030, 0.042, 0.005)),
            origin=_lid_origin((lx, -0.052, 0.0025)),
            material=print_red,
            name=f"red_icon_panel_{index}",
        )
    lid.visual(
        Box((0.076, 0.056, 0.005)),
        origin=_lid_origin((-0.205, 0.080, 0.005)),
        material=dark_plastic,
        name="small_inner_pouch",
    )
    lid.visual(
        Box((0.041, 0.026, 0.004)),
        origin=_lid_origin((-0.208, 0.073, 0.001)),
        material=screen_black,
        name="pouch_label_patch",
    )
    lid.visual(
        Cylinder(radius=0.007, length=0.030),
        origin=_lid_origin((-0.194, 0.118, 0.003), extra_rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pink_key,
        name="pink_pull_key",
    )
    lid.visual(
        Sphere(radius=0.010),
        origin=_lid_origin((-0.194, 0.101, 0.003)),
        material=pink_key,
        name="pink_key_tab",
    )

    aed = model.part("aed_device")
    aed.visual(
        _rounded_plate(0.180, 0.135, 0.044, 0.018, "aed_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=gray_plastic,
        name="aed_body",
    )
    aed.visual(
        _rounded_ring(0.174, 0.129, 0.150, 0.105, 0.010, 0.016, 0.012, "aed_black_top_bezel"),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=dark_plastic,
        name="aed_black_top_bezel",
    )
    aed.visual(
        Box((0.088, 0.052, 0.006)),
        origin=Origin(xyz=(-0.030, -0.012, 0.054)),
        material=screen_black,
        name="display_bezel",
    )
    aed.visual(
        Box((0.067, 0.035, 0.007)),
        origin=Origin(xyz=(-0.030, -0.012, 0.058)),
        material=screen_white,
        name="display_screen",
    )
    aed.visual(
        Sphere(radius=0.007),
        origin=Origin(xyz=(0.042, 0.004, 0.058)),
        material=green_light,
        name="status_light",
    )
    aed.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(-0.057, 0.038, 0.054)),
        material=orange_button,
        name="shock_button",
    )
    aed.visual(
        Cylinder(radius=0.013, length=0.018),
        origin=Origin(xyz=(0.074, -0.006, 0.061), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blue_connector,
        name="electrode_socket",
    )
    aed.visual(
        Cylinder(radius=0.007, length=0.025),
        origin=Origin(xyz=(0.069, -0.006, 0.074)),
        material=blue_connector,
        name="plug_cap",
    )
    aed.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.074, -0.006, 0.063),
                    (0.070, -0.030, 0.087),
                    (0.012, -0.090, 0.100),
                    (-0.086, -0.112, 0.070),
                ],
                radius=0.0032,
                samples_per_segment=18,
                radial_segments=14,
                cap_ends=True,
            ),
            "white_electrode_cable",
        ),
        material=cable_white,
        name="white_electrode_cable",
    )

    flap = model.part("front_flap")
    flap.visual(
        Box((0.010, 0.112, 0.092)),
        origin=Origin(xyz=(-0.024, 0.0, -0.047)),
        material=black_fabric,
        name="soft_flap_body",
    )
    flap.visual(
        Box((0.004, 0.098, 0.060)),
        origin=Origin(xyz=(-0.031, 0.0, -0.051)),
        material=dark_foam,
        name="velcro_face",
    )
    flap.visual(
        Box((0.020, 0.116, 0.010)),
        origin=Origin(xyz=(-0.012, 0.0, -0.004)),
        material=black_fabric,
        name="sewn_hinge_band",
    )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.178, 0.0, 0.09985)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=-OPEN_LID_ANGLE,
            upper=math.radians(12.0),
        ),
    )
    model.articulation(
        "base_to_aed",
        ArticulationType.FIXED,
        parent=base,
        child=aed,
        origin=Origin(xyz=(-0.032, 0.035, 0.021)),
    )
    model.articulation(
        "base_to_front_flap",
        ArticulationType.REVOLUTE,
        parent=base,
        child=flap,
        origin=Origin(xyz=(-0.184, 0.0, 0.090)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("case_base")
    lid = object_model.get_part("lid")
    aed = object_model.get_part("aed_device")
    flap = object_model.get_part("front_flap")
    hinge = object_model.get_articulation("base_to_lid")

    ctx.expect_within(
        aed,
        base,
        axes="xy",
        margin=0.002,
        inner_elem="aed_body",
        outer_elem="inner_floor",
        name="AED device footprint is inside the tray floor",
    )
    ctx.expect_gap(
        aed,
        base,
        axis="z",
        positive_elem="aed_body",
        negative_elem="inner_floor",
        min_gap=-0.000001,
        max_gap=0.006,
        name="AED body rests just above the padded floor",
    )
    ctx.expect_overlap(
        lid,
        base,
        axes="y",
        min_overlap=0.20,
        name="open lid keeps the same case width span as the base",
    )
    ctx.expect_gap(
        base,
        flap,
        axis="x",
        positive_elem="front_latch_patch",
        negative_elem="soft_flap_body",
        min_gap=0.002,
        max_gap=0.035,
        name="front flap hangs outside the front latch area",
    )

    open_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid default pose is visibly open",
        open_aabb is not None and open_aabb[1][2] > 0.24,
        details=f"open_lid_aabb={open_aabb}",
    )

    with ctx.pose({hinge: -OPEN_LID_ANGLE}):
        closed_aabb = ctx.part_world_aabb(lid)
        ctx.expect_overlap(
            lid,
            base,
            axes="xy",
            min_overlap=0.18,
            name="closed lid covers the base footprint",
        )

    ctx.check(
        "hinged lid closes downward from the displayed open pose",
        open_aabb is not None
        and closed_aabb is not None
        and closed_aabb[1][2] < open_aabb[1][2] - 0.10,
        details=f"open={open_aabb}, closed={closed_aabb}",
    )
    ctx.check(
        "movable latch flap part is present",
        flap.name == "front_flap",
        details="front_flap should remain a separate hinged soft strap/latch element",
    )

    return ctx.report()


object_model = build_object_model()
