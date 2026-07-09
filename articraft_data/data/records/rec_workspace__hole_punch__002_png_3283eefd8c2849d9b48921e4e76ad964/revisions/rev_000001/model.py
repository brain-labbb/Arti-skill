from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
    tube_from_spline_points,
)


def _rounded_box_mesh(name: str, size: tuple[float, float, float], radius: float):
    """CadQuery rounded block authored in meters."""
    base = cq.Workplane("XY").box(*size)
    try:
        body = base.edges().fillet(radius)
    except Exception:
        body = cq.Workplane("XY").box(*size).edges("|Z").fillet(radius)
    return mesh_from_cadquery(body, name, tolerance=0.00045, angular_tolerance=0.08)


def _extruded_xz_mesh(
    name: str,
    profile_xz: list[tuple[float, float]],
    thickness_y: float,
):
    """Extrude a side profile drawn in X/Z into a centered Y-thickness mesh."""
    geom = ExtrudeGeometry(profile_xz, thickness_y, center=True).rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _lever_profile() -> list[tuple[float, float]]:
    # Clockwise side outline: low rear pivot cheek, lower pressing arm,
    # smoothly rising hand lever, and rounded front nose.
    return [
        (-0.024, -0.014),
        (0.020, -0.015),
        (0.068, -0.008),
        (0.086, 0.006),
        (0.102, 0.036),
        (0.128, 0.080),
        (0.154, 0.112),
        (0.166, 0.128),
        (0.161, 0.143),
        (0.146, 0.147),
        (0.094, 0.111),
        (0.038, 0.055),
        (0.007, 0.031),
        (-0.023, 0.018),
    ]


def _side_plate_profile() -> list[tuple[float, float]]:
    return [
        (-0.092, 0.024),
        (0.057, 0.024),
        (0.060, 0.038),
        (0.028, 0.046),
        (-0.014, 0.066),
        (-0.071, 0.071),
        (-0.091, 0.055),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="blue_desktop_two_hole_punch")

    blue = model.material("blue_painted_metal", rgba=(0.08, 0.24, 0.62, 1.0))
    blue_highlight = model.material("raised_blue_highlight", rgba=(0.12, 0.33, 0.78, 1.0))
    blue_shadow = model.material("dark_blue_shadow", rgba=(0.035, 0.09, 0.25, 1.0))
    rubber = model.material("black_rubber", rgba=(0.012, 0.012, 0.012, 1.0))
    black = model.material("black_slot", rgba=(0.0, 0.0, 0.0, 1.0))
    steel = model.material("brushed_steel", rgba=(0.78, 0.77, 0.72, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.30, 0.30, 0.32, 1.0))

    base = model.part("base")
    base.visual(
        _rounded_box_mesh("hole_punch_rubber_pad", (0.202, 0.088, 0.009), 0.0030),
        origin=Origin(xyz=(0.0, 0.0, 0.0045)),
        material=rubber,
        name="rubber_pad",
    )
    base.visual(
        _rounded_box_mesh("hole_punch_blue_base_shell", (0.190, 0.076, 0.023), 0.0045),
        origin=Origin(xyz=(0.0, 0.0, 0.0205)),
        material=blue,
        name="blue_base_shell",
    )
    base.visual(
        _rounded_box_mesh("hole_punch_top_channel", (0.145, 0.048, 0.014), 0.0030),
        origin=Origin(xyz=(-0.002, 0.0, 0.0380)),
        material=blue_highlight,
        name="top_channel",
    )
    base.visual(
        _rounded_box_mesh("hole_punch_front_lip", (0.174, 0.012, 0.010), 0.0020),
        origin=Origin(xyz=(0.004, -0.032, 0.034)),
        material=blue,
        name="front_base_lip",
    )
    base.visual(
        _rounded_box_mesh("hole_punch_rear_hinge_block", (0.040, 0.064, 0.027), 0.0025),
        origin=Origin(xyz=(-0.071, 0.0, 0.038)),
        material=blue,
        name="rear_hinge_block",
    )

    side_plate_mesh = _extruded_xz_mesh("hole_punch_side_plate", _side_plate_profile(), 0.006)
    base.visual(
        side_plate_mesh,
        origin=Origin(xyz=(0.0, -0.040, 0.0)),
        material=blue,
        name="side_plate_visible",
    )
    base.visual(
        side_plate_mesh,
        origin=Origin(xyz=(0.0, 0.040, 0.0)),
        material=blue,
        name="side_plate_far",
    )

    # Two visible screw heads on the outside plate, with small dark centers.
    for index, (x, z) in enumerate(((-0.075, 0.058), (-0.052, 0.052))):
        base.visual(
            Cylinder(radius=0.0062, length=0.0032),
            origin=Origin(xyz=(x, -0.0445, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"side_screw_{index}",
        )
        base.visual(
            Cylinder(radius=0.0024, length=0.0035),
            origin=Origin(xyz=(x, -0.0464, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"side_screw_slot_{index}",
        )

    waste_window = _extruded_xz_mesh(
        "hole_punch_black_oval_window",
        superellipse_profile(0.017, 0.0065, exponent=2.2, segments=28),
        0.0022,
    )
    base.visual(
        waste_window,
        origin=Origin(xyz=(-0.061, -0.0440, 0.037)),
        material=black,
        name="side_waste_window",
    )
    base.visual(
        _rounded_box_mesh("hole_punch_paper_throat_shadow", (0.092, 0.041, 0.004), 0.0015),
        origin=Origin(xyz=(-0.006, 0.0, 0.050)),
        material=blue_shadow,
        name="paper_throat_gap",
    )

    base.visual(
        Cylinder(radius=0.0076, length=0.0030),
        origin=Origin(xyz=(0.034, -0.022, 0.0470)),
        material=steel,
        name="die_ring_0",
    )
    base.visual(
        Cylinder(radius=0.0042, length=0.0034),
        origin=Origin(xyz=(0.034, -0.022, 0.0490)),
        material=black,
        name="die_hole_0",
    )
    base.visual(
        Cylinder(radius=0.0095, length=0.0045),
        origin=Origin(xyz=(0.034, -0.022, 0.0445)),
        material=blue,
        name="die_boss_0",
    )
    base.visual(
        Cylinder(radius=0.0076, length=0.0030),
        origin=Origin(xyz=(0.034, 0.022, 0.0470)),
        material=steel,
        name="die_ring_1",
    )
    base.visual(
        Cylinder(radius=0.0042, length=0.0034),
        origin=Origin(xyz=(0.034, 0.022, 0.0490)),
        material=black,
        name="die_hole_1",
    )
    base.visual(
        Cylinder(radius=0.0095, length=0.0045),
        origin=Origin(xyz=(0.034, 0.022, 0.0445)),
        material=blue,
        name="die_boss_1",
    )

    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.10, length=0.050),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    lever = model.part("pressing_lever")
    lever.visual(
        _extruded_xz_mesh("hole_punch_curved_handle", _lever_profile(), 0.052),
        material=blue,
        name="curved_handle",
    )
    lever.visual(
        _rounded_box_mesh("hole_punch_lower_press_channel", (0.145, 0.052, 0.014), 0.0025),
        origin=Origin(xyz=(0.060, 0.0, 0.004)),
        material=blue_highlight,
        name="lower_press_channel",
    )
    lever.visual(
        Cylinder(radius=0.0085, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=blue,
        name="hinge_barrel",
    )
    lever.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.006, 0.0, 0.035),
                    (0.044, 0.0, 0.065),
                    (0.094, 0.0, 0.116),
                    (0.144, 0.0, 0.143),
                ],
                radius=0.0043,
                samples_per_segment=14,
                radial_segments=14,
                cap_ends=True,
            ),
            "hole_punch_handle_raised_rib",
        ),
        material=blue_highlight,
        name="handle_raised_rib",
    )
    lever.inertial = Inertial.from_geometry(
        Cylinder(radius=0.060, length=0.180),
        mass=0.18,
        origin=Origin(xyz=(0.060, 0.0, 0.060), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    pin_x = 0.034
    pin_joint_z = 0.080
    for index, y in enumerate((-0.022, 0.022)):
        pin = model.part(f"punch_pin_{index}")
        pin.visual(
            Cylinder(radius=0.0028, length=0.025),
            origin=Origin(xyz=(0.0, 0.0, -0.0130)),
            material=steel,
            name="punch_pin",
        )
        pin.visual(
            Cylinder(radius=0.0058, length=0.0050),
            origin=Origin(xyz=(0.0, 0.0, 0.0020)),
            material=dark_steel,
            name="pin_top_cap",
        )
        pin.visual(
            Cylinder(radius=0.0032, length=0.0040),
            origin=Origin(xyz=(0.0, 0.0, -0.0270)),
            material=steel,
            name="pin_cutting_tip",
        )
        pin.inertial = Inertial.from_geometry(
            Cylinder(radius=0.004, length=0.034),
            mass=0.025,
            origin=Origin(xyz=(0.0, 0.0, -0.013)),
        )
    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lever,
        origin=Origin(xyz=(-0.070, 0.0, 0.068)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=9.0, velocity=2.0, lower=0.0, upper=0.16),
    )
    for index, y in enumerate((-0.022, 0.022)):
        model.articulation(
            f"lever_to_pin_{index}",
            ArticulationType.FIXED,
            parent=lever,
            child=f"punch_pin_{index}",
            origin=Origin(xyz=(pin_x + 0.070, y, pin_joint_z - 0.068)),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lever = object_model.get_part("pressing_lever")
    pin_0 = object_model.get_part("punch_pin_0")
    pin_1 = object_model.get_part("punch_pin_1")
    lever_joint = object_model.get_articulation("lever_pivot")
    pin_mount_0 = object_model.get_articulation("lever_to_pin_0")
    pin_mount_1 = object_model.get_articulation("lever_to_pin_1")

    base_visuals = {visual.name for visual in base.visuals}
    lever_visuals = {visual.name for visual in lever.visuals}

    ctx.check(
        "desktop hole punch base has layered blue body",
        {"blue_base_shell", "top_channel", "paper_throat_gap"}.issubset(base_visuals),
        details=f"base visuals={sorted(base_visuals)}",
    )
    ctx.check(
        "black rubber base pad is modeled",
        "rubber_pad" in base_visuals,
        details=f"base visuals={sorted(base_visuals)}",
    )
    ctx.check(
        "side plate has two visible screw heads",
        {"side_screw_0", "side_screw_1", "side_waste_window"}.issubset(base_visuals),
        details=f"base visuals={sorted(base_visuals)}",
    )
    ctx.check(
        "curved raised handle is modeled",
        {"curved_handle", "lower_press_channel", "handle_raised_rib", "hinge_barrel"}.issubset(lever_visuals),
        details=f"lever visuals={sorted(lever_visuals)}",
    )
    ctx.check(
        "lever joint is non fixed revolute",
        lever_joint.articulation_type == ArticulationType.REVOLUTE
        and lever_joint.motion_limits is not None
        and lever_joint.motion_limits.upper > lever_joint.motion_limits.lower,
        details=f"type={lever_joint.articulation_type}, limits={lever_joint.motion_limits}",
    )
    ctx.check(
        "twin punch pins are carried by the lever",
        pin_mount_0.articulation_type == ArticulationType.FIXED
        and pin_mount_1.articulation_type == ArticulationType.FIXED,
        details=f"pin mounts={pin_mount_0}, {pin_mount_1}",
    )
    ctx.check(
        "paired round punch dies are in the base",
        {"die_ring_0", "die_ring_1", "die_hole_0", "die_hole_1"}.issubset(base_visuals),
        details=f"base visuals={sorted(base_visuals)}",
    )

    ctx.expect_overlap(
        pin_0,
        base,
        axes="xy",
        elem_a="punch_pin",
        elem_b="die_hole_0",
        min_overlap=0.003,
        name="first punch pin aligns with first die",
    )
    ctx.expect_overlap(
        pin_1,
        base,
        axes="xy",
        elem_a="punch_pin",
        elem_b="die_hole_1",
        min_overlap=0.003,
        name="second punch pin aligns with second die",
    )
    ctx.expect_gap(
        pin_0,
        base,
        axis="z",
        positive_elem="punch_pin",
        negative_elem="die_hole_0",
        min_gap=0.002,
        max_gap=0.012,
        name="open first pin clears die throat",
    )
    ctx.allow_overlap(
        lever,
        pin_0,
        elem_a="lower_press_channel",
        elem_b="pin_top_cap",
        reason="The top of the punch pin is locally captured inside the simplified pressing channel.",
    )
    ctx.allow_overlap(
        lever,
        pin_1,
        elem_a="lower_press_channel",
        elem_b="pin_top_cap",
        reason="The top of the punch pin is locally captured inside the simplified pressing channel.",
    )
    ctx.expect_overlap(
        lever,
        pin_0,
        axes="xy",
        elem_a="lower_press_channel",
        elem_b="pin_top_cap",
        min_overlap=0.004,
        name="first pin cap sits under press channel",
    )
    ctx.expect_overlap(
        lever,
        pin_1,
        axes="xy",
        elem_a="lower_press_channel",
        elem_b="pin_top_cap",
        min_overlap=0.004,
        name="second pin cap sits under press channel",
    )

    open_pin_pos = ctx.part_world_position(pin_0)
    open_channel_aabb = ctx.part_element_world_aabb(lever, elem="lower_press_channel")
    with ctx.pose({lever_joint: lever_joint.motion_limits.upper}):
        closed_pin_pos = ctx.part_world_position(pin_0)
        closed_channel_aabb = ctx.part_element_world_aabb(lever, elem="lower_press_channel")
    ctx.check(
        "squeezing lever drives punch pins downward",
        open_pin_pos is not None
        and closed_pin_pos is not None
        and closed_pin_pos[2] < open_pin_pos[2] - 0.006,
        details=f"open={open_pin_pos}, closed={closed_pin_pos}",
    )
    ctx.check(
        "pressing channel descends during squeeze",
        open_channel_aabb is not None
        and closed_channel_aabb is not None
        and closed_channel_aabb[0][2] < open_channel_aabb[0][2] - 0.010,
        details=f"open={open_channel_aabb}, closed={closed_channel_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
