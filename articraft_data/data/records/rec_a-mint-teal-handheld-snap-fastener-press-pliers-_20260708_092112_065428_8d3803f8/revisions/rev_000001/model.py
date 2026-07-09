from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Snap fastener press pliers (hand press for setting snap buttons on fabric).
#
# Frame convention: +X points from the handles toward the press head, +Z is up.
# Root part `body` = lower handle + C-shaped head frame + base die.
# `lever` = upper squeeze handle, revolute on the pivot screw pin.
# `plunger` = vertical press ram, prismatic mimic of the lever squeeze.
# ---------------------------------------------------------------------------

PIVOT = (0.086, 0.0, 0.022)  # pivot screw axis (world)
PLUNGER_X = 0.062  # press ram axis (world)
SLEEVE_TOP_Z = 0.021  # plunger joint seating plane (world)
LEVER_TRAVEL = 0.21  # rad, full squeeze
RAM_TRAVEL = 0.005  # m, plunger stroke at full squeeze


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="snap_fastener_press_pliers")

    teal = model.material("mint_teal", rgba=(0.60, 0.84, 0.79, 1.0))
    steel = model.material("polished_steel", rgba=(0.78, 0.79, 0.81, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.38, 0.39, 0.42, 1.0))
    nylon = model.material("white_nylon", rgba=(0.94, 0.94, 0.91, 1.0))
    brass = model.material("brass", rgba=(0.79, 0.64, 0.31, 1.0))

    # ------------------------------------------------------------------ body
    body = model.part("body")

    # Lower jaw of the C-frame (carries the base die).
    jaw = ExtrudeGeometry(rounded_rect_profile(0.062, 0.015, 0.006), 0.012, center=True)
    body.visual(
        mesh_from_geometry(jaw, "lower_jaw"),
        origin=Origin(xyz=(0.065, 0.0, -0.036)),
        material=teal,
        name="lower_jaw",
    )

    # Front column of the head (rises from the jaw to the pivot).
    front_col = ExtrudeGeometry(rounded_rect_profile(0.016, 0.046, 0.005), 0.014, center=True)
    front_col.rotate_x(math.pi / 2.0)
    body.visual(
        mesh_from_geometry(front_col, "front_column"),
        origin=Origin(xyz=(0.086, 0.0, -0.009)),
        material=teal,
        name="front_column",
    )

    # Rear column of the head (closes the C behind the press window).
    rear_col = ExtrudeGeometry(rounded_rect_profile(0.014, 0.050, 0.005), 0.014, center=True)
    rear_col.rotate_x(math.pi / 2.0)
    body.visual(
        mesh_from_geometry(rear_col, "rear_column"),
        origin=Origin(xyz=(0.043, 0.0, -0.009)),
        material=teal,
        name="rear_column",
    )

    # Top guide arm: two blocks flanking the steel guide sleeve that carries
    # the press ram, so the ram runs through a true hollow guide.
    arm_rear = ExtrudeGeometry(rounded_rect_profile(0.021, 0.014, 0.005), 0.012, center=True)
    body.visual(
        mesh_from_geometry(arm_rear, "guide_arm_rear"),
        origin=Origin(xyz=(0.0465, 0.0, 0.014)),
        material=teal,
        name="guide_arm_rear",
    )
    arm_front = ExtrudeGeometry(rounded_rect_profile(0.014, 0.014, 0.005), 0.012, center=True)
    body.visual(
        mesh_from_geometry(arm_front, "guide_arm_front"),
        origin=Origin(xyz=(0.073, 0.0, 0.011)),
        material=teal,
        name="guide_arm_front",
    )

    # Steel guide sleeve pressed into the arm bore; the ram slides inside it.
    sleeve = LatheGeometry.from_shell_profiles(
        [(0.0060, 0.0), (0.0060, 0.017)],
        [(0.0042, 0.0), (0.0042, 0.017)],
        segments=32,
    )
    body.visual(
        mesh_from_geometry(sleeve, "guide_sleeve"),
        origin=Origin(xyz=(PLUNGER_X, 0.0, 0.004)),
        material=steel,
        name="guide_sleeve",
    )

    # Pivot saddle cap and clevis ears on top of the front column.
    body.visual(
        Box((0.017, 0.024, 0.005)),
        origin=Origin(xyz=(0.086, 0.0, 0.0125)),
        material=teal,
        name="pivot_saddle",
    )
    for i, sy in enumerate((1.0, -1.0)):
        body.visual(
            Cylinder(radius=0.0085, length=0.0035),
            origin=Origin(xyz=(0.086, sy * 0.00925, 0.022), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=teal,
            name=f"pivot_ear_{i}",
        )

    # Pivot screw pin through the ears and the lever boss.
    body.visual(
        Cylinder(radius=0.0022, length=0.025),
        origin=Origin(xyz=PIVOT, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_pin",
    )
    body.visual(
        Cylinder(radius=0.0036, length=0.0025),
        origin=Origin(xyz=(0.086, 0.0128, 0.022), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_screw_head",
    )
    body.visual(
        Cylinder(radius=0.0036, length=0.0025),
        origin=Origin(xyz=(0.086, -0.0128, 0.022), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_nut",
    )

    # Base die stack seated on the lower jaw, with a snap socket loaded on it.
    body.visual(
        Cylinder(radius=0.0062, length=0.0050),
        origin=Origin(xyz=(PLUNGER_X, 0.0, -0.028)),
        material=steel,
        name="base_die",
    )
    body.visual(
        Cylinder(radius=0.0028, length=0.0055),
        origin=Origin(xyz=(PLUNGER_X, 0.0, -0.0228)),
        material=steel,
        name="die_post",
    )
    socket_ring = LatheGeometry.from_shell_profiles(
        [(0.0056, 0.0), (0.0056, 0.002)],
        [(0.0034, 0.0), (0.0034, 0.002)],
        segments=32,
    )
    body.visual(
        mesh_from_geometry(socket_ring, "snap_socket"),
        origin=Origin(xyz=(PLUNGER_X, 0.0, -0.0257)),
        material=brass,
        name="snap_socket",
    )

    # Fixed lower handle sweeping back from the head.
    lower_handle = sweep_profile_along_spline(
        [
            (0.042, 0.0, -0.034),
            (0.014, 0.0, -0.033),
            (-0.024, 0.0, -0.035),
            (-0.058, 0.0, -0.040),
            (-0.086, 0.0, -0.048),
        ],
        profile=rounded_rect_profile(0.013, 0.0085, 0.003),
        samples_per_segment=10,
    )
    body.visual(
        mesh_from_geometry(lower_handle, "lower_handle"),
        material=teal,
        name="lower_handle",
    )

    # Return coil spring standing on the lower handle, reaching the lever.
    spring_pts = []
    turns = 6.5
    n_pts = 80
    z0, z1 = -0.0310, 0.0186
    for i in range(n_pts + 1):
        t = i / n_pts
        ang = 2.0 * math.pi * turns * t
        spring_pts.append(
            (-0.020 + 0.005 * math.cos(ang), 0.005 * math.sin(ang), z0 + (z1 - z0) * t)
        )
    spring = tube_from_spline_points(
        spring_pts,
        radius=0.0012,
        samples_per_segment=2,
        radial_segments=10,
    )
    body.visual(
        mesh_from_geometry(spring, "return_spring"),
        material=steel,
        name="return_spring",
    )

    # ----------------------------------------------------------------- lever
    # Child frame sits on the pivot axis; geometry is authored pivot-local.
    lever = model.part("lever")
    lever.visual(
        Cylinder(radius=0.006, length=0.013),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=teal,
        name="lever_boss",
    )
    upper_handle = sweep_profile_along_spline(
        [
            (-0.003, 0.0, 0.001),
            (-0.024, 0.0, 0.018),
            (-0.055, 0.0, 0.013),
            (-0.095, 0.0, 0.007),
            (-0.135, 0.0, -0.006),
            (-0.165, 0.0, -0.020),
        ],
        profile=rounded_rect_profile(0.013, 0.0085, 0.003),
        samples_per_segment=10,
    )
    lever.visual(
        mesh_from_geometry(upper_handle, "lever_handle"),
        material=teal,
        name="lever_handle",
    )
    # Press pad that seats on the plunger top button (tiny intentional embed).
    lever.visual(
        Box((0.012, 0.010, 0.005)),
        origin=Origin(xyz=(-0.024, 0.0, 0.0124)),
        material=teal,
        name="press_pad",
    )

    model.articulation(
        "body_to_lever",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(xyz=PIVOT),
        # Handle and press pad extend along local -X; -Y axis makes positive q
        # squeeze the lever down toward the lower handle.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=3.0, lower=0.0, upper=LEVER_TRAVEL),
    )

    # --------------------------------------------------------------- plunger
    # Joint frame at the guide sleeve top; +axis (0,0,-1) presses downward.
    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.0035, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, -0.0075)),
        material=steel,
        name="ram_shaft",
    )
    plunger.visual(
        Cylinder(radius=0.0045, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0095)),
        material=steel,
        name="top_button",
    )
    plunger.visual(
        Cylinder(radius=0.0058, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.027)),
        material=nylon,
        name="nylon_tip",
    )
    plunger.visual(
        Cylinder(radius=0.0052, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.0325)),
        material=steel,
        name="cap_die",
    )
    plunger.visual(
        Cylinder(radius=0.0056, length=0.0015),
        origin=Origin(xyz=(0.0, 0.0, -0.0345)),
        material=brass,
        name="snap_cap",
    )

    model.articulation(
        "body_to_plunger",
        ArticulationType.PRISMATIC,
        parent=body,
        child=plunger,
        origin=Origin(xyz=(PLUNGER_X, 0.0, SLEEVE_TOP_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=200.0, velocity=0.05, lower=0.0, upper=RAM_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lever = object_model.get_part("lever")
    plunger = object_model.get_part("plunger")
    squeeze = object_model.get_articulation("body_to_lever")
    press = object_model.get_articulation("body_to_plunger")

    # The pivot screw pin is intentionally captured inside the lever boss bore.
    ctx.allow_overlap(
        body,
        lever,
        elem_a="pivot_pin",
        elem_b="lever_boss",
        reason="Pivot screw pin is captured inside the lever boss hinge bore.",
    )
    # The press pad seats on the ram top button with a tiny intentional embed
    # so the drive contact between lever and ram is a real touching interface.
    ctx.allow_overlap(
        lever,
        plunger,
        elem_a="press_pad",
        elem_b="top_button",
        reason="Press pad is seated on the ram top button; the lever drives the ram through this contact.",
    )

    # Ram is centered in and retained by the steel guide sleeve.
    ctx.expect_within(
        plunger,
        body,
        axes="xy",
        inner_elem="ram_shaft",
        outer_elem="guide_sleeve",
        margin=0.001,
        name="ram shaft stays centered in the guide sleeve",
    )
    ctx.expect_overlap(
        plunger,
        body,
        axes="z",
        elem_a="ram_shaft",
        elem_b="guide_sleeve",
        min_overlap=0.012,
        name="ram shaft remains inserted in the guide sleeve at rest",
    )

    # Lever press pad seats directly on the plunger top button.
    ctx.expect_gap(
        lever,
        plunger,
        axis="z",
        positive_elem="press_pad",
        negative_elem="top_button",
        max_penetration=0.0008,
        max_gap=0.0005,
        name="press pad is seated on the plunger top button",
    )
    ctx.expect_contact(
        lever,
        plunger,
        elem_a="press_pad",
        elem_b="top_button",
        name="press pad touches the plunger top button",
    )

    # Loaded snap cap hangs coaxially above the base die post with a work gap.
    ctx.expect_overlap(
        plunger,
        body,
        axes="xy",
        elem_a="snap_cap",
        elem_b="die_post",
        min_overlap=0.004,
        name="snap cap is aligned over the base die post",
    )
    ctx.expect_gap(
        plunger,
        body,
        axis="z",
        positive_elem="snap_cap",
        negative_elem="die_post",
        min_gap=0.004,
        max_gap=0.008,
        name="open work gap between snap cap and die post at rest",
    )

    # Return spring stands on the lower handle and reaches the lever underside.
    ctx.expect_contact(
        lever,
        body,
        elem_a="lever_handle",
        elem_b="return_spring",
        contact_tol=0.0015,
        name="return spring reaches the squeeze lever",
    )
    ctx.expect_contact(
        body,
        body,
        elem_a="return_spring",
        elem_b="lower_handle",
        contact_tol=0.0005,
        name="return spring is seated on the lower handle",
    )

    rest_plunger = ctx.part_world_position(plunger)
    rest_lever_aabb = ctx.part_world_aabb(lever)

    # Full squeeze: lever drives the ram down until the cap nearly meets the die.
    with ctx.pose({squeeze: LEVER_TRAVEL, press: RAM_TRAVEL}):
        ctx.expect_gap(
            plunger,
            body,
            axis="z",
            positive_elem="snap_cap",
            negative_elem="die_post",
            min_gap=0.0,
            max_gap=0.002,
            name="full squeeze closes the snap cap onto the die post",
        )
        ctx.expect_overlap(
            plunger,
            body,
            axes="z",
            elem_a="ram_shaft",
            elem_b="guide_sleeve",
            min_overlap=0.010,
            name="ram shaft retains insertion at full stroke",
        )
        pressed_plunger = ctx.part_world_position(plunger)
        pressed_lever_aabb = ctx.part_world_aabb(lever)

    ctx.check(
        "squeeze drives the plunger downward",
        rest_plunger is not None
        and pressed_plunger is not None
        and pressed_plunger[2] < rest_plunger[2] - 0.004,
        details=f"rest={rest_plunger}, pressed={pressed_plunger}",
    )
    ctx.check(
        "squeezing swings the lever handle toward the lower handle",
        rest_lever_aabb is not None
        and pressed_lever_aabb is not None
        and pressed_lever_aabb[0][2] < rest_lever_aabb[0][2] - 0.02,
        details=f"rest_min_z={rest_lever_aabb}, pressed_min_z={pressed_lever_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
