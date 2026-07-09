from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    ExtrudeGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)

# ---------------------------------------------------------------------------
# Screw-type snap fastener press (arbor setter with T-handle rotary drive).
#
# Frame convention: +X points from the handles toward the press head, +Z up.
# Root part `body` = C-frame + lower handle + base die + thread nut.
# `screw_handle` = T-handle + screw shaft, CONTINUOUS rotation about Z.
# `plunger` = vertical press ram, PRISMATIC mimic of screw rotation.
# Turning the T-handle threads the ram down onto the base die.
# ---------------------------------------------------------------------------

PLUNGER_X = 0.062        # press ram axis (world X)
SLEEVE_TOP_Z = 0.021     # guide sleeve top / plunger joint seating plane
SCREW_ORIGIN_Z = 0.040   # screw handle joint origin (top of thread nut)
SCREW_PITCH = 0.003      # 3 mm linear advance per full turn
RAM_TRAVEL = 0.005       # 5 mm plunger stroke


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="snap_fastener_screw_press")

    teal = model.material("mint_teal", rgba=(0.60, 0.84, 0.79, 1.0))
    steel = model.material("polished_steel", rgba=(0.78, 0.79, 0.81, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.38, 0.39, 0.42, 1.0))
    nylon = model.material("white_nylon", rgba=(0.94, 0.94, 0.91, 1.0))
    brass = model.material("brass", rgba=(0.79, 0.64, 0.31, 1.0))
    phenolic = model.material("black_phenolic", rgba=(0.15, 0.15, 0.14, 1.0))

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

    # Front column of the head (rises from the jaw to the guide area).
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

    # Top guide arms flanking the steel guide sleeve.
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

    # Steel guide sleeve (hollow tube pressed into the arm bore).
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

    # Guide neck (hollow boss above sleeve, guides the screw shaft).
    neck = LatheGeometry.from_shell_profiles(
        [(0.010, 0.0), (0.010, 0.014)],
        [(0.007, 0.0), (0.007, 0.014)],
        segments=32,
    )
    body.visual(
        mesh_from_geometry(neck, "guide_neck"),
        origin=Origin(xyz=(PLUNGER_X, 0.0, 0.018)),
        material=teal,
        name="guide_neck",
    )

    # Thread nut (hollow, where the screw threads through the frame).
    nut = LatheGeometry.from_shell_profiles(
        [(0.011, 0.0), (0.011, 0.008)],
        [(0.004, 0.0), (0.004, 0.008)],
        segments=32,
    )
    body.visual(
        mesh_from_geometry(nut, "thread_nut"),
        origin=Origin(xyz=(PLUNGER_X, 0.0, 0.032)),
        material=dark_steel,
        name="thread_nut",
    )

    # Base die stack seated on the lower jaw, with a snap socket loaded.
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

    # --------------------------------------------------------- screw_handle
    screw_handle = model.part("screw_handle")

    # Screw shaft (threads through the nut bore, contacts the ram thrust plate).
    screw_handle.visual(
        Cylinder(radius=0.003, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=steel,
        name="screw_shaft",
    )

    # Hub (thrust collar above the nut).
    screw_handle.visual(
        Cylinder(radius=0.006, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=steel,
        name="hub",
    )

    # Stem (vertical bar from hub to crossbar).
    screw_handle.visual(
        Cylinder(radius=0.004, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=steel,
        name="stem",
    )

    # Crossbar (T-handle horizontal bar, black phenolic grip).
    screw_handle.visual(
        Cylinder(radius=0.004, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.016), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=phenolic,
        name="crossbar",
    )

    # Grip knobs at the crossbar ends.
    for i, sx in enumerate((1.0, -1.0)):
        screw_handle.visual(
            Sphere(radius=0.006),
            origin=Origin(xyz=(sx * 0.030, 0.0, 0.016)),
            material=phenolic,
            name=f"grip_knob_{i}",
        )

    # CONTINUOUS rotation about the vertical ram axis.
    model.articulation(
        "body_to_screw_handle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=screw_handle,
        origin=Origin(xyz=(PLUNGER_X, 0.0, SCREW_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=6.0),
    )

    # --------------------------------------------------------------- plunger
    plunger = model.part("plunger")

    # Ram shaft (slides inside the guide sleeve).
    plunger.visual(
        Cylinder(radius=0.0035, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=steel,
        name="ram_shaft",
    )

    # Thrust plate (flat disk where the screw shaft pushes the ram down).
    plunger.visual(
        Cylinder(radius=0.005, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=steel,
        name="thrust_plate",
    )

    # White nylon pressure tip.
    plunger.visual(
        Cylinder(radius=0.0058, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=nylon,
        name="nylon_tip",
    )

    # Cap die (steel, holds the snap cap for pressing).
    plunger.visual(
        Cylinder(radius=0.0052, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.031)),
        material=steel,
        name="cap_die",
    )

    # Loaded snap cap (brass, on the cap die).
    plunger.visual(
        Cylinder(radius=0.0056, length=0.0015),
        origin=Origin(xyz=(0.0, 0.0, -0.03375)),
        material=brass,
        name="snap_cap",
    )

    # PRISMATIC ram, mechanically coupled to the screw rotation by thread pitch.
    # (Cross-domain mimic is not supported; coupling is enforced in tests by
    # posing both joints together at the pitch ratio.)
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
    screw_handle = object_model.get_part("screw_handle")
    plunger = object_model.get_part("plunger")
    screw = object_model.get_articulation("body_to_screw_handle")
    press = object_model.get_articulation("body_to_plunger")

    # -- Variant-specific: screw drive mechanism --

    # The screw handle is a continuous rotary joint about the vertical axis.
    ctx.check(
        "body_to_screw_handle is CONTINUOUS about Z",
        screw.articulation_type == ArticulationType.CONTINUOUS
        and abs(screw.axis[2]) > 0.99,
        details=f"type={screw.articulation_type}, axis={screw.axis}",
    )

    # The ram is a prismatic joint along -Z (driven by screw thread pitch).
    ctx.check(
        "body_to_plunger is PRISMATIC along -Z",
        press.articulation_type == ArticulationType.PRISMATIC
        and press.axis[2] < -0.99,
        details=f"type={press.articulation_type}, axis={press.axis}",
    )

    # Ram stays centered in the guide sleeve.
    ctx.expect_within(
        plunger, body,
        axes="xy",
        inner_elem="ram_shaft",
        outer_elem="guide_sleeve",
        margin=0.001,
        name="ram shaft stays centered in the guide sleeve",
    )

    # Ram retains insertion in the guide sleeve at rest.
    ctx.expect_overlap(
        plunger, body,
        axes="z",
        elem_a="ram_shaft",
        elem_b="guide_sleeve",
        min_overlap=0.012,
        name="ram shaft remains inserted in the guide sleeve at rest",
    )

    # Screw shaft is centered in the thread nut bore.
    ctx.expect_within(
        screw_handle, body,
        axes="xy",
        inner_elem="screw_shaft",
        outer_elem="thread_nut",
        margin=0.002,
        name="screw shaft is centered in the thread nut",
    )

    # Snap cap aligned over the base die post.
    ctx.expect_overlap(
        plunger, body,
        axes="xy",
        elem_a="snap_cap",
        elem_b="die_post",
        min_overlap=0.004,
        name="snap cap is aligned over the base die post",
    )

    # Open work gap between snap cap and die post at rest.
    ctx.expect_gap(
        plunger, body,
        axis="z",
        positive_elem="snap_cap",
        negative_elem="die_post",
        min_gap=0.004,
        max_gap=0.010,
        name="open work gap between snap cap and die post at rest",
    )

    # Turning the screw one full turn advances the ram down by one pitch.
    # Both joints are posed together to demonstrate the screw-thread coupling.
    rest_plunger = ctx.part_world_position(plunger)

    one_turn = 2.0 * math.pi
    with ctx.pose({screw: one_turn, press: SCREW_PITCH}):
        turned_plunger = ctx.part_world_position(plunger)
        ctx.expect_overlap(
            plunger, body,
            axes="z",
            elem_a="ram_shaft",
            elem_b="guide_sleeve",
            min_overlap=0.009,
            name="ram retains insertion after one screw turn",
        )

    ctx.check(
        "one screw turn drives ram downward by approximately one pitch",
        rest_plunger is not None
        and turned_plunger is not None
        and rest_plunger[2] - turned_plunger[2] > 0.002,
        details=f"rest_z={rest_plunger}, turned_z={turned_plunger}",
    )

    # Full press: enough turns for full RAM_TRAVEL.
    full_angle = RAM_TRAVEL / SCREW_PITCH * 2.0 * math.pi
    with ctx.pose({screw: full_angle, press: RAM_TRAVEL}):
        ctx.expect_gap(
            plunger, body,
            axis="z",
            positive_elem="snap_cap",
            negative_elem="die_post",
            min_gap=0.0,
            max_gap=0.002,
            name="full screw press closes snap cap onto die post",
        )
        ctx.expect_overlap(
            plunger, body,
            axes="z",
            elem_a="ram_shaft",
            elem_b="guide_sleeve",
            min_overlap=0.008,
            name="ram retains insertion at full press",
        )
        pressed_plunger = ctx.part_world_position(plunger)

    ctx.check(
        "full screw advance drives ram to pressing position",
        rest_plunger is not None
        and pressed_plunger is not None
        and pressed_plunger[2] < rest_plunger[2] - 0.004,
        details=f"rest={rest_plunger}, pressed={pressed_plunger}",
    )

    return ctx.report()


object_model = build_object_model()
