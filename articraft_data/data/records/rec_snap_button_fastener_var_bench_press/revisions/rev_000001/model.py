from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Bench-mounted snap fastener press (KAM-style desktop hand press for setting
# snap buttons on fabric).  Converted from the handheld C-frame pliers to a
# table-clamped vertical press: a flat base plate carries the die stack, a
# single rear column rises from it, and the squeeze lever pivots from the
# column top to drive the guided ram straight down.
#
# Frame convention: +X points forward (toward the operator / ram side),
# +Z is up.  Root part `body` = base plate + column + head + die stack.
# `lever` = long top squeeze handle, revolute on the pivot screw pin.
# `plunger` = vertical press ram, prismatic mimic of the lever squeeze.
# ---------------------------------------------------------------------------

PIVOT_X = 0.045
PIVOT_Z = 0.200
PIVOT = (PIVOT_X, 0.0, PIVOT_Z)
PLUNGER_X = 0.025
SLEEVE_TOP_Z = 0.195
COLUMN_X = -0.025
LEVER_TRAVEL = 0.26   # rad, full squeeze
RAM_TRAVEL = 0.005    # m, plunger stroke at full squeeze


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_snap_press")

    teal = model.material("mint_teal", rgba=(0.60, 0.84, 0.79, 1.0))
    steel = model.material("polished_steel", rgba=(0.78, 0.79, 0.81, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.38, 0.39, 0.42, 1.0))
    nylon = model.material("white_nylon", rgba=(0.94, 0.94, 0.91, 1.0))
    brass = model.material("brass", rgba=(0.79, 0.64, 0.31, 1.0))
    cast_iron = model.material("cast_iron", rgba=(0.22, 0.24, 0.25, 1.0))

    # ------------------------------------------------------------------ body
    body = model.part("body")

    # -- Base plate: wide flat foot that sits on the bench -----------------
    base_plate_shape = (
        cq.Workplane("XY")
        .box(0.14, 0.09, 0.014)
        .edges("|Z").fillet(0.006)
    )
    body.visual(
        mesh_from_cadquery(base_plate_shape, "base_plate"),
        origin=Origin(xyz=(0.010, 0.0, 0.007)),
        material=cast_iron,
        name="base_plate",
    )

    # -- Vertical rear column rising from the base plate -------------------
    body.visual(
        Cylinder(radius=0.014, length=0.172),
        origin=Origin(xyz=(COLUMN_X, 0.0, 0.014 + 0.086)),
        material=teal,
        name="column",
    )

    # Column base flange (wider ring seating the column on the plate).
    body.visual(
        Cylinder(radius=0.020, length=0.008),
        origin=Origin(xyz=(COLUMN_X, 0.0, 0.014 + 0.004)),
        material=teal,
        name="column_flange",
    )

    # Head / column cap: extends forward from the column top, carrying the
    # guide sleeve bore and the pivot ears.  A through-bore clears the ram.
    head_shape = (
        cq.Workplane("XY")
        .box(0.080, 0.030, 0.014)
        .edges("|Z").fillet(0.004)
        .faces(">Z").workplane()
        .center(0.020, 0.0)
        .circle(0.0065)
        .cutThruAll()
    )
    body.visual(
        mesh_from_cadquery(head_shape, "column_cap"),
        origin=Origin(xyz=(0.005, 0.0, 0.185)),
        material=teal,
        name="column_cap",
    )

    # Steel guide sleeve pressed through the head bore; the ram slides inside.
    sleeve = LatheGeometry.from_shell_profiles(
        [(0.0070, 0.0), (0.0070, 0.025)],
        [(0.0050, 0.0), (0.0050, 0.025)],
        segments=32,
    )
    body.visual(
        mesh_from_geometry(sleeve, "guide_sleeve"),
        origin=Origin(xyz=(PLUNGER_X, 0.0, 0.175)),
        material=steel,
        name="guide_sleeve",
    )

    # Pivot saddle and clevis ears at the front of the head.
    body.visual(
        Box((0.020, 0.028, 0.014)),
        origin=Origin(xyz=(PIVOT_X, 0.0, 0.196)),
        material=teal,
        name="pivot_saddle",
    )
    for i, sy in enumerate((1.0, -1.0)):
        body.visual(
            Cylinder(radius=0.009, length=0.004),
            origin=Origin(
                xyz=(PIVOT_X, sy * 0.010, PIVOT_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=teal,
            name=f"pivot_ear_{i}",
        )

    # Pivot screw pin through the ears and the lever boss.
    body.visual(
        Cylinder(radius=0.0025, length=0.028),
        origin=Origin(xyz=PIVOT, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_pin",
    )
    body.visual(
        Cylinder(radius=0.004, length=0.003),
        origin=Origin(
            xyz=(PIVOT_X, 0.014, PIVOT_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_steel,
        name="pivot_screw_head",
    )
    body.visual(
        Cylinder(radius=0.004, length=0.003),
        origin=Origin(
            xyz=(PIVOT_X, -0.014, PIVOT_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_steel,
        name="pivot_nut",
    )

    # -- Base die stack seated on the base plate, directly under the ram ---
    body.visual(
        Cylinder(radius=0.007, length=0.006),
        origin=Origin(xyz=(PLUNGER_X, 0.0, 0.017)),
        material=steel,
        name="base_die",
    )
    body.visual(
        Cylinder(radius=0.003, length=0.005),
        origin=Origin(xyz=(PLUNGER_X, 0.0, 0.0225)),
        material=steel,
        name="die_post",
    )
    socket_ring = LatheGeometry.from_shell_profiles(
        [(0.006, 0.0), (0.006, 0.002)],
        [(0.0029, 0.0), (0.0029, 0.002)],
        segments=32,
    )
    body.visual(
        mesh_from_geometry(socket_ring, "snap_socket"),
        origin=Origin(xyz=(PLUNGER_X, 0.0, 0.019)),
        material=brass,
        name="snap_socket",
    )

    # -- Spring bracket on the column and return coil spring ---------------
    body.visual(
        Box((0.016, 0.012, 0.004)),
        origin=Origin(xyz=(COLUMN_X - 0.017, 0.0, 0.170)),
        material=teal,
        name="spring_bracket",
    )

    spring_pts: list[tuple[float, float, float]] = []
    turns = 6.0
    n_pts = 72
    spring_cx = COLUMN_X - 0.025
    z0, z1 = 0.172, 0.204
    for i in range(n_pts + 1):
        t = i / n_pts
        ang = 2.0 * math.pi * turns * t
        spring_pts.append(
            (
                spring_cx + 0.005 * math.cos(ang),
                0.005 * math.sin(ang),
                z0 + (z1 - z0) * t,
            )
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
        Cylinder(radius=0.007, length=0.015),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=teal,
        name="lever_boss",
    )
    lever_handle = sweep_profile_along_spline(
        [
            (-0.003, 0.0, -0.001),
            (-0.024, 0.0, 0.015),
            (-0.055, 0.0, 0.014),
            (-0.095, 0.0, 0.010),
            (-0.135, 0.0, 0.003),
            (-0.165, 0.0, -0.010),
        ],
        profile=rounded_rect_profile(0.014, 0.009, 0.003),
        samples_per_segment=10,
    )
    lever.visual(
        mesh_from_geometry(lever_handle, "lever_handle"),
        material=teal,
        name="lever_handle",
    )
    # Press pad that seats on the plunger top button (tiny intentional embed).
    lever.visual(
        Box((0.014, 0.012, 0.006)),
        origin=Origin(xyz=(-0.020, 0.0, 0.001)),
        material=teal,
        name="press_pad",
    )

    model.articulation(
        "body_to_lever",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(xyz=PIVOT),
        # Handle extends backward in local -X from the pivot; -Y axis makes
        # positive q squeeze the lever downward toward the column.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0, velocity=3.0, lower=0.0, upper=LEVER_TRAVEL,
        ),
    )

    # --------------------------------------------------------------- plunger
    # Joint frame at the guide sleeve top; +axis (0,0,-1) presses downward.
    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.004, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
        material=steel,
        name="ram_shaft",
    )
    plunger.visual(
        Cylinder(radius=0.005, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=steel,
        name="top_button",
    )
    plunger.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.1535)),
        material=nylon,
        name="nylon_tip",
    )
    plunger.visual(
        Cylinder(radius=0.005, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.1605)),
        material=steel,
        name="cap_die",
    )
    plunger.visual(
        Cylinder(radius=0.006, length=0.0015),
        origin=Origin(xyz=(0.0, 0.0, -0.16325)),
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
        motion_limits=MotionLimits(
            effort=300.0, velocity=0.05, lower=0.0, upper=RAM_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lever = object_model.get_part("lever")
    plunger = object_model.get_part("plunger")
    squeeze = object_model.get_articulation("body_to_lever")
    press = object_model.get_articulation("body_to_plunger")

    # -- Structural skeleton: bench-mount base plate + vertical column -----
    # The body must have the bench-mount visuals (base_plate and column)
    # instead of the parent C-frame (front_column / rear_column / lower_handle).
    ctx.check(
        "body has bench-mount base_plate and vertical column",
        body.get_visual("base_plate") is not None
        and body.get_visual("column") is not None,
        details="Bench press body must have base_plate and column visuals.",
    )

    # The column stands on the base plate (seated contact at the junction).
    ctx.expect_gap(
        body,
        body,
        axis="z",
        positive_elem="column",
        negative_elem="base_plate",
        max_penetration=0.001,
        max_gap=0.001,
        name="column stands on the base plate",
    )

    # The base plate is substantially wider than the column for bench stability.
    bp_aabb = ctx.part_element_world_aabb(body, elem="base_plate")
    col_aabb = ctx.part_element_world_aabb(body, elem="column")
    if bp_aabb is not None and col_aabb is not None:
        bp_dx = bp_aabb[1][0] - bp_aabb[0][0]
        col_dx = col_aabb[1][0] - col_aabb[0][0]
        ctx.check(
            "base plate is wider than column for bench stability",
            bp_dx > col_dx * 2.0,
            details=f"base_plate_dx={bp_dx:.4f}, column_dx={col_dx:.4f}",
        )

    # The base die sits on the base plate directly under the ram axis.
    ctx.expect_gap(
        body,
        body,
        axis="z",
        positive_elem="base_die",
        negative_elem="base_plate",
        max_penetration=0.001,
        max_gap=0.001,
        name="base_die is seated on the base plate",
    )

    # -- Pivot screw pin captured inside the lever boss bore ---------------
    ctx.allow_overlap(
        body,
        lever,
        elem_a="pivot_pin",
        elem_b="lever_boss",
        reason="Pivot screw pin is captured inside the lever boss hinge bore.",
    )

    # Lever handle emerges from the pivot boss seated between the ears on the
    # head; a small local overlap at the pivot-cap junction is structural.
    ctx.allow_overlap(
        body,
        lever,
        elem_a="column_cap",
        elem_b="lever_handle",
        reason="The lever handle emerges from the pivot boss on the head cap; small local overlap at the pivot junction is structurally intentional.",
    )
    # The lever boss is captured between the pivot ears on the saddle.
    ctx.allow_overlap(
        body,
        lever,
        elem_a="pivot_saddle",
        elem_b="lever_boss",
        reason="The lever boss rotates within the pivot saddle/ear clevis assembly; the boss is captured between the ears.",
    )
    # The lever handle emerges from the boss through the saddle area.
    ctx.allow_overlap(
        body,
        lever,
        elem_a="pivot_saddle",
        elem_b="lever_handle",
        reason="The lever handle starts at the pivot boss which is seated in the saddle clevis; local overlap at the pivot junction is structural.",
    )
    ctx.expect_overlap(
        lever,
        body,
        axes="xy",
        elem_a="lever_boss",
        elem_b="pivot_saddle",
        min_overlap=0.010,
        name="lever boss is positioned within the pivot saddle",
    )

    # Press pad seated on the ram top button with a tiny intentional embed.
    ctx.allow_overlap(
        lever,
        plunger,
        elem_a="press_pad",
        elem_b="top_button",
        reason="Press pad is seated on the ram top button; the lever drives the ram through this contact.",
    )

    # -- Ram centered in and retained by the steel guide sleeve ------------
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

    # -- Lever press pad seats on the plunger top button -------------------
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

    # -- Loaded snap cap aligned above the base die post with work gap -----
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

    # -- Return spring reaches the lever and is seated on the bracket ------
    ctx.expect_contact(
        lever,
        body,
        elem_a="lever_handle",
        elem_b="return_spring",
        contact_tol=0.003,
        name="return spring reaches the squeeze lever",
    )
    ctx.expect_contact(
        body,
        body,
        elem_a="return_spring",
        elem_b="spring_bracket",
        contact_tol=0.004,
        name="return spring is seated on the spring bracket",
    )

    rest_plunger = ctx.part_world_position(plunger)
    rest_lever_aabb = ctx.part_world_aabb(lever)

    # -- Full squeeze: lever drives the ram down --------------------------
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
        "squeezing swings the lever handle downward",
        rest_lever_aabb is not None
        and pressed_lever_aabb is not None
        and pressed_lever_aabb[0][2] < rest_lever_aabb[0][2] - 0.02,
        details=f"rest_min_z={rest_lever_aabb}, pressed_min_z={pressed_lever_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
