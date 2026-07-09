from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Snap fastener hand punch setter (two-piece anvil + coaxial setting punch).
#
# Frame: +Z up, origin at anvil base center bottom (sits on work surface).
# Root part `body` = stout anvil base disc + guide tube + bushing + die stack.
# `plunger` = setting punch rod that slides down through the guide to press
# a snap cap onto the base die socket.
# ---------------------------------------------------------------------------

GUIDE_TOP_Z = 0.042   # joint origin z (top of guide bushing, world)
RAM_TRAVEL = 0.0025   # m, punch press stroke


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="snap_fastener_hand_punch")

    steel = model.material("zinc_steel", rgba=(0.72, 0.73, 0.75, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.38, 0.39, 0.42, 1.0))
    nylon = model.material("white_nylon", rgba=(0.94, 0.94, 0.91, 1.0))
    brass = model.material("brass", rgba=(0.79, 0.64, 0.31, 1.0))

    # ------------------------------------------------------------------ body
    # Anvil base: stout disc with chamfered edges, sits on the work surface.
    body = model.part("body")

    base_profile = [
        (0.0, 0.0),
        (0.017, 0.0),
        (0.019, 0.002),
        (0.019, 0.010),
        (0.017, 0.012),
        (0.0, 0.012),
    ]
    anvil_disc = LatheGeometry(base_profile, segments=32)
    body.visual(
        mesh_from_geometry(anvil_disc, "anvil_base"),
        material=steel,
        name="anvil_base",
    )

    # Guide tube: hollow cylinder rising from the base, wide bore for die and
    # punch feature clearance.
    guide_tube = LatheGeometry.from_shell_profiles(
        [(0.010, 0.0), (0.010, 0.027)],
        [(0.008, 0.0), (0.008, 0.027)],
        segments=32,
    )
    body.visual(
        mesh_from_geometry(guide_tube, "guide_tube"),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=steel,
        name="guide_tube",
    )

    # Guide bushing: narrower bore at the top of the tube for shaft guidance.
    guide_bushing = LatheGeometry.from_shell_profiles(
        [(0.010, 0.0), (0.010, 0.003)],
        [(0.004, 0.0), (0.004, 0.003)],
        segments=32,
    )
    body.visual(
        mesh_from_geometry(guide_bushing, "guide_bushing"),
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
        material=dark_steel,
        name="guide_bushing",
    )

    # Base die seated on the anvil base inside the guide tube bore.
    body.visual(
        Cylinder(radius=0.006, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0145)),
        material=steel,
        name="base_die",
    )

    # Die post rising from the base die.
    body.visual(
        Cylinder(radius=0.0028, length=0.0055),
        origin=Origin(xyz=(0.0, 0.0, 0.01975)),
        material=steel,
        name="die_post",
    )

    # Snap socket ring seated on the base die around the die post.
    socket_ring = LatheGeometry.from_shell_profiles(
        [(0.0056, 0.0), (0.0056, 0.002)],
        [(0.0034, 0.0), (0.0034, 0.002)],
        segments=32,
    )
    body.visual(
        mesh_from_geometry(socket_ring, "snap_socket"),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=brass,
        name="snap_socket",
    )

    # --------------------------------------------------------------- plunger
    # Setting punch: ram shaft slides through the guide bushing bore; nylon
    # tip, cap die, and snap cap hang below the shaft inside the tube bore.
    # Joint frame at the bushing top; +axis (0,0,-1) presses downward.
    plunger = model.part("plunger")

    # Ram shaft: steel rod through the bushing, extends above for hand grip.
    plunger.visual(
        Cylinder(radius=0.0035, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=steel,
        name="ram_shaft",
    )

    # White nylon pressure tip below the shaft.
    plunger.visual(
        Cylinder(radius=0.0058, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=nylon,
        name="nylon_tip",
    )

    # Cap die disc below the nylon tip.
    plunger.visual(
        Cylinder(radius=0.0052, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.013)),
        material=steel,
        name="cap_die",
    )

    # Snap cap loaded on the punch, ready to press onto the socket.
    plunger.visual(
        Cylinder(radius=0.0056, length=0.0015),
        origin=Origin(xyz=(0.0, 0.0, -0.01575)),
        material=brass,
        name="snap_cap",
    )

    model.articulation(
        "body_to_plunger",
        ArticulationType.PRISMATIC,
        parent=body,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, GUIDE_TOP_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=200.0, velocity=0.05, lower=0.0, upper=RAM_TRAVEL
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    plunger = object_model.get_part("plunger")
    press = object_model.get_articulation("body_to_plunger")

    # Verify the two-piece hand punch topology: body_to_plunger is the only
    # articulation and it is prismatic (no lever, no revolute pivot).
    all_joints = list(object_model.articulations)
    ctx.check(
        "body_to_plunger is the only articulation",
        len(all_joints) == 1 and all_joints[0].name == "body_to_plunger",
        details=f"found {[j.name for j in all_joints]}",
    )
    ctx.check(
        "body_to_plunger is prismatic (punch slides into anvil)",
        press.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={press.articulation_type}",
    )

    # Ram shaft is centered in the guide bushing bore.
    ctx.expect_within(
        plunger,
        body,
        axes="xy",
        inner_elem="ram_shaft",
        outer_elem="guide_bushing",
        margin=0.001,
        name="ram shaft stays centered in the guide bushing",
    )

    # Ram shaft remains inserted in the guide at rest.
    ctx.expect_overlap(
        plunger,
        body,
        axes="z",
        elem_a="ram_shaft",
        elem_b="guide_bushing",
        min_overlap=0.002,
        name="ram shaft remains inserted in the guide bushing at rest",
    )

    # Snap cap is coaxially aligned over the die post.
    ctx.expect_overlap(
        plunger,
        body,
        axes="xy",
        elem_a="snap_cap",
        elem_b="die_post",
        min_overlap=0.004,
        name="snap cap is aligned over the die post",
    )

    # Open work gap between snap cap and die post at rest.
    ctx.expect_gap(
        plunger,
        body,
        axis="z",
        positive_elem="snap_cap",
        negative_elem="die_post",
        min_gap=0.002,
        max_gap=0.006,
        name="open work gap between snap cap and die post at rest",
    )

    rest_plunger = ctx.part_world_position(plunger)

    # Full press: punch drives down, snap cap closes onto the die post.
    with ctx.pose({press: RAM_TRAVEL}):
        ctx.expect_gap(
            plunger,
            body,
            axis="z",
            positive_elem="snap_cap",
            negative_elem="die_post",
            min_gap=0.0,
            max_gap=0.001,
            name="full press closes the snap cap onto the die post",
        )
        ctx.expect_overlap(
            plunger,
            body,
            axes="z",
            elem_a="ram_shaft",
            elem_b="guide_bushing",
            min_overlap=0.001,
            name="ram shaft retains insertion in the guide at full press",
        )
        pressed_plunger = ctx.part_world_position(plunger)

    ctx.check(
        "press drives the punch downward into the anvil",
        rest_plunger is not None
        and pressed_plunger is not None
        and pressed_plunger[2] < rest_plunger[2] - 0.001,
        details=f"rest={rest_plunger}, pressed={pressed_plunger}",
    )

    return ctx.report()


object_model = build_object_model()
