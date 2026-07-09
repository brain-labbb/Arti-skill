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
    TireGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _rounded_box(size: tuple[float, float, float], radius: float = 0.0) -> cq.Workplane:
    """Small CadQuery helper for molded plastic/metal panels."""
    sx, sy, sz = size
    shape = cq.Workplane("XY").box(sx, sy, sz)
    if radius > 0.0:
        # Vertical edge rounding gives consumer-plastic corners while keeping
        # top/bottom faces stable for seating and exact tests.
        shape = shape.edges("|Z").fillet(min(radius, sx * 0.35, sy * 0.35))
    return shape


def _body_shell() -> cq.Workplane:
    """Single connected shell: side/rear/bin frame plus the overhanging shredder head."""
    width = 0.42
    depth = 0.34
    lower_h = 0.49
    lower_z = 0.305

    lower = _rounded_box((width, depth, lower_h), 0.020).translate((0.0, 0.0, lower_z))

    # Front bin opening / basket cavity.  This cuts out a true recess rather
    # than painting a black rectangle on a solid block; a narrow rear wall and
    # side jambs remain to carry the shredder head.
    cavity = (
        cq.Workplane("XY")
        .box(0.322, 0.335, 0.405)
        .translate((0.0, 0.030, 0.302))
    )
    lower = lower.cut(cavity)

    # Top shredder head: a rounded black plastic module that slightly overhangs
    # the bin shell, like the photo reference.
    head = _rounded_box((0.44, 0.365, 0.120), 0.030).translate((0.0, -0.005, 0.570))
    body = lower.union(head)

    # A subtle front lower plinth gives the caster yokes a real mounting pad.
    plinth = _rounded_box((0.42, 0.080, 0.040), 0.012).translate((0.0, 0.130, 0.075))
    return body.union(plinth)


def _front_panel_with_window() -> cq.Workplane:
    """Basket front with a real rectangular viewing cutout."""
    panel = _rounded_box((0.345, 0.034, 0.370), 0.018)
    cutout = cq.Workplane("XY").box(0.105, 0.060, 0.055).translate((0.0, 0.0, -0.045))
    panel = panel.cut(cutout)
    # Small proud top lip like the curved pull surface on a removable basket.
    lip = _rounded_box((0.335, 0.046, 0.030), 0.012).translate((0.0, 0.003, 0.178))
    return panel.union(lip)


def _paper_fill_geometry() -> MeshGeometry:
    """A single connected crumpled-paper mass, not loose floating strips."""
    width = 0.265
    depth = 0.205
    base_z = 0.0
    nx = 8
    ny = 7
    geom = MeshGeometry()

    top_indices: list[list[int]] = []
    bottom_indices: list[list[int]] = []
    for ix in range(nx + 1):
        top_row = []
        bottom_row = []
        x = -width / 2.0 + width * ix / nx
        for iy in range(ny + 1):
            y = -depth / 2.0 + depth * iy / ny
            ripple = 0.010 * math.sin(ix * 1.7) + 0.007 * math.cos(iy * 2.3)
            ridge = 0.004 * math.sin((ix + iy) * 2.1)
            z = 0.145 + ripple + ridge
            top_row.append(geom.add_vertex(x, y, z))
            bottom_row.append(geom.add_vertex(x, y, base_z))
        top_indices.append(top_row)
        bottom_indices.append(bottom_row)

    for ix in range(nx):
        for iy in range(ny):
            a = top_indices[ix][iy]
            b = top_indices[ix + 1][iy]
            c = top_indices[ix + 1][iy + 1]
            d = top_indices[ix][iy + 1]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)

            # Bottom grid mirrors the top grid so side curtains share real
            # edges instead of only meeting at a corner vertex.
            ab = bottom_indices[ix][iy]
            bb = bottom_indices[ix + 1][iy]
            cb = bottom_indices[ix + 1][iy + 1]
            db = bottom_indices[ix][iy + 1]
            geom.add_face(ab, cb, bb)
            geom.add_face(ab, db, cb)

    # Vertical curtain faces connect the pile to its bottom rectangle so the
    # visible paper reads as one supported contents mass.
    for ix in range(nx):
        a = top_indices[ix][0]
        b = top_indices[ix + 1][0]
        bb = bottom_indices[ix + 1][0]
        ab = bottom_indices[ix][0]
        geom.add_face(ab, a, b)
        geom.add_face(ab, b, bb)

        c = top_indices[ix][ny]
        d = top_indices[ix + 1][ny]
        db = bottom_indices[ix + 1][ny]
        cb = bottom_indices[ix][ny]
        geom.add_face(cb, db, d)
        geom.add_face(cb, d, c)

    for iy in range(ny):
        a = top_indices[0][iy]
        b = top_indices[0][iy + 1]
        bb = bottom_indices[0][iy + 1]
        ab = bottom_indices[0][iy]
        geom.add_face(ab, bb, b)
        geom.add_face(ab, b, a)

        c = top_indices[nx][iy]
        d = top_indices[nx][iy + 1]
        db = bottom_indices[nx][iy + 1]
        cb = bottom_indices[nx][iy]
        geom.add_face(cb, c, d)
        geom.add_face(cb, d, db)

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="paper_shredder_with_sliding_bin",
        meta={
            "reference_note": (
                "Modeled as the image/category paper shredder: boxy black plastic "
                "bin, top shredder head, feed slot, controls, pulled waste basket, "
                "viewing window, and caster wheels."
            )
        },
    )

    black = model.material("matte_black_plastic", rgba=(0.005, 0.006, 0.006, 1.0))
    dark = model.material("charcoal_plastic", rgba=(0.035, 0.038, 0.036, 1.0))
    rubber = model.material("black_rubber", rgba=(0.001, 0.001, 0.001, 1.0))
    silver = model.material("brushed_silver", rgba=(0.72, 0.72, 0.68, 1.0))
    smoky = model.material("smoky_translucent_window", rgba=(0.08, 0.10, 0.11, 0.46))
    white = model.material("shredded_white_paper", rgba=(0.92, 0.93, 0.96, 1.0))
    button_mat = model.material("soft_black_buttons", rgba=(0.015, 0.016, 0.017, 1.0))
    hub_mat = model.material("dark_hub", rgba=(0.10, 0.10, 0.10, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell(), "body_shell", tolerance=0.0012),
        material=black,
        name="lower_shell",
    )

    # Basket slide guide rails inside the front opening.  They also make the
    # slider support mechanically legible instead of leaving a floating drawer.
    for side, (x, guide_name) in enumerate(((-0.166, "guide_0"), (0.166, "guide_1"))):
        body.visual(
            Box((0.012, 0.225, 0.018)),
            origin=Origin(xyz=(x, 0.010, 0.290)),
            material=dark,
            name=guide_name,
        )

    # Four caster yokes are fused to the bottom shell; the wheels themselves
    # are separate continuous joints.
    caster_positions = [
        (-0.160, 0.125, 0.030),
        (0.160, 0.125, 0.030),
        (-0.160, -0.125, 0.030),
        (0.160, -0.125, 0.030),
    ]
    for i, (x, y, z) in enumerate(caster_positions):
        for j, dx in enumerate((-0.020, 0.020)):
            body.visual(
                Box((0.006, 0.018, 0.038)),
                origin=Origin(xyz=(x + dx, y, z + 0.017)),
                material=black,
                name=f"caster_yoke_{i}_{j}",
            )
        body.visual(
            Box((0.050, 0.024, 0.010)),
            origin=Origin(xyz=(x, y, 0.062)),
            material=black,
            name=f"caster_mount_{i}",
        )
        body.visual(
            Cylinder(radius=0.006, length=0.054),
            origin=Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hub_mat,
            name=f"caster_axle_{i}",
        )

    # Silver top plate and dark feed slot on the shredder head.  The slot is
    # intentionally long, narrow, and near the rear half of the lid.
    top_plate = _rounded_box((0.285, 0.225, 0.008), 0.010).translate((0.0, -0.022, 0.634))
    body.visual(
        mesh_from_cadquery(top_plate, "silver_top_panel", tolerance=0.0008),
        material=silver,
        name="top_panel",
    )
    body.visual(
        Box((0.235, 0.020, 0.004)),
        origin=Origin(xyz=(0.0, -0.113, 0.640)),
        material=dark,
        name="feed_slot",
    )
    body.visual(
        Box((0.120, 0.038, 0.006)),
        origin=Origin(xyz=(-0.145, 0.070, 0.634)),
        material=dark,
        name="control_panel",
    )

    # Side viewing panel visible on the bin shell, matching the reference's
    # smoked vertical window on the side plastic.
    body.visual(
        Box((0.004, 0.058, 0.205)),
        origin=Origin(xyz=(-0.212, -0.030, 0.305)),
        material=smoky,
        name="side_window",
    )

    basket = model.part("basket")
    basket.visual(
        Box((0.292, 0.315, 0.026)),
        origin=Origin(xyz=(0.0, -0.050, 0.013)),
        material=dark,
        name="basket_bottom",
    )
    for side, (x, side_name, runner_name) in enumerate(
        ((-0.151, "basket_side_0", "runner_0"), (0.151, "basket_side_1", "runner_1"))
    ):
        basket.visual(
            Box((0.018, 0.315, 0.345)),
            origin=Origin(xyz=(x, -0.050, 0.173)),
            material=dark,
            name=side_name,
        )
        basket.visual(
            Box((0.012, 0.225, 0.018)),
            origin=Origin(xyz=(x + (-0.003 if x < 0.0 else 0.003), -0.040, 0.175)),
            material=dark,
            name=runner_name,
        )
    basket.visual(
        Box((0.292, 0.018, 0.325)),
        origin=Origin(xyz=(0.0, -0.205, 0.163)),
        material=dark,
        name="basket_back",
    )
    basket.visual(
        mesh_from_cadquery(_front_panel_with_window(), "basket_front_panel", tolerance=0.001),
        origin=Origin(xyz=(0.0, 0.112, 0.190)),
        material=black,
        name="front_panel",
    )
    basket.visual(
        Box((0.126, 0.010, 0.064)),
        origin=Origin(xyz=(0.0, 0.132, 0.145)),
        material=smoky,
        name="front_window",
    )
    basket.visual(
        Box((0.130, 0.012, 0.024)),
        origin=Origin(xyz=(0.0, 0.132, 0.322)),
        material=dark,
        name="pull_handle_recess",
    )
    basket.visual(
        mesh_from_geometry(_paper_fill_geometry(), "paper_fill"),
        origin=Origin(xyz=(0.0, -0.035, 0.024)),
        material=white,
        name="paper_fill",
    )

    model.articulation(
        "body_to_basket",
        ArticulationType.PRISMATIC,
        parent=body,
        child=basket,
        # q=0 is the reference-image pose: the basket is partially pulled out.
        # Negative q closes it flush; positive q withdraws it while retaining
        # side runners inside the guide rails.
        origin=Origin(xyz=(0.0, 0.100, 0.115)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.35, lower=-0.070, upper=0.120),
    )

    # Distinct user controls: five push buttons and a small sliding mode switch
    # on the top-left panel.  Buttons are evenly spaced across the control_panel
    # x extent (center -0.145, half-width 0.049, margin 0.007 from each edge).
    for i, x in enumerate((-0.187, -0.166, -0.145, -0.124, -0.103)):
        button = model.part(f"button_{i}")
        button.visual(
            Cylinder(radius=0.0075, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
            material=button_mat,
            name="button_cap",
        )
        model.articulation(
            f"body_to_button_{i}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x, 0.071, 0.636)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.05, lower=0.0, upper=0.004),
        )

    switch = model.part("mode_switch")
    switch.visual(
        Box((0.019, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=button_mat,
        name="switch_slider",
    )
    model.articulation(
        "body_to_mode_switch",
        ArticulationType.PRISMATIC,
        parent=body,
        child=switch,
        origin=Origin(xyz=(-0.145, 0.092, 0.638)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=0.08, lower=-0.011, upper=0.011),
    )

    for i, (x, y, z) in enumerate(caster_positions):
        wheel = model.part(f"caster_wheel_{i}")
        wheel.visual(
            mesh_from_geometry(TireGeometry(0.024, 0.024, inner_radius=0.013), f"caster_tire_{i}"),
            material=rubber,
            name="rubber_tire",
        )
        wheel.visual(
            Cylinder(radius=0.014, length=0.027),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hub_mat,
            name="hub",
        )
        model.articulation(
            f"body_to_caster_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(x, y, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.5, velocity=8.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    basket = object_model.get_part("basket")
    slide = object_model.get_articulation("body_to_basket")

    ctx.expect_contact(
        basket,
        body,
        elem_a="runner_0",
        elem_b="guide_0",
        contact_tol=0.0015,
        name="basket runner contacts guide rail",
    )
    ctx.expect_overlap(
        basket,
        body,
        axes="y",
        elem_a="runner_0",
        elem_b="guide_0",
        min_overlap=0.050,
        name="basket remains retained on guide rail",
    )

    rest_pos = ctx.part_world_position(basket)
    rest_aabb = ctx.part_world_aabb(basket)
    body_aabb = ctx.part_world_aabb(body)
    with ctx.pose({slide: 0.120}):
        extended_pos = ctx.part_world_position(basket)
        extended_aabb = ctx.part_world_aabb(basket)
        ctx.expect_overlap(
            basket,
            body,
            axes="y",
            elem_a="runner_0",
            elem_b="guide_0",
            min_overlap=0.020,
            name="extended basket still has retained rail insertion",
        )
    with ctx.pose({slide: -0.070}):
        closed_aabb = ctx.part_world_aabb(basket)

    ctx.check(
        "basket slides outward along front axis",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[1] > rest_pos[1] + 0.10,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )
    ctx.check(
        "closed basket front seats near shredder face",
        body_aabb is not None
        and closed_aabb is not None
        and closed_aabb[1][1] <= body_aabb[1][1] + 0.010,
        details=f"body_aabb={body_aabb}, closed_basket_aabb={closed_aabb}",
    )
    ctx.check(
        "reference pose shows pulled basket and front opening",
        body_aabb is not None
        and rest_aabb is not None
        and rest_aabb[1][1] > body_aabb[1][1] + 0.045,
        details=f"body_aabb={body_aabb}, basket_aabb={rest_aabb}",
    )

    button_joint = object_model.get_articulation("body_to_button_1")
    button = object_model.get_part("button_1")
    button_rest = ctx.part_world_position(button)
    with ctx.pose({button_joint: 0.004}):
        button_pressed = ctx.part_world_position(button)
    ctx.check(
        "push button depresses downward",
        button_rest is not None
        and button_pressed is not None
        and button_pressed[2] < button_rest[2] - 0.003,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )

    # Verify the 5-button control array: buttons 0..4 all exist with press joints
    for btn_idx in range(5):
        btn_part = object_model.get_part(f"button_{btn_idx}")
        btn_joint = object_model.get_articulation(f"body_to_button_{btn_idx}")
        btn_rest_pos = ctx.part_world_position(btn_part)
        with ctx.pose({btn_joint: 0.004}):
            btn_pressed_pos = ctx.part_world_position(btn_part)
        ctx.check(
            f"button_{btn_idx} has prismatic press joint and depresses",
            btn_joint.articulation_type == ArticulationType.PRISMATIC
            and tuple(btn_joint.axis) == (0.0, 0.0, -1.0)
            and btn_rest_pos is not None
            and btn_pressed_pos is not None
            and btn_pressed_pos[2] < btn_rest_pos[2] - 0.003,
            details=f"type={btn_joint.articulation_type}, axis={btn_joint.axis}, rest={btn_rest_pos}, pressed={btn_pressed_pos}",
        )

    switch_joint = object_model.get_articulation("body_to_mode_switch")
    switch = object_model.get_part("mode_switch")
    with ctx.pose({switch_joint: -0.011}):
        left_pos = ctx.part_world_position(switch)
    with ctx.pose({switch_joint: 0.011}):
        right_pos = ctx.part_world_position(switch)
    ctx.check(
        "mode switch slides across control panel",
        left_pos is not None and right_pos is not None and right_pos[0] > left_pos[0] + 0.020,
        details=f"left={left_pos}, right={right_pos}",
    )

    for i, wheel_name, axle_name in (
        (0, "caster_wheel_0", "caster_axle_0"),
        (1, "caster_wheel_1", "caster_axle_1"),
        (2, "caster_wheel_2", "caster_axle_2"),
        (3, "caster_wheel_3", "caster_axle_3"),
    ):
        wheel_joint = object_model.get_articulation(f"body_to_caster_wheel_{i}")
        wheel = object_model.get_part(wheel_name)
        ctx.allow_overlap(
            body,
            wheel,
            elem_a=axle_name,
            elem_b="hub",
            reason=(
                "Each caster hub intentionally rotates around a fixed axle pin; "
                "the pin is represented as captured inside the wheel hub."
            ),
        )
        ctx.expect_overlap(
            body,
            wheel,
            axes="x",
            elem_a=axle_name,
            elem_b="hub",
            min_overlap=0.020,
            name=f"caster wheel {i} hub is captured on axle",
        )
        ctx.expect_contact(
            body,
            wheel,
            elem_a=f"caster_yoke_{i}_0",
            elem_b="rubber_tire",
            contact_tol=0.006,
            name=f"caster wheel {i} sits between fork cheeks",
        )
        ctx.check(
            f"caster wheel {i} has rolling joint",
            wheel_joint.articulation_type == ArticulationType.CONTINUOUS
            and tuple(wheel_joint.axis) == (1.0, 0.0, 0.0),
            details=f"type={wheel_joint.articulation_type}, axis={wheel_joint.axis}",
        )

    return ctx.report()


object_model = build_object_model()
