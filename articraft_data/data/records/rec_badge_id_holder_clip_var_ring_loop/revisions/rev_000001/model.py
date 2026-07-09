from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _rounded_plate(
    length: float,
    width: float,
    thickness: float,
    *,
    radius: float = 0.002,
    round_vertical_edges: bool = True,
    round_holes: tuple[tuple[float, float, float], ...] = (),
    slots: tuple[tuple[float, float, float, float], ...] = (),
    rect_holes: tuple[tuple[float, float, float, float], ...] = (),
) -> cq.Workplane:
    """Thin XY plate extruded upward from z=0 with optional through holes."""

    wp = cq.Workplane("XY").rect(length, width).extrude(thickness)
    if round_vertical_edges and radius > 0:
        wp = wp.edges("|Z").fillet(min(radius, width * 0.45, length * 0.45))

    if round_holes:
        wp = (
            wp.faces(">Z")
            .workplane(centerOption="CenterOfBoundBox")
            .pushPoints([(x, y) for x, y, _r in round_holes])
        )
        for x, y, r in round_holes:
            cutter = (
                cq.Workplane("XY")
                .center(x, y)
                .circle(r)
                .extrude(thickness * 4.0)
                .translate((0.0, 0.0, -thickness * 1.5))
            )
            wp = wp.cut(cutter)

    for x, y, slot_len, slot_dia in slots:
        cutter = (
            cq.Workplane("XY")
            .center(x, y)
            .slot2D(slot_len, slot_dia, 0)
            .extrude(thickness * 4.0)
            .translate((0.0, 0.0, -thickness * 1.5))
        )
        wp = wp.cut(cutter)

    for x, y, sx, sy in rect_holes:
        cutter = (
            cq.Workplane("XY")
            .center(x, y)
            .rect(sx, sy)
            .extrude(thickness * 4.0)
            .translate((0.0, 0.0, -thickness * 1.5))
        )
        wp = wp.cut(cutter)

    return wp


def _tapered_plate(
    x0: float,
    x1: float,
    width0: float,
    width1: float,
    thickness: float,
    *,
    radius: float = 0.0015,
    round_holes: tuple[tuple[float, float, float], ...] = (),
) -> cq.Workplane:
    pts = [
        (x0, -width0 / 2.0),
        (x0, width0 / 2.0),
        (x1, width1 / 2.0),
        (x1, -width1 / 2.0),
    ]
    wp = cq.Workplane("XY").polyline(pts).close().extrude(thickness)
    if radius > 0:
        wp = wp.edges("|Z").fillet(radius)
    for x, y, r in round_holes:
        cutter = (
            cq.Workplane("XY")
            .center(x, y)
            .circle(r)
            .extrude(thickness * 4.0)
            .translate((0.0, 0.0, -thickness * 1.5))
        )
        wp = wp.cut(cutter)
    return wp


def _ring(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
    )


def _tooth_bar(
    count: int,
    pitch: float,
    tooth_width: float,
    bar_width: float,
    height: float,
) -> cq.Workplane:
    """One connected row of small rectangular serrations on a thin back strip."""

    total = (count - 1) * pitch + tooth_width
    wp = cq.Workplane("XY").rect(total + 0.001, bar_width).extrude(height * 0.35)
    for i in range(count):
        y = -total / 2.0 + tooth_width / 2.0 + i * pitch
        tooth = (
            cq.Workplane("XY")
            .center(0.0, y)
            .rect(0.0032, tooth_width)
            .extrude(height)
        )
        wp = wp.union(tooth)
    return wp


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="badge_id_holder_clip",
        meta={
            "reference_note": "Modeled as a single badge/ID holder clip: stamped metal alligator jaw with clear swivel connector.",
        },
    )

    chrome = model.material("polished_chrome", rgba=(0.82, 0.82, 0.78, 1.0))
    dark_chrome = model.material("shadowed_chrome", rgba=(0.30, 0.31, 0.31, 1.0))
    clear_vinyl = model.material("clear_translucent_vinyl", rgba=(0.78, 0.92, 1.0, 0.38))
    hole_shadow = model.material("dark_hole_shadow", rgba=(0.02, 0.025, 0.03, 1.0))

    body = model.part("clip_body")

    lower_plate = _rounded_plate(
        0.120,
        0.030,
        0.0012,
        radius=0.004,
        round_holes=((0.025, 0.0, 0.0028),),
    )
    body.visual(
        mesh_from_cadquery(lower_plate, "lower_stamped_plate", tolerance=0.0003),
        origin=Origin(xyz=(0.005, 0.0, 0.0022)),
        material=chrome,
        name="lower_stamped_plate",
    )

    body.visual(
        Box((0.030, 0.024, 0.0020)),
        origin=Origin(xyz=(-0.053, 0.0, 0.00435), rpy=(0.0, -0.10, 0.0)),
        material=chrome,
        name="lower_jaw_lip",
    )
    body.visual(
        mesh_from_cadquery(_tooth_bar(6, 0.0037, 0.0025, 0.019, 0.0015), "lower_jaw_teeth"),
        origin=Origin(xyz=(-0.064, 0.0, 0.0061), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_chrome,
        name="lower_jaw_teeth",
    )

    # Folded side cheeks: two stamped side walls carry the hinge pin like the
    # alligator clip yoke in the reference image.
    for y, suffix in ((-0.0136, "0"), (0.0136, "1")):
        body.visual(
            Box((0.030, 0.0028, 0.016)),
            origin=Origin(xyz=(-0.022, y, 0.0092)),
            material=chrome,
            name=f"side_cheek_{suffix}",
        )
        body.visual(
            Cylinder(radius=0.0030, length=0.0007),
            origin=Origin(xyz=(-0.022, y * 1.01, 0.0102), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hole_shadow,
            name=f"cheek_hole_{suffix}",
        )

    body.visual(
        Cylinder(radius=0.0020, length=0.033),
        origin=Origin(xyz=(-0.022, 0.0, 0.0102), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_chrome,
        name="hinge_pin",
    )
    body.visual(
        mesh_from_geometry(TorusGeometry(0.0043, 0.0022, radial_segments=28, tubular_segments=8), "torsion_spring"),
        origin=Origin(xyz=(-0.022, -0.0092, 0.0102), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_chrome,
        name="torsion_spring",
    )

    body.visual(
        Cylinder(radius=0.0031, length=0.0110),
        origin=Origin(xyz=(0.045, 0.0, 0.0085)),
        material=dark_chrome,
        name="swivel_post",
    )
    body.visual(
        Cylinder(radius=0.0066, length=0.0009),
        origin=Origin(xyz=(0.045, 0.0, 0.0136)),
        material=chrome,
        name="swivel_boss",
    )

    jaw = model.part("spring_jaw")
    jaw.visual(
        Cylinder(radius=0.0034, length=0.0175),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="hinge_barrel",
    )
    jaw.visual(
        Box((0.038, 0.018, 0.0100)),
        origin=Origin(xyz=(-0.010, 0.0, 0.0065)),
        material=chrome,
        name="barrel_wrap",
    )
    upper_plate = _tapered_plate(
        -0.038,
        0.085,
        0.020,
        0.026,
        0.0011,
        radius=0.0025,
        round_holes=((0.043, 0.0, 0.0032),),
    )
    jaw.visual(
        mesh_from_cadquery(upper_plate, "upper_spring_plate", tolerance=0.0003),
        origin=Origin(xyz=(0.002, 0.0, 0.0105), rpy=(0.0, -0.09, 0.0)),
        material=chrome,
        name="upper_spring_plate",
    )
    jaw.visual(
        Cylinder(radius=0.0048, length=0.0012),
        origin=Origin(xyz=(0.045, 0.0, 0.0152)),
        material=dark_chrome,
        name="pressed_rivet",
    )
    jaw.visual(
        Box((0.024, 0.020, 0.0016)),
        origin=Origin(xyz=(-0.049, 0.0, -0.0040), rpy=(0.0, 0.12, 0.0)),
        material=chrome,
        name="upper_front_blade",
    )
    jaw.visual(
        Box((0.016, 0.016, 0.0130)),
        origin=Origin(xyz=(-0.041, 0.0, 0.0025)),
        material=chrome,
        name="front_fold",
    )
    jaw.visual(
        mesh_from_cadquery(_tooth_bar(5, 0.0037, 0.0024, 0.017, 0.0014), "upper_front_teeth"),
        origin=Origin(xyz=(-0.060, 0.0, -0.0056), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_chrome,
        name="upper_front_teeth",
    )

    connector = model.part("badge_connector")
    # Swivel hub (kept from parent: button ring + window seat on the boss)
    connector.visual(
        mesh_from_cadquery(_ring(0.0076, 0.0046, 0.0013), "swivel_button_ring", tolerance=0.0002),
        origin=Origin(xyz=(0.0, 0.0, -0.00015)),
        material=chrome,
        name="swivel_button_ring",
    )
    connector.visual(
        Cylinder(radius=0.0048, length=0.0010),
        origin=Origin(xyz=(0.0, 0.0, 0.0010)),
        material=clear_vinyl,
        name="button_window",
    )
    # Neck shank: short vertical post rising from the swivel hub to the ring
    connector.visual(
        Cylinder(radius=0.0018, length=0.018),
        origin=Origin(xyz=(0.005, 0.0, 0.008)),
        material=chrome,
        name="neck_shank",
    )
    # Ring loop: torus standing in the XZ plane so a lanyard threads along Y
    connector.visual(
        mesh_from_geometry(
            TorusGeometry(0.010, 0.0015, radial_segments=24, tubular_segments=36),
            "ring_loop",
        ),
        origin=Origin(xyz=(0.015, 0.0, 0.017), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="ring_loop",
    )

    model.articulation(
        "body_to_jaw",
        ArticulationType.REVOLUTE,
        parent=body,
        child=jaw,
        origin=Origin(xyz=(-0.022, 0.0, 0.0102)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0, lower=0.0, upper=0.45),
    )
    model.articulation(
        "body_to_connector",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=connector,
        origin=Origin(xyz=(0.045, 0.0, 0.0140)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("clip_body")
    jaw = object_model.get_part("spring_jaw")
    connector = object_model.get_part("badge_connector")
    jaw_hinge = object_model.get_articulation("body_to_jaw")
    swivel = object_model.get_articulation("body_to_connector")

    ctx.allow_overlap(
        body,
        jaw,
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        reason="The hinge pin is intentionally captured inside the moving barrel.",
    )
    ctx.allow_overlap(
        body,
        connector,
        elem_a="swivel_boss",
        elem_b="swivel_button_ring",
        reason="The swivel button ring is intentionally seated with a tiny crimped overlap on the boss.",
    )
    ctx.allow_overlap(
        body,
        connector,
        elem_a="swivel_boss",
        elem_b="neck_shank",
        reason="The neck shank passes through the swivel boss crimp zone to connect the ring loop to the button hub.",
    )
    ctx.expect_overlap(
        jaw,
        body,
        axes="y",
        elem_a="hinge_barrel",
        elem_b="hinge_pin",
        min_overlap=0.015,
        name="hinge barrel is retained on pin across clip width",
    )
    ctx.expect_contact(
        connector,
        body,
        elem_a="swivel_button_ring",
        elem_b="swivel_boss",
        contact_tol=0.0010,
        name="swivel button is seated on the boss",
    )
    ctx.expect_gap(
        connector,
        body,
        axis="z",
        positive_elem="swivel_button_ring",
        negative_elem="swivel_boss",
        max_gap=0.0002,
        max_penetration=0.0004,
        name="swivel crimp has only local seated penetration",
    )
    ctx.expect_overlap(
        connector,
        body,
        axes="xy",
        elem_a="swivel_button_ring",
        elem_b="swivel_boss",
        min_overlap=0.006,
        name="swivel ring is centered over the rivet boss",
    )

    rest_teeth = ctx.part_element_world_aabb(jaw, elem="upper_front_teeth")
    with ctx.pose({jaw_hinge: 0.35}):
        open_teeth = ctx.part_element_world_aabb(jaw, elem="upper_front_teeth")
    ctx.check(
        "positive hinge pose opens the jaw teeth",
        rest_teeth is not None
        and open_teeth is not None
        and open_teeth[1][2] > rest_teeth[1][2] + 0.010,
        details=f"rest={rest_teeth}, opened={open_teeth}",
    )

    # Ring loop geometry assertion: the torus must be tall and wide (XZ plane)
    # but thin in Y, confirming it reads as a ring rather than a flat strap.
    rest_ring = ctx.part_element_world_aabb(connector, elem="ring_loop")
    ctx.check(
        "ring_loop is a torus standing in the XZ plane",
        rest_ring is not None
        and (rest_ring[1][0] - rest_ring[0][0]) > 0.015  # dx > 15 mm
        and (rest_ring[1][2] - rest_ring[0][2]) > 0.015  # dz > 15 mm
        and (rest_ring[1][1] - rest_ring[0][1]) < 0.006,  # dy < 6 mm (thin tube)
        details=f"ring_aabb={rest_ring}",
    )

    # Swivel rotation: after 90° about Z, the ring plane swaps from X-wide to Y-wide.
    with ctx.pose({swivel: math.pi / 2.0}):
        turned_ring = ctx.part_element_world_aabb(connector, elem="ring_loop")
    if rest_ring is not None and turned_ring is not None:
        rest_dx = rest_ring[1][0] - rest_ring[0][0]
        rest_dy = rest_ring[1][1] - rest_ring[0][1]
        turn_dx = turned_ring[1][0] - turned_ring[0][0]
        turn_dy = turned_ring[1][1] - turned_ring[0][1]
    else:
        rest_dx = rest_dy = turn_dx = turn_dy = 0.0
    ctx.check(
        "badge connector ring swivels ninety degrees about the button",
        rest_dx > rest_dy * 2.0 and turn_dy > turn_dx * 2.0,
        details=f"rest_dx={rest_dx}, rest_dy={rest_dy}, turn_dx={turn_dx}, turn_dy={turn_dy}",
    )

    # Prove the neck shank retains contact with the swivel hub
    ctx.expect_overlap(
        connector,
        body,
        axes="xy",
        elem_a="neck_shank",
        elem_b="swivel_boss",
        min_overlap=0.001,
        name="neck shank remains within the swivel boss footprint",
    )

    return ctx.report()


object_model = build_object_model()
