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
    mesh_from_cadquery,
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


def _ring(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="badge_id_holder_clip",
        meta={
            "reference_note": (
                "Magnetic badge holder: flat nickel front carrier plate with "
                "swivel boss, and a separate rear magnet backer joined by a "
                "prismatic clamp joint that grips fabric between the two plates."
            ),
        },
    )

    nickel = model.material("nickel_plate", rgba=(0.78, 0.78, 0.75, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.22, 0.23, 0.24, 1.0))
    black_magnet = model.material("black_magnet", rgba=(0.10, 0.10, 0.12, 1.0))
    clear_vinyl = model.material("clear_translucent_vinyl", rgba=(0.78, 0.92, 1.0, 0.38))

    # ------------------------------------------------------------------
    # clip_body: flat rounded front carrier plate with swivel features
    # ------------------------------------------------------------------
    body = model.part("clip_body")

    # Front plate: 80mm x 28mm x 1.5mm, z from 0 to 0.0015
    front_plate = _rounded_plate(
        0.080,
        0.028,
        0.0015,
        radius=0.004,
    )
    body.visual(
        mesh_from_cadquery(front_plate, "front_plate", tolerance=0.0003),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=nickel,
        name="front_plate",
    )

    # Small alignment dimple on the back face (cosmetic, suggests magnetic target)
    body.visual(
        Cylinder(radius=0.005, length=0.0004),
        origin=Origin(xyz=(0.0, 0.0, -0.0002)),
        material=dark_metal,
        name="alignment_dimple",
    )

    # Swivel post: stands proud of the front face for badge attachment
    body.visual(
        Cylinder(radius=0.0031, length=0.011),
        origin=Origin(xyz=(0.030, 0.0, 0.0015 + 0.0055)),
        material=dark_metal,
        name="swivel_post",
    )
    # Swivel boss: wider flange at the top of the post (crimped button seat)
    body.visual(
        Cylinder(radius=0.0066, length=0.0009),
        origin=Origin(xyz=(0.030, 0.0, 0.0125 + 0.00045)),
        material=nickel,
        name="swivel_boss",
    )

    # ------------------------------------------------------------------
    # magnet_backer: rear carrier with two magnet poles
    # ------------------------------------------------------------------
    backer = model.part("magnet_backer")

    # Backer plate: 55mm x 18mm x 2mm, positioned behind front plate
    # _rounded_plate extrudes from z=0 to z=0.002; visual origin shifts to z=-0.004
    # so the plate spans z in [-0.004, -0.002]
    backer_plate = _rounded_plate(
        0.055,
        0.018,
        0.002,
        radius=0.003,
    )
    backer.visual(
        mesh_from_cadquery(backer_plate, "backer_plate", tolerance=0.0003),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=nickel,
        name="backer_plate",
    )

    # Two cylindrical magnet poles seated on the front face of the backer plate.
    # Backer plate top face is at z=-0.002. Each magnet is r=0.004, length=0.002,
    # center at z=-0.001, so the front face extends to z=0 where it contacts the
    # front plate back face (magnetic attraction through fabric in real use).
    for i, x_pos in enumerate((-0.015, 0.015)):
        backer.visual(
            Cylinder(radius=0.004, length=0.002),
            origin=Origin(xyz=(x_pos, 0.0, -0.001)),
            material=black_magnet,
            name=f"magnet_pole_{i}",
        )

    # ------------------------------------------------------------------
    # badge_connector: perforated clear strap with swivel button ring
    # (preserved from parent baseline)
    # ------------------------------------------------------------------
    connector = model.part("badge_connector")

    grid_holes = tuple(
        (x, y, 0.0016, 0.0028)
        for x in (0.012, 0.019, 0.026, 0.033)
        for y in (-0.006, 0.0, 0.006)
    )
    connector_panel = _rounded_plate(
        0.115,
        0.024,
        0.0012,
        radius=0.007,
        round_holes=((0.0, 0.0, 0.0045),),
        slots=((0.064, 0.0, 0.029, 0.0065),),
        rect_holes=grid_holes,
    )
    connector.visual(
        mesh_from_cadquery(connector_panel, "perforated_clear_strap", tolerance=0.00025),
        origin=Origin(xyz=(0.034, 0.0, 0.0)),
        material=clear_vinyl,
        name="perforated_clear_strap",
    )
    connector.visual(
        mesh_from_cadquery(_ring(0.0076, 0.0046, 0.0013), "swivel_button_ring", tolerance=0.0002),
        origin=Origin(xyz=(0.0, 0.0, -0.00015)),
        material=nickel,
        name="swivel_button_ring",
    )
    connector.visual(
        Cylinder(radius=0.0048, length=0.0010),
        origin=Origin(xyz=(0.0, 0.0, 0.0010)),
        material=clear_vinyl,
        name="button_window",
    )

    # ------------------------------------------------------------------
    # Articulations
    # ------------------------------------------------------------------

    # Prismatic clamp joint: axis along -z so positive q opens the clamp
    # (moves the magnet backer away from the front plate).
    # At q=0 (closed), backer is at the joint frame with ~0.5mm fabric gap.
    # At q=upper=0.012, backer has moved 12mm away to allow fabric insertion.
    model.articulation(
        "body_to_backer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=backer,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=0.15, lower=0.0, upper=0.012),
    )

    # Continuous swivel: badge connector rotates freely about z on the boss.
    # Joint origin at the top of the swivel boss.
    model.articulation(
        "body_to_connector",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=connector,
        origin=Origin(xyz=(0.030, 0.0, 0.01295)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("clip_body")
    backer = object_model.get_part("magnet_backer")
    connector = object_model.get_part("badge_connector")
    clamp = object_model.get_articulation("body_to_backer")
    swivel = object_model.get_articulation("body_to_connector")

    # --- Swivel seating (preserved from parent) ---
    ctx.allow_overlap(
        body,
        connector,
        elem_a="swivel_boss",
        elem_b="swivel_button_ring",
        reason="The swivel button ring is intentionally seated with a tiny crimped overlap on the boss.",
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
        max_penetration=0.001,
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

    # --- Prismatic magnetic clamp joint (this variant's primary change) ---
    # Prove the magnet backer contacts the front plate at rest (closed clamp).
    ctx.expect_contact(
        backer,
        body,
        elem_a="magnet_pole_0",
        elem_b="front_plate",
        contact_tol=0.0005,
        name="closed clamp: magnet pole contacts front plate back face",
    )

    # Prove the clamp opens by moving the backer away from the front plate.
    rest_backer_pos = ctx.part_world_position(backer)
    with ctx.pose({clamp: 0.010}):
        open_backer_pos = ctx.part_world_position(backer)
    ctx.check(
        "body_to_backer prismatic opens by moving magnet backer away from front plate",
        rest_backer_pos is not None
        and open_backer_pos is not None
        and open_backer_pos[2] < rest_backer_pos[2] - 0.008,
        details=f"rest_z={rest_backer_pos}, open_z={open_backer_pos}",
    )

    # Prove magnet stays within the front plate XY footprint (aligned clamp).
    ctx.expect_within(
        backer,
        body,
        axes="xy",
        inner_elem="backer_plate",
        outer_elem="front_plate",
        margin=0.002,
        name="magnet backer stays within front plate XY footprint",
    )

    # --- Badge connector swivel rotation (preserved from parent) ---
    rest_panel = ctx.part_element_world_aabb(connector, elem="perforated_clear_strap")
    with ctx.pose({swivel: math.pi / 2.0}):
        turned_panel = ctx.part_element_world_aabb(connector, elem="perforated_clear_strap")
    if rest_panel is not None and turned_panel is not None:
        rest_dx = rest_panel[1][0] - rest_panel[0][0]
        rest_dy = rest_panel[1][1] - rest_panel[0][1]
        turn_dx = turned_panel[1][0] - turned_panel[0][0]
        turn_dy = turned_panel[1][1] - turned_panel[0][1]
    else:
        rest_dx = rest_dy = turn_dx = turn_dy = 0.0
    ctx.check(
        "badge connector swivels ninety degrees about the button",
        rest_dx > rest_dy * 2.0 and turn_dy > turn_dx * 2.0,
        details=f"rest_dx={rest_dx}, rest_dy={rest_dy}, turn_dx={turn_dx}, turn_dy={turn_dy}",
    )

    return ctx.report()


object_model = build_object_model()
