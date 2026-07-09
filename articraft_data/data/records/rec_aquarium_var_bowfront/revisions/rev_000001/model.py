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


TANK_W = 0.56
TANK_D = 0.32
BASE_H = 0.035
GLASS_H = 0.36
GLASS_T = 0.004
RIM_H = 0.026
POST = 0.018
RAIL_W = 0.026
TOP_Z = BASE_H + GLASS_H
TOP_RIM_TOP = TOP_Z + RIM_H

BOW = 0.035  # front glass bows outward (-Y) at center by this amount

HOOD_W = TANK_W + 0.070
HOOD_D = TANK_D + 0.060
HOOD_T = 0.040
HOOD_BOTTOM_CLEARANCE = 0.006
HOOD_HINGE_Y = TANK_D / 2 + 0.025
HOOD_HINGE_Z = TOP_RIM_TOP + 0.008

FEED_APERTURE_W = 0.140
FEED_APERTURE_D = 0.095
FEED_APERTURE_Y = -0.205
FEED_HINGE_Y = FEED_APERTURE_Y + FEED_APERTURE_D / 2
FEED_HINGE_Z = HOOD_BOTTOM_CLEARANCE + HOOD_T + 0.006


def _hood_shell_mesh():
    """A shallow black molded hood with chamfered outside edges and a feed opening."""
    outer = cq.Workplane("XY").box(HOOD_W, HOOD_D, HOOD_T)
    outer = outer.edges("|Z").chamfer(0.012)
    outer = outer.translate((0.0, -HOOD_D / 2, HOOD_BOTTOM_CLEARANCE + HOOD_T / 2))

    cutter = cq.Workplane("XY").box(FEED_APERTURE_W, FEED_APERTURE_D, HOOD_T * 3.0)
    cutter = cutter.translate(
        (0.0, FEED_APERTURE_Y, HOOD_BOTTOM_CLEARANCE + HOOD_T / 2)
    )
    return outer.cut(cutter)


def _curved_front_panel(width, height, thickness, bow, z_bottom):
    """Build a thin curved panel that bows outward (-Y) for the bow-front aquarium.

    The panel is constructed by boolean subtraction: an outer region bounded by
    the outer arc minus an inner region offset by *thickness* toward +Y.
    The resulting shell has its bottom at z = z_bottom.
    """
    half_w = width / 2
    front_y = -TANK_D / 2
    back_y = front_y + thickness + 0.005

    outer = (
        cq.Workplane("XY")
        .moveTo(-half_w, back_y)
        .lineTo(-half_w, front_y)
        .threePointArc((0.0, front_y - bow), (half_w, front_y))
        .lineTo(half_w, back_y)
        .close()
        .extrude(height)
    )

    inner_front_y = front_y + thickness
    inner_bow_y = inner_front_y - bow

    inner = (
        cq.Workplane("XY")
        .moveTo(-half_w, back_y)
        .lineTo(-half_w, inner_front_y)
        .threePointArc((0.0, inner_bow_y), (half_w, inner_front_y))
        .lineTo(half_w, back_y)
        .close()
        .extrude(height)
    )

    result = outer.cut(inner)
    if z_bottom != 0.0:
        result = result.translate((0.0, 0.0, z_bottom))
    return result


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="small_aquarium",
        meta={
            "category_note": "Reference and category both indicate a small aquarium/fish tank.",
        },
    )

    glass = model.material("slightly_blue_clear_glass", rgba=(0.72, 0.92, 1.0, 0.32))
    black = model.material("satin_black_plastic", rgba=(0.005, 0.005, 0.004, 1.0))
    dark = model.material("dark_control_insert", rgba=(0.030, 0.032, 0.035, 1.0))
    warm_led = model.material("warm_led_diffuser", rgba=(1.0, 0.94, 0.68, 0.80))
    gravel = model.material("mixed_light_gravel", rgba=(0.64, 0.58, 0.45, 1.0))
    gravel_dark = model.material("mixed_dark_gravel", rgba=(0.34, 0.33, 0.29, 1.0))
    red = model.material("red_status_button", rgba=(0.85, 0.08, 0.04, 1.0))
    orange = model.material("orange_status_button", rgba=(0.95, 0.42, 0.05, 1.0))
    green = model.material("green_status_button", rgba=(0.12, 0.68, 0.18, 1.0))

    tank = model.part("tank_frame")

    # Thin transparent panes are modeled as separate held panels rather than a
    # single solid block, so the tank reads as a hollow glass aquarium.
    tank.visual(
        mesh_from_cadquery(
            _curved_front_panel(TANK_W, GLASS_H, GLASS_T, BOW, BASE_H),
            "bow_front_glass",
        ),
        origin=Origin(),
        material=glass,
        name="front_glass",
    )
    tank.visual(
        Box((TANK_W, GLASS_T, GLASS_H)),
        origin=Origin(xyz=(0.0, TANK_D / 2 + GLASS_T / 2, BASE_H + GLASS_H / 2)),
        material=glass,
        name="rear_glass",
    )
    tank.visual(
        Box((GLASS_T, TANK_D, GLASS_H)),
        origin=Origin(xyz=(-TANK_W / 2 - GLASS_T / 2, 0.0, BASE_H + GLASS_H / 2)),
        material=glass,
        name="side_glass_0",
    )
    tank.visual(
        Box((GLASS_T, TANK_D, GLASS_H)),
        origin=Origin(xyz=(TANK_W / 2 + GLASS_T / 2, 0.0, BASE_H + GLASS_H / 2)),
        material=glass,
        name="side_glass_1",
    )
    tank.visual(
        Box((TANK_W, TANK_D, GLASS_T)),
        origin=Origin(xyz=(0.0, 0.0, BASE_H + GLASS_T / 2)),
        material=glass,
        name="bottom_glass",
    )

    # Black plastic lower and upper frames that clamp the panes.
    rail_x = TANK_W + 2 * RAIL_W
    rail_y = TANK_D + 2 * RAIL_W
    tank.visual(
        mesh_from_cadquery(
            _curved_front_panel(rail_x, BASE_H, RAIL_W, BOW, 0.0),
            "curved_base_front_rail",
        ),
        origin=Origin(),
        material=black,
        name="base_front_rail",
    )
    tank.visual(
        Box((rail_x, RAIL_W, BASE_H)),
        origin=Origin(xyz=(0.0, TANK_D / 2 + RAIL_W / 2, BASE_H / 2)),
        material=black,
        name="base_rear_rail",
    )
    tank.visual(
        Box((RAIL_W, rail_y, BASE_H)),
        origin=Origin(xyz=(-TANK_W / 2 - RAIL_W / 2, 0.0, BASE_H / 2)),
        material=black,
        name="base_side_0",
    )
    tank.visual(
        Box((RAIL_W, rail_y, BASE_H)),
        origin=Origin(xyz=(TANK_W / 2 + RAIL_W / 2, 0.0, BASE_H / 2)),
        material=black,
        name="base_side_1",
    )

    tank.visual(
        mesh_from_cadquery(
            _curved_front_panel(rail_x, RIM_H, RAIL_W, BOW, TOP_Z),
            "curved_top_front_rail",
        ),
        origin=Origin(),
        material=black,
        name="top_front_rail",
    )
    tank.visual(
        Box((rail_x, RAIL_W, RIM_H)),
        origin=Origin(xyz=(0.0, TANK_D / 2 + RAIL_W / 2, TOP_Z + RIM_H / 2)),
        material=black,
        name="top_rear_rail",
    )
    tank.visual(
        Box((RAIL_W, rail_y, RIM_H)),
        origin=Origin(xyz=(-TANK_W / 2 - RAIL_W / 2, 0.0, TOP_Z + RIM_H / 2)),
        material=black,
        name="top_side_0",
    )
    tank.visual(
        Box((RAIL_W, rail_y, RIM_H)),
        origin=Origin(xyz=(TANK_W / 2 + RAIL_W / 2, 0.0, TOP_Z + RIM_H / 2)),
        material=black,
        name="top_side_1",
    )

    for ix, x in enumerate((-TANK_W / 2 - POST / 2, TANK_W / 2 + POST / 2)):
        for iy, y in enumerate((-TANK_D / 2 - POST / 2, TANK_D / 2 + POST / 2)):
            tank.visual(
                Box((POST, POST, GLASS_H + RIM_H)),
                origin=Origin(xyz=(x, y, BASE_H + (GLASS_H + RIM_H) / 2)),
                material=black,
                name=f"corner_post_{ix}_{iy}",
            )

    # Fixed knuckles on the rear top rim for the full hood hinge.
    for idx, x in enumerate((-0.19, 0.19)):
        tank.visual(
            Cylinder(radius=0.008, length=0.105),
            origin=Origin(
                xyz=(x, HOOD_HINGE_Y, HOOD_HINGE_Z),
                rpy=(0.0, math.pi / 2, 0.0),
            ),
            material=black,
            name=f"rear_hinge_knuckle_{idx}",
        )

    substrate = model.part("substrate")
    substrate.visual(
        Box((TANK_W - 0.055, TANK_D - 0.055, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=gravel,
        name="gravel_bed",
    )
    # A few partially embedded pebble ridges keep the substrate from reading as a
    # flat solid slab without turning the aquarium into a scenic diorama.
    for i, (x, y, sx, sy, mat) in enumerate(
        (
            (-0.19, -0.08, 0.050, 0.026, gravel_dark),
            (-0.08, 0.06, 0.042, 0.030, gravel),
            (0.08, -0.05, 0.055, 0.022, gravel_dark),
            (0.20, 0.07, 0.046, 0.028, gravel),
        )
    ):
        substrate.visual(
            Box((sx, sy, 0.010)),
            origin=Origin(xyz=(x, y, 0.043)),
            material=mat,
            name=f"gravel_ridge_{i}",
        )
    model.articulation(
        "tank_to_substrate",
        ArticulationType.FIXED,
        parent=tank,
        child=substrate,
        origin=Origin(xyz=(0.0, 0.0, BASE_H + GLASS_T)),
    )

    filter_part = model.part("filter")
    filter_part.visual(
        Box((0.070, 0.075, 0.115)),
        origin=Origin(xyz=(TANK_W / 2 + 0.064, TANK_D / 2 - 0.055, TOP_Z - 0.040)),
        material=black,
        name="filter_housing",
    )
    filter_part.visual(
        Box((0.014, 0.075, 0.070)),
        origin=Origin(xyz=(TANK_W / 2 + RAIL_W + 0.007, TANK_D / 2 - 0.055, TOP_Z - 0.008)),
        material=black,
        name="rim_hanger",
    )
    filter_part.visual(
        Cylinder(radius=0.007, length=0.310),
        origin=Origin(xyz=(TANK_W / 2 - 0.040, TANK_D / 2 - 0.055, BASE_H + 0.200)),
        material=black,
        name="intake_tube",
    )
    filter_part.visual(
        Cylinder(radius=0.0085, length=0.060),
        origin=Origin(xyz=(TANK_W / 2 - 0.040, TANK_D / 2 - 0.055, BASE_H + 0.080)),
        material=black,
        name="strainer_tip",
    )
    filter_part.visual(
        Cylinder(radius=0.006, length=0.105),
        origin=Origin(
            xyz=(TANK_W / 2 + 0.010, TANK_D / 2 - 0.055, TOP_Z - 0.002),
            rpy=(0.0, math.pi / 2, 0.0),
        ),
        material=black,
        name="filter_outlet_elbow",
    )
    model.articulation(
        "tank_to_filter",
        ArticulationType.FIXED,
        parent=tank,
        child=filter_part,
        origin=Origin(),
    )

    hood = model.part("hood")
    hood.visual(
        mesh_from_cadquery(_hood_shell_mesh(), "chamfered_hood_shell"),
        origin=Origin(),
        material=black,
        name="hood_shell",
    )
    hood.visual(
        Box((0.300, 0.025, 0.006)),
        origin=Origin(xyz=(0.0, -0.170, HOOD_BOTTOM_CLEARANCE - 0.003)),
        material=warm_led,
        name="light_diffuser",
    )
    hood.visual(
        Box((0.190, 0.072, 0.004)),
        origin=Origin(xyz=(0.0, -0.120, HOOD_BOTTOM_CLEARANCE + HOOD_T + 0.002)),
        material=dark,
        name="control_panel",
    )
    for idx, (x, mat) in enumerate(((-0.055, red), (-0.025, orange), (0.010, green))):
        hood.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(x, -0.120, HOOD_BOTTOM_CLEARANCE + HOOD_T + 0.006)),
            material=mat,
            name=f"status_button_{idx}",
        )
    for idx, x in enumerate((-0.075, 0.075)):
        hood.visual(
            Cylinder(radius=0.008, length=0.105),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
            material=black,
            name=f"hood_hinge_knuckle_{idx}",
        )
    # Stationary feed-flap hinge leaves on the molded hood.
    for idx, x in enumerate((-0.045, 0.045)):
        hood.visual(
            Cylinder(radius=0.0035, length=0.030),
            origin=Origin(
                xyz=(x, FEED_HINGE_Y, FEED_HINGE_Z),
                rpy=(0.0, math.pi / 2, 0.0),
            ),
            material=black,
            name=f"feed_hinge_leaf_{idx}",
        )

    model.articulation(
        "tank_to_hood",
        ArticulationType.REVOLUTE,
        parent=tank,
        child=hood,
        origin=Origin(xyz=(0.0, HOOD_HINGE_Y, HOOD_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.2, lower=0.0, upper=1.25),
    )

    feed_flap = model.part("feed_flap")
    feed_flap.visual(
        Box((0.158, 0.108, 0.006)),
        origin=Origin(xyz=(0.0, -0.054, 0.0)),
        material=dark,
        name="flap_panel",
    )
    feed_flap.visual(
        Box((0.052, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, -0.099, 0.006)),
        material=black,
        name="finger_lip",
    )
    feed_flap.visual(
        Cylinder(radius=0.0035, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=black,
        name="flap_hinge_knuckle",
    )
    model.articulation(
        "hood_to_feed_flap",
        ArticulationType.REVOLUTE,
        parent=hood,
        child=feed_flap,
        origin=Origin(xyz=(0.0, FEED_HINGE_Y, FEED_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=3.0, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tank = object_model.get_part("tank_frame")
    hood = object_model.get_part("hood")
    feed_flap = object_model.get_part("feed_flap")
    substrate = object_model.get_part("substrate")
    tank_to_hood = object_model.get_articulation("tank_to_hood")
    hood_to_feed_flap = object_model.get_articulation("hood_to_feed_flap")

    ctx.expect_overlap(
        hood,
        tank,
        axes="xy",
        min_overlap=0.020,
        elem_a="hood_shell",
        elem_b="top_front_rail",
        name="hood covers the top frame footprint",
    )
    ctx.expect_gap(
        hood,
        tank,
        axis="z",
        min_gap=0.0,
        max_gap=0.020,
        positive_elem="hood_shell",
        negative_elem="top_front_rail",
        name="closed hood sits just above the front rim",
    )
    ctx.expect_contact(
        substrate,
        tank,
        elem_a="gravel_bed",
        elem_b="bottom_glass",
        contact_tol=0.001,
        name="gravel bed rests on the glass bottom",
    )

    closed_hood = ctx.part_element_world_aabb(hood, elem="hood_shell")
    with ctx.pose({tank_to_hood: 1.0}):
        open_hood = ctx.part_element_world_aabb(hood, elem="hood_shell")
    ctx.check(
        "main hood hinge opens upward",
        closed_hood is not None
        and open_hood is not None
        and open_hood[1][2] > closed_hood[1][2] + 0.12,
        details=f"closed={closed_hood}, open={open_hood}",
    )

    closed_flap = ctx.part_element_world_aabb(feed_flap, elem="flap_panel")
    hood_world_y = HOOD_HINGE_Y
    hood_opening_min = (
        -FEED_APERTURE_W / 2,
        hood_world_y + FEED_HINGE_Y - FEED_APERTURE_D,
    )
    hood_opening_max = (
        FEED_APERTURE_W / 2,
        hood_world_y + FEED_HINGE_Y,
    )
    flap_min, flap_max = closed_flap if closed_flap is not None else ((0, 0, 0), (0, 0, 0))
    ctx.check(
        "closed feeding flap covers the full hood opening",
        closed_flap is not None
        and flap_min[0] <= hood_opening_min[0] - 0.004
        and flap_max[0] >= hood_opening_max[0] + 0.004
        and flap_min[1] <= hood_opening_min[1] - 0.004
        and flap_max[1] >= hood_opening_max[1] - 0.001,
        details=f"opening_min={hood_opening_min}, opening_max={hood_opening_max}, flap={closed_flap}",
    )
    with ctx.pose({hood_to_feed_flap: 1.0}):
        open_flap = ctx.part_element_world_aabb(feed_flap, elem="flap_panel")
    ctx.check(
        "feeding flap hinge opens upward",
        closed_flap is not None
        and open_flap is not None
        and open_flap[1][2] > closed_flap[1][2] + 0.035,
        details=f"closed={closed_flap}, open={open_flap}",
    )

    # Bow-front curvature: the front_glass should extend forward (-Y) beyond
    # the flat front plane, proving the convex outward bow.
    front_glass_aabb = ctx.part_element_world_aabb(tank, elem="front_glass")
    flat_front_y = -TANK_D / 2
    ctx.check(
        "front_glass has bow-front curvature bulging outward",
        front_glass_aabb is not None
        and front_glass_aabb[0][1] < flat_front_y - 0.020,
        details=(
            f"front_glass min_y={front_glass_aabb[0][1] if front_glass_aabb else None}, "
            f"flat_front_y={flat_front_y}, expected bow > 0.020m forward"
        ),
    )
    # The rear glass should remain flat at the rear plane.
    rear_glass_aabb = ctx.part_element_world_aabb(tank, elem="rear_glass")
    ctx.check(
        "rear_glass remains flat (not curved)",
        rear_glass_aabb is not None
        and abs(rear_glass_aabb[0][1] - (TANK_D / 2)) < 0.008,
        details=(
            f"rear_glass min_y={rear_glass_aabb[0][1] if rear_glass_aabb else None}, "
            f"expected near {TANK_D / 2}"
        ),
    )

    ctx.check(
        "asset focuses on aquarium hardware not animals",
        all("fish" not in part.name and "animal" not in part.name for part in object_model.parts),
        details="No animal/character parts should be authored for this asset.",
    )

    return ctx.report()


object_model = build_object_model()
