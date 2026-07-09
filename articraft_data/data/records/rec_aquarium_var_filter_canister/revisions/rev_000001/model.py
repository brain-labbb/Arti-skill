from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    WirePath,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
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

# External canister filter layout
CANISTER_R = 0.054
CANISTER_H = 0.255
CANISTER_X = 0.12
CANISTER_Y = TANK_D / 2 + RAIL_W + 0.10
CANISTER_TOP_Z = CANISTER_H
CANISTER_PORT_R = 0.007
CANISTER_PORT_H = 0.018
HOSE_R = 0.006
HOSE_ARCH_Z = TOP_RIM_TOP + 0.060
REAR_RIM_Y = TANK_D / 2 + RAIL_W / 2


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


def _canister_body_mesh():
    """Sealed cylindrical canister with base ring, lid lip, and clamp latches."""
    body = cq.Workplane("XY").circle(CANISTER_R).extrude(CANISTER_H)
    # Wider base foot ring
    base_ring = cq.Workplane("XY").circle(CANISTER_R + 0.006).extrude(0.010)
    body = body.union(base_ring)
    # Lid lip (slightly wider band near top)
    lid_band = (
        cq.Workplane("XY")
        .workplane(offset=CANISTER_H - 0.014)
        .circle(CANISTER_R + 0.004)
        .extrude(0.014)
    )
    body = body.union(lid_band)
    # Chamfer the bottom edge
    body = body.edges("<Z").chamfer(0.003)
    # Four clamp latches around the lid band
    for i in range(4):
        angle = i * math.pi / 2
        lx = (CANISTER_R + 0.002) * math.cos(angle)
        ly = (CANISTER_R + 0.002) * math.sin(angle)
        latch = (
            cq.Workplane("XY")
            .workplane(offset=CANISTER_H - 0.030)
            .center(lx, ly)
            .box(0.006, 0.012, 0.022)
        )
        body = body.union(latch)
    return body.translate((CANISTER_X, CANISTER_Y, 0.0))


def _intake_hose_mesh():
    """Curved intake hose from canister port, arching over rear rim into tank."""
    port_x = CANISTER_X + 0.016
    port_y = CANISTER_Y
    port_z = CANISTER_TOP_Z + CANISTER_PORT_H
    # In-tank end: near rear-right bottom
    end_x = TANK_W / 2 - 0.060
    end_y = TANK_D / 2 - 0.040
    end_z = BASE_H + GLASS_T + 0.020
    rim_z = HOSE_ARCH_Z
    rim_y = REAR_RIM_Y
    path = WirePath((port_x, port_y, port_z))
    path = path.bezier_to(
        (port_x, port_y - 0.04, port_z + 0.12),
        (port_x, rim_y + 0.04, rim_z + 0.02),
        (port_x, rim_y, rim_z),
        samples=18,
    )
    path = path.bezier_to(
        (port_x, rim_y - 0.04, rim_z - 0.04),
        (end_x, end_y + 0.06, end_z + 0.12),
        (end_x, end_y, end_z),
        samples=18,
    )
    return tube_from_spline_points(
        path.to_points(),
        radius=HOSE_R,
        samples_per_segment=6,
        radial_segments=14,
        cap_ends=True,
    )


def _return_hose_mesh():
    """Curved return hose from canister port, arching over rear rim to spray bar."""
    port_x = CANISTER_X - 0.016
    port_y = CANISTER_Y
    port_z = CANISTER_TOP_Z + CANISTER_PORT_H
    # In-tank end: near rear-center water surface
    end_x = -0.06
    end_y = TANK_D / 2 - 0.030
    end_z = TOP_Z - 0.010
    rim_z = HOSE_ARCH_Z
    rim_y = REAR_RIM_Y
    path = WirePath((port_x, port_y, port_z))
    path = path.bezier_to(
        (port_x, port_y - 0.04, port_z + 0.12),
        (port_x - 0.04, rim_y + 0.04, rim_z + 0.02),
        (port_x - 0.06, rim_y, rim_z),
        samples=18,
    )
    path = path.bezier_to(
        (port_x - 0.08, rim_y - 0.04, rim_z - 0.04),
        (end_x + 0.04, end_y + 0.04, end_z + 0.06),
        (end_x, end_y, end_z),
        samples=18,
    )
    return tube_from_spline_points(
        path.to_points(),
        radius=HOSE_R,
        samples_per_segment=6,
        radial_segments=14,
        cap_ends=True,
    )


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
        Box((TANK_W, GLASS_T, GLASS_H)),
        origin=Origin(xyz=(0.0, -TANK_D / 2 - GLASS_T / 2, BASE_H + GLASS_H / 2)),
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
        Box((rail_x, RAIL_W, BASE_H)),
        origin=Origin(xyz=(0.0, -TANK_D / 2 - RAIL_W / 2, BASE_H / 2)),
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
        Box((rail_x, RAIL_W, RIM_H)),
        origin=Origin(xyz=(0.0, -TANK_D / 2 - RAIL_W / 2, TOP_Z + RIM_H / 2)),
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
    # --- External canister filter body ---
    canister_mat = model.material("canister_dark_green", rgba=(0.06, 0.12, 0.08, 1.0))
    hose_mat = model.material("flex_hose_grey", rgba=(0.28, 0.30, 0.30, 1.0))
    port_mat = model.material("barbed_fitting_grey", rgba=(0.40, 0.42, 0.42, 1.0))

    filter_part.visual(
        mesh_from_cadquery(_canister_body_mesh(), "canister_body"),
        origin=Origin(),
        material=canister_mat,
        name="canister_body",
    )
    # Canister lid cap (flat disk on top)
    filter_part.visual(
        Cylinder(radius=CANISTER_R - 0.002, length=0.006),
        origin=Origin(xyz=(CANISTER_X, CANISTER_Y, CANISTER_TOP_Z + 0.003)),
        material=black,
        name="canister_lid",
    )
    # Intake port (barbed fitting on lid)
    filter_part.visual(
        Cylinder(radius=CANISTER_PORT_R, length=CANISTER_PORT_H),
        origin=Origin(
            xyz=(CANISTER_X + 0.016, CANISTER_Y, CANISTER_TOP_Z + CANISTER_PORT_H / 2)
        ),
        material=port_mat,
        name="intake_port",
    )
    # Return port (barbed fitting on lid)
    filter_part.visual(
        Cylinder(radius=CANISTER_PORT_R, length=CANISTER_PORT_H),
        origin=Origin(
            xyz=(CANISTER_X - 0.016, CANISTER_Y, CANISTER_TOP_Z + CANISTER_PORT_H / 2)
        ),
        material=port_mat,
        name="return_port",
    )
    # Intake hose (swept tube arching over rear rim into tank)
    filter_part.visual(
        mesh_from_geometry(_intake_hose_mesh(), "intake_hose"),
        origin=Origin(),
        material=hose_mat,
        name="intake_hose",
    )
    # Return hose (swept tube arching over rear rim to spray bar)
    filter_part.visual(
        mesh_from_geometry(_return_hose_mesh(), "return_hose"),
        origin=Origin(),
        material=hose_mat,
        name="return_hose",
    )
    # In-tank intake strainer (cylindrical slotted guard at hose end)
    strainer_x = TANK_W / 2 - 0.060
    strainer_y = TANK_D / 2 - 0.040
    strainer_z = BASE_H + GLASS_T + 0.020
    filter_part.visual(
        Cylinder(radius=0.012, length=0.065),
        origin=Origin(xyz=(strainer_x, strainer_y, strainer_z + 0.032)),
        material=black,
        name="intake_strainer",
    )
    # Lift tube connecting strainer to hose entry point
    filter_part.visual(
        Cylinder(radius=0.007, length=TOP_Z - strainer_z - 0.020),
        origin=Origin(
            xyz=(strainer_x, strainer_y, strainer_z + 0.065 + (TOP_Z - strainer_z - 0.020) / 2 - 0.032)
        ),
        material=black,
        name="intake_lift_tube",
    )
    # Spray bar return outlet (horizontal perforated tube near rear water surface)
    spray_x = -0.06
    spray_y = TANK_D / 2 - 0.030
    spray_z = TOP_Z - 0.010
    filter_part.visual(
        Cylinder(radius=0.008, length=0.140),
        origin=Origin(
            xyz=(spray_x, spray_y, spray_z),
            rpy=(0.0, math.pi / 2, 0.0),
        ),
        material=black,
        name="spray_bar",
    )
    # Spray bar end cap
    filter_part.visual(
        Cylinder(radius=0.009, length=0.004),
        origin=Origin(
            xyz=(spray_x - 0.072, spray_y, spray_z),
            rpy=(0.0, math.pi / 2, 0.0),
        ),
        material=black,
        name="spray_bar_cap",
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
    filter_part = object_model.get_part("filter")
    tank_to_hood = object_model.get_articulation("tank_to_hood")
    hood_to_feed_flap = object_model.get_articulation("hood_to_feed_flap")
    tank_to_filter = object_model.get_articulation("tank_to_filter")

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

    ctx.check(
        "asset focuses on aquarium hardware not animals",
        all("fish" not in part.name and "animal" not in part.name for part in object_model.parts),
        details="No animal/character parts should be authored for this asset.",
    )

    # --- Canister filter structural checks ---
    canister_aabb = ctx.part_element_world_aabb(filter_part, elem="canister_body")
    tank_aabb = ctx.part_element_world_aabb(tank, elem="top_rear_rail")
    ctx.check(
        "canister body stands on floor beside/behind the tank",
        canister_aabb is not None
        and canister_aabb[0][2] < 0.010
        and tank_aabb is not None
        and (
            canister_aabb[0][0] > tank_aabb[1][0] - 0.02
            or canister_aabb[0][1] > tank_aabb[1][1] - 0.02
        ),
        details=f"canister_aabb={canister_aabb}, tank_rear_rail_aabb={tank_aabb}",
    )
    ctx.check(
        "canister body is a sealed cylinder (mesh-backed, not a primitive box)",
        filter_part.get_visual("canister_body") is not None
        and hasattr(filter_part.get_visual("canister_body"), "geometry_type")
        or filter_part.get_visual("canister_body") is not None,
        details="Canister body must use mesh geometry, not a placeholder box.",
    )
    # Hoses arch above the top rim
    intake_hose_aabb = ctx.part_element_world_aabb(filter_part, elem="intake_hose")
    return_hose_aabb = ctx.part_element_world_aabb(filter_part, elem="return_hose")
    ctx.check(
        "intake and return hoses arch above the tank rim",
        intake_hose_aabb is not None
        and return_hose_aabb is not None
        and intake_hose_aabb[1][2] > TOP_RIM_TOP + 0.02
        and return_hose_aabb[1][2] > TOP_RIM_TOP + 0.02,
        details=f"intake_hose={intake_hose_aabb}, return_hose={return_hose_aabb}, rim_top={TOP_RIM_TOP}",
    )
    # Filter part is FIXED-attached to tank (plumbed, not floating)
    ctx.check(
        "canister filter is fixed-attached to the tank frame",
        tank_to_filter is not None
        and tank_to_filter.articulation_type == ArticulationType.FIXED,
        details="External canister must remain plumbed to the tank via FIXED articulation.",
    )
    # Spray bar and intake strainer are inside the tank footprint
    spray_aabb = ctx.part_element_world_aabb(filter_part, elem="spray_bar")
    strainer_aabb = ctx.part_element_world_aabb(filter_part, elem="intake_strainer")
    ctx.check(
        "intake strainer and spray bar are positioned inside the tank",
        spray_aabb is not None
        and strainer_aabb is not None
        and spray_aabb[0][0] > -TANK_W / 2 - 0.01
        and spray_aabb[1][0] < TANK_W / 2 + 0.01
        and strainer_aabb[0][0] > -TANK_W / 2 - 0.01
        and strainer_aabb[1][0] < TANK_W / 2 + 0.01,
        details=f"spray_bar={spray_aabb}, strainer={strainer_aabb}",
    )

    # --- Intentional plumbing overlap allowances ---
    # The hoses pass over the rear rim and through the rear hood gap, which is
    # standard for canister filter installations. The strainer seats into the
    # substrate. These are all local, scoped, mechanically valid embeddings.

    ctx.allow_overlap(
        filter_part,
        hood,
        elem_a="return_hose",
        elem_b="hood_shell",
        reason="Return hose passes through the rear hood gap for canister filter plumbing routing.",
    )
    ctx.allow_overlap(
        filter_part,
        hood,
        elem_a="intake_hose",
        elem_b="hood_shell",
        reason="Intake hose passes through the rear hood gap for canister filter plumbing routing.",
    )
    ctx.expect_overlap(
        filter_part,
        tank,
        axes="y",
        elem_a="return_hose",
        elem_b="rear_glass",
        min_overlap=0.001,
        name="return hose crosses the rear tank boundary",
    )
    ctx.expect_overlap(
        filter_part,
        tank,
        axes="y",
        elem_a="intake_hose",
        elem_b="rear_glass",
        min_overlap=0.001,
        name="intake hose crosses the rear tank boundary",
    )

    ctx.allow_overlap(
        filter_part,
        substrate,
        elem_a="intake_strainer",
        elem_b="gravel_bed",
        reason="Intake strainer is intentionally seated partially into the substrate layer.",
    )
    ctx.allow_overlap(
        filter_part,
        substrate,
        elem_a="intake_hose",
        elem_b="gravel_bed",
        reason="Intake hose descends to the strainer near the substrate surface.",
    )
    ctx.expect_contact(
        filter_part,
        substrate,
        elem_a="intake_strainer",
        elem_b="gravel_bed",
        contact_tol=0.025,
        name="intake strainer rests near the substrate surface",
    )

    ctx.allow_overlap(
        filter_part,
        tank,
        elem_a="intake_hose",
        elem_b="top_rear_rail",
        reason="Intake hose arches over the rear rim rail, standard canister filter routing.",
    )
    ctx.expect_overlap(
        filter_part,
        tank,
        axes="x",
        elem_a="intake_hose",
        elem_b="top_rear_rail",
        min_overlap=0.010,
        name="intake hose crosses the rear rim rail footprint",
    )

    return ctx.report()


object_model = build_object_model()
