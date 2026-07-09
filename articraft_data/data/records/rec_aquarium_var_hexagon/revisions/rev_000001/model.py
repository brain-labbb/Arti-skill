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

# ---------------------------------------------------------------------------
# Hexagonal tank geometry
# ---------------------------------------------------------------------------
NUM_SIDES = 6
HEX_R = 0.27  # circumradius (center to vertex)
HEX_INRADIUS = HEX_R * math.cos(math.pi / 6)  # ~0.2338 (center to edge midpoint)
HEX_SIDE = HEX_R  # regular hexagon: side length = circumradius
HEX_DIAMETER = 2 * HEX_R  # across-corners diameter

BASE_H = 0.035
GLASS_H = 0.36
GLASS_T = 0.004
RIM_H = 0.026
POST = 0.018
RAIL_W = 0.026
TOP_Z = BASE_H + GLASS_H
TOP_RIM_TOP = TOP_Z + RIM_H

# ---------------------------------------------------------------------------
# Hood geometry (hexagonal lid with generous overhang)
# ---------------------------------------------------------------------------
HOOD_OVERHANG = 0.060
HOOD_R = HEX_R + HOOD_OVERHANG
HOOD_INRADIUS = HOOD_R * math.cos(math.pi / 6)
HOOD_T = 0.040
HOOD_BOTTOM_CLEARANCE = 0.006
HOOD_HINGE_Y = HEX_INRADIUS + 0.025
HOOD_HINGE_Z = TOP_RIM_TOP + 0.008

# Hood-local frame: hood origin is at the hinge line.
HOOD_CENTER_Y_LOCAL = -HOOD_HINGE_Y

# Feed aperture (in hood-local frame)
FEED_APERTURE_W = 0.120
FEED_APERTURE_D = 0.085
FEED_APERTURE_Y = HOOD_CENTER_Y_LOCAL - 0.060
FEED_HINGE_Y = FEED_APERTURE_Y + FEED_APERTURE_D / 2
FEED_HINGE_Z = HOOD_BOTTOM_CLEARANCE + HOOD_T + 0.006


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def hex_face_angle(i: int) -> float:
    """Outward-normal angle (radians) for hexagonal face *i*.

    Face 0 -> -Y (front), face 3 -> +Y (rear).
    """
    return -math.pi / 2 + i * math.pi / 3


def hex_vertex_angle(j: int) -> float:
    """Angle (radians) for hexagonal vertex *j*."""
    return -math.pi / 3 + j * math.pi / 3


def _hex_plate(circumradius: float, height: float, z_offset: float = 0.0):
    """CadQuery hexagonal prism (flat face at -Y) centred at XY origin."""
    return (
        cq.Workplane("XY")
        .polygon(NUM_SIDES, 2 * circumradius)
        .extrude(height)
        .translate((0.0, 0.0, z_offset))
    )


def _hood_shell_mesh():
    """Hexagonal hood shell with chamfered edges and feed-opening cut."""
    shell = (
        cq.Workplane("XY")
        .polygon(NUM_SIDES, 2 * HOOD_R)
        .extrude(HOOD_T)
    )
    shell = shell.edges("|Z").chamfer(0.008)
    shell = shell.translate((0.0, HOOD_CENTER_Y_LOCAL, HOOD_BOTTOM_CLEARANCE))

    cutter = cq.Workplane("XY").box(
        FEED_APERTURE_W, FEED_APERTURE_D, HOOD_T * 3.0
    )
    cutter = cutter.translate(
        (0.0, FEED_APERTURE_Y, HOOD_BOTTOM_CLEARANCE + HOOD_T / 2)
    )
    return shell.cut(cutter)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="hexagonal_aquarium",
        meta={
            "category_note": (
                "Hexagonal corner aquarium - transparent glass basin with "
                "black frame, hinged hood, filter and gravel bed."
            ),
        },
    )

    # -- materials -----------------------------------------------------------
    glass = model.material("slightly_blue_clear_glass", rgba=(0.72, 0.92, 1.0, 0.32))
    black = model.material("satin_black_plastic", rgba=(0.005, 0.005, 0.004, 1.0))
    dark = model.material("dark_control_insert", rgba=(0.030, 0.032, 0.035, 1.0))
    warm_led = model.material("warm_led_diffuser", rgba=(1.0, 0.94, 0.68, 0.80))
    gravel = model.material("mixed_light_gravel", rgba=(0.64, 0.58, 0.45, 1.0))
    gravel_dark = model.material("mixed_dark_gravel", rgba=(0.34, 0.33, 0.29, 1.0))
    red = model.material("red_status_button", rgba=(0.85, 0.08, 0.04, 1.0))
    orange = model.material("orange_status_button", rgba=(0.95, 0.42, 0.05, 1.0))
    green = model.material("green_status_button", rgba=(0.12, 0.68, 0.18, 1.0))

    # =======================================================================
    # tank_frame - hexagonal glass prism with black base / rim frame
    # =======================================================================
    tank = model.part("tank_frame")

    # -- six glass wall panels (loop with indexed names) --------------------
    for i in range(NUM_SIDES):
        angle = hex_face_angle(i)
        glass_r = HEX_INRADIUS + GLASS_T / 2
        mx = glass_r * math.cos(angle)
        my = glass_r * math.sin(angle)
        rot_z = angle - math.pi / 2
        tank.visual(
            Box((HEX_SIDE, GLASS_T, GLASS_H)),
            origin=Origin(
                xyz=(mx, my, BASE_H + GLASS_H / 2),
                rpy=(0.0, 0.0, rot_z),
            ),
            material=glass,
            name=f"wall_glass_{i}",
        )

    # -- hexagonal bottom glass plate ---------------------------------------
    tank.visual(
        mesh_from_cadquery(
            _hex_plate(HEX_R, GLASS_T, BASE_H), "hex_bottom_glass"
        ),
        origin=Origin(),
        material=glass,
        name="bottom_glass",
    )

    # -- base rail ring (one bar per face) ----------------------------------
    for i in range(NUM_SIDES):
        angle = hex_face_angle(i)
        rail_r = HEX_INRADIUS + RAIL_W / 2
        mx = rail_r * math.cos(angle)
        my = rail_r * math.sin(angle)
        rot_z = angle - math.pi / 2
        rail_name = "base_front_rail" if i == 0 else f"base_rail_{i}"
        tank.visual(
            Box((HEX_SIDE + RAIL_W, RAIL_W, BASE_H)),
            origin=Origin(
                xyz=(mx, my, BASE_H / 2),
                rpy=(0.0, 0.0, rot_z),
            ),
            material=black,
            name=rail_name,
        )

    # -- top rim ring (one bar per face) ------------------------------------
    for i in range(NUM_SIDES):
        angle = hex_face_angle(i)
        rail_r = HEX_INRADIUS + RAIL_W / 2
        mx = rail_r * math.cos(angle)
        my = rail_r * math.sin(angle)
        rot_z = angle - math.pi / 2
        rail_name = "top_front_rail" if i == 0 else f"top_rail_{i}"
        tank.visual(
            Box((HEX_SIDE + RAIL_W, RAIL_W, RIM_H)),
            origin=Origin(
                xyz=(mx, my, TOP_Z + RIM_H / 2),
                rpy=(0.0, 0.0, rot_z),
            ),
            material=black,
            name=rail_name,
        )

    # -- six corner posts at hexagonal vertices -----------------------------
    for j in range(NUM_SIDES):
        angle = hex_vertex_angle(j)
        vx = HEX_R * math.cos(angle)
        vy = HEX_R * math.sin(angle)
        tank.visual(
            Box((POST, POST, GLASS_H + RIM_H)),
            origin=Origin(xyz=(vx, vy, BASE_H + (GLASS_H + RIM_H) / 2)),
            material=black,
            name=f"corner_post_{j}",
        )

    # -- rear hinge knuckles on the tank (spaced away from hood knuckles) ---
    for idx, x in enumerate((-0.155, 0.155)):
        tank.visual(
            Cylinder(radius=0.008, length=0.040),
            origin=Origin(
                xyz=(x, HOOD_HINGE_Y, HOOD_HINGE_Z),
                rpy=(0.0, math.pi / 2, 0.0),
            ),
            material=black,
            name=f"rear_hinge_knuckle_{idx}",
        )

    # =======================================================================
    # substrate - gravel bed sitting on the glass bottom
    # =======================================================================
    substrate = model.part("substrate")
    substrate.visual(
        mesh_from_cadquery(
            _hex_plate(HEX_R - 0.030, 0.040, 0.0), "hex_gravel_bed"
        ),
        origin=Origin(),
        material=gravel,
        name="gravel_bed",
    )
    for i, (x, y, sx, sy, mat) in enumerate((
        (-0.12, -0.08, 0.045, 0.024, gravel_dark),
        (-0.04, 0.06, 0.038, 0.028, gravel),
        (0.06, -0.05, 0.050, 0.020, gravel_dark),
        (0.13, 0.07, 0.042, 0.026, gravel),
    )):
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

    # =======================================================================
    # filter - hang-on-back filter near the rear face
    # =======================================================================
    filter_part = model.part("filter")

    # Position filter clearly outside the tank rear face.
    filt_x = 0.10
    filt_body_y = HEX_INRADIUS + RAIL_W + 0.040  # ~0.300

    filter_part.visual(
        Box((0.070, 0.075, 0.115)),
        origin=Origin(xyz=(filt_x, filt_body_y, TOP_Z - 0.040)),
        material=black,
        name="filter_housing",
    )

    # Rim hanger: hook plate bridging from the rear rail top to the housing.
    hook_y_center = (HEX_INRADIUS + RAIL_W / 2 + filt_body_y - 0.075 / 2) / 2
    hook_y_length = (filt_body_y - 0.075 / 2) - (HEX_INRADIUS + RAIL_W / 2) + 0.010
    # Z height spans from just below the housing top to above the rim.
    housing_top_z = TOP_Z - 0.040 + 0.115 / 2
    hanger_bot_z = housing_top_z - 0.001
    hanger_top_z = TOP_RIM_TOP + 0.003
    hanger_z_height = hanger_top_z - hanger_bot_z
    hanger_z_center = (hanger_bot_z + hanger_top_z) / 2
    filter_part.visual(
        Box((0.020, hook_y_length, hanger_z_height)),
        origin=Origin(xyz=(filt_x, hook_y_center, hanger_z_center)),
        material=black,
        name="rim_hanger",
    )

    # Intake elbow: horizontal tube from housing toward the tank interior.
    elbow_y_start = filt_body_y - 0.075 / 2  # housing front face
    elbow_y_end = HEX_INRADIUS - 0.030  # just inside the hex
    elbow_length = elbow_y_start - elbow_y_end
    elbow_cy = (elbow_y_start + elbow_y_end) / 2
    elbow_z = TOP_Z - 0.010  # near the top of the glass
    filter_part.visual(
        Cylinder(radius=0.009, length=elbow_length),
        origin=Origin(
            xyz=(filt_x, elbow_cy, elbow_z),
            rpy=(math.pi / 2, 0.0, 0.0),
        ),
        material=black,
        name="intake_elbow",
    )

    # Intake tube: vertical tube going down inside the tank from the elbow.
    tube_top_z = elbow_z
    tube_bot_z = BASE_H + 0.110
    tube_length = tube_top_z - tube_bot_z
    tube_cz = (tube_top_z + tube_bot_z) / 2
    filter_part.visual(
        Cylinder(radius=0.007, length=tube_length),
        origin=Origin(xyz=(filt_x, elbow_y_end, tube_cz)),
        material=black,
        name="intake_tube",
    )

    # Strainer tip at the bottom of the intake tube.
    filter_part.visual(
        Cylinder(radius=0.0085, length=0.060),
        origin=Origin(xyz=(filt_x, elbow_y_end, tube_bot_z - 0.030)),
        material=black,
        name="strainer_tip",
    )

    # Filter outlet elbow on the outside (near housing top).
    outlet_z = TOP_Z - 0.040 + 0.115 / 2 - 0.010  # inside housing Z range
    filter_part.visual(
        Cylinder(radius=0.006, length=0.080),
        origin=Origin(
            xyz=(filt_x, filt_body_y + 0.010, outlet_z),
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

    # =======================================================================
    # hood - hexagonal lid on rear revolute hinge
    # =======================================================================
    hood = model.part("hood")
    hood.visual(
        mesh_from_cadquery(_hood_shell_mesh(), "hex_hood_shell"),
        origin=Origin(),
        material=black,
        name="hood_shell",
    )
    hood.visual(
        Box((0.250, 0.025, 0.006)),
        origin=Origin(
            xyz=(0.0, HOOD_CENTER_Y_LOCAL, HOOD_BOTTOM_CLEARANCE - 0.003)
        ),
        material=warm_led,
        name="light_diffuser",
    )
    hood.visual(
        Box((0.160, 0.060, 0.004)),
        origin=Origin(
            xyz=(0.0, HOOD_CENTER_Y_LOCAL + 0.10,
                 HOOD_BOTTOM_CLEARANCE + HOOD_T + 0.002)
        ),
        material=dark,
        name="control_panel",
    )
    for idx, (x, mat) in enumerate(((-0.045, red), (-0.015, orange), (0.020, green))):
        hood.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(
                xyz=(x, HOOD_CENTER_Y_LOCAL + 0.10,
                     HOOD_BOTTOM_CLEARANCE + HOOD_T + 0.006)
            ),
            material=mat,
            name=f"status_button_{idx}",
        )

    # Hood hinge knuckles (interleaved with tank knuckles, no X overlap).
    hood_shell_rear_local = HOOD_CENTER_Y_LOCAL + HOOD_INRADIUS
    hinge_bracket_dy = hood_shell_rear_local / 2  # midpoint from shell to hinge
    for idx, x in enumerate((-0.055, 0.055)):
        hood.visual(
            Cylinder(radius=0.008, length=0.040),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
            material=black,
            name=f"hood_hinge_knuckle_{idx}",
        )
        # Bracket connecting the knuckle to the hood shell rear edge.
        hood.visual(
            Box((0.018, abs(hood_shell_rear_local) + 0.002, 0.018)),
            origin=Origin(
                xyz=(x, hinge_bracket_dy, HOOD_BOTTOM_CLEARANCE / 2)
            ),
            material=black,
            name=f"hinge_bracket_{idx}",
        )

    # Stationary feed-flap hinge leaves mounted on the hood top surface.
    hood_top_z = HOOD_BOTTOM_CLEARANCE + HOOD_T
    for idx, x in enumerate((-0.040, 0.040)):
        hood.visual(
            Cylinder(radius=0.004, length=0.028),
            origin=Origin(
                xyz=(x, FEED_HINGE_Y, hood_top_z + 0.001),
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
        motion_limits=MotionLimits(
            effort=6.0, velocity=2.2, lower=0.0, upper=1.25
        ),
    )

    # =======================================================================
    # feed_flap - nested revolute on the hood
    # =======================================================================
    feed_flap = model.part("feed_flap")
    flap_d = FEED_APERTURE_D + 0.010
    flap_w = FEED_APERTURE_W + 0.016
    feed_flap.visual(
        Box((flap_w, flap_d, 0.006)),
        origin=Origin(xyz=(0.0, -flap_d / 2, 0.0)),
        material=dark,
        name="flap_panel",
    )
    feed_flap.visual(
        Box((0.048, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, -flap_d + 0.005, 0.006)),
        material=black,
        name="finger_lip",
    )
    feed_flap.visual(
        Cylinder(radius=0.0035, length=0.035),
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
        motion_limits=MotionLimits(
            effort=1.0, velocity=3.0, lower=0.0, upper=1.35
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tank = object_model.get_part("tank_frame")
    hood = object_model.get_part("hood")
    feed_flap = object_model.get_part("feed_flap")
    substrate = object_model.get_part("substrate")
    tank_to_hood = object_model.get_articulation("tank_to_hood")
    hood_to_feed_flap = object_model.get_articulation("hood_to_feed_flap")

    # -- hexagonal prism structure ------------------------------------------
    found_names = {v.name for v in tank.visuals}
    wall_names = {f"wall_glass_{i}" for i in range(NUM_SIDES)}
    post_names = {f"corner_post_{i}" for i in range(NUM_SIDES)}
    ctx.check(
        "hexagonal prism has six wall_glass panels and six corner_posts",
        wall_names.issubset(found_names) and post_names.issubset(found_names),
        details=(
            f"Missing walls: {wall_names - found_names}, "
            f"posts: {post_names - found_names}"
        ),
    )

    # Front (wall_glass_0 at -Y) and rear (wall_glass_3 at +Y) panels
    # should straddle the origin confirming hexagonal prism symmetry.
    ctx.expect_overlap(
        tank, tank,
        axes="x",
        min_overlap=HEX_SIDE * 0.6,
        elem_a="wall_glass_0",
        elem_b="wall_glass_3",
        name="front and rear hex panels share X footprint (hexagonal prism)",
    )

    # -- hood covers the hexagonal top frame --------------------------------
    ctx.expect_overlap(
        hood, tank,
        axes="xy",
        min_overlap=0.020,
        elem_a="hood_shell",
        elem_b="top_front_rail",
        name="hood covers the top frame footprint",
    )
    ctx.expect_gap(
        hood, tank,
        axis="z",
        min_gap=0.0,
        max_gap=0.020,
        positive_elem="hood_shell",
        negative_elem="top_front_rail",
        name="closed hood sits just above the front rim",
    )

    # -- gravel bed on glass bottom -----------------------------------------
    ctx.expect_contact(
        substrate, tank,
        elem_a="gravel_bed",
        elem_b="bottom_glass",
        contact_tol=0.001,
        name="gravel bed rests on the glass bottom",
    )

    # -- main hood hinge opens upward ---------------------------------------
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

    # -- feeding flap covers the hood opening -------------------------------
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
        details=(
            f"opening_min={hood_opening_min}, "
            f"opening_max={hood_opening_max}, flap={closed_flap}"
        ),
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

    # -- filter hook mounts on the rear rim (intentional local overlap) -----
    filter_part = object_model.get_part("filter")
    ctx.allow_overlap(
        filter_part, tank,
        elem_a="rim_hanger",
        elem_b="top_rail_3",
        reason=(
            "The filter rim hanger intentionally wraps over the rear top rail "
            "to hook-mount the hang-on-back filter body on the tank rim."
        ),
    )
    ctx.expect_contact(
        filter_part, tank,
        elem_a="rim_hanger",
        elem_b="top_rail_3",
        contact_tol=0.012,
        name="filter rim hanger seats on the rear top rail",
    )

    # -- no animal / character parts ----------------------------------------
    ctx.check(
        "asset focuses on aquarium hardware not animals",
        all(
            "fish" not in p.name and "animal" not in p.name
            for p in object_model.parts
        ),
        details="No animal/character parts should be authored for this asset.",
    )

    return ctx.report()


object_model = build_object_model()
