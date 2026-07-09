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

# ── cylindrical tank dimensions ──────────────────────────────────────────
TANK_R = 0.20           # outer radius of the glass wall (m)
GLASS_H = 0.36          # glass wall height
GLASS_T = 0.004         # glass thickness
BASE_H = 0.035          # base ring height
RIM_H = 0.026           # top rim ring height
RAIL_W = 0.022          # radial width of the ring frames
TOP_Z = BASE_H + GLASS_H
TOP_RIM_TOP = TOP_Z + RIM_H

# ── hood / lid ────────────────────────────────────────────────────────────
HOOD_R = TANK_R + 0.030        # hood disc radius (slight overhang)
HOOD_T = 0.040                 # hood thickness
HOOD_BOTTOM_CLEARANCE = 0.006  # gap between rim top and hood underside
HOOD_HINGE_Y = TANK_R + 0.025  # hinge at the rear tangent of the tank
HOOD_HINGE_Z = TOP_RIM_TOP + 0.008

# ── feed aperture (circular opening in the round lid) ────────────────────
FEED_APERTURE_R = 0.058        # radius of the round feed opening
FEED_APERTURE_Y = -0.075       # Y-centre of the opening (toward front)
FEED_HINGE_Y = FEED_APERTURE_Y + FEED_APERTURE_R  # front edge of opening
FEED_HINGE_Z = HOOD_BOTTOM_CLEARANCE + HOOD_T + 0.002  # embeds leaves into shell


# ── CadQuery geometry helpers ────────────────────────────────────────────

def _cylindrical_glass_mesh():
    """Thin annular glass wall – a hollow cylinder open at top and bottom."""
    return (
        cq.Workplane("XY")
        .circle(TANK_R)
        .circle(TANK_R - GLASS_T)
        .extrude(GLASS_H)
        .translate((0.0, 0.0, BASE_H))
    )


def _bottom_disc_mesh():
    """Flat glass disc that closes the bottom of the cylinder."""
    return (
        cq.Workplane("XY")
        .circle(TANK_R - GLASS_T)
        .extrude(GLASS_T)
        .translate((0.0, 0.0, BASE_H))
    )


def _base_ring_mesh():
    """Annular black plastic base ring that cradles the glass wall."""
    return (
        cq.Workplane("XY")
        .circle(TANK_R + RAIL_W)
        .circle(TANK_R - RAIL_W)
        .extrude(BASE_H)
    )


def _top_ring_mesh():
    """Annular black plastic top rim ring."""
    return (
        cq.Workplane("XY")
        .circle(TANK_R + RAIL_W)
        .circle(TANK_R - RAIL_W)
        .extrude(RIM_H)
        .translate((0.0, 0.0, TOP_Z))
    )


def _hood_shell_mesh():
    """Round black molded hood disc with a circular feed opening."""
    disc = (
        cq.Workplane("XY")
        .circle(HOOD_R)
        .extrude(HOOD_T)
        .translate((0.0, 0.0, HOOD_BOTTOM_CLEARANCE))
    )
    cutter = (
        cq.Workplane("XY")
        .circle(FEED_APERTURE_R)
        .extrude(HOOD_T * 3.0)
        .translate((0.0, FEED_APERTURE_Y, HOOD_BOTTOM_CLEARANCE - HOOD_T))
    )
    return disc.cut(cutter)


def _flap_disc_mesh():
    """Round flap panel that covers the circular feed aperture."""
    return (
        cq.Workplane("XY")
        .circle(FEED_APERTURE_R + 0.006)
        .extrude(0.006)
    )


# ── model construction ───────────────────────────────────────────────────

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="column_aquarium",
        meta={
            "category_note": "Cylindrical column aquarium – form variant of rectangular tank.",
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

    # ── tank frame (root part) ────────────────────────────────────────
    tank = model.part("tank_frame")

    # Single cylindrical glass wall replaces the four flat panes
    tank.visual(
        mesh_from_cadquery(_cylindrical_glass_mesh(), "cylindrical_glass_wall"),
        origin=Origin(),
        material=glass,
        name="glass_wall",
    )

    # Round bottom glass disc
    tank.visual(
        mesh_from_cadquery(_bottom_disc_mesh(), "bottom_glass_disc"),
        origin=Origin(),
        material=glass,
        name="bottom_glass",
    )

    # Circular base ring (replaces four rectangular base rails)
    tank.visual(
        mesh_from_cadquery(_base_ring_mesh(), "base_ring"),
        origin=Origin(),
        material=black,
        name="base_front_rail",
    )

    # Circular top rim ring (replaces four rectangular top rails)
    tank.visual(
        mesh_from_cadquery(_top_ring_mesh(), "top_ring"),
        origin=Origin(),
        material=black,
        name="top_front_rail",
    )

    # Four vertical reinforcement ribs evenly spaced around the perimeter
    for i in range(4):
        angle = math.pi / 4 + i * math.pi / 2
        rx = (TANK_R + RAIL_W * 0.5) * math.cos(angle)
        ry = (TANK_R + RAIL_W * 0.5) * math.sin(angle)
        tank.visual(
            Cylinder(radius=0.009, length=GLASS_H + RIM_H),
            origin=Origin(xyz=(rx, ry, BASE_H + (GLASS_H + RIM_H) / 2)),
            material=black,
            name=f"rib_{i}",
        )

    # Interleaved hinge knuckles on the rear top rim (tank-hood-tank pattern).
    hinge_knuckle_r = 0.010
    hinge_knuckle_len = 0.036

    # Hinge mounting boss at the rear of the top ring – bridges the ring
    # outer edge to just below the hinge axis so the knuckles sit on top.
    boss_half_w = 0.065
    boss_y_min = TANK_R + RAIL_W - 0.004  # embed slightly into ring top
    boss_y_max = HOOD_HINGE_Y              # stops at hinge axis
    boss_z_bottom = TOP_Z
    boss_z_top = HOOD_HINGE_Z - hinge_knuckle_r  # just below knuckle center
    boss_dy = boss_y_max - boss_y_min
    boss_dz = boss_z_top - boss_z_bottom
    tank.visual(
        Box((boss_half_w * 2, boss_dy, boss_dz)),
        origin=Origin(
            xyz=(0.0, boss_y_min + boss_dy / 2, boss_z_bottom + boss_dz / 2),
        ),
        material=black,
        name="hinge_boss",
    )

    tank_knuckle_xs = (-0.055, 0.055)
    for idx, dx in enumerate(tank_knuckle_xs):
        tank.visual(
            Cylinder(radius=hinge_knuckle_r, length=hinge_knuckle_len),
            origin=Origin(
                xyz=(dx, HOOD_HINGE_Y, HOOD_HINGE_Z),
                rpy=(0.0, math.pi / 2, 0.0),
            ),
            material=black,
            name=f"rear_hinge_knuckle_{idx}",
        )

    # ── substrate ─────────────────────────────────────────────────────
    substrate = model.part("substrate")
    gravel_r = TANK_R - GLASS_T - 0.008
    substrate.visual(
        Cylinder(radius=gravel_r, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=gravel,
        name="gravel_bed",
    )
    for i, (x, y, r, mat) in enumerate(
        (
            (-0.08, -0.06, 0.022, gravel_dark),
            (-0.03, 0.05, 0.018, gravel),
            (0.06, -0.04, 0.024, gravel_dark),
            (0.09, 0.06, 0.020, gravel),
        )
    ):
        substrate.visual(
            Cylinder(radius=r, length=0.010),
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

    # ── submerged internal filter ─────────────────────────────────────
    filter_part = model.part("filter")
    # Housing clips to the inside rear wall of the tank
    filter_y = TANK_R - GLASS_T - 0.035  # just inside the glass
    filter_z = TOP_Z - 0.065              # near the top
    filter_part.visual(
        Box((0.060, 0.055, 0.120)),
        origin=Origin(xyz=(0.0, filter_y, filter_z)),
        material=black,
        name="filter_housing",
    )
    # Mounting clip that bridges the housing back to near the glass wall.
    # Use a narrow clip (half-width 0.008) so box corners stay within the
    # glass inner radius despite the cylinder curvature.
    clip_y_back = TANK_R - GLASS_T
    clip_y_front = filter_y + 0.0275  # back edge of housing
    clip_dy = clip_y_back - clip_y_front
    filter_part.visual(
        Box((0.016, clip_dy, 0.030)),
        origin=Origin(xyz=(0.0, clip_y_front + clip_dy / 2, filter_z + 0.045)),
        material=black,
        name="filter_clip",
    )
    # Intake tube extends down from housing, stops well above the gravel bed.
    # Gravel bed world top ≈ (BASE_H + GLASS_T) + 0.040 ≈ 0.079 m.
    intake_length = 0.120
    intake_z = filter_z - 0.060 - intake_length / 2
    filter_part.visual(
        Cylinder(radius=0.007, length=intake_length),
        origin=Origin(xyz=(0.0, filter_y, intake_z)),
        material=black,
        name="intake_tube",
    )
    # Strainer tip at the bottom of the intake tube (above gravel)
    strainer_z = filter_z - 0.060 - intake_length - 0.030
    filter_part.visual(
        Cylinder(radius=0.0085, length=0.060),
        origin=Origin(xyz=(0.0, filter_y, strainer_z)),
        material=black,
        name="strainer_tip",
    )
    model.articulation(
        "tank_to_filter",
        ArticulationType.FIXED,
        parent=tank,
        child=filter_part,
        origin=Origin(),
    )

    # ── hood (round disc lid) ─────────────────────────────────────────
    hood = model.part("hood")
    hood.visual(
        mesh_from_cadquery(_hood_shell_mesh(), "round_hood_shell"),
        origin=Origin(),
        material=black,
        name="hood_shell",
    )
    hood.visual(
        Box((0.240, 0.025, 0.006)),
        origin=Origin(xyz=(0.0, -0.060, HOOD_BOTTOM_CLEARANCE - 0.003)),
        material=warm_led,
        name="light_diffuser",
    )
    hood.visual(
        Box((0.190, 0.060, 0.004)),
        origin=Origin(xyz=(0.0, -0.100, HOOD_BOTTOM_CLEARANCE + HOOD_T + 0.002)),
        material=dark,
        name="control_panel",
    )
    for idx, (x, mat) in enumerate(((-0.055, red), (-0.025, orange), (0.010, green))):
        hood.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(x, -0.100, HOOD_BOTTOM_CLEARANCE + HOOD_T + 0.006)),
            material=mat,
            name=f"status_button_{idx}",
        )
    # Hood-side hinge knuckles interleave with tank knuckles
    hood_knuckle_xs = (-0.015, 0.015)
    for idx, dx in enumerate(hood_knuckle_xs):
        hood.visual(
            Cylinder(radius=hinge_knuckle_r, length=hinge_knuckle_len),
            origin=Origin(xyz=(dx, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
            material=black,
            name=f"hood_hinge_knuckle_{idx}",
        )
    # Feed-flap hinge leaves on the hood underside
    for idx, dx in enumerate((-0.035, 0.035)):
        hood.visual(
            Cylinder(radius=0.0035, length=0.028),
            origin=Origin(
                xyz=(dx, FEED_HINGE_Y, FEED_HINGE_Z),
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

    # ── feed flap ─────────────────────────────────────────────────────
    feed_flap = model.part("feed_flap")
    feed_flap.visual(
        mesh_from_cadquery(_flap_disc_mesh(), "round_flap_panel"),
        origin=Origin(xyz=(0.0, -FEED_APERTURE_R, 0.0)),
        material=dark,
        name="flap_panel",
    )
    feed_flap.visual(
        Box((0.044, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, -FEED_APERTURE_R * 2 + 0.006, 0.006)),
        material=black,
        name="finger_lip",
    )
    feed_flap.visual(
        Cylinder(radius=0.0035, length=0.036),
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

    # The cylindrical glass wall must exist as a single annular shell
    ctx.check(
        "tank has cylindrical glass wall (not flat panes)",
        any(v.name == "glass_wall" for v in tank.visuals),
        details="Expected a 'glass_wall' visual on tank_frame for the cylindrical column variant.",
    )

    # The filter clip sits inside the hollow glass cylinder (tank interior).
    # The collision system sees the annular glass mesh as a solid volume,
    # but the filter is intentionally in the hollow interior, not inside
    # the glass wall material itself.
    filter_part = object_model.get_part("filter")
    ctx.allow_overlap(
        filter_part,
        tank,
        elem_a="filter_clip",
        elem_b="glass_wall",
        reason=(
            "The filter mounting clip is inside the hollow cylindrical "
            "glass tank interior, not embedded in the glass wall material."
        ),
    )
    # Proof: the clip stays within the glass cylinder footprint on XY
    ctx.expect_within(
        filter_part,
        tank,
        axes="xy",
        inner_elem="filter_clip",
        outer_elem="glass_wall",
        margin=0.002,
        name="filter clip stays inside the glass cylinder footprint",
    )
    # Proof: the filter housing is above the bottom glass
    ctx.expect_gap(
        filter_part,
        tank,
        axis="z",
        min_gap=0.01,
        positive_elem="filter_housing",
        negative_elem="bottom_glass",
        name="filter housing sits above the tank bottom",
    )

    # Hood covers the top ring footprint
    ctx.expect_overlap(
        hood,
        tank,
        axes="xy",
        min_overlap=0.020,
        elem_a="hood_shell",
        elem_b="top_front_rail",
        name="round hood covers the top ring footprint",
    )
    ctx.expect_gap(
        hood,
        tank,
        axis="z",
        min_gap=0.0,
        max_gap=0.020,
        positive_elem="hood_shell",
        negative_elem="top_front_rail",
        name="closed hood sits just above the top rim ring",
    )
    ctx.expect_contact(
        substrate,
        tank,
        elem_a="gravel_bed",
        elem_b="bottom_glass",
        contact_tol=0.001,
        name="round gravel bed rests on the glass bottom disc",
    )

    # Hood hinge opens upward
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

    # Feed flap covers the round aperture at rest (world-space comparison)
    closed_flap = ctx.part_element_world_aabb(feed_flap, elem="flap_panel")
    aperture_world_y = HOOD_HINGE_Y + FEED_APERTURE_Y
    ctx.check(
        "closed feeding flap covers the hood feed aperture",
        closed_flap is not None
        and closed_flap[0][0] < -FEED_APERTURE_R + 0.005
        and closed_flap[1][0] > FEED_APERTURE_R - 0.005
        and closed_flap[0][1] < aperture_world_y - FEED_APERTURE_R + 0.005
        and closed_flap[1][1] > aperture_world_y + FEED_APERTURE_R - 0.005,
        details=(
            f"flap_aabb={closed_flap}, "
            f"aperture_world_y={aperture_world_y}, aperture_r={FEED_APERTURE_R}"
        ),
    )

    # Feed flap hinge opens upward
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
        all("fish" not in p.name and "animal" not in p.name for p in object_model.parts),
        details="No animal/character parts should be authored for this asset.",
    )

    return ctx.report()


object_model = build_object_model()
