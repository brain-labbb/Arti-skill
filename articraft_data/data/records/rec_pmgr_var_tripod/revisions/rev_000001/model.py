from __future__ import annotations

# Spherical playground merry-go-round (orbit spinner) on a tripod stand.
#
# A splayed tripod ground stand (three angled tubular steel legs meeting at a
# central hub) carries a fixed vertical shaft with upper and lower bearing
# journals. A spinning spherical cage of tubular steel hoops (sphere ~1.8 m
# diameter) rides on those journals via collar bearings. The cage has three
# sky-blue latitude rings (largest at the equator), one bright yellow ring
# near the lower-middle, and three full vertical meridian hoops painted in
# red-and-white candy stripes. The whole cage spins freely 360 degrees about
# the vertical shaft axis (continuous revolute joint).

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Key dimensions (meters)
# ---------------------------------------------------------------------------
SPHERE_R = 0.90          # hoop sphere radius (1.8 m diameter)
TUBE_R = 0.020           # steel tube radius (0.04 m diameter)
STRIPE_R = 0.0215        # white stripe sleeve radius (slightly proud of red tube)

CENTER_Z = 1.40          # sphere center height above ground

# Tripod stand
HUB_R = 0.10             # hub outer radius
HUB_H = 0.12             # hub height
HUB_Z = 0.35             # hub center height
HUB_TOP = HUB_Z + HUB_H / 2.0   # 0.41
HUB_BOT = HUB_Z - HUB_H / 2.0   # 0.29

SHAFT_R = 0.045           # central shaft radius
SHAFT_TOP = 2.35          # shaft top height

LEG_TUBE_R = 0.025        # leg tube radius (0.05 m diameter)
LEG_SPREAD = 0.55         # ground-foot distance from center axis
FOOT_R = 0.060            # ground foot pad radius
FOOT_H = 0.025            # ground foot pad height

# Bearing journals
COLLAR_INNER_R = 0.0895   # collar bore radius (light press onto the journal)
COLLAR_OUTER_R = 0.115    # collar outer radius
COLLAR_HALF_H = 0.070     # collar half height
JOURNAL_R = 0.0905        # round bearing journal radius
JOURNAL_LEN = 0.18

# Meridian arcs start/end where their centerline meets the collar wall.
ARC_PHI0 = math.asin(COLLAR_OUTER_R / SPHERE_R)   # polar angle of arc ends
COLLAR_Z = SPHERE_R * math.cos(ARC_PHI0)           # collar centers (local, +/-)

N_MERIDIAN_PLANES = 3     # full hoop planes -> 6 half arcs
N_STRIPE_SEGMENTS = 8     # alternating red/white bands per half arc

# Latitude rings: (name, height above sphere center, material key)
LATITUDE_RINGS = (
    ("ring_upper", 0.52, "sky_blue"),
    ("ring_equator", 0.0, "sky_blue"),
    ("ring_yellow", -0.34, "yellow"),
    ("ring_lower", -0.60, "sky_blue"),
)


def _arc_points(phi_start: float, phi_end: float, n: int) -> list[tuple[float, float, float]]:
    """Points along a meridian circle of radius SPHERE_R in the local XZ plane."""
    pts = []
    for i in range(n):
        phi = phi_start + (phi_end - phi_start) * i / (n - 1)
        pts.append((SPHERE_R * math.sin(phi), 0.0, SPHERE_R * math.cos(phi)))
    return pts


def _leg_points(azimuth: float, n: int = 7) -> list[tuple[float, float, float]]:
    """Points along a tripod leg from hub perimeter down to ground foot."""
    top_r = HUB_R
    top_z = HUB_TOP
    bot_r = LEG_SPREAD
    bot_z = FOOT_H
    pts = []
    for j in range(n):
        t = j / (n - 1)
        r = top_r + t * (bot_r - top_r)
        z = top_z + t * (bot_z - top_z)
        pts.append((r * math.cos(azimuth), r * math.sin(azimuth), z))
    return pts


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="spherical_merry_go_round")

    white_paint = model.material("white_paint", rgba=(0.90, 0.90, 0.87, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.24, 0.25, 0.27, 1.0))
    sky_blue = model.material("sky_blue", rgba=(0.27, 0.60, 0.78, 1.0))
    worn_yellow = model.material("worn_yellow", rgba=(0.90, 0.76, 0.12, 1.0))
    candy_red = model.material("candy_red", rgba=(0.76, 0.13, 0.13, 1.0))
    stripe_white = model.material("stripe_white", rgba=(0.92, 0.90, 0.86, 1.0))
    rust = model.material("rust", rgba=(0.45, 0.27, 0.16, 1.0))

    # ------------------------------------------------------------------
    # Root: splayed tripod ground stand with central hub and shaft
    # ------------------------------------------------------------------
    stand = model.part("tripod_stand")

    # Central hub where legs converge
    stand.visual(
        Cylinder(radius=HUB_R, length=HUB_H),
        origin=Origin(xyz=(0.0, 0.0, HUB_Z)),
        material=white_paint,
        name="hub",
    )
    # Hub top flange plate (wider ring on top of hub cylinder)
    stand.visual(
        Cylinder(radius=HUB_R + 0.020, length=0.015),
        origin=Origin(xyz=(0.0, 0.0, HUB_TOP + 0.005)),
        material=steel_dark,
        name="hub_flange",
    )

    # Central shaft rising from hub through the sphere
    shaft_len = SHAFT_TOP - HUB_BOT
    stand.visual(
        Cylinder(radius=SHAFT_R, length=shaft_len),
        origin=Origin(xyz=(0.0, 0.0, (SHAFT_TOP + HUB_BOT) / 2.0)),
        material=white_paint,
        name="shaft",
    )
    # Shaft collar where shaft exits hub top (weld ring)
    stand.visual(
        Cylinder(radius=0.060, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, HUB_TOP + 0.020)),
        material=steel_dark,
        name="shaft_collar",
    )
    # Shaft cap at top
    stand.visual(
        Cylinder(radius=0.060, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, SHAFT_TOP + 0.010)),
        material=steel_dark,
        name="shaft_cap",
    )

    # Three splayed legs, ground feet, and leg-hub junction clamps
    for i in range(3):
        az = i * 2.0 * math.pi / 3.0

        # Leg tube (hub perimeter down to ground foot)
        leg_mesh = mesh_from_geometry(
            tube_from_spline_points(
                _leg_points(az, n=7),
                radius=LEG_TUBE_R,
                samples_per_segment=4,
                radial_segments=14,
                cap_ends=True,
            ),
            f"leg_{i}",
        )
        stand.visual(
            leg_mesh,
            material=white_paint,
            name=f"leg_{i}",
        )

        # Ground foot pad at leg base
        stand.visual(
            Cylinder(radius=FOOT_R, length=FOOT_H),
            origin=Origin(
                xyz=(LEG_SPREAD * math.cos(az), LEG_SPREAD * math.sin(az), FOOT_H / 2.0)
            ),
            material=steel_dark,
            name=f"foot_{i}",
        )

        # Rusty clamp bracket at leg-hub junction
        clamp_r = HUB_R + 0.010
        stand.visual(
            Box((0.050, 0.040, 0.045)),
            origin=Origin(
                xyz=(clamp_r * math.cos(az), clamp_r * math.sin(az), HUB_TOP - 0.005),
                rpy=(0.0, 0.0, az),
            ),
            material=rust,
            name=f"leg_clamp_{i}",
        )

    # Round bearing journals the cage collars ride on (upper / lower pole)
    for tag, sign in (("upper", 1.0), ("lower", -1.0)):
        stand.visual(
            Cylinder(radius=JOURNAL_R, length=JOURNAL_LEN),
            origin=Origin(xyz=(0.0, 0.0, CENTER_Z + sign * COLLAR_Z)),
            material=steel_dark,
            name=f"journal_{tag}",
        )

    # ------------------------------------------------------------------
    # Child: rigid spinning hoop cage (local frame at sphere center)
    # ------------------------------------------------------------------
    cage = model.part("hoop_cage")

    # Collar bearings: hollow sleeves wrapping the shaft journals
    collar_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(COLLAR_OUTER_R, -COLLAR_HALF_H), (COLLAR_OUTER_R, COLLAR_HALF_H)],
            [(COLLAR_INNER_R, -COLLAR_HALF_H), (COLLAR_INNER_R, COLLAR_HALF_H)],
            segments=48,
        ),
        "bearing_collar",
    )
    cage.visual(
        collar_mesh,
        origin=Origin(xyz=(0.0, 0.0, COLLAR_Z)),
        material=steel_dark,
        name="collar_upper",
    )
    cage.visual(
        collar_mesh,
        origin=Origin(xyz=(0.0, 0.0, -COLLAR_Z)),
        material=steel_dark,
        name="collar_lower",
    )

    # Latitude rings (horizontal tori on the sphere surface)
    for ring_name, height, mat_key in LATITUDE_RINGS:
        ring_r = math.sqrt(SPHERE_R**2 - height**2)
        ring_mesh = mesh_from_geometry(
            TorusGeometry(radius=ring_r, tube=TUBE_R, radial_segments=16, tubular_segments=72),
            ring_name,
        )
        cage.visual(
            ring_mesh,
            origin=Origin(xyz=(0.0, 0.0, height)),
            material=sky_blue if mat_key == "sky_blue" else worn_yellow,
            name=ring_name,
        )

    # Meridian half-arcs (red base tube, pole collar to pole collar)
    arc_mesh = mesh_from_geometry(
        tube_from_spline_points(
            _arc_points(ARC_PHI0, math.pi - ARC_PHI0, 33),
            radius=TUBE_R,
            samples_per_segment=4,
            radial_segments=14,
            cap_ends=True,
        ),
        "meridian_arc",
    )
    # White stripe sleeves on alternating bands -> candy-stripe paint
    delta = (math.pi - 2.0 * ARC_PHI0) / N_STRIPE_SEGMENTS
    stripe_meshes = []
    for j in range(1, N_STRIPE_SEGMENTS, 2):
        stripe_meshes.append(
            mesh_from_geometry(
                tube_from_spline_points(
                    _arc_points(ARC_PHI0 + j * delta, ARC_PHI0 + (j + 1) * delta, 7),
                    radius=STRIPE_R,
                    samples_per_segment=4,
                    radial_segments=14,
                    cap_ends=True,
                ),
                f"meridian_stripe_{j}",
            )
        )
    for k in range(2 * N_MERIDIAN_PLANES):
        yaw = k * math.pi / N_MERIDIAN_PLANES
        cage.visual(
            arc_mesh,
            origin=Origin(rpy=(0.0, 0.0, yaw)),
            material=candy_red,
            name=f"meridian_arc_{k}",
        )
        for s, stripe_mesh in enumerate(stripe_meshes):
            cage.visual(
                stripe_mesh,
                origin=Origin(rpy=(0.0, 0.0, yaw)),
                material=stripe_white,
                name=f"meridian_stripe_{k}_{s}",
            )

    # Rusty clamp brackets where meridians cross the latitude rings
    ring_tags = {"ring_upper": "up", "ring_equator": "eq", "ring_yellow": "yel", "ring_lower": "low"}
    for ring_name, height, _mat in LATITUDE_RINGS:
        ring_r = math.sqrt(SPHERE_R**2 - height**2)
        tag = ring_tags[ring_name]
        for k in range(2 * N_MERIDIAN_PLANES):
            az = k * math.pi / N_MERIDIAN_PLANES
            cage.visual(
                Box((0.055, 0.05, 0.05)),
                origin=Origin(
                    xyz=(ring_r * math.cos(az), ring_r * math.sin(az), height),
                    rpy=(0.0, 0.0, az),
                ),
                material=rust,
                name=f"clamp_{tag}_{k}",
            )

    # ------------------------------------------------------------------
    # Articulation: free 360-degree spin about the vertical shaft axis
    # ------------------------------------------------------------------
    model.articulation(
        "cage_spin",
        ArticulationType.CONTINUOUS,
        parent=stand,
        child=cage,
        origin=Origin(xyz=(0.0, 0.0, CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=200.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("tripod_stand")
    cage = object_model.get_part("hoop_cage")
    spin = object_model.get_articulation("cage_spin")

    # --- Joint identity: continuous spin about the vertical axis ---
    ctx.check(
        "cage spin is continuous about vertical axis",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )

    # --- Cage is concentric with the stand shaft axis ---
    ctx.expect_origin_distance(
        cage, stand, axes="xy", max_dist=0.002, name="cage centered on stand axis"
    )

    # --- Tripod stand has hub, shaft, three legs, and three feet ---
    for name in ("hub", "shaft"):
        aabb = ctx.part_element_world_aabb(stand, elem=name)
        ctx.check(
            f"tripod {name} exists",
            aabb is not None,
            details=f"missing {name} element",
        )
    for i in range(3):
        leg_aabb = ctx.part_element_world_aabb(stand, elem=f"leg_{i}")
        ctx.check(
            f"tripod leg_{i} exists",
            leg_aabb is not None,
            details=f"missing leg_{i}",
        )
        foot_aabb = ctx.part_element_world_aabb(stand, elem=f"foot_{i}")
        ctx.check(
            f"tripod foot_{i} exists",
            foot_aabb is not None,
            details=f"missing foot_{i}",
        )

    # --- Feet sit on the ground ---
    for i in range(3):
        foot_aabb = ctx.part_element_world_aabb(stand, elem=f"foot_{i}")
        ctx.check(
            f"foot_{i} sits on the ground",
            foot_aabb is not None and foot_aabb[0][2] < 0.030,
            details=f"foot_{i} min z={None if foot_aabb is None else foot_aabb[0][2]:.3f}",
        )

    # --- Feet are splayed wider than the hub ---
    foot0_aabb = ctx.part_element_world_aabb(stand, elem="foot_0")
    hub_aabb = ctx.part_element_world_aabb(stand, elem="hub")
    if foot0_aabb is not None and hub_aabb is not None:
        foot_half = max(abs(foot0_aabb[0][0]), abs(foot0_aabb[1][0]))
        hub_half = max(abs(hub_aabb[0][0]), abs(hub_aabb[1][0]))
        ctx.check(
            "feet are splayed wider than the hub",
            foot_half > hub_half + 0.20,
            details=f"foot_half={foot_half:.3f} hub_half={hub_half:.3f}",
        )

    # --- Bearing collar fit (collar captured on journal) ---
    for tag in ("upper", "lower"):
        ctx.allow_overlap(
            cage,
            stand,
            elem_a=f"collar_{tag}",
            elem_b=f"journal_{tag}",
            reason=(
                "The cage bearing collar is intentionally captured on the round "
                "shaft journal so the spinning sphere reads as mounted on its "
                "bearing; the embed is a thin hidden ring inside the collar bore."
            ),
        )
    for tag in ("upper", "lower"):
        ctx.expect_within(
            stand,
            cage,
            axes="xy",
            inner_elem=f"journal_{tag}",
            outer_elem=f"collar_{tag}",
            margin=0.0,
            name=f"{tag} journal sits inside its collar bore",
        )
        ctx.expect_contact(
            cage,
            stand,
            elem_a=f"collar_{tag}",
            elem_b=f"journal_{tag}",
            contact_tol=0.004,
            name=f"{tag} collar rides its journal",
        )

    # --- Hero geometry checks ---
    eq = ctx.part_element_world_aabb(cage, elem="ring_equator")
    up = ctx.part_element_world_aabb(cage, elem="ring_upper")
    low = ctx.part_element_world_aabb(cage, elem="ring_lower")
    yel = ctx.part_element_world_aabb(cage, elem="ring_yellow")

    def _width(aabb):
        return aabb[1][0] - aabb[0][0]

    def _center_z(aabb):
        return (aabb[0][2] + aabb[1][2]) / 2.0

    ctx.check(
        "sphere is about 1.8 m in diameter",
        eq is not None and 1.70 <= _width(eq) <= 1.95,
        details=f"equator width={None if eq is None else _width(eq):.3f}",
    )
    ctx.check(
        "equator ring is the largest latitude ring",
        eq is not None
        and up is not None
        and low is not None
        and yel is not None
        and _width(eq) > _width(up) + 0.10
        and _width(eq) > _width(low) + 0.10
        and _width(eq) > _width(yel) + 0.05,
        details=f"widths eq={_width(eq):.2f} up={_width(up):.2f} low={_width(low):.2f} yel={_width(yel):.2f}",
    )
    ctx.check(
        "yellow ring sits in the lower-middle of the sphere",
        yel is not None and eq is not None and low is not None
        and _center_z(low) < _center_z(yel) < _center_z(eq) - 0.15,
        details=f"z yel={_center_z(yel):.2f} eq={_center_z(eq):.2f} low={_center_z(low):.2f}",
    )

    # --- Meridian hoops span pole to pole ---
    arc = ctx.part_element_world_aabb(cage, elem="meridian_arc_0")
    ctx.check(
        "meridian hoop spans pole to pole",
        arc is not None and (arc[1][2] - arc[0][2]) > 1.70,
        details=f"arc z span={None if arc is None else arc[1][2] - arc[0][2]:.3f}",
    )
    stripe = ctx.part_element_world_aabb(cage, elem="meridian_stripe_0_0")
    ctx.check(
        "candy stripe bands sleeve the meridian hoop",
        stripe is not None,
        details="missing stripe element",
    )

    # --- Cage clears ground; shaft extends above cage ---
    cage_aabb = ctx.part_world_aabb(cage)
    stand_aabb = ctx.part_world_aabb(stand)
    ctx.check(
        "cage clears the ground",
        cage_aabb is not None and cage_aabb[0][2] > 0.10,
        details=f"cage min z={None if cage_aabb is None else cage_aabb[0][2]:.3f}",
    )
    ctx.check(
        "stand shaft tops out near or above cage top",
        stand_aabb is not None
        and cage_aabb is not None
        and stand_aabb[1][2] >= cage_aabb[1][2] - 0.05,
        details=f"stand top={None if stand_aabb is None else stand_aabb[1][2]:.3f} "
                f"cage top={None if cage_aabb is None else cage_aabb[1][2]:.3f}",
    )

    # --- Hub sits between ground and lower cage pole ---
    if hub_aabb is not None and cage_aabb is not None:
        ctx.check(
            "hub is above ground and below the cage",
            hub_aabb[0][2] >= 0.0 and hub_aabb[1][2] < cage_aabb[0][2] + 0.10,
            details=f"hub z=[{hub_aabb[0][2]:.3f},{hub_aabb[1][2]:.3f}] "
                    f"cage min z={cage_aabb[0][2]:.3f}",
        )

    # --- Decisive spin pose: equator clamp swings from +X to +Y ---
    before = ctx.part_element_world_aabb(cage, elem="clamp_eq_0")
    with ctx.pose({spin: math.pi / 2.0}):
        ctx.expect_origin_distance(
            cage, stand, axes="xy", max_dist=0.002, name="spinning cage stays centered"
        )
        after = ctx.part_element_world_aabb(cage, elem="clamp_eq_0")

    def _center_xy(aabb):
        return ((aabb[0][0] + aabb[1][0]) / 2.0, (aabb[0][1] + aabb[1][1]) / 2.0)

    ok = False
    details = "missing clamp element"
    if before is not None and after is not None:
        bx, by = _center_xy(before)
        ax, ay = _center_xy(after)
        ok = bx > 0.70 and abs(by) < 0.05 and abs(ax) < 0.05 and ay > 0.70
        details = f"before=({bx:.2f},{by:.2f}) after=({ax:.2f},{ay:.2f})"
    ctx.check("quarter-turn spin carries the equator clamp from +X to +Y", ok, details=details)

    return ctx.report()


object_model = build_object_model()
