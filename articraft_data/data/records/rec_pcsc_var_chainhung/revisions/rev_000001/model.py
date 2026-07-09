from __future__ import annotations

# Weathered four-seat playground chair-swing carousel (chain-hung variant).
#
# Instead of rigid radial arms, each seat hangs as a pendulum from a pair of
# overhead chain rods dropping from a raised square ring. The ring is carried
# by four diagonal struts rising from the central hub.
#
# Layout (meters, Z up, ground at z=0):
# - square base plate + rusty red/white center column (r=0.06, top at z≈1.20)
# - rotor: bearing-sleeve proxy around column top, white disc cap with red
#   center, four diagonal struts (alternating blue/yellow) rising outward to a
#   raised square ring (r≈1.25, world z≈1.55), four ring tube segments
# - four chain-hung seats: each suspended by a pair of slender chain rods
#   dropping from a ring vertex, with a flat rusted sheet-metal platform
#   (~0.45×0.35, top ≈0.55 above ground) and a low tubular backrest hoop
#
# Articulation:
# - rotor_spin: CONTINUOUS about world Z at the column top
# - seat_swing_0..3: REVOLUTE about the horizontal tangential axis at the ring
#   vertex, ±30°; positive q flies the seat radially outward.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

HUB_Z = 1.10  # world height of the rotor bearing frame
RING_R = 1.25  # ring vertex radius
RING_Z_LOCAL = 0.45  # ring height in rotor-local frame (world z = 1.55)
CHAIN_LENGTH = 0.95  # chain rod length
CHAIN_SPREAD = 0.24  # tangential separation of the two chain rods
CHAIN_DROP = 0.05  # chains start this far below the ring vertex
SWING = math.pi / 6.0  # ±30° seat swing


def _radial(theta: float, radius: float, tangent: float = 0.0) -> tuple[float, float]:
    """Rotate (radial, tangential) offsets into the rotor XY frame."""
    c, s = math.cos(theta), math.sin(theta)
    return (radius * c - tangent * s, radius * s + tangent * c)


def _build_seat_visuals(seat, dark_steel, rust_red, seat_rust):
    """Add hanger bracket, chain rods, platform, and backrest to a seat part.

    Seat local frame: origin at the ring vertex (joint origin), +X radially
    outward, +Y tangential, +Z up.
    """
    # Hanger bracket: vertical plate that wraps around the ring tube at the top
    # and extends down to meet the chain rods. The bracket-ring overlap is a
    # scoped intentional embedding (swing hook capturing the overhead bar).
    bracket_h = CHAIN_DROP + 0.03  # reaches 1 cm above ring center
    bracket_cz = 0.01 - bracket_h / 2.0
    seat.visual(
        Box((0.020, CHAIN_SPREAD + 0.02, bracket_h)),
        origin=Origin(xyz=(0.0, 0.0, bracket_cz)),
        material=dark_steel,
        name="hanger_bracket",
    )

    chain_cz = -(CHAIN_DROP + CHAIN_LENGTH / 2.0)
    for j, t in enumerate((-CHAIN_SPREAD / 2.0, CHAIN_SPREAD / 2.0)):
        seat.visual(
            Cylinder(radius=0.008, length=CHAIN_LENGTH),
            origin=Origin(xyz=(0.0, t, chain_cz)),
            material=dark_steel,
            name=f"chain_{j}",
        )

    plat_cz = -(CHAIN_DROP + CHAIN_LENGTH + 0.011)
    seat.visual(
        Box((0.35, 0.45, 0.022)),
        origin=Origin(xyz=(0.04, 0.0, plat_cz)),
        material=seat_rust,
        name="platform",
    )

    post_cz = plat_cz + 0.011 + 0.11
    rail_cz = plat_cz + 0.011 + 0.22
    for j, t in enumerate((-0.19, 0.19)):
        seat.visual(
            Cylinder(radius=0.012, length=0.22),
            origin=Origin(xyz=(-0.115, t, post_cz)),
            material=rust_red,
            name=f"backrest_post_{j}",
        )
    seat.visual(
        Cylinder(radius=0.012, length=0.42),
        origin=Origin(xyz=(-0.115, 0.0, rail_cz), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rust_red,
        name="backrest_rail",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="playground_chair_swing_carousel")

    steel_gray = model.material("steel_gray", rgba=(0.45, 0.46, 0.48, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.27, 0.26, 0.25, 1.0))
    weathered_white = model.material("weathered_white", rgba=(0.86, 0.85, 0.80, 1.0))
    rust_red = model.material("rust_red", rgba=(0.62, 0.15, 0.11, 1.0))
    carousel_blue = model.material("carousel_blue", rgba=(0.13, 0.38, 0.72, 1.0))
    carousel_yellow = model.material("carousel_yellow", rgba=(0.83, 0.70, 0.12, 1.0))
    seat_rust = model.material("seat_rust", rgba=(0.46, 0.21, 0.14, 1.0))

    # ---- fixed support: base plate + striped center column -------------------
    column = model.part("support_column")
    column.visual(
        Box((0.55, 0.55, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=steel_gray,
        name="base_plate",
    )
    for k, (bx, by) in enumerate(
        ((0.20, 0.20), (-0.20, 0.20), (-0.20, -0.20), (0.20, -0.20))
    ):
        column.visual(
            Cylinder(radius=0.012, length=0.03),
            origin=Origin(xyz=(bx, by, 0.055)),
            material=dark_steel,
            name=f"anchor_bolt_{k}",
        )
    column.visual(
        Cylinder(radius=0.060, length=1.17),
        origin=Origin(xyz=(0.0, 0.0, 0.615)),
        material=weathered_white,
        name="column_shaft",
    )
    column.visual(
        Cylinder(radius=0.090, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=dark_steel,
        name="column_base_collar",
    )
    column.visual(
        Cylinder(radius=0.064, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 0.32)),
        material=rust_red,
        name="column_band_lower",
    )
    column.visual(
        Cylinder(radius=0.064, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 0.72)),
        material=rust_red,
        name="column_band_upper",
    )

    # ---- rotor: hub sleeve, cap disc, struts, square ring --------------------
    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.085, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, -0.03)),
        material=rust_red,
        name="hub_sleeve",
    )
    rotor.visual(
        Cylinder(radius=0.17, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material=weathered_white,
        name="hub_cap_disc",
    )
    rotor.visual(
        Cylinder(radius=0.055, length=0.015),
        origin=Origin(xyz=(0.0, 0.0, 0.1775)),
        material=rust_red,
        name="hub_cap_center",
    )

    # Four diagonal struts from hub to ring vertices (alternating blue/yellow).
    strut_mats = [carousel_blue, carousel_yellow, carousel_blue, carousel_yellow]
    for i in range(4):
        theta = i * math.pi / 2.0
        x1, y1 = _radial(theta, 0.12)
        z1 = 0.17
        x2, y2 = _radial(theta, RING_R)
        z2 = RING_Z_LOCAL
        dx, dy, dz = x2 - x1, y2 - y1, z2 - z1
        length = math.sqrt(dx * dx + dy * dy + dz * dz)
        mx, my, mz = (x1 + x2) / 2.0, (y1 + y2) / 2.0, (z1 + z2) / 2.0
        # Cylinder default axis is +Z; tilt away from Z then rotate by yaw.
        tilt = math.acos(max(-1.0, min(1.0, dz / length)))
        yaw = math.atan2(dy, dx)
        rotor.visual(
            Cylinder(radius=0.025, length=length),
            origin=Origin(xyz=(mx, my, mz), rpy=(0.0, tilt, yaw)),
            material=strut_mats[i],
            name=f"strut_{i}",
        )

    # Square ring: four tube segments connecting adjacent ring vertices.
    for i in range(4):
        th_a = i * math.pi / 2.0
        th_b = ((i + 1) % 4) * math.pi / 2.0
        x1, y1 = _radial(th_a, RING_R)
        x2, y2 = _radial(th_b, RING_R)
        seg_dx, seg_dy = x2 - x1, y2 - y1
        seg_length = math.hypot(seg_dx, seg_dy)
        seg_yaw = math.atan2(seg_dy, seg_dx)
        rotor.visual(
            Cylinder(radius=0.022, length=seg_length),
            origin=Origin(
                xyz=((x1 + x2) / 2.0, (y1 + y2) / 2.0, RING_Z_LOCAL),
                rpy=(0.0, math.pi / 2.0, seg_yaw),
            ),
            material=rust_red,
            name=f"ring_seg_{i}",
        )

    model.articulation(
        "rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=column,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, HUB_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=500.0, velocity=3.0),
    )

    # ---- four chain-hung seats -----------------------------------------------
    for i in range(4):
        theta = i * math.pi / 2.0
        seat = model.part(f"seat_{i}")
        _build_seat_visuals(seat, dark_steel, rust_red, seat_rust)

        jx, jy = _radial(theta, RING_R)
        model.articulation(
            f"seat_swing_{i}",
            ArticulationType.REVOLUTE,
            parent=rotor,
            child=seat,
            origin=Origin(xyz=(jx, jy, RING_Z_LOCAL), rpy=(0.0, 0.0, theta)),
            # Tangential axis; positive q swings the hanging seat radially outward.
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=120.0, velocity=4.0, lower=-SWING, upper=SWING
            ),
        )

    return model


def _center(aabb: tuple, axis: int) -> float:
    return (aabb[0][axis] + aabb[1][axis]) / 2.0


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    column = object_model.get_part("support_column")
    rotor = object_model.get_part("rotor")
    seats = [object_model.get_part(f"seat_{i}") for i in range(4)]
    spin = object_model.get_articulation("rotor_spin")
    swings = [object_model.get_articulation(f"seat_swing_{i}") for i in range(4)]

    # Intentional captured-shaft proxy: hub sleeve wraps column shaft.
    ctx.allow_overlap(
        rotor,
        column,
        elem_a="hub_sleeve",
        elem_b="column_shaft",
        reason="rotor bearing sleeve is a solid proxy intentionally capturing the fixed column shaft",
    )

    # Each seat hanger bracket captures the ring tube at its vertex, like a
    # swing hook wrapping around an overhead bar. Two ring segments meet at
    # each vertex, so each seat needs two scoped allowances.
    for i, seat in enumerate(seats):
        seg_end = f"ring_seg_{(i + 3) % 4}"
        seg_start = f"ring_seg_{i}"
        ctx.allow_overlap(
            rotor,
            seat,
            elem_a=seg_end,
            elem_b="hanger_bracket",
            reason=f"seat_{i} hanger bracket captures the ring tube at vertex {i} (segment endpoint)",
        )
        ctx.allow_overlap(
            rotor,
            seat,
            elem_a=seg_start,
            elem_b="hanger_bracket",
            reason=f"seat_{i} hanger bracket captures the ring tube at vertex {i} (segment startpoint)",
        )
        ctx.allow_overlap(
            rotor,
            seat,
            elem_a=f"strut_{i}",
            elem_b="hanger_bracket",
            reason=f"seat_{i} hanger bracket wraps around the strut-ring junction at vertex {i}",
        )

    # Hub seats on the column and the white cap disc tops the assembly.
    ctx.expect_within(
        column,
        rotor,
        axes="xy",
        inner_elem="column_shaft",
        outer_elem="hub_sleeve",
        name="hub sleeve stays centered around the column shaft",
    )
    ctx.expect_gap(
        rotor,
        column,
        axis="z",
        positive_elem="hub_cap_disc",
        negative_elem="column_shaft",
        min_gap=0.0,
        max_gap=0.06,
        name="white cap disc sits just above the column top",
    )

    # Overhead ring structure: four ring segments and four supporting struts.
    ring_segs = [v.name for v in rotor.visuals if v.name and v.name.startswith("ring_seg_")]
    ctx.check(
        "four ring segments form the overhead square ring",
        len(ring_segs) == 4,
        details=f"ring_segments={ring_segs}",
    )
    strut_names = [v.name for v in rotor.visuals if v.name and v.name.startswith("strut_")]
    ctx.check(
        "four diagonal struts support the overhead ring",
        len(strut_names) == 4,
        details=f"struts={strut_names}",
    )

    def _mat_name(visual) -> str:
        m = visual.material
        return str(getattr(m, "name", m))

    ctx.check(
        "strut paint alternates blue and yellow",
        _mat_name(rotor.get_visual("strut_0")) == "carousel_blue"
        and _mat_name(rotor.get_visual("strut_1")) == "carousel_yellow"
        and _mat_name(rotor.get_visual("strut_2")) == "carousel_blue"
        and _mat_name(rotor.get_visual("strut_3")) == "carousel_yellow",
        details="expected blue/yellow/blue/yellow strut materials",
    )

    # No rigid radial arms remain; seats hang from chains instead.
    arm_names = [v.name for v in rotor.visuals if v.name and v.name.startswith("arm_")]
    ctx.check(
        "no rigid radial arms on the rotor (chain-hung variant)",
        len(arm_names) == 0,
        details=f"unexpected arms={arm_names}",
    )

    # Each seat hangs from a pair of overhead chain rods.
    for i, seat in enumerate(seats):
        chain_names = [v.name for v in seat.visuals if v.name and v.name.startswith("chain_")]
        ctx.check(
            f"seat_{i} has two overhead chain hangers",
            len(chain_names) == 2,
            details=f"chains={chain_names}",
        )

    # Joint contract: continuous Z spin, ±30° revolute seat swings.
    ctx.check(
        "rotor spin is a continuous joint about vertical Z",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    for i, swing in enumerate(swings):
        limits = swing.motion_limits
        ctx.check(
            f"seat_swing_{i} is a +/-30 deg revolute pivot",
            swing.articulation_type == ArticulationType.REVOLUTE
            and limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and abs(limits.lower + SWING) < 1e-6
            and abs(limits.upper - SWING) < 1e-6,
            details=f"type={swing.articulation_type}, limits={limits}",
        )

    # Each seat hangs below the ring vertex at the correct radius and height.
    # The hanger bracket captures the ring tube (proof of the hook-on-bar mount).
    for i, seat in enumerate(seats):
        ctx.expect_overlap(
            rotor,
            seat,
            axes="z",
            elem_a=f"ring_seg_{i}",
            elem_b="hanger_bracket",
            min_overlap=0.005,
            name=f"seat_{i} hanger bracket captures the ring tube at its vertex",
        )
        ctx.expect_origin_distance(
            seat,
            column,
            axes="xy",
            min_dist=1.15,
            max_dist=1.35,
            name=f"seat_{i} hangs below the ring vertex radius",
        )
        plat = ctx.part_element_world_aabb(seat, elem="platform")
        ctx.check(
            f"seat_{i} platform is suspended about 0.5 m above ground",
            plat is not None and 0.40 <= plat[1][2] <= 0.65 and plat[0][2] > 0.35,
            details=f"platform aabb={plat}",
        )

    # Backrest hoop rises above the platform on the inner (column-facing) edge.
    plat0 = ctx.part_element_world_aabb(seats[0], elem="platform")
    rail0 = ctx.part_element_world_aabb(seats[0], elem="backrest_rail")
    ctx.check(
        "low backrest hoop rises above the platform inner edge",
        plat0 is not None
        and rail0 is not None
        and rail0[0][2] > plat0[1][2] + 0.05,
        details=f"platform={plat0}, rail={rail0}",
    )

    # Overall scale: ring reaches about 1.55 m above ground.
    rotor_aabb = ctx.part_world_aabb(rotor)
    ctx.check(
        "carousel overhead ring stands about 1.5 m tall",
        rotor_aabb is not None and 1.45 <= rotor_aabb[1][2] <= 1.65,
        details=f"rotor top={None if rotor_aabb is None else rotor_aabb[1][2]}",
    )

    # Decisive pose: positive swing flies seat_0 outward.
    with ctx.pose({swings[0]: SWING}):
        plat_out = ctx.part_element_world_aabb(seats[0], elem="platform")
    ctx.check(
        "positive seat swing flies the platform outward",
        plat0 is not None
        and plat_out is not None
        and _center(plat_out, 0) > _center(plat0, 0) + 0.10,
        details=f"rest={plat0}, swung_out={plat_out}",
    )

    # Decisive pose: negative swing tucks seat_0 inward.
    with ctx.pose({swings[0]: -SWING}):
        plat_in = ctx.part_element_world_aabb(seats[0], elem="platform")
    ctx.check(
        "negative seat swing tucks the platform inward",
        plat0 is not None
        and plat_in is not None
        and _center(plat_in, 0) < _center(plat0, 0) - 0.10,
        details=f"rest={plat0}, swung_in={plat_in}",
    )

    # Quarter-turn spin carries seat_0 from +X to +Y.
    with ctx.pose({spin: math.pi / 2.0}):
        plat_spun = ctx.part_element_world_aabb(seats[0], elem="platform")
    ctx.check(
        "quarter-turn spin carries seat_0 a quarter circle around the column",
        plat_spun is not None
        and _center(plat_spun, 1) > 1.0
        and abs(_center(plat_spun, 0)) < 0.3,
        details=f"spun platform aabb={plat_spun}",
    )

    return ctx.report()


object_model = build_object_model()
