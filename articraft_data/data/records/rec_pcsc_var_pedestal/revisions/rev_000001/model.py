from __future__ import annotations

# Weathered four-seat playground chair-swing carousel.
#
# Layout (meters, Z up, ground at z=0):
# - square base plate + rusty red/white center column (r=0.06, top at z=1.20)
# - rotor: solid bearing-sleeve proxy around the column top, white disc cap with
#   red center, four radial tubular arms (alternating blue/yellow) at z=1.10,
#   eight diagonal tubes forming an X-truss between neighboring arms, and a
#   clevis (two lugs + tangential pivot pin) at each arm tip
# - four hanging seats: a bushing sleeve captured on the pivot pin, two drop
#   straps, a flat rusted sheet-metal platform (~0.45 x 0.35, top ~0.5 above
#   ground) and a low tubular backrest hoop on the inner edge
#
# Articulation:
# - rotor_spin: CONTINUOUS about world Z at the column top
# - seat_swing_0..3: REVOLUTE about the horizontal tangential pin axis,
#   -30..+30 deg; positive q flies the seat radially outward.

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
    mesh_from_geometry,
)

HUB_Z = 1.10  # world height of the rotor bearing frame (arm centerline)
PIVOT_R = 1.38  # radius of the seat pivot pins
PIVOT_DROP = 0.10  # pivot pin sits this far below the arm centerline
SWING = math.pi / 6.0  # +/- 30 deg seat swing


def _radial(theta: float, radius: float, tangent: float = 0.0) -> tuple[float, float]:
    """Rotate arm-local (radial, tangential) offsets into the rotor XY frame."""
    c, s = math.cos(theta), math.sin(theta)
    return (radius * c - tangent * s, radius * s + tangent * c)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="playground_chair_swing_carousel")

    steel_gray = model.material("steel_gray", rgba=(0.45, 0.46, 0.48, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.27, 0.26, 0.25, 1.0))
    weathered_white = model.material("weathered_white", rgba=(0.86, 0.85, 0.80, 1.0))
    rust_red = model.material("rust_red", rgba=(0.62, 0.15, 0.11, 1.0))
    carousel_blue = model.material("carousel_blue", rgba=(0.13, 0.38, 0.72, 1.0))
    carousel_yellow = model.material("carousel_yellow", rgba=(0.83, 0.70, 0.12, 1.0))
    seat_rust = model.material("seat_rust", rgba=(0.46, 0.21, 0.14, 1.0))

    # ---- fixed support: pedestal column with flared circular foot ------------
    column = model.part("support_column")
    # Lathed pedestal: wide circular foot disc flaring up into the column shaft.
    pedestal_profile = [
        (0.40, 0.0),     # outer foot edge on ground
        (0.40, 0.025),   # top of foot rim
        (0.13, 0.08),    # bell-flare taper
        (0.075, 0.14),   # upper transition
        (0.062, 0.17),   # shaft radius approach
    ]
    column.visual(
        mesh_from_geometry(
            LatheGeometry(pedestal_profile, segments=36),
            "pedestal_foot",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=steel_gray,
        name="pedestal_foot",
    )
    # Tall column shaft from flare top (z=0.17) to z=1.20.
    column.visual(
        Cylinder(radius=0.060, length=1.03),
        origin=Origin(xyz=(0.0, 0.0, 0.685)),
        material=weathered_white,
        name="column_shaft",
    )
    column.visual(
        Cylinder(radius=0.090, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.195)),
        material=dark_steel,
        name="column_base_collar",
    )
    column.visual(
        Cylinder(radius=0.064, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 0.42)),
        material=rust_red,
        name="column_band_lower",
    )
    column.visual(
        Cylinder(radius=0.064, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 0.80)),
        material=rust_red,
        name="column_band_upper",
    )

    # ---- rotor: hub sleeve, cap disc, arms, X-truss braces, tip clevises ------
    rotor = model.part("rotor")
    # Solid proxy bearing sleeve capturing the column top (scoped allowance in tests).
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

    arm_mats = [carousel_blue, carousel_yellow, carousel_blue, carousel_yellow]
    for i in range(4):
        theta = i * math.pi / 2.0
        # Arm tube runs from just outside the column (r=0.07) to the tip (r=1.40).
        ax, ay = _radial(theta, 0.735)
        rotor.visual(
            Cylinder(radius=0.030, length=1.33),
            origin=Origin(xyz=(ax, ay, 0.0), rpy=(0.0, math.pi / 2.0, theta)),
            material=arm_mats[i],
            name=f"arm_{i}",
        )
        # Clevis: yoke crosshead under the arm tip carrying two lugs and a
        # tangential pivot pin.
        yx, yy = _radial(theta, PIVOT_R)
        rotor.visual(
            Box((0.07, 0.178, 0.07)),
            origin=Origin(xyz=(yx, yy, -0.025), rpy=(0.0, 0.0, theta)),
            material=rust_red,
            name=f"tip_yoke_{i}",
        )
        for j, t in enumerate((-0.065, 0.065)):
            lx, ly = _radial(theta, PIVOT_R, t)
            rotor.visual(
                Box((0.07, 0.024, 0.16)),
                origin=Origin(xyz=(lx, ly, -0.055), rpy=(0.0, 0.0, theta)),
                material=rust_red,
                name=f"tip_lug_{i}_{j}",
            )
        px, py = _radial(theta, PIVOT_R)
        rotor.visual(
            Cylinder(radius=0.013, length=0.20),
            origin=Origin(xyz=(px, py, -PIVOT_DROP), rpy=(math.pi / 2.0, 0.0, theta)),
            material=dark_steel,
            name=f"pivot_pin_{i}",
        )

    # X-truss: two crossing diagonals between each pair of neighboring arms.
    brace_idx = 0
    for i in range(4):
        th_a = i * math.pi / 2.0
        th_b = ((i + 1) % 4) * math.pi / 2.0
        for (ra, rb), mat in (
            ((0.40, 1.25), arm_mats[(i + 1) % 4]),
            ((1.25, 0.40), arm_mats[i]),
        ):
            x1, y1 = _radial(th_a, ra)
            x2, y2 = _radial(th_b, rb)
            length = math.hypot(x2 - x1, y2 - y1)
            yaw = math.atan2(y2 - y1, x2 - x1)
            rotor.visual(
                Cylinder(radius=0.018, length=length),
                origin=Origin(
                    xyz=((x1 + x2) / 2.0, (y1 + y2) / 2.0, 0.0),
                    rpy=(0.0, math.pi / 2.0, yaw),
                ),
                material=mat,
                name=f"brace_{brace_idx}",
            )
            brace_idx += 1

    model.articulation(
        "rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=column,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, HUB_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=500.0, velocity=3.0),
    )

    # ---- four hanging chair seats --------------------------------------------
    # Seat local frame: origin at the pivot pin, +X radially outward, +Y tangent.
    for i in range(4):
        theta = i * math.pi / 2.0
        seat = model.part(f"seat_{i}")
        # Bushing-sleeve proxy captured on the pivot pin (scoped allowance in tests).
        seat.visual(
            Cylinder(radius=0.028, length=0.094),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name="hanger_sleeve",
        )
        for j, t in enumerate((-0.033, 0.033)):
            seat.visual(
                Cylinder(radius=0.014, length=0.50),
                origin=Origin(xyz=(0.0, t, -0.266)),
                material=rust_red,
                name=f"hanger_strap_{j}",
            )
        seat.visual(
            Box((0.35, 0.45, 0.022)),
            origin=Origin(xyz=(0.04, 0.0, -0.511)),
            material=seat_rust,
            name="platform",
        )
        for j, t in enumerate((-0.19, 0.19)):
            seat.visual(
                Cylinder(radius=0.012, length=0.22),
                origin=Origin(xyz=(-0.115, t, -0.405)),
                material=rust_red,
                name=f"backrest_post_{j}",
            )
        seat.visual(
            Cylinder(radius=0.012, length=0.42),
            origin=Origin(xyz=(-0.115, 0.0, -0.295), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rust_red,
            name="backrest_rail",
        )
        px, py = _radial(theta, PIVOT_R)
        model.articulation(
            f"seat_swing_{i}",
            ArticulationType.REVOLUTE,
            parent=rotor,
            child=seat,
            origin=Origin(xyz=(px, py, -PIVOT_DROP), rpy=(0.0, 0.0, theta)),
            # Tangential axis; positive q swings the hanging seat radially outward.
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=120.0, velocity=4.0, lower=-SWING, upper=SWING),
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

    # Intentional captured-shaft / captured-pin proxies.
    ctx.allow_overlap(
        rotor,
        column,
        elem_a="hub_sleeve",
        elem_b="column_shaft",
        reason="rotor bearing sleeve is a solid proxy intentionally capturing the fixed column shaft",
    )
    for i, seat in enumerate(seats):
        ctx.allow_overlap(
            rotor,
            seat,
            elem_a=f"pivot_pin_{i}",
            elem_b="hanger_sleeve",
            reason="seat hanger sleeve is a solid bushing proxy capturing the arm-tip pivot pin",
        )

    # Pedestal foot is a circular flared base (not boxy), centered at ground.
    foot = column.get_visual("pedestal_foot")
    ctx.check("pedestal foot visual exists", foot is not None, details="missing pedestal_foot")
    foot_aabb = ctx.part_element_world_aabb(column, elem="pedestal_foot")
    if foot_aabb is not None:
        foot_dx = foot_aabb[1][0] - foot_aabb[0][0]
        foot_dy = foot_aabb[1][1] - foot_aabb[0][1]
        ctx.check(
            "pedestal foot has a circular (not square) footprint",
            abs(foot_dx - foot_dy) < 0.05 and foot_dx > 0.60,
            details=f"foot_dx={foot_dx:.3f}, foot_dy={foot_dy:.3f}",
        )
        ctx.check(
            "pedestal foot is wide enough for stability",
            foot_dx > 0.70,
            details=f"foot_dx={foot_dx:.3f}",
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

    # Four alternating blue/yellow arms with an eight-tube X-truss.
    arm_names = [v.name for v in rotor.visuals if v.name and v.name.startswith("arm_")]
    ctx.check("four radial tubular arms present", len(arm_names) == 4, details=f"arms={arm_names}")

    def _mat_name(visual) -> str:
        material = visual.material
        return str(getattr(material, "name", material))

    ctx.check(
        "arm paint alternates blue and yellow",
        _mat_name(rotor.get_visual("arm_0")) == "carousel_blue"
        and _mat_name(rotor.get_visual("arm_1")) == "carousel_yellow"
        and _mat_name(rotor.get_visual("arm_2")) == "carousel_blue"
        and _mat_name(rotor.get_visual("arm_3")) == "carousel_yellow",
        details="expected blue/yellow/blue/yellow arm materials",
    )
    brace_count = sum(1 for v in rotor.visuals if v.name and v.name.startswith("brace_"))
    ctx.check(
        "eight diagonal tubes form X-trusses between neighboring arms",
        brace_count == 8,
        details=f"brace_count={brace_count}",
    )

    # Joint contract: continuous Z spin, +/-30 deg revolute seat swings.
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

    # Each seat hangs at the arm tip, seated on its pin, ~0.5 m above ground.
    for i, seat in enumerate(seats):
        ctx.expect_origin_distance(
            seat,
            column,
            axes="xy",
            min_dist=1.30,
            max_dist=1.46,
            name=f"seat_{i} hangs at the arm-tip radius",
        )
        ctx.expect_overlap(
            seat,
            rotor,
            axes="xy",
            elem_a="hanger_sleeve",
            elem_b=f"pivot_pin_{i}",
            min_overlap=0.01,
            name=f"seat_{i} sleeve is seated on its pivot pin",
        )
        plat = ctx.part_element_world_aabb(seat, elem="platform")
        ctx.check(
            f"seat_{i} platform is suspended about 0.5 m above ground",
            plat is not None and 0.45 <= plat[1][2] <= 0.55 and plat[0][2] > 0.40,
            details=f"platform aabb={plat}",
        )

    # Backrest hoop rises above the platform on the inner (column-facing) edge.
    plat0 = ctx.part_element_world_aabb(seats[0], elem="platform")
    rail0 = ctx.part_element_world_aabb(seats[0], elem="backrest_rail")
    ctx.check(
        "low backrest hoop rises above the platform inner edge",
        plat0 is not None
        and rail0 is not None
        and rail0[0][2] > plat0[1][2] + 0.10
        and _center(rail0, 0) < _center(plat0, 0) - 0.05,
        details=f"platform={plat0}, rail={rail0}",
    )

    # Overall scale: seat outer edge near 1.6 m radius (~3 m diameter), top ~1.3 m.
    rotor_aabb = ctx.part_world_aabb(rotor)
    ctx.check(
        "carousel spans about 3 m and stands about 1.3 m tall",
        plat0 is not None
        and rotor_aabb is not None
        and 1.50 <= plat0[1][0] <= 1.70
        and 1.25 <= rotor_aabb[1][2] <= 1.35,
        details=f"seat outer x={None if plat0 is None else plat0[1][0]}, rotor top={None if rotor_aabb is None else rotor_aabb[1][2]}",
    )

    # Decisive pose checks: positive swing flies seat_0 outward and upward.
    with ctx.pose({swings[0]: SWING}):
        plat_out = ctx.part_element_world_aabb(seats[0], elem="platform")
    ctx.check(
        "positive seat swing flies the platform outward and upward",
        plat0 is not None
        and plat_out is not None
        and _center(plat_out, 0) > _center(plat0, 0) + 0.15
        and _center(plat_out, 2) > _center(plat0, 2) + 0.04,
        details=f"rest={plat0}, swung_out={plat_out}",
    )
    with ctx.pose({swings[0]: -SWING}):
        plat_in = ctx.part_element_world_aabb(seats[0], elem="platform")
    ctx.check(
        "negative seat swing tucks the platform inward",
        plat0 is not None
        and plat_in is not None
        and _center(plat_in, 0) < _center(plat0, 0) - 0.15,
        details=f"rest={plat0}, swung_in={plat_in}",
    )

    # Quarter-turn spin carries seat_0 from +X to +Y.
    with ctx.pose({spin: math.pi / 2.0}):
        plat_spun = ctx.part_element_world_aabb(seats[0], elem="platform")
    ctx.check(
        "quarter-turn spin carries seat_0 a quarter circle around the column",
        plat_spun is not None
        and _center(plat_spun, 1) > 1.2
        and abs(_center(plat_spun, 0)) < 0.3,
        details=f"spun platform aabb={plat_spun}",
    )

    return ctx.report()


object_model = build_object_model()
