from __future__ import annotations

# Weathered four-seat playground chair-swing carousel — BUCKET SEAT variant.
#
# Layout (meters, Z up, ground at z=0):
# - square base plate + rusty red/white center column (r=0.06, top at z=1.20)
# - rotor: solid bearing-sleeve proxy around the column top, white disc cap with
#   red center, four radial tubular arms (alternating blue/yellow) at z=1.10,
#   eight diagonal tubes forming an X-truss between neighboring arms, and a
#   clevis (two lugs + tangential pivot pin) at each arm tip
# - four hanging bucket seats: a bushing sleeve captured on the pivot pin, two
#   drop straps, a hanger mounting plate, a deep CadQuery bucket shell (~0.30 x
#   0.36 x 0.36, curved high sidewalls wrapping the rider, front entry opening),
#   a cylindrical safety bar across the lap, and two mounting brackets
#
# Articulation:
# - rotor_spin: CONTINUOUS about world Z at the column top
# - seat_swing_0..3: REVOLUTE about the horizontal tangential pin axis,
#   -30..+30 deg; positive q flies the seat radially outward.

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

HUB_Z = 1.10  # world height of the rotor bearing frame (arm centerline)
PIVOT_R = 1.38  # radius of the seat pivot pins
PIVOT_DROP = 0.10  # pivot pin sits this far below the arm centerline
SWING = math.pi / 6.0  # +/- 30 deg seat swing

# Bucket shell dimensions (local frame: origin at center-bottom, +X outward, +Z up)
BUCKET_DX = 0.30  # radial depth
BUCKET_DY = 0.36  # tangential width
BUCKET_DZ = 0.36  # vertical height
BUCKET_WALL = 0.022
BUCKET_FLOOR = 0.028
BUCKET_DROP = 0.50  # seat-frame z of bucket bottom (below pivot)


def _radial(theta: float, radius: float, tangent: float = 0.0) -> tuple[float, float]:
    """Rotate arm-local (radial, tangential) offsets into the rotor XY frame."""
    c, s = math.cos(theta), math.sin(theta)
    return (radius * c - tangent * s, radius * s + tangent * c)


def _make_bucket_shell() -> cq.Workplane:
    """Deep bucket seat shell with high sidewalls and a front entry opening.

    Built via CadQuery booleans: outer filleted box minus interior cavity minus
    front cutout.  The result is a U-shaped hollow shell open at the +X face
    between the two tall sidewalls.
    """
    dx, dy, dz = BUCKET_DX, BUCKET_DY, BUCKET_DZ
    wall = BUCKET_WALL
    floor_t = BUCKET_FLOOR

    # Outer rounded shell
    outer = (
        cq.Workplane("XY")
        .box(dx, dy, dz)
        .translate((0.0, 0.0, dz / 2.0))
        .edges("|Z")
        .fillet(0.04)
    )

    # Interior cavity (slightly shifted toward -X for a thicker back wall)
    cav_dx = dx - 2 * wall - 0.006
    cav_dy = dy - 2 * wall
    cav_dz = dz - floor_t
    cavity = (
        cq.Workplane("XY")
        .box(cav_dx, cav_dy, cav_dz)
        .translate((-0.003, 0.0, floor_t + cav_dz / 2.0))
        .edges("|Z")
        .fillet(0.025)
    )

    # Front entry opening between the high sidewalls
    open_dx = 0.14
    open_dy = cav_dy + 0.002
    open_dz = cav_dz - 0.02
    front_cut = (
        cq.Workplane("XY")
        .box(open_dx, open_dy, open_dz)
        .translate((dx / 2.0 - open_dx / 2.0 + wall + 0.01, 0.0, floor_t + open_dz / 2.0))
    )

    return outer.cut(cavity).cut(front_cut)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="playground_chair_swing_carousel")

    steel_gray = model.material("steel_gray", rgba=(0.45, 0.46, 0.48, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.27, 0.26, 0.25, 1.0))
    weathered_white = model.material("weathered_white", rgba=(0.86, 0.85, 0.80, 1.0))
    rust_red = model.material("rust_red", rgba=(0.62, 0.15, 0.11, 1.0))
    carousel_blue = model.material("carousel_blue", rgba=(0.13, 0.38, 0.72, 1.0))
    carousel_yellow = model.material("carousel_yellow", rgba=(0.83, 0.70, 0.12, 1.0))
    seat_rust = model.material("seat_rust", rgba=(0.46, 0.21, 0.14, 1.0))
    faded_teal = model.material("faded_teal", rgba=(0.22, 0.42, 0.40, 1.0))

    # ---- fixed support: base plate + striped center column -------------------
    column = model.part("support_column")
    column.visual(
        Box((0.55, 0.55, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=steel_gray,
        name="base_plate",
    )
    for k, (bx, by) in enumerate(((0.20, 0.20), (-0.20, 0.20), (-0.20, -0.20), (0.20, -0.20))):
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

    # ---- four hanging bucket seats -------------------------------------------
    # Build the shared CadQuery bucket shell mesh once.
    bucket_mesh = mesh_from_cadquery(_make_bucket_shell(), "bucket_shell")

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
        # Two drop straps from the pivot down to the bucket floor region.
        for j, t in enumerate((-0.033, 0.033)):
            seat.visual(
                Cylinder(radius=0.014, length=0.50),
                origin=Origin(xyz=(0.0, t, -0.266)),
                material=rust_red,
                name=f"hanger_strap_{j}",
            )
        # Mounting plate bridging the straps to the bucket shell floor.
        seat.visual(
            Box((0.10, 0.12, 0.03)),
            origin=Origin(xyz=(0.0, 0.0, -BUCKET_DROP + 0.01)),
            material=rust_red,
            name="hanger_mount",
        )
        # CadQuery bucket shell: deep curved walls, front entry opening.
        seat.visual(
            bucket_mesh,
            origin=Origin(xyz=(0.04, 0.0, -BUCKET_DROP)),
            material=seat_rust,
            name="bucket_shell",
        )
        # Safety bar across the front opening at lap height.
        seat.visual(
            Cylinder(radius=0.013, length=0.34),
            origin=Origin(xyz=(0.14, 0.0, -0.32), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name="safety_bar",
        )
        # Two mounting brackets where the safety bar meets the sidewalls.
        for j, side in enumerate((-1, 1)):
            seat.visual(
                Box((0.04, 0.03, 0.05)),
                origin=Origin(xyz=(0.14, side * 0.165, -0.32)),
                material=faded_teal,
                name=f"bar_bracket_{j}",
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

    # ---- bucket seat checks --------------------------------------------------
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

        # Each seat has a CadQuery bucket shell and a safety bar.
        shell_names = [v.name for v in seat.visuals if v.name == "bucket_shell"]
        bar_names = [v.name for v in seat.visuals if v.name == "safety_bar"]
        ctx.check(
            f"seat_{i} has a bucket shell visual",
            len(shell_names) == 1,
            details=f"bucket_shell visuals={shell_names}",
        )
        ctx.check(
            f"seat_{i} has a safety bar visual",
            len(bar_names) == 1,
            details=f"safety_bar visuals={bar_names}",
        )

    # Bucket shell sidewalls are tall (height > 0.30 m).
    shell0_aabb = ctx.part_element_world_aabb(seats[0], elem="bucket_shell")
    ctx.check(
        "bucket shell has tall curved sidewalls (height > 0.30 m)",
        shell0_aabb is not None and (shell0_aabb[1][2] - shell0_aabb[0][2]) > 0.30,
        details=f"bucket_shell aabb={shell0_aabb}",
    )

    # Bucket shell bottom is about 0.50 m above ground.
    ctx.check(
        "bucket seat bottom is suspended about 0.5 m above ground",
        shell0_aabb is not None and 0.40 <= shell0_aabb[0][2] <= 0.55,
        details=f"bucket_shell bottom z={None if shell0_aabb is None else shell0_aabb[0][2]}",
    )

    # Safety bar spans the bucket width (Y extent > 0.28 m).
    bar0_aabb = ctx.part_element_world_aabb(seats[0], elem="safety_bar")
    ctx.check(
        "safety bar spans across the bucket seat width",
        bar0_aabb is not None and (bar0_aabb[1][1] - bar0_aabb[0][1]) > 0.28,
        details=f"safety_bar aabb={bar0_aabb}",
    )

    # Safety bar is forward of the bucket center and above the floor.
    ctx.check(
        "safety bar is at the front opening above the bucket floor",
        bar0_aabb is not None
        and shell0_aabb is not None
        and _center(bar0_aabb, 0) > _center(shell0_aabb, 0) + 0.04
        and bar0_aabb[0][2] > shell0_aabb[0][2] + 0.10,
        details=f"shell={shell0_aabb}, bar={bar0_aabb}",
    )

    # Bucket sidewalls extend forward: the shell has material at the front edge.
    # The front opening is between the sidewalls, so the shell's X extent should
    # be close to the full BUCKET_DX (sidewalls reach forward to the front face).
    ctx.check(
        "bucket shell sidewalls wrap forward (X extent > 0.25 m)",
        shell0_aabb is not None and (shell0_aabb[1][0] - shell0_aabb[0][0]) > 0.25,
        details=f"bucket_shell x extent={None if shell0_aabb is None else shell0_aabb[1][0] - shell0_aabb[0][0]}",
    )

    # Overall scale: seat outer edge near 1.6 m radius (~3 m diameter), top ~1.3 m.
    rotor_aabb = ctx.part_world_aabb(rotor)
    ctx.check(
        "carousel spans about 3 m and stands about 1.3 m tall",
        shell0_aabb is not None
        and rotor_aabb is not None
        and 1.45 <= shell0_aabb[1][0] <= 1.75
        and 1.25 <= rotor_aabb[1][2] <= 1.35,
        details=f"seat outer x={None if shell0_aabb is None else shell0_aabb[1][0]}, rotor top={None if rotor_aabb is None else rotor_aabb[1][2]}",
    )

    # Decisive pose checks: positive swing flies seat_0 outward and upward.
    with ctx.pose({swings[0]: SWING}):
        shell_out = ctx.part_element_world_aabb(seats[0], elem="bucket_shell")
    ctx.check(
        "positive seat swing flies the bucket outward and upward",
        shell0_aabb is not None
        and shell_out is not None
        and _center(shell_out, 0) > _center(shell0_aabb, 0) + 0.10
        and _center(shell_out, 2) > _center(shell0_aabb, 2) + 0.02,
        details=f"rest={shell0_aabb}, swung_out={shell_out}",
    )
    with ctx.pose({swings[0]: -SWING}):
        shell_in = ctx.part_element_world_aabb(seats[0], elem="bucket_shell")
    ctx.check(
        "negative seat swing tucks the bucket inward",
        shell0_aabb is not None
        and shell_in is not None
        and _center(shell_in, 0) < _center(shell0_aabb, 0) - 0.10,
        details=f"rest={shell0_aabb}, swung_in={shell_in}",
    )

    # Quarter-turn spin carries seat_0 from +X to +Y.
    with ctx.pose({spin: math.pi / 2.0}):
        shell_spun = ctx.part_element_world_aabb(seats[0], elem="bucket_shell")
    ctx.check(
        "quarter-turn spin carries seat_0 a quarter circle around the column",
        shell_spun is not None
        and _center(shell_spun, 1) > 1.2
        and abs(_center(shell_spun, 0)) < 0.3,
        details=f"spun bucket aabb={shell_spun}",
    )

    return ctx.report()


object_model = build_object_model()
