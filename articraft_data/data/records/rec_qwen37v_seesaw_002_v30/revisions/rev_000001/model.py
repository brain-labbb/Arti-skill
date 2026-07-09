from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CylinderGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ----------------------------------------------------------------------------
# Variant 30: Asymmetric-height four-seat playground seesaw with a central
# compression spring and rubber ground pads.
#
# Layout (world frame, Z up, base centered on the origin):
# - Sky-blue base: two arched inverted-U tube legs joined by cross members,
#   with dark rubber ground pads under each foot.
# - Two independent yellow rocking beams (~2.6 m), arranged in a shallow X.
#   Each beam has asymmetric seat heights: the +X seat is mounted higher
#   than the -X seat, but the beam still pivots at its geometric center.
# - A coil spring sits under the lower beam center on a prismatic joint,
#   compressing vertically under load.
# - Articulation: each beam has its own revolute pivot (+/- 18 degrees),
#   plus the spring has a prismatic joint (vertical, 0 to -0.05 m travel).
# ----------------------------------------------------------------------------

TUBE_R = 0.020  # ~40 mm diameter main tubing
BRACE_R = 0.016
SUPPORT_R = 0.018
HANDLE_R = 0.016

YAW = math.radians(10.0)  # half angle of the shallow X between the beams
TILT = math.radians(18.0)  # rocking range of each beam

LOW_ARCH_TOP = 0.56  # pivot height of the lower beam
HIGH_ARCH_TOP = 0.74  # pivot height of the upper beam
ARCH_HALF_SPAN = 0.36  # ground half-span of each arch
CROSS_BRACE_Z = 0.28  # height of the short cross members joining the arches
CROSS_BRACE_U = 0.315  # arch-plane coordinate of the legs at CROSS_BRACE_Z

BEAM_LEN = 2.60
MAIN_Z = 0.08  # main top tube height above the pivot axis
SLEEVE_R = 0.032
SLEEVE_LEN = 0.13
SEAT_X = 1.43  # seat plate center along the beam from the pivot
HANDLE_X = 1.04  # T-handlebar post, just inboard of the seat
HANDLE_TOP_Z = 0.34  # crossbar height above the pivot axis

# Asymmetric seat heights: +X end higher, -X end lower
SEAT_Z_HIGH = 0.065  # seat plate center height above pivot (high end, +X)
SEAT_Z_LOW = 0.020   # seat plate center height above pivot (low end, -X)
SEAT_SIZE = (0.26, 0.30, 0.012)

# Spring geometry
SPRING_WIRE_R = 0.006
SPRING_COIL_R = 0.040
SPRING_COILS = 6
SPRING_FREE_HEIGHT = 0.09
SPRING_ORIGIN_Z = 0.43  # world Z of spring part origin (top of support post)

# Ground pad geometry
PAD_SIZE = (0.12, 0.12, 0.012)
PAD_FOOT_U = ARCH_HALF_SPAN + 0.04  # along arch axis to foot center

# Spring support post on the base
POST_RADIUS = 0.022

SKY_BLUE = Material("sky_blue_paint", rgba=(0.33, 0.62, 0.84, 1.0))
WORN_YELLOW = Material("worn_yellow_paint", rgba=(0.87, 0.74, 0.12, 1.0))
RUST_BROWN = Material("rust_brown_steel", rgba=(0.42, 0.21, 0.13, 1.0))
RUBBER_BLACK = Material("rubber_black", rgba=(0.12, 0.12, 0.12, 1.0))
SPRING_STEEL = Material("spring_steel", rgba=(0.55, 0.55, 0.52, 1.0))


def _tube_between(
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    radius: float,
    *,
    radial_segments: int = 16,
) -> MeshGeometry:
    """Straight capped tube between two 3D points."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    geom = CylinderGeometry(radius, length, radial_segments=radial_segments)
    ux, uy, uz = dx / length, dy / length, dz / length
    ax, ay, az = -uy, ux, 0.0
    s = math.sqrt(ax * ax + ay * ay + az * az)
    if s > 1e-9:
        geom.rotate((ax / s, ay / s, az / s), math.atan2(s, uz))
    elif uz < 0.0:
        geom.rotate_x(math.pi)
    geom.translate(
        (p0[0] + p1[0]) / 2.0,
        (p0[1] + p1[1]) / 2.0,
        (p0[2] + p1[2]) / 2.0,
    )
    return geom


def _arch_mesh(axis_xy: tuple[float, float], top_z: float) -> MeshGeometry:
    """Inverted-U arch tube in the vertical plane spanned by axis_xy."""
    ax, ay = axis_xy
    shoulder = 0.52 if top_z < 0.65 else 0.66
    profile_uz = [
        (-ARCH_HALF_SPAN - 0.055, 0.022),
        (-ARCH_HALF_SPAN - 0.03, 0.028),
        (-0.35, 0.10),
        (-CROSS_BRACE_U, CROSS_BRACE_Z),
        (-0.27, 0.44),
        (-0.18, shoulder),
        (-0.07, top_z),
        (0.0, top_z),
        (0.07, top_z),
        (0.18, shoulder),
        (0.27, 0.44),
        (CROSS_BRACE_U, CROSS_BRACE_Z),
        (0.35, 0.10),
        (ARCH_HALF_SPAN + 0.03, 0.028),
        (ARCH_HALF_SPAN + 0.055, 0.022),
    ]
    points = [(u * ax, u * ay, z) for (u, z) in profile_uz]
    return tube_from_spline_points(
        points,
        radius=TUBE_R,
        samples_per_segment=10,
        radial_segments=16,
        cap_ends=True,
    )


def _beam_meshes() -> tuple[MeshGeometry, MeshGeometry, MeshGeometry, MeshGeometry]:
    """Build one rocking beam in its local frame with asymmetric seat heights.

    The +X seat is mounted higher (SEAT_Z_HIGH), the -X seat lower (SEAT_Z_LOW).
    Returns (truss_tube, axle_sleeve, handlebar_pos_x, handlebar_neg_x).
    """
    # Main top tube, full length, riding above the pivot axis.
    truss = (
        CylinderGeometry(TUBE_R, BEAM_LEN, radial_segments=18)
        .rotate_y(math.pi / 2.0)
        .translate(0.0, 0.0, MAIN_Z)
    )
    for sx in (1.0, -1.0):
        # Diagonal brace from the axle sleeve up to the main tube.
        truss.merge(
            _tube_between(
                (sx * 0.04, 0.0, 0.005),
                (sx * 0.60, 0.0, MAIN_Z),
                BRACE_R,
            )
        )
        # Asymmetric seat support: +X end has taller support reaching SEAT_Z_HIGH,
        # -X end has shorter support reaching SEAT_Z_LOW.
        seat_z = SEAT_Z_HIGH if sx > 0 else SEAT_Z_LOW
        support_top = MAIN_Z
        support_mid_z = max(seat_z + 0.015, 0.030)
        truss.merge(
            tube_from_spline_points(
                [
                    (sx * 1.24, 0.0, support_top),
                    (sx * 1.34, 0.0, support_mid_z + 0.020),
                    (sx * 1.42, 0.0, support_mid_z),
                    (sx * 1.49, 0.0, seat_z + 0.006),
                ],
                radius=SUPPORT_R,
                samples_per_segment=10,
                radial_segments=14,
                cap_ends=True,
            )
        )

    # Axle sleeve wrapping the arch-top pivot tube.
    sleeve = (
        CylinderGeometry(SLEEVE_R, SLEEVE_LEN, radial_segments=20)
        .rotate_x(math.pi / 2.0)
    )
    weld_post = CylinderGeometry(0.014, MAIN_Z - 0.024, radial_segments=14).translate(
        0.0, 0.0, (MAIN_Z + 0.024) / 2.0
    )
    sleeve.merge(weld_post)

    handlebars: list[MeshGeometry] = []
    for sx in (1.0, -1.0):
        post = CylinderGeometry(HANDLE_R, 0.28, radial_segments=14).translate(
            sx * HANDLE_X, 0.0, MAIN_Z + 0.13
        )
        bar = (
            CylinderGeometry(HANDLE_R, 0.30, radial_segments=14)
            .rotate_x(math.pi / 2.0)
            .translate(sx * HANDLE_X, 0.0, HANDLE_TOP_Z)
        )
        handlebars.append(post.merge(bar))

    return truss, sleeve, handlebars[0], handlebars[1]


def _spring_mesh() -> MeshGeometry:
    """Helical coil spring mesh centered at the origin, extending along +Z."""
    n_pts = SPRING_COILS * 24 + 1
    points: list[tuple[float, float, float]] = []
    for i in range(n_pts):
        t = i / (n_pts - 1)
        angle = t * SPRING_COILS * 2.0 * math.pi
        x = SPRING_COIL_R * math.cos(angle)
        y = SPRING_COIL_R * math.sin(angle)
        z = t * SPRING_FREE_HEIGHT
        points.append((x, y, z))
    return tube_from_spline_points(
        points,
        radius=SPRING_WIRE_R,
        samples_per_segment=4,
        radial_segments=10,
        cap_ends=True,
    )


def _add_beam_part(model: ArticulatedObject, part_name: str):
    truss, sleeve, hb0, hb1 = _beam_meshes()
    beam = model.part(part_name)
    beam.visual(
        mesh_from_geometry(truss, f"{part_name}_truss"),
        material=WORN_YELLOW,
        name="truss_tube",
    )
    beam.visual(
        mesh_from_geometry(sleeve, f"{part_name}_sleeve"),
        material=WORN_YELLOW,
        name="axle_sleeve",
    )
    beam.visual(
        mesh_from_geometry(hb0, f"{part_name}_handlebar_0"),
        material=WORN_YELLOW,
        name="handlebar_0",
    )
    beam.visual(
        mesh_from_geometry(hb1, f"{part_name}_handlebar_1"),
        material=WORN_YELLOW,
        name="handlebar_1",
    )
    # Asymmetric seat plates: +X end higher, -X end lower
    beam.visual(
        Box(SEAT_SIZE),
        origin=Origin(xyz=(SEAT_X, 0.0, SEAT_Z_HIGH)),
        material=RUST_BROWN,
        name="seat_plate_0",
    )
    beam.visual(
        Box(SEAT_SIZE),
        origin=Origin(xyz=(-SEAT_X, 0.0, SEAT_Z_LOW)),
        material=RUST_BROWN,
        name="seat_plate_1",
    )
    return beam


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="asymmetric_spring_seesaw")

    # --- static sky-blue base with rubber ground pads -------------------------
    base = model.part("base")
    low_axis = (-math.sin(YAW), math.cos(YAW))
    high_axis = (math.sin(YAW), math.cos(YAW))
    base.visual(
        mesh_from_geometry(_arch_mesh(low_axis, LOW_ARCH_TOP), "low_arch"),
        material=SKY_BLUE,
        name="low_arch",
    )
    base.visual(
        mesh_from_geometry(_arch_mesh(high_axis, HIGH_ARCH_TOP), "high_arch"),
        material=SKY_BLUE,
        name="high_arch",
    )
    # Short cross members tying the four legs into one rigid stand.
    leg_y = CROSS_BRACE_U * math.cos(YAW)
    leg_x = CROSS_BRACE_U * math.sin(YAW)
    for idx, sy in enumerate((1.0, -1.0)):
        brace = _tube_between(
            (-leg_x - 0.012, sy * leg_y, CROSS_BRACE_Z),
            (leg_x + 0.012, sy * leg_y, CROSS_BRACE_Z),
            SUPPORT_R,
        )
        base.visual(
            mesh_from_geometry(brace, f"cross_brace_{idx}"),
            material=SKY_BLUE,
            name=f"cross_brace_{idx}",
        )

    # Vertical support post from cross brace up to the spring mount point,
    # with horizontal struts connecting the post to the cross braces for
    # structural continuity.
    post_height = SPRING_ORIGIN_Z - CROSS_BRACE_Z
    post_mesh = CylinderGeometry(POST_RADIUS, post_height, radial_segments=16)
    post_mesh.translate(0.0, 0.0, CROSS_BRACE_Z + post_height / 2.0)
    # Two Y-axis struts connecting the post to the cross braces
    for sy in (1.0, -1.0):
        strut = _tube_between(
            (0.0, sy * POST_RADIUS, CROSS_BRACE_Z),
            (0.0, sy * (leg_y - 0.018), CROSS_BRACE_Z),
            SUPPORT_R,
            radial_segments=12,
        )
        post_mesh.merge(strut)
    base.visual(
        mesh_from_geometry(post_mesh, "spring_support_post"),
        material=SKY_BLUE,
        name="spring_support_post",
    )

    # Rubber ground pads under each arch foot (4 pads total).
    pad_idx = 0
    for axis_xy in (low_axis, high_axis):
        ax, ay = axis_xy
        for su in (1.0, -1.0):
            foot_x = su * PAD_FOOT_U * ax
            foot_y = su * PAD_FOOT_U * ay
            base.visual(
                Box(PAD_SIZE),
                origin=Origin(xyz=(foot_x, foot_y, 0.006)),
                material=RUBBER_BLACK,
                name=f"ground_pad_{pad_idx}",
            )
            pad_idx += 1

    # --- two independent yellow rocking beams ---------------------------------
    lower_beam = _add_beam_part(model, "lower_beam")
    upper_beam = _add_beam_part(model, "upper_beam")

    limits = MotionLimits(effort=150.0, velocity=2.5, lower=-TILT, upper=TILT)
    model.articulation(
        "lower_beam_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_beam,
        origin=Origin(xyz=(0.0, 0.0, LOW_ARCH_TOP), rpy=(0.0, 0.0, YAW)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=limits,
    )
    model.articulation(
        "upper_beam_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_beam,
        origin=Origin(xyz=(0.0, 0.0, HIGH_ARCH_TOP), rpy=(0.0, 0.0, -YAW)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=limits,
    )

    # --- central compression spring under the lower beam ----------------------
    spring = model.part("spring")
    spring.visual(
        mesh_from_geometry(_spring_mesh(), "spring_coil"),
        material=SPRING_STEEL,
        name="spring_coil",
    )
    # Small mounting plate at the top of the spring base
    spring.visual(
        Box((0.10, 0.10, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=RUBBER_BLACK,
        name="spring_base_plate",
    )

    model.articulation(
        "spring_compress",
        ArticulationType.PRISMATIC,
        parent=base,
        child=spring,
        # Spring sits on the support post, just below the lower beam pivot
        origin=Origin(xyz=(0.0, 0.0, SPRING_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=500.0,
            velocity=0.5,
            lower=-0.05,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lower_beam = object_model.get_part("lower_beam")
    upper_beam = object_model.get_part("upper_beam")
    spring = object_model.get_part("spring")
    lower_pivot = object_model.get_articulation("lower_beam_pivot")
    upper_pivot = object_model.get_articulation("upper_beam_pivot")
    spring_joint = object_model.get_articulation("spring_compress")

    # --- Captured-axle fits ---------------------------------------------------
    ctx.allow_overlap(
        lower_beam,
        base,
        elem_a="axle_sleeve",
        elem_b="low_arch",
        reason="Lower beam axle sleeve intentionally wraps the low arch top tube, its pivot axle.",
    )
    ctx.allow_overlap(
        upper_beam,
        base,
        elem_a="axle_sleeve",
        elem_b="high_arch",
        reason="Upper beam axle sleeve intentionally wraps the high arch top tube, its pivot axle.",
    )
    ctx.expect_contact(
        lower_beam,
        base,
        elem_a="axle_sleeve",
        elem_b="low_arch",
        name="lower beam sleeve rides on the low arch axle",
    )
    ctx.expect_contact(
        upper_beam,
        base,
        elem_a="axle_sleeve",
        elem_b="high_arch",
        name="upper beam sleeve rides on the high arch axle",
    )

    # --- Base dimensions and ground contact -----------------------------------
    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "base is an arched stand about 0.7 m tall",
        base_aabb is not None and 0.70 <= base_aabb[1][2] <= 0.82,
        details=f"base aabb={base_aabb}",
    )
    ctx.check(
        "base feet rest on the ground plane",
        base_aabb is not None and -0.01 <= base_aabb[0][2] <= 0.015,
        details=f"base aabb={base_aabb}",
    )

    # --- Rubber ground pads exist on the base ---------------------------------
    for i in range(4):
        pad_aabb = ctx.part_element_world_aabb(base, elem=f"ground_pad_{i}")
        ctx.check(
            f"ground_pad_{i} exists on the base",
            pad_aabb is not None,
            details=f"pad aabb={pad_aabb}",
        )
        if pad_aabb is not None:
            pad_cz = (pad_aabb[0][2] + pad_aabb[1][2]) / 2.0
            ctx.check(
                f"ground_pad_{i} sits at ground level",
                -0.005 <= pad_cz <= 0.020,
                details=f"pad center z={pad_cz:.4f}",
            )

    # --- Asymmetric seat heights ----------------------------------------------
    for beam, pivot_z in ((lower_beam, LOW_ARCH_TOP), (upper_beam, HIGH_ARCH_TOP)):
        seat0_aabb = ctx.part_element_world_aabb(beam, elem="seat_plate_0")
        seat1_aabb = ctx.part_element_world_aabb(beam, elem="seat_plate_1")
        ctx.check(
            f"{beam.name} has both seat plates",
            seat0_aabb is not None and seat1_aabb is not None,
            details=f"seat0={seat0_aabb}, seat1={seat1_aabb}",
        )
        if seat0_aabb is not None and seat1_aabb is not None:
            seat0_cz = (seat0_aabb[0][2] + seat0_aabb[1][2]) / 2.0
            seat1_cz = (seat1_aabb[0][2] + seat1_aabb[1][2]) / 2.0
            height_diff = abs(seat0_cz - seat1_cz)
            ctx.check(
                f"{beam.name} seats are at asymmetric heights",
                height_diff > 0.020,
                details=f"seat0_cz={seat0_cz:.4f}, seat1_cz={seat1_cz:.4f}, diff={height_diff:.4f}",
            )

    # --- Handlebars exist near seats ------------------------------------------
    for beam, lo_z, hi_z in ((lower_beam, 0.50, 0.72), (upper_beam, 0.68, 0.90)):
        for end in (0, 1):
            handle = ctx.part_element_world_aabb(beam, elem=f"handlebar_{end}")
            ctx.check(
                f"{beam.name} end {end} has a handlebar",
                handle is not None,
                details=f"handle={handle}",
            )

    # --- Shallow X: beams cross above base ------------------------------------
    ctx.expect_overlap(
        lower_beam,
        upper_beam,
        axes="xy",
        min_overlap=0.5,
        name="beams cross above the base in plan view",
    )

    # --- Upper beam pivots above lower beam -----------------------------------
    lo_sleeve = ctx.part_element_world_aabb(lower_beam, elem="axle_sleeve")
    up_sleeve = ctx.part_element_world_aabb(upper_beam, elem="axle_sleeve")
    ctx.check(
        "upper beam pivots above the lower beam",
        lo_sleeve is not None
        and up_sleeve is not None
        and (up_sleeve[0][2] + up_sleeve[1][2]) / 2.0
        > (lo_sleeve[0][2] + lo_sleeve[1][2]) / 2.0 + 0.10,
        details=f"lower sleeve={lo_sleeve}, upper sleeve={up_sleeve}",
    )

    # --- Revolute joint limits: +/- 18 degrees --------------------------------
    for pivot in (lower_pivot, upper_pivot):
        lim = pivot.motion_limits
        ctx.check(
            f"{pivot.name} rocks +/- 18 degrees",
            lim is not None
            and abs(lim.lower + TILT) < 1e-6
            and abs(lim.upper - TILT) < 1e-6,
            details=f"limits=({lim.lower if lim else None}, {lim.upper if lim else None})",
        )

    # --- Spring prismatic joint -----------------------------------------------
    spring_lim = spring_joint.motion_limits
    ctx.check(
        "spring_compress is a prismatic joint with downward travel",
        spring_joint.articulation_type == ArticulationType.PRISMATIC
        and spring_lim is not None
        and spring_lim.lower < -0.01
        and abs(spring_lim.upper) < 1e-6,
        details=f"type={spring_joint.articulation_type}, limits=({spring_lim.lower if spring_lim else None}, {spring_lim.upper if spring_lim else None})",
    )

    # Spring sits between the base cross brace and the lower beam pivot
    spring_aabb = ctx.part_world_aabb(spring)
    ctx.check(
        "spring sits between base cross member and lower beam pivot",
        spring_aabb is not None
        and spring_aabb[0][2] > CROSS_BRACE_Z - 0.02
        and spring_aabb[1][2] < LOW_ARCH_TOP,
        details=f"spring aabb={spring_aabb}",
    )

    # Spring compresses downward under the prismatic joint
    spring_rest_z = ctx.part_world_position(spring)
    with ctx.pose({spring_joint: -0.04}):
        spring_compressed_z = ctx.part_world_position(spring)
        ctx.check(
            "spring compresses downward on prismatic actuation",
            spring_rest_z is not None
            and spring_compressed_z is not None
            and spring_compressed_z[2] < spring_rest_z[2] - 0.02,
            details=f"rest={spring_rest_z}, compressed={spring_compressed_z}",
        )

    # --- Decisive pose checks: beams seesaw independently ---------------------
    rest_lo0 = ctx.part_element_world_aabb(lower_beam, elem="seat_plate_0")
    rest_lo1 = ctx.part_element_world_aabb(lower_beam, elem="seat_plate_1")
    rest_up0 = ctx.part_element_world_aabb(upper_beam, elem="seat_plate_0")
    with ctx.pose({lower_pivot: TILT}):
        tilt_lo0 = ctx.part_element_world_aabb(lower_beam, elem="seat_plate_0")
        tilt_lo1 = ctx.part_element_world_aabb(lower_beam, elem="seat_plate_1")
        tilt_up0 = ctx.part_element_world_aabb(upper_beam, elem="seat_plate_0")
        beam_aabb = ctx.part_world_aabb(lower_beam)
        ctx.check(
            "lower beam seesaws: one seat drops, the opposite seat rises",
            rest_lo0 is not None
            and tilt_lo0 is not None
            and rest_lo1 is not None
            and tilt_lo1 is not None
            and tilt_lo0[0][2] < rest_lo0[0][2] - 0.35
            and tilt_lo1[0][2] > rest_lo1[0][2] + 0.35,
            details=f"seat0 {rest_lo0} -> {tilt_lo0}, seat1 {rest_lo1} -> {tilt_lo1}",
        )
        ctx.check(
            "fully tilted lower beam stays clear of the ground",
            beam_aabb is not None and beam_aabb[0][2] > 0.02,
            details=f"lower beam aabb={beam_aabb}",
        )
        ctx.check(
            "beams rock independently: upper beam holds still while lower rocks",
            rest_up0 is not None
            and tilt_up0 is not None
            and abs(tilt_up0[0][2] - rest_up0[0][2]) < 1e-6,
            details=f"upper seat0 {rest_up0} -> {tilt_up0}",
        )
        ctx.expect_contact(
            lower_beam,
            base,
            elem_a="axle_sleeve",
            elem_b="low_arch",
            name="tilted lower beam sleeve stays on its axle",
        )
    with ctx.pose({upper_pivot: -TILT}):
        tilt_up0 = ctx.part_element_world_aabb(upper_beam, elem="seat_plate_0")
        beam_aabb = ctx.part_world_aabb(upper_beam)
        ctx.check(
            "upper beam seesaws the opposite way: its near seat rises",
            rest_up0 is not None
            and tilt_up0 is not None
            and tilt_up0[0][2] > rest_up0[0][2] + 0.35,
            details=f"upper seat0 {rest_up0} -> {tilt_up0}",
        )
        ctx.check(
            "fully tilted upper beam stays clear of the ground",
            beam_aabb is not None and beam_aabb[0][2] > 0.02,
            details=f"upper beam aabb={beam_aabb}",
        )
        ctx.expect_contact(
            upper_beam,
            base,
            elem_a="axle_sleeve",
            elem_b="high_arch",
            name="tilted upper beam sleeve stays on its axle",
        )

    return ctx.report()


object_model = build_object_model()
