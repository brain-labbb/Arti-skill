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
# Four-seat playground seesaw built from bent steel tubing (~40 mm diameter).
#
# Layout (world frame, Z up, base centered on the origin):
# - Sky-blue base: two arched inverted-U tube legs (a low arch, top at 0.56 m,
#   and a high arch, top at 0.74 m) joined by two short cross members.
# - Two independent yellow rocking beams (~2.6 m), arranged in a shallow X:
#   the lower beam runs at yaw +10 deg, the upper beam at yaw -10 deg, so they
#   cross directly above the base at different heights.
# - Each beam is a triangulated tube truss (main top tube + two diagonal
#   braces meeting at an axle sleeve) and carries at each end a rust-brown
#   sheet-metal seat plate on a short bent tube support plus an upright
#   yellow T-handlebar just inboard of the seat.
# - Articulation: each beam has its own revolute pivot about the top tube of
#   its arch (horizontal axis perpendicular to the beam), +/- 18 degrees.
# ----------------------------------------------------------------------------

TUBE_R = 0.020  # ~40 mm diameter main tubing
BRACE_R = 0.016
SUPPORT_R = 0.018
HANDLE_R = 0.016

YAW = math.radians(10.0)  # half angle of the shallow X between the beams
TILT = math.radians(18.0)  # rocking range of each beam

LOW_ARCH_TOP = 0.56  # pivot height of the lower beam
HIGH_ARCH_TOP = 0.74  # pivot height of the upper beam (base "about 0.7 m tall")
ARCH_HALF_SPAN = 0.36  # ground half-span of each arch
CROSS_BRACE_Z = 0.28  # height of the short cross members joining the arches
CROSS_BRACE_U = 0.315  # arch-plane coordinate of the legs at CROSS_BRACE_Z

BEAM_LEN = 2.60
MAIN_Z = 0.08  # main top tube height above the pivot axis
SLEEVE_R = 0.032
SLEEVE_LEN = 0.13
SEAT_X = 1.43  # seat plate center, measured along the beam from the pivot
SEAT_Z = 0.038  # seat plate center height above the pivot axis
SEAT_SIZE = (0.26, 0.30, 0.012)
HANDLE_X = 1.04  # T-handlebar post, just inboard of the seat
HANDLE_TOP_Z = 0.34  # crossbar height above the pivot axis

SKY_BLUE = Material("sky_blue_paint", rgba=(0.33, 0.62, 0.84, 1.0))
WORN_YELLOW = Material("worn_yellow_paint", rgba=(0.87, 0.74, 0.12, 1.0))
RUST_BROWN = Material("rust_brown_steel", rgba=(0.42, 0.21, 0.13, 1.0))


def _tube_between(
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    radius: float,
    *,
    radial_segments: int = 16,
) -> MeshGeometry:
    """Straight capped tube between two 3D points (cylinder is Z-aligned by default)."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    geom = CylinderGeometry(radius, length, radial_segments=radial_segments)
    # Rotate local +Z onto the segment direction.
    ux, uy, uz = dx / length, dy / length, dz / length
    ax, ay, az = -uy, ux, 0.0  # cross((0,0,1), u)
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
    """Inverted-U arch tube in the vertical plane spanned by axis_xy.

    The top run is flat (it doubles as the pivot axle), the legs curve down
    and out to flared feet on the ground. The path passes exactly through the
    cross-member attachment points at CROSS_BRACE_Z.
    """
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
    """Build one rocking beam in its local frame (X along the beam, pivot at origin).

    Returns (truss_tube, axle_sleeve, handlebar_pos_x, handlebar_neg_x).
    """
    # Main top tube, full length, riding above the pivot axis.
    truss = (
        CylinderGeometry(TUBE_R, BEAM_LEN, radial_segments=18)
        .rotate_y(math.pi / 2.0)
        .translate(0.0, 0.0, MAIN_Z)
    )
    for sx in (1.0, -1.0):
        # Diagonal brace from the axle sleeve up to the main tube (triangulated truss).
        truss.merge(
            _tube_between(
                (sx * 0.04, 0.0, 0.005),
                (sx * 0.60, 0.0, MAIN_Z),
                BRACE_R,
            )
        )
        # Short bent seat support dropping from the main tube end under the seat.
        truss.merge(
            tube_from_spline_points(
                [
                    (sx * 1.24, 0.0, MAIN_Z),
                    (sx * 1.34, 0.0, 0.055),
                    (sx * 1.42, 0.0, 0.020),
                    (sx * 1.49, 0.0, 0.012),
                ],
                radius=SUPPORT_R,
                samples_per_segment=10,
                radial_segments=14,
                cap_ends=True,
            )
        )

    # Axle sleeve wrapping the arch-top pivot tube (axis along local Y), plus a
    # short weld post tying the sleeve to the main top tube so the hub reads
    # as one welded hub, not a floating ring. The post bottom stays at radial
    # distance 0.024 from the pivot axis, clear of the 0.020 axle tube.
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
    beam.visual(
        Box(SEAT_SIZE),
        origin=Origin(xyz=(SEAT_X, 0.0, SEAT_Z)),
        material=RUST_BROWN,
        name="seat_plate_0",
    )
    beam.visual(
        Box(SEAT_SIZE),
        origin=Origin(xyz=(-SEAT_X, 0.0, SEAT_Z)),
        material=RUST_BROWN,
        name="seat_plate_1",
    )
    return beam


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="four_seat_tube_seesaw")

    # --- static sky-blue base -------------------------------------------------
    base = model.part("base")
    # Arch planes are perpendicular to their beam's direction.
    low_axis = (-math.sin(YAW), math.cos(YAW))  # perpendicular to the +10 deg beam
    high_axis = (math.sin(YAW), math.cos(YAW))  # perpendicular to the -10 deg beam
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

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lower_beam = object_model.get_part("lower_beam")
    upper_beam = object_model.get_part("upper_beam")
    lower_pivot = object_model.get_articulation("lower_beam_pivot")
    upper_pivot = object_model.get_articulation("upper_beam_pivot")

    # Captured-axle fits: each beam's sleeve intentionally wraps the top tube
    # of its own arch, which is the physical pivot axle.
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

    # Base reads as the prompt's ~0.7 m tall arched stand resting on the ground.
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

    # All four seats and four handlebars exist, ~1.4 m out from the pivot, at
    # sit height, each with its T-handlebar upright just inboard of the seat.
    for beam, lo_z, hi_z in ((lower_beam, 0.50, 0.68), (upper_beam, 0.68, 0.86)):
        for end in (0, 1):
            seat = ctx.part_element_world_aabb(beam, elem=f"seat_plate_{end}")
            handle = ctx.part_element_world_aabb(beam, elem=f"handlebar_{end}")
            ok = seat is not None and handle is not None
            ctx.check(
                f"{beam.name} end {end} carries a seat plate and a handlebar",
                ok,
                details=f"seat={seat}, handle={handle}",
            )
            if not ok:
                continue
            scx = (seat[0][0] + seat[1][0]) / 2.0
            scy = (seat[0][1] + seat[1][1]) / 2.0
            scz = (seat[0][2] + seat[1][2]) / 2.0
            ctx.check(
                f"{beam.name} seat {end} sits near the beam end at sit height",
                math.hypot(scx, scy) > 1.25 and lo_z <= scz <= hi_z,
                details=f"seat center=({scx:.3f},{scy:.3f},{scz:.3f})",
            )
            hcx = (handle[0][0] + handle[1][0]) / 2.0
            hcy = (handle[0][1] + handle[1][1]) / 2.0
            inboard = math.hypot(scx - hcx, scy - hcy)
            ctx.check(
                f"{beam.name} handlebar {end} stands upright just inboard of its seat",
                handle[1][2] > seat[1][2] + 0.15 and 0.20 <= inboard <= 0.55,
                details=f"handle top={handle[1][2]:.3f}, seat top={seat[1][2]:.3f}, inboard={inboard:.3f}",
            )

    # Shallow X: the two beams cross above the base footprint but their +X end
    # seats diverge to opposite lateral sides.
    ctx.expect_overlap(
        lower_beam,
        upper_beam,
        axes="xy",
        min_overlap=0.5,
        name="beams cross above the base in plan view",
    )
    lo_seat0 = ctx.part_element_world_aabb(lower_beam, elem="seat_plate_0")
    up_seat0 = ctx.part_element_world_aabb(upper_beam, elem="seat_plate_0")
    ctx.check(
        "beam directions splay into a shallow X",
        lo_seat0 is not None
        and up_seat0 is not None
        and (lo_seat0[0][1] + lo_seat0[1][1]) / 2.0 > 0.15
        and (up_seat0[0][1] + up_seat0[1][1]) / 2.0 < -0.15,
        details=f"lower seat0={lo_seat0}, upper seat0={up_seat0}",
    )

    # The two pivots stack at different heights so the beams clear each other.
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

    # Rocking range is +/- 18 degrees on both pivots.
    for pivot in (lower_pivot, upper_pivot):
        lim = pivot.motion_limits
        ctx.check(
            f"{pivot.name} rocks +/- 18 degrees",
            lim is not None
            and abs(lim.lower + TILT) < 1e-6
            and abs(lim.upper - TILT) < 1e-6,
            details=f"limits=({lim.lower if lim else None}, {lim.upper if lim else None})",
        )

    # Decisive pose checks: each beam seesaws about its own pivot, the other
    # beam does not move, and nothing dips below the ground.
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
