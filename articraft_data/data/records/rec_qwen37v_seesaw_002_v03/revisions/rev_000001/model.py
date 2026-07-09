from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CylinderGeometry,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ----------------------------------------------------------------------------
# Spring-assisted modern playground seesaw.
#
# Layout (world frame, Z up, base centered on origin):
# - Dark-green steel pedestal: ground plate + central column + visible coil
#   spring + pivot plate, pivot at ~0.65 m.
# - Single bright-orange beam (~2.4 m) with triangulated tube truss, pivoting
#   on the spring post via a revolute joint (+/- 18 deg).
# - Molded round seats with raised lips at each beam end (LatheGeometry).
# - Upright T-handlebars just inboard of each seat.
# - Black rubber end bumpers under each beam end on short prismatic joints
#   (0 to 25 mm vertical compression).
# ----------------------------------------------------------------------------

TUBE_R = 0.020  # 40 mm main beam tubing
BRACE_R = 0.016
HANDLE_R = 0.014

BEAM_LEN = 2.40
PIVOT_Z = 0.65  # pivot height above ground
TILT = math.radians(18.0)

MAIN_Z = 0.06  # main tube center height above pivot axis
SEAT_X = 1.10  # seat center along beam from pivot
SEAT_Z = 0.025  # seat bottom height above pivot axis
HANDLE_X = 0.78  # handlebar post, inboard of seat
HANDLE_TOP_Z = 0.30  # crossbar height above pivot

BUMPER_R = 0.028
BUMPER_H = 0.035
BUMPER_X = 1.15  # bumper mount position along beam from pivot

# Spring geometry
SPRING_R = 0.050  # spring coil center radius
SPRING_WIRE = 0.010  # spring wire radius
SPRING_COILS = 5
SPRING_BASE_Z = 0.42  # bottom of spring section
SPRING_TOP_Z = 0.58  # top of spring section (below pivot plate)

# Base geometry
PLATE_R = 0.25
PLATE_H = 0.025
COLUMN_R = 0.038
COLUMN_BOTTOM = PLATE_H
COLUMN_TOP = SPRING_BASE_Z
PIVOT_PLATE_R = 0.060
PIVOT_PLATE_H = 0.016

# Seat geometry
SEAT_RADIUS = 0.14
SEAT_LIP_HEIGHT = 0.038

# Materials
DARK_GREEN = Material("dark_green_paint", rgba=(0.15, 0.40, 0.20, 1.0))
BRIGHT_ORANGE = Material("bright_orange_paint", rgba=(0.90, 0.35, 0.08, 1.0))
MOLDED_GRAY = Material("molded_gray_plastic", rgba=(0.30, 0.30, 0.32, 1.0))
BLACK_RUBBER = Material("black_rubber", rgba=(0.08, 0.08, 0.08, 1.0))
STEEL_SPRING = Material("steel_spring", rgba=(0.55, 0.55, 0.52, 1.0))


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


def _base_meshes() -> tuple[MeshGeometry, MeshGeometry]:
    """Build the base pedestal and spring coil visuals.

    Returns (pedestal_mesh, spring_mesh).
    Pedestal: ground plate, column, pivot plate, gusset ribs, spring shaft.
    Spring: coil rings that overlap the shaft for connectivity.
    """
    # Ground plate
    plate = CylinderGeometry(PLATE_R, PLATE_H, radial_segments=32).translate(
        0.0, 0.0, PLATE_H / 2.0
    )
    # Central column: from plate top to spring base
    col_height = COLUMN_TOP - COLUMN_BOTTOM
    column = CylinderGeometry(COLUMN_R, col_height, radial_segments=24).translate(
        0.0, 0.0, COLUMN_BOTTOM + col_height / 2.0
    )
    plate.merge(column)

    # Pivot plate just below pivot height
    pp_bottom = PIVOT_Z - PIVOT_PLATE_H
    pivot_plate = CylinderGeometry(PIVOT_PLATE_R, PIVOT_PLATE_H, radial_segments=28).translate(
        0.0, 0.0, pp_bottom + PIVOT_PLATE_H / 2.0
    )
    plate.merge(pivot_plate)

    # Central shaft through the spring area connecting column top to pivot plate
    # This ensures structural connectivity of the base
    shaft_bottom = COLUMN_TOP - 0.005  # overlap into column top
    shaft_top = pp_bottom + 0.005  # overlap into pivot plate bottom
    shaft_height = shaft_top - shaft_bottom
    shaft = CylinderGeometry(COLUMN_R * 0.65, shaft_height, radial_segments=18).translate(
        0.0, 0.0, shaft_bottom + shaft_height / 2.0
    )
    plate.merge(shaft)

    # Gusset ribs at 45-degree offsets (avoiding beam plane at Y=0)
    # Start inside the column and end inside the pivot plate for connectivity.
    for i in range(4):
        angle = i * math.pi / 2.0 + math.pi / 4.0
        rib_bot_r = COLUMN_R  # at column surface
        rib_top_r = PIVOT_PLATE_R - 0.008
        rib_bot_x = math.cos(angle) * rib_bot_r
        rib_bot_y = math.sin(angle) * rib_bot_r
        rib_top_x = math.cos(angle) * rib_top_r
        rib_top_y = math.sin(angle) * rib_top_r
        rib = _tube_between(
            (rib_bot_x, rib_bot_y, COLUMN_TOP - 0.005),  # overlap into column top
            (rib_top_x, rib_top_y, pp_bottom + 0.005),  # overlap into pivot plate
            0.006,
            radial_segments=10,
        )
        plate.merge(rib)

    # Spring coils: torus rings around the shaft
    # Make wire radius large enough that adjacent coils overlap slightly
    spring_mesh = MeshGeometry()
    coil_spacing = (SPRING_TOP_Z - SPRING_BASE_Z) / SPRING_COILS
    # Wire radius needs to be > spacing/2 for coils to touch: spacing=0.032, so wire>0.016
    spring_wire = max(SPRING_WIRE, coil_spacing / 2.0 + 0.002)
    for i in range(SPRING_COILS):
        z = SPRING_BASE_Z + coil_spacing * (i + 0.5)
        coil = TorusGeometry(
            SPRING_R, spring_wire, radial_segments=12, tubular_segments=32
        ).translate(0.0, 0.0, z)
        spring_mesh.merge(coil)

    return plate, spring_mesh


def _molded_seat_mesh() -> MeshGeometry:
    """Build a round molded seat with dished surface and raised lip."""
    profile = [
        (0.001, 0.012),  # center of dish
        (0.04, 0.008),  # inner dish
        (0.08, 0.006),  # mid dish
        (0.105, 0.010),  # rising to rim
        (0.125, 0.020),  # approaching lip
        (0.140, SEAT_LIP_HEIGHT),  # lip peak
        (0.140, 0.0),  # outer wall bottom
        (0.001, 0.0),  # close bottom
    ]
    return LatheGeometry(profile, segments=32, closed=True)


def _beam_meshes() -> tuple[MeshGeometry, MeshGeometry, MeshGeometry]:
    """Build beam truss and handlebars in beam local frame (X along beam, pivot at origin).

    Returns (truss_mesh, handlebar_0, handlebar_1).
    """
    # Main top tube, full length
    truss = (
        CylinderGeometry(TUBE_R, BEAM_LEN, radial_segments=18)
        .rotate_y(math.pi / 2.0)
        .translate(0.0, 0.0, MAIN_Z)
    )

    # Central bearing hub: short cylinder sitting on the base pivot plate,
    # provides the physical pivot connection between beam and base.
    bearing = CylinderGeometry(0.032, 0.024, radial_segments=22).translate(
        0.0, 0.0, 0.010
    )
    truss.merge(bearing)
    # Weld post connecting bearing top to main tube bottom
    post_bottom = 0.022  # bearing top
    post_top = MAIN_Z - TUBE_R  # tube bottom = 0.04
    post_h = post_top - post_bottom
    weld_post = CylinderGeometry(0.010, post_h, radial_segments=10).translate(
        0.0, 0.0, post_bottom + post_h / 2.0
    )
    truss.merge(weld_post)

    for sx in (1.0, -1.0):
        # Diagonal brace from bearing area to mid-tube (triangulated truss)
        truss.merge(
            _tube_between(
                (sx * 0.05, 0.0, 0.014),
                (sx * 0.50, 0.0, MAIN_Z - TUBE_R + 0.002),
                BRACE_R,
            )
        )
        # Seat support: Y-shaped pair of tubes from main tube down to seat area
        for sy in (-0.09, 0.09):
            truss.merge(
                _tube_between(
                    (sx * 0.92, 0.0, MAIN_Z - TUBE_R + 0.002),
                    (sx * SEAT_X, sy, SEAT_Z + 0.020),
                    0.012,
                    radial_segments=10,
                )
            )

    # Handlebars
    handlebars: list[MeshGeometry] = []
    for sx in (1.0, -1.0):
        # Post overlaps the main tube for structural connectivity
        post = CylinderGeometry(HANDLE_R, 0.28, radial_segments=12).translate(
            sx * HANDLE_X, 0.0, MAIN_Z + 0.10
        )
        bar = (
            CylinderGeometry(HANDLE_R, 0.26, radial_segments=12)
            .rotate_x(math.pi / 2.0)
            .translate(sx * HANDLE_X, 0.0, HANDLE_TOP_Z)
        )
        grip_l = CylinderGeometry(HANDLE_R * 1.3, 0.04, radial_segments=10).translate(
            sx * HANDLE_X, -0.13, HANDLE_TOP_Z
        )
        grip_r = CylinderGeometry(HANDLE_R * 1.3, 0.04, radial_segments=10).translate(
            sx * HANDLE_X, 0.13, HANDLE_TOP_Z
        )
        hb = post.merge(bar).merge(grip_l).merge(grip_r)
        handlebars.append(hb)

    return truss, handlebars[0], handlebars[1]


def _bumper_mesh() -> MeshGeometry:
    """Rubber bumper: short cylinder that mounts under the beam end."""
    return CylinderGeometry(BUMPER_R, BUMPER_H, radial_segments=20)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="spring_assisted_seesaw")

    # --- Base pedestal with spring -------------------------------------------
    base = model.part("base")
    pedestal_mesh, spring_mesh = _base_meshes()
    base.visual(
        mesh_from_geometry(pedestal_mesh, "pedestal"),
        material=DARK_GREEN,
        name="pedestal",
    )
    base.visual(
        mesh_from_geometry(spring_mesh, "spring_coils"),
        material=STEEL_SPRING,
        name="spring_coils",
    )

    # --- Beam with molded seats ----------------------------------------------
    beam = model.part("beam")
    truss, hb0, hb1 = _beam_meshes()
    beam.visual(
        mesh_from_geometry(truss, "beam_truss"),
        material=BRIGHT_ORANGE,
        name="truss_tube",
    )
    beam.visual(
        mesh_from_geometry(hb0, "handlebar_0"),
        material=BRIGHT_ORANGE,
        name="handlebar_0",
    )
    beam.visual(
        mesh_from_geometry(hb1, "handlebar_1"),
        material=BRIGHT_ORANGE,
        name="handlebar_1",
    )

    # Molded seats with raised lips
    seat_mesh = _molded_seat_mesh()
    seat0 = seat_mesh.copy().translate(SEAT_X, 0.0, SEAT_Z)
    beam.visual(
        mesh_from_geometry(seat0, "seat_0"),
        material=MOLDED_GRAY,
        name="molded_seat_0",
    )
    seat1 = seat_mesh.copy().translate(-SEAT_X, 0.0, SEAT_Z)
    beam.visual(
        mesh_from_geometry(seat1, "seat_1"),
        material=MOLDED_GRAY,
        name="molded_seat_1",
    )

    # --- Rubber bumpers on prismatic joints ----------------------------------
    bumper_mesh = _bumper_mesh()

    bumper_0 = model.part("bumper_0")
    bumper_0.visual(
        mesh_from_geometry(bumper_mesh.copy(), "bumper_0_body"),
        material=BLACK_RUBBER,
        name="bumper_body",
    )

    bumper_1 = model.part("bumper_1")
    bumper_1.visual(
        mesh_from_geometry(bumper_mesh.copy(), "bumper_1_body"),
        material=BLACK_RUBBER,
        name="bumper_body",
    )

    # --- Articulations -------------------------------------------------------

    # Beam pivot: revolute at top of base, horizontal Y axis
    model.articulation(
        "beam_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=150.0, velocity=2.5, lower=-TILT, upper=TILT
        ),
    )

    # Bumper mounting: position bumper so its top contacts/overlaps the seat
    # bottom surface by ~2mm (intentional mounting contact).
    # At BUMPER_X, the seat bottom is approximately at SEAT_Z + 0.008 in beam local.
    seat_bottom_at_bumper = SEAT_Z + 0.008  # approximate seat bottom z at bumper x
    bumper_center_z = seat_bottom_at_bumper - BUMPER_H / 2.0 + 0.002

    model.articulation(
        "bumper_0_compress",
        ArticulationType.PRISMATIC,
        parent=beam,
        child=bumper_0,
        origin=Origin(xyz=(BUMPER_X, 0.0, bumper_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=500.0, velocity=0.10, lower=0.0, upper=0.025
        ),
    )

    model.articulation(
        "bumper_1_compress",
        ArticulationType.PRISMATIC,
        parent=beam,
        child=bumper_1,
        origin=Origin(xyz=(-BUMPER_X, 0.0, bumper_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=500.0, velocity=0.10, lower=0.0, upper=0.025
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    beam = object_model.get_part("beam")
    bumper_0 = object_model.get_part("bumper_0")
    bumper_1 = object_model.get_part("bumper_1")
    beam_pivot = object_model.get_articulation("beam_pivot")
    bump0_joint = object_model.get_articulation("bumper_0_compress")
    bump1_joint = object_model.get_articulation("bumper_1_compress")

    # Bearing hub: the beam's central bearing sits on the base pivot plate,
    # creating a small intentional overlap at the pivot interface.
    ctx.allow_overlap(
        beam,
        base,
        elem_a="truss_tube",
        elem_b="pedestal",
        reason="Beam bearing hub intentionally overlaps base pivot plate at the revolute pivot interface.",
    )

    # Bumper mounting: each bumper pad overlaps the seat bottom surface
    # to represent a bolted-on rubber end stop.
    ctx.allow_overlap(
        beam,
        bumper_0,
        elem_a="molded_seat_0",
        elem_b="bumper_body",
        reason="Bumper pad intentionally overlaps seat bottom for bolted mounting under the beam end.",
    )
    ctx.allow_overlap(
        beam,
        bumper_1,
        elem_a="molded_seat_1",
        elem_b="bumper_body",
        reason="Bumper pad intentionally overlaps seat bottom for bolted mounting under the beam end.",
    )

    # --- Base structure checks -----------------------------------------------
    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "base is a pedestal about 0.65 m tall",
        base_aabb is not None and 0.60 <= base_aabb[1][2] <= 0.75,
        details=f"base aabb={base_aabb}",
    )
    ctx.check(
        "base rests on the ground plane",
        base_aabb is not None and -0.01 <= base_aabb[0][2] <= 0.03,
        details=f"base aabb={base_aabb}",
    )

    # Spring coils visible on the base
    spring_aabb = ctx.part_element_world_aabb(base, elem="spring_coils")
    ctx.check(
        "spring coils are visible on the base column",
        spring_aabb is not None
        and spring_aabb[0][2] > 0.30
        and spring_aabb[1][2] < PIVOT_Z,
        details=f"spring aabb={spring_aabb}",
    )

    # --- Beam pivot checks ---------------------------------------------------
    pivot_lim = beam_pivot.motion_limits
    ctx.check(
        "beam pivot rocks +/- 18 degrees",
        pivot_lim is not None
        and abs(pivot_lim.lower + TILT) < 1e-6
        and abs(pivot_lim.upper - TILT) < 1e-6,
        details=f"limits=({pivot_lim.lower if pivot_lim else None}, {pivot_lim.upper if pivot_lim else None})",
    )

    # --- Molded seat checks --------------------------------------------------
    for end, name in ((0, "molded_seat_0"), (1, "molded_seat_1")):
        seat_aabb = ctx.part_element_world_aabb(beam, elem=name)
        ctx.check(
            f"molded seat {end} exists near the beam end",
            seat_aabb is not None and seat_aabb[1][2] > 0.55,
            details=f"seat aabb={seat_aabb}",
        )
        if seat_aabb is not None:
            seat_height = seat_aabb[1][2] - seat_aabb[0][2]
            ctx.check(
                f"molded seat {end} has raised lip (height > 0.025 m)",
                seat_height > 0.025,
                details=f"seat height={seat_height:.4f}",
            )

    # --- Handlebar checks ----------------------------------------------------
    for end in (0, 1):
        handle = ctx.part_element_world_aabb(beam, elem=f"handlebar_{end}")
        seat = ctx.part_element_world_aabb(beam, elem=f"molded_seat_{end}")
        ok = handle is not None and seat is not None
        ctx.check(
            f"handlebar {end} and molded seat {end} both exist",
            ok,
            details=f"handle={handle}, seat={seat}",
        )
        if ok:
            hcz = (handle[0][2] + handle[1][2]) / 2.0
            scz = (seat[0][2] + seat[1][2]) / 2.0
            ctx.check(
                f"handlebar {end} stands above its seat",
                hcz > scz + 0.10,
                details=f"handle center z={hcz:.3f}, seat center z={scz:.3f}",
            )

    # --- Bumper checks -------------------------------------------------------
    for bumper, joint, jname in (
        (bumper_0, bump0_joint, "bumper_0_compress"),
        (bumper_1, bump1_joint, "bumper_1_compress"),
    ):
        lim = joint.motion_limits
        ctx.check(
            f"{jname} is prismatic with 25 mm compression range",
            lim is not None
            and abs(lim.lower) < 1e-6
            and abs(lim.upper - 0.025) < 1e-3,
            details=f"limits=({lim.lower if lim else None}, {lim.upper if lim else None})",
        )
        bump_aabb = ctx.part_world_aabb(bumper)
        ctx.check(
            f"{bumper.name} exists below its beam end",
            bump_aabb is not None and bump_aabb[0][2] > 0.05,
            details=f"bumper aabb={bump_aabb}",
        )

    # Bearing hub rides on the pivot plate
    ctx.expect_contact(
        beam,
        base,
        elem_a="truss_tube",
        elem_b="pedestal",
        name="beam bearing hub rides on base pivot plate",
    )

    # Bumper is mounted in contact with beam underside (seat bottom surface)
    ctx.expect_contact(
        bumper_0,
        beam,
        elem_a="bumper_body",
        elem_b="molded_seat_0",
        name="bumper_0 mounted in contact with seat bottom",
    )
    ctx.expect_contact(
        bumper_1,
        beam,
        elem_a="bumper_body",
        elem_b="molded_seat_1",
        name="bumper_1 mounted in contact with seat bottom",
    )

    # Bumper compression: positive q moves bumper upward
    rest_pos_0 = ctx.part_world_position(bumper_0)
    with ctx.pose({bump0_joint: 0.025}):
        compressed_pos_0 = ctx.part_world_position(bumper_0)
        ctx.check(
            "bumper_0 moves upward when compressed",
            rest_pos_0 is not None
            and compressed_pos_0 is not None
            and compressed_pos_0[2] > rest_pos_0[2] + 0.020,
            details=f"rest={rest_pos_0}, compressed={compressed_pos_0}",
        )

    # --- Decisive pose checks ------------------------------------------------
    rest_seat0 = ctx.part_element_world_aabb(beam, elem="molded_seat_0")
    rest_seat1 = ctx.part_element_world_aabb(beam, elem="molded_seat_1")
    with ctx.pose({beam_pivot: TILT}):
        tilt_seat0 = ctx.part_element_world_aabb(beam, elem="molded_seat_0")
        tilt_seat1 = ctx.part_element_world_aabb(beam, elem="molded_seat_1")
        beam_aabb = ctx.part_world_aabb(beam)
        ctx.check(
            "beam seesaws: one seat drops, the opposite seat rises",
            rest_seat0 is not None
            and tilt_seat0 is not None
            and rest_seat1 is not None
            and tilt_seat1 is not None
            and tilt_seat0[0][2] < rest_seat0[0][2] - 0.25
            and tilt_seat1[0][2] > rest_seat1[0][2] + 0.25,
            details=f"seat0 {rest_seat0} -> {tilt_seat0}, seat1 {rest_seat1} -> {tilt_seat1}",
        )
        ctx.check(
            "tilted beam stays clear of the ground",
            beam_aabb is not None and beam_aabb[0][2] > 0.01,
            details=f"beam aabb={beam_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
