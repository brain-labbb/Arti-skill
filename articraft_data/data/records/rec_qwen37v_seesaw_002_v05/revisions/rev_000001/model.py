from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CapsuleGeometry,
    CylinderGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ----------------------------------------------------------------------------
# Heavy commercial steel beam playground seesaw with rubber bumpers.
#
# Layout (world frame, Z up, base centered on the origin):
# - Steel pedestal base: ground plate plus two upstanding channel walls that
#   house a vertical spring. About 0.70 m to the pivot height.
# - Spring carriage: a plate riding on a prismatic joint inside the channel,
#   carried by a visible coil spring. The spring compresses downward under load.
# - Heavy steel beam: a rectangular tube ~2.6 m long rocking on the carriage
#   pivot. Each end carries a molded seat with raised lip, an upright handle
#   with a rounded rubber grip, and a rubber bumper underneath.
# - Articulation:
#   1. spring (PRISMATIC, Z axis, 0 to -0.04 m compression)
#   2. beam_pivot (REVOLUTE, Y axis, +/- 18 degrees)
# ----------------------------------------------------------------------------

BEAM_LEN = 2.60
BEAM_W = 0.10
BEAM_H = 0.08
PIVOT_Z = 0.70  # pivot height above ground
BASE_W = 0.50
BASE_D = 0.30
BASE_PLATE_H = 0.025
WALL_H = PIVOT_Z - BASE_PLATE_H - 0.06  # walls stop 6 cm below pivot
WALL_T = 0.030

SPRING_R = 0.055  # spring coil outer radius
SPRING_WIRE = 0.012
SPRING_COILS = 5
SPRING_BOTTOM = BASE_PLATE_H  # spring rests on the ground plate
CARRIAGE_BOTTOM = PIVOT_Z - 0.06  # where the carriage plate starts
SPRING_TOP = CARRIAGE_BOTTOM - SPRING_WIRE  # spring path ends so wire surface just touches plate
SPRING_TRAVEL = 0.04  # max compression

SEAT_X = 1.15  # seat center along beam from pivot
SEAT_W = 0.28
SEAT_D = 0.30
SEAT_RIM_H = 0.025
SEAT_BOWL_DEPTH = 0.012
SEAT_BASE_H = 0.015

HANDLE_X = 0.85
HANDLE_POST_R = 0.014
HANDLE_POST_H = 0.28
HANDLE_GRIP_R = 0.022
HANDLE_GRIP_LEN = 0.18

BUMPER_X = 1.22
BUMPER_R = 0.035
BUMPER_H = 0.045

TILT = math.radians(18.0)

STEEL_GRAY = Material("steel_gray", rgba=(0.38, 0.40, 0.42, 1.0))
DARK_STEEL = Material("dark_steel", rgba=(0.22, 0.23, 0.25, 1.0))
YELLOW_BEAM = Material("safety_yellow", rgba=(0.92, 0.78, 0.10, 1.0))
SEAT_GREEN = Material("molded_green", rgba=(0.18, 0.52, 0.28, 1.0))
RUBBER_BLACK = Material("rubber_black", rgba=(0.08, 0.08, 0.09, 1.0))
GRIP_RED = Material("rubber_red", rgba=(0.72, 0.15, 0.12, 1.0))


def _helix_points(
    radius: float,
    z_bottom: float,
    z_top: float,
    coils: int,
    samples_per_coil: int = 24,
) -> list[tuple[float, float, float]]:
    """Generate a helical path for a coil spring."""
    total = coils * samples_per_coil
    height = z_top - z_bottom
    pts: list[tuple[float, float, float]] = []
    for i in range(total + 1):
        t = i / total
        angle = coils * 2.0 * math.pi * t
        pts.append((radius * math.cos(angle), radius * math.sin(angle), z_bottom + height * t))
    return pts


def _build_spring_mesh() -> MeshGeometry:
    """Coil spring mesh centered on Z axis from SPRING_BOTTOM to SPRING_TOP."""
    pts = _helix_points(SPRING_R, SPRING_BOTTOM, SPRING_TOP, SPRING_COILS)
    return tube_from_spline_points(
        pts,
        radius=SPRING_WIRE,
        samples_per_segment=4,
        radial_segments=10,
        cap_ends=True,
    )


def _build_molded_seat() -> cq.Workplane:
    """Molded seat: a shallow rectangular bowl with a raised rim."""
    # Outer rim box
    rim = cq.Workplane("XY").box(SEAT_W, SEAT_D, SEAT_RIM_H + SEAT_BASE_H)
    # Cut a concave bowl from the top
    bowl_w = SEAT_W - 0.04
    bowl_d = SEAT_D - 0.04
    bowl = (
        cq.Workplane("XY")
        .workplane(offset=(SEAT_RIM_H + SEAT_BASE_H) / 2.0 - SEAT_BOWL_DEPTH)
        .box(bowl_w, bowl_d, SEAT_BOWL_DEPTH * 2.0 + 0.001)
    )
    seat = rim.cut(bowl)
    # Round the outer vertical edges
    seat = seat.edges("|Z").fillet(0.02)
    return seat


def _build_beam_body() -> cq.Workplane:
    """Heavy rectangular beam, origin at center (pivot point)."""
    beam = cq.Workplane("XY").box(BEAM_LEN, BEAM_W, BEAM_H)
    # Round long edges slightly
    beam = beam.edges("|X").fillet(0.005)
    return beam


def _build_base_mesh() -> MeshGeometry:
    """Pedestal base: ground plate + two channel walls + gussets."""
    geom = MeshGeometry()
    # Ground plate
    base_plate = BoxGeometry((BASE_W, BASE_D, BASE_PLATE_H)).translate(
        0.0, 0.0, BASE_PLATE_H / 2.0
    )
    geom.merge(base_plate)
    # Two channel walls (left and right, along Y)
    for sx in (-1.0, 1.0):
        wall = BoxGeometry((WALL_T, BASE_D, WALL_H)).translate(
            sx * (BASE_W / 2.0 - WALL_T / 2.0), 0.0, BASE_PLATE_H + WALL_H / 2.0
        )
        geom.merge(wall)
    # Bottom cross-bar tying walls together at mid-height
    mid_bar = BoxGeometry((BASE_W - 2 * WALL_T + 0.005, 0.05, 0.035)).translate(
        0.0, 0.0, BASE_PLATE_H + WALL_H * 0.35
    )
    geom.merge(mid_bar)
    # Gussets at base of walls (contact the ground plate)
    for sx in (-1.0, 1.0):
        gusset = BoxGeometry((0.06, BASE_D - 0.02, 0.06)).translate(
            sx * (BASE_W / 2.0 - WALL_T - 0.03),
            0.0,
            BASE_PLATE_H + 0.03,
        )
        geom.merge(gusset)
    return geom


def _build_carriage_mesh() -> MeshGeometry:
    """Spring carriage plate + vertical stem + pivot bearing block."""
    geom = MeshGeometry()
    # Carriage plate that slides in the channel
    plate_w = BASE_W - 2 * WALL_T - 0.014  # clearance fit
    plate_z_bottom = CARRIAGE_BOTTOM
    plate = BoxGeometry((plate_w, BASE_D - 0.02, 0.020)).translate(
        0.0, 0.0, plate_z_bottom + 0.01
    )
    geom.merge(plate)
    # Slider shoes that extend slightly below the plate to contact the channel walls
    # These embed 2mm into the wall top for a positive connection.
    # Connecting ribs bridge the gap between the plate edge and the shoe.
    for sx in (-1.0, 1.0):
        shoe = BoxGeometry((WALL_T, BASE_D - 0.04, 0.022)).translate(
            sx * (BASE_W / 2.0 - WALL_T / 2.0),
            0.0,
            plate_z_bottom - 0.001,
        )
        geom.merge(shoe)
        # Connecting rib from plate edge to shoe (bridges the ~7mm gap)
        rib_x_center = sx * (plate_w / 2.0 + (BASE_W / 2.0 - WALL_T / 2.0 - plate_w / 2.0) / 2.0)
        rib_w = abs(BASE_W / 2.0 - WALL_T / 2.0 - plate_w / 2.0) + 0.006
        rib = BoxGeometry((rib_w, 0.04, 0.020)).translate(
            rib_x_center,
            0.0,
            plate_z_bottom + 0.01,
        )
        geom.merge(rib)
    # Vertical stem from plate up to bearing
    stem_top = PIVOT_Z - 0.03
    stem_h = stem_top - (plate_z_bottom + 0.02)
    if stem_h > 0.001:
        stem = CylinderGeometry(0.025, stem_h, radial_segments=16).translate(
            0.0, 0.0, plate_z_bottom + 0.02 + stem_h / 2.0
        )
        geom.merge(stem)
    # Pivot bearing block at the pivot height
    bearing = CylinderGeometry(0.04, 0.06, radial_segments=20).translate(
        0.0, 0.0, PIVOT_Z
    )
    geom.merge(bearing)
    # Axle stub through bearing (along Y, shorter than channel width)
    axle_len = 0.20
    axle = CylinderGeometry(0.018, axle_len, radial_segments=16)
    axle.rotate_x(math.pi / 2.0)
    axle.translate(0.0, 0.0, PIVOT_Z)
    geom.merge(axle)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_spring_seesaw")

    # --- Base (static pedestal) -----------------------------------------------
    base = model.part("base")
    base.visual(
        mesh_from_geometry(_build_base_mesh(), "base_pedestal"),
        material=STEEL_GRAY,
        name="pedestal",
    )
    # Coil spring (attached to base visually, but functionally under carriage)
    base.visual(
        mesh_from_geometry(_build_spring_mesh(), "coil_spring"),
        material=DARK_STEEL,
        name="coil_spring",
    )

    # --- Spring carriage (prismatic joint, moves vertically) ------------------
    carriage = model.part("spring_carriage")
    carriage.visual(
        mesh_from_geometry(_build_carriage_mesh(), "carriage_assembly"),
        material=STEEL_GRAY,
        name="carriage_plate",
    )

    # --- Beam with seats, handles, bumpers ------------------------------------
    beam = model.part("beam")
    # Main beam body (solid rectangular steel beam)
    beam_shape = _build_beam_body()
    beam.visual(
        mesh_from_cadquery(beam_shape, "beam_tube"),
        material=YELLOW_BEAM,
        name="beam_tube",
    )

    # Molded seats at each end
    seat_shape = _build_molded_seat()
    for sx, suffix in ((1.0, "0"), (-1.0, "1")):
        seat_origin = Origin(xyz=(sx * SEAT_X, 0.0, BEAM_H / 2.0 + (SEAT_RIM_H + SEAT_BASE_H) / 2.0))
        beam.visual(
            mesh_from_cadquery(seat_shape, f"molded_seat_{suffix}"),
            origin=seat_origin,
            material=SEAT_GREEN,
            name=f"molded_seat_{suffix}",
        )

    # Handle posts and rounded grips at each end
    for sx, suffix in ((1.0, "0"), (-1.0, "1")):
        handle_geom = MeshGeometry()
        # Mounting flange to ensure solid contact with beam top surface
        flange = BoxGeometry((0.05, BEAM_W, 0.008)).translate(
            sx * HANDLE_X, 0.0, BEAM_H / 2.0 + 0.004
        )
        handle_geom.merge(flange)
        # Vertical post
        post = CylinderGeometry(HANDLE_POST_R, HANDLE_POST_H, radial_segments=14)
        post.translate(sx * HANDLE_X, 0.0, BEAM_H / 2.0 + 0.008 + HANDLE_POST_H / 2.0)
        handle_geom.merge(post)
        # Rounded rubber grip (capsule at top of post) - the grip overlaps the post top
        grip = CapsuleGeometry(HANDLE_GRIP_R, HANDLE_GRIP_LEN, radial_segments=16, height_segments=6)
        grip.rotate_x(math.pi / 2.0)  # orient horizontally along Y
        grip.translate(sx * HANDLE_X, 0.0, BEAM_H / 2.0 + 0.008 + HANDLE_POST_H)
        handle_geom.merge(grip)
        beam.visual(
            mesh_from_geometry(handle_geom, f"handle_{suffix}"),
            material=GRIP_RED,
            name=f"handle_{suffix}",
        )

    # Rubber bumpers underneath each end
    for sx, suffix in ((1.0, "0"), (-1.0, "1")):
        bumper_geom = MeshGeometry()
        # Mounting bracket to ensure solid contact with beam bottom surface
        bracket = BoxGeometry((0.06, BEAM_W, 0.006)).translate(
            sx * BUMPER_X, 0.0, -(BEAM_H / 2.0 + 0.003)
        )
        bumper_geom.merge(bracket)
        # Rubber bumper cylinder below the bracket
        bumper = CylinderGeometry(BUMPER_R, BUMPER_H, radial_segments=18)
        bumper.translate(sx * BUMPER_X, 0.0, -(BEAM_H / 2.0 + 0.006 + BUMPER_H / 2.0))
        bumper_geom.merge(bumper)
        beam.visual(
            mesh_from_geometry(bumper_geom, f"bumper_{suffix}"),
            material=RUBBER_BLACK,
            name=f"bumper_{suffix}",
        )

    # --- Articulations --------------------------------------------------------
    # Spring: prismatic, carriage moves vertically relative to base
    model.articulation(
        "spring",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2000.0,
            velocity=0.5,
            lower=-SPRING_TRAVEL,
            upper=0.0,
        ),
    )

    # Beam pivot: revolute, beam rocks on the carriage
    model.articulation(
        "beam_pivot",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=150.0,
            velocity=2.5,
            lower=-TILT,
            upper=TILT,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    carriage = object_model.get_part("spring_carriage")
    beam = object_model.get_part("beam")
    spring = object_model.get_articulation("spring")
    pivot = object_model.get_articulation("beam_pivot")

    # --- Intentional overlap: bearing block wraps the beam pivot area ----------
    ctx.allow_overlap(
        carriage,
        beam,
        elem_a="carriage_plate",
        elem_b="beam_tube",
        reason="Carriage bearing block intentionally wraps the beam pivot region to support the rocking axle.",
    )
    ctx.expect_contact(
        carriage,
        beam,
        elem_a="carriage_plate",
        elem_b="beam_tube",
        name="bearing supports the beam at the pivot",
    )

    # Carriage slider shoes embed slightly into channel wall tops for support
    ctx.allow_overlap(
        base,
        carriage,
        elem_a="pedestal",
        elem_b="carriage_plate",
        reason="Carriage slider shoes intentionally embed into channel wall tops to maintain physical support connection.",
    )
    ctx.expect_contact(
        base,
        carriage,
        name="carriage rides on the channel walls",
    )

    # --- Joint existence and type checks ---
    ctx.check(
        "spring joint is prismatic",
        spring.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={spring.articulation_type}",
    )
    ctx.check(
        "beam_pivot joint is revolute",
        pivot.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={pivot.articulation_type}",
    )

    # --- Spring limits ---
    s_lim = spring.motion_limits
    ctx.check(
        "spring allows 0.04 m compression",
        s_lim is not None and abs(s_lim.lower + SPRING_TRAVEL) < 1e-6 and abs(s_lim.upper) < 1e-6,
        details=f"limits=({s_lim.lower if s_lim else None}, {s_lim.upper if s_lim else None})",
    )

    # --- Beam pivot limits ---
    p_lim = pivot.motion_limits
    ctx.check(
        "beam pivots +/- 18 degrees",
        p_lim is not None and abs(p_lim.lower + TILT) < 1e-6 and abs(p_lim.upper - TILT) < 1e-6,
        details=f"limits=({p_lim.lower if p_lim else None}, {p_lim.upper if p_lim else None})",
    )

    # --- Molded seats exist at both ends ---
    for suffix in ("0", "1"):
        seat_aabb = ctx.part_element_world_aabb(beam, elem=f"molded_seat_{suffix}")
        ctx.check(
            f"molded seat {suffix} exists above the beam",
            seat_aabb is not None and seat_aabb[0][2] > 0.60,
            details=f"seat_{suffix} aabb={seat_aabb}",
        )

    # --- Rounded handle grips exist at both ends ---
    for suffix in ("0", "1"):
        handle_aabb = ctx.part_element_world_aabb(beam, elem=f"handle_{suffix}")
        ctx.check(
            f"handle {suffix} with rounded grip exists",
            handle_aabb is not None,
            details=f"handle_{suffix} aabb={handle_aabb}",
        )
        if handle_aabb is not None:
            # Handle extends well above the beam top
            ctx.check(
                f"handle {suffix} stands upright above the beam",
                handle_aabb[1][2] > PIVOT_Z + 0.20,
                details=f"handle top z={handle_aabb[1][2]:.3f}",
            )

    # --- Rubber bumpers exist under beam ends ---
    for suffix in ("0", "1"):
        bumper_aabb = ctx.part_element_world_aabb(beam, elem=f"bumper_{suffix}")
        ctx.check(
            f"rubber bumper {suffix} exists under beam end",
            bumper_aabb is not None,
            details=f"bumper_{suffix} aabb={bumper_aabb}",
        )

    # --- Coil spring is visible on the base ---
    spring_aabb = ctx.part_element_world_aabb(base, elem="coil_spring")
    ctx.check(
        "coil spring is visible on the base",
        spring_aabb is not None and spring_aabb[0][2] > 0.0 and spring_aabb[1][2] > 0.4,
        details=f"spring aabb={spring_aabb}",
    )

    # --- Prismatic spring actually moves carriage down ---
    rest_carriage = ctx.part_world_position(carriage)
    with ctx.pose({spring: -SPRING_TRAVEL}):
        compressed_carriage = ctx.part_world_position(carriage)
        ctx.check(
            "spring compresses carriage downward",
            rest_carriage is not None
            and compressed_carriage is not None
            and compressed_carriage[2] < rest_carriage[2] - 0.02,
            details=f"rest={rest_carriage}, compressed={compressed_carriage}",
        )

    # --- Beam rocks independently of spring ---
    # With axis=(0,1,0), positive rotation drops the +X end and raises the -X end.
    rest_seat0 = ctx.part_element_world_aabb(beam, elem="molded_seat_0")
    rest_seat1 = ctx.part_element_world_aabb(beam, elem="molded_seat_1")
    with ctx.pose({pivot: TILT}):
        tilt_seat0 = ctx.part_element_world_aabb(beam, elem="molded_seat_0")
        tilt_seat1 = ctx.part_element_world_aabb(beam, elem="molded_seat_1")
        ctx.check(
            "beam rocks: +X seat drops and -X seat rises at positive tilt",
            rest_seat0 is not None
            and tilt_seat0 is not None
            and rest_seat1 is not None
            and tilt_seat1 is not None
            and tilt_seat0[1][2] < rest_seat0[1][2] - 0.30
            and tilt_seat1[1][2] > rest_seat1[1][2] + 0.30,
            details=f"seat0 {rest_seat0}->{tilt_seat0}, seat1 {rest_seat1}->{tilt_seat1}",
        )

    # --- Beam stays clear of ground at full tilt ---
    with ctx.pose({pivot: TILT}):
        beam_aabb = ctx.part_world_aabb(beam)
        ctx.check(
            "beam stays clear of ground at full tilt",
            beam_aabb is not None and beam_aabb[0][2] > 0.01,
            details=f"beam aabb={beam_aabb}",
        )

    # --- Base rests on ground ---
    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "base rests on the ground",
        base_aabb is not None and -0.01 <= base_aabb[0][2] <= 0.015,
        details=f"base aabb={base_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
