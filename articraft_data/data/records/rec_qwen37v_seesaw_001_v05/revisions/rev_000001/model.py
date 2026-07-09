from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Heavy commercial spring-loaded playground seesaw
#
# World frame: beam runs along X, pivot axis along Y, Z up.
# - Two bent galvanized-steel tube arches (~50 mm dia) form an A-shaped
#   saddle; the apex carries the pivot mechanism.
# - A spring carriage rides on a prismatic joint (vertical compression)
#   atop the base. A heavy coil spring sits between the base plate and
#   the carriage plate, providing bounce damping.
# - The rocking beam is a 3.0 m heavy commercial steel bar (100 x 50 mm)
#   painted industrial grey with rust streaks. Each end carries a molded
#   seat with raised lips, an inverted-U grab handle with rounded grip
#   balls, and a curved rubber bumper.
# - Articulation chain:
#     base -> (prismatic, spring_compress) -> spring_carriage
#     spring_carriage -> (revolute, beam_pivot) -> beam
#   Spring compression: 0 to 0.06 m downward.
#   Beam rocking: +/- 20 degrees.
# ---------------------------------------------------------------------------

BEAM_HALF = 1.50  # 3.0 m beam
BEAM_W = 0.10     # heavy commercial 100 mm wide
BEAM_T = 0.050    # 50 mm thick
PIVOT_Z = 0.76    # axle height

ARCH_FOOT_X = 0.66
ARCH_FOOT_Y = 0.34
ARCH_APEX_Y = 0.05
ARCH_FOOT_Z = 0.028
TUBE_R = 0.025

AXLE_R = 0.018
AXLE_LEN = 0.24

# Beam-local frame: origin at the axle center; bar bottom sits above.
BAR_BOT = 0.055
BAR_CTR = BAR_BOT + BEAM_T / 2.0
BAR_TOP = BAR_BOT + BEAM_T

SEAT_X = 1.28
HANDLE_X = 1.02
BUMPER_X = 1.42
TILT = math.radians(20.0)

# Spring geometry
SPRING_R = 0.038       # coil radius
SPRING_WIRE_R = 0.007  # wire radius
SPRING_COILS = 5
SPRING_FREE_H = 0.18   # free height
SPRING_BASE_Z = 0.50   # bottom of spring in world frame

# Spring carriage
CARRIAGE_PLATE_THICK = 0.016
SPRING_COMPRESS_MAX = 0.06  # max compression travel

# Carriage plate center in world frame (sits on top of spring)
CARRIAGE_PLATE_WORLD_Z = SPRING_BASE_Z + SPRING_FREE_H + CARRIAGE_PLATE_THICK / 2.0
# In carriage local frame (origin at PIVOT_Z):
CARRIAGE_PLATE_LOCAL_Z = CARRIAGE_PLATE_WORLD_Z - PIVOT_Z


def _arch_points(side: float) -> list[tuple[float, float, float]]:
    """Centerline of one bent-tube arch."""
    pts: list[tuple[float, float, float]] = []
    rise = PIVOT_Z - ARCH_FOOT_Z
    for i in range(11):
        t = -1.0 + 0.2 * i
        s = 1.0 - t * t
        x = ARCH_FOOT_X * t
        z = ARCH_FOOT_Z + rise * s
        y = side * ARCH_FOOT_Y + (-side * ARCH_APEX_Y - side * ARCH_FOOT_Y) * s
        pts.append((x, y, z))
    return pts


def _spring_helix_points() -> list[tuple[float, float, float]]:
    """Helical centerline for the coil spring, in base frame coordinates."""
    pts: list[tuple[float, float, float]] = []
    n_per_coil = 28
    total = SPRING_COILS * n_per_coil
    for i in range(total + 1):
        t = i / total
        angle = 2.0 * math.pi * SPRING_COILS * t
        x = SPRING_R * math.cos(angle)
        y = SPRING_R * math.sin(angle)
        z = SPRING_BASE_Z + SPRING_FREE_H * t
        pts.append((x, y, z))
    return pts


def _handle_points(x: float) -> list[tuple[float, float, float]]:
    """Inverted-U grab-handle rod centerline, plane across the beam (YZ)."""
    half_w = 0.040
    leg_bot = BAR_TOP - 0.010
    arc_z = 0.285
    pts: list[tuple[float, float, float]] = [
        (x, -half_w, leg_bot),
        (x, -half_w, 0.200),
    ]
    for k in range(7):
        a = math.pi * k / 6.0
        pts.append((x, -half_w * math.cos(a), arc_z + half_w * math.sin(a)))
    pts.append((x, half_w, 0.200))
    pts.append((x, half_w, leg_bot))
    return pts


def _bumper_geometry(x: float, index: int):
    """Curved tire-section bumper: half-annulus shell extruded across the beam."""
    r_out = 0.070
    r_in = 0.052
    profile: list[tuple[float, float]] = []
    n = 20
    for k in range(n + 1):
        a = math.pi + math.pi * k / n
        profile.append((r_out * math.cos(a), r_out * math.sin(a)))
    for k in range(n + 1):
        a = 2.0 * math.pi - math.pi * k / n
        profile.append((r_in * math.cos(a), r_in * math.sin(a)))
    geom = ExtrudeGeometry(profile, 0.12, cap=True, center=True)
    geom.rotate_x(math.pi / 2.0)
    geom.translate(x, 0.0, BAR_BOT + 0.002)
    return mesh_from_geometry(geom, f"seesaw_bumper_{index}")


def _gusset_geometry():
    """Triangular gusset plate joining the beam bar to the pivot sleeve."""
    profile = [(-0.12, 0.060), (0.12, 0.060), (0.0, 0.022)]
    geom = ExtrudeGeometry(profile, 0.024, cap=True, center=True)
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, "seesaw_gusset_plate")


def _molded_seat_shape():
    """Molded seat with raised perimeter lip, built in CadQuery.

    The seat is a flat plate (0.30 x 0.24 x 0.018) with a raised rectangular
    rim (0.015 tall, 0.020 wide) around the top edge. Origin at plate center.
    """
    seat = (
        cq.Workplane("XY")
        .box(0.30, 0.24, 0.018)
        .faces(">Z").workplane()
        .rect(0.30, 0.24)
        .rect(0.26, 0.20)
        .extrude(0.015)
    )
    return seat


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_spring_seesaw")

    # Materials
    galvanized = model.material("weathered_galvanized", rgba=(0.55, 0.58, 0.56, 1.0))
    rust = model.material("rust_steel", rgba=(0.42, 0.25, 0.13, 1.0))
    steel_grey = model.material("commercial_steel", rgba=(0.38, 0.40, 0.42, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.28, 0.30, 0.32, 1.0))
    pale_steel = model.material("pale_weathered_steel", rgba=(0.70, 0.66, 0.58, 1.0))
    rubber = model.material("black_rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    seat_green = model.material("molded_green", rgba=(0.18, 0.42, 0.22, 1.0))
    spring_blue = model.material("spring_steel", rgba=(0.30, 0.45, 0.60, 1.0))
    grip_red = model.material("grip_rubber", rgba=(0.55, 0.12, 0.10, 1.0))

    # --------------------------------------------------------------- base ---
    base = model.part("arched_base")
    for i, side in enumerate((1.0, -1.0)):
        base.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _arch_points(side),
                    radius=TUBE_R,
                    samples_per_segment=8,
                    radial_segments=18,
                    cap_ends=True,
                ),
                f"seesaw_arch_{i}",
            ),
            material=galvanized,
            name=f"arch_{i}",
        )

    # Cross-bar connecting both arches at the spring mount height.
    # At z=SPRING_BASE_Z, compute arch centerline positions.
    _rise = PIVOT_Z - ARCH_FOOT_Z
    _s_cross = (SPRING_BASE_Z - ARCH_FOOT_Z) / _rise
    _t_cross = math.sqrt(max(0.0, 1.0 - _s_cross))
    _cross_x = ARCH_FOOT_X * _t_cross
    _cross_y0 = ARCH_FOOT_Y + (-ARCH_APEX_Y - ARCH_FOOT_Y) * _s_cross
    cross_bar_pts = [
        (_cross_x, _cross_y0, SPRING_BASE_Z),
        (0.0, 0.0, SPRING_BASE_Z),
        (-_cross_x, -_cross_y0, SPRING_BASE_Z),
    ]
    base.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                cross_bar_pts,
                radius=0.018,
                samples_per_segment=4,
                radial_segments=14,
                cap_ends=True,
            ),
            "cross_bar_lower",
        ),
        material=galvanized,
        name="cross_bar",
    )

    # Spring base mount plate (flat disk at base of spring, seated on cross-bar)
    base.visual(
        Cylinder(radius=0.06, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, SPRING_BASE_Z - 0.004)),
        material=dark_steel,
        name="spring_mount_plate",
    )

    # Coil spring (static visual on base)
    base.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                _spring_helix_points(),
                radius=SPRING_WIRE_R,
                samples_per_segment=6,
                radial_segments=12,
                cap_ends=True,
            ),
            "spring_coil",
        ),
        material=spring_blue,
        name="spring_coil",
    )

    # -------------------------------------------------------- carriage ---
    carriage = model.part("spring_carriage")

    # Carriage plate sits on top of the spring
    carriage.visual(
        Cylinder(radius=0.065, length=CARRIAGE_PLATE_THICK),
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_PLATE_LOCAL_Z)),
        material=dark_steel,
        name="carriage_plate",
    )

    # Central post connecting carriage plate to the axle region
    _post_bot_z = CARRIAGE_PLATE_LOCAL_Z  # embed into plate center
    _post_top_z = 0.0  # reaches axle centerline
    _post_len = _post_top_z - _post_bot_z
    _post_center_z = (_post_top_z + _post_bot_z) / 2.0
    if _post_len > 0.005:
        carriage.visual(
            Cylinder(radius=0.022, length=_post_len),
            origin=Origin(xyz=(0.0, 0.0, _post_center_z)),
            material=dark_steel,
            name="carriage_post",
        )

    # Pivot axle bolt on top of carriage (at part frame origin = PIVOT_Z world)
    carriage.visual(
        Cylinder(radius=AXLE_R, length=AXLE_LEN),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rust,
        name="pivot_axle",
    )
    for i, side in enumerate((1.0, -1.0)):
        carriage.visual(
            Cylinder(radius=0.026, length=0.014),
            origin=Origin(
                xyz=(0.0, side * (AXLE_LEN / 2.0 - 0.006), 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=rust,
            name=f"axle_nut_{i}",
        )

    # --------------------------------------------------------------- beam ---
    beam = model.part("beam")

    # Pivot sleeve (bushing around axle)
    beam.visual(
        Cylinder(radius=0.028, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rust,
        name="pivot_sleeve",
    )
    beam.visual(_gusset_geometry(), material=steel_grey, name="gusset_plate")

    # Heavy commercial steel beam bar
    beam.visual(
        Box((2.0 * BEAM_HALF, BEAM_W, BEAM_T)),
        origin=Origin(xyz=(0.0, 0.0, BAR_CTR)),
        material=steel_grey,
        name="beam_bar",
    )

    # Rust streak patches
    for i, px in enumerate((-0.85, -0.30, 0.45, 0.95)):
        beam.visual(
            Box((0.16, BEAM_W + 0.004, 0.012)),
            origin=Origin(xyz=(px, 0.0, BAR_TOP - 0.004)),
            material=rust,
            name=f"rust_patch_{i}",
        )

    # End fittings: molded seats with raised lips, handles with grips, bumpers
    seat_mesh = mesh_from_cadquery(_molded_seat_shape(), "molded_seat")

    for i, side in enumerate((1.0, -1.0)):
        # Molded seat with raised lip, embedded slightly into bar top
        beam.visual(
            seat_mesh,
            origin=Origin(xyz=(side * SEAT_X, 0.0, BAR_TOP + 0.005)),
            material=seat_green,
            name=f"seat_{i}",
        )

        # Handle (inverted-U rod)
        beam.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _handle_points(side * HANDLE_X),
                    radius=0.010,
                    samples_per_segment=8,
                    radial_segments=16,
                    cap_ends=True,
                ),
                f"seesaw_handle_{i}",
            ),
            material=pale_steel,
            name=f"handle_{i}",
        )

        # Rounded grip ball at handle top
        handle_arc_z = 0.285 + 0.040  # top of the handle arc
        beam.visual(
            Sphere(radius=0.022),
            origin=Origin(xyz=(side * HANDLE_X, 0.0, handle_arc_z)),
            material=grip_red,
            name=f"grip_{i}",
        )

        # Rubber bumper
        beam.visual(
            _bumper_geometry(side * BUMPER_X, i),
            material=rubber,
            name=f"bumper_{i}",
        )

    # -------------------------------------------------------- joints ---
    # Prismatic: spring compression (base -> carriage)
    # axis (0, 0, -1): positive q moves carriage DOWN (spring compresses)
    model.articulation(
        "spring_compress",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=2000.0, velocity=0.15,
            lower=0.0, upper=SPRING_COMPRESS_MAX,
        ),
    )

    # Revolute: beam pivot (carriage -> beam)
    model.articulation(
        "beam_pivot",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=200.0, velocity=2.5,
            lower=-TILT, upper=TILT,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("arched_base")
    carriage = object_model.get_part("spring_carriage")
    beam = object_model.get_part("beam")
    spring_joint = object_model.get_articulation("spring_compress")
    pivot = object_model.get_articulation("beam_pivot")

    # --- Intentional overlap allowances ---

    # Pivot sleeve captures the axle bolt
    ctx.allow_overlap(
        beam,
        carriage,
        elem_a="pivot_sleeve",
        elem_b="pivot_axle",
        reason="Pivot sleeve is a bushing intentionally nested around the axle bolt.",
    )
    ctx.expect_contact(
        beam,
        carriage,
        elem_a="pivot_sleeve",
        elem_b="pivot_axle",
        name="pivot sleeve is seated on the axle bolt",
    )
    ctx.expect_within(
        beam,
        carriage,
        axes="y",
        inner_elem="pivot_sleeve",
        outer_elem="pivot_axle",
        margin=0.001,
        name="pivot sleeve stays inside the axle span",
    )

    # Pivot sleeve wraps around the carriage post at the pivot region
    ctx.allow_overlap(
        beam,
        carriage,
        elem_a="pivot_sleeve",
        elem_b="carriage_post",
        reason="Sleeve wraps around the carriage post at the pivot; the post supports the axle and the sleeve rotates around it.",
    )

    # Carriage plate seats onto the top of the spring coil
    ctx.allow_overlap(
        carriage,
        base,
        elem_a="carriage_plate",
        elem_b="spring_coil",
        reason="Carriage plate seats onto the top of the spring coil; small local overlap at rest.",
    )
    ctx.expect_contact(
        carriage,
        base,
        elem_a="carriage_plate",
        elem_b="spring_coil",
        name="carriage plate contacts the spring coil top",
    )

    # Axle bolt passes through the arch apex region (both arches)
    for i in range(2):
        ctx.allow_overlap(
            base,
            carriage,
            elem_a=f"arch_{i}",
            elem_b="pivot_axle",
            reason=f"Axle bolt passes through arch_{i} apex; the arch tube and axle share the pivot region.",
        )

    # --- Prismatic spring joint checks ---
    ctx.check(
        "spring joint is prismatic",
        spring_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={spring_joint.articulation_type}",
    )
    s_axis = spring_joint.axis
    ctx.check(
        "spring axis is vertical (compression direction)",
        abs(s_axis[0]) < 1e-9 and abs(s_axis[1]) < 1e-9 and abs(abs(s_axis[2]) - 1.0) < 1e-9,
        details=f"axis={s_axis}",
    )
    s_lim = spring_joint.motion_limits
    ctx.check(
        "spring has compression travel limits",
        s_lim is not None
        and s_lim.lower is not None
        and s_lim.upper is not None
        and s_lim.lower >= 0.0
        and s_lim.upper > 0.0,
        details=f"limits=({s_lim.lower}, {s_lim.upper})",
    )

    # Spring compression actually moves the carriage downward
    rest_carriage = ctx.part_world_position(carriage)
    with ctx.pose({spring_joint: SPRING_COMPRESS_MAX}):
        compressed_carriage = ctx.part_world_position(carriage)
    ctx.check(
        "spring compression moves carriage downward",
        rest_carriage is not None
        and compressed_carriage is not None
        and compressed_carriage[2] < rest_carriage[2] - 0.02,
        details=f"rest={rest_carriage}, compressed={compressed_carriage}",
    )

    # --- Revolute pivot checks ---
    ax = pivot.axis
    ctx.check(
        "pivot axis is horizontal and perpendicular to the beam",
        abs(ax[0]) < 1e-9 and abs(ax[1] - 1.0) < 1e-9 and abs(ax[2]) < 1e-9,
        details=f"axis={ax}",
    )
    lim = pivot.motion_limits
    ctx.check(
        "rocking limits are about +/- 20 degrees",
        lim is not None
        and lim.lower is not None
        and lim.upper is not None
        and abs(lim.lower + TILT) < 1e-6
        and abs(lim.upper - TILT) < 1e-6,
        details=f"limits=({lim.lower}, {lim.upper})",
    )

    # --- Hero geometry checks ---
    bar_box = ctx.part_element_world_aabb(beam, elem="beam_bar")
    base_box = ctx.part_world_aabb(base)
    axle_box = ctx.part_element_world_aabb(carriage, elem="pivot_axle")

    ctx.check(
        "beam is about 3.0 m long",
        bar_box is not None and abs((bar_box[1][0] - bar_box[0][0]) - 3.0) < 0.02,
        details=f"bar aabb={bar_box}",
    )
    ctx.check(
        "beam bar is heavy commercial (>= 0.09 m wide)",
        bar_box is not None and (bar_box[1][1] - bar_box[0][1]) >= 0.09,
        details=f"bar aabb={bar_box}",
    )
    ctx.check(
        "arched base feet rest on the ground",
        base_box is not None and -0.01 <= base_box[0][2] <= 0.02,
        details=f"base aabb={base_box}",
    )
    ctx.check(
        "pivot axle sits about 0.8 m high",
        axle_box is not None and 0.70 <= (axle_box[0][2] + axle_box[1][2]) / 2.0 <= 0.82,
        details=f"axle aabb={axle_box}",
    )

    # Spring coil exists and is visible between base and carriage
    spring_box = ctx.part_element_world_aabb(base, elem="spring_coil")
    ctx.check(
        "spring coil exists between base and pivot",
        spring_box is not None
        and spring_box[0][2] < PIVOT_Z
        and spring_box[1][2] > SPRING_BASE_Z - 0.01,
        details=f"spring aabb={spring_box}",
    )

    # Carriage plate is below the beam bar (spring gap visible)
    plate_box = ctx.part_element_world_aabb(carriage, elem="carriage_plate")
    ctx.check(
        "carriage plate sits below the beam bar",
        plate_box is not None
        and bar_box is not None
        and plate_box[1][2] < bar_box[0][2],
        details=f"plate top={plate_box[1][2] if plate_box else None}, bar bottom={bar_box[0][2] if bar_box else None}",
    )

    # --- Molded seats with raised lips ---
    for i in range(2):
        seat = ctx.part_element_world_aabb(beam, elem=f"seat_{i}")
        ctx.check(
            f"seat_{i} is mounted on the beam bar",
            seat is not None
            and bar_box is not None
            and seat[0][2] >= bar_box[0][2]
            and seat[1][2] > bar_box[1][2],
            details=f"seat aabb={seat}",
        )
        # Raised lip: seat top should be noticeably above bar top
        ctx.check(
            f"seat_{i} has raised lip above the beam bar top",
            seat is not None
            and bar_box is not None
            and seat[1][2] > bar_box[1][2] + 0.010,
            details=f"seat top={seat[1][2] if seat else None}, bar top={bar_box[1][2] if bar_box else None}",
        )

    # --- Rounded handle grips ---
    for i in range(2):
        handle = ctx.part_element_world_aabb(beam, elem=f"handle_{i}")
        grip = ctx.part_element_world_aabb(beam, elem=f"grip_{i}")
        ctx.check(
            f"handle_{i} stands above the beam",
            handle is not None
            and bar_box is not None
            and handle[1][2] > bar_box[1][2] + 0.18
            and handle[0][2] < bar_box[1][2],
            details=f"handle aabb={handle}",
        )
        ctx.check(
            f"grip_{i} exists at handle top",
            grip is not None
            and handle is not None
            and bar_box is not None
            and grip[1][2] >= handle[1][2] - 0.01
            and grip[0][2] > bar_box[1][2] + 0.18,
            details=f"grip aabb={grip}, handle top={handle[1][2] if handle else None}",
        )

    # --- Bumpers ---
    for i in range(2):
        bumper = ctx.part_element_world_aabb(beam, elem=f"bumper_{i}")
        ctx.check(
            f"bumper_{i} hangs below the beam tip",
            bumper is not None
            and bar_box is not None
            and bumper[0][2] < bar_box[0][2]
            and min(abs(bumper[0][0]), abs(bumper[1][0])) > 1.3,
            details=f"bumper aabb={bumper}",
        )

    # --- Decisive rocking pose checks ---
    rest_b0 = ctx.part_element_world_aabb(beam, elem="bumper_0")
    with ctx.pose({pivot: TILT}):
        down_b0 = ctx.part_element_world_aabb(beam, elem="bumper_0")
        up_b1 = ctx.part_element_world_aabb(beam, elem="bumper_1")
        ctx.check(
            "positive rock lowers the +X end near the ground",
            rest_b0 is not None
            and down_b0 is not None
            and down_b0[0][2] < rest_b0[0][2] - 0.40
            and down_b0[0][2] > 0.0,
            details=f"rest={rest_b0}, tilted={down_b0}",
        )
        ctx.check(
            "positive rock raises the -X end",
            up_b1 is not None and up_b1[0][2] > 1.0,
            details=f"raised bumper aabb={up_b1}",
        )
    with ctx.pose({pivot: -TILT}):
        down_b1 = ctx.part_element_world_aabb(beam, elem="bumper_1")
        ctx.check(
            "negative rock lowers the -X end near the ground",
            down_b1 is not None and 0.0 < down_b1[0][2] < 0.32,
            details=f"tilted bumper aabb={down_b1}",
        )

    # Beam bar clears the arch saddle at rest
    ctx.expect_gap(
        beam,
        base,
        axis="z",
        positive_elem="beam_bar",
        negative_elem="arch_0",
        min_gap=0.005,
        max_gap=0.12,
        name="beam bar clears the arch saddle",
    )

    return ctx.report()


object_model = build_object_model()
