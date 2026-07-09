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
)

# ----------------------------------------------------------------------------
# Spring-assisted modern playground seesaw.
#
# Layout (world frame, Z up, base centered on the origin):
# - Central pedestal base: ground plate, cylindrical column, visible coil
#   spring at the top, and a forked axle bracket with visible axle caps.
# - A single beam (~2.2 m) rocks on the axle bracket via a revolute pivot.
# - Each beam end carries a molded dish seat (with raised lip) and a pivoting
#   T-handlebar just inboard of the seat.
# - Articulations:
#   1. beam_pivot (REVOLUTE): beam rocks +/- 15 deg on the axle bracket.
#   2. handlebar_left_pivot (REVOLUTE): left handlebar tilts +/- 8 deg.
#   3. handlebar_right_pivot (REVOLUTE): right handlebar tilts +/- 8 deg.
# ----------------------------------------------------------------------------

BEAM_LEN = 2.20
BEAM_WIDTH = 0.06
BEAM_HEIGHT = 0.05

COLUMN_RADIUS = 0.045
COLUMN_HEIGHT = 0.42
GROUND_PLATE_SIZE = 0.50
GROUND_PLATE_THICK = 0.018

SPRING_COIL_RADIUS = 0.06
SPRING_TUBE_RADIUS = 0.008
SPRING_COILS = 5
SPRING_BASE_Z = GROUND_PLATE_THICK / 2.0 + COLUMN_HEIGHT  # column top position

BRACKET_PLATE_THICK = 0.012
BRACKET_HEIGHT = 0.10
BRACKET_GAP = 0.08  # inner gap between the two bracket plates
AXLE_RADIUS = 0.018
AXLE_CAP_RADIUS = 0.030
AXLE_CAP_THICK = 0.008

SEAT_OUTER_RADIUS = 0.15
SEAT_INNER_DEPTH = 0.035
SEAT_LIP_HEIGHT = 0.012
SEAT_X = 0.92  # seat center from beam midpoint along X
SEAT_MOUNT_Z = 0.04  # seat bottom above beam center

HANDLE_POST_X = 0.62  # handlebar post position from beam midpoint
HANDLE_POST_HEIGHT = 0.30
HANDLE_BAR_LENGTH = 0.28
HANDLE_RADIUS = 0.012
HANDLE_MOUNT_Z = 0.046  # exactly at boss top so collar contacts boss

TILT = math.radians(15.0)
HANDLEBAR_TILT = math.radians(8.0)

# Materials
DARK_GREEN = Material("dark_green_paint", rgba=(0.18, 0.42, 0.24, 1.0))
BRIGHT_ORANGE = Material("bright_orange_paint", rgba=(0.92, 0.44, 0.10, 1.0))
SEAT_RED = Material("molded_red_seat", rgba=(0.78, 0.15, 0.12, 1.0))
STEEL_GREY = Material("steel_grey", rgba=(0.55, 0.56, 0.58, 1.0))
RUBBER_BLACK = Material("rubber_black", rgba=(0.12, 0.12, 0.13, 1.0))
SPRING_STEEL = Material("spring_steel", rgba=(0.62, 0.63, 0.60, 1.0))
AXLE_CAP_MAT = Material("zinc_cap", rgba=(0.68, 0.69, 0.70, 1.0))


def _build_molded_seat_mesh() -> MeshGeometry:
    """Build a molded dish seat with a raised outer lip using LatheGeometry.

    Profile goes from center bottom outward and upward, then has a raised lip.
    """
    profile = [
        (0.000, 0.000),       # center bottom
        (0.030, 0.001),       # slight dish
        (0.060, 0.004),       # curving up
        (0.090, 0.010),       # rising
        (0.110, 0.020),       # approaching rim
        (0.130, 0.032),       # rim base
        (0.140, SEAT_INNER_DEPTH),  # rim top
        (0.148, SEAT_INNER_DEPTH + 0.003),  # lip start
        (0.150, SEAT_INNER_DEPTH + SEAT_LIP_HEIGHT),  # lip peak (outer)
        (0.142, SEAT_INNER_DEPTH + SEAT_LIP_HEIGHT - 0.002),  # lip inner drop
        (0.130, SEAT_INNER_DEPTH - 0.005),  # inside lip
    ]
    return LatheGeometry(profile, segments=32, closed=True)


def _build_spring_mesh() -> MeshGeometry:
    """Build a visible coil spring from stacked torus rings that contact the column and each other."""
    geom = MeshGeometry()
    # Tube radius large enough that consecutive coils touch (pitch <= 2*tube)
    tube_r = 0.010
    coil_pitch = 2.0 * tube_r  # coils touch when pitch = 2*tube_radius
    # Spring center radius so inner surface overlaps with column for connectivity
    spring_r = COLUMN_RADIUS + tube_r * 0.5
    for i in range(SPRING_COILS):
        z = SPRING_BASE_Z + i * coil_pitch + tube_r
        ring = TorusGeometry(
            radius=spring_r,
            tube=tube_r,
            radial_segments=12,
            tubular_segments=24,
        )
        ring.translate(0.0, 0.0, z)
        geom.merge(ring)
    return geom


def _build_beam_mesh() -> MeshGeometry:
    """Build the main beam tube in local frame (X along beam, origin at midpoint)."""
    geom = CylinderGeometry(0.030, BEAM_LEN, radial_segments=18)
    geom.rotate_y(math.pi / 2.0)
    return geom


def _build_handlebar_mesh() -> MeshGeometry:
    """Build a T-handlebar in local frame (Z up post, Y crossbar)."""
    post = CylinderGeometry(HANDLE_RADIUS, HANDLE_POST_HEIGHT, radial_segments=14)
    post.translate(0.0, 0.0, HANDLE_POST_HEIGHT / 2.0)
    bar = CylinderGeometry(HANDLE_RADIUS, HANDLE_BAR_LENGTH, radial_segments=14)
    bar.rotate_x(math.pi / 2.0)
    bar.translate(0.0, 0.0, HANDLE_POST_HEIGHT)
    # Add rubber grip sleeves at each end of the crossbar
    grip_l = CylinderGeometry(HANDLE_RADIUS * 1.4, 0.08, radial_segments=12)
    grip_l.rotate_x(math.pi / 2.0)
    grip_l.translate(0.0, -HANDLE_BAR_LENGTH / 2.0 + 0.04, HANDLE_POST_HEIGHT)
    grip_r = CylinderGeometry(HANDLE_RADIUS * 1.4, 0.08, radial_segments=12)
    grip_r.rotate_x(math.pi / 2.0)
    grip_r.translate(0.0, HANDLE_BAR_LENGTH / 2.0 - 0.04, HANDLE_POST_HEIGHT)
    post.merge(bar)
    post.merge(grip_l)
    post.merge(grip_r)
    return post


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="spring_assisted_seesaw")

    # --- Base / pedestal -------------------------------------------------------
    base = model.part("base")

    # Ground plate (mesh cylinder for connectivity with column)
    ground_mesh = CylinderGeometry(GROUND_PLATE_SIZE / 2.0, GROUND_PLATE_THICK, radial_segments=24)
    base.visual(
        mesh_from_geometry(ground_mesh, "ground_plate"),
        origin=Origin(xyz=(0.0, 0.0, GROUND_PLATE_THICK / 2.0)),
        material=DARK_GREEN,
        name="ground_plate",
    )

    # Central column (slightly overlaps ground plate for connectivity)
    column_mesh = CylinderGeometry(COLUMN_RADIUS, COLUMN_HEIGHT, radial_segments=20)
    base.visual(
        mesh_from_geometry(column_mesh, "central_column"),
        origin=Origin(xyz=(0.0, 0.0, GROUND_PLATE_THICK / 2.0 + COLUMN_HEIGHT / 2.0)),
        material=DARK_GREEN,
        name="column",
    )

    # Column base flange (overlaps both ground plate and column)
    flange_mesh = CylinderGeometry(COLUMN_RADIUS + 0.02, 0.020, radial_segments=20)
    base.visual(
        mesh_from_geometry(flange_mesh, "column_flange"),
        origin=Origin(xyz=(0.0, 0.0, GROUND_PLATE_THICK)),
        material=STEEL_GREY,
        name="column_flange",
    )

    # Spring coils (visible around the column top)
    spring_mesh = _build_spring_mesh()
    base.visual(
        mesh_from_geometry(spring_mesh, "coil_spring"),
        material=SPRING_STEEL,
        name="coil_spring",
    )

    # Spring top cap / pivot platform
    spring_tube_r = 0.010
    spring_coil_pitch = 2.0 * spring_tube_r
    spring_top_z = SPRING_BASE_Z + SPRING_COILS * spring_coil_pitch
    platform_mesh = CylinderGeometry(0.065, 0.016, radial_segments=20)
    base.visual(
        mesh_from_geometry(platform_mesh, "spring_platform"),
        origin=Origin(xyz=(0.0, 0.0, spring_top_z + 0.008)),
        material=DARK_GREEN,
        name="spring_platform",
    )

    # Axle bracket: two upright plates forming a fork
    bracket_base_z = spring_top_z + 0.016
    for side in (-1.0, 1.0):
        plate_y = side * (BRACKET_GAP / 2.0 + BRACKET_PLATE_THICK / 2.0)
        plate_mesh = Box((0.06, BRACKET_PLATE_THICK, BRACKET_HEIGHT))
        base.visual(
            plate_mesh,
            origin=Origin(xyz=(0.0, plate_y, bracket_base_z + BRACKET_HEIGHT / 2.0)),
            material=DARK_GREEN,
            name=f"bracket_plate_{int((side + 1) / 2)}",
        )

    # Axle pin spanning the bracket gap
    axle_len = BRACKET_GAP + 2 * BRACKET_PLATE_THICK + 0.002
    axle_mesh = CylinderGeometry(AXLE_RADIUS, axle_len, radial_segments=16)
    axle_mesh.rotate_x(math.pi / 2.0)
    axle_z = bracket_base_z + BRACKET_HEIGHT * 0.65
    base.visual(
        mesh_from_geometry(axle_mesh, "axle_pin"),
        origin=Origin(xyz=(0.0, 0.0, axle_z)),
        material=STEEL_GREY,
        name="axle_pin",
    )

    # Visible axle caps (discs on outside of bracket plates)
    for side in (-1.0, 1.0):
        cap_y = side * (BRACKET_GAP / 2.0 + BRACKET_PLATE_THICK + AXLE_CAP_THICK / 2.0 + 0.001)
        cap_mesh = CylinderGeometry(AXLE_CAP_RADIUS, AXLE_CAP_THICK, radial_segments=20)
        cap_mesh.rotate_x(math.pi / 2.0)
        base.visual(
            mesh_from_geometry(cap_mesh, f"axle_cap_{int((side + 1) / 2)}"),
            origin=Origin(xyz=(0.0, cap_y, axle_z)),
            material=AXLE_CAP_MAT,
            name=f"axle_cap_{int((side + 1) / 2)}",
        )

    # --- Beam ------------------------------------------------------------------
    beam = model.part("beam")
    beam_mesh = _build_beam_mesh()
    beam.visual(
        mesh_from_geometry(beam_mesh, "main_beam"),
        material=BRIGHT_ORANGE,
        name="main_beam",
    )

    # Axle sleeve/hub at beam center (wraps around the axle)
    hub_len = BRACKET_GAP - 0.004
    hub_mesh = CylinderGeometry(AXLE_RADIUS + 0.010, hub_len, radial_segments=18)
    hub_mesh.rotate_x(math.pi / 2.0)
    beam.visual(
        mesh_from_geometry(hub_mesh, "beam_hub"),
        material=BRIGHT_ORANGE,
        name="beam_hub",
    )

    # Seat brackets connect directly to the beam tube (no separate pads needed)

    # Handlebar mount bosses (small cylinders on top of beam, inboard of seats)
    for sx in (-1.0, 1.0):
        boss = CylinderGeometry(0.018, 0.016, radial_segments=14)
        beam.visual(
            mesh_from_geometry(boss, f"handlebar_boss_{int((sx + 1) / 2)}"),
            origin=Origin(xyz=(sx * HANDLE_POST_X, 0.0, 0.030 + 0.008)),
            material=BRIGHT_ORANGE,
            name=f"handlebar_boss_{int((sx + 1) / 2)}",
        )

    # --- Molded seats (separate parts for clarity) ----------------------------
    seat_mesh = _build_molded_seat_mesh()

    seat_left = model.part("seat_left")
    seat_left.visual(
        mesh_from_geometry(seat_mesh.copy(), "seat_dish_left"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=SEAT_RED,
        name="seat_dish",
    )
    # Seat mounting bracket (connects seat to beam)
    seat_left.visual(
        Box((0.10, 0.10, SEAT_MOUNT_Z)),
        origin=Origin(xyz=(0.0, 0.0, -SEAT_MOUNT_Z / 2.0)),
        material=STEEL_GREY,
        name="seat_bracket",
    )

    seat_right = model.part("seat_right")
    seat_right.visual(
        mesh_from_geometry(seat_mesh.copy(), "seat_dish_right"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=SEAT_RED,
        name="seat_dish",
    )
    seat_right.visual(
        Box((0.10, 0.10, SEAT_MOUNT_Z)),
        origin=Origin(xyz=(0.0, 0.0, -SEAT_MOUNT_Z / 2.0)),
        material=STEEL_GREY,
        name="seat_bracket",
    )

    # --- Pivoting handlebars --------------------------------------------------
    handlebar_mesh = _build_handlebar_mesh()

    handlebar_left = model.part("handlebar_left")
    handlebar_left.visual(
        mesh_from_geometry(handlebar_mesh.copy(), "handlebar_left_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=STEEL_GREY,
        name="handlebar",
    )
    # Pivot collar at base of handlebar post
    collar_l = CylinderGeometry(0.016, 0.020, radial_segments=14)
    handlebar_left.visual(
        mesh_from_geometry(collar_l, "handlebar_left_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=RUBBER_BLACK,
        name="pivot_collar",
    )

    handlebar_right = model.part("handlebar_right")
    handlebar_right.visual(
        mesh_from_geometry(handlebar_mesh.copy(), "handlebar_right_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=STEEL_GREY,
        name="handlebar",
    )
    collar_r = CylinderGeometry(0.016, 0.020, radial_segments=14)
    handlebar_right.visual(
        mesh_from_geometry(collar_r, "handlebar_right_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=RUBBER_BLACK,
        name="pivot_collar",
    )


    # --- Articulations --------------------------------------------------------

    # Beam pivot: revolute on the axle, axis along Y (perpendicular to beam X)
    pivot_z = axle_z
    model.articulation(
        "beam_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=200.0, velocity=2.0, lower=-TILT, upper=TILT),
    )

    # Seats are fixed to beam ends
    model.articulation(
        "beam_to_seat_left",
        ArticulationType.FIXED,
        parent=beam,
        child=seat_left,
        origin=Origin(xyz=(-SEAT_X, 0.0, SEAT_MOUNT_Z)),
    )
    model.articulation(
        "beam_to_seat_right",
        ArticulationType.FIXED,
        parent=beam,
        child=seat_right,
        origin=Origin(xyz=(SEAT_X, 0.0, SEAT_MOUNT_Z)),
    )

    # Handlebar pivots: revolute, axis along beam X (so handlebar tilts side to side)
    model.articulation(
        "beam_to_handlebar_left",
        ArticulationType.REVOLUTE,
        parent=beam,
        child=handlebar_left,
        origin=Origin(xyz=(-HANDLE_POST_X, 0.0, HANDLE_MOUNT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=3.0, lower=-HANDLEBAR_TILT, upper=HANDLEBAR_TILT
        ),
    )
    model.articulation(
        "beam_to_handlebar_right",
        ArticulationType.REVOLUTE,
        parent=beam,
        child=handlebar_right,
        origin=Origin(xyz=(HANDLE_POST_X, 0.0, HANDLE_MOUNT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=3.0, lower=-HANDLEBAR_TILT, upper=HANDLEBAR_TILT
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    beam = object_model.get_part("beam")
    seat_left = object_model.get_part("seat_left")
    seat_right = object_model.get_part("seat_right")
    handlebar_left = object_model.get_part("handlebar_left")
    handlebar_right = object_model.get_part("handlebar_right")
    beam_pivot = object_model.get_articulation("beam_pivot")
    hb_left_pivot = object_model.get_articulation("beam_to_handlebar_left")
    hb_right_pivot = object_model.get_articulation("beam_to_handlebar_right")

    # --- Intentional overlap allowances ---

    # Beam hub wraps the axle pin as the physical pivot bearing
    ctx.allow_overlap(
        beam,
        base,
        elem_a="beam_hub",
        elem_b="axle_pin",
        reason="Beam hub wraps the axle pin as the physical pivot bearing.",
    )
    # Main beam tube also passes through the axle area (hub surrounds it)
    ctx.allow_overlap(
        beam,
        base,
        elem_a="main_beam",
        elem_b="axle_pin",
        reason="Main beam tube passes through the axle area where the hub wraps the pin.",
    )
    # Handlebar pivot collars sit on beam mount bosses (seated bearing fit)
    ctx.allow_overlap(
        beam,
        handlebar_left,
        elem_a="handlebar_boss_0",
        elem_b="pivot_collar",
        reason="Left handlebar pivot collar seats on the beam mount boss.",
    )
    ctx.allow_overlap(
        beam,
        handlebar_right,
        elem_a="handlebar_boss_1",
        elem_b="pivot_collar",
        reason="Right handlebar pivot collar seats on the beam mount boss.",
    )
    # Seat brackets clamp onto the main beam tube
    ctx.allow_overlap(
        beam,
        seat_left,
        elem_a="main_beam",
        elem_b="seat_bracket",
        reason="Left seat bracket clamps onto the main beam tube.",
    )
    ctx.allow_overlap(
        beam,
        seat_right,
        elem_a="main_beam",
        elem_b="seat_bracket",
        reason="Right seat bracket clamps onto the main beam tube.",
    )

    # Proof checks for pivot and mount contacts
    ctx.expect_contact(
        beam,
        base,
        elem_a="beam_hub",
        elem_b="axle_pin",
        name="beam hub rides on the axle pin",
    )
    ctx.expect_contact(
        beam,
        handlebar_left,
        elem_a="handlebar_boss_0",
        elem_b="pivot_collar",
        name="left handlebar collar contacts beam boss",
    )
    ctx.expect_contact(
        beam,
        handlebar_right,
        elem_a="handlebar_boss_1",
        elem_b="pivot_collar",
        name="right handlebar collar contacts beam boss",
    )

    # --- Beam pivot range ---
    lim = beam_pivot.motion_limits
    ctx.check(
        "beam pivot rocks +/- 15 degrees",
        lim is not None
        and abs(lim.lower + TILT) < 1e-6
        and abs(lim.upper - TILT) < 1e-6,
        details=f"limits=({lim.lower if lim else None}, {lim.upper if lim else None})",
    )

    # --- Handlebar pivots ---
    for pivot, name in (
        (hb_left_pivot, "left"),
        (hb_right_pivot, "right"),
    ):
        plim = pivot.motion_limits
        ctx.check(
            f"handlebar {name} pivots +/- 8 degrees",
            plim is not None
            and abs(plim.lower + HANDLEBAR_TILT) < 1e-6
            and abs(plim.upper - HANDLEBAR_TILT) < 1e-6,
            details=f"limits=({plim.lower if plim else None}, {plim.upper if plim else None})",
        )

    # --- Visible axle caps ---
    cap0 = ctx.part_element_world_aabb(base, elem="axle_cap_0")
    cap1 = ctx.part_element_world_aabb(base, elem="axle_cap_1")
    ctx.check(
        "visible axle caps on both sides of the bracket",
        cap0 is not None and cap1 is not None,
        details=f"cap0={cap0}, cap1={cap1}",
    )

    # --- Molded seats with raised lips ---
    dish_l = ctx.part_element_world_aabb(seat_left, elem="seat_dish")
    dish_r = ctx.part_element_world_aabb(seat_right, elem="seat_dish")
    ctx.check(
        "molded dish seats exist on both beam ends",
        dish_l is not None and dish_r is not None,
        details=f"dish_l={dish_l}, dish_r={dish_r}",
    )
    if dish_l is not None:
        ctx.check(
            "left seat has raised lip (vertical extent)",
            (dish_l[1][2] - dish_l[0][2]) > 0.030,
            details=f"dish height = {dish_l[1][2] - dish_l[0][2]:.4f}",
        )
    if dish_r is not None:
        ctx.check(
            "right seat has raised lip (vertical extent)",
            (dish_r[1][2] - dish_r[0][2]) > 0.030,
            details=f"dish height = {dish_r[1][2] - dish_r[0][2]:.4f}",
        )

    # --- Spring is visible ---
    spring_aabb = ctx.part_element_world_aabb(base, elem="coil_spring")
    ctx.check(
        "visible coil spring on the base",
        spring_aabb is not None and (spring_aabb[1][2] - spring_aabb[0][2]) > 0.08,
        details=f"spring aabb={spring_aabb}",
    )

    # --- Handlebars exist and stand above seats ---
    hb_l = ctx.part_element_world_aabb(handlebar_left, elem="handlebar")
    hb_r = ctx.part_element_world_aabb(handlebar_right, elem="handlebar")
    ctx.check(
        "handlebars exist at both beam ends",
        hb_l is not None and hb_r is not None,
        details=f"hb_l={hb_l}, hb_r={hb_r}",
    )
    if hb_l is not None and dish_l is not None:
        ctx.check(
            "left handlebar stands taller than seat",
            hb_l[1][2] > dish_l[1][2] + 0.15,
            details=f"handlebar top={hb_l[1][2]:.3f}, seat top={dish_l[1][2]:.3f}",
        )
    if hb_r is not None and dish_r is not None:
        ctx.check(
            "right handlebar stands taller than seat",
            hb_r[1][2] > dish_r[1][2] + 0.15,
            details=f"handlebar top={hb_r[1][2]:.3f}, seat top={dish_r[1][2]:.3f}",
        )

    # --- Seats at opposite beam ends ---
    if dish_l is not None and dish_r is not None:
        lx = (dish_l[0][0] + dish_l[1][0]) / 2.0
        rx = (dish_r[0][0] + dish_r[1][0]) / 2.0
        ctx.check(
            "seats are at opposite beam ends",
            abs(rx - lx) > 1.5,
            details=f"left_x={lx:.3f}, right_x={rx:.3f}",
        )

    # --- Handlebar pivot proof: check min Y shift (tilt causes asymmetric Y extent) ---
    hb_l_rest = ctx.part_element_world_aabb(handlebar_left, elem="handlebar")
    with ctx.pose({hb_left_pivot: HANDLEBAR_TILT}):
        hb_l_tilted = ctx.part_element_world_aabb(handlebar_left, elem="handlebar")
        ctx.check(
            "left handlebar tilts on its pivot joint",
            hb_l_rest is not None
            and hb_l_tilted is not None
            and abs(hb_l_tilted[0][1] - hb_l_rest[0][1]) > 0.02,
            details=f"rest_min_y={hb_l_rest[0][1] if hb_l_rest else None}, tilted_min_y={hb_l_tilted[0][1] if hb_l_tilted else None}",
        )

    # --- Beam seesaw proof: with axis=(0,1,0) and +TILT, left(-X) goes up, right(+X) goes down ---
    rest_l = ctx.part_element_world_aabb(seat_left, elem="seat_dish")
    rest_r = ctx.part_element_world_aabb(seat_right, elem="seat_dish")
    with ctx.pose({beam_pivot: TILT}):
        tilt_l = ctx.part_element_world_aabb(seat_left, elem="seat_dish")
        tilt_r = ctx.part_element_world_aabb(seat_right, elem="seat_dish")
        ctx.check(
            "beam seesaws: left seat rises, right seat drops with positive tilt",
            rest_l is not None
            and tilt_l is not None
            and rest_r is not None
            and tilt_r is not None
            and tilt_l[0][2] > rest_l[0][2] + 0.15
            and tilt_r[0][2] < rest_r[0][2] - 0.15,
            details=f"left {rest_l} -> {tilt_l}, right {rest_r} -> {tilt_r}",
        )

    # --- Base sits on ground ---
    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "base rests on ground plane",
        base_aabb is not None and -0.01 <= base_aabb[0][2] <= 0.02,
        details=f"base aabb={base_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
