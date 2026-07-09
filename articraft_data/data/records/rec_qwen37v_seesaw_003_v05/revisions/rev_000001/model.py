from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CapsuleGeometry,
    ConeGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    sample_catmull_rom_spline_2d,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Shared dimensions (meters). World: X along the seesaw length, Z up.
# ---------------------------------------------------------------------------
PIVOT_Z = 0.42          # world height of the rocking axis (top of spring bracket)
BEAM_R = 0.065          # main tube radius (~130 mm diameter - heavy steel)
BEAM_HALF = 1.15        # half-length of the curved main tube
CURVE_C = 0.10          # parabolic curvature of the banana beam
BEAM_CENTER_Z = 0.18    # beam centerline height at x=0, relative to the pivot

COLLAR_X = 0.97         # clamp collar position along the beam
SEAT_CENTER_X = 1.14
SEAT_Z = 0.05           # seat mid-plane, relative to the pivot
PLATE_T = 0.025         # thicker molded seat
HANDLE_X = 1.03
HANDLE_Z = 0.58         # handle grip center, relative to the pivot

ROCK_LIMIT = 0.262      # ~15 degrees each way

# Spring / pedestal dimensions
PEDESTAL_R = 0.12
PEDESTAL_H = 0.18
SPRING_COIL_R = 0.055   # spring coil centerline radius
SPRING_WIRE_R = 0.012   # spring wire radius
SPRING_TURNS = 4
SPRING_HEIGHT = 0.14    # uncompressed spring height
SPRING_COMPRESS = 0.04  # max spring compression (prismatic travel)

BRACKET_W = 0.18
BRACKET_D = 0.14
BRACKET_H = 0.06
BRACKET_CZ = SPRING_HEIGHT + BRACKET_H / 2.0  # bracket sits atop spring


def _beam_z(x: float) -> float:
    """Beam centerline height (relative to the pivot frame) at station x."""
    return BEAM_CENTER_Z + CURVE_C * x * x


def _helix_points(radius: float, height: float, turns: int, samples_per_turn: int = 24):
    """Generate helical spring centerline points."""
    pts = []
    total = turns * samples_per_turn
    for i in range(total + 1):
        t = i / total
        angle = turns * 2.0 * math.pi * t
        pts.append((radius * math.cos(angle), radius * math.sin(angle), height * t))
    return pts


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="playground_seesaw_spring")

    # Materials
    model.material("steel_beam", rgba=(0.42, 0.44, 0.48, 1.0))
    model.material("matte_black", rgba=(0.09, 0.09, 0.10, 1.0))
    model.material("rubber_black", rgba=(0.05, 0.05, 0.06, 1.0))
    model.material("light_gray", rgba=(0.78, 0.78, 0.79, 1.0))
    model.material("spring_yellow", rgba=(0.92, 0.72, 0.12, 1.0))
    model.material("seat_green", rgba=(0.18, 0.52, 0.28, 1.0))
    model.material("handle_red", rgba=(0.75, 0.15, 0.10, 1.0))
    model.material("silver_bolt", rgba=(0.74, 0.75, 0.78, 1.0))

    # -----------------------------------------------------------------
    # Fixed base: ground plate + spring lower mount plate
    # -----------------------------------------------------------------
    base = model.part("base")

    # Heavy ground plate
    base.visual(
        Cylinder(radius=PEDESTAL_R, length=PEDESTAL_H),
        origin=Origin(xyz=(0.0, 0.0, PEDESTAL_H / 2.0)),
        material="light_gray",
        name="ground_plate",
    )

    # Ground anchor bolts
    for i, angle in enumerate((0.0, math.pi / 2.0, math.pi, 1.5 * math.pi)):
        bx = 0.09 * math.cos(angle)
        by = 0.09 * math.sin(angle)
        base.visual(
            Cylinder(radius=0.008, length=0.025),
            origin=Origin(xyz=(bx, by, PEDESTAL_H + 0.005)),
            material="silver_bolt",
            name=f"anchor_bolt_{i}",
        )

    # Spring lower mount plate
    base.visual(
        Cylinder(radius=0.075, length=0.015),
        origin=Origin(xyz=(0.0, 0.0, PEDESTAL_H + 0.0075)),
        material="matte_black",
        name="spring_lower_plate",
    )

    # Spring coil (visual - on base for simplicity, but represents the spring body)
    spring_pts = _helix_points(SPRING_COIL_R, SPRING_HEIGHT, SPRING_TURNS, samples_per_turn=20)
    # Shift up to sit on the lower plate
    spring_pts_shifted = [(x, y, z + PEDESTAL_H + 0.015) for x, y, z in spring_pts]
    base.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                spring_pts_shifted,
                radius=SPRING_WIRE_R,
                samples_per_segment=4,
                radial_segments=12,
                cap_ends=True,
            ),
            "spring_coil",
        ),
        material="spring_yellow",
        name="spring_coil",
    )

    # -----------------------------------------------------------------
    # Spring carriage: pivot bracket that rides on top of the spring.
    # Moves vertically via prismatic joint to represent spring compression.
    # -----------------------------------------------------------------
    carriage = model.part("spring_carriage")

    # Upper spring plate
    carriage.visual(
        Cylinder(radius=0.075, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material="matte_black",
        name="spring_upper_plate",
    )

    # Pivot bracket box (captures the rocker axle)
    carriage.visual(
        Box((BRACKET_W, BRACKET_D, BRACKET_H)),
        origin=Origin(xyz=(0.0, 0.0, BRACKET_H / 2.0)),
        material="matte_black",
        name="pivot_bracket",
    )

    # Pivot bosses on bracket cheeks
    for i, sy in enumerate((1.0, -1.0)):
        carriage.visual(
            Cylinder(radius=0.04, length=0.018),
            origin=Origin(xyz=(0.0, sy * (BRACKET_D / 2.0 + 0.009), BRACKET_H), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="matte_black",
            name=f"pivot_boss_{i}",
        )
        # Bolt heads on bosses
        for j, ang in enumerate((0.0, math.pi / 2.0, math.pi, 1.5 * math.pi)):
            dx = 0.025 * math.cos(ang)
            dz = 0.025 * math.sin(ang)
            carriage.visual(
                Cylinder(radius=0.006, length=0.008),
                origin=Origin(
                    xyz=(dx, sy * (BRACKET_D / 2.0 + 0.020), BRACKET_H + dz),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material="silver_bolt",
                name=f"boss_bolt_{i}_{j}",
            )

    # -----------------------------------------------------------------
    # Rocker: steel beam + bumpers + molded seats + handle grips
    # Part frame sits at the pivot axis; geometry relative to that.
    # -----------------------------------------------------------------
    rocker = model.part("rocker")

    # Heavy steel banana beam, swept along a shallow parabola
    n = 14
    beam_pts = []
    for k in range(-n, n + 1):
        x = BEAM_HALF * k / n
        beam_pts.append((x, 0.0, _beam_z(x)))
    rocker.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                beam_pts,
                radius=BEAM_R,
                samples_per_segment=4,
                radial_segments=28,
                cap_ends=True,
            ),
            "beam_tube",
        ),
        material="steel_beam",
        name="beam_tube",
    )

    # Center reinforcement plate under beam (connects to pivot stub)
    rocker.visual(
        Box((0.20, 0.10, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material="steel_beam",
        name="center_plate",
    )

    # Pivot stub descending from center plate through the bracket (connects beam to bracket)
    rocker.visual(
        Cylinder(radius=0.035, length=0.21),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material="steel_beam",
        name="pivot_stub",
    )

    collar_z = _beam_z(COLLAR_X)
    slope = 2.0 * CURVE_C * COLLAR_X
    tangent = math.atan(slope)

    # Seat profile: rounded shape for molded seat
    seat_profile = sample_catmull_rom_spline_2d(
        [
            (0.20, 0.0),
            (0.08, 0.12),
            (-0.08, 0.14),
            (-0.18, 0.10),
            (-0.20, 0.0),
            (-0.18, -0.10),
            (-0.08, -0.14),
            (0.08, -0.12),
        ],
        samples_per_segment=8,
        closed=True,
    )

    # Seat lip profile (slightly larger, for the raised rim)
    lip_profile = sample_catmull_rom_spline_2d(
        [
            (0.215, 0.0),
            (0.09, 0.135),
            (-0.09, 0.155),
            (-0.195, 0.11),
            (-0.215, 0.0),
            (-0.195, -0.11),
            (-0.09, -0.155),
            (0.09, -0.135),
        ],
        samples_per_segment=8,
        closed=True,
    )

    for i, s in enumerate((1.0, -1.0)):
        # Black clamp collar ring around the beam
        rocker.visual(
            Cylinder(radius=0.085, length=0.09),
            origin=Origin(
                xyz=(s * COLLAR_X, 0.0, collar_z),
                rpy=(0.0, math.pi / 2.0 - s * tangent, 0.0),
            ),
            material="matte_black",
            name=f"clamp_collar_{i}",
        )
        for j, sy in enumerate((1.0, -1.0)):
            rocker.visual(
                Cylinder(radius=0.010, length=0.030),
                origin=Origin(
                    xyz=(s * COLLAR_X, sy * 0.088, collar_z),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material="silver_bolt",
                name=f"collar_bolt_{i}_{j}",
            )

        # Drop tube from collar to seat
        drop_pts = [
            (s * COLLAR_X, 0.0, collar_z),
            (s * 1.05, 0.0, 0.18),
            (s * 1.12, 0.0, 0.10),
            (s * SEAT_CENTER_X, 0.0, SEAT_Z + PLATE_T / 2.0 + 0.01),
        ]
        rocker.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    drop_pts, radius=0.024, samples_per_segment=10, radial_segments=16
                ),
                f"drop_tube_{i}",
            ),
            material="steel_beam",
            name=f"drop_tube_{i}",
        )

        # Molded seat base (thicker green plastic)
        seat_base = ExtrudeGeometry(seat_profile, PLATE_T, cap=True, center=True)
        if s < 0:
            seat_base.rotate_z(math.pi)
        seat_base.translate(s * SEAT_CENTER_X, 0.0, SEAT_Z)
        rocker.visual(
            mesh_from_geometry(seat_base, f"seat_base_{i}"),
            material="seat_green",
            name=f"seat_base_{i}",
        )

        # Raised lip rim on seat (thin ring above the seat surface)
        lip_ring = ExtrudeGeometry(lip_profile, 0.008, cap=True, center=True)
        if s < 0:
            lip_ring.rotate_z(math.pi)
        lip_ring.translate(s * SEAT_CENTER_X, 0.0, SEAT_Z + PLATE_T / 2.0 + 0.004)
        rocker.visual(
            mesh_from_geometry(lip_ring, f"seat_lip_{i}"),
            material="seat_green",
            name=f"seat_lip_{i}",
        )

        # Rubber bumper at beam end (large black cylinder)
        end_z = _beam_z(s * BEAM_HALF)
        rocker.visual(
            Cylinder(radius=0.055, length=0.10),
            origin=Origin(
                xyz=(s * (BEAM_HALF + 0.02), 0.0, end_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material="rubber_black",
            name=f"bumper_{i}",
        )

        # Handle post (steel tube rising from beam)
        post_pts = [
            (s * COLLAR_X, 0.0, collar_z + 0.05),
            (s * 0.99, 0.0, 0.40),
            (s * 1.02, 0.0, 0.50),
            (s * HANDLE_X, 0.0, HANDLE_Z - 0.04),
        ]
        rocker.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    post_pts, radius=0.020, samples_per_segment=10, radial_segments=16
                ),
                f"handle_post_{i}",
            ),
            material="steel_beam",
            name=f"handle_post_{i}",
        )

        # Rounded handle grip (red rubber-coated capsule, horizontal)
        grip_geom = CapsuleGeometry(radius=0.022, length=0.14, radial_segments=16, height_segments=6)
        grip_geom.rotate_x(math.pi / 2.0)  # orient along Y
        grip_geom.translate(s * HANDLE_X, 0.0, HANDLE_Z)
        rocker.visual(
            mesh_from_geometry(grip_geom, f"handle_grip_{i}"),
            material="handle_red",
            name=f"handle_grip_{i}",
        )

        # Small cross-bar connecting grip to post top
        rocker.visual(
            Cylinder(radius=0.012, length=0.06),
            origin=Origin(xyz=(s * HANDLE_X, 0.0, HANDLE_Z - 0.03)),
            material="steel_beam",
            name=f"grip_stem_{i}",
        )

    # -----------------------------------------------------------------
    # Articulations
    # -----------------------------------------------------------------

    # Spring compression: prismatic joint, vertical, on the Z axis
    # The carriage moves downward to represent spring compression
    model.articulation(
        "spring_compress",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, PEDESTAL_H + 0.015 + SPRING_HEIGHT)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=2000.0, velocity=0.3, lower=0.0, upper=SPRING_COMPRESS
        ),
    )

    # Rocking pivot: horizontal axis across the seesaw length (Y axis)
    model.articulation(
        "rocker_pivot",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=rocker,
        origin=Origin(xyz=(0.0, 0.0, BRACKET_H)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=400.0, velocity=1.5, lower=-ROCK_LIMIT, upper=ROCK_LIMIT
        ),
    )

    return model


def _intersects(a, b, tol: float = 1e-4) -> bool:
    if a is None or b is None:
        return False
    return all(a[0][i] <= b[1][i] + tol and b[0][i] <= a[1][i] + tol for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    carriage = object_model.get_part("spring_carriage")
    rocker = object_model.get_part("rocker")
    spring_joint = object_model.get_articulation("spring_compress")
    pivot = object_model.get_articulation("rocker_pivot")

    # --- Spring mechanism ---
    # Spring coil contacts the carriage upper plate (seated interface)
    ctx.allow_overlap(
        base,
        carriage,
        elem_a="spring_coil",
        elem_b="spring_upper_plate",
        reason="The spring coil top is seated against the carriage upper plate that compresses it.",
    )
    # Spring coil also contacts the pivot bracket (spring captured between plates and bracket)
    ctx.allow_overlap(
        base,
        carriage,
        elem_a="spring_coil",
        elem_b="pivot_bracket",
        reason="The spring coil nestles under the pivot bracket that sits atop the upper plate.",
    )

    # Pivot stub passes through the upper plate and into the bracket
    ctx.allow_overlap(
        rocker,
        carriage,
        elem_a="pivot_stub",
        elem_b="spring_upper_plate",
        reason="The pivot stub passes through the spring upper plate on its way into the bracket.",
    )
    ctx.allow_overlap(
        rocker,
        carriage,
        elem_a="pivot_stub",
        elem_b="pivot_bracket",
        reason="The steel pivot stub descends into the cast pivot bracket that captures the rocking axle.",
    )
    ctx.expect_overlap(
        rocker,
        carriage,
        axes="z",
        elem_a="pivot_stub",
        elem_b="pivot_bracket",
        min_overlap=0.02,
        name="pivot stub inserted into bracket",
    )
    ctx.expect_within(
        rocker,
        carriage,
        axes="xy",
        inner_elem="pivot_stub",
        outer_elem="pivot_bracket",
        margin=0.0,
        name="pivot stub centered in bracket",
    )

    # Spring joint is prismatic with vertical axis
    ctx.check(
        "spring joint is prismatic",
        spring_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={spring_joint.articulation_type}",
    )
    spring_lim = spring_joint.motion_limits
    ctx.check(
        "spring compression range about 40mm",
        spring_lim is not None and abs(spring_lim.upper - SPRING_COMPRESS) < 0.005,
        details=f"limits=({spring_lim.lower}, {spring_lim.upper})",
    )

    # Spring coil exists on base
    spring_aabb = ctx.part_element_world_aabb(base, elem="spring_coil")
    ctx.check(
        "spring coil visible between base and carriage",
        spring_aabb is not None and spring_aabb[1][2] - spring_aabb[0][2] > 0.08,
        details=f"spring_coil={spring_aabb}",
    )

    # Spring compression pose: carriage moves down
    carriage_rest = ctx.part_world_aabb(carriage)
    with ctx.pose({spring_joint: SPRING_COMPRESS}):
        carriage_compressed = ctx.part_world_aabb(carriage)
        ctx.check(
            "spring compression moves carriage downward",
            carriage_rest is not None
            and carriage_compressed is not None
            and carriage_compressed[1][2] < carriage_rest[1][2] - 0.02,
            details=f"rest={carriage_rest}, compressed={carriage_compressed}",
        )

    # --- Rocking pivot ---
    lim = pivot.motion_limits
    ctx.check(
        "rocking range about +/- 15 degrees",
        lim is not None
        and abs(lim.lower + ROCK_LIMIT) < 0.02
        and abs(lim.upper - ROCK_LIMIT) < 0.02,
        details=f"limits=({lim.lower}, {lim.upper})",
    )
    ctx.check(
        "rocker pivot is revolute",
        pivot.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={pivot.articulation_type}",
    )

    # --- Heavy steel beam ---
    beam = ctx.part_element_world_aabb(rocker, elem="beam_tube")
    ctx.check(
        "beam tube spans the seesaw length",
        beam is not None and (beam[1][0] - beam[0][0]) >= 2.2,
        details=f"beam={beam}",
    )
    ctx.check(
        "beam sweeps upward toward both ends",
        beam is not None and (beam[1][2] - beam[0][2]) >= 0.20,
        details=f"beam z-range={None if beam is None else beam[1][2] - beam[0][2]}",
    )

    # --- Rubber bumpers at beam ends ---
    bumper0 = ctx.part_element_world_aabb(rocker, elem="bumper_0")
    bumper1 = ctx.part_element_world_aabb(rocker, elem="bumper_1")
    ctx.check(
        "rubber bumpers at both beam ends",
        bumper0 is not None
        and bumper1 is not None
        and bumper0[0][0] > 1.0
        and bumper1[1][0] < -1.0,
        details=f"bumper0={bumper0}, bumper1={bumper1}",
    )

    # --- Molded seats with raised lips ---
    seat0 = ctx.part_element_world_aabb(rocker, elem="seat_base_0")
    seat1 = ctx.part_element_world_aabb(rocker, elem="seat_base_1")
    lip0 = ctx.part_element_world_aabb(rocker, elem="seat_lip_0")
    lip1 = ctx.part_element_world_aabb(rocker, elem="seat_lip_1")
    ctx.check(
        "molded seats present at both ends",
        seat0 is not None and seat1 is not None,
        details=f"seat0={seat0}, seat1={seat1}",
    )
    ctx.check(
        "raised lips above seat surfaces",
        seat0 is not None
        and seat1 is not None
        and lip0 is not None
        and lip1 is not None
        and lip0[0][2] > seat0[0][2] + 0.01
        and lip1[0][2] > seat1[0][2] + 0.01,
        details=f"lip0={lip0}, lip1={lip1}, seat0={seat0}, seat1={seat1}",
    )

    # --- Rounded handle grips ---
    grip0 = ctx.part_element_world_aabb(rocker, elem="handle_grip_0")
    grip1 = ctx.part_element_world_aabb(rocker, elem="handle_grip_1")
    ctx.check(
        "rounded handle grips above beam ends",
        grip0 is not None
        and grip1 is not None
        and beam is not None
        and grip0[0][2] > beam[1][2]
        and grip1[0][2] > beam[1][2],
        details=f"grip0={grip0}, grip1={grip1}, beam={beam}",
    )

    # --- Overall envelope ---
    ra = ctx.part_world_aabb(rocker)
    ba = ctx.part_world_aabb(base)
    ctx.check(
        "overall length about 2.6 m",
        ra is not None and 2.3 <= (ra[1][0] - ra[0][0]) <= 2.9,
        details=f"rocker aabb={ra}",
    )

    # --- Decisive pose checks ---
    with ctx.pose({pivot: ROCK_LIMIT}):
        seat0_dn = ctx.part_element_world_aabb(rocker, elem="seat_base_0")
        seat1_up = ctx.part_element_world_aabb(rocker, elem="seat_base_1")
        ctx.check(
            "positive rock lowers seat_0 and raises seat_1",
            seat0_dn is not None
            and seat1_up is not None
            and seat0 is not None
            and seat1 is not None
            and seat0_dn[1][2] < seat0[1][2] - 0.10
            and seat1_up[1][2] > seat1[1][2] + 0.10,
            details=f"seat0_dn={seat0_dn}, seat1_up={seat1_up}",
        )
        rocker_dn = ctx.part_world_aabb(rocker)
        ctx.check(
            "rocker clears the ground at full tilt",
            rocker_dn is not None and rocker_dn[0][2] > 0.005,
            details=f"rocker={rocker_dn}",
        )

    return ctx.report()


object_model = build_object_model()
