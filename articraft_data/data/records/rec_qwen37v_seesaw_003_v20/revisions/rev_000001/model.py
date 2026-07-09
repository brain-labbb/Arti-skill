from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sample_catmull_rom_spline_2d,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Shared dimensions (meters). World: X along the seesaw length, Z up.
# ---------------------------------------------------------------------------
PIVOT_Z = 0.36          # world height of the rocking axis (inside the bracket)
BEAM_R = 0.06           # main tube radius (~120 mm diameter)
BEAM_HALF = 1.15        # half-length of the curved main tube
CURVE_C = 0.1285        # parabolic curvature of the banana beam
BEAM_CENTER_Z = 0.16    # beam centerline height at x=0, relative to the pivot

COLLAR_X = 0.97         # clamp collar position along the beam
SEAT_CENTER_X = 1.14
# Asymmetric seat heights: side 0 is standard, side 1 sits ~50 mm lower.
SEAT_Z_0 = 0.062        # seat plate 0 mid-plane, relative to the pivot
SEAT_Z_1 = 0.012        # seat plate 1 mid-plane, ~50 mm lower
PLATE_T = 0.012
HANDLE_X = 1.03
HANDLE_Z_0 = 0.552      # handle plate 0 mid-plane
HANDLE_Z_1 = 0.502      # handle plate 1 mid-plane (slightly lower to match)

ROCK_LIMIT = 0.262      # ~15 degrees each way

PEDESTAL_R = 0.075
PEDESTAL_H = 0.22
BRACKET_SIZE = (0.16, 0.13, 0.17)
BRACKET_CZ = 0.295      # bracket box center height (spans 0.21 .. 0.38)

# Ground pad dimensions
PAD_R = 0.10
PAD_H = 0.015

# Bumper dimensions and prismatic travel
BUMPER_R = 0.035
BUMPER_H = 0.04
BUMPER_TRAVEL = 0.018   # max vertical compression

# Footrest dimensions
FOOTREST_W = 0.14
FOOTREST_D = 0.10
FOOTREST_T = 0.008
FOOTREST_RIDGE_H = 0.004


def _beam_z(x: float) -> float:
    """Beam centerline height (relative to the pivot frame) at station x."""
    return BEAM_CENTER_Z + CURVE_C * x * x


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="playground_seesaw")

    model.material("gloss_red_orange", rgba=(0.88, 0.20, 0.06, 1.0))
    model.material("matte_black", rgba=(0.09, 0.09, 0.10, 1.0))
    model.material("light_gray", rgba=(0.78, 0.78, 0.79, 1.0))
    model.material("dark_gray_steel", rgba=(0.34, 0.36, 0.38, 1.0))
    model.material("silver_rivet", rgba=(0.74, 0.75, 0.78, 1.0))
    model.material("rubber_black", rgba=(0.12, 0.12, 0.12, 1.0))
    model.material("rubber_gray", rgba=(0.22, 0.22, 0.24, 1.0))

    # -----------------------------------------------------------------
    # Fixed base: ground pads + light-gray pedestal + black cast bracket.
    # -----------------------------------------------------------------
    base = model.part("pedestal_mount")

    # Rubber ground pads under the pedestal (two stacked pads for stability).
    base.visual(
        Cylinder(radius=PAD_R, length=PAD_H),
        origin=Origin(xyz=(0.0, 0.0, PAD_H / 2.0)),
        material="rubber_black",
        name="ground_pad_main",
    )
    # Secondary smaller pad ring for visual realism.
    base.visual(
        Cylinder(radius=PAD_R + 0.015, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material="rubber_gray",
        name="ground_pad_flange",
    )

    # Pedestal sits on top of the ground pad.
    base.visual(
        Cylinder(radius=PEDESTAL_R, length=PEDESTAL_H),
        origin=Origin(xyz=(0.0, 0.0, PAD_H + PEDESTAL_H / 2.0)),
        material="light_gray",
        name="ground_pedestal",
    )
    base.visual(
        Box(BRACKET_SIZE),
        origin=Origin(xyz=(0.0, 0.0, BRACKET_CZ + PAD_H)),
        material="matte_black",
        name="pivot_bracket",
    )
    # Round pivot bosses on both bracket cheeks, with visible bolt heads.
    for i, sy in enumerate((1.0, -1.0)):
        base.visual(
            Cylinder(radius=0.055, length=0.022),
            origin=Origin(xyz=(0.0, sy * 0.0755, PIVOT_Z + PAD_H), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="matte_black",
            name=f"pivot_boss_{i}",
        )
        for j, ang in enumerate((0.25, 0.75, 1.25, 1.75)):
            dx = 0.034 * math.cos(ang * math.pi)
            dz = 0.034 * math.sin(ang * math.pi)
            base.visual(
                Cylinder(radius=0.0085, length=0.012),
                origin=Origin(
                    xyz=(dx, sy * 0.0895, PIVOT_Z + PAD_H + dz),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material="silver_rivet",
                name=f"bracket_bolt_{i}_{j}",
            )

    # -----------------------------------------------------------------
    # Rocker: curved red beam + pivot stub + mirrored seat/handle ends
    # (asymmetric heights).
    # Part frame sits on the pivot axis; geometry is authored relative
    # to that frame so the revolute joint needs no extra offset.
    # -----------------------------------------------------------------
    rocker = model.part("rocker")

    # Thick glossy banana beam, swept along a shallow parabola.
    n = 12
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
        material="gloss_red_orange",
        name="beam_tube",
    )

    # Red flare wedge under the beam center, blending into the pivot stub.
    wedge = ConeGeometry(0.085, 0.09, radial_segments=28).rotate_x(math.pi)
    wedge.translate(0.0, 0.0, 0.110)
    rocker.visual(
        mesh_from_geometry(wedge, "pivot_wedge"),
        material="gloss_red_orange",
        name="pivot_wedge",
    )

    # Short red stub descending from the beam into the black bracket.
    rocker.visual(
        Cylinder(radius=0.048, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material="gloss_red_orange",
        name="pivot_stub",
    )

    seat_profile = sample_catmull_rom_spline_2d(
        [
            (0.21, 0.0),
            (0.05, 0.115),
            (-0.10, 0.145),
            (-0.185, 0.10),
            (-0.21, 0.0),
            (-0.185, -0.10),
            (-0.10, -0.145),
            (0.05, -0.115),
        ],
        samples_per_segment=8,
        closed=True,
    )
    grip_outer = rounded_rect_profile(0.18, 0.30, 0.05)
    grip_hole = rounded_rect_profile(0.06, 0.09, 0.02)
    grip_holes = [
        [(hx, hy + 0.075) for hx, hy in grip_hole],
        [(hx, hy - 0.075) for hx, hy in grip_hole],
    ]

    collar_z = _beam_z(COLLAR_X)
    slope = 2.0 * CURVE_C * COLLAR_X
    tangent = math.atan(slope)

    # Asymmetric seat and handle heights per side.
    seat_zs = [SEAT_Z_0, SEAT_Z_1]
    handle_zs = [HANDLE_Z_0, HANDLE_Z_1]

    for i, s in enumerate((1.0, -1.0)):
        seat_z = seat_zs[i]
        handle_z = handle_zs[i]

        # Black clamp collar ring around the beam, aligned to the local tangent.
        rocker.visual(
            Cylinder(radius=0.080, length=0.085),
            origin=Origin(
                xyz=(s * COLLAR_X, 0.0, collar_z),
                rpy=(0.0, math.pi / 2.0 - s * tangent, 0.0),
            ),
            material="matte_black",
            name=f"clamp_collar_{i}",
        )
        for j, sy in enumerate((1.0, -1.0)):
            rocker.visual(
                Cylinder(radius=0.011, length=0.032),
                origin=Origin(
                    xyz=(s * COLLAR_X, sy * 0.082, collar_z),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material="silver_rivet",
                name=f"collar_bolt_{i}_{j}",
            )

        # Thin red tube branching downward-outboard from the collar to the seat.
        # Asymmetric: side 1 drops lower to reach its lower seat.
        drop_pts = [
            (s * COLLAR_X, 0.0, collar_z),
            (s * 1.05, 0.0, 0.185 - (0.0 if i == 0 else 0.05)),
            (s * 1.12, 0.0, 0.105 - (0.0 if i == 0 else 0.05)),
            (s * 1.15, 0.0, seat_z + 0.004),
        ]
        rocker.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    drop_pts, radius=0.026, samples_per_segment=10, radial_segments=18
                ),
                f"drop_tube_{i}",
            ),
            material="gloss_red_orange",
            name=f"drop_tube_{i}",
        )

        # Flat dark-gray rounded-triangular seat plate with rivets.
        seat = ExtrudeGeometry(seat_profile, PLATE_T, cap=True, center=True)
        if s < 0:
            seat.rotate_z(math.pi)
        seat.translate(s * SEAT_CENTER_X, 0.0, seat_z)
        rocker.visual(
            mesh_from_geometry(seat, f"seat_plate_{i}"),
            material="dark_gray_steel",
            name=f"seat_plate_{i}",
        )
        rivet_xy = [(0.13, 0.0), (0.0, 0.10), (0.0, -0.10), (-0.13, 0.075), (-0.13, -0.075)]
        for j, (lx, ly) in enumerate(rivet_xy):
            rocker.visual(
                Cylinder(radius=0.008, length=0.010),
                origin=Origin(xyz=(s * (SEAT_CENTER_X + lx), ly, seat_z + 0.008)),
                material="silver_rivet",
                name=f"seat_rivet_{i}_{j}",
            )
        # Small black stop fin under the seat nose.
        rocker.visual(
            Box((0.045, 0.022, 0.04)),
            origin=Origin(xyz=(s * 1.26, 0.0, seat_z - 0.024)),
            material="matte_black",
            name=f"seat_fin_{i}",
        )

        # Textured footrest plate near each seat (rubber with grip ridges).
        # Connected to the drop tube via a thin bracket arm.
        footrest_x = s * (SEAT_CENTER_X - 0.05)
        footrest_z = seat_z - 0.07
        # Connecting bracket arm from drop tube region down to footrest.
        bracket_top_z = seat_z + 0.02
        bracket_mid_z = (bracket_top_z + footrest_z) / 2.0
        rocker.visual(
            Box((0.018, 0.018, bracket_top_z - footrest_z + FOOTREST_T)),
            origin=Origin(xyz=(s * (SEAT_CENTER_X - 0.02), 0.0, (bracket_top_z + footrest_z) / 2.0)),
            material="matte_black",
            name=f"footrest_bracket_{i}",
        )
        rocker.visual(
            Box((FOOTREST_W, FOOTREST_D, FOOTREST_T)),
            origin=Origin(xyz=(footrest_x, 0.0, footrest_z)),
            material="rubber_gray",
            name=f"footrest_plate_{i}",
        )
        # Grip ridges on the footrest surface.
        for ri in range(4):
            rx = footrest_x + (ri - 1.5) * 0.028
            rocker.visual(
                Box((0.008, FOOTREST_D - 0.01, FOOTREST_RIDGE_H)),
                origin=Origin(xyz=(rx, 0.0, footrest_z + FOOTREST_T / 2.0 + FOOTREST_RIDGE_H / 2.0)),
                material="rubber_black",
                name=f"footrest_ridge_{i}_{ri}",
            )

        # Thin red post rising from the beam to the gray handlebar grip plate.
        post_pts = [
            (s * COLLAR_X, 0.0, 0.285 - (0.0 if i == 0 else 0.05)),
            (s * 0.985, 0.0, 0.40 - (0.0 if i == 0 else 0.05)),
            (s * 1.01, 0.0, 0.48 - (0.0 if i == 0 else 0.05)),
            (s * HANDLE_X, 0.0, handle_z - 0.002),
        ]
        rocker.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    post_pts, radius=0.021, samples_per_segment=10, radial_segments=18
                ),
                f"handle_post_{i}",
            ),
            material="gloss_red_orange",
            name=f"handle_post_{i}",
        )
        grip = ExtrudeWithHolesGeometry(grip_outer, grip_holes, PLATE_T, cap=True, center=True)
        grip.translate(s * HANDLE_X, 0.0, handle_z)
        rocker.visual(
            mesh_from_geometry(grip, f"handle_plate_{i}"),
            material="dark_gray_steel",
            name=f"handle_plate_{i}",
        )

    # -----------------------------------------------------------------
    # Rubber end bumpers at beam tips, on short prismatic joints.
    # These compress vertically when the seesaw rocks to its limits.
    # The bumper hangs below the beam underside with a mounting stud.
    # -----------------------------------------------------------------
    bumpers = []
    beam_bottom_at_tip = _beam_z(BEAM_HALF) - BEAM_R  # ~0.270 in rocker frame
    # Place bumper body clearly below beam, connected by a mounting stud.
    bumper_origin_z = beam_bottom_at_tip - BUMPER_H - 0.015
    stud_length = beam_bottom_at_tip - (bumper_origin_z + BUMPER_H / 2.0)

    for i, s in enumerate((1.0, -1.0)):
        bumper = model.part(f"bumper_{i}")
        # Rubber cylinder bumper body.
        bumper.visual(
            Cylinder(radius=BUMPER_R, length=BUMPER_H),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material="rubber_black",
            name=f"bumper_body_{i}",
        )
        # Mounting stud reaching up into the beam underside.
        bumper.visual(
            Cylinder(radius=0.012, length=stud_length),
            origin=Origin(xyz=(0.0, 0.0, BUMPER_H / 2.0 + stud_length / 2.0)),
            material="matte_black",
            name=f"bumper_stud_{i}",
        )
        # Small flange at stud top for visual contact with beam.
        bumper.visual(
            Cylinder(radius=BUMPER_R + 0.005, length=0.005),
            origin=Origin(xyz=(0.0, 0.0, BUMPER_H / 2.0 + stud_length + 0.0025)),
            material="matte_black",
            name=f"bumper_flange_{i}",
        )
        bumpers.append(bumper)

        # Prismatic joint: vertical compression from rest (0) to compressed (BUMPER_TRAVEL).
        # Axis is -Z so positive q compresses the bumper downward.
        model.articulation(
            f"bumper_compress_{i}",
            ArticulationType.PRISMATIC,
            parent=rocker,
            child=bumper,
            origin=Origin(xyz=(s * BEAM_HALF, 0.0, bumper_origin_z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=200.0, velocity=0.5, lower=0.0, upper=BUMPER_TRAVEL
            ),
        )

    # Single rocking pivot: horizontal axis across the seesaw length.
    # Account for the pad height offset.
    model.articulation(
        "rocker_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=rocker,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z + PAD_H)),
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
    base = object_model.get_part("pedestal_mount")
    rocker = object_model.get_part("rocker")
    pivot = object_model.get_articulation("rocker_pivot")
    bumper0 = object_model.get_part("bumper_0")
    bumper1 = object_model.get_part("bumper_1")
    bumper_joint0 = object_model.get_articulation("bumper_compress_0")
    bumper_joint1 = object_model.get_articulation("bumper_compress_1")

    # The red pivot stub is intentionally captured inside the black bracket.
    ctx.allow_overlap(
        rocker,
        base,
        elem_a="pivot_stub",
        elem_b="pivot_bracket",
        reason="The red center stub descends into the cast pivot bracket that captures the rocking axle.",
    )
    ctx.expect_overlap(
        rocker,
        base,
        axes="z",
        elem_a="pivot_stub",
        elem_b="pivot_bracket",
        min_overlap=0.04,
        name="pivot stub inserted into bracket",
    )
    ctx.expect_within(
        rocker,
        base,
        axes="xy",
        inner_elem="pivot_stub",
        outer_elem="pivot_bracket",
        margin=0.0,
        name="pivot stub centered in bracket",
    )

    # Bumper studs and flanges intentionally nest into the beam underside.
    for idx in range(2):
        bpart = object_model.get_part(f"bumper_{idx}")
        ctx.allow_overlap(
            bpart,
            rocker,
            elem_a=f"bumper_stud_{idx}",
            elem_b="beam_tube",
            reason=f"The bumper mounting stud inserts into the beam underside for a captured fit.",
        )
        ctx.allow_overlap(
            bpart,
            rocker,
            elem_a=f"bumper_flange_{idx}",
            elem_b="beam_tube",
            reason=f"The bumper flange seats flush against the beam underside.",
        )

    # Bracket seated on the pedestal (both visuals on the fixed base part).
    bracket = ctx.part_element_world_aabb(base, elem="pivot_bracket")
    pedestal = ctx.part_element_world_aabb(base, elem="ground_pedestal")
    ctx.check(
        "bracket sits atop ground pedestal",
        _intersects(bracket, pedestal),
        details=f"bracket={bracket}, pedestal={pedestal}",
    )

    # Hero beam: ~2.6 m long banana tube that dips at center and rises at ends.
    beam = ctx.part_element_world_aabb(rocker, elem="beam_tube")
    ctx.check(
        "beam tube spans the seesaw length",
        beam is not None and (beam[1][0] - beam[0][0]) >= 2.2,
        details=f"beam={beam}",
    )
    ctx.check(
        "beam sweeps upward toward both ends",
        beam is not None and (beam[1][2] - beam[0][2]) >= 0.25,
        details=f"beam z-range={None if beam is None else beam[1][2] - beam[0][2]}",
    )

    # Overall envelope: about 2.6 m long, about 0.9 m tall.
    ra = ctx.part_world_aabb(rocker)
    ba = ctx.part_world_aabb(base)
    ctx.check(
        "overall length about 2.6 m",
        ra is not None and 2.4 <= (ra[1][0] - ra[0][0]) <= 2.8,
        details=f"rocker aabb={ra}",
    )
    ctx.check(
        "overall height about 0.9 m",
        ra is not None and ba is not None and 0.80 <= max(ra[1][2], ba[1][2]) <= 1.0,
        details=f"rocker={ra}, base={ba}",
    )

    # End assemblies: seats hang below the beam, grip plates rise above it.
    seat0 = ctx.part_element_world_aabb(rocker, elem="seat_plate_0")
    seat1 = ctx.part_element_world_aabb(rocker, elem="seat_plate_1")
    grip0 = ctx.part_element_world_aabb(rocker, elem="handle_plate_0")
    grip1 = ctx.part_element_world_aabb(rocker, elem="handle_plate_1")
    ctx.check(
        "seats at sitting height below the beam",
        seat0 is not None
        and seat1 is not None
        and 0.30 <= seat0[1][2] <= 0.46
        and 0.28 <= seat1[1][2] <= 0.42,
        details=f"seat0={seat0}, seat1={seat1}",
    )
    ctx.check(
        "grip plates above the beam ends",
        grip0 is not None
        and grip1 is not None
        and beam is not None
        and grip0[0][2] > beam[1][2]
        and grip1[0][2] > beam[1][2],
        details=f"grip0={grip0}, grip1={grip1}, beam={beam}",
    )

    # ---- Variant 20 checks ----

    # Asymmetric seat heights: seat 1 is lower than seat 0 at rest.
    ctx.check(
        "asymmetric seat heights: seat 1 lower than seat 0",
        seat0 is not None
        and seat1 is not None
        and seat1[1][2] < seat0[1][2] - 0.02,
        details=f"seat0 top z={seat0[1][2] if seat0 else None}, seat1 top z={seat1[1][2] if seat1 else None}",
    )

    # Rubber ground pads exist under the pedestal.
    pad_main = ctx.part_element_world_aabb(base, elem="ground_pad_main")
    pad_flange = ctx.part_element_world_aabb(base, elem="ground_pad_flange")
    ctx.check(
        "rubber ground pad under pedestal",
        pad_main is not None
        and pedestal is not None
        and pad_main[1][2] <= pedestal[0][2] + 0.005,
        details=f"pad={pad_main}, pedestal={pedestal}",
    )
    ctx.check(
        "ground pad flange extends wider than pad",
        pad_main is not None
        and pad_flange is not None
        and (pad_flange[1][0] - pad_flange[0][0]) > (pad_main[1][0] - pad_main[0][0]),
        details=f"pad={pad_main}, flange={pad_flange}",
    )

    # Textured footrests exist near each seat.
    foot0 = ctx.part_element_world_aabb(rocker, elem="footrest_plate_0")
    foot1 = ctx.part_element_world_aabb(rocker, elem="footrest_plate_1")
    ctx.check(
        "footrest 0 near seat 0",
        foot0 is not None
        and seat0 is not None
        and abs(foot0[0][0] - seat0[0][0]) < 0.20
        and foot0[0][2] < seat0[0][2],
        details=f"foot0={foot0}, seat0={seat0}",
    )
    ctx.check(
        "footrest 1 near seat 1",
        foot1 is not None
        and seat1 is not None
        and abs(foot1[1][0] - seat1[1][0]) < 0.20
        and foot1[0][2] < seat1[0][2],
        details=f"foot1={foot1}, seat1={seat1}",
    )

    # Rubber end bumpers exist at beam tips.
    bump0_body = ctx.part_element_world_aabb(bumper0, elem="bumper_body_0")
    bump1_body = ctx.part_element_world_aabb(bumper1, elem="bumper_body_1")
    ctx.check(
        "bumper 0 at positive beam end",
        bump0_body is not None and beam is not None and bump0_body[0][0] > 0.9,
        details=f"bumper0={bump0_body}, beam={beam}",
    )
    ctx.check(
        "bumper 1 at negative beam end",
        bump1_body is not None and beam is not None and bump1_body[1][0] < -0.9,
        details=f"bumper1={bump1_body}, beam={beam}",
    )

    # Prismatic bumper joints: check limits and that compression works.
    blim0 = bumper_joint0.motion_limits
    blim1 = bumper_joint1.motion_limits
    ctx.check(
        "bumper prismatic joints have compression range",
        blim0 is not None
        and blim1 is not None
        and blim0.upper > 0.005
        and blim1.upper > 0.005
        and blim0.lower == 0.0
        and blim1.lower == 0.0,
        details=f"bumper0 limits=({blim0.lower if blim0 else None}, {blim0.upper if blim0 else None}), bumper1 limits=({blim1.lower if blim1 else None}, {blim1.upper if blim1 else None})",
    )

    # Decisive pose: compress bumper at rest.
    with ctx.pose({bumper_joint0: BUMPER_TRAVEL}):
        bump0_compressed = ctx.part_element_world_aabb(bumper0, elem="bumper_body_0")
        ctx.check(
            "bumper 0 compresses downward at max travel",
            bump0_compressed is not None
            and bump0_body is not None
            and bump0_compressed[1][2] < bump0_body[1][2] - 0.005,
            details=f"rest={bump0_body}, compressed={bump0_compressed}",
        )

    # The two ends are at opposite sides of the pivot (balanced beam).
    def _cx(aabb):
        return 0.5 * (aabb[0][0] + aabb[1][0])

    ctx.check(
        "seat assemblies at opposite ends of the balanced beam",
        seat0 is not None
        and seat1 is not None
        and _cx(seat0) > 0.9
        and _cx(seat1) < -0.9,
        details=f"seat0={seat0}, seat1={seat1}",
    )
    ctx.check(
        "grip plates at opposite ends",
        grip0 is not None and grip1 is not None and abs(_cx(grip0) + _cx(grip1)) < 0.10,
        details=f"grip0={grip0}, grip1={grip1}",
    )

    # Mounted, not floating: drop tubes reach seats, posts reach grip plates,
    # clamp collars ring the beam.
    drop0 = ctx.part_element_world_aabb(rocker, elem="drop_tube_0")
    drop1 = ctx.part_element_world_aabb(rocker, elem="drop_tube_1")
    post0 = ctx.part_element_world_aabb(rocker, elem="handle_post_0")
    post1 = ctx.part_element_world_aabb(rocker, elem="handle_post_1")
    collar0 = ctx.part_element_world_aabb(rocker, elem="clamp_collar_0")
    collar1 = ctx.part_element_world_aabb(rocker, elem="clamp_collar_1")
    ctx.check(
        "drop tubes connect beam to seats",
        _intersects(drop0, beam)
        and _intersects(drop0, seat0)
        and _intersects(drop1, beam)
        and _intersects(drop1, seat1),
        details=f"drop0={drop0}, drop1={drop1}",
    )
    ctx.check(
        "handle posts connect beam to grip plates",
        _intersects(post0, beam)
        and _intersects(post0, grip0)
        and _intersects(post1, beam)
        and _intersects(post1, grip1),
        details=f"post0={post0}, post1={post1}",
    )
    ctx.check(
        "clamp collars ring the beam near its ends",
        _intersects(collar0, beam)
        and _intersects(collar1, beam)
        and collar0 is not None
        and collar1 is not None
        and abs(_cx(collar0)) > 0.85
        and abs(_cx(collar1)) > 0.85,
        details=f"collar0={collar0}, collar1={collar1}",
    )

    # Joint limits: about +/- 15 degrees of rocking.
    lim = pivot.motion_limits
    ctx.check(
        "rocking range about +/- 15 degrees",
        lim is not None
        and abs(lim.lower + ROCK_LIMIT) < 0.02
        and abs(lim.upper - ROCK_LIMIT) < 0.02,
        details=f"limits=({lim.lower}, {lim.upper})",
    )

    # Non-fixed joint check: at least one revolute and two prismatic joints exist.
    all_joints = [
        object_model.get_articulation(n)
        for n in ("rocker_pivot", "bumper_compress_0", "bumper_compress_1")
    ]
    non_fixed = [j for j in all_joints if j is not None]
    ctx.check(
        "at least 3 non-fixed articulations exist",
        len(non_fixed) >= 3,
        details=f"found {len(non_fixed)} non-fixed joints",
    )

    # Decisive pose checks: the whole rocker tilts as one body; seats swap
    # height, everything clears the ground, the base stays put.
    base_rest = ctx.part_world_aabb(base)
    with ctx.pose({pivot: ROCK_LIMIT}):
        seat0_dn = ctx.part_element_world_aabb(rocker, elem="seat_plate_0")
        seat1_up = ctx.part_element_world_aabb(rocker, elem="seat_plate_1")
        rocker_dn = ctx.part_world_aabb(rocker)
        base_posed = ctx.part_world_aabb(base)
        ctx.check(
            "positive rock lowers seat_0 and raises seat_1",
            seat0_dn is not None
            and seat1_up is not None
            and seat0 is not None
            and seat1 is not None
            and seat0_dn[1][2] < seat0[1][2] - 0.15
            and seat1_up[1][2] > seat1[1][2] + 0.15,
            details=f"seat0_dn={seat0_dn}, seat1_up={seat1_up}",
        )
        ctx.check(
            "rocker clears the ground at full tilt",
            rocker_dn is not None and rocker_dn[0][2] > 0.005,
            details=f"rocker={rocker_dn}",
        )
        ctx.check(
            "pedestal and bracket stay fixed while rocking",
            base_rest is not None and base_posed is not None and _intersects(base_rest, base_posed)
            and abs(base_rest[1][2] - base_posed[1][2]) < 1e-6,
            details=f"rest={base_rest}, posed={base_posed}",
        )
    with ctx.pose({pivot: -ROCK_LIMIT}):
        seat0_up = ctx.part_element_world_aabb(rocker, elem="seat_plate_0")
        rocker_up = ctx.part_world_aabb(rocker)
        ctx.check(
            "negative rock raises seat_0",
            seat0_up is not None and seat0 is not None and seat0_up[0][2] > seat0[0][2] + 0.15,
            details=f"seat0_up={seat0_up}",
        )
        ctx.check(
            "rocker clears the ground at opposite tilt",
            rocker_up is not None and rocker_up[0][2] > 0.005,
            details=f"rocker={rocker_up}",
        )

    return ctx.report()


object_model = build_object_model()
