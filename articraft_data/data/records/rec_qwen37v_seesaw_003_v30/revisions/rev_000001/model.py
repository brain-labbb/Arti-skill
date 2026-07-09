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
PIVOT_Z = 0.34          # world height of the rocking axis
BEAM_R = 0.06           # main tube radius (~120 mm diameter)
BEAM_HALF = 1.15        # half-length of the curved main tube
CURVE_C = 0.1285        # parabolic curvature of the banana beam
BEAM_CENTER_Z = 0.16    # beam centerline height at x=0, relative to pivot

COLLAR_X = 0.97         # clamp collar position along the beam
SEAT_CENTER_X = 1.14
SEAT_Z = 0.062          # seat 0 plate mid-plane (right, low)
SEAT_1_Z = 0.142        # seat 1 plate mid-plane (left, raised 80 mm)
PLATE_T = 0.012
HANDLE_X = 1.03
HANDLE_Z = 0.552        # handle plate mid-plane, relative to pivot

ROCK_LIMIT = 0.262      # ~15 degrees each way

PEDESTAL_R = 0.075
PEDESTAL_H = 0.22
BRACKET_SIZE = (0.16, 0.13, 0.17)
BRACKET_CZ = 0.295      # bracket box center height (spans 0.21 .. 0.38)
BRACKET_TOP_Z = BRACKET_CZ + BRACKET_SIZE[2] / 2.0  # 0.38

# Central compression spring
SPRING_COILS = 5
SPRING_R = 0.058        # coil helix radius
SPRING_H = 0.055        # uncompressed spring height
SPRING_WIRE_R = 0.007   # wire cross-section radius
SPRING_COMPRESS = 0.025 # max compression travel (prismatic q)

# Support legs and rubber ground pads
LEG_LENGTH = 0.22
LEG_WIDTH = 0.05
LEG_HEIGHT = 0.02
LEG_CENTER_Y = PEDESTAL_R + LEG_LENGTH / 2.0   # 0.185
PAD_R = 0.032
PAD_H = 0.008
PAD_Y = PEDESTAL_R + LEG_LENGTH - 0.015         # 0.28


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
    model.material("rubber_pad", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("spring_steel", rgba=(0.60, 0.62, 0.65, 1.0))

    # -----------------------------------------------------------------
    # Fixed base: pedestal + bracket + support legs + rubber pads.
    # -----------------------------------------------------------------
    base = model.part("pedestal_mount")
    base.visual(
        Cylinder(radius=PEDESTAL_R, length=PEDESTAL_H),
        origin=Origin(xyz=(0.0, 0.0, PEDESTAL_H / 2.0)),
        material="light_gray",
        name="ground_pedestal",
    )
    base.visual(
        Box(BRACKET_SIZE),
        origin=Origin(xyz=(0.0, 0.0, BRACKET_CZ)),
        material="matte_black",
        name="pivot_bracket",
    )
    # Round pivot bosses on both bracket cheeks, with visible bolt heads.
    for i, sy in enumerate((1.0, -1.0)):
        base.visual(
            Cylinder(radius=0.055, length=0.022),
            origin=Origin(xyz=(0.0, sy * 0.0755, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="matte_black",
            name=f"pivot_boss_{i}",
        )
        for j, ang in enumerate((0.25, 0.75, 1.25, 1.75)):
            dx = 0.034 * math.cos(ang * math.pi)
            dz = 0.034 * math.sin(ang * math.pi)
            base.visual(
                Cylinder(radius=0.0085, length=0.012),
                origin=Origin(
                    xyz=(dx, sy * 0.0895, PIVOT_Z + dz),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material="silver_rivet",
                name=f"bracket_bolt_{i}_{j}",
            )

    # Support legs extending along ±Y from the pedestal base, with rubber
    # ground pads at each foot for anti-slip and vibration damping.
    for i, sy in enumerate((1.0, -1.0)):
        base.visual(
            Box((LEG_WIDTH, LEG_LENGTH, LEG_HEIGHT)),
            origin=Origin(xyz=(0.0, sy * LEG_CENTER_Y, LEG_HEIGHT / 2.0)),
            material="light_gray",
            name=f"support_leg_{i}",
        )
        base.visual(
            Cylinder(radius=PAD_R, length=PAD_H),
            origin=Origin(xyz=(0.0, sy * PAD_Y, PAD_H / 2.0)),
            material="rubber_pad",
            name=f"rubber_pad_{i}",
        )

    # -----------------------------------------------------------------
    # Rocker: curved red beam + pivot stub + mirrored seat/handle ends.
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

    # Asymmetric seat heights: seat 0 stays low, seat 1 raised 80 mm.
    seat_zs = [SEAT_Z, SEAT_1_Z]

    for i, s in enumerate((1.0, -1.0)):
        seat_z = seat_zs[i]
        dz = seat_z - SEAT_Z  # 0 for seat 0, 0.08 for seat 1

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
        # The lower portion shifts up for the raised seat (shorter drop).
        drop_pts = [
            (s * COLLAR_X, 0.0, collar_z),
            (s * 1.05, 0.0, 0.185 + dz),
            (s * 1.12, 0.0, 0.105 + dz),
            (s * 1.15, 0.0, 0.066 + dz),
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

        # Thin red post rising from the beam to the gray handlebar grip plate.
        post_pts = [
            (s * COLLAR_X, 0.0, 0.285),
            (s * 0.985, 0.0, 0.40),
            (s * 1.01, 0.0, 0.48),
            (s * HANDLE_X, 0.0, 0.550),
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
        grip.translate(s * HANDLE_X, 0.0, HANDLE_Z)
        rocker.visual(
            mesh_from_geometry(grip, f"handle_plate_{i}"),
            material="dark_gray_steel",
            name=f"handle_plate_{i}",
        )

    # -----------------------------------------------------------------
    # Central compression spring under the beam, on a prismatic joint.
    # The spring sits atop the bracket and presses against the beam
    # underside, providing bounce and damping during rocking.
    # -----------------------------------------------------------------
    spring = model.part("spring")

    # Helical coil: inset from both ends so the tube caps stay clear of
    # the bracket top and beam underside at rest.
    n_pts = SPRING_COILS * 20 + 1
    helix_inset = 0.008  # keep tube caps above bracket / below beam
    spring_helix_pts = []
    for k in range(n_pts):
        t = k / (n_pts - 1)
        angle = SPRING_COILS * 2.0 * math.pi * t
        sx = SPRING_R * math.cos(angle)
        sy = SPRING_R * math.sin(angle)
        sz = helix_inset + (SPRING_H - 2.0 * helix_inset) * t
        spring_helix_pts.append((sx, sy, sz))
    spring.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                spring_helix_pts,
                radius=SPRING_WIRE_R,
                samples_per_segment=4,
                radial_segments=12,
                cap_ends=True,
            ),
            "spring_coil",
        ),
        material="spring_steel",
        name="spring_coil",
    )
    # Bottom end plate (spring seat).
    spring.visual(
        Cylinder(radius=SPRING_R + 0.008, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material="dark_gray_steel",
        name="spring_plate_bottom",
    )
    # Top end plate (spring seat, contacts beam underside).
    spring.visual(
        Cylinder(radius=SPRING_R + 0.008, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, SPRING_H - 0.005)),
        material="dark_gray_steel",
        name="spring_plate_top",
    )

    # -----------------------------------------------------------------
    # Articulations
    # -----------------------------------------------------------------

    # Single rocking pivot: horizontal axis across the seesaw length.
    model.articulation(
        "rocker_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=rocker,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=400.0, velocity=1.5, lower=-ROCK_LIMIT, upper=ROCK_LIMIT
        ),
    )

    # Central spring compression: prismatic joint, positive q = downward
    # (compression). Axis = (0, 0, -1) so increasing q pushes the spring
    # toward the bracket, representing coil compression.
    model.articulation(
        "spring_compress",
        ArticulationType.PRISMATIC,
        parent=base,
        child=spring,
        origin=Origin(xyz=(0.0, 0.0, BRACKET_TOP_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=200.0, velocity=0.5, lower=0.0, upper=SPRING_COMPRESS
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
    spring = object_model.get_part("spring")
    pivot = object_model.get_articulation("rocker_pivot")
    spring_joint = object_model.get_articulation("spring_compress")

    # The red pivot stub is intentionally captured inside the black bracket.
    ctx.allow_overlap(
        rocker,
        base,
        elem_a="pivot_stub",
        elem_b="pivot_bracket",
        reason="The red center stub descends into the cast pivot bracket that captures the rocking axle.",
    )
    # The pivot wedge and stub pass through the spring end plates
    # (clearance holes in the real assembly).
    ctx.allow_overlap(
        rocker,
        spring,
        elem_a="pivot_wedge",
        elem_b="spring_plate_top",
        reason="The pivot wedge passes through a clearance hole in the spring top end plate.",
    )
    ctx.allow_overlap(
        rocker,
        spring,
        elem_a="pivot_stub",
        elem_b="spring_plate_bottom",
        reason="The pivot stub passes through a clearance hole in the spring bottom end plate.",
    )
    ctx.allow_overlap(
        rocker,
        spring,
        elem_a="pivot_stub",
        elem_b="spring_plate_top",
        reason="The pivot stub passes through a clearance hole in the spring top end plate.",
    )
    # Proof: stub centered within the bottom plate footprint.
    ctx.expect_within(
        rocker,
        spring,
        axes="xy",
        inner_elem="pivot_stub",
        outer_elem="spring_plate_bottom",
        margin=0.0,
        name="pivot stub within spring bottom plate footprint",
    )
    # Proof: spring top plate overlaps the wedge vertically (near beam).
    ctx.expect_overlap(
        rocker,
        spring,
        axes="z",
        elem_a="pivot_wedge",
        elem_b="spring_plate_top",
        min_overlap=0.001,
        name="spring top plate positioned near wedge and beam underside",
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
        ra is not None and ba is not None and 0.82 <= max(ra[1][2], ba[1][2]) <= 0.98,
        details=f"rocker={ra}, base={ba}",
    )

    # ---- Asymmetric seat heights ----
    seat0 = ctx.part_element_world_aabb(rocker, elem="seat_plate_0")
    seat1 = ctx.part_element_world_aabb(rocker, elem="seat_plate_1")
    grip0 = ctx.part_element_world_aabb(rocker, elem="handle_plate_0")
    grip1 = ctx.part_element_world_aabb(rocker, elem="handle_plate_1")

    ctx.check(
        "seat_0 at reasonable sitting height",
        seat0 is not None and 0.30 <= seat0[1][2] <= 0.50,
        details=f"seat0={seat0}",
    )
    ctx.check(
        "seat_1 raised above seat_0 (asymmetric heights)",
        seat0 is not None
        and seat1 is not None
        and seat1[0][2] > seat0[0][2] + 0.04,
        details=f"seat0={seat0}, seat1={seat1}",
    )
    ctx.check(
        "seat height difference at least 60 mm",
        seat0 is not None
        and seat1 is not None
        and (seat1[1][2] - seat0[1][2]) >= 0.06,
        details=f"seat0 top={None if seat0 is None else seat0[1][2]}, seat1 top={None if seat1 is None else seat1[1][2]}",
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

    # Seats are at opposite ends of the beam (mirrored in X).
    def _cx(aabb):
        return 0.5 * (aabb[0][0] + aabb[1][0])

    ctx.check(
        "seat assemblies at opposite ends of the beam",
        seat0 is not None
        and seat1 is not None
        and _cx(seat0) > 0.9
        and _cx(seat1) < -0.9,
        details=f"seat0={seat0}, seat1={seat1}",
    )
    ctx.check(
        "grip plates mirrored about the pivot",
        grip0 is not None and grip1 is not None and abs(_cx(grip0) + _cx(grip1)) < 0.02,
        details=f"grip0={grip0}, grip1={grip1}",
    )

    # ---- Support legs and rubber ground pads ----
    leg0 = ctx.part_element_world_aabb(base, elem="support_leg_0")
    leg1 = ctx.part_element_world_aabb(base, elem="support_leg_1")
    pad0 = ctx.part_element_world_aabb(base, elem="rubber_pad_0")
    pad1 = ctx.part_element_world_aabb(base, elem="rubber_pad_1")

    ctx.check(
        "support legs extend from base",
        leg0 is not None and leg1 is not None,
        details=f"leg0={leg0}, leg1={leg1}",
    )
    ctx.check(
        "rubber ground pads present under support legs",
        pad0 is not None and pad1 is not None,
        details=f"pad0={pad0}, pad1={pad1}",
    )
    ctx.check(
        "rubber pads at ground level",
        pad0 is not None
        and pad1 is not None
        and pad0[0][2] < 0.01
        and pad1[0][2] < 0.01,
        details=f"pad0={pad0}, pad1={pad1}",
    )
    ctx.check(
        "legs extend laterally beyond pedestal for stability",
        leg0 is not None
        and leg1 is not None
        and leg0[1][1] > PEDESTAL_R + 0.05
        and leg1[0][1] < -(PEDESTAL_R + 0.05),
        details=f"leg0={leg0}, leg1={leg1}",
    )

    # ---- Spring: prismatic joint and compression geometry ----
    spring_coil = ctx.part_element_world_aabb(spring, elem="spring_coil")
    spring_top = ctx.part_element_world_aabb(spring, elem="spring_plate_top")
    spring_bot = ctx.part_element_world_aabb(spring, elem="spring_plate_bottom")

    ctx.check(
        "spring coil exists under the beam",
        spring_coil is not None,
        details=f"spring_coil={spring_coil}",
    )
    ctx.check(
        "spring sits between bracket top and beam",
        spring_coil is not None
        and bracket is not None
        and beam is not None
        and spring_coil[0][2] >= bracket[1][2] - 0.005
        and spring_coil[1][2] <= beam[0][2] + 0.005,
        details=f"spring={spring_coil}, bracket_top={None if bracket is None else bracket[1][2]}, beam_bottom={None if beam is None else beam[0][2]}",
    )

    slim = spring_joint.motion_limits
    ctx.check(
        "spring prismatic joint has compression range",
        slim is not None
        and slim.lower is not None
        and slim.upper is not None
        and slim.lower == 0.0
        and slim.upper >= 0.015,
        details=f"limits=({slim.lower}, {slim.upper})",
    )

    # Spring compression pose: positive q moves spring downward.
    spring_rest_pos = ctx.part_world_position(spring)
    with ctx.pose({spring_joint: SPRING_COMPRESS}):
        spring_compressed_pos = ctx.part_world_position(spring)
        ctx.check(
            "spring compresses downward at max prismatic travel",
            spring_rest_pos is not None
            and spring_compressed_pos is not None
            and spring_compressed_pos[2] < spring_rest_pos[2] - 0.01,
            details=f"rest_z={spring_rest_pos[2] if spring_rest_pos else None}, compressed_z={spring_compressed_pos[2] if spring_compressed_pos else None}",
        )

    # ---- Mounted, not floating: drop tubes reach seats, posts reach grips ----
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

    # Decisive pose checks: the whole rocker tilts as one body.
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
            base_rest is not None
            and base_posed is not None
            and _intersects(base_rest, base_posed)
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
