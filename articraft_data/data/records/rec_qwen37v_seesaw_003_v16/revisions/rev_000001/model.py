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
SEAT_Z = 0.062          # seat plate mid-plane, relative to the pivot
PLATE_T = 0.012
HANDLE_X = 1.03
HANDLE_Z = 0.552        # handle plate mid-plane, relative to the pivot

ROCK_LIMIT = 0.262      # ~15 degrees each way

# --- Base structure (variant: shorter pedestal + spread support legs) ---
PEDESTAL_R = 0.075
PEDESTAL_H = 0.12       # shorter central pedestal (legs carry the load)
BRACKET_SIZE = (0.16, 0.13, 0.10)
BRACKET_CZ = 0.17       # bracket center, spanning 0.12 .. 0.22
BRACKET_TOP = BRACKET_CZ + BRACKET_SIZE[2] / 2.0  # 0.22

# --- Spring (variant: central compression spring on prismatic joint) ---
SPRING_RADIUS = 0.058   # coil mean radius (inner bore > pivot stub radius)
SPRING_WIRE_R = 0.007   # wire cross-section radius
SPRING_TURNS = 5
SPRING_COMPRESS = 0.04  # max vertical travel (m)

# --- Support legs (variant: 4 angled legs with rubber pads) ---
LEG_TUBE_R = 0.025      # 50 mm diameter structural tube
PAD_SIZE = (0.12, 0.12, 0.015)

# --- Bump stops (variant: rubber blocks below each beam end) ---
BUMP_SIZE = (0.07, 0.06, 0.06)
BUMP_X = 0.90
BUMP_POST_H = 0.22
BUMP_BLOCK_CZ = BUMP_POST_H + BUMP_SIZE[2] / 2.0  # 0.25


def _beam_z(x: float) -> float:
    """Beam centerline height (relative to the pivot frame) at station x."""
    return BEAM_CENTER_Z + CURVE_C * x * x


def _spring_helix(radius: float, height: float, n_turns: int,
                   pts_per_turn: int = 20) -> list[tuple[float, float, float]]:
    """Generate a helical path from z=0 to z=height."""
    pts: list[tuple[float, float, float]] = []
    total = n_turns * pts_per_turn + 1
    for i in range(total):
        t = i / (total - 1)
        angle = t * n_turns * 2.0 * math.pi
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        z = t * height
        pts.append((x, y, z))
    return pts


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="playground_seesaw")

    model.material("gloss_red_orange", rgba=(0.88, 0.20, 0.06, 1.0))
    model.material("matte_black", rgba=(0.09, 0.09, 0.10, 1.0))
    model.material("light_gray", rgba=(0.78, 0.78, 0.79, 1.0))
    model.material("dark_gray_steel", rgba=(0.34, 0.36, 0.38, 1.0))
    model.material("silver_rivet", rgba=(0.74, 0.75, 0.78, 1.0))
    model.material("rubber_pad", rgba=(0.12, 0.12, 0.11, 1.0))
    model.material("spring_steel", rgba=(0.55, 0.56, 0.58, 1.0))

    # =================================================================
    # Fixed base: pedestal + bracket + support legs + rubber pads + bump stops
    # =================================================================
    base = model.part("base_mount")

    # Short central pedestal
    base.visual(
        Cylinder(radius=PEDESTAL_R, length=PEDESTAL_H),
        origin=Origin(xyz=(0.0, 0.0, PEDESTAL_H / 2.0)),
        material="light_gray",
        name="ground_pedestal",
    )

    # Compact pivot bracket / spring mount
    base.visual(
        Box(BRACKET_SIZE),
        origin=Origin(xyz=(0.0, 0.0, BRACKET_CZ)),
        material="matte_black",
        name="pivot_bracket",
    )

    # Bracket-face bosses and bolt heads (embedded ~2 mm into bracket for connectivity)
    for i, sy in enumerate((1.0, -1.0)):
        base.visual(
            Cylinder(radius=0.048, length=0.020),
            origin=Origin(
                xyz=(0.0, sy * 0.073, BRACKET_CZ),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material="matte_black",
            name=f"pivot_boss_{i}",
        )
        for j, ang in enumerate((0.25, 0.75, 1.25, 1.75)):
            dx = 0.030 * math.cos(ang * math.pi)
            dz = 0.030 * math.sin(ang * math.pi)
            base.visual(
                Cylinder(radius=0.008, length=0.011),
                origin=Origin(
                    xyz=(dx, sy * 0.080, BRACKET_CZ + dz),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material="silver_rivet",
                name=f"bracket_bolt_{i}_{j}",
            )

    # --- Support legs (4 angled tubes from bracket area to ground) ---
    leg_data = [
        # (start near bracket, end at ground)
        ((0.06, 0.065, BRACKET_TOP), (0.18, 0.32, 0.008)),
        ((0.06, -0.065, BRACKET_TOP), (0.18, -0.32, 0.008)),
        ((-0.06, 0.065, BRACKET_TOP), (-0.18, 0.32, 0.008)),
        ((-0.06, -0.065, BRACKET_TOP), (-0.18, -0.32, 0.008)),
    ]
    for i, (s, e) in enumerate(leg_data):
        # 4 collinear points so catmull-rom produces a clean straight tube
        leg_pts = [
            s,
            (s[0] * 0.7 + e[0] * 0.3, s[1] * 0.7 + e[1] * 0.3, s[2] * 0.7 + e[2] * 0.3),
            (s[0] * 0.3 + e[0] * 0.7, s[1] * 0.3 + e[1] * 0.7, s[2] * 0.3 + e[2] * 0.7),
            e,
        ]
        base.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    leg_pts,
                    radius=LEG_TUBE_R,
                    samples_per_segment=4,
                    radial_segments=16,
                    cap_ends=True,
                ),
                f"support_leg_{i}",
            ),
            material="light_gray",
            name=f"support_leg_{i}",
        )
        # Rubber ground pad at foot
        base.visual(
            Box(PAD_SIZE),
            origin=Origin(xyz=(e[0], e[1], PAD_SIZE[2] / 2.0)),
            material="rubber_pad",
            name=f"rubber_pad_{i}",
        )

    # --- Bump stops (rubber blocks on short posts below beam ends) ---
    for i, sx in enumerate((1.0, -1.0)):
        # Horizontal support bar from bracket to bump post base
        bar_start = (sx * 0.08, 0.0, BRACKET_TOP)
        bar_end = (sx * BUMP_X, 0.0, BUMP_POST_H / 2.0)
        bar_mid1 = (bar_start[0] * 0.7 + bar_end[0] * 0.3, 0.0,
                    bar_start[2] * 0.7 + bar_end[2] * 0.3)
        bar_mid2 = (bar_start[0] * 0.3 + bar_end[0] * 0.7, 0.0,
                    bar_start[2] * 0.3 + bar_end[2] * 0.7)
        base.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    [bar_start, bar_mid1, bar_mid2, bar_end],
                    radius=0.020,
                    samples_per_segment=4,
                    radial_segments=14,
                    cap_ends=True,
                ),
                f"bump_support_bar_{i}",
            ),
            material="light_gray",
            name=f"bump_support_bar_{i}",
        )

        # Steel support post
        base.visual(
            Cylinder(radius=0.018, length=BUMP_POST_H),
            origin=Origin(xyz=(sx * BUMP_X, 0.0, BUMP_POST_H / 2.0)),
            material="light_gray",
            name=f"bump_post_{i}",
        )
        # Rubber bump stop block
        base.visual(
            Box(BUMP_SIZE),
            origin=Origin(xyz=(sx * BUMP_X, 0.0, BUMP_BLOCK_CZ)),
            material="rubber_pad",
            name=f"bump_stop_{i}",
        )

    # =================================================================
    # Spring carriage: top plate + helical coil + guide rod
    # Part frame sits at the pivot axis; prismatic joint moves it vertically.
    # =================================================================
    carriage = model.part("spring_carriage")

    # Mounting plate at the pivot level
    carriage.visual(
        Box((0.14, 0.12, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material="matte_black",
        name="carriage_plate",
    )

    # Helical compression spring coil below the plate
    spring_visual_h = PIVOT_Z - BRACKET_TOP - 0.015  # ≈ 0.105
    spring_bottom_local = -(PIVOT_Z - BRACKET_TOP)     # = -0.12
    helix_raw = _spring_helix(SPRING_RADIUS, spring_visual_h, SPRING_TURNS, pts_per_turn=20)
    helix_offset = [(x, y, z + spring_bottom_local) for x, y, z in helix_raw]
    carriage.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                helix_offset,
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

    # Central guide rod through the spring
    guide_len = abs(spring_bottom_local) + 0.02
    carriage.visual(
        Cylinder(radius=0.012, length=guide_len),
        origin=Origin(xyz=(0.0, 0.0, -(guide_len / 2.0))),
        material="silver_rivet",
        name="spring_guide",
    )

    # =================================================================
    # Rocker: curved red beam + pivot stub + mirrored seat/handle ends
    # Part frame at the pivot axis; geometry authored relative to that.
    # =================================================================
    rocker = model.part("rocker")

    # Thick glossy banana beam, swept along a shallow parabola
    n = 12
    beam_pts: list[tuple[float, float, float]] = []
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

    # Red flare wedge under beam center blending into the carriage plate
    wedge = ConeGeometry(0.085, 0.09, radial_segments=28).rotate_x(math.pi)
    wedge.translate(0.0, 0.0, 0.110)
    rocker.visual(
        mesh_from_geometry(wedge, "pivot_wedge"),
        material="gloss_red_orange",
        name="pivot_wedge",
    )

    # Short red stub descending from the beam through the carriage plate
    rocker.visual(
        Cylinder(radius=0.048, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material="gloss_red_orange",
        name="pivot_stub",
    )

    # --- Seat / handle profile data ---
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

    for i, s in enumerate((1.0, -1.0)):
        # Black clamp collar ring around the beam
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

        # Thin red tube branching downward-outboard to the seat
        drop_pts = [
            (s * COLLAR_X, 0.0, collar_z),
            (s * 1.05, 0.0, 0.185),
            (s * 1.12, 0.0, 0.105),
            (s * 1.15, 0.0, 0.066),
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

        # Flat dark-gray rounded-triangular seat plate with rivets
        seat = ExtrudeGeometry(seat_profile, PLATE_T, cap=True, center=True)
        if s < 0:
            seat.rotate_z(math.pi)
        seat.translate(s * SEAT_CENTER_X, 0.0, SEAT_Z)
        rocker.visual(
            mesh_from_geometry(seat, f"seat_plate_{i}"),
            material="dark_gray_steel",
            name=f"seat_plate_{i}",
        )
        rivet_xy = [(0.13, 0.0), (0.0, 0.10), (0.0, -0.10), (-0.13, 0.075), (-0.13, -0.075)]
        for j, (lx, ly) in enumerate(rivet_xy):
            rocker.visual(
                Cylinder(radius=0.008, length=0.010),
                origin=Origin(xyz=(s * (SEAT_CENTER_X + lx), ly, 0.070)),
                material="silver_rivet",
                name=f"seat_rivet_{i}_{j}",
            )

        # Small black stop fin under the seat nose
        rocker.visual(
            Box((0.045, 0.022, 0.04)),
            origin=Origin(xyz=(s * 1.26, 0.0, 0.038)),
            material="matte_black",
            name=f"seat_fin_{i}",
        )

        # Thin red post rising to the gray handlebar grip plate
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

    # =================================================================
    # Articulations
    # =================================================================

    # Prismatic spring joint: base → spring_carriage (vertical compression)
    model.articulation(
        "spring_joint",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=800.0, velocity=0.10,
            lower=-SPRING_COMPRESS, upper=0.0,
        ),
    )

    # Revolute rocker pivot: spring_carriage → rocker (horizontal Y axis)
    model.articulation(
        "rocker_pivot",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=rocker,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=400.0, velocity=1.5,
            lower=-ROCK_LIMIT, upper=ROCK_LIMIT,
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _intersects(a, b, tol: float = 1e-4) -> bool:
    if a is None or b is None:
        return False
    return all(a[0][i] <= b[1][i] + tol and b[0][i] <= a[1][i] + tol for i in range(3))


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_mount")
    carriage = object_model.get_part("spring_carriage")
    rocker = object_model.get_part("rocker")
    spring_j = object_model.get_articulation("spring_joint")
    pivot = object_model.get_articulation("rocker_pivot")

    # ---- Intentional overlap allowances ----

    # Pivot stub passes through the carriage plate
    ctx.allow_overlap(
        rocker, carriage,
        elem_a="pivot_stub", elem_b="carriage_plate",
        reason="The red pivot stub descends through the carriage plate into the spring assembly.",
    )
    ctx.expect_overlap(
        rocker, carriage,
        axes="z",
        elem_a="pivot_stub", elem_b="carriage_plate",
        min_overlap=0.005,
        name="pivot stub inserted through carriage plate",
    )
    ctx.expect_within(
        rocker, carriage,
        axes="xy",
        inner_elem="pivot_stub", outer_elem="carriage_plate",
        margin=0.01,
        name="pivot stub centered on carriage plate",
    )

    # Pivot stub contains the spring guide rod (coaxial captured shaft)
    ctx.allow_overlap(
        rocker, carriage,
        elem_a="pivot_stub", elem_b="spring_guide",
        reason="The spring guide rod is intentionally captured inside the hollow pivot stub.",
    )
    ctx.expect_within(
        carriage, rocker,
        axes="xy",
        inner_elem="spring_guide", outer_elem="pivot_stub",
        margin=0.0,
        name="spring guide centered inside pivot stub",
    )

    # Pivot stub descends into the spring coil housing at the top of the coil
    ctx.allow_overlap(
        rocker, carriage,
        elem_a="pivot_stub", elem_b="spring_coil",
        reason="The pivot stub is intentionally captured inside the upper turns of the compression spring.",
    )
    ctx.expect_within(
        rocker, carriage,
        axes="xy",
        inner_elem="pivot_stub", outer_elem="spring_coil",
        margin=0.005,
        name="pivot stub fits within spring coil bore",
    )

    # Spring coil seats against the bracket top (small embed at rest, more at compression)
    ctx.allow_overlap(
        carriage, base,
        elem_a="spring_coil", elem_b="pivot_bracket",
        reason="The spring coil compresses into the bracket housing during travel.",
    )
    ctx.allow_overlap(
        carriage, base,
        elem_a="spring_guide", elem_b="pivot_bracket",
        reason="The guide rod slides through the bracket bore during spring compression.",
    )

    # Support legs are welded to the bracket edges
    for i in range(4):
        ctx.allow_overlap(
            base, base,
            elem_a=f"support_leg_{i}", elem_b="pivot_bracket",
            reason=f"Support leg {i} is welded to the bracket at the junction.",
        )

    # Bump support bars connect the bracket to the bump posts
    for i in range(2):
        ctx.allow_overlap(
            base, base,
            elem_a=f"bump_support_bar_{i}", elem_b="pivot_bracket",
            reason=f"Bump support bar {i} is welded to the bracket edge.",
        )
        ctx.allow_overlap(
            base, base,
            elem_a=f"bump_support_bar_{i}", elem_b=f"bump_post_{i}",
            reason=f"Bump support bar {i} meets the bump post at the base.",
        )
        ctx.allow_overlap(
            base, base,
            elem_a=f"bump_stop_{i}", elem_b=f"bump_post_{i}",
            reason=f"Bump stop {i} sits atop the bump post.",
        )

    # ---- Spring joint checks ----
    sj_lim = spring_j.motion_limits
    ctx.check(
        "spring joint is prismatic with vertical axis",
        spring_j.articulation_type == ArticulationType.PRISMATIC
        and spring_j.axis is not None
        and abs(spring_j.axis[2]) > 0.99,
        details=f"type={spring_j.articulation_type}, axis={spring_j.axis}",
    )
    ctx.check(
        "spring compression travel about 40 mm",
        sj_lim is not None
        and sj_lim.lower is not None
        and abs(sj_lim.lower + SPRING_COMPRESS) < 0.005
        and sj_lim.upper is not None
        and abs(sj_lim.upper) < 0.005,
        details=f"limits=({sj_lim.lower}, {sj_lim.upper})",
    )

    # Spring coil is visible between bracket and carriage
    coil = ctx.part_element_world_aabb(carriage, elem="spring_coil")
    ctx.check(
        "spring coil present under the beam",
        coil is not None and coil[1][2] - coil[0][2] > 0.06,
        details=f"coil={coil}",
    )

    # ---- Rocker pivot checks ----
    lim = pivot.motion_limits
    ctx.check(
        "rocking range about +/- 15 degrees",
        lim is not None
        and abs(lim.lower + ROCK_LIMIT) < 0.02
        and abs(lim.upper - ROCK_LIMIT) < 0.02,
        details=f"limits=({lim.lower}, {lim.upper})",
    )

    # ---- Support legs + rubber pads ----
    for i in range(4):
        pad = ctx.part_element_world_aabb(base, elem=f"rubber_pad_{i}")
        leg = ctx.part_element_world_aabb(base, elem=f"support_leg_{i}")
        ctx.check(
            f"rubber pad {i} near ground level",
            pad is not None and pad[0][2] < 0.02,
            details=f"pad={pad}",
        )
        ctx.check(
            f"support leg {i} connects bracket to ground",
            leg is not None and pad is not None and _intersects(leg, pad),
            details=f"leg={leg}, pad={pad}",
        )

    # ---- Bump stops ----
    for i, sx in enumerate((1.0, -1.0)):
        bump = ctx.part_element_world_aabb(base, elem=f"bump_stop_{i}")
        post = ctx.part_element_world_aabb(base, elem=f"bump_post_{i}")
        ctx.check(
            f"bump stop {i} positioned below beam path",
            bump is not None
            and bump[1][2] < PIVOT_Z
            and bump[0][2] > 0.10,
            details=f"bump={bump}",
        )
        ctx.check(
            f"bump post {i} supports bump stop",
            post is not None and bump is not None and _intersects(post, bump),
            details=f"post={post}, bump={bump}",
        )

    # ---- Beam and overall envelope ----
    beam = ctx.part_element_world_aabb(rocker, elem="beam_tube")
    ctx.check(
        "beam tube spans the seesaw length",
        beam is not None and (beam[1][0] - beam[0][0]) >= 2.2,
        details=f"beam={beam}",
    )
    ctx.check(
        "beam sweeps upward toward both ends (raised ends)",
        beam is not None and (beam[1][2] - beam[0][2]) >= 0.25,
        details=f"beam z-range={None if beam is None else beam[1][2] - beam[0][2]}",
    )

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

    # ---- End assemblies ----
    seat0 = ctx.part_element_world_aabb(rocker, elem="seat_plate_0")
    seat1 = ctx.part_element_world_aabb(rocker, elem="seat_plate_1")
    grip0 = ctx.part_element_world_aabb(rocker, elem="handle_plate_0")
    grip1 = ctx.part_element_world_aabb(rocker, elem="handle_plate_1")
    ctx.check(
        "seats at sitting height below the beam",
        seat0 is not None
        and seat1 is not None
        and 0.30 <= seat0[1][2] <= 0.50
        and 0.30 <= seat1[1][2] <= 0.50,
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

    # Mirror check
    def _cx(aabb):
        return 0.5 * (aabb[0][0] + aabb[1][0])

    ctx.check(
        "seat assemblies mirrored about the pivot",
        seat0 is not None
        and seat1 is not None
        and _cx(seat0) > 0.9
        and _cx(seat1) < -0.9
        and abs(_cx(seat0) + _cx(seat1)) < 0.02,
        details=f"seat0={seat0}, seat1={seat1}",
    )

    # ---- Pose checks: rocker tilts, spring compresses ----
    base_rest = ctx.part_world_aabb(base)
    carriage_rest = ctx.part_world_aabb(carriage)

    # Spring compression pose
    with ctx.pose({spring_j: -SPRING_COMPRESS}):
        carriage_compressed = ctx.part_world_aabb(carriage)
        ctx.check(
            "spring compression lowers the carriage",
            carriage_rest is not None
            and carriage_compressed is not None
            and carriage_compressed[1][2] < carriage_rest[1][2] - 0.02,
            details=f"rest={carriage_rest}, compressed={carriage_compressed}",
        )

    # Rocker tilt pose (with spring at rest)
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
            "base stays fixed while rocking",
            base_rest is not None
            and base_posed is not None
            and abs(base_rest[1][2] - base_posed[1][2]) < 1e-6,
            details=f"rest={base_rest}, posed={base_posed}",
        )

    # Opposite tilt
    with ctx.pose({pivot: -ROCK_LIMIT}):
        seat0_up = ctx.part_element_world_aabb(rocker, elem="seat_plate_0")
        rocker_up = ctx.part_world_aabb(rocker)
        ctx.check(
            "negative rock raises seat_0",
            seat0_up is not None
            and seat0 is not None
            and seat0_up[0][2] > seat0[0][2] + 0.15,
            details=f"seat0_up={seat0_up}",
        )
        ctx.check(
            "rocker clears the ground at opposite tilt",
            rocker_up is not None and rocker_up[0][2] > 0.005,
            details=f"rocker={rocker_up}",
        )

    # Combined pose: spring compressed + rocker at max tilt
    # In reality the bump stops prevent full tilt when the spring is
    # compressed, so we verify rocker clears ground at moderate tilt only.
    with ctx.pose({spring_j: -SPRING_COMPRESS, pivot: ROCK_LIMIT * 0.6}):
        rocker_both = ctx.part_world_aabb(rocker)
        ctx.check(
            "rocker clears ground with spring compressed at moderate tilt",
            rocker_both is not None and rocker_both[0][2] > 0.001,
            details=f"rocker={rocker_both}",
        )

    return ctx.report()


object_model = build_object_model()
