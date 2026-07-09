from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    ExtrudeWithHolesGeometry,
    LatheGeometry,
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
PIVOT_Z = 0.34          # world height of the rocking axis (inside the bracket)
BEAM_R = 0.06           # main tube radius (~120 mm diameter)
BEAM_HALF = 1.15        # half-length of the curved main tube
CURVE_C = 0.1285        # parabolic curvature of the banana beam
BEAM_CENTER_Z = 0.16    # beam centerline height at x=0, relative to the pivot

COLLAR_X = 0.97         # clamp collar position along the beam
SEAT_CENTER_X = 1.14
SEAT_Z = 0.062          # seat plate mid-plane, relative to the pivot
PLATE_T = 0.012
HANDLE_X = 1.03
HANDLE_Z = 0.552        # handle plate mid-plane, relative to the pivot

ROCK_LIMIT = 0.262      # ~15 degrees each way
HANDLEBAR_LIMIT = 0.175 # ~10 degrees each way for handlebar pivot

PEDESTAL_R = 0.075
PEDESTAL_H = 0.22
BRACKET_SIZE = (0.16, 0.13, 0.17)
BRACKET_CZ = 0.295      # bracket box center height (spans 0.21 .. 0.38)

# Handlebar post base in rocker frame (where the handlebar pivots)
HB_BASE_Z = 0.285


def _beam_z(x: float) -> float:
    """Beam centerline height (relative to the pivot frame) at station x."""
    return BEAM_CENTER_Z + CURVE_C * x * x


def _spring_helix(
    coil_r: float = 0.024,
    pitch: float = 0.022,
    turns: float = 4.5,
    n_per_turn: int = 24,
):
    """Generate helix points for a coil spring along +Z."""
    pts = []
    total = int(turns * n_per_turn)
    for k in range(total + 1):
        t = k / n_per_turn
        angle = 2.0 * math.pi * t
        x = coil_r * math.cos(angle)
        y = coil_r * math.sin(angle)
        z = pitch * t
        pts.append((x, y, z))
    return pts


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="playground_seesaw_spring")

    model.material("gloss_red_orange", rgba=(0.88, 0.20, 0.06, 1.0))
    model.material("matte_black", rgba=(0.09, 0.09, 0.10, 1.0))
    model.material("light_gray", rgba=(0.78, 0.78, 0.79, 1.0))
    model.material("dark_gray_steel", rgba=(0.34, 0.36, 0.38, 1.0))
    model.material("silver_rivet", rgba=(0.74, 0.75, 0.78, 1.0))
    model.material("spring_steel", rgba=(0.55, 0.56, 0.58, 1.0))
    model.material("molded_seat_green", rgba=(0.18, 0.42, 0.22, 1.0))

    # -----------------------------------------------------------------
    # Fixed base: short light-gray ground pedestal + black cast bracket.
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
        # Visible axle caps on outside of bracket cheeks
        base.visual(
            Cylinder(radius=0.032, length=0.018),
            origin=Origin(
                xyz=(0.0, sy * 0.096, PIVOT_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material="dark_gray_steel",
            name=f"axle_cap_{i}",
        )
        # Small center dot on axle cap
        base.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(
                xyz=(0.0, sy * 0.107, PIVOT_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material="silver_rivet",
            name=f"axle_cap_dot_{i}",
        )

    # -----------------------------------------------------------------
    # Rocker: curved red beam + pivot stub + springs + mirrored seat ends.
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

    # Spring-assist coils: two coil springs along the beam axis, flanking the stub.
    # Placed on the X axis (along beam) to avoid pivot bosses on Y axis.
    spring_pts = _spring_helix(coil_r=0.024, pitch=0.022, turns=4.5, n_per_turn=24)
    for i, sx in enumerate((1.0, -1.0)):
        spring_mesh = mesh_from_geometry(
            tube_from_spline_points(
                spring_pts,
                radius=0.005,
                samples_per_segment=4,
                radial_segments=12,
                cap_ends=True,
            ),
            f"spring_coil_{i}",
        )
        rocker.visual(
            spring_mesh,
            origin=Origin(xyz=(sx * 0.10, 0.0, 0.02)),
            material="spring_steel",
            name=f"spring_coil_{i}",
        )

    # Molded seat profile: concave bowl with raised lips
    seat_bowl_profile = [
        (0.000, 0.002),
        (0.045, -0.001),
        (0.090, -0.004),
        (0.130, -0.003),
        (0.155, 0.003),
        (0.170, 0.018),
        (0.180, 0.034),
        (0.186, 0.040),
        (0.192, 0.036),
        (0.196, 0.015),
        (0.198, -0.005),
        (0.198, -0.014),
        (0.000, -0.014),
    ]

    collar_z = _beam_z(COLLAR_X)
    slope = 2.0 * CURVE_C * COLLAR_X
    tangent = math.atan(slope)

    for i, s in enumerate((1.0, -1.0)):
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

        # Molded seat: concave bowl with raised lips
        seat_geom = LatheGeometry(seat_bowl_profile, segments=36, closed=True)
        seat_geom.translate(s * SEAT_CENTER_X, 0.0, SEAT_Z)
        rocker.visual(
            mesh_from_geometry(seat_geom, f"molded_seat_{i}"),
            material="molded_seat_green",
            name=f"molded_seat_{i}",
        )

        # Seat mounting rivets around the lip
        rivet_xy = [(0.14, 0.0), (0.0, 0.11), (0.0, -0.11), (-0.12, 0.08), (-0.12, -0.08)]
        for j, (lx, ly) in enumerate(rivet_xy):
            rocker.visual(
                Cylinder(radius=0.007, length=0.008),
                origin=Origin(xyz=(s * (SEAT_CENTER_X + lx), ly, SEAT_Z - 0.012)),
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

    # -----------------------------------------------------------------
    # Handlebar assemblies: each is a separate part that pivots on the rocker.
    # -----------------------------------------------------------------
    grip_outer = rounded_rect_profile(0.18, 0.30, 0.05)
    grip_hole = rounded_rect_profile(0.06, 0.09, 0.02)
    grip_holes = [
        [(hx, hy + 0.075) for hx, hy in grip_hole],
        [(hx, hy - 0.075) for hx, hy in grip_hole],
    ]

    for i, s in enumerate((1.0, -1.0)):
        hb = model.part(f"handlebar_{i}")

        # Handle post in handlebar local frame (base at origin, extends upward)
        local_post_pts = [
            (0.0, 0.0, 0.0),
            (s * (0.985 - COLLAR_X), 0.0, 0.40 - HB_BASE_Z),
            (s * (1.01 - COLLAR_X), 0.0, 0.48 - HB_BASE_Z),
            (s * (HANDLE_X - COLLAR_X), 0.0, HANDLE_Z - HB_BASE_Z),
        ]
        hb.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    local_post_pts,
                    radius=0.021,
                    samples_per_segment=10,
                    radial_segments=18,
                ),
                f"handle_post_{i}",
            ),
            material="gloss_red_orange",
            name=f"handle_post_{i}",
        )

        # Grip plate with hand cutouts
        grip = ExtrudeWithHolesGeometry(
            grip_outer, grip_holes, PLATE_T, cap=True, center=True
        )
        grip.translate(s * (HANDLE_X - COLLAR_X), 0.0, HANDLE_Z - HB_BASE_Z)
        hb.visual(
            mesh_from_geometry(grip, f"handle_plate_{i}"),
            material="dark_gray_steel",
            name=f"handle_plate_{i}",
        )

        # Small pivot shaft at base of handlebar
        hb.visual(
            Cylinder(radius=0.016, length=0.040),
            origin=Origin(xyz=(0.0, 0.0, -0.020)),
            material="silver_rivet",
            name=f"hb_pivot_shaft_{i}",
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

    # Handlebar pivots: each handlebar tilts slightly on the rocker.
    for i, s in enumerate((1.0, -1.0)):
        model.articulation(
            f"handlebar_pivot_{i}",
            ArticulationType.REVOLUTE,
            parent=rocker,
            child=f"handlebar_{i}",
            origin=Origin(xyz=(s * COLLAR_X, 0.0, HB_BASE_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=20.0, velocity=2.0,
                lower=-HANDLEBAR_LIMIT, upper=HANDLEBAR_LIMIT,
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
    hb0 = object_model.get_part("handlebar_0")
    hb1 = object_model.get_part("handlebar_1")
    pivot = object_model.get_articulation("rocker_pivot")
    hb_pivot_0 = object_model.get_articulation("handlebar_pivot_0")
    hb_pivot_1 = object_model.get_articulation("handlebar_pivot_1")

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

    # Handlebar posts emerge from inside the beam tube through the clamp collars.
    # Handlebar assemblies mount through the collar junction where beam tube,
    # clamp collar, and drop tube all converge. Whole-part allowance is scoped
    # to the intentional mounting interface at each end.
    ctx.allow_overlap(
        hb0,
        rocker,
        reason="Handlebar 0 assembly mounts through the clamp collar junction where beam tube, collar, and drop tube converge.",
    )
    ctx.allow_overlap(
        hb1,
        rocker,
        reason="Handlebar 1 assembly mounts through the clamp collar junction where beam tube, collar, and drop tube converge.",
    )
    ctx.expect_contact(
        hb0,
        rocker,
        name="handlebar_0 mounted at collar junction",
    )
    ctx.expect_contact(
        hb1,
        rocker,
        name="handlebar_1 mounted at collar junction",
    )

    # Springs connect bracket to beam; base partially nested in bracket opening.
    ctx.allow_overlap(
        rocker,
        base,
        elem_a="spring_coil_0",
        elem_b="pivot_bracket",
        reason="The spring coil base seats into the bracket opening to connect the spring-assist mechanism.",
    )
    ctx.allow_overlap(
        rocker,
        base,
        elem_a="spring_coil_1",
        elem_b="pivot_bracket",
        reason="The spring coil base seats into the bracket opening to connect the spring-assist mechanism.",
    )
    ctx.expect_overlap(
        rocker,
        base,
        axes="z",
        elem_a="spring_coil_0",
        elem_b="pivot_bracket",
        min_overlap=0.003,
        name="spring coil 0 engaged with bracket",
    )
    ctx.expect_overlap(
        rocker,
        base,
        axes="z",
        elem_a="spring_coil_1",
        elem_b="pivot_bracket",
        min_overlap=0.003,
        name="spring coil 1 engaged with bracket",
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

    # Overall envelope: about 2.6 m long, about 0.9 m tall (includes handlebars).
    ra = ctx.part_world_aabb(rocker)
    ba = ctx.part_world_aabb(base)
    ha0 = ctx.part_world_aabb(hb0)
    ha1 = ctx.part_world_aabb(hb1)
    all_top = max(
        ra[1][2] if ra else 0.0,
        ba[1][2] if ba else 0.0,
        ha0[1][2] if ha0 else 0.0,
        ha1[1][2] if ha1 else 0.0,
    )
    ctx.check(
        "overall length about 2.6 m",
        ra is not None and 2.4 <= (ra[1][0] - ra[0][0]) <= 2.8,
        details=f"rocker aabb={ra}",
    )
    ctx.check(
        "overall height about 0.9 m",
        0.82 <= all_top <= 0.98,
        details=f"rocker={ra}, base={ba}, hb0={ha0}, hb1={ha1}, top={all_top}",
    )

    # --- Variant 13 checks: spring-assisted, molded seats, axle caps, handlebar pivots ---

    # Springs: coil springs near the pivot area on the rocker
    spring0 = ctx.part_element_world_aabb(rocker, elem="spring_coil_0")
    spring1 = ctx.part_element_world_aabb(rocker, elem="spring_coil_1")
    ctx.check(
        "spring coils present near pivot",
        spring0 is not None and spring1 is not None,
        details=f"spring0={spring0}, spring1={spring1}",
    )
    ctx.check(
        "springs flank the pivot stub along the beam axis",
        spring0 is not None and spring1 is not None
        and spring0[0][0] > 0.04 and spring1[1][0] < -0.04,
        details=f"spring0={spring0}, spring1={spring1}",
    )

    # Axle caps: visible on bracket cheeks
    axle0 = ctx.part_element_world_aabb(base, elem="axle_cap_0")
    axle1 = ctx.part_element_world_aabb(base, elem="axle_cap_1")
    ctx.check(
        "axle caps visible at support bracket",
        axle0 is not None and axle1 is not None,
        details=f"axle0={axle0}, axle1={axle1}",
    )
    ctx.check(
        "axle caps on outside of bracket cheeks",
        axle0 is not None and axle1 is not None
        and axle0[0][1] > 0.06 and axle1[1][1] < -0.06,
        details=f"axle0={axle0}, axle1={axle1}",
    )

    # Molded seats with raised lips
    seat0 = ctx.part_element_world_aabb(rocker, elem="molded_seat_0")
    seat1 = ctx.part_element_world_aabb(rocker, elem="molded_seat_1")
    ctx.check(
        "molded seats exist at both ends",
        seat0 is not None and seat1 is not None,
        details=f"seat0={seat0}, seat1={seat1}",
    )
    ctx.check(
        "molded seats at sitting height",
        seat0 is not None
        and seat1 is not None
        and 0.33 <= seat0[1][2] <= 0.50
        and 0.33 <= seat1[1][2] <= 0.50,
        details=f"seat0={seat0}, seat1={seat1}",
    )
    ctx.check(
        "molded seats have raised lips (height > 0.03m)",
        seat0 is not None
        and seat1 is not None
        and (seat0[1][2] - seat0[0][2]) >= 0.03
        and (seat1[1][2] - seat1[0][2]) >= 0.03,
        details=f"seat0 z-range={seat0}, seat1 z-range={seat1}",
    )

    # Seats mirrored about pivot
    def _cx(aabb):
        return 0.5 * (aabb[0][0] + aabb[1][0])

    ctx.check(
        "seat assemblies mirrored about the pivot",
        seat0 is not None
        and seat1 is not None
        and _cx(seat0) > 0.9
        and _cx(seat1) < -0.9
        and abs(_cx(seat0) + _cx(seat1)) < 0.04
        and abs(seat0[1][2] - seat1[1][2]) < 0.02,
        details=f"seat0={seat0}, seat1={seat1}",
    )

    # Handlebar pivots: joints exist with correct limits
    lim0 = hb_pivot_0.motion_limits
    lim1 = hb_pivot_1.motion_limits
    ctx.check(
        "handlebar pivot 0 has non-zero range",
        lim0 is not None
        and lim0.lower is not None
        and lim0.upper is not None
        and lim0.lower < 0.0
        and lim0.upper > 0.0,
        details=f"limits=({lim0.lower}, {lim0.upper})",
    )
    ctx.check(
        "handlebar pivot 1 has non-zero range",
        lim1 is not None
        and lim1.lower is not None
        and lim1.upper is not None
        and lim1.lower < 0.0
        and lim1.upper > 0.0,
        details=f"limits=({lim1.lower}, {lim1.upper})",
    )

    # Handlebar parts exist with grip plates
    grip0 = ctx.part_element_world_aabb(hb0, elem="handle_plate_0")
    grip1 = ctx.part_element_world_aabb(hb1, elem="handle_plate_1")
    ctx.check(
        "handlebar grip plates above the beam ends",
        grip0 is not None
        and grip1 is not None
        and beam is not None
        and grip0[0][2] > beam[1][2]
        and grip1[0][2] > beam[1][2],
        details=f"grip0={grip0}, grip1={grip1}, beam={beam}",
    )
    ctx.check(
        "grip plates mirrored about the pivot",
        grip0 is not None and grip1 is not None and abs(_cx(grip0) + _cx(grip1)) < 0.04,
        details=f"grip0={grip0}, grip1={grip1}",
    )

    # Mounted: drop tubes reach seats, handlebar posts connect to grip plates
    drop0 = ctx.part_element_world_aabb(rocker, elem="drop_tube_0")
    drop1 = ctx.part_element_world_aabb(rocker, elem="drop_tube_1")
    ctx.check(
        "drop tubes connect beam to seats",
        _intersects(drop0, beam)
        and _intersects(drop0, seat0)
        and _intersects(drop1, beam)
        and _intersects(drop1, seat1),
        details=f"drop0={drop0}, drop1={drop1}",
    )

    post0 = ctx.part_element_world_aabb(hb0, elem="handle_post_0")
    post1 = ctx.part_element_world_aabb(hb1, elem="handle_post_1")
    ctx.check(
        "handlebar posts reach grip plates",
        _intersects(post0, grip0) and _intersects(post1, grip1),
        details=f"post0={post0}, post1={post1}",
    )

    # Clamp collars ring the beam near its ends
    collar0 = ctx.part_element_world_aabb(rocker, elem="clamp_collar_0")
    collar1 = ctx.part_element_world_aabb(rocker, elem="clamp_collar_1")
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

    # Decisive pose checks: rocker tilts, handlebars pivot
    base_rest = ctx.part_world_aabb(base)
    with ctx.pose({pivot: ROCK_LIMIT}):
        seat0_dn = ctx.part_element_world_aabb(rocker, elem="molded_seat_0")
        seat1_up = ctx.part_element_world_aabb(rocker, elem="molded_seat_1")
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
        seat0_up = ctx.part_element_world_aabb(rocker, elem="molded_seat_0")
        rocker_up = ctx.part_world_aabb(rocker)
        ctx.check(
            "negative rock raises seat_0",
            seat0_up is not None and seat0 is not None and seat0_up[0][2] > seat0[0][2] + 0.15,
            details=f"seat0_up={seat0_up}",
        )
        ctx.check(
            "rocker clears the ground at opposite tilt",
            rocker_up is not None and rocker_up[0][2] > 0.005,
            details=f"rocker_up={rocker_up}",
        )

    # Handlebar pivot pose check: grip moves when handlebar tilts
    grip0_rest = ctx.part_element_world_aabb(hb0, elem="handle_plate_0")
    with ctx.pose({hb_pivot_0: HANDLEBAR_LIMIT}):
        grip0_posed = ctx.part_element_world_aabb(hb0, elem="handle_plate_0")
        ctx.check(
            "handlebar_0 pivot tilts grip plate",
            grip0_rest is not None
            and grip0_posed is not None
            and abs(grip0_posed[0][0] - grip0_rest[0][0]) > 0.005,
            details=f"rest={grip0_rest}, posed={grip0_posed}",
        )

    return ctx.report()


object_model = build_object_model()
