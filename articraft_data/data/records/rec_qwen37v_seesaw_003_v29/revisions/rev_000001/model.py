from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    ClevisBracketGeometry,
    ConeGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
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
PIVOT_Z = 0.52          # world height of the rocking axis (at A-frame apex)
BEAM_R = 0.06           # main tube radius (~120 mm diameter)
BEAM_HALF = 1.15        # half-length of the curved main tube
CURVE_C = 0.1285        # parabolic curvature of the banana beam
BEAM_CENTER_Z = 0.0     # beam centerline height at x=0, relative to the pivot

COLLAR_X = 0.97         # clamp collar position along the beam
SEAT_CENTER_X = 1.14
SEAT_Z = -0.10          # seat center, relative to the pivot
HANDLE_X = 1.03
HANDLE_Z = 0.38         # handle center, relative to the pivot

ROCK_LIMIT = 0.262      # ~15 degrees each way

# A-frame dimensions
AFRAME_LEG_SPREAD = 0.42   # half-width of A-frame base on ground (Y direction)
AFRAME_LEG_ANGLE = 0.32    # outward lean angle of legs from vertical (rad)
AFRAME_TUBE_R = 0.032      # A-frame tube radius
AFRAME_FOOT_Z = 0.0        # ground level
AFRAME_CROSSBAR_Z = 0.15   # cross-bar height

PEDESTAL_R = 0.075
BRACKET_SIZE = (0.16, 0.13, 0.10)


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
    model.material("rubber_grip", rgba=(0.18, 0.18, 0.20, 1.0))

    # -----------------------------------------------------------------
    # Fixed base: central A-frame support with visible axle brackets.
    # Two angled tube legs form an inverted-V/A shape in the YZ plane,
    # with a cross-bar near the bottom and a clevis bracket at the apex.
    # -----------------------------------------------------------------
    base = model.part("aframe_base")

    # Two A-frame legs: angled tubes from wide ground stance to the apex
    leg_length = math.sqrt(AFRAME_LEG_SPREAD**2 + PIVOT_Z**2)
    leg_angle = math.atan2(AFRAME_LEG_SPREAD, PIVOT_Z)  # angle from vertical

    for i, sy in enumerate((1.0, -1.0)):
        # Each leg is a tube angled from ground outward to the apex
        leg_pts = [
            (0.0, sy * AFRAME_LEG_SPREAD, AFRAME_FOOT_Z),
            (0.0, sy * AFRAME_LEG_SPREAD * 0.55, PIVOT_Z * 0.5),
            (0.0, sy * 0.085, PIVOT_Z - 0.10),
        ]
        base.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    leg_pts,
                    radius=AFRAME_TUBE_R,
                    samples_per_segment=8,
                    radial_segments=20,
                    cap_ends=True,
                ),
                f"aframe_leg_{i}",
            ),
            material="light_gray",
            name=f"aframe_leg_{i}",
        )

        # Ground foot plate (flat disc at each leg base)
        base.visual(
            Cylinder(radius=0.055, length=0.012),
            origin=Origin(xyz=(0.0, sy * AFRAME_LEG_SPREAD, 0.006)),
            material="dark_gray_steel",
            name=f"foot_plate_{i}",
        )
        # Anchor bolt on each foot
        base.visual(
            Cylinder(radius=0.008, length=0.018),
            origin=Origin(xyz=(0.0, sy * AFRAME_LEG_SPREAD, 0.018)),
            material="silver_rivet",
            name=f"foot_bolt_{i}",
        )

    # Horizontal cross-bar connecting the two legs near the bottom
    crossbar_y = AFRAME_LEG_SPREAD * 0.72
    crossbar_z = AFRAME_CROSSBAR_Z
    base.visual(
        Cylinder(radius=0.022, length=crossbar_y * 2.0),
        origin=Origin(xyz=(0.0, 0.0, crossbar_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="light_gray",
        name="crossbar",
    )

    # Apex gusset plate (connects the two legs below the bracket)
    base.visual(
        Box((0.10, 0.18, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z - 0.16)),
        material="light_gray",
        name="apex_gusset",
    )

    # Axle bracket: clevis-style bracket mounted below the beam center.
    # The beam passes above; the pivot stub descends into this bracket.
    bracket = ClevisBracketGeometry(
        (0.14, 0.12, 0.10),
        gap_width=0.075,
        bore_diameter=0.048,
        bore_center_z=0.07,
        base_thickness=0.014,
        corner_radius=0.008,
    )
    bracket.translate(0.0, 0.0, PIVOT_Z - 0.13)
    base.visual(
        mesh_from_geometry(bracket, "axle_bracket"),
        material="matte_black",
        name="axle_bracket",
    )

    # Visible bolt heads on the bracket cheeks (penetrating into the clevis)
    for i, sy in enumerate((1.0, -1.0)):
        for j, bz in enumerate((PIVOT_Z - 0.16, PIVOT_Z - 0.145)):
            base.visual(
                Cylinder(radius=0.009, length=0.014),
                origin=Origin(
                    xyz=(0.0, sy * 0.055, bz),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material="silver_rivet",
                name=f"bracket_bolt_{i}_{j}",
            )

    # -----------------------------------------------------------------
    # Rocker: curved red beam + mirrored seat/handle ends.
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

    # Red flare wedge at the beam center, blending into the pivot area.
    wedge = ConeGeometry(0.085, 0.08, radial_segments=28).rotate_x(math.pi)
    wedge.translate(0.0, 0.0, 0.06)
    rocker.visual(
        mesh_from_geometry(wedge, "pivot_wedge"),
        material="gloss_red_orange",
        name="pivot_wedge",
    )

    # Short red stub descending from the beam center into the bracket.
    rocker.visual(
        Cylinder(radius=0.044, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, -0.07)),
        material="gloss_red_orange",
        name="pivot_stub",
    )

    # ---- Molded seats with raised lips ----
    # Profile for a shallow bowl: flat bottom curving up to a raised lip rim.
    seat_bowl_profile = [
        (0.00, -0.010),    # center bottom
        (0.06, -0.008),    # flat bottom inner
        (0.12, -0.004),    # gentle curve
        (0.16, 0.005),     # start of lip rise
        (0.19, 0.025),     # lip peak
        (0.20, 0.030),     # outer lip top
        (0.205, 0.020),    # outer lip drop
        (0.21, 0.000),     # outer rim base
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
        # Last point enters the bowl so the seat is connected to the rocker frame.
        drop_pts = [
            (s * COLLAR_X, 0.0, collar_z),
            (s * 1.05, 0.0, 0.02),
            (s * 1.12, 0.0, -0.06),
            (s * SEAT_CENTER_X, 0.0, SEAT_Z),
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

        # Molded seat: shallow bowl with raised lip rim (LatheGeometry)
        seat_bowl = LatheGeometry(seat_bowl_profile, segments=28, closed=True)
        if s < 0:
            seat_bowl.rotate_z(math.pi)
        seat_bowl.translate(s * SEAT_CENTER_X, 0.0, SEAT_Z)
        rocker.visual(
            mesh_from_geometry(seat_bowl, f"seat_bowl_{i}"),
            material="dark_gray_steel",
            name=f"seat_bowl_{i}",
        )

        # Raised backrest lip on the outboard side of each seat
        backrest = CylinderGeometry(radius=0.04, height=0.12, radial_segments=16, closed=True)
        backrest.rotate_x(math.pi / 2.0)
        backrest.translate(s * (SEAT_CENTER_X + 0.16), 0.0, SEAT_Z + 0.02)
        rocker.visual(
            mesh_from_geometry(backrest, f"seat_backrest_{i}"),
            material="dark_gray_steel",
            name=f"seat_backrest_{i}",
        )

        # Small black stop fin under the seat, overlapping with the bowl
        rocker.visual(
            Box((0.05, 0.024, 0.05)),
            origin=Origin(xyz=(s * 1.22, 0.0, SEAT_Z - 0.01)),
            material="matte_black",
            name=f"seat_fin_{i}",
        )

        # ---- Rounded handle grips ----
        # Thin red post rising from the beam to the handle grip
        post_pts = [
            (s * COLLAR_X, 0.0, 0.12),
            (s * 0.985, 0.0, 0.24),
            (s * 1.01, 0.0, 0.32),
            (s * HANDLE_X, 0.0, HANDLE_Z - 0.02),
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

        # Rounded handlebar grip: horizontal capsule oriented along Y
        grip = CapsuleGeometry(radius=0.025, length=0.16, radial_segments=16, height_segments=6)
        grip.rotate_x(math.pi / 2.0)
        grip.translate(s * HANDLE_X, 0.0, HANDLE_Z)
        rocker.visual(
            mesh_from_geometry(grip, f"handle_grip_{i}"),
            material="rubber_grip",
            name=f"handle_grip_{i}",
        )

        # Handlebar crossbar (thin red tube connecting post top to grip)
        rocker.visual(
            Cylinder(radius=0.016, length=0.10),
            origin=Origin(
                xyz=(s * HANDLE_X, 0.0, HANDLE_Z - 0.04),
                rpy=(0.0, 0.0, 0.0),
            ),
            material="gloss_red_orange",
            name=f"handle_stem_{i}",
        )

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

    return model


def _intersects(a, b, tol: float = 1e-4) -> bool:
    if a is None or b is None:
        return False
    return all(a[0][i] <= b[1][i] + tol and b[0][i] <= a[1][i] + tol for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("aframe_base")
    rocker = object_model.get_part("rocker")
    pivot = object_model.get_articulation("rocker_pivot")

    # The red pivot stub is intentionally captured inside the axle bracket.
    ctx.allow_overlap(
        rocker,
        base,
        elem_a="pivot_stub",
        elem_b="axle_bracket",
        reason="The red center stub descends into the clevis axle bracket that captures the rocking axle.",
    )
    ctx.expect_overlap(
        rocker,
        base,
        axes="z",
        elem_a="pivot_stub",
        elem_b="axle_bracket",
        min_overlap=0.02,
        name="pivot stub inserted into axle bracket",
    )
    ctx.expect_within(
        rocker,
        base,
        axes="xy",
        inner_elem="pivot_stub",
        outer_elem="axle_bracket",
        margin=0.0,
        name="pivot stub centered in axle bracket",
    )

    # A-frame legs exist and are spread apart at ground level
    leg0 = ctx.part_element_world_aabb(base, elem="aframe_leg_0")
    leg1 = ctx.part_element_world_aabb(base, elem="aframe_leg_1")
    ctx.check(
        "A-frame has two legs spread apart",
        leg0 is not None and leg1 is not None
        and (leg0[1][1] - leg0[0][1] + leg1[1][1] - leg1[0][1]) > 0.0
        and abs(0.5 * (leg0[0][1] + leg0[1][1]) + 0.5 * (leg1[0][1] + leg1[1][1])) < 0.05,
        details=f"leg0={leg0}, leg1={leg1}",
    )

    # Crossbar connects the two legs
    crossbar = ctx.part_element_world_aabb(base, elem="crossbar")
    ctx.check(
        "crossbar spans between A-frame legs",
        crossbar is not None and leg0 is not None and leg1 is not None
        and crossbar[1][1] > leg1[1][1] - 0.05
        and crossbar[0][1] < leg0[0][1] + 0.05,
        details=f"crossbar={crossbar}",
    )

    # Axle bracket at apex (mounted below the beam, at the top of the A-frame)
    bracket = ctx.part_element_world_aabb(base, elem="axle_bracket")
    ctx.check(
        "axle bracket at A-frame apex",
        bracket is not None and bracket[1][2] > PIVOT_Z - 0.12 and bracket[0][2] < PIVOT_Z,
        details=f"bracket={bracket}",
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
        beam is not None and (beam[1][2] - beam[0][2]) >= 0.20,
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
        ra is not None and ba is not None and 0.82 <= max(ra[1][2], ba[1][2]) <= 1.05,
        details=f"rocker={ra}, base={ba}",
    )

    # Molded seats with raised lips
    seat0 = ctx.part_element_world_aabb(rocker, elem="seat_bowl_0")
    seat1 = ctx.part_element_world_aabb(rocker, elem="seat_bowl_1")
    ctx.check(
        "molded bowl seats exist at both ends",
        seat0 is not None and seat1 is not None,
        details=f"seat0={seat0}, seat1={seat1}",
    )

    # Rounded handle grips
    grip0 = ctx.part_element_world_aabb(rocker, elem="handle_grip_0")
    grip1 = ctx.part_element_world_aabb(rocker, elem="handle_grip_1")
    ctx.check(
        "rounded handle grips exist above beam ends",
        grip0 is not None
        and grip1 is not None
        and beam is not None
        and grip0[0][2] > beam[1][2] - 0.10
        and grip1[0][2] > beam[1][2] - 0.10,
        details=f"grip0={grip0}, grip1={grip1}, beam={beam}",
    )

    # The two ends mirror each other across the pivot.
    def _cx(aabb):
        return 0.5 * (aabb[0][0] + aabb[1][0])

    ctx.check(
        "seat bowls mirrored about the pivot",
        seat0 is not None
        and seat1 is not None
        and _cx(seat0) > 0.9
        and _cx(seat1) < -0.9
        and abs(_cx(seat0) + _cx(seat1)) < 0.02,
        details=f"seat0={seat0}, seat1={seat1}",
    )
    ctx.check(
        "handle grips mirrored about the pivot",
        grip0 is not None and grip1 is not None and abs(_cx(grip0) + _cx(grip1)) < 0.02,
        details=f"grip0={grip0}, grip1={grip1}",
    )

    # Mounted, not floating: drop tubes reach seats, posts reach grips
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
        "handle posts connect beam to grips",
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

    # Revolute joint exists and is non-fixed
    ctx.check(
        "rocker_pivot is a revolute joint",
        pivot is not None and pivot.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={pivot.articulation_type if pivot else None}",
    )

    # Decisive pose checks: the whole rocker tilts as one body
    base_rest = ctx.part_world_aabb(base)
    with ctx.pose({pivot: ROCK_LIMIT}):
        seat0_dn = ctx.part_element_world_aabb(rocker, elem="seat_bowl_0")
        seat1_up = ctx.part_element_world_aabb(rocker, elem="seat_bowl_1")
        rocker_dn = ctx.part_world_aabb(rocker)
        base_posed = ctx.part_world_aabb(base)
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
        ctx.check(
            "rocker clears the ground at full tilt",
            rocker_dn is not None and rocker_dn[0][2] > 0.005,
            details=f"rocker={rocker_dn}",
        )
        ctx.check(
            "A-frame stays fixed while rocking",
            base_rest is not None and base_posed is not None and _intersects(base_rest, base_posed)
            and abs(base_rest[1][2] - base_posed[1][2]) < 1e-6,
            details=f"rest={base_rest}, posed={base_posed}",
        )
    with ctx.pose({pivot: -ROCK_LIMIT}):
        seat0_up = ctx.part_element_world_aabb(rocker, elem="seat_bowl_0")
        rocker_up = ctx.part_world_aabb(rocker)
        ctx.check(
            "negative rock raises seat_0",
            seat0_up is not None and seat0 is not None and seat0_up[0][2] > seat0[0][2] + 0.10,
            details=f"seat0_up={seat0_up}",
        )
        ctx.check(
            "rocker clears the ground at opposite tilt",
            rocker_up is not None and rocker_up[0][2] > 0.005,
            details=f"rocker={rocker_up}",
        )

    return ctx.report()


object_model = build_object_model()
