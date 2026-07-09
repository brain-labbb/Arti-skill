from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    ConeGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    SphereGeometry,
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
# Low inclusive seesaw: overall ~0.65 m tall, easier for children to access.
# ---------------------------------------------------------------------------
PIVOT_Z = 0.28          # world height of the rocking axis (inside the bracket)
BEAM_R = 0.055          # main tube radius (~110 mm diameter)
BEAM_HALF = 1.15        # half-length of the curved main tube
CURVE_C = 0.10          # parabolic curvature (shallower for low inclusive)
BEAM_CENTER_Z = 0.12    # beam centerline height at x=0, relative to the pivot

COLLAR_X = 0.95         # clamp collar position along the beam
SEAT_CENTER_X = 1.12
SEAT_Z = 0.07           # seat mid-plane, relative to the pivot
PLATE_T = 0.012
HANDLE_X = 1.01
HANDLE_Z = 0.42         # handle grip center, relative to the pivot

ROCK_LIMIT = 0.262      # ~15 degrees each way

PEDESTAL_R = 0.08
PEDESTAL_H = 0.18
BRACKET_SIZE = (0.15, 0.12, 0.14)
BRACKET_CZ = 0.25       # bracket box center height (spans 0.18..0.32)

BUMPER_R = 0.038        # rubber bumper radius
BUMPER_H = 0.04         # rubber bumper cylinder height
BUMPER_COMPRESS = 0.025 # max bumper compression travel (meters)


def _beam_z(x: float) -> float:
    """Beam centerline height (relative to the pivot frame) at station x."""
    return BEAM_CENTER_Z + CURVE_C * x * x


def _build_molded_seat(sign: float) -> "MeshGeometry":
    """Build a molded bucket seat with raised lip rim.
    
    The seat is a shallow dish: flat bottom with raised curved lip around the
    edge. Built as a lathe profile revolved around Z.
    """
    # Lathe profile: (radius, z) points from center outward
    # Dish bottom, then rising lip
    profile = [
        (0.00, 0.000),    # center bottom
        (0.06, 0.000),    # inner flat
        (0.11, 0.002),    # slight rise
        (0.14, 0.008),    # beginning of lip curve
        (0.16, 0.020),    # lip rising
        (0.170, 0.038),   # lip peak
        (0.165, 0.046),   # lip top rounded
        (0.155, 0.040),   # inner lip curve down
        (0.14, 0.024),    # inner lip base
        (0.11, 0.012),    # inner dish
        (0.06, 0.008),    # seat center area
        (0.00, 0.008),    # center (slightly dished)
    ]
    seat = LatheGeometry(profile, segments=32, closed=True)
    # Orient: seat faces outward from center (sign determines direction)
    seat.translate(sign * SEAT_CENTER_X, 0.0, SEAT_Z)
    return seat


def _build_backrest(sign: float) -> "MeshGeometry":
    """Build a curved backrest panel behind the seat.
    
    A slightly curved plate rising behind the seat, providing back support.
    """
    # Use a rounded rectangle profile extruded vertically
    back_profile = rounded_rect_profile(0.28, 0.025, 0.012)
    backrest = ExtrudeGeometry(back_profile, 0.22, cap=True, center=True)
    # Rotate so the thin dimension faces outward (along X), wide face faces the rider
    backrest.rotate_y(math.pi / 2.0)
    # Position behind the seat (inboard side, toward center)
    backrest.translate(sign * (SEAT_CENTER_X - 0.16), 0.0, SEAT_Z + 0.14)
    return backrest


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="playground_seesaw_inclusive")

    model.material("gloss_red_orange", rgba=(0.88, 0.20, 0.06, 1.0))
    model.material("matte_black", rgba=(0.09, 0.09, 0.10, 1.0))
    model.material("light_gray", rgba=(0.78, 0.78, 0.79, 1.0))
    model.material("dark_gray_steel", rgba=(0.34, 0.36, 0.38, 1.0))
    model.material("silver_rivet", rgba=(0.74, 0.75, 0.78, 1.0))
    model.material("rubber_black", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("molded_seat_gray", rgba=(0.42, 0.44, 0.46, 1.0))
    model.material("grip_rubber", rgba=(0.18, 0.18, 0.20, 1.0))

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
            Cylinder(radius=0.048, length=0.020),
            origin=Origin(xyz=(0.0, sy * 0.070, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="matte_black",
            name=f"pivot_boss_{i}",
        )
        for j, ang in enumerate((0.25, 0.75, 1.25, 1.75)):
            dx = 0.030 * math.cos(ang * math.pi)
            dz = 0.030 * math.sin(ang * math.pi)
            base.visual(
                Cylinder(radius=0.008, length=0.011),
                origin=Origin(
                    xyz=(dx, sy * 0.082, PIVOT_Z + dz),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material="silver_rivet",
                name=f"bracket_bolt_{i}_{j}",
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
    wedge = ConeGeometry(0.078, 0.08, radial_segments=28).rotate_x(math.pi)
    wedge.translate(0.0, 0.0, 0.16)
    rocker.visual(
        mesh_from_geometry(wedge, "pivot_wedge"),
        material="gloss_red_orange",
        name="pivot_wedge",
    )

    # Short red stub descending from the beam into the black bracket.
    rocker.visual(
        Cylinder(radius=0.044, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material="gloss_red_orange",
        name="pivot_stub",
    )

    collar_z = _beam_z(COLLAR_X)
    slope = 2.0 * CURVE_C * COLLAR_X
    tangent = math.atan(slope)

    for i, s in enumerate((1.0, -1.0)):
        # Black clamp collar ring around the beam, aligned to the local tangent.
        rocker.visual(
            Cylinder(radius=0.074, length=0.080),
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
                    xyz=(s * COLLAR_X, sy * 0.076, collar_z),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material="silver_rivet",
                name=f"collar_bolt_{i}_{j}",
            )

        # Thin red tube branching downward-outboard from the collar to the seat.
        drop_pts = [
            (s * COLLAR_X, 0.0, collar_z),
            (s * 1.03, 0.0, 0.15),
            (s * 1.10, 0.0, 0.10),
            (s * SEAT_CENTER_X, 0.0, SEAT_Z + 0.008),
        ]
        rocker.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    drop_pts, radius=0.024, samples_per_segment=10, radial_segments=18
                ),
                f"drop_tube_{i}",
            ),
            material="gloss_red_orange",
            name=f"drop_tube_{i}",
        )

        # --- Molded bucket seat with raised lip rim ---
        seat_mesh = _build_molded_seat(s)
        rocker.visual(
            mesh_from_geometry(seat_mesh, f"molded_seat_{i}"),
            material="molded_seat_gray",
            name=f"molded_seat_{i}",
        )

        # --- Backrest panel behind the seat ---
        backrest_mesh = _build_backrest(s)
        rocker.visual(
            mesh_from_geometry(backrest_mesh, f"backrest_{i}"),
            material="molded_seat_gray",
            name=f"backrest_{i}",
        )

        # Backrest support strut connecting seat area to backrest
        strut_pts = [
            (s * SEAT_CENTER_X, 0.0, SEAT_Z + 0.02),
            (s * (SEAT_CENTER_X - 0.08), 0.0, SEAT_Z + 0.08),
            (s * (SEAT_CENTER_X - 0.14), 0.0, SEAT_Z + 0.16),
        ]
        rocker.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    strut_pts, radius=0.014, samples_per_segment=6, radial_segments=12
                ),
                f"backrest_strut_{i}",
            ),
            material="gloss_red_orange",
            name=f"backrest_strut_{i}",
        )

        # Rivets on seat (decorative fasteners)
        rivet_xy = [(0.10, 0.0), (0.0, 0.08), (0.0, -0.08), (-0.10, 0.06), (-0.10, -0.06)]
        for j, (lx, ly) in enumerate(rivet_xy):
            rocker.visual(
                Cylinder(radius=0.007, length=0.009),
                origin=Origin(xyz=(s * (SEAT_CENTER_X + lx), ly, SEAT_Z + 0.001)),
                material="silver_rivet",
                name=f"seat_rivet_{i}_{j}",
            )

        # Thin red post rising from the beam to the rounded handle grip.
        post_pts = [
            (s * COLLAR_X, 0.0, 0.24),
            (s * 0.97, 0.0, 0.32),
            (s * 0.99, 0.0, 0.38),
            (s * HANDLE_X, 0.0, HANDLE_Z - 0.02),
        ]
        rocker.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    post_pts, radius=0.020, samples_per_segment=10, radial_segments=18
                ),
                f"handle_post_{i}",
            ),
            material="gloss_red_orange",
            name=f"handle_post_{i}",
        )

        # --- Rounded handle grip: capsule bar oriented along Y (across seesaw) ---
        grip = CapsuleGeometry(radius=0.022, length=0.18, radial_segments=20, height_segments=6)
        grip.rotate_x(math.pi / 2.0)  # orient along Y axis
        grip.translate(s * HANDLE_X, 0.0, HANDLE_Z)
        rocker.visual(
            mesh_from_geometry(grip, f"handle_grip_{i}"),
            material="grip_rubber",
            name=f"handle_grip_{i}",
        )

        # Small grip end caps (visual detail)
        for gy in (-0.11, 0.11):
            cap = SphereGeometry(radius=0.024, width_segments=16, height_segments=10)
            cap.translate(s * HANDLE_X, gy, HANDLE_Z)
            rocker.visual(
                mesh_from_geometry(cap, f"grip_cap_{i}_{0 if gy < 0 else 1}"),
                material="matte_black",
                name=f"grip_cap_{i}_{0 if gy < 0 else 1}",
            )

    # -----------------------------------------------------------------
    # Rubber end bumpers: separate parts on prismatic joints at beam ends.
    # They compress vertically when the seesaw tilts to extreme angles.
    # -----------------------------------------------------------------
    for i, s in enumerate((1.0, -1.0)):
        bumper = model.part(f"bumper_{i}")
        # Rubber bumper: short cylinder (rubber pad) at beam end underside
        bumper_mesh = CylinderGeometry(
            radius=BUMPER_R, height=BUMPER_H,
            radial_segments=20, closed=True,
        )
        bumper.visual(
            mesh_from_geometry(bumper_mesh, f"bumper_body_{i}"),
            material="rubber_black",
            name=f"bumper_body_{i}",
        )
        # Metal mounting plate on top of bumper
        bumper.visual(
            Cylinder(radius=BUMPER_R + 0.006, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, BUMPER_H / 2.0 + 0.003)),
            material="dark_gray_steel",
            name=f"bumper_plate_{i}",
        )

        # Prismatic joint: bumper compresses vertically (Z axis)
        # Position bumper so body top clears beam bottom with small gap
        beam_end_z = _beam_z(s * BEAM_HALF)
        bumper_origin_z = beam_end_z - BEAM_R - BUMPER_H / 2.0 - 0.010

        model.articulation(
            f"bumper_{i}_joint",
            ArticulationType.PRISMATIC,
            parent=rocker,
            child=bumper,
            origin=Origin(xyz=(s * BEAM_HALF, 0.0, bumper_origin_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=200.0,
                velocity=0.5,
                lower=0.0,
                upper=BUMPER_COMPRESS,
            ),
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
    base = object_model.get_part("pedestal_mount")
    rocker = object_model.get_part("rocker")
    pivot = object_model.get_articulation("rocker_pivot")
    bumper_0 = object_model.get_part("bumper_0")
    bumper_1 = object_model.get_part("bumper_1")
    bumper_0_joint = object_model.get_articulation("bumper_0_joint")
    bumper_1_joint = object_model.get_articulation("bumper_1_joint")

    # The red pivot stub is intentionally captured inside the black bracket.
    ctx.allow_overlap(
        rocker,
        base,
        elem_a="pivot_stub",
        elem_b="pivot_bracket",
        reason="The red center stub descends into the cast pivot bracket that captures the rocking axle.",
    )

    # Bumper mounting plates seat against the beam underside (small local contact).
    for bi, bpart in enumerate((bumper_0, bumper_1)):
        ctx.allow_overlap(
            bpart,
            rocker,
            elem_a=f"bumper_plate_{bi}",
            elem_b="beam_tube",
            reason="The bumper mounting plate seats against the beam underside for attachment.",
        )
        ctx.expect_contact(
            bpart,
            rocker,
            elem_a=f"bumper_plate_{bi}",
            elem_b="beam_tube",
            contact_tol=0.015,
            name=f"bumper_{bi} plate contacts beam underside",
        )
    ctx.expect_overlap(
        rocker,
        base,
        axes="z",
        elem_a="pivot_stub",
        elem_b="pivot_bracket",
        min_overlap=0.03,
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

    # Hero beam: ~2.3 m long banana tube that dips at center and rises at ends.
    beam = ctx.part_element_world_aabb(rocker, elem="beam_tube")
    ctx.check(
        "beam tube spans the seesaw length",
        beam is not None and (beam[1][0] - beam[0][0]) >= 2.1,
        details=f"beam={beam}",
    )
    ctx.check(
        "beam sweeps upward toward both ends",
        beam is not None and (beam[1][2] - beam[0][2]) >= 0.18,
        details=f"beam z-range={None if beam is None else beam[1][2] - beam[0][2]}",
    )

    # Overall envelope: about 2.6 m long, low inclusive height ~0.65 m.
    ra = ctx.part_world_aabb(rocker)
    ba = ctx.part_world_aabb(base)
    ctx.check(
        "overall length about 2.6 m",
        ra is not None and 2.2 <= (ra[1][0] - ra[0][0]) <= 2.8,
        details=f"rocker aabb={ra}",
    )
    ctx.check(
        "low inclusive overall height about 0.60-0.80 m",
        ra is not None and ba is not None and 0.55 <= max(ra[1][2], ba[1][2]) <= 0.82,
        details=f"rocker={ra}, base={ba}",
    )

    # --- Molded seats with raised lips ---
    seat0 = ctx.part_element_world_aabb(rocker, elem="molded_seat_0")
    seat1 = ctx.part_element_world_aabb(rocker, elem="molded_seat_1")
    ctx.check(
        "molded seats exist at both ends",
        seat0 is not None and seat1 is not None,
        details=f"seat0={seat0}, seat1={seat1}",
    )
    ctx.check(
        "molded seats have raised lip height (z-extent > 0.03m)",
        seat0 is not None and seat1 is not None
        and (seat0[1][2] - seat0[0][2]) >= 0.03
        and (seat1[1][2] - seat1[0][2]) >= 0.03,
        details=f"seat0 z-range={seat0}, seat1 z-range={seat1}",
    )

    # --- Backrest panels ---
    back0 = ctx.part_element_world_aabb(rocker, elem="backrest_0")
    back1 = ctx.part_element_world_aabb(rocker, elem="backrest_1")
    ctx.check(
        "backrest panels exist behind both seats",
        back0 is not None and back1 is not None,
        details=f"back0={back0}, back1={back1}",
    )
    ctx.check(
        "backrests rise above the seat surface",
        back0 is not None and back1 is not None
        and seat0 is not None and seat1 is not None
        and back0[1][2] > seat0[1][2] - 0.01
        and back1[1][2] > seat1[1][2] - 0.01,
        details=f"back0={back0}, back1={back1}, seat0={seat0}, seat1={seat1}",
    )
    ctx.check(
        "backrests have vertical extent (at least 0.15m tall)",
        back0 is not None and back1 is not None
        and (back0[1][2] - back0[0][2]) >= 0.15
        and (back1[1][2] - back1[0][2]) >= 0.15,
        details=f"back0 z-range={back0}, back1 z-range={back1}",
    )

    # --- Rounded handle grips ---
    grip0 = ctx.part_element_world_aabb(rocker, elem="handle_grip_0")
    grip1 = ctx.part_element_world_aabb(rocker, elem="handle_grip_1")
    ctx.check(
        "rounded handle grips exist at both ends",
        grip0 is not None and grip1 is not None,
        details=f"grip0={grip0}, grip1={grip1}",
    )
    ctx.check(
        "handle grips above the beam ends",
        grip0 is not None
        and grip1 is not None
        and beam is not None
        and grip0[0][2] > beam[0][2] + 0.15
        and grip1[0][2] > beam[0][2] + 0.15,
        details=f"grip0={grip0}, grip1={grip1}, beam={beam}",
    )
    ctx.check(
        "handle grips have rounded thickness (y-extent > 0.10m)",
        grip0 is not None and grip1 is not None
        and (grip0[1][1] - grip0[0][1]) >= 0.10
        and (grip1[1][1] - grip1[0][1]) >= 0.10,
        details=f"grip0 y={grip0}, grip1 y={grip1}",
    )

    # --- Rubber end bumpers on prismatic joints ---
    bump0 = ctx.part_element_world_aabb(bumper_0, elem="bumper_body_0")
    bump1 = ctx.part_element_world_aabb(bumper_1, elem="bumper_body_1")
    ctx.check(
        "rubber bumpers exist at both beam ends",
        bump0 is not None and bump1 is not None,
        details=f"bump0={bump0}, bump1={bump1}",
    )
    ctx.check(
        "bumpers positioned near beam ends",
        bump0 is not None and bump1 is not None
        and bump0[0][0] > 0.9
        and bump1[1][0] < -0.9,
        details=f"bump0={bump0}, bump1={bump1}",
    )

    # Bumper prismatic joint limits
    b0_lim = bumper_0_joint.motion_limits
    b1_lim = bumper_1_joint.motion_limits
    ctx.check(
        "bumper_0 prismatic joint has vertical compression range",
        b0_lim is not None
        and b0_lim.lower >= 0.0
        and b0_lim.upper > 0.01
        and b0_lim.upper <= 0.05,
        details=f"bumper_0 limits=({b0_lim.lower}, {b0_lim.upper})",
    )
    ctx.check(
        "bumper_1 prismatic joint has vertical compression range",
        b1_lim is not None
        and b1_lim.lower >= 0.0
        and b1_lim.upper > 0.01
        and b1_lim.upper <= 0.05,
        details=f"bumper_1 limits=({b1_lim.lower}, {b1_lim.upper})",
    )

    # Bumper compression pose check: positive q compresses bumper upward
    bump0_rest = ctx.part_world_aabb(bumper_0)
    with ctx.pose({bumper_0_joint: BUMPER_COMPRESS}):
        bump0_compressed = ctx.part_world_aabb(bumper_0)
        ctx.check(
            "bumper_0 moves upward when compressed",
            bump0_rest is not None and bump0_compressed is not None
            and bump0_compressed[0][2] > bump0_rest[0][2] + 0.005,
            details=f"rest={bump0_rest}, compressed={bump0_compressed}",
        )

    # The two ends mirror each other across the pivot.
    def _cx(aabb):
        return 0.5 * (aabb[0][0] + aabb[1][0])

    ctx.check(
        "seat assemblies mirrored about the pivot",
        seat0 is not None
        and seat1 is not None
        and _cx(seat0) > 0.8
        and _cx(seat1) < -0.8
        and abs(_cx(seat0) + _cx(seat1)) < 0.04
        and abs(seat0[1][2] - seat1[1][2]) < 0.01,
        details=f"seat0={seat0}, seat1={seat1}",
    )
    ctx.check(
        "grip positions mirrored about the pivot",
        grip0 is not None and grip1 is not None and abs(_cx(grip0) + _cx(grip1)) < 0.04,
        details=f"grip0={grip0}, grip1={grip1}",
    )

    # Mounted, not floating: drop tubes reach seats, posts reach grip bars.
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
        "handle posts connect beam to grip bars",
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
        and abs(_cx(collar0)) > 0.80
        and abs(_cx(collar1)) > 0.80,
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

    # Decisive pose checks: the whole rocker tilts as one body; seats swap
    # height, everything clears the ground, the base stays put.
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
            "pedestal and bracket stay fixed while rocking",
            base_rest is not None and base_posed is not None and _intersects(base_rest, base_posed)
            and abs(base_rest[1][2] - base_posed[1][2]) < 1e-6,
            details=f"rest={base_rest}, posed={base_posed}",
        )
    with ctx.pose({pivot: -ROCK_LIMIT}):
        seat0_up = ctx.part_element_world_aabb(rocker, elem="molded_seat_0")
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

    # At least 3 non-fixed joints: rocker_pivot + 2 bumper prismatic joints
    all_joints = list(object_model.articulations)
    non_fixed = [j for j in all_joints if j.articulation_type != ArticulationType.FIXED]
    ctx.check(
        "at least 3 non-fixed joints (rocker pivot + 2 bumpers)",
        len(non_fixed) >= 3,
        details=f"non_fixed count={len(non_fixed)}",
    )

    return ctx.report()


object_model = build_object_model()
