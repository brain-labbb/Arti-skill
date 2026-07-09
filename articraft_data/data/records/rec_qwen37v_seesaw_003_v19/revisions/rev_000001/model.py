from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
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
PIVOT_Z = 0.45          # world height of the rocking axis (at A-frame top)
BEAM_R = 0.06           # main tube radius (~120 mm diameter)
BEAM_HALF = 1.15        # half-length of the curved main tube
CURVE_C = 0.1285        # parabolic curvature of the banana beam
BEAM_CENTER_Z = 0.16    # beam centerline height at x=0, relative to pivot

COLLAR_X = 0.97         # clamp collar position along the beam
SEAT_CENTER_X = 1.14
SEAT_Z = 0.062          # seat plate mid-plane, relative to pivot
PLATE_T = 0.016         # thicker molded seat plate
HANDLE_X = 1.03
HANDLE_Z = 0.552        # handle plate mid-plane, relative to pivot

ROCK_LIMIT = 0.262      # ~15 degrees each way

# A-frame dimensions
AFRAME_LEG_TOP_Y = 0.075   # Y offset where legs meet the top crossbar
AFRAME_LEG_BOT_Y = 0.32    # Y offset at ground (foot spread)
AFRAME_LEG_TOP_Z = PIVOT_Z - 0.05  # just below pivot axis
AFRAME_LEG_BOT_Z = 0.020   # foot pad center height
AFRAME_LEG_R = 0.028       # leg tube radius

# Crossbar spans the full distance between legs
CROSSBAR_W = 0.16          # crossbar width along X
CROSSBAR_D = 0.18          # crossbar depth along Y (spans between legs)
CROSSBAR_H = 0.035         # crossbar height
CROSSBAR_CZ = PIVOT_Z - 0.065  # crossbar center height

# Bracket plates (at A-frame top, holding the axle)
BRACKET_PLATE_T = 0.012    # plate thickness
BRACKET_PLATE_H = 0.15     # plate height
BRACKET_PLATE_W = 0.12     # plate width (along X)
BRACKET_PLATE_Y = 0.075    # Y offset of each plate from center
BRACKET_PLATE_CZ = PIVOT_Z - 0.01  # plate center z (straddles pivot)

# Axle
AXLE_R = 0.018             # axle radius
AXLE_HALF_LEN = 0.088      # half-length (spans between plates)

# Axle caps
AXLE_CAP_R = 0.032         # cap disc radius
AXLE_CAP_T = 0.010         # cap thickness

# Spring (helical compression spring between crossbar and beam)
SPRING_COILS = 5
SPRING_R = 0.044           # spring coil centerline radius
SPRING_TUBE_R = 0.008      # spring wire cross-section radius
SPRING_HEIGHT = 0.13       # total spring height at rest
# Spring sits between crossbar top and beam bottom, around the pivot area.
# Crossbar top = CROSSBAR_CZ + CROSSBAR_H/2 = 0.385 + 0.0175 = 0.4025
# Beam bottom at x=0 ≈ PIVOT_Z + BEAM_CENTER_Z - BEAM_R = 0.45 + 0.16 - 0.06 = 0.55
# Spring center = (0.4025 + 0.55) / 2 ≈ 0.476
SPRING_CENTER_Z = 0.476

# Spring compression (prismatic joint)
SPRING_COMPRESS = 0.030    # max compression travel

# Foot pads
FOOT_PAD_R = 0.045
FOOT_PAD_T = 0.016


def _beam_z(x: float) -> float:
    """Beam centerline height (relative to the pivot frame) at station x."""
    return BEAM_CENTER_Z + CURVE_C * x * x


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="playground_seesaw_aframe")

    model.material("gloss_red_orange", rgba=(0.88, 0.20, 0.06, 1.0))
    model.material("matte_black", rgba=(0.09, 0.09, 0.10, 1.0))
    model.material("light_gray", rgba=(0.78, 0.78, 0.79, 1.0))
    model.material("dark_gray_steel", rgba=(0.34, 0.36, 0.38, 1.0))
    model.material("silver_rivet", rgba=(0.74, 0.75, 0.78, 1.0))
    model.material("spring_steel", rgba=(0.52, 0.54, 0.56, 1.0))
    model.material("yellow_safety", rgba=(0.95, 0.82, 0.10, 1.0))

    # -----------------------------------------------------------------
    # Fixed base: A-frame support with legs, crossbar, brackets, axle caps.
    # -----------------------------------------------------------------
    base = model.part("aframe_support")

    # Top crossbar connecting the two legs (wide enough in Y to reach legs)
    base.visual(
        Box((CROSSBAR_W, CROSSBAR_D, CROSSBAR_H)),
        origin=Origin(xyz=(0.0, 0.0, CROSSBAR_CZ)),
        material="light_gray",
        name="crossbar",
    )

    # A-frame legs: two angled tubes from crossbar down to ground feet.
    # Legs are extended to clearly penetrate the crossbar at top and overlap
    # the foot pads at bottom, ensuring geometric connectivity.
    for i, sy in enumerate((1.0, -1.0)):
        # Compute leg geometry - extend slightly beyond endpoints for overlap
        dy = sy * (AFRAME_LEG_BOT_Y - AFRAME_LEG_TOP_Y)
        dz = AFRAME_LEG_BOT_Z - AFRAME_LEG_TOP_Z
        base_len = math.sqrt(dy * dy + dz * dz)
        leg_len = base_len + 0.05  # extend for overlap at both ends
        mid_y = 0.5 * (sy * AFRAME_LEG_TOP_Y + sy * AFRAME_LEG_BOT_Y)
        mid_z = 0.5 * (AFRAME_LEG_TOP_Z + AFRAME_LEG_BOT_Z)
        leg_angle = math.atan2(abs(dy), abs(dz))
        roll = -sy * leg_angle

        base.visual(
            Cylinder(radius=AFRAME_LEG_R, length=leg_len),
            origin=Origin(xyz=(0.0, mid_y, mid_z), rpy=(roll, 0.0, 0.0)),
            material="light_gray",
            name=f"aframe_leg_{i}",
        )
        # Gusset block at leg-crossbar junction for visual reinforcement
        base.visual(
            Box((0.070, 0.060, 0.060)),
            origin=Origin(xyz=(0.0, sy * (CROSSBAR_D / 2.0 - 0.010), CROSSBAR_CZ)),
            material="light_gray",
            name=f"leg_gusset_{i}",
        )
        # Foot pad: tall cylinder enveloping the leg bottom for connectivity
        base.visual(
            Cylinder(radius=FOOT_PAD_R, length=0.050),
            origin=Origin(xyz=(0.0, sy * AFRAME_LEG_BOT_Y, AFRAME_LEG_BOT_Z + 0.010)),
            material="matte_black",
            name=f"foot_pad_{i}",
        )

    # Bracket plates (two vertical plates on each Y side, holding the axle)
    for i, sy in enumerate((1.0, -1.0)):
        base.visual(
            Box((BRACKET_PLATE_W, BRACKET_PLATE_T, BRACKET_PLATE_H)),
            origin=Origin(xyz=(0.0, sy * BRACKET_PLATE_Y, BRACKET_PLATE_CZ)),
            material="matte_black",
            name=f"bracket_plate_{i}",
        )
        # Bolt heads on each bracket plate
        for j, (bx, bz) in enumerate([
            (-0.035, 0.04), (0.035, 0.04), (-0.035, -0.04), (0.035, -0.04)
        ]):
            base.visual(
                Cylinder(radius=0.007, length=0.008),
                origin=Origin(
                    xyz=(bx, sy * (BRACKET_PLATE_Y + sy * 0.008), BRACKET_PLATE_CZ + bz),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material="silver_rivet",
                name=f"bracket_bolt_{i}_{j}",
            )

    # Axle (horizontal cylinder along Y, between bracket plates)
    base.visual(
        Cylinder(radius=AXLE_R, length=2.0 * AXLE_HALF_LEN),
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="dark_gray_steel",
        name="axle_shaft",
    )

    # Axle caps: visible discs on outer face of each bracket plate
    for i, sy in enumerate((1.0, -1.0)):
        cap_y = sy * (BRACKET_PLATE_Y + BRACKET_PLATE_T / 2.0 + AXLE_CAP_T / 2.0 + 0.001)
        base.visual(
            Cylinder(radius=AXLE_CAP_R, length=AXLE_CAP_T),
            origin=Origin(xyz=(0.0, cap_y, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="dark_gray_steel",
            name=f"axle_cap_{i}",
        )
        # Cap center bolt
        base.visual(
            Cylinder(radius=0.009, length=0.006),
            origin=Origin(
                xyz=(0.0, sy * (BRACKET_PLATE_Y + BRACKET_PLATE_T / 2.0 + AXLE_CAP_T + 0.004), PIVOT_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material="silver_rivet",
            name=f"axle_cap_bolt_{i}",
        )

    # -----------------------------------------------------------------
    # Rocker: curved red beam + pivot stub + mirrored seat/handle ends.
    # Part frame sits on the pivot axis.
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

    # Short red stub descending from the beam into the bracket area.
    # Positioned so it doesn't extend below the crossbar top.
    # Local z=0 is at PIVOT_Z=0.45 world.
    # Stub center local z=0.05, length 0.10 → world z 0.45 to 0.55.
    rocker.visual(
        Cylinder(radius=0.045, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material="gloss_red_orange",
        name="pivot_stub",
    )

    # Seat profile (rounded triangle-ish shape for molded seats)
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
    # Inner profile for raised lip (shrunk ~84%)
    lip_inner = [(x * 0.84, y * 0.84) for x, y in seat_profile]
    lip_height = 0.020  # raised lip height above seat surface

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

        # Molded seat: thicker base plate + raised lip ring around perimeter.
        seat = ExtrudeGeometry(seat_profile, PLATE_T, cap=True, center=True)
        if s < 0:
            seat.rotate_z(math.pi)
        seat.translate(s * SEAT_CENTER_X, 0.0, SEAT_Z)
        rocker.visual(
            mesh_from_geometry(seat, f"seat_base_{i}"),
            material="dark_gray_steel",
            name=f"seat_base_{i}",
        )

        # Raised lip: ring extrusion around seat perimeter, sitting on top of base.
        lip_ring = ExtrudeWithHolesGeometry(
            seat_profile, [lip_inner], lip_height, cap=True, center=True
        )
        if s < 0:
            lip_ring.rotate_z(math.pi)
        lip_ring.translate(s * SEAT_CENTER_X, 0.0, SEAT_Z + PLATE_T / 2.0 + lip_height / 2.0)
        rocker.visual(
            mesh_from_geometry(lip_ring, f"seat_lip_{i}"),
            material="matte_black",
            name=f"seat_lip_{i}",
        )

        # Rivets on seat base underside
        rivet_xy = [(0.13, 0.0), (0.0, 0.10), (0.0, -0.10), (-0.13, 0.075), (-0.13, -0.075)]
        for j, (lx, ly) in enumerate(rivet_xy):
            rocker.visual(
                Cylinder(radius=0.008, length=0.010),
                origin=Origin(xyz=(s * (SEAT_CENTER_X + lx), ly, SEAT_Z - PLATE_T / 2.0 - 0.004)),
                material="silver_rivet",
                name=f"seat_rivet_{i}_{j}",
            )

        # Small black stop fin under the seat nose.
        rocker.visual(
            Box((0.045, 0.022, 0.04)),
            origin=Origin(xyz=(s * 1.26, 0.0, 0.038)),
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
    # Spring: helical compression spring between crossbar and beam.
    # Mounted on the A-frame with a prismatic joint for vertical compression.
    # The spring wraps around the pivot area.
    # -----------------------------------------------------------------
    spring = model.part("compression_spring")

    # Build a helical tube (single connected mesh for the entire spring).
    n_pts_per_turn = 16
    total_pts = SPRING_COILS * n_pts_per_turn + 1
    helix_points = []
    for k in range(total_pts):
        t = k / (total_pts - 1)
        angle = 2.0 * math.pi * SPRING_COILS * t
        z = -SPRING_HEIGHT / 2.0 + SPRING_HEIGHT * t
        x = SPRING_R * math.cos(angle)
        y = SPRING_R * math.sin(angle)
        helix_points.append((x, y, z))

    spring.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                helix_points,
                radius=SPRING_TUBE_R,
                samples_per_segment=4,
                radial_segments=12,
                cap_ends=True,
            ),
            "spring_helix",
        ),
        material="spring_steel",
        name="spring_helix",
    )

    # Top and bottom spring end plates (flat discs that contact beam/crossbar)
    for ci, z_off in enumerate((-SPRING_HEIGHT / 2.0 - 0.003, SPRING_HEIGHT / 2.0 + 0.003)):
        spring.visual(
            Cylinder(radius=SPRING_R + 0.006, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, z_off)),
            material="matte_black",
            name=f"spring_endplate_{ci}",
        )

    # -----------------------------------------------------------------
    # Articulations
    # -----------------------------------------------------------------

    # Rocking pivot: horizontal axis across the seesaw width (Y axis).
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

    # Spring compression: prismatic joint, vertical axis.
    # At q=0 spring is at rest (upper limit). Negative q compresses downward.
    model.articulation(
        "spring_compress",
        ArticulationType.PRISMATIC,
        parent=base,
        child=spring,
        origin=Origin(xyz=(0.0, 0.0, SPRING_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=800.0, velocity=0.5, lower=-SPRING_COMPRESS, upper=0.0
        ),
    )

    return model


def _intersects(a, b, tol: float = 1e-4) -> bool:
    if a is None or b is None:
        return False
    return all(a[0][i] <= b[1][i] + tol and b[0][i] <= a[1][i] + tol for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("aframe_support")
    rocker = object_model.get_part("rocker")
    spring = object_model.get_part("compression_spring")
    pivot = object_model.get_articulation("rocker_pivot")
    spring_joint = object_model.get_articulation("spring_compress")

    # --- Overlap allowances ---

    # The pivot stub descends into the bracket assembly (intentional nesting).
    ctx.allow_overlap(
        rocker,
        base,
        elem_a="pivot_stub",
        elem_b="bracket_plate_0",
        reason="The red center stub descends between the bracket plates that capture the rocking axle.",
    )
    ctx.allow_overlap(
        rocker,
        base,
        elem_a="pivot_stub",
        elem_b="bracket_plate_1",
        reason="The red center stub descends between the bracket plates that capture the rocking axle.",
    )
    ctx.allow_overlap(
        rocker,
        base,
        elem_a="pivot_stub",
        elem_b="axle_shaft",
        reason="The pivot stub and axle shaft both pass through the bracket assembly at the pivot center.",
    )

    # The spring wraps around the pivot stub between crossbar and beam.
    ctx.allow_overlap(
        spring,
        rocker,
        elem_a="spring_helix",
        elem_b="pivot_stub",
        reason="The compression spring encircles the pivot stub between the crossbar and beam underside.",
    )
    ctx.allow_overlap(
        spring,
        rocker,
        elem_a="spring_endplate_1",
        elem_b="pivot_wedge",
        reason="The top spring endplate seats against the beam underside flare where the stub meets the beam.",
    )
    ctx.allow_overlap(
        spring,
        rocker,
        elem_a="spring_endplate_1",
        elem_b="pivot_stub",
        reason="The top spring endplate contacts the pivot stub where it descends from the beam center.",
    )
    ctx.allow_overlap(
        spring,
        base,
        elem_a="spring_helix",
        elem_b="axle_shaft",
        reason="The spring helix passes around the axle region at the pivot center.",
    )

    # --- A-frame support structure ---
    leg0 = ctx.part_element_world_aabb(base, elem="aframe_leg_0")
    leg1 = ctx.part_element_world_aabb(base, elem="aframe_leg_1")
    crossbar = ctx.part_element_world_aabb(base, elem="crossbar")
    ctx.check(
        "A-frame has two legs",
        leg0 is not None and leg1 is not None,
        details=f"leg0={leg0}, leg1={leg1}",
    )
    ctx.check(
        "A-frame legs spread outward from crossbar",
        leg0 is not None and leg1 is not None
        and (leg0[1][1] - leg0[0][1]) > 0.15
        and (leg1[1][1] - leg1[0][1]) > 0.15,
        details=f"leg0 y-span, leg1 y-span",
    )
    ctx.check(
        "crossbar spans between the two legs",
        crossbar is not None and leg0 is not None and leg1 is not None
        and crossbar[1][1] > leg0[0][1]
        and crossbar[0][1] < leg1[1][1],
        details=f"crossbar={crossbar}",
    )

    # --- Axle brackets and caps ---
    bp0 = ctx.part_element_world_aabb(base, elem="bracket_plate_0")
    bp1 = ctx.part_element_world_aabb(base, elem="bracket_plate_1")
    ac0 = ctx.part_element_world_aabb(base, elem="axle_cap_0")
    ac1 = ctx.part_element_world_aabb(base, elem="axle_cap_1")
    ctx.check(
        "bracket plates exist on both sides of the A-frame top",
        bp0 is not None and bp1 is not None
        and bp0[0][1] > 0.0 and bp1[1][1] < 0.0,
        details=f"bp0={bp0}, bp1={bp1}",
    )
    ctx.check(
        "axle caps visible at outer faces of bracket plates",
        ac0 is not None and ac1 is not None
        and bp0 is not None and bp1 is not None
        and ac0[0][1] > bp0[0][1] and ac1[1][1] < bp1[1][1],
        details=f"ac0={ac0}, ac1={ac1}, bp0={bp0}, bp1={bp1}",
    )

    # --- Spring and prismatic joint ---
    spring_aabb = ctx.part_world_aabb(spring)
    helix_aabb = ctx.part_element_world_aabb(spring, elem="spring_helix")
    ctx.check(
        "compression spring helix exists between crossbar and beam",
        helix_aabb is not None
        and (helix_aabb[1][2] - helix_aabb[0][2]) > 0.08,
        details=f"spring_helix={helix_aabb}",
    )

    # Spring joint is prismatic with vertical axis
    sj_lim = spring_joint.motion_limits
    ctx.check(
        "spring has prismatic joint with compression travel",
        sj_lim is not None
        and sj_lim.lower is not None and sj_lim.upper is not None
        and sj_lim.lower < 0.0 and sj_lim.upper >= 0.0
        and abs(sj_lim.lower) >= 0.02,
        details=f"spring limits=({sj_lim.lower}, {sj_lim.upper})",
    )

    # Verify spring compresses downward at negative pose
    spring_rest_z = ctx.part_world_position(spring)
    with ctx.pose({spring_joint: sj_lim.lower}):
        spring_compressed_z = ctx.part_world_position(spring)
        ctx.check(
            "spring compresses downward at negative prismatic pose",
            spring_rest_z is not None and spring_compressed_z is not None
            and spring_compressed_z[2] < spring_rest_z[2] - 0.01,
            details=f"rest={spring_rest_z}, compressed={spring_compressed_z}",
        )

    # --- Pivot stub inserted between bracket plates ---
    ctx.expect_overlap(
        rocker,
        base,
        axes="z",
        elem_a="pivot_stub",
        elem_b="bracket_plate_0",
        min_overlap=0.02,
        name="pivot stub inserted between bracket plates",
    )

    # --- Molded seats with raised lips ---
    for i in range(2):
        seat_base = ctx.part_element_world_aabb(rocker, elem=f"seat_base_{i}")
        seat_lip = ctx.part_element_world_aabb(rocker, elem=f"seat_lip_{i}")
        ctx.check(
            f"seat_{i} has molded base plate",
            seat_base is not None and (seat_base[1][2] - seat_base[0][2]) >= 0.010,
            details=f"seat_base_{i}={seat_base}",
        )
        ctx.check(
            f"seat_{i} has raised lip above the base",
            seat_lip is not None and seat_base is not None
            and seat_lip[0][2] >= seat_base[1][2] - 0.005
            and (seat_lip[1][2] - seat_lip[0][2]) >= 0.012,
            details=f"seat_lip_{i}={seat_lip}, seat_base_{i}={seat_base}",
        )

    # --- Hero beam: ~2.6 m long banana tube ---
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

    # --- Overall envelope ---
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

    # --- Seats at sitting height, grips above beam ---
    seat0 = ctx.part_element_world_aabb(rocker, elem="seat_base_0")
    seat1 = ctx.part_element_world_aabb(rocker, elem="seat_base_1")
    grip0 = ctx.part_element_world_aabb(rocker, elem="handle_plate_0")
    grip1 = ctx.part_element_world_aabb(rocker, elem="handle_plate_1")
    ctx.check(
        "seats at sitting height below the beam",
        seat0 is not None
        and seat1 is not None
        and 0.35 <= seat0[1][2] <= 0.58
        and 0.35 <= seat1[1][2] <= 0.58,
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

    # --- Mirrored ends ---
    def _cx(aabb):
        return 0.5 * (aabb[0][0] + aabb[1][0])

    ctx.check(
        "seat assemblies mirrored about the pivot",
        seat0 is not None
        and seat1 is not None
        and _cx(seat0) > 0.9
        and _cx(seat1) < -0.9
        and abs(_cx(seat0) + _cx(seat1)) < 0.02
        and abs(seat0[1][2] - seat1[1][2]) < 0.01,
        details=f"seat0={seat0}, seat1={seat1}",
    )

    # --- Mounted geometry: drop tubes reach seats, posts reach grips ---
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

    # --- Rocking joint: about +/- 15 degrees ---
    lim = pivot.motion_limits
    ctx.check(
        "rocking range about +/- 15 degrees",
        lim is not None
        and abs(lim.lower + ROCK_LIMIT) < 0.02
        and abs(lim.upper - ROCK_LIMIT) < 0.02,
        details=f"limits=({lim.lower}, {lim.upper})",
    )

    # --- Decisive pose checks ---
    base_rest = ctx.part_world_aabb(base)
    with ctx.pose({pivot: ROCK_LIMIT}):
        seat0_dn = ctx.part_element_world_aabb(rocker, elem="seat_base_0")
        seat1_up = ctx.part_element_world_aabb(rocker, elem="seat_base_1")
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
            "A-frame stays fixed while rocking",
            base_rest is not None and base_posed is not None and _intersects(base_rest, base_posed)
            and abs(base_rest[1][2] - base_posed[1][2]) < 1e-6,
            details=f"rest={base_rest}, posed={base_posed}",
        )
    with ctx.pose({pivot: -ROCK_LIMIT}):
        seat0_up = ctx.part_element_world_aabb(rocker, elem="seat_base_0")
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
