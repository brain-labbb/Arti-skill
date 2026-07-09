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

PEDESTAL_R = 0.075
PEDESTAL_H = 0.22
BRACKET_SIZE = (0.16, 0.13, 0.17)
BRACKET_CZ = 0.295      # bracket box center height (spans 0.21 .. 0.38)

# Support leg dimensions
LEG_SPREAD = 0.22       # radial distance from center to foot
LEG_BAR_LEN = 0.20      # outrigger bar length
LEG_BAR_W = 0.035       # bar width
LEG_BAR_H = 0.025       # bar height
LEG_Z = 0.013           # bar center height (just above ground pad)
PAD_R = 0.048           # rubber ground pad radius
PAD_T = 0.012           # rubber ground pad thickness

# Bumper dimensions
BUMPER_R = 0.036        # rubber bumper radius
BUMPER_H = 0.050        # rubber bumper height
BUMPER_STEM_R = 0.016   # bumper mounting stem radius
BUMPER_STEM_H = 0.05    # bumper mounting stem height (longer to bridge gap)
BUMPER_COMPRESS = 0.025 # bumper compression travel (meters)
BUMPER_GAP = 0.018      # clearance below beam exterior


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
    model.material("dark_rubber", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("steel_gray", rgba=(0.50, 0.52, 0.54, 1.0))

    # -----------------------------------------------------------------
    # Fixed base: ground pedestal + black cast bracket + support legs
    # with rubber ground pads.
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

    # Support legs: 4 outrigger bars splayed diagonally from pedestal base,
    # each with a rubber ground pad underneath the foot.
    for i, angle_deg in enumerate((45, 135, 225, 315)):
        angle = math.radians(angle_deg)
        cx = LEG_SPREAD * 0.55 * math.cos(angle)
        cy = LEG_SPREAD * 0.55 * math.sin(angle)
        # Outrigger steel bar
        base.visual(
            Box((LEG_BAR_LEN, LEG_BAR_W, LEG_BAR_H)),
            origin=Origin(xyz=(cx, cy, LEG_Z), rpy=(0.0, 0.0, angle)),
            material="steel_gray",
            name=f"support_leg_{i}",
        )
        # Rubber ground pad under each foot
        fx = LEG_SPREAD * math.cos(angle)
        fy = LEG_SPREAD * math.sin(angle)
        base.visual(
            Cylinder(radius=PAD_R, length=PAD_T),
            origin=Origin(xyz=(fx, fy, PAD_T / 2.0)),
            material="dark_rubber",
            name=f"ground_pad_{i}",
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

        # Flat dark-gray rounded-triangular seat plate with rivets.
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
        # Small black stop fin under the seat nose (as in the reference photo).
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
    # Rubber end bumpers: one at each beam tip, compressing vertically
    # on short prismatic joints.
    # -----------------------------------------------------------------
    beam_end_z = _beam_z(BEAM_HALF)  # beam centerline z at tip in rocker frame
    # Position bumper origin below the beam exterior surface with clearance.
    # The bumper rubber hangs down from this point, so its top will be at bumper_mount_z.
    bumper_mount_z = beam_end_z - BEAM_R - BUMPER_GAP  # below beam underside with gap

    for i, s in enumerate((1.0, -1.0)):
        bumper = model.part(f"bumper_{i}")
        # Mounting stem (inserts into beam tube end)
        bumper.visual(
            Cylinder(radius=BUMPER_STEM_R, length=BUMPER_STEM_H),
            origin=Origin(xyz=(0.0, 0.0, BUMPER_STEM_H / 2.0)),
            material="steel_gray",
            name=f"bumper_stem_{i}",
        )
        # Main rubber bumper body (hangs below the beam end)
        bumper.visual(
            Cylinder(radius=BUMPER_R, length=BUMPER_H),
            origin=Origin(xyz=(0.0, 0.0, -BUMPER_H / 2.0)),
            material="dark_rubber",
            name=f"bumper_rubber_{i}",
        )

        # Prismatic joint: compression along local +Z (upward into beam)
        model.articulation(
            f"bumper_{i}_compress",
            ArticulationType.PRISMATIC,
            parent=rocker,
            child=bumper,
            origin=Origin(xyz=(s * BEAM_HALF, 0.0, bumper_mount_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=200.0, velocity=0.5, lower=0.0, upper=BUMPER_COMPRESS
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
    bump_j0 = object_model.get_articulation("bumper_0_compress")
    bump_j1 = object_model.get_articulation("bumper_1_compress")

    # --- Bumper stem insertion into beam (intentional overlap) ---
    ctx.allow_overlap(
        bumper_0,
        rocker,
        elem_a="bumper_stem_0",
        elem_b="beam_tube",
        reason="The bumper mounting stem inserts into the open end of the beam tube for retention.",
    )
    ctx.allow_overlap(
        bumper_1,
        rocker,
        elem_a="bumper_stem_1",
        elem_b="beam_tube",
        reason="The bumper mounting stem inserts into the open end of the beam tube for retention.",
    )

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

    # Bracket seated on the pedestal (both visuals on the fixed base part).
    bracket = ctx.part_element_world_aabb(base, elem="pivot_bracket")
    pedestal = ctx.part_element_world_aabb(base, elem="ground_pedestal")
    ctx.check(
        "bracket sits atop ground pedestal",
        _intersects(bracket, pedestal),
        details=f"bracket={bracket}, pedestal={pedestal}",
    )

    # --- Ground pads exist under support legs ---
    pad_names = [f"ground_pad_{i}" for i in range(4)]
    leg_names = [f"support_leg_{i}" for i in range(4)]
    pads = [ctx.part_element_world_aabb(base, elem=pn) for pn in pad_names]
    legs = [ctx.part_element_world_aabb(base, elem=ln) for ln in leg_names]

    ctx.check(
        "four rubber ground pads present under support legs",
        all(p is not None for p in pads),
        details=f"pads={pads}",
    )
    ctx.check(
        "four support legs present on base",
        all(l is not None for l in legs),
        details=f"legs={legs}",
    )
    # Ground pads sit at ground level (z near 0)
    ctx.check(
        "ground pads sit at ground level",
        all(p is not None and p[0][2] < 0.02 for p in pads),
        details=f"pads={pads}",
    )
    # Support legs connect pedestal to pads
    for i in range(4):
        ctx.check(
            f"support_leg_{i} bridges pedestal to ground_pad_{i}",
            legs[i] is not None and pads[i] is not None
            and _intersects(legs[i], pedestal)
            and _intersects(legs[i], pads[i]),
            details=f"leg={legs[i]}, pad={pads[i]}",
        )

    # --- Rubber end bumpers at beam tips ---
    rubber_0 = ctx.part_element_world_aabb(bumper_0, elem="bumper_rubber_0")
    rubber_1 = ctx.part_element_world_aabb(bumper_1, elem="bumper_rubber_1")
    beam = ctx.part_element_world_aabb(rocker, elem="beam_tube")

    ctx.check(
        "rubber bumpers present at both beam ends",
        rubber_0 is not None and rubber_1 is not None,
        details=f"rubber_0={rubber_0}, rubber_1={rubber_1}",
    )
    # Bumpers are at opposite ends of the beam
    ctx.check(
        "bumpers at opposite beam ends",
        rubber_0 is not None and rubber_1 is not None
        and rubber_0[1][0] > 0.8
        and rubber_1[0][0] < -0.8,
        details=f"rubber_0={rubber_0}, rubber_1={rubber_1}",
    )
    # Bumpers hang below the beam
    ctx.check(
        "bumpers hang below beam underside",
        rubber_0 is not None and rubber_1 is not None and beam is not None
        and rubber_0[0][2] < beam[0][2] + 0.20
        and rubber_1[0][2] < beam[0][2] + 0.20,
        details=f"rubber_0={rubber_0}, rubber_1={rubber_1}, beam={beam}",
    )

    # --- Prismatic joints for bumper compression ---
    ctx.check(
        "bumper_0 joint is prismatic with compression travel",
        bump_j0 is not None
        and bump_j0.articulation_type == ArticulationType.PRISMATIC
        and bump_j0.motion_limits is not None
        and bump_j0.motion_limits.lower == 0.0
        and abs(bump_j0.motion_limits.upper - BUMPER_COMPRESS) < 0.001,
        details=f"limits={bump_j0.motion_limits if bump_j0 else None}",
    )
    ctx.check(
        "bumper_1 joint is prismatic with compression travel",
        bump_j1 is not None
        and bump_j1.articulation_type == ArticulationType.PRISMATIC
        and bump_j1.motion_limits is not None
        and bump_j1.motion_limits.lower == 0.0
        and abs(bump_j1.motion_limits.upper - BUMPER_COMPRESS) < 0.001,
        details=f"limits={bump_j1.motion_limits if bump_j1 else None}",
    )

    # Bumper compression pose: bumpers move upward (compress)
    bumper_0_rest = ctx.part_world_position(bumper_0)
    with ctx.pose({bump_j0: BUMPER_COMPRESS}):
        bumper_0_compressed = ctx.part_world_position(bumper_0)
        ctx.check(
            "bumper_0 compresses upward when prismatic joint actuated",
            bumper_0_rest is not None and bumper_0_compressed is not None
            and bumper_0_compressed[2] > bumper_0_rest[2] + 0.01,
            details=f"rest={bumper_0_rest}, compressed={bumper_0_compressed}",
        )

    # Hero beam: ~2.6 m long banana tube that dips at center and rises at ends.
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

    # End assemblies: seats hang below the beam, grip plates rise above it.
    seat0 = ctx.part_element_world_aabb(rocker, elem="seat_plate_0")
    seat1 = ctx.part_element_world_aabb(rocker, elem="seat_plate_1")
    grip0 = ctx.part_element_world_aabb(rocker, elem="handle_plate_0")
    grip1 = ctx.part_element_world_aabb(rocker, elem="handle_plate_1")
    ctx.check(
        "seats at sitting height below the beam",
        seat0 is not None
        and seat1 is not None
        and 0.35 <= seat0[1][2] <= 0.46
        and 0.35 <= seat1[1][2] <= 0.46,
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

    # The two ends mirror each other across the pivot.
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
    ctx.check(
        "grip plates mirrored about the pivot",
        grip0 is not None and grip1 is not None and abs(_cx(grip0) + _cx(grip1)) < 0.02,
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
