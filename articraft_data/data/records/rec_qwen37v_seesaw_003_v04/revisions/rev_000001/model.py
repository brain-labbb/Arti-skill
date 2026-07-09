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
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sample_catmull_rom_spline_2d,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Shared dimensions (meters). World: X along the seesaw length, Z up.
# Toddler-scale playground horse seesaw (~1.8 m long, ~0.70 m tall).
# ---------------------------------------------------------------------------
PIVOT_Z = 0.28          # world height of the rocking axis
BODY_R = 0.065          # main body tube radius (~130 mm diameter)
BODY_HALF = 0.78        # half-length of the curved body beam
CURVE_C = 0.10          # parabolic curvature (banana shape)
BODY_CENTER_Z = 0.12    # body centerline height at x=0, relative to pivot

SEAT_X = 0.62           # seat position along the body
SEAT_Z_OFFSET = 0.09    # seat above body surface
PLATE_T = 0.010         # seat plate thickness

HANDLE_X = 0.54         # handle position
HANDLE_Z_OFFSET = 0.30  # handle height above pivot

ROCK_LIMIT = 0.262      # ~15 degrees each way

PEDESTAL_R = 0.065      # ground pedestal radius
PEDESTAL_H = 0.18       # pedestal height
BRACKET_SIZE = (0.13, 0.11, 0.14)
BRACKET_CZ = 0.245      # bracket box center height

# Ground pad dimensions
PAD_SIZE = (0.14, 0.14, 0.018)
PAD_OFFSET_R = 0.09     # radial offset from pedestal center

# Bump stop dimensions
BUMP_R = 0.035          # bump stop hemisphere radius
BUMP_X = 0.74           # bump stop X position (near beam ends)


def _body_z(x: float) -> float:
    """Body centerline height (relative to pivot frame) at station x."""
    return BODY_CENTER_Z + CURVE_C * x * x


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toddler_horse_seesaw")

    model.material("body_green", rgba=(0.18, 0.62, 0.30, 1.0))
    model.material("body_green_dark", rgba=(0.12, 0.45, 0.22, 1.0))
    model.material("matte_black", rgba=(0.09, 0.09, 0.10, 1.0))
    model.material("light_gray", rgba=(0.78, 0.78, 0.79, 1.0))
    model.material("dark_gray", rgba=(0.34, 0.36, 0.38, 1.0))
    model.material("rubber_black", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("silver_bolt", rgba=(0.74, 0.75, 0.78, 1.0))
    model.material("horse_head", rgba=(0.22, 0.68, 0.34, 1.0))
    model.material("eye_white", rgba=(0.92, 0.92, 0.90, 1.0))
    model.material("eye_pupil", rgba=(0.06, 0.06, 0.08, 1.0))
    model.material("seat_plate", rgba=(0.28, 0.30, 0.33, 1.0))
    model.material("handle_grip", rgba=(0.55, 0.56, 0.58, 1.0))
    model.material("bump_rubber", rgba=(0.15, 0.14, 0.13, 1.0))
    model.material("mane_gold", rgba=(0.82, 0.62, 0.18, 1.0))

    # -----------------------------------------------------------------
    # Fixed base: pedestal + bracket + rubber ground pads.
    # -----------------------------------------------------------------
    base = model.part("base_mount")

    # Ground pedestal
    base.visual(
        Cylinder(radius=PEDESTAL_R, length=PEDESTAL_H),
        origin=Origin(xyz=(0.0, 0.0, PEDESTAL_H / 2.0)),
        material="light_gray",
        name="ground_pedestal",
    )

    # Pivot bracket
    base.visual(
        Box(BRACKET_SIZE),
        origin=Origin(xyz=(0.0, 0.0, BRACKET_CZ)),
        material="matte_black",
        name="pivot_bracket",
    )

    # Pivot bosses with bolt heads
    for i, sy in enumerate((1.0, -1.0)):
        base.visual(
            Cylinder(radius=0.044, length=0.018),
            origin=Origin(xyz=(0.0, sy * 0.064, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="matte_black",
            name=f"pivot_boss_{i}",
        )
        for j, ang in enumerate((0.25, 0.75, 1.25, 1.75)):
            dx = 0.028 * math.cos(ang * math.pi)
            dz = 0.028 * math.sin(ang * math.pi)
            base.visual(
                Cylinder(radius=0.007, length=0.010),
                origin=Origin(
                    xyz=(dx, sy * 0.076, PIVOT_Z + dz),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material="silver_bolt",
                name=f"bracket_bolt_{i}_{j}",
            )

    # Rubber ground pads under the pedestal (4 pads at cardinal offsets)
    pad_angles = [0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0]
    for i, ang in enumerate(pad_angles):
        px = PAD_OFFSET_R * math.cos(ang)
        py = PAD_OFFSET_R * math.sin(ang)
        base.visual(
            Box(PAD_SIZE),
            origin=Origin(xyz=(px, py, PAD_SIZE[2] / 2.0)),
            material="rubber_black",
            name=f"ground_pad_{i}",
        )

    # -----------------------------------------------------------------
    # Rocker: horse-shaped body beam + pivot stub + seats + handles +
    # bump stops + horse head/tail/legs.
    # Part frame at the pivot axis; geometry relative to that frame.
    # -----------------------------------------------------------------
    rocker = model.part("rocker")

    # Main body: curved green tube (the horse's back/beam)
    n = 12
    body_pts = []
    for k in range(-n, n + 1):
        x = BODY_HALF * k / n
        body_pts.append((x, 0.0, _body_z(x)))
    rocker.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                body_pts,
                radius=BODY_R,
                samples_per_segment=4,
                radial_segments=28,
                cap_ends=True,
            ),
            "body_tube",
        ),
        material="body_green",
        name="body_tube",
    )

    # Pivot stub descending from body center into bracket
    rocker.visual(
        Cylinder(radius=0.040, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material="body_green",
        name="pivot_stub",
    )

    # Transition wedge from stub to body
    wedge = ConeGeometry(0.072, 0.07, radial_segments=28).rotate_x(math.pi)
    wedge.translate(0.0, 0.0, 0.090)
    rocker.visual(
        mesh_from_geometry(wedge, "pivot_wedge"),
        material="body_green",
        name="pivot_wedge",
    )

    # --- HORSE HEAD (at +X end) ---
    head_x = BODY_HALF + 0.02
    head_z = _body_z(BODY_HALF) + 0.08

    # Head sphere
    head = SphereGeometry(0.085, width_segments=20, height_segments=14)
    head.translate(head_x, 0.0, head_z)
    rocker.visual(
        mesh_from_geometry(head, "horse_head_sphere"),
        material="horse_head",
        name="horse_head_sphere",
    )

    # Snout (elongated sphere/cone pointing forward and slightly down)
    snout = CylinderGeometry(0.045, 0.10, radial_segments=18)
    snout.rotate_y(math.pi / 2.0)
    snout.translate(head_x + 0.08, 0.0, head_z - 0.02)
    rocker.visual(
        mesh_from_geometry(snout, "horse_snout"),
        material="horse_head",
        name="horse_snout",
    )

    # Ears (two small cones on top of head)
    for i, sy in enumerate((1.0, -1.0)):
        ear = ConeGeometry(0.020, 0.055, radial_segments=12)
        ear.translate(head_x - 0.01, sy * 0.045, head_z + 0.09)
        rocker.visual(
            mesh_from_geometry(ear, f"horse_ear_{i}"),
            material="body_green_dark",
            name=f"horse_ear_{i}",
        )

    # Eyes (small spheres on each side of head)
    for i, sy in enumerate((1.0, -1.0)):
        # White of eye
        eye_white = SphereGeometry(0.018, width_segments=10, height_segments=8)
        eye_white.translate(head_x + 0.04, sy * 0.072, head_z + 0.02)
        rocker.visual(
            mesh_from_geometry(eye_white, f"eye_white_{i}"),
            material="eye_white",
            name=f"eye_white_{i}",
        )
        # Pupil
        pupil = SphereGeometry(0.010, width_segments=8, height_segments=6)
        pupil.translate(head_x + 0.055, sy * 0.078, head_z + 0.02)
        rocker.visual(
            mesh_from_geometry(pupil, f"eye_pupil_{i}"),
            material="eye_pupil",
            name=f"eye_pupil_{i}",
        )

    # Mane (gold-colored bumps along the top of the head/neck)
    for j in range(4):
        mx = head_x - 0.06 - j * 0.06
        mz = _body_z(mx - 0.06) + BODY_R + 0.02
        mane_bump = SphereGeometry(0.025, width_segments=8, height_segments=6)
        mane_bump.scale(1.0, 0.6, 1.0)
        mane_bump.translate(mx, 0.0, mz)
        rocker.visual(
            mesh_from_geometry(mane_bump, f"mane_{j}"),
            material="mane_gold",
            name=f"mane_{j}",
        )

    # --- HORSE TAIL (at -X end) ---
    tail_x = -BODY_HALF - 0.02
    tail_z = _body_z(-BODY_HALF) + 0.05
    tail_pts = [
        (tail_x + 0.04, 0.0, tail_z),
        (tail_x - 0.02, 0.0, tail_z + 0.06),
        (tail_x - 0.08, 0.0, tail_z + 0.02),
        (tail_x - 0.12, 0.0, tail_z - 0.06),
    ]
    rocker.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                tail_pts, radius=0.022, samples_per_segment=8, radial_segments=14
            ),
            "horse_tail",
        ),
        material="mane_gold",
        name="horse_tail",
    )
    # Tail tuft
    tuft = SphereGeometry(0.030, width_segments=10, height_segments=8)
    tuft.scale(1.0, 0.7, 1.2)
    tuft.translate(tail_x - 0.12, 0.0, tail_z - 0.06)
    rocker.visual(
        mesh_from_geometry(tuft, "tail_tuft"),
        material="mane_gold",
        name="tail_tuft",
    )

    # --- DECORATIVE LEGS (4 short cylinders, 2 front, 2 rear) ---
    leg_positions = [
        (0.48, 0.06),   # front pair
        (0.48, -0.06),
        (-0.48, 0.06),  # rear pair
        (-0.48, -0.06),
    ]
    for i, (lx, ly) in enumerate(leg_positions):
        lz_center = _body_z(lx) - BODY_R * 0.3  # start inside body tube
        leg_len = 0.13
        leg = CylinderGeometry(0.022, leg_len, radial_segments=14)
        leg.translate(lx, ly, lz_center - leg_len / 2.0 + 0.02)
        rocker.visual(
            mesh_from_geometry(leg, f"deco_leg_{i}"),
            material="body_green_dark",
            name=f"deco_leg_{i}",
        )
        # Hoof (small dark cylinder at bottom, connected to leg)
        hoof = CylinderGeometry(0.026, 0.02, radial_segments=14)
        hoof.translate(lx, ly, lz_center - leg_len / 2.0 + 0.02 - leg_len / 2.0 - 0.008)
        rocker.visual(
            mesh_from_geometry(hoof, f"deco_hoof_{i}"),
            material="matte_black",
            name=f"deco_hoof_{i}",
        )

    # --- SEATS (two rounded-rect plates on the horse's back) ---
    seat_profile = rounded_rect_profile(0.20, 0.18, 0.04)
    for i, s in enumerate((1.0, -1.0)):
        sx = s * SEAT_X
        sz = _body_z(sx) + BODY_R + SEAT_Z_OFFSET
        seat = ExtrudeGeometry(seat_profile, PLATE_T, cap=True, center=True)
        seat.translate(sx, 0.0, sz)
        rocker.visual(
            mesh_from_geometry(seat, f"seat_plate_{i}"),
            material="seat_plate",
            name=f"seat_plate_{i}",
        )
        # Seat support bracket (short box connecting seat to body)
        bracket_h = SEAT_Z_OFFSET - 0.005
        rocker.visual(
            Box((0.06, 0.06, bracket_h)),
            origin=Origin(xyz=(sx, 0.0, sz - PLATE_T / 2.0 - bracket_h / 2.0)),
            material="dark_gray",
            name=f"seat_bracket_{i}",
        )

    # --- HANDLES (grip bars in front of each seat) ---
    for i, s in enumerate((1.0, -1.0)):
        hx = s * HANDLE_X
        hz = _body_z(hx) + HANDLE_Z_OFFSET
        # Vertical post
        post_h = HANDLE_Z_OFFSET - BODY_R + 0.02
        post_base_z = _body_z(hx) + BODY_R
        rocker.visual(
            Cylinder(radius=0.014, length=post_h),
            origin=Origin(xyz=(hx, 0.0, post_base_z + post_h / 2.0)),
            material="dark_gray",
            name=f"handle_post_{i}",
        )
        # Horizontal grip bar
        rocker.visual(
            Cylinder(radius=0.012, length=0.14),
            origin=Origin(xyz=(hx, 0.0, hz), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="handle_grip",
            name=f"handle_grip_{i}",
        )
        # Grip end caps
        for j, sy in enumerate((1.0, -1.0)):
            cap = SphereGeometry(0.015, width_segments=8, height_segments=6)
            cap.translate(hx, sy * 0.072, hz)
            rocker.visual(
                mesh_from_geometry(cap, f"grip_cap_{i}_{j}"),
                material="matte_black",
                name=f"grip_cap_{i}_{j}",
            )

    # --- SAFETY BUMP STOPS (rubber hemispheres below each beam end) ---
    for i, s in enumerate((1.0, -1.0)):
        bx = s * BUMP_X
        bz = _body_z(bx) - BODY_R
        # Main bump stop body (flattened sphere)
        bump = SphereGeometry(BUMP_R, width_segments=14, height_segments=10)
        bump.scale(1.0, 1.0, 0.6)
        bump.translate(bx, 0.0, bz - BUMP_R * 0.6)
        rocker.visual(
            mesh_from_geometry(bump, f"bump_stop_{i}"),
            material="bump_rubber",
            name=f"bump_stop_{i}",
        )
        # Bump stop mounting plate (thin disk above the rubber)
        rocker.visual(
            Cylinder(radius=BUMP_R * 0.85, length=0.006),
            origin=Origin(xyz=(bx, 0.0, bz - 0.003)),
            material="dark_gray",
            name=f"bump_plate_{i}",
        )

    # Single rocking pivot: horizontal axis across the seesaw width (Y).
    model.articulation(
        "rocker_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=rocker,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=200.0, velocity=1.5, lower=-ROCK_LIMIT, upper=ROCK_LIMIT
        ),
    )

    return model


def _intersects(a, b, tol: float = 1e-4) -> bool:
    if a is None or b is None:
        return False
    return all(a[0][i] <= b[1][i] + tol and b[0][i] <= a[1][i] + tol for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_mount")
    rocker = object_model.get_part("rocker")
    pivot = object_model.get_articulation("rocker_pivot")

    # --- Pivot stub captured inside bracket (intentional overlap) ---
    ctx.allow_overlap(
        rocker,
        base,
        elem_a="pivot_stub",
        elem_b="pivot_bracket",
        reason="The center pivot stub descends into the cast bracket that captures the rocking axle.",
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

    # --- Bracket seated on pedestal ---
    bracket = ctx.part_element_world_aabb(base, elem="pivot_bracket")
    pedestal = ctx.part_element_world_aabb(base, elem="ground_pedestal")
    ctx.check(
        "bracket sits atop ground pedestal",
        _intersects(bracket, pedestal),
        details=f"bracket={bracket}, pedestal={pedestal}",
    )

    # --- Rubber ground pads exist and are below pedestal ---
    for i in range(4):
        pad = ctx.part_element_world_aabb(base, elem=f"ground_pad_{i}")
        ctx.check(
            f"ground pad {i} at ground level",
            pad is not None and pad[0][2] >= -0.002 and pad[1][2] <= 0.025,
            details=f"pad_{i}={pad}",
        )

    # --- Body tube spans the seesaw length ---
    body = ctx.part_element_world_aabb(rocker, elem="body_tube")
    ctx.check(
        "body tube spans the seesaw length",
        body is not None and (body[1][0] - body[0][0]) >= 1.4,
        details=f"body={body}",
    )

    # --- Overall envelope: toddler scale ~1.8 m long, ~0.70 m tall ---
    ra = ctx.part_world_aabb(rocker)
    ba = ctx.part_world_aabb(base)
    ctx.check(
        "overall length appropriate for toddler seesaw",
        ra is not None and 1.5 <= (ra[1][0] - ra[0][0]) <= 2.1,
        details=f"rocker aabb={ra}",
    )
    ctx.check(
        "overall height appropriate for toddler seesaw",
        ra is not None and ba is not None and 0.50 <= max(ra[1][2], ba[1][2]) <= 0.80,
        details=f"rocker={ra}, base={ba}",
    )

    # --- Horse head features exist at one end ---
    head = ctx.part_element_world_aabb(rocker, elem="horse_head_sphere")
    ctx.check(
        "horse head at one end of the seesaw",
        head is not None and head[0][0] > 0.6,
        details=f"head={head}",
    )

    # --- Horse tail at the other end ---
    tail = ctx.part_element_world_aabb(rocker, elem="horse_tail")
    ctx.check(
        "horse tail at opposite end",
        tail is not None and tail[1][0] < -0.6,
        details=f"tail={tail}",
    )

    # --- Safety bump stops below each beam end ---
    bump0 = ctx.part_element_world_aabb(rocker, elem="bump_stop_0")
    bump1 = ctx.part_element_world_aabb(rocker, elem="bump_stop_1")
    ctx.check(
        "bump stop 0 near beam end",
        bump0 is not None and bump0[0][0] > 0.5,
        details=f"bump0={bump0}",
    )
    ctx.check(
        "bump stop 1 near opposite beam end",
        bump1 is not None and bump1[1][0] < -0.5,
        details=f"bump1={bump1}",
    )
    # Bump stops are mounted on the underside of the body tube:
    # their bottom hangs below the body tube centerline.
    body_center_z = 0.5 * (body[0][2] + body[1][2]) if body else None
    ctx.check(
        "bump stops hang below body center",
        bump0 is not None and bump1 is not None and body_center_z is not None
        and 0.5 * (bump0[0][2] + bump0[1][2]) < body_center_z
        and 0.5 * (bump1[0][2] + bump1[1][2]) < body_center_z,
        details=f"bump0={bump0}, bump1={bump1}, body_center_z={body_center_z}",
    )

    # --- Seats at opposite ends ---
    seat0 = ctx.part_element_world_aabb(rocker, elem="seat_plate_0")
    seat1 = ctx.part_element_world_aabb(rocker, elem="seat_plate_1")
    ctx.check(
        "seats at opposite ends of the beam",
        seat0 is not None and seat1 is not None
        and seat0[0][0] > 0.4 and seat1[1][0] < -0.4,
        details=f"seat0={seat0}, seat1={seat1}",
    )

    # --- Joint limits: +/- 15 degrees ---
    lim = pivot.motion_limits
    ctx.check(
        "rocking range about +/- 15 degrees",
        lim is not None
        and abs(lim.lower + ROCK_LIMIT) < 0.02
        and abs(lim.upper - ROCK_LIMIT) < 0.02,
        details=f"limits=({lim.lower}, {lim.upper})",
    )

    # --- Revolute joint is non-fixed ---
    ctx.check(
        "rocker pivot is revolute (non-fixed)",
        pivot.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={pivot.articulation_type}",
    )

    # --- Decisive pose: rocker tilts, seats swap height ---
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
            and seat0_dn[1][2] < seat0[1][2] - 0.08
            and seat1_up[1][2] > seat1[1][2] + 0.08,
            details=f"seat0_dn={seat0_dn}, seat1_up={seat1_up}",
        )
        ctx.check(
            "rocker clears the ground at full tilt",
            rocker_dn is not None and rocker_dn[0][2] > 0.003,
            details=f"rocker={rocker_dn}",
        )
        ctx.check(
            "base stays fixed while rocking",
            base_rest is not None and base_posed is not None
            and abs(base_rest[1][2] - base_posed[1][2]) < 1e-6,
            details=f"rest={base_rest}, posed={base_posed}",
        )
        # Bump stop approaches ground on tilted side (lower than at rest)
        bump0_dn = ctx.part_element_world_aabb(rocker, elem="bump_stop_0")
        ctx.check(
            "bump stop descends toward ground at full tilt",
            bump0_dn is not None and bump0 is not None
            and bump0_dn[0][2] < bump0[0][2] - 0.10,
            details=f"bump0_rest={bump0}, bump0_dn={bump0_dn}",
        )

    with ctx.pose({pivot: -ROCK_LIMIT}):
        seat0_up = ctx.part_element_world_aabb(rocker, elem="seat_plate_0")
        rocker_up = ctx.part_world_aabb(rocker)
        ctx.check(
            "negative rock raises seat_0",
            seat0_up is not None and seat0 is not None
            and seat0_up[0][2] > seat0[0][2] + 0.08,
            details=f"seat0_up={seat0_up}",
        )
        ctx.check(
            "rocker clears the ground at opposite tilt",
            rocker_up is not None and rocker_up[0][2] > 0.003,
            details=f"rocker={rocker_up}",
        )

    return ctx.report()


object_model = build_object_model()
