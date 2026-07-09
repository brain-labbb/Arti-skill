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
    superellipse_side_loft,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Shared dimensions (meters). World: X along the seesaw length, Z up.
# ---------------------------------------------------------------------------
PIVOT_Z = 0.34          # world height of the rocking axis
ROCK_LIMIT = 0.262      # ~15 degrees each way
HANDLE_ROCK = 0.175     # ~10 degrees each way for handle pivot

PEDESTAL_R = 0.075
PEDESTAL_H = 0.22
BRACKET_SIZE = (0.16, 0.13, 0.17)
BRACKET_CZ = 0.295

# Seat and handle positions (in rocker frame, origin at pivot)
SEAT_X = 0.72
HANDLE_X = 0.82
HANDLE_MOUNT_Z = 0.30   # handle joint origin in rocker frame
HANDLE_STEM_H = 0.22
HANDLE_GRIP_LEN = 0.20

# Support legs
LEG_SPREAD = 0.26       # horizontal spread from pedestal center
PAD_R = 0.045
PAD_H = 0.012


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="animal_toddler_seesaw")

    # Materials
    model.material("gloss_orange", rgba=(0.85, 0.35, 0.08, 1.0))
    model.material("dark_brown", rgba=(0.30, 0.16, 0.06, 1.0))
    model.material("matte_black", rgba=(0.09, 0.09, 0.10, 1.0))
    model.material("light_gray", rgba=(0.78, 0.78, 0.79, 1.0))
    model.material("dark_gray_steel", rgba=(0.34, 0.36, 0.38, 1.0))
    model.material("rubber_black", rgba=(0.06, 0.06, 0.06, 1.0))
    model.material("silver_rivet", rgba=(0.74, 0.75, 0.78, 1.0))
    model.material("seat_green", rgba=(0.18, 0.52, 0.24, 1.0))
    model.material("grip_dark", rgba=(0.15, 0.15, 0.18, 1.0))

    # =================================================================
    # Fixed base: pedestal + 4 splayed legs with rubber pads + bracket
    # =================================================================
    base = model.part("pedestal_mount")

    # Central pedestal cylinder
    base.visual(
        Cylinder(radius=PEDESTAL_R, length=PEDESTAL_H),
        origin=Origin(xyz=(0.0, 0.0, PEDESTAL_H / 2.0)),
        material="light_gray",
        name="ground_pedestal",
    )

    # Pivot bracket on top of pedestal
    base.visual(
        Box(BRACKET_SIZE),
        origin=Origin(xyz=(0.0, 0.0, BRACKET_CZ)),
        material="matte_black",
        name="pivot_bracket",
    )

    # Pivot bosses and bolts on bracket cheeks
    for i, sy in enumerate((1.0, -1.0)):
        base.visual(
            Cylinder(radius=0.050, length=0.020),
            origin=Origin(xyz=(0.0, sy * 0.074, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="matte_black",
            name=f"pivot_boss_{i}",
        )
        for j, ang in enumerate((0.25, 0.75, 1.25, 1.75)):
            dx = 0.030 * math.cos(ang * math.pi)
            dz = 0.030 * math.sin(ang * math.pi)
            base.visual(
                Cylinder(radius=0.008, length=0.010),
                origin=Origin(
                    xyz=(dx, sy * 0.086, PIVOT_Z + dz),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material="silver_rivet",
                name=f"bracket_bolt_{i}_{j}",
            )

    # 4 splayed support legs (tubes from pedestal to pads) with rubber ground pads
    leg_dirs = [(1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0)]
    for i, (sx, sy) in enumerate(leg_dirs):
        foot_x = sx * LEG_SPREAD
        foot_y = sy * LEG_SPREAD

        # Leg tube from inside pedestal wall down to pad (overlap ensures connectivity)
        leg_pts = [
            (sx * PEDESTAL_R * 0.5, sy * PEDESTAL_R * 0.5, PEDESTAL_H * 0.3),
            (foot_x * 0.55, foot_y * 0.55, 0.06),
            (foot_x, foot_y, PAD_H + 0.010),
        ]
        base.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    leg_pts, radius=0.024, samples_per_segment=8, radial_segments=14
                ),
                f"support_leg_{i}",
            ),
            material="light_gray",
            name=f"support_leg_{i}",
        )

        # Rubber ground pad at foot of each leg (overlaps leg endpoint)
        base.visual(
            Cylinder(radius=PAD_R, length=PAD_H),
            origin=Origin(xyz=(foot_x, foot_y, PAD_H / 2.0)),
            material="rubber_black",
            name=f"rubber_pad_{i}",
        )

    # =================================================================
    # Rocker: horse-shaped body beam + pivot stub + seats
    # Part frame at pivot axis; geometry relative to that frame.
    # =================================================================
    rocker = model.part("rocker")

    # Horse body using superellipse side loft (sweep axis is Y, then rotate to X)
    # Sections: (sweep_pos, cross_center_1, cross_center_2, half_width)
    horse_sections = [
        (-1.00, 0.0, 0.11, 0.05),   # tail end (narrow)
        (-0.78, 0.0, 0.14, 0.11),   # hindquarters
        (-0.50, 0.0, 0.17, 0.14),   # rear belly
        (-0.15, 0.0, 0.19, 0.16),   # mid body
        (0.15, 0.0, 0.19, 0.16),    # mid body
        (0.50, 0.0, 0.17, 0.14),    # chest
        (0.78, 0.0, 0.16, 0.11),    # shoulders
        (0.92, 0.0, 0.22, 0.06),    # neck base
    ]
    horse_body = superellipse_side_loft(
        horse_sections, exponents=2.4, segments=48
    )
    # Rotate so sweep axis aligns with X (was Y), then raise to clear bracket
    horse_body.rotate_z(-math.pi / 2.0)
    horse_body.translate(0.0, 0.0, 0.06)
    rocker.visual(
        mesh_from_geometry(horse_body, "horse_body"),
        material="gloss_orange",
        name="horse_body",
    )

    # After rotation + translation, body approx:
    # X: -1.0 to 0.92, Y: ±0.16, Z: 0.06+0.06=0.12 to 0.22+0.06=0.28 (rough)
    # Body top ~z=0.28 at midsection, body bottom ~z=0.12

    BODY_TOP_MID = 0.28  # approximate body top at midsection in rocker frame
    BODY_BOT = 0.12       # approximate body bottom

    # Horse head: sphere at the +X end with a snout
    HEAD_X = 1.00
    HEAD_Z = BODY_TOP_MID + 0.06
    HEAD_R = 0.11
    head_sphere = SphereGeometry(HEAD_R, width_segments=24, height_segments=16)
    head_sphere.translate(HEAD_X, 0.0, HEAD_Z)
    rocker.visual(
        mesh_from_geometry(head_sphere, "horse_head"),
        material="gloss_orange",
        name="horse_head",
    )

    # Snout: capsule extending forward from head
    snout = CapsuleGeometry(0.050, 0.12, radial_segments=18, height_segments=6)
    snout.rotate_y(math.pi / 2.0)
    snout.translate(HEAD_X + 0.10, 0.0, HEAD_Z - 0.04)
    rocker.visual(
        mesh_from_geometry(snout, "horse_snout"),
        material="gloss_orange",
        name="horse_snout",
    )

    # Eyes: small dark spheres on head sides
    for i, sy in enumerate((1.0, -1.0)):
        eye = SphereGeometry(0.016, width_segments=12, height_segments=8)
        eye.translate(HEAD_X + 0.04, sy * 0.085, HEAD_Z + 0.03)
        rocker.visual(
            mesh_from_geometry(eye, f"horse_eye_{i}"),
            material="matte_black",
            name=f"horse_eye_{i}",
        )

    # Ears: small cones on top of head
    for i, sy in enumerate((1.0, -1.0)):
        ear = ConeGeometry(0.022, 0.07, radial_segments=12)
        ear.translate(0.0, 0.0, 0.035)
        ear.rotate_x(sy * 0.2)
        ear.translate(HEAD_X - 0.02, sy * 0.055, HEAD_Z + HEAD_R * 0.75)
        rocker.visual(
            mesh_from_geometry(ear, f"horse_ear_{i}"),
            material="dark_brown",
            name=f"horse_ear_{i}",
        )

    # Mane: small fins embedded into the top of the back
    mane_xs = [0.60, 0.38, 0.16, -0.06, -0.28, -0.50]
    for i, mx in enumerate(mane_xs):
        fin_h = 0.075
        # Embed fin bottom well below body surface at all X positions
        fin_center_z = BODY_TOP_MID + fin_h / 2.0 - 0.060
        rocker.visual(
            Box((0.028, 0.014, fin_h)),
            origin=Origin(xyz=(mx, 0.0, fin_center_z)),
            material="dark_brown",
            name=f"mane_fin_{i}",
        )

    # Tail: curved tube at -X end, connected to body rear
    tail_pts = [
        (-0.98, 0.0, 0.15),
        (-1.06, 0.0, 0.22),
        (-1.12, 0.0, 0.32),
        (-1.08, 0.0, 0.42),
    ]
    rocker.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                tail_pts, radius=0.020, samples_per_segment=10, radial_segments=14
            ),
            "horse_tail",
        ),
        material="dark_brown",
        name="horse_tail",
    )

    # Decorative horse legs (4 stubby cylinders hanging from body)
    # Leg tops are embedded into the body bottom surface to ensure connectivity
    dec_leg_positions = [
        (0.50, 0.06), (0.50, -0.06),
        (-0.50, 0.06), (-0.50, -0.06),
    ]
    for i, (lx, ly) in enumerate(dec_leg_positions):
        leg_top = BODY_BOT + 0.04  # embedded 0.04 above body bottom AABB min
        leg_len = 0.14
        rocker.visual(
            Cylinder(radius=0.028, length=leg_len),
            origin=Origin(xyz=(lx, ly, leg_top - leg_len / 2.0)),
            material="gloss_orange",
            name=f"horse_leg_{i}",
        )
        # Hoof at bottom of leg, overlaps slightly with leg
        rocker.visual(
            Cylinder(radius=0.032, length=0.024),
            origin=Origin(xyz=(lx, ly, leg_top - leg_len + 0.005)),
            material="dark_brown",
            name=f"horse_hoof_{i}",
        )

    # Pivot stub descending from body into bracket
    # Goes from inside body (z=BODY_BOT) down into bracket (z well below 0)
    stub_top = BODY_BOT + 0.04
    stub_bot = -0.08
    stub_len = stub_top - stub_bot
    stub_cz = (stub_top + stub_bot) / 2.0
    rocker.visual(
        Cylinder(radius=0.042, length=stub_len),
        origin=Origin(xyz=(0.0, 0.0, stub_cz)),
        material="gloss_orange",
        name="pivot_stub",
    )

    # Flat seat plates (rounded shape) near each end, on top of the body
    seat_profile = sample_catmull_rom_spline_2d(
        [
            (0.16, 0.0),
            (0.04, 0.09),
            (-0.07, 0.12),
            (-0.14, 0.07),
            (-0.16, 0.0),
            (-0.14, -0.07),
            (-0.07, -0.12),
            (0.04, -0.09),
        ],
        samples_per_segment=8,
        closed=True,
    )
    SEAT_Z = BODY_TOP_MID + 0.008  # seat just above body top

    for i, s in enumerate((1.0, -1.0)):
        seat = ExtrudeGeometry(seat_profile, 0.014, cap=True, center=True)
        if s < 0:
            seat.rotate_z(math.pi)
        seat.translate(s * SEAT_X, 0.0, SEAT_Z)
        rocker.visual(
            mesh_from_geometry(seat, f"seat_plate_{i}"),
            material="seat_green",
            name=f"seat_plate_{i}",
        )
        # Seat support bracket connecting seat down through the body
        # Starts below body bottom; top overlaps into the seat plate
        bracket_bot = BODY_BOT - 0.02
        bracket_top = SEAT_Z + 0.005  # slightly above seat center to overlap plate
        bracket_h = bracket_top - bracket_bot
        bracket_cz = (bracket_top + bracket_bot) / 2.0
        rocker.visual(
            Box((0.07, 0.055, bracket_h)),
            origin=Origin(xyz=(s * SEAT_X, 0.0, bracket_cz)),
            material="dark_gray_steel",
            name=f"seat_bracket_{i}",
        )

    # Handle mounting stubs (fixed part of rocker, connecting body to handle joint)
    # Extend well below the body surface at all X positions to ensure connectivity
    for i, s in enumerate((1.0, -1.0)):
        stub_bot = BODY_BOT - 0.02  # below body bottom AABB, guaranteed overlap
        stub_top = HANDLE_MOUNT_Z + 0.02
        stub_h = stub_top - stub_bot
        stub_cz = (stub_top + stub_bot) / 2.0
        rocker.visual(
            Cylinder(radius=0.018, length=stub_h),
            origin=Origin(xyz=(s * HANDLE_X, 0.0, stub_cz)),
            material="dark_gray_steel",
            name=f"handle_stub_{i}",
        )

    # =================================================================
    # Pivoting handles (separate parts with revolute joints)
    # =================================================================
    for i, s in enumerate((1.0, -1.0)):
        handle = model.part(f"handle_{i}")

        # Handle stem rising from mount point
        handle.visual(
            Cylinder(radius=0.015, length=HANDLE_STEM_H),
            origin=Origin(xyz=(0.0, 0.0, HANDLE_STEM_H / 2.0)),
            material="dark_gray_steel",
            name=f"handle_stem_{i}",
        )

        # Horizontal grip bar at top of stem
        handle.visual(
            Cylinder(radius=0.013, length=HANDLE_GRIP_LEN),
            origin=Origin(
                xyz=(0.0, 0.0, HANDLE_STEM_H),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material="grip_dark",
            name=f"handle_grip_{i}",
        )

        # Grip end caps (rubber balls)
        for j, gy in enumerate((1.0, -1.0)):
            cap = SphereGeometry(0.017, width_segments=10, height_segments=8)
            cap.translate(0.0, gy * HANDLE_GRIP_LEN / 2.0, HANDLE_STEM_H)
            handle.visual(
                mesh_from_geometry(cap, f"grip_cap_{i}_{j}"),
                material="rubber_black",
                name=f"grip_cap_{i}_{j}",
            )

        # Revolute joint: handle pivots forward/backward on the rocker
        model.articulation(
            f"handle_pivot_{i}",
            ArticulationType.REVOLUTE,
            parent=rocker,
            child=handle,
            origin=Origin(xyz=(s * HANDLE_X, 0.0, HANDLE_MOUNT_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0, velocity=2.0, lower=-HANDLE_ROCK, upper=HANDLE_ROCK
            ),
        )

    # =================================================================
    # Main rocking pivot
    # =================================================================
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
    handle_0 = object_model.get_part("handle_0")
    handle_1 = object_model.get_part("handle_1")
    handle_pivot_0 = object_model.get_articulation("handle_pivot_0")
    handle_pivot_1 = object_model.get_articulation("handle_pivot_1")

    # --- Pivot stub captured inside bracket ---
    ctx.allow_overlap(
        rocker,
        base,
        elem_a="pivot_stub",
        elem_b="pivot_bracket",
        reason="The center stub descends into the cast pivot bracket that captures the rocking axle.",
    )
    ctx.expect_overlap(
        rocker, base, axes="z",
        elem_a="pivot_stub", elem_b="pivot_bracket",
        min_overlap=0.02,
        name="pivot stub inserted into bracket",
    )
    ctx.expect_within(
        rocker, base, axes="xy",
        inner_elem="pivot_stub", outer_elem="pivot_bracket",
        margin=0.0,
        name="pivot stub centered in bracket",
    )

    # --- Handle stems nest inside mounting stubs (pivot bore fit) ---
    ctx.allow_overlap(
        handle_0, rocker,
        elem_a="handle_stem_0", elem_b="handle_stub_0",
        reason="Handle stem pivots inside the fixed mounting stub bore on the rocker.",
    )
    ctx.allow_overlap(
        handle_1, rocker,
        elem_a="handle_stem_1", elem_b="handle_stub_1",
        reason="Handle stem pivots inside the fixed mounting stub bore on the rocker.",
    )
    # Prove handle stems are centered in their stubs
    for i in range(2):
        ctx.expect_within(
            object_model.get_part(f"handle_{i}"),
            rocker,
            axes="xy",
            inner_elem=f"handle_stem_{i}",
            outer_elem=f"handle_stub_{i}",
            margin=0.005,
            name=f"handle_{i} stem centered in stub",
        )

    # --- Bracket on pedestal ---
    bracket = ctx.part_element_world_aabb(base, elem="pivot_bracket")
    pedestal = ctx.part_element_world_aabb(base, elem="ground_pedestal")
    ctx.check(
        "bracket sits atop ground pedestal",
        _intersects(bracket, pedestal),
        details=f"bracket={bracket}, pedestal={pedestal}",
    )

    # --- Rubber ground pads exist under support legs ---
    pad_names = [f"rubber_pad_{i}" for i in range(4)]
    pads = [ctx.part_element_world_aabb(base, elem=n) for n in pad_names]
    ctx.check(
        "four rubber ground pads exist",
        all(p is not None for p in pads),
        details=f"pads={pads}",
    )
    ctx.check(
        "rubber pads at ground level",
        all(p is not None and p[0][2] < 0.02 for p in pads),
        details=f"pad_min_z={[p[0][2] if p else None for p in pads]}",
    )
    if all(p is not None for p in pads):
        pad_cxs = [0.5 * (p[0][0] + p[1][0]) for p in pads]
        ctx.check(
            "pads spread outward from center",
            max(pad_cxs) - min(pad_cxs) > 0.20,
            details=f"pad_centers_x={pad_cxs}",
        )

    # --- Support legs connect pedestal to pads ---
    for i in range(4):
        leg = ctx.part_element_world_aabb(base, elem=f"support_leg_{i}")
        pad = pads[i]
        ctx.check(
            f"support_leg_{i} reaches its rubber pad",
            leg is not None and pad is not None and _intersects(leg, pad),
            details=f"leg={leg}, pad={pad}",
        )

    # --- Horse body spans the seesaw length ---
    horse = ctx.part_element_world_aabb(rocker, elem="horse_body")
    ctx.check(
        "horse body spans the seesaw length",
        horse is not None and (horse[1][0] - horse[0][0]) >= 1.5,
        details=f"horse={horse}",
    )

    # --- Head at one end ---
    head = ctx.part_element_world_aabb(rocker, elem="horse_head")
    ctx.check(
        "horse head at front end",
        head is not None and head[0][0] > 0.7,
        details=f"head={head}",
    )

    # --- Tail at other end ---
    tail = ctx.part_element_world_aabb(rocker, elem="horse_tail")
    ctx.check(
        "horse tail at rear end",
        tail is not None and tail[1][0] < -0.7,
        details=f"tail={tail}",
    )

    # --- Overall envelope ---
    ra = ctx.part_world_aabb(rocker)
    ba = ctx.part_world_aabb(base)
    ctx.check(
        "overall length about 2.0-2.6 m",
        ra is not None and 1.8 <= (ra[1][0] - ra[0][0]) <= 2.8,
        details=f"rocker aabb={ra}",
    )
    ctx.check(
        "overall height about 0.6-1.0 m",
        ra is not None and ba is not None and 0.55 <= max(ra[1][2], ba[1][2]) <= 1.05,
        details=f"rocker={ra}, base={ba}",
    )

    # --- Seats at opposite ends ---
    seat0 = ctx.part_element_world_aabb(rocker, elem="seat_plate_0")
    seat1 = ctx.part_element_world_aabb(rocker, elem="seat_plate_1")
    ctx.check(
        "seats exist at opposite ends",
        seat0 is not None and seat1 is not None
        and seat0[1][0] > 0.3 and seat1[0][0] < -0.3,
        details=f"seat0={seat0}, seat1={seat1}",
    )

    # --- Handle pivots: non-zero range ---
    h0_lim = handle_pivot_0.motion_limits
    h1_lim = handle_pivot_1.motion_limits
    ctx.check(
        "handle pivots have non-zero range",
        h0_lim is not None and h1_lim is not None
        and h0_lim.upper > h0_lim.lower
        and h1_lim.upper > h1_lim.lower,
        details=f"h0=({h0_lim.lower}, {h0_lim.upper}), h1=({h1_lim.lower}, {h1_lim.upper})",
    )

    # --- Handle pose: handles actually tilt (check X extent for Y-axis rotation) ---
    h0_rest = ctx.part_world_aabb(handle_0)
    h1_rest = ctx.part_world_aabb(handle_1)
    with ctx.pose({handle_pivot_0: HANDLE_ROCK}):
        h0_posed = ctx.part_world_aabb(handle_0)
        ctx.check(
            "handle_0 tilts when posed",
            h0_rest is not None and h0_posed is not None
            and abs(h0_rest[1][0] - h0_posed[1][0]) > 0.005,
            details=f"rest_max_x={h0_rest[1][0] if h0_rest else None}, posed_max_x={h0_posed[1][0] if h0_posed else None}",
        )
    with ctx.pose({handle_pivot_1: -HANDLE_ROCK}):
        h1_posed = ctx.part_world_aabb(handle_1)
        ctx.check(
            "handle_1 tilts when posed",
            h1_rest is not None and h1_posed is not None
            and abs(h1_rest[0][0] - h1_posed[0][0]) > 0.005,
            details=f"rest_min_x={h1_rest[0][0] if h1_rest else None}, posed_min_x={h1_posed[0][0] if h1_posed else None}",
        )

    # --- Main rocker pivot limits ---
    lim = pivot.motion_limits
    ctx.check(
        "rocking range about +/- 15 degrees",
        lim is not None
        and abs(lim.lower + ROCK_LIMIT) < 0.02
        and abs(lim.upper - ROCK_LIMIT) < 0.02,
        details=f"limits=({lim.lower}, {lim.upper})",
    )

    # --- Rocker pose: seats swap height, base stays fixed ---
    base_rest = ctx.part_world_aabb(base)
    with ctx.pose({pivot: ROCK_LIMIT}):
        seat0_dn = ctx.part_element_world_aabb(rocker, elem="seat_plate_0")
        seat1_up = ctx.part_element_world_aabb(rocker, elem="seat_plate_1")
        rocker_dn = ctx.part_world_aabb(rocker)
        base_posed = ctx.part_world_aabb(base)
        ctx.check(
            "positive rock lowers seat_0 and raises seat_1",
            seat0_dn is not None and seat1_up is not None
            and seat0 is not None and seat1 is not None
            and seat0_dn[1][2] < seat0[1][2] - 0.08
            and seat1_up[1][2] > seat1[1][2] + 0.08,
            details=f"seat0_dn={seat0_dn}, seat1_up={seat1_up}",
        )
        ctx.check(
            "rocker clears the ground at full tilt",
            rocker_dn is not None and rocker_dn[0][2] > 0.005,
            details=f"rocker={rocker_dn}",
        )
        ctx.check(
            "base stays fixed while rocking",
            base_rest is not None and base_posed is not None
            and abs(base_rest[1][2] - base_posed[1][2]) < 1e-6,
            details=f"rest={base_rest}, posed={base_posed}",
        )

    return ctx.report()


object_model = build_object_model()
