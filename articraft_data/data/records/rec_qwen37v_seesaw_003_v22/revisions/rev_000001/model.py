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
# Shared dimensions (meters). World: Z up, X and Y along the two beams.
# ---------------------------------------------------------------------------
PIVOT_Z = 0.34           # world height of the rocking axis
BEAM_R = 0.06            # main tube radius (~120 mm diameter)
BEAM_HALF = 1.15         # half-length of each curved beam
CURVE_C = 0.1285         # parabolic curvature
BEAM_CENTER_Z = 0.16     # beam centerline height at origin, relative to pivot

COLLAR_DIST = 0.97       # clamp collar distance along beam
SEAT_DIST = 1.14         # seat center distance from pivot
SEAT_Z = 0.062           # seat plate mid-plane, relative to pivot
PLATE_T = 0.012
HANDLE_DIST = 1.03
HANDLE_Z = 0.552         # handle plate mid-plane, relative to pivot

ROCK_LIMIT = 0.262       # ~15 degrees each way

PEDESTAL_R = 0.075
PEDESTAL_H = 0.22
BRACKET_SIZE = (0.16, 0.13, 0.17)
BRACKET_CZ = 0.295       # bracket box center height

# Cross-seesaw additions
LEG_SPREAD = 0.38        # horizontal distance from center to foot
LEG_ATTACH_R = 0.08      # where legs attach radially from pedestal center
PAD_R = 0.055            # rubber ground pad radius
PAD_H = 0.015            # rubber ground pad height

BACKREST_W = 0.20
BACKREST_H = 0.18
BACKREST_T = 0.012
BACKREST_TILT_MAX = 0.30 # ~17 degrees recline
BACKREST_HINGE_DIST = 0.90  # behind the seat, clear of clamp collars at 0.97

BUMP_R = 0.032           # bump stop cylinder radius
BUMP_H = 0.040           # bump stop height

HUB_R = 0.10             # central hub connecting plate radius
HUB_H = 0.14             # hub height


def _beam_z(d: float) -> float:
    """Beam centerline height (relative to pivot frame) at distance d."""
    return BEAM_CENTER_Z + CURVE_C * d * d


def _rotate_xy(x: float, y: float, angle: float) -> tuple[float, float]:
    c, s = math.cos(angle), math.sin(angle)
    return (c * x - s * y, s * x + c * y)


# End definitions: (index, sign_x, sign_y, rotation_around_Z for seat/grip)
ENDS = [
    (0,  1.0,  0.0, 0.0),           # +X end
    (1, -1.0,  0.0, math.pi),       # -X end
    (2,  0.0,  1.0, math.pi / 2.0), # +Y end
    (3,  0.0, -1.0, -math.pi / 2.0),# -Y end
]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cross_seesaw")

    model.material("gloss_red_orange", rgba=(0.88, 0.20, 0.06, 1.0))
    model.material("matte_black", rgba=(0.09, 0.09, 0.10, 1.0))
    model.material("light_gray", rgba=(0.78, 0.78, 0.79, 1.0))
    model.material("dark_gray_steel", rgba=(0.34, 0.36, 0.38, 1.0))
    model.material("silver_rivet", rgba=(0.74, 0.75, 0.78, 1.0))
    model.material("rubber_dark", rgba=(0.12, 0.12, 0.13, 1.0))

    # =================================================================
    # Fixed base: pedestal + 4 legs + rubber pads + bracket + bolts
    # =================================================================
    base = model.part("pedestal_mount")

    # Central cylindrical pedestal
    base.visual(
        Cylinder(radius=PEDESTAL_R, length=PEDESTAL_H),
        origin=Origin(xyz=(0.0, 0.0, PEDESTAL_H / 2.0)),
        material="light_gray",
        name="ground_pedestal",
    )

    # Cast pivot bracket
    base.visual(
        Box(BRACKET_SIZE),
        origin=Origin(xyz=(0.0, 0.0, BRACKET_CZ)),
        material="matte_black",
        name="pivot_bracket",
    )

    # Pivot bosses and bolts on bracket cheeks
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

    # Four support legs with rubber ground pads (at 45° between beams)
    for i, angle in enumerate(
        (math.pi / 4.0, 3.0 * math.pi / 4.0, 5.0 * math.pi / 4.0, 7.0 * math.pi / 4.0)
    ):
        cx, cy = math.cos(angle), math.sin(angle)
        leg_pts = [
            (LEG_ATTACH_R * cx, LEG_ATTACH_R * cy, PEDESTAL_H * 0.75),
            (LEG_SPREAD * 0.45 * cx, LEG_SPREAD * 0.45 * cy, PEDESTAL_H * 0.35),
            (LEG_SPREAD * cx, LEG_SPREAD * cy, PAD_H + 0.002),
        ]
        base.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    leg_pts, radius=0.022, samples_per_segment=8, radial_segments=16
                ),
                f"leg_{i}",
            ),
            material="light_gray",
            name=f"leg_{i}",
        )
        # Rubber ground pad
        base.visual(
            Cylinder(radius=PAD_R, length=PAD_H),
            origin=Origin(xyz=(LEG_SPREAD * cx, LEG_SPREAD * cy, PAD_H / 2.0)),
            material="rubber_dark",
            name=f"rubber_pad_{i}",
        )

    # =================================================================
    # Rocker: two perpendicular curved beams + hub + stub + ends
    # =================================================================
    rocker = model.part("rocker")

    # X-axis beam (swept banana tube)
    n = 12
    beam_x_pts = [
        (BEAM_HALF * k / n, 0.0, _beam_z(BEAM_HALF * k / n))
        for k in range(-n, n + 1)
    ]
    rocker.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                beam_x_pts,
                radius=BEAM_R,
                samples_per_segment=4,
                radial_segments=28,
                cap_ends=True,
            ),
            "beam_x",
        ),
        material="gloss_red_orange",
        name="beam_x",
    )

    # Y-axis beam (perpendicular banana tube)
    beam_y_pts = [
        (0.0, BEAM_HALF * k / n, _beam_z(BEAM_HALF * k / n))
        for k in range(-n, n + 1)
    ]
    rocker.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                beam_y_pts,
                radius=BEAM_R,
                samples_per_segment=4,
                radial_segments=28,
                cap_ends=True,
            ),
            "beam_y",
        ),
        material="gloss_red_orange",
        name="beam_y",
    )

    # Central hub plate connecting both beams at the crossing
    rocker.visual(
        Cylinder(radius=HUB_R, length=HUB_H),
        origin=Origin(xyz=(0.0, 0.0, BEAM_CENTER_Z)),
        material="gloss_red_orange",
        name="hub_plate",
    )

    # Red flare wedge blending into the pivot stub
    wedge = ConeGeometry(0.085, 0.09, radial_segments=28).rotate_x(math.pi)
    wedge.translate(0.0, 0.0, 0.110)
    rocker.visual(
        mesh_from_geometry(wedge, "pivot_wedge"),
        material="gloss_red_orange",
        name="pivot_wedge",
    )

    # Short red stub descending from hub into the black bracket
    rocker.visual(
        Cylinder(radius=0.048, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material="gloss_red_orange",
        name="pivot_stub",
    )

    # Shared profiles for seats and grips
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

    collar_z = _beam_z(COLLAR_DIST)
    slope = 2.0 * CURVE_C * COLLAR_DIST
    tangent = math.atan(slope)

    rivet_xy = [
        (0.13, 0.0),
        (0.0, 0.10),
        (0.0, -0.10),
        (-0.13, 0.075),
        (-0.13, -0.075),
    ]

    # Build four mirrored end assemblies
    for idx, sx, sy, rot_z in ENDS:
        # Direction unit vector (along beam for this end)
        dx, dy = sx, sy  # one is ±1, other is 0

        # -- Clamp collar ring --
        if abs(sx) > 0.5:
            # X-axis beam collar
            collar_origin = Origin(
                xyz=(sx * COLLAR_DIST, 0.0, collar_z),
                rpy=(0.0, math.pi / 2.0 - sx * tangent, 0.0),
            )
        else:
            # Y-axis beam collar
            collar_origin = Origin(
                xyz=(0.0, sy * COLLAR_DIST, collar_z),
                rpy=(math.pi / 2.0 - sy * tangent, 0.0, 0.0),
            )
        rocker.visual(
            Cylinder(radius=0.080, length=0.085),
            origin=collar_origin,
            material="matte_black",
            name=f"clamp_collar_{idx}",
        )

        # Collar bolts (perpendicular to beam direction)
        if abs(sx) > 0.5:
            for j, bs in enumerate((1.0, -1.0)):
                rocker.visual(
                    Cylinder(radius=0.011, length=0.032),
                    origin=Origin(
                        xyz=(sx * COLLAR_DIST, bs * 0.082, collar_z),
                        rpy=(math.pi / 2.0, 0.0, 0.0),
                    ),
                    material="silver_rivet",
                    name=f"collar_bolt_{idx}_{j}",
                )
        else:
            for j, bs in enumerate((1.0, -1.0)):
                rocker.visual(
                    Cylinder(radius=0.011, length=0.032),
                    origin=Origin(
                        xyz=(bs * 0.082, sy * COLLAR_DIST, collar_z),
                        rpy=(0.0, math.pi / 2.0, 0.0),
                    ),
                    material="silver_rivet",
                    name=f"collar_bolt_{idx}_{j}",
                )

        # -- Drop tube: from collar down to seat --
        if abs(sx) > 0.5:
            drop_pts = [
                (sx * COLLAR_DIST, 0.0, collar_z),
                (sx * 1.05, 0.0, 0.185),
                (sx * 1.12, 0.0, 0.105),
                (sx * SEAT_DIST + sx * 0.01, 0.0, 0.066),
            ]
        else:
            drop_pts = [
                (0.0, sy * COLLAR_DIST, collar_z),
                (0.0, sy * 1.05, 0.185),
                (0.0, sy * 1.12, 0.105),
                (0.0, sy * SEAT_DIST + sy * 0.01, 0.066),
            ]
        rocker.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    drop_pts, radius=0.026, samples_per_segment=10, radial_segments=18
                ),
                f"drop_tube_{idx}",
            ),
            material="gloss_red_orange",
            name=f"drop_tube_{idx}",
        )

        # -- Seat plate (rounded triangular, rotated for direction) --
        seat = ExtrudeGeometry(seat_profile, PLATE_T, cap=True, center=True)
        seat.rotate_z(rot_z)
        seat.translate(sx * SEAT_DIST, sy * SEAT_DIST, SEAT_Z)
        rocker.visual(
            mesh_from_geometry(seat, f"seat_plate_{idx}"),
            material="dark_gray_steel",
            name=f"seat_plate_{idx}",
        )

        # Seat rivets (rotated with seat)
        for j, (lx, ly) in enumerate(rivet_xy):
            rx, ry = _rotate_xy(lx, ly, rot_z)
            rocker.visual(
                Cylinder(radius=0.008, length=0.010),
                origin=Origin(xyz=(sx * SEAT_DIST + rx, sy * SEAT_DIST + ry, 0.070)),
                material="silver_rivet",
                name=f"seat_rivet_{idx}_{j}",
            )

        # -- Seat fin (small stop under seat nose) --
        fin_x = sx * 1.26 if abs(sx) > 0.5 else 0.0
        fin_y = sy * 1.26 if abs(sy) > 0.5 else 0.0
        rocker.visual(
            Box((0.045, 0.022, 0.04)),
            origin=Origin(xyz=(fin_x, fin_y, 0.038)),
            material="matte_black",
            name=f"seat_fin_{idx}",
        )

        # -- Handle post (thin red tube rising to grip plate) --
        if abs(sx) > 0.5:
            post_pts = [
                (sx * COLLAR_DIST, 0.0, 0.285),
                (sx * 0.985, 0.0, 0.40),
                (sx * 1.01, 0.0, 0.48),
                (sx * HANDLE_DIST, 0.0, 0.550),
            ]
        else:
            post_pts = [
                (0.0, sy * COLLAR_DIST, 0.285),
                (0.0, sy * 0.985, 0.40),
                (0.0, sy * 1.01, 0.48),
                (0.0, sy * HANDLE_DIST, 0.550),
            ]
        rocker.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    post_pts, radius=0.021, samples_per_segment=10, radial_segments=18
                ),
                f"handle_post_{idx}",
            ),
            material="gloss_red_orange",
            name=f"handle_post_{idx}",
        )

        # -- Handle/grip plate (rotated for beam direction) --
        grip = ExtrudeWithHolesGeometry(
            grip_outer, grip_holes, PLATE_T, cap=True, center=True
        )
        grip.rotate_z(rot_z)
        grip.translate(sx * HANDLE_DIST, sy * HANDLE_DIST, HANDLE_Z)
        rocker.visual(
            mesh_from_geometry(grip, f"handle_plate_{idx}"),
            material="dark_gray_steel",
            name=f"handle_plate_{idx}",
        )

        # -- Bump stop (rubber cylinder below beam end) --
        beam_end_z = _beam_z(BEAM_HALF)
        bump_x = sx * BEAM_HALF if abs(sx) > 0.5 else 0.0
        bump_y = sy * BEAM_HALF if abs(sy) > 0.5 else 0.0
        rocker.visual(
            Cylinder(radius=BUMP_R, length=BUMP_H),
            origin=Origin(
                xyz=(bump_x, bump_y, beam_end_z - BEAM_R - BUMP_H / 2.0),
            ),
            material="rubber_dark",
            name=f"bump_stop_{idx}",
        )

    # =================================================================
    # Backrests: four separate parts, each with a revolute tilt joint
    # =================================================================
    # Backrest hinge axes: positive q reclines the backrest top backward
    # (toward center, away from the sitter who faces outward).
    backrest_axes = [
        (0.0, 1.0, 0.0),   # seat 0 (+X): recline toward -X
        (0.0, -1.0, 0.0),  # seat 1 (-X): recline toward +X
        (1.0, 0.0, 0.0),   # seat 2 (+Y): recline toward -Y
        (-1.0, 0.0, 0.0),  # seat 3 (-Y): recline toward +Y
    ]

    hinge_z = SEAT_Z + PLATE_T / 2.0  # just above seat top surface

    for idx, sx, sy, rot_z in ENDS:
        backrest = model.part(f"backrest_{idx}")

        # Backrest plate: thin vertical panel behind the seat
        # For X-axis seats: thin in X, wide in Y, tall in Z
        # For Y-axis seats: wide in X, thin in Y, tall in Z
        if abs(sx) > 0.5:
            bp_size = (BACKREST_T, BACKREST_W, BACKREST_H)
        else:
            bp_size = (BACKREST_W, BACKREST_T, BACKREST_H)

        backrest.visual(
            Box(bp_size),
            origin=Origin(xyz=(0.0, 0.0, BACKREST_H / 2.0)),
            material="dark_gray_steel",
            name=f"backrest_plate_{idx}",
        )

        # Small hinge barrel visual on the backrest
        backrest.visual(
            Cylinder(radius=0.014, length=0.05),
            origin=Origin(
                xyz=(0.0, 0.0, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0) if abs(sx) > 0.5 else (0.0, 0.0, 0.0),
            ),
            material="matte_black",
            name=f"backrest_hinge_barrel_{idx}",
        )

        # Articulation: revolute tilt joint
        hinge_x = sx * BACKREST_HINGE_DIST
        hinge_y = sy * BACKREST_HINGE_DIST
        model.articulation(
            f"backrest_tilt_{idx}",
            ArticulationType.REVOLUTE,
            parent=rocker,
            child=backrest,
            origin=Origin(xyz=(hinge_x, hinge_y, hinge_z)),
            axis=backrest_axes[idx],
            motion_limits=MotionLimits(
                effort=20.0,
                velocity=1.0,
                lower=0.0,
                upper=BACKREST_TILT_MAX,
            ),
        )

    # =================================================================
    # Main rocking pivot: horizontal axis perpendicular to X beam
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

    backrests = [object_model.get_part(f"backrest_{i}") for i in range(4)]
    backrest_joints = [object_model.get_articulation(f"backrest_tilt_{i}") for i in range(4)]

    # --- Intentional overlaps ---
    # Pivot stub captured inside bracket
    ctx.allow_overlap(
        rocker,
        base,
        elem_a="pivot_stub",
        elem_b="pivot_bracket",
        reason="The red center stub descends into the cast pivot bracket that captures the rocking axle.",
    )
    # Backrest plates pass through the curved beam zone where they mount to seats.
    # In reality the backrest bracket has a clearance cut around the beam.
    for i in range(4):
        beam_name = "beam_x" if i < 2 else "beam_y"
        ctx.allow_overlap(
            backrests[i],
            rocker,
            elem_a=f"backrest_plate_{i}",
            elem_b=beam_name,
            reason=f"Backrest plate {i} passes through the curved beam zone at its seat mounting point.",
        )
        # Proof: backrest is near its seat
        ctx.expect_contact(
            backrests[i],
            rocker,
            elem_a=f"backrest_plate_{i}",
            elem_b=f"seat_plate_{i}",
            contact_tol=0.05,
            name=f"backrest_{i} mounted near seat_{i}",
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

    # --- Bracket seated on pedestal ---
    bracket = ctx.part_element_world_aabb(base, elem="pivot_bracket")
    pedestal = ctx.part_element_world_aabb(base, elem="ground_pedestal")
    ctx.check(
        "bracket sits atop ground pedestal",
        _intersects(bracket, pedestal),
        details=f"bracket={bracket}, pedestal={pedestal}",
    )

    # --- Cross beams: two perpendicular curved tubes ---
    beam_x = ctx.part_element_world_aabb(rocker, elem="beam_x")
    beam_y = ctx.part_element_world_aabb(rocker, elem="beam_y")
    ctx.check(
        "beam_x spans along X",
        beam_x is not None and (beam_x[1][0] - beam_x[0][0]) >= 2.2,
        details=f"beam_x={beam_x}",
    )
    ctx.check(
        "beam_y spans along Y",
        beam_y is not None and (beam_y[1][1] - beam_y[0][1]) >= 2.2,
        details=f"beam_y={beam_y}",
    )
    ctx.check(
        "both beams curve upward at ends",
        beam_x is not None
        and beam_y is not None
        and (beam_x[1][2] - beam_x[0][2]) >= 0.25
        and (beam_y[1][2] - beam_y[0][2]) >= 0.25,
        details=f"beam_x z={(None if beam_x is None else beam_x[1][2] - beam_x[0][2])}, "
        f"beam_y z={(None if beam_y is None else beam_y[1][2] - beam_y[0][2])}",
    )

    # --- Four seats at opposite beam ends ---
    seats = [ctx.part_element_world_aabb(rocker, elem=f"seat_plate_{i}") for i in range(4)]
    ctx.check(
        "four seats exist at sitting height",
        all(s is not None for s in seats)
        and all(0.30 <= s[1][2] <= 0.50 for s in seats),
        details=f"seats={seats}",
    )

    # Seats on X beam at ±X, seats on Y beam at ±Y
    def _cx(aabb):
        return 0.5 * (aabb[0][0] + aabb[1][0])

    def _cy(aabb):
        return 0.5 * (aabb[0][1] + aabb[1][1])

    ctx.check(
        "seat_0 at +X end, seat_1 at -X end",
        seats[0] is not None
        and seats[1] is not None
        and _cx(seats[0]) > 0.8
        and _cx(seats[1]) < -0.8,
        details=f"seat0_cx={_cx(seats[0]) if seats[0] else None}, seat1_cx={_cx(seats[1]) if seats[1] else None}",
    )
    ctx.check(
        "seat_2 at +Y end, seat_3 at -Y end",
        seats[2] is not None
        and seats[3] is not None
        and _cy(seats[2]) > 0.8
        and _cy(seats[3]) < -0.8,
        details=f"seat2_cy={_cy(seats[2]) if seats[2] else None}, seat3_cy={_cy(seats[3]) if seats[3] else None}",
    )

    # --- Four backrests on tilt joints ---
    for i in range(4):
        bp = ctx.part_element_world_aabb(backrests[i], elem=f"backrest_plate_{i}")
        ctx.check(
            f"backrest_{i} plate exists above seat_{i}",
            bp is not None and seats[i] is not None and bp[0][2] >= seats[i][0][2] - 0.02,
            details=f"backrest={bp}, seat={seats[i]}",
        )

    # Backrest joint limits
    for i, jnt in enumerate(backrest_joints):
        lim = jnt.motion_limits
        ctx.check(
            f"backrest_tilt_{i} has recline limits",
            lim is not None and abs(lim.lower) < 0.02 and 0.20 <= lim.upper <= 0.40,
            details=f"limits=({lim.lower}, {lim.upper})",
        )

    # --- Rubber ground pads under support legs ---
    pads = [ctx.part_element_world_aabb(base, elem=f"rubber_pad_{i}") for i in range(4)]
    ctx.check(
        "four rubber ground pads at ground level",
        all(p is not None for p in pads)
        and all(p[0][2] < 0.02 for p in pads),
        details=f"pads={pads}",
    )

    # Legs connect pedestal to pads
    legs = [ctx.part_element_world_aabb(base, elem=f"leg_{i}") for i in range(4)]
    for i in range(4):
        ctx.check(
            f"leg_{i} connects pedestal to rubber_pad_{i}",
            _intersects(legs[i], pedestal) and _intersects(legs[i], pads[i]),
            details=f"leg={legs[i]}, pedestal={pedestal}, pad={pads[i]}",
        )

    # --- Safety bump stops below each beam end ---
    bumps = [ctx.part_element_world_aabb(rocker, elem=f"bump_stop_{i}") for i in range(4)]
    ctx.check(
        "four bump stops exist below beam ends",
        all(b is not None for b in bumps),
        details=f"bumps={bumps}",
    )
    for i in range(4):
        ctx.check(
            f"bump_stop_{i} is below nearby beam geometry",
            bumps[i] is not None
            and (
                (beam_x is not None and bumps[i][1][2] < beam_x[1][2])
                or (beam_y is not None and bumps[i][1][2] < beam_y[1][2])
            ),
            details=f"bump={bumps[i]}, beam_x={beam_x}, beam_y={beam_y}",
        )

    # --- Handle/grip plates above beam ends ---
    grips = [ctx.part_element_world_aabb(rocker, elem=f"handle_plate_{i}") for i in range(4)]
    ctx.check(
        "four grip plates above beam ends",
        all(g is not None for g in grips)
        and beam_x is not None
        and beam_y is not None
        and grips[0][0][2] > beam_x[1][2] - 0.1
        and grips[1][0][2] > beam_x[1][2] - 0.1
        and grips[2][0][2] > beam_y[1][2] - 0.1
        and grips[3][0][2] > beam_y[1][2] - 0.1,
        details=f"grips={grips}",
    )

    # --- Drop tubes connect beams to seats, handle posts connect to grips ---
    for i in range(4):
        drop = ctx.part_element_world_aabb(rocker, elem=f"drop_tube_{i}")
        post = ctx.part_element_world_aabb(rocker, elem=f"handle_post_{i}")
        target_beam = beam_x if i < 2 else beam_y
        ctx.check(
            f"drop_tube_{i} bridges beam to seat_{i}",
            _intersects(drop, target_beam) and _intersects(drop, seats[i]),
            details=f"drop={drop}",
        )
        ctx.check(
            f"handle_post_{i} bridges beam to grip_{i}",
            _intersects(post, target_beam) and _intersects(post, grips[i]),
            details=f"post={post}",
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

    # --- Overall envelope ---
    ra = ctx.part_world_aabb(rocker)
    ba = ctx.part_world_aabb(base)
    ctx.check(
        "overall span about 2.6 m",
        ra is not None
        and 2.2 <= max(ra[1][0] - ra[0][0], ra[1][1] - ra[0][1]) <= 2.8,
        details=f"rocker aabb={ra}",
    )
    ctx.check(
        "overall height about 0.9 m",
        ra is not None and ba is not None and 0.82 <= max(ra[1][2], ba[1][2]) <= 0.98,
        details=f"rocker={ra}, base={ba}",
    )

    # --- Decisive pose: rocker tilts, seats swap height, backrest reclines ---
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
            and seats[0] is not None
            and seats[1] is not None
            and seat0_dn[1][2] < seats[0][1][2] - 0.15
            and seat1_up[1][2] > seats[1][1][2] + 0.15,
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

    # Backrest recline pose check: compare upright vs tilted
    bp_rest = ctx.part_element_world_aabb(backrests[0], elem="backrest_plate_0")
    with ctx.pose({backrest_joints[0]: BACKREST_TILT_MAX}):
        bp_tilted = ctx.part_element_world_aabb(backrests[0], elem="backrest_plate_0")
        ctx.check(
            "backrest_0 top lowers when reclined",
            bp_tilted is not None
            and bp_rest is not None
            and bp_tilted[1][2] < bp_rest[1][2] - 0.003,
            details=f"rest_top={None if bp_rest is None else bp_rest[1][2]}, tilted_top={None if bp_tilted is None else bp_tilted[1][2]}",
        )

    return ctx.report()


object_model = build_object_model()
