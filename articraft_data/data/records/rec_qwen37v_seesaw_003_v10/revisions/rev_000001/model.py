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
PLATE_T = 0.012
HANDLE_X = 1.03

# Asymmetric seat heights: seat 0 (+X) at normal height, seat 1 (-X) higher.
SEAT0_Z = 0.062         # seat 0 plate mid-plane, relative to the pivot
SEAT1_Z = 0.130         # seat 1 plate mid-plane, ~68mm higher

HANDLE0_Z = 0.552
HANDLE1_Z = 0.600       # slightly higher to match the raised seat side

ROCK_LIMIT = 0.262      # ~15 degrees each way
HANDLEBAR_LIMIT = 0.175 # ~10 degrees of handlebar pivot

PEDESTAL_R = 0.075
PEDESTAL_H = 0.22
BRACKET_SIZE = (0.16, 0.13, 0.17)
BRACKET_CZ = 0.295      # bracket box center height (spans 0.21 .. 0.38)

# Support legs: angled tubes from bracket sides down to ground
LEG_R = 0.025           # leg tube radius
PAD_R = 0.055           # rubber pad radius
PAD_H = 0.018           # rubber pad thickness

# Bump stops (on rocker beam underside near each end)
BUMP_SIZE = (0.06, 0.05, 0.04)


def _beam_z(x: float) -> float:
    """Beam centerline height (relative to the pivot frame) at station x."""
    return BEAM_CENTER_Z + CURVE_C * x * x


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="playground_seesaw_v10")

    model.material("gloss_red_orange", rgba=(0.88, 0.20, 0.06, 1.0))
    model.material("matte_black", rgba=(0.09, 0.09, 0.10, 1.0))
    model.material("light_gray", rgba=(0.78, 0.78, 0.79, 1.0))
    model.material("dark_gray_steel", rgba=(0.34, 0.36, 0.38, 1.0))
    model.material("silver_rivet", rgba=(0.74, 0.75, 0.78, 1.0))
    model.material("rubber_black", rgba=(0.12, 0.12, 0.11, 1.0))
    model.material("steel_leg", rgba=(0.55, 0.56, 0.58, 1.0))

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

    # Support legs: angled steel tubes from bracket sides down to ground.
    # Each leg is a swept tube connecting the bracket to a rubber pad.
    leg_end_y = 0.32  # ground-level Y spread
    for i, sy in enumerate((1.0, -1.0)):
        leg_pts = [
            (0.0, sy * 0.065, 0.25),    # top: at bracket Y edge, within bracket Z
            (0.0, sy * 0.14, 0.16),      # mid-upper
            (0.0, sy * 0.24, 0.07),      # mid-lower
            (0.0, sy * leg_end_y, PAD_H),  # bottom: at ground pad
        ]
        base.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    leg_pts, radius=LEG_R, samples_per_segment=8,
                    radial_segments=16, cap_ends=True,
                ),
                f"support_leg_{i}",
            ),
            material="steel_leg",
            name=f"support_leg_{i}",
        )
        # Rubber ground pad at the foot of each leg.
        base.visual(
            Cylinder(radius=PAD_R, length=PAD_H),
            origin=Origin(xyz=(0.0, sy * leg_end_y, PAD_H / 2.0)),
            material="rubber_black",
            name=f"rubber_pad_{i}",
        )

    # -----------------------------------------------------------------
    # Rocker: curved red beam + pivot stub + seat ends + bump stops.
    # Handlebar assemblies are now separate parts.
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

    collar_z = _beam_z(COLLAR_X)
    slope = 2.0 * CURVE_C * COLLAR_X
    tangent = math.atan(slope)

    # Asymmetric seat heights
    seat_z_values = (SEAT0_Z, SEAT1_Z)

    for i, s in enumerate((1.0, -1.0)):
        seat_z = seat_z_values[i]

        # Black clamp collar ring around the beam.
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
            (s * 1.05, 0.0, 0.185 + (seat_z - SEAT0_Z) * 0.3),
            (s * 1.12, 0.0, 0.105 + (seat_z - SEAT0_Z) * 0.6),
            (s * 1.15, 0.0, seat_z + 0.004),
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

        # Safety bump stop on beam underside near each end.
        # Positioned below the beam tube surface, connected to it.
        bump_z = _beam_z(s * 1.05) - BEAM_R - BUMP_SIZE[2] / 2.0
        rocker.visual(
            Box(BUMP_SIZE),
            origin=Origin(xyz=(s * 1.05, 0.0, bump_z)),
            material="rubber_black",
            name=f"bump_stop_{i}",
        )

    # -----------------------------------------------------------------
    # Handlebar assemblies: separate parts that pivot on the rocker.
    # -----------------------------------------------------------------
    grip_outer = rounded_rect_profile(0.18, 0.30, 0.05)
    grip_hole = rounded_rect_profile(0.06, 0.09, 0.02)
    grip_holes = [
        [(hx, hy + 0.075) for hx, hy in grip_hole],
        [(hx, hy - 0.075) for hx, hy in grip_hole],
    ]

    handle_z_values = (HANDLE0_Z, HANDLE1_Z)

    for i, s in enumerate((1.0, -1.0)):
        handle_z = handle_z_values[i]
        hb = model.part(f"handlebar_{i}")

        # Handle post: starts above the beam surface, rises to grip.
        # In handlebar frame: pivot is at (s*COLLAR_X, 0, 0.285) in rocker frame.
        # Beam top at collar is ~0.341, so post bottom must be above that.
        # Post radius is 0.021, so post center must be above 0.341+0.021+0.005=0.367.
        # In handlebar frame that's 0.367 - 0.285 = 0.082.
        post_pts = [
            (0.0, 0.0, 0.085),      # well above beam top surface
            (s * 0.015, 0.0, 0.15),
            (s * 0.030, 0.0, 0.22),
            (s * 0.040, 0.0, 0.285),
        ]
        hb.visual(
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
        grip.translate(s * 0.04, 0.0, 0.285)
        hb.visual(
            mesh_from_geometry(grip, f"handle_plate_{i}"),
            material="dark_gray_steel",
            name=f"handle_plate_{i}",
        )

        # Handlebar pivot joint: attaches handlebar to rocker.
        # Pivot at the collar position on the beam, allowing slight forward/back tilt.
        model.articulation(
            f"handlebar_pivot_{i}",
            ArticulationType.REVOLUTE,
            parent=rocker,
            child=hb,
            origin=Origin(xyz=(s * COLLAR_X, 0.0, 0.285)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=20.0,
                velocity=2.0,
                lower=-HANDLEBAR_LIMIT,
                upper=HANDLEBAR_LIMIT,
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
    hb0 = object_model.get_part("handlebar_0")
    hb1 = object_model.get_part("handlebar_1")
    pivot = object_model.get_articulation("rocker_pivot")
    hb_pivot_0 = object_model.get_articulation("handlebar_pivot_0")
    hb_pivot_1 = object_model.get_articulation("handlebar_pivot_1")

    # --- Handle posts pass through clamp collars ---
    ctx.allow_overlap(
        hb0,
        rocker,
        elem_a="handle_post_0",
        elem_b="clamp_collar_0",
        reason="Handlebar 0 post passes through clamp collar 0 as part of the pivoting handlebar assembly.",
    )
    ctx.allow_overlap(
        hb1,
        rocker,
        elem_a="handle_post_1",
        elem_b="clamp_collar_1",
        reason="Handlebar 1 post passes through clamp collar 1 as part of the pivoting handlebar assembly.",
    )
    # Proof: each handlebar post is properly positioned at its clamp collar.
    ctx.expect_overlap(
        hb0, rocker, axes="xy",
        elem_a="handle_post_0", elem_b="clamp_collar_0",
        min_overlap=0.01,
        name="handlebar 0 post at clamp collar 0",
    )
    ctx.expect_overlap(
        hb1, rocker, axes="xy",
        elem_a="handle_post_1", elem_b="clamp_collar_1",
        min_overlap=0.01,
        name="handlebar 1 post at clamp collar 1",
    )

    # --- Pivot stub seated in bracket ---
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

    # --- Bracket seated on pedestal ---
    bracket = ctx.part_element_world_aabb(base, elem="pivot_bracket")
    pedestal = ctx.part_element_world_aabb(base, elem="ground_pedestal")
    ctx.check(
        "bracket sits atop ground pedestal",
        _intersects(bracket, pedestal),
        details=f"bracket={bracket}, pedestal={pedestal}",
    )

    # --- Beam geometry ---
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

    # --- Overall envelope (including handlebars) ---
    ra = ctx.part_world_aabb(rocker)
    ba = ctx.part_world_aabb(base)
    h0a = ctx.part_world_aabb(hb0)
    h1a = ctx.part_world_aabb(hb1)
    all_tops = [a[1][2] for a in (ra, ba, h0a, h1a) if a is not None]
    ctx.check(
        "overall length about 2.6 m",
        ra is not None and 2.4 <= (ra[1][0] - ra[0][0]) <= 2.8,
        details=f"rocker aabb={ra}",
    )
    ctx.check(
        "overall height about 0.9 m",
        all_tops and 0.75 <= max(all_tops) <= 1.05,
        details=f"max_top={max(all_tops) if all_tops else None}",
    )

    # --- Asymmetric seat heights ---
    seat0 = ctx.part_element_world_aabb(rocker, elem="seat_plate_0")
    seat1 = ctx.part_element_world_aabb(rocker, elem="seat_plate_1")
    ctx.check(
        "seat 0 at sitting height below the beam",
        seat0 is not None and 0.30 <= seat0[1][2] <= 0.50,
        details=f"seat0={seat0}",
    )
    ctx.check(
        "seat 1 at sitting height below the beam",
        seat1 is not None and 0.30 <= seat1[1][2] <= 0.55,
        details=f"seat1={seat1}",
    )
    ctx.check(
        "asymmetric seat heights: seat 1 is higher than seat 0",
        seat0 is not None
        and seat1 is not None
        and seat1[0][2] > seat0[0][2] + 0.03,
        details=f"seat0_min_z={None if seat0 is None else seat0[0][2]}, seat1_min_z={None if seat1 is None else seat1[0][2]}",
    )

    # --- Seats at opposite ends ---
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

    # --- Grip plates above beam ends ---
    grip0 = ctx.part_element_world_aabb(hb0, elem="handle_plate_0")
    grip1 = ctx.part_element_world_aabb(hb1, elem="handle_plate_1")
    ctx.check(
        "grip plates above the beam ends",
        grip0 is not None
        and grip1 is not None
        and beam is not None
        and grip0[0][2] > beam[1][2] - 0.15
        and grip1[0][2] > beam[1][2] - 0.15,
        details=f"grip0={grip0}, grip1={grip1}, beam={beam}",
    )

    # --- Handle posts connect to grip plates ---
    post0 = ctx.part_element_world_aabb(hb0, elem="handle_post_0")
    post1 = ctx.part_element_world_aabb(hb1, elem="handle_post_1")
    ctx.check(
        "handle post 0 connects to grip plate 0",
        _intersects(post0, grip0),
        details=f"post0={post0}, grip0={grip0}",
    )
    ctx.check(
        "handle post 1 connects to grip plate 1",
        _intersects(post1, grip1),
        details=f"post1={post1}, grip1={grip1}",
    )

    # --- Rubber ground pads exist at ground level ---
    pad0 = ctx.part_element_world_aabb(base, elem="rubber_pad_0")
    pad1 = ctx.part_element_world_aabb(base, elem="rubber_pad_1")
    ctx.check(
        "rubber pads exist near ground level",
        pad0 is not None
        and pad1 is not None
        and pad0[0][2] < 0.05
        and pad1[0][2] < 0.05,
        details=f"pad0={pad0}, pad1={pad1}",
    )
    ctx.check(
        "rubber pads spread apart for stability",
        pad0 is not None
        and pad1 is not None
        and abs(pad0[1][1] - pad1[0][1]) > 0.2,
        details=f"pad0={pad0}, pad1={pad1}",
    )

    # --- Support legs connect bracket area to pads ---
    leg0 = ctx.part_element_world_aabb(base, elem="support_leg_0")
    leg1 = ctx.part_element_world_aabb(base, elem="support_leg_1")
    ctx.check(
        "support legs exist and span vertically",
        leg0 is not None
        and leg1 is not None
        and leg0[1][2] > 0.10
        and leg1[1][2] > 0.10,
        details=f"leg0={leg0}, leg1={leg1}",
    )
    ctx.check(
        "support leg 0 reaches toward pad 0",
        leg0 is not None and pad0 is not None and leg0[0][2] < pad0[1][2] + 0.03,
        details=f"leg0={leg0}, pad0={pad0}",
    )
    ctx.check(
        "support leg 1 reaches toward pad 1",
        leg1 is not None and pad1 is not None and leg1[0][2] < pad1[1][2] + 0.03,
        details=f"leg1={leg1}, pad1={pad1}",
    )
    ctx.check(
        "support legs connect to bracket area",
        leg0 is not None and bracket is not None and leg0[1][2] > bracket[0][2] - 0.02
        and leg1 is not None and leg1[1][2] > bracket[0][2] - 0.02,
        details=f"leg0={leg0}, leg1={leg1}, bracket={bracket}",
    )

    # --- Safety bump stops on beam underside ---
    bump0 = ctx.part_element_world_aabb(rocker, elem="bump_stop_0")
    bump1 = ctx.part_element_world_aabb(rocker, elem="bump_stop_1")
    ctx.check(
        "bump stops exist below beam near each end",
        bump0 is not None
        and bump1 is not None
        and bump0[0][0] > 0.7
        and bump1[1][0] < -0.7,
        details=f"bump0={bump0}, bump1={bump1}",
    )
    ctx.check(
        "bump stops are below the beam center",
        bump0 is not None
        and bump1 is not None
        and beam is not None
        and bump0[1][2] < beam[1][2]
        and bump1[1][2] < beam[1][2],
        details=f"bump0={bump0}, bump1={bump1}, beam={beam}",
    )
    ctx.check(
        "bump stops contact beam (connected to rocker)",
        _intersects(bump0, beam) and _intersects(bump1, beam),
        details=f"bump0={bump0}, bump1={bump1}, beam={beam}",
    )

    # --- Handlebar pivot joints with correct limits ---
    lim0 = hb_pivot_0.motion_limits
    lim1 = hb_pivot_1.motion_limits
    ctx.check(
        "handlebar 0 pivot has small revolute range",
        lim0 is not None
        and lim0.lower is not None
        and lim0.upper is not None
        and abs(lim0.lower) > 0.05
        and abs(lim0.upper) > 0.05
        and abs(lim0.upper) < 0.5,
        details=f"limits=({lim0.lower}, {lim0.upper})",
    )
    ctx.check(
        "handlebar 1 pivot has small revolute range",
        lim1 is not None
        and lim1.lower is not None
        and lim1.upper is not None
        and abs(lim1.lower) > 0.05
        and abs(lim1.upper) > 0.05
        and abs(lim1.upper) < 0.5,
        details=f"limits=({lim1.lower}, {lim1.upper})",
    )

    # --- Rocking joint limits ---
    lim = pivot.motion_limits
    ctx.check(
        "rocking range about +/- 15 degrees",
        lim is not None
        and abs(lim.lower + ROCK_LIMIT) < 0.02
        and abs(lim.upper - ROCK_LIMIT) < 0.02,
        details=f"limits=({lim.lower}, {lim.upper})",
    )

    # --- Handlebar pivot motion test: check center displacement ---
    grip0_rest_center = None if grip0 is None else [
        0.5 * (grip0[0][k] + grip0[1][k]) for k in range(3)
    ]
    with ctx.pose({hb_pivot_0: HANDLEBAR_LIMIT}):
        grip0_posed = ctx.part_element_world_aabb(hb0, elem="handle_plate_0")
        if grip0_posed is not None and grip0_rest_center is not None:
            posed_center = [0.5 * (grip0_posed[0][k] + grip0_posed[1][k]) for k in range(3)]
            disp = math.sqrt(sum((posed_center[k] - grip0_rest_center[k])**2 for k in range(3)))
            ctx.check(
                "handlebar 0 pivots: grip center displaces at positive angle",
                disp > 0.005,
                details=f"displacement={disp:.4f}",
            )
        else:
            ctx.fail("handlebar 0 pivots", "missing geometry")

    grip1_rest_center = None if grip1 is None else [
        0.5 * (grip1[0][k] + grip1[1][k]) for k in range(3)
    ]
    with ctx.pose({hb_pivot_1: -HANDLEBAR_LIMIT}):
        grip1_posed = ctx.part_element_world_aabb(hb1, elem="handle_plate_1")
        if grip1_posed is not None and grip1_rest_center is not None:
            posed_center = [0.5 * (grip1_posed[0][k] + grip1_posed[1][k]) for k in range(3)]
            disp = math.sqrt(sum((posed_center[k] - grip1_rest_center[k])**2 for k in range(3)))
            ctx.check(
                "handlebar 1 pivots: grip center displaces at negative angle",
                disp > 0.005,
                details=f"displacement={disp:.4f}",
            )
        else:
            ctx.fail("handlebar 1 pivots", "missing geometry")

    # --- Drop tubes connect beam to seats ---
    drop0 = ctx.part_element_world_aabb(rocker, elem="drop_tube_0")
    drop1 = ctx.part_element_world_aabb(rocker, elem="drop_tube_1")
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
        "clamp collars ring the beam near its ends",
        _intersects(collar0, beam)
        and _intersects(collar1, beam)
        and collar0 is not None
        and collar1 is not None
        and abs(_cx(collar0)) > 0.85
        and abs(_cx(collar1)) > 0.85,
        details=f"collar0={collar0}, collar1={collar1}",
    )

    # --- Decisive rocker pose checks ---
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
