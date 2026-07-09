from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
    sample_catmull_rom_spline_2d,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Shared dimensions (meters). World: X along the seesaw length, Z up.
# Variant 27: low inclusive seesaw with backrest seats and pivoting handlebars.
# ---------------------------------------------------------------------------
PIVOT_Z = 0.30          # world height of the rocking axis (inside the bracket)
BEAM_R = 0.06           # main tube radius (~120 mm diameter)
BEAM_HALF = 1.15        # half-length of the curved main tube
CURVE_C = 0.1285        # parabolic curvature of the banana beam
BEAM_CENTER_Z = 0.14    # beam centerline height at x=0, relative to the pivot

COLLAR_X = 0.97         # clamp collar position along the beam
SEAT_CENTER_X = 1.08    # seat center (moved inward for ground clearance)
SEAT_Z = 0.06           # seat pan bottom, relative to the pivot

# Handlebar base must clear both the beam top and the clamp collar at the
# collar position. Collar top ≈ collar_z + 0.080 ≈ 0.341.
HANDLEBAR_BASE_Z = 0.36  # handlebar pivot height in rocker frame (above collar)
HANDLE_TOP_Z = 0.19      # post height in handlebar local frame
HANDLE_X_OFF = 0.05      # post X offset in handlebar local frame

ROCK_LIMIT = 0.262       # ~15 degrees each way
HANDLEBAR_TILT = 0.15    # ~8.6 degrees each way

PEDESTAL_R = 0.075
PEDESTAL_H = 0.20
BRACKET_SIZE = (0.16, 0.13, 0.15)
BRACKET_CZ = PEDESTAL_H + BRACKET_SIZE[2] / 2.0  # bracket box center height


def _beam_z(x: float) -> float:
    """Beam centerline height (relative to the pivot frame) at station x."""
    return BEAM_CENTER_Z + CURVE_C * x * x


def _build_molded_seat_cq() -> cq.Workplane:
    """Molded bucket seat, centered at origin, backrest at +X. Bottom at z=0.

    Features:
    - flat seat pan
    - raised side lips (left/right)
    - raised front lip (-X side, faces the pivot/center)
    - tall backrest panel (+X side, behind the rider)
    """
    pw, pd, pt = 0.26, 0.25, 0.015   # pan width, depth, thickness
    lh, lt = 0.038, 0.013            # lip height, thickness
    bh, bt = 0.21, 0.013             # backrest height, thickness

    # Pan: bottom at z=0
    pan = cq.Workplane("XY").box(pw, pd, pt, centered=(True, True, False))

    # Side lip at +Y (raised wall on top of pan edge)
    lip_yp = (
        cq.Workplane("XY")
        .workplane(offset=pt)
        .transformed(offset=(0.0, pd / 2.0 - lt / 2.0, 0.0))
        .box(pw - 2 * lt, lt, lh, centered=(True, True, False))
    )
    # Side lip at -Y
    lip_yn = (
        cq.Workplane("XY")
        .workplane(offset=pt)
        .transformed(offset=(0.0, -(pd / 2.0 - lt / 2.0), 0.0))
        .box(pw - 2 * lt, lt, lh, centered=(True, True, False))
    )
    # Front lip at -X (faces toward the center/pivot)
    lip_xn = (
        cq.Workplane("XY")
        .workplane(offset=pt)
        .transformed(offset=(-(pw / 2.0 - lt / 2.0), 0.0, 0.0))
        .box(lt, pd - 2 * lt, lh, centered=(True, True, False))
    )
    # Backrest at +X (behind the rider, tall panel)
    back = (
        cq.Workplane("XY")
        .transformed(offset=(pw / 2.0 - bt / 2.0, 0.0, 0.0))
        .box(bt, pd - 2 * lt, bh, centered=(True, True, False))
    )

    result = pan.union(lip_yp).union(lip_yn).union(lip_xn).union(back)
    return result


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="playground_seesaw_low_inclusive")

    model.material("gloss_red_orange", rgba=(0.88, 0.20, 0.06, 1.0))
    model.material("matte_black", rgba=(0.09, 0.09, 0.10, 1.0))
    model.material("light_gray", rgba=(0.78, 0.78, 0.79, 1.0))
    model.material("dark_gray_steel", rgba=(0.30, 0.32, 0.35, 1.0))
    model.material("silver_rivet", rgba=(0.74, 0.75, 0.78, 1.0))
    model.material("seat_green", rgba=(0.18, 0.45, 0.22, 1.0))

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

    # -----------------------------------------------------------------
    # Rocker: curved red beam + pivot stub + mirrored seat ends +
    # handle pivot stubs. Handlebars are separate articulated parts.
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
    wedge.translate(0.0, 0.0, 0.095)
    rocker.visual(
        mesh_from_geometry(wedge, "pivot_wedge"),
        material="gloss_red_orange",
        name="pivot_wedge",
    )

    # Short red stub descending from the beam into the black bracket.
    rocker.visual(
        Cylinder(radius=0.048, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material="gloss_red_orange",
        name="pivot_stub",
    )

    # Grip plate profile (shared between both handlebars).
    grip_outer = rounded_rect_profile(0.16, 0.26, 0.045)
    grip_hole = rounded_rect_profile(0.055, 0.08, 0.018)
    grip_holes = [
        [(hx, hy + 0.065) for hx, hy in grip_hole],
        [(hx, hy - 0.065) for hx, hy in grip_hole],
    ]

    collar_z = _beam_z(COLLAR_X)
    slope = 2.0 * CURVE_C * COLLAR_X
    tangent = math.atan(slope)

    for i, s in enumerate((1.0, -1.0)):
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
        # The endpoint penetrates the seat pan slightly for mesh connectivity.
        drop_pts = [
            (s * COLLAR_X, 0.0, collar_z),
            (s * 1.02, 0.0, 0.17),
            (s * 1.06, 0.0, 0.11),
            (s * SEAT_CENTER_X, 0.0, SEAT_Z),
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

        # --- Molded bucket seat with raised lips and backrest (CadQuery) ---
        seat_shape = _build_molded_seat_cq()
        if s < 0:
            seat_shape = seat_shape.rotate((0, 0, 0), (0, 0, 1), 180)
        seat_shape = seat_shape.translate((s * SEAT_CENTER_X, 0.0, SEAT_Z))
        rocker.visual(
            mesh_from_cadquery(seat_shape, f"molded_seat_{i}"),
            material="seat_green",
            name=f"molded_seat_{i}",
        )

        # --- Handlebar pivot bracket on the rocker ---
        # Extends from the clamp collar upward to the handlebar pivot base,
        # providing both visual support and mesh connectivity.
        bracket_bot = collar_z  # overlaps with clamp collar
        bracket_top = HANDLEBAR_BASE_Z + 0.042
        bracket_len = bracket_top - bracket_bot
        rocker.visual(
            Cylinder(radius=0.022, length=bracket_len),
            origin=Origin(xyz=(s * COLLAR_X, 0.0, bracket_bot + bracket_len / 2.0)),
            material="matte_black",
            name=f"handle_stub_{i}",
        )

    # -----------------------------------------------------------------
    # Handlebars: separate parts, each pivoting on a revolute joint.
    # -----------------------------------------------------------------
    PLATE_T = 0.012
    handlebar_parts = []
    for i, s in enumerate((1.0, -1.0)):
        hbar = model.part(f"handlebar_{i}")
        handlebar_parts.append(hbar)

        # Pivot sleeve (black ring wrapping around the rocker stub).
        hbar.visual(
            Cylinder(radius=0.025, length=0.042),
            origin=Origin(xyz=(0.0, 0.0, 0.021)),
            material="matte_black",
            name="pivot_sleeve",
        )

        # Red handle post tube rising from the pivot to the grip.
        post_pts = [
            (0.0, 0.0, 0.042),
            (s * 0.012, 0.0, 0.085),
            (s * 0.030, 0.0, 0.135),
            (s * HANDLE_X_OFF, 0.0, HANDLE_TOP_Z),
        ]
        hbar.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    post_pts, radius=0.018, samples_per_segment=8, radial_segments=16
                ),
                "handle_post",
            ),
            material="gloss_red_orange",
            name="handle_post",
        )

        # Gray grip plate with two hand cutout holes.
        grip = ExtrudeWithHolesGeometry(
            grip_outer, grip_holes, PLATE_T, cap=True, center=True
        )
        grip.translate(s * HANDLE_X_OFF, 0.0, HANDLE_TOP_Z)
        hbar.visual(
            mesh_from_geometry(grip, "grip_plate"),
            material="dark_gray_steel",
            name="grip_plate",
        )

    # -----------------------------------------------------------------
    # Articulations
    # -----------------------------------------------------------------

    # Main rocking pivot: horizontal axis across the seesaw length.
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

    # Handlebar pivot joints: each handlebar tilts slightly forward/backward.
    for i, s in enumerate((1.0, -1.0)):
        model.articulation(
            f"handlebar_{i}_tilt",
            ArticulationType.REVOLUTE,
            parent=rocker,
            child=handlebar_parts[i],
            origin=Origin(xyz=(s * COLLAR_X, 0.0, HANDLEBAR_BASE_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=20.0, velocity=2.0,
                lower=-HANDLEBAR_TILT, upper=HANDLEBAR_TILT,
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

    hbar0 = object_model.get_part("handlebar_0")
    hbar1 = object_model.get_part("handlebar_1")
    htilt0 = object_model.get_articulation("handlebar_0_tilt")
    htilt1 = object_model.get_articulation("handlebar_1_tilt")

    # --- Main pivot: stub captured inside bracket ---
    ctx.allow_overlap(
        rocker, base,
        elem_a="pivot_stub", elem_b="pivot_bracket",
        reason="The red center stub descends into the cast pivot bracket that captures the rocking axle.",
    )
    ctx.expect_overlap(
        rocker, base, axes="z",
        elem_a="pivot_stub", elem_b="pivot_bracket",
        min_overlap=0.03,
        name="pivot stub inserted into bracket",
    )
    ctx.expect_within(
        rocker, base, axes="xy",
        inner_elem="pivot_stub", outer_elem="pivot_bracket",
        margin=0.0,
        name="pivot stub centered in bracket",
    )

    # --- Handlebar pivot sleeves wrap around rocker stubs ---
    for i in range(2):
        hbar = object_model.get_part(f"handlebar_{i}")
        ctx.allow_overlap(
            hbar, rocker,
            elem_a="pivot_sleeve", elem_b=f"handle_stub_{i}",
            reason=f"Handlebar {i} pivot sleeve wraps around the rocker stub as a captured pivot bearing.",
        )
        ctx.expect_overlap(
            hbar, rocker, axes="z",
            elem_a="pivot_sleeve", elem_b=f"handle_stub_{i}",
            min_overlap=0.02,
            name=f"handlebar_{i} sleeve captures rocker stub",
        )

    # --- Bracket seated on pedestal ---
    bracket = ctx.part_element_world_aabb(base, elem="pivot_bracket")
    pedestal = ctx.part_element_world_aabb(base, elem="ground_pedestal")
    ctx.check(
        "bracket sits atop ground pedestal",
        _intersects(bracket, pedestal),
        details=f"bracket={bracket}, pedestal={pedestal}",
    )

    # --- Beam: ~2.3 m long banana tube ---
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

    # --- Low inclusive overall envelope ---
    ra = ctx.part_world_aabb(rocker)
    ba = ctx.part_world_aabb(base)
    ctx.check(
        "overall length about 2.4 m or more",
        ra is not None and (ra[1][0] - ra[0][0]) >= 2.2,
        details=f"rocker aabb={ra}",
    )
    # Compute overall height including handlebars
    hbar0_aabb = ctx.part_world_aabb(hbar0)
    hbar1_aabb = ctx.part_world_aabb(hbar1)
    all_top = max(
        ra[1][2] if ra else 0,
        ba[1][2] if ba else 0,
        hbar0_aabb[1][2] if hbar0_aabb else 0,
        hbar1_aabb[1][2] if hbar1_aabb else 0,
    )
    ctx.check(
        "low inclusive overall height below 0.88 m",
        all_top <= 0.88,
        details=f"top={all_top:.3f}",
    )

    # --- Molded seats: at sitting height, with raised lips and backrest ---
    seat0 = ctx.part_element_world_aabb(rocker, elem="molded_seat_0")
    seat1 = ctx.part_element_world_aabb(rocker, elem="molded_seat_1")
    ctx.check(
        "molded seats at low accessible sitting height",
        seat0 is not None and seat1 is not None
        and seat0[0][2] < 0.45 and seat1[0][2] < 0.45,
        details=f"seat0={seat0}, seat1={seat1}",
    )

    # Raised lips and backrest: seat Z extent should be tall (pan + lips + backrest).
    ctx.check(
        "molded seat_0 has tall backrest and raised lips (z-extent >= 0.18)",
        seat0 is not None and (seat0[1][2] - seat0[0][2]) >= 0.18,
        details=f"seat0 z-extent={None if seat0 is None else seat0[1][2] - seat0[0][2]:.3f}",
    )
    ctx.check(
        "molded seat_1 has tall backrest and raised lips (z-extent >= 0.18)",
        seat1 is not None and (seat1[1][2] - seat1[0][2]) >= 0.18,
        details=f"seat1 z-extent={None if seat1 is None else seat1[1][2] - seat1[0][2]:.3f}",
    )

    # --- Seats at opposite ends, mirrored ---
    def _cx(aabb):
        return 0.5 * (aabb[0][0] + aabb[1][0])

    ctx.check(
        "seat assemblies mirrored about the pivot",
        seat0 is not None and seat1 is not None
        and _cx(seat0) > 0.8 and _cx(seat1) < -0.8
        and abs(_cx(seat0) + _cx(seat1)) < 0.05
        and abs(seat0[1][2] - seat1[1][2]) < 0.02,
        details=f"seat0={seat0}, seat1={seat1}",
    )

    # --- Drop tubes connect beam to seats ---
    drop0 = ctx.part_element_world_aabb(rocker, elem="drop_tube_0")
    drop1 = ctx.part_element_world_aabb(rocker, elem="drop_tube_1")
    ctx.check(
        "drop tubes connect beam to seats",
        _intersects(drop0, beam) and _intersects(drop0, seat0)
        and _intersects(drop1, beam) and _intersects(drop1, seat1),
        details=f"drop0={drop0}, drop1={drop1}",
    )

    # --- Handlebar articulations exist with correct limits ---
    lim0 = htilt0.motion_limits
    lim1 = htilt1.motion_limits
    ctx.check(
        "handlebar_0 tilt range is symmetric and small",
        lim0 is not None
        and abs(lim0.lower + HANDLEBAR_TILT) < 0.02
        and abs(lim0.upper - HANDLEBAR_TILT) < 0.02,
        details=f"limits=({lim0.lower}, {lim0.upper})",
    )
    ctx.check(
        "handlebar_1 tilt range is symmetric and small",
        lim1 is not None
        and abs(lim1.lower + HANDLEBAR_TILT) < 0.02
        and abs(lim1.upper - HANDLEBAR_TILT) < 0.02,
        details=f"limits=({lim1.lower}, {lim1.upper})",
    )

    # --- Handlebar grip plates are above the beam ---
    grip0 = ctx.part_element_world_aabb(hbar0, elem="grip_plate")
    grip1 = ctx.part_element_world_aabb(hbar1, elem="grip_plate")
    ctx.check(
        "handlebar_0 grip plate above the beam",
        grip0 is not None and beam is not None and grip0[0][2] > beam[1][2] - 0.05,
        details=f"grip0={grip0}, beam={beam}",
    )
    ctx.check(
        "handlebar_1 grip plate above the beam",
        grip1 is not None and beam is not None and grip1[0][2] > beam[1][2] - 0.05,
        details=f"grip1={grip1}, beam={beam}",
    )

    # --- Handlebar posts connect pivot base to grip ---
    post0 = ctx.part_element_world_aabb(hbar0, elem="handle_post")
    post1 = ctx.part_element_world_aabb(hbar1, elem="handle_post")
    sleeve0 = ctx.part_element_world_aabb(hbar0, elem="pivot_sleeve")
    sleeve1 = ctx.part_element_world_aabb(hbar1, elem="pivot_sleeve")
    ctx.check(
        "handlebar_0 post connects sleeve to grip",
        post0 is not None and grip0 is not None and sleeve0 is not None
        and _intersects(post0, grip0) and _intersects(post0, sleeve0),
        details=f"post0={post0}, grip0={grip0}, sleeve0={sleeve0}",
    )
    ctx.check(
        "handlebar_1 post connects sleeve to grip",
        post1 is not None and grip1 is not None and sleeve1 is not None
        and _intersects(post1, grip1) and _intersects(post1, sleeve1),
        details=f"post1={post1}, grip1={grip1}, sleeve1={sleeve1}",
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

    # --- Pose: rocker tilts, handlebars tilt independently ---
    base_rest = ctx.part_world_aabb(base)
    with ctx.pose({pivot: ROCK_LIMIT}):
        seat0_dn = ctx.part_element_world_aabb(rocker, elem="molded_seat_0")
        seat1_up = ctx.part_element_world_aabb(rocker, elem="molded_seat_1")
        rocker_dn = ctx.part_world_aabb(rocker)
        base_posed = ctx.part_world_aabb(base)
        ctx.check(
            "positive rock lowers seat_0 and raises seat_1",
            seat0_dn is not None and seat1_up is not None
            and seat0 is not None and seat1 is not None
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
            base_rest is not None and base_posed is not None
            and abs(base_rest[1][2] - base_posed[1][2]) < 1e-6,
            details=f"rest={base_rest}, posed={base_posed}",
        )

    # Handlebar tilt pose check
    grip0_rest = ctx.part_element_world_aabb(hbar0, elem="grip_plate")
    with ctx.pose({htilt0: HANDLEBAR_TILT}):
        grip0_tilted = ctx.part_element_world_aabb(hbar0, elem="grip_plate")
        ctx.check(
            "handlebar_0 tilts at positive limit",
            grip0_rest is not None and grip0_tilted is not None
            and abs(grip0_tilted[0][0] - grip0_rest[0][0]) > 0.003,
            details=f"rest={grip0_rest}, tilted={grip0_tilted}",
        )

    with ctx.pose({pivot: -ROCK_LIMIT}):
        seat0_up = ctx.part_element_world_aabb(rocker, elem="molded_seat_0")
        rocker_up = ctx.part_world_aabb(rocker)
        ctx.check(
            "negative rock raises seat_0",
            seat0_up is not None and seat0 is not None
            and seat0_up[0][2] > seat0[0][2] + 0.10,
            details=f"seat0_up={seat0_up}",
        )
        ctx.check(
            "rocker clears the ground at opposite tilt",
            rocker_up is not None and rocker_up[0][2] > 0.005,
            details=f"rocker={rocker_up}",
        )

    return ctx.report()


object_model = build_object_model()
