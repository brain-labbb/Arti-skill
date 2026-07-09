from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    ConeGeometry,
    Cylinder,
    CylinderGeometry,
    DomeGeometry,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MeshGeometry,
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
# LOW INCLUSIVE SEESAW: lower pivot height for accessibility.
# ---------------------------------------------------------------------------
PIVOT_Z = 0.34          # world height of the rocking axis (inside the bracket)
BEAM_R = 0.06           # main tube radius (~120 mm diameter)
BEAM_HALF = 1.15        # half-length of the curved main tube
CURVE_C = 0.10          # parabolic curvature (shallower for low profile)
BEAM_CENTER_Z = 0.10    # beam centerline height at x=0, relative to the pivot

COLLAR_X = 0.97         # clamp collar position along the beam
SEAT_CENTER_X = 1.14
SEAT_Z = 0.04           # seat pan mid-plane, relative to the pivot
PLATE_T = 0.012
HANDLE_X = 1.03
HANDLE_Z = 0.38         # handle pivot height (lower for inclusive design)

ROCK_LIMIT = 0.262      # ~15 degrees each way
HANDLE_LIMIT = 0.14     # ~8 degrees handlebar pivot

PEDESTAL_R = 0.075
PEDESTAL_H = 0.20
BRACKET_SIZE = (0.14, 0.12, 0.14)
BRACKET_CZ = 0.27       # bracket box center height

# Seat dimensions for molded bucket seat
SEAT_PAN_W = 0.34       # seat pan width (Y)
SEAT_PAN_D = 0.30       # seat pan depth (X)
SEAT_PAN_T = 0.018      # seat pan thickness
LIP_H = 0.035           # raised lip height
LIP_T = 0.015           # lip wall thickness
BACKREST_H = 0.22       # backrest height
BACKREST_T = 0.016      # backrest thickness
BACKREST_TILT = 0.18    # backrest tilt angle (rad, ~10 degrees)


def _beam_z(x: float) -> float:
    """Beam centerline height (relative to the pivot frame) at station x."""
    return BEAM_CENTER_Z + CURVE_C * x * x


def _build_molded_seat(sign: float) -> MeshGeometry:
    """Build a molded bucket seat with raised lips and a backrest.

    The seat faces toward the center (inboard) so the child faces the pivot.
    sign=+1 means the seat is on the +X end, facing -X; sign=-1 is mirrored.
    """
    geom = MeshGeometry()

    # --- Seat pan: rounded rectangle plate ---
    pan_profile = rounded_rect_profile(SEAT_PAN_D, SEAT_PAN_W, 0.04)
    pan = ExtrudeGeometry(pan_profile, SEAT_PAN_T, cap=True, center=True)
    # Rotate so the pan lies flat (extrusion along Z already)
    # For sign=+1, the back is at +X local, front at -X local
    # We want backrest on the outboard side
    if sign > 0:
        pan.rotate_z(math.pi)
    geom.merge(pan)

    # --- Raised lips: front, left, right edges ---
    # Front lip (inboard edge, toward center)
    front_lip = BoxGeometry((LIP_T, SEAT_PAN_W - 0.04, LIP_H))
    front_lip.translate(sign * (-SEAT_PAN_D / 2.0 + LIP_T / 2.0), 0.0, LIP_H / 2.0 + SEAT_PAN_T / 2.0)
    geom.merge(front_lip)

    # Left lip
    left_lip = BoxGeometry((SEAT_PAN_D - 0.02, LIP_T, LIP_H))
    left_lip.translate(0.0, SEAT_PAN_W / 2.0 - LIP_T / 2.0, LIP_H / 2.0 + SEAT_PAN_T / 2.0)
    geom.merge(left_lip)

    # Right lip
    right_lip = BoxGeometry((SEAT_PAN_D - 0.02, LIP_T, LIP_H))
    right_lip.translate(0.0, -SEAT_PAN_W / 2.0 + LIP_T / 2.0, LIP_H / 2.0 + SEAT_PAN_T / 2.0)
    geom.merge(right_lip)

    # --- Backrest: tilted plate rising from the outboard rear edge ---
    backrest = BoxGeometry((BACKREST_T, SEAT_PAN_W - 0.04, BACKREST_H))
    # Tilt it slightly back from vertical
    if sign > 0:
        backrest.rotate_y(BACKREST_TILT)
        backrest.translate(
            SEAT_PAN_D / 2.0 - 0.01,
            0.0,
            SEAT_PAN_T / 2.0 + BACKREST_H / 2.0 * math.cos(BACKREST_TILT) - 0.02,
        )
    else:
        backrest.rotate_y(-BACKREST_TILT)
        backrest.translate(
            -SEAT_PAN_D / 2.0 + 0.01,
            0.0,
            SEAT_PAN_T / 2.0 + BACKREST_H / 2.0 * math.cos(BACKREST_TILT) - 0.02,
        )
    geom.merge(backrest)

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="playground_seesaw_inclusive")

    model.material("gloss_red_orange", rgba=(0.88, 0.20, 0.06, 1.0))
    model.material("matte_black", rgba=(0.09, 0.09, 0.10, 1.0))
    model.material("light_gray", rgba=(0.78, 0.78, 0.79, 1.0))
    model.material("dark_gray_steel", rgba=(0.34, 0.36, 0.38, 1.0))
    model.material("silver_rivet", rgba=(0.74, 0.75, 0.78, 1.0))
    model.material("seat_mold_gray", rgba=(0.30, 0.32, 0.35, 1.0))
    model.material("axle_cap_yellow", rgba=(0.95, 0.82, 0.10, 1.0))

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
            Cylinder(radius=0.050, length=0.020),
            origin=Origin(xyz=(0.0, sy * 0.0695, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="matte_black",
            name=f"pivot_boss_{i}",
        )
        for j, ang in enumerate((0.25, 0.75, 1.25, 1.75)):
            dx = 0.032 * math.cos(ang * math.pi)
            dz = 0.032 * math.sin(ang * math.pi)
            base.visual(
                Cylinder(radius=0.008, length=0.010),
                origin=Origin(
                    xyz=(dx, sy * 0.082, PIVOT_Z + dz),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material="silver_rivet",
                name=f"bracket_bolt_{i}_{j}",
            )
        # VISIBLE AXLE CAPS: bright yellow safety caps on the axle ends.
        # Cap cylinder overlaps the pivot boss for connectivity.
        cap_cy = sy * 0.088
        base.visual(
            Cylinder(radius=0.032, length=0.024),
            origin=Origin(xyz=(0.0, cap_cy, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="axle_cap_yellow",
            name=f"axle_cap_{i}",
        )
        # Dome cap on the outer face of the axle cap.
        # Overlap dome base slightly into the cap cylinder for connectivity.
        dome = DomeGeometry(0.032, radial_segments=16, height_segments=8, closed=True)
        # Dome base at z=0, extends to z=+0.032. Rotate to face outward along Y.
        if sy > 0:
            dome.rotate_x(-math.pi / 2.0)  # dome faces +Y
        else:
            dome.rotate_x(math.pi / 2.0)   # dome faces -Y
        # Embed dome base 3mm into the cap cylinder for mesh connectivity.
        dome.translate(0.0, sy * 0.097, PIVOT_Z)
        base.visual(
            mesh_from_geometry(dome, f"axle_cap_dome_{i}"),
            material="axle_cap_yellow",
            name=f"axle_cap_dome_{i}",
        )

    # -----------------------------------------------------------------
    # Rocker: curved red beam + pivot stub + mirrored seat assemblies.
    # Handle posts still on the rocker; handlebars are separate parts.
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
    wedge = ConeGeometry(0.080, 0.08, radial_segments=28).rotate_x(math.pi)
    wedge.translate(0.0, 0.0, 0.065)
    rocker.visual(
        mesh_from_geometry(wedge, "pivot_wedge"),
        material="gloss_red_orange",
        name="pivot_wedge",
    )

    # Short red stub descending from the beam into the black bracket.
    rocker.visual(
        Cylinder(radius=0.044, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material="gloss_red_orange",
        name="pivot_stub",
    )

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
            (s * 1.05, 0.0, 0.14),
            (s * 1.12, 0.0, 0.07),
            (s * SEAT_CENTER_X, 0.0, SEAT_Z + 0.02),
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

        # Molded bucket seat with raised lips and backrest.
        seat_geom = _build_molded_seat(s)
        seat_geom.translate(s * SEAT_CENTER_X, 0.0, SEAT_Z)
        rocker.visual(
            mesh_from_geometry(seat_geom, f"molded_seat_{i}"),
            material="seat_mold_gray",
            name=f"molded_seat_{i}",
        )

        # Seat mounting rivets on the drop tube interface
        for j, ly in enumerate((0.08, -0.08)):
            rocker.visual(
                Cylinder(radius=0.008, length=0.010),
                origin=Origin(xyz=(s * SEAT_CENTER_X, ly, SEAT_Z - 0.01)),
                material="silver_rivet",
                name=f"seat_rivet_{i}_{j}",
            )

        # Small black stop fin at the seat nose (foot guard).
        rocker.visual(
            Box((0.040, 0.022, 0.035)),
            origin=Origin(xyz=(s * 1.25, 0.0, SEAT_Z + 0.04)),
            material="matte_black",
            name=f"seat_fin_{i}",
        )

        # Thin red post rising from the beam toward the handlebar pivot point.
        post_pts = [
            (s * COLLAR_X, 0.0, 0.22),
            (s * 0.985, 0.0, 0.32),
            (s * 1.01, 0.0, 0.38),
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

    # -----------------------------------------------------------------
    # Handlebar parts: each pivots slightly on a revolute joint.
    # Frame sits at the top of the handle post; grip extends upward.
    # -----------------------------------------------------------------
    grip_outer = rounded_rect_profile(0.16, 0.26, 0.04)
    grip_hole = rounded_rect_profile(0.05, 0.08, 0.02)
    grip_holes = [
        [(hx, hy + 0.065) for hx, hy in grip_hole],
        [(hx, hy - 0.065) for hx, hy in grip_hole],
    ]

    for i, s in enumerate((1.0, -1.0)):
        hb_name = f"handlebar_{i}"
        hb = model.part(hb_name)

        # Short stem connecting pivot to grip plate.
        # Extend stem through the plate for mesh connectivity.
        hb.visual(
            Cylinder(radius=0.018, length=0.10),
            origin=Origin(xyz=(0.0, 0.0, 0.05)),
            material="gloss_red_orange",
            name=f"handle_stem_{i}",
        )

        # Grip plate with hand cutout holes, stem pokes through for connectivity
        grip = ExtrudeWithHolesGeometry(grip_outer, grip_holes, PLATE_T, cap=True, center=True)
        grip.translate(0.0, 0.0, 0.068)
        hb.visual(
            mesh_from_geometry(grip, f"handle_plate_{i}"),
            material="dark_gray_steel",
            name=f"handle_plate_{i}",
        )

        # Pivot joint: handlebar rotates about the seesaw-length axis (X)
        # at the top of the handle post. Small range for user wobble.
        model.articulation(
            f"handle_pivot_{i}",
            ArticulationType.REVOLUTE,
            parent=rocker,
            child=hb,
            origin=Origin(xyz=(s * HANDLE_X, 0.0, HANDLE_Z - 0.02)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=20.0, velocity=2.0, lower=-HANDLE_LIMIT, upper=HANDLE_LIMIT
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
    hb0 = object_model.get_part("handlebar_0")
    hb1 = object_model.get_part("handlebar_1")
    handle_pivot_0 = object_model.get_articulation("handle_pivot_0")
    handle_pivot_1 = object_model.get_articulation("handle_pivot_1")

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

    # Handlebar stems are intentionally captured at the top of the handle posts
    # as pivot bearings.
    ctx.allow_overlap(
        hb0,
        rocker,
        elem_a="handle_stem_0",
        elem_b="handle_post_0",
        reason="The handlebar stem sits inside the handle post top as a pivot bearing.",
    )
    ctx.allow_overlap(
        hb1,
        rocker,
        elem_a="handle_stem_1",
        elem_b="handle_post_1",
        reason="The handlebar stem sits inside the handle post top as a pivot bearing.",
    )
    ctx.expect_contact(
        hb0,
        rocker,
        elem_a="handle_stem_0",
        elem_b="handle_post_0",
        name="handlebar 0 stem contacts handle post 0",
    )
    ctx.expect_contact(
        hb1,
        rocker,
        elem_a="handle_stem_1",
        elem_b="handle_post_1",
        name="handlebar 1 stem contacts handle post 1",
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
        beam is not None and (beam[1][2] - beam[0][2]) >= 0.18,
        details=f"beam z-range={None if beam is None else beam[1][2] - beam[0][2]}",
    )

    # Overall envelope: about 2.6 m long, lower height for inclusive design.
    ra = ctx.part_world_aabb(rocker)
    ba = ctx.part_world_aabb(base)
    ctx.check(
        "overall length about 2.6 m",
        ra is not None and 2.4 <= (ra[1][0] - ra[0][0]) <= 2.8,
        details=f"rocker aabb={ra}",
    )
    ctx.check(
        "low inclusive height under 0.8 m",
        ra is not None and ba is not None and max(ra[1][2], ba[1][2]) <= 0.80,
        details=f"rocker={ra}, base={ba}",
    )

    # Molded seats with backrests at each end.
    seat0 = ctx.part_element_world_aabb(rocker, elem="molded_seat_0")
    seat1 = ctx.part_element_world_aabb(rocker, elem="molded_seat_1")
    ctx.check(
        "molded seats exist at both ends",
        seat0 is not None and seat1 is not None,
        details=f"seat0={seat0}, seat1={seat1}",
    )
    ctx.check(
        "molded seats have backrest height (taller than a flat plate)",
        seat0 is not None
        and seat1 is not None
        and (seat0[1][2] - seat0[0][2]) > 0.12
        and (seat1[1][2] - seat1[0][2]) > 0.12,
        details=f"seat0 z-range={seat0[1][2]-seat0[0][2] if seat0 else None}, seat1 z-range={seat1[1][2]-seat1[0][2] if seat1 else None}",
    )
    ctx.check(
        "seat pans at low inclusive sitting height",
        seat0 is not None
        and seat1 is not None
        and seat0[0][2] < 0.40
        and seat1[0][2] < 0.40,
        details=f"seat0={seat0}, seat1={seat1}",
    )

    # Seats mirrored about the pivot
    def _cx(aabb):
        return 0.5 * (aabb[0][0] + aabb[1][0])

    ctx.check(
        "seat assemblies mirrored about the pivot",
        seat0 is not None
        and seat1 is not None
        and _cx(seat0) > 0.9
        and _cx(seat1) < -0.9
        and abs(_cx(seat0) + _cx(seat1)) < 0.03
        and abs(seat0[1][2] - seat1[1][2]) < 0.02,
        details=f"seat0={seat0}, seat1={seat1}",
    )

    # Visible axle caps at the support bracket.
    axle_cap_0 = ctx.part_element_world_aabb(base, elem="axle_cap_0")
    axle_cap_1 = ctx.part_element_world_aabb(base, elem="axle_cap_1")
    ctx.check(
        "visible axle caps on both sides of bracket",
        axle_cap_0 is not None
        and axle_cap_1 is not None
        and axle_cap_0[0][1] > 0.06
        and axle_cap_1[1][1] < -0.06,
        details=f"axle_cap_0={axle_cap_0}, axle_cap_1={axle_cap_1}",
    )

    # Handlebar parts exist and are mounted near the beam ends.
    grip0 = ctx.part_element_world_aabb(hb0, elem="handle_plate_0")
    grip1 = ctx.part_element_world_aabb(hb1, elem="handle_plate_1")
    ctx.check(
        "handlebar grip plates exist above the beam",
        grip0 is not None
        and grip1 is not None
        and beam is not None
        and grip0[0][2] > beam[1][2] - 0.05
        and grip1[0][2] > beam[1][2] - 0.05,
        details=f"grip0={grip0}, grip1={grip1}, beam={beam}",
    )

    # Handlebar pivots are non-fixed revolute joints with proper limits.
    hp0_lim = handle_pivot_0.motion_limits
    hp1_lim = handle_pivot_1.motion_limits
    ctx.check(
        "handlebar pivot 0 has non-zero range",
        hp0_lim is not None
        and hp0_lim.lower < 0.0
        and hp0_lim.upper > 0.0,
        details=f"limits=({hp0_lim.lower}, {hp0_lim.upper})" if hp0_lim else "no limits",
    )
    ctx.check(
        "handlebar pivot 1 has non-zero range",
        hp1_lim is not None
        and hp1_lim.lower < 0.0
        and hp1_lim.upper > 0.0,
        details=f"limits=({hp1_lim.lower}, {hp1_lim.upper})" if hp1_lim else "no limits",
    )

    # Mounted, not floating: drop tubes reach seats, posts reach handlebars.
    drop0 = ctx.part_element_world_aabb(rocker, elem="drop_tube_0")
    drop1 = ctx.part_element_world_aabb(rocker, elem="drop_tube_1")
    post0 = ctx.part_element_world_aabb(rocker, elem="handle_post_0")
    post1 = ctx.part_element_world_aabb(rocker, elem="handle_post_1")
    ctx.check(
        "drop tubes connect beam to seats",
        _intersects(drop0, beam)
        and _intersects(drop0, seat0)
        and _intersects(drop1, beam)
        and _intersects(drop1, seat1),
        details=f"drop0={drop0}, drop1={drop1}",
    )
    ctx.check(
        "handle posts connect beam toward handlebars",
        _intersects(post0, beam)
        and _intersects(post1, beam),
        details=f"post0={post0}, post1={post1}",
    )

    # Handlebar parts connect near handle post tops
    stem0 = ctx.part_element_world_aabb(hb0, elem="handle_stem_0")
    stem1 = ctx.part_element_world_aabb(hb1, elem="handle_stem_1")
    ctx.check(
        "handlebar stems overlap with handle posts (pivot mounting)",
        _intersects(stem0, post0) and _intersects(stem1, post1),
        details=f"stem0={stem0}, post0={post0}, stem1={stem1}, post1={post1}",
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

    # Decisive pose checks: rocker tilts, handlebars can pivot independently.
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
            base_rest is not None
            and base_posed is not None
            and _intersects(base_rest, base_posed)
            and abs(base_rest[1][2] - base_posed[1][2]) < 1e-6,
            details=f"rest={base_rest}, posed={base_posed}",
        )

    # Handlebar pivot pose: confirm the handlebar actually moves.
    grip0_rest = ctx.part_element_world_aabb(hb0, elem="handle_plate_0")
    with ctx.pose({handle_pivot_0: HANDLE_LIMIT}):
        grip0_posed = ctx.part_element_world_aabb(hb0, elem="handle_plate_0")
        ctx.check(
            "handlebar 0 pivots (grip plate moves)",
            grip0_rest is not None
            and grip0_posed is not None
            and abs(grip0_posed[0][1] - grip0_rest[0][1]) > 0.002,
            details=f"rest={grip0_rest}, posed={grip0_posed}",
        )

    return ctx.report()


object_model = build_object_model()
