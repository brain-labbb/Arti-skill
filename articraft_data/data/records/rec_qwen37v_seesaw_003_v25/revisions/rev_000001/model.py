from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ConeGeometry,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    LoftGeometry,
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
PIVOT_Z = 0.34          # world height of the rocking axis
BEAM_HALF = 1.15        # half-length of the main beam
CURVE_C = 0.1285        # parabolic curvature
BEAM_CENTER_Z = 0.16    # beam centerline height at x=0

# Heavy rectangular box-section beam: 80 mm wide × 120 mm deep
BEAM_W = 0.08
BEAM_D = 0.12
BEAM_CR = 0.010         # corner radius

COLLAR_X = 0.97
SEAT_CENTER_X = 1.14
SEAT_Z = 0.062
PLATE_T = 0.012
HANDLE_X = 1.03
HANDLE_Z = 0.552

ROCK_LIMIT = 0.262      # ~15 degrees
BACKREST_LIMIT = 0.175  # ~10 degrees

PEDESTAL_R = 0.075
PEDESTAL_H = 0.22
BRACKET_SIZE = (0.16, 0.13, 0.17)
BRACKET_CZ = 0.295


def _beam_z(x: float) -> float:
    """Beam centerline height (relative to the pivot frame) at station x."""
    return BEAM_CENTER_Z + CURVE_C * x * x


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="playground_seesaw_heavy")

    model.material("steel_beam", rgba=(0.42, 0.44, 0.47, 1.0))
    model.material("matte_black", rgba=(0.09, 0.09, 0.10, 1.0))
    model.material("light_gray", rgba=(0.78, 0.78, 0.79, 1.0))
    model.material("dark_gray_steel", rgba=(0.34, 0.36, 0.38, 1.0))
    model.material("silver_rivet", rgba=(0.74, 0.75, 0.78, 1.0))
    model.material("rubber_black", rgba=(0.05, 0.05, 0.06, 1.0))
    model.material("seat_mold", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("axle_steel", rgba=(0.60, 0.62, 0.65, 1.0))
    model.material("backrest_mold", rgba=(0.20, 0.22, 0.25, 1.0))

    # -----------------------------------------------------------------
    # Fixed base: pedestal + bracket + axle caps.
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
    for i, sy in enumerate((1.0, -1.0)):
        # Pivot boss on bracket cheek
        base.visual(
            Cylinder(radius=0.055, length=0.022),
            origin=Origin(xyz=(0.0, sy * 0.0755, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="matte_black",
            name=f"pivot_boss_{i}",
        )
        # Visible axle cap — polished steel disc proud of the bracket cheek
        base.visual(
            Cylinder(radius=0.042, length=0.014),
            origin=Origin(xyz=(0.0, sy * 0.090, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="axle_steel",
            name=f"axle_cap_{i}",
        )
        # Axle cap center hub
        base.visual(
            Cylinder(radius=0.014, length=0.018),
            origin=Origin(xyz=(0.0, sy * 0.094, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="silver_rivet",
            name=f"axle_hub_{i}",
        )
        # Bracket bolts
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
    # Rocker: heavy steel box beam + pivot stub + mirrored end assemblies.
    # -----------------------------------------------------------------
    rocker = model.part("rocker")

    # Heavy box-section steel beam built from segmented box primitives
    # placed and rotated along the parabolic curve.
    n_seg = 10
    seg_len = 2.0 * BEAM_HALF / n_seg * 1.04  # slight overlap for continuity
    for k in range(n_seg):
        x_lo = -BEAM_HALF + (2.0 * BEAM_HALF) * k / n_seg
        x_mid = -BEAM_HALF + (2.0 * BEAM_HALF) * (k + 0.5) / n_seg
        z_mid = _beam_z(x_mid)
        slope = 2.0 * CURVE_C * x_mid
        angle = math.atan(slope)
        rocker.visual(
            Box((seg_len, BEAM_W, BEAM_D)),
            origin=Origin(
                xyz=(x_mid, 0.0, z_mid),
                rpy=(0.0, angle, 0.0),
            ),
            material="steel_beam",
            name=f"beam_seg_{k}",
        )
    # Top and bottom flange plates for box-section look
    for k in range(n_seg):
        x_mid = -BEAM_HALF + (2.0 * BEAM_HALF) * (k + 0.5) / n_seg
        z_mid = _beam_z(x_mid)
        slope = 2.0 * CURVE_C * x_mid
        angle = math.atan(slope)
        # Top flange
        rocker.visual(
            Box((seg_len * 0.95, BEAM_W + 0.008, 0.006)),
            origin=Origin(
                xyz=(x_mid - math.sin(angle) * BEAM_D * 0.5, 0.0, z_mid + math.cos(angle) * BEAM_D * 0.5),
                rpy=(0.0, angle, 0.0),
            ),
            material="steel_beam",
            name=f"beam_top_flange_{k}",
        )
        # Bottom flange
        rocker.visual(
            Box((seg_len * 0.95, BEAM_W + 0.008, 0.006)),
            origin=Origin(
                xyz=(x_mid + math.sin(angle) * BEAM_D * 0.5, 0.0, z_mid - math.cos(angle) * BEAM_D * 0.5),
                rpy=(0.0, angle, 0.0),
            ),
            material="steel_beam",
            name=f"beam_bot_flange_{k}",
        )

    # Pivot stub descending into bracket
    rocker.visual(
        Cylinder(radius=0.048, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material="steel_beam",
        name="pivot_stub",
    )
    # Transition wedge
    wedge = ConeGeometry(0.085, 0.09, radial_segments=28).rotate_x(math.pi)
    wedge.translate(0.0, 0.0, 0.110)
    rocker.visual(
        mesh_from_geometry(wedge, "pivot_wedge"),
        material="steel_beam",
        name="pivot_wedge",
    )

    # Rubber bumpers under beam near each end (cushion stops)
    for i, s in enumerate((1.0, -1.0)):
        bx = s * 0.82
        bz = _beam_z(bx)
        rocker.visual(
            Cylinder(radius=0.038, length=0.065),
            origin=Origin(xyz=(bx, 0.0, bz - BEAM_D / 2.0 - 0.032)),
            material="rubber_black",
            name=f"rubber_bumper_{i}",
        )
        # Bumper mounting plate
        rocker.visual(
            Box((0.06, 0.06, 0.006)),
            origin=Origin(xyz=(bx, 0.0, bz - BEAM_D / 2.0 - 0.003)),
            material="dark_gray_steel",
            name=f"bumper_plate_{i}",
        )

    # End assemblies
    collar_z = _beam_z(COLLAR_X)
    slope = 2.0 * CURVE_C * COLLAR_X
    tangent = math.atan(slope)

    seat_pan_profile = rounded_rect_profile(0.30, 0.28, 0.04)
    lip_t = 0.008
    lip_h = 0.032

    grip_outer = rounded_rect_profile(0.18, 0.30, 0.05)
    grip_hole = rounded_rect_profile(0.06, 0.09, 0.02)
    grip_holes = [
        [(hx, hy + 0.075) for hx, hy in grip_hole],
        [(hx, hy - 0.075) for hx, hy in grip_hole],
    ]

    for i, s in enumerate((1.0, -1.0)):
        # Clamp collar
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

        # Drop tube from collar to seat
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
            material="steel_beam",
            name=f"drop_tube_{i}",
        )

        # Molded seat pan (sitting surface)
        seat_pan = ExtrudeGeometry(seat_pan_profile, PLATE_T + 0.004, cap=True, center=True)
        if s < 0:
            seat_pan.rotate_z(math.pi)
        seat_pan.translate(s * SEAT_CENTER_X, 0.0, SEAT_Z)
        rocker.visual(
            mesh_from_geometry(seat_pan, f"seat_pan_{i}"),
            material="seat_mold",
            name=f"seat_pan_{i}",
        )

        # Raised lips around seat edges (molded bucket detail)
        # Front lip (outboard)
        rocker.visual(
            Box((lip_t, 0.24, lip_h)),
            origin=Origin(xyz=(s * (SEAT_CENTER_X + 0.146), 0.0, SEAT_Z + lip_h / 2.0)),
            material="seat_mold",
            name=f"seat_lip_front_{i}",
        )
        # Side lips
        for j, sy in enumerate((1.0, -1.0)):
            rocker.visual(
                Box((0.26, lip_t, lip_h)),
                origin=Origin(xyz=(s * SEAT_CENTER_X, sy * 0.136, SEAT_Z + lip_h / 2.0)),
                material="seat_mold",
                name=f"seat_lip_side_{i}_{j}",
            )

        # Seat rivets
        rivet_xy = [(0.10, 0.0), (0.0, 0.10), (0.0, -0.10), (-0.10, 0.07), (-0.10, -0.07)]
        for j, (lx, ly) in enumerate(rivet_xy):
            rocker.visual(
                Cylinder(radius=0.008, length=0.010),
                origin=Origin(xyz=(s * (SEAT_CENTER_X + lx), ly, SEAT_Z + 0.010)),
                material="silver_rivet",
                name=f"seat_rivet_{i}_{j}",
            )

        # Hinge lug on rocker for backrest connection (small pin)
        hinge_x = s * (SEAT_CENTER_X + 0.170)
        hinge_z = SEAT_Z + 0.025
        rocker.visual(
            Cylinder(radius=0.012, length=0.05),
            origin=Origin(xyz=(hinge_x, 0.0, hinge_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="matte_black",
            name=f"backrest_lug_{i}",
        )
        # Gusset plate bridging lug to seat back edge (structural mount)
        gusset_cx = s * (SEAT_CENTER_X + 0.150)
        rocker.visual(
            Box((0.030, 0.035, 0.008)),
            origin=Origin(xyz=(gusset_cx, 0.0, hinge_z)),
            material="steel_beam",
            name=f"backrest_gusset_{i}",
        )

        # Handle post
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
            material="steel_beam",
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
    # Backrest parts — separate for tilt articulation on revolute joints.
    # -----------------------------------------------------------------
    for i, s in enumerate((1.0, -1.0)):
        br = model.part(f"backrest_{i}")

        hinge_x = s * (SEAT_CENTER_X + 0.170)
        hinge_z = SEAT_Z + 0.025

        # All backrest visuals are relative to the backrest part frame,
        # which sits at the tilt joint origin (hinge_x, 0, hinge_z) in rocker coords.
        # So local offsets: (0, 0, +dz) means "above the hinge pin".

        # Backrest plate (molded panel, vertical)
        br.visual(
            Box((0.012, 0.24, 0.20)),
            origin=Origin(xyz=(0.0, 0.0, 0.12)),
            material="backrest_mold",
            name=f"backrest_plate_{i}",
        )
        # Top edge lip
        br.visual(
            Box((0.016, 0.24, 0.014)),
            origin=Origin(xyz=(0.0, 0.0, 0.225)),
            material="backrest_mold",
            name=f"backrest_top_lip_{i}",
        )
        # Side edge lips
        for j, sy in enumerate((1.0, -1.0)):
            br.visual(
                Box((0.016, 0.014, 0.20)),
                origin=Origin(xyz=(0.0, sy * 0.12, 0.12)),
                material="backrest_mold",
                name=f"backrest_side_lip_{i}_{j}",
            )
        # Hinge bracket bridging from hinge pin up to the backrest plate
        br.visual(
            Box((0.030, 0.050, 0.044)),
            origin=Origin(xyz=(0.0, 0.0, 0.010)),
            material="matte_black",
            name=f"backrest_bracket_{i}",
        )

        # Backrest tilt joint
        model.articulation(
            f"backrest_tilt_{i}",
            ArticulationType.REVOLUTE,
            parent=rocker,
            child=br,
            origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=10.0, velocity=1.0,
                lower=-BACKREST_LIMIT, upper=BACKREST_LIMIT,
            ),
        )

    # -----------------------------------------------------------------
    # Main rocking pivot
    # -----------------------------------------------------------------
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
    br0 = object_model.get_part("backrest_0")
    br1 = object_model.get_part("backrest_1")
    tilt0 = object_model.get_articulation("backrest_tilt_0")
    tilt1 = object_model.get_articulation("backrest_tilt_1")

    # --- Pivot stub captured in bracket (intentional overlap) ---
    ctx.allow_overlap(
        rocker, base,
        elem_a="pivot_stub", elem_b="pivot_bracket",
        reason="The center stub descends into the cast pivot bracket that captures the rocking axle.",
    )
    ctx.expect_overlap(
        rocker, base, axes="z",
        elem_a="pivot_stub", elem_b="pivot_bracket",
        min_overlap=0.04,
        name="pivot stub inserted into bracket",
    )
    ctx.expect_within(
        rocker, base, axes="xy",
        inner_elem="pivot_stub", outer_elem="pivot_bracket",
        margin=0.0,
        name="pivot stub centered in bracket",
    )

    # --- Backrest hinge brackets overlap with rocker lugs (intentional) ---
    ctx.allow_overlap(
        rocker, br0,
        elem_a="backrest_lug_0", elem_b="backrest_bracket_0",
        reason="Backrest bracket wraps around the rocker hinge lug pin to form a captured pivot.",
    )
    ctx.allow_overlap(
        rocker, br1,
        elem_a="backrest_lug_1", elem_b="backrest_bracket_1",
        reason="Backrest bracket wraps around the rocker hinge lug pin to form a captured pivot.",
    )
    ctx.allow_overlap(
        rocker, br0,
        elem_a="backrest_gusset_0", elem_b="backrest_bracket_0",
        reason="Backrest bracket coexists at the hinge point where the rocker gusset supports the pivot pin.",
    )
    ctx.allow_overlap(
        rocker, br1,
        elem_a="backrest_gusset_1", elem_b="backrest_bracket_1",
        reason="Backrest bracket coexists at the hinge point where the rocker gusset supports the pivot pin.",
    )
    ctx.expect_contact(
        rocker, br0,
        elem_a="backrest_lug_0", elem_b="backrest_bracket_0",
        contact_tol=0.005,
        name="backrest_0 hinge bracket contacts rocker lug",
    )
    ctx.expect_contact(
        rocker, br1,
        elem_a="backrest_lug_1", elem_b="backrest_bracket_1",
        contact_tol=0.005,
        name="backrest_1 hinge bracket contacts rocker lug",
    )

    # --- Bracket seated on pedestal ---
    bracket = ctx.part_element_world_aabb(base, elem="pivot_bracket")
    pedestal = ctx.part_element_world_aabb(base, elem="ground_pedestal")
    ctx.check(
        "bracket sits atop ground pedestal",
        _intersects(bracket, pedestal),
        details=f"bracket={bracket}, pedestal={pedestal}",
    )

    # --- Axle caps visible on bracket cheeks ---
    acap0 = ctx.part_element_world_aabb(base, elem="axle_cap_0")
    acap1 = ctx.part_element_world_aabb(base, elem="axle_cap_1")
    ctx.check(
        "axle caps present on both bracket cheeks",
        acap0 is not None and acap1 is not None
        and acap0[0][1] > 0.06 and acap1[1][1] < -0.06,
        details=f"acap0={acap0}, acap1={acap1}",
    )

    # --- Heavy steel beam: segmented box section, ~2.6 m long ---
    beam_mid = ctx.part_element_world_aabb(rocker, elem="beam_seg_5")
    beam_end0 = ctx.part_element_world_aabb(rocker, elem="beam_seg_0")
    beam_end9 = ctx.part_element_world_aabb(rocker, elem="beam_seg_9")
    ctx.check(
        "beam spans the seesaw length",
        beam_end0 is not None and beam_end9 is not None
        and beam_end0[0][0] < -1.0 and beam_end9[1][0] > 1.0,
        details=f"beam_end0={beam_end0}, beam_end9={beam_end9}",
    )
    ctx.check(
        "beam sweeps upward toward both ends",
        beam_mid is not None and beam_end0 is not None and beam_end9 is not None
        and beam_end0[1][2] > beam_mid[1][2]
        and beam_end9[1][2] > beam_mid[1][2],
        details=f"mid={beam_mid}, end0={beam_end0}, end9={beam_end9}",
    )
    # Box beam depth at center segment should be substantial (≥ 0.10 m)
    ctx.check(
        "beam has heavy box-section depth",
        beam_mid is not None and (beam_mid[1][2] - beam_mid[0][2]) >= 0.10,
        details=f"beam_mid z-extent={None if beam_mid is None else beam_mid[1][2] - beam_mid[0][2]}",
    )
    # Use mid-segment for subsequent connectivity checks
    beam = beam_mid

    # --- Rubber bumpers present under beam ---
    bump0 = ctx.part_element_world_aabb(rocker, elem="rubber_bumper_0")
    bump1 = ctx.part_element_world_aabb(rocker, elem="rubber_bumper_1")
    beam_near0 = ctx.part_element_world_aabb(rocker, elem="beam_seg_8")
    beam_near1 = ctx.part_element_world_aabb(rocker, elem="beam_seg_1")
    ctx.check(
        "rubber bumpers under beam near both ends",
        bump0 is not None and bump1 is not None
        and bump0[0][0] > 0.6 and bump1[1][0] < -0.6,
        details=f"bump0={bump0}, bump1={bump1}",
    )
    ctx.check(
        "bumpers hang below beam underside",
        bump0 is not None and bump1 is not None
        and beam_near0 is not None and beam_near1 is not None
        and bump0[1][2] < beam_near0[1][2]
        and bump1[1][2] < beam_near1[1][2],
        details=f"bump0={bump0}, bump1={bump1}, beam_near0={beam_near0}, beam_near1={beam_near1}",
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
        ra is not None and ba is not None and 0.82 <= max(ra[1][2], ba[1][2]) <= 0.98,
        details=f"rocker={ra}, base={ba}",
    )

    # --- Molded seats with raised lips ---
    seat0 = ctx.part_element_world_aabb(rocker, elem="seat_pan_0")
    seat1 = ctx.part_element_world_aabb(rocker, elem="seat_pan_1")
    lip_f0 = ctx.part_element_world_aabb(rocker, elem="seat_lip_front_0")
    lip_s0 = ctx.part_element_world_aabb(rocker, elem="seat_lip_side_0_0")
    ctx.check(
        "molded seat pans present at both ends",
        seat0 is not None and seat1 is not None,
        details=f"seat0={seat0}, seat1={seat1}",
    )
    ctx.check(
        "seat raised lips above seat pan surface",
        lip_f0 is not None and lip_s0 is not None and seat0 is not None
        and lip_f0[1][2] > seat0[1][2] - 0.005
        and lip_s0[1][2] > seat0[1][2] - 0.005,
        details=f"lip_f0={lip_f0}, lip_s0={lip_s0}, seat0={seat0}",
    )

    # --- Grip plates above beam ends ---
    grip0 = ctx.part_element_world_aabb(rocker, elem="handle_plate_0")
    grip1 = ctx.part_element_world_aabb(rocker, elem="handle_plate_1")
    beam_right = ctx.part_element_world_aabb(rocker, elem="beam_seg_9")
    beam_left = ctx.part_element_world_aabb(rocker, elem="beam_seg_0")
    ctx.check(
        "grip plates above the beam ends",
        grip0 is not None and grip1 is not None
        and beam_right is not None and beam_left is not None
        and grip0[0][2] > beam_right[1][2]
        and grip1[0][2] > beam_left[1][2],
        details=f"grip0={grip0}, grip1={grip1}",
    )

    # --- Mirrored ends ---
    def _cx(aabb):
        return 0.5 * (aabb[0][0] + aabb[1][0])

    ctx.check(
        "seat assemblies mirrored about the pivot",
        seat0 is not None and seat1 is not None
        and _cx(seat0) > 0.9 and _cx(seat1) < -0.9
        and abs(_cx(seat0) + _cx(seat1)) < 0.02,
        details=f"seat0={seat0}, seat1={seat1}",
    )

    # --- Mounted connectivity ---
    drop0 = ctx.part_element_world_aabb(rocker, elem="drop_tube_0")
    drop1 = ctx.part_element_world_aabb(rocker, elem="drop_tube_1")
    post0 = ctx.part_element_world_aabb(rocker, elem="handle_post_0")
    post1 = ctx.part_element_world_aabb(rocker, elem="handle_post_1")
    # Use beam segments near the collar positions for connectivity
    beam_right = ctx.part_element_world_aabb(rocker, elem="beam_seg_9")
    beam_left = ctx.part_element_world_aabb(rocker, elem="beam_seg_0")
    ctx.check(
        "drop tubes connect beam to seats",
        _intersects(drop0, beam_right) and _intersects(drop0, seat0)
        and _intersects(drop1, beam_left) and _intersects(drop1, seat1),
        details=f"drop0={drop0}, drop1={drop1}",
    )
    ctx.check(
        "handle posts connect beam to grip plates",
        _intersects(post0, beam_right) and _intersects(post0, grip0)
        and _intersects(post1, beam_left) and _intersects(post1, grip1),
        details=f"post0={post0}, post1={post1}",
    )

    # --- Rocker pivot joint limits ---
    lim = pivot.motion_limits
    ctx.check(
        "rocking range about +/- 15 degrees",
        lim is not None
        and abs(lim.lower + ROCK_LIMIT) < 0.02
        and abs(lim.upper - ROCK_LIMIT) < 0.02,
        details=f"limits=({lim.lower}, {lim.upper})",
    )

    # --- Backrest tilt joints exist with correct limits ---
    tlim0 = tilt0.motion_limits
    tlim1 = tilt1.motion_limits
    ctx.check(
        "backrest_0 tilt joint has +/- 10 degree range",
        tlim0 is not None
        and abs(tlim0.lower + BACKREST_LIMIT) < 0.02
        and abs(tlim0.upper - BACKREST_LIMIT) < 0.02,
        details=f"tilt0 limits=({tlim0.lower}, {tlim0.upper})",
    )
    ctx.check(
        "backrest_1 tilt joint has +/- 10 degree range",
        tlim1 is not None
        and abs(tlim1.lower + BACKREST_LIMIT) < 0.02
        and abs(tlim1.upper - BACKREST_LIMIT) < 0.02,
        details=f"tilt1 limits=({tlim1.lower}, {tlim1.upper})",
    )

    # --- Decisive pose checks: rocker tilts, seats swap, backrest tilts ---
    base_rest = ctx.part_world_aabb(base)
    with ctx.pose({pivot: ROCK_LIMIT}):
        seat0_dn = ctx.part_element_world_aabb(rocker, elem="seat_pan_0")
        seat1_up = ctx.part_element_world_aabb(rocker, elem="seat_pan_1")
        rocker_dn = ctx.part_world_aabb(rocker)
        base_posed = ctx.part_world_aabb(base)
        ctx.check(
            "positive rock lowers seat_0 and raises seat_1",
            seat0_dn is not None and seat1_up is not None
            and seat0 is not None and seat1 is not None
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
            base_rest is not None and base_posed is not None
            and _intersects(base_rest, base_posed)
            and abs(base_rest[1][2] - base_posed[1][2]) < 1e-6,
            details=f"rest={base_rest}, posed={base_posed}",
        )

    # Backrest tilt pose check: rotation around Y axis shifts the top in X
    br0_rest = ctx.part_world_aabb(br0)
    with ctx.pose({tilt0: BACKREST_LIMIT}):
        br0_posed = ctx.part_world_aabb(br0)
        ctx.check(
            "backrest_0 tilts on its joint",
            br0_rest is not None and br0_posed is not None
            and abs(br0_rest[1][0] - br0_posed[1][0]) > 0.005,
            details=f"rest_max_x={br0_rest[1][0]}, posed_max_x={br0_posed[1][0]}",
        )

    return ctx.report()


object_model = build_object_model()
