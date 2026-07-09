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
# Shared dimensions (metres).  World: Z up, X and Y along the two beams.
# ---------------------------------------------------------------------------

# --- Base / pedestal ---
PEDESTAL_R = 0.085
PEDESTAL_H = 0.24
BASE_PLATE_SZ = 0.22
BASE_PLATE_T = 0.015
HOUSING_R = 0.050
HOUSING_H = 0.06

# --- Support legs (4 diagonals) ---
LEG_DIRS = [(1, 1), (1, -1), (-1, 1), (-1, -1)]
LEG_START_R = 0.08
LEG_START_Z = 0.12
LEG_END_R = 0.44
LEG_END_Z = 0.018
LEG_R = 0.022
PAD_R = 0.055
PAD_T = 0.018

# --- Spring (prismatic) ---
SPRING_H = 0.06
SPRING_R = 0.030
SPRING_WIRE_R = 0.007
SPRING_COILS = 4
SPRING_TRAVEL = 0.05  # max compression

CARRIAGE_PLATE_R = 0.065
CARRIAGE_PLATE_T = 0.010

BRACKET_SZ = (0.15, 0.15, 0.10)
# Bracket centre-Z in carriage frame:
BRACKET_CZ = SPRING_H + CARRIAGE_PLATE_T + BRACKET_SZ[2] / 2.0  # 0.12

# Prismatic joint origin in base frame (top of housing):
PRISMATIC_Z = PEDESTAL_H + HOUSING_H  # 0.30

# Revolute joint origin in carriage frame (inside bracket):
REVOLUTE_Z_CARR = BRACKET_CZ  # 0.12

# Pivot world height at rest:
# PRISMATIC_Z + REVOLUTE_Z_CARR = 0.30 + 0.12 = 0.42

# --- Rocker (cross beams, relative to pivot frame) ---
BEAM_R = 0.048
BEAM_HALF = 1.15
CURVE_C = 0.10
BEAM_CENTER_Z = 0.10

COLLAR_DIST = 0.95
SEAT_DIST = 1.12
SEAT_Z = -0.01
PLATE_T = 0.012
HANDLE_DIST = 1.0
HANDLE_Z = 0.42

ROCK_LIMIT = 0.262  # ~15 deg

# pivot stub (short enough to stay above the spring coil)
STUB_R = 0.040
STUB_LEN = 0.10
STUB_CZ = 0.02  # raised so bottom clears the spring coil

# footrest
FOOTREST_DIST = 0.78
FOOTREST_SZ = (0.10, 0.12, 0.005)
FOOTREST_Z = -0.04
RIDGE_COUNT = 4
RIDGE_W = 0.006
RIDGE_H = 0.007


def _beam_z(t: float) -> float:
    """Beam centreline height (rocker frame) at station *t* along the beam."""
    return BEAM_CENTER_Z + CURVE_C * t * t


def _helix_points(n_coils: int, radius: float, height: float,
                  pts_per_coil: int = 20) -> list[tuple[float, float, float]]:
    total = n_coils * pts_per_coil
    pts: list[tuple[float, float, float]] = []
    for i in range(total + 1):
        t = i / total
        ang = 2.0 * math.pi * n_coils * t
        pts.append((radius * math.cos(ang), radius * math.sin(ang), height * t))
    return pts


# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cross_seesaw")

    model.material("gloss_red", rgba=(0.88, 0.20, 0.06, 1.0))
    model.material("matte_black", rgba=(0.09, 0.09, 0.10, 1.0))
    model.material("light_gray", rgba=(0.78, 0.78, 0.79, 1.0))
    model.material("dark_gray", rgba=(0.34, 0.36, 0.38, 1.0))
    model.material("silver", rgba=(0.74, 0.75, 0.78, 1.0))
    model.material("rubber", rgba=(0.06, 0.06, 0.07, 1.0))
    model.material("spring_steel", rgba=(0.55, 0.57, 0.60, 1.0))
    model.material("footrest_gray", rgba=(0.40, 0.42, 0.44, 1.0))

    # =================================================================
    # 1. Fixed base: pedestal, base plate, housing, legs, rubber pads
    # =================================================================
    base = model.part("base_mount")

    # ground pedestal
    base.visual(
        Cylinder(radius=PEDESTAL_R, length=PEDESTAL_H),
        origin=Origin(xyz=(0.0, 0.0, PEDESTAL_H / 2.0)),
        material="light_gray", name="ground_pedestal",
    )
    # square base plate
    base.visual(
        Box((BASE_PLATE_SZ, BASE_PLATE_SZ, BASE_PLATE_T)),
        origin=Origin(xyz=(0.0, 0.0, BASE_PLATE_T / 2.0)),
        material="light_gray", name="base_plate",
    )
    # spring housing (short cylinder on top of pedestal)
    base.visual(
        Cylinder(radius=HOUSING_R, length=HOUSING_H),
        origin=Origin(xyz=(0.0, 0.0, PEDESTAL_H + HOUSING_H / 2.0)),
        material="dark_gray", name="spring_housing",
    )

    # 4 diagonal support legs with rubber pads
    _inv_sqrt2 = 1.0 / math.sqrt(2.0)
    for li, (dx, dy) in enumerate(LEG_DIRS):
        dxn, dyn = dx * _inv_sqrt2, dy * _inv_sqrt2
        # leg tube
        leg_pts = [
            (LEG_START_R * dxn, LEG_START_R * dyn, LEG_START_Z),
            (0.5 * (LEG_START_R + LEG_END_R) * dxn,
             0.5 * (LEG_START_R + LEG_END_R) * dyn,
             0.5 * (LEG_START_Z + LEG_END_Z)),
            (LEG_END_R * dxn, LEG_END_R * dyn, LEG_END_Z),
        ]
        base.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    leg_pts, radius=LEG_R,
                    samples_per_segment=6, radial_segments=14, cap_ends=True,
                ),
                f"support_leg_{li}",
            ),
            material="light_gray", name=f"support_leg_{li}",
        )
        # rubber ground pad
        base.visual(
            Cylinder(radius=PAD_R, length=PAD_T),
            origin=Origin(xyz=(LEG_END_R * dxn, LEG_END_R * dyn, PAD_T / 2.0)),
            material="rubber", name=f"rubber_pad_{li}",
        )

    # =================================================================
    # 2. Spring carriage (prismatic child of base)
    # =================================================================
    carriage = model.part("spring_carriage")

    # spring coil (helix tube)
    helix_pts = _helix_points(SPRING_COILS, SPRING_R, SPRING_H, 20)
    carriage.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                helix_pts, radius=SPRING_WIRE_R,
                samples_per_segment=4, radial_segments=10, cap_ends=True,
            ),
            "spring_coil",
        ),
        material="spring_steel", name="spring_coil",
    )

    # carriage top plate
    carriage.visual(
        Cylinder(radius=CARRIAGE_PLATE_R, length=CARRIAGE_PLATE_T),
        origin=Origin(xyz=(0.0, 0.0, SPRING_H + CARRIAGE_PLATE_T / 2.0)),
        material="dark_gray", name="carriage_plate",
    )

    # pivot bracket
    carriage.visual(
        Box(BRACKET_SZ),
        origin=Origin(xyz=(0.0, 0.0, BRACKET_CZ)),
        material="matte_black", name="pivot_bracket",
    )
    # pivot bosses + bolts on both cheeks
    for i, sy in enumerate((1.0, -1.0)):
        carriage.visual(
            Cylinder(radius=0.048, length=0.018),
            origin=Origin(
                xyz=(0.0, sy * (BRACKET_SZ[1] / 2.0 + 0.009), BRACKET_CZ),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material="matte_black", name=f"pivot_boss_{i}",
        )
        for j, ang in enumerate((0.25, 0.75, 1.25, 1.75)):
            bx = 0.030 * math.cos(ang * math.pi)
            bz = 0.030 * math.sin(ang * math.pi)
            carriage.visual(
                Cylinder(radius=0.008, length=0.010),
                origin=Origin(
                    xyz=(bx, sy * (BRACKET_SZ[1] / 2.0 + 0.020), BRACKET_CZ + bz),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material="silver", name=f"bracket_bolt_{i}_{j}",
            )

    # =================================================================
    # 3. Cross rocker (revolute child of carriage)
    # =================================================================
    rocker = model.part("cross_rocker")

    # --- Two perpendicular curved beams ---
    n_beam = 12
    # X-beam
    x_beam_pts = [
        (BEAM_HALF * k / n_beam, 0.0, _beam_z(BEAM_HALF * k / n_beam))
        for k in range(-n_beam, n_beam + 1)
    ]
    rocker.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                x_beam_pts, radius=BEAM_R,
                samples_per_segment=4, radial_segments=24, cap_ends=True,
            ),
            "beam_tube_x",
        ),
        material="gloss_red", name="beam_tube_x",
    )
    # Y-beam
    y_beam_pts = [
        (0.0, BEAM_HALF * k / n_beam, _beam_z(BEAM_HALF * k / n_beam))
        for k in range(-n_beam, n_beam + 1)
    ]
    rocker.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                y_beam_pts, radius=BEAM_R,
                samples_per_segment=4, radial_segments=24, cap_ends=True,
            ),
            "beam_tube_y",
        ),
        material="gloss_red", name="beam_tube_y",
    )

    # Centre hub where beams cross (short enough to clear the bracket below)
    rocker.visual(
        Cylinder(radius=BEAM_R * 1.8, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, BEAM_CENTER_Z)),
        material="gloss_red", name="center_hub",
    )

    # Red flare wedge blending beam centre into the pivot stub
    wedge = ConeGeometry(0.065, 0.04, radial_segments=24).rotate_x(math.pi)
    wedge.translate(0.0, 0.0, 0.085)
    rocker.visual(
        mesh_from_geometry(wedge, "pivot_wedge"),
        material="gloss_red", name="pivot_wedge",
    )

    # Pivot stub descending into bracket
    rocker.visual(
        Cylinder(radius=STUB_R, length=STUB_LEN),
        origin=Origin(xyz=(0.0, 0.0, STUB_CZ)),
        material="gloss_red", name="pivot_stub",
    )

    # --- Seat profile and grip profile (shared) ---
    seat_profile = sample_catmull_rom_spline_2d(
        [
            (0.20, 0.0), (0.05, 0.11), (-0.10, 0.14),
            (-0.18, 0.10), (-0.20, 0.0), (-0.18, -0.10),
            (-0.10, -0.14), (0.05, -0.11),
        ],
        samples_per_segment=8, closed=True,
    )
    grip_outer = rounded_rect_profile(0.16, 0.28, 0.045)
    grip_hole = rounded_rect_profile(0.055, 0.085, 0.018)
    grip_holes = [
        [(hx, hy + 0.07) for hx, hy in grip_hole],
        [(hx, hy - 0.07) for hx, hy in grip_hole],
    ]

    collar_z = _beam_z(COLLAR_DIST)
    slope = 2.0 * CURVE_C * COLLAR_DIST
    tang = math.atan(slope)

    # --- 4 end assemblies (two along X, two along Y) ---
    # (end_index, dx, dy, seat_rot_z, is_x_beam)
    ends = [
        (0, 1.0, 0.0, 0.0, True),
        (1, -1.0, 0.0, math.pi, True),
        (2, 0.0, 1.0, math.pi / 2.0, False),
        (3, 0.0, -1.0, -math.pi / 2.0, False),
    ]

    for idx, dx, dy, seat_rot, is_x in ends:
        s = dx + dy  # signed magnitude (+1 or -1)

        # -- Clamp collar --
        if is_x:
            crpy = (0.0, s * (math.pi / 2.0 - tang), 0.0)
        else:
            crpy = (-s * (math.pi / 2.0 - tang), 0.0, 0.0)
        rocker.visual(
            Cylinder(radius=0.072, length=0.075),
            origin=Origin(
                xyz=(dx * COLLAR_DIST, dy * COLLAR_DIST, collar_z),
                rpy=crpy,
            ),
            material="matte_black", name=f"clamp_collar_{idx}",
        )
        # collar bolts
        for j, bsy in enumerate((1.0, -1.0)):
            if is_x:
                bpos = (dx * COLLAR_DIST, bsy * 0.074, collar_z)
                brpy = (math.pi / 2.0, 0.0, 0.0)
            else:
                bpos = (bsy * 0.074, dy * COLLAR_DIST, collar_z)
                brpy = (0.0, math.pi / 2.0, 0.0)
            rocker.visual(
                Cylinder(radius=0.010, length=0.028),
                origin=Origin(xyz=bpos, rpy=brpy),
                material="silver", name=f"collar_bolt_{idx}_{j}",
            )

        # -- Drop tube (collar → seat) --
        drop_pts = [
            (dx * COLLAR_DIST, dy * COLLAR_DIST, collar_z),
            (dx * 1.03, dy * 1.03, 0.14),
            (dx * 1.09, dy * 1.09, 0.06),
            (dx * SEAT_DIST, dy * SEAT_DIST, SEAT_Z + 0.012),
        ]
        rocker.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    drop_pts, radius=0.024,
                    samples_per_segment=8, radial_segments=16,
                ),
                f"drop_tube_{idx}",
            ),
            material="gloss_red", name=f"drop_tube_{idx}",
        )

        # -- Seat plate --
        seat = ExtrudeGeometry(seat_profile, PLATE_T, cap=True, center=True)
        seat.rotate_z(seat_rot)
        seat.translate(dx * SEAT_DIST, dy * SEAT_DIST, SEAT_Z)
        rocker.visual(
            mesh_from_geometry(seat, f"seat_plate_{idx}"),
            material="dark_gray", name=f"seat_plate_{idx}",
        )
        # rivets on seat
        rivet_xy = [(0.12, 0.0), (0.0, 0.09), (0.0, -0.09),
                     (-0.12, 0.07), (-0.12, -0.07)]
        for j, (lx, ly) in enumerate(rivet_xy):
            # rotate rivet by seat_rot about Z then translate
            c, sn = math.cos(seat_rot), math.sin(seat_rot)
            rx = c * lx - sn * ly
            ry = sn * lx + c * ly
            rocker.visual(
                Cylinder(radius=0.007, length=0.009),
                origin=Origin(xyz=(dx * SEAT_DIST + rx, dy * SEAT_DIST + ry, SEAT_Z + 0.008)),
                material="silver", name=f"seat_rivet_{idx}_{j}",
            )

        # -- Seat stop fin (overlaps seat bottom for connectivity) --
        fin_dx, fin_dy = dx * 1.24, dy * 1.24
        rocker.visual(
            Box((0.040, 0.020, 0.035)),
            origin=Origin(xyz=(fin_dx, fin_dy, SEAT_Z - 0.020)),
            material="matte_black", name=f"seat_fin_{idx}",
        )

        # -- Handle post (beam → grip plate) --
        post_pts = [
            (dx * COLLAR_DIST, dy * COLLAR_DIST, collar_z + 0.06),
            (dx * 0.97, dy * 0.97, 0.32),
            (dx * 0.99, dy * 0.99, 0.40),
            (dx * HANDLE_DIST, dy * HANDLE_DIST, HANDLE_Z - 0.02),
        ]
        rocker.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    post_pts, radius=0.019,
                    samples_per_segment=8, radial_segments=14,
                ),
                f"handle_post_{idx}",
            ),
            material="gloss_red", name=f"handle_post_{idx}",
        )

        # -- Handle/grip plate with cutouts --
        grip = ExtrudeWithHolesGeometry(
            grip_outer, grip_holes, PLATE_T, cap=True, center=True,
        )
        grip.rotate_z(seat_rot)
        grip.translate(dx * HANDLE_DIST, dy * HANDLE_DIST, HANDLE_Z)
        rocker.visual(
            mesh_from_geometry(grip, f"handle_plate_{idx}"),
            material="dark_gray", name=f"handle_plate_{idx}",
        )

        # -- Textured footrest near seat --
        fr_x = dx * FOOTREST_DIST
        fr_y = dy * FOOTREST_DIST
        if is_x:
            fr_sz = (FOOTREST_SZ[0], FOOTREST_SZ[1], FOOTREST_SZ[2])
        else:
            fr_sz = (FOOTREST_SZ[1], FOOTREST_SZ[0], FOOTREST_SZ[2])

        # Footrest bracket: thin tube connecting beam to footrest plate
        beam_z_at_fr = _beam_z(FOOTREST_DIST)
        fr_top = FOOTREST_Z + FOOTREST_SZ[2] / 2.0
        bracket_len = beam_z_at_fr - fr_top + 0.002
        bracket_cz = 0.5 * (beam_z_at_fr + fr_top)
        rocker.visual(
            Cylinder(radius=0.012, length=bracket_len),
            origin=Origin(xyz=(fr_x, fr_y, bracket_cz)),
            material="gloss_red", name=f"footrest_bracket_{idx}",
        )

        rocker.visual(
            Box(fr_sz),
            origin=Origin(xyz=(fr_x, fr_y, FOOTREST_Z)),
            material="footrest_gray", name=f"footrest_plate_{idx}",
        )
        # ridges on footrest (perpendicular to beam, embed 1 mm into plate)
        for ri in range(RIDGE_COUNT):
            frac = (ri + 0.5) / RIDGE_COUNT - 0.5
            ridge_z = FOOTREST_Z + FOOTREST_SZ[2] / 2.0 + RIDGE_H / 2.0 - 0.001
            if is_x:
                roff = frac * fr_sz[0] * 0.75
                rpos = (fr_x + roff, fr_y, ridge_z)
                rsz = (RIDGE_W, fr_sz[1] * 0.85, RIDGE_H)
            else:
                roff = frac * fr_sz[1] * 0.75
                rpos = (fr_x, fr_y + roff, ridge_z)
                rsz = (fr_sz[0] * 0.85, RIDGE_W, RIDGE_H)
            rocker.visual(
                Box(rsz),
                origin=Origin(xyz=rpos),
                material="matte_black", name=f"footrest_ridge_{idx}_{ri}",
            )

    # =================================================================
    # Articulations
    # =================================================================

    # Prismatic spring compression: base → carriage
    model.articulation(
        "spring_compress",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, PRISMATIC_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=800.0, velocity=0.3,
            lower=0.0, upper=SPRING_TRAVEL,
        ),
    )

    # Revolute rocker pivot: carriage → cross_rocker
    model.articulation(
        "rocker_pivot",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=rocker,
        origin=Origin(xyz=(0.0, 0.0, REVOLUTE_Z_CARR)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=400.0, velocity=1.5,
            lower=-ROCK_LIMIT, upper=ROCK_LIMIT,
        ),
    )

    return model


# ---------------------------------------------------------------------------

def _intersects(a, b, tol: float = 1e-4) -> bool:
    if a is None or b is None:
        return False
    return all(a[0][i] <= b[1][i] + tol and b[0][i] <= a[1][i] + tol for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_mount")
    carriage = object_model.get_part("spring_carriage")
    rocker = object_model.get_part("cross_rocker")
    spring_joint = object_model.get_articulation("spring_compress")
    pivot_joint = object_model.get_articulation("rocker_pivot")

    # --- Spring coil sits partially inside the housing guide ---
    ctx.allow_overlap(
        base, carriage,
        elem_a="spring_housing", elem_b="spring_coil",
        reason="The spring coil rests partially inside the housing guide at the pedestal top; this is the spring capture zone.",
    )
    ctx.expect_contact(
        base, carriage,
        elem_a="spring_housing", elem_b="spring_coil",
        contact_tol=0.01,
        name="spring coil seated in housing",
    )

    # --- Pivot stub captured inside bracket ---
    ctx.allow_overlap(
        rocker, carriage,
        elem_a="pivot_stub", elem_b="pivot_bracket",
        reason="The red pivot stub descends into the cast pivot bracket that captures the rocking axle.",
    )
    ctx.expect_overlap(
        rocker, carriage, axes="z",
        elem_a="pivot_stub", elem_b="pivot_bracket",
        min_overlap=0.03,
        name="pivot stub inserted into bracket",
    )
    ctx.expect_within(
        rocker, carriage, axes="xy",
        inner_elem="pivot_stub", outer_elem="pivot_bracket",
        margin=0.0,
        name="pivot stub centred in bracket",
    )

    # --- Cross beams: X and Y ---
    beam_x = ctx.part_element_world_aabb(rocker, elem="beam_tube_x")
    beam_y = ctx.part_element_world_aabb(rocker, elem="beam_tube_y")
    ctx.check(
        "X-beam spans the seesaw length",
        beam_x is not None and (beam_x[1][0] - beam_x[0][0]) >= 2.0,
        details=f"beam_x={beam_x}",
    )
    ctx.check(
        "Y-beam spans perpendicular",
        beam_y is not None and (beam_y[1][1] - beam_y[0][1]) >= 2.0,
        details=f"beam_y={beam_y}",
    )

    # --- 4 seats at 4 positions ---
    seat_aabbs = []
    for i in range(4):
        sa = ctx.part_element_world_aabb(rocker, elem=f"seat_plate_{i}")
        seat_aabbs.append(sa)
    ctx.check(
        "all 4 seats exist",
        all(s is not None for s in seat_aabbs),
        details=f"seats={seat_aabbs}",
    )
    # seats along X at ±SEAT_DIST
    ctx.check(
        "X-beam seats at opposite X ends",
        seat_aabbs[0] is not None and seat_aabbs[1] is not None
        and 0.5 * (seat_aabbs[0][0][0] + seat_aabbs[0][1][0]) > 0.8
        and 0.5 * (seat_aabbs[1][0][0] + seat_aabbs[1][1][0]) < -0.8,
        details=f"seat0={seat_aabbs[0]}, seat1={seat_aabbs[1]}",
    )
    # seats along Y at ±SEAT_DIST
    ctx.check(
        "Y-beam seats at opposite Y ends",
        seat_aabbs[2] is not None and seat_aabbs[3] is not None
        and 0.5 * (seat_aabbs[2][0][1] + seat_aabbs[2][1][1]) > 0.8
        and 0.5 * (seat_aabbs[3][0][1] + seat_aabbs[3][1][1]) < -0.8,
        details=f"seat2={seat_aabbs[2]}, seat3={seat_aabbs[3]}",
    )

    # --- 4 rubber pads on ground ---
    pad_aabbs = []
    for i in range(4):
        pa = ctx.part_element_world_aabb(base, elem=f"rubber_pad_{i}")
        pad_aabbs.append(pa)
    ctx.check(
        "all 4 rubber pads exist near ground",
        all(p is not None and p[0][2] < 0.03 for p in pad_aabbs),
        details=f"pads={pad_aabbs}",
    )

    # --- Footrests near seats ---
    fr_aabbs = []
    for i in range(4):
        fa = ctx.part_element_world_aabb(rocker, elem=f"footrest_plate_{i}")
        fr_aabbs.append(fa)
    ctx.check(
        "all 4 footrests exist",
        all(f is not None for f in fr_aabbs),
        details=f"footrests={fr_aabbs}",
    )
    # each footrest between centre and its seat
    for i in range(4):
        if fr_aabbs[i] is not None and seat_aabbs[i] is not None:
            fr_cx = 0.5 * (fr_aabbs[i][0][0] + fr_aabbs[i][1][0])
            fr_cy = 0.5 * (fr_aabbs[i][0][1] + fr_aabbs[i][1][1])
            se_cx = 0.5 * (seat_aabbs[i][0][0] + seat_aabbs[i][1][0])
            se_cy = 0.5 * (seat_aabbs[i][0][1] + seat_aabbs[i][1][1])
            fr_dist = math.sqrt(fr_cx ** 2 + fr_cy ** 2)
            se_dist = math.sqrt(se_cx ** 2 + se_cy ** 2)
            ctx.check(
                f"footrest_{i} inboard of seat_{i}",
                fr_dist < se_dist,
                details=f"fr_dist={fr_dist:.3f}, se_dist={se_dist:.3f}",
            )

    # --- Spring coil exists on carriage ---
    spring_aabb = ctx.part_element_world_aabb(carriage, elem="spring_coil")
    ctx.check(
        "spring coil visible on carriage",
        spring_aabb is not None and (spring_aabb[1][2] - spring_aabb[0][2]) >= 0.04,
        details=f"spring={spring_aabb}",
    )

    # --- Prismatic joint: spring compression ---
    sl = spring_joint.motion_limits
    ctx.check(
        "spring prismatic joint has compression travel",
        sl is not None and abs(sl.lower) < 0.001 and sl.upper >= 0.03,
        details=f"limits=({sl.lower}, {sl.upper})",
    )

    # --- Revolute joint: rocking ±15° ---
    pl = pivot_joint.motion_limits
    ctx.check(
        "rocker pivot has ±15° range",
        pl is not None
        and abs(pl.lower + ROCK_LIMIT) < 0.02
        and abs(pl.upper - ROCK_LIMIT) < 0.02,
        details=f"limits=({pl.lower}, {pl.upper})",
    )

    # --- Pose: spring compression moves carriage down ---
    carr_rest = ctx.part_world_aabb(carriage)
    with ctx.pose({spring_joint: SPRING_TRAVEL}):
        carr_comp = ctx.part_world_aabb(carriage)
        ctx.check(
            "spring compression lowers carriage",
            carr_rest is not None and carr_comp is not None
            and carr_comp[1][2] < carr_rest[1][2] - 0.02,
            details=f"rest={carr_rest}, compressed={carr_comp}",
        )

    # --- Pose: rocker tilt swaps X-beam seat heights ---
    with ctx.pose({pivot_joint: ROCK_LIMIT}):
        s0_dn = ctx.part_element_world_aabb(rocker, elem="seat_plate_0")
        s1_up = ctx.part_element_world_aabb(rocker, elem="seat_plate_1")
        ctx.check(
            "positive rock lowers seat_0 and raises seat_1",
            s0_dn is not None and s1_up is not None
            and seat_aabbs[0] is not None and seat_aabbs[1] is not None
            and s0_dn[1][2] < seat_aabbs[0][1][2] - 0.10
            and s1_up[1][2] > seat_aabbs[1][1][2] + 0.10,
            details=f"s0_dn={s0_dn}, s1_up={s1_up}",
        )

    # --- Overall envelope ---
    ra = ctx.part_world_aabb(rocker)
    ba = ctx.part_world_aabb(base)
    ctx.check(
        "overall span about 2.3 m each direction",
        ra is not None
        and (ra[1][0] - ra[0][0]) >= 2.0
        and (ra[1][1] - ra[0][1]) >= 2.0,
        details=f"rocker_aabb={ra}",
    )
    ctx.check(
        "overall height about 0.9 m",
        ra is not None and ba is not None
        and 0.78 <= max(ra[1][2], ba[1][2]) <= 1.0,
        details=f"rocker={ra}, base={ba}",
    )

    return ctx.report()


object_model = build_object_model()
