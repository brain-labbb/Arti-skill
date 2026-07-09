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
    SphereGeometry,
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
# Shared dimensions (meters). World: X along seesaw length, Z up.
# Rocker part frame sits at the pivot axis so the revolute joint needs
# no extra offset. All rocker geometry is relative to that frame.
# ---------------------------------------------------------------------------
PIVOT_Z = 0.35          # world height of the rocking axis

# Animal body (horse torso) — capsule lying along X
BODY_CYL = 1.10         # capsule cylindrical mid-section length
BODY_R = 0.21           # body capsule radius
BODY_Z = 0.26           # body center height above pivot frame

# Head assembly
NECK_BASE_X = 0.50
NECK_BASE_Z = BODY_Z + 0.06   # 0.32
NECK_TOP_X = 0.68
NECK_TOP_Z = 0.42
HEAD_R = 0.12
HEAD_X = 0.74
HEAD_Z = 0.44
SNOUT_R = 0.065
SNOUT_X = HEAD_X + 0.10
SNOUT_Z = HEAD_Z - 0.04

# Tail
TAIL_X = -0.68
TAIL_Z = BODY_Z + 0.05

# Seats and end assemblies
SEAT_X = 0.62
SEAT_1_X = -0.62
SEAT_Z = BODY_Z + BODY_R + 0.005   # 0.475 — just above body top
HANDLE_POST_X = 0.70
HANDLE_1_POST_X = -0.70
HANDLE_Z = 0.56
FOOTREST_X = 0.55
FOOTREST_1_X = -0.55
FOOTREST_Y = 0.20
FOOTREST_Z = BODY_Z - 0.04   # 0.22

# Bumpers (separate prismatic parts)
BUMPER_R = 0.055
BUMPER_H = 0.06
BUMPER_X = 0.72
BUMPER_1_X = -0.72
# Boss protrudes below body at each end; bumper seats into it
BUMPER_BOSS_LEN = 0.06
BUMPER_TRAVEL = 0.028  # vertical compression travel

# Base
PEDESTAL_R = 0.08
PEDESTAL_H = 0.24
BRACKET_SIZE = (0.18, 0.14, 0.16)
BRACKET_CZ = 0.30       # bracket center height
LEG_CENTER_X = 0.10     # leg center position
LEG_H = 0.22            # leg height
PAD_THICK = 0.018       # rubber ground pad thickness

ROCK_LIMIT = 0.262      # ~15 degrees each way


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="animal_toddler_seesaw")

    model.material("horse_yellow", rgba=(0.95, 0.72, 0.15, 1.0))
    model.material("horse_dark", rgba=(0.35, 0.22, 0.10, 1.0))
    model.material("matte_black", rgba=(0.09, 0.09, 0.10, 1.0))
    model.material("light_gray", rgba=(0.78, 0.78, 0.79, 1.0))
    model.material("dark_rubber", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("seat_blue", rgba=(0.18, 0.35, 0.62, 1.0))
    model.material("silver_rivet", rgba=(0.74, 0.75, 0.78, 1.0))
    model.material("handle_grip", rgba=(0.30, 0.30, 0.32, 1.0))
    model.material("footrest_tex", rgba=(0.25, 0.25, 0.27, 1.0))

    # -----------------------------------------------------------------
    # Fixed base: central pedestal, spread support legs, ground pads,
    # and the black pivot bracket.
    # -----------------------------------------------------------------
    base = model.part("base_support")

    # Central pedestal
    base.visual(
        Cylinder(radius=PEDESTAL_R, length=PEDESTAL_H),
        origin=Origin(xyz=(0.0, 0.0, PEDESTAL_H / 2.0)),
        material="light_gray",
        name="ground_pedestal",
    )

    # Spread support legs (wide enough to overlap with the pedestal)
    for i, sx in enumerate((1.0, -1.0)):
        base.visual(
            Box((0.14, 0.12, LEG_H)),
            origin=Origin(
                xyz=(sx * LEG_CENTER_X, 0.0, LEG_H / 2.0),
                rpy=(0.0, sx * 0.10, 0.0),
            ),
            material="light_gray",
            name=f"support_leg_{i}",
        )
        # Rubber ground pad under each leg
        base.visual(
            Box((0.18, 0.14, PAD_THICK)),
            origin=Origin(xyz=(sx * (LEG_CENTER_X + 0.02), 0.0, PAD_THICK / 2.0)),
            material="dark_rubber",
            name=f"ground_pad_{i}",
        )

    # Pivot bracket on top of pedestal
    base.visual(
        Box(BRACKET_SIZE),
        origin=Origin(xyz=(0.0, 0.0, BRACKET_CZ)),
        material="matte_black",
        name="pivot_bracket",
    )
    # Pivot bosses with bolts
    for i, sy in enumerate((1.0, -1.0)):
        base.visual(
            Cylinder(radius=0.050, length=0.022),
            origin=Origin(
                xyz=(0.0, sy * 0.080, PIVOT_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material="matte_black",
            name=f"pivot_boss_{i}",
        )
        for j, ang in enumerate((0.3, 0.7, 1.3, 1.7)):
            dx = 0.032 * math.cos(ang * math.pi)
            dz = 0.032 * math.sin(ang * math.pi)
            base.visual(
                Cylinder(radius=0.008, length=0.012),
                origin=Origin(
                    xyz=(dx, sy * 0.093, PIVOT_Z + dz),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material="silver_rivet",
                name=f"bracket_bolt_{i}_{j}",
            )

    # -----------------------------------------------------------------
    # Rocker: horse-shaped body with seats, handles, footrests.
    # Part frame at pivot axis; geometry relative to that frame.
    # -----------------------------------------------------------------
    rocker = model.part("rocker_body")

    # --- Main horse torso: large capsule lying along X ---
    torso = CapsuleGeometry(
        radius=BODY_R, length=BODY_CYL,
        radial_segments=28, height_segments=10,
    )
    torso.rotate_y(math.pi / 2.0)  # lie along X
    torso.translate(0.0, 0.0, BODY_Z)
    rocker.visual(
        mesh_from_geometry(torso, "horse_torso"),
        material="horse_yellow",
        name="horse_torso",
    )

    # --- Pivot stub descending from body into the bracket ---
    rocker.visual(
        Cylinder(radius=0.045, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material="horse_yellow",
        name="pivot_stub",
    )

    # --- Neck: cylinder connecting torso to head ---
    neck_dx = NECK_TOP_X - NECK_BASE_X
    neck_dz = NECK_TOP_Z - NECK_BASE_Z
    neck_len = math.sqrt(neck_dx * neck_dx + neck_dz * neck_dz)
    neck_angle = math.atan2(neck_dx, neck_dz)
    neck_cx = 0.5 * (NECK_BASE_X + NECK_TOP_X)
    neck_cz = 0.5 * (NECK_BASE_Z + NECK_TOP_Z)
    neck = CylinderGeometry(
        radius=0.10, height=neck_len + 0.06, radial_segments=20
    )
    neck.rotate_y(-neck_angle)
    neck.translate(neck_cx, 0.0, neck_cz)
    rocker.visual(
        mesh_from_geometry(neck, "horse_neck"),
        material="horse_yellow",
        name="horse_neck",
    )

    # --- Head: sphere ---
    head = SphereGeometry(radius=HEAD_R, width_segments=22, height_segments=16)
    head.translate(HEAD_X, 0.0, HEAD_Z)
    rocker.visual(
        mesh_from_geometry(head, "horse_head"),
        material="horse_yellow",
        name="horse_head",
    )

    # --- Snout: smaller capsule extending from head ---
    snout = CapsuleGeometry(
        radius=SNOUT_R, length=0.06, radial_segments=16, height_segments=4
    )
    snout.rotate_y(math.pi / 2.0)
    snout.translate(SNOUT_X, 0.0, SNOUT_Z)
    rocker.visual(
        mesh_from_geometry(snout, "horse_snout"),
        material="horse_yellow",
        name="horse_snout",
    )

    # --- Ears: two small cones on top of the head ---
    for i, sy in enumerate((1.0, -1.0)):
        ear = ConeGeometry(radius=0.032, height=0.05, radial_segments=12)
        ear.translate(HEAD_X - 0.02, sy * 0.06, HEAD_Z + HEAD_R - 0.02)
        rocker.visual(
            mesh_from_geometry(ear, f"horse_ear_{i}"),
            material="horse_dark",
            name=f"horse_ear_{i}",
        )

    # --- Mane: ridge of small bumps along the neck top ---
    n_mane = 5
    for k in range(n_mane):
        t = (k + 0.5) / n_mane
        mx = NECK_BASE_X + t * (NECK_TOP_X - NECK_BASE_X) - 0.04
        mz = NECK_BASE_Z + t * (NECK_TOP_Z - NECK_BASE_Z) + 0.10
        mane_bump = SphereGeometry(
            radius=0.032, width_segments=10, height_segments=8
        )
        mane_bump.translate(mx, 0.0, mz)
        rocker.visual(
            mesh_from_geometry(mane_bump, f"mane_{k}"),
            material="horse_dark",
            name=f"mane_{k}",
        )

    # --- Tail: curved tube at the rear ---
    tail_pts = [
        (TAIL_X, 0.0, TAIL_Z),
        (TAIL_X - 0.08, 0.0, TAIL_Z + 0.08),
        (TAIL_X - 0.14, 0.0, TAIL_Z + 0.18),
        (TAIL_X - 0.12, 0.0, TAIL_Z + 0.28),
    ]
    rocker.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                tail_pts, radius=0.028,
                samples_per_segment=8, radial_segments=14,
            ),
            "horse_tail",
        ),
        material="horse_dark",
        name="horse_tail",
    )

    # --- Tail tuft: small sphere at end ---
    tuft = SphereGeometry(radius=0.04, width_segments=10, height_segments=8)
    tuft.translate(TAIL_X - 0.12, 0.0, TAIL_Z + 0.28)
    rocker.visual(
        mesh_from_geometry(tuft, "tail_tuft"),
        material="horse_dark",
        name="tail_tuft",
    )

    # --- Seat plates with mount tubes (blue molded seats on body) ---
    seat_profile = sample_catmull_rom_spline_2d(
        [
            (0.14, 0.0),
            (0.04, 0.10),
            (-0.06, 0.12),
            (-0.14, 0.08),
            (-0.14, -0.08),
            (-0.06, -0.12),
            (0.04, -0.10),
        ],
        samples_per_segment=8,
        closed=True,
    )
    for i, (sx, seat_x) in enumerate(((1.0, SEAT_X), (-1.0, SEAT_1_X))):
        # Seat mount tube connecting body top to seat plate
        rocker.visual(
            Cylinder(radius=0.05, length=0.06),
            origin=Origin(xyz=(seat_x, 0.0, SEAT_Z - 0.025)),
            material="horse_yellow",
            name=f"seat_mount_{i}",
        )
        # Seat plate
        seat = ExtrudeGeometry(seat_profile, 0.015, cap=True, center=True)
        if sx < 0:
            seat.rotate_z(math.pi)
        seat.translate(seat_x, 0.0, SEAT_Z + 0.005)
        rocker.visual(
            mesh_from_geometry(seat, f"seat_plate_{i}"),
            material="seat_blue",
            name=f"seat_plate_{i}",
        )
        # Seat back (small vertical wall behind the seat)
        rocker.visual(
            Box((0.02, 0.18, 0.10)),
            origin=Origin(xyz=(seat_x - sx * 0.12, 0.0, SEAT_Z + 0.02)),
            material="seat_blue",
            name=f"seat_back_{i}",
        )

    # --- Handlebar posts and grip plates ---
    grip_outer = rounded_rect_profile(0.14, 0.22, 0.04)
    grip_hole = rounded_rect_profile(0.05, 0.07, 0.015)
    grip_holes = [
        [(hx, hy + 0.055) for hx, hy in grip_hole],
        [(hx, hy - 0.055) for hx, hy in grip_hole],
    ]
    for i, (sx, post_x) in enumerate(
        ((1.0, HANDLE_POST_X), (-1.0, HANDLE_1_POST_X))
    ):
        # Post from deep inside the body up to the grip plate
        post_pts = [
            (sx * 0.46, 0.0, BODY_Z + 0.02),
            (sx * 0.56, 0.0, BODY_Z + BODY_R + 0.06),
            (post_x, 0.0, HANDLE_Z - 0.02),
            (post_x, 0.0, HANDLE_Z + 0.02),
        ]
        rocker.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    post_pts, radius=0.018,
                    samples_per_segment=8, radial_segments=14,
                ),
                f"handle_post_{i}",
            ),
            material="horse_yellow",
            name=f"handle_post_{i}",
        )
        # Grip plate with hand holes
        grip = ExtrudeWithHolesGeometry(
            grip_outer, grip_holes, 0.012, cap=True, center=True
        )
        grip.translate(post_x, 0.0, HANDLE_Z + 0.02)
        rocker.visual(
            mesh_from_geometry(grip, f"handle_plate_{i}"),
            material="handle_grip",
            name=f"handle_plate_{i}",
        )

    # --- Textured footrests (flat plates with grip bumps on body sides) ---
    for i, (sx, fr_x) in enumerate(((1.0, FOOTREST_X), (-1.0, FOOTREST_1_X))):
        for j, sy in enumerate((1.0, -1.0)):
            # Main footrest plate (partially embedded in body for connectivity)
            rocker.visual(
                Box((0.14, 0.10, 0.014)),
                origin=Origin(xyz=(fr_x, sy * FOOTREST_Y, FOOTREST_Z)),
                material="footrest_tex",
                name=f"footrest_{i}_{j}",
            )
            # Texture bumps (raised cylinders on the plate surface)
            for bk in range(3):
                for bl in range(2):
                    bx = fr_x - 0.04 + bk * 0.04
                    by = sy * FOOTREST_Y - 0.02 + bl * 0.04
                    rocker.visual(
                        Cylinder(radius=0.008, length=0.008),
                        origin=Origin(
                            xyz=(bx, by, FOOTREST_Z + 0.008)
                        ),
                        material="footrest_tex",
                        name=f"footrest_bump_{i}_{j}_{bk}_{bl}",
                    )

    # --- Bumper mount bosses on the rocker body ---
    # Tall cylinders bridging from the body underside downward; bumpers
    # seat below these bosses via prismatic compression joints.
    for i, bx in enumerate((BUMPER_X, BUMPER_1_X)):
        cap_dist = abs(bx) - BODY_CYL / 2.0
        if cap_dist > 0:
            local_r = math.sqrt(max(0, BODY_R * BODY_R - cap_dist * cap_dist))
        else:
            local_r = BODY_R
        body_bottom = BODY_Z - local_r
        # Boss extends from well below body up into the body for connectivity
        boss_bottom = body_bottom - 0.08
        boss_top = body_bottom + 0.02
        boss_len = boss_top - boss_bottom
        boss_cz = 0.5 * (boss_bottom + boss_top)
        rocker.visual(
            Cylinder(radius=BUMPER_R + 0.008, length=boss_len),
            origin=Origin(xyz=(bx, 0.0, boss_cz)),
            material="horse_yellow",
            name=f"bumper_boss_{i}",
        )

    # -----------------------------------------------------------------
    # Bumper parts: rubber end bumpers on prismatic compression joints.
    # Each bumper hangs below its boss and compresses upward (+Z).
    # -----------------------------------------------------------------
    for i, bx in enumerate((BUMPER_X, BUMPER_1_X)):
        cap_dist = abs(bx) - BODY_CYL / 2.0
        if cap_dist > 0:
            local_r = math.sqrt(max(0, BODY_R * BODY_R - cap_dist * cap_dist))
        else:
            local_r = BODY_R
        body_bottom = BODY_Z - local_r
        # Mount well below body underside so pad clears the curved torso
        mount_z = body_bottom - 0.08

        bumper = model.part(f"bumper_{i}")
        # Rubber bumper pad (hangs below the mount point)
        bumper.visual(
            Cylinder(radius=BUMPER_R, length=BUMPER_H),
            origin=Origin(xyz=(0.0, 0.0, -BUMPER_H / 2.0)),
            material="dark_rubber",
            name=f"bumper_pad_{i}",
        )
        # Bumper retainer ring (seats into the boss, overlaps with pad top)
        bumper.visual(
            Cylinder(radius=BUMPER_R + 0.005, length=0.014),
            origin=Origin(xyz=(0.0, 0.0, -0.002)),
            material="matte_black",
            name=f"bumper_ring_{i}",
        )

        # Prismatic joint: positive q compresses bumper upward (+Z)
        model.articulation(
            f"bumper_{i}_compress",
            ArticulationType.PRISMATIC,
            parent=rocker,
            child=bumper,
            origin=Origin(xyz=(bx, 0.0, mount_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=800.0, velocity=0.5,
                lower=0.0, upper=BUMPER_TRAVEL,
            ),
        )

    # -----------------------------------------------------------------
    # Main rocking pivot: horizontal axis across the seesaw width (Y).
    # -----------------------------------------------------------------
    model.articulation(
        "rocker_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=rocker,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=400.0, velocity=1.5,
            lower=-ROCK_LIMIT, upper=ROCK_LIMIT,
        ),
    )

    return model


def _intersects(a, b, tol: float = 1e-4) -> bool:
    if a is None or b is None:
        return False
    return all(a[0][i] <= b[1][i] + tol and b[0][i] <= a[1][i] + tol for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_support")
    rocker = object_model.get_part("rocker_body")
    pivot = object_model.get_articulation("rocker_pivot")
    bumper_0 = object_model.get_part("bumper_0")
    bumper_1 = object_model.get_part("bumper_1")
    bump_0_joint = object_model.get_articulation("bumper_0_compress")
    bump_1_joint = object_model.get_articulation("bumper_1_compress")

    # --- Pivot stub captured in bracket (intentional nesting) ---
    ctx.allow_overlap(
        rocker, base,
        elem_a="pivot_stub", elem_b="pivot_bracket",
        reason="The pivot stub descends into the cast pivot bracket that captures the rocking axle.",
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

    # --- Bumper pads seat into rocker bosses (intentional nesting) ---
    for i in range(2):
        ctx.allow_overlap(
            rocker, object_model.get_part(f"bumper_{i}"),
            elem_a=f"bumper_boss_{i}", elem_b=f"bumper_ring_{i}",
            reason=f"Bumper {i} retainer ring seats into the rocker boss for guided compression.",
        )
        ctx.expect_overlap(
            rocker, object_model.get_part(f"bumper_{i}"),
            axes="z",
            elem_a=f"bumper_boss_{i}", elem_b=f"bumper_ring_{i}",
            min_overlap=0.005,
            name=f"bumper {i} ring seated in boss",
        )

    # --- Animal body reads as a horse shape ---
    torso = ctx.part_element_world_aabb(rocker, elem="horse_torso")
    head = ctx.part_element_world_aabb(rocker, elem="horse_head")
    snout = ctx.part_element_world_aabb(rocker, elem="horse_snout")
    ctx.check(
        "horse torso spans the seesaw length",
        torso is not None and (torso[1][0] - torso[0][0]) >= 1.3,
        details=f"torso={torso}",
    )
    ctx.check(
        "horse head is above and forward of the torso center",
        head is not None and torso is not None
        and head[0][0] > 0.4
        and (head[0][2] + head[1][2]) / 2 > (torso[0][2] + torso[1][2]) / 2,
        details=f"head={head}, torso={torso}",
    )
    ctx.check(
        "snout extends forward of the head center",
        snout is not None and head is not None
        and (snout[0][0] + snout[1][0]) / 2 > (head[0][0] + head[1][0]) / 2,
        details=f"snout={snout}, head={head}",
    )

    # --- Ears on top of head ---
    ear_0 = ctx.part_element_world_aabb(rocker, elem="horse_ear_0")
    ear_1 = ctx.part_element_world_aabb(rocker, elem="horse_ear_1")
    ctx.check(
        "ears are above the head center",
        ear_0 is not None and ear_1 is not None and head is not None
        and ear_0[0][2] > (head[0][2] + head[1][2]) / 2
        and ear_1[0][2] > (head[0][2] + head[1][2]) / 2,
        details=f"ear_0={ear_0}, ear_1={ear_1}, head={head}",
    )

    # --- Tail at rear ---
    tail = ctx.part_element_world_aabb(rocker, elem="horse_tail")
    ctx.check(
        "tail is at the rear of the body",
        tail is not None and torso is not None
        and (tail[0][0] + tail[1][0]) / 2 < (torso[0][0] + torso[1][0]) / 2,
        details=f"tail={tail}, torso={torso}",
    )

    # --- Ground pads under support legs ---
    pad_0 = ctx.part_element_world_aabb(base, elem="ground_pad_0")
    pad_1 = ctx.part_element_world_aabb(base, elem="ground_pad_1")
    ctx.check(
        "rubber ground pads exist near ground level",
        pad_0 is not None and pad_1 is not None
        and pad_0[1][2] < 0.03
        and pad_1[1][2] < 0.03,
        details=f"pad_0={pad_0}, pad_1={pad_1}",
    )

    # --- Textured footrests near each seat ---
    fr_0_0 = ctx.part_element_world_aabb(rocker, elem="footrest_0_0")
    fr_1_0 = ctx.part_element_world_aabb(rocker, elem="footrest_1_0")
    ctx.check(
        "textured footrests exist near both seats on body sides",
        fr_0_0 is not None and fr_1_0 is not None
        and fr_0_0[0][0] > 0.2
        and fr_1_0[1][0] < -0.2,
        details=f"fr_0_0={fr_0_0}, fr_1_0={fr_1_0}",
    )

    # --- Bumpers exist at both ends ---
    bp0 = ctx.part_world_aabb(bumper_0)
    bp1 = ctx.part_world_aabb(bumper_1)
    ctx.check(
        "rubber bumpers at both ends of the seesaw",
        bp0 is not None and bp1 is not None
        and bp0[1][0] > 0.4
        and bp1[0][0] < -0.4,
        details=f"bp0={bp0}, bp1={bp1}",
    )

    # --- Prismatic joints: bumper compression range ---
    lim0 = bump_0_joint.motion_limits
    lim1 = bump_1_joint.motion_limits
    ctx.check(
        "bumper 0 has prismatic compression range",
        lim0 is not None
        and abs(lim0.lower) < 0.005
        and 0.015 <= lim0.upper <= 0.04,
        details=f"limits=({lim0.lower}, {lim0.upper})",
    )
    ctx.check(
        "bumper 1 has prismatic compression range",
        lim1 is not None
        and abs(lim1.lower) < 0.005
        and 0.015 <= lim1.upper <= 0.04,
        details=f"limits=({lim1.lower}, {lim1.upper})",
    )

    # --- Bumper compression proof: positive q moves bumper upward ---
    bp0_rest = ctx.part_world_position(bumper_0)
    with ctx.pose({bump_0_joint: BUMPER_TRAVEL}):
        bp0_compressed = ctx.part_world_position(bumper_0)
        ctx.check(
            "bumper 0 compresses upward at max travel",
            bp0_rest is not None and bp0_compressed is not None
            and bp0_compressed[2] > bp0_rest[2] + 0.01,
            details=f"rest={bp0_rest}, compressed={bp0_compressed}",
        )

    # --- Seats at opposite ends ---
    seat0 = ctx.part_element_world_aabb(rocker, elem="seat_plate_0")
    seat1 = ctx.part_element_world_aabb(rocker, elem="seat_plate_1")
    ctx.check(
        "seats at opposite ends of the horse body",
        seat0 is not None and seat1 is not None
        and seat0[0][0] > 0.3
        and seat1[1][0] < -0.3,
        details=f"seat0={seat0}, seat1={seat1}",
    )

    # --- Handle grips above the body ---
    grip0 = ctx.part_element_world_aabb(rocker, elem="handle_plate_0")
    grip1 = ctx.part_element_world_aabb(rocker, elem="handle_plate_1")
    ctx.check(
        "handle grips above the horse body top",
        grip0 is not None and grip1 is not None and torso is not None
        and grip0[0][2] > torso[1][2] - 0.05
        and grip1[0][2] > torso[1][2] - 0.05,
        details=f"grip0={grip0}, grip1={grip1}, torso={torso}",
    )

    # --- Rocking range about +/- 15 degrees ---
    plim = pivot.motion_limits
    ctx.check(
        "rocking range about +/- 15 degrees",
        plim is not None
        and abs(plim.lower + ROCK_LIMIT) < 0.02
        and abs(plim.upper - ROCK_LIMIT) < 0.02,
        details=f"limits=({plim.lower}, {plim.upper})",
    )

    # --- Decisive pose checks: rocker tilts, base stays fixed ---
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
            rocker_up is not None and rocker_up[0][2] > 0.005,
            details=f"rocker={rocker_up}",
        )

    return ctx.report()


object_model = build_object_model()
