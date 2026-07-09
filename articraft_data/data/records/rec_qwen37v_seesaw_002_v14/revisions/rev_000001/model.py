from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CylinderGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ----------------------------------------------------------------------------
# Animal-shaped toddler seesaw (horse theme) with central pivot.
#
# Layout (world frame, Z up, base centered on the origin):
# - Sky-blue tube base: single low arched inverted-U support (~0.38 m tall)
#   with rubber ground pads under each foot.
# - One horse-shaped rocking beam (~1.30 m long) on a central revolute pivot:
#   main body tube, neck/head at front, tail at rear, seats at each end,
#   T-handlebars, and textured footrest platforms near each seat.
# - Two rubber end bumpers, each on a short prismatic joint (vertical axis,
#   0 to 0.03 m compression travel) mounted under the beam ends.
# - Articulation: beam rocks +/- 15 degrees about the horizontal pivot axis.
# ----------------------------------------------------------------------------

TUBE_R = 0.018       # main tubing (slightly smaller for toddler scale)
BRACE_R = 0.014
HANDLE_R = 0.014
SUPPORT_R = 0.016

TILT = math.radians(15.0)  # rocking range

# Base dimensions (toddler scale)
ARCH_TOP = 0.38          # pivot height
ARCH_HALF_SPAN = 0.24    # ground half-span of arch
ARCH_SHOULDER = 0.30     # shoulder height

# Beam dimensions
BEAM_LEN = 1.30
BODY_Z = 0.06           # body tube above pivot axis
SEAT_X = 0.52           # seat center from pivot
SEAT_Z = 0.040          # seat height relative to pivot
SEAT_SIZE = (0.22, 0.24, 0.012)
HANDLE_X = 0.38         # handlebar post position
HANDLE_TOP_Z = 0.28     # crossbar height above pivot

# Neck/head
NECK_BASE_X = 0.56      # where neck meets body
NECK_TOP_X = 0.62
NECK_TOP_Z = 0.28
HEAD_LEN = 0.16

# Tail
TAIL_BASE_X = -0.56
TAIL_TIP_X = -0.66
TAIL_TIP_Z = 0.18

# Footrest
FOOTREST_X = 0.42       # position along beam
FOOTREST_Z = -0.04      # below pivot axis
FOOTREST_SIZE = (0.10, 0.12, 0.008)

# Bumper
BUMPER_SIZE = (0.08, 0.08, 0.04)
BUMPER_COMPRESS = 0.03  # max compression travel

# Ground pad
PAD_SIZE = (0.12, 0.10, 0.015)

# Materials
SKY_BLUE = Material("sky_blue_paint", rgba=(0.33, 0.62, 0.84, 1.0))
HORSE_YELLOW = Material("horse_yellow", rgba=(0.90, 0.78, 0.20, 1.0))
RUST_BROWN = Material("rust_brown", rgba=(0.42, 0.21, 0.13, 1.0))
RUBBER_BLACK = Material("rubber_black", rgba=(0.12, 0.12, 0.12, 1.0))
RUBBER_GRAY = Material("rubber_gray", rgba=(0.25, 0.25, 0.25, 1.0))
HANDLE_RED = Material("handle_red", rgba=(0.75, 0.18, 0.15, 1.0))


def _tube_between(
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    radius: float,
    *,
    radial_segments: int = 16,
) -> MeshGeometry:
    """Straight capped tube between two 3D points."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    geom = CylinderGeometry(radius, length, radial_segments=radial_segments)
    ux, uy, uz = dx / length, dy / length, dz / length
    ax, ay, az = -uy, ux, 0.0
    s = math.sqrt(ax * ax + ay * ay + az * az)
    if s > 1e-9:
        geom.rotate((ax / s, ay / s, az / s), math.atan2(s, uz))
    elif uz < 0.0:
        geom.rotate_x(math.pi)
    geom.translate(
        (p0[0] + p1[0]) / 2.0,
        (p0[1] + p1[1]) / 2.0,
        (p0[2] + p1[2]) / 2.0,
    )
    return geom


def _base_arch_mesh() -> MeshGeometry:
    """Single inverted-U arch for the toddler base."""
    profile_uz = [
        (-ARCH_HALF_SPAN - 0.04, 0.015),
        (-ARCH_HALF_SPAN - 0.02, 0.022),
        (-0.22, 0.08),
        (-0.16, ARCH_SHOULDER),
        (-0.08, ARCH_TOP),
        (0.0, ARCH_TOP),
        (0.08, ARCH_TOP),
        (0.16, ARCH_SHOULDER),
        (0.22, 0.08),
        (ARCH_HALF_SPAN + 0.02, 0.022),
        (ARCH_HALF_SPAN + 0.04, 0.015),
    ]
    # Arch is in the YZ plane (perpendicular to beam direction X)
    points = [(0.0, u, z) for (u, z) in profile_uz]
    return tube_from_spline_points(
        points,
        radius=TUBE_R,
        samples_per_segment=10,
        radial_segments=16,
        cap_ends=True,
    )


def _horse_body_mesh() -> MeshGeometry:
    """Horse-shaped beam: body tube, neck/head, tail, seat supports, footrest mounts."""
    # Main body tube along X axis
    body = (
        CylinderGeometry(TUBE_R, BEAM_LEN, radial_segments=18)
        .rotate_y(math.pi / 2.0)
        .translate(0.0, 0.0, BODY_Z)
    )

    # Diagonal braces from pivot area up to body tube (triangulated truss)
    for sx in (1.0, -1.0):
        body.merge(
            _tube_between(
                (sx * 0.03, 0.0, 0.005),
                (sx * 0.40, 0.0, BODY_Z),
                BRACE_R,
            )
        )

    # Neck: tube angling up from front body end
    body.merge(
        tube_from_spline_points(
            [
                (NECK_BASE_X, 0.0, BODY_Z),
                (NECK_BASE_X + 0.02, 0.0, BODY_Z + 0.06),
                (NECK_TOP_X - 0.02, 0.0, NECK_TOP_Z - 0.04),
                (NECK_TOP_X, 0.0, NECK_TOP_Z),
            ],
            radius=TUBE_R * 0.9,
            samples_per_segment=8,
            radial_segments=14,
            cap_ends=True,
        )
    )

    # Head: horizontal tube extending forward from neck top
    body.merge(
        _tube_between(
            (NECK_TOP_X, 0.0, NECK_TOP_Z),
            (NECK_TOP_X + HEAD_LEN, 0.0, NECK_TOP_Z - 0.02),
            TUBE_R * 0.85,
        )
    )

    # Ears: two small tubes rising from head (base inside neck tube for contact)
    for sy in (0.010, -0.010):
        body.merge(
            _tube_between(
                (NECK_TOP_X + 0.04, sy, NECK_TOP_Z),
                (NECK_TOP_X + 0.03, sy * 2.5, NECK_TOP_Z + 0.06),
                BRACE_R * 0.7,
            )
        )

    # Snout: sphere at head front, overlapping the head tube end for contact
    snout = SphereGeometry(TUBE_R * 1.4, width_segments=14, height_segments=10)
    snout.translate(NECK_TOP_X + HEAD_LEN - 0.01, 0.0, NECK_TOP_Z - 0.02)
    body.merge(snout)

    # Tail: curved tube at rear
    body.merge(
        tube_from_spline_points(
            [
                (TAIL_BASE_X, 0.0, BODY_Z),
                (TAIL_BASE_X - 0.03, 0.0, BODY_Z + 0.04),
                (TAIL_TIP_X + 0.02, 0.02, TAIL_TIP_Z + 0.02),
                (TAIL_TIP_X, 0.03, TAIL_TIP_Z),
            ],
            radius=TUBE_R * 0.75,
            samples_per_segment=8,
            radial_segments=12,
            cap_ends=True,
        )
    )

    # Seat support tubes dropping from body to seat plates
    for sx in (1.0, -1.0):
        body.merge(
            tube_from_spline_points(
                [
                    (sx * 0.44, 0.0, BODY_Z),
                    (sx * 0.48, 0.0, BODY_Z - 0.02),
                    (sx * SEAT_X, 0.0, SEAT_Z + 0.010),
                ],
                radius=SUPPORT_R,
                samples_per_segment=6,
                radial_segments=12,
                cap_ends=True,
            )
        )

    # Footrest support stubs (short tubes from body down to footrest platform)
    for sx in (1.0, -1.0):
        body.merge(
            _tube_between(
                (sx * FOOTREST_X, 0.0, BODY_Z - TUBE_R),
                (sx * FOOTREST_X, 0.0, FOOTREST_Z + FOOTREST_SIZE[2] / 2.0),
                BRACE_R * 0.8,
            )
        )

    # Bumper mount brackets: short tubes from body tube down at beam ends
    # These carry the rubber bumpers below the seat plate level
    bumper_mount_bottom = SEAT_Z - SEAT_SIZE[2] / 2.0 - 0.006  # below seat plate
    for sx in (1.0, -1.0):
        body.merge(
            _tube_between(
                (sx * 0.60, 0.0, BODY_Z - TUBE_R),
                (sx * 0.60, 0.0, bumper_mount_bottom),
                BRACE_R,
            )
        )

    return body


def _handlebar_mesh(sx: float) -> MeshGeometry:
    """T-shaped handlebar post and crossbar."""
    post = CylinderGeometry(HANDLE_R, 0.22, radial_segments=12).translate(
        sx * HANDLE_X, 0.0, BODY_Z + 0.10
    )
    bar = (
        CylinderGeometry(HANDLE_R, 0.24, radial_segments=12)
        .rotate_x(math.pi / 2.0)
        .translate(sx * HANDLE_X, 0.0, HANDLE_TOP_Z)
    )
    # Grip spheres at bar ends
    for sy in (0.12, -0.12):
        grip = SphereGeometry(HANDLE_R * 1.3, width_segments=10, height_segments=6)
        grip.translate(sx * HANDLE_X, sy, HANDLE_TOP_Z)
        bar.merge(grip)
    return post.merge(bar)


def _footrest_mesh(sx: float) -> MeshGeometry:
    """Textured footrest platform: flat plate with raised grip ridges."""
    plate = BoxGeometry(FOOTREST_SIZE)
    # Add grip ridges (small raised bars on top face)
    ridge_count = 4
    ridge_spacing = FOOTREST_SIZE[0] * 0.8 / ridge_count
    for i in range(ridge_count):
        rx = -FOOTREST_SIZE[0] * 0.4 + (i + 0.5) * ridge_spacing
        ridge = BoxGeometry((0.006, FOOTREST_SIZE[1] * 0.85, 0.004))
        ridge.translate(rx, 0.0, FOOTREST_SIZE[2] / 2.0 + 0.002)
        plate.merge(ridge)
    plate.translate(sx * FOOTREST_X, 0.0, FOOTREST_Z)
    return plate


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="animal_toddler_seesaw")

    # --- static base with ground pads ----------------------------------------
    base = model.part("base")
    base.visual(
        mesh_from_geometry(_base_arch_mesh(), "base_arch"),
        material=SKY_BLUE,
        name="arch",
    )

    # Short cross brace for structural rigidity (endpoints reach into the arch legs)
    brace = _tube_between(
        (0.0, -0.24, 0.10),
        (0.0, 0.24, 0.10),
        BRACE_R,
    )
    base.visual(
        mesh_from_geometry(brace, "cross_brace"),
        material=SKY_BLUE,
        name="cross_brace",
    )

    # Rubber ground pads under each foot
    for idx, sy in enumerate((1.0, -1.0)):
        pad_geom = BoxGeometry(PAD_SIZE)
        pad_geom.translate(0.0, sy * (ARCH_HALF_SPAN + 0.03), PAD_SIZE[2] / 2.0)
        base.visual(
            mesh_from_geometry(pad_geom, f"ground_pad_{idx}"),
            material=RUBBER_BLACK,
            name=f"ground_pad_{idx}",
        )

    # --- horse-shaped rocking beam -------------------------------------------
    beam = model.part("beam")
    beam.visual(
        mesh_from_geometry(_horse_body_mesh(), "horse_body"),
        material=HORSE_YELLOW,
        name="horse_body",
    )

    # Handlebars
    for idx, sx in enumerate((1.0, -1.0)):
        beam.visual(
            mesh_from_geometry(_handlebar_mesh(sx), f"handlebar_{idx}"),
            material=HANDLE_RED,
            name=f"handlebar_{idx}",
        )

    # Seat plates
    for idx, sx in enumerate((1.0, -1.0)):
        beam.visual(
            Box(SEAT_SIZE),
            origin=Origin(xyz=(sx * SEAT_X, 0.0, SEAT_Z)),
            material=RUST_BROWN,
            name=f"seat_plate_{idx}",
        )

    # Textured footrests
    for idx, sx in enumerate((1.0, -1.0)):
        beam.visual(
            mesh_from_geometry(_footrest_mesh(sx), f"footrest_{idx}"),
            material=RUBBER_GRAY,
            name=f"footrest_{idx}",
        )

    # Axle sleeve at pivot
    sleeve = (
        CylinderGeometry(0.028, 0.10, radial_segments=18)
        .rotate_x(math.pi / 2.0)
    )
    weld_post = CylinderGeometry(0.012, BODY_Z - 0.020, radial_segments=12).translate(
        0.0, 0.0, (BODY_Z + 0.020) / 2.0
    )
    sleeve.merge(weld_post)
    beam.visual(
        mesh_from_geometry(sleeve, "axle_sleeve"),
        material=HORSE_YELLOW,
        name="axle_sleeve",
    )

    # --- rubber end bumpers on prismatic joints ------------------------------
    bumper_contact_z = SEAT_Z - SEAT_SIZE[2] / 2.0 - 0.006  # bracket bottom
    for idx, sx in enumerate((1.0, -1.0)):
        bumper = model.part(f"bumper_{idx}")
        # Bumper geometry extends downward from origin (top face at z=0)
        bumper_geom = BoxGeometry(BUMPER_SIZE)
        bumper_geom.translate(0.0, 0.0, -BUMPER_SIZE[2] / 2.0)
        bumper.visual(
            mesh_from_geometry(bumper_geom, f"bumper_pad_{idx}"),
            material=RUBBER_BLACK,
            name=f"bumper_pad_{idx}",
        )

        # Prismatic joint: bumper contacts bracket bottom, compresses upward
        model.articulation(
            f"bumper_{idx}_compress",
            ArticulationType.PRISMATIC,
            parent=beam,
            child=bumper,
            origin=Origin(xyz=(sx * 0.60, 0.0, bumper_contact_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=50.0,
                velocity=0.5,
                lower=0.0,
                upper=BUMPER_COMPRESS,
            ),
        )

    # --- main beam pivot (revolute) ------------------------------------------
    model.articulation(
        "beam_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, ARCH_TOP)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=2.0,
            lower=-TILT,
            upper=TILT,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    beam = object_model.get_part("beam")
    bumper_0 = object_model.get_part("bumper_0")
    bumper_1 = object_model.get_part("bumper_1")
    beam_pivot = object_model.get_articulation("beam_pivot")
    bumper_0_joint = object_model.get_articulation("bumper_0_compress")
    bumper_1_joint = object_model.get_articulation("bumper_1_compress")

    # --- animal shape: horse body with neck/head and tail features -----------
    horse_aabb = ctx.part_element_world_aabb(beam, elem="horse_body")
    ctx.check(
        "horse body spans roughly 1.2-1.4 m along the beam axis",
        horse_aabb is not None
        and 1.1 <= (horse_aabb[1][0] - horse_aabb[0][0]) <= 1.5,
        details=f"horse_body aabb={horse_aabb}",
    )
    ctx.check(
        "horse head rises above the body tube (neck extends upward)",
        horse_aabb is not None
        and horse_aabb[1][2] > ARCH_TOP + 0.20,
        details=f"horse_body top z={horse_aabb[1][2] if horse_aabb else None}",
    )

    # --- central pivot is revolute with correct limits ----------------------
    lim = beam_pivot.motion_limits
    ctx.check(
        "beam pivot is revolute with +/- 15 degree limits",
        lim is not None
        and abs(lim.lower + TILT) < 1e-6
        and abs(lim.upper - TILT) < 1e-6,
        details=f"limits=({lim.lower if lim else None}, {lim.upper if lim else None})",
    )

    # --- rubber end bumpers on prismatic joints -----------------------------
    for bjoint, bname in ((bumper_0_joint, "bumper_0"), (bumper_1_joint, "bumper_1")):
        blim = bjoint.motion_limits
        ctx.check(
            f"{bname} compress joint is prismatic with 0 to {BUMPER_COMPRESS} m travel",
            blim is not None
            and abs(blim.lower) < 1e-6
            and abs(blim.upper - BUMPER_COMPRESS) < 1e-6,
            details=f"limits=({blim.lower if blim else None}, {blim.upper if blim else None})",
        )

    # Bumper pads exist below beam ends
    for idx in (0, 1):
        bpad = ctx.part_element_world_aabb(object_model.get_part(f"bumper_{idx}"), elem=f"bumper_pad_{idx}")
        ctx.check(
            f"bumper_{idx} pad hangs below the beam end",
            bpad is not None and bpad[0][2] < ARCH_TOP - 0.01,
            details=f"bumper_pad_{idx} aabb={bpad}",
        )

    # --- rubber ground pads under base legs ---------------------------------
    for idx in (0, 1):
        pad = ctx.part_element_world_aabb(base, elem=f"ground_pad_{idx}")
        ctx.check(
            f"ground_pad_{idx} sits at ground level under a base leg",
            pad is not None and pad[0][2] < 0.02 and pad[1][2] < 0.04,
            details=f"ground_pad_{idx} aabb={pad}",
        )

    # --- textured footrests near each seat ----------------------------------
    for idx in (0, 1):
        footrest = ctx.part_element_world_aabb(beam, elem=f"footrest_{idx}")
        seat = ctx.part_element_world_aabb(beam, elem=f"seat_plate_{idx}")
        ctx.check(
            f"footrest_{idx} exists near seat_plate_{idx}",
            footrest is not None and seat is not None,
            details=f"footrest={footrest}, seat={seat}",
        )
        if footrest and seat:
            # Footrest should be inboard of seat along beam axis
            fcx = (footrest[0][0] + footrest[1][0]) / 2.0
            scx = (seat[0][0] + seat[1][0]) / 2.0
            ctx.check(
                f"footrest_{idx} is inboard of its seat along the beam",
                abs(fcx) < abs(scx) + 0.01,
                details=f"footrest_x={fcx:.3f}, seat_x={scx:.3f}",
            )

    # --- base is a low arch stand for toddlers ------------------------------
    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "base is a low arch stand about 0.35-0.42 m tall",
        base_aabb is not None and 0.34 <= base_aabb[1][2] <= 0.44,
        details=f"base aabb={base_aabb}",
    )

    # --- captured-axle fit: sleeve wraps the arch top -----------------------
    ctx.allow_overlap(
        beam,
        base,
        elem_a="axle_sleeve",
        elem_b="arch",
        reason="Beam axle sleeve intentionally wraps the arch top tube as its pivot axle.",
    )
    ctx.expect_contact(
        beam,
        base,
        elem_a="axle_sleeve",
        elem_b="arch",
        name="beam sleeve rides on the arch axle",
    )

    # --- decisive pose: beam seesaws, one end rises while other drops -------
    rest_seat0 = ctx.part_element_world_aabb(beam, elem="seat_plate_0")
    rest_seat1 = ctx.part_element_world_aabb(beam, elem="seat_plate_1")

    with ctx.pose({beam_pivot: TILT}):
        tilt_seat0 = ctx.part_element_world_aabb(beam, elem="seat_plate_0")
        tilt_seat1 = ctx.part_element_world_aabb(beam, elem="seat_plate_1")
        beam_aabb = ctx.part_world_aabb(beam)
        ctx.check(
            "beam seesaws: front seat drops, rear seat rises",
            rest_seat0 is not None
            and tilt_seat0 is not None
            and rest_seat1 is not None
            and tilt_seat1 is not None
            and tilt_seat0[0][2] < rest_seat0[0][2] - 0.10
            and tilt_seat1[0][2] > rest_seat1[0][2] + 0.10,
            details=f"seat0 {rest_seat0} -> {tilt_seat0}, seat1 {rest_seat1} -> {tilt_seat1}",
        )
        ctx.check(
            "tilted beam stays clear of the ground",
            beam_aabb is not None and beam_aabb[0][2] > -0.01,
            details=f"beam aabb={beam_aabb}",
        )
        ctx.expect_contact(
            beam,
            base,
            elem_a="axle_sleeve",
            elem_b="arch",
            name="tilted beam sleeve stays on its axle",
        )

    with ctx.pose({beam_pivot: -TILT}):
        tilt_seat0_neg = ctx.part_element_world_aabb(beam, elem="seat_plate_0")
        tilt_seat1_neg = ctx.part_element_world_aabb(beam, elem="seat_plate_1")
        ctx.check(
            "beam seesaws opposite way: front rises, rear drops",
            rest_seat0 is not None
            and tilt_seat0_neg is not None
            and rest_seat1 is not None
            and tilt_seat1_neg is not None
            and tilt_seat0_neg[0][2] > rest_seat0[0][2] + 0.10
            and tilt_seat1_neg[0][2] < rest_seat1[0][2] - 0.10,
            details=f"seat0 {rest_seat0} -> {tilt_seat0_neg}, seat1 {rest_seat1} -> {tilt_seat1_neg}",
        )

    # --- prismatic bumper compression pose check ----------------------------
    with ctx.pose({bumper_0_joint: BUMPER_COMPRESS}):
        bp0 = ctx.part_element_world_aabb(bumper_0, elem="bumper_pad_0")
        bp0_rest = ctx.part_element_world_aabb(bumper_0, elem="bumper_pad_0")
        ctx.check(
            "bumper_0 compresses upward when prismatic joint is at upper limit",
            bp0 is not None,
            details=f"compressed bumper aabb={bp0}",
        )

    return ctx.report()


object_model = build_object_model()
