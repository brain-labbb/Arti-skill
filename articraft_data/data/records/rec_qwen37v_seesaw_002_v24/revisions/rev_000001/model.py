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
# Animal-shaped toddler seesaw (horse theme).
#
# Layout (world frame, Z up, base centered on the origin):
# - Central support base: a single sky-blue inverted-U tube arch (~0.35 m tall)
#   with rubber ground pads under each foot.
# - A single rocking beam shaped like a horse: main tube spine with a wider
#   body shell, a neck+head at the +X end, and a tail at the -X end.
#   Two flat seat plates sit on top of the body.
# - Two T-handlebars, each a separate part on its own revolute joint,
#   allowing slight grip pivoting (+/- 12 degrees).
# - Articulation: the beam connects to the base with a revolute joint
#   (horizontal axis perpendicular to beam length), +/- 18 degrees.
# ----------------------------------------------------------------------------

TUBE_R = 0.015       # ~30 mm main tubing (toddler scale)
BRACE_R = 0.012
SUPPORT_R = 0.014
HANDLE_R = 0.012

TILT = math.radians(18.0)   # beam rocking range
GRIP_TILT = math.radians(12.0)  # handlebar pivot range

# Base dimensions
ARCH_TOP = 0.35          # pivot height
ARCH_HALF_SPAN = 0.22    # ground half-span of the arch
ARCH_FOOT_FLARE = 0.04   # extra outward flare at feet

# Beam dimensions
BEAM_LEN = 1.50           # total beam spine length
MAIN_Z = 0.05             # main tube height above pivot axis
SLEEVE_R = 0.025
SLEEVE_LEN = 0.10

# Seat positions along beam from pivot
SEAT_X = 0.52             # seat center offset along beam
SEAT_Z = 0.085            # seat top height above pivot
SEAT_SIZE = (0.20, 0.22, 0.010)

# Handlebar positions
HANDLE_X = 0.36           # handlebar post, inboard of seat
BOSS_HEIGHT = 0.025       # mounting boss height on beam
HANDLE_POST_Z = MAIN_Z + BOSS_HEIGHT  # post base sits on mounting boss top
HANDLE_TOP_Z = MAIN_Z + 0.24   # crossbar height

# Horse body dimensions
BODY_LEN = 0.60           # body shell length along beam
BODY_WIDTH = 0.16         # body shell width
BODY_HEIGHT = 0.10        # body shell height

# Horse head/neck
NECK_LEN = 0.28
HEAD_SIZE = (0.14, 0.09, 0.10)

# Rubber pads
PAD_SIZE = (0.08, 0.08, 0.012)

# Materials
SKY_BLUE = Material("sky_blue_paint", rgba=(0.33, 0.62, 0.84, 1.0))
HORSE_YELLOW = Material("horse_yellow_paint", rgba=(0.92, 0.78, 0.15, 1.0))
HORSE_ORANGE = Material("horse_orange_paint", rgba=(0.88, 0.45, 0.12, 1.0))
SEAT_GREEN = Material("seat_green_paint", rgba=(0.25, 0.55, 0.30, 1.0))
HANDLE_RED = Material("handle_red_paint", rgba=(0.75, 0.15, 0.15, 1.0))
RUBBER_BLACK = Material("rubber_black", rgba=(0.12, 0.12, 0.12, 1.0))
RUST_BROWN = Material("rust_brown", rgba=(0.42, 0.21, 0.13, 1.0))


def _tube_between(
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    radius: float,
    *,
    radial_segments: int = 14,
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


def _arch_mesh(top_z: float) -> MeshGeometry:
    """Inverted-U arch tube in the XZ plane (axis along X)."""
    half = ARCH_HALF_SPAN
    flare = ARCH_FOOT_FLARE
    shoulder = top_z * 0.85
    profile_xz = [
        (-(half + flare + 0.03), 0.008),
        (-(half + flare), 0.015),
        (-(half + 0.02), 0.06),
        (-half, 0.12),
        (-half * 0.75, shoulder * 0.65),
        (-half * 0.45, shoulder),
        (-half * 0.15, top_z - 0.01),
        (0.0, top_z),
        (half * 0.15, top_z - 0.01),
        (half * 0.45, shoulder),
        (half * 0.75, shoulder * 0.65),
        (half, 0.12),
        (half + 0.02, 0.06),
        (half + flare, 0.015),
        (half + flare + 0.03, 0.008),
    ]
    points = [(x, 0.0, z) for (x, z) in profile_xz]
    return tube_from_spline_points(
        points,
        radius=TUBE_R,
        samples_per_segment=8,
        radial_segments=14,
        cap_ends=True,
    )


def _horse_body_mesh() -> MeshGeometry:
    """Build the horse body shell centered at origin, extending along X."""
    # Elliptical body shell using a scaled box + rounded edges
    body = BoxGeometry((BODY_LEN, BODY_WIDTH, BODY_HEIGHT))
    body.translate(0.0, 0.0, MAIN_Z + BODY_HEIGHT / 2.0)
    # Add a spine ridge on top
    ridge = BoxGeometry((BODY_LEN * 0.7, 0.03, 0.025))
    ridge.translate(0.0, 0.0, MAIN_Z + BODY_HEIGHT + 0.012)
    body.merge(ridge)
    return body


def _horse_head_mesh() -> MeshGeometry:
    """Horse head and neck at the +X end of the beam."""
    # Neck starts past the handlebar position, connecting to spine tube
    neck_x_start = HANDLE_X + 0.10
    neck_x_end = BEAM_LEN / 2.0 + NECK_LEN * 0.35
    # Neck: angled tube from spine tube top up to head
    neck = _tube_between(
        (neck_x_start, 0.0, MAIN_Z + TUBE_R),
        (neck_x_end, 0.0, MAIN_Z + BODY_HEIGHT + 0.18),
        radius=0.035,
        radial_segments=14,
    )
    # Head: box shape at top of neck
    head = BoxGeometry(HEAD_SIZE)
    head.translate(
        neck_x_end + HEAD_SIZE[0] * 0.25,
        0.0,
        MAIN_Z + BODY_HEIGHT + 0.22,
    )
    neck.merge(head)
    # Snout: smaller box extending forward
    snout = BoxGeometry((0.08, 0.065, 0.055))
    snout.translate(
        neck_x_end + HEAD_SIZE[0] * 0.25 + 0.09,
        0.0,
        MAIN_Z + BODY_HEIGHT + 0.18,
    )
    neck.merge(snout)
    # Ears: two small tubes on top of head (overlapping head box for connectivity)
    for sy in (1.0, -1.0):
        ear = CylinderGeometry(0.012, 0.045, radial_segments=8)
        ear.translate(
            neck_x_end + 0.02,
            sy * 0.03,
            MAIN_Z + BODY_HEIGHT + 0.25,
        )
        neck.merge(ear)
    return neck


def _horse_tail_mesh() -> MeshGeometry:
    """Horse tail at the -X end of the beam."""
    # Tail starts past the handlebar, connecting to spine tube
    tail_start_x = -(HANDLE_X + 0.10)
    tail = tube_from_spline_points(
        [
            (tail_start_x, 0.0, MAIN_Z + TUBE_R),
            (tail_start_x - 0.06, 0.0, MAIN_Z + BODY_HEIGHT + 0.06),
            (tail_start_x - 0.14, 0.02, MAIN_Z + BODY_HEIGHT + 0.14),
            (tail_start_x - 0.20, 0.0, MAIN_Z + BODY_HEIGHT + 0.10),
            (tail_start_x - 0.24, -0.01, MAIN_Z + BODY_HEIGHT + 0.04),
        ],
        radius=0.018,
        samples_per_segment=6,
        radial_segments=10,
        cap_ends=True,
    )
    return tail


def _handlebar_mesh() -> MeshGeometry:
    """T-shaped handlebar in local frame (post goes up from origin)."""
    post = CylinderGeometry(HANDLE_R, 0.22, radial_segments=12)
    post.translate(0.0, 0.0, 0.11)
    bar = CylinderGeometry(HANDLE_R, 0.22, radial_segments=12)
    bar.rotate_x(math.pi / 2.0)
    bar.translate(0.0, 0.0, 0.22)
    # Grip ends
    for sy in (1.0, -1.0):
        grip = CylinderGeometry(HANDLE_R * 1.4, 0.04, radial_segments=10)
        grip.rotate_x(math.pi / 2.0)
        grip.translate(0.0, sy * 0.12, 0.22)
        bar.merge(grip)
    post.merge(bar)
    return post


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="animal_toddler_seesaw")

    # --- static base with rubber ground pads ----------------------------------
    base = model.part("base")

    # Inverted-U arch (pivot support)
    base.visual(
        mesh_from_geometry(_arch_mesh(ARCH_TOP), "arch_tube"),
        material=SKY_BLUE,
        name="arch_tube",
    )

    # Cross brace for rigidity
    brace_z = ARCH_TOP * 0.55
    brace_half = ARCH_HALF_SPAN * 0.6
    for idx, sy in enumerate((1.0, -1.0)):
        # Side braces perpendicular to arch plane (along Y)
        brace = _tube_between(
            (-brace_half * sy, -0.06, brace_z),
            (-brace_half * sy, 0.06, brace_z),
            SUPPORT_R,
        )
        base.visual(
            mesh_from_geometry(brace, f"side_brace_{idx}"),
            material=SKY_BLUE,
            name=f"side_brace_{idx}",
        )

    # Rubber ground pads under each foot
    foot_x_positions = [
        -(ARCH_HALF_SPAN + ARCH_FOOT_FLARE),
        (ARCH_HALF_SPAN + ARCH_FOOT_FLARE),
    ]
    for idx, fx in enumerate(foot_x_positions):
        pad = BoxGeometry(PAD_SIZE)
        pad.translate(fx, 0.0, PAD_SIZE[2] / 2.0)
        base.visual(
            mesh_from_geometry(pad, f"rubber_pad_{idx}"),
            material=RUBBER_BLACK,
            name=f"rubber_pad_{idx}",
        )

    # --- rocking beam (horse body) -------------------------------------------
    beam = model.part("beam")

    # Main spine tube
    spine = CylinderGeometry(TUBE_R, BEAM_LEN, radial_segments=16)
    spine.rotate_y(math.pi / 2.0)
    spine.translate(0.0, 0.0, MAIN_Z)
    beam.visual(
        mesh_from_geometry(spine, "spine_tube"),
        material=HORSE_YELLOW,
        name="spine_tube",
    )

    # Diagonal braces from axle sleeve to spine (triangulated truss)
    truss = MeshGeometry()
    for sx in (1.0, -1.0):
        truss.merge(
            _tube_between(
                (sx * 0.04, 0.0, 0.030),
                (sx * 0.35, 0.0, MAIN_Z),
                BRACE_R,
            )
        )
    beam.visual(
        mesh_from_geometry(truss, "truss_braces"),
        material=HORSE_YELLOW,
        name="truss_braces",
    )

    # Axle sleeve at center
    sleeve = CylinderGeometry(SLEEVE_R, SLEEVE_LEN, radial_segments=18)
    sleeve.rotate_x(math.pi / 2.0)
    # Weld post connecting sleeve to spine (overlaps both for connectivity)
    wp_bot = SLEEVE_R - 0.003
    wp_top = MAIN_Z
    weld_post = CylinderGeometry(0.012, wp_top - wp_bot, radial_segments=10)
    weld_post.translate(0.0, 0.0, (wp_top + wp_bot) / 2.0)
    sleeve.merge(weld_post)
    beam.visual(
        mesh_from_geometry(sleeve, "axle_sleeve"),
        material=HORSE_YELLOW,
        name="axle_sleeve",
    )

    # Horse body shell
    beam.visual(
        mesh_from_geometry(_horse_body_mesh(), "horse_body"),
        material=HORSE_ORANGE,
        name="horse_body",
    )

    # Horse head/neck
    beam.visual(
        mesh_from_geometry(_horse_head_mesh(), "horse_head"),
        material=HORSE_ORANGE,
        name="horse_head",
    )

    # Horse tail
    beam.visual(
        mesh_from_geometry(_horse_tail_mesh(), "horse_tail"),
        material=HORSE_ORANGE,
        name="horse_tail",
    )

    # Two seat plates
    for idx, sx in enumerate((1.0, -1.0)):
        seat = Box(SEAT_SIZE)
        beam.visual(
            seat,
            origin=Origin(xyz=(sx * SEAT_X, 0.0, SEAT_Z)),
            material=SEAT_GREEN,
            name=f"seat_plate_{idx}",
        )

    # Seat support tubes
    seat_supports = MeshGeometry()
    for sx in (1.0, -1.0):
        seat_supports.merge(
            _tube_between(
                (sx * SEAT_X, 0.0, MAIN_Z + 0.005),
                (sx * SEAT_X, 0.0, SEAT_Z - SEAT_SIZE[2] / 2.0),
                SUPPORT_R,
            )
        )
    beam.visual(
        mesh_from_geometry(seat_supports, "seat_supports"),
        material=HORSE_YELLOW,
        name="seat_supports",
    )

    # Mounting bosses for handlebars (short posts on beam where handlebars mount)
    bosses = MeshGeometry()
    for sx in (1.0, -1.0):
        boss = CylinderGeometry(HANDLE_R * 1.8, BOSS_HEIGHT, radial_segments=12)
        boss.translate(sx * HANDLE_X, 0.0, MAIN_Z + BOSS_HEIGHT / 2.0)
        bosses.merge(boss)
    beam.visual(
        mesh_from_geometry(bosses, "handlebar_bosses"),
        material=HORSE_YELLOW,
        name="handlebar_bosses",
    )

    # --- handlebars as separate articulated parts ----------------------------
    for idx, sx in enumerate((1.0, -1.0)):
        hb = model.part(f"handlebar_{idx}")
        hb.visual(
            mesh_from_geometry(_handlebar_mesh(), f"handlebar_mesh_{idx}"),
            material=HANDLE_RED,
            name=f"grip_{idx}",
        )

    # --- articulations -------------------------------------------------------
    handlebar_0 = model.get_part("handlebar_0")
    handlebar_1 = model.get_part("handlebar_1")

    # Beam pivot: revolute, horizontal axis perpendicular to beam (Y axis)
    model.articulation(
        "beam_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, ARCH_TOP)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0, velocity=2.5, lower=-TILT, upper=TILT
        ),
    )

    # Handlebar joints: each pivots on its grip axis (along Y, perpendicular to beam)
    model.articulation(
        "handlebar_joint_0",
        ArticulationType.REVOLUTE,
        parent=beam,
        child=handlebar_0,
        origin=Origin(xyz=(HANDLE_X, 0.0, HANDLE_POST_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0, velocity=1.5, lower=-GRIP_TILT, upper=GRIP_TILT
        ),
    )
    model.articulation(
        "handlebar_joint_1",
        ArticulationType.REVOLUTE,
        parent=beam,
        child=handlebar_1,
        origin=Origin(xyz=(-HANDLE_X, 0.0, HANDLE_POST_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0, velocity=1.5, lower=-GRIP_TILT, upper=GRIP_TILT
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    beam = object_model.get_part("beam")
    handlebar_0 = object_model.get_part("handlebar_0")
    handlebar_1 = object_model.get_part("handlebar_1")
    beam_pivot = object_model.get_articulation("beam_pivot")
    hb_joint_0 = object_model.get_articulation("handlebar_joint_0")
    hb_joint_1 = object_model.get_articulation("handlebar_joint_1")

    # --- Axle sleeve wraps the arch pivot tube (intentional captured fit) ---
    ctx.allow_overlap(
        beam,
        base,
        elem_a="axle_sleeve",
        elem_b="arch_tube",
        reason="Beam axle sleeve intentionally wraps the arch top tube as its pivot axle.",
    )
    ctx.expect_contact(
        beam,
        base,
        elem_a="axle_sleeve",
        elem_b="arch_tube",
        name="beam sleeve rides on the arch axle",
    )

    # --- Base is a stand about 0.35 m tall with rubber pads on ground -------
    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "base is a support stand about 0.35 m tall",
        base_aabb is not None and 0.30 <= base_aabb[1][2] <= 0.42,
        details=f"base aabb={base_aabb}",
    )

    # Rubber ground pads exist and are near ground level
    for idx in (0, 1):
        pad_aabb = ctx.part_element_world_aabb(base, elem=f"rubber_pad_{idx}")
        ctx.check(
            f"rubber ground pad {idx} exists under support leg",
            pad_aabb is not None and pad_aabb[0][2] < 0.02,
            details=f"pad aabb={pad_aabb}",
        )

    # --- Animal shape: horse body, head, tail all present on beam -----------
    for elem_name in ("horse_body", "horse_head", "horse_tail"):
        elem_aabb = ctx.part_element_world_aabb(beam, elem=elem_name)
        ctx.check(
            f"horse {elem_name} is present on the beam",
            elem_aabb is not None,
            details=f"{elem_name} aabb={elem_aabb}",
        )

    # Horse head is at one end, tail at the other (head X > tail X)
    head_aabb = ctx.part_element_world_aabb(beam, elem="horse_head")
    tail_aabb = ctx.part_element_world_aabb(beam, elem="horse_tail")
    ctx.check(
        "horse head is at the +X end and tail at the -X end",
        head_aabb is not None
        and tail_aabb is not None
        and (head_aabb[0][0] + head_aabb[1][0]) / 2.0
        > (tail_aabb[0][0] + tail_aabb[1][0]) / 2.0 + 0.4,
        details=f"head={head_aabb}, tail={tail_aabb}",
    )

    # --- Two seats exist on the beam ----------------------------------------
    for idx in (0, 1):
        seat_aabb = ctx.part_element_world_aabb(beam, elem=f"seat_plate_{idx}")
        ctx.check(
            f"seat plate {idx} exists on the beam",
            seat_aabb is not None,
            details=f"seat {idx} aabb={seat_aabb}",
        )

    # Seats are at opposite ends of the beam
    seat0 = ctx.part_element_world_aabb(beam, elem="seat_plate_0")
    seat1 = ctx.part_element_world_aabb(beam, elem="seat_plate_1")
    ctx.check(
        "seats are at opposite ends of the beam",
        seat0 is not None
        and seat1 is not None
        and abs(
            (seat0[0][0] + seat0[1][0]) / 2.0 - (seat1[0][0] + seat1[1][0]) / 2.0
        )
        > 0.7,
        details=f"seat0={seat0}, seat1={seat1}",
    )

    # --- Handlebars exist as separate articulated parts ----------------------
    for idx in (0, 1):
        hb = object_model.get_part(f"handlebar_{idx}")
        hb_aabb = ctx.part_world_aabb(hb)
        ctx.check(
            f"handlebar {idx} exists as a separate part",
            hb_aabb is not None and hb_aabb[1][2] > 0.3,
            details=f"handlebar {idx} aabb={hb_aabb}",
        )

    # --- Beam pivot: +/- 18 degrees ----------------------------------------
    lim = beam_pivot.motion_limits
    ctx.check(
        "beam pivot rocks +/- 18 degrees",
        lim is not None
        and abs(lim.lower + TILT) < 1e-6
        and abs(lim.upper - TILT) < 1e-6,
        details=f"limits=({lim.lower if lim else None}, {lim.upper if lim else None})",
    )

    # --- Handlebar joints: +/- 12 degrees ----------------------------------
    for jnt in (hb_joint_0, hb_joint_1):
        jlim = jnt.motion_limits
        ctx.check(
            f"{jnt.name} pivots +/- 12 degrees",
            jlim is not None
            and abs(jlim.lower + GRIP_TILT) < 1e-6
            and abs(jlim.upper - GRIP_TILT) < 1e-6,
            details=f"limits=({jlim.lower if jlim else None}, {jlim.upper if jlim else None})",
        )

    # --- Decisive pose: beam seesaws ----------------------------------------
    rest_seat0 = ctx.part_element_world_aabb(beam, elem="seat_plate_0")
    rest_seat1 = ctx.part_element_world_aabb(beam, elem="seat_plate_1")
    with ctx.pose({beam_pivot: TILT}):
        tilt_seat0 = ctx.part_element_world_aabb(beam, elem="seat_plate_0")
        tilt_seat1 = ctx.part_element_world_aabb(beam, elem="seat_plate_1")
        beam_aabb = ctx.part_world_aabb(beam)
        ctx.check(
            "beam seesaws: one seat drops, the opposite rises",
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
            elem_b="arch_tube",
            name="tilted beam sleeve stays on axle",
        )

    # --- Decisive pose: handlebars pivot independently ----------------------
    rest_hb0 = ctx.part_world_aabb(handlebar_0)
    with ctx.pose({hb_joint_0: GRIP_TILT}):
        tilt_hb0 = ctx.part_world_aabb(handlebar_0)
        ctx.check(
            "handlebar 0 pivots when its joint is posed",
            rest_hb0 is not None
            and tilt_hb0 is not None
            and abs(tilt_hb0[0][2] - rest_hb0[0][2]) > 0.001,
            details=f"rest={rest_hb0}, tilted={tilt_hb0}",
        )

    return ctx.report()


object_model = build_object_model()
