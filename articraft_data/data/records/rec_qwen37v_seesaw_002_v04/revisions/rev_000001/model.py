from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CylinderGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ----------------------------------------------------------------------------
# Animal-shaped toddler playground seesaw (horse theme).
#
# Layout (world frame, Z up, base centered on the origin):
# - Green painted steel A-frame base (~0.38 m tall) with rubber ground pads
#   under each foot.
# - A single horse-shaped rocking beam (~1.4 m long) sits on the central pivot.
#   The beam body is a yellow tube; at one end a horse-head assembly (neck,
#   head, ears, snout) rises upward; at the other end a curved tail arcs back.
# - Two small toddler seat plates sit on the beam between the head and tail.
# - T-shaped handlebars stand upright just inboard of each seat.
# - Rubber bump stops (short cylinders) hang below each beam end for safety.
# - Articulation: single revolute pivot at the beam midpoint, horizontal axis
#   perpendicular to beam length, +/- 15 degrees (toddler-safe range).
# ----------------------------------------------------------------------------

TUBE_R = 0.020       # ~40 mm diameter main tubing
BRACE_R = 0.016
HANDLE_R = 0.014
HEAD_TUBE_R = 0.022  # slightly thicker for the horse head tubes
EAR_R = 0.012
TAIL_R = 0.018

TILT = math.radians(15.0)  # toddler-safe rocking range

ARCH_TOP = 0.38       # pivot height for toddler seesaw
ARCH_HALF_SPAN = 0.24 # ground half-span of the A-frame arch
BASE_FOOT_FLARE = 0.04

BEAM_LEN = 1.40
BEAM_Z = 0.06        # beam tube rides slightly above pivot axis
SLEEVE_R = 0.030
SLEEVE_LEN = 0.12
SEAT_X = 0.50        # seat center distance from pivot
SEAT_Z = 0.03        # seat plate height above pivot axis
SEAT_SIZE = (0.22, 0.24, 0.010)

HANDLE_X = 0.35      # handlebar position, inboard of seat
HANDLE_TOP_Z = 0.28  # crossbar height above pivot

BUMP_X = 0.62        # bump stop position along beam from pivot
BUMP_R = 0.018
BUMP_LEN = 0.08

# Horse head dimensions (from beam end at +X)
NECK_BASE_X = 0.58
HEAD_TIP_X = 0.72
HEAD_TOP_Z = 0.38

# Ground pad dimensions
PAD_SIZE = (0.12, 0.08, 0.012)

GREEN = Material("green_paint", rgba=(0.18, 0.55, 0.22, 1.0))
WARM_YELLOW = Material("warm_yellow_paint", rgba=(0.92, 0.72, 0.15, 1.0))
RUBBER_BLACK = Material("rubber_black", rgba=(0.12, 0.12, 0.12, 1.0))
SADDLE_BROWN = Material("saddle_brown", rgba=(0.45, 0.26, 0.14, 1.0))


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


def _arch_mesh(top_z: float) -> MeshGeometry:
    """Inverted-U arch tube in the YZ plane (beam runs along X)."""
    shoulder = top_z * 0.85
    profile_uz = [
        (-ARCH_HALF_SPAN - BASE_FOOT_FLARE, 0.020),
        (-ARCH_HALF_SPAN - 0.02, 0.030),
        (-ARCH_HALF_SPAN + 0.02, 0.08),
        (-ARCH_HALF_SPAN + 0.06, 0.16),
        (-0.14, shoulder * 0.70),
        (-0.07, top_z),
        (0.0, top_z),
        (0.07, top_z),
        (0.14, shoulder * 0.70),
        (ARCH_HALF_SPAN - 0.06, 0.16),
        (ARCH_HALF_SPAN - 0.02, 0.08),
        (ARCH_HALF_SPAN + 0.02, 0.030),
        (ARCH_HALF_SPAN + BASE_FOOT_FLARE, 0.020),
    ]
    # Arch runs in the YZ plane so the pivot axis is along Y
    points = [(0.0, u, z) for (u, z) in profile_uz]
    return tube_from_spline_points(
        points,
        radius=TUBE_R,
        samples_per_segment=10,
        radial_segments=16,
        cap_ends=True,
    )


def _horse_head_mesh() -> MeshGeometry:
    """Build the horse head assembly in beam-local frame (+X is head end)."""
    mesh = MeshGeometry()
    # Neck: rises from beam end upward and slightly forward
    mesh.merge(_tube_between(
        (NECK_BASE_X, 0.0, BEAM_Z),
        (NECK_BASE_X + 0.06, 0.0, HEAD_TOP_Z * 0.65),
        HEAD_TUBE_R,
    ))
    # Head: angled forward from top of neck
    mesh.merge(_tube_between(
        (NECK_BASE_X + 0.06, 0.0, HEAD_TOP_Z * 0.65),
        (HEAD_TIP_X, 0.0, HEAD_TOP_Z * 0.80),
        HEAD_TUBE_R,
    ))
    # Snout: shorter tube extending forward and slightly down
    mesh.merge(_tube_between(
        (HEAD_TIP_X, 0.0, HEAD_TOP_Z * 0.80),
        (HEAD_TIP_X + 0.06, 0.0, HEAD_TOP_Z * 0.68),
        HEAD_TUBE_R * 0.85,
    ))
    # Left ear
    mesh.merge(_tube_between(
        (NECK_BASE_X + 0.04, -0.018, HEAD_TOP_Z * 0.62),
        (NECK_BASE_X + 0.05, -0.030, HEAD_TOP_Z * 0.78),
        EAR_R,
    ))
    # Right ear
    mesh.merge(_tube_between(
        (NECK_BASE_X + 0.04, 0.018, HEAD_TOP_Z * 0.62),
        (NECK_BASE_X + 0.05, 0.030, HEAD_TOP_Z * 0.78),
        EAR_R,
    ))
    # Mane: small tubes along the back of the neck
    for i in range(4):
        t = 0.15 + i * 0.20
        bx = NECK_BASE_X + 0.06 * t
        bz = BEAM_Z + (HEAD_TOP_Z * 0.65 - BEAM_Z) * t
        mesh.merge(_tube_between(
            (bx, -0.025, bz),
            (bx - 0.02, -0.025, bz + 0.04),
            EAR_R * 0.8,
        ))
        mesh.merge(_tube_between(
            (bx, 0.025, bz),
            (bx - 0.02, 0.025, bz + 0.04),
            EAR_R * 0.8,
        ))
    return mesh


def _horse_tail_mesh() -> MeshGeometry:
    """Build the horse tail assembly in beam-local frame (-X is tail end)."""
    return tube_from_spline_points(
        [
            (-NECK_BASE_X, 0.0, BEAM_Z),
            (-NECK_BASE_X - 0.04, 0.0, BEAM_Z + 0.10),
            (-NECK_BASE_X - 0.07, 0.02, BEAM_Z + 0.22),
            (-NECK_BASE_X - 0.09, 0.04, BEAM_Z + 0.30),
            (-NECK_BASE_X - 0.10, 0.03, BEAM_Z + 0.26),
        ],
        radius=TAIL_R,
        samples_per_segment=10,
        radial_segments=14,
        cap_ends=True,
    )


def _beam_body_mesh() -> MeshGeometry:
    """Main beam tube and structural bracing in beam-local frame."""
    # Main body tube
    truss = (
        CylinderGeometry(TUBE_R, BEAM_LEN, radial_segments=18)
        .rotate_y(math.pi / 2.0)
        .translate(0.0, 0.0, BEAM_Z)
    )
    # Diagonal braces from axle sleeve to main tube (triangulated truss)
    for sx in (1.0, -1.0):
        truss.merge(_tube_between(
            (sx * 0.04, 0.0, 0.005),
            (sx * 0.40, 0.0, BEAM_Z),
            BRACE_R,
        ))
        # Short seat support tubes
        truss.merge(_tube_between(
            (sx * SEAT_X - sx * 0.08, 0.0, BEAM_Z),
            (sx * SEAT_X, 0.0, SEAT_Z + 0.005),
            BRACE_R,
        ))
    return truss


def _axle_sleeve_mesh() -> MeshGeometry:
    """Axle sleeve and weld post connecting sleeve to main tube."""
    sleeve = (
        CylinderGeometry(SLEEVE_R, SLEEVE_LEN, radial_segments=20)
        .rotate_x(math.pi / 2.0)
    )
    weld_post = CylinderGeometry(0.013, BEAM_Z - 0.022, radial_segments=14).translate(
        0.0, 0.0, (BEAM_Z + 0.022) / 2.0
    )
    sleeve.merge(weld_post)
    return sleeve


def _handlebar_mesh(sx: float) -> MeshGeometry:
    """T-shaped handlebar: vertical post + horizontal crossbar."""
    post = CylinderGeometry(HANDLE_R, 0.22, radial_segments=14).translate(
        sx * HANDLE_X, 0.0, BEAM_Z + 0.10
    )
    bar = (
        CylinderGeometry(HANDLE_R, 0.24, radial_segments=14)
        .rotate_x(math.pi / 2.0)
        .translate(sx * HANDLE_X, 0.0, HANDLE_TOP_Z)
    )
    return post.merge(bar)


def _bump_stop_mesh(sx: float) -> MeshGeometry:
    """Rubber bump stop: short thick cylinder hanging below beam end."""
    return CylinderGeometry(BUMP_R, BUMP_LEN, radial_segments=16).translate(
        sx * BUMP_X, 0.0, BEAM_Z - BUMP_LEN / 2.0 - 0.010
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="horse_toddler_seesaw")

    # --- static green A-frame base with rubber ground pads --------------------
    base = model.part("base")
    base.visual(
        mesh_from_geometry(_arch_mesh(ARCH_TOP), "base_arch"),
        material=GREEN,
        name="arch",
    )

    # Short cross-brace for rigidity
    cross = _tube_between(
        (0.0, -ARCH_HALF_SPAN + 0.04, 0.18),
        (0.0, ARCH_HALF_SPAN - 0.04, 0.18),
        BRACE_R,
    )
    base.visual(
        mesh_from_geometry(cross, "cross_brace"),
        material=GREEN,
        name="cross_brace",
    )

    # Rubber ground pads under each foot
    foot_y = ARCH_HALF_SPAN + BASE_FOOT_FLARE
    for idx, sy in enumerate((1.0, -1.0)):
        base.visual(
            Box(PAD_SIZE),
            origin=Origin(xyz=(0.0, sy * foot_y, PAD_SIZE[2] / 2.0)),
            material=RUBBER_BLACK,
            name=f"ground_pad_{idx}",
        )

    # --- horse-shaped rocking beam -------------------------------------------
    beam = model.part("beam")

    # Main beam body tube with braces
    beam.visual(
        mesh_from_geometry(_beam_body_mesh(), "beam_body"),
        material=WARM_YELLOW,
        name="body_tube",
    )

    # Axle sleeve at center
    beam.visual(
        mesh_from_geometry(_axle_sleeve_mesh(), "axle_sleeve"),
        material=WARM_YELLOW,
        name="axle_sleeve",
    )

    # Horse head at +X end
    beam.visual(
        mesh_from_geometry(_horse_head_mesh(), "horse_head"),
        material=WARM_YELLOW,
        name="horse_head",
    )

    # Horse tail at -X end
    beam.visual(
        mesh_from_geometry(_horse_tail_mesh(), "horse_tail"),
        material=WARM_YELLOW,
        name="horse_tail",
    )

    # Seat plates (brown saddle color)
    beam.visual(
        Box(SEAT_SIZE),
        origin=Origin(xyz=(SEAT_X, 0.0, SEAT_Z)),
        material=SADDLE_BROWN,
        name="seat_plate_0",
    )
    beam.visual(
        Box(SEAT_SIZE),
        origin=Origin(xyz=(-SEAT_X, 0.0, SEAT_Z)),
        material=SADDLE_BROWN,
        name="seat_plate_1",
    )

    # Handlebars
    beam.visual(
        mesh_from_geometry(_handlebar_mesh(1.0), "handlebar_0"),
        material=WARM_YELLOW,
        name="handlebar_0",
    )
    beam.visual(
        mesh_from_geometry(_handlebar_mesh(-1.0), "handlebar_1"),
        material=WARM_YELLOW,
        name="handlebar_1",
    )

    # Safety bump stops (rubber black) below each beam end
    beam.visual(
        mesh_from_geometry(_bump_stop_mesh(1.0), "bump_stop_0"),
        material=RUBBER_BLACK,
        name="bump_stop_0",
    )
    beam.visual(
        mesh_from_geometry(_bump_stop_mesh(-1.0), "bump_stop_1"),
        material=RUBBER_BLACK,
        name="bump_stop_1",
    )

    # --- central revolute pivot ----------------------------------------------
    limits = MotionLimits(effort=80.0, velocity=2.0, lower=-TILT, upper=TILT)
    model.articulation(
        "beam_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, ARCH_TOP)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    beam = object_model.get_part("beam")
    pivot = object_model.get_articulation("beam_pivot")

    # --- Pivot and articulation checks ---

    # Captured-axle fit: beam sleeve wraps the arch top tube (the pivot axle).
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

    # Single revolute joint exists with toddler-safe +/- 15 degree range.
    lim = pivot.motion_limits
    ctx.check(
        "beam_pivot is revolute with +/- 15 degree range",
        lim is not None
        and abs(lim.lower + TILT) < 1e-6
        and abs(lim.upper - TILT) < 1e-6,
        details=f"limits=({lim.lower if lim else None}, {lim.upper if lim else None})",
    )

    # Base is a short arch suitable for a toddler seesaw (~0.38 m tall).
    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "base arch is about 0.38 m tall (toddler height)",
        base_aabb is not None and 0.32 <= base_aabb[1][2] <= 0.48,
        details=f"base aabb={base_aabb}",
    )
    ctx.check(
        "base feet rest on the ground plane",
        base_aabb is not None and -0.01 <= base_aabb[0][2] <= 0.015,
        details=f"base aabb={base_aabb}",
    )

    # --- Rubber ground pads ---
    for idx in (0, 1):
        pad_aabb = ctx.part_element_world_aabb(base, elem=f"ground_pad_{idx}")
        ctx.check(
            f"ground_pad_{idx} exists under a base foot",
            pad_aabb is not None and pad_aabb[0][2] >= -0.005 and pad_aabb[1][2] <= 0.025,
            details=f"pad aabb={pad_aabb}",
        )

    # --- Safety bump stops ---
    for idx in (0, 1):
        bump_aabb = ctx.part_element_world_aabb(beam, elem=f"bump_stop_{idx}")
        ctx.check(
            f"bump_stop_{idx} hangs below a beam end",
            bump_aabb is not None and bump_aabb[0][2] < ARCH_TOP - 0.02,
            details=f"bump aabb={bump_aabb}",
        )

    # --- Horse shape features ---
    head_aabb = ctx.part_element_world_aabb(beam, elem="horse_head")
    tail_aabb = ctx.part_element_world_aabb(beam, elem="horse_tail")
    ctx.check(
        "horse head rises above the beam on the +X end",
        head_aabb is not None and head_aabb[1][2] > ARCH_TOP + 0.20,
        details=f"head aabb={head_aabb}",
    )
    ctx.check(
        "horse tail rises above the beam on the -X end",
        tail_aabb is not None and tail_aabb[1][2] > ARCH_TOP + 0.10,
        details=f"tail aabb={tail_aabb}",
    )

    # Head and tail are on opposite sides of the pivot.
    ctx.check(
        "head and tail are on opposite ends of the beam",
        head_aabb is not None
        and tail_aabb is not None
        and (head_aabb[0][0] + head_aabb[1][0]) / 2.0 > 0.15
        and (tail_aabb[0][0] + tail_aabb[1][0]) / 2.0 < -0.15,
        details=f"head center x={(head_aabb[0][0] + head_aabb[1][0]) / 2.0:.3f}, tail center x={(tail_aabb[0][0] + tail_aabb[1][0]) / 2.0:.3f}",
    )

    # --- Seats and handlebars ---
    for end in (0, 1):
        seat = ctx.part_element_world_aabb(beam, elem=f"seat_plate_{end}")
        handle = ctx.part_element_world_aabb(beam, elem=f"handlebar_{end}")
        ctx.check(
            f"beam end {end} carries a seat plate and a handlebar",
            seat is not None and handle is not None,
            details=f"seat={seat}, handle={handle}",
        )
        if seat is not None and handle is not None:
            ctx.check(
                f"handlebar {end} stands upright above its seat",
                handle[1][2] > seat[1][2] + 0.12,
                details=f"handle top={handle[1][2]:.3f}, seat top={seat[1][2]:.3f}",
            )

    # --- Decisive pose check: beam rocks on its pivot ---
    rest_seat0 = ctx.part_element_world_aabb(beam, elem="seat_plate_0")
    rest_seat1 = ctx.part_element_world_aabb(beam, elem="seat_plate_1")
    with ctx.pose({pivot: TILT}):
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
            beam_aabb is not None and beam_aabb[0][2] > 0.01,
            details=f"beam aabb={beam_aabb}",
        )
        ctx.expect_contact(
            beam,
            base,
            elem_a="axle_sleeve",
            elem_b="arch",
            name="tilted beam sleeve stays on its axle",
        )

    return ctx.report()


object_model = build_object_model()
