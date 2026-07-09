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
# Variant 06: Curved-beam four-seat playground seesaw with rubber bumpers
# and ground pads.
#
# Layout (world frame, Z up, base centered on the origin):
# - Sky-blue base: two arched inverted-U tube legs joined by cross members,
#   with dark rubber ground pads under each foot.
# - Two independent yellow rocking beams (~2.6 m), arranged in a shallow X.
#   Each beam is a curved arc (raised ends) instead of a straight tube.
# - Each beam carries at each end: a rust-brown seat plate, a yellow
#   T-handlebar, and a rubber end bumper on a short prismatic joint that
#   allows vertical compression.
# - Articulation: each beam has a revolute pivot (+/- 18 deg), and each
#   bumper has a prismatic joint for vertical compression (0 to 0.035 m).
# ----------------------------------------------------------------------------

TUBE_R = 0.020  # ~40 mm diameter main tubing
BRACE_R = 0.016
SUPPORT_R = 0.018
HANDLE_R = 0.016

YAW = math.radians(10.0)  # half angle of the shallow X between the beams
TILT = math.radians(18.0)  # rocking range of each beam

LOW_ARCH_TOP = 0.56
HIGH_ARCH_TOP = 0.74
ARCH_HALF_SPAN = 0.36
CROSS_BRACE_Z = 0.28
CROSS_BRACE_U = 0.315

BEAM_LEN = 2.60
MAIN_Z = 0.08  # main tube height above pivot at center
BEAM_RISE = 0.18  # how much the beam ends rise above center
SLEEVE_R = 0.032
SLEEVE_LEN = 0.13
SEAT_X = 1.30  # seat plate center along beam from pivot
SEAT_Z_OFFSET = 0.02  # seat above the beam tube at the end
SEAT_SIZE = (0.26, 0.30, 0.012)
HANDLE_X = 0.95  # T-handlebar post, just inboard of the seat
HANDLE_TOP_Z_LOCAL = 0.30  # crossbar height above local beam at that point

# Ground pad dimensions
PAD_SIZE = (0.12, 0.12, 0.018)  # rubber pad under each foot

# Bumper dimensions
BUMPER_R = 0.035  # rubber bumper radius
BUMPER_H = 0.050  # bumper height
BUMPER_INSET_X = 0.10  # bumper placed this far inboard from beam end
BUMPER_COMPRESS = 0.035  # max compression travel
BUMPER_STEM_LEN = 0.06  # short mounting stem hanging below beam tube

SKY_BLUE = Material("sky_blue_paint", rgba=(0.33, 0.62, 0.84, 1.0))
WORN_YELLOW = Material("worn_yellow_paint", rgba=(0.87, 0.74, 0.12, 1.0))
RUST_BROWN = Material("rust_brown_steel", rgba=(0.42, 0.21, 0.13, 1.0))
RUBBER_BLACK = Material("rubber_black", rgba=(0.12, 0.12, 0.12, 1.0))


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


def _arch_mesh(axis_xy: tuple[float, float], top_z: float) -> MeshGeometry:
    """Inverted-U arch tube in the vertical plane spanned by axis_xy."""
    ax, ay = axis_xy
    shoulder = 0.52 if top_z < 0.65 else 0.66
    profile_uz = [
        (-ARCH_HALF_SPAN - 0.055, 0.022),
        (-ARCH_HALF_SPAN - 0.03, 0.028),
        (-0.35, 0.10),
        (-CROSS_BRACE_U, CROSS_BRACE_Z),
        (-0.27, 0.44),
        (-0.18, shoulder),
        (-0.07, top_z),
        (0.0, top_z),
        (0.07, top_z),
        (0.18, shoulder),
        (0.27, 0.44),
        (CROSS_BRACE_U, CROSS_BRACE_Z),
        (0.35, 0.10),
        (ARCH_HALF_SPAN + 0.03, 0.028),
        (ARCH_HALF_SPAN + 0.055, 0.022),
    ]
    points = [(u * ax, u * ay, z) for (u, z) in profile_uz]
    return tube_from_spline_points(
        points,
        radius=TUBE_R,
        samples_per_segment=10,
        radial_segments=16,
        cap_ends=True,
    )


def _beam_z_at(x_local: float) -> float:
    """Approximate Z of the curved beam main tube centerline at local X.

    The beam follows a parabolic arc: center at MAIN_Z, rising by BEAM_RISE
    at the ends (x = +/- SEAT_X).
    """
    t = x_local / SEAT_X  # normalized [-1, 1]
    return MAIN_Z + BEAM_RISE * t * t


def _curved_beam_mesh() -> MeshGeometry:
    """Build the curved main beam tube (arc with raised ends) plus braces.

    In local frame: X along beam, pivot at origin, tube center follows a
    parabolic arc from (−SEAT_X, 0, MAIN_Z + BEAM_RISE) through
    (0, 0, MAIN_Z) to (+SEAT_X, 0, MAIN_Z + BEAM_RISE).
    """
    # Sample the arc for the main tube spline
    n_pts = 11
    pts = []
    for i in range(n_pts):
        t = -1.0 + 2.0 * i / (n_pts - 1)
        x = t * SEAT_X
        z = MAIN_Z + BEAM_RISE * t * t
        pts.append((x, 0.0, z))

    truss = tube_from_spline_points(
        pts,
        radius=TUBE_R,
        samples_per_segment=12,
        radial_segments=18,
        cap_ends=True,
    )

    # Diagonal braces from the axle sleeve area up to the curved main tube
    for sx in (1.0, -1.0):
        # Brace meets the curved tube at about 40% out from center
        brace_end_x = sx * 0.55
        brace_end_z = _beam_z_at(brace_end_x)
        truss.merge(
            _tube_between(
                (sx * 0.04, 0.0, 0.005),
                (brace_end_x, 0.0, brace_end_z),
                BRACE_R,
            )
        )
        # Short bent seat support dropping from the raised beam end to seat level
        end_z = _beam_z_at(sx * SEAT_X)
        truss.merge(
            tube_from_spline_points(
                [
                    (sx * (SEAT_X - 0.06), 0.0, end_z - 0.01),
                    (sx * (SEAT_X + 0.02), 0.0, end_z - 0.04),
                    (sx * (SEAT_X + 0.10), 0.0, end_z - 0.06),
                    (sx * (SEAT_X + 0.16), 0.0, end_z - 0.07),
                ],
                radius=SUPPORT_R,
                samples_per_segment=8,
                radial_segments=14,
                cap_ends=True,
            )
        )
        # Bumper mounting stem: short tube hanging below the beam for the
        # rubber bumper to seat against.
        bumper_x = sx * (SEAT_X - BUMPER_INSET_X)
        bumper_beam_z = _beam_z_at(bumper_x)
        truss.merge(
            _tube_between(
                (bumper_x, 0.0, bumper_beam_z - TUBE_R),
                (bumper_x, 0.0, bumper_beam_z - TUBE_R - BUMPER_STEM_LEN),
                0.012,
            )
        )

    return truss


def _axle_sleeve_mesh() -> MeshGeometry:
    """Axle sleeve hub with weld post to the curved beam center."""
    sleeve = (
        CylinderGeometry(SLEEVE_R, SLEEVE_LEN, radial_segments=20)
        .rotate_x(math.pi / 2.0)
    )
    weld_post = CylinderGeometry(0.014, MAIN_Z - 0.024, radial_segments=14).translate(
        0.0, 0.0, (MAIN_Z + 0.024) / 2.0
    )
    sleeve.merge(weld_post)
    return sleeve


def _handlebar_mesh(sx: float) -> MeshGeometry:
    """T-handlebar at one end of the beam."""
    local_z = _beam_z_at(sx * HANDLE_X)
    # Post extends from beam surface up through the crossbar height so the
    # two tubes physically overlap (no disconnected geometry island).
    post_bottom = local_z - TUBE_R
    post_top = local_z + HANDLE_TOP_Z_LOCAL + 0.02
    post_h = post_top - post_bottom
    post_center_z = (post_bottom + post_top) / 2.0
    post = CylinderGeometry(HANDLE_R, post_h, radial_segments=14).translate(
        sx * HANDLE_X, 0.0, post_center_z
    )
    bar = (
        CylinderGeometry(HANDLE_R, 0.30, radial_segments=14)
        .rotate_x(math.pi / 2.0)
        .translate(sx * HANDLE_X, 0.0, local_z + HANDLE_TOP_Z_LOCAL)
    )
    return post.merge(bar)


def _add_beam_part(model: ArticulatedObject, part_name: str):
    """Add one curved rocking beam part with seats, handlebars."""
    beam = model.part(part_name)

    truss = _curved_beam_mesh()
    beam.visual(
        mesh_from_geometry(truss, f"{part_name}_truss"),
        material=WORN_YELLOW,
        name="truss_tube",
    )

    sleeve = _axle_sleeve_mesh()
    beam.visual(
        mesh_from_geometry(sleeve, f"{part_name}_sleeve"),
        material=WORN_YELLOW,
        name="axle_sleeve",
    )

    for idx, sx in enumerate((1.0, -1.0)):
        hb = _handlebar_mesh(sx)
        beam.visual(
            mesh_from_geometry(hb, f"{part_name}_handlebar_{idx}"),
            material=WORN_YELLOW,
            name=f"handlebar_{idx}",
        )

    # Seat plates at the raised beam ends
    for idx, sx in enumerate((1.0, -1.0)):
        end_z = _beam_z_at(sx * SEAT_X)
        beam.visual(
            Box(SEAT_SIZE),
            origin=Origin(xyz=(sx * (SEAT_X + 0.10), 0.0, end_z + SEAT_Z_OFFSET)),
            material=RUST_BROWN,
            name=f"seat_plate_{idx}",
        )

    return beam


def _add_bumper_part(
    model: ArticulatedObject,
    name: str,
    beam_part,
    beam_name: str,
    sx: float,
    pivot_z: float,
    beam_yaw: float,
) -> None:
    """Add a rubber bumper part on a prismatic joint below a beam end."""
    bumper = model.part(name)
    bumper.visual(
        mesh_from_geometry(
            CylinderGeometry(BUMPER_R, BUMPER_H, radial_segments=16),
            f"{name}_pad",
        ),
        material=RUBBER_BLACK,
        name="bumper_pad",
    )

    # Position bumper at the bottom of the mounting stem, seated against it.
    bumper_x = sx * (SEAT_X - BUMPER_INSET_X)
    bumper_beam_z = _beam_z_at(bumper_x)
    stem_bottom_z = bumper_beam_z - TUBE_R - BUMPER_STEM_LEN
    # Bumper top seats against the stem bottom with small intentional overlap.
    bumper_local_z = stem_bottom_z - BUMPER_H / 2.0 + 0.003

    # Prismatic axis: Z (vertical in articulation frame). Positive q raises
    # the bumper (compression when beam end approaches ground).
    model.articulation(
        f"{name}_joint",
        ArticulationType.PRISMATIC,
        parent=beam_part,
        child=bumper,
        origin=Origin(xyz=(bumper_x, 0.0, bumper_local_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=500.0,
            velocity=0.5,
            lower=0.0,
            upper=BUMPER_COMPRESS,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="curved_beam_seesaw")

    # --- static sky-blue base with rubber ground pads -----------------------
    base = model.part("base")
    low_axis = (-math.sin(YAW), math.cos(YAW))
    high_axis = (math.sin(YAW), math.cos(YAW))
    base.visual(
        mesh_from_geometry(_arch_mesh(low_axis, LOW_ARCH_TOP), "low_arch"),
        material=SKY_BLUE,
        name="low_arch",
    )
    base.visual(
        mesh_from_geometry(_arch_mesh(high_axis, HIGH_ARCH_TOP), "high_arch"),
        material=SKY_BLUE,
        name="high_arch",
    )

    # Cross members
    leg_y = CROSS_BRACE_U * math.cos(YAW)
    leg_x = CROSS_BRACE_U * math.sin(YAW)
    for idx, sy in enumerate((1.0, -1.0)):
        brace = _tube_between(
            (-leg_x - 0.012, sy * leg_y, CROSS_BRACE_Z),
            (leg_x + 0.012, sy * leg_y, CROSS_BRACE_Z),
            SUPPORT_R,
        )
        base.visual(
            mesh_from_geometry(brace, f"cross_brace_{idx}"),
            material=SKY_BLUE,
            name=f"cross_brace_{idx}",
        )

    # Rubber ground pads under each of the four arch feet.
    # The arch feet are at approximately (+/- (ARCH_HALF_SPAN + 0.055), 0, 0.022)
    # in the arch's local plane. Transform into world using the arch axis.
    foot_offset = ARCH_HALF_SPAN + 0.042
    foot_z = 0.012  # just above ground
    for arch_axis, prefix in ((low_axis, "low"), (high_axis, "high")):
        ax, ay = arch_axis
        for fidx, side in enumerate((-1.0, 1.0)):
            fx = side * foot_offset * ax
            fy = side * foot_offset * ay
            base.visual(
                Box(PAD_SIZE),
                origin=Origin(xyz=(fx, fy, foot_z)),
                material=RUBBER_BLACK,
                name=f"{prefix}_pad_{fidx}",
            )

    # --- two independent curved rocking beams --------------------------------
    lower_beam = _add_beam_part(model, "lower_beam")
    upper_beam = _add_beam_part(model, "upper_beam")

    limits = MotionLimits(effort=150.0, velocity=2.5, lower=-TILT, upper=TILT)
    model.articulation(
        "lower_beam_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_beam,
        origin=Origin(xyz=(0.0, 0.0, LOW_ARCH_TOP), rpy=(0.0, 0.0, YAW)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=limits,
    )
    model.articulation(
        "upper_beam_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_beam,
        origin=Origin(xyz=(0.0, 0.0, HIGH_ARCH_TOP), rpy=(0.0, 0.0, -YAW)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=limits,
    )

    # --- rubber bumpers at each beam end (prismatic compression joints) ------
    _add_bumper_part(model, "lower_bumper_0", lower_beam, "lower_beam", 1.0, LOW_ARCH_TOP, YAW)
    _add_bumper_part(model, "lower_bumper_1", lower_beam, "lower_beam", -1.0, LOW_ARCH_TOP, YAW)
    _add_bumper_part(model, "upper_bumper_0", upper_beam, "upper_beam", 1.0, HIGH_ARCH_TOP, -YAW)
    _add_bumper_part(model, "upper_bumper_1", upper_beam, "upper_beam", -1.0, HIGH_ARCH_TOP, -YAW)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lower_beam = object_model.get_part("lower_beam")
    upper_beam = object_model.get_part("upper_beam")
    lower_pivot = object_model.get_articulation("lower_beam_pivot")
    upper_pivot = object_model.get_articulation("upper_beam_pivot")

    # --- Captured-axle fits (same as parent) --------------------------------
    ctx.allow_overlap(
        lower_beam,
        base,
        elem_a="axle_sleeve",
        elem_b="low_arch",
        reason="Lower beam axle sleeve intentionally wraps the low arch top tube, its pivot axle.",
    )
    ctx.allow_overlap(
        upper_beam,
        base,
        elem_a="axle_sleeve",
        elem_b="high_arch",
        reason="Upper beam axle sleeve intentionally wraps the high arch top tube, its pivot axle.",
    )
    ctx.expect_contact(
        lower_beam,
        base,
        elem_a="axle_sleeve",
        elem_b="low_arch",
        name="lower beam sleeve rides on the low arch axle",
    )
    ctx.expect_contact(
        upper_beam,
        base,
        elem_a="axle_sleeve",
        elem_b="high_arch",
        name="upper beam sleeve rides on the high arch axle",
    )

    # --- Rubber ground pads exist under the base feet -----------------------
    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "base is an arched stand about 0.7 m tall",
        base_aabb is not None and 0.70 <= base_aabb[1][2] <= 0.82,
        details=f"base aabb={base_aabb}",
    )
    for pad_name in ("low_pad_0", "low_pad_1", "high_pad_0", "high_pad_1"):
        pad_aabb = ctx.part_element_world_aabb(base, elem=pad_name)
        ctx.check(
            f"rubber ground pad {pad_name} sits near ground level",
            pad_aabb is not None and pad_aabb[0][2] < 0.03 and pad_aabb[1][2] < 0.05,
            details=f"{pad_name} aabb={pad_aabb}",
        )

    # --- Curved beam: seats at raised ends are higher than the pivot center --
    for beam, pivot_z in ((lower_beam, LOW_ARCH_TOP), (upper_beam, HIGH_ARCH_TOP)):
        sleeve_aabb = ctx.part_element_world_aabb(beam, elem="axle_sleeve")
        for end in (0, 1):
            seat_aabb = ctx.part_element_world_aabb(beam, elem=f"seat_plate_{end}")
            ctx.check(
                f"{beam.name} seat {end} is raised above the beam center (curved beam)",
                sleeve_aabb is not None
                and seat_aabb is not None
                and (seat_aabb[0][2] + seat_aabb[1][2]) / 2.0
                > (sleeve_aabb[0][2] + sleeve_aabb[1][2]) / 2.0 + 0.08,
                details=f"sleeve={sleeve_aabb}, seat={seat_aabb}",
            )

    # --- Bumper prismatic joints exist and have correct compression limits ----
    for bumper_name, parent_beam in (
        ("lower_bumper_0", lower_beam),
        ("lower_bumper_1", lower_beam),
        ("upper_bumper_0", upper_beam),
        ("upper_bumper_1", upper_beam),
    ):
        bumper = object_model.get_part(bumper_name)
        joint = object_model.get_articulation(f"{bumper_name}_joint")
        lim = joint.motion_limits
        ctx.check(
            f"{bumper_name} has a prismatic compression joint",
            joint.articulation_type == ArticulationType.PRISMATIC,
            details=f"type={joint.articulation_type}",
        )
        ctx.check(
            f"{bumper_name} compression range is 0 to ~{BUMPER_COMPRESS} m",
            lim is not None
            and abs(lim.lower) < 1e-6
            and abs(lim.upper - BUMPER_COMPRESS) < 1e-4,
            details=f"limits=({lim.lower if lim else None}, {lim.upper if lim else None})",
        )
        # Bumper seats against the beam mounting stem (small intentional overlap).
        ctx.allow_overlap(
            parent_beam,
            bumper,
            reason=f"{bumper_name} rubber pad is seated against the beam mounting stem.",
        )
        ctx.expect_contact(
            bumper,
            parent_beam,
            name=f"{bumper_name} contacts its parent beam at rest",
        )
        # Bumper should be visually below its beam end
        bumper_aabb = ctx.part_world_aabb(bumper)
        ctx.check(
            f"{bumper_name} exists as a visible rubber pad below the beam end",
            bumper_aabb is not None and bumper_aabb[0][2] > 0.0,
            details=f"bumper aabb={bumper_aabb}",
        )

    # --- Rocking range is +/- 18 degrees on both pivots ----------------------
    for pivot in (lower_pivot, upper_pivot):
        lim = pivot.motion_limits
        ctx.check(
            f"{pivot.name} rocks +/- 18 degrees",
            lim is not None
            and abs(lim.lower + TILT) < 1e-6
            and abs(lim.upper - TILT) < 1e-6,
            details=f"limits=({lim.lower if lim else None}, {lim.upper if lim else None})",
        )

    # --- Decisive pose: lower beam seesaws, bumper can compress --------------
    rest_lo0 = ctx.part_element_world_aabb(lower_beam, elem="seat_plate_0")
    rest_lo1 = ctx.part_element_world_aabb(lower_beam, elem="seat_plate_1")
    rest_up0 = ctx.part_element_world_aabb(upper_beam, elem="seat_plate_0")
    with ctx.pose({lower_pivot: TILT}):
        tilt_lo0 = ctx.part_element_world_aabb(lower_beam, elem="seat_plate_0")
        tilt_lo1 = ctx.part_element_world_aabb(lower_beam, elem="seat_plate_1")
        tilt_up0 = ctx.part_element_world_aabb(upper_beam, elem="seat_plate_0")
        beam_aabb = ctx.part_world_aabb(lower_beam)
        ctx.check(
            "lower beam seesaws: one seat drops, the opposite seat rises",
            rest_lo0 is not None
            and tilt_lo0 is not None
            and rest_lo1 is not None
            and tilt_lo1 is not None
            and tilt_lo0[0][2] < rest_lo0[0][2] - 0.30
            and tilt_lo1[0][2] > rest_lo1[0][2] + 0.30,
            details=f"seat0 {rest_lo0} -> {tilt_lo0}, seat1 {rest_lo1} -> {tilt_lo1}",
        )
        ctx.check(
            "fully tilted lower beam stays clear of the ground",
            beam_aabb is not None and beam_aabb[0][2] > 0.01,
            details=f"lower beam aabb={beam_aabb}",
        )
        ctx.check(
            "beams rock independently: upper beam holds still while lower rocks",
            rest_up0 is not None
            and tilt_up0 is not None
            and abs(tilt_up0[0][2] - rest_up0[0][2]) < 1e-6,
            details=f"upper seat0 {rest_up0} -> {tilt_up0}",
        )
        ctx.expect_contact(
            lower_beam,
            base,
            elem_a="axle_sleeve",
            elem_b="low_arch",
            name="tilted lower beam sleeve stays on its axle",
        )

    with ctx.pose({upper_pivot: -TILT}):
        tilt_up0 = ctx.part_element_world_aabb(upper_beam, elem="seat_plate_0")
        beam_aabb = ctx.part_world_aabb(upper_beam)
        ctx.check(
            "upper beam seesaws the opposite way: its near seat rises",
            rest_up0 is not None
            and tilt_up0 is not None
            and tilt_up0[0][2] > rest_up0[0][2] + 0.30,
            details=f"upper seat0 {rest_up0} -> {tilt_up0}",
        )
        ctx.check(
            "fully tilted upper beam stays clear of the ground",
            beam_aabb is not None and beam_aabb[0][2] > 0.01,
            details=f"upper beam aabb={beam_aabb}",
        )
        ctx.expect_contact(
            upper_beam,
            base,
            elem_a="axle_sleeve",
            elem_b="high_arch",
            name="tilted upper beam sleeve stays on its axle",
        )

    # --- Bumper compression pose: show the prismatic joint can move ----------
    for bumper_name in ("lower_bumper_0", "lower_bumper_1"):
        bumper_joint = object_model.get_articulation(f"{bumper_name}_joint")
        bumper_part = object_model.get_part(bumper_name)
        rest_pos = ctx.part_world_aabb(bumper_part)
        with ctx.pose({bumper_joint: BUMPER_COMPRESS}):
            compressed_pos = ctx.part_world_aabb(bumper_part)
            ctx.check(
                f"{bumper_name} compresses upward when prismatic joint actuated",
                rest_pos is not None
                and compressed_pos is not None
                and compressed_pos[0][2] > rest_pos[0][2] + 0.02,
                details=f"rest={rest_pos}, compressed={compressed_pos}",
            )

    # --- Shallow X: beams cross above base ----------------------------------
    ctx.expect_overlap(
        lower_beam,
        upper_beam,
        axes="xy",
        min_overlap=0.5,
        name="beams cross above the base in plan view",
    )

    return ctx.report()


object_model = build_object_model()
