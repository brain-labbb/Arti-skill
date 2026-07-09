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
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ----------------------------------------------------------------------------
# Variant 20: four-seat playground seesaw with asymmetric seat heights,
# rubber end bumpers on prismatic joints, rubber ground pads under the
# support legs, and textured footrests near each seat.
#
# Layout (world frame, Z up, base centered on the origin):
# - Sky-blue base: two arched inverted-U tube legs joined by cross members,
#   with rubber ground pads under each foot.
# - Two independent yellow rocking beams (~2.6 m), arranged in a shallow X.
# - Each beam has asymmetric seat heights: end 0 seat is mounted higher,
#   end 1 seat lower, but the beam remains balanced.
# - Rubber bumpers at each beam end on short prismatic joints (vertical
#   compression).
# - Textured footrest platforms with grip ribs near each seat.
# - Each beam pivots on its own revolute joint, +/- 18 degrees.
# ----------------------------------------------------------------------------

TUBE_R = 0.020  # ~40 mm diameter main tubing
BRACE_R = 0.016
SUPPORT_R = 0.018
HANDLE_R = 0.016

YAW = math.radians(10.0)  # half angle of the shallow X
TILT = math.radians(18.0)  # rocking range

LOW_ARCH_TOP = 0.56
HIGH_ARCH_TOP = 0.74
ARCH_HALF_SPAN = 0.36
CROSS_BRACE_Z = 0.28
CROSS_BRACE_U = 0.315

BEAM_LEN = 2.60
MAIN_Z = 0.08
SLEEVE_R = 0.032
SLEEVE_LEN = 0.13
SEAT_X = 1.43

# Variant 20: asymmetric seat heights
SEAT_Z_HIGH = 0.068  # end 0 seat (higher)
SEAT_Z_LOW = 0.015   # end 1 seat (lower)
SEAT_SIZE = (0.26, 0.30, 0.012)
HANDLE_X = 1.04
HANDLE_TOP_Z = 0.34

# Variant 20: rubber end bumpers
BUMPER_SIZE = (0.055, 0.055, 0.020)
BUMPER_X = 1.50
BUMPER_Z_MOUNT = 0.0  # articulation origin z in beam frame
BUMPER_VISUAL_DROP = 0.016  # bumper visual center below part origin
BUMPER_COMPRESSION = 0.012  # prismatic travel

# Variant 20: ground pads
GROUND_PAD_SIZE = (0.10, 0.10, 0.008)

# Variant 20: textured footrests
FOOTREST_PLATFORM = (0.14, 0.10, 0.008)
FOOTREST_RIB = (0.12, 0.008, 0.006)
FOOTREST_X = 1.12
FOOTREST_Z = 0.002

SKY_BLUE = Material("sky_blue_paint", rgba=(0.33, 0.62, 0.84, 1.0))
WORN_YELLOW = Material("worn_yellow_paint", rgba=(0.87, 0.74, 0.12, 1.0))
RUST_BROWN = Material("rust_brown_steel", rgba=(0.42, 0.21, 0.13, 1.0))
RUBBER_BLACK = Material("rubber_black", rgba=(0.10, 0.10, 0.10, 1.0))
GRIP_RUBBER = Material("grip_rubber_dark", rgba=(0.15, 0.15, 0.13, 1.0))


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


def _footrest_mesh() -> MeshGeometry:
    """Textured footrest platform with raised grip ribs."""
    platform = BoxGeometry(FOOTREST_PLATFORM)
    rib_count = 5
    rib_spacing = FOOTREST_PLATFORM[1] / (rib_count + 1)
    for i in range(rib_count):
        rib_y = -FOOTREST_PLATFORM[1] / 2.0 + rib_spacing * (i + 1)
        rib = BoxGeometry(FOOTREST_RIB)
        rib.translate(0.0, rib_y, FOOTREST_PLATFORM[2] / 2.0 + FOOTREST_RIB[2] / 2.0)
        platform.merge(rib)
    return platform


def _beam_meshes() -> tuple:
    """Build one rocking beam in its local frame with asymmetric seat supports.

    Returns (truss, sleeve, handlebar_0, handlebar_1, footrest_0, footrest_1).
    """
    # Main top tube, full length
    truss = (
        CylinderGeometry(TUBE_R, BEAM_LEN, radial_segments=18)
        .rotate_y(math.pi / 2.0)
        .translate(0.0, 0.0, MAIN_Z)
    )

    # Asymmetric seat heights per end
    seat_z_map = {1.0: SEAT_Z_HIGH, -1.0: SEAT_Z_LOW}

    for sx in (1.0, -1.0):
        sz = seat_z_map[sx]
        # Diagonal brace from axle sleeve to main tube
        truss.merge(
            _tube_between(
                (sx * 0.04, 0.0, 0.005),
                (sx * 0.60, 0.0, MAIN_Z),
                BRACE_R,
            )
        )
        # Seat support: curved tube from main tube down to seat height
        mid_z1 = max(0.025, sz + 0.018)
        mid_z2 = max(0.015, sz + 0.006)
        end_z = max(0.008, sz - 0.006)
        truss.merge(
            tube_from_spline_points(
                [
                    (sx * 1.24, 0.0, MAIN_Z),
                    (sx * 1.34, 0.0, mid_z1),
                    (sx * 1.42, 0.0, mid_z2),
                    (sx * 1.49, 0.0, end_z),
                ],
                radius=SUPPORT_R,
                samples_per_segment=10,
                radial_segments=14,
                cap_ends=True,
            )
        )
        # Bumper mount stub: tube from seat support end past the articulation
        # origin, extending into the bumper block so the slide rail reads as
        # captured inside the rubber bumper.
        stub_end_z = BUMPER_Z_MOUNT - 0.008
        truss.merge(
            _tube_between(
                (sx * 1.49, 0.0, end_z),
                (sx * BUMPER_X, 0.0, stub_end_z),
                SUPPORT_R * 0.7,
            )
        )
        # Footrest support: short vertical tube from main tube to footrest
        truss.merge(
            _tube_between(
                (sx * FOOTREST_X, 0.0, MAIN_Z - TUBE_R),
                (sx * FOOTREST_X, 0.0, FOOTREST_Z + 0.002),
                SUPPORT_R * 0.7,
            )
        )

    # Axle sleeve + weld post
    sleeve = (
        CylinderGeometry(SLEEVE_R, SLEEVE_LEN, radial_segments=20)
        .rotate_x(math.pi / 2.0)
    )
    weld_post = CylinderGeometry(0.014, MAIN_Z - 0.024, radial_segments=14).translate(
        0.0, 0.0, (MAIN_Z + 0.024) / 2.0
    )
    sleeve.merge(weld_post)

    # Handlebars
    handlebars: list[MeshGeometry] = []
    for sx in (1.0, -1.0):
        post = CylinderGeometry(HANDLE_R, 0.28, radial_segments=14).translate(
            sx * HANDLE_X, 0.0, MAIN_Z + 0.13
        )
        bar = (
            CylinderGeometry(HANDLE_R, 0.30, radial_segments=14)
            .rotate_x(math.pi / 2.0)
            .translate(sx * HANDLE_X, 0.0, HANDLE_TOP_Z)
        )
        handlebars.append(post.merge(bar))

    # Footrests (textured platforms with ribs)
    footrests: list[MeshGeometry] = []
    for sx in (1.0, -1.0):
        fr = _footrest_mesh()
        fr.translate(sx * FOOTREST_X, 0.0, FOOTREST_Z)
        footrests.append(fr)

    return truss, sleeve, handlebars[0], handlebars[1], footrests[0], footrests[1]


def _add_beam_part(model: ArticulatedObject, part_name: str):
    truss, sleeve, hb0, hb1, fr0, fr1 = _beam_meshes()
    beam = model.part(part_name)
    beam.visual(
        mesh_from_geometry(truss, f"{part_name}_truss"),
        material=WORN_YELLOW,
        name="truss_tube",
    )
    beam.visual(
        mesh_from_geometry(sleeve, f"{part_name}_sleeve"),
        material=WORN_YELLOW,
        name="axle_sleeve",
    )
    beam.visual(
        mesh_from_geometry(hb0, f"{part_name}_handlebar_0"),
        material=WORN_YELLOW,
        name="handlebar_0",
    )
    beam.visual(
        mesh_from_geometry(hb1, f"{part_name}_handlebar_1"),
        material=WORN_YELLOW,
        name="handlebar_1",
    )
    beam.visual(
        mesh_from_geometry(fr0, f"{part_name}_footrest_0"),
        material=GRIP_RUBBER,
        name="footrest_0",
    )
    beam.visual(
        mesh_from_geometry(fr1, f"{part_name}_footrest_1"),
        material=GRIP_RUBBER,
        name="footrest_1",
    )
    # Asymmetric seat plates
    beam.visual(
        Box(SEAT_SIZE),
        origin=Origin(xyz=(SEAT_X, 0.0, SEAT_Z_HIGH)),
        material=RUST_BROWN,
        name="seat_plate_0",
    )
    beam.visual(
        Box(SEAT_SIZE),
        origin=Origin(xyz=(-SEAT_X, 0.0, SEAT_Z_LOW)),
        material=RUST_BROWN,
        name="seat_plate_1",
    )
    return beam


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="four_seat_tube_seesaw_v20")

    # --- static sky-blue base -------------------------------------------------
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
    # Cross braces
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

    # Rubber ground pads under each arch foot
    pad_idx = 0
    for arch_axis in (low_axis, high_axis):
        for su in (-1.0, 1.0):
            u = su * (ARCH_HALF_SPAN + 0.055)
            fx = u * arch_axis[0]
            fy = u * arch_axis[1]
            base.visual(
                Box(GROUND_PAD_SIZE),
                origin=Origin(xyz=(fx, fy, GROUND_PAD_SIZE[2] / 2.0)),
                material=RUBBER_BLACK,
                name=f"ground_pad_{pad_idx}",
            )
            pad_idx += 1

    # --- two independent yellow rocking beams ---------------------------------
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

    # --- rubber end bumpers on prismatic joints -------------------------------
    bumper_limits = MotionLimits(
        effort=200.0, velocity=0.5, lower=0.0, upper=BUMPER_COMPRESSION,
    )
    for beam_name, beam_part in (
        ("lower_beam", lower_beam),
        ("upper_beam", upper_beam),
    ):
        prefix = beam_name.split("_")[0]
        for end_idx, sx in enumerate((1.0, -1.0)):
            bumper_name = f"{prefix}_bumper_{end_idx}"
            bumper = model.part(bumper_name)
            bumper.visual(
                Box(BUMPER_SIZE),
                origin=Origin(xyz=(0.0, 0.0, -BUMPER_VISUAL_DROP)),
                material=RUBBER_BLACK,
                name="bumper_block",
            )
            # Prismatic joint: axis (0,0,1) so positive q = compression (up)
            model.articulation(
                f"{bumper_name}_slide",
                ArticulationType.PRISMATIC,
                parent=beam_part,
                child=bumper,
                origin=Origin(xyz=(sx * BUMPER_X, 0.0, BUMPER_Z_MOUNT)),
                axis=(0.0, 0.0, 1.0),
                motion_limits=bumper_limits,
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lower_beam = object_model.get_part("lower_beam")
    upper_beam = object_model.get_part("upper_beam")
    lower_pivot = object_model.get_articulation("lower_beam_pivot")
    upper_pivot = object_model.get_articulation("upper_beam_pivot")

    # --- Axle sleeve / arch contact (carried from parent) --------------------
    ctx.allow_overlap(
        lower_beam, base,
        elem_a="axle_sleeve", elem_b="low_arch",
        reason="Lower beam axle sleeve wraps the low arch top tube as its pivot axle.",
    )
    ctx.allow_overlap(
        upper_beam, base,
        elem_a="axle_sleeve", elem_b="high_arch",
        reason="Upper beam axle sleeve wraps the high arch top tube as its pivot axle.",
    )
    ctx.expect_contact(
        lower_beam, base,
        elem_a="axle_sleeve", elem_b="low_arch",
        name="lower beam sleeve rides on the low arch axle",
    )
    ctx.expect_contact(
        upper_beam, base,
        elem_a="axle_sleeve", elem_b="high_arch",
        name="upper beam sleeve rides on the high arch axle",
    )

    # --- Base reads as ~0.7 m tall arched stand on the ground ----------------
    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "base is an arched stand about 0.7 m tall",
        base_aabb is not None and 0.70 <= base_aabb[1][2] <= 0.82,
        details=f"base aabb={base_aabb}",
    )
    ctx.check(
        "base feet rest on the ground plane",
        base_aabb is not None and -0.01 <= base_aabb[0][2] <= 0.015,
        details=f"base aabb={base_aabb}",
    )

    # --- Variant 20: asymmetric seat heights ---------------------------------
    seat_height_diff = SEAT_Z_HIGH - SEAT_Z_LOW
    for beam_name in ("lower_beam", "upper_beam"):
        beam = object_model.get_part(beam_name)
        seat0_aabb = ctx.part_element_world_aabb(beam, elem="seat_plate_0")
        seat1_aabb = ctx.part_element_world_aabb(beam, elem="seat_plate_1")
        if seat0_aabb is not None and seat1_aabb is not None:
            s0z = (seat0_aabb[0][2] + seat0_aabb[1][2]) / 2.0
            s1z = (seat1_aabb[0][2] + seat1_aabb[1][2]) / 2.0
            ctx.check(
                f"{beam_name} has asymmetric seat heights: end 0 higher than end 1",
                s0z > s1z + 0.030,
                details=f"seat0 center z={s0z:.4f}, seat1 center z={s1z:.4f}, diff={s0z - s1z:.4f}",
            )
        else:
            ctx.fail(
                f"{beam_name} seat plates exist",
                f"seat0={seat0_aabb}, seat1={seat1_aabb}",
            )

    # --- Variant 20: rubber ground pads under arch feet ----------------------
    for pad_idx in range(4):
        pad_name = f"ground_pad_{pad_idx}"
        pad_aabb = ctx.part_element_world_aabb(base, elem=pad_name)
        ctx.check(
            f"rubber {pad_name} exists under an arch foot",
            pad_aabb is not None
            and pad_aabb[0][2] < 0.010
            and pad_aabb[1][2] < 0.020,
            details=f"{pad_name} aabb={pad_aabb}",
        )

    # --- Variant 20: textured footrests near each seat -----------------------
    for beam_name in ("lower_beam", "upper_beam"):
        beam = object_model.get_part(beam_name)
        for end in (0, 1):
            fr_aabb = ctx.part_element_world_aabb(beam, elem=f"footrest_{end}")
            ctx.check(
                f"{beam_name} has textured footrest_{end} near seat",
                fr_aabb is not None,
                details=f"footrest_{end} aabb={fr_aabb}",
            )

    # --- Variant 20: rubber end bumpers on prismatic joints ------------------
    for beam_prefix, beam_name in (("lower", "lower_beam"), ("upper", "upper_beam")):
        beam = object_model.get_part(beam_name)
        for end in (0, 1):
            bumper_name = f"{beam_prefix}_bumper_{end}"
            bumper = object_model.get_part(bumper_name)
            slide = object_model.get_articulation(f"{bumper_name}_slide")

            # Bumper block intentionally wraps the mount stub (slide rail
            # captured inside the rubber bumper).
            ctx.allow_overlap(
                bumper, beam,
                elem_a="bumper_block", elem_b="truss_tube",
                reason=f"{bumper_name} rubber bumper captures the mount stub slide rail.",
            )

            # Bumper part exists with rubber block
            b_aabb = ctx.part_world_aabb(bumper)
            ctx.check(
                f"{bumper_name} rubber block exists at beam end",
                b_aabb is not None,
                details=f"{bumper_name} aabb={b_aabb}",
            )
            # Bumper contacts its parent beam (slide rail seated)
            ctx.expect_contact(
                bumper, beam,
                elem_a="bumper_block", elem_b="truss_tube",
                name=f"{bumper_name} bumper seated on beam mount stub",
            )
            # Prismatic joint has correct type and limits
            ctx.check(
                f"{bumper_name}_slide is prismatic with compression range",
                slide.articulation_type == ArticulationType.PRISMATIC
                and slide.motion_limits is not None
                and abs(slide.motion_limits.lower) < 1e-6
                and abs(slide.motion_limits.upper - BUMPER_COMPRESSION) < 1e-6,
                details=f"type={slide.articulation_type}, limits={slide.motion_limits}",
            )

    # --- Verify at least one non-fixed joint (revolute + prismatic) ----------
    non_fixed = [
        a for a in object_model.articulations
        if a.articulation_type != ArticulationType.FIXED
    ]
    ctx.check(
        "model has at least one non-fixed articulation",
        len(non_fixed) >= 1,
        details=f"non-fixed count={len(non_fixed)}",
    )

    # --- Seats and handlebars exist near beam ends ---------------------------
    for beam, lo_z, hi_z in (
        (lower_beam, 0.48, 0.70),
        (upper_beam, 0.66, 0.88),
    ):
        for end in (0, 1):
            seat = ctx.part_element_world_aabb(beam, elem=f"seat_plate_{end}")
            handle = ctx.part_element_world_aabb(beam, elem=f"handlebar_{end}")
            ok = seat is not None and handle is not None
            ctx.check(
                f"{beam.name} end {end} carries a seat plate and a handlebar",
                ok,
                details=f"seat={seat}, handle={handle}",
            )
            if not ok:
                continue
            scx = (seat[0][0] + seat[1][0]) / 2.0
            scy = (seat[0][1] + seat[1][1]) / 2.0
            scz = (seat[0][2] + seat[1][2]) / 2.0
            ctx.check(
                f"{beam.name} seat {end} sits near the beam end at sit height",
                math.hypot(scx, scy) > 1.25 and lo_z <= scz <= hi_z,
                details=f"seat center=({scx:.3f},{scy:.3f},{scz:.3f})",
            )
            hcx = (handle[0][0] + handle[1][0]) / 2.0
            hcy = (handle[0][1] + handle[1][1]) / 2.0
            inboard = math.hypot(scx - hcx, scy - hcy)
            ctx.check(
                f"{beam.name} handlebar {end} stands upright just inboard of its seat",
                handle[1][2] > seat[1][2] + 0.15 and 0.20 <= inboard <= 0.55,
                details=f"handle top={handle[1][2]:.3f}, seat top={seat[1][2]:.3f}, inboard={inboard:.3f}",
            )

    # --- Shallow X: beams cross above the base footprint ---------------------
    ctx.expect_overlap(
        lower_beam, upper_beam,
        axes="xy", min_overlap=0.5,
        name="beams cross above the base in plan view",
    )

    # --- Upper beam pivots above the lower beam ------------------------------
    lo_sleeve = ctx.part_element_world_aabb(lower_beam, elem="axle_sleeve")
    up_sleeve = ctx.part_element_world_aabb(upper_beam, elem="axle_sleeve")
    ctx.check(
        "upper beam pivots above the lower beam",
        lo_sleeve is not None
        and up_sleeve is not None
        and (up_sleeve[0][2] + up_sleeve[1][2]) / 2.0
        > (lo_sleeve[0][2] + lo_sleeve[1][2]) / 2.0 + 0.10,
        details=f"lower sleeve={lo_sleeve}, upper sleeve={up_sleeve}",
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

    # --- Decisive pose checks: independent seesaw rocking --------------------
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
            lower_beam, base,
            elem_a="axle_sleeve", elem_b="low_arch",
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
            upper_beam, base,
            elem_a="axle_sleeve", elem_b="high_arch",
            name="tilted upper beam sleeve stays on its axle",
        )

    return ctx.report()


object_model = build_object_model()
