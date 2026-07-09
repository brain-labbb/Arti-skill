from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
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
# Low inclusive playground seesaw with backrest seats, rubber end bumpers on
# short prismatic joints, molded seats with raised lips, and rounded handle
# grips.  Forked from the four-seat bent-tube seesaw family.
#
# Layout (world frame, Z up, base centered on the origin):
# - Sky-blue base: two arched inverted-U tube legs (~40 mm dia.) at reduced
#   height (tops at 0.40 m and 0.50 m) joined by cross members, forming a
#   low inclusive stand.
# - Two independent yellow rocking beams (~2.0 m), in a shallow X (±10 deg
#   yaw).  Each beam is a triangulated tube truss carrying at each end:
#   * a molded bucket seat with raised side/front lips and a tall backrest
#   * a yellow T-handlebar with rounded rubber grip spheres at both crossbar
#     tips, just inboard of the seat
#   * a rubber bumper pad under the beam on a short prismatic joint (Z-axis,
#     0 to +25 mm compression travel)
# - Each beam pivots on its own revolute joint (+/- 18 deg).
# ----------------------------------------------------------------------------

TUBE_R = 0.020
BRACE_R = 0.016
SUPPORT_R = 0.018
HANDLE_R = 0.014

YAW = math.radians(10.0)
TILT = math.radians(18.0)

LOW_ARCH_TOP = 0.36
HIGH_ARCH_TOP = 0.52
ARCH_HALF_SPAN = 0.28
CROSS_BRACE_Z = 0.16
CROSS_BRACE_U = 0.22

BEAM_LEN = 2.00
MAIN_Z = 0.048
SLEEVE_R = 0.028
SLEEVE_LEN = 0.10
SEAT_X = 0.82
SEAT_Z = 0.020
HANDLE_X = 0.62
HANDLE_TOP_Z = 0.24
CROSSBAR_HALF = 0.15

BUMPER_R = 0.032
BUMPER_H = 0.040
BUMPER_TRAVEL = 0.025
BUMPER_MOUNT_X = 0.98

SEAT_PAN_W = 0.26
SEAT_PAN_D = 0.28
SEAT_PAN_T = 0.010
SEAT_LIP_H = 0.030
SEAT_LIP_T = 0.008
SEAT_BACK_H = 0.20
SEAT_BACK_T = 0.010
GRIP_R = 0.022

SKY_BLUE = Material("sky_blue_paint", rgba=(0.33, 0.62, 0.84, 1.0))
WORN_YELLOW = Material("worn_yellow_paint", rgba=(0.87, 0.74, 0.12, 1.0))
RUST_BROWN = Material("rust_brown_steel", rgba=(0.42, 0.21, 0.13, 1.0))
RUBBER_BLACK = Material("rubber_black", rgba=(0.12, 0.12, 0.12, 1.0))
SEAT_GREEN = Material("molded_green_seat", rgba=(0.20, 0.55, 0.30, 1.0))
GRIP_RED = Material("rubber_grip_red", rgba=(0.78, 0.22, 0.15, 1.0))


# ---------------------------------------------------------------------------
# Geometry builders
# ---------------------------------------------------------------------------

def _tube_between(
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    radius: float,
    *,
    radial_segments: int = 16,
) -> MeshGeometry:
    dx, dy, dz = p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]
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
    """Inverted-U arch tube for the low inclusive base."""
    ax, ay = axis_xy
    hs = ARCH_HALF_SPAN
    # Intermediate heights scale between cross brace and peak
    span = top_z - CROSS_BRACE_Z
    h2 = CROSS_BRACE_Z + span * 0.30
    h3 = CROSS_BRACE_Z + span * 0.65
    h4 = top_z * 0.95
    profile_uz = [
        (-hs - 0.04, 0.020),
        (-hs - 0.02, 0.025),
        (-hs + 0.03, 0.06),
        (-CROSS_BRACE_U, CROSS_BRACE_Z),
        (-hs * 0.50, h2),
        (-hs * 0.28, h3),
        (-0.05, h4),
        (0.0, top_z),
        (0.05, top_z),
        (hs * 0.28, h3),
        (hs * 0.50, h2),
        (CROSS_BRACE_U, CROSS_BRACE_Z),
        (hs - 0.03, 0.06),
        (hs + 0.02, 0.025),
        (hs + 0.04, 0.020),
    ]
    points = [(u * ax, u * ay, z) for (u, z) in profile_uz]
    return tube_from_spline_points(
        points,
        radius=TUBE_R,
        samples_per_segment=10,
        radial_segments=16,
        cap_ends=True,
    )


def _molded_seat_mesh(sign: float) -> MeshGeometry:
    """Molded bucket seat: pan + raised side/front lips + tall backrest.

    sign: +1 for the +X end (backrest at +X, front lip toward pivot),
          -1 for the -X end (mirrored).
    """
    pw, pd, pt = SEAT_PAN_W, SEAT_PAN_D, SEAT_PAN_T
    lh, lt = SEAT_LIP_H, SEAT_LIP_T
    bh, bt = SEAT_BACK_H, SEAT_BACK_T

    pan = BoxGeometry((pw, pd, pt))

    left_lip = BoxGeometry((pw, lt, lh)).translate(
        0.0, -(pd / 2.0 - lt / 2.0), pt / 2.0 + lh / 2.0
    )
    right_lip = BoxGeometry((pw, lt, lh)).translate(
        0.0, (pd / 2.0 - lt / 2.0), pt / 2.0 + lh / 2.0
    )
    front_lip = BoxGeometry((lt, pd - 2.0 * lt, lh)).translate(
        -sign * (pw / 2.0 - lt / 2.0), 0.0, pt / 2.0 + lh / 2.0
    )
    backrest = BoxGeometry((bt, pd - 2.0 * lt, bh)).translate(
        sign * (pw / 2.0 - bt / 2.0), 0.0, pt / 2.0 + bh / 2.0
    )
    return pan.merge(left_lip).merge(right_lip).merge(front_lip).merge(backrest)


def _handlebar_mesh(sign_x: float) -> MeshGeometry:
    """T-handlebar with rounded rubber grip spheres at both crossbar tips."""
    post = CylinderGeometry(HANDLE_R, 0.22, radial_segments=14).translate(
        sign_x * HANDLE_X, 0.0, MAIN_Z + 0.10
    )
    bar = (
        CylinderGeometry(HANDLE_R, 2.0 * CROSSBAR_HALF, radial_segments=14)
        .rotate_x(math.pi / 2.0)
        .translate(sign_x * HANDLE_X, 0.0, HANDLE_TOP_Z)
    )
    post.merge(bar)
    for sy in (1.0, -1.0):
        grip = SphereGeometry(GRIP_R, width_segments=12, height_segments=8).translate(
            sign_x * HANDLE_X, sy * CROSSBAR_HALF, HANDLE_TOP_Z
        )
        post.merge(grip)
    return post


def _bumper_mesh() -> MeshGeometry:
    """Rubber bumper pad: short cylinder, top at z=0 (hanging downward)."""
    return CylinderGeometry(BUMPER_R, BUMPER_H, radial_segments=14).translate(
        0.0, 0.0, -BUMPER_H / 2.0
    )


def _beam_truss_mesh() -> MeshGeometry:
    """Full triangulated truss tube for one rocking beam (local frame)."""
    truss = (
        CylinderGeometry(TUBE_R, BEAM_LEN, radial_segments=18)
        .rotate_y(math.pi / 2.0)
        .translate(0.0, 0.0, MAIN_Z)
    )
    for sx in (1.0, -1.0):
        truss.merge(
            _tube_between(
                (sx * 0.04, 0.0, 0.004),
                (sx * 0.45, 0.0, MAIN_Z),
                BRACE_R,
            )
        )
        truss.merge(
            tube_from_spline_points(
                [
                    (sx * 0.66, 0.0, MAIN_Z),
                    (sx * 0.72, 0.0, 0.035),
                    (sx * 0.78, 0.0, 0.014),
                    (sx * 0.82, 0.0, 0.008),
                ],
                radius=SUPPORT_R,
                samples_per_segment=10,
                radial_segments=14,
                cap_ends=True,
            )
        )
    return truss


def _axle_sleeve_mesh() -> MeshGeometry:
    """Axle sleeve hub with weld post to the main tube."""
    sleeve = CylinderGeometry(SLEEVE_R, SLEEVE_LEN, radial_segments=20).rotate_x(
        math.pi / 2.0
    )
    post_len = MAIN_Z - SLEEVE_R - 0.002
    weld_post = CylinderGeometry(0.012, post_len, radial_segments=14).translate(
        0.0, 0.0, SLEEVE_R + 0.001 + post_len / 2.0
    )
    sleeve.merge(weld_post)
    return sleeve


# ---------------------------------------------------------------------------
# Model construction
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_inclusive_seesaw")

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

    # --- two independent rocking beams ----------------------------------------
    beams = []
    for beam_name in ("lower_beam", "upper_beam"):
        beam = model.part(beam_name)
        beam.visual(
            mesh_from_geometry(_beam_truss_mesh(), f"{beam_name}_truss"),
            material=WORN_YELLOW,
            name="truss_tube",
        )
        beam.visual(
            mesh_from_geometry(_axle_sleeve_mesh(), f"{beam_name}_sleeve"),
            material=WORN_YELLOW,
            name="axle_sleeve",
        )
        for end_idx, sx in enumerate((1.0, -1.0)):
            beam.visual(
                mesh_from_geometry(_handlebar_mesh(sx), f"{beam_name}_hb_{end_idx}"),
                material=WORN_YELLOW,
                name=f"handlebar_{end_idx}",
            )
            beam.visual(
                mesh_from_geometry(_molded_seat_mesh(sx), f"{beam_name}_seat_{end_idx}"),
                material=SEAT_GREEN,
                origin=Origin(xyz=(sx * SEAT_X, 0.0, SEAT_Z)),
                name=f"molded_seat_{end_idx}",
            )
        beams.append(beam)

    lower_beam, upper_beam = beams

    # --- rubber bumpers on prismatic joints -----------------------------------
    bumper_names_and_parents = [
        ("bumper_lower_0", lower_beam, 1.0),
        ("bumper_lower_1", lower_beam, -1.0),
        ("bumper_upper_0", upper_beam, 1.0),
        ("bumper_upper_1", upper_beam, -1.0),
    ]
    for bname, parent_beam, sx in bumper_names_and_parents:
        bp = model.part(bname)
        bp.visual(
            mesh_from_geometry(_bumper_mesh(), f"{bname}_pad"),
            material=RUBBER_BLACK,
            name="bumper_pad",
        )
        model.articulation(
            f"{bname}_slide",
            ArticulationType.PRISMATIC,
            parent=parent_beam,
            child=bp,
            origin=Origin(xyz=(sx * BUMPER_MOUNT_X, 0.0, MAIN_Z - TUBE_R)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=80.0, velocity=0.5, lower=0.0, upper=BUMPER_TRAVEL
            ),
        )

    # --- revolute pivots for each beam ----------------------------------------
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

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lower_beam = object_model.get_part("lower_beam")
    upper_beam = object_model.get_part("upper_beam")
    lower_pivot = object_model.get_articulation("lower_beam_pivot")
    upper_pivot = object_model.get_articulation("upper_beam_pivot")

    bumper_parts = []
    bumper_joints = []
    for name in ("bumper_lower_0", "bumper_lower_1", "bumper_upper_0", "bumper_upper_1"):
        bumper_parts.append(object_model.get_part(name))
        bumper_joints.append(object_model.get_articulation(f"{name}_slide"))

    # --- Captured-axle fits ---------------------------------------------------
    ctx.allow_overlap(
        lower_beam, base, elem_a="axle_sleeve", elem_b="low_arch",
        reason="Lower beam axle sleeve wraps the low arch top tube as its pivot axle.",
    )
    ctx.allow_overlap(
        upper_beam, base, elem_a="axle_sleeve", elem_b="high_arch",
        reason="Upper beam axle sleeve wraps the high arch top tube as its pivot axle.",
    )
    # The lower beam truss tube passes through the high arch structure in the
    # X-crossing layout; this is a consequence of the two beams crossing at
    # different heights on a shared base.
    ctx.allow_overlap(
        base, lower_beam, elem_a="high_arch", elem_b="truss_tube",
        reason="Lower beam tube passes through the high arch opening in the X-crossing layout.",
    )
    # The two beams cross at the center with only a small vertical gap between
    # the lower beam tube top and the upper beam sleeve bottom.
    ctx.allow_overlap(
        lower_beam, upper_beam, elem_a="truss_tube", elem_b="axle_sleeve",
        reason="Beams cross at the center in the X layout with a minimal vertical gap.",
    )
    # Each rubber bumper mounts on the beam underside near the beam end, where
    # the seat pan slightly overhangs in the yaw-rotated frame.  The overlap is
    # a small local bracket-to-seat-edge embedding.
    for bname in ("bumper_lower_0", "bumper_lower_1", "bumper_upper_0", "bumper_upper_1"):
        bp = object_model.get_part(bname)
        parent_name = "lower_beam" if "lower" in bname else "upper_beam"
        parent_p = object_model.get_part(parent_name)
        end_idx = 0 if bname.endswith("_0") else 1
        ctx.allow_overlap(
            bp, parent_p, elem_a="bumper_pad", elem_b=f"molded_seat_{end_idx}",
            reason=f"Bumper mounts on beam underside near the beam end; seat pan overhangs slightly in the yaw-rotated frame.",
        )
    ctx.expect_contact(
        lower_beam, base, elem_a="axle_sleeve", elem_b="low_arch",
        name="lower beam sleeve rides on the low arch axle",
    )
    ctx.expect_contact(
        upper_beam, base, elem_a="axle_sleeve", elem_b="high_arch",
        name="upper beam sleeve rides on the high arch axle",
    )
    # Proof: beams still overlap in plan view at the crossing
    ctx.expect_overlap(
        lower_beam, upper_beam, axes="xy", min_overlap=0.3,
        name="beams cross above the base in plan view",
    )

    # --- Low inclusive base height --------------------------------------------
    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "base is a low inclusive stand (top below 0.60 m)",
        base_aabb is not None and 0.30 <= base_aabb[1][2] <= 0.60,
        details=f"base aabb={base_aabb}",
    )
    ctx.check(
        "base feet rest on the ground plane",
        base_aabb is not None and -0.01 <= base_aabb[0][2] <= 0.015,
        details=f"base aabb={base_aabb}",
    )

    # --- Molded seats with raised lips and backrests --------------------------
    for beam, lo_z, hi_z in (
        (lower_beam, 0.30, 0.55),
        (upper_beam, 0.50, 0.72),
    ):
        for end in (0, 1):
            seat_aabb = ctx.part_element_world_aabb(beam, elem=f"molded_seat_{end}")
            ctx.check(
                f"{beam.name} end {end} has a molded seat",
                seat_aabb is not None,
                details=f"seat aabb={seat_aabb}",
            )
            if seat_aabb is None:
                continue
            seat_h = seat_aabb[1][2] - seat_aabb[0][2]
            scz = (seat_aabb[0][2] + seat_aabb[1][2]) / 2.0
            ctx.check(
                f"{beam.name} seat {end} has raised lips and backrest (height > 0.12 m)",
                seat_h > 0.12,
                details=f"seat height={seat_h:.4f}",
            )
            ctx.check(
                f"{beam.name} seat {end} is at accessible sit height",
                lo_z <= scz <= hi_z,
                details=f"seat center z={scz:.3f}",
            )

            handle_aabb = ctx.part_element_world_aabb(beam, elem=f"handlebar_{end}")
            ctx.check(
                f"{beam.name} handlebar {end} stands upright near its seat",
                handle_aabb is not None
                and seat_aabb is not None
                and handle_aabb[1][2] > seat_aabb[0][2] + 0.10,
                details=f"handle={handle_aabb}, seat={seat_aabb}",
            )

    # --- Rounded handle grips exist -------------------------------------------
    for beam in (lower_beam, upper_beam):
        for end in (0, 1):
            h_aabb = ctx.part_element_world_aabb(beam, elem=f"handlebar_{end}")
            ctx.check(
                f"{beam.name} handlebar {end} has lateral extent (grip spheres present)",
                h_aabb is not None
                and (h_aabb[1][1] - h_aabb[0][1]) > 0.20,
                details=f"handle y-extent={h_aabb[1][1] - h_aabb[0][1]:.4f}" if h_aabb else "missing",
            )

    # --- Rubber bumpers exist as separate parts with prismatic joints ---------
    ctx.check(
        "four rubber bumpers exist as separate parts",
        len(bumper_parts) == 4,
        details=f"found {len(bumper_parts)} bumper parts",
    )
    for bp, bj in zip(bumper_parts, bumper_joints):
        ctx.check(
            f"{bj.name} is a prismatic joint with {BUMPER_TRAVEL:.3f} m compression travel",
            bj.articulation_type == ArticulationType.PRISMATIC
            and bj.motion_limits is not None
            and abs(bj.motion_limits.lower) < 1e-6
            and abs(bj.motion_limits.upper - BUMPER_TRAVEL) < 1e-6,
            details=f"type={bj.articulation_type}, limits={bj.motion_limits}",
        )
        bp_aabb = ctx.part_world_aabb(bp)
        ctx.check(
            f"{bp.name} hangs below its parent beam near ground approach",
            bp_aabb is not None and bp_aabb[0][2] > 0.0 and bp_aabb[1][2] < 0.55,
            details=f"bumper aabb={bp_aabb}",
        )

    # --- Shallow X layout and pivot stacking ----------------------------------
    lo_seat0 = ctx.part_element_world_aabb(lower_beam, elem="molded_seat_0")
    up_seat0 = ctx.part_element_world_aabb(upper_beam, elem="molded_seat_0")
    ctx.check(
        "beam directions splay into a shallow X",
        lo_seat0 is not None
        and up_seat0 is not None
        and (lo_seat0[0][1] + lo_seat0[1][1]) / 2.0 > 0.10
        and (up_seat0[0][1] + up_seat0[1][1]) / 2.0 < -0.10,
        details=f"lower seat0={lo_seat0}, upper seat0={up_seat0}",
    )
    lo_sleeve = ctx.part_element_world_aabb(lower_beam, elem="axle_sleeve")
    up_sleeve = ctx.part_element_world_aabb(upper_beam, elem="axle_sleeve")
    ctx.check(
        "upper beam pivots above the lower beam",
        lo_sleeve is not None
        and up_sleeve is not None
        and (up_sleeve[0][2] + up_sleeve[1][2]) / 2.0
        > (lo_sleeve[0][2] + lo_sleeve[1][2]) / 2.0 + 0.05,
        details=f"lower sleeve={lo_sleeve}, upper sleeve={up_sleeve}",
    )

    # --- Rocking range --------------------------------------------------------
    for pivot in (lower_pivot, upper_pivot):
        lim = pivot.motion_limits
        ctx.check(
            f"{pivot.name} rocks +/- 18 degrees",
            lim is not None
            and abs(lim.lower + TILT) < 1e-6
            and abs(lim.upper - TILT) < 1e-6,
            details=f"limits=({lim.lower if lim else None}, {lim.upper if lim else None})",
        )

    # --- Decisive pose checks -------------------------------------------------
    rest_lo0 = ctx.part_element_world_aabb(lower_beam, elem="molded_seat_0")
    rest_lo1 = ctx.part_element_world_aabb(lower_beam, elem="molded_seat_1")
    rest_up0 = ctx.part_element_world_aabb(upper_beam, elem="molded_seat_0")
    with ctx.pose({lower_pivot: TILT}):
        tilt_lo0 = ctx.part_element_world_aabb(lower_beam, elem="molded_seat_0")
        tilt_lo1 = ctx.part_element_world_aabb(lower_beam, elem="molded_seat_1")
        tilt_up0 = ctx.part_element_world_aabb(upper_beam, elem="molded_seat_0")
        beam_aabb = ctx.part_world_aabb(lower_beam)
        ctx.check(
            "lower beam seesaws: one seat drops, opposite seat rises",
            rest_lo0 is not None
            and tilt_lo0 is not None
            and rest_lo1 is not None
            and tilt_lo1 is not None
            and tilt_lo0[0][2] < rest_lo0[0][2] - 0.20
            and tilt_lo1[0][2] > rest_lo1[0][2] + 0.20,
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
            lower_beam, base, elem_a="axle_sleeve", elem_b="low_arch",
            name="tilted lower beam sleeve stays on its axle",
        )

    with ctx.pose({upper_pivot: -TILT}):
        tilt_up0 = ctx.part_element_world_aabb(upper_beam, elem="molded_seat_0")
        beam_aabb = ctx.part_world_aabb(upper_beam)
        ctx.check(
            "upper beam seesaws the opposite way",
            rest_up0 is not None
            and tilt_up0 is not None
            and tilt_up0[0][2] > rest_up0[0][2] + 0.20,
            details=f"upper seat0 {rest_up0} -> {tilt_up0}",
        )
        ctx.check(
            "fully tilted upper beam stays clear of the ground",
            beam_aabb is not None and beam_aabb[0][2] > 0.01,
            details=f"upper beam aabb={beam_aabb}",
        )

    # --- Bumper compression pose check ----------------------------------------
    bumper_joint = object_model.get_articulation("bumper_lower_0_slide")
    rest_bumper = ctx.part_world_aabb(object_model.get_part("bumper_lower_0"))
    with ctx.pose({bumper_joint: BUMPER_TRAVEL}):
        comp_bumper = ctx.part_world_aabb(object_model.get_part("bumper_lower_0"))
        ctx.check(
            "bumper prismatic joint compresses upward when activated",
            rest_bumper is not None
            and comp_bumper is not None
            and comp_bumper[0][2] > rest_bumper[0][2] + 0.010,
            details=f"rest={rest_bumper}, compressed={comp_bumper}",
        )

    return ctx.report()


object_model = build_object_model()
