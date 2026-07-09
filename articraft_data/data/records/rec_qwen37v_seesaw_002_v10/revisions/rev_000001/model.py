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
# Variant 10: Four-seat playground seesaw with structural changes:
# - Asymmetric seat heights (one high, one low per beam), balanced beam
# - Each handlebar pivots slightly on its own revolute joint
# - Rubber ground pads under each support leg foot
# - Safety bump stops below each beam end
# ----------------------------------------------------------------------------

TUBE_R = 0.020  # ~40 mm diameter main tubing
BRACE_R = 0.016
SUPPORT_R = 0.018
HANDLE_R = 0.016

YAW = math.radians(10.0)  # half angle of the shallow X between the beams
TILT = math.radians(18.0)  # rocking range of each beam

LOW_ARCH_TOP = 0.56  # pivot height of the lower beam
HIGH_ARCH_TOP = 0.74  # pivot height of the upper beam
ARCH_HALF_SPAN = 0.36  # ground half-span of each arch
CROSS_BRACE_Z = 0.28  # height of the short cross members
CROSS_BRACE_U = 0.315  # arch-plane coordinate of legs at CROSS_BRACE_Z

BEAM_LEN = 2.60
MAIN_Z = 0.08  # main top tube center above the pivot axis
SLEEVE_R = 0.032
SLEEVE_LEN = 0.13

# Asymmetric seat heights: seat_0 is high, seat_1 is low
SEAT_X = 1.43
SEAT_Z_HIGH = 0.065  # higher seat (above pivot axis)
SEAT_Z_LOW = 0.012   # lower seat (above pivot axis)
SEAT_SIZE = (0.26, 0.30, 0.012)

HANDLE_X = 1.04
HANDLE_POST_H = 0.28
HANDLE_TOP_Z_LOCAL = 0.26  # crossbar height in handlebar part frame
HANDLEBAR_TILT = math.radians(10.0)  # handlebar pivot range

# Bump stops: rubber blocks below each beam end
BUMP_STOP_SIZE = (0.07, 0.06, 0.06)
BUMP_STOP_X = 1.18
BUMP_STOP_Z = -0.04  # center below pivot axis

# Rubber ground pads
PAD_SIZE = (0.14, 0.10, 0.014)
FOOT_U = ARCH_HALF_SPAN + 0.045  # arch-plane foot coordinate

# Materials
SKY_BLUE = Material("sky_blue_paint", rgba=(0.33, 0.62, 0.84, 1.0))
WORN_YELLOW = Material("worn_yellow_paint", rgba=(0.87, 0.74, 0.12, 1.0))
RUST_BROWN = Material("rust_brown_steel", rgba=(0.42, 0.21, 0.13, 1.0))
RUBBER_BLACK = Material("rubber_black", rgba=(0.10, 0.10, 0.10, 1.0))


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


def _beam_truss_mesh() -> MeshGeometry:
    """Build one rocking beam truss in its local frame (X along beam, pivot at origin).

    Includes asymmetric seat supports (seat_0 high, seat_1 low).
    Does NOT include handlebars (those are separate articulated parts).
    Does NOT include bump stops (those are separate rubber visuals).
    """
    # Main top tube, full length, riding above the pivot axis
    truss = (
        CylinderGeometry(TUBE_R, BEAM_LEN, radial_segments=18)
        .rotate_y(math.pi / 2.0)
        .translate(0.0, 0.0, MAIN_Z)
    )
    for sx in (1.0, -1.0):
        # Diagonal brace from axle sleeve up to main tube (triangulated truss)
        truss.merge(
            _tube_between(
                (sx * 0.04, 0.0, 0.005),
                (sx * 0.60, 0.0, MAIN_Z),
                BRACE_R,
            )
        )

    # Asymmetric seat supports: seat_0 (+X) is HIGH, seat_1 (-X) is LOW
    # High seat support (+X end)
    truss.merge(
        tube_from_spline_points(
            [
                (1.24, 0.0, MAIN_Z),
                (1.30, 0.0, 0.085),
                (1.38, 0.0, SEAT_Z_HIGH + 0.010),
                (1.46, 0.0, SEAT_Z_HIGH),
            ],
            radius=SUPPORT_R,
            samples_per_segment=10,
            radial_segments=14,
            cap_ends=True,
        )
    )
    # Low seat support (-X end)
    truss.merge(
        tube_from_spline_points(
            [
                (-1.24, 0.0, MAIN_Z),
                (-1.34, 0.0, 0.045),
                (-1.42, 0.0, 0.020),
                (-1.49, 0.0, SEAT_Z_LOW),
            ],
            radius=SUPPORT_R,
            samples_per_segment=10,
            radial_segments=14,
            cap_ends=True,
        )
    )

    # Weld posts from main tube down to bump stop mounting points
    post_bottom = BUMP_STOP_Z + BUMP_STOP_SIZE[2] / 2.0
    post_top = MAIN_Z - TUBE_R
    post_len = post_top - post_bottom
    if post_len > 0.001:
        for sx in (1.0, -1.0):
            truss.merge(
                CylinderGeometry(0.010, post_len, radial_segments=10).translate(
                    sx * BUMP_STOP_X, 0.0, (post_top + post_bottom) / 2.0
                )
            )

    return truss


def _bump_stop_mesh(sx: float) -> MeshGeometry:
    """One rubber bump stop block in beam local frame."""
    return BoxGeometry(BUMP_STOP_SIZE).translate(sx * BUMP_STOP_X, 0.0, BUMP_STOP_Z)


def _axle_sleeve_mesh() -> MeshGeometry:
    """Axle sleeve and weld post in beam local frame."""
    sleeve = (
        CylinderGeometry(SLEEVE_R, SLEEVE_LEN, radial_segments=20)
        .rotate_x(math.pi / 2.0)
    )
    weld_post = CylinderGeometry(0.014, MAIN_Z - 0.024, radial_segments=14).translate(
        0.0, 0.0, (MAIN_Z + 0.024) / 2.0
    )
    sleeve.merge(weld_post)
    return sleeve


def _handlebar_mesh() -> MeshGeometry:
    """One handlebar in its own part frame (origin at post base, post goes up +Z)."""
    post = CylinderGeometry(HANDLE_R, HANDLE_POST_H, radial_segments=14).translate(
        0.0, 0.0, HANDLE_POST_H / 2.0
    )
    bar = (
        CylinderGeometry(HANDLE_R, 0.30, radial_segments=14)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, 0.0, HANDLE_TOP_Z_LOCAL)
    )
    return post.merge(bar)


def _foot_positions(axis_xy: tuple[float, float]) -> list[tuple[float, float, float]]:
    """Return the two ground foot positions for one arch."""
    ax, ay = axis_xy
    positions = []
    for sign in (-1.0, 1.0):
        u = sign * FOOT_U
        positions.append((u * ax, u * ay, PAD_SIZE[2] / 2.0))
    return positions


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="four_seat_tube_seesaw_v10")

    # --- static sky-blue base with rubber ground pads -------------------------
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

    # Short cross members tying the legs into one rigid stand
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

    # Rubber ground pads under each arch foot (4 feet total)
    pad_idx = 0
    for axis in (low_axis, high_axis):
        for pos in _foot_positions(axis):
            base.visual(
                Box(PAD_SIZE),
                origin=Origin(xyz=pos),
                material=RUBBER_BLACK,
                name=f"ground_pad_{pad_idx}",
            )
            pad_idx += 1

    # --- two independent yellow rocking beams ---------------------------------
    truss_mesh = _beam_truss_mesh()
    sleeve_mesh = _axle_sleeve_mesh()
    hb_mesh = _handlebar_mesh()

    beam_pivots = []
    for beam_name, arch_top, yaw_sign in [
        ("lower_beam", LOW_ARCH_TOP, YAW),
        ("upper_beam", HIGH_ARCH_TOP, -YAW),
    ]:
        beam = model.part(beam_name)
        beam.visual(
            mesh_from_geometry(truss_mesh.copy(), f"{beam_name}_truss"),
            material=WORN_YELLOW,
            name="truss_tube",
        )
        beam.visual(
            mesh_from_geometry(sleeve_mesh.copy(), f"{beam_name}_sleeve"),
            material=WORN_YELLOW,
            name="axle_sleeve",
        )
        # Safety bump stops: separate rubber blocks at each beam end
        for bump_idx, sx in enumerate((1.0, -1.0)):
            beam.visual(
                mesh_from_geometry(_bump_stop_mesh(sx), f"{beam_name}_bump_{bump_idx}"),
                material=RUBBER_BLACK,
                name=f"bump_stop_{bump_idx}",
            )
        # Asymmetric seats: seat_0 (+X) is HIGH, seat_1 (-X) is LOW
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

        # Beam pivot articulation
        pivot = model.articulation(
            f"{beam_name}_pivot",
            ArticulationType.REVOLUTE,
            parent=base,
            child=beam,
            origin=Origin(xyz=(0.0, 0.0, arch_top), rpy=(0.0, 0.0, yaw_sign)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=150.0, velocity=2.5, lower=-TILT, upper=TILT
            ),
        )
        beam_pivots.append((beam_name, beam, pivot))

        # Handlebar parts: each is a separate part with revolute pivot
        for hb_idx, sx in enumerate((1.0, -1.0)):
            hb_name = f"hb_{beam_name.split('_')[0]}_{hb_idx}"
            hb_part = model.part(hb_name)
            hb_part.visual(
                mesh_from_geometry(hb_mesh.copy(), f"{hb_name}_mesh"),
                material=WORN_YELLOW,
                name="handlebar",
            )
            # Handlebar pivots about Y axis (forward/backward tilt)
            model.articulation(
                f"{hb_name}_pivot",
                ArticulationType.REVOLUTE,
                parent=beam,
                child=hb_part,
                origin=Origin(xyz=(sx * HANDLE_X, 0.0, MAIN_Z)),
                axis=(0.0, 1.0, 0.0),
                motion_limits=MotionLimits(
                    effort=5.0,
                    velocity=1.0,
                    lower=-HANDLEBAR_TILT,
                    upper=HANDLEBAR_TILT,
                ),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lower_beam = object_model.get_part("lower_beam")
    upper_beam = object_model.get_part("upper_beam")
    lower_pivot = object_model.get_articulation("lower_beam_pivot")
    upper_pivot = object_model.get_articulation("upper_beam_pivot")

    # --- Captured-axle fits (intentional overlap) ----------------------------
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

    # --- Asymmetric seat heights ---------------------------------------------
    for beam_name in ("lower_beam", "upper_beam"):
        beam = object_model.get_part(beam_name)
        seat0 = ctx.part_element_world_aabb(beam, elem="seat_plate_0")
        seat1 = ctx.part_element_world_aabb(beam, elem="seat_plate_1")
        ctx.check(
            f"{beam_name} has asymmetric seat heights",
            seat0 is not None and seat1 is not None,
            details=f"seat0={seat0}, seat1={seat1}",
        )
        if seat0 and seat1:
            seat0_cz = (seat0[0][2] + seat0[1][2]) / 2.0
            seat1_cz = (seat1[0][2] + seat1[1][2]) / 2.0
            height_diff = abs(seat0_cz - seat1_cz)
            ctx.check(
                f"{beam_name} seat height difference is visible (>0.03 m)",
                height_diff > 0.03,
                details=f"seat0 cz={seat0_cz:.4f}, seat1 cz={seat1_cz:.4f}, diff={height_diff:.4f}",
            )

    # --- Handlebar revolute joints -------------------------------------------
    # Allow handlebar post bases to overlap the beam tube (welded capture joint)
    for hb_name, beam_name in (("hb_lower_0", "lower_beam"), ("hb_lower_1", "lower_beam"),
                                ("hb_upper_0", "upper_beam"), ("hb_upper_1", "upper_beam")):
        hb_p = object_model.get_part(hb_name)
        beam_p = object_model.get_part(beam_name)
        ctx.allow_overlap(
            hb_p, beam_p,
            elem_a="handlebar", elem_b="truss_tube",
            reason=f"{hb_name} post base is welded into the beam main tube (capture joint).",
        )

    hb_joint_count = 0
    for art in object_model.articulations:
        if art.name.startswith("hb_") and art.name.endswith("_pivot"):
            hb_joint_count += 1
            lim = art.motion_limits
            ctx.check(
                f"{art.name} has small revolute range",
                lim is not None
                and lim.lower < 0
                and lim.upper > 0
                and abs(lim.upper) <= math.radians(15.0) + 1e-6,
                details=f"limits=({lim.lower if lim else None}, {lim.upper if lim else None})",
            )
    ctx.check(
        "all four handlebars have revolute pivot joints",
        hb_joint_count == 4,
        details=f"found {hb_joint_count} handlebar joints",
    )

    # Handlebar pose pose check: pivoting a handlebar moves its crossbar
    hb_lower_0_pivot = object_model.get_articulation("hb_lower_0_pivot")
    hb_lower_0 = object_model.get_part("hb_lower_0")
    rest_hb = ctx.part_world_aabb(hb_lower_0)
    with ctx.pose({hb_lower_0_pivot: HANDLEBAR_TILT}):
        tilted_hb = ctx.part_world_aabb(hb_lower_0)
        ctx.check(
            "handlebar pivot actually tilts the handlebar part",
            rest_hb is not None and tilted_hb is not None
            and abs(tilted_hb[1][2] - rest_hb[1][2]) > 0.001,
            details=f"rest top={rest_hb[1][2] if rest_hb else None}, tilted top={tilted_hb[1][2] if tilted_hb else None}",
        )

    # --- Rubber ground pads --------------------------------------------------
    pad_count = 0
    for v in base.visuals:
        if v.name.startswith("ground_pad_"):
            pad_count += 1
    ctx.check(
        "base has four rubber ground pads under the arch feet",
        pad_count == 4,
        details=f"found {pad_count} ground pads",
    )
    # Pads are near the ground plane
    for i in range(4):
        pad_name = f"ground_pad_{i}"
        pad_aabb = ctx.part_element_world_aabb(base, elem=pad_name)
        ctx.check(
            f"{pad_name} sits on the ground plane",
            pad_aabb is not None and pad_aabb[0][2] < 0.02 and pad_aabb[1][2] < 0.04,
            details=f"pad aabb={pad_aabb}",
        )

    # --- Safety bump stops ---------------------------------------------------
    for beam_name, arch_top in (("lower_beam", LOW_ARCH_TOP), ("upper_beam", HIGH_ARCH_TOP)):
        beam = object_model.get_part(beam_name)
        bump0 = ctx.part_element_world_aabb(beam, elem="bump_stop_0")
        bump1 = ctx.part_element_world_aabb(beam, elem="bump_stop_1")
        ctx.check(
            f"{beam_name} has rubber bump stops below beam ends",
            bump0 is not None and bump1 is not None
            and bump0[0][2] < arch_top - 0.02
            and bump1[0][2] < arch_top - 0.02,
            details=f"bump0={bump0}, bump1={bump1}, pivot z={arch_top}",
        )

    # --- Base structure checks -----------------------------------------------
    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "base is an arched stand about 0.7 m tall",
        base_aabb is not None and 0.70 <= base_aabb[1][2] <= 0.82,
        details=f"base aabb={base_aabb}",
    )

    # --- Beams cross in shallow X -------------------------------------------
    ctx.expect_overlap(
        lower_beam, upper_beam,
        axes="xy", min_overlap=0.5,
        name="beams cross above the base in plan view",
    )

    # --- Upper beam pivots above lower beam ----------------------------------
    lo_sleeve = ctx.part_element_world_aabb(lower_beam, elem="axle_sleeve")
    up_sleeve = ctx.part_element_world_aabb(upper_beam, elem="axle_sleeve")
    ctx.check(
        "upper beam pivots above the lower beam",
        lo_sleeve is not None and up_sleeve is not None
        and (up_sleeve[0][2] + up_sleeve[1][2]) / 2.0
        > (lo_sleeve[0][2] + lo_sleeve[1][2]) / 2.0 + 0.10,
        details=f"lower sleeve={lo_sleeve}, upper sleeve={up_sleeve}",
    )

    # --- Beam rocking range is +/- 18 degrees --------------------------------
    for pivot in (lower_pivot, upper_pivot):
        lim = pivot.motion_limits
        ctx.check(
            f"{pivot.name} rocks +/- 18 degrees",
            lim is not None
            and abs(lim.lower + TILT) < 1e-6
            and abs(lim.upper - TILT) < 1e-6,
            details=f"limits=({lim.lower if lim else None}, {lim.upper if lim else None})",
        )

    # --- Decisive pose: lower beam seesaws -----------------------------------
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
            rest_lo0 is not None and tilt_lo0 is not None
            and rest_lo1 is not None and tilt_lo1 is not None
            and tilt_lo0[0][2] < rest_lo0[0][2] - 0.35
            and tilt_lo1[0][2] > rest_lo1[0][2] + 0.35,
            details=f"seat0 {rest_lo0} -> {tilt_lo0}, seat1 {rest_lo1} -> {tilt_lo1}",
        )
        ctx.check(
            "fully tilted lower beam stays clear of the ground",
            beam_aabb is not None and beam_aabb[0][2] > 0.02,
            details=f"lower beam aabb={beam_aabb}",
        )
        ctx.check(
            "beams rock independently: upper beam holds still while lower rocks",
            rest_up0 is not None and tilt_up0 is not None
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
            rest_up0 is not None and tilt_up0 is not None
            and tilt_up0[0][2] > rest_up0[0][2] + 0.35,
            details=f"upper seat0 {rest_up0} -> {tilt_up0}",
        )
        ctx.check(
            "fully tilted upper beam stays clear of the ground",
            beam_aabb is not None and beam_aabb[0][2] > 0.02,
            details=f"upper beam aabb={beam_aabb}",
        )
        ctx.expect_contact(
            upper_beam, base,
            elem_a="axle_sleeve", elem_b="high_arch",
            name="tilted upper beam sleeve stays on its axle",
        )

    return ctx.report()


object_model = build_object_model()
