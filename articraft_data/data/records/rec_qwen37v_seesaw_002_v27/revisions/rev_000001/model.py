from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CylinderGeometry,
    ExtrudeGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)

# ----------------------------------------------------------------------------
# Low inclusive four-seat playground seesaw with backrest bucket seats.
#
# Variant of the bent-tube seesaw family:
# - Lower base arches (~0.32 m and ~0.52 m) for inclusive access.
# - Molded bucket seats with raised lips and a backrest panel.
# - Each T-handlebar pivots slightly on its own revolute joint.
# - Two independent rocking beams in a shallow X, each ±18°.
# ----------------------------------------------------------------------------

TUBE_R = 0.020
BRACE_R = 0.016
SUPPORT_R = 0.018
HANDLE_R = 0.014

YAW = math.radians(10.0)
TILT = math.radians(18.0)
HANDLEBAR_TILT = math.radians(10.0)

LOW_ARCH_TOP = 0.32
HIGH_ARCH_TOP = 0.52
ARCH_HALF_SPAN = 0.30
CROSS_BRACE_Z = 0.20
CROSS_BRACE_U = 0.24

BEAM_LEN = 2.00
MAIN_Z = 0.08  # main tube height above pivot axis (enough clearance from arches)
SLEEVE_R = 0.030
SLEEVE_LEN = 0.11
SEAT_X = 1.08
SEAT_Z = 0.02
HANDLE_X = 0.78

# Molded seat dimensions
SEAT_PAN_W = 0.28
SEAT_PAN_D = 0.26
SEAT_PAN_T = 0.012
LIP_H = 0.035
LIP_T = 0.010
BACK_H = 0.18
BACK_T = 0.012
BACK_W = 0.24
BACK_ANGLE = 0.18

SKY_BLUE = Material("sky_blue_paint", rgba=(0.33, 0.62, 0.84, 1.0))
WORN_YELLOW = Material("worn_yellow_paint", rgba=(0.87, 0.74, 0.12, 1.0))
SEAT_GREEN = Material("molded_green_plastic", rgba=(0.18, 0.45, 0.22, 1.0))
HANDLEBAR_YELLOW = Material("handlebar_yellow", rgba=(0.90, 0.78, 0.10, 1.0))


def _tube_between(
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    radius: float,
    *,
    radial_segments: int = 16,
) -> MeshGeometry:
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
    ax, ay = axis_xy
    shoulder = top_z - 0.06
    mid_z = top_z * 0.65
    profile_uz = [
        (-ARCH_HALF_SPAN - 0.045, 0.018),
        (-ARCH_HALF_SPAN - 0.02, 0.024),
        (-0.28, 0.08),
        (-CROSS_BRACE_U, CROSS_BRACE_Z),
        (-0.18, mid_z),
        (-0.09, shoulder),
        (-0.04, top_z),
        (0.0, top_z),
        (0.04, top_z),
        (0.09, shoulder),
        (0.18, mid_z),
        (CROSS_BRACE_U, CROSS_BRACE_Z),
        (0.28, 0.08),
        (ARCH_HALF_SPAN + 0.02, 0.024),
        (ARCH_HALF_SPAN + 0.045, 0.018),
    ]
    points = [(u * ax, u * ay, z) for (u, z) in profile_uz]
    return tube_from_spline_points(
        points,
        radius=TUBE_R,
        samples_per_segment=10,
        radial_segments=16,
        cap_ends=True,
    )


def _build_molded_seat_mesh() -> MeshGeometry:
    """Bucket-style molded seat: dished pan with raised lips and backrest."""
    geom = MeshGeometry()

    # Seat pan
    pan_profile = rounded_rect_profile(SEAT_PAN_W, SEAT_PAN_D, 0.025, corner_segments=4)
    pan = ExtrudeGeometry(pan_profile, SEAT_PAN_T, cap=True, center=False)
    geom.merge(pan)

    # Front lip
    front_lip = BoxGeometry((SEAT_PAN_W, LIP_T, LIP_H))
    front_lip.translate(0.0, -SEAT_PAN_D / 2.0 + LIP_T / 2.0, SEAT_PAN_T + LIP_H / 2.0)
    geom.merge(front_lip)

    # Left lip
    left_lip = BoxGeometry((LIP_T, SEAT_PAN_D - LIP_T, LIP_H))
    left_lip.translate(-SEAT_PAN_W / 2.0 + LIP_T / 2.0, LIP_T / 2.0, SEAT_PAN_T + LIP_H / 2.0)
    geom.merge(left_lip)

    # Right lip
    right_lip = BoxGeometry((LIP_T, SEAT_PAN_D - LIP_T, LIP_H))
    right_lip.translate(SEAT_PAN_W / 2.0 - LIP_T / 2.0, LIP_T / 2.0, SEAT_PAN_T + LIP_H / 2.0)
    geom.merge(right_lip)

    # Backrest panel (behind the seat, slightly reclined)
    back = BoxGeometry((BACK_W, BACK_T, BACK_H))
    back_cz = SEAT_PAN_T + BACK_H / 2.0 * math.cos(BACK_ANGLE)
    back_cy = SEAT_PAN_D / 2.0 - BACK_T / 2.0 + BACK_H / 2.0 * math.sin(BACK_ANGLE)
    back.rotate_x(-BACK_ANGLE)
    back.translate(0.0, back_cy, back_cz)
    geom.merge(back)

    return geom


def _build_beam_truss() -> MeshGeometry:
    """One rocking beam truss in local frame (X along beam, pivot at origin)."""
    truss = (
        CylinderGeometry(TUBE_R, BEAM_LEN, radial_segments=18)
        .rotate_y(math.pi / 2.0)
        .translate(0.0, 0.0, MAIN_Z)
    )
    for sx in (1.0, -1.0):
        # Diagonal brace from axle sleeve up to main tube
        truss.merge(
            _tube_between(
                (sx * 0.04, 0.0, 0.005),
                (sx * 0.50, 0.0, MAIN_Z),
                BRACE_R,
            )
        )
        # Short bent seat support dropping from main tube end under the seat
        truss.merge(
            tube_from_spline_points(
                [
                    (sx * 0.90, 0.0, MAIN_Z),
                    (sx * 0.98, 0.0, 0.050),
                    (sx * 1.04, 0.0, 0.020),
                    (sx * 1.12, 0.0, 0.010),
                ],
                radius=SUPPORT_R,
                samples_per_segment=10,
                radial_segments=14,
                cap_ends=True,
            )
        )
    return truss


def _build_axle_sleeve() -> MeshGeometry:
    """Axle sleeve + weld post connecting sleeve to main tube."""
    sleeve = (
        CylinderGeometry(SLEEVE_R, SLEEVE_LEN, radial_segments=20)
        .rotate_x(math.pi / 2.0)
    )
    weld_post = CylinderGeometry(0.012, MAIN_Z - 0.022, radial_segments=14).translate(
        0.0, 0.0, (MAIN_Z + 0.022) / 2.0
    )
    sleeve.merge(weld_post)
    return sleeve


def _build_handlebar_mesh() -> MeshGeometry:
    """T-handlebar: stem penetrates beam tube (weld), post + crossbar above."""
    # Stem extends down into beam tube (represents welded-on handlebar)
    # Handlebar frame origin = beam tube centerline at handlebar position
    # Beam tube surface at z = ±TUBE_R = ±0.020 from handlebar origin
    stem = CylinderGeometry(0.010, 0.040, radial_segments=10).translate(
        0.0, 0.0, 0.005  # spans z=-0.015 to z=0.025, penetrates tube
    )
    # Post rises from above tube surface
    post = CylinderGeometry(HANDLE_R, 0.18, radial_segments=14).translate(
        0.0, 0.0, 0.115  # spans z=0.025 to z=0.205
    )
    # Crossbar at top of post
    bar = (
        CylinderGeometry(HANDLE_R, 0.24, radial_segments=14)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, 0.0, 0.20)
    )
    return stem.merge(post).merge(bar)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_inclusive_seesaw")

    # --- Static base ---------------------------------------------------------
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
            (-leg_x - 0.010, sy * leg_y, CROSS_BRACE_Z),
            (leg_x + 0.010, sy * leg_y, CROSS_BRACE_Z),
            SUPPORT_R,
        )
        base.visual(
            mesh_from_geometry(brace, f"cross_brace_{idx}"),
            material=SKY_BLUE,
            name=f"cross_brace_{idx}",
        )

    # --- Molded seat mesh (shared template) ----------------------------------
    seat_mesh = _build_molded_seat_mesh()

    # --- Lower beam ---------------------------------------------------------
    lower_beam = model.part("lower_beam")
    lower_beam.visual(
        mesh_from_geometry(_build_beam_truss(), "lower_beam_truss"),
        material=WORN_YELLOW,
        name="truss_tube",
    )
    lower_beam.visual(
        mesh_from_geometry(_build_axle_sleeve(), "lower_beam_sleeve"),
        material=WORN_YELLOW,
        name="axle_sleeve",
    )
    for end_idx, sx in enumerate((1.0, -1.0)):
        lower_beam.visual(
            mesh_from_geometry(seat_mesh.clone(), f"lower_seat_{end_idx}"),
            origin=Origin(xyz=(sx * SEAT_X, 0.0, SEAT_Z)),
            material=SEAT_GREEN,
            name=f"seat_{end_idx}",
        )

    # --- Upper beam ---------------------------------------------------------
    upper_beam = model.part("upper_beam")
    upper_beam.visual(
        mesh_from_geometry(_build_beam_truss(), "upper_beam_truss"),
        material=WORN_YELLOW,
        name="truss_tube",
    )
    upper_beam.visual(
        mesh_from_geometry(_build_axle_sleeve(), "upper_beam_sleeve"),
        material=WORN_YELLOW,
        name="axle_sleeve",
    )
    for end_idx, sx in enumerate((1.0, -1.0)):
        upper_beam.visual(
            mesh_from_geometry(seat_mesh.clone(), f"upper_seat_{end_idx}"),
            origin=Origin(xyz=(sx * SEAT_X, 0.0, SEAT_Z)),
            material=SEAT_GREEN,
            name=f"seat_{end_idx}",
        )

    # --- Handlebars as separate articulated parts ----------------------------
    handlebar_mesh_template = _build_handlebar_mesh()
    for beam_name, beam_part in [
        ("lower_beam", lower_beam),
        ("upper_beam", upper_beam),
    ]:
        for end_idx, sx in enumerate((1.0, -1.0)):
            hb_name = f"{beam_name}_handlebar_{end_idx}"
            hb_part = model.part(hb_name)
            hb_part.visual(
                mesh_from_geometry(handlebar_mesh_template.clone(), f"{hb_name}_mesh"),
                material=HANDLEBAR_YELLOW,
                name="handlebar",
            )
            # Handlebar pivot axis along beam-local Y (fore/aft tilt)
            model.articulation(
                f"beam_to_{hb_name}",
                ArticulationType.REVOLUTE,
                parent=beam_part,
                child=hb_part,
                origin=Origin(xyz=(sx * HANDLE_X, 0.0, MAIN_Z)),
                axis=(0.0, 1.0, 0.0),
                motion_limits=MotionLimits(
                    effort=2.0,
                    velocity=3.0,
                    lower=-HANDLEBAR_TILT,
                    upper=HANDLEBAR_TILT,
                ),
            )

    # --- Beam pivots --------------------------------------------------------
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


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lower_beam = object_model.get_part("lower_beam")
    upper_beam = object_model.get_part("upper_beam")
    lower_pivot = object_model.get_articulation("lower_beam_pivot")
    upper_pivot = object_model.get_articulation("upper_beam_pivot")

    # --- Captured-axle fits -------------------------------------------------
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

    # --- Handlebar weld-overlap allowances ---------------------------------
    # Each handlebar stem intentionally penetrates the beam tube (welded joint).
    for beam_name, beam_part in [("lower_beam", lower_beam), ("upper_beam", upper_beam)]:
        for end_idx in (0, 1):
            hb_name = f"{beam_name}_handlebar_{end_idx}"
            hb_part = object_model.get_part(hb_name)
            ctx.allow_overlap(
                beam_part, hb_part,
                elem_a="truss_tube", elem_b="handlebar",
                reason=f"Handlebar stem is welded into the {beam_name} tube, a small intentional overlap at the mount.",
            )
            ctx.expect_contact(
                beam_part, hb_part,
                elem_a="truss_tube", elem_b="handlebar",
                name=f"{hb_name} stem contacts the {beam_name} tube",
            )

    # --- Low inclusive base -------------------------------------------------
    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "base is a low inclusive stand (0.28-0.60 m tall)",
        base_aabb is not None and 0.28 <= base_aabb[1][2] <= 0.60,
        details=f"base aabb={base_aabb}",
    )
    ctx.check(
        "base feet rest on the ground plane",
        base_aabb is not None and -0.01 <= base_aabb[0][2] <= 0.015,
        details=f"base aabb={base_aabb}",
    )

    # --- Molded seats with raised lips and backrests -----------------------
    for beam in (lower_beam, upper_beam):
        for end_idx in (0, 1):
            seat_aabb = ctx.part_element_world_aabb(beam, elem=f"seat_{end_idx}")
            ctx.check(
                f"{beam.name} seat {end_idx} exists as a molded bucket",
                seat_aabb is not None,
                details=f"seat aabb={seat_aabb}",
            )
            if seat_aabb is not None:
                seat_dz = seat_aabb[1][2] - seat_aabb[0][2]
                seat_dx = seat_aabb[1][0] - seat_aabb[0][0]
                seat_dy = seat_aabb[1][1] - seat_aabb[0][1]
                ctx.check(
                    f"{beam.name} seat {end_idx} has backrest (height >= 0.12 m)",
                    seat_dz >= 0.12,
                    details=f"seat height={seat_dz:.3f}",
                )
                ctx.check(
                    f"{beam.name} seat {end_idx} is wide enough for bucket shape",
                    seat_dx >= 0.20 and seat_dy >= 0.20,
                    details=f"seat dx={seat_dx:.3f}, dy={seat_dy:.3f}",
                )

    # --- Seats sit at accessible height near beam ends ----------------------
    for beam, lo_z, hi_z in ((lower_beam, 0.24, 0.50), (upper_beam, 0.44, 0.70)):
        for end_idx in (0, 1):
            seat_aabb = ctx.part_element_world_aabb(beam, elem=f"seat_{end_idx}")
            if seat_aabb is not None:
                scz = (seat_aabb[0][2] + seat_aabb[1][2]) / 2.0
                ctx.check(
                    f"{beam.name} seat {end_idx} sits at low inclusive height",
                    lo_z <= scz <= hi_z,
                    details=f"seat center z={scz:.3f}",
                )

    # --- Handlebars exist as separate articulated parts ---------------------
    for beam_name in ("lower_beam", "upper_beam"):
        for end_idx in (0, 1):
            hb_name = f"{beam_name}_handlebar_{end_idx}"
            hb_part = object_model.get_part(hb_name)
            hb_joint = object_model.get_articulation(f"beam_to_{hb_name}")

            hb_aabb = ctx.part_world_aabb(hb_part)
            ctx.check(
                f"{hb_name} exists as a separate articulated part",
                hb_aabb is not None,
                details=f"handlebar aabb={hb_aabb}",
            )
            if hb_aabb is not None:
                hb_dz = hb_aabb[1][2] - hb_aabb[0][2]
                ctx.check(
                    f"{hb_name} has upright extent (>= 0.10 m)",
                    hb_dz >= 0.10,
                    details=f"handlebar dz={hb_dz:.3f}",
                )

            ctx.check(
                f"{hb_name} pivots on a revolute joint",
                hb_joint.articulation_type == ArticulationType.REVOLUTE,
                details=f"joint type={hb_joint.articulation_type}",
            )
            lim = hb_joint.motion_limits
            ctx.check(
                f"{hb_name} has small pivot range (<= 15 degrees)",
                lim is not None and abs(lim.upper) <= math.radians(15.0) + 1e-6,
                details=f"limits=({lim.lower if lim else None}, {lim.upper if lim else None})",
            )

    # --- Beam seesaw rocking range ------------------------------------------
    for pivot in (lower_pivot, upper_pivot):
        lim = pivot.motion_limits
        ctx.check(
            f"{pivot.name} rocks +/- 18 degrees",
            lim is not None
            and abs(lim.lower + TILT) < 1e-6
            and abs(lim.upper - TILT) < 1e-6,
            details=f"limits=({lim.lower if lim else None}, {lim.upper if lim else None})",
        )

    # --- Shallow X: beams cross above base ----------------------------------
    ctx.expect_overlap(
        lower_beam, upper_beam,
        axes="xy", min_overlap=0.3,
        name="beams cross above the base in plan view",
    )

    # --- Upper beam pivots above lower beam ---------------------------------
    lo_sleeve = ctx.part_element_world_aabb(lower_beam, elem="axle_sleeve")
    up_sleeve = ctx.part_element_world_aabb(upper_beam, elem="axle_sleeve")
    ctx.check(
        "upper beam pivots above the lower beam",
        lo_sleeve is not None
        and up_sleeve is not None
        and (up_sleeve[0][2] + up_sleeve[1][2]) / 2.0
        > (lo_sleeve[0][2] + lo_sleeve[1][2]) / 2.0 + 0.04,
        details=f"lower sleeve={lo_sleeve}, upper sleeve={up_sleeve}",
    )

    # --- Decisive pose: lower beam seesaws, upper stays still ---------------
    rest_lo0 = ctx.part_element_world_aabb(lower_beam, elem="seat_0")
    rest_lo1 = ctx.part_element_world_aabb(lower_beam, elem="seat_1")
    rest_up0 = ctx.part_element_world_aabb(upper_beam, elem="seat_0")
    with ctx.pose({lower_pivot: TILT}):
        tilt_lo0 = ctx.part_element_world_aabb(lower_beam, elem="seat_0")
        tilt_lo1 = ctx.part_element_world_aabb(lower_beam, elem="seat_1")
        tilt_up0 = ctx.part_element_world_aabb(upper_beam, elem="seat_0")
        ctx.check(
            "lower beam seesaws: one seat drops, the opposite rises",
            rest_lo0 is not None and tilt_lo0 is not None
            and rest_lo1 is not None and tilt_lo1 is not None
            and tilt_lo0[0][2] < rest_lo0[0][2] - 0.20
            and tilt_lo1[0][2] > rest_lo1[0][2] + 0.20,
            details=f"seat0 {rest_lo0} -> {tilt_lo0}, seat1 {rest_lo1} -> {tilt_lo1}",
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

    # --- Handlebar pivot pose check ----------------------------------------
    hb_joint_0 = object_model.get_articulation("beam_to_lower_beam_handlebar_0")
    hb_part_0 = object_model.get_part("lower_beam_handlebar_0")
    rest_hb = ctx.part_world_aabb(hb_part_0)
    with ctx.pose({hb_joint_0: HANDLEBAR_TILT}):
        tilt_hb = ctx.part_world_aabb(hb_part_0)
        ctx.check(
            "handlebar pivots visibly when tilted",
            rest_hb is not None and tilt_hb is not None
            and (
                abs(tilt_hb[0][2] - rest_hb[0][2]) > 1e-4
                or abs(tilt_hb[1][2] - rest_hb[1][2]) > 1e-4
            ),
            details=f"rest={rest_hb}, tilted={tilt_hb}",
        )

    return ctx.report()


object_model = build_object_model()
