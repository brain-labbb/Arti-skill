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
# Low inclusive four-seat playground seesaw (variant 07).
#
# Structural changes from parent:
# - Low arch base (~0.36 m / ~0.44 m) for inclusive accessibility.
# - Molded bucket seats with raised perimeter lips and angled backrests.
# - Each T-handlebar is a separate part on its own revolute joint (±10°).
# - Visible axle caps (disc washers) at each support bracket.
#
# Layout (world frame, Z up, base centered on origin):
# - Sky-blue base: two arched inverted-U tube legs (~40 mm dia) joined by
#   cross members, plus dark axle-cap discs at each arch top.
# - Two independent yellow rocking beams (~2.2 m), in a shallow X at yaw
#   ±10°, each a triangulated tube truss with molded seats + backrests at
#   both ends and a T-handlebar on a pivot just inboard of each seat.
# - Beam pivots: REVOLUTE, ±15°, horizontal axis ⊥ beam at midpoint.
# - Handlebar pivots: REVOLUTE, ±10°, lateral axis at post base.
# ----------------------------------------------------------------------------

TUBE_R = 0.020
BRACE_R = 0.016
SUPPORT_R = 0.018
HANDLE_R = 0.016

YAW = math.radians(10.0)
TILT = math.radians(15.0)
HB_TILT = math.radians(10.0)

LOW_ARCH_TOP = 0.36
HIGH_ARCH_TOP = 0.52
ARCH_HALF_SPAN = 0.30

BEAM_LEN = 2.20
MAIN_Z = 0.06
SLEEVE_R = 0.032
SLEEVE_LEN = 0.13
SEAT_X = 0.92
SEAT_Z = 0.025
HANDLE_X = 0.62
HANDLE_TOP_Z = 0.26

CAP_R = 0.038
CAP_T = 0.007

# Molded seat dimensions
SEAT_W = 0.26
SEAT_D = 0.28
SEAT_BASE_T = 0.012
LIP_H = 0.034
LIP_T = 0.010
BACKREST_H = 0.22
BACKREST_T = 0.010
BACKREST_ANGLE = math.radians(12)

SKY_BLUE = Material("sky_blue_paint", rgba=(0.33, 0.62, 0.84, 1.0))
WORN_YELLOW = Material("worn_yellow_paint", rgba=(0.87, 0.74, 0.12, 1.0))
SEAT_GREEN = Material("molded_green_plastic", rgba=(0.18, 0.52, 0.28, 1.0))
DARK_GRAY = Material("dark_gray_metal", rgba=(0.22, 0.22, 0.24, 1.0))


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
    """Inverted-U arch tube scaled for the given top height."""
    ax, ay = axis_xy
    shoulder = top_z - 0.06
    mid_z = top_z * 0.55
    cross_z = top_z * 0.50
    cross_u = ARCH_HALF_SPAN * 0.87
    hs = ARCH_HALF_SPAN
    profile_uz = [
        (-hs - 0.05, 0.020),
        (-hs - 0.02, 0.028),
        (-hs * 0.95, 0.08),
        (-cross_u, cross_z),
        (-hs * 0.72, mid_z),
        (-hs * 0.50, shoulder),
        (-0.06, top_z),
        (0.0, top_z),
        (0.06, top_z),
        (hs * 0.50, shoulder),
        (hs * 0.72, mid_z),
        (cross_u, cross_z),
        (hs * 0.95, 0.08),
        (hs + 0.02, 0.028),
        (hs + 0.05, 0.020),
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
    """Molded bucket seat: base plate, raised perimeter lips, angled backrest.

    sign = +1 for the +X beam end (backrest at +X outer edge),
    sign = -1 for the -X beam end (backrest at -X outer edge).
    The seat sits in the beam frame: X along beam, Y lateral, Z up.
    """
    hw = SEAT_W / 2.0
    hd = SEAT_D / 2.0
    base_z = SEAT_BASE_T / 2.0

    # Thin base plate
    result = BoxGeometry((SEAT_W, SEAT_D, SEAT_BASE_T))

    # Raised lip walls sitting on top of the base plate
    lip_base = SEAT_BASE_T + LIP_H / 2.0

    # Side lips (along X, at ±Y edges)
    for sy in (1.0, -1.0):
        lip = BoxGeometry((SEAT_W, LIP_T, LIP_H))
        lip.translate(0.0, sy * (hd - LIP_T / 2.0), lip_base)
        result.merge(lip)

    # Back lip (at outer X edge, where backrest sits)
    back_lip = BoxGeometry((LIP_T, SEAT_D - 2 * LIP_T, LIP_H))
    back_lip.translate(sign * (hw - LIP_T / 2.0), 0.0, lip_base)
    result.merge(back_lip)

    # Front lip (at inner X edge, lower for leg clearance)
    front_lip_h = LIP_H * 0.55
    front_lip = BoxGeometry((LIP_T, SEAT_D - 2 * LIP_T, front_lip_h))
    front_lip.translate(-sign * (hw - LIP_T / 2.0), 0.0, SEAT_BASE_T + front_lip_h / 2.0)
    result.merge(front_lip)

    # Backrest panel: rises from the outer edge, tilted back by BACKREST_ANGLE
    backrest = BoxGeometry((BACKREST_T, SEAT_W * 0.88, BACKREST_H))
    # Rotate about Y to tilt backward (away from center)
    backrest.rotate((0.0, 1.0, 0.0), sign * BACKREST_ANGLE)
    # Position: at the outer edge, rising from the seat base
    backrest.translate(
        sign * (hw + 0.002),
        0.0,
        SEAT_BASE_T + BACKREST_H / 2.0 * math.cos(BACKREST_ANGLE),
    )
    result.merge(backrest)

    return result


def _handlebar_mesh() -> MeshGeometry:
    """T-handlebar in its own local frame: post rises along +Z from origin."""
    post_h = HANDLE_TOP_Z - MAIN_Z
    post = CylinderGeometry(HANDLE_R, post_h, radial_segments=14)
    post.translate(0.0, 0.0, post_h / 2.0)
    bar = CylinderGeometry(HANDLE_R, 0.28, radial_segments=14)
    bar.rotate_x(math.pi / 2.0)
    bar.translate(0.0, 0.0, post_h)
    post.merge(bar)
    # Small grip caps at bar ends
    for sy in (1.0, -1.0):
        cap = CylinderGeometry(HANDLE_R * 1.3, 0.025, radial_segments=12)
        cap.rotate_x(math.pi / 2.0)
        cap.translate(0.0, sy * 0.15, post_h)
        post.merge(cap)
    return post


def _beam_truss_mesh() -> MeshGeometry:
    """One rocking beam truss in its local frame (X along beam, pivot at origin)."""
    truss = (
        CylinderGeometry(TUBE_R, BEAM_LEN, radial_segments=18)
        .rotate_y(math.pi / 2.0)
        .translate(0.0, 0.0, MAIN_Z)
    )
    for sx in (1.0, -1.0):
        # Diagonal brace from axle sleeve area up to the main tube
        truss.merge(
            _tube_between(
                (sx * 0.04, 0.0, 0.005),
                (sx * 0.50, 0.0, MAIN_Z),
                BRACE_R,
            )
        )
        # Short bent seat support dropping from main tube to seat level
        truss.merge(
            tube_from_spline_points(
                [
                    (sx * 0.78, 0.0, MAIN_Z),
                    (sx * 0.85, 0.0, 0.045),
                    (sx * 0.90, 0.0, 0.020),
                    (sx * 0.96, 0.0, 0.012),
                ],
                radius=SUPPORT_R,
                samples_per_segment=10,
                radial_segments=14,
                cap_ends=True,
            )
        )
    return truss


def _sleeve_mesh() -> MeshGeometry:
    """Axle sleeve + weld post connecting sleeve to main tube."""
    sleeve = (
        CylinderGeometry(SLEEVE_R, SLEEVE_LEN, radial_segments=20)
        .rotate_x(math.pi / 2.0)
    )
    weld_post = CylinderGeometry(0.014, MAIN_Z - 0.024, radial_segments=14)
    weld_post.translate(0.0, 0.0, (MAIN_Z + 0.024) / 2.0)
    sleeve.merge(weld_post)
    return sleeve


def _axle_cap_mesh() -> MeshGeometry:
    """Flat disc axle cap centered at origin, face normal to Y."""
    cap = CylinderGeometry(CAP_R, CAP_T, radial_segments=20)
    cap.rotate_x(math.pi / 2.0)
    return cap


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_inclusive_seesaw")

    # --- static sky-blue base ------------------------------------------------
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
    cross_z = LOW_ARCH_TOP * 0.50
    cross_u = ARCH_HALF_SPAN * 0.87
    leg_y = cross_u * math.cos(YAW)
    leg_x = cross_u * math.sin(YAW)
    for idx, sy in enumerate((1.0, -1.0)):
        brace = _tube_between(
            (-leg_x - 0.012, sy * leg_y, cross_z),
            (leg_x + 0.012, sy * leg_y, cross_z),
            SUPPORT_R,
        )
        base.visual(
            mesh_from_geometry(brace, f"cross_brace_{idx}"),
            material=SKY_BLUE,
            name=f"cross_brace_{idx}",
        )

    # Axle cap discs at each arch top (visible bracket end caps)
    cap_geom_base = _axle_cap_mesh()
    for arch_idx, (arch_top, beam_yaw) in enumerate(
        [(LOW_ARCH_TOP, YAW), (HIGH_ARCH_TOP, -YAW)]
    ):
        for side in (1.0, -1.0):
            cap = cap_geom_base.clone()
            # Position at arch top, offset laterally from center along beam lateral
            lat = side * (SLEEVE_LEN / 2.0 + CAP_T)
            # Beam lateral direction in world: rotate (0,1,0) by beam yaw about Z
            cy, sy_ = math.cos(beam_yaw), math.sin(beam_yaw)
            wx = -sy_ * lat
            wy = cy * lat
            cap.translate(wx, wy, arch_top)
            base.visual(
                mesh_from_geometry(cap, f"axle_cap_{arch_idx}_{int(side > 0)}"),
                material=DARK_GRAY,
                name=f"axle_cap_{arch_idx}_{int(side > 0)}",
            )

    # --- rocking beams with molded seats ------------------------------------
    beam_pivots = []
    handlebar_parts = []

    for beam_idx, (beam_name, arch_top, yaw_sign) in enumerate(
        [
            ("beam_0", LOW_ARCH_TOP, 1.0),
            ("beam_1", HIGH_ARCH_TOP, -1.0),
        ]
    ):
        beam_yaw = yaw_sign * YAW
        beam = model.part(beam_name)

        # Truss tube assembly
        beam.visual(
            mesh_from_geometry(_beam_truss_mesh(), f"{beam_name}_truss"),
            material=WORN_YELLOW,
            name="truss_tube",
        )

        # Axle sleeve
        beam.visual(
            mesh_from_geometry(_sleeve_mesh(), f"{beam_name}_sleeve"),
            material=WORN_YELLOW,
            name="axle_sleeve",
        )

        # Molded seats with backrests at each end
        for end_idx, sx in enumerate((1.0, -1.0)):
            seat_mesh = _molded_seat_mesh(sx)
            beam.visual(
                mesh_from_geometry(seat_mesh, f"{beam_name}_seat_{end_idx}"),
                origin=Origin(xyz=(sx * SEAT_X, 0.0, SEAT_Z)),
                material=SEAT_GREEN,
                name=f"seat_{end_idx}",
            )

        # Handlebar parts (separate, on revolute pivots)
        for end_idx, sx in enumerate((1.0, -1.0)):
            hb_name = f"hb_{beam_idx}_{end_idx}"
            hb_part = model.part(hb_name)
            hb_part.visual(
                mesh_from_geometry(_handlebar_mesh(), f"{hb_name}_mesh"),
                material=WORN_YELLOW,
                name="handlebar",
            )

            # Handlebar revolute joint: parent=beam, child=handlebar part
            # Origin at base of post in beam local frame
            # Axis along Y (lateral) so handlebar tilts fore/aft
            model.articulation(
                f"{hb_name}_pivot",
                ArticulationType.REVOLUTE,
                parent=beam,
                child=hb_part,
                origin=Origin(xyz=(sx * HANDLE_X, 0.0, MAIN_Z)),
                axis=(0.0, 1.0, 0.0),
                motion_limits=MotionLimits(
                    effort=8.0,
                    velocity=2.0,
                    lower=-HB_TILT,
                    upper=HB_TILT,
                ),
            )
            handlebar_parts.append((hb_name, beam_idx, end_idx, sx))

        # Beam pivot articulation
        pivot_name = f"{beam_name}_pivot"
        model.articulation(
            pivot_name,
            ArticulationType.REVOLUTE,
            parent=base,
            child=beam,
            origin=Origin(xyz=(0.0, 0.0, arch_top), rpy=(0.0, 0.0, beam_yaw)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=150.0,
                velocity=2.5,
                lower=-TILT,
                upper=TILT,
            ),
        )
        beam_pivots.append(pivot_name)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    beam_0 = object_model.get_part("beam_0")
    beam_1 = object_model.get_part("beam_1")
    pivot_0 = object_model.get_articulation("beam_0_pivot")
    pivot_1 = object_model.get_articulation("beam_1_pivot")

    # --- Captured-axle fits: sleeves wrap the arch top tubes ---------------
    ctx.allow_overlap(
        beam_0, base,
        elem_a="axle_sleeve", elem_b="low_arch",
        reason="Beam 0 axle sleeve wraps the low arch top tube as its pivot axle.",
    )
    ctx.allow_overlap(
        beam_1, base,
        elem_a="axle_sleeve", elem_b="high_arch",
        reason="Beam 1 axle sleeve wraps the high arch top tube as its pivot axle.",
    )
    ctx.expect_contact(
        beam_0, base,
        elem_a="axle_sleeve", elem_b="low_arch",
        name="beam 0 sleeve rides on the low arch axle",
    )
    ctx.expect_contact(
        beam_1, base,
        elem_a="axle_sleeve", elem_b="high_arch",
        name="beam 1 sleeve rides on the high arch axle",
    )

    # --- Low inclusive base height ------------------------------------------
    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "base is a low inclusive stand (top below 0.60 m)",
        base_aabb is not None and 0.30 <= base_aabb[1][2] <= 0.60,
        details=f"base aabb top={base_aabb[1][2] if base_aabb else None}",
    )
    ctx.check(
        "base feet rest on the ground plane",
        base_aabb is not None and -0.01 <= base_aabb[0][2] <= 0.02,
        details=f"base aabb bottom={base_aabb[0][2] if base_aabb else None}",
    )

    # --- Molded seats with raised lips and backrests ------------------------
    for beam, beam_name in [(beam_0, "beam_0"), (beam_1, "beam_1")]:
        for end in (0, 1):
            seat_aabb = ctx.part_element_world_aabb(beam, elem=f"seat_{end}")
            ctx.check(
                f"{beam_name} seat {end} is a molded bucket seat",
                seat_aabb is not None,
                details=f"seat_{end} aabb={seat_aabb}",
            )
            if seat_aabb is not None:
                # Molded seat should be taller than a flat plate (lips + backrest)
                seat_height = seat_aabb[1][2] - seat_aabb[0][2]
                ctx.check(
                    f"{beam_name} seat {end} has raised lips/backrest (height > 0.04 m)",
                    seat_height > 0.04,
                    details=f"seat height={seat_height:.4f}",
                )
                # Backrest extends above a normal seat height
                ctx.check(
                    f"{beam_name} seat {end} backrest extends well above seat base",
                    seat_aabb[1][2] > seat_aabb[0][2] + 0.08,
                    details=f"seat top z={seat_aabb[1][2]:.3f}, bottom z={seat_aabb[0][2]:.3f}",
                )

    # --- Visible axle caps at support brackets ------------------------------
    cap_names = [f"axle_cap_{i}_{s}" for i in (0, 1) for s in (0, 1)]
    for cap_name in cap_names:
        cap_aabb = ctx.part_element_world_aabb(base, elem=cap_name)
        ctx.check(
            f"axle cap {cap_name} exists at the support bracket",
            cap_aabb is not None,
            details=f"{cap_name} aabb={cap_aabb}",
        )

    # --- Handlebars: intentional weld-joint overlap at post base -------------
    hb_pivot_names = [
        "hb_0_0_pivot", "hb_0_1_pivot",
        "hb_1_0_pivot", "hb_1_1_pivot",
    ]
    hb_beam_map = {
        "hb_0_0": beam_0, "hb_0_1": beam_0,
        "hb_1_0": beam_1, "hb_1_1": beam_1,
    }
    for hb_pivot_name in hb_pivot_names:
        hb_joint = object_model.get_articulation(hb_pivot_name)
        ctx.check(
            f"{hb_pivot_name} is a revolute handlebar pivot",
            hb_joint is not None and hb_joint.articulation_type == ArticulationType.REVOLUTE,
            details=f"type={hb_joint.articulation_type if hb_joint else None}",
        )
        if hb_joint is not None:
            lim = hb_joint.motion_limits
            ctx.check(
                f"{hb_pivot_name} has small pivot range (±{HB_TILT:.3f} rad)",
                lim is not None
                and abs(lim.lower + HB_TILT) < 1e-6
                and abs(lim.upper - HB_TILT) < 1e-6,
                details=f"limits=({lim.lower if lim else None}, {lim.upper if lim else None})",
            )

    # Handlebar post base intentionally overlaps the beam tube (welded joint)
    for hb_name, parent_beam in hb_beam_map.items():
        hb_part = object_model.get_part(hb_name)
        ctx.allow_overlap(
            parent_beam, hb_part,
            elem_a="truss_tube", elem_b="handlebar",
            reason=f"Handlebar {hb_name} post base is welded onto the beam tube surface.",
        )
        ctx.expect_contact(
            parent_beam, hb_part,
            elem_a="truss_tube", elem_b="handlebar",
            name=f"handlebar {hb_name} post contacts beam tube at weld joint",
        )

    # --- Beam pivot range ±15° ----------------------------------------------
    for pivot in (pivot_0, pivot_1):
        lim = pivot.motion_limits
        ctx.check(
            f"{pivot.name} rocks ±{math.degrees(TILT):.0f} degrees",
            lim is not None
            and abs(lim.lower + TILT) < 1e-6
            and abs(lim.upper - TILT) < 1e-6,
            details=f"limits=({lim.lower if lim else None}, {lim.upper if lim else None})",
        )

    # --- Beams cross in plan view ------------------------------------------
    ctx.expect_overlap(
        beam_0, beam_1,
        axes="xy",
        min_overlap=0.4,
        name="beams cross above the base in plan view",
    )

    # --- Upper beam pivots above lower beam ---------------------------------
    lo_sleeve = ctx.part_element_world_aabb(beam_0, elem="axle_sleeve")
    up_sleeve = ctx.part_element_world_aabb(beam_1, elem="axle_sleeve")
    ctx.check(
        "beam 1 pivots above beam 0",
        lo_sleeve is not None
        and up_sleeve is not None
        and (up_sleeve[0][2] + up_sleeve[1][2]) / 2.0
        > (lo_sleeve[0][2] + lo_sleeve[1][2]) / 2.0 + 0.04,
        details=f"lower sleeve={lo_sleeve}, upper sleeve={up_sleeve}",
    )

    # --- Decisive pose: beam 0 rocks, beam 1 stays still -------------------
    rest_seat_0 = ctx.part_element_world_aabb(beam_0, elem="seat_0")
    rest_seat_1 = ctx.part_element_world_aabb(beam_0, elem="seat_1")
    rest_up_0 = ctx.part_element_world_aabb(beam_1, elem="seat_0")
    with ctx.pose({pivot_0: TILT}):
        tilt_seat_0 = ctx.part_element_world_aabb(beam_0, elem="seat_0")
        tilt_seat_1 = ctx.part_element_world_aabb(beam_0, elem="seat_1")
        tilt_up_0 = ctx.part_element_world_aabb(beam_1, elem="seat_0")
        beam_aabb = ctx.part_world_aabb(beam_0)
        ctx.check(
            "beam 0 seesaws: one seat drops, the opposite rises",
            rest_seat_0 is not None
            and tilt_seat_0 is not None
            and rest_seat_1 is not None
            and tilt_seat_1 is not None
            and tilt_seat_0[0][2] < rest_seat_0[0][2] - 0.15
            and tilt_seat_1[0][2] > rest_seat_1[0][2] + 0.15,
            details=f"seat0 {rest_seat_0} -> {tilt_seat_0}, seat1 {rest_seat_1} -> {tilt_seat_1}",
        )
        ctx.check(
            "tilted beam 0 stays clear of the ground",
            beam_aabb is not None and beam_aabb[0][2] > 0.01,
            details=f"beam 0 aabb={beam_aabb}",
        )
        ctx.check(
            "beams rock independently: beam 1 holds while beam 0 rocks",
            rest_up_0 is not None
            and tilt_up_0 is not None
            and abs(tilt_up_0[0][2] - rest_up_0[0][2]) < 1e-6,
            details=f"beam 1 seat0 {rest_up_0} -> {tilt_up_0}",
        )
        ctx.expect_contact(
            beam_0, base,
            elem_a="axle_sleeve", elem_b="low_arch",
            name="tilted beam 0 sleeve stays on its axle",
        )

    # --- Decisive pose: handlebar pivots ------------------------------------
    hb_0_0_pivot = object_model.get_articulation("hb_0_0_pivot")
    hb_part = object_model.get_part("hb_0_0")
    rest_hb_aabb = ctx.part_world_aabb(hb_part)
    with ctx.pose({hb_0_0_pivot: HB_TILT}):
        tilted_hb_aabb = ctx.part_world_aabb(hb_part)
        ctx.check(
            "handlebar hb_0_0 pivots: its bounding box shifts with joint angle",
            rest_hb_aabb is not None
            and tilted_hb_aabb is not None
            and abs(tilted_hb_aabb[1][0] - rest_hb_aabb[1][0]) > 0.003,
            details=f"rest_top_x={rest_hb_aabb[1][0]:.4f}, tilted_top_x={tilted_hb_aabb[1][0]:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
