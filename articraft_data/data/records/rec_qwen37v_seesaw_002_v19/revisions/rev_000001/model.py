from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ----------------------------------------------------------------------------
# Variant 19: A-frame spring-return playground seesaw.
#
# A single rocking beam on a central A-frame support with visible axle brackets
# and axle caps. A central compression spring (prismatic joint) under the beam
# provides return force. Molded seats with raised lips at each beam end.
# ----------------------------------------------------------------------------

TUBE_R = 0.020       # ~40 mm main tubing
BRACE_R = 0.016
SUPPORT_R = 0.018
HANDLE_R = 0.016

TILT = math.radians(18.0)
SPRING_TRAVEL = 0.06

# A-frame
AFRAME_APEX_Z = 0.62      # where the legs converge (below the axle)
AFRAME_AXLE_Z = 0.70      # axle height (pivot)
AFRAME_HALF_SPREAD = 0.32  # half-width of base
AFRAME_DEPTH = 0.18        # front-to-back depth
CROSS_BRACE_Z = 0.30

# Beam
BEAM_LEN = 2.60
MAIN_Z = 0.06     # main tube center above pivot
SLEEVE_R = 0.030
SLEEVE_LEN = 0.12
SEAT_X = 1.15
HANDLE_X = 0.80
HANDLE_TOP_Z = 0.30

# Bracket
BRACKET_W = 0.08
BRACKET_H = 0.10
BRACKET_T = 0.010
AXLE_CAP_R = 0.025
AXLE_CAP_T = 0.010

# Spring
SPRING_R = 0.028
SPRING_H = 0.10
SPRING_WIRE_R = 0.005
SPRING_REST_Z = 0.46    # world Z center of spring at rest

# Seat
SEAT_W = 0.28
SEAT_D = 0.30
SEAT_T = 0.015
SEAT_LIP_H = 0.028
SEAT_LIP_T = 0.008

# Materials
DARK_GREEN = Material("dark_green_paint", rgba=(0.18, 0.42, 0.22, 1.0))
WORN_YELLOW = Material("worn_yellow_paint", rgba=(0.87, 0.74, 0.12, 1.0))
STEEL_GRAY = Material("steel_gray", rgba=(0.45, 0.45, 0.48, 1.0))
RED_SEAT = Material("red_molded_plastic", rgba=(0.72, 0.15, 0.12, 1.0))
SPRING_STEEL = Material("spring_steel", rgba=(0.55, 0.55, 0.52, 1.0))


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


def _aframe_mesh() -> MeshGeometry:
    """A-frame: four tube legs converging to apex, cross braces, and vertical
    bracket posts from apex up to axle height."""
    geom = MeshGeometry()

    # Four legs from ground feet to apex convergence point
    for foot_y in (-AFRAME_HALF_SPREAD, AFRAME_HALF_SPREAD):
        for foot_x in (-AFRAME_DEPTH / 2, AFRAME_DEPTH / 2):
            leg = _tube_between(
                (foot_x, foot_y, 0.01),
                (0.0, 0.0, AFRAME_APEX_Z),
                TUBE_R,
            )
            geom.merge(leg)

    # Cross braces connecting the two legs on each Y side at mid-height
    # At CROSS_BRACE_Z, legs are at known interpolated positions
    t_brace = (CROSS_BRACE_Z - 0.01) / (AFRAME_APEX_Z - 0.01)
    leg_y_at_brace = AFRAME_HALF_SPREAD * (1.0 - t_brace)
    leg_x_at_brace = (AFRAME_DEPTH / 2.0) * (1.0 - t_brace)
    for sy in (-1.0, 1.0):
        brace = _tube_between(
            (-leg_x_at_brace, sy * leg_y_at_brace, CROSS_BRACE_Z),
            (leg_x_at_brace, sy * leg_y_at_brace, CROSS_BRACE_Z),
            BRACE_R,
        )
        geom.merge(brace)

    # Vertical bracket posts from apex to axle height (these carry the bracket)
    # Start at the exact leg convergence point so they share geometry.
    for sy in (-0.025, 0.025):
        post = _tube_between(
            (0.0, sy * 0.15, AFRAME_APEX_Z - 0.005),
            (0.0, sy, AFRAME_AXLE_Z + 0.005),
            SUPPORT_R,
        )
        geom.merge(post)

    # Spring mount: diagonal tube from cross brace junction up to spring seat
    # The mount extends past the seat plate so their top caps are coplanar
    spring_mount_top = SPRING_REST_Z - SPRING_H / 2 + 0.005
    spring_mount = _tube_between(
        (0.0, -leg_y_at_brace, CROSS_BRACE_Z),
        (0.0, 0.0, spring_mount_top + 0.008),
        SUPPORT_R,
    )
    geom.merge(spring_mount)

    # Spring seat plate: wider disk sharing the same top Z as the mount tube cap
    # so the coplanar top faces ensure geometric connectivity
    plate_r = SUPPORT_R * 1.8
    seat_plate = CylinderGeometry(plate_r, 0.016, radial_segments=20)
    seat_plate.translate(0.0, 0.0, spring_mount_top)
    geom.merge(seat_plate)

    return geom


def _bracket_mesh() -> MeshGeometry:
    """Flat bracket plates at axle height, bolted to the vertical posts."""
    geom = MeshGeometry()
    for sy in (-1.0, 1.0):
        y_off = sy * 0.038
        hw = BRACKET_W / 2
        hh = BRACKET_H / 2
        ht = BRACKET_T / 2
        verts = [
            (-hw, y_off - ht, AFRAME_AXLE_Z - hh),
            (hw, y_off - ht, AFRAME_AXLE_Z - hh),
            (hw, y_off + ht, AFRAME_AXLE_Z - hh),
            (-hw, y_off + ht, AFRAME_AXLE_Z - hh),
            (-hw, y_off - ht, AFRAME_AXLE_Z + hh),
            (hw, y_off - ht, AFRAME_AXLE_Z + hh),
            (hw, y_off + ht, AFRAME_AXLE_Z + hh),
            (-hw, y_off + ht, AFRAME_AXLE_Z + hh),
        ]
        base_idx = len(geom.vertices)
        for v in verts:
            geom.add_vertex(*v)
        faces = [
            (0, 1, 2), (0, 2, 3),
            (4, 6, 5), (4, 7, 6),
            (0, 4, 5), (0, 5, 1),
            (2, 6, 7), (2, 7, 3),
            (0, 3, 7), (0, 7, 4),
            (1, 5, 6), (1, 6, 2),
        ]
        for f in faces:
            geom.add_face(base_idx + f[0], base_idx + f[1], base_idx + f[2])
    return geom


def _axle_cap_mesh() -> MeshGeometry:
    """Axle cap disks touching the outer face of each bracket plate."""
    geom = MeshGeometry()
    for sy in (-1.0, 1.0):
        # Cap center just outside the bracket plate face
        cap_y = sy * (0.038 + BRACKET_T / 2 + AXLE_CAP_T / 2)
        cap = CylinderGeometry(AXLE_CAP_R, AXLE_CAP_T, radial_segments=20)
        cap.rotate_x(math.pi / 2)
        cap.translate(0.0, cap_y, AFRAME_AXLE_Z)
        geom.merge(cap)
        # Small connecting stub from bracket to cap
        stub = CylinderGeometry(0.008, 0.012, radial_segments=10)
        stub.rotate_x(math.pi / 2)
        stub.translate(0.0, sy * (0.038 + BRACKET_T / 2 + 0.001), AFRAME_AXLE_Z)
        geom.merge(stub)
    return geom


def _beam_truss_mesh() -> MeshGeometry:
    """Rocking beam truss in local frame (X along beam, pivot at origin)."""
    truss = (
        CylinderGeometry(TUBE_R, BEAM_LEN, radial_segments=18)
        .rotate_y(math.pi / 2.0)
        .translate(0.0, 0.0, MAIN_Z)
    )
    for sx in (1.0, -1.0):
        truss.merge(
            _tube_between(
                (sx * 0.04, 0.0, 0.005),
                (sx * 0.55, 0.0, MAIN_Z),
                BRACE_R,
            )
        )
        # Seat support tubes
        truss.merge(
            tube_from_spline_points(
                [
                    (sx * 1.00, 0.0, MAIN_Z),
                    (sx * 1.08, 0.0, 0.045),
                    (sx * 1.14, 0.0, 0.020),
                    (sx * SEAT_X, 0.0, 0.012),
                ],
                radius=SUPPORT_R,
                samples_per_segment=10,
                radial_segments=14,
                cap_ends=True,
            )
        )
    return truss


def _beam_sleeve_mesh() -> MeshGeometry:
    """Axle sleeve + weld post at beam center."""
    sleeve = (
        CylinderGeometry(SLEEVE_R, SLEEVE_LEN, radial_segments=20)
        .rotate_x(math.pi / 2.0)
    )
    # Weld post from sleeve top to main tube bottom
    post_h = MAIN_Z - SLEEVE_R
    if post_h > 0.005:
        weld_post = CylinderGeometry(0.014, post_h, radial_segments=14).translate(
            0.0, 0.0, SLEEVE_R + post_h / 2.0
        )
        sleeve.merge(weld_post)
    return sleeve


def _handlebar_mesh(x_pos: float) -> MeshGeometry:
    """T-shaped handlebar post rooted at main tube center, crossbar at top."""
    post_h = HANDLE_TOP_Z - MAIN_Z
    post = CylinderGeometry(HANDLE_R, post_h, radial_segments=14).translate(
        x_pos, 0.0, MAIN_Z + post_h / 2.0
    )
    bar = (
        CylinderGeometry(HANDLE_R, 0.28, radial_segments=14)
        .rotate_x(math.pi / 2.0)
        .translate(x_pos, 0.0, HANDLE_TOP_Z)
    )
    return post.merge(bar)


def _molded_seat_cq() -> cq.Workplane:
    """Molded bucket seat with raised lip perimeter."""
    seat = (
        cq.Workplane("XY")
        .box(SEAT_D, SEAT_W, SEAT_T)
    )
    # Raised lip walls
    lip_outer = (
        cq.Workplane("XY")
        .workplane(offset=SEAT_T / 2)
        .box(SEAT_D, SEAT_W, SEAT_LIP_H)
    )
    lip_inner_d = SEAT_D - 2 * SEAT_LIP_T
    lip_inner_w = SEAT_W - 2 * SEAT_LIP_T
    lip_inner = (
        cq.Workplane("XY")
        .workplane(offset=SEAT_T / 2 - 0.001)
        .box(lip_inner_d, lip_inner_w, SEAT_LIP_H + 0.002)
    )
    seat_with_lip = seat.union(lip_outer).cut(lip_inner)

    # Concave dish in seat surface
    dish = (
        cq.Workplane("XY")
        .workplane(offset=SEAT_T / 2 + 0.001)
        .circle(SEAT_W * 0.32)
        .extrude(-0.005)
    )
    seat_with_lip = seat_with_lip.cut(dish)
    return seat_with_lip


def _spring_mesh() -> MeshGeometry:
    """Coil spring with connecting vertical rod for mesh connectivity."""
    geom = MeshGeometry()
    n_coils = 5
    coil_spacing = SPRING_H / n_coils
    wire_r = SPRING_WIRE_R
    coil_r = SPRING_R - wire_r

    for i in range(n_coils):
        z_center = -SPRING_H / 2 + coil_spacing * (i + 0.5)
        torus = MeshGeometry()
        n_major = 28
        n_minor = 8
        for mi in range(n_major):
            theta = 2 * math.pi * mi / n_major
            for mj in range(n_minor):
                phi = 2 * math.pi * mj / n_minor
                x = (coil_r + wire_r * math.cos(phi)) * math.cos(theta)
                y = (coil_r + wire_r * math.cos(phi)) * math.sin(theta)
                z = wire_r * math.sin(phi) + z_center
                torus.add_vertex(x, y, z)
        for mi in range(n_major):
            mi_next = (mi + 1) % n_major
            for mj in range(n_minor):
                mj_next = (mj + 1) % n_minor
                v00 = mi * n_minor + mj
                v01 = mi * n_minor + mj_next
                v10 = mi_next * n_minor + mj
                v11 = mi_next * n_minor + mj_next
                torus.add_face(v00, v10, v11)
                torus.add_face(v00, v11, v01)
        geom.merge(torus)

    # Central vertical rod connecting all coils for mesh connectivity
    # Rod radius must exceed (coil_r - wire_r) to touch inner torus surface
    rod_r = coil_r + 0.001  # slightly inside the coil centerline
    rod = CylinderGeometry(rod_r, SPRING_H, radial_segments=12)
    geom.merge(rod)

    # Top and bottom end plates touching outermost coils
    for z_pos in (-SPRING_H / 2, SPRING_H / 2):
        disk = CylinderGeometry(SPRING_R, 0.005, radial_segments=20)
        disk.translate(0.0, 0.0, z_pos)
        geom.merge(disk)

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="aframe_spring_seesaw")

    # --- A-frame base (static) -----------------------------------------------
    aframe = model.part("aframe")
    aframe.visual(
        mesh_from_geometry(_aframe_mesh(), "aframe_tubes"),
        material=DARK_GREEN,
        name="aframe_tubes",
    )
    aframe.visual(
        mesh_from_geometry(_bracket_mesh(), "axle_brackets"),
        material=STEEL_GRAY,
        name="axle_brackets",
    )
    aframe.visual(
        mesh_from_geometry(_axle_cap_mesh(), "axle_caps"),
        material=STEEL_GRAY,
        name="axle_caps",
    )

    # --- Rocking beam --------------------------------------------------------
    beam = model.part("beam")
    beam.visual(
        mesh_from_geometry(_beam_truss_mesh(), "beam_truss"),
        material=WORN_YELLOW,
        name="truss_tube",
    )
    beam.visual(
        mesh_from_geometry(_beam_sleeve_mesh(), "beam_sleeve"),
        material=WORN_YELLOW,
        name="axle_sleeve",
    )
    beam.visual(
        mesh_from_geometry(_handlebar_mesh(HANDLE_X), "handlebar_0"),
        material=WORN_YELLOW,
        name="handlebar_0",
    )
    beam.visual(
        mesh_from_geometry(_handlebar_mesh(-HANDLE_X), "handlebar_1"),
        material=WORN_YELLOW,
        name="handlebar_1",
    )

    # Molded seats with raised lips
    seat_cq = _molded_seat_cq()
    beam.visual(
        mesh_from_cadquery(seat_cq, "seat_0"),
        origin=Origin(xyz=(SEAT_X, 0.0, 0.035)),
        material=RED_SEAT,
        name="seat_0",
    )
    beam.visual(
        mesh_from_cadquery(seat_cq, "seat_1"),
        origin=Origin(xyz=(-SEAT_X, 0.0, 0.035)),
        material=RED_SEAT,
        name="seat_1",
    )

    # --- Spring (prismatic) --------------------------------------------------
    spring = model.part("spring")
    spring.visual(
        mesh_from_geometry(_spring_mesh(), "spring_coil"),
        material=SPRING_STEEL,
        name="spring_coil",
    )

    # --- Articulations -------------------------------------------------------
    # Beam pivot at axle height, Y axis perpendicular to beam
    model.articulation(
        "beam_pivot",
        ArticulationType.REVOLUTE,
        parent=aframe,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, AFRAME_AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=200.0, velocity=2.0, lower=-TILT, upper=TILT),
    )

    # Spring prismatic joint under the beam
    model.articulation(
        "spring_compress",
        ArticulationType.PRISMATIC,
        parent=aframe,
        child=spring,
        origin=Origin(xyz=(0.0, 0.0, SPRING_REST_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=500.0, velocity=0.5, lower=0.0, upper=SPRING_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    aframe = object_model.get_part("aframe")
    beam = object_model.get_part("beam")
    spring = object_model.get_part("spring")
    beam_pivot = object_model.get_articulation("beam_pivot")
    spring_joint = object_model.get_articulation("spring_compress")

    # --- A-frame structure ---
    aframe_aabb = ctx.part_world_aabb(aframe)
    ctx.check(
        "aframe is about 0.7 m tall",
        aframe_aabb is not None and 0.68 <= aframe_aabb[1][2] <= 0.78,
        details=f"aframe aabb={aframe_aabb}",
    )
    ctx.check(
        "aframe feet rest on the ground",
        aframe_aabb is not None and -0.01 <= aframe_aabb[0][2] <= 0.02,
        details=f"aframe aabb={aframe_aabb}",
    )

    # Axle brackets at apex
    bracket_aabb = ctx.part_element_world_aabb(aframe, elem="axle_brackets")
    ctx.check(
        "visible axle brackets at aframe apex",
        bracket_aabb is not None
        and bracket_aabb[0][2] > AFRAME_AXLE_Z - 0.06
        and bracket_aabb[1][2] < AFRAME_AXLE_Z + 0.06,
        details=f"bracket aabb={bracket_aabb}",
    )

    # Axle caps at bracket ends
    cap_aabb = ctx.part_element_world_aabb(aframe, elem="axle_caps")
    ctx.check(
        "visible axle caps at bracket ends",
        cap_aabb is not None
        and abs((cap_aabb[0][2] + cap_aabb[1][2]) / 2 - AFRAME_AXLE_Z) < 0.03,
        details=f"axle caps aabb={cap_aabb}",
    )
    ctx.check(
        "axle caps span laterally beyond brackets",
        cap_aabb is not None
        and (cap_aabb[1][1] - cap_aabb[0][1]) > 0.08,
        details=f"axle caps Y span={cap_aabb[1][1] - cap_aabb[0][1]:.3f}",
    )

    # --- Molded seats with raised lips ---
    for end in (0, 1):
        seat_aabb = ctx.part_element_world_aabb(beam, elem=f"seat_{end}")
        ctx.check(
            f"molded seat {end} exists at beam end",
            seat_aabb is not None
            and math.hypot(
                (seat_aabb[0][0] + seat_aabb[1][0]) / 2,
                (seat_aabb[0][1] + seat_aabb[1][1]) / 2,
            ) > 0.9,
            details=f"seat_{end} aabb={seat_aabb}",
        )
        if seat_aabb:
            seat_height = seat_aabb[1][2] - seat_aabb[0][2]
            ctx.check(
                f"seat {end} has raised lip (height > 0.025 m)",
                seat_height > 0.025,
                details=f"seat_{end} height={seat_height:.4f}",
            )

    # --- Spring ---
    spring_aabb = ctx.part_world_aabb(spring)
    ctx.check(
        "spring exists under the beam center",
        spring_aabb is not None
        and spring_aabb[1][2] < AFRAME_AXLE_Z
        and spring_aabb[0][2] > 0.15,
        details=f"spring aabb={spring_aabb}",
    )

    # --- Articulation checks ---
    pivot_lim = beam_pivot.motion_limits
    ctx.check(
        "beam pivot is revolute with +/- 18 degree limits",
        pivot_lim is not None
        and abs(pivot_lim.lower + TILT) < 1e-6
        and abs(pivot_lim.upper - TILT) < 1e-6,
        details=f"pivot limits=({pivot_lim.lower if pivot_lim else None}, {pivot_lim.upper if pivot_lim else None})",
    )

    spring_lim = spring_joint.motion_limits
    ctx.check(
        "spring prismatic joint has compression travel",
        spring_lim is not None
        and spring_lim.lower >= -0.001
        and spring_lim.upper > 0.03,
        details=f"spring limits=({spring_lim.lower if spring_lim else None}, {spring_lim.upper if spring_lim else None})",
    )

    # Spring seated on aframe mount plate (intentional small overlap)
    ctx.allow_overlap(
        aframe,
        spring,
        elem_a="aframe_tubes",
        elem_b="spring_coil",
        reason="Spring bottom end plate seats on the aframe spring mount plate, representing the spring mount.",
    )
    ctx.expect_contact(
        aframe,
        spring,
        elem_a="aframe_tubes",
        elem_b="spring_coil",
        name="spring seats on aframe mount plate",
    )

    # Sleeve-on-axle: the sleeve wraps the bracket posts (bearing fit)
    ctx.allow_overlap(
        beam,
        aframe,
        elem_a="axle_sleeve",
        elem_b="aframe_tubes",
        reason="Beam axle sleeve wraps the vertical bracket posts at the pivot, representing the pivot bearing.",
    )
    ctx.allow_overlap(
        beam,
        aframe,
        elem_a="axle_sleeve",
        elem_b="axle_brackets",
        reason="Beam axle sleeve passes through the bracket plates at the pivot axle.",
    )
    ctx.expect_contact(
        beam,
        aframe,
        elem_a="axle_sleeve",
        elem_b="aframe_tubes",
        name="beam sleeve rides on aframe pivot posts",
    )

    # --- Pose checks ---
    rest_seat0 = ctx.part_element_world_aabb(beam, elem="seat_0")
    rest_seat1 = ctx.part_element_world_aabb(beam, elem="seat_1")

    with ctx.pose({beam_pivot: TILT}):
        tilt_seat0 = ctx.part_element_world_aabb(beam, elem="seat_0")
        tilt_seat1 = ctx.part_element_world_aabb(beam, elem="seat_1")
        ctx.check(
            "beam tilts: seat_0 drops and seat_1 rises",
            rest_seat0 is not None
            and tilt_seat0 is not None
            and rest_seat1 is not None
            and tilt_seat1 is not None
            and tilt_seat0[0][2] < rest_seat0[0][2] - 0.30
            and tilt_seat1[0][2] > rest_seat1[0][2] + 0.30,
            details=f"seat0 {rest_seat0} -> {tilt_seat0}, seat1 {rest_seat1} -> {tilt_seat1}",
        )
        ctx.expect_contact(
            beam,
            aframe,
            elem_a="axle_sleeve",
            elem_b="aframe_tubes",
            name="tilted beam sleeve stays on pivot",
        )

    # Spring compresses
    rest_spring_pos = ctx.part_world_position(spring)
    with ctx.pose({spring_joint: SPRING_TRAVEL * 0.8}):
        compressed_pos = ctx.part_world_position(spring)
        ctx.check(
            "spring compresses downward on prismatic actuation",
            rest_spring_pos is not None
            and compressed_pos is not None
            and compressed_pos[2] < rest_spring_pos[2] - 0.02,
            details=f"rest={rest_spring_pos}, compressed={compressed_pos}",
        )

    # Handlebars upright near seats
    for end in (0, 1):
        handle = ctx.part_element_world_aabb(beam, elem=f"handlebar_{end}")
        seat = ctx.part_element_world_aabb(beam, elem=f"seat_{end}")
        ok = handle is not None and seat is not None
        ctx.check(
            f"handlebar {end} stands upright near seat {end}",
            ok and handle[1][2] > seat[1][2] + 0.10,
            details=f"handle={handle}, seat={seat}",
        )

    return ctx.report()


object_model = build_object_model()
