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
# Four-seat cross playground seesaw (variant 02):
# - Perpendicular beams along X and Y forming a cross above a central base.
# - Sky-blue arched tube base (~0.7 m tall) with rubber ground pads under feet.
# - Central compression spring on a prismatic joint under the beam crossing.
# - Textured rubber footrests near each seat (raised grip ribs).
# - Each beam rocks independently +/- 18 degrees about its own pivot axle.
# ----------------------------------------------------------------------------

TUBE_R = 0.020
BRACE_R = 0.016
SUPPORT_R = 0.018
HANDLE_R = 0.016

TILT = math.radians(18.0)

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
SEAT_Z = 0.038
SEAT_SIZE = (0.26, 0.30, 0.012)
HANDLE_X = 1.04
HANDLE_TOP_Z = 0.34

# Spring parameters
SPRING_COIL_R = 0.035
SPRING_HEIGHT = 0.16
SPRING_WIRE_R = 0.006
SPRING_N_COILS = 5
SPRING_ORIGIN_Z = 0.32
SPRING_TRAVEL = 0.04

# Ground pad parameters
PAD_SIZE = (0.10, 0.10, 0.008)
PAD_Z_CENTER = 0.004
FOOT_U = ARCH_HALF_SPAN + 0.055

# Footrest parameters (in beam local frame)
FOOTREST_X = 1.15
FOOTREST_PLATFORM = (0.18, 0.12, 0.010)
FOOTREST_Z = 0.005

# Materials
SKY_BLUE = Material("sky_blue_paint", rgba=(0.33, 0.62, 0.84, 1.0))
WORN_YELLOW = Material("worn_yellow_paint", rgba=(0.87, 0.74, 0.12, 1.0))
RUST_BROWN = Material("rust_brown_steel", rgba=(0.42, 0.21, 0.13, 1.0))
RUBBER_BLACK = Material("rubber_black", rgba=(0.12, 0.12, 0.12, 1.0))
SPRING_STEEL = Material("spring_steel", rgba=(0.55, 0.56, 0.54, 1.0))
FOOTREST_RUBBER = Material("footrest_rubber", rgba=(0.22, 0.20, 0.18, 1.0))


def _tube_between(
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    radius: float,
    *,
    radial_segments: int = 16,
) -> MeshGeometry:
    """Straight capped tube between two 3D points."""
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
    """Inverted-U arch tube in the vertical plane spanned by axis_xy."""
    ax, ay = axis_xy
    shoulder = 0.52 if top_z < 0.65 else 0.66
    profile = [
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
    points = [(u * ax, u * ay, z) for (u, z) in profile]
    return tube_from_spline_points(
        points,
        radius=TUBE_R,
        samples_per_segment=10,
        radial_segments=16,
        cap_ends=True,
    )


def _spring_mesh() -> MeshGeometry:
    """Helical coil spring."""
    points: list[tuple[float, float, float]] = []
    n_per_coil = 16
    total = SPRING_N_COILS * n_per_coil
    for i in range(total + 1):
        t = i / total
        angle = t * SPRING_N_COILS * 2.0 * math.pi
        x = SPRING_COIL_R * math.cos(angle)
        y = SPRING_COIL_R * math.sin(angle)
        z = t * SPRING_HEIGHT
        points.append((x, y, z))
    return tube_from_spline_points(
        points,
        radius=SPRING_WIRE_R,
        samples_per_segment=6,
        radial_segments=10,
        cap_ends=True,
    )


def _beam_meshes() -> tuple[MeshGeometry, MeshGeometry, MeshGeometry, MeshGeometry]:
    """Build one rocking beam in local frame (X along beam, pivot at origin).

    Returns (truss_tube, axle_sleeve, handlebar_pos, handlebar_neg).
    """
    truss = (
        CylinderGeometry(TUBE_R, BEAM_LEN, radial_segments=18)
        .rotate_y(math.pi / 2.0)
        .translate(0.0, 0.0, MAIN_Z)
    )
    for sx in (1.0, -1.0):
        # Diagonal brace (triangulated truss)
        truss.merge(
            _tube_between(
                (sx * 0.04, 0.0, 0.005),
                (sx * 0.60, 0.0, MAIN_Z),
                BRACE_R,
            )
        )
        # Bent seat support
        truss.merge(
            tube_from_spline_points(
                [
                    (sx * 1.24, 0.0, MAIN_Z),
                    (sx * 1.34, 0.0, 0.055),
                    (sx * 1.42, 0.0, 0.020),
                    (sx * 1.49, 0.0, 0.012),
                ],
                radius=SUPPORT_R,
                samples_per_segment=10,
                radial_segments=14,
                cap_ends=True,
            )
        )
        # Footrest bracket: vertical tube from main tube down to footrest
        truss.merge(
            _tube_between(
                (sx * FOOTREST_X, 0.0, MAIN_Z),
                (sx * FOOTREST_X, 0.0, FOOTREST_Z + FOOTREST_PLATFORM[2] * 0.5),
                SUPPORT_R * 0.7,
            )
        )

    # Axle sleeve + weld post
    sleeve = CylinderGeometry(SLEEVE_R, SLEEVE_LEN, radial_segments=20).rotate_x(
        math.pi / 2.0
    )
    weld_post = CylinderGeometry(0.014, MAIN_Z - 0.024, radial_segments=14).translate(
        0.0, 0.0, (MAIN_Z + 0.024) / 2.0
    )
    sleeve.merge(weld_post)

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

    return truss, sleeve, handlebars[0], handlebars[1]


def _add_beam_part(model: ArticulatedObject, part_name: str):
    """Create a beam part with truss, sleeve, handlebars, seats, and footrests."""
    truss, sleeve, hb0, hb1 = _beam_meshes()
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
        mesh_from_geometry(hb0, f"{part_name}_hb0"),
        material=WORN_YELLOW,
        name="handlebar_0",
    )
    beam.visual(
        mesh_from_geometry(hb1, f"{part_name}_hb1"),
        material=WORN_YELLOW,
        name="handlebar_1",
    )
    beam.visual(
        Box(SEAT_SIZE),
        origin=Origin(xyz=(SEAT_X, 0.0, SEAT_Z)),
        material=RUST_BROWN,
        name="seat_plate_0",
    )
    beam.visual(
        Box(SEAT_SIZE),
        origin=Origin(xyz=(-SEAT_X, 0.0, SEAT_Z)),
        material=RUST_BROWN,
        name="seat_plate_1",
    )

    # Textured footrests: platform + raised grip ribs
    for idx, sx in enumerate((1.0, -1.0)):
        beam.visual(
            Box(FOOTREST_PLATFORM),
            origin=Origin(xyz=(sx * FOOTREST_X, 0.0, FOOTREST_Z)),
            material=FOOTREST_RUBBER,
            name=f"footrest_{idx}",
        )
        # Two raised ribs across the platform for grip texture
        rib_top_z = FOOTREST_Z + FOOTREST_PLATFORM[2] * 0.5 + 0.0025
        for rib_idx, dy in enumerate((-0.025, 0.025)):
            beam.visual(
                Box((FOOTREST_PLATFORM[0] - 0.02, 0.008, 0.005)),
                origin=Origin(xyz=(sx * FOOTREST_X, dy, rib_top_z)),
                material=FOOTREST_RUBBER,
                name=f"footrest_{idx}_rib_{rib_idx}",
            )

    return beam


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cross_seesaw_spring")

    # --- Static sky-blue base with perpendicular arches ----------------------
    base = model.part("base")

    # Arch for beam_x spans along Y (perpendicular to X beam direction)
    base.visual(
        mesh_from_geometry(_arch_mesh((0.0, 1.0), LOW_ARCH_TOP), "arch_x"),
        material=SKY_BLUE,
        name="arch_x",
    )
    # Arch for beam_y spans along X (perpendicular to Y beam direction)
    base.visual(
        mesh_from_geometry(_arch_mesh((1.0, 0.0), HIGH_ARCH_TOP), "arch_y"),
        material=SKY_BLUE,
        name="arch_y",
    )

    # Diamond cross braces connecting adjacent legs of the two arches
    CU = CROSS_BRACE_U
    CZ = CROSS_BRACE_Z
    brace_pairs = [
        ((0.0, CU, CZ), (CU, 0.0, CZ)),
        ((CU, 0.0, CZ), (0.0, -CU, CZ)),
        ((0.0, -CU, CZ), (-CU, 0.0, CZ)),
        ((-CU, 0.0, CZ), (0.0, CU, CZ)),
    ]
    for i, (p0, p1) in enumerate(brace_pairs):
        base.visual(
            mesh_from_geometry(_tube_between(p0, p1, SUPPORT_R), f"cross_brace_{i}"),
            material=SKY_BLUE,
            name=f"cross_brace_{i}",
        )

    # Spring support structure: 4 diagonal gusset tubes from the arch legs
    # (at cross-brace height) converging up to a central collar ring that
    # carries the spring plunger. Gussets terminate at the collar outer edge
    # so they do not pass through the spring center.
    collar_z = SPRING_ORIGIN_Z - 0.003
    collar_r = 0.028
    # Each gusset goes from an arch leg to the near point on the collar rim
    gusset_attachments = [
        ((0.0, CROSS_BRACE_U, CZ), (0.0, collar_r, collar_z)),
        ((0.0, -CROSS_BRACE_U, CZ), (0.0, -collar_r, collar_z)),
        ((CROSS_BRACE_U, 0.0, CZ), (collar_r, 0.0, collar_z)),
        ((-CROSS_BRACE_U, 0.0, CZ), (-collar_r, 0.0, collar_z)),
    ]
    for i, ((lx, ly, lz), (cx, cy, cz)) in enumerate(gusset_attachments):
        base.visual(
            mesh_from_geometry(
                _tube_between((lx, ly, lz), (cx, cy, cz), 0.012),
                f"spring_gusset_{i}",
            ),
            material=SKY_BLUE,
            name=f"spring_gusset_{i}",
        )
    # Central collar ring that the gussets converge into
    collar_geom = CylinderGeometry(collar_r, 0.018, radial_segments=18)
    base.visual(
        mesh_from_geometry(collar_geom, "spring_collar"),
        origin=Origin(xyz=(0.0, 0.0, collar_z)),
        material=SKY_BLUE,
        name="spring_collar",
    )

    # Rubber ground pads under each arch foot
    pad_positions = [
        (0.0, -FOOT_U),  # arch_x foot, -Y
        (0.0, +FOOT_U),  # arch_x foot, +Y
        (-FOOT_U, 0.0),  # arch_y foot, -X
        (+FOOT_U, 0.0),  # arch_y foot, +X
    ]
    for i, (px, py) in enumerate(pad_positions):
        base.visual(
            Box(PAD_SIZE),
            origin=Origin(xyz=(px, py, PAD_Z_CENTER)),
            material=RUBBER_BLACK,
            name=f"ground_pad_{i}",
        )

    # --- Two perpendicular rocking beams ------------------------------------
    beam_x = _add_beam_part(model, "beam_x")
    beam_y = _add_beam_part(model, "beam_y")

    # --- Central compression spring -----------------------------------------
    spring = model.part("spring_plunger")
    spring.visual(
        mesh_from_geometry(_spring_mesh(), "spring_coil"),
        material=SPRING_STEEL,
        name="coil",
    )
    # Top cap plate
    top_cap_geom = CylinderGeometry(0.042, 0.008, radial_segments=20)
    spring.visual(
        mesh_from_geometry(top_cap_geom, "spring_top_cap"),
        origin=Origin(xyz=(0.0, 0.0, SPRING_HEIGHT + 0.004)),
        material=SPRING_STEEL,
        name="top_cap",
    )
    # The coil itself provides the visual bottom of the spring assembly.

    # --- Articulations ------------------------------------------------------
    tilt_limits = MotionLimits(effort=150.0, velocity=2.5, lower=-TILT, upper=TILT)

    # beam_x: along world X, rocks about world Y (axis perpendicular to beam)
    model.articulation(
        "beam_x_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=beam_x,
        origin=Origin(xyz=(0.0, 0.0, LOW_ARCH_TOP)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=tilt_limits,
    )

    # beam_y: along world Y, rocks about world X (perpendicular to beam)
    # rpy yaw=pi/2 rotates beam local X onto world Y
    model.articulation(
        "beam_y_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=beam_y,
        origin=Origin(xyz=(0.0, 0.0, HIGH_ARCH_TOP), rpy=(0.0, 0.0, math.pi / 2.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=tilt_limits,
    )

    # Spring: prismatic compression under the beam crossing
    # axis=(0,0,-1) so positive q drives the spring downward (compression)
    # Position spring so coil bottom contacts collar top
    model.articulation(
        "spring_joint",
        ArticulationType.PRISMATIC,
        parent=base,
        child=spring,
        origin=Origin(xyz=(0.0, 0.0, SPRING_ORIGIN_Z - 0.002)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=200.0, velocity=0.5, lower=0.0, upper=SPRING_TRAVEL
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    beam_x = object_model.get_part("beam_x")
    beam_y = object_model.get_part("beam_y")
    spring = object_model.get_part("spring_plunger")
    bx_pivot = object_model.get_articulation("beam_x_pivot")
    by_pivot = object_model.get_articulation("beam_y_pivot")
    s_joint = object_model.get_articulation("spring_joint")

    # --- Captured-axle fits: sleeves wrap arch top tubes ---------------------
    ctx.allow_overlap(
        beam_x,
        base,
        elem_a="axle_sleeve",
        elem_b="arch_x",
        reason="beam_x axle sleeve wraps the arch_x top tube as its pivot axle.",
    )
    ctx.allow_overlap(
        beam_y,
        base,
        elem_a="axle_sleeve",
        elem_b="arch_y",
        reason="beam_y axle sleeve wraps the arch_y top tube as its pivot axle.",
    )
    # --- Spring support embedding: gussets and collar embed into spring -------
    for i in range(4):
        ctx.allow_overlap(
            base,
            spring,
            elem_a=f"spring_gusset_{i}",
            elem_b="coil",
            reason=f"spring_gusset_{i} is welded into the spring collar; small local embed is intentional.",
        )
    ctx.allow_overlap(
        base,
        spring,
        elem_a="spring_collar",
        elem_b="coil",
        reason="Spring coil seats into the base collar ring; small local embed for seated contact.",
    )
    ctx.expect_contact(
        base,
        spring,
        elem_a="spring_collar",
        elem_b="coil",
        contact_tol=0.002,
        name="spring coil contacts the base collar",
    )

    # --- Perpendicular arch/beam crossings (unavoidable in cross layout) -----
    ctx.allow_overlap(
        base,
        beam_x,
        elem_a="arch_y",
        elem_b="truss_tube",
        reason="arch_y legs cross beam_x truss in the perpendicular cross layout.",
    )
    ctx.allow_overlap(
        base,
        beam_y,
        elem_a="arch_x",
        elem_b="truss_tube",
        reason="arch_x legs cross beam_y truss in the perpendicular cross layout.",
    )
    ctx.expect_contact(
        beam_x,
        base,
        elem_a="axle_sleeve",
        elem_b="arch_x",
        name="beam_x sleeve rides on arch_x axle",
    )
    ctx.expect_contact(
        beam_y,
        base,
        elem_a="axle_sleeve",
        elem_b="arch_y",
        name="beam_y sleeve rides on arch_y axle",
    )

    # --- Base height and ground contact --------------------------------------
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

    # --- Perpendicular beam layout -------------------------------------------
    # beam_x seats should be far apart in X, near the X axis
    bx_s0 = ctx.part_element_world_aabb(beam_x, elem="seat_plate_0")
    bx_s1 = ctx.part_element_world_aabb(beam_x, elem="seat_plate_1")
    ctx.check(
        "beam_x seats span along the X axis (perpendicular cross layout)",
        bx_s0 is not None
        and bx_s1 is not None
        and abs(
            (bx_s0[0][0] + bx_s0[1][0]) / 2.0 - (bx_s1[0][0] + bx_s1[1][0]) / 2.0
        )
        > 2.5,
        details=f"s0={bx_s0}, s1={bx_s1}",
    )

    # beam_y seats should be far apart in Y, near the Y axis
    by_s0 = ctx.part_element_world_aabb(beam_y, elem="seat_plate_0")
    by_s1 = ctx.part_element_world_aabb(beam_y, elem="seat_plate_1")
    ctx.check(
        "beam_y seats span along the Y axis (perpendicular cross layout)",
        by_s0 is not None
        and by_s1 is not None
        and abs(
            (by_s0[0][1] + by_s0[1][1]) / 2.0 - (by_s1[0][1] + by_s1[1][1]) / 2.0
        )
        > 2.5,
        details=f"s0={by_s0}, s1={by_s1}",
    )

    # Beams cross in plan view above the base
    ctx.expect_overlap(
        beam_x,
        beam_y,
        axes="xy",
        min_overlap=0.2,
        name="perpendicular beams cross above the base in plan view",
    )

    # Staggered heights: beam_y pivots above beam_x
    bx_sleeve = ctx.part_element_world_aabb(beam_x, elem="axle_sleeve")
    by_sleeve = ctx.part_element_world_aabb(beam_y, elem="axle_sleeve")
    ctx.check(
        "beam_y pivots above beam_x (staggered to avoid collision)",
        bx_sleeve is not None
        and by_sleeve is not None
        and (by_sleeve[0][2] + by_sleeve[1][2]) / 2.0
        > (bx_sleeve[0][2] + bx_sleeve[1][2]) / 2.0 + 0.10,
        details=f"bx_sleeve={bx_sleeve}, by_sleeve={by_sleeve}",
    )

    # --- Spring prismatic mechanism ------------------------------------------
    s_lim = s_joint.motion_limits
    ctx.check(
        "spring joint is prismatic with compression travel",
        s_lim is not None and s_lim.lower == 0.0 and s_lim.upper > 0.02,
        details=f"limits=({s_lim.lower if s_lim else None}, {s_lim.upper if s_lim else None})",
    )

    spring_aabb = ctx.part_world_aabb(spring)
    ctx.check(
        "spring plunger has visible coil geometry",
        spring_aabb is not None and spring_aabb[1][2] - spring_aabb[0][2] > 0.10,
        details=f"spring aabb={spring_aabb}",
    )
    ctx.check(
        "spring sits under the beam crossing point",
        spring_aabb is not None and spring_aabb[1][2] < LOW_ARCH_TOP + 0.02,
        details=f"spring top z={spring_aabb[1][2] if spring_aabb else None}",
    )

    # Spring compression pose: plunger moves downward
    rest_spring_top = spring_aabb[1][2] if spring_aabb else 0.0
    with ctx.pose({s_joint: SPRING_TRAVEL}):
        comp_aabb = ctx.part_world_aabb(spring)
        ctx.check(
            "spring compresses downward under prismatic motion",
            comp_aabb is not None and comp_aabb[1][2] < rest_spring_top - 0.02,
            details=f"rest top={rest_spring_top:.4f}, compressed top={comp_aabb[1][2] if comp_aabb else None}",
        )

    # --- Rubber ground pads --------------------------------------------------
    for i in range(4):
        pad_aabb = ctx.part_element_world_aabb(base, elem=f"ground_pad_{i}")
        ctx.check(
            f"rubber ground pad {i} exists near ground under base legs",
            pad_aabb is not None and pad_aabb[0][2] < 0.02,
            details=f"pad aabb={pad_aabb}",
        )

    # --- Textured footrests near each seat -----------------------------------
    for beam in (beam_x, beam_y):
        for idx in (0, 1):
            fr = ctx.part_element_world_aabb(beam, elem=f"footrest_{idx}")
            ctx.check(
                f"{beam.name} has textured footrest {idx} near seat",
                fr is not None,
                details=f"footrest aabb={fr}",
            )
            if fr is not None:
                seat = ctx.part_element_world_aabb(beam, elem=f"seat_plate_{idx}")
                if seat is not None:
                    seat_cx = (seat[0][0] + seat[1][0]) / 2.0
                    fr_cx = (fr[0][0] + fr[1][0]) / 2.0
                    seat_cy = (seat[0][1] + seat[1][1]) / 2.0
                    fr_cy = (fr[0][1] + fr[1][1]) / 2.0
                    dist = math.hypot(seat_cx - fr_cx, seat_cy - fr_cy)
                    ctx.check(
                        f"{beam.name} footrest {idx} is inboard of its seat",
                        0.15 < dist < 0.55,
                        details=f"footrest-seat distance={dist:.3f}",
                    )

    # --- Rocking range +/- 18 degrees on both pivots -------------------------
    for pivot in (bx_pivot, by_pivot):
        lim = pivot.motion_limits
        ctx.check(
            f"{pivot.name} rocks +/- 18 degrees",
            lim is not None
            and abs(lim.lower + TILT) < 1e-6
            and abs(lim.upper - TILT) < 1e-6,
            details=f"limits=({lim.lower if lim else None}, {lim.upper if lim else None})",
        )

    # --- Decisive pose: beam_x seesaws, beam_y stays still ------------------
    rest_bx_s0 = ctx.part_element_world_aabb(beam_x, elem="seat_plate_0")
    rest_bx_s1 = ctx.part_element_world_aabb(beam_x, elem="seat_plate_1")
    rest_by_s0 = ctx.part_element_world_aabb(beam_y, elem="seat_plate_0")

    with ctx.pose({bx_pivot: TILT}):
        tilt_bx_s0 = ctx.part_element_world_aabb(beam_x, elem="seat_plate_0")
        tilt_bx_s1 = ctx.part_element_world_aabb(beam_x, elem="seat_plate_1")
        tilt_by_s0 = ctx.part_element_world_aabb(beam_y, elem="seat_plate_0")
        bx_aabb = ctx.part_world_aabb(beam_x)

        ctx.check(
            "beam_x seesaws: one seat drops, opposite seat rises",
            rest_bx_s0 is not None
            and tilt_bx_s0 is not None
            and rest_bx_s1 is not None
            and tilt_bx_s1 is not None
            and tilt_bx_s0[0][2] < rest_bx_s0[0][2] - 0.35
            and tilt_bx_s1[0][2] > rest_bx_s1[0][2] + 0.35,
            details=f"s0 {rest_bx_s0} -> {tilt_bx_s0}, s1 {rest_bx_s1} -> {tilt_bx_s1}",
        )
        ctx.check(
            "fully tilted beam_x stays clear of the ground",
            bx_aabb is not None and bx_aabb[0][2] > 0.02,
            details=f"beam_x aabb={bx_aabb}",
        )
        ctx.check(
            "beams rock independently: beam_y holds still while beam_x rocks",
            rest_by_s0 is not None
            and tilt_by_s0 is not None
            and abs(tilt_by_s0[0][2] - rest_by_s0[0][2]) < 1e-6,
            details=f"beam_y s0 {rest_by_s0} -> {tilt_by_s0}",
        )
        ctx.expect_contact(
            beam_x,
            base,
            elem_a="axle_sleeve",
            elem_b="arch_x",
            name="tilted beam_x sleeve stays on its axle",
        )

    # --- Decisive pose: beam_y seesaws the other way -------------------------
    with ctx.pose({by_pivot: -TILT}):
        tilt_by_s0 = ctx.part_element_world_aabb(beam_y, elem="seat_plate_0")
        by_aabb = ctx.part_world_aabb(beam_y)

        ctx.check(
            "beam_y seesaws independently: near seat rises",
            rest_by_s0 is not None
            and tilt_by_s0 is not None
            and tilt_by_s0[0][2] > rest_by_s0[0][2] + 0.35,
            details=f"beam_y s0 {rest_by_s0} -> {tilt_by_s0}",
        )
        ctx.check(
            "fully tilted beam_y stays clear of the ground",
            by_aabb is not None and by_aabb[0][2] > 0.02,
            details=f"beam_y aabb={by_aabb}",
        )
        ctx.expect_contact(
            beam_y,
            base,
            elem_a="axle_sleeve",
            elem_b="arch_y",
            name="tilted beam_y sleeve stays on its axle",
        )

    return ctx.report()


object_model = build_object_model()
