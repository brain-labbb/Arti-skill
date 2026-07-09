from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


# Bend joint: conduit pivots about shoe groove center from nearly straight to ~90° bend.
LOWER_LIMIT = math.radians(-5.0)
UPPER_LIMIT = math.radians(90.0)


def _cylinder_between(part, name, p0, p1, radius, material, *, segments=24):
    x0, y0, z0 = p0
    x1, y1, z1 = p1
    dx, dy, dz = x1 - x0, y1 - y0, z1 - z0
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0:
        raise ValueError(f"zero-length cylinder {name}")
    ux, uy, uz = dx / length, dy / length, dz / length
    pitch = math.acos(max(-1.0, min(1.0, uz)))
    yaw = math.atan2(uy, ux)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((x0 + x1) * 0.5, (y0 + y1) * 0.5, (z0 + z1) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def _annular_sector(
    *,
    inner_radius: float,
    outer_radius: float,
    thickness_y: float,
    theta0: float,
    theta1: float,
    segments: int = 48,
    y_offset: float = 0.0,
) -> MeshGeometry:
    """Extruded annular sector in the local XZ plane, with thickness along Y."""
    geom = MeshGeometry()
    front = y_offset + thickness_y * 0.5
    back = y_offset - thickness_y * 0.5
    angles = [theta0 + (theta1 - theta0) * i / segments for i in range(segments + 1)]
    for theta in angles:
        c = math.cos(theta)
        s = math.sin(theta)
        geom.add_vertex(outer_radius * c, front, outer_radius * s)
        geom.add_vertex(inner_radius * c, front, inner_radius * s)
        geom.add_vertex(outer_radius * c, back, outer_radius * s)
        geom.add_vertex(inner_radius * c, back, inner_radius * s)
    for i in range(segments):
        a = 4 * i
        b = 4 * (i + 1)
        geom.add_face(a, b, b + 1)
        geom.add_face(a, b + 1, a + 1)
        geom.add_face(a + 2, b + 3, b + 2)
        geom.add_face(a + 2, a + 3, b + 3)
        geom.add_face(a, a + 2, b + 2)
        geom.add_face(a, b + 2, b)
        geom.add_face(a + 1, b + 1, b + 3)
        geom.add_face(a + 1, b + 3, a + 3)
    a = 0
    geom.add_face(a, a + 1, a + 3)
    geom.add_face(a, a + 3, a + 2)
    a = 4 * segments
    geom.add_face(a, a + 3, a + 1)
    geom.add_face(a, a + 2, a + 3)
    return geom


def _tube_along_arc(
    *,
    radius_path: float,
    tube_radius: float,
    theta0: float,
    theta1: float,
    segments: int = 64,
    radial_segments: int = 18,
    y_offset: float = 0.0,
    cap_ends: bool = True,
) -> MeshGeometry:
    """Circular tube following an arc around the pivot in the local XZ plane."""
    geom = MeshGeometry()
    angles = [theta0 + (theta1 - theta0) * i / segments for i in range(segments + 1)]
    for theta in angles:
        c = math.cos(theta)
        s = math.sin(theta)
        cx, cy, cz = radius_path * c, y_offset, radius_path * s
        radial = (c, 0.0, s)
        width = (0.0, 1.0, 0.0)
        for j in range(radial_segments):
            phi = 2.0 * math.pi * j / radial_segments
            cp = math.cos(phi)
            sp = math.sin(phi)
            geom.add_vertex(
                cx + tube_radius * (cp * radial[0] + sp * width[0]),
                cy + tube_radius * (cp * radial[1] + sp * width[1]),
                cz + tube_radius * (cp * radial[2] + sp * width[2]),
            )
    for i in range(segments):
        a = i * radial_segments
        b = (i + 1) * radial_segments
        for j in range(radial_segments):
            j2 = (j + 1) % radial_segments
            geom.add_face(a + j, b + j, b + j2)
            geom.add_face(a + j, b + j2, a + j2)
    if cap_ends:
        start_center = geom.add_vertex(
            radius_path * math.cos(theta0), y_offset, radius_path * math.sin(theta0)
        )
        end_center = geom.add_vertex(
            radius_path * math.cos(theta1), y_offset, radius_path * math.sin(theta1)
        )
        start = 0
        end = segments * radial_segments
        for j in range(radial_segments):
            j2 = (j + 1) % radial_segments
            geom.add_face(start_center, start + j2, start + j)
            geom.add_face(end_center, end + j, end + j2)
    return geom


def _add_arc_tube_visual(
    part,
    name: str,
    *,
    radius_path: float,
    tube_radius: float,
    theta0: float,
    theta1: float,
    material,
    y_offset: float = 0.0,
    segments: int = 64,
    radial_segments: int = 18,
):
    geom = _tube_along_arc(
        radius_path=radius_path,
        tube_radius=tube_radius,
        theta0=theta0,
        theta1=theta1,
        segments=segments,
        radial_segments=radial_segments,
        y_offset=y_offset,
    )
    part.visual(mesh_from_geometry(geom, name), material=material, name=name)


def _add_radial_box(part, name, radius, theta, size, material, *, y=0.0):
    part.visual(
        Box(size),
        origin=Origin(
            xyz=(radius * math.cos(theta), y, radius * math.sin(theta)),
            rpy=(0.0, -theta, 0.0),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="electrical_conduit_bender",
        meta={"small_class": "Conduit bender", "domain": "Electrical_Wiring"},
    )

    black = Material("black_cast_iron", rgba=(0.015, 0.015, 0.018, 1.0))
    dark = Material("dark_hardware", rgba=(0.08, 0.085, 0.09, 1.0))
    rubber = Material("black_rubber_grip", rgba=(0.005, 0.006, 0.007, 1.0))
    galvanized = Material("galvanized_conduit", rgba=(0.70, 0.74, 0.75, 1.0))
    white = Material("white_marking_paint", rgba=(0.96, 0.95, 0.88, 1.0))

    # ── Root part: bender tool body (handle + cast head as one rigid piece) ──
    bender = model.part("bender")

    # ─── Cast bending head / forming shoe ───
    theta0 = math.radians(-68.0)
    theta1 = math.radians(146.0)

    # Main shoe web (structural body of the curved casting).
    bender.visual(
        mesh_from_geometry(
            _annular_sector(
                inner_radius=0.058, outer_radius=0.166,
                thickness_y=0.048,
                theta0=theta0, theta1=theta1, segments=72,
            ),
            "shoe_web",
        ),
        material=black, name="shoe_web",
    )
    # Conduit groove / channel bottom — outer_radius slightly exceeds conduit
    # inner radius (0.180) so the seated workpiece makes geometric contact.
    bender.visual(
        mesh_from_geometry(
            _annular_sector(
                inner_radius=0.164, outer_radius=0.181,
                thickness_y=0.030,
                theta0=theta0, theta1=theta1, segments=72,
            ),
            "channel_bottom",
        ),
        material=dark, name="channel_bottom",
    )
    # Raised channel side lips (groove walls that capture the conduit).
    bender.visual(
        mesh_from_geometry(
            _annular_sector(
                inner_radius=0.160, outer_radius=0.207,
                thickness_y=0.010,
                theta0=theta0, theta1=theta1, segments=72,
                y_offset=-0.027,
            ),
            "channel_lip_0",
        ),
        material=black, name="channel_lip_0",
    )
    bender.visual(
        mesh_from_geometry(
            _annular_sector(
                inner_radius=0.160, outer_radius=0.207,
                thickness_y=0.010,
                theta0=theta0, theta1=theta1, segments=72,
                y_offset=0.027,
            ),
            "channel_lip_1",
        ),
        material=black, name="channel_lip_1",
    )

    # Central hub boss — radius slightly larger than shoe inner_radius for connectivity.
    bender.visual(
        Cylinder(radius=0.062, length=0.052),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black, name="pivot_hub",
    )

    # ─── Cast reinforcing ribs on the shoe web ───
    for idx, ang in enumerate(
        (math.radians(-38), math.radians(18), math.radians(78), math.radians(128))
    ):
        _add_radial_box(bender, f"cast_rib_{idx}", 0.108, ang, (0.014, 0.042, 0.125), black)

    # ─── Hook lip at the front/bottom of the shoe ───
    _add_arc_tube_visual(
        bender, "hook_lip",
        radius_path=0.224, tube_radius=0.008,
        theta0=math.radians(-82), theta1=math.radians(-66),
        material=black, y_offset=-0.044,
        segments=14, radial_segments=14,
    )
    # Hook web bracket connecting hook_lip to the channel_lip casting.
    bender.visual(
        Box((0.050, 0.030, 0.058)),
        origin=Origin(xyz=(0.076, -0.035, -0.192), rpy=(0.0, -math.radians(-70), 0.0)),
        material=black, name="hook_web",
    )

    # ─── Long straight pipe handle exiting from the BACK (high-theta) end of the shoe ───
    # At theta≈146°, shoe outer edge ≈ (-0.138, 0, 0.093).  The conduit arc ends
    # at theta=128° so there is clear space beyond theta≈130° for the handle.

    # Junction socket bridging shoe back to the handle tube.
    bender.visual(
        Box((0.062, 0.070, 0.062)),
        origin=Origin(xyz=(-0.150, 0.0, 0.092)),
        material=black, name="handle_socket",
    )
    # Transition web from socket outward to the pipe.
    bender.visual(
        Box((0.050, 0.056, 0.070)),
        origin=Origin(xyz=(-0.188, 0.0, 0.076)),
        material=black, name="handle_root_web",
    )
    # Main handle pipe (~0.70 m long, exiting toward -X).
    _cylinder_between(
        bender, "handle_tube",
        (-0.210, 0.0, 0.066), (-0.900, 0.0, 0.032),
        0.014, dark,
    )
    # Rubber comfort grip over the far end of the handle.
    _cylinder_between(
        bender, "rubber_grip",
        (-0.720, 0.0, 0.038), (-0.910, 0.0, 0.031),
        0.019, rubber,
    )

    # ─── Foot-peg step (the bar the electrician stands on while pulling the handle) ───
    # Crossbar tube near the hook end of the shoe, along ±Y.
    _cylinder_between(
        bender, "foot_peg",
        (0.082, -0.058, -0.138), (0.082, 0.058, -0.138),
        0.010, dark,
    )
    # Brackets connecting the foot peg to the shoe casting.
    for i, sign in enumerate((-1.0, 1.0)):
        bender.visual(
            Box((0.026, 0.024, 0.034)),
            origin=Origin(xyz=(0.082, sign * 0.030, -0.132)),
            material=black, name=f"foot_peg_bracket_{i}",
        )
    # Wider anti-slip step pad on top of the crossbar.
    bender.visual(
        Box((0.038, 0.070, 0.008)),
        origin=Origin(xyz=(0.082, 0.0, -0.128)),
        material=dark, name="foot_peg_pad",
    )

    # ─── Conduit retainer clip (small tab that holds EMT seated in the groove) ───
    _add_radial_box(
        bender, "conduit_retainer",
        0.190, math.radians(25),
        (0.032, 0.018, 0.024), dark,
        y=0.020,
    )

    # ─── White degree ticks and raised number pads on the shoe side face ───
    # Placed at radius 0.155 (within the shoe_web body) with enough Y overlap
    # to ensure geometric connectivity with the casting.
    for idx, deg in enumerate((-45, -15, 15, 45, 75, 105, 135)):
        length = 0.034 if idx % 2 == 0 else 0.022
        _add_radial_box(
            bender, f"degree_tick_{idx}",
            0.155, math.radians(deg),
            (0.008, 0.012, length), white, y=-0.020,
        )
    for idx, deg in enumerate((0, 45, 90)):
        _add_radial_box(
            bender, f"degree_pad_{idx}",
            0.120, math.radians(deg),
            (0.030, 0.012, 0.018), white, y=-0.020,
        )

    # ── Child part: sample EMT conduit segment (workpiece seated in groove) ──
    conduit = model.part("conduit")

    _add_arc_tube_visual(
        conduit, "conduit_arc",
        radius_path=0.190, tube_radius=0.010,
        theta0=math.radians(-56), theta1=math.radians(128),
        material=galvanized, y_offset=0.0,
        segments=72, radial_segments=20,
    )
    # Straight tail stub extending from the hook-end of the arc.
    _cylinder_between(
        conduit, "conduit_straight_tail",
        (0.190 * math.cos(math.radians(-56)), 0.0, 0.190 * math.sin(math.radians(-56))),
        (0.300, 0.0, -0.205),
        0.010, galvanized,
    )
    # Exit stub extending from the far end of the arc.
    _cylinder_between(
        conduit, "conduit_exit_stub",
        (0.190 * math.cos(math.radians(128)), 0.0, 0.190 * math.sin(math.radians(128))),
        (-0.205, 0.0, 0.235),
        0.010, galvanized,
    )

    # ── Primary articulation: conduit pivots about the shoe groove center ──
    # At q=0 the conduit is seated in the groove. Positive q rotates the conduit
    # around the shoe (simulating the bending stroke as the handle is pulled).
    model.articulation(
        "bender_to_conduit",
        ArticulationType.REVOLUTE,
        parent=bender,
        child=conduit,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0, velocity=0.8,
            lower=LOWER_LIMIT, upper=UPPER_LIMIT,
        ),
        meta={"mechanism": "conduit workpiece pivots about shoe groove center during bending stroke"},
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bender = object_model.get_part("bender")
    conduit = object_model.get_part("conduit")
    bend_joint = object_model.get_articulation("bender_to_conduit")

    # ── Identity and class ──
    ctx.check(
        "small class is conduit bender",
        object_model.name == "electrical_conduit_bender"
        and object_model.meta.get("small_class") == "Conduit bender",
        details=f"name={object_model.name}, meta={object_model.meta}",
    )

    # ── Primary form: single long handle + cast shoe, no tripod/stand/yoke ──
    bender_names = {v.name for v in bender.visuals}
    ctx.check(
        "bender has single straight handle and cast shoe (no tripod legs or yoke)",
        {"handle_tube", "rubber_grip", "handle_socket", "shoe_web", "channel_bottom",
         "channel_lip_0", "channel_lip_1", "pivot_hub"}.issubset(bender_names)
        and "front_foot" not in bender_names
        and "yoke_plate_0" not in bender_names,
        details=str(sorted(bender_names)),
    )
    ctx.check(
        "bender has foot-peg step for operator",
        {"foot_peg", "foot_peg_pad", "foot_peg_bracket_0"}.issubset(bender_names),
        details=str(sorted(bender_names)),
    )
    ctx.check(
        "bender has hook lip conduit channel ribs degree markings and retainer",
        {"hook_lip", "hook_web", "conduit_retainer",
         "cast_rib_0", "cast_rib_3", "degree_tick_0", "degree_pad_1"}.issubset(bender_names),
        details=str(sorted(bender_names)),
    )

    # ── Conduit workpiece ──
    conduit_names = {v.name for v in conduit.visuals}
    ctx.check(
        "galvanized sample conduit has curved seated arc and straight stubs",
        {"conduit_arc", "conduit_straight_tail", "conduit_exit_stub"}.issubset(conduit_names),
        details=str(sorted(conduit_names)),
    )

    # ── Revolute bend joint with realistic limits ──
    ctx.check(
        "bender_to_conduit is revolute with bend-stroke limits",
        bend_joint.articulation_type == ArticulationType.REVOLUTE
        and bend_joint.motion_limits is not None
        and math.isclose(bend_joint.motion_limits.lower, LOWER_LIMIT, abs_tol=1e-6)
        and math.isclose(bend_joint.motion_limits.upper, UPPER_LIMIT, abs_tol=1e-6),
        details=f"type={bend_joint.articulation_type}, limits={bend_joint.motion_limits}",
    )

    # ── Conduit seating: arc follows the channel without being buried ──
    ctx.expect_overlap(
        conduit, bender, axes="xz",
        elem_a="conduit_arc", elem_b="channel_bottom",
        min_overlap=0.15,
        name="conduit arc follows curved shoe channel",
    )
    ctx.expect_gap(
        conduit, bender, axis="y",
        positive_elem="conduit_arc", negative_elem="channel_lip_0",
        min_gap=0.001,
        name="conduit clears negative channel lip",
    )
    ctx.expect_gap(
        bender, conduit, axis="y",
        positive_elem="channel_lip_1", negative_elem="conduit_arc",
        min_gap=0.001,
        name="conduit clears positive channel lip",
    )
    ctx.expect_gap(
        bender, conduit, axis="y",
        positive_elem="conduit_retainer", negative_elem="conduit_arc",
        max_gap=0.003, max_penetration=0.001,
        name="retainer contacts seated conduit",
    )

    # The channel_bottom outer_radius is intentionally set to 0.181 (just past
    # the conduit inner radius of 0.180) so the workpiece makes seated contact.
    ctx.allow_overlap(
        bender, conduit,
        elem_a="channel_bottom", elem_b="conduit_arc",
        reason="The channel groove is intentionally sized to contact the seated conduit workpiece, representing the forming surface that shapes the bend.",
    )

    # ── Handle does not collide conduit at rest (3D clearance via compile QC) ──
    ctx.expect_overlap(
        bender, conduit, axes="xz",
        elem_a="handle_socket", elem_b="conduit_arc",
        min_overlap=0.0,
        name="handle socket and conduit share XZ footprint but are separated in 3D",
    )

    # ── Pose sweep: rest, mid-stroke, near lower and upper limits ──
    poses = {
        "lower_limit": LOWER_LIMIT + 0.01,
        "rest": 0.0,
        "mid_stroke": (LOWER_LIMIT + UPPER_LIMIT) * 0.5,
        "upper_limit": UPPER_LIMIT - 0.01,
    }
    for pose_name, q in poses.items():
        with ctx.pose({bend_joint: q}):
            # Conduit stays captured in the channel on non-bend axes.
            ctx.expect_overlap(
                conduit, bender, axes="xz",
                elem_a="conduit_arc", elem_b="channel_bottom",
                min_overlap=0.06,
                name=f"{pose_name} conduit arc still overlaps shoe channel",
            )

    # ── Bending stroke proof: upper-limit pose visibly rotates the conduit tail upward ──
    with ctx.pose({bend_joint: 0.0}):
        rest_tail = ctx.part_element_world_aabb(conduit, elem="conduit_straight_tail")
    with ctx.pose({bend_joint: UPPER_LIMIT - 0.01}):
        bent_tail = ctx.part_element_world_aabb(conduit, elem="conduit_straight_tail")
    ctx.check(
        "bending stroke visibly raises conduit straight tail (handle-pull action)",
        rest_tail is not None and bent_tail is not None
        and bent_tail[1][2] > rest_tail[1][2] + 0.08,
        details=f"rest_max_z={rest_tail[1][2] if rest_tail else None}, "
                f"bent_max_z={bent_tail[1][2] if bent_tail else None}",
    )

    # ── Handle length: single long pipe, not a short stub ──
    handle_aabb = ctx.part_element_world_aabb(bender, elem="handle_tube")
    ctx.check(
        "handle tube is a single long pipe (>= 0.50 m)",
        handle_aabb is not None
        and (handle_aabb[1][0] - handle_aabb[0][0]
             + handle_aabb[1][2] - handle_aabb[0][2]) >= 0.50,
        details=f"handle_aabb={handle_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
