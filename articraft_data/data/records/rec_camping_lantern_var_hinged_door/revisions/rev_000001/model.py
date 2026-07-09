from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


TAU = 2.0 * math.pi


def _radial_xy(radius: float, angle: float) -> tuple[float, float]:
    return (radius * math.cos(angle), radius * math.sin(angle))


def _cylinder_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    radius: float,
    segments: int = 18,
) -> MeshGeometry:
    """Build a slim tube between two points as a mesh cylinder."""
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 1e-9:
        raise ValueError("Cylinder endpoints must be separated.")

    geom = MeshGeometry()

    # Stable frame: local z follows the requested segment.
    wz = (dx / length, dy / length, dz / length)
    up = (0.0, 0.0, 1.0) if abs(wz[2]) < 0.92 else (1.0, 0.0, 0.0)
    ux = (
        up[1] * wz[2] - up[2] * wz[1],
        up[2] * wz[0] - up[0] * wz[2],
        up[0] * wz[1] - up[1] * wz[0],
    )
    un = math.sqrt(ux[0] * ux[0] + ux[1] * ux[1] + ux[2] * ux[2])
    ux = (ux[0] / un, ux[1] / un, ux[2] / un)
    uy = (
        wz[1] * ux[2] - wz[2] * ux[1],
        wz[2] * ux[0] - wz[0] * ux[2],
        wz[0] * ux[1] - wz[1] * ux[0],
    )

    center0 = geom.add_vertex(sx, sy, sz)
    center1 = geom.add_vertex(ex, ey, ez)
    ring0: list[int] = []
    ring1: list[int] = []
    for i in range(segments):
        a = TAU * i / segments
        ca, sa = math.cos(a), math.sin(a)
        ox = radius * (ca * ux[0] + sa * uy[0])
        oy = radius * (ca * ux[1] + sa * uy[1])
        oz = radius * (ca * ux[2] + sa * uy[2])
        ring0.append(geom.add_vertex(sx + ox, sy + oy, sz + oz))
        ring1.append(geom.add_vertex(ex + ox, ey + oy, ez + oz))
    for i in range(segments):
        j = (i + 1) % segments
        geom.add_face(ring0[i], ring0[j], ring1[j])
        geom.add_face(ring0[i], ring1[j], ring1[i])
        geom.add_face(center0, ring0[i], ring0[j])
        geom.add_face(center1, ring1[j], ring1[i])
    return geom


_CAGE_RADIUS = 0.055
_CAGE_N = 96
_CAGE_Z_BREAKS = [0.038, 0.044, 0.050, 0.107, 0.117, 0.171, 0.181, 0.187]
# Door section spans angular indices 88..6 (through 0), ≈56° arc at the front.
_DOOR_AI_START = 88
_DOOR_AI_END = 6


def _in_door_section(ai_mod: int) -> bool:
    return ai_mod >= _DOOR_AI_START or ai_mod <= _DOOR_AI_END


def _build_cage_faces(*, door_only: bool) -> MeshGeometry:
    """Shared lattice builder; emits either the main cage or the door panel."""
    cage = MeshGeometry()
    radius = _CAGE_RADIUS
    n = _CAGE_N
    z_breaks = _CAGE_Z_BREAKS
    verts: dict[tuple[int, int], int] = {}

    def vid(ai: int, zi: int) -> int:
        key = (ai % n, zi)
        if key not in verts:
            a = TAU * (ai % n) / n
            z = z_breaks[zi]
            verts[key] = cage.add_vertex(radius * math.cos(a), radius * math.sin(a), z)
        return verts[key]

    def in_vertical_bar(ai: int) -> bool:
        for bar in range(8):
            center = int(round(n * bar / 8.0))
            half = 2 if bar not in (0, 4) else 3
            delta = min((ai - center) % n, (center - ai) % n)
            if delta <= half:
                return True
        return False

    def in_horizontal_band(zi: int) -> bool:
        return zi in (0, 3, 5)

    for ai in range(n):
        ai_mod = ai % n
        is_door = _in_door_section(ai_mod)
        if door_only and not is_door:
            continue
        if not door_only and is_door:
            continue
        for zi in range(len(z_breaks) - 1):
            if in_horizontal_band(zi) or in_vertical_bar(ai):
                a0 = vid(ai, zi)
                a1 = vid(ai + 1, zi)
                b0 = vid(ai, zi + 1)
                b1 = vid(ai + 1, zi + 1)
                cage.add_face(a0, a1, b1)
                cage.add_face(a0, b1, b0)
    return cage


def _lantern_cage_mesh() -> MeshGeometry:
    """Connected cylindrical lattice approximating the lantern's wire guard cage (minus door section)."""
    return _build_cage_faces(door_only=False)


def _door_hinge_origin() -> tuple[float, float, float]:
    """World-space hinge pivot position (angular index 7, mid-height of cage)."""
    hinge_a = 7 * TAU / _CAGE_N
    hx = _CAGE_RADIUS * math.cos(hinge_a)
    hy = _CAGE_RADIUS * math.sin(hinge_a)
    hz = (_CAGE_Z_BREAKS[0] + _CAGE_Z_BREAKS[-1]) / 2.0
    return (hx, hy, hz)


def _door_panel_mesh() -> MeshGeometry:
    """Front guard panel section of the cage, in the door's local frame (relative to hinge)."""
    raw = _build_cage_faces(door_only=True)
    hx, hy, hz = _door_hinge_origin()
    radius = _CAGE_RADIUS
    n = _CAGE_N
    z_lo, z_hi = _CAGE_Z_BREAKS[0], _CAGE_Z_BREAKS[-1]

    # Rebuild mesh with vertices offset so the hinge point is at local origin.
    door = MeshGeometry()
    offset_verts: dict[int, int] = {}

    def remap(old_idx: int, x: float, y: float, z: float) -> int:
        if old_idx not in offset_verts:
            offset_verts[old_idx] = door.add_vertex(x - hx, y - hy, z - hz)
        return offset_verts[old_idx]

    # Copy raw mesh vertices and faces with offset.
    raw_verts = raw.vertices  # list of (x,y,z)
    raw_faces = raw.faces  # list of (a,b,c)
    vert_map: dict[int, int] = {}
    for i, (vx, vy, vz) in enumerate(raw_verts):
        vert_map[i] = door.add_vertex(vx - hx, vy - hy, vz - hz)
    for fa, fb, fc in raw_faces:
        door.add_face(vert_map[fa], vert_map[fb], vert_map[fc])

    # Hinge-side vertical frame bar (angular index 7, at local origin in XY).
    door.merge(
        _cylinder_between((0.0, 0.0, z_lo - hz), (0.0, 0.0, z_hi - hz), radius=0.002, segments=10)
    )
    # Latch-side vertical frame bar (angular index 88, at the free edge).
    latch_a = _DOOR_AI_START * TAU / n
    lx = radius * math.cos(latch_a) - hx
    ly = radius * math.sin(latch_a) - hy
    door.merge(
        _cylinder_between((lx, ly, z_lo - hz), (lx, ly, z_hi - hz), radius=0.002, segments=10)
    )
    return door


def _vented_base_mesh() -> MeshGeometry:
    return LatheGeometry(
        [
            (0.000, 0.020),
            (0.044, 0.020),
            (0.052, 0.026),
            (0.050, 0.036),
            (0.037, 0.044),
            (0.032, 0.049),
            (0.021, 0.052),
            (0.000, 0.052),
        ],
        segments=72,
    )


def _top_cap_mesh() -> MeshGeometry:
    return LatheGeometry(
        [
            (0.000, 0.176),
            (0.053, 0.176),
            (0.063, 0.181),
            (0.062, 0.190),
            (0.053, 0.195),
            (0.026, 0.195),
            (0.018, 0.198),
            (0.018, 0.203),
            (0.013, 0.207),
            (0.000, 0.207),
        ],
        segments=72,
    )


def _leg_mesh() -> MeshGeometry:
    leg = tube_from_spline_points(
        [
            (0.004, 0.0, -0.002),
            (0.018, 0.0, -0.056),
            (0.039, 0.0, -0.111),
            (0.059, 0.0, -0.153),
        ],
        radius=0.0022,
        samples_per_segment=10,
        radial_segments=14,
        cap_ends=True,
    )
    # Flattened rubber foot, intentionally intersecting the wire end as a crimped cap.
    leg.merge(
        _cylinder_between(
            (0.047, -0.010, -0.154),
            (0.073, 0.010, -0.154),
            radius=0.0032,
            segments=12,
        )
    )
    return leg


def _bail_mesh() -> MeshGeometry:
    path = [
        (-0.050, 0.0, 0.000),
        (-0.052, 0.0, 0.021),
        (-0.034, 0.0, 0.054),
        (0.000, 0.0, 0.066),
        (0.034, 0.0, 0.054),
        (0.052, 0.0, 0.021),
        (0.050, 0.0, 0.000),
    ]
    return tube_from_spline_points(
        path,
        radius=0.0023,
        samples_per_segment=10,
        radial_segments=16,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="hanging_tripod_camping_lantern",
        meta={
            "run_notes": (
                "Modeled only the image-referenced camping lantern and three fold-out support legs; "
                "tabletop, food, lighting ambience, and background are intentionally omitted. "
                "The image content appears consistent with the Camping Lantern category."
            )
        },
    )

    dark_metal = model.material("blackened_steel", rgba=(0.005, 0.006, 0.005, 1.0))
    olive_metal = model.material("olive_gray_metal", rgba=(0.30, 0.35, 0.31, 1.0))
    warm_glass = model.material("warm_translucent_glass", rgba=(1.0, 0.54, 0.08, 0.34))
    warm_core = model.material("warm_led_core", rgba=(1.0, 0.73, 0.20, 0.88))
    switch_black = model.material("matte_black_switch", rgba=(0.018, 0.018, 0.016, 1.0))
    white_mark = model.material("white_brand_mark", rgba=(0.93, 0.93, 0.88, 1.0))

    body = model.part("lantern_body")

    # Glowing cylindrical diffuser and brighter inner LED tube.
    body.visual(
        Cylinder(radius=0.032, length=0.144),
        origin=Origin(xyz=(0.0, 0.0, 0.111)),
        material=warm_glass,
        name="glowing_diffuser",
    )
    body.visual(
        Cylinder(radius=0.020, length=0.128),
        origin=Origin(xyz=(0.0, 0.0, 0.112)),
        material=warm_core,
        name="led_column",
    )

    body.visual(
        mesh_from_geometry(_lantern_cage_mesh(), "cylindrical_wire_cage"),
        material=dark_metal,
        name="cylindrical_wire_cage",
    )
    body.visual(
        mesh_from_geometry(_vented_base_mesh(), "vented_base"),
        material=olive_metal,
        name="vented_base",
    )
    body.visual(
        mesh_from_geometry(_top_cap_mesh(), "top_vent_cap"),
        material=olive_metal,
        name="top_vent_cap",
    )

    # Raised rectangular control/battery block and switch on the cap.
    body.visual(
        Box((0.068, 0.030, 0.020)),
        origin=Origin(xyz=(0.000, -0.004, 0.206)),
        material=olive_metal,
        name="top_control_block",
    )
    body.visual(
        Box((0.047, 0.004, 0.0012)),
        origin=Origin(xyz=(0.000, -0.020, 0.2155)),
        material=white_mark,
        name="brand_stroke",
    )
    body.visual(
        Box((0.025, 0.016, 0.004)),
        origin=Origin(xyz=(0.025, 0.025, 0.196)),
        material=switch_black,
        name="top_switch",
    )

    # Bail pivots on the cap and three leg sockets under the base.
    for x in (-0.055, 0.055):
        body.visual(
            Cylinder(radius=0.006, length=0.014),
            origin=Origin(xyz=(0.050 if x > 0.0 else -0.050, 0.0, 0.193), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_metal,
            name=f"bail_ear_{0 if x < 0.0 else 1}",
        )

    leg_socket_radius = 0.038
    leg_hinge_z = 0.024
    for i in range(3):
        angle = TAU * i / 3.0 + math.pi / 6.0
        x, y = _radial_xy(leg_socket_radius, angle)
        body.visual(
            Box((0.016, 0.010, 0.010)),
            origin=Origin(xyz=(x, y, leg_hinge_z), rpy=(0.0, 0.0, angle)),
            material=dark_metal,
            name=f"leg_socket_{i}",
        )

    bail = model.part("carry_bail")
    bail.visual(
        mesh_from_geometry(_bail_mesh(), "carry_bail_wire"),
        material=dark_metal,
        name="carry_bail_wire",
    )
    model.articulation(
        "body_to_bail",
        ArticulationType.REVOLUTE,
        parent=body,
        child=bail,
        origin=Origin(xyz=(0.0, 0.0, 0.196)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.5, lower=-1.35, upper=1.35),
    )

    # --- Hinged access door ---
    hinge_x, hinge_y, hinge_z = _door_hinge_origin()
    cage_height = _CAGE_Z_BREAKS[-1] - _CAGE_Z_BREAKS[0]

    # Vertical hinge post on the body at the door's pivot edge.
    body.visual(
        Cylinder(radius=0.003, length=cage_height),
        origin=Origin(xyz=(hinge_x, hinge_y, hinge_z)),
        material=dark_metal,
        name="door_hinge_post",
    )

    door = model.part("hinged_door")
    door.visual(
        mesh_from_geometry(_door_panel_mesh(), "door_panel_cage"),
        material=dark_metal,
        name="door_panel_cage",
    )
    # Small latch knob on the free edge of the door (companion variation ④).
    # Position in door-local frame (offset from hinge origin).
    latch_angle = _DOOR_AI_START * TAU / _CAGE_N
    latch_local_x = (_CAGE_RADIUS + 0.005) * math.cos(latch_angle) - hinge_x
    latch_local_y = (_CAGE_RADIUS + 0.005) * math.sin(latch_angle) - hinge_y
    door.visual(
        Box((0.010, 0.006, 0.010)),
        origin=Origin(xyz=(latch_local_x, latch_local_y, 0.0)),
        material=switch_black,
        name="latch_knob",
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, hinge_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=2.0, lower=0.0, upper=1.80),
    )

    leg_mesh = _leg_mesh()
    for i in range(3):
        angle = TAU * i / 3.0 + math.pi / 6.0
        x, y = _radial_xy(leg_socket_radius, angle)
        leg = model.part(f"leg_{i}")
        leg.visual(
            mesh_from_geometry(leg_mesh.copy(), f"foldout_leg_{i}"),
            material=dark_metal,
            name="leg_wire",
        )
        leg.visual(
            Cylinder(radius=0.004, length=0.017),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name="hinge_pin",
        )
        model.articulation(
            f"body_to_leg_{i}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=leg,
            origin=Origin(xyz=(x, y, leg_hinge_z), rpy=(0.0, 0.0, angle)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.25),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("lantern_body")
    bail = object_model.get_part("carry_bail")
    bail_joint = object_model.get_articulation("body_to_bail")

    ctx.expect_overlap(
        body,
        bail,
        axes="x",
        elem_a="top_vent_cap",
        elem_b="carry_bail_wire",
        min_overlap=0.085,
        name="bail spans across the top cap",
    )
    bail_box = ctx.part_element_world_aabb(bail, elem="carry_bail_wire")
    cap_box = ctx.part_element_world_aabb(body, elem="top_vent_cap")
    ctx.check(
        "upright bail rises above the vent cap",
        bail_box is not None and cap_box is not None and bail_box[1][2] > cap_box[1][2] + 0.045,
        details=f"bail_box={bail_box}, cap_box={cap_box}",
    )

    with ctx.pose({bail_joint: 1.0}):
        rotated_bail_box = ctx.part_element_world_aabb(bail, elem="carry_bail_wire")
        ctx.expect_overlap(
            body,
            bail,
            axes="x",
            elem_a="top_vent_cap",
            elem_b="carry_bail_wire",
            min_overlap=0.065,
            name="bail remains captured while rotated",
        )
    ctx.check(
        "bail hinge rotates the carry loop forward",
        bail_box is not None
        and rotated_bail_box is not None
        and rotated_bail_box[0][1] < bail_box[0][1] - 0.030,
        details=f"upright={bail_box}, rotated={rotated_bail_box}",
    )

    # --- Hinged access door tests ---
    door = object_model.get_part("hinged_door")
    door_joint = object_model.get_articulation("body_to_door")

    # The door panel's top cage band seats into the vent cap rim (same nest as main cage).
    ctx.allow_overlap(
        body,
        door,
        elem_a="top_vent_cap",
        elem_b="door_panel_cage",
        reason="The door panel's top cage band is intentionally seated into the vent cap rim, matching the main cage construction.",
    )

    # Door panel overlaps the cage footprint in Y when closed (seated in the cage opening).
    ctx.expect_overlap(
        body,
        door,
        axes="y",
        elem_a="cylindrical_wire_cage",
        elem_b="door_panel_cage",
        min_overlap=0.010,
        name="closed door panel sits within the cage footprint",
    )

    # Door panel and body cage maintain contact at the hinge post.
    ctx.expect_contact(
        door,
        body,
        elem_a="door_panel_cage",
        elem_b="door_hinge_post",
        contact_tol=0.005,
        name="door panel contacts the hinge post when closed",
    )

    # Door swings outward: at a positive angle the door's extent grows radially.
    closed_door_box = ctx.part_element_world_aabb(door, elem="door_panel_cage")
    with ctx.pose({door_joint: 1.2}):
        opened_door_box = ctx.part_element_world_aabb(door, elem="door_panel_cage")
    ctx.check(
        "hinged door swings outward on its revolute pivot",
        closed_door_box is not None
        and opened_door_box is not None
        and (
            opened_door_box[1][0] > closed_door_box[1][0] + 0.015
            or opened_door_box[0][0] < closed_door_box[0][0] - 0.015
            or opened_door_box[1][1] > closed_door_box[1][1] + 0.015
            or opened_door_box[0][1] < closed_door_box[0][1] - 0.015
        ),
        details=f"closed={closed_door_box}, opened={opened_door_box}",
    )

    # Latch knob is present on the door free edge at cage mid-height.
    latch_box = ctx.part_element_world_aabb(door, elem="latch_knob")
    ctx.check(
        "latch knob sits on the door free edge",
        latch_box is not None
        and latch_box[0][2] > 0.06
        and latch_box[1][2] < 0.18,
        details=f"latch_box={latch_box}",
    )

    for i in range(3):
        leg = object_model.get_part(f"leg_{i}")
        joint = object_model.get_articulation(f"body_to_leg_{i}")
        ctx.allow_overlap(
            body,
            leg,
            elem_a=f"leg_socket_{i}",
            elem_b="hinge_pin",
            reason="The slim leg hinge pin is intentionally captured inside the socket block at the lantern base.",
        )
        ctx.expect_contact(
            leg,
            body,
            elem_a="hinge_pin",
            elem_b=f"leg_socket_{i}",
            contact_tol=0.003,
            name=f"leg_{i} hinge sits in its socket",
        )
        leg_box = ctx.part_element_world_aabb(leg, elem="leg_wire")
        base_box = ctx.part_element_world_aabb(body, elem="vented_base")
        ctx.check(
            f"leg_{i} extends below the lantern base",
            leg_box is not None and base_box is not None and leg_box[0][2] < base_box[0][2] - 0.10,
            details=f"leg_box={leg_box}, base_box={base_box}",
        )

        rest_leg_box = ctx.part_element_world_aabb(leg, elem="leg_wire")
        with ctx.pose({joint: 1.0}):
            folded_leg_box = ctx.part_element_world_aabb(leg, elem="leg_wire")
            ctx.expect_contact(
                leg,
                body,
                elem_a="hinge_pin",
                elem_b=f"leg_socket_{i}",
                contact_tol=0.003,
                name=f"leg_{i} remains hinged while folding",
            )
        ctx.check(
            f"leg_{i} folds upward about its base hinge",
            rest_leg_box is not None
            and folded_leg_box is not None
            and folded_leg_box[0][2] > rest_leg_box[0][2] + 0.060,
            details=f"rest={rest_leg_box}, folded={folded_leg_box}",
        )

    return ctx.report()


object_model = build_object_model()
