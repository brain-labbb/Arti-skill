from __future__ import annotations

# Blue plastic barrel keg with a round black screw-on lid and an integrated
# scalloped-lobed body.  The earlier side bail handles and the later stick-on
# vertical strips were removed.  This variant changes the body itself: the mesh
# surface has smooth vertical lobes, shallow concave channels, and fused
# horizontal reinforcement belts generated as radius modulation.
#
# Frame: barrel axis along +Z, base sits on z=0, lid on top.
#   - body (root): one-piece blue barrel shell with closed bottom, hollow
#     interior, visible open mouth, integrated scalloped/lobed sides, rib belts,
#     and a threaded neck.
#   - lid: black round screw cap with knurled skirt and off-axis marker notch.
# Articulations:
#   - lid_rotate: CONTINUOUS, body -> lid_carrier  (spin about +Z)
#   - lid_slide : PRISMATIC,  lid_carrier -> lid   (lift along +Z)

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)

# ---- key dimensions (meters) -------------------------------------------------
BODY_BOTTOM_Z = 0.0
BARREL_TOP_Z = 0.255
NECK_TOP_Z = 0.270
RIM_TOP_Z = NECK_TOP_Z
BOTTOM_FLOOR_Z = 0.012
MOUTH_RADIUS = 0.055
NECK_RADIUS = 0.070
BARREL_MAX_R = 0.124
LID_HEIGHT = 0.030
LID_REST_CLEARANCE = 0.040
LID_OUTER_R = 0.082
LOBE_COUNT = 10
SEGMENTS = 96


def _smoothstep(edge0: float, edge1: float, x: float) -> float:
    if edge0 == edge1:
        return 0.0
    t = max(0.0, min(1.0, (x - edge0) / (edge1 - edge0)))
    return t * t * (3.0 - 2.0 * t)


def _profile_radius(z: float) -> float:
    # A soft keg profile: narrow foot, deep belly, sloped shoulder, neck.
    if z <= 0.030:
        return 0.084 + (0.102 - 0.084) * _smoothstep(0.000, 0.030, z)
    if z <= 0.115:
        return 0.102 + (0.116 - 0.102) * _smoothstep(0.030, 0.115, z)
    if z <= 0.165:
        return 0.116 + (0.118 - 0.116) * _smoothstep(0.115, 0.165, z)
    if z <= 0.235:
        return 0.118 + (0.087 - 0.118) * _smoothstep(0.165, 0.235, z)
    if z <= BARREL_TOP_Z:
        return 0.087 + (0.070 - 0.087) * _smoothstep(0.235, BARREL_TOP_Z, z)
    return NECK_RADIUS


def _rib_bump(z: float) -> float:
    # Fused horizontal belts.  These are not separate torus parts; they are
    # integrated into the body surface by widening the generated radius.
    specs = [
        (0.045, 0.006, 0.010),
        (0.080, 0.007, 0.009),
        (0.118, 0.008, 0.010),
        (0.155, 0.008, 0.010),
        (0.192, 0.007, 0.009),
        (0.224, 0.006, 0.008),
    ]
    bump = 0.0
    for center, amp, sigma in specs:
        bump += amp * math.exp(-((z - center) / sigma) ** 2)
    return bump


def _lobe_amp(z: float) -> float:
    # Lobes are strongest on the belly and fade out before the neck and foot.
    envelope = _smoothstep(0.035, 0.080, z) * (1.0 - _smoothstep(0.218, 0.252, z))
    return 0.010 * envelope


def _outer_radius(z: float, theta: float) -> float:
    # Positive cosine creates rounded lobes; negative regions become shallow
    # concave channels.  The second harmonic softens the channel floor.
    lobes = math.cos(LOBE_COUNT * theta)
    scallop = 0.70 * lobes + 0.30 * math.cos(2.0 * LOBE_COUNT * theta)
    return _profile_radius(z) + _rib_bump(z) + _lobe_amp(z) * scallop


def _inner_radius(z: float) -> float:
    if z >= BARREL_TOP_Z:
        return MOUTH_RADIUS
    return max(MOUTH_RADIUS, _profile_radius(z) - 0.020)


def _barrel_body() -> MeshGeometry:
    # One-piece open-top hollow mesh.  The outer surface is non-axisymmetric,
    # so the silhouette itself changes instead of relying on attached sticks.
    geom = MeshGeometry()
    outer_z = [
        0.000,
        0.012,
        0.025,
        0.045,
        0.065,
        0.085,
        0.105,
        0.125,
        0.145,
        0.165,
        0.185,
        0.205,
        0.225,
        0.242,
        0.255,
        0.270,
    ]

    outer: list[list[int]] = []
    for z in outer_z:
        ring = []
        for i in range(SEGMENTS):
            theta = 2.0 * math.pi * i / SEGMENTS
            r = _outer_radius(z, theta)
            ring.append(geom.add_vertex(r * math.cos(theta), r * math.sin(theta), z))
        outer.append(ring)

    for rings in (outer,):
        for a, b in zip(rings, rings[1:]):
            for i in range(SEGMENTS):
                j = (i + 1) % SEGMENTS
                geom.add_face(a[i], a[j], b[j])
                geom.add_face(a[i], b[j], b[i])

    bottom_center = geom.add_vertex(0.0, 0.0, BODY_BOTTOM_Z)
    for i in range(SEGMENTS):
        j = (i + 1) % SEGMENTS
        geom.add_face(bottom_center, outer[0][j], outer[0][i])

    inner_z = [BOTTOM_FLOOR_Z, 0.045, 0.085, 0.125, 0.165, 0.205, BARREL_TOP_Z, NECK_TOP_Z]
    inner: list[list[int]] = []
    for z in inner_z:
        ring = []
        for i in range(SEGMENTS):
            theta = 2.0 * math.pi * i / SEGMENTS
            r = _inner_radius(z)
            ring.append(geom.add_vertex(r * math.cos(theta), r * math.sin(theta), z))
        inner.append(ring)

    # Inner wall faces, reversed so normals point into the hollow.
    for a, b in zip(inner, inner[1:]):
        for i in range(SEGMENTS):
            j = (i + 1) % SEGMENTS
            geom.add_face(a[i], b[j], a[j])
            geom.add_face(a[i], b[i], b[j])

    # Thick bottom floor.
    floor_center = geom.add_vertex(0.0, 0.0, BOTTOM_FLOOR_Z)
    for i in range(SEGMENTS):
        j = (i + 1) % SEGMENTS
        geom.add_face(floor_center, inner[0][i], inner[0][j])

    # Open top annulus connecting outer neck to visible mouth.
    outer_top = outer[-1]
    inner_top = inner[-1]
    for i in range(SEGMENTS):
        j = (i + 1) % SEGMENTS
        geom.add_face(outer_top[i], inner_top[j], outer_top[j])
        geom.add_face(outer_top[i], inner_top[i], inner_top[j])

    return geom


def _body_mesh():
    body = _barrel_body()
    # Thread detail belongs to the neck/cap interface, not the rejected body
    # variation; keep it as a small integrated visual cue near the mouth.
    for tz in (0.252, 0.262, 0.272):
        thread = TorusGeometry(NECK_RADIUS - 0.002, 0.004, radial_segments=10, tubular_segments=48)
        thread.translate(0.0, 0.0, tz)
        body.merge(thread)
    return mesh_from_geometry(body, "barrel_shell")


def _lid_geometry():
    cap = CylinderGeometry(LID_OUTER_R, LID_HEIGHT, radial_segments=48)
    cap.translate(0.0, 0.0, LID_HEIGHT / 2.0)
    # Knurl ridges around the skirt.
    n = 28
    for k in range(n):
        a = 2.0 * math.pi * k / n
        ridge = CylinderGeometry(0.0035, LID_HEIGHT * 0.85, radial_segments=6)
        ridge.translate(LID_OUTER_R, 0.0, LID_HEIGHT / 2.0)
        ridge.rotate_z(a)
        cap.merge(ridge)
    rim = TorusGeometry(LID_OUTER_R - 0.006, 0.006, radial_segments=10, tubular_segments=48)
    rim.translate(0.0, 0.0, LID_HEIGHT)
    cap.merge(rim)
    return cap


def _lid_mesh():
    cap = _lid_geometry()
    return mesh_from_geometry(cap, "lid_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="blue_barrel_keg_scalloped_lobed_body")

    blue = model.material("barrel_blue", rgba=(0.14, 0.32, 0.74, 1.0))
    black = model.material("cap_black", rgba=(0.10, 0.10, 0.11, 1.0))

    body = model.part("body")
    body.visual(_body_mesh(), material=blue, name="barrel_shell")
    body.inertial = Inertial.from_geometry(
        Cylinder(0.12, 0.27), mass=2.2, origin=Origin(xyz=(0.0, 0.0, 0.13))
    )

    carrier = model.part("lid_carrier")  # NO visuals
    carrier.inertial = Inertial.from_geometry(Box((0.01, 0.01, 0.01)), mass=1e-4)

    lid = model.part("lid")
    lid.visual(_lid_mesh(), material=black, name="lid_shell")
    lid.visual(
        Box((0.012, 0.018, 0.012)),
        origin=Origin(xyz=(LID_OUTER_R - 0.010, 0.0, LID_HEIGHT + 0.004)),
        material=black,
        name="lid_molded_key",
    )
    lid.inertial = Inertial.from_geometry(
        Cylinder(LID_OUTER_R, LID_HEIGHT),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, LID_HEIGHT / 2.0)),
    )

    model.articulation(
        "lid_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, RIM_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )
    model.articulation(
        "lid_slide",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, LID_REST_CLEARANCE)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-LID_REST_CLEARANCE,
            upper=LID_HEIGHT,
            effort=1.0,
            velocity=1.0,
        ),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    rotate = object_model.get_articulation("lid_rotate")
    slide = object_model.get_articulation("lid_slide")

    ctx.allow_overlap(
        lid,
        body,
        elem_a="lid_shell",
        elem_b="barrel_shell",
        reason="The screw-cap skirt is intentionally seated over the barrel neck/threads when lowered.",
    )
    ctx.allow_isolated_part(
        lid,
        reason="Default display pose lifts the solid screw cap so the open barrel mouth is visible.",
    )

    lid_pos = ctx.part_world_position(lid)
    ctx.check(
        "lid sits on top of the barrel",
        lid_pos is not None and lid_pos[2] >= BARREL_TOP_Z - 0.02,
        details=f"lid origin={lid_pos}",
    )
    ctx.check(
        "default lid is raised clear of the open mouth",
        lid_pos is not None and lid_pos[2] >= NECK_TOP_Z + LID_REST_CLEARANCE - 0.002,
        details=f"lid origin={lid_pos}, neck_top={NECK_TOP_Z}",
    )

    with ctx.pose({slide: -LID_REST_CLEARANCE}):
        ctx.expect_overlap(
            lid, body, axes="z", min_overlap=0.004, name="cap seated over neck when lowered"
        )
    rest_z = ctx.part_world_position(lid)[2]
    with ctx.pose({slide: LID_HEIGHT}):
        lifted_z = ctx.part_world_position(lid)[2]
    ctx.check(
        "sliding lid_slide lifts the lid up off the neck",
        lifted_z > rest_z + 0.02,
        details=f"rest_z={rest_z}, lifted_z={lifted_z}",
    )

    def _center(aabb):
        mn, mx = aabb
        return ((mn[0] + mx[0]) / 2.0, (mn[1] + mx[1]) / 2.0)

    rest_c = _center(ctx.part_element_world_aabb(lid, elem="lid_molded_key"))
    with ctx.pose({rotate: math.pi / 2.0}):
        spun_c = _center(ctx.part_element_world_aabb(lid, elem="lid_molded_key"))
    moved = abs(spun_c[0] - rest_c[0]) + abs(spun_c[1] - rest_c[1])
    ctx.check(
        "rotating lid_rotate spins the lid (off-axis molded key moves)",
        moved > 0.005,
        details=f"rest_c={rest_c}, spun_c={spun_c}, moved={moved}",
    )

    body_ext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "barrel body is deeply bulged and visibly wider than neck",
        body_ext[0] > 0.23 and body_ext[1] > 0.23,
        details=f"body extents={body_ext}",
    )

    body_geom = _barrel_body()
    radii_z = [(math.hypot(x, y), z, math.atan2(y, x)) for x, y, z in body_geom.vertices]
    bottom_has_center = any(
        r < 0.003 and abs(z - BODY_BOTTOM_Z) < 1e-6 for r, z, _ in radii_z
    )
    ctx.check(
        "barrel bottom has a center face",
        bottom_has_center,
        details="bottom includes vertices on the center axis at z=0",
    )
    belly = [r for r, z, _ in radii_z if 0.110 <= z <= 0.135]
    shoulder = [r for r, z, _ in radii_z if 0.235 <= z <= 0.255]
    ctx.check(
        "body profile has exaggerated belly-to-shoulder contrast",
        bool(belly)
        and bool(shoulder)
        and max(belly) > max(shoulder) + 0.025,
        details=(
            f"belly={max(belly) if belly else None}, "
            f"shoulder={max(shoulder) if shoulder else None}"
        ),
    )
    mouth_band = [r for r, z, _ in radii_z if abs(z - NECK_TOP_Z) < 1e-6]
    ctx.check(
        "barrel mouth is a visible opening",
        bool(mouth_band) and min(mouth_band) >= MOUTH_RADIUS - 0.001,
        details=f"mouth radii min/max={((min(mouth_band), max(mouth_band)) if mouth_band else None)}",
    )

    sample_z = 0.125
    sampled = [
        _outer_radius(sample_z, 2.0 * math.pi * i / (LOBE_COUNT * 2))
        for i in range(LOBE_COUNT * 2)
    ]
    ctx.check(
        "body lobes are integrated into the generated shell, not separate rods",
        max(sampled) - min(sampled) > 0.010,
        details=f"sampled radius spread={max(sampled) - min(sampled)}",
    )

    lid_geom = _lid_geometry()
    lid_top_radii = [
        math.hypot(x, y) for x, y, z in lid_geom.vertices if abs(z - LID_HEIGHT) < 1e-6
    ]
    ctx.check(
        "screw cap has a closed center",
        bool(lid_top_radii) and min(lid_top_radii) <= 0.003,
        details=(
            "lid top radii min/max="
            f"{((min(lid_top_radii), max(lid_top_radii)) if lid_top_radii else None)}"
        ),
    )

    return ctx.report()
