from __future__ import annotations

# Blue Yeti-style USB condenser microphone with four front control knobs.
#
# Coordinate convention:
#   - up is +Z (the round weighted base sits on the ground at z=0)
#   - the body "front" face (badge / MUTE / control column) faces +X
#   - the Y-fork tilt axis runs along +Y (the two fork arms straddle the body
#     left/right along +/-Y and pivot the body forward/back about +Y)
#
# Structure:
#   - base (root, static): round weighted base + Y-shaped fork stand (two curved
#     arms rising from a central hub to the pivot bosses that capture the body).
#   - body (REVOLUTE about +Y): tall tapered cylinder with a fine vertical
#     mesh-grille band near the top, a rounded dome capsule cap, front badge,
#     MUTE dot, and a vertical column of four front-facing rotary control knobs.
#   - front_knob_0..3 (each CONTINUOUS about +X): four evenly spaced knurled
#     rotary knobs on the front face, each with an off-center pointer marker.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    DomeGeometry,
    Inertial,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    Sphere,
    SphereGeometry,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    sample_catmull_rom_spline_2d,
    tube_from_spline_points,
)

# --- key dimensions (meters) ---
BASE_DIA = 0.110
BASE_R = BASE_DIA / 2.0
BASE_H = 0.018

BODY_DIA = 0.060
BODY_R = BODY_DIA / 2.0
BODY_BOTTOM_Z = 0.060  # lowest body point above the base, in rest pose
BODY_TUBE_H = 0.160  # straight cylindrical section height (taller for 4 knobs)
GRILLE_BOTTOM_Z = BODY_BOTTOM_Z + 0.115  # where the mesh band starts
GRILLE_TOP_Z = BODY_BOTTOM_Z + BODY_TUBE_H  # body tube top
CAP_R = BODY_R  # dome capsule radius

PIVOT_Z = 0.118  # height of the fork tilt axis (between the arms)
PIVOT_Y = BODY_R + 0.012  # arm boss sits just outboard of the body wall
BODY_DZ = -PIVOT_Z  # shift from ground coords into the body part frame

# --- front control column (4 knobs) ---
KNOB_COUNT = 4
KNOB_DIA = 0.018
KNOB_R = KNOB_DIA / 2.0
KNOB_H = 0.010
# Evenly spaced Z centers for the four front knobs (below mute dot, above body bottom).
KNOB_ZS = [0.070, 0.088, 0.106, 0.124]

# Badge and mute positions (above the knob column, below the grille band).
BADGE_Z = GRILLE_BOTTOM_Z - 0.010  # ~0.165
MUTE_Z = BADGE_Z - 0.024  # ~0.141


def _mesh_grille_band(material_name: str):
    """Perforated mesh-grille band: a ribbed cylinder made of fine vertical
    ridges, reading as the windscreen/grille near the top of the capsule."""
    z0 = GRILLE_BOTTOM_Z
    z1 = GRILLE_TOP_Z
    h = z1 - z0
    geom = CylinderGeometry(BODY_R - 0.0015, h, radial_segments=48)
    geom.translate(0.0, 0.0, z0 + h / 2.0)
    # Fine vertical ribs around the circumference to read as a mesh grille.
    n_ribs = 40
    rib = BODY_R - 0.0008
    for i in range(n_ribs):
        ang = 2.0 * math.pi * i / n_ribs
        ridge = CylinderGeometry(0.0010, h, radial_segments=6)
        ridge.translate(rib * math.cos(ang), rib * math.sin(ang), z0 + h / 2.0)
        geom.merge(ridge)
    # Two retaining bands top and bottom of the grille.
    for zb in (z0 + 0.002, z1 - 0.002):
        ring = TorusGeometry(BODY_R - 0.0006, 0.0018, radial_segments=10, tubular_segments=48)
        ring.translate(0.0, 0.0, zb)
        geom.merge(ring)
    return mesh_from_geometry(geom, material_name)


def _body_shell():
    """Lower body: straight cylindrical housing below the grille band, with a
    small bottom chamfer ring so it reads as a machined capsule body."""
    z0 = BODY_BOTTOM_Z
    z1 = GRILLE_BOTTOM_Z
    h = z1 - z0
    geom = CylinderGeometry(BODY_R, h, radial_segments=48)
    geom.translate(0.0, 0.0, z0 + h / 2.0)
    # Bottom rounded lip.
    lip = TorusGeometry(BODY_R - 0.0035, 0.0035, radial_segments=12, tubular_segments=48)
    lip.translate(0.0, 0.0, z0 + 0.0035)
    geom.merge(lip)
    return mesh_from_geometry(geom, "body_shell")


def _dome_cap():
    """Rounded mesh-grille top capsule: a dome closing off the top of the body."""
    geom = DomeGeometry(CAP_R, radial_segments=48, height_segments=18)
    geom.scale(1.0, 1.0, 0.95)
    geom.translate(0.0, 0.0, GRILLE_TOP_Z - 0.001)
    return mesh_from_geometry(geom, "dome_cap")


def _pivot_boss(sign: float):
    """A cylindrical pivot trunnion on the +/-Y side of the body. It reaches from
    the body wall outward through the fork-arm knuckle so the body is physically
    captured by the fork (intentional boss-in-knuckle overlap)."""
    inner = BODY_R - 0.004
    outer = PIVOT_Y + 0.006
    length = outer - inner
    mid = (outer + inner) / 2.0
    geom = CylinderGeometry(0.009, length, radial_segments=24).rotate_x(math.pi / 2.0)
    geom.translate(0.0, sign * mid, PIVOT_Z)
    return geom


def _fork_arm_mesh(sign: float, name: str):
    """One curved Y-fork arm: rises from the central hub, sweeps outward and up,
    and curls in to the pivot boss that captures the body. Built as a swept
    tube along a Catmull-Rom spline in the YZ plane (mirrored by `sign`)."""
    y = sign
    pts2d = [
        (y * 0.010, BASE_H + 0.004),
        (y * 0.030, BASE_H + 0.030),
        (y * 0.052, 0.072),
        (y * 0.050, 0.104),
        (y * (PIVOT_Y), PIVOT_Z),
    ]
    smooth = sample_catmull_rom_spline_2d(pts2d, samples_per_segment=10)
    pts3d = [(0.0, p[0], p[1]) for p in smooth]
    tube = tube_from_spline_points(pts3d, radius=0.0085, samples_per_segment=4, radial_segments=14)
    # Pivot knuckle at the top of the arm where it grips the body boss.
    knuckle = CylinderGeometry(0.013, 0.012, radial_segments=24).rotate_x(math.pi / 2.0)
    knuckle.translate(0.0, y * PIVOT_Y, PIVOT_Z)
    tube.merge(knuckle)
    return mesh_from_geometry(tube, name)


def _base_disc():
    """Round weighted base disc with a slightly domed top and a hub stub that
    the two fork arms grow out of."""
    geom = CylinderGeometry(BASE_R, BASE_H, radial_segments=64)
    geom.translate(0.0, 0.0, BASE_H / 2.0)
    # Top edge fillet ring.
    rim = TorusGeometry(BASE_R - 0.004, 0.004, radial_segments=12, tubular_segments=64)
    rim.translate(0.0, 0.0, BASE_H - 0.003)
    geom.merge(rim)
    # Central hub stub joining the two arms to the base.
    hub = CylinderGeometry(0.020, 0.016, radial_segments=32)
    hub.translate(0.0, 0.0, BASE_H + 0.004)
    geom.merge(hub)
    return mesh_from_geometry(geom, "base_disc")


def _front_knob_mesh(name: str):
    """Shared knurled rotary knob geometry for the front control column.
    Built along local +Z with mount face at z=0 (center=False)."""
    geom = KnobGeometry(
        KNOB_DIA,
        KNOB_H,
        body_style="cylindrical",
        grip=KnobGrip(style="knurled", count=24, depth=0.0008),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007, angle_deg=0.0),
        center=False,
    )
    return mesh_from_geometry(geom, name)


def _front_knob_marker(name: str):
    """Off-center pointer marker for a front knob so spin is legible.
    Placed in the local frame (before rotation to face +X): offset along
    local +X so it sweeps in YZ when the knob rotates about world +X.
    The box protrates through the knob top face (z > KNOB_H) so the marker
    mesh is physically connected to the knob body mesh."""
    geom = BoxGeometry((0.004, 0.002, 0.006))
    geom.translate(KNOB_R - 0.003, 0.0, KNOB_H + 0.001)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="usb_condenser_microphone")

    blue = model.material("body_blue", rgba=(0.30, 0.36, 0.95, 1.0))
    blue_dark = model.material("blue_dark", rgba=(0.20, 0.25, 0.78, 1.0))
    blue_mesh = model.material("blue_mesh", rgba=(0.40, 0.46, 0.98, 1.0))
    blue_base = model.material("blue_base", rgba=(0.34, 0.40, 0.97, 1.0))
    badge_blue = model.material("badge_blue", rgba=(0.86, 0.90, 1.0, 1.0))
    mute_red = model.material("mute_red", rgba=(0.85, 0.20, 0.30, 1.0))
    knob_dark = model.material("knob_dark", rgba=(0.15, 0.17, 0.22, 1.0))
    marker_silver = model.material("marker_silver", rgba=(0.82, 0.84, 0.88, 1.0))

    # ---- base + Y-fork stand (root, static) ----
    base = model.part("base")
    base.visual(_base_disc(), material=blue_base, name="base_disc")
    base.visual(_fork_arm_mesh(+1.0, "fork_arm_pos"), material=blue, name="fork_arm_pos")
    base.visual(_fork_arm_mesh(-1.0, "fork_arm_neg"), material=blue, name="fork_arm_neg")
    base.inertial = Inertial.from_geometry(
        Cylinder(BASE_R, BASE_H),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, BASE_H / 2.0)),
    )

    # ---- mic body: tilts about the +Y fork pivot axis ----
    body_off = Origin(xyz=(0.0, 0.0, BODY_DZ))
    body = model.part("body")
    body.visual(_body_shell(), origin=body_off, material=blue, name="body_shell")
    body.visual(_mesh_grille_band("grille_band"), origin=body_off, material=blue_mesh, name="grille_band")
    body.visual(_dome_cap(), origin=body_off, material=blue_mesh, name="dome_cap")
    # Pivot bosses captured by the fork knuckles.
    body.visual(
        mesh_from_geometry(_pivot_boss(+1.0), "pivot_boss_pos"),
        origin=body_off,
        material=blue_dark,
        name="pivot_boss_pos",
    )
    body.visual(
        mesh_from_geometry(_pivot_boss(-1.0), "pivot_boss_neg"),
        origin=body_off,
        material=blue_dark,
        name="pivot_boss_neg",
    )
    # Front "Blue" badge (oval plate on the +X face, between knob column and grille).
    badge = SphereGeometry(0.011, width_segments=24, height_segments=12)
    badge.scale(0.35, 1.7, 0.8)
    badge.translate(BODY_R - 0.001, 0.0, BADGE_Z)
    body.visual(mesh_from_geometry(badge, "front_badge"), origin=body_off, material=badge_blue, name="front_badge")
    # MUTE indicator dot below the badge on the front face.
    mute = SphereGeometry(0.005, width_segments=16, height_segments=10)
    mute.scale(0.35, 1.0, 1.0)
    mute.translate(BODY_R - 0.0005, 0.0, MUTE_Z)
    body.visual(mesh_from_geometry(mute, "mute_dot"), origin=body_off, material=mute_red, name="mute_dot")
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, BODY_TUBE_H),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, BODY_BOTTOM_Z + BODY_TUBE_H / 2.0 + BODY_DZ)),
    )

    model.articulation(
        "base_to_body",
        ArticulationType.REVOLUTE,
        parent=base,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-0.785, upper=0.785),
    )

    # ---- front control column: 4 knurled rotary knobs, evenly spaced ----
    # Each knob is built along local +Z (mount face at z=0), then rotated so its
    # axis points along world +X (front-facing). The CONTINUOUS joint spins
    # about +X; the off-center marker sweeps in YZ for legible rotation.
    knob_rpy = (0.0, math.pi / 2.0, 0.0)

    for i in range(KNOB_COUNT):
        knob_name = f"front_knob_{i}"
        marker_name = f"front_marker_{i}"
        z_i = KNOB_ZS[i]

        knob_part = model.part(knob_name)
        knob_part.visual(
            _front_knob_mesh(knob_name),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=knob_rpy),
            material=knob_dark,
            name=knob_name,
        )
        knob_part.visual(
            _front_knob_marker(marker_name),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=knob_rpy),
            material=marker_silver,
            name=marker_name,
        )
        knob_part.inertial = Inertial.from_geometry(Sphere(KNOB_R), mass=0.008)

        model.articulation(
            f"body_to_{knob_name}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob_part,
            origin=Origin(xyz=(BODY_R - 0.001, 0.0, z_i + BODY_DZ)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.3, velocity=8.0),
        )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def _aabb_center(aabb):
    mn, mx = aabb
    return ((mn[0] + mx[0]) / 2.0, (mn[1] + mx[1]) / 2.0, (mn[2] + mx[2]) / 2.0)


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    body = object_model.get_part("body")
    tilt = object_model.get_articulation("base_to_body")

    # Collect all four front knobs and their joints.
    knobs = [object_model.get_part(f"front_knob_{i}") for i in range(KNOB_COUNT)]
    knob_joints = [object_model.get_articulation(f"body_to_front_knob_{i}") for i in range(KNOB_COUNT)]

    # --- intentional overlaps: pivot bosses captured in fork knuckles, knobs
    # seated against the body front wall. ---
    ctx.allow_overlap(
        body,
        base,
        elem_a="pivot_boss_pos",
        elem_b="fork_arm_pos",
        reason="Body pivot boss is captured inside the +Y fork-arm knuckle.",
    )
    ctx.allow_overlap(
        body,
        base,
        elem_a="pivot_boss_neg",
        elem_b="fork_arm_neg",
        reason="Body pivot boss is captured inside the -Y fork-arm knuckle.",
    )
    for i in range(KNOB_COUNT):
        ctx.allow_overlap(
            knobs[i],
            body,
            elem_a=f"front_knob_{i}",
            elem_b="body_shell",
            reason=f"Front knob {i} base is seated against the body front wall.",
        )

    # --- body sits between the two fork arms (straddled along +/-Y) ---
    body_aabb = ctx.part_world_aabb(body)
    arm_pos_aabb = ctx.part_element_world_aabb(base, elem="fork_arm_pos")
    arm_neg_aabb = ctx.part_element_world_aabb(base, elem="fork_arm_neg")
    ctx.check(
        "body straddled between the two fork arms along Y",
        arm_neg_aabb[0][1] < body_aabb[0][1] and arm_pos_aabb[1][1] > body_aabb[1][1],
        details=f"arm_neg_minY={arm_neg_aabb[0][1]}, body_minY={body_aabb[0][1]}, "
        f"body_maxY={body_aabb[1][1]}, arm_pos_maxY={arm_pos_aabb[1][1]}",
    )

    # --- base is the widest footprint and rests on the ground ---
    base_aabb = ctx.part_world_aabb(base)
    base_disc_aabb = ctx.part_element_world_aabb(base, elem="base_disc")
    base_ext = _ext(base_disc_aabb)
    body_shell_ext = _ext(ctx.part_element_world_aabb(body, elem="body_shell"))
    ctx.check(
        "round base is the widest footprint",
        base_ext[0] > body_shell_ext[0] + 0.02 and base_ext[1] > body_shell_ext[1] + 0.02,
        details=f"base_xy=({base_ext[0]},{base_ext[1]}), "
        f"body_shell_xy=({body_shell_ext[0]},{body_shell_ext[1]})",
    )
    ctx.check(
        "base rests on the ground at z~0",
        abs(base_aabb[0][2]) < 0.002,
        details=f"base_minZ={base_aabb[0][2]}",
    )

    # --- body tilts about the +Y fork axis ---
    cap_rest = _aabb_center(ctx.part_element_world_aabb(body, elem="dome_cap"))
    with ctx.pose({tilt: math.radians(30.0)}):
        cap_tilted = _aabb_center(ctx.part_element_world_aabb(body, elem="dome_cap"))
    ctx.check(
        "body tilts forward about the +Y fork axis: top swings toward +X",
        cap_tilted[0] > cap_rest[0] + 0.02 and cap_tilted[2] < cap_rest[2] - 0.005,
        details=f"rest cap center={cap_rest}, tilted cap center={cap_tilted}",
    )

    # --- all four front knobs are on the +X front face ---
    for i in range(KNOB_COUNT):
        pos = ctx.part_world_position(knobs[i])
        ctx.check(
            f"front_knob_{i} mounted on the +X front face",
            pos[0] > 0.02,
            details=f"front_knob_{i} origin={pos}",
        )

    # --- knobs are evenly spaced vertically (Z axis) ---
    knob_zs = [ctx.part_world_position(knobs[i])[2] for i in range(KNOB_COUNT)]
    spacings = [knob_zs[i + 1] - knob_zs[i] for i in range(KNOB_COUNT - 1)]
    avg_spacing = sum(spacings) / len(spacings)
    max_deviation = max(abs(s - avg_spacing) for s in spacings)
    ctx.check(
        "four front knobs are evenly spaced vertically",
        max_deviation < 0.003,
        details=f"z_positions={knob_zs}, spacings={spacings}, max_dev={max_deviation:.4f}",
    )

    # --- each knob spins about +X: off-center marker moves in YZ ---
    for i in range(KNOB_COUNT):
        marker_name = f"front_marker_{i}"
        marker_rest = _aabb_center(ctx.part_element_world_aabb(knobs[i], elem=marker_name))
        with ctx.pose({knob_joints[i]: math.pi / 2.0}):
            marker_spun = _aabb_center(ctx.part_element_world_aabb(knobs[i], elem=marker_name))
        ctx.check(
            f"front_knob_{i} spins about the front (X) axis",
            abs(marker_spun[1] - marker_rest[1]) > 0.003
            or abs(marker_spun[2] - marker_rest[2]) > 0.003,
            details=f"rest={marker_rest}, spun={marker_spun}",
        )

    # --- all four knobs exist (count check) ---
    all_parts = [p.name for p in object_model.parts]
    knob_names = [f"front_knob_{i}" for i in range(KNOB_COUNT)]
    ctx.check(
        "exactly four front control knobs present",
        all(n in all_parts for n in knob_names),
        details=f"expected={knob_names}, found_parts={all_parts}",
    )

    return ctx.report()


object_model = build_object_model()
