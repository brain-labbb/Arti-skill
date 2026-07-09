from __future__ import annotations

# Blue Yeti-style USB condenser microphone with a round ball mesh grille head.
#
# Coordinate convention:
#   - up is +Z (the round weighted base sits on the ground at z=0)
#   - the body "front" face (badge / MUTE / VOLUME knob) faces +X
#   - the Y-fork tilt axis runs along +Y (the two fork arms straddle the body
#     left/right along +/-Y and pivot the body forward/back about +Y)
#
# Structure:
#   - base (root, static): round weighted base + Y-shaped fork stand (two curved
#     arms rising from a central hub to the pivot bosses that capture the body).
#   - body (REVOLUTE about +Y): tapered cylindrical body with a spherical ball
#     mesh grille head seated on top (meridian + parallel ribs over a dark inner
#     capsule), front badge, MUTE dot, and the seated control knobs.
#   - volume_knob (CONTINUOUS about +X): front-facing rotary VOLUME knob.
#   - gain_knob (CONTINUOUS about +Y): side-facing gain/pattern knob.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    LatheGeometry,
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
BODY_TOP_R = 0.022  # tapered body top / neck radius (44 mm dia)
BODY_BOTTOM_Z = 0.060
BODY_H = 0.090  # body height to the main shoulder
BODY_TOP_Z = BODY_BOTTOM_Z + BODY_H  # = 0.150 (shoulder)
NECK_TOP_Z = BODY_TOP_Z + 0.010  # neck extends 10 mm above shoulder into sphere

# Spherical ball grille head
HEAD_R = 0.040  # outer rib sphere radius (80 mm diameter grille ball)
HEAD_INNER_R = HEAD_R - 0.001  # dark inner capsule (78 mm dia, ribs penetrate)
HEAD_CENTER_Z = NECK_TOP_Z + HEAD_R * 0.60  # sphere seated on the neck

PIVOT_Z = 0.118  # height of the fork tilt axis (between the arms)
PIVOT_Y = BODY_R + 0.012  # arm boss sits just outboard of the body wall
BODY_DZ = -PIVOT_Z  # shift from ground coords into the body part frame


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _body_shell():
    """Tapered cylindrical body: wider at the bottom, narrowing toward the neck
    where the ball grille head seats. Built as a single lathed profile that
    closes at the bottom (no separate cap or lip) for one connected mesh."""
    profile = [
        (0.002, BODY_BOTTOM_Z),                       # near-center (closes bottom)
        (BODY_R - 0.003, BODY_BOTTOM_Z),              # bottom edge
        (BODY_R, BODY_BOTTOM_Z + 0.004),              # rounded bottom corner
        (BODY_R, BODY_BOTTOM_Z + 0.040),              # straight lower section
        (BODY_R - 0.001, BODY_BOTTOM_Z + 0.055),      # begin taper
        (BODY_TOP_R, BODY_TOP_Z),                     # shoulder
        (BODY_TOP_R, NECK_TOP_Z),                     # neck extension into sphere
    ]
    geom = LatheGeometry(profile, segments=48)
    return mesh_from_geometry(geom, "body_shell")


def _sphere_grille_inner():
    """Dark inner sphere: the capsule housing visible through the mesh grille.
    Positioned so the lower portion protrudes through the body top neck, creating
    a physical connection between the head and the body shell."""
    geom = SphereGeometry(HEAD_INNER_R, width_segments=32, height_segments=24)
    geom.translate(0.0, 0.0, HEAD_CENTER_Z)
    return mesh_from_geometry(geom, "head_inner")


def _sphere_grille_ribs():
    """Spherical mesh grille: meridian and parallel ribs forming a wire cage
    over the dark inner sphere, reading as an open spherical grille.
    Each rib is a closed spline tube built via the shared _rib_tube helper.
    Rib tube radius is chosen so the inner edge of each rib penetrates the
    inner sphere surface, ensuring mesh connectivity."""
    ribs = []

    # Meridian ribs (great circles in vertical planes).
    n_meridians = 12
    for i in range(n_meridians):
        theta = math.pi * i / n_meridians
        pts = _meridian_path(theta, n_samples=28)
        tube = _rib_tube(pts)
        ribs.append(tube)

    # Parallel ribs (horizontal circles at different latitudes).
    n_parallels = 8
    for i in range(n_parallels):
        lat = -math.radians(50) + math.radians(130) * (i + 1) / (n_parallels + 1)
        pts = _parallel_path(lat, n_samples=28)
        tube = _rib_tube(pts)
        ribs.append(tube)

    # Equator reinforcing ring (slightly thicker, reads as the main band).
    eq_pts = _parallel_path(0.0, n_samples=32)
    eq_tube = tube_from_spline_points(
        eq_pts, radius=0.0018, samples_per_segment=3,
        closed_spline=True, radial_segments=8, cap_ends=False,
    )
    ribs.append(eq_tube)

    # Merge all ribs into one mesh.
    geom = ribs[0]
    for r in ribs[1:]:
        geom.merge(r)
    return mesh_from_geometry(geom, "head_ribs")


def _rib_tube(pts):
    """Shared rib helper: build a thin closed spline tube from path points.
    Tube radius 0.0014 ensures the rib inner edge (HEAD_R - 0.0014 = 0.0386)
    penetrates the inner sphere surface (HEAD_INNER_R = 0.039) by ~0.4 mm."""
    return tube_from_spline_points(
        pts, radius=0.0014, samples_per_segment=3,
        closed_spline=True, radial_segments=6, cap_ends=False,
    )


def _meridian_path(theta, n_samples=28):
    """Generate points along a meridian great circle at longitude theta."""
    pts = []
    for j in range(n_samples):
        alpha = 2.0 * math.pi * j / n_samples
        x = HEAD_R * math.cos(alpha) * math.cos(theta)
        y = HEAD_R * math.cos(alpha) * math.sin(theta)
        z = HEAD_R * math.sin(alpha) + HEAD_CENTER_Z
        pts.append((x, y, z))
    return pts


def _parallel_path(lat, n_samples=28):
    """Generate points along a parallel circle at latitude lat."""
    r = HEAD_R * math.cos(lat)
    z = HEAD_R * math.sin(lat) + HEAD_CENTER_Z
    pts = []
    for j in range(n_samples):
        theta = 2.0 * math.pi * j / n_samples
        pts.append((r * math.cos(theta), r * math.sin(theta), z))
    return pts


def _head_collar():
    """Collar ring at the junction between body neck and spherical head.
    Sits at the neck top where the sphere protrudes through the body wall."""
    geom = TorusGeometry(BODY_TOP_R + 0.004, 0.004, radial_segments=10, tubular_segments=48)
    geom.translate(0.0, 0.0, NECK_TOP_Z)
    return mesh_from_geometry(geom, "head_collar")


def _pivot_boss(sign: float):
    """A cylindrical pivot trunnion on the +/-Y side of the body. It reaches from
    inside the body wall outward through the fork-arm knuckle so the body is
    physically captured by the fork (intentional boss-in-knuckle overlap)."""
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


# ---------------------------------------------------------------------------
# Model assembly
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="usb_condenser_microphone")

    blue = model.material("body_blue", rgba=(0.30, 0.36, 0.95, 1.0))
    blue_dark = model.material("blue_dark", rgba=(0.20, 0.25, 0.78, 1.0))
    blue_base = model.material("blue_base", rgba=(0.34, 0.40, 0.97, 1.0))
    badge_blue = model.material("badge_blue", rgba=(0.86, 0.90, 1.0, 1.0))
    mute_red = model.material("mute_red", rgba=(0.85, 0.20, 0.30, 1.0))
    head_dark = model.material("head_dark", rgba=(0.05, 0.05, 0.09, 1.0))
    grille_chrome = model.material("grille_chrome", rgba=(0.72, 0.76, 0.82, 1.0))
    collar_dark = model.material("collar_dark", rgba=(0.15, 0.18, 0.30, 1.0))

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
    # Body geometry is authored in ground coordinates; every body visual is
    # shifted down by PIVOT_Z (BODY_DZ) into the body part frame.
    body_off = Origin(xyz=(0.0, 0.0, BODY_DZ))
    body = model.part("body")

    # Tapered cylindrical body shell (single connected lathe mesh).
    body.visual(_body_shell(), origin=body_off, material=blue, name="body_shell")

    # Spherical ball grille head: dark inner capsule + chrome rib cage + collar.
    body.visual(_sphere_grille_inner(), origin=body_off, material=head_dark, name="head_inner")
    body.visual(_sphere_grille_ribs(), origin=body_off, material=grille_chrome, name="head_ribs")
    body.visual(_head_collar(), origin=body_off, material=collar_dark, name="head_collar")

    # Pivot bosses captured by the fork knuckles.
    body.visual(
        mesh_from_geometry(_pivot_boss(+1.0), "pivot_boss_pos"),
        origin=body_off, material=blue_dark, name="pivot_boss_pos",
    )
    body.visual(
        mesh_from_geometry(_pivot_boss(-1.0), "pivot_boss_neg"),
        origin=body_off, material=blue_dark, name="pivot_boss_neg",
    )

    # Front "Blue" badge (oval plate on the +X face, high on the body).
    badge_z = BODY_TOP_Z - 0.018
    badge = SphereGeometry(0.011, width_segments=24, height_segments=12)
    badge.scale(0.35, 1.7, 0.8)
    badge.translate(BODY_R - 0.001, 0.0, badge_z)
    body.visual(mesh_from_geometry(badge, "front_badge"), origin=body_off, material=badge_blue, name="front_badge")

    # MUTE indicator dot below the badge on the front face.
    mute_z = badge_z - 0.024
    mute = SphereGeometry(0.005, width_segments=16, height_segments=10)
    mute.scale(0.35, 1.0, 1.0)
    mute.translate(BODY_R - 0.0005, 0.0, mute_z)
    body.visual(mesh_from_geometry(mute, "mute_dot"), origin=body_off, material=mute_red, name="mute_dot")

    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, BODY_H + 2 * HEAD_R),
        mass=0.30,
        origin=Origin(xyz=(0.0, 0.0, BODY_BOTTOM_Z + (BODY_H + 2 * HEAD_R) / 2.0 + BODY_DZ)),
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

    # ---- front VOLUME knob: continuous spin about +X (front axis) ----
    vol_z = mute_z - 0.020
    vol_knob = KnobGeometry(
        0.030,
        0.012,
        body_style="cylindrical",
        grip=KnobGrip(style="knurled", count=28, depth=0.0009),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008, angle_deg=0.0),
        center=False,
    )
    vol_mesh = mesh_from_geometry(vol_knob, "volume_knob")
    volume = model.part("volume_knob")
    knob_rpy = (0.0, math.pi / 2.0, 0.0)
    volume.visual(
        vol_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=knob_rpy),
        material=blue_dark,
        name="volume_knob",
    )
    vol_marker = BoxGeometry((0.005, 0.0025, 0.006))
    vol_marker.translate(0.011, 0.0, 0.010)
    volume.visual(
        mesh_from_geometry(vol_marker, "volume_marker"),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=knob_rpy),
        material=badge_blue,
        name="volume_marker",
    )
    volume.inertial = Inertial.from_geometry(Sphere(0.015), mass=0.01)
    model.articulation(
        "body_to_volume_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=volume,
        origin=Origin(xyz=(BODY_R - 0.001, 0.0, vol_z + BODY_DZ)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=8.0),
    )

    # ---- side gain/pattern knob: continuous spin about +Y (side axis) ----
    gain_z = 0.092
    gain_knob = KnobGeometry(
        0.022,
        0.011,
        body_style="cylindrical",
        grip=KnobGrip(style="fluted", count=20, depth=0.0010),
        indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
        center=False,
    )
    gain_mesh = mesh_from_geometry(gain_knob, "gain_knob")
    gain = model.part("gain_knob")
    gain_rpy = (-math.pi / 2.0, 0.0, 0.0)
    gain.visual(
        gain_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=gain_rpy),
        material=blue_dark,
        name="gain_knob",
    )
    gain_marker = BoxGeometry((0.004, 0.002, 0.005))
    gain_marker.translate(0.008, 0.0, 0.009)
    gain.visual(
        mesh_from_geometry(gain_marker, "gain_marker"),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=gain_rpy),
        material=badge_blue,
        name="gain_marker",
    )
    gain.inertial = Inertial.from_geometry(Sphere(0.012), mass=0.008)
    model.articulation(
        "body_to_gain_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=gain,
        origin=Origin(xyz=(0.0, BODY_R - 0.001, gain_z + BODY_DZ)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=8.0),
    )

    return model


# ---------------------------------------------------------------------------
# Test helpers
# ---------------------------------------------------------------------------

def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def _aabb_center(aabb):
    mn, mx = aabb
    return ((mn[0] + mx[0]) / 2.0, (mn[1] + mx[1]) / 2.0, (mn[2] + mx[2]) / 2.0)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    body = object_model.get_part("body")
    volume = object_model.get_part("volume_knob")
    gain = object_model.get_part("gain_knob")
    tilt = object_model.get_articulation("base_to_body")
    vol_joint = object_model.get_articulation("body_to_volume_knob")
    gain_joint = object_model.get_articulation("body_to_gain_knob")

    # --- intentional overlaps ---
    ctx.allow_overlap(
        body, base,
        elem_a="pivot_boss_pos", elem_b="fork_arm_pos",
        reason="Body pivot boss is captured inside the +Y fork-arm knuckle.",
    )
    ctx.allow_overlap(
        body, base,
        elem_a="pivot_boss_neg", elem_b="fork_arm_neg",
        reason="Body pivot boss is captured inside the -Y fork-arm knuckle.",
    )
    ctx.allow_overlap(
        volume, body,
        elem_a="volume_knob", elem_b="body_shell",
        reason="Volume knob base is seated against the body front wall.",
    )
    ctx.allow_overlap(
        gain, body,
        elem_a="gain_knob", elem_b="body_shell",
        reason="Gain knob base is seated against the body side wall.",
    )
    ctx.allow_overlap(
        body, body,
        elem_a="head_inner", elem_b="body_shell",
        reason="Inner grille capsule protrudes through the body top neck to connect the head assembly.",
    )
    ctx.allow_overlap(
        body, body,
        elem_a="head_collar", elem_b="body_shell",
        reason="Head collar ring straddles the body top neck and inner sphere.",
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

    # --- spherical ball grille head exists and reads as round ---
    head_inner_aabb = ctx.part_element_world_aabb(body, elem="head_inner")
    head_inner_ext = _ext(head_inner_aabb)
    ctx.check(
        "spherical grille head is approximately round (similar XY and Z extents)",
        abs(head_inner_ext[0] - head_inner_ext[2]) < 0.008
        and abs(head_inner_ext[1] - head_inner_ext[2]) < 0.008,
        details=f"head inner extents: dx={head_inner_ext[0]:.4f}, "
        f"dy={head_inner_ext[1]:.4f}, dz={head_inner_ext[2]:.4f}",
    )

    # Grille ribs cover the head.
    head_ribs_aabb = ctx.part_element_world_aabb(body, elem="head_ribs")
    head_ribs_ext = _ext(head_ribs_aabb)
    ctx.check(
        "grille ribs span the full ball head diameter",
        head_ribs_ext[0] > 2 * HEAD_R - 0.005 and head_ribs_ext[2] > 2 * HEAD_R - 0.005,
        details=f"ribs extents: dx={head_ribs_ext[0]:.4f}, dz={head_ribs_ext[2]:.4f}, "
        f"expected ~{2 * HEAD_R:.4f}",
    )

    # Head sits above the body shell.
    body_shell_aabb = ctx.part_element_world_aabb(body, elem="body_shell")
    ctx.check(
        "ball grille head center is above the body shell top",
        _aabb_center(head_inner_aabb)[2] > body_shell_aabb[1][2] - 0.005,
        details=f"head_center_z={_aabb_center(head_inner_aabb)[2]:.4f}, "
        f"body_top_z={body_shell_aabb[1][2]:.4f}",
    )

    # --- body tilts about the +Y fork axis: the head swings forward (+X) ---
    head_rest = _aabb_center(head_inner_aabb)
    with ctx.pose({tilt: math.radians(30.0)}):
        head_tilted = _aabb_center(ctx.part_element_world_aabb(body, elem="head_inner"))
    ctx.check(
        "body tilts forward about the +Y fork axis: head swings toward +X",
        head_tilted[0] > head_rest[0] + 0.02 and head_tilted[2] < head_rest[2] - 0.005,
        details=f"rest head center={head_rest}, tilted head center={head_tilted}",
    )

    # --- front VOLUME knob spins about +X ---
    ctx.check(
        "volume knob mounted on the +X front face",
        ctx.part_world_position(volume)[0] > 0.02,
        details=f"volume origin={ctx.part_world_position(volume)}",
    )
    vol_marker_rest = _aabb_center(ctx.part_element_world_aabb(volume, elem="volume_marker"))
    with ctx.pose({vol_joint: math.pi / 2.0}):
        vol_marker_spun = _aabb_center(ctx.part_element_world_aabb(volume, elem="volume_marker"))
    ctx.check(
        "volume knob spins about the front (X) axis",
        abs(vol_marker_spun[1] - vol_marker_rest[1]) > 0.004
        or abs(vol_marker_spun[2] - vol_marker_rest[2]) > 0.004,
        details=f"rest marker={vol_marker_rest}, spun marker={vol_marker_spun}",
    )

    # --- side gain knob spins about +Y ---
    ctx.check(
        "gain knob mounted on the +Y side face",
        ctx.part_world_position(gain)[1] > 0.02,
        details=f"gain origin={ctx.part_world_position(gain)}",
    )
    gain_marker_rest = _aabb_center(ctx.part_element_world_aabb(gain, elem="gain_marker"))
    with ctx.pose({gain_joint: math.pi / 2.0}):
        gain_marker_spun = _aabb_center(ctx.part_element_world_aabb(gain, elem="gain_marker"))
    ctx.check(
        "gain knob spins about the side (Y) axis",
        abs(gain_marker_spun[0] - gain_marker_rest[0]) > 0.003
        or abs(gain_marker_spun[2] - gain_marker_rest[2]) > 0.003,
        details=f"rest marker={gain_marker_rest}, spun marker={gain_marker_spun}",
    )

    return ctx.report()


object_model = build_object_model()
