from __future__ import annotations

# Blue Yeti-style USB condenser microphone suspended in an elastic shock-mount
# cradle.
#
# Coordinate convention:
#   - up is +Z (the round weighted base sits on the ground at z=0)
#   - the body "front" face (badge / MUTE / VOLUME knob) faces +X
#   - the tilt axis runs along +Y (body pivots forward/back about +Y)
#
# Structure (corrected shock-mount kinematics):
#   - base (root, static): round weighted base + curved rear support arm rising
#     to an OUTER support ring; an INNER cradle ring concentric inside it; N
#     elastic suspension bands running radially from the outer ring inward to
#     the cradle ring; and a central tilt-pivot rod across the cradle center.
#     The WHOLE suspension is rigid and static — it does NOT move when the mic
#     tilts. Outer ring + bands + cradle ring + support arm + pivot rod form one
#     connected static assembly.
#   - body (REVOLUTE about +Y): tall cylinder with a fine vertical mesh-grille
#     band near the top, a domed cap, front badge + MUTE dot, and a short pivot
#     bushing on the +Y axis. The body hangs INSIDE the static cradle ring and
#     tilts fore/aft about +Y on the static pivot rod (captured pin).
#   - volume_knob (CONTINUOUS about +X): front-facing rotary VOLUME knob.
#   - gain_knob (CONTINUOUS about +Y): side-facing gain/pattern knob.

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
BODY_BOTTOM_Z = 0.060  # lowest body point above the ground, in rest pose
BODY_TUBE_H = 0.110  # straight cylindrical section height
GRILLE_BOTTOM_Z = BODY_BOTTOM_Z + 0.072  # where the mesh band starts
GRILLE_TOP_Z = BODY_BOTTOM_Z + BODY_TUBE_H  # body tube top
CAP_R = BODY_R  # dome capsule radius

# Shock-mount cradle geometry. The tilt axis sits at RING_Z (mid-body); the
# outer ring, inner cradle ring, bands and pivot rod all live at this height.
RING_Z = 0.118  # height of the cradle/outer ring plane (= tilt axis height)
OUTER_RING_R = 0.068  # outer support ring major radius
OUTER_RING_TUBE = 0.005  # outer ring tube radius
CRADLE_RING_R = 0.045  # inner cradle ring major radius (clears body pivot hub)
CRADLE_RING_TUBE = 0.004  # cradle ring tube radius
N_BANDS = 8  # number of elastic suspension bands
BAND_R = 0.0018  # band cross-section radius
# Bands stretch from just outside the cradle ring tube to just inside the outer
# ring tube, so endpoints embed slightly into each ring for a seated look and
# every band physically bridges both static rings.
BAND_INNER = CRADLE_RING_R + 0.001  # slight embed into cradle ring
BAND_OUTER = OUTER_RING_R - 0.001  # slight embed into outer ring

# Central tilt-pivot rod (static, on the base) + body pivot bushing.
PIVOT_AXLE_R = 0.0045
PIVOT_AXLE_LEN = 2.0 * OUTER_RING_R  # spans the cradle, reaches the outer ring
PIVOT_HUB_R = 0.011
PIVOT_HUB_LEN = 2.0 * (BODY_R + 0.006)  # pokes through both body walls

BODY_DZ = -RING_Z  # shift from ground coords into the body part frame


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


def _pivot_hub_mesh():
    """Short pivot bushing on the body, centered on the +Y tilt axis at the
    cradle height. The static pivot rod passes through it (captured pin); it
    pokes through both body walls so the body stays one connected piece."""
    geom = CylinderGeometry(PIVOT_HUB_R, PIVOT_HUB_LEN, radial_segments=16)
    geom.rotate_x(math.pi / 2.0)  # lay the cylinder along +Y
    geom.translate(0.0, 0.0, RING_Z)
    return mesh_from_geometry(geom, "pivot_hub")


def _base_disc():
    """Round weighted base disc with a slightly domed top and a hub stub that
    the support arm grows out of."""
    geom = CylinderGeometry(BASE_R, BASE_H, radial_segments=64)
    geom.translate(0.0, 0.0, BASE_H / 2.0)
    # Top edge fillet ring.
    rim = TorusGeometry(BASE_R - 0.004, 0.004, radial_segments=12, tubular_segments=64)
    rim.translate(0.0, 0.0, BASE_H - 0.003)
    geom.merge(rim)
    # Central hub stub where the support arm roots.
    hub = CylinderGeometry(0.020, 0.016, radial_segments=32)
    hub.translate(0.0, 0.0, BASE_H + 0.004)
    geom.merge(hub)
    return mesh_from_geometry(geom, "base_disc")


def _support_arm_mesh():
    """Curved support arm: rises from the base hub, sweeps backward and upward
    to connect to the outer support ring at the back. Built as a swept tube
    along a Catmull-Rom spline in the XZ plane."""
    pts2d = [
        (-0.018, BASE_H + 0.008),
        (-0.038, BASE_H + 0.038),
        (-0.054, 0.086),
        (-OUTER_RING_R, RING_Z),
    ]
    smooth = sample_catmull_rom_spline_2d(pts2d, samples_per_segment=10)
    # Map 2D (x, z) to 3D (x, 0, z) — arm lies in the XZ plane at y=0.
    pts3d = [(p[0], 0.0, p[1]) for p in smooth]
    tube = tube_from_spline_points(
        pts3d, radius=0.008, samples_per_segment=4, radial_segments=14,
    )
    return mesh_from_geometry(tube, "support_arm")


def _outer_ring_mesh():
    """Outer support ring: a torus at the cradle height, rigidly mounted on the
    base via the support arm. The elastic bands attach to it."""
    geom = TorusGeometry(
        OUTER_RING_R, OUTER_RING_TUBE,
        radial_segments=14, tubular_segments=56,
    )
    geom.translate(0.0, 0.0, RING_Z)
    return mesh_from_geometry(geom, "outer_ring")


def _cradle_ring_mesh():
    """Inner cradle ring: a static torus suspended at the cradle height by the
    elastic bands, concentric with and surrounding the mic body. The body hangs
    and pivots inside it. No body-clamping clips — the ring is held by the bands
    (which bridge it to the outer ring), so it stays static when the body tilts."""
    geom = TorusGeometry(
        CRADLE_RING_R, CRADLE_RING_TUBE,
        radial_segments=12, tubular_segments=48,
    )
    geom.translate(0.0, 0.0, RING_Z)
    return mesh_from_geometry(geom, "cradle_ring")


def _pivot_axle_mesh():
    """Static central tilt-pivot rod across the cradle along +Y, at the cradle
    height. Connects to the outer/cradle rings at its ends; the body pivot hub
    rotates on it."""
    geom = CylinderGeometry(PIVOT_AXLE_R, PIVOT_AXLE_LEN, radial_segments=14)
    geom.rotate_x(math.pi / 2.0)  # lay along +Y
    geom.translate(0.0, 0.0, RING_Z)
    return mesh_from_geometry(geom, "pivot_axle")


def _elastic_band(i: int):
    """One elastic suspension band at angular position i of N_BANDS, running
    radially from the static cradle ring outward to the static outer ring, at
    the cradle height. Embeds into both ring tubes so the suspension is one
    connected static assembly."""
    angle_offset = math.pi / N_BANDS
    angle = angle_offset + 2.0 * math.pi * i / N_BANDS
    band_length = BAND_OUTER - BAND_INNER
    band_mid = (BAND_INNER + BAND_OUTER) / 2.0
    geom = CylinderGeometry(BAND_R, band_length, radial_segments=8)
    geom.rotate_y(math.pi / 2.0)  # lay radially (along +X before rotate_z)
    geom.translate(band_mid, 0.0, RING_Z)
    geom.rotate_z(angle)
    return mesh_from_geometry(geom, f"band_{i}")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="usb_condenser_microphone")

    blue = model.material("body_blue", rgba=(0.30, 0.36, 0.95, 1.0))
    blue_dark = model.material("blue_dark", rgba=(0.20, 0.25, 0.78, 1.0))
    blue_mesh = model.material("blue_mesh", rgba=(0.40, 0.46, 0.98, 1.0))
    blue_base = model.material("blue_base", rgba=(0.34, 0.40, 0.97, 1.0))
    badge_blue = model.material("badge_blue", rgba=(0.86, 0.90, 1.0, 1.0))
    mute_red = model.material("mute_red", rgba=(0.85, 0.20, 0.30, 1.0))
    band_rubber = model.material("band_rubber", rgba=(0.18, 0.18, 0.22, 1.0))
    arm_grey = model.material("arm_grey", rgba=(0.42, 0.44, 0.48, 1.0))

    # ---- base + entire static suspension (root, static) ----
    base = model.part("base")
    base.visual(_base_disc(), material=blue_base, name="base_disc")
    base.visual(_support_arm_mesh(), material=arm_grey, name="support_arm")
    base.visual(_outer_ring_mesh(), material=arm_grey, name="outer_ring")
    # Inner cradle ring (static) suspended by the bands at the cradle height.
    base.visual(_cradle_ring_mesh(), material=arm_grey, name="cradle_ring")
    # Elastic suspension bands: N_BANDS thin cylinders radiating from the cradle
    # ring out to the outer ring, emitted via a for-i-in-range loop with a
    # shared geometry helper and band_i naming. All on the static base.
    for i in range(N_BANDS):
        base.visual(_elastic_band(i), material=band_rubber, name=f"band_{i}")
    # Central tilt-pivot rod (static).
    base.visual(_pivot_axle_mesh(), material=arm_grey, name="pivot_axle")
    base.inertial = Inertial.from_geometry(
        Cylinder(BASE_R, BASE_H),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, BASE_H / 2.0)),
    )

    # ---- mic body: hangs inside the static cradle, tilts about +Y ----
    # Body geometry is authored in ground coordinates; every body visual is
    # shifted down by RING_Z (BODY_DZ) into the body part frame so that the
    # joint origin at world z=RING_Z coincides with the body-local z=0.
    body_off = Origin(xyz=(0.0, 0.0, BODY_DZ))
    body = model.part("body")
    body.visual(_body_shell(), origin=body_off, material=blue, name="body_shell")
    body.visual(_mesh_grille_band("grille_band"), origin=body_off, material=blue_mesh, name="grille_band")
    body.visual(_dome_cap(), origin=body_off, material=blue_mesh, name="dome_cap")
    # Pivot bushing on the +Y axis: the static pivot rod passes through it.
    body.visual(_pivot_hub_mesh(), origin=body_off, material=blue_dark, name="pivot_hub")

    # Front "Blue" badge (oval plate on the +X face, high on the body).
    badge_z = GRILLE_BOTTOM_Z - 0.012
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
        Cylinder(BODY_R, BODY_TUBE_H),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, BODY_BOTTOM_Z + BODY_TUBE_H / 2.0 + BODY_DZ)),
    )

    model.articulation(
        "base_to_body",
        ArticulationType.REVOLUTE,
        parent=base,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, RING_Z)),
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
    volume = object_model.get_part("volume_knob")
    gain = object_model.get_part("gain_knob")
    tilt = object_model.get_articulation("base_to_body")
    vol_joint = object_model.get_articulation("body_to_volume_knob")
    gain_joint = object_model.get_articulation("body_to_gain_knob")

    # --- intentional overlaps. The whole suspension is one static assembly on
    # the base; the body pivot hub is captured on the static pivot rod. ---
    ctx.allow_overlap(
        base, base, elem_a="support_arm", elem_b="base_disc",
        reason="Support arm roots into the base hub stub.",
    )
    ctx.allow_overlap(
        base, base, elem_a="support_arm", elem_b="outer_ring",
        reason="Support arm connects into the outer ring at the back.",
    )
    # Bands bridge the outer ring and the cradle ring (both static, on base).
    for i in range(N_BANDS):
        ctx.allow_overlap(
            base, base, elem_a=f"band_{i}", elem_b="outer_ring",
            reason=f"Elastic band {i} outer endpoint seats into the outer ring tube.",
        )
        ctx.allow_overlap(
            base, base, elem_a=f"band_{i}", elem_b="cradle_ring",
            reason=f"Elastic band {i} inner endpoint seats into the cradle ring tube.",
        )
    # Pivot rod connects to both rings and is captured in the body pivot hub.
    ctx.allow_overlap(
        base, base, elem_a="pivot_axle", elem_b="outer_ring",
        reason="Central pivot rod spans to the outer ring.",
    )
    ctx.allow_overlap(
        base, base, elem_a="pivot_axle", elem_b="cradle_ring",
        reason="Central pivot rod passes through the cradle ring center.",
    )
    ctx.allow_overlap(
        base, body, elem_a="pivot_axle", elem_b="pivot_hub",
        reason="Static pivot rod is captured inside the body pivot bushing.",
    )
    ctx.allow_overlap(
        base, body, elem_a="pivot_axle", elem_b="body_shell",
        reason="Pivot rod runs through the body center (captured pin).",
    )
    ctx.allow_overlap(
        body, body, elem_a="pivot_hub", elem_b="body_shell",
        reason="Pivot bushing pokes through the body wall to connect.",
    )
    # Knobs seated against the body wall.
    ctx.allow_overlap(
        volume, body, elem_a="volume_knob", elem_b="body_shell",
        reason="Volume knob base is seated against the body front wall.",
    )
    ctx.allow_overlap(
        gain, body, elem_a="gain_knob", elem_b="body_shell",
        reason="Gain knob base is seated against the body side wall.",
    )

    # --- the suspension sits at the cradle height, NOT collapsed to the ground.
    # This is the check the original broken variant failed. ---
    cradle_aabb = ctx.part_element_world_aabb(base, elem="cradle_ring")
    cradle_zc = _aabb_center(cradle_aabb)[2]
    ctx.check(
        "inner cradle ring sits at the cradle height, well above the base",
        cradle_zc > RING_Z - 0.01 and cradle_zc < RING_Z + 0.01,
        details=f"cradle_ring z-center={cradle_zc:.4f}, expected ~{RING_Z}",
    )
    for i in range(N_BANDS):
        ba = ctx.part_element_world_aabb(base, elem=f"band_{i}")
        bz = _aabb_center(ba)[2]
        ctx.check(
            f"band_{i} sits at the cradle height (not collapsed to z~0)",
            bz > RING_Z - 0.012 and bz < RING_Z + 0.012,
            details=f"band_{i} z-center={bz:.4f}, expected ~{RING_Z}",
        )

    # --- cradle ring surrounds the body in XY ---
    body_shell_aabb = ctx.part_element_world_aabb(body, elem="body_shell")
    ctx.check(
        "cradle ring surrounds the body in XY",
        cradle_aabb[0][0] < body_shell_aabb[0][0]
        and cradle_aabb[1][0] > body_shell_aabb[1][0]
        and cradle_aabb[0][1] < body_shell_aabb[0][1]
        and cradle_aabb[1][1] > body_shell_aabb[1][1],
        details=f"cradle_xy=({cradle_aabb[0][:2]}, {cradle_aabb[1][:2]}), "
        f"body_shell_xy=({body_shell_aabb[0][:2]}, {body_shell_aabb[1][:2]})",
    )

    # --- outer ring is larger than the cradle ring ---
    outer_aabb = ctx.part_element_world_aabb(base, elem="outer_ring")
    ctx.check(
        "outer ring encloses the cradle ring in XY",
        outer_aabb[0][0] < cradle_aabb[0][0]
        and outer_aabb[1][0] > cradle_aabb[1][0]
        and outer_aabb[0][1] < cradle_aabb[0][1]
        and outer_aabb[1][1] > cradle_aabb[1][1],
        details=f"outer_xy=({outer_aabb[0][:2]}, {outer_aabb[1][:2]}), "
        f"cradle_xy=({cradle_aabb[0][:2]}, {cradle_aabb[1][:2]})",
    )

    # --- each band bridges from the cradle ring radius out to the outer ring ---
    for i in range(N_BANDS):
        ba = ctx.part_element_world_aabb(base, elem=f"band_{i}")
        max_r = 0.0
        min_r = 1e9
        for x in (ba[0][0], ba[1][0]):
            for y in (ba[0][1], ba[1][1]):
                r = math.sqrt(x * x + y * y)
                max_r = max(max_r, r)
        # nearest corner to the axis approximates the inner end radius
        for x in (ba[0][0], ba[1][0]):
            for y in (ba[0][1], ba[1][1]):
                r = math.sqrt(x * x + y * y)
                min_r = min(min_r, r)
        ctx.check(
            f"band_{i} bridges cradle ring out to outer ring",
            max_r > OUTER_RING_R * 0.85 and min_r < CRADLE_RING_R + 0.012,
            details=f"band_{i} min_r={min_r:.4f} max_r={max_r:.4f}, "
            f"cradle_r={CRADLE_RING_R}, outer_r={OUTER_RING_R}",
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

    # --- body tilts about +Y while the static cradle ring stays put ---
    cap_rest = _aabb_center(ctx.part_element_world_aabb(body, elem="dome_cap"))
    cradle_rest = _aabb_center(ctx.part_element_world_aabb(base, elem="cradle_ring"))
    with ctx.pose({tilt: math.radians(30.0)}):
        cap_tilted = _aabb_center(ctx.part_element_world_aabb(body, elem="dome_cap"))
        cradle_tilted = _aabb_center(ctx.part_element_world_aabb(base, elem="cradle_ring"))
    ctx.check(
        "body tilts forward about +Y: cap swings toward +X and drops",
        cap_tilted[0] > cap_rest[0] + 0.02 and cap_tilted[2] < cap_rest[2] - 0.005,
        details=f"rest cap center={cap_rest}, tilted cap center={cap_tilted}",
    )
    ctx.check(
        "static shock-mount cradle does NOT move when the body tilts",
        abs(cradle_tilted[0] - cradle_rest[0]) < 1e-4
        and abs(cradle_tilted[2] - cradle_rest[2]) < 1e-4,
        details=f"cradle rest={cradle_rest}, cradle tilted={cradle_tilted}",
    )

    # --- front VOLUME knob spins about +X: off-center marker swings in Y/Z ---
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

    # --- side gain knob spins about +Y: off-center marker swings in X/Z ---
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
