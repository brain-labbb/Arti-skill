from __future__ import annotations

# Blue ribbed plastic barrel keg with a rear-hinged flip-top lid.
# Frame: barrel axis along +Z, base sits on z=0, lid on top.
#   - body (root): blue lathed barrel shell with a closed bottom, hollow
#     interior, visible open mouth, horizontal rib tori, threaded neck,
#     and a rear hinge lug with a pin.
#   - lid: black flip-top disk with a hinge knuckle at the rear and a
#     front latch tab, swinging open about a horizontal REVOLUTE hinge
#     anchored on the back of the neck rim.
# Articulation (single REVOLUTE):
#   - lid_hinge : body -> lid, horizontal axis along X at rear of neck rim.
#     Positive q opens the lid upward/backward.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)

# ---- key dimensions (meters) -------------------------------------------------
BODY_BOTTOM_Z = 0.0
BARREL_TOP_Z = 0.255          # top shoulder of the blue barrel body
NECK_TOP_Z = 0.270            # top rim of the neck (hinge height)
BOTTOM_FLOOR_Z = 0.012        # inner floor height, leaving a real bottom thickness
MOUTH_RADIUS = 0.055          # visible open mouth radius
NECK_RADIUS = 0.070           # outer radius of the neck
BARREL_MAX_R = 0.110          # widest barrel radius (belly)
LID_RADIUS = 0.062            # flip-top disk radius (covers mouth, sits on rim)
LID_THICKNESS = 0.008         # disk thickness
HINGE_Y = -NECK_RADIUS        # hinge at rear of neck rim
HINGE_PIN_RADIUS = 0.004      # hinge pin radius
HINGE_KNUCKLE_RADIUS = 0.008  # knuckle outer radius
HINGE_LENGTH = 0.036          # hinge span along X
EAR_HEIGHT = 0.014            # lug ear height (below rim, on neck wall)
EAR_WIDTH = 0.016             # ear depth (radial direction)


def _barrel_body() -> object:
    # Bulged barrel shell as a revolved material cross-section.  The profile
    # includes a center bottom underside and an inner floor, so the barrel has
    # a bottom while retaining an open, hollow interior up to the mouth.
    outer = [
        (0.000, 0.000),   # bottom underside center: closes the base
        (0.090, 0.000),
        (0.098, 0.006),
        (0.105, 0.030),
        (0.110, 0.110),   # belly
        (0.108, 0.160),
        (0.098, 0.215),
        (0.082, 0.245),   # shoulder
        (0.070, 0.255),   # top of body, meeting the neck
        (NECK_RADIUS, 0.270),
    ]
    inner = [
        (MOUTH_RADIUS, 0.270),  # open mouth: no center cap at the top
        (0.060, 0.255),
        (0.074, 0.245),
        (0.090, 0.215),
        (0.100, 0.160),
        (0.102, 0.110),
        (0.097, 0.035),
        (0.090, 0.018),
        (0.082, BOTTOM_FLOOR_Z),
        (0.000, BOTTOM_FLOOR_Z),  # interior floor center
    ]
    return LatheGeometry(outer + inner, segments=64, closed=True)


def _body_mesh():
    body = _barrel_body()
    # Horizontal rib tori wrapped around the belly at the local outer radius.
    rib_specs = [
        (0.045, 0.106),
        (0.075, 0.109),
        (0.105, 0.110),
        (0.135, 0.109),
        (0.165, 0.107),
        (0.195, 0.099),
        (0.222, 0.086),
    ]
    for z, r in rib_specs:
        rib = TorusGeometry(r + 0.004, 0.009, radial_segments=12, tubular_segments=64)
        rib.translate(0.0, 0.0, z)
        body.merge(rib)
    # Threaded neck rings sit around the open mouth without sealing its center.
    for i, tz in enumerate((0.252, 0.262)):
        thread = TorusGeometry(NECK_RADIUS - 0.002, 0.004, radial_segments=10, tubular_segments=48)
        thread.translate(0.0, 0.0, tz)
        body.merge(thread)

    # --- Rear hinge lug: two ears on the neck outer wall, below the rim -----
    # Ears straddle the neck wall so they are physically connected to the body.
    # Center ears at z = NECK_TOP_Z - EAR_HEIGHT/2 so they end flush with rim.
    ear_z_center = NECK_TOP_Z - EAR_HEIGHT / 2.0
    ear_thickness = 0.006
    lug_half_span = HINGE_LENGTH / 2.0
    for i, x_off in enumerate((-lug_half_span + ear_thickness / 2, lug_half_span - ear_thickness / 2)):
        # Ear overlaps the neck wall radially (inner face penetrates the neck
        # outer surface slightly for robust connectivity).
        ear = BoxGeometry((ear_thickness, EAR_WIDTH, EAR_HEIGHT))
        ear.translate(x_off, HINGE_Y + EAR_WIDTH / 2.0 - 0.006, ear_z_center)
        body.merge(ear)

    # Hinge pin: thin cylinder along X through the lug ears, at rim level.
    pin_z = NECK_TOP_Z - 0.002  # slightly below rim, within the ear height
    pin = CylinderGeometry(HINGE_PIN_RADIUS, HINGE_LENGTH, radial_segments=16)
    pin.rotate_y(math.pi / 2.0)  # align cylinder (default +Z) to +X
    pin.translate(0.0, HINGE_Y, pin_z)
    body.merge(pin)

    # Small front latch catch on the opposite side of the neck wall.
    # Positioned on the neck outer surface, below the rim, for connectivity.
    latch = BoxGeometry((0.014, 0.008, 0.012))
    latch.translate(0.0, NECK_RADIUS + 0.002, NECK_TOP_Z - 0.008)
    body.merge(latch)

    return mesh_from_geometry(body, "barrel_shell")


def _lid_disk_geometry():
    """Flip-top lid: a flat disk with a hinge knuckle and front latch tab.

    In the lid local frame, origin is at the hinge point. The disk center
    is offset by +Y (toward the barrel center/mouth) from the hinge.
    """
    # Main disk — sits on the neck rim when closed.
    disk = CylinderGeometry(LID_RADIUS, LID_THICKNESS, radial_segments=48)
    disk.translate(0.0, NECK_RADIUS, LID_THICKNESS / 2.0)

    # Sealing ring on the underside (thin torus near the disk edge).
    seal = TorusGeometry(LID_RADIUS - 0.006, 0.002, radial_segments=8, tubular_segments=48)
    seal.translate(0.0, NECK_RADIUS, 0.001)
    disk.merge(seal)

    # Connecting strap from knuckle area to disk (ensures mesh connectivity).
    # The knuckle is at origin (0,0,0); the disk edge closest to origin is
    # at y = NECK_RADIUS - LID_RADIUS = 0.008. The strap bridges this gap.
    strap_y_start = -0.002
    strap_y_end = NECK_RADIUS - LID_RADIUS + 0.010
    strap_length = strap_y_end - strap_y_start
    strap = BoxGeometry((0.028, strap_length, LID_THICKNESS))
    strap.translate(0.0, strap_y_start + strap_length / 2.0, LID_THICKNESS / 2.0)
    disk.merge(strap)

    # Hinge knuckle at the rear — wraps around the hinge pin.
    # In local frame, hinge is at origin. The knuckle is centered at origin.
    knuckle = CylinderGeometry(HINGE_KNUCKLE_RADIUS, HINGE_LENGTH * 0.60, radial_segments=16)
    knuckle.rotate_y(math.pi / 2.0)
    disk.merge(knuckle)

    # Front latch tab: protrusion that clips into the body catch.
    # Positioned to overlap with the disk front edge for connectivity.
    tab_y = NECK_RADIUS + LID_RADIUS - 0.008
    tab = BoxGeometry((0.012, 0.018, 0.006))
    tab.translate(0.0, tab_y, LID_THICKNESS / 2.0)
    disk.merge(tab)

    return disk


def _lid_mesh():
    lid = _lid_disk_geometry()
    return mesh_from_geometry(lid, "lid_disk")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="blue_barrel_keg_fliptop")

    blue = model.material("barrel_blue", rgba=(0.16, 0.34, 0.78, 1.0))
    black = model.material("lid_black", rgba=(0.10, 0.10, 0.11, 1.0))

    # ---- body (root) --------------------------------------------------------
    body = model.part("body")
    body.visual(_body_mesh(), material=blue, name="barrel_shell")
    body.inertial = Inertial.from_geometry(
        Cylinder(0.11, 0.27), mass=2.2, origin=Origin(xyz=(0.0, 0.0, 0.13))
    )

    # ---- lid (flip-top disk) ------------------------------------------------
    lid = model.part("lid")
    lid.visual(_lid_mesh(), material=black, name="lid_disk")
    lid.inertial = Inertial.from_geometry(
        Cylinder(LID_RADIUS, LID_THICKNESS), mass=0.05,
        origin=Origin(xyz=(0.0, NECK_RADIUS, LID_THICKNESS / 2.0)),
    )

    # ---- articulation: rear-hinged flip-top, horizontal axis along X --------
    # Hinge origin at the rear of the neck rim. The lid local frame coincides
    # with the articulation frame at q=0 (closed). In the lid local frame,
    # the disk extends in +Y (toward barrel center/front) from the hinge.
    # Axis (1,0,0): right-hand rule makes positive q rotate +Y toward +Z,
    # lifting the front edge of the lid upward (opening).
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, NECK_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=2.6,  # ~149 degrees open
            effort=3.0,
            velocity=2.0,
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
    hinge = object_model.get_articulation("lid_hinge")

    # The hinge knuckle on the lid intentionally wraps around the hinge pin
    # area on the body lug — small local overlap at the pivot.
    ctx.allow_overlap(
        lid,
        body,
        elem_a="lid_disk",
        elem_b="barrel_shell",
        reason="The lid hinge knuckle wraps around the body hinge pin at the rear lug, creating small local overlap at the pivot.",
    )

    # --- Closed pose (q=0): lid covers the mouth, sits on the neck rim ------
    with ctx.pose({hinge: 0.0}):
        # Lid disk overlaps the mouth area in XY
        ctx.expect_overlap(
            lid, body, axes="xy", min_overlap=0.04,
            name="closed lid covers the barrel mouth in XY",
        )
        # Lid sits on top of the neck — the knuckle wraps below the hinge
        # origin, so allow up to HINGE_KNUCKLE_RADIUS penetration.
        ctx.expect_gap(
            lid, body, axis="z",
            max_penetration=HINGE_KNUCKLE_RADIUS + 0.004,
            max_gap=0.005,
            name="closed lid sits on the neck rim",
        )

    # --- Open pose: lid swings upward/backward, mouth becomes exposed --------
    open_angle = 1.8  # ~103 degrees
    with ctx.pose({hinge: 0.0}):
        closed_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({hinge: open_angle}):
        open_aabb = ctx.part_world_aabb(lid)
        open_center_z = (open_aabb[0][2] + open_aabb[1][2]) / 2.0
    closed_center_z = (closed_aabb[0][2] + closed_aabb[1][2]) / 2.0
    ctx.check(
        "open lid swings above the mouth",
        open_center_z > closed_center_z + 0.03,
        details=f"closed_center_z={closed_center_z:.4f}, open_center_z={open_center_z:.4f}",
    )

    # --- Hinge location: at the rear of the neck rim -------------------------
    hinge_origin = hinge.origin
    ctx.check(
        "hinge is anchored at the rear of the neck rim",
        hinge_origin is not None
        and hinge_origin.xyz[1] < -NECK_RADIUS + 0.01
        and abs(hinge_origin.xyz[2] - NECK_TOP_Z) < 0.005,
        details=f"hinge origin={hinge_origin.xyz if hinge_origin else None}",
    )

    # --- Barrel body: still bulged and ribbed --------------------------------
    body_ext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "barrel body is bulged/ribbed (wide belly)",
        body_ext[0] > 0.20 and body_ext[1] > 0.20,
        details=f"body extents={body_ext}",
    )

    # --- Geometry checks: barrel interior, mouth, and lid disk ---------------
    body_geom = _barrel_body()
    radii_z = [(math.hypot(x, y), z) for x, y, z in body_geom.vertices]
    bottom_has_center = any(
        r < 0.003 and abs(z - BODY_BOTTOM_Z) < 1e-6 for r, z in radii_z
    )
    ctx.check(
        "barrel bottom has a center face",
        bottom_has_center,
        details="bottom includes vertices on the center axis at z=0",
    )
    mid_band = [r for r, z in radii_z if 0.100 <= z <= 0.120]
    ctx.check(
        "barrel interior remains hollow at the belly",
        bool(mid_band) and min(mid_band) > 0.090,
        details=f"mid-band min radius={min(mid_band) if mid_band else None}",
    )
    mouth_band = [r for r, z in radii_z if abs(z - NECK_TOP_Z) < 1e-6]
    ctx.check(
        "barrel mouth is a visible opening",
        bool(mouth_band) and min(mouth_band) >= MOUTH_RADIUS - 0.001,
        details=f"mouth radii min/max={((min(mouth_band), max(mouth_band)) if mouth_band else None)}",
    )

    # Lid disk has coverage beyond the mouth radius.
    lid_geom = _lid_disk_geometry()
    lid_vertices = lid_geom.vertices
    # Check vertices at the disk top face (z = LID_THICKNESS after translation)
    top_face_radii = [
        math.hypot(x, y) for x, y, z in lid_vertices
        if abs(z - LID_THICKNESS) < 0.002
    ]
    ctx.check(
        "flip-top lid has a disk covering the mouth area",
        bool(top_face_radii) and max(top_face_radii) >= LID_RADIUS + NECK_RADIUS - 0.010,
        details=f"lid top face max radius={max(top_face_radii) if top_face_radii else None}",
    )

    return ctx.report()
