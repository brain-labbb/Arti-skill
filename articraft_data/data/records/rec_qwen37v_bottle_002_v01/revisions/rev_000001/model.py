from __future__ import annotations

# Tall cylindrical clear plastic bottle with a narrow neck, visible hollow mouth
# opening, a rubber gasket ring below the cap, and a black knurled screw cap.
# Frame: vertical axis along +Z, bottle standing on z=0, centerline on x=y=0.
#   - rounded base -> tall straight cylindrical body -> short tapered shoulder
#     -> narrow threaded neck with open mouth bore -> gasket ring -> black cap.
# Articulations:
#   - cap_rotate:    CONTINUOUS spin of the cap about +Z (off-axis marker shows it).
#   - cap_slide:     PRISMATIC lift of the cap straight up off the neck.
#   - gasket_mount:  FIXED attachment of the gasket ring to the bottle neck.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- key heights (m) along +Z ----
BASE_Z = 0.0
BODY_TOP_Z = 0.170       # end of tall straight cylindrical body
SHOULDER_TOP_Z = 0.200   # end of short tapered shoulder, base of neck
NECK_TOP_Z = 0.228       # top of threaded neck / mouth rim

BODY_R = 0.030           # body radius (~0.060 m diameter)
NECK_R = 0.012           # outer neck/thread radius (narrow)
NECK_BORE_R = 0.0095     # mouth bore inner radius

CAP_R = 0.0145           # cap outer radius
CAP_HEIGHT = 0.020       # cap height
CAP_BORE_R = NECK_R      # cap skirt inner bore matches neck outer
CAP_MOUNT_Z = NECK_TOP_Z - CAP_HEIGHT  # cap seated over neck

GASKET_Z = SHOULDER_TOP_Z + 0.001  # gasket sits just above shoulder on neck
GASKET_MAJOR_R = 0.0130  # gasket torus major radius (wraps around neck)
GASKET_MINOR_R = 0.0022  # gasket cross-section radius


def _profile_sections():
    """(z, radius) of the outer wall: base -> tall body -> short shoulder -> narrow neck."""
    return [
        (0.000, 0.0160),   # rounded base heel (tucked-in)
        (0.006, 0.0270),
        (0.014, 0.0298),
        (BODY_TOP_Z, BODY_R),      # tall straight cylindrical body
        (0.180, 0.0280),           # short shoulder starts tapering
        (0.192, 0.0200),
        (SHOULDER_TOP_Z, 0.0140),  # shoulder ends, neck begins
        (0.205, NECK_R),           # neck straight section
        (NECK_TOP_Z, NECK_R),     # top rim of neck / mouth
    ]


def _bottle_solid() -> cq.Workplane:
    """Revolve the outer profile, then shell it into a thin-walled hollow container
    with a visible open mouth bore at the top of the neck."""
    pts = _profile_sections()
    wp = cq.Workplane("XZ").moveTo(0.0, pts[0][0])
    for r, z in [(r, z) for (z, r) in pts]:
        wp = wp.lineTo(r, z)
    wp = wp.lineTo(0.0, pts[-1][0]).close()
    outer = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))

    # Hollow cavity: thin-walled interior that opens through the neck mouth.
    wall = 0.0014
    inner_pts = [
        (0.010, 0.006),
        (0.0250, 0.012),
        (BODY_R - wall, 0.014),
        (BODY_R - wall, BODY_TOP_Z),
        (0.0266, 0.180),
        (0.0186, 0.192),
        (0.0126, SHOULDER_TOP_Z),
        (NECK_BORE_R, 0.205),
        (NECK_BORE_R, NECK_TOP_Z + 0.005),  # opens through mouth rim
    ]
    iwp = cq.Workplane("XZ").moveTo(0.0, inner_pts[0][1])
    for r, z in inner_pts:
        iwp = iwp.lineTo(r, z)
    iwp = iwp.lineTo(0.0, inner_pts[-1][1]).close()
    cavity = iwp.revolve(360.0, (0, 0, 0), (0, 1, 0))
    return outer.cut(cavity)


def _bottle_mesh():
    return mesh_from_cadquery(_bottle_solid(), "bottle_shell")


def _mouth_rim_mesh():
    """A small raised rim/lip ring at the top of the neck to make the mouth opening
    visually prominent."""
    rim = TorusGeometry(NECK_R + 0.0008, 0.0015, radial_segments=10, tubular_segments=40)
    rim.translate(0.0, 0.0, NECK_TOP_Z)
    return mesh_from_geometry(rim, "mouth_rim")


def _neck_threads():
    """Helical-ish thread rings on the neck exterior."""
    g = None
    for zt in (0.208, 0.214, 0.220):
        ring = TorusGeometry(NECK_R - 0.0005, 0.0010, radial_segments=10, tubular_segments=40)
        ring.translate(0.0, 0.0, zt)
        if g is None:
            g = ring
        else:
            g.merge(ring)
    return mesh_from_geometry(g, "neck_threads")


def _gasket_mesh():
    """Rubber gasket/seal ring — a torus centered at local z=0.
    The FIXED articulation places the part frame at the correct world height."""
    gasket = TorusGeometry(
        GASKET_MAJOR_R, GASKET_MINOR_R,
        radial_segments=12, tubular_segments=48,
    )
    # Keep centered at z=0 in local part frame; articulation origin handles placement.
    return mesh_from_geometry(gasket, "gasket_ring")


def _cap_solid() -> cq.Workplane:
    """Short knurled black cap: closed top + skirt that screws over the neck."""
    cap = (
        cq.Workplane("XY")
        .circle(CAP_R)
        .extrude(CAP_HEIGHT)
    )
    # Hollow the underside so the cap slips over the neck (open bottom).
    bore = (
        cq.Workplane("XY")
        .circle(CAP_BORE_R)
        .extrude(CAP_HEIGHT - 0.003)
    )
    cap = cap.cut(bore)
    # Vertical knurl flutes around the skirt.
    n = 24
    for i in range(n):
        a = 2.0 * math.pi * i / n
        groove = (
            cq.Workplane("XY")
            .center(CAP_R * math.cos(a), CAP_R * math.sin(a))
            .circle(0.0008)
            .extrude(CAP_HEIGHT)
        )
        cap = cap.cut(groove)
    return cap


def _cap_mesh():
    return mesh_from_cadquery(_cap_solid(), "cap_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tall_cylindrical_bottle")

    clear = model.material("clear_pet", rgba=(0.78, 0.86, 0.90, 0.22))
    clear_neck = model.material("clear_neck", rgba=(0.72, 0.80, 0.84, 0.30))
    rubber = model.material("gasket_rubber", rgba=(0.18, 0.18, 0.20, 1.0))
    black = model.material("cap_black", rgba=(0.06, 0.06, 0.07, 1.0))
    marker = model.material("cap_marker", rgba=(0.85, 0.15, 0.12, 1.0))

    # ---- bottle body (root): tall transparent hollow cylinder with narrow neck ----
    body = model.part("bottle_body")
    body.visual(_bottle_mesh(), material=clear, name="bottle_shell")
    body.visual(_neck_threads(), material=clear_neck, name="neck_threads")
    body.visual(_mouth_rim_mesh(), material=clear_neck, name="mouth_rim")
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, NECK_TOP_Z),
        mass=0.024,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2.0)),
    )

    # ---- gasket ring: rubber seal that sits on the neck below the cap ----
    gasket = model.part("gasket")
    gasket.visual(_gasket_mesh(), material=rubber, name="gasket_ring")
    gasket.inertial = Inertial.from_geometry(
        Cylinder(GASKET_MAJOR_R + GASKET_MINOR_R, 2.0 * GASKET_MINOR_R),
        mass=0.002,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),  # local frame, torus centered at origin
    )

    # ---- massless carrier: decouples spin from lift ----
    carrier = model.part("cap_carrier")
    carrier.inertial = Inertial.from_geometry(Box((0.008, 0.008, 0.008)), mass=1e-4)

    # ---- black screw cap ----
    cap = model.part("cap")
    cap.visual(_cap_mesh(), material=black, name="cap_shell")
    # Off-axis marker so spin is visible.
    cap.visual(
        Cylinder(0.0016, 0.0035),
        origin=Origin(xyz=(CAP_R - 0.003, 0.0, CAP_HEIGHT - 0.001)),
        material=marker,
        name="cap_marker",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(CAP_R, CAP_HEIGHT),
        mass=0.003,
        origin=Origin(xyz=(0.0, 0.0, CAP_HEIGHT / 2.0)),
    )

    # ---- Articulations ----

    # gasket_mount: FIXED — gasket sits on the neck, attached to body.
    # The gasket torus center is at GASKET_Z + GASKET_MINOR_R in world space.
    model.articulation(
        "gasket_mount",
        ArticulationType.FIXED,
        parent=body,
        child=gasket,
        origin=Origin(xyz=(0.0, 0.0, GASKET_Z + GASKET_MINOR_R)),
    )

    # cap_rotate: CONTINUOUS spin about +Z.
    model.articulation(
        "cap_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, CAP_MOUNT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )

    # cap_slide: PRISMATIC lift along +Z, carrier -> cap.
    model.articulation(
        "cap_slide",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=CAP_HEIGHT + 0.01, effort=1.0, velocity=1.0),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle_body")
    gasket = object_model.get_part("gasket")
    cap = object_model.get_part("cap")
    rotate = object_model.get_articulation("cap_rotate")
    slide = object_model.get_articulation("cap_slide")

    # --- bottle is clear (alpha < 1) ---
    clear_mat = next(m for m in object_model.materials if m.name == "clear_pet")
    a = clear_mat.rgba[3] if clear_mat.rgba is not None else 1.0
    ctx.check(
        "bottle shell is transparent",
        a < 1.0,
        details=f"clear_pet alpha={a}",
    )

    # --- bottle is tall and cylindrical ---
    full = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "bottle is tall (taller than 3x wide)",
        full[2] > 3.0 * full[0],
        details=f"body extents={full}",
    )
    ctx.check(
        "narrow neck is much smaller than body",
        NECK_R < BODY_R * 0.5,
        details=f"neck_r={NECK_R}, body_r={BODY_R}",
    )

    # --- gasket ring exists and is positioned on the neck ---
    gasket_pos = ctx.part_world_position(gasket)
    ctx.check(
        "gasket ring is on the neck region",
        gasket_pos is not None and SHOULDER_TOP_Z - 0.005 < gasket_pos[2] < NECK_TOP_Z,
        details=f"gasket origin z={gasket_pos}",
    )

    # --- gasket is below the cap at rest ---
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "gasket sits below the cap",
        gasket_pos is not None and cap_pos is not None and gasket_pos[2] < cap_pos[2],
        details=f"gasket_z={gasket_pos[2]:.4f}, cap_z={cap_pos[2]:.4f}",
    )

    # --- hollow mouth opening exists (mouth_rim visual on body) ---
    mouth_rim = body.get_visual("mouth_rim")
    ctx.check(
        "visible mouth rim at the neck top",
        mouth_rim is not None,
        details="mouth_rim visual not found on bottle_body",
    )

    # --- cap seated over the neck at rest ---
    ctx.allow_overlap(
        cap,
        body,
        elem_a="cap_shell",
        elem_b="bottle_shell",
        reason="The cap skirt intentionally screws down over the threaded neck.",
    )
    ctx.allow_overlap(
        cap,
        body,
        elem_a="cap_shell",
        elem_b="neck_threads",
        reason="The cap skirt covers the neck threads when closed.",
    )
    ctx.allow_overlap(
        cap,
        gasket,
        elem_a="cap_shell",
        elem_b="gasket_ring",
        reason="The cap skirt extends down over the gasket ring when fully screwed on.",
    )
    ctx.check(
        "cap mounted at the top of the bottle",
        cap_pos is not None and cap_pos[2] > 0.19,
        details=f"cap origin={cap_pos}",
    )

    # --- cap spins about +Z (marker swings) ---
    marker = cap.get_visual("cap_marker")

    def _marker_xy():
        mn, mx = ctx.part_element_world_aabb(cap, elem=marker)
        return ((mn[0] + mx[0]) / 2.0, (mn[1] + mx[1]) / 2.0)

    mk0 = _marker_xy()
    with ctx.pose({rotate: math.pi / 2.0}):
        mk90 = _marker_xy()
    moved = math.hypot(mk90[0] - mk0[0], mk90[1] - mk0[1])
    ctx.check(
        "cap spins about +Z (marker swings around)",
        moved > 0.005,
        details=f"marker rest={mk0}, quarter-turn={mk90}, moved={moved:.4f}",
    )

    # --- cap slides up off the neck ---
    z_rest = ctx.part_world_aabb(cap)[0][2]
    with ctx.pose({slide: CAP_HEIGHT}):
        z_up = ctx.part_world_aabb(cap)[0][2]
    ctx.check(
        "cap slides up off the neck",
        z_up > z_rest + 0.015,
        details=f"cap bottom z rest={z_rest:.4f}, lifted={z_up:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
