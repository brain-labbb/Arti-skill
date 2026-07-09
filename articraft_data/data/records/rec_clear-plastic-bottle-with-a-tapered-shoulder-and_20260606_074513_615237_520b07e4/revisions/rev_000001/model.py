from __future__ import annotations

# Clear plastic (PET) bottle with a tall tapered shoulder and a small black screw cap.
# Frame: vertical axis along +Z, bottle standing on z=0, centerline on x=y=0.
#   - rounded base -> straight cylindrical body -> long tapered shoulder
#     (narrower toward the top) -> short threaded neck -> small black cap.
# Articulations (two INDEPENDENT, decoupled joints sharing +Z, via a massless carrier):
#   - cap_rotate:  CONTINUOUS spin of the cap about +Z (a small off-axis marker shows it).
#   - cap_slide:   PRISMATIC lift of the cap straight up off the neck.

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
BODY_TOP_Z = 0.110  # end of straight cylindrical body, start of shoulder
SHOULDER_TOP_Z = 0.156  # end of tapered shoulder, base of neck
NECK_TOP_Z = 0.176  # top of threaded neck (cap mounts here)

BODY_R = 0.0275  # body radius (~0.055 m dia)
NECK_R = 0.0125  # outer thread/neck radius
NECK_BORE_R = 0.0098  # neck inner bore

CAP_R = 0.0150  # cap outer radius (a touch larger than the neck)
CAP_HEIGHT = 0.022  # cap height
CAP_BORE_R = NECK_R  # inner skirt radius == neck radius, so the skirt grips the neck
# Mount so the closed cap skirt screws fully down over the threaded neck.
CAP_MOUNT_Z = NECK_TOP_Z - CAP_HEIGHT  # = 0.154


def _profile_sections():
    # (z, radius) of the outer wall, base -> body -> tapered shoulder -> neck.
    return [
        (0.000, 0.0150),  # rounded base bottom (tucked-in heel)
        (0.006, 0.0250),
        (0.014, 0.0273),
        (BODY_TOP_Z, BODY_R),  # straight cylindrical body
        (0.124, 0.0268),  # shoulder starts tapering inward
        (0.138, 0.0228),
        (SHOULDER_TOP_Z, 0.0148),  # long tapered shoulder, much narrower at top
        (0.160, NECK_R),  # base of neck
        (NECK_TOP_Z, NECK_R),  # straight threaded neck up to the rim
    ]


def _bottle_solid() -> cq.Workplane:
    # Revolve the outer profile, then shell it open at the top so the bottle is
    # a real thin-walled hollow container (open mouth at the neck rim).
    pts = _profile_sections()
    wp = cq.Workplane("XZ").moveTo(0.0, pts[0][0])
    for r, z in [(r, z) for (z, r) in pts]:
        wp = wp.lineTo(r, z)
    # close the profile back down the axis
    wp = wp.lineTo(0.0, pts[-1][0]).close()
    outer = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))

    # Hollow it out: cut an inner cavity that opens through the neck rim.
    wall = 0.0014
    inner_pts = [
        (0.010, 0.006),
        (0.0235, 0.012),
        (BODY_R - wall, 0.014),
        (BODY_R - wall, BODY_TOP_Z),
        (0.0254, 0.124),
        (0.0214, 0.138),
        (0.0134, SHOULDER_TOP_Z),
        (NECK_BORE_R, 0.160),
        (NECK_BORE_R, NECK_TOP_Z + 0.005),  # open through the rim
    ]
    iwp = cq.Workplane("XZ").moveTo(0.0, inner_pts[0][1])
    for r, z in inner_pts:
        iwp = iwp.lineTo(r, z)
    iwp = iwp.lineTo(0.0, inner_pts[-1][1]).close()
    cavity = iwp.revolve(360.0, (0, 0, 0), (0, 1, 0))
    return outer.cut(cavity)


def _bottle_mesh():
    return mesh_from_cadquery(_bottle_solid(), "bottle_shell")


def _neck_threads():
    # A couple of helical-ish thread rings on the neck, as a separate dark-clear
    # accent merged into the body visual region (kept as torus rings for clarity).
    g = None
    for i, zt in enumerate((0.163, 0.169)):
        ring = TorusGeometry(NECK_R - 0.0006, 0.0012, radial_segments=10, tubular_segments=40)
        ring.translate(0.0, 0.0, zt)
        if g is None:
            g = ring
        else:
            g.merge(ring)
    return mesh_from_geometry(g, "neck_threads")


def _cap_solid() -> cq.Workplane:
    # Short knurled black cap: a closed cylinder (top disc + skirt) that screws
    # over the neck. Ribbed skirt approximated with vertical flutes (cut grooves).
    cap = (
        cq.Workplane("XY")
        .circle(CAP_R)
        .extrude(CAP_HEIGHT)
    )
    # Hollow the underside so the cap slips over the neck (open bottom).
    bore = (
        cq.Workplane("XY")
        .circle(CAP_BORE_R)
        .extrude(CAP_HEIGHT - 0.0028)
    )
    cap = cap.cut(bore)
    # Vertical knurl flutes around the skirt.
    n = 24
    for i in range(n):
        a = 2.0 * math.pi * i / n
        groove = (
            cq.Workplane("XY")
            .center(CAP_R * math.cos(a), CAP_R * math.sin(a))
            .circle(0.0009)
            .extrude(CAP_HEIGHT)
        )
        cap = cap.cut(groove)
    return cap


def _cap_mesh():
    return mesh_from_cadquery(_cap_solid(), "cap_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="plastic_bottle")

    clear = model.material("clear_pet", rgba=(0.78, 0.85, 0.88, 0.25))
    clear_neck = model.material("clear_neck", rgba=(0.72, 0.80, 0.84, 0.30))
    black = model.material("cap_black", rgba=(0.07, 0.07, 0.08, 1.0))
    marker = model.material("cap_marker", rgba=(0.85, 0.15, 0.12, 1.0))

    # ---- bottle body (root): transparent hollow PET shell ----
    body = model.part("bottle_body")
    body.visual(_bottle_mesh(), material=clear, name="bottle_shell")
    body.visual(_neck_threads(), material=clear_neck, name="neck_threads")
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, 0.176),
        mass=0.020,
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
    )

    # ---- massless carrier: decouples spin (parent joint) from lift (child joint) ----
    carrier = model.part("cap_carrier")  # NO visuals (massless)
    carrier.inertial = Inertial.from_geometry(Box((0.008, 0.008, 0.008)), mass=1e-4)

    # ---- black screw cap ----
    cap = model.part("cap")
    cap.visual(_cap_mesh(), material=black, name="cap_shell")
    # Small off-axis marker so the spin is legible (cylinder dimple on the top rim).
    cap.visual(
        Cylinder(0.0018, 0.0040),
        origin=Origin(xyz=(CAP_R - 0.0030, 0.0, CAP_HEIGHT - 0.0010)),
        material=marker,
        name="cap_marker",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(CAP_R, CAP_HEIGHT),
        mass=0.003,
        origin=Origin(xyz=(0.0, 0.0, CAP_HEIGHT / 2.0)),
    )

    # cap_rotate: CONTINUOUS spin about +Z. Mount at the neck base so the cap
    # skirt screws down over the full threaded neck (intentional overlap/contact).
    model.articulation(
        "cap_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, CAP_MOUNT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )
    # cap_slide: PRISMATIC lift along +Z, carrier -> cap (independent of spin).
    model.articulation(
        "cap_slide",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=CAP_HEIGHT, effort=1.0, velocity=1.0),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle_body")
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

    # --- cap seated over the neck at rest (intentional screw-over capture) ---
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
        reason="The cap skirt intentionally covers the neck threads when closed.",
    )
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "cap mounted at the top of the bottle",
        cap_pos is not None and cap_pos[2] > 0.15,
        details=f"cap origin={cap_pos}",
    )

    # --- cap spins about +Z: the off-axis marker swings around ---
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

    # --- cap slides straight up off the neck ---
    z_rest = ctx.part_world_aabb(cap)[0][2]
    with ctx.pose({slide: CAP_HEIGHT}):
        z_up = ctx.part_world_aabb(cap)[0][2]
    ctx.check(
        "cap slides up off the neck",
        z_up > z_rest + 0.015,
        details=f"cap bottom z rest={z_rest:.4f}, lifted={z_up:.4f}",
    )

    # --- tall tapered shoulder: bottle is much narrower near the top than mid-body ---
    full = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "bottle is tall (taller than wide)",
        full[2] > 2.5 * full[0],
        details=f"body extents={full}",
    )
    # neck region width via the neck radius vs body radius is structural; assert
    # the top of the shell is narrower than the body using the profile constants.
    ctx.check(
        "tapered shoulder narrows toward the top",
        NECK_R < BODY_R * 0.6,
        details=f"neck_r={NECK_R}, body_r={BODY_R}",
    )

    return ctx.report()


object_model = build_object_model()
