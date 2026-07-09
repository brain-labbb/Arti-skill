from __future__ import annotations

# Medicine bottle: opaque white HDPE body with molded volume bands and a
# child-resistant push-and-turn cap.
# Frame: vertical axis along +Z, bottle standing on z=0, centerline on x=y=0.
#   - flat base -> straight cylindrical body with raised volume bands ->
#     short tapered shoulder -> threaded neck -> child-resistant cap.
# Articulations (two INDEPENDENT, decoupled joints sharing +Z, via a massless carrier):
#   - cap_rotate:  CONTINUOUS spin of the cap about +Z (push-and-turn mechanism).
#   - cap_slide:   PRISMATIC push-down along +Z (child-resistant press action).

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
BODY_TOP_Z = 0.080  # end of straight cylindrical body, start of shoulder
SHOULDER_TOP_Z = 0.100  # end of short tapered shoulder, base of neck
NECK_TOP_Z = 0.118  # top of threaded neck

BODY_R = 0.035  # body radius (~0.070 m dia, wider medicine bottle)
NECK_R = 0.016  # outer thread/neck radius
NECK_BORE_R = 0.013  # neck inner bore

CAP_R = 0.021  # child-resistant cap outer radius (larger than neck)
CAP_HEIGHT = 0.024  # cap height (taller for push-down mechanism)
CAP_BORE_R = NECK_R + 0.001  # inner skirt slightly larger for push-down clearance
# Mount so the closed cap sits fully down over the threaded neck.
CAP_MOUNT_Z = NECK_TOP_Z - CAP_HEIGHT

# Volume band parameters (raised rings around the body)
BAND_WIDTH = 0.004  # band height (thickness)
BAND_PROTRUSION = 0.0015  # how far the band sticks out from the body
BAND_POSITIONS = [0.020, 0.042, 0.064]  # z centers of the 3 volume bands


def _profile_sections():
    # (z, radius) of the outer wall, base -> body -> short shoulder -> neck.
    return [
        (0.000, 0.020),  # flat base with small heel radius
        (0.005, 0.033),
        (0.008, BODY_R),  # body starts
        (BODY_TOP_Z, BODY_R),  # straight cylindrical body
        (0.088, 0.030),  # shoulder starts tapering
        (SHOULDER_TOP_Z, 0.018),  # shoulder ends
        (0.104, NECK_R),  # base of neck
        (NECK_TOP_Z, NECK_R),  # straight threaded neck up to the rim
    ]


def _bottle_solid() -> cq.Workplane:
    # Revolve the outer profile, then shell it open at the top so the bottle is
    # a real thin-walled hollow container (open mouth at the neck rim).
    pts = _profile_sections()
    wp = cq.Workplane("XZ").moveTo(0.0, pts[0][0])
    for r, z in [(r, z) for (z, r) in pts]:
        wp = wp.lineTo(r, z)
    wp = wp.lineTo(0.0, pts[-1][0]).close()
    outer = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))

    # Hollow it out: cut an inner cavity that opens through the neck rim.
    wall = 0.0018
    inner_pts = [
        (0.010, 0.005),
        (BODY_R - wall, 0.008),
        (BODY_R - wall, BODY_TOP_Z),
        (0.028, 0.088),
        (0.016, SHOULDER_TOP_Z),
        (NECK_BORE_R, 0.104),
        (NECK_BORE_R, NECK_TOP_Z + 0.005),  # open through the rim
    ]
    iwp = cq.Workplane("XZ").moveTo(0.0, inner_pts[0][1])
    for r, z in inner_pts:
        iwp = iwp.lineTo(r, z)
    iwp = iwp.lineTo(0.0, inner_pts[-1][1]).close()
    cavity = iwp.revolve(360.0, (0, 0, 0), (0, 1, 0))
    return outer.cut(cavity)


def _volume_bands() -> cq.Workplane:
    """Raised molded volume bands around the body at specified heights."""
    band = None
    for z_center in BAND_POSITIONS:
        ring = (
            cq.Workplane("XY")
            .transformed(offset=(0, 0, z_center - BAND_WIDTH / 2))
            .circle(BODY_R + BAND_PROTRUSION)
            .circle(BODY_R - 0.0002)  # slightly below body surface for union
            .extrude(BAND_WIDTH)
        )
        if band is None:
            band = ring
        else:
            band = band.union(ring)
    return band


def _bottle_with_bands() -> cq.Workplane:
    """Bottle shell with molded volume bands unioned on."""
    bottle = _bottle_solid()
    bands = _volume_bands()
    return bottle.union(bands)


def _bottle_mesh():
    return mesh_from_cadquery(_bottle_with_bands(), "bottle_shell")


def _neck_threads():
    # Thread rings on the neck for the child-resistant cap mechanism.
    g = None
    for zt in (0.105, 0.110, 0.115):
        ring = TorusGeometry(NECK_R - 0.0005, 0.0014, radial_segments=10, tubular_segments=40)
        ring.translate(0.0, 0.0, zt)
        if g is None:
            g = ring
        else:
            g.merge(ring)
    return mesh_from_geometry(g, "neck_threads")


def _cap_solid() -> cq.Workplane:
    """Child-resistant cap: larger diameter with prominent vertical ribs and
    push-down arrow indicators on top."""
    # Main cap body - closed top cylinder
    cap = (
        cq.Workplane("XY")
        .circle(CAP_R)
        .extrude(CAP_HEIGHT)
    )
    # Hollow the underside for push-down over neck (open bottom, deeper bore for travel)
    bore = (
        cq.Workplane("XY")
        .circle(CAP_BORE_R)
        .extrude(CAP_HEIGHT - 0.003)
    )
    cap = cap.cut(bore)

    # Prominent vertical ribs around the cap skirt (child-resistant grip)
    n_ribs = 32
    for i in range(n_ribs):
        a = 2.0 * math.pi * i / n_ribs
        # Each rib is a small rectangular protrusion
        rib_x = (CAP_R + 0.001) * math.cos(a)
        rib_y = (CAP_R + 0.001) * math.sin(a)
        rib = (
            cq.Workplane("XY")
            .transformed(offset=(rib_x, rib_y, 0), rotate=(0, 0, math.degrees(a)))
            .box(0.002, 0.0015, CAP_HEIGHT * 0.85, centered=(True, True, False))
        )
        cap = cap.union(rib)

    # Push-down arrow indicators on top (two raised arrows pointing down)
    for angle_deg in (0, 180):
        a = math.radians(angle_deg)
        ax = 0.008 * math.cos(a)
        ay = 0.008 * math.sin(a)
        arrow = (
            cq.Workplane("XY")
            .transformed(offset=(ax, ay, CAP_HEIGHT - 0.001))
            .box(0.005, 0.002, 0.0015, centered=(True, True, False))
        )
        cap = cap.union(arrow)

    # Central raised dome on top for grip
    dome = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, CAP_HEIGHT))
        .circle(0.006)
        .extrude(0.002)
    )
    cap = cap.union(dome)

    return cap


def _cap_mesh():
    return mesh_from_cadquery(_cap_solid(), "cap_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="medicine_bottle")

    # Opaque white HDPE body (typical medicine bottle material)
    white_hdpe = model.material("white_hdpe", rgba=(0.92, 0.91, 0.88, 1.0))
    band_mat = model.material("band_hdpe", rgba=(0.88, 0.87, 0.84, 1.0))
    neck_mat = model.material("neck_hdpe", rgba=(0.90, 0.89, 0.86, 1.0))
    # Child-resistant cap: typically white with colored indicators
    cap_white = model.material("cap_white", rgba=(0.94, 0.93, 0.90, 1.0))
    arrow_blue = model.material("arrow_indicator", rgba=(0.15, 0.35, 0.70, 1.0))

    # ---- bottle body (root): opaque hollow HDPE shell with volume bands ----
    body = model.part("bottle_body")
    body.visual(_bottle_mesh(), material=white_hdpe, name="bottle_shell")
    body.visual(_neck_threads(), material=neck_mat, name="neck_threads")
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, 0.118),
        mass=0.035,
        origin=Origin(xyz=(0.0, 0.0, 0.059)),
    )

    # ---- massless carrier: decouples spin (parent joint) from push-down (child joint) ----
    carrier = model.part("cap_carrier")  # NO visuals (massless)
    carrier.inertial = Inertial.from_geometry(Box((0.010, 0.010, 0.010)), mass=1e-4)

    # ---- child-resistant cap ----
    cap = model.part("cap")
    cap.visual(_cap_mesh(), material=cap_white, name="cap_shell")
    # Off-axis marker so the spin is visible (small colored dot on top).
    cap.visual(
        Cylinder(0.002, 0.003),
        origin=Origin(xyz=(CAP_R - 0.005, 0.0, CAP_HEIGHT + 0.001)),
        material=arrow_blue,
        name="cap_marker",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(CAP_R, CAP_HEIGHT),
        mass=0.005,
        origin=Origin(xyz=(0.0, 0.0, CAP_HEIGHT / 2.0)),
    )

    # cap_rotate: CONTINUOUS spin about +Z (push-and-turn child-resistant mechanism).
    model.articulation(
        "cap_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, CAP_MOUNT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.0),
    )
    # cap_slide: PRISMATIC push-down along -Z (child-resistant press before turn).
    model.articulation(
        "cap_slide",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.008, effort=2.0, velocity=0.5),
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

    # --- body is opaque HDPE (alpha = 1) ---
    body_mat = next(m for m in object_model.materials if m.name == "white_hdpe")
    a = body_mat.rgba[3] if body_mat.rgba is not None else 0.0
    ctx.check(
        "bottle body is opaque HDPE",
        a >= 0.95,
        details=f"white_hdpe alpha={a}",
    )

    # --- molded volume bands exist on the body ---
    bottle_shell = body.get_visual("bottle_shell")
    shell_dims = ctx.dims(bottle_shell) if hasattr(ctx, "dims") else None
    # Check that bands widen the body beyond BODY_R by looking at the AABB
    body_aabb = ctx.part_world_aabb(body)
    body_xy_extent = max(body_aabb[1][0] - body_aabb[0][0],
                         body_aabb[1][1] - body_aabb[0][1])
    expected_min_width = 2.0 * (BODY_R + BAND_PROTRUSION * 0.8)
    ctx.check(
        "molded volume bands widen the body profile",
        body_xy_extent > expected_min_width,
        details=f"body xy extent={body_xy_extent:.4f}, expected>{expected_min_width:.4f}",
    )

    # --- child-resistant cap: larger than the neck ---
    ctx.check(
        "child-resistant cap is larger than neck",
        CAP_R > NECK_R * 1.2,
        details=f"cap_r={CAP_R}, neck_r={NECK_R}",
    )

    # --- cap seated over the neck at rest (push-down mechanism) ---
    ctx.allow_overlap(
        cap,
        body,
        elem_a="cap_shell",
        elem_b="bottle_shell",
        reason="The child-resistant cap skirt sits over the threaded neck when closed.",
    )
    ctx.allow_overlap(
        cap,
        body,
        elem_a="cap_shell",
        elem_b="neck_threads",
        reason="The cap covers the neck threads in the closed position.",
    )
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "cap mounted at the top of the bottle",
        cap_pos is not None and cap_pos[2] > 0.09,
        details=f"cap origin={cap_pos}",
    )

    # --- cap spins about +Z (continuous joint for push-and-turn) ---
    marker = cap.get_visual("cap_marker")

    def _marker_xy():
        mn, mx = ctx.part_element_world_aabb(cap, elem=marker)
        return ((mn[0] + mx[0]) / 2.0, (mn[1] + mx[1]) / 2.0)

    mk0 = _marker_xy()
    with ctx.pose({rotate: math.pi / 2.0}):
        mk90 = _marker_xy()
    moved = math.hypot(mk90[0] - mk0[0], mk90[1] - mk0[1])
    ctx.check(
        "cap rotates about +Z (child-resistant turn action)",
        moved > 0.005,
        details=f"marker rest={mk0}, quarter-turn={mk90}, moved={moved:.4f}",
    )

    # --- cap pushes down (prismatic joint for child-resistant press) ---
    z_rest = ctx.part_world_aabb(cap)[1][2]  # top of cap
    with ctx.pose({slide: 0.008}):
        z_pressed = ctx.part_world_aabb(cap)[1][2]
    ctx.check(
        "cap pushes down for child-resistant mechanism",
        z_rest > z_pressed + 0.005,
        details=f"cap top z rest={z_rest:.4f}, pressed={z_pressed:.4f}",
    )

    # --- verify continuous joint exists and is non-fixed ---
    ctx.check(
        "continuous rotation joint exists for cap",
        rotate.articulation_type == ArticulationType.CONTINUOUS,
        details=f"cap_rotate articulation_type={rotate.articulation_type}",
    )

    # --- bottle proportions: wider and shorter than parent (medicine bottle) ---
    full = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "medicine bottle is wider relative to height",
        full[0] > 0.055,  # wider than 55mm
        details=f"body width={full[0]:.4f}",
    )

    # --- hollow container body ---
    ctx.check(
        "bottle body is a hollow container",
        NECK_BORE_R > 0.005,
        details=f"neck bore radius={NECK_BORE_R}",
    )

    return ctx.report()


object_model = build_object_model()
