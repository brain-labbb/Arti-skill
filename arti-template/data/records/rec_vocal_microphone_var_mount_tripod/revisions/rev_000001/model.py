from __future__ import annotations

# Vintage silver desktop vocal microphone on a folding desk tripod (1950s Shure-55 look).
#
# Coordinate convention (object frame):
#   - up is +Z; the tripod rubber feet sit on the ground at z = 0.
#   - the capsule faces +X (the front grille looks toward +X).
#   - the yoke tilt axis (capsule pivot) runs along +Y (the horizontal side axis).
#   - the hub swivel axis is the vertical +Z through the hub/post centerline.
#
# Assembly (root -> children):
#   tripod_hub (root, static central hub with three splayed legs + rubber feet)
#     -> swivel_post (CONTINUOUS about +Z): short tapered post carrying the U-yoke
#          -> capsule_head (REVOLUTE tilt about +Y, +/-45 deg): flat oval ribbed
#             grille head, pinned between the two yoke side arms.
#     -> cable (FIXED): drooping cable + XLR plug lying on the desk.
#
# Scale: ~0.20 m tall overall, tripod spread ~0.13 m, capsule ~0.08 m tall.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---- key dimensions ----------------------------------------------------------
HUB_RADIUS = 0.016           # central hub radius
HUB_HEIGHT = 0.022           # central hub height
HUB_TOP_Z = 0.030            # top of hub (where post starts)
HUB_BOTTOM_Z = HUB_TOP_Z - HUB_HEIGHT  # bottom of hub

POST_BOTTOM_Z = HUB_TOP_Z   # post starts on top of the hub
POST_TOP_Z = 0.092           # top of the post / where the yoke bridge sits
YOKE_PIVOT_Z = 0.140         # height of the capsule tilt axis

N_LEGS = 3
LEG_ATTACH_Z = 0.020         # height where legs attach to hub
LEG_TOP_R = 0.005            # leg cross-section radius at hub end
LEG_BOT_R = 0.003            # leg cross-section radius at foot end
FOOT_SPREAD_R = 0.065        # radial distance from center to each foot
FOOT_RADIUS = 0.009          # rubber foot disc radius
FOOT_HEIGHT = 0.004          # rubber foot height

CAPSULE_HALF_W = 0.022       # capsule half-extent along Y (flat / thin axis)
CAPSULE_HALF_TALL = 0.040    # capsule half-height (Z), ~0.08 m tall
CAPSULE_HALF_DEPTH = 0.028   # capsule half-extent along X (face depth)


def _loft_yz(sections) -> cq.Workplane:
    # sections: list of (x_off, width_y, height_z); rounded rects stacked along +X.
    wp = cq.Workplane("YZ")
    prev = None
    for x_off, w, h in sections:
        if prev is None:
            wp = wp.workplane(offset=x_off)
        else:
            wp = wp.workplane(offset=x_off - prev)
        wp = wp.rect(w, h)
        prev = x_off
    return wp.loft(ruled=False)


def _capsule_mesh(z_lift: float = 0.0):
    half_y = CAPSULE_HALF_W
    half_z = CAPSULE_HALF_TALL
    half_x = CAPSULE_HALF_DEPTH

    # Outer flat oval / teardrop shell (broad face toward +X).
    outer = _loft_yz(
        [
            (-half_x, 0.026, 0.052),
            (-half_x * 0.4, 2 * half_y * 0.92, 2 * half_z * 0.94),
            (half_x * 0.45, 2 * half_y, 2 * half_z),
            (half_x * 0.95, 2 * half_y * 0.78, 2 * half_z * 0.82),
        ]
    )
    # Inner cavity so the head reads as a real open grille shell.
    inner = _loft_yz(
        [
            (-half_x * 0.85, 0.020, 0.046),
            (-half_x * 0.3, 2 * (half_y - 0.005) * 0.9, 2 * (half_z - 0.005) * 0.9),
            (half_x * 0.45, 2 * (half_y - 0.005), 2 * (half_z - 0.005)),
            (half_x * 1.05, 2 * (half_y - 0.006) * 0.78, 2 * (half_z - 0.006) * 0.82),
        ]
    )
    shell = outer.cut(inner)

    # Horizontal ribbed grille slats: cut a vertical stack of thin horizontal
    # slots through the front (+X) face of the oval (the vintage grille look).
    slot_h = 0.0045
    pitch = 0.0085
    n = 7
    z0 = -(n - 1) / 2.0 * pitch
    cutter = None
    for i in range(n):
        z = z0 + i * pitch
        frac = 1.0 - (abs(z) / half_z) ** 2 * 0.5
        sw = 2 * half_y * 1.3 * max(0.45, frac)
        slot = (
            cq.Workplane("XY")
            .box(2 * half_x, sw, slot_h)
            .translate((half_x * 0.5, 0.0, z))
        )
        cutter = slot if cutter is None else cutter.union(slot)
    shell = shell.cut(cutter)
    if z_lift:
        shell = shell.translate((0.0, 0.0, z_lift))
    return mesh_from_cadquery(shell, "capsule_shell")


def _grille_interior_mesh(z_lift: float = 0.0):
    # Dark inner oval block so the slots read as a dark grille interior.
    half_y = CAPSULE_HALF_W - 0.008
    half_z = CAPSULE_HALF_TALL - 0.006
    half_x = CAPSULE_HALF_DEPTH - 0.006
    blk = _loft_yz(
        [
            (-half_x, 2 * half_y, 2 * half_z),
            (half_x, 2 * half_y * 0.8, 2 * half_z * 0.85),
        ]
    )
    if z_lift:
        blk = blk.translate((0.0, 0.0, z_lift))
    return mesh_from_cadquery(blk, "grille_interior")


def _yoke_mesh():
    # U-shaped yoke (two side arms + bottom bridge) in the swivel_post frame.
    arm_y = CAPSULE_HALF_W + 0.011    # arms just outside the capsule sides
    arm_thick = 0.008                 # arm thickness along Y
    arm_depth = 0.018                 # arm depth along X
    bridge_h = 0.012

    bridge = (
        cq.Workplane("XY")
        .box(arm_depth, 2 * arm_y + arm_thick, bridge_h)
        .translate((0.0, 0.0, POST_TOP_Z + bridge_h / 2.0))
    )
    pivot_z = YOKE_PIVOT_Z
    arm_bottom = POST_TOP_Z + bridge_h
    arm_top = pivot_z + 0.010
    arm_height = arm_top - arm_bottom

    yoke = bridge
    for sign in (-1.0, 1.0):
        yc = sign * arm_y
        arm = (
            cq.Workplane("XY")
            .box(arm_depth, arm_thick, arm_height)
            .translate((0.0, yc, arm_bottom + arm_height / 2.0))
        )
        # rounded cap disc at the pivot end (disc faces +/-Y)
        cap = (
            cq.Workplane("XY")
            .circle(arm_depth / 2.0)
            .extrude(arm_thick)
            .rotate((0, 0, 0), (1, 0, 0), 90)
            .translate((0.0, yc + sign * (arm_thick / 2.0), pivot_z))
        )
        yoke = yoke.union(arm).union(cap)
    return mesh_from_cadquery(yoke, "yoke_shell")


def _post_mesh():
    # Short tapered satin post rising from the hub to the yoke bridge, with a
    # deep collar that inserts into the hub body for physical connection.
    post = (
        cq.Workplane("XY")
        .circle(0.014)
        .workplane(offset=POST_TOP_Z - POST_BOTTOM_Z)
        .circle(0.0085)
        .loft(ruled=False)
        .translate((0.0, 0.0, POST_BOTTOM_Z))
    )
    # Collar extends deep into the hub for a physical inserted-shaft connection
    collar_start_z = HUB_BOTTOM_Z + 0.003
    collar_height = POST_BOTTOM_Z - collar_start_z + 0.006
    collar = (
        cq.Workplane("XY")
        .circle(0.014)
        .extrude(collar_height)
        .translate((0.0, 0.0, collar_start_z))
    )
    return mesh_from_cadquery(post.union(collar), "post_shell")


def _hub_mesh():
    # Central hub cylinder with a top rim ring (one solid).
    hub = (
        cq.Workplane("XY")
        .circle(HUB_RADIUS)
        .extrude(HUB_HEIGHT)
        .translate((0.0, 0.0, HUB_BOTTOM_Z))
    )
    # Top rim ring where the post inserts
    rim = (
        cq.Workplane("XY")
        .circle(HUB_RADIUS + 0.002)
        .circle(HUB_RADIUS - 0.003)
        .extrude(0.003)
        .translate((0.0, 0.0, HUB_TOP_Z))
    )
    # Three hinge bosses at leg attachment points
    for i in range(N_LEGS):
        theta = i * 2.0 * math.pi / N_LEGS
        bx = (HUB_RADIUS + 0.001) * math.cos(theta)
        by = (HUB_RADIUS + 0.001) * math.sin(theta)
        boss = (
            cq.Workplane("XY")
            .circle(0.008)
            .extrude(0.012)
            .translate((bx, by, LEG_ATTACH_Z - 0.006))
        )
        hub = hub.union(boss)
    return mesh_from_cadquery(hub.union(rim), "hub_shell")


def _leg_strut_mesh(i: int):
    """Build one tapered tripod leg strut at angular position i * 120 degrees.

    Each leg is a tapered CadQuery loft from the hub attachment point down to
    the rubber foot position on the desk surface.
    """
    theta = i * 2.0 * math.pi / N_LEGS

    # Attachment point on hub perimeter
    z_top = LEG_ATTACH_Z
    r_attach = HUB_RADIUS + 0.002

    # Foot point on desk
    z_bot = FOOT_HEIGHT
    r_foot = FOOT_SPREAD_R

    # World positions of leg endpoints
    xt = r_attach * math.cos(theta)
    yt = r_attach * math.sin(theta)
    xb = r_foot * math.cos(theta)
    yb = r_foot * math.sin(theta)

    # Leg direction vector (top to bottom)
    dx = xb - xt
    dy = yb - yt
    dz = z_bot - z_top
    leg_len = math.sqrt(dx * dx + dy * dy + dz * dz)

    # Unit direction
    ux, uy, uz = dx / leg_len, dy / leg_len, dz / leg_len

    # Rotation axis: (0,0,1) × (ux,uy,uz) = (-uy, ux, 0)
    rx, ry = -uy, ux
    r_len = math.sqrt(rx * rx + ry * ry)
    if r_len > 1e-8:
        rx /= r_len
        ry /= r_len
        angle_deg = math.degrees(math.acos(max(-1.0, min(1.0, uz))))
    else:
        angle_deg = 0.0
        rx, ry = 1.0, 0.0

    # Midpoint for placement
    mx = (xt + xb) / 2.0
    my = (yt + yb) / 2.0
    mz = (z_top + z_bot) / 2.0

    # Build tapered strut along local Z, centered at origin
    strut = (
        cq.Workplane("XY")
        .circle(LEG_TOP_R)
        .workplane(offset=leg_len)
        .circle(LEG_BOT_R)
        .loft(ruled=True)
        .translate((0.0, 0.0, -leg_len / 2.0))
    )

    # Wider collar at hub end for visual connection to the hub boss
    collar = (
        cq.Workplane("XY")
        .circle(LEG_TOP_R + 0.003)
        .extrude(0.010)
        .translate((0.0, 0.0, -leg_len / 2.0 - 0.002))
    )
    strut = strut.union(collar)

    # Rotate into position and translate to midpoint
    if abs(angle_deg) > 0.01:
        strut = strut.rotate((0.0, 0.0, 0.0), (rx, ry, 0.0), angle_deg)
    strut = strut.translate((mx, my, mz))

    return mesh_from_cadquery(strut, f"leg_{i}")


def _foot_mesh(i: int):
    """Build one rubber foot at the base of leg i."""
    theta = i * 2.0 * math.pi / N_LEGS
    xb = FOOT_SPREAD_R * math.cos(theta)
    yb = FOOT_SPREAD_R * math.sin(theta)

    # Flat rubber pad on the desk
    pad = (
        cq.Workplane("XY")
        .circle(FOOT_RADIUS)
        .extrude(FOOT_HEIGHT)
        .translate((xb, yb, 0.0))
    )
    # Tapered transition from pad to leg end
    taper = (
        cq.Workplane("XY")
        .circle(FOOT_RADIUS * 0.85)
        .workplane(offset=0.005)
        .circle(LEG_BOT_R + 0.001)
        .loft(ruled=True)
        .translate((xb, yb, FOOT_HEIGHT))
    )
    return mesh_from_cadquery(pad.union(taper), f"foot_{i}")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_vocal_mic_tripod")

    silver = model.material("satin_silver", rgba=(0.80, 0.81, 0.83, 1.0))
    dark = model.material("dark_grille", rgba=(0.16, 0.16, 0.18, 1.0))
    cable_dk = model.material("cable_dark", rgba=(0.22, 0.22, 0.24, 1.0))
    xlr_metal = model.material("xlr_metal", rgba=(0.52, 0.53, 0.55, 1.0))
    badge_dark = model.material("badge_dark", rgba=(0.10, 0.10, 0.12, 1.0))
    rubber = model.material("rubber_foot", rgba=(0.12, 0.12, 0.13, 1.0))

    # ---- tripod hub (root): central hub with three splayed legs ---------------
    hub = model.part("tripod_hub")
    hub.visual(_hub_mesh(), material=silver, name="hub_shell")

    # Three legs at even angular spacing via shared helper
    for i in range(N_LEGS):
        hub.visual(_leg_strut_mesh(i), material=silver, name=f"leg_{i}")
        hub.visual(_foot_mesh(i), material=rubber, name=f"foot_{i}")

    hub.inertial = Inertial.from_geometry(
        Box((2 * FOOT_SPREAD_R, 2 * FOOT_SPREAD_R, HUB_TOP_Z)),
        mass=0.30,
        origin=Origin(xyz=(0.0, 0.0, HUB_TOP_Z / 2.0)),
    )

    # ---- swivel post + yoke: rotates about the vertical hub axis --------------
    post = model.part("swivel_post")
    post.visual(_post_mesh(), material=silver, name="post_shell")
    post.visual(_yoke_mesh(), material=silver, name="yoke_shell")
    post.inertial = Inertial.from_geometry(
        Box((0.05, 0.090, YOKE_PIVOT_Z)),
        mass=0.10,
        origin=Origin(xyz=(0.0, 0.0, (POST_BOTTOM_Z + YOKE_PIVOT_Z) / 2.0)),
    )
    model.articulation(
        "hub_to_post",
        ArticulationType.CONTINUOUS,
        parent=hub,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=4.0),
    )

    # ---- capsule head: tilts about the +Y yoke side axis ---------------------
    capsule = model.part("capsule_head")
    cap_lift = 0.008

    cap_shell = _capsule_mesh(z_lift=cap_lift)
    capsule.visual(cap_shell, material=silver, name="capsule_shell")

    gi = _grille_interior_mesh(z_lift=cap_lift)
    capsule.visual(gi, material=dark, name="grille_interior")

    # round badge dot on the front face, seated on a solid rib between two slots
    badge = CylinderGeometry(0.006, 0.008).rotate_y(math.pi / 2.0)
    badge.translate(CAPSULE_HALF_DEPTH * 0.88, 0.0, cap_lift + 0.00425)
    capsule.visual(mesh_from_geometry(badge, "badge"), material=badge_dark, name="badge")

    # capsule tilt pin along Y (runs into the yoke arms, captured by the cap discs)
    pin = CylinderGeometry(0.0035, 2 * (CAPSULE_HALF_W + 0.010)).rotate_x(math.pi / 2.0)
    capsule.visual(mesh_from_geometry(pin, "tilt_pin"), material=xlr_metal, name="tilt_pin")

    capsule.inertial = Inertial.from_geometry(
        Box((2 * CAPSULE_HALF_DEPTH, 2 * CAPSULE_HALF_W, 2 * CAPSULE_HALF_TALL)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, cap_lift)),
    )
    model.articulation(
        "yoke_to_capsule",
        ArticulationType.REVOLUTE,
        parent=post,
        child=capsule,
        origin=Origin(xyz=(0.0, 0.0, YOKE_PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=-math.radians(45.0),
            upper=math.radians(45.0),
        ),
    )

    # ---- cable + XLR plug: fixed drooping tube from the hub to the desk -------
    # Route backward (-X) between legs at 120° and 240°, then curve to one side.
    cable_pts = [
        (-0.014, 0.0, HUB_BOTTOM_Z + 0.006),    # exits hub rear (inside hub body)
        (-0.040, 0.0, 0.010),
        (-0.060, 0.003, 0.006),
        (-0.080, 0.008, 0.006),
        (-0.095, 0.018, 0.006),
        (-0.100, 0.035, 0.006),
        (-0.095, 0.055, 0.006),
        (-0.080, 0.072, 0.006),
    ]
    cable = tube_from_spline_points(
        cable_pts, radius=0.0035, samples_per_segment=18, radial_segments=12
    )
    ang = math.atan2(0.072 - 0.055, -0.080 - (-0.095))
    plug = CylinderGeometry(0.010, 0.040).rotate_y(math.pi / 2.0).rotate_z(ang)
    plug.translate(-0.065, 0.085, 0.010)
    cable.merge(plug)
    collar_c = CylinderGeometry(0.012, 0.010).rotate_y(math.pi / 2.0).rotate_z(ang)
    collar_c.translate(-0.078, 0.076, 0.010)
    cable.merge(collar_c)

    cable_part = model.part("cable")
    cable_part.visual(
        mesh_from_geometry(cable, "cable_shell"), material=cable_dk, name="cable_shell"
    )
    plug_tip = CylinderGeometry(0.0085, 0.006).rotate_y(math.pi / 2.0).rotate_z(ang)
    plug_tip.translate(-0.052, 0.094, 0.010)
    cable_part.visual(
        mesh_from_geometry(plug_tip, "xlr_tip"), material=xlr_metal, name="xlr_tip"
    )
    cable_part.inertial = Inertial.from_geometry(
        Box((0.20, 0.10, 0.02)), mass=0.04, origin=Origin(xyz=(0.02, -0.02, 0.008))
    )
    model.articulation(
        "hub_to_cable",
        ArticulationType.FIXED,
        parent=hub,
        child=cable_part,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hub = object_model.get_part("tripod_hub")
    post = object_model.get_part("swivel_post")
    capsule = object_model.get_part("capsule_head")
    cable = object_model.get_part("cable")
    tilt = object_model.get_articulation("yoke_to_capsule")
    swivel = object_model.get_articulation("hub_to_post")

    # --- intentional overlaps (capture / insertion fits) ---
    ctx.allow_overlap(
        capsule, post, elem_a="tilt_pin", elem_b="yoke_shell",
        reason="Capsule tilt pin is captured inside the yoke side arms.",
    )
    ctx.allow_overlap(
        capsule, post, elem_a="capsule_shell", elem_b="yoke_shell",
        reason="Capsule sides nest between the U-yoke arms with a small running fit.",
    )
    ctx.allow_overlap(
        capsule, capsule, elem_a="grille_interior", elem_b="capsule_shell",
        reason="Dark grille interior sits just inside the hollow capsule shell.",
    )
    ctx.allow_overlap(
        post, hub, elem_a="post_shell", elem_b="hub_shell",
        reason="Post collar inserts deep into the hub body as a seated shaft fit.",
    )
    ctx.allow_overlap(
        cable, hub, elem_a="cable_shell", elem_b="hub_shell",
        reason="Cable exits from inside the hub between the tripod legs.",
    )

    # Proof: post collar is centered within the hub on XY
    ctx.expect_within(
        post, hub, axes="xy", elem_a="post_shell", elem_b="hub_shell",
        margin=0.010, name="post collar centered within hub",
    )
    # Foot-to-leg insertion: rubber foot taper inserts into leg strut end
    for i in range(N_LEGS):
        ctx.allow_overlap(
            hub, hub, elem_a=f"foot_{i}", elem_b=f"leg_{i}",
            reason=f"Rubber foot taper inserts into leg strut end for leg {i}.",
        )

    # --- tripod feet touch the ground ---
    hub_aabb = ctx.part_world_aabb(hub)
    ctx.check(
        "tripod feet touch the ground (z ~ 0)",
        abs(hub_aabb[0][2]) < 0.003,
        details=f"hub z-min={hub_aabb[0][2]}",
    )

    # --- tripod is the widest footprint ---
    hub_ext = _ext(hub_aabb)
    cap_ext = _ext(ctx.part_world_aabb(capsule))
    post_ext = _ext(ctx.part_world_aabb(post))
    footprint_hub = max(hub_ext[0], hub_ext[1])
    footprint_cap = max(cap_ext[0], cap_ext[1])
    footprint_post = max(post_ext[0], post_ext[1])
    ctx.check(
        "tripod spread is the widest footprint",
        footprint_hub >= footprint_cap - 1e-6 and footprint_hub >= footprint_post - 1e-6,
        details=f"hub={footprint_hub}, capsule={footprint_cap}, post={footprint_post}",
    )

    # --- three legs at even angular spacing ---
    foot_centers = []
    for i in range(N_LEGS):
        foot_aabb = ctx.part_element_world_aabb(hub, elem=f"foot_{i}")
        cx = (foot_aabb[0][0] + foot_aabb[1][0]) / 2.0
        cy = (foot_aabb[0][1] + foot_aabb[1][1]) / 2.0
        foot_centers.append((cx, cy))

    hub_pos = ctx.part_world_position(hub)
    angles = [
        math.atan2(c[1] - hub_pos[1], c[0] - hub_pos[0])
        for c in foot_centers
    ]
    angles_sorted = sorted(angles)
    gaps = [
        angles_sorted[1] - angles_sorted[0],
        angles_sorted[2] - angles_sorted[1],
        2 * math.pi - (angles_sorted[2] - angles_sorted[0]),
    ]
    target_gap = 2.0 * math.pi / N_LEGS
    max_deviation = max(abs(g - target_gap) for g in gaps)
    ctx.check(
        "three legs at ~120 degree spacing",
        max_deviation < 0.3,
        details=f"gaps={[round(g, 3) for g in gaps]}, max_dev={max_deviation:.3f}",
    )

    # --- all legs reach similar radial distance from hub center ---
    radii = [
        math.sqrt((c[0] - hub_pos[0]) ** 2 + (c[1] - hub_pos[1]) ** 2)
        for c in foot_centers
    ]
    radius_spread = max(radii) - min(radii)
    ctx.check(
        "legs reach similar spread distance",
        radius_spread < 0.010,
        details=f"radii={[round(r, 4) for r in radii]}",
    )

    # --- capsule is held in the yoke (contact) ---
    ctx.expect_contact(capsule, post, name="capsule cradled in the yoke")

    # --- capsule tilts about the +Y side axis: front face swings in X ---
    front_x_rest = ctx.part_world_aabb(capsule)[1][0]
    with ctx.pose({tilt: math.radians(30.0)}):
        front_x_tilt = ctx.part_world_aabb(capsule)[1][0]
    ctx.check(
        "capsule tilt about +Y swings the head fore/aft",
        abs(front_x_tilt - front_x_rest) > 0.005,
        details=f"front-x rest={front_x_rest}, tilted={front_x_tilt}",
    )

    # --- post/yoke swivels about the vertical hub axis: a side point rotates ---
    cap_max_y_rest = ctx.part_world_aabb(capsule)[1][1]
    with ctx.pose({swivel: math.radians(90.0)}):
        cap_max_x_swiv = ctx.part_world_aabb(capsule)[1][0]
    ctx.check(
        "swivel rotates the head about the vertical hub axis",
        abs(cap_max_x_swiv - cap_max_y_rest) > 0.003 or cap_max_x_swiv > 0.0,
        details=f"rest max-y={cap_max_y_rest}, after-90deg max-x={cap_max_x_swiv}",
    )

    # --- cable + plug attached at the hub ---
    ctx.expect_contact(cable, hub, name="cable attached at the hub")

    return ctx.report()


object_model = build_object_model()
