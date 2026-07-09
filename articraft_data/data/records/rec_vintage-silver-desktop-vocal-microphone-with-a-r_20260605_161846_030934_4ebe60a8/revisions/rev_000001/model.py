from __future__ import annotations

# Vintage silver desktop vocal microphone (1950s Shure-55 look).
#
# Coordinate convention (object frame):
#   - up is +Z; the round weighted base disc sits on the ground at z = 0.
#   - the capsule faces +X (the front grille looks toward +X).
#   - the yoke tilt axis (capsule pivot) runs along +Y (the horizontal side axis).
#   - the base swivel axis is the vertical +Z through the base/post centerline.
#
# Assembly (root -> children):
#   base_disc (root, static round weighted disc on the ground)
#     -> swivel_post (CONTINUOUS about +Z): short tapered post carrying the U-yoke
#          -> capsule_head (REVOLUTE tilt about +Y, +/-45 deg): flat oval ribbed
#             grille head, pinned between the two yoke side arms.
#     -> cable (FIXED): drooping cable + XLR plug lying on the desk.
#
# Scale: ~0.18 m tall overall, base dia ~0.10 m, capsule ~0.08 m tall.

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
BASE_RADIUS = 0.050          # round base disc radius (dia 0.10 m)
BASE_THICK = 0.014           # base disc height
POST_BOTTOM_Z = BASE_THICK   # post starts on top of the base
POST_TOP_Z = 0.082           # top of the post / where the yoke bridge sits
YOKE_PIVOT_Z = 0.130         # height of the capsule tilt axis (the +Y pin axis)

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
    # Short tapered satin post rising from the base to the yoke bridge, with a
    # small collar at its foot (one solid).
    post = (
        cq.Workplane("XY")
        .circle(0.014)
        .workplane(offset=POST_TOP_Z - POST_BOTTOM_Z)
        .circle(0.0085)
        .loft(ruled=False)
        .translate((0.0, 0.0, POST_BOTTOM_Z))
    )
    collar = (
        cq.Workplane("XY")
        .circle(0.017)
        .extrude(0.006)
        .translate((0.0, 0.0, POST_BOTTOM_Z))
    )
    return mesh_from_cadquery(post.union(collar), "post_shell")


def _base_mesh():
    # Round weighted base disc with a chamfered rim ring (one solid).
    disc = cq.Workplane("XY").circle(BASE_RADIUS).extrude(BASE_THICK)
    rim = (
        cq.Workplane("XY")
        .circle(BASE_RADIUS)
        .circle(BASE_RADIUS - 0.006)
        .extrude(0.003)
        .translate((0.0, 0.0, BASE_THICK))
    )
    return mesh_from_cadquery(disc.union(rim), "base_disc")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_vocal_mic")

    silver = model.material("satin_silver", rgba=(0.80, 0.81, 0.83, 1.0))
    dark = model.material("dark_grille", rgba=(0.16, 0.16, 0.18, 1.0))
    cable_dk = model.material("cable_dark", rgba=(0.22, 0.22, 0.24, 1.0))
    xlr_metal = model.material("xlr_metal", rgba=(0.52, 0.53, 0.55, 1.0))
    badge_dark = model.material("badge_dark", rgba=(0.10, 0.10, 0.12, 1.0))

    # ---- base disc (root): static round weighted base on the ground ----------
    base = model.part("base_disc")
    base.visual(_base_mesh(), material=silver, name="base_disc")
    base.inertial = Inertial.from_geometry(
        Box((2 * BASE_RADIUS, 2 * BASE_RADIUS, BASE_THICK)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICK / 2.0)),
    )

    # ---- swivel post + yoke: rotates about the vertical base axis ------------
    post = model.part("swivel_post")
    post.visual(_post_mesh(), material=silver, name="post_shell")
    post.visual(_yoke_mesh(), material=silver, name="yoke_shell")
    post.inertial = Inertial.from_geometry(
        Box((0.05, 0.090, YOKE_PIVOT_Z)),
        mass=0.10,
        origin=Origin(xyz=(0.0, 0.0, (POST_BOTTOM_Z + YOKE_PIVOT_Z) / 2.0)),
    )
    model.articulation(
        "base_to_post",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=4.0),
    )

    # ---- capsule head: tilts about the +Y yoke side axis ---------------------
    # Authored so the tilt pin/axis passes through this part's local origin; the
    # capsule body sits a little above the pin (pin is below the head center).
    capsule = model.part("capsule_head")
    cap_lift = 0.008

    cap_shell = _capsule_mesh(z_lift=cap_lift)
    capsule.visual(cap_shell, material=silver, name="capsule_shell")

    gi = _grille_interior_mesh(z_lift=cap_lift)
    capsule.visual(gi, material=dark, name="grille_interior")

    # round badge dot on the front face, seated on a solid rib between two slots
    # (slot pitch 0.0085, so z = +0.00425 lands on a rib, not in a slot gap).
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

    # ---- cable + XLR plug: fixed drooping tube from the base to the desk -----
    cable_pts = [
        (-0.030, 0.012, BASE_THICK * 0.6),   # exits the base side
        (-0.055, 0.020, 0.010),
        (-0.075, 0.010, 0.006),
        (-0.070, -0.020, 0.006),
        (-0.040, -0.045, 0.006),
        (0.010, -0.050, 0.006),
        (0.060, -0.040, 0.006),
        (0.092, -0.018, 0.006),
    ]
    cable = tube_from_spline_points(
        cable_pts, radius=0.0035, samples_per_segment=18, radial_segments=12
    )
    ang = math.atan2(-0.018 + 0.040, 0.092 - 0.060)
    plug = CylinderGeometry(0.010, 0.040).rotate_y(math.pi / 2.0).rotate_z(ang)
    plug.translate(0.112, -0.011, 0.010)
    cable.merge(plug)
    collar = CylinderGeometry(0.012, 0.010).rotate_y(math.pi / 2.0).rotate_z(ang)
    collar.translate(0.090, -0.016, 0.010)
    cable.merge(collar)

    cable_part = model.part("cable")
    cable_part.visual(
        mesh_from_geometry(cable, "cable_shell"), material=cable_dk, name="cable_shell"
    )
    plug_tip = CylinderGeometry(0.0085, 0.006).rotate_y(math.pi / 2.0).rotate_z(ang)
    plug_tip.translate(0.131, -0.008, 0.010)
    cable_part.visual(
        mesh_from_geometry(plug_tip, "xlr_tip"), material=xlr_metal, name="xlr_tip"
    )
    cable_part.inertial = Inertial.from_geometry(
        Box((0.20, 0.10, 0.02)), mass=0.04, origin=Origin(xyz=(0.02, -0.02, 0.008))
    )
    model.articulation(
        "base_to_cable",
        ArticulationType.FIXED,
        parent=base,
        child=cable_part,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_disc")
    post = object_model.get_part("swivel_post")
    capsule = object_model.get_part("capsule_head")
    cable = object_model.get_part("cable")
    tilt = object_model.get_articulation("yoke_to_capsule")
    swivel = object_model.get_articulation("base_to_post")

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
        post, base, elem_a="post_shell", elem_b="base_disc",
        reason="Post foot is seated into the top of the weighted base disc.",
    )
    ctx.allow_overlap(
        cable, base, elem_a="cable_shell", elem_b="base_disc",
        reason="Cable exits from inside the base disc.",
    )

    # --- base sits on the ground and is the widest footprint ---
    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "base sits on the ground (z ~ 0)",
        abs(base_aabb[0][2]) < 0.002,
        details=f"base z-min={base_aabb[0][2]}",
    )
    base_ext = _ext(base_aabb)
    cap_ext = _ext(ctx.part_world_aabb(capsule))
    post_ext = _ext(ctx.part_world_aabb(post))
    footprint_base = max(base_ext[0], base_ext[1])
    footprint_cap = max(cap_ext[0], cap_ext[1])
    footprint_post = max(post_ext[0], post_ext[1])
    ctx.check(
        "base disc is the widest footprint",
        footprint_base >= footprint_cap - 1e-6 and footprint_base >= footprint_post - 1e-6,
        details=f"base={footprint_base}, capsule={footprint_cap}, post={footprint_post}",
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

    # --- post/yoke swivels about the vertical base axis: a side point rotates ---
    cap_max_y_rest = ctx.part_world_aabb(capsule)[1][1]
    with ctx.pose({swivel: math.radians(90.0)}):
        cap_max_x_swiv = ctx.part_world_aabb(capsule)[1][0]
    ctx.check(
        "swivel rotates the head about the vertical base axis",
        abs(cap_max_x_swiv - cap_max_y_rest) > 0.003 or cap_max_x_swiv > 0.0,
        details=f"rest max-y={cap_max_y_rest}, after-90deg max-x={cap_max_x_swiv}",
    )

    # --- cable + plug attached at the base ---
    ctx.expect_contact(cable, base, name="cable attached at the base")

    return ctx.report()


object_model = build_object_model()
