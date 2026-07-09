from __future__ import annotations

# Ultrawide curved computer monitor on a central stand.
# Frame:
#   +X = panel width (long axis), +Z = up, -Y = viewer side (the screen faces -Y).
#   Panel centerline at z=0; the curved (concave) screen wraps so its mid is
#   pushed toward the viewer (-Y) relative to the chord at the wings.
# Kinematic chain (root -> ... -> leaf):
#   base_foot (root, wide V/T metal foot on the ground)
#     -> swivel (vertical Z axis at base): neck_riser
#       -> height (prismatic Z, optional): neck_carriage
#         -> tilt (horizontal X axis at the neck): screen_panel
# Articulations:
#   - swivel  : REVOLUTE about vertical Z at the base (~ +-30 deg)
#   - height  : PRISMATIC up/down on the neck (~0.08 m)
#   - tilt    : REVOLUTE about horizontal X at the neck (~ -5..+15 deg)

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- panel geometry constants ----
# 21:9 curved ultrawide (~0.62 / 0.27 ≈ 2.30 aspect ratio)
PANEL_W = 0.62  # full width (X)
PANEL_H = 0.27  # full height (Z)
PANEL_T = 0.022  # housing thickness front-to-back at the thick rear
BEZEL = 0.010  # bezel width framing the screen
BOW = 0.025  # concave depth bow (mid pushed -Y relative to wings, gentler than super-ultrawide)

SCREEN_Y_FRONT = 0.050  # front face chord offset; positive = further from stand (more -Y)
PANEL_Z_OFFSET = 0.050  # shift panel geometry up so tilt pivot sits at the lower portion


def _arc_y(x: float) -> float:
    """Concave screen offset in -Y as a function of width position x.

    Returns BOW at the center (x=0) and 0 at the wings (x=+-W/2), following a
    circular arc so the panel reads as a real concave curve.
    """
    half = PANEL_W / 2.0
    R = (half * half + BOW * BOW) / (2.0 * BOW)
    inside = R * R - x * x
    if inside < 0.0:
        inside = 0.0
    sag = R - math.sqrt(inside)  # 0 at center, BOW at wings
    return BOW - sag  # BOW at center, 0 at wings


def _curved_housing() -> cq.Workplane:
    """Curved panel rear housing: a thin slab bowed into a concave arc.

    Built as a loft through YZ rectangular sections placed along X so the whole
    slab follows the concave arc. Shifted up by PANEL_Z_OFFSET so the tilt
    pivot sits at the lower portion of the panel.
    """
    half_w = PANEL_W / 2.0
    half_h = PANEL_H / 2.0
    n = 21
    wp = cq.Workplane("YZ")
    prev_x = -half_w
    for i in range(n):
        x = -half_w + PANEL_W * i / (n - 1)
        arc = _arc_y(x)
        yf = -(SCREEN_Y_FRONT + arc)  # front (viewer side, -Y)
        yb = yf + PANEL_T  # back
        yc = (yf + yb) / 2.0
        off = x - prev_x if i > 0 else x
        wp = wp.workplane(offset=off)
        wp = wp.moveTo(yc, PANEL_Z_OFFSET).rect(abs(yb - yf), 2.0 * half_h)
        prev_x = x
    return wp.loft(ruled=True)


def _screen_face() -> cq.Workplane:
    """Dark glass screen: a thin concave shell inset by the bezel, sitting just
    in front of the housing. Lofted YZ sections along X. Shifted up by
    PANEL_Z_OFFSET to match the housing."""
    half_w = PANEL_W / 2.0 - BEZEL
    half_h = PANEL_H / 2.0 - BEZEL
    n = 21
    glass_t = 0.004
    wp = cq.Workplane("YZ")
    prev_x = -half_w
    for i in range(n):
        x = -half_w + (2.0 * half_w) * i / (n - 1)
        arc = _arc_y(x)
        yf = -(SCREEN_Y_FRONT + arc) - 0.0005  # front glass surface, just ahead
        yb = yf + glass_t
        yc = (yf + yb) / 2.0
        off = x - prev_x if i > 0 else x
        wp = wp.workplane(offset=off)
        wp = wp.moveTo(yc, PANEL_Z_OFFSET).rect(glass_t, 2.0 * half_h)
        prev_x = x
    return wp.loft(ruled=True)


def _neck_riser_solid() -> cq.Workplane:
    """Central neck riser column: a flattened tapered post rising from z=0."""
    post = (
        cq.Workplane("XY")
        .rect(0.060, 0.026)
        .workplane(offset=0.150)
        .rect(0.050, 0.024)
        .loft(ruled=True)
    )
    return post


def _base_foot_solid() -> cq.Workplane:
    """Wide V-shaped metal base foot lying flat on the ground (z in [0, arm_th]).

    Central hub plus two arms sweeping forward (-Y) and out (+/-X) into a V, with
    a small rear stub for stability.
    """
    arm_th = 0.020
    foot = cq.Workplane("XY").circle(0.045).extrude(arm_th)
    half_span = 0.16
    reach = 0.20
    for sign in (-1.0, 1.0):
        arm = (
            cq.Workplane("XY")
            .moveTo(sign * 0.020, 0.030)
            .lineTo(sign * 0.050, 0.030)
            .lineTo(sign * half_span, -reach)
            .lineTo(sign * (half_span - 0.045), -reach)
            .lineTo(sign * 0.010, -0.010)
            .close()
            .extrude(arm_th)
        )
        foot = foot.union(arm)
    rear = (
        cq.Workplane("XY")
        .moveTo(0.0, 0.030)
        .lineTo(0.030, 0.095)
        .lineTo(-0.030, 0.095)
        .close()
        .extrude(arm_th)
    )
    foot = foot.union(rear)
    return foot


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="curved_monitor")

    screen_mat = model.material("screen_dark", rgba=(0.03, 0.03, 0.05, 1.0))
    back_mat = model.material("housing_graphite", rgba=(0.18, 0.18, 0.20, 1.0))
    metal_mat = model.material("base_silver", rgba=(0.72, 0.73, 0.76, 1.0))
    neck_mat = model.material("neck_graphite", rgba=(0.24, 0.24, 0.26, 1.0))

    # ===================== base foot (root) =====================
    base = model.part("base_foot")
    foot = _base_foot_solid()
    base.visual(
        mesh_from_cadquery(foot, "base_foot_shell"), material=metal_mat, name="base_foot_shell"
    )
    base.inertial = Inertial.from_geometry(
        Box((0.34, 0.30, 0.020)), mass=1.6, origin=Origin(xyz=(0.0, -0.06, 0.010))
    )

    # ===================== neck riser (swivels about Z) =====================
    neck = model.part("neck_riser")
    post = _neck_riser_solid()
    neck.visual(
        mesh_from_cadquery(post, "neck_riser_shell"), material=neck_mat, name="neck_riser_shell"
    )
    hub = CylinderGeometry(0.038, 0.024).translate(0.0, 0.0, 0.012)
    neck.visual(mesh_from_geometry(hub, "swivel_hub"), material=metal_mat, name="swivel_hub")
    neck.inertial = Inertial.from_geometry(
        Box((0.06, 0.026, 0.16)), mass=0.5, origin=Origin(xyz=(0.0, 0.0, 0.08))
    )
    model.articulation(
        "base_to_neck_swivel",
        ArticulationType.REVOLUTE,
        parent=base,
        child=neck,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),  # at the top of the base foot
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=2.0, lower=-math.radians(30.0), upper=math.radians(30.0)
        ),
    )

    # ===================== height carriage (prismatic Z on neck) =====================
    carriage = model.part("neck_carriage")
    plate = BoxGeometry((0.052, 0.018, 0.060)).translate(0.0, -0.004, 0.0)
    carriage.visual(
        mesh_from_geometry(plate, "carriage_plate"), material=neck_mat, name="carriage_plate"
    )
    barrel = CylinderGeometry(0.014, 0.060, radial_segments=24).rotate_y(math.pi / 2.0)
    barrel.translate(0.0, -0.020, 0.0)
    carriage.visual(
        mesh_from_geometry(barrel, "tilt_barrel"), material=metal_mat, name="tilt_barrel"
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.052, 0.030, 0.060)), mass=0.20, origin=Origin(xyz=(0.0, -0.008, 0.0))
    )
    model.articulation(
        "neck_to_carriage_height",
        ArticulationType.PRISMATIC,
        parent=neck,
        child=carriage,
        origin=Origin(xyz=(0.0, -0.013, 0.108)),  # nominal mid-height seat on the neck
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.2, lower=-0.040, upper=0.040),
    )

    # ===================== screen panel (tilts about X) =====================
    panel = model.part("screen_panel")
    housing = _curved_housing()
    panel.visual(
        mesh_from_cadquery(housing, "panel_housing"), material=back_mat, name="panel_housing"
    )
    panel.visual(
        mesh_from_cadquery(_screen_face(), "screen_glass"),
        material=screen_mat,
        name="screen_glass",
    )
    # Rear VESA mount boss on the panel back center: a raised block the stand's
    # tilt hinge clamps onto. Bridges the concave panel back out to the barrel.
    # Housing back at center ~ y=-0.053 (with SCREEN_Y_FRONT=0.050); barrel
    # center ~ y=-0.033, z=0 in panel local. Boss spans both.
    boss = BoxGeometry((0.085, 0.044, 0.072)).translate(0.0, -0.033, 0.020)
    panel.visual(mesh_from_geometry(boss, "vesa_mount"), material=back_mat, name="vesa_mount")
    panel.inertial = Inertial.from_geometry(
        Box((PANEL_W, PANEL_T + BOW, PANEL_H)),
        mass=2.4,
        origin=Origin(xyz=(0.0, -(SCREEN_Y_FRONT + BOW / 2.0), PANEL_Z_OFFSET)),
    )
    # Tilt hinge: horizontal X axis at the neck, panel-center height. At q=0 the
    # panel hangs vertical; positive q leans the top backward (+Y).
    model.articulation(
        "carriage_to_screen_tilt",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=panel,
        origin=Origin(xyz=(0.0, -0.014, 0.0)),  # at the tilt barrel
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0, velocity=2.0, lower=-math.radians(5.0), upper=math.radians(15.0)
        ),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_foot")
    neck = object_model.get_part("neck_riser")
    panel = object_model.get_part("screen_panel")

    swivel = object_model.get_articulation("base_to_neck_swivel")
    height = object_model.get_articulation("neck_to_carriage_height")
    tilt = object_model.get_articulation("carriage_to_screen_tilt")

    # ---- Intentional seated overlaps (no hidden wrong origins) ----
    ctx.allow_overlap(
        neck, base,
        elem_a="swivel_hub", elem_b="base_foot_shell",
        reason="Swivel hub puck is seated into the base foot hub (rotary capture).",
    )
    ctx.allow_overlap(
        object_model.get_part("neck_carriage"), neck,
        elem_a="carriage_plate", elem_b="neck_riser_shell",
        reason="Height carriage rides on/around the neck post (prismatic slide proxy).",
    )
    ctx.allow_overlap(
        panel, object_model.get_part("neck_carriage"),
        elem_a="vesa_mount", elem_b="tilt_barrel",
        reason="Rear VESA mount boss seats over the tilt hinge barrel on the carriage.",
    )

    carriage = object_model.get_part("neck_carriage")
    # Panel is mounted to the carriage via the VESA boss over the tilt barrel.
    ctx.expect_contact(
        panel, carriage, elem_a="vesa_mount", elem_b="tilt_barrel",
        name="panel VESA boss seated on tilt barrel",
    )

    # ---- Panel is the hero: 21:9 ultrawide (wider than tall, thin, and concave) ----
    pext = _ext(ctx.part_world_aabb(panel))
    ctx.check(
        "panel is ultrawide (width >> height)",
        pext[0] > 0.55 and pext[0] > 2.0 * pext[2],
        details=f"panel extents (x,y,z)={pext}",
    )
    # Specific check for the 21:9 variant: aspect ratio should be ~2.3 (not ~3.4)
    aspect_ratio = pext[0] / pext[2] if pext[2] > 0.01 else 0.0
    ctx.check(
        "screen_panel is 21:9 aspect ratio (~2.3:1)",
        2.0 < aspect_ratio < 2.6,
        details=f"aspect ratio={aspect_ratio:.2f}, expect ~2.3 for 21:9",
    )
    ctx.check(
        "panel is thin in depth",
        pext[1] < 0.12,
        details=f"panel depth (y)={pext[1]:.4f}",
    )

    # Verify the concave curvature: the screen mid bows toward the viewer (-Y).
    glass_aabb = ctx.part_element_world_aabb(panel, elem="screen_glass")
    g_mn, _g_mx = glass_aabb
    ctx.check(
        "screen is concave (mid bows toward viewer)",
        g_mn[1] < -(BOW * 0.6),
        details=f"glass min-Y={g_mn[1]:.4f} (expect < {-(BOW * 0.6):.4f} for ~{BOW} bow)",
    )

    # ---- Stacking / connectivity: panel above base, neck between ----
    base_pos = ctx.part_world_position(base)
    panel_pos = ctx.part_world_position(panel)
    ctx.check(
        "panel sits above the base on the stand",
        panel_pos[2] > base_pos[2] + 0.10,
        details=f"base_z={base_pos[2]:.3f}, panel_z={panel_pos[2]:.3f}",
    )

    # ---- Base is the widest footprint on the ground ----
    base_aabb = ctx.part_world_aabb(base)
    b_mn, _b_mx = base_aabb
    ctx.check(
        "base foot rests on the ground (z ~ 0)",
        abs(b_mn[2]) < 0.01,
        details=f"base min-Z={b_mn[2]:.4f}",
    )
    base_ext = _ext(base_aabb)
    n_ext = _ext(ctx.part_world_aabb(neck))
    ctx.check(
        "base footprint is wider/deeper than the neck",
        base_ext[0] > n_ext[0] and base_ext[1] > n_ext[1],
        details=f"base xy=({base_ext[0]:.3f},{base_ext[1]:.3f}) neck xy=({n_ext[0]:.3f},{n_ext[1]:.3f})",
    )

    # ---- Tilt: revolve about horizontal X at the neck -> front face sweeps in Y ----
    front_y_rest = ctx.part_world_aabb(panel)[0][1]  # min Y at rest (screen front)
    with ctx.pose({tilt: math.radians(15.0)}):
        tilted = ctx.part_world_aabb(panel)
    front_y_tilt = tilted[0][1]
    ctx.check(
        "tilt rotates the panel about the horizontal axis",
        abs(front_y_tilt - front_y_rest) > 0.01,
        details=f"front-Y rest={front_y_rest:.4f}, tilted={front_y_tilt:.4f}",
    )
    ctx.check(
        "tilt axis is horizontal (X) at the neck",
        abs(tilt.axis[0]) > 0.9 and abs(tilt.axis[2]) < 0.1,
        details=f"tilt axis={tilt.axis}",
    )

    # ---- Swivel: revolve about vertical Z at the base -> panel rotates in XY ----
    rest_xext = _ext(ctx.part_world_aabb(panel))[0]
    with ctx.pose({swivel: math.radians(30.0)}):
        sw_xext = _ext(ctx.part_world_aabb(panel))[0]
    ctx.check(
        "swivel rotates the wide panel about the vertical axis",
        sw_xext < rest_xext - 0.02,
        details=f"x-extent rest={rest_xext:.3f}, swiveled={sw_xext:.3f}",
    )
    ctx.check(
        "swivel axis is vertical (Z) at the base",
        abs(swivel.axis[2]) > 0.9 and abs(swivel.axis[0]) < 0.1,
        details=f"swivel axis={swivel.axis}",
    )

    # ---- Height: prismatic up/down on the neck raises the panel ----
    z_rest = ctx.part_world_position(panel)[2]
    with ctx.pose({height: 0.040}):
        z_up = ctx.part_world_position(panel)[2]
    ctx.check(
        "height prismatic raises the panel",
        z_up > z_rest + 0.02,
        details=f"panel z rest={z_rest:.4f}, raised={z_up:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
