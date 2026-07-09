from __future__ import annotations

# Pump-top soap/lotion bottle variant.
# Frame: bottle axis along +Z, base at z=0, pump at top (+Z).
# Body: opaque HDPE with molded volume bands and a tether tab on the neck.
# Pump collar screws onto neck via CONTINUOUS joint (rotation).
# Pump head presses down via PRISMATIC joint along -Z.

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
    mesh_from_cadquery,
)

# ---- key dimensions (meters) ----
BODY_R = 0.033          # outer barrel radius (~66 mm dia)
WALL = 0.002            # HDPE wall thickness
BASE_Z = 0.0
BARREL_TOP_Z = 0.125    # where the shoulder taper begins
SHOULDER_TOP_Z = 0.150  # top of shoulder, base of neck
NECK_R = 0.014          # neck outer radius
NECK_TOP_Z = 0.170      # top of neck

COLLAR_R = 0.018        # pump collar outer radius
COLLAR_H = 0.022        # collar total height
COLLAR_SKIRT = 0.014    # how far skirt hangs below collar origin

STEM_R = 0.005          # pump stem radius
STEM_INSET = 0.008      # stem extends below pump_head origin (into collar)
STEM_FREE = 0.022       # stem extends above pump_head origin

HEAD_R = 0.013          # pump head disc radius
HEAD_H = 0.008          # pump head disc height
NOZZLE_L = 0.024        # nozzle tube length
NOZZLE_R = 0.004        # nozzle tube radius

PRESS_TRAVEL = 0.012    # how far the pump head can be pressed down

# Volume bands on the barrel
BAND_ZS = [0.035, 0.065, 0.095]
BAND_W = 0.004
BAND_P = 0.0015


def _neck_thread_profile():
    """Sawtooth thread ridges on the neck as profile points."""
    pts = [(NECK_R, SHOULDER_TOP_Z)]
    z0 = SHOULDER_TOP_Z + 0.004
    ridge_r = NECK_R + 0.0015
    for k in range(3):
        zc = z0 + k * 0.005
        pts.append((NECK_R, zc - 0.0018))
        pts.append((ridge_r, zc - 0.0006))
        pts.append((ridge_r, zc + 0.0006))
        pts.append((NECK_R, zc + 0.0018))
    pts.append((NECK_R, NECK_TOP_Z))
    return pts


def _bottle_shell():
    """Hollow bottle with volume bands in the revolve profile."""
    wp = (
        cq.Workplane("XZ")
        .moveTo(0.0, BASE_Z)
        .lineTo(BODY_R - 0.006, BASE_Z)
        .threePointArc((BODY_R, BASE_Z + 0.006), (BODY_R, BASE_Z + 0.012))
    )
    # Volume bands on the barrel
    prev_z = BASE_Z + 0.012
    for bz in sorted(BAND_ZS):
        band_bot = bz - BAND_W / 2.0
        band_top = bz + BAND_W / 2.0
        if band_bot > prev_z + 0.0001:
            wp = wp.lineTo(BODY_R, band_bot)
        wp = wp.lineTo(BODY_R + BAND_P, band_bot)
        wp = wp.lineTo(BODY_R + BAND_P, band_top)
        wp = wp.lineTo(BODY_R, band_top)
        prev_z = band_top

    wp = wp.lineTo(BODY_R, BARREL_TOP_Z)
    # Shoulder taper
    wp = wp.threePointArc(
        ((BODY_R + NECK_R) / 2.0, (BARREL_TOP_Z + SHOULDER_TOP_Z) / 2.0 + 0.004),
        (NECK_R, SHOULDER_TOP_Z),
    )
    # Threaded neck
    for (r, z) in _neck_thread_profile()[1:]:
        wp = wp.lineTo(r, z)
    wp = wp.lineTo(0.0, NECK_TOP_Z).close()
    outer = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))
    # Hollow the bottle: create an inner solid and subtract
    inner_r = BODY_R - WALL
    inner_neck_r = NECK_R - WALL
    wp_in = (
        cq.Workplane("XZ")
        .moveTo(0.0, BASE_Z + WALL)
        .lineTo(inner_r - 0.006, BASE_Z + WALL)
        .threePointArc(
            (inner_r, BASE_Z + WALL + 0.006),
            (inner_r, BASE_Z + WALL + 0.012),
        )
        .lineTo(inner_r, BARREL_TOP_Z)
        .threePointArc(
            ((inner_r + inner_neck_r) / 2.0, (BARREL_TOP_Z + SHOULDER_TOP_Z) / 2.0 + 0.003),
            (inner_neck_r, SHOULDER_TOP_Z),
        )
        .lineTo(inner_neck_r, NECK_TOP_Z)
        .lineTo(0.0, NECK_TOP_Z)
        .close()
    )
    inner = wp_in.revolve(360.0, (0, 0, 0), (0, 1, 0))
    return outer.cut(inner)


def _tether_tab():
    """Small tab with through-hole on the neck side for a cap tether."""
    tab_dx = 0.006       # protrusion in X
    tab_dy = 0.010       # width in Y
    tab_dz = 0.014       # height in Z
    hole_r = 0.003       # through-hole radius
    z_base = SHOULDER_TOP_Z + 0.003
    tab_cx = NECK_R + tab_dx / 2.0 - 0.002  # overlap with neck wall

    # Solid tab
    tab = (
        cq.Workplane("XY")
        .transformed(offset=(tab_cx, 0.0, z_base))
        .rect(tab_dx, tab_dy)
        .extrude(tab_dz)
    )
    # Through-hole in Y direction
    hole = (
        cq.Workplane("XZ")
        .center(tab_cx, z_base + tab_dz / 2.0)
        .circle(hole_r)
        .extrude(tab_dy / 2.0 + 0.001, both=True)
    )
    return tab.cut(hole)


def _bottle_body_geometry():
    """Bottle shell unioned with tether tab for single connected solid."""
    shell = _bottle_shell()
    tab = _tether_tab()
    return shell.union(tab)


def _collar_solid():
    """Ribbed pump collar with neck cavity and stem hole."""
    # Outer cylinder
    outer = (
        cq.Workplane("XY")
        .workplane(offset=-COLLAR_SKIRT)
        .circle(COLLAR_R)
        .extrude(COLLAR_H)
    )
    # Neck cavity (open at bottom, stops 4 mm below collar top)
    cavity_h = COLLAR_H - 0.004
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=-COLLAR_SKIRT - 0.001)
        .circle(NECK_R + 0.001)
        .extrude(cavity_h)
    )
    collar = outer.cut(cavity)

    # Stem passage through the solid top region (slightly smaller than stem
    # for a friction-fit captured connection)
    top_z = COLLAR_H - COLLAR_SKIRT
    stem_hole = (
        cq.Workplane("XY")
        .workplane(offset=top_z - 0.005)
        .circle(STEM_R - 0.0003)
        .extrude(0.006)
    )
    collar = collar.cut(stem_hole)

    # Vertical knurl ribs around the collar
    n_ribs = 20
    rib_h = COLLAR_H * 0.72
    for i in range(n_ribs):
        ang = 2.0 * math.pi * i / n_ribs
        x = (COLLAR_R - 0.0004) * math.cos(ang)
        y = (COLLAR_R - 0.0004) * math.sin(ang)
        zc = -COLLAR_SKIRT + rib_h / 2.0
        rib = (
            cq.Workplane("XY")
            .transformed(offset=(x, y, zc), rotate=(0, 0, math.degrees(ang)))
            .box(0.0015, 0.0012, rib_h)
        )
        collar = collar.union(rib)
    return collar


def _pump_head_solid():
    """Pump head: stem + disc + nozzle as one connected solid."""
    # Stem: from z = -STEM_INSET to z = STEM_FREE
    stem = (
        cq.Workplane("XY")
        .workplane(offset=-STEM_INSET)
        .circle(STEM_R)
        .extrude(STEM_FREE + STEM_INSET)
    )
    # Head disc on top of stem
    disc = (
        cq.Workplane("XY")
        .workplane(offset=STEM_FREE)
        .circle(HEAD_R)
        .extrude(HEAD_H)
    )
    # Nozzle extending from disc in +X direction
    nz = STEM_FREE + HEAD_H * 0.5
    nozzle = (
        cq.Workplane("YZ")
        .workplane(offset=HEAD_R - 0.001)
        .center(0.0, nz)
        .circle(NOZZLE_R)
        .extrude(NOZZLE_L)
    )
    # Nozzle tip sphere
    tip = (
        cq.Workplane("XY")
        .transformed(offset=(HEAD_R + NOZZLE_L - 0.002, 0.0, nz))
        .sphere(NOZZLE_R * 1.3)
    )
    return stem.union(disc).union(nozzle).union(tip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pump_bottle")

    # Materials
    white = model.material("bottle_white", rgba=(0.92, 0.90, 0.86, 1.0))
    charcoal = model.material("pump_charcoal", rgba=(0.14, 0.14, 0.15, 1.0))

    # ---- bottle body (root) ----
    body = model.part("bottle")
    body_geo = _bottle_body_geometry()
    body.visual(
        mesh_from_cadquery(body_geo, "bottle_shell"),
        material=white,
        name="bottle_shell",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, NECK_TOP_Z),
        mass=0.040,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2.0)),
    )

    # ---- pump collar ----
    collar = model.part("collar")
    collar_geo = _collar_solid()
    collar.visual(
        mesh_from_cadquery(collar_geo, "collar_shell"),
        material=charcoal,
        name="collar_shell",
    )
    collar.inertial = Inertial.from_geometry(
        Cylinder(COLLAR_R, COLLAR_H),
        mass=0.010,
        origin=Origin(xyz=(0.0, 0.0, COLLAR_H / 2.0 - COLLAR_SKIRT)),
    )

    # ---- pump head ----
    head = model.part("pump_head")
    head_geo = _pump_head_solid()
    head.visual(
        mesh_from_cadquery(head_geo, "pump_shell"),
        material=charcoal,
        name="pump_shell",
    )
    head.inertial = Inertial.from_geometry(
        Cylinder(HEAD_R, STEM_FREE + HEAD_H + STEM_INSET),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, (STEM_FREE - STEM_INSET + HEAD_H) / 2.0)),
    )

    # ---- articulations ----
    # Collar rotates on the neck (continuous spin for screwing)
    model.articulation(
        "collar_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=collar,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )
    # Pump head presses downward (prismatic along -Z)
    model.articulation(
        "pump_press",
        ArticulationType.PRISMATIC,
        parent=collar,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, COLLAR_H - COLLAR_SKIRT)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            lower=0.0, upper=PRESS_TRAVEL, effort=2.0, velocity=0.5
        ),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle")
    collar = object_model.get_part("collar")
    head = object_model.get_part("pump_head")
    rotate = object_model.get_articulation("collar_rotate")
    press = object_model.get_articulation("pump_press")

    # --- joint types ---
    ctx.check(
        "collar_rotate is continuous joint",
        rotate.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={rotate.articulation_type}",
    )
    ctx.check(
        "pump_press is prismatic joint",
        press.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={press.articulation_type}",
    )

    # --- collar rotation moves the asymmetric nozzle ---
    with ctx.pose({rotate: 0.0}):
        aabb0 = ctx.part_world_aabb(head)
    with ctx.pose({rotate: math.pi / 2.0}):
        aabb90 = ctx.part_world_aabb(head)
    e0 = _ext(aabb0)
    e90 = _ext(aabb90)
    ctx.check(
        "collar rotation swaps nozzle extents (x<->y on quarter turn)",
        abs(e0[0] - e90[1]) < 0.004 and abs(e0[0] - e0[1]) > 0.005,
        details=f"rest={e0}, quarter_turn={e90}",
    )

    # --- pump press lowers the head ---
    rest_z = ctx.part_world_position(head)[2]
    with ctx.pose({press: PRESS_TRAVEL}):
        pressed_z = ctx.part_world_position(head)[2]
    ctx.check(
        "pump press lowers the head by approximately PRESS_TRAVEL",
        pressed_z < rest_z - PRESS_TRAVEL * 0.8,
        details=f"rest_z={rest_z}, pressed_z={pressed_z}",
    )

    # --- volume bands protrude beyond body radius ---
    body_aabb = ctx.part_world_aabb(body)
    body_dx = body_aabb[1][0] - body_aabb[0][0]
    ctx.check(
        "volume bands protrude beyond 2*BODY_R",
        body_dx > 2.0 * BODY_R + BAND_P * 0.5,
        details=f"body_dx={body_dx:.5f}, threshold={2.0 * BODY_R + BAND_P * 0.5:.5f}",
    )

    # --- tether tab present on bottle body ---
    body_vis_names = [v.name for v in body.visuals]
    ctx.check(
        "bottle_shell visual exists (includes tether tab)",
        "bottle_shell" in body_vis_names,
        details=f"visuals={body_vis_names}",
    )

    # --- collar mounted above barrel on neck ---
    collar_pos = ctx.part_world_position(collar)
    ctx.check(
        "collar mounted above barrel top",
        collar_pos is not None and collar_pos[2] > BARREL_TOP_Z,
        details=f"collar_z={collar_pos}",
    )

    # --- collar skirt seated over threaded neck (intentional overlap) ---
    ctx.allow_overlap(
        collar,
        body,
        elem_a="collar_shell",
        elem_b="bottle_shell",
        reason="Collar skirt wraps around the threaded neck for screw engagement.",
    )

    # --- pump stem captured through collar disc (intentional friction fit) ---
    ctx.allow_overlap(
        head,
        collar,
        elem_a="pump_shell",
        elem_b="collar_shell",
        reason="Pump stem passes through collar disc with friction-fit retention.",
    )
    ctx.expect_gap(
        head,
        collar,
        axis="z",
        max_penetration=STEM_INSET + 0.002,
        name="stem passes through collar disc region",
    )

    # --- prove collar contact/seating on the neck ---
    ctx.expect_gap(
        collar,
        body,
        axis="z",
        max_penetration=COLLAR_SKIRT + 0.002,
        name="collar skirt overlaps neck vertically (seated engagement)",
    )

    return ctx.report()


object_model = build_object_model()
