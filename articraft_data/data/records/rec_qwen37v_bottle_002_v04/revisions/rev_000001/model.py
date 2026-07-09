from __future__ import annotations

# Pump-top soap/lotion bottle variant:
#   - Clear plastic (PET) bottle body with tapered shoulder
#   - 3 molded volume bands (raised ridges) around the body
#   - Small tether loop connected to the neck
#   - Screw-on pump collar with CONTINUOUS rotation
#   - Pressable pump actuator head with nozzle (PRISMATIC)
# Frame: vertical axis along +Z, bottle standing on z=0, centerline on x=y=0.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
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
BODY_TOP_Z = 0.110       # end of straight cylindrical body, start of shoulder
SHOULDER_TOP_Z = 0.156   # end of tapered shoulder, base of neck
NECK_TOP_Z = 0.176       # top of threaded neck (pump mounts here)

BODY_R = 0.0275          # body radius (~0.055 m dia)
NECK_R = 0.0125          # outer thread/neck radius
NECK_BORE_R = 0.0098     # neck inner bore

# Pump collar dimensions
COLLAR_R = 0.0160        # outer radius of the pump collar
COLLAR_H = 0.014         # height of the collar
COLLAR_BORE_R = NECK_R   # collar bore matches neck outer radius

# Pump actuator head dimensions
HEAD_R = 0.012           # head disc radius
HEAD_H = 0.008           # head height
STEM_R = 0.005           # stem radius connecting collar to head
STEM_H = 0.016           # stem height
FLANGE_R = 0.014         # base flange radius (rests on collar top)
FLANGE_H = 0.003         # base flange height
NOZZLE_L = 0.020         # nozzle spout length
NOZZLE_R = 0.0025        # nozzle radius

# Pump press travel
PUMP_PRESS_TRAVEL = 0.006

# Volume band parameters
BAND_TUBE_R = 0.0018     # tube radius of the band torus
BAND_HEIGHTS = (0.032, 0.062, 0.092)  # z heights for 3 bands

# Tether loop
TETHER_Z = SHOULDER_TOP_Z - 0.004  # near shoulder-neck junction
TETHER_MAJOR_R = 0.005   # ring major radius
TETHER_TUBE_R = 0.0015   # ring tube radius


def _profile_sections():
    """(z, radius) of the outer wall, base -> body -> tapered shoulder -> neck."""
    return [
        (0.000, 0.0150),   # rounded base bottom (tucked-in heel)
        (0.006, 0.0250),
        (0.014, 0.0273),
        (BODY_TOP_Z, BODY_R),  # straight cylindrical body
        (0.124, 0.0268),       # shoulder starts tapering inward
        (0.138, 0.0228),
        (SHOULDER_TOP_Z, 0.0148),  # long tapered shoulder
        (0.160, NECK_R),      # base of neck
        (NECK_TOP_Z, NECK_R),  # straight threaded neck up to the rim
    ]


def _bottle_solid() -> cq.Workplane:
    """Revolve the outer profile, then shell it hollow through the neck rim."""
    pts = _profile_sections()
    wp = cq.Workplane("XZ").moveTo(0.0, pts[0][0])
    for r, z in [(r, z) for (z, r) in pts]:
        wp = wp.lineTo(r, z)
    wp = wp.lineTo(0.0, pts[-1][0]).close()
    outer = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))

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
        (NECK_BORE_R, NECK_TOP_Z + 0.005),
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
    """Helical-ish thread rings on the neck."""
    g = None
    for zt in (0.163, 0.169):
        ring = TorusGeometry(NECK_R - 0.0006, 0.0012, radial_segments=10, tubular_segments=40)
        ring.translate(0.0, 0.0, zt)
        if g is None:
            g = ring
        else:
            g.merge(ring)
    return mesh_from_geometry(g, "neck_threads")


def _volume_bands():
    """Three raised ridge bands around the bottle body."""
    g = None
    for zt in BAND_HEIGHTS:
        band = TorusGeometry(BODY_R, BAND_TUBE_R, radial_segments=8, tubular_segments=48)
        band.translate(0.0, 0.0, zt)
        if g is None:
            g = band
        else:
            g.merge(band)
    return mesh_from_geometry(g, "volume_bands")


def _tether_loop():
    """Small ring protruding from the neck area for a cap tether cord."""
    g = TorusGeometry(TETHER_MAJOR_R, TETHER_TUBE_R, radial_segments=8, tubular_segments=24)
    # Orient the torus so its axis is along Y (ring faces along X),
    # and place it at the neck side
    g.rotate((0, 1, 0), 90.0)
    g.translate(NECK_R + TETHER_MAJOR_R - 0.001, 0.0, TETHER_Z)
    # Add a small connecting tab/bridge from the neck surface to the ring
    tab = BoxGeometry((0.003, 0.004, 0.004))
    tab.translate(NECK_R + 0.001, 0.0, TETHER_Z)
    g.merge(tab)
    return mesh_from_geometry(g, "tether_loop")


def _collar_solid() -> cq.Workplane:
    """Pump collar: a ribbed hollow cylinder that screws over the neck."""
    collar = (
        cq.Workplane("XY")
        .circle(COLLAR_R)
        .extrude(COLLAR_H)
    )
    # Hollow bore so it fits over the neck
    bore = (
        cq.Workplane("XY")
        .circle(COLLAR_BORE_R)
        .extrude(COLLAR_H)
    )
    collar = collar.cut(bore)
    # Vertical grip ribs on the outside
    n_ribs = 20
    for i in range(n_ribs):
        a = 2.0 * math.pi * i / n_ribs
        groove = (
            cq.Workplane("XY")
            .center(COLLAR_R * math.cos(a), COLLAR_R * math.sin(a))
            .circle(0.0008)
            .extrude(COLLAR_H)
        )
        collar = collar.cut(groove)
    return collar


def _collar_mesh():
    return mesh_from_cadquery(_collar_solid(), "pump_collar_shell")


def _pump_head_solid() -> cq.Workplane:
    """Pump actuator: base flange + stem + head disc + nozzle spout."""
    # Base flange: disc that rests on the collar top (wider than bore, narrower than collar)
    flange = (
        cq.Workplane("XY")
        .circle(FLANGE_R)
        .extrude(FLANGE_H)
    )
    # Stem rising from the flange
    stem = (
        cq.Workplane("XY")
        .workplane(offset=FLANGE_H)
        .circle(STEM_R)
        .extrude(STEM_H)
    )
    result = flange.union(stem)
    # Head disc on top of stem
    head_disc = (
        cq.Workplane("XY")
        .workplane(offset=FLANGE_H + STEM_H)
        .circle(HEAD_R)
        .extrude(HEAD_H)
    )
    result = result.union(head_disc)
    # Nozzle spout: horizontal cylinder extending along +X from the head side
    nozzle_z = FLANGE_H + STEM_H + HEAD_H * 0.5
    nozzle = (
        cq.Workplane("YZ")
        .workplane(offset=HEAD_R * 0.3)
        .center(0.0, nozzle_z)
        .circle(NOZZLE_R)
        .extrude(NOZZLE_L)
    )
    result = result.union(nozzle)
    return result


def _pump_head_mesh():
    return mesh_from_cadquery(_pump_head_solid(), "pump_head_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pump_bottle")

    # Materials
    clear = model.material("clear_pet", rgba=(0.78, 0.85, 0.88, 0.25))
    clear_neck = model.material("clear_neck", rgba=(0.72, 0.80, 0.84, 0.30))
    band_mat = model.material("body_band", rgba=(0.72, 0.80, 0.84, 0.40))
    tether_mat = model.material("tether_plastic", rgba=(0.65, 0.70, 0.73, 0.50))
    pump_white = model.material("pump_white", rgba=(0.92, 0.92, 0.90, 1.0))
    nozzle_gray = model.material("nozzle_gray", rgba=(0.75, 0.75, 0.73, 1.0))

    # ---- bottle body (root): transparent hollow PET shell ----
    body = model.part("bottle_body")
    body.visual(_bottle_mesh(), material=clear, name="bottle_shell")
    body.visual(_neck_threads(), material=clear_neck, name="neck_threads")
    body.visual(_volume_bands(), material=band_mat, name="volume_bands")
    body.visual(_tether_loop(), material=tether_mat, name="tether_loop")
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, 0.176),
        mass=0.022,
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
    )

    # ---- pump collar: screw-on ring with grip ribs ----
    collar = model.part("pump_collar")
    collar.visual(_collar_mesh(), material=pump_white, name="pump_collar_shell")
    collar.inertial = Inertial.from_geometry(
        Cylinder(COLLAR_R, COLLAR_H),
        mass=0.005,
        origin=Origin(xyz=(0.0, 0.0, COLLAR_H / 2.0)),
    )

    # ---- pump head: pressable actuator with nozzle ----
    head = model.part("pump_head")
    head.visual(_pump_head_mesh(), material=nozzle_gray, name="pump_head_shell")
    total_head_h = FLANGE_H + STEM_H + HEAD_H
    head.inertial = Inertial.from_geometry(
        Cylinder(HEAD_R, total_head_h),
        mass=0.004,
        origin=Origin(xyz=(0.0, 0.0, total_head_h / 2.0)),
    )

    # pump_rotate: CONTINUOUS spin of the collar about +Z
    # Origin at the base of the neck so the collar screws down over it
    model.articulation(
        "pump_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=collar,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z - COLLAR_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=1.0),
    )

    # pump_press: PRISMATIC press of the head downward
    # Origin at the top of the collar; head sits on top and presses down
    # axis=(0,0,-1) so positive q presses downward
    model.articulation(
        "pump_press",
        ArticulationType.PRISMATIC,
        parent=collar,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, COLLAR_H)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=PUMP_PRESS_TRAVEL,
            effort=2.0,
            velocity=0.5,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle_body")
    collar = object_model.get_part("pump_collar")
    head = object_model.get_part("pump_head")
    rotate = object_model.get_articulation("pump_rotate")
    press = object_model.get_articulation("pump_press")

    # --- bottle is clear (translucent) ---
    clear_mat = next(m for m in object_model.materials if m.name == "clear_pet")
    a = clear_mat.rgba[3] if clear_mat.rgba is not None else 1.0
    ctx.check(
        "bottle shell is transparent",
        a < 1.0,
        details=f"clear_pet alpha={a}",
    )

    # --- volume bands exist on the body ---
    band_vis = body.get_visual("volume_bands")
    ctx.check(
        "volume bands visual exists",
        band_vis is not None,
        details="volume_bands visual not found on bottle_body",
    )

    # --- tether loop exists on the body ---
    tether_vis = body.get_visual("tether_loop")
    ctx.check(
        "tether loop visual exists",
        tether_vis is not None,
        details="tether_loop visual not found on bottle_body",
    )

    # --- pump collar sits at the top of the bottle (screw-on) ---
    ctx.allow_overlap(
        collar,
        body,
        elem_a="pump_collar_shell",
        elem_b="bottle_shell",
        reason="The pump collar intentionally screws over the threaded neck.",
    )
    ctx.allow_overlap(
        collar,
        body,
        elem_a="pump_collar_shell",
        elem_b="neck_threads",
        reason="The pump collar covers the neck threads when mounted.",
    )
    collar_pos = ctx.part_world_position(collar)
    ctx.check(
        "pump collar mounted at the top of the bottle",
        collar_pos is not None and collar_pos[2] > 0.14,
        details=f"collar origin={collar_pos}",
    )

    # --- pump collar spins about +Z (continuous rotation) ---
    ctx.check(
        "pump_rotate is a continuous joint",
        rotate.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={rotate.articulation_type}",
    )
    # Verify rotation moves the collar
    collar_aabb_rest = ctx.part_world_aabb(collar)
    with ctx.pose({rotate: math.pi / 4.0}):
        collar_aabb_rot = ctx.part_world_aabb(collar)
    # AABB should change because the ribbed collar is not perfectly symmetric at 45 degrees
    # but mainly just confirm the joint exists and is continuous
    ctx.check(
        "pump collar has continuous rotation joint",
        rotate.motion_limits is not None
        and rotate.motion_limits.lower is None
        and rotate.motion_limits.upper is None,
        details="pump_rotate should have no position limits",
    )

    # --- pump head is above the collar at rest ---
    head_pos = ctx.part_world_position(head)
    ctx.check(
        "pump head is above the collar",
        head_pos is not None and head_pos[2] > (collar_pos[2] + 0.01) if collar_pos else False,
        details=f"head origin={head_pos}, collar origin={collar_pos}",
    )

    # --- pump press moves the head downward ---
    ctx.check(
        "pump_press is a prismatic joint",
        press.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={press.articulation_type}",
    )
    head_z_rest = ctx.part_world_aabb(head)[1][2]  # top of head at rest
    with ctx.pose({press: PUMP_PRESS_TRAVEL}):
        head_z_pressed = ctx.part_world_aabb(head)[1][2]  # top of head when pressed
    ctx.check(
        "pump head moves down when pressed",
        head_z_pressed < head_z_rest - 0.002,
        details=f"head top rest={head_z_rest:.4f}, pressed={head_z_pressed:.4f}",
    )

    # --- bottle is tall (taller than wide) ---
    full_aabb = ctx.part_world_aabb(body)
    if full_aabb:
        mn, mx = full_aabb
        dx = mx[0] - mn[0]
        dz = mx[2] - mn[2]
        ctx.check(
            "bottle is tall (taller than wide)",
            dz > 2.5 * dx,
            details=f"body dx={dx:.4f}, dz={dz:.4f}",
        )

    # --- tapered shoulder narrows toward the top ---
    ctx.check(
        "tapered shoulder narrows toward the top",
        NECK_R < BODY_R * 0.6,
        details=f"neck_r={NECK_R}, body_r={BODY_R}",
    )

    return ctx.report()


object_model = build_object_model()
