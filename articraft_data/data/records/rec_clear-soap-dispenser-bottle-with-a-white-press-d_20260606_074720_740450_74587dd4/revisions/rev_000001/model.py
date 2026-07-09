from __future__ import annotations

# Clear soap-dispenser bottle with a white press-down pump.
# Frame: bottle main axis along +Z (vertical). Bottle bottom sits at z=0, the
# transparent body rises to the shoulder, then a narrow neck carries a white
# threaded collar. On top of the collar a white pump rides on two joints:
#   - pump swivel : REVOLUTE about +Z (the head/spout rotates to aim/lock)
#   - pump press  : PRISMATIC along +Z (the head presses straight down, springs back)
# Joint chain (screw-cap style): bottle(fixed) -> collar
#   collar --[REVOLUTE swivel +Z]--> head_carrier (massless)
#   head_carrier --[PRISMATIC press +Z]--> head (head + stem + curved spout + dip tube)
# The dip tube hangs from the head down inside the clear bottle to near the bottom,
# so it travels with the head/stem.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CylinderGeometry,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---- key heights along +Z ----
BOTTLE_BOTTOM = 0.0
BOTTLE_BODY_TOP = 0.130  # where the cylindrical body ends and the shoulder starts
SHOULDER_TOP = 0.150  # top of the shoulder, base of the neck
NECK_TOP = 0.168  # top of the bottle neck (collar threads onto this)
BODY_R = 0.030  # bottle body outer radius (~0.06 m diameter)
NECK_R = 0.0150  # bottle neck outer radius

COLLAR_R = 0.0185  # white collar outer radius (wraps over the neck)
COLLAR_BOTTOM = 0.150
COLLAR_TOP = 0.176

PRESS_TRAVEL = 0.015  # pump head presses straight down this far


def _bottle_mesh():
    # Transparent hollow bottle: a lathe shell (outer + inner profile) so the
    # body reads as a real thin-walled container, with a domed-in base and a
    # narrow neck on top.
    wall = 0.0022
    outer = [
        (0.0, BOTTLE_BOTTOM),
        (BODY_R, BOTTLE_BOTTOM + 0.004),
        (BODY_R, BOTTLE_BODY_TOP),
        (NECK_R + 0.004, SHOULDER_TOP),
        (NECK_R, SHOULDER_TOP + 0.004),
        (NECK_R, NECK_TOP),
    ]
    inner = [
        (0.0, BOTTLE_BOTTOM + 0.006),
        (BODY_R - wall, BOTTLE_BOTTOM + 0.010),
        (BODY_R - wall, BOTTLE_BODY_TOP),
        (NECK_R + 0.004 - wall, SHOULDER_TOP),
        (NECK_R - wall, SHOULDER_TOP + 0.004),
        (NECK_R - wall, NECK_TOP + 0.002),
    ]
    geo = LatheGeometry.from_shell_profiles(outer, inner, segments=48)
    return mesh_from_geometry(geo, "bottle_shell")


def _neck_threads_mesh():
    # A few thin thread ridges on the bottle neck so the collar reads as screwed on.
    geo = None
    for i, zz in enumerate((0.154, 0.160, 0.166)):
        ring = TorusGeometry(NECK_R + 0.0005, 0.0012, radial_segments=8, tubular_segments=36)
        ring.translate(0.0, 0.0, zz)
        geo = ring if geo is None else geo.merge(ring)
    return mesh_from_geometry(geo, "neck_threads")


def _collar_mesh():
    # White threaded pump collar: a knurled ring that wraps over the bottle neck.
    outer = [
        (0.0, COLLAR_BOTTOM),
        (COLLAR_R, COLLAR_BOTTOM),
        (COLLAR_R, COLLAR_TOP),
        (NECK_R + 0.0015, COLLAR_TOP),
    ]
    inner = [
        (0.0, COLLAR_BOTTOM + 0.0025),
        (NECK_R + 0.0010, COLLAR_BOTTOM + 0.0025),
        (NECK_R + 0.0010, COLLAR_TOP + 0.002),
    ]
    geo = LatheGeometry.from_shell_profiles(outer, inner, segments=48)
    return mesh_from_geometry(geo, "collar_shell")


def _collar_knurl_mesh():
    geo = None
    h = COLLAR_TOP - COLLAR_BOTTOM
    for i in range(22):
        ang = 2 * math.pi * i / 22
        rib = CylinderGeometry(0.0012, h, radial_segments=6)
        rib.translate(COLLAR_R - 0.0002, 0.0, COLLAR_BOTTOM + h / 2.0)
        rib.rotate_z(ang)
        geo = rib if geo is None else geo.merge(rib)
    return mesh_from_geometry(geo, "collar_knurl")


def _head_mesh():
    # White pump head: a rounded cap body with a horizontal curved spout that
    # sticks out off-axis (so a swivel about +Z is clearly detectable), plus the
    # vertical actuating stem that drops down into the collar/neck.
    head_z = 0.196  # center height of the head cap body
    body = LatheGeometry(
        [
            (0.0, 0.184),
            (0.013, 0.184),
            (0.015, 0.190),
            (0.015, 0.200),
            (0.013, 0.206),
            (0.0, 0.208),
        ],
        segments=40,
    )
    geo = body

    # Curved spout: a swept tube that exits the head sideways (+X) and curves down,
    # ending in a nozzle. Points are in the head-local / bottle frame.
    spout_pts = [
        (0.010, 0.0, 0.196),
        (0.022, 0.0, 0.196),
        (0.033, 0.0, 0.192),
        (0.040, 0.0, 0.183),
        (0.041, 0.0, 0.173),
    ]
    spout = tube_from_spline_points(
        spout_pts, radius=0.0055, samples_per_segment=14, radial_segments=14
    )
    geo = geo.merge(spout)
    # Nozzle tip ring at the spout end.
    tip = CylinderGeometry(0.0062, 0.004, radial_segments=16)
    tip.translate(0.041, 0.0, 0.171)
    geo = geo.merge(tip)

    # Vertical actuating stem from the underside of the head down through the collar.
    stem = CylinderGeometry(0.0085, 0.040, radial_segments=20)
    stem.translate(0.0, 0.0, 0.166)  # spans ~0.146..0.186
    geo = geo.merge(stem)

    return mesh_from_geometry(geo, "head_shell")


def _dip_tube_mesh():
    # Thin dip tube hanging from the stem down to near the bottle bottom.
    tube = CylinderGeometry(0.0028, 0.140, radial_segments=12)
    tube.translate(0.0, 0.0, 0.078)  # spans ~0.008 .. 0.148
    return mesh_from_geometry(tube, "dip_tube")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="soap_dispenser")

    clear = model.material("clear_plastic", rgba=(0.74, 0.80, 0.82, 0.25))
    label_mat = model.material("label", rgba=(0.96, 0.96, 0.94, 1.0))
    white = model.material("pump_white", rgba=(0.93, 0.93, 0.94, 1.0))
    tube_mat = model.material("tube_white", rgba=(0.88, 0.90, 0.90, 0.85))

    # ---- bottle (root): clear shell + label band + neck threads ----
    bottle = model.part("bottle")
    bottle.visual(_bottle_mesh(), material=clear, name="bottle_shell")

    # Wrap-around paper label band on the lower body (a thin sleeve).
    # Thin walled sleeve hugging the body; inner face touches the shell outer wall.
    label = LatheGeometry.from_shell_profiles(
        [
            (BODY_R + 0.0008, 0.040),
            (BODY_R + 0.0008, 0.110),
        ],
        [
            (BODY_R - 0.0002, 0.040),
            (BODY_R - 0.0002, 0.110),
        ],
        segments=48,
    )
    bottle.visual(mesh_from_geometry(label, "label_band"), material=label_mat, name="label_band")
    bottle.visual(_neck_threads_mesh(), material=clear, name="neck_threads")

    bottle.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, BOTTLE_BODY_TOP),
        mass=0.20,
        origin=Origin(xyz=(0.0, 0.0, BOTTLE_BODY_TOP / 2.0)),
    )

    # ---- collar: white threaded cap, fixed onto the bottle neck ----
    collar = model.part("collar")
    collar.visual(_collar_mesh(), material=white, name="collar_shell")
    collar.visual(_collar_knurl_mesh(), material=white, name="collar_knurl")
    collar.inertial = Inertial.from_geometry(
        Cylinder(COLLAR_R, COLLAR_TOP - COLLAR_BOTTOM),
        mass=0.02,
        origin=Origin(xyz=(0.0, 0.0, (COLLAR_BOTTOM + COLLAR_TOP) / 2.0)),
    )
    model.articulation(
        "bottle_to_collar",
        ArticulationType.FIXED,
        parent=bottle,
        child=collar,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # ---- head_carrier: massless link between the swivel and the press joints ----
    carrier = model.part("head_carrier")
    # A low swivel flange that rests on the collar top rim (so the carrier is a
    # real, supported part and the head spins on this seat).
    hub = CylinderGeometry(COLLAR_R, 0.004, radial_segments=24)
    hub.translate(0.0, 0.0, COLLAR_TOP + 0.002)  # bottom face sits on the collar top (z=0.176)
    carrier.visual(mesh_from_geometry(hub, "carrier_hub"), material=white, name="carrier_hub")
    carrier.inertial = Inertial.from_geometry(
        Cylinder(COLLAR_R, 0.004), mass=0.001, origin=Origin(xyz=(0.0, 0.0, COLLAR_TOP + 0.002))
    )
    # REVOLUTE swivel about +Z: head/spout rotates to aim/lock.
    model.articulation(
        "pump_swivel",
        ArticulationType.REVOLUTE,
        parent=collar,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=-math.pi, upper=math.pi),
    )

    # ---- head: cap + curved spout + actuating stem + dip tube ----
    head = model.part("head")
    head.visual(_head_mesh(), material=white, name="head_shell")
    head.visual(_dip_tube_mesh(), material=tube_mat, name="dip_tube")
    head.inertial = Inertial.from_geometry(
        Box((0.085, 0.030, 0.060)),
        mass=0.03,
        origin=Origin(xyz=(0.015, 0.0, 0.190)),
    )
    # PRISMATIC press along +Z: pressing down (negative travel) then springing back.
    model.articulation(
        "pump_press",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.1, lower=-PRESS_TRAVEL, upper=0.0),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    from sdk import TestContext

    ctx = TestContext(object_model)

    bottle = object_model.get_part("bottle")
    collar = object_model.get_part("collar")
    carrier = object_model.get_part("head_carrier")
    head = object_model.get_part("head")
    swivel = object_model.get_articulation("pump_swivel")
    press = object_model.get_articulation("pump_press")

    # --- bottle is clear (alpha < 1) ---
    shell_vis = bottle.get_visual("bottle_shell")
    rgba = shell_vis.material.rgba
    ctx.check(
        "bottle is transparent",
        rgba is not None and rgba[3] < 1.0,
        details=f"bottle_shell rgba={rgba}",
    )

    # --- collar is seated on the bottle neck (intentional thread overlap) ---
    ctx.allow_overlap(
        collar,
        bottle,
        elem_a="collar_shell",
        elem_b="neck_threads",
        reason="The white collar is threaded over the bottle neck; the threads engage inside the collar.",
    )
    ctx.allow_overlap(
        collar,
        bottle,
        elem_a="collar_shell",
        elem_b="bottle_shell",
        reason="The collar skirt wraps over the bottle neck wall.",
    )
    ctx.expect_overlap(
        collar, bottle, axes="z", min_overlap=0.005, name="collar seated over the neck"
    )

    # --- stem/head ride inside the collar bore (intentional nested fit) ---
    ctx.allow_overlap(
        head,
        collar,
        elem_a="head_shell",
        elem_b="collar_shell",
        reason="The pump stem passes down through the collar bore.",
    )
    ctx.allow_overlap(
        carrier,
        collar,
        elem_a="carrier_hub",
        elem_b="collar_shell",
        reason="The swivel hub sits in the collar throat.",
    )
    ctx.allow_overlap(
        head,
        carrier,
        elem_a="head_shell",
        elem_b="carrier_hub",
        reason="The stem passes through the carrier hub it is mounted on.",
    )
    # --- dip tube hangs inside the clear bottle (intentional) ---
    ctx.allow_overlap(
        head,
        bottle,
        elem_a="dip_tube",
        elem_b="bottle_shell",
        reason="The dip tube hangs down inside the bottle and crosses the (thin, hollow) walls' footprint.",
    )
    ctx.allow_overlap(
        head,
        bottle,
        elem_a="head_shell",
        elem_b="bottle_shell",
        reason="The actuating stem reaches down into the bottle neck.",
    )

    # --- spout is present and off-axis (sticks out in +X) so swivel is detectable ---
    spout_aabb = ctx.part_element_world_aabb(head, elem="head_shell")
    ctx.check(
        "spout extends off the vertical axis (+X)",
        spout_aabb is not None and spout_aabb[1][0] > 0.030,
        details=f"head_shell aabb max={spout_aabb[1] if spout_aabb else None}",
    )

    # --- dip tube reaches near the bottle bottom ---
    tube_aabb = ctx.part_element_world_aabb(head, elem="dip_tube")
    ctx.check(
        "dip tube reaches near the bottle bottom",
        tube_aabb is not None and tube_aabb[0][2] < 0.020,
        details=f"dip_tube min z={tube_aabb[0][2] if tube_aabb else None}",
    )

    # --- PRISMATIC press: head moves straight down when pressed ---
    rest = ctx.part_world_position(head)
    with ctx.pose({press: -PRESS_TRAVEL}):
        pressed = ctx.part_world_position(head)
    ctx.check(
        "pump head presses straight down",
        rest is not None and pressed is not None and pressed[2] < rest[2] - 0.010,
        details=f"rest_z={rest[2] if rest else None}, pressed_z={pressed[2] if pressed else None}",
    )

    # --- REVOLUTE swivel: spout heading changes about +Z (90 deg flips X<->Y reach) ---
    ext0 = _ext(ctx.part_world_aabb(head))
    with ctx.pose({swivel: math.pi / 2.0}):
        ext90 = _ext(ctx.part_world_aabb(head))
    ctx.check(
        "spout points along +X at rest",
        ext0[0] > ext0[1] + 0.010,
        details=f"rest extents={ext0}",
    )
    ctx.check(
        "swivel rotates the spout heading about the vertical axis",
        ext90[1] > ext90[0] + 0.010,
        details=f"quarter-turn extents={ext90}",
    )

    # --- carrier/head are mounted on the collar, not floating ---
    ctx.expect_contact(carrier, collar, name="swivel carrier seated on collar")

    return ctx.report()


object_model = build_object_model()
