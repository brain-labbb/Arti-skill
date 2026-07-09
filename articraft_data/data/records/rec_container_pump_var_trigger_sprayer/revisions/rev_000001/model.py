from __future__ import annotations

# Clear soap-dispenser bottle with a white trigger-sprayer head.
# Frame: bottle main axis along +Z (vertical). Bottle bottom sits at z=0, the
# transparent body rises to the shoulder, then a narrow neck carries a white
# threaded collar. On top of the collar a trigger-sprayer rides on two joints:
#   - sprayer swivel : REVOLUTE about +Z (the head/nozzle rotates to aim/lock)
#   - trigger squeeze: REVOLUTE about -Y (finger trigger pivots toward the nozzle)
# Joint chain (screw-cap style): bottle(fixed) -> collar
#   collar --[REVOLUTE swivel +Z]--> sprayer_head (body + nozzle + stem + dip tube)
#   sprayer_head --[REVOLUTE squeeze -Y]--> trigger (finger lever)
# The dip tube hangs from the sprayer head down inside the clear bottle to near
# the bottom, so it travels with the head.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CapsuleGeometry,
    CylinderGeometry,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
    TestContext,
    TestReport,
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

# Sprayer head geometry
SPRAYER_BODY_BOTTOM = COLLAR_TOP + 0.002  # 0.178 - sits just above collar
SPRAYER_BODY_TOP = 0.215  # top of the sprayer body
SPRAYER_BODY_R = 0.0135  # main body radius
NOZZLE_EXIT_Z = 0.200  # height where nozzle exits the body
TRIGGER_PIVOT_Z = 0.196  # height of the trigger pivot pin (raised to avoid bottle overlap)
TRIGGER_PIVOT_X = 0.020  # forward offset of trigger pivot from bottle axis

# Trigger limits
TRIGGER_SQUEEZE = 0.55  # max squeeze angle in radians (~31 deg)


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


def _sprayer_body_mesh():
    # Trigger sprayer main body: a rounded cylindrical housing that sits on
    # the collar top and carries the nozzle and trigger mechanism.
    body = LatheGeometry(
        [
            (0.0, SPRAYER_BODY_BOTTOM),
            (SPRAYER_BODY_R - 0.002, SPRAYER_BODY_BOTTOM),
            (SPRAYER_BODY_R, SPRAYER_BODY_BOTTOM + 0.004),
            (SPRAYER_BODY_R, SPRAYER_BODY_TOP - 0.006),
            (SPRAYER_BODY_R - 0.003, SPRAYER_BODY_TOP - 0.002),
            (SPRAYER_BODY_R - 0.005, SPRAYER_BODY_TOP),
            (0.0, SPRAYER_BODY_TOP + 0.002),
        ],
        segments=40,
    )
    return mesh_from_geometry(body, "sprayer_body")


def _nozzle_mesh():
    # Forward-projecting nozzle tube that exits the body at +X and extends
    # outward with a slight downward angle (typical trigger sprayer aim).
    nozzle_pts = [
        (SPRAYER_BODY_R - 0.002, 0.0, NOZZLE_EXIT_Z),
        (SPRAYER_BODY_R + 0.008, 0.0, NOZZLE_EXIT_Z - 0.001),
        (SPRAYER_BODY_R + 0.022, 0.0, NOZZLE_EXIT_Z - 0.003),
        (SPRAYER_BODY_R + 0.036, 0.0, NOZZLE_EXIT_Z - 0.005),
        (SPRAYER_BODY_R + 0.046, 0.0, NOZZLE_EXIT_Z - 0.006),
    ]
    nozzle = tube_from_spline_points(
        nozzle_pts, radius=0.0045, samples_per_segment=14, radial_segments=14
    )
    return mesh_from_geometry(nozzle, "nozzle")


def _nozzle_tip_mesh():
    # Small nozzle tip ring at the end of the nozzle barrel.
    tip = CylinderGeometry(0.0055, 0.005, radial_segments=16)
    tip.translate(SPRAYER_BODY_R + 0.048, 0.0, NOZZLE_EXIT_Z - 0.006)
    return mesh_from_geometry(tip, "nozzle_tip")


def _stem_mesh():
    # Vertical actuating stem from the underside of the sprayer body down
    # through the collar bore into the bottle neck.
    stem = CylinderGeometry(0.0080, 0.034, radial_segments=20)
    stem.translate(0.0, 0.0, SPRAYER_BODY_BOTTOM - 0.017)  # spans ~0.161..0.195
    return mesh_from_geometry(stem, "stem")


def _dip_tube_mesh():
    # Thin dip tube hanging from the stem down to near the bottle bottom.
    tube = CylinderGeometry(0.0028, 0.140, radial_segments=12)
    tube.translate(0.0, 0.0, 0.078)  # spans ~0.008 .. 0.148
    return mesh_from_geometry(tube, "dip_tube")


def _pivot_boss_mesh(y_sign: float, name_suffix: int) -> tuple:
    """Box-shaped pivot bracket extending from body to trigger pivot point."""
    boss_name = f"pivot_boss_{name_suffix}"
    # Bracket arm extending from body front surface to trigger pivot position
    # This creates a physical connection between the body and the trigger mount
    length = TRIGGER_PIVOT_X - SPRAYER_BODY_R + 0.006  # extends slightly into body for connection
    bracket = BoxGeometry((length, 0.006, 0.010))
    # Center the bracket between body surface and pivot point
    center_x = (SPRAYER_BODY_R + TRIGGER_PIVOT_X) / 2.0
    bracket.translate(center_x, y_sign * 0.007, TRIGGER_PIVOT_Z)
    return mesh_from_geometry(bracket, boss_name), boss_name


def _trigger_lever_mesh():
    # Trigger lever in its own local frame: pivot at origin (0,0,0), lever
    # hangs downward (-Z). The shape is an elongated flat paddle with a
    # slight finger-rest widening at the bottom.
    # Main lever body (shorter to avoid bottle overlap)
    lever = BoxGeometry((0.006, 0.013, 0.024))
    lever.translate(0.0, 0.0, -0.015)  # center at z=-0.015, spans z=-0.027..-0.003
    geo = lever

    # Finger rest pad at the bottom (wider contact area, raised position)
    pad = BoxGeometry((0.008, 0.015, 0.006))
    pad.translate(0.001, 0.0, -0.028)  # slightly forward, at the bottom
    geo = geo.merge(pad)

    # Pivot barrel at the top (the pin that sits in the bosses)
    pin = CylinderGeometry(0.003, 0.018, radial_segments=12)
    pin.rotate_x(math.pi / 2.0)  # orient along Y
    geo = geo.merge(pin)

    return mesh_from_geometry(geo, "trigger_lever")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="trigger_sprayer_bottle")

    clear = model.material("clear_plastic", rgba=(0.74, 0.80, 0.82, 0.25))
    label_mat = model.material("label", rgba=(0.96, 0.96, 0.94, 1.0))
    white = model.material("pump_white", rgba=(0.93, 0.93, 0.94, 1.0))
    grey = model.material("trigger_grey", rgba=(0.82, 0.83, 0.85, 1.0))
    tube_mat = model.material("tube_white", rgba=(0.88, 0.90, 0.90, 0.85))

    # ---- bottle (root): clear shell + label band + neck threads ----
    bottle = model.part("bottle")
    bottle.visual(_bottle_mesh(), material=clear, name="bottle_shell")

    # Wrap-around label band on the lower body (a thin sleeve).
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

    # ---- sprayer_head: trigger sprayer body + nozzle + stem + dip tube ----
    sprayer_head = model.part("sprayer_head")
    sprayer_head.visual(_sprayer_body_mesh(), material=white, name="sprayer_body")
    sprayer_head.visual(_nozzle_mesh(), material=white, name="nozzle")
    sprayer_head.visual(_nozzle_tip_mesh(), material=grey, name="nozzle_tip")
    sprayer_head.visual(_stem_mesh(), material=white, name="stem")
    sprayer_head.visual(_dip_tube_mesh(), material=tube_mat, name="dip_tube")

    # Pivot bosses on both sides (inline decorations, no separate parts)
    for i, y_sign in enumerate((-1.0, 1.0)):
        boss_mesh, boss_name = _pivot_boss_mesh(y_sign, i)
        sprayer_head.visual(boss_mesh, material=grey, name=boss_name)

    sprayer_head.inertial = Inertial.from_geometry(
        Box((0.080, 0.030, 0.050)),
        mass=0.04,
        origin=Origin(xyz=(0.015, 0.0, 0.196)),
    )

    # REVOLUTE swivel about +Z: whole sprayer head rotates to aim nozzle.
    model.articulation(
        "sprayer_swivel",
        ArticulationType.REVOLUTE,
        parent=collar,
        child=sprayer_head,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=-math.pi, upper=math.pi),
    )

    # ---- trigger: finger lever that pivots on the sprayer body ----
    trigger = model.part("trigger")
    trigger.visual(_trigger_lever_mesh(), material=grey, name="trigger_lever")
    trigger.inertial = Inertial.from_geometry(
        Box((0.010, 0.016, 0.034)),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, -0.016)),
    )

    # REVOLUTE squeeze about -Y: positive q swings the trigger bottom toward
    # +X (toward the nozzle direction = squeeze). At rest (q=0) the trigger
    # hangs straight down. Squeezing pulls the trigger toward the body.
    model.articulation(
        "trigger_squeeze",
        ArticulationType.REVOLUTE,
        parent=sprayer_head,
        child=trigger,
        origin=Origin(xyz=(TRIGGER_PIVOT_X, 0.0, TRIGGER_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0, lower=0.0, upper=TRIGGER_SQUEEZE),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bottle = object_model.get_part("bottle")
    collar = object_model.get_part("collar")
    sprayer_head = object_model.get_part("sprayer_head")
    trigger = object_model.get_part("trigger")
    swivel = object_model.get_articulation("sprayer_swivel")
    squeeze = object_model.get_articulation("trigger_squeeze")

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
        sprayer_head,
        collar,
        elem_a="sprayer_body",
        elem_b="collar_shell",
        reason="The sprayer body base sits in the collar throat.",
    )
    ctx.allow_overlap(
        sprayer_head,
        collar,
        elem_a="stem",
        elem_b="collar_shell",
        reason="The pump stem passes down through the collar bore.",
    )

    # --- dip tube hangs inside the clear bottle (intentional) ---
    ctx.allow_overlap(
        sprayer_head,
        bottle,
        elem_a="dip_tube",
        elem_b="bottle_shell",
        reason="The dip tube hangs down inside the bottle and crosses the (thin, hollow) walls' footprint.",
    )
    ctx.allow_overlap(
        sprayer_head,
        bottle,
        elem_a="stem",
        elem_b="bottle_shell",
        reason="The stem reaches down into the bottle neck.",
    )

    # --- nozzle is present and off-axis (sticks out in +X) so swivel is detectable ---
    nozzle_aabb = ctx.part_element_world_aabb(sprayer_head, elem="nozzle")
    ctx.check(
        "nozzle extends off the vertical axis (+X)",
        nozzle_aabb is not None and nozzle_aabb[1][0] > 0.040,
        details=f"nozzle aabb max_x={nozzle_aabb[1][0] if nozzle_aabb else None}",
    )

    # --- dip tube reaches near the bottle bottom ---
    tube_aabb = ctx.part_element_world_aabb(sprayer_head, elem="dip_tube")
    ctx.check(
        "dip tube reaches near the bottle bottom",
        tube_aabb is not None and tube_aabb[0][2] < 0.020,
        details=f"dip_tube min z={tube_aabb[0][2] if tube_aabb else None}",
    )

    # --- trigger exists and hangs below the sprayer body pivot ---
    trigger_aabb = ctx.part_world_aabb(trigger)
    pivot_z = TRIGGER_PIVOT_Z
    ctx.check(
        "trigger hangs below the pivot point",
        trigger_aabb is not None and trigger_aabb[0][2] < pivot_z - 0.010,
        details=f"trigger_min_z={trigger_aabb[0][2] if trigger_aabb else None}, pivot_z={pivot_z}",
    )

    # --- REVOLUTE trigger squeeze: trigger bottom swings forward (+X) when squeezed ---
    rest_pos = ctx.part_world_position(trigger)
    with ctx.pose({squeeze: TRIGGER_SQUEEZE}):
        squeezed_pos = ctx.part_world_position(trigger)
    # The trigger center of mass should not move much (it rotates about pivot)
    # but the trigger part AABB should shift in X extent
    rest_aabb = ctx.part_world_aabb(trigger)
    with ctx.pose({squeeze: TRIGGER_SQUEEZE}):
        squeezed_aabb = ctx.part_world_aabb(trigger)
    ctx.check(
        "trigger squeeze moves the lever forward (+X direction)",
        rest_aabb is not None and squeezed_aabb is not None
        and squeezed_aabb[1][0] > rest_aabb[1][0] + 0.005,
        details=f"rest_max_x={rest_aabb[1][0] if rest_aabb else None}, "
                f"squeezed_max_x={squeezed_aabb[1][0] if squeezed_aabb else None}",
    )

    # --- REVOLUTE swivel: nozzle heading changes about +Z (90 deg flips X<->Y reach) ---
    ext0 = _ext(ctx.part_world_aabb(sprayer_head))
    with ctx.pose({swivel: math.pi / 2.0}):
        ext90 = _ext(ctx.part_world_aabb(sprayer_head))
    ctx.check(
        "nozzle points along +X at rest",
        ext0[0] > ext0[1] + 0.010,
        details=f"rest extents={ext0}",
    )
    ctx.check(
        "swivel rotates the nozzle heading about the vertical axis",
        ext90[1] > ext90[0] + 0.010,
        details=f"quarter-turn extents={ext90}",
    )

    # --- sprayer_head is mounted on the collar, not floating ---
    ctx.expect_contact(sprayer_head, collar, name="sprayer head seated on collar")

    return ctx.report()


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


object_model = build_object_model()
