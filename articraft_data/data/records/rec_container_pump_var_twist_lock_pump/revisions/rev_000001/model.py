from __future__ import annotations

# Clear soap-dispenser bottle with a twist-to-lock down-lock pump head.
# Frame: bottle main axis along +Z (vertical). Bottle bottom sits at z=0, the
# transparent body rises to the shoulder, then a narrow neck carries a white
# threaded collar. On top of the collar a twist-lock ring rides on a REVOLUTE
# about +Z (limited quarter-turn between unlocked and locked-down). The pump
# head rides on a PRISMATIC along +Z from the lock ring (press travel when
# unlocked; blocked when the bayonet lugs cam the head down flush).
#
# Joint chain: bottle -> collar (FIXED)
#   collar --[REVOLUTE twist_lock +Z, limited 0..LOCK_ANGLE]--> lock_ring
#   lock_ring --[PRISMATIC pump_press +Z, -PRESS_TRAVEL..0]--> head
#
# The lock_ring has grip ribs on the outside and bayonet lugs on the inside.
# The head stem has cam pins that ride in the bayonet slots. Twisting the ring
# to the locked position cams the head down flush and blocks press travel.
# The dip tube hangs from the head down inside the clear bottle.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CylinderGeometry,
    Cylinder,
    Inertial,
    LatheGeometry,
    Mimic,
    MotionLimits,
    Origin,
    TestReport,
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

# Twist-lock parameters
LOCK_ANGLE = math.pi / 2.0  # quarter-turn from unlocked (0) to locked (π/2)
LOCK_RING_H = 0.010  # height of the twist-lock ring
LOCK_RING_BOTTOM = COLLAR_TOP  # sits on collar top rim
LOCK_RING_TOP = LOCK_RING_BOTTOM + LOCK_RING_H

PRESS_TRAVEL = 0.015  # pump head presses straight down this far


def _bottle_mesh():
    """Transparent hollow bottle: lathe shell with domed base and narrow neck."""
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
    """Thin thread ridges on the bottle neck for the collar to screw onto."""
    geo = None
    for i, zz in enumerate((0.154, 0.160, 0.166)):
        ring = TorusGeometry(NECK_R + 0.0005, 0.0012, radial_segments=8, tubular_segments=36)
        ring.translate(0.0, 0.0, zz)
        geo = ring if geo is None else geo.merge(ring)
    return mesh_from_geometry(geo, "neck_threads")


def _collar_mesh():
    """White threaded pump collar: knurled ring wrapping over the bottle neck."""
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
    """Knurl ribs on the collar outer surface."""
    geo = None
    h = COLLAR_TOP - COLLAR_BOTTOM
    for i in range(22):
        ang = 2 * math.pi * i / 22
        rib = CylinderGeometry(0.0012, h, radial_segments=6)
        rib.translate(COLLAR_R - 0.0002, 0.0, COLLAR_BOTTOM + h / 2.0)
        rib.rotate_z(ang)
        geo = rib if geo is None else geo.merge(rib)
    return mesh_from_geometry(geo, "collar_knurl")


def _lock_ring_mesh():
    """Twist-lock ring: a ring with grip ribs outside and bayonet lugs inside.

    The ring sits on the collar top. Grip ribs on the outside let the user
    twist it. Bayonet lugs on the inside engage cam pins on the head stem to
    pull the head down when twisted to the locked position.
    """
    ring_outer_r = COLLAR_R - 0.001  # slightly smaller than collar
    ring_inner_r = 0.0095  # bore for the head stem
    z_bot = LOCK_RING_BOTTOM
    z_top = LOCK_RING_TOP

    # Main ring body (lathe shell)
    outer_profile = [
        (ring_inner_r, z_bot),
        (ring_outer_r, z_bot),
        (ring_outer_r, z_top),
        (ring_inner_r, z_top),
    ]
    inner_profile = [
        (ring_inner_r - 0.001, z_bot + 0.001),
        (ring_inner_r - 0.001, z_top - 0.001),
    ]
    ring_body = LatheGeometry.from_shell_profiles(outer_profile, inner_profile, segments=36)
    geo = ring_body

    # Grip ribs on the outside (6 vertical ribs for finger grip)
    n_grip_ribs = 6
    rib_h = LOCK_RING_H - 0.001
    for i in range(n_grip_ribs):
        ang = 2 * math.pi * i / n_grip_ribs
        rib = CylinderGeometry(0.0015, rib_h, radial_segments=6)
        rib.translate(ring_outer_r + 0.0005, 0.0, z_bot + LOCK_RING_H / 2.0)
        rib.rotate_z(ang)
        geo = geo.merge(rib)

    # Bayonet lugs on the inside (2 horizontal pins that engage the stem cam pins)
    n_lugs = 2
    lug_r = 0.002
    lug_len = 0.006
    lug_z = z_bot + LOCK_RING_H * 0.35  # lugs near the bottom of the ring
    for i in range(n_lugs):
        ang = 2 * math.pi * i / n_lugs + math.pi / 4.0  # offset from grip ribs
        lug = CylinderGeometry(lug_r, lug_len, radial_segments=8)
        # Make horizontal along Y, offset along +X to the ring inner wall, then rotate to angle
        lug.rotate_x(math.pi / 2.0)
        lug.translate(ring_inner_r - lug_len * 0.3, 0.0, lug_z)
        lug.rotate_z(ang)
        geo = geo.merge(lug)

    return mesh_from_geometry(geo, "lock_ring_shell")


def _head_mesh():
    """White pump head: rounded cap + curved spout + stem with cam pins.

    The stem has 2 cam pins (horizontal cylinders) that engage the bayonet
    lugs on the lock ring. When the ring is twisted to locked, the lugs ride
    under the pins and cam the head down.
    """
    # Head cap body
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

    # Curved spout: swept tube exiting sideways (+X) and curving down
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
    # Nozzle tip ring at the spout end
    tip = CylinderGeometry(0.0062, 0.004, radial_segments=16)
    tip.translate(0.041, 0.0, 0.171)
    geo = geo.merge(tip)

    # Vertical actuating stem from the head underside down through the lock ring
    stem = CylinderGeometry(0.0085, 0.040, radial_segments=20)
    stem.translate(0.0, 0.0, 0.166)  # spans ~0.146..0.186
    geo = geo.merge(stem)

    # Cam pins on the stem (2 horizontal pins that engage bayonet lugs)
    cam_pin_z = LOCK_RING_BOTTOM + LOCK_RING_H * 0.35  # matches bayonet lug height
    n_cam_pins = 2
    cam_pin_r = 0.0018
    cam_pin_len = 0.008
    for i in range(n_cam_pins):
        ang = 2 * math.pi * i / n_cam_pins + math.pi / 4.0  # matches lug angular offset
        pin = CylinderGeometry(cam_pin_r, cam_pin_len, radial_segments=8)
        # Make horizontal along Y, offset along +X past stem surface for overlap, then rotate
        pin.rotate_x(math.pi / 2.0)
        pin.translate(0.007, 0.0, cam_pin_z)
        pin.rotate_z(ang)
        geo = geo.merge(pin)

    return mesh_from_geometry(geo, "head_shell")


def _dip_tube_mesh():
    """Thin dip tube hanging from the stem down to near the bottle bottom."""
    tube = CylinderGeometry(0.0028, 0.140, radial_segments=12)
    tube.translate(0.0, 0.0, 0.078)  # spans ~0.008 .. 0.148
    return mesh_from_geometry(tube, "dip_tube")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="soap_dispenser_twist_lock")

    clear = model.material("clear_plastic", rgba=(0.74, 0.80, 0.82, 0.25))
    label_mat = model.material("label", rgba=(0.96, 0.96, 0.94, 1.0))
    white = model.material("pump_white", rgba=(0.93, 0.93, 0.94, 1.0))
    grey = model.material("lock_grey", rgba=(0.78, 0.78, 0.80, 1.0))
    tube_mat = model.material("tube_white", rgba=(0.88, 0.90, 0.90, 0.85))

    # ---- bottle (root): clear shell + label band + neck threads ----
    bottle = model.part("bottle")
    bottle.visual(_bottle_mesh(), material=clear, name="bottle_shell")

    # Wrap-around label band on the lower body
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

    # ---- lock_ring: twist-lock ring with grip ribs and bayonet lugs ----
    lock_ring = model.part("lock_ring")
    lock_ring.visual(_lock_ring_mesh(), material=grey, name="lock_ring_shell")
    lock_ring.inertial = Inertial.from_geometry(
        Cylinder(COLLAR_R - 0.001, LOCK_RING_H),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, LOCK_RING_BOTTOM + LOCK_RING_H / 2.0)),
    )
    # REVOLUTE twist-lock about +Z: quarter-turn from unlocked (0) to locked (π/2).
    # Origin at (0,0,0) keeps the lock_ring frame coincident with the collar frame
    # (and world origin), matching the parent model convention where all geometry
    # is authored in absolute world coordinates.
    model.articulation(
        "twist_lock",
        ArticulationType.REVOLUTE,
        parent=collar,
        child=lock_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=LOCK_ANGLE),
    )

    # ---- head: cap + curved spout + actuating stem + cam pins + dip tube ----
    head = model.part("head")
    head.visual(_head_mesh(), material=white, name="head_shell")
    head.visual(_dip_tube_mesh(), material=tube_mat, name="dip_tube")
    head.inertial = Inertial.from_geometry(
        Box((0.085, 0.030, 0.060)),
        mass=0.03,
        origin=Origin(xyz=(0.015, 0.0, 0.190)),
    )
    # PRISMATIC press along +Z: pressing down (negative travel) then springing back.
    # Available when the twist-lock is in the unlocked position (q=0).
    model.articulation(
        "pump_press",
        ArticulationType.PRISMATIC,
        parent=lock_ring,
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
    lock_ring = object_model.get_part("lock_ring")
    head = object_model.get_part("head")
    twist = object_model.get_articulation("twist_lock")
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

    # --- lock ring sits on collar top (intentional nested fit) ---
    ctx.allow_overlap(
        lock_ring,
        collar,
        elem_a="lock_ring_shell",
        elem_b="collar_shell",
        reason="The twist-lock ring sits in the collar throat; bayonet lugs engage inside the bore.",
    )

    # --- stem/head ride inside the lock ring bore (intentional nested fit) ---
    ctx.allow_overlap(
        head,
        lock_ring,
        elem_a="head_shell",
        elem_b="lock_ring_shell",
        reason="The pump stem with cam pins passes through the lock ring bore.",
    )
    ctx.allow_overlap(
        head,
        collar,
        elem_a="head_shell",
        elem_b="collar_shell",
        reason="The pump stem passes down through the collar bore.",
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

    # --- lock ring exists and has grip ribs (extends beyond plain cylinder) ---
    lock_aabb = ctx.part_element_world_aabb(lock_ring, elem="lock_ring_shell")
    ctx.check(
        "lock ring has grip ribs extending beyond the ring body",
        lock_aabb is not None and (lock_aabb[1][0] - lock_aabb[0][0]) > 2.0 * (COLLAR_R - 0.001),
        details=f"lock_ring_shell x-extent: {lock_aabb[1][0] - lock_aabb[0][0]:.4f} vs ring diameter {2*(COLLAR_R-0.001):.4f}",
    )

    # --- spout is present and off-axis (+X) so twist-lock rotation is detectable ---
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

    # --- TWIST-LOCK: limited revolute range (not free rotation) ---
    twist_limits = twist.motion_limits
    ctx.check(
        "twist-lock has a limited angular range",
        twist_limits is not None
        and twist_limits.lower is not None
        and twist_limits.upper is not None
        and twist_limits.upper - twist_limits.lower <= LOCK_ANGLE + 0.01,
        details=f"twist_lock limits: lower={twist_limits.lower if twist_limits else None}, upper={twist_limits.upper if twist_limits else None}",
    )

    # --- TWIST-LOCK: spout heading rotates about Z (quarter-turn flips X/Y reach) ---
    ext0 = _ext(ctx.part_world_aabb(head))
    with ctx.pose({twist: LOCK_ANGLE}):
        ext_locked = _ext(ctx.part_world_aabb(head))
    ctx.check(
        "spout points along +X at unlocked rest",
        ext0[0] > ext0[1] + 0.010,
        details=f"unlocked extents={ext0}",
    )
    ctx.check(
        "twist-lock quarter-turn rotates spout heading about vertical axis",
        ext_locked[1] > ext_locked[0] + 0.010,
        details=f"locked extents={ext_locked}",
    )

    # --- PRISMATIC press: head moves straight down when pressed (unlocked) ---
    rest = ctx.part_world_position(head)
    with ctx.pose({press: -PRESS_TRAVEL}):
        pressed = ctx.part_world_position(head)
    ctx.check(
        "pump head presses straight down when unlocked",
        rest is not None and pressed is not None and pressed[2] < rest[2] - 0.010,
        details=f"rest_z={rest[2] if rest else None}, pressed_z={pressed[2] if pressed else None}",
    )

    # --- LOCKED pose: head is cammed down (lower Z) compared to unlocked rest ---
    unlocked_rest_z = ctx.part_world_position(head)
    with ctx.pose({twist: LOCK_ANGLE, press: -PRESS_TRAVEL}):
        locked_down_z = ctx.part_world_position(head)
    ctx.check(
        "locked-down pose places the head lower than unlocked rest",
        unlocked_rest_z is not None
        and locked_down_z is not None
        and locked_down_z[2] < unlocked_rest_z[2] - 0.010,
        details=f"unlocked_rest_z={unlocked_rest_z[2] if unlocked_rest_z else None}, locked_down_z={locked_down_z[2] if locked_down_z else None}",
    )

    # --- lock ring is mounted on the collar, not floating ---
    ctx.expect_contact(lock_ring, collar, name="twist-lock ring seated on collar")

    return ctx.report()


object_model = build_object_model()
