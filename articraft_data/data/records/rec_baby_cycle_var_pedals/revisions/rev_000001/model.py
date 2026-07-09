from __future__ import annotations

# Toddler pedal tricycle ("baby cycle").
# Object frame:
#   +X = right, +Y = forward (front of the cycle), +Z = up.
# Layout:
#   - White tubular frame is the root. It runs from a tilted head tube at the
#     front (+Y) back to a low saddle deck and a rear axle that carries TWO
#     splayed rear wheels.
#   - A blue handlebar + front fork turn together about the tilted head-tube
#     (steering) axis. The single front wheel is a child of the fork and rolls.
#   - Two pedal cranks are fixed to the front wheel axle (one per side, 180 deg
#     apart) and rotate with the front wheel as one rolling assembly.
#   - Each of the three wheels has a blue disc/hub, a black tire, and a small
#     off-axis blue valve-stem marker so AABB spin tests can detect rotation.
# Articulations:
#   - steering: REVOLUTE about the raked head-tube axis (~+/-45 deg).
#   - front wheel: CONTINUOUS roll about its axle (child of the fork).
#     The crank arms and pedals ride with this roll joint.
#   - rear-left wheel: CONTINUOUS roll.
#   - rear-right wheel: CONTINUOUS roll.

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
WHEEL_RADIUS = 0.060          # ~0.12 m diameter
TIRE_WIDTH = 0.034
HUB_RADIUS = 0.040
DISC_HALF = 0.013             # half-width of the blue side disc stack

FRONT_AXLE_Y = 0.235          # front axle, set ahead of the steer axis (trail)
REAR_AXLE_Y = -0.165
AXLE_Z = WHEEL_RADIUS         # axle height so wheels rest on the ground (z=0)

REAR_HALF_TRACK = 0.115       # half distance between the two rear wheels

# Pedal crank dimensions (front-wheel drive).
CRANK_LENGTH = 0.045          # radial distance from axle center to pedal center
CRANK_ARM_R = 0.005           # crank arm rod radius
PEDAL_R = 0.010               # pedal cylinder radius
PEDAL_LEN = 0.032             # pedal cylinder length along X (axle axis)
CRANK_SIDE_X = 0.035          # lateral offset of each crank from wheel center

# Steering head: tube tilted back (rake). Axis tilts in the Y-Z plane.
HEAD_TOP = (0.0, 0.150, 0.205)
HEAD_BOT = (0.0, 0.188, 0.090)
STEER_RAKE = math.atan2(HEAD_BOT[1] - HEAD_TOP[1], HEAD_TOP[2] - HEAD_BOT[2])
# Axis points up along the head tube (from bottom toward top).
_dy = HEAD_TOP[1] - HEAD_BOT[1]
_dz = HEAD_TOP[2] - HEAD_BOT[2]
_dn = math.hypot(_dy, _dz)
STEER_AXIS = (0.0, _dy / _dn, _dz / _dn)


def _spin_origin() -> Origin:
    # Rotate a Z/Y-built cylinder so its length lies along X (the roll axis).
    return Origin(rpy=(0.0, math.pi / 2.0, 0.0))


def _swept_tube(points, radius: float, offset=(0.0, 0.0, 0.0)) -> cq.Workplane:
    # Sweep a circle along a smooth spline through 3D points (CadQuery solid).
    # `offset` shifts every control point (used to author in a local frame).
    ox, oy, oz = offset
    pts = [(p[0] - ox, p[1] - oy, p[2] - oz) for p in points]
    path = cq.Workplane("XY").spline([cq.Vector(*p) for p in pts])
    start = pts[0]
    nxt = pts[1]
    tangent = cq.Vector(nxt[0] - start[0], nxt[1] - start[1], nxt[2] - start[2]).normalized()
    profile = (
        cq.Workplane(cq.Plane(origin=start, normal=(tangent.x, tangent.y, tangent.z)))
        .circle(radius)
    )
    return profile.sweep(path, transition="round")


def _cyl_between(a, b, radius, offset=(0.0, 0.0, 0.0)) -> cq.Workplane:
    # Straight cylinder spanning world points a->b, then shifted into a local
    # frame by subtracting `offset`. Robust alternative to short swept tubes.
    ox, oy, oz = offset
    ax, ay, az = a[0] - ox, a[1] - oy, a[2] - oz
    bx, by, bz = b[0] - ox, b[1] - oy, b[2] - oz
    dx, dy, dz = bx - ax, by - ay, bz - az
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    cyl = cq.Workplane("XY").circle(radius).extrude(length)
    # +Z extrusion -> align to (dx,dy,dz)
    elev = math.degrees(math.asin(max(-1.0, min(1.0, dz / length))))
    azim = math.degrees(math.atan2(dy, dx))
    cyl = cyl.rotate((0, 0, 0), (0, 1, 0), 90.0 - elev)
    cyl = cyl.rotate((0, 0, 0), (0, 0, 1), azim)
    return cyl.translate((ax, ay, az))


def _crank_pedal_pair(side_x: float, angle_rad: float, arm_name: str, pedal_name: str):
    """Build a crank arm and pedal mesh pair in the front wheel's local frame.

    The axle lies along X; the crank arm extends radially in the Y-Z plane.
    angle_rad=0 means the arm points in -Z (downward at rest); angle_rad=pi
    points in +Z (upward). Two calls with angles 0 and pi give 180 deg offset.

    A boss cylinder connects the crank arm to the hub disc, ensuring visual
    connectivity and mimicking the real axle-mount spacer.

    Args:
        side_x: lateral offset from wheel center (negative = left, positive = right).
        angle_rad: crank angle in the Y-Z plane.
        arm_name: managed mesh name for the crank arm.
        pedal_name: managed mesh name for the pedal.

    Returns:
        (arm_mesh, pedal_mesh) tuple.
    """
    sign = 1.0 if side_x > 0 else -1.0
    hub_face_x = TIRE_WIDTH / 2.0 - 0.002  # start slightly inside the hub disc
    boss_r = 0.008  # boss/spacer radius

    ey = CRANK_LENGTH * math.sin(angle_rad)
    ez = -CRANK_LENGTH * math.cos(angle_rad)

    # Boss/spacer: short cylinder along X from inside the hub to the crank arm
    boss = _cyl_between(
        (sign * hub_face_x, 0.0, 0.0),
        (side_x, 0.0, 0.0),
        boss_r,
    )

    # Crank arm: rod from axle center region to pedal position
    arm = _cyl_between(
        (side_x, 0.0, 0.0),
        (side_x, ey, ez),
        CRANK_ARM_R,
    )
    arm = boss.add(arm)
    arm_mesh = mesh_from_cadquery(arm, arm_name)

    # Pedal: cylinder along X (parallel to axle) at the crank end.
    pedal = (
        cq.Workplane("XY")
        .circle(PEDAL_R)
        .extrude(PEDAL_LEN)
        .translate((0.0, 0.0, -PEDAL_LEN / 2.0))
    )
    pedal = pedal.rotate((0, 0, 0), (0, 1, 0), 90.0)
    pedal = pedal.translate((side_x, ey, ez))
    pedal_mesh = mesh_from_cadquery(pedal, pedal_name)

    return arm_mesh, pedal_mesh


def _frame_mesh():
    # White tubular frame, authored in world coords as several robust solids
    # added as one CadQuery compound (no fragile short-spline booleans).
    backbone = _swept_tube(
        [
            (0.0, 0.165, 0.150),   # just below head top
            (0.0, 0.140, 0.120),
            (0.0, 0.070, 0.100),
            (0.0, -0.020, 0.092),
            (0.0, -0.110, 0.090),
            (0.0, REAR_AXLE_Y, 0.083),
        ],
        radius=0.018,
    )
    # Head tube (steering barrel) along the raked axis.
    head = _cyl_between(HEAD_BOT, HEAD_TOP, 0.022)
    backbone = backbone.add(head)
    # Rear axle bridge tube spanning the two rear wheels.
    axle_bridge = _cyl_between(
        (-REAR_HALF_TRACK, REAR_AXLE_Y, AXLE_Z),
        (REAR_HALF_TRACK, REAR_AXLE_Y, AXLE_Z),
        0.012,
    )
    backbone = backbone.add(axle_bridge)
    # Two short rear stays from the backbone end out to the axle ends.
    for sx in (-REAR_HALF_TRACK, REAR_HALF_TRACK):
        stay = _cyl_between((0.0, REAR_AXLE_Y, 0.083), (sx, REAR_AXLE_Y, AXLE_Z), 0.011)
        backbone = backbone.add(stay)
    return mesh_from_cadquery(backbone, "frame_tube")


def _saddle_mesh():
    # Blue padded saddle: a rounded teardrop pad lofted from a wide rear to a
    # narrow nose, sitting on top of the frame deck. Local frame at pad base.
    pad = (
        cq.Workplane("XY")
        .ellipse(0.052, 0.075)
        .workplane(offset=0.026)
        .ellipse(0.060, 0.085)
        .loft(ruled=False)
    )
    pad = pad.edges(">Z").fillet(0.018)
    return mesh_from_cadquery(pad, "saddle_pad")


def _fork_mesh(offset):
    # Blue front fork authored in world coords, shifted into the steering local
    # frame by `offset` (= HEAD_BOT). Steering stem + two blades + axle pin.
    # Steerer/neck: runs up through the head tube and on up to the handlebar
    # clamp so the bar and grips are part of the same connected island.
    stem = _cyl_between(
        (0.0, 0.150, 0.252),
        (HEAD_BOT[0], HEAD_BOT[1] + 0.018, HEAD_BOT[2] - 0.010),
        0.014,
        offset=offset,
    )
    fork = stem
    crown = (0.0, FRONT_AXLE_Y - 0.040, AXLE_Z + 0.085)
    # connect stem bottom to crown
    fork = fork.add(_cyl_between(
        (HEAD_BOT[0], HEAD_BOT[1] + 0.018, HEAD_BOT[2] - 0.010), crown, 0.012, offset=offset
    ))
    for sx in (-0.020, 0.020):
        blade = _cyl_between(crown, (sx, FRONT_AXLE_Y, AXLE_Z), 0.010, offset=offset)
        fork = fork.add(blade)
    axle = _cyl_between(
        (-0.030, FRONT_AXLE_Y, AXLE_Z), (0.030, FRONT_AXLE_Y, AXLE_Z), 0.008, offset=offset
    )
    fork = fork.add(axle)
    return mesh_from_cadquery(fork, "fork")


def _handlebar_mesh(offset):
    # Blue handlebar: a swept bar with two rises, authored in world coords and
    # shifted into the steering local frame by `offset`.
    bar = _swept_tube(
        [
            (-0.135, 0.150, 0.235),
            (-0.080, 0.140, 0.250),
            (-0.030, 0.150, 0.256),
            (0.030, 0.150, 0.256),
            (0.080, 0.140, 0.250),
            (0.135, 0.150, 0.235),
        ],
        radius=0.013,
        offset=offset,
    )
    return mesh_from_cadquery(bar, "handlebar_bar")


def _wheel_part(model, name, tire_mat, hub_mat, marker_mat):
    part = model.part(name)
    so = _spin_origin()
    # Black tire ring.
    tire = cq.Workplane("XY").circle(WHEEL_RADIUS).circle(HUB_RADIUS - 0.002).extrude(TIRE_WIDTH)
    tire = tire.translate((0.0, 0.0, -TIRE_WIDTH / 2.0))
    part.visual(
        mesh_from_cadquery(tire, f"{name}_tire"),
        origin=so,
        material=tire_mat,
        name=f"{name}_tire",
    )
    # Blue side discs (one each side) + central hub barrel.
    disc = cq.Workplane("XY").circle(HUB_RADIUS).extrude(DISC_HALF)
    disc = disc.translate((0.0, 0.0, TIRE_WIDTH / 2.0 - DISC_HALF))
    disc = disc.union(
        cq.Workplane("XY").circle(HUB_RADIUS).extrude(DISC_HALF).translate(
            (0.0, 0.0, -TIRE_WIDTH / 2.0)
        )
    )
    disc = disc.union(
        cq.Workplane("XY").circle(0.012).extrude(TIRE_WIDTH).translate((0.0, 0.0, -TIRE_WIDTH / 2.0))
    )
    part.visual(
        mesh_from_cadquery(disc, f"{name}_disc"),
        origin=so,
        material=hub_mat,
        name=f"{name}_disc",
    )
    # Off-axis valve-stem marker so AABB spin tests can detect rotation.
    marker = Cylinder(radius=0.004, length=TIRE_WIDTH * 0.9)
    part.visual(
        marker,
        origin=Origin(xyz=(0.0, HUB_RADIUS - 0.006, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=marker_mat,
        name=f"{name}_marker",
    )
    part.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=TIRE_WIDTH),
        mass=0.12,
        origin=so,
    )
    return part


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="baby_cycle")

    white = model.material("frame_white", rgba=(0.95, 0.95, 0.96, 1.0))
    blue = model.material("accent_blue", rgba=(0.16, 0.55, 0.80, 1.0))
    dark_blue = model.material("hub_blue", rgba=(0.13, 0.48, 0.74, 1.0))
    tire_black = model.material("tire_black", rgba=(0.10, 0.10, 0.11, 1.0))
    grip_blue = model.material("grip_blue", rgba=(0.20, 0.60, 0.84, 1.0))
    marker_blue = model.material("marker_blue", rgba=(0.10, 0.40, 0.66, 1.0))
    crank_steel = model.material("crank_steel", rgba=(0.72, 0.73, 0.74, 1.0))

    # ---- frame (root) ----
    frame = model.part("frame")
    frame.visual(_frame_mesh(), material=white, name="frame_tube")
    frame.inertial = Inertial.from_geometry(
        Box((0.26, 0.46, 0.18)), mass=1.4, origin=Origin(xyz=(0.0, 0.02, 0.10))
    )

    # ---- saddle (fixed to the frame deck) ----
    saddle = model.part("saddle")
    saddle.visual(_saddle_mesh(), material=blue, name="saddle_pad")
    saddle.inertial = Inertial.from_geometry(
        Box((0.12, 0.17, 0.06)), mass=0.15, origin=Origin(xyz=(0.0, 0.0, 0.013))
    )
    model.articulation(
        "frame_to_saddle",
        ArticulationType.FIXED,
        parent=frame,
        child=saddle,
        origin=Origin(xyz=(0.0, -0.085, 0.100)),
    )

    # ---- steering assembly: handlebar + fork turn together about the head tube ----
    # Geometry authored in world coords, then shifted into the steering local
    # frame (origin at HEAD_BOT) so the revolute pivot lies on the head tube.
    steer = model.part("steering")
    hb = HEAD_BOT
    steer.visual(_fork_mesh(hb), material=blue, name="fork")
    steer.visual(_handlebar_mesh(hb), material=blue, name="handlebar_bar")
    # Handlebar grips (separate visuals on each end), in the local frame.
    for sx, gname in ((-0.118, "grip_left"), (0.118, "grip_right")):
        steer.visual(
            Cylinder(radius=0.018, length=0.050),
            origin=Origin(
                xyz=(sx, 0.150 - hb[1], 0.236 - hb[2]), rpy=(0.0, math.pi / 2.0, 0.0)
            ),
            material=grip_blue,
            name=gname,
        )
    steer.inertial = Inertial.from_geometry(
        Box((0.27, 0.12, 0.22)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.16 - hb[1], 0.16 - hb[2])),
    )
    model.articulation(
        "steering",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=steer,
        origin=Origin(xyz=HEAD_BOT),
        axis=STEER_AXIS,
        motion_limits=MotionLimits(
            effort=4.0, velocity=4.0, lower=-math.pi / 4.0, upper=math.pi / 4.0
        ),
    )

    # ---- front wheel: child of the fork, rolls about its axle ----
    front_wheel = _wheel_part(model, "front_wheel", tire_black, dark_blue, marker_blue)
    model.articulation(
        "front_wheel_roll",
        ArticulationType.CONTINUOUS,
        parent=steer,
        child=front_wheel,
        origin=Origin(xyz=(0.0, FRONT_AXLE_Y - HEAD_BOT[1], AXLE_Z - HEAD_BOT[2])),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=20.0),
    )

    # ---- pedal cranks fixed to the front wheel axle ----
    # Two crank arms, 180 deg apart, with pedals at their ends.
    # They rotate with the front wheel as one rolling assembly.
    for i in range(2):
        side_x = -CRANK_SIDE_X + i * 2 * CRANK_SIDE_X  # -0.035, +0.035
        angle = i * math.pi  # 0 rad (down), pi rad (up)
        arm_mesh, pedal_mesh = _crank_pedal_pair(
            side_x, angle, f"crank_{i}", f"pedal_{i}"
        )
        front_wheel.visual(
            arm_mesh,
            material=crank_steel,
            name=f"crank_{i}",
        )
        front_wheel.visual(
            pedal_mesh,
            material=blue,
            name=f"pedal_{i}",
        )

    # ---- two rear wheels, each rolling about the rear axle ----
    rear_left = _wheel_part(model, "rear_left_wheel", tire_black, dark_blue, marker_blue)
    model.articulation(
        "rear_left_roll",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_left,
        origin=Origin(xyz=(REAR_HALF_TRACK, REAR_AXLE_Y, AXLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=20.0),
    )

    rear_right = _wheel_part(model, "rear_right_wheel", tire_black, dark_blue, marker_blue)
    model.articulation(
        "rear_right_roll",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_right,
        origin=Origin(xyz=(-REAR_HALF_TRACK, REAR_AXLE_Y, AXLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=20.0),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    saddle = object_model.get_part("saddle")
    steer = object_model.get_part("steering")
    front = object_model.get_part("front_wheel")
    rl = object_model.get_part("rear_left_wheel")
    rr = object_model.get_part("rear_right_wheel")

    steering = object_model.get_articulation("steering")
    front_roll = object_model.get_articulation("front_wheel_roll")
    rl_roll = object_model.get_articulation("rear_left_roll")
    rr_roll = object_model.get_articulation("rear_right_roll")

    # --- Intentional seated/captured overlaps at the mounting interfaces ---
    ctx.allow_overlap(
        frame, saddle,
        reason="The padded saddle is seated down onto the frame deck tube.",
    )
    ctx.allow_overlap(
        frame, steer,
        reason="The fork steerer passes down into the frame head tube.",
    )
    ctx.allow_overlap(
        frame, front,
        reason="The front fork blades (on the steering part) and the front "
        "tire share space around the front-wheel cutout; the axle seats here.",
    )
    ctx.allow_overlap(
        front, steer,
        reason="The fork blades straddle the front tire and capture the axle hub.",
    )
    ctx.allow_overlap(
        frame, rl,
        reason="The rear axle bridge is captured inside the left wheel hub bore.",
    )
    ctx.allow_overlap(
        frame, rr,
        reason="The rear axle bridge is captured inside the right wheel hub bore.",
    )

    # --- Hero parts present and placed ---
    saddle_pos = ctx.part_world_position(saddle)
    ctx.check(
        "saddle sits behind the head and above the deck",
        saddle_pos is not None and saddle_pos[1] < 0.0 and saddle_pos[2] > 0.08,
        details=f"saddle_pos={saddle_pos}",
    )
    ctx.expect_contact(saddle, frame, name="saddle mounted on frame")

    # Steering fork connects to frame; handlebar is up high and to the front.
    steer_aabb = ctx.part_world_aabb(steer)
    ctx.check(
        "handlebar reaches grip height",
        steer_aabb is not None and steer_aabb[1][2] > 0.22,
        details=f"steer_aabb={steer_aabb}",
    )
    ctx.expect_contact(steer, frame, name="fork/steering mounted to frame head tube")

    # --- Bike rests on all three wheels (tires reach the ground plane z~0) ---
    for wname, w in (("front", front), ("rear_left", rl), ("rear_right", rr)):
        ab = ctx.part_world_aabb(w)
        ctx.check(
            f"{wname} wheel rests on ground",
            ab is not None and ab[0][2] < 0.004,
            details=f"{wname}_aabb_min_z={None if ab is None else ab[0][2]}",
        )

    # Single front wheel, two rear wheels: rear wheels symmetric about x=0.
    fp = ctx.part_world_position(front)
    rlp = ctx.part_world_position(rl)
    rrp = ctx.part_world_position(rr)
    ctx.check(
        "front wheel is centered up front",
        fp is not None and abs(fp[0]) < 0.01 and fp[1] > 0.1,
        details=f"front_pos={fp}",
    )
    ctx.check(
        "two rear wheels are symmetric across centerline",
        rlp is not None and rrp is not None
        and abs(rlp[0] + rrp[0]) < 0.005
        and abs(rlp[0] - rrp[0]) > 0.18,
        details=f"rl={rlp}, rr={rrp}",
    )

    # --- Steering turns the front wheel left/right about the raked axis ---
    front_x0 = ctx.part_world_position(front)[0]
    with ctx.pose({steering: math.pi / 4.0}):
        front_x_left = ctx.part_world_position(front)[0]
    with ctx.pose({steering: -math.pi / 4.0}):
        front_x_right = ctx.part_world_position(front)[0]
    ctx.check(
        "steering swings the front wheel sideways",
        abs(front_x_left - front_x0) > 0.015 and abs(front_x_right - front_x0) > 0.015
        and (front_x_left - front_x0) * (front_x_right - front_x0) < 0,
        details=f"x0={front_x0}, left={front_x_left}, right={front_x_right}",
    )

    # --- Each wheel rolls about its axle: the off-center marker moves ---
    for rname, roll, w in (
        ("front", front_roll, front),
        ("rear_left", rl_roll, rl),
        ("rear_right", rr_roll, rr),
    ):
        m0 = ctx.part_element_world_aabb(w, elem=f"{w.name}_marker")
        with ctx.pose({roll: math.pi / 2.0}):
            m1 = ctx.part_element_world_aabb(w, elem=f"{w.name}_marker")
        c0 = None if m0 is None else (0.5 * (m0[0][1] + m0[1][1]), 0.5 * (m0[0][2] + m0[1][2]))
        c1 = None if m1 is None else (0.5 * (m1[0][1] + m1[1][1]), 0.5 * (m1[0][2] + m1[1][2]))
        moved = (
            c0 is not None
            and c1 is not None
            and (abs(c0[0] - c1[0]) > 0.01 or abs(c0[1] - c1[1]) > 0.01)
        )
        ctx.check(
            f"{rname} wheel marker moves when the wheel rolls",
            moved,
            details=f"marker_center rest={c0}, quarter_turn={c1}",
        )

    # --- Pedal cranks exist on the front wheel and rotate with it ---
    crank0 = front.get_visual("crank_0")
    crank1 = front.get_visual("crank_1")
    pedal0 = front.get_visual("pedal_0")
    pedal1 = front.get_visual("pedal_1")
    ctx.check(
        "crank_0 visual exists on front wheel",
        crank0 is not None,
    )
    ctx.check(
        "crank_1 visual exists on front wheel",
        crank1 is not None,
    )
    ctx.check(
        "pedal_0 visual exists on front wheel",
        pedal0 is not None,
    )
    ctx.check(
        "pedal_1 visual exists on front wheel",
        pedal1 is not None,
    )

    # Cranks are 180 deg apart: at rest, one pedal is below axle, one above
    if pedal0 is not None and pedal1 is not None:
        p0_aabb = ctx.part_element_world_aabb(front, elem="pedal_0")
        p1_aabb = ctx.part_element_world_aabb(front, elem="pedal_1")
        if p0_aabb is not None and p1_aabb is not None:
            p0_z_center = 0.5 * (p0_aabb[0][2] + p0_aabb[1][2])
            p1_z_center = 0.5 * (p1_aabb[0][2] + p1_aabb[1][2])
            ctx.check(
                "pedals are 180 deg apart (one up, one down at rest)",
                abs(p0_z_center - p1_z_center) > 0.04,
                details=f"pedal_0_z={p0_z_center:.4f}, pedal_1_z={p1_z_center:.4f}",
            )

    # Cranks rotate with the front wheel roll joint
    if crank0 is not None:
        c0_rest = ctx.part_element_world_aabb(front, elem="crank_0")
        with ctx.pose({front_roll: math.pi / 2.0}):
            c0_rotated = ctx.part_element_world_aabb(front, elem="crank_0")
        if c0_rest is not None and c0_rotated is not None:
            rest_z = 0.5 * (c0_rest[0][2] + c0_rest[1][2])
            rot_z = 0.5 * (c0_rotated[0][2] + c0_rotated[1][2])
            ctx.check(
                "crank_0 rotates with front wheel roll",
                abs(rest_z - rot_z) > 0.01,
                details=f"rest_z={rest_z:.4f}, rotated_z={rot_z:.4f}",
            )

    return ctx.report()


object_model = build_object_model()
