from __future__ import annotations

# Toddler balance "baby cycle" ride-on.
# Object frame:
#   +X = right, +Y = forward (front of the cycle), +Z = up.
# Layout:
#   - White tubular frame is the root. It runs from a tilted head tube at the
#     front (+Y) back to a low saddle deck and a rear axle that carries TWO
#     splayed rear wheels.
#   - A blue handlebar + front fork turn together about the tilted head-tube
#     (steering) axis. The single front wheel is a child of the fork and rolls.
#   - Each of the three wheels has a blue disc/hub, a black tire, and a small
#     off-axis blue valve-stem marker so AABB spin tests can detect rotation.
# Articulations:
#   - steering: REVOLUTE about the raked head-tube axis (~+/-45 deg).
#   - front wheel: CONTINUOUS roll about its axle (child of the fork).
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

    return ctx.report()


object_model = build_object_model()
