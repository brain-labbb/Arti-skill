from __future__ import annotations

# Orange hardtail mountain bike (Trek-style).
# Frame convention:
#   +X = forward (toward the front wheel), +Z = up, +Y = left.
#   Wheels touch the ground at z=0; both axles sit at z = WHEEL_R.
#   Rear axle at x=0, front axle at x=WHEELBASE.
# Articulations:
#   - steering: handlebars+stem+fork+front wheel pivot about the head-tube axis
#     (REVOLUTE, tilted axis in the XZ plane, ~+/-40 deg).
#   - front wheel: CONTINUOUS roll about its axle (child of the fork).
#   - rear wheel: CONTINUOUS roll about its axle.
#   - crank (chainring + 2 crank arms + 2 pedals): CONTINUOUS rotation about the
#     bottom-bracket axis. A small off-axis pedal/marker makes the spin detectable.

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---- key geometry constants (meters) ----
WHEEL_R = 0.35  # wheel radius -> ~0.70 m diameter
TIRE_T = 0.028  # tire cross-section (tube) radius
WHEELBASE = 1.05
REAR_AXLE = (0.0, 0.0, WHEEL_R)
FRONT_AXLE = (WHEELBASE, 0.0, WHEEL_R)
BB = (0.49, 0.0, 0.30)  # bottom bracket (crank axis); chainstay length clears rear tire

# Head tube: top and bottom points; the steering axis runs along this line.
# Placed so the head tube clears the front-wheel circle (center FRONT_AXLE,
# radius ~0.36); the fork rakes forward from here to the axle at x=1.05.
HEAD_TOP = (0.80, 0.0, 1.00)
HEAD_BOT = (0.86, 0.0, 0.70)
SEAT_TOP = (0.085, 0.0, 1.02)  # top of seat tube / seatpost clamp

WHEEL_HALF = 0.026  # half hub/axle width offset for spokes


def _steer_axis() -> tuple[float, float, float]:
    dx = HEAD_TOP[0] - HEAD_BOT[0]
    dz = HEAD_TOP[2] - HEAD_BOT[2]
    n = (dx * dx + dz * dz) ** 0.5
    return (dx / n, 0.0, dz / n)


def _wheel_geometry(prefix: str, tire_mat, rim_mat, hub_mat, marker_mat):
    """Return a list of (geometry_or_box, origin, material, name) visuals for a
    wheel spinning about the local Y axis (roll). Built in the wheel's own frame
    centered on the axle, axle along +/-Y."""
    visuals = []

    # Tire: torus in the XZ plane (axle along Y) -> rotate torus about X by 90deg.
    tire = TorusGeometry(WHEEL_R - TIRE_T, TIRE_T, radial_segments=20, tubular_segments=56)
    tire = tire.rotate_x(pi / 2.0)
    visuals.append((mesh_from_geometry(tire, f"{prefix}_tire"), None, tire_mat, f"{prefix}_tire"))

    # Rim: thin torus just inside the tire.
    rim = TorusGeometry(WHEEL_R - 2.0 * TIRE_T, 0.012, radial_segments=12, tubular_segments=56)
    rim = rim.rotate_x(pi / 2.0)
    visuals.append((mesh_from_geometry(rim, f"{prefix}_rim"), None, rim_mat, f"{prefix}_rim"))

    # Hub: short cylinder along the axle (Y).
    hub = CylinderGeometry(0.026, 0.090, radial_segments=20).rotate_x(pi / 2.0)
    visuals.append((mesh_from_geometry(hub, f"{prefix}_hub"), None, hub_mat, f"{prefix}_hub"))

    # Spokes: many thin wire spokes radiating from the hub flange out to the
    # rim, like a real bicycle wheel. Each is a slender steel cylinder spanning
    # the full hub->rim gap in the XZ plane (axle along Y).
    n_spokes = 28
    spoke_inner = 0.030  # leaves the hub flange (hub radius ~0.026)
    spoke_outer = WHEEL_R - 2.0 * TIRE_T  # meets the rim torus, just inside the tire
    for i in range(n_spokes):
        # small angular offset so no spoke lies exactly on a coordinate axis
        a = 2.0 * pi * i / n_spokes + 0.18
        r_mid = (spoke_inner + spoke_outer) * 0.5
        length = spoke_outer - spoke_inner
        # spoke cylinder oriented along its radial direction in XZ
        cx, cz = cos(a) * r_mid, sin(a) * r_mid
        sp = CylinderGeometry(0.0022, length, radial_segments=5)
        # CylinderGeometry's long axis is Z by default. Spin it about Y so the
        # axis points radially at angle a -> (cos a, 0, sin a), then translate to
        # the radial midpoint so it spans hub flange -> rim in the wheel plane.
        sp = sp.rotate_y(pi / 2.0 - a)
        sp = sp.translate(cx, 0.0, cz)
        visuals.append((mesh_from_geometry(sp, f"{prefix}_spoke_{i}"), None, rim_mat, f"{prefix}_spoke_{i}"))

    # Disc-brake rotor: thin disc just outboard of the hub flange (touches hub).
    rotor = CylinderGeometry(0.090, 0.004, radial_segments=40).rotate_x(pi / 2.0)
    rotor = rotor.translate(0.0, 0.044, 0.0)
    visuals.append((mesh_from_geometry(rotor, f"{prefix}_rotor"), None, hub_mat, f"{prefix}_rotor"))

    # Valve/marker: small off-axis nub on the rim so spin is detectable by AABB.
    visuals.append(
        (
            Box((0.020, 0.020, 0.045)),
            Origin(xyz=(0.0, 0.0, spoke_outer + 0.005)),
            marker_mat,
            f"{prefix}_marker",
        )
    )
    return visuals


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mountain_bike")

    orange = model.material("frame_orange", rgba=(0.95, 0.46, 0.10, 1.0))
    tire_black = model.material("tire_black", rgba=(0.07, 0.07, 0.08, 1.0))
    component_black = model.material("component_black", rgba=(0.12, 0.13, 0.14, 1.0))
    silver = model.material("silver", rgba=(0.74, 0.76, 0.79, 1.0))
    rim_silver = model.material("rim_silver", rgba=(0.55, 0.57, 0.60, 1.0))
    red = model.material("accent_red", rgba=(0.80, 0.12, 0.10, 1.0))

    # =====================================================================
    # FRAME (root): orange diamond frame + seat tube + chainstays + seatstays,
    # head tube, bottom-bracket shell, seatpost + saddle, rear-disc caliper.
    # =====================================================================
    frame = model.part("frame")

    def tube(p_from, p_to, r, name, mat, mid=None, extra=None):
        pts = [p_from]
        if mid:
            pts.extend(mid)
        pts.append(p_to)
        if extra:
            pts = extra
        g = tube_from_spline_points(pts, radius=r, samples_per_segment=10, radial_segments=14)
        frame.visual(mesh_from_geometry(g, name), material=mat, name=name)

    # Top tube: from the head-tube top back to the top of the seat tube.
    tube((0.81, 0.0, 0.985), (0.115, 0.0, 1.005), 0.024, "top_tube", orange)
    # Down tube: head-tube bottom down to the bottom bracket.
    tube((0.85, 0.0, 0.70), BB, 0.028, "down_tube", orange)
    # Seat tube: bottom bracket up to seatpost clamp. Kept near-vertical at the
    # bottom (x~0.40) so it clears the rear tire (tire reaches only x~0.35).
    tube(BB, (0.12, 0.0, 1.00), 0.024, "seat_tube", orange, mid=[(0.47, 0.0, 0.66)])
    # Head tube: short stout tube along the steering axis.
    tube(HEAD_BOT, HEAD_TOP, 0.030, "head_tube", orange)

    # Chainstays: BB back to rear axle, left and right. Rear ends splay outboard
    # so they clear the rear cassette/tire at the dropouts.
    for sgn, nm in ((1.0, "chainstay_left"), (-1.0, "chainstay_right")):
        tube(
            (BB[0], sgn * 0.040, BB[2]),
            (REAR_AXLE[0], sgn * 0.050, REAR_AXLE[2]),
            0.014,
            nm,
            orange,
            mid=[(0.28, sgn * 0.062, 0.31)],
        )
    # Seatstays: top of seat tube down to rear axle, left and right.
    for sgn, nm in ((1.0, "seatstay_left"), (-1.0, "seatstay_right")):
        tube(
            (0.12, sgn * 0.020, 0.98),
            (REAR_AXLE[0], sgn * 0.050, REAR_AXLE[2]),
            0.013,
            nm,
            orange,
            mid=[(0.07, sgn * 0.046, 0.62)],
        )

    # Bottom-bracket shell.
    bb_shell = CylinderGeometry(0.030, 0.070, radial_segments=24).rotate_x(pi / 2.0)
    bb_shell = bb_shell.translate(*BB)
    frame.visual(mesh_from_geometry(bb_shell, "bb_shell"), material=component_black, name="bb_shell")

    # Seatpost: inserted into the seat-tube top and rising to the saddle.
    seatpost = tube_from_spline_points(
        [(0.115, 0.0, 0.97), (0.10, 0.0, 1.05), (0.085, 0.0, 1.14)],
        radius=0.014,
        samples_per_segment=8,
        radial_segments=14,
    )
    frame.visual(mesh_from_geometry(seatpost, "seatpost"), material=silver, name="seatpost")
    saddle = Box((0.24, 0.11, 0.04))
    frame.visual(saddle, origin=Origin(xyz=(0.060, 0.0, 1.150)), material=component_black, name="saddle")
    saddle_nose = Box((0.10, 0.04, 0.035))
    frame.visual(
        saddle_nose, origin=Origin(xyz=(0.165, 0.0, 1.148)), material=component_black, name="saddle_nose"
    )

    # Rear disc-brake caliper, mounted on the left seatstay near the axle.
    rear_caliper = Box((0.05, 0.05, 0.08))
    frame.visual(
        rear_caliper, origin=Origin(xyz=(0.052, 0.060, 0.45)), material=component_black, name="rear_caliper"
    )

    frame.inertial = Inertial.from_geometry(
        Box((1.0, 0.12, 0.9)), mass=9.0, origin=Origin(xyz=(0.45, 0.0, 0.65))
    )

    # =====================================================================
    # FORK (steered): suspension fork + stem + flat handlebars. Child of frame,
    # pivots about the head-tube axis. Built in frame coordinates; the steering
    # joint origin sits on the head-tube axis.
    # =====================================================================
    fork = model.part("fork")
    steer_axis = _steer_axis()

    # Fork is a CHILD of the frame attached at the head-tube bottom (HEAD_BOT).
    # In URDF the child link frame sits at the joint origin, so all fork geometry
    # is authored in absolute frame coords and then shifted by -HEAD_BOT.
    HB = HEAD_BOT

    def _sub_pts(pts):
        return [(x - HB[0], y - HB[1], z - HB[2]) for (x, y, z) in pts]

    def _sub_xyz(x, y, z):
        return (x - HB[0], y - HB[1], z - HB[2])

    def fork_tube(pts, r, name, mat):
        g = tube_from_spline_points(_sub_pts(pts), radius=r, samples_per_segment=8, radial_segments=12)
        fork.visual(mesh_from_geometry(g, name), material=mat, name=name)

    # Steerer tube: runs through the head tube up to the stem (along the steer
    # axis). Starts at the crown (z=0.75) so it clears the down-tube junction.
    fork_tube([(0.84, 0.0, 0.75), (0.795, 0.0, 1.045)], 0.018, "steerer", component_black)

    # Fork crown sits just above the front tire and ties the steerer to the legs.
    crown = Box((0.11, 0.15, 0.05))
    fork.visual(crown, origin=Origin(xyz=_sub_xyz(0.875, 0.0, 0.745)), material=component_black, name="fork_crown")
    # Rigid fork blades: one continuous curved blade per side, running from the
    # crown down and forward to the front-wheel dropout with a gentle forward
    # rake. No telescoping stanchion, no seal, no sliding joint — just a slim
    # single-piece blade like a carbon or chromoly rigid fork.
    for sgn, side in ((1.0, "left"), (-1.0, "right")):
        fork_tube(
            [
                (0.905, sgn * 0.060, 0.735),   # root at crown
                (0.935, sgn * 0.060, 0.64),    # upper blade, still near-straight
                (0.970, sgn * 0.060, 0.54),    # mid blade, curving forward
                (1.010, sgn * 0.060, 0.43),    # lower blade, raked forward
                (FRONT_AXLE[0], sgn * 0.060, FRONT_AXLE[2]),  # dropout at axle
            ],
            0.015,
            f"fork_blade_{side}",
            component_black,
        )

    # Front disc-brake caliper mounted on the left fork lower, near the rotor.
    front_caliper = Box((0.045, 0.045, 0.075))
    fork.visual(
        front_caliper,
        origin=Origin(xyz=_sub_xyz(1.005, 0.075, 0.44)),
        material=component_black,
        name="front_caliper",
    )

    # Stem: clamps the steerer top and carries the handlebars (above the top tube).
    stem = Box((0.11, 0.045, 0.04))
    fork.visual(stem, origin=Origin(xyz=_sub_xyz(0.81, 0.0, 1.045)), material=component_black, name="stem")

    # Flat handlebars: a roughly straight bar across +/-Y with slight back sweep.
    fork_tube(
        [
            (0.785, 0.30, 1.035),
            (0.795, 0.18, 1.042),
            (0.80, 0.0, 1.048),
            (0.795, -0.18, 1.042),
            (0.785, -0.30, 1.035),
        ],
        0.013,
        "handlebar",
        component_black,
    )
    for sgn, side in ((1.0, "left"), (-1.0, "right")):
        grip = Cylinder(radius=0.018, length=0.11)
        fork.visual(
            grip,
            origin=Origin(xyz=_sub_xyz(0.786, sgn * 0.275, 1.036), rpy=(pi / 2.0, 0.0, 0.0)),
            material=component_black,
            name=f"grip_{side}",
        )

    fork.inertial = Inertial.from_geometry(
        Box((0.20, 0.62, 0.55)), mass=3.0, origin=Origin(xyz=_sub_xyz(0.93, 0.0, 0.80))
    )

    # =====================================================================
    # WHEELS
    # =====================================================================
    front_wheel = model.part("front_wheel")
    for geom, origin, mat, nm in _wheel_geometry("front", tire_black, rim_silver, component_black, red):
        front_wheel.visual(geom, origin=origin, material=mat, name=nm)
    front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_R, length=0.09), mass=2.2, origin=Origin(rpy=(pi / 2.0, 0.0, 0.0))
    )

    rear_wheel = model.part("rear_wheel")
    for geom, origin, mat, nm in _wheel_geometry("rear", tire_black, rim_silver, component_black, red):
        rear_wheel.visual(geom, origin=origin, material=mat, name=nm)
    # Rear cassette: stack of small discs on the right of the hub.
    rear_wheel.visual(
        Cylinder(radius=0.042, length=0.024),
        origin=Origin(xyz=(0.0, -0.014, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="rear_cassette",
    )
    rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_R, length=0.09), mass=2.4, origin=Origin(rpy=(pi / 2.0, 0.0, 0.0))
    )

    # =====================================================================
    # CRANK assembly: chainring + 2 crank arms + 2 pedals, spinning about the BB
    # axis (Y). Built centered on the BB; the joint origin is at the BB.
    # =====================================================================
    crank = model.part("crank")
    # Spindle: the crank axle running through the bottom-bracket shell; this is
    # what physically supports the crank in the frame.
    spindle = CylinderGeometry(0.014, 0.20, radial_segments=18).rotate_x(pi / 2.0)
    crank.visual(mesh_from_geometry(spindle, "spindle"), material=silver, name="spindle")
    # Chainring on the right (-Y) side, outboard of the right chainstay and
    # inboard of the right crank arm.
    chainring = CylinderGeometry(0.095, 0.006, radial_segments=44).rotate_x(pi / 2.0)
    chainring = chainring.translate(0.0, -0.078, 0.0)
    crank.visual(mesh_from_geometry(chainring, "chainring"), material=silver, name="chainring")
    # Crank arms point opposite each other (180deg apart) in the XZ plane. Each
    # arm is a tube from the spindle end out to its pedal so it stays connected.
    for ay, side, arm_dir in ((-0.092, "right", 1.0), (0.092, "left", -1.0)):
        px = arm_dir * 0.085
        pz = arm_dir * -0.130  # opposite vertical -> arms 180deg apart
        # Arm root sits at the spindle end (outboard of the chainstays) so it
        # clears them; it then reaches out to the pedal.
        arm = tube_from_spline_points(
            [(0.0, ay * 0.95, 0.0), (arm_dir * 0.045, ay, -pz * 0.10), (px, ay, pz)],
            radius=0.013,
            samples_per_segment=8,
            radial_segments=12,
        )
        crank.visual(
            mesh_from_geometry(arm, f"crank_arm_{side}"),
            material=component_black,
            name=f"crank_arm_{side}",
        )
        # Pedal at the end of each arm, offset off the BB axis (the spin marker).
        pedal = Box((0.078, 0.070, 0.018))
        crank.visual(
            pedal,
            origin=Origin(xyz=(px, ay + arm_dir * 0.035, pz)),
            material=component_black,
            name=f"pedal_{side}",
        )
    crank.inertial = Inertial.from_geometry(
        Cylinder(radius=0.10, length=0.18), mass=1.2, origin=Origin(rpy=(pi / 2.0, 0.0, 0.0))
    )

    # =====================================================================
    # ARTICULATIONS
    # =====================================================================
    # Steering: fork pivots about the head-tube axis.
    model.articulation(
        "steering",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=fork,
        origin=Origin(xyz=(HEAD_BOT[0], HEAD_BOT[1], HEAD_BOT[2])),
        axis=steer_axis,
        motion_limits=MotionLimits(effort=12.0, velocity=3.0, lower=-0.70, upper=0.70),
    )
    # Front wheel rolls about its axle (Y); child of the fork.
    # Front wheel is a child of the fork; its joint origin is the axle expressed
    # in the fork frame (absolute axle minus the steering joint origin HEAD_BOT).
    front_axle_in_fork = (
        FRONT_AXLE[0] - HEAD_BOT[0],
        FRONT_AXLE[1] - HEAD_BOT[1],
        FRONT_AXLE[2] - HEAD_BOT[2],
    )
    model.articulation(
        "front_wheel_roll",
        ArticulationType.CONTINUOUS,
        parent=fork,
        child=front_wheel,
        origin=Origin(xyz=front_axle_in_fork),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=40.0),
    )
    # Rear wheel rolls about its axle (Y); child of the frame.
    model.articulation(
        "rear_wheel_roll",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_wheel,
        origin=Origin(xyz=REAR_AXLE),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=40.0),
    )
    # Crank spins about the bottom-bracket axis (Y).
    model.articulation(
        "crank_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=crank,
        origin=Origin(xyz=BB),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=20.0),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def _center(aabb):
    mn, mx = aabb
    return ((mn[0] + mx[0]) * 0.5, (mn[1] + mx[1]) * 0.5, (mn[2] + mx[2]) * 0.5)


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    fork = object_model.get_part("fork")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")
    crank = object_model.get_part("crank")

    steering = object_model.get_articulation("steering")
    front_roll = object_model.get_articulation("front_wheel_roll")
    rear_roll = object_model.get_articulation("rear_wheel_roll")
    crank_spin = object_model.get_articulation("crank_spin")

    # ---- both wheels same diameter (~0.70 m) ----
    fw_ext = _ext(ctx.part_world_aabb(front_wheel))
    rw_ext = _ext(ctx.part_world_aabb(rear_wheel))
    fw_dia = max(fw_ext[0], fw_ext[2])
    rw_dia = max(rw_ext[0], rw_ext[2])
    ctx.check(
        "front wheel ~0.70m diameter",
        0.66 <= fw_dia <= 0.74,
        details=f"front diameter={fw_dia:.3f}",
    )
    ctx.check(
        "wheels share the same diameter",
        abs(fw_dia - rw_dia) < 0.02,
        details=f"front={fw_dia:.3f}, rear={rw_dia:.3f}",
    )

    # ---- frame connects head tube <-> BB <-> rear axle ----
    head_tube = frame.get_visual("head_tube")
    down_tube = frame.get_visual("down_tube")
    chainstay = frame.get_visual("chainstay_left")
    ctx.expect_contact(
        frame, frame, elem_a="down_tube", elem_b="head_tube", name="down tube meets head tube"
    )
    ctx.expect_contact(
        frame, frame, elem_a="down_tube", elem_b="bb_shell", name="down tube meets bottom bracket"
    )
    ctx.expect_contact(
        frame, frame, elem_a="chainstay_left", elem_b="bb_shell", name="chainstay meets bottom bracket"
    )

    # ---- front wheel mounted at the front, rear at the back ----
    fw_c = _center(ctx.part_world_aabb(front_wheel))
    rw_c = _center(ctx.part_world_aabb(rear_wheel))
    ctx.check(
        "front wheel ahead of rear wheel by ~wheelbase",
        0.95 <= (fw_c[0] - rw_c[0]) <= 1.15,
        details=f"front_x={fw_c[0]:.3f}, rear_x={rw_c[0]:.3f}",
    )

    # ---- steering rotates the front wheel about the head-tube axis (yaws it) ----
    fw_ext0 = _ext(ctx.part_world_aabb(front_wheel))
    with ctx.pose({steering: 0.70}):
        fw_ext_s = _ext(ctx.part_world_aabb(front_wheel))
    # Steering swings the wheel about a near-vertical axis -> Y extent grows.
    ctx.check(
        "steering yaws the front wheel",
        fw_ext_s[1] > fw_ext0[1] + 0.05,
        details=f"rest Y={fw_ext0[1]:.3f}, steered Y={fw_ext_s[1]:.3f}",
    )

    # ---- front wheel rolls about its axle (rim marker swings around) ----
    fm0 = _center(ctx.part_element_world_aabb(front_wheel, elem="front_marker"))
    with ctx.pose({front_roll: pi / 2.0}):
        fm1 = _center(ctx.part_element_world_aabb(front_wheel, elem="front_marker"))
    fmoved = ((fm1[0] - fm0[0]) ** 2 + (fm1[2] - fm0[2]) ** 2) ** 0.5
    ctx.check(
        "front wheel rolls (rim marker swings)",
        fmoved > 0.20,
        details=f"marker moved {fmoved:.3f} (rest={fm0}, rolled={fm1})",
    )

    # ---- rear wheel rolls about its axle (rim marker swings around) ----
    rm0 = _center(ctx.part_element_world_aabb(rear_wheel, elem="rear_marker"))
    with ctx.pose({rear_roll: pi / 2.0}):
        rm1 = _center(ctx.part_element_world_aabb(rear_wheel, elem="rear_marker"))
    rmoved = ((rm1[0] - rm0[0]) ** 2 + (rm1[2] - rm0[2]) ** 2) ** 0.5
    ctx.check(
        "rear wheel rolls (rim marker swings)",
        rmoved > 0.20,
        details=f"marker moved {rmoved:.3f} (rest={rm0}, rolled={rm1})",
    )

    # ---- crank rotates about the bottom bracket (pedal marker swings) ----
    c0 = _center(ctx.part_world_aabb(crank))
    pm0 = _center(ctx.part_element_world_aabb(crank, elem="pedal_right"))
    with ctx.pose({crank_spin: pi / 2.0}):
        pm1 = _center(ctx.part_element_world_aabb(crank, elem="pedal_right"))
    pmoved = ((pm1[0] - pm0[0]) ** 2 + (pm1[2] - pm0[2]) ** 2) ** 0.5
    ctx.check(
        "crank spins about the bottom bracket (pedal swings)",
        pmoved > 0.08,
        details=f"pedal moved {pmoved:.3f} (rest={pm0}, spun={pm1})",
    )
    # crank centered near the BB
    ctx.check(
        "crank mounted at the bottom bracket",
        abs(c0[0] - BB[0]) < 0.06 and abs(c0[2] - BB[2]) < 0.06,
        details=f"crank center={c0}",
    )

    # ---- mounting / seated overlaps are intentional ----
    for elem_b, why in (
        ("head_tube", "Steerer tube runs through the head tube (telescoping headset fit)."),
        ("top_tube", "Steerer passes the head-tube/top-tube weld junction at the headset top."),
    ):
        ctx.allow_overlap(fork, frame, elem_a="steerer", elem_b=elem_b, reason=why)
    for elem_b, why in (
        ("bb_shell", "Crank spindle runs through the bottom-bracket shell bearings."),
        ("down_tube", "Down tube welds into the bottom-bracket junction around the spindle."),
        ("seat_tube", "Seat tube welds into the bottom-bracket junction around the spindle."),
        ("chainstay_left", "Left chainstay welds into the bottom-bracket shell around the spindle."),
        ("chainstay_right", "Right chainstay welds into the bottom-bracket shell around the spindle."),
    ):
        ctx.allow_overlap(crank, frame, elem_a="spindle", elem_b=elem_b, reason=why)
    ctx.expect_contact(
        crank, frame, elem_a="spindle", elem_b="bb_shell", name="crank spindle in bottom bracket"
    )
    ctx.allow_overlap(
        fork,
        frame,
        elem_a="fork_crown",
        elem_b="head_tube",
        reason="Fork crown wraps the head-tube/headset bottom just under the lower bearing.",
    )
    # Prove the crown is actually seated against the head tube (not floating).
    ctx.expect_contact(
        fork, frame, elem_a="fork_crown", elem_b="head_tube", name="fork crown seated at headset"
    )

    # Rear-wheel dropouts: the chainstay/seatstay ends grip the rear hub axle.
    for elem_a in ("chainstay_left", "chainstay_right", "seatstay_left", "seatstay_right"):
        ctx.allow_overlap(
            frame,
            rear_wheel,
            elem_a=elem_a,
            elem_b="rear_hub",
            reason="Rear dropout grips the rear-wheel hub axle.",
        )
    ctx.expect_contact(frame, rear_wheel, elem_b="rear_hub", name="rear wheel mounted in dropouts")
    # Front-wheel dropouts: the rigid fork blades grip the front hub axle.
    for elem_a in ("fork_blade_left", "fork_blade_right"):
        ctx.allow_overlap(
            fork,
            front_wheel,
            elem_a=elem_a,
            elem_b="front_hub",
            reason="Rigid fork-blade dropout grips the front-wheel hub axle.",
        )
    ctx.expect_contact(fork, front_wheel, elem_b="front_hub", name="front wheel mounted in fork")

    # ---- Rigid fork: one continuous blade per side, no telescoping joint ----
    fork_visual_names = {v.name for v in fork.visuals}
    for side in ("left", "right"):
        ctx.check(
            f"rigid fork has continuous {side} blade",
            f"fork_blade_{side}" in fork_visual_names,
            details=f"visual 'fork_blade_{side}' not found on fork",
        )
        ctx.check(
            f"no suspension stanchion on {side} (rigid fork)",
            f"fork_stanchion_{side}" not in fork_visual_names,
            details=f"fork_stanchion_{side} should not exist on a rigid fork",
        )
        ctx.check(
            f"no separate lower leg on {side} (rigid fork)",
            f"fork_lower_{side}" not in fork_visual_names,
            details=f"fork_lower_{side} should not exist on a rigid fork",
        )

    # Blade reaches from crown down to the front-axle dropout (retained insertion
    # along X with the hub) while still clearing the tire in Z at the crown end.
    ctx.expect_contact(
        fork, front_wheel, elem_a="fork_blade_left", elem_b="front_hub",
        name="left rigid blade reaches the front axle dropout",
    )
    ctx.expect_contact(
        fork, front_wheel, elem_a="fork_blade_right", elem_b="front_hub",
        name="right rigid blade reaches the front axle dropout",
    )

    return ctx.report()


object_model = build_object_model()
