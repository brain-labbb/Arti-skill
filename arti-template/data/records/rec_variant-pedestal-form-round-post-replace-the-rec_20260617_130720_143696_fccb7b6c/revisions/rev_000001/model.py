from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    DomeGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# World frame: +X = right, -X = left, +Y = toward the walker (lane runs along
# Y, the walking direction), +Z = up. This is a DUAL-LANE twin tripod
# turnstile with a ROUND CYLINDRICAL POST pedestal: one central post carrying
# a tripod hub on its LEFT face and one on its RIGHT face, flanked by two
# free-standing railings that run parallel to the lane on the far left and
# far right.
# ---------------------------------------------------------------------------

# Round post pedestal dimensions.
BASE_PLATE_R = 0.20  # round base plate radius
BASE_PLATE_H = 0.028  # base plate thickness
POST_R = 0.075  # post outer radius (150 mm diameter)
POST_Z0 = BASE_PLATE_H  # post starts on top of the base plate
POST_Z1 = 0.96  # post top (just below 1 m)
POST_COLLAR_R = 0.085  # slight collar ring at the hub mounting band
POST_COLLAR_Z0 = 0.60  # collar lower edge
POST_COLLAR_Z1 = 0.72  # collar upper edge
DOME_R = POST_R  # dome cap matches post radius

# ---------------------------------------------------------------------------
# Tripod rotors. Each hub mounts on a side face of the round post (left = -X,
# right = +X) and rotates about an axis that points outward sideways and is
# tilted upward by TILT so the upper arm is roughly horizontal pointing out
# toward that side's railing while the other two splay down at 120 deg phases.
# ---------------------------------------------------------------------------
TILT = math.radians(28.0)  # upward inclination of the hub axis
HUB_X = POST_R  # side face of the round post
HUB_Z = 0.66  # on the collar band so the boss buries into the post
HUB_Y = 0.0
STANDOFF = 0.050  # hub face stands off the post surface

# Outward-inclined hub axes (unit vectors).
_CT = math.cos(TILT)
_ST = math.sin(TILT)
AXIS_RIGHT = (_CT, 0.0, _ST)
AXIS_LEFT = (-_CT, 0.0, _ST)

BOSS_R = 0.038
BOSS_L = 0.10  # spans from inside the post out to the hub collar face
BOSS_INBOARD = BOSS_L / 2.0

CONE = math.radians(45.0)
ARM_R = 0.020
ARM_L = 0.45
ARM_START_Z = 0.034
ARM_PHASES_DEG = (90.0, 210.0, 330.0)
_CB = math.cos(CONE)
_SB = math.sin(CONE)

# ---------------------------------------------------------------------------
# Side railings: free-standing rounded-corner hollow tube frames that run
# parallel to the lane (long axis along Y, the walking direction). One on the
# far left, one on the far right, outboard of each tripod.
# ---------------------------------------------------------------------------
RAIL_X = 0.95
RAIL_LEN = 0.90
RAIL_TUBE_R = 0.015
RAIL_TOP_Z = 1.00
RAIL_CORNER_R = 0.07
RAIL_LEG_BOT_Z = 0.012
RAIL_BOT_Z = 0.13
RAIL_N_BAL = 5


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------


def _post_body_mesh():
    """Round cylindrical post with a base plate and collar ring, built via
    LatheGeometry (revolved profile in the XZ plane about the Z axis).

    Profile points are (radius, z) pairs traced from the floor upward:
    - Wide base plate at z=0
    - Straight cylindrical post body
    - Slight collar ring at the hub mounting height
    - Return to post radius above the collar
    """
    profile = [
        (0.0, 0.0),
        (BASE_PLATE_R, 0.0),
        (BASE_PLATE_R, BASE_PLATE_H * 0.4),
        (BASE_PLATE_R - 0.01, BASE_PLATE_H),
        (POST_R + 0.005, BASE_PLATE_H + 0.01),
        (POST_R, BASE_PLATE_H + 0.03),
        (POST_R, POST_COLLAR_Z0 - 0.01),
        (POST_R + 0.003, POST_COLLAR_Z0),
        (POST_COLLAR_R, POST_COLLAR_Z0 + 0.008),
        (POST_COLLAR_R, POST_COLLAR_Z1 - 0.008),
        (POST_R + 0.003, POST_COLLAR_Z1),
        (POST_R, POST_COLLAR_Z1 + 0.01),
        (POST_R, POST_Z1 - 0.01),
        (POST_R + 0.004, POST_Z1),
        (POST_R + 0.004, POST_Z1 + 0.008),
        (POST_R - 0.005, POST_Z1 + 0.015),
        (0.0, POST_Z1 + 0.015),
    ]
    geom = LatheGeometry(profile, segments=48)
    return mesh_from_geometry(geom, "post_body")


def _dome_cap_mesh():
    """Domed top cap for the round post."""
    dome = DomeGeometry(DOME_R, radial_segments=32, height_segments=10, closed=True)
    return mesh_from_geometry(dome, "dome_cap")


def _hub_mesh():
    """Rotor hub: seating collar plus the front cover disc, along local +Z."""
    collar = cq.Workplane("XY", origin=(0.0, 0.0, -0.004)).circle(0.034).extrude(0.030)
    disc = cq.Workplane("XY", origin=(0.0, 0.0, 0.026)).circle(0.052).extrude(0.032)
    return mesh_from_cadquery(
        collar.union(disc), "hub_core", tolerance=0.0006, angular_tolerance=0.08
    )


def _arm_mesh(name: str):
    """One polished tube arm with a domed end cap, authored along local +Z."""
    tube = cq.Solid.makeCylinder(ARM_R, ARM_L, cq.Vector(0, 0, 0), cq.Vector(0, 0, 1))
    cap = cq.Solid.makeSphere(ARM_R, cq.Vector(0.0, 0.0, ARM_L), angleDegrees1=-90)
    solid = cq.Workplane(obj=tube).union(cq.Workplane(obj=cap))
    return mesh_from_cadquery(solid, name, tolerance=0.0006, angular_tolerance=0.08)


def _railing_frame_mesh(name: str):
    """Rectangular frame with rounded TOP corners, railing-local coordinates.

    Local frame: the railing runs along its local Y axis; x is across the
    railing, z is up. This is the TOP rail plus the two vertical legs: a
    straight horizontal top rail running front-to-back, that turns DOWN through
    a small-radius rounded corner at each end into a strictly VERTICAL leg that
    lands on the floor at y = -RAIL_LEN/2 and y = +RAIL_LEN/2. The legs do NOT
    splay outward. Authored in the local YZ plane (x = 0) and later positioned
    into world (long axis -> world Y).
    """
    y0 = -RAIL_LEN / 2.0
    y1 = RAIL_LEN / 2.0
    z_top = RAIL_TOP_Z
    r = RAIL_CORNER_R
    z_corner = z_top - r
    cy0 = y0 + r
    cy1 = y1 - r

    pts: list[tuple[float, float, float]] = []
    for z in (RAIL_LEG_BOT_Z, 0.25, 0.50, 0.75, z_corner):
        pts.append((0.0, y0, z))
    for deg in range(15, 91, 15):
        a = math.radians(deg)
        pts.append((0.0, cy0 - r * math.cos(a), z_corner + r * math.sin(a)))
    pts.append((0.0, (y0 + y1) / 2.0, z_top))
    for deg in range(15, 91, 15):
        a = math.radians(deg)
        pts.append((0.0, cy1 + r * math.sin(a), z_corner + r * math.cos(a)))
    for z in (z_corner, 0.75, 0.50, 0.25, RAIL_LEG_BOT_Z):
        pts.append((0.0, y1, z))

    frame = tube_from_spline_points(
        pts,
        radius=RAIL_TUBE_R,
        samples_per_segment=10,
        radial_segments=18,
        cap_ends=True,
    )
    return mesh_from_geometry(frame, name)


# ---------------------------------------------------------------------------
# Rotation utilities.
# ---------------------------------------------------------------------------


def _rpy_to_matrix(rpy: tuple[float, float, float]) -> list[list[float]]:
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _matvec(m: list[list[float]], v: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    )


def _axis_rpy(axis: tuple[float, float, float]) -> tuple[float, float, float]:
    """RPY (Rz*Ry*Rx) that maps local +Z onto the given unit axis."""
    ax, ay, az = axis
    pitch = math.acos(max(-1.0, min(1.0, az)))
    yaw = math.atan2(ay, ax)
    return (0.0, pitch, yaw)


# Hub world placement (joint origin translation) on each side face.
HUB_RIGHT_XYZ = (HUB_X + STANDOFF, HUB_Y, HUB_Z)
HUB_LEFT_XYZ = (-(HUB_X + STANDOFF), HUB_Y, HUB_Z)
RPY_RIGHT = _axis_rpy(AXIS_RIGHT)
RPY_LEFT = _axis_rpy(AXIS_LEFT)
_M_RIGHT = _rpy_to_matrix(RPY_RIGHT)
_M_LEFT = _rpy_to_matrix(RPY_LEFT)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="round_post_tripod_turnstile")

    brushed = model.material("brushed_stainless", color=(0.64, 0.65, 0.665, 1.0))
    polished = model.material("polished_stainless", color=(0.79, 0.81, 0.835, 1.0))
    dark_cap = model.material("dark_top_cap", color=(0.17, 0.18, 0.19, 1.0))
    black_panel = model.material("reader_black", color=(0.045, 0.045, 0.05, 1.0))
    white_emblem = model.material("emblem_white", color=(0.93, 0.93, 0.91, 1.0))
    green_led = model.material("led_green", color=(0.15, 0.84, 0.28, 1.0))
    bezel_black = model.material("bezel_black", color=(0.07, 0.07, 0.075, 1.0))

    # ------------------------------------------------------------------ #
    # Pedestal: round post body (lathe), dome cap, readers, LEDs, bosses.
    # ------------------------------------------------------------------ #
    pedestal = model.part("pedestal")
    pedestal.visual(
        _post_body_mesh(),
        origin=Origin(),
        material=brushed,
        name="post_body",
    )
    pedestal.visual(
        _dome_cap_mesh(),
        origin=Origin(xyz=(0.0, 0.0, POST_Z1 + 0.015)),
        material=dark_cap,
        name="dome_cap",
    )

    # Two recessed green LED indicators on the dome top.
    # Embedded into the dome surface so they straddle the dome mesh for contact.
    for i in range(2):
        x = -0.035 + i * 0.07
        # Dome surface at this (x, y) is approximately z=1.040.
        # Place bezel center on the dome surface so it straddles it.
        led_z = 1.040
        pedestal.visual(
            Cylinder(radius=0.020, length=0.016),
            origin=Origin(xyz=(x, -0.015, led_z)),
            material=bezel_black,
            name=f"led_bezel_{i}",
        )
        pedestal.visual(
            Cylinder(radius=0.013, length=0.005),
            origin=Origin(xyz=(x, -0.015, led_z + 0.010)),
            material=green_led,
            name=f"led_green_{i}",
        )

    # Two black RFID reader panels on the front (+Y) face of the round post.
    # Embedded slightly into the cylindrical surface for visual seating contact.
    for i in range(2):
        x = -0.040 + i * 0.080
        pedestal.visual(
            Box((0.065, 0.014, 0.080)),
            origin=Origin(xyz=(x, POST_R - 0.001, 0.785)),
            material=black_panel,
            name=f"reader_panel_{i}",
        )
        pedestal.visual(
            Cylinder(radius=0.022, length=0.006),
            origin=Origin(
                xyz=(x, POST_R + 0.004, 0.785),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=white_emblem,
            name=f"reader_emblem_{i}",
        )

    # Bearing boss on each side face, coaxial with that side's hub axis.
    for i, (side, axis, hub_xyz, rpy) in enumerate((
        ("right", AXIS_RIGHT, HUB_RIGHT_XYZ, RPY_RIGHT),
        ("left", AXIS_LEFT, HUB_LEFT_XYZ, RPY_LEFT),
    )):
        bc = (
            hub_xyz[0] - BOSS_INBOARD * axis[0],
            hub_xyz[1] - BOSS_INBOARD * axis[1],
            hub_xyz[2] - BOSS_INBOARD * axis[2],
        )
        pedestal.visual(
            Cylinder(radius=BOSS_R, length=BOSS_L),
            origin=Origin(xyz=bc, rpy=rpy),
            material=polished,
            name=f"rotor_boss_{side}",
        )

    # ------------------------------------------------------------------ #
    # Two tripod hubs: one on the left face, one on the right face.
    # ------------------------------------------------------------------ #
    for i, (side, hub_xyz, rpy) in enumerate((
        ("right", HUB_RIGHT_XYZ, RPY_RIGHT),
        ("left", HUB_LEFT_XYZ, RPY_LEFT),
    )):
        hub = model.part(f"tripod_hub_{side}")
        hub.visual(_hub_mesh(), origin=Origin(), material=polished, name="hub_core")
        for index, phase in enumerate(ARM_PHASES_DEG):
            hub.visual(
                _arm_mesh(f"arm_{side}_{index}"),
                origin=Origin(
                    xyz=(0.0, 0.0, ARM_START_Z),
                    rpy=(0.0, math.pi / 2.0 - CONE, math.radians(phase)),
                ),
                material=polished,
                name=f"arm_{index}",
            )
        model.articulation(
            f"tripod_hub_{side}",
            ArticulationType.CONTINUOUS,
            parent=pedestal,
            child=hub,
            origin=Origin(xyz=hub_xyz, rpy=rpy),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=40.0, velocity=8.0),
        )

    # ------------------------------------------------------------------ #
    # Two free-standing side railings.
    # ------------------------------------------------------------------ #
    for i, sign in enumerate((1.0, -1.0)):
        railing = model.part(f"side_railing_{i}")
        railing.visual(
            _railing_frame_mesh(f"railing_frame_{i}"),
            origin=Origin(),
            material=polished,
            name="frame",
        )
        railing.visual(
            Cylinder(radius=RAIL_TUBE_R, length=RAIL_LEN + 0.004),
            origin=Origin(xyz=(0.0, 0.0, RAIL_BOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=polished,
            name="bottom_rail",
        )
        bal_z0 = RAIL_BOT_Z - RAIL_TUBE_R
        bal_z1 = RAIL_TOP_Z + RAIL_TUBE_R
        bal_len = bal_z1 - bal_z0
        bal_zc = (bal_z0 + bal_z1) / 2.0
        for b in range(RAIL_N_BAL):
            frac = (b + 1) / (RAIL_N_BAL + 1)
            by = -RAIL_LEN / 2.0 + RAIL_LEN * frac
            railing.visual(
                Cylinder(radius=0.0075, length=bal_len),
                origin=Origin(xyz=(0.0, by, bal_zc)),
                material=polished,
                name=f"baluster_{b}",
            )
        for f, fy in enumerate((-RAIL_LEN / 2.0, RAIL_LEN / 2.0)):
            railing.visual(
                Cylinder(radius=0.046, length=0.016),
                origin=Origin(xyz=(0.0, fy, 0.008)),
                material=polished,
                name=f"floor_flange_{f}",
            )
            railing.visual(
                Cylinder(radius=0.026, length=0.04),
                origin=Origin(xyz=(0.0, fy, 0.028)),
                material=polished,
                name=f"flange_collar_{f}",
            )
        model.articulation(
            f"railing_mount_{i}",
            ArticulationType.FIXED,
            parent=pedestal,
            child=railing,
            origin=Origin(xyz=(sign * RAIL_X, 0.0, 0.0)),
        )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def _arm_local_dir(phase_deg: float, q: float) -> tuple[float, float, float]:
    """Arm centerline direction in hub-local frame for hub pose q (rad)."""
    ph = math.radians(phase_deg) + q
    return (_CB * math.cos(ph), _CB * math.sin(ph), _SB)


def _arm_world_center(
    hub_xyz: tuple[float, float, float],
    m: list[list[float]],
    phase_deg: float,
    q: float,
) -> tuple[float, float, float]:
    """World center of an arm tube+cap for a given side hub and pose."""
    d_local = _arm_local_dir(phase_deg, q)
    d_world = _matvec(m, d_local)
    axis_world = _matvec(m, (0.0, 0.0, 1.0))
    half = (ARM_L + ARM_R) / 2.0
    sx = hub_xyz[0] + ARM_START_Z * axis_world[0]
    sy = hub_xyz[1] + ARM_START_Z * axis_world[1]
    sz = hub_xyz[2] + ARM_START_Z * axis_world[2]
    return (sx + half * d_world[0], sy + half * d_world[1], sz + half * d_world[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pedestal = object_model.get_part("pedestal")
    hub_right = object_model.get_part("tripod_hub_right")
    hub_left = object_model.get_part("tripod_hub_left")
    railing_0 = object_model.get_part("side_railing_0")
    railing_1 = object_model.get_part("side_railing_1")
    spin_right = object_model.get_articulation("tripod_hub_right")
    spin_left = object_model.get_articulation("tripod_hub_left")

    # --- Exactly two tripod hub joints, both continuous about local +Z. ----
    hub_joints = [
        a for a in object_model.articulations if a.name.startswith("tripod_hub_")
    ]
    ctx.check(
        "exactly two tripod hub joints exist (left and right)",
        len(hub_joints) == 2
        and {"tripod_hub_left", "tripod_hub_right"} == {a.name for a in hub_joints},
        details=f"hub_joints={[a.name for a in hub_joints]}",
    )

    for spin, axis, side in (
        (spin_right, AXIS_RIGHT, "right"),
        (spin_left, AXIS_LEFT, "left"),
    ):
        ctx.check(
            f"{side} hub joint is continuous about local +Z",
            spin.articulation_type == ArticulationType.CONTINUOUS
            and spin.axis == (0.0, 0.0, 1.0),
            details=f"type={spin.articulation_type}, axis={spin.axis}",
        )
        m = _rpy_to_matrix(spin.origin.rpy)
        world_axis = _matvec(m, (0.0, 0.0, 1.0))
        ctx.check(
            f"{side} hub axis points outward sideways and tilts upward",
            abs(world_axis[0] - axis[0]) < 1e-6
            and abs(world_axis[1]) < 1e-6
            and world_axis[2] > 0.30,
            details=f"world_axis={world_axis}",
        )

    # --- Round post geometry: the post body must read as circular in cross-
    # section (X-Y span roughly equal), not rectangular.
    post_aabb = ctx.part_element_world_aabb(pedestal, elem="post_body")
    ctx.check(
        "post body is round: X-Y span ratio is near 1.0 (circular cross-section)",
        post_aabb is not None,
        details=f"post_body={post_aabb}",
    )
    if post_aabb is not None:
        (x0, y0, z0), (x1, y1, z1) = post_aabb
        span_x = x1 - x0
        span_y = y1 - y0
        # A round post: X and Y spans should be nearly equal (within 20%).
        # A rectangular cabinet would have span_x / span_y >> 1 or << 1.
        ratio = span_x / span_y if span_y > 0.001 else 999.0
        ctx.check(
            "post cross-section is circular (X/Y span ratio 0.8..1.25)",
            0.80 < ratio < 1.25,
            details=f"span_x={span_x:.4f}, span_y={span_y:.4f}, ratio={ratio:.3f}",
        )
        # Post height reaches near 1.0 m (waist-high).
        ctx.check(
            "post reaches near waist height (0.94..1.02 m)",
            0.94 < z1 < 1.02,
            details=f"post_top_z={z1:.4f}",
        )
        # Post starts at the floor (base plate at z=0).
        ctx.check(
            "post base plate rests on the floor",
            z0 < 0.005,
            details=f"post_bottom_z={z0:.4f}",
        )

    # --- Dome cap sits on top of the post.
    dome_aabb = ctx.part_element_world_aabb(pedestal, elem="dome_cap")
    ctx.check(
        "dome cap sits on top of the post",
        dome_aabb is not None
        and post_aabb is not None
        and dome_aabb[0][2] > post_aabb[1][2] - 0.02
        and dome_aabb[1][2] > post_aabb[1][2],
        details=f"dome={dome_aabb}, post_top={post_aabb[1] if post_aabb else None}",
    )

    # --- Each hub carries exactly three arms at 120 degrees apart. ---------
    def measured_arm_center(hub, index: int):
        aabb = ctx.part_element_world_aabb(hub, elem=f"arm_{index}")
        if aabb is None:
            return None
        (x0, y0, z0), (x1, y1, z1) = aabb
        return ((x0 + x1) / 2.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0)

    for hub, hub_xyz, m, side in (
        (hub_right, HUB_RIGHT_XYZ, _M_RIGHT, "right"),
        (hub_left, HUB_LEFT_XYZ, _M_LEFT, "left"),
    ):
        arms = [v for v in hub.visuals if v.name.startswith("arm_")]
        ctx.check(
            f"{side} hub has exactly three arms",
            len(arms) == 3,
            details=f"arms={[v.name for v in arms]}",
        )
        for i in range(3):
            j = (i + 1) % 3
            di = _arm_local_dir(ARM_PHASES_DEG[i], 0.0)
            dj = _arm_local_dir(ARM_PHASES_DEG[j], 0.0)
            dot = sum(a * b for a, b in zip(di, dj))
            expected = _CB * _CB * (-0.5) + _SB * _SB
            ctx.check(
                f"{side} arms {i},{j} are 120 deg apart on the cone",
                abs(dot - expected) < 1e-6,
                details=f"dot={dot:.4f}, expected={expected:.4f}",
            )
        for index, phase in enumerate(ARM_PHASES_DEG):
            measured = measured_arm_center(hub, index)
            predicted = _arm_world_center(hub_xyz, m, phase, 0.0)
            ok = measured is not None and math.dist(measured, predicted) <= 0.025
            ctx.check(
                f"{side} arm_{index} sits on the predicted tilted cone at rest",
                ok,
                details=f"measured={measured}, predicted={predicted}",
            )

    # --- Rest pose: outward arm roughly horizontal toward rail. --
    cR0 = measured_arm_center(hub_right, 0)
    cL0 = measured_arm_center(hub_left, 0)
    ctx.check(
        "right hub arm_0 reaches outward (+X) over the right lane",
        cR0 is not None and cR0[0] > HUB_X + 0.10,
        details=f"right_arm0={cR0}",
    )
    ctx.check(
        "left hub arm_0 reaches outward (-X) over the left lane",
        cL0 is not None and cL0[0] < -(HUB_X + 0.10),
        details=f"left_arm0={cL0}",
    )

    # --- Sweep clearance: arms never hit the round post or base over a full
    # turn. The hub axes are level in Y and sit on the post sides, so arms
    # swing in +/-X and Z away from the central post.
    post_half_x = POST_R
    base_half_x = BASE_PLATE_R
    for hub_xyz, m, sign, side in (
        (HUB_RIGHT_XYZ, _M_RIGHT, 1.0, "right"),
        (HUB_LEFT_XYZ, _M_LEFT, -1.0, "left"),
    ):
        worst_inboard = None
        worst_floor = None
        for step in range(72):
            q = step * math.radians(5.0)
            for phase in ARM_PHASES_DEG:
                d_world = _matvec(m, _arm_local_dir(phase, q))
                axis_world = _matvec(m, (0.0, 0.0, 1.0))
                sx = hub_xyz[0] + ARM_START_Z * axis_world[0]
                sz = hub_xyz[2] + ARM_START_Z * axis_world[2]
                tip_x = sx + (ARM_L + ARM_R) * d_world[0]
                tip_z = sz + (ARM_L + ARM_R) * d_world[2]
                reach = sign * tip_x
                worst_inboard = reach if worst_inboard is None else min(worst_inboard, reach)
                worst_floor = tip_z if worst_floor is None else min(worst_floor, tip_z)
        ctx.check(
            f"{side} arm sweep stays outboard of the post/base",
            worst_inboard is not None and worst_inboard > base_half_x + 0.02,
            details=(
                f"worst_inboard={worst_inboard:.4f}, "
                f"post_half_x={post_half_x:.3f}, base_half_x={base_half_x:.3f}"
            ),
        )
        ctx.check(
            f"{side} arm tips clear the floor over the full sweep",
            worst_floor is not None and worst_floor > 0.03,
            details=f"worst_floor={worst_floor:.4f}",
        )

    # --- Indexed pose: arms rotate together as one unit (left hub by 60deg).
    with ctx.pose({spin_left: math.pi / 3.0}):
        for index, phase in enumerate(ARM_PHASES_DEG):
            measured = measured_arm_center(hub_left, index)
            predicted = _arm_world_center(HUB_LEFT_XYZ, _M_LEFT, phase, math.pi / 3.0)
            ok = measured is not None and math.dist(measured, predicted) <= 0.025
            ctx.check(
                f"left arm_{index} indexes with the hub at q=60deg",
                ok,
                details=f"measured={measured}, predicted={predicted}",
            )

    # --- Hub seating on its bearing boss (intentional local overlap). ------
    for hub, side in ((hub_right, "right"), (hub_left, "left")):
        hub_core = hub.get_visual("hub_core")
        boss = pedestal.get_visual(f"rotor_boss_{side}")
        ctx.allow_overlap(
            hub,
            pedestal,
            elem_a=hub_core,
            elem_b=boss,
            reason=(
                f"{side} hub collar is intentionally seated onto its bearing "
                "boss face so the rotor reads as mounted, not floating"
            ),
        )
        ctx.expect_contact(hub, pedestal, elem_a=hub_core, elem_b=boss, contact_tol=1e-4)

    # --- Two parallel railings: long axis front-to-back, standing on floor. -
    for railing in (railing_0, railing_1):
        ctx.allow_isolated_part(
            railing,
            reason=(
                "guide railing is a free-standing floor-mounted frame, parallel "
                "to and outboard of the lane; it is not joined to the post by any "
                "tube in the reference, and stands on its own disc floor flanges"
            ),
        )
    for railing, sign in ((railing_0, 1.0), (railing_1, -1.0)):
        aabb = ctx.part_world_aabb(railing)
        ok = aabb is None
        if aabb is not None:
            (x0, y0, z0), (x1, y1, z1) = aabb
            span_x = x1 - x0
            span_y = y1 - y0
            ok = (
                span_y > span_x * 2.0
                and min(sign * x0, sign * x1) > 0.55
                and z0 < 0.02
                and 0.95 < z1 < 1.06
                and span_y > 0.7
            )
        ctx.check(
            f"{railing.name} runs front-to-back and stands on the floor",
            ok,
            details=f"aabb={aabb}",
        )

        frame_aabb = ctx.part_element_world_aabb(railing, elem="frame")
        bottom_aabb = ctx.part_element_world_aabb(railing, elem="bottom_rail")
        ctx.check(
            f"{railing.name} frame (top rail + legs) lands on the floor and "
            "tops out at the top rail height",
            frame_aabb is not None
            and frame_aabb[0][2] < 0.03
            and 0.97 < frame_aabb[1][2] < 1.05,
            details=f"frame={frame_aabb}",
        )
        ctx.check(
            f"{railing.name} has a bottom rail a little above the floor, "
            "parallel to the top rail",
            bottom_aabb is not None
            and 0.05 < (bottom_aabb[0][2] + bottom_aabb[1][2]) / 2.0 < 0.25
            and (bottom_aabb[1][1] - bottom_aabb[0][1]) > 0.7,
            details=f"bottom_rail={bottom_aabb}",
        )

        legs_ok = True
        for f in range(2):
            fl = ctx.part_element_world_aabb(railing, elem=f"floor_flange_{f}")
            if fl is None or fl[0][2] > 0.02:
                legs_ok = False
        ctx.check(
            f"{railing.name} has two floor flanges, both resting on the floor",
            legs_ok,
            details=f"legs_ok={legs_ok}",
        )

        balusters = [v for v in railing.visuals if v.name.startswith("baluster_")]
        tall_balusters = 0
        for v in balusters:
            ba = ctx.part_element_world_aabb(railing, elem=v.name)
            if ba is not None and (ba[1][2] - ba[0][2]) > 0.7 and ba[0][2] < 0.2:
                tall_balusters += 1
        ctx.check(
            f"{railing.name} has >= 4 balusters spanning bottom rail to top rail",
            len(balusters) >= 4 and tall_balusters >= 4,
            details=f"n_balusters={len(balusters)}, tall={tall_balusters}",
        )

    # --- RFID readers sit on the front face of the round post. ------------
    for i in range(2):
        reader = ctx.part_element_world_aabb(pedestal, elem=f"reader_panel_{i}")
        ctx.check(
            f"reader_panel_{i} sits proud on the post front face",
            reader is not None
            and reader[1][1] > POST_R
            and 0.70 < (reader[0][2] + reader[1][2]) / 2.0 < 0.88,
            details=f"reader={reader}",
        )

    # --- Overall sanity. ------------------------------------------------
    ped_aabb = ctx.part_world_aabb(pedestal)
    r0_aabb = ctx.part_world_aabb(railing_0)
    ctx.check(
        "overall dimensions are plausible for a waist-high turnstile",
        ped_aabb is not None
        and r0_aabb is not None
        and 0.96 < ped_aabb[1][2] < 1.10
        and ped_aabb[0][2] > -0.001
        and 0.40 < ARM_L < 0.55
        and r0_aabb[1][0] < 1.30,
        details=f"pedestal={ped_aabb}, railing_0={r0_aabb}, arm_len={ARM_L}",
    )

    return ctx.report()


object_model = build_object_model()
