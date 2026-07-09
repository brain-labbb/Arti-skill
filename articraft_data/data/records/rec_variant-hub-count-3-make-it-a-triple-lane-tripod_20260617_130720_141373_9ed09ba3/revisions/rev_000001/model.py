from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
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
# Y, the walking direction), +Z = up. This is a TRIPLE-LANE tripod turnstile
# bank: three pedestal heads in a row along X, each carrying one tripod hub
# on its +X face, flanked by two free-standing railings on the far left and
# far right.
# ---------------------------------------------------------------------------

N_LANES = 3
LANE_SPACING = 0.95  # center-to-center distance between adjacent units (meters)


def _unit_x(i: int) -> float:
    """X offset for lane unit i, centered about x=0."""
    return (i - (N_LANES - 1) / 2.0) * LANE_SPACING


# Pedestal stack.
PLINTH_FOOT = (0.46, 0.36)  # pyramidal base plate footprint (x, y)
PLINTH_TOP = (0.19, 0.16)
PLINTH_H = 0.115
COLUMN_BOT = (0.175, 0.15)
COLUMN_TOP = (0.14, 0.125)
COLUMN_Z0 = 0.105
COLUMN_Z1 = 0.50

# Wedge head: neck -> wide box -> chamfer -> dark sloped top cap.
HEAD_NECK = (0.138, 0.123)
HEAD_BOX = (0.34, 0.26)
HEAD_CHAMFER_TOP = (0.315, 0.235)
HEAD_Z_NECK = 0.495
HEAD_Z_SHOULDER = 0.64
HEAD_Z_CHAMFER0 = 0.92
HEAD_Z_TOP = 0.96
CAP_Y_HALF = 0.118
CAP_X_HALF = 0.158
CAP_Z0 = 0.958
CAP_Z_FRONT = 0.983
CAP_Z_BACK = 1.000

# ---------------------------------------------------------------------------
# Tripod rotors. Every hub mounts on the +X face of its unit's head and
# rotates about AXIS_RIGHT (outward sideways, tilted upward by TILT).
# ---------------------------------------------------------------------------
TILT = math.radians(28.0)  # upward inclination of the hub axis
HUB_X = HEAD_BOX[0] / 2.0  # side face of the head (full-width shoulder region)
HUB_Z = 0.66  # on the wide shoulder band so the boss buries into the head
HUB_Y = 0.0
STANDOFF = 0.055  # hub face stands off the head side face

# Outward-inclined hub axis (unit vector), uniform for all lanes.
_CT = math.cos(TILT)
_ST = math.sin(TILT)
AXIS_RIGHT = (_CT, 0.0, _ST)

BOSS_R = 0.040
BOSS_L = 0.115  # spans from inside the head out to the hub collar face
BOSS_INBOARD = BOSS_L / 2.0

CONE = math.radians(45.0)
ARM_R = 0.020
ARM_L = 0.45
ARM_START_Z = 0.034  # along the hub axis, outboard of the boss/collar face
ARM_PHASES_DEG = (90.0, 210.0, 330.0)
_CB = math.cos(CONE)
_SB = math.sin(CONE)

# ---------------------------------------------------------------------------
# Side railings: free-standing rounded-corner hollow tube frames that run
# parallel to the lane (long axis along Y, the walking direction). One on the
# far left, one on the far right, outboard of the outermost tripods.
# ---------------------------------------------------------------------------
RAIL_X = abs(_unit_x(N_LANES - 1)) + 0.95  # outboard of the outermost unit
RAIL_LEN = 0.90  # railing length along Y (front-to-back), leg-to-leg span
RAIL_TUBE_R = 0.015
RAIL_TOP_Z = 1.00  # top rail centerline height (waist-high, near the head)
RAIL_CORNER_R = 0.07  # rounded top-corner radius (kept small -> boxy frame)
RAIL_LEG_BOT_Z = 0.012  # leg bottom (just above the floor)
RAIL_BOT_Z = 0.13  # bottom rail height (a little above the floor)
RAIL_N_BAL = 5  # number of vertical balusters between top and bottom rails


def _plinth_mesh():
    solid = (
        cq.Workplane("XY")
        .rect(*PLINTH_FOOT)
        .workplane(offset=PLINTH_H)
        .rect(*PLINTH_TOP)
        .loft(ruled=True)
    )
    return mesh_from_cadquery(solid, "base_plinth", tolerance=0.0008, angular_tolerance=0.1)


def _column_mesh():
    solid = (
        cq.Workplane("XY", origin=(0.0, 0.0, COLUMN_Z0))
        .rect(*COLUMN_BOT)
        .workplane(offset=COLUMN_Z1 - COLUMN_Z0)
        .rect(*COLUMN_TOP)
        .loft(ruled=True)
    )
    return mesh_from_cadquery(solid, "column", tolerance=0.0008, angular_tolerance=0.1)


def _head_mesh():
    """Wedge head: narrow neck flaring to the reader box, chamfered at the top."""
    solid = (
        cq.Workplane("XY", origin=(0.0, 0.0, HEAD_Z_NECK))
        .rect(*HEAD_NECK)
        .workplane(offset=HEAD_Z_SHOULDER - HEAD_Z_NECK)
        .rect(*HEAD_BOX)
        .workplane(offset=HEAD_Z_CHAMFER0 - HEAD_Z_SHOULDER)
        .rect(*HEAD_BOX)
        .workplane(offset=HEAD_Z_TOP - HEAD_Z_CHAMFER0)
        .rect(*HEAD_CHAMFER_TOP)
        .loft(ruled=True)
    )
    return mesh_from_cadquery(solid, "head_shell", tolerance=0.0008, angular_tolerance=0.1)


def _top_cap_mesh():
    """Dark top cap with a gentle slope down toward the front edge."""
    profile = [
        (-CAP_Y_HALF, CAP_Z0),
        (CAP_Y_HALF, CAP_Z0),
        (CAP_Y_HALF, CAP_Z_FRONT),
        (-CAP_Y_HALF, CAP_Z_BACK),
    ]
    solid = cq.Workplane("YZ").polyline(profile).close().extrude(CAP_X_HALF, both=True)
    return mesh_from_cadquery(solid, "top_cap", tolerance=0.0008, angular_tolerance=0.1)


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
    """Rectangular frame with rounded TOP corners, railing-local coordinates."""
    y0 = -RAIL_LEN / 2.0  # rear leg
    y1 = RAIL_LEN / 2.0  # front leg
    z_top = RAIL_TOP_Z
    r = RAIL_CORNER_R
    z_corner = z_top - r  # height where the vertical leg meets the corner arc
    cy0 = y0 + r  # rear corner-arc center (in y)
    cy1 = y1 - r  # front corner-arc center (in y)

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


# Uniform hub placement: all hubs on the +X face of their respective heads,
# sharing the same axis and rpy.
HUB_RPY = _axis_rpy(AXIS_RIGHT)
_M_HUB = _rpy_to_matrix(HUB_RPY)


def _hub_world_xyz(i: int) -> tuple[float, float, float]:
    """World position of the hub joint origin for lane unit i."""
    return (_unit_x(i) + HUB_X + STANDOFF, HUB_Y, HUB_Z)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="triple_tripod_turnstile")

    brushed = model.material("brushed_stainless", color=(0.64, 0.65, 0.665, 1.0))
    polished = model.material("polished_stainless", color=(0.79, 0.81, 0.835, 1.0))
    dark_top = model.material("dark_anodized_top", color=(0.17, 0.18, 0.19, 1.0))
    black_panel = model.material("reader_black", color=(0.045, 0.045, 0.05, 1.0))
    white_emblem = model.material("emblem_white", color=(0.93, 0.93, 0.91, 1.0))
    green_led = model.material("led_green", color=(0.15, 0.84, 0.28, 1.0))
    bezel_black = model.material("bezel_black", color=(0.07, 0.07, 0.075, 1.0))

    # ------------------------------------------------------------------ #
    # Cabinet (root): all static pedestal/head visuals for every lane.
    # ------------------------------------------------------------------ #
    cabinet = model.part("cabinet")

    # Pre-generate shared meshes (same geometry reused across lanes).
    plinth_m = _plinth_mesh()
    column_m = _column_mesh()
    head_m = _head_mesh()
    top_cap_m = _top_cap_mesh()
    hub_m = _hub_mesh()
    arm_ms = [_arm_mesh(f"arm_{j}") for j in range(3)]

    # ------------------------------------------------------------------ #
    # Inter-unit base channels: thin stainless bars at floor level that
    # physically connect adjacent plinths (real turnstile banks bolt units
    # to a shared alignment channel).
    # ------------------------------------------------------------------ #
    channel_h = 0.018  # channel height
    channel_w = 0.060  # channel width
    for i in range(N_LANES - 1):
        xa = _unit_x(i)
        xb = _unit_x(i + 1)
        cx = (xa + xb) / 2.0
        span = xb - xa - PLINTH_FOOT[0] + 0.04  # overlap into each plinth
        cabinet.visual(
            Box((span, channel_w, channel_h)),
            origin=Origin(xyz=(cx, 0.0, channel_h / 2.0)),
            material=brushed,
            name=f"base_channel_{i}",
        )

    # ------------------------------------------------------------------ #
    # Lane units: for each lane i, add pedestal+head visuals to the
    # cabinet root, then create a hub part with 3 arms + continuous joint.
    # ------------------------------------------------------------------ #
    for i in range(N_LANES):
        x = _unit_x(i)

        # Pedestal and head visuals, shifted to this unit's x position.
        cabinet.visual(
            plinth_m, origin=Origin(xyz=(x, 0.0, 0.0)),
            material=brushed, name=f"plinth_{i}",
        )
        cabinet.visual(
            column_m, origin=Origin(xyz=(x, 0.0, 0.0)),
            material=brushed, name=f"column_{i}",
        )
        cabinet.visual(
            head_m, origin=Origin(xyz=(x, 0.0, 0.0)),
            material=brushed, name=f"head_shell_{i}",
        )
        cabinet.visual(
            top_cap_m, origin=Origin(xyz=(x, 0.0, 0.0)),
            material=dark_top, name=f"top_cap_{i}",
        )

        # Recessed green LED indicators on the sloped top.
        for j, lx in enumerate((-0.08, 0.08)):
            cabinet.visual(
                Cylinder(radius=0.024, length=0.012),
                origin=Origin(xyz=(x + lx, -0.02, 0.990)),
                material=bezel_black,
                name=f"led_bezel_{i}_{j}",
            )
            cabinet.visual(
                Cylinder(radius=0.0155, length=0.005),
                origin=Origin(xyz=(x + lx, -0.02, 0.9955)),
                material=green_led,
                name=f"led_green_{i}_{j}",
            )

        # RFID reader panels with white emblems.
        for j, rx in enumerate((-0.077, 0.077)):
            cabinet.visual(
                Box((0.10, 0.012, 0.10)),
                origin=Origin(xyz=(x + rx, 0.132, 0.785)),
                material=black_panel,
                name=f"reader_panel_{i}_{j}",
            )
            cabinet.visual(
                Cylinder(radius=0.030, length=0.003),
                origin=Origin(xyz=(x + rx, 0.139, 0.785), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=white_emblem,
                name=f"reader_emblem_{i}_{j}",
            )

        # Bearing boss on the +X face, coaxial with the hub axis.
        hub_xyz = _hub_world_xyz(i)
        bc = (
            hub_xyz[0] - BOSS_INBOARD * AXIS_RIGHT[0],
            hub_xyz[1] - BOSS_INBOARD * AXIS_RIGHT[1],
            hub_xyz[2] - BOSS_INBOARD * AXIS_RIGHT[2],
        )
        cabinet.visual(
            Cylinder(radius=BOSS_R, length=BOSS_L),
            origin=Origin(xyz=bc, rpy=HUB_RPY),
            material=polished,
            name=f"rotor_boss_{i}",
        )

        # Hub part: hub core + three 120-degree arms.
        hub = model.part(f"tripod_hub_{i}")
        hub.visual(hub_m, origin=Origin(), material=polished, name="hub_core")
        for j, phase in enumerate(ARM_PHASES_DEG):
            hub.visual(
                arm_ms[j],
                origin=Origin(
                    xyz=(0.0, 0.0, ARM_START_Z),
                    rpy=(0.0, math.pi / 2.0 - CONE, math.radians(phase)),
                ),
                material=polished,
                name=f"arm_{j}",
            )

        # Continuous rotation joint — uniform policy for all lanes.
        model.articulation(
            f"tripod_hub_{i}",
            ArticulationType.CONTINUOUS,
            parent=cabinet,
            child=hub,
            origin=Origin(xyz=hub_xyz, rpy=HUB_RPY),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=40.0, velocity=8.0),
        )

    # ------------------------------------------------------------------ #
    # Two free-standing side railings, flanking the lane bank on far left
    # and far right. Each anchored to the floor via a fixed joint.
    # ------------------------------------------------------------------ #
    for index, sign in enumerate((1.0, -1.0)):  # 0 = right, 1 = left
        railing = model.part(f"side_railing_{index}")
        railing.visual(
            _railing_frame_mesh(f"railing_frame_{index}"),
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
            f"railing_mount_{index}",
            ArticulationType.FIXED,
            parent=cabinet,
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
    """World center of an arm tube+cap for a given hub and pose."""
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

    cabinet = object_model.get_part("cabinet")

    # Collect hub parts and joints for all lanes.
    hub_parts = [object_model.get_part(f"tripod_hub_{i}") for i in range(N_LANES)]
    hub_joints = [object_model.get_articulation(f"tripod_hub_{i}") for i in range(N_LANES)]
    hub_xyzs = [_hub_world_xyz(i) for i in range(N_LANES)]

    railing_0 = object_model.get_part("side_railing_0")
    railing_1 = object_model.get_part("side_railing_1")

    # --- Exactly N_LANES tripod hub joints. --------------------------------
    all_hub_joints = [
        a for a in object_model.articulations if a.name.startswith("tripod_hub_")
    ]
    expected_names = {f"tripod_hub_{i}" for i in range(N_LANES)}
    ctx.check(
        f"exactly {N_LANES} tripod hub joints exist",
        len(all_hub_joints) == N_LANES
        and expected_names == {a.name for a in all_hub_joints},
        details=f"hub_joints={[a.name for a in all_hub_joints]}",
    )

    # No horizontal guide tube part remains.
    part_names = {p.name for p in object_model.parts}
    ctx.check(
        "no horizontal guide tube part remains",
        "guide_tube" not in part_names
        and not any("guide_tube" in p for p in part_names),
        details=f"parts={sorted(part_names)}",
    )

    # --- All hub joints are continuous about local +Z with uniform axis. ---
    for i, spin in enumerate(hub_joints):
        ctx.check(
            f"hub_{i} joint is continuous about local +Z",
            spin.articulation_type == ArticulationType.CONTINUOUS
            and spin.axis == (0.0, 0.0, 1.0),
            details=f"type={spin.articulation_type}, axis={spin.axis}",
        )
        m = _rpy_to_matrix(spin.origin.rpy)
        world_axis = _matvec(m, (0.0, 0.0, 1.0))
        ctx.check(
            f"hub_{i} axis points outward +X and tilts upward (uniform policy)",
            abs(world_axis[0] - AXIS_RIGHT[0]) < 1e-6
            and abs(world_axis[1]) < 1e-6
            and world_axis[2] > 0.30,
            details=f"world_axis={world_axis}",
        )

    # --- Each hub carries exactly three arms at 120 degrees. ---------------
    def measured_arm_center(hub, index: int) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(hub, elem=f"arm_{index}")
        if aabb is None:
            return None
        (x0, y0, z0), (x1, y1, z1) = aabb
        return ((x0 + x1) / 2.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0)

    for i, (hub, hub_xyz) in enumerate(zip(hub_parts, hub_xyzs)):
        arms = [v for v in hub.visuals if v.name.startswith("arm_")]
        ctx.check(
            f"hub_{i} has exactly three arms",
            len(arms) == 3,
            details=f"arms={[v.name for v in arms]}",
        )
        # 120-degree spacing on the cone.
        for a in range(3):
            b_idx = (a + 1) % 3
            da = _arm_local_dir(ARM_PHASES_DEG[a], 0.0)
            db = _arm_local_dir(ARM_PHASES_DEG[b_idx], 0.0)
            dot = sum(x * y for x, y in zip(da, db))
            expected = _CB * _CB * (-0.5) + _SB * _SB
            ctx.check(
                f"hub_{i} arms {a},{b_idx} are 120 deg apart on the cone",
                abs(dot - expected) < 1e-6,
                details=f"dot={dot:.4f}, expected={expected:.4f}",
            )
        # Arm positions match the predicted cone at rest.
        for index, phase in enumerate(ARM_PHASES_DEG):
            measured = measured_arm_center(hub, index)
            predicted = _arm_world_center(hub_xyz, _M_HUB, phase, 0.0)
            ok = measured is not None and math.dist(measured, predicted) <= 0.025
            ctx.check(
                f"hub_{i} arm_{index} sits on the predicted tilted cone at rest",
                ok,
                details=f"measured={measured}, predicted={predicted}",
            )

    # --- Rest pose: outward arm reaches +X from each hub. -----------------
    for i, (hub, hub_xyz) in enumerate(zip(hub_parts, hub_xyzs)):
        c0 = measured_arm_center(hub, 0)
        ctx.check(
            f"hub_{i} arm_0 reaches outward (+X) from its hub position",
            c0 is not None and c0[0] > hub_xyz[0] + 0.10,
            details=f"hub_{i}_arm0={c0}, hub_x={hub_xyz[0]:.3f}",
        )

    # --- Sweep clearance: arms stay outboard of own column/base, clear floor.
    base_half_x = PLINTH_FOOT[0] / 2.0
    for i, hub_xyz in enumerate(hub_xyzs):
        x_off = _unit_x(i)
        worst_inboard = None
        worst_floor = None
        for step in range(72):
            q = step * math.radians(5.0)
            for phase in ARM_PHASES_DEG:
                d_world = _matvec(_M_HUB, _arm_local_dir(phase, q))
                axis_world = _matvec(_M_HUB, (0.0, 0.0, 1.0))
                sx = hub_xyz[0] + ARM_START_Z * axis_world[0]
                sz = hub_xyz[2] + ARM_START_Z * axis_world[2]
                tip_x = sx + (ARM_L + ARM_R) * d_world[0]
                tip_z = sz + (ARM_L + ARM_R) * d_world[2]
                reach = tip_x - x_off  # positive = outward of unit center
                worst_inboard = reach if worst_inboard is None else min(worst_inboard, reach)
                worst_floor = tip_z if worst_floor is None else min(worst_floor, tip_z)
        ctx.check(
            f"hub_{i} arm sweep stays outboard of the column/base",
            worst_inboard is not None and worst_inboard > base_half_x + 0.02,
            details=(
                f"worst_inboard={worst_inboard:.4f}, base_half_x={base_half_x:.3f}"
            ),
        )
        ctx.check(
            f"hub_{i} arm tips clear the floor over the full sweep",
            worst_floor is not None and worst_floor > 0.03,
            details=f"worst_floor={worst_floor:.4f}",
        )

    # --- Indexed pose: arms rotate together (hub_1 by 60deg). --------------
    spin_1 = hub_joints[1]
    with ctx.pose({spin_1: math.pi / 3.0}):
        hub_1 = hub_parts[1]
        hub_xyz_1 = hub_xyzs[1]
        for index, phase in enumerate(ARM_PHASES_DEG):
            measured = measured_arm_center(hub_1, index)
            predicted = _arm_world_center(hub_xyz_1, _M_HUB, phase, math.pi / 3.0)
            ok = measured is not None and math.dist(measured, predicted) <= 0.025
            ctx.check(
                f"hub_1 arm_{index} indexes with the hub at q=60deg",
                ok,
                details=f"measured={measured}, predicted={predicted}",
            )

    # --- Hub seating on bearing boss (intentional local overlap). ----------
    for i, hub in enumerate(hub_parts):
        hub_core = hub.get_visual("hub_core")
        boss = cabinet.get_visual(f"rotor_boss_{i}")
        ctx.allow_overlap(
            hub,
            cabinet,
            elem_a=hub_core,
            elem_b=boss,
            reason=(
                f"hub_{i} collar is intentionally seated onto its bearing "
                "boss face so the rotor reads as mounted, not floating"
            ),
        )
        ctx.expect_contact(hub, cabinet, elem_a=hub_core, elem_b=boss, contact_tol=1e-4)

    # --- Two parallel railings: long axis front-to-back, standing on floor. -
    for railing in (railing_0, railing_1):
        ctx.allow_isolated_part(
            railing,
            reason=(
                "guide railing is a free-standing floor-mounted frame, parallel "
                "to and outboard of the lane bank; it stands on its own disc "
                "floor flanges"
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
                span_y > span_x * 2.0  # long axis runs front-to-back (Y)
                and min(sign * x0, sign * x1) > 1.40  # outboard of the outermost tripod
                and z0 < 0.02  # stands on the floor
                and 0.95 < z1 < 1.06  # waist-high top rail
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
            f"{railing.name} frame lands on the floor and tops out at top rail height",
            frame_aabb is not None
            and frame_aabb[0][2] < 0.03
            and 0.97 < frame_aabb[1][2] < 1.05,
            details=f"frame={frame_aabb}",
        )
        ctx.check(
            f"{railing.name} has a bottom rail parallel to the top rail",
            bottom_aabb is not None
            and 0.05 < (bottom_aabb[0][2] + bottom_aabb[1][2]) / 2.0 < 0.25
            and (bottom_aabb[1][1] - bottom_aabb[0][1]) > 0.7,
            details=f"bottom_rail={bottom_aabb}",
        )
        leg_feet_z = []
        legs_ok = True
        for f in range(2):
            fl = ctx.part_element_world_aabb(railing, elem=f"floor_flange_{f}")
            if fl is None or fl[0][2] > 0.02:
                legs_ok = False
            else:
                leg_feet_z.append(fl[0][2])
        ctx.check(
            f"{railing.name} has two floor flanges resting on the floor",
            legs_ok and len(leg_feet_z) == 2,
            details=f"flange_min_z={leg_feet_z}",
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

    # --- Head unit composition: N_LANES heads, plinths, caps, readers. ----
    for i in range(N_LANES):
        head_aabb = ctx.part_element_world_aabb(cabinet, elem=f"head_shell_{i}")
        plinth_aabb = ctx.part_element_world_aabb(cabinet, elem=f"plinth_{i}")
        cap_aabb = ctx.part_element_world_aabb(cabinet, elem=f"top_cap_{i}")
        ctx.check(
            f"unit_{i} wedge head sits above its pedestal, caps near 1.0 m",
            head_aabb is not None
            and plinth_aabb is not None
            and cap_aabb is not None
            and head_aabb[0][2] > plinth_aabb[1][2] + 0.3
            and 0.99 <= cap_aabb[1][2] <= 1.01,
            details=f"head={head_aabb}, cap={cap_aabb}",
        )
        # Readers on the fascia (Y position is the same for all units).
        for j in range(2):
            reader = ctx.part_element_world_aabb(cabinet, elem=f"reader_panel_{i}_{j}")
            ctx.check(
                f"unit_{i} reader_panel_{j} sits proud on the head fascia",
                reader is not None
                and reader[1][1] > HEAD_BOX[1] / 2.0
                and 0.70 < (reader[0][2] + reader[1][2]) / 2.0 < 0.88,
                details=f"reader={reader}",
            )
        # LEDs on the sloped top.
        for j in range(2):
            led = ctx.part_element_world_aabb(cabinet, elem=f"led_green_{i}_{j}")
            ctx.check(
                f"unit_{i} led_green_{j} is set into the sloped top cap",
                led is not None
                and cap_aabb is not None
                and led[1][2] > cap_aabb[0][2]
                and led[0][0] > cap_aabb[0][0]
                and led[1][0] < cap_aabb[1][0],
                details=f"led={led}",
            )

    # --- Lane spacing: units are evenly spaced along X. --------------------
    for i in range(N_LANES - 1):
        plinth_a = ctx.part_element_world_aabb(cabinet, elem=f"plinth_{i}")
        plinth_b = ctx.part_element_world_aabb(cabinet, elem=f"plinth_{i + 1}")
        if plinth_a is not None and plinth_b is not None:
            center_a = (plinth_a[0][0] + plinth_a[1][0]) / 2.0
            center_b = (plinth_b[0][0] + plinth_b[1][0]) / 2.0
            gap = center_b - center_a
            ctx.check(
                f"units {i} and {i+1} are spaced ~{LANE_SPACING:.2f}m apart",
                abs(gap - LANE_SPACING) < 0.05,
                details=f"gap={gap:.4f}",
            )

    # --- Overall sanity. ---------------------------------------------------
    cab_aabb = ctx.part_world_aabb(cabinet)
    r0_aabb = ctx.part_world_aabb(railing_0)
    ctx.check(
        "overall dimensions are plausible for a triple-lane waist-high turnstile bank",
        cab_aabb is not None
        and r0_aabb is not None
        and 0.98 < cab_aabb[1][2] < 1.03
        and cab_aabb[0][2] > -0.001
        and 0.40 < ARM_L < 0.55
        and r0_aabb[1][0] < 2.50  # right railing not absurdly far out
        and (cab_aabb[1][0] - cab_aabb[0][0]) > 2.0 * LANE_SPACING  # spans all 3 units
        ,
        details=f"cabinet={cab_aabb}, railing_0={r0_aabb}, arm_len={ARM_L}",
    )

    return ctx.report()


object_model = build_object_model()
