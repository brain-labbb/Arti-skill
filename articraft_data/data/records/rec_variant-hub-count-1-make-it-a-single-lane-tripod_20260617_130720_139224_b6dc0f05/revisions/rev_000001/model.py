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
# Y, the walking direction), +Z = up. This is a SINGLE-LANE tripod turnstile:
# one pedestal head carrying a tripod hub on its RIGHT face, with one free-
# standing railing running parallel to the lane (front-to-back, along Y) on
# the far right.
# ---------------------------------------------------------------------------
HUB_COUNT = 1  # single-lane variant

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
# Tripod rotors. Each hub mounts on a side face of the head (left = -X,
# right = +X) and rotates about an axis that points outward sideways and is
# tilted upward by TILT so the upper arm is roughly horizontal pointing out
# toward that side's railing while the other two splay down at 120 deg phases.
# ---------------------------------------------------------------------------
TILT = math.radians(28.0)  # upward inclination of the hub axis
HUB_X = HEAD_BOX[0] / 2.0  # side face of the head (full-width shoulder region)
HUB_Z = 0.66  # on the wide shoulder band so the boss buries into the head
HUB_Y = 0.0
STANDOFF = 0.055  # hub face stands off the head side face

# Outward-inclined hub axes (unit vectors).
_CT = math.cos(TILT)
_ST = math.sin(TILT)
AXIS_RIGHT = (_CT, 0.0, _ST)
AXIS_LEFT = (-_CT, 0.0, _ST)

BOSS_R = 0.040
BOSS_L = 0.115  # spans from inside the head out to the hub collar face
# Boss center offset inboard so its outer face lands right at the hub origin
# plane (the hub collar reaches 4 mm inboard, giving seated contact without
# protruding past the face into the splayed arm roots).
BOSS_INBOARD = BOSS_L / 2.0

CONE = math.radians(45.0)
ARM_R = 0.020
ARM_L = 0.45
ARM_START_Z = 0.034  # along the hub axis, outboard of the boss/collar face
# arm_0 is the (roughly horizontal) blocking arm pointing outward; the other
# two splay down. Phase is measured in the hub-local plane.
ARM_PHASES_DEG = (90.0, 210.0, 330.0)
_CB = math.cos(CONE)
_SB = math.sin(CONE)

# ---------------------------------------------------------------------------
# Side railings: free-standing rounded-corner hollow tube frames that run
# parallel to the lane (long axis along Y, the walking direction). One on the
# far left, one on the far right, outboard of each tripod.
# ---------------------------------------------------------------------------
RAIL_X = 0.95  # |x| centerline of each railing (outboard of the tripods)
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
    """Rectangular frame with rounded TOP corners, railing-local coordinates.

    Local frame: the railing runs along its local Y axis; x is across the
    railing, z is up. This is the TOP rail plus the two vertical legs: a
    straight horizontal top rail running front-to-back, that turns DOWN through
    a small-radius rounded corner at each end into a strictly VERTICAL leg that
    lands on the floor at y = -RAIL_LEN/2 and y = +RAIL_LEN/2. The legs do NOT
    splay outward. Authored in the local YZ plane (x = 0) and later positioned
    into world (long axis -> world Y).
    """
    y0 = -RAIL_LEN / 2.0  # rear leg
    y1 = RAIL_LEN / 2.0  # front leg
    z_top = RAIL_TOP_Z
    r = RAIL_CORNER_R
    z_corner = z_top - r  # height where the vertical leg meets the corner arc
    cy0 = y0 + r  # rear corner-arc center (in y)
    cy1 = y1 - r  # front corner-arc center (in y)

    pts: list[tuple[float, float, float]] = []
    # rear leg: strictly vertical at y = y0, from the floor up to the corner
    for z in (RAIL_LEG_BOT_Z, 0.25, 0.50, 0.75, z_corner):
        pts.append((0.0, y0, z))
    # rear rounded top corner: sweep from vertical leg into horizontal top
    for deg in range(15, 91, 15):
        a = math.radians(deg)
        pts.append((0.0, cy0 - r * math.cos(a), z_corner + r * math.sin(a)))
    # horizontal top rail midpoint
    pts.append((0.0, (y0 + y1) / 2.0, z_top))
    # front rounded top corner: horizontal top back down into the vertical leg
    for deg in range(15, 91, 15):
        a = math.radians(deg)
        pts.append((0.0, cy1 + r * math.sin(a), z_corner + r * math.cos(a)))
    # front leg: strictly vertical at y = y1, down to the floor
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
# Rotation utilities. We map the hub-local +Z onto each side's world axis via
# an explicit rotation, then build it into the joint origin rpy. To keep the
# tests exactly consistent with the URDF, the same rotation matrix is used to
# predict arm directions in world space.
# ---------------------------------------------------------------------------


def _rpy_to_matrix(rpy: tuple[float, float, float]) -> list[list[float]]:
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    # URDF convention: R = Rz(y) * Ry(p) * Rx(r)
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
    """RPY (Rz*Ry*Rx) that maps local +Z onto the given unit axis.

    We use roll=0 and choose pitch/yaw so Rz(yaw)*Ry(pitch)*[0,0,1] = axis.
    Ry(pitch)*z = (sin p, 0, cos p); then Rz(yaw) rotates in the xy plane.
    So axis = (cos yaw sin p, sin yaw sin p, cos p).
    """
    ax, ay, az = axis
    pitch = math.acos(max(-1.0, min(1.0, az)))
    yaw = math.atan2(ay, ax)
    return (0.0, pitch, yaw)


# Hub configurations: single-lane uses only the right face.
_HUB_CONFIGS = [
    {
        "xyz": (HUB_X + STANDOFF, HUB_Y, HUB_Z),
        "axis": AXIS_RIGHT,
        "rpy": _axis_rpy(AXIS_RIGHT),
    },
]
HUB_RIGHT_XYZ = _HUB_CONFIGS[0]["xyz"]
RPY_RIGHT = _HUB_CONFIGS[0]["rpy"]
_M_RIGHT = _rpy_to_matrix(RPY_RIGHT)


def _build_tripod_hub(model, cabinet, index: int, hub_xyz, rpy, polished):
    """Build one tripod hub part with three 120-degree arms and its continuous joint.

    Shared helper used by the hub loop so each hub is constructed identically.
    Returns the hub part.
    """
    hub = model.part(f"tripod_hub_{index}")
    hub.visual(_hub_mesh(), origin=Origin(), material=polished, name="hub_core")
    for arm_idx, phase in enumerate(ARM_PHASES_DEG):
        hub.visual(
            _arm_mesh(f"arm_{index}_{arm_idx}"),
            origin=Origin(
                xyz=(0.0, 0.0, ARM_START_Z),
                rpy=(0.0, math.pi / 2.0 - CONE, math.radians(phase)),
            ),
            material=polished,
            name=f"arm_{arm_idx}",
        )
    model.articulation(
        f"tripod_hub_{index}",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=hub,
        origin=Origin(xyz=hub_xyz, rpy=rpy),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=8.0),
    )
    return hub


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_tripod_turnstile")

    brushed = model.material("brushed_stainless", color=(0.64, 0.65, 0.665, 1.0))
    polished = model.material("polished_stainless", color=(0.79, 0.81, 0.835, 1.0))
    dark_top = model.material("dark_anodized_top", color=(0.17, 0.18, 0.19, 1.0))
    black_panel = model.material("reader_black", color=(0.045, 0.045, 0.05, 1.0))
    white_emblem = model.material("emblem_white", color=(0.93, 0.93, 0.91, 1.0))
    green_led = model.material("led_green", color=(0.15, 0.84, 0.28, 1.0))
    bezel_black = model.material("bezel_black", color=(0.07, 0.07, 0.075, 1.0))

    # ------------------------------------------------------------------ #
    # Cabinet: plinth, tapered column, wedge head, readers, LEDs, bosses.
    # ------------------------------------------------------------------ #
    cabinet = model.part("cabinet")
    cabinet.visual(_plinth_mesh(), origin=Origin(), material=brushed, name="base_plinth")
    cabinet.visual(_column_mesh(), origin=Origin(), material=brushed, name="column")
    cabinet.visual(_head_mesh(), origin=Origin(), material=brushed, name="head_shell")
    cabinet.visual(_top_cap_mesh(), origin=Origin(), material=dark_top, name="top_cap")

    # Two recessed green LED indicators on the sloped top.
    for index, x in enumerate((-0.08, 0.08)):
        cabinet.visual(
            Cylinder(radius=0.024, length=0.012),
            origin=Origin(xyz=(x, -0.02, 0.990)),
            material=bezel_black,
            name=f"led_bezel_{index}",
        )
        cabinet.visual(
            Cylinder(radius=0.0155, length=0.005),
            origin=Origin(xyz=(x, -0.02, 0.9955)),
            material=green_led,
            name=f"led_green_{index}",
        )

    # Two black square RFID reader panels (one per lane) with white emblems.
    for index, x in enumerate((-0.077, 0.077)):
        cabinet.visual(
            Box((0.10, 0.012, 0.10)),
            origin=Origin(xyz=(x, 0.132, 0.785)),
            material=black_panel,
            name=f"reader_panel_{index}",
        )
        cabinet.visual(
            Cylinder(radius=0.030, length=0.003),
            origin=Origin(xyz=(x, 0.139, 0.785), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=white_emblem,
            name=f"reader_emblem_{index}",
        )

    # Bearing boss on the active hub face, coaxial with the hub axis.
    for i in range(HUB_COUNT):
        cfg = _HUB_CONFIGS[i]
        axis = cfg["axis"]
        hub_xyz = cfg["xyz"]
        rpy = cfg["rpy"]
        bc = (
            hub_xyz[0] - BOSS_INBOARD * axis[0],
            hub_xyz[1] - BOSS_INBOARD * axis[1],
            hub_xyz[2] - BOSS_INBOARD * axis[2],
        )
        cabinet.visual(
            Cylinder(radius=BOSS_R, length=BOSS_L),
            origin=Origin(xyz=bc, rpy=rpy),
            material=polished,
            name=f"rotor_boss_{i}",
        )

    # ------------------------------------------------------------------ #
    # Tripod hub(s): each is its own part with three 120-degree cone arms,
    # rotating about its outward-inclined axis (continuous). Single-lane
    # variant uses HUB_COUNT=1.
    # ------------------------------------------------------------------ #
    hubs = []
    for i in range(HUB_COUNT):
        cfg = _HUB_CONFIGS[i]
        hub = _build_tripod_hub(
            model, cabinet, i, cfg["xyz"], cfg["rpy"], polished
        )
        hubs.append(hub)

    # ------------------------------------------------------------------ #
    # Free-standing side railing(s), parallel to the lane (long axis along
    # world Y). Single-lane variant uses one railing on the active hub side.
    # Each is rigidly anchored to the floor via a fixed joint to the cabinet
    # root (free-standing structure).
    # ------------------------------------------------------------------ #
    _RAILING_SIGNS = (1.0,)  # single-lane: right side only
    for index, sign in enumerate(_RAILING_SIGNS):
        railing = model.part(f"side_railing_{index}")
        # The frame mesh supplies the TOP rail plus the two vertical legs.
        railing.visual(
            _railing_frame_mesh(f"railing_frame_{index}"),
            origin=Origin(),
            material=polished,
            name="frame",
        )
        # BOTTOM rail: a horizontal tube parallel to the top, a little above the
        # floor, running front-to-back between the two legs.
        railing.visual(
            Cylinder(radius=RAIL_TUBE_R, length=RAIL_LEN + 0.004),
            origin=Origin(xyz=(0.0, 0.0, RAIL_BOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=polished,
            name="bottom_rail",
        )
        # Vertical balusters evenly spaced along Y, each spanning from the
        # bottom rail up to the top rail and overlapping both so nothing floats.
        bal_z0 = RAIL_BOT_Z - RAIL_TUBE_R  # dip below the bottom rail center
        bal_z1 = RAIL_TOP_Z + RAIL_TUBE_R  # poke above the top rail center
        bal_len = bal_z1 - bal_z0
        bal_zc = (bal_z0 + bal_z1) / 2.0
        for b in range(RAIL_N_BAL):
            # spread balusters within the clear leg-to-leg span (inset off legs)
            frac = (b + 1) / (RAIL_N_BAL + 1)
            by = -RAIL_LEN / 2.0 + RAIL_LEN * frac
            railing.visual(
                Cylinder(radius=0.0075, length=bal_len),
                origin=Origin(xyz=(0.0, by, bal_zc)),
                material=polished,
                name=f"baluster_{b}",
            )
        # Disc floor flange + short collar at the foot of each of the two legs.
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

    cabinet = object_model.get_part("cabinet")
    hub_0 = object_model.get_part("tripod_hub_0")
    railing_0 = object_model.get_part("side_railing_0")
    spin_0 = object_model.get_articulation("tripod_hub_0")

    # --- Exactly one tripod hub joint (single-lane), continuous about +Z. --
    hub_joints = [
        a
        for a in object_model.articulations
        if a.name.startswith("tripod_hub_")
    ]
    ctx.check(
        "exactly one tripod hub joint exists (single-lane)",
        len(hub_joints) == 1
        and hub_joints[0].name == "tripod_hub_0",
        details=f"hub_joints={[a.name for a in hub_joints]}",
    )

    # No second hub or legacy side-named hub exists.
    part_names = {p.name for p in object_model.parts}
    ctx.check(
        "no second hub exists (single-lane only)",
        "tripod_hub_1" not in part_names
        and "tripod_hub_left" not in part_names
        and "tripod_hub_right" not in part_names,
        details=f"parts={sorted(part_names)}",
    )

    # No horizontal guide tube link or part remains.
    ctx.check(
        "no horizontal guide tube part remains",
        "guide_tube" not in part_names
        and not any("guide_tube" in p for p in part_names),
        details=f"parts={sorted(part_names)}",
    )

    # --- Hub joint is continuous about local +Z. ---------------------------
    cfg = _HUB_CONFIGS[0]
    axis = cfg["axis"]
    ctx.check(
        "hub_0 joint is continuous about local +Z",
        spin_0.articulation_type == ArticulationType.CONTINUOUS
        and spin_0.axis == (0.0, 0.0, 1.0),
        details=f"type={spin_0.articulation_type}, axis={spin_0.axis}",
    )
    m = _rpy_to_matrix(spin_0.origin.rpy)
    world_axis = _matvec(m, (0.0, 0.0, 1.0))
    ctx.check(
        "hub_0 axis points outward sideways (+X) and tilts upward",
        abs(world_axis[0] - axis[0]) < 1e-6
        and abs(world_axis[1]) < 1e-6
        and world_axis[2] > 0.30,
        details=f"world_axis={world_axis}",
    )

    # --- Hub carries exactly three arms at 120 degrees apart. --------------
    def measured_arm_center(hub, index: int) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(hub, elem=f"arm_{index}")
        if aabb is None:
            return None
        (x0, y0, z0), (x1, y1, z1) = aabb
        return ((x0 + x1) / 2.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0)

    arms = [v for v in hub_0.visuals if v.name.startswith("arm_")]
    ctx.check(
        "hub_0 has exactly three arms",
        len(arms) == 3,
        details=f"arms={[v.name for v in arms]}",
    )
    # Arm directions are 120 deg apart on the local cone (by construction).
    for i in range(3):
        j = (i + 1) % 3
        di = _arm_local_dir(ARM_PHASES_DEG[i], 0.0)
        dj = _arm_local_dir(ARM_PHASES_DEG[j], 0.0)
        dot = sum(a * b for a, b in zip(di, dj))
        expected = _CB * _CB * (-0.5) + _SB * _SB
        ctx.check(
            f"arms {i},{j} are 120 deg apart on the cone",
            abs(dot - expected) < 1e-6,
            details=f"dot={dot:.4f}, expected={expected:.4f}",
        )
    # Measured arm centers match the predicted world cone at rest.
    hub_xyz = cfg["xyz"]
    for index, phase in enumerate(ARM_PHASES_DEG):
        measured = measured_arm_center(hub_0, index)
        predicted = _arm_world_center(hub_xyz, m, phase, 0.0)
        ok = measured is not None and math.dist(measured, predicted) <= 0.025
        ctx.check(
            f"arm_{index} sits on the predicted tilted cone at rest",
            ok,
            details=f"measured={measured}, predicted={predicted}",
        )

    # --- Rest pose: outward (blocking) arm roughly horizontal toward rail. --
    c0 = measured_arm_center(hub_0, 0)
    ctx.check(
        "hub_0 arm_0 reaches outward (+X) over the lane",
        c0 is not None and c0[0] > HUB_X + 0.10,
        details=f"arm0={c0}",
    )

    # --- Sweep clearance: arms never hit the column or base over a full turn.
    col_half_x = COLUMN_BOT[0] / 2.0
    base_half_x = PLINTH_FOOT[0] / 2.0
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
            reach = tip_x  # positive = outward of center (+X side)
            worst_inboard = reach if worst_inboard is None else min(worst_inboard, reach)
            worst_floor = tip_z if worst_floor is None else min(worst_floor, tip_z)
    ctx.check(
        "arm sweep stays outboard of the column/base",
        worst_inboard is not None and worst_inboard > base_half_x + 0.02,
        details=(
            f"worst_inboard={worst_inboard:.4f}, "
            f"col_half_x={col_half_x:.3f}, base_half_x={base_half_x:.3f}"
        ),
    )
    ctx.check(
        "arm tips clear the floor over the full sweep",
        worst_floor is not None and worst_floor > 0.03,
        details=f"worst_floor={worst_floor:.4f}",
    )

    # --- Indexed pose: arms rotate together as one unit (hub by 60deg). ----
    with ctx.pose({spin_0: math.pi / 3.0}):
        for index, phase in enumerate(ARM_PHASES_DEG):
            measured = measured_arm_center(hub_0, index)
            predicted = _arm_world_center(hub_xyz, m, phase, math.pi / 3.0)
            ok = measured is not None and math.dist(measured, predicted) <= 0.025
            ctx.check(
                f"arm_{index} indexes with the hub at q=60deg",
                ok,
                details=f"measured={measured}, predicted={predicted}",
            )

    # --- Hub seating on its bearing boss (intentional local overlap). ------
    hub_core = hub_0.get_visual("hub_core")
    boss = cabinet.get_visual("rotor_boss_0")
    ctx.allow_overlap(
        hub_0,
        cabinet,
        elem_a=hub_core,
        elem_b=boss,
        reason=(
            "hub collar is intentionally seated onto its bearing boss face "
            "so the rotor reads as mounted, not floating"
        ),
    )
    ctx.expect_contact(hub_0, cabinet, elem_a=hub_core, elem_b=boss, contact_tol=1e-4)

    # --- Railing: long axis front-to-back, standing on floor. ---------------
    ctx.allow_isolated_part(
        railing_0,
        reason=(
            "guide railing is a free-standing floor-mounted frame, parallel "
            "to and outboard of the lane; it is not joined to the head by any "
            "tube in the reference, and stands on its own disc floor flanges"
        ),
    )
    aabb = ctx.part_world_aabb(railing_0)
    ok = aabb is None
    if aabb is not None:
        (x0, y0, z0), (x1, y1, z1) = aabb
        span_x = x1 - x0
        span_y = y1 - y0
        ok = (
            span_y > span_x * 2.0
            and min(x0, x1) > 0.55
            and z0 < 0.02
            and 0.95 < z1 < 1.06
            and span_y > 0.7
        )
    ctx.check(
        "side_railing_0 runs front-to-back and stands on the floor",
        ok,
        details=f"aabb={aabb}",
    )

    frame_aabb = ctx.part_element_world_aabb(railing_0, elem="frame")
    bottom_aabb = ctx.part_element_world_aabb(railing_0, elem="bottom_rail")
    ctx.check(
        "side_railing_0 frame lands on the floor and tops out near 1.0 m",
        frame_aabb is not None
        and frame_aabb[0][2] < 0.03
        and 0.97 < frame_aabb[1][2] < 1.05,
        details=f"frame={frame_aabb}",
    )
    ctx.check(
        "side_railing_0 has a bottom rail parallel to the top rail",
        bottom_aabb is not None
        and 0.05 < (bottom_aabb[0][2] + bottom_aabb[1][2]) / 2.0 < 0.25
        and (bottom_aabb[1][1] - bottom_aabb[0][1]) > 0.7,
        details=f"bottom_rail={bottom_aabb}",
    )

    leg_feet_z = []
    legs_ok = True
    for f in range(2):
        fl = ctx.part_element_world_aabb(railing_0, elem=f"floor_flange_{f}")
        if fl is None or fl[0][2] > 0.02:
            legs_ok = False
        else:
            leg_feet_z.append(fl[0][2])
    ctx.check(
        "side_railing_0 has two floor flanges resting on the floor",
        legs_ok and len(leg_feet_z) == 2,
        details=f"flange_min_z={leg_feet_z}",
    )

    leg_ys = []
    if frame_aabb is not None:
        leg_ys = [frame_aabb[0][1], frame_aabb[1][1]]
    ctx.check(
        "side_railing_0 legs span the full front-to-back length",
        len(leg_ys) == 2 and (leg_ys[1] - leg_ys[0]) > 0.8,
        details=f"leg_y_span={leg_ys}",
    )

    balusters = [v for v in railing_0.visuals if v.name.startswith("baluster_")]
    tall_balusters = 0
    for v in balusters:
        ba = ctx.part_element_world_aabb(railing_0, elem=v.name)
        if ba is not None and (ba[1][2] - ba[0][2]) > 0.7 and ba[0][2] < 0.2:
            tall_balusters += 1
    ctx.check(
        "side_railing_0 has >= 4 balusters spanning bottom rail to top rail",
        len(balusters) >= 4 and tall_balusters >= 4,
        details=f"n_balusters={len(balusters)}, tall={tall_balusters}",
    )

    # --- Head unit composition. -------------------------------------------
    head_aabb = ctx.part_element_world_aabb(cabinet, elem="head_shell")
    plinth_aabb = ctx.part_element_world_aabb(cabinet, elem="base_plinth")
    cap_aabb = ctx.part_element_world_aabb(cabinet, elem="top_cap")
    ctx.check(
        "wedge head sits above the pedestal and caps out near 1.0 m",
        head_aabb is not None
        and plinth_aabb is not None
        and cap_aabb is not None
        and head_aabb[0][2] > plinth_aabb[1][2] + 0.3
        and 0.99 <= cap_aabb[1][2] <= 1.01,
        details=f"head={head_aabb}, cap={cap_aabb}",
    )
    for index in range(2):
        reader = ctx.part_element_world_aabb(cabinet, elem=f"reader_panel_{index}")
        ctx.check(
            f"reader_panel_{index} sits proud on the head fascia",
            reader is not None
            and reader[1][1] > HEAD_BOX[1] / 2.0
            and 0.70 < (reader[0][2] + reader[1][2]) / 2.0 < 0.88,
            details=f"reader={reader}",
        )
        led = ctx.part_element_world_aabb(cabinet, elem=f"led_green_{index}")
        ctx.check(
            f"led_green_{index} is set into the sloped top cap",
            led is not None
            and cap_aabb is not None
            and led[1][2] > cap_aabb[0][2]
            and led[0][0] > cap_aabb[0][0]
            and led[1][0] < cap_aabb[1][0],
            details=f"led={led}",
        )

    # --- Overall sanity. ----------------------------------------------------
    cab_aabb = ctx.part_world_aabb(cabinet)
    r0_aabb = ctx.part_world_aabb(railing_0)
    ctx.check(
        "overall dimensions are plausible for a waist-high turnstile",
        cab_aabb is not None
        and r0_aabb is not None
        and 0.98 < cab_aabb[1][2] < 1.03
        and cab_aabb[0][2] > -0.001
        and 0.40 < ARM_L < 0.55
        and r0_aabb[1][0] < 1.30,
        details=f"cabinet={cab_aabb}, railing_0={r0_aabb}, arm_len={ARM_L}",
    )

    return ctx.report()


object_model = build_object_model()
