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
)

# ---------------------------------------------------------------------------
# World frame: +X = right, -X = left, +Y = toward the walker (lane runs along
# Y, the walking direction), +Z = up. This is a DUAL-LANE twin tripod
# turnstile: one central head carrying a tripod hub on its LEFT face and one
# on its RIGHT face, flanked by two free-standing railings that run parallel
# to the lane (front-to-back, along Y) on the far left and far right.
# ---------------------------------------------------------------------------

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
# Glass swing-barrier panels: two framed glass leaves share one centered
# vertical post directly in front of the head. At rest (q=0) the leaves extend
# left/right from the post across the front of the turnstile. Each leaf opens
# 90 degrees toward +Y, so both panels fold forward instead of back.
# ---------------------------------------------------------------------------
PANEL_WIDTH = 0.55  # outward reach from hinge-side to free edge
PANEL_HEIGHT = 0.85  # glass pane height
PANEL_THICKNESS = 0.010  # glass thickness
PANEL_BOT_Z = 0.10  # bottom of glass above floor
PANEL_TOP_Z = PANEL_BOT_Z + PANEL_HEIGHT  # top of glass
FRAME_BAR = 0.022  # frame bar cross-section (square profile)
POST_R = 0.025  # hinge post radius
POST_H = 1.02  # hinge post height
CLIP_LEN = 0.014  # shallow clamp depth so paired leaves meet without clipping
HINGE_X = 0.0  # centered hinge post directly in front of the turnstile
HINGE_Y = 0.50  # hinge post centerline, directly in front of the head
PANEL_CLEARANCE = 0.0  # keep each glass leaf visually seated on the hinge post
GLASS_EMBED = 0.003  # glass embeds into frame grooves for seated capture


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


# Hub world placement (joint origin translation) on each side face.
HUB_RIGHT_XYZ = (HUB_X + STANDOFF, HUB_Y, HUB_Z)
HUB_LEFT_XYZ = (-(HUB_X + STANDOFF), HUB_Y, HUB_Z)
RPY_RIGHT = _axis_rpy(AXIS_RIGHT)
RPY_LEFT = _axis_rpy(AXIS_LEFT)
_M_RIGHT = _rpy_to_matrix(RPY_RIGHT)
_M_LEFT = _rpy_to_matrix(RPY_LEFT)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="twin_tripod_turnstile")

    brushed = model.material("brushed_stainless", color=(0.64, 0.65, 0.665, 1.0))
    polished = model.material("polished_stainless", color=(0.79, 0.81, 0.835, 1.0))
    dark_top = model.material("dark_anodized_top", color=(0.17, 0.18, 0.19, 1.0))
    black_panel = model.material("reader_black", color=(0.045, 0.045, 0.05, 1.0))
    white_emblem = model.material("emblem_white", color=(0.93, 0.93, 0.91, 1.0))
    green_led = model.material("led_green", color=(0.15, 0.84, 0.28, 1.0))
    bezel_black = model.material("bezel_black", color=(0.07, 0.07, 0.075, 1.0))
    tinted_glass = model.material("tinted_glass", color=(0.78, 0.86, 0.90, 0.35))

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

    # Bearing boss on each side face, coaxial with that side's hub axis.
    for side, axis, hub_xyz, rpy in (
        ("right", AXIS_RIGHT, HUB_RIGHT_XYZ, RPY_RIGHT),
        ("left", AXIS_LEFT, HUB_LEFT_XYZ, RPY_LEFT),
    ):
        # boss centered just inboard of the hub face, along the axis, long
        # enough to bury its inner end in the head and reach the hub collar.
        bc = (
            hub_xyz[0] - BOSS_INBOARD * axis[0],
            hub_xyz[1] - BOSS_INBOARD * axis[1],
            hub_xyz[2] - BOSS_INBOARD * axis[2],
        )
        cabinet.visual(
            Cylinder(radius=BOSS_R, length=BOSS_L),
            origin=Origin(xyz=bc, rpy=rpy),
            material=polished,
            name=f"rotor_boss_{side}",
        )

    # ------------------------------------------------------------------ #
    # Two tripod hubs: one on the left face, one on the right face. Each is
    # its own part with three 120-degree cone arms, rotating about its own
    # outward-inclined axis (continuous).
    # ------------------------------------------------------------------ #
    for side, hub_xyz, rpy in (
        ("right", HUB_RIGHT_XYZ, RPY_RIGHT),
        ("left", HUB_LEFT_XYZ, RPY_LEFT),
    ):
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
            parent=cabinet,
            child=hub,
            origin=Origin(xyz=hub_xyz, rpy=rpy),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=40.0, velocity=8.0),
        )

    # ------------------------------------------------------------------ #
    # Hinge post fixed directly in front of the turnstile head.
    # ------------------------------------------------------------------ #
    # Vertical hinge post (fixed, floor to above head height).
    cabinet.visual(
        Cylinder(radius=POST_R, length=POST_H),
        origin=Origin(xyz=(HINGE_X, HINGE_Y, POST_H / 2.0)),
        material=polished,
        name="hinge_post",
    )
    # Post top cap.
    cabinet.visual(
        Cylinder(radius=POST_R + 0.004, length=0.012),
        origin=Origin(xyz=(HINGE_X, HINGE_Y, POST_H - 0.006)),
        material=polished,
        name="post_cap",
    )
    # Floor flange at the post base.
    cabinet.visual(
        Cylinder(radius=0.050, length=0.016),
        origin=Origin(xyz=(HINGE_X, HINGE_Y, 0.008)),
        material=polished,
        name="post_flange",
    )

    # ------------------------------------------------------------------ #
    # Two glass swing-barrier leaves. Their child origins sit on the shared
    # centered hinge line. At q=0 the right leaf extends in local +X and the
    # left leaf extends in local -X. The right leaf opens with +90 degrees;
    # the left leaf opens with -90 degrees. Both fold forward into +Y.
    # ------------------------------------------------------------------ #
    glass_w = PANEL_WIDTH + 2.0 * GLASS_EMBED
    glass_h = PANEL_HEIGHT + 2.0 * GLASS_EMBED
    glass_cz = PANEL_BOT_Z + PANEL_HEIGHT / 2.0

    def _add_glass_leaf(side: str, sign: float, lower: float, upper: float):
        glass_panel = model.part(f"glass_panel_{side}")
        glass_cx = sign * (POST_R + PANEL_CLEARANCE + PANEL_WIDTH / 2.0)

        glass_panel.visual(
            Box((glass_w, PANEL_THICKNESS, glass_h)),
            origin=Origin(xyz=(glass_cx, 0.0, glass_cz)),
            material=tinted_glass,
            name="glass_pane",
        )
        glass_panel.visual(
            Box((PANEL_WIDTH, FRAME_BAR, FRAME_BAR)),
            origin=Origin(xyz=(glass_cx, 0.0, PANEL_TOP_Z + FRAME_BAR / 2.0)),
            material=polished,
            name="frame_top",
        )
        glass_panel.visual(
            Box((PANEL_WIDTH, FRAME_BAR, FRAME_BAR)),
            origin=Origin(xyz=(glass_cx, 0.0, PANEL_BOT_Z - FRAME_BAR / 2.0)),
            material=polished,
            name="frame_bottom",
        )
        glass_panel.visual(
            Box((FRAME_BAR, FRAME_BAR, PANEL_HEIGHT + 2.0 * FRAME_BAR)),
            origin=Origin(
                xyz=(
                    sign
                    * (POST_R + PANEL_CLEARANCE + PANEL_WIDTH + FRAME_BAR / 2.0),
                    0.0,
                    glass_cz,
                )
            ),
            material=polished,
            name="frame_free_edge",
        )
        for i, clip_z in enumerate(
            (PANEL_BOT_Z + 0.08, PANEL_BOT_Z + PANEL_HEIGHT / 2.0, PANEL_TOP_Z - 0.08)
        ):
            glass_panel.visual(
                Cylinder(radius=POST_R + 0.001, length=CLIP_LEN),
                origin=Origin(xyz=(0.0, 0.0, clip_z)),
                material=polished,
                name=f"hinge_clip_{i}",
            )

        model.articulation(
            f"glass_panel_{side}_swing",
            ArticulationType.REVOLUTE,
            parent=cabinet,
            child=glass_panel,
            origin=Origin(xyz=(HINGE_X, HINGE_Y, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=20.0, velocity=2.0, lower=lower, upper=upper
            ),
        )
        return glass_panel

    _add_glass_leaf("left", -1.0, -math.pi / 2.0, 0.0)
    _add_glass_leaf("right", 1.0, 0.0, math.pi / 2.0)

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
    hub_right = object_model.get_part("tripod_hub_right")
    hub_left = object_model.get_part("tripod_hub_left")
    glass_panel_left = object_model.get_part("glass_panel_left")
    glass_panel_right = object_model.get_part("glass_panel_right")
    spin_right = object_model.get_articulation("tripod_hub_right")
    spin_left = object_model.get_articulation("tripod_hub_left")
    swing_left = object_model.get_articulation("glass_panel_left_swing")
    swing_right = object_model.get_articulation("glass_panel_right_swing")

    # --- Exactly two tripod hub joints, both continuous about local +Z. ----
    hub_joints = [
        a
        for a in object_model.articulations
        if a.name.startswith("tripod_hub_")
    ]
    ctx.check(
        "exactly two tripod hub joints exist (left and right)",
        len(hub_joints) == 2
        and {"tripod_hub_left", "tripod_hub_right"} == {a.name for a in hub_joints},
        details=f"hub_joints={[a.name for a in hub_joints]}",
    )

    # No horizontal guide tube or side railing parts remain (variant fork).
    part_names = {p.name for p in object_model.parts}
    ctx.check(
        "no guide tube or side railing parts remain (replaced by glass panel)",
        "guide_tube" not in part_names
        and not any("guide_tube" in p for p in part_names)
        and not any("side_railing" in p for p in part_names),
        details=f"parts={sorted(part_names)}",
    )
    ctx.check(
        "left and right glass_panel parts exist",
        {"glass_panel_left", "glass_panel_right"}.issubset(part_names),
        details=f"parts={sorted(part_names)}",
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
        # The joint origin rpy must rotate local +Z onto the outward-inclined
        # world axis: outward in X (sign matches side), level in Y, tilted up.
        m = _rpy_to_matrix(spin.origin.rpy)
        world_axis = _matvec(m, (0.0, 0.0, 1.0))
        ctx.check(
            f"{side} hub axis points outward sideways and tilts upward",
            abs(world_axis[0] - axis[0]) < 1e-6
            and abs(world_axis[1]) < 1e-6
            and world_axis[2] > 0.30,
            details=f"world_axis={world_axis}",
        )

    # --- Each hub carries exactly three arms at 120 degrees apart. ---------
    def measured_arm_center(hub, index: int) -> tuple[float, float, float] | None:
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
        # Arm directions are 120 deg apart on the local cone (by construction).
        for i in range(3):
            j = (i + 1) % 3
            di = _arm_local_dir(ARM_PHASES_DEG[i], 0.0)
            dj = _arm_local_dir(ARM_PHASES_DEG[j], 0.0)
            dot = sum(a * b for a, b in zip(di, dj))
            # cos(120deg)=-0.5 in the cone plane; full 3D dot = cos^2(cone)*(-0.5)+sin^2(cone)
            expected = _CB * _CB * (-0.5) + _SB * _SB
            ctx.check(
                f"{side} arms {i},{j} are 120 deg apart on the cone",
                abs(dot - expected) < 1e-6,
                details=f"dot={dot:.4f}, expected={expected:.4f}",
            )
        # Measured arm centers match the predicted world cone at rest.
        for index, phase in enumerate(ARM_PHASES_DEG):
            measured = measured_arm_center(hub, index)
            predicted = _arm_world_center(hub_xyz, m, phase, 0.0)
            ok = measured is not None and math.dist(measured, predicted) <= 0.025
            ctx.check(
                f"{side} arm_{index} sits on the predicted tilted cone at rest",
                ok,
                details=f"measured={measured}, predicted={predicted}",
            )

    # --- Rest pose: outward (blocking) arm roughly horizontal toward rail. --
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

    # --- Sweep clearance: arms never hit the column or base over a full turn.
    # The hub axes are level in Y and sit on the side faces, so arms swing in
    # the +/-X and Z directions away from the central column. Sample the full
    # rotation and check the minimum |x| reach of every arm tip stays outboard
    # of the column/base, and that arm tips clear the floor.
    col_half_x = COLUMN_BOT[0] / 2.0
    base_half_x = PLINTH_FOOT[0] / 2.0
    for hub_xyz, m, sign, side in (
        (HUB_RIGHT_XYZ, _M_RIGHT, 1.0, "right"),
        (HUB_LEFT_XYZ, _M_LEFT, -1.0, "left"),
    ):
        worst_inboard = None  # smallest outward reach of any arm tip
        worst_floor = None  # lowest arm tip z
        for step in range(72):
            q = step * math.radians(5.0)
            for phase in ARM_PHASES_DEG:
                d_world = _matvec(m, _arm_local_dir(phase, q))
                axis_world = _matvec(m, (0.0, 0.0, 1.0))
                sx = hub_xyz[0] + ARM_START_Z * axis_world[0]
                sz = hub_xyz[2] + ARM_START_Z * axis_world[2]
                tip_x = sx + (ARM_L + ARM_R) * d_world[0]
                tip_z = sz + (ARM_L + ARM_R) * d_world[2]
                reach = sign * tip_x  # positive = outward of center
                worst_inboard = reach if worst_inboard is None else min(worst_inboard, reach)
                worst_floor = tip_z if worst_floor is None else min(worst_floor, tip_z)
        ctx.check(
            f"{side} arm sweep stays outboard of the column/base",
            worst_inboard is not None and worst_inboard > base_half_x + 0.02,
            details=(
                f"worst_inboard={worst_inboard:.4f}, "
                f"col_half_x={col_half_x:.3f}, base_half_x={base_half_x:.3f}"
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
        boss = cabinet.get_visual(f"rotor_boss_{side}")
        ctx.allow_overlap(
            hub,
            cabinet,
            elem_a=hub_core,
            elem_b=boss,
            reason=(
                f"{side} hub collar is intentionally seated onto its bearing "
                "boss face so the rotor reads as mounted, not floating"
            ),
        )
        ctx.expect_contact(hub, cabinet, elem_a=hub_core, elem_b=boss, contact_tol=1e-4)

    # --- Twin glass swing-barrier panels: centered post, two 90 deg leaves. --
    for swing, lower, upper, side in (
        (swing_left, -math.pi / 2.0, 0.0, "left"),
        (swing_right, 0.0, math.pi / 2.0, "right"),
    ):
        ctx.check(
            f"glass_panel_{side}_swing is REVOLUTE about vertical +Z",
            swing.articulation_type == ArticulationType.REVOLUTE
            and swing.axis == (0.0, 0.0, 1.0),
            details=f"type={swing.articulation_type}, axis={swing.axis}",
        )
        ctx.check(
            f"glass_panel_{side}_swing has a 90 degree forward-opening range",
            swing.motion_limits is not None
            and abs(swing.motion_limits.lower - lower) < 1e-6
            and abs(swing.motion_limits.upper - upper) < 1e-6,
            details=f"limits={swing.motion_limits}",
        )

    post_aabb = ctx.part_element_world_aabb(cabinet, elem="hinge_post")
    ctx.check(
        "shared hinge post is centered in front of the head and stands on the floor",
        post_aabb is not None
        and abs((post_aabb[0][0] + post_aabb[1][0]) / 2.0) < 0.015
        and post_aabb[0][1] > HEAD_BOX[1] / 2.0
        and post_aabb[0][2] < 0.005
        and post_aabb[1][2] > 0.95,
        details=f"post={post_aabb}",
    )

    frame_names = {"frame_top", "frame_bottom", "frame_free_edge"}
    panel_visual_names = {}
    for panel, side, sign in (
        (glass_panel_left, "left", -1.0),
        (glass_panel_right, "right", 1.0),
    ):
        panel_visual_names[side] = {v.name for v in panel.visuals}
        panel_aabb = ctx.part_world_aabb(panel)
        glass_aabb = ctx.part_element_world_aabb(panel, elem="glass_pane")
        ctx.check(
            f"glass_panel_{side} reaches waist height above the floor",
            panel_aabb is not None
            and panel_aabb[0][2] < 0.12
            and 0.85 < panel_aabb[1][2] < 1.10,
            details=f"panel_aabb={panel_aabb}",
        )
        ctx.check(
            f"glass_panel_{side} starts on the {side} side of the centered post",
            glass_aabb is not None
            and sign * glass_aabb[0 if sign > 0 else 1][0] > POST_R - 0.02
            and (glass_aabb[1][0] - glass_aabb[0][0]) > 0.45
            and glass_aabb[0][1] > HEAD_BOX[1] / 2.0
            and abs((glass_aabb[0][1] + glass_aabb[1][1]) / 2.0 - HINGE_Y) < 0.02
            and PANEL_BOT_Z - 0.02 < glass_aabb[0][2] < PANEL_BOT_Z + 0.02,
            details=f"glass={glass_aabb}",
        )
        ctx.check(
            f"glass_panel_{side} carries three frame bars plus hinge clips",
            frame_names.issubset(panel_visual_names[side])
            and any(n.startswith("hinge_clip_") for n in panel_visual_names[side]),
            details=f"visuals={sorted(panel_visual_names[side])}",
        )

    left_rest = ctx.part_world_aabb(glass_panel_left)
    right_rest = ctx.part_world_aabb(glass_panel_right)
    with ctx.pose({swing_left: -math.pi / 2.0, swing_right: math.pi / 2.0}):
        left_open = ctx.part_world_aabb(glass_panel_left)
        right_open = ctx.part_world_aabb(glass_panel_right)
    ctx.check(
        "both glass panels rotate forward into +Y instead of backward",
        left_rest is not None
        and right_rest is not None
        and left_open is not None
        and right_open is not None
        and left_open[1][1] > left_rest[1][1] + 0.20
        and right_open[1][1] > right_rest[1][1] + 0.20
        and left_open[0][1] > left_rest[0][1] - 0.05
        and right_open[0][1] > right_rest[0][1] - 0.05
        and abs(left_open[0][0]) < 0.06
        and abs(right_open[0][0]) < 0.06,
        details=f"left_rest={left_rest}, right_rest={right_rest}, left_open={left_open}, right_open={right_open}",
    )

    post_vis = cabinet.get_visual("hinge_post")
    for panel, side in ((glass_panel_left, "left"), (glass_panel_right, "right")):
        for i in range(3):
            clip_name = f"hinge_clip_{i}"
            clip_vis = panel.get_visual(clip_name)
            ctx.allow_overlap(
                panel,
                cabinet,
                elem_a=clip_vis,
                elem_b=post_vis,
                reason=(
                    f"{side} hinge clip {i} wraps around the centered fixed "
                    "post to represent the pivoting clamp connection"
                ),
            )
        ctx.expect_contact(
            panel,
            cabinet,
            elem_a="hinge_clip_1",
            elem_b="hinge_post",
            contact_tol=0.01,
            name=f"{side} middle hinge clip contacts the post",
        )

    for i in range(3):
        ctx.allow_overlap(
            glass_panel_left,
            glass_panel_right,
            elem_a=f"hinge_clip_{i}",
            elem_b=f"hinge_clip_{i}",
            reason=(
                f"left and right hinge clip {i} share the same centered hinge "
                "post as paired clamp hardware"
            ),
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
    left_panel_aabb_full = ctx.part_world_aabb(glass_panel_left)
    right_panel_aabb_full = ctx.part_world_aabb(glass_panel_right)
    ctx.check(
        "overall dimensions are plausible for a waist-high turnstile",
        cab_aabb is not None
        and left_panel_aabb_full is not None
        and right_panel_aabb_full is not None
        and 0.98 < cab_aabb[1][2] < 1.03
        and cab_aabb[0][2] > -0.001
        and 0.40 < ARM_L < 0.55
        and left_panel_aabb_full[0][0] > -1.0
        and right_panel_aabb_full[1][0] < 1.0,
        details=(
            f"cabinet={cab_aabb}, left_panel={left_panel_aabb_full}, "
            f"right_panel={right_panel_aabb_full}, arm_len={ARM_L}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
