from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Stylized automated sentry gun turret on a solid cylindrical floor pedestal.
# ~1.15 m tall, pedestal ~0.80 m diameter footprint.
# World frame: +X is the barrel (fire) direction at yaw = 0, +Z is up.
#
# Articulation chain:
#   pedestal --(continuous yaw, Z)--> support_collar
#   support_collar --(revolute pitch, Y trunnions, -10..+60 deg)--> receiver
#   receiver --(prismatic recoil, barrel axis, 0..0.06 m back)--> barrel_cluster
# ---------------------------------------------------------------------------

# Colors
YELLOW = (0.95, 0.76, 0.10, 1.0)
GUNMETAL = (0.15, 0.16, 0.18, 1.0)
DARK_GREY = (0.24, 0.25, 0.27, 1.0)
BARREL_GREY = (0.46, 0.47, 0.50, 1.0)

# Pedestal drum (lathe profile around Z, origin at ground center)
PEDESTAL_TOP_Z = 0.78          # top face where yaw collar seats
BASE_PLATE_R = 0.40            # wide ground-contact flange
BASE_PLATE_H = 0.040
SHAFT_R = 0.26                 # main column radius
SHAFT_BAND_R = 0.275           # reinforcement ring radius
TOP_FLANGE_R = 0.31            # mounting flange radius
TOP_FLANGE_H = 0.06

# Yaw stage (support collar). Local frame origin at world (0, 0, PEDESTAL_TOP_Z).
YAW_Z = PEDESTAL_TOP_Z
HEX_FLAT_HALF = 0.135          # across-flats half-width
HEX_CIRCUM_R = HEX_FLAT_HALF / math.cos(math.pi / 6.0)
HEX_H = 0.05
CHEEK_Y_IN, CHEEK_Y_OUT = 0.152, 0.182
CHEEK_X = 0.13
CHEEK_Z0, CHEEK_Z1 = 0.045, 0.29
PIVOT_Z_LOCAL = 0.24           # pitch axis height above collar frame (world 1.02)

# Receiver (local frame origin on the pitch axis, world (0, 0, 1.02) at q=0)
RECV_X0, RECV_X1 = -0.11, 0.25
RECV_HALF_Y = 0.149
RECV_HALF_Z = 0.13
RECV_WALL = 0.012
BORE_R = 0.0278

# 2x2 barrel layout (relative to the pitch axis / receiver frame)
BARREL_DY = 0.07
BARREL_DZ = 0.058
BARREL_R_OUT, BARREL_R_IN = 0.028, 0.0185
BARREL_X0, BARREL_X1 = 0.04, 0.66
SHROUD_R_OUT, SHROUD_R_IN = 0.047, 0.0278
SHROUD_X0, SHROUD_LEN = 0.53, 0.115
CAP_R_OUT, CAP_R_IN = 0.048, 0.0295
CAP_X0, CAP_LEN = 0.636, 0.018

RECOIL_TRAVEL = 0.06
PITCH_LO, PITCH_HI = math.radians(-10.0), math.radians(60.0)

# Number of bolt indicators on the top flange
N_FLANGE_BOLTS = 8


def _pedestal_drum() -> cq.Workplane:
    """Lathe-revolved solid cylindrical pedestal drum (base flange → shaft → top flange)."""
    profile = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0)
        .lineTo(BASE_PLATE_R, 0.0)
        .lineTo(BASE_PLATE_R, BASE_PLATE_H)
        .lineTo(SHAFT_R + 0.02, BASE_PLATE_H + 0.025)   # lower chamfer
        .lineTo(SHAFT_R, BASE_PLATE_H + 0.06)            # shaft start
        .lineTo(SHAFT_R, PEDESTAL_TOP_Z - TOP_FLANGE_H)  # shaft end
        .lineTo(TOP_FLANGE_R, PEDESTAL_TOP_Z - TOP_FLANGE_H + 0.02)  # upper flare
        .lineTo(TOP_FLANGE_R, PEDESTAL_TOP_Z)             # top face
        .lineTo(0.0, PEDESTAL_TOP_Z)
        .close()
    )
    return profile.revolve(360, (0, 0), (0, 1))


def _round_tube(r_out: float, r_in: float, length: float) -> cq.Workplane:
    """Hollow round tube along +X starting at x=0."""
    tube = cq.Workplane("XY").circle(r_out).circle(r_in).extrude(length)
    return tube.rotate((0, 0, 0), (0, 1, 0), 90.0)


def _receiver_shell() -> cq.Workplane:
    """Hollow boxy receiver with 4 barrel bores and 2 recessed top pockets."""
    depth = RECV_X1 - RECV_X0
    cx = 0.5 * (RECV_X0 + RECV_X1)
    shell = (
        cq.Workplane("XY")
        .box(depth, 2.0 * RECV_HALF_Y, 2.0 * RECV_HALF_Z)
        .translate((cx, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.012)
    )
    cavity = (
        cq.Workplane("XY")
        .box(depth - 2.0 * RECV_WALL, 2.0 * (RECV_HALF_Y - RECV_WALL), 2.0 * (RECV_HALF_Z - RECV_WALL))
        .translate((cx, 0.0, 0.0))
    )
    shell = shell.cut(cavity)
    for sy in (1.0, -1.0):
        for sz in (1.0, -1.0):
            bore = (
                cq.Workplane("XY")
                .circle(BORE_R)
                .extrude(0.10)
                .rotate((0, 0, 0), (0, 1, 0), 90.0)
                .translate((RECV_X1 - 0.05, sy * BARREL_DY, sz * BARREL_DZ))
            )
            shell = shell.cut(bore)
    for sy in (1.0, -1.0):
        pocket = (
            cq.Workplane("XY")
            .box(0.15, 0.115, 0.02)
            .translate((0.07, sy * 0.0725, RECV_HALF_Z + 0.002))
        )
        shell = shell.cut(pocket)
    return shell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sentry_gun_turret")

    yellow = Material(name="safety_yellow", rgba=YELLOW)
    gunmetal = Material(name="gunmetal", rgba=GUNMETAL)
    dark_grey = Material(name="dark_grey_steel", rgba=DARK_GREY)
    barrel_grey = Material(name="barrel_grey", rgba=BARREL_GREY)

    # ------------------------------------------------------------------ pedestal
    pedestal = model.part("pedestal")

    # Main drum body (lathe-revolved solid)
    pedestal.visual(
        mesh_from_cadquery(_pedestal_drum(), "pedestal_drum"),
        material=yellow,
        name="pedestal_drum",
    )

    # Reinforcement band around shaft mid-height
    band_z = 0.5 * (BASE_PLATE_H + 0.06 + PEDESTAL_TOP_Z - TOP_FLANGE_H)
    pedestal.visual(
        Cylinder(radius=SHAFT_BAND_R, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, band_z)),
        material=dark_grey,
        name="shaft_band",
    )

    # Dark base rim ring (visual accent at the ground edge)
    pedestal.visual(
        Cylinder(radius=BASE_PLATE_R + 0.005, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark_grey,
        name="base_rim",
    )

    # Bolt-head indicators on the top flange (evenly spaced)
    bolt_r = TOP_FLANGE_R - 0.035
    bolt_z = PEDESTAL_TOP_Z - 0.003
    for i in range(N_FLANGE_BOLTS):
        ang = 2.0 * math.pi * i / N_FLANGE_BOLTS
        bx = bolt_r * math.cos(ang)
        by = bolt_r * math.sin(ang)
        pedestal.visual(
            Cylinder(radius=0.012, length=0.008),
            origin=Origin(xyz=(bx, by, bolt_z)),
            material=gunmetal,
            name=f"flange_bolt_{i}",
        )

    # --------------------------------------------------------------- collar
    collar = model.part("support_collar")
    hex_solid = (
        cq.Workplane("XY").polygon(6, 2.0 * HEX_CIRCUM_R).extrude(HEX_H)
    )
    collar.visual(
        mesh_from_cadquery(hex_solid, "hex_collar"),
        material=gunmetal,
        name="hex_collar",
    )
    for i, sy in enumerate((1.0, -1.0)):
        yc = 0.5 * (CHEEK_Y_IN + CHEEK_Y_OUT)
        collar.visual(
            Box((CHEEK_X, CHEEK_Y_OUT - CHEEK_Y_IN, CHEEK_Z1 - CHEEK_Z0)),
            origin=Origin(xyz=(0.0, sy * yc, 0.5 * (CHEEK_Z0 + CHEEK_Z1))),
            material=yellow,
            name=f"trunnion_cheek_{i}",
        )
        collar.visual(
            Box((CHEEK_X, 0.085, 0.03)),
            origin=Origin(xyz=(0.0, sy * 0.1425, 0.06)),
            material=yellow,
            name=f"cheek_flange_{i}",
        )
        collar.visual(
            Cylinder(radius=0.055, length=0.022),
            origin=Origin(xyz=(0.0, sy * 0.193, PIVOT_Z_LOCAL), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=gunmetal,
            name=f"trunnion_disc_{i}",
        )

    model.articulation(
        "pedestal_to_collar_yaw",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=collar,
        origin=Origin(xyz=(0.0, 0.0, YAW_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=2.0),
    )

    # ------------------------------------------------------------- receiver
    receiver = model.part("receiver")
    receiver.visual(
        mesh_from_cadquery(_receiver_shell(), "receiver_shell"),
        material=yellow,
        name="receiver_shell",
    )
    for i, sy in enumerate((1.0, -1.0)):
        receiver.visual(
            Box((0.146, 0.111, 0.006)),
            origin=Origin(xyz=(0.07, sy * 0.0725, RECV_HALF_Z - 0.0055)),
            material=gunmetal,
            name=f"top_panel_{i}",
        )
        receiver.visual(
            Cylinder(radius=0.032, length=0.06),
            origin=Origin(xyz=(0.0, sy * 0.17, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=gunmetal,
            name=f"trunnion_pin_{i}",
        )

    model.articulation(
        "collar_to_receiver_pitch",
        ArticulationType.REVOLUTE,
        parent=collar,
        child=receiver,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z_LOCAL)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.5, lower=PITCH_LO, upper=PITCH_HI),
    )

    # -------------------------------------------------------- barrel cluster
    cluster = model.part("barrel_cluster")
    cluster.visual(
        Box((0.10, 0.24, 0.20)),
        origin=Origin(xyz=(0.07, 0.0, 0.0)),
        material=gunmetal,
        name="carriage",
    )
    barrel_mesh = mesh_from_cadquery(
        _round_tube(BARREL_R_OUT, BARREL_R_IN, BARREL_X1 - BARREL_X0), "barrel_tube"
    )
    shroud_mesh = mesh_from_cadquery(
        _round_tube(SHROUD_R_OUT, SHROUD_R_IN, SHROUD_LEN), "muzzle_shroud"
    )
    cap_mesh = mesh_from_cadquery(
        _round_tube(CAP_R_OUT, CAP_R_IN, CAP_LEN), "muzzle_cap"
    )
    idx = 0
    for sz in (1.0, -1.0):
        for sy in (1.0, -1.0):
            y, z = sy * BARREL_DY, sz * BARREL_DZ
            cluster.visual(
                barrel_mesh,
                origin=Origin(xyz=(BARREL_X0, y, z)),
                material=barrel_grey,
                name=f"barrel_{idx}",
            )
            cluster.visual(
                shroud_mesh,
                origin=Origin(xyz=(SHROUD_X0, y, z)),
                material=yellow,
                name=f"muzzle_shroud_{idx}",
            )
            cluster.visual(
                cap_mesh,
                origin=Origin(xyz=(CAP_X0, y, z)),
                material=gunmetal,
                name=f"muzzle_cap_{idx}",
            )
            idx += 1

    model.articulation(
        "receiver_to_barrels_recoil",
        ArticulationType.PRISMATIC,
        parent=receiver,
        child=cluster,
        origin=Origin(),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=1.0, lower=0.0, upper=RECOIL_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pedestal = object_model.get_part("pedestal")
    collar = object_model.get_part("support_collar")
    receiver = object_model.get_part("receiver")
    cluster = object_model.get_part("barrel_cluster")
    yaw = object_model.get_articulation("pedestal_to_collar_yaw")
    pitch = object_model.get_articulation("collar_to_receiver_pitch")
    recoil = object_model.get_articulation("receiver_to_barrels_recoil")

    # --- pedestal structure checks -----------------------------------------
    ped_aabb = ctx.part_world_aabb(pedestal)
    ctx.check(
        "pedestal rests on the ground plane",
        ped_aabb is not None and -0.001 <= ped_aabb[0][2] <= 0.004,
        details=f"pedestal aabb={ped_aabb}",
    )
    if ped_aabb is not None:
        span_x = ped_aabb[1][0] - ped_aabb[0][0]
        span_y = ped_aabb[1][1] - ped_aabb[0][1]
        ctx.check(
            "pedestal footprint is a wide circular drum (>=0.70 m diameter)",
            span_x >= 0.70 and span_y >= 0.70,
            details=f"footprint spans=({span_x:.3f}, {span_y:.3f})",
        )
        # Circular drum: X and Y spans should be approximately equal
        ctx.check(
            "pedestal footprint is approximately circular (equal X/Y spans)",
            abs(span_x - span_y) < 0.02,
            details=f"spans=({span_x:.3f}, {span_y:.3f})",
        )
    ped_height = (ped_aabb[1][2] - ped_aabb[0][2]) if ped_aabb else 0.0
    ctx.check(
        "pedestal top face reaches the yaw collar seat height (~0.78 m)",
        ped_aabb is not None and abs(ped_aabb[1][2] - PEDESTAL_TOP_Z) < 0.01,
        details=f"pedestal top z={ped_aabb[1][2] if ped_aabb else None}",
    )

    # --- pedestal visuals: drum body, band, bolts, rim ----------------------
    for elem_name in ("pedestal_drum", "shaft_band", "base_rim"):
        ctx.check(
            f"pedestal has {elem_name} visual",
            ctx.part_element_world_aabb(pedestal, elem=elem_name) is not None,
        )
    for i in range(N_FLANGE_BOLTS):
        ctx.check(
            f"flange_bolt_{i} exists on pedestal top",
            ctx.part_element_world_aabb(pedestal, elem=f"flange_bolt_{i}") is not None,
        )

    # Intentional captured-shaft embedding: receiver trunnion pins ride inside
    # the collar cheek bearings / trunnion discs.
    for i in range(2):
        for cheek_elem in (f"trunnion_cheek_{i}", f"trunnion_disc_{i}"):
            ctx.allow_overlap(
                receiver,
                collar,
                elem_a=f"trunnion_pin_{i}",
                elem_b=cheek_elem,
                reason="trunnion pin is intentionally captured inside the cheek bearing and disc",
            )
        ctx.expect_overlap(
            receiver,
            collar,
            axes="y",
            elem_a=f"trunnion_pin_{i}",
            elem_b=f"trunnion_cheek_{i}",
            min_overlap=0.02,
            name=f"trunnion pin {i} is seated through the cheek bearing",
        )

    # Intentional snug sliding fit: grey barrels run through bores in the
    # receiver front wall.
    for i in range(4):
        ctx.allow_overlap(
            cluster,
            receiver,
            elem_a=f"barrel_{i}",
            elem_b="receiver_shell",
            reason="barrel slides through a snug guide bore in the receiver front wall",
        )
        ctx.expect_overlap(
            cluster,
            receiver,
            axes="x",
            elem_a=f"barrel_{i}",
            elem_b="receiver_shell",
            min_overlap=0.10,
            name=f"barrel {i} stays engaged in its receiver guide bore",
        )

    # --- joint plan checks -------------------------------------------------
    ctx.check(
        "yaw joint is continuous about Z",
        yaw.articulation_type == ArticulationType.CONTINUOUS
        and tuple(yaw.axis) == (0.0, 0.0, 1.0),
        details=f"type={yaw.articulation_type}, axis={yaw.axis}",
    )
    pl = pitch.motion_limits
    ctx.check(
        "pitch joint is revolute about horizontal Y trunnion axis, -10..+60 deg",
        pitch.articulation_type == ArticulationType.REVOLUTE
        and tuple(pitch.axis) == (0.0, -1.0, 0.0)
        and pl is not None
        and abs(pl.lower - PITCH_LO) < 1e-6
        and abs(pl.upper - PITCH_HI) < 1e-6,
        details=f"type={pitch.articulation_type}, axis={pitch.axis}, limits={pl}",
    )
    rl = recoil.motion_limits
    ctx.check(
        "recoil joint is prismatic along barrel axis with 0.06 m travel",
        recoil.articulation_type == ArticulationType.PRISMATIC
        and tuple(recoil.axis) == (-1.0, 0.0, 0.0)
        and rl is not None
        and rl.lower == 0.0
        and abs(rl.upper - RECOIL_TRAVEL) < 1e-9,
        details=f"type={recoil.articulation_type}, axis={recoil.axis}, limits={rl}",
    )

    # --- overall scale -------------------------------------------------------
    recv_aabb = ctx.part_world_aabb(receiver)
    ctx.check(
        "turret stands about 1.1-1.3 m tall",
        recv_aabb is not None and 1.05 <= recv_aabb[1][2] <= 1.30,
        details=f"receiver aabb={recv_aabb}",
    )

    # --- 2x2 barrel layout ---------------------------------------------------
    centers = []
    for i in range(4):
        ab = ctx.part_element_world_aabb(cluster, elem=f"barrel_{i}")
        ctx.check(f"barrel_{i} exists", ab is not None)
        if ab is not None:
            centers.append(
                (0.5 * (ab[0][1] + ab[1][1]), 0.5 * (ab[0][2] + ab[1][2]))
            )
    if len(centers) == 4:
        ys = sorted(c[0] for c in centers)
        zs = sorted(c[1] for c in centers)
        ctx.check(
            "barrels form a 2x2 grid around the pitch axis",
            abs(ys[0] + BARREL_DY) < 0.005
            and abs(ys[3] - BARREL_DY) < 0.005
            and abs(zs[0] - (1.02 - BARREL_DZ)) < 0.01
            and abs(zs[3] - (1.02 + BARREL_DZ)) < 0.01,
            details=f"barrel centers={centers}",
        )

    # --- seating and mounting -------------------------------------------------
    ctx.expect_gap(
        collar,
        pedestal,
        axis="z",
        max_gap=0.002,
        max_penetration=0.002,
        name="hex collar seats on the pedestal top face",
    )
    ctx.expect_within(
        cluster,
        receiver,
        axes="yz",
        inner_elem="carriage",
        margin=0.0,
        name="recoil carriage rides inside the receiver cavity",
    )
    ctx.expect_overlap(
        cluster,
        receiver,
        axes="x",
        elem_a="carriage",
        min_overlap=0.05,
        name="carriage retained inside receiver at rest",
    )

    # --- mechanism pose checks ------------------------------------------------
    muzzle_rest = ctx.part_element_world_aabb(cluster, elem="muzzle_cap_0")
    with ctx.pose({pitch: 1.0}):
        muzzle_up = ctx.part_element_world_aabb(cluster, elem="muzzle_cap_0")
    ctx.check(
        "positive pitch elevates the muzzles",
        muzzle_rest is not None
        and muzzle_up is not None
        and muzzle_up[0][2] > muzzle_rest[0][2] + 0.25,
        details=f"rest={muzzle_rest}, elevated={muzzle_up}",
    )

    # Off-axis proof of continuous yaw
    cl_rest = ctx.part_world_aabb(cluster)
    with ctx.pose({yaw: math.pi / 2.0}):
        cl_yawed = ctx.part_world_aabb(cluster)
    ctx.check(
        "quarter-turn yaw swings the off-axis barrel cluster to +Y",
        cl_rest is not None
        and cl_yawed is not None
        and cl_rest[1][0] > 0.5
        and cl_yawed[1][1] > 0.5
        and cl_yawed[1][0] < cl_rest[1][0] - 0.3,
        details=f"rest={cl_rest}, yawed={cl_yawed}",
    )

    with ctx.pose({recoil: RECOIL_TRAVEL}):
        cl_back = ctx.part_world_aabb(cluster)
        ctx.expect_overlap(
            cluster,
            receiver,
            axes="x",
            elem_a="carriage",
            min_overlap=0.05,
            name="carriage retained inside receiver at full recoil",
        )
    ctx.check(
        "recoil slides the barrel cluster 0.06 m back into the receiver",
        cl_rest is not None
        and cl_back is not None
        and abs((cl_rest[1][0] - cl_back[1][0]) - RECOIL_TRAVEL) < 0.005,
        details=f"rest={cl_rest}, recoiled={cl_back}",
    )

    return ctx.report()


object_model = build_object_model()
