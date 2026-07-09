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
# Stylized automated sentry gun turret, ~1.15 m tall, ~1.17 m wide footprint.
# World frame: +X is the barrel (fire) direction at yaw = 0, +Z is up.
#
# Articulation chain:
#   leg_base --(continuous yaw, Z)--> support_collar
#   support_collar --(revolute pitch, Y trunnions, -10..+60 deg)--> receiver
#   receiver --(prismatic recoil, barrel axis, 0..0.06 m back)--> barrel_cluster
# ---------------------------------------------------------------------------

# Colors
YELLOW = (0.95, 0.76, 0.10, 1.0)
GUNMETAL = (0.15, 0.16, 0.18, 1.0)
DARK_GREY = (0.24, 0.25, 0.27, 1.0)
BARREL_GREY = (0.46, 0.47, 0.50, 1.0)

# Leg geometry (in a leg-local frame: +X radial outward, +Z up)
LEG_ANGLES = (0.0, 60.0, 120.0, 180.0, 240.0, 300.0)  # 6 legs at even 60° spacing
HUB_PT = (0.10, 0.74)  # (radial, z) where the leg roots into the platform
KNEE_PT = (0.50, 0.30)
FOOT_PT = (0.72, 0.06)
LEG_W = 0.085  # tangential width of box-section strut
LEG_T = 0.052  # thickness of box-section strut
LEG_WALL = 0.009
FOOT_PAD_R = 0.075
FOOT_PAD_H = 0.05

# Column / platform
PLATFORM_R = 0.18
PLATFORM_Z0, PLATFORM_Z1 = 0.70, 0.78
COLUMN_R = 0.085
COLUMN_Z0, COLUMN_Z1 = 0.26, 0.72
SHELL_R_OUT, SHELL_R_IN = 0.112, 0.103
SHELL_Z0, SHELL_Z1 = 0.30, 0.71

# Yaw stage (support collar). Local frame origin at world (0, 0, 0.78).
YAW_Z = PLATFORM_Z1
HEX_FLAT_HALF = 0.135  # across-flats half-width
HEX_CIRCUM_R = HEX_FLAT_HALF / math.cos(math.pi / 6.0)
HEX_H = 0.05
CHEEK_Y_IN, CHEEK_Y_OUT = 0.152, 0.182
CHEEK_X = 0.13
CHEEK_Z0, CHEEK_Z1 = 0.045, 0.29  # local z
PIVOT_Z_LOCAL = 0.24  # pitch axis height above collar frame (world 1.02)

# Receiver (local frame origin on the pitch axis, world (0, 0, 1.02) at q=0)
RECV_X0, RECV_X1 = -0.11, 0.25
RECV_HALF_Y = 0.149
RECV_HALF_Z = 0.13
RECV_WALL = 0.012
# Snug sliding fit: bore is fractionally under the barrel OD so the barrels
# ride in contact with the receiver front wall (keeps the cluster supported).
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


def _rect_tube(length: float, w: float, t: float, wall: float) -> cq.Workplane:
    """Hollow rectangular box-section tube extruded along +Z."""
    return (
        cq.Workplane("XY")
        .rect(t, w)
        .rect(t - 2.0 * wall, w - 2.0 * wall)
        .extrude(length)
    )


def _segment_tube(
    p0: tuple[float, float],
    p1: tuple[float, float],
    ext0: float,
    ext1: float,
) -> cq.Workplane:
    """Hollow strut between two (radial, z) points in the leg-local XZ plane."""
    dx, dz = p1[0] - p0[0], p1[1] - p0[1]
    seg_len = math.hypot(dx, dz)
    ux, uz = dx / seg_len, dz / seg_len
    tube = _rect_tube(seg_len + ext0 + ext1, LEG_W, LEG_T, LEG_WALL)
    ang_deg = math.degrees(math.atan2(ux, uz))
    start = (p0[0] - ux * ext0, 0.0, p0[1] - uz * ext0)
    return tube.rotate((0, 0, 0), (0, 1, 0), ang_deg).translate(start)


def _round_tube(r_out: float, r_in: float, length: float) -> cq.Workplane:
    """Hollow round tube along +X starting at x=0."""
    tube = cq.Workplane("XY").circle(r_out).circle(r_in).extrude(length)
    return tube.rotate((0, 0, 0), (0, 1, 0), 90.0)


def _perforated_shell() -> cq.Workplane:
    """Thin cylindrical shell with a grid of square perforations (local z=0 base)."""
    h = SHELL_Z1 - SHELL_Z0
    shell = cq.Workplane("XY").circle(SHELL_R_OUT).circle(SHELL_R_IN).extrude(h)
    cutters = []
    hole = 0.024
    rows = [0.045 + i * 0.055 for i in range(6)]
    for i, z in enumerate(rows):
        offset = 10.0 if i % 2 else 0.0
        for k in range(9):  # each diametric box cuts two opposite holes
            ang = offset + 20.0 * k
            cutter = (
                cq.Workplane("XY")
                .box(2.0 * SHELL_R_OUT + 0.05, hole, hole)
                .translate((0.0, 0.0, z))
                .rotate((0, 0, 0), (0, 0, 1), ang)
            )
            cutters.append(cutter.val())
    return shell.cut(cq.Compound.makeCompound(cutters))


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

    # ------------------------------------------------------------------ base
    base = model.part("leg_base")

    # One leg solid (upper strut + lower strut) reused with yaw rotations.
    # Keep the hub-end extension short so the strut root embeds in the
    # platform cylinder without poking above its top face (z=0.78).
    leg_solid = _segment_tube(HUB_PT, KNEE_PT, 0.02, 0.03).union(
        _segment_tube(KNEE_PT, FOOT_PT, 0.03, 0.03)
    )
    leg_mesh = mesh_from_cadquery(leg_solid, "leg_strut")
    knee_dir = math.atan2(
        (HUB_PT[1] - FOOT_PT[1]), (FOOT_PT[0] - HUB_PT[0])
    )  # average downhill slope of the leg, as pitch about +Y
    for i, ang in enumerate(LEG_ANGLES):
        yaw = math.radians(ang)
        base.visual(
            leg_mesh,
            origin=Origin(rpy=(0.0, 0.0, yaw)),
            material=yellow,
            name=f"leg_strut_{i}",
        )
        # Dark gusset plate boxed over the knee joint.
        kx = KNEE_PT[0] * math.cos(yaw)
        ky = KNEE_PT[0] * math.sin(yaw)
        base.visual(
            Box((0.15, 0.10, 0.11)),
            origin=Origin(xyz=(kx, ky, KNEE_PT[1]), rpy=(0.0, knee_dir, yaw)),
            material=dark_grey,
            name=f"knee_gusset_{i}",
        )
        # Round foot pad on the ground.
        fx = FOOT_PT[0] * math.cos(yaw)
        fy = FOOT_PT[0] * math.sin(yaw)
        base.visual(
            Cylinder(radius=FOOT_PAD_R, length=FOOT_PAD_H),
            origin=Origin(xyz=(fx, fy, FOOT_PAD_H / 2.0)),
            material=dark_grey,
            name=f"foot_pad_{i}",
        )

    # Central column, perforated wrap, and the ring platform on top.
    base.visual(
        Cylinder(radius=COLUMN_R, length=COLUMN_Z1 - COLUMN_Z0),
        origin=Origin(xyz=(0.0, 0.0, 0.5 * (COLUMN_Z0 + COLUMN_Z1))),
        material=gunmetal,
        name="column_core",
    )
    base.visual(
        Cylinder(radius=0.10, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
        material=gunmetal,
        name="column_base_cap",
    )
    base.visual(
        mesh_from_cadquery(_perforated_shell(), "perforated_wrap"),
        origin=Origin(xyz=(0.0, 0.0, SHELL_Z0)),
        material=yellow,
        name="perforated_wrap",
    )
    base.visual(
        Cylinder(radius=0.135, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.685)),
        material=gunmetal,
        name="hub_ring",
    )
    base.visual(
        Cylinder(radius=PLATFORM_R, length=PLATFORM_Z1 - PLATFORM_Z0),
        origin=Origin(xyz=(0.0, 0.0, 0.5 * (PLATFORM_Z0 + PLATFORM_Z1))),
        material=yellow,
        name="ring_platform",
    )
    base.visual(
        Cylinder(radius=0.19, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, PLATFORM_Z1 - 0.0125)),
        material=yellow,
        name="platform_lip",
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
        "base_to_collar_yaw",
        ArticulationType.CONTINUOUS,
        parent=base,
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
        # Barrels extend along +X; -Y makes positive q elevate the muzzles.
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
        # Positive q slides the barrel cluster rearward into the receiver.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=1.0, lower=0.0, upper=RECOIL_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("leg_base")
    collar = object_model.get_part("support_collar")
    receiver = object_model.get_part("receiver")
    cluster = object_model.get_part("barrel_cluster")
    yaw = object_model.get_articulation("base_to_collar_yaw")
    pitch = object_model.get_articulation("collar_to_receiver_pitch")
    recoil = object_model.get_articulation("receiver_to_barrels_recoil")

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
    # receiver front wall (bore radius is fractionally under the barrel OD so
    # the recoiling cluster stays guided and supported by the receiver).
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

    # --- grounding, scale, proportions --------------------------------------
    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "foot pads rest on the ground plane",
        base_aabb is not None and -0.001 <= base_aabb[0][2] <= 0.004,
        details=f"base aabb={base_aabb}",
    )
    if base_aabb is not None:
        span_x = base_aabb[1][0] - base_aabb[0][0]
        span_y = base_aabb[1][1] - base_aabb[0][1]
        ctx.check(
            "six-leg splayed footprint is about 1.2–1.6 m wide",
            1.0 <= span_x <= 1.65 and 1.0 <= span_y <= 1.65,
            details=f"footprint spans=({span_x:.3f}, {span_y:.3f})",
        )
    recv_aabb = ctx.part_world_aabb(receiver)
    ctx.check(
        "turret stands about 1.2 m tall",
        recv_aabb is not None and 1.05 <= recv_aabb[1][2] <= 1.30,
        details=f"receiver aabb={recv_aabb}",
    )

    # --- six-leg base layout ---------------------------------------------------
    leg_strut_names = [f"leg_strut_{i}" for i in range(6)]
    for name in leg_strut_names:
        ctx.check(
            f"six-leg base has {name}",
            any(v.name == name for v in base.visuals),
            details=f"visuals={[v.name for v in base.visuals]}",
        )
    for i in range(6):
        ctx.check(
            f"six-leg base has knee_gusset_{i}",
            any(v.name == f"knee_gusset_{i}" for v in base.visuals),
        )
        ctx.check(
            f"six-leg base has foot_pad_{i}",
            any(v.name == f"foot_pad_{i}" for v in base.visuals),
        )
    # Verify even 60-degree angular spacing of foot pads about Z.
    foot_centers = []
    for i in range(6):
        ab = ctx.part_element_world_aabb(base, elem=f"foot_pad_{i}")
        if ab is not None:
            cx = 0.5 * (ab[0][0] + ab[1][0])
            cy = 0.5 * (ab[0][1] + ab[1][1])
            foot_centers.append((cx, cy))
    if len(foot_centers) == 6:
        angles_deg = sorted(
            math.degrees(math.atan2(cy, cx)) % 360.0 for cx, cy in foot_centers
        )
        gaps = [
            (angles_deg[(j + 1) % 6] - angles_deg[j]) % 360.0 for j in range(6)
        ]
        ctx.check(
            "six foot pads are evenly spaced at 60-degree intervals",
            all(abs(g - 60.0) < 2.0 for g in gaps),
            details=f"angles={angles_deg}, gaps={gaps}",
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
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0005,
        name="hex collar seats on the ring platform",
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

    # --- mechanism pose checks --------------------------------------------------
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

    # Off-axis proof of continuous yaw: the muzzle cluster sits far from the Z
    # axis along +X, so a quarter turn must move it to +Y.
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
