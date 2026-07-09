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
# Ceiling-mounted sentry gun turret (inverted mount variant)
# Flat ceiling plate replaces the four-leg base; turret hangs downward.
# World frame: +X is the barrel (fire) direction at yaw = 0, +Z is up.
#
# Functional layers:
#   Mount:  ceiling_plate  (flat plate + boss + perforated shroud + bolts)
#   Yaw:    support_collar (inverted hex + trunnion cheeks hanging downward)
#   Pitch:  receiver       (hollow box, barrel bores, top panels, trunnion pins)
#   Recoil: barrel_cluster (2x2 barrels, muzzle shrouds, end caps, carriage)
#
# Articulation chain:
#   ceiling_plate --(continuous yaw, Z)--> support_collar
#   support_collar --(revolute pitch, Y trunnions, -10..+60 deg)--> receiver
#   receiver --(prismatic recoil, barrel axis, 0..0.06 m back)--> barrel_cluster
# ---------------------------------------------------------------------------

# ── Colors ──────────────────────────────────────────────────────────────────
YELLOW = (0.95, 0.76, 0.10, 1.0)
GUNMETAL = (0.15, 0.16, 0.18, 1.0)
DARK_GREY = (0.24, 0.25, 0.27, 1.0)
BARREL_GREY = (0.46, 0.47, 0.50, 1.0)

# ── Ceiling plate (mount layer) ────────────────────────────────────────────
CEILING_Z_TOP = 1.15            # top surface of plate (mounting elevation)
PLATE_R = 0.25                  # plate outer radius
PLATE_H = 0.04                  # plate thickness
PLATE_BOTTOM = CEILING_Z_TOP - PLATE_H  # 1.11

BOSS_R = 0.12                   # central mounting boss radius
BOSS_H = 0.06                   # boss height
BOSS_BOTTOM = PLATE_BOTTOM - BOSS_H  # 1.05

# Perforated shroud around boss
SHROUD_R_OUT = 0.145
SHROUD_R_IN = 0.135
SHROUD_TOP = PLATE_BOTTOM - 0.005   # 1.105
SHROUD_BOT = BOSS_BOTTOM + 0.005    # 1.055
SHROUD_H = SHROUD_TOP - SHROUD_BOT  # 0.05

# Bolt pattern on plate underside
N_BOLTS = 8
BOLT_CIRCLE_R = PLATE_R - 0.035
BOLT_HOLE_R = 0.010
BOLT_HEAD_R = 0.014
BOLT_HEAD_H = 0.008

# ── Yaw stage ──────────────────────────────────────────────────────────────
YAW_Z = BOSS_BOTTOM  # 1.05 – yaw joint at boss underside

HEX_FLAT_HALF = 0.135
HEX_CIRCUM_R = HEX_FLAT_HALF / math.cos(math.pi / 6.0)
HEX_H = 0.05

CHEEK_Y_IN, CHEEK_Y_OUT = 0.152, 0.182
CHEEK_X = 0.13
CHEEK_Z0 = 0.045    # distance below yaw origin where cheeks start
CHEEK_Z1 = 0.29     # distance below yaw origin where cheeks end
PIVOT_OFFSET = 0.24  # distance below yaw origin for pitch axis

# ── Receiver (local frame origin at pitch axis) ────────────────────────────
RECV_X0, RECV_X1 = -0.11, 0.25
RECV_HALF_Y = 0.149
RECV_HALF_Z = 0.13
RECV_WALL = 0.012
BORE_R = 0.0278

# ── 2×2 barrel layout (relative to pitch axis) ────────────────────────────
BARREL_DY = 0.07
BARREL_DZ = 0.058
BARREL_R_OUT, BARREL_R_IN = 0.028, 0.0185
BARREL_X0, BARREL_X1 = 0.04, 0.66
SHROUD_BARREL_R_OUT, SHROUD_BARREL_R_IN = 0.047, 0.0278
SHROUD_X0, SHROUD_LEN = 0.53, 0.115
CAP_R_OUT, CAP_R_IN = 0.048, 0.0295
CAP_X0, CAP_LEN = 0.636, 0.018

RECOIL_TRAVEL = 0.06
PITCH_LO, PITCH_HI = math.radians(-10.0), math.radians(60.0)

PIVOT_WORLD_Z = YAW_Z - PIVOT_OFFSET  # 0.81


# ── Geometry helpers ───────────────────────────────────────────────────────

def _ceiling_plate_mesh() -> cq.Workplane:
    """Flat circular ceiling plate with central bore and bolt holes."""
    plate = cq.Workplane("XY").circle(PLATE_R).extrude(PLATE_H)
    # Central cable pass-through
    plate = plate.cut(cq.Workplane("XY").circle(0.035).extrude(PLATE_H))
    # Bolt holes around perimeter
    for i in range(N_BOLTS):
        ang = 2.0 * math.pi * i / N_BOLTS
        x = BOLT_CIRCLE_R * math.cos(ang)
        y = BOLT_CIRCLE_R * math.sin(ang)
        hole = (
            cq.Workplane("XY")
            .circle(BOLT_HOLE_R)
            .extrude(PLATE_H)
            .translate((x, y, 0))
        )
        plate = plate.cut(hole)
    return plate


def _boss_shroud_mesh() -> cq.Workplane:
    """Thin perforated cylindrical shell around the mounting boss."""
    shell = (
        cq.Workplane("XY")
        .circle(SHROUD_R_OUT)
        .circle(SHROUD_R_IN)
        .extrude(SHROUD_H)
    )
    cutters = []
    hole = 0.016
    rows = [0.006 + i * 0.014 for i in range(3)]
    for i, z in enumerate(rows):
        offset = 8.0 if i % 2 else 0.0
        for k in range(12):
            ang = offset + 15.0 * k
            cutter = (
                cq.Workplane("XY")
                .box(2.0 * SHROUD_R_OUT + 0.04, hole, hole)
                .translate((0.0, 0.0, z))
                .rotate((0, 0, 0), (0, 0, 1), ang)
            )
            cutters.append(cutter.val())
    return shell.cut(cq.Compound.makeCompound(cutters))


def _shroud_ring_mesh() -> cq.Workplane:
    """Annular disc bridging the boss surface to the perforated shroud.

    Inner radius is 3 mm under the boss radius so the ring embeds into the
    boss mesh, guaranteeing within-part connectivity.
    """
    return (
        cq.Workplane("XY")
        .circle(SHROUD_R_OUT)
        .circle(BOSS_R - 0.003)
        .extrude(0.006)
    )


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
        .box(
            depth - 2.0 * RECV_WALL,
            2.0 * (RECV_HALF_Y - RECV_WALL),
            2.0 * (RECV_HALF_Z - RECV_WALL),
        )
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


# ── Build ──────────────────────────────────────────────────────────────────

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sentry_gun_turret_ceiling")

    yellow = Material(name="safety_yellow", rgba=YELLOW)
    gunmetal = Material(name="gunmetal", rgba=GUNMETAL)
    dark_grey = Material(name="dark_grey_steel", rgba=DARK_GREY)
    barrel_grey = Material(name="barrel_grey", rgba=BARREL_GREY)

    # ========================================================== MOUNT LAYER
    ceiling = model.part("ceiling_plate")

    # Flat plate with bolt holes (bottom face at PLATE_BOTTOM)
    ceiling.visual(
        mesh_from_cadquery(_ceiling_plate_mesh(), "plate_body"),
        origin=Origin(xyz=(0.0, 0.0, PLATE_BOTTOM)),
        material=dark_grey,
        name="plate_body",
    )

    # Central mounting boss hanging from plate underside
    # Boss extends 3 mm into the plate to ensure within-part mesh connectivity.
    boss_overlap = 0.003
    boss_len = BOSS_H + boss_overlap
    ceiling.visual(
        mesh_from_cadquery(
            cq.Workplane("XY").circle(BOSS_R).extrude(boss_len),
            "mounting_boss",
        ),
        origin=Origin(xyz=(0.0, 0.0, BOSS_BOTTOM)),
        material=gunmetal,
        name="mounting_boss",
    )

    # Perforated shroud wrap around boss
    ceiling.visual(
        mesh_from_cadquery(_boss_shroud_mesh(), "boss_shroud"),
        origin=Origin(xyz=(0.0, 0.0, SHROUD_BOT)),
        material=yellow,
        name="boss_shroud",
    )

    # Annular rings connecting boss to shroud (structural continuity)
    ring_mesh = mesh_from_cadquery(_shroud_ring_mesh(), "shroud_ring")
    ceiling.visual(
        ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, SHROUD_TOP - 0.006)),
        material=yellow,
        name="shroud_ring_top",
    )
    ceiling.visual(
        ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, SHROUD_BOT)),
        material=yellow,
        name="shroud_ring_bot",
    )

    # Bolt heads on plate underside (non-moving decoration, inlined)
    for i in range(N_BOLTS):
        ang = 2.0 * math.pi * i / N_BOLTS
        x = BOLT_CIRCLE_R * math.cos(ang)
        y = BOLT_CIRCLE_R * math.sin(ang)
        ceiling.visual(
            Cylinder(radius=BOLT_HEAD_R, length=BOLT_HEAD_H),
            origin=Origin(xyz=(x, y, PLATE_BOTTOM - 0.5 * BOLT_HEAD_H)),
            material=gunmetal,
            name=f"bolt_head_{i}",
        )

    # =========================================================== YAW LAYER
    collar = model.part("support_collar")

    # Hex collar: top face at yaw origin, extending downward
    hex_solid = (
        cq.Workplane("XY")
        .polygon(6, 2.0 * HEX_CIRCUM_R)
        .extrude(HEX_H)
        .translate((0, 0, -HEX_H))
    )
    collar.visual(
        mesh_from_cadquery(hex_solid, "hex_collar"),
        material=gunmetal,
        name="hex_collar",
    )

    # Trunnion cheeks, flanges, and bearing discs (hanging downward)
    yc = 0.5 * (CHEEK_Y_IN + CHEEK_Y_OUT)
    cheek_h = CHEEK_Z1 - CHEEK_Z0
    cheek_zc = -0.5 * (CHEEK_Z0 + CHEEK_Z1)
    for i, sy in enumerate((1.0, -1.0)):
        collar.visual(
            Box((CHEEK_X, CHEEK_Y_OUT - CHEEK_Y_IN, cheek_h)),
            origin=Origin(xyz=(0.0, sy * yc, cheek_zc)),
            material=yellow,
            name=f"trunnion_cheek_{i}",
        )
        collar.visual(
            Box((CHEEK_X, 0.085, 0.03)),
            origin=Origin(xyz=(0.0, sy * 0.1425, -0.055)),
            material=yellow,
            name=f"cheek_flange_{i}",
        )
        collar.visual(
            Cylinder(radius=0.055, length=0.022),
            origin=Origin(
                xyz=(0.0, sy * 0.193, -PIVOT_OFFSET),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=gunmetal,
            name=f"trunnion_disc_{i}",
        )

    model.articulation(
        "plate_to_collar_yaw",
        ArticulationType.CONTINUOUS,
        parent=ceiling,
        child=collar,
        origin=Origin(xyz=(0.0, 0.0, YAW_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=2.0),
    )

    # ========================================================= PITCH LAYER
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
            origin=Origin(
                xyz=(0.0, sy * 0.17, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)
            ),
            material=gunmetal,
            name=f"trunnion_pin_{i}",
        )

    model.articulation(
        "collar_to_receiver_pitch",
        ArticulationType.REVOLUTE,
        parent=collar,
        child=receiver,
        origin=Origin(xyz=(0.0, 0.0, -PIVOT_OFFSET)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=1.5, lower=PITCH_LO, upper=PITCH_HI
        ),
    )

    # ======================================================== RECOIL LAYER
    cluster = model.part("barrel_cluster")
    cluster.visual(
        Box((0.10, 0.24, 0.20)),
        origin=Origin(xyz=(0.07, 0.0, 0.0)),
        material=gunmetal,
        name="carriage",
    )

    barrel_mesh = mesh_from_cadquery(
        _round_tube(BARREL_R_OUT, BARREL_R_IN, BARREL_X1 - BARREL_X0),
        "barrel_tube",
    )
    shroud_mesh = mesh_from_cadquery(
        _round_tube(SHROUD_BARREL_R_OUT, SHROUD_BARREL_R_IN, SHROUD_LEN),
        "muzzle_shroud",
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
        motion_limits=MotionLimits(
            effort=50.0, velocity=1.0, lower=0.0, upper=RECOIL_TRAVEL
        ),
    )

    return model


# ── Tests ──────────────────────────────────────────────────────────────────

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    ceiling = object_model.get_part("ceiling_plate")
    collar = object_model.get_part("support_collar")
    receiver = object_model.get_part("receiver")
    cluster = object_model.get_part("barrel_cluster")
    yaw = object_model.get_articulation("plate_to_collar_yaw")
    pitch = object_model.get_articulation("collar_to_receiver_pitch")
    recoil = object_model.get_articulation("receiver_to_barrels_recoil")

    # ── intentional overlaps: trunnion pins captured in cheek bearings ──
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

    # ── intentional overlaps: barrels through snug receiver bores ───────
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

    # ── joint plan checks ──────────────────────────────────────────────
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

    # ── ceiling mount: flat plate at top, turret hangs below ──────────
    ceiling_aabb = ctx.part_world_aabb(ceiling)
    ctx.check(
        "ceiling plate top is at the mounting elevation (~1.15 m)",
        ceiling_aabb is not None
        and abs(ceiling_aabb[1][2] - CEILING_Z_TOP) < 0.005,
        details=f"ceiling aabb={ceiling_aabb}",
    )
    if ceiling_aabb is not None:
        plate_span_x = ceiling_aabb[1][0] - ceiling_aabb[0][0]
        plate_span_y = ceiling_aabb[1][1] - ceiling_aabb[0][1]
        plate_height = ceiling_aabb[1][2] - ceiling_aabb[0][2]
        ctx.check(
            "ceiling plate is a flat wide mount (not legs)",
            plate_span_x > 0.40
            and plate_span_y > 0.40
            and plate_height < 0.20,
            details=f"span=({plate_span_x:.3f}, {plate_span_y:.3f}), h={plate_height:.3f}",
        )

    recv_aabb = ctx.part_world_aabb(receiver)
    cluster_aabb = ctx.part_world_aabb(cluster)
    ctx.check(
        "receiver and barrels hang below the ceiling plate",
        recv_aabb is not None
        and ceiling_aabb is not None
        and recv_aabb[1][2] < ceiling_aabb[0][2] + 0.01,
        details=f"receiver top z={recv_aabb[1][2] if recv_aabb else None}, "
                f"plate bottom z={ceiling_aabb[0][2] if ceiling_aabb else None}",
    )

    # ── collar seating on boss underside ──────────────────────────────
    ctx.expect_gap(
        ceiling,
        collar,
        axis="z",
        max_gap=0.005,
        max_penetration=0.002,
        name="hex collar seats against the ceiling plate boss underside",
    )

    # ── barrel layout and containment ─────────────────────────────────
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

    # 2×2 barrel grid centred on the pitch axis
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
            "barrels form a 2x2 grid around the hanging pitch axis",
            abs(ys[0] + BARREL_DY) < 0.005
            and abs(ys[3] - BARREL_DY) < 0.005
            and abs(zs[0] - (PIVOT_WORLD_Z - BARREL_DZ)) < 0.015
            and abs(zs[3] - (PIVOT_WORLD_Z + BARREL_DZ)) < 0.015,
            details=f"barrel centers={centers}",
        )

    # ── mechanism pose checks ─────────────────────────────────────────
    # Positive pitch elevates the muzzles
    muzzle_rest = ctx.part_element_world_aabb(cluster, elem="muzzle_cap_0")
    with ctx.pose({pitch: 1.0}):
        muzzle_up = ctx.part_element_world_aabb(cluster, elem="muzzle_cap_0")
    ctx.check(
        "positive pitch elevates the muzzles upward from hanging rest",
        muzzle_rest is not None
        and muzzle_up is not None
        and muzzle_up[0][2] > muzzle_rest[0][2] + 0.25,
        details=f"rest={muzzle_rest}, elevated={muzzle_up}",
    )

    # Continuous yaw: quarter turn swings barrels from +X to +Y
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

    # Recoil: barrel cluster slides 0.06 m back into receiver
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
