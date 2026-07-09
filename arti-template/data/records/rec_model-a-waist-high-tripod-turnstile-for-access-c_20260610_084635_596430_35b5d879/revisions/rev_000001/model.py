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
)

# ---------------------------------------------------------------------------
# Layout constants (world frame: X = width, +Y = toward the user, Z = up).
# ---------------------------------------------------------------------------
COLUMN_W = 0.42
COLUMN_D = 0.18
COLUMN_H = 0.76
COLUMN_FRONT_Y = COLUMN_D / 2.0  # 0.09

HEAD_W = 0.46
HEAD_D = 0.28
HEAD_Y = 0.05  # head box center is shifted forward; back faces stay flush
HEAD_Z0 = 0.855
HEAD_Z1 = 0.98

LOFT_Z0 = 0.755
LOFT_Z1 = 0.86

GRANITE_SIZE = (0.49, 0.31, 0.022)
GRANITE_CENTER = (0.0, HEAD_Y, 0.988)

# Rotor axis: perpendicular to the 45 deg chamfer face, pointing forward-down.
_C = math.sqrt(0.5)
AXIS_DIR = (0.0, _C, -_C)
ROT_RPY = (-3.0 * math.pi / 4.0, 0.0, 0.0)  # maps local +Z onto AXIS_DIR

ANCHOR = (0.0, 0.14, 0.81)  # point on/just inside the chamfer face
BOSS_R = 0.042
BOSS_L = 0.08
STANDOFF = 0.055  # boss face (joint origin) distance from ANCHOR along axis

JOINT_XYZ = (
    ANCHOR[0],
    ANCHOR[1] + STANDOFF * _C,
    ANCHOR[2] - STANDOFF * _C,
)

# Tripod arms: 40 mm dia chrome tubes on a 45 deg cone about the hub axis.
# With the axis tilted 45 deg down-forward, the cone puts one arm exactly
# horizontal pointing at the user (blocking) while the other two splay down,
# and no arm can ever swing backward into the cabinet at any rotation.
ARM_R = 0.020
ARM_L = 0.48
ARM_START_Z = 0.018  # local, just clear of the boss contact plane
CONE = math.radians(45.0)
ARM_PHASES_DEG = (270.0, 30.0, 150.0)  # arm_0 blocks the lane at q = 0

COLLAR_R = 0.036
COLLAR_LEN = 0.034
HUB_RX = 0.075
HUB_RY = 0.058
HUB_THICK = 0.040


def _column_mesh():
    """Hollow thin-wall lower column with a recessed access-panel outline."""
    col = (
        cq.Workplane("XY")
        .box(COLUMN_W, COLUMN_D, COLUMN_H, centered=(True, True, False))
        .faces("<Z")
        .shell(-0.004)
    )
    # Recessed rectangular access-panel groove engraved into the front wall.
    # The cutter straddles the outer surface (y 0.0875..0.0925) so it engraves
    # 2.5 mm into the 4 mm wall and leaves the panel plate attached.
    outer = cq.Workplane("XY").box(0.312, 0.0050, 0.522).translate((0.0, COLUMN_FRONT_Y, 0.42))
    inner = cq.Workplane("XY").box(0.300, 0.0065, 0.510).translate((0.0, COLUMN_FRONT_Y, 0.42))
    col = col.cut(outer.cut(inner))
    return mesh_from_cadquery(col, "column_shell", tolerance=0.0008, angular_tolerance=0.1)


def _transition_mesh():
    """Chamfered transition lofting the slim column into the deeper head box."""
    loft = (
        cq.Workplane("XY", origin=(0.0, 0.0, LOFT_Z0))
        .rect(COLUMN_W, COLUMN_D)
        .workplane(offset=LOFT_Z1 - LOFT_Z0)
        .center(0.0, HEAD_Y)
        .rect(HEAD_W, HEAD_D)
        .loft()
    )
    return mesh_from_cadquery(loft, "head_transition", tolerance=0.0008, angular_tolerance=0.1)


def _hub_mesh():
    """Chrome rotor hub: mounting collar plus the flat elliptical face disc."""
    collar = cq.Workplane("XY").circle(COLLAR_R).extrude(COLLAR_LEN)
    disc = (
        cq.Workplane("XY", origin=(0.0, 0.0, COLLAR_LEN))
        .ellipse(HUB_RX, HUB_RY)
        .extrude(HUB_THICK)
    )
    return mesh_from_cadquery(
        collar.union(disc), "rotor_hub", tolerance=0.0006, angular_tolerance=0.08
    )


def _arm_mesh(name: str):
    """One chrome tube arm with a rounded cap, authored along its own +Z axis.

    Keeping the tube axis-aligned in mesh-local coordinates keeps its local
    AABB tight; the cone tilt and 120-degree phasing are applied through the
    visual origin instead.
    """
    tube = cq.Solid.makeCylinder(ARM_R, ARM_L, cq.Vector(0.0, 0.0, 0.0), cq.Vector(0.0, 0.0, 1.0))
    cap = cq.Solid.makeSphere(ARM_R, cq.Vector(0.0, 0.0, ARM_L), angleDegrees1=-90)
    solid = cq.Workplane(obj=tube).union(cq.Workplane(obj=cap))
    return mesh_from_cadquery(solid, name, tolerance=0.0006, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tripod_turnstile")

    steel = model.material("brushed_stainless", color=(0.615, 0.625, 0.635, 1.0))
    chrome = model.material("polished_chrome", color=(0.84, 0.86, 0.88, 1.0))
    granite = model.material("speckled_granite", color=(0.13, 0.13, 0.14, 1.0))
    black_plastic = model.material("black_plastic", color=(0.055, 0.055, 0.06, 1.0))
    red_led = model.material("red_indicator", color=(0.82, 0.16, 0.10, 1.0))
    lock_steel = model.material("lock_steel", color=(0.42, 0.43, 0.45, 1.0))

    cabinet = model.part("cabinet")

    cabinet.visual(_column_mesh(), origin=Origin(), material=steel, name="column_shell")
    cabinet.visual(_transition_mesh(), origin=Origin(), material=steel, name="head_transition")
    cabinet.visual(
        Box((HEAD_W, HEAD_D, HEAD_Z1 - HEAD_Z0)),
        origin=Origin(xyz=(0.0, HEAD_Y, (HEAD_Z0 + HEAD_Z1) / 2.0)),
        material=steel,
        name="head_housing",
    )
    cabinet.visual(
        Box(GRANITE_SIZE),
        origin=Origin(xyz=GRANITE_CENTER),
        material=granite,
        name="granite_top",
    )
    # Card reader / indicator module on the left side of the head box.
    cabinet.visual(
        Box((0.018, 0.085, 0.055)),
        origin=Origin(xyz=(-HEAD_W / 2.0 - 0.007, 0.12, 0.94)),
        material=black_plastic,
        name="card_reader",
    )
    cabinet.visual(
        Box((0.006, 0.050, 0.010)),
        origin=Origin(xyz=(-HEAD_W / 2.0 - 0.0145, 0.12, 0.955)),
        material=red_led,
        name="reader_indicator",
    )
    # Small circular access-panel lock on the column front face.
    cabinet.visual(
        Cylinder(radius=0.009, length=0.012),
        origin=Origin(xyz=(0.0, COLUMN_FRONT_Y - 0.0005, 0.64), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=lock_steel,
        name="panel_lock",
    )
    # Chrome bearing boss protruding perpendicular from the chamfer face.
    boss_center = (
        ANCHOR[0],
        ANCHOR[1] + 0.015 * _C,
        ANCHOR[2] - 0.015 * _C,
    )
    cabinet.visual(
        Cylinder(radius=BOSS_R, length=BOSS_L),
        origin=Origin(xyz=boss_center, rpy=ROT_RPY),
        material=chrome,
        name="rotor_boss",
    )

    rotor = model.part("tripod_rotor")
    rotor.visual(_hub_mesh(), origin=Origin(), material=chrome, name="hub")
    for index, phase in enumerate(ARM_PHASES_DEG):
        # rpy = (0, pitch, yaw) maps the tube's +Z onto the 45-degree cone
        # direction (cos(45)cos(phase), cos(45)sin(phase), sin(45)).
        rotor.visual(
            _arm_mesh(f"rotor_arm_{index}"),
            origin=Origin(
                xyz=(0.0, 0.0, ARM_START_Z),
                rpy=(0.0, math.pi / 2.0 - CONE, math.radians(phase)),
            ),
            material=chrome,
            name=f"arm_{index}",
        )

    model.articulation(
        "rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=rotor,
        origin=Origin(xyz=JOINT_XYZ, rpy=ROT_RPY),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    rotor = object_model.get_part("tripod_rotor")
    spin = object_model.get_articulation("rotor_spin")
    hub_vis = rotor.get_visual("hub")
    boss_vis = cabinet.get_visual("rotor_boss")

    ctx.check(
        "rotor joint is a continuous revolute about the hub axis",
        spin.articulation_type == ArticulationType.CONTINUOUS and spin.axis == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.fail_if_articulation_origin_far_from_geometry()

    # Hub axis tilts ~45 deg downward-forward and sits at waist height.
    rotor_pos = ctx.part_world_position(rotor)
    ctx.check(
        "rotor hub sits at waist height on the chamfer face",
        rotor_pos is not None and 0.72 <= rotor_pos[2] <= 0.88 and rotor_pos[1] > 0.12,
        details=f"rotor_pos={rotor_pos}",
    )

    # Rotor collar is seated against the chrome bearing boss, not floating.
    # The seat plane is tilted 45 degrees, so axis-aligned bounds of the two
    # coaxial elements intersect even though the solids only share the flush
    # contact face; scope that intentional seating and prove it with exact
    # contact + centering checks below.
    ctx.allow_overlap(
        rotor,
        cabinet,
        elem_a=hub_vis,
        elem_b=boss_vis,
        reason=(
            "rotor collar seats flush on the bearing boss face; the interface "
            "plane is tilted 45 deg so AABBs overlap without real penetration"
        ),
    )
    ctx.expect_contact(rotor, cabinet, elem_a=hub_vis, elem_b=boss_vis, contact_tol=5e-4)
    ctx.expect_overlap(
        rotor,
        cabinet,
        axes="xy",
        elem_a=hub_vis,
        elem_b=boss_vis,
        min_overlap=0.04,
        name="hub collar centered on bearing boss",
    )

    # All three 0.5 m chrome arms exist and have full length.
    for index in range(3):
        aabb = ctx.part_element_world_aabb(rotor, elem=f"arm_{index}")
        ok = False
        diag = 0.0
        if aabb is not None:
            (x0, y0, z0), (x1, y1, z1) = aabb
            diag = math.dist((x0, y0, z0), (x1, y1, z1))
            ok = diag >= 0.45
        ctx.check(
            f"arm_{index} is a full-length tube",
            ok,
            details=f"aabb_diag={diag:.3f}",
        )

    # World AABBs of tilted members are conservative (re-boxed per transform
    # stage), but their centers transform exactly. Verify the tripod kinematic
    # layout by comparing each arm's measured element-box center against the
    # cone-tripod prediction at two joint poses.
    cb = math.cos(CONE)
    sb = math.sin(CONE)

    def predicted_arm_center(q: float, phase_deg: float) -> tuple[float, float, float]:
        ph = math.radians(phase_deg) + q
        # local arm direction on the 45-degree cone, mid-tube point at 0.25 m
        ux, uy, uz = cb * math.cos(ph), cb * math.sin(ph), sb
        lx = 0.25 * ux
        ly = 0.25 * uy
        lz = ARM_START_Z + 0.25 * uz
        # rotor local -> world: x_l -> +X, y_l -> (0,-c,-c), z_l -> (0,c,-c)
        wx = JOINT_XYZ[0] + lx
        wy = JOINT_XYZ[1] - _C * ly + _C * lz
        wz = JOINT_XYZ[2] - _C * ly - _C * lz
        return (wx, wy, wz)

    def measured_arm_center(index: int) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(rotor, elem=f"arm_{index}")
        if aabb is None:
            return None
        (x0, y0, z0), (x1, y1, z1) = aabb
        return ((x0 + x1) / 2.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0)

    def check_arm_layout(q: float, label: str) -> None:
        for index, phase in enumerate(ARM_PHASES_DEG):
            measured = measured_arm_center(index)
            predicted = predicted_arm_center(q, phase)
            ok = measured is not None and math.dist(measured, predicted) <= 0.012
            ctx.check(
                f"arm_{index} follows the tilted 120-degree tripod layout {label}",
                ok,
                details=f"measured={measured}, predicted={predicted}",
            )

    # Home pose: arm_0 horizontal at waist height pointing at the user
    # (blocking the lane), arms 1/2 splayed down-right / down-left.
    check_arm_layout(0.0, "at q=0")
    c0 = measured_arm_center(0)
    c1 = measured_arm_center(1)
    c2 = measured_arm_center(2)
    ctx.check(
        "blocking arm projects horizontally toward the user at q=0",
        c0 is not None
        and c0[1] >= 0.42
        and abs(c0[0]) <= 0.02
        and abs(c0[2] - JOINT_XYZ[2] + ARM_START_Z * _C) <= 0.02,
        details=f"arm0_center={c0}",
    )
    ctx.check(
        "other two arms splay down-right and down-left at q=0",
        c1 is not None
        and c2 is not None
        and c1[0] >= 0.12
        and c2[0] <= -0.12
        and c1[2] <= 0.62
        and c2[2] <= 0.62,
        details=f"arm1_center={c1}, arm2_center={c2}",
    )

    # Sweep clearance, derived from the same cone-tripod kinematics that the
    # measured centers just confirmed: with the axis 45 deg down-forward and
    # arms on a 45 deg cone, every arm's world y-direction is 0.5*(1-sin(ph))
    # >= 0, so the arm set can never swing backward into the cabinet, and the
    # lowest reachable tip stays above the floor.
    start_y = JOINT_XYZ[1] + ARM_START_Z * _C
    start_z = JOINT_XYZ[2] - ARM_START_Z * _C
    min_y = start_y - ARM_R  # worst case over all q: arm pointing straight down
    min_tip_z = start_z - (ARM_L + ARM_R)
    ctx.check(
        "arms clear the cabinet front face and the floor at every rotation",
        min_y >= COLUMN_FRONT_Y + 0.04 and min_tip_z >= 0.05,
        details=f"min_arm_y={min_y:.3f} vs column_front={COLUMN_FRONT_Y}, min_tip_z={min_tip_z:.3f}",
    )

    # Half-index transition pose: the rotor has actually indexed; the blocking
    # arm has swept down-right out of the lane and another arm hangs
    # near-vertical between the hub and the floor.
    with ctx.pose({spin: math.pi / 3.0}):
        check_arm_layout(math.pi / 3.0, "at q=60deg")
        p0 = measured_arm_center(0)
        p1 = measured_arm_center(1)
        ctx.check(
            "at 60 deg no arm blocks the lane and one arm hangs low",
            p0 is not None
            and p1 is not None
            and p0[1] <= 0.40
            and p1[2] <= 0.55
            and abs(p1[1] - start_y) <= 0.02,
            details=f"posed_arm0_center={p0}, posed_arm1_center={p1}",
        )

    # Cabinet form: granite slab caps the head box with a slight overhang,
    # head box is deeper than the slim column, reader module proud on the left
    # side near the top, and the round lock proud on the column front face.
    granite = ctx.part_element_world_aabb(cabinet, elem="granite_top")
    head = ctx.part_element_world_aabb(cabinet, elem="head_housing")
    column = ctx.part_element_world_aabb(cabinet, elem="column_shell")
    reader = ctx.part_element_world_aabb(cabinet, elem="card_reader")
    lock = ctx.part_element_world_aabb(cabinet, elem="panel_lock")
    ctx.check(
        "granite top plate caps the unit at about 1.0 m",
        granite is not None and 0.97 <= granite[1][2] <= 1.03,
        details=f"granite_aabb={granite}",
    )
    ctx.check(
        "granite plate overhangs the steel head housing",
        granite is not None
        and head is not None
        and granite[0][0] <= head[0][0] - 0.005
        and granite[1][1] >= head[1][1] + 0.005,
        details=f"granite={granite}, head={head}",
    )
    ctx.check(
        "head box is deeper than the slim lower column",
        head is not None and column is not None and head[1][1] >= column[1][1] + 0.08,
        details=f"head_ymax={head[1][1] if head else None}, column_ymax={column[1][1] if column else None}",
    )
    ctx.check(
        "card reader module sits proud on the left side near the top",
        reader is not None
        and head is not None
        and reader[0][0] <= head[0][0] - 0.004
        and reader[0][2] >= 0.90,
        details=f"reader_aabb={reader}",
    )
    ctx.check(
        "circular panel lock sits proud on the column front face",
        lock is not None and lock[1][1] >= 0.0915 and 0.60 <= (lock[0][2] + lock[1][2]) / 2.0 <= 0.68,
        details=f"lock_aabb={lock}",
    )

    return ctx.report()


object_model = build_object_model()
