from __future__ import annotations

"""One-hand spring-return C-loop single-hole punch.

A continuous bent spring-steel C-loop frame carries the lower jaw with the
perforated die block; a separate punch lever pivots on a front hinge pin at
the working head and carries the cylindrical punch pin.  The two grips splay
rearward from the front pivot, and the C-loop bridge at the rear provides
elastic return.

Squeezing the lever grip down toward the fixed lower grip rotates the lever
about the front pivot (revolute joint "punch_stroke", axis -Y) and drives the
punch pin down through the paper slot into the die bore.  The punch pin sits
between the pivot and the grips (class-2 lever), so both the pin and the grip
travel downward on positive stroke.

Orientation: in-use pose.  Squeeze plane = XZ, hinge axis = Y, the working
head (punch + die) is near +X, grips extend toward -X, and the C-loop bridge
bows outward at the -X rear.
"""

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ──────────────────────────────────────────────────────────── dimensions
ARM_T = 0.005             # stamped-steel arm plate thickness (Y)
FRAME_Y = 0.004           # fixed-arm layer centre (+Y side)
LEVER_Y = -0.004          # moving-arm layer centre (-Y side)

LOWER_W = 0.009           # lower arm bar width (Z, in squeeze plane)
LEVER_W = 0.012           # lever arm bar width (Z)

# Front pivot
PIVOT_X = 0.055
PIVOT_Z = 0.032
BOSS_R = 0.006            # pivot boss disc radius
SHAFT_R = 0.0035          # pivot shaft radius
BORE_R = 0.003            # lever bore clearance for shaft

# Punch pin / die bore
PIN_X = 0.032             # punch pin / die bore X (behind pivot → class-2 lever)
PIN_R = 0.003
PIN_LEN = 0.011
DIE_BORE_R = 0.0035

# Die block on lower arm
DIE_X0, DIE_X1 = 0.024, 0.046
DIE_Z0, DIE_Z1 = 0.010, 0.020   # die top = paper table
DIE_Y_HALF = 0.008

# Paper slot (rest pose)
PAPER_SLOT = 0.0025
PIN_Z_BOTTOM_REST = DIE_Z1 + PAPER_SLOT        # 0.0225
PIN_Z_CTR_WORLD = PIN_Z_BOTTOM_REST + PIN_LEN / 2  # 0.028
PIN_LOCAL_X = PIN_X - PIVOT_X                  # -0.023
PIN_LOCAL_Z = PIN_Z_CTR_WORLD - PIVOT_Z        # -0.004

# Arm paths (x, z)
LOWER_FRONT = (0.058, 0.026)
LOWER_REAR = (-0.055, 0.005)
LEVER_FRONT = (0.004, 0.002)        # lever-local
LEVER_REAR = (-0.110, 0.016)        # lever-local

# C-loop bridge spline (world, Y = FRAME_Y)
BRIDGE_Y = FRAME_Y
BRIDGE_R = 0.0035
BRIDGE_PTS = [
    (LOWER_REAR[0], BRIDGE_Y, LOWER_REAR[1]),   # lower arm rear junction
    (-0.072, BRIDGE_Y, 0.010),                    # outward bow
    (-0.078, BRIDGE_Y, 0.028),                    # middle (outermost)
    (-0.072, BRIDGE_Y, 0.044),                    # inward bow
    (-0.055, BRIDGE_Y, 0.048),                    # upper rear junction
    (-0.035, BRIDGE_Y, 0.046),                    # upper stub front terminus
]

GRIP_FRAC = 0.60         # grip starts at 60 % of arm length from front
STROKE_MAX = 0.18         # rad

# ──────────────────────────────────────────────────────────── CadQuery helpers

def _bar(p0, p1, width, y_center):
    """Flat bar in the XZ squeeze plane between two (x, z) points."""
    dx, dz = p1[0] - p0[0], p1[1] - p0[1]
    length = math.hypot(dx, dz)
    theta = -math.degrees(math.atan2(dz, dx))
    cx, cz = (p0[0] + p1[0]) / 2.0, (p0[1] + p1[1]) / 2.0
    return (
        cq.Workplane("XY")
        .box(length, ARM_T, width)
        .rotate((0, 0, 0), (0, 1, 0), theta)
        .translate((cx, y_center, cz))
    )


def _boss(y_center):
    """Pivot boss disc at the front pivot, axis along Y."""
    return (
        cq.Workplane("XZ")
        .circle(BOSS_R)
        .extrude(ARM_T * 0.5, both=True)
        .translate((PIVOT_X, y_center, PIVOT_Z))
    )


def _frame_lower_arm():
    """Frame lower arm bar + front pivot boss (world coordinates)."""
    arm = _bar(LOWER_FRONT, LOWER_REAR, LOWER_W, FRAME_Y)
    return arm.union(_boss(FRAME_Y))


def _die_block_solid():
    """Perforated die block: paper table with punch bore and chip windows."""
    blk = (
        cq.Workplane("XY")
        .box(DIE_X1 - DIE_X0, 2 * DIE_Y_HALF, DIE_Z1 - DIE_Z0)
        .translate(((DIE_X0 + DIE_X1) / 2.0, 0.0, (DIE_Z0 + DIE_Z1) / 2.0))
    )
    # vertical punch bore aligned with the pin, through the block
    bore = (
        cq.Workplane("XY")
        .circle(DIE_BORE_R)
        .extrude(0.030)
        .translate((PIN_X, LEVER_Y, DIE_Z0 - 0.005))
    )
    blk = blk.cut(bore)
    # chip-drain confetti holes through the block (Y axis)
    for hz in (0.013, 0.017):
        for xi in range(4):
            hx = 0.028 + 0.005 * xi
            hole = (
                cq.Workplane("XZ")
                .circle(0.001)
                .extrude(0.030, both=True)
                .translate((hx, 0.0, hz))
            )
            blk = blk.cut(hole)
    return blk


def _lever_arm_solid():
    """Lever arm bar with hinge knuckle at origin and bore for shaft (lever-local)."""
    arm = _bar(LEVER_FRONT, LEVER_REAR, LEVER_W, LEVER_Y)
    knuckle = (
        cq.Workplane("XZ")
        .circle(BOSS_R - 0.001)
        .extrude(ARM_T * 0.5, both=True)
        .translate((0.0, LEVER_Y, 0.0))
    )
    solid = arm.union(knuckle)
    bore = (
        cq.Workplane("XZ")
        .circle(BORE_R)
        .extrude(0.020, both=True)
    )
    return solid.cut(bore)


def _grip_solid(p_start, p_end, y_center, arm_width):
    """Knurled grip sleeve over the rear portion of an arm bar."""
    dx, dz = p_end[0] - p_start[0], p_end[1] - p_start[1]
    theta = -math.degrees(math.atan2(dz, dx))
    cx, cz = (p_start[0] + p_end[0]) / 2.0, (p_start[1] + p_end[1]) / 2.0
    seg = math.hypot(dx, dz)
    return (
        cq.Workplane("XY")
        .box(seg, ARM_T + 0.0014, arm_width + 0.002)
        .rotate((0, 0, 0), (0, 1, 0), theta)
        .translate((cx, y_center, cz))
    )


def _grip_start(p_front, p_rear):
    """Compute grip start point at GRIP_FRAC along the arm."""
    return (
        p_front[0] + GRIP_FRAC * (p_rear[0] - p_front[0]),
        p_front[1] + GRIP_FRAC * (p_rear[1] - p_front[1]),
    )


# ──────────────────────────────────────────────────────────── build

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cloop_single_hole_punch")

    chrome = model.material("polished_chrome", rgba=(0.80, 0.82, 0.86, 1.0))
    steel = model.material("die_steel", rgba=(0.60, 0.62, 0.66, 1.0))
    knurl = model.material("knurled_grip", rgba=(0.42, 0.44, 0.47, 1.0))
    shaft_mat = model.material("pivot_steel", rgba=(0.86, 0.88, 0.91, 1.0))

    # ── frame (root) ─────────────────────────────────────────────────
    frame = model.part("frame")

    # lower arm + front pivot boss
    frame.visual(
        mesh_from_cadquery(_frame_lower_arm(), "frame_arm"),
        name="frame_arm",
        material=chrome,
    )

    # die block with bore
    frame.visual(
        mesh_from_cadquery(_die_block_solid(), "die_block"),
        name="die_block",
        material=steel,
    )

    # C-loop bridge: continuous bent spring-steel tube
    frame.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                BRIDGE_PTS,
                radius=BRIDGE_R,
                samples_per_segment=16,
                radial_segments=20,
                cap_ends=True,
            ),
            "c_loop_bridge",
        ),
        name="c_loop_bridge",
        material=chrome,
    )

    # lower knurled grip
    lg0 = _grip_start(LOWER_FRONT, LOWER_REAR)
    frame.visual(
        mesh_from_cadquery(
            _grip_solid(lg0, LOWER_REAR, FRAME_Y, LOWER_W),
            "frame_grip",
        ),
        name="frame_grip",
        material=knurl,
    )

    # pivot shaft through both arm layers
    frame.visual(
        Cylinder(radius=SHAFT_R, length=0.020),
        origin=Origin(xyz=(PIVOT_X, 0.0, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        name="pivot_shaft",
        material=shaft_mat,
    )
    for i, y_cap in enumerate((-0.010, 0.010)):
        frame.visual(
            Cylinder(radius=0.0055, length=0.003),
            origin=Origin(xyz=(PIVOT_X, y_cap, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            name=f"pivot_cap_{i}",
            material=shaft_mat,
        )

    # ── punch lever ──────────────────────────────────────────────────
    lever = model.part("punch_lever")

    lever.visual(
        mesh_from_cadquery(_lever_arm_solid(), "lever_arm"),
        name="lever_arm",
        material=chrome,
    )

    # upper knurled grip (lever-local coordinates)
    ug0 = _grip_start(LEVER_FRONT, LEVER_REAR)
    lever.visual(
        mesh_from_cadquery(
            _grip_solid(ug0, LEVER_REAR, LEVER_Y, LEVER_W),
            "lever_grip",
        ),
        name="lever_grip",
        material=knurl,
    )

    # punch pin hanging from the lever jaw, aligned with die bore
    lever.visual(
        Cylinder(radius=PIN_R, length=PIN_LEN),
        origin=Origin(xyz=(PIN_LOCAL_X, LEVER_Y, PIN_LOCAL_Z)),
        name="punch_pin",
        material=steel,
    )

    # ── articulation ─────────────────────────────────────────────────
    # axis = -Y so positive q drives both pin (at -X local) and grip
    # (further at -X local) downward (-Z world).
    model.articulation(
        "punch_stroke",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=lever,
        origin=Origin(xyz=(PIVOT_X, 0.0, PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=2.0, lower=0.0, upper=STROKE_MAX,
        ),
    )

    return model


# ──────────────────────────────────────────────────────────── tests

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    lever = object_model.get_part("punch_lever")
    stroke = object_model.get_articulation("punch_stroke")
    pin = lever.get_visual("punch_pin")
    die = frame.get_visual("die_block")
    lever_arm = lever.get_visual("lever_arm")
    bridge = frame.get_visual("c_loop_bridge")
    pivot_shaft = frame.get_visual("pivot_shaft")
    lever_grip = lever.get_visual("lever_grip")

    # ── structural delta: C-loop bridge behind front pivot ───────────
    bridge_aabb = ctx.part_element_world_aabb(frame, elem=bridge)
    assert bridge_aabb is not None
    ctx.check(
        "c_loop_bridge_extends_rear_of_front_pivot",
        bridge_aabb[0][0] < PIVOT_X - 0.020,
        f"bridge min_x={bridge_aabb[0][0]:.4f} not behind pivot x={PIVOT_X}",
    )
    bridge_z_span = bridge_aabb[1][2] - bridge_aabb[0][2]
    ctx.check(
        "c_loop_bridge_spans_lower_to_upper_arm",
        bridge_z_span > 0.025,
        f"bridge Z span={bridge_z_span:.4f} too small for C-loop",
    )

    # ── front pivot captures lever (intentional interference fit) ────
    ctx.allow_overlap(
        frame, lever,
        elem_a=pivot_shaft, elem_b=lever_arm,
        reason="pivot shaft is press-fit through the lever hinge knuckle bore",
    )
    ctx.expect_contact(
        frame, lever,
        elem_a=pivot_shaft, elem_b=lever_arm,
        name="pivot_shaft_captures_lever",
    )

    # ── paper slot open at rest ──────────────────────────────────────
    ctx.expect_gap(
        lever, frame, axis="z",
        min_gap=0.001, max_gap=0.006,
        positive_elem=pin, negative_elem=die,
        name="paper_slot_open_at_rest",
    )

    # pin centered over die bore in X
    pin_rest = ctx.part_element_world_aabb(lever, elem=pin)
    assert pin_rest is not None
    pin_cx = (pin_rest[0][0] + pin_rest[1][0]) / 2.0
    ctx.check(
        "pin_over_die_bore",
        abs(pin_cx - PIN_X) < 0.002,
        f"pin x={pin_cx:.4f} vs bore x={PIN_X}",
    )

    # ── rest-pose grip position (reference for squeeze check) ────────
    grip_rest_aabb = ctx.part_element_world_aabb(lever, elem=lever_grip)
    assert grip_rest_aabb is not None

    # ── squeezed pose ────────────────────────────────────────────────
    with ctx.pose({stroke: STROKE_MAX}):
        pin_closed = ctx.part_element_world_aabb(lever, elem=pin)
        die_ab = ctx.part_element_world_aabb(frame, elem=die)
        assert pin_closed is not None and die_ab is not None
        die_top = die_ab[1][2]
        ctx.check(
            "pin_enters_die_when_squeezed",
            pin_closed[0][2] < die_top - 0.001,
            f"pin bottom {pin_closed[0][2]:.4f} not below die top {die_top:.4f}",
        )
        cx = (pin_closed[0][0] + pin_closed[1][0]) / 2.0
        ctx.check(
            "pin_stays_on_bore_axis",
            abs(cx - PIN_X) < 0.002,
            f"pin drifts to x={cx:.4f}",
        )

    grip_squeezed_aabb = ctx.part_element_world_aabb(lever, elem=lever_grip)
    # measure grip drop across poses (outside the with-block for the rest value)
    with ctx.pose({stroke: STROKE_MAX}):
        grip_sq = ctx.part_element_world_aabb(lever, elem=lever_grip)
    assert grip_sq is not None
    grip_drop = grip_rest_aabb[0][2] - grip_sq[0][2]
    ctx.check(
        "squeeze_drops_lever_grip",
        grip_drop > 0.005,
        f"lever grip drops only {grip_drop:.4f} m",
    )

    return ctx.report()


object_model = build_object_model()
