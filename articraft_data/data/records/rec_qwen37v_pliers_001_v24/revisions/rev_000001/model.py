from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Slip-joint pliers — variant of compact flush-cut wire snips.
#
# Two forged halves cross at a slip-joint pivot: one half has an elongated
# slot (two-position slip), the other has a round bore.  A bolt shank on the
# slip carriage slides between the two positions; circular rivet caps sit on
# both sides.  Each half is one rigid link: flat gripping jaw -> slim steel
# neck -> curved over-molded handle.
#
# Geometry is authored per half in a "closed-design" local frame.  The rest
# pose splays each half by HALF_OPEN via visual/joint-frame yaw; closing
# travel brings the jaws together while the handles scissor in opposition.
# ---------------------------------------------------------------------------

HALF_OPEN = math.radians(12.5)
CLOSE_TRAVEL = 2.0 * HALF_OPEN  # ~25 degrees
SLIP_TRAVEL = 0.008  # 8 mm between the two pivot positions

PLATE_T = 0.0035

ZL0, ZL1 = 0.0035, 0.0035 + PLATE_T  # lower plate
ZU0, ZU1 = ZL1, ZL1 + PLATE_T  # upper plate

BLADE_Z0, BLADE_Z1 = ZL0, ZU1
BLADE_X_MIN = 0.0085

BOSS_R = 0.0085
SLOT_WIDTH = 0.005
SLOT_LENGTH = SLIP_TRAVEL + SLOT_WIDTH  # stadium total length
HOLE_R = SLOT_WIDTH / 2

CAP_R = 0.006
CAP_T = 0.0015

EDGE_LAND = 0.0004

GRIP_H = 0.0105
GRIP_LZ0 = 0.0
GRIP_LZ1 = GRIP_LZ0 + GRIP_H
GRIP_UZ0 = GRIP_LZ0 + (ZU0 - ZL0)
GRIP_UZ1 = GRIP_UZ0 + GRIP_H
INLAY_T = 0.0007
INLAY_EMBED = 0.0002

# -- Flat gripping jaw profile (pliers, lower half +y side) ----------------
JAW_PTS = [
    (0.0020, 0.0105),
    (0.0080, 0.0110),
    (0.0160, 0.0105),
    (0.0230, 0.0088),
    (0.0280, 0.0062),
    (0.0300, 0.0040),
    (0.0305, 0.0018),
    (0.0300, 0.0010),
    (0.0270, 0.0022),
    (0.0200, 0.0040),
    (0.0120, 0.0055),
    (0.0020, 0.0060),
]

TANG_PTS = [
    (0.0020, -0.0032),
    (-0.0100, -0.0044),
    (-0.0200, -0.0052),
    (-0.0300, -0.0062),
    (-0.0300, -0.0112),
    (-0.0200, -0.0096),
    (-0.0100, -0.0082),
    (0.0008, -0.0072),
]

GRIP_PTS = [
    (-0.0240, -0.0038),
    (-0.0400, -0.0048),
    (-0.0580, -0.0060),
    (-0.0740, -0.0072),
    (-0.0890, -0.0080),
    (-0.0955, -0.0093),
    (-0.0915, -0.0128),
    (-0.0780, -0.0148),
    (-0.0620, -0.0152),
    (-0.0460, -0.0138),
    (-0.0300, -0.0112),
    (-0.0235, -0.0095),
]

INLAY_PTS = [
    (-0.0305, -0.0068),
    (-0.0480, -0.0082),
    (-0.0660, -0.0095),
    (-0.0790, -0.0102),
    (-0.0860, -0.0106),
    (-0.0810, -0.0116),
    (-0.0670, -0.0122),
    (-0.0510, -0.0114),
    (-0.0370, -0.0100),
    (-0.0315, -0.0090),
]


# ===== helpers =============================================================

def _mirror(pts: list[tuple[float, float]], s: float) -> list[tuple[float, float]]:
    return [(x, s * y) for (x, y) in pts]


def _poly_prism(pts: list[tuple[float, float]], z0: float, z1: float) -> cq.Workplane:
    return (
        cq.Workplane("XY", origin=(0.0, 0.0, z0))
        .polyline(pts)
        .close()
        .extrude(z1 - z0)
    )


def _spline_wire(pts: list[tuple[float, float]], z: float, inset: float = 0.0) -> cq.Wire:
    edge = cq.Edge.makeSpline([cq.Vector(x, y, z) for (x, y) in pts], periodic=True)
    wire = cq.Wire.assembleEdges([edge])
    if inset:
        wire = wire.offset2D(-inset)[0]
    return wire


def _soft_prism(
    pts: list[tuple[float, float]],
    z0: float,
    z1: float,
    cap: float = 0.0020,
    inset: float = 0.0018,
) -> cq.Workplane:
    mid = cq.Solid.extrudeLinear(
        cq.Face.makeFromWires(_spline_wire(pts, z0 + cap)),
        cq.Vector(0.0, 0.0, (z1 - z0) - 2.0 * cap),
    )
    top = cq.Solid.makeLoft([_spline_wire(pts, z1 - cap), _spline_wire(pts, z1, inset=inset)])
    bot = cq.Solid.makeLoft([_spline_wire(pts, z0, inset=inset), _spline_wire(pts, z0 + cap)])
    return cq.Workplane(obj=mid.fuse(top).fuse(bot))


def _spline_prism(pts: list[tuple[float, float]], z0: float, z1: float) -> cq.Workplane:
    face = cq.Face.makeFromWires(_spline_wire(pts, z0))
    return cq.Workplane(obj=cq.Solid.extrudeLinear(face, cq.Vector(0.0, 0.0, z1 - z0)))


# ===== jaw (flat gripping, no bevel) =======================================

def _blade_clip_box() -> cq.Workplane:
    return cq.Workplane("XY", origin=(BLADE_X_MIN + 0.0160, 0.0, 0.0070)).box(
        0.0520, 0.0600, 0.0200
    )


def _half_jaw(s: float, own_z0: float, own_z1: float) -> cq.Workplane:
    prof = _mirror(JAW_PTS, s)
    full = _poly_prism(prof, BLADE_Z0, BLADE_Z1).intersect(_blade_clip_box())
    rear = _poly_prism(prof, own_z0, own_z1)
    return full.union(rear)


# ===== bosses ==============================================================

def _stadium_slot(cx: float, z0: float, z1: float) -> cq.Workplane:
    """Elongated stadium-shaped through-hole for the slip joint."""
    r = SLOT_WIDTH / 2
    half_straight = (SLOT_LENGTH - SLOT_WIDTH) / 2
    pts = [
        (cx - half_straight, -r),
        (cx + half_straight, -r),
        (cx + half_straight, r),
        (cx - half_straight, r),
    ]
    body = _poly_prism(pts, z0 - 0.001, z1 + 0.001)
    left = (
        cq.Workplane("XY", origin=(cx - half_straight, 0.0, z0 - 0.001))
        .circle(r)
        .extrude((z1 - z0) + 0.002)
    )
    right = (
        cq.Workplane("XY", origin=(cx + half_straight, 0.0, z0 - 0.001))
        .circle(r)
        .extrude((z1 - z0) + 0.002)
    )
    return body.union(left).union(right)


def _lower_boss_slotted(own_z0: float, own_z1: float) -> cq.Workplane:
    boss = cq.Workplane("XY", origin=(0.0, 0.0, own_z0)).circle(BOSS_R).extrude(own_z1 - own_z0)
    # Slot runs along -X from the forward bore position to the rear position.
    slot = _stadium_slot(-SLIP_TRAVEL / 2, own_z0, own_z1)
    return boss.cut(slot)


def _upper_boss_holed(own_z0: float, own_z1: float) -> cq.Workplane:
    boss = cq.Workplane("XY", origin=(0.0, 0.0, own_z0)).circle(BOSS_R).extrude(own_z1 - own_z0)
    hole = (
        cq.Workplane("XY", origin=(0.0, 0.0, own_z0 - 0.001))
        .circle(HOLE_R)
        .extrude((own_z1 - own_z0) + 0.002)
    )
    return boss.cut(hole)


# ===== slip carriage (bolt + caps) =========================================

def _carriage_shank() -> cq.Workplane:
    r = HOLE_R - 0.0002
    return (
        cq.Workplane("XY", origin=(0.0, 0.0, ZL0 - 0.001))
        .circle(r)
        .extrude((ZU1 - ZL0) + 0.002)
    )


def _pivot_cap(z_base: float) -> cq.Workplane:
    cap = cq.Workplane("XY", origin=(0.0, 0.0, z_base)).circle(CAP_R).extrude(CAP_T)
    try:
        cap = cap.edges(">Z").fillet(0.0004)
        cap = cap.edges("<Z").fillet(0.0004)
    except Exception:
        pass
    return cap


# ===== build ===============================================================

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slip_joint_pliers")

    steel = model.material("brushed_steel", rgba=(0.72, 0.73, 0.75, 1.0))
    polished = model.material("polished_steel", rgba=(0.86, 0.87, 0.90, 1.0))
    orange = model.material("orange_grip", rgba=(0.97, 0.52, 0.07, 1.0))
    peach = model.material("peach_inlay", rgba=(1.0, 0.80, 0.64, 0.78))

    lower_pose = Origin(rpy=(0.0, 0.0, HALF_OPEN))

    # ----- lower half (base link) ------------------------------------------
    lower = model.part("lower_half")
    lower.visual(
        mesh_from_cadquery(_half_jaw(+1.0, ZL0, ZL1), "lower_jaw"),
        origin=lower_pose,
        material=steel,
        name="jaw",
    )
    lower.visual(
        mesh_from_cadquery(_lower_boss_slotted(ZL0, ZL1), "lower_boss"),
        origin=lower_pose,
        material=steel,
        name="pivot_boss",
    )
    lower.visual(
        mesh_from_cadquery(_poly_prism(_mirror(TANG_PTS, +1.0), ZL0, ZL1), "lower_tang"),
        origin=lower_pose,
        material=steel,
        name="neck_tang",
    )
    lower.visual(
        mesh_from_cadquery(
            _soft_prism(_mirror(GRIP_PTS, +1.0), GRIP_LZ0, GRIP_LZ1), "lower_grip"
        ),
        origin=lower_pose,
        material=orange,
        name="grip",
    )
    lower.visual(
        mesh_from_cadquery(
            _spline_prism(
                _mirror(INLAY_PTS, +1.0),
                GRIP_LZ1 - INLAY_EMBED,
                GRIP_LZ1 - INLAY_EMBED + INLAY_T,
            ),
            "lower_inlay",
        ),
        origin=lower_pose,
        material=peach,
        name="grip_inlay",
    )

    # ----- slip carriage (bolt shank + two circular caps) ------------------
    carriage = model.part("slip_carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_shank(), "shank"),
        material=polished,
        name="bolt_shank",
    )
    carriage.visual(
        mesh_from_cadquery(_pivot_cap(ZL0 - CAP_T), "lower_cap"),
        material=polished,
        name="lower_pivot_cap",
    )
    carriage.visual(
        mesh_from_cadquery(_pivot_cap(ZU1), "upper_cap"),
        material=polished,
        name="upper_pivot_cap",
    )

    # ----- upper half (moving link) ----------------------------------------
    upper = model.part("upper_half")
    upper.visual(
        mesh_from_cadquery(_half_jaw(-1.0, ZU0, ZU1), "upper_jaw"),
        material=steel,
        name="jaw",
    )
    upper.visual(
        mesh_from_cadquery(_upper_boss_holed(ZU0, ZU1), "upper_boss"),
        material=steel,
        name="pivot_boss",
    )
    upper.visual(
        mesh_from_cadquery(_poly_prism(_mirror(TANG_PTS, -1.0), ZU0, ZU1), "upper_tang"),
        material=steel,
        name="neck_tang",
    )
    upper.visual(
        mesh_from_cadquery(
            _soft_prism(_mirror(GRIP_PTS, -1.0), GRIP_UZ0, GRIP_UZ1), "upper_grip"
        ),
        material=orange,
        name="grip",
    )
    upper.visual(
        mesh_from_cadquery(
            _spline_prism(
                _mirror(INLAY_PTS, -1.0),
                GRIP_UZ1 - INLAY_EMBED,
                GRIP_UZ1 - INLAY_EMBED + INLAY_T,
            ),
            "upper_inlay",
        ),
        material=peach,
        name="grip_inlay",
    )

    # ===== articulations ===================================================

    # 1. slip_joint: PRISMATIC — lower_half -> slip_carriage
    #    The carriage slides along the slot direction (tool length axis in
    #    the lower-half visual frame).  Positive q moves the pivot toward
    #    the handle end (wider jaw setting).
    model.articulation(
        "slip_joint",
        ArticulationType.PRISMATIC,
        parent=lower,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, HALF_OPEN)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0, velocity=0.15, lower=0.0, upper=SLIP_TRAVEL
        ),
    )

    # 2. pivot: REVOLUTE — slip_carriage -> upper_half
    #    The upper half rotates around the carriage bolt, closing/opening
    #    the jaws.  The origin rpy undoes both the inherited HALF_OPEN from
    #    the carriage and applies the rest-pose splay so that q = 0 is open
    #    and q = CLOSE_TRAVEL is closed.
    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, -2.0 * HALF_OPEN)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0, velocity=4.0, lower=0.0, upper=CLOSE_TRAVEL
        ),
    )

    return model


# ===== tests ===============================================================

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower = object_model.get_part("lower_half")
    carriage = object_model.get_part("slip_carriage")
    upper = object_model.get_part("upper_half")

    slip = object_model.get_articulation("slip_joint")
    pivot = object_model.get_articulation("pivot")

    # --- articulation types ------------------------------------------------
    ctx.check(
        "slip joint is prismatic",
        slip.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={slip.articulation_type}",
    )
    ctx.check(
        "pivot is revolute",
        pivot.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={pivot.articulation_type}",
    )

    # --- slip joint limits: two-position travel ~8 mm ----------------------
    slip_lim = slip.motion_limits
    ctx.check(
        "slip travel ~8 mm for two positions",
        slip_lim is not None
        and slip_lim.lower is not None
        and slip_lim.upper is not None
        and abs(slip_lim.lower) < 1e-9
        and 0.006 <= slip_lim.upper <= 0.010,
        details=f"limits={slip_lim}",
    )

    # --- pivot travel ~25 degrees ------------------------------------------
    piv_lim = pivot.motion_limits
    ctx.check(
        "pivot travel ~25 degrees",
        piv_lim is not None
        and piv_lim.lower is not None
        and piv_lim.upper is not None
        and abs(piv_lim.lower) < 1e-9
        and 0.38 <= piv_lim.upper <= 0.48,
        details=f"limits={piv_lim}",
    )

    # --- circular pivot caps on both sides ---------------------------------
    lcap = ctx.part_element_world_aabb(carriage, elem="lower_pivot_cap")
    ucap = ctx.part_element_world_aabb(carriage, elem="upper_pivot_cap")
    shank_aabb = ctx.part_element_world_aabb(carriage, elem="bolt_shank")
    ctx.check(
        "lower cap below bolt shank",
        lcap is not None
        and shank_aabb is not None
        and lcap[0][2] < shank_aabb[0][2] - 0.0003,
        details=f"lcap={lcap}, shank={shank_aabb}",
    )
    ctx.check(
        "upper cap above bolt shank",
        ucap is not None
        and shank_aabb is not None
        and ucap[1][2] > shank_aabb[1][2] + 0.0003,
        details=f"ucap={ucap}, shank={shank_aabb}",
    )

    # --- caps are circular (roughly equal XY extents) ----------------------
    for cap_name in ("lower_pivot_cap", "upper_pivot_cap"):
        cap_aabb = ctx.part_element_world_aabb(carriage, elem=cap_name)
        if cap_aabb is not None:
            dx = cap_aabb[1][0] - cap_aabb[0][0]
            dy = cap_aabb[1][1] - cap_aabb[0][1]
            ctx.check(
                f"{cap_name} is roughly circular",
                0.7 <= dx / max(dy, 1e-9) <= 1.3 and 0.008 <= dx <= 0.016,
                details=f"dx={dx:.5f}, dy={dy:.5f}",
            )

    # --- pivot boss overlap between halves ---------------------------------
    ctx.expect_overlap(
        lower,
        carriage,
        axes="xy",
        elem_a="pivot_boss",
        elem_b="bolt_shank",
        min_overlap=0.003,
        name="lower boss overlaps carriage shank",
    )

    # --- rest pose: jaws open at slip position 1 ---------------------------
    ctx.expect_gap(
        lower,
        upper,
        axis="y",
        positive_elem="jaw",
        negative_elem="jaw",
        min_gap=0.001,
        name="jaws open at rest (position 1)",
    )

    # --- handles splay apart at rest ---------------------------------------
    ctx.expect_gap(
        upper,
        lower,
        axis="y",
        positive_elem="grip",
        negative_elem="grip",
        min_gap=0.008,
        name="handles splay apart at rest",
    )

    # --- closing the pivot: jaws approach, handles scissor -----------------
    open_jaw = ctx.part_element_world_aabb(upper, elem="jaw")
    open_grip = ctx.part_element_world_aabb(upper, elem="grip")
    with ctx.pose({pivot: CLOSE_TRAVEL}):
        closed_jaw = ctx.part_element_world_aabb(upper, elem="jaw")
        closed_grip = ctx.part_element_world_aabb(upper, elem="grip")

    ctx.check(
        "closing swings upper jaw toward lower jaw",
        open_jaw is not None
        and closed_jaw is not None
        and closed_jaw[1][1] > open_jaw[1][1] + 0.002,
        details=f"open={open_jaw}, closed={closed_jaw}",
    )
    ctx.check(
        "handles scissor opposite to the jaws",
        open_grip is not None
        and closed_grip is not None
        and closed_grip[0][1] < open_grip[0][1] - 0.002,
        details=f"open={open_grip}, closed={closed_grip}",
    )

    # --- slip position 2 changes jaw gap -----------------------------------
    jaw_gap_pos1 = None
    jaw_gap_pos2 = None
    if open_jaw is not None:
        lower_jaw = ctx.part_element_world_aabb(lower, elem="jaw")
        if lower_jaw is not None:
            jaw_gap_pos1 = lower_jaw[0][1] - open_jaw[1][1]

    with ctx.pose({slip: SLIP_TRAVEL}):
        upper_jaw_pos2 = ctx.part_element_world_aabb(upper, elem="jaw")
        if upper_jaw_pos2 is not None and lower_jaw is not None:
            jaw_gap_pos2 = lower_jaw[0][1] - upper_jaw_pos2[1][1]

    ctx.check(
        "jaw gap differs between slip positions",
        jaw_gap_pos1 is not None
        and jaw_gap_pos2 is not None
        and abs(jaw_gap_pos2 - jaw_gap_pos1) > 0.0003,
        details=f"gap_pos1={jaw_gap_pos1}, gap_pos2={jaw_gap_pos2}",
    )

    # --- overall proportions -----------------------------------------------
    la = ctx.part_world_aabb(lower)
    ua = ctx.part_world_aabb(upper)
    if la is not None and ua is not None:
        length = max(la[1][0], ua[1][0]) - min(la[0][0], ua[0][0])
        width = max(la[1][1], ua[1][1]) - min(la[0][1], ua[0][1])
        height = max(la[1][2], ua[1][2]) - min(la[0][2], ua[0][2])
        ctx.check(
            "overall length ~0.13 m",
            0.10 <= length <= 0.16,
            details=f"length={length:.4f}",
        )
        ctx.check(
            "splayed handle width ~0.05–0.08 m",
            0.04 <= width <= 0.09,
            details=f"width={width:.4f}",
        )
        ctx.check(
            "flat tool thickness ~0.015 m",
            0.010 <= height <= 0.020,
            details=f"height={height:.4f}",
        )
    else:
        ctx.fail("overall proportions", "missing part AABBs")

    # --- inlay within grip footprint --------------------------------------
    for part in (lower, upper):
        grip = ctx.part_element_world_aabb(part, elem="grip")
        inlay = ctx.part_element_world_aabb(part, elem="grip_inlay")
        ctx.check(
            f"{part.name} inlay within grip footprint",
            grip is not None
            and inlay is not None
            and inlay[0][0] >= grip[0][0] - 1e-4
            and inlay[1][0] <= grip[1][0] + 1e-4
            and inlay[0][1] >= grip[0][1] - 1e-4
            and inlay[1][1] <= grip[1][1] + 1e-4,
            details=f"grip={grip}, inlay={inlay}",
        )

    return ctx.report()


object_model = build_object_model()
