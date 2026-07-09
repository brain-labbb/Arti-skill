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
# Bent-nose slip-joint pliers.
#
# Two halves cross at a slip-joint pivot. The lower half has an elongated
# slot in its boss; a pivot pin slides along this slot (PRISMATIC). The upper
# half rotates about the pin (REVOLUTE). Jaws are bent at ~45 degrees near
# the tips. Circular rivet caps sit on both sides of the pivot.
#
# Geometry is authored per half in a "closed-design" local frame where the
# two halves are mirrored about the XZ plane. The rest pose splays each half
# by HALF_OPEN via visual/joint-frame yaw. Positive revolute q closes the
# jaws while handles scissor together.
# ---------------------------------------------------------------------------

HALF_OPEN = math.radians(12.5)
CLOSE_TRAVEL = 2.0 * HALF_OPEN  # ~25 degrees closing

PLATE_T = 0.0035

# Steel layer stack
ZL0, ZL1 = 0.0035, 0.0035 + PLATE_T  # lower: 0.0035..0.0070
ZU0, ZU1 = ZL1, ZL1 + PLATE_T  # upper: 0.0070..0.0105

BLADE_Z0, BLADE_Z1 = ZL0, ZU1
BLADE_X_MIN = 0.0085

BOSS_R = 0.0080
SLOT_LENGTH = 0.008  # slip-joint slot travel
SLOT_WIDTH = 0.005
PIN_R = 0.0024
CAP_R = 0.0050
CAP_T = 0.0012

EDGE_LAND = 0.0003

# Handle over-mold
GRIP_H = 0.0105
GRIP_LZ0 = 0.0
GRIP_LZ1 = GRIP_LZ0 + GRIP_H
GRIP_UZ0 = GRIP_LZ0 + (ZU0 - ZL0)
GRIP_UZ1 = GRIP_UZ0 + GRIP_H
INLAY_T = 0.0007
INLAY_EMBED = 0.0002

# Bent-nose jaw profile (lower half, +y side).
# Inner gripping face stays near the centerline (y~0.001) so the jaws can
# close face-to-face. The outer profile angles outward to show the bent-nose
# character -- the jaw body widens and the tip angles off-axis.
JAW_PTS = [
    (0.0085, 0.0008),   # inner face near boss
    (0.0150, 0.0008),   # inner straight
    (0.0210, 0.0010),   # inner approaching bend
    (0.0260, 0.0018),   # inner bend starts
    (0.0310, 0.0038),   # inner angled
    (0.0350, 0.0065),   # inner near tip
    (0.0375, 0.0088),   # tip point
    (0.0380, 0.0105),   # outer tip edge
    (0.0360, 0.0108),   # outer near tip
    (0.0320, 0.0095),   # outer angled
    (0.0270, 0.0072),   # outer bend
    (0.0220, 0.0055),   # outer straight
    (0.0160, 0.0042),   # outer base
    (0.0105, 0.0038),   # outer near boss
    (0.0085, 0.0036),   # close to start
]

# Slim steel neck/tang
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

# Curved over-molded handle
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

# Peach inlay strip
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


def _half_blade(s: float, own_z0: float, own_z1: float) -> cq.Workplane:
    """Bent-nose jaw: extrude the angled jaw profile at full forged head
    height. Both halves share the same Z extent in the jaw region."""
    prof = _mirror(JAW_PTS, s)
    return _poly_prism(prof, BLADE_Z0, BLADE_Z1)


def _lower_boss_with_slot() -> cq.Workplane:
    """Lower half boss with elongated slip-joint slot cut through it."""
    boss = cq.Workplane("XY", origin=(0.0, 0.0, ZL0)).circle(BOSS_R).extrude(
        ZL1 - ZL0
    )
    # Elongated slot along X axis (the slip-joint travel direction)
    slot_pts = [
        (-SLOT_LENGTH / 2.0, -SLOT_WIDTH / 2.0),
        (SLOT_LENGTH / 2.0, -SLOT_WIDTH / 2.0),
        (SLOT_LENGTH / 2.0, SLOT_WIDTH / 2.0),
        (-SLOT_LENGTH / 2.0, SLOT_WIDTH / 2.0),
    ]
    slot = (
        cq.Workplane("XY", origin=(0.0, 0.0, ZL0 - 0.001))
        .polyline(slot_pts)
        .close()
        .extrude((ZL1 - ZL0) + 0.002)
    )
    # Round the slot ends
    slot_end_l = (
        cq.Workplane("XY", origin=(-SLOT_LENGTH / 2.0, 0.0, ZL0 - 0.001))
        .circle(SLOT_WIDTH / 2.0)
        .extrude((ZL1 - ZL0) + 0.002)
    )
    slot_end_r = (
        cq.Workplane("XY", origin=(SLOT_LENGTH / 2.0, 0.0, ZL0 - 0.001))
        .circle(SLOT_WIDTH / 2.0)
        .extrude((ZL1 - ZL0) + 0.002)
    )
    slot_full = slot.union(slot_end_l).union(slot_end_r)
    return boss.cut(slot_full)


def _upper_boss_with_hole() -> cq.Workplane:
    """Upper half boss with a round hole for the pivot pin."""
    boss = cq.Workplane("XY", origin=(0.0, 0.0, ZU0)).circle(BOSS_R).extrude(
        ZU1 - ZU0
    )
    hole = (
        cq.Workplane("XY", origin=(0.0, 0.0, ZU0 - 0.001))
        .circle(PIN_R + 0.0003)
        .extrude((ZU1 - ZU0) + 0.002)
    )
    return boss.cut(hole)


def _pivot_pin() -> cq.Workplane:
    """Cylindrical pivot pin that slides in the slot."""
    pin = (
        cq.Workplane("XY", origin=(0.0, 0.0, ZL0 - 0.001))
        .circle(PIN_R)
        .extrude((ZU1 - ZL0) + 0.003)
    )
    return pin


def _rivet_cap() -> cq.Workplane:
    """Circular disk cap for the pivot, sits on outer face of each boss."""
    cap = (
        cq.Workplane("XY")
        .circle(CAP_R)
        .extrude(CAP_T)
    )
    try:
        cap = cap.edges(">Z").fillet(0.0004)
    except Exception:
        pass
    return cap


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bent_nose_slip_joint_pliers")

    steel = model.material("brushed_steel", rgba=(0.72, 0.73, 0.75, 1.0))
    polished = model.material("polished_steel", rgba=(0.86, 0.87, 0.90, 1.0))
    orange = model.material("orange_grip", rgba=(0.97, 0.52, 0.07, 1.0))
    peach = model.material("peach_inlay", rgba=(1.0, 0.80, 0.64, 0.78))
    dark_steel = model.material("dark_steel", rgba=(0.45, 0.46, 0.48, 1.0))

    # ---- lower half (base link, root): jaw at +y, handle at -y
    lower = model.part("lower_half")
    lower_pose = Origin(rpy=(0.0, 0.0, HALF_OPEN))

    lower.visual(
        mesh_from_cadquery(_half_blade(+1.0, ZL0, ZL1), "lower_blade"),
        origin=lower_pose,
        material=steel,
        name="jaw",
    )
    lower.visual(
        mesh_from_cadquery(_lower_boss_with_slot(), "lower_boss"),
        origin=lower_pose,
        material=steel,
        name="pivot_boss",
    )
    lower.visual(
        mesh_from_cadquery(_poly_prism(_mirror(TANG_PTS, +1.0), ZL0, ZL1), "lower_tang"),
        origin=lower_pose,
        material=steel,
        name="neck",
    )
    lower.visual(
        mesh_from_cadquery(_soft_prism(_mirror(GRIP_PTS, +1.0), GRIP_LZ0, GRIP_LZ1), "lower_grip"),
        origin=lower_pose,
        material=orange,
        name="grip",
    )
    lower.visual(
        mesh_from_cadquery(
            _spline_prism(
                _mirror(INLAY_PTS, +1.0), GRIP_LZ1 - INLAY_EMBED, GRIP_LZ1 - INLAY_EMBED + INLAY_T
            ),
            "lower_inlay",
        ),
        origin=lower_pose,
        material=peach,
        name="grip_inlay",
    )
    # Lower rivet cap (on the bottom face of the lower boss)
    lower_cap = _rivet_cap()
    lower.visual(
        mesh_from_cadquery(lower_cap, "lower_cap"),
        origin=Origin(
            xyz=(0.0, 0.0, ZL0 - CAP_T),
            rpy=(0.0, 0.0, HALF_OPEN),
        ),
        material=polished,
        name="rivet_cap",
    )

    # ---- pivot pin (slides in the slot, carries the upper half)
    pin_part = model.part("pivot_pin")
    pin_part.visual(
        mesh_from_cadquery(_pivot_pin(), "pin_body"),
        material=polished,
        name="pin",
    )

    # ---- upper half (moving link): mirrored, upper steel layer
    upper = model.part("upper_half")
    upper.visual(
        mesh_from_cadquery(_half_blade(-1.0, ZU0, ZU1), "upper_blade"),
        material=steel,
        name="jaw",
    )
    upper.visual(
        mesh_from_cadquery(_upper_boss_with_hole(), "upper_boss"),
        material=steel,
        name="pivot_boss",
    )
    upper.visual(
        mesh_from_cadquery(_poly_prism(_mirror(TANG_PTS, -1.0), ZU0, ZU1), "upper_tang"),
        material=steel,
        name="neck",
    )
    upper.visual(
        mesh_from_cadquery(_soft_prism(_mirror(GRIP_PTS, -1.0), GRIP_UZ0, GRIP_UZ1), "upper_grip"),
        material=orange,
        name="grip",
    )
    upper.visual(
        mesh_from_cadquery(
            _spline_prism(
                _mirror(INLAY_PTS, -1.0), GRIP_UZ1 - INLAY_EMBED, GRIP_UZ1 - INLAY_EMBED + INLAY_T
            ),
            "upper_inlay",
        ),
        material=peach,
        name="grip_inlay",
    )
    # Upper rivet cap (on the top face of the upper boss)
    upper.visual(
        mesh_from_cadquery(_rivet_cap(), "upper_cap"),
        origin=Origin(xyz=(0.0, 0.0, ZU1)),
        material=polished,
        name="rivet_cap",
    )

    # ---- PRISMATIC: slip joint slot (lower half -> pivot pin)
    # The pin slides along +X within the slot. At q=0 it sits at the rear
    # of the slot; positive q moves it forward.
    model.articulation(
        "slip_slot",
        ArticulationType.PRISMATIC,
        parent=lower,
        child=pin_part,
        origin=Origin(
            xyz=(-SLOT_LENGTH / 2.0, 0.0, 0.0),
            rpy=(0.0, 0.0, HALF_OPEN),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=50.0, velocity=0.1, lower=0.0, upper=SLOT_LENGTH,
        ),
    )

    # ---- REVOLUTE: pivot (pin -> upper half)
    # Positive q closes the jaws together while handles scissor apart.
    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=pin_part,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, -HALF_OPEN)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0, velocity=4.0, lower=0.0, upper=CLOSE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower = object_model.get_part("lower_half")
    upper = object_model.get_part("upper_half")
    pin = object_model.get_part("pivot_pin")
    slip = object_model.get_articulation("slip_slot")
    pivot = object_model.get_articulation("pivot")

    # --- slip-joint prismatic joint exists with correct travel
    slip_limits = slip.motion_limits
    ctx.check(
        "slip slot is prismatic with ~8mm travel",
        slip.articulation_type == ArticulationType.PRISMATIC
        and slip_limits is not None
        and slip_limits.lower is not None
        and slip_limits.upper is not None
        and abs(slip_limits.lower) < 1e-9
        and 0.006 <= slip_limits.upper <= 0.010,
        details=f"type={slip.articulation_type}, limits={slip_limits}",
    )

    # --- pivot revolute joint exists with ~25 deg travel
    pivot_limits = pivot.motion_limits
    ctx.check(
        "pivot is revolute with ~25 deg travel",
        pivot.articulation_type == ArticulationType.REVOLUTE
        and pivot_limits is not None
        and pivot_limits.lower is not None
        and pivot_limits.upper is not None
        and abs(pivot_limits.lower) < 1e-9
        and 0.38 <= pivot_limits.upper <= 0.48,
        details=f"type={pivot.articulation_type}, limits={pivot_limits}",
    )

    # --- pivot pin sits within the lower boss slot area
    ctx.expect_within(
        pin,
        lower,
        axes="xy",
        inner_elem="pin",
        outer_elem="pivot_boss",
        margin=0.002,
        name="pivot pin centered within lower boss",
    )

    # --- upper boss stacks on lower boss
    ctx.expect_overlap(
        lower,
        upper,
        axes="xy",
        elem_a="pivot_boss",
        elem_b="pivot_boss",
        min_overlap=0.010,
        name="pivot bosses stack coaxially",
    )
    ctx.expect_gap(
        upper,
        lower,
        axis="z",
        positive_elem="pivot_boss",
        negative_elem="pivot_boss",
        min_gap=-0.0001,
        max_gap=0.0008,
        name="upper boss sits on lower boss",
    )

    # --- rivet caps on both sides
    lower_cap_aabb = ctx.part_element_world_aabb(lower, elem="rivet_cap")
    upper_cap_aabb = ctx.part_element_world_aabb(upper, elem="rivet_cap")
    ctx.check(
        "lower rivet cap exists below the boss",
        lower_cap_aabb is not None and lower_cap_aabb[0][2] < ZL0 + 0.0002,
        details=f"cap={lower_cap_aabb}",
    )
    ctx.check(
        "upper rivet cap exists above the boss",
        upper_cap_aabb is not None and upper_cap_aabb[1][2] > ZU1 - 0.0002,
        details=f"cap={upper_cap_aabb}",
    )
    # Both caps should be circular (roughly equal XY extents)
    for part_obj, cap_name in [(lower, "lower"), (upper, "upper")]:
        cap_aabb = ctx.part_element_world_aabb(part_obj, elem="rivet_cap")
        if cap_aabb is not None:
            dx = cap_aabb[1][0] - cap_aabb[0][0]
            dy = cap_aabb[1][1] - cap_aabb[0][1]
            ctx.check(
                f"{cap_name} rivet cap is roughly circular",
                0.007 <= dx <= 0.013 and 0.007 <= dy <= 0.013,
                details=f"dx={dx:.4f}, dy={dy:.4f}",
            )

    # --- bent-nose jaws: jaw Y span shows angled profile wider than
    # a simple straight jaw would be
    for part_obj, jaw_name in [(lower, "lower"), (upper, "upper")]:
        jaw_aabb = ctx.part_element_world_aabb(part_obj, elem="jaw")
        if jaw_aabb is not None:
            y_span = jaw_aabb[1][1] - jaw_aabb[0][1]
            x_span = jaw_aabb[1][0] - jaw_aabb[0][0]
            ctx.check(
                f"{jaw_name} jaw has angled bent-nose profile",
                y_span > 0.008 and x_span > 0.020,
                details=f"y_span={y_span:.4f}, x_span={x_span:.4f}",
            )

    # --- rest pose: jaws open, handles splayed
    ctx.expect_gap(
        lower,
        upper,
        axis="y",
        positive_elem="jaw",
        negative_elem="jaw",
        min_gap=0.002,
        name="jaws open at rest",
    )
    ctx.expect_gap(
        upper,
        lower,
        axis="y",
        positive_elem="grip",
        negative_elem="grip",
        min_gap=0.010,
        name="handles splay apart at rest",
    )

    # --- closing pose: jaws come together, handles scissor
    open_jaw = ctx.part_element_world_aabb(upper, elem="jaw")
    open_grip = ctx.part_element_world_aabb(upper, elem="grip")
    with ctx.pose({pivot: CLOSE_TRAVEL}):
        ctx.expect_contact(
            lower,
            upper,
            elem_a="jaw",
            elem_b="jaw",
            contact_tol=0.003,
            name="jaw tips meet when fully closed",
        )
        closed_jaw = ctx.part_element_world_aabb(upper, elem="jaw")
        closed_grip = ctx.part_element_world_aabb(upper, elem="grip")

    ctx.check(
        "closing swings upper jaw toward lower jaw",
        open_jaw is not None
        and closed_jaw is not None
        and closed_jaw[1][1] > open_jaw[1][1] + 0.003,
        details=f"open={open_jaw}, closed={closed_jaw}",
    )
    ctx.check(
        "handles scissor opposite to jaws",
        open_grip is not None
        and closed_grip is not None
        and closed_grip[0][1] < open_grip[0][1] - 0.003,
        details=f"open={open_grip}, closed={closed_grip}",
    )

    # --- slip joint travel: pin actually moves along slot
    pin_rest = ctx.part_world_position(pin)
    with ctx.pose({slip: SLOT_LENGTH}):
        pin_moved = ctx.part_world_position(pin)
    ctx.check(
        "slip joint pin translates when posed",
        pin_rest is not None
        and pin_moved is not None
        and abs(pin_moved[0] - pin_rest[0]) > 0.004,
        details=f"rest={pin_rest}, moved={pin_moved}",
    )

    # --- overall proportions
    la = ctx.part_world_aabb(lower)
    ua = ctx.part_world_aabb(upper)
    if la is not None and ua is not None:
        length = max(la[1][0], ua[1][0]) - min(la[0][0], ua[0][0])
        width = max(la[1][1], ua[1][1]) - min(la[0][1], ua[0][1])
        ctx.check(
            "overall length ~0.13-0.16 m",
            0.11 <= length <= 0.17,
            details=f"length={length:.4f}",
        )
        ctx.check(
            "splayed handle width ~0.05-0.08 m",
            0.04 <= width <= 0.09,
            details=f"width={width:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
