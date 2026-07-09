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
# Slip-joint pliers.
#
# Two forged steel halves cross at a slip-joint pivot. The lower half has an
# elongated slot in its boss; a separate pivot pin with round caps on both
# sides slides along that slot (PRISMATIC). The upper half rotates about the
# pin (REVOLUTE) to open and close the jaws.
#
# The tool lies flat in the XY plane with Z as the thickness axis. Jaws point
# along +X, handles along -X. The lower half jaw is at +Y, upper half jaw at
# -Y, so the jaws cross at the pivot.
#
# Geometry is authored per half in a "closed-design" local frame. The rest
# pose splays each half by HALF_OPEN via visual/joint-frame yaw, so positive
# jaw-pivot q closes the blades while the handles scissor together.
# ---------------------------------------------------------------------------

HALF_OPEN = math.radians(12.5)
CLOSE_TRAVEL = 2.0 * HALF_OPEN  # ~25 deg

PLATE_T = 0.0035

# Z layer stack (lower plate below upper plate, exact face contact at ZL1).
ZL0, ZL1 = 0.0035, 0.0035 + PLATE_T
ZU0, ZU1 = ZL1, ZL1 + PLATE_T

BLADE_Z0, BLADE_Z1 = ZL0, ZU1  # full-height jaw region

BOSS_R = 0.0090
SLOT_LENGTH = 0.008   # slip-joint travel along slot axis
SLOT_WIDTH = 0.0050   # slot bore clearance

PIN_SHANK_R = 0.0022
PIN_CAP_R = 0.0045
PIN_CAP_T = 0.0015

BEVEL_X_START = 0.005
BEVEL_X_LENGTH = 0.012

# Handle over-mold stack.
GRIP_H = 0.0110
GRIP_LZ0 = 0.0
GRIP_LZ1 = GRIP_LZ0 + GRIP_H
GRIP_UZ0 = GRIP_LZ0 + (ZU0 - ZL0)
GRIP_UZ1 = GRIP_UZ0 + GRIP_H
INLAY_T = 0.0007
INLAY_EMBED = 0.0002

# Wider pliers jaw profile (lower half, +Y side, closed-design frame).
JAW_PTS = [
    (0.0020, 0.0100),
    (0.0080, 0.0115),
    (0.0180, 0.0115),
    (0.0300, 0.0105),
    (0.0400, 0.0080),
    (0.0450, 0.0050),
    (0.0460, 0.0030),
    (0.0440, 0.0015),
    (0.0400, 0.0010),
    (0.0250, 0.0020),
    (0.0120, 0.0025),
    (0.0050, 0.0040),
]

TANG_PTS = [
    (0.0020, -0.0040),
    (-0.0080, -0.0050),
    (-0.0200, -0.0060),
    (-0.0320, -0.0072),
    (-0.0320, -0.0120),
    (-0.0200, -0.0102),
    (-0.0080, -0.0088),
    (0.0010, -0.0078),
]

GRIP_PTS = [
    (-0.0260, -0.0045),
    (-0.0450, -0.0058),
    (-0.0680, -0.0072),
    (-0.0880, -0.0086),
    (-0.1040, -0.0096),
    (-0.1120, -0.0110),
    (-0.1080, -0.0145),
    (-0.0920, -0.0168),
    (-0.0740, -0.0175),
    (-0.0560, -0.0162),
    (-0.0360, -0.0132),
    (-0.0255, -0.0105),
]

INLAY_PTS = [
    (-0.0330, -0.0078),
    (-0.0520, -0.0092),
    (-0.0720, -0.0108),
    (-0.0860, -0.0115),
    (-0.0940, -0.0120),
    (-0.0880, -0.0132),
    (-0.0740, -0.0138),
    (-0.0580, -0.0130),
    (-0.0420, -0.0115),
    (-0.0340, -0.0100),
]


def _mirror(pts, s):
    return [(x, s * y) for (x, y) in pts]


def _poly_prism(pts, z0, z1):
    return (
        cq.Workplane("XY", origin=(0.0, 0.0, z0))
        .polyline(pts)
        .close()
        .extrude(z1 - z0)
    )


def _spline_wire(pts, z, inset=0.0):
    edge = cq.Edge.makeSpline([cq.Vector(x, y, z) for (x, y) in pts], periodic=True)
    wire = cq.Wire.assembleEdges([edge])
    if inset:
        wire = wire.offset2D(-inset)[0]
    return wire


def _soft_prism(pts, z0, z1, cap=0.0020, inset=0.0018):
    mid = cq.Solid.extrudeLinear(
        cq.Face.makeFromWires(_spline_wire(pts, z0 + cap)),
        cq.Vector(0.0, 0.0, (z1 - z0) - 2.0 * cap),
    )
    top = cq.Solid.makeLoft([_spline_wire(pts, z1 - cap), _spline_wire(pts, z1, inset=inset)])
    bot = cq.Solid.makeLoft([_spline_wire(pts, z0, inset=inset), _spline_wire(pts, z0 + cap)])
    return cq.Workplane(obj=mid.fuse(top).fuse(bot))


def _spline_prism(pts, z0, z1):
    face = cq.Face.makeFromWires(_spline_wire(pts, z0))
    return cq.Workplane(obj=cq.Solid.extrudeLinear(face, cq.Vector(0.0, 0.0, z1 - z0)))


def _jaw_clip_box():
    """Clip box that selects the full-height forward jaw region."""
    return cq.Workplane("XY", origin=(BEVEL_X_START - 0.002, 0.0, 0.0070)).box(
        0.0600, 0.0600, 0.0200
    )


def _half_jaw(s, own_z0, own_z1):
    """One half's jaw. s=+1 for lower (+Y), s=-1 for upper (-Y)."""
    prof = _mirror(JAW_PTS, s)
    full = _poly_prism(prof, BLADE_Z0, BLADE_Z1).intersect(_jaw_clip_box())
    rear = _poly_prism(prof, own_z0, own_z1)
    return full.union(rear)


def _cutter_bevel(s, z_lo, z_hi):
    """Visible wedge for the cutter bevel near the pivot.

    Creates a triangular prism representing the ground bevel face on the
    inner jaw surface.  s=+1 for lower jaw (bevel on top-inner), s=-1 for
    upper jaw (bevel on bottom-inner).
    """
    y_cut = s * 0.0012   # cutting edge (near center)
    y_back = s * 0.007   # back of bevel (into jaw body)
    bevel_depth = 0.0018

    if s > 0:
        tri = [
            (y_cut, z_hi),
            (y_back, z_hi),
            (y_cut, z_hi - bevel_depth),
        ]
    else:
        tri = [
            (y_cut, z_lo),
            (y_back, z_lo),
            (y_cut, z_lo + bevel_depth),
        ]

    return (
        cq.Workplane("YZ", origin=(BEVEL_X_START, 0.0, 0.0))
        .polyline(tri)
        .close()
        .extrude(BEVEL_X_LENGTH)
    )


def _half_boss_slotted(own_z0, own_z1):
    """Lower-half boss with elongated slip-joint slot along X."""
    boss = cq.Workplane("XY", origin=(0.0, 0.0, own_z0)).circle(BOSS_R).extrude(
        own_z1 - own_z0
    )
    slot_half = SLOT_LENGTH / 2.0
    slot_hw = SLOT_WIDTH / 2.0
    slot_rect = (
        cq.Workplane("XY", origin=(0.0, 0.0, own_z0 - 0.001))
        .rect(SLOT_LENGTH, SLOT_WIDTH)
        .extrude((own_z1 - own_z0) + 0.002)
    )
    slot_cap1 = (
        cq.Workplane("XY", origin=(slot_half, 0.0, own_z0 - 0.001))
        .circle(slot_hw)
        .extrude((own_z1 - own_z0) + 0.002)
    )
    slot_cap2 = (
        cq.Workplane("XY", origin=(-slot_half, 0.0, own_z0 - 0.001))
        .circle(slot_hw)
        .extrude((own_z1 - own_z0) + 0.002)
    )
    slot = slot_rect.union(slot_cap1).union(slot_cap2)
    return boss.cut(slot)


def _half_boss_solid(own_z0, own_z1):
    """Upper-half boss with round hole for the pin shank."""
    boss = cq.Workplane("XY", origin=(0.0, 0.0, own_z0)).circle(BOSS_R).extrude(
        own_z1 - own_z0
    )
    hole = (
        cq.Workplane("XY", origin=(0.0, 0.0, own_z0 - 0.001))
        .circle(PIN_SHANK_R + 0.0003)
        .extrude((own_z1 - own_z0) + 0.002)
    )
    return boss.cut(hole)


def _slip_pin():
    """Pivot pin with circular caps on both sides."""
    total_h = (ZU1 - ZL0) + 2.0 * PIN_CAP_T
    z_base = ZL0 - PIN_CAP_T
    shank = (
        cq.Workplane("XY", origin=(0.0, 0.0, z_base))
        .circle(PIN_SHANK_R)
        .extrude(total_h)
    )
    lower_cap = (
        cq.Workplane("XY", origin=(0.0, 0.0, z_base))
        .circle(PIN_CAP_R)
        .extrude(PIN_CAP_T)
    )
    upper_cap = (
        cq.Workplane("XY", origin=(0.0, 0.0, ZU1))
        .circle(PIN_CAP_R)
        .extrude(PIN_CAP_T)
    )
    pin = shank.union(lower_cap).union(upper_cap)
    try:
        pin = pin.edges("|Z").fillet(0.0005)
    except Exception:
        pass
    try:
        pin = pin.edges(">Z").fillet(0.0006)
        pin = pin.edges("<Z").fillet(0.0006)
    except Exception:
        pass
    return pin


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slip_joint_pliers")

    steel = model.material("brushed_steel", rgba=(0.72, 0.73, 0.75, 1.0))
    polished = model.material("polished_steel", rgba=(0.86, 0.87, 0.90, 1.0))
    orange = model.material("orange_grip", rgba=(0.97, 0.52, 0.07, 1.0))
    peach = model.material("peach_inlay", rgba=(1.0, 0.80, 0.64, 0.78))

    # ----- lower half (base link): jaw at +Y, handle at -Y, lower steel layer
    lower = model.part("lower_half")
    lower_pose = Origin(rpy=(0.0, 0.0, HALF_OPEN))

    lower.visual(
        mesh_from_cadquery(_half_jaw(+1.0, ZL0, ZL1), "lower_jaw"),
        origin=lower_pose,
        material=steel,
        name="jaw",
    )
    lower.visual(
        mesh_from_cadquery(_half_boss_slotted(ZL0, ZL1), "lower_boss"),
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
        mesh_from_cadquery(_soft_prism(_mirror(GRIP_PTS, +1.0), GRIP_LZ0, GRIP_LZ1), "lower_grip"),
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
    lower.visual(
        mesh_from_cadquery(_cutter_bevel(+1.0, ZL0, ZL1), "lower_bevel"),
        origin=lower_pose,
        material=steel,
        name="cutter_bevel",
    )

    # ----- slip pin: slides along slot in the lower boss
    pin = model.part("slip_pin")
    pin.visual(
        mesh_from_cadquery(_slip_pin(), "pin_body"),
        material=polished,
        name="pin_body",
    )

    # ----- upper half: jaw at -Y, handle at +Y, upper steel layer
    upper = model.part("upper_half")
    upper.visual(
        mesh_from_cadquery(_half_jaw(-1.0, ZU0, ZU1), "upper_jaw"),
        material=steel,
        name="jaw",
    )
    upper.visual(
        mesh_from_cadquery(_half_boss_solid(ZU0, ZU1), "upper_boss"),
        material=steel,
        name="pivot_boss",
    )
    upper.visual(
        mesh_from_cadquery(_poly_prism(_mirror(TANG_PTS, -1.0), ZU0, ZU1), "upper_tang"),
        material=steel,
        name="neck_tang",
    )
    upper.visual(
        mesh_from_cadquery(_soft_prism(_mirror(GRIP_PTS, -1.0), GRIP_UZ0, GRIP_UZ1), "upper_grip"),
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
    upper.visual(
        mesh_from_cadquery(_cutter_bevel(-1.0, ZU0, ZU1), "upper_bevel"),
        material=steel,
        name="cutter_bevel",
    )

    # ----- slip joint: pin slides along X (slot axis) in the yawed geometry frame
    # At q=0, pin is at the jaw-end (narrow opening position).
    # At q=SLOT_LENGTH, pin has slid toward the handle-end (wide opening).
    model.articulation(
        "slip_joint",
        ArticulationType.PRISMATIC,
        parent=lower,
        child=pin,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, HALF_OPEN)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0, velocity=0.1, lower=0.0, upper=SLOT_LENGTH
        ),
    )

    # ----- jaw pivot: upper half rotates about the pin Z axis.
    # The pin frame is already yawed by +HALF_OPEN via the slip joint.
    # We need the upper frame at yaw = -HALF_OPEN at q=0, so the jaw pivot
    # rpy must provide -2*HALF_OPEN relative to the pin frame.
    # Positive q closes jaws (brings upper jaw toward lower jaw).
    model.articulation(
        "jaw_pivot",
        ArticulationType.REVOLUTE,
        parent=pin,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, -2.0 * HALF_OPEN)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0, velocity=4.0, lower=0.0, upper=CLOSE_TRAVEL
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower = object_model.get_part("lower_half")
    pin = object_model.get_part("slip_pin")
    upper = object_model.get_part("upper_half")
    slip = object_model.get_articulation("slip_joint")
    pivot = object_model.get_articulation("jaw_pivot")

    # --- slip joint is prismatic with correct limits
    slip_limits = slip.motion_limits
    ctx.check(
        "slip_joint is prismatic",
        slip.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={slip.articulation_type}",
    )
    ctx.check(
        "slip_joint travel covers two pivot positions",
        slip_limits is not None
        and slip_limits.lower is not None
        and slip_limits.upper is not None
        and abs(slip_limits.lower) < 1e-9
        and 0.005 <= slip_limits.upper <= 0.012,
        details=f"limits={slip_limits}",
    )

    # --- jaw pivot is revolute with correct limits (~25 deg)
    pivot_limits = pivot.motion_limits
    ctx.check(
        "jaw_pivot is revolute",
        pivot.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={pivot.articulation_type}",
    )
    ctx.check(
        "jaw_pivot travel ~25 degrees",
        pivot_limits is not None
        and pivot_limits.lower is not None
        and pivot_limits.upper is not None
        and abs(pivot_limits.lower) < 1e-9
        and 0.38 <= pivot_limits.upper <= 0.48,
        details=f"limits={pivot_limits}",
    )

    # --- pin caps: pin extends above and below the assembled halves
    pin_aabb = ctx.part_element_world_aabb(pin, elem="pin_body")
    ctx.check(
        "pin cap extends above upper half",
        pin_aabb is not None and pin_aabb[1][2] > ZU1 + 0.0008,
        details=f"pin_aabb={pin_aabb}",
    )
    ctx.check(
        "pin cap extends below lower half",
        pin_aabb is not None and pin_aabb[0][2] < ZL0 - 0.0008,
        details=f"pin_aabb={pin_aabb}",
    )

    # --- cutter bevel wedges exist on both jaws
    for p, elem_name in [(lower, "cutter_bevel"), (upper, "cutter_bevel")]:
        bevel_aabb = ctx.part_element_world_aabb(p, elem=elem_name)
        ctx.check(
            f"{p.name} has visible cutter bevel wedge",
            bevel_aabb is not None
            and (bevel_aabb[1][0] - bevel_aabb[0][0]) > 0.008,
            details=f"bevel_aabb={bevel_aabb}",
        )

    # --- lower boss has slot (slot removes material: boss is not a full circle)
    lboss_aabb = ctx.part_element_world_aabb(lower, elem="pivot_boss")
    ctx.check(
        "lower boss exists with material",
        lboss_aabb is not None
        and (lboss_aabb[1][0] - lboss_aabb[0][0]) > 0.010,
        details=f"lboss_aabb={lboss_aabb}",
    )

    # --- pivot bosses stack: upper sits above lower
    ctx.expect_overlap(
        lower,
        upper,
        axes="xy",
        elem_a="pivot_boss",
        elem_b="pivot_boss",
        min_overlap=0.012,
        name="pivot bosses overlap in XY",
    )
    ctx.expect_gap(
        upper,
        lower,
        axis="z",
        positive_elem="pivot_boss",
        negative_elem="pivot_boss",
        min_gap=-0.0001,
        max_gap=0.001,
        name="upper boss sits above lower boss",
    )

    # --- rest pose: jaws open, handles splayed
    ctx.expect_gap(
        lower,
        upper,
        axis="y",
        positive_elem="jaw",
        negative_elem="jaw",
        min_gap=0.002,
        name="blade edges open at rest",
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

    # --- overall proportions (~0.16 m long, ~0.06 m wide)
    la = ctx.part_world_aabb(lower)
    ua = ctx.part_world_aabb(upper)
    if la is not None and ua is not None:
        length = max(la[1][0], ua[1][0]) - min(la[0][0], ua[0][0])
        width = max(la[1][1], ua[1][1]) - min(la[0][1], ua[0][1])
        ctx.check(
            "overall length ~0.15-0.17 m",
            0.135 <= length <= 0.185,
            details=f"length={length:.4f}",
        )
        ctx.check(
            "handle spread ~0.05-0.08 m",
            0.045 <= width <= 0.085,
            details=f"width={width:.4f}",
        )
    else:
        ctx.fail("overall proportions", "missing part AABBs")

    # --- articulation behavior: closing jaws
    open_blade = ctx.part_element_world_aabb(upper, elem="jaw")
    open_grip = ctx.part_element_world_aabb(upper, elem="grip")
    with ctx.pose({pivot: CLOSE_TRAVEL}):
        ctx.expect_contact(
            lower,
            upper,
            elem_a="jaw",
            elem_b="jaw",
            contact_tol=0.003,
            name="jaws meet when fully closed",
        )
        closed_blade = ctx.part_element_world_aabb(upper, elem="jaw")
        closed_grip = ctx.part_element_world_aabb(upper, elem="grip")

    ctx.check(
        "closing swings upper jaw toward lower jaw",
        open_blade is not None
        and closed_blade is not None
        and closed_blade[1][1] > open_blade[1][1] + 0.003,
        details=f"open={open_blade}, closed={closed_blade}",
    )
    ctx.check(
        "handles scissor opposite to the jaws",
        open_grip is not None
        and closed_grip is not None
        and closed_grip[0][1] < open_grip[0][1] - 0.003,
        details=f"open={open_grip}, closed={closed_grip}",
    )

    # --- slip joint: sliding the pin changes the pivot position
    pin_pos_0 = ctx.part_world_position(pin)
    with ctx.pose({slip: SLOT_LENGTH}):
        pin_pos_1 = ctx.part_world_position(pin)
    ctx.check(
        "slip joint moves pin along slot axis",
        pin_pos_0 is not None
        and pin_pos_1 is not None
        and abs(pin_pos_1[0] - pin_pos_0[0]) > 0.005,
        details=f"pos_0={pin_pos_0}, pos_1={pin_pos_1}",
    )

    # --- intentional overlaps: pin shank captured in both bosses
    ctx.allow_overlap(
        lower,
        pin,
        elem_a="pivot_boss",
        elem_b="pin_body",
        reason="Pin shank rides inside the slot of the lower boss (slip joint).",
    )
    ctx.allow_overlap(
        upper,
        pin,
        elem_a="pivot_boss",
        elem_b="pin_body",
        reason="Pin shank passes through the bore in the upper boss.",
    )
    ctx.allow_overlap(
        lower,
        upper,
        elem_a="cutter_bevel",
        elem_b="jaw",
        reason="Bevel wedge is seated into the opposing jaw surface as a ground cutting face.",
    )
    ctx.allow_overlap(
        upper,
        lower,
        elem_a="cutter_bevel",
        elem_b="jaw",
        reason="Bevel wedge is seated into the opposing jaw surface as a ground cutting face.",
    )
    ctx.allow_overlap(
        lower,
        lower,
        elem_a="cutter_bevel",
        elem_b="jaw",
        reason="Bevel wedge is seated into its own jaw body as a ground cutting face.",
    )
    ctx.allow_overlap(
        upper,
        upper,
        elem_a="cutter_bevel",
        elem_b="jaw",
        reason="Bevel wedge is seated into its own jaw body as a ground cutting face.",
    )

    # --- prove pin is captured (not floating) via contact with bosses
    ctx.expect_overlap(
        pin,
        lower,
        axes="xy",
        elem_a="pin_body",
        elem_b="pivot_boss",
        min_overlap=0.002,
        name="pin captured in lower boss slot",
    )
    ctx.expect_overlap(
        pin,
        upper,
        axes="xy",
        elem_a="pin_body",
        elem_b="pivot_boss",
        min_overlap=0.002,
        name="pin captured in upper boss bore",
    )

    return ctx.report()


object_model = build_object_model()
