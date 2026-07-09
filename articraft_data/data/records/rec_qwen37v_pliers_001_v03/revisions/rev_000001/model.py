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
# Broad lineman pliers with slip joint.
#
# Two forged halves cross at a slip-joint pin that rides in an elongated
# slot in the lower half. Each half: broad flat jaw (serrated gripping
# teeth on the inner face + a cutter notch near the pivot) -> steel neck
# -> overmolded handle. The upper half pivots on the pin (revolute),
# and the pin slides in the slot (prismatic) to adjust jaw capacity.
#
# Geometry is authored in a "closed-design" local frame per half with
# jaw at +y (lower) or -y (upper) and handle at the opposite side.
# Visual yaw splays the halves open at rest; positive revolute q closes
# the jaws while the handles scissor together.
# ---------------------------------------------------------------------------

HALF_OPEN = math.radians(10.0)
CLOSE_TRAVEL = 2.0 * HALF_OPEN

PLATE_T = 0.005
ZL0, ZL1 = 0.003, 0.008
ZU0, ZU1 = 0.008, 0.013
Z_MID = ZL1  # = ZU0

BOSS_R = 0.012
HOLE_R = 0.004

# Slip-joint slot in lower half
SLOT_LENGTH = 0.010
SLOT_WIDTH = 0.007
SLOT_BACK_X = -0.002
SLOT_FRONT_X = SLOT_BACK_X + SLOT_LENGTH
SLOT_MID_X = (SLOT_BACK_X + SLOT_FRONT_X) / 2.0

# Pin carriage
PIN_R = 0.003
PIN_HEAD_R = 0.005
PIN_HEAD_H = 0.0018
SHANK_HALF = 0.007

# Teeth on inner jaw face
TOOTH_COUNT = 8
TOOTH_DX = 0.0028
TOOTH_DY = 0.0013
TOOTH_X_START = 0.016
TOOTH_X_SPACING = 0.005

# Cutter notch (triangular cut from inner jaw edge near pivot)
CUTTER_X1 = 0.008
CUTTER_X2 = 0.017
CUTTER_DEPTH = 0.003

# Jaw outline (lower half, s=+1, jaw at +y side)
JAW_PTS = [
    (0.006, 0.002),
    (0.015, 0.002),
    (0.030, 0.002),
    (0.046, 0.003),
    (0.054, 0.005),
    (0.058, 0.009),
    (0.056, 0.015),
    (0.048, 0.020),
    (0.036, 0.022),
    (0.022, 0.021),
    (0.010, 0.018),
    (0.006, 0.013),
]

# Tang/neck (lower half, at -y side)
TANG_PTS = [
    (0.006, -0.004),
    (-0.006, -0.006),
    (-0.018, -0.007),
    (-0.030, -0.009),
    (-0.030, -0.015),
    (-0.018, -0.013),
    (-0.006, -0.010),
    (0.006, -0.008),
]

# Handle grip (lower half, at -y side)
GRIP_PTS = [
    (-0.024, -0.006),
    (-0.040, -0.008),
    (-0.058, -0.010),
    (-0.076, -0.012),
    (-0.094, -0.014),
    (-0.106, -0.017),
    (-0.112, -0.020),
    (-0.108, -0.024),
    (-0.094, -0.026),
    (-0.078, -0.026),
    (-0.060, -0.024),
    (-0.044, -0.020),
    (-0.030, -0.015),
    (-0.022, -0.010),
]

# Inlay strip on grip top face
INLAY_PTS = [
    (-0.032, -0.009),
    (-0.050, -0.012),
    (-0.068, -0.014),
    (-0.084, -0.016),
    (-0.094, -0.018),
    (-0.088, -0.021),
    (-0.072, -0.022),
    (-0.056, -0.020),
    (-0.040, -0.016),
    (-0.033, -0.012),
]

# Grip Z extents
GRIP_H = 0.009
GRIP_LZ0 = 0.001
GRIP_LZ1 = GRIP_LZ0 + GRIP_H
GRIP_UZ0 = -0.002
GRIP_UZ1 = GRIP_UZ0 + GRIP_H
INLAY_T = 0.0007
INLAY_EMBED = 0.0002


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
    cap: float = 0.0012,
    inset: float = 0.0010,
) -> cq.Workplane:
    """Extruded spline with loft-capped top/bottom for softly beveled edges."""
    try:
        mid = cq.Solid.extrudeLinear(
            cq.Face.makeFromWires(_spline_wire(pts, z0 + cap)),
            cq.Vector(0.0, 0.0, (z1 - z0) - 2.0 * cap),
        )
        top = cq.Solid.makeLoft(
            [_spline_wire(pts, z1 - cap), _spline_wire(pts, z1, inset=inset)]
        )
        bot = cq.Solid.makeLoft(
            [_spline_wire(pts, z0, inset=inset), _spline_wire(pts, z0 + cap)]
        )
        return cq.Workplane(obj=mid.fuse(top).fuse(bot))
    except Exception:
        # Fallback: plain spline extrusion without caps
        return _spline_prism(pts, z0, z1)


def _spline_prism(pts: list[tuple[float, float]], z0: float, z1: float) -> cq.Workplane:
    face = cq.Face.makeFromWires(_spline_wire(pts, z0))
    return cq.Workplane(obj=cq.Solid.extrudeLinear(face, cq.Vector(0.0, 0.0, z1 - z0)))


def _jaw_body(s: float, z0: float, z1: float) -> cq.Workplane:
    """Broad flat jaw body with cutter notch cut from inner edge."""
    pts = _mirror(JAW_PTS, s)
    jaw = _poly_prism(pts, z0, z1)

    # Cutter notch: triangular cut from inner edge near pivot
    inner_y = s * 0.002
    deep_y = s * (0.002 + CUTTER_DEPTH)
    mid_x = (CUTTER_X1 + CUTTER_X2) / 2.0
    notch_pts = [
        (CUTTER_X1, inner_y),
        (CUTTER_X2, inner_y),
        (mid_x, deep_y),
    ]
    notch = _poly_prism(notch_pts, z0 - 0.001, z1 + 0.001)
    jaw = jaw.cut(notch)
    return jaw


def _jaw_teeth(s: float, z0: float, z1: float) -> cq.Workplane:
    """Serrated gripping teeth on inner jaw face (one connected solid)."""
    # Base strip embedding into jaw face for connectivity
    inner_y = s * 0.002
    embed_y = s * (0.002 + 0.0006)
    x_start = TOOTH_X_START - 0.002
    x_end = TOOTH_X_START + (TOOTH_COUNT - 1) * TOOTH_X_SPACING + TOOTH_DX + 0.002

    base_pts = [
        (x_start, inner_y),
        (x_end, inner_y),
        (x_end, embed_y),
        (x_start, embed_y),
    ]
    solid = _poly_prism(base_pts, z0, z1)

    # Individual teeth protruding from inner face toward center
    tooth_y_center = s * (0.002 - TOOTH_DY / 2.0)
    for i in range(TOOTH_COUNT):
        tx = TOOTH_X_START + i * TOOTH_X_SPACING + TOOTH_DX / 2.0
        tooth = (
            cq.Workplane("XY", origin=(tx, tooth_y_center, (z0 + z1) / 2.0))
            .box(TOOTH_DX, TOOTH_DY, z1 - z0)
        )
        solid = solid.union(tooth)

    return solid


def _lower_boss(z0: float, z1: float) -> cq.Workplane:
    """Pivot boss for lower half with elongated slip-joint slot."""
    boss = (
        cq.Workplane("XY", origin=(SLOT_MID_X, 0.0, z0))
        .circle(BOSS_R)
        .extrude(z1 - z0)
    )
    # Slot: stadium shape (rect + two end circles)
    half_l = SLOT_LENGTH / 2.0
    half_w = SLOT_WIDTH / 2.0
    slot_rect = (
        cq.Workplane("XY", origin=(SLOT_MID_X, 0.0, z0 - 0.001))
        .rect(SLOT_LENGTH, SLOT_WIDTH)
        .extrude((z1 - z0) + 0.002)
    )
    slot_end1 = (
        cq.Workplane("XY", origin=(SLOT_MID_X - half_l, 0.0, z0 - 0.001))
        .circle(half_w)
        .extrude((z1 - z0) + 0.002)
    )
    slot_end2 = (
        cq.Workplane("XY", origin=(SLOT_MID_X + half_l, 0.0, z0 - 0.001))
        .circle(half_w)
        .extrude((z1 - z0) + 0.002)
    )
    slot = slot_rect.union(slot_end1).union(slot_end2)
    boss = boss.cut(slot)
    return boss


def _upper_boss(z0: float, z1: float) -> cq.Workplane:
    """Pivot boss for upper half with round pin bore."""
    boss = (
        cq.Workplane("XY", origin=(0.0, 0.0, z0))
        .circle(BOSS_R)
        .extrude(z1 - z0)
    )
    hole = (
        cq.Workplane("XY", origin=(0.0, 0.0, z0 - 0.001))
        .circle(HOLE_R)
        .extrude((z1 - z0) + 0.002)
    )
    boss = boss.cut(hole)
    return boss


def _tang(s: float, z0: float, z1: float) -> cq.Workplane:
    """Steel neck/tang from boss to handle."""
    return _poly_prism(_mirror(TANG_PTS, s), z0, z1)


def _grip(s: float, gz0: float, gz1: float) -> cq.Workplane:
    """Overmolded handle grip with soft edges."""
    return _soft_prism(_mirror(GRIP_PTS, s), gz0, gz1)


def _grip_inlay(s: float, top_z: float) -> cq.Workplane:
    """Translucent inlay strip proud of grip top face."""
    return _spline_prism(
        _mirror(INLAY_PTS, s),
        top_z - INLAY_EMBED,
        top_z - INLAY_EMBED + INLAY_T,
    )


def _pin_carriage() -> cq.Workplane:
    """Slip-joint pivot pin with bolt heads (built in pin-centered frame)."""
    shank = (
        cq.Workplane("XY", origin=(0.0, 0.0, -SHANK_HALF))
        .circle(PIN_R)
        .extrude(2.0 * SHANK_HALF)
    )
    head_bot = (
        cq.Workplane("XY", origin=(0.0, 0.0, -SHANK_HALF - PIN_HEAD_H))
        .circle(PIN_HEAD_R)
        .extrude(PIN_HEAD_H)
    )
    head_top = (
        cq.Workplane("XY", origin=(0.0, 0.0, SHANK_HALF))
        .circle(PIN_HEAD_R)
        .extrude(PIN_HEAD_H)
    )
    pin = shank.union(head_bot).union(head_top)
    try:
        pin = pin.edges(">Z").fillet(0.0008)
    except Exception:
        pass
    try:
        pin = pin.edges("<Z").fillet(0.0008)
    except Exception:
        pass
    return pin


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lineman_pliers_slip_joint")

    steel = model.material("brushed_steel", rgba=(0.72, 0.73, 0.75, 1.0))
    polished = model.material("polished_steel", rgba=(0.86, 0.87, 0.90, 1.0))
    orange = model.material("orange_grip", rgba=(0.95, 0.48, 0.06, 1.0))
    peach = model.material("peach_inlay", rgba=(1.0, 0.80, 0.64, 0.78))

    # Visual yaw applied to lower-half visuals (closed-design frame -> open rest pose)
    lower_pose = Origin(rpy=(0.0, 0.0, HALF_OPEN))

    # ===== LOWER HALF (base link, has slip-joint slot) =====
    lower = model.part("lower_half")

    lower.visual(
        mesh_from_cadquery(_jaw_body(+1.0, ZL0, ZL1), "lower_jaw"),
        origin=lower_pose,
        material=steel,
        name="jaw",
    )
    lower.visual(
        mesh_from_cadquery(_jaw_teeth(+1.0, ZL0, ZL1), "lower_teeth"),
        origin=lower_pose,
        material=steel,
        name="jaw_teeth",
    )
    lower.visual(
        mesh_from_cadquery(_lower_boss(ZL0, ZL1), "lower_boss"),
        origin=lower_pose,
        material=steel,
        name="pivot_boss",
    )
    lower.visual(
        mesh_from_cadquery(_tang(+1.0, ZL0, ZL1), "lower_tang"),
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
        mesh_from_cadquery(_grip_inlay(+1.0, GRIP_LZ1), "lower_inlay"),
        origin=lower_pose,
        material=peach,
        name="grip_inlay",
    )

    # ===== PIN CARRIAGE (slides in slot) =====
    pin_part = model.part("pin_carriage")
    pin_part.visual(
        mesh_from_cadquery(_pin_carriage(), "pin_bolt"),
        material=polished,
        name="pin_bolt",
    )

    # ===== UPPER HALF (moving link, pivots on pin) =====
    upper = model.part("upper_half")
    upper.visual(
        mesh_from_cadquery(_jaw_body(-1.0, 0.0, PLATE_T), "upper_jaw"),
        material=steel,
        name="jaw",
    )
    upper.visual(
        mesh_from_cadquery(_jaw_teeth(-1.0, 0.0, PLATE_T), "upper_teeth"),
        material=steel,
        name="jaw_teeth",
    )
    upper.visual(
        mesh_from_cadquery(_upper_boss(0.0, PLATE_T), "upper_boss"),
        material=steel,
        name="pivot_boss",
    )
    upper.visual(
        mesh_from_cadquery(_tang(-1.0, 0.0, PLATE_T), "upper_tang"),
        material=steel,
        name="neck_tang",
    )
    upper.visual(
        mesh_from_cadquery(_soft_prism(_mirror(GRIP_PTS, -1.0), GRIP_UZ0, GRIP_UZ1), "upper_grip"),
        material=orange,
        name="grip",
    )
    upper.visual(
        mesh_from_cadquery(_grip_inlay(-1.0, GRIP_UZ1), "upper_inlay"),
        material=peach,
        name="grip_inlay",
    )

    # ===== ARTICULATIONS =====

    # Prismatic: slip-joint pin slides along slot in lower half
    # Joint origin at back of slot in lower part frame, yawed to match visual frame
    model.articulation(
        "slip_joint",
        ArticulationType.PRISMATIC,
        parent=lower,
        child=pin_part,
        origin=Origin(
            xyz=(SLOT_BACK_X, 0.0, Z_MID),
            rpy=(0.0, 0.0, HALF_OPEN),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=50.0, velocity=0.5, lower=0.0, upper=SLOT_LENGTH
        ),
    )

    # Revolute: upper half pivots on pin
    # At q=0 the upper half is splayed open; positive q closes jaws.
    # Pin carriage frame is at HALF_OPEN yaw; revolute origin adds -CLOSE_TRAVEL
    # so upper half ends up at -HALF_OPEN at rest.
    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=pin_part,
        child=upper,
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, 0.0, -CLOSE_TRAVEL),
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0, velocity=4.0, lower=0.0, upper=CLOSE_TRAVEL
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower = object_model.get_part("lower_half")
    upper = object_model.get_part("upper_half")
    pin = object_model.get_part("pin_carriage")
    slip = object_model.get_articulation("slip_joint")
    pivot = object_model.get_articulation("pivot")

    # --- pin carriage is a captured sliding fastener: intentionally isolated
    # because clearance gaps surround the shank in the slot.
    ctx.allow_isolated_part(
        pin,
        reason="Pin carriage is a captured slip-joint fastener that slides with clearance in the lower-half slot; it is mechanically linked via prismatic and revolute articulations.",
    )
    # Proof: pin shank stays within the lower boss slot footprint
    ctx.expect_within(
        pin,
        lower,
        axes="xy",
        inner_elem="pin_bolt",
        outer_elem="pivot_boss",
        margin=0.002,
        name="pin bolt stays within lower boss slot footprint",
    )
    # Proof: pin passes through both plate layers in Z
    ctx.expect_overlap(
        pin,
        lower,
        axes="z",
        elem_a="pin_bolt",
        elem_b="pivot_boss",
        min_overlap=0.004,
        name="pin bolt spans both plate layers through the boss",
    )

    # --- slip joint is prismatic with finite travel
    slip_limits = slip.motion_limits
    ctx.check(
        "slip_joint is prismatic with positive travel",
        slip.articulation_type == ArticulationType.PRISMATIC
        and slip_limits is not None
        and slip_limits.lower is not None
        and slip_limits.upper is not None
        and slip_limits.upper > slip_limits.lower + 0.005,
        details=f"type={slip.articulation_type}, limits={slip_limits}",
    )

    # --- pivot is revolute with ~20 deg travel
    pivot_limits = pivot.motion_limits
    ctx.check(
        "pivot is revolute with reasonable travel",
        pivot.articulation_type == ArticulationType.REVOLUTE
        and pivot_limits is not None
        and pivot_limits.upper is not None
        and 0.25 <= pivot_limits.upper <= 0.50,
        details=f"type={pivot.articulation_type}, limits={pivot_limits}",
    )

    # --- pin carriage sits within the lower boss slot area
    ctx.expect_overlap(
        pin,
        lower,
        axes="xy",
        elem_a="pin_bolt",
        elem_b="pivot_boss",
        min_overlap=0.004,
        name="pin bolt overlaps with lower boss slot region",
    )

    # --- upper boss stacks on lower boss (via pin carriage intermediary)
    ctx.expect_overlap(
        upper,
        lower,
        axes="xy",
        elem_a="pivot_boss",
        elem_b="pivot_boss",
        min_overlap=0.008,
        name="upper and lower bosses overlap in XY at pivot",
    )

    # --- boss Z stacking: upper sits above lower
    ctx.expect_gap(
        upper,
        lower,
        axis="z",
        positive_elem="pivot_boss",
        negative_elem="pivot_boss",
        min_gap=-0.0005,
        max_gap=0.002,
        name="upper boss sits on or slightly above lower boss",
    )

    # --- serrated teeth exist on both jaws
    for p, teeth_name in [(lower, "jaw_teeth"), (upper, "jaw_teeth")]:
        teeth_aabb = ctx.part_element_world_aabb(p, elem=teeth_name)
        jaw_aabb = ctx.part_element_world_aabb(p, elem="jaw")
        ctx.check(
            f"{p.name} has serrated teeth on jaw",
            teeth_aabb is not None and jaw_aabb is not None,
            details=f"teeth={teeth_aabb}, jaw={jaw_aabb}",
        )
        if teeth_aabb is not None and jaw_aabb is not None:
            # Teeth should be near the inner face of the jaw (thin protrusion)
            teeth_span_y = teeth_aabb[1][1] - teeth_aabb[0][1]
            jaw_span_y = jaw_aabb[1][1] - jaw_aabb[0][1]
            ctx.check(
                f"{p.name} teeth are narrower than jaw (serrated ridges)",
                teeth_span_y < jaw_span_y * 0.5,
                details=f"teeth_y={teeth_span_y:.4f}, jaw_y={jaw_span_y:.4f}",
            )

    # --- cutter notch: jaw should have a concavity (notch) near pivot
    # Verified by checking that the jaw spans enough X to include the notch zone
    for p in (lower, upper):
        jaw_aabb = ctx.part_element_world_aabb(p, elem="jaw")
        ctx.check(
            f"{p.name} jaw extends past cutter notch zone",
            jaw_aabb is not None and (jaw_aabb[1][0] - jaw_aabb[0][0]) > 0.040,
            details=f"jaw_aabb={jaw_aabb}",
        )

    # --- broad jaw proportions (not thin tapered like side cutters)
    for p in (lower, upper):
        jaw_aabb = ctx.part_element_world_aabb(p, elem="jaw")
        ctx.check(
            f"{p.name} jaw is broad (>14mm span across face)",
            jaw_aabb is not None and (jaw_aabb[1][1] - jaw_aabb[0][1]) > 0.014,
            details=f"jaw_aabb={jaw_aabb}",
        )

    # --- rest pose: jaws open, handles splayed
    ctx.expect_gap(
        lower,
        upper,
        axis="y",
        positive_elem="jaw",
        negative_elem="jaw",
        min_gap=0.002,
        name="jaws are open at rest pose",
    )
    ctx.expect_gap(
        upper,
        lower,
        axis="y",
        positive_elem="grip",
        negative_elem="grip",
        min_gap=0.008,
        name="handles splay apart at rest",
    )

    # --- closing pose: jaws approach, handles scissor
    open_jaw_upper = ctx.part_element_world_aabb(upper, elem="jaw")
    open_grip_upper = ctx.part_element_world_aabb(upper, elem="grip")
    with ctx.pose({pivot: CLOSE_TRAVEL}):
        closed_jaw_upper = ctx.part_element_world_aabb(upper, elem="jaw")
        closed_grip_upper = ctx.part_element_world_aabb(upper, elem="grip")

    ctx.check(
        "closing swings upper jaw toward lower jaw",
        open_jaw_upper is not None
        and closed_jaw_upper is not None
        and closed_jaw_upper[1][1] > open_jaw_upper[1][1] + 0.002,
        details=f"open={open_jaw_upper}, closed={closed_jaw_upper}",
    )
    ctx.check(
        "handles scissor opposite to jaws",
        open_grip_upper is not None
        and closed_grip_upper is not None
        and closed_grip_upper[0][1] < open_grip_upper[0][1] - 0.002,
        details=f"open={open_grip_upper}, closed={closed_grip_upper}",
    )

    # --- slip joint prismatic motion moves the pin carriage
    pin_rest = ctx.part_world_position(pin)
    with ctx.pose({slip: SLOT_LENGTH}):
        pin_extended = ctx.part_world_position(pin)
    ctx.check(
        "slip joint slides pin carriage along slot",
        pin_rest is not None
        and pin_extended is not None
        and abs(pin_extended[0] - pin_rest[0]) + abs(pin_extended[1] - pin_rest[1]) > 0.006,
        details=f"rest={pin_rest}, extended={pin_extended}",
    )

    # --- overall proportions
    la = ctx.part_world_aabb(lower)
    ua = ctx.part_world_aabb(upper)
    if la is not None and ua is not None:
        length = max(la[1][0], ua[1][0]) - min(la[0][0], ua[0][0])
        width = max(la[1][1], ua[1][1]) - min(la[0][1], ua[0][1])
        height = max(la[1][2], ua[1][2]) - min(la[0][2], ua[0][2])
        ctx.check(
            "overall length ~0.15-0.20 m (lineman pliers)",
            0.14 <= length <= 0.22,
            details=f"length={length:.4f}",
        )
        ctx.check(
            "flat tool thickness ~0.015 m",
            0.012 <= height <= 0.022,
            details=f"height={height:.4f}",
        )
    else:
        ctx.fail("overall proportions", "missing part AABBs")

    # --- grip inlay sits proud on handle top faces
    for p in (lower, upper):
        grip = ctx.part_element_world_aabb(p, elem="grip")
        inlay = ctx.part_element_world_aabb(p, elem="grip_inlay")
        ctx.check(
            f"{p.name} inlay proud of grip top",
            grip is not None
            and inlay is not None
            and grip[1][2] - 0.0001 < inlay[1][2] < grip[1][2] + 0.0015,
            details=f"grip={grip}, inlay={inlay}",
        )

    return ctx.report()


object_model = build_object_model()
