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
# Round-nose jewelry pliers with slip joint.
#
# Two mirrored half-tools with tapered conical jaws, serrated inner teeth,
# a jaw stop boss near the pivot, and a slip-joint pin that slides along a
# short prismatic slot to adjust jaw opening range.
#
# Structure:
#   lower_half (root) --[PRISMATIC: slip_slot]--> slip_pin
#       --[REVOLUTE: pivot_joint]--> upper_half
#
# Geometry per half: tapered half-cone jaw -> slim steel neck -> curved
# over-molded handle. The lower boss has an elongated slot; the upper boss
# has a round bore. A small jaw stop boss protrudes near the pivot on the
# lower half to limit jaw closure.
# ---------------------------------------------------------------------------

HALF_OPEN = math.radians(12.5)  # each half splay at rest
CLOSE_TRAVEL = 2.0 * HALF_OPEN  # ~25 degrees of closing travel

PLATE_T = 0.004  # steel plate thickness per half (slightly thicker for pliers)

# Z layer stack: lower plate below, upper plate above
ZL0, ZL1 = 0.003, 0.003 + PLATE_T
ZU0, ZU1 = ZL1, ZL1 + PLATE_T

# --- Conical jaw parameters ---
JAW_R_BASE = 0.0040  # jaw radius at the pivot end
JAW_R_TIP = 0.0010   # jaw radius at the tapered tip
JAW_LENGTH = 0.036   # jaw extends from pivot toward +X
EDGE_LAND = 0.0006   # flat face offset from center plane (creates open gap)

# --- Serration teeth on inner jaw face ---
SERRATION_COUNT = 7
SERRATION_WIDTH = 0.0012   # along X
SERRATION_HEIGHT = 0.0006  # protrusion from flat inner face
SERRATION_Z_FRAC = 0.75    # fraction of plate height

# --- Jaw stop boss ---
STOP_BOSS_R = 0.0025
STOP_BOSS_H = 0.0012

# --- Pivot boss and slip slot ---
BOSS_R = 0.008
SLOT_LENGTH = 0.008   # prismatic travel
SLOT_WIDTH = 0.004    # slot bore width
PIN_R = 0.0018
PIN_HEAD_R = 0.003

# Handle over-mold
GRIP_H = 0.011
GRIP_LZ0 = 0.0
GRIP_LZ1 = GRIP_LZ0 + GRIP_H
GRIP_UZ0 = GRIP_LZ0 + (ZU0 - ZL0)
GRIP_UZ1 = GRIP_UZ0 + GRIP_H
INLAY_T = 0.0007
INLAY_EMBED = 0.0002

# Neck/tang outline (lower half: -y side of pivot)
TANG_PTS = [
    (0.0020, -0.0036),
    (-0.0100, -0.0048),
    (-0.0220, -0.0056),
    (-0.0320, -0.0064),
    (-0.0320, -0.0118),
    (-0.0220, -0.0100),
    (-0.0100, -0.0086),
    (0.0008, -0.0076),
]

# Curved over-molded handle outline
GRIP_PTS = [
    (-0.0260, -0.0040),
    (-0.0420, -0.0050),
    (-0.0600, -0.0064),
    (-0.0760, -0.0076),
    (-0.0900, -0.0084),
    (-0.0965, -0.0098),
    (-0.0925, -0.0132),
    (-0.0790, -0.0154),
    (-0.0630, -0.0158),
    (-0.0480, -0.0144),
    (-0.0320, -0.0118),
    (-0.0255, -0.0100),
]

# Translucent inlay strip along handle top
INLAY_PTS = [
    (-0.0325, -0.0072),
    (-0.0500, -0.0086),
    (-0.0680, -0.0098),
    (-0.0810, -0.0106),
    (-0.0880, -0.0110),
    (-0.0830, -0.0120),
    (-0.0690, -0.0128),
    (-0.0530, -0.0120),
    (-0.0390, -0.0106),
    (-0.0335, -0.0094),
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


def _conical_jaw(side: float, z0: float, z1: float) -> cq.Workplane:
    """Tapered half-cone jaw for round-nose pliers.
    side=+1: lower jaw at +Y; side=-1: upper jaw at -Y.
    The flat inner face is at y=side*EDGE_LAND."""
    # Full cone along +X axis
    cone = cq.Solid.makeCone(
        JAW_R_BASE, JAW_R_TIP, JAW_LENGTH,
        pnt=cq.Vector(0.0, 0.0, (z0 + z1) / 2),
        dir=cq.Vector(1.0, 0.0, 0.0),
    )
    cone_wp = cq.Workplane(obj=cone)

    # Cut in half at y=side*EDGE_LAND (leaves a small open gap at the center plane)
    cut_l = JAW_LENGTH * 2.0
    cut_h = (z1 - z0) * 3.0
    big = 0.020
    big_ext = big + EDGE_LAND
    if side > 0:
        # Keep +Y side; cut away everything below y = EDGE_LAND
        cy = (EDGE_LAND - big) / 2
        cutter = (
            cq.Workplane("XY", origin=(JAW_LENGTH / 2, cy, (z0 + z1) / 2))
            .box(cut_l, big_ext, cut_h)
        )
    else:
        # Keep -Y side; cut away everything above y = -EDGE_LAND
        cy = (big - EDGE_LAND) / 2
        cutter = (
            cq.Workplane("XY", origin=(JAW_LENGTH / 2, cy, (z0 + z1) / 2))
            .box(cut_l, big_ext, cut_h)
        )
    half_cone = cone_wp.cut(cutter)

    # Clip to Z extent
    z_box = (
        cq.Workplane("XY", origin=(JAW_LENGTH / 2, 0.0, (z0 + z1) / 2))
        .box(JAW_LENGTH * 2, JAW_R_BASE * 4, z1 - z0)
    )
    return half_cone.intersect(z_box)


def _serrations(side: float, z0: float, z1: float) -> cq.Workplane:
    """Serrated teeth ridges on the inner flat face of the jaw.
    The flat face is at y = side * EDGE_LAND. Teeth straddle the face:
    half embedded in the jaw body, half protruding inward (toward y=0)."""
    spacing = (JAW_LENGTH * 0.85) / SERRATION_COUNT
    tooth_h = SERRATION_HEIGHT
    tooth_w = SERRATION_WIDTH
    tooth_z = (z1 - z0) * SERRATION_Z_FRAC
    z_mid = (z0 + z1) / 2

    result = None
    for i in range(SERRATION_COUNT):
        x = 0.004 + spacing * (i + 0.5)
        # Radius at this x position along the cone
        t = x / JAW_LENGTH
        r_at_x = JAW_R_BASE * (1 - t) + JAW_R_TIP * t
        # Tooth depth: limited by jaw radius at that point
        tooth_d = min(tooth_h, r_at_x * 0.6)

        # Center tooth on the flat face: half in jaw body, half protruding inward
        y_center = side * EDGE_LAND

        tooth = (
            cq.Workplane("XY", origin=(x, y_center, z_mid))
            .box(tooth_w, tooth_d, tooth_z)
        )
        if result is None:
            result = tooth
        else:
            result = result.union(tooth)
    return result


def _jaw_stop_boss(z0: float, z1: float) -> cq.Workplane:
    """Small boss protruding from the lower jaw near the pivot, limiting closure."""
    # Protrudes upward from the lower jaw's top face, near x=0.005, y=+JAW_R_BASE*0.5
    boss = (
        cq.Workplane("XY", origin=(0.005, JAW_R_BASE * 0.4, z1))
        .circle(STOP_BOSS_R)
        .extrude(STOP_BOSS_H)
    )
    try:
        boss = boss.edges(">Z").fillet(STOP_BOSS_R * 0.3)
    except Exception:
        pass
    return boss


def _lower_boss_with_slot(z0: float, z1: float) -> cq.Workplane:
    """Lower pivot boss with an elongated slot for the slip joint pin."""
    boss = (
        cq.Workplane("XY", origin=(0.0, 0.0, z0))
        .circle(BOSS_R)
        .extrude(z1 - z0)
    )
    # Cut elongated slot along X
    slot = (
        cq.Workplane("XY", origin=(SLOT_LENGTH / 2, 0.0, z0 - 0.001))
        .rect(SLOT_LENGTH + SLOT_WIDTH, SLOT_WIDTH)
        .extrude((z1 - z0) + 0.002)
    )
    # Round the slot ends
    slot_end_a = (
        cq.Workplane("XY", origin=(0.0, 0.0, z0 - 0.001))
        .circle(SLOT_WIDTH / 2)
        .extrude((z1 - z0) + 0.002)
    )
    slot_end_b = (
        cq.Workplane("XY", origin=(SLOT_LENGTH, 0.0, z0 - 0.001))
        .circle(SLOT_WIDTH / 2)
        .extrude((z1 - z0) + 0.002)
    )
    full_slot = slot.union(slot_end_a).union(slot_end_b)
    return boss.cut(full_slot)


def _upper_boss_with_hole(z0: float, z1: float) -> cq.Workplane:
    """Upper pivot boss with a round bore for the pin."""
    boss = (
        cq.Workplane("XY", origin=(0.0, 0.0, z0))
        .circle(BOSS_R)
        .extrude(z1 - z0)
    )
    hole = (
        cq.Workplane("XY", origin=(0.0, 0.0, z0 - 0.001))
        .circle(PIN_R + 0.0002)
        .extrude((z1 - z0) + 0.002)
    )
    return boss.cut(hole)


def _slip_pin() -> cq.Workplane:
    """Slip joint pin: cylindrical shank with a domed head."""
    pin_h = (ZU1 - ZL0) + 0.003
    shank = (
        cq.Workplane("XY", origin=(0.0, 0.0, ZL0 - 0.001))
        .circle(PIN_R)
        .extrude(pin_h)
    )
    head = (
        cq.Workplane("XY", origin=(0.0, 0.0, ZU1 + 0.001))
        .circle(PIN_HEAD_R)
        .extrude(0.0015)
    )
    tail = (
        cq.Workplane("XY", origin=(0.0, 0.0, ZL0 - 0.0025))
        .circle(PIN_HEAD_R)
        .extrude(0.0015)
    )
    pin = shank.union(head).union(tail)
    try:
        pin = pin.edges(">Z").fillet(0.0006)
    except Exception:
        pass
    return pin


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="round_nose_jewelry_pliers")

    steel = model.material("brushed_steel", rgba=(0.72, 0.73, 0.75, 1.0))
    polished = model.material("polished_steel", rgba=(0.86, 0.87, 0.90, 1.0))
    orange = model.material("orange_grip", rgba=(0.95, 0.45, 0.08, 1.0))
    peach = model.material("peach_inlay", rgba=(1.0, 0.80, 0.64, 0.78))

    # ----- lower half (base link): jaw at +y, handle at -y, lower steel layer
    lower = model.part("lower_half")
    lower_pose = Origin(rpy=(0.0, 0.0, HALF_OPEN))

    lower.visual(
        mesh_from_cadquery(_conical_jaw(+1.0, ZL0, ZL1), "lower_jaw"),
        origin=lower_pose,
        material=steel,
        name="conical_jaw",
    )
    lower.visual(
        mesh_from_cadquery(_serrations(+1.0, ZL0, ZL1), "lower_serrations"),
        origin=lower_pose,
        material=steel,
        name="jaw_teeth",
    )
    lower.visual(
        mesh_from_cadquery(_lower_boss_with_slot(ZL0, ZL1), "lower_boss_slot"),
        origin=lower_pose,
        material=steel,
        name="pivot_boss",
    )
    lower.visual(
        mesh_from_cadquery(_jaw_stop_boss(ZL0, ZL1), "jaw_stop"),
        origin=lower_pose,
        material=steel,
        name="jaw_stop_boss",
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
                _mirror(INLAY_PTS, +1.0), GRIP_LZ1 - INLAY_EMBED, GRIP_LZ1 - INLAY_EMBED + INLAY_T
            ),
            "lower_inlay",
        ),
        origin=lower_pose,
        material=peach,
        name="grip_inlay",
    )

    # ----- slip pin (intermediate link for slip joint)
    slip_pin_part = model.part("slip_pin")
    slip_pin_part.visual(
        mesh_from_cadquery(_slip_pin(), "slip_pin_mesh"),
        material=polished,
        name="pin",
    )

    # ----- upper half (moving link): mirrored, upper steel layer
    upper = model.part("upper_half")
    upper.visual(
        mesh_from_cadquery(_conical_jaw(-1.0, ZU0, ZU1), "upper_jaw"),
        material=steel,
        name="conical_jaw",
    )
    upper.visual(
        mesh_from_cadquery(_serrations(-1.0, ZU0, ZU1), "upper_serrations"),
        material=steel,
        name="jaw_teeth",
    )
    upper.visual(
        mesh_from_cadquery(_upper_boss_with_hole(ZU0, ZU1), "upper_boss_hole"),
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
                _mirror(INLAY_PTS, -1.0), GRIP_UZ1 - INLAY_EMBED, GRIP_UZ1 - INLAY_EMBED + INLAY_T
            ),
            "upper_inlay",
        ),
        material=peach,
        name="grip_inlay",
    )

    # ----- Articulation 1: slip slot (PRISMATIC)
    # Pin slides along +X in the lower boss slot.
    # At q=0, pin is at slot start (x=0). At q=SLOT_LENGTH, pin at slot end.
    model.articulation(
        "slip_slot",
        ArticulationType.PRISMATIC,
        parent=lower,
        child=slip_pin_part,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, HALF_OPEN)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.5, lower=0.0, upper=SLOT_LENGTH),
    )

    # ----- Articulation 2: pivot joint (REVOLUTE)
    # Upper half rotates around the slip pin.
    # At q=0, jaws are open (splayed). Positive q closes jaws.
    model.articulation(
        "pivot_joint",
        ArticulationType.REVOLUTE,
        parent=slip_pin_part,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, -HALF_OPEN)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=4.0, lower=0.0, upper=CLOSE_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower = object_model.get_part("lower_half")
    upper = object_model.get_part("upper_half")
    pin_part = object_model.get_part("slip_pin")
    slip = object_model.get_articulation("slip_slot")
    pivot = object_model.get_articulation("pivot_joint")

    # --- Conical jaws exist on both halves
    for part, pname in [(lower, "lower"), (upper, "upper")]:
        jaw = part.get_visual("conical_jaw")
        ctx.check(
            f"{pname} half has conical jaw",
            jaw is not None,
            details=f"visual={jaw}",
        )

    # --- Serrated teeth exist on both halves
    for part, pname in [(lower, "lower"), (upper, "upper")]:
        teeth = part.get_visual("jaw_teeth")
        ctx.check(
            f"{pname} half has serrated teeth",
            teeth is not None,
            details=f"visual={teeth}",
        )

    # --- Jaw stop boss exists on lower half
    stop = lower.get_visual("jaw_stop_boss")
    ctx.check(
        "jaw stop boss exists on lower half",
        stop is not None,
        details=f"visual={stop}",
    )

    # --- Jaw stop boss protrudes above lower boss top face
    stop_aabb = ctx.part_element_world_aabb(lower, elem="jaw_stop_boss")
    boss_aabb = ctx.part_element_world_aabb(lower, elem="pivot_boss")
    ctx.check(
        "jaw stop boss protrudes above lower boss",
        stop_aabb is not None
        and boss_aabb is not None
        and stop_aabb[1][2] > boss_aabb[1][2] - 0.0001,
        details=f"stop={stop_aabb}, boss={boss_aabb}",
    )

    # --- Slip joint: slot exists in lower boss, pin exists
    slot_boss = lower.get_visual("pivot_boss")
    pin_vis = pin_part.get_visual("pin")
    ctx.check("lower boss has slot geometry", slot_boss is not None)
    ctx.check("slip pin exists", pin_vis is not None)

    # --- Pin is within the boss XY footprint
    ctx.expect_within(
        pin_part,
        lower,
        axes="xy",
        inner_elem="pin",
        outer_elem="pivot_boss",
        margin=0.002,
        name="pin within lower boss footprint",
    )

    # --- Pivot bosses stack: upper sits above lower
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
        min_gap=-0.0002,
        max_gap=0.001,
        name="upper boss sits above lower boss",
    )

    # --- Rest pose: jaws open, handles splayed apart
    ctx.expect_gap(
        lower,
        upper,
        axis="y",
        positive_elem="conical_jaw",
        negative_elem="conical_jaw",
        min_gap=0.001,
        name="conical jaws are open at rest",
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

    # --- Slip joint articulation is PRISMATIC with correct limits
    slip_limits = slip.motion_limits
    ctx.check(
        "slip slot is prismatic with positive travel",
        slip.articulation_type == ArticulationType.PRISMATIC
        and slip_limits is not None
        and slip_limits.lower is not None
        and slip_limits.upper is not None
        and abs(slip_limits.lower) < 1e-9
        and 0.005 <= slip_limits.upper <= 0.012,
        details=f"type={slip.articulation_type}, limits={slip_limits}",
    )

    # --- Pivot articulation is REVOLUTE with ~25 degree travel
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

    # --- Closing: positive pivot q closes jaws, handles scissor opposite
    open_jaw = ctx.part_element_world_aabb(upper, elem="conical_jaw")
    open_grip = ctx.part_element_world_aabb(upper, elem="grip")
    with ctx.pose({pivot: CLOSE_TRAVEL}):
        closed_jaw = ctx.part_element_world_aabb(upper, elem="conical_jaw")
        closed_grip = ctx.part_element_world_aabb(upper, elem="grip")

    ctx.check(
        "closing swings upper jaw toward lower jaw",
        open_jaw is not None
        and closed_jaw is not None
        and closed_jaw[1][1] > open_jaw[1][1] + 0.003,
        details=f"open={open_jaw}, closed={closed_jaw}",
    )
    ctx.check(
        "handles scissor opposite to the jaws",
        open_grip is not None
        and closed_grip is not None
        and closed_grip[0][1] < open_grip[0][1] - 0.003,
        details=f"open={open_grip}, closed={closed_grip}",
    )

    # --- Slip joint: pin slides along X when prismatic q changes
    pin_rest = ctx.part_world_position(pin_part)
    with ctx.pose({slip: SLOT_LENGTH}):
        pin_extended = ctx.part_world_position(pin_part)
    ctx.check(
        "slip pin translates along slot when actuated",
        pin_rest is not None
        and pin_extended is not None
        and abs(pin_extended[0] - pin_rest[0]) > 0.004,
        details=f"rest={pin_rest}, extended={pin_extended}",
    )

    # --- Overall proportions: ~0.13-0.15 m long, ~0.06 across handles
    la = ctx.part_world_aabb(lower)
    ua = ctx.part_world_aabb(upper)
    if la is not None and ua is not None:
        length = max(la[1][0], ua[1][0]) - min(la[0][0], ua[0][0])
        width = max(la[1][1], ua[1][1]) - min(la[0][1], ua[0][1])
        ctx.check(
            "overall length ~0.13 m",
            0.110 <= length <= 0.160,
            details=f"length={length:.4f}",
        )
        ctx.check(
            "splayed handle width ~0.06 m",
            0.045 <= width <= 0.085,
            details=f"width={width:.4f}",
        )
    else:
        ctx.fail("overall proportions", "missing part AABBs")

    # --- Serrated teeth are mounted on the inner jaw face
    for part, pname in [(lower, "lower"), (upper, "upper")]:
        # Teeth overlap with jaw in X (mounted along jaw length)
        ctx.expect_overlap(
            part, part,
            axes="x",
            elem_a="jaw_teeth",
            elem_b="conical_jaw",
            min_overlap=0.010,
            name=f"{pname} teeth mounted along jaw length",
        )
        # Teeth are within the jaw's Z extent
        ctx.expect_within(
            part, part,
            axes="z",
            inner_elem="jaw_teeth",
            outer_elem="conical_jaw",
            margin=0.001,
            name=f"{pname} teeth within jaw height",
        )

    return ctx.report()


object_model = build_object_model()
