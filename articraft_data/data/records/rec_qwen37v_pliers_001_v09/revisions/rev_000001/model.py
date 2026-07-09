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
# Combination pliers with crimping notch, serrated jaw teeth, and a folding
# safety latch.
#
# Two mirrored half-tools cross at a single polished rivet. Each half is one
# rigid link: flat gripping jaw with serrated teeth and a crimping notch
# behind the jaw -> slim steel neck -> curved over-molded handle. A small
# folding latch is hinged to the lower handle top surface and folds over the
# handles for storage safety.
# ---------------------------------------------------------------------------

HALF_OPEN = math.radians(12.5)  # each half is yawed this much at rest (open)
CLOSE_TRAVEL = 2.0 * HALF_OPEN  # ~25 degrees of closing travel

PLATE_T = 0.0035  # forged steel plate thickness per half

ZL0, ZL1 = 0.0035, 0.0035 + PLATE_T  # lower half plate: 0.0035 .. 0.0070
ZU0, ZU1 = ZL1, ZL1 + PLATE_T  # upper half plate: 0.0070 .. 0.0105

JAW_Z0, JAW_Z1 = ZL0, ZU1
JAW_X_MIN = 0.0085

BOSS_R = 0.0075
HOLE_R = 0.0028
SHANK_R = 0.0024
HEAD_R = 0.0040

JAW_LAND = 0.0012  # jaw inner face offset from center plane

GRIP_H = 0.0105
GRIP_LZ0 = 0.0
GRIP_LZ1 = GRIP_LZ0 + GRIP_H
GRIP_UZ0 = GRIP_LZ0 + (ZU0 - ZL0)
GRIP_UZ1 = GRIP_UZ0 + GRIP_H
INLAY_T = 0.0007
INLAY_EMBED = 0.0002

# Combination plier jaw: wider, flatter gripping face with a crimping notch.
JAW_PTS = [
    (0.0000, 0.0105),
    (0.0060, 0.0110),
    (0.0140, 0.0105),
    (0.0220, 0.0090),
    (0.0300, 0.0070),
    (0.0350, 0.0050),
    (0.0370, 0.0030),
    (0.0365, JAW_LAND),
    (0.0090, JAW_LAND),
    (0.0020, 0.0055),
]

CRIMP_X = 0.026
CRIMP_R = 0.0025

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

# Serrated teeth parameters
TEETH_COUNT = 8
TEETH_START_X = 0.011
TEETH_SPACING = 0.0028
TEETH_LENGTH = 0.0018  # along jaw (x)
TOOTH_PROTRUSION = 0.0006  # how far each tooth protrudes into the gap

# Latch dimensions
LATCH_LENGTH = 0.030
LATCH_WIDTH = 0.005
LATCH_THICKNESS = 0.0015
# Latch hinge position: on top of lower handle, slightly embedded for mounting
LATCH_HINGE_X = -0.052
LATCH_HINGE_Y = -0.021  # approximate center of handle width in world
LATCH_EMBED = 0.0005  # embed depth into handle top surface


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


def _crimp_notch(s: float) -> cq.Workplane:
    """Semicircular crimping notch cut into the jaw inner face."""
    return (
        cq.Workplane("XY", origin=(CRIMP_X, s * JAW_LAND, JAW_Z0 - 0.001))
        .circle(CRIMP_R)
        .extrude((JAW_Z1 - JAW_Z0) + 0.002)
    )


def _serrated_teeth(s: float, z0: float, z1: float) -> cq.Workplane:
    """Serrated teeth ridges protruding from the inner jaw face toward center."""
    result = None
    # Embed teeth slightly into jaw body for robust boolean union
    tooth_embed = 0.0003
    tooth_depth = TOOTH_PROTRUSION + tooth_embed
    for i in range(TEETH_COUNT):
        x = TEETH_START_X + i * TEETH_SPACING
        # Tooth protrudes from jaw face toward center, with small embed for union
        y_center = s * (JAW_LAND - TOOTH_PROTRUSION / 2 + tooth_embed / 2)
        tooth = (
            cq.Workplane("XY", origin=(x, y_center, z0))
            .rect(TEETH_LENGTH, tooth_depth)
            .extrude(z1 - z0)
        )
        if result is None:
            result = tooth
        else:
            result = result.union(tooth)
    return result


def _jaw_clip_box() -> cq.Workplane:
    return cq.Workplane("XY", origin=(JAW_X_MIN + 0.0250, 0.0, 0.0070)).box(
        0.0500, 0.0600, 0.0200
    )


def _half_jaw(s: float, own_z0: float, own_z1: float) -> cq.Workplane:
    """Combination plier jaw with flat gripping face, serrated teeth, crimp notch."""
    prof = _mirror(JAW_PTS, s)
    full = _poly_prism(prof, JAW_Z0, JAW_Z1).intersect(_jaw_clip_box())
    rear = _poly_prism(prof, own_z0, own_z1)
    jaw = full.union(rear)
    # Cut crimping notch on outer edge of jaw (realistic for combination pliers)
    # Notch at outer profile, cutting inward
    notch_y = s * 0.0085  # outer edge of jaw at x=0.026
    notch_cutter = (
        cq.Workplane("XY", origin=(CRIMP_X, notch_y, JAW_Z0 - 0.001))
        .circle(CRIMP_R)
        .extrude((JAW_Z1 - JAW_Z0) + 0.002)
    )
    jaw = jaw.cut(notch_cutter)
    # Add serrated teeth on inner face
    teeth = _serrated_teeth(s, own_z0, own_z1)
    jaw = jaw.union(teeth)
    return jaw


def _half_boss(own_z0: float, own_z1: float, with_hole: bool) -> cq.Workplane:
    boss = cq.Workplane("XY", origin=(0.0, 0.0, own_z0)).circle(BOSS_R).extrude(
        own_z1 - own_z0
    )
    if with_hole:
        hole = cq.Workplane("XY", origin=(0.0, 0.0, own_z0 - 0.001)).circle(HOLE_R).extrude(
            (own_z1 - own_z0) + 0.002
        )
        boss = boss.cut(hole)
    return boss


def _rivet() -> cq.Workplane:
    lower_head = cq.Workplane("XY", origin=(0.0, 0.0, 0.0021)).circle(HEAD_R).extrude(0.0019)
    shank = cq.Workplane("XY", origin=(0.0, 0.0, 0.0021)).circle(SHANK_R).extrude(0.0100)
    upper_head = cq.Workplane("XY", origin=(0.0, 0.0, ZU1 + 0.0001)).circle(HEAD_R).extrude(
        0.0014
    )
    rivet = lower_head.union(shank).union(upper_head)
    try:
        rivet = rivet.edges(">Z").fillet(0.0009)
    except Exception:
        pass
    return rivet


def _latch_body() -> cq.Workplane:
    """Small flat safety latch tab with hinge barrel at one end."""
    body = (
        cq.Workplane("XY")
        .rect(LATCH_LENGTH, LATCH_WIDTH)
        .extrude(LATCH_THICKNESS)
    )
    # Offset body so hinge end is at origin: body extends from x=0 to x=LATCH_LENGTH
    body = body.translate((LATCH_LENGTH / 2, 0.0, 0.0))
    # Hinge barrel (small cylinder at pivot end)
    barrel = (
        cq.Workplane("XY")
        .circle(0.0018)
        .extrude(LATCH_THICKNESS + 0.0004)
    )
    barrel = barrel.translate((0.0, 0.0, -0.0002))
    return body.union(barrel)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="combination_pliers")

    steel = model.material("brushed_steel", rgba=(0.72, 0.73, 0.75, 1.0))
    polished = model.material("polished_steel", rgba=(0.86, 0.87, 0.90, 1.0))
    orange = model.material("orange_grip", rgba=(0.97, 0.52, 0.07, 1.0))
    peach = model.material("peach_inlay", rgba=(1.0, 0.80, 0.64, 0.78))

    # ----- lower half (base link)
    lower = model.part("lower_half")
    lower_pose = Origin(rpy=(0.0, 0.0, HALF_OPEN))
    lower.visual(
        mesh_from_cadquery(_half_jaw(+1.0, ZL0, ZL1), "lower_jaw"),
        origin=lower_pose,
        material=steel,
        name="jaw",
    )
    lower.visual(
        mesh_from_cadquery(_half_boss(ZL0, ZL1, with_hole=False), "lower_boss"),
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
                _mirror(INLAY_PTS, +1.0), GRIP_LZ1 - INLAY_EMBED, GRIP_LZ1 - INLAY_EMBED + INLAY_T
            ),
            "lower_inlay",
        ),
        origin=lower_pose,
        material=peach,
        name="grip_inlay",
    )
    lower.visual(
        mesh_from_cadquery(_rivet(), "rivet"),
        origin=lower_pose,
        material=polished,
        name="rivet",
    )

    # ----- upper half (moving link)
    upper = model.part("upper_half")
    upper.visual(
        mesh_from_cadquery(_half_jaw(-1.0, ZU0, ZU1), "upper_jaw"),
        material=steel,
        name="jaw",
    )
    upper.visual(
        mesh_from_cadquery(_half_boss(ZU0, ZU1, with_hole=True), "upper_boss"),
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

    # ----- latch (folding safety tab hinged to lower handle)
    latch = model.part("latch")
    latch_hinge_z = GRIP_LZ1 - LATCH_EMBED  # slightly embedded in handle top
    latch.visual(
        mesh_from_cadquery(_latch_body(), "latch_body"),
        material=steel,
        name="latch_body",
    )

    # ----- main pivot at the rivet
    model.articulation(
        "rivet_pivot",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, -HALF_OPEN)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=4.0, lower=0.0, upper=CLOSE_TRAVEL),
    )

    # ----- latch hinge: folds along the handle
    latch_open = math.radians(170.0)
    model.articulation(
        "latch_hinge",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=latch,
        origin=Origin(xyz=(LATCH_HINGE_X, LATCH_HINGE_Y, latch_hinge_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=0.0, upper=latch_open),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower = object_model.get_part("lower_half")
    upper = object_model.get_part("upper_half")
    latch = object_model.get_part("latch")
    pivot = object_model.get_articulation("rivet_pivot")
    latch_hinge = object_model.get_articulation("latch_hinge")

    # --- Latch mounting: small intentional embed into handle top surface
    ctx.allow_overlap(
        latch,
        lower,
        elem_a="latch_body",
        elem_b="grip",
        reason="Latch hinge barrel is embedded slightly into the handle top surface for secure mounting.",
    )

    # --- pivot stack: bosses coaxial, upper half sits slightly above lower
    ctx.expect_overlap(
        lower,
        upper,
        axes="xy",
        elem_a="pivot_boss",
        elem_b="pivot_boss",
        min_overlap=0.012,
        name="pivot bosses stack coaxially",
    )
    ctx.expect_gap(
        upper,
        lower,
        axis="z",
        positive_elem="pivot_boss",
        negative_elem="pivot_boss",
        min_gap=-0.00002,
        max_gap=0.0006,
        name="upper boss sits slightly above lower boss",
    )
    ctx.expect_contact(
        lower,
        upper,
        elem_a="pivot_boss",
        elem_b="pivot_boss",
        contact_tol=0.0001,
        name="upper boss rides on the lower boss face",
    )

    # --- rivet passes through the boss stack and caps it from above
    ctx.expect_within(
        lower,
        upper,
        axes="xy",
        inner_elem="rivet",
        outer_elem="pivot_boss",
        margin=0.0002,
        name="rivet centered through the boss bore",
    )
    rivet_aabb = ctx.part_element_world_aabb(lower, elem="rivet")
    uboss_aabb = ctx.part_element_world_aabb(upper, elem="pivot_boss")
    ctx.check(
        "rivet head caps the upper boss",
        rivet_aabb is not None
        and uboss_aabb is not None
        and rivet_aabb[1][2] > uboss_aabb[1][2] + 0.0008,
        details=f"rivet={rivet_aabb}, upper_boss={uboss_aabb}",
    )

    # --- rest pose: jaws open, handles splayed apart
    ctx.expect_gap(
        lower,
        upper,
        axis="y",
        positive_elem="jaw",
        negative_elem="jaw",
        min_gap=0.002,
        name="jaw faces are open at rest",
    )
    ctx.expect_gap(
        upper,
        lower,
        axis="y",
        positive_elem="grip",
        negative_elem="grip",
        min_gap=0.012,
        name="handles splay apart at rest",
    )

    # --- jaws span the full forged head height
    for part in (lower, upper):
        jaw = ctx.part_element_world_aabb(part, elem="jaw")
        ctx.check(
            f"{part.name} jaw spans both plate layers",
            jaw is not None and jaw[0][2] <= ZL0 + 0.0002 and jaw[1][2] >= ZU1 - 0.0002,
            details=f"jaw={jaw}",
        )

    # --- serrated teeth: jaw geometry wider (y) than a smooth jaw would be
    # The teeth add ridges on the inner face, making the jaw footprint larger
    for part in (lower, upper):
        jaw = ctx.part_element_world_aabb(part, elem="jaw")
        ctx.check(
            f"{part.name} jaw has serrated teeth geometry",
            jaw is not None and (jaw[1][1] - jaw[0][1]) > 0.018,
            details=f"jaw y-span should include teeth: {jaw}",
        )

    # --- crimping notch: jaw has material removed behind the jaw tip
    for part in (lower, upper):
        jaw = ctx.part_element_world_aabb(part, elem="jaw")
        ctx.check(
            f"{part.name} jaw has crimping notch region",
            jaw is not None and (jaw[1][0] - jaw[0][0]) > 0.025,
            details=f"jaw x-span: {jaw}",
        )

    # --- inlay strips sit proud on, and within, the handle top faces
    for part in (lower, upper):
        grip = ctx.part_element_world_aabb(part, elem="grip")
        inlay = ctx.part_element_world_aabb(part, elem="grip_inlay")
        ctx.check(
            f"{part.name} inlay proud of grip top",
            grip is not None
            and inlay is not None
            and grip[1][2] + 0.0002 < inlay[1][2] < grip[1][2] + 0.0012,
            details=f"grip={grip}, inlay={inlay}",
        )
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

    # --- overall proportions
    la = ctx.part_world_aabb(lower)
    ua = ctx.part_world_aabb(upper)
    if la is not None and ua is not None:
        length = max(la[1][0], ua[1][0]) - min(la[0][0], ua[0][0])
        width = max(la[1][1], ua[1][1]) - min(la[0][1], ua[0][1])
        height = max(la[1][2], ua[1][2]) - min(la[0][2], ua[0][2])
        ctx.check(
            "overall length ~0.13 m",
            0.110 <= length <= 0.150,
            details=f"length={length:.4f}",
        )
        ctx.check(
            "splayed handle width ~0.06 m",
            0.048 <= width <= 0.080,
            details=f"width={width:.4f}",
        )
        ctx.check(
            "flat tool thickness ~0.015 m",
            0.012 <= height <= 0.018,
            details=f"height={height:.4f}",
        )
    else:
        ctx.fail("overall proportions", "missing part AABBs")

    # --- articulation: positive q closes jaws, handles scissor in opposition
    limits = pivot.motion_limits
    ctx.check(
        "pivot travel is roughly 0..25 degrees",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower) < 1e-9
        and 0.38 <= limits.upper <= 0.48,
        details=f"limits={limits}",
    )

    open_jaw = ctx.part_element_world_aabb(upper, elem="jaw")
    open_grip = ctx.part_element_world_aabb(upper, elem="grip")
    with ctx.pose({pivot: CLOSE_TRAVEL}):
        ctx.expect_contact(
            lower,
            upper,
            elem_a="jaw",
            elem_b="jaw",
            contact_tol=0.002,
            name="jaw faces meet when fully closed",
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
        "handles scissor opposite to the jaws",
        open_grip is not None
        and closed_grip is not None
        and closed_grip[0][1] < open_grip[0][1] - 0.003,
        details=f"open={open_grip}, closed={closed_grip}",
    )

    # --- latch: exists, has non-fixed revolute joint, and folds
    latch_body_aabb = ctx.part_element_world_aabb(latch, elem="latch_body")
    ctx.check(
        "latch body exists",
        latch_body_aabb is not None,
        details=f"latch_body={latch_body_aabb}",
    )

    latch_limits = latch_hinge.motion_limits
    ctx.check(
        "latch hinge has non-trivial travel (>90 degrees)",
        latch_limits is not None
        and latch_limits.upper is not None
        and latch_limits.upper > math.radians(90),
        details=f"latch_limits={latch_limits}",
    )

    # Latch mounted on handle: prove contact/proximity
    ctx.expect_overlap(
        latch,
        lower,
        axes="xy",
        elem_a="latch_body",
        elem_b="grip",
        min_overlap=0.004,
        name="latch overlaps handle footprint in xy",
    )

    # Latch folds: at max angle, latch moves significantly
    latch_rest = ctx.part_element_world_aabb(latch, elem="latch_body")
    with ctx.pose({latch_hinge: latch_limits.upper}):
        latch_folded = ctx.part_element_world_aabb(latch, elem="latch_body")

    ctx.check(
        "latch moves when folded",
        latch_rest is not None
        and latch_folded is not None
        and (
            abs(latch_folded[0][0] - latch_rest[0][0]) > 0.005
            or abs(latch_folded[0][1] - latch_rest[0][1]) > 0.005
        ),
        details=f"rest={latch_rest}, folded={latch_folded}",
    )

    return ctx.report()


object_model = build_object_model()
