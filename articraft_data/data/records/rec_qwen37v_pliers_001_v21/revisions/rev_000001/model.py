from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Needle-nose pliers variant.
#
# Two mirrored half-tools cross at a single polished rivet. Each half is one
# rigid link: long tapered needle-nose jaw with serrated inner teeth -> slim
# steel neck -> curved over-molded handle. A leaf return spring arcs between
# the handles to bias them open.
#
# Geometry is authored per half in a "closed-design" local frame in which the
# two halves are exactly mirrored about the XZ plane and the jaw inner-face
# serrations sit at y = +/-EDGE_LAND. The rest pose (q=0) splays each half by
# HALF_OPEN via visual/joint-frame yaw, so q in [0, 2*HALF_OPEN] closes the
# jaw tips together while the handles scissor apart and together in opposition.
# ---------------------------------------------------------------------------

HALF_OPEN = math.radians(12.5)  # each half is yawed this much at rest (open)
CLOSE_TRAVEL = 2.0 * HALF_OPEN  # ~25 degrees of closing travel

PLATE_T = 0.0035  # forged steel plate thickness per half

# Steel layer stack (lower plate sits below upper plate at the pivot boss;
# the upper plate rides directly on the lower plate, exact face contact).
ZL0, ZL1 = 0.0035, 0.0035 + PLATE_T  # lower half plate: 0.0035 .. 0.0070
ZU0, ZU1 = ZL1, ZL1 + PLATE_T  # upper half plate: 0.0070 .. 0.0105

# Full-height jaw region (jaw head spans both plate layers, flat underside).
JAW_Z0, JAW_Z1 = ZL0, ZU1
JAW_X_MIN = 0.0085  # full-height jaw exists only forward of the boss

BOSS_R = 0.0075
HOLE_R = 0.0028  # clearance hole in the upper boss for the rivet shank
SHANK_R = 0.0024
HEAD_R = 0.0040

EDGE_LAND = 0.0004  # each jaw inner face sits this far off the shear plane

# Handle over-mold (z extents per half, soft loft-capped edges).
GRIP_H = 0.0095
GRIP_LZ0 = 0.0
GRIP_LZ1 = GRIP_LZ0 + GRIP_H  # lower grip 0.0 .. 0.0095
GRIP_UZ0 = GRIP_LZ0 + (ZU0 - ZL0)  # upper grip 0.0035 .. 0.0130
GRIP_UZ1 = GRIP_UZ0 + GRIP_H
INLAY_T = 0.0007
INLAY_EMBED = 0.0002

# Needle-nose jaw profile (lower half, jaw body at +y, inner face along
# y = EDGE_LAND). Long tapered shape ~45mm from pivot center to tip.
JAW_PTS = [
    (0.0000, 0.0080),
    (0.0060, 0.0082),
    (0.0140, 0.0070),
    (0.0240, 0.0050),
    (0.0340, 0.0032),
    (0.0420, 0.0018),
    (0.0470, 0.0010),
    (0.0490, EDGE_LAND),
    (0.0090, EDGE_LAND),
    (0.0020, 0.0042),
]

# Serrated teeth on the inner jaw face - small triangular ridges
# perpendicular to the jaw length. These are modeled as a series of small
# triangular prisms along the inner face (y near EDGE_LAND).
TEETH_COUNT = 10
TEETH_X_START = 0.012
TEETH_X_END = 0.044
TEETH_DEPTH = 0.0006  # how far teeth protrude inward from EDGE_LAND
TEETH_HEIGHT = 0.0012  # height of each tooth (z direction)
TEETH_WIDTH = 0.0018  # width of each tooth along x

# Slim steel neck/tang from the boss back to the handle (lower half: -y side).
TANG_PTS = [
    (0.0020, -0.0030),
    (-0.0100, -0.0040),
    (-0.0200, -0.0048),
    (-0.0300, -0.0056),
    (-0.0300, -0.0100),
    (-0.0200, -0.0088),
    (-0.0100, -0.0076),
    (0.0008, -0.0066),
]

# Curved over-molded handle outline (periodic spline; slender, rounded tip).
GRIP_PTS = [
    (-0.0240, -0.0034),
    (-0.0380, -0.0042),
    (-0.0540, -0.0052),
    (-0.0700, -0.0062),
    (-0.0850, -0.0070),
    (-0.0940, -0.0080),
    (-0.0920, -0.0110),
    (-0.0800, -0.0128),
    (-0.0660, -0.0136),
    (-0.0520, -0.0128),
    (-0.0360, -0.0108),
    (-0.0235, -0.0088),
]

# Translucent peach inlay strip along the top face of the handle.
INLAY_PTS = [
    (-0.0305, -0.0060),
    (-0.0480, -0.0072),
    (-0.0640, -0.0084),
    (-0.0770, -0.0092),
    (-0.0840, -0.0096),
    (-0.0790, -0.0106),
    (-0.0650, -0.0112),
    (-0.0500, -0.0104),
    (-0.0370, -0.0090),
    (-0.0315, -0.0080),
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
    """Extruded spline outline with loft-capped (softly beveled) top/bottom."""
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


def _jaw_teeth(s: float, own_z0: float, own_z1: float) -> cq.Workplane:
    """Build serrated teeth on the inner jaw face. Each tooth is a small
    triangular ridge protruding inward from the jaw inner face."""
    teeth_z_mid = (own_z0 + own_z1) * 0.5
    tooth_h = TEETH_HEIGHT
    tooth_z0 = teeth_z_mid - tooth_h * 0.5
    tooth_z1 = teeth_z_mid + tooth_h * 0.5

    dx = (TEETH_X_END - TEETH_X_START) / max(TEETH_COUNT - 1, 1)
    result = None
    for i in range(TEETH_COUNT):
        cx = TEETH_X_START + i * dx
        # Triangular tooth cross-section in YZ plane:
        # base at y = s*EDGE_LAND (inner jaw face), tip pointing inward
        y_base = s * EDGE_LAND
        y_tip = y_base - s * TEETH_DEPTH  # protrudes inward (toward center)
        tri = [
            (y_base, tooth_z0),
            (y_base, tooth_z1),
            (y_tip, teeth_z_mid),
        ]
        tooth = (
            cq.Workplane("YZ", origin=(cx - TEETH_WIDTH * 0.5, 0.0, 0.0))
            .polyline(tri)
            .close()
            .extrude(TEETH_WIDTH)
        )
        if result is None:
            result = tooth
        else:
            result = result.union(tooth)
    return result


def _blade_clip_box() -> cq.Workplane:
    return cq.Workplane("XY", origin=(JAW_X_MIN + 0.0250, 0.0, 0.0070)).box(
        0.0600, 0.0600, 0.0200
    )


def _half_jaw(s: float, own_z0: float, own_z1: float) -> cq.Workplane:
    """Build the needle-nose jaw body with serrated inner teeth."""
    prof = _mirror(JAW_PTS, s)
    full = _poly_prism(prof, JAW_Z0, JAW_Z1).intersect(_blade_clip_box())
    rear = _poly_prism(prof, own_z0, own_z1)
    jaw = full.union(rear)
    # Add serrated teeth on inner face
    teeth = _jaw_teeth(s, own_z0, own_z1)
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


def _return_spring() -> cq.Workplane:
    """Leaf return spring: a curved strip of spring steel anchored at the
    pivot boss and arcing between the two handles. The anchor is a washer
    that spans the full plate stack height for robust contact detection."""
    # Anchor washer spans the full plate stack for reliable FCL contact
    anchor = (
        cq.Workplane("XY", origin=(0.0, 0.0, ZL0 - 0.0002))
        .circle(BOSS_R - 0.001)
        .extrude((ZU1 - ZL0) + 0.0004)
    )
    # Hollow center so it reads as a washer/ring around the rivet
    hole = (
        cq.Workplane("XY", origin=(0.0, 0.0, ZL0 - 0.0005))
        .circle(SHANK_R + 0.0002)
        .extrude((ZU1 - ZL0) + 0.001)
    )
    anchor = anchor.cut(hole)

    # Curved leaf strip from the anchor arcing between the handles
    spring_z = (ZL0 + ZU1) * 0.5
    spring_thick = 0.0010
    z0 = spring_z - spring_thick * 0.5
    z1 = spring_z + spring_thick * 0.5
    spring_pts = [
        (0.0000, -0.0016),
        (-0.0120, -0.0024),
        (-0.0280, -0.0028),
        (-0.0440, -0.0024),
        (-0.0560, -0.0010),
        (-0.0600, 0.0006),
        (-0.0560, 0.0020),
        (-0.0440, 0.0028),
        (-0.0280, 0.0026),
        (-0.0120, 0.0018),
        (0.0000, 0.0010),
    ]
    strip = _poly_prism(spring_pts, z0, z1)
    return anchor.union(strip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="needle_nose_pliers")

    steel = model.material("brushed_steel", rgba=(0.72, 0.73, 0.75, 1.0))
    polished = model.material("polished_steel", rgba=(0.86, 0.87, 0.90, 1.0))
    orange = model.material("orange_grip", rgba=(0.97, 0.52, 0.07, 1.0))
    peach = model.material("peach_inlay", rgba=(1.0, 0.80, 0.64, 0.78))
    spring_mat = model.material("spring_steel", rgba=(0.60, 0.62, 0.65, 1.0))

    # ----- lower half (base link): jaw at +y, handle at -y, lower steel layer
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
        name="neck",
    )
    lower.visual(
        mesh_from_cadquery(_spline_prism(_mirror(GRIP_PTS, +1.0), GRIP_LZ0, GRIP_LZ1), "lower_grip"),
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

    # ----- upper half (moving link): mirrored, upper steel layer, bored boss
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
        name="neck",
    )
    upper.visual(
        mesh_from_cadquery(_spline_prism(_mirror(GRIP_PTS, -1.0), GRIP_UZ0, GRIP_UZ1), "upper_grip"),
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

    # ----- return spring: separate part with mimic revolute joint
    # The spring arcs between the two handles. It rotates with the upper half
    # via a mimic articulation (multiplier=0.5, so it moves half the travel
    # of the upper half, representing the spring flex).
    spring = model.part("return_spring")
    spring.visual(
        mesh_from_cadquery(_return_spring(), "spring_leaf"),
        material=spring_mat,
        name="spring_leaf",
    )

    # ----- single revolute pivot at the rivet, axis perpendicular to the tool
    # plane. q=0 is the splayed rest pose; positive q closes the jaw tips
    # together while the handles scissor toward each other.
    pivot = model.articulation(
        "rivet_pivot",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, -HALF_OPEN)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=4.0, lower=0.0, upper=CLOSE_TRAVEL),
    )

    # ----- spring arc: mimic revolute that follows the pivot
    # The spring rotates half as much as the upper jaw, representing leaf
    # spring flex between the handles.
    model.articulation(
        "spring_arc",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=spring,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, -HALF_OPEN)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=4.0, lower=0.0, upper=HALF_OPEN),
        mimic=Mimic(joint="rivet_pivot", multiplier=0.5, offset=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower = object_model.get_part("lower_half")
    upper = object_model.get_part("upper_half")
    spring = object_model.get_part("return_spring")
    pivot = object_model.get_articulation("rivet_pivot")
    spring_joint = object_model.get_articulation("spring_arc")

    # --- intentional overlaps: spring washer rides on the pivot between halves
    ctx.allow_overlap(
        lower,
        spring,
        elem_a="rivet",
        elem_b="spring_leaf",
        reason="Spring anchor washer wraps around the rivet shank at the pivot.",
    )
    ctx.allow_overlap(
        lower,
        spring,
        elem_a="jaw",
        elem_b="spring_leaf",
        reason="Spring anchor washer rides at the pivot boss where the lower jaw meets the boss.",
    )
    ctx.allow_overlap(
        spring,
        upper,
        elem_a="spring_leaf",
        elem_b="jaw",
        reason="Spring washer sits between the two halves at the pivot boss region.",
    )

    # Proof: spring is centered on the pivot and overlaps the boss footprint
    ctx.expect_overlap(
        spring,
        lower,
        axes="xy",
        elem_a="spring_leaf",
        elem_b="pivot_boss",
        min_overlap=0.008,
        name="spring anchor overlaps boss footprint at pivot",
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
        name="jaw inner faces are open at rest",
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

    # --- needle-nose jaws are long and tapered (span well forward of pivot)
    for part in (lower, upper):
        jaw = ctx.part_element_world_aabb(part, elem="jaw")
        boss = ctx.part_element_world_aabb(part, elem="pivot_boss")
        ctx.check(
            f"{part.name} jaw extends well beyond boss (needle-nose)",
            jaw is not None
            and boss is not None
            and jaw[1][0] > boss[1][0] + 0.030,
            details=f"jaw_max_x={jaw[1][0]:.4f}, boss_max_x={boss[1][0]:.4f}",
        )

    # --- serrated teeth protrude inward past the jaw inner face centerline
    # The teeth extend past y=0 (center plane) so the two jaws' teeth
    # overlap slightly in the Y axis when measured at rest.
    lower_jaw = ctx.part_element_world_aabb(lower, elem="jaw")
    upper_jaw = ctx.part_element_world_aabb(upper, elem="jaw")
    ctx.check(
        "lower jaw teeth protrude inward past center",
        lower_jaw is not None and lower_jaw[0][1] < -0.0001,
        details=f"lower_jaw_min_y={lower_jaw[0][1]:.5f}" if lower_jaw else "no jaw",
    )
    ctx.check(
        "upper jaw teeth protrude inward past center",
        upper_jaw is not None and upper_jaw[1][1] > 0.0001,
        details=f"upper_jaw_max_y={upper_jaw[1][1]:.5f}" if upper_jaw else "no jaw",
    )

    # --- return spring is present and positioned between the handles
    spring_aabb = ctx.part_world_aabb(spring)
    ctx.check(
        "return spring exists with valid geometry",
        spring_aabb is not None and spring_aabb[1][0] - spring_aabb[0][0] > 0.02,
        details=f"spring_aabb={spring_aabb}",
    )

    # --- spring extends well behind the pivot into the handle region
    ctx.check(
        "spring arcs into the handle region",
        spring_aabb is not None and spring_aabb[0][0] < -0.04,
        details=f"spring_min_x={spring_aabb[0][0]:.4f}" if spring_aabb else "no spring",
    )

    # --- spring joint is a mimic of the pivot
    ctx.check(
        "spring_arc is a mimic of rivet_pivot",
        spring_joint.mimic is not None
        and spring_joint.mimic.joint == "rivet_pivot",
        details=f"mimic={spring_joint.mimic}",
    )

    # --- overall proportions: ~0.15 m long with needle nose
    la = ctx.part_world_aabb(lower)
    ua = ctx.part_world_aabb(upper)
    if la is not None and ua is not None:
        length = max(la[1][0], ua[1][0]) - min(la[0][0], ua[0][0])
        width = max(la[1][1], ua[1][1]) - min(la[0][1], ua[0][1])
        height = max(la[1][2], ua[1][2]) - min(la[0][2], ua[0][2])
        ctx.check(
            "overall length ~0.15 m",
            0.125 <= length <= 0.175,
            details=f"length={length:.4f}",
        )
        ctx.check(
            "splayed handle width reasonable",
            0.040 <= width <= 0.078,
            details=f"width={width:.4f}",
        )
        ctx.check(
            "flat tool thickness ~0.015 m",
            0.010 <= height <= 0.017,
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
        "handles scissor opposite to the jaws",
        open_grip is not None
        and closed_grip is not None
        and closed_grip[0][1] < open_grip[0][1] - 0.003,
        details=f"open={open_grip}, closed={closed_grip}",
    )

    # --- spring moves with the pivot (mimic articulation check)
    spring_rest = ctx.part_world_aabb(spring)
    with ctx.pose({pivot: CLOSE_TRAVEL}):
        spring_closed = ctx.part_world_aabb(spring)
    ctx.check(
        "spring arcs with handle motion (mimic active)",
        spring_rest is not None
        and spring_closed is not None
        and abs(spring_closed[0][1] - spring_rest[0][1]) > 0.001,
        details=f"rest={spring_rest}, closed={spring_closed}",
    )

    return ctx.report()


object_model = build_object_model()
