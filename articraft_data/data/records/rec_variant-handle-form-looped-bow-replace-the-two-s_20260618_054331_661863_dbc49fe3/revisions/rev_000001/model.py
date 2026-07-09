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
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Flush-cut wire snips with bow / ring loop handles (scissor-style finger
# loops), about 0.13 m long overall.
#
# Two mirrored half-tools cross at a single polished rivet. Each half is one
# rigid link: tapered cutting jaw -> slim steel neck -> tapered arm -> closed
# bow finger ring. The lower half is the base link; the upper half rotates
# about the rivet axis (Z, perpendicular to the flat plane of the tool).
#
# Geometry is authored per half in a "closed-design" local frame in which the
# two halves are exactly mirrored about the XZ plane and the cutting edge
# lands sit at y = +/-EDGE_LAND. The rest pose (q=0) splays each half by
# HALF_OPEN via visual/joint-frame yaw, so q in [0, 2*HALF_OPEN] closes the
# blades while the handles scissor together in opposition.
# ---------------------------------------------------------------------------

HALF_OPEN = math.radians(12.5)  # each half is yawed this much at rest (open)
CLOSE_TRAVEL = 2.0 * HALF_OPEN  # ~25 degrees of closing travel

PLATE_T = 0.0035  # forged steel plate thickness per half

# Steel layer stack (lower plate sits below upper plate at the pivot boss;
# the upper plate rides directly on the lower plate, exact face contact).
ZL0, ZL1 = 0.0035, 0.0035 + PLATE_T  # lower half plate: 0.0035 .. 0.0070
ZU0, ZU1 = ZL1, ZL1 + PLATE_T  # upper half plate: 0.0070 .. 0.0105

# Full-height blade region (jaw head spans both plate layers, flat underside).
BLADE_Z0, BLADE_Z1 = ZL0, ZU1
BLADE_X_MIN = 0.0085  # full-height blade exists only forward of the boss

BOSS_R = 0.0075
HOLE_R = 0.0028  # clearance hole in the upper boss for the rivet shank
SHANK_R = 0.0024
HEAD_R = 0.0040

EDGE_LAND = 0.0003  # each cutting edge land sits this far off the shear plane

# Bow handle parameters (finger-loop rings replacing straight grips).
BOW_RING_R = 0.013       # major radius of finger ring torus
BOW_TUBE_R = 0.003       # tube cross-section radius
BOW_RING_CX = -0.072     # ring center X in local half frame
BOW_RING_CY = -0.008     # ring center Y in local half frame (before mirror)

# Bow arm Z extents (over-molded handle arm, same offset logic as original grip).
BOW_ARM_H = 0.0105
BOW_ARM_Z0_L = 0.0
BOW_ARM_Z1_L = BOW_ARM_Z0_L + BOW_ARM_H       # 0.0 .. 0.0105
BOW_ARM_Z0_U = BOW_ARM_Z0_L + (ZU0 - ZL0)     # 0.0035 .. 0.0140
BOW_ARM_Z1_U = BOW_ARM_Z0_U + BOW_ARM_H

# Jaw plate outline in the closed-design frame (lower half, jaw body at +y,
# cutting edge land along y = EDGE_LAND). Polyline, meters.
JAW_PTS = [
    (0.0000, 0.0090),
    (0.0060, 0.0094),
    (0.0140, 0.0086),
    (0.0240, 0.0060),
    (0.0330, 0.0028),
    (0.0352, 0.0012),
    (0.0346, EDGE_LAND),
    (0.0090, EDGE_LAND),
    (0.0020, 0.0048),
]

# Slim steel neck/tang from the boss back to the handle (lower half: -y side).
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

# Bow arm profile (over-molded handle arm connecting tang to ring).
# Tapers from the tang width to a narrower stem that enters the ring.
BOW_ARM_PTS = [
    (-0.024, -0.004),
    (-0.036, -0.005),
    (-0.048, -0.006),
    (-0.058, -0.007),
    (-0.063, -0.008),    # approaching ring, tip at ring center Y
    (-0.058, -0.009),
    (-0.048, -0.010),
    (-0.036, -0.011),
    (-0.024, -0.012),
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


def _bevel_wedge(s: float) -> cq.Workplane:
    """Material removed above the flat angled blade face (cut from the top,
    leaving a fine flush-cutting edge land at y = s*EDGE_LAND, z near the flat
    underside of the jaw head)."""
    tri = [
        (s * EDGE_LAND, BLADE_Z0 + 0.0004),
        (s * 0.0052, 0.0112),
        (s * EDGE_LAND, 0.0112),
    ]
    return (
        cq.Workplane("YZ", origin=(BLADE_X_MIN, 0.0, 0.0))
        .polyline(tri)
        .close()
        .extrude(0.0330)
    )


def _blade_clip_box() -> cq.Workplane:
    return cq.Workplane("XY", origin=(BLADE_X_MIN + 0.0250, 0.0, 0.0070)).box(
        0.0500, 0.0600, 0.0200
    )


def _half_blade(s: float, own_z0: float, own_z1: float) -> cq.Workplane:
    prof = _mirror(JAW_PTS, s)
    full = _poly_prism(prof, BLADE_Z0, BLADE_Z1).intersect(_blade_clip_box())
    rear = _poly_prism(prof, own_z0, own_z1)
    return full.union(rear).cut(_bevel_wedge(s))


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


def _bow_arm(s: float, z0: float, z1: float) -> cq.Workplane:
    """Over-molded handle arm connecting the steel tang to the finger ring.
    Uses a simple polygon extrusion to avoid spline-offset issues at the
    narrow tip; vertical edges are filleted for a soft over-mold look."""
    arm = _poly_prism(_mirror(BOW_ARM_PTS, s), z0, z1)
    try:
        arm = arm.edges("|Z").fillet(0.0012)
    except Exception:
        pass
    return arm


def _bow_ring(s: float, z0: float, z1: float):
    """Finger ring torus for the bow handle. Returns a TorusGeometry positioned
    at the ring center for the given half."""
    ring_cx = BOW_RING_CX
    ring_cy = s * BOW_RING_CY
    z_mid = (z0 + z1) / 2.0
    return TorusGeometry(
        radius=BOW_RING_R,
        tube=BOW_TUBE_R,
        radial_segments=20,
        tubular_segments=48,
    ).translate(ring_cx, ring_cy, z_mid)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flush_cut_bow_snips")

    steel = model.material("brushed_steel", rgba=(0.72, 0.73, 0.75, 1.0))
    polished = model.material("polished_steel", rgba=(0.86, 0.87, 0.90, 1.0))
    orange = model.material("orange_grip", rgba=(0.97, 0.52, 0.07, 1.0))

    # ----- lower half (base link): jaw at +y, handle at -y, lower steel layer
    lower = model.part("lower_cutter_half")
    lower_pose = Origin(rpy=(0.0, 0.0, HALF_OPEN))
    lower.visual(
        mesh_from_cadquery(_half_blade(+1.0, ZL0, ZL1), "lower_blade"),
        origin=lower_pose,
        material=steel,
        name="jaw_blade",
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
        mesh_from_cadquery(_bow_arm(+1.0, BOW_ARM_Z0_L, BOW_ARM_Z1_L), "lower_bow_arm"),
        origin=lower_pose,
        material=orange,
        name="bow_arm",
    )
    lower.visual(
        mesh_from_geometry(_bow_ring(+1.0, BOW_ARM_Z0_L, BOW_ARM_Z1_L), "lower_bow_ring"),
        origin=lower_pose,
        material=orange,
        name="bow_ring",
    )
    lower.visual(
        mesh_from_cadquery(_rivet(), "rivet"),
        origin=lower_pose,
        material=polished,
        name="rivet",
    )

    # ----- upper half (moving link): mirrored, upper steel layer, bored boss
    upper = model.part("upper_cutter_half")
    upper.visual(
        mesh_from_cadquery(_half_blade(-1.0, ZU0, ZU1), "upper_blade"),
        material=steel,
        name="jaw_blade",
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
        mesh_from_cadquery(_bow_arm(-1.0, BOW_ARM_Z0_U, BOW_ARM_Z1_U), "upper_bow_arm"),
        material=orange,
        name="bow_arm",
    )
    upper.visual(
        mesh_from_geometry(_bow_ring(-1.0, BOW_ARM_Z0_U, BOW_ARM_Z1_U), "upper_bow_ring"),
        material=orange,
        name="bow_ring",
    )

    # ----- single revolute pivot at the rivet, axis perpendicular to the tool
    # plane. q=0 is the splayed rest pose; positive q closes the blade edges
    # together while the handles scissor toward each other.
    model.articulation(
        "rivet_pivot",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, -HALF_OPEN)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=4.0, lower=0.0, upper=CLOSE_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower = object_model.get_part("lower_cutter_half")
    upper = object_model.get_part("upper_cutter_half")
    pivot = object_model.get_articulation("rivet_pivot")

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

    # --- rest pose: jaws open, bow handles splayed apart
    ctx.expect_gap(
        lower,
        upper,
        axis="y",
        positive_elem="jaw_blade",
        negative_elem="jaw_blade",
        min_gap=0.0025,
        name="blade edges are open at rest",
    )
    ctx.expect_gap(
        upper,
        lower,
        axis="y",
        positive_elem="bow_ring",
        negative_elem="bow_ring",
        min_gap=0.008,
        name="bow rings splay apart at rest",
    )

    # --- blades span the full forged head height with a flat flush underside
    for p in (lower, upper):
        blade = ctx.part_element_world_aabb(p, elem="jaw_blade")
        ctx.check(
            f"{p.name} blade spans both plate layers",
            blade is not None and blade[0][2] <= ZL0 + 0.0002 and blade[1][2] >= ZU1 - 0.0002,
            details=f"blade={blade}",
        )

    # --- bow rings have finger-loop proportions (substantial XY span, thin Z)
    for p in (lower, upper):
        ring = ctx.part_element_world_aabb(p, elem="bow_ring")
        ctx.check(
            f"{p.name} bow ring has finger-loop proportions",
            ring is not None
            and (ring[1][0] - ring[0][0]) > 0.020
            and (ring[1][1] - ring[0][1]) > 0.020
            and (ring[1][2] - ring[0][2]) < 0.010,
            details=f"ring={ring}",
        )

    # --- bow arm connects tang to ring (overlaps both in XY footprint)
    for p in (lower, upper):
        arm = ctx.part_element_world_aabb(p, elem="bow_arm")
        tang = ctx.part_element_world_aabb(p, elem="neck_tang")
        ring = ctx.part_element_world_aabb(p, elem="bow_ring")
        ctx.check(
            f"{p.name} bow arm overlaps tang in XY",
            arm is not None and tang is not None
            and arm[0][0] <= tang[1][0] + 0.001
            and arm[1][0] >= tang[0][0] - 0.001,
            details=f"arm={arm}, tang={tang}",
        )
        ctx.check(
            f"{p.name} bow arm overlaps ring in XY",
            arm is not None and ring is not None
            and arm[1][0] >= ring[0][0] - 0.001
            and arm[0][0] <= ring[1][0] + 0.001,
            details=f"arm={arm}, ring={ring}",
        )

    # --- overall proportions
    la = ctx.part_world_aabb(lower)
    ua = ctx.part_world_aabb(upper)
    if la is not None and ua is not None:
        length = max(la[1][0], ua[1][0]) - min(la[0][0], ua[0][0])
        width = max(la[1][1], ua[1][1]) - min(la[0][1], ua[0][1])
        height = max(la[1][2], ua[1][2]) - min(la[0][2], ua[0][2])
        ctx.check(
            "overall length ~0.10-0.15 m",
            0.100 <= length <= 0.150,
            details=f"length={length:.4f}",
        )
        ctx.check(
            "splayed bow handle width ~0.05-0.10 m",
            0.045 <= width <= 0.100,
            details=f"width={width:.4f}",
        )
        ctx.check(
            "flat tool thickness ~0.010-0.020 m",
            0.010 <= height <= 0.020,
            details=f"height={height:.4f}",
        )
    else:
        ctx.fail("overall proportions", "missing part AABBs")

    # --- articulation: positive q closes blades, handles scissor in opposition
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

    open_blade = ctx.part_element_world_aabb(upper, elem="jaw_blade")
    open_ring = ctx.part_element_world_aabb(upper, elem="bow_ring")
    with ctx.pose({pivot: CLOSE_TRAVEL}):
        ctx.expect_contact(
            lower,
            upper,
            elem_a="jaw_blade",
            elem_b="jaw_blade",
            contact_tol=0.0015,
            name="blade edges meet when fully closed",
        )
        closed_blade = ctx.part_element_world_aabb(upper, elem="jaw_blade")
        closed_ring = ctx.part_element_world_aabb(upper, elem="bow_ring")

    ctx.check(
        "closing swings upper jaw toward lower jaw",
        open_blade is not None
        and closed_blade is not None
        and closed_blade[1][1] > open_blade[1][1] + 0.004,
        details=f"open={open_blade}, closed={closed_blade}",
    )
    ctx.check(
        "bow handles scissor opposite to the jaws",
        open_ring is not None
        and closed_ring is not None
        and closed_ring[0][1] < open_ring[0][1] - 0.004,
        details=f"open={open_ring}, closed={closed_ring}",
    )

    return ctx.report()


object_model = build_object_model()
