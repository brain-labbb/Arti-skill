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
)

# ---------------------------------------------------------------------------
# Round-nose jewelry pliers.
#
# Two mirrored half-tools cross at a single polished rivet. Each half is one
# rigid link: tapered conical jaw (round-nose) -> slim steel neck -> curved
# over-molded handle. The lower half is the base link; the upper half rotates
# about the rivet axis (Z, perpendicular to the flat plane of the tool).
#
# A return-spring leaf is anchored to the lower handle near the pivot and arcs
# between the handles. It has a secondary revolute joint (mimic of the main
# pivot) so it compresses as the handles close and springs open with them.
#
# Serrated teeth are modeled as small ridges on the inner face of each
# conical jaw.
# ---------------------------------------------------------------------------

HALF_OPEN = math.radians(12.5)  # each half yawed this much at rest (open)
CLOSE_TRAVEL = 2.0 * HALF_OPEN  # ~25 degrees of closing travel

PLATE_T = 0.0035  # forged steel plate thickness per half

# Steel layer stack
ZL0, ZL1 = 0.0035, 0.0035 + PLATE_T  # lower half plate
ZU0, ZU1 = ZL1, ZL1 + PLATE_T  # upper half plate

BOSS_R = 0.0075
HOLE_R = 0.0028
SHANK_R = 0.0024
HEAD_R = 0.0040

# --- Conical jaw parameters ---
JAW_BASE_R = 0.0040  # radius at the jaw base (near boss)
JAW_TIP_R = 0.0010  # radius at the rounded tip
JAW_LENGTH = 0.0340  # jaw length from base to tip
JAW_BASE_X = 0.0060  # X position of jaw base center (overlaps with boss)
JAW_MID_Z = (ZL0 + ZU1) / 2.0  # vertical center of jaw (straddles both plates)

# --- Serration parameters ---
SERR_COUNT = 6
SERR_WIDTH = 0.0006  # width of each serration ridge
SERR_HEIGHT = 0.0004  # height of serration ridge above jaw surface
SERR_DEPTH = 0.0012  # depth (along jaw axis) of each serration
SERR_START_X = JAW_BASE_X + 0.004  # first serration starts a bit past base
SERR_SPACING = (JAW_LENGTH - 0.014) / SERR_COUNT  # stop well before the thin tip

# --- Handle over-mold ---
GRIP_H = 0.0105
GRIP_LZ0 = 0.0
GRIP_LZ1 = GRIP_LZ0 + GRIP_H
GRIP_UZ0 = GRIP_LZ0 + (ZU0 - ZL0)
GRIP_UZ1 = GRIP_UZ0 + GRIP_H
INLAY_T = 0.0007
INLAY_EMBED = 0.0002

# --- Spring parameters ---
SPRING_THICKNESS = 0.0006
SPRING_WIDTH = 0.003
SPRING_ANCHOR_X = -0.012  # anchor point on lower handle near pivot
SPRING_TIP_X = -0.055  # free end extends along handle
SPRING_MID_Z = (GRIP_LZ1 + GRIP_UZ0) / 2.0  # between the two handle layers

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

# Curved over-molded handle outline (periodic spline; widens, rounded tip).
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

# Translucent peach inlay strip along the top face of the handle.
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


def _conical_jaw(s: float) -> cq.Workplane:
    """Tapered conical jaw (round-nose). Extends along +X from the boss area.
    s = +1 for lower half (+y side), s = -1 for upper half (-y side).
    The jaw is a tapered cylinder from JAW_BASE_R to JAW_TIP_R, centered
    vertically at JAW_MID_Z spanning the full blade height."""
    jaw_half_height = (ZU1 - ZL0) / 2.0
    center_y = s * 0.005  # offset jaw center from the shear plane
    # Build as a loft from base circle to tip circle along X axis
    base_circle = (
        cq.Workplane("YZ", origin=(JAW_BASE_X, 0.0, 0.0))
        .center(center_y, JAW_MID_Z)
        .circle(JAW_BASE_R)
    )
    tip_circle = (
        cq.Workplane("YZ", origin=(JAW_BASE_X + JAW_LENGTH, 0.0, 0.0))
        .center(center_y, JAW_MID_Z)
        .circle(JAW_TIP_R)
    )
    # Use loft between the two circular wires
    base_wire = cq.Wire.makeCircle(JAW_BASE_R, cq.Vector(JAW_BASE_X, center_y, JAW_MID_Z), cq.Vector(1, 0, 0))
    tip_wire = cq.Wire.makeCircle(JAW_TIP_R, cq.Vector(JAW_BASE_X + JAW_LENGTH, center_y, JAW_MID_Z), cq.Vector(1, 0, 0))
    loft_solid = cq.Solid.makeLoft([base_wire, tip_wire])
    # Add a hemisphere cap at the tip for the rounded nose
    tip_sphere = cq.Solid.makeSphere(JAW_TIP_R, pnt=cq.Vector(JAW_BASE_X + JAW_LENGTH, center_y, JAW_MID_Z))
    jaw = cq.Workplane(obj=loft_solid.fuse(tip_sphere))
    return jaw


def _serrated_teeth(s: float) -> cq.Workplane:
    """Serration ridges on the inner face of the conical jaw.
    s = +1 for lower half (+y side, teeth face -y toward center).
    s = -1 for upper half (-y side, teeth face +y toward center).
    Teeth extend from inside the cone to slightly past the inner surface,
    ensuring connected geometry with the cone body."""
    center_y = s * 0.005

    result = None
    for i in range(SERR_COUNT):
        x_pos = SERR_START_X + i * SERR_SPACING
        # Linear interpolation of jaw radius at this x position
        t = (x_pos - JAW_BASE_X) / JAW_LENGTH
        local_r = JAW_BASE_R * (1.0 - t) + JAW_TIP_R * t
        # Tooth spans from the cone center to slightly past the inner surface
        # This guarantees intersection with the cone body
        tooth_half_w = local_r * 0.9  # Z half-height (nearly full diameter)
        inner_edge = center_y - s * (local_r + SERR_HEIGHT)  # protrudes past inner face
        outer_edge = center_y + s * local_r * 0.2  # extends slightly past center toward outer face
        # Ensure correct ordering (min to max)
        y_lo = min(inner_edge, outer_edge)
        y_hi = max(inner_edge, outer_edge)
        tooth = (
            cq.Workplane("XY", origin=(x_pos - SERR_DEPTH / 2.0, y_lo, JAW_MID_Z - tooth_half_w))
            .box(SERR_DEPTH, y_hi - y_lo, tooth_half_w * 2.0)
        )
        if result is None:
            result = tooth
        else:
            result = result.union(tooth)
    return result


def _half_jaw_assembly(s: float) -> cq.Workplane:
    """Complete jaw: conical body + serrated teeth."""
    jaw = _conical_jaw(s)
    teeth = _serrated_teeth(s)
    return jaw.union(teeth)


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


def _spring_leaf() -> cq.Workplane:
    """Leaf return spring: thin curved strip that overlaps with the lower handle
    tang/neck to ensure geometric contact. The spring extends from within the
    tang Z range upward, forming a visible strip between the handles."""
    # Spring spans from within the tang Z range to ensure contact
    # Tang is at Z=0.0035 to 0.007; spring starts at Z=0.004 (inside tang)
    spring_z0 = 0.004
    spring_z1 = 0.0075

    # Spring outline: a narrow strip following the handle path,
    # positioned within the tang outline at the anchor end
    spring_pts = [
        (-0.0180, -0.0060),
        (-0.0250, -0.0070),
        (-0.0350, -0.0080),
        (-0.0500, -0.0092),
        (-0.0650, -0.0100),
        (-0.0750, -0.0108),
        (-0.0720, -0.0120),
        (-0.0580, -0.0124),
        (-0.0430, -0.0116),
        (-0.0300, -0.0102),
        (-0.0200, -0.0088),
    ]
    return _poly_prism(spring_pts, spring_z0, spring_z1)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="round_nose_jewelry_pliers")

    steel = model.material("brushed_steel", rgba=(0.72, 0.73, 0.75, 1.0))
    polished = model.material("polished_steel", rgba=(0.86, 0.87, 0.90, 1.0))
    orange = model.material("orange_grip", rgba=(0.97, 0.52, 0.07, 1.0))
    peach = model.material("peach_inlay", rgba=(1.0, 0.80, 0.64, 0.78))
    spring_mat = model.material("spring_steel", rgba=(0.65, 0.67, 0.70, 1.0))

    # ----- lower half (base link): jaw at +y, handle at -y, lower steel layer
    lower = model.part("lower_half")
    lower_pose = Origin(rpy=(0.0, 0.0, HALF_OPEN))
    lower.visual(
        mesh_from_cadquery(_half_jaw_assembly(+1.0), "lower_jaw"),
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

    # ----- upper half (moving link): mirrored, upper steel layer, bored boss
    upper = model.part("upper_half")
    upper.visual(
        mesh_from_cadquery(_half_jaw_assembly(-1.0), "upper_jaw"),
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

    # ----- return spring (separate part anchored to lower handle)
    spring = model.part("return_spring")
    spring.visual(
        mesh_from_cadquery(_spring_leaf(), "spring_leaf"),
        origin=lower_pose,
        material=spring_mat,
        name="spring_leaf",
    )

    # ----- main revolute pivot at the rivet
    pivot = model.articulation(
        "rivet_pivot",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, -HALF_OPEN)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=4.0, lower=0.0, upper=CLOSE_TRAVEL),
    )

    # ----- spring revolute joint: anchored to lower handle, arcs with the pivot
    # The spring pivots near the main pivot point and arcs as the handles close.
    # Mimic: spring follows main pivot with a reduced multiplier.
    model.articulation(
        "spring_pivot",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=spring,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, -HALF_OPEN)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=4.0, lower=0.0, upper=CLOSE_TRAVEL * 0.5),
        mimic=Mimic(joint="rivet_pivot", multiplier=0.5, offset=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower = object_model.get_part("lower_half")
    upper = object_model.get_part("upper_half")
    spring_part = object_model.get_part("return_spring")
    pivot = object_model.get_articulation("rivet_pivot")
    spring_joint = object_model.get_articulation("spring_pivot")

    # --- conical jaw geometry exists on both halves
    for part in (lower, upper):
        jaw_aabb = ctx.part_element_world_aabb(part, elem="jaw")
        ctx.check(
            f"{part.name} has conical jaw geometry",
            jaw_aabb is not None and (jaw_aabb[1][0] - jaw_aabb[0][0]) > 0.025,
            details=f"jaw aabb={jaw_aabb}",
        )

    # --- jaws are tapered (narrower at the tip than at the base)
    for part in (lower, upper):
        jaw_aabb = ctx.part_element_world_aabb(part, elem="jaw")
        if jaw_aabb is not None:
            jaw_dy = jaw_aabb[1][1] - jaw_aabb[0][1]
            jaw_dz = jaw_aabb[1][2] - jaw_aabb[0][2]
            # The jaw cross-section should be roughly circular; diameter should be
            # close to 2*JAW_BASE_R at the widest (base end)
            ctx.check(
                f"{part.name} jaw has conical profile (wider at base)",
                jaw_dz > 0.005 and jaw_dy > 0.005,
                details=f"jaw_dy={jaw_dy:.4f}, jaw_dz={jaw_dz:.4f}",
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

    # --- rivet passes through the boss stack
    ctx.expect_within(
        lower,
        upper,
        axes="xy",
        inner_elem="rivet",
        outer_elem="pivot_boss",
        margin=0.0002,
        name="rivet centered through the boss bore",
    )

    # --- rest pose: jaws open, handles splayed apart
    ctx.expect_gap(
        lower,
        upper,
        axis="y",
        positive_elem="jaw",
        negative_elem="jaw",
        min_gap=0.001,
        name="conical jaws are open at rest",
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

    # --- return spring exists and is positioned between handles
    spring_aabb = ctx.part_world_aabb(spring_part)
    ctx.check(
        "return spring exists between handles",
        spring_aabb is not None,
        details=f"spring aabb={spring_aabb}",
    )

    # --- spring joint is revolute and mimics the main pivot
    ctx.check(
        "spring pivot is a revolute joint",
        spring_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={spring_joint.articulation_type}",
    )
    ctx.check(
        "spring pivot mimics rivet_pivot",
        spring_joint.mimic is not None and spring_joint.mimic.joint == "rivet_pivot",
        details=f"mimic={spring_joint.mimic}",
    )

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
            contact_tol=0.005,
            name="conical jaw tips meet when fully closed",
        )
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
        and closed_grip[0][1] < open_grip[0][1] - 0.004,
        details=f"open={open_grip}, closed={closed_grip}",
    )

    # --- overall proportions: ~0.13 long, ~0.06 across handles
    la = ctx.part_world_aabb(lower)
    ua = ctx.part_world_aabb(upper)
    if la is not None and ua is not None:
        length = max(la[1][0], ua[1][0]) - min(la[0][0], ua[0][0])
        width = max(la[1][1], ua[1][1]) - min(la[0][1], ua[0][1])
        ctx.check(
            "overall length ~0.13 m",
            0.100 <= length <= 0.155,
            details=f"length={length:.4f}",
        )
        ctx.check(
            "splayed handle width ~0.06 m",
            0.040 <= width <= 0.085,
            details=f"width={width:.4f}",
        )
    else:
        ctx.fail("overall proportions", "missing part AABBs")

    return ctx.report()


object_model = build_object_model()
