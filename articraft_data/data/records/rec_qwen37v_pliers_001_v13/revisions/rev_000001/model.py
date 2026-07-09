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
# Compact lineman pliers with gripping teeth, cutter notch, return spring,
# and textured grip ribs.
#
# Two forged steel halves cross at a central pivot rivet. Each half is one
# rigid link: broad jaw head with serrated gripping teeth and a side-cutter
# notch -> steel neck/tang -> curved over-molded handle with transverse ribs.
#
# A leaf-spring arm is anchored to the lower handle and arcs toward the upper
# handle inner face. Its revolute articulation mimics the main pivot (inverted)
# so the spring deflects as the handles close.
#
# Geometry is authored per half in a "closed-design" local frame where the
# two halves are mirrored about the XZ plane. The rest pose (q=0) splays
# each half by HALF_OPEN; positive q closes the jaws while the handles
# scissor together.
# ---------------------------------------------------------------------------

HALF_OPEN = math.radians(12.5)  # each half yawed this much at rest
CLOSE_TRAVEL = 2.0 * HALF_OPEN  # ~25 degrees total closing travel

PLATE_T = 0.004  # forged steel plate thickness per half (slightly beefier)

# Steel layer stack
ZL0, ZL1 = 0.004, 0.004 + PLATE_T   # lower: 0.004..0.008
ZU0, ZU1 = ZL1, ZL1 + PLATE_T       # upper: 0.008..0.012

# Full-height jaw region
JAW_Z0, JAW_Z1 = ZL0, ZU1
JAW_X_MIN = 0.008  # full-height jaw exists forward of boss

BOSS_R = 0.009
HOLE_R = 0.003
SHANK_R = 0.0026
HEAD_R = 0.0045

# Jaw dimensions for lineman pliers (broad, blocky)
JAW_LENGTH = 0.032  # jaw extends forward from pivot
JAW_WIDTH = 0.016   # width of jaw face (gripping direction)
JAW_TIP_W = 0.012   # slightly narrower at tip

# Teeth parameters
TEETH_COUNT = 8
TEETH_DEPTH = 0.0008
TEETH_WIDTH = 0.0012

# Cutter notch (near pivot)
NOTCH_DEPTH = 0.004
NOTCH_WIDTH = 0.006

# Handle
GRIP_H = 0.012
GRIP_LZ0 = 0.0
GRIP_LZ1 = GRIP_LZ0 + GRIP_H
GRIP_UZ0 = GRIP_LZ0 + (ZU0 - ZL0)
GRIP_UZ1 = GRIP_UZ0 + GRIP_H

# Grip ribs
RIB_COUNT = 10
RIB_HEIGHT = 0.0008
RIB_WIDTH = 0.0018

EDGE_LAND = 0.0003

# Jaw plate outline (lower half, +y side for gripping jaw)
# Broad rectangular jaw with slight taper at tip
JAW_PTS = [
    (0.0000, 0.0110),
    (0.0050, 0.0120),
    (0.0150, 0.0125),
    (0.0250, 0.0115),
    (0.0320, 0.0090),
    (0.0350, 0.0060),
    (0.0340, 0.0030),
    (0.0310, EDGE_LAND),
    (0.0090, EDGE_LAND),
    (0.0020, 0.0055),
]

# Tang from boss back to handle
TANG_PTS = [
    (0.0020, -0.0040),
    (-0.0120, -0.0052),
    (-0.0240, -0.0062),
    (-0.0340, -0.0074),
    (-0.0340, -0.0128),
    (-0.0240, -0.0108),
    (-0.0120, -0.0094),
    (0.0008, -0.0082),
]

# Curved handle outline (wider, beefier for lineman pliers)
GRIP_PTS = [
    (-0.0280, -0.0045),
    (-0.0450, -0.0058),
    (-0.0640, -0.0072),
    (-0.0820, -0.0086),
    (-0.0980, -0.0096),
    (-0.1060, -0.0112),
    (-0.1020, -0.0150),
    (-0.0880, -0.0172),
    (-0.0720, -0.0178),
    (-0.0540, -0.0162),
    (-0.0380, -0.0138),
    (-0.0270, -0.0110),
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
    """Extruded spline outline with loft-capped softly beveled top/bottom."""
    mid = cq.Solid.extrudeLinear(
        cq.Face.makeFromWires(_spline_wire(pts, z0 + cap)),
        cq.Vector(0.0, 0.0, (z1 - z0) - 2.0 * cap),
    )
    top = cq.Solid.makeLoft([_spline_wire(pts, z1 - cap), _spline_wire(pts, z1, inset=inset)])
    bot = cq.Solid.makeLoft([_spline_wire(pts, z0, inset=inset), _spline_wire(pts, z0 + cap)])
    return cq.Workplane(obj=mid.fuse(top).fuse(bot))


def _jaw_body(s: float, own_z0: float, own_z1: float) -> cq.Workplane:
    """Broad jaw head for lineman pliers - full height at front, own plate at rear.
    Includes a cutter notch cutout near the pivot."""
    prof = _mirror(JAW_PTS, s)
    full = _poly_prism(prof, JAW_Z0, JAW_Z1)
    # Clip to forward region
    clip = cq.Workplane("XY", origin=(JAW_X_MIN + 0.020, 0.0, 0.007)).box(
        0.058, 0.060, 0.024
    )
    full = full.intersect(clip)
    rear = _poly_prism(prof, own_z0, own_z1)
    jaw = full.union(rear)
    # Cut the side-cutter notch near the pivot
    jaw = jaw.cut(_cutter_notch_cutter(s))
    return jaw


def _serrated_teeth(s: float, z_face: float, z_height: float) -> cq.Workplane:
    """Serrated gripping teeth on the inner jaw face.
    
    Creates small transverse ridges across the jaw gripping surface.
    Teeth are embedded into the jaw face (half inside, half protruding)
    so they connect to the parent geometry.
    s: mirror sign (+1 for lower jaw at +y, -1 for upper at -y)
    z_face: z of the bottom face
    z_height: height of the tooth zone
    """
    # Teeth embedded into inner face of jaw - they straddle the surface
    tooth_y_center = s * (EDGE_LAND + 0.002)  # center deeper into jaw body
    # Distribute teeth along jaw, staying well within jaw body
    tooth_x_start = 0.012
    tooth_x_end = 0.030  # stop before jaw tip narrows too much
    tooth_spacing = (tooth_x_end - tooth_x_start) / max(TEETH_COUNT - 1, 1)
    # Full plate height so teeth span the jaw vertically and embed into it
    tooth_z_center = z_face + z_height * 0.5
    
    first = True
    result = None
    for i in range(TEETH_COUNT):
        x_pos = tooth_x_start + i * tooth_spacing
        # Each tooth is a ridge that spans the full jaw height and protrudes inward
        tooth = (
            cq.Workplane("XY", origin=(x_pos, tooth_y_center, tooth_z_center))
            .box(TEETH_WIDTH, TEETH_DEPTH * 3, z_height * 1.1)
        )
        if first:
            result = tooth
            first = False
        else:
            result = result.union(tooth)
    
    if result is None:
        result = cq.Workplane("XY").box(0.001, 0.001, 0.001)
    return result


def _cutter_notch_cutter(s: float) -> cq.Workplane:
    """V-shaped cutter notch cutout near the pivot on each jaw."""
    # Triangular notch cut from the inner edge of the jaw near pivot
    notch_pts = [
        (0.010, s * EDGE_LAND),
        (0.010, s * (EDGE_LAND + NOTCH_DEPTH)),
        (0.010 + NOTCH_WIDTH, s * EDGE_LAND),
    ]
    return (
        cq.Workplane("XY", origin=(0.0, 0.0, JAW_Z0 - 0.001))
        .polyline(notch_pts)
        .close()
        .extrude((JAW_Z1 - JAW_Z0) + 0.002)
    )


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
    lower_head = cq.Workplane("XY", origin=(0.0, 0.0, 0.0025)).circle(HEAD_R).extrude(0.002)
    shank = cq.Workplane("XY", origin=(0.0, 0.0, 0.0025)).circle(SHANK_R).extrude(0.012)
    upper_head = cq.Workplane("XY", origin=(0.0, 0.0, ZU1 + 0.0001)).circle(HEAD_R).extrude(
        0.0016
    )
    rivet = lower_head.union(shank).union(upper_head)
    try:
        rivet = rivet.edges(">Z").fillet(0.001)
    except Exception:
        pass
    return rivet


def _grip_ribs(s: float, z_center: float, z_span: float) -> cq.Workplane:
    """Transverse grip ribs on the handle outer surface.
    
    Ribs span the full handle height (embedding into grip) and protrude
    outward, so they connect to the parent grip geometry.
    s: mirror sign
    z_center: z center of the rib zone (midpoint of grip height)
    z_span: total z span of each rib (should be >= grip height for embedding)
    """
    # Rib positions along handle length (on outer face of handle)
    rib_positions = [
        (-0.040, -0.0085),
        (-0.048, -0.0092),
        (-0.056, -0.0100),
        (-0.064, -0.0106),
        (-0.072, -0.0112),
        (-0.080, -0.0116),
        (-0.088, -0.0118),
        (-0.096, -0.0118),
        (-0.088, -0.0155),
        (-0.078, -0.0162),
    ]
    
    first = True
    result = None
    for (cx, cy) in rib_positions[:RIB_COUNT]:
        rib_y = s * cy  # on outer face of handle
        rib = (
            cq.Workplane("XY", origin=(cx, rib_y, z_center))
            .box(RIB_WIDTH, 0.005, z_span)
        )
        if first:
            result = rib
            first = False
        else:
            result = result.union(rib)
    
    if result is None:
        result = cq.Workplane("XY").box(0.001, 0.001, 0.001)
    return result


def _leaf_spring() -> cq.Workplane:
    """Leaf return spring anchored in the lower handle, arcing toward the upper handle.
    
    The spring base is embedded into the lower handle grip (overlap ensures
    connectivity), and the free end arcs across the gap toward the upper handle.
    Modeled in the lower-half local frame.
    """
    # Spring extends from deep inside the lower handle grip (-y side, at -0.010)
    # outward across the gap toward where upper handle is (+y direction to ~0.004)
    # The base at y~-0.010 is embedded in the lower grip body.
    spring_pts = [
        (-0.034, -0.010),   # anchor deep in lower grip
        (-0.034, -0.007),
        (-0.042, -0.002),
        (-0.052, 0.002),
        (-0.062, 0.004),
        (-0.070, 0.004),
        (-0.074, 0.003),
        (-0.074, 0.001),
        (-0.068, -0.001),
        (-0.058, -0.003),
        (-0.048, -0.006),
        (-0.038, -0.009),
    ]
    return _poly_prism(spring_pts, ZL0 - 0.0005, ZL1 + 0.001)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lineman_pliers")

    steel = model.material("brushed_steel", rgba=(0.72, 0.73, 0.75, 1.0))
    polished = model.material("polished_steel", rgba=(0.86, 0.87, 0.90, 1.0))
    orange = model.material("orange_grip", rgba=(0.95, 0.45, 0.05, 1.0))
    dark_grip = model.material("dark_grip", rgba=(0.18, 0.18, 0.20, 1.0))
    spring_steel = model.material("spring_steel", rgba=(0.60, 0.62, 0.65, 1.0))

    # ----- lower half (base link): jaw at +y, handle at -y
    lower = model.part("lower_half")
    lower_pose = Origin(rpy=(0.0, 0.0, HALF_OPEN))

    # Jaw body
    lower.visual(
        mesh_from_cadquery(_jaw_body(+1.0, ZL0, ZL1), "lower_jaw"),
        origin=lower_pose,
        material=steel,
        name="jaw",
    )
    # Serrated teeth on lower jaw inner face
    lower.visual(
        mesh_from_cadquery(_serrated_teeth(+1.0, ZL0, ZL1 - ZL0), "lower_teeth"),
        origin=lower_pose,
        material=steel,
        name="jaw_teeth",
    )
    # Pivot boss
    lower.visual(
        mesh_from_cadquery(_half_boss(ZL0, ZL1, with_hole=False), "lower_boss"),
        origin=lower_pose,
        material=steel,
        name="pivot_boss",
    )
    # Neck/tang
    lower.visual(
        mesh_from_cadquery(_poly_prism(_mirror(TANG_PTS, +1.0), ZL0, ZL1), "lower_tang"),
        origin=lower_pose,
        material=steel,
        name="neck_tang",
    )
    # Handle grip
    lower.visual(
        mesh_from_cadquery(_soft_prism(_mirror(GRIP_PTS, +1.0), GRIP_LZ0, GRIP_LZ1), "lower_grip"),
        origin=lower_pose,
        material=orange,
        name="grip",
    )
    # Grip ribs on lower handle (span full grip height, embedding into grip body)
    lower.visual(
        mesh_from_cadquery(
            _grip_ribs(+1.0, (GRIP_LZ0 + GRIP_LZ1) * 0.5, GRIP_H * 1.08),
            "lower_ribs",
        ),
        origin=lower_pose,
        material=dark_grip,
        name="grip_ribs",
    )
    # Rivet
    lower.visual(
        mesh_from_cadquery(_rivet(), "rivet"),
        origin=lower_pose,
        material=polished,
        name="rivet",
    )

    # ----- upper half (moving link): mirrored, upper steel layer
    upper = model.part("upper_half")
    # Upper jaw body
    upper.visual(
        mesh_from_cadquery(_jaw_body(-1.0, ZU0, ZU1), "upper_jaw"),
        material=steel,
        name="jaw",
    )
    # Serrated teeth on upper jaw inner face
    upper.visual(
        mesh_from_cadquery(_serrated_teeth(-1.0, ZU0, ZU1 - ZU0), "upper_teeth"),
        material=steel,
        name="jaw_teeth",
    )
    # Pivot boss with hole
    upper.visual(
        mesh_from_cadquery(_half_boss(ZU0, ZU1, with_hole=True), "upper_boss"),
        material=steel,
        name="pivot_boss",
    )
    # Neck/tang
    upper.visual(
        mesh_from_cadquery(_poly_prism(_mirror(TANG_PTS, -1.0), ZU0, ZU1), "upper_tang"),
        material=steel,
        name="neck_tang",
    )
    # Handle grip
    upper.visual(
        mesh_from_cadquery(_soft_prism(_mirror(GRIP_PTS, -1.0), GRIP_UZ0, GRIP_UZ1), "upper_grip"),
        material=orange,
        name="grip",
    )
    # Grip ribs on upper handle (span full grip height, embedding into grip body)
    upper.visual(
        mesh_from_cadquery(
            _grip_ribs(-1.0, (GRIP_UZ0 + GRIP_UZ1) * 0.5, GRIP_H * 1.08),
            "upper_ribs",
        ),
        material=dark_grip,
        name="grip_ribs",
    )

    # ----- leaf spring arm (separate part, mimic-driven)
    # Spring base is embedded into the lower grip (intentional overlap for
    # anchoring); free end arcs across the gap toward the upper handle.
    spring = model.part("spring_arm")
    spring.visual(
        mesh_from_cadquery(_leaf_spring(), "spring_leaf"),
        origin=Origin(rpy=(0.0, 0.0, 2.0 * HALF_OPEN)),
        material=spring_steel,
        name="spring_leaf",
    )

    # ----- main pivot: revolute at rivet, Z axis perpendicular to tool plane
    model.articulation(
        "rivet_pivot",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, -HALF_OPEN)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=4.0, lower=0.0, upper=CLOSE_TRAVEL),
    )

    # ----- spring pivot: revolute mimic of main pivot (inverted)
    # The spring deflects opposite to handle closure: when handles close (q increases),
    # the spring compresses (its angle decreases from rest toward closed).
    model.articulation(
        "spring_pivot",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=spring,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, -HALF_OPEN)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=4.0, lower=0.0, upper=CLOSE_TRAVEL * 0.5),
        mimic=Mimic(joint="rivet_pivot", multiplier=0.5, offset=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower = object_model.get_part("lower_half")
    upper = object_model.get_part("upper_half")
    spring = object_model.get_part("spring_arm")
    pivot = object_model.get_articulation("rivet_pivot")
    spring_j = object_model.get_articulation("spring_pivot")

    # --- pivot stack: bosses coaxial, upper sits slightly above lower
    ctx.expect_overlap(
        lower,
        upper,
        axes="xy",
        elem_a="pivot_boss",
        elem_b="pivot_boss",
        min_overlap=0.014,
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

    # --- rivet centered through boss bore
    ctx.expect_within(
        lower,
        upper,
        axes="xy",
        inner_elem="rivet",
        outer_elem="pivot_boss",
        margin=0.0002,
        name="rivet centered through the boss bore",
    )

    # --- jaws: serrated teeth exist on both halves
    for p, teeth_name in [(lower, "jaw_teeth"), (upper, "jaw_teeth")]:
        teeth_aabb = ctx.part_element_world_aabb(p, elem=teeth_name)
        ctx.check(
            f"{p.name} has serrated jaw teeth",
            teeth_aabb is not None and (teeth_aabb[1][0] - teeth_aabb[0][0]) > 0.015,
            details=f"teeth_aabb={teeth_aabb}",
        )

    # --- grip ribs exist on both handles
    for p in (lower, upper):
        ribs_aabb = ctx.part_element_world_aabb(p, elem="grip_ribs")
        ctx.check(
            f"{p.name} has textured grip ribs",
            ribs_aabb is not None and (ribs_aabb[1][0] - ribs_aabb[0][0]) > 0.025,
            details=f"ribs_aabb={ribs_aabb}",
        )

    # --- spring arm exists between handles
    spring_aabb = ctx.part_world_aabb(spring)
    ctx.check(
        "return spring arm present",
        spring_aabb is not None and (spring_aabb[1][0] - spring_aabb[0][0]) > 0.020,
        details=f"spring_aabb={spring_aabb}",
    )

    # --- spring base is anchored into the lower grip (intentional overlap)
    ctx.allow_overlap(
        lower,
        spring,
        elem_a="grip",
        elem_b="spring_leaf",
        reason="The spring leaf base is intentionally embedded into the lower handle grip for anchoring.",
    )
    ctx.expect_overlap(
        lower,
        spring,
        axes="xy",
        elem_a="grip",
        elem_b="spring_leaf",
        min_overlap=0.005,
        name="spring base anchored within lower grip footprint",
    )

    # --- spring pivot is a mimic of the main pivot
    ctx.check(
        "spring pivot mimics main pivot",
        spring_j.mimic is not None and spring_j.mimic.joint == "rivet_pivot",
        details=f"mimic={spring_j.mimic}",
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

    # --- overall proportions (lineman pliers: ~0.17m long, ~0.08m wide, ~0.018m thick)
    la = ctx.part_world_aabb(lower)
    ua = ctx.part_world_aabb(upper)
    if la is not None and ua is not None:
        length = max(la[1][0], ua[1][0]) - min(la[0][0], ua[0][0])
        width = max(la[1][1], ua[1][1]) - min(la[0][1], ua[0][1])
        height = max(la[1][2], ua[1][2]) - min(la[0][2], ua[0][2])
        ctx.check(
            "overall length ~0.15-0.20 m",
            0.13 <= length <= 0.22,
            details=f"length={length:.4f}",
        )
        ctx.check(
            "splayed handle width ~0.06-0.10 m",
            0.05 <= width <= 0.12,
            details=f"width={width:.4f}",
        )
        ctx.check(
            "flat tool thickness ~0.015-0.025 m",
            0.012 <= height <= 0.028,
            details=f"height={height:.4f}",
        )
    else:
        ctx.fail("overall proportions", "missing part AABBs")

    # --- articulation: positive q closes jaws, handles scissor
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

    # --- spring moves with the pivot (mimic check)
    spring_open = ctx.part_world_aabb(spring)
    with ctx.pose({pivot: CLOSE_TRAVEL}):
        spring_closed = ctx.part_world_aabb(spring)
    ctx.check(
        "spring arm deflects as handles close",
        spring_open is not None
        and spring_closed is not None
        and abs(spring_closed[0][1] - spring_open[0][1]) > 0.001,
        details=f"open={spring_open}, closed={spring_closed}",
    )

    # --- broad jaw width distinguishes lineman pliers from snips
    jaw_aabb = ctx.part_element_world_aabb(lower, elem="jaw")
    if jaw_aabb is not None:
        jaw_y_span = jaw_aabb[1][1] - jaw_aabb[0][1]
        ctx.check(
            "broad lineman jaw width >= 0.012 m",
            jaw_y_span >= 0.012,
            details=f"jaw_y_span={jaw_y_span:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
