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
# Needle-nose pliers (electronics precision pliers).
#
# Two mirrored half-tools cross at a single polished rivet. Each half is one
# rigid link: long tapered needle-nose jaw with serrated gripping teeth ->
# slim steel neck -> curved over-molded handle with textured grip ribs.
# The lower half is the base link; the upper half rotates about the rivet
# axis (Z, perpendicular to the flat plane of the tool).
#
# Geometry is authored per half in a "closed-design" local frame. The rest
# pose (q=0) splays each half by HALF_OPEN via yaw, so q in [0, CLOSE_TRAVEL]
# closes the jaws while the handles scissor together in opposition.
# ---------------------------------------------------------------------------

HALF_OPEN = math.radians(12.5)  # each half yawed this much at rest (open)
CLOSE_TRAVEL = 2.0 * HALF_OPEN  # ~25 degrees of closing travel

PLATE_T = 0.0035  # forged steel plate thickness per half

# Steel layer stack (lower plate below upper plate at the pivot boss).
ZL0, ZL1 = 0.0035, 0.0035 + PLATE_T  # lower half plate: 0.0035 .. 0.0070
ZU0, ZU1 = ZL1, ZL1 + PLATE_T  # upper half plate: 0.0070 .. 0.0105

# Full-height jaw region (jaw head spans both plate layers forward of boss).
BLADE_Z0, BLADE_Z1 = ZL0, ZU1
BLADE_X_MIN = 0.0085  # full-height jaw exists only forward of the boss

BOSS_R = 0.0075
HOLE_R = 0.0028
SHANK_R = 0.0024
HEAD_R = 0.0040

EDGE_LAND = 0.0003  # each gripping face sits this far off the center plane

# Handle over-mold z extents per half.
GRIP_H = 0.0105
GRIP_LZ0 = 0.0
GRIP_LZ1 = GRIP_LZ0 + GRIP_H
GRIP_UZ0 = GRIP_LZ0 + (ZU0 - ZL0)
GRIP_UZ1 = GRIP_UZ0 + GRIP_H

# Long needle-nose jaw profile (lower half at +y, closed-design frame).
# Tapers from ~7.5 mm half-width at the base to ~0.3 mm at the tip.
# Inner face is flat at y = EDGE_LAND for gripping; outer face curves inward.
# Base points extend back into the boss circle for geometric connectivity.
JAW_PTS = [
    (0.0030, 0.0060),   # base outer (inside boss radius for connectivity)
    (0.0080, 0.0075),
    (0.0130, 0.0065),
    (0.0200, 0.0052),
    (0.0280, 0.0040),
    (0.0350, 0.0030),
    (0.0410, 0.0020),
    (0.0460, 0.0012),
    (0.0500, 0.0006),
    (0.0520, 0.0003),
    (0.0500, EDGE_LAND),
    (0.0410, EDGE_LAND),
    (0.0350, EDGE_LAND),
    (0.0280, EDGE_LAND),
    (0.0200, EDGE_LAND),
    (0.0130, EDGE_LAND),
    (0.0080, 0.0028),
    (0.0030, 0.0030),   # base inner (inside boss radius for connectivity)
]

# Slim steel neck/tang from the boss back to the handle (lower half: -y side).
TANG_PTS = [
    (0.0020, -0.0030),
    (-0.0080, -0.0038),
    (-0.0180, -0.0044),
    (-0.0260, -0.0052),
    (-0.0260, -0.0090),
    (-0.0180, -0.0078),
    (-0.0080, -0.0066),
    (0.0008, -0.0058),
]

# Slender curved handle outline (periodic spline; widens, rounded tip).
GRIP_PTS = [
    (-0.0240, -0.0040),
    (-0.0360, -0.0048),
    (-0.0500, -0.0056),
    (-0.0640, -0.0064),
    (-0.0760, -0.0072),
    (-0.0860, -0.0080),
    (-0.0920, -0.0088),
    (-0.0940, -0.0094),
    (-0.0920, -0.0108),
    (-0.0820, -0.0120),
    (-0.0700, -0.0128),
    (-0.0560, -0.0126),
    (-0.0420, -0.0112),
    (-0.0300, -0.0090),
    (-0.0235, -0.0070),
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


def _blade_clip_box() -> cq.Workplane:
    """Clip region for full-height jaw head forward of the boss."""
    return cq.Workplane("XY", origin=(BLADE_X_MIN + 0.0220, 0.0, 0.0070)).box(
        0.0500, 0.0400, 0.0200
    )


def _half_jaw(s: float, own_z0: float, own_z1: float) -> cq.Workplane:
    """Needle-nose jaw: long tapered prism with flat inner gripping face."""
    prof = _mirror(JAW_PTS, s)
    full = _poly_prism(prof, BLADE_Z0, BLADE_Z1).intersect(_blade_clip_box())
    rear = _poly_prism(prof, own_z0, own_z1)
    return full.union(rear)


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


def _jaw_teeth(s: float, z0: float, z1: float) -> cq.Workplane:
    """Serrated teeth ridges on the inner gripping face of the jaw.

    Each tooth is a thin transverse ridge that protrudes from the flat inner
    face (y = s * EDGE_LAND) toward the center plane (y = 0). The tooth is
    embedded slightly into the jaw body for geometric connectivity.
    """
    tooth_w = 0.0006  # width along jaw length (x)
    tooth_d = 0.0004  # total depth: protrusion + embed
    embed = 0.0002    # embed into jaw body for mesh connectivity
    tooth_h = (z1 - z0) - 0.001  # nearly full blade height
    z_mid = (z0 + z1) / 2.0

    spacing = 0.003
    x_start = 0.014

    result = None
    i = 0
    while True:
        x = x_start + i * spacing
        if x > 0.048:
            break
        # Center tooth so it protrudes from jaw face but embeds into jaw body
        y_center = s * (EDGE_LAND - tooth_d / 2.0 + embed)
        tooth = (
            cq.Workplane("XY", origin=(x, y_center, z_mid))
            .box(tooth_w, tooth_d, tooth_h)
        )
        result = tooth if result is None else result.union(tooth)
        i += 1
    return result


def _grip_ribs(s: float, z_top: float) -> cq.Workplane:
    """Transverse textured ribs on the handle top surface.

    Each rib is a small raised ridge running across the handle width,
    positioned along the handle length. Ribs embed into the handle body
    for geometric connectivity.
    """
    rib_w = 0.0012  # width along handle length (x)
    rib_h = 0.0007  # total height (protrusion + embed)
    embed = 0.0003  # embed into handle for mesh connectivity

    # (x_position, y_width) for each rib along the handle
    rib_specs = [
        (-0.034, 0.005),
        (-0.042, 0.006),
        (-0.050, 0.007),
        (-0.058, 0.007),
        (-0.066, 0.007),
        (-0.074, 0.006),
        (-0.082, 0.005),
        (-0.089, 0.004),
    ]

    result = None
    for rx, ry_w in rib_specs:
        y_center = s * (-0.0085)
        z_center = z_top + rib_h / 2.0 - embed  # embed into handle body
        rib = (
            cq.Workplane("XY", origin=(rx, y_center, z_center))
            .box(rib_w, ry_w, rib_h)
        )
        result = rib if result is None else result.union(rib)
    return result


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="needle_nose_pliers")

    steel = model.material("brushed_steel", rgba=(0.72, 0.73, 0.75, 1.0))
    polished = model.material("polished_steel", rgba=(0.86, 0.87, 0.90, 1.0))
    orange = model.material("orange_grip", rgba=(0.97, 0.52, 0.07, 1.0))
    dark_rubber = model.material("grip_rubber", rgba=(0.25, 0.22, 0.20, 1.0))

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
        name="neck_tang",
    )
    lower.visual(
        mesh_from_cadquery(_soft_prism(_mirror(GRIP_PTS, +1.0), GRIP_LZ0, GRIP_LZ1), "lower_grip"),
        origin=lower_pose,
        material=orange,
        name="grip",
    )
    lower.visual(
        mesh_from_cadquery(_jaw_teeth(+1.0, BLADE_Z0, BLADE_Z1), "lower_teeth"),
        origin=lower_pose,
        material=steel,
        name="jaw_teeth",
    )
    lower.visual(
        mesh_from_cadquery(_grip_ribs(+1.0, GRIP_LZ1), "lower_ribs"),
        origin=lower_pose,
        material=dark_rubber,
        name="grip_ribs",
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
        name="neck_tang",
    )
    upper.visual(
        mesh_from_cadquery(_soft_prism(_mirror(GRIP_PTS, -1.0), GRIP_UZ0, GRIP_UZ1), "upper_grip"),
        material=orange,
        name="grip",
    )
    upper.visual(
        mesh_from_cadquery(_jaw_teeth(-1.0, BLADE_Z0, BLADE_Z1), "upper_teeth"),
        material=steel,
        name="jaw_teeth",
    )
    upper.visual(
        mesh_from_cadquery(_grip_ribs(-1.0, GRIP_UZ1), "upper_ribs"),
        material=dark_rubber,
        name="grip_ribs",
    )

    # ----- single revolute pivot at the rivet, axis perpendicular to the tool
    # plane. q=0 is the splayed rest pose; positive q closes the jaws
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

    lower = object_model.get_part("lower_half")
    upper = object_model.get_part("upper_half")
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

    # --- needle-nose jaws: long and tapered, extending well past the boss
    for part_obj, part_name in [(lower, "lower"), (upper, "upper")]:
        jaw_aabb = ctx.part_element_world_aabb(part_obj, elem="jaw")
        boss_aabb = ctx.part_element_world_aabb(part_obj, elem="pivot_boss")
        ctx.check(
            f"{part_name} jaw extends well past boss (needle-nose)",
            jaw_aabb is not None
            and boss_aabb is not None
            and jaw_aabb[1][0] > boss_aabb[1][0] + 0.030,
            details=f"jaw_tip_x={jaw_aabb[1][0]:.4f}, boss_edge_x={boss_aabb[1][0]:.4f}",
        )

    # --- serrated teeth geometry on both jaws
    for part_obj, part_name in [(lower, "lower"), (upper, "upper")]:
        teeth_aabb = ctx.part_element_world_aabb(part_obj, elem="jaw_teeth")
        ctx.check(
            f"{part_name} jaw has serrated teeth geometry",
            teeth_aabb is not None
            and (teeth_aabb[1][0] - teeth_aabb[0][0]) > 0.020,
            details=f"teeth={teeth_aabb}",
        )
        ctx.expect_overlap(
            part_obj,
            part_obj,
            axes="x",
            elem_a="jaw_teeth",
            elem_b="jaw",
            min_overlap=0.020,
            name=f"{part_name} teeth span the jaw gripping length",
        )

    # --- textured grip ribs on both handles
    for part_obj, part_name in [(lower, "lower"), (upper, "upper")]:
        ribs_aabb = ctx.part_element_world_aabb(part_obj, elem="grip_ribs")
        ctx.check(
            f"{part_name} handle has textured grip ribs",
            ribs_aabb is not None
            and (ribs_aabb[1][0] - ribs_aabb[0][0]) > 0.030,
            details=f"ribs={ribs_aabb}",
        )
        ctx.expect_overlap(
            part_obj,
            part_obj,
            axes="x",
            elem_a="grip_ribs",
            elem_b="grip",
            min_overlap=0.030,
            name=f"{part_name} ribs span the handle length",
        )

    # --- rest pose: jaws open, handles splayed apart
    ctx.expect_gap(
        lower,
        upper,
        axis="y",
        positive_elem="jaw",
        negative_elem="jaw",
        min_gap=0.002,
        name="jaw tips are open at rest",
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

    # --- overall proportions
    la = ctx.part_world_aabb(lower)
    ua = ctx.part_world_aabb(upper)
    if la is not None and ua is not None:
        length = max(la[1][0], ua[1][0]) - min(la[0][0], ua[0][0])
        width = max(la[1][1], ua[1][1]) - min(la[0][1], ua[0][1])
        ctx.check(
            "overall length ~0.13-0.16 m",
            0.12 <= length <= 0.17,
            details=f"length={length:.4f}",
        )
        ctx.check(
            "splayed handle width ~0.04-0.07 m",
            0.035 <= width <= 0.08,
            details=f"width={width:.4f}",
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

    return ctx.report()


object_model = build_object_model()
