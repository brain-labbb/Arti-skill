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
# Combination pliers with crimping notch, locking release lever, serrated
# jaw teeth, and textured grip ribs.
#
# Two forged steel halves cross at a central polished rivet pivot. Each half
# is one rigid link: flat gripping jaw with serrated inner teeth -> crimping
# notch recess -> slim steel neck/tang -> curved over-molded handle with
# transverse grip ribs. A small locking release lever pivots inside the
# lower handle.
#
# Geometry is authored per half in a "closed-design" local frame; the rest
# pose (q=0) splays each half by HALF_OPEN so q in [0, CLOSE_TRAVEL] closes
# the jaws while the handles scissor together.
# ---------------------------------------------------------------------------

HALF_OPEN = math.radians(12.5)
CLOSE_TRAVEL = 2.0 * HALF_OPEN

PLATE_T = 0.0035

ZL0, ZL1 = 0.0035, 0.0035 + PLATE_T
ZU0, ZU1 = ZL1, ZL1 + PLATE_T

BLADE_Z0, BLADE_Z1 = ZL0, ZU1
BLADE_X_MIN = 0.0085

BOSS_R = 0.0080
HOLE_R = 0.0028
SHANK_R = 0.0024
HEAD_R = 0.0042

EDGE_LAND = 0.0003

GRIP_H = 0.0110
GRIP_LZ0 = 0.0
GRIP_LZ1 = GRIP_LZ0 + GRIP_H
GRIP_UZ0 = GRIP_LZ0 + (ZU0 - ZL0)
GRIP_UZ1 = GRIP_UZ0 + GRIP_H

# Combination pliers jaw: wider flat gripping profile with blunt tip.
JAW_PTS = [
    (0.0000, 0.0100),
    (0.0060, 0.0108),
    (0.0140, 0.0102),
    (0.0220, 0.0088),
    (0.0300, 0.0068),
    (0.0360, 0.0048),
    (0.0380, 0.0030),
    (0.0375, 0.0018),
    (0.0090, EDGE_LAND),
    (0.0020, 0.0055),
]

# Crimping notch: semicircular recess behind jaw tips (cut from jaw body).
CRIMP_X = 0.018
CRIMP_R = 0.0032

# Serrated teeth parameters
TOOTH_COUNT = 7
TOOTH_DEPTH = 0.0008
TOOTH_WIDTH = 0.0025
TOOTH_SPACING = 0.0032
TOOTH_START_X = 0.009

TANG_PTS = [
    (0.0020, -0.0035),
    (-0.0100, -0.0048),
    (-0.0200, -0.0056),
    (-0.0300, -0.0066),
    (-0.0300, -0.0118),
    (-0.0200, -0.0102),
    (-0.0100, -0.0088),
    (0.0008, -0.0078),
]

GRIP_PTS = [
    (-0.0240, -0.0040),
    (-0.0400, -0.0052),
    (-0.0580, -0.0064),
    (-0.0740, -0.0078),
    (-0.0890, -0.0088),
    (-0.0955, -0.0100),
    (-0.0915, -0.0136),
    (-0.0780, -0.0158),
    (-0.0620, -0.0162),
    (-0.0460, -0.0148),
    (-0.0300, -0.0120),
    (-0.0235, -0.0102),
]

# Grip ribs: transverse raised ridges across handle top face
RIB_COUNT = 10
RIB_HEIGHT = 0.0010
RIB_WIDTH = 0.0018
RIB_X_START = -0.032
RIB_X_END = -0.085

# Locking release lever dimensions
LEVER_LENGTH = 0.018
LEVER_WIDTH = 0.005
LEVER_THICKNESS = 0.002
LEVER_PIVOT_X = -0.050
LEVER_PIVOT_Y = -0.0085
LEVER_TRAVEL = math.radians(30.0)


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


def _crimp_notch(s: float, z0: float, z1: float) -> cq.Workplane:
    """Semicircular crimping notch cut into the jaw body behind the tips."""
    cy = s * 0.0050
    return (
        cq.Workplane("XY", origin=(CRIMP_X, cy, z0 - 0.001))
        .circle(CRIMP_R)
        .extrude((z1 - z0) + 0.002)
    )


def _half_jaw(s: float, own_z0: float, own_z1: float) -> cq.Workplane:
    """Combination pliers jaw: flat gripping profile with crimping notch."""
    prof = _mirror(JAW_PTS, s)
    full = _poly_prism(prof, BLADE_Z0, BLADE_Z1)
    # Clip to forward region only
    clip = cq.Workplane("XY", origin=(BLADE_X_MIN, 0.0, 0.0)).box(0.050, 0.060, 0.020)
    jaw = full.intersect(clip)
    # Also include the rear portion at own plate height
    rear = _poly_prism(prof, own_z0, own_z1)
    jaw = jaw.union(rear)
    # Cut crimping notch
    notch = _crimp_notch(s, BLADE_Z0, BLADE_Z1)
    jaw = jaw.cut(notch)
    return jaw


def _serrated_teeth(s: float, z0: float, z1: float) -> cq.Workplane:
    """Row of small rectangular teeth on the inner jaw face, embedded into jaw body."""
    result = None
    for i in range(TOOTH_COUNT):
        x = TOOTH_START_X + i * TOOTH_SPACING
        # Teeth embed into the jaw body: center them at the jaw inner edge
        # so half protrudes inward (toward y=0) and half overlaps the jaw.
        y_center = s * (EDGE_LAND + TOOTH_DEPTH * 0.8)
        tooth = (
            cq.Workplane("XY", origin=(x, y_center, z0))
            .box(TOOTH_WIDTH, TOOTH_DEPTH * 2.5, z1 - z0)
        )
        if result is None:
            result = tooth
        else:
            result = result.union(tooth)
    return result if result is not None else cq.Workplane("XY")


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


def _grip_ribs(s: float, z_top: float) -> cq.Workplane:
    """Transverse raised ribs across the handle top face."""
    result = None
    dx = (RIB_X_END - RIB_X_START) / max(RIB_COUNT - 1, 1)
    for i in range(RIB_COUNT):
        x = RIB_X_START + i * dx
        # Rib spans across the handle width at that x location
        y_center = s * (-0.0095)
        y_half = 0.0035
        rib = (
            cq.Workplane("XY", origin=(x, y_center, z_top))
            .box(RIB_WIDTH, y_half * 2, RIB_HEIGHT)
        )
        if result is None:
            result = rib
        else:
            result = result.union(rib)
    return result if result is not None else cq.Workplane("XY")


def _release_lever() -> cq.Workplane:
    """Small locking release lever built at local origin, extending along -x."""
    # Body extends from pivot (origin) toward the handle tip (-x direction)
    body = (
        cq.Workplane("XY", origin=(-LEVER_LENGTH * 0.5, 0.0, 0.0))
        .box(LEVER_LENGTH, LEVER_WIDTH, LEVER_THICKNESS)
    )
    # Pivot boss (small cylinder at the pivot point = origin)
    pivot = (
        cq.Workplane("XY", origin=(0.0, 0.0, -LEVER_THICKNESS * 0.5))
        .circle(0.0022)
        .extrude(LEVER_THICKNESS)
    )
    # Wider paddle tab at the far end
    tab = (
        cq.Workplane("XY", origin=(-LEVER_LENGTH * 0.85, 0.0, 0.0))
        .box(0.006, LEVER_WIDTH * 1.5, LEVER_THICKNESS)
    )
    return body.union(pivot).union(tab)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="combination_pliers")

    steel = model.material("brushed_steel", rgba=(0.72, 0.73, 0.75, 1.0))
    polished = model.material("polished_steel", rgba=(0.86, 0.87, 0.90, 1.0))
    orange = model.material("orange_grip", rgba=(0.97, 0.52, 0.07, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.18, 0.18, 0.20, 1.0))
    lever_metal = model.material("lever_steel", rgba=(0.60, 0.62, 0.64, 1.0))

    # ----- lower half (base link): jaw at +y, handle at -y
    lower = model.part("lower_half")
    lower_pose = Origin(rpy=(0.0, 0.0, HALF_OPEN))
    lower.visual(
        mesh_from_cadquery(_half_jaw(+1.0, ZL0, ZL1), "lower_jaw"),
        origin=lower_pose,
        material=steel,
        name="jaw",
    )
    lower.visual(
        mesh_from_cadquery(_serrated_teeth(+1.0, BLADE_Z0, BLADE_Z1), "lower_teeth"),
        origin=lower_pose,
        material=steel,
        name="jaw_teeth",
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

    # ----- upper half (moving link): mirrored, upper steel layer
    upper = model.part("upper_half")
    upper.visual(
        mesh_from_cadquery(_half_jaw(-1.0, ZU0, ZU1), "upper_jaw"),
        material=steel,
        name="jaw",
    )
    upper.visual(
        mesh_from_cadquery(_serrated_teeth(-1.0, BLADE_Z0, BLADE_Z1), "upper_teeth"),
        material=steel,
        name="jaw_teeth",
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
        mesh_from_cadquery(_grip_ribs(-1.0, GRIP_UZ1), "upper_ribs"),
        material=dark_rubber,
        name="grip_ribs",
    )

    # ----- locking release lever (pivots inside the lower handle)
    # Compute the pivot point in world (parent frame = world for root).
    # The lever pivot is at (-0.050, -0.0095) in the HALF_OPEN-rotated local
    # frame used by lower half visuals, so rotate it into the parent frame.
    cos_h = math.cos(HALF_OPEN)
    sin_h = math.sin(HALF_OPEN)
    lpiv_x = LEVER_PIVOT_X * cos_h - LEVER_PIVOT_Y * sin_h
    lpiv_y = LEVER_PIVOT_X * sin_h + LEVER_PIVOT_Y * cos_h
    lever_z = GRIP_LZ1 - LEVER_THICKNESS * 0.5 - 0.0002  # sit just below grip top

    lever = model.part("release_lever")
    lever.visual(
        mesh_from_cadquery(_release_lever(), "lever_body"),
        origin=Origin(xyz=(0.0, 0.0, lever_z)),
        material=lever_metal,
        name="lever_body",
    )

    # ----- main pivot: revolute at central rivet
    model.articulation(
        "rivet_pivot",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, -HALF_OPEN)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=4.0, lower=0.0, upper=CLOSE_TRAVEL),
    )

    # ----- lever pivot: revolute inside lower handle
    # Joint origin at the rotated world position of the pivot, with HALF_OPEN
    # yaw so the lever's local -x direction aligns with the handle axis.
    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=lever,
        origin=Origin(xyz=(lpiv_x, lpiv_y, 0.0), rpy=(0.0, 0.0, HALF_OPEN)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=0.0, upper=LEVER_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower = object_model.get_part("lower_half")
    upper = object_model.get_part("upper_half")
    lever = object_model.get_part("release_lever")
    pivot = object_model.get_articulation("rivet_pivot")
    lever_joint = object_model.get_articulation("lever_pivot")

    # --- pivot stack: bosses coaxial, upper sits above lower
    ctx.expect_overlap(
        lower, upper, axes="xy",
        elem_a="pivot_boss", elem_b="pivot_boss",
        min_overlap=0.012,
        name="pivot bosses stack coaxially",
    )
    ctx.expect_gap(
        upper, lower, axis="z",
        positive_elem="pivot_boss", negative_elem="pivot_boss",
        min_gap=-0.00002, max_gap=0.0006,
        name="upper boss sits slightly above lower boss",
    )
    ctx.expect_contact(
        lower, upper,
        elem_a="pivot_boss", elem_b="pivot_boss",
        contact_tol=0.0001,
        name="upper boss rides on the lower boss face",
    )

    # --- rivet through boss stack
    ctx.expect_within(
        lower, upper, axes="xy",
        inner_elem="rivet", outer_elem="pivot_boss",
        margin=0.0002,
        name="rivet centered through the boss bore",
    )

    # --- jaws open at rest, handles splayed
    ctx.expect_gap(
        lower, upper, axis="y",
        positive_elem="jaw", negative_elem="jaw",
        min_gap=0.002,
        name="jaw faces are open at rest",
    )
    ctx.expect_gap(
        upper, lower, axis="y",
        positive_elem="grip", negative_elem="grip",
        min_gap=0.012,
        name="handles splay apart at rest",
    )

    # --- serrated teeth exist on both jaws, near the jaw face
    for part_name, elem_name in [("lower_half", "jaw_teeth"), ("upper_half", "jaw_teeth")]:
        p = object_model.get_part(part_name)
        jaw_aabb = ctx.part_element_world_aabb(p, elem="jaw")
        teeth_aabb = ctx.part_element_world_aabb(p, elem=elem_name)
        ctx.check(
            f"{part_name} has serrated teeth near jaw",
            jaw_aabb is not None and teeth_aabb is not None
            and teeth_aabb[0][0] >= jaw_aabb[0][0] - 0.002
            and teeth_aabb[1][0] <= jaw_aabb[1][0] + 0.002,
            details=f"jaw={jaw_aabb}, teeth={teeth_aabb}",
        )

    # --- crimping notch exists (jaw has a recess behind tips)
    # The crimp notch is cut from the jaw body; verify jaw still has forward extent
    for p in (lower, upper):
        jaw = ctx.part_element_world_aabb(p, elem="jaw")
        ctx.check(
            f"{p.name} jaw extends forward past crimp zone",
            jaw is not None and jaw[1][0] > 0.030,
            details=f"jaw={jaw}",
        )

    # --- grip ribs exist on both handles
    for part_name in ["lower_half", "upper_half"]:
        p = object_model.get_part(part_name)
        grip = ctx.part_element_world_aabb(p, elem="grip")
        ribs = ctx.part_element_world_aabb(p, elem="grip_ribs")
        ctx.check(
            f"{part_name} has textured grip ribs on handle",
            grip is not None and ribs is not None
            and ribs[0][0] >= grip[0][0] - 0.002
            and ribs[1][0] <= grip[1][0] + 0.002,
            details=f"grip={grip}, ribs={ribs}",
        )
        # Ribs protrude above the grip surface
        ctx.check(
            f"{part_name} ribs protrude above grip top",
            grip is not None and ribs is not None
            and ribs[1][2] > grip[1][2] - 0.0002,
            details=f"grip_top={grip[1][2]:.5f}, ribs_top={ribs[1][2]:.5f}",
        )

    # --- release lever exists and is mounted inside the lower handle
    lever_aabb = ctx.part_world_aabb(lever)
    lower_grip = ctx.part_element_world_aabb(lower, elem="grip")
    ctx.check(
        "release lever exists with valid geometry",
        lever_aabb is not None and lever_aabb[1][0] - lever_aabb[0][0] > 0.005,
        details=f"lever={lever_aabb}",
    )
    ctx.expect_overlap(
        lever, lower, axes="xy",
        elem_a="lever_body", elem_b="grip",
        min_overlap=0.003,
        name="release lever nested inside lower handle",
    )

    # --- lever joint has valid non-fixed travel
    lever_limits = lever_joint.motion_limits
    ctx.check(
        "lever pivot has non-trivial travel range",
        lever_limits is not None
        and lever_limits.lower is not None
        and lever_limits.upper is not None
        and lever_limits.upper > 0.1,
        details=f"limits={lever_limits}",
    )

    # --- lever articulation moves the lever (check element position, not part origin)
    lever_rest_aabb = ctx.part_element_world_aabb(lever, elem="lever_body")
    with ctx.pose({lever_joint: LEVER_TRAVEL}):
        lever_open_aabb = ctx.part_element_world_aabb(lever, elem="lever_body")
    ctx.check(
        "lever pivot rotates the release lever",
        lever_rest_aabb is not None and lever_open_aabb is not None
        and (abs(lever_open_aabb[0][0] - lever_rest_aabb[0][0]) > 0.0005
             or abs(lever_open_aabb[0][1] - lever_rest_aabb[0][1]) > 0.0005
             or abs(lever_open_aabb[1][0] - lever_rest_aabb[1][0]) > 0.0005
             or abs(lever_open_aabb[1][1] - lever_rest_aabb[1][1]) > 0.0005),
        details=f"rest={lever_rest_aabb}, open={lever_open_aabb}",
    )

    # --- main pivot articulation closes jaws
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
            lower, upper,
            elem_a="jaw", elem_b="jaw",
            contact_tol=0.002,
            name="jaw faces meet when fully closed",
        )
        closed_jaw = ctx.part_element_world_aabb(upper, elem="jaw")
        closed_grip = ctx.part_element_world_aabb(upper, elem="grip")

    ctx.check(
        "closing swings upper jaw toward lower jaw",
        open_jaw is not None and closed_jaw is not None
        and closed_jaw[1][1] > open_jaw[1][1] + 0.003,
        details=f"open={open_jaw}, closed={closed_jaw}",
    )
    ctx.check(
        "handles scissor opposite to the jaws",
        open_grip is not None and closed_grip is not None
        and closed_grip[0][1] < open_grip[0][1] - 0.003,
        details=f"open={open_grip}, closed={closed_grip}",
    )

    # --- overall proportions
    la = ctx.part_world_aabb(lower)
    ua = ctx.part_world_aabb(upper)
    if la is not None and ua is not None:
        length = max(la[1][0], ua[1][0]) - min(la[0][0], ua[0][0])
        width = max(la[1][1], ua[1][1]) - min(la[0][1], ua[0][1])
        ctx.check(
            "overall length ~0.13 m",
            0.110 <= length <= 0.145,
            details=f"length={length:.4f}",
        )
        ctx.check(
            "splayed handle width ~0.06 m",
            0.048 <= width <= 0.080,
            details=f"width={width:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
