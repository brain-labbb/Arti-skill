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
# Compact diagonal cutting pliers with short handles and folding safety latch.
#
# Variant of flush-cut snips: diagonal cutting jaws (cutting edge angled ~45°
# to the handle axis), shorter handles, circular pivot rivet caps on both
# sides, and a small folding latch on a revolute joint that folds over the
# handles when stowed.
#
# Two mirrored half-tools cross at a polished rivet. The lower half is the
# base link; the upper half rotates about the rivet axis (Z). A latch part
# is hinged to the lower handle tip and can fold across both handles.
# ---------------------------------------------------------------------------

HALF_OPEN = math.radians(12.5)  # each half is yawed this much at rest (open)
CLOSE_TRAVEL = 2.0 * HALF_OPEN  # ~25 degrees of closing travel

PLATE_T = 0.0035  # forged steel plate thickness per half

# Steel layer stack (lower plate sits below upper plate at the pivot boss)
ZL0, ZL1 = 0.0035, 0.0035 + PLATE_T  # lower half plate: 0.0035 .. 0.0070
ZU0, ZU1 = ZL1, ZL1 + PLATE_T  # upper half plate: 0.0070 .. 0.0105

# Full-height blade region
BLADE_Z0, BLADE_Z1 = ZL0, ZU1
BLADE_X_MIN = 0.0085

BOSS_R = 0.0075
HOLE_R = 0.0028
SHANK_R = 0.0024
HEAD_R = 0.0040

# Rivet cap dimensions (circular caps visible on both sides)
CAP_R = 0.0052
CAP_T = 0.0012

EDGE_LAND = 0.0003

# Short handle dimensions (compact pliers)
GRIP_H = 0.009
GRIP_LZ0 = 0.0
GRIP_LZ1 = GRIP_LZ0 + GRIP_H
GRIP_UZ0 = GRIP_LZ0 + (ZU0 - ZL0)
GRIP_UZ1 = GRIP_UZ0 + GRIP_H
INLAY_T = 0.0007
INLAY_EMBED = 0.0002

# Latch dimensions
LATCH_LENGTH = 0.032  # spans across both handles
LATCH_WIDTH = 0.006
LATCH_THICKNESS = 0.002
LATCH_PIVOT_R = 0.0025
LATCH_HINGE_ANGLE = math.radians(160)  # fold range

# Diagonal jaw outline (lower half, +y side). The cutting edge runs at ~45°
# to the handle axis, characteristic of diagonal cutters.
JAW_PTS = [
    (0.0000, 0.0085),
    (0.0050, 0.0090),
    (0.0120, 0.0082),
    (0.0200, 0.0062),
    (0.0260, 0.0038),
    (0.0290, 0.0018),
    (0.0280, EDGE_LAND),
    (0.0090, EDGE_LAND),
    (0.0020, 0.0045),
]

# Slim steel neck/tang (shorter than parent)
TANG_PTS = [
    (0.0020, -0.0030),
    (-0.0080, -0.0040),
    (-0.0160, -0.0050),
    (-0.0240, -0.0058),
    (-0.0240, -0.0100),
    (-0.0160, -0.0088),
    (-0.0080, -0.0076),
    (0.0008, -0.0068),
]

# Short curved handle outline (compact pliers - ~60% of parent length)
GRIP_PTS = [
    (-0.0200, -0.0036),
    (-0.0320, -0.0046),
    (-0.0440, -0.0058),
    (-0.0540, -0.0068),
    (-0.0620, -0.0078),
    (-0.0650, -0.0090),
    (-0.0620, -0.0112),
    (-0.0520, -0.0128),
    (-0.0400, -0.0132),
    (-0.0300, -0.0120),
    (-0.0210, -0.0098),
    (-0.0195, -0.0082),
]

# Inlay strip on handle top face
INLAY_PTS = [
    (-0.0260, -0.0062),
    (-0.0380, -0.0076),
    (-0.0500, -0.0088),
    (-0.0580, -0.0094),
    (-0.0540, -0.0104),
    (-0.0440, -0.0110),
    (-0.0340, -0.0102),
    (-0.0270, -0.0086),
]

# Diagonal bevel cut profile - the key visual difference from flush cutters.
# The cutting face is angled at ~45° to the jaw axis.
DIAG_BEVEL_ANGLE = math.radians(45)


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
    cap: float = 0.0018,
    inset: float = 0.0015,
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


def _diagonal_bevel(s: float) -> cq.Workplane:
    """Diagonal cutting face bevel. Unlike flush cutters (flat face), the
    diagonal cutter has an angled face running at ~45° from cutting edge to
    the jaw spine, giving the characteristic diagonal appearance."""
    # The bevel wedge runs along the jaw from BLADE_X_MIN forward
    # It cuts from the top at an angle, creating the diagonal face
    bevel_depth = 0.006
    bevel_length = 0.025
    tri = [
        (s * EDGE_LAND, BLADE_Z0 + 0.0004),
        (s * (EDGE_LAND + bevel_depth * math.tan(DIAG_BEVEL_ANGLE)), BLADE_Z0 + bevel_depth),
        (s * EDGE_LAND, BLADE_Z0 + bevel_depth),
    ]
    return (
        cq.Workplane("YZ", origin=(BLADE_X_MIN, 0.0, 0.0))
        .polyline(tri)
        .close()
        .extrude(bevel_length)
    )


def _blade_clip_box() -> cq.Workplane:
    return cq.Workplane("XY", origin=(BLADE_X_MIN + 0.0200, 0.0, 0.0070)).box(
        0.0420, 0.0600, 0.0200
    )


def _half_blade(s: float, own_z0: float, own_z1: float) -> cq.Workplane:
    prof = _mirror(JAW_PTS, s)
    full = _poly_prism(prof, BLADE_Z0, BLADE_Z1).intersect(_blade_clip_box())
    rear = _poly_prism(prof, own_z0, own_z1)
    return full.union(rear).cut(_diagonal_bevel(s))


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


def _rivet_cap(z_center: float) -> cq.Workplane:
    """Circular rivet cap disc visible on one side of the pivot."""
    return (
        cq.Workplane("XY", origin=(0.0, 0.0, z_center - CAP_T / 2))
        .circle(CAP_R)
        .extrude(CAP_T)
    )


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
    """Small folding latch bar that spans across both handles when folded over.
    Built as a rounded rectangular bar extending along +X from the hinge pivot.
    The bar sits at z=0 (part frame), which maps to the grip top surface."""
    # Bar extends along +X from origin, centered on Y, sitting at z=0
    bar = (
        cq.Workplane("XY", origin=(LATCH_LENGTH / 2, 0.0, LATCH_THICKNESS / 2))
        .box(LATCH_LENGTH, LATCH_WIDTH, LATCH_THICKNESS)
    )
    # Round the far end
    try:
        bar = bar.edges("|Z").fillet(LATCH_WIDTH * 0.35)
    except Exception:
        pass
    return bar


def _latch_pivot_pin() -> cq.Workplane:
    """Small pivot pin/boss for the latch hinge, embedded into the handle."""
    # Pin extends from below the grip top into the latch body
    return (
        cq.Workplane("XY", origin=(0.0, 0.0, -0.004))
        .circle(LATCH_PIVOT_R)
        .extrude(0.004 + LATCH_THICKNESS + 0.001)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_diagonal_pliers")

    steel = model.material("brushed_steel", rgba=(0.72, 0.73, 0.75, 1.0))
    polished = model.material("polished_steel", rgba=(0.86, 0.87, 0.90, 1.0))
    orange = model.material("orange_grip", rgba=(0.97, 0.52, 0.07, 1.0))
    peach = model.material("peach_inlay", rgba=(1.0, 0.80, 0.64, 0.78))
    dark_grey = model.material("dark_steel", rgba=(0.35, 0.36, 0.38, 1.0))

    # ----- lower half (base link): jaw at +y, handle at -y, lower steel layer
    lower = model.part("lower_half")
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
    # Lower rivet cap (circular disc on bottom side of pivot)
    lower.visual(
        mesh_from_cadquery(_rivet_cap(ZL0 - CAP_T), "rivet_cap_lower"),
        origin=lower_pose,
        material=polished,
        name="rivet_cap",
    )

    # ----- upper half (moving link): mirrored, upper steel layer, bored boss
    upper = model.part("upper_half")
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
    # Upper rivet cap (circular disc on top side of pivot)
    upper.visual(
        mesh_from_cadquery(_rivet_cap(ZU1 + 0.0002), "rivet_cap_upper"),
        material=polished,
        name="rivet_cap",
    )

    # ----- latch part: small folding bar hinged to lower handle tip
    latch = model.part("latch")
    # The latch hinge is placed at the grip tip top surface in world coords.
    # Lower half is root, so its part frame = world frame.
    # The grip tip (after HALF_OPEN rotation) is at approximately:
    # x ≈ -0.056, y ≈ -0.018, z = GRIP_LZ1 (grip top surface)
    # Latch visuals use identity origin since the part frame handles placement.
    latch.visual(
        mesh_from_cadquery(_latch_body(), "latch_bar"),
        origin=Origin(),
        material=dark_grey,
        name="latch_bar",
    )
    latch.visual(
        mesh_from_cadquery(_latch_pivot_pin(), "latch_pin"),
        origin=Origin(),
        material=polished,
        name="latch_pin",
    )

    # ----- main pivot articulation: revolute at rivet
    model.articulation(
        "rivet_pivot",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, -HALF_OPEN)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=4.0, lower=0.0, upper=CLOSE_TRAVEL),
    )

    # ----- latch hinge: revolute, latch folds over handles
    # The latch pivots about the Y axis (across the handle width).
    # q=0: latch stowed alongside lower handle (bar extends toward pivot).
    # q=LATCH_HINGE_ANGLE: latch folded over both handles (safety catch).
    # Hinge origin is in world coords (= lower part frame since lower is root).
    # The grip tip top surface is at approximately (-0.056, -0.018, 0.009).
    # The rpy rotates the child frame so +X aligns with the grip direction.
    model.articulation(
        "latch_hinge",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=latch,
        origin=Origin(
            xyz=(-0.056, -0.018, GRIP_LZ1),
            rpy=(0.0, 0.0, HALF_OPEN),
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=0.0, upper=LATCH_HINGE_ANGLE
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower = object_model.get_part("lower_half")
    upper = object_model.get_part("upper_half")
    latch = object_model.get_part("latch")
    pivot = object_model.get_articulation("rivet_pivot")
    latch_hinge = object_model.get_articulation("latch_hinge")

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

    # --- rivet caps present on both sides of pivot
    lower_cap_aabb = ctx.part_element_world_aabb(lower, elem="rivet_cap")
    upper_cap_aabb = ctx.part_element_world_aabb(upper, elem="rivet_cap")
    boss_aabb_l = ctx.part_element_world_aabb(lower, elem="pivot_boss")
    boss_aabb_u = ctx.part_element_world_aabb(upper, elem="pivot_boss")
    ctx.check(
        "lower rivet cap exists below pivot boss",
        lower_cap_aabb is not None
        and boss_aabb_l is not None
        and lower_cap_aabb[0][2] < boss_aabb_l[0][2] - 0.0002,
        details=f"cap={lower_cap_aabb}, boss={boss_aabb_l}",
    )
    ctx.check(
        "upper rivet cap exists above pivot boss",
        upper_cap_aabb is not None
        and boss_aabb_u is not None
        and upper_cap_aabb[1][2] > boss_aabb_u[1][2] + 0.0002,
        details=f"cap={upper_cap_aabb}, boss={boss_aabb_u}",
    )
    # Caps should be circular - roughly equal X and Y extents
    for cap_name, cap_aabb in [("lower", lower_cap_aabb), ("upper", upper_cap_aabb)]:
        if cap_aabb is not None:
            dx = cap_aabb[1][0] - cap_aabb[0][0]
            dy = cap_aabb[1][1] - cap_aabb[0][1]
            ctx.check(
                f"{cap_name} rivet cap is roughly circular",
                0.7 < dy / dx < 1.3 if dx > 0.001 else False,
                details=f"dx={dx:.4f}, dy={dy:.4f}",
            )

    # --- rest pose: jaws open, handles splayed apart
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
        positive_elem="grip",
        negative_elem="grip",
        min_gap=0.008,
        name="handles splay apart at rest",
    )

    # --- compact proportions: shorter than parent (~0.10m long)
    la = ctx.part_world_aabb(lower)
    ua = ctx.part_world_aabb(upper)
    if la is not None and ua is not None:
        length = max(la[1][0], ua[1][0]) - min(la[0][0], ua[0][0])
        width = max(la[1][1], ua[1][1]) - min(la[0][1], ua[0][1])
        ctx.check(
            "compact overall length ~0.10 m (shorter than parent)",
            0.080 <= length <= 0.120,
            details=f"length={length:.4f}",
        )
        ctx.check(
            "compact handle width",
            0.035 <= width <= 0.065,
            details=f"width={width:.4f}",
        )

    # --- handles are visibly shorter than parent (~60% of parent ~0.095m)
    for p in (lower, upper):
        grip_aabb = ctx.part_element_world_aabb(p, elem="grip")
        if grip_aabb is not None:
            grip_len = grip_aabb[1][0] - grip_aabb[0][0]
            ctx.check(
                f"{p.name} handle is short (compact design)",
                grip_len < 0.055,
                details=f"grip_length={grip_len:.4f}",
            )

    # --- diagonal jaw geometry: jaws are tapered and compact
    for p in (lower, upper):
        jaw_aabb = ctx.part_element_world_aabb(p, elem="jaw_blade")
        if jaw_aabb is not None:
            jaw_len = jaw_aabb[1][0] - jaw_aabb[0][0]
            ctx.check(
                f"{p.name} jaw is compact and tapered",
                0.018 <= jaw_len <= 0.032,
                details=f"jaw_length={jaw_len:.4f}",
            )

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
    open_grip = ctx.part_element_world_aabb(upper, elem="grip")
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
        closed_grip = ctx.part_element_world_aabb(upper, elem="grip")

    ctx.check(
        "closing swings upper jaw toward lower jaw",
        open_blade is not None
        and closed_blade is not None
        and closed_blade[1][1] > open_blade[1][1] + 0.004,
        details=f"open={open_blade}, closed={closed_blade}",
    )
    ctx.check(
        "handles scissor opposite to the jaws",
        open_grip is not None
        and closed_grip is not None
        and closed_grip[0][1] < open_grip[0][1] - 0.004,
        details=f"open={open_grip}, closed={closed_grip}",
    )

    # --- latch mechanism: exists, is articulated, folds over handles
    latch_bar_aabb = ctx.part_element_world_aabb(latch, elem="latch_bar")
    ctx.check(
        "latch bar exists",
        latch_bar_aabb is not None,
        details=f"latch_bar={latch_bar_aabb}",
    )

    # Latch hinge has non-trivial motion range
    latch_limits = latch_hinge.motion_limits
    ctx.check(
        "latch hinge has non-zero travel range",
        latch_limits is not None
        and latch_limits.upper is not None
        and latch_limits.upper > math.radians(90),
        details=f"latch_limits={latch_limits}",
    )

    # At rest (q=0), latch is stowed alongside the lower handle
    lower_grip_aabb = ctx.part_element_world_aabb(lower, elem="grip")
    if latch_bar_aabb is not None and lower_grip_aabb is not None:
        ctx.check(
            "latch stowed position is near lower handle",
            latch_bar_aabb[0][0] < lower_grip_aabb[0][0] + 0.010
            and latch_bar_aabb[1][0] > lower_grip_aabb[0][0] - 0.020,
            details=f"latch={latch_bar_aabb}, grip={lower_grip_aabb}",
        )

    # When latch hinge is at max, latch has rotated significantly
    with ctx.pose({latch_hinge: LATCH_HINGE_ANGLE}):
        folded_aabb = ctx.part_element_world_aabb(latch, elem="latch_bar")
    ctx.check(
        "latch rotates when hinge is actuated",
        latch_bar_aabb is not None
        and folded_aabb is not None
        and abs(folded_aabb[1][0] - latch_bar_aabb[1][0]) > 0.005,
        details=f"rest={latch_bar_aabb}, folded={folded_aabb}",
    )

    # Latch pin connects latch to lower handle (support check)
    latch_pin_aabb = ctx.part_element_world_aabb(latch, elem="latch_pin")
    if latch_pin_aabb is not None and lower_grip_aabb is not None:
        ctx.expect_overlap(
            latch,
            lower,
            axes="xy",
            elem_a="latch_pin",
            elem_b="grip",
            min_overlap=0.001,
            name="latch pin overlaps lower handle (hinge mount)",
        )

    # Allow the latch pin to overlap with the lower handle grip since the
    # pin is intentionally embedded in the handle as a hinge mount.
    ctx.allow_overlap(
        lower,
        latch,
        elem_a="grip",
        elem_b="latch_pin",
        reason="Latch pivot pin is intentionally embedded in the handle as a hinge mount point.",
    )
    ctx.expect_contact(
        lower,
        latch,
        elem_a="grip",
        elem_b="latch_pin",
        contact_tol=0.003,
        name="latch pin contacts lower handle at hinge",
    )

    return ctx.report()


object_model = build_object_model()
