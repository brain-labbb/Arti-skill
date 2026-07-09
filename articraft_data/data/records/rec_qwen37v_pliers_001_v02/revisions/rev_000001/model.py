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
# Compact diagonal cutting pliers (diagonal cutters / diag cutters).
#
# Variant of flush-cut wire snips: shorter diagonal cutting jaws, short
# handles, locking release lever pivoting inside one handle, circular pivot
# rivet caps on both sides, and visible cutter bevel wedges.
#
# Two mirrored half-tools cross at a central rivet. Each half is one rigid
# link: compact diagonal cutting jaw -> slim steel neck -> short curved
# over-molded handle. The lower half is the base link; the upper half
# rotates about the rivet axis (Z, perpendicular to the flat plane of the
# tool). A locking release lever pivots inside the lower handle.
# ---------------------------------------------------------------------------

HALF_OPEN = math.radians(12.5)
CLOSE_TRAVEL = 2.0 * HALF_OPEN  # ~25 degrees of closing travel

PLATE_T = 0.0035

# Steel layer stack
ZL0, ZL1 = 0.0035, 0.0035 + PLATE_T
ZU0, ZU1 = ZL1, ZL1 + PLATE_T

BLADE_Z0, BLADE_Z1 = ZL0, ZU1
BLADE_X_MIN = 0.0085

BOSS_R = 0.0075
HOLE_R = 0.0028
SHANK_R = 0.0024
HEAD_R = 0.0040

# Rivet cap dimensions (circular disc caps on both sides)
CAP_R = 0.0052
CAP_T = 0.0012

EDGE_LAND = 0.0003

# Handle over-mold
GRIP_H = 0.0105
GRIP_LZ0 = 0.0
GRIP_LZ1 = GRIP_LZ0 + GRIP_H
GRIP_UZ0 = GRIP_LZ0 + (ZU0 - ZL0)
GRIP_UZ1 = GRIP_UZ0 + GRIP_H
INLAY_T = 0.0007
INLAY_EMBED = 0.0002

# Lever dimensions
LEVER_LENGTH = 0.022
LEVER_WIDTH = 0.005
LEVER_THICK = 0.0025
LEVER_PIVOT_R = 0.002
LEVER_TRAVEL = math.radians(15.0)  # lever pivots ~15 degrees

# Compact diagonal jaw outline (shorter, more angled than flush cut).
# The jaw tip is more forward-angled for diagonal cutting.
JAW_PTS = [
    (0.0000, 0.0085),
    (0.0050, 0.0088),
    (0.0120, 0.0078),
    (0.0200, 0.0052),
    (0.0270, 0.0024),
    (0.0290, 0.0010),
    (0.0284, EDGE_LAND),
    (0.0090, EDGE_LAND),
    (0.0020, 0.0045),
]

# Short tang/neck
TANG_PTS = [
    (0.0020, -0.0032),
    (-0.0080, -0.0042),
    (-0.0160, -0.0050),
    (-0.0240, -0.0058),
    (-0.0240, -0.0100),
    (-0.0160, -0.0086),
    (-0.0080, -0.0074),
    (0.0008, -0.0068),
]

# Short curved handle (compact diagonal cutters have shorter handles)
GRIP_PTS = [
    (-0.0190, -0.0038),
    (-0.0320, -0.0048),
    (-0.0460, -0.0058),
    (-0.0580, -0.0068),
    (-0.0680, -0.0078),
    (-0.0720, -0.0090),
    (-0.0690, -0.0120),
    (-0.0580, -0.0138),
    (-0.0460, -0.0142),
    (-0.0340, -0.0130),
    (-0.0240, -0.0108),
    (-0.0185, -0.0088),
]

# Translucent inlay strip
INLAY_PTS = [
    (-0.0255, -0.0065),
    (-0.0380, -0.0078),
    (-0.0520, -0.0090),
    (-0.0620, -0.0096),
    (-0.0660, -0.0102),
    (-0.0620, -0.0112),
    (-0.0500, -0.0118),
    (-0.0390, -0.0110),
    (-0.0300, -0.0096),
    (-0.0262, -0.0085),
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


def _bevel_wedge(s: float) -> cq.Workplane:
    """Cutter bevel wedge: visible angled wedge geometry cut from the blade
    face to create the diagonal cutting edge bevel."""
    tri = [
        (s * EDGE_LAND, BLADE_Z0 + 0.0004),
        (s * 0.0048, 0.0110),
        (s * EDGE_LAND, 0.0110),
    ]
    return (
        cq.Workplane("YZ", origin=(BLADE_X_MIN, 0.0, 0.0))
        .polyline(tri)
        .close()
        .extrude(0.0270)
    )


def _cutter_bevel_visual(s: float) -> cq.Workplane:
    """Visible bevel wedge geometry on the cutting face - a thin angled wedge
    showing the ground bevel of the diagonal cutter."""
    bevel_pts = [
        (s * EDGE_LAND, BLADE_Z0 + 0.0002),
        (s * 0.0035, BLADE_Z0 + 0.0028),
        (s * 0.0038, BLADE_Z0 + 0.0028),
        (s * EDGE_LAND, BLADE_Z0 + 0.0002),
    ]
    # Make a thin wedge profile and extrude along the jaw length
    wedge = (
        cq.Workplane("YZ", origin=(BLADE_X_MIN + 0.002, 0.0, 0.0))
        .polyline(bevel_pts)
        .close()
        .extrude(0.0220)
    )
    return wedge


def _blade_clip_box() -> cq.Workplane:
    return cq.Workplane("XY", origin=(BLADE_X_MIN + 0.0200, 0.0, 0.0070)).box(
        0.0420, 0.0500, 0.0200
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


def _rivet_cap(z_center: float) -> cq.Workplane:
    """Circular pivot rivet cap disc."""
    cap = (
        cq.Workplane("XY", origin=(0.0, 0.0, z_center - CAP_T / 2.0))
        .circle(CAP_R)
        .extrude(CAP_T)
    )
    try:
        cap = cap.edges(">Z").fillet(0.0004)
        cap = cap.edges("<Z").fillet(0.0004)
    except Exception:
        pass
    return cap


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


def _locking_lever() -> cq.Workplane:
    """Locking release lever that pivots inside one handle.
    Pivot bore at origin (0,0,0); body extends along +X.
    Thumb-catch tab at the far end."""
    # Lever body: elongated rounded shape, pivot at origin, extends +X
    lever = (
        cq.Workplane("XY", origin=(LEVER_LENGTH / 2.0, 0.0, 0.0))
        .rect(LEVER_LENGTH, LEVER_WIDTH)
        .extrude(LEVER_THICK)
    )
    # Round the ends
    try:
        lever = lever.edges("|Z").fillet(LEVER_WIDTH / 2.0 - 0.0002)
    except Exception:
        pass
    # Pivot bore hole at origin
    bore = (
        cq.Workplane("XY", origin=(0.0, 0.0, -0.001))
        .circle(LEVER_PIVOT_R * 0.6)
        .extrude(LEVER_THICK + 0.002)
    )
    lever = lever.cut(bore)
    # Thumb catch tab at the far end: small raised bump
    tab = (
        cq.Workplane("XY", origin=(LEVER_LENGTH - 0.003, 0.0, LEVER_THICK))
        .rect(0.004, LEVER_WIDTH * 0.7)
        .extrude(0.0012)
    )
    lever = lever.union(tab)
    return lever


def _lever_pivot_pin() -> cq.Workplane:
    """Small pivot pin/shaft for the locking lever, embedded in the handle."""
    pin = (
        cq.Workplane("XY", origin=(0.0, 0.0, -0.002))
        .circle(LEVER_PIVOT_R)
        .extrude(LEVER_THICK + 0.004)
    )
    return pin


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_diagonal_cutters")

    steel = model.material("brushed_steel", rgba=(0.72, 0.73, 0.75, 1.0))
    polished = model.material("polished_steel", rgba=(0.86, 0.87, 0.90, 1.0))
    orange = model.material("orange_grip", rgba=(0.97, 0.52, 0.07, 1.0))
    peach = model.material("peach_inlay", rgba=(1.0, 0.80, 0.64, 0.78))
    dark_grey = model.material("dark_grey_plastic", rgba=(0.22, 0.22, 0.24, 1.0))

    # ----- lower half (base link): jaw at +y, handle at -y
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
    # Visible cutter bevel wedge on lower jaw
    lower.visual(
        mesh_from_cadquery(_cutter_bevel_visual(+1.0), "lower_bevel"),
        origin=lower_pose,
        material=polished,
        name="cutter_bevel",
    )
    # Rivet (shared pivot hardware)
    lower.visual(
        mesh_from_cadquery(_rivet(), "rivet"),
        origin=lower_pose,
        material=polished,
        name="rivet",
    )
    # Lower rivet cap (bottom side)
    lower.visual(
        mesh_from_cadquery(_rivet_cap(0.0021 - 0.0002), "lower_cap"),
        origin=lower_pose,
        material=polished,
        name="rivet_cap_lower",
    )
    # Upper rivet cap (top side) - on lower half for visual symmetry
    lower.visual(
        mesh_from_cadquery(_rivet_cap(ZU1 + 0.0016), "upper_cap"),
        origin=lower_pose,
        material=polished,
        name="rivet_cap_upper",
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
    # Visible cutter bevel wedge on upper jaw
    upper.visual(
        mesh_from_cadquery(_cutter_bevel_visual(-1.0), "upper_bevel"),
        material=polished,
        name="cutter_bevel",
    )

    # Lever pivot location on the lower handle (world space, since lower is root)
    # Handle runs along negative y side at about y=-0.018 center, grip midsection
    LEVER_PIVOT_XYZ = (-0.036, -0.016, GRIP_LZ0 + GRIP_H * 0.35)
    lever_yaw = HALF_OPEN + math.radians(5.0)

    # ----- locking release lever (pivots inside lower handle)
    lever = model.part("locking_lever")
    # Lever frame at origin; body extends along +X from pivot bore
    lever.visual(
        mesh_from_cadquery(_locking_lever(), "lever_body"),
        material=dark_grey,
        name="lever_body",
    )

    # ----- main pivot articulation (rivet)
    model.articulation(
        "rivet_pivot",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, -HALF_OPEN)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=4.0, lower=0.0, upper=CLOSE_TRAVEL),
    )

    # ----- locking lever articulation (pivots inside lower handle)
    # The lever pivots about its bore at origin; axis is Z (perpendicular to tool plane)
    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=lever,
        origin=Origin(xyz=LEVER_PIVOT_XYZ, rpy=(0.0, 0.0, lever_yaw)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=6.0, lower=0.0, upper=LEVER_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower = object_model.get_part("lower_half")
    upper = object_model.get_part("upper_half")
    lever = object_model.get_part("locking_lever")
    pivot = object_model.get_articulation("rivet_pivot")
    lever_joint = object_model.get_articulation("lever_pivot")

    # --- pivot stack: bosses coaxial, upper sits above lower
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

    # --- rivet caps present on both sides of the pivot
    lower_cap_aabb = ctx.part_element_world_aabb(lower, elem="rivet_cap_lower")
    upper_cap_aabb = ctx.part_element_world_aabb(lower, elem="rivet_cap_upper")
    ctx.check(
        "lower rivet cap exists below the boss stack",
        lower_cap_aabb is not None and lower_cap_aabb[0][2] < ZL0 + 0.001,
        details=f"lower_cap={lower_cap_aabb}",
    )
    ctx.check(
        "upper rivet cap exists above the boss stack",
        upper_cap_aabb is not None and upper_cap_aabb[1][2] > ZU1 - 0.001,
        details=f"upper_cap={upper_cap_aabb}",
    )
    # Both caps are circular and centered on the pivot
    ctx.expect_overlap(
        lower,
        lower,
        axes="xy",
        elem_a="rivet_cap_lower",
        elem_b="pivot_boss",
        min_overlap=0.006,
        name="lower cap centered on pivot boss",
    )
    ctx.expect_overlap(
        lower,
        lower,
        axes="xy",
        elem_a="rivet_cap_upper",
        elem_b="pivot_boss",
        min_overlap=0.006,
        name="upper cap centered on pivot boss",
    )

    # --- cutter bevel wedges visible on both jaws
    for part_obj, bevel_side in [(lower, "lower"), (upper, "upper")]:
        bevel_aabb = ctx.part_element_world_aabb(part_obj, elem="cutter_bevel")
        blade_aabb = ctx.part_element_world_aabb(part_obj, elem="jaw_blade")
        ctx.check(
            f"{bevel_side} cutter bevel exists on jaw",
            bevel_aabb is not None and blade_aabb is not None,
            details=f"bevel={bevel_aabb}, blade={blade_aabb}",
        )
        if bevel_aabb is not None and blade_aabb is not None:
            # Bevel overlaps with blade in X (along jaw length)
            ctx.expect_overlap(
                part_obj,
                part_obj,
                axes="x",
                elem_a="cutter_bevel",
                elem_b="jaw_blade",
                min_overlap=0.008,
                name=f"{bevel_side} bevel overlaps blade along jaw length",
            )

    # --- locking release lever exists and is mounted in the handle
    lever_aabb = ctx.part_element_world_aabb(lever, elem="lever_body")
    grip_aabb = ctx.part_element_world_aabb(lower, elem="grip")
    ctx.check(
        "locking lever body exists",
        lever_aabb is not None,
        details=f"lever={lever_aabb}",
    )
    # Lever overlaps with the handle region in XY (it sits inside the handle)
    ctx.expect_overlap(
        lever,
        lower,
        axes="xy",
        elem_a="lever_body",
        elem_b="grip",
        min_overlap=0.004,
        name="lever sits within the lower handle region",
    )
    # The lever is intentionally housed/embedded inside the grip
    ctx.allow_overlap(
        lower,
        lever,
        elem_a="grip",
        elem_b="lever_body",
        reason="The locking release lever is housed inside the lower handle grip as a pivoting internal mechanism.",
    )
    # Proof: lever Z stays within grip Z (embedded in the handle thickness)
    ctx.expect_within(
        lever,
        lower,
        axes="z",
        inner_elem="lever_body",
        outer_elem="grip",
        margin=0.002,
        name="lever body embedded within grip thickness",
    )

    # --- lever articulation is non-fixed with proper limits
    lever_limits = lever_joint.motion_limits
    ctx.check(
        "lever pivot has non-trivial travel",
        lever_limits is not None
        and lever_limits.upper is not None
        and lever_limits.upper > math.radians(5.0),
        details=f"limits={lever_limits}",
    )

    # --- lever pivots when actuated (check far end of lever body moves)
    lever_rest_aabb = ctx.part_element_world_aabb(lever, elem="lever_body")
    with ctx.pose({lever_joint: LEVER_TRAVEL}):
        lever_act_aabb = ctx.part_element_world_aabb(lever, elem="lever_body")
    ctx.check(
        "lever swings when actuated",
        lever_rest_aabb is not None
        and lever_act_aabb is not None
        and (
            abs(lever_act_aabb[1][1] - lever_rest_aabb[1][1]) > 0.0003
            or abs(lever_act_aabb[1][0] - lever_rest_aabb[1][0]) > 0.0003
        ),
        details=f"rest_max={lever_rest_aabb[1]}, act_max={lever_act_aabb[1]}",
    )

    # --- rest pose: jaws open, handles splayed apart
    ctx.expect_gap(
        lower,
        upper,
        axis="y",
        positive_elem="jaw_blade",
        negative_elem="jaw_blade",
        min_gap=0.0020,
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

    # --- compact diagonal jaws are shorter than flush-cut parent
    for part_obj, label in [(lower, "lower"), (upper, "upper")]:
        blade = ctx.part_element_world_aabb(part_obj, elem="jaw_blade")
        ctx.check(
            f"{label} jaw is compact (diagonal profile)",
            blade is not None and (blade[1][0] - blade[0][0]) < 0.032,
            details=f"blade_x_span={blade}",
        )

    # --- overall proportions: compact diagonal cutters ~0.10m long
    la = ctx.part_world_aabb(lower)
    ua = ctx.part_world_aabb(upper)
    if la is not None and ua is not None:
        length = max(la[1][0], ua[1][0]) - min(la[0][0], ua[0][0])
        width = max(la[1][1], ua[1][1]) - min(la[0][1], ua[0][1])
        ctx.check(
            "compact overall length ~0.10 m",
            0.080 <= length <= 0.120,
            details=f"length={length:.4f}",
        )
        ctx.check(
            "splayed handle width reasonable",
            0.035 <= width <= 0.070,
            details=f"width={width:.4f}",
        )

    # --- articulation: positive q closes blades, handles scissor
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
        and closed_blade[1][1] > open_blade[1][1] + 0.003,
        details=f"open={open_blade}, closed={closed_blade}",
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
