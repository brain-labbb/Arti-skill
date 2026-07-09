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
# Broad lineman pliers with gripping teeth, cutter notch, and jaw stop boss.
#
# Two mirrored half-tools cross at a single polished rivet. Each half is one
# rigid link: broad flat gripping jaw with serrated teeth -> heavy steel neck
# -> long dipped handle. The lower half is the base link; the upper half
# rotates about the rivet axis (Z, perpendicular to the flat plane of the tool).
# ---------------------------------------------------------------------------

HALF_OPEN = math.radians(14.0)  # each half is yawed this much at rest (open)
CLOSE_TRAVEL = 2.0 * HALF_OPEN  # ~28 degrees of closing travel

PLATE_T = 0.0040  # forged steel plate thickness per half (heavier than snips)

# Steel layer stack.
ZL0, ZL1 = 0.0030, 0.0030 + PLATE_T  # lower half plate
ZU0, ZU1 = ZL1, ZL1 + PLATE_T  # upper half plate

# Full-height jaw region spans both plate layers.
BLADE_Z0, BLADE_Z1 = ZL0, ZU1
BLADE_X_MIN = 0.0090  # full-height jaw exists only forward of the boss

BOSS_R = 0.0090
HOLE_R = 0.0030
SHANK_R = 0.0026
HEAD_R = 0.0044

EDGE_LAND = 0.0004  # jaw face land offset from shear plane

# Handle over-mold (z extents per half).
GRIP_H = 0.012
GRIP_LZ0 = 0.0
GRIP_LZ1 = GRIP_LZ0 + GRIP_H
GRIP_UZ0 = GRIP_LZ0 + (ZU0 - ZL0)
GRIP_UZ1 = GRIP_UZ0 + GRIP_H
INLAY_T = 0.0008
INLAY_EMBED = 0.0002

# Jaw stop boss: a raised bump near the pivot on the lower half that limits
# jaw opening. Located just behind the pivot boss.
JAW_STOP_R = 0.004
JAW_STOP_H = 0.003  # protrudes above the upper plate

# --- Broad lineman pliers jaw profile (lower half, +y side) ---
# Wide flat gripping jaw, roughly rectangular with rounded tip.
JAW_PTS = [
    (0.0010, 0.0120),
    (0.0080, 0.0128),
    (0.0180, 0.0130),
    (0.0300, 0.0125),
    (0.0400, 0.0110),
    (0.0440, 0.0080),
    (0.0420, 0.0040),
    (0.0380, 0.0018),
    (0.0090, EDGE_LAND),
    (0.0020, 0.0060),
]

# Cutter notch: a semicircular cutout in the jaw near the pivot, on the
# inner face. We'll cut this from the jaw body.
CUTTER_NOTCH_X = 0.016
CUTTER_NOTCH_R = 0.003

# Heavy steel neck/tang.
TANG_PTS = [
    (0.0020, -0.0040),
    (-0.0120, -0.0054),
    (-0.0250, -0.0064),
    (-0.0380, -0.0076),
    (-0.0380, -0.0140),
    (-0.0250, -0.0120),
    (-0.0120, -0.0100),
    (0.0010, -0.0084),
]

# Curved over-molded handle outline (wider, heavier than snips).
GRIP_PTS = [
    (-0.0300, -0.0048),
    (-0.0500, -0.0060),
    (-0.0720, -0.0074),
    (-0.0920, -0.0090),
    (-0.1100, -0.0102),
    (-0.1180, -0.0120),
    (-0.1130, -0.0160),
    (-0.0960, -0.0186),
    (-0.0760, -0.0192),
    (-0.0560, -0.0174),
    (-0.0380, -0.0140),
    (-0.0295, -0.0115),
]

# Inlay strip along handle top face.
INLAY_PTS = [
    (-0.0380, -0.0082),
    (-0.0580, -0.0098),
    (-0.0800, -0.0112),
    (-0.0960, -0.0120),
    (-0.1040, -0.0126),
    (-0.0980, -0.0140),
    (-0.0820, -0.0148),
    (-0.0640, -0.0140),
    (-0.0460, -0.0124),
    (-0.0390, -0.0108),
]

# Serrated teeth: small triangular ridges on the inner jaw face.
# We'll create them as small triangular prisms along the jaw inner edge.
TOOTH_COUNT = 12
TOOTH_X_START = 0.012
TOOTH_X_END = 0.038
TOOTH_WIDTH = (TOOTH_X_END - TOOTH_X_START) / TOOTH_COUNT
TOOTH_DEPTH = 0.0012  # how far tooth protrudes toward center
TOOTH_HEIGHT = 0.0010  # tooth height in z

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


def _jaw_clip_box() -> cq.Workplane:
    """Clip box for the full-height jaw region."""
    return cq.Workplane("XY", origin=(BLADE_X_MIN + 0.0250, 0.0, 0.0070)).box(
        0.0600, 0.0600, 0.0250
    )


def _cutter_notch_cutter(s: float) -> cq.Workplane:
    """Semicircular cutter for the wire cutter notch on the inner jaw face."""
    # Positioned near the pivot, cuts into the jaw from the inner (center) side.
    return (
        cq.Workplane("XY", origin=(CUTTER_NOTCH_X, s * EDGE_LAND, BLADE_Z0 - 0.001))
        .circle(CUTTER_NOTCH_R)
        .extrude(BLADE_Z1 - BLADE_Z0 + 0.002)
    )


def _half_jaw(s: float, own_z0: float, own_z1: float) -> cq.Workplane:
    """Build one jaw half: broad gripping jaw with cutter notch cut out."""
    prof = _mirror(JAW_PTS, s)
    full = _poly_prism(prof, BLADE_Z0, BLADE_Z1).intersect(_jaw_clip_box())
    rear = _poly_prism(prof, own_z0, own_z1)
    jaw = full.union(rear)
    # Cut the cutter notch from the inner face
    jaw = jaw.cut(_cutter_notch_cutter(s))
    return jaw


def _serrated_teeth(s: float, own_z0: float, own_z1: float) -> cq.Workplane:
    """Serrated gripping teeth strip on the inner jaw face.

    Built as a single continuous strip that embeds into the jaw body in both
    Y (extends from jaw inner face inward, past the jaw polygon boundary) and
    Z (spans the full jaw plate thickness) to ensure geometric connectivity
    with the parent jaw.
    """
    y_outer = s * (EDGE_LAND + TOOTH_DEPTH + 0.001)  # embed into jaw body
    y_inner = s * (EDGE_LAND - 0.001)  # protrude slightly past center

    # Build a serrated strip along X with triangular profile in YZ
    # The strip spans full jaw Z height for connectivity
    strip_z0 = own_z0 - 0.0005
    strip_z1 = own_z1 + 0.0005

    # Create the serrated profile as a series of connected boxes with
    # triangular cross-section, all fused into one solid
    # Use a base bar for connectivity, then add tooth ridges on top
    base_bar = (
        cq.Workplane("XY", origin=(TOOTH_X_START - 0.001, 0.0, strip_z0))
        .box(
            TOOTH_X_END - TOOTH_X_START + 0.002,
            abs(y_outer - y_inner),
            strip_z1 - strip_z0,
            centered=False,
        )
    )
    # Shift the bar to the right Y position
    if s > 0:
        # Lower half: teeth at +y side, bar from y_inner to y_outer
        base_bar = (
            cq.Workplane("XY", origin=(TOOTH_X_START - 0.001, y_inner, strip_z0))
            .box(
                TOOTH_X_END - TOOTH_X_START + 0.002,
                y_outer - y_inner,
                strip_z1 - strip_z0,
                centered=False,
            )
        )
    else:
        # Upper half: teeth at -y side
        base_bar = (
            cq.Workplane("XY", origin=(TOOTH_X_START - 0.001, y_outer, strip_z0))
            .box(
                TOOTH_X_END - TOOTH_X_START + 0.002,
                y_inner - y_outer,
                strip_z1 - strip_z0,
                centered=False,
            )
        )

    # Add triangular tooth ridges on the inner face
    ridges = None
    ridge_base = s * (EDGE_LAND - 0.0005)  # slightly past center toward opposing jaw
    ridge_tip = s * (EDGE_LAND - TOOTH_DEPTH - 0.001)  # protrude toward opposing jaw

    for i in range(TOOTH_COUNT):
        x = TOOTH_X_START + i * TOOTH_WIDTH + TOOTH_WIDTH * 0.5
        # Triangular ridge in YZ plane
        tri_pts = [
            (ridge_base, strip_z0 + 0.0002),
            (ridge_base, strip_z1 - 0.0002),
            (ridge_tip, (strip_z0 + strip_z1) / 2.0),
        ]
        ridge = (
            cq.Workplane("YZ", origin=(x - TOOTH_WIDTH * 0.3, 0.0, 0.0))
            .polyline(tri_pts)
            .close()
            .extrude(TOOTH_WIDTH * 0.6)
        )
        ridges = ridge if ridges is None else ridges.union(ridge)

    if ridges is not None:
        return base_bar.union(ridges)
    return base_bar


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


def _jaw_stop_boss() -> cq.Workplane:
    """Raised bump on the lower half near the pivot that limits jaw opening.
    Protrudes above ZU1 to contact the upper half when jaws are fully open."""
    return (
        cq.Workplane("XY", origin=(-0.005, 0.006, ZU1))
        .circle(JAW_STOP_R)
        .extrude(JAW_STOP_H)
    )


def _rivet() -> cq.Workplane:
    lower_head = cq.Workplane("XY", origin=(0.0, 0.0, 0.0016)).circle(HEAD_R).extrude(0.0020)
    shank = cq.Workplane("XY", origin=(0.0, 0.0, 0.0016)).circle(SHANK_R).extrude(0.0120)
    upper_head = cq.Workplane("XY", origin=(0.0, 0.0, ZU1 + 0.0001)).circle(HEAD_R).extrude(
        0.0016
    )
    rivet = lower_head.union(shank).union(upper_head)
    try:
        rivet = rivet.edges(">Z").fillet(0.0010)
    except Exception:
        pass
    return rivet


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lineman_pliers")

    steel = model.material("brushed_steel", rgba=(0.68, 0.70, 0.72, 1.0))
    polished = model.material("polished_steel", rgba=(0.84, 0.85, 0.88, 1.0))
    orange = model.material("orange_grip", rgba=(0.95, 0.48, 0.06, 1.0))
    peach = model.material("peach_inlay", rgba=(1.0, 0.78, 0.60, 0.75))
    dark_steel = model.material("dark_steel", rgba=(0.45, 0.46, 0.48, 1.0))

    # ----- lower half (base link): jaw at +y, handle at -y, lower steel layer
    lower = model.part("lower_half")
    lower_pose = Origin(rpy=(0.0, 0.0, HALF_OPEN))
    lower.visual(
        mesh_from_cadquery(_half_jaw(+1.0, ZL0, ZL1), "lower_jaw"),
        origin=lower_pose,
        material=steel,
        name="jaw_body",
    )
    lower.visual(
        mesh_from_cadquery(_serrated_teeth(+1.0, ZL0, ZU1), "lower_teeth"),
        origin=lower_pose,
        material=dark_steel,
        name="jaw_teeth",
    )
    lower.visual(
        mesh_from_cadquery(_half_boss(ZL0, ZL1, with_hole=False), "lower_boss"),
        origin=lower_pose,
        material=steel,
        name="pivot_boss",
    )
    lower.visual(
        mesh_from_cadquery(_jaw_stop_boss(), "jaw_stop"),
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
        name="jaw_body",
    )
    upper.visual(
        mesh_from_cadquery(_serrated_teeth(-1.0, ZL0, ZU1), "upper_teeth"),
        material=dark_steel,
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
        mesh_from_cadquery(
            _spline_prism(
                _mirror(INLAY_PTS, -1.0), GRIP_UZ1 - INLAY_EMBED, GRIP_UZ1 - INLAY_EMBED + INLAY_T
            ),
            "upper_inlay",
        ),
        material=peach,
        name="grip_inlay",
    )

    # ----- main pivot: revolute at the rivet
    model.articulation(
        "rivet_pivot",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, -HALF_OPEN)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=4.0, lower=0.0, upper=CLOSE_TRAVEL),
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
    ctx.expect_contact(
        lower,
        upper,
        elem_a="pivot_boss",
        elem_b="pivot_boss",
        contact_tol=0.0001,
        name="upper boss rides on the lower boss face",
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
    rivet_aabb = ctx.part_element_world_aabb(lower, elem="rivet")
    uboss_aabb = ctx.part_element_world_aabb(upper, elem="pivot_boss")
    ctx.check(
        "rivet head caps the upper boss",
        rivet_aabb is not None
        and uboss_aabb is not None
        and rivet_aabb[1][2] > uboss_aabb[1][2] + 0.0008,
        details=f"rivet={rivet_aabb}, upper_boss={uboss_aabb}",
    )

    # --- jaw stop boss exists on lower half and protrudes above upper plate
    stop_aabb = ctx.part_element_world_aabb(lower, elem="jaw_stop_boss")
    jaw_aabb = ctx.part_element_world_aabb(lower, elem="jaw_body")
    ctx.check(
        "jaw stop boss protrudes above jaw top",
        stop_aabb is not None
        and jaw_aabb is not None
        and stop_aabb[1][2] > jaw_aabb[1][2] + 0.001,
        details=f"stop={stop_aabb}, jaw={jaw_aabb}",
    )

    # --- serrated teeth exist on both jaws
    for part_name, part in [("lower", lower), ("upper", upper)]:
        teeth_aabb = ctx.part_element_world_aabb(part, elem="jaw_teeth")
        jaw_aabb_p = ctx.part_element_world_aabb(part, elem="jaw_body")
        ctx.check(
            f"{part_name} jaw has serrated teeth near jaw face",
            teeth_aabb is not None
            and jaw_aabb_p is not None
            and teeth_aabb[0][0] >= jaw_aabb_p[0][0] - 0.002
            and teeth_aabb[1][0] <= jaw_aabb_p[1][0] + 0.002,
            details=f"teeth={teeth_aabb}, jaw={jaw_aabb_p}",
        )

    # --- cutter notch: the jaw body has a visible semicircular cutout
    # (jaw body width should be reduced near the notch x-location)
    lower_jaw_aabb = ctx.part_element_world_aabb(lower, elem="jaw_body")
    ctx.check(
        "lower jaw body spans forward of the pivot for gripping",
        lower_jaw_aabb is not None and lower_jaw_aabb[1][0] > 0.035,
        details=f"jaw_aabb={lower_jaw_aabb}",
    )

    # --- rest pose: jaws open, handles splayed apart
    ctx.expect_gap(
        lower,
        upper,
        axis="y",
        positive_elem="jaw_body",
        negative_elem="jaw_body",
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

    # --- overall proportions: ~0.18-0.20 m long, ~0.07 across handles
    la = ctx.part_world_aabb(lower)
    ua = ctx.part_world_aabb(upper)
    if la is not None and ua is not None:
        length = max(la[1][0], ua[1][0]) - min(la[0][0], ua[0][0])
        width = max(la[1][1], ua[1][1]) - min(la[0][1], ua[0][1])
        height = max(la[1][2], ua[1][2]) - min(la[0][2], ua[0][2])
        ctx.check(
            "overall length ~0.18 m (lineman pliers)",
            0.150 <= length <= 0.220,
            details=f"length={length:.4f}",
        )
        ctx.check(
            "splayed handle width ~0.07 m",
            0.055 <= width <= 0.095,
            details=f"width={width:.4f}",
        )
        ctx.check(
            "flat tool thickness ~0.018 m",
            0.014 <= height <= 0.024,
            details=f"height={height:.4f}",
        )
    else:
        ctx.fail("overall proportions", "missing part AABBs")

    # --- articulation: positive q closes jaws, handles scissor in opposition
    limits = pivot.motion_limits
    ctx.check(
        "pivot travel is roughly 0..28 degrees",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower) < 1e-9
        and 0.42 <= limits.upper <= 0.56,
        details=f"limits={limits}",
    )

    open_jaw = ctx.part_element_world_aabb(upper, elem="jaw_body")
    open_grip = ctx.part_element_world_aabb(upper, elem="grip")
    with ctx.pose({pivot: CLOSE_TRAVEL}):
        ctx.expect_contact(
            lower,
            upper,
            elem_a="jaw_body",
            elem_b="jaw_body",
            contact_tol=0.002,
            name="jaw faces meet when fully closed",
        )
        closed_jaw = ctx.part_element_world_aabb(upper, elem="jaw_body")
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
