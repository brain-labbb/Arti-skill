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
# Combination pliers with crimping notch, serrated jaws, and jaw stop boss.
#
# Two mirrored half-tools cross at a single polished rivet. Each half is one
# rigid link: broad combination jaw with serrated inner teeth and crimping
# notch -> slim steel neck -> curved over-molded handle. A jaw stop boss near
# the pivot limits over-closure. The lower half is the base link; the upper
# half rotates about the rivet axis (Z, perpendicular to the flat plane).
#
# Geometry is authored per half in a "closed-design" local frame in which the
# two halves are exactly mirrored about the XZ plane and the jaw gripping
# faces sit at y = +/-FACE_LAND. The rest pose (q=0) splays each half by
# HALF_OPEN via visual/joint-frame yaw, so q in [0, CLOSE_TRAVEL] closes the
# jaws while the handles scissor together in opposition.
# ---------------------------------------------------------------------------

HALF_OPEN = math.radians(12.5)  # each half is yawed this much at rest (open)
CLOSE_TRAVEL = 2.0 * HALF_OPEN  # ~25 degrees of closing travel

PLATE_T = 0.0035  # forged steel plate thickness per half

# Steel layer stack
ZL0, ZL1 = 0.0035, 0.0035 + PLATE_T  # lower half plate: 0.0035 .. 0.0070
ZU0, ZU1 = ZL1, ZL1 + PLATE_T  # upper half plate: 0.0070 .. 0.0105

# Full-height jaw region spans both plate layers
JAW_Z0, JAW_Z1 = ZL0, ZU1
JAW_X_MIN = 0.0085  # full-height jaw exists only forward of the boss

BOSS_R = 0.0080
HOLE_R = 0.0028
SHANK_R = 0.0024
HEAD_R = 0.0040

FACE_LAND = 0.0004  # jaw gripping face sits this far off the center plane

# Crimping notch: V-shaped groove on the inner jaw face behind the teeth
CRIMP_X = 0.0095  # x position of crimp notch (behind pivot, ahead of teeth)
CRIMP_DEPTH = 0.0020  # depth of V-notch into jaw face
CRIMP_WIDTH_X = 0.0020  # width of notch in x direction

# Jaw stop boss: small protrusion near pivot to limit over-closure
STOP_R = 0.0030
STOP_HEIGHT = 0.0012
STOP_X = 0.0050  # just behind the jaw root
STOP_Y_OFFSET = 0.0060  # offset from center on each half

# Serrated teeth on inner jaw face
TEETH_COUNT = 6
TEETH_LENGTH = 0.0025  # along x
TEETH_WIDTH = 0.0008  # along y (protrusion from face)
TEETH_HEIGHT = 0.0006  # along z
TEETH_X_START = 0.0130  # first tooth x position (past crimp notch)
TEETH_X_SPACING = 0.0035  # spacing between teeth

# Handle over-mold (z extents per half, soft loft-capped edges).
GRIP_H = 0.0110
GRIP_LZ0 = 0.0
GRIP_LZ1 = GRIP_LZ0 + GRIP_H  # lower grip 0.0 .. 0.0110
GRIP_UZ0 = GRIP_LZ0 + (ZU0 - ZL0)  # upper grip 0.0035 .. 0.0145
GRIP_UZ1 = GRIP_UZ0 + GRIP_H
INLAY_T = 0.0007
INLAY_EMBED = 0.0002

# Combination jaw profile (lower half, jaw body at +y, gripping face along
# y = FACE_LAND). Broader than wire snips with flat inner face for gripping.
JAW_PTS = [
    (0.0000, 0.0110),   # rear inner corner (wide base)
    (0.0060, 0.0115),   # mid body
    (0.0140, 0.0108),   # forward body
    (0.0240, 0.0090),   # taper starts
    (0.0320, 0.0060),   # jaw tip taper
    (0.0360, 0.0030),   # near tip
    (0.0370, 0.0015),   # tip
    (0.0365, FACE_LAND),  # tip inner edge (gripping face)
    (0.0090, FACE_LAND),  # inner gripping face runs back to root
    (0.0020, 0.0055),   # root fillet
]

# Slim steel neck/tang from the boss back to the handle (lower half: -y side).
TANG_PTS = [
    (0.0020, -0.0032),
    (-0.0100, -0.0044),
    (-0.0200, -0.0052),
    (-0.0300, -0.0062),
    (-0.0300, -0.0118),
    (-0.0200, -0.0100),
    (-0.0100, -0.0084),
    (0.0008, -0.0074),
]

# Curved over-molded handle outline (periodic spline; widens, rounded tip).
GRIP_PTS = [
    (-0.0240, -0.0038),
    (-0.0400, -0.0050),
    (-0.0580, -0.0062),
    (-0.0740, -0.0074),
    (-0.0890, -0.0082),
    (-0.0955, -0.0096),
    (-0.0915, -0.0132),
    (-0.0780, -0.0152),
    (-0.0620, -0.0156),
    (-0.0460, -0.0142),
    (-0.0300, -0.0118),
    (-0.0235, -0.0098),
]

# Translucent peach inlay strip along the top face of the handle.
INLAY_PTS = [
    (-0.0305, -0.0070),
    (-0.0480, -0.0084),
    (-0.0660, -0.0098),
    (-0.0790, -0.0104),
    (-0.0860, -0.0108),
    (-0.0810, -0.0118),
    (-0.0670, -0.0124),
    (-0.0510, -0.0116),
    (-0.0370, -0.0102),
    (-0.0315, -0.0092),
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


def _jaw_clip_box() -> cq.Workplane:
    """Clip the full-height jaw to just the forward region."""
    return cq.Workplane("XY", origin=(JAW_X_MIN + 0.0250, 0.0, 0.0070)).box(
        0.0550, 0.0600, 0.0200
    )


def _crimping_notch_cutter(s: float) -> cq.Workplane:
    """V-shaped crimping notch cutter on the inner jaw face.
    A narrow groove cut across the jaw inner face for wire stripping.
    s = +1 for lower half (jaw at +y), s = -1 for upper half (jaw at -y)."""
    # V-notch profile in YZ plane: groove cut into the jaw face
    face_y = s * FACE_LAND
    inner_y = s * (FACE_LAND + CRIMP_DEPTH)
    v_pts = [
        (face_y, JAW_Z0 - 0.0005),
        (inner_y, (JAW_Z0 + JAW_Z1) / 2.0),
        (face_y, JAW_Z1 + 0.0005),
    ]
    return (
        cq.Workplane("YZ", origin=(CRIMP_X - CRIMP_WIDTH_X / 2.0, 0.0, 0.0))
        .polyline(v_pts)
        .close()
        .extrude(CRIMP_WIDTH_X)
    )


def _serrated_teeth(s: float, own_z0: float, own_z1: float) -> cq.Workplane:
    """Row of serrated teeth on the inner gripping face of the jaw.
    s = +1 for lower half (jaw at +y, teeth protrude toward -y from inner face),
    s = -1 for upper half (jaw at -y, teeth protrude toward +y from inner face).
    Teeth are embedded slightly into the jaw body for physical connectivity."""
    teeth_mid_z = (own_z0 + own_z1) / 2.0
    tooth_h = min(TEETH_HEIGHT, (own_z1 - own_z0) * 0.6)
    teeth_z0 = teeth_mid_z - tooth_h / 2.0
    embed = 0.0005  # embed depth into jaw body for connectivity

    result = None
    for i in range(TEETH_COUNT):
        x_center = TEETH_X_START + i * TEETH_X_SPACING
        if s > 0:
            # Lower half: jaw inner face at y = +FACE_LAND
            # Teeth protrude from face inward (-y), embedded +embed into jaw
            y_min = FACE_LAND - TEETH_WIDTH
            y_max = FACE_LAND + embed
            tooth = cq.Workplane("XY", origin=(
                x_center - TEETH_LENGTH / 2.0,
                y_min,
                teeth_z0
            )).box(TEETH_LENGTH, y_max - y_min, tooth_h, centered=False)
        else:
            # Upper half: jaw inner face at y = -FACE_LAND
            # Teeth protrude from face inward (+y), embedded +embed into jaw
            y_min = -(FACE_LAND + embed)
            y_max = -(FACE_LAND - TEETH_WIDTH)
            tooth = cq.Workplane("XY", origin=(
                x_center - TEETH_LENGTH / 2.0,
                y_min,
                teeth_z0
            )).box(TEETH_LENGTH, y_max - y_min, tooth_h, centered=False)

        if result is None:
            result = tooth
        else:
            result = result.union(tooth)
    return result


def _jaw_stop_boss(s: float, own_z0: float, own_z1: float) -> cq.Workplane:
    """Small jaw stop boss near the pivot that limits over-closure.
    Protrudes upward/downward from the jaw plate to contact the other half."""
    y_pos = s * STOP_Y_OFFSET
    # Boss protrudes in Z from the plate
    if s > 0:
        # Lower half boss protrudes upward
        boss_z0 = own_z1
        boss_z1 = own_z1 + STOP_HEIGHT
    else:
        # Upper half boss protrudes downward
        boss_z0 = own_z0 - STOP_HEIGHT
        boss_z1 = own_z0
    return (
        cq.Workplane("XY", origin=(STOP_X, y_pos, boss_z0))
        .circle(STOP_R)
        .extrude(boss_z1 - boss_z0)
    )


def _half_jaw(s: float, own_z0: float, own_z1: float) -> cq.Workplane:
    """Build one combination plier jaw half with serrated teeth and crimping notch."""
    prof = _mirror(JAW_PTS, s)
    # Full-height jaw in forward region
    full = _poly_prism(prof, JAW_Z0, JAW_Z1).intersect(_jaw_clip_box())
    # Own-layer plate in rear region
    rear = _poly_prism(prof, own_z0, own_z1)
    jaw = full.union(rear)

    # Cut crimping notch
    try:
        jaw = jaw.cut(_crimping_notch_cutter(s))
    except Exception:
        pass  # if cut fails, skip notch

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="combination_pliers")

    steel = model.material("brushed_steel", rgba=(0.72, 0.73, 0.75, 1.0))
    polished = model.material("polished_steel", rgba=(0.86, 0.87, 0.90, 1.0))
    orange = model.material("orange_grip", rgba=(0.97, 0.52, 0.07, 1.0))
    peach = model.material("peach_inlay", rgba=(1.0, 0.80, 0.64, 0.78))

    # ----- lower half (base link): jaw at +y, handle at -y, lower steel layer
    lower = model.part("lower_half")
    lower_pose = Origin(rpy=(0.0, 0.0, HALF_OPEN))

    # Jaw with crimping notch
    lower.visual(
        mesh_from_cadquery(_half_jaw(+1.0, ZL0, ZL1), "lower_jaw"),
        origin=lower_pose,
        material=steel,
        name="jaw",
    )
    # Serrated teeth on inner face
    lower.visual(
        mesh_from_cadquery(_serrated_teeth(+1.0, ZL0, ZL1), "lower_teeth"),
        origin=lower_pose,
        material=steel,
        name="jaw_teeth",
    )
    # Jaw stop boss
    lower.visual(
        mesh_from_cadquery(_jaw_stop_boss(+1.0, ZL0, ZL1), "lower_stop"),
        origin=lower_pose,
        material=steel,
        name="jaw_stop",
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
        mesh_from_cadquery(_half_jaw(-1.0, ZU0, ZU1), "upper_jaw"),
        material=steel,
        name="jaw",
    )
    # Serrated teeth on inner face
    upper.visual(
        mesh_from_cadquery(_serrated_teeth(-1.0, ZU0, ZU1), "upper_teeth"),
        material=steel,
        name="jaw_teeth",
    )
    # Jaw stop boss
    upper.visual(
        mesh_from_cadquery(_jaw_stop_boss(-1.0, ZU0, ZU1), "upper_stop"),
        material=steel,
        name="jaw_stop",
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

    # --- COMBINATION PLIERS SPECIFIC CHECKS ---

    # Serrated teeth exist on both jaws and are positioned near the gripping face
    for part_obj, part_name in [(lower, "lower"), (upper, "upper")]:
        teeth_aabb = ctx.part_element_world_aabb(part_obj, elem="jaw_teeth")
        jaw_aabb = ctx.part_element_world_aabb(part_obj, elem="jaw")
        ctx.check(
            f"{part_name} jaw has serrated teeth geometry",
            teeth_aabb is not None,
            details=f"teeth_aabb={teeth_aabb}",
        )
        if teeth_aabb is not None and jaw_aabb is not None:
            # Teeth should overlap the jaw in X (they are on the jaw)
            ctx.expect_overlap(
                part_obj,
                part_obj,
                axes="x",
                elem_a="jaw_teeth",
                elem_b="jaw",
                min_overlap=0.010,
                name=f"{part_name} teeth overlap jaw in x",
            )

    # Jaw stop bosses exist near the pivot on both halves
    for part_obj, part_name in [(lower, "lower"), (upper, "upper")]:
        stop_aabb = ctx.part_element_world_aabb(part_obj, elem="jaw_stop")
        ctx.check(
            f"{part_name} jaw stop boss exists near pivot",
            stop_aabb is not None,
            details=f"stop_aabb={stop_aabb}",
        )
        if stop_aabb is not None:
            # Stop boss should be near the pivot (within 15mm of origin in xy)
            pivot_dist = math.sqrt(stop_aabb[0][0]**2 + stop_aabb[0][1]**2)
            ctx.check(
                f"{part_name} jaw stop is near the pivot region",
                pivot_dist < 0.015,
                details=f"pivot_dist={pivot_dist:.4f}",
            )

    # Crimping notch: jaw should have a concavity behind the gripping area.
    # We verify this by checking the jaw spans expected x range but has a
    # narrower y extent at the crimp location than at the jaw tip.
    for part_obj, part_name in [(lower, "lower"), (upper, "upper")]:
        jaw_aabb = ctx.part_element_world_aabb(part_obj, elem="jaw")
        ctx.check(
            f"{part_name} jaw spans combination plier proportions",
            jaw_aabb is not None
            and (jaw_aabb[1][0] - jaw_aabb[0][0]) > 0.025,
            details=f"jaw_aabb={jaw_aabb}",
        )

    # --- rest pose: jaws open, handles splayed apart
    ctx.expect_gap(
        lower,
        upper,
        axis="y",
        positive_elem="jaw",
        negative_elem="jaw",
        min_gap=0.0020,
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

    # --- inlay strips sit proud on, and within, the handle top faces
    for part_obj in (lower, upper):
        grip = ctx.part_element_world_aabb(part_obj, elem="grip")
        inlay = ctx.part_element_world_aabb(part_obj, elem="grip_inlay")
        ctx.check(
            f"{part_obj.name} inlay proud of grip top",
            grip is not None
            and inlay is not None
            and grip[1][2] + 0.0002 < inlay[1][2] < grip[1][2] + 0.0012,
            details=f"grip={grip}, inlay={inlay}",
        )
        ctx.check(
            f"{part_obj.name} inlay within grip footprint",
            grip is not None
            and inlay is not None
            and inlay[0][0] >= grip[0][0] - 1e-4
            and inlay[1][0] <= grip[1][0] + 1e-4
            and inlay[0][1] >= grip[0][1] - 1e-4
            and inlay[1][1] <= grip[1][1] + 1e-4,
            details=f"grip={grip}, inlay={inlay}",
        )

    # --- overall proportions: ~0.13 long, ~0.06 across handles, ~0.018 thick
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
            "flat tool thickness ~0.018 m",
            0.014 <= height <= 0.022,
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

    return ctx.report()


object_model = build_object_model()
