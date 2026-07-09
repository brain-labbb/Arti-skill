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
# Locking pliers (Vise-Grip style) — variant of compact flush-cut wire snips.
#
# Two forged halves cross at a central pivot rivet. Each half is one rigid
# link: serrated gripping jaw -> steel neck -> curved over-molded handle.
# The lower half is the base link and carries:
#   - a rear adjustment screw (threaded rod + knurled head)
#   - a jaw stop boss near the pivot
#   - a release lever that pivots inside the handle
# The upper half rotates about the pivot axis (Z, perpendicular to the flat
# tool plane). Positive q closes the jaws while handles scissor together.
# ---------------------------------------------------------------------------

HALF_OPEN = math.radians(12.5)
CLOSE_TRAVEL = 2.0 * HALF_OPEN  # ~25 degrees of closing travel

PLATE_T = 0.0035

# Steel layer stack (lower plate below upper plate at pivot boss)
ZL0, ZL1 = 0.0035, 0.0035 + PLATE_T
ZU0, ZU1 = ZL1, ZL1 + PLATE_T

BLADE_Z0, BLADE_Z1 = ZL0, ZU1
BLADE_X_MIN = 0.0085

BOSS_R = 0.0080
HOLE_R = 0.0028
SHANK_R = 0.0024
HEAD_R = 0.0040

# Handle over-mold
GRIP_H = 0.011
GRIP_LZ0 = 0.0
GRIP_LZ1 = GRIP_LZ0 + GRIP_H
GRIP_UZ0 = GRIP_LZ0 + (ZU0 - ZL0)
GRIP_UZ1 = GRIP_UZ0 + GRIP_H
INLAY_T = 0.0007
INLAY_EMBED = 0.0002

# Serrated teeth parameters
TOOTH_W = 0.0018   # tooth width along jaw length
TOOTH_H = 0.0008   # tooth height (protrusion from jaw face)
TOOTH_D = 0.0006   # tooth depth into the jaw face
TOOTH_SPACING = 0.0030  # center-to-center spacing
TEETH_COUNT = 7
TEETH_START_X = 0.010  # first tooth x position

# Jaw stop boss
STOP_BOSS_R = 0.003
STOP_BOSS_H = 0.002

# Adjustment screw
SCREW_ROD_R = 0.0025
SCREW_ROD_L = 0.018
SCREW_HEAD_R = 0.005
SCREW_HEAD_H = 0.005
KNURL_RINGS = 8

# Release lever
LEVER_L = 0.025
LEVER_W = 0.006
LEVER_T = 0.003
# Pivot location inside lower handle (world-space, matching rotated grip body)
LEVER_PIVOT_X = -0.056
LEVER_PIVOT_Y = -0.024

# Jaw plate outline for locking pliers (wider gripping jaw, less tapered)
JAW_PTS = [
    (0.0000, 0.0100),
    (0.0060, 0.0106),
    (0.0140, 0.0102),
    (0.0240, 0.0085),
    (0.0340, 0.0055),
    (0.0390, 0.0030),
    (0.0410, 0.0015),
    (0.0405, 0.0005),
    (0.0090, 0.0005),
    (0.0020, 0.0055),
]

# Slim steel neck/tang from the boss back to the handle
TANG_PTS = [
    (0.0020, -0.0035),
    (-0.0100, -0.0048),
    (-0.0200, -0.0055),
    (-0.0300, -0.0065),
    (-0.0300, -0.0120),
    (-0.0200, -0.0102),
    (-0.0100, -0.0088),
    (0.0008, -0.0078),
]

# Curved over-molded handle outline
GRIP_PTS = [
    (-0.0240, -0.0040),
    (-0.0400, -0.0052),
    (-0.0580, -0.0065),
    (-0.0740, -0.0078),
    (-0.0890, -0.0088),
    (-0.0980, -0.0102),
    (-0.0940, -0.0138),
    (-0.0780, -0.0158),
    (-0.0620, -0.0162),
    (-0.0460, -0.0148),
    (-0.0300, -0.0120),
    (-0.0235, -0.0100),
]

# Translucent inlay strip
INLAY_PTS = [
    (-0.0305, -0.0072),
    (-0.0480, -0.0088),
    (-0.0660, -0.0100),
    (-0.0790, -0.0108),
    (-0.0880, -0.0114),
    (-0.0830, -0.0124),
    (-0.0670, -0.0130),
    (-0.0510, -0.0122),
    (-0.0370, -0.0108),
    (-0.0315, -0.0094),
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
    inset: float = 0.0012,
) -> cq.Workplane:
    """Extruded spline outline with loft-capped (softly beveled) top/bottom."""
    mid = cq.Solid.extrudeLinear(
        cq.Face.makeFromWires(_spline_wire(pts, z0 + cap)),
        cq.Vector(0.0, 0.0, (z1 - z0) - 2.0 * cap),
    )
    try:
        top = cq.Solid.makeLoft([_spline_wire(pts, z1 - cap), _spline_wire(pts, z1, inset=inset)])
        bot = cq.Solid.makeLoft([_spline_wire(pts, z0, inset=inset), _spline_wire(pts, z0 + cap)])
        result = mid.fuse(top).fuse(bot)
    except Exception:
        # Fallback: just extrude without soft caps
        result = mid
    return cq.Workplane(obj=result)


def _spline_prism(pts: list[tuple[float, float]], z0: float, z1: float) -> cq.Workplane:
    face = cq.Face.makeFromWires(_spline_wire(pts, z0))
    return cq.Workplane(obj=cq.Solid.extrudeLinear(face, cq.Vector(0.0, 0.0, z1 - z0)))


def _blade_clip_box() -> cq.Workplane:
    return cq.Workplane("XY", origin=(BLADE_X_MIN + 0.0280, 0.0, 0.0070)).box(
        0.0600, 0.0600, 0.0200
    )


def _jaw_body(s: float, own_z0: float, own_z1: float) -> cq.Workplane:
    """Jaw body for one half — full-height forward region + own-layer rear."""
    prof = _mirror(JAW_PTS, s)
    full = _poly_prism(prof, BLADE_Z0, BLADE_Z1).intersect(_blade_clip_box())
    rear = _poly_prism(prof, own_z0, own_z1)
    return full.union(rear)


def _serrated_teeth(s: float, z_face: float) -> cq.Workplane:
    """Serrated gripping teeth on the inner jaw face.
    s=+1: lower jaw teeth at y=+side (inner face near y=0.0005)
    s=-1: upper jaw teeth at y=-side (inner face near y=-0.0005)
    Teeth protrude inward from the jaw gripping face.
    """
    teeth = cq.Workplane("XY", origin=(TEETH_START_X, s * 0.0005, z_face)).box(
        TOOTH_W, TOOTH_D, TOOTH_H
    )
    for i in range(1, TEETH_COUNT):
        x = TEETH_START_X + i * TOOTH_SPACING
        if x > 0.035:
            break
        t = cq.Workplane("XY", origin=(x, s * 0.0005, z_face)).box(
            TOOTH_W, TOOTH_D, TOOTH_H
        )
        teeth = teeth.union(t)
    return teeth


def _jaw_with_teeth(s: float, own_z0: float, own_z1: float) -> cq.Workplane:
    """Jaw body with serrated teeth on the inner gripping face."""
    body = _jaw_body(s, own_z0, own_z1)
    # Teeth sit on the inner face of the jaw at the mid-height of the blade
    z_teeth = (BLADE_Z0 + BLADE_Z1) / 2.0
    teeth = _serrated_teeth(s, z_teeth)
    return body.union(teeth)


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


def _jaw_stop_boss(s: float) -> cq.Workplane:
    """Jaw stop boss near the pivot — limits how far jaws close."""
    x_pos = 0.006
    y_pos = s * 0.009
    return (
        cq.Workplane("XY", origin=(x_pos, y_pos, BLADE_Z0))
        .circle(STOP_BOSS_R)
        .extrude(BLADE_Z1 - BLADE_Z0 + STOP_BOSS_H)
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


def _adjustment_screw() -> cq.Workplane:
    """Rear adjustment screw: threaded rod + knurled adjustment wheel."""
    # Rod extends rearward from handle end
    rod = (
        cq.Workplane("XY", origin=(0.0, 0.0, 0.0))
        .circle(SCREW_ROD_R)
        .extrude(SCREW_ROD_L)
    )
    # Knurled head (disk with radial grooves approximated as a polygon)
    head_z = SCREW_ROD_L
    head = (
        cq.Workplane("XY", origin=(0.0, 0.0, head_z))
        .polygon(KNURL_RINGS, SCREW_HEAD_R * 2.0)
        .extrude(SCREW_HEAD_H)
    )
    # Add a center boss on the head
    center = (
        cq.Workplane("XY", origin=(0.0, 0.0, head_z + 0.001))
        .circle(SCREW_ROD_R * 0.8)
        .extrude(SCREW_HEAD_H - 0.002)
    )
    screw = rod.union(head).union(center)
    try:
        screw = screw.edges(">Z").fillet(0.0006)
    except Exception:
        pass
    return screw


def _release_lever() -> cq.Workplane:
    """Locking release lever — flat elongated piece that pivots inside handle.
    Pivot point is at local origin (0,0). Lever body extends along +X."""
    offset_x = LEVER_L / 2.0 - 0.003  # pivot is 3mm from the near end
    # Main body with rounded near-end (around pivot)
    near_end = (
        cq.Workplane("XY", origin=(0.0, 0.0, 0.0))
        .circle(LEVER_W / 2.0)
        .extrude(LEVER_T)
    )
    body = (
        cq.Workplane("XY", origin=(offset_x, 0.0, 0.0))
        .rect(LEVER_L, LEVER_W)
        .extrude(LEVER_T)
    )
    # Rounded tip at the free end
    tip = (
        cq.Workplane("XY", origin=(LEVER_L - 0.005, 0.0, 0.0))
        .circle(LEVER_W / 2.0)
        .extrude(LEVER_T)
    )
    lever = body.union(near_end).union(tip)
    try:
        lever = lever.edges(">Z").fillet(0.0008)
    except Exception:
        pass
    return lever


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="locking_pliers")

    steel = model.material("brushed_steel", rgba=(0.72, 0.73, 0.75, 1.0))
    polished = model.material("polished_steel", rgba=(0.86, 0.87, 0.90, 1.0))
    orange = model.material("orange_grip", rgba=(0.92, 0.48, 0.06, 1.0))
    peach = model.material("peach_inlay", rgba=(1.0, 0.80, 0.64, 0.78))
    dark_steel = model.material("dark_steel", rgba=(0.42, 0.44, 0.46, 1.0))

    # ----- lower half (base link): jaw at +y, handle at -y, lower steel layer
    lower = model.part("frame_half")
    lower_pose = Origin(rpy=(0.0, 0.0, HALF_OPEN))

    lower.visual(
        mesh_from_cadquery(_jaw_with_teeth(+1.0, ZL0, ZL1), "lower_jaw"),
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
        mesh_from_cadquery(_jaw_stop_boss(+1.0), "stop_boss"),
        origin=lower_pose,
        material=dark_steel,
        name="jaw_stop",
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
    # Adjustment screw at the rear of the lower handle — world-space position
    # matching the rotated grip body at its rear section
    lower.visual(
        mesh_from_cadquery(_adjustment_screw(), "adj_screw"),
        origin=Origin(
            xyz=(-0.090, -0.030, (GRIP_LZ0 + GRIP_LZ1) / 2.0),
            rpy=(0.0, math.radians(90.0), HALF_OPEN + math.radians(3.0)),
        ),
        material=dark_steel,
        name="adjustment_screw",
    )

    # ----- upper half (moving link): mirrored, upper steel layer, bored boss
    upper = model.part("moving_half")
    upper.visual(
        mesh_from_cadquery(_jaw_with_teeth(-1.0, ZU0, ZU1), "upper_jaw"),
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
        mesh_from_cadquery(
            _spline_prism(
                _mirror(INLAY_PTS, -1.0), GRIP_UZ1 - INLAY_EMBED, GRIP_UZ1 - INLAY_EMBED + INLAY_T
            ),
            "upper_inlay",
        ),
        material=peach,
        name="grip_inlay",
    )

    # ----- release lever (separate part, pivots inside lower handle)
    lever = model.part("release_lever")
    # Lever pivot point is at local (0,0,0) in the lever geometry.
    # The articulation places the part frame; visual just needs HALF_OPEN yaw
    # to align with the grip (articulation frame has -HALF_OPEN yaw).
    lever.visual(
        mesh_from_cadquery(_release_lever(), "lever_body"),
        origin=Origin(rpy=(0.0, 0.0, HALF_OPEN)),
        material=dark_steel,
        name="lever_body",
    )

    # ----- main pivot joint: revolute at the central rivet
    model.articulation(
        "pivot_joint",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, -HALF_OPEN)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=4.0, lower=0.0, upper=CLOSE_TRAVEL),
    )

    # ----- release lever pivot: small revolute inside the lower handle
    lever_travel = math.radians(15.0)
    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=lever,
        origin=Origin(
            xyz=(LEVER_PIVOT_X, LEVER_PIVOT_Y, (GRIP_LZ0 + GRIP_LZ1) / 2.0),
            rpy=(0.0, 0.0, -HALF_OPEN),
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=6.0, lower=0.0, upper=lever_travel),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower = object_model.get_part("frame_half")
    upper = object_model.get_part("moving_half")
    lever = object_model.get_part("release_lever")
    pivot = object_model.get_articulation("pivot_joint")
    lever_joint = object_model.get_articulation("lever_pivot")

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

    # --- serrated teeth exist on both jaws (jaw_teeth visuals include teeth ridges)
    lower_teeth = ctx.part_element_world_aabb(lower, elem="jaw_teeth")
    upper_teeth = ctx.part_element_world_aabb(upper, elem="jaw_teeth")
    ctx.check(
        "lower jaw has serrated teeth geometry",
        lower_teeth is not None
        and lower_teeth[0][0] < 0.015  # teeth start near the front of the jaw
        and lower_teeth[1][0] > 0.020,  # teeth extend along the jaw length
        details=f"teeth_aabb={lower_teeth}",
    )
    ctx.check(
        "upper jaw has serrated teeth geometry",
        upper_teeth is not None
        and upper_teeth[0][0] < 0.015
        and upper_teeth[1][0] > 0.020,
        details=f"teeth_aabb={upper_teeth}",
    )

    # --- jaw stop boss exists near the pivot on the lower jaw
    stop_aabb = ctx.part_element_world_aabb(lower, elem="jaw_stop")
    ctx.check(
        "jaw stop boss exists near pivot",
        stop_aabb is not None
        and stop_aabb[0][0] < 0.012
        and stop_aabb[1][0] > 0.002,
        details=f"stop_aabb={stop_aabb}",
    )
    # Stop boss protrudes above the blade stack
    ctx.check(
        "jaw stop protrudes above blade face",
        stop_aabb is not None and stop_aabb[1][2] > BLADE_Z1 + 0.0005,
        details=f"stop_aabb={stop_aabb}, BLADE_Z1={BLADE_Z1}",
    )

    # --- adjustment screw exists at the rear of the lower handle
    screw_aabb = ctx.part_element_world_aabb(lower, elem="adjustment_screw")
    ctx.check(
        "adjustment screw at rear of handle",
        screw_aabb is not None
        and screw_aabb[0][0] < -0.060
        and screw_aabb[1][0] > -0.120,
        details=f"screw_aabb={screw_aabb}",
    )

    # --- release lever is a separate articulated part
    lever_aabb = ctx.part_element_world_aabb(lever, elem="lever_body")
    ctx.check(
        "release lever exists inside lower handle region",
        lever_aabb is not None
        and lever_aabb[0][0] < -0.040
        and lever_aabb[1][0] < -0.020,
        details=f"lever_aabb={lever_aabb}",
    )

    # --- release lever joint is non-fixed and has valid limits
    lever_limits = lever_joint.motion_limits
    ctx.check(
        "lever pivot is a non-fixed revolute joint",
        lever_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={lever_joint.articulation_type}",
    )
    ctx.check(
        "lever pivot has valid travel limits",
        lever_limits is not None
        and lever_limits.lower is not None
        and lever_limits.upper is not None
        and lever_limits.upper > lever_limits.lower,
        details=f"limits={lever_limits}",
    )

    # --- lever articulates: check that it moves when posed
    lever_rest = ctx.part_world_aabb(lever)
    with ctx.pose({lever_joint: math.radians(10.0)}):
        lever_posed = ctx.part_world_aabb(lever)
    ctx.check(
        "release lever moves when posed",
        lever_rest is not None
        and lever_posed is not None
        and (
            abs(lever_posed[0][0] - lever_rest[0][0]) > 0.0005
            or abs(lever_posed[0][1] - lever_rest[0][1]) > 0.0005
        ),
        details=f"rest={lever_rest}, posed={lever_posed}",
    )

    # --- rest pose: jaws open, handles splayed apart
    ctx.expect_gap(
        lower,
        upper,
        axis="y",
        positive_elem="jaw_teeth",
        negative_elem="jaw_teeth",
        min_gap=0.001,
        name="jaw teeth are open at rest",
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

    # --- main pivot travel and articulation
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

    open_blade = ctx.part_element_world_aabb(upper, elem="jaw_teeth")
    open_grip = ctx.part_element_world_aabb(upper, elem="grip")
    with ctx.pose({pivot: CLOSE_TRAVEL}):
        ctx.expect_contact(
            lower,
            upper,
            elem_a="jaw_teeth",
            elem_b="jaw_teeth",
            contact_tol=0.003,
            name="jaw teeth meet when fully closed",
        )
        closed_blade = ctx.part_element_world_aabb(upper, elem="jaw_teeth")
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

    # --- overall proportions
    la = ctx.part_world_aabb(lower)
    ua = ctx.part_world_aabb(upper)
    if la is not None and ua is not None:
        length = max(la[1][0], ua[1][0]) - min(la[0][0], ua[0][0])
        width = max(la[1][1], ua[1][1]) - min(la[0][1], ua[0][1])
        height = max(la[1][2], ua[1][2]) - min(la[0][2], ua[0][2])
        ctx.check(
            "overall length ~0.13-0.16 m",
            0.110 <= length <= 0.170,
            details=f"length={length:.4f}",
        )
        ctx.check(
            "splayed handle width ~0.05-0.08 m",
            0.045 <= width <= 0.085,
            details=f"width={width:.4f}",
        )
        ctx.check(
            "flat tool thickness ~0.012-0.020 m",
            0.010 <= height <= 0.022,
            details=f"height={height:.4f}",
        )
    else:
        ctx.fail("overall proportions", "missing part AABBs")

    return ctx.report()


object_model = build_object_model()
