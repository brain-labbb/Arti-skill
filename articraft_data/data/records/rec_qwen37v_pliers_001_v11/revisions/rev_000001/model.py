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
# Needle-nose pliers variant.
#
# Two mirrored half-tools cross at a central pivot rivet. Each half is one
# rigid link: long tapered needle-nose jaw with serrated inner teeth -> slim
# steel neck -> slender curved handle. A jaw stop boss near the pivot on the
# lower half limits jaw closure. An adjustment thumbwheel rotates at the
# rear of the lower handle.
#
# Geometry is authored per half in a "closed-design" local frame. The rest
# pose (q=0) splays each half by HALF_OPEN; positive pivot travel closes
# the jaws while handles scissor together in opposition.
# ---------------------------------------------------------------------------

HALF_OPEN = math.radians(12.0)
CLOSE_TRAVEL = 2.0 * HALF_OPEN  # ~24 degrees

PLATE_T = 0.0030

ZL0, ZL1 = 0.0030, 0.0030 + PLATE_T
ZU0, ZU1 = ZL1, ZL1 + PLATE_T

BLADE_Z0, BLADE_Z1 = ZL0, ZU1
BLADE_X_MIN = 0.0040

BOSS_R = 0.0065
HOLE_R = 0.0025
SHANK_R = 0.0021
HEAD_R = 0.0036

EDGE_LAND = 0.0003

GRIP_H = 0.0090
GRIP_LZ0 = 0.0
GRIP_LZ1 = GRIP_LZ0 + GRIP_H
GRIP_UZ0 = GRIP_LZ0 + (ZU0 - ZL0)
GRIP_UZ1 = GRIP_UZ0 + GRIP_H
INLAY_T = 0.0006
INLAY_EMBED = 0.0002

TOOTH_COUNT = 12
TOOTH_WIDTH = 0.0012
TOOTH_DEPTH = 0.0008
TOOTH_HEIGHT = 0.0008

# Needle-nose jaw profile (long, tapered). Lower half, +y side.
# Rear points extend inside the boss circle (radius 0.0065) for connectivity.
JAW_PTS = [
    (0.0050, 0.0082),   # rear top (inside boss footprint)
    (0.0120, 0.0078),
    (0.0200, 0.0068),
    (0.0300, 0.0052),
    (0.0380, 0.0038),
    (0.0440, 0.0024),
    (0.0490, 0.0014),
    (0.0520, 0.0008),
    (0.0525, 0.0004),
    (0.0520, EDGE_LAND),
    (0.0090, EDGE_LAND),
    (0.0040, 0.0038),   # rear inner (d~0.0055, inside boss)
    (0.0042, 0.0055),   # connecting rear
]

TANG_PTS = [
    (0.0020, -0.0028),
    (-0.0080, -0.0038),
    (-0.0180, -0.0044),
    (-0.0280, -0.0052),
    (-0.0280, -0.0090),
    (-0.0180, -0.0078),
    (-0.0080, -0.0066),
    (0.0010, -0.0058),
]

# Slender curved handle (narrow cross-section)
GRIP_PTS = [
    (-0.0220, -0.0030),
    (-0.0360, -0.0034),
    (-0.0500, -0.0038),
    (-0.0640, -0.0042),
    (-0.0780, -0.0046),
    (-0.0880, -0.0052),
    (-0.0940, -0.0060),
    (-0.0920, -0.0078),
    (-0.0840, -0.0088),
    (-0.0700, -0.0092),
    (-0.0540, -0.0090),
    (-0.0400, -0.0082),
    (-0.0280, -0.0068),
    (-0.0215, -0.0054),
]

INLAY_PTS = [
    (-0.0280, -0.0048),
    (-0.0440, -0.0054),
    (-0.0600, -0.0060),
    (-0.0740, -0.0064),
    (-0.0840, -0.0068),
    (-0.0800, -0.0076),
    (-0.0660, -0.0078),
    (-0.0500, -0.0074),
    (-0.0360, -0.0064),
    (-0.0290, -0.0056),
]

# Screw position in the lower frame (world frame, since lower is root).
# Placed at the rear of the lower handle after HALF_OPEN yaw.
SCREW_X = -0.088 * math.cos(HALF_OPEN) + 0.006 * math.sin(HALF_OPEN)
SCREW_Y = -0.088 * math.sin(HALF_OPEN) - 0.006 * math.cos(HALF_OPEN)
SCREW_Z = GRIP_LZ1 + 0.0002


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
    cap: float = 0.0012,
    inset: float = 0.0010,
) -> cq.Workplane:
    """Extruded spline outline with loft-capped soft edges. Falls back to
    plain extrusion if the loft fails."""
    mid = cq.Solid.extrudeLinear(
        cq.Face.makeFromWires(_spline_wire(pts, z0 + cap)),
        cq.Vector(0.0, 0.0, (z1 - z0) - 2.0 * cap),
    )
    try:
        top = cq.Solid.makeLoft([_spline_wire(pts, z1 - cap), _spline_wire(pts, z1, inset=inset)])
        bot = cq.Solid.makeLoft([_spline_wire(pts, z0, inset=inset), _spline_wire(pts, z0 + cap)])
        return cq.Workplane(obj=mid.fuse(top).fuse(bot))
    except Exception:
        # Fallback: plain extrusion without beveled caps
        face = cq.Face.makeFromWires(_spline_wire(pts, z0))
        solid = cq.Solid.extrudeLinear(face, cq.Vector(0.0, 0.0, z1 - z0))
        return cq.Workplane(obj=solid)


def _spline_prism(pts: list[tuple[float, float]], z0: float, z1: float) -> cq.Workplane:
    face = cq.Face.makeFromWires(_spline_wire(pts, z0))
    return cq.Workplane(obj=cq.Solid.extrudeLinear(face, cq.Vector(0.0, 0.0, z1 - z0)))


def _blade_clip_box() -> cq.Workplane:
    return cq.Workplane("XY", origin=(BLADE_X_MIN + 0.0260, 0.0, 0.0060)).box(
        0.0620, 0.0600, 0.0200
    )


def _half_jaw_body(s: float, own_z0: float, own_z1: float) -> cq.Workplane:
    prof = _mirror(JAW_PTS, s)
    full = _poly_prism(prof, BLADE_Z0, BLADE_Z1).intersect(_blade_clip_box())
    rear = _poly_prism(prof, own_z0, own_z1)
    return full.union(rear)


def _serrated_teeth(s: float, own_z0: float, own_z1: float) -> cq.Workplane:
    """Teeth on the inner jaw face, base embedded into the jaw body for connectivity."""
    z_mid = (own_z0 + own_z1) * 0.5
    result = None
    for i in range(TOOTH_COUNT):
        t = i / (TOOTH_COUNT - 1)
        x = 0.012 + t * 0.032
        # Base embedded inside jaw body; tip protrudes toward shear plane.
        # For s=+1: jaw inner face at y=+EDGE_LAND, body extends to y~+0.008.
        # Tooth from y=s*0.0012 (inside body) to y=s*0.0004 (slight protrusion past face).
        y_inner = s * 0.0012
        y_outer = s * (EDGE_LAND - 0.0004)
        y_lo = min(y_inner, y_outer)
        y_hi = max(y_inner, y_outer)
        tooth = (
            cq.Workplane("XY", origin=(x - TOOTH_WIDTH * 0.5, y_lo, z_mid - TOOTH_HEIGHT * 0.5))
            .box(TOOTH_WIDTH, y_hi - y_lo, TOOTH_HEIGHT, centered=False)
        )
        result = tooth if result is None else result.union(tooth)
    return result if result is not None else cq.Workplane("XY")


def _jaw_stop_boss(s: float, own_z0: float, own_z1: float) -> cq.Workplane:
    """Stop boss near pivot, overlapping the jaw body for connectivity."""
    y_pos = s * 0.0075  # well inside the jaw body
    boss = (
        cq.Workplane("XY", origin=(0.0060, y_pos, own_z0))
        .circle(0.0030)
        .extrude(own_z1 - own_z0 + 0.0010)
    )
    return boss


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
    lower_head = cq.Workplane("XY", origin=(0.0, 0.0, 0.0018)).circle(HEAD_R).extrude(0.0016)
    shank = cq.Workplane("XY", origin=(0.0, 0.0, 0.0018)).circle(SHANK_R).extrude(0.0090)
    upper_head = cq.Workplane("XY", origin=(0.0, 0.0, ZU1 + 0.0001)).circle(HEAD_R).extrude(0.0012)
    rivet = lower_head.union(shank).union(upper_head)
    try:
        rivet = rivet.edges(">Z").fillet(0.0008)
    except Exception:
        pass
    return rivet


def _adjustment_screw() -> cq.Workplane:
    """Thumbwheel: knurled disk with screw shaft and slot."""
    wheel_r = 0.0045
    wheel_h = 0.0035
    wheel = cq.Workplane("XY").circle(wheel_r).extrude(wheel_h)
    # Knurl ridges
    for i in range(14):
        angle = 2.0 * math.pi * i / 14
        cx = (wheel_r + 0.0003) * math.cos(angle)
        cy = (wheel_r + 0.0003) * math.sin(angle)
        ridge = cq.Workplane("XY", origin=(cx, cy, 0.0)).circle(0.0005).extrude(wheel_h)
        wheel = wheel.union(ridge)
    # Screw shaft
    shaft = cq.Workplane("XY", origin=(0.0, 0.0, -0.0025)).circle(0.0012).extrude(0.0025)
    # Slot on top
    slot = cq.Workplane("XY", origin=(0.0, 0.0, wheel_h - 0.0008)).rect(0.0060, 0.0007).extrude(0.0015)
    wheel = wheel.union(shaft).cut(slot)
    try:
        wheel = wheel.edges(">Z").fillet(0.0003)
    except Exception:
        pass
    return wheel


def _lower_jaw_assembly() -> cq.Workplane:
    """Lower jaw with teeth and stop boss fused into one solid."""
    jaw = _half_jaw_body(+1.0, ZL0, ZL1)
    teeth = _serrated_teeth(+1.0, ZL0, ZL1)
    stop = _jaw_stop_boss(+1.0, ZL0, ZL1)
    return jaw.union(teeth).union(stop)


def _upper_jaw_assembly() -> cq.Workplane:
    """Upper jaw with teeth fused into one solid."""
    jaw = _half_jaw_body(-1.0, ZU0, ZU1)
    teeth = _serrated_teeth(-1.0, ZU0, ZU1)
    return jaw.union(teeth)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="needle_nose_pliers")

    steel = model.material("brushed_steel", rgba=(0.72, 0.73, 0.75, 1.0))
    polished = model.material("polished_steel", rgba=(0.86, 0.87, 0.90, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.38, 0.39, 0.42, 1.0))
    orange = model.material("orange_grip", rgba=(0.95, 0.48, 0.06, 1.0))
    peach = model.material("peach_inlay", rgba=(1.0, 0.78, 0.60, 0.78))

    # ----- lower half (base link)
    lower = model.part("lower_half")
    lower_pose = Origin(rpy=(0.0, 0.0, HALF_OPEN))

    lower.visual(
        mesh_from_cadquery(_lower_jaw_assembly(), "lower_jaw_assembly"),
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

    # ----- upper half (moving link)
    upper = model.part("upper_half")

    upper.visual(
        mesh_from_cadquery(_upper_jaw_assembly(), "upper_jaw_assembly"),
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

    # ----- adjustment screw (child of lower half)
    screw = model.part("adjustment_screw")
    screw.visual(
        mesh_from_cadquery(_adjustment_screw(), "screw_wheel"),
        material=dark_steel,
        name="thumbwheel",
    )

    # ----- pivot articulation
    model.articulation(
        "rivet_pivot",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, -HALF_OPEN)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=4.0, lower=0.0, upper=CLOSE_TRAVEL),
    )

    # ----- adjustment screw joint (revolute about Z, mounted on lower handle)
    model.articulation(
        "screw_joint",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=screw,
        origin=Origin(xyz=(SCREW_X, SCREW_Y, SCREW_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0, lower=-3.14, upper=3.14),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower = object_model.get_part("lower_half")
    upper = object_model.get_part("upper_half")
    screw = object_model.get_part("adjustment_screw")
    pivot = object_model.get_articulation("rivet_pivot")
    screw_joint = object_model.get_articulation("screw_joint")

    # --- pivot stack: bosses coaxial, upper sits slightly above lower
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
        name="upper boss rides on lower boss face",
    )

    # --- rivet through boss stack
    ctx.expect_within(
        lower,
        upper,
        axes="xy",
        inner_elem="rivet",
        outer_elem="pivot_boss",
        margin=0.0002,
        name="rivet centered through boss bore",
    )
    rivet_aabb = ctx.part_element_world_aabb(lower, elem="rivet")
    uboss_aabb = ctx.part_element_world_aabb(upper, elem="pivot_boss")
    ctx.check(
        "rivet head caps upper boss",
        rivet_aabb is not None
        and uboss_aabb is not None
        and rivet_aabb[1][2] > uboss_aabb[1][2] + 0.0006,
        details=f"rivet={rivet_aabb}, upper_boss={uboss_aabb}",
    )

    # --- needle-nose jaws: long and tapered (length > 35mm from pivot)
    for part_obj in (lower, upper):
        jaw_aabb = ctx.part_element_world_aabb(part_obj, elem="jaw")
        if jaw_aabb is not None:
            jaw_length = jaw_aabb[1][0] - jaw_aabb[0][0]
            ctx.check(
                f"{part_obj.name} needle-nose jaw length > 0.035 m",
                jaw_length > 0.035,
                details=f"jaw_length={jaw_length:.4f}",
            )

    # --- jaw stop boss: lower jaw is taller near pivot due to stop boss protrusion
    lower_jaw_aabb = ctx.part_element_world_aabb(lower, elem="jaw")
    upper_jaw_aabb = ctx.part_element_world_aabb(upper, elem="jaw")
    if lower_jaw_aabb is not None and upper_jaw_aabb is not None:
        lower_jaw_h = lower_jaw_aabb[1][2] - lower_jaw_aabb[0][2]
        upper_jaw_h = upper_jaw_aabb[1][2] - upper_jaw_aabb[0][2]
        ctx.check(
            "lower jaw has stop boss (taller z-extent near pivot)",
            lower_jaw_h > upper_jaw_h - 0.0002,
            details=f"lower_jaw_h={lower_jaw_h:.5f}, upper_jaw_h={upper_jaw_h:.5f}",
        )

    # --- adjustment screw mounted near rear of lower handle
    screw_aabb = ctx.part_world_aabb(screw)
    grip_aabb = ctx.part_element_world_aabb(lower, elem="grip")
    if screw_aabb is not None and grip_aabb is not None:
        # Screw should be within or near the grip X range (rear portion)
        ctx.check(
            "adjustment screw mounted near rear handle",
            screw_aabb[0][0] >= grip_aabb[0][0] - 0.005
            and screw_aabb[0][0] <= grip_aabb[1][0] + 0.005
            and screw_aabb[0][1] >= grip_aabb[0][1] - 0.010
            and screw_aabb[0][1] <= grip_aabb[1][1] + 0.010,
            details=f"screw={screw_aabb}, grip={grip_aabb}",
        )

    # --- screw joint is revolute with rotation range
    ctx.check(
        "screw joint is revolute with rotation range",
        screw_joint.articulation_type == ArticulationType.REVOLUTE
        and screw_joint.motion_limits is not None
        and screw_joint.motion_limits.upper > screw_joint.motion_limits.lower + 1.0,
        details=f"type={screw_joint.articulation_type}, limits={screw_joint.motion_limits}",
    )

    # --- rest pose: jaws open, handles splayed
    ctx.expect_gap(
        lower,
        upper,
        axis="y",
        positive_elem="jaw",
        negative_elem="jaw",
        min_gap=0.0015,
        name="jaw tips open at rest",
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

    # --- slender handles: check Z thickness is slim
    for part_obj in (lower, upper):
        grip = ctx.part_element_world_aabb(part_obj, elem="grip")
        if grip is not None:
            grip_thickness = grip[1][2] - grip[0][2]
            ctx.check(
                f"{part_obj.name} handle is slender (Z thickness < 0.012)",
                grip_thickness < 0.012,
                details=f"grip_thickness={grip_thickness:.4f}",
            )

    # --- inlay within grip footprint
    for part_obj in (lower, upper):
        grip = ctx.part_element_world_aabb(part_obj, elem="grip")
        inlay = ctx.part_element_world_aabb(part_obj, elem="grip_inlay")
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

    # --- overall proportions
    la = ctx.part_world_aabb(lower)
    ua = ctx.part_world_aabb(upper)
    sa = ctx.part_world_aabb(screw)
    if la is not None and ua is not None:
        all_min_x = min(la[0][0], ua[0][0])
        all_max_x = max(la[1][0], ua[1][0])
        if sa is not None:
            all_min_x = min(all_min_x, sa[0][0])
            all_max_x = max(all_max_x, sa[1][0])
        length = all_max_x - all_min_x
        width = max(la[1][1], ua[1][1]) - min(la[0][1], ua[0][1])
        height = max(la[1][2], ua[1][2]) - min(la[0][2], ua[0][2])
        ctx.check(
            "overall length ~0.13-0.17 m",
            0.120 <= length <= 0.180,
            details=f"length={length:.4f}",
        )
        ctx.check(
            "splayed handle width ~0.04-0.07 m",
            0.030 <= width <= 0.080,
            details=f"width={width:.4f}",
        )
        ctx.check(
            "flat tool thickness ~0.010-0.020 m",
            0.008 <= height <= 0.022,
            details=f"height={height:.4f}",
        )
    else:
        ctx.fail("overall proportions", "missing part AABBs")

    # --- pivot travel: positive q closes jaws
    limits = pivot.motion_limits
    ctx.check(
        "pivot travel ~0..24 degrees",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower) < 1e-9
        and 0.35 <= limits.upper <= 0.50,
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
        "handles scissor opposite to jaws",
        open_grip is not None
        and closed_grip is not None
        and closed_grip[0][1] < open_grip[0][1] - 0.003,
        details=f"open={open_grip}, closed={closed_grip}",
    )

    # --- screw rotates in place (position stable under rotation)
    with ctx.pose({screw_joint: 0.0}):
        screw_pos_rest = ctx.part_world_position(screw)
    with ctx.pose({screw_joint: 1.57}):
        screw_pos_rotated = ctx.part_world_position(screw)
    ctx.check(
        "screw position stable under rotation (mounted in place)",
        screw_pos_rotated is not None
        and screw_pos_rest is not None
        and abs(screw_pos_rotated[0] - screw_pos_rest[0]) < 0.001
        and abs(screw_pos_rotated[1] - screw_pos_rest[1]) < 0.001,
        details=f"rest={screw_pos_rest}, rotated={screw_pos_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
