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
# Insulated locking pliers variant.
#
# Two mirrored half-tools cross at a central pivot rivet with visible circular
# caps on both sides. Each half: tapered jaw -> slim steel neck -> thick
# layered insulated grip sleeve (inner dark rubber + outer bright orange).
#
# A locking release lever pivots inside the lower handle near the pivot boss.
#
# The lower half is the base link; the upper half rotates about the rivet axis
# (Z, perpendicular to the flat plane of the tool). The lock lever has its own
# revolute joint with limited travel.
# ---------------------------------------------------------------------------

HALF_OPEN = math.radians(12.5)  # each half is yawed this much at rest (open)
CLOSE_TRAVEL = 2.0 * HALF_OPEN  # ~25 degrees of closing travel

PLATE_T = 0.0035  # forged steel plate thickness per half

# Steel layer stack
ZL0, ZL1 = 0.0035, 0.0035 + PLATE_T  # lower half plate: 0.0035 .. 0.0070
ZU0, ZU1 = ZL1, ZL1 + PLATE_T  # upper half plate: 0.0070 .. 0.0105

# Full-height blade region
BLADE_Z0, BLADE_Z1 = ZL0, ZU1
BLADE_X_MIN = 0.0085

BOSS_R = 0.0075
HOLE_R = 0.0028
SHANK_R = 0.0024
HEAD_R = 0.0040

EDGE_LAND = 0.0003

# Thick insulated grip sleeves (layered: inner dark rubber + outer orange)
GRIP_H = 0.015  # thicker insulation
GRIP_LZ0 = -0.002
GRIP_LZ1 = GRIP_LZ0 + GRIP_H  # lower grip
GRIP_UZ0 = GRIP_LZ0 + (ZU0 - ZL0)
GRIP_UZ1 = GRIP_UZ0 + GRIP_H
INNER_GRIP_INSET = 0.0012  # outer sleeve inset from inner sleeve edge
INNER_GRIP_H = GRIP_H  # inner sleeve same height
INLAY_T = 0.0007
INLAY_EMBED = 0.0002

# Rivet cap dimensions
CAP_R = 0.0055
CAP_T = 0.0012

# Lock lever dimensions
LEVER_LENGTH = 0.018
LEVER_WIDTH = 0.005
LEVER_THICK = 0.003
LEVER_PIVOT_X = -0.028
LEVER_PIVOT_Y = -0.009
LEVER_TRAVEL = math.radians(35.0)

# Jaw plate outline (lower half, jaw body at +y)
JAW_PTS = [
    (0.0000, 0.0090),
    (0.0060, 0.0094),
    (0.0140, 0.0086),
    (0.0240, 0.0060),
    (0.0330, 0.0028),
    (0.0352, 0.0012),
    (0.0346, EDGE_LAND),
    (0.0090, EDGE_LAND),
    (0.0020, 0.0048),
]

# Slim steel neck/tang
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

# Inner grip sleeve outline (the full-size dark rubber base layer)
INNER_GRIP_PTS = [
    (-0.0240, -0.0038),
    (-0.0400, -0.0052),
    (-0.0580, -0.0068),
    (-0.0740, -0.0082),
    (-0.0890, -0.0094),
    (-0.0960, -0.0110),
    (-0.0920, -0.0150),
    (-0.0780, -0.0172),
    (-0.0620, -0.0176),
    (-0.0460, -0.0160),
    (-0.0300, -0.0130),
    (-0.0235, -0.0108),
]

# Outer grip sleeve (bright orange, clearly inset from inner so dark rubber shows)
GRIP_PTS = [
    (-0.0260, -0.0050),
    (-0.0420, -0.0064),
    (-0.0590, -0.0078),
    (-0.0740, -0.0090),
    (-0.0870, -0.0100),
    (-0.0920, -0.0112),
    (-0.0890, -0.0142),
    (-0.0770, -0.0160),
    (-0.0630, -0.0164),
    (-0.0480, -0.0150),
    (-0.0330, -0.0124),
    (-0.0260, -0.0104),
]

# Translucent inlay strip
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


def _bevel_wedge(s: float) -> cq.Workplane:
    """Material removed above the flat angled blade face."""
    tri = [
        (s * EDGE_LAND, BLADE_Z0 + 0.0004),
        (s * 0.0052, 0.0112),
        (s * EDGE_LAND, 0.0112),
    ]
    return (
        cq.Workplane("YZ", origin=(BLADE_X_MIN, 0.0, 0.0))
        .polyline(tri)
        .close()
        .extrude(0.0330)
    )


def _blade_clip_box() -> cq.Workplane:
    return cq.Workplane("XY", origin=(BLADE_X_MIN + 0.0250, 0.0, 0.0070)).box(
        0.0500, 0.0600, 0.0200
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


def _rivet_cap_lower() -> cq.Workplane:
    """Circular pivot rivet cap disc on the bottom side, touching the lower boss/rivet."""
    # Position so top face overlaps the rivet lower head slightly for connectivity
    z_base = 0.0021 - CAP_T + 0.0002
    cap = cq.Workplane("XY", origin=(0.0, 0.0, z_base)).circle(CAP_R).extrude(CAP_T)
    try:
        cap = cap.edges("<Z").fillet(0.0004)
    except Exception:
        pass
    return cap


def _rivet_cap_upper() -> cq.Workplane:
    """Circular pivot rivet cap disc on the top side, touching the upper boss."""
    # Position so bottom face overlaps the upper boss top slightly for connectivity
    z_base = ZU1 - 0.0002
    cap = cq.Workplane("XY", origin=(0.0, 0.0, z_base)).circle(CAP_R).extrude(CAP_T)
    try:
        cap = cap.edges(">Z").fillet(0.0004)
    except Exception:
        pass
    return cap


def _lock_lever() -> cq.Workplane:
    """Small locking release lever tab that pivots inside the handle."""
    # Lever body: rounded rectangle tab
    lever = (
        cq.Workplane("XY", origin=(LEVER_PIVOT_X, LEVER_PIVOT_Y, ZL0 - 0.001))
        .rect(LEVER_LENGTH, LEVER_WIDTH)
        .extrude(LEVER_THICK)
    )
    # Pivot boss (small cylinder at the pivot end)
    pivot_boss = (
        cq.Workplane("XY", origin=(LEVER_PIVOT_X, LEVER_PIVOT_Y, ZL0 - 0.001))
        .circle(LEVER_WIDTH * 0.6)
        .extrude(LEVER_THICK)
    )
    lever = lever.union(pivot_boss)
    # Rounded tip at the far end
    try:
        lever = lever.edges(">Z").fillet(0.0008)
    except Exception:
        pass
    return lever


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="insulated_locking_pliers")

    steel = model.material("brushed_steel", rgba=(0.72, 0.73, 0.75, 1.0))
    polished = model.material("polished_steel", rgba=(0.86, 0.87, 0.90, 1.0))
    orange = model.material("orange_grip", rgba=(0.97, 0.52, 0.07, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.18, 0.12, 0.10, 1.0))
    peach = model.material("peach_inlay", rgba=(1.0, 0.80, 0.64, 0.78))
    lever_mat = model.material("lever_steel", rgba=(0.60, 0.61, 0.63, 1.0))
    cap_mat = model.material("cap_chrome", rgba=(0.82, 0.83, 0.86, 1.0))

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
    # Inner grip sleeve (dark rubber insulation layer)
    lower.visual(
        mesh_from_cadquery(
            _soft_prism(_mirror(INNER_GRIP_PTS, +1.0), GRIP_LZ0, GRIP_LZ1),
            "lower_inner_grip",
        ),
        origin=lower_pose,
        material=dark_rubber,
        name="grip_inner",
    )
    # Outer grip sleeve (bright orange insulation)
    lower.visual(
        mesh_from_cadquery(
            _soft_prism(_mirror(GRIP_PTS, +1.0), GRIP_LZ0 + 0.001, GRIP_LZ1 - 0.001),
            "lower_grip",
        ),
        origin=lower_pose,
        material=orange,
        name="grip",
    )
    lower.visual(
        mesh_from_cadquery(
            _spline_prism(
                _mirror(INLAY_PTS, +1.0), GRIP_LZ1 - INLAY_EMBED - 0.001, GRIP_LZ1 - INLAY_EMBED + INLAY_T - 0.001
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
        mesh_from_cadquery(_rivet_cap_lower(), "rivet_cap_lower"),
        origin=lower_pose,
        material=cap_mat,
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
    # Inner grip sleeve (dark rubber)
    upper.visual(
        mesh_from_cadquery(
            _soft_prism(_mirror(INNER_GRIP_PTS, -1.0), GRIP_UZ0, GRIP_UZ1),
            "upper_inner_grip",
        ),
        material=dark_rubber,
        name="grip_inner",
    )
    # Outer grip sleeve (bright orange)
    upper.visual(
        mesh_from_cadquery(
            _soft_prism(_mirror(GRIP_PTS, -1.0), GRIP_UZ0 + 0.001, GRIP_UZ1 - 0.001),
            "upper_grip",
        ),
        material=orange,
        name="grip",
    )
    upper.visual(
        mesh_from_cadquery(
            _spline_prism(
                _mirror(INLAY_PTS, -1.0), GRIP_UZ1 - INLAY_EMBED - 0.001, GRIP_UZ1 - INLAY_EMBED + INLAY_T - 0.001
            ),
            "upper_inlay",
        ),
        material=peach,
        name="grip_inlay",
    )
    # Upper rivet cap (circular disc on top side of pivot)
    upper.visual(
        mesh_from_cadquery(_rivet_cap_upper(), "rivet_cap_upper"),
        material=cap_mat,
        name="rivet_cap",
    )

    # ----- lock lever: small release lever pivoting inside the lower handle
    lever = model.part("lock_lever")
    lever.visual(
        mesh_from_cadquery(_lock_lever(), "lever_body"),
        origin=lower_pose,
        material=lever_mat,
        name="lever_body",
    )

    # ----- main pivot: revolute at the rivet, axis perpendicular to tool plane
    model.articulation(
        "rivet_pivot",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, -HALF_OPEN)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=4.0, lower=0.0, upper=CLOSE_TRAVEL),
    )

    # ----- lever pivot: revolute, lever swings inside the handle slot
    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=lever,
        origin=Origin(xyz=(LEVER_PIVOT_X, LEVER_PIVOT_Y, ZL0), rpy=(0.0, 0.0, HALF_OPEN)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=0.0, upper=LEVER_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower = object_model.get_part("lower_half")
    upper = object_model.get_part("upper_half")
    lever = object_model.get_part("lock_lever")
    pivot = object_model.get_articulation("rivet_pivot")
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

    # --- rivet caps present on both sides of the pivot
    lower_cap = ctx.part_element_world_aabb(lower, elem="rivet_cap")
    upper_cap = ctx.part_element_world_aabb(upper, elem="rivet_cap")
    ctx.check(
        "lower rivet cap exists below pivot",
        lower_cap is not None and lower_cap[0][2] < ZL0,
        details=f"lower_cap={lower_cap}",
    )
    ctx.check(
        "upper rivet cap exists above pivot",
        upper_cap is not None and upper_cap[1][2] > ZU1,
        details=f"upper_cap={upper_cap}",
    )
    # Caps should be circular (roughly equal XY extents)
    if lower_cap is not None:
        dx_l = lower_cap[1][0] - lower_cap[0][0]
        dy_l = lower_cap[1][1] - lower_cap[0][1]
        ctx.check(
            "lower rivet cap is roughly circular",
            abs(dx_l - dy_l) < 0.003 and dx_l > 0.008,
            details=f"dx={dx_l:.4f}, dy={dy_l:.4f}",
        )
    if upper_cap is not None:
        dx_u = upper_cap[1][0] - upper_cap[0][0]
        dy_u = upper_cap[1][1] - upper_cap[0][1]
        ctx.check(
            "upper rivet cap is roughly circular",
            abs(dx_u - dy_u) < 0.003 and dx_u > 0.008,
            details=f"dx={dx_u:.4f}, dy={dy_u:.4f}",
        )

    # --- thick layered grip sleeves: inner dark rubber visible around outer
    for part_obj, part_name in [(lower, "lower"), (upper, "upper")]:
        inner = ctx.part_element_world_aabb(part_obj, elem="grip_inner")
        outer = ctx.part_element_world_aabb(part_obj, elem="grip")
        ctx.check(
            f"{part_name} inner grip sleeve exists",
            inner is not None,
            details=f"inner={inner}",
        )
        ctx.check(
            f"{part_name} outer grip sleeve exists",
            outer is not None,
            details=f"outer={outer}",
        )
        if inner is not None and outer is not None:
            # Inner sleeve should extend beyond outer on at least one XY edge
            inner_extends = (
                inner[0][0] < outer[0][0] - 0.0005
                or inner[1][0] > outer[1][0] + 0.0005
                or inner[0][1] < outer[0][1] - 0.0005
                or inner[1][1] > outer[1][1] + 0.0005
            )
            ctx.check(
                f"{part_name} inner sleeve visible around outer sleeve",
                inner_extends,
                details=f"inner={inner}, outer={outer}",
            )

    # --- lock lever exists and is near the lower handle
    lever_aabb = ctx.part_world_aabb(lever)
    lower_aabb = ctx.part_world_aabb(lower)
    ctx.check(
        "lock lever exists with valid geometry",
        lever_aabb is not None,
        details=f"lever={lever_aabb}",
    )
    if lever_aabb is not None and lower_aabb is not None:
        # Lever should overlap the lower handle in XY (it sits inside/near it)
        ctx.expect_overlap(
            lever,
            lower,
            axes="xy",
            min_overlap=0.005,
            name="lock lever overlaps lower handle footprint",
        )

    # --- lever pivot joint is non-fixed with correct limits
    lever_limits = lever_joint.motion_limits
    ctx.check(
        "lever pivot has non-trivial travel",
        lever_limits is not None
        and lever_limits.lower is not None
        and lever_limits.upper is not None
        and lever_limits.upper > 0.1,
        details=f"limits={lever_limits}",
    )

    # --- lever actually moves when posed (check visual AABB center shift)
    lever_rest_aabb = ctx.part_element_world_aabb(lever, elem="lever_body")
    with ctx.pose({lever_joint: LEVER_TRAVEL}):
        lever_moved_aabb = ctx.part_element_world_aabb(lever, elem="lever_body")
    ctx.check(
        "lock lever moves when pivoted",
        lever_rest_aabb is not None
        and lever_moved_aabb is not None
        and (
            abs((lever_moved_aabb[0][0] + lever_moved_aabb[1][0]) / 2 - (lever_rest_aabb[0][0] + lever_rest_aabb[1][0]) / 2) > 0.0005
            or abs((lever_moved_aabb[0][1] + lever_moved_aabb[1][1]) / 2 - (lever_rest_aabb[0][1] + lever_rest_aabb[1][1]) / 2) > 0.0005
        ),
        details=f"rest={lever_rest_aabb}, moved={lever_moved_aabb}",
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
        min_gap=0.012,
        name="handles splay apart at rest",
    )

    # --- blades span the full forged head height
    for part_obj in (lower, upper):
        blade = ctx.part_element_world_aabb(part_obj, elem="jaw_blade")
        ctx.check(
            f"{part_obj.name} blade spans both plate layers",
            blade is not None and blade[0][2] <= ZL0 + 0.0002 and blade[1][2] >= ZU1 - 0.0002,
            details=f"blade={blade}",
        )

    # --- main pivot articulation: positive q closes blades
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
            0.048 <= width <= 0.082,
            details=f"width={width:.4f}",
        )
    else:
        ctx.fail("overall proportions", "missing part AABBs")

    return ctx.report()


object_model = build_object_model()
