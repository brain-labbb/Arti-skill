from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    KnobGeometry,
    KnobGrip,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Compact diagonal cutting pliers variant.
#
# Structural changes from parent flush-cut snips:
# - Compact diagonal cutting jaws (shorter, angled cutting edge)
# - Short handles (~0.050 m vs parent ~0.070 m)
# - Circular pivot rivet cap on BOTH sides
# - Color-separated geometric grip sleeves (outer sleeve + inner core)
# - Adjustment screw (continuous joint) at rear of lower handle
#
# Two mirrored half-tools cross at a polished rivet. The lower half is the
# base link; the upper half rotates about the rivet axis (Z). An adjustment
# screw at the rear of the lower handle rotates freely (continuous).
# ---------------------------------------------------------------------------

HALF_OPEN = math.radians(12.5)
CLOSE_TRAVEL = 2.0 * HALF_OPEN  # ~25 degrees

PLATE_T = 0.0035

ZL0, ZL1 = 0.0035, 0.0035 + PLATE_T
ZU0, ZU1 = ZL1, ZL1 + PLATE_T

BLADE_Z0, BLADE_Z1 = ZL0, ZU1
BLADE_X_MIN = 0.0080

BOSS_R = 0.0070
HOLE_R = 0.0028
SHANK_R = 0.0024
HEAD_R = 0.0040
CAP_R = 0.0055
CAP_T = 0.0012

EDGE_LAND = 0.0003

# Grip dimensions (shorter handles)
GRIP_H = 0.0100
GRIP_LZ0 = 0.0
GRIP_LZ1 = GRIP_LZ0 + GRIP_H
GRIP_UZ0 = GRIP_LZ0 + (ZU0 - ZL0)
GRIP_UZ1 = GRIP_UZ0 + GRIP_H

# Sleeve shell slightly proud of the grip core
SLEEVE_INSET = 0.0006
SLEEVE_PROUD = 0.0004

# Compact diagonal jaw outline (lower half, jaw at +y).
# Shorter than parent, cutting edge runs diagonally.
JAW_PTS = [
    (0.0000, 0.0085),
    (0.0050, 0.0088),
    (0.0120, 0.0078),
    (0.0200, 0.0052),
    (0.0260, 0.0024),
    (0.0270, 0.0010),
    (0.0264, EDGE_LAND),
    (0.0090, EDGE_LAND),
    (0.0020, 0.0045),
]

# Slim neck/tang (shorter to match compact proportions)
TANG_PTS = [
    (0.0020, -0.0030),
    (-0.0080, -0.0042),
    (-0.0160, -0.0050),
    (-0.0240, -0.0058),
    (-0.0240, -0.0100),
    (-0.0160, -0.0088),
    (-0.0080, -0.0076),
    (0.0008, -0.0068),
]

# Short curved handle outline (compact: ~0.050 m from boss)
GRIP_PTS = [
    (-0.0195, -0.0038),
    (-0.0320, -0.0050),
    (-0.0460, -0.0062),
    (-0.0580, -0.0072),
    (-0.0660, -0.0084),
    (-0.0680, -0.0096),
    (-0.0640, -0.0120),
    (-0.0540, -0.0136),
    (-0.0420, -0.0140),
    (-0.0310, -0.0126),
    (-0.0220, -0.0104),
    (-0.0190, -0.0090),
]

# Outer sleeve outline (inset from grip, for color-separated shell)
SLEEVE_PTS = [
    (-0.0220, -0.0046),
    (-0.0340, -0.0058),
    (-0.0470, -0.0070),
    (-0.0580, -0.0080),
    (-0.0640, -0.0090),
    (-0.0620, -0.0114),
    (-0.0520, -0.0128),
    (-0.0410, -0.0132),
    (-0.0310, -0.0120),
    (-0.0235, -0.0100),
    (-0.0215, -0.0082),
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
    cap: float = 0.0018,
    inset: float = 0.0015,
) -> cq.Workplane:
    """Extruded spline outline with loft-capped softly beveled top/bottom."""
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
    """Angled diagonal cutting bevel cut from the top of the jaw."""
    tri = [
        (s * EDGE_LAND, BLADE_Z0 + 0.0004),
        (s * 0.0048, 0.0108),
        (s * EDGE_LAND, 0.0108),
    ]
    return (
        cq.Workplane("YZ", origin=(BLADE_X_MIN, 0.0, 0.0))
        .polyline(tri)
        .close()
        .extrude(0.0250)
    )


def _blade_clip_box() -> cq.Workplane:
    return cq.Workplane("XY", origin=(BLADE_X_MIN + 0.0200, 0.0, 0.0070)).box(
        0.0420, 0.0600, 0.0200
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


def _pivot_cap(z_base: float, side: str) -> cq.Workplane:
    """Circular rivet cap disc on one side of the pivot."""
    cap = cq.Workplane("XY", origin=(0.0, 0.0, z_base)).circle(CAP_R).extrude(CAP_T)
    try:
        cap = cap.edges(">Z").fillet(0.0004)
    except Exception:
        pass
    # Add a small center dimple for visual detail
    dimple = (
        cq.Workplane("XY", origin=(0.0, 0.0, z_base + CAP_T - 0.0002))
        .circle(0.0015)
        .extrude(0.0004)
    )
    cap = cap.cut(dimple)
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


def _screw_body() -> cq.Workplane:
    """Small adjustment screw: shaft embeds below z=0 into handle, head sits on top."""
    # Shaft embedded into handle (below z=0)
    shaft = cq.Workplane("XY", origin=(0.0, 0.0, -0.006)).circle(0.0018).extrude(0.008)
    # Head sits above z=0
    head = cq.Workplane("XY", origin=(0.0, 0.0, 0.002)).circle(0.0035).extrude(0.003)
    try:
        head = head.edges(">Z").fillet(0.0006)
    except Exception:
        pass
    # Knurl ring around head side
    knurl = cq.Workplane("XY", origin=(0.0, 0.0, 0.002)).circle(0.0040).extrude(0.001)
    knurl = knurl.cut(cq.Workplane("XY", origin=(0.0, 0.0, 0.0019)).circle(0.0030).extrude(0.0012))
    # Slot across the head top
    slot = (
        cq.Workplane("XY", origin=(0.0, 0.0, 0.004))
        .rect(0.006, 0.0010)
        .extrude(0.002)
    )
    screw = shaft.union(head).union(knurl).cut(slot)
    return screw


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_diagonal_pliers")

    steel = model.material("brushed_steel", rgba=(0.72, 0.73, 0.75, 1.0))
    polished = model.material("polished_steel", rgba=(0.86, 0.87, 0.90, 1.0))
    dark_grip = model.material("dark_grip_sleeve", rgba=(0.15, 0.15, 0.18, 1.0))
    red_core = model.material("red_grip_core", rgba=(0.85, 0.18, 0.10, 1.0))
    screw_metal = model.material("screw_steel", rgba=(0.60, 0.62, 0.65, 1.0))
    cap_chrome = model.material("chrome_cap", rgba=(0.90, 0.91, 0.93, 1.0))

    # ----- lower half (base link): jaw at +y, handle at -y
    lower = model.part("lower_half")
    lower_pose = Origin(rpy=(0.0, 0.0, HALF_OPEN))

    lower.visual(
        mesh_from_cadquery(_half_blade(+1.0, ZL0, ZL1), "lower_blade"),
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
        name="neck",
    )
    # Inner grip core (red)
    lower.visual(
        mesh_from_cadquery(_soft_prism(_mirror(GRIP_PTS, +1.0), GRIP_LZ0, GRIP_LZ1), "lower_grip_core"),
        origin=lower_pose,
        material=red_core,
        name="grip_core",
    )
    # Outer sleeve shell (dark, slightly smaller profile, sits on top of core)
    lower.visual(
        mesh_from_cadquery(
            _spline_prism(
                _mirror(SLEEVE_PTS, +1.0),
                GRIP_LZ0 + SLEEVE_INSET,
                GRIP_LZ1 + SLEEVE_PROUD,
            ),
            "lower_sleeve",
        ),
        origin=lower_pose,
        material=dark_grip,
        name="grip_sleeve",
    )
    # Lower pivot cap (bottom side)
    lower.visual(
        mesh_from_cadquery(_pivot_cap(ZL0 - CAP_T, "lower"), "lower_cap"),
        origin=lower_pose,
        material=cap_chrome,
        name="pivot_cap",
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

    upper.visual(
        mesh_from_cadquery(_half_blade(-1.0, ZU0, ZU1), "upper_blade"),
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
        name="neck",
    )
    # Inner grip core (red)
    upper.visual(
        mesh_from_cadquery(_soft_prism(_mirror(GRIP_PTS, -1.0), GRIP_UZ0, GRIP_UZ1), "upper_grip_core"),
        material=red_core,
        name="grip_core",
    )
    # Outer sleeve shell (dark)
    upper.visual(
        mesh_from_cadquery(
            _spline_prism(
                _mirror(SLEEVE_PTS, -1.0),
                GRIP_UZ0 + SLEEVE_INSET,
                GRIP_UZ1 + SLEEVE_PROUD,
            ),
            "upper_sleeve",
        ),
        material=dark_grip,
        name="grip_sleeve",
    )
    # Upper pivot cap (top side)
    upper.visual(
        mesh_from_cadquery(_pivot_cap(ZU1, "upper"), "upper_cap"),
        material=cap_chrome,
        name="pivot_cap",
    )

    # ----- adjustment screw at rear of lower handle
    screw = model.part("adjustment_screw")
    # Screw visual at part-frame origin; shaft embeds below z=0 into handle,
    # head sits above z=0 on the handle top surface.
    screw.visual(
        mesh_from_cadquery(_screw_body(), "screw_body"),
        material=screw_metal,
        name="screw_body",
    )

    # ----- main pivot articulation
    model.articulation(
        "rivet_pivot",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, -HALF_OPEN)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=4.0, lower=0.0, upper=CLOSE_TRAVEL),
    )

    # ----- adjustment screw continuous rotation at the rear handle top
    model.articulation(
        "screw_rotation",
        ArticulationType.CONTINUOUS,
        parent=lower,
        child=screw,
        origin=Origin(xyz=(-0.049, -0.021, GRIP_LZ1)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower = object_model.get_part("lower_half")
    upper = object_model.get_part("upper_half")
    screw = object_model.get_part("adjustment_screw")
    pivot = object_model.get_articulation("rivet_pivot")
    screw_joint = object_model.get_articulation("screw_rotation")

    # --- pivot bosses coaxial, upper sits above lower
    ctx.expect_overlap(
        lower, upper, axes="xy",
        elem_a="pivot_boss", elem_b="pivot_boss",
        min_overlap=0.010,
        name="pivot bosses stack coaxially",
    )
    ctx.expect_gap(
        upper, lower, axis="z",
        positive_elem="pivot_boss", negative_elem="pivot_boss",
        min_gap=-0.00002, max_gap=0.0006,
        name="upper boss sits above lower boss",
    )
    ctx.expect_contact(
        lower, upper,
        elem_a="pivot_boss", elem_b="pivot_boss",
        contact_tol=0.0001,
        name="boss faces contact",
    )

    # --- pivot caps present on both sides
    lower_cap_aabb = ctx.part_element_world_aabb(lower, elem="pivot_cap")
    upper_cap_aabb = ctx.part_element_world_aabb(upper, elem="pivot_cap")
    ctx.check(
        "lower pivot cap exists below pivot",
        lower_cap_aabb is not None and lower_cap_aabb[0][2] < ZL0,
        details=f"lower_cap={lower_cap_aabb}",
    )
    ctx.check(
        "upper pivot cap exists above pivot",
        upper_cap_aabb is not None and upper_cap_aabb[1][2] > ZU1,
        details=f"upper_cap={upper_cap_aabb}",
    )

    # --- color-separated grip sleeves exist on both halves
    for part_obj in (lower, upper):
        sleeve = ctx.part_element_world_aabb(part_obj, elem="grip_sleeve")
        core = ctx.part_element_world_aabb(part_obj, elem="grip_core")
        ctx.check(
            f"{part_obj.name} grip sleeve overlaps core in XY",
            sleeve is not None and core is not None
            and sleeve[0][0] < core[1][0] and sleeve[1][0] > core[0][0]
            and sleeve[0][1] < core[1][1] and sleeve[1][1] > core[0][1],
            details=f"sleeve={sleeve}, core={core}",
        )

    # --- adjustment screw exists and is a separate part
    screw_aabb = ctx.part_world_aabb(screw)
    ctx.check(
        "adjustment screw has geometry",
        screw_aabb is not None,
        details=f"screw_aabb={screw_aabb}",
    )

    # --- screw joint is continuous (non-fixed)
    ctx.check(
        "screw joint is continuous",
        screw_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={screw_joint.articulation_type}",
    )

    # --- rest pose: jaws open, handles splayed
    ctx.expect_gap(
        lower, upper, axis="y",
        positive_elem="jaw", negative_elem="jaw",
        min_gap=0.002,
        name="blade edges open at rest",
    )
    ctx.expect_gap(
        upper, lower, axis="y",
        positive_elem="grip_core", negative_elem="grip_core",
        min_gap=0.008,
        name="handles splay apart at rest",
    )

    # --- compact overall proportions (~0.10 m long)
    la = ctx.part_world_aabb(lower)
    ua = ctx.part_world_aabb(upper)
    if la is not None and ua is not None:
        length = max(la[1][0], ua[1][0]) - min(la[0][0], ua[0][0])
        width = max(la[1][1], ua[1][1]) - min(la[0][1], ua[0][1])
        ctx.check(
            "compact length ~0.09-0.12 m",
            0.080 <= length <= 0.125,
            details=f"length={length:.4f}",
        )
        ctx.check(
            "splayed width ~0.04-0.07 m",
            0.035 <= width <= 0.075,
            details=f"width={width:.4f}",
        )

    # --- pivot joint: positive q closes blades
    limits = pivot.motion_limits
    ctx.check(
        "pivot travel ~0..25 degrees",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower) < 1e-9
        and 0.38 <= limits.upper <= 0.48,
        details=f"limits={limits}",
    )

    open_blade = ctx.part_element_world_aabb(upper, elem="jaw")
    open_grip = ctx.part_element_world_aabb(upper, elem="grip_core")
    with ctx.pose({pivot: CLOSE_TRAVEL}):
        ctx.expect_contact(
            lower, upper,
            elem_a="jaw", elem_b="jaw",
            contact_tol=0.002,
            name="blade edges meet when closed",
        )
        closed_blade = ctx.part_element_world_aabb(upper, elem="jaw")
        closed_grip = ctx.part_element_world_aabb(upper, elem="grip_core")

    ctx.check(
        "closing swings upper jaw toward lower jaw",
        open_blade is not None and closed_blade is not None
        and closed_blade[1][1] > open_blade[1][1] + 0.003,
        details=f"open={open_blade}, closed={closed_blade}",
    )
    ctx.check(
        "handles scissor opposite to jaws",
        open_grip is not None and closed_grip is not None
        and closed_grip[0][1] < open_grip[0][1] - 0.003,
        details=f"open={open_grip}, closed={closed_grip}",
    )

    # --- screw sits on the handle top surface (Z overlap = intentional embed)
    ctx.expect_gap(
        screw, lower, axis="z",
        positive_elem="screw_body", negative_elem="grip_core",
        min_gap=-0.008, max_gap=0.006,
        name="screw head sits near handle top",
    )
    # Screw XY within handle footprint
    ctx.expect_within(
        screw, lower, axes="xy",
        inner_elem="screw_body", outer_elem="grip_core",
        margin=0.002,
        name="screw centered on handle",
    )

    # --- screw rotates (continuous joint, pose check)
    screw_aabb_rest = ctx.part_world_aabb(screw)
    with ctx.pose({screw_joint: math.pi / 2}):
        screw_aabb_rot = ctx.part_world_aabb(screw)
    ctx.check(
        "screw joint is poseable",
        screw_aabb_rest is not None and screw_aabb_rot is not None,
        details=f"rest={screw_aabb_rest}, rotated={screw_aabb_rot}",
    )

    return ctx.report()


object_model = build_object_model()
