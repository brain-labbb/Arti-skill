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
# Bent-nose pliers variant.
#
# Two forged-steel half-tools cross at a central pivot rivet. Each half is a
# rigid link: angled bent-nose jaw -> slim steel neck -> curved handle with
# color-separated geometric grip sleeve. The lower half is the base link;
# the upper half rotates about the rivet axis (Z, perpendicular to the flat
# tool plane).
#
# Additional features vs. parent flush-cut snips:
#   - Bent-nose jaws angle downward at ~45 degrees near the tip
#   - Circular pivot rivet caps on both sides of the pivot stack
#   - Color-separated geometric grip sleeves (dark inner + colored outer)
#   - Locking release lever pivoting inside the lower handle
#
# Geometry is authored per half in a "closed-design" local frame. The rest
# pose (q=0) splays each half by HALF_OPEN; positive q closes the jaws while
# handles scissor together in opposition.
# ---------------------------------------------------------------------------

HALF_OPEN = math.radians(12.5)
CLOSE_TRAVEL = 2.0 * HALF_OPEN  # ~25 degrees

PLATE_T = 0.0035

ZL0, ZL1 = 0.0035, 0.0035 + PLATE_T
ZU0, ZU1 = ZL1, ZL1 + PLATE_T

BLADE_Z0, BLADE_Z1 = ZL0, ZU1
BLADE_X_MIN = 0.0085

BOSS_R = 0.0075
HOLE_R = 0.0028
SHANK_R = 0.0024
HEAD_R = 0.0042

# Pivot cap dimensions (circular disc on each side of the pivot stack)
CAP_R = 0.0058
CAP_T = 0.0012

EDGE_LAND = 0.0003

# Handle grip
GRIP_H = 0.011
GRIP_LZ0 = 0.0
GRIP_LZ1 = GRIP_LZ0 + GRIP_H
GRIP_UZ0 = GRIP_LZ0 + (ZU0 - ZL0)
GRIP_UZ1 = GRIP_UZ0 + GRIP_H

# Inner sleeve (dark rubber) slightly taller than outer sleeve
INNER_SLEEVE_EXTRA = 0.001

# Release lever dimensions
LEVER_LEN = 0.022
LEVER_W = 0.005
LEVER_T = 0.002
LEVER_PIVOT_X = -0.035
LEVER_PIVOT_Y = -0.009

# Bent-nose jaw: straight section then angled tip at ~45 degrees
BEND_ANGLE = math.radians(45.0)
BEND_X = 0.026  # where the bend starts along the jaw

# Jaw plate outline (closed-design frame, lower half at +y). The jaw has a
# straight base then an angled bent-nose tip section.
JAW_PTS = [
    (0.0000, 0.0090),
    (0.0060, 0.0094),
    (0.0140, 0.0088),
    (0.0220, 0.0078),
    (BEND_X, 0.0068),
    # Bent-nose section: angled downward
    (BEND_X + 0.008 * math.cos(BEND_ANGLE), 0.0068 - 0.008 * math.sin(BEND_ANGLE)),
    (BEND_X + 0.013 * math.cos(BEND_ANGLE), 0.0068 - 0.013 * math.sin(BEND_ANGLE)),
    (BEND_X + 0.014 * math.cos(BEND_ANGLE), 0.0068 - 0.014 * math.sin(BEND_ANGLE) + 0.0008),
    # Tip returns back
    (BEND_X + 0.013 * math.cos(BEND_ANGLE) + 0.001, 0.0068 - 0.013 * math.sin(BEND_ANGLE) + 0.0018),
    (BEND_X + 0.008 * math.cos(BEND_ANGLE) + 0.002, 0.0068 - 0.008 * math.sin(BEND_ANGLE) + 0.003),
    (BEND_X, 0.0048),
    (0.0090, EDGE_LAND + 0.003),
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

# Curved handle outline (periodic spline)
GRIP_PTS = [
    (-0.0240, -0.0038),
    (-0.0400, -0.0048),
    (-0.0580, -0.0060),
    (-0.0740, -0.0072),
    (-0.0890, -0.0080),
    (-0.0955, -0.0093),
    (-0.0915, -0.0128),
    (-0.0780, -0.0148),
    (-0.0620, -0.0152),
    (-0.0460, -0.0138),
    (-0.0300, -0.0112),
    (-0.0235, -0.0095),
]

# Outer grip sleeve (shorter, with geometric contour - wider midsection)
GRIP_OUTER_PTS = [
    (-0.0300, -0.0050),
    (-0.0460, -0.0058),
    (-0.0620, -0.0070),
    (-0.0760, -0.0080),
    (-0.0840, -0.0090),
    (-0.0840, -0.0120),
    (-0.0740, -0.0138),
    (-0.0600, -0.0144),
    (-0.0460, -0.0132),
    (-0.0330, -0.0110),
    (-0.0290, -0.0088),
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


def _blade_clip_box() -> cq.Workplane:
    return cq.Workplane("XY", origin=(BLADE_X_MIN + 0.0250, 0.0, 0.0070)).box(
        0.0600, 0.0600, 0.0200
    )


def _half_jaw(s: float, own_z0: float, own_z1: float) -> cq.Workplane:
    """Bent-nose jaw: full-height head forward of boss, own-plate rear."""
    prof = _mirror(JAW_PTS, s)
    full = _poly_prism(prof, BLADE_Z0, BLADE_Z1).intersect(_blade_clip_box())
    rear = _poly_prism(prof, own_z0, own_z1)
    return full.union(rear)


def _half_boss(own_z0: float, own_z1: float, with_hole: bool) -> cq.Workplane:
    boss = cq.Workplane("XY", origin=(0.0, 0.0, own_z0)).circle(BOSS_R).extrude(own_z1 - own_z0)
    if with_hole:
        hole = cq.Workplane("XY", origin=(0.0, 0.0, own_z0 - 0.001)).circle(HOLE_R).extrude(
            (own_z1 - own_z0) + 0.002
        )
        boss = boss.cut(hole)
    return boss


def _pivot_cap(z_base: float) -> cq.Workplane:
    """Circular rivet cap disc."""
    cap = cq.Workplane("XY", origin=(0.0, 0.0, z_base)).circle(CAP_R).extrude(CAP_T)
    try:
        cap = cap.edges(">Z").fillet(0.0005)
    except Exception:
        pass
    return cap


def _rivet_shank() -> cq.Workplane:
    """Central rivet shank passing through both halves."""
    shank = cq.Workplane("XY", origin=(0.0, 0.0, ZL0 - 0.001)).circle(SHANK_R).extrude(
        (ZU1 - ZL0) + 0.002
    )
    return shank


def _release_lever() -> cq.Workplane:
    """Locking release lever: flat metal arm with rounded ends."""
    lever_z0 = ZL0 + 0.0005
    lever_z1 = ZL0 + PLATE_T - 0.0005
    # Create lever as a rounded rectangle pivoting at LEVER_PIVOT_X, LEVER_PIVOT_Y
    pts = [
        (LEVER_PIVOT_X - LEVER_W * 0.4, LEVER_PIVOT_Y - LEVER_W * 0.5),
        (LEVER_PIVOT_X + LEVER_LEN * 0.7, LEVER_PIVOT_Y - LEVER_W * 0.35),
        (LEVER_PIVOT_X + LEVER_LEN, LEVER_PIVOT_Y - LEVER_W * 0.2),
        (LEVER_PIVOT_X + LEVER_LEN + 0.001, LEVER_PIVOT_Y),
        (LEVER_PIVOT_X + LEVER_LEN, LEVER_PIVOT_Y + LEVER_W * 0.2),
        (LEVER_PIVOT_X + LEVER_LEN * 0.7, LEVER_PIVOT_Y + LEVER_W * 0.35),
        (LEVER_PIVOT_X - LEVER_W * 0.4, LEVER_PIVOT_Y + LEVER_W * 0.5),
        (LEVER_PIVOT_X - LEVER_W * 0.6, LEVER_PIVOT_Y),
    ]
    lever = _poly_prism(pts, lever_z0, lever_z1)
    # Add a pivot boss (small cylinder) at the pivot point
    pivot_cyl = (
        cq.Workplane("XY", origin=(LEVER_PIVOT_X, LEVER_PIVOT_Y, lever_z0))
        .circle(0.0018)
        .extrude(lever_z1 - lever_z0)
    )
    return lever.union(pivot_cyl)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bent_nose_pliers")

    steel = model.material("brushed_steel", rgba=(0.72, 0.73, 0.75, 1.0))
    polished = model.material("polished_steel", rgba=(0.86, 0.87, 0.90, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.12, 0.12, 0.14, 1.0))
    blue_grip = model.material("blue_grip", rgba=(0.15, 0.40, 0.82, 1.0))
    lever_steel = model.material("lever_steel", rgba=(0.60, 0.62, 0.64, 1.0))

    lower_pose = Origin(rpy=(0.0, 0.0, HALF_OPEN))

    # ----- lower half (base link) -----
    lower = model.part("lower_half")
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
    # Inner grip sleeve (dark rubber, full handle shape)
    lower.visual(
        mesh_from_cadquery(
            _soft_prism(_mirror(GRIP_PTS, +1.0), GRIP_LZ0, GRIP_LZ1), "lower_inner_sleeve"
        ),
        origin=lower_pose,
        material=dark_rubber,
        name="inner_sleeve",
    )
    # Outer grip sleeve (blue, shorter with geometric contour)
    outer_z0 = GRIP_LZ0 + 0.0008
    outer_z1 = GRIP_LZ1 - 0.0008
    lower.visual(
        mesh_from_cadquery(
            _soft_prism(_mirror(GRIP_OUTER_PTS, +1.0), outer_z0, outer_z1, inset=0.0012),
            "lower_outer_sleeve",
        ),
        origin=lower_pose,
        material=blue_grip,
        name="outer_sleeve",
    )
    # Rivet shank
    lower.visual(
        mesh_from_cadquery(_rivet_shank(), "rivet_shank"),
        origin=lower_pose,
        material=polished,
        name="rivet",
    )
    # Lower pivot cap (circular disc below the lower boss, touching)
    lower.visual(
        mesh_from_cadquery(_pivot_cap(ZL0 - CAP_T), "lower_cap"),
        origin=lower_pose,
        material=polished,
        name="pivot_cap_lower",
    )

    # ----- upper half (moving link) -----
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
    # Inner grip sleeve (dark rubber)
    upper.visual(
        mesh_from_cadquery(
            _soft_prism(_mirror(GRIP_PTS, -1.0), GRIP_UZ0, GRIP_UZ1), "upper_inner_sleeve"
        ),
        material=dark_rubber,
        name="inner_sleeve",
    )
    # Outer grip sleeve (blue, shorter with geometric contour)
    outer_uz0 = GRIP_UZ0 + 0.0008
    outer_uz1 = GRIP_UZ1 - 0.0008
    upper.visual(
        mesh_from_cadquery(
            _soft_prism(_mirror(GRIP_OUTER_PTS, -1.0), outer_uz0, outer_uz1, inset=0.0012),
            "upper_outer_sleeve",
        ),
        material=blue_grip,
        name="outer_sleeve",
    )
    # Upper pivot cap (circular disc above the upper boss, touching)
    upper.visual(
        mesh_from_cadquery(_pivot_cap(ZU1), "upper_cap"),
        material=polished,
        name="pivot_cap_upper",
    )

    # ----- Release lever (pivots inside lower handle) -----
    lever = model.part("release_lever")
    lever.visual(
        mesh_from_cadquery(_release_lever(), "lever_body"),
        origin=lower_pose,
        material=lever_steel,
        name="lever_body",
    )

    # ----- Articulation: main pivot at the rivet -----
    model.articulation(
        "rivet_pivot",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, -HALF_OPEN)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=4.0, lower=0.0, upper=CLOSE_TRAVEL),
    )

    # ----- Articulation: release lever pivots inside lower handle -----
    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=lever,
        origin=Origin(xyz=(LEVER_PIVOT_X, LEVER_PIVOT_Y, ZL0 + PLATE_T * 0.5), rpy=(0.0, 0.0, HALF_OPEN)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=0.0, upper=math.radians(15.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower = object_model.get_part("lower_half")
    upper = object_model.get_part("upper_half")
    lever = object_model.get_part("release_lever")
    pivot = object_model.get_articulation("rivet_pivot")
    lever_joint = object_model.get_articulation("lever_pivot")

    # --- pivot stack: bosses coaxial, upper sits above lower ---
    ctx.expect_overlap(
        lower, upper,
        axes="xy",
        elem_a="pivot_boss", elem_b="pivot_boss",
        min_overlap=0.012,
        name="pivot bosses stack coaxially",
    )
    ctx.expect_gap(
        upper, lower,
        axis="z",
        positive_elem="pivot_boss", negative_elem="pivot_boss",
        min_gap=-0.00002, max_gap=0.0006,
        name="upper boss sits slightly above lower boss",
    )

    # --- pivot caps on both sides ---
    lower_cap_aabb = ctx.part_element_world_aabb(lower, elem="pivot_cap_lower")
    upper_cap_aabb = ctx.part_element_world_aabb(upper, elem="pivot_cap_upper")
    lboss_aabb = ctx.part_element_world_aabb(lower, elem="pivot_boss")
    uboss_aabb = ctx.part_element_world_aabb(upper, elem="pivot_boss")
    ctx.check(
        "lower cap sits below lower boss",
        lower_cap_aabb is not None and lboss_aabb is not None
        and lower_cap_aabb[0][2] < lboss_aabb[0][2] - 0.0005,
        details=f"lower_cap={lower_cap_aabb}, lower_boss={lboss_aabb}",
    )
    ctx.check(
        "upper cap sits above upper boss",
        upper_cap_aabb is not None and uboss_aabb is not None
        and upper_cap_aabb[1][2] > uboss_aabb[1][2] + 0.0005,
        details=f"upper_cap={upper_cap_aabb}, upper_boss={uboss_aabb}",
    )
    # Caps are circular - check XY overlap with bosses
    ctx.expect_overlap(
        lower, lower,
        axes="xy",
        elem_a="pivot_cap_lower", elem_b="pivot_boss",
        min_overlap=0.008,
        name="lower cap overlaps pivot boss in XY",
    )
    ctx.expect_overlap(
        upper, upper,
        axes="xy",
        elem_a="pivot_cap_upper", elem_b="pivot_boss",
        min_overlap=0.008,
        name="upper cap overlaps pivot boss in XY",
    )

    # --- bent-nose jaws: jaw has substantial forward extent and lateral width ---
    for p in (lower, upper):
        jaw_aabb = ctx.part_element_world_aabb(p, elem="jaw")
        ctx.check(
            f"{p.name} jaw has bent-nose extent",
            jaw_aabb is not None
            and (jaw_aabb[1][0] - jaw_aabb[0][0]) > 0.025
            and (jaw_aabb[1][1] - jaw_aabb[0][1]) > 0.006,
            details=f"jaw_aabb={jaw_aabb}",
        )

    # --- grip sleeves: outer is within inner footprint, color separation ---
    for p in (lower, upper):
        inner = ctx.part_element_world_aabb(p, elem="inner_sleeve")
        outer = ctx.part_element_world_aabb(p, elem="outer_sleeve")
        ctx.check(
            f"{p.name} outer sleeve within inner sleeve footprint",
            inner is not None and outer is not None
            and outer[0][0] >= inner[0][0] - 0.001
            and outer[1][0] <= inner[1][0] + 0.001
            and outer[0][1] >= inner[0][1] - 0.001
            and outer[1][1] <= inner[1][1] + 0.001,
            details=f"inner={inner}, outer={outer}",
        )
        # Outer sleeve is shorter in X (geometric separation)
        ctx.check(
            f"{p.name} outer sleeve is shorter than inner sleeve",
            inner is not None and outer is not None
            and (inner[1][0] - inner[0][0]) > (outer[1][0] - outer[0][0]) + 0.005,
            details=f"inner_len={inner[1][0] - inner[0][0]:.4f}, outer_len={outer[1][0] - outer[0][0]:.4f}",
        )

    # --- release lever exists within lower handle region ---
    lever_aabb = ctx.part_element_world_aabb(lever, elem="lever_body")
    lower_inner = ctx.part_element_world_aabb(lower, elem="inner_sleeve")
    ctx.check(
        "release lever within lower handle region",
        lever_aabb is not None and lower_inner is not None
        and lever_aabb[0][0] >= lower_inner[0][0] - 0.005
        and lever_aabb[1][0] <= lower_inner[1][0] + 0.005,
        details=f"lever={lever_aabb}, handle={lower_inner}",
    )

    # --- lever joint: non-fixed, has motion limits ---
    lever_limits = lever_joint.motion_limits
    ctx.check(
        "lever pivot has non-trivial motion range",
        lever_limits is not None
        and lever_limits.lower is not None
        and lever_limits.upper is not None
        and lever_limits.upper > lever_limits.lower + 0.05,
        details=f"limits={lever_limits}",
    )

    # --- rest pose: jaws open, handles splayed ---
    ctx.expect_gap(
        lower, upper,
        axis="y",
        positive_elem="jaw", negative_elem="jaw",
        min_gap=0.002,
        name="jaw tips are open at rest",
    )
    ctx.expect_gap(
        upper, lower,
        axis="y",
        positive_elem="outer_sleeve", negative_elem="outer_sleeve",
        min_gap=0.010,
        name="handles splay apart at rest",
    )

    # --- main pivot articulation: positive q closes jaws ---
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
    open_grip = ctx.part_element_world_aabb(upper, elem="outer_sleeve")
    with ctx.pose({pivot: CLOSE_TRAVEL}):
        closed_jaw = ctx.part_element_world_aabb(upper, elem="jaw")
        closed_grip = ctx.part_element_world_aabb(upper, elem="outer_sleeve")

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

    # --- lever articulation: positive q moves the lever arm ---
    lever_rest = ctx.part_element_world_aabb(lever, elem="lever_body")
    with ctx.pose({lever_joint: math.radians(12.0)}):
        lever_actuated = ctx.part_element_world_aabb(lever, elem="lever_body")
    ctx.check(
        "lever rotates when actuated",
        lever_rest is not None and lever_actuated is not None
        and (
            abs(lever_actuated[1][1] - lever_rest[1][1]) > 0.001
            or abs(lever_actuated[1][0] - lever_rest[1][0]) > 0.001
        ),
        details=f"rest={lever_rest}, actuated={lever_actuated}",
    )

    # --- overall proportions ---
    la = ctx.part_world_aabb(lower)
    ua = ctx.part_world_aabb(upper)
    if la is not None and ua is not None:
        length = max(la[1][0], ua[1][0]) - min(la[0][0], ua[0][0])
        width = max(la[1][1], ua[1][1]) - min(la[0][1], ua[0][1])
        ctx.check(
            "overall length ~0.13 m",
            0.11 <= length <= 0.15,
            details=f"length={length:.4f}",
        )
        ctx.check(
            "splayed handle width ~0.06 m",
            0.045 <= width <= 0.080,
            details=f"width={width:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
