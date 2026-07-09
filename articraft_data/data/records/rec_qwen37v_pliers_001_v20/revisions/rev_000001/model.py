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
# Insulated slip-joint pliers (electronics/lineman variant).
#
# Two forged-steel halves cross at a slip-joint pivot. The lower half carries
# a fixed pivot pin; the upper half has an elongated slot so the pin can slide
# along a short prismatic travel (slip-joint adjustment). A separate visible
# "slip carriage" part represents the pivot pin/washer that rides in the slot.
#
# Each half: tapered jaw head with visible bevel wedges -> slim steel neck ->
# thick layered insulated grip sleeve (dark rubber inner + bright red outer).
# Circular rivet caps on both sides of the pivot.
#
# Articulations:
#   1. slip_slide (PRISMATIC): lower_half -> slip_carriage, axis along X
#   2. pivot_hinge (REVOLUTE): slip_carriage -> upper_half, axis along Z
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
HEAD_R = 0.0045

# Slip-joint slot dimensions (elongated hole in upper boss).
SLOT_LENGTH = 0.010  # total slot travel length along X
SLOT_WIDTH = 0.005   # slot width (matches shank diameter with clearance)
SLIP_TRAVEL = 0.006  # usable prismatic travel

EDGE_LAND = 0.0003

# Thick layered grip sleeves.
GRIP_H_INNER = 0.008  # dark rubber inner core
GRIP_H_OUTER = 0.005  # red insulation outer sleeve
GRIP_LZ0 = 0.0
GRIP_LZ_INNER1 = GRIP_LZ0 + GRIP_H_INNER
GRIP_LZ_OUTER0 = GRIP_LZ0 - 0.001  # outer sleeve starts slightly below
GRIP_LZ_OUTER1 = GRIP_LZ_INNER1 + GRIP_H_OUTER

GRIP_UZ0 = GRIP_LZ0 + (ZU0 - ZL0)
GRIP_UZ_INNER1 = GRIP_UZ0 + GRIP_H_INNER
GRIP_UZ_OUTER0 = GRIP_UZ0 - 0.001
GRIP_UZ_OUTER1 = GRIP_UZ_INNER1 + GRIP_H_OUTER

# Bevel wedge dimensions (visible cutter bevel on jaw edges).
BEVEL_HEIGHT = 0.0025
BEVEL_DEPTH = 0.003
BEVEL_LENGTH = 0.025

# Rivet cap dimensions.
CAP_R = 0.0055
CAP_T = 0.0012

# Jaw plate outline (lower half, jaw body at +y).
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

# Slim steel neck/tang.
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

# Curved handle outline for inner grip core.
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

# Outer sleeve is a fatter version of the grip outline.
GRIP_OUTER_PTS = [
    (-0.0230, -0.0028),
    (-0.0390, -0.0036),
    (-0.0570, -0.0048),
    (-0.0730, -0.0060),
    (-0.0885, -0.0068),
    (-0.0970, -0.0082),
    (-0.0940, -0.0138),
    (-0.0800, -0.0162),
    (-0.0640, -0.0168),
    (-0.0470, -0.0152),
    (-0.0300, -0.0125),
    (-0.0225, -0.0105),
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
        0.0500, 0.0600, 0.0200
    )


def _half_blade(s: float, own_z0: float, own_z1: float) -> cq.Workplane:
    prof = _mirror(JAW_PTS, s)
    full = _poly_prism(prof, BLADE_Z0, BLADE_Z1).intersect(_blade_clip_box())
    rear = _poly_prism(prof, own_z0, own_z1)
    return full.union(rear)


def _bevel_wedge(s: float) -> cq.Workplane:
    """Visible cutter bevel wedge on the jaw cutting edge. A triangular prism
    showing the ground bevel face angling from the cutting edge land to the
    flat top of the jaw head."""
    tri = [
        (s * EDGE_LAND, BLADE_Z0 + 0.0004),
        (s * (EDGE_LAND + BEVEL_DEPTH), BLADE_Z0 + 0.0004 + BEVEL_HEIGHT),
        (s * EDGE_LAND, BLADE_Z0 + 0.0004 + BEVEL_HEIGHT),
    ]
    return (
        cq.Workplane("YZ", origin=(BLADE_X_MIN + 0.002, 0.0, 0.0))
        .polyline(tri)
        .close()
        .extrude(BEVEL_LENGTH)
    )


def _half_boss(own_z0: float, own_z1: float, with_slot: bool) -> cq.Workplane:
    boss = cq.Workplane("XY", origin=(0.0, 0.0, own_z0)).circle(BOSS_R).extrude(
        own_z1 - own_z0
    )
    if with_slot:
        # Elongated slip-joint slot cut through the boss.
        slot = (
            cq.Workplane("XY", origin=(0.0, 0.0, own_z0 - 0.001))
            .slot2D(SLOT_LENGTH, SLOT_WIDTH, angle=0)
            .extrude((own_z1 - own_z0) + 0.002)
        )
        boss = boss.cut(slot)
    else:
        # Round hole for the fixed pivot side.
        hole = cq.Workplane("XY", origin=(0.0, 0.0, own_z0 - 0.001)).circle(HOLE_R).extrude(
            (own_z1 - own_z0) + 0.002
        )
        boss = boss.cut(hole)
    return boss


def _rivet_cap(z_center: float) -> cq.Workplane:
    """Circular pivot rivet cap disc."""
    return (
        cq.Workplane("XY", origin=(0.0, 0.0, z_center - CAP_T / 2.0))
        .circle(CAP_R)
        .extrude(CAP_T)
    )


def _slip_carriage_body() -> cq.Workplane:
    """Pivot pin assembly that slides in the slot: shank + washer discs."""
    shank = (
        cq.Workplane("XY", origin=(0.0, 0.0, ZL0 - 0.001))
        .circle(SHANK_R)
        .extrude((ZU1 - ZL0) + 0.004)
    )
    # Lower washer disc.
    lower_washer = (
        cq.Workplane("XY", origin=(0.0, 0.0, ZL0 - 0.003))
        .circle(HEAD_R)
        .extrude(0.0025)
    )
    # Upper washer disc.
    upper_washer = (
        cq.Workplane("XY", origin=(0.0, 0.0, ZU1 + 0.0005))
        .circle(HEAD_R)
        .extrude(0.0025)
    )
    body = shank.union(lower_washer).union(upper_washer)
    try:
        body = body.edges(">Z").fillet(0.0008)
    except Exception:
        pass
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="insulated_slip_joint_pliers")

    steel = model.material("brushed_steel", rgba=(0.72, 0.73, 0.75, 1.0))
    polished = model.material("polished_steel", rgba=(0.86, 0.87, 0.90, 1.0))
    dark_rubber = model.material("dark_rubber_inner", rgba=(0.18, 0.18, 0.20, 1.0))
    red_insulation = model.material("red_insulation", rgba=(0.88, 0.12, 0.10, 1.0))
    bevel_steel = model.material("ground_bevel", rgba=(0.80, 0.81, 0.83, 1.0))
    cap_chrome = model.material("chrome_cap", rgba=(0.82, 0.84, 0.88, 1.0))

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
        mesh_from_cadquery(_bevel_wedge(+1.0), "lower_bevel"),
        origin=lower_pose,
        material=bevel_steel,
        name="cutter_bevel",
    )
    lower.visual(
        mesh_from_cadquery(_half_boss(ZL0, ZL1, with_slot=False), "lower_boss"),
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
    # Inner grip core (dark rubber).
    lower.visual(
        mesh_from_cadquery(
            _soft_prism(_mirror(GRIP_PTS, +1.0), GRIP_LZ0, GRIP_LZ_INNER1),
            "lower_grip_inner",
        ),
        origin=lower_pose,
        material=dark_rubber,
        name="grip_inner",
    )
    # Outer insulation sleeve (bright red, fatter profile).
    lower.visual(
        mesh_from_cadquery(
            _soft_prism(_mirror(GRIP_OUTER_PTS, +1.0), GRIP_LZ_OUTER0, GRIP_LZ_OUTER1),
            "lower_grip_outer",
        ),
        origin=lower_pose,
        material=red_insulation,
        name="grip_outer",
    )
    # Lower rivet cap (bottom side, embedded into boss face for connectivity).
    lower.visual(
        mesh_from_cadquery(_rivet_cap(ZL0 - CAP_T / 2.0 + 0.0005), "lower_cap"),
        origin=lower_pose,
        material=cap_chrome,
        name="rivet_cap_lower",
    )

    # ----- slip carriage: pivot pin/washer that slides in the upper slot
    carriage = model.part("slip_carriage")
    carriage.visual(
        mesh_from_cadquery(_slip_carriage_body(), "carriage_body"),
        material=polished,
        name="pivot_pin",
    )

    # ----- upper half (moving jaw/handle): mirrored, upper steel layer, slotted boss
    upper = model.part("upper_half")
    upper.visual(
        mesh_from_cadquery(_half_blade(-1.0, ZU0, ZU1), "upper_blade"),
        material=steel,
        name="jaw_blade",
    )
    upper.visual(
        mesh_from_cadquery(_bevel_wedge(-1.0), "upper_bevel"),
        material=bevel_steel,
        name="cutter_bevel",
    )
    upper.visual(
        mesh_from_cadquery(_half_boss(ZU0, ZU1, with_slot=True), "upper_boss"),
        material=steel,
        name="pivot_boss",
    )
    upper.visual(
        mesh_from_cadquery(_poly_prism(_mirror(TANG_PTS, -1.0), ZU0, ZU1), "upper_tang"),
        material=steel,
        name="neck_tang",
    )
    # Inner grip core (dark rubber).
    upper.visual(
        mesh_from_cadquery(
            _soft_prism(_mirror(GRIP_PTS, -1.0), GRIP_UZ0, GRIP_UZ_INNER1),
            "upper_grip_inner",
        ),
        material=dark_rubber,
        name="grip_inner",
    )
    # Outer insulation sleeve (bright red, fatter profile).
    upper.visual(
        mesh_from_cadquery(
            _soft_prism(_mirror(GRIP_OUTER_PTS, -1.0), GRIP_UZ_OUTER0, GRIP_UZ_OUTER1),
            "upper_grip_outer",
        ),
        material=red_insulation,
        name="grip_outer",
    )
    # Upper rivet cap (top side, embedded into boss face for connectivity).
    upper.visual(
        mesh_from_cadquery(_rivet_cap(ZU1 + CAP_T / 2.0 - 0.0005), "upper_cap"),
        material=cap_chrome,
        name="rivet_cap_upper",
    )

    # ----- Prismatic: lower_half -> slip_carriage (slides along slot axis X)
    model.articulation(
        "slip_slide",
        ArticulationType.PRISMATIC,
        parent=lower,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, -HALF_OPEN)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.1, lower=0.0, upper=SLIP_TRAVEL),
    )

    # ----- Revolute: slip_carriage -> upper_half (jaw open/close)
    model.articulation(
        "pivot_hinge",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=4.0, lower=0.0, upper=CLOSE_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower = object_model.get_part("lower_half")
    upper = object_model.get_part("upper_half")
    carriage = object_model.get_part("slip_carriage")
    slip = object_model.get_articulation("slip_slide")
    pivot = object_model.get_articulation("pivot_hinge")

    # --- Slip joint: prismatic articulation exists with valid travel
    slip_limits = slip.motion_limits
    ctx.check(
        "slip joint has prismatic travel",
        slip_limits is not None
        and slip_limits.lower is not None
        and slip_limits.upper is not None
        and slip_limits.upper > 0.003
        and slip_limits.upper < 0.015,
        details=f"slip_limits={slip_limits}",
    )

    # --- Slip carriage connects lower and upper at the pivot
    ctx.expect_contact(
        lower,
        carriage,
        contact_tol=0.002,
        name="slip carriage seated on lower half pivot",
    )
    ctx.expect_contact(
        carriage,
        upper,
        contact_tol=0.002,
        name="slip carriage engages upper half slot",
    )

    # --- Pivot stack: bosses coaxial, upper sits above lower
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
        min_gap=-0.0005,
        max_gap=0.002,
        name="upper boss sits above lower boss",
    )

    # --- Rivet caps on both sides
    lower_cap_aabb = ctx.part_element_world_aabb(lower, elem="rivet_cap_lower")
    upper_cap_aabb = ctx.part_element_world_aabb(upper, elem="rivet_cap_upper")
    lower_boss_aabb = ctx.part_element_world_aabb(lower, elem="pivot_boss")
    upper_boss_aabb = ctx.part_element_world_aabb(upper, elem="pivot_boss")

    ctx.check(
        "lower rivet cap exists below lower boss",
        lower_cap_aabb is not None
        and lower_boss_aabb is not None
        and lower_cap_aabb[1][2] <= lower_boss_aabb[0][2] + 0.001,
        details=f"cap={lower_cap_aabb}, boss={lower_boss_aabb}",
    )
    ctx.check(
        "upper rivet cap exists above upper boss",
        upper_cap_aabb is not None
        and upper_boss_aabb is not None
        and upper_cap_aabb[0][2] >= upper_boss_aabb[1][2] - 0.001,
        details=f"cap={upper_cap_aabb}, boss={upper_boss_aabb}",
    )

    # --- Cutter bevel wedges visible on both jaws
    for part_obj in (lower, upper):
        bevel_aabb = ctx.part_element_world_aabb(part_obj, elem="cutter_bevel")
        jaw_aabb = ctx.part_element_world_aabb(part_obj, elem="jaw_blade")
        ctx.check(
            f"{part_obj.name} cutter bevel exists on jaw",
            bevel_aabb is not None and jaw_aabb is not None,
            details=f"bevel={bevel_aabb}, jaw={jaw_aabb}",
        )
        if bevel_aabb is not None and jaw_aabb is not None:
            ctx.check(
                f"{part_obj.name} bevel within jaw X span",
                bevel_aabb[0][0] >= jaw_aabb[0][0] - 0.001
                and bevel_aabb[1][0] <= jaw_aabb[1][0] + 0.001,
                details=f"bevel_x={bevel_aabb[0][0]:.4f}..{bevel_aabb[1][0]:.4f}, jaw_x={jaw_aabb[0][0]:.4f}..{jaw_aabb[1][0]:.4f}",
            )

    # --- Thick layered grip: outer sleeve encloses inner core
    for part_obj in (lower, upper):
        inner_aabb = ctx.part_element_world_aabb(part_obj, elem="grip_inner")
        outer_aabb = ctx.part_element_world_aabb(part_obj, elem="grip_outer")
        ctx.check(
            f"{part_obj.name} outer sleeve exists",
            outer_aabb is not None and inner_aabb is not None,
            details=f"inner={inner_aabb}, outer={outer_aabb}",
        )
        if inner_aabb is not None and outer_aabb is not None:
            # Outer sleeve should be thicker (larger Z span) than inner core.
            inner_z = inner_aabb[1][2] - inner_aabb[0][2]
            outer_z = outer_aabb[1][2] - outer_aabb[0][2]
            ctx.check(
                f"{part_obj.name} outer sleeve taller than inner core",
                outer_z > inner_z + 0.001,
                details=f"inner_z={inner_z:.4f}, outer_z={outer_z:.4f}",
            )

    # --- Rest pose: jaws open, handles splayed apart
    ctx.expect_gap(
        lower,
        upper,
        axis="y",
        positive_elem="jaw_blade",
        negative_elem="jaw_blade",
        min_gap=0.002,
        name="blade edges are open at rest",
    )
    ctx.expect_gap(
        upper,
        lower,
        axis="y",
        positive_elem="grip_outer",
        negative_elem="grip_outer",
        min_gap=0.008,
        name="handles splay apart at rest",
    )

    # --- Pivot hinge: revolute joint with ~25 deg travel
    pivot_limits = pivot.motion_limits
    ctx.check(
        "pivot hinge travel ~25 degrees",
        pivot_limits is not None
        and pivot_limits.lower is not None
        and pivot_limits.upper is not None
        and abs(pivot_limits.lower) < 1e-9
        and 0.38 <= pivot_limits.upper <= 0.48,
        details=f"pivot_limits={pivot_limits}",
    )

    # --- Articulated closing: blades close, handles scissor
    open_blade = ctx.part_element_world_aabb(upper, elem="jaw_blade")
    open_grip = ctx.part_element_world_aabb(upper, elem="grip_outer")
    with ctx.pose({pivot: CLOSE_TRAVEL}):
        ctx.expect_contact(
            lower,
            upper,
            elem_a="jaw_blade",
            elem_b="jaw_blade",
            contact_tol=0.002,
            name="blade edges meet when fully closed",
        )
        closed_blade = ctx.part_element_world_aabb(upper, elem="jaw_blade")
        closed_grip = ctx.part_element_world_aabb(upper, elem="grip_outer")

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

    # --- Slip joint prismatic motion moves carriage along slot
    carriage_rest = ctx.part_world_position(carriage)
    with ctx.pose({slip: SLIP_TRAVEL}):
        carriage_slid = ctx.part_world_position(carriage)
    ctx.check(
        "slip joint moves carriage along slot axis",
        carriage_rest is not None
        and carriage_slid is not None
        and carriage_slid[0] > carriage_rest[0] + 0.002,
        details=f"rest={carriage_rest}, slid={carriage_slid}",
    )

    # --- Overall proportions: ~0.13 long, reasonable width/thickness
    la = ctx.part_world_aabb(lower)
    ua = ctx.part_world_aabb(upper)
    if la is not None and ua is not None:
        length = max(la[1][0], ua[1][0]) - min(la[0][0], ua[0][0])
        width = max(la[1][1], ua[1][1]) - min(la[0][1], ua[0][1])
        height = max(la[1][2], ua[1][2]) - min(la[0][2], ua[0][2])
        ctx.check(
            "overall length ~0.13 m",
            0.10 <= length <= 0.16,
            details=f"length={length:.4f}",
        )
        ctx.check(
            "splayed handle width reasonable",
            0.04 <= width <= 0.09,
            details=f"width={width:.4f}",
        )
        ctx.check(
            "tool thickness reasonable for insulated grips",
            0.015 <= height <= 0.035,
            details=f"height={height:.4f}",
        )

    # --- Allow intentional overlaps for seated caps and grip layers
    ctx.allow_overlap(
        lower,
        lower,
        elem_a="grip_outer",
        elem_b="grip_inner",
        reason="Outer insulation sleeve intentionally overlaps inner rubber core as a layered grip assembly.",
    )
    ctx.allow_overlap(
        upper,
        upper,
        elem_a="grip_outer",
        elem_b="grip_inner",
        reason="Outer insulation sleeve intentionally overlaps inner rubber core as a layered grip assembly.",
    )
    ctx.allow_overlap(
        lower,
        lower,
        elem_a="rivet_cap_lower",
        elem_b="pivot_boss",
        reason="Lower rivet cap is seated against the lower pivot boss face.",
    )
    ctx.allow_overlap(
        upper,
        upper,
        elem_a="rivet_cap_upper",
        elem_b="pivot_boss",
        reason="Upper rivet cap is seated against the upper pivot boss face.",
    )
    ctx.allow_overlap(
        lower,
        carriage,
        reason="Slip carriage pivot pin passes through the lower boss hole.",
    )
    ctx.allow_overlap(
        carriage,
        upper,
        reason="Slip carriage pivot pin passes through the upper boss slot.",
    )

    return ctx.report()


object_model = build_object_model()
