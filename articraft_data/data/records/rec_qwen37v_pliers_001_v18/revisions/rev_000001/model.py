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
# End-cutting nippers (carpenter's pincers / end nippers).
#
# Two L-shaped half-tools cross at a central pivot rivet. Each half has a
# short rounded jaw extending perpendicular to its long curved handle. Both
# jaws face +Y (perpendicular to handles along -X). The lower half is the
# base link; the upper half rotates about the rivet axis (Z, perpendicular
# to the flat plane of the tool).
#
# Geometry is authored per half in a "closed-design" local frame. The rest
# pose (q=0) splays each half by HALF_OPEN via visual/joint-frame yaw so
# q in [0, 2*HALF_OPEN] closes the jaw edges together while the handles
# scissor together in opposition.
# ---------------------------------------------------------------------------

HALF_OPEN = math.radians(12.5)
CLOSE_TRAVEL = 2.0 * HALF_OPEN  # ~25 deg of closing travel

PLATE_T = 0.005  # forged steel plate thickness per half

# Steel layer stack (lower sits below upper at the pivot boss).
ZL0, ZL1 = 0.002, 0.002 + PLATE_T  # lower half plate: 0.002 .. 0.007
ZU0, ZU1 = ZL1, ZL1 + PLATE_T      # upper half plate: 0.007 .. 0.012

BOSS_R = 0.008
HOLE_R = 0.003

# Circular rivet caps on both sides
CAP_R = 0.0065
CAP_T = 0.0018

# Handle grip overmold (z extents per half)
GRIP_H = 0.010
GRIP_LZ0 = 0.000
GRIP_LZ1 = GRIP_LZ0 + GRIP_H  # lower grip 0.000 .. 0.010
GRIP_UZ0 = GRIP_LZ0 + (ZU0 - ZL0)  # upper grip 0.005 .. 0.015
GRIP_UZ1 = GRIP_UZ0 + GRIP_H

INLAY_T = 0.0010
INLAY_EMBED = 0.0004

# ---- Jaw profile: extends in +Y from near the pivot (perpendicular to handle).
# Rounded, broad base tapering to a cutting edge at the tip.
JAW_PTS = [
    (-0.003, 0.006),   # base left (overlaps boss)
    (-0.005, 0.010),
    (-0.006, 0.015),
    (-0.006, 0.020),
    (-0.005, 0.025),
    (-0.003, 0.029),
    (-0.001, 0.032),
    (0.000, 0.034),    # tip (cutting edge, facing +Y)
    (0.001, 0.032),
    (0.003, 0.029),
    (0.005, 0.025),
    (0.006, 0.020),
    (0.006, 0.015),
    (0.005, 0.010),
    (0.003, 0.006),    # base right (overlaps boss)
]

# ---- Lower tang: connects boss to handle at -Y side.
# Extended to x=-0.035 so it clearly overlaps with the grip start.
TANG_LOWER_PTS = [
    (0.005, -0.003),
    (-0.005, -0.004),
    (-0.015, -0.005),
    (-0.025, -0.005),
    (-0.032, -0.006),    # reaching into grip inner edge
    (-0.035, -0.009),    # overlapping grip body
    (-0.032, -0.013),    # overlapping grip outer
    (-0.025, -0.015),
    (-0.018, -0.012),
    (-0.010, -0.009),
    (-0.002, -0.006),
    (0.005, -0.006),
]

# ---- Lower grip (overmolded handle at -Y offset, extends in -X).
# Starts at x=-0.025 to overlap with the tang.
GRIP_LOWER_PTS = [
    (-0.025, -0.005),    # inner start (overlaps tang)
    (-0.028, -0.006),
    (-0.030, -0.009),    # leftmost transition
    (-0.028, -0.013),    # outer transition (overlaps tang)
    (-0.040, -0.015),
    (-0.060, -0.016),
    (-0.082, -0.017),
    (-0.102, -0.016),
    (-0.118, -0.014),
    (-0.122, -0.011),
    (-0.115, -0.008),
    (-0.095, -0.007),
    (-0.075, -0.006),
    (-0.055, -0.005),
    (-0.038, -0.004),
    (-0.028, -0.004),
]

# ---- Lower inlay strip on top of grip (well within grip footprint)
INLAY_LOWER_PTS = [
    (-0.042, -0.006),
    (-0.060, -0.008),
    (-0.080, -0.009),
    (-0.100, -0.010),
    (-0.110, -0.011),
    (-0.108, -0.013),
    (-0.090, -0.014),
    (-0.070, -0.013),
    (-0.052, -0.011),
    (-0.040, -0.008),
]


def _mirror_y(pts: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Mirror point list about the X axis (negate Y)."""
    return [(x, -y) for (x, y) in pts]


def _poly(pts: list[tuple[float, float]], z0: float, z1: float) -> cq.Workplane:
    """Polyline prism extruded from z0 to z1."""
    return (
        cq.Workplane("XY", origin=(0.0, 0.0, z0))
        .polyline(pts)
        .close()
        .extrude(z1 - z0)
    )


def _spline_wire(
    pts: list[tuple[float, float]], z: float, inset: float = 0.0
) -> cq.Wire:
    edge = cq.Edge.makeSpline(
        [cq.Vector(x, y, z) for (x, y) in pts], periodic=True
    )
    wire = cq.Wire.assembleEdges([edge])
    if inset:
        wire = wire.offset2D(-inset)[0]
    return wire


def _spline_prism(
    pts: list[tuple[float, float]], z0: float, z1: float
) -> cq.Workplane:
    """Extruded periodic-spline prism (no loft caps)."""
    face = cq.Face.makeFromWires(_spline_wire(pts, z0))
    return cq.Workplane(
        obj=cq.Solid.extrudeLinear(face, cq.Vector(0.0, 0.0, z1 - z0))
    )


def _boss(z0: float, z1: float, with_hole: bool) -> cq.Workplane:
    boss = (
        cq.Workplane("XY", origin=(0.0, 0.0, z0))
        .circle(BOSS_R)
        .extrude(z1 - z0)
    )
    if with_hole:
        hole = (
            cq.Workplane("XY", origin=(0.0, 0.0, z0 - 0.001))
            .circle(HOLE_R)
            .extrude((z1 - z0) + 0.002)
        )
        boss = boss.cut(hole)
    return boss


def _rivet_cap(z_base: float, direction: int) -> cq.Workplane:
    """Circular rivet cap disc. direction=+1 for top cap, -1 for bottom cap."""
    if direction > 0:
        z0 = z_base
    else:
        z0 = z_base - CAP_T
    cap = (
        cq.Workplane("XY", origin=(0.0, 0.0, z0))
        .circle(CAP_R)
        .extrude(CAP_T)
    )
    try:
        if direction > 0:
            cap = cap.edges(">Z").fillet(0.0007)
        else:
            cap = cap.edges("<Z").fillet(0.0007)
    except Exception:
        pass
    return cap


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="end_cutting_nippers")

    steel = model.material("brushed_steel", rgba=(0.72, 0.73, 0.75, 1.0))
    polished = model.material("polished_steel", rgba=(0.86, 0.87, 0.90, 1.0))
    orange = model.material("orange_grip", rgba=(0.92, 0.35, 0.06, 1.0))
    peach = model.material("peach_inlay", rgba=(1.0, 0.80, 0.64, 0.78))

    # Pose offsets: lower half yawed +HALF_OPEN at rest
    lower_pose = Origin(rpy=(0.0, 0.0, HALF_OPEN))

    # ---- lower half (base link) ----
    lower = model.part("lower_half")

    # Jaw: extends in +Y (perpendicular to handle)
    lower.visual(
        mesh_from_cadquery(_poly(JAW_PTS, ZL0, ZL1), "lower_jaw"),
        origin=lower_pose,
        material=steel,
        name="jaw",
    )
    # Pivot boss
    lower.visual(
        mesh_from_cadquery(_boss(ZL0, ZL1, with_hole=False), "lower_boss"),
        origin=lower_pose,
        material=steel,
        name="pivot_boss",
    )
    # Tang/neck connecting boss to handle
    lower.visual(
        mesh_from_cadquery(_poly(TANG_LOWER_PTS, ZL0, ZL1), "lower_tang"),
        origin=lower_pose,
        material=steel,
        name="neck",
    )
    # Overmolded grip
    lower.visual(
        mesh_from_cadquery(
            _spline_prism(GRIP_LOWER_PTS, GRIP_LZ0, GRIP_LZ1), "lower_grip"
        ),
        origin=lower_pose,
        material=orange,
        name="grip",
    )
    # Translucent inlay strip
    lower.visual(
        mesh_from_cadquery(
            _spline_prism(
                INLAY_LOWER_PTS,
                GRIP_LZ1 - INLAY_EMBED,
                GRIP_LZ1 - INLAY_EMBED + INLAY_T,
            ),
            "lower_inlay",
        ),
        origin=lower_pose,
        material=peach,
        name="grip_inlay",
    )
    # Bottom rivet cap (circular, polished)
    lower.visual(
        mesh_from_cadquery(_rivet_cap(ZL0, -1), "lower_cap"),
        origin=lower_pose,
        material=polished,
        name="rivet_cap",
    )

    # ---- upper half (moving link) ----
    upper = model.part("upper_half")

    # Jaw: same profile, both extend in +Y
    upper.visual(
        mesh_from_cadquery(_poly(JAW_PTS, ZU0, ZU1), "upper_jaw"),
        material=steel,
        name="jaw",
    )
    # Pivot boss (with hole for rivet shank)
    upper.visual(
        mesh_from_cadquery(_boss(ZU0, ZU1, with_hole=True), "upper_boss"),
        material=steel,
        name="pivot_boss",
    )
    # Tang (mirrored in Y: handle on +Y side)
    upper.visual(
        mesh_from_cadquery(
            _poly(_mirror_y(TANG_LOWER_PTS), ZU0, ZU1), "upper_tang"
        ),
        material=steel,
        name="neck",
    )
    # Overmolded grip (mirrored in Y)
    upper.visual(
        mesh_from_cadquery(
            _spline_prism(_mirror_y(GRIP_LOWER_PTS), GRIP_UZ0, GRIP_UZ1),
            "upper_grip",
        ),
        material=orange,
        name="grip",
    )
    # Translucent inlay strip (mirrored)
    upper.visual(
        mesh_from_cadquery(
            _spline_prism(
                _mirror_y(INLAY_LOWER_PTS),
                GRIP_UZ1 - INLAY_EMBED,
                GRIP_UZ1 - INLAY_EMBED + INLAY_T,
            ),
            "upper_inlay",
        ),
        material=peach,
        name="grip_inlay",
    )
    # Top rivet cap (circular, polished)
    upper.visual(
        mesh_from_cadquery(_rivet_cap(ZU1, +1), "upper_cap"),
        material=polished,
        name="rivet_cap",
    )

    # ---- single revolute pivot at the rivet, axis perpendicular to tool plane.
    # q=0 is splayed rest; positive q closes jaw edges and handles together.
    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, -HALF_OPEN)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0, velocity=4.0, lower=0.0, upper=CLOSE_TRAVEL
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower = object_model.get_part("lower_half")
    upper = object_model.get_part("upper_half")
    pivot = object_model.get_articulation("pivot")

    # --- jaws are perpendicular to handles: jaw extends in Y, handle in X
    jaw_aabb_l = ctx.part_element_world_aabb(lower, elem="jaw")
    grip_aabb_l = ctx.part_element_world_aabb(lower, elem="grip")
    if jaw_aabb_l is not None and grip_aabb_l is not None:
        jaw_dy = jaw_aabb_l[1][1] - jaw_aabb_l[0][1]
        jaw_dx = jaw_aabb_l[1][0] - jaw_aabb_l[0][0]
        grip_dx = grip_aabb_l[1][0] - grip_aabb_l[0][0]
        grip_dy = grip_aabb_l[1][1] - grip_aabb_l[0][1]
        ctx.check(
            "jaw extends more in Y than X (perpendicular to handle)",
            jaw_dy > jaw_dx * 1.5,
            details=f"jaw_dx={jaw_dx:.4f}, jaw_dy={jaw_dy:.4f}",
        )
        ctx.check(
            "handle extends more in X than Y (along handle axis)",
            grip_dx > grip_dy * 2.0,
            details=f"grip_dx={grip_dx:.4f}, grip_dy={grip_dy:.4f}",
        )
        ctx.check(
            "jaw is forward of the handle (jaw Y center > grip Y center)",
            (jaw_aabb_l[0][1] + jaw_aabb_l[1][1]) / 2
            > (grip_aabb_l[0][1] + grip_aabb_l[1][1]) / 2 + 0.010,
            details=f"jaw_y_center={(jaw_aabb_l[0][1] + jaw_aabb_l[1][1]) / 2:.4f}, "
            f"grip_y_center={(grip_aabb_l[0][1] + grip_aabb_l[1][1]) / 2:.4f}",
        )

    # --- circular pivot rivet caps exist on both sides
    for half, name in [(lower, "lower"), (upper, "upper")]:
        cap_aabb = ctx.part_element_world_aabb(half, elem="rivet_cap")
        ctx.check(
            f"{name} rivet cap exists",
            cap_aabb is not None,
            details="rivet_cap AABB is None",
        )
        if cap_aabb is not None:
            cap_dx = cap_aabb[1][0] - cap_aabb[0][0]
            cap_dy = cap_aabb[1][1] - cap_aabb[0][1]
            ctx.check(
                f"{name} rivet cap is roughly circular",
                abs(cap_dx - cap_dy) < 0.003,
                details=f"cap_dx={cap_dx:.4f}, cap_dy={cap_dy:.4f}",
            )

    # Bottom cap is below top cap
    bot_cap = ctx.part_element_world_aabb(lower, elem="rivet_cap")
    top_cap = ctx.part_element_world_aabb(upper, elem="rivet_cap")
    if bot_cap is not None and top_cap is not None:
        ctx.check(
            "bottom rivet cap is below top rivet cap",
            bot_cap[0][2] < top_cap[0][2],
            details=f"bot_z0={bot_cap[0][2]:.4f}, top_z0={top_cap[0][2]:.4f}",
        )

    # --- pivot bosses stack coaxially
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
        min_gap=-0.0001, max_gap=0.0006,
        name="upper boss sits on lower boss",
    )

    # --- rest pose: jaws tips on opposite sides (open), handles splayed
    lower_jaw_aabb = ctx.part_element_world_aabb(lower, elem="jaw")
    upper_jaw_aabb = ctx.part_element_world_aabb(upper, elem="jaw")
    if lower_jaw_aabb is not None and upper_jaw_aabb is not None:
        lower_jaw_cx = (lower_jaw_aabb[0][0] + lower_jaw_aabb[1][0]) / 2
        upper_jaw_cx = (upper_jaw_aabb[0][0] + upper_jaw_aabb[1][0]) / 2
        ctx.check(
            "jaw tips on opposite sides of center at rest (open)",
            lower_jaw_cx < upper_jaw_cx - 0.004,
            details=f"lower_cx={lower_jaw_cx:.4f}, upper_cx={upper_jaw_cx:.4f}",
        )
    ctx.expect_gap(
        upper, lower,
        axis="y",
        positive_elem="grip", negative_elem="grip",
        min_gap=0.005,
        name="handles splay apart at rest",
    )

    # --- articulation is non-fixed revolute with correct travel
    limits = pivot.motion_limits
    ctx.check(
        "pivot is revolute",
        pivot.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={pivot.articulation_type}",
    )
    ctx.check(
        "pivot travel is roughly 0..25 degrees",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower) < 1e-9
        and 0.38 <= limits.upper <= 0.48,
        details=f"limits={limits}",
    )

    # --- closing: jaws come together, handles come together
    open_jaw = ctx.part_element_world_aabb(upper, elem="jaw")
    open_grip = ctx.part_element_world_aabb(upper, elem="grip")
    with ctx.pose({pivot: CLOSE_TRAVEL}):
        closed_jaw = ctx.part_element_world_aabb(upper, elem="jaw")
        closed_grip = ctx.part_element_world_aabb(upper, elem="grip")

    if open_jaw is not None and closed_jaw is not None:
        ctx.check(
            "closing swings upper jaw toward lower jaw",
            abs(closed_jaw[0][0] - open_jaw[0][0]) > 0.003
            or abs(closed_jaw[1][0] - open_jaw[1][0]) > 0.003,
            details=f"open_x={open_jaw[0][0]:.4f}..{open_jaw[1][0]:.4f}, "
            f"closed_x={closed_jaw[0][0]:.4f}..{closed_jaw[1][0]:.4f}",
        )
    if open_grip is not None and closed_grip is not None:
        ctx.check(
            "handles scissor together when closing",
            closed_grip[0][1] < open_grip[0][1] - 0.003
            or closed_grip[1][1] > open_grip[1][1] + 0.003,
            details=f"open_grip_y={open_grip[0][1]:.4f}..{open_grip[1][1]:.4f}, "
            f"closed_grip_y={closed_grip[0][1]:.4f}..{closed_grip[1][1]:.4f}",
        )

    # --- overall proportions
    la = ctx.part_world_aabb(lower)
    ua = ctx.part_world_aabb(upper)
    if la is not None and ua is not None:
        length = max(la[1][0], ua[1][0]) - min(la[0][0], ua[0][0])
        width = max(la[1][1], ua[1][1]) - min(la[0][1], ua[0][1])
        height = max(la[1][2], ua[1][2]) - min(la[0][2], ua[0][2])
        ctx.check(
            "overall extent ~0.12-0.18 m",
            0.10 <= length <= 0.20,
            details=f"length={length:.4f}",
        )
        ctx.check(
            "tool thickness ~0.012-0.020 m",
            0.012 <= height <= 0.022,
            details=f"height={height:.4f}",
        )
    else:
        ctx.fail("overall proportions", "missing part AABBs")

    return ctx.report()


object_model = build_object_model()
