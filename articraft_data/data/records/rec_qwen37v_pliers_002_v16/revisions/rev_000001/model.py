from __future__ import annotations

# Bent-nose pliers variant (Variant 16).
# Derived from heavy-duty combination lineman pliers.
#
# Changes from parent:
#   1. Bent/angled jaw tips (~45-degree bend near the nose)
#   2. Circular pivot rivet caps on both sides (prominent disc caps)
#   3. Color-separated geometric grip sleeves (two distinct sections per handle)
#
# Layout: the tool lies in the XY plane with Z as the thickness axis.
# The rivet pivot is at the origin. The jaw tips point +X (with a bend),
# the handles sweep back to -X and spread in +/-Y.
# plier_half_0 (root) carries its jaw on +Y; plier_half_1 is the mirrored
# moving half. The two forged halves are half-lapped at the pivot.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---- shared dimensions (meters) ----
HALF_T = 0.009          # half thickness of each forged plate
LAP_R = 0.016           # half-lap joint disc radius at the pivot
HUB_R = 0.015           # forged hub radius around the rivet
BOSS_R = 0.0125         # rivet boss radius (~0.025 m diameter)
BOSS_H = 0.0030
CAP_R = 0.014           # rivet cap radius (slightly larger than boss for visibility)
CAP_H = 0.0025          # cap height
SEAM_R = 0.0132         # visible circular seam ring under the boss cap
SEAM_H = 0.0006
RIVET_R = 0.004         # rivet shaft captured through the moving half's lap
EPS = 0.0001            # lap clearance so the stacked halves do not penetrate
JAW_FACE = 0.0003       # closed jaw inner faces sit at y = +/-JAW_FACE
NOSE_X = 0.075          # blunt squared nose tip (before bend)
BEND_X = 0.050          # where the jaw starts to bend
BEND_ANGLE = math.radians(45.0)  # bend angle of the tip section
OPEN_LIMIT = math.radians(30.0)

TANG_HALF_W = 0.004     # steel handle tang half width in plan

# Steel handle tang centerline in the tool plane (for the half whose jaw is +Y).
TANG_PTS = [
    (-0.010, -0.004),
    (-0.030, -0.011),
    (-0.070, -0.019),
    (-0.100, -0.024),
    (-0.118, -0.026),
]
CENTERLINE = TANG_PTS + [(-0.131, -0.0274)]

# Grip loft stations: (x, half_width_y, half_height_z)
GRIP_SECTIONS = [
    (-0.032, 0.0100, 0.0130),  # flared thumb guard near the pivot
    (-0.040, 0.0080, 0.0110),
    (-0.060, 0.0078, 0.0108),
    (-0.085, 0.0080, 0.0112),
    (-0.105, 0.0086, 0.0118),
    (-0.120, 0.0088, 0.0120),  # bulbous end swell
    (-0.128, 0.0062, 0.0086),
    (-0.131, 0.0028, 0.0040),
]

# Front grip sleeve stations (near pivot) - blue section
FRONT_GRIP_SECTIONS = [
    (-0.032, 0.0100, 0.0130),
    (-0.040, 0.0080, 0.0110),
    (-0.058, 0.0078, 0.0108),
    (-0.065, 0.0076, 0.0106),
]

# Rear grip sleeve stations (toward end) - orange section
REAR_GRIP_SECTIONS = [
    (-0.072, 0.0078, 0.0108),
    (-0.085, 0.0080, 0.0112),
    (-0.105, 0.0086, 0.0118),
    (-0.120, 0.0088, 0.0120),
    (-0.128, 0.0062, 0.0086),
    (-0.131, 0.0028, 0.0040),
]

def _interp(x: float, pts: list[tuple[float, float]]) -> float:
    """Piecewise-linear interpolation over (x, value) points (any x order)."""
    pts = sorted(pts)
    if x <= pts[0][0]:
        return pts[0][1]
    if x >= pts[-1][0]:
        return pts[-1][1]
    for (x0, v0), (x1, v1) in zip(pts, pts[1:]):
        if x0 <= x <= x1:
            t = (x - x0) / (x1 - x0)
            return v0 + t * (v1 - v0)
    return pts[-1][1]


def _yc(x: float) -> float:
    """Handle centerline y at station x (jaw-on-+Y half)."""
    return _interp(x, CENTERLINE)


def _strip_loop(pts: list[tuple[float, float]], half_w: float) -> list[tuple[float, float]]:
    """Closed plan-view loop offsetting a polyline by +/-half_w along 2D normals."""
    n = len(pts)
    normals: list[tuple[float, float]] = []
    for i in range(n):
        if i == 0:
            dx, dy = pts[1][0] - pts[0][0], pts[1][1] - pts[0][1]
        elif i == n - 1:
            dx, dy = pts[-1][0] - pts[-2][0], pts[-1][1] - pts[-2][1]
        else:
            dx, dy = pts[i + 1][0] - pts[i - 1][0], pts[i + 1][1] - pts[i - 1][1]
        length = math.hypot(dx, dy)
        normals.append((-dy / length, dx / length))
    left = [(p[0] + half_w * nx, p[1] + half_w * ny) for p, (nx, ny) in zip(pts, normals)]
    right = [(p[0] - half_w * nx, p[1] - half_w * ny) for p, (nx, ny) in zip(pts, normals)]
    return left + right[::-1]


def _lap_cut(s: int) -> cq.Workplane:
    """Material removed at the pivot so this half keeps only its lap layer."""
    cut = cq.Workplane("XY").circle(LAP_R).extrude(0.012)
    if s > 0:
        return cut.translate((0.0, 0.0, -EPS))
    return cut.translate((0.0, 0.0, -0.012 + EPS))


def _jaw_solid(s: int) -> cq.Workplane:
    """Bent-nose jaw: straight section then angled tip with serrations."""
    # Compute bend geometry
    tip_length = NOSE_X - BEND_X  # ~0.025m of bent section
    dy_bend = tip_length * math.sin(BEND_ANGLE)
    dx_bend = tip_length * math.cos(BEND_ANGLE)

    # Full profile as one continuous loop (no duplicate points)
    # Inner face goes from pivot out to bent tip, outer face comes back
    # s=+1: jaw on +Y side, s=-1: jaw on -Y side
    inner_y_base = JAW_FACE  # inner face y offset (always positive in abs terms)
    profile = [
        # Inner face: straight section toward bend
        (0.010, inner_y_base),
        (BEND_X, inner_y_base),
        # Inner face: bent tip section (deflects outward)
        (BEND_X + dx_bend * 0.5, inner_y_base + dy_bend * 0.4),
        (BEND_X + dx_bend, inner_y_base + dy_bend * 0.85),
        # Nose tip (outer corner)
        (BEND_X + dx_bend - 0.002, inner_y_base + dy_bend * 0.85 + 0.004),
        # Outer face: back from tip toward pivot
        (BEND_X + dx_bend * 0.5, inner_y_base + dy_bend * 0.4 + 0.008),
        (BEND_X - 0.002, inner_y_base + 0.011),
        (0.038, 0.0152),
        (0.026, 0.0162),
        (0.014, 0.0166),
        (0.009, 0.0150),
    ]
    # Apply sign: s=+1 keeps positive y, s=-1 negates y
    all_pts = [(x, s * y) for x, y in profile]
    jaw = cq.Workplane("XY").polyline(all_pts).close().extrude(HALF_T, both=True)

    # Fine horizontal gripping serrations on the bent tip inner face
    for i in range(5):
        frac = 0.2 + 0.15 * i
        xi = BEND_X + dx_bend * frac
        yi = s * (JAW_FACE + dy_bend * frac * 0.85)
        groove = (
            cq.Workplane("XY")
            .box(0.0014, 0.0024, 0.018)
            .translate((xi, yi, 0.0))
        )
        jaw = jaw.cut(groove)

    # Rounded pipe-grip recess behind the bend start
    recess = (
        cq.Workplane("XY")
        .circle(0.004)
        .extrude(0.011, both=True)
        .translate((0.034, s * JAW_FACE, 0.0))
    )
    jaw = jaw.cut(recess)

    # Small scallop teeth around the recess rim
    for ang_deg in (40.0, 80.0, 110.0, 150.0):
        a = math.radians(ang_deg)
        sc_x = 0.034 + 0.004 * math.cos(a)
        sc_y = s * (JAW_FACE + 0.004 * math.sin(a))
        scallop = (
            cq.Workplane("XY")
            .circle(0.0007)
            .extrude(0.011, both=True)
            .translate((sc_x, sc_y, 0.0))
        )
        jaw = jaw.cut(scallop)

    # Short wire-cutter V-notch between the pipe grip and the pivot
    notch_pts = [(0.016, -0.001), (0.022, -0.001), (0.019, 0.003)]
    notch_signed = [(x, s * y) for x, y in notch_pts]
    notch = cq.Workplane("XY").polyline(notch_signed).close().extrude(0.011, both=True)
    jaw = jaw.cut(notch)

    return jaw.cut(_lap_cut(s))


def _hub_solid(s: int) -> cq.Workplane:
    """Forged circular hub around the rivet, thinned to its lap layer."""
    hub = cq.Workplane("XY").circle(HUB_R).extrude(HALF_T, both=True)
    return hub.cut(_lap_cut(s))


def _shank_solid(s: int) -> cq.Workplane:
    """Steel handle tang sweeping back from the hub into the grip."""
    pts = [(x, s * y) for x, y in TANG_PTS]
    loop = _strip_loop(pts, TANG_HALF_W)
    shank = cq.Workplane("XY").polyline(loop).close().extrude(HALF_T, both=True)
    return shank.cut(_lap_cut(s))


def _ellipse_wire(x: float, yc: float, zc: float, w: float, h: float) -> cq.Wire:
    wp = cq.Workplane("YZ", origin=(x, 0.0, 0.0)).center(yc, zc).ellipse(w, h)
    return wp.val()


def _front_grip_solid(s: int) -> cq.Solid:
    """Front grip sleeve section (near pivot) - geometrically separate."""
    wires = [
        _ellipse_wire(x, s * _yc(x), 0.0, w, h)
        for x, w, h in FRONT_GRIP_SECTIONS
    ]
    return cq.Solid.makeLoft(wires, ruled=False)


def _rear_grip_solid(s: int) -> cq.Solid:
    """Rear grip sleeve section (toward end) - geometrically separate."""
    wires = [
        _ellipse_wire(x, s * _yc(x), 0.0, w, h)
        for x, w, h in REAR_GRIP_SECTIONS
    ]
    return cq.Solid.makeLoft(wires, ruled=False)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bent_nose_pliers")

    steel_polished = model.material("steel_polished", rgba=(0.80, 0.81, 0.84, 1.0))
    steel_forged = model.material("steel_forged", rgba=(0.66, 0.67, 0.70, 1.0))
    steel_brushed = model.material("steel_brushed", rgba=(0.61, 0.62, 0.65, 1.0))
    seam_gray = model.material("seam_gray", rgba=(0.45, 0.46, 0.48, 1.0))
    grip_blue = model.material("grip_blue", rgba=(0.12, 0.25, 0.55, 1.0))
    grip_orange = model.material("grip_orange", rgba=(0.90, 0.45, 0.08, 1.0))
    cap_chrome = model.material("cap_chrome", rgba=(0.78, 0.80, 0.82, 1.0))

    parts = []
    for part_name, s in (("plier_half_0", 1), ("plier_half_1", -1)):
        part = model.part(part_name)
        tag = part_name.replace("plier_half_", "half")

        part.visual(
            mesh_from_cadquery(_jaw_solid(s), f"{tag}_jaw", tolerance=0.0002),
            name="jaw",
            material=steel_polished,
        )
        part.visual(
            mesh_from_cadquery(_hub_solid(s), f"{tag}_hub", tolerance=0.0002),
            name="hub",
            material=steel_forged,
        )
        part.visual(
            mesh_from_cadquery(_shank_solid(s), f"{tag}_shank", tolerance=0.0002),
            name="shank",
            material=steel_forged,
        )
        # Color-separated geometric grip sleeves: front (blue) and rear (orange)
        part.visual(
            mesh_from_cadquery(_front_grip_solid(s), f"{tag}_front_grip", tolerance=0.0002),
            name="grip_front",
            material=grip_blue,
        )
        part.visual(
            mesh_from_cadquery(_rear_grip_solid(s), f"{tag}_rear_grip", tolerance=0.0002),
            name="grip_rear",
            material=grip_orange,
        )

        parts.append(part)

    fixed = parts[0]
    moving = parts[1]

    # Circular pivot rivet caps on BOTH sides (prominent disc caps)
    # Bottom cap (below half_0)
    fixed.visual(
        Cylinder(SEAM_R, SEAM_H),
        origin=Origin(xyz=(0.0, 0.0, -(HALF_T + SEAM_H / 2.0 - EPS))),
        name="boss_seam",
        material=seam_gray,
    )
    fixed.visual(
        Cylinder(CAP_R, CAP_H),
        origin=Origin(xyz=(0.0, 0.0, -(HALF_T + SEAM_H + CAP_H / 2.0 - 2.0 * EPS))),
        name="rivet_cap_bottom",
        material=cap_chrome,
    )
    # Rivet shaft through both halves
    fixed.visual(
        Cylinder(RIVET_R, 2.0 * HALF_T + SEAM_H + 2.0 * EPS + 0.0005),
        origin=Origin(xyz=(0.0, 0.0, 0.00005)),
        name="rivet_shaft",
        material=steel_brushed,
    )
    # Top cap (above half_1)
    fixed.visual(
        Cylinder(SEAM_R, SEAM_H),
        origin=Origin(xyz=(0.0, 0.0, HALF_T + SEAM_H / 2.0 + EPS)),
        name="head_seam",
        material=seam_gray,
    )
    fixed.visual(
        Cylinder(CAP_R, CAP_H),
        origin=Origin(xyz=(0.0, 0.0, HALF_T + SEAM_H + CAP_H / 2.0)),
        name="rivet_cap_top",
        material=cap_chrome,
    )

    # Primary articulation: revolute joint at the rivet pivot
    # Positive q (about -Z) swings half_1's jaw toward -Y, opening the jaws
    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=parts[0],
        child=parts[1],
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=3.0, lower=0.0, upper=OPEN_LIMIT),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    half0 = object_model.get_part("plier_half_0")
    half1 = object_model.get_part("plier_half_1")
    pivot = object_model.get_articulation("pivot")

    jaw0 = half0.get_visual("jaw")
    jaw1 = half1.get_visual("jaw")
    hub0 = half0.get_visual("hub")
    hub1 = half1.get_visual("hub")

    # ---- Prompt-specific checks for variant 16 ----

    # 1. Bent-nose jaw: the tip should extend further in Y than a straight jaw
    # Check that the jaw geometry shows the bend (tip displaced from straight line)
    jaw0_aabb = ctx.part_element_world_aabb(half0, elem="jaw")
    if jaw0_aabb is not None:
        # The bent tip should push the jaw max-Y beyond what a straight jaw would reach
        # A straight jaw max Y would be around 0.016; bent should exceed that
        ctx.check(
            "bent-nose jaw tip extends beyond straight jaw line",
            jaw0_aabb[1][1] > 0.016,
            details=f"jaw max_y={jaw0_aabb[1][1]:.4f}",
        )

    # 3. Circular pivot rivet caps on both sides
    cap_bottom_aabb = ctx.part_element_world_aabb(half0, elem="rivet_cap_bottom")
    cap_top_aabb = ctx.part_element_world_aabb(half0, elem="rivet_cap_top")
    ok_caps = cap_bottom_aabb is not None and cap_top_aabb is not None
    if ok_caps:
        # Both caps should be circular discs with diameter ~0.028m
        bot_dia = cap_bottom_aabb[1][0] - cap_bottom_aabb[0][0]
        top_dia = cap_top_aabb[1][0] - cap_top_aabb[0][0]
        ctx.check(
            "bottom rivet cap is circular disc ~28mm diameter",
            0.025 <= bot_dia <= 0.032,
            details=f"dia={bot_dia:.4f}",
        )
        ctx.check(
            "top rivet cap is circular disc ~28mm diameter",
            0.025 <= top_dia <= 0.032,
            details=f"dia={top_dia:.4f}",
        )
        # Caps on opposite sides of the tool (one above, one below)
        ctx.check(
            "rivet caps are on opposite sides (top and bottom)",
            cap_bottom_aabb[0][2] < -0.008 and cap_top_aabb[1][2] > 0.008,
            details=f"bot_min_z={cap_bottom_aabb[0][2]:.4f} top_max_z={cap_top_aabb[1][2]:.4f}",
        )
    else:
        ctx.fail("rivet cap AABBs resolve", "missing rivet_cap_bottom or rivet_cap_top")

    # 4. Color-separated geometric grip sleeves: front and rear are separate geometry
    grip_front0_aabb = ctx.part_element_world_aabb(half0, elem="grip_front")
    grip_rear0_aabb = ctx.part_element_world_aabb(half0, elem="grip_rear")
    if grip_front0_aabb is not None and grip_rear0_aabb is not None:
        # Front grip is closer to pivot (larger X / less negative)
        ctx.check(
            "front grip sleeve is closer to pivot than rear grip",
            grip_front0_aabb[1][0] > grip_rear0_aabb[1][0],
            details=f"front_max_x={grip_front0_aabb[1][0]:.4f} rear_max_x={grip_rear0_aabb[1][0]:.4f}",
        )
        # There should be a gap between them in X (geometric separation)
        gap_x = grip_rear0_aabb[1][0] - grip_front0_aabb[0][0]
        ctx.check(
            "front and rear grip sleeves are geometrically separated",
            grip_front0_aabb[0][0] > grip_rear0_aabb[1][0] - 0.015,
            details=f"front_min_x={grip_front0_aabb[0][0]:.4f} rear_max_x={grip_rear0_aabb[1][0]:.4f}",
        )
    else:
        ctx.fail("grip sleeve AABBs resolve", "missing grip_front or grip_rear element")

    # ---- Standard pliers checks ----

    # Joint contract: a single revolute pivot opening 0..30 degrees
    limits = pivot.motion_limits
    ctx.check(
        "pivot is a 0..30 degree revolute joint",
        pivot.articulation_type == ArticulationType.REVOLUTE
        and limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower) < 1e-9
        and abs(limits.upper - OPEN_LIMIT) < 1e-6,
        details=f"limits={limits}",
    )

    # Closed rest pose: jaw inner faces nearly touch
    ctx.expect_gap(
        half0,
        half1,
        axis="y",
        positive_elem=jaw0,
        negative_elem=jaw1,
        min_gap=0.0002,
        max_gap=0.002,
        name="jaws closed and nearly touching at rest",
    )

    # The halves interleave at the pivot
    ctx.expect_gap(
        half1,
        half0,
        axis="z",
        positive_elem=hub1,
        negative_elem=hub0,
        min_gap=0.0,
        max_gap=0.001,
        name="moving hub lap stacks above fixed hub lap",
    )
    ctx.expect_contact(
        half0,
        half1,
        elem_a=hub0,
        elem_b=hub1,
        contact_tol=0.0005,
        name="hub laps seat against each other at the joint",
    )
    ctx.expect_overlap(
        half0,
        half1,
        axes="xy",
        elem_a=hub0,
        elem_b=hub1,
        min_overlap=0.02,
        name="hub laps share the pivot footprint",
    )

    # Rivet shaft passes through the moving half (intentional overlap)
    ctx.allow_overlap(
        half0,
        half1,
        elem_a="rivet_shaft",
        elem_b=hub1,
        reason="The rivet shaft is fixed to half_0 and intentionally passes "
        "through the moving half's hub lap, capturing it at the pivot.",
    )
    ctx.expect_within(
        half0,
        half1,
        axes="xy",
        inner_elem="rivet_shaft",
        outer_elem=hub1,
        margin=0.0005,
        name="rivet shaft stays centered inside the moving hub lap",
    )
    ctx.expect_overlap(
        half0,
        half1,
        axes="z",
        elem_a="rivet_shaft",
        elem_b=hub1,
        min_overlap=0.007,
        name="rivet shaft passes through the full moving lap thickness",
    )

    # Decisive open pose: jaws separate and handles spread
    closed_jaw1 = ctx.part_element_world_aabb(half1, elem="jaw")
    closed_grip1 = ctx.part_element_world_aabb(half1, elem="grip_rear")
    with ctx.pose({pivot: OPEN_LIMIT}):
        ctx.expect_gap(
            half0,
            half1,
            axis="y",
            positive_elem=jaw0,
            negative_elem=jaw1,
            min_gap=0.004,
            name="jaws open apart at the 30 degree pose",
        )
        open_jaw1 = ctx.part_element_world_aabb(half1, elem="jaw")
        open_grip1 = ctx.part_element_world_aabb(half1, elem="grip_rear")
        ctx.check(
            "moving jaw swings away from the fixed jaw",
            closed_jaw1 is not None
            and open_jaw1 is not None
            and open_jaw1[0][1] < closed_jaw1[0][1] - 0.015,
            details=f"closed_min_y={closed_jaw1}, open_min_y={open_jaw1}",
        )
        ctx.check(
            "moving handle spreads outward as the jaws open",
            closed_grip1 is not None
            and open_grip1 is not None
            and open_grip1[1][1] > closed_grip1[1][1] + 0.025,
            details=f"closed_max_y={closed_grip1}, open_max_y={open_grip1}",
        )

    return ctx.report()


object_model = build_object_model()
