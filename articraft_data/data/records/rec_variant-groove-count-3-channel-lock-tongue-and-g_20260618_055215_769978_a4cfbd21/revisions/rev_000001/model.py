from __future__ import annotations

# Channel-lock tongue-and-groove pliers variant.
# Three adjustment grooves on the lower jaw shank.
# PRISMATIC pivot-index slide + REVOLUTE jaw open/close.
#
# Layout: the tool lies in the XY plane with Z as the thickness axis.
# The nominal pivot is at the origin. The blunt squared nose points +X,
# the handles sweep back to -X and spread in +/-Y.
# upper_half (root, s=1) carries its jaw on +Y; lower_half (s=-1) is the
# moving half with jaw on -Y. The slide_carriage bridges them at the pivot.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---- shared dimensions (meters) ----
GROOVE_COUNT = 3
GROOVE_SPACING = 0.008       # 8 mm between groove centres
HALF_T = 0.009               # half thickness of each forged plate
LAP_R = 0.019                # half-lap joint disc radius at the pivot (larger than hub)
HUB_R = 0.016                # forged hub radius around the pivot
BOSS_R = 0.0125              # rivet boss radius
JAW_FACE = 0.0003            # closed jaw inner faces sit at y = +/-JAW_FACE
NOSE_X = 0.075               # blunt squared nose tip
EPS = 0.0001                 # lap clearance
OPEN_LIMIT = math.radians(30.0)
SLIDE_UPPER = GROOVE_SPACING * (GROOVE_COUNT - 1)  # 0.016 m total travel

TANG_HALF_W = 0.004          # steel handle tang half width in plan

TANG_PTS = [
    (-0.010, -0.004),
    (-0.030, -0.011),
    (-0.070, -0.019),
    (-0.100, -0.024),
    (-0.118, -0.026),
]
CENTERLINE = TANG_PTS + [(-0.131, -0.0274)]

GRIP_SECTIONS = [
    (-0.032, 0.0100, 0.0130),
    (-0.040, 0.0080, 0.0110),
    (-0.060, 0.0078, 0.0108),
    (-0.085, 0.0080, 0.0112),
    (-0.105, 0.0086, 0.0118),
    (-0.120, 0.0088, 0.0120),
    (-0.128, 0.0062, 0.0086),
    (-0.131, 0.0028, 0.0040),
]

INLAY_XS = [-0.036, -0.055, -0.075, -0.095, -0.112, -0.118]
INLAY_HALF_H = 0.0075
INLAY_Z_CENTER = 0.006

# Groove geometry
GROOVE_W = 0.003             # groove width along shank (X)
GROOVE_Y_SPAN = 0.010        # groove span across shank (Y)
GROOVE_Z_DEPTH = 0.004       # groove depth into shank (Z)

# Channel boss geometry (upper half)
BOSS_X = 0.040               # boss extent along shank
BOSS_Y = 0.028               # boss width across shank
BOSS_PLATE_T = 0.003         # boss plate thickness (each side)
BOSS_FILLET = 0.006

# Pivot pin / carriage
PIN_R = 0.0045
PIN_H = 2.0 * HALF_T + BOSS_PLATE_T + 0.002
COLLAR_R = 0.007
COLLAR_H = 0.003


def _interp(x: float, pts: list[tuple[float, float]]) -> float:
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
    return _interp(x, CENTERLINE)


def _grip_w(x: float) -> float:
    return _interp(x, [(sx, w) for sx, w, _h in GRIP_SECTIONS])


def _strip_loop(pts: list[tuple[float, float]], half_w: float) -> list[tuple[float, float]]:
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
    """Half-lap at the pivot.

    s=+1 (upper half): keep UPPER lap (z >= +EPS), remove below.
    s=-1 (lower half): keep LOWER lap (z <= -EPS), remove above.
    """
    cut_h = HALF_T + 0.002
    cut = cq.Workplane("XY").circle(LAP_R).extrude(cut_h)
    if s > 0:
        return cut.translate((0.0, 0.0, -(cut_h) + EPS))
    return cut.translate((0.0, 0.0, -EPS))


def _jaw_solid(s: int) -> cq.Workplane:
    """Polished jaw: squared nose, serrations, pipe-grip recess, wire cutter."""
    profile = [
        (0.010, JAW_FACE),
        (NOSE_X, JAW_FACE),
        (NOSE_X, 0.0105),
        (0.066, 0.0118),
        (0.052, 0.0136),
        (0.038, 0.0152),
        (0.026, 0.0162),
        (0.014, 0.0166),
        (0.009, 0.0150),
    ]
    pts = [(x, s * y) for x, y in profile]
    jaw = cq.Workplane("XY").polyline(pts).close().extrude(HALF_T, both=True)

    for i in range(7):
        xi = 0.046 + 0.004 * i
        groove = (
            cq.Workplane("XY")
            .box(0.0014, 0.0024, 0.020)
            .translate((xi, s * JAW_FACE, 0.0))
        )
        jaw = jaw.cut(groove)

    recess = (
        cq.Workplane("XY")
        .circle(0.0045)
        .extrude(0.011, both=True)
        .translate((0.030, s * JAW_FACE, 0.0))
    )
    jaw = jaw.cut(recess)

    for ang_deg in (40.0, 75.0, 105.0, 140.0):
        a = math.radians(ang_deg)
        sx = 0.030 + 0.0045 * math.cos(a)
        sy = s * (JAW_FACE + 0.0045 * math.sin(a))
        scallop = (
            cq.Workplane("XY")
            .circle(0.0008)
            .extrude(0.011, both=True)
            .translate((sx, sy, 0.0))
        )
        jaw = jaw.cut(scallop)

    notch_pts = [(0.015, s * -0.001), (0.0215, s * -0.001), (0.0185, s * 0.0035)]
    notch = cq.Workplane("XY").polyline(notch_pts).close().extrude(0.011, both=True)
    jaw = jaw.cut(notch)

    return jaw.cut(_lap_cut(s))


def _hub_solid(s: int) -> cq.Workplane:
    """Forged circular hub around the pivot, thinned to its lap layer."""
    hub = cq.Workplane("XY").circle(HUB_R).extrude(HALF_T, both=True)
    return hub.cut(_lap_cut(s))


def _channel_boss_solid() -> cq.Workplane:
    """Channel-lock boss housing plate on the upper face of the pivot.

    A raised rounded rectangular plate sitting on top of the upper half's
    hub, with a visible tongue slot cut through it. Positioned entirely
    above the standard ±HALF_T range so it does not overlap the lower half.
    """
    boss = cq.Workplane("XY").box(BOSS_X, BOSS_Y, BOSS_PLATE_T)
    boss = boss.edges("|Z").fillet(BOSS_FILLET)
    # Visible tongue slot
    slot_x = BOSS_X - 0.008
    slot_y = 0.014
    slot = (
        cq.Workplane("XY")
        .box(slot_x, slot_y, BOSS_PLATE_T + 0.004)
        .translate((-0.002, 0.0, 0.0))
    )
    boss = boss.cut(slot)
    # Translate to sit on top of the upper half's plate
    return boss.translate((0.0, 0.0, HALF_T + BOSS_PLATE_T / 2.0))


def _shank_solid(s: int) -> cq.Workplane:
    """Steel handle tang sweeping back from the hub into the grip."""
    pts = [(x, s * y) for x, y in TANG_PTS]
    loop = _strip_loop(pts, TANG_HALF_W)
    shank = cq.Workplane("XY").polyline(loop).close().extrude(HALF_T, both=True)
    return shank.cut(_lap_cut(s))


def _groove_inset_solid() -> cq.Workplane:
    """One groove channel inset – shared geometry helper for groove visuals.

    Slightly taller than the groove cut so the inset protrudes past the
    hub/shank outer face, ensuring mesh connectivity.
    """
    return (
        cq.Workplane("XY")
        .box(GROOVE_W - 0.0002, GROOVE_Y_SPAN - 0.0004, GROOVE_Z_DEPTH + 0.001)
    )


def _grooved_hub_solid(s: int) -> cq.Workplane:
    """Hub with groove channel cutouts for the lower (grooved) half."""
    hub = cq.Workplane("XY").circle(HUB_R).extrude(HALF_T, both=True)
    hub = hub.cut(_lap_cut(s))
    # Groove cuts on the outer face of the lap layer
    z_face = s * (HALF_T - GROOVE_Z_DEPTH / 2.0)
    for i in range(GROOVE_COUNT):
        x_pos = -i * GROOVE_SPACING
        y_c = s * _interp(x_pos, TANG_PTS)
        cut = (
            cq.Workplane("XY")
            .box(GROOVE_W + 0.0004, GROOVE_Y_SPAN + 0.0004, GROOVE_Z_DEPTH + 0.0004)
            .translate((x_pos, y_c, z_face))
        )
        hub = hub.cut(cut)
    return hub


def _ellipse_wire(x: float, yc: float, zc: float, w: float, h: float) -> cq.Wire:
    wp = cq.Workplane("YZ", origin=(x, 0.0, 0.0)).center(yc, zc).ellipse(w, h)
    return wp.val()


def _grip_solid(s: int) -> cq.Solid:
    wires = [_ellipse_wire(x, s * _yc(x), 0.0, w, h) for x, w, h in GRIP_SECTIONS]
    return cq.Solid.makeLoft(wires, ruled=False)


def _inlay_solid(s: int) -> cq.Solid:
    wires = [
        _ellipse_wire(x, s * _yc(x), INLAY_Z_CENTER, _grip_w(x) - 0.0012, INLAY_HALF_H)
        for x in INLAY_XS
    ]
    return cq.Solid.makeLoft(wires, ruled=False)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="channel_lock_pliers")

    steel_polished = model.material("steel_polished", rgba=(0.80, 0.81, 0.84, 1.0))
    steel_forged = model.material("steel_forged", rgba=(0.66, 0.67, 0.70, 1.0))
    steel_brushed = model.material("steel_brushed", rgba=(0.61, 0.62, 0.65, 1.0))
    seam_gray = model.material("seam_gray", rgba=(0.45, 0.46, 0.48, 1.0))
    groove_dark = model.material("groove_dark", rgba=(0.35, 0.36, 0.38, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.08, 0.09, 1.0))
    grip_red = model.material("grip_red", rgba=(0.80, 0.09, 0.11, 1.0))

    # ---- upper_half (root, s=1): jaw at +Y, handle at -Y ----
    upper = model.part("upper_half")
    upper.visual(
        mesh_from_cadquery(_jaw_solid(1), "upper_jaw", tolerance=0.0002),
        name="jaw",
        material=steel_polished,
    )
    upper.visual(
        mesh_from_cadquery(_hub_solid(1), "upper_hub", tolerance=0.0002),
        name="hub",
        material=steel_forged,
    )
    upper.visual(
        mesh_from_cadquery(_channel_boss_solid(), "channel_boss", tolerance=0.0002),
        name="boss",
        material=steel_forged,
    )
    upper.visual(
        mesh_from_cadquery(_shank_solid(1), "upper_shank", tolerance=0.0002),
        name="shank",
        material=steel_forged,
    )
    upper.visual(
        mesh_from_cadquery(_grip_solid(1), "upper_grip", tolerance=0.0002),
        name="grip",
        material=rubber_black,
    )
    upper.visual(
        mesh_from_cadquery(_inlay_solid(1), "upper_inlay", tolerance=0.0002),
        name="grip_inlay",
        material=grip_red,
    )

    # ---- slide_carriage (intermediate, child of upper_half via PRISMATIC) ----
    carriage = model.part("slide_carriage")
    # Pivot pin spanning from below the lower hub to above the boss plate
    pin_z_center = BOSS_PLATE_T / 2.0
    carriage.visual(
        Cylinder(PIN_R, PIN_H),
        origin=Origin(xyz=(0.0, 0.0, pin_z_center)),
        name="pivot_pin",
        material=steel_brushed,
    )
    # Upper collar sits on top of the boss plate, overlapping the pin top
    carriage.visual(
        Cylinder(COLLAR_R, COLLAR_H),
        origin=Origin(
            xyz=(0.0, 0.0, HALF_T + BOSS_PLATE_T + COLLAR_H / 2.0)
        ),
        name="pivot_collar",
        material=steel_brushed,
    )
    # Lower collar sits below the lower hub, overlapping the pin bottom
    carriage.visual(
        Cylinder(COLLAR_R, COLLAR_H),
        origin=Origin(
            xyz=(0.0, 0.0, -(HALF_T + COLLAR_H / 2.0))
        ),
        name="pivot_collar_lower",
        material=seam_gray,
    )

    # ---- lower_half (s=-1): jaw at -Y, handle at +Y ----
    lower = model.part("lower_half")
    lower.visual(
        mesh_from_cadquery(_jaw_solid(-1), "lower_jaw", tolerance=0.0002),
        name="jaw",
        material=steel_polished,
    )
    lower.visual(
        mesh_from_cadquery(_grooved_hub_solid(-1), "lower_hub", tolerance=0.0002),
        name="hub",
        material=steel_forged,
    )
    lower.visual(
        mesh_from_cadquery(_shank_solid(-1), "lower_shank", tolerance=0.0002),
        name="shank",
        material=steel_forged,
    )
    lower.visual(
        mesh_from_cadquery(_grip_solid(-1), "lower_grip", tolerance=0.0002),
        name="grip",
        material=rubber_black,
    )
    lower.visual(
        mesh_from_cadquery(_inlay_solid(-1), "lower_inlay", tolerance=0.0002),
        name="grip_inlay",
        material=grip_red,
    )

    # Groove visuals: repeated sub-parts via for loop with shared geometry helper.
    # Each groove_i is a dark inset filling the channel cut in the shank.
    # Lower half uses lower lap (z <= -EPS), so the outer face is at -Z.
    groove_z = -(HALF_T - GROOVE_Z_DEPTH / 2.0)
    for i in range(GROOVE_COUNT):
        x_pos = -i * GROOVE_SPACING
        y_c = (-1) * _interp(x_pos, TANG_PTS)
        lower.visual(
            mesh_from_cadquery(
                _groove_inset_solid(), f"groove_geom_{i}", tolerance=0.0002
            ),
            origin=Origin(xyz=(x_pos, y_c, groove_z)),
            name=f"groove_{i}",
            material=groove_dark,
        )

    # ---- Articulations ----

    # PRISMATIC: upper_half → slide_carriage
    # Positive q slides carriage in +X (toward jaw), engaging successive grooves.
    model.articulation(
        "groove_slide",
        ArticulationType.PRISMATIC,
        parent=upper,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.1,
            lower=0.0,
            upper=SLIDE_UPPER,
        ),
    )

    # REVOLUTE: slide_carriage → lower_half
    # Positive q (about -Z) swings lower jaw toward -Y, opening the jaws.
    model.articulation(
        "jaw_pivot",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=lower,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=3.0,
            lower=0.0,
            upper=OPEN_LIMIT,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    upper = object_model.get_part("upper_half")
    carriage = object_model.get_part("slide_carriage")
    lower = object_model.get_part("lower_half")
    slide = object_model.get_articulation("groove_slide")
    pivot = object_model.get_articulation("jaw_pivot")

    upper_jaw = upper.get_visual("jaw")
    lower_jaw = lower.get_visual("jaw")
    upper_hub = upper.get_visual("hub")
    lower_hub = lower.get_visual("hub")

    # ---- Joint contract ----
    slide_limits = slide.motion_limits
    ctx.check(
        "groove_slide is a prismatic joint with 0..0.016 m travel",
        slide.articulation_type == ArticulationType.PRISMATIC
        and slide_limits is not None
        and slide_limits.lower is not None
        and slide_limits.upper is not None
        and abs(slide_limits.lower) < 1e-9
        and abs(slide_limits.upper - SLIDE_UPPER) < 1e-6,
        details=f"limits={slide_limits}",
    )

    pivot_limits = pivot.motion_limits
    ctx.check(
        "jaw_pivot is a 0..30 degree revolute joint",
        pivot.articulation_type == ArticulationType.REVOLUTE
        and pivot_limits is not None
        and pivot_limits.lower is not None
        and pivot_limits.upper is not None
        and abs(pivot_limits.lower) < 1e-9
        and abs(pivot_limits.upper - OPEN_LIMIT) < 1e-6,
        details=f"limits={pivot_limits}",
    )

    # ---- Groove count and spacing ----
    groove_visuals = [lower.get_visual(f"groove_{i}") for i in range(GROOVE_COUNT)]
    ctx.check(
        f"lower shank has {GROOVE_COUNT} groove visuals",
        all(g is not None for g in groove_visuals),
        details="missing groove visual(s)",
    )

    # Grooves evenly spaced at GROOVE_SPACING intervals along the shank.
    if all(g is not None for g in groove_visuals):
        groove_xs = [g.origin.xyz[0] for g in groove_visuals]
        spacings = [groove_xs[i + 1] - groove_xs[i] for i in range(len(groove_xs) - 1)]
        ctx.check(
            "grooves evenly spaced at 8 mm intervals",
            all(abs(sp - (-GROOVE_SPACING)) < 1e-6 for sp in spacings),
            details=f"groove_xs={groove_xs}, spacings={spacings}",
        )

    # ---- Hub lap stacking ----
    ctx.expect_gap(
        upper,
        lower,
        axis="z",
        positive_elem=upper_hub,
        negative_elem=lower_hub,
        min_gap=0.0,
        max_gap=0.001,
        name="upper hub lap stacks above lower hub lap",
    )
    ctx.expect_contact(
        upper,
        lower,
        elem_a=upper_hub,
        elem_b=lower_hub,
        contact_tol=0.0005,
        name="hub laps seat against each other at the joint",
    )
    ctx.expect_overlap(
        upper,
        lower,
        axes="xy",
        elem_a=upper_hub,
        elem_b=lower_hub,
        min_overlap=0.02,
        name="hub laps share the pivot footprint",
    )

    # ---- Carriage pivot pin ----
    # The pivot pin passes through the boss and hubs, capturing the assembly.
    ctx.allow_overlap(
        carriage,
        lower,
        elem_a="pivot_pin",
        elem_b=lower_hub,
        reason="The pivot pin intentionally passes through the lower hub lap, "
        "capturing it at the tongue-and-groove engagement.",
    )
    ctx.expect_within(
        carriage,
        lower,
        axes="xy",
        inner_elem="pivot_pin",
        outer_elem=lower_hub,
        margin=0.001,
        name="pivot pin stays centered inside the lower hub",
    )

    ctx.allow_overlap(
        carriage,
        upper,
        elem_a="pivot_pin",
        elem_b=upper_hub,
        reason="The pivot pin passes through the upper hub lap as part of "
        "the tongue-and-groove pivot capture.",
    )
    ctx.allow_overlap(
        carriage,
        upper,
        elem_a="pivot_pin",
        elem_b="boss",
        reason="The pivot pin passes through the boss plate on the upper "
        "half as part of the tongue-and-groove engagement.",
    )
    ctx.allow_overlap(
        carriage,
        lower,
        elem_a="pivot_collar_lower",
        elem_b=lower_hub,
        reason="The lower pivot collar seats against the lower hub lap, "
        "capturing the assembly at the tongue-and-groove pivot.",
    )

    # ---- Channel boss visible on the upper half ----
    boss_aabb = ctx.part_element_world_aabb(upper, elem="boss")
    if boss_aabb is not None:
        b_min, b_max = boss_aabb
        boss_dx = b_max[0] - b_min[0]
        boss_dy = b_max[1] - b_min[1]
        boss_dz = b_max[2] - b_min[2]
        ctx.check(
            "channel boss is a substantial visible feature at the pivot",
            boss_dx >= 0.025 and boss_dy >= 0.020 and boss_dz >= 0.002,
            details=f"boss_size=({boss_dx:.4f}, {boss_dy:.4f}, {boss_dz:.4f})",
        )
        # Boss sits on top of the upper half plate
        ctx.check(
            "boss plate sits above the standard plate thickness",
            b_min[2] >= HALF_T - 0.001,
            details=f"boss_min_z={b_min[2]:.4f}",
        )
    else:
        ctx.fail("boss AABB resolves", "missing boss element AABB")

    # ---- Closed rest pose: jaws nearly touching ----
    ctx.expect_gap(
        upper,
        lower,
        axis="y",
        positive_elem=upper_jaw,
        negative_elem=lower_jaw,
        min_gap=0.0002,
        max_gap=0.002,
        name="jaws closed and nearly touching at rest",
    )

    # ---- Envelope: ~0.20 m long, ~0.07 m across the open handles ----
    a_upper = ctx.part_world_aabb(upper)
    a_lower = ctx.part_world_aabb(lower)
    g_upper = ctx.part_element_world_aabb(upper, elem="grip")
    g_lower = ctx.part_element_world_aabb(lower, elem="grip")
    ok_env = (
        a_upper is not None
        and a_lower is not None
        and g_upper is not None
        and g_lower is not None
    )
    if ok_env:
        length = max(a_upper[1][0], a_lower[1][0]) - min(a_upper[0][0], a_lower[0][0])
        # Upper grip at -Y, lower grip at +Y; span is the full Y extent.
        across = max(g_upper[1][1], g_lower[1][1]) - min(g_upper[0][1], g_lower[0][1])
        ctx.check(
            "overall length about 0.20 m",
            0.19 <= length <= 0.215,
            details=f"length={length:.4f}",
        )
        ctx.check(
            "handle grips span about 0.07 m across",
            0.060 <= across <= 0.082,
            details=f"across={across:.4f}",
        )
    else:
        ctx.fail("envelope AABBs resolve", "missing part/grip AABB")

    # ---- Prismatic slide moves the carriage ----
    rest_carriage = ctx.part_world_position(carriage)
    with ctx.pose({slide: SLIDE_UPPER}):
        extended_carriage = ctx.part_world_position(carriage)
    ctx.check(
        "prismatic slide moves the carriage along +X",
        rest_carriage is not None
        and extended_carriage is not None
        and extended_carriage[0] > rest_carriage[0] + 0.010,
        details=f"rest={rest_carriage}, extended={extended_carriage}",
    )

    # ---- Decisive open pose: jaws separate and handles spread ----
    closed_jaw_lower = ctx.part_element_world_aabb(lower, elem="jaw")
    closed_grip_lower = ctx.part_element_world_aabb(lower, elem="grip")
    with ctx.pose({pivot: OPEN_LIMIT}):
        ctx.expect_gap(
            upper,
            lower,
            axis="y",
            positive_elem=upper_jaw,
            negative_elem=lower_jaw,
            min_gap=0.004,
            name="jaws open apart at the 30 degree pose",
        )
        open_jaw_lower = ctx.part_element_world_aabb(lower, elem="jaw")
        open_grip_lower = ctx.part_element_world_aabb(lower, elem="grip")
        ctx.check(
            "lower jaw swings away from the upper jaw",
            closed_jaw_lower is not None
            and open_jaw_lower is not None
            and open_jaw_lower[0][1] < closed_jaw_lower[0][1] - 0.018,
            details=f"closed_min_y={closed_jaw_lower}, open_min_y={open_jaw_lower}",
        )
        ctx.check(
            "lower handle spreads outward as the jaws open",
            closed_grip_lower is not None
            and open_grip_lower is not None
            and open_grip_lower[1][1] > closed_grip_lower[1][1] + 0.03,
            details=f"closed_max_y={closed_grip_lower}, open_max_y={open_grip_lower}",
        )

    # ---- Red inlay proud on grip face ----
    i0 = ctx.part_element_world_aabb(upper, elem="grip_inlay")
    if i0 is not None and g_upper is not None:
        ctx.check(
            "red inlay proud on the upper grip top face",
            i0[1][2] >= g_upper[1][2] - 0.0005,
            details=f"inlay_top={i0[1][2]:.4f} grip_top={g_upper[1][2]:.4f}",
        )
    else:
        ctx.fail("inlay AABB resolves", "missing grip_inlay element AABB")

    return ctx.report()


object_model = build_object_model()
