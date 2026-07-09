from __future__ import annotations

"""Flat box (pickup) jaw blacksmith tongs.

Two identical forged halves cross at a flattened elliptical boss and are
joined by a round-head rivet.  The tool lies flat in the XY plane:

- +X runs from the boss toward the jaws, -X along the reins.
- Z is perpendicular to the tool plane (the rivet/pivot axis).

Each half is forged from one bar: a short flat box / pickup jaw (a squared
rectangular bit with parallel flat gripping faces for clamping flat plate
or bar stock), a half-lapped elliptical boss, and a very long tapering rein
(square near the pivot, round at the tip, with a subtle outward bow).  The
moving arm is the same forging flipped 180 deg about the X axis, exactly
like a real pair of tongs, so the two boss halves stack in Z and the
jaw/rein swap sides.

Articulation: one revolute joint at the rivet, axis perpendicular to the
tool plane.  Positive q (0..0.3 rad) opens the jaws while the moving rein
swings away from the fixed rein.
"""

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

# ---------------------------------------------------------------- dimensions
# Tool plane is z = Z_LIFT (the tongs lie flat on the ground plane z=0).
Z_LIFT = 0.0085

JAW_T = 0.0096  # full bar thickness at the jaw (z)
JAW_TIP_X = 0.080  # jaw tip (flat box jaw)
BOSS_A = 0.020  # boss ellipse semi-axis along X
BOSS_B = 0.0155  # boss ellipse semi-axis along Y
BOSS_PLATE_T = 0.0048  # one half-lapped boss plate thickness

LAP_R = 0.022  # half-lap clearance radius around the rivet
LAP_EPS = 0.00006  # tiny z clearance between the two lapped halves

REIN_TIP_X = -0.462
REIN_TIP_Y = -0.036  # splayed fixed-rein tip (moving rein mirrors to +Y)
REIN_TIP_R = 0.0035  # ~0.007 m round at the tip

RIVET_SHAFT_R = 0.0045
RIVET_HEAD_R = 0.0060

# Flat box / pickup jaw outline for the fixed (+Y) half.
# A short squared rectangular bit with flat parallel inner and outer faces
# for clamping flat plate or bar stock.  Squared-off corners at the tip.
# The inner (gripping) face sits at y=JAW_INNER_Y so both closed jaws
# present parallel flat faces with a small uniform gap.
JAW_INNER_Y = 0.0010  # flat gripping face, near the centerline
JAW_OUTER_Y = 0.0150  # outer face of the jaw bit
JAW_START_X = 0.010   # rear of the jaw bit (boss transition)
JAW_TIP_X_BOX = 0.080 # squared tip of the jaw bit
JAW_BOX_PTS = [
    (JAW_START_X, JAW_OUTER_Y),    # rear outer corner
    (JAW_TIP_X_BOX, JAW_OUTER_Y),  # tip outer corner (squared)
    (JAW_TIP_X_BOX, JAW_INNER_Y),  # tip inner corner (squared)
    (JAW_START_X, JAW_INNER_Y),    # rear inner corner
]

# Hammer-mark dimples (x, y, z_face_sign) on boss and jaw faces.
DIMPLE_R = 0.004
DIMPLE_DEPTH = 0.0005
BOSS_DIMPLES = [(0.006, 0.0065), (-0.0075, -0.0045), (-0.0115, 0.0060), (0.0015, -0.0105)]
JAW_DIMPLES_TOP = [(0.024, 0.0085), (0.040, 0.0062), (0.057, 0.0048)]
JAW_DIMPLES_BOT = [(0.031, 0.0072), (0.049, 0.0052)]

# Rein loft stations: (x, y_center, half_w, half_h, superellipse_exponent).
# Square (~0.0155) near the pivot blending continuously to round at the tip,
# with a subtle outward bow (extra -Y bulge mid-length on the fixed half).
REIN_SECTIONS = [
    (-0.014, -0.0090, 0.0070, 0.0047, 5.0),
    (-0.052, -0.0125, 0.00775, 0.00775, 5.0),
    (-0.130, -0.0190, 0.00675, 0.00675, 4.0),
    (-0.240, -0.0268, 0.00575, 0.00575, 3.2),
    (-0.350, -0.0322, 0.00475, 0.00475, 2.6),
    (REIN_TIP_X, REIN_TIP_Y, REIN_TIP_R, REIN_TIP_R, 2.0),
]
REIN_SECTION_PTS = 32


# ----------------------------------------------------------------- builders
def _lap_cutter() -> cq.Workplane:
    """Material below z=LAP_EPS within LAP_R of the rivet is forged away."""
    return (
        cq.Workplane("XY")
        .workplane(offset=-0.040)
        .circle(LAP_R)
        .extrude(0.040 + LAP_EPS)
    )


def _dimple(x: float, y: float, z_face: float, sign: int) -> cq.Workplane:
    cz = z_face + sign * (DIMPLE_R - DIMPLE_DEPTH)
    return cq.Workplane("XY", origin=(x, y, cz)).sphere(DIMPLE_R)


def _jaw_solid() -> cq.Workplane:
    """Flat box / pickup jaw: a short squared rectangular bit with flat
    parallel inner and outer gripping faces for clamping flat plate or bar
    stock.  Squared-off corners at the tip; no V-notch."""
    jaw = cq.Workplane("XY").polyline(JAW_BOX_PTS).close().extrude(JAW_T / 2.0, both=True)
    # Hammer marks on both flat faces of the jaw (Z faces).
    for dx, dy in JAW_DIMPLES_TOP:
        jaw = jaw.cut(_dimple(dx, dy, JAW_T / 2.0, +1))
    for dx, dy in JAW_DIMPLES_BOT:
        jaw = jaw.cut(_dimple(dx, dy, -JAW_T / 2.0, -1))
    return jaw.cut(_lap_cutter())


def _boss_solid() -> cq.Workplane:
    boss = cq.Workplane("XY").ellipse(BOSS_A, BOSS_B).extrude(BOSS_PLATE_T)
    for dx, dy in BOSS_DIMPLES:
        boss = boss.cut(_dimple(dx, dy, BOSS_PLATE_T, +1))
    return boss.cut(_lap_cutter())


def _rein_solid() -> cq.Workplane:
    wires: list[cq.Wire] = []
    for x, yc, hw, hh, n_exp in REIN_SECTIONS:
        pts: list[cq.Vector] = []
        for i in range(REIN_SECTION_PTS):
            t = 2.0 * math.pi * i / REIN_SECTION_PTS
            c, s = math.cos(t), math.sin(t)
            dy = hw * math.copysign(abs(c) ** (2.0 / n_exp), c)
            dz = hh * math.copysign(abs(s) ** (2.0 / n_exp), s)
            pts.append(cq.Vector(x, yc + dy, dz))
        pts.append(pts[0])
        wires.append(cq.Wire.makePolygon(pts))
    rein = cq.Workplane(obj=cq.Solid.makeLoft(wires))
    # Rounded rein tip.
    tip = cq.Workplane("XY", origin=(REIN_TIP_X, REIN_TIP_Y, 0.0)).sphere(REIN_TIP_R)
    return rein.union(tip).cut(_lap_cutter())


def _rivet_solid() -> cq.Workplane:
    shaft = (
        cq.Workplane("XY")
        .workplane(offset=-0.0046)
        .circle(RIVET_SHAFT_R)
        .extrude(0.0092)
    )
    head_top = cq.Workplane("XY", origin=(0.0, 0.0, 0.0022)).sphere(RIVET_HEAD_R)
    head_bot = cq.Workplane("XY", origin=(0.0, 0.0, -0.0022)).sphere(RIVET_HEAD_R)
    return shaft.union(head_top).union(head_bot)


def _place(solid: cq.Workplane, *, flipped: bool) -> cq.Workplane:
    """Lift into the lying-flat pose; the moving half is the identical
    forging flipped 180 deg about the X axis (jaw/rein swap sides, the
    half-lap faces the other way)."""
    if flipped:
        solid = solid.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 180.0)
    return solid.translate((0.0, 0.0, Z_LIFT))


# -------------------------------------------------------------------- model
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flat_box_jaw_blacksmith_tongs")

    steel = model.material("forged_steel_gray", rgba=(0.37, 0.38, 0.40, 1.0))
    jaw_steel = model.material("jaw_scale_steel", rgba=(0.30, 0.31, 0.33, 1.0))
    rivet_steel = model.material("rivet_steel", rgba=(0.25, 0.26, 0.28, 1.0))

    fixed_arm = model.part("fixed_arm")
    fixed_arm.visual(
        mesh_from_cadquery(_place(_jaw_solid(), flipped=False), "fixed_jaw", tolerance=0.00015),
        name="jaw",
        material=jaw_steel,
    )
    fixed_arm.visual(
        mesh_from_cadquery(_place(_boss_solid(), flipped=False), "fixed_boss", tolerance=0.00015),
        name="boss",
        material=steel,
    )
    fixed_arm.visual(
        mesh_from_cadquery(_place(_rein_solid(), flipped=False), "fixed_rein", tolerance=0.00015),
        name="rein",
        material=steel,
    )
    fixed_arm.visual(
        mesh_from_cadquery(_place(_rivet_solid(), flipped=False), "rivet", tolerance=0.0001),
        name="rivet",
        material=rivet_steel,
    )

    moving_arm = model.part("moving_arm")
    moving_arm.visual(
        mesh_from_cadquery(_place(_jaw_solid(), flipped=True), "moving_jaw", tolerance=0.00015),
        name="jaw",
        material=jaw_steel,
    )
    moving_arm.visual(
        mesh_from_cadquery(_place(_boss_solid(), flipped=True), "moving_boss", tolerance=0.00015),
        name="boss",
        material=steel,
    )
    moving_arm.visual(
        mesh_from_cadquery(_place(_rein_solid(), flipped=True), "moving_rein", tolerance=0.00015),
        name="rein",
        material=steel,
    )

    model.articulation(
        "rivet_pivot",
        ArticulationType.REVOLUTE,
        parent=fixed_arm,
        child=moving_arm,
        origin=Origin(xyz=(0.0, 0.0, Z_LIFT)),
        # Axis perpendicular to the tool plane.  The moving jaw sits on -Y
        # and its rein on +Y, so -Z makes positive q swing the jaw away
        # from the fixed jaw (opening) while the rein spreads away from
        # the fixed rein.
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=2.5, lower=0.0, upper=0.3),
    )

    return model


# -------------------------------------------------------------------- tests
def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed_arm = object_model.get_part("fixed_arm")
    moving_arm = object_model.get_part("moving_arm")
    pivot = object_model.get_articulation("rivet_pivot")

    ctx.allow_overlap(
        fixed_arm,
        moving_arm,
        elem_a="rivet",
        elem_b="boss",
        reason="The round-head rivet is the captured pivot pin and intentionally passes through the moving arm boss plate.",
    )

    # Closed flat box jaws present parallel flat gripping faces with a small
    # uniform gap for clamping flat plate or bar stock.
    ctx.expect_gap(
        fixed_arm,
        moving_arm,
        axis="y",
        positive_elem="jaw",
        negative_elem="jaw",
        min_gap=0.001,
        max_gap=0.004,
        name="closed flat box jaws present parallel gripping faces with a small gap",
    )
    ctx.expect_overlap(
        fixed_arm,
        moving_arm,
        axes="x",
        elem_a="jaw",
        elem_b="jaw",
        min_overlap=0.05,
        name="the two flat box jaws run side by side along the jaw end",
    )

    # Flat box jaw has squared-off corners: the jaw Y width (inner-to-outer
    # face distance) is consistent across the full length, confirming the
    # rectangular flat-faced bit shape rather than a converging V-notch.
    jaw_f = ctx.part_element_world_aabb(fixed_arm, elem="jaw")
    if jaw_f is not None:
        jaw_width_y = jaw_f[1][1] - jaw_f[0][1]
        ctx.check(
            "flat box jaw has parallel inner and outer faces",
            0.010 <= jaw_width_y <= 0.018,
            details=f"jaw_width_y={jaw_width_y:.4f}",
        )
        jaw_length_x = jaw_f[1][0] - jaw_f[0][0]
        ctx.check(
            "flat box jaw is a short rectangular bit about 0.07 m long",
            0.055 <= jaw_length_x <= 0.085,
            details=f"jaw_length_x={jaw_length_x:.4f}",
        )

    # Rivet seated at the boss, visible on both faces.
    ctx.expect_within(
        fixed_arm,
        moving_arm,
        axes="xy",
        inner_elem="rivet",
        outer_elem="boss",
        margin=0.0005,
        name="rivet centered inside the boss footprint",
    )
    riv = ctx.part_element_world_aabb(fixed_arm, elem="rivet")
    boss_f = ctx.part_element_world_aabb(fixed_arm, elem="boss")
    boss_m = ctx.part_element_world_aabb(moving_arm, elem="boss")
    ctx.check(
        "rivet head proud of the upper boss face",
        riv is not None and boss_f is not None and riv[1][2] > boss_f[1][2] + 0.002,
        details=f"rivet={riv}, fixed_boss={boss_f}",
    )
    ctx.check(
        "rivet head proud of the lower boss face",
        riv is not None and boss_m is not None and riv[0][2] < boss_m[0][2] - 0.002,
        details=f"rivet={riv}, moving_boss={boss_m}",
    )

    # Boss is a flattened ellipse roughly 0.035 m across.
    ctx.check(
        "boss roughly 0.035 m across",
        boss_f is not None
        and 0.028 <= (boss_f[1][0] - boss_f[0][0]) <= 0.045
        and 0.026 <= (boss_f[1][1] - boss_f[0][1]) <= 0.040,
        details=f"fixed_boss={boss_f}",
    )

    # Overall proportions: ~0.55 m long, lying flat in one plane.
    fa = ctx.part_world_aabb(fixed_arm)
    ma = ctx.part_world_aabb(moving_arm)
    assert fa is not None and ma is not None
    length = max(fa[1][0], ma[1][0]) - min(fa[0][0], ma[0][0])
    ctx.check("overall length about 0.55 m", 0.50 <= length <= 0.60, details=f"length={length:.4f}")
    z_extent = max(fa[1][2], ma[1][2]) - min(fa[0][2], ma[0][2])
    ctx.check("tool lies flat in one plane", z_extent <= 0.030, details=f"z_extent={z_extent:.4f}")
    ctx.check(
        "tongs rest just above the ground plane",
        min(fa[0][2], ma[0][2]) >= -0.0005,
        details=f"min_z={min(fa[0][2], ma[0][2]):.5f}",
    )

    # Reins: very long (~0.45 m) and gently splayed; tips ~0.007 m round.
    rein_f = ctx.part_element_world_aabb(fixed_arm, elem="rein")
    rein_m = ctx.part_element_world_aabb(moving_arm, elem="rein")
    ctx.check(
        "reins run about 0.45 m from the boss",
        rein_f is not None and 0.40 <= (rein_f[1][0] - rein_f[0][0]) <= 0.50,
        details=f"fixed_rein={rein_f}",
    )
    ctx.check(
        "rein tips splay to opposite sides",
        rein_f is not None
        and rein_m is not None
        and rein_f[0][1] < -0.030
        and rein_m[1][1] > 0.030,
        details=f"fixed_rein={rein_f}, moving_rein={rein_m}",
    )
    # (jaw end length and parallel-face checks are above in the flat box jaw section)

    # Joint configuration matches the prompt: 0..0.3 rad about the rivet.
    limits = pivot.motion_limits
    ctx.check(
        "pivot limits are 0 to 0.3 rad",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower) < 1e-9
        and abs(limits.upper - 0.3) < 1e-6,
        details=f"limits={limits}",
    )

    rein_m_rest_max_y = rein_m[1][1] if rein_m is not None else None

    # Decisive open pose: jaws open while the moving rein swings away.
    with ctx.pose({pivot: 0.3}):
        ctx.expect_gap(
            fixed_arm,
            moving_arm,
            axis="y",
            positive_elem="jaw",
            negative_elem="jaw",
            min_gap=0.004,
            name="flat box jaws open apart at full pivot travel",
        )
        rein_m_open = ctx.part_element_world_aabb(moving_arm, elem="rein")
        ctx.check(
            "moving rein swings away from the fixed rein",
            rein_m_open is not None
            and rein_m_rest_max_y is not None
            and rein_m_open[1][1] > rein_m_rest_max_y + 0.08,
            details=f"rest_max_y={rein_m_rest_max_y}, open={rein_m_open}",
        )

    return ctx.report()


object_model = build_object_model()
