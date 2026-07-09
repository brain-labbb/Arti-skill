from __future__ import annotations

"""Long slim V-groove jaw blacksmith tongs.

Two identical forged halves cross at a flattened elliptical boss and are
joined by a round-head rivet.  The tool lies flat in the XY plane:

- +X runs from the boss toward the jaws, -X along the reins.
- Z is perpendicular to the tool plane (the rivet/pivot axis).

Each half is forged from one bar: a short V-groove jaw bit, a half-lapped
elliptical boss, and a very long tapering rein (square near the pivot,
round at the tip, with a subtle outward bow).  Each jaw bit carries a
prominent longitudinal V-channel cut into its gripping face so that when
the two jaws close they form a diamond / V socket that grips round bar
stock along its axis.  The moving arm is the same forging flipped 180 deg
about the X axis, exactly like a real pair of tongs, so the two boss
halves stack in Z and the jaw/rein swap sides.

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
JAW_TIP_X = 0.086  # jaw tip
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

# Jaw plan outline for the fixed (+Y) half.  The inner face converges
# toward the centerline and chamfers back up at the very tip so the two
# closed jaws form a narrow V notch for gripping bar stock.
JAW_PTS = [
    (0.010, 0.0150),
    (0.050, 0.0100),
    (0.086, 0.0058),
    (0.086, 0.0036),
    (0.073, 0.0008),
    (0.040, 0.0016),
    (0.010, 0.0045),
]

# Longitudinal V-groove cut into the inner gripping face of each jaw.
# The V opens toward the centerline (-Y for the fixed arm); when both jaws
# close the matched grooves form a diamond socket for round bar stock.
V_GROOVE_DEPTH = 0.0020   # how far the valley sits behind the jaw face (Y)
V_GROOVE_HALF_Z = 0.0035  # half-opening of the V at the face surface (Z)
# Stations along the inner face where the V-groove profile is sampled.
# (x, face_y) follows the inner contour of JAW_PTS.
V_GROOVE_STATIONS = [
    (0.015, 0.00402),
    (0.030, 0.00257),
    (0.045, 0.00148),
    (0.060, 0.00112),
    (0.073, 0.00080),
    (0.082, 0.00274),
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


def _vgroove_cutter() -> cq.Workplane:
    """Longitudinal V-channel that runs along the jaw inner gripping face.

    The cross-section at each station is a triangle in the YZ plane whose
    two lips sit on the jaw face and whose valley penetrates into the jaw
    body.  Lofting the triangles along X produces a smooth V-channel that
    follows the jaw contour.  The cutter is made slightly oversized so the
    boolean cut is clean at the face boundary.
    """
    # Extend the V slightly past the face so the boolean slices cleanly.
    lip_overshoot = 0.0004
    wires: list[cq.Wire] = []
    for x, fy in V_GROOVE_STATIONS:
        # Triangle in YZ: two lips on (or just past) the face, one valley
        # inside the jaw body.  For the unflipped (fixed, +Y) half the V
        # opens toward -Y; the valley is at face_y + depth.
        pts = [
            cq.Vector(x, fy - lip_overshoot, -V_GROOVE_HALF_Z),
            cq.Vector(x, fy + V_GROOVE_DEPTH, 0.0),
            cq.Vector(x, fy - lip_overshoot, +V_GROOVE_HALF_Z),
        ]
        pts.append(pts[0])
        wires.append(cq.Wire.makePolygon(pts))
    return cq.Workplane(obj=cq.Solid.makeLoft(wires))


def _jaw_solid() -> cq.Workplane:
    jaw = cq.Workplane("XY").polyline(JAW_PTS).close().extrude(JAW_T / 2.0, both=True)
    # Longitudinal V-groove along the inner gripping face.
    jaw = jaw.cut(_vgroove_cutter())
    # Hammer marks on both flat faces of the jaw.
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
    model = ArticulatedObject(name="vgroove_jaw_blacksmith_tongs")

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

    # Closed V-groove jaws: the V-lips nearly meet, forming a diamond socket.
    ctx.expect_gap(
        fixed_arm,
        moving_arm,
        axis="y",
        positive_elem="jaw",
        negative_elem="jaw",
        min_gap=-0.001,
        max_gap=0.005,
        name="closed V-groove jaw lips nearly meet at the centerline",
    )
    ctx.expect_overlap(
        fixed_arm,
        moving_arm,
        axes="x",
        elem_a="jaw",
        elem_b="jaw",
        min_overlap=0.05,
        name="the two V-groove jaws run side by side along the jaw end",
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
    jaw_f = ctx.part_element_world_aabb(fixed_arm, elem="jaw")
    ctx.check(
        "jaw end is short, about 0.07 m",
        jaw_f is not None and 0.055 <= (jaw_f[1][0] - jaw_f[0][0]) <= 0.085,
        details=f"fixed_jaw={jaw_f}",
    )
    # V-groove is cut into the Y face, so the jaw Z-extent should equal JAW_T
    # (the bar thickness); the V-channel is visible on the inner face only.
    ctx.check(
        "jaw Z-extent matches bar thickness (V-groove is on the Y face)",
        jaw_f is not None and abs((jaw_f[1][2] - jaw_f[0][2]) - JAW_T) < 0.002,
        details=f"fixed_jaw={jaw_f}",
    )

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
            min_gap=0.005,
            name="jaws open apart at full pivot travel",
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
