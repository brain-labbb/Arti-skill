from __future__ import annotations

"""Long slim bit-jaw blacksmith tongs.

Two identical forged halves cross at a flattened elliptical boss and are
joined by a round-head rivet.  The tool lies flat in the XY plane:

- +X runs from the boss toward the jaws, -X along the reins.
- Z is perpendicular to the tool plane (the rivet/pivot axis).

Each half is forged from one bar: a concave bit jaw (scoop-shaped with a
cylindrical hollow gripping face), a half-lapped elliptical boss, and a
very long tapering rein (square near the pivot, round at the tip, with a
subtle outward bow).  The moving arm is the same forging flipped 180 deg
about the X axis, exactly like a real pair of tongs, so the two boss halves
stack in Z and the jaw/rein swap sides.

When closed, the two opposed concave half-cylinder bits form a round bore
that cradles round rod or pipe (not a flat or V face).

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

JAW_T = 0.011  # full bar thickness at the jaw (z) — slightly taller than
# the wolf-jaw parent so the bore cylinder is fully contained.
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

# ---------------------------------------------------------------- bit jaw
# Concave bit jaw: the bore radius for gripping round rod / pipe.
# When both jaws close the two opposed half-cylinders form a bore of
# diameter ~2 * BORE_R.
BORE_R = 0.005
# Small Y-offset so the closed jaw edges (above and below the bore
# channel) keep a visible separation instead of touching.
BIT_FACE_GAP = 0.0005

# Bit jaw plan outline for the fixed (+Y) half.
# The inner face extends close to the centreline so the bore cylinder
# carves a visible concave scoop along the full jaw length.
BIT_JAW_PTS = [
    (0.010, 0.0150),
    (0.050, 0.0120),
    (0.086, 0.0080),
    (0.086, BIT_FACE_GAP),
    (0.073, BIT_FACE_GAP),
    (0.040, BIT_FACE_GAP),
    (0.010, BIT_FACE_GAP),
]

# Hammer-mark dimples (x, y, z_face_sign) on boss and jaw faces.
DIMPLE_R = 0.004
DIMPLE_DEPTH = 0.0005
BOSS_DIMPLES = [(0.006, 0.0065), (-0.0075, -0.0045), (-0.0115, 0.0060), (0.0015, -0.0105)]
JAW_DIMPLES_TOP = [(0.024, 0.0100), (0.040, 0.0080), (0.057, 0.0065)]
JAW_DIMPLES_BOT = [(0.031, 0.0090), (0.049, 0.0070)]

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


def _bore_cutter() -> cq.Workplane:
    """Cylinder along X that carves the concave half-cylinder bore into the
    inner face of each jaw.  Centered at (Y=0, Z=0) so the two closed jaws
    form a round bore."""
    return (
        cq.Workplane("YZ", origin=(0.005, 0.0, 0.0))
        .circle(BORE_R)
        .extrude(JAW_TIP_X + 0.002)
    )


def _bit_jaw_solid() -> cq.Workplane:
    """Concave bit jaw: forged outer body with a cylindrical hollow on the
    inner gripping face.  When two of these close they curl around and
    cradle round rod or pipe."""
    jaw = (
        cq.Workplane("XY")
        .polyline(BIT_JAW_PTS)
        .close()
        .extrude(JAW_T / 2.0, both=True)
    )
    # Concave bore: the cylinder cut carves a half-cylinder scoop on the
    # inner face of the jaw.
    jaw = jaw.cut(_bore_cutter())
    # Hammer marks on both flat faces of the jaw (outside the bore zone).
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
    half-lap faces the other way).

    The fixed arm (root) needs the Z_LIFT baked into its meshes.  The
    moving arm is a child of the revolute articulation whose origin is
    already at Z_LIFT, so its meshes stay at Z=0 in the child frame."""
    if flipped:
        solid = solid.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 180.0)
        return solid  # child frame; articulation origin provides Z_LIFT
    return solid.translate((0.0, 0.0, Z_LIFT))


# -------------------------------------------------------------------- model
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bit_jaw_blacksmith_tongs")

    steel = model.material("forged_steel_gray", rgba=(0.37, 0.38, 0.40, 1.0))
    jaw_steel = model.material("jaw_scale_steel", rgba=(0.30, 0.31, 0.33, 1.0))
    rivet_steel = model.material("rivet_steel", rgba=(0.25, 0.26, 0.28, 1.0))

    arms = [("fixed_arm", False), ("moving_arm", True)]
    arm_parts: list = []
    for i, (arm_name, flipped) in enumerate(arms):
        arm = model.part(arm_name)
        arm_parts.append(arm)
        arm.visual(
            mesh_from_cadquery(
                _place(_bit_jaw_solid(), flipped=flipped),
                f"jaw_{i}",
                tolerance=0.00015,
            ),
            name="jaw",
            material=jaw_steel,
        )
        arm.visual(
            mesh_from_cadquery(
                _place(_boss_solid(), flipped=flipped),
                f"boss_{i}",
                tolerance=0.00015,
            ),
            name="boss",
            material=steel,
        )
        arm.visual(
            mesh_from_cadquery(
                _place(_rein_solid(), flipped=flipped),
                f"rein_{i}",
                tolerance=0.00015,
            ),
            name="rein",
            material=steel,
        )
        if i == 0:
            arm.visual(
                mesh_from_cadquery(
                    _place(_rivet_solid(), flipped=False),
                    "rivet",
                    tolerance=0.0001,
                ),
                name="rivet",
                material=rivet_steel,
            )

    model.articulation(
        "rivet_pivot",
        ArticulationType.REVOLUTE,
        parent=arm_parts[0],
        child=arm_parts[1],
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

    # --- Concave bit-jaw specific checks ---

    # Closed concave jaws: the bore edges (above and below the channel)
    # keep a small Y-gap rather than touching flat-to-flat.
    ctx.expect_gap(
        fixed_arm,
        moving_arm,
        axis="y",
        positive_elem="jaw",
        negative_elem="jaw",
        min_gap=0.0002,
        max_gap=0.004,
        name="closed concave bit jaws keep a small edge gap (bore channel visible)",
    )

    # The two opposed concave half-cylinders run side by side along X.
    ctx.expect_overlap(
        fixed_arm,
        moving_arm,
        axes="x",
        elem_a="jaw",
        elem_b="jaw",
        min_overlap=0.05,
        name="the two concave bits run side by side along the jaw end",
    )

    # Bore containment: the bore is fully contained within the jaw
    # thickness (bore diameter < jaw Z-extent).
    jaw_f = ctx.part_element_world_aabb(fixed_arm, elem="jaw")
    if jaw_f is not None:
        jaw_z = jaw_f[1][2] - jaw_f[0][2]
        ctx.check(
            "bore diameter fits within the jaw thickness",
            jaw_z > 2.0 * BORE_R + 0.0002,
            details=f"jaw_z={jaw_z:.5f}, bore_d={2.0 * BORE_R:.4f}",
        )
        # The forged body around the bore: jaw Y-extent must be wider
        # than the bore radius alone.
        jaw_y = jaw_f[1][1] - jaw_f[0][1]
        ctx.check(
            "jaw Y-extent shows forged body around the concave bore",
            jaw_y > BORE_R + 0.004,
            details=f"jaw_y={jaw_y:.5f}",
        )

    # --- Retained parent checks (boss, rivet, proportions, reins) ---

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
    ctx.check(
        "jaw end is short, about 0.07 m",
        jaw_f is not None and 0.055 <= (jaw_f[1][0] - jaw_f[0][0]) <= 0.085,
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
    # The minimum Y-gap is near the boss (close to the pivot), so the
    # threshold is set for that location; the jaw tips separate much more.
    with ctx.pose({pivot: 0.3}):
        ctx.expect_gap(
            fixed_arm,
            moving_arm,
            axis="y",
            positive_elem="jaw",
            negative_elem="jaw",
            min_gap=0.003,
            name="concave jaws open apart at full pivot travel",
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
