from __future__ import annotations

"""Long slim wolf-jaw blacksmith tongs.

Two identical forged halves cross at a flattened elliptical boss and are
joined by a round-head rivet.  The tool lies flat in the XY plane:

- +X runs from the boss toward the jaws, -X along the reins.
- Z is perpendicular to the tool plane (the rivet/pivot axis).

Each half is forged from one bar: a short flat wolf jaw, a half-lapped
elliptical boss, and a very long tapering rein (square near the pivot,
round at the tip, with a subtle outward bow).  The moving arm is the same
forging flipped 180 deg about the X axis, exactly like a real pair of
tongs, so the two boss halves stack in Z and the jaw/rein swap sides.

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
Z_LIFT = 0.014

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

# Raised cylindrical boss collar (forged hub / washer ring on outer boss face).
COLLAR_R = 0.0090  # collar outer radius
COLLAR_H = 0.0030  # collar height above the boss plate
COLLAR_BORE_R = RIVET_SHAFT_R + 0.0003  # bore clearance for the rivet shaft
RIVET_HEAD_Z = BOSS_PLATE_T + COLLAR_H  # head center sits at collar outer face

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

# Shallow transverse grooves on the jaw gripping face (x, face_y).
GROOVE_R = 0.0014
GROOVE_SINK = 0.0008  # center offset behind the face -> ~0.6 mm deep groove
GROOVES = [
    (0.047, 0.0016 - 0.0008 * (0.047 - 0.040) / 0.033),
    (0.056, 0.0016 - 0.0008 * (0.056 - 0.040) / 0.033),
    (0.065, 0.0016 - 0.0008 * (0.065 - 0.040) / 0.033),
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
    jaw = cq.Workplane("XY").polyline(JAW_PTS).close().extrude(JAW_T / 2.0, both=True)
    # Shallow transverse grooves across the gripping face (vertical, axis Z).
    for gx, face_y in GROOVES:
        groove = (
            cq.Workplane("XY")
            .center(gx, face_y + GROOVE_SINK)
            .circle(GROOVE_R)
            .extrude(0.02, both=True)
        )
        jaw = jaw.cut(groove)
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
    boss = boss.cut(_lap_cutter())
    # Raised cylindrical collar on the outer boss face: a short forged hub
    # encircling the rivet with a bore for the shank to pass through.
    collar = (
        cq.Workplane("XY")
        .workplane(offset=BOSS_PLATE_T)
        .circle(COLLAR_R)
        .circle(COLLAR_BORE_R)
        .extrude(COLLAR_H)
    )
    return boss.union(collar)


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
    # Shaft spans through both boss plates and both raised collars.
    shaft_half = RIVET_HEAD_Z + 0.0002
    shaft = (
        cq.Workplane("XY")
        .workplane(offset=-shaft_half)
        .circle(RIVET_SHAFT_R)
        .extrude(2.0 * shaft_half)
    )
    # Round heads seated at each collar outer face.
    head_top = cq.Workplane("XY", origin=(0.0, 0.0, RIVET_HEAD_Z)).sphere(RIVET_HEAD_R)
    head_bot = cq.Workplane("XY", origin=(0.0, 0.0, -RIVET_HEAD_Z)).sphere(RIVET_HEAD_R)
    return shaft.union(head_top).union(head_bot)


def _place(solid: cq.Workplane, *, flipped: bool) -> cq.Workplane:
    """Lift into the lying-flat pose; the moving half is the identical
    forging flipped 180 deg about the X axis (jaw/rein swap sides, the
    half-lap faces the other way)."""
    s = solid.val()
    if flipped:
        # Rotate 180° about global X axis using Location.
        # The Workplane wrapper lifts the result to Z_LIFT.
        rot_loc = cq.Location(cq.Vector(0, 0, 0), cq.Vector(1, 0, 0), 180)
        s = s.moved(rot_loc)
    else:
        trans_loc = cq.Location(cq.Vector(0, 0, Z_LIFT))
        s = s.moved(trans_loc)
    return cq.Workplane("XY").newObject([s])


# -------------------------------------------------------------------- model
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wolf_jaw_blacksmith_tongs")

    steel = model.material("forged_steel_gray", rgba=(0.37, 0.38, 0.40, 1.0))
    jaw_steel = model.material("jaw_scale_steel", rgba=(0.30, 0.31, 0.33, 1.0))
    rivet_steel = model.material("rivet_steel", rgba=(0.25, 0.26, 0.28, 1.0))

    # Build both arms via a shared geometry helper and regular placement.
    # The moving half is the identical forging flipped 180 deg about X,
    # so the boss collar appears on the outer face of each arm.
    arm_parts: dict[str, object] = {}
    for i, (arm_name, is_flipped) in enumerate([
        ("fixed_arm", False),
        ("moving_arm", True),
    ]):
        arm = model.part(arm_name)
        arm_parts[arm_name] = arm
        arm.visual(
            mesh_from_cadquery(_place(_jaw_solid(), flipped=is_flipped), f"jaw_{i}", tolerance=0.00015),
            name="jaw",
            material=jaw_steel,
        )
        arm.visual(
            mesh_from_cadquery(_place(_boss_solid(), flipped=is_flipped), f"boss_{i}", tolerance=0.00015),
            name="boss",
            material=steel,
        )
        arm.visual(
            mesh_from_cadquery(_place(_rein_solid(), flipped=is_flipped), f"rein_{i}", tolerance=0.00015),
            name="rein",
            material=steel,
        )
        # The rivet is carried by the fixed arm only; its shank passes
        # through both boss collars as the captured pivot pin.
        if not is_flipped:
            arm.visual(
                mesh_from_cadquery(_place(_rivet_solid(), flipped=False), "rivet", tolerance=0.0001),
                name="rivet",
                material=rivet_steel,
            )

    fixed_arm = arm_parts["fixed_arm"]
    moving_arm = arm_parts["moving_arm"]

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
        reason="The round-head rivet is the captured pivot pin passing through both raised boss collars and the half-lapped boss plates.",
    )

    # Closed jaws angle toward each other and nearly meet near the tip.
    ctx.expect_gap(
        fixed_arm,
        moving_arm,
        axis="y",
        positive_elem="jaw",
        negative_elem="jaw",
        min_gap=0.0005,
        max_gap=0.004,
        name="closed jaws nearly meet near the tip",
    )
    ctx.expect_overlap(
        fixed_arm,
        moving_arm,
        axes="x",
        elem_a="jaw",
        elem_b="jaw",
        min_overlap=0.05,
        name="the two flat jaws run side by side along the jaw end",
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

    # Raised cylindrical collar on each boss outer face.
    boss_f_z = (boss_f[1][2] - boss_f[0][2]) if boss_f else 0.0
    boss_m_z = (boss_m[1][2] - boss_m[0][2]) if boss_m else 0.0
    ctx.check(
        "fixed boss has raised collar taller than plate alone",
        boss_f is not None and boss_f_z > 0.006,
        details=f"fixed_boss_z_extent={boss_f_z:.4f}",
    )
    ctx.check(
        "moving boss has raised collar taller than plate alone",
        boss_m is not None and boss_m_z > 0.006,
        details=f"moving_boss_z_extent={boss_m_z:.4f}",
    )
    ctx.check(
        "both boss collars are symmetric in height",
        boss_f is not None
        and boss_m is not None
        and abs(boss_f_z - boss_m_z) < 0.001,
        details=f"fixed_z={boss_f_z:.4f}, moving_z={boss_m_z:.4f}",
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
