from __future__ import annotations

# Insulated lineman pliers (VDE-style variant).
# Fork of the heavy-duty combination (lineman) pliers with:
# - thick layered insulated grip sleeves (inner yellow + outer orange)
# - locking release lever pivoting inside one handle
# - circular pivot rivet cap on both sides
#
# Layout: the tool lies in the XY plane with Z as the thickness axis.
# The rivet pivot is at the origin. The blunt squared nose points +X,
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
HALF_T = 0.009          # half thickness of each forged plate (full plate 0.018)
LAP_R = 0.016           # half-lap joint disc radius at the pivot
HUB_R = 0.015           # forged hub radius around the rivet
BOSS_R = 0.0125         # rivet boss radius (~0.025 m diameter)
BOSS_H = 0.0030
SEAM_R = 0.0132         # visible circular seam ring under the boss cap
SEAM_H = 0.0006
RIVET_R = 0.004         # rivet shaft captured through the moving half's lap
EPS = 0.0001            # lap clearance so the stacked halves do not penetrate
JAW_FACE = 0.0003       # closed jaw inner faces sit at y = +/-JAW_FACE
NOSE_X = 0.075          # blunt squared nose tip
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

# Inner insulation layer stations: (x, half_width_y, half_height_z)
# Slightly larger than the parent grip - this is the yellow base insulation.
INNER_INSUL_SECTIONS = [
    (-0.032, 0.0108, 0.0138),
    (-0.040, 0.0088, 0.0118),
    (-0.060, 0.0086, 0.0116),
    (-0.085, 0.0088, 0.0120),
    (-0.105, 0.0094, 0.0126),
    (-0.120, 0.0096, 0.0128),
    (-0.128, 0.0070, 0.0094),
    (-0.131, 0.0036, 0.0048),
]

# Outer insulation sleeve stations: thick orange VDE layer.
# First section matches inner insulation exactly so end caps are coincident
# (ensures mesh connectivity). Subsequent stations are larger (thicker sleeve).
OUTER_INSUL_SECTIONS = [
    (-0.032, 0.0108, 0.0138),  # coincident with inner insulation front face
    (-0.038, 0.0115, 0.0145),  # sleeve thickens
    (-0.050, 0.0112, 0.0142),
    (-0.070, 0.0112, 0.0142),
    (-0.090, 0.0115, 0.0145),
    (-0.110, 0.0120, 0.0150),
    (-0.122, 0.0122, 0.0152),  # bulbous end swell
    (-0.129, 0.0092, 0.0118),
    (-0.132, 0.0050, 0.0064),
]

# Locking lever dimensions
LEVER_LENGTH = 0.028
LEVER_WIDTH = 0.008
LEVER_THICK = 0.003
LEVER_PIVOT_OFFSET = 0.009  # pivot is 9mm from the front of the lever
LEVER_PIVOT_X = -0.055      # lever pivot location along the handle
LEVER_PIVOT_LIMIT = math.radians(35.0)


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

    # Fine horizontal gripping serrations across the inner nose face.
    for i in range(7):
        xi = 0.046 + 0.004 * i
        groove = (
            cq.Workplane("XY")
            .box(0.0014, 0.0024, 0.020)
            .translate((xi, s * JAW_FACE, 0.0))
        )
        jaw = jaw.cut(groove)

    # Rounded pipe-grip recess behind the nose.
    recess = (
        cq.Workplane("XY")
        .circle(0.0045)
        .extrude(0.011, both=True)
        .translate((0.030, s * JAW_FACE, 0.0))
    )
    jaw = jaw.cut(recess)

    # Small scallop teeth around the recess rim (serrated pipe grip).
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

    # Short wire-cutter V-notch between the pipe grip and the pivot.
    notch_pts = [(0.015, s * -0.001), (0.0215, s * -0.001), (0.0185, s * 0.0035)]
    notch = cq.Workplane("XY").polyline(notch_pts).close().extrude(0.011, both=True)
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


def _inner_insulation_solid(s: int) -> cq.Solid:
    """Inner yellow insulation layer - base dielectric sleeve."""
    wires = [
        _ellipse_wire(x, s * _yc(x), 0.0, w, h)
        for x, w, h in INNER_INSUL_SECTIONS
    ]
    return cq.Solid.makeLoft(wires, ruled=False)


def _outer_insulation_solid(s: int) -> cq.Solid:
    """Outer orange VDE insulation sleeve - thick protective layer.

    Solid sleeve that wraps over the inner insulation. The layered look
    comes from the inner yellow layer peeking out at the jaw-end transition
    where the outer sleeve is slightly shorter.
    """
    wires = [
        _ellipse_wire(x, s * _yc(x), 0.0, w, h)
        for x, w, h in OUTER_INSUL_SECTIONS
    ]
    return cq.Solid.makeLoft(wires, ruled=False)


def _lever_slot_solid(s: int) -> cq.Workplane:
    """Recessed slot in the outer grip face where the lever sits."""
    yc_at_lever = s * _yc(LEVER_PIVOT_X)
    slot = (
        cq.Workplane("XY")
        .box(LEVER_LENGTH + 0.002, LEVER_WIDTH + 0.002, LEVER_THICK + 0.001)
        .translate((LEVER_PIVOT_X + LEVER_LENGTH / 2.0 - LEVER_PIVOT_OFFSET, yc_at_lever, 0.012))
    )
    return slot


def _lever_body_solid() -> cq.Workplane:
    """Flat locking release lever: stadium plate with integral pivot boss and thumb pad.

    Single connected solid - no bore, no separate pin. The boss extends below
    the plate to seat in the handle recess. A small thumb pad on top provides
    the actuation surface.
    """
    # Main plate: stadium (rounded rectangle) extruded in Z
    plate = (
        cq.Workplane("XY")
        .slot2D(LEVER_LENGTH, LEVER_WIDTH, angle=0)
        .extrude(LEVER_THICK)
        .translate((-LEVER_PIVOT_OFFSET, 0.0, -LEVER_THICK / 2.0))
    )
    # Pivot boss: cylinder extending below the plate, with 2mm overlap into plate
    # for better mesh connectivity
    boss = (
        cq.Workplane("XY")
        .circle(0.003)
        .extrude(0.008)
        .translate((0.0, 0.0, -LEVER_THICK / 2.0 - 0.006))
    )
    body = plate.union(boss)
    # Thumb pad: small raised rectangle on top near the free end
    # Positioned to overlap with the plate (starts 0.5mm into the plate)
    pad = (
        cq.Workplane("XY")
        .box(0.010, 0.006, 0.0025)
        .translate((LEVER_LENGTH - LEVER_PIVOT_OFFSET - 0.006, 0.0, LEVER_THICK / 2.0 + 0.00075))
    )
    body = body.union(pad)
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="insulated_pliers")

    steel_polished = model.material("steel_polished", rgba=(0.80, 0.81, 0.84, 1.0))
    steel_forged = model.material("steel_forged", rgba=(0.66, 0.67, 0.70, 1.0))
    steel_brushed = model.material("steel_brushed", rgba=(0.61, 0.62, 0.65, 1.0))
    seam_gray = model.material("seam_gray", rgba=(0.45, 0.46, 0.48, 1.0))
    insulation_yellow = model.material("insulation_yellow", rgba=(0.95, 0.82, 0.10, 1.0))
    insulation_orange = model.material("insulation_orange", rgba=(0.92, 0.38, 0.05, 1.0))
    lever_steel = model.material("lever_steel", rgba=(0.55, 0.56, 0.58, 1.0))

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
        # Layered insulated grip: inner yellow base + outer orange sleeve
        part.visual(
            mesh_from_cadquery(_inner_insulation_solid(s), f"{tag}_inner_insul", tolerance=0.0002),
            name="inner_insulation",
            material=insulation_yellow,
        )
        part.visual(
            mesh_from_cadquery(_outer_insulation_solid(s), f"{tag}_outer_insul", tolerance=0.0002),
            name="outer_insulation",
            material=insulation_orange,
        )

        parts.append(part)

    fixed = parts[0]
    moving = parts[1]

    # Circular pivot rivet cap on both sides (bottom face)
    fixed.visual(
        Cylinder(SEAM_R, SEAM_H),
        origin=Origin(xyz=(0.0, 0.0, -(HALF_T + SEAM_H / 2.0 - EPS))),
        name="boss_seam",
        material=seam_gray,
    )
    fixed.visual(
        Cylinder(BOSS_R, BOSS_H),
        origin=Origin(xyz=(0.0, 0.0, -(HALF_T + SEAM_H + BOSS_H / 2.0 - 2.0 * EPS))),
        name="rivet_boss",
        material=steel_brushed,
    )
    # Rivet shaft through both halves
    fixed.visual(
        Cylinder(RIVET_R, 2.0 * HALF_T + SEAM_H + 2.0 * EPS + 0.0005),
        origin=Origin(xyz=(0.0, 0.0, 0.00005)),
        name="rivet_shaft",
        material=steel_brushed,
    )
    # Circular pivot rivet cap on both sides (top face)
    fixed.visual(
        Cylinder(SEAM_R, SEAM_H),
        origin=Origin(xyz=(0.0, 0.0, HALF_T + SEAM_H / 2.0 + EPS)),
        name="head_seam",
        material=seam_gray,
    )
    fixed.visual(
        Cylinder(BOSS_R, BOSS_H),
        origin=Origin(xyz=(0.0, 0.0, HALF_T + SEAM_H + BOSS_H / 2.0)),
        name="rivet_head",
        material=steel_brushed,
    )

    # Primary articulation: revolute pivot at the rivet.
    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=fixed,
        child=moving,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=3.0, lower=0.0, upper=OPEN_LIMIT),
    )

    # ---- Locking release lever ----
    # The lever sits recessed in the outer face of half_0's grip,
    # near the pivot area. It pivots on a small pin (Z axis).
    lever = model.part("locking_lever")
    yc_lever = _yc(LEVER_PIVOT_X)
    lever_z = 0.018  # sits proud on the outer face of the grip

    lever.visual(
        mesh_from_cadquery(_lever_body_solid(), "lever_body", tolerance=0.0002),
        name="lever_body",
        material=lever_steel,
    )

    # Lever articulation: pivots inside handle about Z axis at its pivot point.
    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=fixed,
        child=lever,
        origin=Origin(xyz=(LEVER_PIVOT_X, yc_lever, lever_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=0.0, upper=LEVER_PIVOT_LIMIT
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    half0 = object_model.get_part("plier_half_0")
    half1 = object_model.get_part("plier_half_1")
    lever = object_model.get_part("locking_lever")
    pivot = object_model.get_articulation("pivot")
    lever_pivot = object_model.get_articulation("lever_pivot")

    jaw0 = half0.get_visual("jaw")
    jaw1 = half1.get_visual("jaw")
    hub0 = half0.get_visual("hub")
    hub1 = half1.get_visual("hub")

    # ---- Pivot joint contract ----
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

    # ---- Lever joint contract ----
    lev_limits = lever_pivot.motion_limits
    ctx.check(
        "lever_pivot is a non-fixed revolute joint",
        lever_pivot.articulation_type == ArticulationType.REVOLUTE
        and lev_limits is not None
        and lev_limits.lower is not None
        and lev_limits.upper is not None
        and lev_limits.upper > lev_limits.lower + 0.1,
        details=f"limits={lev_limits}",
    )

    # ---- Insulated grip layers exist on both halves ----
    inner0 = half0.get_visual("inner_insulation")
    outer0 = half0.get_visual("outer_insulation")
    inner1 = half1.get_visual("inner_insulation")
    outer1 = half1.get_visual("outer_insulation")
    ctx.check(
        "inner insulation layer exists on both halves",
        inner0 is not None and inner1 is not None,
        details="missing inner_insulation visual",
    )
    ctx.check(
        "outer insulation sleeve exists on both halves",
        outer0 is not None and outer1 is not None,
        details="missing outer_insulation visual",
    )

    # Outer insulation wraps around inner (is strictly larger in cross-section)
    outer0_aabb = ctx.part_element_world_aabb(half0, elem="outer_insulation")
    inner0_aabb = ctx.part_element_world_aabb(half0, elem="inner_insulation")
    if outer0_aabb is not None and inner0_aabb is not None:
        ctx.check(
            "outer insulation sleeve is larger than inner layer on half_0",
            outer0_aabb[1][2] > inner0_aabb[1][2] - 0.0005
            and outer0_aabb[0][2] < inner0_aabb[0][2] + 0.0005,
            details=f"outer_z=({outer0_aabb[0][2]:.4f},{outer0_aabb[1][2]:.4f}) "
                    f"inner_z=({inner0_aabb[0][2]:.4f},{inner0_aabb[1][2]:.4f})",
        )
    else:
        ctx.fail("insulation AABBs resolve", "missing insulation element AABB")

    # ---- Rivet cap on both sides ----
    boss_aabb = ctx.part_element_world_aabb(half0, elem="rivet_boss")
    head_aabb = ctx.part_element_world_aabb(half0, elem="rivet_head")
    if boss_aabb is not None and head_aabb is not None:
        boss_dia = boss_aabb[1][0] - boss_aabb[0][0]
        head_dia = head_aabb[1][0] - head_aabb[0][0]
        ctx.check(
            "circular rivet cap on bottom side (~25mm)",
            0.023 <= boss_dia <= 0.027,
            details=f"boss_dia={boss_dia:.4f}",
        )
        ctx.check(
            "circular rivet cap on top side (~25mm)",
            0.023 <= head_dia <= 0.027,
            details=f"head_dia={head_dia:.4f}",
        )
        # Both caps are proud of the respective outer faces
        ctx.check(
            "rivet boss proud below and rivet head proud above",
            boss_aabb[0][2] < -(HALF_T + 0.001) and head_aabb[1][2] > HALF_T + 0.001,
            details=f"boss_min_z={boss_aabb[0][2]:.4f} head_max_z={head_aabb[1][2]:.4f}",
        )
    else:
        ctx.fail("rivet cap AABBs resolve", "missing rivet_boss/rivet_head AABB")

    # ---- Lever mounted on half_0 handle ----
    lever_aabb = ctx.part_world_aabb(lever)
    grip0_outer_aabb = ctx.part_element_world_aabb(half0, elem="outer_insulation")
    if lever_aabb is not None and grip0_outer_aabb is not None:
        # Lever is near the handle (within the grip zone, x between -0.08 and -0.03)
        ctx.check(
            "lever is positioned along the handle grip area",
            -0.08 <= lever_aabb[0][0] and lever_aabb[1][0] <= -0.02,
            details=f"lever_x=({lever_aabb[0][0]:.4f},{lever_aabb[1][0]:.4f})",
        )
        # Lever sits above the tool center plane on the outer face
        ctx.check(
            "lever sits on the outer face of the grip (above tool mid-plane)",
            lever_aabb[0][2] >= 0.008,
            details=f"lever_min_z={lever_aabb[0][2]:.4f}",
        )
        # Lever footprint overlaps with the grip in XY (it's mounted on the grip)
        ctx.expect_overlap(
            lever,
            half0,
            axes="xy",
            elem_a="lever_body",
            elem_b="outer_insulation",
            min_overlap=0.003,
            name="lever overlaps grip footprint in plan view",
        )
    else:
        ctx.fail("lever/grip AABBs resolve", "missing lever or grip AABB")

    # The lever boss (part of lever_body) seats into the outer insulation sleeve
    # recess. Allow this small local overlap for the pivot mounting.
    ctx.allow_overlap(
        lever,
        half0,
        elem_a="lever_body",
        elem_b="outer_insulation",
        reason="The lever pivot boss seats into the outer insulation sleeve recess "
               "to mount the lever inside the handle grip.",
    )
    ctx.expect_contact(
        lever,
        half0,
        elem_a="lever_body",
        elem_b="outer_insulation",
        contact_tol=0.005,
        name="lever body contacts the handle grip surface",
    )

    # ---- Closed rest pose: jaws nearly touch ----
    ctx.expect_gap(
        half0,
        half1,
        axis="y",
        positive_elem=jaw0,
        negative_elem=jaw1,
        min_gap=0.0002,
        max_gap=0.0015,
        name="jaws closed and nearly touching at rest",
    )

    # Hub interleaving at pivot
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

    # Rivet shaft captured through moving half
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

    # ---- Decisive open pose ----
    closed_jaw1 = ctx.part_element_world_aabb(half1, elem="jaw")
    closed_grip1 = ctx.part_element_world_aabb(half1, elem="outer_insulation")
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
        open_grip1 = ctx.part_element_world_aabb(half1, elem="outer_insulation")
        ctx.check(
            "moving jaw swings away from the fixed jaw",
            closed_jaw1 is not None
            and open_jaw1 is not None
            and open_jaw1[0][1] < closed_jaw1[0][1] - 0.018,
            details=f"closed_min_y={closed_jaw1}, open_min_y={open_jaw1}",
        )
        ctx.check(
            "moving handle spreads outward as the jaws open",
            closed_grip1 is not None
            and open_grip1 is not None
            and open_grip1[1][1] > closed_grip1[1][1] + 0.03,
            details=f"closed_max_y={closed_grip1}, open_max_y={open_grip1}",
        )

    # ---- Lever pivot motion: lever swings when actuated ----
    closed_lever = ctx.part_world_aabb(lever)
    with ctx.pose({lever_pivot: LEVER_PIVOT_LIMIT}):
        open_lever = ctx.part_world_aabb(lever)
        ctx.check(
            "lever swings when pivoted to upper limit",
            closed_lever is not None
            and open_lever is not None
            and (abs(open_lever[1][1] - closed_lever[1][1]) > 0.003
                 or abs(open_lever[0][0] - closed_lever[0][0]) > 0.003),
            details=f"closed={closed_lever}, open={open_lever}",
        )

    return ctx.report()


object_model = build_object_model()
