from __future__ import annotations

# Compact diagonal cutting pliers.
# Variant of heavy-duty lineman pliers: shorter handles, angled diagonal
# cutting jaws, and circular pivot rivet caps visible on both sides.
#
# Layout: tool lies in XY plane with Z as thickness axis.
# Rivet pivot at origin. Jaws point +X, handles sweep to -X.
# plier_half_0 (root, jaw on +Y), plier_half_1 (moving, jaw on -Y).
# The two halves are half-lapped at the pivot.

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
HALF_T = 0.007           # half thickness of each forged plate (full 0.014)
LAP_R = 0.012            # half-lap joint disc radius at the pivot
HUB_R = 0.011            # forged hub radius around the rivet
CAP_R = 0.010            # rivet cap radius on both sides (~0.020 m diameter)
CAP_H = 0.0025           # cap height (proud of surface)
SEAM_R = 0.0105          # visible seam ring under the cap
SEAM_H = 0.0005
RIVET_R = 0.003          # rivet shaft
EPS = 0.0001             # lap clearance
JAW_FACE = 0.0002        # closed jaw inner faces
NOSE_X = 0.038           # diagonal cutter jaw tip (compact)
OPEN_LIMIT = math.radians(30.0)

TANG_HALF_W = 0.003      # steel handle tang half width

# Handle tang centerline (compact short handles).
TANG_PTS = [
    (-0.008, -0.003),
    (-0.020, -0.008),
    (-0.040, -0.014),
    (-0.058, -0.018),
    (-0.068, -0.020),
]
CENTERLINE = TANG_PTS + [(-0.074, -0.021)]

# Grip loft stations: (x, half_width_y, half_height_z) - compact grip
GRIP_SECTIONS = [
    (-0.022, 0.0072, 0.0095),   # flared guard near pivot
    (-0.030, 0.0060, 0.0085),
    (-0.042, 0.0058, 0.0082),
    (-0.055, 0.0060, 0.0084),
    (-0.065, 0.0064, 0.0088),
    (-0.072, 0.0046, 0.0064),
    (-0.074, 0.0022, 0.0032),
]

# Red inlay stations along the outer/top face
INLAY_XS = [-0.026, -0.038, -0.050, -0.060, -0.066]
INLAY_HALF_H = 0.0055
INLAY_Z_CENTER = 0.005


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
    s_val = 1 if s > 0 else -1
    cut = cq.Workplane("XY").circle(LAP_R).extrude(0.010)
    if s_val > 0:
        return cut.translate((0.0, 0.0, -EPS))
    return cut.translate((0.0, 0.0, -0.010 + EPS))


def _diag_jaw_solid(s: int) -> cq.Workplane:
    """Compact diagonal cutting jaw: angled bevel cutting edges, pointed tip."""
    # Diagonal cutter profile: tapered, with angled cutting edge
    profile = [
        (0.008, JAW_FACE),
        (NOSE_X, JAW_FACE),       # pointed tip at inner face
        (NOSE_X - 0.002, 0.008),  # tip top
        (0.030, 0.0105),           # angled cutting edge
        (0.020, 0.0120),
        (0.012, 0.0130),
        (0.008, 0.0115),
    ]
    pts = [(x, s * y) for x, y in profile]
    jaw = cq.Workplane("XY").polyline(pts).close().extrude(HALF_T, both=True)

    # Diagonal cutting bevel: angled cut across the inner face near the tip
    bevel_pts = [
        (NOSE_X - 0.001, s * JAW_FACE),
        (NOSE_X - 0.010, s * JAW_FACE),
        (NOSE_X - 0.006, s * (JAW_FACE + 0.003)),
        (NOSE_X - 0.001, s * (JAW_FACE + 0.002)),
    ]
    bevel = cq.Workplane("XY").polyline(bevel_pts).close().extrude(0.008, both=True)
    jaw = jaw.cut(bevel)

    # Small wire-cutter notch near the pivot
    notch_pts = [
        (0.010, s * -0.001),
        (0.015, s * -0.001),
        (0.0125, s * 0.002),
    ]
    notch = cq.Workplane("XY").polyline(notch_pts).close().extrude(0.008, both=True)
    jaw = jaw.cut(notch)

    return jaw.cut(_lap_cut(s))


def _hub_solid(s: int) -> cq.Workplane:
    hub = cq.Workplane("XY").circle(HUB_R).extrude(HALF_T, both=True)
    return hub.cut(_lap_cut(s))


def _shank_solid(s: int) -> cq.Workplane:
    pts = [(x, s * y) for x, y in TANG_PTS]
    loop = _strip_loop(pts, TANG_HALF_W)
    shank = cq.Workplane("XY").polyline(loop).close().extrude(HALF_T, both=True)
    return shank.cut(_lap_cut(s))


def _ellipse_wire(x: float, yc: float, zc: float, w: float, h: float) -> cq.Wire:
    wp = cq.Workplane("YZ", origin=(x, 0.0, 0.0)).center(yc, zc).ellipse(w, h)
    return wp.val()


def _grip_solid(s: int) -> cq.Solid:
    wires = [
        _ellipse_wire(x, s * _yc(x), 0.0, w, h)
        for x, w, h in GRIP_SECTIONS
    ]
    return cq.Solid.makeLoft(wires, ruled=False)


def _inlay_solid(s: int) -> cq.Solid:
    wires = [
        _ellipse_wire(x, s * _yc(x), INLAY_Z_CENTER, _grip_w(x) - 0.0010, INLAY_HALF_H)
        for x in INLAY_XS
    ]
    return cq.Solid.makeLoft(wires, ruled=False)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_diagonal_pliers")

    steel_polished = model.material("steel_polished", rgba=(0.82, 0.83, 0.86, 1.0))
    steel_forged = model.material("steel_forged", rgba=(0.62, 0.63, 0.66, 1.0))
    steel_brushed = model.material("steel_brushed", rgba=(0.58, 0.59, 0.62, 1.0))
    seam_gray = model.material("seam_gray", rgba=(0.42, 0.43, 0.45, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.06, 0.06, 0.07, 1.0))
    grip_red = model.material("grip_red", rgba=(0.82, 0.10, 0.12, 1.0))

    # ---- Two plier halves ----
    half_parts = []
    for part_name, s in (("plier_half_0", 1), ("plier_half_1", -1)):
        part = model.part(part_name)
        tag = part_name.replace("plier_half_", "half")

        part.visual(
            mesh_from_cadquery(_diag_jaw_solid(s), f"{tag}_jaw", tolerance=0.0002),
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
        part.visual(
            mesh_from_cadquery(_grip_solid(s), f"{tag}_grip", tolerance=0.0002),
            name="grip",
            material=rubber_black,
        )
        part.visual(
            mesh_from_cadquery(_inlay_solid(s), f"{tag}_inlay", tolerance=0.0002),
            name="grip_inlay",
            material=grip_red,
        )
        half_parts.append(part)

    fixed = half_parts[0]
    moving = half_parts[1]

    # ---- Circular pivot rivet cap on BOTH sides ----
    # Bottom side (on fixed half): seam + cap
    fixed.visual(
        Cylinder(SEAM_R, SEAM_H),
        origin=Origin(xyz=(0.0, 0.0, -(HALF_T + SEAM_H / 2.0 - EPS))),
        name="bottom_seam",
        material=seam_gray,
    )
    fixed.visual(
        Cylinder(CAP_R, CAP_H),
        origin=Origin(xyz=(0.0, 0.0, -(HALF_T + SEAM_H + CAP_H / 2.0 - 2.0 * EPS))),
        name="rivet_cap_bottom",
        material=steel_brushed,
    )

    # Rivet shaft through both halves
    fixed.visual(
        Cylinder(RIVET_R, 2.0 * HALF_T + SEAM_H + 2.0 * EPS + 0.0004),
        origin=Origin(xyz=(0.0, 0.0, 0.00005)),
        name="rivet_shaft",
        material=steel_brushed,
    )

    # Top side (also on fixed half, visible above moving half): seam + cap
    fixed.visual(
        Cylinder(SEAM_R, SEAM_H),
        origin=Origin(xyz=(0.0, 0.0, HALF_T + SEAM_H / 2.0 + EPS)),
        name="top_seam",
        material=seam_gray,
    )
    fixed.visual(
        Cylinder(CAP_R, CAP_H),
        origin=Origin(xyz=(0.0, 0.0, HALF_T + SEAM_H + CAP_H / 2.0)),
        name="rivet_cap_top",
        material=steel_brushed,
    )

    # ---- Primary pivot articulation ----
    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=fixed,
        child=moving,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=3.0, lower=0.0, upper=OPEN_LIMIT),
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
    grip0 = half0.get_visual("grip")
    grip1 = half1.get_visual("grip")

    # ---- Joint contract: pivot is revolute 0..30 deg ----
    plim = pivot.motion_limits
    ctx.check(
        "pivot is a 0..30 degree revolute joint",
        pivot.articulation_type == ArticulationType.REVOLUTE
        and plim is not None
        and plim.lower is not None
        and plim.upper is not None
        and abs(plim.lower) < 1e-9
        and abs(plim.upper - OPEN_LIMIT) < 1e-6,
        details=f"limits={plim}",
    )

    # ---- Compact diagonal jaws: jaw tip is shorter than parent's 0.075m ----
    jaw0_aabb = ctx.part_element_world_aabb(half0, elem="jaw")
    if jaw0_aabb is not None:
        jaw_length = jaw0_aabb[1][0] - jaw0_aabb[0][0]
        ctx.check(
            "diagonal cutting jaws are compact (< 0.045m from pivot)",
            jaw0_aabb[1][0] < 0.045 and jaw_length < 0.042,
            details=f"jaw_tip_x={jaw0_aabb[1][0]:.4f} jaw_length={jaw_length:.4f}",
        )
    else:
        ctx.fail("jaw AABB resolves", "missing jaw element AABB")

    # ---- Short handles: grip extends less than parent's ~0.13m ----
    g0 = ctx.part_element_world_aabb(half0, elem="grip")
    if g0 is not None:
        handle_reach = abs(g0[0][0])
        ctx.check(
            "short handles: grip does not extend past -0.080m",
            handle_reach < 0.082,
            details=f"grip_min_x={g0[0][0]:.4f}",
        )

    # ---- Closed jaw gap ----
    ctx.expect_gap(
        half0,
        half1,
        axis="y",
        positive_elem=jaw0,
        negative_elem=jaw1,
        min_gap=0.0001,
        max_gap=0.0012,
        name="diagonal cutting jaws closed and nearly touching at rest",
    )

    # ---- Half-lap stacking at pivot ----
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

    # ---- Rivet shaft captured through moving half ----
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

    # ---- Rivet caps on both sides ----
    cap_top_aabb = ctx.part_element_world_aabb(half0, elem="rivet_cap_top")
    cap_bot_aabb = ctx.part_element_world_aabb(half0, elem="rivet_cap_bottom")
    ok_caps = cap_top_aabb is not None and cap_bot_aabb is not None
    if ok_caps:
        top_min, top_max = cap_top_aabb
        bot_min, bot_max = cap_bot_aabb
        # Both caps are circular, roughly same diameter
        top_dia = top_max[0] - top_min[0]
        bot_dia = bot_max[0] - bot_min[0]
        ctx.check(
            "circular rivet cap on top side (~0.020m diameter)",
            0.018 <= top_dia <= 0.022,
            details=f"top_cap_dia={top_dia:.4f}",
        )
        ctx.check(
            "circular rivet cap on bottom side (~0.020m diameter)",
            0.018 <= bot_dia <= 0.022,
            details=f"bot_cap_dia={bot_dia:.4f}",
        )
        # Top cap is above Z=0, bottom cap below Z=0
        ctx.check(
            "top cap sits above tool midplane",
            top_min[2] > 0.005,
            details=f"top_cap_min_z={top_min[2]:.4f}",
        )
        ctx.check(
            "bottom cap sits below tool midplane",
            bot_max[2] < -0.005,
            details=f"bot_cap_max_z={bot_max[2]:.4f}",
        )
    else:
        ctx.fail("rivet cap AABBs resolve", "missing rivet_cap_top or rivet_cap_bottom")

    # ---- Overall compact size ----
    a0 = ctx.part_world_aabb(half0)
    a1 = ctx.part_world_aabb(half1)
    if a0 is not None and a1 is not None and g0 is not None:
        g1 = ctx.part_element_world_aabb(half1, elem="grip")
        if g1 is not None:
            length = max(a0[1][0], a1[1][0]) - min(a0[0][0], a1[0][0])
            across = g1[1][1] - g0[0][1]
            ctx.check(
                "overall compact length (~0.10-0.15m)",
                0.095 <= length <= 0.155,
                details=f"length={length:.4f}",
            )
            ctx.check(
                "handle grips span about 0.04-0.06m across",
                0.035 <= across <= 0.065,
                details=f"across={across:.4f}",
            )

    # ---- Decisive open pose ----
    closed_jaw1 = ctx.part_element_world_aabb(half1, elem="jaw")
    closed_grip1 = ctx.part_element_world_aabb(half1, elem="grip")
    with ctx.pose({pivot: OPEN_LIMIT}):
        ctx.expect_gap(
            half0,
            half1,
            axis="y",
            positive_elem=jaw0,
            negative_elem=jaw1,
            min_gap=0.003,
            name="jaws open apart at the 30 degree pose",
        )
        open_jaw1 = ctx.part_element_world_aabb(half1, elem="jaw")
        open_grip1 = ctx.part_element_world_aabb(half1, elem="grip")
        ctx.check(
            "moving jaw swings away from the fixed jaw",
            closed_jaw1 is not None
            and open_jaw1 is not None
            and open_jaw1[0][1] < closed_jaw1[0][1] - 0.010,
            details=f"closed_min_y={closed_jaw1}, open_min_y={open_jaw1}",
        )
        ctx.check(
            "moving handle spreads outward as the jaws open",
            closed_grip1 is not None
            and open_grip1 is not None
            and open_grip1[1][1] > closed_grip1[1][1] + 0.015,
            details=f"closed_max_y={closed_grip1}, open_max_y={open_grip1}",
        )

    return ctx.report()


object_model = build_object_model()
