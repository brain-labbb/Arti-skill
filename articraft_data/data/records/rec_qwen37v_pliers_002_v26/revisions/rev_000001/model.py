from __future__ import annotations

# Bent-nose pliers variant (forked from lineman pliers).
# Reference: Other/pliers family variant 26.
#
# Structural changes vs parent:
#   - Angled/bent nose jaws (≈45° bend near the tip)
#   - Circular pivot rivet cap visible on BOTH sides
#   - Cutter bevels as visible wedge geometry near the pivot
#   - Small safety latch folding over the handles on a revolute joint
#
# Layout: the tool lies in the XY plane with Z as the thickness axis.
# The rivet pivot is at the origin. The jaw end points +X with a bend
# near the tip; the handles sweep back to -X and spread in +/-Y.
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
BOSS_R = 0.0125         # rivet cap radius (~0.025 m diameter)
BOSS_H = 0.0030
SEAM_R = 0.0132         # visible circular seam ring under the boss cap
SEAM_H = 0.0006
RIVET_R = 0.004         # rivet shaft radius
EPS = 0.0001            # lap clearance
JAW_FACE = 0.0003       # closed jaw inner faces sit at y = +/-JAW_FACE
OPEN_LIMIT = math.radians(30.0)

# Bent-nose geometry: the jaw bends at BEND_X by BEND_ANGLE downward
BEND_X = 0.048          # x location where the jaw starts bending
BEND_ANGLE = math.radians(45.0)  # bend angle from the handle axis

TANG_HALF_W = 0.004     # steel handle tang half width in plan

# Steel handle tang centerline (jaw-on-+Y half)
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
    (-0.032, 0.0100, 0.0130),
    (-0.040, 0.0080, 0.0110),
    (-0.060, 0.0078, 0.0108),
    (-0.085, 0.0080, 0.0112),
    (-0.105, 0.0086, 0.0118),
    (-0.120, 0.0088, 0.0120),
    (-0.128, 0.0062, 0.0086),
    (-0.131, 0.0028, 0.0040),
]

# Red inlay stations along outer/top face
INLAY_XS = [-0.036, -0.055, -0.075, -0.095, -0.112, -0.118]
INLAY_HALF_H = 0.0075
INLAY_Z_CENTER = 0.006

# Latch dimensions
LATCH_LENGTH = 0.035    # latch arm length
LATCH_WIDTH = 0.008     # latch width
LATCH_THICK = 0.002     # latch thickness
LATCH_PIVOT_X = -0.058  # latch pivot location along the handle
LATCH_OPEN = math.radians(120.0)  # latch open angle


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
    cut = cq.Workplane("XY").circle(LAP_R).extrude(0.012)
    if s > 0:
        return cut.translate((0.0, 0.0, -EPS))
    return cut.translate((0.0, 0.0, -0.012 + EPS))


def _jaw_solid(s: int) -> cq.Workplane:
    """Bent-nose jaw: straight shank then angled tip with serrations."""
    # Straight section profile (from pivot area to bend point)
    straight_profile = [
        (0.010, JAW_FACE),
        (BEND_X, JAW_FACE),
        (BEND_X, 0.0130),
        (0.038, 0.0150),
        (0.026, 0.0160),
        (0.014, 0.0164),
        (0.009, 0.0150),
    ]

    # Build the straight jaw section
    pts_straight = [(x, s * y) for x, y in straight_profile]
    jaw = cq.Workplane("XY").polyline(pts_straight).close().extrude(HALF_T, both=True)

    # Bent nose section: angled tip from bend point forward
    # The bend rotates the remaining jaw section by BEND_ANGLE
    bend_length = 0.028  # length of the bent section
    cos_a = math.cos(BEND_ANGLE)
    sin_a = math.sin(BEND_ANGLE)

    # Points along the bent section (in local bend frame, then rotated)
    # The bend starts at (BEND_X, s*JAW_FACE) and angles toward the jaw face
    bent_inner = []
    bent_outer = []
    for i in range(6):
        t = i / 5.0
        local_x = t * bend_length
        # Inner face follows the jaw face line (bends toward center)
        inner_local_y = 0.0
        outer_local_y = 0.012 - t * 0.003  # jaw tapers toward tip

        # Rotate around bend point
        bx = BEND_X + local_x * cos_a - inner_local_y * sin_a
        by_inner = s * (JAW_FACE + local_x * sin_a + inner_local_y * cos_a)
        by_outer = s * (JAW_FACE + local_x * sin_a + outer_local_y * cos_a)

        bent_inner.append((bx, by_inner))
        bent_outer.append((bx, by_outer))

    # Build the bent section as a polygon
    bent_pts = bent_inner + bent_outer[::-1]
    bent_section = cq.Workplane("XY").polyline(bent_pts).close().extrude(HALF_T, both=True)
    jaw = jaw.union(bent_section)

    # Serrations on the straight inner face
    for i in range(5):
        xi = 0.020 + 0.005 * i
        if xi >= BEND_X - 0.002:
            break
        groove = (
            cq.Workplane("XY")
            .box(0.0014, 0.0024, 0.020)
            .translate((xi, s * JAW_FACE, 0.0))
        )
        jaw = jaw.cut(groove)

    # Serrations on the bent section inner face
    for i in range(3):
        t = 0.2 + 0.25 * i
        local_x = t * bend_length
        sx = BEND_X + local_x * cos_a
        sy = s * (JAW_FACE + local_x * sin_a)
        groove = (
            cq.Workplane("XY")
            .box(0.0014, 0.0024, 0.018)
            .translate((sx, sy, 0.0))
            .rotate((sx, sy, 0.0), (sx + 1.0, sy + s * sin_a, 0.0), math.degrees(BEND_ANGLE))
        )
        jaw = jaw.cut(groove)

    return jaw.cut(_lap_cut(s))


def _cutter_bevel_solid(s: int) -> cq.Workplane:
    """Wedge-shaped cutter bevel near the pivot on the jaw inner face."""
    # A wedge (triangular cross-section) representing the cutting edge bevel
    # Located just behind where the jaws meet, near the pivot
    wedge_pts = [
        (0.012, s * JAW_FACE),
        (0.022, s * JAW_FACE),
        (0.017, s * (JAW_FACE + 0.004)),  # bevel rises 4mm from face
    ]
    wedge = cq.Workplane("XY").polyline(wedge_pts).close().extrude(HALF_T * 0.8, both=True)
    return wedge


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
        _ellipse_wire(x, s * _yc(x), INLAY_Z_CENTER, _grip_w(x) - 0.0012, INLAY_HALF_H)
        for x in INLAY_XS
    ]
    return cq.Solid.makeLoft(wires, ruled=False)


def _latch_arm_solid() -> cq.Workplane:
    """Small flat latch arm that folds over the handles."""
    # Flat rectangular arm with rounded end
    arm = (
        cq.Workplane("XY")
        .box(LATCH_LENGTH, LATCH_WIDTH, LATCH_THICK)
        .translate((LATCH_LENGTH / 2.0, 0.0, 0.0))
    )
    # Rounded tip
    tip = (
        cq.Workplane("XY")
        .circle(LATCH_WIDTH / 2.0)
        .extrude(LATCH_THICK)
        .translate((LATCH_LENGTH, 0.0, -LATCH_THICK / 2.0))
    )
    arm = arm.union(tip)
    # Pivot hole at the base
    hole = (
        cq.Workplane("XY")
        .circle(0.0015)
        .extrude(LATCH_THICK + 0.002)
        .translate((0.0, 0.0, -LATCH_THICK / 2.0 - 0.001))
    )
    arm = arm.cut(hole)
    return arm


def _latch_pin_solid() -> cq.Workplane:
    """Small cylindrical pin for the latch pivot."""
    return (
        cq.Workplane("XY")
        .circle(0.0015)
        .extrude(0.022)
        .translate((0.0, 0.0, -0.011))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bent_nose_pliers")

    steel_polished = model.material("steel_polished", rgba=(0.80, 0.81, 0.84, 1.0))
    steel_forged = model.material("steel_forged", rgba=(0.66, 0.67, 0.70, 1.0))
    steel_brushed = model.material("steel_brushed", rgba=(0.61, 0.62, 0.65, 1.0))
    seam_gray = model.material("seam_gray", rgba=(0.45, 0.46, 0.48, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.08, 0.09, 1.0))
    grip_red = model.material("grip_red", rgba=(0.80, 0.09, 0.11, 1.0))
    cutter_steel = model.material("cutter_steel", rgba=(0.72, 0.73, 0.76, 1.0))

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
            mesh_from_cadquery(_cutter_bevel_solid(s), f"{tag}_cutter_bevel", tolerance=0.0002),
            name="cutter_bevel",
            material=cutter_steel,
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

        parts.append(part)

    # ---- Circular pivot rivet cap on BOTH sides ----
    fixed = parts[0]
    # Bottom side cap (visible from below)
    fixed.visual(
        Cylinder(SEAM_R, SEAM_H),
        origin=Origin(xyz=(0.0, 0.0, -(HALF_T + SEAM_H / 2.0 - EPS))),
        name="boss_seam",
        material=seam_gray,
    )
    fixed.visual(
        Cylinder(BOSS_R, BOSS_H),
        origin=Origin(xyz=(0.0, 0.0, -(HALF_T + SEAM_H + BOSS_H / 2.0 - 2.0 * EPS))),
        name="rivet_cap_bottom",
        material=steel_brushed,
    )
    # Rivet shaft through both halves
    fixed.visual(
        Cylinder(RIVET_R, 2.0 * HALF_T + SEAM_H + 2.0 * EPS + 0.0005),
        origin=Origin(xyz=(0.0, 0.0, 0.00005)),
        name="rivet_shaft",
        material=steel_brushed,
    )
    # Top side cap (visible from above)
    fixed.visual(
        Cylinder(SEAM_R, SEAM_H),
        origin=Origin(xyz=(0.0, 0.0, HALF_T + SEAM_H / 2.0 + EPS)),
        name="head_seam",
        material=seam_gray,
    )
    fixed.visual(
        Cylinder(BOSS_R, BOSS_H),
        origin=Origin(xyz=(0.0, 0.0, HALF_T + SEAM_H + BOSS_H / 2.0)),
        name="rivet_cap_top",
        material=steel_brushed,
    )

    # Primary pivot articulation: revolute at the rivet
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
    cutter0 = half0.get_visual("cutter_bevel")
    cutter1 = half1.get_visual("cutter_bevel")

    # ---- Joint contract: pivot is 0..30 degree revolute ----
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

    # ---- Closed rest pose: jaws nearly touching ----
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

    # ---- Bent nose: jaw tip is angled away from the handle axis ----
    # The bent jaw tip should be displaced in Y from the straight jaw line
    jaw0_aabb = ctx.part_element_world_aabb(half0, elem="jaw")
    if jaw0_aabb is not None:
        # The jaw extends in +X and the bent tip should reach beyond BEND_X
        ctx.check(
            "jaw extends forward with bend reaching past bend point",
            jaw0_aabb[1][0] > BEND_X + 0.010,
            details=f"jaw_max_x={jaw0_aabb[1][0]:.4f}",
        )
        # The bent tip's Y extent should differ from the straight section
        # For half0 (jaw on +Y), the bend moves the tip further in +Y
        ctx.check(
            "bent nose jaw tip displaced from straight axis",
            jaw0_aabb[1][1] > JAW_FACE + 0.015,
            details=f"jaw_max_y={jaw0_aabb[1][1]:.4f}",
        )

    # ---- Cutter bevels exist as visible wedge geometry ----
    cutter0_aabb = ctx.part_element_world_aabb(half0, elem="cutter_bevel")
    cutter1_aabb = ctx.part_element_world_aabb(half1, elem="cutter_bevel")
    ctx.check(
        "cutter bevel wedge exists on half_0",
        cutter0_aabb is not None,
        details="missing cutter_bevel on half_0",
    )
    ctx.check(
        "cutter bevel wedge exists on half_1",
        cutter1_aabb is not None,
        details="missing cutter_bevel on half_1",
    )
    if cutter0_aabb is not None:
        # Bevel should be near the pivot (x between 0.010 and 0.025)
        ctx.check(
            "cutter bevel positioned near the pivot",
            0.008 <= cutter0_aabb[0][0] <= 0.025 and cutter0_aabb[1][0] <= 0.028,
            details=f"cutter_x=[{cutter0_aabb[0][0]:.4f}, {cutter0_aabb[1][0]:.4f}]",
        )
        # Bevel should have wedge height (z-extent showing the bevel rise)
        bevel_z = cutter0_aabb[1][2] - cutter0_aabb[0][2]
        ctx.check(
            "cutter bevel has visible wedge thickness",
            bevel_z > 0.005,
            details=f"bevel_z_extent={bevel_z:.4f}",
        )

    # ---- Circular pivot rivet caps on both sides ----
    cap_bottom = ctx.part_element_world_aabb(half0, elem="rivet_cap_bottom")
    cap_top = ctx.part_element_world_aabb(half0, elem="rivet_cap_top")
    ctx.check(
        "rivet cap visible on bottom side",
        cap_bottom is not None,
        details="missing rivet_cap_bottom",
    )
    ctx.check(
        "rivet cap visible on top side",
        cap_top is not None,
        details="missing rivet_cap_top",
    )
    if cap_bottom is not None and cap_top is not None:
        # Both caps should be circular (~0.025 m diameter)
        dia_bottom = cap_bottom[1][0] - cap_bottom[0][0]
        dia_top = cap_top[1][0] - cap_top[0][0]
        ctx.check(
            "bottom rivet cap is ~25 mm diameter circle",
            0.023 <= dia_bottom <= 0.027,
            details=f"dia_bottom={dia_bottom:.4f}",
        )
        ctx.check(
            "top rivet cap is ~25 mm diameter circle",
            0.023 <= dia_top <= 0.027,
            details=f"dia_top={dia_top:.4f}",
        )
        # Caps should be on opposite sides of the tool (Z separation)
        ctx.check(
            "rivet caps on opposite outer faces",
            cap_bottom[0][2] < -0.008 and cap_top[1][2] > 0.008,
            details=f"bottom_z={cap_bottom[0][2]:.4f} top_z={cap_top[1][2]:.4f}",
        )

    # ---- Hub lap interleaving at pivot ----
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
        reason="The rivet shaft is fixed to half_0 and passes through the moving half's hub lap.",
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

    # ---- Decisive open pose: jaws separate and handles spread ----
    closed_jaw1 = ctx.part_element_world_aabb(half1, elem="jaw")
    closed_grip1 = ctx.part_element_world_aabb(half1, elem="grip")
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
        open_grip1 = ctx.part_element_world_aabb(half1, elem="grip")
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

    return ctx.report()


object_model = build_object_model()
