from __future__ import annotations

# Slip-joint lineman pliers variant.
# Broad jaws with prominent serrated gripping teeth, cutter notch, and a
# slip-joint mechanism: a short prismatic slot in the fixed half lets the
# pivot pin slide to adjust jaw opening range.
#
# Layout: the tool lies in the XY plane with Z as the thickness axis.
# The pivot slot is centered near the origin. The blunt squared nose
# points +X; the handles sweep back to -X and spread in +/-Y.
#
# Part chain:
#   half_0 (root, fixed jaw/handle, carries the slot)
#     -> pivot_slider (prismatic along the slot, carries rivet boss/shaft)
#       -> half_1 (revolute about the rivet, moving jaw/handle)

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
HALF_T = 0.009          # half thickness of each forged plate (full plate 0.018)
LAP_R = 0.016           # half-lap joint disc radius at the pivot
HUB_R = 0.015           # forged hub radius around the pivot area
BOSS_R = 0.0125         # rivet boss radius (~0.025 m diameter)
BOSS_H = 0.0030
SEAM_R = 0.0132         # visible circular seam ring under the boss cap
SEAM_H = 0.0006
RIVET_R = 0.004         # rivet shaft captured through the moving half's lap
EPS = 0.0001            # lap clearance so the stacked halves do not penetrate
JAW_FACE = 0.0003       # closed jaw inner faces sit at y = +/-JAW_FACE
NOSE_X = 0.078          # blunt squared nose tip
OPEN_LIMIT = math.radians(30.0)

# Slip-joint slot dimensions
SLOT_LENGTH = 0.008     # slot travel length along X
SLOT_HALF_W = 0.0045    # slot half-width (the pin slides in this channel)
SLOT_X_CENTER = 0.0     # slot centered at origin

# Prismatic travel for the slip joint
SLIP_LOWER = -SLOT_LENGTH / 2.0
SLIP_UPPER = SLOT_LENGTH / 2.0

TANG_HALF_W = 0.004     # steel handle tang half width in plan

# Steel handle tang centerline in the tool plane (for the half whose jaw is +Y).
TANG_PTS = [
    (-0.010, -0.004),
    (-0.030, -0.011),
    (-0.070, -0.019),
    (-0.100, -0.024),
    (-0.118, -0.026),
]
# Extended centerline used by the over-molded grip (reaches the bulbous end).
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

# Red inlay stations along the outer/top face (stops before the black end bulb).
INLAY_XS = [-0.036, -0.055, -0.075, -0.095, -0.112, -0.118]
INLAY_HALF_H = 0.0075
INLAY_Z_CENTER = 0.006


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


def _grip_w(x: float) -> float:
    return _interp(x, [(sx, w) for sx, w, _h in GRIP_SECTIONS])


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
    """Material removed at the pivot so this half keeps only its lap layer.

    s=+1 keeps the lower lap (z <= -EPS); s=-1 keeps the upper lap (z >= +EPS).
    """
    cut = cq.Workplane("XY").circle(LAP_R).extrude(0.012)
    if s > 0:
        return cut.translate((0.0, 0.0, -EPS))
    return cut.translate((0.0, 0.0, -0.012 + EPS))


def _slot_cut() -> cq.Workplane:
    """Elongated racetrack slot cut through half_0's hub for the slip joint.

    The slot runs along X with rounded ends, passing fully through the plate.
    """
    # Racetrack: rectangle with semicircle caps
    slot_x_min = SLOT_X_CENTER - SLOT_LENGTH / 2.0
    slot_x_max = SLOT_X_CENTER + SLOT_LENGTH / 2.0
    hw = SLOT_HALF_W
    # Build as a rectangle unioned with two end circles
    slot = (
        cq.Workplane("XY")
        .rect(slot_x_max - slot_x_min, 2.0 * hw)
        .extrude(0.025)
        .translate((SLOT_X_CENTER, 0.0, -0.0125))
    )
    cap_l = (
        cq.Workplane("XY")
        .circle(hw)
        .extrude(0.025)
        .translate((slot_x_min, 0.0, -0.0125))
    )
    cap_r = (
        cq.Workplane("XY")
        .circle(hw)
        .extrude(0.025)
        .translate((slot_x_max, 0.0, -0.0125))
    )
    return slot.union(cap_l).union(cap_r)


def _jaw_solid(s: int) -> cq.Workplane:
    """Broad jaw: squared nose, prominent serrated gripping teeth, pipe-grip recess, cutter notch."""
    profile = [
        (0.010, JAW_FACE),
        (NOSE_X, JAW_FACE),
        (NOSE_X, 0.0110),   # blunt squared nose tip
        (0.068, 0.0125),
        (0.054, 0.0142),
        (0.040, 0.0158),
        (0.028, 0.0168),
        (0.016, 0.0172),
        (0.009, 0.0155),
    ]
    pts = [(x, s * y) for x, y in profile]
    jaw = cq.Workplane("XY").polyline(pts).close().extrude(HALF_T, both=True)

    # Prominent serrated gripping teeth on the inner face near the nose.
    # Rows of small V-groove cuts creating tooth ridges.
    for row in range(9):
        xi = 0.042 + 0.0035 * row
        # Horizontal groove cuts creating raised tooth ridges
        groove = (
            cq.Workplane("XY")
            .box(0.0018, 0.003, 0.020)
            .translate((xi, s * JAW_FACE, 0.0))
        )
        jaw = jaw.cut(groove)

    # Cross-serrations: perpendicular grooves creating a crosshatch tooth pattern
    for col in range(4):
        zi = -0.006 + 0.004 * col
        cross_groove = (
            cq.Workplane("XY")
            .box(0.028, 0.002, 0.0012)
            .translate((0.056, s * (JAW_FACE + 0.001), zi))
        )
        jaw = jaw.cut(cross_groove)

    # Rounded pipe-grip recess behind the nose (broader for lineman style).
    recess = (
        cq.Workplane("XY")
        .circle(0.005)
        .extrude(0.011, both=True)
        .translate((0.030, s * JAW_FACE, 0.0))
    )
    jaw = jaw.cut(recess)

    # Serrated scallop teeth around the pipe-grip recess rim.
    for ang_deg in (30.0, 60.0, 90.0, 120.0, 150.0):
        a = math.radians(ang_deg)
        sx = 0.030 + 0.005 * math.cos(a)
        sy = s * (JAW_FACE + 0.005 * math.sin(a))
        scallop = (
            cq.Workplane("XY")
            .circle(0.0009)
            .extrude(0.011, both=True)
            .translate((sx, sy, 0.0))
        )
        jaw = jaw.cut(scallop)

    # Wire-cutter V-notch between the pipe grip and the pivot (prominent).
    notch_pts = [
        (0.014, s * -0.001),
        (0.022, s * -0.001),
        (0.018, s * 0.004),
    ]
    notch = cq.Workplane("XY").polyline(notch_pts).close().extrude(0.011, both=True)
    jaw = jaw.cut(notch)

    return jaw.cut(_lap_cut(s))


def _hub_solid_half0() -> cq.Workplane:
    """Forged hub for half_0 with the visible slip-joint slot cut through."""
    hub = cq.Workplane("XY").circle(HUB_R).extrude(HALF_T, both=True)
    hub = hub.cut(_lap_cut(1))  # keep lower lap
    # Cut the elongated slip-joint slot through the hub
    hub = hub.cut(_slot_cut())
    return hub


def _hub_solid_half1() -> cq.Workplane:
    """Forged hub for half_1 (moving half, no slot, round pivot hole)."""
    hub = cq.Workplane("XY").circle(HUB_R).extrude(HALF_T, both=True)
    hub = hub.cut(_lap_cut(-1))  # keep upper lap
    return hub


def _shank_solid(s: int) -> cq.Workplane:
    """Steel handle tang sweeping back from the hub into the grip."""
    pts = [(x, s * y) for x, y in TANG_PTS]
    loop = _strip_loop(pts, TANG_HALF_W)
    shank = cq.Workplane("XY").polyline(loop).close().extrude(HALF_T, both=True)
    return shank.cut(_lap_cut(s))


def _ellipse_wire(x: float, yc: float, zc: float, w: float, h: float) -> cq.Wire:
    wp = cq.Workplane("YZ", origin=(x, 0.0, 0.0)).center(yc, zc).ellipse(w, h)
    return wp.val()


def _grip_solid(s: int) -> cq.Solid:
    """Soft-touch rubber grip: flared guard, curved shaft, bulbous end."""
    wires = [
        _ellipse_wire(x, s * _yc(x), 0.0, w, h)
        for x, w, h in GRIP_SECTIONS
    ]
    return cq.Solid.makeLoft(wires, ruled=False)


def _inlay_solid(s: int) -> cq.Solid:
    """Glossy red inlay loft running along the outer/top face of the grip."""
    wires = [
        _ellipse_wire(x, s * _yc(x), INLAY_Z_CENTER, _grip_w(x) - 0.0012, INLAY_HALF_H)
        for x in INLAY_XS
    ]
    return cq.Solid.makeLoft(wires, ruled=False)


def _pivot_slider_boss() -> cq.Workplane:
    """Rivet boss disc on the bottom face of the slider (visible under the tool)."""
    return cq.Workplane("XY").circle(BOSS_R).extrude(BOSS_H)


def _pivot_slider_shaft() -> cq.Workplane:
    """Rivet shaft passing through both halves and the slot."""
    total_h = 2.0 * HALF_T + 2.0 * SEAM_H + 2.0 * BOSS_H + 0.001
    return cq.Workplane("XY").circle(RIVET_R).extrude(total_h)


def _pivot_slider_head() -> cq.Workplane:
    """Rivet head disc on the top face of the slider (visible above the tool)."""
    return cq.Workplane("XY").circle(BOSS_R).extrude(BOSS_H)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slip_joint_pliers")

    steel_polished = model.material("steel_polished", rgba=(0.80, 0.81, 0.84, 1.0))
    steel_forged = model.material("steel_forged", rgba=(0.66, 0.67, 0.70, 1.0))
    steel_brushed = model.material("steel_brushed", rgba=(0.61, 0.62, 0.65, 1.0))
    seam_gray = model.material("seam_gray", rgba=(0.45, 0.46, 0.48, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.08, 0.09, 1.0))
    grip_red = model.material("grip_red", rgba=(0.80, 0.09, 0.11, 1.0))

    # ---- half_0: fixed half with the slip-joint slot ----
    half_0 = model.part("plier_half_0")

    half_0.visual(
        mesh_from_cadquery(_jaw_solid(1), "half0_jaw", tolerance=0.0002),
        name="jaw",
        material=steel_polished,
    )
    half_0.visual(
        mesh_from_cadquery(_hub_solid_half0(), "half0_hub", tolerance=0.0002),
        name="hub",
        material=steel_forged,
    )
    half_0.visual(
        mesh_from_cadquery(_shank_solid(1), "half0_shank", tolerance=0.0002),
        name="shank",
        material=steel_forged,
    )
    half_0.visual(
        mesh_from_cadquery(_grip_solid(1), "half0_grip", tolerance=0.0002),
        name="grip",
        material=rubber_black,
    )
    half_0.visual(
        mesh_from_cadquery(_inlay_solid(1), "half0_inlay", tolerance=0.0002),
        name="grip_inlay",
        material=grip_red,
    )

    # ---- pivot_slider: the captured pin that slides in the slot ----
    pivot_slider = model.part("pivot_slider")

    # Bottom seam ring
    pivot_slider.visual(
        Cylinder(SEAM_R, SEAM_H),
        origin=Origin(xyz=(0.0, 0.0, -(HALF_T + SEAM_H / 2.0))),
        name="boss_seam",
        material=seam_gray,
    )
    # Bottom boss disc
    pivot_slider.visual(
        mesh_from_cadquery(
            _pivot_slider_boss().translate((0.0, 0.0, -(HALF_T + SEAM_H + BOSS_H / 2.0))),
            "slider_boss",
            tolerance=0.0002,
        ),
        name="rivet_boss",
        material=steel_brushed,
    )
    # Rivet shaft through the stack
    shaft_total = 2.0 * HALF_T + 2.0 * SEAM_H + 2.0 * BOSS_H + 0.001
    pivot_slider.visual(
        Cylinder(RIVET_R, shaft_total),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="rivet_shaft",
        material=steel_brushed,
    )
    # Top seam ring
    pivot_slider.visual(
        Cylinder(SEAM_R, SEAM_H),
        origin=Origin(xyz=(0.0, 0.0, HALF_T + SEAM_H / 2.0)),
        name="head_seam",
        material=seam_gray,
    )
    # Top boss/head disc
    pivot_slider.visual(
        mesh_from_cadquery(
            _pivot_slider_head().translate((0.0, 0.0, HALF_T + SEAM_H)),
            "slider_head",
            tolerance=0.0002,
        ),
        name="rivet_head",
        material=steel_brushed,
    )

    # ---- half_1: moving half that rotates about the pivot pin ----
    half_1 = model.part("plier_half_1")

    half_1.visual(
        mesh_from_cadquery(_jaw_solid(-1), "half1_jaw", tolerance=0.0002),
        name="jaw",
        material=steel_polished,
    )
    half_1.visual(
        mesh_from_cadquery(_hub_solid_half1(), "half1_hub", tolerance=0.0002),
        name="hub",
        material=steel_forged,
    )
    half_1.visual(
        mesh_from_cadquery(_shank_solid(-1), "half1_shank", tolerance=0.0002),
        name="shank",
        material=steel_forged,
    )
    half_1.visual(
        mesh_from_cadquery(_grip_solid(-1), "half1_grip", tolerance=0.0002),
        name="grip",
        material=rubber_black,
    )
    half_1.visual(
        mesh_from_cadquery(_inlay_solid(-1), "half1_inlay", tolerance=0.0002),
        name="grip_inlay",
        material=grip_red,
    )

    # ---- Articulations ----

    # Slip joint: prismatic along the slot (X axis).
    # Positive q slides the pivot pin toward +X (toward the jaws).
    model.articulation(
        "slip_joint",
        ArticulationType.PRISMATIC,
        parent=half_0,
        child=pivot_slider,
        origin=Origin(xyz=(SLOT_X_CENTER, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.1,
            lower=SLIP_LOWER,
            upper=SLIP_UPPER,
        ),
    )

    # Pivot: revolute about Z (perpendicular to tool plane).
    # Positive q (about -Z) opens the jaws while handles spread.
    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=pivot_slider,
        child=half_1,
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

    half0 = object_model.get_part("plier_half_0")
    half1 = object_model.get_part("plier_half_1")
    slider = object_model.get_part("pivot_slider")
    slip = object_model.get_articulation("slip_joint")
    pivot = object_model.get_articulation("pivot")

    jaw0 = half0.get_visual("jaw")
    jaw1 = half1.get_visual("jaw")
    hub0 = half0.get_visual("hub")
    hub1 = half1.get_visual("hub")

    # ---- Joint contract checks ----

    # Slip joint: prismatic along X with the slot travel range.
    slip_limits = slip.motion_limits
    ctx.check(
        "slip_joint is prismatic with slot-range travel",
        slip.articulation_type == ArticulationType.PRISMATIC
        and slip_limits is not None
        and slip_limits.lower is not None
        and slip_limits.upper is not None
        and abs(slip_limits.lower - SLIP_LOWER) < 1e-6
        and abs(slip_limits.upper - SLIP_UPPER) < 1e-6,
        details=f"limits={slip_limits}",
    )

    # Pivot: revolute 0..30 degrees.
    pivot_limits = pivot.motion_limits
    ctx.check(
        "pivot is a 0..30 degree revolute joint",
        pivot.articulation_type == ArticulationType.REVOLUTE
        and pivot_limits is not None
        and pivot_limits.lower is not None
        and pivot_limits.upper is not None
        and abs(pivot_limits.lower) < 1e-9
        and abs(pivot_limits.upper - OPEN_LIMIT) < 1e-6,
        details=f"limits={pivot_limits}",
    )

    # ---- Slip joint slot geometry ----

    # The slot in half_0's hub is visible: hub has a through-hole elongated along X.
    hub0_aabb = ctx.part_element_world_aabb(half0, elem="hub")
    if hub0_aabb is not None:
        hub_dx = hub0_aabb[1][0] - hub0_aabb[0][0]
        ctx.check(
            "half_0 hub spans the slot length plus material on both sides",
            hub_dx >= 2.0 * HUB_R - 0.002,
            details=f"hub_dx={hub_dx:.4f}",
        )

    # ---- Closed rest pose: serrated jaw inner faces nearly touch ----
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

    # ---- Hub overlap: the two hub laps share the pivot footprint ----
    ctx.expect_overlap(
        half0,
        half1,
        axes="xy",
        elem_a=hub0,
        elem_b=hub1,
        min_overlap=0.02,
        name="hub laps share the pivot footprint",
    )

    # ---- Half-lap interleaving: the two forged halves pass over each other ----
    # At the pivot, half_0 keeps the lower lap and half_1 keeps the upper lap.
    # Where the shank of one half enters the hub zone of the other, a thin
    # crescent of intentional overlap exists at the half-lap joint boundary.
    ctx.allow_overlap(
        half0,
        half1,
        elem_a="hub",
        elem_b="shank",
        reason="The forged halves interleave at the half-lap pivot joint: "
        "half_0's lower-lap hub overlaps half_1's upper-lap shank entry "
        "in a thin crescent at the lap boundary.",
    )
    ctx.allow_overlap(
        half1,
        half0,
        elem_a="hub",
        elem_b="shank",
        reason="Symmetric half-lap interleaving: half_1's upper-lap hub "
        "overlaps half_0's lower-lap shank entry at the pivot boundary.",
    )
    # Proof: the hubs properly stack with half_1 above half_0.
    ctx.expect_gap(
        half1,
        half0,
        axis="z",
        positive_elem=hub1,
        negative_elem=hub0,
        min_gap=0.0,
        max_gap=0.001,
        name="half-lap: moving hub lap stacks above fixed hub lap",
    )

    # ---- Pivot slider is sandwiched between the halves ----
    # The rivet shaft passes through both halves' hubs.
    ctx.allow_overlap(
        slider,
        half0,
        elem_a="rivet_shaft",
        elem_b=hub0,
        reason="The rivet shaft passes through the slot in half_0's hub, "
        "capturing it for the slip-joint mechanism.",
    )
    ctx.allow_overlap(
        slider,
        half1,
        elem_a="rivet_shaft",
        elem_b=hub1,
        reason="The rivet shaft passes through half_1's hub lap, "
        "capturing it for the pivot mechanism.",
    )

    # Rivet shaft centered in both hubs (XY).
    ctx.expect_within(
        slider,
        half0,
        axes="xy",
        inner_elem="rivet_shaft",
        outer_elem=hub0,
        margin=0.001,
        name="rivet shaft centered in half_0 hub slot",
    )
    ctx.expect_within(
        slider,
        half1,
        axes="xy",
        inner_elem="rivet_shaft",
        outer_elem=hub1,
        margin=0.001,
        name="rivet shaft centered in half_1 hub",
    )

    # ---- Serrated teeth geometry present on the inner jaw faces ----
    # The jaw visuals include the tooth groove cuts; verify the jaw extends
    # to the inner face plane (serrations don't remove all inner material).
    jaw0_aabb = ctx.part_element_world_aabb(half0, elem="jaw")
    jaw1_aabb = ctx.part_element_world_aabb(half1, elem="jaw")
    if jaw0_aabb is not None and jaw1_aabb is not None:
        # Jaw tips reach the nose position
        ctx.check(
            "half_0 jaw reaches the nose tip",
            jaw0_aabb[1][0] >= NOSE_X - 0.003,
            details=f"jaw0_max_x={jaw0_aabb[1][0]:.4f}",
        )
        ctx.check(
            "half_1 jaw reaches the nose tip",
            jaw1_aabb[1][0] >= NOSE_X - 0.003,
            details=f"jaw1_max_x={jaw1_aabb[1][0]:.4f}",
        )
        # Jaw width spans the serration zone
        jaw0_dy = jaw0_aabb[1][1] - jaw0_aabb[0][1]
        ctx.check(
            "jaw has broad profile with serration zone",
            jaw0_dy >= 0.010,
            details=f"jaw0_dy={jaw0_dy:.4f}",
        )

    # ---- Boss diameter ~25 mm ----
    boss_aabb = ctx.part_element_world_aabb(slider, elem="rivet_boss")
    head_aabb = ctx.part_element_world_aabb(slider, elem="rivet_head")
    if boss_aabb is not None and head_aabb is not None:
        dia = boss_aabb[1][0] - boss_aabb[0][0]
        ctx.check(
            "rivet boss is ~25 mm diameter",
            0.023 <= dia <= 0.027,
            details=f"dia={dia:.4f}",
        )
    else:
        ctx.fail("boss AABBs resolve", "missing rivet_boss/rivet_head element AABB")

    # ---- Overall envelope ----
    a0 = ctx.part_world_aabb(half0)
    a1 = ctx.part_world_aabb(half1)
    g0 = ctx.part_element_world_aabb(half0, elem="grip")
    g1 = ctx.part_element_world_aabb(half1, elem="grip")
    ok_env = a0 is not None and a1 is not None and g0 is not None and g1 is not None
    if ok_env:
        length = max(a0[1][0], a1[1][0]) - min(a0[0][0], a1[0][0])
        across = g1[1][1] - g0[0][1]
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

    # ---- Slip joint travel: slider actually moves along X ----
    rest_slider_pos = ctx.part_world_position(slider)
    with ctx.pose({slip: SLIP_UPPER}):
        moved_slider_pos = ctx.part_world_position(slider)
        ctx.check(
            "slip joint translates the pivot slider along +X",
            rest_slider_pos is not None
            and moved_slider_pos is not None
            and moved_slider_pos[0] > rest_slider_pos[0] + 0.003,
            details=f"rest={rest_slider_pos}, moved={moved_slider_pos}",
        )

    # ---- Open pose: jaws separate and handles spread ----
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

    # ---- Combined slip + pivot pose: slider at far slot end, jaws open ----
    with ctx.pose({slip: SLIP_UPPER, pivot: OPEN_LIMIT}):
        ctx.expect_gap(
            half0,
            half1,
            axis="y",
            positive_elem=jaw0,
            negative_elem=jaw1,
            min_gap=0.004,
            name="jaws still open at max slip + max pivot pose",
        )

    return ctx.report()


object_model = build_object_model()
