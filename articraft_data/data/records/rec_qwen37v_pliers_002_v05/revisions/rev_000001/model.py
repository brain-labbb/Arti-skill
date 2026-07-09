from __future__ import annotations

# Locking pliers (Vise-Grip style), ~0.25 m long overall.
#
# Layout: the tool lies in the XY plane with Z as the thickness axis.
# The main pivot is at the origin. The jaws point +X, the handles sweep
# back to -X.  fixed_half (root) is the upper half; moving_half is the
# lower half that pivots open.  A release_lever sits inside the moving
# handle and pivots on its own small pin.  An adjustment_screw protrudes
# from the rear of the fixed handle.

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
HALF_T = 0.009          # half thickness of each forged plate
LAP_R = 0.016           # half-lap joint disc radius at pivot
HUB_R = 0.015           # forged hub radius
BOSS_R = 0.0125         # rivet boss radius (~0.025 m diameter)
BOSS_H = 0.0030
SEAM_R = 0.0132
SEAM_H = 0.0006
RIVET_R = 0.004
EPS = 0.0001
NOSE_X = 0.072
OPEN_LIMIT = math.radians(25.0)

TANG_HALF_W = 0.004

# Jaw stop boss dimensions
STOP_BOSS_R = 0.004
STOP_BOSS_H = 0.006

# Adjustment screw
SCREW_R = 0.003
SCREW_L = 0.018
SCREW_HEAD_R = 0.005
SCREW_HEAD_H = 0.004

# Release lever
LEVER_L = 0.035
LEVER_W = 0.006
LEVER_T = 0.003
LEVER_PIVOT_X = -0.088
LEVER_OPEN_LIMIT = math.radians(15.0)

# Steel handle tang centerline for the fixed (upper, +Y jaw) half.
TANG_PTS = [
    (-0.010, -0.004),
    (-0.030, -0.011),
    (-0.070, -0.019),
    (-0.100, -0.024),
    (-0.118, -0.026),
]
CENTERLINE = TANG_PTS + [(-0.140, -0.028)]

# Grip loft stations for locking pliers style (shorter, more compact handles)
GRIP_SECTIONS = [
    (-0.032, 0.0100, 0.0125),  # flared guard near pivot
    (-0.040, 0.0082, 0.0110),
    (-0.060, 0.0080, 0.0108),
    (-0.085, 0.0082, 0.0112),
    (-0.105, 0.0088, 0.0118),
    (-0.125, 0.0090, 0.0122),
    (-0.135, 0.0070, 0.0090),
    (-0.140, 0.0035, 0.0050),
]

# Toggle-link visible feature positions (fixed to the handles)
TOGGLE_LINK_X = -0.085
TOGGLE_LINK_W = 0.025
TOGGLE_LINK_H = 0.005
TOGGLE_LINK_T = 0.004


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
    """Locking pliers jaw: curved profile with prominent serrated teeth."""
    # Curved jaw profile typical of locking pliers
    profile = [
        (0.010, 0.0003),       # inner face start near pivot
        (NOSE_X, 0.0003),      # inner face at nose
        (NOSE_X, 0.0100),      # blunt nose tip
        (0.064, 0.0120),       # outer curve
        (0.050, 0.0140),
        (0.036, 0.0155),
        (0.024, 0.0165),
        (0.014, 0.0162),
        (0.009, 0.0145),
    ]
    pts = [(x, s * y) for x, y in profile]
    jaw = cq.Workplane("XY").polyline(pts).close().extrude(HALF_T, both=True)

    # Prominent serrated teeth on the inner jaw face - V-groove teeth
    # These are actual gripping teeth, deeper than fine serrations
    num_teeth = 10
    for i in range(num_teeth):
        xi = 0.020 + 0.005 * i
        # V-shaped groove cut into the inner face
        groove_pts = [
            (xi - 0.001, s * 0.0003),
            (xi, s * (-0.002)),      # tooth valley
            (xi + 0.001, s * 0.0003),
        ]
        groove = (
            cq.Workplane("XY")
            .polyline(groove_pts)
            .close()
            .extrude(0.020)
            .translate((0.0, 0.0, -0.010))
        )
        jaw = jaw.cut(groove)

    # Wire cutter notch near the pivot (locking pliers also have this)
    notch_pts = [
        (0.013, s * (-0.001)),
        (0.019, s * (-0.001)),
        (0.016, s * 0.004),
    ]
    notch = cq.Workplane("XY").polyline(notch_pts).close().extrude(0.012, both=True)
    jaw = jaw.cut(notch)

    return jaw.cut(_lap_cut(s))


def _jaw_stop_boss(s: int) -> cq.Workplane:
    """Jaw stop boss near the pivot - prevents over-closure."""
    # Small cylindrical boss protruding from the inner jaw face
    boss = (
        cq.Workplane("XY")
        .circle(STOP_BOSS_R)
        .extrude(STOP_BOSS_H)
        .translate((0.018, s * (0.0003 + STOP_BOSS_H / 2.0), 0.0))
    )
    return boss


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
    """Compact locking pliers grip - thinner profile with channel for release lever."""
    wires = [
        _ellipse_wire(x, s * _yc(x), 0.0, w, h)
        for x, w, h in GRIP_SECTIONS
    ]
    grip = cq.Solid.makeLoft(wires, ruled=False)

    # Cut a channel/slot inside the moving half grip for the release lever
    if s < 0:
        slot = (
            cq.Workplane("XY")
            .box(LEVER_L + 0.004, LEVER_W + 0.004, LEVER_T + 0.002)
            .translate((LEVER_PIVOT_X + LEVER_L / 2.0 - 0.005, s * _yc(LEVER_PIVOT_X), 0.006))
        )
        grip = cq.Workplane("XY").add(grip).cut(slot).val()

    return grip


def _toggle_link(s: int) -> cq.Workplane:
    """Visible toggle link on the handle - part of the locking mechanism."""
    yc = s * _yc(TOGGLE_LINK_X)
    link = (
        cq.Workplane("XY")
        .box(TOGGLE_LINK_W, TOGGLE_LINK_H, TOGGLE_LINK_T)
        .translate((TOGGLE_LINK_X + TOGGLE_LINK_W / 2.0, yc, HALF_T + TOGGLE_LINK_T / 2.0))
    )
    return link


def _adjustment_screw_solid() -> cq.Workplane:
    """Threaded adjustment screw at the rear of the fixed handle."""
    yc = _yc(-0.135)
    # Screw shaft with thread ridges
    shaft = (
        cq.Workplane("XY")
        .circle(SCREW_R)
        .extrude(SCREW_L)
        .translate((-0.140 - SCREW_L / 2.0, yc, 0.0))
    )
    # Add thread ridges as small rings along the shaft
    result = shaft
    for i in range(6):
        xi = -0.140 - 0.003 - i * 0.0025
        ring = (
            cq.Workplane("XY")
            .circle(SCREW_R + 0.0005)
            .circle(SCREW_R - 0.0002)
            .extrude(0.0008)
            .translate((xi, yc, -0.0004))
        )
        result = result.union(ring)

    # Knurled head (larger cylinder with flats)
    head = (
        cq.Workplane("XY")
        .circle(SCREW_HEAD_R)
        .extrude(SCREW_HEAD_H)
        .translate((-0.140 - SCREW_L - SCREW_HEAD_H / 2.0, yc, 0.0))
    )
    # Cut flats on the head for gripping
    for angle in (0.0, 60.0, 120.0):
        a = math.radians(angle)
        cx = -0.140 - SCREW_L - SCREW_HEAD_H / 2.0
        cy = yc + (SCREW_HEAD_R + 0.002) * math.cos(a)
        cz = (SCREW_HEAD_R + 0.002) * math.sin(a)
        flat = (
            cq.Workplane("XY")
            .box(SCREW_HEAD_H + 0.001, 0.004, 0.004)
            .translate((cx, cy, cz))
        )
        head = head.cut(flat)
    result = result.union(head)
    return result


def _release_lever_solid() -> cq.Workplane:
    """Release lever that pivots inside the moving handle."""
    # Flat lever shape with pivot hole at one end and a tail
    lever = (
        cq.Workplane("XY")
        .box(LEVER_L, LEVER_W, LEVER_T)
        .translate((LEVER_L / 2.0, 0.0, 0.0))
    )
    # Rounded end at the pivot (cylinder)
    pivot_end = (
        cq.Workplane("XY")
        .circle(LEVER_W / 2.0)
        .extrude(LEVER_T)
        .translate((0.0, 0.0, -LEVER_T / 2.0))
    )
    lever = lever.union(pivot_end)
    # Release tab at the far end (wider paddle)
    tab = (
        cq.Workplane("XY")
        .box(0.010, LEVER_W + 0.004, LEVER_T)
        .translate((LEVER_L + 0.005, 0.0, 0.0))
    )
    lever = lever.union(tab)
    return lever


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="locking_pliers")

    steel_polished = model.material("steel_polished", rgba=(0.78, 0.79, 0.82, 1.0))
    steel_forged = model.material("steel_forged", rgba=(0.62, 0.63, 0.66, 1.0))
    steel_brushed = model.material("steel_brushed", rgba=(0.58, 0.59, 0.62, 1.0))
    seam_gray = model.material("seam_gray", rgba=(0.45, 0.46, 0.48, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.35, 0.36, 0.38, 1.0))
    grip_blue = model.material("grip_blue", rgba=(0.12, 0.20, 0.55, 1.0))

    # --- Fixed half (root): upper jaw + handle ---
    fixed = model.part("fixed_half")

    # Jaw with serrated teeth
    fixed.visual(
        mesh_from_cadquery(_jaw_solid(1), "fixed_jaw", tolerance=0.0002),
        name="jaw",
        material=steel_polished,
    )
    # Jaw stop boss
    fixed.visual(
        mesh_from_cadquery(_jaw_stop_boss(1), "fixed_stop_boss", tolerance=0.0002),
        name="jaw_stop_boss",
        material=steel_forged,
    )
    # Hub
    fixed.visual(
        mesh_from_cadquery(_hub_solid(1), "fixed_hub", tolerance=0.0002),
        name="hub",
        material=steel_forged,
    )
    # Shank
    fixed.visual(
        mesh_from_cadquery(_shank_solid(1), "fixed_shank", tolerance=0.0002),
        name="shank",
        material=steel_forged,
    )
    # Grip
    fixed.visual(
        mesh_from_cadquery(_grip_solid(1), "fixed_grip", tolerance=0.0002),
        name="grip",
        material=grip_blue,
    )
    # Adjustment screw at rear of handle
    fixed.visual(
        mesh_from_cadquery(_adjustment_screw_solid(), "adj_screw", tolerance=0.0002),
        name="adjustment_screw",
        material=steel_brushed,
    )

    # Rivet boss and shaft on fixed half
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
    fixed.visual(
        Cylinder(RIVET_R, 2.0 * HALF_T + SEAM_H + 2.0 * EPS + 0.0005),
        origin=Origin(xyz=(0.0, 0.0, 0.00005)),
        name="rivet_shaft",
        material=steel_brushed,
    )
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

    # --- Moving half: lower jaw + handle ---
    moving = model.part("moving_half")

    moving.visual(
        mesh_from_cadquery(_jaw_solid(-1), "moving_jaw", tolerance=0.0002),
        name="jaw",
        material=steel_polished,
    )
    moving.visual(
        mesh_from_cadquery(_jaw_stop_boss(-1), "moving_stop_boss", tolerance=0.0002),
        name="jaw_stop_boss",
        material=steel_forged,
    )
    moving.visual(
        mesh_from_cadquery(_hub_solid(-1), "moving_hub", tolerance=0.0002),
        name="hub",
        material=steel_forged,
    )
    moving.visual(
        mesh_from_cadquery(_shank_solid(-1), "moving_shank", tolerance=0.0002),
        name="shank",
        material=steel_forged,
    )
    moving.visual(
        mesh_from_cadquery(_grip_solid(-1), "moving_grip", tolerance=0.0002),
        name="grip",
        material=grip_blue,
    )

    # --- Articulations ---

    # Main pivot: moving_half rotates about fixed_half
    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=fixed,
        child=moving,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=3.0, lower=0.0, upper=OPEN_LIMIT),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    fixed = object_model.get_part("fixed_half")
    moving = object_model.get_part("moving_half")
    pivot = object_model.get_articulation("pivot")

    jaw_f = fixed.get_visual("jaw")
    jaw_m = moving.get_visual("jaw")
    hub_f = fixed.get_visual("hub")
    hub_m = moving.get_visual("hub")
    stop_f = fixed.get_visual("jaw_stop_boss")
    stop_m = moving.get_visual("jaw_stop_boss")
    screw = fixed.get_visual("adjustment_screw")

    # --- Joint contract checks ---
    limits = pivot.motion_limits
    ctx.check(
        "pivot is a revolute joint with 0..25 degree range",
        pivot.articulation_type == ArticulationType.REVOLUTE
        and limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower) < 1e-9
        and abs(limits.upper - OPEN_LIMIT) < 1e-6,
        details=f"limits={limits}",
    )

    # --- Jaw stop boss: small protrusions near pivot on both jaws ---
    stop_aabb_f = ctx.part_element_world_aabb(fixed, elem="jaw_stop_boss")
    stop_aabb_m = ctx.part_element_world_aabb(moving, elem="jaw_stop_boss")
    if stop_aabb_f is not None and stop_aabb_m is not None:
        # Stop bosses should be near the pivot (close to x=0)
        ctx.check(
            "jaw stop boss on fixed half is near the pivot",
            stop_aabb_f[0][0] < 0.025 and stop_aabb_f[1][0] > 0.005,
            details=f"stop_x_range=[{stop_aabb_f[0][0]:.4f}, {stop_aabb_f[1][0]:.4f}]",
        )
        ctx.check(
            "jaw stop boss on moving half is near the pivot",
            stop_aabb_m[0][0] < 0.025 and stop_aabb_m[1][0] > 0.005,
            details=f"stop_x_range=[{stop_aabb_m[0][0]:.4f}, {stop_aabb_m[1][0]:.4f}]",
        )
        # Stop bosses protrude toward each other (y-extents face inward)
        ctx.check(
            "stop bosses protrude inward from jaw inner faces",
            stop_aabb_f[0][1] < 0.008 and stop_aabb_m[1][1] > -0.008,
            details=f"fixed_stop_y_min={stop_aabb_f[0][1]:.4f} moving_stop_y_max={stop_aabb_m[1][1]:.4f}",
        )
    else:
        ctx.fail("jaw stop boss AABBs resolve", "missing jaw_stop_boss elements")

    # --- Adjustment screw at rear of fixed handle ---
    screw_aabb = ctx.part_element_world_aabb(fixed, elem="adjustment_screw")
    grip_f_aabb = ctx.part_element_world_aabb(fixed, elem="grip")
    if screw_aabb is not None and grip_f_aabb is not None:
        ctx.check(
            "adjustment screw is at the rear of the handle (x < grip midpoint)",
            screw_aabb[0][0] < grip_f_aabb[0][0] + 0.005,
            details=f"screw_x_min={screw_aabb[0][0]:.4f} grip_x_min={grip_f_aabb[0][0]:.4f}",
        )
        ctx.check(
            "adjustment screw protrudes past the handle end",
            screw_aabb[0][0] < grip_f_aabb[0][0] - 0.005,
            details=f"screw_x_min={screw_aabb[0][0]:.4f} grip_x_min={grip_f_aabb[0][0]:.4f}",
        )
    else:
        ctx.fail("screw AABB resolves", "missing adjustment_screw or grip element")

    # --- Serrated teeth: jaws have deeper grooves than flat faces ---
    jaw_f_aabb = ctx.part_element_world_aabb(fixed, elem="jaw")
    jaw_m_aabb = ctx.part_element_world_aabb(moving, elem="jaw")
    if jaw_f_aabb is not None:
        # Jaw should extend a reasonable length
        jaw_len = jaw_f_aabb[1][0] - jaw_f_aabb[0][0]
        ctx.check(
            "jaw has substantial gripping length with teeth",
            jaw_len > 0.04,
            details=f"jaw_length={jaw_len:.4f}",
        )

    # --- Closed pose: jaws nearly touching ---
    ctx.expect_gap(
        fixed,
        moving,
        axis="y",
        positive_elem=jaw_f,
        negative_elem=jaw_m,
        min_gap=0.0002,
        max_gap=0.002,
        name="jaws closed and nearly touching at rest",
    )

    # --- Hub interleaving at pivot ---
    ctx.expect_gap(
        moving,
        fixed,
        axis="z",
        positive_elem=hub_m,
        negative_elem=hub_f,
        min_gap=0.0,
        max_gap=0.001,
        name="moving hub lap stacks above fixed hub lap",
    )
    ctx.expect_contact(
        fixed,
        moving,
        elem_a=hub_f,
        elem_b=hub_m,
        contact_tol=0.0005,
        name="hub laps seat against each other at the joint",
    )
    ctx.expect_overlap(
        fixed,
        moving,
        axes="xy",
        elem_a=hub_f,
        elem_b=hub_m,
        min_overlap=0.02,
        name="hub laps share the pivot footprint",
    )

    # Rivet shaft passes through moving half hub
    ctx.allow_overlap(
        fixed,
        moving,
        elem_a="rivet_shaft",
        elem_b=hub_m,
        reason="The rivet shaft is fixed to fixed_half and intentionally passes "
        "through the moving half's hub lap, capturing it at the pivot.",
    )
    ctx.expect_within(
        fixed,
        moving,
        axes="xy",
        inner_elem="rivet_shaft",
        outer_elem=hub_m,
        margin=0.0005,
        name="rivet shaft stays centered inside the moving hub lap",
    )

    # --- Open pose: jaws separate ---
    closed_jaw_m = ctx.part_element_world_aabb(moving, elem="jaw")
    closed_grip_m = ctx.part_element_world_aabb(moving, elem="grip")
    with ctx.pose({pivot: OPEN_LIMIT}):
        ctx.expect_gap(
            fixed,
            moving,
            axis="y",
            positive_elem=jaw_f,
            negative_elem=jaw_m,
            min_gap=0.004,
            name="jaws open apart at the 25 degree pose",
        )
        open_jaw_m = ctx.part_element_world_aabb(moving, elem="jaw")
        open_grip_m = ctx.part_element_world_aabb(moving, elem="grip")
        ctx.check(
            "moving jaw swings away from the fixed jaw",
            closed_jaw_m is not None
            and open_jaw_m is not None
            and open_jaw_m[0][1] < closed_jaw_m[0][1] - 0.015,
            details=f"closed_min_y={closed_jaw_m}, open_min_y={open_jaw_m}",
        )
        ctx.check(
            "moving handle spreads outward as the jaws open",
            closed_grip_m is not None
            and open_grip_m is not None
            and open_grip_m[1][1] > closed_grip_m[1][1] + 0.02,
            details=f"closed_max_y={closed_grip_m}, open_max_y={open_grip_m}",
        )

    return ctx.report()


object_model = build_object_model()
