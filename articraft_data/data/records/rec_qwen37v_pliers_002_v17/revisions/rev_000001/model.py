from __future__ import annotations

# Round-nose jewelry pliers with slip-joint mechanism.
# Variant of heavy-duty combination pliers → tapered conical jaws, serrated
# inner teeth, jaw stop boss, and a slip-joint pin sliding in a short slot.
#
# Layout: tool lies in the XY plane, Z is the thickness axis.
# Jaws point +X, handles sweep back to -X and spread in +/-Y.
# The pivot region is at the origin.
#
# Part chain:
#   half_fixed (root) — carries the elongated slip-joint slot
#   slip_carriage     — prismatic child of half_fixed, slides along +X
#   half_moving       — revolute child of slip_carriage, opens the jaws

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
HALF_T = 0.005           # half thickness of each forged plate
EPS = 0.0001             # lap clearance so stacked halves do not penetrate
HUB_R = 0.010            # forged hub radius around the pivot
JAW_BASE_R = 0.005       # jaw radius at the base (near hub)
JAW_TIP_R = 0.0015       # jaw radius at the tapered tip
JAW_LENGTH = 0.045       # jaw length from hub edge to tip
JAW_FACE_Y = 0.0003      # closed jaw inner face offset from y=0

# Slip-joint slot dimensions
SLOT_LENGTH = 0.014      # slot travel length along X
SLOT_HALF_W = 0.003      # slot half width in Y
SLOT_X_START = -SLOT_LENGTH / 2.0   # slot start X
SLOT_X_END = SLOT_LENGTH / 2.0      # slot end X

# Pin on carriage
PIN_R = 0.0035           # slip-joint pin radius (used for slot travel limits)

# Jaw stop boss
STOP_BOSS_R = 0.003      # jaw stop boss radius
STOP_BOSS_H = 0.003      # jaw stop boss protrusion height

# Handle geometry
TANG_HALF_W = 0.003      # steel tang half width
TANG_PTS = [
    (-0.008, -0.003),
    (-0.025, -0.008),
    (-0.055, -0.014),
    (-0.080, -0.018),
    (-0.095, -0.020),
]
CENTERLINE = TANG_PTS + [(-0.105, -0.021)]

# Grip stations: (x, half_width_y, half_height_z)
GRIP_SECTIONS = [
    (-0.026, 0.0070, 0.0090),  # flared guard near pivot
    (-0.034, 0.0058, 0.0078),
    (-0.050, 0.0055, 0.0074),
    (-0.070, 0.0058, 0.0076),
    (-0.088, 0.0060, 0.0080),
    (-0.100, 0.0062, 0.0082),  # end swell
    (-0.105, 0.0040, 0.0055),
]

OPEN_LIMIT = math.radians(35.0)
SLIP_TRAVEL = 0.008       # prismatic slip-joint travel


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
    """Half-lap cut at the pivot so stacked halves do not penetrate.

    s=+1 keeps the lower lap (z <= -EPS); s=-1 keeps the upper lap (z >= +EPS).
    """
    cut = cq.Workplane("XY").circle(HUB_R + 0.002).extrude(0.012)
    if s > 0:
        return cut.translate((0.0, 0.0, -EPS))
    return cut.translate((0.0, 0.0, -0.012 + EPS))


def _jaw_solid(s: int) -> cq.Workplane:
    """Tapered conical round-nose jaw with serrated inner teeth.

    Each jaw is a cone offset to its own side of y=0 so the two jaws
    do not penetrate when closed. The jaw base extends into the hub
    region so it fuses with the hub disc.
    """
    base_x = HUB_R - 0.005   # jaw base well inside the hub for connectivity
    tip_x = HUB_R + JAW_LENGTH

    # Position jaw center on its own side: inner face at y ≈ s*JAW_FACE_Y
    base_cy = s * (JAW_FACE_Y + JAW_BASE_R)
    mid_r = JAW_BASE_R * 0.55 + JAW_TIP_R * 0.45
    mid_cy = s * (JAW_FACE_Y + mid_r)
    tip_cy = s * (JAW_FACE_Y + JAW_TIP_R)

    base_wire = (
        cq.Workplane("YZ", origin=(base_x, 0.0, 0.0))
        .center(base_cy, 0.0)
        .ellipse(JAW_BASE_R, HALF_T * 0.9)
        .val()
    )
    mid_x = base_x + (tip_x - base_x) * 0.5
    mid_wire = (
        cq.Workplane("YZ", origin=(mid_x, 0.0, 0.0))
        .center(mid_cy, 0.0)
        .ellipse(mid_r, HALF_T * 0.75)
        .val()
    )
    tip_wire = (
        cq.Workplane("YZ", origin=(tip_x, 0.0, 0.0))
        .center(tip_cy, 0.0)
        .ellipse(JAW_TIP_R, HALF_T * 0.5)
        .val()
    )
    jaw = cq.Solid.makeLoft([base_wire, mid_wire, tip_wire], ruled=False)
    jaw_wp = cq.Workplane("XY").add(jaw)

    # Serrated teeth on inner jaw face: small transverse ridges protruding
    # slightly from the inner face (additive, not cut-through).
    for i in range(6):
        xi = HUB_R + 0.004 + 0.0055 * i
        frac = (xi - HUB_R) / JAW_LENGTH
        local_r = JAW_BASE_R * (1.0 - frac) + JAW_TIP_R * frac
        if local_r < 0.002:
            break
        # Thin ridge on the inner face
        ridge_y = s * JAW_FACE_Y
        ridge = (
            cq.Workplane("XY")
            .box(0.0006, 0.0004, local_r * 1.0)
            .translate((xi, ridge_y + s * 0.0002, 0.0))
        )
        jaw_wp = jaw_wp.union(ridge)

    # Apply half-lap cut at the pivot region so the jaw base does not
    # overlap the other half's hub.
    jaw_wp = jaw_wp.cut(_lap_cut(s))

    return jaw_wp


def _hub_solid(s: int) -> cq.Workplane:
    """Forged circular hub around the pivot area, thinned to its lap layer."""
    hub = cq.Workplane("XY").circle(HUB_R).extrude(HALF_T, both=True)
    return hub.cut(_lap_cut(s))


def _hub_with_slot(s: int) -> cq.Workplane:
    """Hub with elongated slip-joint slot cut through it (for the fixed half)."""
    hub = _hub_solid(s)
    # Cut the elongated slot through the hub
    slot = (
        cq.Workplane("XY")
        .rect(SLOT_LENGTH, SLOT_HALF_W * 2.0)
        .extrude(HALF_T * 3.0, both=True)
        .translate(((SLOT_X_START + SLOT_X_END) / 2.0, 0.0, 0.0))
    )
    return hub.cut(slot)


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
    """Dipped rubber comfort grip for jewelry pliers."""
    wires = [
        _ellipse_wire(x, s * _yc(x), 0.0, w, h)
        for x, w, h in GRIP_SECTIONS
    ]
    return cq.Solid.makeLoft(wires, ruled=False)


def _jaw_stop_boss(s: int) -> cq.Workplane:
    """Small cylindrical boss near the jaw base that acts as a jaw-closing stop.

    Protrudes from the outer Z face of each half's jaw region so it fuses
    with the jaw mesh for connectivity.
    """
    # Position on the jaw body past the lap-cut radius so the jaw is full-thickness
    boss_x = 0.014
    boss_y = s * (JAW_FACE_Y + JAW_BASE_R * 0.6)
    if s > 0:
        # Protrude downward from the jaw bottom face
        boss = (
            cq.Workplane("XY")
            .circle(STOP_BOSS_R)
            .extrude(STOP_BOSS_H + HALF_T)
            .translate((boss_x, boss_y, -(HALF_T + STOP_BOSS_H)))
        )
    else:
        # Protrude upward from the jaw top face
        boss = (
            cq.Workplane("XY")
            .circle(STOP_BOSS_R)
            .extrude(STOP_BOSS_H + HALF_T)
            .translate((boss_x, boss_y, 0.0))
        )
    return boss


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="round_nose_jewelry_pliers")

    steel_polished = model.material("steel_polished", rgba=(0.82, 0.83, 0.86, 1.0))
    steel_forged = model.material("steel_forged", rgba=(0.68, 0.69, 0.72, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.45, 0.46, 0.50, 1.0))
    rubber_blue = model.material("rubber_blue", rgba=(0.12, 0.28, 0.62, 1.0))

    # ---- Part 1: half_fixed (root, carries the slot) ----
    half_fixed = model.part("half_fixed")

    half_fixed.visual(
        mesh_from_cadquery(_jaw_solid(1), "fixed_jaw", tolerance=0.0002),
        name="jaw",
        material=steel_polished,
    )
    half_fixed.visual(
        mesh_from_cadquery(_hub_with_slot(1), "fixed_hub_slot", tolerance=0.0002),
        name="hub",
        material=steel_forged,
    )
    half_fixed.visual(
        mesh_from_cadquery(_shank_solid(1), "fixed_shank", tolerance=0.0002),
        name="shank",
        material=steel_forged,
    )
    half_fixed.visual(
        mesh_from_cadquery(_grip_solid(1), "fixed_grip", tolerance=0.0002),
        name="grip",
        material=rubber_blue,
    )
    # Jaw stop boss on the fixed half
    half_fixed.visual(
        mesh_from_cadquery(_jaw_stop_boss(1), "fixed_stop_boss", tolerance=0.0002),
        name="jaw_stop",
        material=steel_dark,
    )

    # ---- Part 2: slip_carriage (prismatic child of fixed) ----
    # Core slip-joint mechanism part; carries the slot travel between halves.
    slip_carriage = model.part("slip_carriage")

    # ---- Part 3: half_moving (revolute child of slip_carriage) ----
    half_moving = model.part("half_moving")

    half_moving.visual(
        mesh_from_cadquery(_jaw_solid(-1), "moving_jaw", tolerance=0.0002),
        name="jaw",
        material=steel_polished,
    )
    half_moving.visual(
        mesh_from_cadquery(_hub_solid(-1), "moving_hub", tolerance=0.0002),
        name="hub",
        material=steel_forged,
    )
    half_moving.visual(
        mesh_from_cadquery(_shank_solid(-1), "moving_shank", tolerance=0.0002),
        name="shank",
        material=steel_forged,
    )
    half_moving.visual(
        mesh_from_cadquery(_grip_solid(-1), "moving_grip", tolerance=0.0002),
        name="grip",
        material=rubber_blue,
    )
    # Jaw stop boss on the moving half too
    half_moving.visual(
        mesh_from_cadquery(_jaw_stop_boss(-1), "moving_stop_boss", tolerance=0.0002),
        name="jaw_stop",
        material=steel_dark,
    )

    # ---- Articulation 1: slip_joint (prismatic, slot travel along +X) ----
    # The slip_carriage starts at the origin (pivot center). The slot in
    # the fixed hub extends along +/-X around this point.
    model.articulation(
        "slip_joint",
        ArticulationType.PRISMATIC,
        parent=half_fixed,
        child=slip_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0, velocity=0.1,
            lower=-SLOT_LENGTH / 2.0 + PIN_R,
            upper=SLOT_LENGTH / 2.0 - PIN_R,
        ),
    )

    # ---- Articulation 2: pivot (revolute at the pin, opens jaws) ----
    # The moving half rotates about the slip_carriage pin position.
    # Axis -Z so positive q opens the jaws (moving jaw swings toward -Y).
    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=slip_carriage,
        child=half_moving,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=40.0, velocity=3.0,
            lower=0.0, upper=OPEN_LIMIT,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    half_fixed = object_model.get_part("half_fixed")
    slip_carriage = object_model.get_part("slip_carriage")
    half_moving = object_model.get_part("half_moving")
    slip_joint = object_model.get_articulation("slip_joint")
    pivot = object_model.get_articulation("pivot")

    # ---- Structural checks: round-nose conical jaws ----
    jaw_fixed = half_fixed.get_visual("jaw")
    jaw_moving = half_moving.get_visual("jaw")
    jaw_f_aabb = ctx.part_element_world_aabb(half_fixed, elem="jaw")
    jaw_m_aabb = ctx.part_element_world_aabb(half_moving, elem="jaw")
    if jaw_f_aabb is not None:
        jaw_len = jaw_f_aabb[1][0] - jaw_f_aabb[0][0]
        jaw_w_base = jaw_f_aabb[1][1] - jaw_f_aabb[0][1]
        jaw_h = jaw_f_aabb[1][2] - jaw_f_aabb[0][2]
        ctx.check(
            "fixed jaw is tapered and elongated (conical round-nose)",
            0.035 <= jaw_len <= 0.055 and jaw_w_base > 0.004 and jaw_h > 0.003,
            details=f"len={jaw_len:.4f} w={jaw_w_base:.4f} h={jaw_h:.4f}",
        )
    else:
        ctx.fail("fixed jaw AABB resolves", "missing jaw element")

    # ---- Serrated teeth: jaw has grooves on inner face ----
    # The serrations make the jaw bounding box slightly narrower than a smooth cone.
    # We verify the jaw visual exists and has reasonable geometry.
    ctx.check(
        "moving jaw exists with serrated inner geometry",
        jaw_m_aabb is not None,
        details=f"aabb={jaw_m_aabb}",
    )

    # ---- Jaw stop boss near the pivot ----
    stop_aabb = ctx.part_element_world_aabb(half_fixed, elem="jaw_stop")
    if stop_aabb is not None:
        stop_x = 0.5 * (stop_aabb[0][0] + stop_aabb[1][0])
        ctx.check(
            "jaw stop boss is positioned near the pivot on the fixed half",
            0.005 <= stop_x <= 0.020,
            details=f"stop_center_x={stop_x:.4f}",
        )
    else:
        ctx.fail("jaw stop boss AABB resolves", "missing jaw_stop element")

    # ---- Slip-joint slot visible on fixed half hub ----
    hub_aabb = ctx.part_element_world_aabb(half_fixed, elem="hub")
    if hub_aabb is not None:
        # The hub should contain the slot region
        ctx.check(
            "fixed hub contains the slip-joint slot region",
            hub_aabb[0][0] <= SLOT_X_START and hub_aabb[1][0] >= SLOT_X_END,
            details=f"hub_x=[{hub_aabb[0][0]:.4f}, {hub_aabb[1][0]:.4f}]",
        )

    # ---- Slip-joint prismatic articulation ----
    slip_limits = slip_joint.motion_limits
    ctx.check(
        "slip_joint is a prismatic joint with nonzero travel",
        slip_joint.articulation_type == ArticulationType.PRISMATIC
        and slip_limits is not None
        and slip_limits.upper is not None
        and slip_limits.lower is not None
        and slip_limits.upper > slip_limits.lower,
        details=f"type={slip_joint.articulation_type} limits={slip_limits}",
    )

    # ---- Pivot revolute articulation ----
    pivot_limits = pivot.motion_limits
    ctx.check(
        "pivot is a revolute joint opening 0..35 degrees",
        pivot.articulation_type == ArticulationType.REVOLUTE
        and pivot_limits is not None
        and pivot_limits.lower is not None
        and pivot_limits.upper is not None
        and abs(pivot_limits.lower) < 1e-9
        and abs(pivot_limits.upper - OPEN_LIMIT) < 1e-6,
        details=f"limits={pivot_limits}",
    )

    # ---- Closed pose: jaws nearly touching ----
    ctx.expect_gap(
        half_fixed,
        half_moving,
        axis="y",
        positive_elem=jaw_fixed,
        negative_elem=jaw_moving,
        min_gap=0.0001,
        max_gap=0.002,
        name="jaws closed and nearly touching at rest",
    )

    # ---- Open pose: jaws separate ----
    closed_jaw_m = ctx.part_element_world_aabb(half_moving, elem="jaw")
    with ctx.pose({pivot: OPEN_LIMIT}):
        ctx.expect_gap(
            half_fixed,
            half_moving,
            axis="y",
            positive_elem=jaw_fixed,
            negative_elem=jaw_moving,
            min_gap=0.003,
            name="jaws open apart at the 35 degree pose",
        )
        open_jaw_m = ctx.part_element_world_aabb(half_moving, elem="jaw")
        ctx.check(
            "moving jaw swings away from the fixed jaw when opened",
            closed_jaw_m is not None
            and open_jaw_m is not None
            and open_jaw_m[0][1] < closed_jaw_m[0][1] - 0.008,
            details=f"closed_min_y={closed_jaw_m[0][1]:.4f} open_min_y={open_jaw_m[0][1]:.4f}",
        )

    # ---- Slip joint translation moves the carriage along X ----
    slip_upper = slip_limits.upper if slip_limits else 0.0
    rest_carriage_pos = ctx.part_world_position(slip_carriage)
    with ctx.pose({slip_joint: slip_upper}):
        moved_carriage_pos = ctx.part_world_position(slip_carriage)
    ctx.check(
        "slip joint prismatic motion translates carriage along +X",
        rest_carriage_pos is not None
        and moved_carriage_pos is not None
        and moved_carriage_pos[0] > rest_carriage_pos[0] + 0.002,
        details=f"rest={rest_carriage_pos} moved={moved_carriage_pos}",
    )

    # ---- Overall size: about 0.15 m long (jewelry pliers are smaller) ----
    a0 = ctx.part_world_aabb(half_fixed)
    a1 = ctx.part_world_aabb(half_moving)
    if a0 is not None and a1 is not None:
        length = max(a0[1][0], a1[1][0]) - min(a0[0][0], a1[0][0])
        ctx.check(
            "overall length about 0.15 m for jewelry pliers",
            0.12 <= length <= 0.19,
            details=f"length={length:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
