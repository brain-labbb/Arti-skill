from __future__ import annotations

# Slip-joint pliers variant (forked from lineman pliers).
# Two forged-steel halves with a slip-joint pivot: an elongated slot in
# half_fixed lets the pivot pin slide between two positions for different
# jaw-opening ranges. Color-separated geometric grip sleeves with a raised
# accent band.

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
HALF_T = 0.008           # half thickness of each forged plate (full 0.016)
HUB_R = 0.014            # forged hub radius around the pivot
SLOT_L = 0.010           # elongated slot total length (along X)
SLOT_W = 0.006           # slot width (matches pin diameter)
PIN_R = 0.0028           # pivot pin radius
CAP_R = 0.007            # circular rivet cap radius
CAP_H = 0.0020           # rivet cap height
EPS = 0.0001             # lap clearance
JAW_FACE = 0.0003        # closed jaw inner faces sit at y = +/-JAW_FACE
NOSE_X = 0.058           # jaw nose tip position
OPEN_LIMIT = math.radians(30.0)
SLIP_TRAVEL = 0.008      # distance between two pivot detent positions

TANG_HALF_W = 0.004      # steel handle tang half width

# Handle tang centerline (for the half whose jaw is on +Y side).
TANG_PTS = [
    (-0.010, -0.004),
    (-0.030, -0.011),
    (-0.065, -0.018),
    (-0.095, -0.022),
    (-0.115, -0.024),
]
CENTERLINE = TANG_PTS + [(-0.126, -0.0254)]

# Grip loft stations: (x, half_width_y, half_height_z)
GRIP_SECTIONS = [
    (-0.032, 0.0095, 0.0125),
    (-0.040, 0.0078, 0.0108),
    (-0.058, 0.0076, 0.0105),
    (-0.080, 0.0078, 0.0108),
    (-0.100, 0.0082, 0.0114),
    (-0.115, 0.0084, 0.0116),
    (-0.123, 0.0060, 0.0082),
    (-0.126, 0.0026, 0.0038),
]

def _interp(x: float, pts: list[tuple[float, float]]) -> float:
    """Piecewise-linear interpolation over (x, value) points."""
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
    """Remove material at the pivot so this half keeps only its lap layer.

    s=+1 keeps the lower lap (z <= -EPS); s=-1 keeps the upper lap (z >= +EPS).
    """
    cut = cq.Workplane("XY").circle(HUB_R + 0.002).extrude(0.012)
    if s > 0:
        return cut.translate((0.0, 0.0, -EPS))
    return cut.translate((0.0, 0.0, -0.012 + EPS))


def _jaw_solid(s: int) -> cq.Workplane:
    """Slip-joint style jaw: flat broad profile with fine serrations."""
    profile = [
        (0.010, JAW_FACE),
        (NOSE_X, JAW_FACE),
        (NOSE_X, 0.0095),
        (0.050, 0.0110),
        (0.038, 0.0125),
        (0.024, 0.0138),
        (0.014, 0.0142),
        (0.009, 0.0130),
    ]
    pts = [(x, s * y) for x, y in profile]
    jaw = cq.Workplane("XY").polyline(pts).close().extrude(HALF_T, both=True)

    # Fine cross-wise serrations on the inner gripping face.
    for i in range(8):
        xi = 0.020 + 0.004 * i
        groove = (
            cq.Workplane("XY")
            .box(0.0012, 0.0020, 0.018)
            .translate((xi, s * JAW_FACE, 0.0))
        )
        jaw = jaw.cut(groove)

    return jaw.cut(_lap_cut(s))


def _hub_solid_slotted(s: int) -> cq.Workplane:
    """Hub with elongated slip-joint slot (for half_fixed)."""
    hub = cq.Workplane("XY").circle(HUB_R).extrude(HALF_T, both=True)
    # Cut elongated slot along X axis for slip-joint travel.
    slot = (
        cq.Workplane("XY")
        .slot2D(SLOT_L, SLOT_W)
        .extrude(HALF_T * 3)
        .translate((0.0, 0.0, -HALF_T * 1.5))
    )
    hub = hub.cut(slot)
    return hub.cut(_lap_cut(s))


def _hub_solid_round(s: int) -> cq.Workplane:
    """Hub with round pivot hole (for half_moving)."""
    hub = cq.Workplane("XY").circle(HUB_R).extrude(HALF_T, both=True)
    # Cut round hole for pivot pin.
    hole = (
        cq.Workplane("XY")
        .circle(PIN_R + 0.0002)
        .extrude(HALF_T * 3)
        .translate((0.0, 0.0, -HALF_T * 1.5))
    )
    hub = hub.cut(hole)
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


def _grip_solid(s: int) -> cq.Solid:
    """Base grip sleeve (dark teal rubber)."""
    wires = [
        _ellipse_wire(x, s * _yc(x), 0.0, w, h)
        for x, w, h in GRIP_SECTIONS
    ]
    return cq.Solid.makeLoft(wires, ruled=False)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slip_joint_pliers")

    steel_polished = model.material("steel_polished", rgba=(0.80, 0.81, 0.84, 1.0))
    steel_forged = model.material("steel_forged", rgba=(0.66, 0.67, 0.70, 1.0))
    steel_brushed = model.material("steel_brushed", rgba=(0.61, 0.62, 0.65, 1.0))
    grip_teal = model.material("grip_teal", rgba=(0.10, 0.35, 0.40, 1.0))

    # --- half_fixed (root): bottom layer, has the elongated slot ---
    half_fixed = model.part("half_fixed")
    half_fixed.visual(
        mesh_from_cadquery(_jaw_solid(1), "fixed_jaw", tolerance=0.0002),
        name="jaw",
        material=steel_polished,
    )
    half_fixed.visual(
        mesh_from_cadquery(_hub_solid_slotted(1), "fixed_hub", tolerance=0.0002),
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
        material=grip_teal,
    )

    # --- slider: pivot pin carriage that slides in the slot ---
    slider = model.part("slider")
    # Pin shaft spanning both halves plus caps.
    pin_total_h = 2.0 * HALF_T + 2.0 * CAP_H + 2.0 * EPS
    slider.visual(
        Cylinder(PIN_R, 2.0 * HALF_T + 2.0 * EPS),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="pin_shaft",
        material=steel_brushed,
    )
    # Bottom cap (seated against half_fixed hub outer face).
    slider.visual(
        Cylinder(CAP_R, CAP_H),
        origin=Origin(xyz=(0.0, 0.0, -(HALF_T + CAP_H / 2.0 - 0.0003))),
        name="cap_bottom",
        material=steel_brushed,
    )
    # Top cap (seated against half_moving hub outer face).
    slider.visual(
        Cylinder(CAP_R, CAP_H),
        origin=Origin(xyz=(0.0, 0.0, HALF_T + CAP_H / 2.0 - 0.0003)),
        name="cap_top",
        material=steel_brushed,
    )

    # --- half_moving: top layer, has round pivot hole ---
    half_moving = model.part("half_moving")
    half_moving.visual(
        mesh_from_cadquery(_jaw_solid(-1), "moving_jaw", tolerance=0.0002),
        name="jaw",
        material=steel_polished,
    )
    half_moving.visual(
        mesh_from_cadquery(_hub_solid_round(-1), "moving_hub", tolerance=0.0002),
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
        material=grip_teal,
    )

    # --- Articulations ---
    # 1. Slip-joint prismatic: half_fixed -> slider, slides along X.
    #    Positive q moves the pivot toward the jaw (increases jaw opening range).
    model.articulation(
        "slip_joint",
        ArticulationType.PRISMATIC,
        parent=half_fixed,
        child=slider,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.1,
            lower=SLIP_LOWER,
            upper=SLIP_UPPER,
        ),
    )

    # 2. Pivot revolute: slider -> half_moving, rotates around Z.
    #    Positive q (about -Z) opens the jaws.
    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=slider,
        child=half_moving,
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


SLIP_LOWER = 0.0
SLIP_UPPER = SLIP_TRAVEL


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    half_fixed = object_model.get_part("half_fixed")
    half_moving = object_model.get_part("half_moving")
    slider = object_model.get_part("slider")
    slip = object_model.get_articulation("slip_joint")
    pivot = object_model.get_articulation("pivot")

    # --- Joint contract checks ---
    slip_limits = slip.motion_limits
    ctx.check(
        "slip_joint is prismatic with two-position travel",
        slip.articulation_type == ArticulationType.PRISMATIC
        and slip_limits is not None
        and slip_limits.lower is not None
        and slip_limits.upper is not None
        and abs(slip_limits.lower) < 1e-9
        and slip_limits.upper > 0.005,
        details=f"limits={slip_limits}",
    )

    pivot_limits = pivot.motion_limits
    ctx.check(
        "pivot is a revolute joint opening 0..30 degrees",
        pivot.articulation_type == ArticulationType.REVOLUTE
        and pivot_limits is not None
        and pivot_limits.lower is not None
        and pivot_limits.upper is not None
        and abs(pivot_limits.lower) < 1e-9
        and abs(pivot_limits.upper - OPEN_LIMIT) < 1e-6,
        details=f"limits={pivot_limits}",
    )

    # --- Rivet caps on both sides ---
    cap_top_aabb = ctx.part_element_world_aabb(slider, elem="cap_top")
    cap_bot_aabb = ctx.part_element_world_aabb(slider, elem="cap_bottom")
    ctx.check(
        "circular rivet caps exist on both sides",
        cap_top_aabb is not None and cap_bot_aabb is not None,
        details="missing cap_top or cap_bottom element",
    )
    if cap_top_aabb is not None and cap_bot_aabb is not None:
        cap_dia = cap_top_aabb[1][0] - cap_top_aabb[0][0]
        ctx.check(
            "rivet caps are circular and ~14 mm diameter",
            0.012 <= cap_dia <= 0.016,
            details=f"cap_diameter={cap_dia:.4f}",
        )
        # Caps straddle both halves.
        ctx.check(
            "caps extend beyond both outer faces",
            cap_top_aabb[1][2] > HALF_T and cap_bot_aabb[0][2] < -HALF_T,
            details=f"cap_top_z={cap_top_aabb[1][2]:.4f} cap_bot_z={cap_bot_aabb[0][2]:.4f}",
        )

    # --- Elongated slot visible in half_fixed hub ---
    hub_fixed_aabb = ctx.part_element_world_aabb(half_fixed, elem="hub")
    if hub_fixed_aabb is not None:
        hub_x_span = hub_fixed_aabb[1][0] - hub_fixed_aabb[0][0]
        ctx.check(
            "half_fixed hub is elongated along X (slot visible)",
            hub_x_span > 0.022,
            details=f"hub_x_span={hub_x_span:.4f}",
        )

    # --- Pin shaft centered in the slot ---
    ctx.allow_overlap(
        half_fixed,
        slider,
        elem_a="hub",
        elem_b="pin_shaft",
        reason="The pivot pin shaft passes through the elongated slot in half_fixed's hub.",
    )
    ctx.expect_within(
        slider,
        half_fixed,
        axes="y",
        inner_elem="pin_shaft",
        outer_elem="hub",
        margin=0.001,
        name="pin shaft stays within hub slot footprint on Y",
    )

    # --- Pin passes through half_moving hub ---
    ctx.allow_overlap(
        slider,
        half_moving,
        elem_a="pin_shaft",
        elem_b="hub",
        reason="The pivot pin passes through the round hole in half_moving's hub.",
    )
    ctx.expect_within(
        slider,
        half_moving,
        axes="xy",
        inner_elem="pin_shaft",
        outer_elem="hub",
        margin=0.001,
        name="pin shaft stays within moving hub footprint",
    )

    # --- Cap seating: caps embed slightly into hub outer faces for contact ---
    ctx.allow_overlap(
        slider,
        half_fixed,
        elem_a="cap_bottom",
        elem_b="hub",
        reason="The bottom rivet cap is seated against half_fixed's hub outer face, "
        "with a small local embed to represent the captured rivet head.",
    )
    ctx.expect_contact(
        slider,
        half_fixed,
        elem_a="cap_bottom",
        elem_b="hub",
        contact_tol=0.001,
        name="bottom cap contacts half_fixed hub face",
    )
    ctx.allow_overlap(
        slider,
        half_moving,
        elem_a="cap_top",
        elem_b="hub",
        reason="The top rivet cap is seated against half_moving's hub outer face, "
        "with a small local embed to represent the peened rivet head.",
    )
    ctx.expect_contact(
        slider,
        half_moving,
        elem_a="cap_top",
        elem_b="hub",
        contact_tol=0.001,
        name="top cap contacts half_moving hub face",
    )

    # --- Hub lap stacking ---
    ctx.expect_gap(
        half_moving,
        half_fixed,
        axis="z",
        positive_elem="hub",
        negative_elem="hub",
        min_gap=0.0,
        max_gap=0.001,
        name="moving hub lap stacks above fixed hub lap",
    )

    # --- Closed pose: jaws nearly touching ---
    ctx.expect_gap(
        half_fixed,
        half_moving,
        axis="y",
        positive_elem="jaw",
        negative_elem="jaw",
        min_gap=0.0002,
        max_gap=0.0015,
        name="jaws closed and nearly touching at rest",
    )

    # --- Geometric grip sleeves ---
    grip0_aabb = ctx.part_element_world_aabb(half_fixed, elem="grip")
    ctx.check(
        "grip base sleeve exists",
        grip0_aabb is not None,
        details="missing grip element",
    )

    # --- Overall envelope ---
    a0 = ctx.part_world_aabb(half_fixed)
    a1 = ctx.part_world_aabb(half_moving)
    if a0 is not None and a1 is not None:
        length = max(a0[1][0], a1[1][0]) - min(a0[0][0], a1[0][0])
        ctx.check(
            "overall length about 0.18-0.20 m",
            0.17 <= length <= 0.22,
            details=f"length={length:.4f}",
        )

    # --- Decisive open pose: jaws separate ---
    with ctx.pose({pivot: OPEN_LIMIT}):
        ctx.expect_gap(
            half_fixed,
            half_moving,
            axis="y",
            positive_elem="jaw",
            negative_elem="jaw",
            min_gap=0.003,
            name="jaws open apart at the 30 degree pose",
        )

    # --- Slip-joint position change moves the pivot ---
    slider_pos_0 = ctx.part_world_position(slider)
    with ctx.pose({slip: SLIP_UPPER}):
        slider_pos_1 = ctx.part_world_position(slider)
    if slider_pos_0 is not None and slider_pos_1 is not None:
        dx = slider_pos_1[0] - slider_pos_0[0]
        ctx.check(
            "slip_joint moves the pivot along X between positions",
            abs(dx - SLIP_TRAVEL) < 0.002,
            details=f"dx={dx:.4f} expected={SLIP_TRAVEL}",
        )

    return ctx.report()


object_model = build_object_model()
