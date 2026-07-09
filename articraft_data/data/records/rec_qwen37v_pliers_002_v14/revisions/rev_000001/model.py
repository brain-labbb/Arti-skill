from __future__ import annotations

# Slip-joint pliers with two-position pivot.
# Variant of heavy-duty combination pliers: same overall scale (~0.20 m long)
# but with a slip-joint mechanism — an elongated slot in one half lets the
# pivot pin slide to two discrete positions for different jaw openings.
#
# Layout: the tool lies in the XY plane with Z as the thickness axis.
# The hub centre is at the origin. The jaw tips point +X; handles sweep
# back to -X and spread in +/-Y.
# jaw_fixed (root) carries its jaw on +Y and has the elongated slot.
# jaw_moving (child) carries its jaw on -Y.
# pivot_pin is a small intermediate part whose prismatic travel along
# the slot gives two pivot positions.

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

# ---- shared dimensions (metres) ----
HALF_T = 0.008            # half thickness of each forged plate
HUB_R = 0.014             # forged hub radius around the pivot
EPS = 0.0001              # lap clearance
JAW_FACE = 0.0003         # closed jaw inner faces sit at y = +/-JAW_FACE
NOSE_X = 0.068            # blunt nose tip
OPEN_LIMIT = math.radians(28.0)

# Slip-joint slot
SLOT_LENGTH = 0.014       # total capsule length (end-to-end)
SLOT_DIA = 0.006          # slot width
SLOT_TRAVEL = 0.010       # prismatic travel of pin centre

# Pivot pin
PIN_R = 0.0028            # pin shaft radius
CAP_R = 0.006             # circular rivet cap radius
CAP_H = 0.002             # cap height

# Cutter bevel wedge
BEVEL_X_START = 0.010
BEVEL_X_END = 0.022
BEVEL_DEPTH = 0.003       # how far the bevel intrudes toward the jaw face
BEVEL_HEIGHT = HALF_T * 1.6

# Handle tang
TANG_HALF_W = 0.004

TANG_PTS = [
    (-0.010, -0.005),
    (-0.030, -0.012),
    (-0.065, -0.019),
    (-0.095, -0.023),
    (-0.115, -0.025),
]
CENTERLINE = TANG_PTS + [(-0.128, -0.026)]

GRIP_SECTIONS = [
    (-0.032, 0.0098, 0.0125),
    (-0.040, 0.0078, 0.0108),
    (-0.060, 0.0076, 0.0105),
    (-0.085, 0.0078, 0.0110),
    (-0.105, 0.0084, 0.0115),
    (-0.118, 0.0086, 0.0118),
    (-0.126, 0.0060, 0.0082),
    (-0.128, 0.0028, 0.0040),
]

INLAY_XS = [-0.036, -0.055, -0.075, -0.095, -0.110, -0.116]
INLAY_HALF_H = 0.0070
INLAY_Z_CENTER = 0.006


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
    """Material removed so this half keeps only its lap layer."""
    cut = cq.Workplane("XY").circle(HUB_R + 0.002).extrude(0.012)
    if s > 0:
        return cut.translate((0.0, 0.0, -EPS))
    return cut.translate((0.0, 0.0, -0.012 + EPS))


def _slot_cut() -> cq.Workplane:
    """Elongated slip-joint slot cut through the hub (capsule shape)."""
    slot = (
        cq.Workplane("XY")
        .slot2D(SLOT_LENGTH, SLOT_DIA, angle=0)
        .extrude(0.030)
        .translate((0.0, 0.0, -0.015))
    )
    return slot


def _jaw_lap_cut(s: int) -> cq.Workplane:
    """Rectangular lap cut for the jaw base, covering the full hub overlap area."""
    # Box covering x=[-0.005, 0.020], y=[-0.020, 0.020] — wider than the
    # circular hub to ensure the jaw base is properly half-lapped even at
    # profile points outside the hub circle.
    cut = cq.Workplane("XY").box(0.025, 0.042, HALF_T + 0.002)
    if s > 0:
        # Remove upper half of jaw base (keep lower lap)
        return cut.translate((0.0075, 0.0, HALF_T / 2.0 + 0.001 - EPS))
    # Remove lower half of jaw base (keep upper lap)
    return cut.translate((0.0075, 0.0, -(HALF_T / 2.0 + 0.001 - EPS)))


def _jaw_solid(s: int) -> cq.Workplane:
    """Flat slip-joint jaw: wide serrated gripping face, blunt tip, cutter area."""
    profile = [
        (0.012, JAW_FACE),
        (NOSE_X, JAW_FACE),
        (NOSE_X, 0.0095),       # blunt flat nose
        (0.058, 0.0120),
        (0.042, 0.0140),
        (0.028, 0.0150),
        (0.016, 0.0148),
        (0.012, 0.0130),
    ]
    pts = [(x, s * y) for x, y in profile]
    jaw = cq.Workplane("XY").polyline(pts).close().extrude(HALF_T, both=True)

    # Fine horizontal gripping serrations across the inner face.
    for i in range(8):
        xi = 0.030 + 0.004 * i
        groove = (
            cq.Workplane("XY")
            .box(0.0012, 0.0020, 0.018)
            .translate((xi, s * JAW_FACE, 0.0))
        )
        jaw = jaw.cut(groove)

    # Small pipe-grip notch behind the nose.
    recess = (
        cq.Workplane("XY")
        .circle(0.0035)
        .extrude(0.010, both=True)
        .translate((0.035, s * JAW_FACE, 0.0))
    )
    jaw = jaw.cut(recess)

    return jaw.cut(_jaw_lap_cut(s))


def _hub_solid(s: int, with_slot: bool = False) -> cq.Workplane:
    """Forged circular hub, directly extruded to half-thickness.
    If with_slot, cut the slip-joint slot through it."""
    thickness = HALF_T - EPS
    if s > 0:
        hub = (cq.Workplane("XY")
               .circle(HUB_R)
               .extrude(thickness)
               .translate((0.0, 0.0, -HALF_T)))
    else:
        hub = (cq.Workplane("XY")
               .circle(HUB_R)
               .extrude(thickness)
               .translate((0.0, 0.0, EPS)))
    if with_slot:
        hub = hub.cut(_slot_cut())
    return hub


def _shank_solid(s: int) -> cq.Workplane:
    """Steel handle tang, directly extruded to half-thickness."""
    pts = [(x, s * y) for x, y in TANG_PTS]
    loop = _strip_loop(pts, TANG_HALF_W)
    thickness = HALF_T - EPS
    if s > 0:
        return (cq.Workplane("XY")
                .polyline(loop).close()
                .extrude(thickness)
                .translate((0.0, 0.0, -HALF_T)))
    else:
        return (cq.Workplane("XY")
                .polyline(loop).close()
                .extrude(thickness)
                .translate((0.0, 0.0, EPS)))


def _cutter_bevel_solid(s: int) -> cq.Workplane:
    """Wedge-shaped cutter bevel on the inner jaw face near the pivot."""
    # Triangular wedge: wide at the inner face, tapering to an edge.
    # Sits between BEVEL_X_START and BEVEL_X_END, on the inner side (y toward 0).
    pts = [
        (BEVEL_X_START, s * (JAW_FACE + 0.001)),
        (BEVEL_X_END, s * (JAW_FACE + 0.001)),
        (BEVEL_X_END, s * (JAW_FACE + BEVEL_DEPTH)),
    ]
    wedge = cq.Workplane("XY").polyline(pts).close().extrude(BEVEL_HEIGHT / 2.0, both=True)
    return wedge


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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slip_joint_pliers")

    steel_polished = model.material("steel_polished", rgba=(0.80, 0.81, 0.84, 1.0))
    steel_forged = model.material("steel_forged", rgba=(0.66, 0.67, 0.70, 1.0))
    steel_brushed = model.material("steel_brushed", rgba=(0.61, 0.62, 0.65, 1.0))
    cutter_steel = model.material("cutter_steel", rgba=(0.72, 0.73, 0.76, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.08, 0.09, 1.0))
    grip_red = model.material("grip_red", rgba=(0.80, 0.09, 0.11, 1.0))
    cap_gray = model.material("cap_gray", rgba=(0.50, 0.51, 0.53, 1.0))

    # ---- jaw_fixed (root, jaw on +Y, lower lap, has the slot) ----
    jaw_fixed = model.part("jaw_fixed")

    jaw_fixed.visual(
        mesh_from_cadquery(_jaw_solid(1), "fixed_jaw", tolerance=0.0002),
        name="jaw",
        material=steel_polished,
    )
    jaw_fixed.visual(
        mesh_from_cadquery(_hub_solid(1, with_slot=True), "fixed_hub", tolerance=0.0002),
        name="hub",
        material=steel_forged,
    )
    jaw_fixed.visual(
        mesh_from_cadquery(_shank_solid(1), "fixed_shank", tolerance=0.0002),
        name="shank",
        material=steel_forged,
    )
    jaw_fixed.visual(
        mesh_from_cadquery(_grip_solid(1), "fixed_grip", tolerance=0.0002),
        name="grip",
        material=rubber_black,
    )
    jaw_fixed.visual(
        mesh_from_cadquery(_inlay_solid(1), "fixed_inlay", tolerance=0.0002),
        name="grip_inlay",
        material=grip_red,
    )
    jaw_fixed.visual(
        mesh_from_cadquery(_cutter_bevel_solid(1), "fixed_cutter", tolerance=0.0002),
        name="cutter_bevel",
        material=cutter_steel,
    )

    # ---- pivot_pin (intermediate, carries the pin and both caps) ----
    pivot_pin = model.part("pivot_pin")

    # Pin shaft spans the full thickness of both halves plus a small margin.
    pin_height = 2.0 * HALF_T + 2.0 * EPS + 0.001
    pivot_pin.visual(
        Cylinder(PIN_R, pin_height),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="pin_shaft",
        material=steel_brushed,
    )
    # Circular rivet cap on the bottom face.
    pivot_pin.visual(
        Cylinder(CAP_R, CAP_H),
        origin=Origin(xyz=(0.0, 0.0, -(HALF_T + EPS + CAP_H / 2.0))),
        name="cap_bottom",
        material=cap_gray,
    )
    # Circular rivet cap on the top face.
    pivot_pin.visual(
        Cylinder(CAP_R, CAP_H),
        origin=Origin(xyz=(0.0, 0.0, HALF_T + EPS + CAP_H / 2.0)),
        name="cap_top",
        material=cap_gray,
    )

    # ---- jaw_moving (jaw on -Y, upper lap, no slot) ----
    jaw_moving = model.part("jaw_moving")

    jaw_moving.visual(
        mesh_from_cadquery(_jaw_solid(-1), "moving_jaw", tolerance=0.0002),
        name="jaw",
        material=steel_polished,
    )
    jaw_moving.visual(
        mesh_from_cadquery(_hub_solid(-1, with_slot=False), "moving_hub", tolerance=0.0002),
        name="hub",
        material=steel_forged,
    )
    jaw_moving.visual(
        mesh_from_cadquery(_shank_solid(-1), "moving_shank", tolerance=0.0002),
        name="shank",
        material=steel_forged,
    )
    jaw_moving.visual(
        mesh_from_cadquery(_grip_solid(-1), "moving_grip", tolerance=0.0002),
        name="grip",
        material=rubber_black,
    )
    jaw_moving.visual(
        mesh_from_cadquery(_inlay_solid(-1), "moving_inlay", tolerance=0.0002),
        name="grip_inlay",
        material=grip_red,
    )
    jaw_moving.visual(
        mesh_from_cadquery(_cutter_bevel_solid(-1), "moving_cutter", tolerance=0.0002),
        name="cutter_bevel",
        material=cutter_steel,
    )

    # ---- Articulations ----

    # 1. Prismatic: pin slides along the slot in jaw_fixed.
    #    Origin at the near end of the slot travel; axis +X.
    #    At q=0 the pin is at the narrow position; at q=SLOT_TRAVEL it is
    #    at the wide position.
    model.articulation(
        "slot_slide",
        ArticulationType.PRISMATIC,
        parent=jaw_fixed,
        child=pivot_pin,
        origin=Origin(xyz=(-SLOT_TRAVEL / 2.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=50.0, velocity=0.1, lower=0.0, upper=SLOT_TRAVEL,
        ),
    )

    # 2. Revolute: jaw_moving pivots around the pin to open/close the jaws.
    #    Positive q (about -Z) swings the moving jaw toward -Y, opening the
    #    jaws while the handles spread apart.
    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=pivot_pin,
        child=jaw_moving,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=80.0, velocity=3.0, lower=0.0, upper=OPEN_LIMIT,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    jaw_fixed = object_model.get_part("jaw_fixed")
    jaw_moving = object_model.get_part("jaw_moving")
    pivot_pin = object_model.get_part("pivot_pin")
    slot_slide = object_model.get_articulation("slot_slide")
    pivot = object_model.get_articulation("pivot")

    # --- Articulation structure ---
    ctx.check(
        "slot_slide is a prismatic joint",
        slot_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={slot_slide.articulation_type}",
    )
    ctx.check(
        "pivot is a revolute joint",
        pivot.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={pivot.articulation_type}",
    )

    slide_limits = slot_slide.motion_limits
    ctx.check(
        "slot_slide has two-position travel (~10 mm)",
        slide_limits is not None
        and slide_limits.lower is not None
        and slide_limits.upper is not None
        and abs(slide_limits.lower) < 1e-9
        and 0.008 <= slide_limits.upper <= 0.012,
        details=f"limits={slide_limits}",
    )

    pivot_limits = pivot.motion_limits
    ctx.check(
        "pivot opens 0 to ~28 degrees",
        pivot_limits is not None
        and pivot_limits.lower is not None
        and pivot_limits.upper is not None
        and abs(pivot_limits.lower) < 1e-9
        and abs(pivot_limits.upper - OPEN_LIMIT) < 1e-6,
        details=f"limits={pivot_limits}",
    )

    # --- Closed rest pose: jaws nearly touch ---
    jaw_f = jaw_fixed.get_visual("jaw")
    jaw_m = jaw_moving.get_visual("jaw")
    ctx.expect_gap(
        jaw_fixed,
        jaw_moving,
        axis="y",
        positive_elem=jaw_f,
        negative_elem=jaw_m,
        min_gap=0.0002,
        max_gap=0.002,
        name="jaws closed and nearly touching at rest",
    )

    # --- Half-lap interleaving at the hub ---
    hub_f = jaw_fixed.get_visual("hub")
    hub_m = jaw_moving.get_visual("hub")
    ctx.expect_gap(
        jaw_moving,
        jaw_fixed,
        axis="z",
        positive_elem=hub_m,
        negative_elem=hub_f,
        min_gap=0.0,
        max_gap=0.001,
        name="moving hub lap stacks above fixed hub lap",
    )

    # --- Pin passes through both halves (intentional overlap) ---
    ctx.allow_overlap(
        pivot_pin,
        jaw_fixed,
        elem_a="pin_shaft",
        elem_b=hub_f,
        reason="The pivot pin shaft passes through the slip-joint slot in the "
        "fixed jaw hub, which is the physical slip-joint connection.",
    )
    ctx.allow_overlap(
        pivot_pin,
        jaw_moving,
        elem_a="pin_shaft",
        elem_b=hub_m,
        reason="The pivot pin shaft passes through the moving jaw hub, "
        "capturing it at the pivot.",
    )

    ctx.expect_within(
        pivot_pin,
        jaw_fixed,
        axes="xy",
        inner_elem="pin_shaft",
        outer_elem=hub_f,
        margin=0.002,
        name="pin shaft stays within the fixed hub footprint",
    )

    # --- Rivet caps visible on both sides ---
    cap_b = ctx.part_element_world_aabb(pivot_pin, elem="cap_bottom")
    cap_t = ctx.part_element_world_aabb(pivot_pin, elem="cap_top")
    ctx.check(
        "pivot pin has circular caps on both faces",
        cap_b is not None and cap_t is not None,
        details="missing cap_bottom or cap_top element",
    )
    if cap_b is not None and cap_t is not None:
        cap_dia = cap_t[1][0] - cap_t[0][0]
        ctx.check(
            "rivet caps are ~12 mm diameter circles",
            0.010 <= cap_dia <= 0.014,
            details=f"cap_dia={cap_dia:.4f}",
        )
        ctx.check(
            "caps are proud on both outer faces of the tool",
            cap_b[0][2] < -(HALF_T + EPS) - 0.0005
            and cap_t[1][2] > (HALF_T + EPS) + 0.0005,
            details=f"cap_bottom_z={cap_b[0][2]:.4f} cap_top_z={cap_t[1][2]:.4f}",
        )

    # --- Cutter bevel wedges present on both jaws ---
    bevel_f = ctx.part_element_world_aabb(jaw_fixed, elem="cutter_bevel")
    bevel_m = ctx.part_element_world_aabb(jaw_moving, elem="cutter_bevel")
    ctx.check(
        "cutter bevel wedges exist on both jaws",
        bevel_f is not None and bevel_m is not None,
        details="missing cutter_bevel element",
    )
    if bevel_f is not None:
        bevel_dx = bevel_f[1][0] - bevel_f[0][0]
        ctx.check(
            "cutter bevel has visible wedge extent (~10-15 mm)",
            0.008 <= bevel_dx <= 0.016,
            details=f"bevel_dx={bevel_dx:.4f}",
        )

    # --- Slip-joint slot visible in the fixed hub ---
    hub_f_aabb = ctx.part_element_world_aabb(jaw_fixed, elem="hub")
    if hub_f_aabb is not None:
        # The hub has a slot cut through it; verify it is roughly circular
        # in plan but with the slot elongation along X.
        hub_dx = hub_f_aabb[1][0] - hub_f_aabb[0][0]
        hub_dy = hub_f_aabb[1][1] - hub_f_aabb[0][1]
        ctx.check(
            "fixed hub is roughly circular in plan with slot elongation",
            hub_dx >= 2.0 * HUB_R - 0.004
            and hub_dy >= 2.0 * HUB_R - 0.004,
            details=f"hub_dx={hub_dx:.4f} hub_dy={hub_dy:.4f}",
        )

    # --- Overall envelope: ~0.20 m long ---
    a0 = ctx.part_world_aabb(jaw_fixed)
    a1 = ctx.part_world_aabb(jaw_moving)
    if a0 is not None and a1 is not None:
        length = max(a0[1][0], a1[1][0]) - min(a0[0][0], a1[0][0])
        ctx.check(
            "overall length about 0.20 m",
            0.19 <= length <= 0.215,
            details=f"length={length:.4f}",
        )

    # --- Prismatic slide moves the pin along +X ---
    pin_pos_rest = ctx.part_world_position(pivot_pin)
    with ctx.pose({slot_slide: SLOT_TRAVEL}):
        pin_pos_slid = ctx.part_world_position(pivot_pin)
        ctx.check(
            "slot_slide moves pin along +X",
            pin_pos_rest is not None
            and pin_pos_slid is not None
            and pin_pos_slid[0] > pin_pos_rest[0] + 0.005,
            details=f"rest={pin_pos_rest}, slid={pin_pos_slid}",
        )

    # --- Revolute pivot opens the jaws ---
    closed_jaw_m = ctx.part_element_world_aabb(jaw_moving, elem="jaw")
    closed_grip_m = ctx.part_element_world_aabb(jaw_moving, elem="grip")
    with ctx.pose({pivot: OPEN_LIMIT}):
        open_jaw_m = ctx.part_element_world_aabb(jaw_moving, elem="jaw")
        open_grip_m = ctx.part_element_world_aabb(jaw_moving, elem="grip")
        ctx.check(
            "moving jaw swings away at the open pose",
            closed_jaw_m is not None
            and open_jaw_m is not None
            and open_jaw_m[0][1] < closed_jaw_m[0][1] - 0.015,
            details=f"closed_min_y={closed_jaw_m}, open_min_y={open_jaw_m}",
        )
        ctx.check(
            "moving handle spreads outward as the jaws open",
            closed_grip_m is not None
            and open_grip_m is not None
            and open_grip_m[1][1] > closed_grip_m[1][1] + 0.025,
            details=f"closed_max_y={closed_grip_m}, open_max_y={open_grip_m}",
        )

    return ctx.report()


object_model = build_object_model()
