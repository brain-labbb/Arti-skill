from __future__ import annotations

# End-cutting nippers with slip-joint pivot.
# Variant of heavy-duty lineman pliers reconfigured as end-cutting nippers.
#
# Key changes from parent:
# - End-cutting jaws perpendicular to the handles (cutting edges face +X)
# - Slip-joint pin slides along a short prismatic slot in the fixed half
# - Circular pivot rivet caps on both outer sides
# - Color-separated geometric grip sleeves (blue / orange)
#
# Layout: the tool lies in the XY plane with Z as the thickness axis.
# The slot pivot area is near the origin. Jaws extend toward +X with cutting
# edges perpendicular to the handle axis. Handles sweep back toward -X and
# spread in ±Y.

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
LAP_R = 0.016            # half-lap joint disc radius at the pivot
HUB_R = 0.014            # forged hub radius around the pivot area
SLOT_LEN = 0.012         # slip-joint slot travel (prismatic range)
SLOT_W = 0.006           # slot width (narrower than pin for captured fit)
PIN_R = 0.0035           # pivot pin shaft radius (diameter 0.007 > SLOT_W)
CAP_R = 0.011            # rivet cap radius (~0.022 m diameter)
CAP_H = 0.0025           # rivet cap height
SEAM_R = 0.012           # seam ring radius
SEAM_H = 0.0006
EPS = 0.0001             # lap clearance
JAW_FACE = 0.0003        # closed jaw inner face offset
OPEN_LIMIT = math.radians(25.0)
SLIDE_LOWER = 0.0
SLIDE_UPPER = SLOT_LEN

# Handle tang centerline (for the +Y half in tool plane).
TANG_PTS = [
    (-0.010, -0.004),
    (-0.028, -0.010),
    (-0.055, -0.016),
    (-0.080, -0.019),
    (-0.100, -0.021),
    (-0.118, -0.022),
]

# Grip loft stations: (x, half_width_y, half_height_z)
GRIP_SECTIONS = [
    (-0.030, 0.0095, 0.0125),  # flared guard near pivot
    (-0.042, 0.0082, 0.0110),
    (-0.060, 0.0078, 0.0105),
    (-0.080, 0.0080, 0.0110),
    (-0.100, 0.0084, 0.0115),
    (-0.115, 0.0086, 0.0118),  # bulbous end
    (-0.124, 0.0060, 0.0082),
    (-0.128, 0.0028, 0.0040),
]

# Centerline for grip placement (same as tang extended).
CENTERLINE = TANG_PTS + [(-0.124, -0.0224), (-0.128, -0.0226)]

TANG_HALF_W = 0.004      # steel tang half-width


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
    """Handle centerline y at station x."""
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


def _jaw_solid(s: int) -> cq.Workplane:
    """End-cutting jaw: blade extends along +X with cutting edge perpendicular
    to handle axis. The cutting face is at the +X end."""
    # End-cutting jaw profile: thick at base, tapering to cutting edge
    profile = [
        (0.008, JAW_FACE),          # base near pivot
        (0.035, JAW_FACE),          # flat inner cutting face
        (0.042, JAW_FACE + 0.001),  # slight bevel at cutting edge
        (0.042, 0.008),             # cutting edge corner (perpendicular face)
        (0.040, 0.013),             # outer curve top
        (0.032, 0.016),
        (0.020, 0.018),
        (0.012, 0.016),
        (0.008, 0.013),             # back to base
    ]
    pts = [(x, s * y) for x, y in profile]
    jaw = cq.Workplane("XY").polyline(pts).close().extrude(HALF_T, both=True)

    # Cutting edge groove - fine V-notch at the perpendicular face
    for i in range(5):
        xi = 0.036 + 0.0012 * i
        groove = (
            cq.Workplane("XY")
            .box(0.0008, 0.0018, 0.020)
            .translate((xi, s * JAW_FACE, 0.0))
        )
        jaw = jaw.cut(groove)

    return jaw.cut(_lap_cut(s))


def _hub_solid(s: int, has_slot: bool = False) -> cq.Workplane:
    """Forged circular hub around the pivot. The fixed half has an elongated slot."""
    hub = cq.Workplane("XY").circle(HUB_R).extrude(HALF_T, both=True)

    if has_slot:
        # Elongated slot for slip-joint: along X axis (narrower than pin for overlap)
        slot = (
            cq.Workplane("XY")
            .box(SLOT_LEN + 2.0 * PIN_R, SLOT_W, 2.0 * HALF_T + 0.002)
            .translate((SLOT_LEN / 2.0, 0.0, 0.0))
        )
        slot_end1 = (
            cq.Workplane("XY")
            .circle(SLOT_W / 2.0)
            .extrude(HALF_T + 0.001, both=True)
            .translate((0.0, 0.0, 0.0))
        )
        slot_end2 = (
            cq.Workplane("XY")
            .circle(SLOT_W / 2.0)
            .extrude(HALF_T + 0.001, both=True)
            .translate((SLOT_LEN, 0.0, 0.0))
        )
        hub = hub.cut(slot).cut(slot_end1).cut(slot_end2)
    # Moving half: no hole cut — pin shaft is captured through the hub material

    return hub.cut(_lap_cut(s))


def _shank_solid(s: int) -> cq.Workplane:
    """Steel handle tang sweeping back from the hub."""
    pts = [(x, s * y) for x, y in TANG_PTS]
    loop = _strip_loop(pts, TANG_HALF_W)
    shank = cq.Workplane("XY").polyline(loop).close().extrude(HALF_T, both=True)
    return shank.cut(_lap_cut(s))


def _ellipse_wire(x: float, yc: float, zc: float, w: float, h: float) -> cq.Wire:
    wp = cq.Workplane("YZ", origin=(x, 0.0, 0.0)).center(yc, zc).ellipse(w, h)
    return wp.val()


def _grip_solid(s: int) -> cq.Solid:
    """Geometric grip sleeve: curved ergonomic shape."""
    wires = [
        _ellipse_wire(x, s * _yc(x), 0.0, w, h)
        for x, w, h in GRIP_SECTIONS
    ]
    return cq.Solid.makeLoft(wires, ruled=False)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="end_cutting_nippers")

    # Materials
    steel_polished = model.material("steel_polished", rgba=(0.80, 0.81, 0.84, 1.0))
    steel_forged = model.material("steel_forged", rgba=(0.62, 0.63, 0.66, 1.0))
    steel_brushed = model.material("steel_brushed", rgba=(0.58, 0.59, 0.62, 1.0))
    seam_gray = model.material("seam_gray", rgba=(0.42, 0.43, 0.46, 1.0))
    grip_blue = model.material("grip_blue", rgba=(0.12, 0.35, 0.68, 1.0))
    grip_orange = model.material("grip_orange", rgba=(0.92, 0.45, 0.08, 1.0))

    # ---- Part 1: handle_fixed (root) - the half with the slot ----
    fixed = model.part("handle_fixed")

    fixed.visual(
        mesh_from_cadquery(_jaw_solid(1), "fixed_jaw", tolerance=0.0002),
        name="jaw",
        material=steel_polished,
    )
    fixed.visual(
        mesh_from_cadquery(_hub_solid(1, has_slot=True), "fixed_hub", tolerance=0.0002),
        name="hub",
        material=steel_forged,
    )
    fixed.visual(
        mesh_from_cadquery(_shank_solid(1), "fixed_shank", tolerance=0.0002),
        name="shank",
        material=steel_forged,
    )
    fixed.visual(
        mesh_from_cadquery(_grip_solid(1), "fixed_grip", tolerance=0.0002),
        name="grip",
        material=grip_blue,
    )

    # ---- Part 2: pivot_pin - slides in the slot ----
    pin = model.part("pivot_pin")

    # Pin shaft
    pin.visual(
        Cylinder(PIN_R, 2.0 * HALF_T + 0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="shaft",
        material=steel_brushed,
    )

    # Bottom rivet cap (visible on outer face)
    pin.visual(
        Cylinder(SEAM_R, SEAM_H),
        origin=Origin(xyz=(0.0, 0.0, -(HALF_T + SEAM_H / 2.0 + 0.001))),
        name="cap_bottom_seam",
        material=seam_gray,
    )
    pin.visual(
        Cylinder(CAP_R, CAP_H),
        origin=Origin(xyz=(0.0, 0.0, -(HALF_T + SEAM_H + CAP_H / 2.0 + 0.001))),
        name="cap_bottom",
        material=steel_brushed,
    )

    # Top rivet cap (visible on outer face)
    pin.visual(
        Cylinder(SEAM_R, SEAM_H),
        origin=Origin(xyz=(0.0, 0.0, HALF_T + SEAM_H / 2.0 + 0.001)),
        name="cap_top_seam",
        material=seam_gray,
    )
    pin.visual(
        Cylinder(CAP_R, CAP_H),
        origin=Origin(xyz=(0.0, 0.0, HALF_T + SEAM_H + CAP_H / 2.0 + 0.001)),
        name="cap_top",
        material=steel_brushed,
    )

    # ---- Part 3: handle_moving - rotates about the pin ----
    moving = model.part("handle_moving")

    moving.visual(
        mesh_from_cadquery(_jaw_solid(-1), "moving_jaw", tolerance=0.0002),
        name="jaw",
        material=steel_polished,
    )
    moving.visual(
        mesh_from_cadquery(_hub_solid(-1, has_slot=False), "moving_hub", tolerance=0.0002),
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
        material=grip_orange,
    )

    # ---- Articulations ----
    # 1. Prismatic: pin slides in slot along X
    model.articulation(
        "slot_slide",
        ArticulationType.PRISMATIC,
        parent=fixed,
        child=pin,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=50.0, velocity=0.1,
            lower=SLIDE_LOWER, upper=SLIDE_UPPER,
        ),
    )

    # 2. Revolute: moving half rotates about the pin (Z axis, perpendicular to tool plane)
    # Positive q (about -Z) opens the jaws apart (jaw on -Y swings further to -Y).
    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=pin,
        child=moving,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=80.0, velocity=3.0,
            lower=0.0, upper=OPEN_LIMIT,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    fixed = object_model.get_part("handle_fixed")
    pin = object_model.get_part("pivot_pin")
    moving = object_model.get_part("handle_moving")

    slide = object_model.get_articulation("slot_slide")
    pivot = object_model.get_articulation("pivot")

    # ---- Joint existence and type checks ----
    slide_limits = slide.motion_limits
    ctx.check(
        "slot_slide is a prismatic joint with slot travel",
        slide.articulation_type == ArticulationType.PRISMATIC
        and slide_limits is not None
        and slide_limits.lower is not None
        and slide_limits.upper is not None
        and abs(slide_limits.lower) < 1e-9
        and abs(slide_limits.upper - SLIDE_UPPER) < 1e-6,
        details=f"type={slide.articulation_type} limits={slide_limits}",
    )

    pivot_limits = pivot.motion_limits
    ctx.check(
        "pivot is a revolute joint with 0..25 degree range",
        pivot.articulation_type == ArticulationType.REVOLUTE
        and pivot_limits is not None
        and pivot_limits.lower is not None
        and pivot_limits.upper is not None
        and abs(pivot_limits.lower) < 1e-9
        and abs(pivot_limits.upper - OPEN_LIMIT) < 1e-6,
        details=f"type={pivot.articulation_type} limits={pivot_limits}",
    )

    # ---- Kinematic chain: fixed -> pin (prismatic) -> moving (revolute) ----
    ctx.check(
        "kinematic chain: fixed -> pin -> moving",
        slide.parent == "handle_fixed"
        and slide.child == "pivot_pin"
        and pivot.parent == "pivot_pin"
        and pivot.child == "handle_moving",
        details=f"slide: {slide.parent}->{slide.child}, pivot: {pivot.parent}->{pivot.child}",
    )

    # ---- End-cutting jaws: perpendicular to handle axis ----
    # The jaw cutting edge extends to +X while handles go to -X.
    # Verify jaw extends in +X and the perpendicular cutting face is present.
    jaw_fixed_aabb = ctx.part_element_world_aabb(fixed, elem="jaw")
    jaw_moving_aabb = ctx.part_element_world_aabb(moving, elem="jaw")
    if jaw_fixed_aabb is not None:
        ctx.check(
            "fixed jaw extends toward +X (end-cutting orientation)",
            jaw_fixed_aabb[1][0] > 0.030,
            details=f"jaw_max_x={jaw_fixed_aabb[1][0]:.4f}",
        )
    if jaw_moving_aabb is not None:
        ctx.check(
            "moving jaw extends toward +X (end-cutting orientation)",
            jaw_moving_aabb[1][0] > 0.030,
            details=f"jaw_max_x={jaw_moving_aabb[1][0]:.4f}",
        )

    # Jaw perpendicularity: the jaw blade extends forward along X (handle axis),
    # with the cutting face perpendicular to it (in YZ plane).
    # This distinguishes end-cutting nippers from parallel-jaw pliers.
    if jaw_fixed_aabb is not None:
        jaw_x_span = jaw_fixed_aabb[1][0] - jaw_fixed_aabb[0][0]
        jaw_y_span = jaw_fixed_aabb[1][1] - jaw_fixed_aabb[0][1]
        ctx.check(
            "jaw blade extends forward (X span > Y span, end-cutting orientation)",
            jaw_x_span > jaw_y_span,
            details=f"x_span={jaw_x_span:.4f} y_span={jaw_y_span:.4f}",
        )

    # ---- Closed pose: jaws nearly touching ----
    ctx.expect_gap(
        fixed,
        moving,
        axis="y",
        positive_elem="jaw",
        negative_elem="jaw",
        min_gap=0.0002,
        max_gap=0.002,
        name="closed jaws nearly touching at rest",
    )

    # ---- Rivet caps on both sides ----
    cap_bottom_aabb = ctx.part_element_world_aabb(pin, elem="cap_bottom")
    cap_top_aabb = ctx.part_element_world_aabb(pin, elem="cap_top")
    ctx.check(
        "bottom rivet cap exists",
        cap_bottom_aabb is not None,
        details="cap_bottom AABB missing",
    )
    ctx.check(
        "top rivet cap exists",
        cap_top_aabb is not None,
        details="cap_top AABB missing",
    )
    if cap_bottom_aabb is not None and cap_top_aabb is not None:
        cap_dia = cap_bottom_aabb[1][0] - cap_bottom_aabb[0][0]
        total_thick = cap_top_aabb[1][2] - cap_bottom_aabb[0][2]
        ctx.check(
            "rivet caps are ~22mm diameter on both sides",
            0.019 <= cap_dia <= 0.025,
            details=f"cap_dia={cap_dia:.4f}",
        )
        ctx.check(
            "caps span the full pivot thickness",
            total_thick > 0.020,
            details=f"total_thick={total_thick:.4f}",
        )

    # ---- Color-separated grip sleeves ----
    grip_fixed = fixed.get_visual("grip")
    grip_moving = moving.get_visual("grip")
    ctx.check(
        "fixed grip has blue material",
        grip_fixed.material is not None
        and grip_fixed.material.rgba is not None
        and grip_fixed.material.rgba[2] > 0.5,  # blue channel dominant
        details=f"fixed_grip_rgba={grip_fixed.material.rgba if grip_fixed.material else None}",
    )
    ctx.check(
        "moving grip has orange material",
        grip_moving.material is not None
        and grip_moving.material.rgba is not None
        and grip_moving.material.rgba[0] > 0.7  # red channel high (orange)
        and grip_moving.material.rgba[1] > 0.3  # some green (orange)
        and grip_moving.material.rgba[2] < 0.3,  # low blue
        details=f"moving_grip_rgba={grip_moving.material.rgba if grip_moving.material else None}",
    )

    # ---- Slip-joint slot in fixed hub ----
    hub_fixed_aabb = ctx.part_element_world_aabb(fixed, elem="hub")
    if hub_fixed_aabb is not None:
        # The slot extends along X in the hub - hub should be wider in X than Y
        hub_x_span = hub_fixed_aabb[1][0] - hub_fixed_aabb[0][0]
        hub_y_span = hub_fixed_aabb[1][1] - hub_fixed_aabb[0][1]
        ctx.check(
            "fixed hub has slot elongated along X",
            hub_x_span >= hub_y_span * 0.95,
            details=f"hub_x={hub_x_span:.4f} hub_y={hub_y_span:.4f}",
        )

    # ---- Pin shaft captured through both halves ----
    ctx.allow_overlap(
        pin,
        fixed,
        elem_a="shaft",
        elem_b="hub",
        reason="Pivot pin shaft is intentionally captured through the fixed half's slot region, riding in the slip-joint groove.",
    )
    ctx.allow_overlap(
        pin,
        moving,
        elem_a="shaft",
        elem_b="hub",
        reason="Pivot pin shaft is intentionally embedded in the moving half's hub, capturing it for rotation about the pin.",
    )

    ctx.expect_overlap(
        pin,
        fixed,
        axes="z",
        elem_a="shaft",
        elem_b="hub",
        min_overlap=0.005,
        name="pin shaft passes through fixed hub lap thickness",
    )
    ctx.expect_overlap(
        pin,
        moving,
        axes="z",
        elem_a="shaft",
        elem_b="hub",
        min_overlap=0.005,
        name="pin shaft passes through moving hub lap thickness",
    )
    ctx.expect_within(
        pin,
        fixed,
        axes="xy",
        inner_elem="shaft",
        outer_elem="hub",
        margin=0.002,
        name="pin shaft stays within fixed hub footprint",
    )
    ctx.expect_within(
        pin,
        moving,
        axes="xy",
        inner_elem="shaft",
        outer_elem="hub",
        margin=0.002,
        name="pin shaft stays within moving hub footprint",
    )

    # Hub stacking: the two half-lapped hubs seat against each other with a
    # thin Z gap; the moving hub sits above the fixed hub.
    ctx.expect_gap(
        moving,
        fixed,
        axis="z",
        positive_elem="hub",
        negative_elem="hub",
        min_gap=0.0,
        max_gap=0.001,
        name="moving hub lap stacks above fixed hub lap",
    )
    ctx.expect_contact(
        fixed,
        moving,
        elem_a="hub",
        elem_b="hub",
        contact_tol=0.001,
        name="hub laps seat against each other at the joint",
    )

    # ---- Open pose: jaws separate ----
    with ctx.pose({pivot: OPEN_LIMIT}):
        ctx.expect_gap(
            fixed,
            moving,
            axis="y",
            positive_elem="jaw",
            negative_elem="jaw",
            min_gap=0.003,
            name="jaws open apart at 25 degree pose",
        )

    # ---- Decisive pose: prismatic slide moves the pin along X ----
    pin_rest = ctx.part_world_position(pin)
    with ctx.pose({slide: SLIDE_UPPER}):
        pin_slid = ctx.part_world_position(pin)
        ctx.check(
            "prismatic slide moves pin along +X",
            pin_rest is not None
            and pin_slid is not None
            and pin_slid[0] > pin_rest[0] + 0.005,
            details=f"rest_x={pin_rest}, slid_x={pin_slid}",
        )

    # ---- Overall dimensions ----
    a_fixed = ctx.part_world_aabb(fixed)
    a_moving = ctx.part_world_aabb(moving)
    if a_fixed is not None and a_moving is not None:
        length = max(a_fixed[1][0], a_moving[1][0]) - min(a_fixed[0][0], a_moving[0][0])
        ctx.check(
            "overall length about 0.17-0.20 m",
            0.16 <= length <= 0.22,
            details=f"length={length:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
