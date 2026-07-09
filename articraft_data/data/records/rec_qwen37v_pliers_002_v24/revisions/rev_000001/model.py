from __future__ import annotations

# Slip-joint pliers with two pivot positions and a return spring.
# Variant of the heavy-duty combination lineman pliers.
#
# Layout: the tool lies in the XY plane with Z as the thickness axis.
# The jaws point +X, handles sweep to -X and spread in +/-Y.
# slot_half (root) carries the elongated slip-joint slot; jaw_half pivots
# around the pivot_pin which slides in the slot.
# A leaf return spring is anchored to slot_half and arcs toward jaw_half.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---- shared dimensions (meters) ----
HALF_T = 0.009            # half thickness of each forged plate
HUB_R = 0.014             # hub radius around pivot area
SLOT_HALF_LEN = 0.006     # half-length of elongated slip slot
SLOT_HALF_W = 0.004       # half-width of the slot
CAP_R = 0.0125            # pivot rivet cap radius (~0.025 m diameter)
CAP_H = 0.0025            # cap thickness
PIN_R = 0.0035            # pivot pin shaft radius
SLOT_CLEAR = 0.0003       # radial sliding clearance between pin and slot walls
SLOT_R = PIN_R + SLOT_CLEAR  # rounded-slot wall radius hugging the pin barrel
EPS = 0.0001              # clearance
JAW_FACE = 0.0003         # closed jaw inner face offset
NOSE_X = 0.068            # jaw nose tip
SLIP_TRAVEL = 0.012       # total slip-joint travel between two positions
OPEN_LIMIT = math.radians(28.0)

# Spring dimensions
SPRING_LEN = 0.045        # leaf spring length
SPRING_W = 0.006          # spring width
SPRING_T = 0.0012         # spring thickness

# Steel handle tang centerline (for the half whose jaw is +Y).
TANG_PTS = [
    (-0.010, -0.004),
    (-0.030, -0.011),
    (-0.070, -0.019),
    (-0.100, -0.024),
    (-0.118, -0.026),
]
CENTERLINE = TANG_PTS + [(-0.131, -0.0274)]

# Grip loft stations
GRIP_SECTIONS = [
    (-0.032, 0.0095, 0.0125),
    (-0.040, 0.0078, 0.0108),
    (-0.060, 0.0076, 0.0106),
    (-0.085, 0.0078, 0.0110),
    (-0.105, 0.0084, 0.0116),
    (-0.120, 0.0086, 0.0118),
    (-0.128, 0.0060, 0.0084),
    (-0.131, 0.0026, 0.0038),
]

# Inlay stations (blue accent for slip-joint variant)
INLAY_XS = [-0.036, -0.055, -0.075, -0.095, -0.112, -0.118]
INLAY_HALF_H = 0.0072
INLAY_Z_CENTER = 0.006

TANG_HALF_W = 0.004


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


def _jaw_solid(s: int, z_offset: float, z_half: float) -> cq.Workplane:
    """Polished jaw with serrations, gripping recess, and cutting notch.
    
    z_offset: center Z of this half's layer
    z_half: half-thickness of this half's layer
    """
    profile = [
        (0.010, JAW_FACE),
        (NOSE_X, JAW_FACE),
        (NOSE_X, 0.0100),
        (0.060, 0.0114),
        (0.048, 0.0130),
        (0.036, 0.0144),
        (0.024, 0.0154),
        (0.014, 0.0158),
        (0.009, 0.0142),
    ]
    pts = [(x, s * y) for x, y in profile]
    jaw = (
        cq.Workplane("XY")
        .polyline(pts)
        .close()
        .extrude(z_half)
        .translate((0.0, 0.0, z_offset - z_half / 2.0))
    )

    # Horizontal gripping serrations on inner face
    for i in range(6):
        xi = 0.042 + 0.004 * i
        groove = (
            cq.Workplane("XY")
            .box(0.0012, 0.0020, 0.020)
            .translate((xi, s * JAW_FACE, z_offset))
        )
        jaw = jaw.cut(groove)

    # Rounded gripping recess behind nose
    recess = (
        cq.Workplane("XY")
        .circle(0.004)
        .extrude(0.012, both=True)
        .translate((0.028, s * JAW_FACE, z_offset))
    )
    jaw = jaw.cut(recess)

    # Small scallop teeth around recess
    for ang_deg in (45.0, 80.0, 110.0, 145.0):
        a = math.radians(ang_deg)
        sx = 0.028 + 0.004 * math.cos(a)
        sy = s * (JAW_FACE + 0.004 * math.sin(a))
        scallop = (
            cq.Workplane("XY")
            .circle(0.0007)
            .extrude(0.012, both=True)
            .translate((sx, sy, z_offset))
        )
        jaw = jaw.cut(scallop)

    # Wire-cutter notch near pivot
    notch_pts = [(0.014, s * -0.001), (0.020, s * -0.001), (0.017, s * 0.003)]
    notch = (
        cq.Workplane("XY")
        .polyline(notch_pts)
        .close()
        .extrude(0.012, both=True)
        .translate((0.0, 0.0, z_offset))
    )
    jaw = jaw.cut(notch)

    return jaw


def _hub_with_slot() -> cq.Workplane:
    """Hub with elongated slip-joint slot, occupying the lower half (z <= -EPS)."""
    z_center = -(HALF_T + EPS) / 2.0  # center of lower half
    hub = (
        cq.Workplane("XY")
        .circle(HUB_R)
        .extrude(HALF_T)
        .translate((0.0, 0.0, z_center - HALF_T / 2.0))
    )
    # Elongated rounded (obround) slot along X axis (the slip-joint travel
    # direction). Rounded walls hug the pin barrel with a small sliding
    # clearance so the pivot pin is captured snugly rather than rattling in an
    # oversized rectangular hole; the curved walls are densely tessellated and
    # sit ~SLOT_CLEAR from the pin surface all along its length.
    slot = (
        cq.Workplane("XY")
        .slot2D(2.0 * SLOT_HALF_LEN + SLIP_TRAVEL, 2.0 * SLOT_R, 0.0)
        .extrude(HALF_T * 3.0, both=True)
        .translate((SLIP_TRAVEL / 2.0, 0.0, z_center))
    )
    hub = hub.cut(slot)
    return hub


def _hub_moving() -> cq.Workplane:
    """Round hub for the pivoting half, occupying the upper half (z >= +EPS)."""
    z_center = (HALF_T + EPS) / 2.0  # center of upper half
    return (
        cq.Workplane("XY")
        .circle(HUB_R)
        .extrude(HALF_T)
        .translate((0.0, 0.0, z_center - HALF_T / 2.0))
    )


def _jaw_hub_solid_slot() -> cq.Workplane:
    """Combined jaw + hub with slot for the slot_half (single forged piece)."""
    jaw = _jaw_solid(1)
    hub = _hub_with_slot()
    return jaw.union(hub)


def _jaw_hub_solid_moving() -> cq.Workplane:
    """Combined jaw + hub for the moving half (single forged piece)."""
    jaw = _jaw_solid(-1)
    hub = _hub_moving()
    return jaw.union(hub)


def _shank_solid(s: int, z_offset: float, z_half: float) -> cq.Workplane:
    """Steel handle tang sweeping back from the hub.
    
    z_offset: center Z of this half's layer
    z_half: half-thickness of this half's layer
    """
    # Start the tang from the hub region to ensure connectivity
    # Include points that extend into the hub area (X >= -0.014)
    pts = [(x, s * y) for x, y in TANG_PTS]
    # Ensure we start from near the hub
    if pts[0][0] > -0.012:
        pts = [(-0.012, s * -0.003)] + pts
    loop = _strip_loop(pts, TANG_HALF_W)
    return (
        cq.Workplane("XY")
        .polyline(loop)
        .close()
        .extrude(z_half)
        .translate((0.0, 0.0, z_offset - z_half / 2.0))
    )


def _ellipse_wire(x: float, yc: float, zc: float, w: float, h: float) -> cq.Wire:
    wp = cq.Workplane("YZ", origin=(x, 0.0, 0.0)).center(yc, zc).ellipse(w, h)
    return wp.val()


def _grip_solid(s: int) -> cq.Solid:
    """Soft-touch rubber grip with flared guard and bulbous end."""
    wires = [
        _ellipse_wire(x, s * _yc(x), 0.0, w, h)
        for x, w, h in GRIP_SECTIONS
    ]
    return cq.Solid.makeLoft(wires, ruled=False)


def _inlay_solid(s: int) -> cq.Solid:
    """Accent inlay along the outer/top face of the grip."""
    wires = [
        _ellipse_wire(x, s * _yc(x), INLAY_Z_CENTER, _grip_w(x) - 0.0012, INLAY_HALF_H)
        for x in INLAY_XS
    ]
    return cq.Solid.makeLoft(wires, ruled=False)


def _pivot_pin_solid() -> cq.Workplane:
    """The pivot pin shaft that passes through both halves."""
    total_len = 2.0 * HALF_T + 2.0 * CAP_H + 0.002
    return (
        cq.Workplane("XY")
        .circle(PIN_R)
        .extrude(total_len, both=True)
    )


def _spring_solid() -> cq.Workplane:
    """Leaf return spring: curved strip, anchor at local origin, extending -X."""
    # Local frame: anchor at (0,0,0), spring extends toward -X
    # Z goes from 0 (bottom, sitting on shank surface) upward
    spring_profile = [
        (0.0, 0.0),
        (-0.012, -0.002),
        (-0.025, -0.002),
        (-0.035, 0.001),
        (-0.040, 0.004),
        (-0.035, 0.004 + SPRING_T),
        (-0.025, SPRING_T),
        (-0.012, SPRING_T),
        (0.0, SPRING_T),
    ]
    spring = (
        cq.Workplane("XZ")
        .polyline(spring_profile)
        .close()
        .extrude(SPRING_W, both=True)
    )
    return spring


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slip_joint_pliers")

    steel_polished = model.material("steel_polished", rgba=(0.78, 0.80, 0.83, 1.0))
    steel_forged = model.material("steel_forged", rgba=(0.62, 0.64, 0.67, 1.0))
    steel_brushed = model.material("steel_brushed", rgba=(0.58, 0.60, 0.63, 1.0))
    spring_steel = model.material("spring_steel", rgba=(0.72, 0.73, 0.70, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.07, 0.07, 0.08, 1.0))
    grip_blue = model.material("grip_blue", rgba=(0.12, 0.35, 0.72, 1.0))

    # ---- slot_half (root): has the elongated slip-joint slot ----
    slot_half = model.part("slot_half")

    # Combine jaw and hub into a single forged piece for slot_half
    slot_jaw_hub = _jaw_solid(1, z_offset=-(HALF_T + EPS) / 2.0, z_half=HALF_T).union(
        _hub_with_slot()
    )
    slot_half.visual(
        mesh_from_cadquery(slot_jaw_hub, "slot_jaw_hub", tolerance=0.0002),
        name="jaw_hub",
        material=steel_polished,
    )
    slot_half.visual(
        mesh_from_cadquery(_shank_solid(1, z_offset=-(HALF_T + EPS) / 2.0, z_half=HALF_T), "slot_shank", tolerance=0.0002),
        name="shank",
        material=steel_forged,
    )
    slot_half.visual(
        mesh_from_cadquery(_grip_solid(1), "slot_grip", tolerance=0.0002),
        name="grip",
        material=rubber_black,
    )
    slot_half.visual(
        mesh_from_cadquery(_inlay_solid(1), "slot_inlay", tolerance=0.0002),
        name="grip_inlay",
        material=grip_blue,
    )

    # ---- jaw_half: the pivoting half ----
    jaw_half = model.part("jaw_half")

    # Combine jaw and hub into a single forged piece for jaw_half
    jaw_half_jaw_hub = _jaw_solid(-1, z_offset=(HALF_T + EPS) / 2.0, z_half=HALF_T).union(
        _hub_moving()
    )
    jaw_half.visual(
        mesh_from_cadquery(jaw_half_jaw_hub, "moving_jaw_hub", tolerance=0.0002),
        name="jaw_hub",
        material=steel_polished,
    )
    jaw_half.visual(
        mesh_from_cadquery(_shank_solid(-1, z_offset=(HALF_T + EPS) / 2.0, z_half=HALF_T), "moving_shank", tolerance=0.0002),
        name="shank",
        material=steel_forged,
    )
    jaw_half.visual(
        mesh_from_cadquery(_grip_solid(-1), "moving_grip", tolerance=0.0002),
        name="grip",
        material=rubber_black,
    )
    jaw_half.visual(
        mesh_from_cadquery(_inlay_solid(-1), "moving_inlay", tolerance=0.0002),
        name="grip_inlay",
        material=grip_blue,
    )

    # ---- pivot_pin: separate part with circular caps on both sides ----
    pivot_pin = model.part("pivot_pin")

    # Pin shaft
    pivot_pin.visual(
        mesh_from_cadquery(_pivot_pin_solid(), "pin_shaft", tolerance=0.0002),
        name="shaft",
        material=steel_brushed,
    )
    # Bottom cap (circular rivet cap)
    bottom_z = -(HALF_T + CAP_H / 2.0 + 0.0005)
    pivot_pin.visual(
        Cylinder(CAP_R, CAP_H),
        origin=Origin(xyz=(0.0, 0.0, bottom_z)),
        name="bottom_cap",
        material=steel_brushed,
    )
    # Top cap (circular rivet cap)
    top_z = HALF_T + CAP_H / 2.0 + 0.0005
    pivot_pin.visual(
        Cylinder(CAP_R, CAP_H),
        origin=Origin(xyz=(0.0, 0.0, top_z)),
        name="top_cap",
        material=steel_brushed,
    )

    # ---- return_spring: leaf spring anchored to slot_half ----
    return_spring = model.part("return_spring")

    return_spring.visual(
        mesh_from_cadquery(_spring_solid(), "spring_leaf", tolerance=0.0002),
        name="spring_body",
        material=spring_steel,
    )

    # ---- Articulations ----

    # 1. Slip joint: PRISMATIC, moves pivot_pin between two positions along X
    # Position 0 = closer to jaws (wider opening), position max = farther
    model.articulation(
        "slip_joint",
        ArticulationType.PRISMATIC,
        parent=slot_half,
        child=pivot_pin,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0, velocity=0.1, lower=0.0, upper=SLIP_TRAVEL,
        ),
    )

    # 2. Pivot: REVOLUTE, jaw_half rotates around the pivot_pin
    # axis perpendicular to tool plane, positive q opens jaws
    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=pivot_pin,
        child=jaw_half,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=3.0, lower=0.0, upper=OPEN_LIMIT,
        ),
    )

    # 3. Spring flex: REVOLUTE mimic of pivot, the spring arcs as handles open
    model.articulation(
        "spring_flex",
        ArticulationType.REVOLUTE,
        parent=slot_half,
        child=return_spring,
        origin=Origin(xyz=(0.0, 0.0, HALF_T)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=0.0, upper=math.radians(12.0),
        ),
        mimic=Mimic(joint="pivot", multiplier=0.4, offset=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    slot_half = object_model.get_part("slot_half")
    jaw_half = object_model.get_part("jaw_half")
    pivot_pin = object_model.get_part("pivot_pin")
    return_spring = object_model.get_part("return_spring")

    slip = object_model.get_articulation("slip_joint")
    pivot = object_model.get_articulation("pivot")
    spring_flex = object_model.get_articulation("spring_flex")

    # ---- Structural checks ----

    # Slip joint is prismatic with two-position travel
    slip_limits = slip.motion_limits
    ctx.check(
        "slip_joint is prismatic with ~12mm travel",
        slip.articulation_type == ArticulationType.PRISMATIC
        and slip_limits is not None
        and abs(slip_limits.lower) < 1e-9
        and 0.008 <= (slip_limits.upper or 0.0) <= 0.016,
        details=f"type={slip.articulation_type} limits={slip_limits}",
    )

    # Pivot is revolute opening 0..28 degrees
    pivot_limits = pivot.motion_limits
    ctx.check(
        "pivot is revolute 0..28 degrees",
        pivot.articulation_type == ArticulationType.REVOLUTE
        and pivot_limits is not None
        and abs(pivot_limits.lower) < 1e-9
        and abs((pivot_limits.upper or 0.0) - OPEN_LIMIT) < 1e-6,
        details=f"limits={pivot_limits}",
    )

    # Spring flex is a mimic of the pivot
    ctx.check(
        "spring_flex mimics the pivot joint",
        spring_flex.mimic is not None
        and spring_flex.mimic.joint == "pivot",
        details=f"mimic={spring_flex.mimic}",
    )

    # ---- Circular pivot rivet caps on both sides ----
    top_cap_aabb = ctx.part_element_world_aabb(pivot_pin, elem="top_cap")
    bottom_cap_aabb = ctx.part_element_world_aabb(pivot_pin, elem="bottom_cap")
    ok_caps = top_cap_aabb is not None and bottom_cap_aabb is not None
    if ok_caps:
        cap_dia = top_cap_aabb[1][0] - top_cap_aabb[0][0]
        ctx.check(
            "pivot rivet caps ~25mm diameter on both sides",
            0.022 <= cap_dia <= 0.028
            and bottom_cap_aabb[0][2] < -HALF_T
            and top_cap_aabb[1][2] > HALF_T,
            details=f"dia={cap_dia:.4f} top_z={top_cap_aabb[1][2]:.4f} bot_z={bottom_cap_aabb[0][2]:.4f}",
        )
    else:
        ctx.fail("pivot cap AABBs resolve", "missing cap elements")

    # ---- Pivot pin passes through both halves' jaw_hub pieces ----
    # The pin shaft is intentionally captured through the jaw_hub of each half
    ctx.allow_overlap(
        pivot_pin,
        slot_half,
        elem_a="shaft",
        elem_b="jaw_hub",
        reason="The pivot pin shaft passes through the slot_half jaw_hub slot, capturing it at the slip joint.",
    )
    ctx.allow_overlap(
        pivot_pin,
        jaw_half,
        elem_a="shaft",
        elem_b="jaw_hub",
        reason="The pivot pin shaft passes through the jaw_half jaw_hub, forming the pivot axis.",
    )

    ctx.expect_within(
        pivot_pin,
        slot_half,
        axes="xy",
        inner_elem="shaft",
        outer_elem="jaw_hub",
        margin=0.002,
        name="pin shaft centered in slot_half jaw_hub",
    )
    ctx.expect_within(
        pivot_pin,
        jaw_half,
        axes="xy",
        inner_elem="shaft",
        outer_elem="jaw_hub",
        margin=0.002,
        name="pin shaft centered in jaw_half jaw_hub",
    )

    # ---- Jaw_hub overlap at pivot (both halves share the pivot footprint) ----
    ctx.expect_overlap(
        slot_half,
        jaw_half,
        axes="xy",
        elem_a="jaw_hub",
        elem_b="jaw_hub",
        min_overlap=0.018,
        name="jaw_hub pieces share the pivot footprint",
    )

    # ---- Closed pose: jaw_hub pieces are Z-separated at the hub ----
    # The two halves interleave at the pivot: one occupies the lower Z layer,
    # the other the upper Z layer, with a small clearance gap
    ctx.expect_gap(
        jaw_half,
        slot_half,
        axis="z",
        positive_elem="jaw_hub",
        negative_elem="jaw_hub",
        min_gap=0.0,
        max_gap=0.001,
        name="jaw_hub pieces are Z-separated at the pivot (half-lapped joint)",
    )

    # ---- Slip joint changes pivot position ----
    pin_pos_rest = ctx.part_world_position(pivot_pin)
    with ctx.pose({slip: SLIP_TRAVEL}):
        pin_pos_slipped = ctx.part_world_position(pivot_pin)
    ctx.check(
        "slip_joint moves the pivot pin along X",
        pin_pos_rest is not None
        and pin_pos_slipped is not None
        and pin_pos_slipped[0] > pin_pos_rest[0] + 0.008,
        details=f"rest={pin_pos_rest} slipped={pin_pos_slipped}",
    )

    # ---- Open pose: jaws separate and handles spread ----
    closed_jaw_aabb = ctx.part_element_world_aabb(jaw_half, elem="jaw_hub")
    closed_grip_aabb = ctx.part_element_world_aabb(jaw_half, elem="grip")
    with ctx.pose({pivot: OPEN_LIMIT}):
        open_jaw_aabb = ctx.part_element_world_aabb(jaw_half, elem="jaw_hub")
        open_grip_aabb = ctx.part_element_world_aabb(jaw_half, elem="grip")
        # Check that the jaw tips separate (the jaw_half jaw is on -Y side,
        # so when it opens, its min Y should move more negative)
        ctx.check(
            "jaws open apart at full pivot angle",
            closed_jaw_aabb is not None
            and open_jaw_aabb is not None
            and open_jaw_aabb[0][1] < closed_jaw_aabb[0][1] - 0.015,
            details=f"closed_min_y={closed_jaw_aabb[0][1]:.4f} open_min_y={open_jaw_aabb[0][1]:.4f}",
        )
        ctx.check(
            "moving jaw swings away from fixed jaw",
            closed_jaw_aabb is not None
            and open_jaw_aabb is not None
            and open_jaw_aabb[0][1] < closed_jaw_aabb[0][1] - 0.015,
            details=f"closed={closed_jaw_aabb} open={open_jaw_aabb}",
        )
        ctx.check(
            "handles spread as jaws open",
            closed_grip_aabb is not None
            and open_grip_aabb is not None
            and open_grip_aabb[1][1] > closed_grip_aabb[1][1] + 0.025,
            details=f"closed={closed_grip_aabb} open={open_grip_aabb}",
        )

    # ---- Return spring arcs with pivot motion ----
    spring_rest = ctx.part_element_world_aabb(return_spring, elem="spring_body")
    with ctx.pose({pivot: OPEN_LIMIT}):
        spring_open = ctx.part_element_world_aabb(return_spring, elem="spring_body")
    ctx.check(
        "return spring rotates with pivot motion",
        spring_rest is not None
        and spring_open is not None
        and abs(spring_open[0][1] - spring_rest[0][1]) > 0.001,
        details=f"rest={spring_rest} open={spring_open}",
    )

    # ---- Overall envelope: ~0.20 m long ----
    a0 = ctx.part_world_aabb(slot_half)
    a1 = ctx.part_world_aabb(jaw_half)
    if a0 is not None and a1 is not None:
        length = max(a0[1][0], a1[1][0]) - min(a0[0][0], a1[0][0])
        ctx.check(
            "overall length about 0.20 m",
            0.18 <= length <= 0.22,
            details=f"length={length:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
