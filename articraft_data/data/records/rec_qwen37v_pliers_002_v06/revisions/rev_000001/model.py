from __future__ import annotations

# Bent-nose slip-joint pliers variant.
# Variant 06 of the lineman pliers family.
#
# Changes from parent:
# 1. Bent-nose jaws (45° bend toward +Y for both halves)
# 2. Slip-joint: elongated slot in half_0 hub, separate pivot_pin part
#    with prismatic + revolute articulation chain
# 3. Circular pivot rivet caps on both sides of the pin
#
# Layout: the tool lies in the XY plane with Z as the thickness axis.
# The pivot is at the origin. The blunt nose points +X (then bends toward +Y),
# the handles sweep back to -X and spread in +/-Y.

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
LAP_R = 0.016           # half-lap joint disc radius at the pivot
HUB_R = 0.015           # forged hub radius around the pivot
BOSS_R = 0.0125         # rivet boss radius
BOSS_H = 0.0030
SEAM_R = 0.0132
SEAM_H = 0.0006
PIN_R = 0.004           # pivot pin shaft radius
CAP_R = 0.009           # circular rivet cap radius (both sides)
CAP_H = 0.003           # cap height
EPS = 0.0001            # lap clearance
JAW_FACE = 0.0003       # closed jaw inner face offset from center
OPEN_LIMIT = math.radians(30.0)
SLOT_TRAVEL = 0.006     # slip-joint prismatic travel (meters)

# Slot dimensions (elongated hole in half_0 hub, runs along X)
SLOT_LENGTH = 0.014     # total slot length along X
SLOT_WIDTH = 0.009      # slot width along Y

BEND_ANGLE = math.radians(45.0)  # jaw bend angle
BEND_X = 0.036          # X position where jaw bend starts

TANG_HALF_W = 0.004

# Handle tang centerline (for jaw-on-+Y half, s=+1)
TANG_PTS = [
    (-0.010, -0.004),
    (-0.030, -0.011),
    (-0.070, -0.019),
    (-0.100, -0.024),
    (-0.118, -0.026),
]
CENTERLINE = TANG_PTS + [(-0.131, -0.0274)]

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

INLAY_XS = [-0.036, -0.055, -0.075, -0.095, -0.112, -0.118]
INLAY_HALF_H = 0.0075
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
    """Material removed at the pivot so this half keeps only its lap layer."""
    cut = cq.Workplane("XY").circle(LAP_R).extrude(0.012)
    if s > 0:
        return cut.translate((0.0, 0.0, -EPS))
    return cut.translate((0.0, 0.0, -0.012 + EPS))


def _slot_cut() -> cq.Workplane:
    """Elongated slot through half_0's hub for the slip-joint pin."""
    # Build slot as rounded rectangle: rect body + circle endcaps
    half_len = (SLOT_LENGTH - SLOT_WIDTH) / 2.0
    half_w = SLOT_WIDTH / 2.0
    # Center rectangle
    slot = (
        cq.Workplane("XY")
        .rect(SLOT_LENGTH - SLOT_WIDTH, SLOT_WIDTH)
        .extrude(0.030, both=True)
    )
    # End circles
    for dx in (-half_len, half_len):
        cap = (
            cq.Workplane("XY")
            .circle(half_w)
            .extrude(0.030, both=True)
            .translate((dx, 0.0, 0.0))
        )
        slot = slot.union(cap)
    # Offset slot center along +X so pin at q=0 is near the rear of the slot
    return slot.translate((SLOT_TRAVEL / 2.0, 0.0, 0.0))


def _jaw_solid(s: int) -> cq.Workplane:
    """Bent-nose jaw: straight base near pivot + 45° angled tip.

    Both halves bend in the same direction (+Y in world space).
    Each jaw is half-thickness in Z (s=+1 lower, s=-1 upper) so the
    two halves interleave like the hub lap joint throughout the jaw region.
    """
    cos_a = math.cos(BEND_ANGLE)
    sin_a = math.sin(BEND_ANGLE)

    jaw_thick = HALF_T - EPS  # half-thickness per jaw plate

    def _extrude_half(profile_pts: list) -> cq.Workplane:
        """Extrude a polyline profile to one side of the XY plane."""
        solid = (
            cq.Workplane("XY")
            .polyline(profile_pts)
            .close()
            .extrude(jaw_thick)
        )
        if s > 0:
            # Lower half: z from -HALF_T to -EPS
            return solid.translate((0.0, 0.0, -HALF_T))
        else:
            # Upper half: z from +EPS to +HALF_T
            return solid.translate((0.0, 0.0, EPS))

    # === Straight base section (from hub to bend point + overlap) ===
    base_profile = [
        (0.010, JAW_FACE),
        (BEND_X + 0.005, JAW_FACE),
        (BEND_X + 0.005, 0.0120),
        (0.026, 0.0162),
        (0.014, 0.0166),
        (0.009, 0.0150),
    ]
    base_pts = [(x, s * y) for x, y in base_profile]
    base = _extrude_half(base_pts)

    # === Bent nose section ===
    # Both jaws bend toward +Y. Build in local frame then rotate.
    # Bend center: (BEND_X, s * 0.008) in world
    bcx = BEND_X
    bcy = s * 0.008

    # Local-to-world: local X → bend direction (cos_a, sin_a)
    # local Y → outward perpendicular = s * (-sin_a, cos_a)
    out_x = s * (-sin_a)
    out_y = s * cos_a

    def l2w(lx: float, ly: float) -> tuple[float, float]:
        return (bcx + lx * cos_a + ly * out_x,
                bcy + lx * sin_a + ly * out_y)

    nose_length = 0.025
    hw_s = 0.007   # half-width at bend start
    hw_e = 0.004   # half-width at tip (tapers)

    local_profile = [
        (-0.003, -hw_s),          # inner start (overlap zone)
        (nose_length, -hw_e),     # inner tip
        (nose_length + 0.002, 0.0),  # rounded tip center
        (nose_length, hw_e),      # outer tip
        (-0.003, hw_s),           # outer start
    ]
    world_profile = [l2w(lx, ly) for lx, ly in local_profile]
    nose = _extrude_half(world_profile)

    jaw = base.union(nose)

    # Serrations on the straight inner face (fine horizontal grooves)
    for i in range(4):
        xi = 0.014 + 0.004 * i
        groove = (
            cq.Workplane("XY")
            .box(0.0012, 0.002, 0.020)
            .translate((xi, s * JAW_FACE, 0.0))
        )
        jaw = jaw.cut(groove)

    # Serrations on the bent inner face
    for i in range(3):
        t = 0.005 + 0.006 * i
        gx, gy = l2w(t, -hw_s + 0.001)
        groove = (
            cq.Workplane("XY")
            .box(0.001, 0.002, 0.020)
            .translate((gx, gy, 0.0))
        )
        jaw = jaw.cut(groove)

    # Pipe-grip recess in the bent section
    gx, gy = l2w(0.010, 0.0)
    recess = (
        cq.Workplane("XY")
        .circle(0.003)
        .extrude(0.012, both=True)
        .translate((gx, gy, 0.0))
    )
    jaw = jaw.cut(recess)

    # Wire-cutter V-notch near the pivot (straight section)
    notch_pts = [
        (0.013, s * -0.001),
        (0.019, s * -0.001),
        (0.016, s * 0.003),
    ]
    notch = cq.Workplane("XY").polyline(notch_pts).close().extrude(0.012, both=True)
    jaw = jaw.cut(notch)

    return jaw


def _hub_solid(s: int, has_slot: bool = False) -> cq.Workplane:
    """Forged circular hub around the pivot, thinned to its lap layer."""
    hub = cq.Workplane("XY").circle(HUB_R).extrude(HALF_T, both=True)
    hub = hub.cut(_lap_cut(s))
    if has_slot:
        hub = hub.cut(_slot_cut())
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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bent_nose_slip_joint_pliers")

    steel_polished = model.material("steel_polished", rgba=(0.80, 0.81, 0.84, 1.0))
    steel_forged = model.material("steel_forged", rgba=(0.66, 0.67, 0.70, 1.0))
    steel_brushed = model.material("steel_brushed", rgba=(0.61, 0.62, 0.65, 1.0))
    seam_gray = model.material("seam_gray", rgba=(0.45, 0.46, 0.48, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.08, 0.09, 1.0))
    grip_red = model.material("grip_red", rgba=(0.80, 0.09, 0.11, 1.0))

    # --- Half 0 (root, has the slip-joint slot) ---
    half0 = model.part("plier_half_0")
    half0.visual(
        mesh_from_cadquery(_jaw_solid(1), "half0_jaw", tolerance=0.0002),
        name="jaw",
        material=steel_polished,
    )
    half0.visual(
        mesh_from_cadquery(_hub_solid(1, has_slot=True), "half0_hub", tolerance=0.0002),
        name="hub",
        material=steel_forged,
    )
    half0.visual(
        mesh_from_cadquery(_shank_solid(1), "half0_shank", tolerance=0.0002),
        name="shank",
        material=steel_forged,
    )
    half0.visual(
        mesh_from_cadquery(_grip_solid(1), "half0_grip", tolerance=0.0002),
        name="grip",
        material=rubber_black,
    )
    half0.visual(
        mesh_from_cadquery(_inlay_solid(1), "half0_inlay", tolerance=0.0002),
        name="grip_inlay",
        material=grip_red,
    )

    # --- Half 1 (moving half, rotates on the pin) ---
    half1 = model.part("plier_half_1")
    half1.visual(
        mesh_from_cadquery(_jaw_solid(-1), "half1_jaw", tolerance=0.0002),
        name="jaw",
        material=steel_polished,
    )
    half1.visual(
        mesh_from_cadquery(_hub_solid(-1, has_slot=False), "half1_hub", tolerance=0.0002),
        name="hub",
        material=steel_forged,
    )
    half1.visual(
        mesh_from_cadquery(_shank_solid(-1), "half1_shank", tolerance=0.0002),
        name="shank",
        material=steel_forged,
    )
    half1.visual(
        mesh_from_cadquery(_grip_solid(-1), "half1_grip", tolerance=0.0002),
        name="grip",
        material=rubber_black,
    )
    half1.visual(
        mesh_from_cadquery(_inlay_solid(-1), "half1_inlay", tolerance=0.0002),
        name="grip_inlay",
        material=grip_red,
    )

    # --- Pivot pin (slides in half_0's slot, half_1 rotates on it) ---
    pivot_pin = model.part("pivot_pin")

    # Pin shaft passes through both halves
    shaft_h = 2.0 * HALF_T + 2.0 * EPS + 0.004
    pivot_pin.visual(
        Cylinder(PIN_R, shaft_h),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="pin_shaft",
        material=steel_brushed,
    )

    # Bottom circular rivet cap (below half_0)
    cap_z_bottom = -(shaft_h / 2.0 + CAP_H / 2.0)
    pivot_pin.visual(
        Cylinder(CAP_R, CAP_H),
        origin=Origin(xyz=(0.0, 0.0, cap_z_bottom)),
        name="rivet_cap_bottom",
        material=steel_brushed,
    )
    # Seam ring under bottom cap
    pivot_pin.visual(
        Cylinder(SEAM_R, SEAM_H),
        origin=Origin(xyz=(0.0, 0.0, cap_z_bottom + CAP_H / 2.0 + SEAM_H / 2.0)),
        name="seam_bottom",
        material=seam_gray,
    )

    # Top circular rivet cap (above half_1)
    cap_z_top = shaft_h / 2.0 + CAP_H / 2.0
    pivot_pin.visual(
        Cylinder(CAP_R, CAP_H),
        origin=Origin(xyz=(0.0, 0.0, cap_z_top)),
        name="rivet_cap_top",
        material=steel_brushed,
    )
    # Seam ring under top cap
    pivot_pin.visual(
        Cylinder(SEAM_R, SEAM_H),
        origin=Origin(xyz=(0.0, 0.0, cap_z_top - CAP_H / 2.0 - SEAM_H / 2.0)),
        name="seam_top",
        material=seam_gray,
    )

    # --- Articulation chain ---
    # 1. Prismatic: half_0 -> pivot_pin (pin slides in slot along X)
    #    At q=0, pin is at the rear of the slot (slot is offset +X/2).
    #    Positive q slides the pin toward +X (forward in the slot).
    model.articulation(
        "slot_slide",
        ArticulationType.PRISMATIC,
        parent=half0,
        child=pivot_pin,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=50.0, velocity=0.5,
            lower=0.0, upper=SLOT_TRAVEL,
        ),
    )

    # 2. Revolute: pivot_pin -> half_1 (half_1 rotates around the pin)
    #    Positive q (about -Z) opens the jaws.
    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=pivot_pin,
        child=half1,
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

    half0 = object_model.get_part("plier_half_0")
    half1 = object_model.get_part("plier_half_1")
    pivot_pin = object_model.get_part("pivot_pin")
    slot_slide = object_model.get_articulation("slot_slide")
    pivot = object_model.get_articulation("pivot")

    jaw0 = half0.get_visual("jaw")
    jaw1 = half1.get_visual("jaw")
    hub0 = half0.get_visual("hub")
    hub1 = half1.get_visual("hub")
    grip0 = half0.get_visual("grip")
    grip1 = half1.get_visual("grip")

    # === Joint contract ===

    # Prismatic slip-joint: 0 to SLOT_TRAVEL meters along X
    slide_limits = slot_slide.motion_limits
    ctx.check(
        "slot_slide is a prismatic joint with correct travel",
        slot_slide.articulation_type == ArticulationType.PRISMATIC
        and slide_limits is not None
        and abs(slide_limits.lower) < 1e-9
        and abs(slide_limits.upper - SLOT_TRAVEL) < 1e-6,
        details=f"type={slot_slide.articulation_type}, limits={slide_limits}",
    )

    # Revolute pivot: 0 to 30 degrees
    pivot_limits = pivot.motion_limits
    ctx.check(
        "pivot is a 0..30 degree revolute joint",
        pivot.articulation_type == ArticulationType.REVOLUTE
        and pivot_limits is not None
        and abs(pivot_limits.lower) < 1e-9
        and abs(pivot_limits.upper - OPEN_LIMIT) < 1e-6,
        details=f"limits={pivot_limits}",
    )

    # === Bent nose geometry ===

    # The jaw tips should be offset from the straight jaw axis (bent toward +Y).
    # At rest, both jaw tips should have significant +Y extent beyond the
    # straight section's outer face.
    jaw0_aabb = ctx.part_element_world_aabb(half0, elem="jaw")
    jaw1_aabb = ctx.part_element_world_aabb(half1, elem="jaw")
    if jaw0_aabb is not None and jaw1_aabb is not None:
        # half_0's jaw max Y should be well beyond 0.016 (straight outer face)
        # due to the 45° bend toward +Y
        ctx.check(
            "half_0 jaw bends beyond straight outer face toward +Y",
            jaw0_aabb[1][1] >= 0.018,
            details=f"jaw0_max_y={jaw0_aabb[1][1]:.4f}",
        )
        # half_1's jaw also bends toward +Y: its max Y should be positive
        # (it started on -Y side but bent past center)
        ctx.check(
            "half_1 jaw bends toward +Y (crosses center line)",
            jaw1_aabb[1][1] >= 0.005,
            details=f"jaw1_max_y={jaw1_aabb[1][1]:.4f}",
        )
        # The bent nose tip extends beyond the straight jaw length
        # (the bend adds forward reach)
        ctx.check(
            "bent nose tip reaches past 0.050 m in X",
            jaw0_aabb[1][0] >= 0.050,
            details=f"jaw0_max_x={jaw0_aabb[1][0]:.4f}",
        )
    else:
        ctx.fail("jaw AABBs resolve", "missing jaw element AABB")

    # === Slot in half_0 hub ===

    # The hub should have a slot cut - verify the hub exists and has
    # reasonable size (slot reduces but doesn't eliminate the hub)
    hub0_aabb = ctx.part_element_world_aabb(half0, elem="hub")
    if hub0_aabb is not None:
        hub_dx = hub0_aabb[1][0] - hub0_aabb[0][0]
        hub_dy = hub0_aabb[1][1] - hub0_aabb[0][1]
        ctx.check(
            "half_0 hub has reasonable extent (slot does not destroy it)",
            hub_dx >= 0.015 and hub_dy >= 0.015,
            details=f"hub_dx={hub_dx:.4f}, hub_dy={hub_dy:.4f}",
        )
    else:
        ctx.fail("hub0 AABB resolves", "missing hub element AABB")

    # === Circular rivet caps on both sides ===

    cap_bot_aabb = ctx.part_element_world_aabb(pivot_pin, elem="rivet_cap_bottom")
    cap_top_aabb = ctx.part_element_world_aabb(pivot_pin, elem="rivet_cap_top")
    if cap_bot_aabb is not None and cap_top_aabb is not None:
        # Bottom cap should be below the tool (negative Z)
        ctx.check(
            "bottom rivet cap below the tool body",
            cap_bot_aabb[0][2] < -HALF_T,
            details=f"cap_bot_min_z={cap_bot_aabb[0][2]:.4f}",
        )
        # Top cap should be above the tool (positive Z)
        ctx.check(
            "top rivet cap above the tool body",
            cap_top_aabb[1][2] > HALF_T,
            details=f"cap_top_max_z={cap_top_aabb[1][2]:.4f}",
        )
        # Both caps should be circular with ~CAP_R diameter
        bot_dia = cap_bot_aabb[1][0] - cap_bot_aabb[0][0]
        top_dia = cap_top_aabb[1][0] - cap_top_aabb[0][0]
        ctx.check(
            "rivet caps are ~18 mm diameter circles",
            0.015 <= bot_dia <= 0.021 and 0.015 <= top_dia <= 0.021,
            details=f"bot_dia={bot_dia:.4f}, top_dia={top_dia:.4f}",
        )
    else:
        ctx.fail("rivet cap AABBs resolve", "missing rivet_cap element AABB")

    # === Half-lap at pivot ===

    ctx.expect_gap(
        half1, half0,
        axis="z",
        positive_elem=hub1,
        negative_elem=hub0,
        min_gap=0.0,
        max_gap=0.001,
        name="moving hub lap stacks above fixed hub lap",
    )
    ctx.expect_contact(
        half0, half1,
        elem_a=hub0, elem_b=hub1,
        contact_tol=0.0005,
        name="hub laps seat against each other at the joint",
    )
    ctx.expect_overlap(
        half0, half1,
        axes="xy",
        elem_a=hub0, elem_b=hub1,
        min_overlap=0.015,
        name="hub laps share the pivot footprint",
    )

    # === Pin captured in slot and through moving hub ===

    # Pin shaft overlaps half_0's hub (pin in slot)
    ctx.allow_overlap(
        pivot_pin, half0,
        elem_a="pin_shaft", elem_b="hub",
        reason="The pivot pin shaft is intentionally captured in half_0's "
               "slip-joint slot, representing the sliding fit.",
    )
    ctx.expect_overlap(
        pivot_pin, half0,
        axes="xy",
        elem_a="pin_shaft", elem_b="hub",
        min_overlap=0.003,
        name="pin shaft sits inside half_0 hub slot footprint",
    )

    # Pin shaft overlaps half_1's hub (pin through round hole)
    ctx.allow_overlap(
        pivot_pin, half1,
        elem_a="pin_shaft", elem_b="hub",
        reason="The pivot pin shaft intentionally passes through half_1's "
               "hub, capturing it for the revolute pivot.",
    )
    ctx.expect_within(
        pivot_pin, half1,
        axes="xy",
        inner_elem="pin_shaft", outer_elem="hub",
        margin=0.001,
        name="pin shaft stays within half_1 hub footprint",
    )
    ctx.expect_overlap(
        pivot_pin, half1,
        axes="z",
        elem_a="pin_shaft", elem_b="hub",
        min_overlap=0.007,
        name="pin shaft passes through half_1 hub thickness",
    )

    # === Decisive pose checks ===

    # Revolute open pose: jaws separate and handles spread
    closed_jaw1_y = jaw1_aabb[0][1] if jaw1_aabb is not None else None
    closed_grip1_y = None
    g1_aabb = ctx.part_element_world_aabb(half1, elem="grip")
    if g1_aabb is not None:
        closed_grip1_y = g1_aabb[1][1]

    with ctx.pose({pivot: OPEN_LIMIT}):
        ctx.expect_gap(
            half0, half1,
            axis="y",
            positive_elem=jaw0, negative_elem=jaw1,
            min_gap=0.003,
            name="jaws open apart at the 30 degree pose",
        )
        open_jaw1 = ctx.part_element_world_aabb(half1, elem="jaw")
        open_grip1 = ctx.part_element_world_aabb(half1, elem="grip")
        ctx.check(
            "moving jaw swings away from fixed jaw at open pose",
            closed_jaw1_y is not None
            and open_jaw1 is not None
            and open_jaw1[0][1] < closed_jaw1_y - 0.012,
            details=f"closed_min_y={closed_jaw1_y}, open_min_y={open_jaw1}",
        )
        ctx.check(
            "moving handle spreads outward at open pose",
            closed_grip1_y is not None
            and open_grip1 is not None
            and open_grip1[1][1] > closed_grip1_y + 0.02,
            details=f"closed_max_y={closed_grip1_y}, open_max_y={open_grip1}",
        )

    # Prismatic slide pose: pin moves forward in the slot
    rest_pin_pos = ctx.part_world_position(pivot_pin)
    with ctx.pose({slot_slide: SLOT_TRAVEL}):
        slid_pin_pos = ctx.part_world_position(pivot_pin)
        ctx.check(
            "pin slides forward along +X at max slot travel",
            rest_pin_pos is not None
            and slid_pin_pos is not None
            and slid_pin_pos[0] > rest_pin_pos[0] + SLOT_TRAVEL * 0.9,
            details=f"rest={rest_pin_pos}, slid={slid_pin_pos}",
        )

    # === Envelope checks ===

    a0 = ctx.part_world_aabb(half0)
    a1 = ctx.part_world_aabb(half1)
    g0 = ctx.part_element_world_aabb(half0, elem="grip")
    if a0 is not None and a1 is not None and g0 is not None and g1_aabb is not None:
        length = max(a0[1][0], a1[1][0]) - min(a0[0][0], a1[0][0])
        across = g1_aabb[1][1] - g0[0][1]
        ctx.check(
            "overall length about 0.20 m",
            0.18 <= length <= 0.22,
            details=f"length={length:.4f}",
        )
        ctx.check(
            "handle grips span about 0.06-0.08 m across",
            0.055 <= across <= 0.085,
            details=f"across={across:.4f}",
        )

    # Red inlay proud on grip top face
    i0 = ctx.part_element_world_aabb(half0, elem="grip_inlay")
    if i0 is not None and g0 is not None:
        ctx.check(
            "red inlay proud on the grip top face",
            i0[1][2] >= g0[1][2] - 0.0005,
            details=f"inlay_top={i0[1][2]:.4f} grip_top={g0[1][2]:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
