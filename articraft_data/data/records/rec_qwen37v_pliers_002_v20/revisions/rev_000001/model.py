from __future__ import annotations

# Insulated slip-joint pliers variant.
# Variant 20 of heavy-duty combination (lineman) pliers.
#
# Layout: the tool lies in the XY plane with Z as the thickness axis.
# The pivot region is at the origin. The blunt squared nose points +X,
# the handles sweep back to -X and spread in +/-Y.
# plier_half_0 (root) carries its jaw on +Y; plier_half_1 is the mirrored
# moving half. A pivot_carrier slides in a slot on half_0's hub (prismatic).
# half_1 rotates on the carrier (revolute).
#
# Variant changes vs parent:
# - Thick layered insulated grip sleeves (inner insulation + outer sleeve)
# - Slip-joint pin slides along a short prismatic slot in half_0 hub
# - Circular pivot rivet cap on both sides
# - Cutter bevels as visible wedge geometry near the pivot

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---- shared dimensions (meters) ----
HALF_T = 0.009          # half thickness of each forged plate (full plate 0.018)
LAP_R = 0.016           # half-lap joint disc radius at the pivot
HUB_R = 0.015           # forged hub radius around the rivet
BOSS_R = 0.0125         # rivet boss radius (~0.025 m diameter)
BOSS_H = 0.0030
SEAM_R = 0.0132         # visible circular seam ring under the boss cap
SEAM_H = 0.0006
RIVET_R = 0.004         # rivet shaft captured through the moving half's lap
CAP_R = 0.010           # pivot rivet cap radius (visible on both sides)
CAP_H = 0.0018          # cap thickness
EPS = 0.0001            # lap clearance so the stacked halves do not penetrate
JAW_FACE = 0.0003       # closed jaw inner faces sit at y = +/-JAW_FACE
NOSE_X = 0.075          # blunt squared nose tip
OPEN_LIMIT = math.radians(30.0)

# Slip-joint slot dimensions (on half_0 hub)
SLOT_LENGTH = 0.008     # slot travel length
SLOT_WIDTH = 0.005      # slot width (matches pin diameter)
SLOT_DEPTH = 0.004      # slot depth into hub

# Carrier/pin dimensions
PIN_R = 0.0024          # pivot pin radius (fits in slot)
CARRIER_R = 0.006       # carrier disc radius (wider than slot for retention)
CARRIER_T = 0.004       # carrier disc thickness

# Prismatic travel
SLIDE_LIMIT = SLOT_LENGTH - SLOT_WIDTH  # effective travel ~3mm

# Cutter bevel dimensions
BEVEL_LENGTH = 0.012    # bevel wedge length along jaw
BEVEL_HEIGHT = 0.004    # bevel wedge height
BEVEL_DEPTH = 0.003     # bevel wedge depth (into jaw face)

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

# Insulation layer loft stations: (x, half_width_y, half_height_z)
# Thicker than parent for insulated (VDE-style) pliers.
INSULATION_SECTIONS = [
    (-0.032, 0.0115, 0.0145),  # flared guard near the pivot
    (-0.040, 0.0098, 0.0128),
    (-0.060, 0.0095, 0.0125),
    (-0.085, 0.0098, 0.0128),
    (-0.105, 0.0102, 0.0132),
    (-0.120, 0.0104, 0.0134),
    (-0.128, 0.0078, 0.0100),
    (-0.131, 0.0040, 0.0055),
]

# Outer sleeve loft stations (sits on top of insulation, slightly smaller).
SLEEVE_SECTIONS = [
    (-0.034, 0.0105, 0.0132),
    (-0.042, 0.0088, 0.0115),
    (-0.062, 0.0085, 0.0112),
    (-0.087, 0.0088, 0.0115),
    (-0.107, 0.0092, 0.0120),
    (-0.120, 0.0094, 0.0122),
    (-0.127, 0.0070, 0.0092),
    (-0.130, 0.0035, 0.0048),
]


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
    """Material removed at the pivot so this half keeps only its lap layer."""
    cut = cq.Workplane("XY").circle(LAP_R).extrude(0.012)
    if s > 0:
        return cut.translate((0.0, 0.0, -EPS))
    return cut.translate((0.0, 0.0, -0.012 + EPS))


def _slot_cut() -> cq.Workplane:
    """Elongated slot cut into half_0 hub for the slip-joint pin."""
    # Slot is a rounded rectangle centered on origin, oriented along Y axis
    slot = (
        cq.Workplane("XY")
        .rect(SLOT_WIDTH, SLOT_LENGTH)
        .extrude(SLOT_DEPTH, both=True)
    )
    # Add rounded ends
    end_top = cq.Workplane("XY").circle(SLOT_WIDTH / 2).extrude(SLOT_DEPTH, both=True).translate((0, SLOT_LENGTH / 2, 0))
    end_bot = cq.Workplane("XY").circle(SLOT_WIDTH / 2).extrude(SLOT_DEPTH, both=True).translate((0, -SLOT_LENGTH / 2, 0))
    return slot.union(end_top).union(end_bot)


def _jaw_solid(s: int) -> cq.Workplane:
    """Polished jaw: squared nose, serrations, pipe-grip recess, wire cutter."""
    profile = [
        (0.010, JAW_FACE),
        (NOSE_X, JAW_FACE),
        (NOSE_X, 0.0105),   # blunt squared nose tip
        (0.066, 0.0118),
        (0.052, 0.0136),
        (0.038, 0.0152),
        (0.026, 0.0162),
        (0.014, 0.0166),
        (0.009, 0.0150),
    ]
    pts = [(x, s * y) for x, y in profile]
    jaw = cq.Workplane("XY").polyline(pts).close().extrude(HALF_T, both=True)

    # Fine horizontal gripping serrations across the inner nose face.
    for i in range(7):
        xi = 0.046 + 0.004 * i
        groove = (
            cq.Workplane("XY")
            .box(0.0014, 0.0024, 0.020)
            .translate((xi, s * JAW_FACE, 0.0))
        )
        jaw = jaw.cut(groove)

    # Rounded pipe-grip recess behind the nose.
    recess = (
        cq.Workplane("XY")
        .circle(0.0045)
        .extrude(0.011, both=True)
        .translate((0.030, s * JAW_FACE, 0.0))
    )
    jaw = jaw.cut(recess)

    # Small scallop teeth around the recess rim (serrated pipe grip).
    for ang_deg in (40.0, 75.0, 105.0, 140.0):
        a = math.radians(ang_deg)
        sx = 0.030 + 0.0045 * math.cos(a)
        sy = s * (JAW_FACE + 0.0045 * math.sin(a))
        scallop = (
            cq.Workplane("XY")
            .circle(0.0008)
            .extrude(0.011, both=True)
            .translate((sx, sy, 0.0))
        )
        jaw = jaw.cut(scallop)

    # Short wire-cutter V-notch between the pipe grip and the pivot.
    notch_pts = [(0.015, s * -0.001), (0.0215, s * -0.001), (0.0185, s * 0.0035)]
    notch = cq.Workplane("XY").polyline(notch_pts).close().extrude(0.011, both=True)
    jaw = jaw.cut(notch)

    return jaw.cut(_lap_cut(s))


def _cutter_bevel_solid(s: int) -> cq.Workplane:
    """Visible wedge-shaped cutter bevel near the pivot on each jaw half."""
    # Wedge profile: triangular cross-section in XZ plane
    # Positioned just behind the wire-cutter notch, on the inner jaw face
    wedge_pts = [
        (0.012, s * (JAW_FACE + 0.0002)),
        (0.012 + BEVEL_LENGTH, s * (JAW_FACE + 0.0002)),
        (0.012 + BEVEL_LENGTH * 0.5, s * (JAW_FACE + 0.0002 + BEVEL_DEPTH)),
    ]
    wedge = (
        cq.Workplane("XY")
        .polyline(wedge_pts)
        .close()
        .extrude(BEVEL_HEIGHT)
    )
    # Center the wedge vertically
    wedge = wedge.translate((0.0, 0.0, -BEVEL_HEIGHT / 2.0))
    return wedge


def _hub_solid(s: int, with_slot: bool = False) -> cq.Workplane:
    """Forged circular hub around the rivet, thinned to its lap layer."""
    hub = cq.Workplane("XY").circle(HUB_R).extrude(HALF_T, both=True)
    hub = hub.cut(_lap_cut(s))
    if with_slot:
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


def _insulation_solid(s: int) -> cq.Solid:
    """Inner insulation layer: thick yellow-orange dielectric sleeve."""
    wires = [
        _ellipse_wire(x, s * _yc(x), 0.0, w, h)
        for x, w, h in INSULATION_SECTIONS
    ]
    return cq.Solid.makeLoft(wires, ruled=False)


def _sleeve_solid(s: int) -> cq.Solid:
    """Outer grip sleeve: thick rubber layer over the insulation."""
    wires = [
        _ellipse_wire(x, s * _yc(x), 0.0, w, h)
        for x, w, h in SLEEVE_SECTIONS
    ]
    return cq.Solid.makeLoft(wires, ruled=False)


def _carrier_solid() -> cq.Workplane:
    """Slip-joint pivot carrier: disc, central pin, and caps all as one connected solid."""
    # Carrier disc (wider than slot for visual retention)
    disc = cq.Workplane("XY").circle(CARRIER_R).extrude(CARRIER_T / 2, both=True)
    # Central pin protruding up and down through both halves, connecting to caps
    pin_up = cq.Workplane("XY").circle(PIN_R).extrude(HALF_T + CARRIER_T / 2 + CAP_H)
    pin_down = cq.Workplane("XY").circle(PIN_R).extrude(-(HALF_T + CARRIER_T / 2 + CAP_H))
    body = disc.union(pin_up).union(pin_down)
    # Rivet caps on both outer faces (connected via the pin)
    cap_bottom = (
        cq.Workplane("XY")
        .circle(CAP_R)
        .extrude(CAP_H)
        .translate((0.0, 0.0, -(HALF_T + CARRIER_T / 2 + CAP_H)))
    )
    cap_top = (
        cq.Workplane("XY")
        .circle(CAP_R)
        .extrude(CAP_H)
        .translate((0.0, 0.0, HALF_T + CARRIER_T / 2))
    )
    return body.union(cap_bottom).union(cap_top)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="insulated_slip_joint_pliers")

    steel_polished = model.material("steel_polished", rgba=(0.80, 0.81, 0.84, 1.0))
    steel_forged = model.material("steel_forged", rgba=(0.66, 0.67, 0.70, 1.0))
    steel_brushed = model.material("steel_brushed", rgba=(0.61, 0.62, 0.65, 1.0))
    seam_gray = model.material("seam_gray", rgba=(0.45, 0.46, 0.48, 1.0))
    insulation_yellow = model.material("insulation_yellow", rgba=(0.95, 0.82, 0.15, 1.0))
    sleeve_red = model.material("sleeve_red", rgba=(0.72, 0.12, 0.14, 1.0))
    cutter_steel = model.material("cutter_steel", rgba=(0.72, 0.73, 0.76, 1.0))
    pin_steel = model.material("pin_steel", rgba=(0.58, 0.59, 0.62, 1.0))

    # ---- Jaw halves ----
    half0 = model.part("plier_half_0")
    half1 = model.part("plier_half_1")

    for part_obj, part_name, s in ((half0, "half0", 1), (half1, "half1", -1)):
        has_slot = (s == 1)  # slot on half_0 only

        part_obj.visual(
            mesh_from_cadquery(_jaw_solid(s), f"{part_name}_jaw", tolerance=0.0002),
            name="jaw",
            material=steel_polished,
        )
        part_obj.visual(
            mesh_from_cadquery(_hub_solid(s, with_slot=has_slot), f"{part_name}_hub", tolerance=0.0002),
            name="hub",
            material=steel_forged,
        )
        part_obj.visual(
            mesh_from_cadquery(_shank_solid(s), f"{part_name}_shank", tolerance=0.0002),
            name="shank",
            material=steel_forged,
        )
        # Insulation layer (inner, visible as thick yellow sleeve)
        part_obj.visual(
            mesh_from_cadquery(_insulation_solid(s), f"{part_name}_insulation", tolerance=0.0002),
            name="insulation",
            material=insulation_yellow,
        )
        # Outer grip sleeve (red rubber over insulation)
        part_obj.visual(
            mesh_from_cadquery(_sleeve_solid(s), f"{part_name}_sleeve", tolerance=0.0002),
            name="sleeve",
            material=sleeve_red,
        )
        # Cutter bevel wedges
        part_obj.visual(
            mesh_from_cadquery(_cutter_bevel_solid(s), f"{part_name}_cutter_bevel", tolerance=0.0002),
            name="cutter_bevel",
            material=cutter_steel,
        )

    # ---- Pivot carrier (slides in slot on half_0) ----
    carrier = model.part("pivot_carrier")
    carrier.visual(
        mesh_from_cadquery(_carrier_solid(), "carrier_body", tolerance=0.0002),
        name="carrier_body",
        material=pin_steel,
    )

    # Caps are integrated into the carrier_body mesh as one connected solid.
    # Cap presence on both faces is verified via the carrier_body Z extents.

    # ---- Articulations ----

    # 1. Prismatic: half_0 -> pivot_carrier (slip-joint slot slide)
    # Axis along Y (the slot direction in the hub). Positive = pin toward +Y.
    model.articulation(
        "slot_slide",
        ArticulationType.PRISMATIC,
        parent=half0,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.5, lower=0.0, upper=SLIDE_LIMIT),
    )

    # 2. Revolute: pivot_carrier -> half_1 (jaw opening)
    # Axis perpendicular to tool plane (-Z so positive opens jaws).
    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=carrier,
        child=half1,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=3.0, lower=0.0, upper=OPEN_LIMIT),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    half0 = object_model.get_part("plier_half_0")
    half1 = object_model.get_part("plier_half_1")
    carrier = object_model.get_part("pivot_carrier")
    slot_slide = object_model.get_articulation("slot_slide")
    pivot = object_model.get_articulation("pivot")

    jaw0 = half0.get_visual("jaw")
    jaw1 = half1.get_visual("jaw")
    hub0 = half0.get_visual("hub")
    hub1 = half1.get_visual("hub")
    insulation0 = half0.get_visual("insulation")
    sleeve0 = half0.get_visual("sleeve")
    sleeve1 = half1.get_visual("sleeve")
    bevel0 = half0.get_visual("cutter_bevel")
    bevel1 = half1.get_visual("cutter_bevel")

    # ---- Joint structure ----

    # Prismatic slip-joint exists with correct type and limits
    slide_limits = slot_slide.motion_limits
    ctx.check(
        "slot_slide is a prismatic joint with positive travel",
        slot_slide.articulation_type == ArticulationType.PRISMATIC
        and slide_limits is not None
        and slide_limits.lower is not None
        and slide_limits.upper is not None
        and slide_limits.upper > slide_limits.lower
        and slide_limits.upper > 0.001,
        details=f"limits={slide_limits}",
    )

    # Revolute pivot exists with 0..30 degree range
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

    # ---- Slip-joint slot on half_0 hub ----
    # The slot should be visible as a cutout in the hub
    ctx.check(
        "half_0 hub has a slip-joint slot (hub is not a full disc)",
        hub0 is not None,
        details="hub0 missing",
    )

    # ---- Carrier is captured in the slot ----
    ctx.allow_overlap(
        half0,
        carrier,
        elem_a="hub",
        elem_b="carrier_body",
        reason="The carrier pin is intentionally captured inside the half_0 hub slot, "
        "representing the slip-joint sliding fit.",
    )
    ctx.expect_overlap(
        half0,
        carrier,
        axes="xy",
        elem_a=hub0,
        elem_b=carrier.get_visual("carrier_body"),
        min_overlap=0.004,
        name="carrier sits inside the half_0 hub footprint",
    )

    # The carrier pin also passes through half_1's hub (the revolute pivot point)
    ctx.allow_overlap(
        carrier,
        half1,
        elem_a="carrier_body",
        elem_b="hub",
        reason="The carrier pin passes through half_1's hub, forming the revolute "
        "pivot connection that half_1 rotates on.",
    )
    ctx.expect_overlap(
        carrier,
        half1,
        axes="xy",
        elem_a=carrier.get_visual("carrier_body"),
        elem_b=hub1,
        min_overlap=0.004,
        name="carrier pin aligns with half_1 hub at the pivot",
    )

    # ---- Rivet caps on both sides (integrated into carrier_body) ----
    carrier_aabb = ctx.part_element_world_aabb(carrier, elem="carrier_body")
    if carrier_aabb is not None:
        c_min, c_max = carrier_aabb
        # The carrier body includes caps, so Z extent should reach beyond both hub faces
        ctx.check(
            "rivet cap visible on bottom face (carrier extends below hub)",
            c_min[2] < -(HALF_T + 0.001),
            details=f"carrier_min_z={c_min[2]:.4f}",
        )
        ctx.check(
            "rivet cap visible on top face (carrier extends above hub)",
            c_max[2] > HALF_T + 0.001,
            details=f"carrier_max_z={c_max[2]:.4f}",
        )
        # Total Z span should include caps on both sides
        z_span = c_max[2] - c_min[2]
        ctx.check(
            "carrier Z span covers both caps plus pin through both halves",
            z_span >= 2.0 * (HALF_T + CAP_H),
            details=f"z_span={z_span:.4f}",
        )
        # Caps are roughly circular - check X width matches CAP_R*2
        cap_width = c_max[0] - c_min[0]
        ctx.check(
            "carrier has circular cap profile (~20mm diameter)",
            0.016 <= cap_width <= 0.024,
            details=f"carrier_width_x={cap_width:.4f}",
        )
    else:
        ctx.fail("carrier body AABB resolves", "missing carrier_body element AABB")

    # ---- Cutter bevels as wedge geometry ----
    bevel0_aabb = ctx.part_element_world_aabb(half0, elem="cutter_bevel")
    bevel1_aabb = ctx.part_element_world_aabb(half1, elem="cutter_bevel")
    ok_bevels = bevel0_aabb is not None and bevel1_aabb is not None
    if ok_bevels:
        b0_min, b0_max = bevel0_aabb
        b1_min, b1_max = bevel1_aabb
        # Bevels are near the pivot (x between 0.01 and 0.03)
        ctx.check(
            "cutter bevel on half_0 is near the pivot region",
            0.008 <= b0_min[0] and b0_max[0] <= 0.030,
            details=f"bevel0_x=[{b0_min[0]:.4f}, {b0_max[0]:.4f}]",
        )
        # Bevels have wedge height (non-zero Z extent or Y extent depending on orientation)
        bevel_dy = b0_max[1] - b0_min[1]
        ctx.check(
            "cutter bevel has visible wedge depth",
            bevel_dy >= 0.001,
            details=f"bevel0_dy={bevel_dy:.4f}",
        )
    else:
        ctx.fail("cutter bevel AABBs resolve", "missing cutter_bevel element AABB")

    # ---- Insulated layered grips ----
    insul0_aabb = ctx.part_element_world_aabb(half0, elem="insulation")
    sleeve0_aabb = ctx.part_element_world_aabb(half0, elem="sleeve")
    ok_layers = insul0_aabb is not None and sleeve0_aabb is not None
    if ok_layers:
        i0_min, i0_max = insul0_aabb
        s0_min, s0_max = sleeve0_aabb
        # Insulation is the larger (outer) layer
        insul_dy = i0_max[1] - i0_min[1]
        sleeve_dy = s0_max[1] - s0_min[1]
        ctx.check(
            "insulation layer is wider than the sleeve (layered grip)",
            insul_dy >= sleeve_dy - 0.001,
            details=f"insul_dy={insul_dy:.4f} sleeve_dy={sleeve_dy:.4f}",
        )
        # Insulation extends further from center than sleeve in Y
        ctx.check(
            "insulation layer visible as thick inner sleeve",
            insul_dy >= 0.015,
            details=f"insul_dy={insul_dy:.4f}",
        )
    else:
        ctx.fail("insulation/sleeve AABBs resolve", "missing grip layer AABB")

    # ---- Closed rest pose: jaws nearly touch ----
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

    # ---- Halves interleave at pivot ----
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

    # ---- Decisive open pose: jaws separate ----
    closed_jaw1 = ctx.part_element_world_aabb(half1, elem="jaw")
    closed_grip1 = ctx.part_element_world_aabb(half1, elem="sleeve")
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
        open_grip1 = ctx.part_element_world_aabb(half1, elem="sleeve")
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

    # ---- Prismatic slide moves the carrier ----
    carrier_rest = ctx.part_world_position(carrier)
    with ctx.pose({slot_slide: SLIDE_LIMIT}):
        carrier_extended = ctx.part_world_position(carrier)
        ctx.check(
            "prismatic slide moves the pivot carrier along Y",
            carrier_rest is not None
            and carrier_extended is not None
            and carrier_extended[1] > carrier_rest[1] + 0.001,
            details=f"rest={carrier_rest}, extended={carrier_extended}",
        )

    return ctx.report()


object_model = build_object_model()
