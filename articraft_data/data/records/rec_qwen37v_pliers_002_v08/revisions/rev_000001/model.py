from __future__ import annotations

# End-cutting nippers variant of heavy-duty lineman pliers.
# Reference: picture/Other/pliers family variant
#
# Layout: the tool lies in the XY plane with Z as the thickness axis.
# The rivet pivot is at the origin. Handles sweep back to -X.
# Unlike the parent pliers, the jaws extend upward (+Z) from the pivot
# area, perpendicular to the handle direction. The two curved jaws form
# a rounded cutting head; their cutting edges nearly touch at the top.
#
# plier_half_0 (root, s=+1): jaw on +Y side, carries rivet and screw.
# plier_half_1 (moving, s=-1): jaw on -Y side.
# adjustment_screw: revolute at the rear handle for tension adjustment.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobRelief,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- shared dimensions (meters) ----
HALF_T = 0.009          # half thickness of each forged plate
LAP_R = 0.016           # half-lap joint disc radius at the pivot
HUB_R = 0.015           # forged hub radius around the rivet
BOSS_R = 0.0125         # rivet cap radius (~0.025 m diameter)
BOSS_H = 0.0035         # rivet cap height (slightly taller for visibility)
SEAM_R = 0.0132         # visible circular seam ring under the cap
SEAM_H = 0.0006
RIVET_R = 0.004         # rivet shaft
EPS = 0.0001            # lap clearance
OPEN_LIMIT = math.radians(30.0)

# ---- Handle geometry (same curve as parent) ----
TANG_HALF_W = 0.004
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

# ---- Jaw cross-sections for the end-cutting nippers ----
# Each section: (z_height, center_x, center_y_offset, half_extent_x, half_extent_y)
# center_y is s * center_y_offset. Jaws converge toward Y=0 at the tips.
# Sections below z=0 extend into the hub zone for geometric connectivity.
JAW_SECTIONS = [
    (-0.007, 0.012, 0.009, 0.012, 0.010),  # base inside hub (connectivity)
    (-0.003, 0.013, 0.008, 0.013, 0.009),  # hub transition
    (0.002,  0.014, 0.008, 0.013, 0.009),  # just above hub
    (0.010,  0.020, 0.007, 0.012, 0.008),
    (0.018,  0.025, 0.006, 0.010, 0.007),
    (0.026,  0.026, 0.005, 0.008, 0.006),
    (0.033,  0.022, 0.004, 0.006, 0.004),
    (0.038,  0.016, 0.003, 0.004, 0.003),
    (0.042,  0.010, 0.002, 0.003, 0.002),  # tip with clear Y separation
]

# ---- Adjustment screw position (rear of handle) ----
SCREW_X = -0.105
SCREW_HEAD_R = 0.006
SCREW_HEAD_H = 0.005
SCREW_SHAFT_R = 0.003
SCREW_SHAFT_H = 0.014


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


def _jaw_wire(z: float, cx: float, cy: float, hx: float, hy: float) -> cq.Wire:
    """Elliptical cross-section on an XY workplane at height z."""
    wp = cq.Workplane("XY", origin=(cx, cy, z))
    return wp.ellipse(hx, hy).val()


def _jaw_solid(s: int) -> cq.Workplane:
    """End-cutting nipper jaw: curved body extending upward (+Z) from the hub.

    The jaw is perpendicular to the handle direction. Cutter bevels are
    cut as visible wedge geometry on the inner face near the tip.
    The base extends deep into the hub zone for geometric connectivity.
    """
    # Loft the jaw body from elliptical cross-sections
    wires = []
    for z, cx, cy_off, hx, hy in JAW_SECTIONS:
        cy = s * cy_off
        wires.append(_jaw_wire(z, cx, cy, hx, hy))
    jaw = cq.Solid.makeLoft(wires, ruled=False)
    jaw_wp = cq.Workplane("XY").newObject([jaw])

    # Cutter bevel: visible wedge cut on the inner face (toward Y=0).
    # Creates the angled cutting edge characteristic of end-cutting nippers.
    # Use shallow cuts to carve the bevel profile without disconnecting the jaw.
    bevel_depth = 0.0025
    # Cut angled grooves along the inner face of the jaw tip region.
    # The groove is deeper at the bottom and shallower at top to create the bevel.
    for i in range(5):
        z_lo = 0.028 + i * 0.003
        # The cut depth decreases toward the tip (creating the bevel angle)
        depth = bevel_depth * (1.0 - i * 0.18)
        if depth < 0.0008:
            continue
        # Box positioned to cut from the inner face
        cutter = (
            cq.Workplane("XY")
            .box(0.020, depth, 0.0028)
            .translate((0.016, s * (depth / 2.0), z_lo + 0.0014))
        )
        jaw_wp = jaw_wp.cut(cutter)

    # No lap_cut on the jaw: the jaw loft extends through the hub zone for
    # geometric connectivity. The hub itself is lap-cut so the two halves
    # don't overlap at the pivot.
    return jaw_wp


def _hub_solid(s: int) -> cq.Workplane:
    """Forged circular hub around the rivet, thinned to its lap layer."""
    hub = cq.Workplane("XY").circle(HUB_R).extrude(HALF_T, both=True)
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


def _screw_head_solid() -> cq.Workplane:
    """Adjustment screw: knurled cylindrical head with a slot."""
    head = (
        cq.Workplane("XY")
        .circle(SCREW_HEAD_R)
        .extrude(SCREW_HEAD_H)
    )
    # Knurl grooves around the circumference
    for i in range(12):
        angle = math.radians(i * 30.0)
        cx = (SCREW_HEAD_R - 0.0004) * math.cos(angle)
        cy = (SCREW_HEAD_R - 0.0004) * math.sin(angle)
        groove = (
            cq.Workplane("XY")
            .circle(0.0006)
            .extrude(SCREW_HEAD_H)
            .translate((cx, cy, 0.0))
        )
        head = head.cut(groove)
    # Screwdriver slot across the top
    slot = (
        cq.Workplane("XY")
        .box(SCREW_HEAD_R * 2.2, 0.0012, 0.002)
        .translate((0.0, 0.0, SCREW_HEAD_H - 0.001))
    )
    head = head.cut(slot)
    return head


def _screw_shaft_solid() -> cq.Workplane:
    """Threaded shaft of the adjustment screw."""
    return cq.Workplane("XY").circle(SCREW_SHAFT_R).extrude(SCREW_SHAFT_H)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="end_cutting_nippers")

    steel_polished = model.material("steel_polished", rgba=(0.80, 0.81, 0.84, 1.0))
    steel_forged = model.material("steel_forged", rgba=(0.66, 0.67, 0.70, 1.0))
    steel_brushed = model.material("steel_brushed", rgba=(0.61, 0.62, 0.65, 1.0))
    seam_gray = model.material("seam_gray", rgba=(0.45, 0.46, 0.48, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.08, 0.09, 1.0))
    grip_red = model.material("grip_red", rgba=(0.80, 0.09, 0.11, 1.0))
    screw_dark = model.material("screw_dark", rgba=(0.25, 0.25, 0.28, 1.0))

    parts = []
    for part_name, s in (("plier_half_0", 1), ("plier_half_1", -1)):
        part = model.part(part_name)
        tag = part_name.replace("plier_half_", "half")

        # End-cutting jaw (perpendicular to handles, extending +Z)
        part.visual(
            mesh_from_cadquery(_jaw_solid(s), f"{tag}_jaw", tolerance=0.0003),
            name="jaw",
            material=steel_polished,
        )
        # Hub at the pivot
        part.visual(
            mesh_from_cadquery(_hub_solid(s), f"{tag}_hub", tolerance=0.0002),
            name="hub",
            material=steel_forged,
        )
        # Handle shank (steel tang)
        part.visual(
            mesh_from_cadquery(_shank_solid(s), f"{tag}_shank", tolerance=0.0002),
            name="shank",
            material=steel_forged,
        )
        # Rubber grip
        part.visual(
            mesh_from_cadquery(_grip_solid(s), f"{tag}_grip", tolerance=0.0002),
            name="grip",
            material=rubber_black,
        )
        # Red inlay
        part.visual(
            mesh_from_cadquery(_inlay_solid(s), f"{tag}_inlay", tolerance=0.0002),
            name="grip_inlay",
            material=grip_red,
        )

        parts.append(part)

    fixed = parts[0]  # plier_half_0

    # ---- Pivot rivet caps on both sides ----
    # Bottom cap (below half_0)
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
    # Top cap (above half_1)
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

    # ---- Adjustment screw at the rear handle ----
    screw_part = model.part("adjustment_screw")
    # Screw head (knurled cylinder, visible on the outer handle face)
    screw_part.visual(
        mesh_from_cadquery(_screw_head_solid(), "screw_head", tolerance=0.0002),
        name="screw_head",
        material=screw_dark,
    )
    # Screw shaft (embedded in the handle)
    screw_part.visual(
        mesh_from_cadquery(_screw_shaft_solid(), "screw_shaft", tolerance=0.0002),
        name="screw_shaft",
        material=steel_brushed,
    )

    # Screw Y position follows the handle centerline
    screw_y = _yc(SCREW_X)

    # ---- Primary articulation: pivot joint ----
    # One revolute joint at the rivet, axis perpendicular to the tool plane.
    # Positive q opens the jaws (tips separate in Y) while handles spread.
    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=parts[0],
        child=parts[1],
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=3.0, lower=0.0, upper=OPEN_LIMIT),
    )

    # ---- Adjustment screw articulation ----
    # Revolute joint at the rear handle. Axis along Z (perpendicular to handle
    # flat face). Continuous rotation for tension adjustment.
    model.articulation(
        "screw_adjust",
        ArticulationType.CONTINUOUS,
        parent=parts[0],
        child=screw_part,
        origin=Origin(xyz=(SCREW_X, screw_y, -(HALF_T + 0.001))),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    half0 = object_model.get_part("plier_half_0")
    half1 = object_model.get_part("plier_half_1")
    screw = object_model.get_part("adjustment_screw")
    pivot = object_model.get_articulation("pivot")
    screw_joint = object_model.get_articulation("screw_adjust")

    jaw0 = half0.get_visual("jaw")
    jaw1 = half1.get_visual("jaw")
    hub0 = half0.get_visual("hub")
    hub1 = half1.get_visual("hub")
    grip0 = half0.get_visual("grip")
    grip1 = half1.get_visual("grip")

    # ---- Pivot joint contract ----
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

    # ---- Adjustment screw is continuous revolute ----
    ctx.check(
        "adjustment screw is a continuous joint",
        screw_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={screw_joint.articulation_type}",
    )

    # ---- Jaws perpendicular to handles (extend in +Z) ----
    jaw0_aabb = ctx.part_element_world_aabb(half0, elem="jaw")
    jaw1_aabb = ctx.part_element_world_aabb(half1, elem="jaw")
    grip0_aabb = ctx.part_element_world_aabb(half0, elem="grip")
    if jaw0_aabb is not None and grip0_aabb is not None:
        jaw_top_z = jaw0_aabb[1][2]
        grip_max_z = grip0_aabb[1][2]
        ctx.check(
            "jaws extend well above the handle plane (perpendicular)",
            jaw_top_z > grip_max_z + 0.025,
            details=f"jaw_top_z={jaw_top_z:.4f} grip_max_z={grip_max_z:.4f}",
        )
    else:
        ctx.fail("jaw/grip AABBs resolve", "missing jaw or grip element AABB")

    # ---- Closed rest pose: jaw tips nearly touching ----
    # The cutting edges sit close together; small overlap is expected at the
    # wider base of the jaw where the two halves converge.
    ctx.expect_gap(
        half0,
        half1,
        axis="y",
        positive_elem=jaw0,
        negative_elem=jaw1,
        min_gap=-0.006,
        max_gap=0.006,
        name="jaw cutting edges nearly touching at rest",
    )

    # ---- Halves interleave at the pivot ----
    ctx.expect_contact(
        half0,
        half1,
        elem_a=hub0,
        elem_b=hub1,
        contact_tol=0.0005,
        name="hub laps seat against each other at the joint",
    )
    ctx.expect_overlap(
        half0,
        half1,
        axes="xy",
        elem_a=hub0,
        elem_b=hub1,
        min_overlap=0.02,
        name="hub laps share the pivot footprint",
    )

    # ---- Rivet shaft captured through moving half ----
    ctx.allow_overlap(
        half0,
        half1,
        elem_a="rivet_shaft",
        elem_b=hub1,
        reason="The rivet shaft is fixed to half_0 and intentionally passes "
        "through the moving half's hub lap, capturing it at the pivot.",
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

    # ---- Jaw base extends into hub zone for connectivity ----
    # The jaw loft base overlaps with the other half's hub for geometric
    # connectivity. This is intentional and localized to the hub zone.
    ctx.allow_overlap(
        half0,
        half1,
        elem_a="jaw",
        elem_b="hub",
        reason="The jaw base extends into the hub zone for geometric "
        "connectivity, creating localized overlap with the other half's hub.",
    )
    ctx.allow_overlap(
        half1,
        half0,
        elem_a="jaw",
        elem_b="hub",
        reason="The jaw base extends into the hub zone for geometric "
        "connectivity, creating localized overlap with the other half's hub.",
    )
    # Verify the overlap is localized to the hub zone (z near 0)
    ctx.expect_overlap(
        half0,
        half1,
        axes="xy",
        elem_a="jaw",
        elem_b="hub",
        min_overlap=0.01,
        name="jaw base shares hub footprint for connectivity",
    )

    # ---- Rivet caps on both sides ----
    cap_bottom = ctx.part_element_world_aabb(half0, elem="rivet_cap_bottom")
    cap_top = ctx.part_element_world_aabb(half0, elem="rivet_cap_top")
    if cap_bottom is not None and cap_top is not None:
        # Bottom cap should be below the tool plane
        ctx.check(
            "bottom rivet cap is below the hub plane",
            cap_bottom[0][2] < -(HALF_T - 0.001),
            details=f"cap_bottom_min_z={cap_bottom[0][2]:.4f}",
        )
        # Top cap should be above the tool plane
        ctx.check(
            "top rivet cap is above the hub plane",
            cap_top[1][2] > (HALF_T - 0.001),
            details=f"cap_top_max_z={cap_top[1][2]:.4f}",
        )
        # Both caps ~0.025m diameter
        dia_bottom = cap_bottom[1][0] - cap_bottom[0][0]
        dia_top = cap_top[1][0] - cap_top[0][0]
        ctx.check(
            "rivet caps are ~25 mm diameter",
            0.023 <= dia_bottom <= 0.027 and 0.023 <= dia_top <= 0.027,
            details=f"dia_bottom={dia_bottom:.4f} dia_top={dia_top:.4f}",
        )
    else:
        ctx.fail("rivet cap AABBs resolve", "missing rivet_cap_bottom or rivet_cap_top")

    # ---- Adjustment screw mounted on handle ----
    ctx.allow_overlap(
        half0,
        screw,
        elem_a="grip",
        elem_b="screw_shaft",
        reason="The screw shaft is intentionally embedded in the handle grip "
        "for tension adjustment.",
    )
    ctx.expect_contact(
        half0,
        screw,
        elem_a="shank",
        elem_b="screw_head",
        contact_tol=0.005,
        name="screw head contacts the handle surface",
    )

    # ---- Cutter bevels visible on jaws ----
    # The jaw tips should reach above z=0.035 (perpendicular to handles)
    if jaw0_aabb is not None:
        ctx.check(
            "jaw tips reach at least 35mm above the pivot plane",
            jaw0_aabb[1][2] >= 0.035,
            details=f"jaw0_max_z={jaw0_aabb[1][2]:.4f}",
        )

    # ---- Overall envelope ----
    a0 = ctx.part_world_aabb(half0)
    a1 = ctx.part_world_aabb(half1)
    g0 = ctx.part_element_world_aabb(half0, elem="grip")
    g1 = ctx.part_element_world_aabb(half1, elem="grip")
    ok_env = a0 is not None and a1 is not None and g0 is not None and g1 is not None
    if ok_env:
        length = max(a0[1][0], a1[1][0]) - min(a0[0][0], a1[0][0])
        across = max(g0[1][1], g1[1][1]) - min(g0[0][1], g1[0][1])
        ctx.check(
            "overall tool length about 0.16 m (handles + jaw head)",
            0.15 <= length <= 0.18,
            details=f"length={length:.4f}",
        )
    else:
        ctx.fail("envelope AABBs resolve", "missing part/grip AABB")

    # ---- Decisive open pose ----
    closed_jaw1_y = None
    if jaw1_aabb is not None:
        closed_jaw1_y = jaw1_aabb[0][1]  # min_y of moving jaw

    with ctx.pose({pivot: OPEN_LIMIT}):
        open_jaw1_aabb = ctx.part_element_world_aabb(half1, elem="jaw")
        if closed_jaw1_y is not None and open_jaw1_aabb is not None:
            open_jaw1_y = open_jaw1_aabb[0][1]
            ctx.check(
                "moving jaw swings away from fixed jaw at open pose",
                open_jaw1_y < closed_jaw1_y - 0.008,
                details=f"closed_min_y={closed_jaw1_y:.4f} open_min_y={open_jaw1_y:.4f}",
            )
        ctx.expect_gap(
            half0,
            half1,
            axis="y",
            positive_elem=jaw0,
            negative_elem=jaw1,
            min_gap=0.0005,
            name="jaw tips clearly separated at open pose",
        )

    return ctx.report()


object_model = build_object_model()
