from __future__ import annotations

# Slip-joint pliers variant.
# Reference image: picture/Other/pliers/002.png
#
# Layout: the tool lies in the XY plane with Z as the thickness axis.
# The rivet pivot region is at the origin. The blunt squared nose points +X,
# the handles sweep back to -X and spread in +/-Y.
#
# Functional layers:
#   Layer 1 — Forged steel halves (jaw, hub, shank)
#   Layer 2 — Slip-joint pivot mechanism (slot, carriage, grooves)
#   Layer 3 — Over-molded grips with inlays
#
# plier_half_0 (root) carries the elongated slot in its hub.
# pivot_carriage slides in the slot (PRISMATIC slot_slide).
# plier_half_1 rotates about the carriage rivet (REVOLUTE pivot).

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

# ==================================================================
# Shared dimensions (meters)
# ==================================================================

HALF_T = 0.009          # half thickness of each forged plate (full plate 0.018)
LAP_R = 0.016           # half-lap joint disc radius at the pivot
HUB_R = 0.015           # forged hub radius around the rivet
BOSS_R = 0.0125         # rivet boss radius (~0.025 m diameter)
BOSS_H = 0.0030
SEAM_R = 0.0132         # visible circular seam ring under the boss cap
SEAM_H = 0.0006
RIVET_R = 0.004         # rivet shaft captured through the moving half's lap
EPS = 0.0001            # lap clearance so the stacked halves do not penetrate
JAW_FACE = 0.0003       # closed jaw inner faces sit at y = +/-JAW_FACE
NOSE_X = 0.075          # blunt squared nose tip
OPEN_LIMIT = math.radians(30.0)

# ==================================================================
# Slip-joint mechanism dimensions
# ==================================================================

SLOT_LENGTH = 0.020     # total capsule length of the elongated slot
SLOT_WIDTH = 0.009      # slot width (capsule diameter)
SLOT_TRAVEL = 0.010     # prismatic joint travel
GROOVE_COUNT = 2        # number of adjustment grooves on the shank

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


# ==================================================================
# Interpolation helpers
# ==================================================================

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


# ==================================================================
# Layer 1: Forged steel halves
# ==================================================================

def _lap_cut(s: int) -> cq.Workplane:
    """Material removed at the pivot so this half keeps only its lap layer.

    s=+1 keeps the lower lap (z <= -EPS); s=-1 keeps the upper lap (z >= +EPS).
    """
    cut = cq.Workplane("XY").circle(LAP_R).extrude(0.012)
    if s > 0:
        return cut.translate((0.0, 0.0, -EPS))
    return cut.translate((0.0, 0.0, -0.012 + EPS))


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


def _hub_solid(s: int) -> cq.Workplane:
    """Plain forged circular hub (no slot) for the non-slotted half."""
    hub = cq.Workplane("XY").circle(HUB_R).extrude(HALF_T, both=True)
    return hub.cut(_lap_cut(s))


def _slot_cut_solid() -> cq.Workplane:
    """Elongated capsule-shaped cutting tool for the slip-joint slot."""
    half_rect = (SLOT_LENGTH - SLOT_WIDTH) / 2.0
    r = SLOT_WIDTH / 2.0
    h = HALF_T * 2 + 0.004
    body = cq.Workplane("XY").box(SLOT_LENGTH - SLOT_WIDTH, SLOT_WIDTH, h)
    end_l = (
        cq.Workplane("XY")
        .circle(r)
        .extrude(h / 2.0, both=True)
        .translate((-half_rect, 0.0, 0.0))
    )
    end_r = (
        cq.Workplane("XY")
        .circle(r)
        .extrude(h / 2.0, both=True)
        .translate((half_rect, 0.0, 0.0))
    )
    return body.union(end_l).union(end_r)


def _hub_slotted_solid(s: int) -> cq.Workplane:
    """Hub with elongated slot for the slip-joint pivot."""
    hub = cq.Workplane("XY").circle(HUB_R).extrude(HALF_T, both=True)
    hub = hub.cut(_lap_cut(s))
    hub = hub.cut(_slot_cut_solid())
    return hub


def _shank_solid(s: int) -> cq.Workplane:
    """Steel handle tang sweeping back from the hub into the grip."""
    pts = [(x, s * y) for x, y in TANG_PTS]
    loop = _strip_loop(pts, TANG_HALF_W)
    shank = cq.Workplane("XY").polyline(loop).close().extrude(HALF_T, both=True)
    return shank.cut(_lap_cut(s))


# ==================================================================
# Layer 2: Slip-joint mechanism — grooves
# ==================================================================

def _groove_x(i: int) -> float:
    """X position of adjustment groove i, evenly spaced along the slot travel."""
    if GROOVE_COUNT <= 1:
        return 0.0
    return -SLOT_TRAVEL / 2.0 + (i / (GROOVE_COUNT - 1)) * SLOT_TRAVEL


def _groove_solid(i: int) -> cq.Workplane:
    """One adjustment detent groove mark on the slotted half's hub/shank.

    Placed on either side of the slot (outside the slot cutout) on the
    bottom face of the hub, where hub material remains after the slot cut.
    """
    x = _groove_x(i)
    # Two small notch marks on either side of the slot, on the hub bottom face.
    # y = +/-0.007 is outside the slot half-width (0.0045) but inside the hub
    # radius (0.015), so hub material is present for connectivity.
    notch_pos = (
        cq.Workplane("XY")
        .box(0.0014, 0.003, 0.0008)
        .translate((x, 0.007, -(HALF_T + 0.0002)))
    )
    notch_neg = (
        cq.Workplane("XY")
        .box(0.0014, 0.003, 0.0008)
        .translate((x, -0.007, -(HALF_T + 0.0002)))
    )
    return notch_pos.union(notch_neg)


# ==================================================================
# Layer 3: Over-molded grips
# ==================================================================

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


# ==================================================================
# Build
# ==================================================================

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slip_joint_pliers")

    steel_polished = model.material("steel_polished", rgba=(0.80, 0.81, 0.84, 1.0))
    steel_forged = model.material("steel_forged", rgba=(0.66, 0.67, 0.70, 1.0))
    steel_brushed = model.material("steel_brushed", rgba=(0.61, 0.62, 0.65, 1.0))
    seam_gray = model.material("seam_gray", rgba=(0.45, 0.46, 0.48, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.08, 0.09, 1.0))
    grip_red = model.material("grip_red", rgba=(0.80, 0.09, 0.11, 1.0))
    groove_dark = model.material("groove_dark", rgba=(0.30, 0.31, 0.33, 1.0))

    # ---- Layer 1: Forged steel halves ----

    # Half 0 (root, fixed): jaw on +Y side, slotted hub for the slip-joint.
    half_0 = model.part("plier_half_0")
    half_0.visual(
        mesh_from_cadquery(_jaw_solid(1), "half0_jaw", tolerance=0.0002),
        name="jaw",
        material=steel_polished,
    )
    half_0.visual(
        mesh_from_cadquery(_hub_slotted_solid(1), "half0_hub_slotted", tolerance=0.0002),
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

    # Half 1 (rotating): jaw on -Y side, plain hub.
    half_1 = model.part("plier_half_1")
    half_1.visual(
        mesh_from_cadquery(_jaw_solid(-1), "half1_jaw", tolerance=0.0002),
        name="jaw",
        material=steel_polished,
    )
    half_1.visual(
        mesh_from_cadquery(_hub_solid(-1), "half1_hub", tolerance=0.0002),
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

    # ---- Layer 2: Slip-joint mechanism ----

    # Adjustment grooves on the slotted half (non-moving decorations inlined
    # as parent visuals, generated via a shared helper + for-loop).
    for i in range(GROOVE_COUNT):
        half_0.visual(
            mesh_from_cadquery(_groove_solid(i), f"groove_step_{i}", tolerance=0.0002),
            name=f"groove_{i}",
            material=groove_dark,
        )

    # Pivot carriage: rivet assembly that slides in the slot.
    # The rivet shaft passes through the slot in half_0 and through half_1's hub.
    carriage = model.part("pivot_carriage")

    # Rivet shaft — spans the full stack through both halves.
    carriage.visual(
        Cylinder(RIVET_R, 2.0 * HALF_T + SEAM_H + 2.0 * EPS + 0.0005),
        origin=Origin(xyz=(0.0, 0.0, 0.00005)),
        name="rivet_shaft",
        material=steel_brushed,
    )

    # Bottom face: seam ring and boss (sit below half_0's hub).
    # Boss overlaps into seam, seam overlaps into shaft for connectivity.
    carriage.visual(
        Cylinder(SEAM_R, SEAM_H),
        origin=Origin(xyz=(0.0, 0.0, -(HALF_T + SEAM_H / 2.0 + EPS))),
        name="boss_seam",
        material=seam_gray,
    )
    carriage.visual(
        Cylinder(BOSS_R, BOSS_H),
        origin=Origin(xyz=(0.0, 0.0, -(HALF_T + SEAM_H + BOSS_H / 2.0))),
        name="rivet_boss",
        material=steel_brushed,
    )

    # Top face: seam ring and head (sit above half_1's hub).
    carriage.visual(
        Cylinder(SEAM_R, SEAM_H),
        origin=Origin(xyz=(0.0, 0.0, HALF_T + SEAM_H / 2.0 + EPS)),
        name="head_seam",
        material=seam_gray,
    )
    carriage.visual(
        Cylinder(BOSS_R, BOSS_H),
        origin=Origin(xyz=(0.0, 0.0, HALF_T + SEAM_H + BOSS_H / 2.0)),
        name="rivet_head",
        material=steel_brushed,
    )

    # ---- Articulations ----

    # PRISMATIC: slot slide (half_0 → carriage).
    # Axis along +X: positive q slides the carriage toward the jaws,
    # widening the jaw capacity (the slip-joint's purpose).
    model.articulation(
        "slot_slide",
        ArticulationType.PRISMATIC,
        parent=half_0,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=50.0,
            velocity=0.5,
            lower=-SLOT_TRAVEL / 2.0,
            upper=SLOT_TRAVEL / 2.0,
        ),
    )

    # REVOLUTE: pivot (carriage → half_1).
    # Axis perpendicular to the tool plane. Positive q (about -Z) swings
    # half_1's jaw toward -Y, opening the jaws while the handles spread.
    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=carriage,
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


# ==================================================================
# Tests
# ==================================================================

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    half_0 = object_model.get_part("plier_half_0")
    half_1 = object_model.get_part("plier_half_1")
    carriage = object_model.get_part("pivot_carriage")
    slide = object_model.get_articulation("slot_slide")
    pivot = object_model.get_articulation("pivot")

    jaw0 = half_0.get_visual("jaw")
    jaw1 = half_1.get_visual("jaw")
    hub0 = half_0.get_visual("hub")
    hub1 = half_1.get_visual("hub")
    grip0 = half_0.get_visual("grip")
    grip1 = half_1.get_visual("grip")

    # ---- Joint structure: PRISMATIC + REVOLUTE ----

    slide_limits = slide.motion_limits
    ctx.check(
        "slot_slide is a prismatic joint with correct travel",
        slide.articulation_type == ArticulationType.PRISMATIC
        and slide_limits is not None
        and slide_limits.lower is not None
        and slide_limits.upper is not None
        and abs(slide_limits.upper - slide_limits.lower - SLOT_TRAVEL) < 1e-6,
        details=f"limits={slide_limits}",
    )

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

    # ---- Adjustment grooves: count and spacing ----

    groove_names = [f"groove_{i}" for i in range(GROOVE_COUNT)]
    for gname in groove_names:
        v = half_0.get_visual(gname)
        ctx.check(
            f"adjustment {gname} exists on slotted half",
            v is not None,
            details=f"missing visual {gname} on plier_half_0",
        )

    groove_aabbs = [
        ctx.part_element_world_aabb(half_0, elem=gn) for gn in groove_names
    ]
    if all(a is not None for a in groove_aabbs) and GROOVE_COUNT >= 2:
        centers_x = [0.5 * (a[0][0] + a[1][0]) for a in groove_aabbs]
        spacing = centers_x[-1] - centers_x[0]
        expected = SLOT_TRAVEL
        ctx.check(
            "grooves evenly spaced along the slot travel",
            abs(spacing - expected) < 0.002,
            details=f"spacing={spacing:.4f} expected={expected:.4f}",
        )
        # Grooves should be on the bottom face of the hub (visible from below).
        for idx, a in enumerate(groove_aabbs):
            ctx.check(
                f"{groove_names[idx]} sits below the hub bottom face",
                a[0][2] < -HALF_T,
                details=f"groove_min_z={a[0][2]:.4f} hub_bottom={-HALF_T:.4f}",
            )

    # ---- Slotted hub: the slot is an internal cutout visible as a gap ----

    hub0_aabb = ctx.part_element_world_aabb(half_0, elem="hub")
    hub1_aabb = ctx.part_element_world_aabb(half_1, elem="hub")
    shaft_aabb = ctx.part_element_world_aabb(carriage, elem="rivet_shaft")
    if hub0_aabb is not None and shaft_aabb is not None:
        # The rivet shaft must sit within the slotted hub's XY footprint,
        # proving the slot opening exists and the shaft passes through it.
        ctx.expect_within(
            carriage,
            half_0,
            axes="xy",
            inner_elem="rivet_shaft",
            outer_elem="hub",
            margin=0.002,
            name="rivet shaft sits within the slotted hub footprint",
        )
    if hub0_aabb is not None and hub1_aabb is not None:
        # Both hubs should have the same outer diameter (the slot is internal).
        hub0_dy = hub0_aabb[1][1] - hub0_aabb[0][1]
        hub1_dy = hub1_aabb[1][1] - hub1_aabb[0][1]
        ctx.check(
            "both hubs have the same outer diameter",
            abs(hub0_dy - hub1_dy) < 0.002,
            details=f"hub0_dy={hub0_dy:.4f} hub1_dy={hub1_dy:.4f}",
        )

    # ---- Half-lap and hub contact at rest ----

    # Closed rest pose: serrated jaw inner faces nearly touch.
    ctx.expect_gap(
        half_0,
        half_1,
        axis="y",
        positive_elem=jaw0,
        negative_elem=jaw1,
        min_gap=0.0002,
        max_gap=0.0015,
        name="jaws closed and nearly touching at rest",
    )

    # The halves interleave at the pivot: half_1's lap passes over half_0's.
    ctx.expect_gap(
        half_1,
        half_0,
        axis="z",
        positive_elem=hub1,
        negative_elem=hub0,
        min_gap=0.0,
        max_gap=0.001,
        name="moving hub lap stacks above fixed hub lap",
    )
    ctx.expect_contact(
        half_0,
        half_1,
        elem_a=hub0,
        elem_b=hub1,
        contact_tol=0.0005,
        name="hub laps seat against each other at the joint",
    )
    ctx.expect_overlap(
        half_0,
        half_1,
        axes="xy",
        elem_a=hub0,
        elem_b=hub1,
        min_overlap=0.02,
        name="hub laps share the pivot footprint",
    )

    # ---- Rivet shaft captured through half_1's hub ----

    ctx.allow_overlap(
        carriage,
        half_1,
        elem_a="rivet_shaft",
        elem_b="hub",
        reason="The rivet shaft on the carriage intentionally passes through "
        "half_1's hub, capturing it at the pivot.",
    )
    ctx.expect_within(
        carriage,
        half_1,
        axes="xy",
        inner_elem="rivet_shaft",
        outer_elem="hub",
        margin=0.0005,
        name="rivet shaft stays centered inside half_1 hub",
    )
    ctx.expect_overlap(
        carriage,
        half_1,
        axes="z",
        elem_a="rivet_shaft",
        elem_b="hub",
        min_overlap=0.007,
        name="rivet shaft passes through half_1 hub thickness",
    )

    # ---- Rivet shaft slides through half_0's slot ----

    ctx.allow_overlap(
        carriage,
        half_0,
        elem_a="rivet_shaft",
        elem_b="hub",
        reason="The rivet shaft on the carriage slides through the elongated "
        "slot in half_0's hub — the core slip-joint mechanism.",
    )

    # ---- Rivet boss dimensions ----

    boss_aabb = ctx.part_element_world_aabb(carriage, elem="rivet_boss")
    head_aabb = ctx.part_element_world_aabb(carriage, elem="rivet_head")
    if boss_aabb is not None and head_aabb is not None:
        dia = boss_aabb[1][0] - boss_aabb[0][0]
        thick = head_aabb[1][2] - boss_aabb[0][2]
        ctx.check(
            "rivet boss is ~25 mm diameter",
            0.0235 <= dia <= 0.0265,
            details=f"dia={dia:.4f}",
        )
        ctx.check(
            "boss caps proud on both outer faces, ~25 mm total at the boss",
            boss_aabb[0][2] <= -0.0115
            and head_aabb[1][2] >= 0.0115
            and 0.023 <= thick <= 0.030,
            details=f"boss_z={boss_aabb[0][2]:.4f} head_z={head_aabb[1][2]:.4f} thick={thick:.4f}",
        )
    else:
        ctx.fail("rivet boss AABBs resolve", "missing rivet_boss/rivet_head element AABB")

    # ---- Closed-pose envelope ----

    a0 = ctx.part_world_aabb(half_0)
    a1 = ctx.part_world_aabb(half_1)
    g0 = ctx.part_element_world_aabb(half_0, elem="grip")
    g1 = ctx.part_element_world_aabb(half_1, elem="grip")
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

    # ---- Red inlay proud on grip ----

    i0 = ctx.part_element_world_aabb(half_0, elem="grip_inlay")
    if i0 is not None and g0 is not None:
        ctx.check(
            "red inlay proud on the grip top face",
            i0[1][2] >= g0[1][2] - 0.0005,
            details=f"inlay_top={i0[1][2]:.4f} grip_top={g0[1][2]:.4f}",
        )

    # ---- Slide mechanism: carriage shifts along the slot ----

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: SLOT_TRAVEL / 2.0}):
        shifted_pos = ctx.part_world_position(carriage)
        ctx.check(
            "carriage shifts toward +X at positive slide",
            rest_pos is not None
            and shifted_pos is not None
            and shifted_pos[0] > rest_pos[0] + 0.003,
            details=f"rest_x={rest_pos}, shifted_x={shifted_pos}",
        )
    with ctx.pose({slide: -SLOT_TRAVEL / 2.0}):
        neg_pos = ctx.part_world_position(carriage)
        ctx.check(
            "carriage shifts toward -X at negative slide",
            rest_pos is not None
            and neg_pos is not None
            and neg_pos[0] < rest_pos[0] - 0.003,
            details=f"rest_x={rest_pos}, neg_x={neg_pos}",
        )

    # ---- Decisive open pose: jaws separate, handles spread ----

    closed_jaw1 = ctx.part_element_world_aabb(half_1, elem="jaw")
    closed_grip1 = ctx.part_element_world_aabb(half_1, elem="grip")
    with ctx.pose({pivot: OPEN_LIMIT}):
        ctx.expect_gap(
            half_0,
            half_1,
            axis="y",
            positive_elem=jaw0,
            negative_elem=jaw1,
            min_gap=0.004,
            name="jaws open apart at the 30 degree pose",
        )
        open_jaw1 = ctx.part_element_world_aabb(half_1, elem="jaw")
        open_grip1 = ctx.part_element_world_aabb(half_1, elem="grip")
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

    return ctx.report()


object_model = build_object_model()
