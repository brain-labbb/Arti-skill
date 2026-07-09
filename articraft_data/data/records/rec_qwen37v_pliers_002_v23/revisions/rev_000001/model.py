from __future__ import annotations

# Variant 23: Broad lineman pliers with gripping teeth, cutter notch,
# jaw stop boss, and a folding safety latch over the handles.
# Reference image: picture/Other/pliers/002.png
#
# Layout: the tool lies in the XY plane with Z as the thickness axis.
# The rivet pivot is at the origin. The blunt squared nose points +X,
# the handles sweep back to -X and spread in +/-Y.
# plier_half_0 (root) carries its jaw on +Y; plier_half_1 is the mirrored
# moving half. The two forged halves are half-lapped at the pivot.
# A small latch bar is mounted on half_0's handle and folds over both handles.

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
HUB_R = 0.015           # forged hub radius around the rivet
BOSS_R = 0.0125         # rivet boss radius (~0.025 m diameter)
BOSS_H = 0.0030
SEAM_R = 0.0132         # visible circular seam ring under the boss cap
SEAM_H = 0.0006
RIVET_R = 0.004         # rivet shaft captured through the moving half's lap
EPS = 0.0001            # lap clearance
JAW_FACE = 0.0003       # closed jaw inner faces sit at y = +/-JAW_FACE
NOSE_X = 0.075          # blunt squared nose tip
OPEN_LIMIT = math.radians(30.0)

TANG_HALF_W = 0.004     # steel handle tang half width in plan

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

# Jaw stop boss
STOP_X = 0.012          # boss X position (near pivot, behind jaw gripping zone)
STOP_R = 0.002          # boss radius
STOP_H = 0.00025        # boss protrusion from jaw face toward centerline

# Serrated gripping teeth on the inner jaw face
TEETH_START_X = 0.042
TEETH_SPACING = 0.0035
TEETH_COUNT = 7
TEETH_HALF_W = 0.0008   # tooth base half-width
TEETH_PROTRUSION = 0.0003  # tooth tip protrusion from jaw face toward center

# Latch dimensions
LATCH_LEN = 0.052       # latch bar length
LATCH_W = 0.007         # latch bar width
LATCH_T = 0.003         # latch bar thickness
LATCH_PIVOT_X = -0.105  # latch pivot X on half_0's handle
LATCH_PIVOT_Z = 0.015   # latch pivot Z (above tallest grip section)
LATCH_PIN_R = 0.0015    # latch pivot pin radius
LATCH_PIN_H = 0.010     # latch pivot pin length (extends down into grip)
LATCH_UPPER = math.radians(90.0)  # latch opens 90 degrees across handles


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


def _lap_cut(s: int) -> cq.Workplane:
    """Material removed at the pivot so this half keeps only its lap layer."""
    cut = cq.Workplane("XY").circle(LAP_R).extrude(0.012)
    if s > 0:
        return cut.translate((0.0, 0.0, -EPS))
    return cut.translate((0.0, 0.0, -0.012 + EPS))


def _jaw_solid(s: int) -> cq.Workplane:
    """Polished jaw: squared nose, serrations, pipe-grip recess, wire cutter,
    plus raised serrated gripping teeth on the inner face."""
    profile = [
        (0.010, JAW_FACE),
        (NOSE_X, JAW_FACE),
        (NOSE_X, 0.0105),
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

    # Serrated gripping teeth: raised triangular ridges on the inner jaw face.
    # Staggered by half-spacing between halves so teeth interleave when closed.
    x_offset = 0.0 if s > 0 else TEETH_SPACING / 2.0
    for i in range(TEETH_COUNT):
        xi = TEETH_START_X + TEETH_SPACING * i + x_offset
        tooth_pts = [
            (xi - TEETH_HALF_W, s * JAW_FACE),
            (xi + TEETH_HALF_W, s * JAW_FACE),
            (xi, s * (JAW_FACE - TEETH_PROTRUSION)),
        ]
        tooth = (
            cq.Workplane("XY")
            .polyline(tooth_pts)
            .close()
            .extrude(0.007, both=True)
        )
        jaw = jaw.union(tooth)

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


def _jaw_stop_boss_solid(s: int) -> cq.Workplane:
    """Small round boss on the inner jaw face near the pivot, acting as a
    jaw-closure stop. Protrudes from the jaw face toward the centerline."""
    h = STOP_H
    if s > 0:
        # Boss base at y=+JAW_FACE, tip toward -Y (toward center)
        y_start = JAW_FACE - h
    else:
        # Boss base at y=-JAW_FACE, tip toward +Y (toward center)
        y_start = -JAW_FACE
    boss = (
        cq.Workplane("XZ", origin=(STOP_X, y_start, 0.0))
        .circle(STOP_R)
        .extrude(h)
    )
    return boss


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


def _latch_bar_solid() -> cq.Workplane:
    """Flat latch bar that folds over the handles.
    In the latch part frame, the bar extends along +X from the pivot end.
    The pivot end is at the part origin; the catch tab is at the free end."""
    # Main bar body, offset so pivot end is at x=0
    bar = (
        cq.Workplane("XY")
        .box(LATCH_LEN, LATCH_W, LATCH_T)
        .translate((LATCH_LEN / 2.0, 0.0, 0.0))
    )
    # Rounded pivot end (half-disc at x=0)
    pivot_end = (
        cq.Workplane("XY")
        .circle(LATCH_W / 2.0)
        .extrude(LATCH_T)
        .translate((0.0, 0.0, -LATCH_T / 2.0))
    )
    bar = bar.union(pivot_end)
    # Wider catch tab at the free end
    catch = (
        cq.Workplane("XY")
        .box(0.008, LATCH_W + 0.004, LATCH_T)
        .translate((LATCH_LEN - 0.004, 0.0, 0.0))
    )
    bar = bar.union(catch)
    # Pivot hole at the pivot end (clearance for the pin)
    hole = (
        cq.Workplane("XY")
        .circle(LATCH_PIN_R + 0.0003)
        .extrude(LATCH_T + 0.004)
        .translate((0.0, 0.0, -LATCH_T / 2.0 - 0.002))
    )
    bar = bar.cut(hole)
    return bar


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lineman_pliers_latch")

    steel_polished = model.material("steel_polished", rgba=(0.80, 0.81, 0.84, 1.0))
    steel_forged = model.material("steel_forged", rgba=(0.66, 0.67, 0.70, 1.0))
    steel_brushed = model.material("steel_brushed", rgba=(0.61, 0.62, 0.65, 1.0))
    seam_gray = model.material("seam_gray", rgba=(0.45, 0.46, 0.48, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.08, 0.09, 1.0))
    grip_red = model.material("grip_red", rgba=(0.80, 0.09, 0.11, 1.0))

    parts = []
    for part_name, s in (("plier_half_0", 1), ("plier_half_1", -1)):
        part = model.part(part_name)
        tag = part_name.replace("plier_half_", "half")

        part.visual(
            mesh_from_cadquery(_jaw_solid(s), f"{tag}_jaw", tolerance=0.0002),
            name="jaw",
            material=steel_polished,
        )
        part.visual(
            mesh_from_cadquery(_hub_solid(s), f"{tag}_hub", tolerance=0.0002),
            name="hub",
            material=steel_forged,
        )
        part.visual(
            mesh_from_cadquery(_shank_solid(s), f"{tag}_shank", tolerance=0.0002),
            name="shank",
            material=steel_forged,
        )
        part.visual(
            mesh_from_cadquery(_grip_solid(s), f"{tag}_grip", tolerance=0.0002),
            name="grip",
            material=rubber_black,
        )
        part.visual(
            mesh_from_cadquery(_inlay_solid(s), f"{tag}_inlay", tolerance=0.0002),
            name="grip_inlay",
            material=grip_red,
        )
        # Jaw stop boss on inner face near pivot
        part.visual(
            mesh_from_cadquery(_jaw_stop_boss_solid(s), f"{tag}_stop_boss", tolerance=0.0002),
            name="jaw_stop_boss",
            material=steel_forged,
        )

        parts.append(part)

    # ---- Rivet assembly (fixed to half_0) ----
    fixed = parts[0]
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

    # ---- Latch pivot pin (mounted on half_0's handle top) ----
    latch_pivot_y = _yc(LATCH_PIVOT_X)
    latch_pivot_z = LATCH_PIVOT_Z
    fixed.visual(
        Cylinder(LATCH_PIN_R, LATCH_PIN_H),
        origin=Origin(xyz=(LATCH_PIVOT_X, latch_pivot_y, latch_pivot_z)),
        name="latch_pin",
        material=steel_brushed,
    )

    # ---- Latch part: small folding bar that locks over both handles ----
    latch = model.part("latch")
    latch.visual(
        mesh_from_cadquery(_latch_bar_solid(), "latch_bar", tolerance=0.0002),
        name="bar",
        material=steel_brushed,
    )

    # ---- Primary articulation: pivot joint ----
    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=parts[0],
        child=parts[1],
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=3.0, lower=0.0, upper=OPEN_LIMIT),
    )

    # ---- Latch articulation: folds over the handles ----
    # At q=0 latch lies along half_0's handle (stowed).
    # At q=pi/2 latch swings across to the other handle (locked).
    model.articulation(
        "latch_pivot",
        ArticulationType.REVOLUTE,
        parent=parts[0],
        child=latch,
        origin=Origin(xyz=(LATCH_PIVOT_X, latch_pivot_y, latch_pivot_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=0.0, upper=LATCH_UPPER),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    half0 = object_model.get_part("plier_half_0")
    half1 = object_model.get_part("plier_half_1")
    latch = object_model.get_part("latch")
    pivot = object_model.get_articulation("pivot")
    latch_joint = object_model.get_articulation("latch_pivot")

    jaw0 = half0.get_visual("jaw")
    jaw1 = half1.get_visual("jaw")
    hub0 = half0.get_visual("hub")
    hub1 = half1.get_visual("hub")
    grip0 = half0.get_visual("grip")
    grip1 = half1.get_visual("grip")
    inlay0 = half0.get_visual("grip_inlay")
    stop0 = half0.get_visual("jaw_stop_boss")
    stop1 = half1.get_visual("jaw_stop_boss")

    # ---- Joint contract: pivot 0..30 degrees ----
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

    # ---- Latch joint contract: revolute, 0..90 degrees ----
    latch_limits = latch_joint.motion_limits
    ctx.check(
        "latch_pivot is a 0..90 degree revolute joint",
        latch_joint.articulation_type == ArticulationType.REVOLUTE
        and latch_limits is not None
        and latch_limits.lower is not None
        and latch_limits.upper is not None
        and abs(latch_limits.lower) < 1e-9
        and abs(latch_limits.upper - LATCH_UPPER) < 1e-6,
        details=f"limits={latch_limits}",
    )

    # ---- Closed rest pose: serrated jaw inner faces nearly touch ----
    # Teeth protrude from each jaw face toward the centerline; opposing teeth
    # are staggered in X so they interleave without real 3D collision, but the
    # projected Y gap of the full jaw visuals can reach zero or slightly
    # negative when the tooth tips cross the centerline.
    ctx.expect_gap(
        half0, half1,
        axis="y",
        positive_elem=jaw0,
        negative_elem=jaw1,
        max_penetration=0.0005,
        max_gap=0.002,
        name="jaws closed and nearly touching at rest",
    )

    # ---- Hub lap stacking ----
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
        elem_a=hub0,
        elem_b=hub1,
        contact_tol=0.0005,
        name="hub laps seat against each other at the joint",
    )
    ctx.expect_overlap(
        half0, half1,
        axes="xy",
        elem_a=hub0,
        elem_b=hub1,
        min_overlap=0.02,
        name="hub laps share the pivot footprint",
    )

    # ---- Rivet shaft through moving hub ----
    ctx.allow_overlap(
        half0, half1,
        elem_a="rivet_shaft",
        elem_b=hub1,
        reason="The rivet shaft is fixed to half_0 and intentionally passes "
        "through the moving half's hub lap, capturing it at the pivot.",
    )
    ctx.expect_within(
        half0, half1,
        axes="xy",
        inner_elem="rivet_shaft",
        outer_elem=hub1,
        margin=0.0005,
        name="rivet shaft stays centered inside the moving hub lap",
    )
    ctx.expect_overlap(
        half0, half1,
        axes="z",
        elem_a="rivet_shaft",
        elem_b=hub1,
        min_overlap=0.007,
        name="rivet shaft passes through the full moving lap thickness",
    )

    # ---- Rivet boss: ~0.025 m diameter, centered on the pivot ----
    boss0_aabb = ctx.part_element_world_aabb(half0, elem="rivet_boss")
    boss1_aabb = ctx.part_element_world_aabb(half0, elem="rivet_head")
    ok_boss = boss0_aabb is not None and boss1_aabb is not None
    if ok_boss:
        b0_min, b0_max = boss0_aabb
        b1_min, b1_max = boss1_aabb
        dia = b0_max[0] - b0_min[0]
        cx = 0.5 * (b0_min[0] + b0_max[0])
        cy = 0.5 * (b0_min[1] + b0_max[1])
        thick = b1_max[2] - b0_min[2]
        ctx.check(
            "rivet boss is ~25 mm diameter and centered on the pivot",
            0.0235 <= dia <= 0.0265 and abs(cx) <= 0.002 and abs(cy) <= 0.002,
            details=f"dia={dia:.4f} center=({cx:.4f},{cy:.4f})",
        )
        ctx.check(
            "boss caps proud on both outer faces, ~25 mm total at the boss",
            b0_min[2] <= -0.0115 and b1_max[2] >= 0.0115 and 0.023 <= thick <= 0.027,
            details=f"b0_min_z={b0_min[2]:.4f} b1_max_z={b1_max[2]:.4f} thick={thick:.4f}",
        )
    else:
        ctx.fail("rivet boss AABBs resolve", "missing rivet_boss element AABB")

    # ---- Envelope: ~0.20 m long, ~0.07 m across handles ----
    a0 = ctx.part_world_aabb(half0)
    a1 = ctx.part_world_aabb(half1)
    g0 = ctx.part_element_world_aabb(half0, elem="grip")
    g1 = ctx.part_element_world_aabb(half1, elem="grip")
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

    # ---- Red inlay proud on grip top ----
    i0 = ctx.part_element_world_aabb(half0, elem="grip_inlay")
    if i0 is not None and g0 is not None:
        ctx.check(
            "red inlay proud on the grip top face",
            i0[1][2] >= g0[1][2] - 0.0005,
            details=f"inlay_top={i0[1][2]:.4f} grip_top={g0[1][2]:.4f}",
        )
    else:
        ctx.fail("inlay AABB resolves", "missing grip_inlay element AABB")

    # ---- Thumb guard close behind pivot ----
    if g0 is not None:
        ctx.check(
            "thumb guard front sits close behind the pivot",
            -0.036 <= g0[1][0] <= -0.026,
            details=f"grip_front_x={g0[1][0]:.4f}",
        )

    # ---- Jaw stop boss: present near pivot on both halves ----
    s0_aabb = ctx.part_element_world_aabb(half0, elem="jaw_stop_boss")
    s1_aabb = ctx.part_element_world_aabb(half1, elem="jaw_stop_boss")
    ok_stop = s0_aabb is not None and s1_aabb is not None
    if ok_stop:
        s0_min, s0_max = s0_aabb
        s1_min, s1_max = s1_aabb
        ctx.check(
            "jaw stop boss on half_0 is near the pivot",
            0.008 <= s0_max[0] <= 0.018 and abs(s0_max[0] + s0_min[0]) / 2.0 < 0.020,
            details=f"stop0_x=({s0_min[0]:.4f},{s0_max[0]:.4f})",
        )
        ctx.check(
            "jaw stop boss on half_1 is near the pivot",
            0.008 <= s1_max[0] <= 0.018 and abs(s1_max[0] + s1_min[0]) / 2.0 < 0.020,
            details=f"stop1_x=({s1_min[0]:.4f},{s1_max[0]:.4f})",
        )
        # Bosses protrude toward centerline from opposite sides
        ctx.check(
            "jaw stop bosses face each other across the jaw gap",
            s0_min[1] > s1_max[1] - 0.002,
            details=f"stop0_min_y={s0_min[1]:.4f} stop1_max_y={s1_max[1]:.4f}",
        )
    else:
        ctx.fail("jaw stop boss AABBs resolve", "missing jaw_stop_boss element AABB")

    # ---- Serrated teeth: jaw extends slightly past the flat face inward ----
    # The teeth protrude from the jaw inner face, so the jaw visual should
    # reach closer to the centerline than the flat JAW_FACE alone.
    j0_aabb = ctx.part_element_world_aabb(half0, elem="jaw")
    j1_aabb = ctx.part_element_world_aabb(half1, elem="jaw")
    if j0_aabb is not None and j1_aabb is not None:
        # Half_0 jaw (on +Y side) should have its inner face reaching toward y=0
        # (teeth protrude from JAW_FACE toward centerline by TEETH_PROTRUSION)
        ctx.check(
            "half_0 jaw has serrated teeth protruding inward",
            j0_aabb[0][1] <= JAW_FACE - TEETH_PROTRUSION + 0.0002,
            details=f"jaw0_min_y={j0_aabb[0][1]:.5f} expected<={JAW_FACE - TEETH_PROTRUSION + 0.0002:.5f}",
        )
        ctx.check(
            "half_1 jaw has serrated teeth protruding inward",
            j1_aabb[1][1] >= -(JAW_FACE - TEETH_PROTRUSION) - 0.0002,
            details=f"jaw1_max_y={j1_aabb[1][1]:.5f} expected>={-(JAW_FACE - TEETH_PROTRUSION) - 0.0002:.5f}",
        )
    else:
        ctx.fail("jaw AABBs resolve for teeth check", "missing jaw element AABB")

    # ---- Latch: bar exists and stowed along half_0 handle at q=0 ----
    latch_bar_aabb = ctx.part_element_world_aabb(latch, elem="bar")
    if latch_bar_aabb is not None and g0 is not None:
        l_min, l_max = latch_bar_aabb
        # At rest, latch lies along half_0's handle (negative X, negative Y side)
        ctx.check(
            "latch bar stowed along the handle at rest",
            l_min[0] < -0.08 and l_max[0] > -0.12,
            details=f"latch_x=({l_min[0]:.4f},{l_max[0]:.4f})",
        )
        ctx.check(
            "latch bar is on the half_0 handle side (negative Y)",
            l_max[1] < 0.0,
            details=f"latch_max_y={l_max[1]:.4f}",
        )
    else:
        ctx.fail("latch bar AABB resolves", "missing bar element AABB")

    # ---- Latch sits near the handle surface at rest ----
    ctx.expect_gap(
        latch, half0,
        axis="z",
        positive_elem="bar",
        negative_elem="grip",
        min_gap=-0.001,
        max_gap=0.008,
        name="latch bar is close above the handle at rest",
    )

    # ---- Latch pin: mounted on half_0 near the latch pivot ----
    pin_aabb = ctx.part_element_world_aabb(half0, elem="latch_pin")
    if pin_aabb is not None:
        p_min, p_max = pin_aabb
        pin_cx = 0.5 * (p_min[0] + p_max[0])
        pin_cy = 0.5 * (p_min[1] + p_max[1])
        ctx.check(
            "latch pin is on half_0's handle near x=-0.105",
            -0.115 <= pin_cx <= -0.095,
            details=f"pin_cx={pin_cx:.4f}",
        )
        ctx.check(
            "latch pin is on the half_0 handle side",
            pin_cy < 0.0,
            details=f"pin_cy={pin_cy:.4f}",
        )
    else:
        ctx.fail("latch pin AABB resolves", "missing latch_pin element AABB")

    # ---- Open pose: jaws separate, handles spread ----
    closed_jaw1 = ctx.part_element_world_aabb(half1, elem="jaw")
    closed_grip1 = ctx.part_element_world_aabb(half1, elem="grip")
    with ctx.pose({pivot: OPEN_LIMIT}):
        ctx.expect_gap(
            half0, half1,
            axis="y",
            positive_elem=jaw0,
            negative_elem=jaw1,
            min_gap=0.004,
            name="jaws open apart at the 30 degree pose",
        )
        open_jaw1 = ctx.part_element_world_aabb(half1, elem="jaw")
        open_grip1 = ctx.part_element_world_aabb(half1, elem="grip")
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

    # ---- Latch open pose: bar swings across to the other handle ----
    closed_latch = ctx.part_element_world_aabb(latch, elem="bar")
    with ctx.pose({latch_joint: LATCH_UPPER}):
        open_latch = ctx.part_element_world_aabb(latch, elem="bar")
        if closed_latch is not None and open_latch is not None:
            cl_min, cl_max = closed_latch
            ol_min, ol_max = open_latch
            ctx.check(
                "latch bar swings across the handle gap at 90 degrees",
                ol_max[1] > cl_max[1] + 0.03,
                details=f"closed_max_y={cl_max[1]:.4f} open_max_y={ol_max[1]:.4f}",
            )
            ctx.check(
                "latch bar reaches the opposite handle side at full open",
                ol_max[1] > 0.01,
                details=f"open_latch_max_y={ol_max[1]:.4f}",
            )
        else:
            ctx.fail("latch open pose AABBs resolve", "missing latch bar AABB")

    return ctx.report()


object_model = build_object_model()
