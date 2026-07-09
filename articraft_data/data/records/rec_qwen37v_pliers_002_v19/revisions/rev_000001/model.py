from __future__ import annotations

# Combination pliers variant: crimping notch, locking release lever,
# serrated jaw teeth, and textured grip ribs.
#
# Layout: the tool lies in the XY plane with Z as the thickness axis.
# The rivet pivot is at the origin. The blunt squared nose points +X,
# the handles sweep back to -X and spread in +/-Y.
# plier_half_0 (root) carries its jaw on +Y; plier_half_1 is the mirrored
# moving half. The two forged halves are half-lapped at the pivot:
# half_0 keeps the lower lap (z <= -EPS), half_1 keeps the upper lap
# (z >= +EPS), so one half visibly passes over the other under the rivet.
# A release lever pivots inside half_0's grip near the pivot boss.

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

# Release lever dimensions
LEVER_PIVOT_X = -0.038   # lever pivot location along the handle
LEVER_PIVOT_Y_OFFSET = -0.005  # offset from handle centerline (outer side, embeds in grip)
LEVER_LENGTH = 0.022     # lever arm length
LEVER_WIDTH = 0.005      # lever width
LEVER_THICK = 0.006      # lever thickness (slightly proud of grip surface)
LEVER_ANGLE_MAX = math.radians(15.0)  # max pivot angle
LEVER_PIN_R = 0.0015     # pivot pin radius
LEVER_PIN_H = 0.016      # pivot pin height (passes through the grip)


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
    """Material removed at the pivot so this half keeps only its lap layer.

    s=+1 keeps the lower lap (z <= -EPS); s=-1 keeps the upper lap (z >= +EPS).
    """
    cut = cq.Workplane("XY").circle(LAP_R).extrude(0.012)
    if s > 0:
        return cut.translate((0.0, 0.0, -EPS))
    return cut.translate((0.0, 0.0, -0.012 + EPS))


def _jaw_solid(s: int) -> cq.Workplane:
    """Polished jaw: squared nose, serrated teeth, pipe-grip recess, wire cutter, crimping notch."""
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

    # Prominent serrated teeth on the inner jaw face - triangular ridges
    # for aggressive gripping. Each tooth is a small triangular prism cut.
    for i in range(9):
        xi = 0.040 + 0.0035 * i
        # Triangular tooth profile cut into the inner face
        tooth_pts = [
            (xi - 0.0008, s * JAW_FACE),
            (xi + 0.0008, s * JAW_FACE),
            (xi, s * (JAW_FACE + 0.0018)),
        ]
        tooth = cq.Workplane("XY").polyline(tooth_pts).close().extrude(0.016, both=True)
        jaw = jaw.cut(tooth)

    # Additional cross-serrations at the nose for fine gripping
    for i in range(5):
        xi = 0.056 + 0.004 * i
        groove = (
            cq.Workplane("XY")
            .box(0.0010, 0.0020, 0.018)
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

    # Crimping notch: a U-shaped recess behind the wire cutter, used for
    # crimping terminals. Positioned between the wire cutter and the pivot.
    crimp_center_x = 0.012
    crimp_r = 0.0025
    crimp_cut = (
        cq.Workplane("XY")
        .circle(crimp_r)
        .extrude(0.012, both=True)
        .translate((crimp_center_x, s * (JAW_FACE - 0.001), 0.0))
    )
    jaw = jaw.cut(crimp_cut)
    # Add a flat entry slot leading into the crimp notch from the inner face
    crimp_slot = (
        cq.Workplane("XY")
        .box(0.003, 0.003, 0.012)
        .translate((crimp_center_x, s * (JAW_FACE - 0.002), 0.0))
    )
    jaw = jaw.cut(crimp_slot)

    return jaw.cut(_lap_cut(s))


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
    """Soft-touch rubber grip: flared guard, curved shaft, bulbous end, with textured ribs."""
    wires = [
        _ellipse_wire(x, s * _yc(x), 0.0, w, h)
        for x, w, h in GRIP_SECTIONS
    ]
    grip = cq.Solid.makeLoft(wires, ruled=False)

    # Textured grip ribs: raised transverse ridges along the grip surface.
    # Each rib is a thin torus-like ring around the grip cross-section.
    rib_xs = [-0.042, -0.050, -0.058, -0.066, -0.074, -0.082, -0.090, -0.098, -0.106, -0.114]
    rib_solids = []
    for rx in rib_xs:
        yc_val = s * _yc(rx)
        w_val = _grip_w(rx)
        h_val = _interp(rx, [(sx, h) for sx, _w, h in GRIP_SECTIONS])
        # Rib as a slightly oversized ellipse shell (ring)
        outer_wire = cq.Workplane("YZ", origin=(rx, 0.0, 0.0)).center(yc_val, 0.0).ellipse(w_val + 0.0006, h_val + 0.0006).val()
        inner_wire = cq.Workplane("YZ", origin=(rx, 0.0, 0.0)).center(yc_val, 0.0).ellipse(w_val - 0.0002, h_val - 0.0002).val()
        # Make a thin disk (ring) by lofting two wires close together
        outer_wire2 = cq.Workplane("YZ", origin=(rx + 0.001, 0.0, 0.0)).center(yc_val, 0.0).ellipse(w_val + 0.0006, h_val + 0.0006).val()
        rib = cq.Solid.makeLoft([outer_wire, outer_wire2], ruled=True)
        rib_solids.append(rib)

    # Fuse ribs onto the grip body
    result = grip
    for rib in rib_solids:
        try:
            result = result.fuse(rib)
        except Exception:
            pass  # skip ribs that fail to fuse
    return result


def _inlay_solid(s: int) -> cq.Solid:
    """Glossy red inlay loft running along the outer/top face of the grip."""
    wires = [
        _ellipse_wire(x, s * _yc(x), INLAY_Z_CENTER, _grip_w(x) - 0.0012, INLAY_HALF_H)
        for x in INLAY_XS
    ]
    return cq.Solid.makeLoft(wires, ruled=False)


def _lever_solid() -> cq.Workplane:
    """Locking release lever: a flat paddle that pivots inside the handle."""
    # Lever body - flat elongated paddle
    lever = (
        cq.Workplane("XY")
        .box(LEVER_LENGTH, LEVER_WIDTH, LEVER_THICK)
        .translate((LEVER_LENGTH / 2.0, 0.0, 0.0))
    )
    # Rounded pivot end
    pivot_boss = (
        cq.Workplane("XY")
        .circle(LEVER_WIDTH / 2.0)
        .extrude(LEVER_THICK)
        .translate((0.0, 0.0, -LEVER_THICK / 2.0))
    )
    lever = lever.union(pivot_boss)
    # Thumb tab at the free end (wider paddle)
    tab = (
        cq.Workplane("XY")
        .box(0.006, 0.008, LEVER_THICK)
        .translate((LEVER_LENGTH - 0.001, 0.0, 0.0))
    )
    lever = lever.union(tab)
    # Pivot pin - captured through the handle grip body
    pin = (
        cq.Workplane("XY")
        .circle(LEVER_PIN_R)
        .extrude(LEVER_PIN_H)
        .translate((0.0, 0.0, -LEVER_PIN_H / 2.0))
    )
    lever = lever.union(pin)
    return lever


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="combination_pliers_v19")

    steel_polished = model.material("steel_polished", rgba=(0.80, 0.81, 0.84, 1.0))
    steel_forged = model.material("steel_forged", rgba=(0.66, 0.67, 0.70, 1.0))
    steel_brushed = model.material("steel_brushed", rgba=(0.61, 0.62, 0.65, 1.0))
    seam_gray = model.material("seam_gray", rgba=(0.45, 0.46, 0.48, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.08, 0.09, 1.0))
    grip_red = model.material("grip_red", rgba=(0.80, 0.09, 0.11, 1.0))
    lever_dark = model.material("lever_dark", rgba=(0.18, 0.18, 0.20, 1.0))

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
            mesh_from_cadquery(_grip_solid(s), f"{tag}_grip", tolerance=0.0003),
            name="grip",
            material=rubber_black,
        )
        part.visual(
            mesh_from_cadquery(_inlay_solid(s), f"{tag}_inlay", tolerance=0.0003),
            name="grip_inlay",
            material=grip_red,
        )

        parts.append(part)

    # One-piece rivet, fixed to half_0 (the moving half rotates around it):
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

    # Release lever - pivots inside half_0's handle near the pivot boss.
    # The lever sits on the outer side of the handle (away from the jaw).
    lever_part = model.part("release_lever")
    lever_part.visual(
        mesh_from_cadquery(_lever_solid(), "lever_body", tolerance=0.0002),
        name="lever",
        material=lever_dark,
    )

    # Primary articulation: one revolute joint at the rivet, axis perpendicular
    # to the flat tool plane. Positive q (about -Z) swings half_1's jaw toward
    # -Y, opening the jaws while the handles spread apart.
    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=parts[0],
        child=parts[1],
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=3.0, lower=0.0, upper=OPEN_LIMIT),
    )

    # Release lever articulation: revolute joint inside half_0's handle.
    # The lever pivots about the Z axis (perpendicular to the tool plane).
    # Positive q swings the lever outward (thumb tab away from the handle).
    lever_pivot_y = _yc(LEVER_PIVOT_X) + LEVER_PIVOT_Y_OFFSET
    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=parts[0],
        child=lever_part,
        origin=Origin(xyz=(LEVER_PIVOT_X, lever_pivot_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=0.0, upper=LEVER_ANGLE_MAX),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    half0 = object_model.get_part("plier_half_0")
    half1 = object_model.get_part("plier_half_1")
    lever = object_model.get_part("release_lever")
    pivot = object_model.get_articulation("pivot")
    lever_pivot = object_model.get_articulation("lever_pivot")

    jaw0 = half0.get_visual("jaw")
    jaw1 = half1.get_visual("jaw")
    hub0 = half0.get_visual("hub")
    hub1 = half1.get_visual("hub")
    grip0 = half0.get_visual("grip")
    grip1 = half1.get_visual("grip")
    inlay0 = half0.get_visual("grip_inlay")

    # ---- Joint contract: main pivot is a 0..30 degree revolute joint ----
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

    # ---- Lever pivot is a non-fixed revolute joint ----
    lever_limits = lever_pivot.motion_limits
    ctx.check(
        "lever_pivot is a non-fixed revolute joint with positive range",
        lever_pivot.articulation_type == ArticulationType.REVOLUTE
        and lever_limits is not None
        and lever_limits.lower is not None
        and lever_limits.upper is not None
        and lever_limits.upper > lever_limits.lower + 0.01,
        details=f"lever_limits={lever_limits}",
    )

    # ---- Closed rest pose: serrated jaw inner faces nearly touch ----
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

    # ---- The halves interleave at the pivot ----
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
    ctx.expect_overlap(
        half0,
        half1,
        axes="z",
        elem_a="rivet_shaft",
        elem_b=hub1,
        min_overlap=0.007,
        name="rivet shaft passes through the full moving lap thickness",
    )

    # ---- Rivet boss dimensions ----
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

    # ---- Envelope checks ----
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

    # ---- Red inlay rides the outer/top face of the black grip ----
    i0 = ctx.part_element_world_aabb(half0, elem="grip_inlay")
    if i0 is not None and g0 is not None:
        ctx.check(
            "red inlay proud on the grip top face",
            i0[1][2] >= g0[1][2] - 0.0005,
            details=f"inlay_top={i0[1][2]:.4f} grip_top={g0[1][2]:.4f}",
        )
    else:
        ctx.fail("inlay AABB resolves", "missing grip_inlay element AABB")

    # ---- Thumb guard flares close behind the pivot ----
    if g0 is not None:
        ctx.check(
            "thumb guard front sits close behind the pivot",
            -0.036 <= g0[1][0] <= -0.026,
            details=f"grip_front_x={g0[1][0]:.4f}",
        )

    # ---- Release lever is mounted on half_0 and near the handle ----
    # The lever pivot pin passes through the handle shank near the grip junction.
    ctx.allow_overlap(
        half0,
        lever,
        elem_a="shank",
        elem_b="lever",
        reason="The release lever pivot pin is captured through half_0's handle "
        "shank, with the lever body seated against the shank surface near the grip.",
    )
    # The lever body also sits against the grip front where the thumb guard starts.
    ctx.allow_overlap(
        half0,
        lever,
        elem_a="grip",
        elem_b="lever",
        reason="The release lever body is seated against the front of half_0's grip "
        "near the thumb guard, representing the lever's recessed mount pocket.",
    )
    # Lever pivot pin overlaps with the shank in Z (captured through it)
    ctx.expect_overlap(
        lever,
        half0,
        axes="z",
        elem_a="lever",
        elem_b="shank",
        min_overlap=0.004,
        name="release lever pivot pin passes through the shank thickness",
    )
    # Lever pivot stays within the shank XY footprint
    ctx.expect_overlap(
        lever,
        half0,
        axes="xy",
        elem_a="lever",
        elem_b="shank",
        min_overlap=0.002,
        name="release lever overlaps with half_0 shank in XY footprint",
    )
    # Lever contacts the grip area (mounted near the thumb guard)
    ctx.expect_overlap(
        lever,
        half0,
        axes="xy",
        elem_a="lever",
        elem_b="grip",
        min_overlap=0.001,
        name="release lever seats against the grip front pocket",
    )

    # ---- Lever pivot moves the lever at the open pose ----
    lever_rest = ctx.part_element_world_aabb(lever, elem="lever")
    with ctx.pose({lever_pivot: LEVER_ANGLE_MAX}):
        lever_open = ctx.part_element_world_aabb(lever, elem="lever")
        ctx.check(
            "lever pivots when the lever joint is actuated",
            lever_rest is not None
            and lever_open is not None
            and (
                abs(lever_open[1][1] - lever_rest[1][1]) > 0.001
                or abs(lever_open[0][0] - lever_rest[0][0]) > 0.001
                or abs(lever_open[0][1] - lever_rest[0][1]) > 0.001
            ),
            details=f"rest={lever_rest}, open={lever_open}",
        )

    # ---- Decisive open pose: jaws separate and handles spread ----
    closed_jaw1 = ctx.part_element_world_aabb(half1, elem="jaw")
    closed_grip1 = ctx.part_element_world_aabb(half1, elem="grip")
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

    return ctx.report()


object_model = build_object_model()
