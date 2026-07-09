from __future__ import annotations

# Locking pliers variant (Vise-Grip style).
# Forked from heavy-duty combination lineman pliers.
# Changes: locking-style serrated jaw teeth, rear adjustment screw
# (continuous joint at the handle end), textured grip ribs on both handles.
#
# Layout: the tool lies in the XY plane with Z as the thickness axis.
# The rivet pivot is at the origin. The blunt squared nose points +X,
# the handles sweep back to -X and spread in +/-Y.
# plier_half_0 (root) carries its jaw on +Y; plier_half_1 is the mirrored
# moving half. The two forged halves are half-lapped at the pivot.

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
RIVET_R = 0.004         # rivet shaft
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
    (-0.032, 0.0100, 0.0130),  # flared thumb guard near the pivot
    (-0.040, 0.0080, 0.0110),
    (-0.060, 0.0078, 0.0108),
    (-0.085, 0.0080, 0.0112),
    (-0.105, 0.0086, 0.0118),
    (-0.120, 0.0088, 0.0120),  # bulbous end swell
    (-0.128, 0.0062, 0.0086),
    (-0.131, 0.0028, 0.0040),
]

INLAY_XS = [-0.036, -0.055, -0.075, -0.095, -0.112, -0.118]
INLAY_HALF_H = 0.0075
INLAY_Z_CENTER = 0.006

# Adjustment screw dimensions
SCREW_R = 0.005          # screw body radius (10 mm diameter)
SCREW_H = 0.008          # screw body length
SCREW_HEAD_R = 0.0065    # knurled head flange radius
SCREW_HEAD_H = 0.003     # knurled head flange height

# Grip rib dimensions
RIB_HEIGHT = 0.0008      # rib protrusion from grip surface
RIB_WIDTH = 0.0012       # rib thickness along grip length


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


def _grip_h(x: float) -> float:
    return _interp(x, [(sx, h) for sx, _w, h in GRIP_SECTIONS])


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
    cut = cq.Workplane("XY").circle(LAP_R).extrude(0.012)
    if s > 0:
        return cut.translate((0.0, 0.0, -EPS))
    return cut.translate((0.0, 0.0, -0.012 + EPS))


def _jaw_solid(s: int) -> cq.Workplane:
    """Locking-pliers jaw: curved profile with prominent serrated teeth."""
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

    # Prominent serrated teeth on the inner jaw face (locking pliers style).
    # V-shaped tooth notches creating aggressive gripping pattern.
    tooth_spacing = 0.005
    tooth_depth = 0.003
    tooth_half_w = 0.0018
    for i in range(10):
        xi = 0.024 + tooth_spacing * i
        if xi > 0.073:
            break
        tooth_pts = [
            (xi - tooth_half_w, s * JAW_FACE),
            (xi + tooth_half_w, s * JAW_FACE),
            (xi, s * (JAW_FACE + tooth_depth)),
        ]
        tooth_cut = (
            cq.Workplane("XY")
            .polyline(tooth_pts)
            .close()
            .extrude(0.020, both=True)
        )
        jaw = jaw.cut(tooth_cut)

    # Secondary cross-serrations between teeth for extra grip
    for i in range(5):
        xi = 0.028 + 0.009 * i
        groove = (
            cq.Workplane("XY")
            .box(0.001, 0.004, 0.018)
            .translate((xi, s * (JAW_FACE + 0.001), 0.0))
        )
        jaw = jaw.cut(groove)

    # Rounded pipe-grip recess behind the nose
    recess = (
        cq.Workplane("XY")
        .circle(0.0045)
        .extrude(0.011, both=True)
        .translate((0.030, s * JAW_FACE, 0.0))
    )
    jaw = jaw.cut(recess)

    # Scallops around pipe grip rim
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

    # Wire-cutter V-notch near the pivot
    notch_pts = [(0.015, s * -0.001), (0.0215, s * -0.001), (0.0185, s * 0.0035)]
    notch = cq.Workplane("XY").polyline(notch_pts).close().extrude(0.011, both=True)
    jaw = jaw.cut(notch)

    return jaw.cut(_lap_cut(s))


def _hub_solid(s: int) -> cq.Workplane:
    hub = cq.Workplane("XY").circle(HUB_R).extrude(HALF_T, both=True)
    return hub.cut(_lap_cut(s))


def _shank_solid(s: int) -> cq.Workplane:
    pts = [(x, s * y) for x, y in TANG_PTS]
    loop = _strip_loop(pts, TANG_HALF_W)
    shank = cq.Workplane("XY").polyline(loop).close().extrude(HALF_T, both=True)
    return shank.cut(_lap_cut(s))


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


def _grip_ribs_solid(s: int) -> cq.Workplane:
    """Textured grip ribs: thin raised rectangular-frame rings around the grip."""
    rib_xs = [-0.040, -0.050, -0.060, -0.070, -0.080, -0.090, -0.100, -0.110]

    ribs = None
    for xi in rib_xs:
        yc_val = s * _yc(xi)
        gw = _grip_w(xi)
        gh = _grip_h(xi)
        # Outer frame slightly larger than grip cross-section
        outer_w = gw + RIB_HEIGHT
        outer_h = gh + RIB_HEIGHT
        # Inner cutout slightly smaller than grip (ribs embed into grip surface)
        inner_w = gw - 0.0004
        inner_h = gh - 0.0004

        outer = (
            cq.Workplane("XY")
            .box(RIB_WIDTH, 2.0 * outer_w, 2.0 * outer_h)
            .translate((xi, yc_val, 0.0))
        )
        inner = (
            cq.Workplane("XY")
            .box(RIB_WIDTH + 0.001, 2.0 * inner_w, 2.0 * inner_h)
            .translate((xi, yc_val, 0.0))
        )
        rib = outer.cut(inner)

        if ribs is None:
            ribs = rib
        else:
            ribs = ribs.union(rib)

    return ribs


def _screw_solid() -> cq.Workplane:
    """Adjustment screw: knurled thumb screw at the rear handle end.

    Built along +Z (standard CadQuery cylinder axis). The visual origin
    rotates it to align with the handle axis.
    """
    # Main threaded body
    body = cq.Workplane("XY").circle(SCREW_R).extrude(SCREW_H)

    # Knurled head flange (wider disc at the outer end)
    head = (
        cq.Workplane("XY")
        .circle(SCREW_HEAD_R)
        .extrude(SCREW_HEAD_H)
        .translate((0.0, 0.0, SCREW_H))
    )
    body = body.union(head)

    # Longitudinal knurl grooves around the head circumference
    for i in range(20):
        angle = math.radians(i * 18.0)
        gx = SCREW_HEAD_R * math.cos(angle)
        gy = SCREW_HEAD_R * math.sin(angle)
        groove = (
            cq.Workplane("XY")
            .circle(0.0005)
            .extrude(SCREW_HEAD_H)
            .translate((gx, gy, SCREW_H))
        )
        body = body.cut(groove)

    # Screwdriver slot across the head top face
    slot = (
        cq.Workplane("XY")
        .box(0.009, 0.0015, 0.002)
        .translate((0.0, 0.0, SCREW_H + SCREW_HEAD_H - 0.0005))
    )
    body = body.cut(slot)

    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="locking_pliers")

    steel_polished = model.material("steel_polished", rgba=(0.80, 0.81, 0.84, 1.0))
    steel_forged = model.material("steel_forged", rgba=(0.66, 0.67, 0.70, 1.0))
    steel_brushed = model.material("steel_brushed", rgba=(0.61, 0.62, 0.65, 1.0))
    seam_gray = model.material("seam_gray", rgba=(0.45, 0.46, 0.48, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.08, 0.09, 1.0))
    grip_red = model.material("grip_red", rgba=(0.80, 0.09, 0.11, 1.0))
    screw_steel = model.material("screw_steel", rgba=(0.52, 0.53, 0.55, 1.0))
    rib_rubber = model.material("rib_rubber", rgba=(0.14, 0.14, 0.15, 1.0))

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
        # Textured grip ribs wrapping around the grip
        part.visual(
            mesh_from_cadquery(_grip_ribs_solid(s), f"{tag}_grip_ribs", tolerance=0.0003),
            name="grip_ribs",
            material=rib_rubber,
        )

        parts.append(part)

    # One-piece rivet, fixed to half_0
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

    # ---- Adjustment screw part at the rear of the fixed handle ----
    screw_part = model.part("adjustment_screw")
    # Handle end for s=1 half: approximately (-0.131, -0.0274, 0)
    # The screw threads into the handle end; knurled head protrudes outward.
    screw_mount_x = -0.131
    screw_mount_y = -0.0274

    # Screw built along +Z; rotate Ry(pi/2) maps +Z to -X (outward from handle).
    # Small +X offset embeds the threaded end slightly into the handle.
    screw_part.visual(
        mesh_from_cadquery(_screw_solid(), "screw_body", tolerance=0.0002),
        # Ry(-pi/2) maps +Z (CadQuery cylinder axis) to -X (outward from handle end)
        origin=Origin(xyz=(0.002, 0.0, 0.0), rpy=(0.0, -math.pi / 2.0, 0.0)),
        name="screw",
        material=screw_steel,
    )

    # ---- Primary pivot articulation ----
    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=parts[0],
        child=parts[1],
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=3.0, lower=0.0, upper=OPEN_LIMIT),
    )

    # ---- Adjustment screw: continuous rotation at the rear handle ----
    model.articulation(
        "adjustment_screw",
        ArticulationType.CONTINUOUS,
        parent=parts[0],
        child=screw_part,
        origin=Origin(xyz=(screw_mount_x, screw_mount_y, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    half0 = object_model.get_part("plier_half_0")
    half1 = object_model.get_part("plier_half_1")
    screw = object_model.get_part("adjustment_screw")
    pivot = object_model.get_articulation("pivot")
    adj_screw_joint = object_model.get_articulation("adjustment_screw")

    jaw0 = half0.get_visual("jaw")
    jaw1 = half1.get_visual("jaw")
    hub0 = half0.get_visual("hub")
    hub1 = half1.get_visual("hub")
    grip0 = half0.get_visual("grip")
    grip1 = half1.get_visual("grip")
    ribs0 = half0.get_visual("grip_ribs")
    ribs1 = half1.get_visual("grip_ribs")
    screw_vis = screw.get_visual("screw")

    # ---- Joint contract: pivot is revolute 0..30 degrees ----
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

    # ---- Adjustment screw joint is continuous (no position limits) ----
    screw_limits = adj_screw_joint.motion_limits
    ctx.check(
        "adjustment_screw is a continuous joint (no position limits)",
        adj_screw_joint.articulation_type == ArticulationType.CONTINUOUS
        and screw_limits is not None
        and screw_limits.lower is None
        and screw_limits.upper is None,
        details=f"type={adj_screw_joint.articulation_type} limits={screw_limits}",
    )

    # ---- Grip ribs exist on both halves ----
    ctx.check(
        "grip ribs present on fixed half",
        ribs0 is not None,
        details="missing grip_ribs visual on plier_half_0",
    )
    ctx.check(
        "grip ribs present on moving half",
        ribs1 is not None,
        details="missing grip_ribs visual on plier_half_1",
    )

    # ---- Grip ribs sit along the grip length (not at jaw or pivot) ----
    r0_aabb = ctx.part_element_world_aabb(half0, elem="grip_ribs")
    if r0_aabb is not None:
        ctx.check(
            "grip ribs span the handle region (x between -0.12 and -0.035)",
            r0_aabb[0][0] >= -0.125 and r0_aabb[1][0] <= -0.030,
            details=f"ribs_aabb_x=[{r0_aabb[0][0]:.4f}, {r0_aabb[1][0]:.4f}]",
        )
    else:
        ctx.fail("grip ribs AABB resolves", "missing grip_ribs element AABB")

    # ---- Grip ribs overlap the grip footprint in XY ----
    g0_aabb = ctx.part_element_world_aabb(half0, elem="grip")
    if r0_aabb is not None and g0_aabb is not None:
        ctx.check(
            "grip ribs overlap the grip footprint in X",
            r0_aabb[0][0] >= g0_aabb[0][0] - 0.005
            and r0_aabb[1][0] <= g0_aabb[1][0] + 0.005,
            details=f"ribs_x=[{r0_aabb[0][0]:.4f},{r0_aabb[1][0]:.4f}] "
            f"grip_x=[{g0_aabb[0][0]:.4f},{g0_aabb[1][0]:.4f}]",
        )
        # Ribs should have meaningful Z extent (raised ridges wrap around grip)
        rib_z_span = r0_aabb[1][2] - r0_aabb[0][2]
        ctx.check(
            "grip ribs have raised ridge Z extent (> 0.018 m)",
            rib_z_span > 0.018,
            details=f"ribs_z_span={rib_z_span:.4f}",
        )

    # ---- Adjustment screw visual exists and is positioned at handle end ----
    screw_aabb = ctx.part_element_world_aabb(screw, elem="screw")
    if screw_aabb is not None:
        ctx.check(
            "adjustment screw is at the rear of the handle (x < -0.125)",
            screw_aabb[0][0] < -0.125,
            details=f"screw_min_x={screw_aabb[0][0]:.4f}",
        )
        ctx.check(
            "adjustment screw head protrudes beyond handle end",
            screw_aabb[0][0] < -0.133,
            details=f"screw_min_x={screw_aabb[0][0]:.4f}",
        )
    else:
        ctx.fail("screw AABB resolves", "missing screw element AABB")

    # ---- Serrated teeth: jaw inner face has tooth pattern ----
    # Check that the jaw extends inward (tooth depth adds material removal near inner face)
    jaw0_aabb = ctx.part_element_world_aabb(half0, elem="jaw")
    if jaw0_aabb is not None:
        ctx.check(
            "jaw has locking-pliers profile (extends to nose at ~0.075 m)",
            jaw0_aabb[1][0] >= 0.070,
            details=f"jaw_max_x={jaw0_aabb[1][0]:.4f}",
        )

    # ---- Closed rest pose: jaws nearly touching ----
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

    # ---- Hub interleaving at pivot ----
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

    # ---- Rivet boss: ~25 mm diameter, centered on pivot ----
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

    # ---- Screw threaded into handle end (intentional overlap) ----
    ctx.allow_overlap(
        half0,
        screw,
        elem_a="grip",
        elem_b="screw",
        reason="The adjustment screw threads into the handle end; the "
        "threaded portion is intentionally embedded inside the grip.",
    )
    ctx.expect_overlap(
        half0,
        screw,
        axes="y",
        elem_a="grip",
        elem_b="screw",
        min_overlap=0.002,
        name="adjustment screw shares footprint with handle grip end",
    )

    # ---- Envelope: ~0.20 m long, ~0.07 m across ----
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
            0.19 <= length <= 0.22,
            details=f"length={length:.4f}",
        )
        ctx.check(
            "handle grips span about 0.07 m across",
            0.060 <= across <= 0.082,
            details=f"across={across:.4f}",
        )
    else:
        ctx.fail("envelope AABBs resolve", "missing part/grip AABB")

    # ---- Open pose: jaws separate and handles spread ----
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

    # ---- Screw rotation: joint axis passes through the screw center ----
    # Verify the screw stays at its mount point when rotated (correct axis).
    rest_screw_pos = ctx.part_world_position(screw)
    with ctx.pose({adj_screw_joint: math.pi}):
        rotated_screw_pos = ctx.part_world_position(screw)
        if rest_screw_pos is not None and rotated_screw_pos is not None:
            drift = math.sqrt(
                sum((a - b) ** 2 for a, b in zip(rest_screw_pos, rotated_screw_pos))
            )
            ctx.check(
                "adjustment screw rotates in place (axis through center)",
                drift < 0.003,
                details=f"rest={rest_screw_pos} rotated={rotated_screw_pos} drift={drift:.4f}",
            )
        screw_rotated_aabb = ctx.part_element_world_aabb(screw, elem="screw")
        if screw_aabb is not None and screw_rotated_aabb is not None:
            # The screw X span should remain stable (rotation is about X axis)
            x_span_rest = screw_aabb[1][0] - screw_aabb[0][0]
            x_span_rot = screw_rotated_aabb[1][0] - screw_rotated_aabb[0][0]
            ctx.check(
                "screw X span stable under rotation (axis along X)",
                abs(x_span_rest - x_span_rot) < 0.002,
                details=f"rest_x_span={x_span_rest:.4f} rot_x_span={x_span_rot:.4f}",
            )

    return ctx.report()


object_model = build_object_model()
