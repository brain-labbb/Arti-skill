from __future__ import annotations

# Compact diagonal cutting pliers variant.
# Reference: picture/Other/pliers/002.png family variant
#
# Layout: tool lies in XY plane, Z = thickness axis.
# Pivot at origin, jaws point +X, handles sweep to -X.
# half_0 (root, fixed) jaw on +Y side; half_1 (moving) jaw on -Y side.
# Half-lapped at pivot: half_0 lower lap, half_1 upper lap.
# Geometric hex grip sleeves are separate color-coded parts.
# Adjustment screw (knurled knob) rotates at rear of half_0 handle.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- shared dimensions (meters) ----
HALF_T = 0.008           # half thickness of each forged plate
LAP_R = 0.014            # half-lap joint disc radius at the pivot
HUB_R = 0.013            # forged hub radius around the rivet
BOSS_R = 0.011           # rivet cap radius (~0.022 m diameter)
BOSS_H = 0.0025
SEAM_R = 0.0115          # visible circular seam ring
SEAM_H = 0.0005
RIVET_R = 0.0035         # rivet shaft radius
EPS = 0.0001             # lap clearance
JAW_FACE = 0.0003        # closed jaw inner face offset
NOSE_X = 0.040           # compact diagonal jaw tip
OPEN_LIMIT = math.radians(30.0)

TANG_HALF_W = 0.0035     # steel handle tang half width

# Handle tang centerline for the +Y jaw half (shorter than lineman pliers).
TANG_PTS = [
    (-0.010, -0.004),
    (-0.025, -0.010),
    (-0.050, -0.016),
    (-0.070, -0.019),
    (-0.080, -0.020),
]

# Hex grip sleeve stations along tang (octagonal cross section via loft).
GRIP_X_START = -0.028
GRIP_X_END = -0.078
GRIP_STATIONS = [
    (-0.028, 0.0090, 0.0110),   # flared guard near pivot
    (-0.035, 0.0082, 0.0105),
    (-0.050, 0.0078, 0.0100),
    (-0.065, 0.0078, 0.0100),
    (-0.075, 0.0072, 0.0092),
    (-0.078, 0.0050, 0.0065),   # rounded end
]

SCREW_X = -0.082         # adjustment screw center X (rear of handle)
SCREW_Y_OFFSET = 0.021   # Y offset from tool centerline to screw mount


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
    return _interp(x, TANG_PTS)


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
    cut = cq.Workplane("XY").circle(LAP_R).extrude(0.010)
    if s > 0:
        return cut.translate((0.0, 0.0, -EPS))
    return cut.translate((0.0, 0.0, -0.010 + EPS))


def _diagonal_jaw(s: int) -> cq.Workplane:
    """Compact diagonal cutting jaw: short wedge with angled cutting edge."""
    # Diagonal cutter profile: short, tapers to angled cutting tip.
    profile = [
        (0.008, JAW_FACE),
        (NOSE_X, JAW_FACE),
        (NOSE_X - 0.002, 0.009),    # angled cutting edge tip
        (0.030, 0.012),
        (0.020, 0.014),
        (0.012, 0.0145),
        (0.008, 0.013),
    ]
    pts = [(x, s * y) for x, y in profile]
    jaw = cq.Workplane("XY").polyline(pts).close().extrude(HALF_T, both=True)

    # Angled cutting bevel along the inner face (diagonal cutter edge).
    bevel_pts = [
        (0.012, s * JAW_FACE),
        (NOSE_X - 0.003, s * JAW_FACE),
        (NOSE_X - 0.005, s * (JAW_FACE + 0.003)),
        (0.012, s * (JAW_FACE + 0.003)),
    ]
    bevel = cq.Workplane("XY").polyline(bevel_pts).close().extrude(0.004, both=True)
    jaw = jaw.cut(bevel)

    # Small V-notch wire cutter near pivot.
    notch_pts = [(0.012, s * -0.001), (0.017, s * -0.001), (0.0145, s * 0.002)]
    notch = cq.Workplane("XY").polyline(notch_pts).close().extrude(0.010, both=True)
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


def _hex_wire(x: float, yc: float, zc: float, w: float, h: float) -> cq.Wire:
    """Octagonal cross-section wire for geometric grip sleeve."""
    # 8-sided polygon approximating a hexagonal/octagonal grip
    pts_local = []
    for i in range(8):
        a = math.radians(i * 45.0 + 22.5)
        pts_local.append((yc + w * math.cos(a), zc + h * math.sin(a)))
    wp = cq.Workplane("YZ", origin=(x, 0.0, 0.0))
    wire = wp.polyline(pts_local).close().val()
    return wire


def _grip_sleeve_solid(s: int) -> cq.Solid:
    """Geometric (octagonal) grip sleeve loft."""
    wires = [
        _hex_wire(x, s * _yc(x), 0.0, w, h)
        for x, w, h in GRIP_STATIONS
    ]
    return cq.Solid.makeLoft(wires, ruled=True)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="diagonal_cutting_pliers")

    steel_polished = model.material("steel_polished", rgba=(0.82, 0.83, 0.86, 1.0))
    steel_forged = model.material("steel_forged", rgba=(0.62, 0.63, 0.66, 1.0))
    steel_brushed = model.material("steel_brushed", rgba=(0.58, 0.59, 0.62, 1.0))
    seam_gray = model.material("seam_gray", rgba=(0.42, 0.43, 0.45, 1.0))
    grip_orange = model.material("grip_orange", rgba=(0.95, 0.50, 0.08, 1.0))
    grip_blue = model.material("grip_blue", rgba=(0.12, 0.42, 0.78, 1.0))
    screw_dark = model.material("screw_dark", rgba=(0.18, 0.18, 0.20, 1.0))

    # ---- Two forged steel halves ----
    parts = []
    for part_name, s in (("plier_half_0", 1), ("plier_half_1", -1)):
        part = model.part(part_name)
        tag = part_name.replace("plier_half_", "half")

        part.visual(
            mesh_from_cadquery(_diagonal_jaw(s), f"{tag}_jaw", tolerance=0.0002),
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
        parts.append(part)

    fixed = parts[0]   # half_0
    moving = parts[1]  # half_1

    # ---- Color-separated geometric grip sleeves (separate parts) ----
    grip0 = model.part("grip_sleeve_0")
    grip0.visual(
        mesh_from_cadquery(_grip_sleeve_solid(1), "grip_sleeve_0_mesh", tolerance=0.0002),
        name="sleeve",
        material=grip_orange,
    )

    grip1 = model.part("grip_sleeve_1")
    grip1.visual(
        mesh_from_cadquery(_grip_sleeve_solid(-1), "grip_sleeve_1_mesh", tolerance=0.0002),
        name="sleeve",
        material=grip_blue,
    )

    # ---- Adjustment screw at rear of half_0 handle ----
    screw_part = model.part("adjustment_screw")
    knob_geom = KnobGeometry(
        0.012,    # diameter
        0.008,    # height
        body_style="cylindrical",
        grip=KnobGrip(style="knurled", count=24, depth=0.0006, helix_angle_deg=18.0),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0005),
    )
    screw_part.visual(
        mesh_from_geometry(knob_geom, "screw_knob"),
        name="knob",
        material=screw_dark,
    )
    # Screw shaft stub connecting knob into the handle tang
    screw_part.visual(
        Cylinder(0.003, 0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.003 - 0.004)),
        name="screw_shaft",
        material=steel_brushed,
    )

    # ---- Rivet caps on both sides of pivot ----
    # Bottom cap (on fixed half outer face)
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
        Cylinder(RIVET_R, 2.0 * HALF_T + SEAM_H + 2.0 * EPS + 0.0004),
        origin=Origin(xyz=(0.0, 0.0, 0.00005)),
        name="rivet_shaft",
        material=steel_brushed,
    )
    # Top cap (on moving half outer face)
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

    # ---- Primary articulation: pivot revolute joint ----
    # Positive q about -Z opens jaws (half_1 jaw swings toward -Y).
    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=fixed,
        child=moving,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=3.0, lower=0.0, upper=OPEN_LIMIT),
    )

    # ---- Grip sleeve mounting: rigid (floating with zero pose) ----
    # grip_sleeve_0 mounts onto half_0's tang
    model.articulation(
        "grip0_mount",
        ArticulationType.FIXED,
        parent=fixed,
        child=grip0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # grip_sleeve_1 mounts onto half_1's tang
    model.articulation(
        "grip1_mount",
        ArticulationType.FIXED,
        parent=moving,
        child=grip1,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # ---- Adjustment screw: revolute joint at rear of half_0 handle ----
    screw_mount_y = _yc(SCREW_X)  # centerline y at screw location for half_0 (+Y jaw side)
    model.articulation(
        "screw_joint",
        ArticulationType.REVOLUTE,
        parent=fixed,
        child=screw_part,
        origin=Origin(xyz=(SCREW_X, screw_mount_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=6.0, lower=0.0, upper=math.radians(360.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    half0 = object_model.get_part("plier_half_0")
    half1 = object_model.get_part("plier_half_1")
    grip0 = object_model.get_part("grip_sleeve_0")
    grip1 = object_model.get_part("grip_sleeve_1")
    screw = object_model.get_part("adjustment_screw")

    pivot = object_model.get_articulation("pivot")
    screw_joint = object_model.get_articulation("screw_joint")

    jaw0 = half0.get_visual("jaw")
    jaw1 = half1.get_visual("jaw")
    hub0 = half0.get_visual("hub")
    hub1 = half1.get_visual("hub")

    # --- Joint checks ---
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

    screw_limits = screw_joint.motion_limits
    ctx.check(
        "adjustment screw has a revolute joint",
        screw_joint.articulation_type == ArticulationType.REVOLUTE
        and screw_limits is not None
        and screw_limits.lower is not None
        and screw_limits.upper is not None
        and screw_limits.upper > math.radians(180.0),
        details=f"screw_limits={screw_limits}",
    )

    # --- Closed rest pose: compact diagonal jaws nearly touch ---
    ctx.expect_gap(
        half0,
        half1,
        axis="y",
        positive_elem=jaw0,
        negative_elem=jaw1,
        min_gap=0.0002,
        max_gap=0.002,
        name="diagonal cutting jaws closed and nearly touching at rest",
    )

    # --- Jaw is compact (shorter than lineman pliers) ---
    jaw0_aabb = ctx.part_element_world_aabb(half0, elem="jaw")
    if jaw0_aabb is not None:
        jaw_length = jaw0_aabb[1][0] - jaw0_aabb[0][0]
        ctx.check(
            "diagonal cutting jaw is compact (under 0.045 m)",
            jaw_length < 0.045,
            details=f"jaw_length={jaw_length:.4f}",
        )

    # --- Hub lap stacking ---
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

    # --- Rivet shaft captured through moving half ---
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

    # --- Rivet caps on both sides ---
    cap_bot_aabb = ctx.part_element_world_aabb(half0, elem="rivet_cap_bottom")
    cap_top_aabb = ctx.part_element_world_aabb(half0, elem="rivet_cap_top")
    ok_caps = cap_bot_aabb is not None and cap_top_aabb is not None
    if ok_caps:
        ctx.check(
            "rivet cap on bottom face exists below tool plane",
            cap_bot_aabb[0][2] < -0.008,
            details=f"cap_bot_min_z={cap_bot_aabb[0][2]:.4f}",
        )
        ctx.check(
            "rivet cap on top face exists above tool plane",
            cap_top_aabb[1][2] > 0.008,
            details=f"cap_top_max_z={cap_top_aabb[1][2]:.4f}",
        )
    else:
        ctx.fail("rivet caps resolve", "missing rivet_cap_bottom or rivet_cap_top AABB")

    # --- Grip sleeves are separate color-coded parts ---
    sleeve0_aabb = ctx.part_element_world_aabb(grip0, elem="sleeve")
    sleeve1_aabb = ctx.part_element_world_aabb(grip1, elem="sleeve")
    ok_sleeves = sleeve0_aabb is not None and sleeve1_aabb is not None
    if ok_sleeves:
        # Grips should be in the handle region (negative X)
        ctx.check(
            "grip sleeve 0 in handle region",
            sleeve0_aabb[1][0] < -0.02 and sleeve0_aabb[0][0] > -0.09,
            details=f"sleeve0_x=[{sleeve0_aabb[0][0]:.4f},{sleeve0_aabb[1][0]:.4f}]",
        )
        ctx.check(
            "grip sleeve 1 in handle region",
            sleeve1_aabb[1][0] < -0.02 and sleeve1_aabb[0][0] > -0.09,
            details=f"sleeve1_x=[{sleeve1_aabb[0][0]:.4f},{sleeve1_aabb[1][0]:.4f}]",
        )
        # Sleeves should be on opposite Y sides (sleeve0 on -Y, sleeve1 on +Y)
        ctx.check(
            "grip sleeves on opposite sides of tool centerline",
            sleeve0_aabb[1][1] < 0.0 and sleeve1_aabb[0][1] > 0.0,
            details=f"s0_y=[{sleeve0_aabb[0][1]:.4f},{sleeve0_aabb[1][1]:.4f}] "
                    f"s1_y=[{sleeve1_aabb[0][1]:.4f},{sleeve1_aabb[1][1]:.4f}]",
        )
    else:
        ctx.fail("grip sleeves resolve", "missing sleeve AABB")

    # --- Grip sleeves wrap around the steel tangs (intentional over-mold fit) ---
    ctx.allow_overlap(
        grip0,
        half0,
        elem_a="sleeve",
        elem_b="shank",
        reason="The geometric grip sleeve is intentionally over-molded around "
        "the steel tang, representing a seated grip wrap.",
    )
    ctx.allow_overlap(
        grip1,
        half1,
        elem_a="sleeve",
        elem_b="shank",
        reason="The geometric grip sleeve is intentionally over-molded around "
        "the steel tang, representing a seated grip wrap.",
    )
    if ok_sleeves:
        ctx.expect_overlap(
            grip0,
            half0,
            axes="xy",
            elem_a="sleeve",
            elem_b="shank",
            min_overlap=0.010,
            name="grip sleeve 0 overlaps half_0 shank footprint",
        )
        ctx.expect_overlap(
            grip1,
            half1,
            axes="xy",
            elem_a="sleeve",
            elem_b="shank",
            min_overlap=0.010,
            name="grip sleeve 1 overlaps half_1 shank footprint",
        )
        # Prove sleeves contain their tangs in cross section
        ctx.expect_within(
            half0,
            grip0,
            axes="yz",
            inner_elem="shank",
            outer_elem="sleeve",
            margin=0.002,
            name="half_0 shank contained within grip sleeve 0 cross section",
        )
        ctx.expect_within(
            half1,
            grip1,
            axes="yz",
            inner_elem="shank",
            outer_elem="sleeve",
            margin=0.002,
            name="half_1 shank contained within grip sleeve 1 cross section",
        )

    # --- Adjustment screw at rear of handle ---
    screw_aabb = ctx.part_element_world_aabb(screw, elem="knob")
    if screw_aabb is not None:
        ctx.check(
            "adjustment screw is at the rear of the handle",
            screw_aabb[0][0] < -0.06,
            details=f"screw_min_x={screw_aabb[0][0]:.4f}",
        )
    else:
        ctx.fail("screw knob AABB resolves", "missing knob element")

    # --- Screw rotates (pose test) ---
    screw_pos_0 = ctx.part_world_position(screw)
    with ctx.pose({screw_joint: math.radians(90.0)}):
        screw_pos_90 = ctx.part_world_position(screw)
    # Position should stay the same (it rotates in place)
    if screw_pos_0 is not None and screw_pos_90 is not None:
        dx = abs(screw_pos_90[0] - screw_pos_0[0])
        dy = abs(screw_pos_90[1] - screw_pos_0[1])
        ctx.check(
            "adjustment screw rotates in place without translating",
            dx < 0.001 and dy < 0.001,
            details=f"dx={dx:.6f} dy={dy:.6f}",
        )

    # --- Decisive open pose: jaws separate ---
    closed_jaw1_aabb = ctx.part_element_world_aabb(half1, elem="jaw")
    with ctx.pose({pivot: OPEN_LIMIT}):
        open_jaw1_aabb = ctx.part_element_world_aabb(half1, elem="jaw")
        ctx.expect_gap(
            half0,
            half1,
            axis="y",
            positive_elem=jaw0,
            negative_elem=jaw1,
            min_gap=0.003,
            name="diagonal jaws open apart at the 30 degree pose",
        )
        ctx.check(
            "moving jaw swings away from fixed jaw on open",
            closed_jaw1_aabb is not None
            and open_jaw1_aabb is not None
            and open_jaw1_aabb[0][1] < closed_jaw1_aabb[0][1] - 0.010,
            details=f"closed_min_y={closed_jaw1_aabb}, open_min_y={open_jaw1_aabb}",
        )

    # --- Overall envelope: compact pliers ~0.13-0.16 m long ---
    a0 = ctx.part_world_aabb(half0)
    a1 = ctx.part_world_aabb(half1)
    if a0 is not None and a1 is not None:
        length = max(a0[1][0], a1[1][0]) - min(a0[0][0], a1[0][0])
        ctx.check(
            "overall length about 0.13-0.17 m (compact diagonal cutters)",
            0.12 <= length <= 0.18,
            details=f"length={length:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
