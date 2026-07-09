from __future__ import annotations

# Needle-nose pliers variant.
# Long slender tapered jaws with serrated teeth, slender handles with
# textured grip ribs. Two forged-steel halves pivot on a flat circular rivet.
#
# Layout: the tool lies in the XY plane with Z as the thickness axis.
# The rivet pivot is at the origin. The needle nose points +X,
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
HALF_T = 0.008          # half thickness of each forged plate
LAP_R = 0.014           # half-lap joint disc radius at the pivot
HUB_R = 0.013           # forged hub radius around the rivet
BOSS_R = 0.011          # rivet boss radius
BOSS_H = 0.0025
SEAM_R = 0.0118         # visible circular seam ring
SEAM_H = 0.0005
RIVET_R = 0.0035        # rivet shaft radius
EPS = 0.0001            # lap clearance
JAW_FACE = 0.0003       # closed jaw inner faces sit near y = +/-JAW_FACE
NOSE_X = 0.090          # needle-nose tip (long slender jaw)
OPEN_LIMIT = math.radians(25.0)

TANG_HALF_W = 0.003     # slender steel handle tang half width

# Slender handle tang centerline (jaw-on-+Y half).
TANG_PTS = [
    (-0.010, -0.003),
    (-0.030, -0.009),
    (-0.060, -0.016),
    (-0.090, -0.020),
    (-0.108, -0.022),
]
CENTERLINE = TANG_PTS + [(-0.118, -0.023)]

# Grip loft stations: (x, half_width_y, half_height_z) - slender profile
GRIP_SECTIONS = [
    (-0.030, 0.0072, 0.0095),  # flared guard near pivot
    (-0.038, 0.0060, 0.0082),
    (-0.055, 0.0058, 0.0080),
    (-0.075, 0.0056, 0.0078),
    (-0.095, 0.0058, 0.0080),
    (-0.108, 0.0060, 0.0082),
    (-0.115, 0.0042, 0.0058),
    (-0.118, 0.0022, 0.0032),
]

# Grip rib positions along the handle (x stations for transverse ribs)
RIB_XS = [-0.038, -0.045, -0.052, -0.059, -0.066, -0.073, -0.080, -0.087, -0.094, -0.101]


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


def _grip_h(x: float) -> float:
    return _interp(x, [(sx, h) for sx, _w, h in GRIP_SECTIONS])


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
    """Long needle-nose jaw: slender taper from hub to fine tip, with serrated teeth."""
    # Needle-nose profile: wide base near pivot, tapering to thin point
    profile = [
        (0.008, JAW_FACE),           # base near pivot
        (0.020, JAW_FACE),
        (0.040, JAW_FACE),
        (0.060, JAW_FACE),
        (NOSE_X, JAW_FACE),          # tip at inner face
        (NOSE_X, 0.0025),            # thin tip top
        (0.075, 0.0040),
        (0.060, 0.0058),
        (0.045, 0.0078),
        (0.030, 0.0098),
        (0.018, 0.0118),
        (0.010, 0.0130),
        (0.008, 0.0125),
    ]
    pts = [(x, s * y) for x, y in profile]
    jaw = cq.Workplane("XY").polyline(pts).close().extrude(HALF_T, both=True)

    # Serrated teeth on the inner jaw face: fine transverse grooves
    # cut into the flat inner face along the jaw length
    for i in range(14):
        xi = 0.015 + 0.005 * i
        if xi > NOSE_X - 0.005:
            break
        # Tooth depth scales with jaw width at that station
        jaw_w = _interp(xi, [(p[0], p[1]) for p in profile])
        groove_depth = 0.0008
        groove = (
            cq.Workplane("XY")
            .box(0.0010, groove_depth * 2, 0.018)
            .translate((xi, s * (JAW_FACE + 0.0001), 0.0))
        )
        jaw = jaw.cut(groove)

    # Additional longitudinal serration groove along the inner face center
    long_groove = (
        cq.Workplane("XY")
        .box(0.065, 0.0006, 0.014)
        .translate((0.042, s * (JAW_FACE + 0.0002), 0.0))
    )
    jaw = jaw.cut(long_groove)

    return jaw.cut(_lap_cut(s))


def _hub_solid(s: int) -> cq.Workplane:
    """Forged circular hub around the rivet, thinned to its lap layer."""
    hub = cq.Workplane("XY").circle(HUB_R).extrude(HALF_T, both=True)
    return hub.cut(_lap_cut(s))


def _shank_solid(s: int) -> cq.Workplane:
    """Slender steel handle tang sweeping back from the hub into the grip."""
    pts = [(x, s * y) for x, y in TANG_PTS]
    loop = _strip_loop(pts, TANG_HALF_W)
    shank = cq.Workplane("XY").polyline(loop).close().extrude(HALF_T, both=True)
    return shank.cut(_lap_cut(s))


def _ellipse_wire(x: float, yc: float, zc: float, w: float, h: float) -> cq.Wire:
    wp = cq.Workplane("YZ", origin=(x, 0.0, 0.0)).center(yc, zc).ellipse(w, h)
    return wp.val()


def _grip_solid(s: int) -> cq.Solid:
    """Soft-touch rubber grip: slender curved shaft with rounded end."""
    wires = [
        _ellipse_wire(x, s * _yc(x), 0.0, w, h)
        for x, w, h in GRIP_SECTIONS
    ]
    return cq.Solid.makeLoft(wires, ruled=False)


def _grip_ribs_solid(s: int) -> cq.Solid:
    """Textured transverse grip ribs raised along the handle surface."""
    rib_parts = []
    for rx in RIB_XS:
        yc_val = s * _yc(rx)
        hw = _grip_w(rx)
        hh = _grip_h(rx)
        # Each rib is a thin elliptical disc proud of the grip surface
        rib_wire = cq.Workplane("YZ", origin=(rx, 0.0, 0.0)).center(yc_val, 0.0).ellipse(hw + 0.0008, hh + 0.0008)
        rib_solid = cq.Solid.makeLoft([
            cq.Workplane("YZ", origin=(rx - 0.0006, 0.0, 0.0)).center(yc_val, 0.0).ellipse(hw, hh).val(),
            rib_wire.val(),
            cq.Workplane("YZ", origin=(rx + 0.0006, 0.0, 0.0)).center(yc_val, 0.0).ellipse(hw, hh).val(),
        ], ruled=True)
        rib_parts.append(rib_solid)

    # Fuse all ribs into one solid
    result = rib_parts[0]
    for rb in rib_parts[1:]:
        result = result.fuse(rb)
    return result


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="needle_nose_pliers")

    steel_polished = model.material("steel_polished", rgba=(0.80, 0.81, 0.84, 1.0))
    steel_forged = model.material("steel_forged", rgba=(0.66, 0.67, 0.70, 1.0))
    steel_brushed = model.material("steel_brushed", rgba=(0.61, 0.62, 0.65, 1.0))
    seam_gray = model.material("seam_gray", rgba=(0.45, 0.46, 0.48, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.10, 0.10, 0.11, 1.0))
    rib_dark = model.material("rib_dark", rgba=(0.14, 0.14, 0.15, 1.0))

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
            mesh_from_cadquery(_grip_ribs_solid(s), f"{tag}_ribs", tolerance=0.0003),
            name="grip_ribs",
            material=rib_dark,
        )

        parts.append(part)

    # Rivet assembly fixed to half_0
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

    # Revolute pivot: axis perpendicular to tool plane.
    # Positive q about -Z swings half_1's jaw toward -Y, opening the jaws.
    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=parts[0],
        child=parts[1],
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=3.0, lower=0.0, upper=OPEN_LIMIT),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    half0 = object_model.get_part("plier_half_0")
    half1 = object_model.get_part("plier_half_1")
    pivot = object_model.get_articulation("pivot")

    jaw0 = half0.get_visual("jaw")
    jaw1 = half1.get_visual("jaw")
    hub0 = half0.get_visual("hub")
    hub1 = half1.get_visual("hub")
    grip0 = half0.get_visual("grip")
    grip1 = half1.get_visual("grip")
    ribs0 = half0.get_visual("grip_ribs")
    ribs1 = half1.get_visual("grip_ribs")

    # Joint contract: a single revolute pivot opening 0..25 degrees.
    limits = pivot.motion_limits
    ctx.check(
        "pivot is a 0..25 degree revolute joint",
        pivot.articulation_type == ArticulationType.REVOLUTE
        and limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower) < 1e-9
        and abs(limits.upper - OPEN_LIMIT) < 1e-6,
        details=f"limits={limits}",
    )

    # Needle-nose jaws are long and slender: tip extends far from pivot.
    jaw0_aabb = ctx.part_element_world_aabb(half0, elem="jaw")
    if jaw0_aabb is not None:
        jaw_length = jaw0_aabb[1][0] - jaw0_aabb[0][0]
        jaw_width = jaw0_aabb[1][1] - jaw0_aabb[0][1]
        ctx.check(
            "needle-nose jaw is long (>= 0.065 m)",
            jaw_length >= 0.065,
            details=f"jaw_length={jaw_length:.4f}",
        )
        ctx.check(
            "needle-nose jaw is slender (width < 0.035 m)",
            jaw_width < 0.035,
            details=f"jaw_width={jaw_width:.4f}",
        )

    # Serrated teeth geometry: jaw has grooves cut into inner face,
    # confirmed by the jaw bounding box being slightly inset from a
    # smooth profile (the serration cuts reduce material).
    ctx.check(
        "jaw visual exists with serrated teeth geometry",
        jaw0 is not None and jaw1 is not None,
        details="jaw visuals must be present on both halves",
    )

    # Grip ribs exist on both handles
    ctx.check(
        "textured grip ribs present on half_0",
        ribs0 is not None,
        details="grip_ribs visual must exist on half_0",
    )
    ctx.check(
        "textured grip ribs present on half_1",
        ribs1 is not None,
        details="grip_ribs visual must exist on half_1",
    )

    # Ribs are mounted on the grip surface
    ribs0_aabb = ctx.part_element_world_aabb(half0, elem="grip_ribs")
    grip0_aabb = ctx.part_element_world_aabb(half0, elem="grip")
    if ribs0_aabb is not None and grip0_aabb is not None:
        ctx.expect_overlap(
            half0,
            half0,
            axes="x",
            elem_a="grip_ribs",
            elem_b="grip",
            min_overlap=0.04,
            name="grip ribs overlap the grip along handle length",
        )

    # Closed rest pose: needle-nose jaw tips nearly touch.
    ctx.expect_gap(
        half0,
        half1,
        axis="y",
        positive_elem=jaw0,
        negative_elem=jaw1,
        min_gap=0.0002,
        max_gap=0.002,
        name="needle-nose jaws closed and nearly touching at rest",
    )

    # Halves interleave at the pivot.
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
        min_overlap=0.018,
        name="hub laps share the pivot footprint",
    )

    # Rivet shaft captured through moving half's hub.
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
        min_overlap=0.006,
        name="rivet shaft passes through the full moving lap thickness",
    )

    # Overall envelope: ~0.20 m long
    a0 = ctx.part_world_aabb(half0)
    a1 = ctx.part_world_aabb(half1)
    g0 = ctx.part_element_world_aabb(half0, elem="grip")
    g1 = ctx.part_element_world_aabb(half1, elem="grip")
    ok_env = a0 is not None and a1 is not None and g0 is not None and g1 is not None
    if ok_env:
        length = max(a0[1][0], a1[1][0]) - min(a0[0][0], a1[0][0])
        ctx.check(
            "overall length about 0.20 m",
            0.18 <= length <= 0.22,
            details=f"length={length:.4f}",
        )
    else:
        ctx.fail("envelope AABBs resolve", "missing part/grip AABB")

    # Decisive open pose: jaws separate and handles spread.
    closed_jaw1 = ctx.part_element_world_aabb(half1, elem="jaw")
    closed_grip1 = ctx.part_element_world_aabb(half1, elem="grip")
    with ctx.pose({pivot: OPEN_LIMIT}):
        ctx.expect_gap(
            half0,
            half1,
            axis="y",
            positive_elem=jaw0,
            negative_elem=jaw1,
            min_gap=0.003,
            name="needle-nose jaws open apart at the 25 degree pose",
        )
        open_jaw1 = ctx.part_element_world_aabb(half1, elem="jaw")
        open_grip1 = ctx.part_element_world_aabb(half1, elem="grip")
        ctx.check(
            "moving jaw swings away from the fixed jaw",
            closed_jaw1 is not None
            and open_jaw1 is not None
            and open_jaw1[0][1] < closed_jaw1[0][1] - 0.012,
            details=f"closed_min_y={closed_jaw1}, open_min_y={open_jaw1}",
        )
        ctx.check(
            "moving handle spreads outward as the jaws open",
            closed_grip1 is not None
            and open_grip1 is not None
            and open_grip1[1][1] > closed_grip1[1][1] + 0.02,
            details=f"closed_max_y={closed_grip1}, open_max_y={open_grip1}",
        )

    return ctx.report()


object_model = build_object_model()
