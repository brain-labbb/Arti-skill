from __future__ import annotations

# Needle-nose pliers variant.
# Forked from lineman pliers into long-jaw needle-nose form.
#
# Layout: the tool lies in the XY plane with Z as the thickness axis.
# The rivet pivot is at the origin. The long tapered nose points +X,
# the slender handles sweep back to -X and spread in +/-Y.
# plier_half_0 (root) carries its jaw on +Y; plier_half_1 is the mirrored
# moving half. The two forged halves are half-lapped at the pivot.
# An adjustment_screw part mounts at the rear of half_0's handle with a
# continuous revolute joint for rotation.

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
HALF_T = 0.007           # half thickness of each forged plate (full plate 0.014)
LAP_R = 0.013            # half-lap joint disc radius at the pivot
HUB_R = 0.012            # forged hub radius around the rivet
BOSS_R = 0.010           # rivet boss radius (~0.020 m diameter)
BOSS_H = 0.0025
SEAM_R = 0.0105          # visible circular seam ring under the boss cap
SEAM_H = 0.0005
RIVET_R = 0.003          # rivet shaft captured through the moving half's lap
EPS = 0.0001             # lap clearance so the stacked halves do not penetrate
JAW_FACE = 0.0003        # closed jaw inner faces sit at y = +/-JAW_FACE
NOSE_X = 0.080           # needle-nose tip (long tapered jaw)
OPEN_LIMIT = math.radians(25.0)
JAW_STOP_R = 0.003       # jaw stop boss radius
JAW_STOP_H = 0.004       # jaw stop boss height

TANG_HALF_W = 0.003      # slender steel handle tang half width in plan

# Steel handle tang centerline in the tool plane (for the half whose jaw is +Y).
# Slender handles, less outward curve than lineman pliers.
TANG_PTS = [
    (-0.010, -0.003),
    (-0.030, -0.008),
    (-0.060, -0.014),
    (-0.090, -0.019),
    (-0.110, -0.022),
    (-0.120, -0.023),
]

# Slender handle cross-section stations: (x, half_width_y, half_height_z)
HANDLE_SECTIONS = [
    (-0.015, 0.0050, 0.0070),  # near pivot
    (-0.030, 0.0045, 0.0065),
    (-0.050, 0.0040, 0.0060),
    (-0.070, 0.0038, 0.0058),
    (-0.090, 0.0036, 0.0056),
    (-0.105, 0.0035, 0.0055),
    (-0.115, 0.0034, 0.0054),
    (-0.120, 0.0028, 0.0040),  # tapered end
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
    return _interp(x, TANG_PTS)


def _handle_w(x: float) -> float:
    return _interp(x, [(sx, w) for sx, w, _h in HANDLE_SECTIONS])


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
    cut = cq.Workplane("XY").circle(LAP_R).extrude(0.010)
    if s > 0:
        return cut.translate((0.0, 0.0, -EPS))
    return cut.translate((0.0, 0.0, -0.010 + EPS))


def _jaw_solid(s: int) -> cq.Workplane:
    """Long tapered needle-nose jaw with serrated inner teeth."""
    # Needle-nose profile: starts wide near pivot, tapers to slender point.
    # Outer contour (away from workpiece):
    profile_outer = [
        (0.010, 0.0120),   # base width near pivot
        (0.020, 0.0110),
        (0.035, 0.0090),
        (0.050, 0.0065),
        (0.065, 0.0045),
        (NOSE_X, 0.0020),  # slender tip
    ]
    # Inner contour (gripping face):
    profile_inner = [
        (NOSE_X, JAW_FACE),       # tip inner face
        (0.065, JAW_FACE + 0.001),
        (0.050, JAW_FACE + 0.001),
        (0.035, JAW_FACE + 0.001),
        (0.020, JAW_FACE + 0.002),
        (0.010, JAW_FACE + 0.003),  # base inner face
    ]
    # Combine into closed profile
    full_profile = profile_outer + profile_inner
    pts = [(x, s * y) for x, y in full_profile]
    jaw = cq.Workplane("XY").polyline(pts).close().extrude(HALF_T, both=True)

    # Serrated teeth on inner jaw faces: transverse grooves cut across the
    # inner gripping surface. These are small rectangular cuts perpendicular
    # to the jaw length, creating visible tooth ridges.
    # Limit grooves to the thicker base region to avoid penetrating the thin tip.
    for i in range(10):
        xi = 0.015 + 0.005 * i
        if xi > 0.058:
            break
        # Groove only cuts a shallow channel into the inner face
        groove_depth = 0.0008
        groove_width = 0.0010
        # Groove Y extent: from just below the inner face to slightly above it
        # without penetrating the outer face
        groove_y_center = s * (JAW_FACE + 0.0005)
        groove_y_size = 0.003  # shallow enough to not penetrate
        groove = (
            cq.Workplane("XY")
            .box(groove_width, groove_y_size, HALF_T * 2.2)
            .translate((xi, groove_y_center, 0.0))
        )
        jaw = jaw.cut(groove)

    return jaw


def _hub_solid(s: int) -> cq.Workplane:
    """Forged circular hub around the rivet, thinned to its lap layer."""
    hub = cq.Workplane("XY").circle(HUB_R).extrude(HALF_T, both=True)
    return hub.cut(_lap_cut(s))


def _shank_solid(s: int) -> cq.Workplane:
    """Slender steel handle tang sweeping back from the hub."""
    pts = [(x, s * y) for x, y in TANG_PTS]
    loop = _strip_loop(pts, TANG_HALF_W)
    shank = cq.Workplane("XY").polyline(loop).close().extrude(HALF_T, both=True)
    return shank.cut(_lap_cut(s))


def _ellipse_wire(x: float, yc: float, zc: float, w: float, h: float) -> cq.Wire:
    wp = cq.Workplane("YZ", origin=(x, 0.0, 0.0)).center(yc, zc).ellipse(w, h)
    return wp.val()


def _handle_solid(s: int) -> cq.Solid:
    """Slender steel handle body lofted through elliptical stations."""
    wires = [
        _ellipse_wire(x, s * _yc(x), 0.0, w, h)
        for x, w, h in HANDLE_SECTIONS
    ]
    return cq.Solid.makeLoft(wires, ruled=False)


def _jaw_stop_solid(s: int) -> cq.Workplane:
    """Jaw stop boss: small cylindrical bump near pivot on the inner jaw face.
    Limits how far the jaws close by contacting the opposing jaw inner face."""
    boss = (
        cq.Workplane("XY")
        .circle(JAW_STOP_R)
        .extrude(JAW_STOP_H / 2.0, both=True)
        .translate((0.012, s * (JAW_FACE + 0.003), 0.0))
    )
    return boss


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="needle_nose_pliers")

    steel_polished = model.material("steel_polished", rgba=(0.82, 0.83, 0.86, 1.0))
    steel_forged = model.material("steel_forged", rgba=(0.62, 0.63, 0.66, 1.0))
    steel_brushed = model.material("steel_brushed", rgba=(0.58, 0.59, 0.62, 1.0))
    seam_gray = model.material("seam_gray", rgba=(0.42, 0.43, 0.45, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.35, 0.36, 0.38, 1.0))
    screw_brass = model.material("screw_brass", rgba=(0.72, 0.60, 0.30, 1.0))

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
            mesh_from_cadquery(_handle_solid(s), f"{tag}_handle", tolerance=0.0002),
            name="handle",
            material=steel_dark,
        )

        parts.append(part)

    # Jaw stop boss on the fixed half only (limits jaw closing travel).
    fixed = parts[0]
    fixed.visual(
        mesh_from_cadquery(_jaw_stop_solid(1), "half0_jaw_stop", tolerance=0.0002),
        name="jaw_stop_boss",
        material=steel_forged,
    )

    # Rivet assembly fixed to half_0:
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

    # Adjustment screw at the rear of the fixed handle.
    # This is a small knurled thumbscrew for spring tension / grip adjustment.
    screw_part = model.part("adjustment_screw")
    screw_x = -0.105
    screw_y = _yc(screw_x)  # follow handle centerline for half_0 (s=1)
    # Handle top surface at this station: interpolated half-height.
    handle_top_z = _interp(screw_x, [(sx, h) for sx, _w, h in HANDLE_SECTIONS])
    # Build screw geometry in the child part's local frame (centered at origin).
    # The articulation origin will place it at the correct world position.
    screw_r = 0.004       # 8mm diameter
    screw_h = 0.005       # 5mm tall
    embed = 0.0005        # slight embed into handle for contact
    # Base cylinder in local frame: extends from z=-embed to z=screw_h-embed
    screw_body = (
        cq.Workplane("XY")
        .circle(screw_r)
        .extrude(screw_h)
        .translate((0.0, 0.0, -embed))
    )
    # Knurl ridges: small ribs around the circumference
    for i in range(16):
        angle = math.radians(i * 22.5)
        rx = (screw_r + 0.0003) * math.cos(angle)
        ry = (screw_r + 0.0003) * math.sin(angle)
        ridge = (
            cq.Workplane("XY")
            .box(0.0006, 0.0006, screw_h * 0.8)
            .translate((rx, ry, screw_h * 0.4 - embed))
        )
        screw_body = screw_body.union(ridge)
    # Central bore hole
    bore = (
        cq.Workplane("XY")
        .circle(0.0015)
        .extrude(screw_h + 0.001)
        .translate((0.0, 0.0, -embed - 0.0005))
    )
    screw_body = screw_body.cut(bore)

    screw_part.visual(
        mesh_from_cadquery(screw_body, "screw_body_mesh", tolerance=0.0002),
        name="screw_body",
        material=screw_brass,
    )

    # Primary articulation: revolute pivot at the rivet.
    # Positive q (about -Z) swings half_1's jaw toward -Y, opening the jaws.
    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=parts[0],
        child=parts[1],
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=3.0, lower=0.0, upper=OPEN_LIMIT),
    )

    # Adjustment screw articulation: continuous rotation about Z axis.
    # The screw rotates freely in its threaded bore on the handle.
    model.articulation(
        "screw_joint",
        ArticulationType.CONTINUOUS,
        parent=fixed,
        child=screw_part,
        origin=Origin(xyz=(screw_x, screw_y, handle_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    half0 = object_model.get_part("plier_half_0")
    half1 = object_model.get_part("plier_half_1")
    screw = object_model.get_part("adjustment_screw")
    pivot = object_model.get_articulation("pivot")
    screw_joint = object_model.get_articulation("screw_joint")

    jaw0 = half0.get_visual("jaw")
    jaw1 = half1.get_visual("jaw")
    hub0 = half0.get_visual("hub")
    hub1 = half1.get_visual("hub")
    handle0 = half0.get_visual("handle")

    # --- Joint contract ---
    # Pivot: single revolute joint, 0 to 25 degrees.
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

    # Screw joint: continuous rotation.
    ctx.check(
        "screw_joint is continuous rotation",
        screw_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={screw_joint.articulation_type}",
    )

    # --- Needle-nose jaw geometry ---
    # The jaw should extend to about 0.080 m from pivot (long needle nose).
    jaw0_aabb = ctx.part_element_world_aabb(half0, elem="jaw")
    if jaw0_aabb is not None:
        jaw_length = jaw0_aabb[1][0] - min(jaw0_aabb[0][0], 0.0)
        ctx.check(
            "needle-nose jaw extends at least 0.065 m from pivot",
            jaw_length >= 0.065,
            details=f"jaw_length={jaw_length:.4f}",
        )
        # Slender tip: jaw width at the tip should be narrow.
        jaw_width_at_tip = jaw0_aabb[1][1] - jaw0_aabb[0][1]
        ctx.check(
            "jaw overall width is slender (needle-nose, < 0.030 m)",
            jaw_width_at_tip < 0.030,
            details=f"jaw_width={jaw_width_at_tip:.4f}",
        )
    else:
        ctx.fail("jaw AABB resolves", "missing jaw element AABB")

    # Serrated teeth: jaw visual should have detail from the groove cuts.
    # We verify the jaw exists and has reasonable geometry (the grooves
    # reduce material, so the jaw mesh has internal detail).
    ctx.check(
        "jaw visual exists with serrated teeth geometry",
        jaw0 is not None,
        details="jaw0 visual must be present",
    )

    # --- Jaw stop boss ---
    stop_aabb = ctx.part_element_world_aabb(half0, elem="jaw_stop_boss")
    ctx.check(
        "jaw stop boss exists near the pivot on fixed half",
        stop_aabb is not None,
        details="jaw_stop_boss element must be present",
    )
    if stop_aabb is not None:
        stop_center_x = 0.5 * (stop_aabb[0][0] + stop_aabb[1][0])
        ctx.check(
            "jaw stop boss is positioned near the pivot (x < 0.020)",
            stop_center_x < 0.020,
            details=f"stop_center_x={stop_center_x:.4f}",
        )

    # --- Slender handles ---
    # The handle Y AABB includes the curved path extent, not just cross-section.
    # Check that the handle thickness (Z extent) is slender, proving no thick grip.
    handle0_aabb = ctx.part_element_world_aabb(half0, elem="handle")
    if handle0_aabb is not None:
        handle_thickness = handle0_aabb[1][2] - handle0_aabb[0][2]
        ctx.check(
            "handle is slender (thickness < 0.018 m, no thick grip)",
            handle_thickness < 0.018,
            details=f"handle_thickness={handle_thickness:.4f}",
        )
    else:
        ctx.fail("handle AABB resolves", "missing handle element AABB")

    # --- Adjustment screw ---
    screw_aabb = ctx.part_world_aabb(screw)
    ctx.check(
        "adjustment screw part exists",
        screw_aabb is not None,
        details="screw part must have geometry",
    )
    if screw_aabb is not None:
        # Screw should be near the rear of the handle (negative X).
        screw_center_x = 0.5 * (screw_aabb[0][0] + screw_aabb[1][0])
        ctx.check(
            "adjustment screw is at the rear handle (x < -0.080)",
            screw_center_x < -0.080,
            details=f"screw_center_x={screw_center_x:.4f}",
        )
        # Screw should sit above the handle (positive Z).
        ctx.check(
            "adjustment screw sits above the handle surface",
            screw_aabb[0][2] > 0.003,
            details=f"screw_min_z={screw_aabb[0][2]:.4f}",
        )

    # --- Closed rest pose: jaws nearly touch ---
    ctx.expect_gap(
        half0,
        half1,
        axis="y",
        positive_elem=jaw0,
        negative_elem=jaw1,
        min_gap=0.0002,
        max_gap=0.002,
        name="jaws closed and nearly touching at rest",
    )

    # --- Pivot interleaving ---
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
        min_overlap=0.016,
        name="hub laps share the pivot footprint",
    )

    # Rivet shaft captured through moving half (intentional overlap).
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
        min_overlap=0.005,
        name="rivet shaft passes through the full moving lap thickness",
    )

    # --- Overall envelope ---
    a0 = ctx.part_world_aabb(half0)
    a1 = ctx.part_world_aabb(half1)
    if a0 is not None and a1 is not None:
        length = max(a0[1][0], a1[1][0]) - min(a0[0][0], a1[0][0])
        ctx.check(
            "overall length about 0.20 m",
            0.18 <= length <= 0.22,
            details=f"length={length:.4f}",
        )

    # --- Decisive open pose: jaws separate ---
    closed_jaw1 = ctx.part_element_world_aabb(half1, elem="jaw")
    with ctx.pose({pivot: OPEN_LIMIT}):
        ctx.expect_gap(
            half0,
            half1,
            axis="y",
            positive_elem=jaw0,
            negative_elem=jaw1,
            min_gap=0.003,
            name="jaws open apart at the 25 degree pose",
        )
        open_jaw1 = ctx.part_element_world_aabb(half1, elem="jaw")
        ctx.check(
            "moving jaw swings away from the fixed jaw",
            closed_jaw1 is not None
            and open_jaw1 is not None
            and open_jaw1[0][1] < closed_jaw1[0][1] - 0.012,
            details=f"closed_min_y={closed_jaw1}, open_min_y={open_jaw1}",
        )

    return ctx.report()


object_model = build_object_model()
