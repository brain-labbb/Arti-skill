from __future__ import annotations

# Insulated lineman pliers with return spring — Variant 10.
# Fork of heavy-duty combination (lineman) pliers.
# Reference: picture/Other/pliers/002.png family variant.
#
# Layout: the tool lies in the XY plane with Z as the thickness axis.
# The rivet pivot is at the origin. The blunt squared nose points +X,
# the handles sweep back to -X and spread in +/-Y.
# plier_half_0 (root) carries its jaw on +Y; plier_half_1 is the mirrored
# moving half. The two forged halves are half-lapped at the pivot.
#
# Variant changes vs parent:
#   - Thick two-layer insulated grip sleeves (VDE yellow inner + dark red outer)
#   - Torsion return spring at the pivot with legs pressing against the handles
#   - Prominent circular rivet caps on both sides
#   - Color-separated geometric outer grip sleeves with finger-groove bands

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
BOSS_R = 0.0135         # rivet cap radius (~0.027 m diameter, prominent cap)
BOSS_H = 0.0025
SEAM_R = 0.0142         # visible rim ring under the cap
SEAM_H = 0.0008
RIVET_R = 0.004         # rivet shaft captured through the moving half's lap
EPS = 0.0001            # lap clearance
JAW_FACE = 0.0003
NOSE_X = 0.075
OPEN_LIMIT = math.radians(30.0)

TANG_HALF_W = 0.004

# Steel handle tang centerline in the tool plane (for the half whose jaw is +Y).
TANG_PTS = [
    (-0.010, -0.004),
    (-0.030, -0.011),
    (-0.070, -0.019),
    (-0.100, -0.024),
    (-0.118, -0.026),
]
CENTERLINE = TANG_PTS + [(-0.131, -0.0274)]

# Inner insulation loft stations: (x, half_width_y, half_height_z)
# Thicker than the parent grip for VDE-rated insulation.
INSULATION_SECTIONS = [
    (-0.030, 0.0110, 0.0140),  # flared guard near pivot
    (-0.040, 0.0100, 0.0130),
    (-0.060, 0.0098, 0.0128),
    (-0.085, 0.0100, 0.0132),
    (-0.105, 0.0106, 0.0138),
    (-0.120, 0.0108, 0.0140),
    (-0.128, 0.0082, 0.0106),
    (-0.132, 0.0048, 0.0060),
]

# Outer grip sleeve sections: slightly larger, shorter span so the
# yellow insulation peeks out at both ends.
OUTER_GRIP_SECTIONS = [
    (-0.038, 0.0118, 0.0152),
    (-0.050, 0.0108, 0.0140),
    (-0.065, 0.0106, 0.0138),
    (-0.080, 0.0108, 0.0140),
    (-0.095, 0.0112, 0.0146),
    (-0.108, 0.0116, 0.0150),
    (-0.115, 0.0092, 0.0120),
]

# Finger-groove band positions along the outer grip (x stations).
FINGER_GROOVE_XS = [-0.052, -0.066, -0.080, -0.094, -0.106]

# Spring geometry
SPRING_COIL_R = 0.0075
SPRING_WIRE_R = 0.0010
SPRING_TURNS = 3
SPRING_Z_HALF = 0.005


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


def _insul_w(x: float) -> float:
    return _interp(x, [(sx, w) for sx, w, _h in INSULATION_SECTIONS])


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

    for i in range(7):
        xi = 0.046 + 0.004 * i
        groove = (
            cq.Workplane("XY")
            .box(0.0014, 0.0024, 0.020)
            .translate((xi, s * JAW_FACE, 0.0))
        )
        jaw = jaw.cut(groove)

    recess = (
        cq.Workplane("XY")
        .circle(0.0045)
        .extrude(0.011, both=True)
        .translate((0.030, s * JAW_FACE, 0.0))
    )
    jaw = jaw.cut(recess)

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


def _insulation_solid(s: int) -> cq.Solid:
    """Inner VDE insulation sleeve: thick, smooth elliptical loft, yellow."""
    wires = [
        _ellipse_wire(x, s * _yc(x), 0.0, w, h)
        for x, w, h in INSULATION_SECTIONS
    ]
    return cq.Solid.makeLoft(wires, ruled=False)


def _outer_grip_solid(s: int) -> cq.Workplane:
    """Outer grip sleeve: geometric, color-separated, with finger-groove bands."""
    wires = [
        _ellipse_wire(x, s * _yc(x), 0.0, w, h)
        for x, w, h in OUTER_GRIP_SECTIONS
    ]
    grip_solid = cq.Solid.makeLoft(wires, ruled=False)
    result = cq.Workplane("XY").newObject([cq.Compound.makeCompound([grip_solid])])

    # Cut finger-groove bands: thin horizontal slots across top and bottom faces
    for x_pos in FINGER_GROOVE_XS:
        yc_val = s * _yc(x_pos)
        w_val = _interp(x_pos, [(sx, w) for sx, w, _h in OUTER_GRIP_SECTIONS])
        h_val = _interp(x_pos, [(sx, h) for sx, _w, h in OUTER_GRIP_SECTIONS])
        # Top groove
        groove_top = (
            cq.Workplane("XY")
            .box(0.0025, w_val * 2.4, 0.002)
            .translate((x_pos, yc_val, h_val - 0.0005))
        )
        result = result.cut(groove_top)
        # Bottom groove
        groove_bot = (
            cq.Workplane("XY")
            .box(0.0025, w_val * 2.4, 0.002)
            .translate((x_pos, yc_val, -(h_val - 0.0005)))
        )
        result = result.cut(groove_bot)

    return result


def _spring_coil_cq() -> cq.Workplane:
    """Torsion spring coil built with CadQuery sweep along a helical path."""
    start_angle = math.pi

    # Generate helix polyline
    N = 150
    helix_3d: list[tuple[float, float, float]] = []
    for i in range(N + 1):
        t = i / N
        angle = start_angle + t * SPRING_TURNS * 2.0 * math.pi
        z = -SPRING_Z_HALF + t * 2.0 * SPRING_Z_HALF
        x = SPRING_COIL_R * math.cos(angle)
        y = SPRING_COIL_R * math.sin(angle)
        helix_3d.append((x, y, z))

    # Build 3D path wire from line edges
    edges = []
    for i in range(len(helix_3d) - 1):
        p1 = helix_3d[i]
        p2 = helix_3d[i + 1]
        edges.append(cq.Edge.makeLine(cq.Vector(*p1), cq.Vector(*p2)))
    path_wire = cq.Wire.assembleEdges(edges)

    # Compute path tangent at start for profile orientation
    p0 = cq.Vector(*helix_3d[0])
    p1 = cq.Vector(*helix_3d[1])
    normal = (p1 - p0).normalized()

    # Build a plane perpendicular to the tangent at the helix start
    up = cq.Vector(0, 0, 1)
    x_dir = up.cross(normal).normalized()
    if x_dir.Length < 0.01:
        up = cq.Vector(1, 0, 0)
        x_dir = up.cross(normal).normalized()
    plane = cq.Plane(p0, x_dir, normal)

    # Draw the circle profile on this oriented workplane (creates a pending wire)
    profile = cq.Workplane(plane).circle(SPRING_WIRE_R)

    # Path workplane
    path_wp = cq.Workplane("XY").newObject([path_wire])

    # Sweep profile along path
    return profile.sweep(path_wp, isFrenet=True)


def _spring_leg_solid(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> cq.Workplane:
    """Build a spring leg as a thin cylinder between two 3D points."""
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    mid = (0.5 * (start[0] + end[0]), 0.5 * (start[1] + end[1]), 0.5 * (start[2] + end[2]))

    # Build a cylinder along Z, then rotate to align with the direction
    cyl = cq.Workplane("XY").circle(SPRING_WIRE_R).extrude(length)
    # Center it
    cyl = cyl.translate((0, 0, -length / 2.0))

    # Compute rotation from Z to (dx, dy, dz)
    direction = cq.Vector(dx, dy, dz).normalized()
    z_axis = cq.Vector(0, 0, 1)
    if abs(direction.dot(z_axis)) > 0.9999:
        # Nearly aligned with Z, no rotation needed (or 180 flip)
        if direction.z < 0:
            cyl = cyl.rotate((0, 0, 0), (1, 0, 0), 180.0)
    else:
        rot_axis = z_axis.cross(direction).normalized()
        rot_angle = math.degrees(math.acos(max(-1.0, min(1.0, z_axis.dot(direction)))))
        cyl = cyl.rotate((0, 0, 0), (rot_axis.x, rot_axis.y, rot_axis.z), rot_angle)

    cyl = cyl.translate(mid)
    return cyl


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="insulated_pliers_spring")

    steel_polished = model.material("steel_polished", rgba=(0.80, 0.81, 0.84, 1.0))
    steel_forged = model.material("steel_forged", rgba=(0.66, 0.67, 0.70, 1.0))
    steel_brushed = model.material("steel_brushed", rgba=(0.61, 0.62, 0.65, 1.0))
    seam_gray = model.material("seam_gray", rgba=(0.45, 0.46, 0.48, 1.0))
    insulation_yellow = model.material("insulation_yellow", rgba=(0.96, 0.82, 0.10, 1.0))
    grip_dark_red = model.material("grip_dark_red", rgba=(0.68, 0.07, 0.10, 1.0))
    spring_steel = model.material("spring_steel", rgba=(0.52, 0.53, 0.56, 1.0))
    cap_chrome = model.material("cap_chrome", rgba=(0.75, 0.76, 0.78, 1.0))

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
        # Inner insulation sleeve (VDE yellow)
        part.visual(
            mesh_from_cadquery(_insulation_solid(s), f"{tag}_insulation", tolerance=0.0002),
            name="insulation_sleeve",
            material=insulation_yellow,
        )
        # Outer grip sleeve (dark red, geometric with finger grooves)
        part.visual(
            mesh_from_cadquery(_outer_grip_solid(s), f"{tag}_outer_grip", tolerance=0.0002),
            name="outer_grip_sleeve",
            material=grip_dark_red,
        )

        parts.append(part)

    # ---- Prominent circular rivet caps on both sides ----
    fixed = parts[0]

    # Bottom cap (below half_0)
    fixed.visual(
        Cylinder(SEAM_R, SEAM_H),
        origin=Origin(xyz=(0.0, 0.0, -(HALF_T + SEAM_H / 2.0 - EPS))),
        name="boss_rim",
        material=seam_gray,
    )
    fixed.visual(
        Cylinder(BOSS_R, BOSS_H),
        origin=Origin(xyz=(0.0, 0.0, -(HALF_T + SEAM_H + BOSS_H / 2.0 - 2.0 * EPS))),
        name="rivet_cap_bottom",
        material=cap_chrome,
    )
    # Rivet shaft through both halves
    fixed.visual(
        Cylinder(RIVET_R, 2.0 * HALF_T + SEAM_H * 2.0 + BOSS_H * 2.0 + 2.0 * EPS + 0.0005),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="rivet_shaft",
        material=steel_brushed,
    )
    # Top cap (above half_1)
    fixed.visual(
        Cylinder(SEAM_R, SEAM_H),
        origin=Origin(xyz=(0.0, 0.0, HALF_T + SEAM_H / 2.0 + EPS)),
        name="head_rim",
        material=seam_gray,
    )
    fixed.visual(
        Cylinder(BOSS_R, BOSS_H),
        origin=Origin(xyz=(0.0, 0.0, HALF_T + SEAM_H + BOSS_H / 2.0 + 2.0 * EPS)),
        name="rivet_cap_top",
        material=cap_chrome,
    )

    # ---- Torsion return spring (attached to fixed half) ----
    # Coil
    spring_coil = _spring_coil_cq()
    fixed.visual(
        mesh_from_cadquery(spring_coil, "spring_coil", tolerance=0.0003),
        name="return_spring",
        material=spring_steel,
    )
    # Leg 1: from coil start toward half_0's handle (-Y side)
    start_angle = math.pi
    coil_start = (
        SPRING_COIL_R * math.cos(start_angle),
        SPRING_COIL_R * math.sin(start_angle),
        -SPRING_Z_HALF,
    )
    leg1_end = (-0.035, -0.011, 0.0)
    leg1 = _spring_leg_solid(coil_start, leg1_end)
    fixed.visual(
        mesh_from_cadquery(leg1, "spring_leg_0", tolerance=0.0003),
        name="spring_leg_0",
        material=spring_steel,
    )
    # Leg 2: from coil end toward half_1's handle (+Y side)
    end_angle = start_angle + SPRING_TURNS * 2.0 * math.pi
    coil_end = (
        SPRING_COIL_R * math.cos(end_angle),
        SPRING_COIL_R * math.sin(end_angle),
        SPRING_Z_HALF,
    )
    leg2_end = (-0.035, 0.011, 0.0)
    leg2 = _spring_leg_solid(coil_end, leg2_end)
    fixed.visual(
        mesh_from_cadquery(leg2, "spring_leg_1", tolerance=0.0003),
        name="spring_leg_1",
        material=spring_steel,
    )

    # ---- Primary articulation: revolute pivot ----
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
    insul0 = half0.get_visual("insulation_sleeve")
    insul1 = half1.get_visual("insulation_sleeve")
    outer0 = half0.get_visual("outer_grip_sleeve")
    outer1 = half1.get_visual("outer_grip_sleeve")
    spring = half0.get_visual("return_spring")

    # ---- Joint contract: revolute 0..30 degrees ----
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

    # ---- Insulation sleeves exist on both halves ----
    insul0_aabb = ctx.part_element_world_aabb(half0, elem="insulation_sleeve")
    insul1_aabb = ctx.part_element_world_aabb(half1, elem="insulation_sleeve")
    ctx.check(
        "insulation sleeve present on fixed half",
        insul0_aabb is not None,
        details="missing insulation_sleeve AABB on half_0",
    )
    ctx.check(
        "insulation sleeve present on moving half",
        insul1_aabb is not None,
        details="missing insulation_sleeve AABB on half_1",
    )

    # ---- Outer grip sleeves are geometrically distinct from insulation ----
    outer0_aabb = ctx.part_element_world_aabb(half0, elem="outer_grip_sleeve")
    outer1_aabb = ctx.part_element_world_aabb(half1, elem="outer_grip_sleeve")
    ctx.check(
        "outer grip sleeve present on fixed half",
        outer0_aabb is not None,
        details="missing outer_grip_sleeve AABB on half_0",
    )
    ctx.check(
        "outer grip sleeve present on moving half",
        outer1_aabb is not None,
        details="missing outer_grip_sleeve AABB on half_1",
    )

    # Outer grip sits within the insulation span (insulation peeks out at ends)
    if insul0_aabb is not None and outer0_aabb is not None:
        ctx.check(
            "outer grip sleeve shorter than insulation on fixed half",
            outer0_aabb[0][0] > insul0_aabb[0][0] + 0.003
            and outer0_aabb[1][0] < insul0_aabb[1][0] - 0.003,
            details=f"insul_x=({insul0_aabb[0][0]:.4f},{insul0_aabb[1][0]:.4f}) "
                    f"outer_x=({outer0_aabb[0][0]:.4f},{outer0_aabb[1][0]:.4f})",
        )

    # Outer grip overlaps insulation in XY (wraps around it)
    if insul0_aabb is not None and outer0_aabb is not None:
        ctx.expect_overlap(
            half0,
            half0,
            axes="xy",
            elem_a="outer_grip_sleeve",
            elem_b="insulation_sleeve",
            min_overlap=0.015,
            name="outer grip wraps around the insulation sleeve",
        )

    # ---- Return spring exists at the pivot ----
    spring_aabb = ctx.part_element_world_aabb(half0, elem="return_spring")
    leg0_aabb = ctx.part_element_world_aabb(half0, elem="spring_leg_0")
    leg1_aabb = ctx.part_element_world_aabb(half0, elem="spring_leg_1")
    ctx.check(
        "return spring present on fixed half",
        spring_aabb is not None,
        details="missing return_spring AABB",
    )
    if spring_aabb is not None:
        # Spring coil should be centered near the pivot origin
        spring_cx = 0.5 * (spring_aabb[0][0] + spring_aabb[1][0])
        spring_cy = 0.5 * (spring_aabb[0][1] + spring_aabb[1][1])
        ctx.check(
            "return spring coil centered near the pivot",
            abs(spring_cx) < 0.015 and abs(spring_cy) < 0.015,
            details=f"spring_center=({spring_cx:.4f},{spring_cy:.4f})",
        )
    # Spring legs extend into the handle area
    ctx.check(
        "spring leg 0 reaches the handle zone",
        leg0_aabb is not None and leg0_aabb[0][0] < -0.020,
        details=f"leg0_min_x={leg0_aabb[0][0] if leg0_aabb else None}",
    )
    ctx.check(
        "spring leg 1 reaches the handle zone",
        leg1_aabb is not None and leg1_aabb[0][0] < -0.020,
        details=f"leg1_min_x={leg1_aabb[0][0] if leg1_aabb else None}",
    )

    # ---- Rivet caps on both sides ----
    cap_bot = ctx.part_element_world_aabb(half0, elem="rivet_cap_bottom")
    cap_top = ctx.part_element_world_aabb(half0, elem="rivet_cap_top")
    ctx.check(
        "rivet cap on bottom side",
        cap_bot is not None,
        details="missing rivet_cap_bottom",
    )
    ctx.check(
        "rivet cap on top side",
        cap_top is not None,
        details="missing rivet_cap_top",
    )
    if cap_bot is not None and cap_top is not None:
        bot_dia = cap_bot[1][0] - cap_bot[0][0]
        top_dia = cap_top[1][0] - cap_top[0][0]
        ctx.check(
            "both rivet caps are ~27 mm diameter circular disks",
            0.024 <= bot_dia <= 0.030 and 0.024 <= top_dia <= 0.030,
            details=f"bot_dia={bot_dia:.4f} top_dia={top_dia:.4f}",
        )
        # Caps are on opposite sides of the tool
        ctx.check(
            "rivet caps on opposite Z faces of the tool",
            cap_bot[1][2] < 0.0 and cap_top[0][2] > 0.0,
            details=f"bot_max_z={cap_bot[1][2]:.4f} top_min_z={cap_top[0][2]:.4f}",
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

    # Hub lap stacking
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

    # Rivet shaft captured through moving half's hub
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

    # Return spring: the coil wraps inside the hub footprint and leg_1
    # intentionally presses against the moving half's handle shank.
    ctx.allow_overlap(
        half0,
        half1,
        elem_a="return_spring",
        elem_b=hub1,
        reason="The torsion spring coil sits around the pivot inside the hub "
               "footprint, intentionally nested within the moving hub region.",
    )
    ctx.allow_overlap(
        half0,
        half1,
        elem_a="spring_leg_1",
        elem_b="shank",
        reason="The return spring leg intentionally presses against the inner "
               "face of the moving half's handle shank to bias the handles open.",
    )
    ctx.allow_overlap(
        half0,
        half1,
        elem_a="spring_leg_1",
        elem_b="insulation_sleeve",
        reason="The return spring leg passes through the insulation sleeve "
               "gap to press against the moving handle — local contact embed.",
    )
    ctx.allow_overlap(
        half0,
        half1,
        elem_a="spring_leg_1",
        elem_b="outer_grip_sleeve",
        reason="The return spring leg passes through the outer grip sleeve "
               "region to press against the moving handle — local contact embed.",
    )
    ctx.allow_overlap(
        half0,
        half1,
        elem_a="spring_leg_1",
        elem_b="hub",
        reason="The spring leg exits the coil at the pivot and passes through "
               "the moving half's hub region on its way to the handle.",
    )
    # Prove the spring leg contacts the moving shank (not floating)
    ctx.expect_contact(
        half0,
        half1,
        elem_a="spring_leg_1",
        elem_b="shank",
        contact_tol=0.005,
        name="spring leg presses against the moving handle shank at rest",
    )

    # ---- Overall envelope ----
    a0 = ctx.part_world_aabb(half0)
    a1 = ctx.part_world_aabb(half1)
    if a0 is not None and a1 is not None:
        length = max(a0[1][0], a1[1][0]) - min(a0[0][0], a1[0][0])
        ctx.check(
            "overall length about 0.20 m",
            0.19 <= length <= 0.215,
            details=f"length={length:.4f}",
        )

    # ---- Decisive open pose: jaws separate, handles spread ----
    closed_jaw1 = ctx.part_element_world_aabb(half1, elem="jaw")
    closed_grip1 = ctx.part_element_world_aabb(half1, elem="outer_grip_sleeve")
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
        open_grip1 = ctx.part_element_world_aabb(half1, elem="outer_grip_sleeve")
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
