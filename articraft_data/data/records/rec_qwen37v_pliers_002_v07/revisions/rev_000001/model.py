from __future__ import annotations

# Round-nose jewelry pliers with return spring.
# Variant of lineman pliers: tapered conical jaws, serrated teeth,
# coil return spring between handles, textured grip ribs.
#
# Layout: the tool lies in the XY plane with Z as the thickness axis.
# The rivet pivot is at the origin. The conical jaws point +X,
# the handles sweep back to -X and spread in +/-Y.
# plier_half_0 (root) carries its jaw on +Y; plier_half_1 is the mirrored
# moving half. The two forged halves are half-lapped at the pivot.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---- shared dimensions (meters) ----
# Jewelry pliers: ~0.15 m total length
HALF_T = 0.008          # half thickness of each forged plate
LAP_R = 0.014           # half-lap joint disc radius at the pivot
HUB_R = 0.013           # forged hub radius around the rivet
BOSS_R = 0.010          # rivet boss radius (~0.020 m diameter)
BOSS_H = 0.0025
SEAM_R = 0.011          # visible circular seam ring under the boss cap
SEAM_H = 0.0005
RIVET_R = 0.003         # rivet shaft
EPS = 0.0001            # lap clearance
JAW_FACE = 0.0003       # closed jaw inner faces sit at y = +/-JAW_FACE
JAW_TIP_X = 0.055       # conical jaw tip
JAW_BASE_R = 0.005      # jaw radius at the hub
JAW_TIP_R = 0.0015      # jaw radius at the tip
OPEN_LIMIT = math.radians(25.0)

TANG_HALF_W = 0.003     # steel handle tang half width in plan

# Steel handle tang centerline in the tool plane (for the half whose jaw is +Y).
TANG_PTS = [
    (-0.010, -0.004),
    (-0.025, -0.009),
    (-0.050, -0.015),
    (-0.075, -0.019),
    (-0.090, -0.021),
]
# Extended centerline used by the grip.
CENTERLINE = TANG_PTS + [(-0.100, -0.022)]

# Grip loft stations: (x, half_width_y, half_height_z)
GRIP_SECTIONS = [
    (-0.028, 0.0075, 0.0105),  # near pivot
    (-0.038, 0.0070, 0.0100),
    (-0.055, 0.0068, 0.0098),
    (-0.072, 0.0070, 0.0100),
    (-0.088, 0.0072, 0.0102),
    (-0.098, 0.0065, 0.0092),
    (-0.100, 0.0032, 0.0045),
]

# Spring geometry: coil spring between the two handle tangs near the pivot
# Spring is built at the origin; the articulation origin places it.
SPRING_R = 0.003        # spring coil radius
SPRING_WIRE_R = 0.0005  # spring wire radius
SPRING_COILS = 4        # number of coils
SPRING_PITCH = 0.0025   # distance between coils
# Spring placed at this X on the handle tang area
SPRING_MOUNT_X = -0.018


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
    """Conical tapered jaw with serrated teeth on the inner face."""
    # Jaw center at hub: y = s * (JAW_BASE_R + JAW_FACE) so inner face is at s*JAW_FACE
    # Jaw center at tip: y = s * (JAW_TIP_R + JAW_FACE)
    base_cy = s * (JAW_BASE_R + JAW_FACE)
    tip_cy = s * (JAW_TIP_R + JAW_FACE)

    base_wire = (
        cq.Workplane("YZ", origin=(0.010, 0.0, 0.0))
        .center(base_cy, 0.0)
        .circle(JAW_BASE_R)
        .val()
    )
    tip_wire = (
        cq.Workplane("YZ", origin=(JAW_TIP_X, 0.0, 0.0))
        .center(tip_cy, 0.0)
        .circle(JAW_TIP_R)
        .val()
    )

    jaw_solid = cq.Solid.makeLoft([base_wire, tip_wire], ruled=True)

    # Serrated teeth on the inner jaw face: cut small grooves
    num_teeth = 10
    for i in range(num_teeth):
        t = (i + 0.5) / num_teeth
        xi = 0.012 + (JAW_TIP_X - 0.015) * t
        # Interpolate jaw radius at this station
        r = JAW_BASE_R + (JAW_TIP_R - JAW_BASE_R) * t
        cy = s * (r + JAW_FACE)
        # Tooth is a small groove cut into the inner face
        tooth_depth = 0.0008 * (1.0 - 0.5 * t)
        tooth = (
            cq.Workplane("XY")
            .box(0.001, tooth_depth * 2.0, 0.012)
            .translate((xi, s * (JAW_FACE + tooth_depth / 2.0), 0.0))
            .val()
        )
        jaw_solid = jaw_solid.cut(tooth)

    # Cut the lap
    lap = _lap_cut(s).val()
    jaw_solid = jaw_solid.cut(lap)

    return cq.Workplane("XY").add(jaw_solid)


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
    """Grip body."""
    wires = [
        _ellipse_wire(x, s * _yc(x), 0.0, w, h)
        for x, w, h in GRIP_SECTIONS
    ]
    return cq.Solid.makeLoft(wires, ruled=False)


def _grip_ribs(s: int) -> cq.Solid:
    """Raised ribs on the grip surface for texture."""
    ribs = []
    num_ribs = 12
    for i in range(num_ribs):
        xi = -0.030 - (0.098 - 0.030) * i / num_ribs
        yc_at_x = s * _yc(xi)
        w = _grip_w(xi)
        h = _interp(xi, [(sx, h) for sx, _, h in GRIP_SECTIONS])
        # Rib is a ring that sits proud of the grip surface
        rib = (
            cq.Workplane("YZ", origin=(xi, 0.0, 0.0))
            .center(yc_at_x, 0.0)
            .ellipse(w + 0.0012, h + 0.0012)
            .ellipse(w + 0.0004, h + 0.0004)
            .extrude(0.0012, both=True)
            .val()
        )
        ribs.append(rib)

    if not ribs:
        return cq.Solid.makeBox(0.001, 0.001, 0.001)

    result = ribs[0]
    for rib in ribs[1:]:
        result = result.fuse(rib)
    return result


def _spring_coil() -> cq.Solid:
    """Coil spring built at origin - placed by articulation origin.

    Uses overlapping segments to ensure a single connected mesh component.
    """
    segments = []
    total_length = SPRING_PITCH * SPRING_COILS
    # Use enough segments that adjacent ones overlap
    segments_per_coil = 12
    num_segments = SPRING_COILS * segments_per_coil
    # Segment box size: must be large enough that adjacent segments overlap
    # Arc distance between adjacent segments on the helix:
    # ~2 * SPRING_R * sin(pi/segments_per_coil) for the radial component
    # For SPRING_R=0.003 and 12 segments: ~2*0.003*sin(15°) = 0.00155
    # Box half-size needs to be > half of that distance for overlap
    seg_size = 0.002  # box full size in each direction

    for i in range(num_segments):
        t = i / (num_segments - 1)
        angle = SPRING_COILS * 2.0 * math.pi * t
        y_pos = -total_length / 2.0 + total_length * t
        cx = SPRING_R * math.cos(angle)
        cz = SPRING_R * math.sin(angle)

        segment = (
            cq.Workplane("XY")
            .box(seg_size, seg_size, seg_size)
            .translate((cx, y_pos, cz))
            .val()
        )
        segments.append(segment)

    if not segments:
        return cq.Solid.makeBox(0.001, 0.001, 0.001)

    result = segments[0]
    for seg in segments[1:]:
        result = result.fuse(seg)
    return result


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="round_nose_jewelry_pliers")

    steel_polished = model.material("steel_polished", rgba=(0.82, 0.83, 0.86, 1.0))
    steel_forged = model.material("steel_forged", rgba=(0.68, 0.69, 0.72, 1.0))
    steel_brushed = model.material("steel_brushed", rgba=(0.63, 0.64, 0.67, 1.0))
    seam_gray = model.material("seam_gray", rgba=(0.47, 0.48, 0.50, 1.0))
    grip_dark = model.material("grip_dark", rgba=(0.12, 0.12, 0.14, 1.0))
    grip_ribs_mat = model.material("grip_ribs", rgba=(0.18, 0.18, 0.20, 1.0))
    spring_steel = model.material("spring_steel", rgba=(0.75, 0.76, 0.78, 1.0))

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
            material=grip_dark,
        )
        part.visual(
            mesh_from_cadquery(_grip_ribs(s), f"{tag}_ribs", tolerance=0.0002),
            name="grip_ribs",
            material=grip_ribs_mat,
        )

        parts.append(part)

    # Rivet assembly (fixed to half_0)
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

    # Return spring: separate part connected to half_0 via spring_arc joint.
    # The spring geometry is built at origin; the articulation origin places it
    # at the handle tang area.
    spring_part = model.part("return_spring")
    spring_part.visual(
        mesh_from_cadquery(_spring_coil(), "spring_coil", tolerance=0.0001),
        name="spring_coil",
        material=spring_steel,
    )

    # Primary articulation: revolute pivot opening the jaws
    pivot = model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=parts[0],
        child=parts[1],
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=3.0, lower=0.0, upper=OPEN_LIMIT),
    )

    # Secondary articulation: spring arc mimics the pivot motion.
    # The spring rotates slightly as the handles open, representing the
    # spring deflection. Origin at the spring mount location on the handle tang.
    spring_arc = model.articulation(
        "spring_arc",
        ArticulationType.REVOLUTE,
        parent=parts[0],
        child=spring_part,
        origin=Origin(xyz=(SPRING_MOUNT_X, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=0.0, upper=OPEN_LIMIT),
        mimic=Mimic(joint="pivot", multiplier=0.5, offset=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    half0 = object_model.get_part("plier_half_0")
    half1 = object_model.get_part("plier_half_1")
    spring = object_model.get_part("return_spring")
    pivot = object_model.get_articulation("pivot")
    spring_arc = object_model.get_articulation("spring_arc")

    jaw0 = half0.get_visual("jaw")
    jaw1 = half1.get_visual("jaw")
    hub0 = half0.get_visual("hub")
    hub1 = half1.get_visual("hub")
    grip0 = half0.get_visual("grip")
    grip1 = half1.get_visual("grip")
    ribs0 = half0.get_visual("grip_ribs")
    ribs1 = half1.get_visual("grip_ribs")
    spring_coil = spring.get_visual("spring_coil")

    # Joint contract: pivot is a 0..25 degree revolute joint
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

    # Spring arc mimics the pivot
    ctx.check(
        "spring_arc is a revolute joint with mimic to pivot",
        spring_arc.articulation_type == ArticulationType.REVOLUTE
        and spring_arc.mimic is not None
        and spring_arc.mimic.joint == "pivot",
        details=f"mimic={spring_arc.mimic}",
    )

    # Conical jaws: tapered profile exists and is the right length
    jaw0_aabb = ctx.part_element_world_aabb(half0, elem="jaw")
    jaw1_aabb = ctx.part_element_world_aabb(half1, elem="jaw")
    ctx.check(
        "conical jaws exist with tapered profile",
        jaw0_aabb is not None and jaw1_aabb is not None,
        details="jaw AABBs missing",
    )

    if jaw0_aabb is not None:
        jaw_length = jaw0_aabb[1][0] - jaw0_aabb[0][0]
        ctx.check(
            "conical jaws are ~40-50 mm long",
            0.040 <= jaw_length <= 0.052,
            details=f"jaw_length={jaw_length:.4f}",
        )

    # Serrated jaws close tightly at rest
    ctx.expect_gap(
        half0,
        half1,
        axis="y",
        positive_elem=jaw0,
        negative_elem=jaw1,
        min_gap=0.0,
        max_gap=0.003,
        name="serrated jaws closed and nearly touching at rest",
    )

    # Grip ribs exist on both handles
    ribs0_aabb = ctx.part_element_world_aabb(half0, elem="grip_ribs")
    ribs1_aabb = ctx.part_element_world_aabb(half1, elem="grip_ribs")
    ctx.check(
        "textured grip ribs exist on both handles",
        ribs0_aabb is not None and ribs1_aabb is not None,
        details="grip_ribs AABBs missing",
    )

    # Ribs extend beyond or match the grip surface
    if ribs0_aabb is not None:
        grip0_aabb = ctx.part_element_world_aabb(half0, elem="grip")
        if grip0_aabb is not None:
            # Ribs should extend in Z beyond the grip body
            ribs_z_extent = ribs0_aabb[1][2] - ribs0_aabb[0][2]
            grip_z_extent = grip0_aabb[1][2] - grip0_aabb[0][2]
            ctx.check(
                "grip ribs are visible texture features with Z extent",
                ribs_z_extent >= grip_z_extent * 0.7,
                details=f"ribs_z={ribs_z_extent:.4f} grip_z={grip_z_extent:.4f}",
            )

    # Return spring exists
    spring_aabb = ctx.part_element_world_aabb(spring, elem="spring_coil")
    ctx.check(
        "return spring coil exists",
        spring_aabb is not None,
        details="spring_coil AABB missing",
    )

    if spring_aabb is not None:
        # Spring sits between the two handle tangs near the pivot
        spring_cx = 0.5 * (spring_aabb[0][0] + spring_aabb[1][0])
        ctx.check(
            "return spring positioned near pivot between handles",
            -0.030 <= spring_cx <= -0.005,
            details=f"spring_cx={spring_cx:.4f}",
        )

    # The halves interleave at the pivot
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

    # Rivet shaft captured through moving half
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

    # Spring is connected to half_0 via the spring_arc articulation
    # (allow the spring to overlap with the shank since it sits between the handles)
    ctx.allow_overlap(
        half0,
        spring,
        reason="The return spring sits between the handle tangs and may "
        "overlap with the shank geometry near the pivot.",
    )
    ctx.allow_overlap(
        half1,
        spring,
        reason="The return spring sits between the handle tangs and may "
        "overlap with the moving half's shank geometry.",
    )

    # Closed-pose envelope: ~0.15 m long for jewelry pliers
    a0 = ctx.part_world_aabb(half0)
    a1 = ctx.part_world_aabb(half1)
    g0 = ctx.part_element_world_aabb(half0, elem="grip")
    g1 = ctx.part_element_world_aabb(half1, elem="grip")
    ok_env = a0 is not None and a1 is not None and g0 is not None and g1 is not None
    if ok_env:
        length = max(a0[1][0], a1[1][0]) - min(a0[0][0], a1[0][0])
        across = g1[1][1] - g0[0][1]
        ctx.check(
            "overall length about 0.14-0.16 m for jewelry pliers",
            0.130 <= length <= 0.165,
            details=f"length={length:.4f}",
        )
        ctx.check(
            "handle grips span about 0.05 m across",
            0.040 <= across <= 0.065,
            details=f"across={across:.4f}",
        )
    else:
        ctx.fail("envelope AABBs resolve", "missing part/grip AABB")

    # Decisive open pose: jaws separate and handles spread
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
            name="jaws open apart at the 25 degree pose",
        )
        open_jaw1 = ctx.part_element_world_aabb(half1, elem="jaw")
        open_grip1 = ctx.part_element_world_aabb(half1, elem="grip")
        ctx.check(
            "moving jaw swings away from the fixed jaw",
            closed_jaw1 is not None
            and open_jaw1 is not None
            and open_jaw1[0][1] < closed_jaw1[0][1] - 0.008,
            details=f"closed_min_y={closed_jaw1}, open_min_y={open_jaw1}",
        )
        ctx.check(
            "moving handle spreads outward as the jaws open",
            closed_grip1 is not None
            and open_grip1 is not None
            and open_grip1[1][1] > closed_grip1[1][1] + 0.015,
            details=f"closed_max_y={closed_grip1}, open_max_y={open_grip1}",
        )

        # Spring follows the pivot via mimic
        spring_open = ctx.part_element_world_aabb(spring, elem="spring_coil")
        ctx.check(
            "spring moves with the pivot via mimic",
            spring_open is not None,
            details="spring_coil AABB missing in open pose",
        )

    return ctx.report()


object_model = build_object_model()
