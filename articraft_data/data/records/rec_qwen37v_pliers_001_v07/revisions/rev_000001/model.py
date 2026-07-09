from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MeshGeometry,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Round-nose jewelry pliers with return spring.
#
# Two mirrored halves cross at a central rivet. Each half: tapered conical
# jaw with serrated inner face -> slim steel neck -> curved handle with
# textured grip ribs. A coil return spring sits at the pivot and opens the
# handles via a secondary revolute-linked mimic articulation.
#
# Geometry is authored per half in a "closed-design" local frame. The rest
# pose (q=0) splays each half by HALF_OPEN via yaw; positive q closes the
# jaws while handles scissor together.
# ---------------------------------------------------------------------------

HALF_OPEN = math.radians(12.5)
CLOSE_TRAVEL = 2.0 * HALF_OPEN  # ~25 deg closing

PLATE_T = 0.0035
ZL0, ZL1 = 0.0035, 0.0035 + PLATE_T  # lower plate z range
ZU0, ZU1 = ZL1, ZL1 + PLATE_T          # upper plate z range

BOSS_R = 0.0075
HOLE_R = 0.0028
SHANK_R = 0.0024
HEAD_R = 0.0040

# Conical jaw: tapered cylinder along +X, offset from centerline per half
JAW_BASE_X = 0.005      # jaw root (overlaps boss for connectivity)
JAW_TIP_X = 0.040       # jaw tip
JAW_BASE_R = 0.0032     # radius at root
JAW_TIP_R = 0.0008      # radius at tip
JAW_CY = 0.0045         # y offset per half from centerline
JAW_Z_MID = (ZL0 + ZU1) / 2  # jaw centered on full plate stack

# Handle dimensions
GRIP_H = 0.0105
GRIP_LZ0, GRIP_LZ1 = 0.0, GRIP_H
GRIP_UZ0 = ZU0 - ZL0
GRIP_UZ1 = GRIP_UZ0 + GRIP_H

# Serration teeth parameters
TOOTH_COUNT = 10
TOOTH_W = 0.0008   # width along jaw axis
TOOTH_H = 0.0006   # protrusion height toward centerline
TOOTH_D = 0.003    # depth (span across jaw face)

# Grip rib parameters
RIB_COUNT = 7
RIB_W = 0.0012    # width along handle length
RIB_H = 0.0005    # height above handle surface
RIB_D = 0.008     # depth across handle width

# Neck/tang outline (lower half at -y, mirrored for upper)
TANG_PTS = [
    (0.0020, -0.0032),
    (-0.0100, -0.0044),
    (-0.0200, -0.0052),
    (-0.0300, -0.0062),
    (-0.0300, -0.0112),
    (-0.0200, -0.0096),
    (-0.0100, -0.0082),
    (0.0008, -0.0072),
]

# Curved handle outline (periodic spline)
GRIP_PTS = [
    (-0.0240, -0.0038),
    (-0.0400, -0.0048),
    (-0.0580, -0.0060),
    (-0.0740, -0.0072),
    (-0.0890, -0.0080),
    (-0.0955, -0.0093),
    (-0.0915, -0.0128),
    (-0.0780, -0.0148),
    (-0.0620, -0.0152),
    (-0.0460, -0.0138),
    (-0.0300, -0.0112),
    (-0.0235, -0.0095),
]


def _mirror(pts: list[tuple[float, float]], s: float) -> list[tuple[float, float]]:
    return [(x, s * y) for (x, y) in pts]


def _poly_prism(pts: list[tuple[float, float]], z0: float, z1: float) -> cq.Workplane:
    return (
        cq.Workplane("XY", origin=(0.0, 0.0, z0))
        .polyline(pts)
        .close()
        .extrude(z1 - z0)
    )


def _spline_wire(pts: list[tuple[float, float]], z: float, inset: float = 0.0) -> cq.Wire:
    edge = cq.Edge.makeSpline([cq.Vector(x, y, z) for (x, y) in pts], periodic=True)
    wire = cq.Wire.assembleEdges([edge])
    if inset:
        wire = wire.offset2D(-inset)[0]
    return wire


def _soft_prism(
    pts: list[tuple[float, float]],
    z0: float,
    z1: float,
    cap: float = 0.0020,
    inset: float = 0.0018,
) -> cq.Workplane:
    mid = cq.Solid.extrudeLinear(
        cq.Face.makeFromWires(_spline_wire(pts, z0 + cap)),
        cq.Vector(0.0, 0.0, (z1 - z0) - 2.0 * cap),
    )
    top = cq.Solid.makeLoft([_spline_wire(pts, z1 - cap), _spline_wire(pts, z1, inset=inset)])
    bot = cq.Solid.makeLoft([_spline_wire(pts, z0, inset=inset), _spline_wire(pts, z0 + cap)])
    return cq.Workplane(obj=mid.fuse(top).fuse(bot))


def _conical_jaw(s: float) -> cq.Workplane:
    """Tapered round-nose jaw: cone frustum along +X axis, offset in Y."""
    cy = s * JAW_CY
    sections = []
    n_sections = 8
    for i in range(n_sections + 1):
        t = i / n_sections
        x = JAW_BASE_X + t * (JAW_TIP_X - JAW_BASE_X)
        r = JAW_BASE_R + t * (JAW_TIP_R - JAW_BASE_R)
        center = cq.Vector(x, cy, JAW_Z_MID)
        normal = cq.Vector(1, 0, 0)
        circle = cq.Wire.makeCircle(r, center, normal)
        sections.append(circle)

    jaw_solid = cq.Solid.makeLoft(sections)

    # Root connector: short cylinder at jaw base overlapping with boss area
    root_circle = cq.Wire.makeCircle(JAW_BASE_R, cq.Vector(JAW_BASE_X, cy, JAW_Z_MID), cq.Vector(1, 0, 0))
    boss_circle = cq.Wire.makeCircle(JAW_BASE_R * 0.9, cq.Vector(0.0, cy, JAW_Z_MID), cq.Vector(1, 0, 0))
    root_solid = cq.Solid.makeLoft([boss_circle, root_circle])

    combined = jaw_solid.fuse(root_solid)
    return cq.Workplane(obj=combined)


def _jaw_teeth(s: float) -> cq.Workplane:
    """Serrated teeth on inner face of the conical jaw."""
    cy = s * JAW_CY
    result = None
    for i in range(TOOTH_COUNT):
        t = 0.05 + 0.9 * (i + 0.5) / TOOTH_COUNT
        x = JAW_BASE_X + t * (JAW_TIP_X - JAW_BASE_X)
        r = JAW_BASE_R + t * (JAW_TIP_R - JAW_BASE_R)

        # Tooth protrudes from inner face toward centerline
        tooth_y = cy - s * (r - TOOTH_H * 0.3)
        tooth = (
            cq.Workplane("XY", origin=(x - TOOTH_W / 2, tooth_y - TOOTH_D / 2, JAW_Z_MID - TOOTH_W / 2))
            .box(TOOTH_W, TOOTH_D, TOOTH_W * 2)
        )
        result = tooth if result is None else result.union(tooth)

    return result


def _half_boss(own_z0: float, own_z1: float, with_hole: bool) -> cq.Workplane:
    boss = cq.Workplane("XY", origin=(0.0, 0.0, own_z0)).circle(BOSS_R).extrude(own_z1 - own_z0)
    if with_hole:
        hole = cq.Workplane("XY", origin=(0.0, 0.0, own_z0 - 0.001)).circle(HOLE_R).extrude(
            (own_z1 - own_z0) + 0.002
        )
        boss = boss.cut(hole)
    return boss


def _rivet() -> cq.Workplane:
    lower_head = cq.Workplane("XY", origin=(0.0, 0.0, 0.0021)).circle(HEAD_R).extrude(0.0019)
    shank = cq.Workplane("XY", origin=(0.0, 0.0, 0.0021)).circle(SHANK_R).extrude(0.0100)
    upper_head = cq.Workplane("XY", origin=(0.0, 0.0, ZU1 + 0.0001)).circle(HEAD_R).extrude(0.0014)
    rivet = lower_head.union(shank).union(upper_head)
    try:
        rivet = rivet.edges(">Z").fillet(0.0009)
    except Exception:
        pass
    return rivet


def _grip_ribs(s: float, z_top: float) -> cq.Workplane:
    """Raised grip ribs on the handle top surface."""
    result = None
    for i in range(RIB_COUNT):
        t = 0.12 + 0.76 * i / (RIB_COUNT - 1)
        # Interpolate position along the handle spline
        x = GRIP_PTS[0][0] + t * (GRIP_PTS[6][0] - GRIP_PTS[0][0])
        y_center = s * (abs(GRIP_PTS[3][1]) + t * (abs(GRIP_PTS[6][1]) - abs(GRIP_PTS[3][1])))
        y = -y_center if s > 0 else y_center
        y = s * (-abs(GRIP_PTS[0][1]) - t * (abs(GRIP_PTS[6][1]) - abs(GRIP_PTS[0][1])))

        rib = (
            cq.Workplane("XY", origin=(x, y, z_top))
            .box(RIB_W, RIB_D, RIB_H)
        )
        result = rib if result is None else result.union(rib)

    return result


def _spring_mesh() -> MeshGeometry:
    """Build a coil spring as a procedural mesh (helical tube)."""
    coil_r = 0.0045   # coil radius
    wire_r = 0.0005   # wire cross-section radius
    n_turns = 4
    pitch = 0.0018    # spacing between turns
    n_along = n_turns * 28  # segments along helix
    n_around = 10     # segments around wire cross-section

    geom = MeshGeometry()

    # Generate vertices along the helix
    for i in range(n_along + 1):
        t = i / n_along
        helix_angle = 2.0 * math.pi * n_turns * t
        # Center of wire cross-section
        cx = coil_r * math.cos(helix_angle)
        cy = coil_r * math.sin(helix_angle)
        cz = pitch * n_turns * t + ZL0  # spring sits at lower plate level

        # Tangent to helix
        d_angle = 2.0 * math.pi * n_turns
        tx = -coil_r * d_angle * math.sin(helix_angle)
        ty = coil_r * d_angle * math.cos(helix_angle)
        tz = pitch * n_turns
        tlen = math.sqrt(tx * tx + ty * ty + tz * tz)
        tx, ty, tz = tx / tlen, ty / tlen, tz / tlen

        # Build local frame (Frenet-like)
        # Initial guess for normal
        nx, ny, nz = math.cos(helix_angle), math.sin(helix_angle), 0.0
        # Binormal = tangent x normal
        bx = ty * nz - tz * ny
        by = tz * nx - tx * nz
        bz = tx * ny - ty * nx
        blen = math.sqrt(bx * bx + by * by + bz * bz)
        if blen < 1e-10:
            nx, ny, nz = 0.0, 0.0, 1.0
            bx = ty * nz - tz * ny
            by = tz * nx - tx * nz
            bz = tx * ny - ty * nx
            blen = math.sqrt(bx * bx + by * by + bz * bz)
        bx, by, bz = bx / blen, by / blen, bz / blen

        # Recompute normal = binormal x tangent
        nx = by * tz - bz * ty
        ny = bz * tx - bx * tz
        nz = bx * ty - by * tx

        for j in range(n_around):
            phi = 2.0 * math.pi * j / n_around
            cp, sp = math.cos(phi), math.sin(phi)
            vx = cx + wire_r * (cp * nx + sp * bx)
            vy = cy + wire_r * (cp * ny + sp * by)
            vz = cz + wire_r * (cp * nz + sp * bz)
            geom.add_vertex(vx, vy, vz)

    # Generate faces
    for i in range(n_along):
        for j in range(n_around):
            j_next = (j + 1) % n_around
            v00 = i * n_around + j
            v01 = i * n_around + j_next
            v10 = (i + 1) * n_around + j
            v11 = (i + 1) * n_around + j_next
            geom.add_face(v00, v10, v01)
            geom.add_face(v01, v10, v11)

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="round_nose_jewelry_pliers")

    steel = model.material("brushed_steel", rgba=(0.72, 0.73, 0.75, 1.0))
    polished = model.material("polished_steel", rgba=(0.86, 0.87, 0.90, 1.0))
    orange = model.material("orange_grip", rgba=(0.92, 0.45, 0.08, 1.0))
    dark_grip = model.material("dark_rubber", rgba=(0.18, 0.18, 0.20, 1.0))
    spring_mat = model.material("spring_steel", rgba=(0.65, 0.66, 0.68, 1.0))

    # ----- lower half (base link): jaw at +y, handle at -y
    lower = model.part("lower_half")
    lower_pose = Origin(rpy=(0.0, 0.0, HALF_OPEN))

    lower.visual(
        mesh_from_cadquery(_conical_jaw(+1.0), "lower_jaw"),
        origin=lower_pose,
        material=steel,
        name="jaw",
    )
    lower.visual(
        mesh_from_cadquery(_jaw_teeth(+1.0), "lower_teeth"),
        origin=lower_pose,
        material=steel,
        name="jaw_teeth",
    )
    lower.visual(
        mesh_from_cadquery(_half_boss(ZL0, ZL1, with_hole=False), "lower_boss"),
        origin=lower_pose,
        material=steel,
        name="pivot_boss",
    )
    lower.visual(
        mesh_from_cadquery(_poly_prism(_mirror(TANG_PTS, +1.0), ZL0, ZL1), "lower_tang"),
        origin=lower_pose,
        material=steel,
        name="neck_tang",
    )
    lower.visual(
        mesh_from_cadquery(_soft_prism(_mirror(GRIP_PTS, +1.0), GRIP_LZ0, GRIP_LZ1), "lower_grip"),
        origin=lower_pose,
        material=orange,
        name="grip",
    )
    lower.visual(
        mesh_from_cadquery(_grip_ribs(+1.0, GRIP_LZ1), "lower_ribs"),
        origin=lower_pose,
        material=dark_grip,
        name="grip_ribs",
    )
    lower.visual(
        mesh_from_cadquery(_rivet(), "rivet"),
        origin=lower_pose,
        material=polished,
        name="rivet",
    )

    # ----- upper half (moving link): mirrored, upper steel layer
    upper = model.part("upper_half")

    upper.visual(
        mesh_from_cadquery(_conical_jaw(-1.0), "upper_jaw"),
        material=steel,
        name="jaw",
    )
    upper.visual(
        mesh_from_cadquery(_jaw_teeth(-1.0), "upper_teeth"),
        material=steel,
        name="jaw_teeth",
    )
    upper.visual(
        mesh_from_cadquery(_half_boss(ZU0, ZU1, with_hole=True), "upper_boss"),
        material=steel,
        name="pivot_boss",
    )
    upper.visual(
        mesh_from_cadquery(_poly_prism(_mirror(TANG_PTS, -1.0), ZU0, ZU1), "upper_tang"),
        material=steel,
        name="neck_tang",
    )
    upper.visual(
        mesh_from_cadquery(_soft_prism(_mirror(GRIP_PTS, -1.0), GRIP_UZ0, GRIP_UZ1), "upper_grip"),
        material=orange,
        name="grip",
    )
    upper.visual(
        mesh_from_cadquery(_grip_ribs(-1.0, GRIP_UZ1), "upper_ribs"),
        material=dark_grip,
        name="grip_ribs",
    )

    # ----- return spring: coil at the pivot area
    spring = model.part("return_spring")
    spring.visual(
        mesh_from_geometry(_spring_mesh(), "spring_coil"),
        material=spring_mat,
        name="coil",
    )

    # ----- main pivot: revolute at the rivet
    model.articulation(
        "jaw_pivot",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, -HALF_OPEN)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=4.0, lower=0.0, upper=CLOSE_TRAVEL),
    )

    # ----- spring pivot: secondary revolute linked to main pivot via mimic
    model.articulation(
        "spring_pivot",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=spring,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, -HALF_OPEN)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=4.0, lower=0.0, upper=CLOSE_TRAVEL * 0.3),
        mimic=Mimic(joint="jaw_pivot", multiplier=0.3, offset=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower = object_model.get_part("lower_half")
    upper = object_model.get_part("upper_half")
    spring = object_model.get_part("return_spring")
    pivot = object_model.get_articulation("jaw_pivot")
    spring_joint = object_model.get_articulation("spring_pivot")

    # --- conical jaws exist and are tapered (tip narrower than base)
    for part_name in ("lower_half", "upper_half"):
        p = object_model.get_part(part_name)
        jaw = ctx.part_element_world_aabb(p, elem="jaw")
        ctx.check(
            f"{part_name} has a conical jaw visual",
            jaw is not None,
            details=f"jaw_aabb={jaw}",
        )

    # --- serrated teeth on inner jaws
    for part_name in ("lower_half", "upper_half"):
        p = object_model.get_part(part_name)
        teeth = ctx.part_element_world_aabb(p, elem="jaw_teeth")
        jaw = ctx.part_element_world_aabb(p, elem="jaw")
        ctx.check(
            f"{part_name} has serrated teeth on jaw",
            teeth is not None and jaw is not None,
            details=f"teeth={teeth}, jaw={jaw}",
        )
        # Teeth should overlap with jaw in X (along jaw length)
        if teeth is not None and jaw is not None:
            ctx.check(
                f"{part_name} teeth span the jaw length",
                teeth[0][0] >= jaw[0][0] - 0.002 and teeth[1][0] <= jaw[1][0] + 0.002,
                details=f"teeth_x=[{teeth[0][0]:.4f},{teeth[1][0]:.4f}], jaw_x=[{jaw[0][0]:.4f},{jaw[1][0]:.4f}]",
            )

    # --- grip ribs on both handles
    for part_name in ("lower_half", "upper_half"):
        p = object_model.get_part(part_name)
        ribs = ctx.part_element_world_aabb(p, elem="grip_ribs")
        grip = ctx.part_element_world_aabb(p, elem="grip")
        ctx.check(
            f"{part_name} has textured grip ribs",
            ribs is not None and grip is not None,
            details=f"ribs={ribs}, grip={grip}",
        )
        if ribs is not None and grip is not None:
            # Ribs should sit on or above the grip top surface
            ctx.check(
                f"{part_name} ribs protrude above grip surface",
                ribs[1][2] > grip[1][2] - 0.0002,
                details=f"ribs_z_max={ribs[1][2]:.5f}, grip_z_max={grip[1][2]:.5f}",
            )

    # --- return spring exists and is at the pivot area
    spring_coil = ctx.part_element_world_aabb(spring, elem="coil")
    lower_boss = ctx.part_element_world_aabb(lower, elem="pivot_boss")
    ctx.check(
        "return spring coil exists",
        spring_coil is not None,
        details=f"spring={spring_coil}",
    )
    if spring_coil is not None and lower_boss is not None:
        ctx.expect_overlap(
            spring,
            lower,
            axes="xy",
            elem_a="coil",
            elem_b="pivot_boss",
            min_overlap=0.002,
            name="spring coil overlaps pivot boss area",
        )

    # --- spring coil wraps around pivot/rivet area (intentional embedding)
    ctx.allow_overlap(
        lower, spring,
        elem_a="rivet", elem_b="coil",
        reason="The return spring coil wraps around the pivot rivet area, which is the real mounting position for a pliers return spring.",
    )
    ctx.allow_overlap(
        upper, spring,
        elem_a="pivot_boss", elem_b="coil",
        reason="The return spring coil sits at the pivot boss height between the two halves.",
    )
    ctx.expect_overlap(
        spring, lower,
        axes="xy",
        elem_a="coil", elem_b="pivot_boss",
        min_overlap=0.003,
        name="spring coil centered on pivot boss area",
    )

    # --- pivot stack: bosses coaxial, upper sits above lower
    ctx.expect_overlap(
        lower, upper,
        axes="xy",
        elem_a="pivot_boss", elem_b="pivot_boss",
        min_overlap=0.012,
        name="pivot bosses stack coaxially",
    )
    ctx.expect_gap(
        upper, lower,
        axis="z",
        positive_elem="pivot_boss", negative_elem="pivot_boss",
        min_gap=-0.00002, max_gap=0.0006,
        name="upper boss sits slightly above lower boss",
    )

    # --- rest pose: jaws open, handles splayed apart
    ctx.expect_gap(
        lower, upper,
        axis="y",
        positive_elem="jaw", negative_elem="jaw",
        min_gap=0.001,
        name="jaw tips are open at rest",
    )
    ctx.expect_gap(
        upper, lower,
        axis="y",
        positive_elem="grip", negative_elem="grip",
        min_gap=0.010,
        name="handles splay apart at rest",
    )

    # --- articulation: positive q closes jaws, handles scissor
    limits = pivot.motion_limits
    ctx.check(
        "jaw pivot travel is roughly 0..25 degrees",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower) < 1e-9
        and 0.38 <= limits.upper <= 0.48,
        details=f"limits={limits}",
    )

    open_jaw = ctx.part_element_world_aabb(upper, elem="jaw")
    open_grip = ctx.part_element_world_aabb(upper, elem="grip")
    with ctx.pose({pivot: CLOSE_TRAVEL}):
        closed_jaw = ctx.part_element_world_aabb(upper, elem="jaw")
        closed_grip = ctx.part_element_world_aabb(upper, elem="grip")

    ctx.check(
        "closing swings upper jaw toward lower jaw",
        open_jaw is not None
        and closed_jaw is not None
        and closed_jaw[1][1] > open_jaw[1][1] + 0.003,
        details=f"open={open_jaw}, closed={closed_jaw}",
    )
    ctx.check(
        "handles scissor opposite to jaws",
        open_grip is not None
        and closed_grip is not None
        and closed_grip[0][1] < open_grip[0][1] - 0.003,
        details=f"open={open_grip}, closed={closed_grip}",
    )

    # --- spring pivot is a non-fixed mimic of the main pivot
    ctx.check(
        "spring pivot is revolute (not fixed)",
        spring_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={spring_joint.articulation_type}",
    )
    ctx.check(
        "spring pivot mimics jaw pivot",
        spring_joint.mimic is not None
        and spring_joint.mimic.joint == "jaw_pivot",
        details=f"mimic={spring_joint.mimic}",
    )

    # --- overall proportions: ~0.13 long, ~0.06 wide, ~0.015 thick
    la = ctx.part_world_aabb(lower)
    ua = ctx.part_world_aabb(upper)
    if la is not None and ua is not None:
        length = max(la[1][0], ua[1][0]) - min(la[0][0], ua[0][0])
        width = max(la[1][1], ua[1][1]) - min(la[0][1], ua[0][1])
        height = max(la[1][2], ua[1][2]) - min(la[0][2], ua[0][2])
        ctx.check(
            "overall length ~0.13 m",
            0.10 <= length <= 0.15,
            details=f"length={length:.4f}",
        )
        ctx.check(
            "splayed handle width ~0.06 m",
            0.040 <= width <= 0.080,
            details=f"width={width:.4f}",
        )
        ctx.check(
            "flat tool thickness ~0.015 m",
            0.010 <= height <= 0.020,
            details=f"height={height:.4f}",
        )
    else:
        ctx.fail("overall proportions", "missing part AABBs")

    return ctx.report()


object_model = build_object_model()
