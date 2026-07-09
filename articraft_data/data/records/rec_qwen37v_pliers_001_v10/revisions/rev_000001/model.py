from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Insulated pliers with return spring and layered grip sleeves.
#
# Variant of compact flush-cut wire snips → insulated flat-nose pliers with:
#   - thick layered grip sleeves (inner dark rubber + outer red insulation)
#   - torsion return spring at the pivot with a secondary revolute arc
#   - circular pivot caps on both sides
#   - color-separated but geometric grip sections
#
# Two mirrored half-tools cross at a single rivet. Each half: short flat-nose
# gripping jaw -> steel neck/tang -> thick insulated handle. The lower half is
# the base link; the upper half rotates about the rivet axis (Z, perpendicular
# to the tool plane). A torsion spring part follows the pivot via a mimic joint.
# ---------------------------------------------------------------------------

HALF_OPEN = math.radians(12.5)  # each half yawed this much at rest (open)
CLOSE_TRAVEL = 2.0 * HALF_OPEN  # ~25 degrees of closing travel

PLATE_T = 0.0035  # forged steel plate thickness per half

# Steel layer stack (lower plate below upper plate at the pivot boss).
ZL0, ZL1 = 0.0035, 0.0035 + PLATE_T  # lower half plate: 0.0035 .. 0.0070
ZU0, ZU1 = ZL1, ZL1 + PLATE_T  # upper half plate: 0.0070 .. 0.0105

# Full-height jaw region (flat-nose gripping head spans both layers).
BLADE_Z0, BLADE_Z1 = ZL0, ZU1
BLADE_X_MIN = 0.0085

BOSS_R = 0.0075
HOLE_R = 0.0028
SHANK_R = 0.0024
HEAD_R = 0.0040

EDGE_LAND = 0.0003  # gripping face land offset

# Thick grip dimensions (insulated handles are much larger than bare steel).
GRIP_H = 0.0140  # total grip height (taller than snips for insulation)
GRIP_LZ0 = -0.002  # lower grip extends below the steel
GRIP_LZ1 = GRIP_LZ0 + GRIP_H
GRIP_UZ0 = GRIP_LZ0 + (ZU0 - ZL0)
GRIP_UZ1 = GRIP_UZ0 + GRIP_H

# Inner rubber sleeve is slightly smaller than outer colored sleeve.
INNER_GRIP_INSET = 0.0012  # inner sleeve is inset from outer sleeve outline

# Pivot cap dimensions (visible circular caps on both sides of the pivot).
CAP_R = 0.0060
CAP_T = 0.0015  # cap thickness

# Torsion spring parameters.
SPRING_COIL_R = 0.0038  # coil center radius from pivot axis (inside boss)
SPRING_WIRE_R = 0.0007  # wire cross-section radius
SPRING_TURNS = 3.0  # number of coil turns
SPRING_PITCH = 0.0018  # vertical rise per turn
SPRING_ARM_LEN = 0.022  # straight arm length from coil to handle

# Flat-nose plier jaw outline (closed-design frame, lower half, jaw at +y).
JAW_PTS = [
    (0.0000, 0.0090),
    (0.0060, 0.0098),
    (0.0140, 0.0096),
    (0.0220, 0.0084),
    (0.0280, 0.0060),
    (0.0300, 0.0040),
    (0.0295, 0.0018),
    (0.0090, EDGE_LAND),
    (0.0020, 0.0048),
]

# Slim steel neck/tang from boss back to handle (lower half: -y side).
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

# Thick outer grip sleeve outline (much wider than original for insulation).
GRIP_OUTER_PTS = [
    (-0.0220, -0.0030),
    (-0.0360, -0.0042),
    (-0.0520, -0.0062),
    (-0.0680, -0.0080),
    (-0.0820, -0.0096),
    (-0.0920, -0.0110),
    (-0.0960, -0.0128),
    (-0.0920, -0.0158),
    (-0.0800, -0.0176),
    (-0.0660, -0.0182),
    (-0.0500, -0.0174),
    (-0.0360, -0.0152),
    (-0.0240, -0.0120),
    (-0.0210, -0.0088),
]

# Inner grip sleeve outline (inset version of outer for the rubber base layer).
GRIP_INNER_PTS = [
    (-0.0230, -0.0038),
    (-0.0370, -0.0052),
    (-0.0530, -0.0072),
    (-0.0690, -0.0090),
    (-0.0830, -0.0106),
    (-0.0910, -0.0118),
    (-0.0940, -0.0134),
    (-0.0900, -0.0158),
    (-0.0780, -0.0170),
    (-0.0640, -0.0174),
    (-0.0490, -0.0166),
    (-0.0360, -0.0148),
    (-0.0250, -0.0118),
    (-0.0220, -0.0092),
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
    cap: float = 0.0018,
    inset: float = 0.0015,
) -> cq.Workplane:
    """Extruded spline outline with loft-capped (softly beveled) top/bottom."""
    mid = cq.Solid.extrudeLinear(
        cq.Face.makeFromWires(_spline_wire(pts, z0 + cap)),
        cq.Vector(0.0, 0.0, (z1 - z0) - 2.0 * cap),
    )
    top = cq.Solid.makeLoft([_spline_wire(pts, z1 - cap), _spline_wire(pts, z1, inset=inset)])
    bot = cq.Solid.makeLoft([_spline_wire(pts, z0, inset=inset), _spline_wire(pts, z0 + cap)])
    return cq.Workplane(obj=mid.fuse(top).fuse(bot))


def _spline_prism(pts: list[tuple[float, float]], z0: float, z1: float) -> cq.Workplane:
    face = cq.Face.makeFromWires(_spline_wire(pts, z0))
    return cq.Workplane(obj=cq.Solid.extrudeLinear(face, cq.Vector(0.0, 0.0, z1 - z0)))


def _jaw_grip_flat(s: float) -> cq.Workplane:
    """Flat gripping face pad on the jaw tip (instead of a cutting bevel)."""
    pad_pts = [
        (0.0090, s * EDGE_LAND),
        (0.0280, s * 0.0018),
        (0.0275, s * 0.0050),
        (0.0090, s * 0.0050),
    ]
    return _poly_prism(pad_pts, BLADE_Z0 + 0.0002, BLADE_Z1 - 0.0002)


def _half_jaw(s: float, own_z0: float, own_z1: float) -> cq.Workplane:
    """Flat-nose plier jaw: full-height front section + own-layer rear."""
    prof = _mirror(JAW_PTS, s)
    full = _poly_prism(prof, BLADE_Z0, BLADE_Z1)
    # Clip to forward region only
    clip = cq.Workplane("XY", origin=(BLADE_X_MIN + 0.0180, 0.0, 0.0070)).box(
        0.0500, 0.0600, 0.0200
    )
    full = full.intersect(clip)
    rear = _poly_prism(prof, own_z0, own_z1)
    jaw = full.union(rear)
    # Add grip texture pad on the inner face
    grip_pad = _jaw_grip_flat(s)
    jaw = jaw.union(grip_pad)
    return jaw


def _half_boss(own_z0: float, own_z1: float, with_hole: bool) -> cq.Workplane:
    boss = cq.Workplane("XY", origin=(0.0, 0.0, own_z0)).circle(BOSS_R).extrude(
        own_z1 - own_z0
    )
    if with_hole:
        hole = cq.Workplane("XY", origin=(0.0, 0.0, own_z0 - 0.001)).circle(HOLE_R).extrude(
            (own_z1 - own_z0) + 0.002
        )
        boss = boss.cut(hole)
    return boss


def _pivot_cap(z_center: float) -> cq.Workplane:
    """Circular pivot cap disk."""
    cap = (
        cq.Workplane("XY", origin=(0.0, 0.0, z_center - CAP_T / 2))
        .circle(CAP_R)
        .extrude(CAP_T)
    )
    try:
        cap = cap.edges(">Z").fillet(0.0005)
        cap = cap.edges("<Z").fillet(0.0005)
    except Exception:
        pass
    return cap


def _rivet() -> cq.Workplane:
    lower_head = cq.Workplane("XY", origin=(0.0, 0.0, 0.0021)).circle(HEAD_R).extrude(0.0019)
    shank = cq.Workplane("XY", origin=(0.0, 0.0, 0.0021)).circle(SHANK_R).extrude(0.0100)
    upper_head = cq.Workplane("XY", origin=(0.0, 0.0, ZU1 + 0.0001)).circle(HEAD_R).extrude(
        0.0014
    )
    rivet = lower_head.union(shank).union(upper_head)
    try:
        rivet = rivet.edges(">Z").fillet(0.0009)
    except Exception:
        pass
    return rivet


def _torsion_spring() -> cq.Workplane:
    """Torsion spring: helical coil around pivot axis with two extending arms.

    The arms are built as Y-aligned cylinders that physically overlap with
    the coil wire at the start and end points, ensuring a single connected
    solid after boolean union.
    """
    z_base = ZL0 - 0.001  # spring sits just below lower plate
    total_z = SPRING_TURNS * SPRING_PITCH
    n_pts = int(SPRING_TURNS * 48) + 1

    # Build helix path
    helix_pts = []
    for i in range(n_pts):
        t = i / (n_pts - 1)
        angle = t * SPRING_TURNS * 2.0 * math.pi
        x = SPRING_COIL_R * math.cos(angle)
        y = SPRING_COIL_R * math.sin(angle)
        z = z_base + t * total_z
        helix_pts.append((x, y, z))

    # Create helix wire and sweep a circle profile along it
    helix_vectors = [cq.Vector(x, y, z) for (x, y, z) in helix_pts]
    helix_edge = cq.Edge.makeSpline(helix_vectors, periodic=False)
    helix_wire = cq.Wire.assembleEdges([helix_edge])

    profile = (
        cq.Workplane("XY", origin=(helix_pts[0][0], helix_pts[0][1], helix_pts[0][2]))
        .transformed(rotate=(90, 0, 0))
        .circle(SPRING_WIRE_R)
    )
    path_wp = cq.Workplane("XY").newObject([helix_wire])
    coil_solid = profile.sweep(path_wp)

    # Build arms as Y-aligned cylinders with minimal overlap into the coil wire.
    arm_r = SPRING_WIRE_R
    overlap = SPRING_WIRE_R  # minimal overlap for connectivity (stays inside coil wire)

    # Coil start: angle=0, at (SPRING_COIL_R, 0, z_base).
    # The helix tangent at angle=0 is in the +Y direction.
    # Arms extend ONLY in -Y (toward handles), not into the jaw (+Y) region.
    sx, sy, sz = SPRING_COIL_R, 0.0, z_base
    arm_y_max = sy + overlap  # barely into coil wire for connectivity
    arm_y_min = sy - SPRING_ARM_LEN
    arm_mid_y = (arm_y_max + arm_y_min) / 2.0
    arm_half_len = (arm_y_max - arm_y_min) / 2.0

    # Y-aligned cylinder: build from XZ workplane at the midpoint
    lower_arm = (
        cq.Workplane("XZ", origin=(sx, arm_mid_y, sz))
        .circle(arm_r)
        .extrude(arm_half_len)
    )
    lower_arm_neg = (
        cq.Workplane("XZ", origin=(sx, arm_mid_y, sz))
        .circle(arm_r)
        .extrude(-arm_half_len)
    )
    lower_arm = lower_arm.union(lower_arm_neg)

    # Coil end: angle = SPRING_TURNS * 2π (integer turns → same as start in XY)
    end_angle = SPRING_TURNS * 2.0 * math.pi
    ex = SPRING_COIL_R * math.cos(end_angle)
    ey = SPRING_COIL_R * math.sin(end_angle)
    ez = z_base + total_z

    arm_y_max_u = ey + overlap
    arm_y_min_u = ey - SPRING_ARM_LEN
    arm_mid_yu = (arm_y_max_u + arm_y_min_u) / 2.0
    arm_half_len_u = (arm_y_max_u - arm_y_min_u) / 2.0

    upper_arm = (
        cq.Workplane("XZ", origin=(ex, arm_mid_yu, ez))
        .circle(arm_r)
        .extrude(arm_half_len_u)
    )
    upper_arm_neg = (
        cq.Workplane("XZ", origin=(ex, arm_mid_yu, ez))
        .circle(arm_r)
        .extrude(-arm_half_len_u)
    )
    upper_arm = upper_arm.union(upper_arm_neg)

    spring = coil_solid.union(lower_arm).union(upper_arm)
    return spring


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="insulated_pliers_with_spring")

    # Materials
    steel = model.material("brushed_steel", rgba=(0.72, 0.73, 0.75, 1.0))
    polished = model.material("polished_steel", rgba=(0.86, 0.87, 0.90, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.15, 0.15, 0.17, 1.0))
    red_insulation = model.material("red_insulation", rgba=(0.85, 0.12, 0.10, 1.0))
    chrome_cap = model.material("chrome_cap", rgba=(0.82, 0.83, 0.86, 1.0))
    spring_steel = model.material("spring_steel", rgba=(0.55, 0.56, 0.58, 1.0))

    # ----- lower half (base link): jaw at +y, handle at -y
    lower = model.part("lower_half")
    lower_pose = Origin(rpy=(0.0, 0.0, HALF_OPEN))

    lower.visual(
        mesh_from_cadquery(_half_jaw(+1.0, ZL0, ZL1), "lower_jaw"),
        origin=lower_pose,
        material=steel,
        name="jaw",
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
    # Inner grip sleeve (dark rubber base layer)
    lower.visual(
        mesh_from_cadquery(
            _soft_prism(_mirror(GRIP_INNER_PTS, +1.0), GRIP_LZ0, GRIP_LZ1), "lower_grip_inner"
        ),
        origin=lower_pose,
        material=dark_rubber,
        name="grip_inner",
    )
    # Outer grip sleeve (red insulation layer, slightly smaller in Z range)
    lower.visual(
        mesh_from_cadquery(
            _soft_prism(_mirror(GRIP_OUTER_PTS, +1.0), GRIP_LZ0 + 0.001, GRIP_LZ1 - 0.001),
            "lower_grip_outer",
        ),
        origin=lower_pose,
        material=red_insulation,
        name="grip_outer",
    )
    # Rivet
    lower.visual(
        mesh_from_cadquery(_rivet(), "rivet"),
        origin=lower_pose,
        material=polished,
        name="rivet",
    )
    # Lower pivot cap (circular disk on the bottom)
    lower.visual(
        mesh_from_cadquery(_pivot_cap(ZL0 - CAP_T), "lower_cap"),
        origin=lower_pose,
        material=chrome_cap,
        name="pivot_cap_lower",
    )

    # ----- upper half (moving link): mirrored, upper steel layer, bored boss
    upper = model.part("upper_half")
    upper.visual(
        mesh_from_cadquery(_half_jaw(-1.0, ZU0, ZU1), "upper_jaw"),
        material=steel,
        name="jaw",
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
    # Inner grip sleeve (dark rubber base layer)
    upper.visual(
        mesh_from_cadquery(
            _soft_prism(_mirror(GRIP_INNER_PTS, -1.0), GRIP_UZ0, GRIP_UZ1), "upper_grip_inner"
        ),
        material=dark_rubber,
        name="grip_inner",
    )
    # Outer grip sleeve (red insulation layer)
    upper.visual(
        mesh_from_cadquery(
            _soft_prism(_mirror(GRIP_OUTER_PTS, -1.0), GRIP_UZ0 + 0.001, GRIP_UZ1 - 0.001),
            "upper_grip_outer",
        ),
        material=red_insulation,
        name="grip_outer",
    )
    # Upper pivot cap (circular disk on the top)
    upper.visual(
        mesh_from_cadquery(_pivot_cap(ZU1), "upper_cap"),
        material=chrome_cap,
        name="pivot_cap_upper",
    )

    # ----- Return spring: separate part linked via mimic joint
    spring = model.part("return_spring")
    spring.visual(
        mesh_from_cadquery(_torsion_spring(), "spring_coil"),
        material=spring_steel,
        name="spring_coil",
    )

    # ----- main revolute pivot at the rivet
    model.articulation(
        "rivet_pivot",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, -HALF_OPEN)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=4.0, lower=0.0, upper=CLOSE_TRAVEL),
    )

    # ----- spring arc: secondary revolute that mimics the main pivot
    # The spring partially follows the pivot (smaller angular travel representing
    # the spring winding up as the handles close).
    model.articulation(
        "spring_arc",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=spring,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, -HALF_OPEN)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=4.0,
            lower=0.0,
            upper=CLOSE_TRAVEL * 0.35,
        ),
        mimic=Mimic(joint="rivet_pivot", multiplier=0.35, offset=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower = object_model.get_part("lower_half")
    upper = object_model.get_part("upper_half")
    spring = object_model.get_part("return_spring")
    pivot = object_model.get_articulation("rivet_pivot")
    spring_joint = object_model.get_articulation("spring_arc")

    # --- Intentional overlap allowances ---
    # The torsion spring coil wraps around the rivet at the pivot axis.
    ctx.allow_overlap(
        lower,
        spring,
        elem_a="rivet",
        elem_b="spring_coil",
        reason="The torsion spring coil intentionally wraps around the rivet at the pivot axis.",
    )
    # The spring arm passes through the pivot region near the jaw base.
    ctx.allow_overlap(
        spring,
        upper,
        elem_a="spring_coil",
        elem_b="jaw",
        reason="The spring coil and arms pass through the pivot region near the jaw base; this is intentional for the torsion spring mechanism at the central rivet.",
    )

    # Proof: spring coil center is near the pivot axis (XY) and overlaps the boss Z range
    ctx.expect_overlap(
        spring,
        lower,
        axes="xy",
        elem_a="spring_coil",
        elem_b="pivot_boss",
        min_overlap=0.005,
        name="spring coil overlaps with boss footprint",
    )
    ctx.expect_overlap(
        spring,
        lower,
        axes="z",
        elem_a="spring_coil",
        elem_b="pivot_boss",
        min_overlap=0.001,
        name="spring coil overlaps with boss in Z",
    )

    # --- pivot stack: bosses coaxial, upper half sits slightly above lower
    ctx.expect_overlap(
        lower,
        upper,
        axes="xy",
        elem_a="pivot_boss",
        elem_b="pivot_boss",
        min_overlap=0.012,
        name="pivot bosses stack coaxially",
    )
    ctx.expect_gap(
        upper,
        lower,
        axis="z",
        positive_elem="pivot_boss",
        negative_elem="pivot_boss",
        min_gap=-0.00002,
        max_gap=0.0006,
        name="upper boss sits slightly above lower boss",
    )

    # --- pivot caps exist on both sides and are circular disks
    for part, cap_name in [(lower, "pivot_cap_lower"), (upper, "pivot_cap_upper")]:
        cap_aabb = ctx.part_element_world_aabb(part, elem=cap_name)
        ctx.check(
            f"{part.name} has circular pivot cap",
            cap_aabb is not None,
            details=f"cap_aabb={cap_aabb}",
        )
        if cap_aabb is not None:
            cap_dx = cap_aabb[1][0] - cap_aabb[0][0]
            cap_dy = cap_aabb[1][1] - cap_aabb[0][1]
            cap_dz = cap_aabb[1][2] - cap_aabb[0][2]
            ctx.check(
                f"{part.name} pivot cap is disk-shaped (thin in Z, round in XY)",
                cap_dx > 0.008 and cap_dy > 0.008 and cap_dz < 0.004,
                details=f"dx={cap_dx:.4f}, dy={cap_dy:.4f}, dz={cap_dz:.4f}",
            )

    # --- thick layered grip sleeves: inner (dark rubber) and outer (red) both present
    for part in (lower, upper):
        inner = ctx.part_element_world_aabb(part, elem="grip_inner")
        outer = ctx.part_element_world_aabb(part, elem="grip_outer")
        ctx.check(
            f"{part.name} has inner grip sleeve",
            inner is not None,
            details=f"inner={inner}",
        )
        ctx.check(
            f"{part.name} has outer grip sleeve",
            outer is not None,
            details=f"outer={outer}",
        )
        if inner is not None and outer is not None:
            # Outer sleeve is slightly inset in Z (shorter) and slightly larger in XY
            inner_dz = inner[1][2] - inner[0][2]
            outer_dz = outer[1][2] - outer[0][2]
            ctx.check(
                f"{part.name} outer sleeve is color-separated (shorter in Z than inner)",
                outer_dz < inner_dz - 0.001,
                details=f"inner_dz={inner_dz:.4f}, outer_dz={outer_dz:.4f}",
            )
            # Outer sleeve has larger XY footprint than inner (geometric separation)
            outer_dx = outer[1][0] - outer[0][0]
            inner_dx = inner[1][0] - inner[0][0]
            ctx.check(
                f"{part.name} outer sleeve extends beyond inner in X",
                outer_dx >= inner_dx - 0.001,
                details=f"inner_dx={inner_dx:.4f}, outer_dx={outer_dx:.4f}",
            )

    # --- grip sleeves are thick (insulated, not bare handles)
    for part in (lower, upper):
        outer = ctx.part_element_world_aabb(part, elem="grip_outer")
        if outer is not None:
            grip_dy = outer[1][1] - outer[0][1]
            grip_dz = outer[1][2] - outer[0][2]
            ctx.check(
                f"{part.name} grip sleeve is thick (insulated width)",
                grip_dy > 0.010,
                details=f"grip_dy={grip_dy:.4f}",
            )
            ctx.check(
                f"{part.name} grip sleeve is tall (insulated height)",
                grip_dz > 0.010,
                details=f"grip_dz={grip_dz:.4f}",
            )

    # --- return spring part exists and has coil geometry
    spring_aabb = ctx.part_world_aabb(spring)
    ctx.check(
        "return spring has visible coil geometry",
        spring_aabb is not None,
        details=f"spring_aabb={spring_aabb}",
    )
    if spring_aabb is not None:
        spring_dx = spring_aabb[1][0] - spring_aabb[0][0]
        spring_dy = spring_aabb[1][1] - spring_aabb[0][1]
        ctx.check(
            "return spring coil has reasonable diameter",
            spring_dx > 0.006 and spring_dy > 0.006,
            details=f"dx={spring_dx:.4f}, dy={spring_dy:.4f}",
        )

    # --- spring arc is a mimic joint with non-trivial travel
    spring_limits = spring_joint.motion_limits
    ctx.check(
        "spring arc has motion limits (secondary revolute)",
        spring_limits is not None
        and spring_limits.upper is not None
        and spring_limits.upper > 0.01,
        details=f"spring_limits={spring_limits}",
    )
    ctx.check(
        "spring arc is a mimic of the main pivot",
        spring_joint.mimic is not None and spring_joint.mimic.joint == "rivet_pivot",
        details=f"mimic={spring_joint.mimic}",
    )

    # --- rest pose: jaws open, handles splayed apart
    ctx.expect_gap(
        lower,
        upper,
        axis="y",
        positive_elem="jaw",
        negative_elem="jaw",
        min_gap=0.002,
        name="jaw faces are open at rest",
    )
    ctx.expect_gap(
        upper,
        lower,
        axis="y",
        positive_elem="grip_outer",
        negative_elem="grip_outer",
        min_gap=0.008,
        name="handles splay apart at rest",
    )

    # --- overall proportions
    la = ctx.part_world_aabb(lower)
    ua = ctx.part_world_aabb(upper)
    if la is not None and ua is not None:
        length = max(la[1][0], ua[1][0]) - min(la[0][0], ua[0][0])
        width = max(la[1][1], ua[1][1]) - min(la[0][1], ua[0][1])
        ctx.check(
            "overall length ~0.12-0.15 m",
            0.10 <= length <= 0.16,
            details=f"length={length:.4f}",
        )
        ctx.check(
            "splayed handle width ~0.05-0.08 m",
            0.04 <= width <= 0.09,
            details=f"width={width:.4f}",
        )
    else:
        ctx.fail("overall proportions", "missing part AABBs")

    # --- articulation: positive q closes jaws, handles scissor in opposition
    limits = pivot.motion_limits
    ctx.check(
        "pivot travel is roughly 0..25 degrees",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower) < 1e-9
        and 0.38 <= limits.upper <= 0.48,
        details=f"limits={limits}",
    )

    open_jaw = ctx.part_element_world_aabb(upper, elem="jaw")
    open_grip = ctx.part_element_world_aabb(upper, elem="grip_outer")
    with ctx.pose({pivot: CLOSE_TRAVEL}):
        closed_jaw = ctx.part_element_world_aabb(upper, elem="jaw")
        closed_grip = ctx.part_element_world_aabb(upper, elem="grip_outer")

    ctx.check(
        "closing swings upper jaw toward lower jaw",
        open_jaw is not None
        and closed_jaw is not None
        and closed_jaw[1][1] > open_jaw[1][1] + 0.003,
        details=f"open={open_jaw}, closed={closed_jaw}",
    )
    ctx.check(
        "handles scissor opposite to the jaws",
        open_grip is not None
        and closed_grip is not None
        and closed_grip[0][1] < open_grip[0][1] - 0.003,
        details=f"open={open_grip}, closed={closed_grip}",
    )

    # --- non-fixed joint exists (main pivot is revolute with non-zero range)
    ctx.check(
        "main pivot is a non-fixed revolute joint",
        pivot.articulation_type == ArticulationType.REVOLUTE
        and limits is not None
        and limits.upper is not None
        and limits.upper > 0.1,
        details=f"type={pivot.articulation_type}, limits={limits}",
    )

    return ctx.report()


object_model = build_object_model()
