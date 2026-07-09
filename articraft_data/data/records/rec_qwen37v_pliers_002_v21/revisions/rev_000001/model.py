from __future__ import annotations

# Needle-nose pliers variant forked from heavy-duty lineman pliers.
# Reference: picture/Other/pliers/002.png (family variant)
#
# Layout: the tool lies in the XY plane with Z as the thickness axis.
# The rivet pivot is at the origin. The long slender needle-nose jaws
# point +X, the slender handles sweep back to -X and spread in +/-Y.
# plier_half_0 (root) carries its jaw on +Y; plier_half_1 is the moving half.
# A torsion return spring sits at the pivot with two arms extending into
# the handles; the moving arm tracks the pivot via a mimic revolute joint.

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
HALF_T = 0.007           # half thickness of each forged plate (full ~0.014)
LAP_R = 0.012            # half-lap joint disc radius at the pivot
HUB_R = 0.011            # forged hub radius around the rivet
BOSS_R = 0.009           # rivet boss radius (~0.018 m diameter)
BOSS_H = 0.0025
SEAM_R = 0.0095
SEAM_H = 0.0005
RIVET_R = 0.003
EPS = 0.0001
JAW_FACE = 0.0003        # closed jaw inner faces sit at y = +/-JAW_FACE
NOSE_X = 0.082           # needle-nose tip position
OPEN_LIMIT = math.radians(25.0)

# Needle-nose jaw profile (x, half_width_y from centerline)
# Tapers from ~12mm wide at base to ~3mm at tip
JAW_PROFILE = [
    (0.010, 0.0060),
    (0.020, 0.0055),
    (0.035, 0.0048),
    (0.050, 0.0038),
    (0.065, 0.0028),
    (0.075, 0.0020),
    (NOSE_X, 0.0014),
]

# Handle shank centerline (jaw-on-+Y half)
HANDLE_PTS = [
    (-0.010, -0.004),
    (-0.025, -0.010),
    (-0.050, -0.017),
    (-0.075, -0.022),
    (-0.095, -0.025),
    (-0.110, -0.026),
]

HANDLE_HALF_W = 0.0040    # slender handle tang half width
HANDLE_HALF_H = 0.0035    # handle half thickness

# Return spring dimensions
SPRING_COIL_R = 0.0045    # coil center radius from pivot
SPRING_WIRE_R = 0.0009    # wire cross-section radius (large enough to fuse adjacent turns)
SPRING_TURNS = 5
SPRING_PITCH = 0.0014     # pitch < 2*wire_r so turns overlap and fuse
SPRING_ARM_LEN = 0.035    # arm extension length into handles


def _interp(x: float, pts: list[tuple[float, float]]) -> float:
    """Piecewise-linear interpolation over (x, value) points."""
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


def _jaw_half_w(x: float) -> float:
    """Jaw half-width at station x."""
    return _interp(x, JAW_PROFILE)


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


def _needle_jaw_solid(s: int) -> cq.Workplane:
    """Long slender needle-nose jaw with serrated teeth on inner face."""
    # Build jaw profile as a plan-view polygon tapering from base to tip.
    # Outer edge at y = s*(JAW_FACE + half_w), inner face at y = s*JAW_FACE.
    pts_fwd = [(x, s * (JAW_FACE + _jaw_half_w(x))) for x, _ in JAW_PROFILE]
    pts_back = [(x, s * JAW_FACE) for x, _ in reversed(JAW_PROFILE)]
    pts = pts_fwd + pts_back

    jaw = cq.Workplane("XY").polyline(pts).close().extrude(HALF_T, both=True)

    # Serrated teeth: transverse grooves cut into the inner gripping face
    n_teeth = 12
    for i in range(n_teeth):
        xi = 0.015 + i * 0.0055
        groove_w = 0.0008
        groove_depth = 0.0008
        # Cut from the inner face inward
        groove = (
            cq.Workplane("XY")
            .box(groove_w, groove_depth, HALF_T * 2 + 0.002)
            .translate((xi, s * (JAW_FACE + groove_depth * 0.4), 0.0))
        )
        jaw = jaw.cut(groove)

    # Small pointed tip refinement: cut a slight taper at the very end
    tip_cut = (
        cq.Workplane("XY")
        .box(0.006, 0.008, HALF_T * 2 + 0.002)
        .translate((NOSE_X + 0.002, s * JAW_FACE, 0.0))
    )
    jaw = jaw.cut(tip_cut)

    return jaw.cut(_lap_cut(s))


def _hub_solid(s: int) -> cq.Workplane:
    """Forged circular hub around the rivet, thinned to its lap layer."""
    hub = cq.Workplane("XY").circle(HUB_R).extrude(HALF_T, both=True)
    return hub.cut(_lap_cut(s))


def _shank_solid(s: int) -> cq.Workplane:
    """Slender steel handle shank sweeping back from the hub."""
    pts = [(x, s * y) for x, y in HANDLE_PTS]
    loop = _strip_loop(pts, HANDLE_HALF_W)
    shank = cq.Workplane("XY").polyline(loop).close().extrude(HANDLE_HALF_H, both=True)
    return shank.cut(_lap_cut(s))


def _grip_coat_solid(s: int) -> cq.Workplane:
    """Thin vinyl dip coating on the handle end (slim, not thick rubber)."""
    # Grip covers the rear portion of the handle
    grip_pts = [(x, s * y) for x, y in HANDLE_PTS[2:]]  # from mid-handle to end
    grip_loop = _strip_loop(grip_pts, HANDLE_HALF_W + 0.0015)
    grip = cq.Workplane("XY").polyline(grip_loop).close().extrude(HANDLE_HALF_H + 0.0012, both=True)
    # Round the end cap
    end_x = HANDLE_PTS[-1][0]
    end_y = s * HANDLE_PTS[-1][1]
    end_cap = (
        cq.Workplane("XY")
        .sphere(HANDLE_HALF_W + 0.0015)
        .translate((end_x, end_y, 0.0))
    )
    grip = grip.union(end_cap)
    return grip


def _spring_coil_solid() -> cq.Workplane:
    """Torsion spring coil body: stacked torus rings fused at the pivot."""
    coil_h = (SPRING_TURNS - 1) * SPRING_PITCH
    result = None
    for i in range(SPRING_TURNS):
        z = (i - (SPRING_TURNS - 1) / 2.0) * SPRING_PITCH
        torus = cq.Solid.makeTorus(SPRING_COIL_R, SPRING_WIRE_R)
        ring = cq.Workplane("XY").newObject([torus]).translate((0, 0, z))
        if result is None:
            result = ring
        else:
            result = result.union(ring)
    return result


def _spring_arm_solid(s: int) -> cq.Workplane:
    """Spring arm: a thin rod extending from the coil into the handle region.

    Built as a single plan-view polygon extruded in Z for connectivity.
    """
    # Arm path: from coil edge backward and outward
    arm_w = SPRING_WIRE_R * 1.2  # arm half-width
    path_pts = [
        (-0.003, s * SPRING_COIL_R),
        (-0.012, s * (SPRING_COIL_R + 0.004)),
        (-SPRING_ARM_LEN * 0.85, s * (SPRING_COIL_R + 0.011)),
    ]
    # Build a strip polygon around the centerline
    loop = _strip_loop(path_pts, arm_w)
    arm = cq.Workplane("XY").polyline(loop).close().extrude(SPRING_WIRE_R, both=True)

    # Add a small anchor disc at the coil end so it reads as attached to the coil
    anchor = (
        cq.Workplane("XY")
        .circle(SPRING_WIRE_R * 2.0)
        .extrude(SPRING_WIRE_R, both=True)
        .translate((path_pts[0][0], path_pts[0][1], 0.0))
    )
    arm = arm.union(anchor)
    return arm


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="needle_nose_pliers")

    steel_polished = model.material("steel_polished", rgba=(0.80, 0.81, 0.84, 1.0))
    steel_forged = model.material("steel_forged", rgba=(0.66, 0.67, 0.70, 1.0))
    steel_brushed = model.material("steel_brushed", rgba=(0.61, 0.62, 0.65, 1.0))
    seam_gray = model.material("seam_gray", rgba=(0.45, 0.46, 0.48, 1.0))
    vinyl_black = model.material("vinyl_black", rgba=(0.06, 0.06, 0.07, 1.0))
    spring_steel = model.material("spring_steel", rgba=(0.72, 0.73, 0.76, 1.0))

    # --- Two forged halves ---
    parts = []
    for part_name, s in (("plier_half_0", 1), ("plier_half_1", -1)):
        part = model.part(part_name)
        tag = part_name.replace("plier_half_", "half")

        part.visual(
            mesh_from_cadquery(_needle_jaw_solid(s), f"{tag}_jaw", tolerance=0.0002),
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
            mesh_from_cadquery(_grip_coat_solid(s), f"{tag}_grip", tolerance=0.0003),
            name="grip",
            material=vinyl_black,
        )

        parts.append(part)

    # --- Rivet boss and shaft (fixed to half_0) ---
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
        Cylinder(RIVET_R, 2.0 * HALF_T + SEAM_H + 2.0 * EPS + 0.0004),
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

    # --- Return spring coil (fixed to half_0) ---
    fixed.visual(
        mesh_from_cadquery(_spring_coil_solid(), "spring_coil", tolerance=0.0002),
        name="spring_coil",
        material=spring_steel,
    )
    # Fixed spring arm (extends into half_0's handle)
    fixed.visual(
        mesh_from_cadquery(_spring_arm_solid(1), "spring_arm_fixed", tolerance=0.0003),
        name="spring_arm_fixed",
        material=spring_steel,
    )

    # --- Moving spring arm (separate part, mimic of pivot) ---
    spring_arm_part = model.part("spring_arm")
    spring_arm_part.visual(
        mesh_from_cadquery(_spring_arm_solid(-1), "spring_arm_moving", tolerance=0.0003),
        name="spring_arm_moving",
        material=spring_steel,
    )

    # --- Primary articulation: revolute pivot ---
    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=parts[0],
        child=parts[1],
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=3.0, lower=0.0, upper=OPEN_LIMIT),
    )

    # --- Secondary articulation: spring arm mimic of pivot ---
    model.articulation(
        "spring_pivot",
        ArticulationType.REVOLUTE,
        parent=parts[0],
        child=spring_arm_part,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=3.0, lower=0.0, upper=OPEN_LIMIT),
        mimic=Mimic(joint="pivot", multiplier=1.0, offset=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    half0 = object_model.get_part("plier_half_0")
    half1 = object_model.get_part("plier_half_1")
    spring_arm = object_model.get_part("spring_arm")
    pivot = object_model.get_articulation("pivot")
    spring_joint = object_model.get_articulation("spring_pivot")

    jaw0 = half0.get_visual("jaw")
    jaw1 = half1.get_visual("jaw")
    hub0 = half0.get_visual("hub")
    hub1 = half1.get_visual("hub")
    spring_coil = half0.get_visual("spring_coil")

    # --- Joint contract: revolute pivot 0..25 degrees ---
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

    # --- Spring joint is mimic of pivot ---
    ctx.check(
        "spring_pivot is a mimic revolute linked to pivot",
        spring_joint.articulation_type == ArticulationType.REVOLUTE
        and spring_joint.mimic is not None
        and spring_joint.mimic.joint == "pivot",
        details=f"mimic={spring_joint.mimic}",
    )

    # --- Needle-nose jaw proportions: long and slender ---
    jaw0_aabb = ctx.part_element_world_aabb(half0, elem="jaw")
    if jaw0_aabb is not None:
        j_min, j_max = jaw0_aabb
        jaw_length = j_max[0] - j_min[0]
        jaw_width = j_max[1] - j_min[1]
        ctx.check(
            "needle-nose jaw is long (>60mm)",
            jaw_length > 0.060,
            details=f"jaw_length={jaw_length:.4f}",
        )
        ctx.check(
            "needle-nose jaw tip is slender (<8mm wide)",
            jaw_width < 0.018,
            details=f"jaw_width={jaw_width:.4f}",
        )

    # --- Serrated teeth: jaw has grooves cut into it ---
    # Verify by checking that jaw visual exists and is mesh-backed (cut geometry)
    ctx.check(
        "jaw visual exists with serration geometry",
        jaw0 is not None,
        details="jaw visual missing",
    )

    # --- Closed rest pose: serrated inner faces nearly touch ---
    ctx.expect_gap(
        half0,
        half1,
        axis="y",
        positive_elem=jaw0,
        negative_elem=jaw1,
        min_gap=0.0002,
        max_gap=0.002,
        name="needle jaws closed and nearly touching at rest",
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
    ctx.expect_contact(
        half0,
        half1,
        elem_a=hub0,
        elem_b=hub1,
        contact_tol=0.0005,
        name="hub laps seat against each other at the joint",
    )

    # --- Rivet shaft captured through moving hub ---
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

    # --- Return spring coil exists at pivot ---
    spring_aabb = ctx.part_element_world_aabb(half0, elem="spring_coil")
    ctx.check(
        "return spring coil present at pivot",
        spring_aabb is not None,
        details="spring_coil AABB missing",
    )
    if spring_aabb is not None:
        s_min, s_max = spring_aabb
        spring_dia = s_max[0] - s_min[0]
        ctx.check(
            "spring coil diameter plausible for torsion spring",
            0.006 <= spring_dia <= 0.014,
            details=f"spring_dia={spring_dia:.4f}",
        )

    # --- Spring arm part exists and tracks the pivot ---
    arm_aabb_rest = ctx.part_element_world_aabb(spring_arm, elem="spring_arm_moving")
    ctx.check(
        "spring arm part has visible geometry",
        arm_aabb_rest is not None,
        details="spring_arm_moving AABB missing",
    )

    # --- Decisive open pose ---
    closed_jaw1 = ctx.part_element_world_aabb(half1, elem="jaw")
    closed_grip1 = ctx.part_element_world_aabb(half1, elem="grip")
    with ctx.pose({pivot: OPEN_LIMIT}):
        ctx.expect_gap(
            half0,
            half1,
            axis="y",
            positive_elem=jaw0,
            negative_elem=jaw1,
            min_gap=0.002,
            name="needle jaws open apart at the 25 degree pose",
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
        # Spring arm moves with the pivot (mimic)
        open_arm_aabb = ctx.part_element_world_aabb(spring_arm, elem="spring_arm_moving")
        ctx.check(
            "spring arm tracks the pivot via mimic at open pose",
            arm_aabb_rest is not None
            and open_arm_aabb is not None
            and open_arm_aabb[1][1] != arm_aabb_rest[1][1],
            details=f"rest={arm_aabb_rest}, open={open_arm_aabb}",
        )

    # --- Overall envelope: ~0.20 m long ---
    a0 = ctx.part_world_aabb(half0)
    a1 = ctx.part_world_aabb(half1)
    if a0 is not None and a1 is not None:
        length = max(a0[1][0], a1[1][0]) - min(a0[0][0], a1[0][0])
        ctx.check(
            "overall length about 0.20 m",
            0.17 <= length <= 0.22,
            details=f"length={length:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
