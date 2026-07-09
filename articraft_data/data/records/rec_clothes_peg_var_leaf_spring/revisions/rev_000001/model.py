from __future__ import annotations

# Realistic articulated clothes peg – leaf-spring fork variant.
#
# Articraft brief:
# - Object: classic wooden spring clothes peg, ~72 mm long, ~8.5 mm wide,
#   ~13 mm tall at the pivot. Two mirror-image wooden halves pivot against
#   each other around a flat bent leaf-spring strip (replacing the coiled
#   torsion spring of the parent variant).
# - Pose: the peg LIES FLAT on its side on the ground (Z-up). The peg's long
#   axis runs along world X, the two halves sit side by side along world Y
#   (lower_half at +Y, upper_half at -Y), and the peg width spans world Z
#   from 0 (ground) to HALF_W.
# - Root/support: lower_half is the fixed root and rests flat on the ground.
#   It carries the leaf spring (bend seated in the carved pivot recess, arms
#   lying along the relieved inner tail faces of both halves) and the upper
#   half pivots on the shared fulcrum axis (vertical in world).
# - Parts: lower_half (root wooden leg), upper_half (moving wooden leg),
#   leaf_spring (one-piece flat bent steel strip: central bend + two arms
#   lying against the relieved inner tail faces of the two halves).
# - Articulation: pivot, REVOLUTE. The joint child frame is the "peg frame"
#   (long axis X, halves stacked along peg-frame Z); axis (0,-1,0) in that
#   frame so positive q opens the gripping jaws while the back finger tails
#   squeeze toward each other. q=0 is the closed rest pose.
# - Spring: the bend is centered exactly on the pivot axis (captured in the
#   rectangular seats carved into both halves); its two arms run back along
#   the relieved inner faces of the two tails, pressing them apart and
#   thereby keeping the jaws closed.
# - Intentional overlaps: the steel leaf spring arms nest inside the carved
#   wooden pivot seats of both halves (captured spring).
# - Tests: both wooden halves + leaf spring present; pivot is revolute about
#   the peg-frame -Y (world vertical) axis; opening swings the upper nose
#   away from the lower nose; closed jaws nearly touch without penetration;
#   the peg rests on the ground (z_min ~ 0); leaf spring extends backward
#   from pivot proving flat-strip geometry.
# - Assumptions: generic flat-leg clothespin proportions, weathered wood +
#   spring-steel leaf spring matching the reference category.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Dimensions (meters)
# ---------------------------------------------------------------------------
LEG_LEN = 0.072            # full peg length along X
HALF_W = 0.0085            # width of one wooden leg (peg-frame Y / world Z)
GAP = 0.001                # closed gap between the two flat jaw faces

PIVOT_X = -0.004           # X of the pivot/fulcrum (slightly back of center)

NOSE_X = LEG_LEN / 2.0     # front gripping tip
BACK_X = -LEG_LEN / 2.0    # back finger end

# Tail relief: behind the pivot the inner face of each half ramps away from
# the parting plane so the finger tails are splayed apart at rest and can be
# squeezed together to open the jaws (like a real peg).
TAIL_ANGLE = 0.08                                    # rad, relief per half
TAIL_RISE = (PIVOT_X - BACK_X) * math.tan(TAIL_ANGLE)  # ~2.6 mm at the back

# Leaf spring strip
STRIP_T = 0.0005           # strip thickness (0.5 mm)
STRIP_W = 0.006            # strip width along peg Y (6 mm)
ARM_LEN = 0.024            # arm length from pivot centre backward

# Rectangular seat carved into each half (replaces the round barrel seat)
SEAT_L = 0.006             # 6 mm along X (covers pivot + arm start)
SEAT_W = HALF_W + 0.004    # through-width cut (same strategy as parent)
SEAT_D = 0.003             # 3 mm deep into the body from the parting face

# Joint range: tails collide when
#   sin(q) ~ 2*tan(TAIL_ANGLE) + GAP/(PIVOT_X-BACK_X) ~ 0.175  (q ~ 0.176 rad)
# Keep the upper limit below that with margin.
PIVOT_MAX = 0.10           # rad, max jaw opening (~4 mm extra at the nose)


def _wood_half() -> cq.Workplane:
    """Build one wooden clothespin leg as a CadQuery solid.

    Authored in the local part frame: long axis +X, width +/-Y, the parting
    (gripping) face on z=0 from the pivot forward, body growing toward +Z.
    Behind the pivot the bottom face ramps up by TAIL_ANGLE (tail relief).
    """
    # Side silhouette in the XZ plane (z measured up from the parting plane).
    z_back = 0.0095
    z_pivot = 0.0115
    z_mid = 0.0070
    z_nose = 0.0042

    pts = [
        (BACK_X, TAIL_RISE),
        (PIVOT_X, 0.0),
        (NOSE_X, 0.0),
        (NOSE_X, z_nose),
        (NOSE_X - 0.010, z_nose + 0.0006),
        (PIVOT_X + 0.012, z_mid),
        (PIVOT_X, z_pivot),
        (PIVOT_X - 0.010, z_mid + 0.0010),
        (BACK_X + 0.008, z_back + 0.0010),
        (BACK_X, z_back),
    ]

    leg = (
        cq.Workplane("XZ")
        .polyline(pts)
        .close()
        .extrude(HALF_W)
    )
    leg = leg.translate((0.0, HALF_W / 2.0, 0.0))
    leg = leg.edges("|Y").fillet(0.0010)

    # --- Clothesline groove behind the gripping tip ---
    groove_w = HALF_W + 0.002
    groove = (
        cq.Workplane("XZ")
        .center(NOSE_X - 0.009, 0.0)
        .circle(0.0022)
        .extrude(groove_w)
        .translate((0.0, groove_w / 2.0, 0.0))
    )
    leg = leg.cut(groove)

    # --- Rectangular leaf-spring seat carved at the pivot ---
    # A rectangular recess cut into the parting face, centred on the pivot
    # axis, so the leaf-spring bend and the start of each arm nest inside
    # the wood of both halves.
    seat = (
        cq.Workplane("XY")
        .box(SEAT_L, SEAT_W, SEAT_D)
        .translate((PIVOT_X - SEAT_L / 4.0, 0.0, SEAT_D / 2.0))
    )
    leg = leg.cut(seat)

    return leg


def _leaf_spring_arm(sign: float) -> cq.Workplane:
    """Build one flat leaf-spring arm in the peg frame.

    *sign* = +1 for the upper arm (tilted +TAIL_ANGLE, at +GAP/2),
             −1 for the lower arm (tilted −TAIL_ANGLE, at −GAP/2).
    """
    arm = cq.Workplane("XY").box(ARM_LEN, STRIP_W, STRIP_T)
    # Tilt the arm to follow the tail relief.
    arm = arm.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0),
                     sign * math.degrees(TAIL_ANGLE))
    # Centre the arm along its length, backward from the pivot.
    cx = PIVOT_X - (ARM_LEN / 2.0) * math.cos(TAIL_ANGLE)
    cz = sign * (GAP / 2.0 + (ARM_LEN / 2.0) * math.sin(TAIL_ANGLE))
    arm = arm.translate((cx, 0.0, cz))
    return arm


def _leaf_spring_bend() -> cq.Workplane:
    """The short bend piece connecting the two arms at the pivot, peg frame."""
    bend_h = GAP + STRIP_T  # spans from lower arm to upper arm with overlap
    bend = cq.Workplane("XY").box(STRIP_T, STRIP_W, bend_h)
    bend = bend.translate((PIVOT_X, 0.0, 0.0))
    return bend


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wooden_clothes_peg")

    wood = model.material("worn_wood", rgba=(0.42, 0.29, 0.17, 1.0))
    spring_steel = model.material("spring_steel", rgba=(0.35, 0.38, 0.42, 1.0))

    half_solid = _wood_half()

    # World placement: the peg lies flat on its side. The "peg frame" (long
    # axis X, halves stacked along peg Z) is rotated by roll=+90 deg into the
    # world and lifted by HALF_W/2 so the flat side face rests on z=0:
    #   world = Trans(0,0,HALF_W/2) * Rx(+90) * peg
    # Peg-frame +Z maps to world -Y; peg-frame +/-Y (width) maps to world Z.

    # --- Lower half (root, fixed, at world +Y). ---
    lower = model.part("lower_half")
    lower.visual(
        mesh_from_cadquery(half_solid, "lower_half"),
        origin=Origin(
            xyz=(0.0, GAP / 2.0, HALF_W / 2.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=wood,
        name="lower_half",
    )

    # --- Pivot: upper half rotates about the fulcrum axis. ---
    upper = model.part("upper_half")
    upper.visual(
        mesh_from_cadquery(half_solid, "upper_half"),
        origin=Origin(xyz=(-PIVOT_X, 0.0, GAP / 2.0), rpy=(0.0, 0.0, 0.0)),
        material=wood,
        name="upper_half",
    )

    # --- Leaf spring: rides with the lower/root half at the pivot. ---
    # Authored in the peg frame with the bend centred on the pivot axis;
    # mapped into the world with the same roll=+90 deg + lift transform.
    spring = model.part("leaf_spring")

    # Repeated sub-parts: the two spring arms, emitted via a for-loop with
    # a shared geometry helper, regular (symmetric) placement, and a uniform
    # visual-on-part joint policy.
    spring_visual_origin = Origin(
        xyz=(0.0, 0.0, HALF_W / 2.0),
        rpy=(math.pi / 2.0, 0.0, 0.0),
    )
    for i in range(2):
        sign = 1.0 if i == 1 else -1.0
        arm_cq = _leaf_spring_arm(sign)
        spring.visual(
            mesh_from_cadquery(arm_cq, f"arm_{i}"),
            origin=spring_visual_origin,
            material=spring_steel,
            name=f"arm_{i}",
        )

    # Bend connecting the two arms at the pivot.
    bend_cq = _leaf_spring_bend()
    spring.visual(
        mesh_from_cadquery(bend_cq, "bend"),
        origin=spring_visual_origin,
        material=spring_steel,
        name="bend",
    )

    # Rigidly mount the leaf spring to the lower half (its bend is captured
    # in the carved pivot seat and its lower arm lies on the lower tail face).
    model.articulation(
        "lower_to_spring",
        ArticulationType.FIXED,
        parent=lower,
        child=spring,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=upper,
        origin=Origin(
            xyz=(PIVOT_X, 0.0, HALF_W / 2.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=4.0, lower=0.0, upper=PIVOT_MAX
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower = object_model.get_part("lower_half")
    upper = object_model.get_part("upper_half")
    spring = object_model.get_part("leaf_spring")
    pivot = object_model.get_articulation("pivot")
    fixed = object_model.get_articulation("lower_to_spring")

    # The leaf spring intentionally nests inside the carved pivot seats of
    # both halves (it is a captured flat spring).
    ctx.allow_overlap(
        spring,
        lower,
        reason="Leaf spring arms are captured/seated inside the lower half pivot recess.",
    )
    ctx.allow_overlap(
        spring,
        upper,
        reason="Leaf spring arms are captured/seated inside the upper half pivot recess.",
    )

    # --- Joint topology / type ---
    ctx.check(
        "pivot is revolute",
        pivot.joint_type == "revolute",
        details=f"joint_type={pivot.joint_type}",
    )
    ctx.check(
        "pivot axis is -Y of the joint frame (fulcrum, world vertical)",
        tuple(round(a, 6) for a in pivot.axis) == (0.0, -1.0, 0.0),
        details=f"axis={pivot.axis}",
    )
    ctx.check(
        "spring is fixed to lower half",
        fixed.joint_type == "fixed",
        details=f"joint_type={fixed.joint_type}",
    )

    # --- Leaf spring present and shaped correctly ---
    spring_box = ctx.part_world_aabb(spring)
    ctx.check(
        "leaf spring located at the pivot",
        spring_box is not None
        and spring_box[0][0] < PIVOT_X
        and spring_box[1][0] >= PIVOT_X,
        details=f"spring_aabb={spring_box}, pivot_x={PIVOT_X}",
    )
    # The spring straddles the parting mid-plane (world y=0) between the halves.
    ctx.check(
        "leaf spring straddles the parting plane",
        spring_box is not None
        and spring_box[0][1] < -0.0005
        and spring_box[1][1] > 0.0005,
        details=f"spring_y=({spring_box[0][1]:.4f},{spring_box[1][1]:.4f})",
    )
    # Key leaf-spring claim: the spring extends well backward from the pivot
    # along -X, proving the flat-strip arms (not a compact coil).
    ctx.check(
        "leaf spring arms extend backward from pivot (flat strip, not coil)",
        spring_box is not None
        and (PIVOT_X - spring_box[0][0]) > ARM_LEN * 0.7,
        details=(
            f"backward_extent={PIVOT_X - spring_box[0][0]:.4f}, "
            f"expected>{ARM_LEN * 0.7:.4f}"
        ),
    )
    # Spring visuals include the two named arms and the bend.
    spring_visual_names = {v.name for v in spring.visuals}
    ctx.check(
        "leaf spring has arm_0, arm_1, and bend visuals",
        {"arm_0", "arm_1", "bend"}.issubset(spring_visual_names),
        details=f"visuals={spring_visual_names}",
    )

    # --- Both wooden legs span the peg length ---
    lower_box = ctx.part_world_aabb(lower)
    upper_box = ctx.part_world_aabb(upper)
    lower_len = None if lower_box is None else lower_box[1][0] - lower_box[0][0]
    upper_len = None if upper_box is None else upper_box[1][0] - upper_box[0][0]
    ctx.check(
        "lower leg spans the peg length",
        lower_len is not None and lower_len > LEG_LEN * 0.9,
        details=f"len_x={lower_len}",
    )
    ctx.check(
        "upper leg spans the peg length",
        upper_len is not None and upper_len > LEG_LEN * 0.9,
        details=f"len_x={upper_len}",
    )

    # The peg lies flat and rests on the ground plane (z ~ 0).
    ctx.check(
        "peg rests on the ground",
        lower_box is not None and abs(lower_box[0][2]) < 0.0005,
        details=f"lower_zmin={lower_box[0][2]:.5f}",
    )

    # --- Closed rest pose: mirror halves nearly touch at the jaws. ---
    with ctx.pose({pivot: 0.0}):
        lb = ctx.part_world_aabb(lower)
        ub = ctx.part_world_aabb(upper)
        ctx.check(
            "closed: lower half at +Y, upper half at -Y of the parting plane",
            lb is not None and ub is not None
            and lb[1][1] > 0.002 and ub[0][1] < -0.002,
            details=f"lower_ymax={lb[1][1]:.4f}, upper_ymin={ub[0][1]:.4f}",
        )
        ctx.expect_gap(
            lower,
            upper,
            axis="y",
            max_gap=0.005,
            max_penetration=0.0002,
            name="closed jaws nearly touch at parting plane",
        )

    # --- Opened pose: actuating the pivot swings the upper nose away. ---
    with ctx.pose({pivot: 0.0}):
        upper_closed = ctx.part_world_aabb(upper)
    with ctx.pose({pivot: PIVOT_MAX}):
        upper_open = ctx.part_world_aabb(upper)
        # The Y-projection gap shrinks because the upper tail swings toward
        # +Y (the 1-D projection overlaps even though the 3-D parts clear
        # each other thanks to the tail relief).  Allow the projection
        # penetration that results from the tail swing.
        ctx.expect_gap(
            lower,
            upper,
            axis="y",
            max_gap=0.02,
            max_penetration=0.003,
            name="fully open halves clear in Y projection",
        )
    # Prove the jaw actually opens: the upper half's Y extent must grow
    # (the nose swings away from the lower half while the tail swings
    # toward it).  At least 3 mm growth proves meaningful opening.
    closed_extent = (upper_closed[1][1] - upper_closed[0][1]
                     if upper_closed else 0.0)
    open_extent = (upper_open[1][1] - upper_open[0][1]
                   if upper_open else 0.0)
    ctx.check(
        "opening the pivot increases upper Y extent (jaw opens)",
        upper_closed is not None and upper_open is not None
        and open_extent > closed_extent + 0.003,
        details=(
            f"closed_extent={closed_extent:.4f}, "
            f"open_extent={open_extent:.4f}"
        ),
    )

    # Lower leg reaches the gripping nose tip.
    nose_tip = ctx.part_element_world_aabb(lower, elem="lower_half")
    lower_max_x = None if nose_tip is None else nose_tip[1][0]
    ctx.check(
        "lower leg reaches the gripping nose",
        lower_max_x is not None and lower_max_x >= NOSE_X - 0.001,
        details=f"lower_max_x={lower_max_x}",
    )

    return ctx.report()


object_model = build_object_model()
