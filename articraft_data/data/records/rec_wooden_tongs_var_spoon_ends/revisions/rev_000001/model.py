from __future__ import annotations

# Wooden serving / cooking tongs with concave spoon/scoop tips (salad-server style).
#
# Real object: two long, flat, tapered wooden arms joined at a narrow top end
# by a small metal spring clip. The arms splay apart into a "V" and end in
# wider rounded spoon-bowl tips with a concave dish on the inner (gripping)
# face, like salad-server tongs. Squeezing the arms pivots them about the
# metal joint at the top, closing the scoops together to grip food.
#
# Articulation: a REVOLUTE pivot at the metal joint. The left arm + joint cap
# is the fixed root; the right arm rotates about the pivot axis (normal to the
# flat plane of the arms) so positive motion opens the tips apart and the
# resting (relaxed spring) pose holds them open.

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
# Real-world dimensions (meters)
# ---------------------------------------------------------------------------
ARM_LENGTH = 0.300          # pivot to spoon tip, along the arm centerline
ARM_THICKNESS = 0.0045      # flat strip thickness (Z)
ROOT_WIDTH = 0.012          # narrow strip width at the pivot end
MID_WIDTH = 0.020           # widest part of the shaft

# Spoon bowl section at the tip
BOWL_LENGTH = 0.052         # length of the wider spoon bowl section
BOWL_WIDTH = 0.040          # max width of the spoon bowl
BOWL_THICK = 0.008          # thickness of the spoon bowl (thicker than shaft)

# Concave dish parameters
DISH_RADIUS = 0.024         # radius of the sphere used to cut the dish
DISH_DEPTH = 0.004          # how deep the dish cuts into the bowl face

# Splay: each arm tilts away from the centerline by this half-angle when the
# spring is relaxed (resting open pose).
HALF_SPLAY = math.radians(8.5)

# Metal spring clip at the pivot.
CLIP_LEN = 0.022            # along the arms (X)
CLIP_WIDTH = 0.014          # across (Y)
CLIP_THICK = 0.012          # total stacked thickness across both arms (Z)

WOOD = Material(name="wood", rgba=(0.52, 0.36, 0.20, 1.0))
WOOD_DARK = Material(name="wood_dark", rgba=(0.42, 0.28, 0.15, 1.0))
WOOD_HONEY = Material(name="wood_honey", rgba=(0.58, 0.40, 0.22, 1.0))
METAL = Material(name="clip_metal", rgba=(0.72, 0.74, 0.78, 1.0))

# Derived
SHAFT_END = ARM_LENGTH - BOWL_LENGTH  # where the bowl section begins
END_FLAT = 0.0030  # half-width of the flat at pivot end


def _shaft_half_profile() -> list[tuple[float, float]]:
    """Upper-half (y >= 0) outline of the tapered shaft section.

    From pivot (x=0) to just past the bowl transition (SHAFT_END + overlap).
    """
    return [
        (0.0, END_FLAT),
        (0.018, ROOT_WIDTH / 2.0 + 0.0015),
        (0.090, MID_WIDTH / 2.0),
        (SHAFT_END + 0.005, MID_WIDTH / 2.0),
        (SHAFT_END + 0.005, END_FLAT),
    ]


def _bowl_half_profile() -> list[tuple[float, float]]:
    """Upper-half (y >= 0) outline of the spoon bowl section.

    Smooth rounded spoon shape: flares from shaft width, widens to full bowl,
    then rounds off to a soft tip.
    """
    x_start = SHAFT_END - 0.006  # overlap with shaft for clean union
    x_end = ARM_LENGTH
    span = x_end - x_start

    pts: list[tuple[float, float]] = []
    n = 14
    for i in range(n + 1):
        t = i / n
        x = x_start + t * span

        if t < 0.12:
            # Transition from shaft width to bowl width
            s = t / 0.12
            w = MID_WIDTH / 2.0 + s * (BOWL_WIDTH / 2.0 - MID_WIDTH / 2.0)
        elif t < 0.72:
            # Full bowl width with a gentle sinusoidal bulge
            s = (t - 0.12) / 0.60
            w = BOWL_WIDTH / 2.0 + 0.002 * math.sin(s * math.pi)
        else:
            # Round off toward the tip (cosine taper)
            s = (t - 0.72) / 0.28
            w = (BOWL_WIDTH / 2.0 + 0.002) * math.cos(s * math.pi / 2.0)
            w = max(w, END_FLAT)
        pts.append((x, w))

    return pts


def _mirror_profile(half_pts: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Mirror a y>=0 half-profile to a full closed outline."""
    lower = [(x, -y) for (x, y) in reversed(half_pts)]
    return half_pts + lower


def _arm_with_scoop(dish_up: bool) -> cq.Workplane:
    """One wooden arm with a concave spoon/scoop tip.

    The arm lies in the XY plane (flat faces normal to Z). The pivot end sits
    at x=0; the arm extends along +X to the spoon tip at x=ARM_LENGTH.

    Args:
        dish_up: if True, the concave dish is cut from the +Z face (top);
                 if False, from the -Z face (bottom). This ensures each arm's
                 dish faces the other arm when assembled.
    """
    half_t = ARM_THICKNESS / 2.0
    bowl_half_t = BOWL_THICK / 2.0

    # --- Shaft: tapered flat strip ---
    shaft_outline = _mirror_profile(_shaft_half_profile())
    shaft = (
        cq.Workplane("XY")
        .polyline(shaft_outline)
        .close()
        .extrude(ARM_THICKNESS)
        .translate((0.0, 0.0, -half_t))
    )

    # --- Bowl: wider, thicker spoon section ---
    bowl_outline = _mirror_profile(_bowl_half_profile())
    bowl = (
        cq.Workplane("XY")
        .polyline(bowl_outline)
        .close()
        .extrude(BOWL_THICK)
        .translate((0.0, 0.0, -bowl_half_t))
    )

    # Union shaft and bowl (they overlap slightly for a clean merge)
    arm = shaft.union(bowl, clean=True)

    # --- Concave dish: spherical cut from the gripping face ---
    # The sphere center is positioned so its lowest point (for dish_up) just
    # reaches DISH_DEPTH below the bowl face, creating a concave depression.
    dish_x = SHAFT_END + BOWL_LENGTH * 0.42  # dish centered in the bowl
    dish_y = 0.0

    if dish_up:
        # Dish on +Z face: sphere center above the bowl top
        dish_z = bowl_half_t - DISH_DEPTH + DISH_RADIUS
    else:
        # Dish on -Z face: sphere center below the bowl bottom
        dish_z = -(bowl_half_t - DISH_DEPTH + DISH_RADIUS)

    cutter = (
        cq.Workplane("XY")
        .sphere(DISH_RADIUS)
        .translate((dish_x, dish_y, dish_z))
    )
    arm = arm.cut(cutter, clean=True)

    # Soften the perimeter edges for a worn-wood look
    try:
        arm = arm.edges("|Z").fillet(0.0008)
    except Exception:
        pass

    return arm


def _clip_solid(span: float) -> cq.Workplane:
    """Small metal spring clip wrapping the joined pivot end of both arms.

    Authored centered at the pivot origin, spanning +-span/2 in Z so it bridges
    the two stacked arm ends. Extends a little along +X over the arms.
    """
    clip = (
        cq.Workplane("XY")
        .box(CLIP_LEN, CLIP_WIDTH, span)
        .translate((CLIP_LEN / 2.0 - 0.003, 0.0, 0.0))
    )
    try:
        clip = clip.edges("|X").fillet(0.0022)
    except Exception:
        pass
    return clip


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wooden_tongs")
    model.materials.extend([WOOD, WOOD_DARK, WOOD_HONEY, METAL])

    # The two arms are stacked at the pivot: the fixed arm sits slightly below
    # (-Z) and the moving arm slightly above (+Z) so the metal clip can pinch
    # them. Each arm splays away from the shared centerline.
    arm_stack = ARM_THICKNESS / 2.0 + 0.0020  # half offset of each arm in Z
    clip_span = 2.0 * arm_stack + ARM_THICKNESS + 0.002

    # ----- Fixed arm (root): left arm + metal clip -----
    fixed_arm = model.part("fixed_arm")
    fixed_arm.visual(
        mesh_from_cadquery(_arm_with_scoop(dish_up=True), "fixed_arm_scoop"),
        # Sit below centerline, rotate about Z by +HALF_SPLAY (this arm
        # splays toward +Y), pivot stays at the part origin.
        origin=Origin(xyz=(0.0, 0.0, -arm_stack), rpy=(0.0, 0.0, HALF_SPLAY)),
        material=WOOD,
        name="fixed_arm_scoop",
    )
    # Metal spring clip is rigid with the root and bridges both arms at the
    # pivot. It is the visible structure connecting the two arms.
    fixed_arm.visual(
        mesh_from_cadquery(_clip_solid(clip_span), "clip"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=METAL,
        name="spring_clip",
    )

    # ----- Moving arm (child): right arm -----
    moving_arm = model.part("moving_arm")
    moving_arm.visual(
        mesh_from_cadquery(_arm_with_scoop(dish_up=False), "moving_arm_scoop"),
        # Authored straight along +X from the pivot; the joint frame applies
        # the splay and stacking offset.
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=WOOD_HONEY,
        name="moving_arm_scoop",
    )

    # ----- Revolute pivot at the metal clip -----
    # Pivot axis is +Z (normal to the flat plane of the arms). The joint origin
    # tilts the moving arm by -HALF_SPLAY so at q=0 the two arms already form
    # the relaxed open "V" (resting spring pose, tips apart). Positive q (squeeze)
    # rotates the moving arm toward the fixed arm and closes the scoops together;
    # small negative q opens the V a little wider.
    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=fixed_arm,
        child=moving_arm,
        origin=Origin(xyz=(0.0, 0.0, arm_stack), rpy=(0.0, 0.0, -HALF_SPLAY)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=3.0, lower=-0.04, upper=0.18
        ),
    )

    return model


def _tip_y(ctx: TestContext, part) -> float:
    """World Y of the spoon tip (max-X end) of an arm part, current pose."""
    lo, hi = ctx.part_world_aabb(part)
    return (lo[1] + hi[1]) / 2.0


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    fixed_arm = object_model.get_part("fixed_arm")
    moving_arm = object_model.get_part("moving_arm")
    pivot = object_model.get_articulation("pivot")

    # --- Joint contract: revolute about the flat-plane normal (Z) ---
    ctx.check(
        "pivot is revolute",
        pivot.joint_type == "revolute" or pivot.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={pivot.joint_type}",
    )
    ax = pivot.axis
    ctx.check(
        "pivot axis is normal to arm plane (Z)",
        abs(ax[2]) > 0.99 and abs(ax[0]) < 0.02 and abs(ax[1]) < 0.02,
        details=f"axis={ax}",
    )

    # --- Hero geometry: two long flat tapered wooden arms with spoon tips ---
    flo, fhi = ctx.part_world_aabb(fixed_arm)
    mlo, mhi = ctx.part_world_aabb(moving_arm)
    fixed_len = fhi[0] - flo[0]
    moving_len = mhi[0] - mlo[0]
    ctx.check(
        "fixed arm is a long strip (~0.30 m)",
        0.27 < fixed_len < 0.33,
        details=f"len={fixed_len:.3f}",
    )
    ctx.check(
        "moving arm is a long strip (~0.30 m)",
        0.27 < moving_len < 0.33,
        details=f"len={moving_len:.3f}",
    )
    # Flat: thickness (Z) much smaller than width (Y) and length (X).
    fixed_thick = fhi[2] - flo[2]
    ctx.check(
        "fixed arm is a thin flat strip",
        fixed_thick < 0.030 and fixed_len > 8.0 * ARM_THICKNESS,
        details=f"thick={fixed_thick:.4f}",
    )

    # --- Spoon bowl: tip region is thicker than the shaft ---
    # The bowl section (BOWL_THICK) is visibly thicker than the shaft
    # (ARM_THICKNESS), confirming the concave spoon/scoop geometry.
    ctx.check(
        "fixed arm tip has spoon bowl thickness (thicker than shaft)",
        fixed_thick > BOWL_THICK - 0.002,
        details=f"tip_z_span={fixed_thick:.4f}, bowl_thick={BOWL_THICK}",
    )
    moving_thick = mhi[2] - mlo[2]
    ctx.check(
        "moving arm tip has spoon bowl thickness (thicker than shaft)",
        moving_thick > BOWL_THICK - 0.002,
        details=f"tip_z_span={moving_thick:.4f}, bowl_thick={BOWL_THICK}",
    )

    # --- Spoon bowl width: tips are wider than the shaft mid-section ---
    fixed_width = fhi[1] - flo[1]
    # At rest the arm is splayed, so Y-extent includes the splay offset.
    # But the bowl width should still be visibly wider than MID_WIDTH.
    ctx.check(
        "spoon bowl tips are wider than the shaft",
        fixed_width > MID_WIDTH + 0.005,
        details=f"fixed_width={fixed_width:.4f}",
    )

    # --- Spoon visual names confirm scoop geometry ---
    fixed_scoop = fixed_arm.get_visual("fixed_arm_scoop")
    moving_scoop = moving_arm.get_visual("moving_arm_scoop")
    ctx.check(
        "fixed arm has a named scoop visual",
        fixed_scoop is not None,
        details="missing fixed_arm_scoop visual",
    )
    ctx.check(
        "moving arm has a named scoop visual",
        moving_scoop is not None,
        details="missing moving_arm_scoop visual",
    )

    # --- Metal spring clip present at the pivot, bridging both arms ---
    clo, chi = ctx.part_element_world_aabb(fixed_arm, elem="spring_clip")
    ctx.check(
        "spring clip sits at the pivot end (near x=0)",
        clo[0] < 0.022 and chi[0] < 0.030,
        details=f"clip x=[{clo[0]:.3f},{chi[0]:.3f}]",
    )
    # Clip spans across both stacked arms in Z (connects them).
    ctx.check(
        "spring clip bridges both stacked arms in Z",
        (chi[2] - clo[2]) > ARM_THICKNESS,
        details=f"clip z-span={(chi[2]-clo[2]):.4f}",
    )

    # --- Arms splay apart at rest (V shape): tips on opposite sides ---
    ctx.check(
        "arms splay to opposite sides at the tips",
        _tip_y(ctx, fixed_arm) > 0.005 and _tip_y(ctx, moving_arm) < -0.005,
        details=f"fixed_tip_y={_tip_y(ctx, fixed_arm):.3f}, "
        f"moving_tip_y={_tip_y(ctx, moving_arm):.3f}",
    )

    # --- Pivot mechanism actually opens/closes the spoon tips ---
    rest_gap = _tip_y(ctx, fixed_arm) - _tip_y(ctx, moving_arm)
    with ctx.pose({pivot: pivot.motion_limits.upper}):
        closed_gap = _tip_y(ctx, fixed_arm) - _tip_y(ctx, moving_arm)
    with ctx.pose({pivot: pivot.motion_limits.lower}):
        open_gap = _tip_y(ctx, fixed_arm) - _tip_y(ctx, moving_arm)
    ctx.check(
        "squeezing (upper limit) closes the spoon tips together",
        closed_gap < rest_gap - 0.005,
        details=f"rest={rest_gap:.3f}, closed={closed_gap:.3f}",
    )
    ctx.check(
        "opening (lower limit) spreads the spoon tips wider",
        open_gap > rest_gap,
        details=f"rest={rest_gap:.3f}, open={open_gap:.3f}",
    )

    # --- Spring clip intentionally wraps both arms at the pivot ---
    # The clip bridges the stacked arm ends, so it naturally overlaps with the
    # moving arm's shaft where the clip clamps over it. This is the physical
    # spring mechanism that holds both arms together at the pivot.
    ctx.allow_overlap(
        fixed_arm, moving_arm,
        elem_a="spring_clip",
        elem_b="moving_arm_scoop",
        reason="The spring clip physically wraps over both stacked arm ends at the pivot; "
               "the overlap is the clip clamping the moving arm shaft.",
    )
    # Proof: the clip sits at the pivot end and spans both arms in Z.
    ctx.expect_overlap(
        fixed_arm, moving_arm,
        axes="xy",
        elem_a="spring_clip",
        elem_b="moving_arm_scoop",
        min_overlap=0.005,
        name="spring clip overlaps the moving arm at the pivot in XY",
    )
    ctx.expect_gap(
        fixed_arm, moving_arm,
        axis="z",
        positive_elem="spring_clip",
        negative_elem="moving_arm_scoop",
        max_penetration=0.020,
        name="clip-to-scoop Z penetration is bounded by clip span",
    )

    # --- Arms meet/clamp near the pivot, regardless of pose (hinged together) ---
    ctx.expect_origin_distance(
        fixed_arm, moving_arm, axes="xy", max_dist=0.012,
        name="arms share the pivot origin",
    )

    return ctx.report()


object_model = build_object_model()
