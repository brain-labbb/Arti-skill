from __future__ import annotations

# Wooden serving / cooking tongs (spring tongs).
#
# Real object: two long, flat, tapered wooden arms joined at a narrow top end
# by a small metal spring clip. The arms splay apart into a "V" and end in
# slightly wider rounded paddle tips that grip food. Squeezing the arms pivots
# them about the metal joint at the top, closing the tips together.
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
ARM_LENGTH = 0.300          # pivot to paddle tip, along the arm centerline
ARM_THICKNESS = 0.0045      # flat strip thickness (Z)
ROOT_WIDTH = 0.012          # narrow strip width at the pivot end
MID_WIDTH = 0.020           # widest part of the shaft
TIP_WIDTH = 0.030           # rounded paddle tip width
TIP_LENGTH = 0.045          # length of the wider paddle section

# Splay: each arm tilts away from the centerline by this half-angle when the
# spring is relaxed (resting open pose).
HALF_SPLAY = math.radians(8.5)

# Metal spring clip at the pivot.
CLIP_LEN = 0.022            # along the arms (X)
CLIP_WIDTH = 0.014          # across (Y)
CLIP_THICK = 0.012          # total stacked thickness across both arms (Z)

WOOD = Material(name="wood", rgba=(0.52, 0.36, 0.20, 1.0))
WOOD_DARK = Material(name="wood_dark", rgba=(0.42, 0.28, 0.15, 1.0))
METAL = Material(name="clip_metal", rgba=(0.72, 0.74, 0.78, 1.0))


def _arm_solid() -> cq.Workplane:
    """One flat tapered wooden arm authored in its own local frame.

    The arm lies in the XY plane (flat faces normal to Z). The pivot end sits
    at x=0; the arm extends along +X to the paddle tip at x=ARM_LENGTH. The
    strip is centered on its own y=0 centerline; splay is applied later by the
    part/joint frame, so each arm is modeled straight here.
    """
    half_t = ARM_THICKNESS / 2.0

    # Top-half outline (y >= 0) of the tapered strip, swept along +X.
    # Start narrow (but with a small flat end, not a point) at the pivot, widen
    # through the shaft, flare to the paddle, then a small flat at the rounded
    # tip. Keeping both ends as short flats (never a y=0 vertex) keeps the
    # extruded edges well-defined for filleting.
    shaft_end = ARM_LENGTH - TIP_LENGTH
    end_flat = 0.0030  # half-width of the flat at pivot and tip ends
    pts = [
        (0.0, end_flat),
        (0.018, ROOT_WIDTH / 2.0 + 0.0015),
        (0.090, MID_WIDTH / 2.0),
        (shaft_end - 0.020, MID_WIDTH / 2.0),
        (shaft_end, TIP_WIDTH / 2.0 - 0.004),
        (shaft_end + 0.012, TIP_WIDTH / 2.0),
        (ARM_LENGTH - 0.016, TIP_WIDTH / 2.0),
        (ARM_LENGTH - 0.004, TIP_WIDTH / 2.0 - 0.006),
        (ARM_LENGTH, end_flat),
    ]
    # Mirror to the bottom half to form a closed symmetric profile.
    lower = [(x, -y) for (x, y) in reversed(pts)]
    outline = pts + lower

    arm = (
        cq.Workplane("XY")
        .polyline(outline)
        .close()
        .extrude(ARM_THICKNESS)
        .translate((0.0, 0.0, -half_t))
    )
    # Soften the flat top/bottom faces' perimeter so it reads as worn wood.
    try:
        arm = arm.edges("|Z").fillet(0.0010)
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
    model.materials.extend([WOOD, WOOD_DARK, METAL])

    # The two arms are stacked at the pivot: the fixed arm sits slightly below
    # (-Z) and the moving arm slightly above (+Z) so the metal clip can pinch
    # them. Each arm splays away from the shared centerline.
    arm_stack = ARM_THICKNESS / 2.0 + 0.0010  # half offset of each arm in Z
    clip_span = 2.0 * arm_stack + ARM_THICKNESS + 0.002

    # ----- Fixed arm (root): left arm + metal clip -----
    fixed_arm = model.part("fixed_arm")
    fixed_mesh = mesh_from_cadquery(_arm_solid(), "fixed_arm")
    fixed_arm.visual(
        fixed_mesh,
        # Sit below centerline, rotate about Z by +HALF_SPLAY (this arm
        # splays toward +Y), pivot stays at the part origin.
        origin=Origin(xyz=(0.0, 0.0, -arm_stack), rpy=(0.0, 0.0, HALF_SPLAY)),
        material=WOOD,
        name="fixed_arm_strip",
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
    moving_mesh = mesh_from_cadquery(_arm_solid(), "moving_arm")
    moving_arm.visual(
        moving_mesh,
        # Authored straight along +X from the pivot; the joint frame applies
        # the splay and stacking offset.
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=WOOD_DARK,
        name="moving_arm_strip",
    )

    # ----- Revolute pivot at the metal clip -----
    # Pivot axis is +Z (normal to the flat plane of the arms). The joint origin
    # tilts the moving arm by -HALF_SPLAY so at q=0 the two arms already form
    # the relaxed open "V" (resting spring pose, tips apart). Positive q (squeeze)
    # rotates the moving arm toward the fixed arm and closes the tips together;
    # small negative q opens the V a little wider.
    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=fixed_arm,
        child=moving_arm,
        origin=Origin(xyz=(0.0, 0.0, arm_stack), rpy=(0.0, 0.0, -HALF_SPLAY)),
        axis=(0.0, 0.0, 1.0),
        # q=0 is the relaxed open rest; upper bound is the fully squeezed/closed
        # pose; small negative travel opens slightly wider than rest.
        motion_limits=MotionLimits(
            effort=2.0, velocity=3.0, lower=-0.04, upper=0.18
        ),
    )

    return model


def _tip_y(ctx: TestContext, part) -> float:
    """World Y of the paddle tip (max-X end) of an arm part, current pose."""
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

    # --- Hero geometry: two long flat tapered wooden arms ---
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

    # --- Metal spring clip present at the pivot, bridging both arms ---
    clip = fixed_arm.get_visual("spring_clip")
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

    # --- Pivot mechanism actually opens/closes the tips ---
    # q=0 is the relaxed open rest; upper limit squeezes the tips closed; small
    # negative travel opens slightly wider than rest.
    rest_gap = _tip_y(ctx, fixed_arm) - _tip_y(ctx, moving_arm)
    with ctx.pose({pivot: pivot.motion_limits.upper}):
        closed_gap = _tip_y(ctx, fixed_arm) - _tip_y(ctx, moving_arm)
    with ctx.pose({pivot: pivot.motion_limits.lower}):
        open_gap = _tip_y(ctx, fixed_arm) - _tip_y(ctx, moving_arm)
    ctx.check(
        "squeezing (upper limit) closes the tips together",
        closed_gap < rest_gap - 0.005,
        details=f"rest={rest_gap:.3f}, closed={closed_gap:.3f}",
    )
    ctx.check(
        "opening (lower limit) spreads the tips wider",
        open_gap > rest_gap,
        details=f"rest={rest_gap:.3f}, open={open_gap:.3f}",
    )

    # --- Arms meet/clamp near the pivot, regardless of pose (hinged together) ---
    ctx.expect_origin_distance(
        fixed_arm, moving_arm, axes="xy", max_dist=0.012,
        name="arms share the pivot origin",
    )

    return ctx.report()


object_model = build_object_model()
