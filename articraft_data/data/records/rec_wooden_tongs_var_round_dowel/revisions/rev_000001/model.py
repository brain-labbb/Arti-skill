from __future__ import annotations

# Wooden serving / cooking tongs (spring tongs) — round dowel arm variant.
#
# Real object: two long, round-dowel wooden arms joined at a narrow top end
# by a small metal spring clip. The arms splay apart into a "V" and end in
# slightly wider rounded paddle tips that grip food. Squeezing the arms pivots
# them about the metal joint at the top, closing the tips together.
#
# Arm cross-section: ROUND DOWEL (cylindrical lathe-profile) along the full
# arm length, tapering gently from the pivot to a wider paddle grip section.
#
# Articulation: a REVOLUTE pivot at the metal joint. Arm 0 + joint cap
# is the fixed root; arm 1 rotates about the pivot axis (normal to the
# splay plane) so positive motion opens the tips apart and the resting
# (relaxed spring) pose holds them open.

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
    mesh_from_geometry,
    section_loft,
)

# ---------------------------------------------------------------------------
# Real-world dimensions (meters)
# ---------------------------------------------------------------------------
ARM_LENGTH = 0.300          # pivot to paddle tip, along the arm centerline
TIP_LENGTH = 0.045          # length of the wider paddle section

# Round dowel radii at key arm stations.
DOWEL_ROOT_R = 0.005        # 10 mm diameter at the pivot end
DOWEL_SHAFT_R = 0.006       # 12 mm diameter through the mid-shaft
DOWEL_TIP_R = 0.008         # 16 mm diameter at the wider paddle grip

# Splay: each arm tilts away from the centerline by this half-angle when the
# spring is relaxed (resting open pose).
HALF_SPLAY = math.radians(8.5)

# Metal spring clip at the pivot.
CLIP_LEN = 0.022            # along the arms (X)
CLIP_WIDTH = 0.014          # across (Y)

WOOD = Material(name="wood", rgba=(0.52, 0.36, 0.20, 1.0))
WOOD_DARK = Material(name="wood_dark", rgba=(0.42, 0.28, 0.15, 1.0))
METAL = Material(name="clip_metal", rgba=(0.72, 0.74, 0.78, 1.0))


# ---------------------------------------------------------------------------
# Shared geometry helpers
# ---------------------------------------------------------------------------

def _circle_section(
    x: float, radius: float, segments: int = 20
) -> list[tuple[float, float, float]]:
    """One closed circular cross-section in the YZ plane at position x."""
    return [
        (
            x,
            radius * math.cos(i * 2.0 * math.pi / segments),
            radius * math.sin(i * 2.0 * math.pi / segments),
        )
        for i in range(segments)
    ]


def _arm_dowel():
    """One round-dowel tapered wooden arm, authored along +X from x=0 (pivot)
    to x=ARM_LENGTH (paddle tip).

    Returns a MeshGeometry from section_loft through circular cross-sections.
    Both arms share this identical helper.
    """
    shaft_end = ARM_LENGTH - TIP_LENGTH

    # (x_position, radius) — tapered round dowel profile
    profile = [
        (0.0, DOWEL_ROOT_R),                       # pivot end
        (0.020, DOWEL_ROOT_R + 0.001),             # slight widening
        (0.090, DOWEL_SHAFT_R),                    # mid-shaft
        (shaft_end - 0.020, DOWEL_SHAFT_R),        # maintain shaft
        (shaft_end, DOWEL_SHAFT_R + 0.002),        # start paddle flare
        (shaft_end + 0.015, DOWEL_TIP_R),          # paddle grip
        (ARM_LENGTH - 0.015, DOWEL_TIP_R),         # maintain paddle
        (ARM_LENGTH - 0.005, DOWEL_SHAFT_R + 0.001),  # taper back
        (ARM_LENGTH, DOWEL_ROOT_R),                # rounded tip end
    ]

    sections = [_circle_section(x, r) for x, r in profile]
    return section_loft(sections)


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


# ---------------------------------------------------------------------------
# Model construction
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wooden_tongs")
    model.materials.extend([WOOD, WOOD_DARK, METAL])

    # Stacking: the two round dowel arms are stacked in Z at the pivot. Each
    # arm center is offset from the midplane by arm_stack so the dowels sit
    # side-by-side with a small gap between surfaces.
    arm_stack = DOWEL_ROOT_R + 0.001   # center offset of each arm in Z
    clip_span = 2.0 * (arm_stack + DOWEL_ROOT_R) + 0.002  # total Z clip extent

    # Shared arm mesh — both arms use the same round-dowel geometry.
    arm_mesh = mesh_from_geometry(_arm_dowel(), "arm_dowel")

    # Materials for each arm (visual variety).
    arm_materials = [WOOD, WOOD_DARK]

    # ----- Create both arm parts via loop (shared geometry, regular placement) -----
    arm_names = ("arm_0", "arm_1")
    arm_visual_names = ("arm_0_dowel", "arm_1_dowel")
    arm_parts = []
    for i in range(2):
        arm_part = model.part(arm_names[i])

        if i == 0:
            # Root arm: splayed toward +Y, sits below centerline (-Z).
            arm_part.visual(
                arm_mesh,
                origin=Origin(xyz=(0.0, 0.0, -arm_stack), rpy=(0.0, 0.0, HALF_SPLAY)),
                material=arm_materials[i],
                name=arm_visual_names[i],
            )
            # Metal spring clip is rigid with the root and bridges both arms
            # at the pivot. It is the visible structure connecting the two arms.
            arm_part.visual(
                mesh_from_cadquery(_clip_solid(clip_span), "clip"),
                origin=Origin(xyz=(0.0, 0.0, 0.0)),
                material=METAL,
                name="spring_clip",
            )
        else:
            # Moving arm: authored straight along +X from the pivot; the joint
            # frame applies the splay and stacking offset.
            arm_part.visual(
                arm_mesh,
                origin=Origin(xyz=(0.0, 0.0, 0.0)),
                material=arm_materials[i],
                name=arm_visual_names[i],
            )

        arm_parts.append(arm_part)

    # ----- Revolute pivot at the metal clip -----
    # Pivot axis is +Z (normal to the splay plane of the arms). The joint
    # origin tilts arm_1 by -HALF_SPLAY so at q=0 the two arms already form
    # the relaxed open "V" (resting spring pose, tips apart). Positive q
    # (squeeze) rotates arm_1 toward arm_0 and closes the tips together;
    # small negative q opens the V a little wider.
    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=arm_parts[0],
        child=arm_parts[1],
        origin=Origin(xyz=(0.0, 0.0, arm_stack), rpy=(0.0, 0.0, -HALF_SPLAY)),
        axis=(0.0, 0.0, 1.0),
        # q=0 is the relaxed open rest; upper bound is the fully squeezed/closed
        # pose; small negative travel opens slightly wider than rest.
        motion_limits=MotionLimits(
            effort=2.0, velocity=3.0, lower=-0.04, upper=0.18
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def _tip_y(ctx: TestContext, part_obj) -> float:
    """World Y of the paddle tip (max-X end) of an arm part, current pose."""
    lo, hi = ctx.part_world_aabb(part_obj)
    return (lo[1] + hi[1]) / 2.0


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    arm_0 = object_model.get_part("arm_0")
    arm_1 = object_model.get_part("arm_1")
    pivot = object_model.get_articulation("pivot")

    # --- Joint contract: revolute about the splay-plane normal (Z) ---
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

    # --- Hero geometry: two long round-dowel wooden arms ---
    lo0, hi0 = ctx.part_world_aabb(arm_0)
    lo1, hi1 = ctx.part_world_aabb(arm_1)
    arm0_len = hi0[0] - lo0[0]
    arm1_len = hi1[0] - lo1[0]
    ctx.check(
        "arm_0 is a long strip (~0.30 m)",
        0.27 < arm0_len < 0.33,
        details=f"len={arm0_len:.3f}",
    )
    ctx.check(
        "arm_1 is a long strip (~0.30 m)",
        0.27 < arm1_len < 0.33,
        details=f"len={arm1_len:.3f}",
    )

    # Round dowel cross-section: check the dowel element (not the whole part
    # which includes the clip). The Z extent should reflect the dowel diameter
    # (>= 2*DOWEL_ROOT_R = 10mm), NOT a thin flat strip (< 5mm).
    d0_lo, d0_hi = ctx.part_element_world_aabb(arm_0, elem="arm_0_dowel")
    arm0_z = d0_hi[2] - d0_lo[2]
    ctx.check(
        "arm_0 has round dowel cross-section (Z extent > flat strip thickness)",
        arm0_z > 2.0 * DOWEL_ROOT_R * 0.9,
        details=f"z_extent={arm0_z:.4f}, expected>={2.0*DOWEL_ROOT_R*0.9:.4f}",
    )
    # The dowel Z extent should be consistent with the max dowel diameter,
    # confirming it is round (not flat where Z << Y width).
    ctx.check(
        "arm_0 dowel Z extent matches dowel diameter range",
        0.008 < arm0_z < 0.025,
        details=f"z_extent={arm0_z:.4f}",
    )

    # --- Metal spring clip present at the pivot, bridging both arms ---
    clo, chi = ctx.part_element_world_aabb(arm_0, elem="spring_clip")
    ctx.check(
        "spring clip sits at the pivot end (near x=0)",
        clo[0] < 0.022 and chi[0] < 0.030,
        details=f"clip x=[{clo[0]:.3f},{chi[0]:.3f}]",
    )
    # Clip spans across both stacked arms in Z (connects them).
    ctx.check(
        "spring clip bridges both stacked arms in Z",
        (chi[2] - clo[2]) > 2.0 * DOWEL_ROOT_R,
        details=f"clip z-span={(chi[2]-clo[2]):.4f}",
    )

    # --- Arms splay apart at rest (V shape): tips on opposite sides ---
    ctx.check(
        "arms splay to opposite sides at the tips",
        _tip_y(ctx, arm_0) > 0.005 and _tip_y(ctx, arm_1) < -0.005,
        details=f"arm_0_tip_y={_tip_y(ctx, arm_0):.3f}, "
        f"arm_1_tip_y={_tip_y(ctx, arm_1):.3f}",
    )

    # --- Pivot mechanism actually opens/closes the tips ---
    # q=0 is the relaxed open rest; upper limit squeezes the tips closed; small
    # negative travel opens slightly wider than rest.
    rest_gap = _tip_y(ctx, arm_0) - _tip_y(ctx, arm_1)
    with ctx.pose({pivot: pivot.motion_limits.upper}):
        closed_gap = _tip_y(ctx, arm_0) - _tip_y(ctx, arm_1)
    with ctx.pose({pivot: pivot.motion_limits.lower}):
        open_gap = _tip_y(ctx, arm_0) - _tip_y(ctx, arm_1)
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
        arm_0, arm_1, axes="xy", max_dist=0.015,
        name="arms share the pivot origin",
    )

    # --- Spring clip intentionally bridges both dowel arms at the pivot ---
    # The clip wraps around both stacked dowels to hold them together; this
    # local overlap is the visible connection between the two arms.
    ctx.allow_overlap(
        "arm_0", "arm_1",
        elem_a="spring_clip",
        elem_b="arm_1_dowel",
        reason="The spring clip intentionally bridges and wraps around both dowel arms at the pivot to connect them.",
    )
    ctx.expect_contact(
        arm_0, arm_1,
        elem_a="spring_clip", elem_b="arm_1_dowel",
        contact_tol=0.005,
        name="clip contacts arm_1 dowel at pivot",
    )

    return ctx.report()


object_model = build_object_model()
