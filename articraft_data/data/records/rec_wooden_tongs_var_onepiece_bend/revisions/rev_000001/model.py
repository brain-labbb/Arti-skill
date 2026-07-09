from __future__ import annotations

# Wooden serving / cooking tongs — one-piece bent-wood design.
#
# A single continuous U / hairpin of wood where the curved bend at the top IS
# the spring (no separate metal clip). Two flat, tapered wooden arms extend
# from the bend to rounded paddle tips that grip food. Squeezing the arms
# flexes the wooden bend, closing the tips together.
#
# Articulation: a REVOLUTE flex at the bend where the moving arm emerges.
# Axis is normal to the flat plane of the arms (Z). Positive q closes the
# tips together (squeezing); the relaxed rest pose holds the tips open.

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
# bend is relaxed (resting open pose).
HALF_SPLAY = math.radians(8.5)

# Bent-wood U-bend at the top: the wooden spring. The bend radius sets the
# separation between the two arm roots.
BEND_RADIUS = 0.012

WOOD = Material(name="wood", rgba=(0.52, 0.36, 0.20, 1.0))
WOOD_DARK = Material(name="wood_dark", rgba=(0.42, 0.28, 0.15, 1.0))


# ---------------------------------------------------------------------------
# Shared geometry helpers
# ---------------------------------------------------------------------------
def _arm_solid() -> cq.Workplane:
    """One flat tapered wooden arm authored in its own local frame.

    The arm lies in the XY plane (flat faces normal to Z). The pivot end sits
    at x=0; the arm extends along +X to the paddle tip at x=ARM_LENGTH. The
    strip is centered on its own y=0 centerline; splay is applied later by the
    part/joint frame, so each arm is modeled straight here.
    """
    half_t = ARM_THICKNESS / 2.0

    shaft_end = ARM_LENGTH - TIP_LENGTH
    end_flat = 0.0030
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
    lower = [(x, -y) for (x, y) in reversed(pts)]
    outline = pts + lower

    arm = (
        cq.Workplane("XY")
        .polyline(outline)
        .close()
        .extrude(ARM_THICKNESS)
        .translate((0.0, 0.0, -half_t))
    )
    try:
        arm = arm.edges("|Z").fillet(0.0010)
    except Exception:
        pass
    return arm


def _bend_solid() -> cq.Workplane:
    """U-shaped wooden bend (half-torus with rectangular cross-section).

    Authored centered at the origin. The bend curves in the XZ plane from
    (0, 0, -BEND_RADIUS) through (-BEND_RADIUS, 0, 0) to (0, 0, +BEND_RADIUS).
    Cross-section: ROOT_WIDTH in Y (arm width) x ARM_THICKNESS radially.

    Constructed by revolving a rectangular profile 180 degrees around the
    world Y axis on the YZ workplane. Profile vertices are placed explicitly
    (no .center()) to avoid coordinate-system mismatch with the revolve axis.
    """
    R = BEND_RADIUS
    hw = ROOT_WIDTH / 2.0
    ht = ARM_THICKNESS / 2.0

    # On the YZ workplane: local X = world Y, local Y = world Z.
    # Profile centered at (local X=0, local Y=-R) = world (Y=0, Z=-R).
    pts = [
        (-hw, -R - ht),
        (hw, -R - ht),
        (hw, -R + ht),
        (-hw, -R + ht),
    ]
    # Revolve axis from (0,0) to (1,0) is along local X = world Y at Z=0,
    # in the +Y direction. Right-hand rule around +Y:
    # 0° = (0,0,-R), 90° = (-R,0,0), 180° = (0,0,+R).
    # This curves the bend backward (-X), away from the arm tips.
    bend = (
        cq.Workplane("YZ")
        .polyline(pts)
        .close()
        .revolve(180, (0, 0), (1, 0))
    )
    try:
        bend = bend.edges().fillet(0.0006)
    except Exception:
        pass
    return bend


# ---------------------------------------------------------------------------
# Arm placement configs (repeated sub-parts via loop)
# ---------------------------------------------------------------------------
_ARM_CONFIGS = [
    {
        "name": "fixed_arm",
        "material": WOOD,
        "z_offset": -BEND_RADIUS,
        "splay": HALF_SPLAY,
    },
    {
        "name": "moving_arm",
        "material": WOOD_DARK,
        "z_offset": 0.0,
        "splay": 0.0,
    },
]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wooden_tongs")
    model.materials.extend([WOOD, WOOD_DARK])

    # --- Build both arms from shared geometry helper via loop ---
    # Visual names are explicit string literals (not f-strings) so the
    # exact-geometry contract checker can resolve them.
    _ARM_VISUAL_NAMES = ("arm_strip_0", "arm_strip_1")
    parts = {}
    for i, cfg in enumerate(_ARM_CONFIGS):
        p = model.part(cfg["name"])
        p.visual(
            mesh_from_cadquery(_arm_solid(), f"arm_{i}"),
            origin=Origin(
                xyz=(0.0, 0.0, cfg["z_offset"]),
                rpy=(0.0, 0.0, cfg["splay"]),
            ),
            material=cfg["material"],
            name=_ARM_VISUAL_NAMES[i],
        )
        parts[cfg["name"]] = p

    fixed_arm = parts["fixed_arm"]
    moving_arm = parts["moving_arm"]

    # --- Wooden bend (the spring): rigid with the fixed arm ---
    # The continuous U/hairpin connecting both arm roots. No separate metal
    # clip — the wood itself flexes at the bend.
    fixed_arm.visual(
        mesh_from_cadquery(_bend_solid(), "bend"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=WOOD,
        name="wood_bend",
    )

    # --- Revolute flex joint at the bend/arm junction ---
    # Joint origin at the surface where the bend meets the moving arm
    # (z = +BEND_RADIUS). The joint frame tilts the moving arm by -HALF_SPLAY
    # so at q=0 the two arms form the relaxed open "V". Positive q (squeeze)
    # rotates the moving arm toward the fixed arm, closing the tips.
    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=fixed_arm,
        child=moving_arm,
        origin=Origin(
            xyz=(0.0, 0.0, BEND_RADIUS),
            rpy=(0.0, 0.0, -HALF_SPLAY),
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=3.0, lower=-0.04, upper=0.18
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def _tip_y(ctx: TestContext, part) -> float:
    """World Y of the paddle tip (max-X end) of an arm part, current pose."""
    lo, hi = ctx.part_world_aabb(part)
    return (lo[1] + hi[1]) / 2.0


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    fixed_arm = object_model.get_part("fixed_arm")
    moving_arm = object_model.get_part("moving_arm")
    pivot = object_model.get_articulation("pivot")

    # --- Joint contract: revolute about Z (normal to arm plane) ---
    ctx.check(
        "pivot is revolute",
        pivot.joint_type == "revolute"
        or pivot.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={pivot.joint_type}",
    )
    ax = pivot.axis
    ctx.check(
        "pivot axis is Z (normal to arm plane)",
        abs(ax[2]) > 0.99 and abs(ax[0]) < 0.02 and abs(ax[1]) < 0.02,
        details=f"axis={ax}",
    )

    # --- Two long flat tapered wooden arms ---
    flo, fhi = ctx.part_world_aabb(fixed_arm)
    mlo, mhi = ctx.part_world_aabb(moving_arm)
    fixed_len = fhi[0] - flo[0]
    moving_len = mhi[0] - mlo[0]
    ctx.check(
        "fixed arm is a long strip (~0.30 m)",
        0.25 < fixed_len < 0.35,
        details=f"len={fixed_len:.3f}",
    )
    ctx.check(
        "moving arm is a long strip (~0.30 m)",
        0.25 < moving_len < 0.35,
        details=f"len={moving_len:.3f}",
    )

    # Arm strip element is thin and flat (element-level, excluding bend)
    arm_lo, arm_hi = ctx.part_element_world_aabb(fixed_arm, elem="arm_strip_0")
    arm_thick = arm_hi[2] - arm_lo[2]
    ctx.check(
        "arm strip is thin flat (~4.5 mm)",
        arm_thick < 0.010,
        details=f"strip thick={arm_thick:.4f}",
    )

    # --- Wooden bend present at the pivot (one-piece bent-wood design) ---
    bend_lo, bend_hi = ctx.part_element_world_aabb(fixed_arm, elem="wood_bend")
    ctx.check(
        "wood bend sits at the pivot end (near x=0)",
        bend_hi[0] < 0.025,
        details=f"bend max_x={bend_hi[0]:.3f}",
    )
    bend_z_span = bend_hi[2] - bend_lo[2]
    ctx.check(
        "wood bend spans both arm roots in Z",
        bend_z_span > 1.5 * BEND_RADIUS,
        details=f"bend z-span={bend_z_span:.4f}",
    )
    ctx.check(
        "wood bend curves outward in -X (U/hairpin shape)",
        bend_lo[0] < -0.005,
        details=f"bend min_x={bend_lo[0]:.4f}",
    )

    # --- No metal material (bent-wood design has no metal parts) ---
    metal_mats = [
        m for m in object_model.materials
        if "metal" in m.name.lower() or "clip" in m.name.lower()
    ]
    ctx.check(
        "no metal clip material",
        len(metal_mats) == 0,
        details=f"found: {[m.name for m in metal_mats]}",
    )

    # --- No spring_clip visual on any part ---
    for p_name in ("fixed_arm", "moving_arm"):
        p = object_model.get_part(p_name)
        visual_names = [v.name for v in p.visuals]
        ctx.check(
            f"{p_name} has no spring_clip visual",
            "spring_clip" not in visual_names,
            details=f"visuals={visual_names}",
        )

    # --- Arms splay apart at rest (V shape): tips on opposite sides ---
    ctx.check(
        "arms splay to opposite sides at tips",
        _tip_y(ctx, fixed_arm) > 0.005 and _tip_y(ctx, moving_arm) < -0.005,
        details=f"fixed_tip_y={_tip_y(ctx, fixed_arm):.3f}, "
        f"moving_tip_y={_tip_y(ctx, moving_arm):.3f}",
    )

    # --- Pivot mechanism actually opens/closes the tips ---
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

    # --- Arms share the pivot origin ---
    ctx.expect_origin_distance(
        fixed_arm, moving_arm, axes="xy", max_dist=0.020,
        name="arms share the pivot origin",
    )

    # --- Junction overlap allowance ---
    # The moving arm seats into the bend at the flex junction. Small local
    # penetration from the splay rotation is intentional (seated connection).
    ctx.allow_overlap(
        fixed_arm, moving_arm,
        elem_a="wood_bend", elem_b="arm_strip_1",
        reason="Moving arm seats into the bend at the flex junction; "
        "small local penetration from splay rotation is intentional.",
    )
    ctx.expect_overlap(
        fixed_arm, moving_arm,
        axes="y",
        elem_a="wood_bend", elem_b="arm_strip_1",
        min_overlap=0.005,
        name="bend and moving arm share the flex junction width in Y",
    )

    return ctx.report()


object_model = build_object_model()
