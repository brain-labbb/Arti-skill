from __future__ import annotations

# Scissor-style wooden tongs.
#
# Real object: two flat tapered wooden arms cross over each other and are
# joined by a single round pin partway along their length. Shorter handle
# ends are on one side of the pivot; longer gripping paddle tips are on the
# other. Squeezing the handles pivots the arms about the central pin,
# closing the tips together to grip food.
#
# Articulation: a REVOLUTE pivot at the crossing pin. Arm 0 is the fixed
# root; arm 1 rotates about the pin axis (Z, normal to the flat arm plane).
# Positive q closes the tips together; the resting pose holds them open.

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
HANDLE_LENGTH = 0.110       # pivot to handle end (short side)
TIP_REACH = 0.190           # pivot to tip end (long side)
ARM_THICKNESS = 0.005       # flat strip thickness (Z)

# Width profile of the tapered arm.
HANDLE_WIDTH = 0.025        # wider grip at handle end
PIVOT_WIDTH = 0.016         # narrower shaft at the pivot crossing
TIP_WIDTH = 0.028           # paddle at tip end

# Round pivot pin passing through both arms.
PIN_RADIUS = 0.004
PIN_EXTRA = 0.002           # proud extension above/below outer arm faces

# Splay: each arm tilts away from the centerline by this half-angle when
# the tongs are at rest (open pose).
HALF_SPLAY = math.radians(7.0)

WOOD = Material(name="wood", rgba=(0.55, 0.38, 0.22, 1.0))
WOOD_DARK = Material(name="wood_dark", rgba=(0.40, 0.26, 0.14, 1.0))
METAL = Material(name="pin_metal", rgba=(0.68, 0.70, 0.72, 1.0))


def _arm_solid() -> cq.Workplane:
    """One scissor-tong arm in its own local frame.

    The arm lies in the XY plane (flat faces normal to Z). The pivot center
    sits at x=0. The handle extends along -X to x=-HANDLE_LENGTH; the tip
    extends along +X to x=+TIP_REACH. The strip is symmetric about y=0.
    """
    half_t = ARM_THICKNESS / 2.0

    # Upper-half outline (y >= 0) from handle end to tip end.
    # Handle is wider for grip, pivot area is narrow, tip flares to a paddle.
    pts = [
        (-HANDLE_LENGTH + 0.003, 0.003),                   # handle end flat
        (-HANDLE_LENGTH + 0.015, HANDLE_WIDTH / 2.0),      # handle flare
        (-HANDLE_LENGTH + 0.040, HANDLE_WIDTH / 2.0),      # handle grip
        (-0.020, PIVOT_WIDTH / 2.0 + 0.002),               # boss approaching pivot
        (-0.006, PIVOT_WIDTH / 2.0),                       # at pivot
        (0.006, PIVOT_WIDTH / 2.0),                        # past pivot
        (0.020, PIVOT_WIDTH / 2.0 + 0.001),                # boss past pivot
        (TIP_REACH - 0.050, TIP_WIDTH / 2.0 - 0.003),     # approach paddle
        (TIP_REACH - 0.018, TIP_WIDTH / 2.0),              # tip paddle
        (TIP_REACH - 0.004, TIP_WIDTH / 2.0 - 0.005),     # tip rounding
        (TIP_REACH, 0.003),                                # tip end flat
    ]
    # Mirror to lower half for a closed symmetric profile.
    lower = [(x, -y) for (x, y) in reversed(pts)]
    outline = pts + lower

    arm = (
        cq.Workplane("XY")
        .polyline(outline)
        .close()
        .extrude(ARM_THICKNESS)
        .translate((0.0, 0.0, -half_t))
    )
    # Soften perimeter edges for a worn-wood look.
    try:
        arm = arm.edges("|Z").fillet(0.0008)
    except Exception:
        pass
    return arm


def _pin_solid(height: float) -> cq.Workplane:
    """Round pivot pin cylinder, centered at origin along Z."""
    return (
        cq.Workplane("XY")
        .circle(PIN_RADIUS)
        .extrude(height)
        .translate((0.0, 0.0, -height / 2.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wooden_tongs")
    model.materials.extend([WOOD, WOOD_DARK, METAL])

    arm_geo = _arm_solid()

    # Z-stacking: the two arms cross over each other at the pivot, one above
    # the other with a small clearance gap between them.
    arm_gap = 0.002
    arm_stack = ARM_THICKNESS / 2.0 + arm_gap / 2.0
    pin_height = 2.0 * (arm_stack + ARM_THICKNESS / 2.0) + 2.0 * PIN_EXTRA
    pin_geo = _pin_solid(pin_height)

    # Per-arm placement: symmetric Z offsets and opposite splay angles so the
    # arms cross at the pivot. Arm 0 tips toward +Y; arm 1 tips toward -Y.
    arm_mats = [WOOD, WOOD_DARK]
    arm_z = [-arm_stack, arm_stack]
    arm_splay = [HALF_SPLAY, -HALF_SPLAY]

    parts = []
    for i in range(2):
        p = model.part(f"arm_{i}")
        p.visual(
            mesh_from_cadquery(arm_geo, f"arm_{i}_strip"),
            origin=Origin(xyz=(0.0, 0.0, arm_z[i]), rpy=(0.0, 0.0, arm_splay[i])),
            material=arm_mats[i],
            name=f"strip_{i}",
        )
        parts.append(p)

    # Pivot pin is rigid with the root arm (arm_0), passing vertically
    # through both arms at the crossing point.
    parts[0].visual(
        mesh_from_cadquery(pin_geo, "pivot_pin"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=METAL,
        name="pivot_pin",
    )

    # Revolute pivot at the crossing pin. Joint origin at the contact plane
    # between the stacked arms (z=0). At q=0, the child part frame coincides
    # with the parent; each arm's visual origin applies its own splay.
    # Positive q (CCW about +Z) swings arm_1's tip from -Y toward +Y,
    # closing the tips together.
    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=parts[0],
        child=parts[1],
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=3.0, lower=-0.05, upper=0.28,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    arm_0 = object_model.get_part("arm_0")
    arm_1 = object_model.get_part("arm_1")
    pivot = object_model.get_articulation("pivot")

    # --- Intentional overlap: pivot pin passes through arm_1 ---
    ctx.allow_overlap(
        "arm_0", "arm_1",
        elem_a="pivot_pin", elem_b="strip_1",
        reason="The pivot pin passes through both arms at the scissor crossing, representing the physical pin joint.",
    )

    # Proof: pin overlaps arm_1 in XY (passes through it) but the arm strips
    # themselves are separated in Z.
    ctx.expect_overlap(
        "arm_0", "arm_1",
        axes="xy", elem_a="pivot_pin", elem_b="strip_1",
        min_overlap=0.003,
        name="pivot pin sits within arm_1 footprint at crossing",
    )
    ctx.expect_gap(
        arm_1, arm_0,
        axis="z", positive_elem="strip_1", negative_elem="strip_0",
        min_gap=0.0005, max_gap=0.005,
        name="arm strips are separated in Z (no arm-to-arm overlap)",
    )

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

    # --- Hero geometry: two long flat tapered arms ---
    for i, p in enumerate([arm_0, arm_1]):
        lo, hi = ctx.part_world_aabb(p)
        length = hi[0] - lo[0]
        thick = hi[2] - lo[2]
        ctx.check(
            f"arm_{i} is a long strip (~0.30 m)",
            0.27 < length < 0.33,
            details=f"len={length:.3f}",
        )
        ctx.check(
            f"arm_{i} is thin and flat",
            thick < 0.020 and length > 10.0 * ARM_THICKNESS,
            details=f"thick={thick:.4f}",
        )

    # --- Pivot pin is partway along each arm (scissor pivot, not end-clip) ---
    pin_lo, pin_hi = ctx.part_element_world_aabb(arm_0, elem="pivot_pin")
    arm_lo, arm_hi = ctx.part_world_aabb(arm_0)
    pin_x_center = (pin_lo[0] + pin_hi[0]) / 2.0
    handle_side = pin_x_center - arm_lo[0]
    tip_side = arm_hi[0] - pin_x_center
    ctx.check(
        "pivot pin is partway along the arm (not at an end)",
        handle_side > 0.05 and tip_side > 0.05,
        details=f"pin_x={pin_x_center:.3f}, handle_side={handle_side:.3f}, tip_side={tip_side:.3f}",
    )

    # --- Pivot pin is round (circular cross-section) ---
    pin_dx = pin_hi[0] - pin_lo[0]
    pin_dy = pin_hi[1] - pin_lo[1]
    ctx.check(
        "pivot pin is round (circular cross-section)",
        0.5 < pin_dx / max(pin_dy, 1e-6) < 2.0 and pin_dx < 0.015,
        details=f"pin dx={pin_dx:.4f}, dy={pin_dy:.4f}",
    )

    # --- Arms cross at the pivot: tips on opposite Y sides ---
    lo0, hi0 = ctx.part_world_aabb(arm_0)
    lo1, hi1 = ctx.part_world_aabb(arm_1)
    y_center_0 = (lo0[1] + hi0[1]) / 2.0
    y_center_1 = (lo1[1] + hi1[1]) / 2.0
    ctx.check(
        "arms cross at pivot: Y centers on opposite sides",
        y_center_0 > 0.002 and y_center_1 < -0.002,
        details=f"arm_0_y={y_center_0:.3f}, arm_1_y={y_center_1:.3f}",
    )

    # --- Pivot mechanism opens/closes the tips ---
    rest_tip_gap = abs(y_center_0 - y_center_1)

    with ctx.pose({pivot: pivot.motion_limits.upper}):
        lo0c, hi0c = ctx.part_world_aabb(arm_0)
        lo1c, hi1c = ctx.part_world_aabb(arm_1)
        closed_tip_gap = abs((lo0c[1] + hi0c[1]) / 2.0 - (lo1c[1] + hi1c[1]) / 2.0)

    with ctx.pose({pivot: pivot.motion_limits.lower}):
        lo0o, hi0o = ctx.part_world_aabb(arm_0)
        lo1o, hi1o = ctx.part_world_aabb(arm_1)
        open_tip_gap = abs((lo0o[1] + hi0o[1]) / 2.0 - (lo1o[1] + hi1o[1]) / 2.0)

    ctx.check(
        "squeezing (upper limit) closes the tips together",
        closed_tip_gap < rest_tip_gap - 0.005,
        details=f"rest={rest_tip_gap:.3f}, closed={closed_tip_gap:.3f}",
    )
    ctx.check(
        "opening (lower limit) spreads the tips wider",
        open_tip_gap > rest_tip_gap,
        details=f"rest={rest_tip_gap:.3f}, open={open_tip_gap:.3f}",
    )

    # --- Arms share the pivot origin ---
    ctx.expect_origin_distance(
        arm_0, arm_1, axes="xy", max_dist=0.015,
        name="arms share the pivot origin",
    )

    return ctx.report()


object_model = build_object_model()
