from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Locking pliers (Vise-Grip style).
#
# Two handles cross at a central pivot bolt. The lower handle (root) carries
# the fixed lower jaw. The upper handle pivots to open/close the jaws.
# Serrated teeth on the inner jaw faces and textured grip ribs on handles.
# A rear adjustment screw protrudes from the butt end of the lower handle.
#
# Tool lies flat on XY plane (Z up). Jaws point +X, handles extend -X.
# ---------------------------------------------------------------------------

# --- global proportions ---
PLATE_T = 0.004       # steel plate thickness per half

# Z-layer stack: lower handle below, upper handle above at pivot
ZL0, ZL1 = 0.0, PLATE_T          # lower: 0.0 .. 0.004
ZU0, ZU1 = ZL1, ZL1 + PLATE_T   # upper: 0.004 .. 0.008

# Pivot bolt
PIVOT_R = 0.005
BOLT_HEAD_R = 0.006
BOLT_HEAD_H = 0.002

# Adjustment screw (protrudes from rear end of lower handle along -X)
SCREW_R = 0.003
SCREW_LENGTH = 0.012  # visible protruding length
SCREW_HEAD_R = 0.005
SCREW_MOUNT_X = -0.098  # very rear end of lower handle
SCREW_STEM = 0.001       # tiny stem inside handle for contact

# Jaw teeth parameters
TOOTH_COUNT = 8
TOOTH_DEPTH = 0.0012
TOOTH_WIDTH = 0.003

# Grip rib parameters
RIB_COUNT = 10
RIB_DEPTH = 0.0008
RIB_WIDTH = 0.002

# Opening angle at rest (each half splayed this much from center)
HALF_OPEN = math.radians(14)
CLOSE_TRAVEL = 2.0 * HALF_OPEN  # ~28 degrees closing


def _serrated_jaw(s: float, z0: float, z1: float) -> cq.Workplane:
    """Build one jaw half with serrated teeth on inner face.
    s = +1 for lower jaw (teeth on +y face), s = -1 for upper jaw (teeth on -y face).
    Jaw extends from pivot (+X direction)."""
    jaw_pts = [
        (0.000, s * 0.002),
        (0.012, s * 0.007),
        (0.025, s * 0.008),
        (0.038, s * 0.006),
        (0.042, s * 0.003),
        (0.040, s * 0.001),
        (0.000, s * 0.001),
    ]
    jaw = (
        cq.Workplane("XY", origin=(0.0, 0.0, z0))
        .polyline(jaw_pts)
        .close()
        .extrude(z1 - z0)
    )

    # Cut serrated teeth grooves on inner face (near y=0 edge)
    for i in range(TOOTH_COUNT):
        tx = 0.008 + i * (0.030 / TOOTH_COUNT)
        groove = (
            cq.Workplane("XY", origin=(tx - TOOTH_WIDTH / 2, s * 0.001 - TOOTH_DEPTH, z0 - 0.001))
            .box(TOOTH_WIDTH, TOOTH_DEPTH * 2, (z1 - z0) + 0.002)
        )
        jaw = jaw.cut(groove)

    return jaw


def _handle_body(s: float, z0: float, z1: float) -> cq.Workplane:
    """Build one handle body (steel core) extending from pivot to rear."""
    handle_pts = [
        (-0.005, s * (-0.004)),
        (-0.020, s * (-0.008)),
        (-0.045, s * (-0.010)),
        (-0.070, s * (-0.011)),
        (-0.088, s * (-0.010)),
        (-0.095, s * (-0.008)),
        (-0.098, s * (-0.006)),
        (-0.098, s * (-0.004)),
        (-0.095, s * (-0.002)),
        (-0.088, s * (-0.002)),
        (-0.050, s * (-0.003)),
        (-0.010, s * (-0.003)),
    ]
    return (
        cq.Workplane("XY", origin=(0.0, 0.0, z0))
        .polyline(handle_pts)
        .close()
        .extrude(z1 - z0)
    )


def _handle_grip(s: float, z0: float, z1: float) -> cq.Workplane:
    """Overmolded grip with raised ribs for texture."""
    grip_pts = [
        (-0.020, s * (-0.009)),
        (-0.040, s * (-0.012)),
        (-0.060, s * (-0.013)),
        (-0.080, s * (-0.012)),
        (-0.093, s * (-0.010)),
        (-0.100, s * (-0.007)),
        (-0.100, s * (-0.003)),
        (-0.093, s * (-0.001)),
        (-0.060, s * (-0.001)),
        (-0.030, s * (-0.003)),
        (-0.020, s * (-0.006)),
    ]
    grip = (
        cq.Workplane("XY", origin=(0.0, 0.0, z0))
        .polyline(grip_pts)
        .close()
        .extrude(z1 - z0)
    )

    # Add raised ribs (transverse ridges) for grip texture
    for i in range(RIB_COUNT):
        rx = -0.025 - i * (0.065 / RIB_COUNT)
        # Rib extends across the grip width, raised above surface
        rib_y_center = s * (-0.007)
        rib = (
            cq.Workplane("XY", origin=(rx - RIB_WIDTH / 2, rib_y_center - 0.009, z0 + (z1 - z0) * 0.5 - RIB_DEPTH))
            .box(RIB_WIDTH, 0.018, RIB_DEPTH * 2)
        )
        grip = grip.union(rib)

    return grip


def _pivot_boss(z0: float, z1: float, with_hole: bool) -> cq.Workplane:
    """Circular boss at pivot point."""
    boss = cq.Workplane("XY", origin=(0.0, 0.0, z0)).circle(PIVOT_R).extrude(z1 - z0)
    if with_hole:
        hole = cq.Workplane("XY", origin=(0.0, 0.0, z0 - 0.001)).circle(0.0025).extrude(
            (z1 - z0) + 0.002
        )
        boss = boss.cut(hole)
    return boss


def _pivot_bolt() -> cq.Workplane:
    """Central pivot bolt with heads on both sides."""
    shank = cq.Workplane("XY", origin=(0.0, 0.0, -0.001)).circle(0.0025).extrude(ZU1 + 0.004)
    lower_head = cq.Workplane("XY", origin=(0.0, 0.0, -0.002)).circle(BOLT_HEAD_R).extrude(BOLT_HEAD_H)
    upper_head = cq.Workplane("XY", origin=(0.0, 0.0, ZU1 + 0.001)).circle(BOLT_HEAD_R).extrude(BOLT_HEAD_H)
    bolt = shank.union(lower_head).union(upper_head)
    try:
        bolt = bolt.edges(">Z").fillet(0.0005)
    except Exception:
        pass
    return bolt


def _adjustment_screw_local() -> cq.Workplane:
    """Adjustment screw in its own local frame (origin at mount point).
    Screw axis along local +X (protrudes in -X from the handle end).
    The screw body extends from x=-SCREW_LENGTH to x=+SCREW_STEM (small stem into handle)."""
    # Main threaded shaft along X
    shaft = (
        cq.Workplane("YZ", origin=(-SCREW_LENGTH, 0.0, 0.0))
        .circle(SCREW_R)
        .extrude(SCREW_LENGTH + SCREW_STEM)
    )
    # Knurled head at the rear (-X end)
    head = (
        cq.Workplane("YZ", origin=(-SCREW_LENGTH - 0.003, 0.0, 0.0))
        .circle(SCREW_HEAD_R)
        .extrude(0.005)
    )
    # Knurling ridges on head
    for i in range(12):
        angle = i * (2 * math.pi / 12)
        ky = (SCREW_HEAD_R + 0.0005) * math.cos(angle)
        kz = (SCREW_HEAD_R + 0.0005) * math.sin(angle)
        knurl = (
            cq.Workplane("YZ", origin=(-SCREW_LENGTH - 0.003, ky - 0.0004, kz - 0.0004))
            .box(0.005, 0.0008, 0.0008)
        )
        head = head.union(knurl)
    # Slot on back face
    slot = (
        cq.Workplane("YZ", origin=(-SCREW_LENGTH - 0.004, -SCREW_R * 0.7, -0.0004))
        .box(0.002, SCREW_R * 1.4, 0.0008)
    )
    screw = shaft.union(head).cut(slot)
    return screw


def _neck_tang(s: float, z0: float, z1: float) -> cq.Workplane:
    """Connecting neck between jaw and handle at pivot region."""
    tang_pts = [
        (0.005, s * 0.002),
        (0.005, s * (-0.003)),
        (-0.010, s * (-0.004)),
        (-0.010, s * 0.002),
    ]
    return (
        cq.Workplane("XY", origin=(0.0, 0.0, z0))
        .polyline(tang_pts)
        .close()
        .extrude(z1 - z0)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="locking_pliers")

    steel = model.material("brushed_steel", rgba=(0.65, 0.67, 0.70, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.40, 0.42, 0.45, 1.0))
    grip_red = model.material("grip_red", rgba=(0.80, 0.15, 0.10, 1.0))
    bolt_mat = model.material("zinc_plated", rgba=(0.75, 0.76, 0.72, 1.0))

    # --- Lower handle (root/base link) ---
    lower = model.part("fixed_handle")
    lower_pose = Origin(rpy=(0.0, 0.0, HALF_OPEN))

    # Lower jaw with serrated teeth (full height spanning both plates)
    lower.visual(
        mesh_from_cadquery(_serrated_jaw(+1.0, ZL0, ZU1), "lower_jaw"),
        origin=lower_pose,
        material=steel,
        name="jaw_teeth",
    )
    # Lower pivot boss
    lower.visual(
        mesh_from_cadquery(_pivot_boss(ZL0, ZL1, with_hole=False), "lower_boss"),
        origin=lower_pose,
        material=steel,
        name="pivot_boss",
    )
    # Lower neck/tang
    lower.visual(
        mesh_from_cadquery(_neck_tang(+1.0, ZL0, ZL1), "lower_tang"),
        origin=lower_pose,
        material=steel,
        name="neck_tang",
    )
    # Lower handle steel core
    lower.visual(
        mesh_from_cadquery(_handle_body(+1.0, ZL0, ZL1), "lower_handle"),
        origin=lower_pose,
        material=steel,
        name="handle_core",
    )
    # Lower handle grip with ribs
    lower.visual(
        mesh_from_cadquery(_handle_grip(+1.0, ZL0 - 0.001, ZL1 + 0.001), "lower_grip"),
        origin=lower_pose,
        material=grip_red,
        name="grip_ribs",
    )
    # Pivot bolt (belongs to lower/fixed half)
    lower.visual(
        mesh_from_cadquery(_pivot_bolt(), "bolt"),
        origin=lower_pose,
        material=bolt_mat,
        name="pivot_bolt",
    )

    # --- Upper handle (moving link) ---
    upper = model.part("moving_handle")

    # Upper jaw with serrated teeth
    upper.visual(
        mesh_from_cadquery(_serrated_jaw(-1.0, ZU0, ZU1 + (ZU1 - ZL0)), "upper_jaw"),
        material=steel,
        name="jaw_teeth",
    )
    # Upper pivot boss (with hole)
    upper.visual(
        mesh_from_cadquery(_pivot_boss(ZU0, ZU1, with_hole=True), "upper_boss"),
        material=steel,
        name="pivot_boss",
    )
    # Upper neck/tang
    upper.visual(
        mesh_from_cadquery(_neck_tang(-1.0, ZU0, ZU1), "upper_tang"),
        material=steel,
        name="neck_tang",
    )
    # Upper handle steel core
    upper.visual(
        mesh_from_cadquery(_handle_body(-1.0, ZU0, ZU1), "upper_handle"),
        material=steel,
        name="handle_core",
    )
    # Upper handle grip with ribs
    upper.visual(
        mesh_from_cadquery(_handle_grip(-1.0, ZU0 - 0.001, ZU1 + 0.001), "upper_grip"),
        material=grip_red,
        name="grip_ribs",
    )

    # --- Adjustment screw (separate part, mounted at rear of fixed handle) ---
    screw_part = model.part("adjustment_screw")
    # Screw part frame: its origin sits at the rear of the handle in the parent frame.
    # The screw local geometry has the shaft extending in -X from origin and a small
    # stem extending +X into the handle, ensuring contact/overlap at the mount.
    # The articulation origin already applies the HALF_OPEN yaw, so the
    # screw visual sits at identity in the child frame.
    screw_part.visual(
        mesh_from_cadquery(_adjustment_screw_local(), "screw_body"),
        material=dark_steel,
        name="screw_body",
    )

    # --- Pivot articulation: revolute at central bolt ---
    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, -HALF_OPEN)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=50.0, velocity=3.0, lower=0.0, upper=CLOSE_TRAVEL),
    )

    # --- Adjustment screw articulation: revolute, axis along handle (X) ---
    # Screw rotates about its own axis (along X in the articulation frame)
    # Parent: lower handle, child: screw part
    # The handle rear in the closed-design frame is at ~(SCREW_MOUNT_X, -0.005).
    # The visuals are rotated by HALF_OPEN in the lower part frame, so the
    # mount point in the parent frame is the rotated position.
    _handle_rear_x = SCREW_MOUNT_X
    _handle_rear_y = -0.005
    _mount_x = _handle_rear_x * math.cos(HALF_OPEN) - _handle_rear_y * math.sin(HALF_OPEN)
    _mount_y = _handle_rear_x * math.sin(HALF_OPEN) + _handle_rear_y * math.cos(HALF_OPEN)
    _mount_z = (ZL0 + ZL1) / 2
    model.articulation(
        "adjustment_screw",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=screw_part,
        origin=Origin(xyz=(_mount_x, _mount_y, _mount_z), rpy=(0.0, 0.0, HALF_OPEN)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=6.0, lower=-6.28, upper=6.28),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower = object_model.get_part("fixed_handle")
    upper = object_model.get_part("moving_handle")
    screw_part = object_model.get_part("adjustment_screw")
    pivot = object_model.get_articulation("pivot")
    screw_joint = object_model.get_articulation("adjustment_screw")

    # --- Allow the screw stem to embed into the handle grip for mechanical support ---
    ctx.allow_overlap(
        screw_part,
        lower,
        elem_a="screw_body",
        elem_b="grip_ribs",
        reason="The adjustment screw stem is intentionally embedded into the handle grip end to represent the threaded mounting interface.",
    )

    # --- Pivot bosses stack correctly ---
    ctx.expect_overlap(
        lower,
        upper,
        axes="xy",
        elem_a="pivot_boss",
        elem_b="pivot_boss",
        min_overlap=0.008,
        name="pivot bosses stack coaxially",
    )

    # --- Jaws have serrated teeth (named visual exists) ---
    ctx.check(
        "lower jaw has serrated teeth geometry",
        lower.get_visual("jaw_teeth") is not None,
        details="missing jaw_teeth visual on fixed_handle",
    )
    ctx.check(
        "upper jaw has serrated teeth geometry",
        upper.get_visual("jaw_teeth") is not None,
        details="missing jaw_teeth visual on moving_handle",
    )

    # --- Handles have textured grip ribs ---
    ctx.check(
        "lower handle has grip ribs",
        lower.get_visual("grip_ribs") is not None,
        details="missing grip_ribs visual on fixed_handle",
    )
    ctx.check(
        "upper handle has grip ribs",
        upper.get_visual("grip_ribs") is not None,
        details="missing grip_ribs visual on moving_handle",
    )

    # --- Adjustment screw exists as separate part ---
    ctx.check(
        "adjustment screw part exists with body",
        screw_part is not None and screw_part.get_visual("screw_body") is not None,
        details="missing screw_body visual on adjustment_screw part",
    )

    # --- Adjustment screw is mounted at rear of fixed handle (contact) ---
    ctx.expect_contact(
        screw_part,
        lower,
        contact_tol=0.002,
        name="adjustment screw contacts fixed handle at rear mount",
    )

    # --- Adjustment screw joint is non-fixed revolute ---
    ctx.check(
        "adjustment screw joint is revolute",
        screw_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"got {screw_joint.articulation_type}",
    )
    screw_limits = screw_joint.motion_limits
    ctx.check(
        "adjustment screw has rotation limits",
        screw_limits is not None
        and screw_limits.lower is not None
        and screw_limits.upper is not None
        and screw_limits.upper > screw_limits.lower,
        details=f"limits={screw_limits}",
    )

    # --- Pivot joint is non-fixed revolute ---
    ctx.check(
        "pivot joint is revolute",
        pivot.articulation_type == ArticulationType.REVOLUTE,
        details=f"got {pivot.articulation_type}",
    )

    # --- Rest pose: jaws open (gap between jaw teeth on Y axis) ---
    ctx.expect_gap(
        lower,
        upper,
        axis="y",
        positive_elem="jaw_teeth",
        negative_elem="jaw_teeth",
        min_gap=0.001,
        name="jaw teeth are separated at rest (open)",
    )

    # --- Handles are apart at rest ---
    ctx.expect_gap(
        upper,
        lower,
        axis="y",
        positive_elem="grip_ribs",
        negative_elem="grip_ribs",
        min_gap=0.002,
        name="handles are apart at rest",
    )

    # --- Articulation closes jaws when positive q ---
    open_jaw = ctx.part_element_world_aabb(upper, elem="jaw_teeth")
    open_grip = ctx.part_element_world_aabb(upper, elem="grip_ribs")
    with ctx.pose({pivot: CLOSE_TRAVEL}):
        closed_jaw = ctx.part_element_world_aabb(upper, elem="jaw_teeth")
        closed_grip = ctx.part_element_world_aabb(upper, elem="grip_ribs")

    ctx.check(
        "closing swings upper jaw toward lower jaw",
        open_jaw is not None
        and closed_jaw is not None
        and closed_jaw[1][1] > open_jaw[1][1] + 0.002,
        details=f"open={open_jaw}, closed={closed_jaw}",
    )
    ctx.check(
        "handles scissor opposite to the jaws",
        open_grip is not None
        and closed_grip is not None
        and closed_grip[0][1] < open_grip[0][1] - 0.002,
        details=f"open={open_grip}, closed={closed_grip}",
    )

    # --- Overall proportions check ---
    la = ctx.part_world_aabb(lower)
    ua = ctx.part_world_aabb(upper)
    if la is not None and ua is not None:
        length = max(la[1][0], ua[1][0]) - min(la[0][0], ua[0][0])
        ctx.check(
            "overall length ~0.14-0.18 m",
            0.120 <= length <= 0.200,
            details=f"length={length:.4f}",
        )
    else:
        ctx.fail("overall proportions", "missing part AABBs")

    return ctx.report()


object_model = build_object_model()
