from __future__ import annotations

# Asymmetric tailor-style scissors: two steel blades crossing at a single
# central pivot, with mismatched handles — a small round thumb loop on half A
# and a larger elongated finger loop on half B (as on barber/tailor shears).
# The two halves rotate relative to each other about the pivot (REVOLUTE) to
# open and close.
#
# Frame convention (object laid flat in the XY plane, viewed from +Z):
#   +Y  -> toward the blade tips (up in the reference image)
#   -Y  -> toward the finger loops (down in the reference image)
#   +Z  -> out of the plane (the pivot axis)
# The pivot sits at the world origin (0, 0, 0). Half A is the root; half B is
# the child that rotates about +Z. Each half is authored in a local frame whose
# pivot is at the origin, then rotated about Z so the two blades cross.

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

# --- Real-world dimensions (meters) ----------------------------------------
BLADE_LENGTH = 0.092          # pivot -> blade tip
BLADE_ROOT_WIDTH = 0.013      # blade width near the pivot
BLADE_TIP_WIDTH = 0.0022      # blade width near the tip
BLADE_THICKNESS = 0.0020      # blade stock thickness

HANDLE_NECK_WIDTH = 0.011     # plastic neck just below the pivot
HANDLE_THICKNESS = 0.0060     # plastic handle thickness (proud of the blade)

# Thumb loop (small, round, for single thumb — on half A)
THUMB_HANDLE_LENGTH = 0.068   # pivot -> far end of the thumb loop
THUMB_LOOP_OD = 0.030         # thumb-loop outer diameter
THUMB_LOOP_WALL = 0.0055      # rim thickness

# Finger loop (larger, elongated, for several fingers — on half B)
FINGER_HANDLE_LENGTH = 0.088  # pivot -> far end of the finger loop
FINGER_LOOP_LONG = 0.058      # finger-loop outer size along the handle axis
FINGER_LOOP_WIDE = 0.038      # finger-loop outer size across the handle axis
FINGER_LOOP_WALL = 0.0075     # rim thickness

PIVOT_HUB_R = 0.0075          # steel boss / cap radius at the pivot
PIVOT_HOLE_R = 0.0026         # screw hole through the pivot
PIVOT_CAP_R = 0.0066          # visible disc cap radius
PIVOT_CAP_T = 0.0016          # cap thickness

# Each half is canted off the handle axis by this angle so the blades cross
# above the pivot and the handles splay below it (open scissors pose).
HALF_CANT = math.radians(11.0)

STEEL_RGBA = (0.74, 0.76, 0.79, 1.0)
RED_RGBA = (0.86, 0.13, 0.11, 1.0)
CAP_RGBA = (0.80, 0.82, 0.85, 1.0)


def _blade_solid() -> cq.Workplane:
    """Tapered, slightly curved steel blade in the XY plane.

    Authored in a local frame: pivot at origin, blade running toward +Y, the
    inner (cutting) edge near X=0 and the back rising toward -X near the pivot.
    """
    half_root = BLADE_ROOT_WIDTH / 2.0
    half_tip = BLADE_TIP_WIDTH / 2.0
    y0 = PIVOT_HUB_R * 0.4          # blade body starts just past the hub
    y1 = BLADE_LENGTH
    # Tapered profile: inner edge straight-ish, outer (back) edge tapers to tip.
    pts = [
        (half_root, y0),
        (half_root * 0.85, (y0 + y1) * 0.45),
        (half_tip, y1 - 0.004),
        (0.0, y1),                 # sharp tip on the centerline
        (-half_root * 0.55, (y0 + y1) * 0.5),
        (-half_root, y0),
    ]
    blade = (
        cq.Workplane("XY")
        .polyline(pts)
        .close()
        .extrude(BLADE_THICKNESS / 2.0, both=True)
    )
    # Round the spine/back edges so it reads as a ground blade, not a flat plate.
    try:
        blade = blade.edges("|Z").fillet(0.0008)
    except Exception:
        pass
    return blade


def _make_neck(handle_length: float, loop_extent: float) -> cq.Workplane:
    """Tapered plastic neck from pivot hub down to the start of the loop.

    `handle_length` is the pivot-to-loop-far-end distance.
    `loop_extent` is how far the loop body reaches from its center toward the pivot.
    Local frame: pivot at origin, handle running toward -Y.
    """
    half_neck = HANDLE_NECK_WIDTH / 2.0
    neck_top_y = -PIVOT_HUB_R * 0.2
    loop_center_y = -(handle_length - loop_extent)
    neck_bottom_y = loop_center_y + loop_extent - 0.006
    neck_pts = [
        (half_neck * 1.25, neck_top_y),
        (half_neck, neck_bottom_y),
        (-half_neck, neck_bottom_y),
        (-half_neck * 1.25, neck_top_y),
    ]
    return (
        cq.Workplane("XY")
        .polyline(neck_pts)
        .close()
        .extrude(HANDLE_THICKNESS / 2.0, both=True)
    )


def _thumb_loop_solid() -> cq.Workplane:
    """Small round thumb loop (for a single thumb) in the XY plane.

    Local frame: pivot at origin, handle running toward -Y.
    """
    loop_center_y = -(THUMB_HANDLE_LENGTH - THUMB_LOOP_OD / 2.0)
    outer_r = THUMB_LOOP_OD / 2.0
    inner_r = outer_r - THUMB_LOOP_WALL

    neck = _make_neck(THUMB_HANDLE_LENGTH, THUMB_LOOP_OD / 2.0)

    # Outer circle ring
    outer = (
        cq.Workplane("XY")
        .center(0.0, loop_center_y)
        .circle(outer_r)
        .extrude(HANDLE_THICKNESS / 2.0, both=True)
    )
    # Inner hole (through-cut)
    inner = (
        cq.Workplane("XY")
        .center(0.0, loop_center_y)
        .circle(inner_r)
        .extrude(HANDLE_THICKNESS * 1.4, both=True)
    )
    loop = outer.cut(inner)
    return neck.union(loop)


def _finger_loop_solid() -> cq.Workplane:
    """Large elongated finger loop (for several fingers) in the XY plane.

    Local frame: pivot at origin, handle running toward -Y.
    """
    loop_center_y = -(FINGER_HANDLE_LENGTH - FINGER_LOOP_LONG / 2.0)

    neck = _make_neck(FINGER_HANDLE_LENGTH, FINGER_LOOP_LONG / 2.0)

    # Elongated rounded-rectangle outer shell
    outer = (
        cq.Workplane("XY")
        .center(0.0, loop_center_y)
        .rect(FINGER_LOOP_WIDE, FINGER_LOOP_LONG)
        .extrude(HANDLE_THICKNESS / 2.0, both=True)
        .edges("|Z")
        .fillet(FINGER_LOOP_WIDE * 0.38)
    )
    # Inner cutout — elongated hole for multiple fingers
    inner_w = FINGER_LOOP_WIDE - 2 * FINGER_LOOP_WALL
    inner_l = FINGER_LOOP_LONG - 2 * FINGER_LOOP_WALL
    inner = (
        cq.Workplane("XY")
        .center(0.0, loop_center_y)
        .rect(inner_w, inner_l)
        .extrude(HANDLE_THICKNESS * 1.4, both=True)
        .edges("|Z")
        .fillet(inner_w * 0.40)
    )
    loop = outer.cut(inner)
    return neck.union(loop)


def _hub_solid() -> cq.Workplane:
    """Steel pivot boss with a through hole, centered on the pivot."""
    hub = (
        cq.Workplane("XY")
        .circle(PIVOT_HUB_R)
        .extrude(BLADE_THICKNESS / 2.0, both=True)
        .faces(">Z or <Z")
        .chamfer(0.0006)
    )
    hole = (
        cq.Workplane("XY")
        .circle(PIVOT_HOLE_R)
        .extrude(BLADE_THICKNESS, both=True)
    )
    return hub.cut(hole)


def _half_shape(side: str) -> cq.Workplane:
    """One full scissor half (steel blade + steel hub + red handle is separate).

    Returns ONLY the steel parts (blade + hub) of the half, canted about the
    pivot. `side` is "a" or "b"; the two are mirror images across X.
    """
    blade = _blade_solid().union(_hub_solid())
    sign = 1.0 if side == "a" else -1.0
    # Mirror half B across the YZ plane so the two halves are true mirror images.
    if side == "b":
        blade = blade.mirror("YZ")
    # Cant the blade so it crosses the centerline above the pivot.
    blade = blade.rotate((0, 0, 0), (0, 0, 1), math.degrees(-sign * HALF_CANT))
    return blade


def _handle_shape(side: str, handle_type: str) -> cq.Workplane:
    """Red plastic handle for one half, canted and offset in Z to clear the
    other half's handle (real scissors handles are stacked in thickness).

    `handle_type` is "thumb" (small round loop) or "finger" (large elongated loop).
    """
    handle = _thumb_loop_solid() if handle_type == "thumb" else _finger_loop_solid()
    sign = 1.0 if side == "a" else -1.0
    if side == "b":
        handle = handle.mirror("YZ")
    handle = handle.rotate((0, 0, 0), (0, 0, 1), math.degrees(-sign * HALF_CANT))
    return handle


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="scissors")

    steel = model.material("blade_steel", rgba=STEEL_RGBA)
    red = model.material("handle_red", rgba=RED_RGBA)
    cap_mat = model.material("pivot_cap", rgba=CAP_RGBA)

    # Handles are stacked in thickness so the two halves nest at the pivot like
    # real scissors. Each handle is HANDLE_THICKNESS thick and centered on its
    # local z=0; offsetting by +/- HANDLE_THICKNESS/2 stacks them with a shared
    # face at z=0 (half A below, half B above) and no interpenetration.
    handle_z_a = -HANDLE_THICKNESS / 2.0
    handle_z_b = +HANDLE_THICKNESS / 2.0

    # --- Half A (root) -----------------------------------------------------
    blade_a = model.part("blade_a")
    blade_a.visual(
        mesh_from_cadquery(_half_shape("a"), "blade_a", assets=model.assets),
        name="blade_steel",
        material=steel,
    )
    blade_a.visual(
        mesh_from_cadquery(_handle_shape("a", "thumb"), "handle_a", assets=model.assets),
        name="thumb_loop",
        material=red,
        origin=Origin(xyz=(0.0, 0.0, handle_z_a)),
    )
    # Visible disc cap on the front face of the pivot.
    cap_shape = (
        cq.Workplane("XY")
        .circle(PIVOT_CAP_R)
        .extrude(PIVOT_CAP_T)
        .faces(">Z")
        .chamfer(0.0005)
    )
    # Seat the cap base flush on the hub's front face (z = +BLADE_THICKNESS/2)
    # so it reads as a riveted disc bearing on the steel boss, not a floating
    # decal. A tiny embed keeps the cap connected to the hub geometry.
    blade_a.visual(
        mesh_from_cadquery(cap_shape, "pivot_cap", assets=model.assets),
        name="pivot_cap",
        material=cap_mat,
        origin=Origin(xyz=(0.0, 0.0, BLADE_THICKNESS / 2.0 - 0.0002)),
    )

    # --- Half B (rotates about the pivot) ---------------------------------
    blade_b = model.part("blade_b")
    blade_b.visual(
        mesh_from_cadquery(_half_shape("b"), "blade_b", assets=model.assets),
        name="blade_steel",
        material=steel,
    )
    blade_b.visual(
        mesh_from_cadquery(_handle_shape("b", "finger"), "handle_b", assets=model.assets),
        name="finger_loop",
        material=red,
        origin=Origin(xyz=(0.0, 0.0, handle_z_b)),
    )

    # --- Pivot articulation ------------------------------------------------
    # Both halves share the pivot at the origin. Axis = +Z (out of plane).
    # q = 0 is the modeled open pose. Negative q closes the scissors (blade tips
    # come together); positive q opens them wider.
    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=blade_a,
        child=blade_b,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=4.0,
            lower=-2.0 * HALF_CANT,   # fully closed: tips meet on the centerline
            upper=math.radians(28.0), # wide open
        ),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    blade_a = object_model.get_part("blade_a")
    blade_b = object_model.get_part("blade_b")
    pivot = object_model.get_articulation("pivot")

    # --- Joint contract ----------------------------------------------------
    ctx.check(
        "pivot is revolute",
        pivot.joint_type == "revolute" or pivot.articulation_type == ArticulationType.REVOLUTE,
        details=f"joint_type={pivot.joint_type}",
    )
    axis = tuple(round(c, 6) for c in pivot.axis)
    ctx.check(
        "pivot axis is +Z (out of scissor plane)",
        axis == (0.0, 0.0, 1.0),
        details=f"axis={axis}",
    )
    porg = pivot.origin.xyz
    ctx.check(
        "pivot sits at the central crossing point",
        all(abs(c) < 1e-6 for c in porg),
        details=f"origin={porg}",
    )

    # --- Both halves carry a steel blade; handles are asymmetric -----------
    # Half A has a small round thumb loop; half B has a larger elongated finger loop.
    a_names = {v.name for v in blade_a.visuals}
    b_names = {v.name for v in blade_b.visuals}
    for half, loop_name in ((blade_a, "thumb_loop"), (blade_b, "finger_loop")):
        names = {v.name for v in half.visuals}
        ctx.check(
            f"{half.name} has a steel blade",
            "blade_steel" in names,
            details=f"visuals={sorted(names)}",
        )
        ctx.check(
            f"{half.name} has its {loop_name}",
            loop_name in names,
            details=f"visuals={sorted(names)}",
        )

    # The visible pivot cap belongs to the root half.
    ctx.check("pivot cap is present", "pivot_cap" in a_names, details=f"{sorted(a_names)}")

    # --- Geometry sanity: blade reaches up (+Y), handle reaches down (-Y) ---
    blade_box = ctx.part_element_world_aabb(blade_a, elem="blade_steel")
    thumb_box = ctx.part_element_world_aabb(blade_a, elem="thumb_loop")
    finger_box = ctx.part_element_world_aabb(blade_b, elem="finger_loop")
    if blade_box is not None and thumb_box is not None:
        (_, b_min_y, _), (_, b_max_y, _) = blade_box
        (_, h_min_y, _), (_, h_max_y, _) = thumb_box
        ctx.check(
            "blade extends toward the tips (+Y)",
            b_max_y > 0.07,
            details=f"blade max_y={b_max_y}",
        )
        ctx.check(
            "thumb loop extends toward the fingers (-Y)",
            h_min_y < -0.04,
            details=f"thumb_loop min_y={h_min_y}",
        )

    # --- Asymmetric handles: finger loop is larger than thumb loop ---------
    if thumb_box is not None and finger_box is not None:
        # Compare the Y-extent (handle length) of each loop
        thumb_y_extent = thumb_box[1][1] - thumb_box[0][1]
        finger_y_extent = finger_box[1][1] - finger_box[0][1]
        ctx.check(
            "finger loop is longer than thumb loop (asymmetric handles)",
            finger_y_extent > thumb_y_extent * 1.2,
            details=f"thumb_y_extent={thumb_y_extent:.4f}, finger_y_extent={finger_y_extent:.4f}",
        )
        # Compare the X-extent (loop width) of each loop
        thumb_x_extent = thumb_box[1][0] - thumb_box[0][0]
        finger_x_extent = finger_box[1][0] - finger_box[0][0]
        ctx.check(
            "finger loop is wider than thumb loop (asymmetric handles)",
            finger_x_extent > thumb_x_extent * 1.1,
            details=f"thumb_x_extent={thumb_x_extent:.4f}, finger_x_extent={finger_x_extent:.4f}",
        )

    # --- The two blades cross at the pivot in the modeled pose -------------
    # Half A is canted to one side, half B to the other, so their steel tips
    # straddle the centerline (one tip at +X, the other at -X).
    a_tip = ctx.part_element_world_aabb(blade_a, elem="blade_steel")
    b_tip = ctx.part_element_world_aabb(blade_b, elem="blade_steel")
    if a_tip is not None and b_tip is not None:
        a_cx = (a_tip[0][0] + a_tip[1][0]) / 2.0
        b_cx = (b_tip[0][0] + b_tip[1][0]) / 2.0
        ctx.check(
            "blades cross (steel halves on opposite sides of center)",
            a_cx * b_cx < 0.0,
            details=f"a_center_x={a_cx}, b_center_x={b_cx}",
        )

    # --- Pivot kinematics: the blade tips swing about the central pivot ----
    # Helper: signed x-center of blade_b's steel tip region in a given pose.
    def _b_tip_cx() -> float | None:
        box = ctx.part_element_world_aabb(blade_b, elem="blade_steel")
        if box is None:
            return None
        return (box[0][0] + box[1][0]) / 2.0

    a_box = ctx.part_element_world_aabb(blade_a, elem="blade_steel")
    rest_bx = _b_tip_cx()

    # Half-closed pose: blade_b tip should swing onto the centerline (x ~ 0),
    # i.e. closer to zero than at rest. lower == -2*HALF_CANT is fully closed
    # (blade_b parallel to blade_a), so -HALF_CANT is the crossing point.
    with ctx.pose({pivot: -HALF_CANT}):
        half_closed_bx = _b_tip_cx()
    # Fully open pose: blade_b tip swings further out on its own side.
    with ctx.pose({pivot: pivot.motion_limits.upper}):
        wide_bx = _b_tip_cx()
    # Fully closed pose: blade_b becomes parallel to blade_a (same side as A).
    with ctx.pose({pivot: pivot.motion_limits.lower}):
        closed_bx = _b_tip_cx()

    if a_box is not None and rest_bx is not None and half_closed_bx is not None:
        a_cx = (a_box[0][0] + a_box[1][0]) / 2.0
        ctx.check(
            "half-closing swings blade_b tip toward the centerline",
            abs(half_closed_bx) < abs(rest_bx),
            details=f"rest_bx={rest_bx}, half_closed_bx={half_closed_bx}",
        )
        if closed_bx is not None:
            ctx.check(
                "fully closed: blade_b aligns to blade_a's side (parallel blades)",
                (closed_bx * a_cx) > 0.0,
                details=f"closed_bx={closed_bx}, a_cx={a_cx}",
            )

    if rest_bx is not None and wide_bx is not None:
        ctx.check(
            "opening the pivot swings blade_b tip further out",
            abs(wide_bx) > abs(rest_bx) and (wide_bx * rest_bx) > 0.0,
            details=f"rest_bx={rest_bx}, wide_bx={wide_bx}",
        )

    # --- Pivot region: the two steel hubs share the same axis and touch ----
    ctx.expect_overlap(
        blade_a,
        blade_b,
        axes="xy",
        elem_a="blade_steel",
        elem_b="blade_steel",
        min_overlap=0.004,
        name="steel halves overlap at the pivot hub",
    )
    # The two steel halves nest at the pivot (riveted lap joint): allow the
    # small intentional interpenetration of the two hub bosses sharing one axis.
    ctx.allow_overlap(
        blade_a,
        blade_b,
        elem_a="blade_steel",
        elem_b="blade_steel",
        reason="The two steel halves form a riveted lap joint sharing one pivot axis; their hub bosses intentionally nest.",
    )

    return ctx.report()
