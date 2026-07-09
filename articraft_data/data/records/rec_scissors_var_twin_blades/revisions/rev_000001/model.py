from __future__ import annotations

# Herb scissors: two mirrored halves, each with 4 parallel thin blades
# stacked across the thickness with small gaps, crossing and interleaving
# at a single central pivot. Red plastic finger-loop handles on each half
# mount on opposite sides of the blade stack.
#
# Frame convention (object laid flat in the XY plane, viewed from +Z):
#   +Y  -> toward the blade tips
#   -Y  -> toward the finger loops
#   +Z  -> out of the plane (the pivot axis, blade stack direction)
# The pivot sits at the world origin (0, 0, 0). Half A is the root; half B
# rotates about +Z. Blades interleave: A at even stack slots, B at odd.

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

HANDLE_LENGTH = 0.082         # pivot -> far end of the finger loop
HANDLE_NECK_WIDTH = 0.011     # plastic neck just below the pivot
HANDLE_THICKNESS = 0.006      # plastic handle thickness

LOOP_OUTER_LONG = 0.052       # finger-loop outer size along the handle axis
LOOP_OUTER_WIDE = 0.040       # finger-loop outer size across the handle axis
LOOP_WALL = 0.0075            # plastic rim thickness of the finger loop

PIVOT_HUB_R = 0.0075          # steel boss radius at the pivot
PIVOT_HOLE_R = 0.0026         # screw hole through the pivot
PIVOT_CAP_R = 0.0066          # visible disc cap radius
PIVOT_CAP_T = 0.0016          # cap thickness

# --- Herb-scissors stack parameters ----------------------------------------
HERB_BLADES_PER_HALF = 4      # parallel blades on each scissor half
HERB_BLADE_THICKNESS = 0.0010 # thin herb-scissor blade stock (1 mm)
HERB_STACK_PITCH = 0.0030     # center-to-center blade spacing (3 mm)

# Derived stack geometry
_TOTAL_BLADES = HERB_BLADES_PER_HALF * 2
_STACK_HALF_HEIGHT = (_TOTAL_BLADES - 1) * HERB_STACK_PITCH / 2.0
_HUB_HALF_HEIGHT = _STACK_HALF_HEIGHT + HERB_BLADE_THICKNESS / 2.0
# Handle sits just outside the stack with a small seating embed.
_HANDLE_Z_OFFSET = _HUB_HALF_HEIGHT + HANDLE_THICKNESS / 2.0 - 0.0004

# Each half is canted off the handle axis so the blades cross above the pivot.
HALF_CANT = math.radians(11.0)

STEEL_RGBA = (0.74, 0.76, 0.79, 1.0)
GREEN_RGBA = (0.16, 0.52, 0.20, 1.0)   # herb-scissor green handles
CAP_RGBA = (0.80, 0.82, 0.85, 1.0)


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _blade_z_positions(side: str) -> list[float]:
    """Z centers for one half's blades in the interleaved stack.

    Even interleaved slots belong to side "a", odd to side "b".
    """
    positions: list[float] = []
    for i in range(_TOTAL_BLADES):
        z = (i - (_TOTAL_BLADES - 1) / 2.0) * HERB_STACK_PITCH
        if side == "a" and i % 2 == 0:
            positions.append(round(z, 9))
        elif side == "b" and i % 2 == 1:
            positions.append(round(z, 9))
    return positions


def _herb_blade_solid() -> cq.Workplane:
    """One thin tapered steel blade in the XY plane.

    Same profile as the parent single blade but extruded to
    HERB_BLADE_THICKNESS instead of the heavier shears stock.
    """
    half_root = BLADE_ROOT_WIDTH / 2.0
    half_tip = BLADE_TIP_WIDTH / 2.0
    y0 = PIVOT_HUB_R * 0.4
    y1 = BLADE_LENGTH
    pts = [
        (half_root, y0),
        (half_root * 0.85, (y0 + y1) * 0.45),
        (half_tip, y1 - 0.004),
        (0.0, y1),
        (-half_root * 0.55, (y0 + y1) * 0.5),
        (-half_root, y0),
    ]
    blade = (
        cq.Workplane("XY")
        .polyline(pts)
        .close()
        .extrude(HERB_BLADE_THICKNESS / 2.0, both=True)
    )
    try:
        blade = blade.edges("|Z").fillet(0.0006)
    except Exception:
        pass
    return blade


def _canted_blade(side: str, z_pos: float) -> cq.Workplane:
    """One blade translated to its stack slot, then mirrored/canted for its half."""
    blade = _herb_blade_solid().translate((0.0, 0.0, z_pos))
    sign = 1.0 if side == "a" else -1.0
    if side == "b":
        blade = blade.mirror("YZ")
    blade = blade.rotate((0, 0, 0), (0, 0, 1), math.degrees(-sign * HALF_CANT))
    return blade


def _herb_hub_solid() -> cq.Workplane:
    """Steel pivot boss spanning the full interleaved blade stack height."""
    hub = (
        cq.Workplane("XY")
        .circle(PIVOT_HUB_R)
        .extrude(_HUB_HALF_HEIGHT, both=True)
    )
    hole = (
        cq.Workplane("XY")
        .circle(PIVOT_HOLE_R)
        .extrude(_HUB_HALF_HEIGHT + 0.001, both=True)
    )
    hub = hub.cut(hole)
    try:
        hub = hub.faces(">Z or <Z").chamfer(0.0005)
    except Exception:
        pass
    return hub


def _handle_solid() -> cq.Workplane:
    """Plastic neck + finger loop in the XY plane (centered at z=0)."""
    half_neck = HANDLE_NECK_WIDTH / 2.0
    neck_top_y = -PIVOT_HUB_R * 0.2
    loop_center_y = -(HANDLE_LENGTH - LOOP_OUTER_LONG / 2.0)
    neck_bottom_y = loop_center_y + LOOP_OUTER_LONG / 2.0 - 0.006
    neck_pts = [
        (half_neck * 1.25, neck_top_y),
        (half_neck, neck_bottom_y),
        (-half_neck, neck_bottom_y),
        (-half_neck * 1.25, neck_top_y),
    ]
    neck = (
        cq.Workplane("XY")
        .polyline(neck_pts)
        .close()
        .extrude(HANDLE_THICKNESS / 2.0, both=True)
    )

    outer = (
        cq.Workplane("XY")
        .center(0.0, loop_center_y)
        .rect(LOOP_OUTER_WIDE, LOOP_OUTER_LONG)
        .extrude(HANDLE_THICKNESS / 2.0, both=True)
        .edges("|Z")
        .fillet(LOOP_OUTER_WIDE * 0.38)
    )
    inner = (
        cq.Workplane("XY")
        .center(0.0, loop_center_y)
        .rect(LOOP_OUTER_WIDE - 2 * LOOP_WALL, LOOP_OUTER_LONG - 2 * LOOP_WALL)
        .extrude(HANDLE_THICKNESS * 1.4, both=True)
        .edges("|Z")
        .fillet((LOOP_OUTER_WIDE - 2 * LOOP_WALL) * 0.40)
    )
    loop = outer.cut(inner)
    return neck.union(loop)


def _canted_handle(side: str) -> cq.Workplane:
    """Handle for one half, mirrored and canted to match the blade cant."""
    handle = _handle_solid()
    sign = 1.0 if side == "a" else -1.0
    if side == "b":
        handle = handle.mirror("YZ")
    handle = handle.rotate((0, 0, 0), (0, 0, 1), math.degrees(-sign * HALF_CANT))
    return handle


# ---------------------------------------------------------------------------
# Object model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="herb_scissors")

    steel = model.material("blade_steel", rgba=STEEL_RGBA)
    green = model.material("handle_green", rgba=GREEN_RGBA)
    cap_mat = model.material("pivot_cap", rgba=CAP_RGBA)

    # --- Half A (root) -----------------------------------------------------
    blade_a = model.part("blade_a")

    # Stacked parallel blades (repeated regular set via for-i-in-range)
    a_z_positions = _blade_z_positions("a")
    for i in range(HERB_BLADES_PER_HALF):
        blade_a.visual(
            mesh_from_cadquery(
                _canted_blade("a", a_z_positions[i]),
                f"blade_a_{i}",
                assets=model.assets,
            ),
            name=f"blade_{i}",
            material=steel,
        )

    # Pivot hub spanning the full stack
    blade_a.visual(
        mesh_from_cadquery(_herb_hub_solid(), "hub_a", assets=model.assets),
        name="hub",
        material=steel,
    )

    # Handle on the -Z side of the stack (outside the blade set)
    blade_a.visual(
        mesh_from_cadquery(_canted_handle("a"), "handle_a", assets=model.assets),
        name="handle_loop",
        material=green,
        origin=Origin(xyz=(0.0, 0.0, -_HANDLE_Z_OFFSET)),
    )

    # Visible disc cap on top (+Z face) of the hub
    cap_shape = (
        cq.Workplane("XY")
        .circle(PIVOT_CAP_R)
        .extrude(PIVOT_CAP_T)
        .faces(">Z")
        .chamfer(0.0005)
    )
    blade_a.visual(
        mesh_from_cadquery(cap_shape, "pivot_cap", assets=model.assets),
        name="pivot_cap",
        material=cap_mat,
        origin=Origin(xyz=(0.0, 0.0, _HUB_HALF_HEIGHT - 0.0002)),
    )

    # --- Half B (rotates about the pivot) ----------------------------------
    blade_b = model.part("blade_b")

    b_z_positions = _blade_z_positions("b")
    for i in range(HERB_BLADES_PER_HALF):
        blade_b.visual(
            mesh_from_cadquery(
                _canted_blade("b", b_z_positions[i]),
                f"blade_b_{i}",
                assets=model.assets,
            ),
            name=f"blade_{i}",
            material=steel,
        )

    blade_b.visual(
        mesh_from_cadquery(_herb_hub_solid(), "hub_b", assets=model.assets),
        name="hub",
        material=steel,
    )

    blade_b.visual(
        mesh_from_cadquery(_canted_handle("b"), "handle_b", assets=model.assets),
        name="handle_loop",
        material=green,
        origin=Origin(xyz=(0.0, 0.0, _HANDLE_Z_OFFSET)),
    )

    # --- Pivot articulation ------------------------------------------------
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
            lower=-2.0 * HALF_CANT,
            upper=math.radians(28.0),
        ),
    )

    return model


object_model = build_object_model()


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    blade_a = object_model.get_part("blade_a")
    blade_b = object_model.get_part("blade_b")
    pivot = object_model.get_articulation("pivot")

    # --- Joint contract ----------------------------------------------------
    ctx.check(
        "pivot is revolute",
        pivot.joint_type == "revolute"
        or pivot.articulation_type == ArticulationType.REVOLUTE,
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

    # --- Each half carries HERB_BLADES_PER_HALF stacked blades + handle + hub
    for half in (blade_a, blade_b):
        names = {v.name for v in half.visuals}
        for i in range(HERB_BLADES_PER_HALF):
            ctx.check(
                f"{half.name} has blade_{i}",
                f"blade_{i}" in names,
                details=f"visuals={sorted(names)}",
            )
        ctx.check(
            f"{half.name} has a finger-loop handle",
            "handle_loop" in names,
            details=f"visuals={sorted(names)}",
        )
        ctx.check(
            f"{half.name} has a pivot hub",
            "hub" in names,
            details=f"visuals={sorted(names)}",
        )

    # Pivot cap on root half
    a_names = {v.name for v in blade_a.visuals}
    ctx.check("pivot cap is present", "pivot_cap" in a_names)

    # --- Blade stack is separated in Z with regular spacing ----------------
    a_boxes = []
    for i in range(HERB_BLADES_PER_HALF):
        box = ctx.part_element_world_aabb(blade_a, elem=f"blade_{i}")
        if box is not None:
            a_boxes.append(box)

    if len(a_boxes) >= 2:
        z_centers = sorted((b[0][2] + b[1][2]) / 2.0 for b in a_boxes)
        gaps = [z_centers[j + 1] - z_centers[j] for j in range(len(z_centers) - 1)]
        ctx.check(
            "half A blades are stacked in Z with regular spacing",
            all(g > 0.001 for g in gaps),
            details=f"gaps={[round(g, 6) for g in gaps]}",
        )
        # Spacing should be close to 2 * HERB_STACK_PITCH (A takes every other slot)
        expected_spacing = 2.0 * HERB_STACK_PITCH
        ctx.check(
            "half A blade spacing matches 2x stack pitch",
            all(abs(g - expected_spacing) < 0.001 for g in gaps),
            details=f"gaps={[round(g, 6) for g in gaps]}, expected={expected_spacing}",
        )

    # --- A and B blade sets interleave in Z --------------------------------
    b_boxes = []
    for i in range(HERB_BLADES_PER_HALF):
        box = ctx.part_element_world_aabb(blade_b, elem=f"blade_{i}")
        if box is not None:
            b_boxes.append(box)

    if len(a_boxes) >= 2 and len(b_boxes) >= 2:
        a_zs = sorted((b[0][2] + b[1][2]) / 2.0 for b in a_boxes)
        b_zs = sorted((b[0][2] + b[1][2]) / 2.0 for b in b_boxes)
        all_zs = sorted(
            [(z, "a") for z in a_zs] + [(z, "b") for z in b_zs],
            key=lambda x: x[0],
        )
        labels = [entry[1] for entry in all_zs]
        alternating = all(labels[j] != labels[j + 1] for j in range(len(labels) - 1))
        ctx.check(
            "A and B blade sets interleave (alternate in Z)",
            alternating,
            details=f"order={labels}",
        )

    # --- Blades reach toward tips (+Y), handles reach toward fingers (-Y) --
    blade_box = ctx.part_element_world_aabb(blade_a, elem="blade_0")
    handle_box = ctx.part_element_world_aabb(blade_a, elem="handle_loop")
    if blade_box is not None and handle_box is not None:
        b_max_y = blade_box[1][1]
        h_min_y = handle_box[0][1]
        ctx.check(
            "blade extends toward the tips (+Y)",
            b_max_y > 0.07,
            details=f"blade max_y={b_max_y}",
        )
        ctx.check(
            "handle loop extends toward the fingers (-Y)",
            h_min_y < -0.05,
            details=f"handle min_y={h_min_y}",
        )

    # --- Blades cross at the pivot (halves on opposite sides of center) ----
    a_tip = ctx.part_element_world_aabb(blade_a, elem="blade_0")
    b_tip = ctx.part_element_world_aabb(blade_b, elem="blade_0")
    if a_tip is not None and b_tip is not None:
        a_cx = (a_tip[0][0] + a_tip[1][0]) / 2.0
        b_cx = (b_tip[0][0] + b_tip[1][0]) / 2.0
        ctx.check(
            "blades cross (steel halves on opposite sides of center)",
            a_cx * b_cx < 0.0,
            details=f"a_center_x={a_cx}, b_center_x={b_cx}",
        )

    # --- Pivot kinematics: blade tips swing about the central pivot ---------
    def _b_tip_cx() -> float | None:
        box = ctx.part_element_world_aabb(blade_b, elem="blade_0")
        if box is None:
            return None
        return (box[0][0] + box[1][0]) / 2.0

    rest_bx = _b_tip_cx()

    with ctx.pose({pivot: -HALF_CANT}):
        half_closed_bx = _b_tip_cx()
    with ctx.pose({pivot: pivot.motion_limits.upper}):
        wide_bx = _b_tip_cx()
    with ctx.pose({pivot: pivot.motion_limits.lower}):
        closed_bx = _b_tip_cx()

    a_box = ctx.part_element_world_aabb(blade_a, elem="blade_0")
    if a_box is not None and rest_bx is not None and half_closed_bx is not None:
        a_cx = (a_box[0][0] + a_box[1][0]) / 2.0
        ctx.check(
            "half-closing swings blade_b tip toward the centerline",
            abs(half_closed_bx) < abs(rest_bx),
            details=f"rest_bx={rest_bx}, half_closed_bx={half_closed_bx}",
        )
        if closed_bx is not None:
            ctx.check(
                "fully closed: blade_b aligns to blade_a side (parallel blades)",
                (closed_bx * a_cx) > 0.0,
                details=f"closed_bx={closed_bx}, a_cx={a_cx}",
            )

    if rest_bx is not None and wide_bx is not None:
        ctx.check(
            "opening the pivot swings blade_b tip further out",
            abs(wide_bx) > abs(rest_bx) and (wide_bx * rest_bx) > 0.0,
            details=f"rest_bx={rest_bx}, wide_bx={wide_bx}",
        )

    # --- Overlap allowances for the riveted interleaved pivot --------------
    # The two hub halves share one pivot axis (riveted lap joint).
    ctx.allow_overlap(
        blade_a, blade_b,
        elem_a="hub", elem_b="hub",
        reason="The two steel hub bosses nest at the shared pivot axis as a riveted lap joint.",
    )
    ctx.expect_overlap(
        blade_a, blade_b,
        axes="xy",
        elem_a="hub", elem_b="hub",
        min_overlap=0.004,
        name="hub halves overlap at the pivot",
    )

    # Each hub encloses the other half's blades where they pass through the
    # pivot region. This is the defining feature of interleaved herb scissors.
    for i in range(HERB_BLADES_PER_HALF):
        ctx.allow_overlap(
            blade_a, blade_b,
            elem_a="hub", elem_b=f"blade_{i}",
            reason=f"Hub A encloses blade_b blade_{i} at the shared pivot axis.",
        )
        ctx.allow_overlap(
            blade_a, blade_b,
            elem_a=f"blade_{i}", elem_b="hub",
            reason=f"Hub B encloses blade_a blade_{i} at the shared pivot axis.",
        )

    # Handle loops seat onto the hub with a small intentional embed for
    # structural connection (the handle inner face touches the hub end face).
    ctx.allow_overlap(
        blade_a, blade_b,
        elem_a="handle_loop", elem_b="hub",
        reason="Handle A seats onto hub B end face with a small structural embed.",
    )
    ctx.allow_overlap(
        blade_a, blade_b,
        elem_a="hub", elem_b="handle_loop",
        reason="Handle B seats onto hub A end face with a small structural embed.",
    )

    # Pivot cap seats onto hub with a tiny embed.
    ctx.allow_overlap(
        blade_a, blade_b,
        elem_a="pivot_cap", elem_b="hub",
        reason="Pivot cap seats onto hub B end face with a small embed.",
    )

    return ctx.report()
