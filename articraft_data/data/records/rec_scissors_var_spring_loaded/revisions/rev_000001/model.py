from __future__ import annotations

# Spring-loaded scissors: two mirrored steel blades crossing at a single central
# pivot, each continuing into a red plastic finger-loop handle, with a curved
# leaf spring bridging the two handles below the pivot. The two halves rotate
# about the pivot (REVOLUTE) to open and close. A second revolute joint
# represents the leaf-spring flex: the spring is anchored on the inner face of
# handle A and its free end contacts the inner face of handle B, biasing the
# scissors open.
#
# Frame convention (object laid flat in the XY plane, viewed from +Z):
#   +Y  -> toward the blade tips (up in the reference image)
#   -Y  -> toward the finger loops (down in the reference image)
#   +Z  -> out of the plane (the pivot axis)
# The pivot sits at the world origin (0, 0, 0). Half A is the root; half B is
# the child that rotates about +Z. Each half is authored in a local frame whose
# pivot is at the origin, then rotated about Z so the two blades cross.
# The spring part is anchored on blade_a via a second REVOLUTE joint.

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
    mesh_from_geometry,
    sweep_profile_along_spline,
    rounded_rect_profile,
)

# --- Real-world dimensions (meters) ----------------------------------------
BLADE_LENGTH = 0.092          # pivot -> blade tip
BLADE_ROOT_WIDTH = 0.013      # blade width near the pivot
BLADE_TIP_WIDTH = 0.0022      # blade width near the tip
BLADE_THICKNESS = 0.0020      # blade stock thickness

HANDLE_LENGTH = 0.082         # pivot -> far end of the finger loop
HANDLE_NECK_WIDTH = 0.011     # plastic neck just below the pivot
HANDLE_THICKNESS = 0.0060     # plastic handle thickness (proud of the blade)

LOOP_OUTER_LONG = 0.052       # finger-loop outer size along the handle axis
LOOP_OUTER_WIDE = 0.040       # finger-loop outer size across the handle axis
LOOP_WALL = 0.0075            # plastic rim thickness of the finger loop

PIVOT_HUB_R = 0.0075          # steel boss / cap radius at the pivot
PIVOT_HOLE_R = 0.0026         # screw hole through the pivot
PIVOT_CAP_R = 0.0066          # visible disc cap radius
PIVOT_CAP_T = 0.0016          # cap thickness

# Each half is canted off the handle axis by this angle so the blades cross
# above the pivot and the handles splay below it (open scissors pose).
HALF_CANT = math.radians(11.0)

# --- Spring dimensions ------------------------------------------------------
SPRING_GAP = 0.002             # Z-gap between handle inner faces for the spring
SPRING_WIDTH = 0.006           # leaf-spring strip width (across X)
SPRING_THICK = 0.0008          # leaf-spring strip thickness (thin spring steel)
SPRING_ANCHOR_Y = -0.013       # Y position of the spring anchor on the handle neck
SPRING_CONTACT_Y = -0.038      # Y position of the spring free-end contact pad
SPRING_BOW_Y = -0.028          # Y position where the spring bows outward most
SPRING_BOW_Z_EXTRA = 0.004     # extra Z bow (spring curves away from the gap)

STEEL_RGBA = (0.74, 0.76, 0.79, 1.0)
RED_RGBA = (0.86, 0.13, 0.11, 1.0)
CAP_RGBA = (0.80, 0.82, 0.85, 1.0)
SPRING_RGBA = (0.55, 0.57, 0.60, 1.0)   # darker spring steel / gunmetal


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


def _handle_solid() -> cq.Workplane:
    """Red plastic neck + finger loop in the XY plane.

    Local frame: pivot at origin, handle running toward -Y. The neck blends out
    of the pivot hub; the loop is a rounded rectangular ring (a real finger hole).
    """
    half_neck = HANDLE_NECK_WIDTH / 2.0
    # Neck: from just below the hub down to where the loop begins.
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

    # Finger loop: outer rounded rect minus inner rounded rect -> a real ring.
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

    handle = neck.union(loop)
    return handle


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


def _handle_shape(side: str) -> cq.Workplane:
    """Red plastic handle for one half, canted and offset in Z to clear the
    other half's handle (real scissors handles are stacked in thickness)."""
    handle = _handle_solid()
    sign = 1.0 if side == "a" else -1.0
    if side == "b":
        handle = handle.mirror("YZ")
    handle = handle.rotate((0, 0, 0), (0, 0, 1), math.degrees(-sign * HALF_CANT))
    return handle


def _spring_anchor_boss() -> cq.Workplane:
    """Small cylindrical boss where the leaf spring anchors on a handle face.

    Centered at the origin in the XY plane, extruded upward (+Z).
    """
    boss = (
        cq.Workplane("XY")
        .circle(SPRING_WIDTH * 0.55)
        .extrude(SPRING_THICK * 1.5)
        .faces(">Z")
        .chamfer(SPRING_THICK * 0.4)
    )
    return boss


def _spring_shape() -> "object":
    """Curved leaf spring bridging the two handles below the pivot.

    Returns a MeshGeometry built via sweep_profile_along_spline. The spring is
    a thin flat strip that curves from the anchor on handle A (lower Z face)
    through a bowed arc to the contact pad on handle B (upper Z face).

    Local frame: origin at the spring anchor point, X across the spring width,
    Y along the handle axis (toward -Y in the scissor frame), Z across the gap.
    """
    # The spring spans the Z-gap between the two handles plus bows outward.
    # z_a = 0 (anchor face of handle A), z_b = SPRING_GAP (contact face of B)
    # The spring bows outward in +Z by SPRING_BOW_Z_EXTRA at its midpoint.
    z_a = 0.0
    z_mid = SPRING_GAP / 2.0 + SPRING_BOW_Z_EXTRA
    z_b = SPRING_GAP

    # Y positions relative to the anchor (spring runs from anchor toward -Y
    # and back; in the spring's local frame the anchor is at y=0).
    y_anchor = 0.0
    y_bow = -(abs(SPRING_BOW_Y - SPRING_ANCHOR_Y))
    y_contact = -(abs(SPRING_CONTACT_Y - SPRING_ANCHOR_Y))

    # Spline path: anchor -> bow -> contact
    points = [
        (0.0, y_anchor, z_a),
        (0.0, y_bow * 0.5, z_mid * 0.7),
        (0.0, y_bow, z_mid),
        (0.0, (y_bow + y_contact) * 0.5, z_mid * 0.8),
        (0.0, y_contact, z_b),
    ]

    profile = rounded_rect_profile(SPRING_WIDTH, SPRING_THICK, radius=SPRING_THICK * 0.35)

    spring_mesh = sweep_profile_along_spline(
        points,
        profile=profile,
        samples_per_segment=14,
        cap_profile=True,
        up_hint=(0.0, 0.0, 1.0),
    )
    return spring_mesh


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="spring_scissors")

    steel = model.material("blade_steel", rgba=STEEL_RGBA)
    red = model.material("handle_red", rgba=RED_RGBA)
    cap_mat = model.material("pivot_cap", rgba=CAP_RGBA)
    spring_mat = model.material("spring_steel", rgba=SPRING_RGBA)

    # Handles are stacked in thickness with a spring gap between them.
    # Handle A sits below (-Z), handle B above (+Z), with SPRING_GAP between
    # their inner faces so the leaf spring is visible and functional.
    handle_z_a = -HANDLE_THICKNESS / 2.0 - SPRING_GAP / 2.0
    handle_z_b = +HANDLE_THICKNESS / 2.0 + SPRING_GAP / 2.0

    # --- Half A (root) -----------------------------------------------------
    blade_a = model.part("blade_a")
    blade_a.visual(
        mesh_from_cadquery(_half_shape("a"), "blade_a", assets=model.assets),
        name="blade_steel",
        material=steel,
    )
    blade_a.visual(
        mesh_from_cadquery(_handle_shape("a"), "handle_a", assets=model.assets),
        name="handle_loop",
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
    # Spring anchor boss on the inner (+Z) face of handle A's neck.
    # The boss sits on the +Z face of handle A at the anchor Y position.
    # A small embed into the handle face ensures mesh connectivity.
    anchor_z_on_a = handle_z_a + HANDLE_THICKNESS / 2.0  # top face of handle A
    boss_embed = 0.0004  # embed depth into handle face for connectivity
    blade_a.visual(
        mesh_from_cadquery(_spring_anchor_boss(), "anchor_boss_a", assets=model.assets),
        name="spring_anchor_a",
        material=spring_mat,
        origin=Origin(xyz=(0.0, SPRING_ANCHOR_Y, anchor_z_on_a - boss_embed)),
    )

    # --- Half B (rotates about the pivot) ---------------------------------
    blade_b = model.part("blade_b")
    blade_b.visual(
        mesh_from_cadquery(_half_shape("b"), "blade_b", assets=model.assets),
        name="blade_steel",
        material=steel,
    )
    blade_b.visual(
        mesh_from_cadquery(_handle_shape("b"), "handle_b", assets=model.assets),
        name="handle_loop",
        material=red,
        origin=Origin(xyz=(0.0, 0.0, handle_z_b)),
    )
    # Spring contact boss on the inner (-Z) face of handle B's neck.
    # A small embed into the handle face ensures mesh connectivity.
    contact_z_on_b = handle_z_b - HANDLE_THICKNESS / 2.0  # bottom face of handle B
    blade_b.visual(
        mesh_from_cadquery(_spring_anchor_boss(), "contact_boss_b", assets=model.assets),
        name="spring_contact_b",
        material=spring_mat,
        origin=Origin(xyz=(0.0, SPRING_CONTACT_Y, contact_z_on_b - SPRING_THICK * 1.5 + boss_embed)),
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

    # --- Spring part and articulation --------------------------------------
    # The leaf spring is a separate part anchored on blade_a. Its revolute
    # joint represents the spring flex: at q=0 the spring is relaxed (pushing
    # the handles apart = biasing open); positive q compresses the spring
    # (representing the handles being squeezed together).
    spring = model.part("spring")
    spring.visual(
        mesh_from_geometry(_spring_shape(), "leaf_spring"),
        name="leaf_spring",
        material=spring_mat,
    )

    # Spring revolute joint: anchored on blade_a at the spring anchor point.
    # The joint origin is at the anchor boss location on handle A's inner face.
    # Axis = +X so the spring flexes in the YZ plane (bowing up/down in the
    # gap between the handles). At q=0 the spring is in its natural curved
    # rest position; positive q tilts the free end toward handle A (compressed).
    model.articulation(
        "spring_flex",
        ArticulationType.REVOLUTE,
        parent=blade_a,
        child=spring,
        origin=Origin(xyz=(0.0, SPRING_ANCHOR_Y, anchor_z_on_a)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,                # relaxed / biasing open
            upper=math.radians(12.0), # compressed when handles squeezed
        ),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    blade_a = object_model.get_part("blade_a")
    blade_b = object_model.get_part("blade_b")
    spring = object_model.get_part("spring")
    pivot = object_model.get_articulation("pivot")
    spring_flex = object_model.get_articulation("spring_flex")

    # --- Joint contract: pivot ---------------------------------------------
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

    # --- Joint contract: spring flex ---------------------------------------
    ctx.check(
        "spring_flex is revolute",
        spring_flex.joint_type == "revolute" or spring_flex.articulation_type == ArticulationType.REVOLUTE,
        details=f"joint_type={spring_flex.joint_type}",
    )
    ctx.check(
        "spring_flex is not fixed (genuine moving joint)",
        spring_flex.articulation_type != ArticulationType.FIXED,
        details=f"type={spring_flex.articulation_type}",
    )
    sf_limits = spring_flex.motion_limits
    ctx.check(
        "spring_flex has positive motion range (flex travel)",
        sf_limits is not None and sf_limits.upper > sf_limits.lower,
        details=f"lower={sf_limits.lower if sf_limits else None}, upper={sf_limits.upper if sf_limits else None}",
    )

    # --- Both halves carry a steel blade and a red handle loop -------------
    for half in (blade_a, blade_b):
        names = {v.name for v in half.visuals}
        ctx.check(
            f"{half.name} has a steel blade",
            "blade_steel" in names,
            details=f"visuals={sorted(names)}",
        )
        ctx.check(
            f"{half.name} has a finger-loop handle",
            "handle_loop" in names,
            details=f"visuals={sorted(names)}",
        )

    # The visible pivot cap belongs to the root half.
    a_names = {v.name for v in blade_a.visuals}
    ctx.check("pivot cap is present", "pivot_cap" in a_names, details=f"{sorted(a_names)}")

    # --- Spring geometry and mounting --------------------------------------
    spring_names = {v.name for v in spring.visuals}
    ctx.check(
        "spring part carries a leaf_spring visual",
        "leaf_spring" in spring_names,
        details=f"visuals={sorted(spring_names)}",
    )
    ctx.check(
        "blade_a carries a spring anchor boss",
        "spring_anchor_a" in a_names,
        details=f"{sorted(a_names)}",
    )
    b_names = {v.name for v in blade_b.visuals}
    ctx.check(
        "blade_b carries a spring contact boss",
        "spring_contact_b" in b_names,
        details=f"{sorted(b_names)}",
    )

    # The spring should bridge between the two handles in Y and Z.
    spring_box = ctx.part_element_world_aabb(spring, elem="leaf_spring")
    handle_a_box = ctx.part_element_world_aabb(blade_a, elem="handle_loop")
    handle_b_box = ctx.part_element_world_aabb(blade_b, elem="handle_loop")

    if spring_box is not None and handle_a_box is not None and handle_b_box is not None:
        (_, sp_min_y, sp_min_z), (_, sp_max_y, sp_max_z) = spring_box
        (_, ha_min_y, _), (_, ha_max_y, ha_max_z) = handle_a_box
        (_, hb_min_y, hb_min_z), (_, hb_max_y, _) = handle_b_box
        # Spring should extend into the Y region between anchor and contact.
        ctx.check(
            "spring extends along handle neck (below pivot)",
            sp_min_y < -0.02 and sp_max_y > -0.04,
            details=f"spring_y=[{sp_min_y:.4f}, {sp_max_y:.4f}]",
        )
        # Spring Z range should span the gap between handle inner faces.
        ctx.check(
            "spring spans the Z gap between handles",
            sp_max_z - sp_min_z > SPRING_GAP * 0.5,
            details=f"spring_z_range={sp_max_z - sp_min_z:.4f}, gap={SPRING_GAP}",
        )

    # --- Spring flex pose: compressed spring moves its free end ------------
    spring_rest_box = ctx.part_element_world_aabb(spring, elem="leaf_spring")
    with ctx.pose({spring_flex: spring_flex.motion_limits.upper}):
        spring_compressed_box = ctx.part_element_world_aabb(spring, elem="leaf_spring")

    if spring_rest_box is not None and spring_compressed_box is not None:
        # The leaf spring AABB center should shift when the flex joint moves.
        rest_cz = (spring_rest_box[0][2] + spring_rest_box[1][2]) / 2.0
        rest_cy = (spring_rest_box[0][1] + spring_rest_box[1][1]) / 2.0
        comp_cz = (spring_compressed_box[0][2] + spring_compressed_box[1][2]) / 2.0
        comp_cy = (spring_compressed_box[0][1] + spring_compressed_box[1][1]) / 2.0
        delta = ((rest_cz - comp_cz) ** 2 + (rest_cy - comp_cy) ** 2) ** 0.5
        ctx.check(
            "spring flex joint moves the leaf spring",
            delta > 0.0001,
            details=f"rest_center=({rest_cy:.4f},{rest_cz:.4f}), compressed_center=({comp_cy:.4f},{comp_cz:.4f}), delta={delta:.6f}",
        )

    # --- Geometry sanity: blade reaches up (+Y), handle reaches down (-Y) ---
    blade_box = ctx.part_element_world_aabb(blade_a, elem="blade_steel")
    handle_box = ctx.part_element_world_aabb(blade_a, elem="handle_loop")
    if blade_box is not None and handle_box is not None:
        (_, b_min_y, _), (_, b_max_y, _) = blade_box
        (_, h_min_y, _), (_, h_max_y, _) = handle_box
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

    # --- The two blades cross at the pivot in the modeled pose -------------
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
    def _b_tip_cx() -> float | None:
        box = ctx.part_element_world_aabb(blade_b, elem="blade_steel")
        if box is None:
            return None
        return (box[0][0] + box[1][0]) / 2.0

    a_box = ctx.part_element_world_aabb(blade_a, elem="blade_steel")
    rest_bx = _b_tip_cx()

    with ctx.pose({pivot: -HALF_CANT}):
        half_closed_bx = _b_tip_cx()
    with ctx.pose({pivot: pivot.motion_limits.upper}):
        wide_bx = _b_tip_cx()
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
    ctx.allow_overlap(
        blade_a,
        blade_b,
        elem_a="blade_steel",
        elem_b="blade_steel",
        reason="The two steel halves form a riveted lap joint sharing one pivot axis; their hub bosses intentionally nest.",
    )

    return ctx.report()
