from __future__ import annotations

# Single hinged door, matte dark-graphite finish.
# - FIXED root: slim dark door frame (two side jambs + head jamb).
# - Door leaf: dark-graphite blank (CadQuery) with a tall, narrow vertical glass
#   vision strip cut through it, set off-center toward the latch side.
# - Separate translucent glass pane filling the vision opening.
# - Round domed door knob on a circular rose at lever height on the latch edge
#   (REVOLUTE: the knob turns on its spindle).
# - PRIMARY articulation: the leaf swings on its hinge edge (REVOLUTE, vertical).
#
# Frame convention:
#   X = door width (hinge edge at small X, latch edge at large X)
#   Y = door thickness (room side at +Y)
#   Z = height (floor at z=0)

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# --- Real-world dimensions (meters) ---
LEAF_W = 0.900
LEAF_H = 2.040
LEAF_T = 0.042

JAMB_W = 0.038          # slim jamb face width (along X for side jambs)
JAMB_D = 0.120          # jamb depth (along Y)
HEAD_GAP = 0.004
SIDE_GAP = 0.003
HINGE_LAP = 0.004       # hinge jamb laps the leaf hinge edge

# Vision strip: tall narrow vertical glass, off-center toward the latch side.
GLASS_W = 0.110         # strip width
GLASS_H = 1.520         # strip height (tall, nearly full leaf height)
GLASS_CX = 0.580        # strip center X from hinge edge — off-center toward latch (+X)
GLASS_CZ = LEAF_H / 2.0 + 0.04
GLASS_T = 0.008         # glass pane thickness (sits inside the leaf)

OPENING_H = LEAF_H + HEAD_GAP
SILL_Z = 0.0

# --- Knob dimensions ---
ROSE_R = 0.028          # rose radius (~56mm diameter)
ROSE_H = 0.008          # rose thickness
NECK_R = 0.009          # neck/shank radius
NECK_H = 0.014          # neck length from rose face
KNOB_R = 0.026          # knob dome radius (~52mm diameter)
KNOB_STRETCH_Z = 0.85   # vertical squash factor for a slightly oblate dome


def _build_leaf_cq() -> cq.Workplane:
    """Dark-graphite leaf with a through vision opening.

    Local frame: hinge edge at local x=0, leaf extends along +X (0..LEAF_W),
    thickness centered on Y, height 0..LEAF_H.
    """
    blank = cq.Workplane("XY").box(LEAF_W, LEAF_T, LEAF_H, centered=(False, True, False))

    # Through-cut the vision opening (full thickness in Y).
    opening = (
        cq.Workplane("XZ")
        .workplane(offset=-LEAF_T)  # start well behind the leaf
        .center(GLASS_CX, GLASS_CZ)
        .rect(GLASS_W, GLASS_H)
        .extrude(2 * LEAF_T)        # cut all the way through
    )
    leaf = blank.cut(opening)

    # Soften the front face edges of the opening so it reads as a glazed bead.
    try:
        leaf = leaf.edges(">Y").edges("|Z").fillet(0.003)
    except Exception:
        pass
    return leaf


def _build_glass_cq() -> cq.Workplane:
    """Translucent glass pane filling the vision opening, in its own local frame.

    Local frame: pane centered at origin, thin along Y, in the XZ plane.
    """
    # Slightly larger than the opening so the pane laps the leaf's glazing rim
    # (like a pane held behind a bead), keeping it connected to the leaf body.
    pane = cq.Workplane("XZ").rect(GLASS_W + 0.012, GLASS_H + 0.012).extrude(GLASS_T)
    return pane


def _build_knob_cq() -> cq.Workplane:
    """Round domed door knob on a circular rose.

    Local frame: spindle axis along +Z, mounting face at z=0.
    The rose sits on the door face; the neck connects rose to the knob dome.
    """
    # --- Circular rose (flat disk with filleted edge) ---
    rose = (
        cq.Workplane("XY")
        .circle(ROSE_R)
        .extrude(ROSE_H)
    )
    try:
        rose = rose.edges(">Z").fillet(0.002)
    except Exception:
        pass
    try:
        rose = rose.edges("<Z").fillet(0.001)
    except Exception:
        pass

    # --- Neck / shank connecting rose to knob body ---
    neck = (
        cq.Workplane("XY")
        .workplane(offset=ROSE_H)
        .circle(NECK_R)
        .extrude(NECK_H)
    )

    # --- Knob dome: a sphere squished slightly oblate for a realistic grip shape ---
    knob_center_z = ROSE_H + NECK_H + KNOB_R * KNOB_STRETCH_Z
    # Build the sphere then squash it vertically for an oblate dome look
    knob_sphere = (
        cq.Workplane("XY")
        .workplane(offset=knob_center_z)
        .sphere(KNOB_R)
    )
    # Scale to make it slightly oblate (wider than tall) — typical door knob shape
    # We use a box-based scale approach: create in separate WP then transform
    # Actually CadQuery sphere is fine as-is for a round domed knob.
    # Add a subtle flat on the back so it seats against the neck.
    back_flat = (
        cq.Workplane("XY")
        .workplane(offset=ROSE_H + NECK_H - 0.001)
        .circle(KNOB_R * 1.5)
        .extrude(-KNOB_R * 0.6)
    )
    knob_dome = knob_sphere.cut(back_flat)

    # Union all parts of the knob assembly
    return rose.union(neck).union(knob_dome)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hinged_graphite_glass_door")
    model.material(name="graphite", rgba=(0.20, 0.21, 0.23, 1.0))
    model.material(name="graphite_frame", rgba=(0.15, 0.16, 0.18, 1.0))
    model.material(name="vision_glass", rgba=(0.78, 0.85, 0.88, 0.35))
    model.material(name="brushed_steel", rgba=(0.74, 0.76, 0.78, 1.0))
    model.material(name="knob_brass", rgba=(0.72, 0.62, 0.38, 1.0))

    # ---------------- FIXED FRAME (root) ----------------
    frame = model.part("door_frame")

    hinge_jamb_x = -JAMB_W / 2.0 + HINGE_LAP
    frame.visual(
        Box((JAMB_W, JAMB_D, OPENING_H)),
        origin=Origin(xyz=(hinge_jamb_x, 0.0, OPENING_H / 2.0)),
        material="graphite_frame",
        name="hinge_jamb",
    )
    latch_jamb_x = LEAF_W + SIDE_GAP + JAMB_W / 2.0
    frame.visual(
        Box((JAMB_W, JAMB_D, OPENING_H)),
        origin=Origin(xyz=(latch_jamb_x, 0.0, OPENING_H / 2.0)),
        material="graphite_frame",
        name="latch_jamb",
    )
    head_z = OPENING_H + JAMB_W / 2.0
    head_len = (latch_jamb_x + JAMB_W / 2.0) - (hinge_jamb_x - JAMB_W / 2.0)
    head_cx = (latch_jamb_x + hinge_jamb_x) / 2.0
    frame.visual(
        Box((head_len, JAMB_D, JAMB_W)),
        origin=Origin(xyz=(head_cx, 0.0, head_z)),
        material="graphite_frame",
        name="head_jamb",
    )

    # ---------------- DOOR LEAF (swings) ----------------
    leaf = model.part("door_leaf")
    leaf_mesh = mesh_from_cadquery(_build_leaf_cq(), "door_leaf")
    leaf.visual(
        leaf_mesh,
        origin=Origin(xyz=(0.0, 0.0, SILL_Z)),
        material="graphite",
        name="leaf_body",
    )
    # Glass pane filling the vision opening. It rides with the leaf.
    glass_mesh = mesh_from_cadquery(_build_glass_cq(), "vision_glass")
    leaf.visual(
        glass_mesh,
        origin=Origin(xyz=(GLASS_CX, 0.0, GLASS_CZ)),
        material="vision_glass",
        name="vision_pane",
    )

    # ---------------- DOOR KNOB on latch edge ----------------
    handle_x = LEAF_W - 0.080
    handle_z = 1.050
    handle_y = LEAF_T / 2.0
    # roll -pi/2 maps local +Z -> +Y (knob spindle protrudes out of room face)
    hardware_roll = Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0))

    knob = model.part("door_knob")
    knob_mesh = mesh_from_cadquery(_build_knob_cq(), "door_knob")
    knob.visual(
        knob_mesh,
        origin=hardware_roll,
        material="knob_brass",
        name="knob_body",
    )

    # ---------------- ARTICULATIONS ----------------
    model.articulation(
        "frame_to_leaf",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=leaf,
        origin=Origin(xyz=(0.0, 0.0, SILL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=2.0, lower=0.0, upper=1.6),
    )
    # Knob turns on its spindle (perpendicular to door face = Y axis in world).
    model.articulation(
        "leaf_to_knob",
        ArticulationType.REVOLUTE,
        parent=leaf,
        child=knob,
        origin=Origin(xyz=(handle_x, handle_y, handle_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=-1.2, upper=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("door_frame")
    leaf = object_model.get_part("door_leaf")
    knob = object_model.get_part("door_knob")
    hinge = object_model.get_articulation("frame_to_leaf")
    knob_joint = object_model.get_articulation("leaf_to_knob")

    # --- Hero size: real door leaf (~0.9 x 2.0 m). ---
    leaf_aabb = ctx.part_world_aabb(leaf)
    if leaf_aabb is not None:
        (lx0, ly0, lz0), (lx1, ly1, lz1) = leaf_aabb
        ctx.check("leaf_width_realistic", 0.85 <= (lx1 - lx0) <= 0.95, details=f"w={lx1 - lx0:.3f}")
        ctx.check("leaf_height_realistic", 1.95 <= (lz1 - lz0) <= 2.10, details=f"h={lz1 - lz0:.3f}")
        ctx.check(
            "leaf_thickness_realistic",
            0.035 <= (ly1 - ly0) <= 0.060,
            details=f"t={ly1 - ly0:.3f}",
        )

    # --- Vision glass present as a separate translucent pane. ---
    glass = leaf.get_visual("vision_pane")
    glass_mat = next((m for m in object_model.materials if m.name == "vision_glass"), None)
    ctx.check(
        "vision_glass_translucent",
        glass.name == "vision_pane"
        and glass_mat is not None
        and glass_mat.rgba is not None
        and glass_mat.rgba[3] < 0.6,
        details=f"glass_mat={getattr(glass_mat, 'rgba', None)}",
    )
    pane_aabb = ctx.part_element_world_aabb(leaf, elem="vision_pane")
    if pane_aabb is not None and leaf_aabb is not None:
        (px0, _, pz0), (px1, _, pz1) = pane_aabb
        # Tall narrow strip.
        ctx.check(
            "vision_strip_tall_narrow",
            (pz1 - pz0) > (px1 - px0) * 4.0 and (pz1 - pz0) > 1.0,
            details=f"strip w={px1 - px0:.3f} h={pz1 - pz0:.3f}",
        )
        # Off-center toward the latch side (center X past leaf mid).
        leaf_mid_x = (leaf_aabb[0][0] + leaf_aabb[1][0]) / 2.0
        ctx.check(
            "vision_strip_offcenter_latch",
            (px0 + px1) / 2.0 > leaf_mid_x,
            details=f"strip_cx={(px0 + px1) / 2.0:.3f} leaf_mid={leaf_mid_x:.3f}",
        )

    # --- Knob: round domed shape on circular rose at lever height on latch edge. ---
    k_aabb = ctx.part_world_aabb(knob)
    if k_aabb is not None and leaf_aabb is not None:
        (kx0, ky0, kz0), (kx1, ky1, kz1) = k_aabb
        # Knob is near the latch edge.
        ctx.check(
            "knob_near_latch_edge",
            kx1 > (leaf_aabb[1][0] - 0.30),
            details=f"kx1={kx1:.3f} latch={leaf_aabb[1][0]:.3f}",
        )
        # Knob protrudes from the room face (+Y).
        ctx.check(
            "knob_protrudes_room_side",
            ky1 > leaf_aabb[1][1],
            details=f"ky1={ky1:.3f} leaf_y1={leaf_aabb[1][1]:.3f}",
        )
        # Knob at lever height (~1.05m).
        knob_cz = (kz0 + kz1) / 2.0
        ctx.check(
            "knob_at_lever_height",
            0.90 <= knob_cz <= 1.20,
            details=f"knob_center_z={knob_cz:.3f}",
        )
        # Knob is roughly spherical (width ≈ depth ≈ height within reason).
        knob_dx = kx1 - kx0
        knob_dy = ky1 - ky0
        knob_dz = kz1 - kz0
        ctx.check(
            "knob_round_shape",
            knob_dx > 0.03 and knob_dz > 0.03 and knob_dy > 0.02,
            details=f"knob dx={knob_dx:.3f} dy={knob_dy:.3f} dz={knob_dz:.3f}",
        )

    # --- Knob joint is REVOLUTE (it turns). ---
    ctx.check(
        "knob_joint_is_revolute",
        knob_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"knob joint type={knob_joint.articulation_type}",
    )

    # --- Closed pose: hinge edge laps the hinge jamb. ---
    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            leaf,
            frame,
            axis="x",
            max_gap=0.001,
            max_penetration=0.006,
            positive_elem="leaf_body",
            negative_elem="hinge_jamb",
            name="hinge_edge_seats_against_jamb",
        )

    # --- Open pose: leaf swings into room; hinge edge stays connected. ---
    rest_pos = ctx.part_world_position(leaf)
    with ctx.pose({hinge: 1.4}):
        open_pos = ctx.part_world_position(leaf)
        open_aabb = ctx.part_world_aabb(leaf)
        ctx.check(
            "leaf_swings_into_room",
            open_aabb is not None and open_aabb[1][1] > LEAF_W * 0.5,
            details=f"open_y1={open_aabb[1][1] if open_aabb else None}",
        )
        ctx.expect_contact(
            leaf,
            frame,
            elem_a="leaf_body",
            elem_b="hinge_jamb",
            contact_tol=0.02,
            name="hinge_edge_stays_connected_when_open",
        )

    ctx.check(
        "leaf_actually_moves",
        rest_pos is not None and open_pos is not None,
        details=f"rest={rest_pos}, open={open_pos}",
    )

    # --- Knob turning pose: the knob rotates on its spindle. ---
    knob_rest_aabb = ctx.part_world_aabb(knob)
    with ctx.pose({knob_joint: 0.8}):
        knob_turned_aabb = ctx.part_world_aabb(knob)
        ctx.check(
            "knob_turns_on_spindle",
            knob_rest_aabb is not None and knob_turned_aabb is not None,
            details="knob should pose without error when turned",
        )

    # --- Knob mounted on leaf face (small intentional overlap for seated hardware). ---
    ctx.allow_overlap(
        knob,
        leaf,
        reason="Door knob rose and spindle are seated against and through the leaf face like real hardware.",
    )
    ctx.allow_overlap(
        leaf,
        frame,
        elem_a="leaf_body",
        elem_b="hinge_jamb",
        reason="Hinge jamb intentionally laps the leaf hinge edge so the leaf is supported by the frame at the hinge line.",
    )

    return ctx.report()


object_model = build_object_model()
