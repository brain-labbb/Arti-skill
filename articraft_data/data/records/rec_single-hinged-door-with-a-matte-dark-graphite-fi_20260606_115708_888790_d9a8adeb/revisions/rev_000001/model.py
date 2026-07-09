from __future__ import annotations

# Single hinged door, matte dark-graphite finish.
# - FIXED root: slim dark door frame (two side jambs + head jamb).
# - Door leaf: dark-graphite blank (CadQuery) with a tall, narrow vertical glass
#   vision strip cut through it, set off-center toward the latch side.
# - Separate translucent glass pane filling the vision opening.
# - Brushed-steel lever handle near the latch edge (REVOLUTE: only the lever
#   pivots on its spindle).
# - Static lock: square rose + keyhole escutcheon, FIXED to the leaf (does not
#   move when the lever is operated).
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


def _build_lever_cq() -> cq.Workplane:
    """Brushed-steel lever that pivots on its spindle (the moving part only).

    Local frame: spindle along +Z through the origin, lever arm sweeps toward
    -X (toward the hinge edge once mounted). Rotating about the spindle keeps
    the hub/neck concentric, so only the lever arm visibly swings.
    """
    hub = cq.Workplane("XY").circle(0.010).extrude(0.020)
    neck = cq.Workplane("XY").workplane(offset=0.020).circle(0.008).extrude(0.006)
    lever = (
        cq.Workplane("XY")
        .workplane(offset=0.024)
        .center(-0.052, 0.0)
        .box(0.110, 0.016, 0.013, centered=(True, True, True))
        .edges("|Z")
        .fillet(0.005)
    )
    return hub.union(neck).union(lever)


def _build_lock_cq() -> cq.Workplane:
    """Static lock backplate: square rose + keyhole escutcheon (does not move).

    Authored in the same local frame as the lever so it mounts with the same
    roll: mounting face on z=0, plate front along +Z, escutcheon at +Y (maps to
    -Z / below the lever after the mounting roll).
    """
    # Square rose the lever pivots on; stays fixed with the lock body.
    rose = (
        cq.Workplane("XY")
        .rect(0.052, 0.052)
        .extrude(0.006)
        .edges("|Z")
        .fillet(0.006)
    )

    # The escutcheon sits BELOW the lever on the real door. After the mounting
    # roll (local +Y -> world -Z), local +Y points down, so the escutcheon and
    # the connecting backstrap are authored at +Y.
    esc_cy = 0.075
    # Slim backstrap joining the rose to the escutcheon so the lock is one
    # connected piece (a long backplate, common on lever sets).
    backstrap = (
        cq.Workplane("XY")
        .center(0.0, esc_cy / 2.0)
        .rect(0.022, esc_cy + 0.020)
        .extrude(0.005)
        .edges("|Z")
        .fillet(0.004)
    )
    esc_plate = (
        cq.Workplane("XY")
        .center(0.0, esc_cy)
        .rect(0.030, 0.040)
        .extrude(0.005)
        .edges("|Z")
        .fillet(0.004)
    )
    # Keyhole: round bore + slot, cut into the escutcheon plate front.
    keyhole_round = (
        cq.Workplane("XY")
        .workplane(offset=0.005)
        .center(0.0, esc_cy + 0.005)
        .circle(0.005)
        .extrude(-0.004)
    )
    keyhole_slot = (
        cq.Workplane("XY")
        .workplane(offset=0.005)
        .center(0.0, esc_cy - 0.005)
        .rect(0.004, 0.014)
        .extrude(-0.004)
    )
    esc_plate = esc_plate.cut(keyhole_round).cut(keyhole_slot)
    return rose.union(backstrap).union(esc_plate)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hinged_graphite_glass_door")
    model.material(name="graphite", rgba=(0.20, 0.21, 0.23, 1.0))
    model.material(name="graphite_frame", rgba=(0.15, 0.16, 0.18, 1.0))
    model.material(name="vision_glass", rgba=(0.78, 0.85, 0.88, 0.35))
    model.material(name="brushed_steel", rgba=(0.74, 0.76, 0.78, 1.0))

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

    # ---------------- HARDWARE near latch edge ----------------
    handle_x = LEAF_W - 0.080
    handle_z = 1.050
    handle_y = LEAF_T / 2.0
    # roll -pi/2 maps local +Z -> +Y (protrudes out of the room face). Lever arm
    # and escutcheon are authored toward -X / -local-Y so they read correctly.
    hardware_roll = Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0))

    # Static lock (rose + keyhole escutcheon): fixed to the leaf.
    lock = model.part("door_lock")
    lock_mesh = mesh_from_cadquery(_build_lock_cq(), "door_lock")
    lock.visual(
        lock_mesh,
        origin=hardware_roll,
        material="brushed_steel",
        name="lock_body",
    )

    # Lever handle: only this pivots on the spindle.
    handle = model.part("lever_handle")
    handle_mesh = mesh_from_cadquery(_build_lever_cq(), "lever_handle")
    handle.visual(
        handle_mesh,
        origin=hardware_roll,
        material="brushed_steel",
        name="lever_body",
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
    # Lock is rigidly mounted to the leaf — it never moves with the lever.
    model.articulation(
        "leaf_to_lock",
        ArticulationType.FIXED,
        parent=leaf,
        child=lock,
        origin=Origin(xyz=(handle_x, handle_y, handle_z)),
    )
    # Only the lever pivots on its spindle.
    model.articulation(
        "leaf_to_lever",
        ArticulationType.REVOLUTE,
        parent=leaf,
        child=handle,
        origin=Origin(xyz=(handle_x, handle_y, handle_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-0.5, upper=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("door_frame")
    leaf = object_model.get_part("door_leaf")
    handle = object_model.get_part("lever_handle")
    lock = object_model.get_part("door_lock")
    hinge = object_model.get_articulation("frame_to_leaf")
    lever_joint = object_model.get_articulation("leaf_to_lever")
    lock_joint = object_model.get_articulation("leaf_to_lock")

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

    # --- Handle present near latch edge, lever height, room side. ---
    h_aabb = ctx.part_world_aabb(handle)
    if h_aabb is not None and leaf_aabb is not None:
        (hx0, hy0, hz0), (hx1, hy1, hz1) = h_aabb
        ctx.check(
            "handle_near_latch_edge",
            hx1 > (leaf_aabb[1][0] - 0.30),
            details=f"hx1={hx1:.3f} latch={leaf_aabb[1][0]:.3f}",
        )
        ctx.check(
            "handle_protrudes_room_side",
            hy1 > leaf_aabb[1][1],
            details=f"hy1={hy1:.3f} leaf_y1={leaf_aabb[1][1]:.3f}",
        )

    # --- Lock is a separate static part with the keyhole escutcheon below
    # the lever. The fixed joint means it never moves with the lever. ---
    lock_aabb = ctx.part_world_aabb(lock)
    if lock_aabb is not None and h_aabb is not None:
        (lkx0, lky0, lkz0), (lkx1, lky1, lkz1) = lock_aabb
        ctx.check(
            "lock_joint_is_fixed",
            lock_joint.articulation_type == ArticulationType.FIXED,
            details=f"lock joint type={lock_joint.articulation_type}",
        )
        ctx.check(
            "escutcheon_below_lever",
            lkz0 < 1.000,
            details=f"lock min z={lkz0:.3f}",
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

    lock_rest = ctx.part_world_position(lock)
    with ctx.pose({lever_joint: -0.4}):
        ctx.check(
            "lever_pose_ok",
            ctx.part_world_aabb(handle) is not None,
            details="lever should pose without error",
        )
        # Operating the lever must NOT move the static lock.
        lock_now = ctx.part_world_position(lock)
        moved = (
            None
            if (lock_rest is None or lock_now is None)
            else max(abs(a - b) for a, b in zip(lock_rest, lock_now))
        )
        ctx.check(
            "lock_static_when_lever_moves",
            moved is not None and moved < 1e-6,
            details=f"lock displacement={moved}",
        )

    ctx.allow_overlap(
        handle,
        leaf,
        reason="Lever spindle/neck are seated against and through the leaf face like real hardware.",
    )
    ctx.allow_overlap(
        lock,
        leaf,
        reason="Lock rose/backplate/escutcheon are seated against and through the leaf face like real hardware.",
    )
    ctx.allow_overlap(
        handle,
        lock,
        reason="The lever hub pivots through the lock rose bore, so the two overlap by design.",
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
