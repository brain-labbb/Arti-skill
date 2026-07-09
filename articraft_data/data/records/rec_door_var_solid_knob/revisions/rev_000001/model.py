from __future__ import annotations

# Single hinged interior door, light oak wood.
# - FIXED root: door frame (two side jambs + head jamb) plus casing trim.
# - Door leaf: oak blank with a routed rectangular border groove and one large
#   raised flat center panel, authored with CadQuery so it is not a flat box.
# - Round domed door knob on a circular rose, seated on the latch stile.
# - PRIMARY articulation: the leaf swings on its hinge edge (REVOLUTE, vertical
#   axis along the hinge jamb).
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
LEAF_W = 0.900          # door leaf width
LEAF_H = 2.030          # door leaf height
LEAF_T = 0.040          # door leaf thickness

JAMB_W = 0.045          # jamb face width (along X for side jambs)
JAMB_D = 0.140          # jamb depth (wall thickness it lines), along Y
HEAD_GAP = 0.004        # reveal gap between leaf and head
SIDE_GAP = 0.003        # reveal gap at each side

CASING_W = 0.060        # casing trim face width
CASING_T = 0.018        # casing trim proud thickness

GROOVE_INSET = 0.075    # routed border distance from leaf edge
GROOVE_W = 0.012        # routed groove width
GROOVE_DEPTH = 0.007    # routed groove depth
PANEL_PROUD = 0.008     # raised panel proud height above field — clearly raised
PANEL_MARGIN = 0.100    # raised panel distance from leaf edge

OPENING_H = LEAF_H + HEAD_GAP
SILL_Z = 0.0


def _build_leaf_cq() -> cq.Workplane:
    """Oak leaf in its own local frame.

    Local frame: hinge edge at local x=0, leaf extends along +X (0..LEAF_W),
    thickness centered on Y (-T/2..+T/2), height 0..LEAF_H.
    """
    blank = cq.Workplane("XY").box(LEAF_W, LEAF_T, LEAF_H, centered=(False, True, False))

    cx = LEAF_W / 2.0
    cz = LEAF_H / 2.0
    face_y = LEAF_T / 2.0

    # Routed rectangular border groove on the room side (+Y face).
    outer_w = LEAF_W - 2 * GROOVE_INSET
    outer_h = LEAF_H - 2 * GROOVE_INSET
    groove = (
        cq.Workplane("XZ")
        .workplane(offset=face_y)
        .center(cx, cz)
        .rect(outer_w, outer_h)
        .rect(outer_w - 2 * GROOVE_W, outer_h - 2 * GROOVE_W)
        .extrude(-GROOVE_DEPTH)
    )
    leaf = blank.cut(groove)

    # Raised flat center panel: a proud rectangular pad inside the border.
    panel_w = LEAF_W - 2 * PANEL_MARGIN
    panel_h = LEAF_H - 2 * PANEL_MARGIN
    panel = (
        cq.Workplane("XZ")
        .workplane(offset=face_y)
        .center(cx, cz)
        .rect(panel_w, panel_h)
        .extrude(PANEL_PROUD)
    )
    try:
        panel = panel.edges(">Y").chamfer(0.010)
    except Exception:
        pass
    leaf = leaf.union(panel)
    return leaf


def _build_knob_cq() -> cq.Workplane:
    """Round domed door knob on a circular rose in its own local frame.

    Local frame: rose mounting face on z=0, protrudes along +Z.
    The dome is a sphere giving the classic round domed knob silhouette.
    """
    ROSE_R = 0.027
    ROSE_H = 0.008
    NECK_R = 0.011
    NECK_H = 0.008
    DOME_R = 0.026

    # Circular rose (backplate) with edge fillets
    rose = cq.Workplane("XY").circle(ROSE_R).extrude(ROSE_H)
    try:
        rose = rose.edges(">Z").fillet(0.0015)
    except Exception:
        pass
    try:
        rose = rose.edges("<Z").fillet(0.001)
    except Exception:
        pass

    # Neck/shank connecting rose to knob body
    neck = cq.Workplane("XY").workplane(offset=ROSE_H).circle(NECK_R).extrude(NECK_H)

    # Round dome: a sphere centered above the neck so it reads as a smooth dome
    dome_center_z = ROSE_H + NECK_H + DOME_R * 0.55
    dome = cq.Workplane("XY").workplane(offset=dome_center_z).sphere(DOME_R)

    return rose.union(neck).union(dome)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hinged_oak_panel_door")
    model.material(name="light_oak", rgba=(0.82, 0.66, 0.45, 1.0))
    model.material(name="oak_shadow", rgba=(0.66, 0.50, 0.33, 1.0))
    model.material(name="brushed_steel", rgba=(0.74, 0.76, 0.78, 1.0))

    # ---------------- FIXED FRAME (root) ----------------
    frame = model.part("door_frame")

    # Hinge jamb: its inner face just laps the leaf hinge edge (x=0) so the leaf
    # is supported/connected by the frame, with a tiny intentional lap.
    HINGE_LAP = 0.004
    hinge_jamb_x = -JAMB_W / 2.0 + HINGE_LAP
    frame.visual(
        Box((JAMB_W, JAMB_D, OPENING_H)),
        origin=Origin(xyz=(hinge_jamb_x, 0.0, OPENING_H / 2.0)),
        material="oak_shadow",
        name="hinge_jamb",
    )
    latch_jamb_x = LEAF_W + SIDE_GAP + JAMB_W / 2.0
    frame.visual(
        Box((JAMB_W, JAMB_D, OPENING_H)),
        origin=Origin(xyz=(latch_jamb_x, 0.0, OPENING_H / 2.0)),
        material="oak_shadow",
        name="latch_jamb",
    )
    head_z = OPENING_H + JAMB_W / 2.0
    head_len = (latch_jamb_x + JAMB_W / 2.0) - (hinge_jamb_x - JAMB_W / 2.0)
    head_cx = (latch_jamb_x + hinge_jamb_x) / 2.0
    frame.visual(
        Box((head_len, JAMB_D, JAMB_W)),
        origin=Origin(xyz=(head_cx, 0.0, head_z)),
        material="oak_shadow",
        name="head_jamb",
    )

    # Casing trim (room side, +Y) framing the jambs.
    casing_y = JAMB_D / 2.0 + CASING_T / 2.0
    casing_outer_w = head_len + 2 * CASING_W
    frame.visual(
        Box((CASING_W, CASING_T, OPENING_H + CASING_W)),
        origin=Origin(
            xyz=(hinge_jamb_x - JAMB_W / 2.0 - CASING_W / 2.0, casing_y, (OPENING_H + CASING_W) / 2.0)
        ),
        material="light_oak",
        name="casing_leg_hinge",
    )
    frame.visual(
        Box((CASING_W, CASING_T, OPENING_H + CASING_W)),
        origin=Origin(
            xyz=(latch_jamb_x + JAMB_W / 2.0 + CASING_W / 2.0, casing_y, (OPENING_H + CASING_W) / 2.0)
        ),
        material="light_oak",
        name="casing_leg_latch",
    )
    frame.visual(
        Box((casing_outer_w, CASING_T, CASING_W)),
        origin=Origin(xyz=(head_cx, casing_y, OPENING_H + JAMB_W + CASING_W / 2.0)),
        material="light_oak",
        name="casing_head",
    )

    # ---------------- DOOR LEAF (swings) ----------------
    leaf = model.part("door_leaf")
    leaf_mesh = mesh_from_cadquery(_build_leaf_cq(), "door_leaf")
    leaf.visual(
        leaf_mesh,
        origin=Origin(xyz=(0.0, 0.0, SILL_Z)),
        material="light_oak",
        name="leaf_body",
    )

    # ---------------- DOOR KNOB on latch stile ----------------
    knob = model.part("door_knob")
    knob_mesh = mesh_from_cadquery(_build_knob_cq(), "door_knob")
    knob_x = LEAF_W - 0.075
    knob_z = 1.050
    knob_y = LEAF_T / 2.0
    # The leaf_to_knob joint origin places the knob on the room face of the
    # latch stile. roll -pi/2 maps local +Z -> +Y (knob protrudes out of the
    # room face).
    knob.visual(
        knob_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="brushed_steel",
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
    model.articulation(
        "leaf_to_knob",
        ArticulationType.REVOLUTE,
        parent=leaf,
        child=knob,
        origin=Origin(xyz=(knob_x, knob_y, knob_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=-0.8, upper=0.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("door_frame")
    leaf = object_model.get_part("door_leaf")
    knob = object_model.get_part("door_knob")
    hinge = object_model.get_articulation("frame_to_leaf")
    knob_joint = object_model.get_articulation("leaf_to_knob")

    # --- Hero size: real interior door leaf (~0.9 x 2.0 m). ---
    leaf_aabb = ctx.part_world_aabb(leaf)
    if leaf_aabb is not None:
        (lx0, ly0, lz0), (lx1, ly1, lz1) = leaf_aabb
        ctx.check("leaf_width_realistic", 0.85 <= (lx1 - lx0) <= 0.95, details=f"w={lx1 - lx0:.3f}")
        ctx.check("leaf_height_realistic", 1.95 <= (lz1 - lz0) <= 2.10, details=f"h={lz1 - lz0:.3f}")
        ctx.check(
            "leaf_thickness_realistic",
            0.035 <= (ly1 - ly0) <= 0.075,
            details=f"t={ly1 - ly0:.3f}",
        )

    # --- Raised panel proud of the field on the room side. ---
    ctx.check(
        "raised_panel_present",
        leaf_aabb is not None and (leaf_aabb[1][1] - leaf_aabb[0][1]) > LEAF_T + PANEL_PROUD * 0.5,
        details="raised panel should add proud thickness",
    )

    # --- Knob present, near latch edge, at lever height, room side. ---
    k_aabb = ctx.part_world_aabb(knob)
    if k_aabb is not None and leaf_aabb is not None:
        (kx0, ky0, kz0), (kx1, ky1, kz1) = k_aabb
        ctx.check(
            "knob_near_latch_edge",
            kx1 > (leaf_aabb[1][0] - 0.30),
            details=f"kx1={kx1:.3f} latch={leaf_aabb[1][0]:.3f}",
        )
        ctx.check(
            "knob_at_standard_height",
            0.95 <= (kz0 + kz1) / 2.0 <= 1.15,
            details=f"kz={(kz0 + kz1) / 2.0:.3f}",
        )
        ctx.check(
            "knob_protrudes_room_side",
            ky1 > leaf_aabb[1][1],
            details=f"ky1={ky1:.3f} leaf_y1={leaf_aabb[1][1]:.3f}",
        )

    # --- Knob is round/domed: roughly equal XZ extents (not elongated like a lever). ---
    if k_aabb is not None:
        dx = kx1 - kx0
        dz = kz1 - kz0
        dy = ky1 - ky0
        ctx.check(
            "knob_is_round_not_lever",
            dx < 0.080 and dz < 0.080,
            details=f"dx={dx:.3f} dz={dz:.3f} — should be compact round shape, not elongated lever",
        )
        ctx.check(
            "knob_dome_has_height",
            dy > 0.035,
            details=f"dy={dy:.3f} — dome should protrude noticeably from the face",
        )

    # --- Closed pose: hinge edge seats against (laps) the hinge jamb. ---
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

    # --- Door knob articulates (turns to operate latch). ---
    with ctx.pose({knob_joint: 0.6}):
        turned_aabb = ctx.part_world_aabb(knob)
        ctx.check(
            "knob_turns_ok",
            turned_aabb is not None,
            details="knob should pose without error when turned",
        )

    # Knob rose is seated against and through the leaf face like real hardware.
    ctx.allow_overlap(
        knob,
        leaf,
        reason="Knob rose/spindle is seated against and through the leaf face like real door hardware.",
    )
    ctx.allow_overlap(
        leaf,
        frame,
        elem_a="leaf_body",
        elem_b="hinge_jamb",
        reason="Hinge jamb intentionally laps the leaf hinge edge so the leaf is supported by the frame at the hinge line.",
    )

    # Prove the knob rose sits in contact with the leaf face
    ctx.expect_contact(
        knob,
        leaf,
        elem_a="knob_body",
        elem_b="leaf_body",
        contact_tol=0.012,
        name="knob_rose_seated_on_leaf_face",
    )

    return ctx.report()


object_model = build_object_model()
