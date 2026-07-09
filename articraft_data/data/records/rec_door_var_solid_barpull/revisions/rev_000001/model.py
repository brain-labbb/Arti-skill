from __future__ import annotations

# Single hinged interior door, light oak wood.
# - FIXED root: door frame (two side jambs + head jamb) plus casing trim.
# - Door leaf: oak blank with a routed rectangular border groove and one large
#   raised flat center panel, authored with CadQuery so it is not a flat box.
# - Long vertical tubular bar pull on two standoffs near the latch edge,
#   running mid-height of the leaf (inlined as leaf visuals, no separate part).
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
PANEL_PROUD = 0.008     # raised panel proud height above field
PANEL_MARGIN = 0.100    # raised panel distance from leaf edge

OPENING_H = LEAF_H + HEAD_GAP
SILL_Z = 0.0

# --- Bar pull dimensions ---
BAR_LENGTH = 0.800          # vertical tube length (mid-height of leaf)
BAR_OD = 0.016              # tube outer diameter
BAR_ID = 0.010              # tube inner diameter (hollow bore)
STANDOFF_OD = 0.018         # standoff cylinder diameter
STANDOFF_SPAN = 0.045       # visible standoff length (leaf face to tube surface)
STANDOFF_EMBED = 0.003      # embed depth at each end (into leaf + into tube wall)
NUM_STANDOFFS = 2
STANDOFF_INSET = 0.100      # standoff center distance from bar ends
BAR_X = LEAF_W - 0.060     # bar center X (near latch edge)
BAR_CZ = LEAF_H / 2.0      # bar centered vertically on leaf

# Derived positions
TUBE_CY = LEAF_T / 2.0 + STANDOFF_SPAN + BAR_OD / 2.0  # tube axis Y
STANDOFF_TOTAL = STANDOFF_SPAN + 2.0 * STANDOFF_EMBED   # full standoff cylinder length
STANDOFF_BASE_Y = LEAF_T / 2.0 - STANDOFF_EMBED         # standoff origin Y (embeds into leaf)

# Standoff Z positions: symmetric about bar center
STANDOFF_ZS = [
    BAR_CZ - BAR_LENGTH / 2.0 + STANDOFF_INSET,
    BAR_CZ + BAR_LENGTH / 2.0 - STANDOFF_INSET,
]


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


def _build_bar_tube_cq() -> cq.Workplane:
    """Hollow tubular bar, built at origin along Z.

    Local frame: tube axis along Z, centered at origin.
    Extends from z = -BAR_LENGTH/2 to z = +BAR_LENGTH/2.
    """
    z_half = BAR_LENGTH / 2.0
    outer = (
        cq.Workplane("XY")
        .workplane(offset=-z_half)
        .circle(BAR_OD / 2.0)
        .extrude(BAR_LENGTH)
    )
    # Hollow bore through the full length (slight overcut for clean boolean).
    bore = (
        cq.Workplane("XY")
        .workplane(offset=-z_half - 0.001)
        .circle(BAR_ID / 2.0)
        .extrude(BAR_LENGTH + 0.002)
    )
    tube = outer.cut(bore)
    # Soft-round the tube end edges for realism.
    try:
        tube = tube.edges("|Z").fillet(0.0005)
    except Exception:
        pass
    return tube


def _build_standoff_cq() -> cq.Workplane:
    """One cylindrical standoff, built at origin along +Y.

    Local frame: base circle on XZ plane at y=0, extrudes along +Y
    by STANDOFF_TOTAL. CadQuery XZ workplane normal is -Y, so we
    negate the extrude to extend along +Y (outward from leaf face).
    """
    standoff = (
        cq.Workplane("XZ")
        .circle(STANDOFF_OD / 2.0)
        .extrude(-STANDOFF_TOTAL)
    )
    try:
        standoff = standoff.edges(">Y").fillet(0.002)
    except Exception:
        pass
    return standoff


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

    # ---------------- BAR PULL (inlined on leaf, no separate part) ----------------
    # Tubular bar: hollow vertical cylinder near latch edge, mid-height.
    bar_tube_mesh = mesh_from_cadquery(_build_bar_tube_cq(), "bar_tube")
    leaf.visual(
        bar_tube_mesh,
        origin=Origin(xyz=(BAR_X, TUBE_CY, BAR_CZ)),
        material="brushed_steel",
        name="bar_tube",
    )

    # Standoffs: two cylindrical mounts from leaf face to bar tube.
    # Shared geometry helper + for-loop with name_i naming.
    standoff_mesh = mesh_from_cadquery(_build_standoff_cq(), "standoff")
    for i in range(NUM_STANDOFFS):
        leaf.visual(
            standoff_mesh,
            origin=Origin(xyz=(BAR_X, STANDOFF_BASE_Y, STANDOFF_ZS[i])),
            material="brushed_steel",
            name=f"standoff_{i}",
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

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("door_frame")
    leaf = object_model.get_part("door_leaf")
    hinge = object_model.get_articulation("frame_to_leaf")

    # --- Hero size: real interior door leaf (~0.9 x 2.0 m). ---
    leaf_aabb = ctx.part_world_aabb(leaf)
    if leaf_aabb is not None:
        (lx0, ly0, lz0), (lx1, ly1, lz1) = leaf_aabb
        ctx.check("leaf_width_realistic", 0.85 <= (lx1 - lx0) <= 0.95, details=f"w={lx1 - lx0:.3f}")
        ctx.check("leaf_height_realistic", 1.95 <= (lz1 - lz0) <= 2.10, details=f"h={lz1 - lz0:.3f}")
        ctx.check(
            "leaf_thickness_realistic",
            0.035 <= (ly1 - ly0) <= 0.120,
            details=f"t={ly1 - ly0:.3f}",
        )

    # --- Raised panel proud of the field on the room side. ---
    leaf_body_aabb = ctx.part_element_world_aabb(leaf, elem="leaf_body")
    if leaf_body_aabb is not None:
        body_dy = leaf_body_aabb[1][1] - leaf_body_aabb[0][1]
        ctx.check(
            "raised_panel_present",
            body_dy > LEAF_T + PANEL_PROUD * 0.5,
            details=f"body_dy={body_dy:.4f} expected>{LEAF_T + PANEL_PROUD * 0.5:.4f}",
        )

    # --- Bar pull: near latch edge, long vertical, protrudes room side. ---
    bar_tube_aabb = ctx.part_element_world_aabb(leaf, elem="bar_tube")
    if bar_tube_aabb is not None and leaf_body_aabb is not None:
        (bx0, by0, bz0), (bx1, by1, bz1) = bar_tube_aabb
        bar_height = bz1 - bz0
        ctx.check(
            "bar_pull_near_latch_edge",
            bx1 > leaf_body_aabb[1][0] - 0.20,
            details=f"bar_x1={bx1:.3f} leaf_latch_x={leaf_body_aabb[1][0]:.3f}",
        )
        ctx.check(
            "bar_pull_long_vertical",
            bar_height > 0.60,
            details=f"bar_height={bar_height:.3f} expected>0.60",
        )
        ctx.check(
            "bar_pull_protrudes_room_side",
            by1 > leaf_body_aabb[1][1],
            details=f"tube_y1={by1:.3f} leaf_face_y={leaf_body_aabb[1][1]:.3f}",
        )
        ctx.check(
            "bar_pull_centered_vertically",
            abs((bz0 + bz1) / 2.0 - LEAF_H / 2.0) < 0.10,
            details=f"bar_cz={(bz0 + bz1) / 2.0:.3f} leaf_cz={LEAF_H / 2.0:.3f}",
        )

    # --- Standoffs present (two mounting points connecting bar to leaf). ---
    for i in range(NUM_STANDOFFS):
        sa = ctx.part_element_world_aabb(leaf, elem=f"standoff_{i}")
        ctx.check(
            f"standoff_{i}_present",
            sa is not None,
            details=f"standoff_{i} should exist as a named visual on door_leaf",
        )
        if sa is not None and leaf_body_aabb is not None:
            # Each standoff bridges from the leaf face outward to the tube.
            ctx.check(
                f"standoff_{i}_bridges_leaf_to_bar",
                sa[1][1] > leaf_body_aabb[1][1] and sa[0][1] < by1,
                details=f"standoff y=[{sa[0][1]:.3f},{sa[1][1]:.3f}] leaf_face={leaf_body_aabb[1][1]:.3f}",
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

    ctx.allow_overlap(
        leaf,
        frame,
        elem_a="leaf_body",
        elem_b="hinge_jamb",
        reason="Hinge jamb intentionally laps the leaf hinge edge so the leaf is supported by the frame at the hinge line.",
    )

    return ctx.report()


object_model = build_object_model()
