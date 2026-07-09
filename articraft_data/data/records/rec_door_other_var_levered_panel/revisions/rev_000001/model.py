from __future__ import annotations

# Dutch (stable) door, split horizontally into two independently swinging leaves.
#
# - FIXED root: wood door frame (two side jambs + head jamb) plus casing trim,
#   standing on the ground plane (geometry from z=0 upward). A low threshold
#   sits on the floor between the jambs.
# - UPPER LEAF: solid wood leaf with a routed border groove and one raised
#   center panel (matching the lower leaf).
# - LOWER LEAF: solid wood leaf with a routed border groove and one raised
#   center panel (CadQuery, so it reads as a real raised panel, not a flat box).
# - Both leaves hinge on the SAME jamb (the hinge edge at local x=0) and swing
#   on VERTICAL (Z) axes. The two revolute joints are at different heights:
#   the upper hinge sits above the mid split, the lower hinge below it.
#   The leaves open INDEPENDENTLY (two separate joints, not coupled).
# - Visible barrel hinges (two per leaf) read as the real pivot hardware.
# - Lever handle on a rectangular backplate mounted on the lower leaf latch
#   edge, room side.
#
# Frame convention (matches the single-leaf door analog):
#   X = door width   (hinge edge at small/0 X, latch edge at large X)
#   Y = door thickness (room side at +Y)
#   Z = height        (floor at z=0, door rises to ~2.0 m)

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# --- Real-world dimensions (meters) ---
LEAF_W = 0.900          # door width (each leaf spans the full width)
DOOR_H = 2.020          # full door height (both leaves stacked)
LEAF_T = 0.044          # leaf thickness

# Horizontal split between the two leaves.
SPLIT_Z = 1.060         # height of the split line (mid-rail) above the floor
SPLIT_GAP = 0.004       # reveal gap between the two leaves at the split
THRESHOLD_H = 0.020     # low sill the lower leaf clears

LOWER_BOTTOM = THRESHOLD_H               # lower leaf starts just above threshold
LOWER_TOP = SPLIT_Z - SPLIT_GAP / 2.0    # lower leaf top edge
UPPER_BOTTOM = SPLIT_Z + SPLIT_GAP / 2.0 # upper leaf bottom edge
UPPER_TOP = DOOR_H                       # upper leaf top edge

LOWER_H = LOWER_TOP - LOWER_BOTTOM       # lower leaf height (~1.03 m)
UPPER_H = UPPER_TOP - UPPER_BOTTOM       # upper leaf height (~0.96 m)

# Frame (jambs / head).
JAMB_W = 0.050          # jamb face width (along X for side jambs)
JAMB_D = 0.150          # jamb depth (wall thickness it lines), along Y
HEAD_GAP = 0.006        # reveal gap between upper leaf and head
SIDE_GAP = 0.004        # reveal gap at each side
HINGE_LAP = 0.005       # jamb laps the leaf hinge edge so the leaf is carried

CASING_W = 0.065        # casing trim face width
CASING_T = 0.020        # casing trim proud thickness

OPENING_H = DOOR_H + HEAD_GAP

# --- Raised-panel construction (shared by both leaves) ---
# Reference: one large recessed/raised panel framed by a clear stepped border.
GROOVE_INSET = 0.100    # routed border distance from leaf edge
GROOVE_W = 0.016        # routed groove width
GROOVE_DEPTH = 0.009    # routed groove depth (deeper -> border reads clearly)
PANEL_PROUD = 0.011     # raised panel proud height above the field
PANEL_MARGIN = 0.130    # raised panel distance from leaf edge

# --- Hinge hardware ---
HINGE_LEN = 0.100       # barrel hinge length
HINGE_R = 0.011         # barrel outer radius

# --- Lever handle dimensions ---
BACKPLATE_W = 0.040     # backplate width (vertical extent on door face)
BACKPLATE_H = 0.140     # backplate height (vertical extent on door face)
BACKPLATE_T = 0.008     # backplate thickness (proud of door face)
LEVER_LEN = 0.120       # lever arm length (horizontal reach)
LEVER_DIA = 0.018       # lever arm diameter
LEVER_BASE_R = 0.022    # pivot boss radius at the backplate


def _raised_panel_leaf_cq(width: float, height: float, thickness: float) -> cq.Workplane:
    """Solid wood leaf with a routed border groove + one raised center panel.

    Local frame: hinge edge at local x=0, leaf extends along +X (0..width),
    thickness centered on Y, height 0..height.
    """
    blank = cq.Workplane("XY").box(width, thickness, height, centered=(False, True, False))

    cx = width / 2.0
    cz = height / 2.0
    face_y = thickness / 2.0

    # Routed rectangular border groove on the room side (+Y face).
    outer_w = width - 2 * GROOVE_INSET
    outer_h = height - 2 * GROOVE_INSET
    groove = (
        cq.Workplane("XZ")
        .workplane(offset=face_y)
        .center(cx, cz)
        .rect(outer_w, outer_h)
        .rect(outer_w - 2 * GROOVE_W, outer_h - 2 * GROOVE_W)
        .extrude(-GROOVE_DEPTH)
    )
    leaf = blank.cut(groove)

    # Raised flat center panel: a proud rectangular pad inside the border, with
    # a chamfered (raised-panel) edge.
    panel_w = width - 2 * PANEL_MARGIN
    panel_h = height - 2 * PANEL_MARGIN
    panel = (
        cq.Workplane("XZ")
        .workplane(offset=face_y)
        .center(cx, cz)
        .rect(panel_w, panel_h)
        .extrude(PANEL_PROUD)
    )
    # Chamfer the proud face edge so it reads as a true bevelled raised panel.
    try:
        panel = panel.edges(">Y").chamfer(min(0.010, PANEL_PROUD - 0.001))
    except Exception:
        pass
    leaf = leaf.union(panel)
    return leaf


def _hinge_barrel_mesh(name: str):
    """A simple visible barrel hinge: a knuckle barrel with two leaf plates.

    Built around a local Z pin axis (matches the leaf's vertical hinge axis).
    Local frame: barrel centered on x=0, base on z=0. One leaf plate reaches
    toward -X (the jamb) and one toward +X (the door leaf), both lying in the
    leaf's room-side face plane (thin in Y).
    """
    barrel = cq.Workplane("XY").circle(HINGE_R).extrude(HINGE_LEN)
    plate_w = 0.038
    plate_t = 0.004
    # Jamb-side leaf plate: extends toward -X from the barrel.
    plate_jamb = (
        cq.Workplane("XY")
        .transformed(offset=cq.Vector(-plate_w / 2.0 - HINGE_R * 0.5, 0.0, HINGE_LEN / 2.0))
        .box(plate_w, plate_t, HINGE_LEN * 0.85, centered=(True, True, True))
    )
    # Leaf-side leaf plate: extends toward +X onto the door stile.
    plate_leaf = (
        cq.Workplane("XY")
        .transformed(offset=cq.Vector(plate_w / 2.0 + HINGE_R * 0.5, 0.0, HINGE_LEN / 2.0))
        .box(plate_w, plate_t, HINGE_LEN * 0.85, centered=(True, True, True))
    )
    hinge = barrel.union(plate_jamb).union(plate_leaf)
    return mesh_from_cadquery(hinge, name)


def _lever_handle_mesh(name: str):
    """Horizontal lever handle on a rectangular backplate.

    Local frame: centered on origin. The backplate is thin in Y (flush against
    the door +Y face), wide in X, tall in Z. The lever arm extends along +Y
    (outward from the door face).
    """
    boss_len = 0.014
    shaft_total_len = BACKPLATE_T + boss_len + LEVER_LEN

    # Rectangular backplate (escutcheon): thin in Y, wide in X, tall in Z.
    bp = (
        cq.Workplane("XY")
        .box(BACKPLATE_W, BACKPLATE_T, BACKPLATE_H, centered=(True, True, True))
    )
    try:
        bp = bp.edges("|Y").fillet(0.004)
    except Exception:
        pass

    # Continuous lever shaft: a box running along +Y from inside the backplate
    # through the boss region and out to the lever tip. Overlaps the backplate
    # for mesh connectivity.
    shaft_cy = -BACKPLATE_T / 2.0 + shaft_total_len / 2.0
    shaft = (
        cq.Workplane("XY")
        .transformed(offset=cq.Vector(0.0, shaft_cy, 0.0))
        .box(LEVER_DIA, shaft_total_len, LEVER_DIA, centered=(True, True, True))
    )
    # Fillet longitudinal edges for a rounded lever profile.
    try:
        shaft = shaft.edges("|Y").fillet(LEVER_DIA * 0.25)
    except Exception:
        pass
    handle = bp.union(shaft)

    # Pivot boss: larger box around the shaft at the +Y face of the backplate,
    # reading as the rosette/pivot housing. Overlaps both backplate and shaft.
    boss_ext = boss_len + 0.004
    boss_cy = BACKPLATE_T / 2.0 - 0.004 + boss_ext / 2.0
    boss = (
        cq.Workplane("XY")
        .transformed(offset=cq.Vector(0.0, boss_cy, 0.0))
        .box(LEVER_BASE_R * 2.0, boss_ext, LEVER_BASE_R * 2.0, centered=(True, True, True))
    )
    try:
        boss = boss.edges("|Y").fillet(LEVER_BASE_R * 0.4)
    except Exception:
        pass
    handle = handle.union(boss)

    # Rounded grip return at the end of the lever.
    tip_y = -BACKPLATE_T / 2.0 + shaft_total_len
    tip = (
        cq.Workplane("XY")
        .transformed(offset=cq.Vector(0.0, tip_y, 0.0))
        .sphere(LEVER_DIA * 0.7)
    )
    handle = handle.union(tip)

    return mesh_from_cadquery(handle, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dutch_stable_door")
    # Light natural pine: warm tan with a touch more lift than the raw blank so
    # the leaf reads as fresh pine, with a slightly darker/cooler shadow tone
    # for the recessed jambs and grooves.
    model.material(name="pine_wood", rgba=(0.82, 0.66, 0.45, 1.0))
    model.material(name="pine_shadow", rgba=(0.66, 0.51, 0.33, 1.0))
    model.material(name="dark_iron", rgba=(0.20, 0.20, 0.22, 1.0))
    model.material(name="brushed_steel", rgba=(0.72, 0.74, 0.76, 1.0))

    # ---------------- FIXED FRAME (root) ----------------
    frame = model.part("door_frame")

    hinge_jamb_x = -JAMB_W / 2.0 + HINGE_LAP
    frame.visual(
        Box((JAMB_W, JAMB_D, OPENING_H)),
        origin=Origin(xyz=(hinge_jamb_x, 0.0, OPENING_H / 2.0)),
        material="pine_shadow",
        name="hinge_jamb",
    )
    latch_jamb_x = LEAF_W + SIDE_GAP + JAMB_W / 2.0
    frame.visual(
        Box((JAMB_W, JAMB_D, OPENING_H)),
        origin=Origin(xyz=(latch_jamb_x, 0.0, OPENING_H / 2.0)),
        material="pine_shadow",
        name="latch_jamb",
    )
    head_z = OPENING_H + JAMB_W / 2.0
    head_len = (latch_jamb_x + JAMB_W / 2.0) - (hinge_jamb_x - JAMB_W / 2.0)
    head_cx = (latch_jamb_x + hinge_jamb_x) / 2.0
    frame.visual(
        Box((head_len, JAMB_D, JAMB_W)),
        origin=Origin(xyz=(head_cx, 0.0, head_z)),
        material="pine_shadow",
        name="head_jamb",
    )

    # Low threshold/sill on the floor between the jambs (door stands on ground).
    sill_len = LEAF_W + SIDE_GAP
    frame.visual(
        Box((sill_len, JAMB_D, THRESHOLD_H)),
        origin=Origin(xyz=(sill_len / 2.0, 0.0, THRESHOLD_H / 2.0)),
        material="pine_shadow",
        name="threshold",
    )

    # Casing trim (room side, +Y) framing the jambs.
    casing_y = JAMB_D / 2.0 + CASING_T / 2.0
    casing_outer_w = head_len + 2 * CASING_W
    frame.visual(
        Box((CASING_W, CASING_T, OPENING_H + CASING_W)),
        origin=Origin(
            xyz=(
                hinge_jamb_x - JAMB_W / 2.0 - CASING_W / 2.0,
                casing_y,
                (OPENING_H + CASING_W) / 2.0,
            )
        ),
        material="pine_wood",
        name="casing_leg_hinge",
    )
    frame.visual(
        Box((CASING_W, CASING_T, OPENING_H + CASING_W)),
        origin=Origin(
            xyz=(
                latch_jamb_x + JAMB_W / 2.0 + CASING_W / 2.0,
                casing_y,
                (OPENING_H + CASING_W) / 2.0,
            )
        ),
        material="pine_wood",
        name="casing_leg_latch",
    )
    frame.visual(
        Box((casing_outer_w, CASING_T, CASING_W)),
        origin=Origin(xyz=(head_cx, casing_y, OPENING_H + JAMB_W + CASING_W / 2.0)),
        material="pine_wood",
        name="casing_head",
    )

    # ---------------- UPPER LEAF (solid raised panel, swings) ----------------
    upper = model.part("upper_leaf")
    upper_mesh = mesh_from_cadquery(
        _raised_panel_leaf_cq(LEAF_W, UPPER_H, LEAF_T), "upper_leaf"
    )
    upper.visual(
        upper_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="pine_wood",
        name="upper_body",
    )

    # ---------------- LOWER LEAF (raised panel, swings) ----------------
    lower = model.part("lower_leaf")
    lower_mesh = mesh_from_cadquery(
        _raised_panel_leaf_cq(LEAF_W, LOWER_H, LEAF_T), "lower_leaf"
    )
    lower.visual(
        lower_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="pine_wood",
        name="lower_body",
    )

    # ---------------- HINGE HARDWARE (visible, on each leaf) ----------------
    # Hinges live on the upper/lower leaf parts at the hinge edge (local x=0).
    # The barrel sits on the pivot line; the leaf-side plate laps onto the door
    # stile (so the hinge is connected to the leaf), and the jamb-side plate
    # reaches toward the jamb. The barrel rides on the room-side (+Y) face line.
    hinge_y = LEAF_T / 2.0 - HINGE_R + 0.002  # barrel just proud of +Y face
    hinge_x = 0.0

    upper_hinge_mesh = _hinge_barrel_mesh("upper_hinge_barrel")
    # Two hinges along the upper leaf height (local z within 0..UPPER_H).
    for i, hz in enumerate((0.16, UPPER_H - 0.16)):
        upper.visual(
            upper_hinge_mesh,
            origin=Origin(xyz=(hinge_x, hinge_y, hz - HINGE_LEN / 2.0)),
            material="dark_iron",
            name=f"upper_hinge_{i}",
        )

    lower_hinge_mesh = _hinge_barrel_mesh("lower_hinge_barrel")
    for i, hz in enumerate((0.16, LOWER_H - 0.16)):
        lower.visual(
            lower_hinge_mesh,
            origin=Origin(xyz=(hinge_x, hinge_y, hz - HINGE_LEN / 2.0)),
            material="dark_iron",
            name=f"lower_hinge_{i}",
        )

    # ---------------- LEVER HANDLE on lower leaf latch edge ----------------
    # Horizontal lever handle on a rectangular backplate, mounted on the room
    # side (+Y face) of the lower leaf near the split, at the latch edge.
    handle = model.part("lever_handle")
    lever_mesh = _lever_handle_mesh("lever_handle")
    handle.visual(
        lever_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="brushed_steel",
        name="lever_body",
    )

    # ---------------- ARTICULATIONS ----------------
    # Two independent revolute joints, both vertical (Z) on the SAME jamb, at
    # different heights. Positive q opens each leaf into the room (+Y) — the
    # closed leaf extends along +X from the hinge at x=0, so +Z (right-hand
    # rule) swings the free (latch) edge toward +Y.
    model.articulation(
        "frame_to_upper",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=upper,
        # Hinge frame at the hinge edge, lifted to the upper leaf base.
        origin=Origin(xyz=(0.0, 0.0, UPPER_BOTTOM)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=2.0, lower=0.0, upper=1.7),
    )
    model.articulation(
        "frame_to_lower",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=lower,
        origin=Origin(xyz=(0.0, 0.0, LOWER_BOTTOM)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=2.0, lower=0.0, upper=1.7),
    )
    # The lever handle is mounted on the lower leaf near the split, latch edge,
    # room side. The backplate sits flush on the +Y wood face; the lever arm
    # protrudes outward (+Y).
    handle_x = LEAF_W - 0.080
    handle_z = LOWER_H - 0.100
    handle_y = LEAF_T / 2.0
    model.articulation(
        "lower_to_handle",
        ArticulationType.FIXED,
        parent=lower,
        child=handle,
        origin=Origin(xyz=(handle_x, handle_y, handle_z)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("door_frame")
    upper = object_model.get_part("upper_leaf")
    lower = object_model.get_part("lower_leaf")
    handle = object_model.get_part("lever_handle")
    upper_hinge = object_model.get_articulation("frame_to_upper")
    lower_hinge = object_model.get_articulation("frame_to_lower")

    # --- Hero size: full Dutch door ~0.9 m wide x ~2.0 m tall, standing on floor. ---
    upper_aabb = ctx.part_world_aabb(upper)
    lower_aabb = ctx.part_world_aabb(lower)
    frame_aabb = ctx.part_world_aabb(frame)

    if frame_aabb is not None:
        (fx0, fy0, fz0), (fx1, fy1, fz1) = frame_aabb
        ctx.check("stands_on_ground", abs(fz0) <= 0.002, details=f"frame_z0={fz0:.4f}")
        ctx.check("door_height_realistic", 1.95 <= fz1 <= 2.30, details=f"top={fz1:.3f}")

    if upper_aabb is not None and lower_aabb is not None:
        (ux0, uy0, uz0), (ux1, uy1, uz1) = upper_aabb
        (lx0, ly0, lz0), (lx1, ly1, lz1) = lower_aabb
        ctx.check(
            "upper_leaf_width_realistic", 0.85 <= (ux1 - ux0) <= 0.95, details=f"w={ux1 - ux0:.3f}"
        )
        ctx.check(
            "lower_leaf_width_realistic", 0.85 <= (lx1 - lx0) <= 0.95, details=f"w={lx1 - lx0:.3f}"
        )
        # The door is split HORIZONTALLY: lower leaf sits below the upper leaf.
        ctx.check(
            "split_is_horizontal",
            uz0 >= lz1 - 0.02,
            details=f"upper_z0={uz0:.3f} lower_z1={lz1:.3f}",
        )
        ctx.check(
            "lower_starts_above_floor",
            lz0 >= 0.0 - 0.002,
            details=f"lower_z0={lz0:.3f}",
        )
        # Both leaves are the same door — share width footprint at x.
        ctx.check(
            "leaves_share_hinge_edge",
            abs(ux0 - lx0) <= 0.01,
            details=f"upper_x0={ux0:.3f} lower_x0={lx0:.3f}",
        )

    # --- Both leaves are solid raised-panel wood (no glazing anywhere). ---
    # Upper leaf must have a raised panel proud of the field.
    if upper_aabb is not None:
        (ux0, uy0, uz0), (ux1, uy1, uz1) = upper_aabb
        ctx.check(
            "upper_raised_panel_present",
            (uy1 - uy0) > LEAF_T + PANEL_PROUD * 0.5,
            details=f"thickness w/ panel={uy1 - uy0:.4f} vs leaf_t={LEAF_T:.3f}",
        )
    # Lower leaf raised panel proud of the field.
    if lower_aabb is not None:
        (lx0, ly0, lz0), (lx1, ly1, lz1) = lower_aabb
        ctx.check(
            "lower_raised_panel_present",
            (ly1 - ly0) > LEAF_T + PANEL_PROUD * 0.5,
            details=f"thickness w/ panel={ly1 - ly0:.4f} vs leaf_t={LEAF_T:.3f}",
        )
    # No transparent/glass material should exist on this solid-panel variant.
    glass_mat = next(
        (m for m in object_model.materials if m.name == "frosted_glass"), None
    )
    ctx.check(
        "no_glass_on_solid_door",
        glass_mat is None,
        details="solid two-panel split door must not carry frosted glass material",
    )
    # No window_glass visual element on upper leaf.
    upper_visual_names = [v.name for v in upper.visuals]
    ctx.check(
        "no_window_visual_on_upper",
        "window_glass" not in upper_visual_names,
        details=f"upper visuals: {upper_visual_names}",
    )

    # --- Lever handle: horizontal, protruding past the leaf face, near latch edge. ---
    h_aabb = ctx.part_world_aabb(handle)
    body_aabb = ctx.part_element_world_aabb(lower, elem="lower_body")
    if h_aabb is not None and lower_aabb is not None and body_aabb is not None:
        (hx0, hy0, hz0), (hx1, hy1, hz1) = h_aabb
        # Handle must be near the latch edge of the lower leaf.
        ctx.check(
            "handle_near_latch_edge",
            hx1 > (lower_aabb[1][0] - 0.30),
            details=f"hx1={hx1:.3f} latch={lower_aabb[1][0]:.3f}",
        )
        # The lever must protrude past the wood leaf face (room side +Y).
        ctx.check(
            "lever_protrudes_room_side",
            hy1 > body_aabb[1][1] + 0.02,
            details=f"hy1={hy1:.3f} leaf_body_y1={body_aabb[1][1]:.3f}",
        )
        # The lever handle should be predominantly horizontal: its Y extent
        # (outward reach) should be significantly larger than its X extent
        # (backplate width), proving the lever arm extends outward.
        handle_dy = hy1 - hy0
        handle_dx = hx1 - hx0
        ctx.check(
            "lever_is_horizontal",
            handle_dy > handle_dx * 1.5,
            details=f"handle_dy={handle_dy:.3f} handle_dx={handle_dx:.3f}",
        )

    # --- Closed pose: each leaf hinge edge seats against (laps) the hinge jamb. ---
    with ctx.pose({upper_hinge: 0.0, lower_hinge: 0.0}):
        ctx.expect_gap(
            upper,
            frame,
            axis="x",
            max_gap=0.002,
            max_penetration=0.010,
            positive_elem="upper_body",
            negative_elem="hinge_jamb",
            name="upper_hinge_edge_seats_against_jamb",
        )
        ctx.expect_gap(
            lower,
            frame,
            axis="x",
            max_gap=0.002,
            max_penetration=0.010,
            positive_elem="lower_body",
            negative_elem="hinge_jamb",
            name="lower_hinge_edge_seats_against_jamb",
        )
        # Closed: the leaves stack with only the reveal gap at the split.
        ctx.expect_gap(
            upper,
            lower,
            axis="z",
            min_gap=0.0,
            max_gap=0.012,
            positive_elem="upper_body",
            negative_elem="lower_body",
            name="leaves_meet_at_split",
        )

    # --- Upper leaf opens independently into the room; lower stays closed. ---
    upper_rest_aabb_closed = ctx.part_world_aabb(upper)
    lower_rest_closed = ctx.part_world_aabb(lower)
    with ctx.pose({upper_hinge: 1.5, lower_hinge: 0.0}):
        upper_open_aabb = ctx.part_world_aabb(upper)
        lower_still = ctx.part_world_aabb(lower)
        ctx.check(
            "upper_swings_into_room",
            upper_open_aabb is not None and upper_open_aabb[1][1] > LEAF_W * 0.4,
            details=f"open_y1={upper_open_aabb[1][1] if upper_open_aabb else None}",
        )
        ctx.check(
            "lower_unaffected_by_upper",
            lower_still is not None
            and lower_rest_closed is not None
            and abs(lower_still[1][1] - lower_rest_closed[1][1]) < 0.005,
            details="lower leaf must not move when only the upper opens",
        )
        ctx.expect_contact(
            upper,
            frame,
            elem_a="upper_body",
            elem_b="hinge_jamb",
            contact_tol=0.03,
            name="upper_hinge_edge_stays_connected_when_open",
        )
    # The free (latch) edge swings far in +Y when opened; the closed pose's
    # +Y extent is just the leaf thickness, so a large +Y growth proves motion.
    ctx.check(
        "upper_actually_moves",
        upper_rest_aabb_closed is not None
        and upper_open_aabb is not None
        and (upper_open_aabb[1][1] - upper_rest_aabb_closed[1][1]) > 0.30,
        details=f"closed_y1={upper_rest_aabb_closed[1][1] if upper_rest_aabb_closed else None}, "
        f"open_y1={upper_open_aabb[1][1] if upper_open_aabb else None}",
    )

    # --- Lower leaf opens independently into the room; upper stays closed. ---
    upper_rest_closed = ctx.part_world_aabb(upper)
    with ctx.pose({upper_hinge: 0.0, lower_hinge: 1.5}):
        lower_open_aabb = ctx.part_world_aabb(lower)
        upper_still = ctx.part_world_aabb(upper)
        ctx.check(
            "lower_swings_into_room",
            lower_open_aabb is not None and lower_open_aabb[1][1] > LEAF_W * 0.4,
            details=f"open_y1={lower_open_aabb[1][1] if lower_open_aabb else None}",
        )
        ctx.check(
            "upper_unaffected_by_lower",
            upper_still is not None
            and upper_rest_closed is not None
            and abs(upper_still[1][1] - upper_rest_closed[1][1]) < 0.005,
            details="upper leaf must not move when only the lower opens",
        )
        ctx.expect_contact(
            lower,
            frame,
            elem_a="lower_body",
            elem_b="hinge_jamb",
            contact_tol=0.03,
            name="lower_hinge_edge_stays_connected_when_open",
        )

    # --- Intentional overlaps (local, hardware/seating only). ---
    ctx.allow_overlap(
        upper,
        frame,
        elem_a="upper_body",
        elem_b="hinge_jamb",
        reason="Hinge jamb intentionally laps the upper leaf hinge edge so the leaf is carried by the frame at the hinge line.",
    )
    ctx.allow_overlap(
        lower,
        frame,
        elem_a="lower_body",
        elem_b="hinge_jamb",
        reason="Hinge jamb intentionally laps the lower leaf hinge edge so the leaf is carried by the frame at the hinge line.",
    )
    ctx.allow_overlap(
        handle,
        lower,
        reason="Lever handle backplate is seated against and through the lower leaf face like real door hardware.",
    )
    # Visible barrel hinges intentionally bridge the leaf and the hinge jamb:
    # the jamb-side plate/barrel laps onto the jamb, like a real mortise hinge.
    ctx.allow_overlap(
        upper,
        frame,
        elem_a="upper_hinge_0",
        elem_b="hinge_jamb",
        reason="Upper barrel hinge laps onto the hinge jamb; the jamb-side leaf plate screws to the jamb.",
    )
    ctx.allow_overlap(
        upper,
        frame,
        elem_a="upper_hinge_1",
        elem_b="hinge_jamb",
        reason="Upper barrel hinge laps onto the hinge jamb; the jamb-side leaf plate screws to the jamb.",
    )
    ctx.allow_overlap(
        lower,
        frame,
        elem_a="lower_hinge_0",
        elem_b="hinge_jamb",
        reason="Lower barrel hinge laps onto the hinge jamb; the jamb-side leaf plate screws to the jamb.",
    )
    ctx.allow_overlap(
        lower,
        frame,
        elem_a="lower_hinge_1",
        elem_b="hinge_jamb",
        reason="Lower barrel hinge laps onto the hinge jamb; the jamb-side leaf plate screws to the jamb.",
    )

    return ctx.report()


object_model = build_object_model()
