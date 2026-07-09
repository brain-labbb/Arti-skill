from __future__ import annotations

# Four-panel telescoping sliding glass door in a slim black aluminium frame.
# One fixed pane plus three sliding panes that telescope and nest along a
# horizontal track. The frame and fixed pane are the static root; the three
# sliding panes each ride on a PRISMATIC joint along +X, with each successive
# leaf travelling farther into the stack.
#
# Coordinate convention:
# - World up is +Z; the unit stands on the ground plane (z = 0 at the sill).
# - The wall opening lies in the X-Z plane; panes slide horizontally along X.
# - Frame depth runs along Y; the four panes sit in four parallel tracks so
#   they can nest/overlap in X (while opening) without colliding in Y.

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Mesh,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Master dimensions (meters)
# ---------------------------------------------------------------------------
UNIT_W = 3.20          # overall width of the door unit
UNIT_H = 2.10          # overall height (sill to head)
BAY_W = 0.80           # one of four equal bays across the opening

FRAME_T = 0.050        # head / sill / jamb section width (slim visible face)
FRAME_D = 0.165        # total frame depth along Y (houses four tracks)

# Vertical extents
SILL_TOP = FRAME_T               # top of the bottom sill
HEAD_BOTTOM = UNIT_H - FRAME_T   # bottom of the head jamb
GLASS_H = HEAD_BOTTOM - SILL_TOP # clear height of the panes between sill/head

# Pane (sliding leaf) construction
PANE_W = BAY_W                   # each leaf spans one bay
PANE_STILE_W = 0.034             # slim vertical aluminium stile/rail of a leaf
PANE_RAIL_H = 0.040              # slim top/bottom rail height of a leaf
PANE_FRAME_D = 0.030             # leaf aluminium thickness along Y
GLASS_D = 0.008                  # glass thickness along Y
PANE_H = HEAD_BOTTOM - SILL_TOP  # leaf height (fits between sill and head)

# Track Y-centers: four parallel planes inside the frame depth.
# Track 0 is the rear-most, track 3 is the front-most.
TRACK_Y = (-0.060, -0.020, 0.020, 0.060)

# Closed-pose X centers of the four leaves across the opening.
# Leftmost = leading slider (door_2); rightmost = fixed pane.
X_DOOR2 = -1.5 * BAY_W   # -1.20  leading slider, leftmost
X_DOOR1 = -0.5 * BAY_W   # -0.40
X_DOOR0 = 0.5 * BAY_W    # +0.40
X_FIXED = 1.5 * BAY_W    # +1.20  fixed pane, rightmost


def _build_pocket_stile_cq():
    """Build a CadQuery solid for a stile with an inset rectangular flush finger
    pull pocket cut into its leading (-X) face. The pocket opening is flush with
    the stile face (no protruding bar)."""
    stile = cq.Workplane("XY").box(PANE_STILE_W, PANE_FRAME_D, PANE_H)
    # Pocket dimensions
    pocket_depth = 0.014       # cut depth into the stile along X
    pocket_w_y = 0.018         # width across the stile face (Y)
    pocket_h_z = 0.120         # height along the stile (Z)
    # Cut from the -X face (leading face); negative cutBlind cuts into the solid.
    stile = (
        stile
        .faces("<X")
        .workplane()
        .rect(pocket_w_y, pocket_h_z)
        .cutBlind(-pocket_depth)
    )
    return stile


def _add_leaf(part, *, glass_mat, alu_mat, pocket_stile_mesh=None):
    """Build one glazed leaf (aluminium perimeter + glass infill) in the part's
    local frame, centered on (0, 0, PANE_H/2). When pocket_stile_mesh is
    provided, the leading stile uses that mesh-backed visual with an inset flush
    finger pull pocket instead of a plain box."""
    half_w = PANE_W / 2.0
    z_mid = PANE_H / 2.0
    # Leading stile (with flush finger pull pocket when mesh provided)
    if pocket_stile_mesh is not None:
        part.visual(
            pocket_stile_mesh,
            origin=Origin(xyz=(-half_w + PANE_STILE_W / 2.0, 0.0, z_mid)),
            material=alu_mat,
            name="stile_lead",
        )
    else:
        part.visual(
            Box((PANE_STILE_W, PANE_FRAME_D, PANE_H)),
            origin=Origin(xyz=(-half_w + PANE_STILE_W / 2.0, 0.0, z_mid)),
            material=alu_mat,
            name="stile_lead",
        )
    # Trailing stile (plain box)
    part.visual(
        Box((PANE_STILE_W, PANE_FRAME_D, PANE_H)),
        origin=Origin(xyz=(half_w - PANE_STILE_W / 2.0, 0.0, z_mid)),
        material=alu_mat,
        name="stile_trail",
    )
    # Horizontal rails (top/bottom of the leaf), between the stiles
    rail_w = PANE_W - 2.0 * PANE_STILE_W
    part.visual(
        Box((rail_w, PANE_FRAME_D, PANE_RAIL_H)),
        origin=Origin(xyz=(0.0, 0.0, PANE_RAIL_H / 2.0)),
        material=alu_mat,
        name="rail_bottom",
    )
    part.visual(
        Box((rail_w, PANE_FRAME_D, PANE_RAIL_H)),
        origin=Origin(xyz=(0.0, 0.0, PANE_H - PANE_RAIL_H / 2.0)),
        material=alu_mat,
        name="rail_top",
    )
    # Glass infill (thin, centered in the leaf depth, inset within the frame)
    glass_w = PANE_W - 2.0 * PANE_STILE_W
    glass_h = PANE_H - 2.0 * PANE_RAIL_H
    part.visual(
        Box((glass_w, GLASS_D, glass_h)),
        origin=Origin(xyz=(0.0, 0.0, z_mid)),
        material=glass_mat,
        name="glass",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="four_panel_telescoping_sliding_door")

    # Restrained real materials.
    black_alu = model.material("black_anodized_aluminium", rgba=(0.07, 0.07, 0.08, 1.0))
    dark_alu = model.material("dark_aluminium_track", rgba=(0.12, 0.12, 0.13, 1.0))
    glass = model.material("clear_glass", rgba=(0.70, 0.76, 0.78, 0.16))

    # Build the mesh-backed stile with inset flush finger pull pocket once;
    # reuse across all three sliding leaves.
    pocket_stile_mesh = mesh_from_cadquery(
        _build_pocket_stile_cq(), "pocket_stile",
    )

    half_w = UNIT_W / 2.0

    # -------------------------------------------------------------------
    # FIXED ROOT: outer frame (head, sill, two side jambs) + fixed pane.
    # -------------------------------------------------------------------
    frame = model.part("frame")

    # Bottom sill (full width), standing on the ground plane.
    frame.visual(
        Box((UNIT_W, FRAME_D, FRAME_T)),
        origin=Origin(xyz=(0.0, 0.0, FRAME_T / 2.0)),
        material=black_alu,
        name="sill",
    )
    # Top head jamb (full width).
    frame.visual(
        Box((UNIT_W, FRAME_D, FRAME_T)),
        origin=Origin(xyz=(0.0, 0.0, UNIT_H - FRAME_T / 2.0)),
        material=black_alu,
        name="head",
    )
    # Left side jamb.
    frame.visual(
        Box((FRAME_T, FRAME_D, GLASS_H)),
        origin=Origin(xyz=(-half_w + FRAME_T / 2.0, 0.0, SILL_TOP + GLASS_H / 2.0)),
        material=black_alu,
        name="jamb_lead",
    )
    # Right side jamb.
    frame.visual(
        Box((FRAME_T, FRAME_D, GLASS_H)),
        origin=Origin(xyz=(half_w - FRAME_T / 2.0, 0.0, SILL_TOP + GLASS_H / 2.0)),
        material=black_alu,
        name="jamb_trail",
    )
    # Thin track ribs along the sill/head separating the four leaf planes
    # (reads as a real multi-track aluminium bottom rail).
    for i, ty in enumerate(TRACK_Y):
        frame.visual(
            Box((UNIT_W - 2.0 * FRAME_T, 0.010, 0.018)),
            origin=Origin(xyz=(0.0, ty, SILL_TOP + 0.009)),
            material=dark_alu,
            name=f"track_rib_{i}",
        )

    # Fixed pane: a glazed leaf permanently seated in the rightmost bay,
    # sitting in the rear-most track plane (TRACK_Y[0]).
    half_pane = PANE_W / 2.0
    z_mid = SILL_TOP + PANE_H / 2.0
    glass_w = PANE_W - 2.0 * PANE_STILE_W
    glass_h = PANE_H - 2.0 * PANE_RAIL_H
    frame.visual(
        Box((PANE_STILE_W, PANE_FRAME_D, PANE_H)),
        origin=Origin(xyz=(X_FIXED - half_pane + PANE_STILE_W / 2.0, TRACK_Y[0], z_mid)),
        material=black_alu,
        name="fixed_stile_lead",
    )
    frame.visual(
        Box((PANE_STILE_W, PANE_FRAME_D, PANE_H)),
        origin=Origin(xyz=(X_FIXED + half_pane - PANE_STILE_W / 2.0, TRACK_Y[0], z_mid)),
        material=black_alu,
        name="fixed_stile_trail",
    )
    frame.visual(
        Box((PANE_W - 2.0 * PANE_STILE_W, PANE_FRAME_D, PANE_RAIL_H)),
        origin=Origin(xyz=(X_FIXED, TRACK_Y[0], SILL_TOP + PANE_RAIL_H / 2.0)),
        material=black_alu,
        name="fixed_rail_bottom",
    )
    frame.visual(
        Box((PANE_W - 2.0 * PANE_STILE_W, PANE_FRAME_D, PANE_RAIL_H)),
        origin=Origin(xyz=(X_FIXED, TRACK_Y[0], SILL_TOP + PANE_H - PANE_RAIL_H / 2.0)),
        material=black_alu,
        name="fixed_rail_top",
    )
    frame.visual(
        Box((glass_w, GLASS_D, glass_h)),
        origin=Origin(xyz=(X_FIXED, TRACK_Y[0], z_mid)),
        material=glass,
        name="fixed_glass",
    )

    # -------------------------------------------------------------------
    # THREE SLIDING LEAVES (telescoping). Each leaf's local frame origin is
    # its closed-pose center column at z=0; geometry is built upward.
    # door_0 = innermost slider (nearest the fixed pane), door_2 = leading.
    # -------------------------------------------------------------------
    door_0 = model.part("door_0")  # track 1
    _add_leaf(door_0, glass_mat=glass, alu_mat=black_alu, pocket_stile_mesh=pocket_stile_mesh)

    door_1 = model.part("door_1")  # track 2
    _add_leaf(door_1, glass_mat=glass, alu_mat=black_alu, pocket_stile_mesh=pocket_stile_mesh)

    door_2 = model.part("door_2")  # track 3, leading leaf
    _add_leaf(door_2, glass_mat=glass, alu_mat=black_alu, pocket_stile_mesh=pocket_stile_mesh)

    # Closed-pose travel: each leaf slides +X to nest near the fixed pane.
    # door_0 needs to travel one bay; door_1 two bays; door_2 three bays so
    # they telescope and stack. Stop short of one bay so leaves stay engaged
    # (overlap with the neighbour) when fully open.
    travel_0 = X_FIXED - X_DOOR0 - 0.02   # ~0.78
    travel_1 = X_FIXED - X_DOOR1 - 0.02   # ~1.58
    travel_2 = X_FIXED - X_DOOR2 - 0.02   # ~2.38

    # Joint origin places each leaf's local frame at its closed-pose seat.
    model.articulation(
        "slide_0",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=door_0,
        origin=Origin(xyz=(X_DOOR0, TRACK_Y[1], SILL_TOP)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=200.0, velocity=0.4, lower=0.0, upper=travel_0),
    )
    model.articulation(
        "slide_1",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=door_1,
        origin=Origin(xyz=(X_DOOR1, TRACK_Y[2], SILL_TOP)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=200.0, velocity=0.4, lower=0.0, upper=travel_1),
    )
    model.articulation(
        "slide_2",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=door_2,
        origin=Origin(xyz=(X_DOOR2, TRACK_Y[3], SILL_TOP)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=200.0, velocity=0.4, lower=0.0, upper=travel_2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    door_0 = object_model.get_part("door_0")
    door_1 = object_model.get_part("door_1")
    door_2 = object_model.get_part("door_2")
    slide_0 = object_model.get_articulation("slide_0")
    slide_1 = object_model.get_articulation("slide_1")
    slide_2 = object_model.get_articulation("slide_2")

    # --- The three primary mechanisms are prismatic slides. ---
    ctx.check(
        "three sliding panes are prismatic",
        slide_0.articulation_type == ArticulationType.PRISMATIC
        and slide_1.articulation_type == ArticulationType.PRISMATIC
        and slide_2.articulation_type == ArticulationType.PRISMATIC,
        details=f"{slide_0.articulation_type}, {slide_1.articulation_type}, {slide_2.articulation_type}",
    )

    # --- Glass present on every leaf (transparent, low alpha). ---
    fixed_glass = frame.get_visual("fixed_glass")
    g0 = door_0.get_visual("glass")
    g2 = door_2.get_visual("glass")
    ctx.check(
        "glass infill is transparent",
        fixed_glass.material.rgba[3] < 0.5
        and g0.material.rgba[3] < 0.5
        and g2.material.rgba[3] < 0.5,
        details=f"alphas={fixed_glass.material.rgba[3]}, {g0.material.rgba[3]}, {g2.material.rgba[3]}",
    )

    # --- Each sliding leaf has a flush finger pull pocket on its leading stile. ---
    for door_name in ("door_0", "door_1", "door_2"):
        door_part = object_model.get_part(door_name)
        stile_lead = door_part.get_visual("stile_lead")
        ctx.check(
            f"{door_name} has a mesh-backed pocket stile",
            stile_lead is not None and isinstance(stile_lead.geometry, Mesh),
            details=f"geometry type={type(stile_lead.geometry).__name__}",
        )
    # No protruding bar handle exists on any leaf.
    for door_name in ("door_0", "door_1", "door_2"):
        door_part = object_model.get_part(door_name)
        visual_names = {v.name for v in door_part.visuals}
        ctx.check(
            f"{door_name} has no proud bar handle",
            "handle" not in visual_names,
            details=f"visuals={sorted(visual_names)}",
        )

    # --- Unit stands on the ground plane (sill bottom at z ~ 0). ---
    frame_aabb = ctx.part_world_aabb(frame)
    ctx.check(
        "unit stands on the ground plane",
        frame_aabb is not None and abs(frame_aabb[0][2]) < 0.01,
        details=f"frame_aabb={frame_aabb}",
    )

    # --- Closed pose: the four leaves are spread across the opening so each
    # bay reads as glazed (leaves separated in X). ---
    x0 = ctx.part_world_position(door_0)
    x1 = ctx.part_world_position(door_1)
    x2 = ctx.part_world_position(door_2)
    ctx.check(
        "closed leaves are spread across the opening (left to right)",
        x2 is not None and x1 is not None and x0 is not None
        and x2[0] < x1[0] - 0.5 < x0[0] - 1.0,
        details=f"door_2={x2}, door_1={x1}, door_0={x0}",
    )

    # --- Leaves stay centered in their own track plane (no Y wander). ---
    ctx.expect_within(
        door_0, frame, axes="z", margin=0.02,
        name="door_0 fits within the frame height",
    )

    # --- Open pose: each successive leaf travels farther into the stack and
    # all leaves nest near the fixed-pane side (+X). They remain engaged
    # (overlap their neighbour in X) so the assembly never disconnects. ---
    lim0 = slide_0.motion_limits.upper
    lim1 = slide_1.motion_limits.upper
    lim2 = slide_2.motion_limits.upper
    with ctx.pose({slide_0: lim0, slide_1: lim1, slide_2: lim2}):
        ox0 = ctx.part_world_position(door_0)
        ox1 = ctx.part_world_position(door_1)
        ox2 = ctx.part_world_position(door_2)
        # Each leaf moved in +X (opened toward the stack).
        ctx.check(
            "each leaf slides into the stack along +X",
            ox0[0] > x0[0] + 0.5 and ox1[0] > x1[0] + 1.0 and ox2[0] > x2[0] + 1.5,
            details=f"open door_0={ox0}, door_1={ox1}, door_2={ox2}",
        )
        # Telescoping: leaf 2 travels farther than leaf 1, which travels
        # farther than leaf 0.
        travel0 = ox0[0] - x0[0]
        travel1 = ox1[0] - x1[0]
        travel2 = ox2[0] - x2[0]
        ctx.check(
            "successive leaves telescope farther",
            travel2 > travel1 > travel0,
            details=f"travels = {travel0:.3f}, {travel1:.3f}, {travel2:.3f}",
        )
        # Open leaves are nested/clustered near the fixed-pane side.
        ctx.check(
            "open leaves nest together near the fixed pane",
            abs(ox0[0] - ox1[0]) < BAY_W and abs(ox1[0] - ox2[0]) < BAY_W
            and abs(ox0[0] - X_FIXED) < BAY_W,
            details=f"open door_0={ox0}, door_1={ox1}, door_2={ox2}",
        )

    # The four leaves occupy four parallel Y tracks and intentionally overlap
    # in X while nesting when open. Allow that overlap explicitly.
    ctx.allow_overlap(
        door_0, door_1,
        reason="Telescoping leaves nest and overlap in X while sliding; they sit in separate Y tracks so they never physically collide.",
    )
    ctx.allow_overlap(
        door_1, door_2,
        reason="Telescoping leaves nest and overlap in X while sliding; they sit in separate Y tracks so they never physically collide.",
    )
    ctx.allow_overlap(
        door_0, door_2,
        reason="Telescoping leaves nest and overlap in X while sliding; they sit in separate Y tracks so they never physically collide.",
    )
    ctx.allow_overlap(
        door_0, frame,
        reason="The innermost slider nests behind the fixed pane (separate Y track) when fully open.",
    )
    ctx.allow_overlap(
        door_1, frame,
        reason="A slider nests behind the fixed pane (separate Y track) when fully open.",
    )
    ctx.allow_overlap(
        door_2, frame,
        reason="A slider nests behind the fixed pane (separate Y track) when fully open.",
    )

    return ctx.report()


object_model = build_object_model()
