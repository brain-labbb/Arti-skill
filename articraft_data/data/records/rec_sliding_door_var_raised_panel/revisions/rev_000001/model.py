from __future__ import annotations

# Two-panel sliding wood door in a slim black aluminium frame.
# Root = outer frame + fixed Shaker wood panel (static). One sliding Shaker
# wood panel leaf on a single PRISMATIC joint runs along the horizontal track
# and tucks behind the fixed panel. Each leaf reads as a closet/pantry door:
# a recessed flat center field bordered by a slightly proud stile-and-rail
# frame. A small recessed lever handle rides on the moving leaf.
#
# Conventions:
# - World up is +Z. Width along X, height along Z, depth along Y.
# - All geometry from z=0 up.
# - The sliding leaf sits toward +Y (room side) so it passes in front of the
#   fixed panel; positive q slides it along -X to overlap the fixed bay.

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

# ---------------------------------------------------------------------------
# Real-world dimensions (meters)
# ---------------------------------------------------------------------------
UNIT_W = 2.80
UNIT_H = 2.00
FRAME_FACE = 0.042
FRAME_DEPTH = 0.058

LEAF_FRAME_FACE = 0.035
LEAF_FRAME_DEPTH = 0.040

INNER_W = UNIT_W - 2 * FRAME_FACE
INNER_H = UNIT_H - 2 * FRAME_FACE
BAY_W = INNER_W / 2.0

# Y placement (depth)
Y_FIXED = -0.010
Y_LEAF = FRAME_DEPTH / 2.0 + LEAF_FRAME_DEPTH / 2.0 + 0.004

# Shaker panel infill constants
PANEL_BORDER_W = 0.065      # stile/rail face width for the fixed panel border
PANEL_BORDER_D = 0.025      # border depth (proud of center field)
PANEL_CENTER_D = 0.016      # fixed panel center field thickness
PANEL_RECESS = 0.008        # how far the center face is set back from the border face
LEAF_CENTER_D = LEAF_FRAME_DEPTH - PANEL_RECESS  # leaf center fills frame back to recess line

# Pre-computed Y centers for recessed center fields
FIXED_CENTER_CY = Y_FIXED + PANEL_BORDER_D / 2.0 - PANEL_RECESS - PANEL_CENTER_D / 2.0
LEAF_CENTER_CY = LEAF_FRAME_DEPTH / 2.0 - PANEL_RECESS - LEAF_CENTER_D / 2.0

# Colors / materials
BLACK_ALU = (0.09, 0.09, 0.10)
WOOD_BORDER = (0.40, 0.26, 0.14)      # warm stained wood (stiles & rails)
WOOD_CENTER = (0.52, 0.36, 0.22)      # lighter natural wood (flat center field)
HANDLE_METAL = (0.16, 0.16, 0.17)


def add_shaker_panel(part, *, total_w, total_h, border_w, border_d,
                     center_d, cx, cz, border_cy, center_cy,
                     border_mat, center_mat, prefix):
    """Add Shaker-style panel visuals: proud stile-and-rail border + recessed flat center."""
    inner_w = total_w - 2 * border_w
    inner_h = total_h - 2 * border_w

    # Stiles (vertical members, full panel height)
    for i in range(2):
        sx = cx + (2 * i - 1) * (total_w / 2.0 - border_w / 2.0)
        part.visual(
            Box((border_w, border_d, total_h)),
            origin=Origin(xyz=(sx, border_cy, cz)),
            name=f"{prefix}_stile_{i}",
            material=border_mat,
        )

    # Rails (horizontal members, fitting between stiles)
    for i in range(2):
        rz = cz + (2 * i - 1) * (total_h / 2.0 - border_w / 2.0)
        part.visual(
            Box((inner_w, border_d, border_w)),
            origin=Origin(xyz=(cx, border_cy, rz)),
            name=f"{prefix}_rail_{i}",
            material=border_mat,
        )

    # Recessed flat center field
    part.visual(
        Box((inner_w, center_d, inner_h)),
        origin=Origin(xyz=(cx, center_cy, cz)),
        name=f"{prefix}_center",
        material=center_mat,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_panel_sliding_wood_door")

    model.material("black_alu", rgba=(*BLACK_ALU, 1.0))
    model.material("wood_border", rgba=(*WOOD_BORDER, 1.0))
    model.material("wood_center", rgba=(*WOOD_CENTER, 1.0))
    model.material("handle_metal", rgba=(*HANDLE_METAL, 1.0))

    # ------------------------------------------------------------------
    # ROOT: outer frame + fixed Shaker wood panel (static)
    # ------------------------------------------------------------------
    frame = model.part("door_frame")

    half_w = UNIT_W / 2.0

    # Left jamb (vertical stile)
    frame.visual(
        Box((FRAME_FACE, FRAME_DEPTH, UNIT_H)),
        origin=Origin(xyz=(-half_w + FRAME_FACE / 2.0, 0.0, UNIT_H / 2.0)),
        name="jamb_left",
        material="black_alu",
    )
    # Right jamb (vertical stile)
    frame.visual(
        Box((FRAME_FACE, FRAME_DEPTH, UNIT_H)),
        origin=Origin(xyz=(half_w - FRAME_FACE / 2.0, 0.0, UNIT_H / 2.0)),
        name="jamb_right",
        material="black_alu",
    )
    # Head (top rail)
    frame.visual(
        Box((INNER_W, FRAME_DEPTH, FRAME_FACE)),
        origin=Origin(xyz=(0.0, 0.0, UNIT_H - FRAME_FACE / 2.0)),
        name="head_rail",
        material="black_alu",
    )
    # Sill (bottom rail) sitting on the floor
    frame.visual(
        Box((INNER_W, FRAME_DEPTH, FRAME_FACE)),
        origin=Origin(xyz=(0.0, 0.0, FRAME_FACE / 2.0)),
        name="sill_rail",
        material="black_alu",
    )
    # Center meeting stile between the two bays
    frame.visual(
        Box((FRAME_FACE, FRAME_DEPTH, INNER_H)),
        origin=Origin(xyz=(0.0, 0.0, UNIT_H / 2.0)),
        name="meeting_stile",
        material="black_alu",
    )

    # Head and sill tracks the moving leaf runs in. Each track is a solid lip
    # growing OUT of the frame toward +Y (room side), reaching the leaf depth
    # plane so it is one connected piece with the frame.
    frame_front = FRAME_DEPTH / 2.0
    lip_back = frame_front - 0.006
    lip_front = Y_LEAF + 0.004
    lip_dy = lip_front - lip_back
    lip_cy = (lip_front + lip_back) / 2.0
    HEAD_TRACK_H = 0.022
    SILL_TRACK_H = 0.020
    head_track_cz = UNIT_H - FRAME_FACE - HEAD_TRACK_H / 2.0
    sill_track_cz = FRAME_FACE + SILL_TRACK_H / 2.0
    frame.visual(
        Box((INNER_W, lip_dy, HEAD_TRACK_H)),
        origin=Origin(xyz=(0.0, lip_cy, head_track_cz)),
        name="head_track",
        material="black_alu",
    )
    frame.visual(
        Box((INNER_W, lip_dy, SILL_TRACK_H)),
        origin=Origin(xyz=(0.0, lip_cy, sill_track_cz)),
        name="sill_track",
        material="black_alu",
    )

    # Fixed Shaker wood panel filling the LEFT bay (rear plane).
    fixed_panel_w = BAY_W - FRAME_FACE / 2.0
    fixed_panel_h = INNER_H
    fixed_panel_cx = -BAY_W / 2.0 - FRAME_FACE / 4.0

    add_shaker_panel(
        frame,
        total_w=fixed_panel_w,
        total_h=fixed_panel_h,
        border_w=PANEL_BORDER_W,
        border_d=PANEL_BORDER_D,
        center_d=PANEL_CENTER_D,
        cx=fixed_panel_cx,
        cz=UNIT_H / 2.0,
        border_cy=Y_FIXED,
        center_cy=FIXED_CENTER_CY,
        border_mat="wood_border",
        center_mat="wood_center",
        prefix="fixed",
    )

    # ------------------------------------------------------------------
    # MOVING LEAF: Shaker-style wood panel (rides the track)
    # ------------------------------------------------------------------
    leaf = model.part("sliding_leaf")

    leaf_w = BAY_W
    leaf_h = INNER_H - 0.032
    fa = LEAF_FRAME_FACE
    fd = LEAF_FRAME_DEPTH

    # Leaf stiles (between top and bottom rails, matching parent construction
    # so only the rails engage the head/sill tracks).
    stile_h = leaf_h - 2 * fa
    for i in range(2):
        sx = (2 * i - 1) * (leaf_w / 2.0 - fa / 2.0)
        leaf.visual(
            Box((fa, fd, stile_h)),
            origin=Origin(xyz=(sx, 0.0, 0.0)),
            name=f"leaf_stile_{i}",
            material="wood_border",
        )

    # Leaf rails (full width, top and bottom)
    for i in range(2):
        rz = (2 * i - 1) * (leaf_h / 2.0 - fa / 2.0)
        leaf.visual(
            Box((leaf_w - 2 * fa, fd, fa)),
            origin=Origin(xyz=(0.0, 0.0, rz)),
            name=f"leaf_rail_{i}",
            material="wood_border",
        )

    # Recessed flat center field (opaque wood panel infill)
    leaf_center_w = leaf_w - 2 * fa
    leaf_center_h = leaf_h - 2 * fa
    leaf.visual(
        Box((leaf_center_w, LEAF_CENTER_D, leaf_center_h)),
        origin=Origin(xyz=(0.0, LEAF_CENTER_CY, 0.0)),
        name="leaf_center",
        material="wood_center",
    )

    # Slim flush pull handle on the leading (left) stile, room-side face (+Y).
    handle_z = 1.0 - UNIT_H / 2.0
    handle_x = -leaf_w / 2.0 + fa / 2.0
    HANDLE_LEN = 0.28
    leaf.visual(
        Box((0.012, 0.016, HANDLE_LEN)),
        origin=Origin(xyz=(handle_x, fd / 2.0 + 0.008, handle_z)),
        name="handle_lever",
        material="handle_metal",
    )

    # ------------------------------------------------------------------
    # ARTICULATION: single prismatic joint along the horizontal track (X).
    # ------------------------------------------------------------------
    leaf_closed_cx = BAY_W / 2.0 + FRAME_FACE / 4.0
    travel = BAY_W - FRAME_FACE
    model.articulation(
        "leaf_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=leaf,
        origin=Origin(xyz=(leaf_closed_cx, Y_LEAF, UNIT_H / 2.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0, velocity=0.8, lower=0.0, upper=travel
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("door_frame")
    leaf = object_model.get_part("sliding_leaf")
    slide = object_model.get_articulation("leaf_slide")

    # --- Intentional engagement overlap (leaf top rail in head channel) ---
    ctx.allow_overlap(
        frame,
        leaf,
        elem_a="head_track",
        elem_b="leaf_rail_1",
        reason=(
            "The leaf top rail intentionally tucks up into the head track "
            "channel; this is the captured/retained running engagement at the "
            "head, not an unintended collision."
        ),
    )
    ctx.expect_overlap(
        leaf,
        frame,
        axes="z",
        elem_a="leaf_rail_1",
        elem_b="head_track",
        min_overlap=0.004,
        name="leaf top rail stays engaged in the head channel",
    )

    # --- Wood panel hero geometry is present ---
    fixed_center = frame.get_visual("fixed_center")
    leaf_center = leaf.get_visual("leaf_center")
    handle = leaf.get_visual("handle_lever")
    ctx.check(
        "fixed wood panel center field exists",
        fixed_center is not None,
        details="fixed_center visual missing",
    )
    ctx.check(
        "sliding leaf wood center field exists",
        leaf_center is not None,
        details="leaf_center visual missing",
    )
    ctx.check(
        "recessed lever handle exists",
        handle is not None,
        details="handle_lever visual missing",
    )

    # --- Shaker stile-and-rail border members exist on both panels ---
    ctx.check(
        "fixed panel has stile-and-rail border",
        all(frame.get_visual(f"fixed_{m}_{i}") is not None
            for m in ("stile", "rail") for i in range(2)),
        details="fixed panel border members missing",
    )
    ctx.check(
        "leaf has stile-and-rail frame",
        all(leaf.get_visual(f"leaf_{m}_{i}") is not None
            for m in ("stile", "rail") for i in range(2)),
        details="leaf frame members missing",
    )

    # --- Wood materials are opaque (not transparent like the parent glass) ---
    wood_border_mat = next(
        (m for m in object_model.materials if m.name == "wood_border"), None
    )
    wood_center_mat = next(
        (m for m in object_model.materials if m.name == "wood_center"), None
    )
    ctx.check(
        "wood border material is opaque",
        wood_border_mat is not None
        and wood_border_mat.rgba is not None
        and wood_border_mat.rgba[3] >= 0.9,
        details=f"wood_border alpha={wood_border_mat.rgba[3] if wood_border_mat and wood_border_mat.rgba else None}",
    )
    ctx.check(
        "wood center material is opaque",
        wood_center_mat is not None
        and wood_center_mat.rgba is not None
        and wood_center_mat.rgba[3] >= 0.9,
        details=f"wood_center alpha={wood_center_mat.rgba[3] if wood_center_mat and wood_center_mat.rgba else None}",
    )

    # --- Shaker construction: border stands proud of the recessed center ---
    fixed_border_front = Y_FIXED + PANEL_BORDER_D / 2.0
    fixed_center_front = FIXED_CENTER_CY + PANEL_CENTER_D / 2.0
    ctx.check(
        "fixed panel border stands proud of recessed center field",
        fixed_border_front - fixed_center_front >= PANEL_RECESS - 1e-6,
        details=(
            f"border_front_y={fixed_border_front:.4f}, "
            f"center_front_y={fixed_center_front:.4f}"
        ),
    )
    leaf_frame_front = LEAF_FRAME_DEPTH / 2.0
    leaf_center_front = LEAF_CENTER_CY + LEAF_CENTER_D / 2.0
    ctx.check(
        "leaf frame stands proud of recessed center field",
        leaf_frame_front - leaf_center_front >= PANEL_RECESS - 1e-6,
        details=(
            f"frame_front_y={leaf_frame_front:.4f}, "
            f"center_front_y={leaf_center_front:.4f}"
        ),
    )

    # --- Door stands on the ground plane ---
    frame_aabb = ctx.part_world_aabb(frame)
    leaf_aabb = ctx.part_world_aabb(leaf)
    ctx.check(
        "frame sits on the floor (z>=0)",
        frame_aabb is not None and frame_aabb[0][2] >= -1e-4,
        details=f"frame z-min={None if frame_aabb is None else frame_aabb[0][2]}",
    )
    ctx.check(
        "frame is about 2.0 m tall",
        frame_aabb is not None
        and abs((frame_aabb[1][2] - frame_aabb[0][2]) - UNIT_H) < 0.02,
        details=f"frame height={None if frame_aabb is None else frame_aabb[1][2] - frame_aabb[0][2]}",
    )
    ctx.check(
        "leaf does not sink below the floor",
        leaf_aabb is not None and leaf_aabb[0][2] >= -1e-4,
        details=f"leaf z-min={None if leaf_aabb is None else leaf_aabb[0][2]}",
    )

    # --- Closed pose: leaf seats in the right bay ---
    ctx.expect_origin_gap(
        leaf, frame,
        axis="x",
        min_gap=0.2,
        name="closed leaf sits in the right bay (right of fixed panel center)",
    )
    # Leaf center field clears the fixed center field in depth (Y)
    ctx.expect_gap(
        leaf, frame,
        axis="y",
        positive_elem="leaf_center",
        negative_elem="fixed_center",
        min_gap=0.0,
        name="leaf center panel clears fixed center panel in depth",
    )

    # --- Articulation slides the leaf open (correct direction) ---
    closed_pos = ctx.part_world_position(leaf)
    upper = slide.motion_limits.upper
    with ctx.pose({slide: upper}):
        open_pos = ctx.part_world_position(leaf)
        ctx.expect_overlap(
            leaf, frame,
            axes="x",
            elem_a="leaf_center",
            elem_b="fixed_center",
            min_overlap=0.30,
            name="open leaf tucks across the fixed panel bay",
        )
    ctx.check(
        "positive q slides the leaf toward -X (open)",
        closed_pos is not None
        and open_pos is not None
        and open_pos[0] < closed_pos[0] - 0.30,
        details=f"closed_x={None if closed_pos is None else closed_pos[0]}, "
        f"open_x={None if open_pos is None else open_pos[0]}",
    )

    return ctx.report()


object_model = build_object_model()
