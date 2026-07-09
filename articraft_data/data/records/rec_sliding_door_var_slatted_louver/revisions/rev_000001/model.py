from __future__ import annotations

# Two-panel sliding louvered door in a slim black aluminium frame.
# Root = outer frame + fixed louver panel (static). One sliding louver leaf on
# a single PRISMATIC joint runs along the horizontal track and tucks behind the
# fixed panel. A small recessed lever handle rides on the moving leaf.
#
# This is a louver-variant fork of the two-panel sliding glass door: the frame,
# tracks, handle and prismatic joint are identical; only the panel infill is
# changed from glass sheets to horizontal louver slats emitted via a shared
# helper with a for-i-in-range loop and regular vertical spacing.
#
# Conventions:
# - World up is +Z. Width runs along X, height along Z, thin depth along Y.
# - The whole door stands on the ground: all geometry is authored from z=0 up.
# - The sliding leaf sits slightly toward +Y (room side) so it can pass in
#   front of the fixed panel without colliding; positive q slides it along -X
#   to overlap (tuck behind, in depth) the fixed panel.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

# ----------------------------------------------------------------------------
# Real-world dimensions (meters)
# ----------------------------------------------------------------------------
UNIT_W = 2.80          # total opening width (outer frame outer width)
UNIT_H = 2.00          # total height
FRAME_FACE = 0.042     # face width of the slim aluminium frame members
FRAME_DEPTH = 0.058    # frame depth in Y (jamb depth)

LEAF_FRAME_FACE = 0.035  # face width of the moving leaf's own slim frame
LEAF_FRAME_DEPTH = 0.040

# Louver slat dimensions
SLAT_DEPTH = 0.040       # slat blade depth (front-to-back before tilt)
SLAT_THICK = 0.005       # slat blade thickness
SLAT_SPACING = 0.042     # vertical center-to-center pitch
SLAT_ANGLE = 0.52        # tilt angle in radians (~30°), front edge up
SLAT_EMBED = 0.003       # how far slat ends embed into the stile/jamb for connectivity

# Two equal bays inside the outer frame.
INNER_W = UNIT_W - 2 * FRAME_FACE           # clear width between jambs
INNER_H = UNIT_H - 2 * FRAME_FACE           # clear height between head and sill
BAY_W = INNER_W / 2.0                        # nominal width of each bay

# Y placement (depth).
Y_FIXED = -0.010                             # center plane of fixed panel (rear)
Y_LEAF = FRAME_DEPTH / 2.0 + LEAF_FRAME_DEPTH / 2.0 + 0.004

# Colors / materials
BLACK_ALU = (0.09, 0.09, 0.10)               # matte charcoal-black aluminium
DARK_ALU = (0.13, 0.13, 0.14)                # leaf frame, slightly lighter
SLAT_COLOR = (0.52, 0.40, 0.26)              # warm natural wood (teak/oak)
HANDLE_METAL = (0.16, 0.16, 0.17)            # slim flush pull handle


def _add_louver_slats(
    part,
    *,
    name_prefix: str,
    width: float,
    height: float,
    center_xyz: tuple[float, float, float],
    slat_depth: float = SLAT_DEPTH,
    slat_thick: float = SLAT_THICK,
    slat_spacing: float = SLAT_SPACING,
    slat_angle: float = SLAT_ANGLE,
    material: str = "slat_wood",
) -> int:
    """Emit horizontal louver slats into *part* filling the given height.

    Each slat is a thin tilted blade emitted as a named Box visual with
    ``name_prefix_i`` naming. The slats are centered vertically around
    ``center_xyz[2]`` and share a uniform tilt (roll around X).
    Returns the number of slats emitted.
    """
    n = max(2, int(height / slat_spacing))
    total_span = (n - 1) * slat_spacing
    z_start = center_xyz[2] - total_span / 2.0

    for i in range(n):
        z = z_start + i * slat_spacing
        part.visual(
            Box((width, slat_depth, slat_thick)),
            origin=Origin(
                xyz=(center_xyz[0], center_xyz[1], z),
                rpy=(slat_angle, 0.0, 0.0),
            ),
            name=f"{name_prefix}_{i}",
            material=material,
        )
    return n


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_panel_sliding_louver_door")

    model.material("black_alu", rgba=(*BLACK_ALU, 1.0))
    model.material("dark_alu", rgba=(*DARK_ALU, 1.0))
    model.material("slat_wood", rgba=(*SLAT_COLOR, 1.0))
    model.material("handle_metal", rgba=(*HANDLE_METAL, 1.0))

    # ------------------------------------------------------------------
    # ROOT: outer frame + fixed louver panel (static)
    # ------------------------------------------------------------------
    frame = model.part("door_frame")

    half_w = UNIT_W / 2.0

    # Left jamb (vertical stile).
    frame.visual(
        Box((FRAME_FACE, FRAME_DEPTH, UNIT_H)),
        origin=Origin(xyz=(-half_w + FRAME_FACE / 2.0, 0.0, UNIT_H / 2.0)),
        name="jamb_left",
        material="black_alu",
    )
    # Right jamb (vertical stile).
    frame.visual(
        Box((FRAME_FACE, FRAME_DEPTH, UNIT_H)),
        origin=Origin(xyz=(half_w - FRAME_FACE / 2.0, 0.0, UNIT_H / 2.0)),
        name="jamb_right",
        material="black_alu",
    )
    # Head (top rail), spanning between the jambs.
    frame.visual(
        Box((INNER_W, FRAME_DEPTH, FRAME_FACE)),
        origin=Origin(xyz=(0.0, 0.0, UNIT_H - FRAME_FACE / 2.0)),
        name="head_rail",
        material="black_alu",
    )
    # Sill (bottom rail) sitting on the floor.
    frame.visual(
        Box((INNER_W, FRAME_DEPTH, FRAME_FACE)),
        origin=Origin(xyz=(0.0, 0.0, FRAME_FACE / 2.0)),
        name="sill_rail",
        material="black_alu",
    )
    # Center meeting stile between the two bays (interlock post for the leaf).
    frame.visual(
        Box((FRAME_FACE, FRAME_DEPTH, INNER_H)),
        origin=Origin(xyz=(0.0, 0.0, UNIT_H / 2.0)),
        name="meeting_stile",
        material="black_alu",
    )

    # Head and sill tracks the moving leaf runs in.
    track_y = Y_LEAF
    frame_front = FRAME_DEPTH / 2.0
    lip_back = frame_front - 0.006
    lip_front = track_y + 0.004
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

    # Fixed louver panel filling the LEFT bay (rear plane).
    # Slat width extends SLAT_EMBED into the jamb and meeting stile for
    # geometric connectivity (represents the mortised/pinned mounting).
    fixed_glass_w = BAY_W - FRAME_FACE / 2.0          # clear bay width
    fixed_slat_w = fixed_glass_w + 2 * SLAT_EMBED     # embed into frame members
    fixed_slat_cx = -BAY_W / 2.0 - FRAME_FACE / 4.0   # centered in left bay
    n_fixed = _add_louver_slats(
        frame,
        name_prefix="slat_fixed",
        width=fixed_slat_w,
        height=INNER_H,
        center_xyz=(fixed_slat_cx, Y_FIXED, UNIT_H / 2.0),
    )

    # ------------------------------------------------------------------
    # MOVING LEAF: slim-framed sliding louver panel (rides the track)
    # ------------------------------------------------------------------
    leaf = model.part("sliding_leaf")

    leaf_w = BAY_W                          # fills the right bay
    leaf_h = INNER_H - 0.032
    fa = LEAF_FRAME_FACE
    fd = LEAF_FRAME_DEPTH

    # Leaf frame: four members (local coords, leaf center at origin).
    stile_h = leaf_h - 2 * fa
    # Left stile.
    leaf.visual(
        Box((fa, fd, stile_h)),
        origin=Origin(xyz=(-leaf_w / 2.0 + fa / 2.0, 0.0, 0.0)),
        name="leaf_stile_a",
        material="dark_alu",
    )
    # Right stile.
    leaf.visual(
        Box((fa, fd, stile_h)),
        origin=Origin(xyz=(leaf_w / 2.0 - fa / 2.0, 0.0, 0.0)),
        name="leaf_stile_b",
        material="dark_alu",
    )
    # Top rail.
    leaf.visual(
        Box((leaf_w - 2 * fa, fd, fa)),
        origin=Origin(xyz=(0.0, 0.0, leaf_h / 2.0 - fa / 2.0)),
        name="leaf_top_rail",
        material="dark_alu",
    )
    # Bottom rail.
    leaf.visual(
        Box((leaf_w - 2 * fa, fd, fa)),
        origin=Origin(xyz=(0.0, 0.0, -leaf_h / 2.0 + fa / 2.0)),
        name="leaf_bottom_rail",
        material="dark_alu",
    )

    # Louver slat infill inside the leaf frame.
    leaf_infill_w = leaf_w - 2 * fa
    leaf_infill_h = leaf_h - 2 * fa
    leaf_slat_w = leaf_infill_w + 2 * SLAT_EMBED  # embed into stiles
    n_leaf = _add_louver_slats(
        leaf,
        name_prefix="slat_leaf",
        width=leaf_slat_w,
        height=leaf_infill_h,
        center_xyz=(0.0, 0.0, 0.0),
    )

    # Slim flush pull handle on the leading (left) stile, room-side face (+Y).
    handle_z = 1.0 - UNIT_H / 2.0           # ~1.0 m above ground in world
    handle_x = -leaf_w / 2.0 + fa / 2.0     # centered on the leading stile
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

    # --- Intentional engagement overlap (retained leaf in the head channel) ---
    ctx.allow_overlap(
        frame,
        leaf,
        elem_a="head_track",
        elem_b="leaf_top_rail",
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
        elem_a="leaf_top_rail",
        elem_b="head_track",
        min_overlap=0.004,
        name="leaf top rail stays engaged in the head channel",
    )

    # --- Louver slats exist on both panels (for-i-in-range emission) ---
    # At least one slat from each set must exist; the helper emits many.
    fixed_slat_0 = frame.get_visual("slat_fixed_0")
    fixed_slat_1 = frame.get_visual("slat_fixed_1")
    leaf_slat_0 = leaf.get_visual("slat_leaf_0")
    leaf_slat_1 = leaf.get_visual("slat_leaf_1")
    ctx.check(
        "fixed panel has louver slats",
        fixed_slat_0 is not None and fixed_slat_1 is not None,
        details="slat_fixed_0 or slat_fixed_1 missing",
    )
    ctx.check(
        "sliding leaf has louver slats",
        leaf_slat_0 is not None and leaf_slat_1 is not None,
        details="slat_leaf_0 or slat_leaf_1 missing",
    )

    # Count slats to verify the for-loop emitted a reasonable number
    fixed_slat_count = sum(
        1 for v in frame.visuals if v.name.startswith("slat_fixed_")
    )
    leaf_slat_count = sum(
        1 for v in leaf.visuals if v.name.startswith("slat_leaf_")
    )
    ctx.check(
        "fixed panel has many louver slats (loop emission)",
        fixed_slat_count >= 20,
        details=f"fixed slat count={fixed_slat_count}",
    )
    ctx.check(
        "leaf panel has many louver slats (loop emission)",
        leaf_slat_count >= 20,
        details=f"leaf slat count={leaf_slat_count}",
    )

    # --- Slats are tilted (not flat) — proves louver ventilation angle ---
    # The slat visuals should have non-zero roll in their origin rpy.
    slat_vis = fixed_slat_0
    if slat_vis is not None:
        roll = slat_vis.origin.rpy[0] if slat_vis.origin.rpy else 0.0
        ctx.check(
            "louver slats are tilted for ventilation",
            abs(roll) > 0.1,
            details=f"slat roll={roll:.3f} rad (expected ~0.5 rad)",
        )

    # --- Handle is present ---
    handle = leaf.get_visual("handle_lever")
    ctx.check(
        "recessed lever handle exists",
        handle is not None,
        details="handle_lever visual missing",
    )

    # --- Slat material is opaque wood (not transparent glass) ---
    slat_mat = next((m for m in object_model.materials if m.name == "slat_wood"), None)
    alpha = slat_mat.rgba[3] if slat_mat is not None and slat_mat.rgba else 0.0
    ctx.check(
        "slat material is opaque",
        slat_mat is not None and alpha > 0.9,
        details=f"slat_wood alpha={alpha}",
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
        frame_aabb is not None and abs((frame_aabb[1][2] - frame_aabb[0][2]) - UNIT_H) < 0.02,
        details=f"frame height={None if frame_aabb is None else frame_aabb[1][2] - frame_aabb[0][2]}",
    )
    ctx.check(
        "leaf does not sink below the floor",
        leaf_aabb is not None and leaf_aabb[0][2] >= -1e-4,
        details=f"leaf z-min={None if leaf_aabb is None else leaf_aabb[0][2]}",
    )

    # --- Closed pose: leaf seats in the right bay, beside the fixed panel ---
    ctx.expect_origin_gap(
        leaf, frame,
        axis="x",
        min_gap=0.2,
        name="closed leaf sits in the right bay (right of fixed panel center)",
    )
    # Leaf rides forward of the fixed panel in Y; slats clear in depth.
    ctx.expect_gap(
        leaf, frame,
        axis="y",
        positive_elem="slat_leaf_0",
        negative_elem="slat_fixed_0",
        min_gap=0.0,
        name="leaf slats clear the fixed slats in depth",
    )

    # --- Articulation actually slides the leaf open (and the right way) ---
    closed_pos = ctx.part_world_position(leaf)
    upper = slide.motion_limits.upper
    with ctx.pose({slide: upper}):
        open_pos = ctx.part_world_position(leaf)
        ctx.expect_overlap(
            leaf, frame,
            axes="x",
            elem_a="slat_leaf_0",
            elem_b="slat_fixed_0",
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
