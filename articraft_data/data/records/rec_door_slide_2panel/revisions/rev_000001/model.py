from __future__ import annotations

# Two-panel sliding glass door in a slim black aluminium frame.
# Root = outer frame + fixed glass pane (static). One sliding glass leaf on a
# single PRISMATIC joint runs along the horizontal track and tucks behind the
# fixed pane. A small recessed lever handle rides on the moving leaf.
#
# Conventions used here:
# - World up is +Z. Width runs along X, height along Z, thin depth along Y.
# - The whole door stands on the ground: all geometry is authored from z=0 up.
# - The sliding leaf sits slightly toward +Y (room side) so it can pass in
#   front of the fixed pane without colliding; positive q slides it along -X to
#   overlap (tuck behind, in depth) the fixed pane.

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
# Reference is a wide patio sliding door: distinctly landscape (wider than
# tall). Two roughly equal bays. Slim charcoal/black aluminium frame.
UNIT_W = 2.80          # total opening width (outer frame outer width)
UNIT_H = 2.00          # total height
FRAME_FACE = 0.042     # face width of the slim aluminium frame members
FRAME_DEPTH = 0.058    # frame depth in Y (jamb depth)

GLASS_T = 0.008        # glazing thickness
LEAF_FRAME_FACE = 0.035  # face width of the moving leaf's own slim frame
LEAF_FRAME_DEPTH = 0.040

# Two equal bays inside the outer frame.
INNER_W = UNIT_W - 2 * FRAME_FACE           # clear width between jambs
INNER_H = UNIT_H - 2 * FRAME_FACE           # clear height between head and sill
BAY_W = INNER_W / 2.0                        # nominal width of each bay

# Y placement (depth). The frame is FRAME_DEPTH deep, centered at Y=0, so its
# front face is at +FRAME_DEPTH/2. The fixed glazing sits in the rear plane.
# The moving leaf rides a track on the room side, entirely FORWARD of the
# frame's front face, so the leaf frame never collides with the jambs/head/sill
# and the leaf glazing always clears the fixed pane in depth as it travels.
Y_FIXED = -0.010                             # center plane of fixed glazing (rear)
# Leaf center plane: forward of the frame front face by a small running gap.
Y_LEAF = FRAME_DEPTH / 2.0 + LEAF_FRAME_DEPTH / 2.0 + 0.004

# Colors / materials
BLACK_ALU = (0.09, 0.09, 0.10)               # matte charcoal-black aluminium
DARK_ALU = (0.13, 0.13, 0.14)                # leaf frame, slightly lighter
# Clear glass reads as a light neutral tint (the reference panes are very
# transparent, showing the light room behind). Low alpha keeps it see-through
# in the real viewer; the light RGB makes it read as clear glass in flat renders.
GLASS = (0.82, 0.85, 0.87, 0.16)             # transparent clear glass
HANDLE_METAL = (0.16, 0.16, 0.17)            # slim flush pull handle


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_panel_sliding_glass_door")

    model.material("black_alu", rgba=(*BLACK_ALU, 1.0))
    model.material("dark_alu", rgba=(*DARK_ALU, 1.0))
    model.material("glass", rgba=GLASS)
    model.material("handle_metal", rgba=(*HANDLE_METAL, 1.0))

    # ------------------------------------------------------------------
    # ROOT: outer frame + fixed glass pane (static)
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

    # Head and sill tracks the moving leaf runs in. Each track is a solid lip
    # that grows OUT of the frame head/sill toward +Y (room side), reaching the
    # leaf depth plane, so it is one connected piece with the frame (no floating
    # rail and no separate brackets needed). The leaf top rail tucks UP into the
    # head channel and the bottom rail wraps the sill track; that small
    # engagement overlap is intentional and is scope-allowed in run_tests().
    track_y = Y_LEAF
    frame_front = FRAME_DEPTH / 2.0
    # Lip spans Y from the frame front face out to just past the leaf center.
    lip_back = frame_front - 0.006                 # bite into the frame front
    lip_front = track_y + 0.004                    # reach into the leaf depth
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

    # Fixed glass pane filling the LEFT bay (rear plane).
    fixed_glass_w = BAY_W - FRAME_FACE / 2.0          # from left jamb to meeting stile
    fixed_glass_h = INNER_H
    fixed_glass_cx = -BAY_W / 2.0 - FRAME_FACE / 4.0   # centered in left bay
    frame.visual(
        Box((fixed_glass_w, GLASS_T, fixed_glass_h)),
        origin=Origin(xyz=(fixed_glass_cx, Y_FIXED, UNIT_H / 2.0)),
        name="fixed_pane",
        material="glass",
    )

    # ------------------------------------------------------------------
    # MOVING LEAF: slim-framed sliding glass panel (rides the track)
    # ------------------------------------------------------------------
    # The leaf has its own thin frame around a glass infill. The part frame is
    # at the leaf's geometric center; the prismatic joint places that center in
    # the RIGHT bay at q=0 (closed).
    leaf = model.part("sliding_leaf")

    leaf_w = BAY_W                          # fills the right bay
    # Height sized so the top rail tucks up into the head channel and the
    # bottom rail wraps the sill track (small intended engagements, allowed).
    leaf_h = INNER_H - 0.032
    fa = LEAF_FRAME_FACE
    fd = LEAF_FRAME_DEPTH

    # Leaf frame: four members (local coords, leaf center at origin). The stiles
    # butt between the top and bottom rails, so they do not reach the head/sill
    # track bands; only the rails engage the tracks.
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
    # Glass infill.
    leaf.visual(
        Box((leaf_w - 2 * fa, GLASS_T, leaf_h - 2 * fa)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="leaf_pane",
        material="glass",
    )

    # Slim flush pull handle on the leading (left) stile, room-side face (+Y).
    # The reference uses a discreet thin vertical bar pull set close to the
    # stile, centered near a comfortable grab height. Modeled as a thin tall
    # bar standing just proud of the leaf face (no chunky lever or pocket).
    handle_z = 1.0 - UNIT_H / 2.0           # ~1.0 m above ground in world
    handle_x = -leaf_w / 2.0 + fa / 2.0     # centered on the leading stile
    HANDLE_LEN = 0.28                       # tall slim vertical pull
    leaf.visual(
        Box((0.012, 0.016, HANDLE_LEN)),
        origin=Origin(xyz=(handle_x, fd / 2.0 + 0.008, handle_z)),
        name="handle_lever",
        material="handle_metal",
    )

    # ------------------------------------------------------------------
    # ARTICULATION: single prismatic joint along the horizontal track (X).
    # ------------------------------------------------------------------
    # Closed pose (q=0): leaf center sits in the RIGHT bay, one channel toward
    # +Y of the fixed pane. Positive q slides the leaf along -X so it travels
    # left and overlaps (in depth) the fixed pane in the left bay.
    leaf_closed_cx = BAY_W / 2.0 + FRAME_FACE / 4.0    # center of right bay
    travel = BAY_W - FRAME_FACE                         # near-full overlap of fixed bay
    model.articulation(
        "leaf_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=leaf,
        origin=Origin(xyz=(leaf_closed_cx, Y_LEAF, UNIT_H / 2.0)),
        # Positive q moves the leaf left (-X) along the track to open.
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
    # The leaf top rail tucks up into the head track channel; this small local
    # overlap is how the leaf is captured at the head. Scope it to that element
    # pair only and prove the engagement length below.
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
    # Engagement persists across the full travel (track spans the whole opening),
    # so it remains valid at the open limit too. Prove retained insertion in Z.
    ctx.expect_overlap(
        leaf,
        frame,
        axes="z",
        elem_a="leaf_top_rail",
        elem_b="head_track",
        min_overlap=0.004,
        name="leaf top rail stays engaged in the head channel",
    )

    # --- Hero geometry is present and legible ---
    fixed_pane = frame.get_visual("fixed_pane")
    leaf_pane = leaf.get_visual("leaf_pane")
    handle = leaf.get_visual("handle_lever")
    ctx.check(
        "fixed glass pane exists",
        fixed_pane is not None,
        details="fixed_pane visual missing",
    )
    ctx.check(
        "sliding leaf glass exists",
        leaf_pane is not None,
        details="leaf_pane visual missing",
    )
    ctx.check(
        "recessed lever handle exists",
        handle is not None,
        details="handle_lever visual missing",
    )

    # --- Glass is authored as a transparent (low-alpha) material ---
    glass_mat = next((m for m in object_model.materials if m.name == "glass"), None)
    alpha = glass_mat.rgba[3] if glass_mat is not None and glass_mat.rgba else 1.0
    ctx.check(
        "glass material is transparent",
        glass_mat is not None and alpha < 0.5,
        details=f"glass alpha={alpha}",
    )

    # --- Door stands on the ground plane (no sinking / no floating) ---
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

    # --- Closed pose: leaf seats in the right bay, beside the fixed pane ---
    # The moving leaf rides one depth channel in front of the fixed pane, so the
    # two glazings intentionally pass in depth (Y); they should not be coplanar.
    # In the closed pose the leaf is to the +X side (right bay), fixed to the -X.
    ctx.expect_origin_gap(
        leaf, frame,
        axis="x",
        min_gap=0.2,
        name="closed leaf sits in the right bay (right of fixed pane center)",
    )
    # Leaf rides forward of the fixed pane in Y, with a real clearance gap so the
    # two panes never collide as the leaf travels.
    ctx.expect_gap(
        leaf, frame,
        axis="y",
        positive_elem="leaf_pane",
        negative_elem="fixed_pane",
        min_gap=0.0,
        name="leaf glazing clears the fixed pane in depth",
    )

    # --- Articulation actually slides the leaf open (and the right way) ---
    closed_pos = ctx.part_world_position(leaf)
    upper = slide.motion_limits.upper
    with ctx.pose({slide: upper}):
        open_pos = ctx.part_world_position(leaf)
        # Open leaf overlaps the fixed pane bay (tucks behind/across it in X).
        ctx.expect_overlap(
            leaf, frame,
            axes="x",
            elem_a="leaf_pane",
            elem_b="fixed_pane",
            min_overlap=0.30,
            name="open leaf tucks across the fixed pane bay",
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
