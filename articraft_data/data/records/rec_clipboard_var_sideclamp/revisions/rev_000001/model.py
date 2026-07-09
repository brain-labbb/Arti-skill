from __future__ import annotations

# Realistic articulated clipboard with a side-mounted metal spring clamp.
#
# Object identity (from picture/Stationary/Clipboard/001.png, landscape variant):
#   A letter/A4-size blue plastic clipboard in LANDSCAPE orientation with a
#   low-profile chrome spring clamp riveted to the long LEFT side edge. The
#   clamp is the classic torsion-spring jaw mechanism: a fixed metal base
#   bracket pinned to the board carries a pivot barrel, and a curved metal
#   lever (with two dark plastic finger caps at the rear and a rolled gripping
#   lip at the front) rotates on that barrel. A hidden torsion spring keeps the
#   front lip pressed against the board; pressing the rear finger pads lifts
#   the front jaw to insert paper.
#
# Layout: the board lies flat in the XY plane, top surface facing +Z. The board
# is in LANDSCAPE orientation: long dimension (0.330m) along Y, short dimension
# (0.232m) along +X. The clamp is mounted at the -X (left) edge, centered on Y.
# The clamp width runs along Y (along the long edge), so the lever pivots about
# a Y axis. Positive joint motion lifts the front gripping lip up and away from
# the board (the "open" gesture), pressing paper against the board from the side.

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
)

# ----------------------------------------------------------------------------
# Real-world dimensions (meters)
# ----------------------------------------------------------------------------
BOARD_LEN = 0.330  # along Y (long dimension, runs along the clamp edge)
BOARD_WID = 0.232  # along +X (short dimension, perpendicular to clamp edge)
BOARD_THK = 0.0032  # plastic panel thickness
BOARD_CORNER_R = 0.012

# Landscape board: x in [0, BOARD_WID], centered on Y, top face at z=BOARD_THK.
# The clamp lives near the left edge (small x).

CLAMP_WID = 0.078  # clamp width along Y
CLAMP_CENTER_X = 0.034  # x of the pivot barrel center
PIVOT_Z = BOARD_THK + 0.0090  # height of the pivot axis above board top face
PIVOT_RADIUS = 0.0030  # pin/barrel radius

# Base bracket footprint (the riveted-down stationary metal part).
BASE_X0 = 0.012
BASE_X1 = 0.058
# Crown of the fixed base hump. Kept BELOW the pivot/lever underside so the
# moving cover arches over it without colliding; only the barrel reaches up to
# the pivot to be captured by the lever sleeve.
BASE_TOP_Z = BOARD_THK + 0.0058

# Lever (moving jaw) geometry, authored in the JOINT frame so the mesh frame and
# articulation frame coincide. The joint frame is placed at the pivot barrel.
# In the joint-local frame: +x_local points toward the board's front gripping
# lip (toward +X world at q=0), +z_local is up.
LEVER_FRONT_X = 0.026  # front lip extends this far ahead of pivot (local +x)
LEVER_BACK_X = -0.024  # finger-pad tail extends this far behind pivot (local -x)
LEVER_CROWN_Z = 0.0080  # height of the curved metal cover above the pivot

MAT_BOARD = "clipboard_blue"
MAT_METAL = "chrome_clamp"
MAT_CAP = "clamp_cap_black"


# ----------------------------------------------------------------------------
# Geometry builders
# ----------------------------------------------------------------------------
def _board_solid() -> cq.Workplane:
    """Thin rounded-corner plastic panel in landscape, top face at z=BOARD_THK.

    Landscape orientation: short dimension (BOARD_WID) along X, long dimension
    (BOARD_LEN) along Y. Board occupies x in [0, BOARD_WID], centered on Y.
    """
    board = (
        cq.Workplane("XY")
        .box(BOARD_WID, BOARD_LEN, BOARD_THK, centered=(False, True, False))
        .edges("|Z")
        .fillet(BOARD_CORNER_R)
    )
    # Lift so the bottom face sits on z=0 and top face on z=BOARD_THK.
    return board.translate((0.0, 0.0, BOARD_THK / 2.0))


def _clamp_base_solid() -> cq.Workplane:
    """Fixed metal base bracket: a low riveted plate that rises to a barrel mount.

    Built in WORLD coordinates (board top at z=BOARD_THK). Includes the two
    rivet/pin bosses and the rear barrel that the lever pivots on, plus side
    cheeks that capture the lever pin.
    """
    base_len = BASE_X1 - BASE_X0
    base_cx = (BASE_X0 + BASE_X1) / 2.0

    # Flat riveted footplate hugging the board top.
    plate_thk = 0.0016
    plate = (
        cq.Workplane("XY")
        .workplane(offset=BOARD_THK + plate_thk / 2.0)
        .box(base_len, CLAMP_WID, plate_thk)
        .translate((base_cx, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.004)
    )

    # Rear hump that lifts up to carry the pivot barrel. A wedge-like block
    # rising from the footplate to the barrel height near the top edge.
    hump_len = 0.020
    hump_cx = CLAMP_CENTER_X
    hump = (
        cq.Workplane("XY")
        .workplane(offset=BOARD_THK)
        .box(hump_len, CLAMP_WID, BASE_TOP_Z - BOARD_THK, centered=(True, True, False))
        .translate((hump_cx, 0.0, 0.0))
        .edges("|Y and >Z")
        .fillet(0.0035)
    )

    # Two side cheeks that hold the pivot barrel ends, flanking the lever width.
    cheek_thk = 0.0040
    cheek_y = CLAMP_WID / 2.0 - cheek_thk / 2.0
    cheeks = cq.Workplane("XY")
    for sign in (-1.0, 1.0):
        cheek = (
            cq.Workplane("XY")
            .workplane(offset=BOARD_THK)
            .box(0.018, cheek_thk, PIVOT_Z - BOARD_THK + 0.0030, centered=(True, True, False))
            .translate((CLAMP_CENTER_X, sign * cheek_y, 0.0))
        )
        cheeks = cheeks.union(cheek)

    # Pivot barrel along Y between the cheeks.
    barrel = (
        cq.Workplane("XZ")
        .workplane(offset=-CLAMP_WID / 2.0)
        .center(CLAMP_CENTER_X, PIVOT_Z)
        .circle(PIVOT_RADIUS + 0.0009)
        .extrude(CLAMP_WID)
    )

    # Two rivet heads on the footplate front (visible fasteners through board).
    rivets = cq.Workplane("XY")
    for sign in (-1.0, 1.0):
        rivet = (
            cq.Workplane("XY")
            .workplane(offset=BOARD_THK + plate_thk)
            .center(BASE_X1 - 0.008, sign * 0.020)
            .circle(0.0035)
            .extrude(0.0012)
        )
        rivets = rivets.union(rivet)

    base = plate.union(hump).union(cheeks).union(barrel).union(rivets)
    return base


def _clamp_lever_solid() -> cq.Workplane:
    """Moving spring-clamp jaw, authored in the JOINT-LOCAL frame.

    Origin is the pivot axis. +x_local -> board's front lip; +z_local -> up.
    Contains: the curved sheet-metal cover, the front rolled gripping lip that
    presses on the board, the rear finger-pad shelf, and the pin sleeve through
    the barrel.
    """
    half_w = CLAMP_WID / 2.0 - 0.0050  # lever slightly narrower than base cheeks

    # Curved sheet-metal cover: a thin arched shell over the base hump. The
    # profile is a clean simple polygon traced counter-clockwise in the XZ
    # plane: along the underside (low) from rear tail to front, then back along
    # the arched top (high). Extruded across the clamp width.
    cover = (
        cq.Workplane("XZ")
        # underside, rear -> front (low edge)
        .moveTo(LEVER_BACK_X, 0.0026)
        .lineTo(-0.006, 0.0040)
        .lineTo(0.010, 0.0028)
        .lineTo(LEVER_FRONT_X, 0.0010)
        # up the front face
        .lineTo(LEVER_FRONT_X, 0.0024)
        # arched top, front -> rear (high edge)
        .lineTo(0.008, LEVER_CROWN_Z - 0.0008)
        .lineTo(-0.008, LEVER_CROWN_Z)
        .lineTo(LEVER_BACK_X, 0.0050)
        .close()
        .extrude(half_w, both=True)
    )

    # Front gripping lip: a downward rolled metal edge that contacts the board.
    # It hangs from the cover front (z>=0.0010) down to ~0.6mm above the board
    # top. Spanning from above the cover underside down past it guarantees the
    # lip fuses with the cover into one connected solid.
    lip_top = 0.0024
    lip_bottom = -(PIVOT_Z - BOARD_THK - 0.0006)  # ~0.6mm above board in world
    lip_h = lip_top - lip_bottom
    lip = (
        cq.Workplane("XY")
        .box(0.0060, 2.0 * half_w, lip_h, centered=(True, True, False))
        .edges("|Y and <Z")
        .fillet(0.0010)
        .translate((LEVER_FRONT_X - 0.0030, 0.0, lip_bottom))
    )

    # Rear finger shelf: flat metal tab where the two plastic caps sit. Made
    # tall enough to intersect the cover underside so it fuses into the lever.
    shelf = (
        cq.Workplane("XY")
        .box(0.014, 2.0 * half_w, 0.0040, centered=(True, True, True))
        .translate((LEVER_BACK_X + 0.006, 0.0, 0.0030))
    )

    # Pin sleeve: tube wrapping the pivot barrel so the lever is captured on it.
    sleeve = (
        cq.Workplane("XZ")
        .workplane(offset=-half_w)
        .center(0.0, 0.0)
        .circle(PIVOT_RADIUS + 0.0018)
        .circle(PIVOT_RADIUS + 0.0010)
        .extrude(2.0 * half_w)
    )

    lever = cover.union(lip).union(shelf).union(sleeve)
    return lever


def _finger_caps_solid() -> cq.Workplane:
    """Two dark plastic finger pads on the rear shelf, in JOINT-LOCAL frame."""
    half_w = CLAMP_WID / 2.0 - 0.0050
    caps = cq.Workplane("XY")
    for sign in (-1.0, 1.0):
        cap = (
            cq.Workplane("XY")
            .box(0.0110, 0.0150, 0.0040, centered=(True, True, False))
            .edges("|Z")
            .fillet(0.0016)
            .translate((LEVER_BACK_X + 0.004, sign * (half_w - 0.010), 0.0044))
        )
        caps = caps.union(cap)
    return caps


# ----------------------------------------------------------------------------
# Model
# ----------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clipboard_spring_clamp")

    model.material(MAT_BOARD, rgba=(0.13, 0.45, 0.92, 1.0))
    model.material(MAT_METAL, rgba=(0.78, 0.80, 0.83, 1.0))
    model.material(MAT_CAP, rgba=(0.10, 0.10, 0.11, 1.0))

    # --- Board (root) ---
    board = model.part("board")
    board.visual(
        mesh_from_cadquery(_board_solid(), "board_panel"),
        material=MAT_BOARD,
        name="board_panel",
    )

    # --- Fixed clamp base bracket (riveted to board) ---
    clamp_base = model.part("clamp_base")
    clamp_base.visual(
        mesh_from_cadquery(_clamp_base_solid(), "clamp_base"),
        material=MAT_METAL,
        name="clamp_base",
    )

    # --- Moving spring-clamp lever (jaw) ---
    clamp_lever = model.part("clamp_lever")
    clamp_lever.visual(
        mesh_from_cadquery(_clamp_lever_solid(), "clamp_lever"),
        material=MAT_METAL,
        name="clamp_lever",
    )
    clamp_lever.visual(
        mesh_from_cadquery(_finger_caps_solid(), "finger_caps"),
        material=MAT_CAP,
        name="finger_caps",
    )

    # --- Board carries the fixed clamp base (rigid mount) ---
    model.articulation(
        "board_to_clamp_base",
        ArticulationType.FIXED,
        parent=board,
        child=clamp_base,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # --- Clamp base carries the pivoting lever on the barrel ---
    # The lever mesh is authored in joint-local coords (origin at pivot axis,
    # local +x toward the front lip). Placing the joint frame at the barrel in
    # world makes the frames line up. The front lip sits at local +x (world +X);
    # axis is -Y so that by the right-hand rule positive q rotates the front lip
    # UP toward +Z, i.e. opens the jaw away from the board.
    model.articulation(
        "base_to_lever",
        ArticulationType.REVOLUTE,
        parent=clamp_base,
        child=clamp_lever,
        origin=Origin(xyz=(CLAMP_CENTER_X, 0.0, PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=0.42),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    board = object_model.get_part("board")
    clamp_base = object_model.get_part("clamp_base")
    clamp_lever = object_model.get_part("clamp_lever")
    pivot = object_model.get_articulation("base_to_lever")

    lever_vis = clamp_lever.get_visual("clamp_lever")
    base_vis = clamp_base.get_visual("clamp_base")

    # The lever sleeve is intentionally captured around the fixed pivot barrel,
    # so the moving cover nests over the base near the pivot. Allow that local
    # mechanical capture overlap; it is proved by the pose checks below.
    ctx.allow_overlap(
        clamp_base,
        clamp_lever,
        elem_a=base_vis,
        elem_b=lever_vis,
        reason="Lever pin sleeve is captured around the fixed pivot barrel; the curved cover nests over the base at the pivot.",
    )

    # ---- Joint contract: revolute about the clamp-width (Y) axis ----
    ctx.check(
        "pivot is revolute",
        pivot.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={pivot.articulation_type}",
    )
    ax = tuple(pivot.axis)
    ctx.check(
        "pivot axis runs along the side edge (Y)",
        abs(ax[1]) > 0.99 and abs(ax[0]) < 1e-6 and abs(ax[2]) < 1e-6,
        details=f"axis={ax}",
    )

    # ---- Board hero geometry: landscape thin flat panel ----
    bmin, bmax = ctx.part_world_aabb(board)
    board_x = bmax[0] - bmin[0]  # short dimension in landscape
    board_y = bmax[1] - bmin[1]  # long dimension in landscape
    board_thk = bmax[2] - bmin[2]
    ctx.check(
        "board reads as a landscape thin clipboard panel (long along Y)",
        board_y > 0.30 and board_x > 0.21 and board_x < 0.26 and board_thk < 0.006,
        details=f"x={board_x:.3f} y={board_y:.3f} thk={board_thk:.4f}",
    )

    # ---- Clamp sits on the left (-X) side edge of the landscape board ----
    cmin, cmax = ctx.part_world_aabb(clamp_base)
    ctx.check(
        "clamp base is near the left (-X) edge of the board",
        cmax[0] < 0.10 and cmin[0] >= bmin[0] - 1e-4,
        details=f"clamp x=[{cmin[0]:.3f},{cmax[0]:.3f}] board x0={bmin[0]:.3f}",
    )
    ctx.check(
        "clamp base is centered along the long (Y) edge",
        abs((cmin[1] + cmax[1]) / 2.0) < 0.02,
        details=f"clamp y center={(cmin[1] + cmax[1]) / 2.0:.3f}",
    )
    ctx.check(
        "clamp base rises above the board top face",
        cmax[2] > board_thk + 0.005,
        details=f"clamp top z={cmax[2]:.4f} board top z={bmax[2]:.4f}",
    )

    # ---- Base is mounted to the board (rests on its top face, no float) ----
    ctx.expect_overlap(clamp_base, board, axes="xy", min_overlap=0.02)
    ctx.expect_gap(
        clamp_base,
        board,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0020,
        name="clamp base seats on the board top face",
    )

    # ---- Lever is captured on the base pivot (not floating) ----
    ctx.expect_contact(
        clamp_lever,
        clamp_base,
        elem_a=lever_vis,
        elem_b=base_vis,
        contact_tol=0.001,
        name="lever sleeve is captured on the base pivot barrel",
    )

    # ---- Finger caps present and are the dark rear pads on the lever ----
    caps = clamp_lever.get_visual("finger_caps")
    pmin, pmax = ctx.part_element_world_aabb(clamp_lever, elem=caps)
    ctx.check(
        "finger caps sit on the rear (toward -X) of the lever",
        pmax[0] < cmax[0],
        details=f"caps x max={pmax[0]:.3f} clamp x max={cmax[0]:.3f}",
    )

    # The front gripping lip is the front (high world x) end of the lever; the
    # rear finger pads are the low-x end. Pressing the rear pads down lifts the
    # front jaw off the board. The rear pads are isolable as the finger caps;
    # the front jaw height is read from the lever's top (max z).

    # ---- Closed (rest) pose: front lip presses near the board, jaw shut ----
    with ctx.pose({pivot: 0.0}):
        lev_min0, lev_max0 = ctx.part_world_aabb(clamp_lever)
        lev_max0_top = lev_max0[2]
        cap_min0, cap_max0 = ctx.part_element_world_aabb(clamp_lever, elem=caps)
        cap_zc0 = (cap_min0[2] + cap_max0[2]) / 2.0
        front_lip_z0 = lev_min0[2]
        ctx.check(
            "closed jaw front lip reaches down near the board top face",
            front_lip_z0 < board_thk + 0.004,
            details=f"front lip z={front_lip_z0:.4f} board top={board_thk:.4f}",
        )
        # The lever overhangs the board top surface (the paper-gripping zone).
        ctx.expect_overlap(
            clamp_lever,
            board,
            axes="xy",
            min_overlap=0.01,
            name="closed jaw overlaps the board (grips paper area)",
        )

    # ---- Open pose: pressing rear pads down lifts the front lip up & away ----
    upper = pivot.motion_limits.upper
    with ctx.pose({pivot: upper}):
        cap_min1, cap_max1 = ctx.part_element_world_aabb(clamp_lever, elem=caps)
        cap_zc1 = (cap_min1[2] + cap_max1[2]) / 2.0
        # Front lip z = lowest z at the front (max-x) half of the lever.
        lev_min1, lev_max1 = ctx.part_world_aabb(clamp_lever)
        ctx.check(
            "opening presses the rear finger pads down",
            cap_zc1 < cap_zc0 - 0.004,
            details=f"closed cap z={cap_zc0:.4f} open cap z={cap_zc1:.4f}",
        )
        ctx.check(
            "opening lifts the front jaw up off the board",
            lev_max1[2] > lev_max0_top + 0.005,
            details=f"closed front top z={lev_max0_top:.4f} open front top z={lev_max1[2]:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
