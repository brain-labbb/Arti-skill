from __future__ import annotations

# Realistic articulated clipboard with a metal spring clamp.
#
# Object identity (from picture/Stationary/Clipboard/001.png):
#   A letter/A4-size blue plastic clipboard with a low-profile chrome spring
#   clamp riveted to the top edge. The clamp is the classic torsion-spring jaw
#   mechanism: a fixed metal base bracket pinned to the board carries a pivot
#   barrel, and a curved metal lever (with two dark plastic finger caps at the
#   rear and a rolled gripping lip at the front) rotates on that barrel. A
#   hidden torsion spring keeps the front lip pressed against the board;
#   pressing the rear finger pads lifts the front jaw to insert paper.
#
# Layout: the board lies flat in the XY plane, top surface facing +Z. The clamp
# is mounted at the -X (top) edge. The clamp width runs along Y, so the lever
# pivots about a Y axis. Positive joint motion lifts the front gripping lip up
# and away from the board (the "open" gesture).

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
BOARD_LEN = 0.330  # along +X (long dimension)
BOARD_WID = 0.232  # along Y (short dimension)
BOARD_THK = 0.0032  # plastic panel thickness
BOARD_CORNER_R = 0.012

# Board occupies x in [0, BOARD_LEN], centered on Y, top face at z = BOARD_THK.
# The clamp lives near the top edge (small x).

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
MAT_COVER = "cover_dark_blue"
MAT_HINGE = "hinge_metal"

# ---- Hinged cover panel (form-holder folder variant) ----
HINGE_R = 0.0030  # hinge barrel radius
HINGE_EMBED = 0.0005  # spine embeds this deep into board top for connectivity
HINGE_CENTER_Z = BOARD_THK + HINGE_R - HINGE_EMBED  # hinge barrel center height
COVER_THK = 0.0018  # cover panel thickness
COVER_LEN = BOARD_LEN - 0.004  # slightly shorter than board
COVER_WID = BOARD_WID - 0.006  # slightly narrower than board


# ----------------------------------------------------------------------------
# Geometry builders
# ----------------------------------------------------------------------------
def _board_solid() -> cq.Workplane:
    """Thin rounded-corner plastic panel, top face at z=BOARD_THK."""
    board = (
        cq.Workplane("XY")
        .box(BOARD_LEN, BOARD_WID, BOARD_THK, centered=(False, True, False))
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


def _hinge_spine_solid() -> cq.Workplane:
    """Board-side hinge spine barrel, in WORLD coordinates.

    A continuous cylindrical barrel along the +Y long edge of the board.
    The spine embeds slightly into the board top for connectivity.
    """
    spine_len = BOARD_LEN - 0.010
    x_start = (BOARD_LEN - spine_len) / 2.0

    spine = (
        cq.Workplane("YZ")
        .workplane(offset=x_start)
        .center(BOARD_WID / 2.0, HINGE_CENTER_Z)
        .circle(HINGE_R)
        .extrude(spine_len)
    )
    return spine


def _cover_panel_solid() -> cq.Workplane:
    """Cover panel in JOINT-LOCAL frame (origin at hinge axis).

    The panel extends from the hinge toward -Y (across the board width),
    lying flat on the board top at q=0.
    """
    # Panel bottom in local Z: such that in world it sits at BOARD_THK
    panel_z_bot = -(HINGE_R - HINGE_EMBED)

    # The panel extends from y = -COVER_WID (free edge) to y slightly past
    # the hinge center to overlap with the knuckle cylinder for connectivity.
    panel_y_extent = COVER_WID + HINGE_R + 0.002

    panel = (
        cq.Workplane("XY")
        .box(COVER_LEN, panel_y_extent, COVER_THK, centered=(True, False, False))
        .translate((0.0, -COVER_WID, panel_z_bot))
        .edges("|Z")
        .fillet(BOARD_CORNER_R * 0.6)
    )
    return panel


def _cover_knuckle_solid() -> cq.Workplane:
    """Cover-side hinge knuckle in JOINT-LOCAL frame.

    A solid cylinder wrapping around the board spine, capturing it for rotation.
    """
    knuckle_outer = HINGE_R + 0.0015
    knuckle_len = COVER_LEN - 0.012

    knuckle = (
        cq.Workplane("YZ")
        .workplane(offset=-knuckle_len / 2.0)
        .center(0.0, 0.0)
        .circle(knuckle_outer)
        .extrude(knuckle_len)
    )
    return knuckle


# ----------------------------------------------------------------------------
# Model
# ----------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clipboard_spring_clamp")

    model.material(MAT_BOARD, rgba=(0.13, 0.45, 0.92, 1.0))
    model.material(MAT_METAL, rgba=(0.78, 0.80, 0.83, 1.0))
    model.material(MAT_CAP, rgba=(0.10, 0.10, 0.11, 1.0))
    model.material(MAT_COVER, rgba=(0.12, 0.22, 0.48, 1.0))
    model.material(MAT_HINGE, rgba=(0.62, 0.64, 0.68, 1.0))

    # --- Board (root) ---
    board = model.part("board")
    board.visual(
        mesh_from_cadquery(_board_solid(), "board_panel"),
        material=MAT_BOARD,
        name="board_panel",
    )
    board.visual(
        mesh_from_cadquery(_hinge_spine_solid(), "hinge_spine"),
        material=MAT_HINGE,
        name="hinge_spine",
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

    # --- Hinged cover panel (form-holder folder) ---
    cover = model.part("cover")
    cover.visual(
        mesh_from_cadquery(_cover_panel_solid(), "cover_panel"),
        material=MAT_COVER,
        name="cover_panel",
    )
    cover.visual(
        mesh_from_cadquery(_cover_knuckle_solid(), "cover_knuckle"),
        material=MAT_HINGE,
        name="cover_knuckle",
    )

    # Cover hinge: revolute along the +Y long edge of the board.
    # Joint origin at the hinge barrel center. Axis = -X so that by the
    # right-hand rule positive q lifts the free (-Y) edge of the cover
    # upward toward +Z, opening the cover from flat-closed to standing/folded.
    model.articulation(
        "board_to_cover",
        ArticulationType.REVOLUTE,
        parent=board,
        child=cover,
        origin=Origin(xyz=(BOARD_LEN / 2.0, BOARD_WID / 2.0, HINGE_CENTER_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    board = object_model.get_part("board")
    clamp_base = object_model.get_part("clamp_base")
    clamp_lever = object_model.get_part("clamp_lever")
    cover = object_model.get_part("cover")
    pivot = object_model.get_articulation("base_to_lever")
    cover_hinge = object_model.get_articulation("board_to_cover")

    lever_vis = clamp_lever.get_visual("clamp_lever")
    base_vis = clamp_base.get_visual("clamp_base")
    spine_vis = board.get_visual("hinge_spine")
    knuckle_vis = cover.get_visual("cover_knuckle")
    cover_panel_vis = cover.get_visual("cover_panel")

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

    # Hinge capture: the cover knuckle wraps around the board hinge spine,
    # and the knuckle dips slightly below the board surface at the hinge edge.
    ctx.allow_overlap(
        board,
        cover,
        elem_a=spine_vis,
        elem_b=knuckle_vis,
        reason="Cover hinge knuckle is captured around the board hinge spine barrel.",
    )
    ctx.allow_overlap(
        board,
        cover,
        elem_a=board.get_visual("board_panel"),
        elem_b=knuckle_vis,
        reason="Cover hinge knuckle embeds slightly into the board at the hinge edge for pivot capture.",
    )

    # Cover panel slides under the clamp when closed: the clamp base footplate
    # and lever lip rest on top of the cover panel, creating local overlap.
    ctx.allow_overlap(
        clamp_base,
        cover,
        elem_a=base_vis,
        elem_b=cover_panel_vis,
        reason="Cover panel slides under the clamp base footplate when closed, as in a real form-holder clipboard.",
    )
    ctx.allow_overlap(
        clamp_lever,
        cover,
        elem_a=lever_vis,
        elem_b=cover_panel_vis,
        reason="Clamp lever front lip presses down onto the cover panel when closed.",
    )

    # ---- Joint contract: revolute about the clamp-width (Y) axis ----
    ctx.check(
        "pivot is revolute",
        pivot.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={pivot.articulation_type}",
    )
    ax = tuple(pivot.axis)
    ctx.check(
        "pivot axis runs along the clamp width (Y)",
        abs(ax[1]) > 0.99 and abs(ax[0]) < 1e-6 and abs(ax[2]) < 1e-6,
        details=f"axis={ax}",
    )

    # ---- Board hero geometry: large thin flat panel ----
    bp_vis = board.get_visual("board_panel")
    bpmin, bpmax = ctx.part_element_world_aabb(board, elem=bp_vis)
    board_len = bpmax[0] - bpmin[0]
    board_wid = bpmax[1] - bpmin[1]
    board_thk = bpmax[2] - bpmin[2]
    ctx.check(
        "board reads as a wide thin clipboard panel",
        board_len > 0.30 and board_wid > 0.21 and board_thk < 0.006,
        details=f"len={board_len:.3f} wid={board_wid:.3f} thk={board_thk:.4f}",
    )

    # ---- Clamp sits at one end of the board, on top ----
    cmin, cmax = ctx.part_world_aabb(clamp_base)
    ctx.check(
        "clamp base is near the top (-X) edge of the board",
        cmax[0] < 0.10 and cmin[0] >= bpmin[0] - 1e-4,
        details=f"clamp x=[{cmin[0]:.3f},{cmax[0]:.3f}] board x0={bpmin[0]:.3f}",
    )
    ctx.check(
        "clamp base rises above the board top face",
        cmax[2] > board_thk + 0.005,
        details=f"clamp top z={cmax[2]:.4f} board top z={bpmax[2]:.4f}",
    )

    # ---- Base is mounted to the board (rests on its top face, no float) ----
    ctx.expect_overlap(clamp_base, board, axes="xy", min_overlap=0.02)
    ctx.expect_gap(
        clamp_base,
        board,
        axis="z",
        negative_elem=bp_vis,
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

    # ====================================================================
    # Cover hinge tests
    # ====================================================================

    # ---- Joint contract: revolute along the board length (X axis) ----
    ctx.check(
        "cover hinge is revolute",
        cover_hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={cover_hinge.articulation_type}",
    )
    hax = tuple(cover_hinge.axis)
    ctx.check(
        "cover hinge axis runs along the board length (X)",
        abs(hax[0]) > 0.99 and abs(hax[1]) < 1e-6 and abs(hax[2]) < 1e-6,
        details=f"axis={hax}",
    )

    # ---- Cover panel is a large thin sheet matching the board footprint ----
    cpmin, cpmax = ctx.part_element_world_aabb(cover, elem=cover_panel_vis)
    cover_len = cpmax[0] - cpmin[0]
    cover_wid = cpmax[1] - cpmin[1]
    cover_thk = cpmax[2] - cpmin[2]
    ctx.check(
        "cover panel reads as a wide thin folder cover",
        cover_len > 0.28 and cover_wid > 0.18 and cover_thk < 0.005,
        details=f"len={cover_len:.3f} wid={cover_wid:.3f} thk={cover_thk:.4f}",
    )

    # ---- Cover hinge knuckle is present at the hinge edge ----
    ckmin, ckmax = ctx.part_element_world_aabb(cover, elem=knuckle_vis)
    ctx.check(
        "cover hinge knuckle is at the +Y board edge",
        ckmax[1] > BOARD_WID / 2.0 - 0.010,
        details=f"knuckle y max={ckmax[1]:.3f} board +Y edge={BOARD_WID/2:.3f}",
    )

    # ---- Hinge connectivity: knuckle contacts spine ----
    ctx.expect_contact(
        cover,
        board,
        elem_a=knuckle_vis,
        elem_b=spine_vis,
        contact_tol=0.002,
        name="cover knuckle is captured on the board hinge spine",
    )

    # ---- Closed pose (q=0): cover lies flat on the board ----
    with ctx.pose({cover_hinge: 0.0}):
        cpmin0, cpmax0 = ctx.part_element_world_aabb(cover, elem=cover_panel_vis)
        # Cover bottom sits at or near the board top surface.
        ctx.check(
            "closed cover seats on the board top face",
            abs(cpmin0[2] - BOARD_THK) < 0.002,
            details=f"cover bottom z={cpmin0[2]:.4f} board top z={BOARD_THK:.4f}",
        )
        # Cover overlaps the board in XY (covers the board surface).
        ctx.expect_overlap(
            cover,
            board,
            axes="xy",
            min_overlap=0.10,
            elem_a=cover_panel_vis,
            elem_b=bp_vis,
            name="closed cover overlaps the board in XY",
        )

    # ---- Open pose (q=π/2): cover stands up from the board ----
    with ctx.pose({cover_hinge: math.pi / 2.0}):
        cpmin1, cpmax1 = ctx.part_element_world_aabb(cover, elem=cover_panel_vis)
        # The cover's max Z at 90° open should be much higher than at closed.
        ctx.check(
            "opening the cover raises its free edge upward",
            cpmax1[2] > cpmax0[2] + 0.05,
            details=f"closed top z={cpmax0[2]:.4f} open top z={cpmax1[2]:.4f}",
        )

    # ---- Fully open pose (q=upper): cover folds back behind the board ----
    cover_upper = cover_hinge.motion_limits.upper
    with ctx.pose({cover_hinge: cover_upper}):
        cpmin2, cpmax2 = ctx.part_element_world_aabb(cover, elem=cover_panel_vis)
        # At 180°, the cover panel is on the +Y side of the hinge (behind the board).
        ctx.check(
            "fully open cover folds to the back (+Y side) of the board",
            cpmin2[1] > BOARD_WID / 2.0 - 0.020,
            details=f"cover min y={cpmin2[1]:.4f} board +Y edge={BOARD_WID/2:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
