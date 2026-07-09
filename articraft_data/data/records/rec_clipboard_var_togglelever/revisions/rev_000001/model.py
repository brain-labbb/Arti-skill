from __future__ import annotations

# Clipboard with an over-center toggle cam lever clamp.
#
# Variant of the parent torsion-spring jaw clipboard: the spring jaw is replaced
# by a cam lever mechanism. A fixed metal bracket on the board top edge carries
# a pivot pin; a cam lever rotates on that pin. The eccentric cam lobe presses
# directly onto the paper when the lever swings flat (clamped), and releases
# when the lever stands up.
#
# Layout: board in XY plane, top surface +Z. Clamp at the -X (top) edge.
# Clamp width along Y. Lever pivots about a Y axis.
#
# Joint convention: axis = (0, +1, 0). At q=0 the handle points up (+Z);
# positive q swings the handle forward (toward +X / board center) and the
# cam lobe rotates to press its eccentric surface down onto the paper.
#
# The cam is a real circular profile (eccentric to the pivot) built with
# CadQuery. The max clamping occurs at θ≈0.93 rad where the cam surface
# just contacts the board. Past that (over-center), the lever continues
# to ~1.2 rad where the mechanism locks.

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

# ---------------------------------------------------------------------------
# Real-world dimensions (meters)
# ---------------------------------------------------------------------------
BOARD_LEN = 0.330       # along +X (long dimension)
BOARD_WID = 0.232       # along Y
BOARD_THK = 0.0032      # plastic panel thickness
BOARD_CORNER_R = 0.012

# Board occupies x in [0, BOARD_LEN], centered on Y, top face at z = BOARD_THK.

CLAMP_WID = 0.068       # clamp width along Y
CLAMP_CENTER_X = 0.034  # x of pivot axis
PIVOT_Z = BOARD_THK + 0.013  # pivot pin height: 0.0162

# Cam parameters (joint-local frame, origin at pivot pin axis)
# +x_local toward board center, +z_local up.
# The eccentricity is chosen so the cam bottom touches the board at θ≈0.93 rad
# (max clamping). Past that the over-center action locks.
CAM_ECC_X = 0.004       # cam center offset in local +x (forward)
CAM_ECC_Z = -0.003      # cam center offset in local -z (below pivot)
CAM_RADIUS = 0.008       # cam lobe radius (real circular profile)
HUB_RADIUS = 0.0055      # hub around pivot pin
PIVOT_PIN_R = 0.0020

# Handle
HANDLE_LEN = 0.040       # from pivot to tip
HANDLE_WID = 0.014
HANDLE_THK = 0.003       # thickness along x

# Bracket
BRACKET_X0 = 0.016
BRACKET_X1 = 0.052
BRACKET_PLATE_THK = 0.0010
CHEEK_THK = 0.003
CHEEK_TOP_ABOVE_PIVOT = 0.003  # cheeks extend past pivot

# Lever half-width (slightly narrower than bracket gap)
LEVER_HALF_W = CLAMP_WID / 2.0 - 0.008  # = 0.026

# Materials
MAT_BOARD = "clipboard_blue"
MAT_METAL = "chrome_bracket"
MAT_CAM = "zinc_cam"
MAT_GRIP = "rubber_grip"

# ---------------------------------------------------------------------------
# Geometry builders
# ---------------------------------------------------------------------------

def _board_solid() -> cq.Workplane:
    """Thin rounded-corner plastic panel, bottom at z=0, top at z=BOARD_THK."""
    board = (
        cq.Workplane("XY")
        .box(BOARD_LEN, BOARD_WID, BOARD_THK, centered=(False, True, True))
        .edges("|Z")
        .fillet(BOARD_CORNER_R)
    )
    # Centered in Z: box spans [-THK/2, +THK/2]. Lift so bottom=0, top=THK.
    return board.translate((0.0, 0.0, BOARD_THK / 2.0))


def _bracket_solid() -> cq.Workplane:
    """Fixed mounting bracket in world coordinates.

    Two side mounting pads (with a central slot for the cam to pass through),
    two upstanding cheeks, a pivot pin, and rivet heads.
    """
    base_len = BRACKET_X1 - BRACKET_X0
    base_cx = (BRACKET_X0 + BRACKET_X1) / 2.0
    half_clamp_w = CLAMP_WID / 2.0
    # Slot half-width: just clears the lever body
    slot_half_w = LEVER_HALF_W + 0.002

    # --- Two side mounting pads (plate with central slot) ---
    full_plate = (
        cq.Workplane("XY")
        .workplane(offset=BOARD_THK + BRACKET_PLATE_THK / 2.0)
        .box(base_len, CLAMP_WID, BRACKET_PLATE_THK)
        .translate((base_cx, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.004)
    )
    # Central slot cutout for cam clearance
    slot = (
        cq.Workplane("XY")
        .workplane(offset=BOARD_THK - 0.0005)
        .box(base_len + 0.002, 2.0 * slot_half_w, BRACKET_PLATE_THK + 0.001)
        .translate((base_cx, 0.0, 0.0))
    )
    plate = full_plate.cut(slot)

    # --- Two side cheeks ---
    cheek_y = half_clamp_w - CHEEK_THK / 2.0
    cheek_h = PIVOT_Z - BOARD_THK + CHEEK_TOP_ABOVE_PIVOT
    cheeks = cq.Workplane("XY")
    for sign in (-1.0, 1.0):
        cheek = (
            cq.Workplane("XY")
            .workplane(offset=BOARD_THK)
            .box(0.020, CHEEK_THK, cheek_h, centered=(True, True, False))
            .translate((CLAMP_CENTER_X, sign * cheek_y, 0.0))
            .edges("|Y and >Z")
            .fillet(0.002)
        )
        cheeks = cheeks.union(cheek)

    # --- Pivot pin between cheeks ---
    pin_span = CLAMP_WID - 2.0 * CHEEK_THK
    pin = (
        cq.Workplane("XZ")
        .workplane(offset=-pin_span / 2.0)
        .center(CLAMP_CENTER_X, PIVOT_Z)
        .circle(PIVOT_PIN_R)
        .extrude(pin_span)
    )

    # --- Rivet heads on each pad (loop over 4 rivets) ---
    rivets = cq.Workplane("XY")
    for i in range(4):
        sign = -1.0 if i < 2 else 1.0
        dx = -0.006 if i % 2 == 0 else 0.008
        rivet = (
            cq.Workplane("XY")
            .workplane(offset=BOARD_THK + BRACKET_PLATE_THK)
            .center(CLAMP_CENTER_X + dx, sign * (slot_half_w + 0.003))
            .circle(0.0025)
            .extrude(0.0008)
        )
        rivets = rivets.union(rivet)

    return plate.union(cheeks).union(pin).union(rivets)


def _cam_lever_solid() -> cq.Workplane:
    """Cam lever in joint-local frame (origin at pivot pin axis).

    +x_local: toward board center (front)
    +z_local: up

    Contains: hub (around pivot), eccentric cam lobe (the pressure foot),
    and handle bar. The cam lobe is a real circular profile built with
    CadQuery -- not a box placeholder.
    """
    hw = LEVER_HALF_W

    # --- Hub: cylinder around pivot pin ---
    hub = (
        cq.Workplane("XZ")
        .workplane(offset=-hw)
        .center(0.0, 0.0)
        .circle(HUB_RADIUS)
        .extrude(2.0 * hw)
    )

    # --- Eccentric cam lobe: real circular cam profile ---
    # The cam center is offset from the pivot, creating the clamping action.
    # This eccentric disk IS the pressure foot: its surface presses on the
    # paper when the lever is swung to the clamped position.
    cam = (
        cq.Workplane("XZ")
        .workplane(offset=-hw)
        .center(CAM_ECC_X, CAM_ECC_Z)
        .circle(CAM_RADIUS)
        .extrude(2.0 * hw)
    )

    # --- Handle: flat bar extending upward from the hub ---
    # Tapers slightly toward the tip for a realistic lever shape.
    handle_profile = (
        cq.Workplane("XZ")
        .moveTo(-HANDLE_THK / 2.0, HUB_RADIUS * 0.4)
        .lineTo(HANDLE_THK / 2.0, HUB_RADIUS * 0.4)
        .lineTo(HANDLE_THK / 2.0 * 0.7, HANDLE_LEN)
        .lineTo(-HANDLE_THK / 2.0 * 0.7, HANDLE_LEN)
        .close()
    )
    handle = handle_profile.extrude(HANDLE_WID, both=True)

    # Round the handle tip
    handle = handle.edges(">Z").fillet(0.002)

    lever = hub.union(cam).union(handle)
    return lever


def _handle_grip_solid() -> cq.Workplane:
    """Rubber grip sleeve on the handle tip, in joint-local frame.

    Wraps around the handle upper section for finger contact.
    """
    grip_len = 0.020
    grip_start = HANDLE_LEN - grip_len - 0.001
    grip_w = HANDLE_WID + 0.004
    grip_thk = HANDLE_THK + 0.004
    grip = (
        cq.Workplane("XY")
        .box(grip_thk, grip_w, grip_len, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.002)
        .translate((0.0, 0.0, grip_start))
    )
    return grip


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clipboard_cam_lever_clamp")

    model.material(MAT_BOARD, rgba=(0.13, 0.45, 0.92, 1.0))
    model.material(MAT_METAL, rgba=(0.75, 0.77, 0.80, 1.0))
    model.material(MAT_CAM, rgba=(0.65, 0.67, 0.70, 1.0))
    model.material(MAT_GRIP, rgba=(0.12, 0.12, 0.13, 1.0))

    # --- Board (root) ---
    board = model.part("board")
    board.visual(
        mesh_from_cadquery(_board_solid(), "board_panel"),
        material=MAT_BOARD,
        name="board_panel",
    )

    # --- Fixed bracket (riveted to board) ---
    bracket = model.part("clamp_bracket")
    bracket.visual(
        mesh_from_cadquery(_bracket_solid(), "bracket_body"),
        material=MAT_METAL,
        name="bracket_body",
    )

    # --- Cam lever (pivots on bracket) ---
    cam_lever = model.part("cam_lever")
    cam_lever.visual(
        mesh_from_cadquery(_cam_lever_solid(), "cam_body"),
        material=MAT_CAM,
        name="cam_body",
    )
    cam_lever.visual(
        mesh_from_cadquery(_handle_grip_solid(), "handle_grip"),
        material=MAT_GRIP,
        name="handle_grip",
    )

    # --- Board carries the fixed bracket (rigid mount) ---
    model.articulation(
        "board_to_bracket",
        ArticulationType.FIXED,
        parent=board,
        child=bracket,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # --- Bracket carries the cam lever on the pivot pin ---
    # The lever mesh is in joint-local coords (origin at pivot, +x toward
    # board center, +z up). With axis=(0, +1, 0), positive q rotates
    # the handle from up (+z) toward forward (+x), and the eccentric cam
    # lobe rotates to press its surface down onto the paper.
    model.articulation(
        "bracket_to_lever",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=cam_lever,
        origin=Origin(xyz=(CLAMP_CENTER_X, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0,
            lower=0.0,   # released: handle up, cam retracted
            upper=1.20,  # clamped: handle forward, cam pressing on paper
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    board = object_model.get_part("board")
    bracket = object_model.get_part("clamp_bracket")
    cam_lever = object_model.get_part("cam_lever")
    pivot = object_model.get_articulation("bracket_to_lever")

    cam_vis = cam_lever.get_visual("cam_body")
    bracket_vis = bracket.get_visual("bracket_body")

    # The cam hub wraps around the pivot pin. Allow the local mechanical
    # capture overlap at the pivot interface.
    ctx.allow_overlap(
        bracket,
        cam_lever,
        elem_a=bracket_vis,
        elem_b=cam_vis,
        reason="Cam lever hub captures the bracket pivot pin; the hub bore wraps the pin for pivoting.",
    )

    # ---- Joint contract: revolute about Y axis ----
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

    # ---- Board: large thin flat panel ----
    bmin, bmax = ctx.part_world_aabb(board)
    board_len = bmax[0] - bmin[0]
    board_wid = bmax[1] - bmin[1]
    board_thk = bmax[2] - bmin[2]
    ctx.check(
        "board reads as a wide thin clipboard panel",
        board_len > 0.30 and board_wid > 0.21 and board_thk < 0.006,
        details=f"len={board_len:.3f} wid={board_wid:.3f} thk={board_thk:.4f}",
    )

    # ---- Bracket sits at the top edge of the board ----
    br_min, br_max = ctx.part_world_aabb(bracket)
    ctx.check(
        "bracket is near the top (-X) edge of the board",
        br_max[0] < 0.10 and br_min[0] >= bmin[0] - 1e-4,
        details=f"bracket x=[{br_min[0]:.3f},{br_max[0]:.3f}] board x0={bmin[0]:.3f}",
    )
    ctx.check(
        "bracket rises above the board top face",
        br_max[2] > BOARD_THK + 0.005,
        details=f"bracket top z={br_max[2]:.4f} board top z={bmax[2]:.4f}",
    )

    # ---- Bracket is mounted to the board ----
    ctx.expect_overlap(bracket, board, axes="xy", min_overlap=0.01)
    ctx.expect_gap(
        bracket, board,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0020,
        name="bracket seats on the board top face",
    )

    # ---- Cam lever is captured on the bracket pivot ----
    ctx.expect_contact(
        cam_lever, bracket,
        elem_a=cam_vis, elem_b=bracket_vis,
        contact_tol=0.002,
        name="cam lever hub contacts the bracket pivot pin",
    )

    # ---- Handle grip present on the upper handle ----
    grip = cam_lever.get_visual("handle_grip")
    grip_min0, grip_max0 = ctx.part_element_world_aabb(cam_lever, elem=grip)
    ctx.check(
        "handle grip is on the upper portion of the lever",
        grip_max0[2] > PIVOT_Z + 0.015,
        details=f"grip top z={grip_max0[2]:.4f} pivot z={PIVOT_Z:.4f}",
    )

    # ---- Released pose (q=0): handle up, cam retracted, paper free ----
    with ctx.pose({pivot: 0.0}):
        lev_min0, lev_max0 = ctx.part_world_aabb(cam_lever)
        # Handle should be pointing up: lever top well above the pivot
        ctx.check(
            "released: handle stands up above the pivot",
            lev_max0[2] > PIVOT_Z + 0.020,
            details=f"lever top z={lev_max0[2]:.4f} pivot z={PIVOT_Z:.4f}",
        )
        # Cam bottom should clear the board for paper insertion
        ctx.check(
            "released: cam lobe clears the board for paper insertion",
            lev_min0[2] > BOARD_THK + 0.001,
            details=f"lever bottom z={lev_min0[2]:.4f} board top={BOARD_THK:.4f}",
        )

    # ---- Clamped pose (q=upper): handle forward, cam pressing on paper ----
    upper = pivot.motion_limits.upper
    with ctx.pose({pivot: upper}):
        lev_min1, lev_max1 = ctx.part_world_aabb(cam_lever)
        # Handle has swung forward: lever top is lower than released
        ctx.check(
            "clamped: handle swung forward (lower than released)",
            lev_max1[2] < lev_max0[2] - 0.010,
            details=f"released top z={lev_max0[2]:.4f} clamped top z={lev_max1[2]:.4f}",
        )
        # Handle extends toward board center (positive X shift)
        ctx.check(
            "clamped: handle extends toward the board center",
            lev_max1[0] > CLAMP_CENTER_X + 0.025,
            details=f"lever max x={lev_max1[0]:.4f}",
        )
        # Cam surface reaches near the board (pressing on paper)
        ctx.check(
            "clamped: cam lobe presses near the board surface",
            lev_min1[2] < BOARD_THK + 0.002,
            details=f"lever bottom z={lev_min1[2]:.4f} board top={BOARD_THK:.4f}",
        )
        # Cam overlaps the board gripping area in XY
        ctx.expect_overlap(
            cam_lever, board,
            axes="xy",
            min_overlap=0.01,
            name="clamped: cam lever overlaps the board paper-gripping zone",
        )

    return ctx.report()


object_model = build_object_model()
