from __future__ import annotations

# Realistic articulated clipboard with a sprung wire-bail clamp.
#
# Object identity (variant of picture/Stationary/Clipboard/001.png):
#   A letter/A4-size blue plastic clipboard with a low-profile chrome
#   wire-bail clamp riveted to the top edge. The clamp is a sprung wire-bail
#   mechanism: a fixed low metal anchor block pinned to the board carries two
#   pivot ears, and a single bent round-wire bail (U-shaped hairpin loop) pivots
#   on those ears. The bail cross-bar presses paper flat against the board at
#   rest; lifting the cross-bar swings the bail up and back to load paper.
#
# Layout: the board lies flat in the XY plane, top surface facing +Z. The clamp
# is mounted at the low-X (top) edge. The bail pivots about a Y axis. Positive
# joint motion lifts the cross-bar up and away from the board ("open" gesture).

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
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Real-world dimensions (meters)
# ---------------------------------------------------------------------------
BOARD_LEN = 0.330       # along +X (long dimension)
BOARD_WID = 0.232       # along Y  (short dimension)
BOARD_THK = 0.0032      # plastic panel thickness
BOARD_CORNER_R = 0.012

# Board occupies x in [0, BOARD_LEN], centered on Y, top face at z = BOARD_THK.

# ---------------------------------------------------------------------------
# Clamp location (same as parent)
# ---------------------------------------------------------------------------
CLAMP_CENTER_X = 0.034  # x of the pivot axis
PIVOT_Z = BOARD_THK + 0.009  # height of the pivot axis above ground

# ---------------------------------------------------------------------------
# Anchor block (low metal plate riveted to board, with pivot ears)
# ---------------------------------------------------------------------------
BLOCK_LEN = 0.022       # x-extent of the plate
BLOCK_HEIGHT = 0.003    # main body height above board top
EAR_THK = 0.004         # ear thickness along Y
EAR_RISE = 0.002        # ears extend this far above the pivot axis

# ---------------------------------------------------------------------------
# Wire bail (bent round wire, U-shaped hairpin)
# ---------------------------------------------------------------------------
WIRE_R = 0.0015         # wire radius (3 mm diameter stock)
BAIL_HW = 0.032         # bail half-width along Y
BAIL_REACH = 0.042      # cross-bar forward reach from pivot (local +x)

# Cross-bar z in joint-local frame. The wire bottom sits ~0.3 mm above the
# board top face so there is no geometric overlap with the board.
_CROSSBAR_Z = -(PIVOT_Z - BOARD_THK - WIRE_R - 0.0003)

MAT_BOARD = "clipboard_blue"
MAT_METAL = "chrome_clamp"


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def _board_solid() -> cq.Workplane:
    """Thin rounded-corner plastic panel, top face at z=BOARD_THK."""
    board = (
        cq.Workplane("XY")
        .box(BOARD_LEN, BOARD_WID, BOARD_THK, centered=(False, True, False))
        .edges("|Z")
        .fillet(BOARD_CORNER_R)
    )
    return board.translate((0.0, 0.0, BOARD_THK / 2.0))


def _rivet_head(x: float, y: float, z_base: float) -> cq.Workplane:
    """Small cylindrical rivet head for the anchor block fasteners."""
    return (
        cq.Workplane("XY")
        .circle(0.003)
        .extrude(0.0012)
        .translate((x, y, z_base))
    )


def _anchor_block_solid() -> cq.Workplane:
    """Fixed low metal anchor block with two pivot ears and rivet heads.

    Built in WORLD coordinates (board top at z=BOARD_THK). The main plate is a
    low rounded box. Two pivot ears rise from the plate edges to capture the
    bail wire at the pivot height. Two rivet heads are visible on the plate top.
    """
    cx = CLAMP_CENTER_X
    body_w = 2.0 * (BAIL_HW + 0.002)  # plate spans between the ears

    # Main plate body
    body = (
        cq.Workplane("XY")
        .box(BLOCK_LEN, body_w, BLOCK_HEIGHT, centered=(True, True, False))
        .translate((cx, 0.0, BOARD_THK))
        .edges("|Z")
        .fillet(0.003)
    )

    # Pivot ears: rectangular tabs that rise from the plate edges past the
    # pivot axis so the bail wire is captured between them.
    ear_h = PIVOT_Z - BOARD_THK + EAR_RISE
    for sign in (-1.0, 1.0):
        ear_cy = sign * (BAIL_HW + EAR_THK / 2.0)
        ear = (
            cq.Workplane("XY")
            .box(0.012, EAR_THK, ear_h, centered=(True, True, False))
            .translate((cx, ear_cy, BOARD_THK))
            .edges("|Z")
            .fillet(0.001)
        )
        body = body.union(ear)

    # Rivet heads on the plate top (visible fasteners through board).
    n_rivets = 2
    for i in range(n_rivets):
        dy = (-1.0 + 2.0 * i / (n_rivets - 1)) * 0.018
        rivet = _rivet_head(cx, dy, BOARD_THK + BLOCK_HEIGHT)
        body = body.union(rivet)

    return body


def _bail_path_joint_local() -> list[tuple[float, float, float]]:
    """Wire bail centreline path in the joint-local frame.

    Origin at the pivot axis. +x_local toward the board centre (front cross-bar
    direction), +z_local up. At q=0 the cross-bar sits just above the board top
    surface, pressing paper flat.

    The path traces: left ear → left arm forward and down → cross-bar → right
    arm back and up → right ear. The Catmull-Rom spline smooths the bends,
    matching real wire-bending radii.
    """
    hw = BAIL_HW
    r = BAIL_REACH
    cz = _CROSSBAR_Z

    return [
        (0.000, -hw,  0.000),       # left ear pivot
        (0.008, -hw, -0.001),       # left arm, slight forward-down
        (0.020, -hw, -0.003),       # left arm, continuing descent
        (0.034, -hw, -0.006),       # left arm, bending toward board
        (0.040, -hw,  cz + 0.001),  # left arm, transition to cross-bar
        (r,     -hw,  cz),          # left foot at cross-bar
        (r, -hw * 0.6, cz),         # cross-bar quarter
        (r,      0.0,  cz),         # cross-bar centre
        (r,  hw * 0.6, cz),         # cross-bar quarter
        (r,      hw,  cz),          # right foot at cross-bar
        (0.040,  hw,  cz + 0.001),  # right arm, transition from cross-bar
        (0.034,  hw, -0.006),       # right arm, bending toward board
        (0.020,  hw, -0.003),       # right arm, continuing descent
        (0.008,  hw, -0.001),       # right arm, slight forward-down
        (0.000,  hw,  0.000),       # right ear pivot
    ]


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clipboard_wire_bail")

    model.material(MAT_BOARD, rgba=(0.13, 0.45, 0.92, 1.0))
    model.material(MAT_METAL, rgba=(0.78, 0.80, 0.83, 1.0))

    # --- Board (root) ---
    board = model.part("board")
    board.visual(
        mesh_from_cadquery(_board_solid(), "board_panel"),
        material=MAT_BOARD,
        name="board_panel",
    )

    # --- Fixed anchor block (riveted to board) ---
    anchor = model.part("anchor_block")
    anchor.visual(
        mesh_from_cadquery(_anchor_block_solid(), "anchor_block"),
        material=MAT_METAL,
        name="anchor_block",
    )

    # --- Wire bail (revolute on anchor) ---
    # The bail mesh is authored in joint-local coordinates (origin at pivot
    # axis, local +x toward the front cross-bar). The revolute articulation
    # places the child part frame at the pivot in world space.
    bail = model.part("wire_bail")
    bail.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                _bail_path_joint_local(),
                radius=WIRE_R,
                samples_per_segment=16,
                radial_segments=16,
                cap_ends=True,
            ),
            "wire_bail",
        ),
        material=MAT_METAL,
        name="wire_bail",
    )

    # --- Board carries the fixed anchor block (rigid mount) ---
    model.articulation(
        "board_to_anchor",
        ArticulationType.FIXED,
        parent=board,
        child=anchor,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # --- Anchor carries the pivoting bail on the ear axis ---
    # axis=(0,-1,0): by the right-hand rule positive q rotates the front
    # cross-bar (at local +x) upward toward +Z, opening the bail.
    model.articulation(
        "anchor_to_bail",
        ArticulationType.REVOLUTE,
        parent=anchor,
        child=bail,
        origin=Origin(xyz=(CLAMP_CENTER_X, 0.0, PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=3.0, lower=0.0, upper=1.8,
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    board = object_model.get_part("board")
    anchor = object_model.get_part("anchor_block")
    bail = object_model.get_part("wire_bail")
    pivot = object_model.get_articulation("anchor_to_bail")

    bail_vis = bail.get_visual("wire_bail")
    anchor_vis = anchor.get_visual("anchor_block")

    # The bail wire at the pivot ears is intentionally captured by the ear
    # geometry (wire passes between the ears). Small local overlap at the
    # pivot capture is mechanically correct.
    ctx.allow_overlap(
        anchor,
        bail,
        elem_a=anchor_vis,
        elem_b=bail_vis,
        reason=(
            "Wire bail pivot ends pass between the anchor block pivot ears; "
            "small local overlap represents the pivot capture."
        ),
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
    bmin, bmax = ctx.part_world_aabb(board)
    board_len = bmax[0] - bmin[0]
    board_wid = bmax[1] - bmin[1]
    board_thk = bmax[2] - bmin[2]
    ctx.check(
        "board reads as a wide thin clipboard panel",
        board_len > 0.30 and board_wid > 0.21 and board_thk < 0.006,
        details=f"len={board_len:.3f} wid={board_wid:.3f} thk={board_thk:.4f}",
    )

    # ---- Anchor block at the top of the board ----
    amin, amax = ctx.part_world_aabb(anchor)
    ctx.check(
        "anchor block is near the top (-X) edge of the board",
        amax[0] < 0.10 and amin[0] >= bmin[0] - 1e-4,
        details=f"anchor x=[{amin[0]:.3f},{amax[0]:.3f}] board x0={bmin[0]:.3f}",
    )
    ctx.check(
        "anchor block is low (rises only slightly above board)",
        amax[2] < BOARD_THK + 0.016,
        details=f"anchor top z={amax[2]:.4f}",
    )

    # ---- Anchor is mounted to the board (rests on top face) ----
    ctx.expect_overlap(anchor, board, axes="xy", min_overlap=0.02)
    ctx.expect_gap(
        anchor, board, axis="z",
        max_gap=0.0005, max_penetration=0.002,
        name="anchor block seats on the board top face",
    )

    # ---- Bail is captured on the anchor ears (not floating) ----
    ctx.expect_contact(
        bail, anchor,
        elem_a=bail_vis, elem_b=anchor_vis,
        contact_tol=0.005,
        name="wire bail is captured on the anchor pivot ears",
    )

    # ---- Wire bail is a swept tube spanning the clamp width ----
    wmin, wmax = ctx.part_element_world_aabb(bail, elem=bail_vis)
    bail_dy = wmax[1] - wmin[1]
    bail_dz_rest = wmax[2] - wmin[2]
    ctx.check(
        "wire bail spans across the clamp width (swept tube, not a narrow box)",
        bail_dy > 0.050,
        details=f"bail Y span={bail_dy:.3f}",
    )
    ctx.check(
        "wire bail Z extent at rest is consistent with thin wire (not a thick block)",
        bail_dz_rest < 0.020,
        details=f"bail Z extent={bail_dz_rest:.3f}",
    )

    # ---- Closed (rest) pose: cross-bar presses near the board ----
    with ctx.pose({pivot: 0.0}):
        wmin0, wmax0 = ctx.part_world_aabb(bail)
        crossbar_z0 = wmin0[2]  # lowest bail point ≈ cross-bar wire bottom
        ctx.check(
            "closed bail cross-bar reaches down near the board top face",
            crossbar_z0 < BOARD_THK + 0.004,
            details=f"cross-bar z={crossbar_z0:.4f} board top={BOARD_THK:.4f}",
        )
        ctx.expect_overlap(
            bail, board, axes="xy", min_overlap=0.01,
            name="closed bail overlaps the board footprint (paper gripping area)",
        )

    # ---- Open pose: cross-bar lifts up and away from the board ----
    upper = pivot.motion_limits.upper
    with ctx.pose({pivot: upper}):
        wmin1, wmax1 = ctx.part_world_aabb(bail)
        ctx.check(
            "opening lifts the bail well above the board",
            wmax1[2] > BOARD_THK + 0.030,
            details=f"open bail top z={wmax1[2]:.4f}",
        )
        ctx.check(
            "opening raises the bail lowest point above the closed cross-bar",
            wmin1[2] > crossbar_z0 + 0.007,
            details=f"closed bottom z={crossbar_z0:.4f} open bottom z={wmin1[2]:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
