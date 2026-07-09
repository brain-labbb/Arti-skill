from __future__ import annotations

# Realistic articulated clipboard with TWO metal spring clamps side by side.
#
# Object identity (from picture/Stationary/Clipboard/001.png, variant):
#   A wider letter/A4-size blue plastic clipboard with two identical chrome
#   spring clamps mounted side by side along the top edge. Each clamp is the
#   classic torsion-spring jaw mechanism: a fixed metal base bracket pinned to
#   the board carries a pivot barrel, and a curved metal lever (with two dark
#   plastic finger caps at the rear and a rolled gripping lip at the front)
#   rotates on that barrel. A hidden torsion spring keeps the front lip
#   pressed against the board; pressing the rear finger pads lifts the front
#   jaw to insert paper.
#
# Layout: the board lies flat in the XY plane, top surface facing +Z. The two
# clamps are mounted at the -X (top) edge, evenly spaced along Y. Each clamp
# width runs along Y, so each lever pivots about a Y axis. Positive joint
# motion lifts the front gripping lip up and away from the board.

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
BOARD_WID = 0.350  # along Y (wider to fit two clamps)
BOARD_THK = 0.0032  # plastic panel thickness
BOARD_CORNER_R = 0.012

# Board occupies x in [0, BOARD_LEN], centered on Y, top face at z = BOARD_THK.
# The clamps live near the top edge (small x).

CLAMP_WID = 0.078  # clamp width along Y
CLAMP_CENTER_X = 0.034  # x of each pivot barrel center
PIVOT_Z = BOARD_THK + 0.0090  # height of the pivot axis above board top face
PIVOT_RADIUS = 0.0030  # pin/barrel radius

# Two clamp Y positions: evenly spaced across the board width (1/3 and 2/3).
NUM_CLAMPS = 2
CLAMP_Y_POSITIONS = [
    -BOARD_WID / 6.0,  # clamp 0
    +BOARD_WID / 6.0,  # clamp 1
]

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
# Geometry helpers (parameterized by y_center for regular placement)
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


def _clamp_base_solid(y_center: float = 0.0) -> cq.Workplane:
    """Fixed metal base bracket at a given Y center.

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
        .translate((base_cx, y_center, 0.0))
        .edges("|Z")
        .fillet(0.004)
    )

    # Rear hump that lifts up to carry the pivot barrel.
    hump_len = 0.020
    hump_cx = CLAMP_CENTER_X
    hump = (
        cq.Workplane("XY")
        .workplane(offset=BOARD_THK)
        .box(hump_len, CLAMP_WID, BASE_TOP_Z - BOARD_THK, centered=(True, True, False))
        .translate((hump_cx, y_center, 0.0))
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
            .translate((CLAMP_CENTER_X, y_center + sign * cheek_y, 0.0))
        )
        cheeks = cheeks.union(cheek)

    # Pivot barrel along Y between the cheeks.
    barrel = (
        cq.Workplane("XZ")
        .workplane(offset=-(y_center + CLAMP_WID / 2.0))
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
            .center(BASE_X1 - 0.008, y_center + sign * 0.020)
            .circle(0.0035)
            .extrude(0.0012)
        )
        rivets = rivets.union(rivet)

    base = plate.union(hump).union(cheeks).union(barrel).union(rivets)
    return base


def _clamp_lever_solid(y_center: float = 0.0) -> cq.Workplane:
    """Moving spring-clamp jaw at a given Y center, authored in a frame where
    the pivot axis is at (CLAMP_CENTER_X, y_center, PIVOT_Z) in world.

    Contains: the curved sheet-metal cover, the front rolled gripping lip that
    presses on the board, the rear finger-pad shelf, and the pin sleeve through
    the barrel.

    NOTE: The lever mesh is authored so its pivot axis coincides with the
    articulation origin. We build in local joint frame then translate to world.
    """
    half_w = CLAMP_WID / 2.0 - 0.0050  # lever slightly narrower than base cheeks

    # Curved sheet-metal cover: a thin arched shell over the base hump.
    cover = (
        cq.Workplane("XZ")
        .moveTo(LEVER_BACK_X, 0.0026)
        .lineTo(-0.006, 0.0040)
        .lineTo(0.010, 0.0028)
        .lineTo(LEVER_FRONT_X, 0.0010)
        .lineTo(LEVER_FRONT_X, 0.0024)
        .lineTo(0.008, LEVER_CROWN_Z - 0.0008)
        .lineTo(-0.008, LEVER_CROWN_Z)
        .lineTo(LEVER_BACK_X, 0.0050)
        .close()
        .extrude(half_w, both=True)
    )

    # Front gripping lip: a downward rolled metal edge that contacts the board.
    lip_top = 0.0024
    lip_bottom = -(PIVOT_Z - BOARD_THK - 0.0006)
    lip_h = lip_top - lip_bottom
    lip = (
        cq.Workplane("XY")
        .box(0.0060, 2.0 * half_w, lip_h, centered=(True, True, False))
        .edges("|Y and <Z")
        .fillet(0.0010)
        .translate((LEVER_FRONT_X - 0.0030, 0.0, lip_bottom))
    )

    # Rear finger shelf: flat metal tab where the two plastic caps sit.
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
    # Authored in joint-local frame (origin at pivot axis). The articulation
    # origin places this at the correct world position; do NOT translate here.
    return lever


def _finger_caps_solid(y_center: float = 0.0) -> cq.Workplane:
    """Two dark plastic finger pads on the rear shelf, at a given Y center.

    Built in joint-local frame then translated to world like the lever.
    """
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
    # Authored in joint-local frame (origin at pivot axis), same as the lever.
    # The articulation origin places this at the correct world position.
    return caps


# ----------------------------------------------------------------------------
# Model
# ----------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clipboard_dual_clamp")

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

    # --- Two clamp assemblies, evenly spaced along Y ---
    for i in range(NUM_CLAMPS):
        yc = CLAMP_Y_POSITIONS[i]

        # Fixed clamp base bracket (riveted to board)
        base_part = model.part(f"clamp_base_{i}")
        base_part.visual(
            mesh_from_cadquery(_clamp_base_solid(yc), f"clamp_base_{i}"),
            material=MAT_METAL,
            name=f"clamp_base_{i}",
        )

        # Moving spring-clamp lever (jaw)
        lever_part = model.part(f"clamp_lever_{i}")
        lever_part.visual(
            mesh_from_cadquery(_clamp_lever_solid(yc), f"clamp_lever_{i}"),
            material=MAT_METAL,
            name=f"clamp_lever_{i}",
        )
        lever_part.visual(
            mesh_from_cadquery(_finger_caps_solid(yc), f"finger_caps_{i}"),
            material=MAT_CAP,
            name=f"finger_caps_{i}",
        )

        # Board carries the fixed clamp base (rigid mount)
        model.articulation(
            f"board_to_clamp_base_{i}",
            ArticulationType.FIXED,
            parent=board,
            child=base_part,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
        )

        # Clamp base carries the pivoting lever on the barrel.
        # The lever mesh is authored so its pivot axis is at world
        # (CLAMP_CENTER_X, yc, PIVOT_Z). The joint origin matches that.
        # Axis is -Y so positive q rotates the front lip UP toward +Z.
        model.articulation(
            f"base_{i}_to_lever_{i}",
            ArticulationType.REVOLUTE,
            parent=base_part,
            child=lever_part,
            origin=Origin(xyz=(CLAMP_CENTER_X, yc, PIVOT_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=0.42),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    board = object_model.get_part("board")

    # Board hero geometry: large thin flat panel
    bmin, bmax = ctx.part_world_aabb(board)
    board_len = bmax[0] - bmin[0]
    board_wid = bmax[1] - bmin[1]
    board_thk = bmax[2] - bmin[2]
    ctx.check(
        "board reads as a wide thin clipboard panel",
        board_len > 0.30 and board_wid > 0.30 and board_thk < 0.006,
        details=f"len={board_len:.3f} wid={board_wid:.3f} thk={board_thk:.4f}",
    )

    # Collect parts and articulations per clamp
    clamp_bases = []
    clamp_levers = []
    pivots = []
    for i in range(NUM_CLAMPS):
        base = object_model.get_part(f"clamp_base_{i}")
        lever = object_model.get_part(f"clamp_lever_{i}")
        pivot = object_model.get_articulation(f"base_{i}_to_lever_{i}")
        clamp_bases.append(base)
        clamp_levers.append(lever)
        pivots.append(pivot)

    # ---- Allow intentional overlap for each clamp's barrel capture ----
    for i in range(NUM_CLAMPS):
        base_vis = clamp_bases[i].get_visual(f"clamp_base_{i}")
        lever_vis = clamp_levers[i].get_visual(f"clamp_lever_{i}")
        ctx.allow_overlap(
            clamp_bases[i],
            clamp_levers[i],
            elem_a=base_vis,
            elem_b=lever_vis,
            reason=f"Lever {i} pin sleeve is captured around the fixed pivot barrel; the curved cover nests over the base at the pivot.",
        )

    # ---- Both pivots are revolute about the Y axis ----
    for i, pivot in enumerate(pivots):
        ctx.check(
            f"pivot_{i} is revolute",
            pivot.articulation_type == ArticulationType.REVOLUTE,
            details=f"type={pivot.articulation_type}",
        )
        ax = tuple(pivot.axis)
        ctx.check(
            f"pivot_{i} axis runs along the clamp width (Y)",
            abs(ax[1]) > 0.99 and abs(ax[0]) < 1e-6 and abs(ax[2]) < 1e-6,
            details=f"axis={ax}",
        )

    # ---- Both clamps sit at the top edge, on top of the board ----
    for i in range(NUM_CLAMPS):
        cmin, cmax = ctx.part_world_aabb(clamp_bases[i])
        ctx.check(
            f"clamp_base_{i} is near the top (-X) edge of the board",
            cmax[0] < 0.10 and cmin[0] >= bmin[0] - 1e-4,
            details=f"clamp x=[{cmin[0]:.3f},{cmax[0]:.3f}] board x0={bmin[0]:.3f}",
        )
        ctx.check(
            f"clamp_base_{i} rises above the board top face",
            cmax[2] > board_thk + 0.005,
            details=f"clamp top z={cmax[2]:.4f} board top z={bmax[2]:.4f}",
        )

        # Base is mounted to the board (rests on its top face, no float)
        ctx.expect_overlap(clamp_bases[i], board, axes="xy", min_overlap=0.02)
        ctx.expect_gap(
            clamp_bases[i],
            board,
            axis="z",
            max_gap=0.0005,
            max_penetration=0.0020,
            name=f"clamp_base_{i} seats on the board top face",
        )

    # ---- Regular Y spacing: the two clamps are separated along Y ----
    c0_min, c0_max = ctx.part_world_aabb(clamp_bases[0])
    c1_min, c1_max = ctx.part_world_aabb(clamp_bases[1])
    c0_yc = (c0_min[1] + c0_max[1]) / 2.0
    c1_yc = (c1_min[1] + c1_max[1]) / 2.0
    y_sep = abs(c1_yc - c0_yc)
    ctx.check(
        "two clamps are regularly spaced along Y",
        y_sep > 0.06,
        details=f"y_separation={y_sep:.4f}",
    )
    # Both at same X extent
    c0_xc = (c0_min[0] + c0_max[0]) / 2.0
    c1_xc = (c1_min[0] + c1_max[0]) / 2.0
    ctx.check(
        "both clamp bases share the same X position",
        abs(c1_xc - c0_xc) < 0.002,
        details=f"x0={c0_xc:.4f} x1={c1_xc:.4f}",
    )

    # ---- Each lever is captured on its base pivot (not floating) ----
    for i in range(NUM_CLAMPS):
        lever_vis = clamp_levers[i].get_visual(f"clamp_lever_{i}")
        base_vis = clamp_bases[i].get_visual(f"clamp_base_{i}")
        ctx.expect_contact(
            clamp_levers[i],
            clamp_bases[i],
            elem_a=lever_vis,
            elem_b=base_vis,
            contact_tol=0.001,
            name=f"lever_{i} sleeve is captured on the base_{i} pivot barrel",
        )

    # ---- Finger caps present on each lever ----
    for i in range(NUM_CLAMPS):
        caps = clamp_levers[i].get_visual(f"finger_caps_{i}")
        cmin, cmax = ctx.part_world_aabb(clamp_bases[i])
        pmin, pmax = ctx.part_element_world_aabb(clamp_levers[i], elem=caps)
        ctx.check(
            f"finger_caps_{i} sit on the rear (toward -X) of lever_{i}",
            pmax[0] < cmax[0],
            details=f"caps x max={pmax[0]:.3f} clamp x max={cmax[0]:.3f}",
        )

    # ---- Closed (rest) pose: front lips press near the board, jaws shut ----
    with ctx.pose({pivots[0]: 0.0, pivots[1]: 0.0}):
        for i in range(NUM_CLAMPS):
            lev_min0, lev_max0 = ctx.part_world_aabb(clamp_levers[i])
            front_lip_z0 = lev_min0[2]
            ctx.check(
                f"closed jaw_{i} front lip reaches down near the board top face",
                front_lip_z0 < board_thk + 0.004,
                details=f"front lip z={front_lip_z0:.4f} board top={board_thk:.4f}",
            )
            ctx.expect_overlap(
                clamp_levers[i],
                board,
                axes="xy",
                min_overlap=0.01,
                name=f"closed jaw_{i} overlaps the board (grips paper area)",
            )

    # ---- Open pose: pressing rear pads down lifts the front lip up ----
    for i in range(NUM_CLAMPS):
        caps = clamp_levers[i].get_visual(f"finger_caps_{i}")
        upper = pivots[i].motion_limits.upper

        # Closed pose measurements
        with ctx.pose({pivots[i]: 0.0}):
            cap_min0, cap_max0 = ctx.part_element_world_aabb(clamp_levers[i], elem=caps)
            cap_zc0 = (cap_min0[2] + cap_max0[2]) / 2.0
            lev_min0, lev_max0 = ctx.part_world_aabb(clamp_levers[i])
            lev_max0_top = lev_max0[2]

        # Open pose measurements
        with ctx.pose({pivots[i]: upper}):
            cap_min1, cap_max1 = ctx.part_element_world_aabb(clamp_levers[i], elem=caps)
            cap_zc1 = (cap_min1[2] + cap_max1[2]) / 2.0
            lev_min1, lev_max1 = ctx.part_world_aabb(clamp_levers[i])

            ctx.check(
                f"opening lever_{i} presses the rear finger pads down",
                cap_zc1 < cap_zc0 - 0.004,
                details=f"closed cap z={cap_zc0:.4f} open cap z={cap_zc1:.4f}",
            )
            ctx.check(
                f"opening lever_{i} lifts the front jaw up off the board",
                lev_max1[2] > lev_max0_top + 0.005,
                details=f"closed front top z={lev_max0_top:.4f} open front top z={lev_max1[2]:.4f}",
            )

    # ---- Each lever opens independently (one open, other stays closed) ----
    upper0 = pivots[0].motion_limits.upper
    # Measure lever_0 closed
    with ctx.pose({pivots[0]: 0.0, pivots[1]: 0.0}):
        _, lev_max_0closed = ctx.part_world_aabb(clamp_levers[0])
    # Measure lever_0 open while lever_1 stays closed
    with ctx.pose({pivots[0]: upper0, pivots[1]: 0.0}):
        _, lev_max_0open = ctx.part_world_aabb(clamp_levers[0])
    ctx.check(
        "lever_0 opens independently while lever_1 stays closed",
        lev_max_0open[2] > lev_max_0closed[2] + 0.003,
        details=f"open top z={lev_max_0open[2]:.4f} closed top z={lev_max_0closed[2]:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
