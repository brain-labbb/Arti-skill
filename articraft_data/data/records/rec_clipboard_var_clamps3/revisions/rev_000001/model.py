from __future__ import annotations

# Realistic articulated clipboard with three metal spring clamps.
#
# Variant of the single-clamp clipboard: three identical torsion-spring jaw
# clamps mounted in an evenly spaced row along the top edge of a wider board.
# Each clamp is its own assembly (fixed base bracket + revolute spring lever).
# The mechanism is identical to the parent; only the count and regular
# placement change.
#
# Layout: the board lies flat in the XY plane, top surface facing +Z. The
# clamps are mounted at the -X (top) edge in a row along Y. Each lever pivots
# about a Y axis; positive joint motion lifts the front gripping lip up and
# away from the board ("open" gesture).

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
BOARD_WID = 0.310  # along Y (wider for 3 clamps)
BOARD_THK = 0.0032  # plastic panel thickness
BOARD_CORNER_R = 0.012

# Board occupies x in [0, BOARD_LEN], centered on Y, top face at z = BOARD_THK.
# The clamps live near the top edge (small x).

CLAMP_COUNT = 3
CLAMP_WID = 0.078  # each clamp width along Y
CLAMP_SPACING = 0.090  # center-to-center Y distance between adjacent clamps
CLAMP_CENTER_X = 0.034  # x of each pivot barrel center
PIVOT_Z = BOARD_THK + 0.0090  # height of pivot axis above board top face
PIVOT_RADIUS = 0.0030  # pin/barrel radius

# Base bracket footprint (the riveted-down stationary metal part).
BASE_X0 = 0.012
BASE_X1 = 0.058
# Crown of the fixed base hump. Kept BELOW the pivot/lever underside so the
# moving cover arches over it without colliding; only the barrel reaches up to
# the pivot to be captured by the lever sleeve.
BASE_TOP_Z = BOARD_THK + 0.0058

# Lever (moving jaw) geometry, authored in the JOINT frame so the mesh frame
# and articulation frame coincide. The joint frame is placed at the pivot
# barrel. In the joint-local frame: +x_local points toward the board's front
# gripping lip (toward +X world at q=0), +z_local is up.
LEVER_FRONT_X = 0.026  # front lip extends this far ahead of pivot (local +x)
LEVER_BACK_X = -0.024  # finger-pad tail extends this far behind pivot (local -x)
LEVER_CROWN_Z = 0.0080  # height of the curved metal cover above the pivot

MAT_BOARD = "clipboard_blue"
MAT_METAL = "chrome_clamp"
MAT_CAP = "clamp_cap_black"

# Evenly spaced Y positions for the clamp row, centered on Y=0.
_CLAMP_Y_POSITIONS = [
    (i - (CLAMP_COUNT - 1) / 2.0) * CLAMP_SPACING
    for i in range(CLAMP_COUNT)
]


# ----------------------------------------------------------------------------
# Geometry helpers
# ----------------------------------------------------------------------------
def _board_solid() -> cq.Workplane:
    """Thin rounded-corner plastic panel, top face at z=BOARD_THK."""
    board = (
        cq.Workplane("XY")
        .box(BOARD_LEN, BOARD_WID, BOARD_THK, centered=(False, True, False))
        .edges("|Z")
        .fillet(BOARD_CORNER_R)
    )
    return board.translate((0.0, 0.0, BOARD_THK / 2.0))


def _clamp_base_solid(y_offset: float = 0.0) -> cq.Workplane:
    """Fixed metal base bracket at a given Y offset (world coords).

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

    # Rear hump that lifts up to carry the pivot barrel.
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

    # Two side cheeks that hold the pivot barrel ends.
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

    # Two rivet heads on the footplate front.
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
    if abs(y_offset) > 1e-9:
        base = base.translate((0.0, y_offset, 0.0))
    return base


def _clamp_lever_solid() -> cq.Workplane:
    """Moving spring-clamp jaw, authored in the JOINT-LOCAL frame.

    Origin is the pivot axis. +x_local -> board's front lip; +z_local -> up.
    Contains: the curved sheet-metal cover, the front rolled gripping lip that
    presses on the board, the rear finger-pad shelf, and the pin sleeve through
    the barrel.
    """
    half_w = CLAMP_WID / 2.0 - 0.0050  # lever slightly narrower than base cheeks

    # Curved sheet-metal cover: thin arched shell over the base hump.
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

    # Front gripping lip: downward rolled metal edge that contacts the board.
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

    return cover.union(lip).union(shelf).union(sleeve)


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
    model = ArticulatedObject(name="clipboard_triple_clamp")

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

    # Shared lever/cap geometry (joint-local, identical for all 3 clamps)
    lever_solid = _clamp_lever_solid()
    caps_solid = _finger_caps_solid()

    # --- Three clamp assemblies, evenly spaced along Y ---
    for i in range(CLAMP_COUNT):
        y_i = _CLAMP_Y_POSITIONS[i]

        # Fixed base bracket (world coords, shifted to Y position)
        base_i = model.part(f"clamp_base_{i}")
        base_i.visual(
            mesh_from_cadquery(_clamp_base_solid(y_offset=y_i), f"clamp_base_{i}"),
            material=MAT_METAL,
            name=f"clamp_base_{i}",
        )

        # Moving spring-clamp lever (jaw) — joint-local frame
        lever_i = model.part(f"clamp_lever_{i}")
        lever_i.visual(
            mesh_from_cadquery(lever_solid, f"clamp_lever_{i}"),
            material=MAT_METAL,
            name=f"clamp_lever_{i}",
        )
        lever_i.visual(
            mesh_from_cadquery(caps_solid, f"finger_caps_{i}"),
            material=MAT_CAP,
            name=f"finger_caps_{i}",
        )

        # Board carries the fixed clamp base (rigid mount)
        model.articulation(
            f"board_to_base_{i}",
            ArticulationType.FIXED,
            parent=board,
            child=base_i,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
        )

        # Clamp base carries the pivoting lever on the barrel.
        # The lever mesh is authored in joint-local coords (origin at pivot axis,
        # local +x toward the front lip). Placing the joint frame at the barrel
        # in world makes the frames line up. Axis is -Y so that by the right-hand
        # rule positive q rotates the front lip UP toward +Z (opens the jaw).
        model.articulation(
            f"base_to_lever_{i}",
            ArticulationType.REVOLUTE,
            parent=base_i,
            child=lever_i,
            origin=Origin(xyz=(CLAMP_CENTER_X, y_i, PIVOT_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=0.42),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    board = object_model.get_part("board")

    # ---- Board hero geometry: wide thin flat panel ----
    bmin, bmax = ctx.part_world_aabb(board)
    board_len = bmax[0] - bmin[0]
    board_wid = bmax[1] - bmin[1]
    board_thk = bmax[2] - bmin[2]
    ctx.check(
        "board reads as a wide thin clipboard panel",
        board_len > 0.30 and board_wid > 0.28 and board_thk < 0.006,
        details=f"len={board_len:.3f} wid={board_wid:.3f} thk={board_thk:.4f}",
    )

    # ---- Collect all clamp parts and articulations ----
    bases = [object_model.get_part(f"clamp_base_{i}") for i in range(CLAMP_COUNT)]
    levers = [object_model.get_part(f"clamp_lever_{i}") for i in range(CLAMP_COUNT)]
    pivots = [object_model.get_articulation(f"base_to_lever_{i}") for i in range(CLAMP_COUNT)]

    # ---- All 3 pivots are revolute about Y ----
    for i in range(CLAMP_COUNT):
        pivot = pivots[i]
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

    # ---- Each base sits at the top (-X) edge, on the board top face ----
    for i in range(CLAMP_COUNT):
        cmin, cmax = ctx.part_world_aabb(bases[i])
        ctx.check(
            f"base_{i} is near the top (-X) edge of the board",
            cmax[0] < 0.10 and cmin[0] >= bmin[0] - 1e-4,
            details=f"clamp x=[{cmin[0]:.3f},{cmax[0]:.3f}] board x0={bmin[0]:.3f}",
        )
        ctx.check(
            f"base_{i} rises above the board top face",
            cmax[2] > board_thk + 0.005,
            details=f"clamp top z={cmax[2]:.4f} board top z={bmax[2]:.4f}",
        )

    # ---- Each base is mounted to the board (rests on top face, no float) ----
    for i in range(CLAMP_COUNT):
        ctx.expect_overlap(
            bases[i], board, axes="xy", min_overlap=0.02,
            name=f"base_{i} overlaps board in XY",
        )
        ctx.expect_gap(
            bases[i], board, axis="z",
            max_gap=0.0005, max_penetration=0.0020,
            name=f"base_{i} seats on the board top face",
        )

    # ---- Even spacing: measure Y centers of each base via visual AABB ----
    base_y_centers = []
    for i in range(CLAMP_COUNT):
        base_vis = bases[i].get_visual(f"clamp_base_{i}")
        emin, emax = ctx.part_element_world_aabb(bases[i], elem=base_vis)
        base_y_centers.append((emin[1] + emax[1]) / 2.0)

    spacings = [
        base_y_centers[i + 1] - base_y_centers[i]
        for i in range(CLAMP_COUNT - 1)
    ]
    ctx.check(
        "three clamps are evenly spaced along Y",
        all(abs(s - spacings[0]) < 0.005 for s in spacings),
        details=f"y_centers={[round(c, 4) for c in base_y_centers]} spacings={[round(s, 4) for s in spacings]}",
    )
    ctx.check(
        "clamp row has meaningful spacing between units",
        all(abs(s) > 0.060 for s in spacings),
        details=f"spacings={[round(s, 4) for s in spacings]}",
    )

    # ---- Each lever is captured on its base pivot barrel ----
    for i in range(CLAMP_COUNT):
        lever_vis = levers[i].get_visual(f"clamp_lever_{i}")
        base_vis = bases[i].get_visual(f"clamp_base_{i}")
        # Lever pin sleeve is captured around the fixed pivot barrel; the curved
        # cover nests over the base at the pivot. Allow that local overlap.
        ctx.allow_overlap(
            bases[i], levers[i],
            elem_a=base_vis, elem_b=lever_vis,
            reason=f"Lever_{i} pin sleeve is captured around the fixed pivot barrel; the curved cover nests over the base at the pivot.",
        )
        ctx.expect_contact(
            levers[i], bases[i],
            elem_a=lever_vis, elem_b=base_vis,
            contact_tol=0.001,
            name=f"lever_{i} sleeve is captured on base_{i} pivot barrel",
        )

    # ---- Finger caps present on each lever ----
    for i in range(CLAMP_COUNT):
        caps = levers[i].get_visual(f"finger_caps_{i}")
        pmin, pmax = ctx.part_element_world_aabb(levers[i], elem=caps)
        cmin, cmax = ctx.part_world_aabb(bases[i])
        ctx.check(
            f"finger_caps_{i} sit on the rear (toward -X) of lever_{i}",
            pmax[0] < cmax[0],
            details=f"caps x max={pmax[0]:.3f} base x max={cmax[0]:.3f}",
        )

    # ---- Closed (rest) pose: front lips press near the board for all levers ----
    with ctx.pose({pivots[i]: 0.0 for i in range(CLAMP_COUNT)}):
        for i in range(CLAMP_COUNT):
            lev_min0, lev_max0 = ctx.part_world_aabb(levers[i])
            ctx.check(
                f"closed lever_{i} front lip reaches down near the board top face",
                lev_min0[2] < board_thk + 0.004,
                details=f"front lip z={lev_min0[2]:.4f} board top={board_thk:.4f}",
            )
            ctx.expect_overlap(
                levers[i], board, axes="xy", min_overlap=0.01,
                name=f"closed lever_{i} overlaps the board (grips paper area)",
            )

    # ---- Open pose: pressing rear pads down lifts the front lip up for all ----
    upper = pivots[0].motion_limits.upper

    # Capture closed tops for comparison
    closed_tops = []
    closed_cap_zc = []
    with ctx.pose({pivots[i]: 0.0 for i in range(CLAMP_COUNT)}):
        for i in range(CLAMP_COUNT):
            _, lev_max = ctx.part_world_aabb(levers[i])
            closed_tops.append(lev_max[2])
            caps = levers[i].get_visual(f"finger_caps_{i}")
            cap_min, cap_max = ctx.part_element_world_aabb(levers[i], elem=caps)
            closed_cap_zc.append((cap_min[2] + cap_max[2]) / 2.0)

    with ctx.pose({pivots[i]: upper for i in range(CLAMP_COUNT)}):
        for i in range(CLAMP_COUNT):
            caps = levers[i].get_visual(f"finger_caps_{i}")
            cap_min, cap_max = ctx.part_element_world_aabb(levers[i], elem=caps)
            open_cap_zc = (cap_min[2] + cap_max[2]) / 2.0
            _, lev_max = ctx.part_world_aabb(levers[i])
            ctx.check(
                f"opening lever_{i} presses the rear finger pads down",
                open_cap_zc < closed_cap_zc[i] - 0.004,
                details=f"closed cap z={closed_cap_zc[i]:.4f} open cap z={open_cap_zc:.4f}",
            )
            ctx.check(
                f"opening lever_{i} lifts the front jaw up off the board",
                lev_max[2] > closed_tops[i] + 0.005,
                details=f"closed front top z={closed_tops[i]:.4f} open front top z={lev_max[2]:.4f}",
            )

    return ctx.report()


object_model = build_object_model()
