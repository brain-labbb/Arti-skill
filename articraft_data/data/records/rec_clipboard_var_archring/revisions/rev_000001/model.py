from __future__ import annotations

# Clipboard with arch ring-binder fastener.
#
# Variant of the spring-clamp clipboard: the torsion-spring jaw clamp is
# replaced by a low-profile arch ring-binder fastener. A flat metal spine
# plate is riveted along the top edge of the board. Two hinge bosses on the
# spine plate carry a round-wire arch that pivots open (upward, for paper
# loading) and closed (flat over the board, capturing hole-punched sheets).
#
# Layout: the board lies flat in the XY plane, top surface at +Z. The spine
# plate is mounted at the -X (top) edge. The arch wire spans along Y between
# two hinge bosses and arcs forward (+X) over the board. Hinge axis along Y.
# Positive joint motion swings the arch upward (open gesture).

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

# Spine plate: flat metal strip riveted along the top (-X) edge.
SPINE_WID = 0.080       # along Y
SPINE_X0 = 0.008        # front edge (toward board center, +X)
SPINE_X1 = 0.030        # rear edge (near board top, -X side)
SPINE_THK = 0.002       # plate thickness
SPINE_CX = (SPINE_X0 + SPINE_X1) / 2.0

# Hinge bosses: short cylinders along Y at the arch foot positions.
BOSS_RADIUS = 0.003
BOSS_LEN = 0.006
ARCH_HALF_W = 0.028     # half-span between arch feet along Y
HINGE_Z = BOARD_THK + SPINE_THK + BOSS_RADIUS  # hinge axis height

# Arch wire: round-wire semicircular arch in joint-local XY plane.
ARCH_REACH = 0.045      # apex distance from hinge toward +X
WIRE_RADIUS = 0.0018    # wire cross-section radius
ARCH_N_PTS = 17         # path sample count

MAT_BOARD = "clipboard_blue"
MAT_SPINE = "brushed_steel"
MAT_ARCH = "chrome_wire"


# ---------------------------------------------------------------------------
# Geometry builders
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


def _spine_plate_solid() -> cq.Workplane:
    """Flat metal spine plate with two cylindrical hinge bosses.

    Built in WORLD coordinates. The plate sits on the board top face; the
    bosses rise above the plate to carry the arch wire hinge pins.
    """
    plate_len = SPINE_X1 - SPINE_X0

    # Flat riveted footplate.
    plate = (
        cq.Workplane("XY")
        .workplane(offset=BOARD_THK + SPINE_THK / 2.0)
        .box(plate_len, SPINE_WID, SPINE_THK)
        .translate((SPINE_CX, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.003)
    )

    # Two hinge bosses: short cylinders along Y at the arch foot positions.
    boss_z_center = BOARD_THK + SPINE_THK + BOSS_RADIUS
    result = plate
    for sign in (-1.0, 1.0):
        y_start = sign * ARCH_HALF_W - BOSS_LEN / 2.0
        boss = (
            cq.Workplane("XZ")
            .workplane(offset=y_start)
            .center(SPINE_CX, boss_z_center)
            .circle(BOSS_RADIUS)
            .extrude(BOSS_LEN)
        )
        result = result.union(boss)

    return result


def _rivet_head(x: float, y: float) -> cq.Workplane:
    """Single dome rivet head sitting on the spine plate top surface."""
    return (
        cq.Workplane("XY")
        .workplane(offset=BOARD_THK + SPINE_THK)
        .center(x, y)
        .circle(0.0028)
        .extrude(0.0010)
    )


def _arch_wire_points() -> list[tuple[float, float, float]]:
    """Semicircular arch path in joint-local frame.

    All points lie in the XY plane (z=0). The path traces a half-ellipse from
    foot 1 at (0, -ARCH_HALF_W) through the apex at (ARCH_REACH, 0) to foot 2
    at (0, +ARCH_HALF_W). At q=0 the arch lies flat over the board.
    """
    points: list[tuple[float, float, float]] = []
    for i in range(ARCH_N_PTS):
        theta = -math.pi / 2.0 + math.pi * i / (ARCH_N_PTS - 1)
        x = ARCH_REACH * math.cos(theta)
        y = ARCH_HALF_W * math.sin(theta)
        points.append((x, y, 0.0))
    return points


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clipboard_ring_binder")

    model.material(MAT_BOARD, rgba=(0.13, 0.45, 0.92, 1.0))
    model.material(MAT_SPINE, rgba=(0.72, 0.73, 0.76, 1.0))
    model.material(MAT_ARCH, rgba=(0.80, 0.82, 0.85, 1.0))

    # --- Board (root) ---
    board = model.part("board")
    board.visual(
        mesh_from_cadquery(_board_solid(), "board_panel"),
        material=MAT_BOARD,
        name="board_panel",
    )

    # --- Spine plate (fixed to board) ---
    spine = model.part("spine_plate")
    spine.visual(
        mesh_from_cadquery(_spine_plate_solid(), "spine_body"),
        material=MAT_SPINE,
        name="spine_body",
    )

    # Rivet heads: repeated decoration on spine plate, loop with name_i.
    rivet_positions = [
        (SPINE_X0 + 0.005, -0.016),
        (SPINE_X0 + 0.005,  0.016),
        (SPINE_X1 - 0.005, -0.016),
        (SPINE_X1 - 0.005,  0.016),
    ]
    for i in range(len(rivet_positions)):
        rx, ry = rivet_positions[i]
        spine.visual(
            mesh_from_cadquery(_rivet_head(rx, ry), f"rivet_{i}"),
            material=MAT_SPINE,
            name=f"rivet_{i}",
        )

    # --- Arch wire (single revolute moving part) ---
    arch = model.part("arch")
    arch_geom = tube_from_spline_points(
        _arch_wire_points(),
        radius=WIRE_RADIUS,
        samples_per_segment=10,
        radial_segments=16,
        cap_ends=True,
    )
    arch.visual(
        mesh_from_geometry(arch_geom, "arch_wire"),
        material=MAT_ARCH,
        name="arch_wire",
    )

    # --- Board carries the spine plate (rigid mount) ---
    model.articulation(
        "board_to_spine",
        ArticulationType.FIXED,
        parent=board,
        child=spine,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # --- Spine carries the arch on the hinge ---
    # Arch mesh is authored in joint-local coords: origin at the hinge axis
    # center, local XY plane contains the arch path. Placing the joint frame at
    # the hinge center in world aligns the frames. The arch extends along local
    # +X at q=0 (forward over the board). Axis is -Y so that by the right-hand
    # rule, positive q rotates the apex from +X toward +Z (opens the arch up).
    model.articulation(
        "spine_to_arch",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=arch,
        origin=Origin(xyz=(SPINE_CX, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=3.0, lower=0.0, upper=1.50,
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    board = object_model.get_part("board")
    spine = object_model.get_part("spine_plate")
    arch = object_model.get_part("arch")
    hinge = object_model.get_articulation("spine_to_arch")

    arch_vis = arch.get_visual("arch_wire")
    spine_vis = spine.get_visual("spine_body")

    # The arch wire feet are intentionally captured by the hinge bosses on the
    # spine plate. Allow that local mechanical capture overlap.
    ctx.allow_overlap(
        spine,
        arch,
        elem_a=spine_vis,
        elem_b=arch_vis,
        reason="Arch wire feet are captured by the hinge bosses on the spine plate; the wire nests inside the boss cylinders at the pivot.",
    )

    # ---- Joint contract: revolute about Y (spine width) axis ----
    ctx.check(
        "hinge is revolute",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={hinge.articulation_type}",
    )
    ax = tuple(hinge.axis)
    ctx.check(
        "hinge axis runs along the spine width (Y)",
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

    # ---- Spine plate at top edge of board ----
    smin, smax = ctx.part_world_aabb(spine)
    ctx.check(
        "spine plate is near the top (-X) edge of the board",
        smax[0] < 0.06 and smin[0] >= bmin[0] - 1e-4,
        details=f"spine x=[{smin[0]:.3f},{smax[0]:.3f}] board x0={bmin[0]:.3f}",
    )
    ctx.check(
        "spine plate rises above the board top face",
        smax[2] > board_thk + 0.003,
        details=f"spine top z={smax[2]:.4f} board top z={bmax[2]:.4f}",
    )

    # ---- Spine is mounted on the board (seats on top face) ----
    ctx.expect_overlap(spine, board, axes="xy", min_overlap=0.02)
    ctx.expect_gap(
        spine,
        board,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.002,
        name="spine plate seats on the board top face",
    )

    # ---- Arch wire reads as a curved semicircular shape ----
    amin, amax = ctx.part_world_aabb(arch)
    arch_dx = amax[0] - amin[0]
    arch_dy = amax[1] - amin[1]
    ctx.check(
        "arch wire spans significantly in both X and Y (semicircular shape)",
        arch_dx > 0.025 and arch_dy > 0.030,
        details=f"dx={arch_dx:.3f} dy={arch_dy:.3f}",
    )

    # ---- Arch is supported by spine (contact at hinge bosses) ----
    ctx.expect_contact(
        arch,
        spine,
        elem_a=arch_vis,
        elem_b=spine_vis,
        contact_tol=0.003,
        name="arch wire contacts spine hinge bosses",
    )

    # ---- Closed pose (q=0): arch lies flat over the board ----
    with ctx.pose({hinge: 0.0}):
        amin0, amax0 = ctx.part_world_aabb(arch)
        ctx.expect_overlap(
            arch,
            board,
            axes="xy",
            min_overlap=0.02,
            name="closed arch overlaps the board in XY (arch over board)",
        )
        ctx.check(
            "closed arch apex is low (near hinge height)",
            amax0[2] < HINGE_Z + WIRE_RADIUS + 0.004,
            details=f"apex top z={amax0[2]:.4f} hinge z={HINGE_Z:.4f}",
        )

    # ---- Open pose (q=upper): arch swings up for paper loading ----
    upper = hinge.motion_limits.upper
    with ctx.pose({hinge: upper}):
        amin1, amax1 = ctx.part_world_aabb(arch)
        ctx.check(
            "open arch apex rises significantly above closed position",
            amax1[2] > amax0[2] + 0.020,
            details=f"closed apex z={amax0[2]:.4f} open apex z={amax1[2]:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
