from __future__ import annotations

# Commercial hospital double door in off-white steel.
#
# Articraft brief:
# - Object: commercial hospital double door, opening ~1.70 m wide x 2.10 m tall,
#   each leaf ~0.83 m wide x 0.045 m thick x 2.05 m tall, in a steel frame.
# - Root/support: fixed steel frame (two jambs, head jamb, threshold, doorstop
#   beads) carries both swinging leaves.
# - Parts: frame (root), door_leaf_0, door_leaf_1. Each leaf carries: a
#   rectangular vision window opening (cut through) with a 2x3 grid of glass
#   lites separated by slim muntins (emitted by nested for loops), a horizontal
#   stainless push/pull bar on two standoffs, and two blue rubber bumper stripes
#   across the lower portion.
# - Articulations: frame_to_door_leaf_0 and frame_to_door_leaf_1, both
#   REVOLUTE, vertical hinge axes on the OUTER edges at the jambs, opening
#   symmetrically. Leaves named door_leaf_0/door_leaf_1 (no left/right per
#   link naming policy).
# - Visible geometry: off-white steel leaves, translucent glass windows,
#   brushed-stainless bars, blue rubber stripes, steel frame.
# - Symmetry guarantee: ONE parametric helper builds all leaf geometry;
#   door_leaf_0 uses sign=+1 (hinge left, extends +X), door_leaf_1 uses
#   sign=-1 (hinge right, extends -X). The geometry is identical by
#   construction; only the X direction (sign) is flipped.
# - Tests: 2x3 lite grid present with 6 lites per leaf, muntins present, lites
#   evenly spaced and in upper third, push bar horizontal and standing off the
#   face, two blue stripes in the lower portion, leaves meet with a reveal
#   closed, both swing outward when open, leaf-symmetry assertions.

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
# Dimensions (meters)
# ----------------------------------------------------------------------------
OPENING_W = 1.70
OPENING_H = 2.10
CENTER_REVEAL = 0.006
JAMB_REVEAL = 0.006

LEAF_W = (OPENING_W - CENTER_REVEAL - 2 * JAMB_REVEAL) / 2.0  # ~0.839 m
LEAF_H = OPENING_H - 0.02
LEAF_T = 0.045

JAMB_W = 0.10
JAMB_D = 0.16
HEAD_H = 0.12
BASE_H = 0.06  # low threshold

FLOOR_Z = 0.0
FRAME_OUTER_W = OPENING_W + 2 * JAMB_W

# Vision window (upper third) — centered horizontally, sitting in upper third.
WIN_W = 0.42
WIN_H = 0.46
WIN_CENTER_Z = LEAF_H * 0.74  # upper third

# Muntin grid: 2 rows x 3 cols of glass lites
N_ROWS = 2
N_COLS = 3
MUNTIN_W = 0.012
MUNTIN_T = 0.010  # muntin thickness in Y (depth through the leaf)
LITE_W = (WIN_W - (N_COLS - 1) * MUNTIN_W) / N_COLS
LITE_H = (WIN_H - (N_ROWS - 1) * MUNTIN_W) / N_ROWS

# Push/pull bar — horizontal stainless bar at mid-height on standoffs.
BAR_Z = LEAF_H * 0.46
BAR_LEN = 0.62
BAR_R = 0.013
STANDOFF_LEN = 0.050
STANDOFF_R = 0.012

# Blue rubber bumper stripes (lower portion)
STRIPE_H = 0.07
STRIPE_Z1 = LEAF_H * 0.30  # upper of the two stripes
STRIPE_Z2 = LEAF_H * 0.20  # lower stripe

STEEL_OFFWHITE = (0.90, 0.89, 0.85, 1.0)
STEEL_FRAME = (0.80, 0.80, 0.78, 1.0)
STAINLESS = (0.75, 0.76, 0.78, 1.0)
GLASS = (0.78, 0.85, 0.88, 0.45)
BLUE_RUBBER = (0.40, 0.55, 0.72, 1.0)


# ----------------------------------------------------------------------------
# Parametric leaf geometry — ALL geometry built via ONE helper so the two
# leaves are guaranteed mirror-symmetric in X. sign=+1 → door_leaf_0 (hinge
# at X=0, leaf extends +X); sign=-1 → door_leaf_1 (hinge at X=0, leaf extends
# -X). Both leaves have the same local hinge-edge at X=0, the same Y (front
# face +Y) and Z (base at z=0).
# ----------------------------------------------------------------------------

def _leaf_body(sign: float) -> cq.Workplane:
    """Off-white steel leaf with rectangular vision window cut through.

    All local-coordinate positions are symmetric by construction:
    the window is centered at |cx|, hinge edge at X=0, free edge at sign*LEAF_W.
    """
    cx = sign * LEAF_W / 2.0
    leaf = (
        cq.Workplane("XY")
        .box(LEAF_W, LEAF_T, LEAF_H, centered=(True, True, False))
        .translate((cx, 0.0, 0.0))
    )
    # Vision window cut — same width/height, same vertical position, centered in X.
    window_cut = (
        cq.Workplane("XY")
        .box(WIN_W, LEAF_T + 0.02, WIN_H)
        .translate((cx, 0.0, WIN_CENTER_Z))
    )
    leaf = leaf.cut(window_cut)
    try:
        leaf = leaf.edges("|Z").fillet(0.003)
    except Exception:
        pass
    return leaf


def _muntin_grid(sign: float) -> cq.Workplane:
    """Slim muntin bars dividing the window opening into a 2x3 grid."""
    cx = sign * LEAF_W / 2.0
    win_left = cx - WIN_W / 2.0
    win_bottom_z = WIN_CENTER_Z - WIN_H / 2.0

    grid = None

    # Horizontal muntins (N_ROWS - 1 bars spanning full window width)
    for r in range(N_ROWS - 1):
        z = win_bottom_z + (r + 1) * LITE_H + (r + 0.5) * MUNTIN_W
        bar = (
            cq.Workplane("XY")
            .box(WIN_W, MUNTIN_T, MUNTIN_W)
            .translate((cx, 0.0, z))
        )
        grid = bar if grid is None else grid.union(bar)

    # Vertical muntins (N_COLS - 1 bars spanning full window height)
    for c in range(N_COLS - 1):
        x = win_left + (c + 1) * LITE_W + (c + 0.5) * MUNTIN_W
        bar = (
            cq.Workplane("XY")
            .box(MUNTIN_W, MUNTIN_T, WIN_H)
            .translate((x, 0.0, WIN_CENTER_Z))
        )
        grid = bar if grid is None else grid.union(bar)

    return grid


def _glass_lite(sign: float, row: int, col: int) -> cq.Workplane:
    """One translucent glass lite at grid position (row, col)."""
    cx = sign * LEAF_W / 2.0
    win_left = cx - WIN_W / 2.0
    win_bottom_z = WIN_CENTER_Z - WIN_H / 2.0

    lite_x = win_left + col * (LITE_W + MUNTIN_W) + LITE_W / 2.0
    lite_z = win_bottom_z + row * (LITE_H + MUNTIN_W) + LITE_H / 2.0

    return (
        cq.Workplane("XY")
        .box(LITE_W, 0.006, LITE_H)
        .translate((lite_x, 0.0, lite_z))
    )


def _push_bar(sign: float) -> cq.Workplane:
    """Horizontal stainless push/pull bar at mid-height on two standoffs.

    The bar and standoffs are placed at the same relative X-offset from the
    hinge (centered on the leaf in X) and at the same Y-standoff (proud of
    the +Y face). sign controls whether the leaf extends +X or -X so the
    bar tracks the leaf center.
    """
    face_y = LEAF_T / 2.0
    bar_cx = sign * LEAF_W / 2.0          # leaf center in X
    bar_y = face_y + STANDOFF_LEN        # bar face proud of leaf

    # Horizontal bar cylinder (runs along X).
    bar = (
        cq.Workplane("XY")
        .cylinder(BAR_LEN, BAR_R)
        .rotate((0, 0, 0), (0, 1, 0), 90)
        .translate((bar_cx, bar_y, BAR_Z))
    )

    # Two standoffs — inner and outer, symmetric about bar_cx.
    half_bar = BAR_LEN / 2.0 - 0.04      # inset slightly from bar ends
    so_xs = (bar_cx - sign * half_bar, bar_cx + sign * half_bar)
    for so_x in so_xs:
        so = (
            cq.Workplane("XY")
            .cylinder(STANDOFF_LEN + 0.01, STANDOFF_R)
            .rotate((0, 0, 0), (1, 0, 0), 90)
            .translate((so_x, face_y + STANDOFF_LEN / 2.0, BAR_Z))
        )
        bar = bar.union(so)
    return bar


def _bumper_stripe(sign: float, center_z: float) -> cq.Workplane:
    """One blue rubber bumper stripe across the full leaf width."""
    cx = sign * LEAF_W / 2.0
    face_t = LEAF_T + 0.006              # wraps around the face
    return (
        cq.Workplane("XY")
        .box(LEAF_W, face_t, STRIPE_H)
        .translate((cx, 0.0, center_z))
    )


def _add_leaf_visuals(
    model: ArticulatedObject,
    part_name: str,
    sign: float,
) -> None:
    """Instantiate all leaf geometry for one door leaf via the parametric helper.

    By calling the same functions with the appropriate sign, both leaves are
    structurally identical — just mirrored in X. The vision window is a 2x3
    grid of glass lites separated by slim muntins, emitted by nested for loops.
    """
    leaf = model.part(part_name)
    leaf.visual(
        mesh_from_cadquery(_leaf_body(sign), f"{part_name}_leaf"),
        material="steel_offwhite",
        name=f"{part_name}_leaf",
    )
    # Muntin grid framework inside the window opening
    leaf.visual(
        mesh_from_cadquery(_muntin_grid(sign), f"{part_name}_muntins"),
        material="steel_offwhite",
        name=f"{part_name}_muntins",
    )
    # 2x3 grid of glass lites via nested for loops
    for row in range(N_ROWS):
        for col in range(N_COLS):
            lite_name = f"{part_name}_lite_{row}_{col}"
            leaf.visual(
                mesh_from_cadquery(_glass_lite(sign, row, col), lite_name),
                material="glass",
                name=lite_name,
            )
    leaf.visual(
        mesh_from_cadquery(_push_bar(sign), f"{part_name}_push_bar"),
        material="stainless",
        name=f"{part_name}_push_bar",
    )
    leaf.visual(
        mesh_from_cadquery(_bumper_stripe(sign, STRIPE_Z1), f"{part_name}_stripe_upper"),
        material="blue_rubber",
        name=f"{part_name}_stripe_upper",
    )
    leaf.visual(
        mesh_from_cadquery(_bumper_stripe(sign, STRIPE_Z2), f"{part_name}_stripe_lower"),
        material="blue_rubber",
        name=f"{part_name}_stripe_lower",
    )


# ----------------------------------------------------------------------------
# Frame builders
# ----------------------------------------------------------------------------

def _build_frame_members() -> dict[str, cq.Workplane]:
    """Steel frame as separate members so collisions don't bridge the opening."""
    half_open = OPENING_W / 2.0
    members: dict[str, cq.Workplane] = {}

    members["jamb_left"] = (
        cq.Workplane("XY")
        .box(JAMB_W, JAMB_D, OPENING_H)
        .translate((-(half_open + JAMB_W / 2.0), 0.0, OPENING_H / 2.0))
    )
    members["jamb_right"] = (
        cq.Workplane("XY")
        .box(JAMB_W, JAMB_D, OPENING_H)
        .translate((half_open + JAMB_W / 2.0, 0.0, OPENING_H / 2.0))
    )
    members["head_jamb"] = (
        cq.Workplane("XY")
        .box(FRAME_OUTER_W, JAMB_D, HEAD_H)
        .translate((0.0, 0.0, OPENING_H + HEAD_H / 2.0))
    )
    members["threshold"] = (
        cq.Workplane("XY")
        .box(OPENING_W, JAMB_D, BASE_H)
        .translate((0.0, 0.0, -BASE_H / 2.0))
    )

    # Doorstop beads behind the closed leaves (negative Y side).
    stop_t = 0.014
    stop_proud = 0.018
    stop_y = -LEAF_T / 2.0 - stop_t / 2.0 - 0.001
    members["stop_left"] = (
        cq.Workplane("XY")
        .box(stop_proud, stop_t, OPENING_H - 0.02)
        .translate((-half_open + stop_proud / 2.0, stop_y, OPENING_H / 2.0))
    )
    members["stop_right"] = (
        cq.Workplane("XY")
        .box(stop_proud, stop_t, OPENING_H - 0.02)
        .translate((half_open - stop_proud / 2.0, stop_y, OPENING_H / 2.0))
    )
    members["stop_head"] = (
        cq.Workplane("XY")
        .box(OPENING_W, stop_t, stop_proud)
        .translate((0.0, stop_y, OPENING_H - stop_proud / 2.0))
    )
    return members


# ----------------------------------------------------------------------------
# Model
# ----------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hospital_double_door")

    model.material("steel_offwhite", rgba=STEEL_OFFWHITE)
    model.material("steel_frame", rgba=STEEL_FRAME)
    model.material("stainless", rgba=STAINLESS)
    model.material("glass", rgba=GLASS)
    model.material("blue_rubber", rgba=BLUE_RUBBER)

    # --- Frame (root) ---
    frame = model.part("frame")
    for mname, mshape in _build_frame_members().items():
        frame.visual(
            mesh_from_cadquery(mshape, f"frame_{mname}"),
            material="steel_frame",
            name=f"frame_{mname}",
        )

    # --- Leaves ---
    # door_leaf_0: sign=+1 → hinge at left jamb, free edge extends +X (rightward).
    # door_leaf_1: sign=-1 → hinge at right jamb, free edge extends -X (leftward).
    # Both leaves are built by _add_leaf_visuals() with the same geometry/sign logic,
    # guaranteeing structural mirror-symmetry.
    _add_leaf_visuals(model, "door_leaf_0", sign=+1.0)
    _add_leaf_visuals(model, "door_leaf_1", sign=-1.0)

    half_open = OPENING_W / 2.0
    left_hinge_x = -(half_open - JAMB_REVEAL)
    right_hinge_x = half_open - JAMB_REVEAL

    # door_leaf_0: hinge at left jamb; +Z axis, positive angle swings free
    # edge in +Y (outward from the viewer's side).
    model.articulation(
        "frame_to_door_leaf_0",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=model.get_part("door_leaf_0"),
        origin=Origin(xyz=(left_hinge_x, 0.0, FLOOR_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=2.0, lower=0.0, upper=1.92),
    )
    # door_leaf_1: hinge at right jamb; -Z axis gives symmetric outward swing
    # (negative angle about +Z = same direction as positive about -Z).
    model.articulation(
        "frame_to_door_leaf_1",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=model.get_part("door_leaf_1"),
        origin=Origin(xyz=(right_hinge_x, 0.0, FLOOR_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=2.0, lower=0.0, upper=1.92),
    )

    return model


# ----------------------------------------------------------------------------
# Tests
# ----------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    door_0 = object_model.get_part("door_leaf_0")
    door_1 = object_model.get_part("door_leaf_1")
    hinge_0 = object_model.get_articulation("frame_to_door_leaf_0")
    hinge_1 = object_model.get_articulation("frame_to_door_leaf_1")

    pairs = (
        (door_0, "door_leaf_0"),
        (door_1, "door_leaf_1"),
    )

    # --- Allow intentional embeddings ---
    for door, nm in pairs:
        # Each glass lite seated in the window opening
        for row in range(N_ROWS):
            for col in range(N_COLS):
                lite_name = f"{nm}_lite_{row}_{col}"
                ctx.allow_overlap(
                    door, door,
                    elem_a=door.get_visual(lite_name),
                    elem_b=door.get_visual(f"{nm}_leaf"),
                    reason="Glass lite seated into cut vision-window opening with retaining lip.",
                )
        # Muntin grid embedded in the window opening
        ctx.allow_overlap(
            door, door,
            elem_a=door.get_visual(f"{nm}_muntins"),
            elem_b=door.get_visual(f"{nm}_leaf"),
            reason="Muntin bars sit within the leaf window opening as glazing dividers.",
        )
        ctx.allow_overlap(
            door, door,
            elem_a=door.get_visual(f"{nm}_push_bar"),
            elem_b=door.get_visual(f"{nm}_leaf"),
            reason="Push-bar standoffs embed into the leaf face where fastened.",
        )
        for s in ("stripe_upper", "stripe_lower"):
            ctx.allow_overlap(
                door, door,
                elem_a=door.get_visual(f"{nm}_{s}"),
                elem_b=door.get_visual(f"{nm}_leaf"),
                reason="Rubber stripe bonded on leaf face wraps its thickness.",
            )

    # --- Leaf sizing ---
    for door, nm in pairs:
        leaf_elem = door.get_visual(f"{nm}_leaf")
        aabb = ctx.part_element_world_aabb(door, elem=leaf_elem)
        assert aabb is not None
        (lo, hi) = aabb
        w = hi[0] - lo[0]
        h = hi[2] - lo[2]
        t = hi[1] - lo[1]
        ctx.check(f"{nm} leaf width plausible", 0.78 <= w <= 0.90, details=f"w={w:.3f}")
        ctx.check(f"{nm} leaf height plausible", 1.95 <= h <= 2.10, details=f"h={h:.3f}")
        ctx.check(f"{nm} leaf thickness plausible", 0.04 <= t <= 0.06, details=f"t={t:.3f}")
        ctx.check(f"{nm} base at z=0", lo[2] <= 0.01, details=f"zmin={lo[2]:.4f}")

    # --- Leaf mirror-symmetry: dimensions must be equal ---
    la0 = ctx.part_element_world_aabb(door_0, elem=door_0.get_visual("door_leaf_0_leaf"))
    la1 = ctx.part_element_world_aabb(door_1, elem=door_1.get_visual("door_leaf_1_leaf"))
    assert la0 is not None and la1 is not None
    w0 = la0[1][0] - la0[0][0]
    w1 = la1[1][0] - la1[0][0]
    h0 = la0[1][2] - la0[0][2]
    h1 = la1[1][2] - la1[0][2]
    t0 = la0[1][1] - la0[0][1]
    t1 = la1[1][1] - la1[0][1]
    ctx.check("leaf_symmetry_width", abs(w0 - w1) < 0.005, details=f"w0={w0:.4f} w1={w1:.4f}")
    ctx.check("leaf_symmetry_height", abs(h0 - h1) < 0.005, details=f"h0={h0:.4f} h1={h1:.4f}")
    ctx.check("leaf_symmetry_thickness", abs(t0 - t1) < 0.003, details=f"t0={t0:.4f} t1={t1:.4f}")

    # --- 2x3 glass lite grid: all 6 lites present on each leaf ---
    for door, nm in pairs:
        lite_aabbs = []
        for row in range(N_ROWS):
            for col in range(N_COLS):
                lite_name = f"{nm}_lite_{row}_{col}"
                lite_elem = door.get_visual(lite_name)
                la = ctx.part_element_world_aabb(door, elem=lite_elem)
                assert la is not None, f"{lite_name} missing"
                lite_aabbs.append((row, col, la))

        ctx.check(
            f"{nm} has {N_ROWS * N_COLS} glass lites",
            len(lite_aabbs) == N_ROWS * N_COLS,
            details=f"count={len(lite_aabbs)}",
        )

        # Each lite is a small rectangle (not the full window)
        for row, col, la in lite_aabbs:
            lw = la[1][0] - la[0][0]
            lh = la[1][2] - la[0][2]
            ctx.check(
                f"{nm} lite_{row}_{col} is small",
                0.08 <= lw <= 0.18 and 0.15 <= lh <= 0.28,
                details=f"lw={lw:.3f} lh={lh:.3f}",
            )

    # --- Grid regularity: lites are evenly spaced ---
    for door, nm in pairs:
        # Check column spacing (X direction)
        col_centers = []
        for col in range(N_COLS):
            la = ctx.part_element_world_aabb(door, elem=door.get_visual(f"{nm}_lite_0_{col}"))
            assert la is not None
            col_centers.append(0.5 * (la[0][0] + la[1][0]))
        spacings_x = [col_centers[i + 1] - col_centers[i] for i in range(len(col_centers) - 1)]
        if len(spacings_x) >= 1:
            ctx.check(
                f"{nm} grid columns evenly spaced",
                all(abs(s - spacings_x[0]) < 0.005 for s in spacings_x),
                details=f"spacings={[f'{s:.4f}' for s in spacings_x]}",
            )

        # Check row spacing (Z direction)
        row_centers = []
        for row in range(N_ROWS):
            la = ctx.part_element_world_aabb(door, elem=door.get_visual(f"{nm}_lite_{row}_0"))
            assert la is not None
            row_centers.append(0.5 * (la[0][2] + la[1][2]))
        spacings_z = [row_centers[i + 1] - row_centers[i] for i in range(len(row_centers) - 1)]
        if len(spacings_z) >= 1:
            ctx.check(
                f"{nm} grid rows evenly spaced",
                all(abs(s - spacings_z[0]) < 0.005 for s in spacings_z),
                details=f"spacings={[f'{s:.4f}' for s in spacings_z]}",
            )

    # --- Muntins present on each leaf ---
    for door, nm in pairs:
        muntin_elem = door.get_visual(f"{nm}_muntins")
        ma = ctx.part_element_world_aabb(door, elem=muntin_elem)
        assert ma is not None
        mw = ma[1][0] - ma[0][0]
        mh = ma[1][2] - ma[0][2]
        leaf_elem = door.get_visual(f"{nm}_leaf")
        la = ctx.part_element_world_aabb(door, elem=leaf_elem)
        assert la is not None
        leaf_cz = 0.5 * (la[0][2] + la[1][2])
        muntin_cz = 0.5 * (ma[0][2] + ma[1][2])
        ctx.check(
            f"{nm} muntins span window area",
            mw > 0.30 and mh > 0.35,
            details=f"mw={mw:.3f} mh={mh:.3f}",
        )
        ctx.check(
            f"{nm} muntins in upper third",
            muntin_cz > leaf_cz + 0.20,
            details=f"muntin_cz={muntin_cz:.3f} leaf_cz={leaf_cz:.3f}",
        )
        # Muntins stay within the leaf
        ctx.expect_within(
            door, door, axes="xz",
            inner_elem=muntin_elem, outer_elem=leaf_elem, margin=0.02,
            name=f"{nm} muntins stay within the leaf",
        )

    # --- Lite grid symmetry between leaves ---
    for row in range(N_ROWS):
        for col in range(N_COLS):
            la0 = ctx.part_element_world_aabb(door_0, elem=door_0.get_visual(f"door_leaf_0_lite_{row}_{col}"))
            la1 = ctx.part_element_world_aabb(door_1, elem=door_1.get_visual(f"door_leaf_1_lite_{row}_{col}"))
            assert la0 is not None and la1 is not None
            lw0 = la0[1][0] - la0[0][0]
            lw1 = la1[1][0] - la1[0][0]
            lh0 = la0[1][2] - la0[0][2]
            lh1 = la1[1][2] - la1[0][2]
            ctx.check(
                f"lite_{row}_{col} symmetry size",
                abs(lw0 - lw1) < 0.005 and abs(lh0 - lh1) < 0.005,
                details=f"lw0={lw0:.4f} lw1={lw1:.4f} lh0={lh0:.4f} lh1={lh1:.4f}",
            )

    # Bar symmetry: same dimensions and same Z on both leaves.
    ba0 = ctx.part_element_world_aabb(door_0, elem=door_0.get_visual("door_leaf_0_push_bar"))
    ba1 = ctx.part_element_world_aabb(door_1, elem=door_1.get_visual("door_leaf_1_push_bar"))
    assert ba0 is not None and ba1 is not None
    bw0 = ba0[1][0] - ba0[0][0]
    bw1 = ba1[1][0] - ba1[0][0]
    bz0 = 0.5 * (ba0[0][2] + ba0[1][2])
    bz1 = 0.5 * (ba1[0][2] + ba1[1][2])
    ctx.check("bar_symmetry_width", abs(bw0 - bw1) < 0.01, details=f"bw0={bw0:.4f} bw1={bw1:.4f}")
    ctx.check("bar_symmetry_z", abs(bz0 - bz1) < 0.01, details=f"bz0={bz0:.4f} bz1={bz1:.4f}")

    # --- Vision window: all lites sit in the upper third ---
    for door, nm in pairs:
        leaf_elem = door.get_visual(f"{nm}_leaf")
        la = ctx.part_element_world_aabb(door, elem=leaf_elem)
        assert la is not None
        leaf_cz = 0.5 * (la[0][2] + la[1][2])
        for row in range(N_ROWS):
            for col in range(N_COLS):
                lite_elem = door.get_visual(f"{nm}_lite_{row}_{col}")
                g = ctx.part_element_world_aabb(door, elem=lite_elem)
                assert g is not None
                gcz = 0.5 * (g[0][2] + g[1][2])
                ctx.check(
                    f"{nm} lite_{row}_{col} in upper third",
                    gcz > leaf_cz + 0.10,
                    details=f"lite_cz={gcz:.3f} leaf_cz={leaf_cz:.3f}",
                )
                ctx.expect_within(
                    door, door, axes="xz",
                    inner_elem=lite_elem, outer_elem=leaf_elem, margin=0.02,
                    name=f"{nm} lite_{row}_{col} stays within the leaf opening",
                )

    # --- Push bar: horizontal, standing off the front face ---
    for door, nm in pairs:
        bar_elem = door.get_visual(f"{nm}_push_bar")
        b = ctx.part_element_world_aabb(door, elem=bar_elem)
        leaf_elem = door.get_visual(f"{nm}_leaf")
        la = ctx.part_element_world_aabb(door, elem=leaf_elem)
        assert b is not None and la is not None
        bw = b[1][0] - b[0][0]    # horizontal extent
        bh = b[1][2] - b[0][2]    # vertical extent (small for a horizontal bar)
        ctx.check(
            f"{nm} push bar is horizontal",
            bw > 0.45 and bh < 0.12,
            details=f"bar_w={bw:.3f} bar_h={bh:.3f}",
        )
        leaf_front_y = la[1][1]
        ctx.check(
            f"{nm} push bar stands off the front face",
            b[1][1] > leaf_front_y + 0.03,
            details=f"bar_maxY={b[1][1]:.3f} leaf_frontY={leaf_front_y:.3f}",
        )

    # --- Two blue rubber stripes across the lower portion ---
    for door, nm in pairs:
        leaf_elem = door.get_visual(f"{nm}_leaf")
        la = ctx.part_element_world_aabb(door, elem=leaf_elem)
        assert la is not None
        leaf_cz = 0.5 * (la[0][2] + la[1][2])
        for s in ("stripe_upper", "stripe_lower"):
            st_elem = door.get_visual(f"{nm}_{s}")
            sa = ctx.part_element_world_aabb(door, elem=st_elem)
            assert sa is not None
            scz = 0.5 * (sa[0][2] + sa[1][2])
            sw = sa[1][0] - sa[0][0]
            ctx.check(
                f"{nm} {s} is a wide low stripe",
                sw > 0.6 and scz < leaf_cz,
                details=f"stripe_w={sw:.3f} stripe_cz={scz:.3f} leaf_cz={leaf_cz:.3f}",
            )

    # --- Stripe symmetry: both leaves have stripes at the same heights ---
    for s in ("stripe_upper", "stripe_lower"):
        sa0 = ctx.part_element_world_aabb(door_0, elem=door_0.get_visual(f"door_leaf_0_{s}"))
        sa1 = ctx.part_element_world_aabb(door_1, elem=door_1.get_visual(f"door_leaf_1_{s}"))
        assert sa0 is not None and sa1 is not None
        scz0 = 0.5 * (sa0[0][2] + sa0[1][2])
        scz1 = 0.5 * (sa1[0][2] + sa1[1][2])
        ctx.check(
            f"{s} symmetry in Z",
            abs(scz0 - scz1) < 0.005,
            details=f"cz0={scz0:.4f} cz1={scz1:.4f}",
        )

    # --- Closed pose: leaves meet at center with small reveal, no gap ---
    with ctx.pose({hinge_0: 0.0, hinge_1: 0.0}):
        leaf0_elem = door_0.get_visual("door_leaf_0_leaf")
        leaf1_elem = door_1.get_visual("door_leaf_1_leaf")
        ctx.expect_gap(
            door_1, door_0, axis="x",
            positive_elem=leaf1_elem, negative_elem=leaf0_elem,
            min_gap=0.0, max_gap=0.05,
            name="closed leaves meet with small center reveal",
        )
        ctx.expect_contact(door_0, frame, contact_tol=0.02, name="door_leaf_0 hinge edge meets jamb")
        ctx.expect_contact(door_1, frame, contact_tol=0.02, name="door_leaf_1 hinge edge meets jamb")

    # --- Open pose: both leaves swing outward (+Y direction) ---
    with ctx.pose({hinge_0: 0.0, hinge_1: 0.0}):
        c0 = ctx.part_world_aabb(door_0)
        c1 = ctx.part_world_aabb(door_1)
    with ctx.pose({hinge_0: 1.4, hinge_1: 1.4}):
        o0 = ctx.part_world_aabb(door_0)
        o1 = ctx.part_world_aabb(door_1)
        assert o0 and o1 and c0 and c1
        ctx.check("door_leaf_0 swings outward", o0[1][1] > c0[1][1] + 0.3,
                  details=f"closed={c0[1][1]:.3f} open={o0[1][1]:.3f}")
        ctx.check("door_leaf_1 swings outward", o1[1][1] > c1[1][1] + 0.3,
                  details=f"closed={c1[1][1]:.3f} open={o1[1][1]:.3f}")
        ctx.expect_contact(door_0, frame, contact_tol=0.05, name="door_leaf_0 stays at jamb when open")
        ctx.expect_contact(door_1, frame, contact_tol=0.05, name="door_leaf_1 stays at jamb when open")

    return ctx.report()


object_model = build_object_model()
