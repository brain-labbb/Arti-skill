from __future__ import annotations

# Slide-out concealer palette compact: outer shell sleeve + prismatic drawer.
# Frame: palette lies flat in the XY plane.
#   +X = length (~0.13 m), +Y = depth (~0.06 m), +Z = up (height ~0.018 m).
#   The drawer slides out along +X (the long axis) from the +X opening.
# Parts:
#   - shell (root): hollow rounded-rect sleeve carrying the silver frame,
#     label strip, and an interior ceiling mirror.
#   - drawer: PRISMATIC along +X; dark tray with a 2x4 grid of 8 recessed
#     concealer pans and a front cap that seals the opening when closed.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---- master dimensions ----
LEN_X = 0.130       # palette length (X)
DEP_Y = 0.060       # palette depth (Y)
SHELL_H = 0.018     # shell outer height (Z)
CORNER_R = 0.006    # outer corner radius

# Shell walls
WALL_T = 0.002      # top / bottom wall thickness
WALL_S = 0.002      # side wall thickness (Y)
WALL_B = 0.003      # back wall thickness (-X end)

# Drawer tray
TRAY_H = 0.010      # tray slab height
TRAY_WID = 0.053    # tray width (Y) – 1.5 mm clearance per side in cavity
TRAY_LEN = 0.115    # tray body length (X)
CAP_TH = 0.003      # front cap plate thickness
CAP_WID = 0.055     # cap width (Y) – fits the shell opening
CAP_HGT = 0.013     # cap height (Z) – fits the shell opening

# Derived drawer positions (at q = 0, closed)
TRAY_Z_CENTER = -SHELL_H / 2.0 + WALL_T + TRAY_H / 2.0
CAP_FRONT_X = LEN_X / 2.0                      # flush with shell +X face
CAP_BACK_X = CAP_FRONT_X - CAP_TH              # 0.062
TRAY_FRONT_X = CAP_BACK_X                      # 0.062
TRAY_BACK_X = TRAY_FRONT_X - TRAY_LEN          # −0.053
TRAY_CENTER_X = (TRAY_FRONT_X + TRAY_BACK_X) / 2.0
CAP_CENTER_X = (CAP_FRONT_X + CAP_BACK_X) / 2.0

# Shell cavity (boolean-cut to hollow the sleeve)
CAV_BACK_X = -LEN_X / 2.0 + WALL_B             # −0.062
CAV_FRONT_X = LEN_X / 2.0 + 0.010              # extends past +X face
CAV_X = CAV_FRONT_X - CAV_BACK_X               # 0.137
CAV_Y = DEP_Y - 2.0 * WALL_S                   # 0.056
CAV_Z = SHELL_H - 2.0 * WALL_T                 # 0.014
CAV_CX = (CAV_BACK_X + CAV_FRONT_X) / 2.0

# Pan grid: 2 rows (Y) × 4 cols (X)
PAN_COLS = 4
PAN_ROWS = 2
PAN_SX = 0.022
PAN_SY = 0.018
PAN_DEPTH = 0.004
PAN_GAP_X = 0.003
PAN_GAP_Y = 0.003
GRID_W = PAN_COLS * PAN_SX + (PAN_COLS - 1) * PAN_GAP_X
GRID_D = PAN_ROWS * PAN_SY + (PAN_ROWS - 1) * PAN_GAP_Y
PAN_Z_CENTER = TRAY_Z_CENTER + TRAY_H / 2.0 - PAN_DEPTH / 2.0

# Egg-crate divider on the drawer tray: every pan is framed in its own walled
# cell ("一格一格") instead of sitting flush in one open tray.
PAN_CLEAR = 0.0006      # clearance between a pan and its cell wall
CELL_BORDER = 0.002     # outer frame ring thickness beyond the pan grid
TRAY_TOP_Z = TRAY_Z_CENTER + TRAY_H / 2.0     # tray top surface
CELL_TOP_Z = TRAY_TOP_Z + 0.0015              # cell walls rise above the tray top
CELL_BOT_Z = TRAY_TOP_Z - 0.002               # embed into the tray for bonding

# Pastel / skin-tone palette for the 8 pans
PAN_COLORS = [
    (0.93, 0.78, 0.62, 1.0),  # light beige
    (0.88, 0.70, 0.55, 1.0),  # warm tan
    (0.70, 0.86, 0.70, 1.0),  # green corrector
    (0.92, 0.62, 0.45, 1.0),  # peach corrector
    (0.95, 0.82, 0.68, 1.0),  # fair
    (0.84, 0.74, 0.86, 1.0),  # lavender corrector
    (0.90, 0.66, 0.50, 1.0),  # medium peach
    (0.86, 0.64, 0.42, 1.0),  # deep tan
]

# Prismatic travel
SLIDE_UPPER = 0.080  # max drawer extension (metres)


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _rounded_box(sx: float, sy: float, sz: float, r: float) -> cq.Workplane:
    """Axis-aligned box centred at the origin with filleted vertical (Z) edges."""
    wp = cq.Workplane("XY").box(sx, sy, sz)
    r = min(r, sx / 2.0 - 1e-4, sy / 2.0 - 1e-4)
    if r > 0:
        wp = wp.edges("|Z").fillet(r)
    return wp


def _shell_body_mesh():
    """Hollow rounded-rect sleeve open on the +X end, built additively from
    five wall plates (bottom, top, left side, right side, back) so the
    interior cavity is genuinely empty for collision detection."""
    # Bottom plate – full footprint, thin
    bot = _rounded_box(LEN_X, DEP_Y, WALL_T, CORNER_R)
    bot = bot.translate((0.0, 0.0, -SHELL_H / 2.0 + WALL_T / 2.0))

    # Top plate – full footprint, thin
    top = _rounded_box(LEN_X, DEP_Y, WALL_T, CORNER_R)
    top = top.translate((0.0, 0.0, SHELL_H / 2.0 - WALL_T / 2.0))

    # Side/back walls extend 0.5 mm into each plate for valid boolean union.
    wall_h = SHELL_H - 2.0 * WALL_T + 0.001  # 0.015 m
    side_len = LEN_X  # full length including the opening edge

    # Left side wall (−Y side)
    lwall = cq.Workplane("XY").box(side_len, WALL_S, wall_h)
    lwall = lwall.translate((0.0, -DEP_Y / 2.0 + WALL_S / 2.0, 0.0))

    # Right side wall (+Y side)
    rwall = cq.Workplane("XY").box(side_len, WALL_S, wall_h)
    rwall = rwall.translate((0.0, DEP_Y / 2.0 - WALL_S / 2.0, 0.0))

    # Back wall (−X end) – overlaps side walls by 0.5 mm in Y
    back_w = DEP_Y - WALL_S  # wider than cavity so it overlaps side walls
    bwall = cq.Workplane("XY").box(WALL_B, back_w, wall_h)
    bwall = bwall.translate((-LEN_X / 2.0 + WALL_B / 2.0, 0.0, 0.0))

    # Union all five plates into one connected solid.
    # Each plate shares at least an edge/face with its neighbours, so the
    # boolean union produces one solid shell sleeve.
    body = bot.union(top).union(lwall).union(rwall).union(bwall)
    return mesh_from_cadquery(body, "shell_body")


def _shell_frame_mesh():
    """Thin silver border frame sitting on the top of the shell."""
    frame_h = 0.0012
    embed = 0.0003  # embed slightly into shell top for connectivity
    outer = _rounded_box(LEN_X - 0.001, DEP_Y - 0.001, frame_h, CORNER_R)
    inner = cq.Workplane("XY").box(
        LEN_X - 2.0 * WALL_S - 0.004,
        DEP_Y - 2.0 * WALL_S - 0.004,
        frame_h + 0.010,
    )
    frame = outer.cut(inner)
    frame = frame.translate((0.0, 0.0, SHELL_H / 2.0 + frame_h / 2.0 - embed))
    return mesh_from_cadquery(frame, "shell_frame")


def _label_strip_mesh():
    """White brand-label strip on the shell top, inside the frame border."""
    label_w = 0.040
    label_d = 0.010
    label_h = 0.0008
    embed = 0.0003
    label = _rounded_box(label_w, label_d, label_h, 0.003)
    label = label.translate((-0.020, 0.0, SHELL_H / 2.0 + label_h / 2.0 - embed))
    return mesh_from_cadquery(label, "label_strip")


def _mirror_plate_mesh():
    """Reflective mirror plate seated on the interior ceiling of the shell."""
    mirror_x = 0.080
    mirror_y = CAV_Y - 0.006
    mirror_th = 0.0015
    m = _rounded_box(mirror_x, mirror_y, mirror_th, 0.003)
    # Embed 0.5 mm into the ceiling so it bonds to the shell body
    mirror_z = SHELL_H / 2.0 - WALL_T - mirror_th / 2.0 + 0.0005
    m = m.translate((0.0, 0.0, mirror_z))
    return mesh_from_cadquery(m, "mirror")


def _drawer_body_mesh():
    """Dark tray slab + front cap plate as one connected solid (boolean union
    with a 1 mm overlap region hidden inside the tray front)."""
    # Tray slab
    tray = _rounded_box(TRAY_LEN, TRAY_WID, TRAY_H, 0.003)
    tray = tray.translate((TRAY_CENTER_X, 0.0, TRAY_Z_CENTER))

    # Front cap – extends 1 mm into the tray for a valid boolean union
    overlap = 0.001
    cap_len = CAP_TH + overlap
    cap_cx = CAP_FRONT_X - cap_len / 2.0
    cap = _rounded_box(cap_len, CAP_WID, CAP_HGT, 0.001)
    cap = cap.translate((cap_cx, 0.0, 0.0))

    body = tray.union(cap)
    return mesh_from_cadquery(body, "drawer_body")


def _pan_mesh():
    """A single rounded concealer-cream pan (shared geometry helper)."""
    pan = _rounded_box(PAN_SX, PAN_SY, PAN_DEPTH, 0.004)
    return mesh_from_cadquery(pan, "pan_cream")


def _pan_centers():
    """World-XY centres of the 8 pans, grid centred on the tray."""
    centers = []
    grid_cx = TRAY_CENTER_X
    grid_cy = 0.0
    x0 = grid_cx - GRID_W / 2.0 + PAN_SX / 2.0
    y0 = grid_cy - GRID_D / 2.0 + PAN_SY / 2.0
    for row in range(PAN_ROWS):
        for col in range(PAN_COLS):
            cx = x0 + col * (PAN_SX + PAN_GAP_X)
            cy = y0 + row * (PAN_SY + PAN_GAP_Y)
            centers.append((row, col, cx, cy))
    return centers


def _cell_grid_mesh():
    """Silver egg-crate divider riding on the drawer tray: a block covering the
    pan grid (plus a thin outer border) with one pocket cut per pan. The walls
    left between pockets frame every pan in its own cell, rising just above the
    pan tops so none sits flush/proud in an open tray."""
    centers = _pan_centers()
    xs = [cx for (_, _, cx, _) in centers]
    ys = [cy for (_, _, _, cy) in centers]
    block_cx = (min(xs) + max(xs)) / 2.0
    block_cy = (min(ys) + max(ys)) / 2.0
    block_x = (max(xs) - min(xs)) + PAN_SX + 2.0 * CELL_BORDER
    block_y = (max(ys) - min(ys)) + PAN_SY + 2.0 * CELL_BORDER
    block_h = CELL_TOP_Z - CELL_BOT_Z
    block_cz = CELL_BOT_Z + block_h / 2.0
    block = _rounded_box(block_x, block_y, block_h, 0.0015)
    block = block.translate((block_cx, block_cy, block_cz))
    # Cut one through-pocket per pan (pan footprint + clearance).
    pocket_h = block_h + 0.01
    for (row, col, cx, cy) in centers:
        pocket = _rounded_box(
            PAN_SX + 2.0 * PAN_CLEAR, PAN_SY + 2.0 * PAN_CLEAR, pocket_h, 0.0015
        )
        pocket = pocket.translate((cx, cy, block_cz))
        block = block.cut(pocket)
    return mesh_from_cadquery(block, "cell_grid")


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="concealer_palette_slide_drawer")

    dark = model.material("case_dark", rgba=(0.10, 0.10, 0.12, 1.0))
    silver = model.material("frame_silver", rgba=(0.78, 0.80, 0.83, 1.0))
    mirror_mat = model.material("mirror_glass", rgba=(0.82, 0.88, 0.92, 1.0))
    label_mat = model.material("label_white", rgba=(0.95, 0.95, 0.96, 1.0))
    pan_mats = [
        model.material(f"pan_mat_{i}", rgba=c) for i, c in enumerate(PAN_COLORS)
    ]

    # =================================================================
    # SHELL (root): hollow sleeve + silver frame + label + mirror
    # =================================================================
    shell = model.part("shell")
    shell.visual(_shell_body_mesh(), material=dark, name="shell_body")
    shell.visual(_shell_frame_mesh(), material=silver, name="shell_frame")
    shell.visual(_label_strip_mesh(), material=label_mat, name="label_strip")
    shell.visual(_mirror_plate_mesh(), material=mirror_mat, name="mirror")

    shell.inertial = Inertial.from_geometry(
        Box((LEN_X, DEP_Y, SHELL_H)),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # =================================================================
    # DRAWER (child): dark tray + 8 pans + front cap grip accent
    # =================================================================
    drawer = model.part("drawer")
    drawer.visual(_drawer_body_mesh(), material=dark, name="drawer_body")

    # Silver pull-grip accent on the front cap face
    grip_th = 0.0008
    grip = _rounded_box(grip_th, 0.025, 0.003, 0.001)
    grip = grip.translate((CAP_FRONT_X + grip_th / 2.0, 0.0, 0.002))
    drawer.visual(mesh_from_cadquery(grip, "grip_accent"), material=silver, name="grip_accent")

    # Silver egg-crate divider: one walled cell per pan.
    drawer.visual(_cell_grid_mesh(), material=silver, name="cell_grid")

    # 8 concealer pans recessed into their cells (loop + shared helper)
    for (row, col, cx, cy) in _pan_centers():
        idx = row * PAN_COLS + col
        drawer.visual(
            _pan_mesh(),
            origin=Origin(xyz=(cx, cy, PAN_Z_CENTER)),
            material=pan_mats[idx],
            name=f"pan_{row}_{col}",
        )

    drawer.inertial = Inertial.from_geometry(
        Box((TRAY_LEN + CAP_TH, TRAY_WID, TRAY_H)),
        mass=0.05,
        origin=Origin(xyz=(TRAY_CENTER_X, 0.0, TRAY_Z_CENTER)),
    )

    # =================================================================
    # PRISMATIC joint: drawer slides out along +X
    # =================================================================
    model.articulation(
        "shell_to_drawer",
        ArticulationType.PRISMATIC,
        parent=shell,
        child=drawer,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=0.30,
            lower=0.0,
            upper=SLIDE_UPPER,
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    shell = object_model.get_part("shell")
    drawer = object_model.get_part("drawer")
    slide = object_model.get_articulation("shell_to_drawer")

    # ---- joint type ----
    ctx.check(
        "prismatic joint for drawer slide",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"got {slide.articulation_type}",
    )

    # ---- 8 concealer pans present ----
    pan_names = [f"pan_{r}_{c}" for r in range(PAN_ROWS) for c in range(PAN_COLS)]
    ctx.check(
        "eight concealer pans authored",
        len(pan_names) == 8 and all(drawer.get_visual(n) is not None for n in pan_names),
        details=f"pans={pan_names}",
    )

    # Pans sit within the drawer body footprint in XY
    for n in pan_names:
        ctx.expect_within(
            drawer, drawer, axes="xy",
            inner_elem=n, outer_elem="drawer_body",
            margin=0.002,
            name=f"{n} within drawer body",
        )

    # ---- egg-crate cell frame: every pan is walled into its own cell ----
    ctx.check(
        "pan cell grid authored",
        drawer.get_visual("cell_grid") is not None,
        details="cell_grid visual missing",
    )
    cell_aabb = ctx.part_element_world_aabb(drawer, elem="cell_grid")
    cell_top = cell_aabb[1][2] if cell_aabb else None
    for n in pan_names:
        ctx.expect_within(
            drawer, drawer, axes="xy",
            inner_elem=n, outer_elem="cell_grid",
            margin=0.0, name=f"{n} framed within cell grid",
        )
        p_aabb = ctx.part_element_world_aabb(drawer, elem=n)
        ctx.check(
            f"{n} recessed below cell rim (not protruding)",
            p_aabb is not None and cell_top is not None
            and p_aabb[1][2] <= cell_top + 0.0002,
            details=f"pan_top={p_aabb[1][2] if p_aabb else None}, cell_top={cell_top}",
        )

    # ---- mirror on shell interior ceiling ----
    ctx.check(
        "mirror authored on shell",
        shell.get_visual("mirror") is not None,
        details="mirror visual missing",
    )
    mir_aabb = ctx.part_element_world_aabb(shell, elem="mirror")
    body_aabb = ctx.part_element_world_aabb(shell, elem="shell_body")
    ctx.check(
        "mirror on shell interior ceiling (below shell top)",
        mir_aabb is not None
        and body_aabb is not None
        and mir_aabb[1][2] < body_aabb[1][2] - 0.001,
        details=(
            f"mirror_top={mir_aabb[1][2] if mir_aabb else None}, "
            f"body_top={body_aabb[1][2] if body_aabb else None}"
        ),
    )

    # ---- silver frame on shell ----
    ctx.check(
        "silver frame on shell top",
        shell.get_visual("shell_frame") is not None,
        details="shell_frame visual missing",
    )
    frame_aabb = ctx.part_element_world_aabb(shell, elem="shell_frame")
    ctx.check(
        "shell frame sits above shell body top",
        frame_aabb is not None and body_aabb is not None
        and frame_aabb[1][2] > body_aabb[1][2] - 0.0005,
        details=(
            f"frame_top={frame_aabb[1][2] if frame_aabb else None}, "
            f"body_top={body_aabb[1][2] if body_aabb else None}"
        ),
    )

    # ---- label strip present ----
    ctx.check(
        "label strip authored on shell",
        shell.get_visual("label_strip") is not None,
        details="label_strip visual missing",
    )

    # ---- nested drawer-in-sleeve allowance ----
    # The drawer slides inside the hollow shell sleeve. The collision mesh
    # treats the shell as a filled volume, so scoped allow_overlap is needed
    # for the specific sleeve/member interface.
    ctx.allow_overlap(
        drawer, shell,
        elem_a="drawer_body", elem_b="shell_body",
        reason="Drawer body slides inside the hollow shell sleeve; the sleeve is modeled as wall plates but the collision mesh treats the shell volume as filled.",
    )
    ctx.allow_overlap(
        drawer, shell,
        elem_a="cell_grid", elem_b="shell_body",
        reason="The pan cell-divider grid rides inside the hollow shell sleeve; the sleeve is modeled as wall plates but the collision mesh treats the shell volume as filled.",
    )

    # ---- at rest: drawer inside shell (proof checks) ----
    ctx.expect_within(
        drawer, shell, axes="xy",
        inner_elem="drawer_body", outer_elem="shell_body",
        margin=0.005,
        name="drawer within shell footprint at rest",
    )
    ctx.expect_overlap(
        drawer, shell, axes="z",
        elem_a="drawer_body", elem_b="shell_body",
        min_overlap=0.005,
        name="drawer inside shell height at rest",
    )
    ctx.expect_overlap(
        drawer, shell, axes="x",
        elem_a="drawer_body", elem_b="shell_body",
        min_overlap=0.080,
        name="drawer retained insertion in shell at rest",
    )

    # ---- drawer slides out along +X ----
    rest_pos = ctx.part_world_position(drawer)
    with ctx.pose({slide: SLIDE_UPPER}):
        ext_pos = ctx.part_world_position(drawer)
    ctx.check(
        "drawer slides outward along +X",
        rest_pos is not None
        and ext_pos is not None
        and ext_pos[0] > rest_pos[0] + 0.05,
        details=f"rest_x={rest_pos}, ext_x={ext_pos}",
    )

    # At max extension the drawer tray back is still inside the shell
    with ctx.pose({slide: SLIDE_UPPER}):
        drawer_aabb = ctx.part_element_world_aabb(drawer, elem="drawer_body")
        shell_aabb = ctx.part_world_aabb(shell)
    ctx.check(
        "drawer retains insertion at max extension",
        drawer_aabb is not None
        and shell_aabb is not None
        and drawer_aabb[0][0] < shell_aabb[1][0] - 0.010,
        details=(
            f"drawer_back_x={drawer_aabb[0][0] if drawer_aabb else None}, "
            f"shell_front_x={shell_aabb[1][0] if shell_aabb else None}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
