from __future__ import annotations

# Concealer makeup palette compact with a hinged mirror lid AND a front-edge
# spring-clasp latch.  Forked from the rear-hinge parent: the rear revolute
# hinge is preserved, but a real movable clasp catch part now swings on a small
# revolute joint at the front edge to lock the lid closed.
#
# Frame: palette lies flat in the XY plane.
#   +X = length (~0.13 m), +Y = depth (~0.06 m), +Z = up (thickness ~0.015 m).
#   The rear hinge runs along the +Y edge; the front clasp sits on the -Y edge.
#
# Parts:
#   - base (root): dark tray with silver frame, 2x4 pan grid, label strip,
#     hinge knuckles, and two clasp-mount bosses at the front edge.
#   - lid: REVOLUTE about the rear hinge; silver frame + clear window + mirror.
#     A small catch lip on the lid front edge engages the clasp hook.
#   - clasp: REVOLUTE about the front-edge pivot; silver lever arm with a hook
#     at the top and a push-button grip at mid-height.  Swings forward/down to
#     release the lid (unlatch), or upward to catch the lid lip (latch).

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- master dimensions (closed pose) ----
LEN_X = 0.130
DEP_Y = 0.060
BASE_H = 0.009  # base tray thickness
LID_H = 0.006  # lid slab thickness

WALL = 0.004  # outer wall thickness of the base tray
FLOOR_TOP = 0.0025  # z of the tray interior floor (above the bottom slab)
SLAB_WELL_TOP = -LID_H / 2.0 + 0.002

# Pan grid: 2 rows (along Y) x 4 cols (along X)
PAN_COLS = 4
PAN_ROWS = 2
PAN_SX = 0.024
PAN_SY = 0.020
PAN_DEPTH = 0.0058
PAN_GAP_X = 0.0035
PAN_GAP_Y = 0.0035

GRID_W = PAN_COLS * PAN_SX + (PAN_COLS - 1) * PAN_GAP_X
GRID_D = PAN_ROWS * PAN_SY + (PAN_ROWS - 1) * PAN_GAP_Y

# Egg-crate divider: every pan is framed in its own walled cell ("一格一格")
# instead of standing proud in one open well.
PAN_CLEAR = 0.0006      # clearance between a pan and its cell wall
CELL_BORDER = 0.002     # outer frame ring thickness beyond the pan grid
CELL_TOP_Z = BASE_H     # cell walls rise to the tray rim
CELL_BOT_Z = FLOOR_TOP - 0.0005  # embed slightly into the tray floor
CELL_POCKET_FLOOR = 0.0008  # solid floor left under each pan pocket

# Hinge along rear long edge.
HINGE_Y = DEP_Y / 2.0
HINGE_Z = BASE_H + LID_H / 2.0

# ---- front clasp dimensions ----
CLASP_W = 0.016  # arm width (X)
CLASP_ARM_H = 0.008  # arm height from pivot
CLASP_ARM_TH = 0.002  # arm plate thickness (Y)
CLASP_HOOK_LEN = 0.005  # hook extends rearward (+Y) from arm top
CLASP_HOOK_TH = 0.002  # hook plate thickness (Z)
CLASP_PIVOT_R = 0.0010  # pivot barrel radius
# Pivot sits just above the base-frame top so the barrel clears the frame.
CLASP_PIVOT_Z = BASE_H + 0.003  # 0.012 m (= HINGE_Z)
CLASP_PIVOT_Y = -DEP_Y / 2.0  # front edge

# Lid catch lip on the front-top edge of the lid.
LID_LIP_W = 0.018
LID_LIP_DEPTH = 0.003
LID_LIP_H = 0.003

# Pastel / skin tone palette for the 8 pans.
PAN_COLORS = [
    (0.93, 0.78, 0.62, 1.0),
    (0.88, 0.70, 0.55, 1.0),
    (0.70, 0.86, 0.70, 1.0),
    (0.92, 0.62, 0.45, 1.0),
    (0.95, 0.82, 0.68, 1.0),
    (0.84, 0.74, 0.86, 1.0),
    (0.90, 0.66, 0.50, 1.0),
    (0.86, 0.64, 0.42, 1.0),
]


# ---------------------------------------------------------------------------
# geometry helpers
# ---------------------------------------------------------------------------

def _rounded_box(sx: float, sy: float, sz: float, r: float) -> cq.Workplane:
    """Axis-aligned box centered at origin with filleted vertical (Z) edges."""
    wp = cq.Workplane("XY").box(sx, sy, sz)
    r = min(r, sx / 2.0 - 1e-4, sy / 2.0 - 1e-4)
    if r > 0:
        wp = wp.edges("|Z").fillet(r)
    return wp


def _base_tray_mesh():
    """Dark tray: solid slab with a shallow top cavity that holds the pans."""
    outer = _rounded_box(LEN_X, DEP_Y, BASE_H, 0.006)
    outer = outer.translate((0.0, 0.0, BASE_H / 2.0))

    cav_x = LEN_X - 2.0 * WALL
    cav_y = DEP_Y - 2.0 * WALL
    cav_h = BASE_H - FLOOR_TOP
    cavity = _rounded_box(cav_x, cav_y, cav_h + 0.02, 0.004)
    cavity = cavity.translate((0.0, 0.0, FLOOR_TOP + (cav_h + 0.02) / 2.0))
    tray = outer.cut(cavity)
    return mesh_from_cadquery(tray, "base_tray")


def _pan_centers():
    """World-XY centers of the 8 pans.  Grid sits toward the front (-Y)."""
    centers = []
    grid_cx = 0.0
    grid_cy = -0.006
    x0 = grid_cx - GRID_W / 2.0 + PAN_SX / 2.0
    y0 = grid_cy - GRID_D / 2.0 + PAN_SY / 2.0
    for row in range(PAN_ROWS):
        for col in range(PAN_COLS):
            cx = x0 + col * (PAN_SX + PAN_GAP_X)
            cy = y0 + row * (PAN_SY + PAN_GAP_Y)
            centers.append((row, col, cx, cy))
    return centers


def _pan_mesh():
    """A single rounded concealer pan: shallow rounded slab of cream."""
    pan = _rounded_box(PAN_SX, PAN_SY, PAN_DEPTH, 0.004)
    return mesh_from_cadquery(pan, "pan_cream")


def _cell_grid_mesh():
    """Silver egg-crate divider: a block covering the pan grid (plus a thin outer
    border) with one recessed pocket cut per pan. The walls left between pockets
    frame every concealer pan in its own cell so none stands proud of the tray."""
    centers = _pan_centers()
    xs = [cx for (_, _, cx, _) in centers]
    ys = [cy for (_, _, _, cy) in centers]
    block_cx = (min(xs) + max(xs)) / 2.0
    block_cy = (min(ys) + max(ys)) / 2.0
    block_x = (max(xs) - min(xs)) + PAN_SX + 2.0 * CELL_BORDER
    block_y = (max(ys) - min(ys)) + PAN_SY + 2.0 * CELL_BORDER
    block_h = CELL_TOP_Z - CELL_BOT_Z
    block = _rounded_box(block_x, block_y, block_h, 0.0015)
    block = block.translate((block_cx, block_cy, CELL_BOT_Z + block_h / 2.0))
    # Cut one pocket per pan (pan footprint + clearance), open at the top and
    # leaving a thin floor the pan seats on.
    pocket_floor_z = FLOOR_TOP + CELL_POCKET_FLOOR
    pocket_h = (CELL_TOP_Z - pocket_floor_z) + 0.01
    for (row, col, cx, cy) in centers:
        pocket = _rounded_box(
            PAN_SX + 2.0 * PAN_CLEAR, PAN_SY + 2.0 * PAN_CLEAR, pocket_h, 0.0015
        )
        pocket = pocket.translate((cx, cy, pocket_floor_z + pocket_h / 2.0))
        block = block.cut(pocket)
    return mesh_from_cadquery(block, "cell_grid")


def _label_strip_mesh():
    """Raised glossy label shelf along the rear edge."""
    strip_h = BASE_H - FLOOR_TOP + 0.0014
    strip_y = DEP_Y / 2.0 - WALL - 0.0055
    strip = _rounded_box(LEN_X - 2.0 * WALL - 0.004, 0.011, strip_h, 0.0008)
    strip = strip.translate((0.0, strip_y, FLOOR_TOP + strip_h / 2.0))
    return mesh_from_cadquery(strip, "label_strip")


def _lid_slab_mesh():
    """Lid slab with a shallow well on the inner face for the mirror."""
    slab = _rounded_box(LEN_X, DEP_Y, LID_H, 0.006)
    well_h = (SLAB_WELL_TOP - (-LID_H / 2.0)) + 0.01
    well = _rounded_box(LEN_X - 2.0 * WALL, DEP_Y - 2.0 * WALL, well_h, 0.004)
    well = well.translate((0.0, 0.0, SLAB_WELL_TOP - well_h / 2.0))
    slab = slab.cut(well)
    return mesh_from_cadquery(slab, "lid_slab")


def _lid_lip_mesh():
    """Small catch lip/ridge on the lid front-top edge.  The clasp hook
    catches behind this lip to latch the compact closed.

    Built in the lid part frame (origin at hinge).  The lid slab front face
    is at y = lid_y - DEP_Y/2 = -DEP_Y.  The lid top is at z = LID_H.
    The lip sits just behind the front face, protruding upward from the lid
    top surface so the clasp hook can engage it from above."""
    lid_y = -DEP_Y / 2.0
    # Lip center in lid part frame:
    lip_cy = lid_y - DEP_Y / 2.0 + LID_LIP_DEPTH / 2.0 + 0.001
    lip_cz = LID_H + LID_LIP_H / 2.0
    lip = _rounded_box(LID_LIP_W, LID_LIP_DEPTH, LID_LIP_H, 0.0006)
    lip = lip.translate((0.0, lip_cy, lip_cz))
    return mesh_from_cadquery(lip, "lid_lip")


def _clasp_lever_mesh():
    """Spring-clasp lever: arm plate + hook + pivot barrel + push-button grip.
    Built in the clasp part frame (origin at pivot axis).  At q=0 (latched)
    the arm extends upward (+Z) and the hook extends rearward (+Y)."""
    # Main arm plate – slightly forward of the pivot so the inner face is
    # approximately at y=0 (flush with the base front face in world).
    arm = _rounded_box(CLASP_W, CLASP_ARM_TH, CLASP_ARM_H, 0.0006)
    arm = arm.translate((0.0, -CLASP_ARM_TH / 2.0, CLASP_ARM_H / 2.0))

    # Pivot barrel at the bottom (solid cylinder along X).
    barrel_r = CLASP_PIVOT_R + 0.0005
    barrel = (
        cq.Workplane("YZ")
        .circle(barrel_r)
        .extrude(CLASP_W + 0.002)
    )
    barrel = barrel.translate((-(CLASP_W + 0.002) / 2.0, 0.0, 0.0))

    # Hook at the top: extends rearward (+Y) to catch the lid lip.
    hook = _rounded_box(
        CLASP_W - 0.002, CLASP_HOOK_LEN, CLASP_HOOK_TH, 0.0004
    )
    hook = hook.translate(
        (0.0, CLASP_HOOK_LEN / 2.0, CLASP_ARM_H - CLASP_HOOK_TH / 2.0)
    )

    # Push-button grip on the outer face (-Y) at mid-arm height.
    btn_w, btn_d, btn_h = 0.008, 0.003, 0.004
    button = _rounded_box(btn_w, btn_d, btn_h, 0.001)
    button = button.translate(
        (0.0, -CLASP_ARM_TH - btn_d / 2.0, CLASP_ARM_H * 0.38)
    )

    clasp = arm.union(barrel).union(hook).union(button)
    return mesh_from_cadquery(clasp, "clasp_lever")


# ---------------------------------------------------------------------------
# build
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="concealer_palette_clasp_latch")

    dark = model.material("case_dark", rgba=(0.10, 0.10, 0.12, 1.0))
    silver = model.material("frame_silver", rgba=(0.78, 0.80, 0.83, 1.0))
    mirror_mat = model.material("mirror_glass", rgba=(0.82, 0.88, 0.92, 1.0))
    clear = model.material("lid_clear", rgba=(0.86, 0.90, 0.93, 0.55))
    label_mat = model.material("label_white", rgba=(0.95, 0.95, 0.96, 1.0))
    clasp_mat = model.material("clasp_silver", rgba=(0.72, 0.74, 0.78, 1.0))

    pan_mats = [
        model.material(f"pan_mat_{i}", rgba=c) for i, c in enumerate(PAN_COLORS)
    ]

    # =====================================================================
    # BASE (root)
    # =====================================================================
    base = model.part("base")
    base.visual(_base_tray_mesh(), material=dark, name="base_tray")

    # Silver rim frame around the top of the tray.
    frame = _rounded_box(LEN_X, DEP_Y, 0.0018, 0.006)
    frame_inner = _rounded_box(LEN_X - 2.0 * WALL, DEP_Y - 2.0 * WALL, 0.01, 0.004)
    frame = frame.cut(frame_inner)
    frame = frame.translate((0.0, 0.0, BASE_H + 0.0009))
    base.visual(mesh_from_cadquery(frame, "base_frame"), material=silver, name="base_frame")

    # Raised label shelf along the rear edge.
    # Silver egg-crate divider: one walled cell per pan.
    base.visual(_cell_grid_mesh(), material=silver, name="cell_grid")

    base.visual(_label_strip_mesh(), material=label_mat, name="label_strip")

    # 8 concealer pans recessed into the tray cavity.
    pan_center_z = FLOOR_TOP + CELL_POCKET_FLOOR - 0.0005 + PAN_DEPTH / 2.0
    for (row, col, cx, cy) in _pan_centers():
        idx = row * PAN_COLS + col
        base.visual(
            _pan_mesh(),
            origin=Origin(xyz=(cx, cy, pan_center_z)),
            material=pan_mats[idx],
            name=f"pan_{row}_{col}",
        )

    base.inertial = Inertial.from_geometry(
        Box((LEN_X, DEP_Y, BASE_H)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, BASE_H / 2.0)),
    )

    # Visible hinge barrels along the rear edge (two silver knuckles).
    for i, hx in enumerate((-0.030, 0.030)):
        knuckle = CylinderGeometry(0.0022, 0.018, radial_segments=24)
        knuckle = knuckle.rotate_y(math.pi / 2.0)
        knuckle = knuckle.translate(hx, HINGE_Y, HINGE_Z)
        base.visual(
            mesh_from_geometry(knuckle, f"hinge_knuckle_{i}"),
            material=silver,
            name=f"hinge_knuckle_{i}",
        )

    # Clasp-mount bosses: two small cylindrical ears on the base front edge
    # that the clasp lever pivots between.
    boss_r = 0.0020
    boss_len = 0.005
    boss_offset = CLASP_W / 2.0 + boss_len / 2.0 + 0.001
    for i, bx in enumerate((-boss_offset, boss_offset)):
        boss = CylinderGeometry(boss_r, boss_len, radial_segments=16)
        boss = boss.rotate_y(math.pi / 2.0)
        boss = boss.translate(bx, CLASP_PIVOT_Y, CLASP_PIVOT_Z)
        base.visual(
            mesh_from_geometry(boss, f"clasp_boss_{i}"),
            material=silver,
            name=f"clasp_boss_{i}",
        )

    # =====================================================================
    # LID: revolute about the rear hinge
    # =====================================================================
    lid = model.part("lid")
    lid_x = 0.0
    lid_y = -DEP_Y / 2.0
    lid_z = LID_H / 2.0

    lid.visual(
        _lid_slab_mesh(),
        origin=Origin(xyz=(lid_x, lid_y, lid_z)),
        material=clear,
        name="lid_slab",
    )

    # Silver frame border on the lid.
    lid_frame = _rounded_box(LEN_X, DEP_Y, 0.0018, 0.006)
    lid_frame_inner = _rounded_box(LEN_X - 2.0 * WALL, DEP_Y - 2.0 * WALL, 0.01, 0.004)
    lid_frame = lid_frame.cut(lid_frame_inner)
    lid_frame = lid_frame.translate((0.0, lid_y, LID_H - 0.0009 + 1e-4))
    lid.visual(
        mesh_from_cadquery(lid_frame, "lid_frame"),
        material=silver,
        name="lid_frame",
    )

    # Mirror on the inner (lower, -Z) face of the lid.
    mirror_th = 0.0020
    well_floor_z = lid_z + SLAB_WELL_TOP
    mirror_plate = _rounded_box(
        LEN_X - 2.0 * WALL - 0.001,
        DEP_Y - 2.0 * WALL - 0.001,
        mirror_th,
        0.003,
    )
    mirror_plate = mirror_plate.translate(
        (0.0, lid_y, well_floor_z - mirror_th / 2.0 + 0.0006)
    )
    lid.visual(
        mesh_from_cadquery(mirror_plate, "mirror"),
        material=mirror_mat,
        name="mirror",
    )

    # Catch lip on the lid front-top edge for the clasp hook.
    lid.visual(_lid_lip_mesh(), material=silver, name="lid_lip")

    lid.inertial = Inertial.from_geometry(
        Box((LEN_X, DEP_Y, LID_H)),
        mass=0.04,
        origin=Origin(xyz=(0.0, lid_y, 0.0)),
    )

    # =====================================================================
    # CLASP: revolute about the front-edge pivot
    # =====================================================================
    clasp = model.part("clasp")
    # The clasp part frame origin sits at the pivot axis.  The lever mesh is
    # built with its pivot barrel centered at local origin, arm extending +Z.
    clasp.visual(_clasp_lever_mesh(), material=clasp_mat, name="clasp_lever")

    clasp.inertial = Inertial.from_geometry(
        Box((CLASP_W, CLASP_ARM_TH + CLASP_HOOK_LEN, CLASP_ARM_H)),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, CLASP_ARM_H / 2.0)),
    )

    # =====================================================================
    # ARTICULATIONS
    # =====================================================================

    # Rear hinge: axis along -X.  Positive rotation lifts the front edge up.
    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )

    # Front clasp: axis along +X at the front-edge pivot.  Positive rotation
    # (right-hand rule about +X) swings the arm top from +Z toward -Y, i.e.
    # forward and down – this is the unlatch direction.
    #   q=0  → latched (arm vertical, hook catches lid lip)
    #   q>0  → unlatched (arm swings forward/down, hook clears lid)
    model.articulation(
        "base_to_clasp",
        ArticulationType.REVOLUTE,
        parent=base,
        child=clasp,
        origin=Origin(xyz=(0.0, CLASP_PIVOT_Y, CLASP_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=4.0,
            lower=0.0,
            upper=math.radians(85.0),
        ),
    )

    return model


# ---------------------------------------------------------------------------
# tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    clasp = object_model.get_part("clasp")
    hinge = object_model.get_articulation("base_to_lid")
    latch = object_model.get_articulation("base_to_clasp")

    # ---- 8 concealer pans present and seated ----
    pan_names = [f"pan_{r}_{c}" for r in range(PAN_ROWS) for c in range(PAN_COLS)]
    ctx.check(
        "eight concealer pans authored",
        len(pan_names) == 8 and all(base.get_visual(n) is not None for n in pan_names),
        details=f"pans={pan_names}",
    )
    for n in pan_names:
        ctx.expect_within(
            base, base, axes="xy", inner_elem=n, outer_elem="base_tray",
            margin=0.001, name=f"{n} within tray footprint",
        )

    # ---- egg-crate cell frame: every pan is walled into its own cell ----
    ctx.check(
        "pan cell grid authored",
        base.get_visual("cell_grid") is not None,
        details="cell_grid visual missing",
    )
    cell_aabb = ctx.part_element_world_aabb(base, elem="cell_grid")
    cell_top = cell_aabb[1][2] if cell_aabb else None
    for n in pan_names:
        ctx.expect_within(
            base, base, axes="xy", inner_elem=n, outer_elem="cell_grid",
            margin=0.0, name=f"{n} framed within cell grid",
        )
        p_aabb = ctx.part_element_world_aabb(base, elem=n)
        ctx.check(
            f"{n} recessed below cell rim (not protruding)",
            p_aabb is not None and cell_top is not None
            and p_aabb[1][2] <= cell_top + 0.0002,
            details=f"pan_top={p_aabb[1][2] if p_aabb else None}, cell_top={cell_top}",
        )

    # ---- mirror on the lid inner face ----
    ctx.check(
        "mirror authored on lid",
        lid.get_visual("mirror") is not None,
        details="mirror visual missing",
    )
    mir_aabb = ctx.part_element_world_aabb(lid, elem="mirror")
    slab_aabb = ctx.part_element_world_aabb(lid, elem="lid_slab")
    ctx.check(
        "mirror on inner face of lid",
        mir_aabb is not None and slab_aabb is not None
        and mir_aabb[1][2] < slab_aabb[1][2] - 0.001,
        details=f"mirror_top={mir_aabb[1][2] if mir_aabb else None}, "
                f"slab_top={slab_aabb[1][2] if slab_aabb else None}",
    )

    # ---- closed lid covers the base ----
    ctx.expect_overlap(
        lid, base, axes="xy", min_overlap=0.05,
        name="closed lid covers base footprint",
    )
    ctx.expect_gap(
        lid, base, axis="z", positive_elem="lid_slab", negative_elem="base_frame",
        min_gap=-0.001, name="closed lid rests above base",
    )

    # ---- lid opens upward about the rear hinge ----
    closed_aabb = ctx.part_world_aabb(lid)
    closed_top_z = closed_aabb[1][2]
    closed_front_y = closed_aabb[0][1]
    with ctx.pose({hinge: math.radians(100.0)}):
        open_aabb = ctx.part_world_aabb(lid)
        open_top_z = open_aabb[1][2]
        open_front_y = open_aabb[0][1]
    ctx.check(
        "lid opens upward about rear hinge",
        open_top_z > closed_top_z + 0.02,
        details=f"closed_top_z={closed_top_z}, open_top_z={open_top_z}",
    )
    ctx.check(
        "lid front edge swings back toward hinge",
        open_front_y > closed_front_y + 0.01,
        details=f"closed_front_y={closed_front_y}, open_front_y={open_front_y}",
    )

    # ---- hinge knuckles on base ----
    ctx.check(
        "hinge knuckles authored on base",
        base.get_visual("hinge_knuckle_0") is not None
        and base.get_visual("hinge_knuckle_1") is not None,
        details="missing hinge knuckles",
    )

    # ---- clasp part and joint exist ----
    ctx.check(
        "clasp part authored",
        clasp is not None and clasp.get_visual("clasp_lever") is not None,
        details="clasp part or lever visual missing",
    )
    ctx.check(
        "clasp articulation is revolute",
        latch.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={latch.articulation_type}",
    )

    # ---- clasp mount bosses on base front edge ----
    ctx.check(
        "clasp mount bosses authored on base",
        base.get_visual("clasp_boss_0") is not None
        and base.get_visual("clasp_boss_1") is not None,
        details="missing clasp mount bosses",
    )

    # ---- lid catch lip authored ----
    ctx.check(
        "lid catch lip authored",
        lid.get_visual("lid_lip") is not None,
        details="lid_lip visual missing",
    )

    # ---- latched (q=0): clasp hook engages lid lip (XY overlap) ----
    ctx.expect_overlap(
        clasp, lid, axes="xy",
        elem_a="clasp_lever", elem_b="lid_lip",
        min_overlap=0.002,
        name="clasp hook engages lid lip when latched",
    )
    # The hook and lip occupy the same Z band when latched (hook wraps over lip).
    ctx.expect_overlap(
        clasp, lid, axes="z",
        elem_a="clasp_lever", elem_b="lid_lip",
        min_overlap=0.001,
        name="clasp hook overlaps lid lip height when latched",
    )

    # ---- unlatched: clasp hook swings clear of the lid lip ----
    with ctx.pose({latch: math.radians(75.0)}):
        # The clasp arm has swung forward (-Y) and down; the hook should be
        # well clear of the lid lip in Y (clasp is forward of lip).
        clasp_aabb = ctx.part_element_world_aabb(clasp, elem="clasp_lever")
        lip_aabb = ctx.part_element_world_aabb(lid, elem="lid_lip")
        ctx.check(
            "clasp hook clears lid lip when unlatched",
            clasp_aabb is not None and lip_aabb is not None
            and clasp_aabb[1][2] < lip_aabb[0][2] + 0.001,
            details=f"clasp_top_z={clasp_aabb[1][2] if clasp_aabb else None}, "
                    f"lip_bottom_z={lip_aabb[0][2] if lip_aabb else None}",
        )

    # ---- lid opens freely when clasp is unlatched ----
    with ctx.pose({latch: math.radians(75.0), hinge: math.radians(90.0)}):
        open_lid_aabb = ctx.part_world_aabb(lid)
        ctx.check(
            "lid opens freely when clasp is unlatched",
            open_lid_aabb is not None
            and open_lid_aabb[1][2] > BASE_H + LID_H + 0.02,
            details=f"lid_top_z={open_lid_aabb[1][2] if open_lid_aabb else None}",
        )

    # ---- intentional overlap allowances ----
    # Lid frame seats flush onto the base rim when closed.
    ctx.allow_overlap(
        lid, base,
        elem_a="lid_frame", elem_b="base_frame",
        reason="Lid silver frame seats flush onto the base silver rim when closed.",
    )
    # Clasp hook intentionally catches over the lid lip to latch the compact.
    ctx.allow_overlap(
        clasp, lid,
        elem_a="clasp_lever", elem_b="lid_lip",
        reason="Clasp hook intentionally catches over the lid lip to latch the compact closed.",
    )
    # Clasp pivot barrel sits at the base/lid front-edge junction; the barrel
    # wraps around the pivot pin and slightly embeds into the lid slab front
    # face where the two shells meet at the clasp mount.
    ctx.allow_overlap(
        clasp, lid,
        elem_a="clasp_lever", elem_b="lid_slab",
        reason="Clasp pivot barrel sits at the front-edge junction where the lid slab meets the base; small barrel embed into the slab front face is the mount interface.",
    )

    return ctx.report()


object_model = build_object_model()
