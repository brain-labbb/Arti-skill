from __future__ import annotations

# Concealer makeup palette compact with a hinged mirror lid.
# Frame: palette lies flat in the XY plane.
#   +X = length (~0.13 m), +Y = depth (~0.06 m), +Z = up (thickness ~0.015 m).
#   The hinge runs along the REAR long edge (+Y side); the lid closes down onto
#   the base (-Z) and opens by swinging the FRONT edge up and back about that
#   rear hinge axis (axis parallel to +X).
# Parts:
#   - base (root): dark tray with a silver frame, a silver egg-crate divider
#     holding a 2x4 grid of 8 recessed concealer pans (each framed in its own
#     cell) in pastel/skin tones, and a raised "CONCEALER" label strip.
#   - lid: REVOLUTE about the rear hinge; silver frame + clear top window and a
#     reflective mirror on its inner (lower) face. Opens 0 -> ~110 deg.

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
# Slab-local z of the inner well floor (remaining slab) the mirror seats against.
# The slab is built centered at z=0; the well cut leaves solid above z=SLAB_WELL_TOP.
SLAB_WELL_TOP = -LID_H / 2.0 + 0.002

# Pan grid: 2 rows (along Y) x 4 cols (along X)
PAN_COLS = 4
PAN_ROWS = 2
PAN_SX = 0.024  # pan width (X)
PAN_SY = 0.020  # pan depth (Y)
PAN_DEPTH = 0.0058  # cream height (seats in its cell, top just below the rim)
PAN_GAP_X = 0.0035
PAN_GAP_Y = 0.0035

# Egg-crate divider: every pan is framed in its own walled cell ("一格一格")
# instead of standing proud in one open well.
PAN_CLEAR = 0.0006      # clearance between a pan and its cell wall
CELL_BORDER = 0.002     # outer frame ring thickness beyond the pan grid
CELL_TOP_Z = BASE_H     # cell walls rise to the tray rim
CELL_BOT_Z = FLOOR_TOP - 0.0005  # embed slightly into the tray floor
CELL_POCKET_FLOOR = 0.0008  # solid floor left under each pan pocket

# Grid occupies the front portion of the tray; rear strip holds the label.
GRID_W = PAN_COLS * PAN_SX + (PAN_COLS - 1) * PAN_GAP_X
GRID_D = PAN_ROWS * PAN_SY + (PAN_ROWS - 1) * PAN_GAP_Y

# Hinge along rear long edge.
HINGE_Y = DEP_Y / 2.0  # rear edge in part-local Y
HINGE_Z = BASE_H + LID_H / 2.0  # hinge barrel centerline height

# Pastel / skin tone palette for the 8 pans (matches reference: skin tones,
# a green corrector, a lavender corrector, a peach corrector).
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


def _rounded_box(sx: float, sy: float, sz: float, r: float) -> cq.Workplane:
    """Axis-aligned box centered at origin with filleted vertical (Z) edges."""
    wp = cq.Workplane("XY").box(sx, sy, sz)
    r = min(r, sx / 2.0 - 1e-4, sy / 2.0 - 1e-4)
    if r > 0:
        wp = wp.edges("|Z").fillet(r)
    return wp


def _base_tray_mesh():
    """Dark tray: solid slab with a shallow top cavity that holds the pans, and
    a slightly raised label shelf along the rear edge."""
    outer = _rounded_box(LEN_X, DEP_Y, BASE_H, 0.006)
    outer = outer.translate((0.0, 0.0, BASE_H / 2.0))

    # Hollow the top to form the tray interior (leave outer walls + floor).
    # Floor top sits at z=FLOOR_TOP; cavity is open at the top.
    cav_x = LEN_X - 2.0 * WALL
    cav_y = DEP_Y - 2.0 * WALL
    cav_h = BASE_H - FLOOR_TOP  # depth of the open cavity above the floor
    cavity = _rounded_box(cav_x, cav_y, cav_h + 0.02, 0.004)
    # Place cavity so its bottom face lands at FLOOR_TOP (rest extends above).
    cavity = cavity.translate((0.0, 0.0, FLOOR_TOP + (cav_h + 0.02) / 2.0))
    tray = outer.cut(cavity)
    return mesh_from_cadquery(tray, "base_tray")


def _pan_centers():
    """World-XY centers of the 8 pans. Grid sits toward the front (-Y)."""
    centers = []
    # Center the grid in X; push it toward the front so the rear strip is free.
    grid_cx = 0.0
    grid_cy = -0.006  # nudge grid toward front
    x0 = grid_cx - GRID_W / 2.0 + PAN_SX / 2.0
    y0 = grid_cy - GRID_D / 2.0 + PAN_SY / 2.0
    for row in range(PAN_ROWS):
        for col in range(PAN_COLS):
            cx = x0 + col * (PAN_SX + PAN_GAP_X)
            cy = y0 + row * (PAN_SY + PAN_GAP_Y)
            centers.append((row, col, cx, cy))
    return centers


def _pan_mesh():
    """A single rounded concealer pan: a shallow rounded slab of cream that sits
    recessed in its cell, its top surface just below the cell-wall rim."""
    pan = _rounded_box(PAN_SX, PAN_SY, PAN_DEPTH, 0.004)
    return mesh_from_cadquery(pan, "pan_cream")


def _cell_grid_mesh():
    """Silver egg-crate divider: a block covering the pan grid (plus a thin outer
    border) with one recessed pocket cut per pan. The walls left between pockets
    frame every concealer pan in its own cell so none stands proud of the tray."""
    grid_cx = 0.0
    grid_cy = -0.006  # matches _pan_centers grid offset
    block_x = GRID_W + 2.0 * CELL_BORDER
    block_y = GRID_D + 2.0 * CELL_BORDER
    block_h = CELL_TOP_Z - CELL_BOT_Z
    block = _rounded_box(block_x, block_y, block_h, 0.0015)
    block = block.translate((grid_cx, grid_cy, CELL_BOT_Z + block_h / 2.0))
    # Cut one pocket per pan (pan footprint + clearance), open at the top and
    # leaving a thin floor the pan seats on.
    pocket_floor_z = FLOOR_TOP + CELL_POCKET_FLOOR
    pocket_h = (CELL_TOP_Z - pocket_floor_z) + 0.01
    for (row, col, cx, cy) in _pan_centers():
        pocket = _rounded_box(
            PAN_SX + 2.0 * PAN_CLEAR, PAN_SY + 2.0 * PAN_CLEAR, pocket_h, 0.0015
        )
        pocket = pocket.translate((cx, cy, pocket_floor_z + pocket_h / 2.0))
        block = block.cut(pocket)
    return mesh_from_cadquery(block, "cell_grid")


def _label_strip_mesh():
    """Raised glossy label shelf along the rear edge carrying the CONCEALER text
    band (modeled as a thin embossed strip)."""
    # Strip rises from the tray floor and hugs the rear interior wall so it reads
    # as a raised label shelf bonded to the tray (stays geometrically connected).
    strip_h = BASE_H - FLOOR_TOP + 0.0014  # from floor up past the rim
    strip_y = DEP_Y / 2.0 - WALL - 0.0055  # against the rear interior wall
    strip = _rounded_box(LEN_X - 2.0 * WALL - 0.004, 0.011, strip_h, 0.0008)
    strip = strip.translate((0.0, strip_y, FLOOR_TOP + strip_h / 2.0))
    return mesh_from_cadquery(strip, "label_strip")


def _lid_slab_mesh():
    """Lid slab modeled in the lid-local frame (its own origin at the hinge).
    Built here centered at origin; positioned via the visual origin so that when
    closed it covers the base exactly."""
    slab = _rounded_box(LEN_X, DEP_Y, LID_H, 0.006)
    # Hollow a shallow well on the inner (-Z) face to seat the mirror. The well
    # carves from the bottom face up to z=SLAB_WELL_TOP, leaving solid above it.
    well_h = (SLAB_WELL_TOP - (-LID_H / 2.0)) + 0.01  # extends below the slab bottom
    well = _rounded_box(LEN_X - 2.0 * WALL, DEP_Y - 2.0 * WALL, well_h, 0.004)
    well = well.translate((0.0, 0.0, SLAB_WELL_TOP - well_h / 2.0))
    slab = slab.cut(well)
    return mesh_from_cadquery(slab, "lid_slab")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="concealer_palette_compact")

    dark = model.material("case_dark", rgba=(0.10, 0.10, 0.12, 1.0))
    silver = model.material("frame_silver", rgba=(0.78, 0.80, 0.83, 1.0))
    mirror = model.material("mirror_glass", rgba=(0.82, 0.88, 0.92, 1.0))
    clear = model.material("lid_clear", rgba=(0.86, 0.90, 0.93, 0.55))
    label_mat = model.material("label_white", rgba=(0.95, 0.95, 0.96, 1.0))

    pan_mats = [
        model.material(f"pan_mat_{i}", rgba=c) for i, c in enumerate(PAN_COLORS)
    ]

    # =====================================================================
    # BASE (root): dark tray + silver frame + 8 pans + label strip
    # =====================================================================
    base = model.part("base")
    base.visual(_base_tray_mesh(), material=dark, name="base_tray")

    # Silver rim frame around the top of the tray (thin raised border).
    frame = _rounded_box(LEN_X, DEP_Y, 0.0018, 0.006)
    frame_inner = _rounded_box(LEN_X - 2.0 * WALL, DEP_Y - 2.0 * WALL, 0.01, 0.004)
    frame_inner = frame_inner.translate((0.0, 0.0, 0.0))
    frame = frame.cut(frame_inner)
    frame = frame.translate((0.0, 0.0, BASE_H + 0.0009))
    base.visual(mesh_from_cadquery(frame, "base_frame"), material=silver, name="base_frame")

    # Silver egg-crate divider: one walled cell per pan.
    base.visual(_cell_grid_mesh(), material=silver, name="cell_grid")

    # Raised label shelf along the rear edge.
    base.visual(_label_strip_mesh(), material=label_mat, name="label_strip")

    # 8 concealer pans, each seated in its own cell pocket. The pan rests on the
    # pocket floor (embedding ~0.5 mm for connectivity) with its top just below
    # the cell-wall rim, so it reads as framed and recessed, never protruding.
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

    # =====================================================================
    # LID: revolute about the rear hinge; silver frame + clear top + mirror
    # =====================================================================
    # The lid part-local frame has its ORIGIN at the hinge axis (rear edge,
    # at height HINGE_Z). The slab is centered in X but offset in -Y so the
    # rear edge of the lid sits at the hinge, and offset in +Z so the closed
    # lid rests just above the base. Visual origin places geometry accordingly.
    lid = model.part("lid")

    # Lid geometry is built centered at origin, then placed via the visual
    # origin: shift -Y by DEP_Y/2 so its rear edge meets the hinge, and +Z so
    # the lid floats just above the base top.
    lid_x = 0.0
    lid_y = -DEP_Y / 2.0  # rear edge of lid at local y=0 (hinge line)
    lid_z = LID_H / 2.0  # lid slab center above the hinge centerline

    lid.visual(
        _lid_slab_mesh(),
        origin=Origin(xyz=(lid_x, lid_y, lid_z)),
        material=clear,
        name="lid_slab",
    )

    # Silver frame border on the lid (matches the base frame silhouette).
    lid_frame = _rounded_box(LEN_X, DEP_Y, 0.0018, 0.006)
    lid_frame_inner = _rounded_box(LEN_X - 2.0 * WALL, DEP_Y - 2.0 * WALL, 0.01, 0.004)
    lid_frame = lid_frame.cut(lid_frame_inner)
    lid_frame = lid_frame.translate((0.0, lid_y, LID_H - 0.0009 + 1e-4))
    lid.visual(
        mesh_from_cadquery(lid_frame, "lid_frame"),
        material=silver,
        name="lid_frame",
    )

    # Mirror on the INNER (lower, -Z) face of the lid, seated in the well. The
    # slab is shifted up by lid_z, so the well floor sits at lid_z + SLAB_WELL_TOP.
    # Seat the mirror so its top embeds slightly into that floor, bonding it to
    # the lid slab; the mirror face points down (inward) when the lid is closed.
    mirror_th = 0.0020
    well_floor_z = lid_z + SLAB_WELL_TOP
    mirror_plate = _rounded_box(LEN_X - 2.0 * WALL - 0.001, DEP_Y - 2.0 * WALL - 0.001, mirror_th, 0.003)
    mirror_plate = mirror_plate.translate(
        (0.0, lid_y, well_floor_z - mirror_th / 2.0 + 0.0006)
    )
    lid.visual(
        mesh_from_cadquery(mirror_plate, "mirror"),
        material=mirror,
        name="mirror",
    )

    lid.inertial = Inertial.from_geometry(
        Box((LEN_X, DEP_Y, LID_H)),
        mass=0.04,
        origin=Origin(xyz=(0.0, lid_y, 0.0)),
    )

    # Visible hinge barrel along the rear edge (two silver knuckles), authored
    # on the BASE so the pin line is grounded; the lid swings about it.
    for hx in (-0.030, 0.030):
        knuckle = CylinderGeometry(0.0022, 0.018, radial_segments=24).rotate_y(math.pi / 2.0)
        knuckle.translate(hx, HINGE_Y, HINGE_Z)
        base.visual(
            mesh_from_geometry(knuckle, f"hinge_knuckle_{'l' if hx < 0 else 'r'}"),
            material=silver,
            name=f"hinge_knuckle_{'l' if hx < 0 else 'r'}",
        )

    # Revolute hinge: axis along -X at the rear edge. The lid extends toward -Y
    # (front). Positive rotation about -X (right-hand rule) lifts the FRONT edge
    # (-Y) up (+Z) and swings it back toward the hinge (+Y).
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

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("base_to_lid")

    # ---- 8 concealer pans present, distinct, and seated in the tray ----
    pan_names = [f"pan_{r}_{c}" for r in range(PAN_ROWS) for c in range(PAN_COLS)]
    ctx.check(
        "eight concealer pans authored",
        len(pan_names) == 8 and all(base.get_visual(n) is not None for n in pan_names),
        details=f"pans={pan_names}",
    )
    # Pans sit recessed inside the tray footprint (within the base in XY).
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
        # Each pan is framed within the cell-grid footprint...
        ctx.expect_within(
            base, base, axes="xy", inner_elem=n, outer_elem="cell_grid",
            margin=0.0, name=f"{n} framed within cell grid",
        )
        # ...and recessed: its top never rises above the cell-wall rim.
        p_aabb = ctx.part_element_world_aabb(base, elem=n)
        ctx.check(
            f"{n} recessed below cell rim (not protruding)",
            p_aabb is not None and cell_top is not None
            and p_aabb[1][2] <= cell_top + 0.0002,
            details=f"pan_top={p_aabb[1][2] if p_aabb else None}, cell_top={cell_top}",
        )

    # ---- mirror is on the lid inner face ----
    ctx.check(
        "mirror authored on lid",
        lid.get_visual("mirror") is not None,
        details="mirror visual missing",
    )
    # Mirror is on the lower (inner) face of the lid slab when closed: its top
    # should be below the lid slab top.
    mir_aabb = ctx.part_element_world_aabb(lid, elem="mirror")
    slab_aabb = ctx.part_element_world_aabb(lid, elem="lid_slab")
    ctx.check(
        "mirror on inner face of lid",
        mir_aabb is not None and slab_aabb is not None and mir_aabb[1][2] < slab_aabb[1][2] - 0.001,
        details=f"mirror_top={mir_aabb[1][2] if mir_aabb else None}, slab_top={slab_aabb[1][2] if slab_aabb else None}",
    )

    # ---- closed lid covers the base (footprint overlap in XY) ----
    ctx.expect_overlap(
        lid, base, axes="xy", min_overlap=0.05,
        name="closed lid covers base footprint",
    )
    # Closed lid sits above the base (no penetration into the tray), small seam.
    ctx.expect_gap(
        lid, base, axis="z", positive_elem="lid_slab", negative_elem="base_frame",
        min_gap=-0.001, name="closed lid rests above base",
    )

    # ---- lid rotates open about the rear hinge: FRONT edge lifts and swings back ----
    closed_aabb = ctx.part_world_aabb(lid)
    closed_front_y = closed_aabb[0][1]  # most -Y (front) extent
    closed_top_z = closed_aabb[1][2]
    with ctx.pose({hinge: math.radians(100.0)}):
        open_aabb = ctx.part_world_aabb(lid)
        open_top_z = open_aabb[1][2]
        open_front_y = open_aabb[0][1]
    # Opening lifts the lid much higher (front edge swings up).
    ctx.check(
        "lid opens upward about rear hinge",
        open_top_z > closed_top_z + 0.02,
        details=f"closed_top_z={closed_top_z}, open_top_z={open_top_z}",
    )
    # Front edge swings rearward (toward +Y / the hinge) as it lifts.
    ctx.check(
        "lid front edge swings back toward hinge",
        open_front_y > closed_front_y + 0.01,
        details=f"closed_front_y={closed_front_y}, open_front_y={open_front_y}",
    )

    # ---- hinge knuckles grounded on the base near the rear edge ----
    ctx.check(
        "hinge knuckles authored on base",
        base.get_visual("hinge_knuckle_l") is not None
        and base.get_visual("hinge_knuckle_r") is not None,
        details="missing hinge knuckles",
    )

    # The closed lid seats over the base rim; allow the tiny intentional embed of
    # the lid frame against the base frame and the hinge interface.
    ctx.allow_overlap(
        lid, base,
        elem_a="lid_frame", elem_b="base_frame",
        reason="Lid silver frame seats flush onto the base silver rim when closed.",
    )

    return ctx.report()


object_model = build_object_model()
