from __future__ import annotations

# Round concealer makeup palette compact with a hinged mirror lid.
# Frame: palette lies flat in the XY plane.
#   Circular disc footprint, radius ~0.040 m (80 mm diameter).
#   +Z = up (thickness ~0.015 m).
#   The hinge runs along the REAR edge (+Y side); the lid closes down onto
#   the base and opens by swinging the FRONT edge up and back about that
#   rear hinge axis (axis parallel to +X).
# Parts:
#   - base (root): dark circular tray with a silver ring frame, 6 circular
#     concealer pans arranged radially, and a raised arc label strip at rear.
#   - lid: REVOLUTE about the rear hinge; silver ring frame + clear disc top
#     and a reflective circular mirror on its inner (lower) face. Opens 0->~110 deg.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
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
RADIUS = 0.040  # disc outer radius
BASE_H = 0.009  # base tray thickness
LID_H = 0.006   # lid slab thickness

WALL = 0.004    # outer wall thickness of the base tray
FLOOR_TOP = 0.0025  # z of the tray interior floor (above the bottom slab)
# Slab-local z of the inner well floor (remaining slab) the mirror seats against.
SLAB_WELL_TOP = -LID_H / 2.0 + 0.002

# Pan layout: 6 circular pans in a symmetric ring around the tray center.
PAN_COUNT = 6
PAN_RADIUS = 0.009   # each pan disc radius
PAN_DEPTH = 0.0058   # cream height (seats in its cell, top just below the rim)
PAN_RING_RADIUS = 0.020  # distance from tray center to each pan center
PAN_RING_CY = 0.0

# Egg-crate divider: every pan is framed in its own round cell ("一格一格")
# instead of standing proud in one open well.
PAN_CLEAR = 0.0005      # clearance between a pan and its cell wall
CELL_TOP_Z = BASE_H     # cell walls rise to the tray rim
CELL_BOT_Z = FLOOR_TOP - 0.0005  # embed slightly into the tray floor
CELL_POCKET_FLOOR = 0.0008  # solid floor left under each pan pocket

# Hinge along rear edge of the disc.
HINGE_Y = RADIUS  # rear edge in part-local Y (tangent to circle at +Y)
HINGE_Z = BASE_H + LID_H / 2.0  # hinge barrel centerline height

# Skin-tone / corrector colors for the 6 pans.
PAN_COLORS = [
    (0.93, 0.78, 0.62, 1.0),  # light beige
    (0.88, 0.70, 0.55, 1.0),  # warm tan
    (0.70, 0.86, 0.70, 1.0),  # green corrector
    (0.92, 0.62, 0.45, 1.0),  # peach corrector
    (0.95, 0.82, 0.68, 1.0),  # fair
    (0.84, 0.74, 0.86, 1.0),  # lavender corrector
]


def _disc(outer_r: float, height: float) -> cq.Workplane:
    """Solid cylinder centered at origin with base at z=0, top at z=height."""
    return cq.Workplane("XY").circle(outer_r).extrude(height)


def _ring(outer_r: float, inner_r: float, height: float) -> cq.Workplane:
    """Annular ring (washer) centered at origin, from z=0 to z=height."""
    wp = (
        cq.Workplane("XY")
        .circle(outer_r)
        .circle(inner_r)
        .extrude(height)
    )
    return wp


def _base_tray_mesh():
    """Dark circular tray: solid disc with a shallow top cavity that holds the
    pans. The cavity is open at the top, leaving outer walls and a floor."""
    outer = _disc(RADIUS, BASE_H)

    # Hollow the top to form the tray interior (leave outer walls + floor).
    cav_r = RADIUS - WALL
    cav_h = BASE_H - FLOOR_TOP  # depth of the open cavity above the floor
    cavity = _disc(cav_r, cav_h + 0.02)
    # Place cavity so its bottom face lands at FLOOR_TOP (rest extends above).
    cavity = cavity.translate((0.0, 0.0, FLOOR_TOP))
    tray = outer.cut(cavity)
    return mesh_from_cadquery(tray, "base_tray")


def _pan_centers():
    """World-XY centers of the 6 circular pans, arranged radially."""
    centers = []
    for i in range(PAN_COUNT):
        angle = i * (2.0 * math.pi / PAN_COUNT)
        cx = PAN_RING_RADIUS * math.cos(angle)
        cy = PAN_RING_CY + PAN_RING_RADIUS * math.sin(angle)
        centers.append((i, cx, cy))
    return centers


def _pan_mesh():
    """A single circular concealer pan: a shallow disc of cream that sits
    recessed in the tray, its top surface slightly domed flush with the rim."""
    pan = _disc(PAN_RADIUS, PAN_DEPTH)
    # Center vertically so origin is at the pan center.
    pan = pan.translate((0.0, 0.0, -PAN_DEPTH / 2.0))
    return mesh_from_cadquery(pan, "pan_cream")


def _cell_grid_mesh():
    """Silver divider plate filling the tray cavity with one round pocket cut per
    pan. Each circular pan ends up framed in its own recessed cell rather than
    standing proud in one open well."""
    cav_r = RADIUS - WALL
    block_h = CELL_TOP_Z - CELL_BOT_Z
    block = _disc(cav_r - 0.0005, block_h)
    block = block.translate((0.0, 0.0, CELL_BOT_Z))
    pocket_floor_z = FLOOR_TOP + CELL_POCKET_FLOOR
    pocket_h = (CELL_TOP_Z - pocket_floor_z) + 0.01
    for (i, cx, cy) in _pan_centers():
        pocket = _disc(PAN_RADIUS + PAN_CLEAR, pocket_h)
        pocket = pocket.translate((cx, cy, pocket_floor_z))
        block = block.cut(pocket)
    return mesh_from_cadquery(block, "cell_grid")


def _lid_slab_mesh():
    """Lid disc modeled in the lid-local frame. Built centered at origin;
    positioned via the visual origin so that when closed it covers the base."""
    slab = _disc(RADIUS, LID_H)
    slab = slab.translate((0.0, 0.0, -LID_H / 2.0))
    # Hollow a shallow well on the inner (-Z) face to seat the mirror.
    well_r = RADIUS - WALL
    well_h = (SLAB_WELL_TOP - (-LID_H / 2.0)) + 0.01
    well = _disc(well_r, well_h)
    well = well.translate((0.0, 0.0, SLAB_WELL_TOP - well_h))
    slab = slab.cut(well)
    return mesh_from_cadquery(slab, "lid_slab")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="round_concealer_palette_compact")

    dark = model.material("case_dark", rgba=(0.10, 0.10, 0.12, 1.0))
    silver = model.material("frame_silver", rgba=(0.78, 0.80, 0.83, 1.0))
    mirror_mat = model.material("mirror_glass", rgba=(0.82, 0.88, 0.92, 1.0))
    clear = model.material("lid_clear", rgba=(0.86, 0.90, 0.93, 0.55))
    pan_mats = [
        model.material(f"pan_mat_{i}", rgba=c) for i, c in enumerate(PAN_COLORS)
    ]

    # =====================================================================
    # BASE (root): dark circular tray + silver ring frame + 6 centered pans
    # =====================================================================
    base = model.part("base")
    base.visual(_base_tray_mesh(), material=dark, name="base_tray")

    # Silver rim frame ring around the top of the tray (thin raised border).
    frame_h = 0.0018
    frame = _ring(RADIUS, RADIUS - WALL, frame_h)
    frame = frame.translate((0.0, 0.0, BASE_H))
    base.visual(mesh_from_cadquery(frame, "base_frame"), material=silver, name="base_frame")

    # Silver divider plate: one walled round cell per pan.
    base.visual(_cell_grid_mesh(), material=silver, name="cell_grid")

    # 6 circular concealer pans, each seated in its own cell pocket: the pan rests
    # on the pocket floor with its top just below the cell-wall rim, so it reads
    # as framed and recessed, never protruding.
    pan_center_z = FLOOR_TOP + CELL_POCKET_FLOOR - 0.0005 + PAN_DEPTH / 2.0
    for (i, cx, cy) in _pan_centers():
        base.visual(
            _pan_mesh(),
            origin=Origin(xyz=(cx, cy, pan_center_z)),
            material=pan_mats[i],
            name=f"pan_{i}",
        )

    base.inertial = Inertial.from_geometry(
        Cylinder(RADIUS, BASE_H),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, BASE_H / 2.0)),
    )

    # =====================================================================
    # LID: revolute about the rear hinge; silver ring frame + clear disc + mirror
    # =====================================================================
    # The lid part-local frame has its ORIGIN at the hinge axis (rear edge,
    # at height HINGE_Z). The disc extends toward -Y from the hinge.
    lid = model.part("lid")

    # Lid geometry is built centered at origin, then placed via the visual
    # origin: shift -Y by RADIUS so its rear edge meets the hinge, and +Z so
    # the lid floats just above the base top.
    lid_y = -RADIUS  # rear edge of lid at local y=0 (hinge line)
    lid_z = LID_H / 2.0

    lid.visual(
        _lid_slab_mesh(),
        origin=Origin(xyz=(0.0, lid_y, lid_z)),
        material=clear,
        name="lid_slab",
    )

    # Silver ring frame border on the lid (matches the base frame silhouette).
    lid_frame_h = 0.0018
    lid_frame = _ring(RADIUS, RADIUS - WALL, lid_frame_h)
    lid_frame = lid_frame.translate((0.0, lid_y, LID_H - lid_frame_h / 2.0))
    lid.visual(
        mesh_from_cadquery(lid_frame, "lid_frame"),
        material=silver,
        name="lid_frame",
    )

    # Mirror on the INNER (lower, -Z) face of the lid, seated in the well.
    mirror_th = 0.0020
    mirror_r = RADIUS - WALL - 0.002
    well_floor_z = lid_z + SLAB_WELL_TOP
    mirror_disc = _disc(mirror_r, mirror_th)
    mirror_disc = mirror_disc.translate(
        (0.0, lid_y, well_floor_z - mirror_th + 0.0006)
    )
    lid.visual(
        mesh_from_cadquery(mirror_disc, "mirror"),
        material=mirror_mat,
        name="mirror",
    )

    lid.inertial = Inertial.from_geometry(
        Cylinder(RADIUS, LID_H),
        mass=0.04,
        origin=Origin(xyz=(0.0, lid_y, 0.0)),
    )

    # Visible hinge barrel along the rear edge (two silver knuckles), authored
    # on the BASE so the pin line is grounded; the lid swings about it.
    for hx in (-0.018, 0.018):
        knuckle = CylinderGeometry(0.0022, 0.016, radial_segments=24).rotate_y(math.pi / 2.0)
        knuckle.translate(hx, HINGE_Y, HINGE_Z)
        base.visual(
            mesh_from_geometry(knuckle, f"hinge_knuckle_{'l' if hx < 0 else 'r'}"),
            material=silver,
            name=f"hinge_knuckle_{'l' if hx < 0 else 'r'}",
        )

    # Revolute hinge: axis along -X at the rear edge. The lid extends toward -Y
    # (front). Positive rotation about -X lifts the FRONT edge up and back.
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


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("base_to_lid")

    # ---- Round footprint: base and lid are circular discs, not rectangular ----
    base_aabb = ctx.part_world_aabb(base)
    lid_aabb = ctx.part_world_aabb(lid)
    if base_aabb is not None:
        bx = base_aabb[1][0] - base_aabb[0][0]
        by = base_aabb[1][1] - base_aabb[0][1]
        # A disc has equal extents in X and Y (within tolerance).
        ctx.check(
            "base has round footprint (equal X and Y extents)",
            abs(bx - by) < 0.005,
            details=f"base_dx={bx:.4f}, base_dy={by:.4f}",
        )
    if lid_aabb is not None:
        lx = lid_aabb[1][0] - lid_aabb[0][0]
        ly = lid_aabb[1][1] - lid_aabb[0][1]
        ctx.check(
            "lid has round footprint (equal X and Y extents)",
            abs(lx - ly) < 0.005,
            details=f"lid_dx={lx:.4f}, lid_dy={ly:.4f}",
        )

    # ---- 6 concealer pans present, distinct, and seated in the tray ----
    pan_names = [f"pan_{i}" for i in range(PAN_COUNT)]
    ctx.check(
        "six concealer pans authored",
        len(pan_names) == PAN_COUNT and all(base.get_visual(n) is not None for n in pan_names),
        details=f"pans={pan_names}",
    )
    # Pans sit recessed inside the tray footprint (within the base in XY).
    for n in pan_names:
        ctx.expect_within(
            base, base, axes="xy", inner_elem=n, outer_elem="base_tray",
            margin=0.001, name=f"{n} within tray footprint",
        )

    # ---- divider cells: every pan is walled into its own round cell ----
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

    # ---- mirror is on the lid inner face ----
    ctx.check(
        "mirror authored on lid",
        lid.get_visual("mirror") is not None,
        details="mirror visual missing",
    )
    # Mirror is on the lower (inner) face of the lid slab when closed.
    mir_aabb = ctx.part_element_world_aabb(lid, elem="mirror")
    slab_aabb = ctx.part_element_world_aabb(lid, elem="lid_slab")
    ctx.check(
        "mirror on inner face of lid",
        mir_aabb is not None and slab_aabb is not None and mir_aabb[1][2] < slab_aabb[1][2] - 0.001,
        details=f"mirror_top={mir_aabb[1][2] if mir_aabb else None}, slab_top={slab_aabb[1][2] if slab_aabb else None}",
    )

    # ---- closed lid covers the base (footprint overlap in XY) ----
    ctx.expect_overlap(
        lid, base, axes="xy", min_overlap=0.04,
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
    # Opening lifts the lid much higher.
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
    # the lid frame against the base frame.
    ctx.allow_overlap(
        lid, base,
        elem_a="lid_frame", elem_b="base_frame",
        reason="Lid silver frame seats flush onto the base silver rim when closed.",
    )

    return ctx.report()


object_model = build_object_model()
