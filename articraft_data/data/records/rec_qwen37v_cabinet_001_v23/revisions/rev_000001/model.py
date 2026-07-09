from __future__ import annotations

"""Vintage industrial steel locker cabinet – door + drawer variant (v23).

Left half: one full-height hinged door with three visible hinge barrels and a
rotating latch at the centre seam.  Right half: three stacked drawers on
prismatic slides.  Centre divider separates the two sections.  Caster blocks
at the base.  Overall envelope ~1.6 m wide × 0.5 m deep × ~1.8 m tall,
brushed/tarnished raw steel.
"""

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Global dimensions (meters).  Cabinet centred on X, front face at +Y.
# ---------------------------------------------------------------------------
CAB_W = 1.60
CAB_D = 0.50
CAB_TOP = 1.80
CASTER_H = 0.15          # caster block total height; carcass bottom here
WALL_T = 0.02

FRONT_Y = CAB_D / 2.0    # +0.25
BACK_Y = -CAB_D / 2.0    # -0.25

BOTTOM_RAIL_TOP = CASTER_H + 0.06   # 0.21
TOP_RAIL_BOT = CAB_TOP - 0.06       # 1.74

# Centre divider
DIVIDER_X = 0.0
DIVIDER_T = WALL_T

# ---- Door section (left half) -------------------------------------------
DOOR_SEC_L = -CAB_W / 2.0 + WALL_T   # -0.78  (inner left wall face)
DOOR_SEC_R = DIVIDER_X - DIVIDER_T / 2.0  # -0.01

DOOR_W = DOOR_SEC_R - DOOR_SEC_L - 0.005  # ≈0.765
DOOR_T = WALL_T
DOOR_Z0 = BOTTOM_RAIL_TOP + 0.002
DOOR_Z1 = TOP_RAIL_BOT - 0.002
DOOR_H = DOOR_Z1 - DOOR_Z0
DOOR_ZC = 0.5 * (DOOR_Z0 + DOOR_Z1)
DOOR_HINGE_X = DOOR_SEC_L               # hinge line at inner left wall

# ---- Drawer section (right half) ----------------------------------------
DRAWER_SEC_L = DIVIDER_X + DIVIDER_T / 2.0  # 0.01
DRAWER_SEC_R = CAB_W / 2.0 - WALL_T         # 0.78
DRAWER_W = DRAWER_SEC_R - DRAWER_SEC_L - 0.01  # ≈0.76
DRAWER_FRONT_T = WALL_T
DRAWER_DEPTH = 0.38
DRAWER_XC = (DRAWER_SEC_L + DRAWER_SEC_R) / 2.0

N_DRAWERS = 3
DRAWER_GAP = 0.015
DRAWER_SEC_H = TOP_RAIL_BOT - BOTTOM_RAIL_TOP
DRAWER_H = (DRAWER_SEC_H - (N_DRAWERS - 1) * DRAWER_GAP) / N_DRAWERS

drawer_zc: list[float] = []
for _i in range(N_DRAWERS):
    _zbot = BOTTOM_RAIL_TOP + _i * (DRAWER_H + DRAWER_GAP)
    drawer_zc.append(_zbot + DRAWER_H / 2.0)

# ---- Shared detail sizes -------------------------------------------------
BARREL_R = 0.010
KNUCKLE_R = 0.013
BARREL_SEG_H = 0.08

SLOT_LEN = 0.40
SLOT_W = 0.030
SLOT_ZC = -0.45            # door-local z

CAP_T = 0.022
CAP_OVERHANG = 0.02

CASTER_PLATE_W = 0.08
CASTER_PLATE_D = 0.08
CASTER_WHEEL_R = 0.025
CASTER_WHEEL_W = 0.025

# ---- Motion limits -------------------------------------------------------
DOOR_OPEN = math.radians(110.0)
LATCH_TURN = math.radians(90.0)
DRAWER_TRAVEL = 0.35


# ===================================================================
# Geometry helpers
# ===================================================================

def _door_leaf_mesh(name: str):
    """Flat panel with a rounded-end through slot near the bottom.
    Panel extends along +X from the hinge line (left-hinged)."""
    xc = DOOR_W / 2.0
    panel = (
        cq.Workplane("XY")
        .box(DOOR_W, DOOR_T, DOOR_H)
        .translate((xc, -DOOR_T / 2.0, 0.0))
    )
    cutter = (
        cq.Workplane("XZ")
        .slot2D(SLOT_LEN, SLOT_W, 90)
        .extrude(0.05, both=True)
        .translate((xc, 0.0, SLOT_ZC))
    )
    leaf = panel.cut(cutter)
    return mesh_from_cadquery(leaf, name)


def _hinge_barrel_mesh(name: str):
    """Short visible hinge barrel with three knuckle rings."""
    barrel = (
        cq.Workplane("XY")
        .circle(BARREL_R)
        .extrude(BARREL_SEG_H / 2.0, both=True)
    )
    ring_h = 0.018
    for zc in (-BARREL_SEG_H / 2.0 + ring_h / 2.0, 0.0, BARREL_SEG_H / 2.0 - ring_h / 2.0):
        ring = (
            cq.Workplane("XY")
            .circle(KNUCKLE_R)
            .extrude(ring_h / 2.0, both=True)
            .translate((0.0, 0.0, zc))
        )
        barrel = barrel.union(ring)
    return mesh_from_cadquery(barrel, name)


def _caster_block_mesh(name: str):
    """Caster block: mounting plate + two fork plates + horizontal wheel."""
    plate = (
        cq.Workplane("XY")
        .box(CASTER_PLATE_W, CASTER_PLATE_D, 0.02)
        .translate((0.0, 0.0, CASTER_H - 0.01))
    )
    fork_h = CASTER_H - 0.02 - CASTER_WHEEL_R + 0.005
    fork_zc = CASTER_WHEEL_R + fork_h / 2.0
    for sx in (-1.0, 1.0):
        fork = (
            cq.Workplane("XY")
            .box(0.008, CASTER_PLATE_D * 0.6, fork_h)
            .translate((sx * (CASTER_WHEEL_W / 2.0 + 0.005), 0.0, fork_zc))
        )
        plate = plate.union(fork)
    wheel = (
        cq.Workplane("XZ")
        .circle(CASTER_WHEEL_R)
        .extrude(CASTER_WHEEL_W / 2.0, both=True)
        .translate((0.0, 0.0, CASTER_WHEEL_R))
    )
    result = plate.union(wheel)
    return mesh_from_cadquery(result, name)


# ===================================================================
# Build
# ===================================================================

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_steel_locker_cabinet_v23")

    steel_body = model.material("steel_body", rgba=(0.60, 0.61, 0.63, 1.0))
    steel_door = model.material("steel_door", rgba=(0.55, 0.56, 0.58, 1.0))
    steel_drawer = model.material("steel_drawer", rgba=(0.52, 0.53, 0.56, 1.0))
    steel_trim = model.material("steel_trim", rgba=(0.46, 0.47, 0.49, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.07, 0.07, 0.08, 1.0))
    steel_caster = model.material("steel_caster", rgba=(0.38, 0.39, 0.41, 1.0))
    steel_knob = model.material("steel_knob", rgba=(0.18, 0.18, 0.20, 1.0))
    steel_rivet = model.material("steel_rivet", rgba=(0.52, 0.53, 0.55, 1.0))
    steel_handle = model.material("steel_handle", rgba=(0.42, 0.43, 0.46, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.12, 0.12, 0.13, 1.0))

    # ==================================================================
    # Cabinet body (root)
    # ==================================================================
    body = model.part("cabinet_body")
    carcass_h = CAB_TOP - CASTER_H
    carcass_zc = CASTER_H + carcass_h / 2.0

    # Side walls
    for sx, vn in ((-1.0, "side_wall_left"), (1.0, "side_wall_right")):
        body.visual(
            Box((WALL_T, CAB_D, carcass_h)),
            origin=Origin(xyz=(sx * (CAB_W / 2.0 - WALL_T / 2.0), 0.0, carcass_zc)),
            material=steel_body, name=vn,
        )
    # Back wall
    body.visual(
        Box((CAB_W - WALL_T, WALL_T, carcass_h - 0.02)),
        origin=Origin(xyz=(0.0, BACK_Y + WALL_T / 2.0, carcass_zc)),
        material=steel_body, name="back_wall",
    )
    # Bottom / top panels
    body.visual(
        Box((CAB_W, CAB_D, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, CASTER_H + WALL_T / 2.0)),
        material=steel_body, name="bottom_panel",
    )
    body.visual(
        Box((CAB_W, CAB_D, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, CAB_TOP - WALL_T / 2.0)),
        material=steel_body, name="top_panel",
    )
    # Centre divider
    body.visual(
        Box((DIVIDER_T, CAB_D - 2 * WALL_T, carcass_h - 0.02)),
        origin=Origin(xyz=(DIVIDER_X, 0.0, carcass_zc)),
        material=steel_body, name="center_divider",
    )
    # Front frame rails (stop exactly at the door/drawer opening edges)
    body.visual(
        Box((CAB_W - 2 * WALL_T, WALL_T, BOTTOM_RAIL_TOP - CASTER_H)),
        origin=Origin(
            xyz=(0.0, FRONT_Y - WALL_T / 2.0, (CASTER_H + BOTTOM_RAIL_TOP) / 2.0)
        ),
        material=steel_body, name="front_bottom_rail",
    )
    body.visual(
        Box((CAB_W - 2 * WALL_T, WALL_T, CAB_TOP - TOP_RAIL_BOT)),
        origin=Origin(
            xyz=(0.0, FRONT_Y - WALL_T / 2.0, (TOP_RAIL_BOT + CAB_TOP) / 2.0)
        ),
        material=steel_body, name="front_top_rail",
    )
    # Horizontal drawer dividers (2 between 3 drawers)
    for i in range(N_DRAWERS - 1):
        z_div = BOTTOM_RAIL_TOP + (i + 1) * DRAWER_H + i * DRAWER_GAP + DRAWER_GAP / 2.0
        body.visual(
            Box((DRAWER_W + 0.01, WALL_T, DRAWER_GAP + 0.004)),
            origin=Origin(
                xyz=(DRAWER_XC, FRONT_Y - WALL_T / 2.0, z_div)
            ),
            material=steel_trim, name=f"drawer_divider_{i}",
        )
    # Interior shelf (door section)
    body.visual(
        Box((DOOR_W + 0.01, CAB_D - 2 * WALL_T - 0.02, 0.015)),
        origin=Origin(
            xyz=((DOOR_SEC_L + DOOR_SEC_R) / 2.0, 0.0, 0.95)
        ),
        material=steel_body, name="interior_shelf",
    )
    # Drawer slide rails (thin strips overlapping divider + right wall, per drawer)
    for i in range(N_DRAWERS):
        zc = drawer_zc[i]
        rail_z = zc - DRAWER_H / 2.0 + 0.015
        rail_depth = CAB_D - 2 * WALL_T - 0.04
        body.visual(
            Box((0.015, rail_depth, 0.015)),
            origin=Origin(xyz=(DRAWER_SEC_L + 0.006, 0.0, rail_z)),
            material=steel_trim, name=f"drawer_rail_l_{i}",
        )
        body.visual(
            Box((0.015, rail_depth, 0.015)),
            origin=Origin(xyz=(DRAWER_SEC_R - 0.006, 0.0, rail_z)),
            material=steel_trim, name=f"drawer_rail_r_{i}",
        )
    # Top cap + rivets
    body.visual(
        Box((CAB_W + 2 * CAP_OVERHANG, CAB_D + 2 * CAP_OVERHANG, CAP_T)),
        origin=Origin(xyz=(0.0, 0.0, CAB_TOP - 0.001 + CAP_T / 2.0)),
        material=steel_trim, name="top_cap",
    )
    n_riv = 13
    for i in range(n_riv):
        rx = -0.72 + i * (1.44 / (n_riv - 1))
        body.visual(
            Sphere(radius=0.0045),
            origin=Origin(xyz=(rx, FRONT_Y + 0.002, 1.77)),
            material=steel_rivet, name=f"rivet_{i}",
        )
    # Visible hinge barrels (3, on the body at the door hinge edge)
    hinge_barrel_mesh = _hinge_barrel_mesh("hinge_barrel")
    hinge_zs = [DOOR_Z0 + 0.18, DOOR_ZC, DOOR_Z1 - 0.18]
    for j, hz in enumerate(hinge_zs):
        body.visual(
            hinge_barrel_mesh,
            origin=Origin(xyz=(DOOR_HINGE_X, FRONT_Y, hz)),
            material=steel_trim, name=f"hinge_barrel_{j}",
        )
    # Caster blocks (4 corners)
    caster_mesh = _caster_block_mesh("caster_block")
    caster_corners = [
        (0.72, 0.19), (-0.72, 0.19),
        (-0.72, -0.19), (0.72, -0.19),
    ]
    for i, (cx, cy) in enumerate(caster_corners):
        body.visual(
            caster_mesh,
            origin=Origin(xyz=(cx, cy, 0.0)),
            material=steel_caster, name=f"caster_{i}",
        )

    # ==================================================================
    # Door (left side, hinged on left edge)
    # ==================================================================
    door = model.part("door")
    door_xc = DOOR_W / 2.0

    door.visual(
        _door_leaf_mesh("door_leaf"),
        material=steel_door, name="leaf",
    )
    # Dark backing behind vent slot
    door.visual(
        Box((SLOT_W + 0.016, 0.005, SLOT_LEN + 0.036)),
        origin=Origin(xyz=(door_xc, -DOOR_T - 0.001, SLOT_ZC)),
        material=steel_dark, name="vent_backing",
    )
    # Stamped vent lines near top
    for j, dz in enumerate((0.60, 0.62, 0.64)):
        door.visual(
            Box((0.22, 0.004, 0.006)),
            origin=Origin(xyz=(door_xc, -0.0012, dz)),
            material=steel_dark, name=f"vent_line_{j}",
        )
    # Door articulation
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body, child=door,
        origin=Origin(xyz=(DOOR_HINGE_X, FRONT_Y, DOOR_ZC)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0, velocity=2.0, lower=0.0, upper=DOOR_OPEN,
        ),
    )

    # ==================================================================
    # Centre latch (on door near right edge, at centre seam)
    # ==================================================================
    latch = model.part("center_latch")
    latch.visual(
        Cylinder(radius=0.016, length=0.005),
        origin=Origin(xyz=(0.0, 0.0025, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_knob, name="backplate",
    )
    latch.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_knob, name="boss",
    )
    latch.visual(
        Box((0.008, 0.006, 0.030)),
        origin=Origin(xyz=(0.0, 0.018, 0.0)),
        material=steel_knob, name="latch_bar",
    )
    latch.visual(
        Sphere(radius=0.005),
        origin=Origin(xyz=(0.0, 0.018, -0.017)),
        material=steel_knob, name="latch_tip",
    )
    model.articulation(
        "latch_turn",
        ArticulationType.REVOLUTE,
        parent=door, child=latch,
        origin=Origin(xyz=(DOOR_W - 0.05, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0, velocity=4.0, lower=0.0, upper=LATCH_TURN,
        ),
    )

    # ==================================================================
    # Drawers (3 stacked, right side)
    # ==================================================================
    for i in range(N_DRAWERS):
        drawer = model.part(f"drawer_{i}")
        zc = drawer_zc[i]

        # Front panel
        drawer.visual(
            Box((DRAWER_W, DRAWER_FRONT_T, DRAWER_H)),
            origin=Origin(xyz=(0.0, -DRAWER_FRONT_T / 2.0, 0.0)),
            material=steel_drawer, name="front_panel",
        )
        # Side walls (extend back into cabinet, slight overlap with front panel)
        side_depth = DRAWER_DEPTH + 0.01
        side_h = DRAWER_H - 0.04
        for sx, sn in ((-1.0, "side_left"), (1.0, "side_right")):
            drawer.visual(
                Box((0.012, side_depth, side_h)),
                origin=Origin(
                    xyz=(
                        sx * (DRAWER_W / 2.0 - 0.006),
                        -0.01 - side_depth / 2.0,
                        0.01,
                    )
                ),
                material=steel_body, name=sn,
            )
        # Bottom panel
        drawer.visual(
            Box((DRAWER_W - 0.024, DRAWER_DEPTH, 0.012)),
            origin=Origin(
                xyz=(0.0, -DRAWER_FRONT_T - DRAWER_DEPTH / 2.0, -DRAWER_H / 2.0 + 0.006)
            ),
            material=steel_body, name="bottom",
        )
        # Back panel
        drawer.visual(
            Box((DRAWER_W - 0.024, 0.010, side_h)),
            origin=Origin(
                xyz=(0.0, -DRAWER_FRONT_T - DRAWER_DEPTH - 0.005, 0.01)
            ),
            material=steel_body, name="back_panel",
        )
        # Handle bar on front face
        drawer.visual(
            Box((0.12, 0.008, 0.018)),
            origin=Origin(xyz=(0.0, 0.004, 0.0)),
            material=steel_handle, name="handle",
        )
        for j, hx in enumerate((-0.05, 0.05)):
            drawer.visual(
                Cylinder(radius=0.005, length=0.010),
                origin=Origin(
                    xyz=(hx, -0.001, 0.0),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=steel_handle, name=f"handle_post_{j}",
            )
        # Label holder (small dark plate near top of front)
        drawer.visual(
            Box((0.06, 0.003, 0.025)),
            origin=Origin(xyz=(0.0, 0.0015, DRAWER_H / 2.0 - 0.04)),
            material=steel_dark, name="label_plate",
        )

        # Prismatic slide joint (+Y = pull outward)
        model.articulation(
            f"drawer_{i}_slide",
            ArticulationType.PRISMATIC,
            parent=body, child=drawer,
            origin=Origin(xyz=(DRAWER_XC, FRONT_Y, zc)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=30.0, velocity=0.5, lower=0.0, upper=DRAWER_TRAVEL,
            ),
        )

    return model


# ===================================================================
# Tests
# ===================================================================

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("cabinet_body")
    door = object_model.get_part("door")
    latch = object_model.get_part("center_latch")
    door_hinge = object_model.get_articulation("door_hinge")
    latch_turn = object_model.get_articulation("latch_turn")
    drawers = [object_model.get_part(f"drawer_{i}") for i in range(N_DRAWERS)]
    slides = [object_model.get_articulation(f"drawer_{i}_slide") for i in range(N_DRAWERS)]

    # --- Intentional hinge-barrel / door overlap ----------------------
    for j in range(3):
        ctx.allow_overlap(
            body, door,
            elem_a=f"hinge_barrel_{j}", elem_b="leaf",
            reason="Visible hinge barrel intentionally straddles the door hinge edge (captured knuckle).",
        )

    # --- Overall envelope ---------------------------------------------
    aabb = ctx.part_world_aabb(body)
    ok = aabb is not None
    ctx.check("cabinet body has bounds", ok, details=str(aabb))
    if aabb is not None:
        (x0, y0, z0), (x1, y1, z1) = aabb
        ctx.check("width ~1.6 m", 1.58 <= (x1 - x0) <= 1.70, details=f"w={x1 - x0:.3f}")
        ctx.check("depth ~0.5 m", 0.48 <= (y1 - y0) <= 0.58, details=f"d={y1 - y0:.3f}")
        ctx.check("height ~1.8 m", 1.78 <= z1 <= 1.86, details=f"top={z1:.3f}")
        ctx.check("casters rest on floor", abs(z0) <= 0.001, details=f"zmin={z0:.5f}")

    # --- Door ---------------------------------------------------------
    ctx.check(
        "door hinge is revolute",
        door_hinge.articulation_type == ArticulationType.REVOLUTE,
    )
    ax = door_hinge.axis
    ctx.check(
        "door hinge axis vertical",
        abs(ax[0]) < 1e-9 and abs(ax[1]) < 1e-9 and abs(abs(ax[2]) - 1.0) < 1e-9,
        details=str(ax),
    )
    lim = door_hinge.motion_limits
    ctx.check(
        "door opens 0..~110 deg",
        lim is not None and lim.lower == 0.0 and abs(lim.upper - DOOR_OPEN) < 1e-6,
    )
    # Closed: flush with front face
    daabb = ctx.part_element_world_aabb(door, elem="leaf")
    ctx.check(
        "door closed leaf flush with front",
        daabb is not None and abs(daabb[1][1] - FRONT_Y) < 1e-3,
        details=str(daabb),
    )
    ctx.expect_within(
        door, body, axes="x", margin=0.012,
        name="door stays inside cabinet width when closed",
    )
    # Open pose: swings outward
    closed_door = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: DOOR_OPEN}):
        open_door = ctx.part_world_aabb(door)
    ctx.check(
        "door swings outward past front face",
        open_door is not None and open_door[1][1] > FRONT_Y + 0.25,
        details=f"open={open_door}",
    )
    ctx.check(
        "door free edge swings leftward when open",
        closed_door is not None and open_door is not None
        and open_door[0][0] < closed_door[0][0] - 0.05,
        details=f"closed={closed_door}, open={open_door}",
    )

    # --- Visible hinge barrels ----------------------------------------
    for j in range(3):
        hb = ctx.part_element_world_aabb(body, elem=f"hinge_barrel_{j}")
        ctx.check(
            f"hinge_barrel_{j} present on door side",
            hb is not None and hb[1][1] >= FRONT_Y - 0.015
            and hb[0][2] > DOOR_Z0 and hb[1][2] < DOOR_Z1,
            details=str(hb),
        )

    # --- Centre latch -------------------------------------------------
    ctx.check(
        "latch is revolute about door normal",
        latch_turn.articulation_type == ArticulationType.REVOLUTE
        and latch_turn.axis == (0.0, 1.0, 0.0),
    )
    latch_lim = latch_turn.motion_limits
    ctx.check(
        "latch rotates 0..90 deg",
        latch_lim is not None and latch_lim.lower == 0.0
        and abs(latch_lim.upper - LATCH_TURN) < 1e-6,
    )
    ctx.expect_contact(
        latch, door, elem_a="backplate", elem_b="leaf",
        contact_tol=1e-4,
        name="latch backplate seats on door face",
    )
    tip_rest = ctx.part_element_world_aabb(latch, elem="latch_tip")
    with ctx.pose({latch_turn: LATCH_TURN}):
        tip_turned = ctx.part_element_world_aabb(latch, elem="latch_tip")
    ctx.check(
        "turning latch sweeps tip sideways",
        tip_rest is not None and tip_turned is not None
        and abs(tip_turned[0][0] - tip_rest[0][0]) > 0.010,
        details=f"rest={tip_rest}, turned={tip_turned}",
    )

    # --- Drawers ------------------------------------------------------
    for i, (drawer, slide) in enumerate(zip(drawers, slides)):
        ctx.check(
            f"drawer_{i} slide is prismatic",
            slide.articulation_type == ArticulationType.PRISMATIC,
        )
        ctx.check(
            f"drawer_{i} slides along +Y",
            slide.axis == (0.0, 1.0, 0.0),
            details=str(slide.axis),
        )
        dlim = slide.motion_limits
        ctx.check(
            f"drawer_{i} travel 0..0.35 m",
            dlim is not None and dlim.lower == 0.0
            and abs(dlim.upper - DRAWER_TRAVEL) < 1e-6,
        )
        # Closed front flush
        fp = ctx.part_element_world_aabb(drawer, elem="front_panel")
        ctx.check(
            f"drawer_{i} closed front flush with cabinet front",
            fp is not None and abs(fp[1][1] - FRONT_Y) < 1e-3,
            details=str(fp),
        )
        # Open pose: extends outward
        rest_pos = ctx.part_world_position(drawer)
        with ctx.pose({slide: DRAWER_TRAVEL}):
            ext_pos = ctx.part_world_position(drawer)
        ctx.check(
            f"drawer_{i} extends outward when opened",
            rest_pos is not None and ext_pos is not None
            and ext_pos[1] > rest_pos[1] + 0.10,
            details=f"rest={rest_pos}, ext={ext_pos}",
        )
        # Drawer stays within cabinet width
        ctx.expect_within(
            drawer, body, axes="x", margin=0.015,
            name=f"drawer_{i} stays inside cabinet width when closed",
        )

    # --- Caster blocks ------------------------------------------------
    for i in range(4):
        cb = ctx.part_element_world_aabb(body, elem=f"caster_{i}")
        ctx.check(
            f"caster_{i} present at base",
            cb is not None and cb[0][2] < 0.005 and cb[1][2] > 0.08,
            details=str(cb),
        )

    # --- Centre divider -----------------------------------------------
    cd = ctx.part_element_world_aabb(body, elem="center_divider")
    ctx.check(
        "centre divider present between door and drawer sections",
        cd is not None and abs(0.5 * (cd[0][0] + cd[1][0])) < 0.02,
        details=str(cd),
    )

    # --- Drawer section is right of door section ----------------------
    door_aabb = ctx.part_world_aabb(door)
    dr0_aabb = ctx.part_world_aabb(drawers[0])
    ctx.check(
        "drawers are right of the door",
        door_aabb is not None and dr0_aabb is not None
        and dr0_aabb[0][0] > door_aabb[0][0],
        details=f"door={door_aabb}, drawer0={dr0_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
