from __future__ import annotations

"""Vintage industrial steel locker cabinet – variant 14.

Sliding tambour-style front panel replaces the two centre doors, one mirrored
left door swings on a side hinge, shelf boards are visible through the centre
opening, and both remaining doors carry recessed panel borders.

Overall envelope ~1.6 m wide × 0.5 m deep × ~1.8 m tall, brushed/tarnished raw
steel.  A hollow thin-wall (~0.02 m) carcass sits on four splayed legs and
carries a riveted top cap strip.  The front frame has a bottom rail, a top
rail, and two intermediate stiles dividing the opening into three bays:
  • Left bay  – hinged door with a full-length mirror (revolute, opens outward).
  • Centre bay – sliding tambour panel on vertical prismatic rails.
  • Right bay – hinged door (revolute, opens outward).
Both hinged doors have recessed panel borders, a dark vent slot near the
bottom, stamped vent lines near the top, and a quarter-turn latch knob.
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
# Global dimensions (metres).  Cabinet centred on X, front face at +Y.
# ---------------------------------------------------------------------------
CAB_W = 1.60
CAB_D = 0.50
CAB_TOP = 1.80
LEG_H = 0.15
WALL_T = 0.02

FRONT_Y = CAB_D / 2.0          # +0.25
BACK_Y = -CAB_D / 2.0

BOTTOM_RAIL_TOP = LEG_H + 0.06  # 0.21
TOP_RAIL_BOT = CAB_TOP - 0.06   # 1.74

STILE_W = 0.03

# Door leaf dimensions
DOOR_W = 0.364
DOOR_T = WALL_T
DOOR_Z0 = BOTTOM_RAIL_TOP + 0.002   # 0.212
DOOR_Z1 = TOP_RAIL_BOT - 0.002      # 1.738
DOOR_H = DOOR_Z1 - DOOR_Z0          # 1.526
DOOR_ZC = 0.5 * (DOOR_Z0 + DOOR_Z1)  # 0.975
HINGE_INSET = 0.0005

# Tambour panel dimensions
TAMBOUR_W = 0.740           # clear width inside the two stiles
TAMBOUR_T = 0.018           # slat-pack thickness
TAMBOUR_H = DOOR_H - 0.006  # 1.520 – contacts bottom rail at rest
TAMBOUR_TRAVEL = 1.00       # metres of vertical travel

# Door hinge world-x positions
DOOR_LEFT_HINGE_X = -0.78 + HINGE_INSET   # far-left edge
DOOR_RIGHT_HINGE_X = 0.78 - HINGE_INSET   # far-right edge

# Vent slot (dark rounded-end through slot near the bottom of each door)
SLOT_LEN = 0.36
SLOT_W = 0.030
SLOT_ZC = -0.40             # in door-local z (door centre = 0)

# Piano-hinge knuckle column
BARREL_R = 0.0075
KNUCKLE_R = 0.0095
BARREL_LEN = DOOR_H - 0.03

# Top cap
CAP_T = 0.022
CAP_OVERHANG = 0.02

# Recessed panel border
BORDER_W = 0.035
RECESS_DEPTH = 0.005

# Joint limits
DOOR_OPEN = math.radians(110.0)
KNOB_TURN = math.radians(90.0)


# ---------------------------------------------------------------------------
# CadQuery mesh helpers
# ---------------------------------------------------------------------------

def _door_with_recessed_border(sign: float, mesh_name: str):
    """Door leaf with a stepped recessed-panel border and a rounded-end
    through vent slot.  *sign* = +1 → panel extends +X from the hinge
    (left-hinged); −1 → extends −X (right-hinged)."""
    xc = sign * DOOR_W / 2.0
    panel = (
        cq.Workplane("XY")
        .box(DOOR_W, DOOR_T, DOOR_H)
        .translate((xc, -DOOR_T / 2.0, 0.0))
    )
    # Recessed centre area – leaves a full-thickness border frame around the
    # edges.  Cutter extends 1 mm above the front face for a clean boolean.
    inner_w = DOOR_W - 2.0 * BORDER_W
    inner_h = DOOR_H - 2.0 * BORDER_W
    cut_h = RECESS_DEPTH + 0.001
    recess = (
        cq.Workplane("XY")
        .box(inner_w, cut_h, inner_h)
        .translate((xc, -(RECESS_DEPTH - 0.001) / 2.0, 0.0))
    )
    panel = panel.cut(recess)
    # Rounded-end vent slot through full leaf thickness
    slot = (
        cq.Workplane("XZ")
        .slot2D(SLOT_LEN, SLOT_W, 90)
        .extrude(0.05, both=True)
        .translate((xc, 0.0, SLOT_ZC))
    )
    panel = panel.cut(slot)
    return mesh_from_cadquery(panel, mesh_name)


def _hinge_barrel_solid(mesh_name: str):
    """Piano-hinge knuckle column along the door hinge edge."""
    barrel = cq.Workplane("XY").circle(BARREL_R).extrude(BARREL_LEN / 2.0, both=True)
    ring_h = 0.055
    for zc in (-0.60, -0.30, 0.0, 0.30, 0.60):
        ring = (
            cq.Workplane("XY")
            .circle(KNUCKLE_R)
            .extrude(ring_h / 2.0, both=True)
            .translate((0.0, 0.0, zc))
        )
        barrel = barrel.union(ring)
    return mesh_from_cadquery(barrel, mesh_name)


def _leg_solid(mesh_name: str):
    """Splayed tapered leg."""
    leg = (
        cq.Workplane("XY")
        .center(0.03, 0.03)
        .rect(0.035, 0.035)
        .workplane(offset=LEG_H + 0.01)
        .center(-0.03, -0.03)
        .rect(0.06, 0.06)
        .loft()
    )
    return mesh_from_cadquery(leg, mesh_name)


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_steel_locker_cabinet_v14")

    # -- materials ----------------------------------------------------------
    steel_body = model.material("steel_body", rgba=(0.60, 0.61, 0.63, 1.0))
    steel_door = model.material("steel_door", rgba=(0.55, 0.56, 0.58, 1.0))
    steel_trim = model.material("steel_trim", rgba=(0.46, 0.47, 0.49, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.07, 0.07, 0.08, 1.0))
    steel_leg = model.material("steel_leg", rgba=(0.38, 0.39, 0.41, 1.0))
    steel_knob = model.material("steel_knob", rgba=(0.18, 0.18, 0.20, 1.0))
    steel_rivet = model.material("steel_rivet", rgba=(0.52, 0.53, 0.55, 1.0))
    mirror_mat = model.material("mirror_glass", rgba=(0.82, 0.87, 0.91, 1.0))
    tambour_mat = model.material("tambour_steel", rgba=(0.52, 0.53, 0.56, 1.0))
    shelf_mat = model.material("shelf_steel", rgba=(0.58, 0.59, 0.61, 1.0))

    # ======================================================================
    # Cabinet body (root)
    # ======================================================================
    body = model.part("cabinet_body")
    carcass_h = CAB_TOP - LEG_H          # 1.65
    carcass_zc = LEG_H + carcass_h / 2.0  # 0.975

    # Side walls
    body.visual(
        Box((WALL_T, CAB_D, carcass_h)),
        origin=Origin(xyz=(-(CAB_W / 2.0 - WALL_T / 2.0), 0.0, carcass_zc)),
        material=steel_body, name="side_wall_0",
    )
    body.visual(
        Box((WALL_T, CAB_D, carcass_h)),
        origin=Origin(xyz=(+(CAB_W / 2.0 - WALL_T / 2.0), 0.0, carcass_zc)),
        material=steel_body, name="side_wall_1",
    )
    # Back wall
    body.visual(
        Box((CAB_W - WALL_T, WALL_T, carcass_h - 0.02)),
        origin=Origin(xyz=(0.0, BACK_Y + WALL_T / 2.0, carcass_zc)),
        material=steel_body, name="back_wall",
    )
    # Bottom and top panels
    body.visual(
        Box((CAB_W, CAB_D, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, LEG_H + WALL_T / 2.0)),
        material=steel_body, name="bottom_panel",
    )
    body.visual(
        Box((CAB_W, CAB_D, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, CAB_TOP - WALL_T / 2.0)),
        material=steel_body, name="top_panel",
    )

    # Front frame – bottom rail, top rail, two stiles (no centre stile)
    body.visual(
        Box((CAB_W - 2.0 * WALL_T, WALL_T, BOTTOM_RAIL_TOP - LEG_H + 0.01)),
        origin=Origin(
            xyz=(0.0, FRONT_Y - WALL_T / 2.0, (LEG_H + BOTTOM_RAIL_TOP) / 2.0)
        ),
        material=steel_body, name="front_bottom_rail",
    )
    body.visual(
        Box((CAB_W - 2.0 * WALL_T, WALL_T, CAB_TOP - TOP_RAIL_BOT + 0.01)),
        origin=Origin(
            xyz=(0.0, FRONT_Y - WALL_T / 2.0, (TOP_RAIL_BOT + CAB_TOP) / 2.0)
        ),
        material=steel_body, name="front_top_rail",
    )
    stile_h = TOP_RAIL_BOT - BOTTOM_RAIL_TOP + 0.01
    for i, xc in enumerate((-0.3975, 0.3975)):
        body.visual(
            Box((STILE_W, WALL_T, stile_h)),
            origin=Origin(xyz=(xc, FRONT_Y - WALL_T / 2.0, DOOR_ZC)),
            material=steel_trim, name=f"front_stile_{i}",
        )

    # Riveted top cap strip
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

    # Splayed legs
    leg_mesh = _leg_solid("splayed_leg")
    leg_corners = [
        (0.72, 0.19, 0.0),
        (-0.72, 0.19, math.pi / 2.0),
        (-0.72, -0.19, math.pi),
        (0.72, -0.19, 3.0 * math.pi / 2.0),
    ]
    for i, (lx, ly, yaw) in enumerate(leg_corners):
        body.visual(
            leg_mesh,
            origin=Origin(xyz=(lx, ly, 0.0), rpy=(0.0, 0.0, yaw)),
            material=steel_leg, name=f"leg_{i}",
        )

    # Tambour guide rails (thin vertical tracks contacting panel edges)
    rail_w = 0.012
    rail_d = TAMBOUR_T  # matches panel thickness for full contact
    rail_h = DOOR_H + 0.02
    for i, sx in enumerate((-1, 1)):
        rail_x = sx * (TAMBOUR_W / 2.0 + rail_w / 2.0)
        body.visual(
            Box((rail_w, rail_d, rail_h)),
            origin=Origin(
                xyz=(rail_x, FRONT_Y - TAMBOUR_T / 2.0, DOOR_ZC)
            ),
            material=steel_trim, name=f"tambour_rail_{i}",
        )

    # Tambour housing (roll pocket at the top of the centre bay)
    housing_w = TAMBOUR_W + 0.04
    housing_d = 0.10
    housing_h = 0.07
    body.visual(
        Box((housing_w, housing_d, housing_h)),
        origin=Origin(
            xyz=(0.0, FRONT_Y - WALL_T - 0.01 - housing_d / 2.0,
                 CAB_TOP - WALL_T - housing_h / 2.0)
        ),
        material=steel_trim, name="tambour_housing",
    )

    # Centre-section shelf boards (visible when tambour is raised)
    shelf_w = TAMBOUR_W - 0.04
    shelf_d = CAB_D - 2.0 * WALL_T - 0.04
    for i, sz in enumerate((0.50, 0.85, 1.20)):
        body.visual(
            Box((shelf_w, shelf_d, 0.015)),
            origin=Origin(xyz=(0.0, -0.02, sz)),
            material=shelf_mat, name=f"center_shelf_{i}",
        )

    # ======================================================================
    # Door 0 – left bay (hinged on left edge, mirrored)
    # ======================================================================
    sign_left = +1.0
    xc_left = sign_left * DOOR_W / 2.0

    door_left = model.part("door_0")
    door_left.visual(
        _door_with_recessed_border(sign_left, "door_leaf_0"),
        material=steel_door, name="leaf",
    )
    # Dark backing behind the through vent slot
    door_left.visual(
        Box((SLOT_W + 0.016, 0.005, SLOT_LEN + 0.036)),
        origin=Origin(xyz=(xc_left, -DOOR_T - 0.001, SLOT_ZC)),
        material=steel_dark, name="vent_backing",
    )
    # Stamped vent lines near the top (seated on recessed inner face)
    for j, dz in enumerate((0.60, 0.62, 0.64)):
        door_left.visual(
            Box((0.16, 0.004, 0.006)),
            origin=Origin(xyz=(xc_left, -RECESS_DEPTH + 0.001, dz)),
            material=steel_dark, name=f"vent_line_{j}",
        )
    # Piano-hinge knuckle column
    door_left.visual(
        _hinge_barrel_solid("hinge_barrel_0"),
        origin=Origin(xyz=(0.0, 0.004, 0.0)),
        material=steel_trim, name="hinge_barrel",
    )
    # Mirror panel – seated on the recessed inner face, inside the border
    mirror_w = DOOR_W - 2.0 * BORDER_W - 0.010  # ~0.284
    mirror_h = 0.86                                 # upper portion of door
    mirror_t = 0.004
    mirror_zc = 0.22                                # above centre, above slot
    door_left.visual(
        Box((mirror_w, mirror_t, mirror_h)),
        origin=Origin(
            xyz=(xc_left, -RECESS_DEPTH + mirror_t / 2.0, mirror_zc)
        ),
        material=mirror_mat, name="mirror_panel",
    )

    model.articulation(
        "door_0_hinge",
        ArticulationType.REVOLUTE,
        parent=body, child=door_left,
        origin=Origin(xyz=(DOOR_LEFT_HINGE_X, FRONT_Y, DOOR_ZC)),
        axis=(0.0, 0.0, 1.0),          # +Z opens left-hinged leaf outward
        motion_limits=MotionLimits(
            effort=40.0, velocity=2.0, lower=0.0, upper=DOOR_OPEN
        ),
    )

    # Latch knob on door 0
    knob_0 = model.part("latch_knob_0")
    knob_0.visual(
        Cylinder(radius=0.018, length=0.005),
        origin=Origin(xyz=(0.0, 0.0025, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_knob, name="backplate",
    )
    knob_0.visual(
        Cylinder(radius=0.0065, length=0.014),
        origin=Origin(xyz=(0.0, 0.011, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_knob, name="boss",
    )
    knob_0.visual(
        Box((0.010, 0.008, 0.034)),
        origin=Origin(xyz=(0.0, 0.020, 0.0)),
        material=steel_knob, name="handle_bar",
    )
    knob_0.visual(
        Sphere(radius=0.006),
        origin=Origin(xyz=(0.0, 0.020, -0.019)),
        material=steel_knob, name="handle_tip",
    )
    model.articulation(
        "latch_0",
        ArticulationType.REVOLUTE,
        parent=door_left, child=knob_0,
        origin=Origin(xyz=(sign_left * (DOOR_W - 0.02), 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0, velocity=4.0, lower=0.0, upper=KNOB_TURN
        ),
    )

    # ======================================================================
    # Door 3 – right bay (hinged on right edge)
    # ======================================================================
    sign_right = -1.0
    xc_right = sign_right * DOOR_W / 2.0

    door_right = model.part("door_3")
    door_right.visual(
        _door_with_recessed_border(sign_right, "door_leaf_3"),
        material=steel_door, name="leaf",
    )
    door_right.visual(
        Box((SLOT_W + 0.016, 0.005, SLOT_LEN + 0.036)),
        origin=Origin(xyz=(xc_right, -DOOR_T - 0.001, SLOT_ZC)),
        material=steel_dark, name="vent_backing",
    )
    for j, dz in enumerate((0.60, 0.62, 0.64)):
        door_right.visual(
            Box((0.16, 0.004, 0.006)),
            origin=Origin(xyz=(xc_right, -RECESS_DEPTH + 0.001, dz)),
            material=steel_dark, name=f"vent_line_{j}",
        )
    door_right.visual(
        _hinge_barrel_solid("hinge_barrel_3"),
        origin=Origin(xyz=(0.0, 0.004, 0.0)),
        material=steel_trim, name="hinge_barrel",
    )

    model.articulation(
        "door_3_hinge",
        ArticulationType.REVOLUTE,
        parent=body, child=door_right,
        origin=Origin(xyz=(DOOR_RIGHT_HINGE_X, FRONT_Y, DOOR_ZC)),
        axis=(0.0, 0.0, -1.0),         # −Z opens right-hinged leaf outward
        motion_limits=MotionLimits(
            effort=40.0, velocity=2.0, lower=0.0, upper=DOOR_OPEN
        ),
    )

    knob_3 = model.part("latch_knob_3")
    knob_3.visual(
        Cylinder(radius=0.018, length=0.005),
        origin=Origin(xyz=(0.0, 0.0025, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_knob, name="backplate",
    )
    knob_3.visual(
        Cylinder(radius=0.0065, length=0.014),
        origin=Origin(xyz=(0.0, 0.011, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_knob, name="boss",
    )
    knob_3.visual(
        Box((0.010, 0.008, 0.034)),
        origin=Origin(xyz=(0.0, 0.020, 0.0)),
        material=steel_knob, name="handle_bar",
    )
    knob_3.visual(
        Sphere(radius=0.006),
        origin=Origin(xyz=(0.0, 0.020, -0.019)),
        material=steel_knob, name="handle_tip",
    )
    model.articulation(
        "latch_3",
        ArticulationType.REVOLUTE,
        parent=door_right, child=knob_3,
        origin=Origin(xyz=(sign_right * (DOOR_W - 0.02), 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0, velocity=4.0, lower=0.0, upper=KNOB_TURN
        ),
    )

    # ======================================================================
    # Tambour panel – centre bay (prismatic, slides upward)
    # ======================================================================
    tambour = model.part("tambour_panel")
    # Main panel body (flat slab)
    tambour.visual(
        Box((TAMBOUR_W, TAMBOUR_T, TAMBOUR_H)),
        material=tambour_mat, name="panel",
    )
    # Horizontal slat-groove lines on the front face (thin dark strips)
    n_grooves = 7
    groove_span = TAMBOUR_H - 0.20
    for j in range(n_grooves):
        gz = -groove_span / 2.0 + j * groove_span / (n_grooves - 1)
        tambour.visual(
            Box((TAMBOUR_W - 0.04, 0.004, 0.005)),
            origin=Origin(xyz=(0.0, TAMBOUR_T / 2.0 + 0.001, gz)),
            material=steel_dark, name=f"slat_groove_{j}",
        )
    # Bottom pull handle
    tambour.visual(
        Box((0.14, 0.022, 0.018)),
        origin=Origin(
            xyz=(0.0, TAMBOUR_T / 2.0 + 0.009, -TAMBOUR_H / 2.0 + 0.04)
        ),
        material=steel_knob, name="pull_handle",
    )
    # Bottom edge cap strip
    tambour.visual(
        Box((TAMBOUR_W - 0.02, 0.005, 0.024)),
        origin=Origin(
            xyz=(0.0, TAMBOUR_T / 2.0 + 0.001, -TAMBOUR_H / 2.0 + 0.012)
        ),
        material=steel_trim, name="bottom_cap",
    )

    model.articulation(
        "tambour_slide",
        ArticulationType.PRISMATIC,
        parent=body, child=tambour,
        origin=Origin(xyz=(0.0, FRONT_Y - TAMBOUR_T / 2.0, DOOR_ZC)),
        axis=(0.0, 0.0, 1.0),           # slides upward along +Z
        motion_limits=MotionLimits(
            effort=30.0, velocity=0.5, lower=0.0, upper=TAMBOUR_TRAVEL
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("cabinet_body")
    door_left = object_model.get_part("door_0")
    door_right = object_model.get_part("door_3")
    hinge_left = object_model.get_articulation("door_0_hinge")
    hinge_right = object_model.get_articulation("door_3_hinge")
    tambour = object_model.get_part("tambour_panel")
    tambour_slide = object_model.get_articulation("tambour_slide")
    knob_0 = object_model.get_part("latch_knob_0")
    knob_3 = object_model.get_part("latch_knob_3")
    latch_0 = object_model.get_articulation("latch_0")
    latch_3 = object_model.get_articulation("latch_3")

    # -- overlap allowances ------------------------------------------------
    # Piano-hinge knuckle columns lap the fixed frame edges they pivot on.
    ctx.allow_overlap(
        door_left, body,
        elem_a="hinge_barrel", elem_b="side_wall_0",
        reason="Piano-hinge knuckle column intentionally laps the frame edge it pivots on.",
    )
    ctx.allow_overlap(
        door_right, body,
        elem_a="hinge_barrel", elem_b="side_wall_1",
        reason="Piano-hinge knuckle column intentionally laps the frame edge it pivots on.",
    )

    # -- overall envelope --------------------------------------------------
    aabb = ctx.part_world_aabb(body)
    ctx.check("cabinet body has bounds", aabb is not None, details=str(aabb))
    if aabb is not None:
        (x0, y0, z0), (x1, y1, z1) = aabb
        ctx.check("width ~1.6 m", 1.58 <= (x1 - x0) <= 1.70, details=f"w={x1-x0:.3f}")
        ctx.check("depth ~0.5 m", 0.48 <= (y1 - y0) <= 0.58, details=f"d={y1-y0:.3f}")
        ctx.check("height ~1.8 m", 1.78 <= z1 <= 1.86, details=f"top={z1:.3f}")
        ctx.check("legs on floor", abs(z0) <= 1e-6, details=f"zmin={z0:.6f}")

    # -- door hinges -------------------------------------------------------
    for door, hinge, name, expected_axis_z in (
        (door_left, hinge_left, "door_0", +1.0),
        (door_right, hinge_right, "door_3", -1.0),
    ):
        ctx.check(
            f"{name} hinge is revolute",
            hinge.articulation_type == ArticulationType.REVOLUTE,
        )
        ax = hinge.axis
        ctx.check(
            f"{name} hinge axis is vertical",
            abs(ax[0]) < 1e-9 and abs(ax[1]) < 1e-9 and abs(ax[2] - expected_axis_z) < 1e-9,
            details=str(ax),
        )
        lim = hinge.motion_limits
        ctx.check(
            f"{name} opens 0..~110°",
            lim is not None and lim.lower == 0.0 and abs(lim.upper - DOOR_OPEN) < 1e-6,
        )
        # Closed leaf flush with front face
        daabb = ctx.part_element_world_aabb(door, elem="leaf")
        ctx.check(
            f"{name} closed leaf flush with front face",
            daabb is not None and abs(daabb[1][1] - FRONT_Y) < 1e-3,
            details=str(daabb),
        )

    # Hinge sides
    ctx.check(
        "left door hinges at left edge",
        hinge_left.origin.xyz[0] < -0.77,
    )
    ctx.check(
        "right door hinges at right edge",
        hinge_right.origin.xyz[0] > 0.77,
    )

    # Opening poses – doors swing outward past the front face
    with ctx.pose({hinge_left: DOOR_OPEN}):
        open_left = ctx.part_world_aabb(door_left)
    with ctx.pose({hinge_right: DOOR_OPEN}):
        open_right = ctx.part_world_aabb(door_right)
    ctx.check(
        "left door swings outward",
        open_left is not None and open_left[1][1] > FRONT_Y + 0.20,
        details=str(open_left),
    )
    ctx.check(
        "right door swings outward",
        open_right is not None and open_right[1][1] > FRONT_Y + 0.20,
        details=str(open_right),
    )

    # -- mirror on left door -----------------------------------------------
    mirror_aabb = ctx.part_element_world_aabb(door_left, elem="mirror_panel")
    ctx.check(
        "mirror panel exists on left door",
        mirror_aabb is not None,
        details=str(mirror_aabb),
    )
    if mirror_aabb is not None:
        mw = mirror_aabb[1][0] - mirror_aabb[0][0]
        mh = mirror_aabb[1][2] - mirror_aabb[0][2]
        ctx.check(
            "mirror covers upper door area (>0.20 m wide, >0.70 m tall)",
            mw > 0.20 and mh > 0.70,
            details=f"mw={mw:.3f} mh={mh:.3f}",
        )
        ctx.check(
            "mirror sits on door front face",
            mirror_aabb[1][1] > FRONT_Y - 0.015 and mirror_aabb[0][1] > FRONT_Y - 0.020,
            details=f"y_range=[{mirror_aabb[0][1]:.4f}, {mirror_aabb[1][1]:.4f}]",
        )

    # -- recessed panel borders -------------------------------------------
    for door, name in ((door_left, "door_0"), (door_right, "door_3")):
        leaf_aabb = ctx.part_element_world_aabb(door, elem="leaf")
        ctx.check(
            f"{name} leaf has recessed-border panel",
            leaf_aabb is not None,
            details=str(leaf_aabb),
        )
        if leaf_aabb is not None:
            lw = leaf_aabb[1][0] - leaf_aabb[0][0]
            lh = leaf_aabb[1][2] - leaf_aabb[0][2]
            ctx.check(
                f"{name} leaf outer dims match door size",
                abs(lw - DOOR_W) < 0.01 and abs(lh - DOOR_H) < 0.01,
                details=f"lw={lw:.3f} lh={lh:.3f}",
            )

    # -- tambour panel (prismatic slide) ----------------------------------
    ctx.check(
        "tambour_slide is prismatic",
        tambour_slide.articulation_type == ArticulationType.PRISMATIC,
    )
    sax = tambour_slide.axis
    ctx.check(
        "tambour axis is vertical +Z",
        abs(sax[0]) < 1e-9 and abs(sax[1]) < 1e-9 and abs(sax[2] - 1.0) < 1e-9,
        details=str(sax),
    )
    slim = tambour_slide.motion_limits
    ctx.check(
        "tambour travel 0..~1.0 m",
        slim is not None and slim.lower == 0.0 and abs(slim.upper - TAMBOUR_TRAVEL) < 0.01,
        details=f"limits={slim}",
    )

    # At rest the tambour sits in the centre bay at the front face
    rest_pos = ctx.part_world_position(tambour)
    rest_aabb = ctx.part_world_aabb(tambour)
    ctx.check(
        "tambour at rest is at front-face centre",
        rest_aabb is not None
        and abs(rest_aabb[1][1] - FRONT_Y) < 0.025
        and abs(rest_aabb[0][1] - (FRONT_Y - TAMBOUR_T)) < 0.025,
        details=str(rest_aabb),
    )
    ctx.expect_within(
        tambour, body,
        axes="x",
        margin=0.02,
        name="tambour stays inside cabinet width at rest",
    )

    # Opened pose – panel has moved upward
    with ctx.pose({tambour_slide: TAMBOUR_TRAVEL}):
        open_pos = ctx.part_world_position(tambour)
    ctx.check(
        "tambour moves upward when opened",
        rest_pos is not None and open_pos is not None
        and open_pos[2] > rest_pos[2] + 0.5,
        details=f"rest_z={rest_pos[2]:.3f} open_z={open_pos[2]:.3f}",
    )

    # -- shelf boards visible through centre opening -----------------------
    for i in range(3):
        saabb = ctx.part_element_world_aabb(body, elem=f"center_shelf_{i}")
        ctx.check(
            f"center_shelf_{i} exists in centre bay",
            saabb is not None
            and saabb[0][0] > -0.42
            and saabb[1][0] < 0.42
            and saabb[0][2] > LEG_H
            and saabb[1][2] < CAB_TOP,
            details=str(saabb),
        )

    # -- latch knobs -------------------------------------------------------
    for knob, latch, door, name in (
        (knob_0, latch_0, door_left, "latch_0"),
        (knob_3, latch_3, door_right, "latch_3"),
    ):
        ctx.check(
            f"{name} is quarter-turn revolute",
            latch.articulation_type == ArticulationType.REVOLUTE
            and latch.axis == (0.0, 1.0, 0.0)
            and latch.motion_limits is not None
            and abs(latch.motion_limits.upper - math.pi / 2.0) < 1e-6,
        )
        ctx.expect_contact(
            knob, door,
            elem_a="backplate", elem_b="leaf",
            contact_tol=1e-6,
            name=f"{name} backplate seats on leaf",
        )

    # Knob handle-tip sweep proves rotation
    tip_rest = ctx.part_element_world_aabb(knob_0, elem="handle_tip")
    with ctx.pose({latch_0: KNOB_TURN}):
        tip_turn = ctx.part_element_world_aabb(knob_0, elem="handle_tip")
    ctx.check(
        "turning latch_0 sweeps the handle tip",
        tip_rest is not None and tip_turn is not None
        and abs(tip_turn[0][0] - tip_rest[0][0]) > 0.010
        and tip_turn[0][2] > tip_rest[0][2] + 0.010,
        details=f"rest={tip_rest}, turned={tip_turn}",
    )

    # -- rivets present on top rail ----------------------------------------
    rivet_aabb = ctx.part_element_world_aabb(body, elem="rivet_0")
    ctx.check(
        "rivet dots proud of top rail face",
        rivet_aabb is not None and rivet_aabb[1][1] > FRONT_Y + 0.003,
    )

    return ctx.report()


object_model = build_object_model()
