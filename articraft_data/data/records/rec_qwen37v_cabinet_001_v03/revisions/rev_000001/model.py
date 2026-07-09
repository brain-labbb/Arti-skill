from __future__ import annotations

"""Vintage industrial steel cabinet — asymmetric door + drawers variant.

Reference image: picture/Other/Cabinet/001.png

Overall envelope ~1.6 m wide x 0.5 m deep x ~1.8 m tall, brushed/tarnished raw
steel. A hollow thin-wall (~0.02 m) carcass sits on four short splayed legs and
carries a thin riveted top cap strip. The front is divided asymmetrically:

- Left half: one full-height hinged door, hinged on its left edge at the left
  side wall. The door carries a dark recessed ventilation slot with rounded
  ends near the bottom, stamped vent lines near the top, and a quarter-turn
  latch knob at mid-height. Three groups of visible barrel-hinge knuckles run
  along the hinge edge.

- Right half: three stacked drawers on prismatic slides. Each drawer is a
  hollow open-top steel tray with a flat front panel and a small pull handle.
  Drawers slide outward along +Y, range 0 to ~0.38 m.

A vertical center-divider stile separates the door opening from the drawer
opening, and two horizontal rails separate the drawer openings from each other.
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
# Global dimensions (meters). Cabinet is centred on X, front face at +Y.
# ---------------------------------------------------------------------------
CAB_W = 1.60  # overall carcass width  (X)
CAB_D = 0.50  # overall carcass depth  (Y)
CAB_TOP = 1.80  # carcass top height   (Z)
LEG_H = 0.15  # splayed leg height; carcass starts here
WALL_T = 0.02  # thin steel wall thickness

FRONT_Y = CAB_D / 2.0  # +0.25, front face plane
BACK_Y = -CAB_D / 2.0

BOTTOM_RAIL_TOP = LEG_H + 0.06  # 0.21
TOP_RAIL_BOT = CAB_TOP - 0.06  # 1.74

STILE_W = 0.03  # center divider stile width (X)
H_RAIL_H = 0.025  # horizontal rail height (Z) between drawers

DOOR_Z0 = BOTTOM_RAIL_TOP + 0.002  # 0.212
DOOR_Z1 = TOP_RAIL_BOT - 0.002  # 1.738
DOOR_H = DOOR_Z1 - DOOR_Z0  # 1.526
DOOR_ZC = 0.5 * (DOOR_Z0 + DOOR_Z1)  # 0.975

# Left pocket: from left wall inner face to center divider left face.
LEFT_POCKET_L = -(CAB_W / 2.0 - WALL_T)  # -0.78
LEFT_POCKET_R = -STILE_W / 2.0  # -0.015

DOOR_W = (LEFT_POCKET_R - LEFT_POCKET_L) - 0.005  # 0.760
DOOR_T = WALL_T
HINGE_X = LEFT_POCKET_L + 0.001  # hinge line just inside left wall

# Right pocket: from center divider right face to right wall inner face.
RIGHT_POCKET_L = STILE_W / 2.0  # 0.015
RIGHT_POCKET_R = CAB_W / 2.0 - WALL_T  # 0.78
POCKET_W = RIGHT_POCKET_R - RIGHT_POCKET_L  # 0.765
DRAWER_XC = 0.5 * (RIGHT_POCKET_L + RIGHT_POCKET_R)  # 0.3975

# Drawer dimensions.
N_DRAWERS = 3
DRAWER_EDGE_GAP = 0.008  # gap above bottom rail and below top rail
_available = DOOR_H - 2 * DRAWER_EDGE_GAP - (N_DRAWERS - 1) * H_RAIL_H
DRAWER_H = _available / N_DRAWERS  # ~0.487
DRAWER_W = POCKET_W - 0.020  # 0.745, 0.010 clearance per side
DRAWER_FRONT_T = WALL_T  # 0.02
DRAWER_D = 0.42  # tray depth behind the front panel
DRAWER_TOTAL_D = DRAWER_FRONT_T + DRAWER_D  # 0.44

# Drawer Z centers (bottom to top).
DRAWER_ZCS = []
_z = DOOR_Z0 + DRAWER_EDGE_GAP + DRAWER_H / 2.0
for _i in range(N_DRAWERS):
    DRAWER_ZCS.append(_z)
    _z += DRAWER_H + H_RAIL_H

# Horizontal rail Z centers (between drawers).
H_RAIL_ZCS = []
_z = DOOR_Z0 + DRAWER_EDGE_GAP + DRAWER_H
for _i in range(N_DRAWERS - 1):
    H_RAIL_ZCS.append(_z + H_RAIL_H / 2.0)
    _z += H_RAIL_H + DRAWER_H

MAX_SLIDE = 0.38  # max drawer extension

SLOT_LEN = 0.36  # dark rounded-end vent slot near the bottom of the door
SLOT_W = 0.030
SLOT_ZC = -0.40  # in door-local z (door centre = 0)

BARREL_R = 0.009
KNUCKLE_R = 0.013
BARREL_LEN = DOOR_H - 0.06

CAP_T = 0.022  # riveted top cap strip
CAP_OVERHANG = 0.02

DOOR_OPEN = math.radians(110.0)
KNOB_TURN = math.radians(90.0)


def _door_solid(mesh_name: str):
    """Door leaf: flat panel with a rounded-end through slot near the bottom.
    Panel extends along +X from the hinge line (left-hinged)."""
    xc = DOOR_W / 2.0
    panel = (
        cq.Workplane("XY")
        .box(DOOR_W, DOOR_T, DOOR_H)
        .translate((xc, -DOOR_T / 2.0, 0.0))
    )
    # Vertical slot, rounded ends, cut through the leaf thickness.
    cutter = (
        cq.Workplane("XZ")
        .slot2D(SLOT_LEN, SLOT_W, 90)
        .extrude(0.05, both=True)
        .translate((xc, 0.0, SLOT_ZC))
    )
    leaf = panel.cut(cutter)
    return mesh_from_cadquery(leaf, mesh_name)


def _visible_hinge_barrels(mesh_name: str):
    """Three groups of visible barrel-hinge knuckles along the door hinge edge.
    Each group has three knuckle rings; groups are spaced along the height."""
    barrel = (
        cq.Workplane("XY")
        .circle(BARREL_R)
        .extrude(BARREL_LEN / 2.0, both=True)
    )
    ring_h = 0.040
    # Three groups: bottom, middle, top.
    group_centers = [-0.50, 0.0, 0.50]
    for gc in group_centers:
        for dz in (-0.028, 0.0, 0.028):
            ring = (
                cq.Workplane("XY")
                .circle(KNUCKLE_R)
                .extrude(ring_h / 2.0, both=True)
                .translate((0.0, 0.0, gc + dz))
            )
            barrel = barrel.union(ring)
    return mesh_from_cadquery(barrel, mesh_name)


def _drawer_tray(mesh_name: str):
    """Hollow open-top drawer tray with a solid front panel."""
    w = DRAWER_W
    d = DRAWER_TOTAL_D
    h = DRAWER_H
    t = WALL_T
    # Outer shell: front face at y=0, extends along -Y.
    outer = cq.Workplane("XY").box(w, d, h).translate((0.0, -d / 2.0, 0.0))
    # Inner cavity: recessed from front by t (front panel), open at top,
    # leaves t-thick walls on sides, back, and bottom.
    inner = (
        cq.Workplane("XY")
        .box(w - 2.0 * t, d - 2.0 * t, h - t)
        .translate((0.0, -d / 2.0, t / 2.0))
    )
    tray = outer.cut(inner)
    return mesh_from_cadquery(tray, mesh_name)


def _leg_solid(mesh_name: str):
    """Splayed tapered leg: small foot on the floor, wide top embedded into
    the carcass bottom."""
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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_steel_cabinet_door_drawers")

    steel_body = model.material("steel_body", rgba=(0.60, 0.61, 0.63, 1.0))
    steel_door = model.material("steel_door", rgba=(0.55, 0.56, 0.58, 1.0))
    steel_drawer = model.material("steel_drawer", rgba=(0.52, 0.53, 0.56, 1.0))
    steel_trim = model.material("steel_trim", rgba=(0.46, 0.47, 0.49, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.07, 0.07, 0.08, 1.0))
    steel_leg = model.material("steel_leg", rgba=(0.38, 0.39, 0.41, 1.0))
    steel_knob = model.material("steel_knob", rgba=(0.18, 0.18, 0.20, 1.0))
    steel_rivet = model.material("steel_rivet", rgba=(0.52, 0.53, 0.55, 1.0))
    steel_handle = model.material("steel_handle", rgba=(0.25, 0.25, 0.28, 1.0))

    # ------------------------------------------------------------------
    # Cabinet body: hollow carcass + legs + front frame + top cap + rivets
    # ------------------------------------------------------------------
    body = model.part("cabinet_body")
    carcass_h = CAB_TOP - LEG_H  # 1.65
    carcass_zc = LEG_H + carcass_h / 2.0

    # Side walls (full depth, full carcass height).
    for sx, vname in ((-1.0, "side_wall_0"), (1.0, "side_wall_1")):
        body.visual(
            Box((WALL_T, CAB_D, carcass_h)),
            origin=Origin(xyz=(sx * (CAB_W / 2.0 - WALL_T / 2.0), 0.0, carcass_zc)),
            material=steel_body,
            name=vname,
        )
    # Back wall.
    body.visual(
        Box((CAB_W - WALL_T, WALL_T, carcass_h - 0.02)),
        origin=Origin(xyz=(0.0, BACK_Y + WALL_T / 2.0, carcass_zc)),
        material=steel_body,
        name="back_wall",
    )
    # Bottom and top panels.
    body.visual(
        Box((CAB_W, CAB_D, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, LEG_H + WALL_T / 2.0)),
        material=steel_body,
        name="bottom_panel",
    )
    body.visual(
        Box((CAB_W, CAB_D, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, CAB_TOP - WALL_T / 2.0)),
        material=steel_body,
        name="top_panel",
    )
    # Interior shelf on the left side (behind the door).
    shelf_w = abs(LEFT_POCKET_R - LEFT_POCKET_L) - 0.01
    shelf_xc = 0.5 * (LEFT_POCKET_L + LEFT_POCKET_R)
    body.visual(
        Box((shelf_w, 0.43, 0.015)),
        origin=Origin(xyz=(shelf_xc, -0.02, 0.95)),
        material=steel_body,
        name="interior_shelf",
    )
    # Front frame: bottom rail, top rail (full width).
    body.visual(
        Box((CAB_W - 2.0 * WALL_T, WALL_T, BOTTOM_RAIL_TOP - LEG_H + 0.01)),
        origin=Origin(
            xyz=(0.0, FRONT_Y - WALL_T / 2.0, (LEG_H + BOTTOM_RAIL_TOP) / 2.0)
        ),
        material=steel_body,
        name="front_bottom_rail",
    )
    body.visual(
        Box((CAB_W - 2.0 * WALL_T, WALL_T, CAB_TOP - TOP_RAIL_BOT + 0.01)),
        origin=Origin(xyz=(0.0, FRONT_Y - WALL_T / 2.0, (TOP_RAIL_BOT + CAB_TOP) / 2.0)),
        material=steel_body,
        name="front_top_rail",
    )
    # Center vertical divider stile (separates door from drawers).
    stile_h = TOP_RAIL_BOT - BOTTOM_RAIL_TOP + 0.01
    body.visual(
        Box((STILE_W, WALL_T, stile_h)),
        origin=Origin(xyz=(0.0, FRONT_Y - WALL_T / 2.0, DOOR_ZC)),
        material=steel_trim,
        name="center_divider",
    )
    # Horizontal rails between drawers (right half only).
    h_rail_w = RIGHT_POCKET_R - RIGHT_POCKET_L  # spans the right pocket
    for i, rz in enumerate(H_RAIL_ZCS):
        body.visual(
            Box((h_rail_w, WALL_T, H_RAIL_H)),
            origin=Origin(xyz=(DRAWER_XC, FRONT_Y - WALL_T / 2.0, rz)),
            material=steel_trim,
            name=f"h_rail_{i}",
        )
    # Thin riveted top cap strip with a slight overhang.
    body.visual(
        Box((CAB_W + 2 * CAP_OVERHANG, CAB_D + 2 * CAP_OVERHANG, CAP_T)),
        origin=Origin(xyz=(0.0, 0.0, CAB_TOP - 0.001 + CAP_T / 2.0)),
        material=steel_trim,
        name="top_cap",
    )
    # Raised rivet dots along the top rail.
    n_riv = 13
    for i in range(n_riv):
        rx = -0.72 + i * (1.44 / (n_riv - 1))
        body.visual(
            Sphere(radius=0.0045),
            origin=Origin(xyz=(rx, FRONT_Y + 0.002, 1.77)),
            material=steel_rivet,
            name=f"rivet_{i}",
        )

    # Splayed legs.
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
            material=steel_leg,
            name=f"leg_{i}",
        )

    # ------------------------------------------------------------------
    # Door (left side): full-height leaf hinged on its left edge.
    # ------------------------------------------------------------------
    door = model.part("door")
    door.visual(
        _door_solid("door_leaf"),
        material=steel_door,
        name="leaf",
    )
    # Dark backing plate behind the through slot -> recessed dark slot.
    door.visual(
        Box((SLOT_W + 0.016, 0.005, SLOT_LEN + 0.036)),
        origin=Origin(xyz=(DOOR_W / 2.0, -DOOR_T - 0.001, SLOT_ZC)),
        material=steel_dark,
        name="vent_backing",
    )
    # Stamped vent lines near the top (slightly proud thin dark strips).
    for j, dz in enumerate((0.60, 0.62, 0.64)):
        door.visual(
            Box((0.30, 0.004, 0.006)),
            origin=Origin(xyz=(DOOR_W / 2.0, -0.0012, dz)),
            material=steel_dark,
            name=f"vent_line_{j}",
        )
    # Visible barrel-hinge knuckle column on the hinge edge.
    door.visual(
        _visible_hinge_barrels("hinge_barrels"),
        origin=Origin(xyz=(0.0, 0.004, 0.0)),
        material=steel_trim,
        name="hinge_barrel",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(HINGE_X, FRONT_Y, DOOR_ZC)),
        # Door extends +X from hinge; +Z axis rotates +X toward +Y (outward).
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0, velocity=2.0, lower=0.0, upper=DOOR_OPEN
        ),
    )

    # Quarter-turn latch knob at mid-height near the free edge of the door.
    knob = model.part("latch_knob")
    knob.visual(
        Cylinder(radius=0.018, length=0.005),
        origin=Origin(xyz=(0.0, 0.0025, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_knob,
        name="backplate",
    )
    knob.visual(
        Cylinder(radius=0.0065, length=0.014),
        origin=Origin(xyz=(0.0, 0.011, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_knob,
        name="boss",
    )
    knob.visual(
        Box((0.010, 0.008, 0.034)),
        origin=Origin(xyz=(0.0, 0.020, 0.0)),
        material=steel_knob,
        name="handle_bar",
    )
    knob.visual(
        Sphere(radius=0.006),
        origin=Origin(xyz=(0.0, 0.020, -0.019)),
        material=steel_knob,
        name="handle_tip",
    )
    model.articulation(
        "latch",
        ArticulationType.REVOLUTE,
        parent=door,
        child=knob,
        origin=Origin(xyz=(DOOR_W - 0.10, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0, velocity=4.0, lower=0.0, upper=KNOB_TURN
        ),
    )

    # ------------------------------------------------------------------
    # Drawers (right side): three stacked prismatic-sliding trays.
    # ------------------------------------------------------------------
    drawer_tray_mesh = _drawer_tray("drawer_tray")
    drawers = []
    for i in range(N_DRAWERS):
        drawer = model.part(f"drawer_{i}")
        # Tray body (front face centered at local origin, extends -Y).
        drawer.visual(
            drawer_tray_mesh,
            material=steel_drawer,
            name="tray",
        )
        # Pull handle: small horizontal bar on the front face.
        drawer.visual(
            Box((0.12, 0.005, 0.030)),
            origin=Origin(xyz=(0.0, 0.0025, 0.04)),
            material=steel_handle,
            name="handle_plate",
        )
        drawer.visual(
            Box((0.10, 0.016, 0.018)),
            origin=Origin(xyz=(0.0, 0.005 + 0.008, 0.04)),
            material=steel_handle,
            name="handle_bar",
        )

        model.articulation(
            f"drawer_{i}_slide",
            ArticulationType.PRISMATIC,
            parent=body,
            child=drawer,
            origin=Origin(xyz=(DRAWER_XC, FRONT_Y, DRAWER_ZCS[i])),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=30.0, velocity=0.5, lower=0.0, upper=MAX_SLIDE
            ),
        )
        drawers.append(drawer)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("cabinet_body")
    door = object_model.get_part("door")
    door_hinge = object_model.get_articulation("door_hinge")
    knob = object_model.get_part("latch_knob")
    latch = object_model.get_articulation("latch")
    drawers = [object_model.get_part(f"drawer_{i}") for i in range(N_DRAWERS)]
    slides = [object_model.get_articulation(f"drawer_{i}_slide") for i in range(N_DRAWERS)]

    # --- Intentional overlap: hinge knuckle column laps the left wall edge ---
    ctx.allow_overlap(
        door,
        body,
        elem_a="hinge_barrel",
        elem_b="side_wall_0",
        reason="Barrel-hinge knuckle column intentionally laps the fixed frame edge it pivots on.",
    )

    # --- Overall envelope -----------------------------------------------
    aabb = ctx.part_world_aabb(body)
    ok = aabb is not None
    ctx.check("cabinet body has bounds", ok, details=str(aabb))
    if aabb is not None:
        (x0, y0, z0), (x1, y1, z1) = aabb
        ctx.check(
            "overall width ~1.6 m",
            1.58 <= (x1 - x0) <= 1.70,
            details=f"width={x1 - x0:.3f}",
        )
        ctx.check(
            "overall depth ~0.5 m",
            0.48 <= (y1 - y0) <= 0.58,
            details=f"depth={y1 - y0:.3f}",
        )
        ctx.check(
            "overall height ~1.8 m",
            1.78 <= z1 <= 1.86,
            details=f"top={z1:.3f}",
        )
        ctx.check("legs rest on the floor", abs(z0) <= 1e-6, details=f"zmin={z0:.5f}")

    # --- Door: hinge type, axis, range ----------------------------------
    ctx.check(
        "door hinge is revolute",
        door_hinge.articulation_type == ArticulationType.REVOLUTE,
    )
    ax = door_hinge.axis
    ctx.check(
        "door hinge axis is vertical",
        abs(ax[0]) < 1e-9 and abs(ax[1]) < 1e-9 and abs(abs(ax[2]) - 1.0) < 1e-9,
        details=str(ax),
    )
    lim = door_hinge.motion_limits
    ctx.check(
        "door opens 0..~110 deg",
        lim is not None
        and lim.lower == 0.0
        and abs(lim.upper - math.radians(110.0)) < 1e-6,
    )

    # Door hinge is on the left side of the cabinet.
    ctx.check(
        "door hinge is on the left side",
        door_hinge.origin.xyz[0] < -0.70,
        details=f"hinge_x={door_hinge.origin.xyz[0]:.3f}",
    )

    # Closed leaf sits flush with the front frame.
    daabb = ctx.part_element_world_aabb(door, elem="leaf")
    ctx.check(
        "door closed leaf is flush with the front face",
        daabb is not None
        and abs(daabb[1][1] - FRONT_Y) < 1e-4
        and abs(daabb[0][1] - (FRONT_Y - DOOR_T)) < 1e-4,
        details=str(daabb),
    )

    # Door stays within cabinet width when closed.
    ctx.expect_within(
        door,
        body,
        axes="x",
        margin=0.012,
        name="door stays inside the cabinet width when closed",
    )

    # Opening pose: door swings outward (+Y).
    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: DOOR_OPEN}):
        open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door swings outward past the front face",
        open_aabb is not None and open_aabb[1][1] > FRONT_Y + 0.25,
        details=f"open={open_aabb}",
    )

    # Visible hinge barrels present on the door hinge edge.
    hb_aabb = ctx.part_element_world_aabb(door, elem="hinge_barrel")
    ctx.check(
        "visible hinge barrels on the door side",
        hb_aabb is not None
        and hb_aabb[0][0] < -0.70
        and (hb_aabb[1][2] - hb_aabb[0][2]) > 0.80,
        details=str(hb_aabb),
    )

    # Vent slot sits in the lower half of the door leaf.
    vb = ctx.part_element_world_aabb(door, elem="vent_backing")
    ctx.check(
        "door vent slot sits in the lower half of the leaf",
        vb is not None and vb[1][2] < DOOR_ZC and vb[0][2] > DOOR_Z0,
        details=str(vb),
    )

    # --- Latch knob ------------------------------------------------------
    ctx.check(
        "latch is a quarter-turn revolute about the door normal",
        latch.articulation_type == ArticulationType.REVOLUTE
        and latch.axis == (0.0, 1.0, 0.0)
        and latch.motion_limits is not None
        and abs(latch.motion_limits.upper - math.pi / 2.0) < 1e-6,
    )
    ctx.expect_contact(
        knob,
        door,
        elem_a="backplate",
        elem_b="leaf",
        contact_tol=1e-6,
        name="latch knob backplate seats on the door leaf face",
    )

    # Off-axis handle tip proves the knob really rotates.
    tip_rest = ctx.part_element_world_aabb(knob, elem="handle_tip")
    with ctx.pose({latch: KNOB_TURN}):
        tip_turn = ctx.part_element_world_aabb(knob, elem="handle_tip")
    ctx.check(
        "turning latch sweeps the handle tip sideways and upward",
        tip_rest is not None
        and tip_turn is not None
        and abs(tip_turn[0][0] - tip_rest[0][0]) > 0.012
        and tip_turn[0][2] > tip_rest[0][2] + 0.012,
        details=f"rest={tip_rest}, turned={tip_turn}",
    )

    # --- Drawers: prismatic joints, sliding, geometry --------------------
    for i, (drawer, slide) in enumerate(zip(drawers, slides)):
        ctx.check(
            f"drawer_{i} slide is prismatic",
            slide.articulation_type == ArticulationType.PRISMATIC,
        )
        sax = slide.axis
        ctx.check(
            f"drawer_{i} slides along +Y (forward)",
            abs(sax[0]) < 1e-9 and abs(sax[1] - 1.0) < 1e-9 and abs(sax[2]) < 1e-9,
            details=str(sax),
        )
        slim = slide.motion_limits
        ctx.check(
            f"drawer_{i} slide range 0..~0.38 m",
            slim is not None and slim.lower == 0.0 and slim.upper > 0.30,
            details=str(slim),
        )

        # Drawer is on the right side of the cabinet.
        ctx.check(
            f"drawer_{i} is on the right half",
            slide.origin.xyz[0] > 0.05,
            details=f"x={slide.origin.xyz[0]:.3f}",
        )

        # Closed drawer front is flush with the front frame.
        taabb = ctx.part_element_world_aabb(drawer, elem="tray")
        ctx.check(
            f"drawer_{i} closed tray front is near the cabinet front",
            taabb is not None and abs(taabb[1][1] - FRONT_Y) < 0.005,
            details=str(taabb),
        )

        # Drawer stays within cabinet width when closed.
        ctx.expect_within(
            drawer,
            body,
            axes="x",
            margin=0.015,
            name=f"drawer_{i} stays inside the cabinet width when closed",
        )

        # Slide pose: drawer moves forward.
        closed_pos = ctx.part_world_position(drawer)
        with ctx.pose({slide: MAX_SLIDE}):
            open_pos = ctx.part_world_position(drawer)
        ctx.check(
            f"drawer_{i} slides forward when extended",
            closed_pos is not None
            and open_pos is not None
            and open_pos[1] > closed_pos[1] + 0.30,
            details=f"closed={closed_pos}, open={open_pos}",
        )

    # --- Drawers are vertically stacked ---------------------------------
    for i in range(N_DRAWERS - 1):
        lower_z = slides[i].origin.xyz[2]
        upper_z = slides[i + 1].origin.xyz[2]
        ctx.check(
            f"drawer_{i + 1} is above drawer_{i}",
            upper_z > lower_z + 0.30,
            details=f"lower_z={lower_z:.3f}, upper_z={upper_z:.3f}",
        )

    # Door and drawers occupy different halves of the cabinet.
    door_x = door_hinge.origin.xyz[0]
    drawer_x = slides[0].origin.xyz[0]
    ctx.check(
        "door on left, drawers on right",
        door_x < 0.0 and drawer_x > 0.0,
        details=f"door_hinge_x={door_x:.3f}, drawer_x={drawer_x:.3f}",
    )

    # Riveted top cap detail present along the top rail.
    rivet_aabb = ctx.part_element_world_aabb(body, elem="rivet_0")
    ctx.check(
        "rivet dots stand proud of the top rail face",
        rivet_aabb is not None
        and rivet_aabb[1][1] > FRONT_Y + 0.003
        and rivet_aabb[0][2] > TOP_RAIL_BOT,
        details=str(rivet_aabb),
    )

    return ctx.report()


object_model = build_object_model()
