from __future__ import annotations

"""Locker-style steel cabinet with upper hinged doors and lower sliding drawers.

Variant 27 of the vintage industrial steel locker cabinet family.

Overall envelope ~1.6 m wide x 0.5 m deep x ~1.8 m tall, brushed/tarnished raw
steel. A hollow thin-wall (~0.02 m) carcass sits on four short splayed legs.
The front is divided into an upper section with two full-height hinged doors
(visible barrel hinges on each door side) and a lower section with two stacked
drawers on prismatic slides. Each door has a dark recessed ventilation slot
with rounded ends near the bottom and stamped vent lines near the top. Each
door carries a latch knob at mid-height. Drawers have steel front panels with
integral pull handles.
"""

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BarrelHingeGeometry,
    Box,
    Cylinder,
    HingeHolePattern,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Global dimensions (meters). Cabinet is centred on X, front face at +Y.
# ---------------------------------------------------------------------------
CAB_W = 1.60
CAB_D = 0.50
CAB_TOP = 1.80
LEG_H = 0.15
WALL_T = 0.02

FRONT_Y = CAB_D / 2.0   # +0.25
BACK_Y = -CAB_D / 2.0

BOTTOM_RAIL_TOP = LEG_H + 0.06  # 0.21
TOP_RAIL_BOT = CAB_TOP - 0.06   # 1.74

# Horizontal divider between upper doors and lower drawers
DIVIDER_Z = 0.94
DIVIDER_T = 0.025
DIVIDER_TOP = DIVIDER_Z + DIVIDER_T  # 0.965

# Upper door section
DOOR_SECTION_BOT = DIVIDER_TOP + 0.002  # 0.967
DOOR_SECTION_TOP = TOP_RAIL_BOT - 0.002  # 1.738
DOOR_H = DOOR_SECTION_TOP - DOOR_SECTION_BOT  # ~0.771
DOOR_ZC = 0.5 * (DOOR_SECTION_BOT + DOOR_SECTION_TOP)

# Interior pocket width (between inner side wall faces)
POCKET_W = CAB_W - 2.0 * WALL_T  # 1.56
CENTRE_STILE_W = 0.03
DOOR_W = (POCKET_W - CENTRE_STILE_W) / 2.0  # ~0.765
DOOR_T = WALL_T

# Hinge line positions (outer edges of doors)
HINGE_X_LEFT = -(POCKET_W / 2.0) + 0.001   # left door hinges on left
HINGE_X_RIGHT = (POCKET_W / 2.0) - 0.001    # right door hinges on right

# Lower drawer section
DRAWER_SECTION_BOT = BOTTOM_RAIL_TOP + 0.002  # 0.212
DRAWER_SECTION_TOP = DIVIDER_Z - 0.002        # 0.938
DRAWER_SECTION_H = DRAWER_SECTION_TOP - DRAWER_SECTION_BOT  # ~0.726
DRAWER_DIVIDER_Z = 0.5 * (DRAWER_SECTION_BOT + DRAWER_SECTION_TOP)  # ~0.575
DRAWER_DIVIDER_T = 0.018

# Individual drawer openings
DRAWER_0_BOT = DRAWER_SECTION_BOT
DRAWER_0_TOP = DRAWER_DIVIDER_Z - DRAWER_DIVIDER_T / 2.0  # ~0.566
DRAWER_1_BOT = DRAWER_DIVIDER_Z + DRAWER_DIVIDER_T / 2.0  # ~0.584
DRAWER_1_TOP = DRAWER_SECTION_TOP

DRAWER_OPENING_H_0 = DRAWER_0_TOP - DRAWER_0_BOT  # ~0.354
DRAWER_OPENING_H_1 = DRAWER_1_TOP - DRAWER_1_BOT  # ~0.354

# Drawer box dimensions
DRAWER_BOX_W = POCKET_W - 0.01  # slight clearance
DRAWER_BOX_D = 0.38
DRAWER_FRONT_T = WALL_T
DRAWER_FRONT_H_0 = DRAWER_OPENING_H_0 - 0.004
DRAWER_FRONT_H_1 = DRAWER_OPENING_H_1 - 0.004
DRAWER_SIDE_T = 0.012
DRAWER_BOTTOM_T = 0.010

DRAWER_TRAVEL = 0.35  # prismatic slide travel

# Vent slot on doors
SLOT_LEN = 0.30
SLOT_W = 0.028
SLOT_ZC = -0.22  # in door-local z (relative to door centre)

CAP_T = 0.022
CAP_OVERHANG = 0.02

DOOR_OPEN = math.radians(110.0)
KNOB_TURN = math.radians(90.0)

# Barrel hinge parameters
BH_LENGTH = 0.080
BH_LEAF_W_A = 0.025  # frame leaf width
BH_LEAF_W_B = 0.022  # door leaf width
BH_LEAF_T = 0.003
BH_PIN_D = 0.005
BH_KNUCKLE_OD = 0.014


def _door_leaf(sign: float, mesh_name: str):
    """Door panel with rounded-end vent slot near the bottom.
    sign=+1: panel extends +X from hinge (left-hinged).
    sign=-1: panel extends -X from hinge (right-hinged).
    """
    xc = sign * DOOR_W / 2.0
    panel = (
        cq.Workplane("XY")
        .box(DOOR_W, DOOR_T, DOOR_H)
        .translate((xc, -DOOR_T / 2.0, 0.0))
    )
    # Vent slot: rounded-end vertical slot cut through the leaf
    cutter = (
        cq.Workplane("XZ")
        .slot2D(SLOT_LEN, SLOT_W, 90)
        .extrude(0.05, both=True)
        .translate((xc, 0.0, SLOT_ZC))
    )
    leaf = panel.cut(cutter)
    return mesh_from_cadquery(leaf, mesh_name)


def _barrel_hinge_mesh(name: str):
    """Visible barrel hinge using SDK BarrelHingeGeometry."""
    geom = BarrelHingeGeometry(
        BH_LENGTH,
        leaf_width_a=BH_LEAF_W_A,
        leaf_width_b=BH_LEAF_W_B,
        leaf_thickness=BH_LEAF_T,
        pin_diameter=BH_PIN_D,
        knuckle_outer_diameter=BH_KNUCKLE_OD,
        knuckle_count=5,
        clearance=0.0005,
        open_angle_deg=120.0,
        holes_a=HingeHolePattern(style="round", count=3, diameter=0.0035, edge_margin=0.010),
        holes_b=HingeHolePattern(style="round", count=3, diameter=0.0035, edge_margin=0.010),
    )
    return mesh_from_geometry(geom, name)


def _drawer_box(idx: int, front_h: float, mesh_name: str):
    """Drawer box: open-top tray with front panel, sides, back, and bottom.
    Part origin at the centre of the box. Front panel at local +Y."""
    box_w = DRAWER_BOX_W
    box_d = DRAWER_BOX_D
    box_h = front_h - 0.01  # slightly shorter than front panel

    # Front panel
    front = (
        cq.Workplane("XY")
        .box(box_w, DRAWER_FRONT_T, front_h)
        .translate((0.0, box_d / 2.0 - DRAWER_FRONT_T / 2.0, 0.0))
    )
    # Back panel
    back = (
        cq.Workplane("XY")
        .box(box_w, DRAWER_SIDE_T, box_h)
        .translate((0.0, -box_d / 2.0 + DRAWER_SIDE_T / 2.0, -0.005))
    )
    # Left side
    left = (
        cq.Workplane("XY")
        .box(DRAWER_SIDE_T, box_d - DRAWER_FRONT_T, box_h)
        .translate((-box_w / 2.0 + DRAWER_SIDE_T / 2.0, -DRAWER_FRONT_T / 2.0, -0.005))
    )
    # Right side
    right = (
        cq.Workplane("XY")
        .box(DRAWER_SIDE_T, box_d - DRAWER_FRONT_T, box_h)
        .translate((box_w / 2.0 - DRAWER_SIDE_T / 2.0, -DRAWER_FRONT_T / 2.0, -0.005))
    )
    # Bottom
    bottom = (
        cq.Workplane("XY")
        .box(box_w - 2 * DRAWER_SIDE_T, box_d - DRAWER_FRONT_T, DRAWER_BOTTOM_T)
        .translate((0.0, -DRAWER_FRONT_T / 2.0, -box_h / 2.0 + DRAWER_BOTTOM_T / 2.0 - 0.005))
    )
    box = front.union(back).union(left).union(right).union(bottom)
    return mesh_from_cadquery(box, mesh_name)


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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="locker_cabinet_doors_drawers")

    steel_body = model.material("steel_body", rgba=(0.58, 0.59, 0.61, 1.0))
    steel_door = model.material("steel_door", rgba=(0.54, 0.55, 0.57, 1.0))
    steel_drawer = model.material("steel_drawer", rgba=(0.50, 0.51, 0.53, 1.0))
    steel_trim = model.material("steel_trim", rgba=(0.44, 0.45, 0.47, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.07, 0.07, 0.08, 1.0))
    steel_leg = model.material("steel_leg", rgba=(0.36, 0.37, 0.39, 1.0))
    steel_knob = model.material("steel_knob", rgba=(0.16, 0.16, 0.18, 1.0))
    steel_rivet = model.material("steel_rivet", rgba=(0.50, 0.51, 0.53, 1.0))
    steel_hinge = model.material("steel_hinge", rgba=(0.42, 0.43, 0.45, 1.0))
    steel_handle = model.material("steel_handle", rgba=(0.22, 0.22, 0.24, 1.0))

    # ==================================================================
    # Cabinet body: hollow carcass + legs + frame + top cap + rivets
    # ==================================================================
    body = model.part("cabinet_body")
    carcass_h = CAB_TOP - LEG_H
    carcass_zc = LEG_H + carcass_h / 2.0

    # Side walls
    for sx, vname in ((-1.0, "side_wall_0"), (1.0, "side_wall_1")):
        body.visual(
            Box((WALL_T, CAB_D, carcass_h)),
            origin=Origin(xyz=(sx * (CAB_W / 2.0 - WALL_T / 2.0), 0.0, carcass_zc)),
            material=steel_body,
            name=vname,
        )
    # Back wall
    body.visual(
        Box((CAB_W - WALL_T, WALL_T, carcass_h - 0.02)),
        origin=Origin(xyz=(0.0, BACK_Y + WALL_T / 2.0, carcass_zc)),
        material=steel_body,
        name="back_wall",
    )
    # Bottom panel
    body.visual(
        Box((CAB_W, CAB_D, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, LEG_H + WALL_T / 2.0)),
        material=steel_body,
        name="bottom_panel",
    )
    # Top panel
    body.visual(
        Box((CAB_W, CAB_D, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, CAB_TOP - WALL_T / 2.0)),
        material=steel_body,
        name="top_panel",
    )
    # Horizontal divider between doors and drawers
    body.visual(
        Box((CAB_W - 2 * WALL_T + 0.01, CAB_D - WALL_T, DIVIDER_T)),
        origin=Origin(xyz=(0.0, -WALL_T / 2.0, DIVIDER_Z + DIVIDER_T / 2.0)),
        material=steel_body,
        name="horizontal_divider",
    )
    # Drawer internal divider
    body.visual(
        Box((CAB_W - 2 * WALL_T + 0.01, CAB_D - WALL_T - 0.02, DRAWER_DIVIDER_T)),
        origin=Origin(xyz=(0.0, -WALL_T / 2.0 - 0.01, DRAWER_DIVIDER_Z)),
        material=steel_body,
        name="drawer_divider",
    )

    # Front frame: bottom rail, top rail, centre stile (door section only)
    body.visual(
        Box((CAB_W - 2 * WALL_T, WALL_T, BOTTOM_RAIL_TOP - LEG_H + 0.01)),
        origin=Origin(
            xyz=(0.0, FRONT_Y - WALL_T / 2.0, (LEG_H + BOTTOM_RAIL_TOP) / 2.0)
        ),
        material=steel_body,
        name="front_bottom_rail",
    )
    body.visual(
        Box((CAB_W - 2 * WALL_T, WALL_T, CAB_TOP - TOP_RAIL_BOT + 0.01)),
        origin=Origin(xyz=(0.0, FRONT_Y - WALL_T / 2.0, (TOP_RAIL_BOT + CAB_TOP) / 2.0)),
        material=steel_body,
        name="front_top_rail",
    )
    # Centre stile between the two doors (upper section only)
    body.visual(
        Box((CENTRE_STILE_W, WALL_T, DOOR_H + 0.01)),
        origin=Origin(xyz=(0.0, FRONT_Y - WALL_T / 2.0, DOOR_ZC)),
        material=steel_trim,
        name="centre_stile",
    )
    # Horizontal front rail at divider level
    body.visual(
        Box((CAB_W - 2 * WALL_T, WALL_T, DIVIDER_T + 0.01)),
        origin=Origin(xyz=(0.0, FRONT_Y - WALL_T / 2.0, DIVIDER_Z + DIVIDER_T / 2.0)),
        material=steel_trim,
        name="divider_rail",
    )

    # Riveted top cap
    body.visual(
        Box((CAB_W + 2 * CAP_OVERHANG, CAB_D + 2 * CAP_OVERHANG, CAP_T)),
        origin=Origin(xyz=(0.0, 0.0, CAB_TOP - 0.001 + CAP_T / 2.0)),
        material=steel_trim,
        name="top_cap",
    )
    # Rivet dots along top rail
    n_riv = 13
    for i in range(n_riv):
        rx = -0.72 + i * (1.44 / (n_riv - 1))
        body.visual(
            Sphere(radius=0.0045),
            origin=Origin(xyz=(rx, FRONT_Y + 0.002, 1.77)),
            material=steel_rivet,
            name=f"rivet_{i}",
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
            material=steel_leg,
            name=f"leg_{i}",
        )

    # Slide rails for drawers (simple channel rails on inner side walls)
    for dz_label, dz_c in (("0", 0.5 * (DRAWER_0_BOT + DRAWER_0_TOP)),
                            ("1", 0.5 * (DRAWER_1_BOT + DRAWER_1_TOP))):
        for sx in (-1.0, 1.0):
            body.visual(
                Box((0.008, CAB_D - 0.06, 0.015)),
                origin=Origin(xyz=(
                    sx * (POCKET_W / 2.0 - 0.004),
                    -0.01,
                    dz_c - 0.04,
                )),
                material=steel_trim,
                name=f"slide_rail_{dz_label}_{'L' if sx < 0 else 'R'}",
            )

    # ==================================================================
    # Two upper doors with visible barrel hinges
    # ==================================================================
    hinge_mesh = _barrel_hinge_mesh("barrel_hinge")

    door_specs = [
        # (hinge_x, sign, name)
        (HINGE_X_LEFT, +1.0, "door_0"),   # left door, hinge on left
        (HINGE_X_RIGHT, -1.0, "door_1"),  # right door, hinge on right
    ]
    doors = []
    for i, (hinge_x, sign, dname) in enumerate(door_specs):
        door = model.part(dname)
        xc = sign * DOOR_W / 2.0

        # Door leaf with vent slot
        door.visual(
            _door_leaf(sign, f"door_leaf_{i}"),
            material=steel_door,
            name="leaf",
        )
        # Dark backing behind vent slot
        door.visual(
            Box((SLOT_W + 0.014, 0.005, SLOT_LEN + 0.030)),
            origin=Origin(xyz=(xc, -DOOR_T - 0.001, SLOT_ZC)),
            material=steel_dark,
            name="vent_backing",
        )
        # Stamped vent lines near the top
        for j, dz in enumerate((0.30, 0.32, 0.34)):
            door.visual(
                Box((0.18, 0.004, 0.006)),
                origin=Origin(xyz=(xc, -0.0012, dz)),
                material=steel_dark,
                name=f"vent_line_{j}",
            )

        # Visible barrel hinges: 3 per door (top, middle, bottom)
        hinge_z_offsets = [
            DOOR_H / 2.0 - 0.08,   # near top
            0.0,                     # middle
            -DOOR_H / 2.0 + 0.08,  # near bottom
        ]
        for hi, hz in enumerate(hinge_z_offsets):
            door.visual(
                hinge_mesh,
                origin=Origin(xyz=(0.0, 0.0, hz)),
                material=steel_hinge,
                name=f"hinge_{hi}",
            )

        # Door articulation: revolute about vertical Z axis
        model.articulation(
            f"door_{i}_hinge",
            ArticulationType.REVOLUTE,
            parent=body,
            child=door,
            origin=Origin(xyz=(hinge_x, FRONT_Y, DOOR_ZC)),
            axis=(0.0, 0.0, sign),  # +Z for left-hinged, -Z for right-hinged
            motion_limits=MotionLimits(
                effort=40.0, velocity=2.0, lower=0.0, upper=DOOR_OPEN
            ),
        )
        doors.append(door)

        # Latch knob at mid-height near the free edge
        knob = model.part(f"latch_knob_{i}")
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
            f"latch_{i}",
            ArticulationType.REVOLUTE,
            parent=door,
            child=knob,
            origin=Origin(xyz=(sign * (DOOR_W - 0.10), 0.0, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0, velocity=4.0, lower=0.0, upper=KNOB_TURN
            ),
        )

    # ==================================================================
    # Two lower drawers on prismatic slides
    # ==================================================================
    drawer_specs = [
        (0, DRAWER_FRONT_H_0, 0.5 * (DRAWER_0_BOT + DRAWER_0_TOP)),
        (1, DRAWER_FRONT_H_1, 0.5 * (DRAWER_1_BOT + DRAWER_1_TOP)),
    ]
    drawers = []
    for idx, front_h, zc in drawer_specs:
        drawer = model.part(f"drawer_{idx}")

        # Drawer box (open-top tray)
        drawer.visual(
            _drawer_box(idx, front_h, f"drawer_box_{idx}"),
            material=steel_drawer,
            name="box",
        )

        # Pull handle: horizontal bar across the front panel
        handle_y = DRAWER_BOX_D / 2.0 + 0.005
        drawer.visual(
            Box((0.14, 0.012, 0.018)),
            origin=Origin(xyz=(0.0, handle_y, 0.04)),
            material=steel_handle,
            name="handle_bar",
        )
        # Handle standoffs
        for hx in (-0.055, 0.055):
            drawer.visual(
                Cylinder(radius=0.006, length=0.012),
                origin=Origin(xyz=(hx, handle_y - 0.006, 0.04), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=steel_handle,
                name=f"handle_standoff_{'L' if hx < 0 else 'R'}",
            )

        # Drawer front at rest sits at FRONT_Y. Part origin = centre of box.
        # Box centre Y at rest: the front panel is at +DRAWER_BOX_D/2 from centre,
        # so centre_y = FRONT_Y - DRAWER_BOX_D/2
        drawer_center_y = FRONT_Y - DRAWER_BOX_D / 2.0

        model.articulation(
            f"drawer_{idx}_slide",
            ArticulationType.PRISMATIC,
            parent=body,
            child=drawer,
            origin=Origin(xyz=(0.0, drawer_center_y, zc)),
            axis=(0.0, 1.0, 0.0),  # slides out toward +Y
            motion_limits=MotionLimits(
                effort=30.0, velocity=0.5, lower=0.0, upper=DRAWER_TRAVEL
            ),
        )
        drawers.append(drawer)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("cabinet_body")
    doors = [object_model.get_part(f"door_{i}") for i in range(2)]
    hinges = [object_model.get_articulation(f"door_{i}_hinge") for i in range(2)]
    drawers = [object_model.get_part(f"drawer_{i}") for i in range(2)]
    slides = [object_model.get_articulation(f"drawer_{i}_slide") for i in range(2)]
    knobs = [object_model.get_part(f"latch_knob_{i}") for i in range(2)]
    latches = [object_model.get_articulation(f"latch_{i}") for i in range(2)]

    # --- Barrel hinge overlap allowances ---
    # The barrel hinge leaves intentionally lap the frame edge and door edge.
    for door, frame_elem in zip(doors, ["side_wall_0", "side_wall_1"]):
        for hi in range(3):
            ctx.allow_overlap(
                door,
                body,
                elem_a=f"hinge_{hi}",
                elem_b=frame_elem,
                reason="Barrel hinge frame leaf intentionally laps the fixed frame edge it pivots on.",
            )

    # --- Overall envelope ---
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

    # --- Door hinge checks: revolute, vertical axis, 0..110° ---
    for i, (door, hinge) in enumerate(zip(doors, hinges)):
        ctx.check(
            f"door_{i} hinge is revolute",
            hinge.articulation_type == ArticulationType.REVOLUTE,
        )
        ax = hinge.axis
        ctx.check(
            f"door_{i} hinge axis is vertical",
            abs(ax[0]) < 1e-9 and abs(ax[1]) < 1e-9 and abs(abs(ax[2]) - 1.0) < 1e-9,
            details=str(ax),
        )
        lim = hinge.motion_limits
        ctx.check(
            f"door_{i} opens 0..~110 deg",
            lim is not None
            and lim.lower == 0.0
            and abs(lim.upper - math.radians(110.0)) < 1e-6,
        )
        # Closed leaf flush with front face
        daabb = ctx.part_element_world_aabb(door, elem="leaf")
        ctx.check(
            f"door_{i} closed leaf is flush with the front face",
            daabb is not None
            and abs(daabb[1][1] - FRONT_Y) < 1e-4,
            details=str(daabb),
        )
        # Vent slot in lower half
        vb = ctx.part_element_world_aabb(door, elem="vent_backing")
        ctx.check(
            f"door_{i} vent slot sits in the lower half of the leaf",
            vb is not None and vb[1][2] < DOOR_ZC and vb[0][2] > DOOR_SECTION_BOT,
            details=str(vb),
        )

    # Left door hinges on left, right door on right
    ctx.check(
        "door_0 hinges left, door_1 hinges right",
        hinges[0].origin.xyz[0] < -0.70 and hinges[1].origin.xyz[0] > 0.70,
        details=str([h.origin.xyz[0] for h in hinges]),
    )

    # Opening pose: doors swing outward
    closed0 = ctx.part_world_aabb(doors[0])
    closed1 = ctx.part_world_aabb(doors[1])
    with ctx.pose({hinges[0]: DOOR_OPEN, hinges[1]: DOOR_OPEN}):
        open0 = ctx.part_world_aabb(doors[0])
        open1 = ctx.part_world_aabb(doors[1])
    ctx.check(
        "open doors swing outward past the front face",
        open0 is not None
        and open1 is not None
        and open0[1][1] > FRONT_Y + 0.20
        and open1[1][1] > FRONT_Y + 0.20,
        details=f"open0={open0}, open1={open1}",
    )
    ctx.check(
        "doors open away from centre",
        closed0 is not None
        and closed1 is not None
        and open0[0][0] < closed0[0][0] - 0.05
        and open1[1][0] > closed1[1][0] + 0.05,
        details=f"closed0={closed0}, open0={open0}",
    )

    # Visible barrel hinges present on each door
    for i, door in enumerate(doors):
        for hi in range(3):
            h_aabb = ctx.part_element_world_aabb(door, elem=f"hinge_{hi}")
            ctx.check(
                f"door_{i} barrel hinge_{hi} is visible",
                h_aabb is not None,
                details=str(h_aabb),
            )

    # --- Drawer slide checks: prismatic, +Y axis, travel ---
    for i, (drawer, slide) in enumerate(zip(drawers, slides)):
        ctx.check(
            f"drawer_{i} slide is prismatic",
            slide.articulation_type == ArticulationType.PRISMATIC,
        )
        ax = slide.axis
        ctx.check(
            f"drawer_{i} slide axis is +Y (pulls out front)",
            abs(ax[0]) < 1e-9 and abs(ax[1] - 1.0) < 1e-9 and abs(ax[2]) < 1e-9,
            details=str(ax),
        )
        lim = slide.motion_limits
        ctx.check(
            f"drawer_{i} slide travel 0..0.35 m",
            lim is not None
            and lim.lower == 0.0
            and abs(lim.upper - DRAWER_TRAVEL) < 1e-6,
        )

        # At rest: drawer front near the cabinet front face
        rest_aabb = ctx.part_element_world_aabb(drawer, elem="box")
        ctx.check(
            f"drawer_{i} at rest is inside the cabinet",
            rest_aabb is not None and rest_aabb[1][1] <= FRONT_Y + 0.005,
            details=str(rest_aabb),
        )

        # Extended pose: drawer moves outward
        rest_center_y = 0.5 * (rest_aabb[0][1] + rest_aabb[1][1]) if rest_aabb else 0.0
        with ctx.pose({slide: DRAWER_TRAVEL}):
            ext_aabb = ctx.part_element_world_aabb(drawer, elem="box")
        ext_center_y = 0.5 * (ext_aabb[0][1] + ext_aabb[1][1]) if ext_aabb else 0.0
        ctx.check(
            f"drawer_{i} extends outward when slide opens",
            ext_aabb is not None
            and ext_center_y > rest_center_y + DRAWER_TRAVEL - 0.01,
            details=f"rest_y={rest_center_y:.3f}, ext_y={ext_center_y:.3f}",
        )

        # Drawer stays within cabinet width
        ctx.expect_within(
            drawer,
            body,
            axes="x",
            margin=0.015,
            name=f"drawer_{i} stays inside the cabinet width",
        )

    # Drawer vertical positions: lower drawer below upper drawer
    d0_aabb = ctx.part_world_aabb(drawers[0])
    d1_aabb = ctx.part_world_aabb(drawers[1])
    ctx.check(
        "drawer_0 is below drawer_1",
        d0_aabb is not None
        and d1_aabb is not None
        and d0_aabb[1][2] < d1_aabb[0][2] + 0.01,
        details=f"d0={d0_aabb}, d1={d1_aabb}",
    )

    # --- Latch knobs ---
    for i, (knob, latch, door) in enumerate(zip(knobs, latches, doors)):
        ctx.check(
            f"latch_{i} is a quarter-turn revolute about the door normal",
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
            name=f"latch_knob_{i} backplate seats on the leaf face",
        )

    # Off-axis handle tip proves knob rotation
    tip_rest = ctx.part_element_world_aabb(knobs[0], elem="handle_tip")
    with ctx.pose({latches[0]: KNOB_TURN}):
        tip_turn = ctx.part_element_world_aabb(knobs[0], elem="handle_tip")
    ctx.check(
        "turning latch_0 sweeps the handle tip sideways and upward",
        tip_rest is not None
        and tip_turn is not None
        and abs(tip_turn[0][0] - tip_rest[0][0]) > 0.012
        and tip_turn[0][2] > tip_rest[0][2] + 0.012,
        details=f"rest={tip_rest}, turned={tip_turn}",
    )

    # Riveted top cap
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
