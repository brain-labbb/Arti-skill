from __future__ import annotations

"""Tall two-door industrial steel cabinet with raised plinth base and sliding drawers.

Variant of the vintage industrial steel locker cabinet. Approximately 0.80 m wide,
0.50 m deep, 1.80 m tall. Brushed/polished raw steel with a mottled, slightly
tarnished gray finish. The body is a hollow rectangular carcass sitting on a
solid raised plinth base (~0.12 m tall). The upper section has two full-height
hinged doors with visible barrel hinges on the outer edges; the lower section
has two stacked drawers that slide out on prismatic joints. A thin riveted top
cap strip crowns the piece.

Reference image: picture/Other/Cabinet/001.png
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
# Global dimensions (meters). Cabinet centred on X, front face at +Y.
# ---------------------------------------------------------------------------
CAB_W = 0.80       # overall carcass width  (X)
CAB_D = 0.50       # overall carcass depth  (Y)
CAB_TOP = 1.80     # carcass top height     (Z)
PLINTH_H = 0.12    # plinth base height; carcass starts here
WALL_T = 0.02      # thin steel wall thickness

FRONT_Y = CAB_D / 2.0   # +0.25
BACK_Y = -CAB_D / 2.0

# Upper/lower section split
DIVIDER_Z = 0.75   # horizontal divider between drawers and doors (centre)
DIVIDER_BOTTOM = DIVIDER_Z - WALL_T / 2.0  # 0.74
DOOR_Z0 = DIVIDER_Z + WALL_T / 2.0 + 0.004   # ~0.754
DOOR_Z1 = CAB_TOP - 0.06 - 0.002              # top rail leaves 0.06 m
DOOR_H = DOOR_Z1 - DOOR_Z0
DOOR_ZC = 0.5 * (DOOR_Z0 + DOOR_Z1)
DOOR_W = 0.355   # leaf width — leaves a gap to the centre stile
DOOR_T = WALL_T

# Centre stile
STILE_W = 0.028

# Drawer section: from plinth top to divider bottom
DRAWER_OPEN_BOTTOM = PLINTH_H + WALL_T + 0.004   # 0.144
DRAWER_OPEN_TOP = DIVIDER_BOTTOM - 0.004          # 0.736
DRAWER_SECTION_H = DRAWER_OPEN_TOP - DRAWER_OPEN_BOTTOM  # ~0.592
DRAWER_GAP = 0.012
DRAWER_H = (DRAWER_SECTION_H - DRAWER_GAP) / 2.0  # two drawers stacked
DRAWER_W = CAB_W - 2.0 * WALL_T - 0.016
DRAWER_D = CAB_D - 2.0 * WALL_T - 0.06  # box depth (inside the carcass)
DRAWER_FRONT_T = WALL_T
DRAWER_SIDE_T = 0.012

# Drawer centres (Z)
DRAWER_0_ZC = DRAWER_OPEN_BOTTOM + DRAWER_H / 2.0
DRAWER_1_ZC = DRAWER_OPEN_BOTTOM + DRAWER_H + DRAWER_GAP + DRAWER_H / 2.0

# Drawer front panel sits flush with the cabinet front face
DRAWER_FRONT_Y = FRONT_Y - DRAWER_FRONT_T / 2.0  # centre of front panel

# Drawer slide travel
DRAWER_TRAVEL = 0.35

# Top cap
CAP_T = 0.022
CAP_OVERHANG = 0.02

# Hinge
HINGE_LEN = DOOR_H - 0.04
BARREL_HINGE_LEAF_W = 0.022
DOOR_OPEN = math.radians(110.0)

# Vent slot on doors
SLOT_LEN = 0.20
SLOT_W = 0.025
SLOT_ZC = -0.35  # door-local z offset

# Drawer pull dimensions
PULL_W = 0.08
PULL_H = 0.018
PULL_D = 0.022


def _door_leaf(sign: float, mesh_name: str):
    """Door panel: flat steel with rounded-end through slot near bottom.
    sign=+1 extends along +X (left-hinged), sign=-1 along -X (right-hinged)."""
    xc = sign * DOOR_W / 2.0
    panel = (
        cq.Workplane("XY")
        .box(DOOR_W, DOOR_T, DOOR_H)
        .translate((xc, -DOOR_T / 2.0, 0.0))
    )
    # Rounded-end vent slot near the bottom
    cutter = (
        cq.Workplane("XZ")
        .slot2D(SLOT_LEN, SLOT_W, 90)
        .extrude(0.05, both=True)
        .translate((xc, 0.0, SLOT_ZC))
    )
    leaf = panel.cut(cutter)
    return mesh_from_cadquery(leaf, mesh_name)


def _drawer_box(mesh_name: str):
    """Open-top drawer box with integrated front panel, all one solid."""
    # Box front face touches the back of the front panel
    panel_back_y = DRAWER_FRONT_Y - DRAWER_FRONT_T / 2.0
    box_y_center = panel_back_y - DRAWER_D / 2.0
    outer = (
        cq.Workplane("XY")
        .box(DRAWER_W, DRAWER_D, DRAWER_H)
        .translate((0.0, box_y_center, 0.0))
    )
    # Hollow interior (open top)
    inner_w = DRAWER_W - 2.0 * DRAWER_SIDE_T
    inner_d = DRAWER_D - DRAWER_SIDE_T
    inner_h = DRAWER_H - DRAWER_SIDE_T
    cavity = (
        cq.Workplane("XY")
        .box(inner_w, inner_d, inner_h)
        .translate((0.0, box_y_center + DRAWER_SIDE_T / 2.0, DRAWER_SIDE_T / 2.0 + 0.001))
    )
    box = outer.cut(cavity)
    # Front panel: slightly wider/taller face that overlaps the box front for connectivity
    front = (
        cq.Workplane("XY")
        .box(DRAWER_W + 0.006, DRAWER_FRONT_T, DRAWER_H)
        .translate((0.0, DRAWER_FRONT_Y, 0.0))
    )
    solid = box.union(front)
    return mesh_from_cadquery(solid, mesh_name)


def _drawer_front(mesh_name: str):
    """Drawer front face panel, flush with cabinet front (cosmetic overlay)."""
    front = (
        cq.Workplane("XY")
        .box(DRAWER_W + 0.006, DRAWER_FRONT_T, DRAWER_H)
        .translate((0.0, DRAWER_FRONT_Y, 0.0))
    )
    return mesh_from_cadquery(front, mesh_name)


def _plinth_solid(mesh_name: str):
    """Solid plinth base with a slight inset from the carcass footprint."""
    inset = 0.015
    plinth = (
        cq.Workplane("XY")
        .box(CAB_W - 2.0 * inset, CAB_D - 2.0 * inset, PLINTH_H)
        .translate((0.0, 0.0, PLINTH_H / 2.0))
    )
    return mesh_from_cadquery(plinth, mesh_name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tall_two_door_cabinet")

    steel_body = model.material("steel_body", rgba=(0.60, 0.61, 0.63, 1.0))
    steel_door = model.material("steel_door", rgba=(0.55, 0.56, 0.58, 1.0))
    steel_trim = model.material("steel_trim", rgba=(0.46, 0.47, 0.49, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.07, 0.07, 0.08, 1.0))
    steel_plinth = model.material("steel_plinth", rgba=(0.40, 0.41, 0.43, 1.0))
    steel_drawer = model.material("steel_drawer", rgba=(0.52, 0.53, 0.55, 1.0))
    steel_knob = model.material("steel_knob", rgba=(0.18, 0.18, 0.20, 1.0))
    steel_rivet = model.material("steel_rivet", rgba=(0.52, 0.53, 0.55, 1.0))
    steel_hinge = model.material("steel_hinge", rgba=(0.42, 0.43, 0.45, 1.0))

    # ------------------------------------------------------------------
    # Cabinet body: hollow carcass + plinth + frame + top cap + rivets
    # ------------------------------------------------------------------
    body = model.part("cabinet_body")
    carcass_h = CAB_TOP - PLINTH_H  # 1.68
    carcass_zc = PLINTH_H + carcass_h / 2.0

    # Plinth base
    body.visual(
        _plinth_solid("plinth_base"),
        material=steel_plinth,
        name="plinth_base",
    )

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
    # Bottom panel (above plinth)
    body.visual(
        Box((CAB_W, CAB_D, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_H + WALL_T / 2.0)),
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
    # Horizontal divider between drawer section and door section
    body.visual(
        Box((CAB_W - 2.0 * WALL_T + 0.01, CAB_D - 0.04, WALL_T)),
        origin=Origin(xyz=(0.0, -0.01, DIVIDER_Z)),
        material=steel_body,
        name="divider_shelf",
    )
    # Centre stile between doors (upper section)
    body.visual(
        Box((STILE_W, WALL_T, DOOR_H + 0.01)),
        origin=Origin(xyz=(0.0, FRONT_Y - WALL_T / 2.0, DOOR_ZC)),
        material=steel_trim,
        name="centre_stile",
    )
    # Front top rail
    body.visual(
        Box((CAB_W - 2.0 * WALL_T, WALL_T, CAB_TOP - DOOR_Z1 + 0.01)),
        origin=Origin(xyz=(0.0, FRONT_Y - WALL_T / 2.0, (DOOR_Z1 + CAB_TOP) / 2.0)),
        material=steel_body,
        name="front_top_rail",
    )
    # Drawer section front frame: bottom rail only (divider shelf frames the top)
    body.visual(
        Box((CAB_W - 2.0 * WALL_T, WALL_T, 0.03)),
        origin=Origin(xyz=(0.0, FRONT_Y - WALL_T / 2.0, DRAWER_OPEN_BOTTOM - 0.015)),
        material=steel_body,
        name="drawer_bottom_rail",
    )
    # Interior shelf
    body.visual(
        Box((CAB_W - 2.0 * WALL_T + 0.01, CAB_D - 0.08, 0.015)),
        origin=Origin(xyz=(0.0, -0.02, DOOR_ZC + 0.15)),
        material=steel_body,
        name="interior_shelf",
    )
    # Drawer slide rails (fixed to side walls, thin strips that support the drawer
    # box from below — intentional slide-runner contact with the drawer box)
    rail_half_w = 0.008
    rail_x = CAB_W / 2.0 - WALL_T - rail_half_w  # outer face flush with side wall inner face
    for di, dz in enumerate((DRAWER_0_ZC, DRAWER_1_ZC)):
        for sx, si in ((-1.0, 0), (1.0, 1)):
            body.visual(
                Box((2.0 * rail_half_w, DRAWER_D - 0.06, 0.010)),
                origin=Origin(xyz=(sx * rail_x, -0.01, dz - DRAWER_H / 2.0 + 0.005)),
                material=steel_trim,
                name=f"drawer_rail_{di}_{si}",
            )

    # Thin riveted top cap strip
    body.visual(
        Box((CAB_W + 2 * CAP_OVERHANG, CAB_D + 2 * CAP_OVERHANG, CAP_T)),
        origin=Origin(xyz=(0.0, 0.0, CAB_TOP - 0.001 + CAP_T / 2.0)),
        material=steel_trim,
        name="top_cap",
    )
    # Rivet dots along the top rail
    n_riv = 7
    for i in range(n_riv):
        rx = -0.32 + i * (0.64 / (n_riv - 1))
        body.visual(
            Sphere(radius=0.0045),
            origin=Origin(xyz=(rx, FRONT_Y + 0.002, CAB_TOP - 0.04)),
            material=steel_rivet,
            name=f"rivet_{i}",
        )

    # ------------------------------------------------------------------
    # Two doors with visible barrel hinges
    # ------------------------------------------------------------------
    # Door 0: left door, hinges on left edge (hinge_x near -CAB_W/2 + WALL_T)
    # Door 1: right door, hinges on right edge (hinge_x near +CAB_W/2 - WALL_T)
    door_specs = [
        # (hinge_world_x, sign, which side wall for hinge)
        (-CAB_W / 2.0 + WALL_T + 0.001, +1.0),  # left door, hinge on left
        (+CAB_W / 2.0 - WALL_T - 0.001, -1.0),  # right door, hinge on right
    ]
    doors = []
    for i, (hinge_x, sign) in enumerate(door_specs):
        door = model.part(f"door_{i}")
        xc = sign * DOOR_W / 2.0  # door-local panel centre offset

        # Door leaf
        door.visual(
            _door_leaf(sign, f"door_leaf_{i}"),
            material=steel_door,
            name="leaf",
        )
        # Dark backing behind vent slot
        door.visual(
            Box((SLOT_W + 0.012, 0.005, SLOT_LEN + 0.03)),
            origin=Origin(xyz=(xc, -DOOR_T - 0.001, SLOT_ZC)),
            material=steel_dark,
            name="vent_backing",
        )
        # Stamped vent lines near the top
        for j, dz in enumerate((0.40, 0.42, 0.44)):
            door.visual(
                Box((0.14, 0.004, 0.006)),
                origin=Origin(xyz=(xc, -0.0012, dz)),
                material=steel_dark,
                name=f"vent_line_{j}",
            )
        # Visible barrel hinge on the door hinge edge
        hinge_geom = BarrelHingeGeometry(
            HINGE_LEN,
            leaf_width_a=BARREL_HINGE_LEAF_W,
            leaf_width_b=BARREL_HINGE_LEAF_W * 0.85,
            leaf_thickness=0.003,
            pin_diameter=0.005,
            knuckle_outer_diameter=0.014,
            knuckle_count=7,
            clearance=0.0005,
            holes_a=HingeHolePattern(style="round", count=4, diameter=0.004, edge_margin=0.012),
            holes_b=HingeHolePattern(style="round", count=4, diameter=0.004, edge_margin=0.012),
        )
        hinge_mesh = mesh_from_geometry(hinge_geom, f"barrel_hinge_{i}")
        # Place hinge at the door's hinge edge (local x=0 since part origin is at hinge line)
        door.visual(
            hinge_mesh,
            origin=Origin(xyz=(0.0, 0.003, 0.0)),
            material=steel_hinge,
            name="hinge_barrel",
        )
        # Small round latch knob near free edge at mid-height
        door.visual(
            Cylinder(radius=0.012, length=0.012),
            origin=Origin(xyz=(sign * (DOOR_W - 0.05), 0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel_knob,
            name="latch_knob",
        )

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

    # ------------------------------------------------------------------
    # Two sliding drawers with prismatic joints
    # ------------------------------------------------------------------
    drawers = []
    drawer_zcs = [DRAWER_0_ZC, DRAWER_1_ZC]
    for i, dz in enumerate(drawer_zcs):
        drawer = model.part(f"drawer_{i}")

        # Drawer box (open top, with integrated front panel)
        drawer.visual(
            _drawer_box(f"drawer_box_{i}"),
            material=steel_drawer,
            name="box",
        )
        # Drawer pull handle (small bar on the front face)
        pull_y = FRONT_Y + PULL_D / 2.0
        drawer.visual(
            Box((PULL_W, PULL_D, PULL_H)),
            origin=Origin(xyz=(0.0, pull_y, 0.0)),
            material=steel_knob,
            name="pull",
        )
        # Pull mounting posts
        post_y = FRONT_Y + 0.006
        for px in (-PULL_W / 2.0 + 0.01, PULL_W / 2.0 - 0.01):
            drawer.visual(
                Cylinder(radius=0.004, length=0.012),
                origin=Origin(xyz=(px, post_y, 0.0),
                              rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=steel_knob,
                name="pull_post",
            )

        model.articulation(
            f"drawer_{i}_slide",
            ArticulationType.PRISMATIC,
            parent=body,
            child=drawer,
            # Drawer origin at its closed position centre
            origin=Origin(xyz=(0.0, 0.0, dz)),
            axis=(0.0, 1.0, 0.0),  # slides out along +Y (toward front)
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

    # Intentional overlaps: barrel hinge knuckles embed slightly into the side wall
    # at the hinge line (captured hinge pivot).
    for i, door in enumerate(doors):
        wall_elem = f"side_wall_{i}"
        ctx.allow_overlap(
            door,
            body,
            elem_a="hinge_barrel",
            elem_b=wall_elem,
            reason="Barrel hinge knuckles intentionally overlap the side wall at the pivot edge.",
        )

    # Drawer slide rails intentionally overlap the drawer box bottom (slide-runner
    # contact surface that supports the drawer).
    for i, drawer in enumerate(drawers):
        for si in (0, 1):
            ctx.allow_overlap(
                body,
                drawer,
                elem_a=f"drawer_rail_{i}_{si}",
                elem_b="box",
                reason="Drawer slide runner overlaps the box bottom as a sliding support surface.",
            )
        # Proof: drawer box bottom overlaps the rail in Z (small embed proving support)
        ctx.expect_overlap(
            drawer,
            body,
            axes="z",
            elem_a="box",
            elem_b=f"drawer_rail_{i}_0",
            min_overlap=0.002,
            name=f"drawer_{i} box bottom embeds into rail (slide support)",
        )

    # --- Overall envelope ---
    aabb = ctx.part_world_aabb(body)
    ok = aabb is not None
    ctx.check("cabinet body has bounds", ok, details=str(aabb))
    if aabb is not None:
        (x0, y0, z0), (x1, y1, z1) = aabb
        ctx.check(
            "overall width ~0.8 m",
            0.76 <= (x1 - x0) <= 0.90,
            details=f"width={x1 - x0:.3f}",
        )
        ctx.check(
            "overall depth ~0.5 m",
            0.48 <= (y1 - y0) <= 0.58,
            details=f"depth={y1 - y0:.3f}",
        )
        ctx.check(
            "overall height ~1.8 m",
            1.78 <= z1 <= 1.88,
            details=f"top={z1:.3f}",
        )
        ctx.check("plinth rests on the floor", abs(z0) <= 1e-6, details=f"zmin={z0:.5f}")

    # Plinth base exists and is visible
    plinth_aabb = ctx.part_element_world_aabb(body, elem="plinth_base")
    ctx.check(
        "plinth base present at floor level",
        plinth_aabb is not None and plinth_aabb[0][2] >= -1e-6 and plinth_aabb[1][2] <= PLINTH_H + 0.01,
        details=str(plinth_aabb),
    )

    # --- Doors: hinge type, axis, range ---
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
        # Visible hinge barrel on the door
        hinge_aabb = ctx.part_element_world_aabb(door, elem="hinge_barrel")
        ctx.check(
            f"door_{i} has visible hinge barrel",
            hinge_aabb is not None and (hinge_aabb[1][2] - hinge_aabb[0][2]) > 0.5,
            details=str(hinge_aabb),
        )
        # Closed door sits in the upper section
        ctx.expect_within(
            door,
            body,
            axes="x",
            margin=0.02,
            name=f"door_{i} stays inside cabinet width when closed",
        )

    # Left door hinges on left edge, right door on right edge
    ctx.check(
        "left door hinges on left, right door on right",
        hinges[0].origin.xyz[0] < -0.30 and hinges[1].origin.xyz[0] > 0.30,
        details=str([h.origin.xyz[0] for h in hinges]),
    )

    # Opening pose: doors swing outward
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

    # --- Drawers: prismatic type, axis, travel ---
    for i, (drawer, slide) in enumerate(zip(drawers, slides)):
        ctx.check(
            f"drawer_{i} slide is prismatic",
            slide.articulation_type == ArticulationType.PRISMATIC,
        )
        ax = slide.axis
        ctx.check(
            f"drawer_{i} slide axis is along Y (front-out)",
            abs(ax[0]) < 1e-9 and abs(abs(ax[1]) - 1.0) < 1e-9 and abs(ax[2]) < 1e-9,
            details=str(ax),
        )
        lim = slide.motion_limits
        ctx.check(
            f"drawer_{i} slide range 0..~0.35 m",
            lim is not None
            and lim.lower == 0.0
            and abs(lim.upper - DRAWER_TRAVEL) < 1e-6,
        )
        # Drawer sits in the lower section of the cabinet
        ctx.expect_within(
            drawer,
            body,
            axes="x",
            margin=0.015,
            name=f"drawer_{i} stays inside cabinet width at rest",
        )

    # Drawer extended pose: moves forward (+Y)
    for i, (drawer, slide) in enumerate(zip(drawers, slides)):
        rest_pos = ctx.part_world_aabb(drawer)
        with ctx.pose({slide: DRAWER_TRAVEL}):
            ext_pos = ctx.part_world_aabb(drawer)
        ctx.check(
            f"drawer_{i} extends forward when slid open",
            rest_pos is not None
            and ext_pos is not None
            and ext_pos[1][1] > rest_pos[1][1] + 0.20,
            details=f"rest={rest_pos}, extended={ext_pos}",
        )

    # Drawer front panel is visible and distinct
    for i, drawer in enumerate(drawers):
        box_aabb = ctx.part_element_world_aabb(drawer, elem="box")
        ctx.check(
            f"drawer_{i} has a visible front panel",
            box_aabb is not None and (box_aabb[1][2] - box_aabb[0][2]) > 0.1,
            details=str(box_aabb),
        )

    # Drawers are stacked vertically (drawer_1 above drawer_0)
    aabb0 = ctx.part_world_aabb(drawers[0])
    aabb1 = ctx.part_world_aabb(drawers[1])
    ctx.check(
        "drawers are stacked vertically",
        aabb0 is not None
        and aabb1 is not None
        and aabb1[0][2] > aabb0[1][2] - 0.02,
        details=f"d0={aabb0}, d1={aabb1}",
    )

    # Riveted top cap present
    rivet_aabb = ctx.part_element_world_aabb(body, elem="rivet_0")
    ctx.check(
        "rivet dots stand proud along the top rail",
        rivet_aabb is not None
        and rivet_aabb[1][1] > FRONT_Y + 0.003
        and rivet_aabb[0][2] > CAB_TOP - 0.10,
        details=str(rivet_aabb),
    )

    return ctx.report()


object_model = build_object_model()
