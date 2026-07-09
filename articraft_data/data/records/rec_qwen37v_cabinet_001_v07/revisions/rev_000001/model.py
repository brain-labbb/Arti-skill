from __future__ import annotations

"""Locker-style industrial steel cabinet with upper hinged doors and lower sliding drawers.

Variant of the vintage industrial steel locker cabinet: the upper section carries
two full-height hinged doors with vent slots and visible barrel hinges on the
hinge side; the lower section houses two full-width sliding drawers on prismatic
joints, each with a separate pull handle.

Overall envelope ~1.6 m wide x 0.5 m deep x ~1.8 m tall, brushed/tarnished raw
steel finish. A hollow thin-wall (~0.02 m) carcass sits on four short splayed
legs and carries a thin riveted top cap strip. A horizontal divider shelf at
~1.10 m separates the upper door compartment from the lower drawer compartment.

The two upper doors hinge on their outer edges (left door on the left, right
door on the right) via three visible barrel hinges each, and swing outward
0..~110 deg. Each door carries a vertical vent slot near the bottom and stamped
vent lines near the top. Two full-width drawers slide out on prismatic joints
along +Y with 0..0.35 m travel, each with a bar pull handle on the front face.
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

FRONT_Y = CAB_D / 2.0  # +0.25
BACK_Y = -CAB_D / 2.0  # -0.25

BOTTOM_RAIL_TOP = LEG_H + 0.06  # 0.21
TOP_RAIL_BOT = CAB_TOP - 0.06  # 1.74

# Horizontal divider between door section and drawer section
DIVIDER_Z = 1.10

# ---- Upper door section ----
DOOR_Z0 = BOTTOM_RAIL_TOP + 0.002  # 0.212
DOOR_Z1 = DIVIDER_Z - WALL_T / 2.0 - 0.002  # ~1.09
DOOR_H = DOOR_Z1 - DOOR_Z0
DOOR_ZC = 0.5 * (DOOR_Z0 + DOOR_Z1)

INTERIOR_W = CAB_W - 2.0 * WALL_T  # 1.56
STILE_W = 0.03
DOOR_W = (INTERIOR_W - STILE_W) / 2.0 - 0.004  # two doors + centre stile
DOOR_T = WALL_T

# Hinge x positions (outer edges of door pockets)
HINGE_LEFT_X = -(CAB_W / 2.0 - WALL_T)  # -0.78
HINGE_RIGHT_X = +(CAB_W / 2.0 - WALL_T)  # +0.78

# Vent slot on each door
SLOT_LEN = 0.26
SLOT_W = 0.026
SLOT_ZC = -0.22  # door-local z offset (relative to door part frame at DOOR_ZC)

# Barrel hinge geometry
BARREL_LEN = 0.080
BARREL_PIN_D = 0.006
BARREL_KNUCKLE_OD = 0.016
BARREL_LEAF_W = 0.022
BARREL_LEAF_T = 0.003

# ---- Lower drawer section ----
DRAWER_Z0 = DIVIDER_Z + WALL_T / 2.0 + 0.005  # ~1.115
DRAWER_Z1 = TOP_RAIL_BOT - 0.002  # ~1.738
DRAWER_SECTION_H = DRAWER_Z1 - DRAWER_Z0
DRAWER_GAP = 0.010
DRAWER_H = (DRAWER_SECTION_H - DRAWER_GAP) / 2.0  # ~0.304 each

DRAWER_W = INTERIOR_W - 0.01  # ~1.55
DRAWER_D = CAB_D - WALL_T - 0.06  # ~0.42
DRAWER_FRONT_T = WALL_T
DRAWER_WALL_T = 0.012
DRAWER_BOTTOM_T = 0.010

# Pull handle dimensions
HANDLE_BAR_LEN = 0.18
HANDLE_BAR_D = 0.012
HANDLE_POST_H = 0.025
HANDLE_POST_D = 0.010

# Drawer slide travel
DRAWER_TRAVEL = 0.35

# Runner rails for drawer support
RUNNER_H = 0.015
RUNNER_W = 0.030
RUNNER_D = DRAWER_D - 0.04

# Door open angle
DOOR_OPEN = math.radians(110.0)

CAP_T = 0.022
CAP_OVERHANG = 0.02


def _door_leaf_solid(sign: float, mesh_name: str):
    """Door panel with a rounded-end through slot near the bottom.
    sign=+1: panel extends along +X from hinge (left-hinged).
    sign=-1: panel extends along -X from hinge (right-hinged)."""
    xc = sign * DOOR_W / 2.0
    panel = (
        cq.Workplane("XY")
        .box(DOOR_W, DOOR_T, DOOR_H)
        .translate((xc, -DOOR_T / 2.0, 0.0))
    )
    # Vertical vent slot, rounded ends, cut through thickness
    cutter = (
        cq.Workplane("XZ")
        .slot2D(SLOT_LEN, SLOT_W, 90)
        .extrude(0.05, both=True)
        .translate((xc, 0.0, SLOT_ZC))
    )
    leaf = panel.cut(cutter)
    return mesh_from_cadquery(leaf, mesh_name)


def _barrel_hinge_mesh(mesh_name: str):
    """Visible barrel hinge for the door hinge side."""
    hinge = BarrelHingeGeometry(
        BARREL_LEN,
        leaf_width_a=BARREL_LEAF_W,
        leaf_width_b=BARREL_LEAF_W,
        leaf_thickness=BARREL_LEAF_T,
        pin_diameter=BARREL_PIN_D,
        knuckle_outer_diameter=BARREL_KNUCKLE_OD,
        knuckle_count=5,
        clearance=0.0005,
        open_angle_deg=110.0,
        holes_a=HingeHolePattern(
            style="round", count=2, diameter=0.004, edge_margin=0.012
        ),
        holes_b=HingeHolePattern(
            style="round", count=2, diameter=0.004, edge_margin=0.012
        ),
    )
    return mesh_from_geometry(hinge, mesh_name)


def _drawer_box_solid(mesh_name: str):
    """Open-top drawer tray: front panel + side walls + back + bottom.
    All coords relative to tray center (origin at front-face center)."""
    hw = DRAWER_W / 2.0
    # Front panel (full height, thin)
    front = (
        cq.Workplane("XY")
        .box(DRAWER_W, DRAWER_FRONT_T, DRAWER_H)
        .translate((0.0, -DRAWER_FRONT_T / 2.0, 0.0))
    )
    # Back panel
    back = (
        cq.Workplane("XY")
        .box(DRAWER_W - 2 * DRAWER_WALL_T, DRAWER_WALL_T, DRAWER_H - 0.03)
        .translate((0.0, -DRAWER_D + DRAWER_WALL_T / 2.0, -0.015))
    )
    # Side walls
    side_l = (
        cq.Workplane("XY")
        .box(DRAWER_WALL_T, DRAWER_D - DRAWER_FRONT_T, DRAWER_H - 0.02)
        .translate((-(hw - DRAWER_WALL_T / 2.0),
                     -DRAWER_FRONT_T - (DRAWER_D - DRAWER_FRONT_T) / 2.0,
                     -0.01))
    )
    side_r = (
        cq.Workplane("XY")
        .box(DRAWER_WALL_T, DRAWER_D - DRAWER_FRONT_T, DRAWER_H - 0.02)
        .translate((+(hw - DRAWER_WALL_T / 2.0),
                     -DRAWER_FRONT_T - (DRAWER_D - DRAWER_FRONT_T) / 2.0,
                     -0.01))
    )
    # Bottom panel - extends slightly beyond side walls to contact runner rails
    bottom = (
        cq.Workplane("XY")
        .box(DRAWER_W + 0.01, DRAWER_D - DRAWER_FRONT_T, DRAWER_BOTTOM_T)
        .translate((0.0,
                     -DRAWER_FRONT_T - (DRAWER_D - DRAWER_FRONT_T) / 2.0,
                     -(DRAWER_H / 2.0)))
    )
    tray = front.union(back).union(side_l).union(side_r).union(bottom)
    return mesh_from_cadquery(tray, mesh_name)


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
    model = ArticulatedObject(name="locker_steel_cabinet")

    steel_body = model.material("steel_body", rgba=(0.60, 0.61, 0.63, 1.0))
    steel_door = model.material("steel_door", rgba=(0.54, 0.55, 0.57, 1.0))
    steel_trim = model.material("steel_trim", rgba=(0.46, 0.47, 0.49, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.07, 0.07, 0.08, 1.0))
    steel_leg = model.material("steel_leg", rgba=(0.38, 0.39, 0.41, 1.0))
    steel_rivet = model.material("steel_rivet", rgba=(0.52, 0.53, 0.55, 1.0))
    steel_drawer = model.material("steel_drawer", rgba=(0.56, 0.57, 0.59, 1.0))
    steel_handle = model.material("steel_handle", rgba=(0.30, 0.31, 0.33, 1.0))
    steel_hinge = model.material("steel_hinge", rgba=(0.42, 0.43, 0.45, 1.0))
    steel_runner = model.material("steel_runner", rgba=(0.48, 0.49, 0.51, 1.0))

    # ------------------------------------------------------------------
    # Cabinet body: hollow carcass + legs + frame + divider + top cap
    # ------------------------------------------------------------------
    body = model.part("cabinet_body")
    carcass_h = CAB_TOP - LEG_H  # 1.65
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
    # Bottom and top panels
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
    # Horizontal divider shelf between doors and drawers
    body.visual(
        Box((INTERIOR_W + 0.01, CAB_D - WALL_T - 0.01, WALL_T)),
        origin=Origin(xyz=(0.0, -0.005, DIVIDER_Z)),
        material=steel_body,
        name="divider_shelf",
    )
    # Front frame: bottom rail and top rail
    body.visual(
        Box((INTERIOR_W, WALL_T, BOTTOM_RAIL_TOP - LEG_H + 0.01)),
        origin=Origin(
            xyz=(0.0, FRONT_Y - WALL_T / 2.0, (LEG_H + BOTTOM_RAIL_TOP) / 2.0)
        ),
        material=steel_body,
        name="front_bottom_rail",
    )
    body.visual(
        Box((INTERIOR_W, WALL_T, CAB_TOP - TOP_RAIL_BOT + 0.01)),
        origin=Origin(
            xyz=(0.0, FRONT_Y - WALL_T / 2.0, (TOP_RAIL_BOT + CAB_TOP) / 2.0)
        ),
        material=steel_body,
        name="front_top_rail",
    )
    # Centre stile between the two doors
    stile_h = DOOR_H + 0.01
    body.visual(
        Box((STILE_W, WALL_T, stile_h)),
        origin=Origin(xyz=(0.0, FRONT_Y - WALL_T / 2.0, DOOR_ZC)),
        material=steel_trim,
        name="centre_stile",
    )

    # Drawer runner rails (support the drawer bottoms, contact from below)
    # Each drawer has a pair of runners on the inner side walls
    for i in range(2):
        drawer_bottom_z = DRAWER_Z0 + i * (DRAWER_H + DRAWER_GAP)
        runner_center_z = drawer_bottom_z + RUNNER_H / 2.0  # top of runner above drawer bottom
        for sx, rname in ((-1.0, f"runner_{i}_0"), (1.0, f"runner_{i}_1")):
            body.visual(
                Box((RUNNER_W, RUNNER_D, RUNNER_H)),
                origin=Origin(
                    xyz=(
                        sx * (INTERIOR_W / 2.0 - RUNNER_W / 2.0),
                        -(RUNNER_D / 2.0) + FRONT_Y - DRAWER_FRONT_T,
                        runner_center_z,
                    )
                ),
                material=steel_runner,
                name=rname,
            )

    # Riveted top cap strip
    body.visual(
        Box((CAB_W + 2 * CAP_OVERHANG, CAB_D + 2 * CAP_OVERHANG, CAP_T)),
        origin=Origin(xyz=(0.0, 0.0, CAB_TOP - 0.001 + CAP_T / 2.0)),
        material=steel_trim,
        name="top_cap",
    )
    # Rivet dots along the top rail
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

    # ------------------------------------------------------------------
    # Two upper doors with vent slots and visible barrel hinges
    # ------------------------------------------------------------------
    hinge_mesh = _barrel_hinge_mesh("barrel_hinge")

    door_specs = [
        (HINGE_LEFT_X, +1.0, "door_0"),
        (HINGE_RIGHT_X, -1.0, "door_1"),
    ]
    doors = []
    for i, (hinge_x, sign, dname) in enumerate(door_specs):
        door = model.part(dname)
        xc = sign * DOOR_W / 2.0

        # Door leaf with vent slot cutout
        door.visual(
            _door_leaf_solid(sign, f"door_leaf_{i}"),
            material=steel_door,
            name="leaf",
        )
        # Dark backing behind the vent slot
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

        # Three visible barrel hinges on the hinge edge.
        # Rotate 90° around Z so leaves extend along Y (one leaf against door face).
        # Pin at x=0 (door hinge edge), y at door mid-thickness.
        hinge_z_offsets = [-DOOR_H / 2.0 + 0.10, 0.0, DOOR_H / 2.0 - 0.10]
        for k, dz in enumerate(hinge_z_offsets):
            door.visual(
                hinge_mesh,
                origin=Origin(
                    xyz=(0.0, -DOOR_T / 2.0, dz),
                    rpy=(0.0, 0.0, math.pi / 2.0),
                ),
                material=steel_hinge,
                name=f"hinge_barrel_{k}",
            )

        # Door articulation: revolute about vertical axis at hinge edge
        model.articulation(
            f"{dname}_hinge",
            ArticulationType.REVOLUTE,
            parent=body,
            child=door,
            origin=Origin(xyz=(hinge_x, FRONT_Y, DOOR_ZC)),
            axis=(0.0, 0.0, sign),
            motion_limits=MotionLimits(
                effort=40.0, velocity=2.0, lower=0.0, upper=DOOR_OPEN
            ),
        )
        doors.append(door)

    # ------------------------------------------------------------------
    # Two lower drawers with prismatic joints and pull handles
    # All drawer visuals use part-local coords (z=0 at articulation origin).
    # ------------------------------------------------------------------
    drawer_tray_meshes = [_drawer_box_solid(f"drawer_box_{i}") for i in range(2)]

    drawers = []
    for i in range(2):
        drawer = model.part(f"drawer_{i}")
        # Part frame z = articulation origin z
        # All visual z offsets are 0 (relative to part frame)

        # Drawer box (front panel + tray) - centered on part frame
        drawer.visual(
            drawer_tray_meshes[i],
            material=steel_drawer,
            name="tray",
        )

        # Pull handle: two posts + horizontal bar
        # Posts extend outward from front face (y = 0 in part frame = FRONT_Y world)
        for hi, sx in enumerate((-1.0, 1.0)):
            drawer.visual(
                Cylinder(radius=HANDLE_POST_D / 2.0, length=HANDLE_POST_H),
                origin=Origin(
                    xyz=(sx * HANDLE_BAR_LEN / 3.0, HANDLE_POST_H / 2.0, 0.0),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=steel_handle,
                name=f"handle_post_{hi}",
            )
        # Horizontal bar connecting the post tops
        drawer.visual(
            Cylinder(radius=HANDLE_BAR_D / 2.0, length=HANDLE_BAR_LEN),
            origin=Origin(
                xyz=(0.0, HANDLE_POST_H, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=steel_handle,
            name="handle_bar",
        )

        # Prismatic joint: slides along +Y (outward from front face)
        dz_center = DRAWER_Z0 + DRAWER_H / 2.0 + i * (DRAWER_H + DRAWER_GAP)
        model.articulation(
            f"drawer_{i}_slide",
            ArticulationType.PRISMATIC,
            parent=body,
            child=drawer,
            origin=Origin(xyz=(0.0, FRONT_Y, dz_center)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=60.0, velocity=0.5, lower=0.0, upper=DRAWER_TRAVEL
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

    # --- Allow intentional overlap: hinge barrels lap the frame edges ---
    frame_elems = ["side_wall_0", "side_wall_1"]
    for door, elem in zip(doors, frame_elems):
        for k in range(3):
            ctx.allow_overlap(
                door,
                body,
                elem_a=f"hinge_barrel_{k}",
                elem_b=elem,
                reason="Barrel hinge leaves intentionally lap the fixed frame edge they pivot on.",
            )

    # Allow runner-drawer overlap (runners slightly embed into drawer bottom
    # to represent the sliding support contact)
    for i, drawer in enumerate(drawers):
        for j in range(2):
            ctx.allow_overlap(
                body,
                drawer,
                elem_a=f"runner_{i}_{j}",
                elem_b="tray",
                reason="Drawer bottom panel rests on the support runner rail with slight embed.",
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

    # --- Doors: revolute joints, hinge barrels, vent slots ---
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
        # Visible hinge barrels exist on the door
        hb = ctx.part_element_world_aabb(door, elem="hinge_barrel_0")
        ctx.check(
            f"door_{i} has visible hinge barrel",
            hb is not None,
            details=str(hb),
        )
        # Vent slot backing present in lower half of door
        vb = ctx.part_element_world_aabb(door, elem="vent_backing")
        ctx.check(
            f"door_{i} vent slot in lower half",
            vb is not None and vb[1][2] < DOOR_ZC and vb[0][2] > DOOR_Z0,
            details=str(vb),
        )

    # Door hinge positions: left door on left edge, right door on right edge
    ctx.check(
        "left door hinges on left, right door on right",
        hinges[0].origin.xyz[0] < -0.77 and hinges[1].origin.xyz[0] > 0.77,
        details=str([h.origin.xyz[0] for h in hinges]),
    )

    # Opening pose: doors swing outward
    with ctx.pose({hinges[0]: DOOR_OPEN, hinges[1]: DOOR_OPEN}):
        open0 = ctx.part_world_aabb(doors[0])
        open1 = ctx.part_world_aabb(doors[1])
    ctx.check(
        "doors swing outward past the front face",
        open0 is not None
        and open1 is not None
        and open0[1][1] > FRONT_Y + 0.20
        and open1[1][1] > FRONT_Y + 0.20,
        details=f"open0={open0}, open1={open1}",
    )

    # --- Drawers: prismatic joints, handles, slide motion ---
    for i, (drawer, slide) in enumerate(zip(drawers, slides)):
        ctx.check(
            f"drawer_{i} slide is prismatic",
            slide.articulation_type == ArticulationType.PRISMATIC,
        )
        ax = slide.axis
        ctx.check(
            f"drawer_{i} slides along +Y",
            abs(ax[0]) < 1e-9 and abs(ax[1] - 1.0) < 1e-9 and abs(ax[2]) < 1e-9,
            details=str(ax),
        )
        lim = slide.motion_limits
        ctx.check(
            f"drawer_{i} travel 0..0.35 m",
            lim is not None
            and lim.lower == 0.0
            and abs(lim.upper - DRAWER_TRAVEL) < 1e-6,
        )
        # Pull handle bar exists and is proud of the front face
        hb = ctx.part_element_world_aabb(drawer, elem="handle_bar")
        ctx.check(
            f"drawer_{i} has pull handle bar",
            hb is not None,
            details=str(hb),
        )
        ctx.check(
            f"drawer_{i} handle is proud of the front face",
            hb is not None and hb[0][1] > FRONT_Y,
            details=str(hb),
        )

    # Drawer slide motion: verify drawers actually move outward
    rest0 = ctx.part_world_aabb(drawers[0])
    with ctx.pose({slides[0]: DRAWER_TRAVEL}):
        ext0 = ctx.part_world_aabb(drawers[0])
    ctx.check(
        "drawer_0 slides outward along +Y",
        rest0 is not None
        and ext0 is not None
        and ext0[1][1] > rest0[1][1] + 0.20,
        details=f"rest={rest0}, extended={ext0}",
    )

    rest1 = ctx.part_world_aabb(drawers[1])
    with ctx.pose({slides[1]: DRAWER_TRAVEL}):
        ext1 = ctx.part_world_aabb(drawers[1])
    ctx.check(
        "drawer_1 slides outward along +Y",
        rest1 is not None
        and ext1 is not None
        and ext1[1][1] > rest1[1][1] + 0.20,
        details=f"rest={rest1}, extended={ext1}",
    )

    # Drawers are stacked vertically (drawer_1 above drawer_0)
    ctx.check(
        "drawers are vertically stacked",
        rest0 is not None
        and rest1 is not None
        and rest1[0][2] > rest0[1][2] - 0.02,
        details=f"rest0={rest0}, rest1={rest1}",
    )

    # Drawer front faces sit near cabinet front when closed
    for i, drawer in enumerate(drawers):
        tray_aabb = ctx.part_element_world_aabb(drawer, elem="tray")
        ctx.check(
            f"drawer_{i} front face near cabinet front when closed",
            tray_aabb is not None and abs(tray_aabb[1][1] - FRONT_Y) < 0.015,
            details=str(tray_aabb),
        )

    # Divider shelf separates doors from drawers
    div_aabb = ctx.part_element_world_aabb(body, elem="divider_shelf")
    ctx.check(
        "divider shelf between door and drawer sections",
        div_aabb is not None
        and abs(0.5 * (div_aabb[0][2] + div_aabb[1][2]) - DIVIDER_Z) < 0.02,
        details=str(div_aabb),
    )

    return ctx.report()


object_model = build_object_model()
