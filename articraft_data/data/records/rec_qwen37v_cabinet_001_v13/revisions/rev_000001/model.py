from __future__ import annotations

"""Vintage industrial steel locker cabinet variant 13: hinged door on the left,
three stacked drawers on the right.

Overall envelope ~1.6 m wide x 0.5 m deep x ~1.8 m tall, brushed/tarnished raw
steel. A hollow thin-wall (~0.02 m) carcass sits on four short splayed legs and
carries a thin riveted top cap strip. The front is divided by a vertical centre
stile: the left half has one full-height hinged door with three exposed barrel
hinges, a dark recessed vent slot, stamped vent lines, and a latch knob. The
right half has three stacked drawers on prismatic slides, each with a bar-style
pull handle.
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

FRONT_Y = CAB_D / 2.0  # +0.25
BACK_Y = -CAB_D / 2.0  # -0.25

BOTTOM_RAIL_TOP = LEG_H + 0.06  # 0.21
TOP_RAIL_BOT = CAB_TOP - 0.06  # 1.74

STILE_W = 0.03

# Door area (left half of front)
INNER_X = CAB_W / 2.0 - WALL_T  # 0.78
DOOR_POCKET_W = INNER_X - STILE_W / 2.0  # 0.765
DOOR_W = DOOR_POCKET_W - 0.005  # 0.760 (swing clearance)
DOOR_T = WALL_T
DOOR_Z0 = BOTTOM_RAIL_TOP + 0.002  # 0.212
DOOR_Z1 = TOP_RAIL_BOT - 0.002  # 1.738
DOOR_H = DOOR_Z1 - DOOR_Z0  # 1.526
DOOR_ZC = 0.5 * (DOOR_Z0 + DOOR_Z1)  # 0.975
DOOR_HINGE_X = -INNER_X  # -0.78

# Drawer area (right half of front)
DRAWER_POCKET_W = INNER_X - STILE_W / 2.0  # 0.765
DRAWER_FRONT_W = DRAWER_POCKET_W - 0.010  # 0.755 (slide clearance)
DRAWER_XC = STILE_W / 2.0 + DRAWER_POCKET_W / 2.0  # 0.3975

# Drawer divisions
RAIL_H = 0.025  # horizontal rail between drawers
N_DRAWERS = 3

# Exact rail boundaries for contact:
# Bottom rail top: center=(LEG_H+BOTTOM_RAIL_TOP)/2, height=0.07 → top=0.215
FRONT_BOTTOM_RAIL_TOP = (LEG_H + BOTTOM_RAIL_TOP) / 2.0 + (BOTTOM_RAIL_TOP - LEG_H + 0.01) / 2.0
# Top rail bottom: center=(TOP_RAIL_BOT+CAB_TOP)/2, height=0.07 → bottom=1.735
FRONT_TOP_RAIL_BOT = (TOP_RAIL_BOT + CAB_TOP) / 2.0 - (CAB_TOP - TOP_RAIL_BOT + 0.01) / 2.0

AVAIL_H = FRONT_TOP_RAIL_BOT - FRONT_BOTTOM_RAIL_TOP  # ~1.52
DRAWER_SLOT = (AVAIL_H - (N_DRAWERS - 1) * RAIL_H) / N_DRAWERS  # ~0.49
DRAWER_FRONT_H = DRAWER_SLOT  # no gap, front panels contact rails

DRAWER_BOX_DEPTH = 0.38
DRAWER_BOX_W = DRAWER_FRONT_W - 0.020  # 0.735
DRAWER_BOX_H = DRAWER_FRONT_H - 0.030
DRAWER_WALL_T = 0.012

# Drawer front z-centers (world coords) — stacked from bottom rail top
DRAWER_ZC_LIST = []
for _i in range(N_DRAWERS):
    z_bot = FRONT_BOTTOM_RAIL_TOP + _i * (DRAWER_SLOT + RAIL_H)
    DRAWER_ZC_LIST.append(z_bot + DRAWER_FRONT_H / 2.0)

# Horizontal rail z-centers between drawers
RAIL_ZC_LIST = []
for _i in range(N_DRAWERS - 1):
    rail_z_bot = FRONT_BOTTOM_RAIL_TOP + (_i + 1) * DRAWER_SLOT + _i * RAIL_H
    RAIL_ZC_LIST.append(rail_z_bot + RAIL_H / 2.0)

# Vent slot on door
SLOT_LEN = 0.36
SLOT_W = 0.030
SLOT_ZC = -0.40  # in door-local z

# Hinge barrel dimensions
BARREL_R = 0.010
KNUCKLE_R = 0.014
BARREL_H = 0.055

# Top cap
CAP_T = 0.022
CAP_OVERHANG = 0.02

# Articulation limits
DOOR_OPEN_ANGLE = math.radians(110.0)
DRAWER_EXTEND = 0.35
KNOB_TURN = math.radians(90.0)

# Pull handle dimensions
HANDLE_POST_R = 0.006
HANDLE_POST_LEN = 0.022
HANDLE_BAR_SIZE = (0.12, 0.014, 0.018)
HANDLE_SPACING = 0.050  # half-distance between posts


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------


def _door_leaf(mesh_name: str):
    """Door leaf: flat panel with a rounded-end through slot near the bottom.
    Panel extends along +X from the hinge line (local x=0)."""
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
    return mesh_from_cadquery(leaf, mesh_name)


def _barrel_hinge_solid(mesh_name: str):
    """Single exposed barrel hinge knuckle cluster (local Z axis)."""
    barrel = (
        cq.Workplane("XY")
        .circle(BARREL_R)
        .extrude(BARREL_H / 2.0, both=True)
    )
    for dz in (-0.018, 0.0, 0.018):
        ring = (
            cq.Workplane("XY")
            .circle(KNUCKLE_R)
            .extrude(0.014 / 2.0, both=True)
            .translate((0.0, 0.0, dz))
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


def _drawer_handle_mesh(mesh_name: str):
    """Bar-style pull handle: two box posts joined by a horizontal bar.
    Posts extend along +Y from the front face; bar runs along X at the top.
    Posts extend slightly below Y=0 to embed into the front panel."""
    post_w = HANDLE_POST_R * 2.0
    post_d = HANDLE_POST_R * 2.0
    embed = 0.004  # embed depth into front panel
    post_total_h = HANDLE_POST_LEN + HANDLE_BAR_SIZE[2] + embed  # include embed
    bar_y_center = HANDLE_POST_LEN + embed + HANDLE_BAR_SIZE[1] / 2.0
    
    # Build as one CadQuery solid from boxes
    base = (
        cq.Workplane("XY")
        .box(HANDLE_BAR_SIZE[0], HANDLE_BAR_SIZE[1], HANDLE_BAR_SIZE[2])
        .translate((0.0, bar_y_center, 0.0))
    )
    # Left post (box, overlaps bar in Y, embeds below Y=0)
    # box(width_X, height_Y, depth_Z)
    left_post = (
        cq.Workplane("XY")
        .box(post_w, post_total_h, post_d)
        .translate((-HANDLE_SPACING, post_total_h / 2.0 - embed, 0.0))
    )
    # Right post
    right_post = (
        cq.Workplane("XY")
        .box(post_w, post_total_h, post_d)
        .translate((HANDLE_SPACING, post_total_h / 2.0 - embed, 0.0))
    )
    handle = base.union(left_post).union(right_post)
    return mesh_from_cadquery(handle, mesh_name)


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_steel_locker_cabinet_v13")

    steel_body = model.material("steel_body", rgba=(0.60, 0.61, 0.63, 1.0))
    steel_door = model.material("steel_door", rgba=(0.55, 0.56, 0.58, 1.0))
    steel_drawer = model.material("steel_drawer", rgba=(0.52, 0.53, 0.56, 1.0))
    steel_trim = model.material("steel_trim", rgba=(0.46, 0.47, 0.49, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.07, 0.07, 0.08, 1.0))
    steel_leg = model.material("steel_leg", rgba=(0.38, 0.39, 0.41, 1.0))
    steel_knob = model.material("steel_knob", rgba=(0.18, 0.18, 0.20, 1.0))
    steel_rivet = model.material("steel_rivet", rgba=(0.52, 0.53, 0.55, 1.0))
    steel_box = model.material("steel_box", rgba=(0.48, 0.49, 0.51, 1.0))
    steel_handle = model.material("steel_handle", rgba=(0.25, 0.25, 0.27, 1.0))

    # ==================================================================
    # Cabinet body: hollow carcass + legs + front frame + top cap + rivets
    # ==================================================================
    body = model.part("cabinet_body")
    carcass_h = CAB_TOP - LEG_H  # 1.65
    carcass_zc = LEG_H + carcass_h / 2.0

    # Side walls
    body.visual(
        Box((WALL_T, CAB_D, carcass_h)),
        origin=Origin(xyz=(-(CAB_W / 2.0 - WALL_T / 2.0), 0.0, carcass_zc)),
        material=steel_body,
        name="side_wall_left",
    )
    body.visual(
        Box((WALL_T, CAB_D, carcass_h)),
        origin=Origin(xyz=((CAB_W / 2.0 - WALL_T / 2.0), 0.0, carcass_zc)),
        material=steel_body,
        name="side_wall_right",
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
    # Interior shelf (left half only, under the door)
    shelf_w = DOOR_POCKET_W - 0.01
    shelf_xc = -(STILE_W / 2.0 + DOOR_POCKET_W / 2.0)
    body.visual(
        Box((shelf_w, 0.43, 0.015)),
        origin=Origin(xyz=(shelf_xc, -0.02, 0.95)),
        material=steel_body,
        name="interior_shelf",
    )

    # Front frame: bottom rail, top rail (full width)
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
        origin=Origin(
            xyz=(0.0, FRONT_Y - WALL_T / 2.0, (TOP_RAIL_BOT + CAB_TOP) / 2.0)
        ),
        material=steel_body,
        name="front_top_rail",
    )

    # Centre stile (dividing door from drawers)
    stile_h = TOP_RAIL_BOT - BOTTOM_RAIL_TOP + 0.01
    body.visual(
        Box((STILE_W, WALL_T, stile_h)),
        origin=Origin(xyz=(0.0, FRONT_Y - WALL_T / 2.0, DOOR_ZC)),
        material=steel_trim,
        name="centre_stile",
    )

    # Horizontal rails on the right half (between drawers)
    for i in range(N_DRAWERS - 1):
        body.visual(
            Box((DRAWER_POCKET_W, WALL_T, RAIL_H)),
            origin=Origin(xyz=(DRAWER_XC, FRONT_Y - WALL_T / 2.0, RAIL_ZC_LIST[i])),
            material=steel_body,
            name=f"drawer_rail_{i}",
        )

    # Top cap with overhang
    body.visual(
        Box((CAB_W + 2 * CAP_OVERHANG, CAB_D + 2 * CAP_OVERHANG, CAP_T)),
        origin=Origin(xyz=(0.0, 0.0, CAB_TOP - 0.001 + CAP_T / 2.0)),
        material=steel_trim,
        name="top_cap",
    )

    # Rivets along top rail
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

    # ==================================================================
    # Left side: full-height hinged door with exposed barrel hinges
    # ==================================================================
    door = model.part("door")
    door_xc = DOOR_W / 2.0  # panel centre in local x

    door.visual(
        _door_leaf("door_leaf"),
        material=steel_door,
        name="leaf",
    )
    # Dark backing behind vent slot
    door.visual(
        Box((SLOT_W + 0.016, 0.005, SLOT_LEN + 0.036)),
        origin=Origin(xyz=(door_xc, -DOOR_T - 0.001, SLOT_ZC)),
        material=steel_dark,
        name="vent_backing",
    )
    # Stamped vent lines near the top
    for j, dz in enumerate((0.60, 0.62, 0.64)):
        door.visual(
            Box((0.16, 0.004, 0.006)),
            origin=Origin(xyz=(door_xc, -0.0012, dz)),
            material=steel_dark,
            name=f"vent_line_{j}",
        )

    # Three exposed barrel hinges along the hinge edge (local x=0)
    barrel_mesh = _barrel_hinge_solid("barrel_hinge")
    barrel_zs = (-0.55, 0.0, 0.55)
    for j, bz in enumerate(barrel_zs):
        door.visual(
            barrel_mesh,
            origin=Origin(xyz=(0.0, 0.005, bz)),
            material=steel_trim,
            name=f"hinge_barrel_{j}",
        )

    # Door hinge articulation: revolute, vertical axis at left edge
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(DOOR_HINGE_X, FRONT_Y, DOOR_ZC)),
        axis=(0.0, 0.0, 1.0),  # +q opens free edge outward (+Y)
        motion_limits=MotionLimits(
            effort=40.0, velocity=2.0, lower=0.0, upper=DOOR_OPEN_ANGLE,
        ),
    )

    # Latch knob on the door near the free edge
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
            effort=4.0, velocity=4.0, lower=0.0, upper=KNOB_TURN,
        ),
    )

    # ==================================================================
    # Right side: three stacked drawers on prismatic slides
    # ==================================================================
    handle_mesh = _drawer_handle_mesh("drawer_handle")

    for i in range(N_DRAWERS):
        drawer = model.part(f"drawer_{i}")
        dz_center = DRAWER_ZC_LIST[i]

        # Drawer front panel (outer face at local y=0 → world y=FRONT_Y at rest)
        drawer.visual(
            Box((DRAWER_FRONT_W, DOOR_T, DRAWER_FRONT_H)),
            origin=Origin(xyz=(0.0, -DOOR_T / 2.0, 0.0)),
            material=steel_drawer,
            name="front_panel",
        )

        # Drawer box: sides, bottom, back
        box_y_start = -DOOR_T  # behind front panel inner face
        box_y_center = box_y_start - DRAWER_BOX_DEPTH / 2.0

        # Side walls
        for sx, sname in ((-1.0, "box_side_left"), (1.0, "box_side_right")):
            drawer.visual(
                Box((DRAWER_WALL_T, DRAWER_BOX_DEPTH, DRAWER_BOX_H)),
                origin=Origin(xyz=(
                    sx * (DRAWER_BOX_W / 2.0 + DRAWER_WALL_T / 2.0),
                    box_y_center,
                    0.0,
                )),
                material=steel_box,
                name=sname,
            )
        # Bottom panel
        drawer.visual(
            Box((DRAWER_BOX_W, DRAWER_BOX_DEPTH, DRAWER_WALL_T)),
            origin=Origin(xyz=(
                0.0,
                box_y_center,
                -DRAWER_BOX_H / 2.0 + DRAWER_WALL_T / 2.0,
            )),
            material=steel_box,
            name="box_bottom",
        )
        # Back panel
        drawer.visual(
            Box((DRAWER_BOX_W, DRAWER_WALL_T, DRAWER_BOX_H)),
            origin=Origin(xyz=(
                0.0,
                box_y_start - DRAWER_BOX_DEPTH + DRAWER_WALL_T / 2.0,
                0.0,
            )),
            material=steel_box,
            name="box_back",
        )

        # Pull handle near top of drawer front
        handle_z = DRAWER_FRONT_H / 2.0 - 0.060
        drawer.visual(
            handle_mesh,
            origin=Origin(xyz=(0.0, 0.0, handle_z)),
            material=steel_handle,
            name="handle",
        )

        # Prismatic slide: +Y axis pulls drawer outward
        model.articulation(
            f"drawer_{i}_slide",
            ArticulationType.PRISMATIC,
            parent=body,
            child=drawer,
            origin=Origin(xyz=(DRAWER_XC, FRONT_Y, dz_center)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=30.0, velocity=0.5, lower=0.0, upper=DRAWER_EXTEND,
            ),
        )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("cabinet_body")
    door = object_model.get_part("door")
    door_hinge = object_model.get_articulation("door_hinge")
    knob = object_model.get_part("latch_knob")
    latch = object_model.get_articulation("latch")
    drawers = [object_model.get_part(f"drawer_{i}") for i in range(N_DRAWERS)]
    slides = [object_model.get_articulation(f"drawer_{i}_slide") for i in range(N_DRAWERS)]

    # --- Intentional overlaps: hinge barrels embed into the frame edge ------
    for j in range(3):
        ctx.allow_overlap(
            door, body,
            elem_a=f"hinge_barrel_{j}",
            elem_b="side_wall_left",
            reason="Exposed barrel hinge knuckles intentionally lap the fixed frame edge they pivot on.",
        )

    # --- Overall envelope ---------------------------------------------------
    aabb = ctx.part_world_aabb(body)
    ok = aabb is not None
    ctx.check("cabinet body has bounds", ok, details=str(aabb))
    if aabb is not None:
        (x0, y0, z0), (x1, y1, z1) = aabb
        ctx.check(
            "overall width ~1.6 m",
            1.58 <= (x1 - x0) <= 1.70,
            details=f"w={x1 - x0:.3f}",
        )
        ctx.check(
            "overall depth ~0.5 m",
            0.48 <= (y1 - y0) <= 0.58,
            details=f"d={y1 - y0:.3f}",
        )
        ctx.check(
            "overall height ~1.8 m",
            1.78 <= z1 <= 1.86,
            details=f"top={z1:.3f}",
        )
        ctx.check(
            "legs rest on the floor",
            abs(z0) <= 1e-6,
            details=f"zmin={z0:.5f}",
        )

    # --- Door hinge: type, axis, range, barrel presence, open pose ----------
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

    # Hinge barrels exist near the hinge edge
    for j in range(3):
        hb = ctx.part_element_world_aabb(door, elem=f"hinge_barrel_{j}")
        ctx.check(
            f"hinge_barrel_{j} near left hinge edge",
            hb is not None and hb[0][0] < -0.76 and hb[1][0] < -0.74,
            details=str(hb),
        )

    # Closed door flush with front face
    daabb = ctx.part_element_world_aabb(door, elem="leaf")
    ctx.check(
        "door closed leaf flush with front face",
        daabb is not None and abs(daabb[1][1] - FRONT_Y) < 1e-4,
        details=str(daabb),
    )

    # Open pose: door swings outward
    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: DOOR_OPEN_ANGLE}):
        open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door swings outward past front face",
        open_aabb is not None and open_aabb[1][1] > FRONT_Y + 0.25,
        details=f"open={open_aabb}",
    )
    ctx.check(
        "door free edge swings outward from centre",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][0] < closed_aabb[0][0] - 0.05,
    )

    # --- Drawer slides: type, axis, range, handle, extension ----------------
    for i, (drawer, slide) in enumerate(zip(drawers, slides)):
        ctx.check(
            f"drawer_{i} slide is prismatic",
            slide.articulation_type == ArticulationType.PRISMATIC,
        )
        sax = slide.axis
        ctx.check(
            f"drawer_{i} slide axis is +Y",
            abs(sax[0]) < 1e-9 and abs(sax[1] - 1.0) < 1e-9 and abs(sax[2]) < 1e-9,
            details=str(sax),
        )
        slim = slide.motion_limits
        ctx.check(
            f"drawer_{i} slide range 0..~0.35 m",
            slim is not None
            and slim.lower == 0.0
            and abs(slim.upper - DRAWER_EXTEND) < 1e-6,
        )

        # Pull handle proud of front face
        hb = ctx.part_element_world_aabb(drawer, elem="handle")
        ctx.check(
            f"drawer_{i} pull handle proud of front face",
            hb is not None and hb[1][1] > FRONT_Y + 0.015,
            details=str(hb),
        )

        # Front panel flush at rest
        fp = ctx.part_element_world_aabb(drawer, elem="front_panel")
        ctx.check(
            f"drawer_{i} front panel flush at rest",
            fp is not None and abs(fp[1][1] - FRONT_Y) < 1e-4,
            details=str(fp),
        )

    # Drawer 0 extends outward
    rest_aabb = ctx.part_world_aabb(drawers[0])
    with ctx.pose({slides[0]: DRAWER_EXTEND}):
        ext_aabb = ctx.part_world_aabb(drawers[0])
    ctx.check(
        "drawer_0 extends outward past front face",
        ext_aabb is not None and ext_aabb[1][1] > FRONT_Y + DRAWER_EXTEND - 0.02,
        details=f"extended={ext_aabb}",
    )
    ctx.check(
        "drawer_0 actually moves in +Y",
        rest_aabb is not None
        and ext_aabb is not None
        and ext_aabb[1][1] > rest_aabb[1][1] + 0.10,
    )

    # --- Centre stile divides the front ------------------------------------
    stile_aabb = ctx.part_element_world_aabb(body, elem="centre_stile")
    ctx.check(
        "centre stile at front centre",
        stile_aabb is not None
        and abs(stile_aabb[0][0] + STILE_W / 2.0) < 0.005
        and abs(stile_aabb[1][0] - STILE_W / 2.0) < 0.005,
        details=str(stile_aabb),
    )

    # Horizontal drawer rails exist on the right half
    for i in range(N_DRAWERS - 1):
        rail = ctx.part_element_world_aabb(body, elem=f"drawer_rail_{i}")
        ctx.check(
            f"drawer_rail_{i} on right half",
            rail is not None and rail[0][0] > 0.0,
            details=str(rail),
        )

    # --- Latch knob --------------------------------------------------------
    ctx.check(
        "latch is quarter-turn revolute",
        latch.articulation_type == ArticulationType.REVOLUTE
        and latch.axis == (0.0, 1.0, 0.0)
        and latch.motion_limits is not None
        and abs(latch.motion_limits.upper - math.pi / 2.0) < 1e-6,
    )
    ctx.expect_contact(
        knob, door,
        elem_a="backplate",
        elem_b="leaf",
        contact_tol=1e-6,
        name="latch knob backplate seats on door leaf",
    )

    # Knob turns: handle tip sweeps
    tip_rest = ctx.part_element_world_aabb(knob, elem="handle_tip")
    with ctx.pose({latch: KNOB_TURN}):
        tip_turn = ctx.part_element_world_aabb(knob, elem="handle_tip")
    ctx.check(
        "turning latch sweeps handle tip",
        tip_rest is not None
        and tip_turn is not None
        and abs(tip_turn[0][0] - tip_rest[0][0]) > 0.012
        and tip_turn[0][2] > tip_rest[0][2] + 0.012,
        details=f"rest={tip_rest}, turned={tip_turn}",
    )

    # --- Riveted top cap ---------------------------------------------------
    rivet_aabb = ctx.part_element_world_aabb(body, elem="rivet_0")
    ctx.check(
        "rivets proud of top rail",
        rivet_aabb is not None and rivet_aabb[1][1] > FRONT_Y + 0.003,
        details=str(rivet_aabb),
    )

    return ctx.report()


object_model = build_object_model()
