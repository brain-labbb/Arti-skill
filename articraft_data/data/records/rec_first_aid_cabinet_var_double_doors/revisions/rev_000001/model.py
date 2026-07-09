from __future__ import annotations

# Wall-mounted metal first aid cabinet with two hinged glass-window doors.
#
# Identity (from picture/Science/First aid cabinet/001.png):
#   A white sheet-metal wall cabinet with red-cross "FIRST AID" emblems on its
#   doors. Two narrower front doors meet at the cabinet centerline: the left
#   door is hinged on the left edge and the right door is hinged on the right
#   edge. Both swing open outward to expose interior shelves stocked with
#   medical supplies. A carry handle spans the top of the cabinet.
#
# Real mechanism: each door is a REVOLUTE joint about a VERTICAL (Z) hinge line
# at its respective front corner of the cabinet body.
#   - Left door:  axis=(0,0,+1), positive q swings the free edge forward (+Y).
#   - Right door: axis=(0,0,-1), positive q swings the free edge forward (+Y).
#
# Frame convention:
#   +X = right, -X = left
#   +Y = forward (out of the cabinet face, toward viewer); wall is at -Y
#   +Z = up
# The cabinet body is the single root; both doors are its articulated children.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Real-world dimensions (meters)
# ---------------------------------------------------------------------------
BODY_W = 0.340  # overall cabinet width (X)
BODY_D = 0.130  # cabinet depth front-to-back (Y)
BODY_H = 0.400  # cabinet height (Z)
WALL_T = 0.010  # sheet-metal wall thickness
SHELF_T = 0.008  # interior shelf thickness

DOOR_H = 0.390  # door panel height (Z)
DOOR_T = 0.022  # door panel thickness (Y)
DOOR_FRAME = 0.040  # white frame border width around the glass window
GLASS_T = 0.004  # glazing thickness

HINGE_INSET = 0.006  # how far the hinge line sits in front of the body front face
HINGE_R = 0.0075  # hinge knuckle outer radius
PANEL_X0 = HINGE_R  # door panel inner face sits at door-local x=HINGE_R (clears pin)

# Each leaf spans from its hinge pin to the cabinet centerline.
# hinge offset from body center = BODY_W/2 - 0.006
# panel starts at PANEL_X0 from hinge pin
DOOR_LEAF_W = BODY_W / 2.0 - 0.006 - PANEL_X0  # ~0.1565 m per leaf

# Ground rest: geometry is authored centered about z=0; lift the whole
# assembly (every body visual + the door hinge joint origins) by half the body
# height so the cabinet bottom sits exactly on the ground plane (z = 0).
Z_LIFT = BODY_H / 2.0

# Body interior cavity (front-open shell)
INNER_W = BODY_W - 2 * WALL_T
INNER_H = BODY_H - 2 * WALL_T
INNER_D = BODY_D - WALL_T  # back wall only; front is open

# Materials --------------------------------------------------------------------
WHITE_METAL = (0.92, 0.92, 0.93, 1.0)  # painted white steel
RED = (0.78, 0.10, 0.12, 1.0)  # first-aid red
WHITE_TRIM = (0.97, 0.97, 0.97, 1.0)
GLASS = (0.62, 0.74, 0.80, 0.45)  # translucent pale glazing
STEEL = (0.70, 0.72, 0.75, 1.0)  # bright metal handle / hinges
SHELF_GREY = (0.85, 0.86, 0.87, 1.0)
SUPPLY_BLUE = (0.20, 0.40, 0.72, 1.0)
SUPPLY_TAN = (0.86, 0.80, 0.66, 1.0)


# ---------------------------------------------------------------------------
# CadQuery geometry builders (authored directly in meters)
# ---------------------------------------------------------------------------
def build_body_shell() -> cq.Workplane:
    """Hollow white cabinet box, open on the +Y (front) face, closed at back."""
    outer = cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H)
    cavity = (
        cq.Workplane("XY")
        .box(INNER_W, BODY_D, INNER_H)
        .translate((0.0, WALL_T / 2.0 + 0.001, 0.0))
    )
    shell = outer.cut(cavity)
    shell = shell.edges("|Z").fillet(0.004)
    return shell


SHELF_EMBED = 0.003
SHELF_Y_BACK = -BODY_D / 2.0 + WALL_T - SHELF_EMBED
SHELF_Y_FRONT = 0.056
SHELF_DEPTH = SHELF_Y_FRONT - SHELF_Y_BACK
SHELF_Y_CENTER = (SHELF_Y_FRONT + SHELF_Y_BACK) / 2.0


def build_shelf() -> cq.Workplane:
    """One interior shelf that tucks into the side and back walls."""
    return cq.Workplane("XY").box(INNER_W + 2 * SHELF_EMBED, SHELF_DEPTH, SHELF_T)


def build_supply_block(w: float, d: float, h: float) -> cq.Workplane:
    """A simple stacked medical-supply box on a shelf."""
    return cq.Workplane("XY").box(w, d, h).edges("|Z").fillet(0.002)


def build_handle() -> cq.Workplane:
    """Bright-metal carry handle: a flat strap arched across the cabinet top."""
    grip_len = 0.150
    riser_h = 0.022
    bar_t = 0.006
    bar_w = 0.014
    grip = (
        cq.Workplane("XY")
        .box(grip_len, bar_w, bar_t)
        .translate((0.0, 0.0, riser_h))
    )
    riser_l = (
        cq.Workplane("XY")
        .box(bar_t, bar_w, riser_h)
        .translate((-grip_len / 2.0 + bar_t / 2.0, 0.0, riser_h / 2.0))
    )
    riser_r = (
        cq.Workplane("XY")
        .box(bar_t, bar_w, riser_h)
        .translate((grip_len / 2.0 - bar_t / 2.0, 0.0, riser_h / 2.0))
    )
    handle = grip.union(riser_l).union(riser_r)
    handle = handle.edges("|Y").fillet(0.0025)
    return handle


# ---------------------------------------------------------------------------
# Shared door-leaf geometry helper (parameterized by side)
# ---------------------------------------------------------------------------
# side = +1 → left door (hinge at local x=0, panel extends toward +X)
# side = -1 → right door (hinge at local x=0, panel extends toward -X)

def build_door_frame(side: int) -> cq.Workplane:
    """White metal door leaf with a rectangular window opening."""
    cx = side * (PANEL_X0 + DOOR_LEAF_W / 2.0)
    panel = (
        cq.Workplane("XY")
        .box(DOOR_LEAF_W, DOOR_T, DOOR_H)
        .translate((cx, 0.0, 0.0))
        .edges("|Z").fillet(0.003)
    )
    win_w = DOOR_LEAF_W - 2 * DOOR_FRAME
    win_h = DOOR_H - 2 * DOOR_FRAME
    window_cut = (
        cq.Workplane("XY")
        .box(win_w, DOOR_T + 0.01, win_h)
        .translate((cx, 0.0, 0.0))
    )
    return panel.cut(window_cut)


def build_door_glass(side: int) -> cq.Workplane:
    """Translucent glazing seated in the door window opening."""
    cx = side * (PANEL_X0 + DOOR_LEAF_W / 2.0)
    win_w = DOOR_LEAF_W - 2 * DOOR_FRAME
    win_h = DOOR_H - 2 * DOOR_FRAME
    return (
        cq.Workplane("XY")
        .box(win_w + 0.006, GLASS_T, win_h + 0.006)
        .translate((cx, 0.0, 0.0))
    )


def build_door_emblem(side: int) -> cq.Workplane:
    """Red plus-cross emblem decal raised on the glass center."""
    arm = 0.025  # half-length (slightly smaller for narrower leaf)
    bar = 0.020
    t = 0.0015
    cx = side * (PANEL_X0 + DOOR_LEAF_W / 2.0)
    vert = cq.Workplane("XY").box(bar, t, 2 * arm)
    horiz = cq.Workplane("XY").box(2 * arm, t, bar)
    cross = vert.union(horiz)
    cross = cross.translate((cx, GLASS_T / 2.0, 0.0))
    return cross


def build_door_banner(side: int) -> cq.Workplane:
    """Red 'FIRST AID' banner strip across the top of the door frame."""
    cx = side * (PANEL_X0 + DOOR_LEAF_W / 2.0)
    return (
        cq.Workplane("XY")
        .box(DOOR_LEAF_W - 0.012, 0.0020, DOOR_FRAME - 0.010)
        .translate(
            (
                cx,
                DOOR_T / 2.0,
                DOOR_H / 2.0 - (DOOR_FRAME - 0.010) / 2.0 - 0.005,
            )
        )
    )


def build_door_knob(side: int) -> cq.Workplane:
    """Small grip knob on the free (centerline) edge of the door leaf."""
    # Free edge is at side * (PANEL_X0 + DOOR_LEAF_W) from hinge;
    # knob sits inset by half the frame width from the free edge.
    knob_x = side * (PANEL_X0 + DOOR_LEAF_W - DOOR_FRAME / 2.0)
    knob = cq.Workplane("XY").cylinder(0.022, 0.008)
    knob = knob.rotate((0, 0, 0), (1, 0, 0), 90)
    knob = knob.translate((knob_x, DOOR_T / 2.0 + 0.007, 0.0))
    return knob


# Interleaving knuckle stations (centers along the pin / local Z).
KNUCKLE_H = 0.055
DOOR_KNUCKLE_Z = [-0.116, 0.0, 0.116]  # door leaf carries 3 knuckles
BODY_KNUCKLE_Z = [-0.058, 0.058]  # body leaf carries 2, nested between door's


def build_hinge_knuckles(z_stations: list[float]) -> cq.Workplane:
    """Vertical hinge knuckles (barrels) about a shared local Z pin axis."""
    barrels = None
    for z in z_stations:
        seg = cq.Workplane("XY").cylinder(KNUCKLE_H, HINGE_R).translate((0.0, 0.0, z))
        barrels = seg if barrels is None else barrels.union(seg)
    return barrels


# ---------------------------------------------------------------------------
# Model assembly
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="first_aid_cabinet")

    # --- Body (root) -------------------------------------------------------
    body = model.part("cabinet_body")
    body.visual(
        mesh_from_cadquery(build_body_shell(), "body_shell"),
        origin=Origin(xyz=(0.0, 0.0, Z_LIFT)),
        material="white_metal",
        name="body_shell",
    )

    # Interior shelves divide the cavity into thirds.
    shelf_mesh = mesh_from_cadquery(build_shelf(), "shelf")
    shelf_z = [-INNER_H / 6.0, INNER_H / 6.0]
    for i, z in enumerate(shelf_z):
        body.visual(
            shelf_mesh,
            origin=Origin(xyz=(0.0, SHELF_Y_CENTER, z + Z_LIFT)),
            material="shelf_grey",
            name=f"shelf_{i}",
        )

    # Carry handle mounted on the top face of the cabinet.
    body.visual(
        mesh_from_cadquery(build_handle(), "handle"),
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0 + Z_LIFT)),
        material="steel",
        name="carry_handle",
    )

    # Supplies sitting on the shelves and cavity floor (read as "stocked").
    supply_mesh_a = mesh_from_cadquery(
        build_supply_block(0.090, 0.060, 0.060), "supply_a"
    )
    supply_mesh_b = mesh_from_cadquery(
        build_supply_block(0.070, 0.055, 0.070), "supply_b"
    )
    EMBED = 0.002
    supply_y = WALL_T / 2.0 + 0.012
    floor_top = -INNER_H / 2.0
    mid_shelf_top = shelf_z[0] + SHELF_T / 2.0
    body.visual(
        supply_mesh_a,
        origin=Origin(xyz=(-0.060, supply_y, floor_top + 0.030 - EMBED + Z_LIFT)),
        material="supply_blue",
        name="supply_floor_l",
    )
    body.visual(
        supply_mesh_b,
        origin=Origin(xyz=(0.060, supply_y, floor_top + 0.035 - EMBED + Z_LIFT)),
        material="supply_tan",
        name="supply_floor_r",
    )
    body.visual(
        supply_mesh_b,
        origin=Origin(xyz=(-0.050, supply_y, mid_shelf_top + 0.035 - EMBED + Z_LIFT)),
        material="supply_tan",
        name="supply_mid_l",
    )
    body.visual(
        supply_mesh_a,
        origin=Origin(xyz=(0.055, supply_y, mid_shelf_top + 0.030 - EMBED + Z_LIFT)),
        material="supply_blue",
        name="supply_mid_r",
    )

    # Hinge barrels on the BODY side — one set at each front corner.
    hinge_body_mesh = mesh_from_cadquery(
        build_hinge_knuckles(BODY_KNUCKLE_Z), "body_hinge"
    )
    hinge_y = BODY_D / 2.0 + HINGE_INSET
    for i, sx in enumerate([-1.0, 1.0]):
        hinge_x = sx * (BODY_W / 2.0 - 0.006)
        body.visual(
            hinge_body_mesh,
            origin=Origin(xyz=(hinge_x, hinge_y, Z_LIFT)),
            material="steel",
            name=f"body_hinge_barrel_{i}",
        )

    # --- Doors (two articulated children) ----------------------------------
    # side=+1 → left door (index 0), side=-1 → right door (index 1)
    door_sides = [1, -1]
    door_parts = []
    for i, side in enumerate(door_sides):
        leaf = model.part(f"door_{i}")
        leaf.visual(
            mesh_from_cadquery(build_door_frame(side), f"door_frame_{i}"),
            material="white_metal",
            name=f"frame_{i}",
        )
        leaf.visual(
            mesh_from_cadquery(build_door_glass(side), f"door_glass_{i}"),
            material="glass",
            name=f"glass_{i}",
        )
        leaf.visual(
            mesh_from_cadquery(build_door_emblem(side), f"door_emblem_{i}"),
            material="red",
            name=f"emblem_{i}",
        )
        leaf.visual(
            mesh_from_cadquery(build_door_banner(side), f"door_banner_{i}"),
            material="red",
            name=f"banner_{i}",
        )
        leaf.visual(
            mesh_from_cadquery(build_door_knob(side), f"door_knob_{i}"),
            material="steel",
            name=f"knob_{i}",
        )
        # Hinge knuckles on the door side, centered on the pin (local x=0).
        leaf.visual(
            mesh_from_cadquery(build_door_hinge_knuckles(), f"door_hinge_{i}"),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material="steel",
            name=f"hinge_barrel_{i}",
        )
        door_parts.append(leaf)

    # --- Materials ---------------------------------------------------------
    model.material("white_metal", rgba=WHITE_METAL)
    model.material("red", rgba=RED)
    model.material("white_trim", rgba=WHITE_TRIM)
    model.material("glass", rgba=GLASS)
    model.material("steel", rgba=STEEL)
    model.material("shelf_grey", rgba=SHELF_GREY)
    model.material("supply_blue", rgba=SUPPLY_BLUE)
    model.material("supply_tan", rgba=SUPPLY_TAN)

    # --- Articulations: vertical hinge at each front corner ----------------
    # Left door (i=0): hinge on the left front corner.
    #   axis=(0,0,+1), positive q swings free (+X) edge forward (+Y) → opens.
    # Right door (i=1): hinge on the right front corner.
    #   axis=(0,0,-1), positive q swings free (-X) edge forward (+Y) → opens.
    hinge_configs = [
        # (side_x_sign, axis_z, door_index)
        (-1.0, 1.0, 0),   # left door
        (1.0, -1.0, 1),   # right door
    ]
    for sx, az, di in hinge_configs:
        hinge_x = sx * (BODY_W / 2.0 - 0.006)
        model.articulation(
            f"body_to_door_{di}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=door_parts[di],
            origin=Origin(xyz=(hinge_x, hinge_y, Z_LIFT)),
            axis=(0.0, 0.0, az),
            motion_limits=MotionLimits(
                effort=8.0, velocity=2.0, lower=0.0, upper=math.radians(150.0)
            ),
        )

    return model


def build_door_hinge_knuckles() -> cq.Workplane:
    """Alias to build hinge knuckles for a door leaf."""
    return build_hinge_knuckles(DOOR_KNUCKLE_Z)


# ---------------------------------------------------------------------------
# Tests: prove the visual + mechanical claims
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("cabinet_body")
    door_0 = object_model.get_part("door_0")
    door_1 = object_model.get_part("door_1")
    hinge_0 = object_model.get_articulation("body_to_door_0")
    hinge_1 = object_model.get_articulation("body_to_door_1")

    # --- Mechanism: both joints are revolute about vertical Z ---
    for hinge, label, expected_az in [
        (hinge_0, "left door", 1.0),
        (hinge_1, "right door", -1.0),
    ]:
        ctx.check(
            f"{label} joint is revolute",
            hinge.joint_type == ArticulationType.REVOLUTE,
            details=f"joint_type={hinge.joint_type}",
        )
        axis = tuple(round(a, 6) for a in hinge.axis)
        ctx.check(
            f"{label} hinge axis is vertical (az={expected_az:+.0f})",
            axis == (0.0, 0.0, expected_az),
            details=f"axis={axis}",
        )
        limits = hinge.motion_limits
        ctx.check(
            f"{label} opens (positive upper limit)",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower == 0.0
            and limits.upper > 1.0,
            details=f"limits=({limits.lower if limits else None},"
            f"{limits.upper if limits else None})",
        )

    # --- Hero parts present on each door ---
    for i in range(2):
        door = object_model.get_part(f"door_{i}")
        for elem in (f"frame_{i}", f"glass_{i}", f"emblem_{i}", f"banner_{i}", f"knob_{i}"):
            door.get_visual(elem)

    # --- Closed pose: both doors span the cabinet front ---
    with ctx.pose({hinge_0: 0.0, hinge_1: 0.0}):
        for door in (door_0, door_1):
            ctx.expect_overlap(
                door, body, axes="xz", min_overlap=0.10,
                name=f"closed {door.name} covers its half of cabinet front",
            )
        # Doors sit in front of (+Y of) the body front face.
        body_aabb = ctx.part_world_aabb(body)
        for door in (door_0, door_1):
            door_aabb = ctx.part_world_aabb(door)
            ctx.check(
                f"closed {door.name} is in front of body face",
                door_aabb is not None
                and body_aabb is not None
                and door_aabb[1][1] > body_aabb[1][1],
                details=f"door_max_y={door_aabb[1][1] if door_aabb else None}, "
                f"body_max_y={body_aabb[1][1] if body_aabb else None}",
            )
        closed_knob_0 = ctx.part_element_world_aabb(door_0, elem="knob_0")
        closed_knob_1 = ctx.part_element_world_aabb(door_1, elem="knob_1")

    # --- Hinge support: barrels contact at each hinge line ---
    for i in range(2):
        door = object_model.get_part(f"door_{i}")
        ctx.expect_contact(
            door,
            body,
            elem_a=f"hinge_barrel_{i}",
            elem_b=f"body_hinge_barrel_{i}",
            contact_tol=0.004,
            name=f"door_{i} hinge barrel meets body hinge barrel",
        )

    # --- Glass + emblem are mounted on each door and centered in the window ---
    for i in range(2):
        door = object_model.get_part(f"door_{i}")
        ctx.expect_within(
            door, door, axes="xz",
            inner_elem=f"emblem_{i}", outer_elem=f"glass_{i}",
            margin=0.001,
            name=f"door_{i} emblem sits within the glass window",
        )
        ctx.expect_contact(
            door, door,
            elem_a=f"glass_{i}", elem_b=f"frame_{i}",
            contact_tol=0.004,
            name=f"door_{i} glazing seated in the frame",
        )

    # --- Two doors meet near the centerline when closed ---
    with ctx.pose({hinge_0: 0.0, hinge_1: 0.0}):
        ctx.expect_overlap(
            door_0, door_1, axes="z", min_overlap=0.30,
            name="both doors share the full cabinet height when closed",
        )

    # --- Open pose: each door swings its free edge outward ---
    open_angle = math.radians(120.0)
    with ctx.pose({hinge_0: open_angle, hinge_1: open_angle}):
        open_knob_0 = ctx.part_element_world_aabb(door_0, elem="knob_0")
        open_knob_1 = ctx.part_element_world_aabb(door_1, elem="knob_1")

    for label, rest_knob, open_knob, expect_x_direction in [
        ("left door", closed_knob_0, open_knob_0, -1),   # free edge swings left (-X)
        ("right door", closed_knob_1, open_knob_1, 1),   # free edge swings right (+X)
    ]:
        if rest_knob is not None and open_knob is not None:
            rest_c = (
                (rest_knob[0][0] + rest_knob[1][0]) / 2.0,
                (rest_knob[0][1] + rest_knob[1][1]) / 2.0,
            )
            open_c = (
                (open_knob[0][0] + open_knob[1][0]) / 2.0,
                (open_knob[0][1] + open_knob[1][1]) / 2.0,
            )
            moved = math.hypot(open_c[0] - rest_c[0], open_c[1] - rest_c[1])
            ctx.check(
                f"opening swings the {label} free edge",
                moved > 0.08,
                details=f"travel={moved:.3f} m (rest={rest_c}, open={open_c})",
            )
            # Left door free edge moves toward -X; right door toward +X.
            if expect_x_direction < 0:
                ctx.check(
                    f"opened {label} free edge swings outward (-X)",
                    open_c[0] < rest_c[0],
                    details=f"rest_x={rest_c[0]:.3f}, open_x={open_c[0]:.3f}",
                )
            else:
                ctx.check(
                    f"opened {label} free edge swings outward (+X)",
                    open_c[0] > rest_c[0],
                    details=f"rest_x={rest_c[0]:.3f}, open_x={open_c[0]:.3f}",
                )
        else:
            ctx.fail(f"{label} knob aabb resolvable", "could not resolve knob aabb")

    # --- Intentional seated overlaps ---
    for i in range(2):
        door = object_model.get_part(f"door_{i}")
        ctx.allow_overlap(
            door, door,
            elem_a=f"glass_{i}", elem_b=f"frame_{i}",
            reason=f"Glazing is captured in door_{i} frame's window rebate (seated fit).",
        )
        ctx.allow_overlap(
            door, door,
            elem_a=f"emblem_{i}", elem_b=f"glass_{i}",
            reason=f"Red cross emblem is a raised decal bonded onto door_{i} glass.",
        )
        ctx.allow_overlap(
            door, door,
            elem_a=f"banner_{i}", elem_b=f"frame_{i}",
            reason=f"FIRST AID banner is a printed decal on door_{i} upper frame.",
        )

    # Interior shelves tuck into the cabinet walls (seated structure).
    for shelf_elem in ("shelf_0", "shelf_1"):
        ctx.allow_overlap(
            body, body,
            elem_a=shelf_elem, elem_b="body_shell",
            reason="Shelf is welded into the side and back walls (seated shelf).",
        )
    # Supplies rest with a small embed into the shelf/floor they sit on.
    ctx.allow_overlap(
        body, body,
        elem_a="supply_floor_l", elem_b="body_shell",
        reason="Supply box rests seated on the cavity floor (small embed).",
    )
    ctx.allow_overlap(
        body, body,
        elem_a="supply_floor_r", elem_b="body_shell",
        reason="Supply box rests seated on the cavity floor (small embed).",
    )
    ctx.allow_overlap(
        body, body,
        elem_a="supply_mid_l", elem_b="shelf_0",
        reason="Supply box rests seated on the middle shelf (small embed).",
    )
    ctx.allow_overlap(
        body, body,
        elem_a="supply_mid_r", elem_b="shelf_0",
        reason="Supply box rests seated on the middle shelf (small embed).",
    )

    return ctx.report()


object_model = build_object_model()
