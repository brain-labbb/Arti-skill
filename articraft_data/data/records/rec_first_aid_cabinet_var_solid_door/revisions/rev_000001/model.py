from __future__ import annotations

# Wall-mounted metal first aid cabinet with a solid sheet-metal hinged door.
#
# Identity (from picture/Science/First aid cabinet/001.png, solid-door variant):
#   A white sheet-metal wall cabinet with a red-cross "FIRST AID" emblem
#   printed on its solid metal door. The single front door is hinged on its
#   LEFT edge and swings open forward/around to expose interior shelves stocked
#   with medical supplies. A carry handle spans the top of the cabinet.
#
# Real mechanism: the door is a REVOLUTE joint about a VERTICAL (Z) hinge line
# on the left front corner of the cabinet body. Positive q swings the door open
# (outward to +Y then around toward -X). Everything else is rigid structure.
#
# Frame convention:
#   +X = right, -X = left (hinge side)
#   +Y = forward (out of the cabinet face, toward viewer); wall is at -Y
#   +Z = up
# The cabinet body is the single root; the door is its only articulated child.

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

DOOR_W = 0.330  # door panel width (X)
DOOR_H = 0.390  # door panel height (Z)
DOOR_T = 0.018  # solid sheet-metal door panel thickness (Y)
DECAL_T = 0.0015  # thin printed decal / raised emblem thickness

HINGE_INSET = 0.006  # how far the hinge line sits in front of the body front face
HINGE_R = 0.0075  # hinge knuckle outer radius
PANEL_X0 = HINGE_R  # door panel left face sits at door-local x=HINGE_R (clears pin)

# Ground rest: geometry is authored centered about z=0; lift the whole
# assembly (every body visual + the door hinge joint origin) by half the body
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


SHELF_EMBED = 0.003  # how far each shelf tucks into the side/back walls
SHELF_Y_BACK = -BODY_D / 2.0 + WALL_T - SHELF_EMBED
SHELF_Y_FRONT = 0.056  # clear of the closed-door panel back (~y=0.060)
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


def build_door_panel() -> cq.Workplane:
    """Solid white sheet-metal door panel (no window cutout)."""
    cx = PANEL_X0 + DOOR_W / 2.0
    panel = (
        cq.Workplane("XY")
        .box(DOOR_W, DOOR_T, DOOR_H)
        .translate((cx, 0.0, 0.0))
        .edges("|Z").fillet(0.003)
    )
    return panel


def build_door_emblem() -> cq.Workplane:
    """Red plus-cross emblem printed/raised on the solid door front face."""
    arm = 0.030  # half-length of a cross arm
    bar = 0.024  # cross bar thickness
    t = DECAL_T
    cx = PANEL_X0 + DOOR_W / 2.0
    vert = cq.Workplane("XY").box(bar, t, 2 * arm)
    horiz = cq.Workplane("XY").box(2 * arm, t, bar)
    cross = vert.union(horiz)
    # Position at the door center, printed onto the front face of the panel.
    # The panel front face is at y=+DOOR_T/2; place the decal so it bonds to
    # the surface (centered at y = DOOR_T/2 + t/2, slightly proud).
    cross = cross.translate((cx, DOOR_T / 2.0, 0.0))
    return cross


def build_door_banner() -> cq.Workplane:
    """Red 'FIRST AID' banner strip printed on the upper door face."""
    cx = PANEL_X0 + DOOR_W / 2.0
    banner_h = 0.028  # height of the text banner strip
    band = (
        cq.Workplane("XY")
        .box(DOOR_W - 0.012, DECAL_T, banner_h)
        .translate(
            (
                cx,
                DOOR_T / 2.0,  # printed onto the door front face
                DOOR_H / 2.0 - banner_h / 2.0 - 0.020,
            )
        )
    )
    return band


def build_door_knob() -> cq.Workplane:
    """Small grip knob on the free (right) edge of the door."""
    knob = (
        cq.Workplane("XY")
        .cylinder(0.022, 0.008)
    )
    knob = knob.rotate((0, 0, 0), (1, 0, 0), 90)
    knob = knob.translate(
        (PANEL_X0 + DOOR_W - 0.020, DOOR_T / 2.0 + 0.007, 0.0)
    )
    return knob


# Interleaving knuckle stations (centers along the pin / local Z).
KNUCKLE_H = 0.055
DOOR_KNUCKLE_Z = [-0.116, 0.0, 0.116]
BODY_KNUCKLE_Z = [-0.058, 0.058]


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

    # Hinge barrel on the BODY side, at the left front corner.
    body.visual(
        mesh_from_cadquery(build_hinge_knuckles(BODY_KNUCKLE_Z), "body_hinge"),
        origin=Origin(
            xyz=(-BODY_W / 2.0 + 0.006, BODY_D / 2.0 + HINGE_INSET, Z_LIFT)
        ),
        material="steel",
        name="body_hinge_barrel",
    )

    # --- Door (articulated child) -----------------------------------------
    door = model.part("cabinet_door")
    door.visual(
        mesh_from_cadquery(build_door_panel(), "door_panel"),
        material="white_metal",
        name="door_panel",
    )
    door.visual(
        mesh_from_cadquery(build_door_emblem(), "door_emblem"),
        material="red",
        name="door_emblem",
    )
    door.visual(
        mesh_from_cadquery(build_door_banner(), "door_banner"),
        material="red",
        name="door_banner",
    )
    door.visual(
        mesh_from_cadquery(build_door_knob(), "door_knob"),
        material="steel",
        name="door_knob",
    )
    # Hinge knuckles on the DOOR side, centered on the pin (door-local x=0).
    door.visual(
        mesh_from_cadquery(build_hinge_knuckles(DOOR_KNUCKLE_Z), "door_hinge"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="steel",
        name="door_hinge_barrel",
    )

    # --- Materials ---------------------------------------------------------
    model.material("white_metal", rgba=WHITE_METAL)
    model.material("red", rgba=RED)
    model.material("white_trim", rgba=WHITE_TRIM)
    model.material("steel", rgba=STEEL)
    model.material("shelf_grey", rgba=SHELF_GREY)
    model.material("supply_blue", rgba=SUPPLY_BLUE)
    model.material("supply_tan", rgba=SUPPLY_TAN)

    # --- Articulation: vertical hinge on the left front edge ---------------
    hinge_x = -BODY_W / 2.0 + 0.006
    hinge_y = BODY_D / 2.0 + HINGE_INSET
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, Z_LIFT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=2.0, lower=0.0, upper=math.radians(150.0)
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests: prove the visual + mechanical claims
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("cabinet_body")
    door = object_model.get_part("cabinet_door")
    hinge = object_model.get_articulation("body_to_door")

    # --- Mechanism: joint type + axis ---
    ctx.check(
        "door joint is revolute",
        hinge.joint_type == ArticulationType.REVOLUTE,
        details=f"joint_type={hinge.joint_type}",
    )
    axis = tuple(round(a, 6) for a in hinge.axis)
    ctx.check(
        "door hinge axis is vertical Z",
        axis == (0.0, 0.0, 1.0),
        details=f"axis={axis}",
    )
    limits = hinge.motion_limits
    ctx.check(
        "door opens (positive upper limit)",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower == 0.0
        and limits.upper > 1.0,
        details=f"limits=({limits.lower if limits else None},"
        f"{limits.upper if limits else None})",
    )

    # --- Hero parts present ---
    body.get_visual("body_shell")
    for elem in (
        "door_panel",
        "door_emblem",
        "door_banner",
        "door_knob",
    ):
        door.get_visual(elem)

    # --- Solid door: no glass visual exists ---
    door_visuals = [v.name for v in door.visuals]
    ctx.check(
        "door has no glass window (solid metal panel)",
        "door_glass" not in door_visuals,
        details=f"door visuals={door_visuals}",
    )

    # --- Closed pose: door spans the cabinet front and seats against it ---
    with ctx.pose({hinge: 0.0}):
        ctx.expect_overlap(
            door, body, axes="xz", min_overlap=0.20,
            name="closed door covers cabinet front",
        )
        door_aabb = ctx.part_world_aabb(door)
        body_aabb = ctx.part_world_aabb(body)
        ctx.check(
            "closed door is in front of body face",
            door_aabb is not None
            and body_aabb is not None
            and door_aabb[1][1] > body_aabb[1][1],
            details=f"door_max_y={door_aabb[1][1] if door_aabb else None}, "
            f"body_max_y={body_aabb[1][1] if body_aabb else None}",
        )
        closed_knob = ctx.part_element_world_aabb(door, elem="door_knob")

    # --- Hinge support: barrels contact at the hinge line (no floating door) ---
    ctx.expect_contact(
        door,
        body,
        elem_a="door_hinge_barrel",
        elem_b="body_hinge_barrel",
        contact_tol=0.004,
        name="door hinge barrel meets body hinge barrel",
    )

    # --- Emblem and banner are printed on the solid door panel face ---
    ctx.expect_within(
        door, door, axes="xz",
        inner_elem="door_emblem", outer_elem="door_panel",
        margin=0.001,
        name="emblem sits within the door panel face",
    )
    ctx.expect_within(
        door, door, axes="xz",
        inner_elem="door_banner", outer_elem="door_panel",
        margin=0.001,
        name="banner sits within the door panel face",
    )
    # Emblem and banner are printed decals bonded onto the metal panel surface.
    ctx.expect_contact(
        door, door,
        elem_a="door_emblem", elem_b="door_panel",
        contact_tol=0.004,
        name="emblem printed on door panel surface",
    )
    ctx.expect_contact(
        door, door,
        elem_a="door_banner", elem_b="door_panel",
        contact_tol=0.004,
        name="banner printed on door panel surface",
    )

    # --- Open pose: actuating the joint swings the free edge away from closed ---
    rest_knob = closed_knob
    with ctx.pose({hinge: math.radians(120.0)}):
        open_knob = ctx.part_element_world_aabb(door, elem="door_knob")
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
            "opening the hinge swings the door's free edge",
            moved > 0.10,
            details=f"free-edge travel={moved:.3f} m (rest={rest_c}, open={open_c})",
        )
        ctx.check(
            "opened door free edge swings off the front (-X of closed)",
            open_c[0] < rest_c[0],
            details=f"rest_x={rest_c[0]:.3f}, open_x={open_c[0]:.3f}",
        )
    else:
        ctx.fail("door knob aabb resolvable", "could not resolve door_knob aabb")

    # --- Intentional seated overlaps (printed decals, hinge barrels, shelves, supplies) ---
    ctx.allow_overlap(
        door, door,
        elem_a="door_emblem", elem_b="door_panel",
        reason="Red cross emblem is a printed decal bonded onto the solid door panel face.",
    )
    ctx.allow_overlap(
        door, door,
        elem_a="door_banner", elem_b="door_panel",
        reason="FIRST AID banner is a printed decal on the solid door panel face.",
    )
    for i in range(2):
        ctx.allow_overlap(
            body, body,
            elem_a=f"shelf_{i}", elem_b="body_shell",
            reason="Shelf is welded into the side and back walls (seated shelf).",
        )
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
