from __future__ import annotations

# Wall-mounted metal first aid cabinet with a hinged glass-window door.
#
# Identity (from picture/Science/First aid cabinet/001.png):
#   A white sheet-metal wall cabinet with a red-cross "FIRST AID" emblem on its
#   door. The single front door is hinged on its LEFT edge and swings open
#   forward/around to expose interior shelves stocked with medical supplies.
#   A carry handle spans the top of the cabinet.
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
DOOR_T = 0.022  # door panel thickness (Y)
DOOR_FRAME = 0.040  # white frame border width around the glass window
GLASS_T = 0.004  # glazing thickness

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
    # Solid outer box centered at body origin.
    outer = cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H)
    # Hollow out the interior, leaving the back wall and side/top/bottom walls.
    # Cavity spans from just behind the back wall to fully through the front.
    cavity = (
        cq.Workplane("XY")
        .box(INNER_W, BODY_D, INNER_H)
        # shift toward +Y so it opens the front and keeps a back wall at -Y
        .translate((0.0, WALL_T / 2.0 + 0.001, 0.0))
    )
    shell = outer.cut(cavity)
    # Soften the visible front and outer vertical edges slightly.
    shell = shell.edges("|Z").fillet(0.004)
    return shell


SHELF_EMBED = 0.003  # how far each shelf tucks into the side/back walls
# Shelf Y extent: from a few mm into the back wall to just behind the door
# panel's seated back face (so the shelf never collides with the closed door).
SHELF_Y_BACK = -BODY_D / 2.0 + WALL_T - SHELF_EMBED  # inside the back wall
SHELF_Y_FRONT = 0.056  # clear of the closed-door panel back (~y=0.060)
SHELF_DEPTH = SHELF_Y_FRONT - SHELF_Y_BACK
SHELF_Y_CENTER = (SHELF_Y_FRONT + SHELF_Y_BACK) / 2.0


def build_shelf() -> cq.Workplane:
    """One interior shelf that tucks into the side and back walls."""
    # Oversized in X so the shelf embeds into both side walls; Y span is chosen
    # to reach into the back wall but stay clear of the closed door panel.
    return cq.Workplane("XY").box(INNER_W + 2 * SHELF_EMBED, SHELF_DEPTH, SHELF_T)


def build_supply_block(w: float, d: float, h: float) -> cq.Workplane:
    """A simple stacked medical-supply box on a shelf."""
    return cq.Workplane("XY").box(w, d, h).edges("|Z").fillet(0.002)


def build_handle() -> cq.Workplane:
    """Bright-metal carry handle: a flat strap arched across the cabinet top."""
    # Two vertical risers and a horizontal grip bar forming a squared arch.
    grip_len = 0.150
    riser_h = 0.022
    bar_t = 0.006
    bar_w = 0.014  # depth (Y) of the strap
    # Horizontal grip bar (runs along X), top of the arch.
    grip = (
        cq.Workplane("XY")
        .box(grip_len, bar_w, bar_t)
        .translate((0.0, 0.0, riser_h))
    )
    # Left and right risers (run along Z) connecting grip down to the top face.
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


def build_door_frame() -> cq.Workplane:
    """White metal door panel with a rectangular window opening (a frame)."""
    # Door authored in its own local frame: the hinge line is at local x=0,
    # the panel extends along +X, centered in Z. Front face toward +Y.
    cx = PANEL_X0 + DOOR_W / 2.0
    panel = (
        cq.Workplane("XY")
        .box(DOOR_W, DOOR_T, DOOR_H)
        .translate((cx, 0.0, 0.0))
        .edges("|Z").fillet(0.003)
    )
    # Window opening: cut a rectangular hole, leaving a frame border.
    win_w = DOOR_W - 2 * DOOR_FRAME
    win_h = DOOR_H - 2 * DOOR_FRAME
    window_cut = (
        cq.Workplane("XY")
        .box(win_w, DOOR_T + 0.01, win_h)
        .translate((cx, 0.0, 0.0))
    )
    frame = panel.cut(window_cut)
    return frame


def build_door_glass() -> cq.Workplane:
    """Translucent glazing seated in the door window opening."""
    cx = PANEL_X0 + DOOR_W / 2.0
    win_w = DOOR_W - 2 * DOOR_FRAME
    win_h = DOOR_H - 2 * DOOR_FRAME
    glass = (
        cq.Workplane("XY")
        .box(win_w + 0.006, GLASS_T, win_h + 0.006)  # slight overlap into frame rebate
        .translate((cx, 0.0, 0.0))
    )
    return glass


def build_door_emblem() -> cq.Workplane:
    """Red plus-cross emblem decal raised on the glass center."""
    arm = 0.030  # half-length of a cross arm
    bar = 0.024  # cross bar thickness
    t = 0.0015
    cx = PANEL_X0 + DOOR_W / 2.0
    vert = cq.Workplane("XY").box(bar, t, 2 * arm)
    horiz = cq.Workplane("XY").box(2 * arm, t, bar)
    cross = vert.union(horiz)
    # Position at the door center, bonded onto the front face of the glass.
    # The glass front face is at y=+GLASS_T/2; bond the decal so it slightly
    # embeds into the glass (touching) rather than floating in front of it.
    cross = cross.translate((cx, GLASS_T / 2.0, 0.0))
    return cross


def build_door_banner() -> cq.Workplane:
    """Red 'FIRST AID' banner strip across the top of the door frame."""
    cx = PANEL_X0 + DOOR_W / 2.0
    band = (
        cq.Workplane("XY")
        .box(DOOR_W - 0.012, 0.0020, DOOR_FRAME - 0.010)
        .translate(
            (
                cx,
                DOOR_T / 2.0,  # bonded onto the frame front face (touching decal)
                DOOR_H / 2.0 - (DOOR_FRAME - 0.010) / 2.0 - 0.005,
            )
        )
    )
    return band


def build_door_knob() -> cq.Workplane:
    """Small grip knob on the free (right) edge of the door frame."""
    knob = (
        cq.Workplane("XY")
        .cylinder(0.022, 0.008)  # height along Z by default? cylinder(height, radius)
    )
    # cq cylinder(height, radius) -> axis along Z. We want it protruding along +Y.
    knob = knob.rotate((0, 0, 0), (1, 0, 0), 90)
    # Seat the knob base into the frame front face so it is mounted, not floating.
    knob = knob.translate(
        (PANEL_X0 + DOOR_W - DOOR_FRAME / 2.0, DOOR_T / 2.0 + 0.007, 0.0)
    )
    return knob


# Interleaving knuckle stations (centers along the pin / local Z).
# Five knuckles tile the door height; door and body alternate so their
# Z-ranges do not overlap (real piano/barrel-hinge interleaving).
KNUCKLE_H = 0.055
DOOR_KNUCKLE_Z = [-0.116, 0.0, 0.116]  # door leaf carries 3 knuckles
BODY_KNUCKLE_Z = [-0.058, 0.058]  # body leaf carries 2, nested between door's


def build_hinge_knuckles(z_stations: list[float]) -> cq.Workplane:
    """Vertical hinge knuckles (barrels) about a shared local Z pin axis.

    Each knuckle is a short cylinder centered at a station along the door
    height. Body and door knuckle lists are offset so they interleave along
    the pin like a real piano/barrel hinge instead of colliding head-on.
    """
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
    # Each supply is lowered EMBED below its support surface so it overlaps the
    # shelf/floor it rests on (guarantees within-part connectivity, reads seated).
    EMBED = 0.002
    supply_y = WALL_T / 2.0 + 0.012
    floor_top = -INNER_H / 2.0  # cavity floor inner face (bottom wall top)
    mid_shelf_top = shelf_z[0] + SHELF_T / 2.0  # middle shelf top surface
    # Bottom row on the cavity floor (block half-heights 0.030 / 0.035).
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
    # Middle shelf row.
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
        mesh_from_cadquery(build_door_frame(), "door_frame"),
        material="white_metal",
        name="door_frame",
    )
    door.visual(
        mesh_from_cadquery(build_door_glass(), "door_glass"),
        material="glass",
        name="door_glass",
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
    model.material("glass", rgba=GLASS)
    model.material("steel", rgba=STEEL)
    model.material("shelf_grey", rgba=SHELF_GREY)
    model.material("supply_blue", rgba=SUPPLY_BLUE)
    model.material("supply_tan", rgba=SUPPLY_TAN)

    # --- Articulation: vertical hinge on the left front edge ---------------
    # The door's local frame has its hinge line at x=0, panel extending +X.
    # Place the joint frame at the left front corner of the body. The door at
    # q=0 sits flush across the front. Positive q (right-hand rule about +Z)
    # swings the free (+X) edge forward (+Y) and around -> door opens.
    # The joint origin carries the same Z_LIFT as the body geometry so the
    # door (authored centered about its hinge line) rides up with the body.
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
    for elem in ("body_shell",):
        body.get_visual(elem)
    for elem in (
        "door_frame",
        "door_glass",
        "door_emblem",
        "door_banner",
        "door_knob",
    ):
        door.get_visual(elem)

    # --- Closed pose: door spans the cabinet front and seats against it ---
    with ctx.pose({hinge: 0.0}):
        # Door footprint overlaps the body footprint in X and Z (covers face).
        ctx.expect_overlap(
            door, body, axes="xz", min_overlap=0.20,
            name="closed door covers cabinet front",
        )
        # Door sits in front of (+Y of) the body front face, lightly seated.
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

    # --- Glass + emblem are mounted on the door and centered in the window ---
    ctx.expect_within(
        door, door, axes="xz",
        inner_elem="door_emblem", outer_elem="door_glass",
        margin=0.001,
        name="emblem sits within the glass window",
    )
    ctx.expect_contact(
        door, door,
        elem_a="door_glass", elem_b="door_frame",
        contact_tol=0.004,
        name="glazing seated in the door frame",
    )

    # --- Open pose: actuating the joint swings the free edge away from closed ---
    rest_knob = closed_knob
    with ctx.pose({hinge: math.radians(120.0)}):
        open_knob = ctx.part_element_world_aabb(door, elem="door_knob")
    # The free-edge knob must move substantially when the door opens.
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
        # Opening should pull the free edge back (toward -X) and forward then around.
        ctx.check(
            "opened door free edge swings off the front (-X of closed)",
            open_c[0] < rest_c[0],
            details=f"rest_x={rest_c[0]:.3f}, open_x={open_c[0]:.3f}",
        )
    else:
        ctx.fail("door knob aabb resolvable", "could not resolve door_knob aabb")

    # --- Intentional seated overlaps (glazing rebate, emblem/banner decals) ---
    ctx.allow_overlap(
        door, door,
        elem_a="door_glass", elem_b="door_frame",
        reason="Glazing is captured in the door frame's window rebate (seated fit).",
    )
    ctx.allow_overlap(
        door, door,
        elem_a="door_emblem", elem_b="door_glass",
        reason="Red cross emblem is a raised decal bonded onto the glass face.",
    )
    ctx.allow_overlap(
        door, door,
        elem_a="door_banner", elem_b="door_frame",
        reason="FIRST AID banner is a printed decal on the upper door frame.",
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
