from __future__ import annotations

# Wall-mounted metal first aid cabinet with a hinged glass-window door over the
# upper shelf region and three pull-out supply drawers in the lower half.
#
# Identity (from picture/Science/First aid cabinet/001.png, drawer variant):
#   A white sheet-metal wall cabinet. The upper half has a single front door
#   hinged on its LEFT edge, swinging open to expose interior shelves stocked
#   with medical supplies. The lower half has three identical pull-out supply
#   drawers, each sliding forward on its own prismatic slide.
#   A carry handle spans the top of the cabinet.
#
# Mechanisms:
#   door: REVOLUTE about a vertical (Z) hinge line on the left front corner of
#     the upper region. Positive q swings the door open (outward to +Y then
#     around toward -X).
#   drawer_i (i=0..2): PRISMATIC along +Y (forward out of the cabinet). Positive
#     q extends the drawer outward. Uniform joint policy across all three.
#
# Frame convention:
#   +X = right, -X = left (hinge side)
#   +Y = forward (out of the cabinet face, toward viewer); wall is at -Y
#   +Z = up
# The cabinet body is the single root; the door and drawers are its children.

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

# Body interior cavity (front-open shell)
INNER_W = BODY_W - 2 * WALL_T  # 0.320
INNER_H = BODY_H - 2 * WALL_T  # 0.380
INNER_D = BODY_D - WALL_T  # 0.120 (back wall only; front is open)

# Ground rest: geometry is authored centered about z=0; lift the whole
# assembly by half the body height so the cabinet bottom sits on z = 0.
Z_LIFT = BODY_H / 2.0

# ---------------------------------------------------------------------------
# Upper / lower split
# ---------------------------------------------------------------------------
SPLIT_Z = 0.0  # body-local z dividing upper (door) from lower (drawers)
UPPER_H = INNER_H / 2.0  # 0.190 — upper door region height
LOWER_H = INNER_H / 2.0  # 0.190 — lower drawer region height

# ---------------------------------------------------------------------------
# Drawer geometry (lower half)
# ---------------------------------------------------------------------------
N_DRAWERS = 3
DRAWER_SLOT_H = LOWER_H / N_DRAWERS  # ~0.0633 per slot
DRAWER_DIVIDER_T = 0.004  # metal strip between openings
DRAWER_OPEN_H = DRAWER_SLOT_H - DRAWER_DIVIDER_T  # opening height in front panel
DRAWER_OPEN_W = INNER_W - 0.010  # opening width (5 mm margin each side)
DRAWER_WALL_T = 0.003  # drawer tray wall thickness
DRAWER_FRONT_T = 0.005  # front face panel thickness
DRAWER_BOX_H = DRAWER_OPEN_H - 0.006  # tray height (clearance to slide)
DRAWER_W = INNER_W - 0.008  # tray width (side clearance)
DRAWER_D = INNER_D - 0.012  # tray depth (back clearance)
DRAWER_TRAVEL = 0.085  # prismatic upper limit (meters)


def drawer_slot_z(i: int) -> float:
    """Body-local Z centre of drawer slot *i* (before Z_LIFT)."""
    return -INNER_H / 2.0 + DRAWER_SLOT_H * (i + 0.5)


# ---------------------------------------------------------------------------
# Door (upper half only)
# ---------------------------------------------------------------------------
DOOR_W = 0.330
DOOR_H = UPPER_H - 0.005  # 0.185 — slightly less than the upper region
DOOR_T = 0.022  # door panel thickness (Y)
DOOR_FRAME = 0.030  # white frame border width around the glass window
GLASS_T = 0.004  # glazing thickness

HINGE_INSET = 0.006  # how far the hinge line sits in front of the body front face
HINGE_R = 0.006  # hinge knuckle outer radius
PANEL_X0 = HINGE_R  # door panel left face sits at door-local x=HINGE_R

# Hinge Z centre in body-local frame (middle of the upper region)
HINGE_Z_BODY = SPLIT_Z + UPPER_H / 2.0  # 0.095

# Interleaving knuckle stations (centres along the pin / local Z).
KNUCKLE_H = 0.040
DOOR_KNUCKLE_Z = [-0.042, 0.042]  # door leaf carries 2 knuckles
BODY_KNUCKLE_Z = [0.0]  # body leaf carries 1, nested between door's

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------
WHITE_METAL = (0.92, 0.92, 0.93, 1.0)
RED = (0.78, 0.10, 0.12, 1.0)
WHITE_TRIM = (0.97, 0.97, 0.97, 1.0)
GLASS = (0.62, 0.74, 0.80, 0.45)
STEEL = (0.70, 0.72, 0.75, 1.0)
SHELF_GREY = (0.85, 0.86, 0.87, 1.0)
SUPPLY_BLUE = (0.20, 0.40, 0.72, 1.0)
SUPPLY_TAN = (0.86, 0.80, 0.66, 1.0)
DRAWER_WHITE = (0.90, 0.90, 0.91, 1.0)


# ---------------------------------------------------------------------------
# CadQuery geometry builders (authored directly in meters)
# ---------------------------------------------------------------------------
def build_body_shell() -> cq.Workplane:
    """Hollow white cabinet box with a horizontal divider and lower front
    panel carrying three drawer openings."""
    # Outer box with softened vertical edges.
    outer = cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H).edges("|Z").fillet(0.004)
    # Cavity cut — opens the entire front face.
    cavity = (
        cq.Workplane("XY")
        .box(INNER_W, BODY_D, INNER_H)
        .translate((0.0, WALL_T / 2.0 + 0.001, 0.0))
    )
    shell = outer.cut(cavity)

    # Horizontal divider at z = SPLIT_Z (separates door region from drawer region).
    divider = (
        cq.Workplane("XY")
        .box(INNER_W + 0.002, INNER_D + 0.002, WALL_T)
        .translate((0.0, WALL_T / 2.0, SPLIT_Z))
    )
    shell = shell.union(divider)

    # Lower front panel (covers the lower front opening, with drawer slots).
    lower_open_bottom = -INNER_H / 2.0
    lower_open_top = SPLIT_Z - WALL_T / 2.0
    lower_panel_h = lower_open_top - lower_open_bottom
    lower_panel_z = (lower_open_bottom + lower_open_top) / 2.0
    lower_panel = (
        cq.Workplane("XY")
        .box(INNER_W + 0.002, WALL_T, lower_panel_h)
        .translate((0.0, BODY_D / 2.0 - WALL_T / 2.0, lower_panel_z))
    )
    # Cut three rectangular drawer openings.
    for i in range(N_DRAWERS):
        slot_z = drawer_slot_z(i)
        opening = (
            cq.Workplane("XY")
            .box(DRAWER_OPEN_W, WALL_T + 0.010, DRAWER_OPEN_H)
            .translate((0.0, BODY_D / 2.0 - WALL_T / 2.0, slot_z))
        )
        lower_panel = lower_panel.cut(opening)
    shell = shell.union(lower_panel)
    return shell


# Shelf geometry --------------------------------------------------------
SHELF_EMBED = 0.003
SHELF_Y_BACK = -BODY_D / 2.0 + WALL_T - SHELF_EMBED
SHELF_Y_FRONT = 0.050  # clear of the closed-door panel back
SHELF_DEPTH = SHELF_Y_FRONT - SHELF_Y_BACK
SHELF_Y_CENTER = (SHELF_Y_FRONT + SHELF_Y_BACK) / 2.0


def build_shelf() -> cq.Workplane:
    """One interior shelf that tucks into the side and back walls."""
    return cq.Workplane("XY").box(INNER_W + 2 * SHELF_EMBED, SHELF_DEPTH, SHELF_T)


def build_supply_block(w: float, d: float, h: float) -> cq.Workplane:
    """A simple stacked medical-supply box."""
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


# Drawer tray geometry (shared helper) ---------------------------------
def build_drawer_tray() -> cq.Workplane:
    """Open-top drawer tray with a front pull face.

    Authored in the drawer part frame: at q = 0 the drawer front face sits at
    local y ≈ 0 (flush with the cabinet front at the articulation frame) and
    the tray body extends backward along -Y.
    """
    w = DRAWER_W
    d = DRAWER_D
    h = DRAWER_BOX_H
    t = DRAWER_WALL_T
    ft = DRAWER_FRONT_T

    # Front face panel — slightly oversized to overlap the opening edges.
    front_w = DRAWER_OPEN_W
    front_h = h + 0.004
    front = cq.Workplane("XY").box(front_w, ft, front_h)

    # Bottom plate (extends backward from front face inner edge).
    bottom = (
        cq.Workplane("XY")
        .box(w, d, t)
        .translate((0.0, -d / 2.0 - ft / 2.0, -h / 2.0 + t / 2.0))
    )

    # Back wall.
    back = (
        cq.Workplane("XY")
        .box(w, t, h)
        .translate((0.0, -d - ft / 2.0 + t / 2.0, 0.0))
    )

    # Side walls.
    left = (
        cq.Workplane("XY")
        .box(t, d, h)
        .translate((-w / 2.0 + t / 2.0, -d / 2.0 - ft / 2.0, 0.0))
    )
    right = (
        cq.Workplane("XY")
        .box(t, d, h)
        .translate((w / 2.0 - t / 2.0, -d / 2.0 - ft / 2.0, 0.0))
    )

    # Pull handle — small horizontal bar proud of the front face.
    handle = (
        cq.Workplane("XY")
        .box(0.060, 0.010, 0.010)
        .translate((0.0, ft / 2.0 + 0.005, 0.0))
    )

    tray = front.union(bottom).union(back).union(left).union(right).union(handle)
    return tray


# Door geometry (upper half) -------------------------------------------
def build_door_frame() -> cq.Workplane:
    """White metal door panel with a rectangular window opening (a frame)."""
    cx = PANEL_X0 + DOOR_W / 2.0
    panel = (
        cq.Workplane("XY")
        .box(DOOR_W, DOOR_T, DOOR_H)
        .translate((cx, 0.0, 0.0))
        .edges("|Z").fillet(0.003)
    )
    win_w = DOOR_W - 2 * DOOR_FRAME
    win_h = DOOR_H - 2 * DOOR_FRAME
    window_cut = (
        cq.Workplane("XY")
        .box(win_w, DOOR_T + 0.01, win_h)
        .translate((cx, 0.0, 0.0))
    )
    return panel.cut(window_cut)


def build_door_glass() -> cq.Workplane:
    """Translucent glazing seated in the door window opening."""
    cx = PANEL_X0 + DOOR_W / 2.0
    win_w = DOOR_W - 2 * DOOR_FRAME
    win_h = DOOR_H - 2 * DOOR_FRAME
    return (
        cq.Workplane("XY")
        .box(win_w + 0.006, GLASS_T, win_h + 0.006)
        .translate((cx, 0.0, 0.0))
    )


def build_door_emblem() -> cq.Workplane:
    """Red plus-cross emblem decal raised on the glass centre."""
    arm = 0.025
    bar = 0.020
    t = 0.0015
    cx = PANEL_X0 + DOOR_W / 2.0
    vert = cq.Workplane("XY").box(bar, t, 2 * arm)
    horiz = cq.Workplane("XY").box(2 * arm, t, bar)
    cross = vert.union(horiz)
    cross = cross.translate((cx, GLASS_T / 2.0, 0.0))
    return cross


def build_door_banner() -> cq.Workplane:
    """Red 'FIRST AID' banner strip across the top of the door frame."""
    cx = PANEL_X0 + DOOR_W / 2.0
    return (
        cq.Workplane("XY")
        .box(DOOR_W - 0.012, 0.0020, DOOR_FRAME - 0.010)
        .translate(
            (
                cx,
                DOOR_T / 2.0,
                DOOR_H / 2.0 - (DOOR_FRAME - 0.010) / 2.0 - 0.003,
            )
        )
    )


def build_door_knob() -> cq.Workplane:
    """Small grip knob on the free (right) edge of the door frame."""
    knob = cq.Workplane("XY").cylinder(0.018, 0.007)
    knob = knob.rotate((0, 0, 0), (1, 0, 0), 90)
    knob = knob.translate(
        (PANEL_X0 + DOOR_W - DOOR_FRAME / 2.0, DOOR_T / 2.0 + 0.006, 0.0)
    )
    return knob


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

    # Interior shelf in the upper region (visible through the glass door).
    shelf_mesh = mesh_from_cadquery(build_shelf(), "shelf")
    upper_shelf_z = SPLIT_Z + UPPER_H * 0.50  # mid-upper
    body.visual(
        shelf_mesh,
        origin=Origin(xyz=(0.0, SHELF_Y_CENTER, upper_shelf_z + Z_LIFT)),
        material="shelf_grey",
        name="shelf_0",
    )

    # Carry handle on top.
    body.visual(
        mesh_from_cadquery(build_handle(), "handle"),
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0 + Z_LIFT)),
        material="steel",
        name="carry_handle",
    )

    # Supplies in the upper region (on divider floor and on the shelf).
    supply_mesh_a = mesh_from_cadquery(
        build_supply_block(0.090, 0.060, 0.060), "supply_a"
    )
    supply_mesh_b = mesh_from_cadquery(
        build_supply_block(0.070, 0.055, 0.055), "supply_b"
    )
    EMBED = 0.002
    supply_y = WALL_T / 2.0 + 0.012
    divider_top = SPLIT_Z + WALL_T / 2.0  # top of horizontal divider
    shelf_top = upper_shelf_z + SHELF_T / 2.0

    body.visual(
        supply_mesh_a,
        origin=Origin(xyz=(-0.060, supply_y, divider_top + 0.030 - EMBED + Z_LIFT)),
        material="supply_blue",
        name="supply_lower_l",
    )
    body.visual(
        supply_mesh_b,
        origin=Origin(xyz=(0.060, supply_y, divider_top + 0.028 - EMBED + Z_LIFT)),
        material="supply_tan",
        name="supply_lower_r",
    )
    body.visual(
        supply_mesh_b,
        origin=Origin(xyz=(-0.040, supply_y, shelf_top + 0.028 - EMBED + Z_LIFT)),
        material="supply_tan",
        name="supply_upper_l",
    )
    body.visual(
        supply_mesh_a,
        origin=Origin(xyz=(0.055, supply_y, shelf_top + 0.030 - EMBED + Z_LIFT)),
        material="supply_blue",
        name="supply_upper_r",
    )

    # Hinge barrel on the BODY side (upper region, left front corner).
    # Embed 2 mm into the body wall so the knuckle reads as welded to the shell.
    body.visual(
        mesh_from_cadquery(build_hinge_knuckles(BODY_KNUCKLE_Z), "body_hinge"),
        origin=Origin(
            xyz=(-BODY_W / 2.0 + 0.006, BODY_D / 2.0 + HINGE_INSET - 0.002,
                 HINGE_Z_BODY + Z_LIFT)
        ),
        material="steel",
        name="body_hinge_barrel",
    )

    # --- Door (articulated child, upper half) -----------------------------
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
    door.visual(
        mesh_from_cadquery(build_hinge_knuckles(DOOR_KNUCKLE_Z), "door_hinge"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="steel",
        name="door_hinge_barrel",
    )

    # --- Drawers (3 × prismatic, lower half) ------------------------------
    drawer_tray_mesh = mesh_from_cadquery(build_drawer_tray(), "drawer_tray")
    for i in range(N_DRAWERS):
        drw = model.part(f"drawer_{i}")
        drw.visual(
            drawer_tray_mesh,
            material="drawer_white",
            name="tray",
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
    model.material("drawer_white", rgba=DRAWER_WHITE)

    # --- Articulation: vertical hinge on the left front edge (upper) ------
    hinge_x = -BODY_W / 2.0 + 0.006
    hinge_y = BODY_D / 2.0 + HINGE_INSET
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, HINGE_Z_BODY + Z_LIFT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=2.0, lower=0.0, upper=math.radians(150.0)
        ),
    )

    # --- Articulations: prismatic drawer slides (lower half) -------------
    for i in range(N_DRAWERS):
        slot_z = drawer_slot_z(i)
        model.articulation(
            f"body_to_drawer_{i}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=f"drawer_{i}",
            # Joint frame at the front panel face at this drawer slot centre.
            origin=Origin(xyz=(0.0, BODY_D / 2.0, slot_z + Z_LIFT)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=12.0, velocity=0.30, lower=0.0, upper=DRAWER_TRAVEL
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

    drawers = [object_model.get_part(f"drawer_{i}") for i in range(N_DRAWERS)]
    drawer_joints = [
        object_model.get_articulation(f"body_to_drawer_{i}") for i in range(N_DRAWERS)
    ]

    # --- Door mechanism ---
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

    # --- Drawer mechanisms (uniform policy) ---
    for i, (drw, jt) in enumerate(zip(drawers, drawer_joints)):
        ctx.check(
            f"drawer_{i} joint is prismatic",
            jt.joint_type == ArticulationType.PRISMATIC,
            details=f"joint_type={jt.joint_type}",
        )
        jt_axis = tuple(round(a, 6) for a in jt.axis)
        ctx.check(
            f"drawer_{i} slides forward along +Y",
            jt_axis == (0.0, 1.0, 0.0),
            details=f"axis={jt_axis}",
        )
        jl = jt.motion_limits
        ctx.check(
            f"drawer_{i} has uniform travel policy",
            jl is not None
            and jl.lower == 0.0
            and abs(jl.upper - DRAWER_TRAVEL) < 1e-6,
            details=f"limits=({jl.lower if jl else None},{jl.upper if jl else None})",
        )

    # --- Hero parts present ---
    body.get_visual("body_shell")
    for elem in (
        "door_frame", "door_glass", "door_emblem", "door_banner", "door_knob",
    ):
        door.get_visual(elem)
    for i in range(N_DRAWERS):
        drawers[i].get_visual("tray")

    # --- Door closed pose ---
    with ctx.pose({hinge: 0.0}):
        ctx.expect_overlap(
            door, body, axes="xz", min_overlap=0.10,
            name="closed door covers upper cabinet front",
        )
        door_aabb = ctx.part_world_aabb(door)
        body_aabb = ctx.part_world_aabb(body)
        ctx.check(
            "closed door is in front of body face",
            door_aabb is not None
            and body_aabb is not None
            and door_aabb[1][1] > body_aabb[1][1] - 0.005,
            details=f"door_max_y={door_aabb[1][1] if door_aabb else None}, "
            f"body_max_y={body_aabb[1][1] if body_aabb else None}",
        )
        closed_knob = ctx.part_element_world_aabb(door, elem="door_knob")

    # --- Door hinge support ---
    ctx.expect_contact(
        door, body,
        elem_a="door_hinge_barrel", elem_b="body_hinge_barrel",
        contact_tol=0.004,
        name="door hinge barrel meets body hinge barrel",
    )

    # --- Glass + emblem on the door ---
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

    # --- Door open pose ---
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
            "opening the hinge swings the door free edge",
            moved > 0.08,
            details=f"free-edge travel={moved:.3f} m",
        )
    else:
        ctx.fail("door knob aabb resolvable", "could not resolve door_knob aabb")

    # --- Drawer closed pose: trays inside the cabinet ---
    for i, (drw, jt) in enumerate(zip(drawers, drawer_joints)):
        with ctx.pose({jt: 0.0}):
            drw_aabb = ctx.part_world_aabb(drw)
            body_aabb = ctx.part_world_aabb(body)
            ctx.check(
                f"drawer_{i} closed is inside cabinet footprint (XY)",
                drw_aabb is not None
                and body_aabb is not None
                and drw_aabb[0][0] >= body_aabb[0][0] - 0.005
                and drw_aabb[1][0] <= body_aabb[1][0] + 0.005,
                details=f"drawer_x=[{drw_aabb[0][0]:.3f},{drw_aabb[1][0]:.3f}], "
                f"body_x=[{body_aabb[0][0]:.3f},{body_aabb[1][0]:.3f}]"
                if drw_aabb and body_aabb else "aabb unavailable",
            )

    # --- Drawer extended pose: trays slide forward ---
    for i, (drw, jt) in enumerate(zip(drawers, drawer_joints)):
        with ctx.pose({jt: 0.0}):
            closed_pos = ctx.part_world_position(drw)
        with ctx.pose({jt: DRAWER_TRAVEL}):
            extended_pos = ctx.part_world_position(drw)
        if closed_pos is not None and extended_pos is not None:
            delta_y = extended_pos[1] - closed_pos[1]
            ctx.check(
                f"drawer_{i} extends forward (+Y) when opened",
                delta_y > 0.04,
                details=f"delta_y={delta_y:.3f} m",
            )

    # --- Drawers occupy the lower half (Z separation from door) ---
    for i, drw in enumerate(drawers):
        drw_aabb = ctx.part_world_aabb(drw)
        door_aabb = ctx.part_world_aabb(door)
        if drw_aabb and door_aabb:
            ctx.check(
                f"drawer_{i} is below the door region",
                drw_aabb[1][2] < door_aabb[0][2] + 0.010,
                details=f"drawer_max_z={drw_aabb[1][2]:.3f}, "
                f"door_min_z={door_aabb[0][2]:.3f}",
            )

    # --- Intentional seated overlaps ---
    # Door frame seats against the cabinet front face (small Y embed at closed pose).
    ctx.allow_overlap(
        body, door,
        elem_a="body_shell", elem_b="door_frame",
        reason="Door frame seats against the cabinet front face at the closed position.",
    )
    ctx.expect_gap(
        door, body,
        axis="y",
        max_penetration=0.008,
        positive_elem="door_frame",
        negative_elem="body_shell",
        name="door frame penetration into body is shallow (seated fit)",
    )

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
    ctx.allow_overlap(
        body, body,
        elem_a="body_hinge_barrel", elem_b="body_shell",
        reason="Hinge barrel is embedded into the body side wall (welded mount).",
    )
    ctx.expect_contact(
        body, body,
        elem_a="body_hinge_barrel", elem_b="body_shell",
        contact_tol=0.003,
        name="body hinge barrel contacts body shell",
    )

    ctx.allow_overlap(
        body, body,
        elem_a="shelf_0", elem_b="body_shell",
        reason="Shelf is welded into the side and back walls (seated shelf).",
    )
    # Supplies rest with a small embed.
    for supply_name, seat in (
        ("supply_lower_l", "body_shell"),
        ("supply_lower_r", "body_shell"),
        ("supply_upper_l", "shelf_0"),
        ("supply_upper_r", "shelf_0"),
    ):
        ctx.allow_overlap(
            body, body,
            elem_a=supply_name, elem_b=seat,
            reason=f"Supply box rests seated (small embed for connectivity).",
        )

    # Drawer trays overlap the body shell at closed pose (trays slide inside
    # the cabinet cavity through the front-panel openings).
    for i in range(N_DRAWERS):
        ctx.allow_overlap(
            body, f"drawer_{i}",
            reason="Drawer tray slides inside the cabinet cavity through its front-panel opening.",
        )
        # Prove retained insertion: drawer stays within the cabinet on X/Z at rest.
        ctx.expect_within(
            drawers[i], body,
            axes="xz",
            margin=0.005,
            name=f"drawer_{i} stays within cabinet footprint (XZ) at rest",
        )
        # Prove the drawer actually moves on Y between closed and extended.
        with ctx.pose({drawer_joints[i]: 0.0}):
            closed_y = ctx.part_world_position(drawers[i])
        with ctx.pose({drawer_joints[i]: DRAWER_TRAVEL}):
            ext_y = ctx.part_world_position(drawers[i])
        if closed_y is not None and ext_y is not None:
            ctx.check(
                f"drawer_{i} forward travel matches joint limit",
                abs((ext_y[1] - closed_y[1]) - DRAWER_TRAVEL) < 0.005,
                details=f"measured_travel={ext_y[1] - closed_y[1]:.4f}, "
                f"expected={DRAWER_TRAVEL}",
            )

    return ctx.report()


object_model = build_object_model()
