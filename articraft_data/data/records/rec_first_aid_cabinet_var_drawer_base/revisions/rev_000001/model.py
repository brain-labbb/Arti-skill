from __future__ import annotations

# Wall-mounted metal first aid cabinet with a hinged glass-window door and
# a pull-out supply drawer in the bottom compartment.
#
# Identity (from picture/Science/First aid cabinet/001.png):
#   A white sheet-metal wall cabinet with a red-cross "FIRST AID" emblem on its
#   door. The single front door is hinged on its LEFT edge and swings open
#   forward/around to expose interior shelves stocked with medical supplies.
#   A carry handle spans the top of the cabinet. A pull-out supply drawer
#   occupies the bottom compartment below the lowest shelf.
#
# Real mechanisms:
#   1. The door is a REVOLUTE joint about a VERTICAL (Z) hinge line on the
#      left front corner. Positive q swings the door open outward.
#   2. The drawer is a PRISMATIC joint sliding forward along +Y on drawer
#      slides. Positive q pulls the drawer out toward the viewer.
#
# Frame convention:
#   +X = right, -X = left (hinge side)
#   +Y = forward (out of the cabinet face, toward viewer); wall is at -Y
#   +Z = up
# The cabinet body is the single root; the door and drawer are its children.

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

# Bottom compartment (below shelf_0) -----------------------------------------
COMPARTMENT_FLOOR = -INNER_H / 2.0  # cavity floor inner face (body-local z)
COMPARTMENT_CEILING = -INNER_H / 6.0 - SHELF_T / 2.0  # shelf_0 bottom face
COMPARTMENT_H = COMPARTMENT_CEILING - COMPARTMENT_FLOOR

# Front panel covering the bottom compartment, with a drawer opening
FRONT_PANEL_BOTTOM = -BODY_H / 2.0
FRONT_PANEL_TOP = COMPARTMENT_CEILING
FRONT_PANEL_H = FRONT_PANEL_TOP - FRONT_PANEL_BOTTOM
FRONT_PANEL_W = INNER_W + 0.004  # 2 mm embed into each side wall
FRONT_PANEL_Y_CENTER = BODY_D / 2.0 - WALL_T / 2.0

# Drawer dimensions (narrower than the body to clear the left hinge barrels)
DRAWER_MARGIN = 0.003
DRAWER_H = COMPARTMENT_H - 2 * DRAWER_MARGIN
DRAWER_W = 0.260  # narrower than body inner width to clear hinge barrels at x~±0.164
DRAWER_D = BODY_D - WALL_T - 0.020
DRAWER_WALL_T = 0.003
DRAWER_FRONT_T = 0.006
DRAWER_OPENING_W = DRAWER_W + 0.006
DRAWER_OPENING_H = DRAWER_H + 0.004
DRAWER_FRONT_W = DRAWER_OPENING_W + 0.014
DRAWER_FRONT_H = DRAWER_OPENING_H + 0.010

# Drawer joint position (body-local z: center drawer box in compartment)
DRAWER_JOINT_Z_LOCAL = COMPARTMENT_FLOOR + DRAWER_MARGIN + DRAWER_H / 2.0
DRAWER_JOINT_Y = BODY_D / 2.0  # at body front face

# Drawer slide rails
SLIDE_RAIL_L = DRAWER_D + 0.010
SLIDE_RAIL_H = 0.010
SLIDE_RAIL_T = 0.003

# Materials --------------------------------------------------------------------
WHITE_METAL = (0.92, 0.92, 0.93, 1.0)  # painted white steel
RED = (0.78, 0.10, 0.12, 1.0)  # first-aid red
WHITE_TRIM = (0.97, 0.97, 0.97, 1.0)
GLASS = (0.62, 0.74, 0.80, 0.45)  # translucent pale glazing
STEEL = (0.70, 0.72, 0.75, 1.0)  # bright metal handle / hinges
SHELF_GREY = (0.85, 0.86, 0.87, 1.0)
SUPPLY_BLUE = (0.20, 0.40, 0.72, 1.0)
SUPPLY_TAN = (0.86, 0.80, 0.66, 1.0)
DRAWER_WHITE = (0.90, 0.90, 0.91, 1.0)  # slightly off-white drawer


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


def build_bottom_front_panel() -> cq.Workplane:
    """Front wall panel for the bottom compartment with a rectangular drawer opening."""
    fp_z = (FRONT_PANEL_BOTTOM + FRONT_PANEL_TOP) / 2.0
    panel = (
        cq.Workplane("XY")
        .box(FRONT_PANEL_W, WALL_T, FRONT_PANEL_H)
        .translate((0.0, FRONT_PANEL_Y_CENTER, fp_z))
    )
    # Cut the drawer opening (centered at the drawer joint height)
    opening = (
        cq.Workplane("XY")
        .box(DRAWER_OPENING_W, WALL_T + 0.010, DRAWER_OPENING_H)
        .translate((0.0, FRONT_PANEL_Y_CENTER, DRAWER_JOINT_Z_LOCAL))
    )
    return panel.cut(opening)


SHELF_EMBED = 0.003  # how far each shelf tucks into the side/back walls
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
    frame = panel.cut(window_cut)
    return frame


def build_door_glass() -> cq.Workplane:
    """Translucent glazing seated in the door window opening."""
    cx = PANEL_X0 + DOOR_W / 2.0
    win_w = DOOR_W - 2 * DOOR_FRAME
    win_h = DOOR_H - 2 * DOOR_FRAME
    glass = (
        cq.Workplane("XY")
        .box(win_w + 0.006, GLASS_T, win_h + 0.006)
        .translate((cx, 0.0, 0.0))
    )
    return glass


def build_door_emblem() -> cq.Workplane:
    """Red plus-cross emblem decal raised on the glass center."""
    arm = 0.030
    bar = 0.024
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
    band = (
        cq.Workplane("XY")
        .box(DOOR_W - 0.012, 0.0020, DOOR_FRAME - 0.010)
        .translate(
            (
                cx,
                DOOR_T / 2.0,
                DOOR_H / 2.0 - (DOOR_FRAME - 0.010) / 2.0 - 0.005,
            )
        )
    )
    return band


def build_door_knob() -> cq.Workplane:
    """Small grip knob on the free (right) edge of the door frame."""
    knob = cq.Workplane("XY").cylinder(0.022, 0.008)
    knob = knob.rotate((0, 0, 0), (1, 0, 0), 90)
    knob = knob.translate(
        (PANEL_X0 + DOOR_W - DOOR_FRAME / 2.0, DOOR_T / 2.0 + 0.007, 0.0)
    )
    return knob


KNUCKLE_H = 0.035
# All knuckle stations are above the drawer zone (drawer top ~ -0.063 body-local).
# Spacing gives ~1 mm gap between interleaved knuckles.
DOOR_KNUCKLE_Z = [-0.018, 0.058, 0.134]
BODY_KNUCKLE_Z = [0.018, 0.094]


def build_hinge_knuckles(z_stations: list[float]) -> cq.Workplane:
    """Vertical hinge knuckles (barrels) about a shared local Z pin axis."""
    barrels = None
    for z in z_stations:
        seg = cq.Workplane("XY").cylinder(KNUCKLE_H, HINGE_R).translate((0.0, 0.0, z))
        barrels = seg if barrels is None else barrels.union(seg)
    return barrels


# ---------------------------------------------------------------------------
# Drawer geometry builders
# ---------------------------------------------------------------------------
def build_drawer_tray() -> cq.Workplane:
    """Open-top drawer tray: four walls and a floor, hollow interior."""
    outer = cq.Workplane("XY").box(DRAWER_W, DRAWER_D, DRAWER_H)
    # Hollow interior: open at top, leaving walls on all four sides and a floor
    inner_w = DRAWER_W - 2 * DRAWER_WALL_T
    inner_d = DRAWER_D - 2 * DRAWER_WALL_T
    cavity_top_z = DRAWER_H / 2.0 + 0.005  # extend above box top for clean cut
    cavity_bottom_z = -DRAWER_H / 2.0 + DRAWER_WALL_T
    cavity_h = cavity_top_z - cavity_bottom_z
    cavity = (
        cq.Workplane("XY")
        .box(inner_w, inner_d, cavity_h)
        .translate((0.0, 0.0, cavity_bottom_z + cavity_h / 2.0))
    )
    tray = outer.cut(cavity)
    tray = tray.edges("|Z").fillet(0.002)
    return tray


def build_drawer_front_panel() -> cq.Workplane:
    """Flat drawer front face panel (slightly larger than the opening)."""
    # Positioned so its back face is at local y=0 (flush with drawer front)
    panel = (
        cq.Workplane("XY")
        .box(DRAWER_FRONT_W, DRAWER_FRONT_T, DRAWER_FRONT_H)
        .translate((0.0, DRAWER_FRONT_T / 2.0, 0.0))
    )
    panel = panel.edges("|Z").fillet(0.002)
    return panel


def build_drawer_pull() -> cq.Workplane:
    """Small bar pull handle: two posts with a horizontal grip bar."""
    post_spacing = 0.050
    post_h = 0.012
    post_r = 0.003
    bar_r = 0.004
    # Mounting posts protruding along +Y from the drawer face
    post_l = (
        cq.Workplane("XZ")
        .cylinder(post_h, post_r)
        .translate((-post_spacing / 2.0, post_h / 2.0, 0.0))
    )
    post_r_geom = (
        cq.Workplane("XZ")
        .cylinder(post_h, post_r)
        .translate((post_spacing / 2.0, post_h / 2.0, 0.0))
    )
    # Horizontal grip bar connecting the post tops (along X)
    bar = (
        cq.Workplane("YZ")
        .cylinder(post_spacing + 2 * bar_r, bar_r)
        .translate((0.0, post_h, 0.0))
    )
    handle = post_l.union(post_r_geom).union(bar)
    return handle


def build_slide_rail() -> cq.Workplane:
    """Thin flat slide-rail strip mounted on a cabinet side wall or drawer side."""
    return (
        cq.Workplane("XY")
        .box(SLIDE_RAIL_T, SLIDE_RAIL_L, SLIDE_RAIL_H)
        .edges("|Z").fillet(0.001)
    )


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
    # Bottom-compartment front panel with drawer opening
    body.visual(
        mesh_from_cadquery(build_bottom_front_panel(), "bottom_front_panel"),
        origin=Origin(xyz=(0.0, 0.0, Z_LIFT)),
        material="white_metal",
        name="bottom_front_panel",
    )

    # Interior shelves (upper two compartments only; bottom is the drawer)
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

    # Supplies on the middle shelf (upper compartments stay stocked).
    supply_mesh_a = mesh_from_cadquery(
        build_supply_block(0.090, 0.060, 0.060), "supply_a"
    )
    supply_mesh_b = mesh_from_cadquery(
        build_supply_block(0.070, 0.055, 0.070), "supply_b"
    )
    EMBED = 0.002
    supply_y = WALL_T / 2.0 + 0.012
    mid_shelf_top = shelf_z[0] + SHELF_T / 2.0
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

    # Body-side drawer slide rails (mounted on inner side walls)
    rail_mesh = mesh_from_cadquery(build_slide_rail(), "slide_rail")
    rail_x_signs = [-1.0, 1.0]
    rail_names = ["slide_rail_l", "slide_rail_r"]
    # Body rails are centered in Y on the closed drawer tray midpoint.
    body_rail_y = DRAWER_JOINT_Y - DRAWER_D / 2.0
    for i, (sign, rname) in enumerate(zip(rail_x_signs, rail_names)):
        body.visual(
            rail_mesh,
            origin=Origin(
                xyz=(
                    sign * (INNER_W / 2.0 - SLIDE_RAIL_T / 2.0 + 0.001),
                    body_rail_y,
                    DRAWER_JOINT_Z_LOCAL + Z_LIFT,
                )
            ),
            material="steel",
            name=rname,
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
    door.visual(
        mesh_from_cadquery(build_hinge_knuckles(DOOR_KNUCKLE_Z), "door_hinge"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="steel",
        name="door_hinge_barrel",
    )

    # --- Drawer (articulated child) ---------------------------------------
    drawer = model.part("drawer")
    # Drawer tray (open-top box), front face at local y=0 touching the front panel
    drawer_tray_y = -DRAWER_D / 2.0
    drawer.visual(
        mesh_from_cadquery(build_drawer_tray(), "drawer_tray"),
        origin=Origin(xyz=(0.0, drawer_tray_y, 0.0)),
        material="drawer_white",
        name="drawer_tray",
    )
    # Drawer front face panel (at local y=0, protruding forward)
    drawer.visual(
        mesh_from_cadquery(build_drawer_front_panel(), "drawer_front"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="white_metal",
        name="drawer_front",
    )
    # Pull handle mounted on the drawer front face, slightly above center
    drawer.visual(
        mesh_from_cadquery(build_drawer_pull(), "drawer_pull"),
        origin=Origin(xyz=(0.0, DRAWER_FRONT_T, 0.010)),
        material="steel",
        name="drawer_pull",
    )
    # Drawer-side slide rails (on outer sides of the drawer tray)
    drawer_rail_mesh = mesh_from_cadquery(build_slide_rail(), "drawer_slide_rail")
    for i, sign in enumerate([-1.0, 1.0]):
        drawer.visual(
            drawer_rail_mesh,
            origin=Origin(
                xyz=(
                    sign * (DRAWER_W / 2.0 + SLIDE_RAIL_T / 2.0 - 0.001),
                    drawer_tray_y,
                    0.0,
                )
            ),
            material="steel",
            name=f"drawer_rail_{i}",
        )
    # Supplies stored inside the drawer (centered in the tray Y extent)
    drawer_supply_meshes = [supply_mesh_a, supply_mesh_b]
    drawer_supply_mats = ["supply_blue", "supply_tan"]
    supply_floor_local_z = -DRAWER_H / 2.0 + DRAWER_WALL_T
    drawer_supply_offsets = [
        (-0.040, drawer_tray_y, supply_floor_local_z + 0.028),
        (0.040, drawer_tray_y, supply_floor_local_z + 0.033),
    ]
    for i, (mesh, mat, off) in enumerate(
        zip(drawer_supply_meshes, drawer_supply_mats, drawer_supply_offsets)
    ):
        drawer.visual(
            mesh,
            origin=Origin(xyz=off),
            material=mat,
            name=f"drawer_supply_{i}",
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

    # --- Articulation 1: vertical hinge on the left front edge -------------
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

    # --- Articulation 2: prismatic drawer slide along +Y -----------------
    # Joint origin at the body front face, centered on the drawer opening.
    # At q=0 the drawer is fully closed; positive q pulls it forward (+Y).
    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.0, DRAWER_JOINT_Y, DRAWER_JOINT_Z_LOCAL + Z_LIFT)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0, velocity=0.30, lower=0.0, upper=0.080
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
    drawer = object_model.get_part("drawer")
    hinge = object_model.get_articulation("body_to_door")
    slide = object_model.get_articulation("body_to_drawer")

    # --- Door mechanism: joint type + axis ---
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

    # --- Drawer mechanism: joint type + axis + limits ---
    ctx.check(
        "drawer joint is prismatic",
        slide.joint_type == ArticulationType.PRISMATIC,
        details=f"joint_type={slide.joint_type}",
    )
    slide_axis = tuple(round(a, 6) for a in slide.axis)
    ctx.check(
        "drawer slide axis is +Y (forward)",
        slide_axis == (0.0, 1.0, 0.0),
        details=f"axis={slide_axis}",
    )
    slide_limits = slide.motion_limits
    ctx.check(
        "drawer has positive travel range",
        slide_limits is not None
        and slide_limits.lower == 0.0
        and slide_limits.upper is not None
        and slide_limits.upper > 0.04,
        details=f"limits=({slide_limits.lower if slide_limits else None},"
        f"{slide_limits.upper if slide_limits else None})",
    )

    # --- Hero parts present ---
    body.get_visual("body_shell")
    body.get_visual("bottom_front_panel")
    for elem in (
        "door_frame", "door_glass", "door_emblem", "door_banner", "door_knob",
    ):
        door.get_visual(elem)
    for elem in ("drawer_tray", "drawer_front", "drawer_pull"):
        drawer.get_visual(elem)

    # --- Closed pose: door spans the cabinet front ---
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

    # --- Hinge support: barrels contact at the hinge line ---
    ctx.expect_contact(
        door, body,
        elem_a="door_hinge_barrel", elem_b="body_hinge_barrel",
        contact_tol=0.004,
        name="door hinge barrel meets body hinge barrel",
    )

    # --- Glass + emblem mounted on the door ---
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

    # --- Open door pose: free edge swings away ---
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

    # --- Drawer closed pose: front panel seats at the body front ---
    with ctx.pose({slide: 0.0}):
        # Drawer front overlaps the body front panel in XZ (covers the opening)
        ctx.expect_overlap(
            drawer, body, axes="xz",
            elem_a="drawer_front", elem_b="bottom_front_panel",
            min_overlap=0.08,
            name="closed drawer front covers the drawer opening",
        )
        closed_drawer_pos = ctx.part_world_position(drawer)

    # --- Drawer extended pose: slides forward along +Y ---
    with ctx.pose({slide: 0.060}):
        extended_drawer_pos = ctx.part_world_position(drawer)
        # Drawer tray is now partially outside the body front
        ctx.expect_overlap(
            drawer, body, axes="xz",
            elem_a="drawer_tray", elem_b="body_shell",
            min_overlap=0.05,
            name="extended drawer tray still partly overlaps body footprint",
        )
    if closed_drawer_pos is not None and extended_drawer_pos is not None:
        dy = extended_drawer_pos[1] - closed_drawer_pos[1]
        ctx.check(
            "drawer slides forward (+Y) when extended",
            dy > 0.03,
            details=f"closed_y={closed_drawer_pos[1]:.4f}, "
            f"extended_y={extended_drawer_pos[1]:.4f}, dy={dy:.4f}",
        )
    else:
        ctx.fail("drawer position resolvable", "could not resolve drawer world position")

    # --- Drawer front pull is mounted on the drawer front face ---
    ctx.expect_contact(
        drawer, drawer,
        elem_a="drawer_pull", elem_b="drawer_front",
        contact_tol=0.004,
        name="pull handle is mounted on the drawer front",
    )

    # --- Drawer opening is present in the bottom front panel ---
    # The drawer tray (inside) should be within the body footprint on XZ
    ctx.expect_within(
        drawer, body, axes="xz",
        inner_elem="drawer_tray", outer_elem="body_shell",
        margin=0.005,
        name="drawer tray fits within the cabinet body cross-section",
    )

    # --- Intentional seated overlaps ---
    # Door glazing and decals
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
    # Interior shelves tuck into the cabinet walls
    for shelf_elem in ("shelf_0", "shelf_1"):
        ctx.allow_overlap(
            body, body,
            elem_a=shelf_elem, elem_b="body_shell",
            reason="Shelf is welded into the side and back walls (seated shelf).",
        )
    # Bottom front panel embeds into body side walls
    ctx.allow_overlap(
        body, body,
        elem_a="bottom_front_panel", elem_b="body_shell",
        reason="Bottom front panel is spot-welded into the body side walls.",
    )
    # Body slide rails mounted on inner side walls (small embed)
    for rail_name in ("slide_rail_l", "slide_rail_r"):
        ctx.allow_overlap(
            body, body,
            elem_a=rail_name, elem_b="body_shell",
            reason="Drawer slide rail is fastened to the cabinet inner side wall.",
        )
    # Drawer rails mounted on drawer tray outer sides
    for i in range(2):
        ctx.allow_overlap(
            drawer, drawer,
            elem_a=f"drawer_rail_{i}", elem_b="drawer_tray",
            reason="Drawer slide rail is fastened to the drawer tray side wall.",
        )
    # Pull handle mounted on drawer front face
    ctx.allow_overlap(
        drawer, drawer,
        elem_a="drawer_pull", elem_b="drawer_front",
        reason="Pull handle posts are fastened through the drawer front panel.",
    )
    # Drawer front panel contacts body front panel when closed (seated trim)
    ctx.allow_overlap(
        drawer, body,
        elem_a="drawer_front", elem_b="bottom_front_panel",
        reason="Drawer front panel seats against the body front panel surrounding the opening.",
    )
    ctx.expect_contact(
        drawer, body,
        elem_a="drawer_front", elem_b="bottom_front_panel",
        contact_tol=0.008,
        name="drawer front contacts body front panel when closed",
    )
    # Drawer front sits behind the closed door (door must open first)
    ctx.allow_overlap(
        door, drawer,
        elem_a="door_frame", elem_b="drawer_front",
        reason="Drawer front is behind the closed door panel; door must open to access drawer.",
    )
    # Supplies inside the drawer rest on the tray floor
    for i in range(2):
        ctx.allow_overlap(
            drawer, drawer,
            elem_a=f"drawer_supply_{i}", elem_b="drawer_tray",
            reason="Supply box rests seated on the drawer tray floor (small embed).",
        )
    # Mid-shelf supplies on the body
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
