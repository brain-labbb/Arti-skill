from __future__ import annotations

# Rolling tool cart / roller cabinet — cabinet-door variant.
#
# The object is a red-and-black mobile tool chest:
#   - A black steel carcass (root) standing on four black swivel casters.
#   - Three shallow red drawers across the upper front face, each sliding out
#     on a PRISMATIC joint along +Y.
#   - A single red cabinet door covering the lower front opening, hinged on the
#     left (-X) side with a vertical Z-axis REVOLUTE joint that swings open.
#   - A red perforated pegboard panel on the left side wall.
#   - A black top tray with a molded socket recess and a raised lip.
#   - A dark tubular push handle with a red rubber grip on the top-right rear.
#
# Coordinate convention: +Z up, +Y toward the front (the side the drawers, door,
# and handle grip face), +X to the right. The carcass floor sits above the
# caster height so the whole cart rests on the four wheels at z = 0.
#
# Primary mechanisms:
#   - THREE UPPER DRAWERS slide out on PRISMATIC joints along +Y.
#   - CABINET DOOR swings open on a REVOLUTE joint (vertical Z hinge on the
#     left side of the front opening).
#   - Four casters SWIVEL about vertical Z (CONTINUOUS) and wheels ROLL about X.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Dimensions (meters)
# ---------------------------------------------------------------------------
# Cabinet carcass
CAB_W = 0.560  # cabinet width (X)
CAB_D = 0.420  # cabinet depth (Y)
CAB_H = 0.640  # cabinet body height (Z), not counting top trim / casters
WALL = 0.012  # carcass wall thickness
BASE_BAND = 0.055  # solid toe-kick band at the bottom, below the drawers
TOP_MARGIN = 0.006  # solid band under the top trim, above the top drawer

# Caster stand-off: gap between ground and cabinet floor
CASTER_GAP = 0.150  # vertical room the casters/swivel forks occupy
FLOOR_Z = CASTER_GAP  # cabinet floor underside sits here

# Top assembly
TOP_TRIM_H = 0.022  # black top trim band above the drawer stack
TRAY_LIP = 0.020  # raised lip around the top tray
TRAY_WALL = 0.010

# Drawer stack: three shallow drawers from top to bottom, then a cabinet door
# covering the lower section where two deep drawers used to be.
DRAWER_FACE_T = 0.018  # red drawer-front panel thickness (Y)
DRAWER_GAP = 0.006  # reveal gap between adjacent drawer faces
SIDE_REVEAL = 0.014  # gap from carcass side wall to drawer face edge
# (h0..h2) drawer FACE heights, top -> bottom
DRAWER_HEIGHTS = (0.072, 0.072, 0.072)
N_DRAWERS = len(DRAWER_HEIGHTS)

DRAWER_TRAVEL = 0.300  # pull-out travel along +Y
DRAWER_CLEAR = 0.004  # sliding clearance per side inside the opening

# Cabinet door dimensions (covers the lower section: two deep-drawer heights
# plus the gap between them).
DOOR_DEEP_H = 0.150  # height of each deep drawer slot the door replaces
DOOR_H = 2 * DOOR_DEEP_H + DRAWER_GAP  # total door face height
DOOR_FACE_T = DRAWER_FACE_T  # same thickness as drawer faces
DOOR_OPEN_ANGLE = 1.50  # max open angle in radians (~86 degrees)

# Side pegboard (left wall, -X face)
PEG_W = 0.300  # pegboard panel size along Y
PEG_H = 0.340  # pegboard panel size along Z
PEG_T = 0.006
PEG_HOLE_D = 0.012
PEG_PITCH = 0.030

# Push handle
HANDLE_TUBE_R = 0.013
HANDLE_RISE = 0.150  # how far the handle loop rises above the top trim
HANDLE_REACH = 0.110  # how far the grip extends back past the cabinet (-Y)
GRIP_R = 0.017
GRIP_LEN = 0.230

# Casters
CASTER_WHEEL_R = 0.052
CASTER_WHEEL_W = 0.034
CASTER_FORK_DROP = 0.030  # swivel offset: wheel axle trails behind swivel axis
CASTER_INSET_X = 0.070  # caster center inset from the cabinet side
CASTER_INSET_Y = 0.060  # caster center inset from the cabinet front/back

# Material colors
RED = (0.74, 0.07, 0.07, 1.0)
BLACK = (0.10, 0.10, 0.11, 1.0)
DARK = (0.16, 0.16, 0.18, 1.0)
RUBBER = (0.09, 0.09, 0.10, 1.0)
GRIP_RED = (0.70, 0.10, 0.10, 1.0)
STEEL = (0.55, 0.55, 0.58, 1.0)


# ---------------------------------------------------------------------------
# Derived layout helpers
# ---------------------------------------------------------------------------
def _drawer_band_top() -> float:
    """Z of the top edge of the drawer stack. Sits below the cavity ceiling
    (z1 - WALL) with a clear gap so the top drawer never touches the top wall."""
    return FLOOR_Z + CAB_H - WALL - TOP_MARGIN


def _drawer_face_centers() -> list[float]:
    """Return the world Z center of each drawer face, top -> bottom."""
    top = _drawer_band_top()
    centers: list[float] = []
    z = top
    for h in DRAWER_HEIGHTS:
        z -= DRAWER_GAP
        centers.append(z - h / 2.0)
        z -= h
    return centers


def _door_opening() -> dict[str, float]:
    """Return the door opening bounds: top z, bottom z, center z, and height."""
    centers = _drawer_face_centers()
    last_h = DRAWER_HEIGHTS[-1]
    # Bottom of the last shallow drawer area
    last_bottom = centers[-1] - last_h / 2.0
    # Door opening starts one gap below the last drawer
    door_top = last_bottom - DRAWER_GAP
    door_bottom = door_top - DOOR_H
    door_cz = (door_top + door_bottom) / 2.0
    return {
        "top": door_top,
        "bottom": door_bottom,
        "center_z": door_cz,
        "height": DOOR_H,
    }


# ---------------------------------------------------------------------------
# Geometry builders
# ---------------------------------------------------------------------------
def _carcass_mesh() -> object:
    """Black steel cabinet carcass: a hollow box open at the front (+Y) so the
    drawers and door slot in, with a top trim band, a recessed top tray, and
    the left side pegboard fused in. Sits above the caster gap."""
    z0 = FLOOR_Z
    z1 = FLOOR_Z + CAB_H

    body = (
        cq.Workplane("XY", origin=(0, 0, (z0 + z1) / 2.0))
        .box(CAB_W, CAB_D, CAB_H)
    )
    try:
        body = body.edges("|Z").fillet(0.010)
    except Exception:
        pass

    # Interior cavity: leave WALL-thick side/back/top walls and a solid BASE_BAND
    # toe-kick at the bottom. The cavity is open at the front (+Y).
    cav_z0 = z0 + BASE_BAND
    cav_z1 = z1 - WALL
    cavity = (
        cq.Workplane(
            "XY",
            origin=(0, WALL, (cav_z0 + cav_z1) / 2.0),
        )
        .box(CAB_W - 2 * WALL, CAB_D, cav_z1 - cav_z0)
    )
    body = body.cut(cavity)

    # --- Top trim band ---
    trim_overlap = 0.020
    trim_h = TOP_TRIM_H + trim_overlap
    trim_z = z1 + TOP_TRIM_H / 2.0 - trim_overlap / 2.0
    trim = (
        cq.Workplane("XY", origin=(0, 0, trim_z))
        .box(CAB_W + 0.012, CAB_D + 0.012, trim_h)
    )
    try:
        trim = trim.edges("|Z").fillet(0.012)
    except Exception:
        pass
    body = body.union(trim)

    # --- Top tray ---
    tray_top = z1 + TOP_TRIM_H
    tray_overlap = 0.012
    tray_h = TRAY_LIP + tray_overlap
    tray_outer = (
        cq.Workplane("XY", origin=(0, 0, tray_top + TRAY_LIP / 2.0 - tray_overlap / 2.0))
        .box(CAB_W + 0.010, CAB_D + 0.010, tray_h)
    )
    try:
        tray_outer = tray_outer.edges("|Z").fillet(0.012)
    except Exception:
        pass
    basin = (
        cq.Workplane("XY", origin=(0, 0, tray_top + TRAY_LIP))
        .box(CAB_W - 0.040, CAB_D - 0.040, 2 * (TRAY_LIP - TRAY_WALL))
    )
    tray_outer = tray_outer.cut(basin)
    for sx in (-0.14, 0.0, 0.14):
        pocket = (
            cq.Workplane("XY", origin=(sx, 0.04, tray_top + TRAY_WALL))
            .circle(0.016)
            .extrude(2 * TRAY_LIP)
        )
        tray_outer = tray_outer.cut(pocket)
    body = body.union(tray_outer)

    return mesh_from_cadquery(body, "carcass")


def _pegboard_mesh() -> object:
    """Red perforated pegboard panel mounted flat on the left (-X) side wall."""
    x_face = -CAB_W / 2.0 - PEG_T / 2.0 + 0.004
    z_c = FLOOR_Z + CAB_H * 0.55
    plate = (
        cq.Workplane("YZ", origin=(x_face, 0.0, z_c))
        .box(PEG_W, PEG_H, PEG_T)
    )
    nyy = int(PEG_W // PEG_PITCH)
    nzz = int(PEG_H // PEG_PITCH)
    y0 = -((nyy - 1) * PEG_PITCH) / 2.0
    zz0 = -((nzz - 1) * PEG_PITCH) / 2.0
    for iy in range(nyy):
        for iz in range(nzz):
            yy = y0 + iy * PEG_PITCH
            zz = zz0 + iz * PEG_PITCH
            hole = (
                cq.Workplane("YZ", origin=(x_face, yy, z_c + zz))
                .circle(PEG_HOLE_D / 2.0)
                .extrude(2 * PEG_T, both=True)
            )
            plate = plate.cut(hole)
    return mesh_from_cadquery(plate, "pegboard")


def _drawer_mesh(index: int) -> object:
    """One shallow drawer: a red front face panel plus a black box that slides
    into the carcass. Authored centered at the local origin so the prismatic
    joint frame sits at the drawer's resting center."""
    h = DRAWER_HEIGHTS[index]
    face_w = CAB_W - 2 * SIDE_REVEAL
    face_h = h
    box_w = face_w - 2 * DRAWER_CLEAR
    box_h = h - 2 * DRAWER_CLEAR
    box_depth = CAB_D - WALL - 0.030

    # Front face: rounded red panel sitting on +Y.
    face_y = CAB_D / 2.0 - DRAWER_FACE_T / 2.0
    face = (
        cq.Workplane("XY", origin=(0, face_y, 0))
        .box(face_w, DRAWER_FACE_T, face_h)
    )
    try:
        face = face.edges("|Y").fillet(0.006)
    except Exception:
        pass

    # Recessed finger pull along the top of the face.
    pull = (
        cq.Workplane("XY", origin=(0, face_y + 0.002, face_h / 2.0 - 0.012))
        .box(face_w * 0.62, DRAWER_FACE_T, 0.016)
    )
    face = face.cut(pull)

    # Drawer box (black): open-top tray behind the face.
    box_y_c = CAB_D / 2.0 - DRAWER_FACE_T - box_depth / 2.0
    box_outer = (
        cq.Workplane("XY", origin=(0, box_y_c, 0))
        .box(box_w, box_depth, box_h)
    )
    box = box_outer.faces(">Z").shell(-0.005)

    drawer = face.union(box)
    return mesh_from_cadquery(drawer, f"drawer_{index}")


def _cabinet_door_mesh() -> object:
    """Cabinet door panel authored in a hinge-line local frame. The part frame
    sits at the hinge (left edge of the opening, vertical center of the door).
    The door panel extends along local +X from the hinge. The front face is on
    local +Y.

    Features:
    - Red front face panel matching the drawer faces.
    - A recessed finger pull near the top edge.
    - A surface-mounted steel handle/latch on the right side.
    - Two small hinge barrel stubs on the left (-X) edge that visually connect
      to the carcass hinge point.
    """
    face_w = CAB_W - 2 * SIDE_REVEAL
    face_h = DOOR_H

    # Main door face panel: centered at (face_w/2, 0, 0) in the door frame.
    face = (
        cq.Workplane("XY", origin=(face_w / 2.0, 0.0, 0.0))
        .box(face_w, DOOR_FACE_T, face_h)
    )
    try:
        face = face.edges("|Y").fillet(0.006)
    except Exception:
        pass

    # Recessed finger pull near the top-right of the face.
    pull = (
        cq.Workplane("XY", origin=(face_w * 0.65, 0.002, face_h / 2.0 - 0.020))
        .box(0.080, DOOR_FACE_T, 0.018)
    )
    face = face.cut(pull)

    # Surface-mounted handle/latch bracket on the right side of the door face.
    handle_x = face_w - 0.040
    handle_z = 0.0
    handle_bracket = (
        cq.Workplane("XY", origin=(handle_x, DOOR_FACE_T / 2.0 + 0.006, handle_z))
        .box(0.020, 0.012, 0.070)
    )
    # Round the handle bracket edges for a grip-friendly shape.
    try:
        handle_bracket = handle_bracket.edges("|Y").fillet(0.004)
    except Exception:
        pass
    face = face.union(handle_bracket)

    # Hinge barrel stubs on the left edge: two short cylinders that represent
    # the hinge knuckles at the top and bottom thirds of the door.
    barrel_r = 0.008
    barrel_h = 0.030
    for frac in (0.30, 0.70):
        bz = -face_h / 2.0 + frac * face_h
        barrel = (
            cq.Workplane("XY", origin=(0.0, 0.0, bz))
            .circle(barrel_r)
            .extrude(barrel_h)
        )
        # Rotate so the barrel axis is vertical (along Z) — already extruded
        # along Z, centered at x=0 (the hinge line).
        # Offset down so it straddles the hinge point.
        barrel = (
            cq.Workplane("XY", origin=(0.0, 0.0, bz - barrel_h / 2.0))
            .circle(barrel_r)
            .extrude(barrel_h)
        )
        face = face.union(barrel)

    return mesh_from_cadquery(face, "cabinet_door")


# Handle geometry constants (carcass frame).
HANDLE_TRIM_TOP = FLOOR_Z + CAB_H + TOP_TRIM_H
HANDLE_Z_BASE = HANDLE_TRIM_TOP + TRAY_LIP + 0.004
HANDLE_Y_ANCHOR = -CAB_D / 2.0 + 0.020
HANDLE_X_OFF = CAB_W / 2.0 - 0.065
HANDLE_X_IN = HANDLE_X_OFF - 0.170
HANDLE_Z_TOP = HANDLE_Z_BASE + HANDLE_RISE
HANDLE_Y_GRIP = -CAB_D / 2.0 - HANDLE_REACH


def _handle_mesh() -> object:
    """Dark tubular push handle: an inverted-U loop rising from the top-rear of
    the carcass and reaching back, with a red rubber grip sleeve over the top
    run. Returns (frame_tube_mesh, grip_mesh)."""
    z_base = HANDLE_Z_BASE
    y_back = HANDLE_Y_ANCHOR
    x_off = HANDLE_X_OFF
    x_in = HANDLE_X_IN
    z_top = HANDLE_Z_TOP
    y_grip = HANDLE_Y_GRIP

    pts = [
        (x_off, y_back, z_base),
        (x_off, y_back, z_base + 0.050),
        (x_off, (y_back + y_grip) / 2.0, z_top - 0.012),
        (x_off, y_grip, z_top),
        (x_in, y_grip, z_top),
        (x_in, (y_back + y_grip) / 2.0, z_top - 0.012),
        (x_in, y_back, z_base + 0.050),
        (x_in, y_back, z_base),
    ]
    wire_pts = [cq.Vector(*p) for p in pts]
    spline_edge = cq.Edge.makeSpline(wire_pts)
    wire = cq.Wire.assembleEdges([spline_edge])
    start = wire_pts[0]
    tan = spline_edge.tangentAt(0.0)
    tube = (
        cq.Workplane(cq.Plane(origin=start.toTuple(), normal=tan.toTuple()))
        .circle(HANDLE_TUBE_R)
        .sweep(cq.Workplane(obj=wire), transition="round")
    )

    for sx in (x_off, x_in):
        boss = (
            cq.Workplane("XY", origin=(sx, y_back, z_base - 0.014))
            .circle(HANDLE_TUBE_R + 0.004)
            .extrude(0.028)
        )
        tube = tube.union(boss)

    frame_mesh = mesh_from_cadquery(tube, "handle_frame")

    grip = (
        cq.Workplane("YZ", origin=((x_in + x_off) / 2.0, y_grip, z_top))
        .circle(GRIP_R)
        .extrude(GRIP_LEN / 2.0, both=True)
    )
    grip_mesh = mesh_from_cadquery(grip, "handle_grip")
    return frame_mesh, grip_mesh


AXLE_Z = -CASTER_GAP + CASTER_WHEEL_R


def _caster_fork_mesh() -> object:
    """One swivel-caster yoke authored in a LOCAL frame whose swivel axis is
    local +Z through the origin."""
    leg_y = -CASTER_FORK_DROP

    plate = (
        cq.Workplane("XY", origin=(0, 0, -0.007))
        .box(0.060, 0.060, 0.014)
    )
    post = (
        cq.Workplane("XY", origin=(0, 0, 0.0))
        .circle(0.013)
        .extrude(0.030)
    )
    post = post.union(
        cq.Workplane("XY", origin=(0, 0, -0.018)).circle(0.013).extrude(0.018)
    )
    fork = plate.union(post)

    riser_top = 0.0
    riser_bot = AXLE_Z - 0.006
    fork = fork.union(
        cq.Workplane(
            "XY",
            origin=(0.0, leg_y * 0.45, (riser_top + riser_bot) / 2.0),
        )
        .box(0.052, abs(leg_y) + 0.030, abs(riser_top - riser_bot))
    )

    leg_top = AXLE_Z + 0.022
    leg_bot = AXLE_Z - 0.014
    for sx in (-1.0, 1.0):
        fork = fork.union(
            cq.Workplane(
                "XY",
                origin=(
                    sx * (CASTER_WHEEL_W / 2.0 + 0.006),
                    leg_y,
                    (leg_top + leg_bot) / 2.0,
                ),
            )
            .box(0.014, 0.036, abs(leg_top - leg_bot))
        )
    return mesh_from_cadquery(fork, "caster_fork")


def _caster_wheel_mesh() -> object:
    """Wheel authored centered on its own part origin (the axle)."""
    wheel = (
        cq.Workplane("YZ", origin=(0, 0, 0))
        .circle(CASTER_WHEEL_R)
        .extrude(CASTER_WHEEL_W / 2.0, both=True)
    )
    hub_cut = (
        cq.Workplane("YZ", origin=(0, 0, 0))
        .circle(CASTER_WHEEL_R - 0.012)
        .extrude(CASTER_WHEEL_W / 2.0 + 0.002, both=True)
    )
    rim = wheel.cut(hub_cut)
    hub = (
        cq.Workplane("YZ", origin=(0, 0, 0))
        .circle(0.015)
        .extrude(CASTER_WHEEL_W / 2.0, both=True)
    )
    disc = (
        cq.Workplane("YZ", origin=(0, 0, 0))
        .circle(CASTER_WHEEL_R - 0.012)
        .extrude(0.004, both=True)
    )
    wheel = rim.union(hub).union(disc)
    return mesh_from_cadquery(wheel, "caster_wheel")


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_tool_cart")

    m_red = model.material("cart_red", rgba=RED)
    m_black = model.material("cart_black", rgba=BLACK)
    m_dark = model.material("cart_dark", rgba=DARK)
    m_rubber = model.material("caster_rubber", rgba=RUBBER)
    m_grip = model.material("grip_red", rgba=GRIP_RED)
    m_steel = model.material("brushed_steel", rgba=STEEL)

    # --- Carcass (root) ---
    carcass = model.part("cabinet_frame")
    carcass.visual(_carcass_mesh(), material=m_black, name="carcass_shell")
    carcass.visual(_pegboard_mesh(), material=m_red, name="side_pegboard")
    carcass.inertial = Inertial.from_geometry(
        Box((CAB_W, CAB_D, CAB_H)),
        mass=22.0,
        origin=Origin(xyz=(0.0, 0.0, FLOOR_Z + CAB_H / 2.0)),
    )

    # --- Push handle (fixed to the carcass top-rear) ---
    handle = model.part("push_handle")
    frame_mesh, grip_mesh = _handle_mesh()
    handle.visual(frame_mesh, material=m_dark, name="handle_frame")
    handle.visual(grip_mesh, material=m_grip, name="handle_grip")
    handle.inertial = Inertial.from_geometry(
        Box((0.20, HANDLE_REACH + 0.10, HANDLE_RISE)),
        mass=1.2,
        origin=Origin(
            xyz=(CAB_W / 2.0 - 0.13, -CAB_D / 2.0 - 0.02, FLOOR_Z + CAB_H + 0.10)
        ),
    )
    model.articulation(
        "frame_to_handle",
        ArticulationType.FIXED,
        parent=carcass,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # --- Upper shallow drawers (three PRISMATIC slide-out joints along +Y) ---
    face_centers = _drawer_face_centers()
    for i in range(N_DRAWERS):
        zc = face_centers[i]
        drawer = model.part(f"drawer_{i}")
        drawer.visual(_drawer_mesh(i), material=m_red, name=f"drawer_face_{i}")
        drawer.inertial = Inertial.from_geometry(
            Box((CAB_W - 2 * SIDE_REVEAL, CAB_D - 0.05, DRAWER_HEIGHTS[i])),
            mass=2.0,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
        )
        model.articulation(
            f"frame_to_drawer_{i}",
            ArticulationType.PRISMATIC,
            parent=carcass,
            child=drawer,
            origin=Origin(xyz=(0.0, 0.0, zc)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=60.0,
                velocity=0.30,
                lower=0.0,
                upper=DRAWER_TRAVEL,
            ),
        )

    # --- Cabinet door (REVOLUTE on a vertical Z side hinge, left edge) ---
    door_info = _door_opening()
    door_cz = door_info["center_z"]
    # Hinge line: left edge of the front opening, at the door's vertical center.
    hinge_x = -(CAB_W / 2.0 - SIDE_REVEAL)
    hinge_y = CAB_D / 2.0

    door = model.part("cabinet_door")
    door.visual(_cabinet_door_mesh(), material=m_red, name="door_panel")
    door.inertial = Inertial.from_geometry(
        Box((CAB_W - 2 * SIDE_REVEAL, DOOR_FACE_T, DOOR_H)),
        mass=3.5,
        origin=Origin(xyz=((CAB_W - 2 * SIDE_REVEAL) / 2.0, 0.0, 0.0)),
    )
    # The door part frame sits at the hinge line. The door panel extends along
    # local +X. Positive rotation about +Z swings the free edge (+X end) from
    # the front face (+Y) outward, opening the door.
    model.articulation(
        "frame_to_door",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, door_cz)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=0.0,
            upper=DOOR_OPEN_ANGLE,
        ),
    )

    # --- Four swivel casters (CONTINUOUS swivel about Z; wheel rolls about X) ---
    fork_mesh = _caster_fork_mesh()
    wheel_mesh = _caster_wheel_mesh()
    caster_positions = [
        (sx * (CAB_W / 2.0 - CASTER_INSET_X), sy * (CAB_D / 2.0 - CASTER_INSET_Y))
        for sx in (-1.0, 1.0)
        for sy in (-1.0, 1.0)
    ]
    for i, (cx, cy) in enumerate(caster_positions):
        fork = model.part(f"caster_fork_{i}")
        fork.visual(fork_mesh, material=m_dark, name=f"fork_{i}")
        fork.inertial = Inertial.from_geometry(
            Box((0.06, 0.06, CASTER_GAP)),
            mass=0.4,
            origin=Origin(xyz=(0.0, -CASTER_FORK_DROP / 2.0, -CASTER_GAP / 2.0)),
        )
        wheel = model.part(f"caster_wheel_{i}")
        wheel.visual(wheel_mesh, material=m_rubber, name=f"wheel_{i}")
        wheel.inertial = Inertial.from_geometry(
            Box((CASTER_WHEEL_W, 2 * CASTER_WHEEL_R, 2 * CASTER_WHEEL_R)),
            mass=0.5,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
        )
        model.articulation(
            f"frame_to_caster_{i}",
            ArticulationType.CONTINUOUS,
            parent=carcass,
            child=fork,
            origin=Origin(xyz=(cx, cy, FLOOR_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=6.0),
        )
        model.articulation(
            f"caster_to_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(xyz=(0.0, -CASTER_FORK_DROP, AXLE_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    carcass = object_model.get_part("cabinet_frame")
    handle = object_model.get_part("push_handle")
    drawers = [object_model.get_part(f"drawer_{i}") for i in range(N_DRAWERS)]
    drawer_joints = [
        object_model.get_articulation(f"frame_to_drawer_{i}") for i in range(N_DRAWERS)
    ]
    door = object_model.get_part("cabinet_door")
    door_joint = object_model.get_articulation("frame_to_door")
    forks = [object_model.get_part(f"caster_fork_{i}") for i in range(4)]
    wheels = [object_model.get_part(f"caster_wheel_{i}") for i in range(4)]
    swivel_joints = [
        object_model.get_articulation(f"frame_to_caster_{i}") for i in range(4)
    ]
    roll_joints = [
        object_model.get_articulation(f"caster_to_wheel_{i}") for i in range(4)
    ]

    # --- Carcass present, upright box, taller than wide, rests on casters. ---
    cab_aabb = ctx.part_world_aabb(carcass)
    ctx.check(
        "carcass present with world AABB",
        cab_aabb is not None,
        details=f"cab_aabb={cab_aabb}",
    )
    if cab_aabb is not None:
        (cmn, cmx) = cab_aabb
        ext_x = cmx[0] - cmn[0]
        ext_z = cmx[2] - cmn[2]
        ctx.check(
            "cabinet body is roughly cabinet-sized",
            0.45 < ext_x < 0.65 and 0.45 < ext_z < 0.75,
            details=f"ext_x={ext_x:.3f}, ext_z={ext_z:.3f}",
        )
        ctx.check(
            "cabinet floor stands above the caster gap",
            cmn[2] > CASTER_GAP - 0.02,
            details=f"carcass_z_min={cmn[2]:.4f}, caster_gap={CASTER_GAP}",
        )

    # --- Three shallow drawers authored, stacked, all shallow. ---
    ctx.check(
        "three upper drawers authored",
        len(drawers) == N_DRAWERS and N_DRAWERS == 3,
        details=f"n_drawers={len(drawers)}",
    )
    centers = _drawer_face_centers()
    ctx.check(
        "drawers stacked top -> bottom (descending z)",
        all(centers[i] > centers[i + 1] for i in range(N_DRAWERS - 1)),
        details=f"centers={['%.3f' % c for c in centers]}",
    )
    ctx.check(
        "all upper drawers are shallow (< 0.10 m)",
        all(h < 0.10 for h in DRAWER_HEIGHTS),
        details=f"heights={DRAWER_HEIGHTS}",
    )

    # --- Each drawer joint is prismatic, slides along +Y, positive travel. ---
    for i, dj in enumerate(drawer_joints):
        ctx.check(
            f"drawer {i} joint is prismatic",
            str(dj.articulation_type).upper().endswith("PRISMATIC"),
            details=f"type={dj.articulation_type}",
        )
        ax = dj.axis
        ctx.check(
            f"drawer {i} slides along +Y",
            abs(ax[1]) > 0.99 and abs(ax[0]) < 0.01 and abs(ax[2]) < 0.01,
            details=f"axis={ax}",
        )
        lim = dj.motion_limits
        ctx.check(
            f"drawer {i} has positive pull-out travel",
            lim is not None
            and lim.lower is not None
            and lim.upper is not None
            and lim.lower == 0.0
            and lim.upper > 0.20,
            details=f"lower={None if lim is None else lim.lower}, upper={None if lim is None else lim.upper}",
        )

    # --- Drawer faces sit within the carcass front footprint when closed, and
    # actually extend out past the front when opened. ---
    if cab_aabb is not None:
        cab_front_y = cab_aabb[1][1]
        test_i = 1  # test a middle shallow drawer
        dj = drawer_joints[test_i]
        with ctx.pose({dj: 0.0}):
            rest = ctx.part_world_aabb(drawers[test_i])
        with ctx.pose({dj: DRAWER_TRAVEL}):
            ext = ctx.part_world_aabb(drawers[test_i])
        ctx.check(
            "drawer AABBs resolve",
            rest is not None and ext is not None,
            details=f"rest={rest}, ext={ext}",
        )
        if rest is not None and ext is not None:
            rest_cy = 0.5 * (rest[0][1] + rest[1][1])
            ext_cy = 0.5 * (ext[0][1] + ext[1][1])
            ctx.check(
                "opening the drawer moves it out in +Y",
                ext_cy > rest_cy + 0.20,
                details=f"rest_cy={rest_cy:.3f}, ext_cy={ext_cy:.3f}",
            )
            ctx.check(
                "closed drawer face is flush with the cabinet front",
                abs(rest[1][1] - cab_front_y) < 0.02,
                details=f"drawer_front={rest[1][1]:.3f}, cab_front={cab_front_y:.3f}",
            )
            ctx.check(
                "opened drawer protrudes well past the cabinet front",
                ext[1][1] > cab_front_y + 0.20,
                details=f"open_front={ext[1][1]:.3f}, cab_front={cab_front_y:.3f}",
            )

    # Drawers slide inside openings with small clearances.
    for i, drawer in enumerate(drawers):
        ctx.allow_isolated_part(
            drawer,
            reason=(
                "Drawer slides on its prismatic joint inside the carcass opening; "
                "a small sliding clearance keeps the closed drawer from contacting "
                "the surrounding steel, but the joint provides the connection."
            ),
        )

    # --- Cabinet door: revolute joint on vertical Z axis, swings open. ---
    ctx.check(
        "cabinet door part exists",
        door is not None,
        details="door part not found",
    )
    ctx.check(
        "door joint is revolute",
        str(door_joint.articulation_type).upper().endswith("REVOLUTE"),
        details=f"type={door_joint.articulation_type}",
    )
    door_ax = door_joint.axis
    ctx.check(
        "door hinge axis is vertical Z",
        abs(door_ax[2]) > 0.99 and abs(door_ax[0]) < 0.01 and abs(door_ax[1]) < 0.01,
        details=f"axis={door_ax}",
    )
    door_lim = door_joint.motion_limits
    ctx.check(
        "door has realistic open limits (0 to ~1.5 rad)",
        door_lim is not None
        and door_lim.lower is not None
        and door_lim.upper is not None
        and door_lim.lower == 0.0
        and 1.0 < door_lim.upper < 2.5,
        details=f"lower={None if door_lim is None else door_lim.lower}, upper={None if door_lim is None else door_lim.upper}",
    )

    # Door hinge origin sits on the left edge of the front opening.
    door_info = _door_opening()
    hinge_x = -(CAB_W / 2.0 - SIDE_REVEAL)
    hinge_y = CAB_D / 2.0
    door_origin = door_joint.origin
    ctx.check(
        "door hinge origin is at the left front edge",
        door_origin is not None
        and abs(door_origin.xyz[0] - hinge_x) < 0.02
        and abs(door_origin.xyz[1] - hinge_y) < 0.02,
        details=f"origin_xyz={None if door_origin is None else door_origin.xyz}, expected=({hinge_x:.3f}, {hinge_y:.3f})",
    )
    ctx.check(
        "door hinge origin is at the door vertical center",
        door_origin is not None
        and abs(door_origin.xyz[2] - door_info["center_z"]) < 0.02,
        details=f"origin_z={None if door_origin is None else door_origin.xyz[2]:.3f}, expected={door_info['center_z']:.3f}",
    )

    # Opening the door swings the free edge outward (in +Y, then around).
    if cab_aabb is not None:
        cab_front_y = cab_aabb[1][1]
        with ctx.pose({door_joint: 0.0}):
            door_closed = ctx.part_world_aabb(door)
        with ctx.pose({door_joint: DOOR_OPEN_ANGLE}):
            door_open = ctx.part_world_aabb(door)
        ctx.check(
            "door AABBs resolve at closed and open poses",
            door_closed is not None and door_open is not None,
            details=f"closed={door_closed}, open={door_open}",
        )
        if door_closed is not None and door_open is not None:
            # When closed, the door face is flush with the cabinet front.
            ctx.check(
                "closed door face is near the cabinet front",
                abs(door_closed[1][1] - cab_front_y) < 0.025,
                details=f"door_front={door_closed[1][1]:.3f}, cab_front={cab_front_y:.3f}",
            )
            # When open, the door extends well past the cabinet front on one
            # side (the free edge swings out).
            ctx.check(
                "open door free edge swings out past the cabinet side",
                door_open[0][0] < door_closed[0][0] - 0.10
                or door_open[1][1] > cab_front_y + 0.10,
                details=f"closed_x_min={door_closed[0][0]:.3f}, open_x_min={door_open[0][0]:.3f}, open_y_max={door_open[1][1]:.3f}",
            )

    # The door hinge barrels seat into the carcass front-left edge; allow the
    # small local overlap that represents the hinge pin capture.
    ctx.allow_overlap(
        carcass,
        door,
        reason=(
            "The cabinet door hinge barrel stubs are intentionally seated at the "
            "carcass front-left edge; the small local overlap represents the "
            "captured hinge pin that mounts the door to the cabinet."
        ),
    )

    # --- Push handle present, fixed, rises above the cabinet and reaches back. ---
    h_aabb = ctx.part_world_aabb(handle)
    ctx.check(
        "push handle present",
        h_aabb is not None,
        details=f"handle_aabb={h_aabb}",
    )
    if h_aabb is not None and cab_aabb is not None:
        ctx.check(
            "handle rises above the cabinet top",
            h_aabb[1][2] > cab_aabb[1][2] + 0.08,
            details=f"handle_top={h_aabb[1][2]:.3f}, cab_top={cab_aabb[1][2]:.3f}",
        )
        ctx.check(
            "handle grip reaches back past the cabinet rear",
            h_aabb[0][1] < cab_aabb[0][1] - 0.05,
            details=f"handle_back_y={h_aabb[0][1]:.3f}, cab_back_y={cab_aabb[0][1]:.3f}",
        )
    handle_joint = object_model.get_articulation("frame_to_handle")
    ctx.check(
        "handle is rigidly fixed to the cabinet",
        str(handle_joint.articulation_type).upper().endswith("FIXED"),
        details=f"type={handle_joint.articulation_type}",
    )
    ctx.allow_overlap(
        carcass,
        handle,
        reason=(
            "The push-handle upright mounting bosses are intentionally seated "
            "into the cabinet top trim band; the small local overlap represents "
            "the bolted handle mount."
        ),
    )

    # --- Four casters: swivel about Z, wheel rolls about X, sit at ground z~=0. ---
    ctx.check(
        "four casters authored",
        len(forks) == 4 and len(wheels) == 4,
        details=f"forks={len(forks)}, wheels={len(wheels)}",
    )
    for i, sj in enumerate(swivel_joints):
        sax = sj.axis
        ctx.check(
            f"caster {i} swivels about vertical Z",
            abs(sax[2]) > 0.99 and abs(sax[0]) < 0.01 and abs(sax[1]) < 0.01,
            details=f"axis={sax}",
        )
        ctx.check(
            f"caster {i} swivel is continuous",
            str(sj.articulation_type).upper().endswith("CONTINUOUS"),
            details=f"type={sj.articulation_type}",
        )
    for i, rj in enumerate(roll_joints):
        rax = rj.axis
        ctx.check(
            f"caster {i} wheel rolls about its X axle",
            abs(rax[0]) > 0.99 and abs(rax[1]) < 0.01 and abs(rax[2]) < 0.01,
            details=f"axis={rax}",
        )

    for i, wheel in enumerate(wheels):
        w_aabb = ctx.part_world_aabb(wheel)
        ctx.check(
            f"caster wheel {i} reaches the ground plane",
            w_aabb is not None and abs(w_aabb[0][2]) < 0.02,
            details=f"wheel_z_min={None if w_aabb is None else w_aabb[0][2]}",
        )

    sj0 = swivel_joints[0]
    with ctx.pose({sj0: 0.0}):
        w0 = ctx.part_world_position(wheels[0])
    with ctx.pose({sj0: math.pi}):
        w0b = ctx.part_world_position(wheels[0])
    ctx.check(
        "swiveling a caster relocates its trailing wheel",
        w0 is not None
        and w0b is not None
        and math.hypot(w0b[0] - w0[0], w0b[1] - w0[1]) > 0.03,
        details=f"wheel0={w0}, wheel0_180={w0b}",
    )

    for i in range(4):
        ctx.allow_overlap(
            forks[i],
            wheels[i],
            reason=(
                "The caster wheel is captured in the trailing yoke of the fork "
                "on the axle; the small local overlap at the axle represents the "
                "real pin-in-yoke rolling joint."
            ),
        )
        ctx.allow_overlap(
            carcass,
            forks[i],
            reason=(
                "The caster swivel post is intentionally seated up into the "
                "cabinet floor; the small local overlap represents the captured "
                "swivel bearing that mounts the caster to the cart."
            ),
        )

    return ctx.report()


object_model = build_object_model()
