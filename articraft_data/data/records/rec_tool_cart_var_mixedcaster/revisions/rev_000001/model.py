from __future__ import annotations

# Rolling tool cart / roller cabinet, modeled faithfully from the reference
# image (picture/Handtools/Tool cart/001.png).
#
# The object is a red-and-black mobile tool chest:
#   - A black steel carcass (root) standing on two front swivel casters
#     and two rear fixed casters.
#   - A stack of five red drawers in the front face: three shallow drawers up
#     top and two deep drawers at the bottom. Each drawer pulls straight out.
#   - A red perforated pegboard panel on the left side wall.
#   - A black top tray with a molded socket recess and a raised lip.
#   - A dark tubular push handle with a red rubber grip on the top-right rear.
#
# Coordinate convention: +Z up, +Y toward the front (the side the drawers and
# handle grip face), +X to the right. The carcass floor sits above the caster
# height so the whole cart rests on the four wheels at z = 0.
#
# Primary mechanism: the FIVE DRAWERS each slide out horizontally on a
# PRISMATIC joint along +Y (one open in the reference image). Secondary
# mechanisms: two FRONT swivel casters SWIVEL about a vertical Z axis
# (CONTINUOUS) with rolling wheels; two REAR fixed casters with brackets
# bolted rigidly to the frame (FIXED) and wheels that only roll about
# their X axle (CONTINUOUS).

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

# Drawer stack: five drawers (3 shallow, 2 deep), from top to bottom.
# Heights chosen to read like the reference (shallow trio then deep pair).
DRAWER_FACE_T = 0.018  # red drawer-front panel thickness (Y)
DRAWER_GAP = 0.006  # reveal gap between adjacent drawer faces
SIDE_REVEAL = 0.014  # gap from carcass side wall to drawer face edge
# (h0..h4) drawer FACE heights, top -> bottom
DRAWER_HEIGHTS = (0.072, 0.072, 0.072, 0.150, 0.150)

DRAWER_TRAVEL = 0.300  # pull-out travel along +Y
DRAWER_CLEAR = 0.004  # sliding clearance per side inside the opening

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


# ---------------------------------------------------------------------------
# Geometry builders
# ---------------------------------------------------------------------------
def _carcass_mesh() -> object:
    """Black steel cabinet carcass: a hollow box open at the front (+Y) so the
    drawers slot in, with a top trim band, a recessed top tray, and the left
    side pegboard fused in. Sits above the caster gap."""
    z0 = FLOOR_Z
    z1 = FLOOR_Z + CAB_H

    # Build the carcass as a SINGLE solid via explicit boolean cuts (robust,
    # always one fused solid) rather than a thin shell. Start with a solid box
    # and carve out the interior cavity and the open front.
    body = (
        cq.Workplane("XY", origin=(0, 0, (z0 + z1) / 2.0))
        .box(CAB_W, CAB_D, CAB_H)
    )
    # Soften the visible outer vertical edges.
    try:
        body = body.edges("|Z").fillet(0.010)
    except Exception:
        pass

    # Interior cavity: leave WALL-thick side/back/top walls and a solid BASE_BAND
    # toe-kick at the bottom. The cavity is open at the front (+Y) so the cut box
    # protrudes out the front face, carving the front opening too.
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

    # --- Top trim band: a slightly oversized black cap on the carcass top.
    # It extends a few mm DOWN past the shell top so the union truly fuses. ---
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

    # --- Top tray: a shallow molded recess with a raised perimeter lip on top
    # of the trim, like the molded tool tray in the reference. It extends DOWN
    # into the trim so the union fuses into one solid. ---
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
    # Hollow the tray basin (open top).
    basin = (
        cq.Workplane("XY", origin=(0, 0, tray_top + TRAY_LIP))
        .box(CAB_W - 0.040, CAB_D - 0.040, 2 * (TRAY_LIP - TRAY_WALL))
    )
    tray_outer = tray_outer.cut(basin)
    # A couple of molded socket pockets in the tray floor for hand tools.
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
    """Red perforated pegboard panel mounted flat on the left (-X) side wall.
    Authored in the carcass frame so it reads as bolted onto the side."""
    x_face = -CAB_W / 2.0 - PEG_T / 2.0 + 0.004  # overlaps the side wall to bond
    z_c = FLOOR_Z + CAB_H * 0.55
    plate = (
        cq.Workplane("YZ", origin=(x_face, 0.0, z_c))
        .box(PEG_W, PEG_H, PEG_T)
    )
    # Round-hole perforation grid.
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
    """One drawer: a red front face panel plus a black box that slides into the
    carcass. The drawer is authored centered at the local origin so the
    prismatic joint frame sits at the drawer's resting center; the front face
    is on +Y, the box body extends back into -Y."""
    h = DRAWER_HEIGHTS[index]
    face_w = CAB_W - 2 * SIDE_REVEAL
    face_h = h
    box_w = face_w - 2 * DRAWER_CLEAR
    box_h = h - 2 * DRAWER_CLEAR
    box_depth = CAB_D - WALL - 0.030  # drawer box length into the cabinet (Y)

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

    # Recessed finger pull along the top of the face (a horizontal groove).
    pull = (
        cq.Workplane("XY", origin=(0, face_y + 0.002, face_h / 2.0 - 0.012))
        .box(face_w * 0.62, DRAWER_FACE_T, 0.016)
    )
    face = face.cut(pull)

    # Drawer box (black): open-top tray-like body behind the face.
    box_y_c = CAB_D / 2.0 - DRAWER_FACE_T - box_depth / 2.0
    box_outer = (
        cq.Workplane("XY", origin=(0, box_y_c, 0))
        .box(box_w, box_depth, box_h)
    )
    # Hollow the box (open top), leaving a thin floor and side walls.
    box = box_outer.faces(">Z").shell(-0.005)

    drawer = face.union(box)
    return mesh_from_cadquery(drawer, f"drawer_{index}")


# Handle geometry constants (carcass frame).
HANDLE_TRIM_TOP = FLOOR_Z + CAB_H + TOP_TRIM_H  # top of the black trim band
HANDLE_Z_BASE = HANDLE_TRIM_TOP + TRAY_LIP + 0.004  # start above the tray lip
HANDLE_Y_ANCHOR = -CAB_D / 2.0 + 0.020  # uprights at the rear rim (on solid trim)
HANDLE_X_OFF = CAB_W / 2.0 - 0.065  # loop toward the right side (as in image)
HANDLE_X_IN = HANDLE_X_OFF - 0.170  # loop width
HANDLE_Z_TOP = HANDLE_Z_BASE + HANDLE_RISE
HANDLE_Y_GRIP = -CAB_D / 2.0 - HANDLE_REACH  # grip reaches back past the cabinet


def _handle_mesh() -> object:
    """Dark tubular push handle: an inverted-U loop rising from the top-rear of
    the carcass and reaching back, with a red rubber grip sleeve over the top
    run. Authored in the carcass frame. Returns (frame_tube_mesh, grip_mesh).

    The two uprights start above the tray lip on the rear trim rim and drop
    small mounting bosses down into the trim band for connection."""
    z_base = HANDLE_Z_BASE
    y_back = HANDLE_Y_ANCHOR
    x_off = HANDLE_X_OFF
    x_in = HANDLE_X_IN
    z_top = HANDLE_Z_TOP
    y_grip = HANDLE_Y_GRIP

    # Single swept tube path: down-mount on the right, up, back and up to the
    # grip line, across to the left, then forward and down to the left mount.
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

    # Mounting bosses: short stubs from each upright base down into the trim.
    for sx in (x_off, x_in):
        boss = (
            cq.Workplane("XY", origin=(sx, y_back, z_base - 0.014))
            .circle(HANDLE_TUBE_R + 0.004)
            .extrude(0.028)
        )
        tube = tube.union(boss)

    frame_mesh = mesh_from_cadquery(tube, "handle_frame")

    # Red grip sleeve over the top horizontal run (from x_in..x_off at y_grip).
    grip = (
        cq.Workplane("YZ", origin=((x_in + x_off) / 2.0, y_grip, z_top))
        .circle(GRIP_R)
        .extrude(GRIP_LEN / 2.0, both=True)
    )
    grip_mesh = mesh_from_cadquery(grip, "handle_grip")
    return frame_mesh, grip_mesh


AXLE_Z = -CASTER_GAP + CASTER_WHEEL_R  # axle height in the fork-local frame


def _fixed_bracket_mesh() -> object:
    """Fixed (non-swivel) caster bracket: a rigid mounting plate with a U-yoke
    that captures the wheel axle directly. No swivel plate or post — this bracket
    bolts flat to the cabinet underside and the wheel rolls on a fixed axle.
    Authored in a LOCAL frame with the mounting plate top at z=0 (seats against
    the cabinet floor). The axle sits directly below at AXLE_Z (no trailing
    offset since there is no steering action)."""
    # Mounting plate: top face flush at z=0, body just below.
    plate = (
        cq.Workplane("XY", origin=(0, 0, -0.007))
        .box(0.060, 0.050, 0.014)
    )

    # Short vertical riser from the plate down toward the axle region.
    riser_top = 0.0
    riser_bot = AXLE_Z + 0.022
    riser = (
        cq.Workplane("XY", origin=(0, 0, (riser_top + riser_bot) / 2.0))
        .box(0.052, 0.040, abs(riser_top - riser_bot))
    )
    bracket = plate.union(riser)

    # U-yoke: two legs straddling the wheel, overlapping the riser so the whole
    # bracket fuses into a single solid.
    leg_top = AXLE_Z + 0.022
    leg_bot = AXLE_Z - 0.014
    for sx in (-1.0, 1.0):
        leg = (
            cq.Workplane(
                "XY",
                origin=(
                    sx * (CASTER_WHEEL_W / 2.0 + 0.006),
                    0.0,
                    (leg_top + leg_bot) / 2.0,
                ),
            )
            .box(0.014, 0.036, abs(leg_top - leg_bot))
        )
        bracket = bracket.union(leg)

    return mesh_from_cadquery(bracket, "fixed_bracket")


def _caster_fork_mesh() -> object:
    """One swivel-caster yoke authored in a LOCAL frame whose swivel axis is
    local +Z through the origin. The bracket is a single connected solid:
    a mounting plate with its TOP face at z=0 (which seats up against the
    cabinet floor), a short swivel post that pokes up past z=0 to bite into
    the floor, a vertical riser, and a U-yoke straddling the wheel. The axle
    seat trails the swivel axis by CASTER_FORK_DROP in local -Y, so steering
    the swivel sweeps the trailing wheel around like a real caster."""
    leg_y = -CASTER_FORK_DROP  # axle trails the swivel axis in -Y

    # Mounting plate: top face flush at z=0, body just below it.
    plate = (
        cq.Workplane("XY", origin=(0, 0, -0.007))
        .box(0.060, 0.060, 0.014)
    )
    # Swivel post: a stub that rises well ABOVE z=0 so it bites into the cabinet
    # floor lid (mechanical capture of the swivel) and ties the plate together.
    post = (
        cq.Workplane("XY", origin=(0, 0, 0.0))
        .circle(0.013)
        .extrude(0.030)  # up into the floor lid
    )
    post = post.union(
        cq.Workplane("XY", origin=(0, 0, -0.018)).circle(0.013).extrude(0.018)
    )
    fork = plate.union(post)

    # Riser: a solid block running from INSIDE the plate (overlapping it) down to
    # just below the axle, offset toward the trailing axle so the bracket leans
    # back like a caster. Overlapping the plate guarantees one fused solid.
    riser_top = 0.0  # overlaps up into the plate body
    riser_bot = AXLE_Z - 0.006  # reaches down past the axle to join the legs
    fork = fork.union(
        cq.Workplane(
            "XY",
            origin=(0.0, leg_y * 0.45, (riser_top + riser_bot) / 2.0),
        )
        .box(0.052, abs(leg_y) + 0.030, abs(riser_top - riser_bot))
    )

    # U-yoke: two legs straddling the wheel, overlapping the wide riser block so
    # the whole bracket fuses into a single solid.
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
    """Wheel authored centered on its OWN part origin (the axle). The part
    frame is the axle; the wheel rolls about local X. The wheel sits in the
    fork frame trailing in -Y, so the wheel's roll joint origin carries that
    offset (the mesh itself is centered at the origin)."""
    wheel = (
        cq.Workplane("YZ", origin=(0, 0, 0))
        .circle(CASTER_WHEEL_R)
        .extrude(CASTER_WHEEL_W / 2.0, both=True)
    )
    # Recessed sidewall rings so it reads as a tire on a hub, not a plain disc.
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

    # --- Drawers (five PRISMATIC slide-out joints along +Y) ---
    face_centers = _drawer_face_centers()
    for i, zc in enumerate(face_centers):
        drawer = model.part(f"drawer_{i}")
        drawer.visual(_drawer_mesh(i), material=m_red, name=f"drawer_face_{i}")
        drawer.inertial = Inertial.from_geometry(
            Box((CAB_W - 2 * SIDE_REVEAL, CAB_D - 0.05, DRAWER_HEIGHTS[i])),
            mass=2.0 + 2.5 * (DRAWER_HEIGHTS[i] > 0.10),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
        )
        # Joint frame at the resting drawer center; +Y pulls the drawer out.
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

    # --- Two front swivel casters (CONTINUOUS swivel about Z; wheel rolls about X) ---
    fork_mesh = _caster_fork_mesh()
    wheel_mesh = _caster_wheel_mesh()
    bracket_mesh = _fixed_bracket_mesh()

    front_caster_xs = [
        sx * (CAB_W / 2.0 - CASTER_INSET_X) for sx in (-1.0, 1.0)
    ]
    front_cy = CAB_D / 2.0 - CASTER_INSET_Y
    for i in range(2):
        cx = front_caster_xs[i]
        fork = model.part(f"swivel_fork_{i}")
        fork.visual(fork_mesh, material=m_dark, name=f"fork_{i}")
        fork.inertial = Inertial.from_geometry(
            Box((0.06, 0.06, CASTER_GAP)),
            mass=0.4,
            origin=Origin(xyz=(0.0, -CASTER_FORK_DROP / 2.0, -CASTER_GAP / 2.0)),
        )
        wheel = model.part(f"swivel_wheel_{i}")
        wheel.visual(wheel_mesh, material=m_rubber, name=f"wheel_{i}")
        wheel.inertial = Inertial.from_geometry(
            Box((CASTER_WHEEL_W, 2 * CASTER_WHEEL_R, 2 * CASTER_WHEEL_R)),
            mass=0.5,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
        )
        # Swivel: caster steers about a vertical axis at the mounting point
        # under the cabinet floor. The fork plate seats at z=FLOOR_Z and its
        # post bites up into the floor for capture.
        model.articulation(
            f"frame_to_swivel_{i}",
            ArticulationType.CONTINUOUS,
            parent=carcass,
            child=fork,
            origin=Origin(xyz=(cx, front_cy, FLOOR_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=6.0),
        )
        # Wheel rolls about its local axle (X). The joint origin places the
        # wheel at the trailing axle seat in the fork frame.
        model.articulation(
            f"swivel_to_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(xyz=(0.0, -CASTER_FORK_DROP, AXLE_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=20.0),
        )

    # --- Two rear fixed casters (FIXED bracket to frame; wheel rolls about X) ---
    rear_cy = -(CAB_D / 2.0 - CASTER_INSET_Y)
    for i in range(2):
        cx = front_caster_xs[i]
        bracket = model.part(f"fixed_bracket_{i}")
        bracket.visual(bracket_mesh, material=m_dark, name=f"bracket_{i}")
        bracket.inertial = Inertial.from_geometry(
            Box((0.06, 0.05, CASTER_GAP)),
            mass=0.3,
            origin=Origin(xyz=(0.0, 0.0, -CASTER_GAP / 2.0)),
        )
        wheel = model.part(f"fixed_wheel_{i}")
        wheel.visual(wheel_mesh, material=m_rubber, name=f"wheel_{i}")
        wheel.inertial = Inertial.from_geometry(
            Box((CASTER_WHEEL_W, 2 * CASTER_WHEEL_R, 2 * CASTER_WHEEL_R)),
            mass=0.5,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
        )
        # Fixed bracket: bolted rigidly to the cabinet underside, no swivel.
        model.articulation(
            f"frame_to_bracket_{i}",
            ArticulationType.FIXED,
            parent=carcass,
            child=bracket,
            origin=Origin(xyz=(cx, rear_cy, FLOOR_Z)),
        )
        # Wheel rolls about its local axle (X). Axle is directly below the
        # bracket mount (no trailing offset since there is no steering).
        model.articulation(
            f"bracket_to_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=bracket,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.0, AXLE_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    carcass = object_model.get_part("cabinet_frame")
    handle = object_model.get_part("push_handle")
    drawers = [object_model.get_part(f"drawer_{i}") for i in range(5)]
    drawer_joints = [
        object_model.get_articulation(f"frame_to_drawer_{i}") for i in range(5)
    ]

    # Front swivel casters
    swivel_forks = [object_model.get_part(f"swivel_fork_{i}") for i in range(2)]
    swivel_wheels = [object_model.get_part(f"swivel_wheel_{i}") for i in range(2)]
    swivel_joints = [
        object_model.get_articulation(f"frame_to_swivel_{i}") for i in range(2)
    ]
    swivel_roll_joints = [
        object_model.get_articulation(f"swivel_to_wheel_{i}") for i in range(2)
    ]

    # Rear fixed casters
    fixed_brackets = [object_model.get_part(f"fixed_bracket_{i}") for i in range(2)]
    fixed_wheels = [object_model.get_part(f"fixed_wheel_{i}") for i in range(2)]
    fixed_mount_joints = [
        object_model.get_articulation(f"frame_to_bracket_{i}") for i in range(2)
    ]
    fixed_roll_joints = [
        object_model.get_articulation(f"bracket_to_wheel_{i}") for i in range(2)
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

    # --- Five drawers exist, stacked, with the deep pair below the shallow trio. ---
    ctx.check(
        "five drawers authored",
        len(drawers) == 5,
        details=f"n_drawers={len(drawers)}",
    )
    centers = _drawer_face_centers()
    ctx.check(
        "drawers stacked top -> bottom (descending z)",
        all(centers[i] > centers[i + 1] for i in range(4)),
        details=f"centers={['%.3f' % c for c in centers]}",
    )
    ctx.check(
        "bottom two drawers are the deep drawers",
        DRAWER_HEIGHTS[3] > 0.12 and DRAWER_HEIGHTS[4] > 0.12
        and DRAWER_HEIGHTS[0] < 0.10,
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
        test_i = 2
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

    # Drawers slide inside openings with small clearances; the prismatic joint
    # provides the mechanical connection.
    for i, drawer in enumerate(drawers):
        ctx.allow_isolated_part(
            drawer,
            reason=(
                "Drawer slides on its prismatic joint inside the carcass opening; "
                "a small sliding clearance keeps the closed drawer from contacting "
                "the surrounding steel, but the joint provides the connection."
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

    # --- Front swivel casters: swivel about Z (CONTINUOUS), wheel rolls about X ---
    ctx.check(
        "two front swivel casters authored",
        len(swivel_forks) == 2 and len(swivel_wheels) == 2,
        details=f"forks={len(swivel_forks)}, wheels={len(swivel_wheels)}",
    )
    for i, sj in enumerate(swivel_joints):
        sax = sj.axis
        ctx.check(
            f"front caster {i} swivels about vertical Z",
            abs(sax[2]) > 0.99 and abs(sax[0]) < 0.01 and abs(sax[1]) < 0.01,
            details=f"axis={sax}",
        )
        ctx.check(
            f"front caster {i} swivel is continuous",
            str(sj.articulation_type).upper().endswith("CONTINUOUS"),
            details=f"type={sj.articulation_type}",
        )
    for i, rj in enumerate(swivel_roll_joints):
        rax = rj.axis
        ctx.check(
            f"front caster {i} wheel rolls about its X axle",
            abs(rax[0]) > 0.99 and abs(rax[1]) < 0.01 and abs(rax[2]) < 0.01,
            details=f"axis={rax}",
        )

    # --- Rear fixed casters: FIXED bracket to frame, wheel rolls about X ---
    ctx.check(
        "two rear fixed casters authored",
        len(fixed_brackets) == 2 and len(fixed_wheels) == 2,
        details=f"brackets={len(fixed_brackets)}, wheels={len(fixed_wheels)}",
    )
    for i, fj in enumerate(fixed_mount_joints):
        ctx.check(
            f"rear caster {i} bracket is FIXED (no swivel)",
            str(fj.articulation_type).upper().endswith("FIXED"),
            details=f"type={fj.articulation_type}",
        )
    for i, rj in enumerate(fixed_roll_joints):
        rax = rj.axis
        ctx.check(
            f"rear caster {i} wheel rolls about its X axle",
            abs(rax[0]) > 0.99 and abs(rax[1]) < 0.01 and abs(rax[2]) < 0.01,
            details=f"axis={rax}",
        )
        ctx.check(
            f"rear caster {i} wheel roll is continuous",
            str(rj.articulation_type).upper().endswith("CONTINUOUS"),
            details=f"type={rj.articulation_type}",
        )

    # All four wheels touch the ground plane (z ~= 0).
    all_wheels = swivel_wheels + fixed_wheels
    for i, wheel in enumerate(all_wheels):
        w_aabb = ctx.part_world_aabb(wheel)
        ctx.check(
            f"wheel {i} reaches the ground plane",
            w_aabb is not None and abs(w_aabb[0][2]) < 0.02,
            details=f"wheel_z_min={None if w_aabb is None else w_aabb[0][2]}",
        )

    # Rear fixed brackets sit at the back of the cabinet (-Y side).
    if cab_aabb is not None:
        cab_center_y = 0.5 * (cab_aabb[0][1] + cab_aabb[1][1])
        for i, bracket in enumerate(fixed_brackets):
            b_aabb = ctx.part_world_aabb(bracket)
            ctx.check(
                f"rear bracket {i} is behind cabinet center",
                b_aabb is not None
                and 0.5 * (b_aabb[0][1] + b_aabb[1][1]) < cab_center_y,
                details=f"bracket_aabb={b_aabb}, cab_center_y={cab_center_y:.3f}",
            )

    # Front swivel casters: swiveling sweeps the trailing wheel around.
    sj0 = swivel_joints[0]
    with ctx.pose({sj0: 0.0}):
        w0 = ctx.part_world_position(swivel_wheels[0])
    with ctx.pose({sj0: math.pi}):
        w0b = ctx.part_world_position(swivel_wheels[0])
    ctx.check(
        "swiveling a front caster relocates its trailing wheel",
        w0 is not None
        and w0b is not None
        and math.hypot(w0b[0] - w0[0], w0b[1] - w0[1]) > 0.03,
        details=f"wheel0={w0}, wheel0_180={w0b}",
    )

    # Rear fixed wheels: rolling does NOT relocate the wheel center (fixed axle).
    rj0 = fixed_roll_joints[0]
    with ctx.pose({rj0: 0.0}):
        rw0 = ctx.part_world_position(fixed_wheels[0])
    with ctx.pose({rj0: math.pi}):
        rw0b = ctx.part_world_position(fixed_wheels[0])
    ctx.check(
        "rolling a rear fixed wheel does not relocate its center",
        rw0 is not None
        and rw0b is not None
        and math.hypot(rw0b[0] - rw0[0], rw0b[1] - rw0[1]) < 0.005,
        details=f"rest={rw0}, rolled={rw0b}",
    )

    # Overlap allowances: wheel captured in fork/bracket yoke at the axle.
    for i in range(2):
        ctx.allow_overlap(
            swivel_forks[i],
            swivel_wheels[i],
            reason=(
                "The swivel caster wheel is captured in the trailing yoke of the "
                "fork on the axle; the small local overlap at the axle represents "
                "the real pin-in-yoke rolling joint."
            ),
        )
        ctx.allow_overlap(
            carcass,
            swivel_forks[i],
            reason=(
                "The swivel caster post is intentionally seated up into the "
                "cabinet floor; the small local overlap represents the captured "
                "swivel bearing that mounts the caster to the cart."
            ),
        )
    for i in range(2):
        ctx.allow_overlap(
            fixed_brackets[i],
            fixed_wheels[i],
            reason=(
                "The fixed caster wheel is captured in the bracket yoke on the "
                "axle; the small local overlap at the axle represents the real "
                "pin-in-yoke rolling joint."
            ),
        )

    return ctx.report()


object_model = build_object_model()
