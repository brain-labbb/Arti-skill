from __future__ import annotations

# Rolling tool cart / roller cabinet, modeled faithfully from the reference
# image (picture/Handtools/Tool cart/001.png).
#
# The object is a red-and-black mobile tool chest:
#   - A black steel carcass (root) standing on four black swivel casters.
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
# mechanism: the four casters SWIVEL about a vertical Z axis (CONTINUOUS),
# which is the visible steering action of a real swivel caster.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_cadquery,
    mesh_from_geometry,
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
CASTER_GAP = 0.190  # vertical room the larger pneumatic casters/swivel forks occupy
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

# Casters (pneumatic-style swivel casters with side brake levers)
CASTER_WHEEL_R = 0.076  # 150 mm pneumatic tire outer radius
CASTER_WHEEL_W = 0.050  # tire width
CASTER_RIM_R = 0.050  # metal rim radius inside the tire
CASTER_HUB_R = 0.020  # hub bore radius
CASTER_FORK_DROP = 0.036  # swivel offset: wheel axle trails behind swivel axis
CASTER_INSET_X = 0.080  # caster center inset from the cabinet side
CASTER_INSET_Y = 0.070  # caster center inset from the cabinet front/back
# Brake lever (mounted on +X side of each fork yoke leg)
BRAKE_LEVER_LEN = 0.065  # lever arm length from pivot
BRAKE_LEVER_W = 0.020  # lever width (along X, into the fork side)
BRAKE_LEVER_T = 0.006  # lever thickness
BRAKE_PIVOT_R = 0.006  # pivot boss radius
BRAKE_ENGAGE_ANGLE = 0.55  # radians: engaged brake angle from rest

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


def _caster_fork_mesh() -> object:
    """Pneumatic swivel-caster yoke: a heavy-duty fork sized for the larger
    tire. Authored in a LOCAL frame whose swivel axis is local +Z through the
    origin. The bracket is a single connected solid: a mounting plate with its
    TOP face at z=0 (seats against the cabinet floor), a swivel post poking up
    past z=0 for capture, a vertical riser, and a U-yoke straddling the
    pneumatic tire. A pivot boss on the +X yoke leg provides the brake lever
    mount. The axle seat trails the swivel axis by CASTER_FORK_DROP in -Y."""
    leg_y = -CASTER_FORK_DROP  # axle trails the swivel axis in -Y
    half_w = CASTER_WHEEL_W / 2.0 + 0.008  # clearance each side of tire

    # Mounting plate: top face flush at z=0, body below.
    plate = (
        cq.Workplane("XY", origin=(0, 0, -0.008))
        .box(0.072, 0.072, 0.016)
    )
    # Swivel post: rises above z=0 to bite into the cabinet floor.
    post = (
        cq.Workplane("XY", origin=(0, 0, 0.0))
        .circle(0.015)
        .extrude(0.030)
    )
    post = post.union(
        cq.Workplane("XY", origin=(0, 0, -0.020)).circle(0.015).extrude(0.020)
    )
    fork = plate.union(post)

    # Riser: a solid block from inside the plate down past the axle, offset
    # toward the trailing axle. Overlapping the plate guarantees one fused solid.
    riser_top = 0.0
    riser_bot = AXLE_Z - 0.008
    fork = fork.union(
        cq.Workplane(
            "XY",
            origin=(0.0, leg_y * 0.45, (riser_top + riser_bot) / 2.0),
        )
        .box(0.058, abs(leg_y) + 0.036, abs(riser_top - riser_bot))
    )

    # U-yoke: two legs straddling the wider pneumatic tire.
    leg_top = AXLE_Z + CASTER_WHEEL_R * 0.55
    leg_bot = AXLE_Z - 0.016
    for sx in (-1.0, 1.0):
        leg = (
            cq.Workplane(
                "XY",
                origin=(
                    sx * half_w,
                    leg_y,
                    (leg_top + leg_bot) / 2.0,
                ),
            )
            .box(0.016, 0.040, abs(leg_top - leg_bot))
        )
        fork = fork.union(leg)

    # Brake-lever pivot boss on the +X leg: a cylindrical boss protruding
    # outward from the leg. The brake lever part pivots on this boss.
    boss_z = AXLE_Z + CASTER_WHEEL_R * 0.40  # above axle on the leg
    boss = (
        cq.Workplane("YZ", origin=(half_w + 0.008, leg_y, boss_z))
        .circle(BRAKE_PIVOT_R)
        .extrude(0.012)
    )
    fork = fork.union(boss)

    return mesh_from_cadquery(fork, "caster_fork")


def _caster_wheel_meshes() -> tuple[object, object]:
    """Pneumatic wheel: a metal rim with hub and spoke disc, plus a rubber
    tire with rounded sidewall and block tread. Both authored centered on
    their part origin (the axle), spinning about local X.
    Returns (rim_mesh, tire_mesh)."""
    half_w = CASTER_WHEEL_W * 0.44

    # Metal rim: hub disc + connecting web + outer rim ring, all one fused solid.
    # Hub cylinder
    hub = (
        cq.Workplane("YZ", origin=(0, 0, 0))
        .circle(CASTER_HUB_R + 0.004)
        .extrude(half_w, both=True)
    )
    # Connecting web disc (solid face between hub and outer rim)
    web = (
        cq.Workplane("YZ", origin=(0, 0, 0))
        .circle(CASTER_RIM_R - 0.004)
        .extrude(half_w * 0.6, both=True)
    )
    # Outer rim ring
    rim_outer = (
        cq.Workplane("YZ", origin=(0, 0, 0))
        .circle(CASTER_RIM_R)
        .extrude(half_w, both=True)
    )
    rim_inner = (
        cq.Workplane("YZ", origin=(0, 0, 0))
        .circle(CASTER_RIM_R - 0.006)
        .extrude(half_w + 0.002, both=True)
    )
    rim_ring = rim_outer.cut(rim_inner)
    rim_solid = hub.union(web).union(rim_ring)

    # Cut spoke windows: 5 radial slots between hub and rim
    n_spokes = 5
    spoke_gap_r = 0.012  # inner radius of window
    spoke_outer_r = CASTER_RIM_R - 0.008
    for k in range(n_spokes):
        angle = 2 * math.pi * k / n_spokes
        mid_r = 0.5 * (spoke_gap_r + spoke_outer_r)
        slot_len = spoke_outer_r - spoke_gap_r
        # A thin slot in the YZ plane at the spoke angle
        cy = mid_r * math.cos(angle)
        cz = mid_r * math.sin(angle)
        slot = (
            cq.Workplane("YZ", origin=(0, cy, cz))
            .box(slot_len * 0.35, slot_len, half_w * 0.5 * 2.0)
            .rotateAboutCenter((1, 0, 0), angle)
        )
        # Rotate the slot box around X axis to align with the spoke angle
        slot = (
            cq.Workplane("XY", origin=(0, 0, 0))
            .transformed(rotate=(0, 0, math.degrees(angle)))
            .transformed(offset=(0, mid_r, 0))
            .box(half_w * 1.2, slot_len, slot_len * 0.35)
        )
        rim_solid = rim_solid.cut(slot)

    # Axle bore
    bore = (
        cq.Workplane("YZ", origin=(0, 0, 0))
        .circle(0.006)
        .extrude(half_w + 0.010, both=True)
    )
    rim_solid = rim_solid.cut(bore)

    rim_mesh = mesh_from_cadquery(rim_solid, "caster_rim")

    # Rubber tire: use TireGeometry for realistic tread and sidewall.
    tire = TireGeometry(
        CASTER_WHEEL_R,
        CASTER_WHEEL_W,
        inner_radius=CASTER_RIM_R - 0.004,
        carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.06),
        tread=TireTread(style="block", depth=0.008, count=22, land_ratio=0.55),
        sidewall=TireSidewall(style="rounded", bulge=0.05),
        shoulder=TireShoulder(width=0.006, radius=0.004),
    )
    tire_mesh = mesh_from_geometry(tire, "caster_tire")

    return rim_mesh, tire_mesh


def _brake_lever_mesh() -> object:
    """Side brake lever: a flat stamped-steel tab that pivots on the fork leg
    boss. Authored in a LOCAL frame whose pivot axis is local +X through the
    origin. The lever body extends downward (-Z) from the pivot and has a
    rounded toe. When engaged (positive rotation about X), the lever swings
    inward (+Y toward the wheel center) to press against the tire."""
    # Pivot boss hole (cylindrical, along X at origin)
    boss = (
        cq.Workplane("YZ", origin=(0, 0, 0))
        .circle(BRAKE_PIVOT_R - 0.001)
        .extrude(BRAKE_LEVER_W / 2.0, both=True)
    )
    # Lever arm: extends downward (-Z) from the pivot, with a slight curve
    # toward the wheel. The body overlaps the boss cylinder for fusion.
    arm = (
        cq.Workplane("XY", origin=(0, 0.008, -BRAKE_LEVER_LEN / 2.0 + BRAKE_PIVOT_R))
        .box(BRAKE_LEVER_W, BRAKE_LEVER_T, BRAKE_LEVER_LEN)
    )
    lever = boss.union(arm)
    # Rounded toe at the bottom of the lever
    toe = (
        cq.Workplane("XY", origin=(0, 0.008, -BRAKE_LEVER_LEN + BRAKE_PIVOT_R))
        .box(BRAKE_LEVER_W, BRAKE_LEVER_T + 0.004, 0.012)
    )
    lever = lever.union(toe)
    return mesh_from_cadquery(lever, "brake_lever")


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

    # --- Four pneumatic swivel casters with side brake levers ---
    # (CONTINUOUS swivel about Z; CONTINUOUS wheel roll about X;
    #  REVOLUTE brake lever about X on each fork)
    fork_mesh = _caster_fork_mesh()
    rim_mesh, tire_mesh = _caster_wheel_meshes()
    lever_mesh = _brake_lever_mesh()
    caster_positions = [
        (sx * (CAB_W / 2.0 - CASTER_INSET_X), sy * (CAB_D / 2.0 - CASTER_INSET_Y))
        for sx in (-1.0, 1.0)
        for sy in (-1.0, 1.0)
    ]
    for i, (cx, cy) in enumerate(caster_positions):
        # Fork part (swivels about Z)
        fork = model.part(f"caster_fork_{i}")
        fork.visual(fork_mesh, material=m_dark, name=f"fork_{i}")
        fork.inertial = Inertial.from_geometry(
            Box((0.08, 0.08, CASTER_GAP)),
            mass=0.6,
            origin=Origin(xyz=(0.0, -CASTER_FORK_DROP / 2.0, -CASTER_GAP / 2.0)),
        )

        # Wheel part (rolls about X): rim + tire as separate visuals
        wheel = model.part(f"caster_wheel_{i}")
        wheel.visual(rim_mesh, material=m_dark, name=f"rim_{i}")
        wheel.visual(tire_mesh, material=m_rubber, name=f"tire_{i}")
        wheel.inertial = Inertial.from_geometry(
            Box((CASTER_WHEEL_W, 2 * CASTER_WHEEL_R, 2 * CASTER_WHEEL_R)),
            mass=0.8,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
        )

        # Brake lever part (revolutes about X on the fork +X side)
        lever = model.part(f"brake_lever_{i}")
        lever.visual(lever_mesh, material=m_dark, name=f"lever_{i}")
        lever.inertial = Inertial.from_geometry(
            Box((BRAKE_LEVER_W, BRAKE_LEVER_T, BRAKE_LEVER_LEN)),
            mass=0.05,
            origin=Origin(xyz=(0.0, 0.0, -BRAKE_LEVER_LEN / 2.0)),
        )

        # Swivel: caster steers about a vertical axis at the mounting point
        model.articulation(
            f"frame_to_caster_{i}",
            ArticulationType.CONTINUOUS,
            parent=carcass,
            child=fork,
            origin=Origin(xyz=(cx, cy, FLOOR_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=6.0),
        )
        # Wheel rolls about its local axle (X). Joint origin places the wheel
        # at the trailing axle seat in the fork frame.
        model.articulation(
            f"caster_to_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(xyz=(0.0, -CASTER_FORK_DROP, AXLE_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=20.0),
        )
        # Brake lever: pivots about X on the fork leg boss. Positive rotation
        # swings the lever inward (+Y) toward the tire to engage the brake.
        # Origin is at the pivot boss location on the +X fork leg.
        half_w = CASTER_WHEEL_W / 2.0 + 0.008
        boss_z = AXLE_Z + CASTER_WHEEL_R * 0.40
        model.articulation(
            f"fork_to_brake_{i}",
            ArticulationType.REVOLUTE,
            parent=fork,
            child=lever,
            origin=Origin(xyz=(half_w + 0.008, -CASTER_FORK_DROP, boss_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=20.0,
                velocity=2.0,
                lower=0.0,
                upper=BRAKE_ENGAGE_ANGLE,
            ),
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
    forks = [object_model.get_part(f"caster_fork_{i}") for i in range(4)]
    wheels = [object_model.get_part(f"caster_wheel_{i}") for i in range(4)]
    levers = [object_model.get_part(f"brake_lever_{i}") for i in range(4)]
    swivel_joints = [
        object_model.get_articulation(f"frame_to_caster_{i}") for i in range(4)
    ]
    roll_joints = [
        object_model.get_articulation(f"caster_to_wheel_{i}") for i in range(4)
    ]
    brake_joints = [
        object_model.get_articulation(f"fork_to_brake_{i}") for i in range(4)
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
        # Test the open drawer from the reference (a middle shallow drawer).
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
    # provides the mechanical connection (faces are close but not touching).
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
    # The handle upright bosses bite down into the top trim band to bolt the
    # handle to the cabinet; allow that local mounting overlap.
    ctx.allow_overlap(
        carcass,
        handle,
        reason=(
            "The push-handle upright mounting bosses are intentionally seated "
            "into the cabinet top trim band; the small local overlap represents "
            "the bolted handle mount."
        ),
    )

    # --- Four pneumatic casters: swivel about Z, wheel rolls about X, ground. ---
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

    # --- Pneumatic wheels are visibly larger than the old 52 mm casters ---
    for i, wheel in enumerate(wheels):
        w_aabb = ctx.part_world_aabb(wheel)
        ctx.check(
            f"caster wheel {i} reaches the ground plane",
            w_aabb is not None and abs(w_aabb[0][2]) < 0.02,
            details=f"wheel_z_min={None if w_aabb is None else w_aabb[0][2]}",
        )
        if w_aabb is not None:
            wheel_diam = w_aabb[1][2] - w_aabb[0][2]
            ctx.check(
                f"caster wheel {i} is pneumatic-sized (>120 mm diameter)",
                wheel_diam > 0.12,
                details=f"wheel_diam={wheel_diam:.4f}",
            )

    # Swiveling caster 0 sweeps its wheel center around the swivel axis.
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

    # --- Four brake levers: revolute about X, mounted on fork, with limits. ---
    ctx.check(
        "four brake levers authored",
        len(levers) == 4,
        details=f"levers={len(levers)}",
    )
    for i, bj in enumerate(brake_joints):
        bax = bj.axis
        ctx.check(
            f"brake lever {i} is revolute",
            str(bj.articulation_type).upper().endswith("REVOLUTE"),
            details=f"type={bj.articulation_type}",
        )
        ctx.check(
            f"brake lever {i} pivots about horizontal X axis",
            abs(bax[0]) > 0.99 and abs(bax[1]) < 0.01 and abs(bax[2]) < 0.01,
            details=f"axis={bax}",
        )
        lim = bj.motion_limits
        ctx.check(
            f"brake lever {i} has engagement range",
            lim is not None
            and lim.lower is not None
            and lim.upper is not None
            and lim.lower == 0.0
            and lim.upper > 0.2,
            details=f"lower={lim.lower}, upper={lim.upper}",
        )

    # Engaging the brake lever sweeps the lever arm inward. Check via AABB
    # center displacement (the part origin is the pivot, which doesn't move).
    bj0 = brake_joints[0]
    with ctx.pose({bj0: 0.0}):
        lever_rest_aabb = ctx.part_world_aabb(levers[0])
    with ctx.pose({bj0: BRAKE_ENGAGE_ANGLE}):
        lever_eng_aabb = ctx.part_world_aabb(levers[0])
    if lever_rest_aabb is not None and lever_eng_aabb is not None:
        rest_cz = 0.5 * (lever_rest_aabb[0][2] + lever_rest_aabb[1][2])
        eng_cz = 0.5 * (lever_eng_aabb[0][2] + lever_eng_aabb[1][2])
        rest_cy = 0.5 * (lever_rest_aabb[0][1] + lever_rest_aabb[1][1])
        eng_cy = 0.5 * (lever_eng_aabb[0][1] + lever_eng_aabb[1][1])
        ctx.check(
            "brake lever 0 sweeps when engaged",
            abs(eng_cz - rest_cz) + abs(eng_cy - rest_cy) > 0.005,
            details=f"rest_c=(..{rest_cy:.4f},{rest_cz:.4f}), eng_c=(..{eng_cy:.4f},{eng_cz:.4f})",
        )
    else:
        ctx.fail("brake lever 0 AABBs resolve", f"rest={lever_rest_aabb}, eng={lever_eng_aabb}")

    # The wheel is captured in the fork yoke on the axle; the brake lever pivots
    # on the fork boss; the swivel post bites into the cabinet floor.
    for i in range(4):
        ctx.allow_overlap(
            forks[i],
            wheels[i],
            reason=(
                "The pneumatic caster wheel is captured in the trailing yoke of "
                "the fork on the axle; the small local overlap at the axle "
                "represents the real pin-in-yoke rolling joint."
            ),
        )
        ctx.allow_overlap(
            forks[i],
            levers[i],
            reason=(
                "The brake lever pivots on the fork leg boss; the small local "
                "overlap at the pivot boss represents the captured pin joint."
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
