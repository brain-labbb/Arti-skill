from __future__ import annotations

# Air conditioner outdoor unit (condensing unit), wall-mounted.
# Variant: side-hinged service door with visible hinge barrels and cam latch.
#
# Reference image: a weathered white/rusty steel condensing-unit box mounted on a
# bottom steel angle-iron bracket against a wall. The front-left face carries a
# large circular protective wire fan grille with an axial fan spinning behind it.
# The right side has a side-hinged service/electrical access door with visible
# barrel hinge knuckles and a quarter-turn cam latch, plus exposed copper
# refrigerant lines / service valves.
#
# Articulated mechanisms (reasoned from physics):
#   * housing_to_fan          -> CONTINUOUS axial fan spinning about the front (+Y) axis.
#   * housing_to_service_door -> REVOLUTE side access door swinging open on a vertical hinge.
# The mounting bracket is the fixed root; the housing is rigidly bolted to it.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ----------------------------------------------------------------------------
# Real-world dimensions (meters).
# ----------------------------------------------------------------------------
BODY_W = 0.800  # along +X (front-to-back the unit is shallow; width across the front face)
BODY_D = 0.300  # along +Y (depth, wall-to-front)
BODY_H = 0.540  # along +Z (height)

WALL_TH = 0.012  # sheet-metal apparent wall thickness for the open back shell

FAN_BORE_R = 0.215  # circular grille opening radius on the front face
FAN_CENTER_X = -0.165  # fan opening center along the body width (front-left in image)
FAN_CENTER_Z = 0.300  # fan opening center height above the body bottom

FAN_OUTER_R = 0.200
FAN_HUB_R = 0.045
FAN_THICK = 0.045

GRILLE_WIRE_R = 0.0035  # radius of the protective grille wires
GRILLE_RINGS = 4  # number of concentric grille rings (plus hub spider)
GRILLE_SPOKES = 6

PANEL_W = 0.170  # service access panel (on the +X / right side face)
PANEL_H = 0.300
PANEL_TH = 0.010
PANEL_RECESS = 0.010  # how far the panel sits proud of the recessed pocket

BRACKET_LEG_H = 0.150  # height of the wall bracket below the body
BRACKET_ANGLE = 0.035  # angle-iron leg width

# Hinge-line position (service door hinge, matches the articulation origin).
POCKET_FLOOR_X = BODY_W / 2.0 - 0.022
HINGE_X = POCKET_FLOOR_X + PANEL_TH / 2.0
HINGE_Y = -(PANEL_W / 2.0)
HINGE_Z = 0.0


# ----------------------------------------------------------------------------
# Geometry builders.
# ----------------------------------------------------------------------------
def _housing_shell() -> cq.Workplane:
    """Sheet-metal cabinet: a box hollowed from the back (-Y), with a circular
    fan opening cut through the front (+Y) face and a recessed service pocket on
    the right (+X) side face."""
    # Outer box centered on its own origin at the body center.
    outer = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H)
        .edges("|Y or |Z")
        .fillet(0.010)
    )

    # Hollow the cabinet from the open back face (-Y), leaving real wall thickness.
    inner = cq.Workplane("XY").box(
        BODY_W - 2 * WALL_TH,
        BODY_D - WALL_TH,  # open at the back, so only one wall removed in Y
        BODY_H - 2 * WALL_TH,
    )
    # Shift the inner cavity toward -Y so the back is fully open.
    inner = inner.translate((0.0, -WALL_TH / 2.0, 0.0))
    shell = outer.cut(inner)

    # Circular fan throat cut through the front (+Y) wall only (back stays closed).
    cz = FAN_CENTER_Z - BODY_H / 2.0
    fan_cut = cq.Solid.makeCylinder(
        FAN_BORE_R,
        BODY_D,  # plenty long; positioned to clear only the front wall
        pnt=cq.Vector(FAN_CENTER_X, BODY_D / 2.0 + 0.001, cz),
        dir=cq.Vector(0.0, -1.0, 0.0),
    )
    shell = shell.cut(cq.Workplane("XY").add(fan_cut))

    # Shallow rectangular service pocket recessed into the right (+X) side face.
    pocket_depth = 0.022
    pocket = (
        cq.Workplane("YZ")
        .workplane(offset=BODY_W / 2.0 + 0.001)
        .center(0.0, 0.0)  # YZ plane: local x->Y(depth), y->Z(height)
        .rect(PANEL_W + 0.010, PANEL_H + 0.010)
        .extrude(-pocket_depth)
    )
    shell = shell.cut(pocket)

    return shell


def _fan_shroud_ring() -> cq.Workplane:
    """A short raised flow-ring (orifice) framing the fan opening on the +Y front
    face, so the circular opening reads as a real venturi rather than a raw hole."""
    cz = FAN_CENTER_Z - BODY_H / 2.0
    y0 = BODY_D / 2.0 - 0.004  # start just inside the front wall
    outer = cq.Solid.makeCylinder(
        FAN_BORE_R + 0.020,
        0.026,
        pnt=cq.Vector(FAN_CENTER_X, y0, cz),
        dir=cq.Vector(0.0, 1.0, 0.0),
    )
    bore = cq.Solid.makeCylinder(
        FAN_BORE_R,
        0.040,
        pnt=cq.Vector(FAN_CENTER_X, y0 - 0.005, cz),
        dir=cq.Vector(0.0, 1.0, 0.0),
    )
    ring = cq.Workplane("XY").add(outer).cut(cq.Workplane("XY").add(bore))
    return ring


def _grille() -> cq.Workplane:
    """Protective wire fan grille: concentric rings plus radial diameter spokes
    plus a center boss, all welded into one solid, sitting just proud of the
    front face over the fan opening.

    Authored in a local frame where the grille lies in the XZ plane (its tube
    axis along local +Y), centered on (0, 0, 0); the FIXED joint places this
    frame on the cabinet front face, with +Y pointing out in front of the fan."""
    z_front = 0.032  # standoff of the grille plane in front of the shroud ring (+Y)

    solids: list[cq.Solid] = []

    # Concentric rings (tori) whose tube axis points along +Y.
    for i in range(GRILLE_RINGS):
        rr = FAN_BORE_R * (i + 1) / GRILLE_RINGS
        solids.append(
            cq.Solid.makeTorus(
                rr,
                GRILLE_WIRE_R,
                pnt=cq.Vector(0.0, z_front, 0.0),
                dir=cq.Vector(0.0, 1.0, 0.0),
            )
        )

    # Radial diameter spokes: rods crossing the full opening through center.
    # Slightly thicker than the rings so the boolean union welds cleanly at every
    # ring crossing instead of merely touching tangentially.
    spoke_r = GRILLE_WIRE_R + 0.0015
    for k in range(GRILLE_SPOKES):
        ang = math.pi * k / GRILLE_SPOKES
        dx = math.cos(ang)
        dz = math.sin(ang)
        length = 2.0 * FAN_BORE_R
        solids.append(
            cq.Solid.makeCylinder(
                spoke_r,
                length,
                pnt=cq.Vector(-dx * FAN_BORE_R, z_front, -dz * FAN_BORE_R),
                dir=cq.Vector(dx, 0.0, dz),
            )
        )

    # Center boss / fan cap: a short cylinder along +Y at the hub, overlapping
    # all the spokes so the whole grille fuses into one connected island.
    solids.append(
        cq.Solid.makeCylinder(
            FAN_HUB_R * 0.9,
            0.016,
            pnt=cq.Vector(0.0, z_front - 0.006, 0.0),
            dir=cq.Vector(0.0, 1.0, 0.0),
        )
    )

    # Four mounting tabs at the rim that reach back along -Y from the grille
    # plane into the raised shroud ring, fastening the grille to the housing.
    # In the grille local frame the shroud ring wall lies around local y in
    # [-0.004, 0.022] (world 0.146..0.172), so the tabs dip to local y ~0.006.
    tab_r = FAN_BORE_R + 0.006  # at the shroud-ring wall radius
    for k in range(4):
        ang = math.pi / 4.0 + math.pi * k / 2.0
        cx = math.cos(ang) * tab_r
        cz = math.sin(ang) * tab_r
        solids.append(
            cq.Solid.makeCylinder(
                0.007,
                z_front - 0.006,  # from local y=0.006 (in the shroud) to the rim ring
                pnt=cq.Vector(cx, 0.006, cz),
                dir=cq.Vector(0.0, 1.0, 0.0),
            )
        )

    fused = solids[0]
    for s in solids[1:]:
        fused = fused.fuse(s)

    return cq.Workplane("XY").add(fused)


def _fan_motor_mount() -> cq.Workplane:
    """Fan motor and its cross-strut bracket, authored in the housing part frame.

    A motor can sits on a horizontal strut bar that spans the fan throat and ties
    into the cabinet front opening, grounding the motor (and therefore the rotor,
    which seats on the motor shaft) to the housing."""
    cx = FAN_CENTER_X
    cz = FAN_CENTER_Z - BODY_H / 2.0
    # Rotor center is at y=0.120 (rotor spans y 0.0975..0.1425). The motor can
    # sits behind the rotor, with a short shaft reaching forward into the hub.

    solids: list[cq.Solid] = []

    # Motor can (axis along +Y, facing the rotor).
    solids.append(
        cq.Solid.makeCylinder(
            0.040,
            0.070,
            pnt=cq.Vector(cx, 0.030, cz),  # y: 0.030 -> 0.100
            dir=cq.Vector(0.0, 1.0, 0.0),
        )
    )
    # Short output shaft toward the rotor hub (stops at the rotor mid-plane).
    solids.append(
        cq.Solid.makeCylinder(
            0.008,
            0.040,
            pnt=cq.Vector(cx, 0.090, cz),  # y: 0.090 -> 0.130
            dir=cq.Vector(0.0, 1.0, 0.0),
        )
    )
    # Horizontal strut bar spanning the throat across X, tying both opening edges.
    strut_len = 2.0 * FAN_BORE_R + 0.02
    solids.append(
        cq.Solid.makeBox(
            strut_len,
            0.020,
            0.024,
            pnt=cq.Vector(cx - strut_len / 2.0, 0.050, cz - 0.012),
        )
    )

    fused = solids[0]
    for s in solids[1:]:
        fused = fused.fuse(s)
    return cq.Workplane("XY").add(fused)


def _fan_rotor_mesh():
    """Axial fan rotor that spins about local +Z (FanRotorGeometry convention)."""
    return mesh_from_geometry(
        FanRotorGeometry(
            FAN_OUTER_R,
            FAN_HUB_R,
            5,
            thickness=FAN_THICK,
            blade_pitch_deg=26.0,
            blade_sweep_deg=18.0,
            blade=FanRotorBlade(shape="broad", camber=0.12),
            hub=FanRotorHub(style="domed", bore_diameter=0.012),
            center=True,
        ),
        "fan_rotor",
    )


def _service_panel() -> cq.Workplane:
    """Hinged sheet-metal service/electrical access cover for the right side.

    Authored in a hinge-line local frame: the vertical hinge runs along local Z
    at local origin, and the closed panel extends along +Y (depth) from the
    hinge. The slab is thin along X (its outward normal)."""
    # Slab spans X in [-PANEL_TH/2, +PANEL_TH/2], Y in [0, PANEL_W], Z in [-PANEL_H/2, +PANEL_H/2].
    panel = (
        cq.Workplane("XY")
        .box(PANEL_TH, PANEL_W, PANEL_H, centered=(True, False, True))
        .edges("|X")
        .fillet(0.006)
    )
    # Recessed louver lines and a finger-pull lip near the free (+Y) edge.
    pull = (
        cq.Workplane("XY")
        .box(0.006, 0.040, 0.060, centered=(True, False, True))
        .translate((PANEL_TH / 2.0, PANEL_W - 0.040, 0.0))
    )
    panel = panel.union(pull)
    return panel


def _hinge_barrels() -> cq.Workplane:
    """Two visible hinge barrel knuckles at the service door hinge line.

    Each knuckle is a barrel cylinder plus a mounting leaf that ties into the
    housing side wall. Authored in housing part-frame coordinates so the mesh
    can be attached as a housing visual.
    """
    barrel_r = 0.007
    barrel_len = 0.038
    leaf_width = BODY_W / 2.0 - HINGE_X + barrel_r  # reach from barrel center to wall
    leaf_thick = 0.020  # along Y (width of the leaf along the hinge line)

    solids: list[cq.Solid] = []
    for zc in (PANEL_H * 0.35, -PANEL_H * 0.35):
        # Barrel cylinder along Z at the hinge line.
        barrel = cq.Solid.makeCylinder(
            barrel_r,
            barrel_len,
            pnt=cq.Vector(HINGE_X, HINGE_Y, zc - barrel_len / 2.0),
            dir=cq.Vector(0.0, 0.0, 1.0),
        )
        solids.append(barrel)
        # Mounting leaf plate from barrel outward to the housing wall (+X).
        leaf = cq.Solid.makeBox(
            leaf_width,
            leaf_thick,
            barrel_len,
            pnt=cq.Vector(HINGE_X, HINGE_Y - leaf_thick / 2.0, zc - barrel_len / 2.0),
        )
        solids.append(leaf)
        # Pin cap disc on top of each barrel for visible pin-end detail.
        cap = cq.Solid.makeCylinder(
            barrel_r + 0.002,
            0.003,
            pnt=cq.Vector(HINGE_X, HINGE_Y, zc + barrel_len / 2.0),
            dir=cq.Vector(0.0, 0.0, 1.0),
        )
        solids.append(cap)

    fused = solids[0]
    for s in solids[1:]:
        fused = fused.fuse(s)
    return cq.Workplane("XY").add(fused)


def _door_latch() -> cq.Workplane:
    """Quarter-turn cam latch on the free edge of the service door.

    Authored in the door part local frame. The door extends along +Y from
    the hinge (y=0), and the slab is thin along X. The latch sits near the
    free (+Y) edge, protruding outward (+X) from the door face.
    """
    latch_y = PANEL_W - 0.040
    face_x = PANEL_TH / 2.0

    # Latch body: a short cylinder on the outer door face.
    body = cq.Solid.makeCylinder(
        0.012,
        0.010,
        pnt=cq.Vector(face_x, latch_y, 0.0),
        dir=cq.Vector(1.0, 0.0, 0.0),
    )
    # Cam tab: a flat handle extending outward for finger grip.
    tab = cq.Solid.makeBox(
        0.004,
        0.030,
        0.010,
        pnt=cq.Vector(face_x + 0.010, latch_y - 0.015, -0.005),
    )
    # Base washer against the door face.
    washer = cq.Solid.makeCylinder(
        0.015,
        0.002,
        pnt=cq.Vector(face_x - 0.001, latch_y, 0.0),
        dir=cq.Vector(1.0, 0.0, 0.0),
    )
    fused = body.fuse(tab).fuse(washer)
    return cq.Workplane("XY").add(fused)


def _bracket() -> cq.Workplane:
    """Steel wall-mounting bracket: two angle-iron legs under the body joined by
    a front and rear cross rail, forming the support frame seen at the base."""
    leg_w = BRACKET_ANGLE
    leg_t = 0.006
    rail_t = 0.006
    inset = 0.060  # how far the legs sit in from the side edges

    parts = None

    def _add(wp):
        nonlocal parts
        parts = wp if parts is None else parts.union(wp)

    # Two L-shaped legs along the depth (Y), one near each side (+/-X).
    for sign in (-1.0, 1.0):
        x0 = sign * (BODY_W / 2.0 - inset)
        # vertical web of the L
        web = (
            cq.Workplane("XY")
            .box(leg_t, BODY_D + 0.040, BRACKET_LEG_H)
            .translate((x0, 0.0, -BRACKET_LEG_H / 2.0))
        )
        _add(web)
        # horizontal foot flange at the bottom
        foot = (
            cq.Workplane("XY")
            .box(leg_w, BODY_D + 0.040, leg_t)
            .translate((x0 + sign * leg_w / 2.0, 0.0, -BRACKET_LEG_H + leg_t / 2.0))
        )
        _add(foot)
        # top flange that bolts under the cabinet
        top = (
            cq.Workplane("XY")
            .box(leg_w, BODY_D + 0.040, leg_t)
            .translate((x0 + sign * leg_w / 2.0, 0.0, -leg_t / 2.0))
        )
        _add(top)

    # Front and rear cross rails tying the two legs together near the bottom.
    for yy in (BODY_D / 2.0 - 0.010, -(BODY_D / 2.0 - 0.010)):
        rail = (
            cq.Workplane("XY")
            .box(BODY_W - 2 * inset + leg_w, rail_t, leg_w)
            .translate((0.0, yy, -BRACKET_LEG_H + leg_w / 2.0))
        )
        _add(rail)

    return parts


def _refrigerant_lines() -> cq.Workplane:
    """Two service valves / copper refrigerant stub lines on the right side, below
    the service panel, connecting the unit to the building lineset."""
    lines = None

    def _add(wp):
        nonlocal lines
        lines = wp if lines is None else lines.union(wp)

    base_x = BODY_W / 2.0 - 0.012  # embed slightly into the solid side wall
    for zc in (-0.205, -0.160):
        # short horizontal copper stub pointing out +X
        stub = cq.Solid.makeCylinder(
            0.009,
            0.060,
            pnt=cq.Vector(base_x, 0.030, zc),
            dir=cq.Vector(1.0, 0.0, 0.0),
        )
        _add(cq.Workplane("XY").add(stub))
        # service-valve body where it meets the cabinet
        valve = cq.Solid.makeCylinder(
            0.015,
            0.026,
            pnt=cq.Vector(base_x, 0.030, zc),
            dir=cq.Vector(1.0, 0.0, 0.0),
        )
        _add(cq.Workplane("XY").add(valve))

    # Vertical copper manifold tying the two valve stubs into one connected part.
    manifold = cq.Solid.makeCylinder(
        0.008,
        0.060,
        pnt=cq.Vector(base_x + 0.012, 0.030, -0.205),
        dir=cq.Vector(0.0, 0.0, 1.0),
    )
    _add(cq.Workplane("XY").add(manifold))
    return lines


# ----------------------------------------------------------------------------
# Model assembly.
# ----------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ac_outdoor_unit")

    casing_mat = model.material("ac_casing", rgba=(0.86, 0.85, 0.82, 1.0))
    steel_mat = model.material("ac_steel", rgba=(0.35, 0.35, 0.37, 1.0))
    grille_mat = model.material("ac_grille", rgba=(0.18, 0.18, 0.19, 1.0))
    fan_mat = model.material("ac_fan", rgba=(0.10, 0.10, 0.11, 1.0))
    panel_mat = model.material("ac_panel", rgba=(0.78, 0.77, 0.74, 1.0))
    copper_mat = model.material("ac_copper", rgba=(0.62, 0.40, 0.22, 1.0))

    # --- Root: wall-mounting bracket ---------------------------------------
    bracket = model.part("mounting_bracket")
    bracket.visual(
        mesh_from_cadquery(_bracket(), "mounting_bracket"),
        material=steel_mat,
        name="bracket_frame",
    )
    bracket.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, BRACKET_LEG_H)), mass=4.0
    )

    # --- Housing (cabinet shell) -------------------------------------------
    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_housing_shell(), "housing_shell"),
        material=casing_mat,
        name="housing_shell",
    )
    housing.visual(
        mesh_from_cadquery(_fan_shroud_ring(), "fan_shroud_ring"),
        material=casing_mat,
        name="fan_shroud_ring",
    )
    housing.visual(
        mesh_from_cadquery(_fan_motor_mount(), "fan_motor_mount"),
        material=steel_mat,
        name="fan_motor_mount",
    )
    housing.visual(
        mesh_from_cadquery(_hinge_barrels(), "hinge_barrels"),
        material=steel_mat,
        name="hinge_barrels",
    )
    housing.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, BODY_H)), mass=28.0
    )

    # --- Refrigerant service valves / copper stub lines --------------------
    lineset = model.part("service_valves")
    lineset.visual(
        mesh_from_cadquery(_refrigerant_lines(), "service_valves"),
        material=copper_mat,
        name="valve_stubs",
    )
    lineset.inertial = Inertial.from_geometry(Box((0.07, 0.02, 0.10)), mass=0.5)

    # --- Protective grille over the fan opening ----------------------------
    grille = model.part("fan_grille")
    grille.visual(
        mesh_from_cadquery(_grille(), "fan_grille"),
        material=grille_mat,
        name="grille_rings",
    )
    grille.inertial = Inertial.from_geometry(Box((0.42, 0.05, 0.42)), mass=0.6)

    # --- Axial fan rotor ---------------------------------------------------
    fan = model.part("fan_rotor")
    # FanRotorGeometry spins about local +Z; rotate so its axis points along the
    # housing's front (+Y) direction, then the joint axis will be (0,1,0).
    fan.visual(
        _fan_rotor_mesh(),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=fan_mat,
        name="fan_rotor",
    )
    fan.inertial = Inertial.from_geometry(Box((0.40, 0.05, 0.40)), mass=0.9)

    # --- Service / electrical access door ----------------------------------
    door = model.part("service_door")
    door.visual(
        mesh_from_cadquery(_service_panel(), "service_door"),
        material=panel_mat,
        name="door_panel",
    )
    door.visual(
        mesh_from_cadquery(_door_latch(), "door_latch"),
        material=steel_mat,
        name="door_latch",
    )
    door.inertial = Inertial.from_geometry(
        Box((PANEL_TH, PANEL_H, PANEL_W)), mass=0.8
    )

    # ----- Articulations ---------------------------------------------------
    # Bracket is the fixed root. Housing bottom sits on the bracket top flange.
    # Bracket local frame: body center at z=0, bracket legs below (z<0).
    # So the housing body center is at +BODY_H/2 above the bracket frame origin.
    model.articulation(
        "bracket_to_housing",
        ArticulationType.FIXED,
        parent=bracket,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
    )

    # Fan opening center, expressed in housing part frame (body centered at origin):
    fan_x = FAN_CENTER_X
    fan_z = FAN_CENTER_Z - BODY_H / 2.0
    fan_front_y = BODY_D / 2.0 - 0.030  # rotor sits just inside the front face

    # Grille mounts on the housing front, fixed, centered on the fan opening.
    model.articulation(
        "housing_to_grille",
        ArticulationType.FIXED,
        parent=housing,
        child=grille,
        origin=Origin(xyz=(fan_x, BODY_D / 2.0, fan_z)),
    )

    # Copper service valves / lineset: fixed to the housing (geometry already in
    # housing coordinates, so the joint origin is the identity).
    model.articulation(
        "housing_to_service_valves",
        ArticulationType.FIXED,
        parent=housing,
        child=lineset,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # Axial fan: CONTINUOUS about the housing front (+Y) axis.
    model.articulation(
        "housing_to_fan",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=fan,
        origin=Origin(xyz=(fan_x, fan_front_y, fan_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=120.0),
    )

    # Service door: REVOLUTE on a vertical (Z) hinge at the rear edge of the
    # right side service pocket. The closed door seats in the recessed pocket
    # and extends along +Y from the hinge; axis -Z makes positive q swing the
    # free (+Y) edge outward in +X, away from the cabinet.
    model.articulation(
        "housing_to_service_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, HINGE_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.9),
    )

    return model


# ----------------------------------------------------------------------------
# Tests.
# ----------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bracket = object_model.get_part("mounting_bracket")
    housing = object_model.get_part("housing")
    grille = object_model.get_part("fan_grille")
    fan = object_model.get_part("fan_rotor")
    door = object_model.get_part("service_door")

    valves = object_model.get_part("service_valves")

    fan_joint = object_model.get_articulation("housing_to_fan")
    door_joint = object_model.get_articulation("housing_to_service_door")

    # Intentional embeds:
    #  - copper valve stubs pass through / seat into the cabinet side wall.
    #  - the fan rotor hub seats on the motor shaft / motor can (shaft capture).
    #  - hinge barrel knuckles embed into the door edge at the hinge line.
    ctx.allow_overlap(
        housing,
        valves,
        elem_a="housing_shell",
        elem_b="valve_stubs",
        reason="Copper service valves intentionally penetrate the side wall where the lineset enters the cabinet.",
    )
    ctx.allow_overlap(
        housing,
        fan,
        elem_a="fan_motor_mount",
        elem_b="fan_rotor",
        reason="The axial fan rotor hub is captured on the motor shaft of the motor-mount bracket.",
    )
    ctx.allow_overlap(
        grille,
        housing,
        elem_a="grille_rings",
        elem_b="fan_shroud_ring",
        reason="The grille mounting tabs seat into the raised shroud ring that frames the fan opening.",
    )
    ctx.allow_overlap(
        housing,
        door,
        elem_a="hinge_barrels",
        elem_b="door_panel",
        reason="The hinge barrel knuckles seat into the door edge at the hinge line (captured-pin hinge).",
    )

    # --- Hero parts present ------------------------------------------------
    for name, prt in (
        ("mounting_bracket", bracket),
        ("housing", housing),
        ("fan_grille", grille),
        ("fan_rotor", fan),
        ("service_door", door),
    ):
        ctx.check(f"{name}_present", prt is not None, f"Expected part {name}.")

    # --- Single root is the bracket ---------------------------------------
    roots = object_model.root_parts()
    ctx.check(
        "single_root_bracket",
        len(roots) == 1 and roots[0].name == "mounting_bracket",
        f"roots={[r.name for r in roots]}",
    )

    # --- Joint types / axes ------------------------------------------------
    ctx.check(
        "fan_joint_continuous",
        str(fan_joint.articulation_type).endswith("CONTINUOUS"),
        f"type={fan_joint.articulation_type}",
    )
    ctx.check(
        "fan_axis_front_y",
        abs(fan_joint.axis[1]) > 0.99 and abs(fan_joint.axis[0]) < 0.01,
        f"axis={fan_joint.axis}",
    )
    ctx.check(
        "door_joint_revolute",
        str(door_joint.articulation_type).endswith("REVOLUTE"),
        f"type={door_joint.articulation_type}",
    )
    ctx.check(
        "door_axis_vertical",
        abs(door_joint.axis[2]) > 0.99,
        f"axis={door_joint.axis}",
    )

    # --- Overall silhouette: tall white box on a bracket -------------------
    hb = ctx.part_world_aabb(housing)
    if hb is not None:
        mins, maxs = hb
        size = tuple(float(maxs[i] - mins[i]) for i in range(3))
        ctx.check(
            "housing_proportions",
            0.70 <= size[0] <= 0.90 and 0.45 <= size[2] <= 0.62,
            f"housing size={size!r}",
        )

    # --- Bracket sits below the housing ------------------------------------
    bb = ctx.part_world_aabb(bracket)
    if hb is not None and bb is not None:
        ctx.check(
            "bracket_below_housing",
            bb[0][2] < hb[0][2] + 0.01,
            f"bracket_minz={bb[0][2]}, housing_minz={hb[0][2]}",
        )

    # --- Fan rotor is round and sized to the grille opening ----------------
    fb = ctx.part_world_aabb(fan)
    if fb is not None:
        fmins, fmaxs = fb
        fsize = tuple(float(fmaxs[i] - fmins[i]) for i in range(3))
        diameter = max(fsize[0], fsize[2])
        ctx.check(
            "fan_diameter",
            abs(diameter - FAN_OUTER_R * 2.0) <= 0.03,
            f"fan size={fsize!r}",
        )
        ctx.check(
            "fan_thin_along_axis",
            fsize[1] <= 0.10,
            f"fan size={fsize!r}",
        )

    # --- Fan sits behind / within the grille footprint (front face) --------
    ctx.expect_within(
        fan,
        grille,
        axes="xz",
        margin=0.02,
        name="fan_within_grille_footprint",
    )

    # --- Grille is fastened to the shroud ring (mounting tabs seat in) ------
    ctx.expect_contact(
        grille,
        housing,
        elem_a="grille_rings",
        elem_b="fan_shroud_ring",
        contact_tol=0.001,
        name="grille_seated_on_shroud",
    )

    # --- Rotor hub is captured on the motor mount (grounds the rotor) -------
    ctx.expect_contact(
        fan,
        housing,
        elem_a="fan_rotor",
        elem_b="fan_motor_mount",
        contact_tol=0.001,
        name="rotor_on_motor_shaft",
    )

    # --- Grille sits in front of the fan along +Y (does not pass behind it) -
    gb = ctx.part_world_aabb(grille)
    if gb is not None and fb is not None:
        ctx.check(
            "grille_in_front_of_fan",
            gb[1][1] >= fb[1][1] - 0.005,
            f"grille_maxy={gb[1][1]}, fan_maxy={fb[1][1]}",
        )

    # --- Fan actually spins: blade extents shift under rotation ------------
    tip_rest = ctx.part_element_world_aabb(fan, elem="fan_rotor")
    with ctx.pose({fan_joint: 0.6}):
        tip_spun = ctx.part_element_world_aabb(fan, elem="fan_rotor")
    if tip_rest is not None and tip_spun is not None:
        changed = (
            abs(tip_rest[1][0] - tip_spun[1][0]) > 0.02
            or abs(tip_rest[1][2] - tip_spun[1][2]) > 0.02
            or abs(tip_rest[0][0] - tip_spun[0][0]) > 0.02
        )
        ctx.check("fan_rotation_moves_blades", changed, "Blade extents unchanged under spin.")

    # --- Visible hinge barrels present on housing at hinge line ------------
    ctx.check(
        "hinge_barrels_present",
        any(v.name == "hinge_barrels" for v in housing.visuals),
        "Expected hinge_barrels visual on housing.",
    )
    hinge_aabb = ctx.part_element_world_aabb(housing, elem="hinge_barrels")
    if hinge_aabb is not None:
        hmins, hmaxs = hinge_aabb
        hsize = tuple(float(hmaxs[i] - hmins[i]) for i in range(3))
        ctx.check(
            "hinge_barrels_span_door_height",
            hsize[2] >= PANEL_H * 0.40,
            f"hinge barrel z-span={hsize[2]:.3f}, expected>={PANEL_H * 0.40:.3f}",
        )

    # --- Visible latch present on the door ---------------------------------
    ctx.check(
        "door_latch_present",
        any(v.name == "door_latch" for v in door.visuals),
        "Expected door_latch visual on service_door.",
    )
    latch_aabb = ctx.part_element_world_aabb(door, elem="door_latch")
    door_aabb = ctx.part_world_aabb(door)
    if latch_aabb is not None and door_aabb is not None:
        # Latch should be near the free (+Y) edge of the door, not at the hinge.
        latch_cy = (latch_aabb[0][1] + latch_aabb[1][1]) / 2.0
        door_cy = (door_aabb[0][1] + door_aabb[1][1]) / 2.0
        ctx.check(
            "latch_on_free_edge",
            latch_cy > door_cy,
            f"latch_cy={latch_cy:.3f}, door_cy={door_cy:.3f}",
        )

    # --- Service door swings outward (+X) when opened ----------------------
    door_closed = ctx.part_world_position(door)
    door_closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: 1.4}):
        door_open_aabb = ctx.part_world_aabb(door)
        door_open = ctx.part_world_position(door)
    if door_closed is not None and door_open is not None:
        moved_out = door_open_aabb[1][0] > door_closed_aabb[1][0] + 0.05
        ctx.check(
            "door_opens_outward",
            moved_out,
            f"closed_maxx={door_closed_aabb[1][0]}, open_maxx={door_open_aabb[1][0]}",
        )

    # --- Closed door hugs the right side face -----------------------------
    if door_closed_aabb is not None and hb is not None:
        ctx.check(
            "door_seated_on_side",
            abs(door_closed_aabb[0][0] - hb[1][0]) <= 0.03,
            f"door_minx={door_closed_aabb[0][0]}, housing_maxx={hb[1][0]}",
        )

    # --- Hinge barrel stays at the hinge line when door opens --------------
    if hinge_aabb is not None and door_open_aabb is not None:
        # Hinge barrels remain on the housing frame side, not flying away with the door.
        hinge_cx = (hinge_aabb[0][0] + hinge_aabb[1][0]) / 2.0
        ctx.check(
            "hinge_stays_on_frame",
            hinge_cx < hb[1][0] + 0.01 if hb is not None else True,
            f"hinge_cx={hinge_cx:.3f}, housing_maxx={hb[1][0]:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
