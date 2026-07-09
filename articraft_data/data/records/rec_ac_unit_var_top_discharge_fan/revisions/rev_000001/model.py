from __future__ import annotations

# Air conditioner outdoor unit (condensing unit), top-discharge variant.
#
# Variant of the wall-mounted condensing unit where the axial fan discharges
# upward through a circular top grille instead of through the front face.
# The rectangular housing, side service panel, refrigerant lines, and wall
# mounting bracket are identical to the parent side-discharge unit.
#
# Articulated mechanisms:
#   * housing_to_fan           -> CONTINUOUS axial fan spinning about the top (+Z) axis.
#   * housing_to_service_panel -> REVOLUTE side access cover swinging open on a vertical hinge.
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
BODY_W = 0.800  # along +X (width across the front face)
BODY_D = 0.300  # along +Y (depth, wall-to-front)
BODY_H = 0.540  # along +Z (height)

WALL_TH = 0.012  # sheet-metal wall thickness

# Top-discharge fan: smaller rotor to fit within the 0.800 x 0.300 top face.
FAN_BORE_R = 0.130  # circular grille opening radius on the top face
FAN_CENTER_X = 0.0  # fan centered on the top face in X
FAN_CENTER_Y = 0.0  # fan centered on the top face in Y

FAN_OUTER_R = 0.120
FAN_HUB_R = 0.030
FAN_THICK = 0.035

GRILLE_WIRE_R = 0.0035  # radius of the protective grille wires
GRILLE_RINGS = 4  # number of concentric grille rings
GRILLE_SPOKES = 6

PANEL_W = 0.170  # service access panel (on the +X / right side face)
PANEL_H = 0.300
PANEL_TH = 0.010
PANEL_RECESS = 0.010

BRACKET_LEG_H = 0.150  # height of the wall bracket below the body
BRACKET_ANGLE = 0.035  # angle-iron leg width


# ----------------------------------------------------------------------------
# Geometry builders.
# ----------------------------------------------------------------------------
def _housing_shell() -> cq.Workplane:
    """Sheet-metal cabinet: a box hollowed from the back (-Y), with a circular
    fan opening cut through the top (+Z) face and a recessed service pocket on
    the right (+X) side face."""
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
    inner = inner.translate((0.0, -WALL_TH / 2.0, 0.0))
    shell = outer.cut(inner)

    # Circular fan throat cut through the top (+Z) wall only.
    # Start below the inner cavity top face to ensure full cut-through.
    top_cut = cq.Solid.makeCylinder(
        FAN_BORE_R,
        WALL_TH + 0.010,
        pnt=cq.Vector(FAN_CENTER_X, FAN_CENTER_Y, BODY_H / 2.0 - WALL_TH - 0.002),
        dir=cq.Vector(0.0, 0.0, 1.0),
    )
    shell = shell.cut(cq.Workplane("XY").add(top_cut))

    # Shallow rectangular service pocket recessed into the right (+X) side face.
    pocket_depth = 0.022
    pocket = (
        cq.Workplane("YZ")
        .workplane(offset=BODY_W / 2.0 + 0.001)
        .center(0.0, 0.0)
        .rect(PANEL_W + 0.010, PANEL_H + 0.010)
        .extrude(-pocket_depth)
    )
    shell = shell.cut(pocket)

    return shell


def _fan_shroud_ring() -> cq.Workplane:
    """A short raised flow-ring (orifice) framing the fan opening on the top
    (+Z) face, so the circular opening reads as a real venturi rather than a
    raw hole."""
    z0 = BODY_H / 2.0 - 0.004  # start just inside the top wall
    outer = cq.Solid.makeCylinder(
        FAN_BORE_R + 0.020,
        0.026,
        pnt=cq.Vector(FAN_CENTER_X, FAN_CENTER_Y, z0),
        dir=cq.Vector(0.0, 0.0, 1.0),
    )
    bore = cq.Solid.makeCylinder(
        FAN_BORE_R,
        0.040,
        pnt=cq.Vector(FAN_CENTER_X, FAN_CENTER_Y, z0 - 0.005),
        dir=cq.Vector(0.0, 0.0, 1.0),
    )
    ring = cq.Workplane("XY").add(outer).cut(cq.Workplane("XY").add(bore))
    return ring


def _grille() -> cq.Workplane:
    """Protective wire fan grille: concentric rings plus radial diameter spokes
    plus a center boss, all welded into one solid, sitting just proud of the
    top face over the fan opening.

    Authored in a local frame where the grille lies in the XY plane (its tube
    axis along local +Z), centered on (0, 0, 0); the FIXED joint places this
    frame on the cabinet top face, with +Z pointing up above the fan."""
    z_front = 0.025  # standoff of the grille plane above the top face (+Z)

    solids: list[cq.Solid] = []

    # Concentric rings (tori) whose tube axis points along +Z.
    for i in range(GRILLE_RINGS):
        rr = FAN_BORE_R * (i + 1) / GRILLE_RINGS
        solids.append(
            cq.Solid.makeTorus(
                rr,
                GRILLE_WIRE_R,
                pnt=cq.Vector(0.0, 0.0, z_front),
                dir=cq.Vector(0.0, 0.0, 1.0),
            )
        )

    # Radial diameter spokes: rods crossing the full opening through center,
    # lying in the XY plane at height z_front.
    spoke_r = GRILLE_WIRE_R + 0.0015
    for k in range(GRILLE_SPOKES):
        ang = math.pi * k / GRILLE_SPOKES
        dx = math.cos(ang)
        dy = math.sin(ang)
        length = 2.0 * FAN_BORE_R
        solids.append(
            cq.Solid.makeCylinder(
                spoke_r,
                length,
                pnt=cq.Vector(-dx * FAN_BORE_R, -dy * FAN_BORE_R, z_front),
                dir=cq.Vector(dx, dy, 0.0),
            )
        )

    # Center boss / fan cap: a short cylinder along +Z at the hub, overlapping
    # all the spokes so the whole grille fuses into one connected island.
    solids.append(
        cq.Solid.makeCylinder(
            FAN_HUB_R * 0.9,
            0.016,
            pnt=cq.Vector(0.0, 0.0, z_front - 0.006),
            dir=cq.Vector(0.0, 0.0, 1.0),
        )
    )

    # Four mounting tabs at the rim that reach down (-Z) from the grille
    # plane into the raised shroud ring, fastening the grille to the housing.
    tab_r = FAN_BORE_R + 0.006  # at the shroud-ring wall radius
    for k in range(4):
        ang = math.pi / 4.0 + math.pi * k / 2.0
        cx = math.cos(ang) * tab_r
        cy = math.sin(ang) * tab_r
        solids.append(
            cq.Solid.makeCylinder(
                0.007,
                z_front - 0.002,  # from local z=0.002 (in the shroud) to the rim ring
                pnt=cq.Vector(cx, cy, 0.002),
                dir=cq.Vector(0.0, 0.0, 1.0),
            )
        )

    fused = solids[0]
    for s in solids[1:]:
        fused = fused.fuse(s)

    return cq.Workplane("XY").add(fused)


def _fan_motor_mount() -> cq.Workplane:
    """Fan motor and its cross-bracket mount, authored in the housing part frame.

    A motor can hangs below the top opening on a cross-shaped bracket that
    spans the fan throat and ties into the cabinet top wall, grounding the
    motor (and therefore the rotor, which seats on the motor shaft) to the
    housing."""
    cx = FAN_CENTER_X
    cy = FAN_CENTER_Y

    # Bracket sits just below the top wall inner face.
    bracket_z = BODY_H / 2.0 - WALL_TH  # 0.258

    # Motor can hangs below the bracket.
    motor_height = 0.065
    motor_top = bracket_z - 0.002  # slight overlap with bracket for clean fuse
    motor_bottom = motor_top - motor_height

    # Rotor center sits just below the top face inside the shroud zone.
    rotor_z = BODY_H / 2.0 - 0.018  # 0.252

    solids: list[cq.Solid] = []

    # Motor can (axis along +Z, hanging below the bracket).
    solids.append(
        cq.Solid.makeCylinder(
            0.038,
            motor_height,
            pnt=cq.Vector(cx, cy, motor_bottom),
            dir=cq.Vector(0.0, 0.0, 1.0),
        )
    )
    # Output shaft from inside the motor can up through the bracket to the rotor.
    shaft_bottom = motor_bottom + 0.020
    shaft_len = rotor_z - shaft_bottom + 0.010
    solids.append(
        cq.Solid.makeCylinder(
            0.008,
            shaft_len,
            pnt=cq.Vector(cx, cy, shaft_bottom),
            dir=cq.Vector(0.0, 0.0, 1.0),
        )
    )
    # Cross-bracket along X, spanning the top opening.
    strut_len = 2.0 * FAN_BORE_R + 0.020  # extends past the bore edge
    solids.append(
        cq.Solid.makeBox(
            strut_len,
            0.025,
            0.006,
            pnt=cq.Vector(cx - strut_len / 2.0, cy - 0.0125, bracket_z - 0.003),
        )
    )
    # Cross-bracket along Y, spanning the top opening.
    strut_len_y = 2.0 * FAN_BORE_R + 0.010
    solids.append(
        cq.Solid.makeBox(
            0.025,
            strut_len_y,
            0.006,
            pnt=cq.Vector(cx - 0.0125, cy - strut_len_y / 2.0, bracket_z - 0.003),
        )
    )

    fused = solids[0]
    for s in solids[1:]:
        fused = fused.fuse(s)
    return cq.Workplane("XY").add(fused)


def _fan_rotor_mesh():
    """Axial fan rotor that spins about local +Z (FanRotorGeometry convention).
    For the top-discharge variant, local +Z aligns with world +Z, so no
    reorientation is needed."""
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


def _bracket() -> cq.Workplane:
    """Steel wall-mounting bracket: two angle-iron legs under the body joined by
    a front and rear cross rail, forming the support frame seen at the base."""
    leg_w = BRACKET_ANGLE
    leg_t = 0.006
    rail_t = 0.006
    inset = 0.060

    parts = None

    def _add(wp):
        nonlocal parts
        parts = wp if parts is None else parts.union(wp)

    for sign in (-1.0, 1.0):
        x0 = sign * (BODY_W / 2.0 - inset)
        web = (
            cq.Workplane("XY")
            .box(leg_t, BODY_D + 0.040, BRACKET_LEG_H)
            .translate((x0, 0.0, -BRACKET_LEG_H / 2.0))
        )
        _add(web)
        foot = (
            cq.Workplane("XY")
            .box(leg_w, BODY_D + 0.040, leg_t)
            .translate((x0 + sign * leg_w / 2.0, 0.0, -BRACKET_LEG_H + leg_t / 2.0))
        )
        _add(foot)
        top = (
            cq.Workplane("XY")
            .box(leg_w, BODY_D + 0.040, leg_t)
            .translate((x0 + sign * leg_w / 2.0, 0.0, -leg_t / 2.0))
        )
        _add(top)

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

    base_x = BODY_W / 2.0 - 0.012
    for zc in (-0.205, -0.160):
        stub = cq.Solid.makeCylinder(
            0.009,
            0.060,
            pnt=cq.Vector(base_x, 0.030, zc),
            dir=cq.Vector(1.0, 0.0, 0.0),
        )
        _add(cq.Workplane("XY").add(stub))
        valve = cq.Solid.makeCylinder(
            0.015,
            0.026,
            pnt=cq.Vector(base_x, 0.030, zc),
            dir=cq.Vector(1.0, 0.0, 0.0),
        )
        _add(cq.Workplane("XY").add(valve))

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
    model = ArticulatedObject(name="ac_outdoor_unit_top_discharge")

    casing_mat = model.material("ac_casing", rgba=(0.86, 0.85, 0.82, 1.0))
    steel_mat = model.material("ac_steel", rgba=(0.35, 0.35, 0.37, 1.0))
    grille_mat = model.material("ac_grille", rgba=(0.20, 0.20, 0.21, 1.0))
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

    # --- Protective grille over the top fan opening ------------------------
    grille = model.part("fan_grille")
    grille.visual(
        mesh_from_cadquery(_grille(), "fan_grille"),
        material=grille_mat,
        name="grille_rings",
    )
    grille.inertial = Inertial.from_geometry(Box((0.28, 0.05, 0.28)), mass=0.5)

    # --- Axial fan rotor ---------------------------------------------------
    fan = model.part("fan_rotor")
    # FanRotorGeometry spins about local +Z; for the top-discharge variant
    # local +Z aligns with world +Z, so no rotation is needed.
    fan.visual(
        _fan_rotor_mesh(),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=fan_mat,
        name="fan_rotor",
    )
    fan.inertial = Inertial.from_geometry(Box((0.24, 0.24, 0.04)), mass=0.7)

    # --- Service / electrical access panel ---------------------------------
    panel = model.part("service_panel")
    panel.visual(
        mesh_from_cadquery(_service_panel(), "service_panel"),
        material=panel_mat,
        name="panel_cover",
    )
    panel.inertial = Inertial.from_geometry(
        Box((PANEL_TH, PANEL_H, PANEL_W)), mass=0.8
    )

    # ----- Articulations ---------------------------------------------------
    # Bracket is the fixed root. Housing bottom sits on the bracket top flange.
    model.articulation(
        "bracket_to_housing",
        ArticulationType.FIXED,
        parent=bracket,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
    )

    # Fan opening center on the top face, expressed in housing part frame:
    fan_x = FAN_CENTER_X
    fan_y = FAN_CENTER_Y
    fan_z = BODY_H / 2.0 - 0.018  # rotor center just below the top wall

    # Grille mounts on the housing top, fixed, centered on the fan opening.
    model.articulation(
        "housing_to_grille",
        ArticulationType.FIXED,
        parent=housing,
        child=grille,
        origin=Origin(xyz=(fan_x, fan_y, BODY_H / 2.0)),
    )

    # Copper service valves / lineset: fixed to the housing.
    model.articulation(
        "housing_to_service_valves",
        ArticulationType.FIXED,
        parent=housing,
        child=lineset,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # Axial fan: CONTINUOUS about the top (+Z) axis.
    model.articulation(
        "housing_to_fan",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=fan,
        origin=Origin(xyz=(fan_x, fan_y, fan_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=120.0),
    )

    # Service panel: REVOLUTE on a vertical (Z) hinge at the rear edge of the
    # right side service pocket.
    pocket_floor_x = BODY_W / 2.0 - 0.022
    panel_x = pocket_floor_x + PANEL_TH / 2.0
    panel_hinge_y = -(PANEL_W / 2.0)
    panel_z = 0.0
    model.articulation(
        "housing_to_service_panel",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=panel,
        origin=Origin(xyz=(panel_x, panel_hinge_y, panel_z)),
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
    panel = object_model.get_part("service_panel")
    valves = object_model.get_part("service_valves")

    fan_joint = object_model.get_articulation("housing_to_fan")
    panel_joint = object_model.get_articulation("housing_to_service_panel")

    # Intentional embeds:
    #  - copper valve stubs pass through / seat into the cabinet side wall.
    #  - the fan rotor hub seats on the motor shaft / motor can (shaft capture).
    #  - the grille mounting tabs seat into the raised shroud ring.
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
        reason="The grille mounting tabs seat into the raised shroud ring that frames the top fan opening.",
    )

    # --- Hero parts present ------------------------------------------------
    for name, prt in (
        ("mounting_bracket", bracket),
        ("housing", housing),
        ("fan_grille", grille),
        ("fan_rotor", fan),
        ("service_panel", panel),
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
        "fan_axis_top_z",
        abs(fan_joint.axis[2]) > 0.99 and abs(fan_joint.axis[0]) < 0.01,
        f"axis={fan_joint.axis}",
    )
    ctx.check(
        "panel_joint_revolute",
        str(panel_joint.articulation_type).endswith("REVOLUTE"),
        f"type={panel_joint.articulation_type}",
    )
    ctx.check(
        "panel_axis_vertical",
        abs(panel_joint.axis[2]) > 0.99,
        f"axis={panel_joint.axis}",
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

    # --- Fan rotor is round and sized to the top grille opening ------------
    fb = ctx.part_world_aabb(fan)
    if fb is not None:
        fmins, fmaxs = fb
        fsize = tuple(float(fmaxs[i] - fmins[i]) for i in range(3))
        diameter = max(fsize[0], fsize[1])
        ctx.check(
            "fan_diameter",
            abs(diameter - FAN_OUTER_R * 2.0) <= 0.03,
            f"fan size={fsize!r}",
        )
        # The fan disc should be thin along its spin axis (Z).
        ctx.check(
            "fan_thin_along_axis",
            fsize[2] <= 0.10,
            f"fan size={fsize!r}",
        )

    # --- Fan sits within the grille footprint on the top (XY) -------------
    ctx.expect_within(
        fan,
        grille,
        axes="xy",
        margin=0.02,
        name="fan_within_grille_footprint",
    )

    # --- Fan and grille are near the top of the housing -------------------
    if fb is not None and hb is not None:
        ctx.check(
            "fan_near_top_of_housing",
            fb[1][2] >= hb[1][2] - 0.06,
            f"fan_maxz={fb[1][2]}, housing_maxz={hb[1][2]}",
        )

    gb = ctx.part_world_aabb(grille)
    if gb is not None and hb is not None:
        ctx.check(
            "grille_on_top_of_housing",
            gb[0][2] >= hb[1][2] - 0.02,
            f"grille_minz={gb[0][2]}, housing_maxz={hb[1][2]}",
        )

    # --- Grille is above the fan along +Z ---------------------------------
    if gb is not None and fb is not None:
        ctx.check(
            "grille_above_fan",
            gb[1][2] >= fb[1][2] - 0.005,
            f"grille_maxz={gb[1][2]}, fan_maxz={fb[1][2]}",
        )

    # --- Grille is fastened to the shroud ring (mounting tabs seat in) -----
    ctx.expect_contact(
        grille,
        housing,
        elem_a="grille_rings",
        elem_b="fan_shroud_ring",
        contact_tol=0.001,
        name="grille_seated_on_shroud",
    )

    # --- Rotor hub is captured on the motor mount (grounds the rotor) ------
    ctx.expect_contact(
        fan,
        housing,
        elem_a="fan_rotor",
        elem_b="fan_motor_mount",
        contact_tol=0.001,
        name="rotor_on_motor_shaft",
    )

    # --- Fan actually spins: blade extents shift under rotation ------------
    tip_rest = ctx.part_element_world_aabb(fan, elem="fan_rotor")
    with ctx.pose({fan_joint: 0.6}):
        tip_spun = ctx.part_element_world_aabb(fan, elem="fan_rotor")
    if tip_rest is not None and tip_spun is not None:
        changed = (
            abs(tip_rest[1][0] - tip_spun[1][0]) > 0.02
            or abs(tip_rest[1][1] - tip_spun[1][1]) > 0.02
            or abs(tip_rest[0][0] - tip_spun[0][0]) > 0.02
            or abs(tip_rest[0][1] - tip_spun[0][1]) > 0.02
        )
        ctx.check("fan_rotation_moves_blades", changed, "Blade extents unchanged under spin.")

    # --- Service panel swings outward (+X) when opened ---------------------
    panel_closed = ctx.part_world_position(panel)
    panel_closed_aabb = ctx.part_world_aabb(panel)
    with ctx.pose({panel_joint: 1.4}):
        panel_open_aabb = ctx.part_world_aabb(panel)
        panel_open = ctx.part_world_position(panel)
    if panel_closed is not None and panel_open is not None:
        moved_out = panel_open_aabb[1][0] > panel_closed_aabb[1][0] + 0.05
        ctx.check(
            "panel_opens_outward",
            moved_out,
            f"closed_maxx={panel_closed_aabb[1][0]}, open_maxx={panel_open_aabb[1][0]}",
        )

    # --- Closed panel hugs the right side face -----------------------------
    if panel_closed_aabb is not None and hb is not None:
        ctx.check(
            "panel_seated_on_side",
            abs(panel_closed_aabb[0][0] - hb[1][0]) <= 0.03,
            f"panel_minx={panel_closed_aabb[0][0]}, housing_maxx={hb[1][0]}",
        )

    return ctx.report()


object_model = build_object_model()
