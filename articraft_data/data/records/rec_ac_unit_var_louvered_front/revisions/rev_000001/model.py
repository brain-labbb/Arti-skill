from __future__ import annotations

# Air conditioner outdoor unit (condensing unit), wall-mounted.
# Variant 2: rectangular louvered grille with horizontal slats over the fan.
#
# Reference image: a weathered white/rusty steel condensing-unit box mounted on a
# bottom steel angle-iron bracket against a wall. The front-left face carries a
# rectangular louvered grille with repeated horizontal slats covering the fan
# opening, with an axial fan spinning behind it. The right side has a recessed
# service/electrical access panel and exposed copper refrigerant lines / service
# valves.
#
# Articulated mechanisms (reasoned from physics):
#   * housing_to_fan        -> CONTINUOUS axial fan spinning about the front (+Y) axis.
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
    VentGrilleFrame,
    VentGrilleGeometry,
    VentGrilleSlats,
    VentGrilleSleeve,
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

# Rectangular louvered grille dimensions (variant: horizontal slats over the fan).
GRILLE_PANEL_W = 0.460  # grille panel width (along X, covers fan opening + margin)
GRILLE_PANEL_H = 0.440  # grille panel height (along Z in world)
GRILLE_FRAME = 0.014  # outer frame border width
GRILLE_FACE_TH = 0.004  # face plate thickness behind the slats
GRILLE_DUCT_DEPTH = 0.028  # short rear sleeve depth for mounting into the shroud
GRILLE_SLAT_PITCH = 0.022  # vertical spacing between horizontal slats
GRILLE_SLAT_WIDTH = 0.012  # slat chord width
GRILLE_SLAT_ANGLE = 35.0  # slat louver angle (degrees)

PANEL_W = 0.170  # service access panel (on the +X / right side face)
PANEL_H = 0.300
PANEL_TH = 0.010
PANEL_RECESS = 0.010  # how far the panel sits proud of the recessed pocket

BRACKET_LEG_H = 0.150  # height of the wall bracket below the body
BRACKET_ANGLE = 0.035  # angle-iron leg width


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


def _louver_grille_geometry() -> VentGrilleGeometry:
    """Rectangular louvered grille with repeated horizontal slats over the fan.

    Variant replacement for the parent's circular wire grille. The grille is a
    framed vent panel built by ``VentGrilleGeometry`` with horizontal flat slats
    angled downward at 35°. A short rear sleeve seats into the housing shroud
    ring for mechanical retention.

    The VentGrilleGeometry lies in local XY, extending along +Z. The visual
    origin rotates it so the face sits in the XZ plane (vertical) facing +Y
    (outward from the cabinet front), with the sleeve extending along -Y into
    the housing."""
    return VentGrilleGeometry(
        (GRILLE_PANEL_W, GRILLE_PANEL_H),
        frame=GRILLE_FRAME,
        face_thickness=GRILLE_FACE_TH,
        duct_depth=GRILLE_DUCT_DEPTH,
        duct_wall=0.003,
        slat_pitch=GRILLE_SLAT_PITCH,
        slat_width=GRILLE_SLAT_WIDTH,
        slat_angle_deg=GRILLE_SLAT_ANGLE,
        corner_radius=0.005,
        slats=VentGrilleSlats(
            profile="flat",
            direction="down",
            divider_count=1,
            divider_width=0.005,
        ),
        frame_profile=VentGrilleFrame(style="radiused", depth=0.002),
        sleeve=VentGrilleSleeve(style="short"),
        center=True,
    )


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

    # --- Rectangular louvered grille over the fan opening -------------------
    grille = model.part("fan_grille")
    # VentGrilleGeometry lies in local XY extending along Z. Rotate so the face
    # sits in the XZ plane facing +Y, with the sleeve extending along -Y.
    grille.visual(
        mesh_from_geometry(_louver_grille_geometry(), "louver_grille"),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=grille_mat,
        name="louver_panel",
    )
    grille.inertial = Inertial.from_geometry(
        Box((GRILLE_PANEL_W, GRILLE_DUCT_DEPTH + GRILLE_FACE_TH, GRILLE_PANEL_H)),
        mass=0.8,
    )

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

    # Service panel: REVOLUTE on a vertical (Z) hinge at the rear edge of the
    # right side service pocket. The closed panel seats in the recessed pocket
    # and extends along +Y from the hinge; axis -Z makes positive q swing the
    # free (+Y) edge outward in +X, away from the cabinet.
    pocket_floor_x = BODY_W / 2.0 - 0.022  # matches _housing_shell pocket depth
    panel_x = pocket_floor_x + PANEL_TH / 2.0
    panel_hinge_y = -(PANEL_W / 2.0)  # hinge at the rear (-Y) edge of the panel
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
        elem_a="louver_panel",
        elem_b="fan_shroud_ring",
        reason="The rectangular louver grille rear sleeve seats into the raised shroud ring that frames the fan opening.",
    )
    ctx.allow_overlap(
        grille,
        housing,
        elem_a="louver_panel",
        elem_b="housing_shell",
        reason="The rectangular louver panel face-mounts against the housing front wall; its rectangular frame and sleeve corners extend beyond the circular fan opening cutout.",
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
        "fan_axis_front_y",
        abs(fan_joint.axis[1]) > 0.99 and abs(fan_joint.axis[0]) < 0.01,
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
        # The fan disc should be thin along its spin axis (Y).
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
        elem_a="louver_panel",
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
    # Use an angle off the 5-fold blade symmetry (72 deg) so the rotor's bounding
    # box demonstrably changes when the joint is actuated.
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

    # --- Variant: rectangular louvered grille (not circular wire grille) ----
    # The louver_panel visual must exist on the grille part.
    louver_vis = grille.get_visual("louver_panel")
    ctx.check("louver_panel_visual_present", louver_vis is not None, "Missing louver_panel visual on fan_grille.")

    # The grille panel should be rectangular with real width and height.
    grl_aabb = ctx.part_element_world_aabb(grille, elem="louver_panel")
    if grl_aabb is not None:
        grl_mins, grl_maxs = grl_aabb
        grl_dx = float(grl_maxs[0] - grl_mins[0])  # width (X)
        grl_dy = float(grl_maxs[1] - grl_mins[1])  # depth (Y, thin panel + sleeve)
        grl_dz = float(grl_maxs[2] - grl_mins[2])  # height (Z)
        ctx.check(
            "louver_panel_rectangular",
            grl_dx > 0.35 and grl_dz > 0.35,
            f"grille panel size x={grl_dx:.3f} z={grl_dz:.3f}",
        )
        # Panel should be thin along Y (face + sleeve, not a solid block).
        ctx.check(
            "louver_panel_thin_depth",
            grl_dy < 0.08,
            f"grille panel depth y={grl_dy:.3f}",
        )
        # The rectangular grille should have roughly square proportions
        # (width ~ height) since it covers the circular fan opening.
        aspect = grl_dx / grl_dz if grl_dz > 0.01 else 0.0
        ctx.check(
            "louver_panel_aspect_near_square",
            0.85 <= aspect <= 1.15,
            f"grille aspect w/h={aspect:.2f}",
        )

    # --- Service panel swings outward (+X) when opened ---------------------
    panel_closed = ctx.part_world_position(panel)
    panel_closed_aabb = ctx.part_world_aabb(panel)
    with ctx.pose({panel_joint: 1.4}):
        panel_open_aabb = ctx.part_world_aabb(panel)
        panel_open = ctx.part_world_position(panel)
    if panel_closed is not None and panel_open is not None:
        # Opening should push the free edge out in +X (away from the cabinet side).
        moved_out = panel_open_aabb[1][0] > panel_closed_aabb[1][0] + 0.05
        ctx.check(
            "panel_opens_outward",
            moved_out,
            f"closed_maxx={panel_closed_aabb[1][0]}, open_maxx={panel_open_aabb[1][0]}",
        )

    # --- Closed panel hugs the right side face -----------------------------
    if panel_closed_aabb is not None and hb is not None:
        # panel inner face should be near the housing +X face.
        ctx.check(
            "panel_seated_on_side",
            abs(panel_closed_aabb[0][0] - hb[1][0]) <= 0.03,
            f"panel_minx={panel_closed_aabb[0][0]}, housing_maxx={hb[1][0]}",
        )

    return ctx.report()


object_model = build_object_model()
