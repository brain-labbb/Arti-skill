from __future__ import annotations

# Industrial pendant control station (push-button hoist/crane control box).
#
# Real object identity (from the reference image): a small die-cast metal
# control box suspended on a vertical steel support rod. The front face carries
# two large round momentary push buttons (up/down hoist control). A small toggle
# switch and indicator detail sit on one side edge. The housing hangs from the
# rod through cable glands at the top and bottom.
#
# Coordinate convention used here:
#   +Z = up (the suspension rod runs along Z, the world up axis)
#   +X = front (toward the viewer; the button face points along +X)
#   +Y = right
#
# Articulated mechanism:
#   - each round push button is a PRISMATIC plunger that depresses into the
#     housing along -X (the real motion of a momentary push button)
#   - the side toggle switch is a small REVOLUTE lever
#   The suspension rod is the fixed root; the housing hangs from it.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
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

HOUSING_W = 0.058  # X depth of the body (front-to-back)
HOUSING_D = 0.066  # Y width (left-to-right)
HOUSING_H = 0.118  # Z height of the body
HOUSING_EDGE_R = 0.008  # rounded vertical edges of the cast body

FRONT_X = HOUSING_W / 2.0  # +X surface of the housing (button face)

# Button geometry
BUTTON_R = 0.0145  # outer collar radius of a push button
BUTTON_CAP_R = 0.0115  # round cap radius
BUTTON_COLLAR_H = 0.0050  # bezel collar height proud of the face
BUTTON_CAP_H = 0.0060  # cap height proud of the collar
BUTTON_TRAVEL = 0.0028  # plunger depression travel
BUTTON_Z_TOP = 0.028  # top button center, measured up from housing mid
BUTTON_Z_BOTTOM = -0.024  # bottom button center

# Suspension rod
ROD_R = 0.0042  # steel support rod radius
ROD_HALF_LEN = 0.150  # rod extends this far above and below housing center

# Side toggle switch
SWITCH_LEVER_LEN = 0.016
SWITCH_LEVER_R = 0.0022


# ---------------------------------------------------------------------------
# CadQuery geometry builders
# ---------------------------------------------------------------------------


def _build_housing_shell() -> cq.Workplane:
    """Die-cast control box body with rounded vertical edges and a slightly
    proud front bezel face. Hollow-read is conveyed by the recessed front panel
    and the cap/collar gland features rather than a solid block silhouette."""
    body = (
        cq.Workplane("YZ")  # section in YZ, extrude along X
        .rect(HOUSING_D, HOUSING_H)
        .extrude(HOUSING_W)
        .translate((-HOUSING_W / 2.0, 0.0, 0.0))
        .edges("|X")
        .fillet(HOUSING_EDGE_R)
    )
    # Soften the top and bottom rim edges so it reads as a cast enclosure.
    body = body.edges(">X or <X").fillet(0.0025)

    # Recessed front panel pocket so the face reads as a bezel, not a flat slab.
    pocket = (
        cq.Workplane("YZ")
        .rect(HOUSING_D - 0.014, HOUSING_H - 0.018)
        .extrude(0.004)
        .translate((FRONT_X - 0.004 + 0.0001, 0.0, 0.0))
        .edges("|X")
        .fillet(0.006)
    )
    body = body.cut(pocket)

    # Four corner mounting bosses on the front face (cast screw bosses).
    boss_h = 0.0035
    for sy in (-1.0, 1.0):
        for sz in (-1.0, 1.0):
            boss = (
                cq.Workplane("YZ")
                .circle(0.0045)
                .extrude(boss_h)
                .translate(
                    (
                        FRONT_X - 0.004,
                        sy * (HOUSING_D / 2.0 - 0.009),
                        sz * (HOUSING_H / 2.0 - 0.011),
                    )
                )
            )
            body = body.union(boss)
    return body


def _build_cable_gland(length: float) -> cq.Workplane:
    """A threaded-look cable gland: a stepped boss with a hex-ish ring through
    which the suspension rod passes. Built along +Z."""
    gland = (
        cq.Workplane("XY")
        .circle(0.0085)
        .extrude(length * 0.55)
        .faces(">Z")
        .workplane()
        .circle(0.0065)
        .extrude(length * 0.45)
    )
    # A captive nut ring near the base.
    nut = (
        cq.Workplane("XY")
        .polygon(6, 0.020)
        .extrude(0.0045)
        .translate((0.0, 0.0, 0.0015))
    )
    return gland.union(nut)


def _build_button_cap() -> cq.Workplane:
    """A round momentary push-button: a bezel collar ring plus a domed cap.
    Built along +X so it depresses into the housing along -X.
    The mesh local frame has the collar base at x=0, cap proud toward +X."""
    collar = (
        cq.Workplane("YZ")
        .circle(BUTTON_R)
        .extrude(BUTTON_COLLAR_H)
        .faces(">X")
        .edges()
        .chamfer(0.0012)
    )
    cap = (
        cq.Workplane("YZ")
        .circle(BUTTON_CAP_R)
        .extrude(BUTTON_CAP_H)
        .translate((BUTTON_COLLAR_H, 0.0, 0.0))
        .faces(">X")
        .fillet(0.004)
    )
    # Small finger-grip recess on the cap top.
    body = collar.union(cap)
    return body


def _build_switch_lever() -> cq.Workplane:
    """Short cylindrical toggle lever with a rounded tip. Built along +Z (its
    pivot is at z=0, the lever points up); it gets oriented when placed."""
    lever = (
        cq.Workplane("XY")
        .circle(SWITCH_LEVER_R)
        .extrude(SWITCH_LEVER_LEN)
    )
    tip = (
        cq.Workplane("XY")
        .sphere(SWITCH_LEVER_R * 1.4)
        .translate((0.0, 0.0, SWITCH_LEVER_LEN))
    )
    base = (
        cq.Workplane("XY")
        .circle(SWITCH_LEVER_R * 2.0)
        .extrude(0.0030)
        .translate((0.0, 0.0, -0.0030))
    )
    return lever.union(tip).union(base)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pendant_control_station")

    cast_metal = model.material("cast_metal", rgba=(0.30, 0.31, 0.33, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.18, 0.19, 0.20, 1.0))
    steel = model.material("steel", rgba=(0.55, 0.56, 0.58, 1.0))
    button_black = model.material("button_black", rgba=(0.09, 0.09, 0.10, 1.0))
    bezel_grey = model.material("bezel_grey", rgba=(0.24, 0.25, 0.27, 1.0))

    # --- Root: the vertical suspension rod ---------------------------------
    rod = model.part("suspension_rod")
    rod.visual(
        Cylinder(radius=ROD_R, length=2.0 * ROD_HALF_LEN),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=steel,
        name="rod_shaft",
    )
    rod.inertial = Inertial.from_geometry(
        Cylinder(radius=ROD_R, length=2.0 * ROD_HALF_LEN),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # --- Housing -----------------------------------------------------------
    housing = model.part("housing")
    housing_mesh = mesh_from_cadquery(_build_housing_shell(), "housing_shell")
    housing.visual(housing_mesh, material=cast_metal, name="housing_shell")

    # Front bezel insert (the recessed dark face the buttons sit on).
    housing.visual(
        Box((0.0035, HOUSING_D - 0.016, HOUSING_H - 0.020)),
        origin=Origin(xyz=(FRONT_X - 0.0035, 0.0, 0.002)),
        material=bezel_grey,
        name="front_bezel",
    )

    # Top and bottom cable glands where the rod enters the housing.
    top_gland_mesh = mesh_from_cadquery(_build_cable_gland(0.024), "top_gland")
    housing.visual(
        top_gland_mesh,
        origin=Origin(xyz=(0.0, 0.0, HOUSING_H / 2.0 - 0.002)),
        material=dark_metal,
        name="top_gland",
    )
    bottom_gland_mesh = mesh_from_cadquery(_build_cable_gland(0.024), "bottom_gland")
    housing.visual(
        bottom_gland_mesh,
        origin=Origin(xyz=(0.0, 0.0, -(HOUSING_H / 2.0 - 0.002) - 0.024), rpy=(0.0, 0.0, 0.0)),
        material=dark_metal,
        name="bottom_gland",
    )

    # Small side indicator/marker block and switch escutcheon on the left edge.
    housing.visual(
        Box((0.010, 0.006, 0.034)),
        origin=Origin(xyz=(FRONT_X - 0.014, -(HOUSING_D / 2.0) + 0.0005, 0.004)),
        material=dark_metal,
        name="side_escutcheon",
    )

    housing.inertial = Inertial.from_geometry(
        Box((HOUSING_W, HOUSING_D, HOUSING_H)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # Fix the housing onto the rod (housing hangs from the rod = the root).
    model.articulation(
        "rod_to_housing",
        ArticulationType.FIXED,
        parent=rod,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # --- Push buttons (prismatic plungers) ---------------------------------
    # The cap mesh is built along +X with its collar base at x=0. The joint
    # frame is placed at the bezel face; the button geometry sits proud along
    # +X and depresses along -X.
    bezel_face_x = FRONT_X - 0.0035  # front surface of the recessed bezel insert

    for label, button_z in (("top", BUTTON_Z_TOP), ("bottom", BUTTON_Z_BOTTOM)):
        button = model.part(f"{label}_button")
        cap_mesh = mesh_from_cadquery(_build_button_cap(), f"{label}_button_cap")
        # Collar ring base on the bezel face (x=0 in the part frame), cap proud.
        button.visual(
            cap_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=button_black,
            name="button_cap",
        )
        button.inertial = Inertial.from_geometry(
            Cylinder(radius=BUTTON_R, length=BUTTON_COLLAR_H + BUTTON_CAP_H),
            mass=0.02,
            origin=Origin(
                xyz=((BUTTON_COLLAR_H + BUTTON_CAP_H) / 2.0, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
        )
        model.articulation(
            f"housing_to_{label}_button",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=button,
            # Joint frame at the bezel face, centered on the button.
            origin=Origin(xyz=(bezel_face_x, 0.0, button_z)),
            # Positive q presses the button inward (-X).
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=10.0,
                velocity=0.05,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    # --- Side toggle switch (revolute lever) -------------------------------
    switch = model.part("side_switch")
    lever_mesh = mesh_from_cadquery(_build_switch_lever(), "switch_lever")
    # Mesh built along +Z (pivot at z=0). We orient the lever to point outward
    # to the -Y side and slightly up, then the revolute axis flips it.
    switch.visual(
        lever_mesh,
        # Rotate so the lever points along -Y (out the left side), pivot at frame origin.
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="switch_lever",
    )
    switch.inertial = Inertial.from_geometry(
        Cylinder(radius=SWITCH_LEVER_R, length=SWITCH_LEVER_LEN),
        mass=0.004,
        origin=Origin(xyz=(0.0, -SWITCH_LEVER_LEN / 2.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    # Pivot on the left edge of the housing, mid-low on the escutcheon.
    switch_pivot = Origin(xyz=(FRONT_X - 0.014, -(HOUSING_D / 2.0) + 0.0015, 0.004))
    model.articulation(
        "housing_to_side_switch",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=switch,
        origin=switch_pivot,
        # Lever points along -Y; rotating about -X swings the tip up for +q (toggle up).
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=-0.5,
            upper=0.5,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    rod = object_model.get_part("suspension_rod")
    housing = object_model.get_part("housing")
    top_button = object_model.get_part("top_button")
    bottom_button = object_model.get_part("bottom_button")
    switch = object_model.get_part("side_switch")

    # The suspension rod intentionally passes captive through the housing and its
    # top/bottom cable glands (the box hangs on the rod). Allow that nesting for
    # each element the rod threads through.
    for housing_elem in ("housing_shell", "top_gland", "bottom_gland"):
        ctx.allow_overlap(
            housing,
            rod,
            elem_a=housing_elem,
            elem_b="rod_shaft",
            reason="The suspension rod passes captive through the housing and cable glands.",
        )
    # Prove the rod stays centered inside the glands (retained insertion).
    ctx.expect_within(
        rod,
        housing,
        axes="xy",
        inner_elem="rod_shaft",
        outer_elem="top_gland",
        margin=0.001,
        name="rod stays centered in the top gland",
    )
    ctx.expect_within(
        rod,
        housing,
        axes="xy",
        inner_elem="rod_shaft",
        outer_elem="bottom_gland",
        margin=0.001,
        name="rod stays centered in the bottom gland",
    )

    top_joint = object_model.get_articulation("housing_to_top_button")
    bottom_joint = object_model.get_articulation("housing_to_bottom_button")
    switch_joint = object_model.get_articulation("housing_to_side_switch")
    rod_joint = object_model.get_articulation("rod_to_housing")

    # --- Joint type / axis claims -----------------------------------------
    ctx.check(
        "top button is prismatic",
        top_joint.joint_type == ArticulationType.PRISMATIC,
        details=f"got {top_joint.joint_type}",
    )
    ctx.check(
        "bottom button is prismatic",
        bottom_joint.joint_type == ArticulationType.PRISMATIC,
        details=f"got {bottom_joint.joint_type}",
    )
    ctx.check(
        "side switch is revolute",
        switch_joint.joint_type == ArticulationType.REVOLUTE,
        details=f"got {switch_joint.joint_type}",
    )
    ctx.check(
        "housing fixed to rod",
        rod_joint.joint_type == ArticulationType.FIXED,
        details=f"got {rod_joint.joint_type}",
    )
    # Buttons press into the face: axis is along -X.
    ctx.check(
        "top button axis is -X",
        abs(top_joint.axis[0]) > 0.99 and abs(top_joint.axis[1]) < 0.01,
        details=f"axis={top_joint.axis}",
    )

    # --- Hero parts present and placed ------------------------------------
    # Buttons sit on the front (+X) face, top above bottom.
    top_pos = ctx.part_world_position(top_button)
    bottom_pos = ctx.part_world_position(bottom_button)
    ctx.check(
        "top button is above bottom button",
        top_pos is not None and bottom_pos is not None and top_pos[2] > bottom_pos[2] + 0.03,
        details=f"top={top_pos}, bottom={bottom_pos}",
    )
    ctx.check(
        "buttons are on the front (+X) face",
        top_pos is not None and bottom_pos is not None and top_pos[0] > 0.0 and bottom_pos[0] > 0.0,
        details=f"top={top_pos}, bottom={bottom_pos}",
    )

    # The rod is the root and spans well above and below the housing.
    rod_aabb = ctx.part_world_aabb(rod)
    housing_aabb = ctx.part_world_aabb(housing)
    ctx.check(
        "rod extends above and below housing",
        rod_aabb is not None
        and housing_aabb is not None
        and rod_aabb[1][2] > housing_aabb[1][2] + 0.02
        and rod_aabb[0][2] < housing_aabb[0][2] - 0.02,
        details=f"rod={rod_aabb}, housing={housing_aabb}",
    )

    # Buttons sit proud of the bezel face at rest: the cap front pokes out well
    # past the bezel front surface (collar seats into the bezel, cap protrudes).
    cap_aabb = ctx.part_element_world_aabb(top_button, elem="button_cap")
    bezel_aabb = ctx.part_element_world_aabb(housing, elem="front_bezel")
    ctx.check(
        "top button cap protrudes past the bezel",
        cap_aabb is not None
        and bezel_aabb is not None
        and cap_aabb[1][0] > bezel_aabb[1][0] + 0.004,
        details=f"cap_max_x={cap_aabb[1][0] if cap_aabb else None}, "
        f"bezel_max_x={bezel_aabb[1][0] if bezel_aabb else None}",
    )

    # --- Mechanism actuation: pressing moves the cap inward (-X) ------------
    rest_top = ctx.part_world_position(top_button)
    with ctx.pose({top_joint: BUTTON_TRAVEL}):
        pressed_top = ctx.part_world_position(top_button)
        ctx.check(
            "pressing top button moves it inward (-X)",
            rest_top is not None
            and pressed_top is not None
            and pressed_top[0] < rest_top[0] - 0.0015,
            details=f"rest={rest_top}, pressed={pressed_top}",
        )

    # --- Switch lever swings about its pivot -------------------------------
    rest_switch_aabb = ctx.part_world_aabb(switch)
    with ctx.pose({switch_joint: 0.45}):
        up_switch_aabb = ctx.part_world_aabb(switch)
        ctx.check(
            "toggling the side switch raises the lever tip",
            rest_switch_aabb is not None
            and up_switch_aabb is not None
            and up_switch_aabb[1][2] > rest_switch_aabb[1][2] + 0.002,
            details=f"rest={rest_switch_aabb}, up={up_switch_aabb}",
        )

    # Switch lever stays attached to the left (-Y) side of the housing.
    switch_pos = ctx.part_world_position(switch)
    ctx.check(
        "side switch is on the left (-Y) edge",
        switch_pos is not None and switch_pos[1] < 0.0,
        details=f"switch={switch_pos}",
    )

    return ctx.report()


object_model = build_object_model()
