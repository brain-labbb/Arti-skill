from __future__ import annotations

# Industrial pendant control station (push-button hoist/crane control box).
#
# Real object identity (from the reference image): a small die-cast metal
# control box suspended on a vertical steel support rod. The front face carries
# one large mushroom emergency-stop plunger on the front face. A small toggle
# switch and indicator detail sit on one side edge. The housing hangs from the
# rod through cable glands at the top and bottom.
#
# Coordinate convention used here:
#   +Z = up (the suspension rod runs along Z, the world up axis)
#   +X = front (toward the viewer; the button face points along +X)
#   +Y = right
#
# Articulated mechanism:
#   - the mushroom emergency stop is a deep-travel PRISMATIC latching plunger
#     that depresses into the housing along -X
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

# Emergency-stop geometry
ESTOP_CENTER_Z = 0.006  # centered between the former two button locations
ESTOP_GUARD_R = 0.0265  # yellow safety collar, almost the full face width
ESTOP_GUARD_H = 0.0060
ESTOP_GASKET_R = 0.0185
ESTOP_GASKET_H = 0.0040
ESTOP_CAP_R = 0.0245  # broad mushroom cap radius
ESTOP_CAP_H = 0.0250  # tall cap height proud of the guard
ESTOP_TRAVEL = 0.0120  # deep latching plunger depression travel

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


def _build_estop_guard() -> cq.Workplane:
    """Shallow yellow emergency-stop safety collar, built along +X with its
    mounting face at x=0 so the joint origin can sit on the visible front panel."""
    guard = (
        cq.Workplane("YZ")
        .circle(ESTOP_GUARD_R)
        .extrude(ESTOP_GUARD_H)
        .edges(">X or <X")
        .chamfer(0.0012)
    )
    return guard


def _build_estop_gasket() -> cq.Workplane:
    """Black retaining/gasket ring between the yellow collar and red mushroom."""
    gasket = (
        cq.Workplane("YZ")
        .circle(ESTOP_GASKET_R)
        .extrude(ESTOP_GASKET_H)
        .edges(">X or <X")
        .chamfer(0.0008)
    )
    return gasket


def _build_estop_mushroom_cap() -> cq.Workplane:
    """Lathe-style red mushroom cap revolved around the local X axis.

    The profile has a narrow rear hub that flares into a broad skirt, then a
    high rounded crown. Local x=0 is the cap's rear face; the cap sits on top of
    the guard and moves with the prismatic latching plunger.
    """
    # A dense polyline profile is more robust than a free spline here while
    # still reading as a smooth lathed dome after revolution.
    profile_points = [
        (0.0000, 0.0000),
        (0.0000, 0.0130),
        (0.0020, 0.0170),
        (0.0050, 0.0215),
        (0.0090, ESTOP_CAP_R),
        (0.0150, ESTOP_CAP_R),
        (0.0185, 0.0230),
        (0.0215, 0.0185),
        (0.0235, 0.0110),
        (ESTOP_CAP_H, 0.0000),
    ]
    profile = cq.Workplane("XZ").polyline(profile_points).close()
    return profile.revolve(360.0, axisStart=(0.0, 0.0, 0.0), axisEnd=(1.0, 0.0, 0.0))


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
    emergency_red = model.material("emergency_red", rgba=(0.78, 0.03, 0.025, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.95, 0.70, 0.05, 1.0))
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

    # --- Emergency stop (deep prismatic latching mushroom) ------------------
    # The button assembly is built along +X with the yellow guard's rear face at
    # x=0. The joint frame sits directly on the visible recessed front panel;
    # positive travel moves the whole mushroom inward along -X.
    bezel_face_x = FRONT_X - 0.00175  # front surface of the recessed bezel insert

    emergency_stop = model.part("emergency_stop")
    guard_mesh = mesh_from_cadquery(_build_estop_guard(), "estop_guard")
    gasket_mesh = mesh_from_cadquery(_build_estop_gasket(), "estop_gasket")
    cap_mesh = mesh_from_cadquery(_build_estop_mushroom_cap(), "estop_mushroom_cap")
    emergency_stop.visual(
        guard_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=safety_yellow,
        name="safety_guard",
    )
    emergency_stop.visual(
        gasket_mesh,
        origin=Origin(xyz=(ESTOP_GUARD_H - 0.0010, 0.0, 0.0)),
        material=button_black,
        name="retaining_gasket",
    )
    emergency_stop.visual(
        cap_mesh,
        origin=Origin(xyz=(ESTOP_GUARD_H - 0.0005, 0.0, 0.0)),
        material=emergency_red,
        name="mushroom_cap",
    )
    emergency_stop.inertial = Inertial.from_geometry(
        Cylinder(radius=ESTOP_GUARD_R, length=ESTOP_GUARD_H + ESTOP_CAP_H),
        mass=0.06,
        origin=Origin(
            xyz=((ESTOP_GUARD_H + ESTOP_CAP_H) / 2.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
    )
    model.articulation(
        "housing_to_emergency_stop",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=emergency_stop,
        # Joint frame at the actual contact face of the front bezel.
        origin=Origin(xyz=(bezel_face_x, 0.0, ESTOP_CENTER_Z)),
        # Positive q latches the mushroom inward (-X).
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.12,
            lower=0.0,
            upper=ESTOP_TRAVEL,
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
    emergency_stop = object_model.get_part("emergency_stop")
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

    emergency_joint = object_model.get_articulation("housing_to_emergency_stop")
    switch_joint = object_model.get_articulation("housing_to_side_switch")
    rod_joint = object_model.get_articulation("rod_to_housing")

    # --- Joint type / axis claims -----------------------------------------
    ctx.check(
        "emergency stop is prismatic",
        emergency_joint.joint_type == ArticulationType.PRISMATIC,
        details=f"got {emergency_joint.joint_type}",
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
    # Emergency stop presses into the face: axis is along -X.
    ctx.check(
        "emergency stop axis is -X",
        emergency_joint.axis[0] < -0.99
        and abs(emergency_joint.axis[1]) < 0.01
        and abs(emergency_joint.axis[2]) < 0.01,
        details=f"axis={emergency_joint.axis}",
    )
    ctx.check(
        "emergency stop has deep latching travel",
        emergency_joint.motion_limits is not None
        and emergency_joint.motion_limits.upper >= 0.010
        and emergency_joint.motion_limits.lower == 0.0,
        details=f"limits={emergency_joint.motion_limits}",
    )

    # --- Hero parts present and placed ------------------------------------
    # The targeted fork replaces the two former flat round push-buttons with
    # one larger mushroom emergency stop; rod and side switch remain present.
    part_names = {part.name for part in object_model.parts}
    ctx.check(
        "single emergency stop replaces two flat push-buttons",
        "emergency_stop" in part_names
        and "top_button" not in part_names
        and "bottom_button" not in part_names,
        details=f"parts={sorted(part_names)}",
    )
    emergency_pos = ctx.part_world_position(emergency_stop)
    ctx.check(
        "emergency stop is on the front (+X) face",
        emergency_pos is not None and emergency_pos[0] > 0.0 and abs(emergency_pos[1]) < 0.004,
        details=f"emergency_stop={emergency_pos}",
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

    # The yellow guard is seated on the front bezel and the red cap is a broad,
    # tall mushroom that protrudes well past the panel.
    cap_aabb = ctx.part_element_world_aabb(emergency_stop, elem="mushroom_cap")
    guard_aabb = ctx.part_element_world_aabb(emergency_stop, elem="safety_guard")
    bezel_aabb = ctx.part_element_world_aabb(housing, elem="front_bezel")
    ctx.expect_gap(
        emergency_stop,
        housing,
        axis="x",
        max_gap=0.0006,
        max_penetration=0.000001,
        positive_elem="safety_guard",
        negative_elem="front_bezel",
        name="emergency guard seats on the front bezel face",
    )
    ctx.check(
        "mushroom cap protrudes far past the bezel",
        cap_aabb is not None
        and bezel_aabb is not None
        and cap_aabb[1][0] > bezel_aabb[1][0] + 0.024,
        details=f"cap_max_x={cap_aabb[1][0] if cap_aabb else None}, "
        f"bezel_max_x={bezel_aabb[1][0] if bezel_aabb else None}",
    )
    ctx.check(
        "mushroom cap is broad and domed",
        cap_aabb is not None
        and (cap_aabb[1][1] - cap_aabb[0][1]) > 0.046
        and (cap_aabb[1][2] - cap_aabb[0][2]) > 0.046
        and (cap_aabb[1][0] - cap_aabb[0][0]) > 0.023,
        details=f"cap_aabb={cap_aabb}",
    )
    ctx.check(
        "yellow safety guard surrounds the cap",
        guard_aabb is not None
        and cap_aabb is not None
        and (guard_aabb[1][1] - guard_aabb[0][1]) > (cap_aabb[1][1] - cap_aabb[0][1]) + 0.002,
        details=f"guard_aabb={guard_aabb}, cap_aabb={cap_aabb}",
    )

    # --- Mechanism actuation: pressing moves the mushroom inward (-X) --------
    rest_stop = ctx.part_world_position(emergency_stop)
    with ctx.pose({emergency_joint: ESTOP_TRAVEL}):
        pressed_stop = ctx.part_world_position(emergency_stop)
        ctx.check(
            "pressing emergency stop moves it deeply inward (-X)",
            rest_stop is not None
            and pressed_stop is not None
            and pressed_stop[0] < rest_stop[0] - 0.010,
            details=f"rest={rest_stop}, pressed={pressed_stop}",
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
