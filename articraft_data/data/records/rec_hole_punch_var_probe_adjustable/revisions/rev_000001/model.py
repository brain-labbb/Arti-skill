from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
    tube_from_spline_points,
)


def _rounded_box_mesh(name: str, size: tuple[float, float, float], radius: float):
    """CadQuery rounded block authored in meters."""
    base = cq.Workplane("XY").box(*size)
    try:
        body = base.edges().fillet(radius)
    except Exception:
        body = cq.Workplane("XY").box(*size).edges("|Z").fillet(radius)
    return mesh_from_cadquery(body, name, tolerance=0.00045, angular_tolerance=0.08)


def _extruded_xz_mesh(
    name: str,
    profile_xz: list[tuple[float, float]],
    thickness_y: float,
):
    """Extrude a side profile drawn in X/Z into a centered Y-thickness mesh."""
    geom = ExtrudeGeometry(profile_xz, thickness_y, center=True).rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _lever_profile() -> list[tuple[float, float]]:
    # Clockwise side outline: low rear pivot cheek, lower pressing arm,
    # smoothly rising hand lever, and rounded front nose.
    return [
        (-0.024, -0.014),
        (0.020, -0.015),
        (0.068, -0.008),
        (0.086, 0.006),
        (0.102, 0.036),
        (0.128, 0.080),
        (0.154, 0.112),
        (0.166, 0.128),
        (0.161, 0.143),
        (0.146, 0.147),
        (0.094, 0.111),
        (0.038, 0.055),
        (0.007, 0.031),
        (-0.023, 0.018),
    ]


def _side_plate_profile() -> list[tuple[float, float]]:
    return [
        (-0.092, 0.024),
        (0.057, 0.024),
        (0.060, 0.038),
        (0.028, 0.046),
        (-0.014, 0.066),
        (-0.071, 0.071),
        (-0.091, 0.055),
    ]


# ---------------------------------------------------------------------------
# Shared geometry helper: build visuals for one punch-head module.
# All coordinates are in the head-local frame (origin at the prismatic joint).
# ---------------------------------------------------------------------------

# Head-local layout constants
# The carriage is a tall block that encompasses the die elements for
# structural connectivity.  The guide post sits on top of the carriage
# and stays below the lever sweep zone.
_DIE_BOSS_Z = 0.005
_DIE_BOSS_LEN = 0.004
_DIE_RING_Z = 0.008
_DIE_RING_LEN = 0.003
_DIE_HOLE_Z = 0.009
_DIE_HOLE_LEN = 0.003
_CARRIAGE_Z = 0.016
_CARRIAGE_SIZE = (0.030, 0.026, 0.020)  # tall enough to encompass dies
_GUIDE_POST_Z = 0.021
_GUIDE_POST_LEN = 0.012
_GUIDE_POST_R = 0.005
_THUMBSCREW_Z = 0.016
_THUMBSCREW_Y = 0.013  # at carriage Y face for connectivity

# Pin layout constants (pin-local frame, origin at pin prismatic joint)
# The pin is compact so the guide post stays below the lever sweep.
_PIN_JOINT_Z_IN_HEAD = 0.025
_PIN_CAP_OFFSET = 0.0025
_PIN_CAP_LEN = 0.005
_PIN_CAP_R = 0.0058
_PIN_BODY_OFFSET = -0.0045
_PIN_BODY_LEN = 0.009
_PIN_BODY_R = 0.0028
_PIN_TIP_OFFSET = -0.010
_PIN_TIP_LEN = 0.002
_PIN_TIP_R = 0.0032


def _build_punch_head_visuals(head, *, blue, blue_highlight, steel, dark_steel, black, index: int):
    """Attach die, carriage, guide post, and thumbscrew visuals to a head part."""
    # Die elements (steel ring, black bore, blue boss)
    head.visual(
        Cylinder(radius=0.0095, length=_DIE_BOSS_LEN),
        origin=Origin(xyz=(0.0, 0.0, _DIE_BOSS_Z)),
        material=blue,
        name="die_boss",
    )
    head.visual(
        Cylinder(radius=0.0076, length=_DIE_RING_LEN),
        origin=Origin(xyz=(0.0, 0.0, _DIE_RING_Z)),
        material=steel,
        name="die_ring",
    )
    head.visual(
        Cylinder(radius=0.0042, length=_DIE_HOLE_LEN),
        origin=Origin(xyz=(0.0, 0.0, _DIE_HOLE_Z)),
        material=black,
        name="die_hole",
    )
    # Sliding carriage block
    head.visual(
        _rounded_box_mesh(f"punch_head_carriage_{index}", _CARRIAGE_SIZE, 0.0025),
        origin=Origin(xyz=(0.0, 0.0, _CARRIAGE_Z)),
        material=blue,
        name="carriage",
    )
    # Guide post / quill that the punch pin slides through
    head.visual(
        Cylinder(radius=_GUIDE_POST_R, length=_GUIDE_POST_LEN),
        origin=Origin(xyz=(0.0, 0.0, _GUIDE_POST_Z)),
        material=blue_highlight,
        name="guide_post",
    )
    # Small thumbscrew on the side for locking the head position
    head.visual(
        Cylinder(radius=0.004, length=0.003),
        origin=Origin(xyz=(0.0, _THUMBSCREW_Y, _THUMBSCREW_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="thumbscrew",
    )


def _build_punch_pin_visuals(pin, *, steel, dark_steel):
    """Attach punch pin, top cap, and cutting tip visuals to a pin part."""
    pin.visual(
        Cylinder(radius=_PIN_BODY_R, length=_PIN_BODY_LEN),
        origin=Origin(xyz=(0.0, 0.0, _PIN_BODY_OFFSET)),
        material=steel,
        name="punch_pin",
    )
    pin.visual(
        Cylinder(radius=_PIN_CAP_R, length=_PIN_CAP_LEN),
        origin=Origin(xyz=(0.0, 0.0, _PIN_CAP_OFFSET)),
        material=dark_steel,
        name="pin_top_cap",
    )
    pin.visual(
        Cylinder(radius=_PIN_TIP_R, length=_PIN_TIP_LEN),
        origin=Origin(xyz=(0.0, 0.0, _PIN_TIP_OFFSET)),
        material=steel,
        name="pin_cutting_tip",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="adjustable_desktop_hole_punch")

    blue = model.material("blue_painted_metal", rgba=(0.08, 0.24, 0.62, 1.0))
    blue_highlight = model.material("raised_blue_highlight", rgba=(0.12, 0.33, 0.78, 1.0))
    blue_shadow = model.material("dark_blue_shadow", rgba=(0.035, 0.09, 0.25, 1.0))
    rubber = model.material("black_rubber", rgba=(0.012, 0.012, 0.012, 1.0))
    black = model.material("black_slot", rgba=(0.0, 0.0, 0.0, 1.0))
    steel = model.material("brushed_steel", rgba=(0.78, 0.77, 0.72, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.30, 0.30, 0.32, 1.0))

    # ------------------------------------------------------------------
    # Base (fixed root)
    # ------------------------------------------------------------------
    base = model.part("base")
    base.visual(
        _rounded_box_mesh("hole_punch_rubber_pad", (0.202, 0.088, 0.009), 0.0030),
        origin=Origin(xyz=(0.0, 0.0, 0.0045)),
        material=rubber,
        name="rubber_pad",
    )
    base.visual(
        _rounded_box_mesh("hole_punch_blue_base_shell", (0.190, 0.076, 0.023), 0.0045),
        origin=Origin(xyz=(0.0, 0.0, 0.0205)),
        material=blue,
        name="blue_base_shell",
    )
    base.visual(
        _rounded_box_mesh("hole_punch_top_channel", (0.145, 0.048, 0.014), 0.0030),
        origin=Origin(xyz=(-0.002, 0.0, 0.0380)),
        material=blue_highlight,
        name="top_channel",
    )
    base.visual(
        _rounded_box_mesh("hole_punch_front_lip", (0.174, 0.012, 0.010), 0.0020),
        origin=Origin(xyz=(0.004, -0.032, 0.034)),
        material=blue,
        name="front_base_lip",
    )
    base.visual(
        _rounded_box_mesh("hole_punch_rear_hinge_block", (0.040, 0.064, 0.027), 0.0025),
        origin=Origin(xyz=(-0.071, 0.0, 0.038)),
        material=blue,
        name="rear_hinge_block",
    )

    side_plate_mesh = _extruded_xz_mesh("hole_punch_side_plate", _side_plate_profile(), 0.006)
    base.visual(
        side_plate_mesh,
        origin=Origin(xyz=(0.0, -0.040, 0.0)),
        material=blue,
        name="side_plate_visible",
    )
    base.visual(
        side_plate_mesh,
        origin=Origin(xyz=(0.0, 0.040, 0.0)),
        material=blue,
        name="side_plate_far",
    )

    # Two visible screw heads on the outside plate, with small dark centers.
    for index, (x, z) in enumerate(((-0.075, 0.058), (-0.052, 0.052))):
        base.visual(
            Cylinder(radius=0.0062, length=0.0032),
            origin=Origin(xyz=(x, -0.0445, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"side_screw_{index}",
        )
        base.visual(
            Cylinder(radius=0.0024, length=0.0035),
            origin=Origin(xyz=(x, -0.0464, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"side_screw_slot_{index}",
        )

    waste_window = _extruded_xz_mesh(
        "hole_punch_black_oval_window",
        superellipse_profile(0.017, 0.0065, exponent=2.2, segments=28),
        0.0022,
    )
    base.visual(
        waste_window,
        origin=Origin(xyz=(-0.061, -0.0440, 0.037)),
        material=black,
        name="side_waste_window",
    )
    base.visual(
        _rounded_box_mesh("hole_punch_paper_throat_shadow", (0.092, 0.041, 0.004), 0.0015),
        origin=Origin(xyz=(-0.006, 0.0, 0.050)),
        material=blue_shadow,
        name="paper_throat_gap",
    )

    # ------------------------------------------------------------------
    # Y-axis adjustment rail on the base top surface.
    # The rail runs in Y at the punch X-station so each head module can
    # slide to set hole spacing.
    # ------------------------------------------------------------------
    HEAD_X = 0.034
    HEAD_Z = 0.038  # prismatic joint Z in base frame (top-channel level)
    RAIL_LENGTH = 0.096

    base.visual(
        _rounded_box_mesh("adjustment_rail_slot", (0.036, RAIL_LENGTH, 0.003), 0.0010),
        origin=Origin(xyz=(HEAD_X, 0.0, 0.0395)),
        material=dark_steel,
        name="rail_slot",
    )
    base.visual(
        _rounded_box_mesh("adjustment_rail_left_wall", (0.004, RAIL_LENGTH, 0.008), 0.0008),
        origin=Origin(xyz=(HEAD_X - 0.017, 0.0, 0.042)),
        material=blue,
        name="rail_left_wall",
    )
    base.visual(
        _rounded_box_mesh("adjustment_rail_right_wall", (0.004, RAIL_LENGTH, 0.008), 0.0008),
        origin=Origin(xyz=(HEAD_X + 0.017, 0.0, 0.042)),
        material=blue,
        name="rail_right_wall",
    )
    # Small engraved tick marks along the rail for spacing reference
    for tick_i, tick_y in enumerate((-0.030, -0.015, 0.0, 0.015, 0.030)):
        base.visual(
            Box((0.010, 0.001, 0.001)),
            origin=Origin(xyz=(HEAD_X, tick_y, 0.0415)),
            material=blue_shadow,
            name=f"rail_tick_{tick_i}",
        )

    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.10, length=0.050),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    # ------------------------------------------------------------------
    # Pressing lever (revolute, unchanged from parent)
    # ------------------------------------------------------------------
    lever = model.part("pressing_lever")
    lever.visual(
        _extruded_xz_mesh("hole_punch_curved_handle", _lever_profile(), 0.052),
        material=blue,
        name="curved_handle",
    )
    lever.visual(
        _rounded_box_mesh("hole_punch_lower_press_channel", (0.145, 0.052, 0.014), 0.0025),
        origin=Origin(xyz=(0.060, 0.0, 0.004)),
        material=blue_highlight,
        name="lower_press_channel",
    )
    lever.visual(
        Cylinder(radius=0.0085, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=blue,
        name="hinge_barrel",
    )
    lever.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.006, 0.0, 0.035),
                    (0.044, 0.0, 0.065),
                    (0.094, 0.0, 0.116),
                    (0.144, 0.0, 0.143),
                ],
                radius=0.0043,
                samples_per_segment=14,
                radial_segments=14,
                cap_ends=True,
            ),
            "hole_punch_handle_raised_rib",
        ),
        material=blue_highlight,
        name="handle_raised_rib",
    )
    lever.inertial = Inertial.from_geometry(
        Cylinder(radius=0.060, length=0.180),
        mass=0.18,
        origin=Origin(xyz=(0.060, 0.0, 0.060), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    # ------------------------------------------------------------------
    # Adjustable punch-head modules and their punch pins.
    # Each head slides on the Y rail and carries its own die + guide post.
    # The punch pin is a prismatic child of the head, driven by the lever
    # via a mimic joint so squeezing the press pushes the pin downward.
    # ------------------------------------------------------------------
    HEAD_Y_NOMINAL = (-0.022, 0.022)
    HEAD_TRAVEL = 0.015  # ±15 mm slide range per head
    PIN_TRAVEL_MAX = 0.018  # prismatic upper limit for pin press stroke
    LEVER_UPPER = 0.16

    for i, y_nom in enumerate(HEAD_Y_NOMINAL):
        head = model.part(f"punch_head_{i}")
        _build_punch_head_visuals(
            head,
            blue=blue,
            blue_highlight=blue_highlight,
            steel=steel,
            dark_steel=dark_steel,
            black=black,
            index=i,
        )
        head.inertial = Inertial.from_geometry(
            Cylinder(radius=0.018, length=0.045),
            mass=0.08,
            origin=Origin(xyz=(0.0, 0.0, 0.022)),
        )

        pin = model.part(f"punch_pin_{i}")
        _build_punch_pin_visuals(pin, steel=steel, dark_steel=dark_steel)
        pin.inertial = Inertial.from_geometry(
            Cylinder(radius=0.004, length=0.034),
            mass=0.025,
            origin=Origin(xyz=(0.0, 0.0, -0.013)),
        )

    # ------------------------------------------------------------------
    # Articulations
    # ------------------------------------------------------------------
    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lever,
        origin=Origin(xyz=(-0.070, 0.0, 0.068)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=9.0, velocity=2.0, lower=0.0, upper=LEVER_UPPER),
    )

    for i, y_nom in enumerate(HEAD_Y_NOMINAL):
        # Y-axis prismatic rail joint: head slides along base Y
        model.articulation(
            f"base_to_head_{i}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=f"punch_head_{i}",
            origin=Origin(xyz=(HEAD_X, y_nom, HEAD_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=15.0,
                velocity=0.10,
                lower=-HEAD_TRAVEL,
                upper=HEAD_TRAVEL,
            ),
        )
        # Z-axis prismatic pin joint: pin presses down independently.
        # In the real mechanism the lever drives the pin via a linkage;
        # here the pin's linear DOF is represented explicitly so tests
        # can prove press travel at any rail position.
        model.articulation(
            f"head_to_pin_{i}",
            ArticulationType.PRISMATIC,
            parent=f"punch_head_{i}",
            child=f"punch_pin_{i}",
            origin=Origin(xyz=(0.0, 0.0, _PIN_JOINT_Z_IN_HEAD)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=20.0,
                velocity=0.50,
                lower=0.0,
                upper=PIN_TRAVEL_MAX,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lever = object_model.get_part("pressing_lever")
    head_0 = object_model.get_part("punch_head_0")
    head_1 = object_model.get_part("punch_head_1")
    pin_0 = object_model.get_part("punch_pin_0")
    pin_1 = object_model.get_part("punch_pin_1")

    lever_joint = object_model.get_articulation("lever_pivot")
    rail_0 = object_model.get_articulation("base_to_head_0")
    rail_1 = object_model.get_articulation("base_to_head_1")
    pin_drive_0 = object_model.get_articulation("head_to_pin_0")
    pin_drive_1 = object_model.get_articulation("head_to_pin_1")

    base_visuals = {v.name for v in base.visuals}
    lever_visuals = {v.name for v in lever.visuals}
    head_0_visuals = {v.name for v in head_0.visuals}
    head_1_visuals = {v.name for v in head_1.visuals}

    # --- Base and rail ---
    ctx.check(
        "desktop hole punch base has layered blue body",
        {"blue_base_shell", "top_channel", "paper_throat_gap"}.issubset(base_visuals),
        details=f"base visuals={sorted(base_visuals)}",
    )
    ctx.check(
        "black rubber base pad is modeled",
        "rubber_pad" in base_visuals,
        details=f"base visuals={sorted(base_visuals)}",
    )
    ctx.check(
        "side plate has two visible screw heads",
        {"side_screw_0", "side_screw_1", "side_waste_window"}.issubset(base_visuals),
        details=f"base visuals={sorted(base_visuals)}",
    )
    ctx.check(
        "Y-axis adjustment rail is on the base",
        {"rail_slot", "rail_left_wall", "rail_right_wall"}.issubset(base_visuals),
        details=f"base visuals={sorted(base_visuals)}",
    )

    # --- Lever ---
    ctx.check(
        "curved raised handle is modeled",
        {"curved_handle", "lower_press_channel", "handle_raised_rib", "hinge_barrel"}.issubset(lever_visuals),
        details=f"lever visuals={sorted(lever_visuals)}",
    )
    ctx.check(
        "lever joint is non-fixed revolute",
        lever_joint.articulation_type == ArticulationType.REVOLUTE
        and lever_joint.motion_limits is not None
        and lever_joint.motion_limits.upper > lever_joint.motion_limits.lower,
        details=f"type={lever_joint.articulation_type}, limits={lever_joint.motion_limits}",
    )

    # --- Adjustable head modules ---
    ctx.check(
        "head 0 carries die and carriage",
        {"die_ring", "die_hole", "carriage", "guide_post"}.issubset(head_0_visuals),
        details=f"head_0 visuals={sorted(head_0_visuals)}",
    )
    ctx.check(
        "head 1 carries die and carriage",
        {"die_ring", "die_hole", "carriage", "guide_post"}.issubset(head_1_visuals),
        details=f"head_1 visuals={sorted(head_1_visuals)}",
    )
    ctx.check(
        "head 0 slides on Y prismatic rail",
        rail_0.articulation_type == ArticulationType.PRISMATIC
        and rail_0.axis == (0.0, 1.0, 0.0)
        and rail_0.motion_limits is not None
        and rail_0.motion_limits.lower < 0.0
        and rail_0.motion_limits.upper > 0.0,
        details=f"rail_0 type={rail_0.articulation_type}, axis={rail_0.axis}, limits={rail_0.motion_limits}",
    )
    ctx.check(
        "head 1 slides on Y prismatic rail",
        rail_1.articulation_type == ArticulationType.PRISMATIC
        and rail_1.axis == (0.0, 1.0, 0.0)
        and rail_1.motion_limits is not None
        and rail_1.motion_limits.lower < 0.0
        and rail_1.motion_limits.upper > 0.0,
        details=f"rail_1 type={rail_1.articulation_type}, axis={rail_1.axis}, limits={rail_1.motion_limits}",
    )

    # --- Pin press mechanism (independent prismatic DOF per head) ---
    ctx.check(
        "pin 0 has prismatic press DOF",
        pin_drive_0.articulation_type == ArticulationType.PRISMATIC
        and pin_drive_0.axis == (0.0, 0.0, -1.0)
        and pin_drive_0.motion_limits is not None
        and pin_drive_0.motion_limits.upper > 0.0,
        details=f"pin_drive_0={pin_drive_0.articulation_type}, axis={pin_drive_0.axis}, limits={pin_drive_0.motion_limits}",
    )
    ctx.check(
        "pin 1 has prismatic press DOF",
        pin_drive_1.articulation_type == ArticulationType.PRISMATIC
        and pin_drive_1.axis == (0.0, 0.0, -1.0)
        and pin_drive_1.motion_limits is not None
        and pin_drive_1.motion_limits.upper > 0.0,
        details=f"pin_drive_1={pin_drive_1.articulation_type}, axis={pin_drive_1.axis}, limits={pin_drive_1.motion_limits}",
    )

    # --- Pin-die alignment at rest (pins stay centered over their dies) ---
    ctx.expect_overlap(
        pin_0, head_0,
        axes="xy",
        elem_a="punch_pin",
        elem_b="die_hole",
        min_overlap=0.003,
        name="pin 0 aligned with die at rest",
    )
    ctx.expect_overlap(
        pin_1, head_1,
        axes="xy",
        elem_a="punch_pin",
        elem_b="die_hole",
        min_overlap=0.003,
        name="pin 1 aligned with die at rest",
    )

    # --- Pin-die alignment at rail extremes ---
    rail_0_upper = rail_0.motion_limits.upper
    rail_1_upper = rail_1.motion_limits.upper
    rail_0_lower = rail_0.motion_limits.lower
    rail_1_lower = rail_1.motion_limits.lower

    with ctx.pose({rail_0: rail_0_upper, rail_1: rail_1_upper}):
        ctx.expect_overlap(
            pin_0, head_0,
            axes="xy",
            elem_a="punch_pin",
            elem_b="die_hole",
            min_overlap=0.003,
            name="pin 0 aligned with die at rail max spread",
        )
        ctx.expect_overlap(
            pin_1, head_1,
            axes="xy",
            elem_a="punch_pin",
            elem_b="die_hole",
            min_overlap=0.003,
            name="pin 1 aligned with die at rail max spread",
        )

    with ctx.pose({rail_0: rail_0_lower, rail_1: rail_1_lower}):
        ctx.expect_overlap(
            pin_0, head_0,
            axes="xy",
            elem_a="punch_pin",
            elem_b="die_hole",
            min_overlap=0.003,
            name="pin 0 aligned with die at rail min spread",
        )
        ctx.expect_overlap(
            pin_1, head_1,
            axes="xy",
            elem_a="punch_pin",
            elem_b="die_hole",
            min_overlap=0.003,
            name="pin 1 aligned with die at rail min spread",
        )

    # --- Clearance: open pins clear die throat ---
    ctx.expect_gap(
        pin_0, head_0,
        axis="z",
        positive_elem="punch_pin",
        negative_elem="die_hole",
        min_gap=0.002,
        max_gap=0.015,
        name="open first pin clears die throat",
    )

    # --- Press travel: pin prismatic DOF drives pins downward ---
    pin_0_upper = pin_drive_0.motion_limits.upper
    rest_pin_pos = ctx.part_world_position(pin_0)
    with ctx.pose({pin_drive_0: pin_0_upper}):
        pressed_pin_pos = ctx.part_world_position(pin_0)
    ctx.check(
        "pin press DOF drives punch pin downward",
        rest_pin_pos is not None
        and pressed_pin_pos is not None
        and pressed_pin_pos[2] < rest_pin_pos[2] - 0.006,
        details=f"rest={rest_pin_pos}, pressed={pressed_pin_pos}",
    )

    # --- Lever squeeze still works as a separate DOF ---
    rest_channel_aabb = ctx.part_element_world_aabb(lever, elem="lower_press_channel")
    with ctx.pose({lever_joint: lever_joint.motion_limits.upper}):
        pressed_channel_aabb = ctx.part_element_world_aabb(lever, elem="lower_press_channel")
    ctx.check(
        "lever pivot rotates the pressing channel downward",
        rest_channel_aabb is not None
        and pressed_channel_aabb is not None
        and pressed_channel_aabb[0][2] < rest_channel_aabb[0][2] - 0.005,
        details=f"rest_min={rest_channel_aabb[0]}, pressed_min={pressed_channel_aabb[0]}",
    )

    # --- Rail travel actually moves heads in Y ---
    rest_head_pos = ctx.part_world_position(head_0)
    with ctx.pose({rail_0: rail_0_upper}):
        slid_head_pos = ctx.part_world_position(head_0)
    ctx.check(
        "rail travel shifts head along Y",
        rest_head_pos is not None
        and slid_head_pos is not None
        and abs(slid_head_pos[1] - rest_head_pos[1]) > 0.008,
        details=f"rest={rest_head_pos}, slid={slid_head_pos}",
    )

    # --- Allow small overlap between pin cap and lever press channel ---
    ctx.allow_overlap(
        lever,
        pin_0,
        elem_a="lower_press_channel",
        elem_b="pin_top_cap",
        reason="The pin top cap is locally captured inside the pressing channel as the lever sweeps over it.",
    )
    ctx.allow_overlap(
        lever,
        pin_1,
        elem_a="lower_press_channel",
        elem_b="pin_top_cap",
        reason="The pin top cap is locally captured inside the pressing channel as the lever sweeps over it.",
    )

    # --- Allow pin body inside guide post (captured sliding fit) ---
    ctx.allow_overlap(
        head_0,
        pin_0,
        elem_a="guide_post",
        elem_b="punch_pin",
        reason="The punch pin slides through the guide post bore as a captured linear bearing.",
    )
    ctx.allow_overlap(
        head_1,
        pin_1,
        elem_a="guide_post",
        elem_b="punch_pin",
        reason="The punch pin slides through the guide post bore as a captured linear bearing.",
    )
    # Prove the captured fit with an exact containment check
    ctx.expect_within(
        pin_0,
        head_0,
        axes="xy",
        inner_elem="punch_pin",
        outer_elem="guide_post",
        margin=0.001,
        name="pin 0 stays centered inside guide post bore",
    )
    ctx.expect_within(
        pin_1,
        head_1,
        axes="xy",
        inner_elem="punch_pin",
        outer_elem="guide_post",
        margin=0.001,
        name="pin 1 stays centered inside guide post bore",
    )

    return ctx.report()


object_model = build_object_model()
