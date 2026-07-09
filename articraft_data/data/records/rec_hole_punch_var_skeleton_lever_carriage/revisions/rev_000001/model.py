from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
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


def _punch_pin_visuals(pin, steel, dark_steel):
    """Shared geometry helper for twin punch pin visuals.

    Pins are shorter to fit between the carriage and die holes:
    - carriage sits at z≈0.067; pin body hangs below
    - cutting tip clears the die throat at rest
    - pins enter the dies during the squeeze stroke
    """
    pin.visual(
        Cylinder(radius=0.0028, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=steel,
        name="punch_pin",
    )
    pin.visual(
        Cylinder(radius=0.0058, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=dark_steel,
        name="pin_top_cap",
    )
    pin.visual(
        Cylinder(radius=0.0032, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, -0.0135)),
        material=steel,
        name="pin_cutting_tip",
    )
    pin.inertial = Inertial.from_geometry(
        Cylinder(radius=0.004, length=0.018),
        mass=0.018,
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="blue_desktop_two_hole_punch")

    blue = model.material("blue_painted_metal", rgba=(0.08, 0.24, 0.62, 1.0))
    blue_highlight = model.material("raised_blue_highlight", rgba=(0.12, 0.33, 0.78, 1.0))
    blue_shadow = model.material("dark_blue_shadow", rgba=(0.035, 0.09, 0.25, 1.0))
    rubber = model.material("black_rubber", rgba=(0.012, 0.012, 0.012, 1.0))
    black = model.material("black_slot", rgba=(0.0, 0.0, 0.0, 1.0))
    steel = model.material("brushed_steel", rgba=(0.78, 0.77, 0.72, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.30, 0.30, 0.32, 1.0))

    # ── Base (fixed root) ────────────────────────────────────────────────
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

    # Paired round punch dies in the base (loop-based, N=2).
    pin_y_positions = (-0.022, 0.022)
    pin_x = 0.034
    for index, y in enumerate(pin_y_positions):
        base.visual(
            Cylinder(radius=0.0076, length=0.0030),
            origin=Origin(xyz=(pin_x, y, 0.0470)),
            material=steel,
            name=f"die_ring_{index}",
        )
        base.visual(
            Cylinder(radius=0.0042, length=0.0034),
            origin=Origin(xyz=(pin_x, y, 0.0490)),
            material=black,
            name=f"die_hole_{index}",
        )
        base.visual(
            Cylinder(radius=0.0095, length=0.0045),
            origin=Origin(xyz=(pin_x, y, 0.0445)),
            material=blue,
            name=f"die_boss_{index}",
        )

    # Head casting: raised block on the base that houses the plunger guide posts.
    base.visual(
        _rounded_box_mesh("hole_punch_head_casting", (0.044, 0.082, 0.016), 0.003),
        origin=Origin(xyz=(pin_x, 0.0, 0.056)),
        material=blue,
        name="head_casting",
    )

    # Vertical guide posts for the plunger carriage (loop-based, N=2).
    guide_y_positions = (-0.033, 0.033)
    for index, y in enumerate(guide_y_positions):
        base.visual(
            Cylinder(radius=0.003, length=0.036),
            origin=Origin(xyz=(pin_x, y, 0.066)),
            material=steel,
            name=f"guide_post_{index}",
        )

    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.10, length=0.050),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    # ── Pressing lever (revolute) ────────────────────────────────────────
    lever = model.part("pressing_lever")
    lever.visual(
        _extruded_xz_mesh("hole_punch_curved_handle", _lever_profile(), 0.052),
        material=blue,
        name="curved_handle",
    )
    lever.visual(
        _rounded_box_mesh("hole_punch_lower_press_channel", (0.145, 0.052, 0.004), 0.0025),
        origin=Origin(xyz=(0.060, 0.0, 0.010)),
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
    # Cam nose: vertical plate on the lever underside that pushes the carriage down.
    lever.visual(
        _rounded_box_mesh("hole_punch_cam_nose", (0.022, 0.034, 0.028), 0.003),
        origin=Origin(xyz=(0.104, 0.0, 0.008)),
        material=blue,
        name="cam_nose",
    )
    lever.inertial = Inertial.from_geometry(
        Cylinder(radius=0.060, length=0.180),
        mass=0.18,
        origin=Origin(xyz=(0.060, 0.0, 0.060), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    # ── Plunger carriage (prismatic, vertical guide) ─────────────────────
    carriage = model.part("plunger_carriage")
    carriage.visual(
        _rounded_box_mesh("hole_punch_carriage_body", (0.034, 0.058, 0.010), 0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_steel,
        name="carriage_body",
    )
    carriage.visual(
        _rounded_box_mesh("hole_punch_carriage_cam_pad", (0.020, 0.030, 0.004), 0.001),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=steel,
        name="carriage_cam_pad",
    )
    # Guide bores: show where the carriage slides on the base guide posts.
    for index, y in enumerate(guide_y_positions):
        carriage.visual(
            Cylinder(radius=0.0045, length=0.012),
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=dark_steel,
            name=f"guide_bore_{index}",
        )
    carriage.inertial = Inertial.from_geometry(
        Cylinder(radius=0.025, length=0.012),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # ── Punch pins (FIXED to carriage, vertical travel) ──────────────────
    for index, y in enumerate(pin_y_positions):
        pin = model.part(f"punch_pin_{index}")
        _punch_pin_visuals(pin, steel, dark_steel)

    # ── Articulations ────────────────────────────────────────────────────

    # Lever pivot: revolute around the rear hinge line.
    # Positive q rotates the lever nose downward (squeezing the punch).
    lever_pivot = model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lever,
        origin=Origin(xyz=(-0.070, 0.0, 0.068)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=9.0, velocity=2.0, lower=0.0, upper=0.16),
    )

    # Carriage guide: prismatic vertical travel on the base head casting.
    # The lever cam nose pushes the carriage straight down during squeeze.
    carriage_travel = 0.012  # meters of vertical punch stroke
    carriage_guide = model.articulation(
        "carriage_guide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(pin_x, 0.0, 0.067)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.5,
            lower=0.0,
            upper=carriage_travel,
        ),
    )

    # Punch pins are FIXED to the carriage (straight vertical travel, no arc).
    for index, y in enumerate(pin_y_positions):
        model.articulation(
            f"carriage_to_pin_{index}",
            ArticulationType.FIXED,
            parent=carriage,
            child=f"punch_pin_{index}",
            origin=Origin(xyz=(0.0, y, 0.0)),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lever = object_model.get_part("pressing_lever")
    carriage = object_model.get_part("plunger_carriage")
    pin_0 = object_model.get_part("punch_pin_0")
    pin_1 = object_model.get_part("punch_pin_1")
    lever_joint = object_model.get_articulation("lever_pivot")
    carriage_joint = object_model.get_articulation("carriage_guide")
    pin_mount_0 = object_model.get_articulation("carriage_to_pin_0")
    pin_mount_1 = object_model.get_articulation("carriage_to_pin_1")

    base_visuals = {visual.name for visual in base.visuals}
    lever_visuals = {visual.name for visual in lever.visuals}
    carriage_visuals = {visual.name for visual in carriage.visuals}

    # ── Structural checks ────────────────────────────────────────────────
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
        "curved raised handle is modeled",
        {"curved_handle", "lower_press_channel", "handle_raised_rib", "hinge_barrel"}.issubset(lever_visuals),
        details=f"lever visuals={sorted(lever_visuals)}",
    )
    ctx.check(
        "lever joint is non fixed revolute",
        lever_joint.articulation_type == ArticulationType.REVOLUTE
        and lever_joint.motion_limits is not None
        and lever_joint.motion_limits.upper > lever_joint.motion_limits.lower,
        details=f"type={lever_joint.articulation_type}, limits={lever_joint.motion_limits}",
    )
    ctx.check(
        "plunger_carriage has prismatic vertical guide joint",
        carriage_joint.articulation_type == ArticulationType.PRISMATIC
        and carriage_joint.motion_limits is not None
        and carriage_joint.motion_limits.upper > carriage_joint.motion_limits.lower,
        details=f"type={carriage_joint.articulation_type}, limits={carriage_joint.motion_limits}",
    )
    ctx.check(
        "plunger carriage has visible body and cam pad",
        {"carriage_body", "carriage_cam_pad"}.issubset(carriage_visuals),
        details=f"carriage visuals={sorted(carriage_visuals)}",
    )
    ctx.check(
        "base has head casting and guide posts for carriage",
        {"head_casting", "guide_post_0", "guide_post_1"}.issubset(base_visuals),
        details=f"base visuals={sorted(base_visuals)}",
    )
    ctx.check(
        "twin punch pins are FIXED to the plunger carriage (not the arcing lever)",
        pin_mount_0.articulation_type == ArticulationType.FIXED
        and pin_mount_1.articulation_type == ArticulationType.FIXED
        and pin_mount_0.parent == "plunger_carriage"
        and pin_mount_1.parent == "plunger_carriage",
        details=f"pin mount parents={pin_mount_0.parent}, {pin_mount_1.parent}",
    )
    ctx.check(
        "paired round punch dies are in the base",
        {"die_ring_0", "die_ring_1", "die_hole_0", "die_hole_1"}.issubset(base_visuals),
        details=f"base visuals={sorted(base_visuals)}",
    )
    ctx.check(
        "lever has cam nose to drive carriage",
        "cam_nose" in lever_visuals,
        details=f"lever visuals={sorted(lever_visuals)}",
    )

    # ── Pin-to-die alignment (XY) ────────────────────────────────────────
    ctx.expect_overlap(
        pin_0,
        base,
        axes="xy",
        elem_a="punch_pin",
        elem_b="die_hole_0",
        min_overlap=0.003,
        name="first punch pin aligns with first die",
    )
    ctx.expect_overlap(
        pin_1,
        base,
        axes="xy",
        elem_a="punch_pin",
        elem_b="die_hole_1",
        min_overlap=0.003,
        name="second punch pin aligns with second die",
    )

    # ── Rest pose: pins clear the die throat ─────────────────────────────
    ctx.expect_gap(
        pin_0,
        base,
        axis="z",
        positive_elem="pin_cutting_tip",
        negative_elem="die_hole_0",
        min_gap=0.001,
        max_gap=0.012,
        name="open first pin clears die throat",
    )

    # ── Intentional overlaps at mechanical interfaces ────────────────────

    # Cam nose on lever contacts/embeds into carriage cam pad region.
    ctx.allow_overlap(
        lever,
        carriage,
        elem_a="cam_nose",
        elem_b="carriage_body",
        reason="The lever cam nose wraps around the carriage body to represent the cam-contact push interface.",
    )
    ctx.allow_overlap(
        lever,
        carriage,
        elem_a="cam_nose",
        elem_b="carriage_cam_pad",
        reason="The lever cam nose contacts the carriage cam pad as the sliding push interface.",
    )
    ctx.expect_overlap(
        lever,
        carriage,
        axes="xy",
        elem_a="cam_nose",
        elem_b="carriage_body",
        min_overlap=0.004,
        name="cam nose overlaps carriage in XY at the contact interface",
    )

    # Guide posts pass through the carriage guide bores (captured shaft).
    for index in range(2):
        ctx.allow_overlap(
            base,
            carriage,
            elem_a=f"guide_post_{index}",
            elem_b=f"guide_bore_{index}",
            reason=f"Guide post {index} passes through the carriage bore as a captured sliding shaft.",
        )
        ctx.expect_overlap(
            base,
            carriage,
            axes="xy",
            elem_a=f"guide_post_{index}",
            elem_b=f"guide_bore_{index}",
            min_overlap=0.002,
            name=f"guide post {index} is centered in carriage bore",
        )

    # Punch pins pass through the carriage body (captured pin).
    for index, pin in enumerate((pin_0, pin_1)):
        ctx.allow_overlap(
            carriage,
            pin,
            elem_a="carriage_body",
            elem_b="punch_pin",
            reason=f"Punch pin {index} is intentionally captured inside the carriage body for vertical travel.",
        )
        ctx.expect_overlap(
            carriage,
            pin,
            axes="xy",
            elem_a="carriage_body",
            elem_b="punch_pin",
            min_overlap=0.003,
            name=f"punch pin {index} is centered in carriage",
        )

    # ── Squeeze stroke: carriage and pins travel straight down ───────────
    open_carriage_pos = ctx.part_world_position(carriage)
    open_pin_pos = ctx.part_world_position(pin_0)
    with ctx.pose({
        lever_joint: lever_joint.motion_limits.upper,
        carriage_joint: carriage_joint.motion_limits.upper,
    }):
        closed_carriage_pos = ctx.part_world_position(carriage)
        closed_pin_pos = ctx.part_world_position(pin_0)

    ctx.check(
        "squeezing lever drives plunger carriage downward (prismatic stroke)",
        open_carriage_pos is not None
        and closed_carriage_pos is not None
        and closed_carriage_pos[2] < open_carriage_pos[2] - 0.006,
        details=f"open={open_carriage_pos}, closed={closed_carriage_pos}",
    )
    ctx.check(
        "squeezing lever drives punch pins downward via carriage",
        open_pin_pos is not None
        and closed_pin_pos is not None
        and closed_pin_pos[2] < open_pin_pos[2] - 0.006,
        details=f"open={open_pin_pos}, closed={closed_pin_pos}",
    )
    ctx.check(
        "carriage travels vertically (X stays constant)",
        open_carriage_pos is not None
        and closed_carriage_pos is not None
        and abs(closed_carriage_pos[0] - open_carriage_pos[0]) < 0.001,
        details=f"open_x={open_carriage_pos[0] if open_carriage_pos else None}, "
        f"closed_x={closed_carriage_pos[0] if closed_carriage_pos else None}",
    )

    return ctx.report()


object_model = build_object_model()
