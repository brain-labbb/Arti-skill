from __future__ import annotations

# Cordless battery-powered walk-behind push lawn mower.
#
# Layout (world frame, deck part frame at the origin):
#   +X = forward (deck front), +Y = left, +Z = up.
#   Glossy burnt-orange revolved steel deck shell, hollow underneath, with a
#   domed top, a short side skirt, a rear apron, and four corner axle brackets.
#   A compact brushless motor hub sits on the spindle boss at the deck center;
#   a removable rectangular battery pack is seated in a deck-top cradle behind
#   the motor. The cutting blade hangs from the motor shaft inside the chamber.
#   A thin tubular U-handlebar with red lower segments and a control cable
#   sweeps up and back from rear deck brackets and folds forward for storage.
#
# Articulations:
#   - four CONTINUOUS wheel-spin joints about the lateral (Y) axis
#   - one CONTINUOUS blade-spin joint about the vertical motor shaft (Z)
#   - one REVOLUTE handlebar fold joint about the lateral (Y) axis.
#     With the realistic ~40 deg handle rake required by the 1.5 m overall
#     length and 1.0 m grip height, folding flat over the deck needs about
#     -130 deg of travel (the prompt's "-100 deg" assumes a steeper handle).

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireGroove,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    tube_from_spline_points,
)

DECK_TOP_Z = 0.228  # top plane of the spindle boss / engine seating plane
HANDLE_PIVOT = (-0.28, 0.0, 0.17)
HANDLE_FOLD_LOWER = -2.27  # ~-130 deg: handle lies nearly flat over the deck

# (part name, x, side(+1 left / -1 right), wheel center z = tire radius,
#  tire width, rim radius, rim width)
WHEEL_SPECS = [
    ("front_wheel_0", 0.19, 1.0, 0.100, 0.045, 0.076, 0.040),
    ("front_wheel_1", 0.19, -1.0, 0.100, 0.045, 0.076, 0.040),
    ("rear_wheel_0", -0.19, 1.0, 0.125, 0.050, 0.094, 0.044),
    ("rear_wheel_1", -0.19, -1.0, 0.125, 0.050, 0.094, 0.044),
]


def _mirror_y(points: list[tuple[float, float, float]], side: float) -> list[tuple[float, float, float]]:
    return [(x, side * y, z) for x, y, z in points]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="push_lawn_mower")

    deck_orange = model.material("deck_orange", rgba=(0.72, 0.21, 0.08, 1.0))
    motor_black = model.material("motor_black", rgba=(0.08, 0.08, 0.10, 1.0))
    panel_dark = model.material("panel_dark", rgba=(0.04, 0.04, 0.05, 1.0))
    steel_gray = model.material("steel_gray", rgba=(0.62, 0.63, 0.66, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.55, 0.56, 0.58, 1.0))
    hub_gray = model.material("hub_gray", rgba=(0.55, 0.56, 0.58, 1.0))
    tire_rubber = model.material("tire_rubber", rgba=(0.07, 0.07, 0.08, 1.0))
    handle_black = model.material("handle_black", rgba=(0.10, 0.10, 0.11, 1.0))
    accent_red = model.material("accent_red", rgba=(0.74, 0.14, 0.09, 1.0))
    cable_black = model.material("cable_black", rgba=(0.04, 0.04, 0.04, 1.0))
    grip_foam = model.material("grip_foam", rgba=(0.13, 0.13, 0.14, 1.0))
    battery_body = model.material("battery_body", rgba=(0.06, 0.06, 0.07, 1.0))
    battery_accent = model.material("battery_accent", rgba=(0.18, 0.62, 0.32, 1.0))
    cradle_dark = model.material("cradle_dark", rgba=(0.05, 0.05, 0.06, 1.0))

    # ------------------------------------------------------------------ deck
    deck = model.part("deck")

    # Hollow revolved shell: short side skirt + smoothly domed top, open at
    # the bottom (the cutting chamber), with a center hole for the crankshaft.
    shell = LatheGeometry.from_shell_profiles(
        [
            (0.230, 0.042),
            (0.230, 0.120),
            (0.222, 0.150),
            (0.205, 0.176),
            (0.178, 0.197),
            (0.140, 0.212),
            (0.095, 0.220),
            (0.055, 0.2235),
        ],
        [
            (0.224, 0.042),
            (0.224, 0.117),
            (0.216, 0.146),
            (0.199, 0.171),
            (0.172, 0.191),
            (0.135, 0.206),
            (0.092, 0.2135),
            (0.055, 0.217),
        ],
        segments=80,
    )
    deck.visual(mesh_from_geometry(shell, "deck_shell"), material=deck_orange, name="shell")

    # Annular engine-mount boss with a real crankshaft bore (r=0.025).
    boss = LatheGeometry.from_shell_profiles(
        [(0.115, 0.205), (0.115, DECK_TOP_Z)],
        [(0.025, 0.205), (0.025, DECK_TOP_Z)],
        segments=64,
    )
    deck.visual(mesh_from_geometry(boss, "deck_spindle_boss"), material=deck_orange, name="spindle_boss")

    # Rear apron where the handlebar brackets mount.
    deck.visual(
        Box((0.082, 0.30, 0.085)),
        origin=Origin(xyz=(-0.259, 0.0, 0.0875)),
        material=deck_orange,
        name="rear_apron",
    )

    # Corner axle brackets and stub axles.
    for name, wx, side, wz, _tw, _wr, _ww in WHEEL_SPECS:
        deck.visual(
            Box((0.05, 0.10, 0.10)),
            origin=Origin(xyz=(wx, side * 0.165, wz)),
            material=panel_dark,
            name=f"{name}_bracket",
        )
        deck.visual(
            Cylinder(radius=0.0105, length=0.075),
            origin=Origin(xyz=(wx, side * 0.2275, wz), rpy=(pi / 2.0, 0.0, 0.0)),
            material=steel_gray,
            name=f"{name}_axle",
        )

    # Handlebar pivot brackets: two plates on the apron plus bolt bosses.
    for i, side in enumerate((1.0, -1.0)):
        deck.visual(
            Box((0.05, 0.014, 0.11)),
            origin=Origin(xyz=(-0.285, side * 0.135, 0.155)),
            material=panel_dark,
            name=f"handle_bracket_{i}",
        )
        deck.visual(
            Cylinder(radius=0.014, length=0.055),
            origin=Origin(xyz=(HANDLE_PIVOT[0], side * 0.1475, HANDLE_PIVOT[2]), rpy=(pi / 2.0, 0.0, 0.0)),
            material=steel_gray,
            name=f"handle_bolt_boss_{i}",
        )

    # ---------------------------------------------------------- motor hub
    motor = model.part("motor")
    # Compact cylindrical brushless motor housing, much smaller than a gas
    # engine block. Seated on the spindle boss at deck top.
    motor.visual(
        Cylinder(radius=0.058, length=0.068),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=motor_black,
        name="motor_housing",
    )
    # Domed top cap.
    motor.visual(
        Cylinder(radius=0.050, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.073)),
        material=panel_dark,
        name="motor_cap",
    )
    # Cooling vent slots on opposite sides of the housing.
    for i, side in enumerate((1.0, -1.0)):
        motor.visual(
            Box((0.065, 0.005, 0.032)),
            origin=Origin(xyz=(0.0, side * 0.054, 0.040)),
            material=panel_dark,
            name=f"motor_vent_{i}",
        )
    # Small controller module on the rear of the housing.
    motor.visual(
        Box((0.030, 0.040, 0.028)),
        origin=Origin(xyz=(-0.050, 0.0, 0.048)),
        material=panel_dark,
        name="motor_controller",
    )
    # Short power leads from controller back toward the battery cradle.
    motor.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (-0.065, 0.0, 0.048),
                    (-0.090, 0.0, 0.040),
                    (-0.110, 0.0, 0.028),
                ],
                radius=0.004,
                samples_per_segment=8,
                radial_segments=8,
            ),
            "motor_power_lead",
        ),
        material=cable_black,
        name="power_lead",
    )

    model.articulation(
        "motor_mount",
        ArticulationType.FIXED,
        parent=deck,
        child=motor,
        origin=Origin(xyz=(0.0, 0.0, DECK_TOP_Z)),
    )

    # -------------------------------------------- battery cradle (on deck)
    # Two parallel guide rails and a rear latch stop, mounted on the deck
    # dome just behind the motor hub.
    BATTERY_CRADLE_X = -0.19
    BATTERY_CRADLE_Z = DECK_TOP_Z - 0.008  # sits slightly into the dome slope
    for i, side in enumerate((1.0, -1.0)):
        deck.visual(
            Box((0.155, 0.014, 0.028)),
            origin=Origin(xyz=(BATTERY_CRADLE_X, side * 0.048, BATTERY_CRADLE_Z + 0.014)),
            material=cradle_dark,
            name=f"battery_rail_{i}",
        )
    deck.visual(
        Box((0.014, 0.11, 0.032)),
        origin=Origin(xyz=(BATTERY_CRADLE_X - 0.078, 0.0, BATTERY_CRADLE_Z + 0.016)),
        material=cradle_dark,
        name="battery_rear_stop",
    )
    deck.visual(
        Box((0.014, 0.11, 0.024)),
        origin=Origin(xyz=(BATTERY_CRADLE_X + 0.078, 0.0, BATTERY_CRADLE_Z + 0.012)),
        material=cradle_dark,
        name="battery_front_stop",
    )

    # --------------------------------------------------- battery pack
    battery = model.part("battery")
    # Removable rectangular battery pack seated in the cradle.
    battery.visual(
        Box((0.145, 0.088, 0.058)),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=battery_body,
        name="battery_body",
    )
    # Green accent label strip on top face.
    battery.visual(
        Box((0.100, 0.070, 0.005)),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=battery_accent,
        name="battery_label",
    )
    # Release latch tab on the front face.
    battery.visual(
        Box((0.008, 0.032, 0.018)),
        origin=Origin(xyz=(0.076, 0.0, 0.040)),
        material=battery_accent,
        name="battery_latch",
    )
    # Contact terminals on the rear bottom edge, flush with the pack rear face.
    battery.visual(
        Box((0.010, 0.040, 0.012)),
        origin=Origin(xyz=(-0.064, 0.0, 0.006)),
        material=steel_gray,
        name="battery_terminals",
    )

    model.articulation(
        "battery_seat",
        ArticulationType.FIXED,
        parent=deck,
        child=battery,
        origin=Origin(xyz=(BATTERY_CRADLE_X, 0.0, BATTERY_CRADLE_Z + 0.002)),
    )

    # ----------------------------------------------------------------- blade
    blade = model.part("blade")
    blade.visual(
        Box((0.38, 0.05, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=blade_steel,
        name="blade_bar",
    )
    blade.visual(
        Cylinder(radius=0.030, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=panel_dark,
        name="blade_hub",
    )
    # Motor shaft: rises through the spindle-boss bore (clearance fit) and is
    # intentionally captured inside the motor housing proxy.
    blade.visual(
        Cylinder(radius=0.010, length=0.170),
        origin=Origin(xyz=(0.0, 0.0, 0.097)),
        material=steel_gray,
        name="motor_shaft",
    )

    model.articulation(
        "blade_spin",
        ArticulationType.CONTINUOUS,
        parent=motor,
        child=blade,
        origin=Origin(xyz=(0.0, 0.0, -0.153)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=400.0),
    )

    # ---------------------------------------------------------------- wheels
    for name, wx, side, wz, tw, wr, ww in WHEEL_SPECS:
        wheel = model.part(name)
        wheel.visual(
            mesh_from_geometry(
                WheelGeometry(
                    wr,
                    ww,
                    rim=WheelRim(inner_radius=wr * 0.66, flange_height=0.006, flange_thickness=0.003),
                    hub=WheelHub(radius=0.024, width=ww + 0.004, cap_style="domed"),
                    face=WheelFace(dish_depth=0.004, front_inset=0.002),
                    spokes=WheelSpokes(style="straight", count=6, thickness=0.004, window_radius=0.006),
                    bore=WheelBore(style="round", diameter=0.016),
                ),
                f"{name}_hub",
            ),
            origin=Origin(rpy=(0.0, 0.0, side * pi / 2.0)),
            material=hub_gray,
            name="hub",
        )
        wheel.visual(
            mesh_from_geometry(
                TireGeometry(
                    # Tread blocks add their depth outward, so the carcass
                    # radius is reduced to keep the rolling radius at wz.
                    wz - 0.005,
                    tw,
                    inner_radius=wr - 0.006,
                    tread=TireTread(style="block", depth=0.005, count=16 if wz < 0.11 else 18, land_ratio=0.55),
                    grooves=(TireGroove(center_offset=0.0, width=0.005, depth=0.0025),),
                    sidewall=TireSidewall(style="rounded", bulge=0.05),
                ),
                f"{name}_tire",
            ),
            origin=Origin(rpy=(0.0, 0.0, side * pi / 2.0)),
            material=tire_rubber,
            name="tire",
        )
        model.articulation(
            f"{name}_spin",
            ArticulationType.CONTINUOUS,
            parent=deck,
            child=wheel,
            origin=Origin(xyz=(wx, side * 0.25, wz)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=20.0, velocity=35.0),
        )

    # ------------------------------------------------------------- handlebar
    handlebar = model.part("handlebar")
    lower_pts = [
        (0.03, 0.165, -0.075),
        (0.0, 0.165, 0.0),
        (-0.16, 0.175, 0.145),
        (-0.40, 0.190, 0.355),
    ]
    upper_pts = [
        (-0.37, 0.188, 0.329),
        (-0.60, 0.205, 0.530),
        (-0.78, 0.215, 0.690),
        (-0.93, 0.220, 0.830),
    ]
    for i, side in enumerate((1.0, -1.0)):
        handlebar.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _mirror_y(lower_pts, side),
                    radius=0.011,
                    samples_per_segment=10,
                    radial_segments=14,
                ),
                f"handle_lower_tube_{i}",
            ),
            material=accent_red,
            name=f"side_tube_lower_{i}",
        )
        handlebar.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _mirror_y(upper_pts, side),
                    radius=0.011,
                    samples_per_segment=10,
                    radial_segments=14,
                ),
                f"handle_upper_tube_{i}",
            ),
            material=handle_black,
            name=f"side_tube_upper_{i}",
        )
    handlebar.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [(-0.45, -0.196, 0.40), (-0.45, 0.0, 0.40), (-0.45, 0.196, 0.40)],
                radius=0.009,
                samples_per_segment=6,
                radial_segments=12,
            ),
            "handle_cross_brace",
        ),
        material=handle_black,
        name="cross_brace",
    )
    handlebar.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [(-0.93, -0.235, 0.83), (-0.93, 0.0, 0.83), (-0.93, 0.235, 0.83)],
                radius=0.012,
                samples_per_segment=6,
                radial_segments=14,
            ),
            "handle_grip_bar",
        ),
        material=handle_black,
        name="grip_bar",
    )
    handlebar.visual(
        Cylinder(radius=0.018, length=0.26),
        origin=Origin(xyz=(-0.93, 0.0, 0.83), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_foam,
        name="grip_foam",
    )
    # Thin red blade-control bail loop just above/forward of the grip bar.
    handlebar.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (-0.87, -0.2177, 0.770),
                    (-0.95, -0.21, 0.860),
                    (-1.00, 0.0, 0.900),
                    (-0.95, 0.21, 0.860),
                    (-0.87, 0.2177, 0.770),
                ],
                radius=0.005,
                samples_per_segment=10,
                radial_segments=10,
            ),
            "handle_control_bail",
        ),
        material=accent_red,
        name="control_bail",
    )
    # Control cable clipped along the left side tube, deck end to bail.
    handlebar.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (-0.015, 0.152, 0.048),
                    (-0.16, 0.165, 0.145),
                    (-0.40, 0.180, 0.355),
                    (-0.60, 0.195, 0.530),
                    (-0.78, 0.205, 0.690),
                    (-0.86, 0.208, 0.765),
                ],
                radius=0.0035,
                samples_per_segment=10,
                radial_segments=10,
            ),
            "handle_cable",
        ),
        material=cable_black,
        name="cable",
    )

    model.articulation(
        "handlebar_fold",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=handlebar,
        origin=Origin(xyz=HANDLE_PIVOT),
        # Handle geometry extends along local -X/+Z; with axis -Y, negative q
        # folds the handle forward flat over the deck (q=0 is the working pose).
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.5, lower=HANDLE_FOLD_LOWER, upper=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    motor = object_model.get_part("motor")
    battery = object_model.get_part("battery")
    blade = object_model.get_part("blade")
    handlebar = object_model.get_part("handlebar")
    wheels = [object_model.get_part(spec[0]) for spec in WHEEL_SPECS]
    blade_joint = object_model.get_articulation("blade_spin")
    fold_joint = object_model.get_articulation("handlebar_fold")
    front_spin = object_model.get_articulation("front_wheel_0_spin")

    # Intentional captured-fit overlaps.
    ctx.allow_overlap(
        blade,
        motor,
        elem_a="motor_shaft",
        elem_b="motor_housing",
        reason="Motor shaft is intentionally captured in the motor housing; the housing is a simplified proxy without an open bore.",
    )
    for wheel in wheels:
        ctx.allow_overlap(
            deck,
            wheel,
            elem_a=f"{wheel.name}_axle",
            elem_b="hub",
            reason="Stub axle is intentionally captured in the wheel hub bore (interference proxy fit).",
        )
    for i in range(2):
        ctx.allow_overlap(
            deck,
            handlebar,
            elem_a=f"handle_bolt_boss_{i}",
            elem_b=f"side_tube_lower_{i}",
            reason="Handlebar lower tube end is intentionally bolted over the deck pivot bolt boss.",
        )

    # Four wheels stand on the ground plane at the deck corners.
    for (name, wx, side, wz, _tw, _wr, _ww), wheel in zip(WHEEL_SPECS, wheels):
        aabb = ctx.part_world_aabb(wheel)
        ok = aabb is not None and abs(aabb[0][2]) <= 0.006
        ctx.check(f"{name}_touches_ground", ok, f"aabb={aabb!r}")
        pos = ctx.part_world_position(wheel)
        ok = (
            pos is not None
            and abs(pos[0] - wx) < 0.01
            and abs(pos[1] - side * 0.25) < 0.01
            and abs(pos[2] - wz) < 0.01
        )
        ctx.check(f"{name}_at_deck_corner", ok, f"pos={pos!r}")

    # Rear wheels are larger than front wheels (~0.25 m vs ~0.20 m diameter).
    front_aabb = ctx.part_world_aabb(wheels[0])
    rear_aabb = ctx.part_world_aabb(wheels[2])
    if front_aabb is not None and rear_aabb is not None:
        front_d = front_aabb[1][2] - front_aabb[0][2]
        rear_d = rear_aabb[1][2] - rear_aabb[0][2]
        ctx.check(
            "rear_wheels_larger_than_front",
            0.19 <= front_d <= 0.21 and 0.24 <= rear_d <= 0.26,
            f"front_d={front_d:.3f}, rear_d={rear_d:.3f}",
        )
    else:
        ctx.fail("rear_wheels_larger_than_front", "missing wheel AABBs")

    # Overall track width ~0.55 m comes from the wheels.
    y_lo = min(ctx.part_world_aabb(w)[0][1] for w in wheels)
    y_hi = max(ctx.part_world_aabb(w)[1][1] for w in wheels)
    ctx.check("overall_width_about_055", 0.53 <= y_hi - y_lo <= 0.58, f"width={y_hi - y_lo:.3f}")

    # Wheel spin is a pure rotation about the corner axle.
    pos0 = ctx.part_world_position(wheels[0])
    with ctx.pose({front_spin: pi / 3.0}):
        pos1 = ctx.part_world_position(wheels[0])
    ctx.check(
        "wheel_spin_pure_rotation",
        pos0 is not None and pos1 is not None and max(abs(a - b) for a, b in zip(pos0, pos1)) < 1e-6,
        f"pos0={pos0!r}, pos1={pos1!r}",
    )

    # Blade is hidden inside the hollow chamber under the domed deck.
    bar = ctx.part_element_world_aabb(blade, elem="blade_bar")
    ok = bar is not None and bar[0][2] > 0.045 and bar[1][2] < 0.12
    ctx.check("blade_hidden_under_deck", ok, f"blade_bar_aabb={bar!r}")
    ctx.expect_within(blade, deck, axes="xy", inner_elem="blade_bar", outer_elem="shell", margin=0.0, name="blade_inside_deck_footprint")

    # Blade spins about the vertical crankshaft axis.
    if bar is not None:
        rest_x = bar[1][0] - bar[0][0]
        rest_y = bar[1][1] - bar[0][1]
        with ctx.pose({blade_joint: pi / 2.0}):
            turned = ctx.part_element_world_aabb(blade, elem="blade_bar")
        ok = (
            turned is not None
            and rest_x > rest_y
            and (turned[1][1] - turned[0][1]) > (turned[1][0] - turned[0][0])
        )
        ctx.check("blade_spins_about_vertical_axis", ok, f"rest={bar!r}, turned={turned!r}")

    # Motor hub sits centered on the deck spindle boss.
    ctx.expect_contact(motor, deck, elem_a="motor_housing", elem_b="spindle_boss", contact_tol=0.002, name="motor_seated_on_deck_boss")
    mpos = ctx.part_world_position(motor)
    ctx.check(
        "motor_centered_on_deck",
        mpos is not None and abs(mpos[0]) < 0.02 and abs(mpos[1]) < 0.02,
        f"pos={mpos!r}",
    )
    cap = ctx.part_element_world_aabb(motor, elem="motor_cap")
    ctx.check(
        "motor_cap_on_top",
        cap is not None and cap[0][2] > 0.28 and abs((cap[0][0] + cap[1][0]) * 0.5) < 0.05,
        f"cap_aabb={cap!r}",
    )
    
    # Battery pack seated in the deck-top cradle.
    ctx.expect_contact(battery, deck, elem_a="battery_body", elem_b="battery_rail_0", contact_tol=0.003, name="battery_seated_in_cradle")
    bpos = ctx.part_world_position(battery)
    ctx.check(
        "battery_behind_motor",
        bpos is not None and bpos[0] < -0.05 and abs(bpos[1]) < 0.05,
        f"pos={bpos!r}",
    )
    label = ctx.part_element_world_aabb(battery, elem="battery_label")
    ctx.check(
        "battery_has_accent_label",
        label is not None and label[0][2] > 0.25,
        f"label_aabb={label!r}",
    )

    # Handlebar rakes up and back: grips ~1.0 m high, ~1.5 m overall length.
    hb = ctx.part_world_aabb(handlebar)
    grip = ctx.part_element_world_aabb(handlebar, elem="grip_foam")
    ctx.check(
        "grip_height_about_1m",
        grip is not None and 0.93 <= (grip[0][2] + grip[1][2]) * 0.5 <= 1.06,
        f"grip_aabb={grip!r}",
    )
    front_x = max(ctx.part_world_aabb(w)[1][0] for w in wheels[:2])
    ctx.check(
        "overall_length_about_15m",
        hb is not None and 1.40 <= front_x - hb[0][0] <= 1.68,
        f"front_x={front_x:.3f}, rear_x={hb[0][0] if hb else None}",
    )

    # Cable runs along the handlebar from the deck end up to the bail.
    cable = ctx.part_element_world_aabb(handlebar, elem="cable")
    ctx.check(
        "cable_runs_along_handlebar",
        cable is not None and (cable[1][2] - cable[0][2]) > 0.55,
        f"cable_aabb={cable!r}",
    )

    # Folding the handlebar forward lays it flat over the deck for storage:
    # the grip swings from ~1.0 m high behind the mower to low over the front.
    with ctx.pose({fold_joint: HANDLE_FOLD_LOWER}):
        folded_grip = ctx.part_element_world_aabb(handlebar, elem="grip_foam")
    ok = (
        folded_grip is not None
        and grip is not None
        and (folded_grip[0][0] + folded_grip[1][0]) * 0.5 > 0.7
        and (folded_grip[0][2] + folded_grip[1][2]) * 0.5 < 0.55
        and folded_grip[0][2] > 0.05
    )
    ctx.check("handlebar_folds_flat_over_deck", ok, f"folded_grip={folded_grip!r}")

    # Each stub axle stays engaged with its wheel hub along the lateral axis.
    for (name, _wx, _side, _wz, _tw, _wr, _ww), wheel in zip(WHEEL_SPECS, wheels):
        ctx.expect_overlap(
            deck,
            wheel,
            axes="y",
            elem_a=f"{name}_axle",
            elem_b="hub",
            min_overlap=0.005,
            name=f"{name}_axle_engaged_in_hub",
        )

    return ctx.report()


object_model = build_object_model()
