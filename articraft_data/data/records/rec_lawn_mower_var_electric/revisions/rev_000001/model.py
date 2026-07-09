from __future__ import annotations

# Corded electric walk-behind push lawn mower.
#
# Layout (world frame, deck part frame at the origin):
#   +X = forward (deck front), +Y = left, +Z = up.
#   Glossy burnt-orange revolved steel deck shell, hollow underneath, with a
#   domed top, a short side skirt, a rear apron, and four corner axle brackets.
#   A smooth low-profile electric motor housing sits on a spindle boss at the
#   deck center; the cutting blade hangs from the motor shaft inside the
#   chamber. A thin trailing power cord exits the motor rear, runs to a deck
#   cord guide, then clips up the right side of the handlebar.
#   A thin tubular U-handlebar with red lower segments and a control cable
#   sweeps up and back from rear deck brackets and folds forward for storage.
#
# Articulations:
#   - four CONTINUOUS wheel-spin joints about the lateral (Y) axis
#   - one CONTINUOUS blade-spin joint about the vertical motor shaft (Z)
#   - one REVOLUTE handlebar fold joint about the lateral (Y) axis.

from math import cos, pi, sin

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

DECK_TOP_Z = 0.228  # top plane of the spindle boss / motor seating plane
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
    motor_black = model.material("motor_black", rgba=(0.08, 0.08, 0.09, 1.0))
    panel_dark = model.material("panel_dark", rgba=(0.04, 0.04, 0.05, 1.0))
    steel_gray = model.material("steel_gray", rgba=(0.62, 0.63, 0.66, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.55, 0.56, 0.58, 1.0))
    hub_gray = model.material("hub_gray", rgba=(0.55, 0.56, 0.58, 1.0))
    tire_rubber = model.material("tire_rubber", rgba=(0.07, 0.07, 0.08, 1.0))
    handle_black = model.material("handle_black", rgba=(0.10, 0.10, 0.11, 1.0))
    accent_red = model.material("accent_red", rgba=(0.74, 0.14, 0.09, 1.0))
    cable_black = model.material("cable_black", rgba=(0.04, 0.04, 0.04, 1.0))
    grip_foam = model.material("grip_foam", rgba=(0.13, 0.13, 0.14, 1.0))
    cord_orange = model.material("cord_orange", rgba=(0.95, 0.45, 0.05, 1.0))
    clip_black = model.material("clip_black", rgba=(0.06, 0.06, 0.07, 1.0))
    motor_gray = model.material("motor_gray", rgba=(0.30, 0.31, 0.33, 1.0))

    # ------------------------------------------------------------------ deck
    deck = model.part("deck")

    # Hollow revolved shell: short side skirt + smoothly domed top, open at
    # the bottom (the cutting chamber), with a center hole for the motor shaft.
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

    # Annular motor-mount boss with a real shaft bore (r=0.025).
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

    # Deck cord guide: small clip on top of the rear apron routing the power
    # cord from the motor toward the handlebar. Placed to contact the apron top
    # face (apron top z=0.130, guide bottom at z=0.130).
    deck.visual(
        Box((0.022, 0.024, 0.026)),
        origin=Origin(xyz=(-0.265, -0.10, 0.143)),
        material=clip_black,
        name="cord_guide",
    )

    # ---------------------------------------------------------------- motor
    # Motor part frame sits at deck top; all motor visuals use motor-local z
    # where z=0 is the seating plane on the spindle boss.
    motor = model.part("motor")

    # Smooth low-profile motor cover: revolved dome shell with integrated base
    # flange, open at the bottom. The flange widens at the base for mounting.
    motor_cover = LatheGeometry.from_shell_profiles(
        [
            (0.098, 0.000),   # flange outer bottom
            (0.098, 0.010),   # flange outer top
            (0.092, 0.013),   # transition to body
            (0.092, 0.035),   # body straight
            (0.088, 0.065),   # taper start
            (0.078, 0.090),   # dome curve
            (0.060, 0.108),   # dome curve
            (0.036, 0.118),   # near top
            (0.010, 0.123),   # dome apex
        ],
        [
            (0.088, 0.000),   # flange inner bottom
            (0.088, 0.010),   # flange inner top
            (0.086, 0.013),   # transition inner
            (0.086, 0.033),   # body inner
            (0.082, 0.062),   # taper inner
            (0.072, 0.086),   # dome inner
            (0.055, 0.103),   # dome inner
            (0.031, 0.112),   # near top inner
            (0.007, 0.116),   # apex inner
        ],
        segments=64,
    )
    motor.visual(mesh_from_geometry(motor_cover, "motor_cover"), material=motor_black, name="motor_cover")

    # Ventilation slots on the motor cover sides (4 thin recessed strips).
    for i in range(4):
        angle = pi / 4.0 + i * (pi / 2.0)
        motor.visual(
            Box((0.006, 0.002, 0.055)),
            origin=Origin(
                xyz=(0.088 * sin(angle), 0.088 * cos(angle), 0.060),
                rpy=(0.0, 0.0, -angle),
            ),
            material=panel_dark,
            name=f"vent_slot_{i}",
        )

    # Shaft bearing plate: solid disk at the motor base that the motor shaft
    # passes through. Bridges the shaft (blade part) to the motor housing for
    # structural connectivity; the shaft spins inside the bearing bore.
    # Radius matches the cover inner wall so the disk seats flush inside.
    motor.visual(
        Cylinder(radius=0.088, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=motor_gray,
        name="shaft_bearing",
    )

    # Cord exit grommet at the motor rear.
    motor.visual(
        Cylinder(radius=0.012, length=0.016),
        origin=Origin(xyz=(-0.088, 0.0, 0.025), rpy=(0.0, pi / 2.0, 0.0)),
        material=motor_gray,
        name="cord_grommet",
    )

    # Motor label plate (small raised rectangle on top of dome).
    motor.visual(
        Box((0.050, 0.030, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.121)),
        material=motor_gray,
        name="label_plate",
    )

    model.articulation(
        "motor_mount",
        ArticulationType.FIXED,
        parent=deck,
        child=motor,
        origin=Origin(xyz=(0.0, 0.0, DECK_TOP_Z)),
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
        Cylinder(radius=0.012, length=0.175),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
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
        motion_limits=MotionLimits(effort=25.0, velocity=320.0),
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

    # Power cord: runs along the right side (side=-1) of the handlebar from
    # the pivot area up to slightly past the grip, trailing behind the mower.
    handlebar.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (-0.01, -0.155, 0.02),
                    (-0.10, -0.165, 0.10),
                    (-0.25, -0.178, 0.23),
                    (-0.42, -0.192, 0.38),
                    (-0.60, -0.205, 0.535),
                    (-0.78, -0.215, 0.695),
                    (-0.93, -0.222, 0.835),
                    (-1.02, -0.225, 0.895),
                    (-1.06, -0.222, 0.920),
                ],
                radius=0.005,
                samples_per_segment=10,
                radial_segments=10,
            ),
            "power_cord",
        ),
        material=cord_orange,
        name="power_cord",
    )
    # Cord clips: small retaining clips along the right handle tube.
    for i in range(3):
        t = (i + 1) / 4.0
        cx = -0.25 + t * (-0.78 - (-0.25))
        cy = -0.178 + t * (-0.215 - (-0.178))
        cz = 0.23 + t * (0.695 - 0.23)
        handlebar.visual(
            Box((0.020, 0.024, 0.014)),
            origin=Origin(xyz=(cx, cy - 0.016, cz)),
            material=clip_black,
            name=f"cord_clip_{i}",
        )

    model.articulation(
        "handlebar_fold",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=handlebar,
        origin=Origin(xyz=HANDLE_PIVOT),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.5, lower=HANDLE_FOLD_LOWER, upper=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    motor = object_model.get_part("motor")
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
        elem_b="shaft_bearing",
        reason="Motor shaft is intentionally captured inside the bearing plate bore at the motor base; the shaft spins within the bearing housing.",
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

    # Blade spins about the vertical motor shaft axis.
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

    # Motor housing sits centered on the deck spindle boss.
    ctx.expect_contact(motor, deck, elem_a="motor_cover", elem_b="spindle_boss", contact_tol=0.002, name="motor_seated_on_deck_boss")
    mpos = ctx.part_world_position(motor)
    ctx.check(
        "motor_centered_on_deck",
        mpos is not None and abs(mpos[0]) < 0.02 and abs(mpos[1]) < 0.02,
        f"pos={mpos!r}",
    )

    # Motor housing is low-profile: total height well under a gas engine block.
    motor_aabb = ctx.part_world_aabb(motor)
    if motor_aabb is not None:
        motor_height = motor_aabb[1][2] - motor_aabb[0][2]
        ctx.check(
            "motor_housing_low_profile",
            0.10 <= motor_height <= 0.16,
            f"motor_height={motor_height:.3f}",
        )
    else:
        ctx.fail("motor_housing_low_profile", "missing motor AABB")

    # Motor cover is a smooth dome: no recoil starter or fuel cap present.
    motor_visuals = [v.name for v in motor.visuals]
    ctx.check(
        "no_recoil_starter_on_motor",
        "recoil_cover" not in motor_visuals and "recoil_cap" not in motor_visuals,
        f"motor_visuals={motor_visuals}",
    )
    ctx.check(
        "no_fuel_cap_on_motor",
        "fuel_cap" not in motor_visuals and "primer_bulb" not in motor_visuals,
        f"motor_visuals={motor_visuals}",
    )
    ctx.check(
        "motor_has_smooth_cover",
        "motor_cover" in motor_visuals,
        f"motor_visuals={motor_visuals}",
    )

    # Power cord runs along the handlebar from the pivot area to trailing end.
    cord = ctx.part_element_world_aabb(handlebar, elem="power_cord")
    ctx.check(
        "power_cord_runs_along_handle",
        cord is not None and (cord[1][2] - cord[0][2]) > 0.60 and (cord[1][0] - cord[0][0]) > 0.60,
        f"cord_aabb={cord!r}",
    )
    # Cord is on the right side of the handle (negative Y).
    ctx.check(
        "power_cord_on_right_side",
        cord is not None and (cord[0][1] + cord[1][1]) * 0.5 < -0.05,
        f"cord_aabb={cord!r}",
    )
    # Cord clips retain the cord to the handle tube.
    clip_names = [v.name for v in handlebar.visuals]
    ctx.check(
        "cord_has_retaining_clips",
        all(f"cord_clip_{i}" in clip_names for i in range(3)),
        f"handlebar_visuals={clip_names}",
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

    # Folding the handlebar forward lays it flat over the deck for storage.
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
