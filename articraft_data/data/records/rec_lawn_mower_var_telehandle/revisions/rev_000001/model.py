from __future__ import annotations

# Gas-powered walk-behind push lawn mower.
#
# Layout (world frame, deck part frame at the origin):
#   +X = forward (deck front), +Y = left, +Z = up.
#   Glossy burnt-orange revolved steel deck shell, hollow underneath, with a
#   domed top, a short side skirt, a rear apron, and four corner axle brackets.
#   A matte-black single-cylinder engine sits on a spindle boss at the deck
#   center; the cutting blade hangs from the crankshaft inside the chamber.
#   A two-segment height-adjustable telescoping handlebar: a lower handle
#   (red lower tubes, black outer sleeves, cross brace, clamp levers) anchors
#   to the deck and folds forward for storage; an upper grip (thinner black
#   tubes, grip bar, foam wrap, control bail, cable) slides inside the sleeves
#   for height adjustment via a prismatic joint along the handle rake.
#
# Articulations:
#   - four CONTINUOUS wheel-spin joints about the lateral (Y) axis
#   - one CONTINUOUS blade-spin joint about the vertical crankshaft (Z)
#   - one REVOLUTE handlebar fold joint about the lateral (Y) axis
#   - one PRISMATIC handle slide joint along the handle rake direction

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

# Telescoping handle: junction point and prismatic slide parameters.
# The junction is the upper_grip frame origin at q=0, expressed in the
# lower_handle frame (which at fold q=0 coincides with the deck frame
# translated to HANDLE_PIVOT).
HANDLE_JUNCTION = (-0.40, 0.0, 0.355)
# Slide axis: along the handle rake direction (up and back).
HANDLE_SLIDE_AXIS = (-0.745, 0.0, 0.667)
HANDLE_SLIDE_UPPER = 0.12  # 12 cm of height adjustment travel

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
    engine_black = model.material("engine_black", rgba=(0.09, 0.09, 0.10, 1.0))
    panel_dark = model.material("panel_dark", rgba=(0.04, 0.04, 0.05, 1.0))
    steel_gray = model.material("steel_gray", rgba=(0.62, 0.63, 0.66, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.55, 0.56, 0.58, 1.0))
    hub_gray = model.material("hub_gray", rgba=(0.55, 0.56, 0.58, 1.0))
    tire_rubber = model.material("tire_rubber", rgba=(0.07, 0.07, 0.08, 1.0))
    handle_black = model.material("handle_black", rgba=(0.10, 0.10, 0.11, 1.0))
    accent_red = model.material("accent_red", rgba=(0.74, 0.14, 0.09, 1.0))
    cable_black = model.material("cable_black", rgba=(0.04, 0.04, 0.04, 1.0))
    grip_foam = model.material("grip_foam", rgba=(0.13, 0.13, 0.14, 1.0))

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

    # ---------------------------------------------------------------- engine
    engine = model.part("engine")
    engine.visual(
        Box((0.26, 0.24, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=engine_black,
        name="engine_block",
    )
    for i, side in enumerate((1.0, -1.0)):
        engine.visual(
            Box((0.18, 0.014, 0.10)),
            origin=Origin(xyz=(0.0, side * 0.117, 0.085)),
            material=panel_dark,
            name=f"recessed_panel_{i}",
        )
    engine.visual(
        Box((0.22, 0.20, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
        material=engine_black,
        name="top_shroud",
    )
    engine.visual(
        Cylinder(radius=0.082, length=0.034),
        origin=Origin(xyz=(-0.01, 0.0, 0.235)),
        material=panel_dark,
        name="recoil_cover",
    )
    engine.visual(
        Cylinder(radius=0.028, length=0.012),
        origin=Origin(xyz=(-0.01, 0.0, 0.254)),
        material=engine_black,
        name="recoil_cap",
    )
    engine.visual(
        Box((0.05, 0.025, 0.02)),
        origin=Origin(xyz=(0.06, 0.045, 0.243)),
        material=engine_black,
        name="starter_handle",
    )
    engine.visual(
        Cylinder(radius=0.026, length=0.02),
        origin=Origin(xyz=(0.09, -0.072, 0.226)),
        material=panel_dark,
        name="fuel_cap",
    )
    engine.visual(
        Cylinder(radius=0.015, length=0.016),
        origin=Origin(xyz=(0.05, 0.124, 0.05), rpy=(pi / 2.0, 0.0, 0.0)),
        material=accent_red,
        name="primer_bulb",
    )
    engine.visual(
        Box((0.06, 0.05, 0.07)),
        origin=Origin(xyz=(-0.12, 0.105, 0.135)),
        material=panel_dark,
        name="air_filter_box",
    )
    # Throttle cable running from the engine back toward the handle bracket.
    engine.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (-0.09, 0.105, 0.092),
                    (-0.16, 0.120, 0.030),
                    (-0.23, 0.133, -0.005),
                    (-0.285, 0.140, -0.003),
                ],
                radius=0.0035,
                samples_per_segment=10,
                radial_segments=10,
            ),
            "engine_throttle_cable",
        ),
        material=cable_black,
        name="throttle_cable",
    )

    model.articulation(
        "engine_mount",
        ArticulationType.FIXED,
        parent=deck,
        child=engine,
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
    # Crankshaft: rises through the spindle-boss bore (clearance fit) and is
    # intentionally captured inside the engine crankcase proxy.
    blade.visual(
        Cylinder(radius=0.012, length=0.175),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=steel_gray,
        name="crankshaft",
    )

    model.articulation(
        "blade_spin",
        ArticulationType.CONTINUOUS,
        parent=engine,
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

    # ------------------------------------------------------- lower handle
    # Lower handle tube: anchored to the deck via the fold pivot. Carries the
    # red lower side tubes, the black outer sleeves that receive the sliding
    # upper grip, the cross brace, and the clamp levers.
    lower_handle = model.part("lower_handle")

    # Red lower side tubes: from deck brackets up to the sleeve junction.
    lower_tube_pts = [
        (0.03, 0.165, -0.075),
        (0.0, 0.165, 0.0),
        (-0.16, 0.175, 0.145),
        (-0.28, 0.182, 0.233),
    ]
    # Black outer sleeves (larger radius) receive the sliding upper tubes.
    # Start where the red tube ends so the two meshes share an endpoint.
    sleeve_pts = [
        (-0.28, 0.182, 0.233),
        (-0.40, 0.190, 0.355),
        (-0.52, 0.198, 0.475),
    ]

    for i, side in enumerate((1.0, -1.0)):
        lower_handle.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _mirror_y(lower_tube_pts, side),
                    radius=0.011,
                    samples_per_segment=10,
                    radial_segments=14,
                ),
                f"handle_lower_tube_{i}",
            ),
            material=accent_red,
            name=f"side_tube_lower_{i}",
        )
        lower_handle.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _mirror_y(sleeve_pts, side),
                    radius=0.014,
                    samples_per_segment=10,
                    radial_segments=14,
                ),
                f"handle_sleeve_{i}",
            ),
            material=handle_black,
            name=f"sleeve_{i}",
        )
        # Quick-release clamp lever on the outside of each sleeve top.
        lower_handle.visual(
            Box((0.038, 0.020, 0.014)),
            origin=Origin(xyz=(-0.52, side * 0.218, 0.475)),
            material=steel_gray,
            name=f"clamp_lever_{i}",
        )

    # Cross brace connecting the two sides of the lower handle, positioned
    # below the sleeve junction and cleared from the sliding upper tube path.
    # Endpoints match the lower tube Y/Z at this X station for real contact.
    lower_handle.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [(-0.22, -0.180, 0.175), (-0.22, 0.0, 0.175), (-0.22, 0.180, 0.175)],
                radius=0.010,
                samples_per_segment=6,
                radial_segments=12,
            ),
            "handle_cross_brace",
        ),
        material=handle_black,
        name="cross_brace",
    )

    # Fold joint: deck → lower_handle (same pivot as before).
    model.articulation(
        "handlebar_fold",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=lower_handle,
        origin=Origin(xyz=HANDLE_PIVOT),
        # Handle geometry extends along local -X/+Z; with axis -Y, negative q
        # folds the handle forward flat over the deck (q=0 is the working pose).
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.5, lower=HANDLE_FOLD_LOWER, upper=0.0),
    )

    # ------------------------------------------------------- upper grip
    # Upper grip tube: slides inside the lower handle sleeves for height
    # adjustment. Carries the thinner upper side tubes, grip bar, foam wrap,
    # control bail, and the control cable.
    upper_grip = model.part("upper_grip")

    # Junction offset: upper_grip frame origin at q=0 coincides with the
    # prismatic articulation frame at HANDLE_JUNCTION in the lower_handle frame.
    jx, jy, jz = HANDLE_JUNCTION

    # Upper side tubes (thinner to slide inside the sleeves). The lower end
    # extends well into the sleeve so the tube remains inserted at max travel.
    upper_tube_pts = [
        (-0.30 - jx, 0.184 - jy, 0.255 - jz),
        (-0.50 - jx, 0.197 - jy, 0.455 - jz),
        (-0.70 - jx, 0.210 - jy, 0.630 - jz),
        (-0.93 - jx, 0.220 - jy, 0.830 - jz),
    ]

    for i, side in enumerate((1.0, -1.0)):
        upper_grip.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _mirror_y(upper_tube_pts, side),
                    radius=0.009,
                    samples_per_segment=10,
                    radial_segments=14,
                ),
                f"handle_upper_tube_{i}",
            ),
            material=handle_black,
            name=f"side_tube_upper_{i}",
        )

    # Grip bar (U-shaped cross tube at the top).
    upper_grip.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (-0.93 - jx, -0.235 - jy, 0.83 - jz),
                    (-0.93 - jx, 0.0 - jy, 0.83 - jz),
                    (-0.93 - jx, 0.235 - jy, 0.83 - jz),
                ],
                radius=0.012,
                samples_per_segment=6,
                radial_segments=14,
            ),
            "handle_grip_bar",
        ),
        material=handle_black,
        name="grip_bar",
    )

    # Foam grip wrap around the grip bar.
    upper_grip.visual(
        Cylinder(radius=0.018, length=0.26),
        origin=Origin(xyz=(-0.93 - jx, 0.0 - jy, 0.83 - jz), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_foam,
        name="grip_foam",
    )

    # Thin red blade-control bail loop just above/forward of the grip bar.
    upper_grip.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (-0.87 - jx, -0.2177 - jy, 0.770 - jz),
                    (-0.95 - jx, -0.21 - jy, 0.860 - jz),
                    (-1.00 - jx, 0.0 - jy, 0.900 - jz),
                    (-0.95 - jx, 0.21 - jy, 0.860 - jz),
                    (-0.87 - jx, 0.2177 - jy, 0.770 - jz),
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

    # Control cable clipped along the left side tube, from the sleeve
    # junction upward to the bail (stays within the upper grip zone).
    upper_grip.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (-0.44 - jx, 0.193 - jy, 0.395 - jz),
                    (-0.60 - jx, 0.195 - jy, 0.530 - jz),
                    (-0.78 - jx, 0.205 - jy, 0.690 - jz),
                    (-0.86 - jx, 0.208 - jy, 0.765 - jz),
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

    # Prismatic slide joint: lower_handle → upper_grip (height adjustment).
    # Positive q extends the grip upward and backward along the handle rake.
    model.articulation(
        "handle_slide",
        ArticulationType.PRISMATIC,
        parent=lower_handle,
        child=upper_grip,
        origin=Origin(xyz=HANDLE_JUNCTION),
        axis=HANDLE_SLIDE_AXIS,
        motion_limits=MotionLimits(effort=30.0, velocity=0.15, lower=0.0, upper=HANDLE_SLIDE_UPPER),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    engine = object_model.get_part("engine")
    blade = object_model.get_part("blade")
    lower_handle = object_model.get_part("lower_handle")
    upper_grip = object_model.get_part("upper_grip")
    wheels = [object_model.get_part(spec[0]) for spec in WHEEL_SPECS]
    blade_joint = object_model.get_articulation("blade_spin")
    fold_joint = object_model.get_articulation("handlebar_fold")
    slide_joint = object_model.get_articulation("handle_slide")
    front_spin = object_model.get_articulation("front_wheel_0_spin")

    # Intentional captured-fit overlaps.
    ctx.allow_overlap(
        blade,
        engine,
        elem_a="crankshaft",
        elem_b="engine_block",
        reason="Crankshaft is intentionally captured in the engine crankcase bore; the bore is not modeled in the block proxy.",
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
            lower_handle,
            elem_a=f"handle_bolt_boss_{i}",
            elem_b=f"side_tube_lower_{i}",
            reason="Lower handle tube end is intentionally bolted over the deck pivot bolt boss.",
        )
        ctx.allow_overlap(
            lower_handle,
            upper_grip,
            elem_a=f"sleeve_{i}",
            elem_b=f"side_tube_upper_{i}",
            reason="Upper grip tube slides inside the lower handle sleeve (telescoping prismatic fit).",
        )
        ctx.allow_overlap(
            lower_handle,
            upper_grip,
            elem_a=f"sleeve_{i}",
            elem_b="cable",
            reason="Control cable runs along the sliding upper grip and naturally passes through the sleeve zone (representation artifact of telescoping assembly).",
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

    # Engine sits centered on the deck spindle boss.
    ctx.expect_contact(engine, deck, elem_a="engine_block", elem_b="spindle_boss", contact_tol=0.002, name="engine_seated_on_deck_boss")
    epos = ctx.part_world_position(engine)
    ctx.check(
        "engine_centered_on_deck",
        epos is not None and abs(epos[0]) < 0.02 and abs(epos[1]) < 0.02,
        f"pos={epos!r}",
    )
    recoil = ctx.part_element_world_aabb(engine, elem="recoil_cover")
    ctx.check(
        "recoil_cover_on_engine_top",
        recoil is not None and recoil[0][2] > 0.42 and abs((recoil[0][0] + recoil[1][0]) * 0.5) < 0.05,
        f"recoil_aabb={recoil!r}",
    )
    cap = ctx.part_element_world_aabb(engine, elem="fuel_cap")
    ctx.check("fuel_cap_on_engine_top", cap is not None and cap[0][2] > 0.42, f"fuel_cap_aabb={cap!r}")

    # -------------------------------------------------- telescoping handle
    # Handle rakes up and back: grips ~1.0 m high, ~1.5 m overall length.
    # The overall handle AABB spans both lower_handle and upper_grip.
    lower_aabb = ctx.part_world_aabb(lower_handle)
    upper_aabb = ctx.part_world_aabb(upper_grip)
    grip = ctx.part_element_world_aabb(upper_grip, elem="grip_foam")
    ctx.check(
        "grip_height_about_1m",
        grip is not None and 0.93 <= (grip[0][2] + grip[1][2]) * 0.5 <= 1.06,
        f"grip_aabb={grip!r}",
    )
    front_x = max(ctx.part_world_aabb(w)[1][0] for w in wheels[:2])
    handle_rear_x = min(aabb[0][0] for aabb in (lower_aabb, upper_aabb) if aabb is not None)
    ctx.check(
        "overall_length_about_15m",
        lower_aabb is not None and upper_aabb is not None and 1.40 <= front_x - handle_rear_x <= 1.68,
        f"front_x={front_x:.3f}, rear_x={handle_rear_x}",
    )

    # Cable runs along the upper grip from the sleeve junction up to the bail.
    cable = ctx.part_element_world_aabb(upper_grip, elem="cable")
    ctx.check(
        "cable_runs_along_handlebar",
        cable is not None and (cable[1][2] - cable[0][2]) > 0.25,
        f"cable_aabb={cable!r}",
    )

    # Telescoping slide: upper grip extends along the handle rake direction.
    rest_grip_pos = ctx.part_world_position(upper_grip)
    with ctx.pose({slide_joint: HANDLE_SLIDE_UPPER}):
        extended_grip_pos = ctx.part_world_position(upper_grip)
        extended_grip_aabb = ctx.part_element_world_aabb(upper_grip, elem="grip_foam")
    ctx.check(
        "handle_slide_extends_grip_upward",
        rest_grip_pos is not None
        and extended_grip_pos is not None
        and extended_grip_pos[2] > rest_grip_pos[2] + 0.02,
        f"rest={rest_grip_pos!r}, extended={extended_grip_pos!r}",
    )
    # At full extension, grip height increases by approximately 0.08 m.
    ctx.check(
        "handle_slide_full_extension_range",
        rest_grip_pos is not None
        and extended_grip_pos is not None
        and 0.04 <= extended_grip_pos[2] - rest_grip_pos[2] <= 0.12,
        f"delta_z={extended_grip_pos[2] - rest_grip_pos[2]:.3f}",
    )

    # Upper grip tubes remain inserted in the lower sleeves at both rest
    # and full extension (retained insertion check).
    ctx.expect_overlap(
        upper_grip,
        lower_handle,
        axes="z",
        elem_a="side_tube_upper_0",
        elem_b="sleeve_0",
        min_overlap=0.01,
        name="upper_tube_0_inserted_in_sleeve_at_rest",
    )
    with ctx.pose({slide_joint: HANDLE_SLIDE_UPPER}):
        ctx.expect_overlap(
            upper_grip,
            lower_handle,
            axes="z",
            elem_a="side_tube_upper_0",
            elem_b="sleeve_0",
            min_overlap=0.005,
            name="upper_tube_0_retains_insertion_at_full_extension",
        )

    # Folding the handle forward lays it flat over the deck for storage.
    # The fold joint drives both lower_handle and its child upper_grip.
    with ctx.pose({fold_joint: HANDLE_FOLD_LOWER}):
        folded_grip = ctx.part_element_world_aabb(upper_grip, elem="grip_foam")
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
