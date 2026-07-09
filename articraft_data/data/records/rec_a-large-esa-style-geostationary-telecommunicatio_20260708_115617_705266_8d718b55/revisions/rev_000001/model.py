from __future__ import annotations

"""ESA-style dual-reflector telecom satellite (Artemis-class).

Dark box bus with a radiator/solar-cell tiled front face, gold MLI equipment
patches, a small camera cylinder, two long blue multi-segment solar wings on
continuous array drives, and the identity feature: two large white saucer dish
reflectors deployed on articulated arms (one up, one down), each on a revolute
gimbal hinge.
"""

import math

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
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------- dimensions
BUS_X = 1.30  # depth (front face at +X)
BUS_Y = 1.70  # width (wings on +/-Y)
BUS_Z = 1.90  # height (dish arms on +/-Z)

WING_PANEL_COUNT = 3
PANEL_LEN_Y = 1.05
PANEL_HEIGHT_Z = 1.30
PANEL_THICK_X = 0.035
PANEL_GAP = 0.04

DISH_RADIUS = 0.80

WING_SIDES = ("pos_y", "neg_y")
DISH_SIDES = ("upper", "lower")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_reflector_telecom_satellite")

    # ------------------------------------------------------------ materials
    bus_dark = model.material("bus_dark", rgba=(0.21, 0.22, 0.24, 1.0))
    radiator_dark = model.material("radiator_dark", rgba=(0.08, 0.09, 0.13, 1.0))
    mli_gold = model.material("mli_gold", rgba=(0.85, 0.62, 0.13, 1.0))
    panel_blue = model.material("panel_blue", rgba=(0.13, 0.18, 0.52, 1.0))
    strut_silver = model.material("strut_silver", rgba=(0.74, 0.76, 0.79, 1.0))
    dish_white = model.material("dish_white", rgba=(0.92, 0.93, 0.95, 1.0))
    instrument_black = model.material("instrument_black", rgba=(0.07, 0.08, 0.09, 1.0))

    # ------------------------------------------------------------ shared meshes
    # Shallow white saucer reflector, revolved closed profile (r, z):
    # out along the convex back, over the rim, back in along the concave front.
    dish_profile = [
        (0.00, 0.000),
        (0.28, 0.010),
        (0.52, 0.045),
        (0.70, 0.100),
        (0.80, 0.160),
        (0.79, 0.172),
        (0.66, 0.118),
        (0.46, 0.062),
        (0.24, 0.032),
        (0.00, 0.022),
    ]
    dish_mesh = mesh_from_geometry(LatheGeometry(dish_profile, segments=72), "reflector_dish")

    arm_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.00, 0.00, 0.05),
                (0.10, 0.09, 0.28),
                (0.30, 0.20, 0.60),
                (0.47, 0.27, 0.84),
            ],
            radius=0.032,
            samples_per_segment=14,
            radial_segments=16,
            cap_ends=True,
        ),
        "dish_arm_tube",
    )

    # ------------------------------------------------------------ bus (root)
    bus = model.part("bus")
    bus.visual(
        Box((BUS_X, BUS_Y, BUS_Z)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=bus_dark,
        name="bus_body",
    )

    # Front face tiled with dark solar-cell-like radiator plates (3 x 3 grid).
    for row in range(3):
        for col in range(3):
            y = (col - 1) * 0.52
            z = (row - 1) * 0.60
            bus.visual(
                Box((0.02, 0.46, 0.55)),
                origin=Origin(xyz=(BUS_X / 2.0 + 0.005, y, z)),
                material=radiator_dark,
                name=f"front_radiator_tile_r{row}_c{col}",
            )

    # Crinkled gold MLI foil equipment patches near the bus corners.
    mli_patches = [
        ("mli_front_top_left", (0.10, 0.42, 0.30), (BUS_X / 2.0 + 0.04, 0.55, 0.70)),
        ("mli_front_bottom_right", (0.12, 0.36, 0.40), (BUS_X / 2.0 + 0.05, -0.50, -0.55)),
        ("mli_top_deck", (0.30, 0.40, 0.10), (0.10, -0.35, BUS_Z / 2.0 + 0.04)),
        ("mli_bottom_deck", (0.30, 0.35, 0.10), (0.0, 0.45, -(BUS_Z / 2.0 + 0.04))),
    ]
    for name, dims, xyz in mli_patches:
        bus.visual(Box(dims), origin=Origin(xyz=xyz), material=mli_gold, name=name)

    # Small camera / sensor cylinder near the lower front corner.
    bus.visual(
        Cylinder(radius=0.085, length=0.28),
        origin=Origin(xyz=(0.74, -0.50, -0.62), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=strut_silver,
        name="camera_barrel",
    )
    bus.visual(
        Cylinder(radius=0.060, length=0.02),
        origin=Origin(xyz=(0.885, -0.50, -0.62), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=instrument_black,
        name="camera_lens",
    )

    # ------------------------------------------------------------ solar wings
    # Two identical multi-segment blue wings on continuous array-drive joints.
    for side in WING_SIDES:
        sign = 1.0 if side == "pos_y" else -1.0
        wing = model.part(f"solar_wing_{side}")

        # Yoke boom: drive shaft rooted a little inside the bus side wall,
        # reaching into the first panel segment.
        boom_len = 0.485
        wing.visual(
            Cylinder(radius=0.035, length=boom_len),
            origin=Origin(xyz=(0.0, boom_len / 2.0 - 0.015, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=strut_silver,
            name="yoke_boom",
        )
        # A-frame yoke struts fanning from the boom into the first panel.
        for k, z_tip in enumerate((0.45, -0.45)):
            length = math.hypot(0.20, abs(z_tip)) + 0.06
            tilt = math.atan2(0.20, z_tip)  # from +Z toward +Y
            wing.visual(
                Cylinder(radius=0.020, length=length),
                origin=Origin(
                    xyz=(0.0, 0.40, z_tip / 2.0),
                    rpy=(-tilt, 0.0, 0.0),
                ),
                material=strut_silver,
                name=f"yoke_strut_{k}",
            )

        # Blue solar panel segments.
        panel_pitch = PANEL_LEN_Y + PANEL_GAP
        first_center = 0.45 + PANEL_LEN_Y / 2.0
        for i in range(WING_PANEL_COUNT):
            wing.visual(
                Box((PANEL_THICK_X, PANEL_LEN_Y, PANEL_HEIGHT_Z)),
                origin=Origin(xyz=(0.0, first_center + i * panel_pitch, 0.0)),
                material=panel_blue,
                name=f"array_panel_{i}",
            )
        # Hinge spars bridging each inter-panel gap at top and bottom edges.
        for i in range(WING_PANEL_COUNT - 1):
            gap_center = 0.45 + (i + 1) * PANEL_LEN_Y + i * PANEL_GAP + PANEL_GAP / 2.0
            for k, z_edge in enumerate((0.55, -0.55)):
                wing.visual(
                    Box((0.03, 0.14, 0.06)),
                    origin=Origin(xyz=(0.0, gap_center, z_edge)),
                    material=strut_silver,
                    name=f"panel_hinge_spar_{i}_{k}",
                )

        model.articulation(
            f"solar_wing_drive_{side}",
            ArticulationType.CONTINUOUS,
            parent=bus,
            child=wing,
            origin=Origin(
                xyz=(0.0, sign * BUS_Y / 2.0, 0.0),
                rpy=(0.0, 0.0, 0.0 if sign > 0 else math.pi),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=40.0, velocity=0.3),
        )

    # ------------------------------------------------------------ reflector dishes
    # Two large white saucer reflectors on articulated arms: one deployed up
    # over the top deck, one swung down past the bottom deck (built in a local
    # "up" frame; the lower assembly is flipped by the joint origin rpy).
    for side in DISH_SIDES:
        flip = 0.0 if side == "upper" else math.pi
        dish = model.part(f"reflector_{side}")

        # Gimbal hinge barrel, half embedded into the bus deck at the pivot.
        dish.visual(
            Cylinder(radius=0.05, length=0.22),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=strut_silver,
            name="shoulder_barrel",
        )
        # Curved deployment arm from the hinge up to the dish hub.
        dish.visual(arm_mesh, material=strut_silver, name="deploy_arm")
        dish.visual(
            Cylinder(radius=0.09, length=0.06),
            origin=Origin(xyz=(0.48, 0.28, 0.85), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=strut_silver,
            name="dish_hub",
        )
        # White saucer reflector, concave face toward +X (Earth-facing).
        dish.visual(
            dish_mesh,
            origin=Origin(xyz=(0.50, 0.28, 0.85), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dish_white,
            name="dish_saucer",
        )
        # Small dark feed aperture cutout near the rim (off-axis feed).
        dish.visual(
            Cylinder(radius=0.045, length=0.05),
            origin=Origin(xyz=(0.575, -0.14, 0.43), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=instrument_black,
            name="feed_aperture",
        )

        model.articulation(
            f"dish_gimbal_{side}",
            ArticulationType.REVOLUTE,
            parent=bus,
            child=dish,
            origin=Origin(
                xyz=(
                    0.25,
                    0.30 if side == "upper" else -0.30,
                    (BUS_Z / 2.0) * (1.0 if side == "upper" else -1.0),
                ),
                rpy=(flip, 0.0, 0.0),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=25.0, velocity=0.4, lower=-0.35, upper=0.35),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bus = object_model.get_part("bus")
    wings = [object_model.get_part(f"solar_wing_{side}") for side in WING_SIDES]
    dishes = [object_model.get_part(f"reflector_{side}") for side in DISH_SIDES]
    wing_drives = [
        object_model.get_articulation(f"solar_wing_drive_{side}") for side in WING_SIDES
    ]
    gimbals = [object_model.get_articulation(f"dish_gimbal_{side}") for side in DISH_SIDES]

    # Intentional local embeddings: drive shafts and gimbal barrels socketed
    # into the bus walls/decks at their pivots.
    for wing in wings:
        ctx.allow_overlap(
            bus,
            wing,
            elem_a="bus_body",
            elem_b="yoke_boom",
            reason="Solar array drive boom is intentionally socketed into the bus side wall at the drive pivot.",
        )
    for dish in dishes:
        ctx.allow_overlap(
            bus,
            dish,
            elem_a="bus_body",
            elem_b="shoulder_barrel",
            reason="Dish gimbal hinge barrel is intentionally half-embedded into the bus deck at the pivot.",
        )

    # --- two solar wings, opposite sides, correct drive type -------------
    ctx.check("two solar wings present", len(wings) == 2)
    ctx.check(
        "wing drives are continuous",
        all(d.articulation_type == ArticulationType.CONTINUOUS for d in wing_drives),
        details=str([d.articulation_type for d in wing_drives]),
    )
    ctx.expect_origin_gap(
        wings[0], wings[1], axis="y", min_gap=1.5, name="wings on opposite sides of the bus"
    )
    for side, wing in zip(WING_SIDES, wings):
        aabb = ctx.part_world_aabb(wing)
        ctx.check(
            f"wing {side} spans three segments",
            aabb is not None and (aabb[1][1] - aabb[0][1]) > 3.2,
            details=f"aabb={aabb}",
        )

    # --- two reflector dishes, one up and one down, revolute gimbals -----
    ctx.check("two reflector dishes present", len(dishes) == 2)
    ctx.check(
        "dish gimbals are revolute",
        all(g.articulation_type == ArticulationType.REVOLUTE for g in gimbals),
        details=str([g.articulation_type for g in gimbals]),
    )
    upper_aabb = ctx.part_element_world_aabb(dishes[0], elem="dish_saucer")
    lower_aabb = ctx.part_element_world_aabb(dishes[1], elem="dish_saucer")
    ctx.check(
        "upper dish deployed above the bus top deck",
        upper_aabb is not None and upper_aabb[0][2] > BUS_Z / 2.0,
        details=f"aabb={upper_aabb}",
    )
    ctx.check(
        "lower dish deployed below the bus bottom deck",
        lower_aabb is not None and lower_aabb[1][2] < -BUS_Z / 2.0,
        details=f"aabb={lower_aabb}",
    )
    for side, dish in zip(DISH_SIDES, dishes):
        ctx.expect_contact(
            dish,
            dish,
            elem_a="deploy_arm",
            elem_b="dish_hub",
            contact_tol=1e-4,
            name=f"{side} dish arm reaches its hub",
        )

    # --- wing drive actually spins the array ------------------------------
    rest_aabb = ctx.part_world_aabb(wings[0])
    with ctx.pose({wing_drives[0]: math.pi / 2.0}):
        spun_aabb = ctx.part_world_aabb(wings[0])
    ctx.check(
        "wing drive rotates panels out of plane",
        rest_aabb is not None
        and spun_aabb is not None
        and (spun_aabb[1][0] - spun_aabb[0][0]) > 1.0
        and (rest_aabb[1][0] - rest_aabb[0][0]) < 0.6,
        details=f"rest={rest_aabb}, spun={spun_aabb}",
    )

    # --- dish gimbal actually steers the dish ------------------------------
    rest_dish = ctx.part_element_world_aabb(dishes[0], elem="dish_saucer")
    with ctx.pose({gimbals[0]: 0.3}):
        tilted_dish = ctx.part_element_world_aabb(dishes[0], elem="dish_saucer")
    if rest_dish is not None and tilted_dish is not None:
        rest_c = [(rest_dish[0][i] + rest_dish[1][i]) / 2.0 for i in range(3)]
        tilt_c = [(tilted_dish[0][i] + tilted_dish[1][i]) / 2.0 for i in range(3)]
        moved = math.dist(rest_c, tilt_c)
    else:
        moved = 0.0
    ctx.check(
        "upper dish gimbal steers the reflector",
        moved > 0.05,
        details=f"aabb-center displacement={moved:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
