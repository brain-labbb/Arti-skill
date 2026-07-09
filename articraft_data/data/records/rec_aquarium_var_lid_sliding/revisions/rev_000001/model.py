from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


TANK_W = 0.56
TANK_D = 0.32
BASE_H = 0.035
GLASS_H = 0.36
GLASS_T = 0.004
RIM_H = 0.026
POST = 0.018
RAIL_W = 0.026
TOP_Z = BASE_H + GLASS_H
TOP_RIM_TOP = TOP_Z + RIM_H

# Sliding glass canopy dimensions — nearly matches the outer rim footprint
# so the panel edges ride on the top-rim rail surfaces as slide guides.
SLIDE_PANEL_W = TANK_W + 2 * RAIL_W - 0.008
SLIDE_PANEL_D = TANK_D + 2 * RAIL_W - 0.008
SLIDE_PANEL_T = 0.005
SLIDE_TRAVEL = 0.14


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="small_aquarium",
        meta={
            "category_note": "Reference and category both indicate a small aquarium/fish tank.",
            "variant_note": "Sliding glass canopy on top-rim rails (prismatic) replaces rear-hinged hood.",
        },
    )

    glass = model.material("slightly_blue_clear_glass", rgba=(0.72, 0.92, 1.0, 0.32))
    slide_glass = model.material(
        "frosted_canopy_glass", rgba=(0.80, 0.88, 0.93, 0.42)
    )
    black = model.material("satin_black_plastic", rgba=(0.005, 0.005, 0.004, 1.0))
    dark = model.material("dark_control_insert", rgba=(0.030, 0.032, 0.035, 1.0))
    warm_led = model.material("warm_led_diffuser", rgba=(1.0, 0.94, 0.68, 0.80))
    gravel = model.material("mixed_light_gravel", rgba=(0.64, 0.58, 0.45, 1.0))
    gravel_dark = model.material("mixed_dark_gravel", rgba=(0.34, 0.33, 0.29, 1.0))
    red = model.material("red_status_button", rgba=(0.85, 0.08, 0.04, 1.0))
    orange = model.material("orange_status_button", rgba=(0.95, 0.42, 0.05, 1.0))
    green = model.material("green_status_button", rgba=(0.12, 0.68, 0.18, 1.0))

    tank = model.part("tank_frame")

    # Thin transparent panes are modeled as separate held panels rather than a
    # single solid block, so the tank reads as a hollow glass aquarium.
    tank.visual(
        Box((TANK_W, GLASS_T, GLASS_H)),
        origin=Origin(xyz=(0.0, -TANK_D / 2 - GLASS_T / 2, BASE_H + GLASS_H / 2)),
        material=glass,
        name="front_glass",
    )
    tank.visual(
        Box((TANK_W, GLASS_T, GLASS_H)),
        origin=Origin(xyz=(0.0, TANK_D / 2 + GLASS_T / 2, BASE_H + GLASS_H / 2)),
        material=glass,
        name="rear_glass",
    )
    tank.visual(
        Box((GLASS_T, TANK_D, GLASS_H)),
        origin=Origin(xyz=(-TANK_W / 2 - GLASS_T / 2, 0.0, BASE_H + GLASS_H / 2)),
        material=glass,
        name="side_glass_0",
    )
    tank.visual(
        Box((GLASS_T, TANK_D, GLASS_H)),
        origin=Origin(xyz=(TANK_W / 2 + GLASS_T / 2, 0.0, BASE_H + GLASS_H / 2)),
        material=glass,
        name="side_glass_1",
    )
    tank.visual(
        Box((TANK_W, TANK_D, GLASS_T)),
        origin=Origin(xyz=(0.0, 0.0, BASE_H + GLASS_T / 2)),
        material=glass,
        name="bottom_glass",
    )

    # Black plastic lower and upper frames that clamp the panes.
    rail_x = TANK_W + 2 * RAIL_W
    rail_y = TANK_D + 2 * RAIL_W
    tank.visual(
        Box((rail_x, RAIL_W, BASE_H)),
        origin=Origin(xyz=(0.0, -TANK_D / 2 - RAIL_W / 2, BASE_H / 2)),
        material=black,
        name="base_front_rail",
    )
    tank.visual(
        Box((rail_x, RAIL_W, BASE_H)),
        origin=Origin(xyz=(0.0, TANK_D / 2 + RAIL_W / 2, BASE_H / 2)),
        material=black,
        name="base_rear_rail",
    )
    tank.visual(
        Box((RAIL_W, rail_y, BASE_H)),
        origin=Origin(xyz=(-TANK_W / 2 - RAIL_W / 2, 0.0, BASE_H / 2)),
        material=black,
        name="base_side_0",
    )
    tank.visual(
        Box((RAIL_W, rail_y, BASE_H)),
        origin=Origin(xyz=(TANK_W / 2 + RAIL_W / 2, 0.0, BASE_H / 2)),
        material=black,
        name="base_side_1",
    )

    tank.visual(
        Box((rail_x, RAIL_W, RIM_H)),
        origin=Origin(xyz=(0.0, -TANK_D / 2 - RAIL_W / 2, TOP_Z + RIM_H / 2)),
        material=black,
        name="top_front_rail",
    )
    tank.visual(
        Box((rail_x, RAIL_W, RIM_H)),
        origin=Origin(xyz=(0.0, TANK_D / 2 + RAIL_W / 2, TOP_Z + RIM_H / 2)),
        material=black,
        name="top_rear_rail",
    )
    tank.visual(
        Box((RAIL_W, rail_y, RIM_H)),
        origin=Origin(xyz=(-TANK_W / 2 - RAIL_W / 2, 0.0, TOP_Z + RIM_H / 2)),
        material=black,
        name="top_side_0",
    )
    tank.visual(
        Box((RAIL_W, rail_y, RIM_H)),
        origin=Origin(xyz=(TANK_W / 2 + RAIL_W / 2, 0.0, TOP_Z + RIM_H / 2)),
        material=black,
        name="top_side_1",
    )

    for ix, x in enumerate((-TANK_W / 2 - POST / 2, TANK_W / 2 + POST / 2)):
        for iy, y in enumerate((-TANK_D / 2 - POST / 2, TANK_D / 2 + POST / 2)):
            tank.visual(
                Box((POST, POST, GLASS_H + RIM_H)),
                origin=Origin(xyz=(x, y, BASE_H + (GLASS_H + RIM_H) / 2)),
                material=black,
                name=f"corner_post_{ix}_{iy}",
            )

    # Thin outer guide lips on the top side rails that capture the sliding
    # canopy edges and keep it tracking fore-aft.
    guide_w = 0.006
    guide_h = SLIDE_PANEL_T + 0.004
    for idx, sign in enumerate((-1.0, 1.0)):
        tank.visual(
            Box((guide_w, rail_y - 2 * RAIL_W, guide_h)),
            origin=Origin(
                xyz=(
                    sign * (SLIDE_PANEL_W / 2 + guide_w / 2),
                    0.0,
                    TOP_RIM_TOP + guide_h / 2,
                )
            ),
            material=black,
            name=f"slide_guide_{idx}",
        )

    substrate = model.part("substrate")
    substrate.visual(
        Box((TANK_W - 0.055, TANK_D - 0.055, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=gravel,
        name="gravel_bed",
    )
    for i, (x, y, sx, sy, mat) in enumerate(
        (
            (-0.19, -0.08, 0.050, 0.026, gravel_dark),
            (-0.08, 0.06, 0.042, 0.030, gravel),
            (0.08, -0.05, 0.055, 0.022, gravel_dark),
            (0.20, 0.07, 0.046, 0.028, gravel),
        )
    ):
        substrate.visual(
            Box((sx, sy, 0.010)),
            origin=Origin(xyz=(x, y, 0.043)),
            material=mat,
            name=f"gravel_ridge_{i}",
        )
    model.articulation(
        "tank_to_substrate",
        ArticulationType.FIXED,
        parent=tank,
        child=substrate,
        origin=Origin(xyz=(0.0, 0.0, BASE_H + GLASS_T)),
    )

    filter_part = model.part("filter")
    filter_part.visual(
        Box((0.070, 0.075, 0.115)),
        origin=Origin(xyz=(TANK_W / 2 + 0.064, TANK_D / 2 - 0.055, TOP_Z - 0.040)),
        material=black,
        name="filter_housing",
    )
    filter_part.visual(
        Box((0.014, 0.075, 0.070)),
        origin=Origin(xyz=(TANK_W / 2 + RAIL_W + 0.007, TANK_D / 2 - 0.055, TOP_Z - 0.008)),
        material=black,
        name="rim_hanger",
    )
    filter_part.visual(
        Cylinder(radius=0.007, length=0.310),
        origin=Origin(xyz=(TANK_W / 2 - 0.040, TANK_D / 2 - 0.055, BASE_H + 0.200)),
        material=black,
        name="intake_tube",
    )
    filter_part.visual(
        Cylinder(radius=0.0085, length=0.060),
        origin=Origin(xyz=(TANK_W / 2 - 0.040, TANK_D / 2 - 0.055, BASE_H + 0.080)),
        material=black,
        name="strainer_tip",
    )
    filter_part.visual(
        Cylinder(radius=0.006, length=0.105),
        origin=Origin(
            xyz=(TANK_W / 2 + 0.010, TANK_D / 2 - 0.055, TOP_Z - 0.002),
            rpy=(0.0, math.pi / 2, 0.0),
        ),
        material=black,
        name="filter_outlet_elbow",
    )
    model.articulation(
        "tank_to_filter",
        ArticulationType.FIXED,
        parent=tank,
        child=filter_part,
        origin=Origin(),
    )

    # -- Sliding glass canopy (hood) ------------------------------------------
    # The hood part frame sits at the center of the glass panel at rest (q=0).
    # The prismatic joint translates along +Y (fore-aft) to expose the front of
    # the tank for feeding or maintenance access.
    hood = model.part("hood")
    hood.visual(
        Box((SLIDE_PANEL_W, SLIDE_PANEL_D, SLIDE_PANEL_T)),
        origin=Origin(),
        material=slide_glass,
        name="hood_shell",
    )
    # LED light strip bonded to the underside of the canopy.
    hood.visual(
        Box((0.300, 0.025, 0.006)),
        origin=Origin(xyz=(0.0, -0.020, -SLIDE_PANEL_T / 2 - 0.003)),
        material=warm_led,
        name="light_diffuser",
    )
    # Slim control module on top, near the front edge.
    hood.visual(
        Box((0.190, 0.060, 0.004)),
        origin=Origin(
            xyz=(0.0, -SLIDE_PANEL_D / 2 + 0.050, SLIDE_PANEL_T / 2 + 0.002)
        ),
        material=dark,
        name="control_panel",
    )
    for idx, (x, mat) in enumerate(((-0.055, red), (-0.025, orange), (0.010, green))):
        hood.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(
                xyz=(x, -SLIDE_PANEL_D / 2 + 0.050, SLIDE_PANEL_T / 2 + 0.006)
            ),
            material=mat,
            name=f"status_button_{idx}",
        )
    # Small front-edge grip bar so the user can pull the canopy forward/back.
    hood.visual(
        Box((0.080, 0.010, 0.012)),
        origin=Origin(
            xyz=(0.0, -SLIDE_PANEL_D / 2 - 0.005, SLIDE_PANEL_T / 2 + 0.006)
        ),
        material=black,
        name="front_grip",
    )

    model.articulation(
        "tank_to_hood",
        ArticulationType.PRISMATIC,
        parent=tank,
        child=hood,
        origin=Origin(xyz=(0.0, 0.0, TOP_RIM_TOP + SLIDE_PANEL_T / 2)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0, velocity=0.4, lower=0.0, upper=SLIDE_TRAVEL
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tank = object_model.get_part("tank_frame")
    hood = object_model.get_part("hood")
    substrate = object_model.get_part("substrate")
    tank_to_hood = object_model.get_articulation("tank_to_hood")

    # --- Variant-critical: tank_to_hood is prismatic and slides fore-aft ----
    ctx.check(
        "tank_to_hood is prismatic",
        tank_to_hood.articulation_type == ArticulationType.PRISMATIC,
        details=f"got {tank_to_hood.articulation_type}",
    )

    # At rest (q=0), the sliding canopy covers the tank opening.
    ctx.expect_overlap(
        hood,
        tank,
        axes="xy",
        min_overlap=0.020,
        elem_a="hood_shell",
        elem_b="top_front_rail",
        name="sliding canopy covers the top frame footprint at rest",
    )
    ctx.expect_gap(
        hood,
        tank,
        axis="z",
        min_gap=0.0,
        max_gap=0.012,
        positive_elem="hood_shell",
        negative_elem="top_front_rail",
        name="canopy sits just above the top rim rails",
    )

    # Sliding the canopy toward +Y exposes the front of the tank.
    closed_pos = ctx.part_world_position(hood)
    with ctx.pose({tank_to_hood: SLIDE_TRAVEL}):
        open_pos = ctx.part_world_position(hood)
    ctx.check(
        "canopy slides fore-aft along +Y",
        closed_pos is not None
        and open_pos is not None
        and open_pos[1] > closed_pos[1] + SLIDE_TRAVEL - 0.005,
        details=f"closed_y={closed_pos}, open_y={open_pos}",
    )
    ctx.check(
        "canopy slide does not change height",
        closed_pos is not None
        and open_pos is not None
        and abs(open_pos[2] - closed_pos[2]) < 0.002,
        details=f"closed_z={closed_pos[2]}, open_z={open_pos[2]}",
    )

    # --- Preserved baseline checks ------------------------------------------
    ctx.expect_contact(
        substrate,
        tank,
        elem_a="gravel_bed",
        elem_b="bottom_glass",
        contact_tol=0.001,
        name="gravel bed rests on the glass bottom",
    )

    ctx.check(
        "asset focuses on aquarium hardware not animals",
        all(
            "fish" not in part.name and "animal" not in part.name
            for part in object_model.parts
        ),
        details="No animal/character parts should be authored for this asset.",
    )

    return ctx.report()


object_model = build_object_model()
