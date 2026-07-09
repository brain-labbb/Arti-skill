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

# Split glass canopy geometry
LEAF_T = 0.004  # glass panel thickness
LEAF_W = TANK_W - 0.014  # fits inside top side rails with small clearance
FRONT_HINGE_Y = 0.0  # center of tank opening
FRONT_HINGE_Z = TOP_RIM_TOP + 0.004  # just above the top rim
CENTER_BRACE_H = FRONT_HINGE_Z - TOP_Z  # brace fills from top of glass to hinge line
LEAF_DEPTH_FRONT = TANK_D / 2 - 0.008  # front leaf covers front half
REAR_HINGE_Y = TANK_D / 2 + 0.025  # at rear rim (matches existing knuckle Y)
REAR_HINGE_Z = TOP_RIM_TOP + 0.008
LEAF_DEPTH_REAR = REAR_HINGE_Y - 0.003  # rear leaf covers rear half to center gap


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="small_aquarium",
        meta={
            "category_note": "Reference and category both indicate a small aquarium/fish tank.",
            "variant_note": "Split glass canopy with twin independently hinged lid leaves.",
        },
    )

    glass = model.material("slightly_blue_clear_glass", rgba=(0.72, 0.92, 1.0, 0.32))
    black = model.material("satin_black_plastic", rgba=(0.005, 0.005, 0.004, 1.0))
    dark = model.material("dark_control_insert", rgba=(0.030, 0.032, 0.035, 1.0))
    warm_led = model.material("warm_led_diffuser", rgba=(1.0, 0.94, 0.68, 0.80))
    gravel = model.material("mixed_light_gravel", rgba=(0.64, 0.58, 0.45, 1.0))
    gravel_dark = model.material("mixed_dark_gravel", rgba=(0.34, 0.33, 0.29, 1.0))
    canopy_glass = model.material(
        "canopy_glass_tint", rgba=(0.82, 0.90, 0.94, 0.38)
    )

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

    # Fixed knuckles on the rear top rim for the rear leaf hinge.
    for idx, x in enumerate((-0.19, 0.19)):
        tank.visual(
            Cylinder(radius=0.008, length=0.105),
            origin=Origin(
                xyz=(x, REAR_HINGE_Y, REAR_HINGE_Z),
                rpy=(0.0, math.pi / 2, 0.0),
            ),
            material=black,
            name=f"rear_hinge_knuckle_{idx}",
        )

    # Center cross-brace spanning the tank opening at the midpoint in Y.
    # This provides the physical mounting surface for the front leaf hinge.
    brace_w = TANK_W + 2 * RAIL_W + 0.004  # spans past the side rails for contact
    brace_d = 0.030
    tank.visual(
        Box((brace_w, brace_d, CENTER_BRACE_H)),
        origin=Origin(
            xyz=(0.0, FRONT_HINGE_Y, TOP_Z + CENTER_BRACE_H / 2)
        ),
        material=black,
        name="center_cross_brace",
    )

    # Center hinge mounts on the cross-brace for the front leaf.
    for idx, x in enumerate((-0.19, 0.19)):
        tank.visual(
            Cylinder(radius=0.007, length=0.080),
            origin=Origin(
                xyz=(x, FRONT_HINGE_Y, FRONT_HINGE_Z),
                rpy=(0.0, math.pi / 2, 0.0),
            ),
            material=black,
            name=f"center_hinge_knuckle_{idx}",
        )

    substrate = model.part("substrate")
    substrate.visual(
        Box((TANK_W - 0.055, TANK_D - 0.055, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=gravel,
        name="gravel_bed",
    )
    # A few partially embedded pebble ridges keep the substrate from reading as a
    # flat solid slab without turning the aquarium into a scenic diorama.
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

    # --- Split glass canopy: two independent rear-hinged lid leaves ---
    # Each leaf is a thin glass panel covering half the top opening,
    # mounted on its own revolute hinge at the rear edge of that panel.

    for leaf_idx, (leaf_name, hinge_y, hinge_z, leaf_depth) in enumerate(
        (
            ("lid_leaf_front", FRONT_HINGE_Y, FRONT_HINGE_Z, LEAF_DEPTH_FRONT),
            ("lid_leaf_rear", REAR_HINGE_Y, REAR_HINGE_Z, LEAF_DEPTH_REAR),
        )
    ):
        leaf = model.part(leaf_name)

        # Glass panel extends in -Y from the hinge line (its rear edge).
        leaf.visual(
            Box((LEAF_W, leaf_depth, LEAF_T)),
            origin=Origin(xyz=(0.0, -leaf_depth / 2, 0.0)),
            material=canopy_glass,
            name=f"{leaf_name}_panel",
        )

        # Thin black plastic edge trim along the front edge for grip.
        leaf.visual(
            Box((LEAF_W, 0.012, LEAF_T + 0.003)),
            origin=Origin(xyz=(0.0, -leaf_depth + 0.006, 0.0)),
            material=black,
            name=f"{leaf_name}_front_trim",
        )

        # Hinge knuckles on the leaf (rotate with it).
        for kx in (-0.12, 0.12):
            leaf.visual(
                Cylinder(radius=0.006, length=0.045),
                origin=Origin(
                    xyz=(kx, 0.0, 0.0),
                    rpy=(0.0, math.pi / 2, 0.0),
                ),
                material=black,
                name=f"{leaf_name}_knuckle_{0 if kx < 0 else 1}",
            )

        model.articulation(
            f"tank_to_{leaf_name}",
            ArticulationType.REVOLUTE,
            parent=tank,
            child=leaf,
            origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
            # axis=(-1,0,0): positive q rotates panel in -Y upward (+Z).
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0, velocity=2.5, lower=0.0, upper=1.30
            ),
        )

    # Light diffuser bar mounted under the front leaf.
    lid_front = model.get_part("lid_leaf_front")
    lid_front.visual(
        Box((0.300, 0.025, 0.006)),
        origin=Origin(
            xyz=(0.0, -LEAF_DEPTH_FRONT + 0.040, -LEAF_T / 2 - 0.003)
        ),
        material=warm_led,
        name="light_diffuser",
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tank = object_model.get_part("tank_frame")
    substrate = object_model.get_part("substrate")
    lid_front = object_model.get_part("lid_leaf_front")
    lid_rear = object_model.get_part("lid_leaf_rear")
    tank_to_front = object_model.get_articulation("tank_to_lid_leaf_front")
    tank_to_rear = object_model.get_articulation("tank_to_lid_leaf_rear")

    # The front leaf hinge knuckles sit partially embedded in the cross-brace
    # top surface — a captured-pin mounting that represents the hinge pivot.
    ctx.allow_overlap(
        lid_front,
        tank,
        elem_a="lid_leaf_front_knuckle_0",
        elem_b="center_cross_brace",
        reason="Front leaf hinge knuckle is intentionally seated in the cross-brace pivot recess.",
    )
    ctx.allow_overlap(
        lid_front,
        tank,
        elem_a="lid_leaf_front_knuckle_1",
        elem_b="center_cross_brace",
        reason="Front leaf hinge knuckle is intentionally seated in the cross-brace pivot recess.",
    )
    # Proof: knuckle stays in contact with the cross-brace (seated, not floating away).
    ctx.expect_contact(
        lid_front,
        tank,
        elem_a="lid_leaf_front_knuckle_0",
        elem_b="center_cross_brace",
        contact_tol=0.008,
        name="front knuckle 0 remains seated on cross-brace",
    )

    # Gravel bed rests on glass bottom (unchanged).
    ctx.expect_contact(
        substrate,
        tank,
        elem_a="gravel_bed",
        elem_b="bottom_glass",
        contact_tol=0.001,
        name="gravel bed rests on the glass bottom",
    )

    # --- Split canopy: both leaves cover the top opening ---
    ctx.expect_overlap(
        lid_front,
        tank,
        axes="x",
        min_overlap=0.20,
        elem_a="lid_leaf_front_panel",
        elem_b="top_front_rail",
        name="front leaf spans the tank width at the top",
    )
    ctx.expect_overlap(
        lid_rear,
        tank,
        axes="x",
        min_overlap=0.20,
        elem_a="lid_leaf_rear_panel",
        elem_b="top_rear_rail",
        name="rear leaf spans the tank width at the top",
    )

    # Closed leaves sit just above the top rim.
    ctx.expect_gap(
        lid_front,
        tank,
        axis="z",
        min_gap=-0.005,
        max_gap=0.020,
        positive_elem="lid_leaf_front_panel",
        negative_elem="top_front_rail",
        name="closed front leaf sits near the front rim",
    )
    ctx.expect_gap(
        lid_rear,
        tank,
        axis="z",
        min_gap=-0.005,
        max_gap=0.020,
        positive_elem="lid_leaf_rear_panel",
        negative_elem="top_rear_rail",
        name="closed rear leaf sits near the rear rim",
    )

    # --- Each leaf opens independently upward ---
    closed_front = ctx.part_element_world_aabb(lid_front, elem="lid_leaf_front_panel")
    with ctx.pose({tank_to_front: 1.0}):
        open_front = ctx.part_element_world_aabb(lid_front, elem="lid_leaf_front_panel")
    ctx.check(
        "front leaf hinge opens upward",
        closed_front is not None
        and open_front is not None
        and open_front[1][2] > closed_front[1][2] + 0.08,
        details=f"closed={closed_front}, open={open_front}",
    )

    closed_rear = ctx.part_element_world_aabb(lid_rear, elem="lid_leaf_rear_panel")
    with ctx.pose({tank_to_rear: 1.0}):
        open_rear = ctx.part_element_world_aabb(lid_rear, elem="lid_leaf_rear_panel")
    ctx.check(
        "rear leaf hinge opens upward",
        closed_rear is not None
        and open_rear is not None
        and open_rear[1][2] > closed_rear[1][2] + 0.08,
        details=f"closed={closed_rear}, open={open_rear}",
    )

    # Independent actuation: opening one leaf does not move the other.
    with ctx.pose({tank_to_front: 0.8}):
        rear_still_closed = ctx.part_element_world_aabb(
            lid_rear, elem="lid_leaf_rear_panel"
        )
    ctx.check(
        "rear leaf stays closed when front leaf opens",
        closed_rear is not None
        and rear_still_closed is not None
        and abs(rear_still_closed[1][2] - closed_rear[1][2]) < 0.005,
        details=f"closed={closed_rear}, with_front_open={rear_still_closed}",
    )

    # Light diffuser is mounted under the front leaf and moves with it.
    closed_diffuser = ctx.part_element_world_aabb(lid_front, elem="light_diffuser")
    with ctx.pose({tank_to_front: 1.0}):
        open_diffuser = ctx.part_element_world_aabb(lid_front, elem="light_diffuser")
    ctx.check(
        "light diffuser moves with front leaf",
        closed_diffuser is not None
        and open_diffuser is not None
        and open_diffuser[1][2] > closed_diffuser[1][2] + 0.04,
        details=f"closed={closed_diffuser}, open={open_diffuser}",
    )

    ctx.check(
        "asset focuses on aquarium hardware not animals",
        all(
            "fish" not in p.name and "animal" not in p.name
            for p in object_model.parts
        ),
        details="No animal/character parts should be authored for this asset.",
    )

    return ctx.report()


object_model = build_object_model()
