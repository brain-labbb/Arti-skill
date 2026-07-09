from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# --- Dimensions (meters) -----------------------------------------------------
W = 0.34          # width (X)
D = 0.26          # depth (Y)
H = 0.20          # wall height / top opening (Z)
WALL_T = 0.006
RIM_Z = H - 0.009


def _slot_wall(panel_w, panel_h, name):
    return mesh_from_geometry(
        SlotPatternPanelGeometry(
            (panel_w, panel_h),
            WALL_T,
            slot_size=(0.070, 0.008),
            pitch=(0.090, 0.020),
            frame=0.016,
            corner_radius=0.006,
            stagger=False,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="plastic_shopping_basket")

    blue = model.material("basket_blue", rgba=(0.12, 0.34, 0.85, 1.0))
    blue_dark = model.material("rim_blue", rgba=(0.09, 0.26, 0.66, 1.0))
    black = model.material("handle_black", rgba=(0.10, 0.10, 0.12, 1.0))

    # --- Basket body (root) --------------------------------------------------
    body = model.part("basket_body")
    # base panel
    body.visual(Box((W - 0.004, D - 0.004, 0.010)),
                origin=Origin(xyz=(0.0, 0.0, 0.006)),
                material=blue, name="base_panel")

    # slotted walls (front/back along X, left/right along Y)
    body.visual(_slot_wall(W, H, "wall_front"),
                origin=Origin(xyz=(0.0, -(D / 2.0 - WALL_T / 2.0), H / 2.0),
                              rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=blue, name="wall_front")
    body.visual(_slot_wall(W, H, "wall_back"),
                origin=Origin(xyz=(0.0, (D / 2.0 - WALL_T / 2.0), H / 2.0),
                              rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=blue, name="wall_back")
    body.visual(_slot_wall(D, H, "wall_left"),
                origin=Origin(xyz=(-(W / 2.0 - WALL_T / 2.0), 0.0, H / 2.0),
                              rpy=(math.pi / 2.0, 0.0, math.pi / 2.0)),
                material=blue, name="wall_left")
    body.visual(_slot_wall(D, H, "wall_right"),
                origin=Origin(xyz=((W / 2.0 - WALL_T / 2.0), 0.0, H / 2.0),
                              rpy=(math.pi / 2.0, 0.0, math.pi / 2.0)),
                material=blue, name="wall_right")

    # rolled top rim lip
    body.visual(Box((W + 0.022, 0.024, 0.020)),
                origin=Origin(xyz=(0.0, -(D / 2.0), RIM_Z)),
                material=blue_dark, name="rim_front")
    body.visual(Box((W + 0.022, 0.024, 0.020)),
                origin=Origin(xyz=(0.0, (D / 2.0), RIM_Z)),
                material=blue_dark, name="rim_back")
    body.visual(Box((0.024, D + 0.022, 0.020)),
                origin=Origin(xyz=(-(W / 2.0), 0.0, RIM_Z)),
                material=blue_dark, name="rim_left")
    body.visual(Box((0.024, D + 0.022, 0.020)),
                origin=Origin(xyz=((W / 2.0), 0.0, RIM_Z)),
                material=blue_dark, name="rim_right")

    # --- Two folding carry handles (revolute, pivot on front/back rims) ------
    arm_x = 0.12
    arm_h = 0.17
    handle_specs = [("front", -1.0), ("back", 1.0)]
    for tag, sy in handle_specs:
        handle = model.part(f"handle_{tag}")
        for sx in (-arm_x, arm_x):
            handle.visual(
                Box((0.014, 0.016, arm_h)),
                origin=Origin(xyz=(sx, 0.0, arm_h / 2.0)),
                material=black,
                name=f"arm_{'l' if sx < 0 else 'r'}",
            )
        handle.visual(
            Box((2.0 * arm_x + 0.014, 0.020, 0.016)),
            origin=Origin(xyz=(0.0, 0.0, arm_h)),
            material=black,
            name="grip",
        )
        model.articulation(
            f"handle_{tag}_fold",
            ArticulationType.REVOLUTE,
            parent=body,
            child=handle,
            # pivot just inside the rim; arm bases sit on the rim top (z=H).
            origin=Origin(xyz=(0.0, sy * (D / 2.0 - 0.006), H)),
            axis=(1.0, 0.0, 0.0),
            # q=0 upright; folds down toward its own side (negative q).
            motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=-1.7, upper=0.0),
        )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("basket_body")
    handle_f = object_model.get_part("handle_front")
    handle_b = object_model.get_part("handle_back")
    fold_f = object_model.get_articulation("handle_front_fold")

    # Slotted walls form a hollow open-top basket: opposite walls are separated.
    ctx.expect_gap(
        body, body, axis="y",
        positive_elem="wall_back", negative_elem="wall_front",
        min_gap=0.20, name="front and back walls bound a hollow interior",
    )

    # Base sits below the open rim.
    base_aabb = ctx.part_element_world_aabb(body, elem="base_panel")
    rim_aabb = ctx.part_element_world_aabb(body, elem="rim_front")
    ctx.check(
        "rim opening is above the base",
        base_aabb is not None and rim_aabb is not None
        and rim_aabb[1][2] > base_aabb[1][2] + 0.15,
        details=f"base={base_aabb}, rim={rim_aabb}",
    )

    # Two handles exist.
    ctx.check(
        "two carry handles are present",
        handle_f is not None and handle_b is not None,
        details="expected handle_front and handle_back",
    )

    # Handle stands upright at rest and folds down.
    with ctx.pose({fold_f: 0.0}):
        grip_up = ctx.part_element_world_aabb(handle_f, elem="grip")
    with ctx.pose({fold_f: -1.5}):
        grip_down = ctx.part_element_world_aabb(handle_f, elem="grip")
    ctx.check(
        "carry handle folds down from upright",
        grip_up is not None and grip_down is not None
        and grip_up[1][2] > grip_down[1][2] + 0.10,
        details=f"up={grip_up}, down={grip_down}",
    )

    return ctx.report()
