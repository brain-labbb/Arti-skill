from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _rounded_ring(outer_w: float, outer_h: float, inner_w: float, inner_h: float, depth: float):
    """Rounded rectangular through-frame in local X/Z, extruded through Y."""
    return (
        cq.Workplane("XZ")
        .rect(outer_w, outer_h)
        .rect(inner_w, inner_h)
        .extrude(depth / 2.0, both=True)
        .edges("|Y")
        .fillet(0.018)
    )


def _rounded_panel(width: float, height: float, thickness: float):
    """Thin rounded rectangle in local X/Z, extruded through Y."""
    return (
        cq.Workplane("XZ")
        .rect(width, height)
        .extrude(thickness / 2.0, both=True)
        .edges("|Y")
        .fillet(0.018)
    )


def _add_flap(model, frame, *, index: int, hinge_y: float, panel_y_bias: float, flap_material):
    """Shared builder for one swinging flap part with its hinge and magnet keeper.

    Both flaps share the same rounded-panel geometry, hinge barrel, keeper,
    and top seal; only the Y placement (front vs rear face of the tunnel)
    and tint differ.
    """
    white_plastic = Material("warm_white_plastic", rgba=(0.92, 0.90, 0.84, 1.0))
    screw_metal = Material("brushed_screw_heads", rgba=(0.55, 0.55, 0.52, 1.0))
    dark_rubber = Material("dark_rubber_gasket", rgba=(0.015, 0.014, 0.012, 1.0))

    flap = model.part(f"flap_{index}")

    # Translucent swing panel – hangs below the hinge line.
    flap.visual(
        mesh_from_cadquery(_rounded_panel(0.268, 0.282, 0.005), f"translucent_swing_flap_{index}"),
        origin=Origin(xyz=(0.0, panel_y_bias, -0.149)),
        material=flap_material,
        name=f"flap_panel_{index}",
    )

    # Hinge barrel centred at the part origin (= articulation frame).
    flap.visual(
        Cylinder(radius=0.009, length=0.184),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=white_plastic,
        name=f"moving_hinge_barrel_{index}",
    )

    # Magnet keeper plate at the bottom edge of the flap.
    flap.visual(
        Box((0.090, 0.004, 0.014)),
        origin=Origin(xyz=(0.0, panel_y_bias * 0.5, -0.282)),
        material=screw_metal,
        name=f"magnet_keeper_{index}",
    )

    # Thin rubber top seal strip near the hinge.
    flap.visual(
        Box((0.252, 0.004, 0.008)),
        origin=Origin(xyz=(0.0, panel_y_bias / 3.0, -0.014)),
        material=dark_rubber,
        name=f"top_seal_{index}",
    )

    # Revolute hinge at the tunnel top edge.
    model.articulation(
        f"frame_to_flap_{index}",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=flap,
        origin=Origin(xyz=(0.0, hinge_y, 0.348)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=2.5, lower=-1.05, upper=1.05),
    )

    return flap


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="cat_flap_panel",
        meta={
            "reference_note": (
                "Reference and category both indicate a cat flap / pet door; "
                "no classification mismatch suspected."
            )
        },
    )

    white_plastic = Material("warm_white_plastic", rgba=(0.92, 0.90, 0.84, 1.0))
    glass = Material("slightly_green_glass", rgba=(0.58, 0.78, 0.82, 0.30))
    smoke_front = Material("smoky_translucent_flap", rgba=(0.36, 0.30, 0.24, 0.42))
    smoke_rear = Material("smoky_translucent_flap_rear", rgba=(0.32, 0.28, 0.22, 0.38))
    dark_rubber = Material("dark_rubber_gasket", rgba=(0.015, 0.014, 0.012, 1.0))
    screw_metal = Material("brushed_screw_heads", rgba=(0.55, 0.55, 0.52, 1.0))
    magnet = Material("black_magnet", rgba=(0.02, 0.02, 0.018, 1.0))

    # ------------------------------------------------------------------
    # Frame (root part) – surrounding panel, trim, tunnel, hardware
    # ------------------------------------------------------------------
    frame = model.part("panel_frame")

    # Minimal surrounding door/glass panel, cut away around the pet door.
    frame.visual(
        Box((0.19, 0.012, 0.58)),
        origin=Origin(xyz=(-0.285, 0.0, 0.18)),
        material=glass,
        name="panel_side_0",
    )
    frame.visual(
        Box((0.19, 0.012, 0.58)),
        origin=Origin(xyz=(0.285, 0.0, 0.18)),
        material=glass,
        name="panel_side_1",
    )
    frame.visual(
        Box((0.38, 0.012, 0.09)),
        origin=Origin(xyz=(0.0, 0.0, 0.435)),
        material=glass,
        name="panel_top",
    )
    frame.visual(
        Box((0.38, 0.012, 0.09)),
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
        material=glass,
        name="panel_bottom",
    )

    # White molded trim: a rounded rectangular ring with a real through-opening.
    frame.visual(
        mesh_from_cadquery(_rounded_ring(0.44, 0.42, 0.30, 0.32, 0.060), "molded_trim"),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=white_plastic,
        name="front_trim",
    )

    # Inner tunnel/sleeve surfaces visible through the opening.
    frame.visual(
        Box((0.018, 0.090, 0.315)),
        origin=Origin(xyz=(-0.159, 0.0, 0.18)),
        material=white_plastic,
        name="tunnel_side_0",
    )
    frame.visual(
        Box((0.018, 0.090, 0.315)),
        origin=Origin(xyz=(0.159, 0.0, 0.18)),
        material=white_plastic,
        name="tunnel_side_1",
    )
    frame.visual(
        Box((0.318, 0.060, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.349)),
        material=white_plastic,
        name="tunnel_top",
    )
    frame.visual(
        Box((0.318, 0.090, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=white_plastic,
        name="tunnel_sill",
    )

    # ---- Front-face gaskets and magnet (outer entry) ----
    frame.visual(
        Box((0.006, 0.006, 0.305)),
        origin=Origin(xyz=(-0.147, -0.034, 0.18)),
        material=dark_rubber,
        name="gasket_side_0",
    )
    frame.visual(
        Box((0.006, 0.006, 0.305)),
        origin=Origin(xyz=(0.147, -0.034, 0.18)),
        material=dark_rubber,
        name="gasket_side_1",
    )
    frame.visual(
        Box((0.255, 0.007, 0.006)),
        origin=Origin(xyz=(0.0, -0.0335, 0.337)),
        material=dark_rubber,
        name="top_gasket",
    )
    frame.visual(
        Box((0.070, 0.012, 0.016)),
        origin=Origin(xyz=(0.0, -0.038, 0.029)),
        material=magnet,
        name="bottom_magnet",
    )

    # ---- Rear-face gaskets and magnet (inner entry) ----
    frame.visual(
        Box((0.006, 0.006, 0.305)),
        origin=Origin(xyz=(-0.147, 0.034, 0.18)),
        material=dark_rubber,
        name="rear_gasket_side_0",
    )
    frame.visual(
        Box((0.006, 0.006, 0.305)),
        origin=Origin(xyz=(0.147, 0.034, 0.18)),
        material=dark_rubber,
        name="rear_gasket_side_1",
    )
    frame.visual(
        Box((0.255, 0.007, 0.006)),
        origin=Origin(xyz=(0.0, 0.0335, 0.337)),
        material=dark_rubber,
        name="rear_top_gasket",
    )
    frame.visual(
        Box((0.070, 0.012, 0.016)),
        origin=Origin(xyz=(0.0, 0.038, 0.029)),
        material=magnet,
        name="rear_bottom_magnet",
    )

    # Surface screws – seated into the plastic trim (front face only).
    screw_positions = [
        (-0.168, 0.334),
        (0.168, 0.334),
        (-0.168, 0.026),
        (0.168, 0.026),
        (-0.205, 0.180),
        (0.205, 0.180),
    ]
    for i, (x, z) in enumerate(screw_positions):
        frame.visual(
            Cylinder(radius=0.010, length=0.004),
            origin=Origin(xyz=(x, -0.032, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=screw_metal,
            name=f"screw_{i}",
        )
        frame.visual(
            Box((0.014, 0.002, 0.002)),
            origin=Origin(xyz=(x, -0.035, z), rpy=(0.0, 0.0, math.pi / 6.0)),
            material=dark_rubber,
            name=f"screw_slot_{i}",
        )

    # ---- Front-face fixed hinge knuckles (keep parent names) ----
    for i, x in enumerate((-0.165, 0.165)):
        frame.visual(
            Box((0.055, 0.028, 0.024)),
            origin=Origin(xyz=(x, -0.032, 0.340)),
            material=white_plastic,
            name=f"hinge_boss_{i}",
        )
        frame.visual(
            Cylinder(radius=0.012, length=0.050),
            origin=Origin(xyz=(x, -0.042, 0.348), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=white_plastic,
            name=f"fixed_hinge_pin_{i}",
        )

    # ---- Rear-face fixed hinge knuckles ----
    for i, x in enumerate((-0.165, 0.165)):
        frame.visual(
            Box((0.055, 0.028, 0.024)),
            origin=Origin(xyz=(x, 0.032, 0.340)),
            material=white_plastic,
            name=f"rear_hinge_boss_{i}",
        )
        frame.visual(
            Cylinder(radius=0.012, length=0.050),
            origin=Origin(xyz=(x, 0.042, 0.348), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=white_plastic,
            name=f"rear_fixed_hinge_pin_{i}",
        )

    # ------------------------------------------------------------------
    # Two flaps in series along the tunnel depth (Y axis)
    # index 0 = front (outer face), index 1 = rear (inner face)
    # ------------------------------------------------------------------
    flap_configs = [
        {"index": 0, "hinge_y": -0.042, "panel_y_bias": 0.006, "material": smoke_front},
        {"index": 1, "hinge_y": 0.042, "panel_y_bias": -0.006, "material": smoke_rear},
    ]
    for cfg in flap_configs:
        _add_flap(
            model,
            frame,
            index=cfg["index"],
            hinge_y=cfg["hinge_y"],
            panel_y_bias=cfg["panel_y_bias"],
            flap_material=cfg["material"],
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("panel_frame")

    # --- Both flaps must exist and have measurable panels ---
    flaps = []
    hinges = []
    for i in range(2):
        flap = object_model.get_part(f"flap_{i}")
        hinge = object_model.get_articulation(f"frame_to_flap_{i}")
        flaps.append(flap)
        hinges.append(hinge)

        aabb = ctx.part_element_world_aabb(flap, elem=f"flap_panel_{i}")
        if aabb is None:
            ctx.fail(f"flap_{i} panel has measurable bounds", f"No AABB for flap_panel_{i}.")
        else:
            lower, upper = aabb
            ctx.check(
                f"flap_panel_{i} fits inside pet-door opening",
                lower[0] > -0.150 and upper[0] < 0.150 and lower[2] > 0.020 and upper[2] < 0.342,
                details=f"flap_panel_{i}_aabb=({lower}, {upper})",
            )

    # --- Front flap (index 0) sits just behind front trim ---
    ctx.expect_gap(
        frame,
        flaps[0],
        axis="y",
        min_gap=0.001,
        max_gap=0.015,
        positive_elem="front_trim",
        negative_elem="flap_panel_0",
        name="front flap sits just behind front trim",
    )

    # --- Rear flap (index 1) is mounted on the rear side of the tunnel ---
    # The rear hinge sits at +Y; verify the flap panel is in the rear half.
    rear_panel_aabb = ctx.part_element_world_aabb(flaps[1], elem="flap_panel_1")
    if rear_panel_aabb is not None:
        rear_panel_center_y = (rear_panel_aabb[0][1] + rear_panel_aabb[1][1]) * 0.5
        ctx.check(
            "rear flap panel is on the rear (+Y) side of the tunnel",
            rear_panel_center_y > 0.010,
            details=f"rear_panel_center_y={rear_panel_center_y}",
        )

    # --- The two flaps must be separated along Y (series arrangement) ---
    front_center = None
    rear_center = None
    for i, flap in enumerate(flaps):
        aabb = ctx.part_element_world_aabb(flap, elem=f"flap_panel_{i}")
        if aabb is not None:
            cy = (aabb[0][1] + aabb[1][1]) * 0.5
            if i == 0:
                front_center = cy
            else:
                rear_center = cy
    ctx.check(
        "two flaps are separated along tunnel depth (Y)",
        front_center is not None
        and rear_center is not None
        and abs(rear_center - front_center) > 0.020,
        details=f"front_center_y={front_center}, rear_center_y={rear_center}",
    )

    # --- Both hinges open the flaps outward/upward with positive angle ---
    for i, (flap, hinge) in enumerate(zip(flaps, hinges)):
        rest_aabb = ctx.part_element_world_aabb(flap, elem=f"magnet_keeper_{i}")
        with ctx.pose({hinge: 0.75}):
            open_aabb = ctx.part_element_world_aabb(flap, elem=f"magnet_keeper_{i}")

        rest_cz = None if rest_aabb is None else (rest_aabb[0][2] + rest_aabb[1][2]) * 0.5
        open_cz = None if open_aabb is None else (open_aabb[0][2] + open_aabb[1][2]) * 0.5
        ctx.check(
            f"frame_to_flap_{i} positive swing raises flap_{i}",
            rest_cz is not None
            and open_cz is not None
            and open_cz > rest_cz + 0.030,
            details=f"rest_keeper_z={rest_cz}, open_keeper_z={open_cz}",
        )

    return ctx.report()


object_model = build_object_model()
