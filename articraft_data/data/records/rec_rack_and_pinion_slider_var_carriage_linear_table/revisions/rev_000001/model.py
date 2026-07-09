from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _trapezoid_prism(
    inner_width: float,
    outer_width: float,
    height: float,
    depth: float,
    *,
    centered_z: bool = False,
) -> MeshGeometry:
    """Trapezoidal tooth prism.

    X is tooth thickness along the pitch/tangent direction, Y is face width,
    and Z is tooth height/radial depth.  ``inner_width`` is the root width.
    """

    z0, z1 = (-height / 2.0, height / 2.0) if centered_z else (0.0, height)
    y0, y1 = -depth / 2.0, depth / 2.0
    xi, xo = inner_width / 2.0, outer_width / 2.0

    geom = MeshGeometry()
    verts = [
        (-xi, y0, z0),
        (xi, y0, z0),
        (xo, y0, z1),
        (-xo, y0, z1),
        (-xi, y1, z0),
        (xi, y1, z0),
        (xo, y1, z1),
        (-xo, y1, z1),
    ]
    for x, y, z in verts:
        geom.add_vertex(x, y, z)
    for tri in (
        (0, 1, 2),
        (0, 2, 3),
        (4, 7, 6),
        (4, 6, 5),
        (0, 4, 5),
        (0, 5, 1),
        (3, 2, 6),
        (3, 6, 7),
        (0, 3, 7),
        (0, 7, 4),
        (1, 5, 6),
        (1, 6, 2),
    ):
        geom.add_face(*tri)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="rack_and_pinion_slider",
        meta={"category": "Robotics / Rack-and-pinion slider"},
    )

    brushed = model.material("brushed_aluminum", rgba=(0.72, 0.75, 0.76, 1.0))
    dark = model.material("dark_bearing_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    rack_mat = model.material("satin_steel_rack", rgba=(0.62, 0.64, 0.63, 1.0))
    pinion_mat = model.material("warm_machined_gear", rgba=(0.74, 0.68, 0.58, 1.0))
    blue = model.material("blue_hub_cap", rgba=(0.20, 0.36, 0.55, 1.0))

    # Shared rack-and-pinion tooth geometry.  The rack pitch matches the pinion
    # circular pitch at the pitch radius used by the mimic relationship.
    tooth_pitch = 0.019
    pinion_teeth = 22
    pitch_radius = tooth_pitch * pinion_teeth / (2.0 * math.pi)
    gear_root_radius = pitch_radius - 0.006
    gear_tooth_depth = 0.014
    gear_width = 0.034
    rack_width = 0.060
    rack_bar_height = 0.026
    rack_bar_top_z = 0.085
    rack_tooth_height = 0.017
    rack_tooth_tip_z = rack_bar_top_z + rack_tooth_height
    pinion_center_z = rack_tooth_tip_z + gear_root_radius + 0.001

    rack_tooth_mesh = mesh_from_geometry(
        _trapezoid_prism(0.011, 0.006, rack_tooth_height, rack_width),
        "rack_tooth_profile",
    )
    gear_tooth_mesh = mesh_from_geometry(
        _trapezoid_prism(0.009, 0.006, gear_tooth_depth, gear_width, centered_z=True),
        "pinion_tooth_profile",
    )
    bearing_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.018, tube=0.0048, radial_segments=20, tubular_segments=36),
        "bearing_collar_ring",
    )

    frame = model.part("guide_frame")
    frame.visual(
        Box((0.86, 0.24, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=brushed,
        name="base_plate",
    )
    frame.visual(
        Box((0.74, 0.032, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=dark,
        name="guide_rail",
    )
    for idx, x in enumerate((-0.405, 0.405)):
        frame.visual(
            Box((0.028, 0.12, 0.040)),
            origin=Origin(xyz=(x, 0.0, 0.038)),
            material=brushed,
            name=f"rail_stop_{idx}",
        )
    for idx, (x, y) in enumerate(
        ((-0.36, -0.09), (-0.36, 0.09), (0.36, -0.09), (0.36, 0.09))
    ):
        frame.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(x, y, 0.021)),
            material=dark,
            name=f"mount_bolt_{idx}",
        )

    # Two bearing collars held by web brackets carry the pinion axis.  The
    # rotating shaft is smaller than the torus inner diameter, leaving a real
    # clearance hole instead of an overlap.
    # Bearing supports moved outward to clear the broad stage table
    for idx, y in enumerate((-0.068, 0.068)):
        web_height = pinion_center_z - 0.038
        frame.visual(
            Box((0.050, 0.014, web_height)),
            origin=Origin(xyz=(0.0, y, 0.018 + web_height / 2.0)),
            material=brushed,
            name=f"bearing_web_{idx}",
        )
        frame.visual(
            Box((0.105, 0.014, 0.014)),
            origin=Origin(xyz=(0.0, y, pinion_center_z - 0.026)),
            material=brushed,
            name=f"bearing_saddle_{idx}",
        )
        frame.visual(
            bearing_mesh,
            origin=Origin(xyz=(0.0, y, pinion_center_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark,
            name=f"bearing_collar_{idx}",
        )

    rack = model.part("rack_carriage")
    # Broad flat T-slot stage table replacing the slim carriage bridge.
    # Sits between the carriage shoes and the rack_bar as a rigid platform.
    stage_table_thickness = 0.012
    stage_table_top_z = rack_bar_top_z - rack_bar_height  # flush with rack_bar bottom = 0.059
    stage_table_center_z = stage_table_top_z - stage_table_thickness / 2.0
    rack.visual(
        Box((0.340, 0.122, stage_table_thickness)),
        origin=Origin(xyz=(0.0, 0.0, stage_table_center_z)),
        material=brushed,
        name="stage_table",
    )
    # T-slot ribs on the exposed table surface, outside the rack bar footprint
    for idx, y in enumerate((-0.050, -0.038, 0.038, 0.050)):
        rack.visual(
            Box((0.300, 0.006, 0.003)),
            origin=Origin(xyz=(0.0, y, stage_table_top_z + 0.0015)),
            material=dark,
            name=f"table_slot_rib_{idx}",
        )
    for idx, y in enumerate((-0.034, 0.034)):
        rack.visual(
            Box((0.250, 0.018, 0.024)),
            origin=Origin(xyz=(0.0, y, 0.035)),
            material=brushed,
            name=f"carriage_shoe_{idx}",
        )
    rack.visual(
        Box((0.620, rack_width, rack_bar_height)),
        origin=Origin(xyz=(0.0, 0.0, rack_bar_top_z - rack_bar_height / 2.0)),
        material=rack_mat,
        name="rack_bar",
    )
    for idx, x in enumerate((-0.305, 0.305)):
        rack.visual(
            Box((0.016, rack_width + 0.010, rack_bar_height + 0.010)),
            origin=Origin(xyz=(x, 0.0, rack_bar_top_z - rack_bar_height / 2.0 + 0.001)),
            material=rack_mat,
            name=f"rack_end_cap_{idx}",
        )
    for idx, i in enumerate(range(-15, 15)):
        # Half-pitch phasing leaves a clear valley directly below the pinion.
        x = (i + 0.5) * tooth_pitch
        rack.visual(
            rack_tooth_mesh,
            origin=Origin(xyz=(x, 0.0, rack_bar_top_z - 0.0005)),
            material=rack_mat,
            name=f"rack_tooth_{idx:02d}",
        )

    pinion = model.part("pinion")
    pinion.visual(
        Cylinder(radius=gear_root_radius, length=gear_width),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pinion_mat,
        name="root_wheel",
    )
    for idx in range(pinion_teeth):
        theta = math.pi + idx * (2.0 * math.pi / pinion_teeth)
        radius = gear_root_radius + gear_tooth_depth / 2.0 - 0.0008
        pinion.visual(
            gear_tooth_mesh,
            origin=Origin(
                xyz=(radius * math.sin(theta), 0.0, radius * math.cos(theta)),
                rpy=(0.0, theta, 0.0),
            ),
            material=pinion_mat,
            name=f"pinion_tooth_{idx:02d}",
        )
    pinion.visual(
        Cylinder(radius=0.026, length=0.052),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="raised_hub",
    )
    pinion.visual(
        Cylinder(radius=0.0132, length=0.150),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="axle",
    )
    pinion.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.0, -0.075, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=blue,
        name="axle_cap_0",
    )
    pinion.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.0, 0.075, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=blue,
        name="axle_cap_1",
    )

    model.articulation(
        "pinion_spin",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=pinion,
        origin=Origin(xyz=(0.0, 0.0, pinion_center_z)),
        # Positive rotation advances the rack/carriage along +X at the bottom
        # mesh point.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=4.0, lower=-1.0, upper=1.0),
    )
    model.articulation(
        "rack_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=rack,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=70.0,
            velocity=0.25,
            lower=-pitch_radius,
            upper=pitch_radius,
        ),
        meta={"pitch_radius_m": pitch_radius, "tooth_pitch_m": tooth_pitch},
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("guide_frame")
    rack = object_model.get_part("rack_carriage")
    pinion = object_model.get_part("pinion")
    spin = object_model.get_articulation("pinion_spin")
    slide = object_model.get_articulation("rack_slide")

    for idx in (0, 1):
        ctx.allow_overlap(
            frame,
            pinion,
            elem_a=f"bearing_collar_{idx}",
            elem_b="axle",
            reason=(
                "The rotating axle is intentionally captured in the bearing collar "
                "as a close journal fit."
            ),
        )
        ctx.expect_within(
            pinion,
            frame,
            axes="xz",
            inner_elem="axle",
            outer_elem=f"bearing_collar_{idx}",
            margin=0.001,
            name=f"axle centered in bearing collar {idx}",
        )
        ctx.expect_overlap(
            pinion,
            frame,
            axes="y",
            elem_a="axle",
            elem_b=f"bearing_collar_{idx}",
            min_overlap=0.006,
            name=f"axle retained through bearing collar {idx}",
        )

    ctx.check(
        "rack and pinion joints are present",
        spin.articulation_type == ArticulationType.REVOLUTE
        and slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"spin={spin.articulation_type}, slide={slide.articulation_type}",
    )
    ctx.check(
        "rack pitch metadata matches pinion size",
        0.060 < slide.meta.get("pitch_radius_m", 0.0) < 0.075
        and 0.018 < slide.meta.get("tooth_pitch_m", 0.0) < 0.020,
        details=f"meta={slide.meta}",
    )
    ctx.expect_overlap(
        pinion,
        rack,
        axes="x",
        min_overlap=0.035,
        name="pinion centered over rack teeth",
    )
    ctx.expect_overlap(
        pinion,
        rack,
        axes="z",
        min_overlap=0.006,
        name="tooth envelopes vertically intermesh",
    )
    ctx.expect_within(
        rack,
        frame,
        axes="y",
        margin=0.002,
        name="rack carriage stays within guide width",
    )
    ctx.expect_gap(
        rack,
        frame,
        axis="z",
        positive_elem="rack_bar",
        negative_elem="guide_rail",
        min_gap=0.006,
        max_gap=0.020,
        name="rack is carried above guide rail",
    )

    # --- Stage table variant checks ---
    # The stage_table is a broad flat platform replacing the slim carriage_bridge.
    # Verify it rides above the guide rail with a small clearance.
    ctx.expect_gap(
        rack,
        frame,
        axis="z",
        positive_elem="stage_table",
        negative_elem="guide_rail",
        min_gap=0.0,
        max_gap=0.015,
        name="stage_table rides above guide rail without penetration",
    )
    # Verify the stage_table is broader than the rack_bar across Y (T-slot platform).
    table_box = ctx.part_element_world_aabb(rack, elem="stage_table")
    bar_box = ctx.part_element_world_aabb(rack, elem="rack_bar")
    if table_box and bar_box:
        table_dy = table_box[1][1] - table_box[0][1]
        bar_dy = bar_box[1][1] - bar_box[0][1]
        ctx.check(
            "stage_table is broader than rack_bar across Y (T-slot platform)",
            table_dy > bar_dy + 0.04,
            details=f"table_dy={table_dy:.4f}, bar_dy={bar_dy:.4f}",
        )
    # Verify T-slot ribs sit on the stage table surface (Z above table top).
    for rib_idx in range(4):
        rib_name = f"table_slot_rib_{rib_idx}"
        ctx.expect_gap(
            rack,
            rack,
            axis="z",
            positive_elem=rib_name,
            negative_elem="stage_table",
            min_gap=-0.001,
            max_gap=0.005,
            name=f"table_slot_rib_{rib_idx} seated on stage_table surface",
        )

    rest_pos = ctx.part_world_position(rack)
    with ctx.pose({slide: 0.055, spin: 0.8}):
        moved_pos = ctx.part_world_position(rack)
        ctx.expect_within(
            rack,
            frame,
            axes="y",
            margin=0.002,
            name="sliding rack remains laterally guided",
        )
        ctx.expect_overlap(
            rack,
            frame,
            axes="x",
            min_overlap=0.45,
            name="extended rack remains supported on base",
        )
    ctx.check(
        "positive rack prismatic motion travels in +X",
        rest_pos is not None and moved_pos is not None and moved_pos[0] > rest_pos[0] + 0.045,
        details=f"rest={rest_pos}, moved={moved_pos}",
    )

    return ctx.report()


object_model = build_object_model()
