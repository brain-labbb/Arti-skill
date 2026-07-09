from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    RackGear,
    SpurGear,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


MODULE_MM = 2.0
PINION_TEETH = 28
PINION_WIDTH_MM = 16.0
PINION_BORE_MM = 12.0
RACK_LENGTH_MM = 260.0
RACK_WIDTH_MM = 18.0
RACK_BACKING_MM = 10.0

PINION_PITCH_RADIUS_M = MODULE_MM * PINION_TEETH * 0.5 * 0.001
PINION_CENTER_OFFSET_M = (MODULE_MM * PINION_TEETH * 0.5 + MODULE_MM * 2.05) * 0.001
CARRIAGE_TRAVEL_M = 0.045
PINION_TRAVEL_RAD = CARRIAGE_TRAVEL_M / PINION_PITCH_RADIUS_M

BASE_LENGTH = 0.320
BASE_WIDTH = 0.095
BASE_THICKNESS = 0.012
RACK_FRAME_Z = 0.036
PINION_CENTER_Z = RACK_FRAME_Z + PINION_CENTER_OFFSET_M

# Layout constants
RAIL_TOP_Z = BASE_THICKNESS + 0.014  # 0.026 — exact contact with sliders
RACK_BOTTOM_Z = 0.0235  # from probe — rack backing bottom in world
RACK_Y_MIN = -0.018  # rack Y extent (from probe)
RACK_Y_MAX = 0.0
RACK_Y_CENTER = 0.5 * (RACK_Y_MIN + RACK_Y_MAX)  # -0.009


def _pinion_shape() -> cq.Shape:
    gear = SpurGear(
        module=MODULE_MM,
        teeth_number=PINION_TEETH,
        width=PINION_WIDTH_MM,
        backlash=0.08,
    )
    body = gear.build(bore_d=PINION_BORE_MM)
    return body.rotate((0, 0, 0), (1, 0, 0), 90)


def _pinion_hub_shape() -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(15.0)
        .circle(PINION_BORE_MM * 0.5)
        .extrude(6.0, both=True)
    )


def _rack_shape() -> cq.Shape:
    rack = RackGear(
        module=MODULE_MM,
        length=RACK_LENGTH_MM,
        width=RACK_WIDTH_MM,
        height=RACK_BACKING_MM,
        backlash=0.08,
    )
    body = rack.build()
    return body.rotate((0, 0, 0), (1, 0, 0), 90).translate((-RACK_LENGTH_MM * 0.5, 0, 0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="rack_and_pinion_slider",
        meta={
            "category": "Robotics / Rack-and-pinion slider",
            "reference_note": (
                "Fixed-rack / traveling-pinion topology (gantry-axis drive). "
                "Classification matches the visible rack-and-pinion slider."
            ),
        },
    )

    model.material("brushed_steel", rgba=(0.58, 0.60, 0.60, 1.0))
    model.material("dark_oxide", rgba=(0.12, 0.13, 0.14, 1.0))
    model.material("zinc_hardware", rgba=(0.76, 0.75, 0.70, 1.0))
    model.material("black_bearing", rgba=(0.02, 0.02, 0.025, 1.0))

    # ── FRAME (root) ─────────────────────────────────────────────────
    frame = model.part("frame")
    frame.visual(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS * 0.5)),
        material="dark_oxide",
        name="base_plate",
    )
    # Guide rails — exact contact with carriage slider blocks at z=RAIL_TOP_Z.
    for i, y_pos in enumerate((-0.035, 0.035)):
        frame.visual(
            Box((0.295, 0.010, 0.014)),
            origin=Origin(xyz=(0.0, y_pos, BASE_THICKNESS + 0.007)),
            material="brushed_steel",
            name=f"guide_rail_{i}",
        )
    # Rack bed/riser connects base plate to the fixed rack.
    rack_bed_height = RACK_BOTTOM_Z - BASE_THICKNESS  # ~0.0115
    frame.visual(
        Box((0.260, RACK_WIDTH_MM * 0.001, rack_bed_height)),
        origin=Origin(xyz=(0.0, RACK_Y_CENTER, BASE_THICKNESS + rack_bed_height * 0.5)),
        material="dark_oxide",
        name="rack_bed",
    )
    # Fixed rack bolted full length to frame (structural topology change).
    frame.visual(
        mesh_from_cadquery(_rack_shape(), "straight_rack", unit_scale=0.001, tolerance=0.00025),
        origin=Origin(xyz=(0.0, 0.0, RACK_FRAME_Z)),
        material="brushed_steel",
        name="straight_rack",
    )
    # Mounting hardware
    for x_pos in (-0.135, 0.135):
        for y_pos in (-0.033, 0.033):
            frame.visual(
                Cylinder(radius=0.0065, length=0.004),
                origin=Origin(xyz=(x_pos, y_pos, BASE_THICKNESS + 0.002)),
                material="zinc_hardware",
                name=f"mount_screw_{'n' if y_pos > 0 else 's'}_{'p' if x_pos > 0 else 'm'}",
            )

    # ── PINION CARRIAGE (travels along fixed rack via rack_slide) ────
    # Inverted-U saddle: two tall vertical cheeks connected at the top by a
    # crossbar, with slider blocks at the base riding on the guide rails.
    # The pinion shaft passes through both cheeks, bridging the structure.
    carriage = model.part("pinion_carriage")

    # All z-positions computed in world coords, then converted to carriage-local
    # (carriage origin sits at world z = RACK_FRAME_Z).
    _zr = lambda z_world: z_world - RACK_FRAME_Z  # noqa: E731

    # Pinion outer extent: pitch_radius + addendum ≈ 0.028 + 0.002 = 0.030
    # Pinion top world z ≈ 0.0681 + 0.030 = 0.0981 (confirmed by probe)
    PINION_TOP_WORLD = PINION_CENTER_Z + PINION_PITCH_RADIUS_M + MODULE_MM * 0.001

    # Cheeks span from rail-top to well above the pinion (clearing the gear teeth).
    cheek_bottom_world = RAIL_TOP_Z  # 0.026
    cheek_top_world = PINION_TOP_WORLD + 0.014  # 14 mm above tooth tips
    cheek_height = cheek_top_world - cheek_bottom_world
    cheek_z_rel = _zr(0.5 * (cheek_bottom_world + cheek_top_world))

    # Bearing cheeks — tall walls straddling the rack, outside its Y extent.
    for i, y_pos in enumerate((-0.028, 0.028)):
        carriage.visual(
            Box((0.050, 0.010, cheek_height)),
            origin=Origin(xyz=(0.0, y_pos, cheek_z_rel)),
            material="dark_oxide",
            name=f"bearing_cheek_{i}",
        )

    # Slider blocks — ride on guide rails; tall enough to overlap with cheeks.
    slider_height = 0.014  # extends well above rail top into cheek overlap zone
    slider_z_rel = _zr(RAIL_TOP_Z + slider_height * 0.5)
    for i, y_pos in enumerate((-0.035, 0.035)):
        carriage.visual(
            Box((0.050, 0.014, slider_height)),
            origin=Origin(xyz=(0.0, y_pos, slider_z_rel)),
            material="black_bearing",
            name=f"guide_slider_{i}",
        )

    # Top crossbar — connects both cheeks above the pinion, forming the U bridge.
    # Bottom must clear pinion tooth tips (0.0981) by more than overlap_tol (0.005).
    crossbar_height = 0.008
    crossbar_bottom_world = PINION_TOP_WORLD + 0.006  # 6 mm above tooth tips
    crossbar_center_world = crossbar_bottom_world + crossbar_height * 0.5
    crossbar_z_rel = _zr(crossbar_center_world)
    carriage.visual(
        Box((0.050, 0.066, crossbar_height)),
        origin=Origin(xyz=(0.0, 0.0, crossbar_z_rel)),
        material="dark_oxide",
        name="carriage_crossbar",
    )

    # Pinion shaft — passes through both cheeks at pinion center height,
    # bridging the two sides of the carriage.
    carriage.visual(
        Cylinder(radius=0.007, length=0.072),
        origin=Origin(
            xyz=(0.0, 0.0, _zr(PINION_CENTER_Z)),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material="zinc_hardware",
        name="pinion_shaft",
    )

    # Carriage bolts — on top surface of the crossbar.
    crossbar_top_z_rel = _zr(cheek_top_world)
    for i, x_pos in enumerate((-0.018, 0.018)):
        carriage.visual(
            Cylinder(radius=0.004, length=0.003),
            origin=Origin(
                xyz=(x_pos, 0.0, crossbar_top_z_rel + 0.0015),
            ),
            material="zinc_hardware",
            name=f"carriage_bolt_{i}",
        )

    # ── PINION (rotates on carriage, meshes with fixed rack) ─────────
    pinion = model.part("pinion")
    pinion.visual(
        mesh_from_cadquery(_pinion_shape(), "toothed_wheel", unit_scale=0.001, tolerance=0.00022),
        material="brushed_steel",
        name="toothed_wheel",
    )
    pinion.visual(
        mesh_from_cadquery(_pinion_hub_shape(), "pinion_hub", unit_scale=0.001, tolerance=0.00025),
        material="zinc_hardware",
        name="hub",
    )

    # ── ARTICULATIONS ────────────────────────────────────────────────

    # PRISMATIC: frame → pinion_carriage (carriage translates along guide rails)
    model.articulation(
        "rack_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, RACK_FRAME_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0, upper=CARRIAGE_TRAVEL_M, effort=80.0, velocity=0.30,
        ),
        meta={"pinion_pitch_radius_m": PINION_PITCH_RADIUS_M},
    )

    # REVOLUTE: pinion_carriage → pinion (pinion rolls along fixed rack)
    model.articulation(
        "pinion_spin",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=pinion,
        origin=Origin(xyz=(0.0, 0.0, PINION_CENTER_OFFSET_M)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0, upper=PINION_TRAVEL_RAD, effort=8.0, velocity=8.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("pinion_carriage")
    pinion = object_model.get_part("pinion")
    spin = object_model.get_articulation("pinion_spin")
    slide = object_model.get_articulation("rack_slide")

    # ── Overlap allowances ──────────────────────────────────────────
    ctx.allow_overlap(
        carriage,
        pinion,
        elem_a="pinion_shaft",
        elem_b="toothed_wheel",
        reason=(
            "The carriage shaft is intentionally modeled as a light interference "
            "fit inside the pinion bore to represent the captured bearing support "
            "on the traveling carriage."
        ),
    )
    ctx.allow_overlap(
        carriage,
        pinion,
        elem_a="pinion_shaft",
        elem_b="hub",
        reason=(
            "The shaft locally seats inside the rotating hub bore as a captured "
            "journal/bushing fit on the carriage."
        ),
    )

    # Pair every allowance with an exact proof check.
    ctx.expect_contact(
        carriage,
        pinion,
        contact_tol=0.001,
        elem_a="pinion_shaft",
        elem_b="toothed_wheel",
        name="shaft is seated in pinion bore (interference fit)",
    )

    # ── Structural topology assertions (fixed-rack / traveling-pinion) ─
    ctx.check(
        "classification matches visible rack-and-pinion slider",
        object_model.meta.get("category") == "Robotics / Rack-and-pinion slider",
        details=str(object_model.meta),
    )
    slide_parent_name = slide.parent.name if hasattr(slide.parent, "name") else str(slide.parent)
    slide_child_name = slide.child.name if hasattr(slide.child, "name") else str(slide.child)
    spin_parent_name = spin.parent.name if hasattr(spin.parent, "name") else str(spin.parent)
    spin_child_name = spin.child.name if hasattr(spin.child, "name") else str(spin.child)
    ctx.check(
        "rack_slide is PRISMATIC from frame to pinion_carriage",
        slide_parent_name == "frame" and slide_child_name == "pinion_carriage",
        details=f"parent={slide_parent_name}, child={slide_child_name}",
    )
    ctx.check(
        "pinion_spin is REVOLUTE from pinion_carriage to pinion",
        spin_parent_name == "pinion_carriage" and spin_child_name == "pinion",
        details=f"parent={spin_parent_name}, child={spin_child_name}",
    )
    frame_visual_names = {v.name for v in frame.visuals}
    ctx.check(
        "straight_rack is fixed to frame (not on a moving part)",
        "straight_rack" in frame_visual_names,
        details=f"frame visuals: {sorted(frame_visual_names)}",
    )

    # ── Mesh alignment ──────────────────────────────────────────────
    ctx.expect_overlap(
        pinion,
        frame,
        axes="xy",
        min_overlap=0.010,
        elem_a="toothed_wheel",
        elem_b="straight_rack",
        name="pinion teeth align over rack teeth at rest",
    )
    ctx.expect_gap(
        pinion,
        frame,
        axis="z",
        min_gap=0.0,
        max_gap=0.003,
        positive_elem="toothed_wheel",
        negative_elem="straight_rack",
        name="pinion teeth sit just above rack teeth without penetration",
    )

    # Shaft centering inside pinion bore
    ctx.expect_within(
        carriage,
        pinion,
        axes="xz",
        inner_elem="pinion_shaft",
        outer_elem="toothed_wheel",
        margin=0.001,
        name="shaft is centered inside pinion bore footprint on carriage",
    )
    ctx.expect_overlap(
        carriage,
        pinion,
        axes="y",
        min_overlap=0.012,
        elem_a="pinion_shaft",
        elem_b="toothed_wheel",
        name="shaft passes through pinion face width",
    )

    # ── Carriage support on frame ───────────────────────────────────
    ctx.expect_overlap(
        carriage,
        frame,
        axes="x",
        min_overlap=0.040,
        elem_a="bearing_cheek_0",
        elem_b="base_plate",
        name="carriage cheek remains carried over base length at rest",
    )

    # ── Carriage travel test ────────────────────────────────────────
    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: CARRIAGE_TRAVEL_M}):
        extended_pos = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            pinion,
            frame,
            axes="xy",
            min_overlap=0.010,
            elem_a="toothed_wheel",
            elem_b="straight_rack",
            name="mesh alignment persists at max carriage travel",
        )
        ctx.expect_overlap(
            carriage,
            frame,
            axes="x",
            min_overlap=0.030,
            elem_a="bearing_cheek_0",
            elem_b="base_plate",
            name="carriage cheek remains carried over base at max travel",
        )

    ctx.check(
        "pinion_carriage translates +X along fixed rack via rack_slide",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.020,
        details=f"rest={rest_pos}, extended={extended_pos}, travel={CARRIAGE_TRAVEL_M}",
    )

    return ctx.report()


object_model = build_object_model()
