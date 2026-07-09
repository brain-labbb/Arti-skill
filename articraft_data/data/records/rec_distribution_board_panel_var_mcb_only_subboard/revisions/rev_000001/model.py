from __future__ import annotations

import math

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
    WirePath,
    mesh_from_geometry,
    tube_from_spline_points,
)


DOOR_OPEN_ANGLE = 1.08


def _box(part, size, xyz, material, name, rpy=(0.0, 0.0, 0.0)):
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _cylinder(part, radius, length, xyz, material, name, rpy=(0.0, 0.0, 0.0)):
    part.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _door_pose_xy(x: float, y: float, angle: float = DOOR_OPEN_ANGLE) -> tuple[float, float]:
    c = math.cos(angle)
    s = math.sin(angle)
    return (x * c - y * s, x * s + y * c)


def _door_box(part, size, closed_xyz, material, name):
    x, y = _door_pose_xy(closed_xyz[0], closed_xyz[1])
    _box(
        part,
        size,
        (x, y, closed_xyz[2]),
        material,
        name,
        rpy=(0.0, 0.0, DOOR_OPEN_ANGLE),
    )


def _wire_mesh(points, radius=0.003):
    return tube_from_spline_points(
        points,
        radius=radius,
        samples_per_segment=10,
        radial_segments=14,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="distribution_board_panel",
        meta={
            "domain": "Electrical_Wiring",
            "small_class": "Distribution board panel",
            "description": "MCB-only sub-board with hinged sheet-metal enclosure, breaker field, neutral and earth bars, labels, glands, wiring channels, and lock hardware. Fed from upstream main board.",
        },
    )

    painted = model.material("painted_gray_sheet_metal", rgba=(0.42, 0.46, 0.47, 1.0))
    dark_paint = model.material("dark_recess_gray", rgba=(0.25, 0.28, 0.29, 1.0))
    black_plastic = model.material("molded_black_plastic", rgba=(0.015, 0.014, 0.013, 1.0))
    rubber = model.material("black_rubber_cable", rgba=(0.02, 0.02, 0.018, 1.0))
    dark_handle = model.material("charcoal_toggle_plastic", rgba=(0.09, 0.09, 0.085, 1.0))
    brass = model.material("brass_terminal_bar", rgba=(0.78, 0.56, 0.25, 1.0))
    copper = model.material("copper_bus_bar", rgba=(0.86, 0.38, 0.18, 1.0))
    galvanized = model.material("galvanized_screw_metal", rgba=(0.74, 0.76, 0.73, 1.0))
    label_white = model.material("white_printed_label", rgba=(0.95, 0.94, 0.90, 1.0))
    label_blue = model.material("blue_circuit_label", rgba=(0.03, 0.16, 0.70, 1.0))
    label_red = model.material("red_warning_stripe", rgba=(0.82, 0.05, 0.04, 1.0))
    label_orange = model.material("orange_warning_stripe", rgba=(1.0, 0.48, 0.06, 1.0))
    label_yellow = model.material("yellow_warning_icon", rgba=(0.96, 0.82, 0.05, 1.0))
    transparent = model.material("smoky_transparent_window", rgba=(0.55, 0.65, 0.70, 0.38))
    wire_red = model.material("red_insulated_wire", rgba=(0.85, 0.02, 0.01, 1.0))
    wire_blue = model.material("blue_insulated_wire", rgba=(0.03, 0.12, 0.85, 1.0))
    wire_green = model.material("green_earth_wire", rgba=(0.04, 0.55, 0.16, 1.0))

    enclosure = model.part("enclosure", meta={"role": "root sheet-metal cabinet"})

    # Open-backed sheet-metal box with front flange and recessed internal mounting pan.
    _box(enclosure, (0.620, 0.010, 0.950), (0.0, 0.055, 0.475), painted, "back_pan")
    _box(enclosure, (0.026, 0.120, 0.950), (-0.297, 0.000, 0.475), painted, "side_wall_0")
    _box(enclosure, (0.026, 0.120, 0.950), (0.297, 0.000, 0.475), painted, "side_wall_1")
    _box(enclosure, (0.620, 0.120, 0.026), (0.000, 0.000, 0.937), painted, "top_wall")
    _box(enclosure, (0.620, 0.120, 0.026), (0.000, 0.000, 0.013), painted, "bottom_wall")

    _box(enclosure, (0.055, 0.008, 0.800), (-0.220, -0.071, 0.500), painted, "front_left_rail")
    _box(enclosure, (0.055, 0.008, 0.800), (0.220, -0.071, 0.500), painted, "front_right_rail")
    _box(enclosure, (0.440, 0.008, 0.040), (0.000, -0.071, 0.880), painted, "front_top_rail")
    _box(enclosure, (0.440, 0.008, 0.040), (0.000, -0.071, 0.120), painted, "front_bottom_rail")
    _box(enclosure, (0.385, 0.006, 0.720), (0.000, -0.057, 0.500), dark_paint, "recessed_deadfront")
    _box(enclosure, (0.370, 0.010, 0.035), (0.000, -0.069, 0.850), painted, "upper_panel_lip")
    _box(enclosure, (0.370, 0.010, 0.035), (0.000, -0.069, 0.150), painted, "lower_panel_lip")

    # Slotted screw holes on the front rails and visible screw heads on the deadfront.
    for side_x, prefix in [(-0.246, "left"), (0.246, "right")]:
        for i, z in enumerate([0.170, 0.315, 0.500, 0.685, 0.830]):
            _box(enclosure, (0.010, 0.003, 0.035), (side_x, -0.076, z), dark_paint, f"{prefix}_slot_{i}")
    for i, (x, z) in enumerate([(-0.180, 0.175), (0.180, 0.175), (-0.165, 0.830), (0.165, 0.830), (-0.190, 0.780), (0.190, 0.780)]):
        _cylinder(enclosure, 0.0075, 0.004, (x, -0.074, z), galvanized, f"panel_screw_{i}", rpy=(math.pi / 2, 0.0, 0.0))

    # Two breaker banks with many individual molded breaker bodies and fixed handle details.
    for bank, x0 in enumerate([-0.085, 0.085]):
        _box(enclosure, (0.078, 0.018, 0.520), (x0, -0.076, 0.515), black_plastic, f"breaker_well_{bank}")
        _box(enclosure, (0.006, 0.022, 0.540), (x0 - 0.043, -0.078, 0.515), black_plastic, f"breaker_side_rail_{bank}_0")
        _box(enclosure, (0.006, 0.022, 0.540), (x0 + 0.043, -0.078, 0.515), black_plastic, f"breaker_side_rail_{bank}_1")
        for row in range(12):
            z = 0.285 + row * 0.040
            _box(enclosure, (0.071, 0.015, 0.030), (x0, -0.090, z), black_plastic, f"breaker_{bank}_{row}")
            _box(enclosure, (0.025, 0.006, 0.022), (x0, -0.100, z + 0.002), dark_handle, f"fixed_handle_{bank}_{row}")
            _box(enclosure, (0.010, 0.004, 0.004), (x0 + 0.030, -0.099, z + 0.010), label_white, f"circuit_tick_{bank}_{row}")

    # Neutral/earth terminal bars with small terminal screws and standoffs.
    # This is an MCB-only sub-board fed from an upstream main board, so the
    # main breaker and incoming copper busbar are intentionally omitted.
    _box(enclosure, (0.275, 0.008, 0.014), (0.000, -0.081, 0.235), brass, "neutral_bar")
    _box(enclosure, (0.275, 0.008, 0.014), (0.000, -0.081, 0.205), brass, "earth_bar")
    # Incoming sub-feed terminal block where the upstream supply lands.
    _box(enclosure, (0.120, 0.016, 0.028), (0.000, -0.079, 0.810), black_plastic, "sub_feed_terminal_block")
    for i, x in enumerate([-0.040, 0.000, 0.040]):
        _cylinder(enclosure, 0.005, 0.008, (x, -0.088, 0.810), galvanized, f"sub_feed_screw_{i}", rpy=(math.pi / 2, 0.0, 0.0))
    for bar_z, stem in [(0.235, "neutral"), (0.205, "earth")]:
        for i, x in enumerate([-0.120, -0.080, -0.040, 0.000, 0.040, 0.080, 0.120]):
            _box(enclosure, (0.015, 0.010, 0.020), (x, -0.073, bar_z), dark_paint, f"{stem}_standoff_{i}")
            _cylinder(enclosure, 0.0045, 0.003, (x, -0.086, bar_z), galvanized, f"{stem}_screw_{i}", rpy=(math.pi / 2, 0.0, 0.0))

    # Color-coded circuit/warning labels below the breaker banks.
    _box(enclosure, (0.220, 0.003, 0.040), (-0.030, -0.060, 0.200), label_white, "legend_label")
    _box(enclosure, (0.180, 0.004, 0.012), (-0.010, -0.0625, 0.212), label_blue, "blue_directory_strip")
    _box(enclosure, (0.220, 0.003, 0.042), (-0.030, -0.062, 0.145), label_white, "danger_label")
    _box(enclosure, (0.220, 0.004, 0.012), (-0.030, -0.064, 0.164), label_orange, "orange_warning_bar")
    _box(enclosure, (0.220, 0.003, 0.032), (-0.030, -0.068, 0.125), label_white, "arcflash_label")
    _box(enclosure, (0.220, 0.004, 0.009), (-0.030, -0.0625, 0.134), label_red, "red_warning_bar")
    for i, x in enumerate([-0.125, -0.060, 0.005, 0.070]):
        _box(enclosure, (0.045, 0.004, 0.003), (x, -0.065, 0.138), dark_paint, f"label_print_{i}")
        _box(enclosure, (0.007, 0.004, 0.007), (x - 0.025, -0.065, 0.158), label_yellow, f"warning_icon_{i}")

    # Knockouts, cable glands, conduits, and molded wiring channels.
    for i, x in enumerate([-0.180, -0.060, 0.060, 0.180]):
        _cylinder(enclosure, 0.020, 0.006, (x, -0.020, 0.952), galvanized, f"top_knockout_{i}")
    for i, x in enumerate([-0.135, 0.000, 0.135]):
        _cylinder(enclosure, 0.018, 0.120, (x, 0.020, 1.005), galvanized, f"top_conduit_{i}")
        _cylinder(enclosure, 0.024, 0.018, (x, 0.020, 0.950), rubber, f"top_gland_{i}")
    for i, x in enumerate([-0.120, 0.120]):
        _cylinder(enclosure, 0.019, 0.055, (x, 0.015, -0.015), rubber, f"bottom_cable_{i}")
        _cylinder(enclosure, 0.025, 0.018, (x, 0.015, 0.025), galvanized, f"bottom_gland_{i}")
    _box(enclosure, (0.034, 0.020, 0.600), (-0.172, -0.068, 0.520), dark_paint, "left_wiring_channel")
    _box(enclosure, (0.034, 0.020, 0.600), (0.172, -0.068, 0.520), dark_paint, "right_wiring_channel")
    for i, z in enumerate([0.330, 0.430, 0.530, 0.630, 0.730]):
        _box(enclosure, (0.026, 0.007, 0.006), (-0.172, -0.080, z), painted, f"left_channel_tooth_{i}")
        _box(enclosure, (0.026, 0.007, 0.006), (0.172, -0.080, z), painted, f"right_channel_tooth_{i}")

    # A few routed insulated wires are true tubes with endpoints buried in glands/terminal bars.
    wire_specs = [
        ("sub_feed_live_wire", wire_red, [(-0.060, 0.020, 0.952), (-0.060, -0.060, 0.880), (-0.040, -0.088, 0.810)]),
        ("blue_neutral_wire", wire_blue, [(0.000, 0.020, 0.952), (0.140, -0.056, 0.680), (0.120, -0.088, 0.235)]),
        ("green_earth_wire", wire_green, [(0.135, 0.020, 0.952), (0.172, -0.065, 0.520), (0.105, -0.088, 0.205)]),
        ("bottom_load_wire", rubber, [(-0.120, 0.015, 0.020), (-0.160, -0.065, 0.180), (-0.085, -0.094, 0.300)]),
    ]
    for name, mat, points in wire_specs:
        enclosure.visual(mesh_from_geometry(_wire_mesh(points), name), material=mat, name=name)

    # Stationary hinge pin and welded hinge bridge blocks physically tie the door
    # knuckles to the grounded enclosure in the visible open pose.
    _cylinder(enclosure, 0.006, 0.650, (0.315, -0.096, 0.500), galvanized, "hinge_pin")
    _cylinder(enclosure, 0.011, 0.080, (0.315, -0.096, 0.500), galvanized, "center_hinge_collar")
    _box(enclosure, (0.012, 0.050, 0.360), (0.303, -0.075, 0.500), painted, "hinge_jamb_spine")
    for i, zc in enumerate([-0.265, 0.265]):
        _box(enclosure, (0.042, 0.026, 0.100), (0.304, -0.061, 0.500 + zc), painted, f"frame_hinge_bridge_{i}")

    # The right-side hinged front door is authored in its visible open pose.
    door = model.part("front_door", meta={"role": "hinged front cover"})
    door_width = 0.405
    door_height = 0.790
    _door_box(door, (door_width - 0.028, 0.018, door_height), (-door_width / 2.0 - 0.014, 0.0, 0.000), painted, "door_leaf")
    _door_box(door, (door_width - 0.035, 0.010, 0.018), (-door_width / 2.0, -0.015, 0.350), painted, "door_top_return")
    _door_box(door, (door_width - 0.035, 0.010, 0.018), (-door_width / 2.0, -0.015, -0.350), painted, "door_bottom_return")
    _door_box(door, (0.018, 0.010, door_height - 0.040), (-0.018, -0.015, 0.000), painted, "door_hinge_return")
    _door_box(door, (0.018, 0.010, door_height - 0.040), (-door_width + 0.018, -0.015, 0.000), painted, "door_latch_return")

    # Labels, inspection pouch, latch/lock and handle hardware on the moving door.
    _door_box(door, (0.110, 0.004, 0.045), (-0.145, -0.011, 0.205), label_white, "door_nameplate")
    _door_box(door, (0.080, 0.005, 0.014), (-0.145, -0.012, 0.193), dark_paint, "door_barcode")
    _door_box(door, (0.115, 0.004, 0.055), (-0.145, -0.011, 0.285), label_white, "door_schedule_label")
    for i, z in enumerate([0.272, 0.285, 0.298]):
        _door_box(door, (0.075, 0.005, 0.003), (-0.145, -0.014, z), dark_paint, f"schedule_line_{i}")
    _door_box(door, (0.105, 0.006, 0.120), (-0.205, -0.012, -0.030), transparent, "inspection_pouch")
    _door_box(door, (0.040, 0.020, 0.135), (-0.032, -0.019, -0.060), galvanized, "lock_plate")
    _door_box(door, (0.028, 0.030, 0.030), (-0.032, -0.039, 0.000), galvanized, "round_lock")
    _door_box(door, (0.026, 0.020, 0.125), (-0.025, -0.039, -0.120), galvanized, "swing_latch")
    _door_box(door, (0.052, 0.018, 0.032), (-0.050, -0.039, -0.200), transparent, "clear_tag_bag")
    _door_box(door, (0.022, 0.018, 0.075), (-0.030, -0.039, -0.178), transparent, "tag_bag_fold")
    for i, z in enumerate([-0.115, -0.050, 0.000]):
        _door_box(door, (0.014, 0.006, 0.014), (-0.032, -0.030, z), galvanized, f"lock_screw_{i}")

    # Moving hinge leaves and knuckles stay with the door leaf; the joint itself
    # supplies the retained hinge-pin connection to the cabinet.
    for i, zc in enumerate([-0.265, 0.265]):
        _door_box(door, (0.026, 0.016, 0.135), (-0.025, 0.002, zc), painted, f"hinge_leaf_{i}")
        _cylinder(door, 0.012, 0.120, (0.0, 0.0, zc), galvanized, f"hinge_barrel_{i}")
        _door_box(door, (0.010, 0.006, 0.040), (-0.028, -0.010, zc), galvanized, f"hinge_screw_{i}_0")
        _door_box(door, (0.010, 0.006, 0.040), (-0.028, -0.010, zc + 0.045), galvanized, f"hinge_screw_{i}_1")

    door_hinge = model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=enclosure,
        child=door,
        origin=Origin(xyz=(0.315, -0.096, 0.500)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=-DOOR_OPEN_ANGLE, upper=0.45),
        meta={"positive_motion": "front cover swings farther open around the right-side hinge"},
    )

    # Two real breaker toggles are separate movable controls in the breaker banks.
    for idx, (x, z) in enumerate([(-0.085, 0.565), (0.085, 0.485)]):
        toggle = model.part(f"toggle_{idx}", meta={"role": "movable breaker handle"})
        _box(toggle, (0.024, 0.010, 0.040), (0.0, -0.004, 0.020), dark_handle, "toggle_paddle")
        _box(toggle, (0.032, 0.004, 0.010), (0.0, 0.002, 0.002), black_plastic, "toggle_pivot_block")
        model.articulation(
            f"toggle_pivot_{idx}",
            ArticulationType.REVOLUTE,
            parent=enclosure,
            child=toggle,
            origin=Origin(xyz=(x, -0.104, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=5.0, lower=-0.55, upper=0.55),
            meta={"positive_motion": "breaker handle tips outward and downward"},
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    enclosure = object_model.get_part("enclosure")
    door = object_model.get_part("front_door")
    hinge = object_model.get_articulation("door_hinge")
    toggle_0 = object_model.get_part("toggle_0")
    toggle_joint = object_model.get_articulation("toggle_pivot_0")

    ctx.check(
        "small class is distribution board panel",
        object_model.name == "distribution_board_panel"
        and object_model.meta.get("small_class") == "Distribution board panel",
        details=f"name={object_model.name}, meta={object_model.meta}",
    )
    visual_names = {visual.name for visual in enclosure.visuals}
    for required in [
        "back_pan",
        "recessed_deadfront",
        "breaker_well_0",
        "breaker_well_1",
        "sub_feed_terminal_block",
        "neutral_bar",
        "earth_bar",
        "top_gland_0",
        "bottom_gland_0",
        "sub_feed_live_wire",
        "legend_label",
        "door_leaf",
    ]:
        target_names = visual_names if required != "door_leaf" else {visual.name for visual in door.visuals}
        ctx.check(f"has {required}", required in target_names, details=f"available={sorted(target_names)[:12]}")

    breaker_count = sum(1 for name in visual_names if name.startswith("breaker_") and name.count("_") == 2)
    terminal_screw_count = sum(1 for name in visual_names if name.endswith("_screw_0") or "_screw_" in name)
    ctx.check("has two rows of breaker modules", breaker_count >= 24, details=f"breaker_count={breaker_count}")
    ctx.check("has many terminal screws", terminal_screw_count >= 18, details=f"screw_count={terminal_screw_count}")
    ctx.check(
        "has non-fixed mechanisms",
        len([j for j in object_model.articulations if str(j.articulation_type).upper().endswith("REVOLUTE")]) >= 3,
        details=f"articulations={[j.name for j in object_model.articulations]}",
    )

    for barrel in ("hinge_barrel_0", "hinge_barrel_1"):
        ctx.allow_overlap(
            enclosure,
            door,
            elem_a="hinge_pin",
            elem_b=barrel,
            reason="The grounded hinge pin is intentionally captured inside the moving door knuckle.",
        )
        ctx.expect_within(
            enclosure,
            door,
            axes="xy",
            inner_elem="hinge_pin",
            outer_elem=barrel,
            margin=0.002,
            name=f"{barrel} captures hinge pin in plan",
        )
        ctx.expect_overlap(
            enclosure,
            door,
            axes="z",
            elem_a="hinge_pin",
            elem_b=barrel,
            min_overlap=0.090,
            name=f"{barrel} overlaps hinge pin along barrel length",
        )

    ctx.expect_gap(
        enclosure,
        door,
        axis="y",
        positive_elem="front_right_rail",
        negative_elem="door_leaf",
        min_gap=0.005,
        max_gap=0.050,
        name="closed door sits proud of front rail",
    )
    with ctx.pose({hinge: -DOOR_OPEN_ANGLE}):
        ctx.expect_overlap(
            door,
            enclosure,
            axes="xz",
            elem_a="door_leaf",
            elem_b="front_right_rail",
            min_overlap=0.030,
            name="closed door covers the cabinet opening height",
        )
        closed_aabb = ctx.part_element_world_aabb(door, elem="door_leaf")
        ctx.expect_gap(
            enclosure,
            door,
            axis="y",
            positive_elem="front_right_rail",
            negative_elem="door_leaf",
            min_gap=0.005,
            max_gap=0.050,
            name="closed door clears front rail without collision",
        )
    open_aabb = ctx.part_element_world_aabb(door, elem="door_leaf")
    ctx.check(
        "door opens outward on side hinge",
        open_aabb is not None
        and closed_aabb is not None
        and open_aabb[0][1] < closed_aabb[0][1] - 0.15,
        details=f"open_aabb={open_aabb}, closed_aabb={closed_aabb}",
    )

    rest_toggle = ctx.part_world_aabb(toggle_0)
    with ctx.pose({toggle_joint: 0.50}):
        moved_toggle = ctx.part_world_aabb(toggle_0)
    ctx.check(
        "breaker toggle tips outward",
        rest_toggle is not None
        and moved_toggle is not None
        and moved_toggle[0][1] < rest_toggle[0][1] - 0.006,
        details=f"rest={rest_toggle}, moved={moved_toggle}",
    )

    # Variant-specific: MCB-only sub-board must NOT have the main breaker
    # busbar assembly, and MUST have the sub-feed terminal block instead.
    ctx.check(
        "MCB-only variant: no main busbar present",
        "copper_bus_bar" not in visual_names,
        details=f"copper_bus_bar should be absent from enclosure visuals",
    )
    ctx.check(
        "MCB-only variant: sub_feed_terminal_block present",
        "sub_feed_terminal_block" in visual_names,
        details=f"sub_feed_terminal_block should be present in enclosure visuals",
    )
    ctx.check(
        "MCB-only variant: MCB field intact with two breaker wells",
        "breaker_well_0" in visual_names and "breaker_well_1" in visual_names,
        details="Both breaker wells must remain in the MCB-only sub-board",
    )

    return ctx.report()


object_model = build_object_model()
