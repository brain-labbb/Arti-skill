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
    mesh_from_geometry,
    tube_from_spline_points,
)


BOARD_W = 1.20
BOARD_H = 1.10
BOARD_D = 0.18
FLANGE_D = 0.025  # short bent flange depth for open backplate rigidity
BACK_Y = 0.0
LEFT_X = -BOARD_W / 2.0
RIGHT_X = BOARD_W / 2.0
DIVIDER_X = -0.205


def _box(part, name, size, xyz, material):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _cyl(part, name, radius, length, xyz, material, rpy=(0.0, 0.0, 0.0)):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _wire(part, name, points, material, radius=0.006, segments=16):
    geom = tube_from_spline_points(
        points,
        radius=radius,
        samples_per_segment=10,
        radial_segments=segments,
        cap_ends=True,
    )
    part.visual(mesh_from_geometry(geom, name), material=material, name=name)


def _add_cable_bundle(part, prefix, xs, z_start, z_end, x_term, material, color_sleeves=()):
    for i, x in enumerate(xs):
        _wire(
            part,
            f"{prefix}_wire_{i}",
            [
                (x, -0.020, z_start),
                (x + 0.010, -0.055, z_start - 0.055),
                (x_term + 0.020 * (i - len(xs) / 2.0), -0.061, z_end + 0.045),
                (x_term + 0.020 * (i - len(xs) / 2.0), -0.068, z_end),
            ],
            material,
            radius=0.0055,
        )
        if i < len(color_sleeves):
            _cyl(
                part,
                f"{prefix}_ferrule_{i}",
                0.0075,
                0.030,
                (x_term + 0.020 * (i - len(xs) / 2.0), -0.071, z_end + 0.022),
                color_sleeves[i],
                rpy=(math.pi / 2.0, 0.0, 0.0),
            )


def _add_breaker_row(part, row_name, z, x0, count, materials, articulated_slots=()):
    """A DIN row: metal rail, white MCB bodies, screw terminals, and toggle pockets."""
    metal = materials["galvanized"]
    white = materials["white_plastic"]
    dark = materials["black_plastic"]
    screw = materials["dark_screw"]
    green = materials["green_label"]
    blue = materials["blue_plastic"]

    row_w = count * 0.034 + 0.060
    _box(part, f"{row_name}_din_rail", (row_w + 0.080, 0.012, 0.018), (x0 + row_w / 2.0, -0.006, z), metal)
    _box(part, f"{row_name}_upper_body", (row_w, 0.048, 0.046), (x0 + row_w / 2.0, -0.036, z + 0.030), white)
    _box(part, f"{row_name}_lower_body", (row_w, 0.048, 0.046), (x0 + row_w / 2.0, -0.036, z - 0.030), white)
    _box(part, f"{row_name}_center_step", (row_w, 0.011, 0.012), (x0 + row_w / 2.0, -0.063, z), white)
    _box(part, f"{row_name}_label_strip", (row_w * 0.86, 0.003, 0.008), (x0 + row_w / 2.0, -0.066, z + 0.004), green)
    _box(part, f"{row_name}_end_terminal", (0.018, 0.050, 0.070), (x0 + row_w + 0.023, -0.036, z), blue)

    # module seams, terminal screws, and non-articulated toggles
    for i in range(count):
        x = x0 + 0.045 + i * 0.034
        _box(part, f"{row_name}_module_seam_{i}", (0.0020, 0.0035, 0.082), (x - 0.017, -0.0665, z), materials["panel_shadow"])
        _cyl(part, f"{row_name}_upper_screw_{i}", 0.0050, 0.003, (x, -0.067, z + 0.044), screw, rpy=(math.pi / 2.0, 0.0, 0.0))
        _cyl(part, f"{row_name}_lower_screw_{i}", 0.0050, 0.003, (x, -0.067, z - 0.044), screw, rpy=(math.pi / 2.0, 0.0, 0.0))
        if i not in articulated_slots:
            _box(part, f"{row_name}_toggle_{i}", (0.014, 0.012, 0.030), (x, -0.071, z - 0.006), dark)


def _add_left_power_devices(part, materials):
    metal = materials["galvanized"]
    gray = materials["breaker_gray"]
    white = materials["white_plastic"]
    dark = materials["black_plastic"]
    screw = materials["dark_screw"]
    copper = materials["copper"]
    brass = materials["brass"]

    _box(part, "left_clear_device_shroud", (0.345, 0.080, 0.740), (-0.405, -0.050, 0.515), materials["transparent_window"])

    # back-mounted shelf ribs tie the left bay devices into the backplate.
    for z in (0.270, 0.565, 0.840):
        _box(part, f"left_support_shelf_{int(z*1000)}", (0.360, 0.012, 0.018), (-0.405, -0.006, z), metal)

    for idx, z in enumerate((0.370, 0.690)):
        _box(part, f"main_breaker_{idx}_case", (0.165, 0.060, 0.145), (-0.460, -0.042, z), gray)
        _box(part, f"main_breaker_{idx}_green_label", (0.060, 0.003, 0.045), (-0.490, -0.074, z + 0.018), materials["green_label"])
        _box(part, f"main_breaker_{idx}_warning_label", (0.098, 0.003, 0.020), (-0.455, -0.075, z + 0.060), materials["orange_label"])
        _box(part, f"main_breaker_{idx}_black_handle", (0.030, 0.016, 0.074), (-0.405, -0.080, z - 0.005), dark)
        for sx in (-0.055, 0.0, 0.055):
            _cyl(part, f"main_breaker_{idx}_top_screw_{sx:.2f}", 0.006, 0.004, (-0.460 + sx, -0.076, z + 0.080), screw, rpy=(math.pi / 2.0, 0.0, 0.0))
            _cyl(part, f"main_breaker_{idx}_bottom_screw_{sx:.2f}", 0.006, 0.004, (-0.460 + sx, -0.076, z - 0.080), screw, rpy=(math.pi / 2.0, 0.0, 0.0))

    # vertical copper bus bars between the main switch bodies, with colored sleeves.
    bar_xs = [-0.520, -0.475, -0.430, -0.385]
    sleeve_mats = [materials["red_jacket"], white, materials["yellow_jacket"], materials["blue_jacket"]]
    for i, x in enumerate(bar_xs):
        _box(part, f"phase_busbar_{i}", (0.018, 0.012, 0.210), (x, -0.075, 0.530), copper if i != 3 else brass)
        _box(part, f"phase_sleeve_{i}", (0.025, 0.014, 0.165), (x, -0.083, 0.530), sleeve_mats[i])

    # auxiliary three-pole breaker and blue terminal module.
    _box(part, "aux_breaker_case", (0.110, 0.050, 0.120), (-0.315, -0.040, 0.700), white)
    for i in range(3):
        x = -0.345 + i * 0.030
        _box(part, f"aux_breaker_toggle_{i}", (0.014, 0.012, 0.045), (x, -0.071, 0.692), dark)
        _cyl(part, f"aux_breaker_screw_{i}", 0.005, 0.003, (x, -0.067, 0.755), screw, rpy=(math.pi / 2.0, 0.0, 0.0))
    _box(part, "blue_meter_body", (0.090, 0.055, 0.075), (-0.305, -0.042, 0.330), materials["translucent_blue"])
    for i in range(4):
        _box(part, f"blue_meter_fin_{i}", (0.008, 0.004, 0.065), (-0.337 + i * 0.020, -0.073, 0.330), materials["blue_plastic"])

    # cable loops to main devices and meter.
    _add_cable_bundle(
        part,
        "left_top",
        [-0.525, -0.490, -0.455, -0.420],
        0.880,
        0.790,
        -0.460,
        materials["black_rubber"],
        color_sleeves=(materials["blue_jacket"], white, materials["red_jacket"], materials["blue_jacket"]),
    )
    for i, x in enumerate((-0.335, -0.315, -0.295)):
        _wire(
            part,
            f"meter_drop_cable_{i}",
            [(x, -0.074, 0.292), (x - 0.025, -0.095, 0.235), (-0.355 + i * 0.025, -0.096, 0.175)],
            materials["black_rubber"] if i == 0 else (materials["red_jacket"] if i == 1 else materials["blue_jacket"]),
            radius=0.006,
        )


def _add_backplate(backplate, materials):
    """Open backplate chassis: flat mounting plate with bent flanges, no door."""
    sheet = materials["painted_metal"]
    screw = materials["dark_screw"]
    galvan = materials["galvanized"]

    # Main backplate sheet — the primary mounting surface
    _box(backplate, "backplate_sheet", (BOARD_W, 0.012, BOARD_H), (0.0, 0.006, BOARD_H / 2.0), sheet)

    # Bent flanges on all four edges for rigidity (typical of open backplates)
    _box(backplate, "top_flange", (BOARD_W, FLANGE_D, 0.018), (0.0, -FLANGE_D / 2.0 + 0.012, BOARD_H - 0.009), sheet)
    _box(backplate, "bottom_flange", (BOARD_W, FLANGE_D, 0.018), (0.0, -FLANGE_D / 2.0 + 0.012, 0.009), sheet)
    _box(backplate, "left_flange", (0.018, FLANGE_D, BOARD_H), (LEFT_X + 0.009, -FLANGE_D / 2.0 + 0.012, BOARD_H / 2.0), sheet)
    _box(backplate, "right_flange", (0.018, FLANGE_D, BOARD_H), (RIGHT_X - 0.009, -FLANGE_D / 2.0 + 0.012, BOARD_H / 2.0), sheet)

    # Structural center divider (welded to backplate)
    _box(backplate, "center_divider", (0.020, 0.040, BOARD_H), (DIVIDER_X, -0.020, BOARD_H / 2.0), sheet)

    # Knockouts/glands and conduits across the top
    for i, x in enumerate((-0.480, -0.360, 0.000, 0.180, 0.360)):
        _cyl(backplate, f"top_knockout_ring_{i}", 0.027, 0.010, (x, -0.005, BOARD_H + 0.005), galvan)
        _cyl(backplate, f"conduit_stub_{i}", 0.020, 0.125, (x, -0.005, BOARD_H + 0.067), galvan)

    # Bottom cable entry knockouts
    for i, x in enumerate((-0.400, -0.200, 0.100, 0.300, 0.480)):
        _cyl(backplate, f"bottom_knockout_{i}", 0.022, 0.010, (x, -0.005, -0.005), galvan)

    # Back-panel wall mounting holes (four corners)
    for i, (x, z) in enumerate(((-0.545, 1.025), (0.545, 1.025), (-0.545, 0.075), (0.545, 0.075))):
        _cyl(backplate, f"mounting_hole_{i}", 0.012, 0.003, (x, -0.001, z), screw, rpy=(math.pi / 2.0, 0.0, 0.0))

    # DIN rail mounting hole columns on each side of center divider
    for side_x, side in ((DIVIDER_X + 0.030, "center"), (RIGHT_X - 0.035, "right"), (LEFT_X + 0.035, "left")):
        for j in range(9):
            _cyl(backplate, f"{side}_rail_hole_{j}", 0.004, 0.004, (side_x, 0.002, 0.170 + j * 0.095), screw, rpy=(math.pi / 2.0, 0.0, 0.0))

    # Earth bonding stud (prominent brass stud on backplate)
    _cyl(backplate, "earth_bonding_stud", 0.008, 0.016, (-0.540, -0.008, 0.060), materials["brass"], rpy=(math.pi / 2.0, 0.0, 0.0))
    _box(backplate, "earth_label", (0.040, 0.002, 0.020), (-0.540, -0.017, 0.060), materials["green_label"])


def _add_right_breaker_bank(part, materials):
    _box(part, "right_clear_bus_shroud", (0.660, 0.065, 0.870), (0.205, -0.0425, 0.485), materials["transparent_window"])
    for z, name in ((0.835, "upper_row"), (0.565, "middle_row"), (0.295, "lower_row")):
        _add_breaker_row(part, name, z, -0.115, 14, materials, articulated_slots={1, 2, 3, 4} if name == "middle_row" else set())
        _add_cable_bundle(
            part,
            f"{name}_left",
            [-0.095, -0.068, -0.040, -0.012],
            z + 0.140,
            z + 0.060,
            -0.045,
            materials["black_rubber"],
            color_sleeves=(materials["red_jacket"], materials["yellow_jacket"], materials["blue_jacket"], materials["blue_jacket"]),
        )
        _wire(
            part,
            f"{name}_right_feed",
            [(0.445, -0.048, z + 0.090), (0.560, -0.050, z + 0.070), (0.548, -0.034, z + 0.000)],
            materials["black_rubber"],
            radius=0.0055,
        )

    # Brass neutral/earth bar along the bottom with individual screw clamps.
    _box(part, "bottom_earth_bar", (0.360, 0.020, 0.018), (0.230, -0.030, 0.086), materials["brass"])
    for i in range(10):
        x = 0.075 + i * 0.034
        _cyl(part, f"earth_bar_screw_{i}", 0.006, 0.006, (x, -0.043, 0.099), materials["dark_screw"], rpy=(math.pi / 2.0, 0.0, 0.0))
        _box(part, f"earth_bar_terminal_slot_{i}", (0.016, 0.004, 0.006), (x, -0.045, 0.083), materials["panel_shadow"])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="electrical_distribution_board_panel")

    materials = {
        "painted_metal": model.material("powder_coated_light_gray_sheet_metal", rgba=(0.82, 0.81, 0.74, 1.0)),
        "panel_shadow": model.material("dark_recess_shadow", rgba=(0.035, 0.038, 0.035, 1.0)),
        "galvanized": model.material("galvanized_zinc_metal", rgba=(0.58, 0.60, 0.57, 1.0)),
        "white_plastic": model.material("molded_white_breaker_plastic", rgba=(0.94, 0.94, 0.90, 1.0)),
        "black_plastic": model.material("black_molded_plastic", rgba=(0.015, 0.016, 0.015, 1.0)),
        "breaker_gray": model.material("gray_molded_breaker_case", rgba=(0.38, 0.40, 0.39, 1.0)),
        "dark_screw": model.material("dark_phosphate_screw_heads", rgba=(0.08, 0.08, 0.075, 1.0)),
        "copper": model.material("exposed_copper_busbar", rgba=(0.84, 0.38, 0.14, 1.0)),
        "brass": model.material("brass_terminal_bar", rgba=(0.86, 0.65, 0.29, 1.0)),
        "green_label": model.material("green_white_device_label", rgba=(0.12, 0.70, 0.46, 1.0)),
        "orange_label": model.material("orange_caution_label", rgba=(1.0, 0.28, 0.06, 1.0)),
        "blue_plastic": model.material("blue_terminal_plastic", rgba=(0.02, 0.19, 0.78, 1.0)),
        "translucent_blue": model.material("translucent_blue_meter_cover", rgba=(0.05, 0.31, 0.74, 0.72)),
        "transparent_window": model.material("slightly_smoked_clear_window", rgba=(0.68, 0.83, 0.90, 0.34)),
        "black_rubber": model.material("black_rubber_cable_jacket", rgba=(0.005, 0.005, 0.005, 1.0)),
        "red_jacket": model.material("red_phase_insulation", rgba=(0.78, 0.02, 0.02, 1.0)),
        "yellow_jacket": model.material("yellow_phase_insulation", rgba=(0.94, 0.80, 0.05, 1.0)),
        "blue_jacket": model.material("blue_neutral_insulation", rgba=(0.02, 0.20, 0.88, 1.0)),
        "earth_wire": model.material("green_yellow_earth_wire", rgba=(0.34, 0.82, 0.10, 1.0)),
    }

    # Open backplate chassis — no enclosure walls, no door
    backplate = model.part("backplate")
    _add_backplate(backplate, materials)

    breaker_bank = model.part("breaker_bank")
    _add_right_breaker_bank(breaker_bank, materials)
    model.articulation(
        "backplate_to_breaker_bank",
        ArticulationType.FIXED,
        parent=backplate,
        child=breaker_bank,
        origin=Origin(),
    )

    left_devices = model.part("left_devices")
    _add_left_power_devices(left_devices, materials)
    model.articulation(
        "backplate_to_left_devices",
        ArticulationType.FIXED,
        parent=backplate,
        child=left_devices,
        origin=Origin(),
    )

    # Four individual MCB handles are real moving controls; the rest of the row is static detail.
    for i, x in enumerate((-0.036, -0.002, 0.032, 0.066)):
        toggle = model.part(f"breaker_toggle_{i}")
        _box(toggle, "toggle_paddle", (0.014, 0.014, 0.034), (0.0, -0.006, -0.006), materials["black_plastic"])
        _box(toggle, "toggle_axle_cap", (0.020, 0.006, 0.010), (0.0, 0.000, 0.010), materials["panel_shadow"])
        model.articulation(
            f"toggle_pivot_{i}",
            ArticulationType.REVOLUTE,
            parent=breaker_bank,
            child=toggle,
            origin=Origin(xyz=(x, -0.078, 0.565)),
            # Rocker pivots about the horizontal module width; positive throws the handle down.
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(lower=-0.42, upper=0.42, effort=1.0, velocity=3.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    backplate = object_model.get_part("backplate")
    breaker_bank = object_model.get_part("breaker_bank")
    left_devices = object_model.get_part("left_devices")
    toggle_joint = object_model.get_articulation("toggle_pivot_0")

    # Variant identity: open backplate, no door
    ctx.check(
        "small class is distribution board panel",
        object_model.name == "electrical_distribution_board_panel",
        details=f"model name is {object_model.name}",
    )
    part_names = {p.name for p in object_model.parts}
    joint_names = {a.name for a in object_model.articulations}
    ctx.check(
        "open backplate form has no door parts",
        "left_door" not in part_names and "right_door" not in part_names,
        details="door parts should be removed in the open backplate variant",
    )
    ctx.check(
        "no door hinge articulations exist",
        "left_door_hinge" not in joint_names and "right_door_hinge" not in joint_names,
        details="door hinge joints should be removed in the open backplate variant",
    )
    ctx.check(
        "backplate has open chassis geometry",
        backplate.get_visual("backplate_sheet") is not None
        and backplate.get_visual("top_flange") is not None
        and backplate.get_visual("bottom_flange") is not None
        and backplate.get_visual("left_flange") is not None
        and backplate.get_visual("right_flange") is not None,
        details="open backplate should have a flat plate with bent flanges, not full enclosure walls",
    )
    ctx.check(
        "contains electrical subassemblies on backplate",
        all(p is not None for p in (backplate, breaker_bank, left_devices)),
        details="missing backplate, breaker bank, or left devices",
    )

    # Breaker toggles remain the primary non-fixed articulation
    ctx.check(
        "breaker toggle is a separate movable control",
        toggle_joint.articulation_type == ArticulationType.REVOLUTE
        and toggle_joint.motion_limits.lower < 0.0
        and toggle_joint.motion_limits.upper > 0.0,
        details="at least one miniature breaker handle must rock about its pivot",
    )

    # Named electrical details exist
    ctx.check(
        "named electrical details exist",
        backplate.get_visual("backplate_sheet") is not None
        and backplate.get_visual("center_divider") is not None
        and backplate.get_visual("earth_bonding_stud") is not None
        and breaker_bank.get_visual("upper_row_din_rail") is not None
        and breaker_bank.get_visual("bottom_earth_bar") is not None
        and left_devices.get_visual("left_support_shelf_565") is not None,
        details="missing named backplate, DIN rail, earth bar, shelf, or bonding stud visuals",
    )

    # Backplate has cable management
    ctx.check(
        "backplate has cable entry knockouts",
        backplate.get_visual("top_knockout_ring_0") is not None
        and backplate.get_visual("bottom_knockout_0") is not None,
        details="open backplate should have cable entry knockouts",
    )

    # Mounting relationships
    ctx.expect_contact(
        breaker_bank,
        backplate,
        contact_tol=0.001,
        name="breaker bank is mounted to the backplate",
    )
    ctx.expect_contact(
        left_devices,
        backplate,
        contact_tol=0.001,
        name="left power devices are mounted to the backplate",
    )
    ctx.expect_within(
        breaker_bank,
        backplate,
        axes="xz",
        margin=0.020,
        name="breaker rows and terminal bars stay inside the backplate footprint",
    )

    # Toggle articulation proof
    toggle = object_model.get_part("breaker_toggle_0")
    toggle_rest = ctx.part_world_aabb(toggle)
    with ctx.pose({toggle_joint: 0.40}):
        toggle_thrown = ctx.part_world_aabb(toggle)
    ctx.check(
        "breaker handle rocks visibly",
        toggle_rest is not None
        and toggle_thrown is not None
        and abs(toggle_thrown[0][2] - toggle_rest[0][2]) > 0.002,
        details=f"rest={toggle_rest}, thrown={toggle_thrown}",
    )

    return ctx.report()


object_model = build_object_model()
