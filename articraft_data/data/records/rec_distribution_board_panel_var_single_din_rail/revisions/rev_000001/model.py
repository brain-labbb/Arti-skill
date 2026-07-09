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
FRONT_Y = -0.180
BACK_INNER_Y = 0.0
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

    # back-mounted shelf ribs tie the left bay devices into the sheet metal back.
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


def _add_enclosure_shell(enclosure, materials):
    sheet = materials["painted_metal"]
    shadow = materials["panel_shadow"]
    screw = materials["dark_screw"]
    galvan = materials["galvanized"]

    _box(enclosure, "back_sheet", (BOARD_W, 0.012, BOARD_H), (0.0, 0.006, BOARD_H / 2.0), sheet)
    _box(enclosure, "left_wall", (0.018, BOARD_D, BOARD_H), (LEFT_X + 0.009, -0.081, BOARD_H / 2.0), sheet)
    _box(enclosure, "right_wall", (0.018, BOARD_D, BOARD_H), (RIGHT_X - 0.009, -0.081, BOARD_H / 2.0), sheet)
    _box(enclosure, "top_wall", (BOARD_W, BOARD_D, 0.018), (0.0, -0.081, BOARD_H - 0.009), sheet)
    _box(enclosure, "bottom_wall", (BOARD_W, BOARD_D, 0.018), (0.0, -0.081, 0.009), sheet)
    _box(enclosure, "center_divider", (0.020, BOARD_D, BOARD_H), (DIVIDER_X, -0.081, BOARD_H / 2.0), sheet)

    # Raised front gutter/lip around both openings and the center mullion.
    _box(enclosure, "front_top_lip", (BOARD_W, 0.018, 0.038), (0.0, FRONT_Y + 0.009, BOARD_H - 0.019), sheet)
    _box(enclosure, "front_bottom_lip", (BOARD_W, 0.018, 0.038), (0.0, FRONT_Y + 0.009, 0.019), sheet)
    _box(enclosure, "front_left_lip", (0.040, 0.018, BOARD_H), (LEFT_X + 0.020, FRONT_Y + 0.009, BOARD_H / 2.0), sheet)
    _box(enclosure, "front_right_lip", (0.040, 0.018, BOARD_H), (RIGHT_X - 0.020, FRONT_Y + 0.009, BOARD_H / 2.0), sheet)
    _box(enclosure, "front_center_mullion", (0.052, 0.020, BOARD_H), (DIVIDER_X, FRONT_Y + 0.010, BOARD_H / 2.0), sheet)
    for x, nm in ((-0.405, "left"), (0.205, "right")):
        _box(enclosure, f"{nm}_inner_recess_shadow", (0.300 if nm == "left" else 0.610, 0.003, 0.030), (x, FRONT_Y - 0.0005, 0.049), shadow)
        _box(enclosure, f"{nm}_top_recess_shadow", (0.300 if nm == "left" else 0.610, 0.003, 0.030), (x, FRONT_Y - 0.0005, BOARD_H - 0.049), shadow)

    # Door-side hinge leaves and alternating fixed knuckles.
    for x, sign, side in ((LEFT_X - 0.014, -1.0, "left"), (RIGHT_X + 0.014, 1.0, "right")):
        for i, zc in enumerate((0.180, 0.545, 0.910)):
            _box(enclosure, f"{side}_hinge_leaf_{i}", (0.026, 0.006, 0.132), (x - sign * 0.007, FRONT_Y + 0.005, zc), galvan)
            _cyl(enclosure, f"{side}_fixed_knuckle_{i}", 0.010, 0.132, (x, FRONT_Y - 0.004, zc), galvan)

    # Knockouts/glands and conduits across the top.
    for i, x in enumerate((-0.480, -0.360, 0.000, 0.180, 0.360)):
        _cyl(enclosure, f"top_knockout_ring_{i}", 0.027, 0.010, (x, -0.080, BOARD_H + 0.005), galvan)
        _cyl(enclosure, f"conduit_stub_{i}", 0.020, 0.125, (x, -0.080, BOARD_H + 0.067), galvan)

    # Back-panel mounting holes and side rows of adjustment holes.
    for i, (x, z) in enumerate(((-0.545, 1.025), (0.545, 1.025), (-0.545, 0.075), (0.545, 0.075))):
        _cyl(enclosure, f"mounting_hole_dark_{i}", 0.012, 0.003, (x, -0.001, z), screw, rpy=(math.pi / 2.0, 0.0, 0.0))
    for side_x, side in ((DIVIDER_X + 0.030, "center"), (RIGHT_X - 0.035, "right")):
        for j in range(9):
            _cyl(enclosure, f"{side}_rail_hole_{j}", 0.004, 0.003, (side_x, FRONT_Y + 0.004, 0.170 + j * 0.095), screw, rpy=(math.pi / 2.0, 0.0, 0.0))


def _add_right_breaker_bank(part, materials):
    # Single horizontal DIN rail — one row of MCBs (small sub-board / lighting board).
    row_z = 0.550
    _box(part, "right_clear_bus_shroud", (0.660, 0.065, 0.300), (0.205, -0.0425, row_z), materials["transparent_window"])
    _add_breaker_row(part, "single_row", row_z, -0.115, 14, materials, articulated_slots={1, 2, 3, 4})
    _add_cable_bundle(
        part,
        "single_row_left",
        [-0.095, -0.068, -0.040, -0.012],
        row_z + 0.140,
        row_z + 0.060,
        -0.045,
        materials["black_rubber"],
        color_sleeves=(materials["red_jacket"], materials["yellow_jacket"], materials["blue_jacket"], materials["blue_jacket"]),
    )
    _wire(
        part,
        "single_row_right_feed",
        [(0.445, -0.048, row_z + 0.090), (0.560, -0.050, row_z + 0.070), (0.548, -0.034, row_z + 0.000)],
        materials["black_rubber"],
        radius=0.0055,
    )

    # Brass neutral/earth bar along the bottom with individual screw clamps.
    _box(part, "bottom_earth_bar", (0.360, 0.020, 0.018), (0.230, -0.030, 0.086), materials["brass"])
    for i in range(10):
        x = 0.075 + i * 0.034
        _cyl(part, f"earth_bar_screw_{i}", 0.006, 0.006, (x, -0.043, 0.099), materials["dark_screw"], rpy=(math.pi / 2.0, 0.0, 0.0))
        _box(part, f"earth_bar_terminal_slot_{i}", (0.016, 0.004, 0.006), (x, -0.045, 0.083), materials["panel_shadow"])


def _add_door(part, side, materials):
    sheet = materials["painted_metal"]
    glass = materials["transparent_window"]
    dark = materials["black_plastic"]
    galvan = materials["galvanized"]
    screw = materials["dark_screw"]

    if side == "left":
        width = 0.390
        sx = 1.0
        center_x = width / 2.0
        latch_x = width - 0.030
    else:
        width = 0.780
        sx = -1.0
        center_x = -width / 2.0
        latch_x = -width + 0.030

    # Pressed sheet-metal frame with transparent window.
    _box(part, "door_outer_panel", (width, 0.012, 1.050), (center_x, -0.012, 0.550), sheet)
    _box(part, "window_glass", (width - 0.130, 0.004, 0.770), (center_x, -0.020, 0.560), glass)
    _box(part, "window_top_rail", (width - 0.080, 0.014, 0.030), (center_x, -0.026, 0.960), sheet)
    _box(part, "window_bottom_rail", (width - 0.080, 0.014, 0.030), (center_x, -0.026, 0.160), sheet)
    _box(part, "window_hinge_stile", (0.034, 0.014, 0.850), (sx * 0.040, -0.026, 0.560), sheet)
    _box(part, "window_latch_stile", (0.034, 0.014, 0.850), (latch_x, -0.026, 0.560), sheet)
    _box(part, "door_gasket_shadow", (width - 0.055, 0.004, 0.950), (center_x, -0.032, 0.550), materials["panel_shadow"])

    # Rotating door knuckles in the hinge gaps, plus short leaf straps.
    for i, zc in enumerate((0.360, 0.725)):
        _cyl(part, f"door_knuckle_{i}", 0.010, 0.150, (0.0, -0.004, zc), galvan)
        _box(part, f"door_hinge_leaf_{i}", (0.030, 0.006, 0.130), (sx * 0.014, -0.013, zc), galvan)
        _cyl(part, f"hinge_pin_screw_{i}", 0.004, 0.003, (sx * 0.030, -0.018, zc + 0.035), screw, rpy=(math.pi / 2.0, 0.0, 0.0))

    # Paddle latch/lock and an earth bonding lead fixed to the door.
    _box(part, "latch_plate", (0.036, 0.012, 0.100), (latch_x, -0.040, 0.540), dark)
    _cyl(part, "round_lock_core", 0.012, 0.010, (latch_x, -0.048, 0.575), galvan, rpy=(math.pi / 2.0, 0.0, 0.0))
    _wire(
        part,
        "green_yellow_bond_wire",
        [
            (latch_x * 0.98, -0.036, 0.200),
            (latch_x * 0.70, -0.060, 0.120),
            (latch_x * 0.38, -0.050, 0.090),
        ],
        materials["earth_wire"],
        radius=0.0045,
        segments=12,
    )


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

    enclosure = model.part("enclosure")
    _add_enclosure_shell(enclosure, materials)

    breaker_bank = model.part("breaker_bank")
    _add_right_breaker_bank(breaker_bank, materials)
    model.articulation(
        "enclosure_to_breaker_bank",
        ArticulationType.FIXED,
        parent=enclosure,
        child=breaker_bank,
        origin=Origin(),
    )

    left_devices = model.part("left_devices")
    _add_left_power_devices(left_devices, materials)
    model.articulation(
        "enclosure_to_left_devices",
        ArticulationType.FIXED,
        parent=enclosure,
        child=left_devices,
        origin=Origin(),
    )

    left_door = model.part("left_door")
    _add_door(left_door, "left", materials)
    model.articulation(
        "left_door_hinge",
        ArticulationType.REVOLUTE,
        parent=enclosure,
        child=left_door,
        origin=Origin(xyz=(LEFT_X - 0.014, FRONT_Y - 0.004, 0.0)),
        # The closed left door extends along local +X; negative Z opens it toward front -Y.
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.75, effort=12.0, velocity=1.1),
    )

    right_door = model.part("right_door")
    _add_door(right_door, "right", materials)
    model.articulation(
        "right_door_hinge",
        ArticulationType.REVOLUTE,
        parent=enclosure,
        child=right_door,
        origin=Origin(xyz=(RIGHT_X + 0.014, FRONT_Y - 0.004, 0.0)),
        # The closed right door extends along local -X; positive Z opens it toward front -Y.
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.75, effort=12.0, velocity=1.1),
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
            origin=Origin(xyz=(x, -0.078, 0.550)),
            # Rocker pivots about the horizontal module width; positive throws the handle down.
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(lower=-0.42, upper=0.42, effort=1.0, velocity=3.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    enclosure = object_model.get_part("enclosure")
    breaker_bank = object_model.get_part("breaker_bank")
    left_devices = object_model.get_part("left_devices")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    left_hinge = object_model.get_articulation("left_door_hinge")
    right_hinge = object_model.get_articulation("right_door_hinge")
    toggle_joint = object_model.get_articulation("toggle_pivot_0")

    ctx.check(
        "small class is distribution board panel",
        object_model.name == "electrical_distribution_board_panel",
        details=f"model name is {object_model.name}",
    )
    ctx.check(
        "contains electrical enclosure subassemblies",
        all(p is not None for p in (enclosure, breaker_bank, left_devices, left_door, right_door)),
        details="missing enclosure, breaker bank, left devices, or hinged doors",
    )
    ctx.check(
        "front doors are hinged non fixed mechanisms",
        left_hinge.articulation_type == ArticulationType.REVOLUTE
        and right_hinge.articulation_type == ArticulationType.REVOLUTE
        and left_hinge.motion_limits.upper > 1.5
        and right_hinge.motion_limits.upper > 1.5,
        details="door hinge limits should open more than 85 degrees",
    )
    ctx.check(
        "breaker toggle is a separate movable control",
        toggle_joint.articulation_type == ArticulationType.REVOLUTE
        and toggle_joint.motion_limits.lower < 0.0
        and toggle_joint.motion_limits.upper > 0.0,
        details="at least one miniature breaker handle must rock about its pivot",
    )
    ctx.check(
        "single horizontal DIN rail topology",
        breaker_bank.get_visual("single_row_din_rail") is not None
        and breaker_bank.get_visual("single_row_upper_body") is not None
        and breaker_bank.get_visual("single_row_lower_body") is not None,
        details="single_row DIN rail and MCB bodies must exist for the lighting sub-board variant",
    )
    bank_visual_names = {v.name for v in breaker_bank.visuals}
    ctx.check(
        "no stacked rows in single rail variant",
        "upper_row_din_rail" not in bank_visual_names
        and "middle_row_din_rail" not in bank_visual_names
        and "lower_row_din_rail" not in bank_visual_names,
        details="single rail variant must not contain multiple stacked DIN rows",
    )
    ctx.check(
        "named electrical details exist",
        enclosure.get_visual("back_sheet") is not None
        and breaker_bank.get_visual("single_row_din_rail") is not None
        and left_devices.get_visual("left_support_shelf_565") is not None
        and left_door.get_visual("door_knuckle_0") is not None
        and right_door.get_visual("door_knuckle_0") is not None,
        details="missing named back sheet, DIN rail, shelf, or door-hinge knuckle visuals",
    )

    ctx.expect_contact(
        breaker_bank,
        enclosure,
        contact_tol=0.001,
        name="breaker bank is mounted to the metal enclosure",
    )
    ctx.expect_contact(
        left_devices,
        enclosure,
        contact_tol=0.001,
        name="left power devices are mounted to the metal enclosure",
    )
    ctx.expect_within(
        breaker_bank,
        enclosure,
        axes="xz",
        margin=0.020,
        name="breaker rows and terminal bars stay inside the enclosure footprint",
    )
    ctx.expect_overlap(
        left_door,
        enclosure,
        axes="z",
        min_overlap=0.600,
        name="left hinged door spans the enclosure height",
    )
    ctx.expect_overlap(
        right_door,
        enclosure,
        axes="z",
        min_overlap=0.600,
        name="right hinged door spans the enclosure height",
    )

    left_closed_aabb = ctx.part_world_aabb(left_door)
    right_closed_aabb = ctx.part_world_aabb(right_door)
    with ctx.pose({left_hinge: 1.20, right_hinge: 1.20}):
        left_open_aabb = ctx.part_world_aabb(left_door)
        right_open_aabb = ctx.part_world_aabb(right_door)
    ctx.check(
        "hinged doors swing outward toward the front",
        left_closed_aabb is not None
        and left_open_aabb is not None
        and right_closed_aabb is not None
        and right_open_aabb is not None
        and left_open_aabb[0][1] < left_closed_aabb[0][1] - 0.12
        and right_open_aabb[0][1] < right_closed_aabb[0][1] - 0.12,
        details=f"left closed/open={left_closed_aabb}/{left_open_aabb}, right closed/open={right_closed_aabb}/{right_open_aabb}",
    )

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
