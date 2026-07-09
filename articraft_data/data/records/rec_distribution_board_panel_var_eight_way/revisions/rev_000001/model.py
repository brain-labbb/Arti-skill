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

# --- Board sizing: compact 8-way consumer unit (2 banks × 4 rows) ---
ENC_W = 0.620
ENC_D = 0.120
ENC_H = 0.550
ENC_CZ = ENC_H / 2.0  # 0.275

BANKS_X = [-0.085, 0.085]
ROWS_PER_BANK = 4
ROW_SPACING = 0.040
FIELD_CZ = ENC_CZ + 0.025  # 0.300 — breaker field center

DOOR_W = 0.405
DOOR_H = ENC_H - 0.160  # 0.390


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
            "description": "Hinged sheet-metal 8-way compact electrical distribution board panel with breakers, bus bars, labels, glands, wiring, and lock hardware.",
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

    # ========== ENCLOSURE (root sheet-metal cabinet) ==========
    enclosure = model.part("enclosure", meta={"role": "root sheet-metal cabinet"})

    # Open-backed sheet-metal box with front flange
    _box(enclosure, (ENC_W, 0.010, ENC_H), (0.0, 0.055, ENC_CZ), painted, "back_pan")
    _box(enclosure, (0.026, ENC_D, ENC_H), (-0.297, 0.000, ENC_CZ), painted, "side_wall_0")
    _box(enclosure, (0.026, ENC_D, ENC_H), (0.297, 0.000, ENC_CZ), painted, "side_wall_1")
    _box(enclosure, (ENC_W, ENC_D, 0.026), (0.000, 0.000, ENC_H - 0.013), painted, "top_wall")
    _box(enclosure, (ENC_W, ENC_D, 0.026), (0.000, 0.000, 0.013), painted, "bottom_wall")

    # Front frame rails
    rail_h = ENC_H - 0.170  # 0.380
    _box(enclosure, (0.055, 0.008, rail_h), (-0.220, -0.071, ENC_CZ), painted, "front_left_rail")
    _box(enclosure, (0.055, 0.008, rail_h), (0.220, -0.071, ENC_CZ), painted, "front_right_rail")
    _box(enclosure, (0.440, 0.008, 0.040), (0.000, -0.071, ENC_H - 0.090), painted, "front_top_rail")
    _box(enclosure, (0.440, 0.008, 0.040), (0.000, -0.071, 0.090), painted, "front_bottom_rail")

    # Recessed deadfront and panel lips
    deadfront_h = ENC_H - 0.230  # 0.320
    _box(enclosure, (0.385, 0.006, deadfront_h), (0.000, -0.057, ENC_CZ), dark_paint, "recessed_deadfront")
    _box(enclosure, (0.370, 0.010, 0.035), (0.000, -0.069, ENC_CZ + deadfront_h / 2 + 0.010), painted, "upper_panel_lip")
    _box(enclosure, (0.370, 0.010, 0.035), (0.000, -0.069, ENC_CZ - deadfront_h / 2 - 0.010), painted, "lower_panel_lip")

    # Slotted screw holes on the front rails
    for side_x, prefix in [(-0.246, "left"), (0.246, "right")]:
        for i, z in enumerate([ENC_CZ - 0.135, ENC_CZ, ENC_CZ + 0.135]):
            _box(enclosure, (0.010, 0.003, 0.035), (side_x, -0.076, z), dark_paint, f"{prefix}_slot_{i}")

    # Visible screw heads on the deadfront
    for i, (x, z) in enumerate([
        (-0.180, ENC_CZ - 0.160), (0.180, ENC_CZ - 0.160),
        (-0.165, ENC_CZ + 0.165), (0.165, ENC_CZ + 0.165),
        (-0.190, ENC_CZ + 0.105), (0.190, ENC_CZ + 0.105),
    ]):
        _cylinder(enclosure, 0.0075, 0.004, (x, -0.074, z), galvanized, f"panel_screw_{i}", rpy=(math.pi / 2, 0.0, 0.0))

    # ========== BREAKER BANKS (2 banks × 4 rows = 8 ways) ==========
    well_h = ROWS_PER_BANK * ROW_SPACING + 0.080  # 0.240
    rail_h_bk = ROWS_PER_BANK * ROW_SPACING + 0.100  # 0.260
    breaker_z0 = FIELD_CZ - (ROWS_PER_BANK - 1) * ROW_SPACING / 2.0  # 0.240

    for bank, x0 in enumerate(BANKS_X):
        _box(enclosure, (0.078, 0.018, well_h), (x0, -0.076, FIELD_CZ), black_plastic, f"breaker_well_{bank}")
        _box(enclosure, (0.006, 0.022, rail_h_bk), (x0 - 0.043, -0.078, FIELD_CZ), black_plastic, f"breaker_side_rail_{bank}_0")
        _box(enclosure, (0.006, 0.022, rail_h_bk), (x0 + 0.043, -0.078, FIELD_CZ), black_plastic, f"breaker_side_rail_{bank}_1")
        for row in range(ROWS_PER_BANK):
            z = breaker_z0 + row * ROW_SPACING
            _box(enclosure, (0.071, 0.015, 0.030), (x0, -0.090, z), black_plastic, f"breaker_{bank}_{row}")
            _box(enclosure, (0.025, 0.006, 0.022), (x0, -0.100, z + 0.002), dark_handle, f"fixed_handle_{bank}_{row}")
            _box(enclosure, (0.010, 0.004, 0.004), (x0 + 0.030, -0.099, z + 0.010), label_white, f"circuit_tick_{bank}_{row}")

    # ========== COPPER / BRASS BUS AND NEUTRAL / EARTH BARS ==========
    bus_z = FIELD_CZ + well_h / 2 + 0.010  # 0.430
    neutral_z = FIELD_CZ - well_h / 2 - 0.025  # 0.155
    earth_z = neutral_z - 0.030  # 0.125

    _box(enclosure, (0.300, 0.008, 0.016), (0.000, -0.081, bus_z), copper, "copper_bus_bar")
    _box(enclosure, (0.275, 0.008, 0.014), (0.000, -0.081, neutral_z), brass, "neutral_bar")
    _box(enclosure, (0.275, 0.008, 0.014), (0.000, -0.081, earth_z), brass, "earth_bar")

    standoff_xs = [-0.100, -0.050, 0.000, 0.050, 0.100]
    for bar_z, stem in [(bus_z, "bus"), (neutral_z, "neutral"), (earth_z, "earth")]:
        for i, x in enumerate(standoff_xs):
            _box(enclosure, (0.015, 0.010, 0.020), (x, -0.073, bar_z), dark_paint, f"{stem}_standoff_{i}")
            _cylinder(enclosure, 0.0045, 0.003, (x, -0.086, bar_z), galvanized, f"{stem}_screw_{i}", rpy=(math.pi / 2, 0.0, 0.0))

    # ========== LABELS ==========
    label_cz = 0.070
    _box(enclosure, (0.220, 0.003, 0.040), (-0.030, -0.060, label_cz + 0.010), label_white, "legend_label")
    _box(enclosure, (0.180, 0.004, 0.012), (-0.010, -0.0625, label_cz + 0.022), label_blue, "blue_directory_strip")
    _box(enclosure, (0.220, 0.003, 0.042), (-0.030, -0.062, label_cz - 0.025), label_white, "danger_label")
    _box(enclosure, (0.220, 0.004, 0.012), (-0.030, -0.064, label_cz - 0.006), label_orange, "orange_warning_bar")
    _box(enclosure, (0.220, 0.003, 0.032), (-0.030, -0.068, label_cz - 0.045), label_white, "arcflash_label")
    _box(enclosure, (0.220, 0.004, 0.009), (-0.030, -0.0625, label_cz - 0.036), label_red, "red_warning_bar")
    for i, x in enumerate([-0.125, -0.060, 0.005, 0.070]):
        _box(enclosure, (0.045, 0.004, 0.003), (x, -0.065, label_cz - 0.032), dark_paint, f"label_print_{i}")
        _box(enclosure, (0.007, 0.004, 0.007), (x - 0.012, -0.065, label_cz - 0.012), label_yellow, f"warning_icon_{i}")

    # ========== KNOCKOUTS, CABLE GLANDS, CONDUITS ==========
    for i, x in enumerate([-0.180, -0.060, 0.060, 0.180]):
        _cylinder(enclosure, 0.020, 0.006, (x, -0.020, ENC_H + 0.002), galvanized, f"top_knockout_{i}")
    for i, x in enumerate([-0.135, 0.000, 0.135]):
        _cylinder(enclosure, 0.018, 0.120, (x, 0.020, ENC_H + 0.055), galvanized, f"top_conduit_{i}")
        _cylinder(enclosure, 0.024, 0.018, (x, 0.020, ENC_H), rubber, f"top_gland_{i}")
    for i, x in enumerate([-0.120, 0.120]):
        _cylinder(enclosure, 0.019, 0.055, (x, 0.015, -0.015), rubber, f"bottom_cable_{i}")
        _cylinder(enclosure, 0.025, 0.018, (x, 0.015, 0.025), galvanized, f"bottom_gland_{i}")

    # ========== WIRING CHANNELS ==========
    channel_h = ENC_H - 0.250  # 0.300
    _box(enclosure, (0.034, 0.020, channel_h), (-0.172, -0.068, ENC_CZ), dark_paint, "left_wiring_channel")
    _box(enclosure, (0.034, 0.020, channel_h), (0.172, -0.068, ENC_CZ), dark_paint, "right_wiring_channel")
    for i, z in enumerate([ENC_CZ - 0.090, ENC_CZ, ENC_CZ + 0.090]):
        _box(enclosure, (0.026, 0.007, 0.006), (-0.172, -0.080, z), painted, f"left_channel_tooth_{i}")
        _box(enclosure, (0.026, 0.007, 0.006), (0.172, -0.080, z), painted, f"right_channel_tooth_{i}")

    # ========== ROUTED INSULATED WIRES ==========
    wire_specs = [
        ("red_feeder_wire", wire_red, [(-0.135, 0.020, ENC_H + 0.002), (-0.135, -0.060, ENC_H - 0.070), (-0.110, -0.088, bus_z)]),
        ("blue_neutral_wire", wire_blue, [(0.000, 0.020, ENC_H + 0.002), (0.140, -0.056, ENC_CZ + 0.050), (0.120, -0.088, neutral_z)]),
        ("green_earth_wire", wire_green, [(0.135, 0.020, ENC_H + 0.002), (0.172, -0.065, ENC_CZ), (0.105, -0.088, earth_z)]),
        ("bottom_load_wire", rubber, [(-0.120, 0.015, 0.020), (-0.160, -0.065, 0.100), (-0.085, -0.094, breaker_z0)]),
    ]
    for name, mat, points in wire_specs:
        enclosure.visual(mesh_from_geometry(_wire_mesh(points), name), material=mat, name=name)

    # ========== HINGE HARDWARE (stationary on enclosure) ==========
    hinge_pin_h = ENC_H - 0.200  # 0.350
    _cylinder(enclosure, 0.006, hinge_pin_h, (0.315, -0.096, ENC_CZ), galvanized, "hinge_pin")
    _cylinder(enclosure, 0.011, 0.080, (0.315, -0.096, ENC_CZ), galvanized, "center_hinge_collar")
    # Spine shortened to 0.130 to stay between the two barrel z-ranges (no overlap)
    # but wide enough in x to contact the hinge pin cylinder surface
    _box(enclosure, (0.014, 0.050, 0.130), (0.303, -0.075, ENC_CZ), painted, "hinge_jamb_spine")
    for i, zc in enumerate([-0.120, 0.120]):
        _box(enclosure, (0.042, 0.026, 0.080), (0.304, -0.061, ENC_CZ + zc), painted, f"frame_hinge_bridge_{i}")

    # ========== FRONT DOOR (authored in visible open pose) ==========
    door = model.part("front_door", meta={"role": "hinged front cover"})

    _door_box(door, (DOOR_W - 0.028, 0.018, DOOR_H), (-DOOR_W / 2.0 - 0.014, 0.0, 0.000), painted, "door_leaf")
    _door_box(door, (DOOR_W - 0.035, 0.010, 0.018), (-DOOR_W / 2.0, -0.015, DOOR_H / 2.0 - 0.030), painted, "door_top_return")
    _door_box(door, (DOOR_W - 0.035, 0.010, 0.018), (-DOOR_W / 2.0, -0.015, -(DOOR_H / 2.0 - 0.030)), painted, "door_bottom_return")
    _door_box(door, (0.018, 0.010, DOOR_H - 0.040), (-0.018, -0.015, 0.000), painted, "door_hinge_return")
    _door_box(door, (0.018, 0.010, DOOR_H - 0.040), (-DOOR_W + 0.018, -0.015, 0.000), painted, "door_latch_return")

    # Labels, inspection pouch, latch/lock and handle hardware on the moving door
    _door_box(door, (0.110, 0.004, 0.045), (-0.145, -0.011, 0.080), label_white, "door_nameplate")
    _door_box(door, (0.080, 0.005, 0.014), (-0.145, -0.012, 0.068), dark_paint, "door_barcode")
    _door_box(door, (0.115, 0.004, 0.055), (-0.145, -0.011, 0.140), label_white, "door_schedule_label")
    for i, z in enumerate([0.127, 0.140, 0.153]):
        _door_box(door, (0.075, 0.005, 0.003), (-0.145, -0.014, z), dark_paint, f"schedule_line_{i}")
    _door_box(door, (0.105, 0.006, 0.120), (-0.205, -0.012, -0.030), transparent, "inspection_pouch")
    _door_box(door, (0.040, 0.020, 0.120), (-0.032, -0.019, -0.040), galvanized, "lock_plate")
    _door_box(door, (0.028, 0.030, 0.030), (-0.032, -0.039, 0.000), galvanized, "round_lock")
    _door_box(door, (0.026, 0.020, 0.110), (-0.025, -0.039, -0.100), galvanized, "swing_latch")
    _door_box(door, (0.052, 0.018, 0.032), (-0.050, -0.039, -0.150), transparent, "clear_tag_bag")
    _door_box(door, (0.022, 0.018, 0.075), (-0.030, -0.039, -0.130), transparent, "tag_bag_fold")
    for i, z in enumerate([-0.080, -0.040, 0.000]):
        _door_box(door, (0.014, 0.006, 0.014), (-0.032, -0.030, z), galvanized, f"lock_screw_{i}")

    # Moving hinge leaves and knuckles stay with the door leaf
    for i, zc in enumerate([-0.120, 0.120]):
        _door_box(door, (0.026, 0.016, 0.100), (-0.025, 0.002, zc), painted, f"hinge_leaf_{i}")
        _cylinder(door, 0.012, 0.100, (0.0, 0.0, zc), galvanized, f"hinge_barrel_{i}")
        _door_box(door, (0.010, 0.006, 0.040), (-0.028, -0.010, zc), galvanized, f"hinge_screw_{i}_0")
        _door_box(door, (0.010, 0.006, 0.040), (-0.028, -0.010, zc + 0.045), galvanized, f"hinge_screw_{i}_1")

    door_hinge = model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=enclosure,
        child=door,
        origin=Origin(xyz=(0.315, -0.096, ENC_CZ)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=-DOOR_OPEN_ANGLE, upper=0.45),
        meta={"positive_motion": "front cover swings farther open around the right-side hinge"},
    )

    # ========== BREAKER TOGGLES (real articulated controls among the 8 ways) ==========
    # Toggle 0 at bank 0 row 2 (z=0.320), toggle 1 at bank 1 row 1 (z=0.280)
    toggle_specs = [
        (BANKS_X[0], FIELD_CZ + ROW_SPACING / 2.0),  # (-0.085, 0.320)
        (BANKS_X[1], FIELD_CZ - ROW_SPACING / 2.0),  # (0.085, 0.280)
    ]
    for idx, (x, z) in enumerate(toggle_specs):
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

    # --- Identity: small class is Distribution board panel ---
    ctx.check(
        "small class is distribution board panel",
        object_model.name == "distribution_board_panel"
        and object_model.meta.get("small_class") == "Distribution board panel",
        details=f"name={object_model.name}, meta={object_model.meta}",
    )

    # --- Required enclosure visuals ---
    enc_visuals = {v.name for v in enclosure.visuals}
    for required in [
        "back_pan",
        "recessed_deadfront",
        "breaker_well_0",
        "breaker_well_1",
        "copper_bus_bar",
        "neutral_bar",
        "earth_bar",
        "top_gland_0",
        "bottom_gland_0",
        "red_feeder_wire",
        "legend_label",
    ]:
        ctx.check(f"enclosure has {required}", required in enc_visuals,
                  details=f"available={sorted(enc_visuals)[:12]}")

    # --- Required door visual ---
    door_visuals = {v.name for v in door.visuals}
    ctx.check("door has door_leaf", "door_leaf" in door_visuals,
              details=f"available={sorted(door_visuals)[:12]}")

    # --- 8-way breaker count (TARGET axis) ---
    breaker_count = sum(1 for name in enc_visuals if name.startswith("breaker_") and "well" not in name and "rail" not in name)
    ctx.check(
        "8-way compact board has exactly 8 breaker modules (2 banks x 4 rows)",
        breaker_count == 8,
        details=f"breaker_count={breaker_count}",
    )

    # --- Terminal and panel screws ---
    terminal_screw_count = sum(1 for name in enc_visuals if "_screw_" in name)
    ctx.check("has terminal and panel screws", terminal_screw_count >= 15,
              details=f"screw_count={terminal_screw_count}")

    # --- Non-fixed mechanisms (door hinge + 2 toggle pivots = 3 revolute) ---
    revolute_joints = [j for j in object_model.articulations if str(j.articulation_type).upper().endswith("REVOLUTE")]
    ctx.check(
        "has non-fixed mechanisms (door hinge + breaker toggles)",
        len(revolute_joints) >= 3,
        details=f"articulations={[j.name for j in object_model.articulations]}",
    )

    # --- Both toggle parts exist ---
    toggle_1 = object_model.get_part("toggle_1")
    ctx.check("toggle_0 part exists", toggle_0 is not None, details="toggle_0 missing")
    ctx.check("toggle_1 part exists", toggle_1 is not None, details="toggle_1 missing")

    # --- Hinge pin captured in door barrels ---
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
            min_overlap=0.060,
            name=f"{barrel} overlaps hinge pin along barrel length",
        )

    # --- Door gap at rest (open) pose ---
    ctx.expect_gap(
        enclosure,
        door,
        axis="y",
        positive_elem="front_right_rail",
        negative_elem="door_leaf",
        min_gap=0.005,
        max_gap=0.050,
        name="open door clears front rail",
    )

    # --- Closed pose: coverage and clearance ---
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

    # --- Breaker toggle articulation (TARGET: real toggles among 8 ways) ---
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

    return ctx.report()


object_model = build_object_model()
