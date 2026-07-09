from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_W = 0.064
BODY_H = 0.154
BODY_T = 0.016
FRONT_Z = BODY_T / 2.0


def _rounded_box(width: float, height: float, thickness: float, radius: float) -> cq.Workplane:
    """Rounded rectangular consumer-plastic slab, centered on the origin."""
    body = cq.Workplane("XY").box(width, height, thickness)
    return body.edges("|Z").fillet(radius).edges(">Z").fillet(0.0012).edges("<Z").fillet(0.0012)


def _rounded_plate(width: float, height: float, thickness: float, radius: float) -> cq.Workplane:
    # Use a centered box then lift it so callers can reason about a z=0 mounting
    # face.  Filleting the vertical edges gives a rounded-rectangle footprint.
    plate = cq.Workplane("XY").box(width, height, thickness).edges("|Z").fillet(radius)
    return plate.translate((0.0, 0.0, thickness / 2.0))


def _annular_sector(
    inner_radius: float,
    outer_radius: float,
    start_deg: float,
    end_deg: float,
    thickness: float,
    *,
    steps: int = 20,
) -> cq.Workplane:
    """Extruded annular sector for one directional-pad rocker button."""
    outer_pts = []
    inner_pts = []
    for i in range(steps + 1):
        a = math.radians(start_deg + (end_deg - start_deg) * i / steps)
        outer_pts.append((outer_radius * math.cos(a), outer_radius * math.sin(a)))
    for i in range(steps, -1, -1):
        a = math.radians(start_deg + (end_deg - start_deg) * i / steps)
        inner_pts.append((inner_radius * math.cos(a), inner_radius * math.sin(a)))
    return cq.Workplane("XY").polyline(outer_pts + inner_pts).close().extrude(thickness)


def _ink_bar(width: float, height: float, x: float, y: float, z: float, thickness: float = 0.00018) -> cq.Workplane:
    return cq.Workplane("XY").box(width, height, thickness).translate((x, y, z + thickness / 2.0))


def _add_lcd_ink(body, geometry: cq.Workplane, name: str, material: str) -> None:
    body.visual(
        mesh_from_cadquery(geometry, name, tolerance=0.00025, angular_tolerance=0.08),
        material=material,
        name=name,
    )


def _add_bar(
    part,
    x: float,
    y: float,
    z: float,
    width: float,
    height: float,
    name: str,
    *,
    material: str,
    yaw: float = 0.0,
    thickness: float = 0.00022,
) -> None:
    part.visual(
        Box((width, height, thickness)),
        origin=Origin(xyz=(x, y, z), rpy=(0.0, 0.0, yaw)),
        material=material,
        name=name,
    )


def _add_plus(part, x: float, y: float, z: float, size: float, prefix: str, material: str) -> None:
    stroke = size * 0.18
    _add_bar(part, x, y, z, size, stroke, f"{prefix}_h", material=material)
    _add_bar(part, x, y, z, stroke, size, f"{prefix}_v", material=material)


def _add_minus(part, x: float, y: float, z: float, size: float, prefix: str, material: str) -> None:
    _add_bar(part, x, y, z, size, size * 0.18, f"{prefix}_h", material=material)


def _add_chevron(part, x: float, y: float, z: float, size: float, prefix: str, material: str, *, right: bool) -> None:
    yaw = math.radians(35.0)
    if right:
        _add_bar(part, x, y + size * 0.16, z, size * 0.72, size * 0.15, f"{prefix}_a", material=material, yaw=-yaw)
        _add_bar(part, x, y - size * 0.16, z, size * 0.72, size * 0.15, f"{prefix}_b", material=material, yaw=yaw)
    else:
        _add_bar(part, x, y + size * 0.16, z, size * 0.72, size * 0.15, f"{prefix}_a", material=material, yaw=yaw)
        _add_bar(part, x, y - size * 0.16, z, size * 0.72, size * 0.15, f"{prefix}_b", material=material, yaw=-yaw)


def _add_digit(part, digit: str, x: float, y: float, z: float, height: float, prefix: str, material: str) -> None:
    width = height * 0.52
    stroke = max(height * 0.085, 0.00028)
    segments = {
        "0": "abcfed",
        "1": "bc",
        "2": "abged",
        "3": "abgcd",
        "4": "fgbc",
        "5": "afgcd",
        "6": "afgecd",
        "7": "abc",
        "8": "abcdefg",
        "9": "abfgcd",
    }[digit]
    specs = {
        "a": (0.0, height * 0.50, width, stroke, 0.0),
        "g": (0.0, 0.0, width, stroke, 0.0),
        "d": (0.0, -height * 0.50, width, stroke, 0.0),
        "b": (width * 0.50, height * 0.25, stroke, height * 0.45, 0.0),
        "c": (width * 0.50, -height * 0.25, stroke, height * 0.45, 0.0),
        "f": (-width * 0.50, height * 0.25, stroke, height * 0.45, 0.0),
        "e": (-width * 0.50, -height * 0.25, stroke, height * 0.45, 0.0),
    }
    for seg in segments:
        dx, dy, w, h, yaw = specs[seg]
        _add_bar(part, x + dx, y + dy, z, w, h, f"{prefix}_{seg}", material=material, yaw=yaw, thickness=0.00018)


def _add_colon(part, x: float, y: float, z: float, prefix: str, material: str) -> None:
    part.visual(Cylinder(radius=0.00045, length=0.00018), origin=Origin(xyz=(x, y + 0.0014, z)), material=material, name=f"{prefix}_top")
    part.visual(Cylinder(radius=0.00045, length=0.00018), origin=Origin(xyz=(x, y - 0.0014, z)), material=material, name=f"{prefix}_bottom")


def _add_letter(part, letter: str, x: float, y: float, z: float, height: float, prefix: str, material: str) -> None:
    width = height * 0.55
    stroke = max(height * 0.10, 0.00030)
    left = -width * 0.5
    right = width * 0.5
    top = height * 0.5
    mid = 0.0
    bot = -height * 0.5
    if letter == "M":
        _add_bar(part, x + left, y, z, stroke, height, f"{prefix}_l", material=material)
        _add_bar(part, x + right, y, z, stroke, height, f"{prefix}_r", material=material)
        _add_bar(part, x - width * 0.18, y + height * 0.18, z, stroke, height * 0.55, f"{prefix}_ml", material=material, yaw=math.radians(-25))
        _add_bar(part, x + width * 0.18, y + height * 0.18, z, stroke, height * 0.55, f"{prefix}_mr", material=material, yaw=math.radians(25))
    elif letter == "O":
        _add_bar(part, x, y + top, z, width, stroke, f"{prefix}_t", material=material)
        _add_bar(part, x, y + bot, z, width, stroke, f"{prefix}_b", material=material)
        _add_bar(part, x + left, y, z, stroke, height, f"{prefix}_l", material=material)
        _add_bar(part, x + right, y, z, stroke, height, f"{prefix}_r", material=material)
    elif letter == "D":
        _add_bar(part, x + left, y, z, stroke, height, f"{prefix}_l", material=material)
        _add_bar(part, x, y + top, z, width, stroke, f"{prefix}_t", material=material)
        _add_bar(part, x, y + bot, z, width, stroke, f"{prefix}_b", material=material)
        _add_bar(part, x + right, y, z, stroke, height * 0.82, f"{prefix}_r", material=material)
    elif letter == "E":
        _add_bar(part, x + left, y, z, stroke, height, f"{prefix}_l", material=material)
        _add_bar(part, x, y + top, z, width, stroke, f"{prefix}_t", material=material)
        _add_bar(part, x - width * 0.05, y + mid, z, width * 0.86, stroke, f"{prefix}_m", material=material)
        _add_bar(part, x, y + bot, z, width, stroke, f"{prefix}_b", material=material)
    elif letter == "T":
        _add_bar(part, x, y + top, z, width, stroke, f"{prefix}_t", material=material)
        _add_bar(part, x, y, z, stroke, height, f"{prefix}_v", material=material)
    elif letter == "U":
        _add_bar(part, x + left, y, z, stroke, height, f"{prefix}_l", material=material)
        _add_bar(part, x + right, y, z, stroke, height, f"{prefix}_r", material=material)
        _add_bar(part, x, y + bot, z, width, stroke, f"{prefix}_b", material=material)
    elif letter == "N":
        _add_bar(part, x + left, y, z, stroke, height, f"{prefix}_l", material=material)
        _add_bar(part, x + right, y, z, stroke, height, f"{prefix}_r", material=material)
        _add_bar(part, x, y, z, stroke, height * 1.1, f"{prefix}_diag", material=material, yaw=math.radians(26))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fireplace_thermostat_remote")

    model.material("soft_black", rgba=(0.005, 0.006, 0.005, 1.0))
    model.material("charcoal", rgba=(0.018, 0.020, 0.018, 1.0))
    model.material("button_grey", rgba=(0.62, 0.65, 0.62, 1.0))
    model.material("dark_ink", rgba=(0.015, 0.018, 0.017, 1.0))
    model.material("lcd_orange", rgba=(1.0, 0.29, 0.075, 1.0))
    model.material("lcd_ink", rgba=(0.30, 0.09, 0.055, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_rounded_box(BODY_W, BODY_H, BODY_T, 0.0105), "body_shell", tolerance=0.00045),
        material="soft_black",
        name="body_shell",
    )

    # Inset, backlit thermostat LCD in a shallow black bezel.
    lcd_y = 0.041
    body.visual(
        mesh_from_cadquery(
            _rounded_plate(0.052, 0.048, 0.0008, 0.0020).translate((0.0, lcd_y, FRONT_Z - 0.00015)),
            "lcd_bezel",
            tolerance=0.00025,
        ),
        material="charcoal",
        name="lcd_bezel",
    )
    body.visual(
        mesh_from_cadquery(
            _rounded_plate(0.0475, 0.0435, 0.00075, 0.0016).translate((0.0, lcd_y, FRONT_Z + 0.00025)),
            "lcd_glass",
            tolerance=0.00025,
        ),
        material="lcd_orange",
        name="lcd_glass",
    )

    ink_z = FRONT_Z + 0.00095
    ink_center_z = ink_z + 0.00009
    # Top row: small 14:16 time and TUE day strokes.
    time_x0 = -0.0070
    for i, ch in enumerate(("1", "4")):
        _add_digit(body, ch, time_x0 + i * 0.0032, lcd_y + 0.016, ink_center_z, 0.0033, f"time_{i}", "lcd_ink")
    _add_colon(body, time_x0 + 0.0064, lcd_y + 0.016, ink_center_z, "time_colon", "lcd_ink")
    for i, ch in enumerate(("1", "6")):
        _add_digit(body, ch, time_x0 + 0.0096 + i * 0.0032, lcd_y + 0.016, ink_center_z, 0.0033, f"time_{i+2}", "lcd_ink")
    for i, ch in enumerate("TUE"):
        _add_letter(body, ch, -0.014 + i * 0.0038, lcd_y + 0.0095, ink_center_z, 0.0032, f"weekday_{ch}_{i}", "lcd_ink")

    # Large central thermostat temperature reading.
    _add_digit(body, "2", -0.0040, lcd_y - 0.002, ink_center_z, 0.014, "temp_2", "lcd_ink")
    _add_digit(body, "0", 0.0055, lcd_y - 0.002, ink_center_z, 0.014, "temp_0", "lcd_ink")
    body.visual(
        Cylinder(radius=0.0009, length=0.00018),
        origin=Origin(xyz=(0.0120, lcd_y + 0.0035, ink_center_z)),
        material="lcd_ink",
        name="degree_dot",
    )
    _add_letter(body, "O", 0.0170, lcd_y - 0.010, ink_center_z, 0.0029, "on_o", "lcd_ink")
    _add_letter(body, "N", 0.0205, lcd_y - 0.010, ink_center_z, 0.0029, "on_n", "lcd_ink")

    # LCD mode/status icons: flame, thermostat, clock, alarm/sun, and battery.
    _add_bar(
        body,
        -0.0190,
        lcd_y - 0.0160,
        ink_center_z,
        0.0033,
        0.0070,
        "flame_icon_a",
        material="lcd_ink",
        yaw=math.radians(-28.0),
        thickness=0.00018,
    )
    _add_bar(
        body,
        -0.0165,
        lcd_y - 0.0170,
        ink_center_z,
        0.0023,
        0.0052,
        "flame_icon_b",
        material="lcd_ink",
        yaw=math.radians(25.0),
        thickness=0.00018,
    )
    body.visual(
        Box((0.009, 0.011, 0.00018)),
        origin=Origin(xyz=(-0.0065, lcd_y - 0.017, ink_z + 0.00009)),
        material="lcd_ink",
        name="thermo_icon_box",
    )
    body.visual(
        Cylinder(radius=0.0011, length=0.00018),
        origin=Origin(xyz=(-0.0065, lcd_y - 0.0185, ink_z + 0.00009)),
        material="lcd_orange",
        name="thermo_icon_bulb_cut",
    )
    body.visual(
        Cylinder(radius=0.0031, length=0.00018),
        origin=Origin(xyz=(0.0085, lcd_y - 0.017, ink_center_z)),
        material="lcd_ink",
        name="clock_icon_outer",
    )
    body.visual(
        Cylinder(radius=0.00235, length=0.00020),
        origin=Origin(xyz=(0.0085, lcd_y - 0.017, ink_center_z + 0.00003)),
        material="lcd_orange",
        name="clock_icon_inner",
    )
    _add_bar(body, 0.0085, lcd_y - 0.0158, ink_center_z + 0.00006, 0.00035, 0.0022, "clock_hand_min", material="lcd_ink", thickness=0.00018)
    _add_bar(body, 0.0094, lcd_y - 0.0170, ink_center_z + 0.00006, 0.0021, 0.00035, "clock_hand_hour", material="lcd_ink", thickness=0.00018)
    body.visual(
        Cylinder(radius=0.0022, length=0.00018),
        origin=Origin(xyz=(0.0200, lcd_y - 0.017, ink_center_z)),
        material="lcd_ink",
        name="sun_icon_center",
    )
    for i, yaw in enumerate((0.0, math.pi / 4.0, math.pi / 2.0, 3.0 * math.pi / 4.0)):
        _add_bar(body, 0.0200, lcd_y - 0.017, ink_center_z + 0.00005, 0.0070, 0.00032, f"sun_ray_{i}", material="lcd_ink", yaw=yaw, thickness=0.00018)
    body.visual(
        Box((0.0065, 0.0030, 0.00018)),
        origin=Origin(xyz=(0.0215, lcd_y + 0.0170, ink_z + 0.00009)),
        material="lcd_ink",
        name="battery_outline",
    )
    for i, x in enumerate((0.0196, 0.0212, 0.0228)):
        body.visual(
            Box((0.0007, 0.0020, 0.00020)),
            origin=Origin(xyz=(x, lcd_y + 0.0170, ink_z + 0.00010)),
            material="lcd_orange",
            name=f"battery_gap_{i}",
        )

    # Recessed circular well beneath the directional pad.
    body.visual(
        Cylinder(radius=0.0318, length=0.0005),
        origin=Origin(xyz=(0.0, -0.038, FRONT_Z + 0.00025)),
        material="charcoal",
        name="dpad_well",
    )

    # Hinge boss/knuckle for the flip-down cover at the top of the dpad cluster.
    # A visible mounting bracket protrudes from the body front face and carries
    # the cylindrical pivot barrel at the top.
    hinge_y = -0.006
    hinge_z = FRONT_Z + 0.006
    knuckle_r = 0.0022
    knuckle_len = 0.024
    boss_base_z = FRONT_Z - 0.0002  # slight embed into body shell for connectivity
    boss_top_z = hinge_z - knuckle_r + 0.001
    boss_h = boss_top_z - boss_base_z
    boss_w = 0.018  # narrower than button spacing to avoid collision
    boss_d = 0.005
    hinge_boss = (
        cq.Workplane("XY")
        .box(boss_w, boss_d, boss_h)
        .edges("|Z")
        .fillet(0.001)
        .translate((0.0, hinge_y, boss_base_z + boss_h / 2.0))
    )
    hinge_barrel = (
        cq.Workplane("YZ")
        .circle(knuckle_r)
        .extrude(knuckle_len)
        .translate((-knuckle_len / 2.0, hinge_y, hinge_z))
    )
    body.visual(
        mesh_from_cadquery(
            hinge_boss.union(hinge_barrel),
            "hinge_knuckle",
            tolerance=0.00025,
        ),
        material="charcoal",
        name="hinge_knuckle",
    )

    press_limits = MotionLimits(effort=1.0, velocity=0.05, lower=0.0, upper=0.0018)

    def add_round_button(part_name: str, joint_name: str, x: float, y: float, label: str) -> None:
        part = model.part(part_name)
        part.visual(
            Cylinder(radius=0.0079, length=0.0030),
            origin=Origin(xyz=(0.0, 0.0, 0.0015)),
            material="button_grey",
            name="cap",
        )
        symbol_z = 0.00304
        if label == "power":
            _add_bar(part, 0.0, 0.0016, symbol_z, 0.00045, 0.0040, "power_stem", material="dark_ink")
            for i, yaw in enumerate((0.0, math.pi / 4.0, -math.pi / 4.0, math.pi / 2.0)):
                _add_bar(part, 0.0, -0.0006, symbol_z, 0.0054, 0.00042, f"power_ring_{i}", material="dark_ink", yaw=yaw)
        else:
            part.visual(
                Cylinder(radius=0.00115, length=0.00022),
                origin=Origin(xyz=(0.0, 0.0, symbol_z)),
                material="dark_ink",
                name="light_center",
            )
            for i, yaw in enumerate((0.0, math.pi / 4.0, math.pi / 2.0, 3.0 * math.pi / 4.0)):
                _add_bar(part, 0.0, 0.0, symbol_z, 0.0060, 0.00035, f"light_ray_{i}", material="dark_ink", yaw=yaw)
        model.articulation(
            joint_name,
            ArticulationType.PRISMATIC,
            parent=body,
            child=part,
            origin=Origin(xyz=(x, y, FRONT_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=press_limits,
        )

    add_round_button("power_button", "body_to_power_button", -0.018, -0.004, "power")
    add_round_button("light_button", "body_to_light_button", 0.018, -0.004, "light")

    dpad_y = -0.040
    dpad_base_z = FRONT_Z + 0.0005
    sectors = (
        ("dpad_up", "body_to_dpad_up", 50.0, 130.0, "+"),
        ("dpad_down", "body_to_dpad_down", 230.0, 310.0, "-"),
        ("dpad_left", "body_to_dpad_left", 140.0, 220.0, "<"),
        ("dpad_right", "body_to_dpad_right", -40.0, 40.0, ">"),
    )
    for part_name, joint_name, start, end, label in sectors:
        part = model.part(part_name)
        part.visual(
            mesh_from_cadquery(
                _annular_sector(0.0176, 0.0296, start, end, 0.0030),
                f"{part_name}_cap",
                tolerance=0.00035,
                angular_tolerance=0.06,
            ),
            material="button_grey",
            name="cap",
        )
        mid = math.radians((start + end) / 2.0)
        label_r = 0.0236
        label_x = label_r * math.cos(mid)
        label_y = label_r * math.sin(mid)
        label_z = 0.00305
        if label == "+":
            _add_plus(part, label_x, label_y, label_z, 0.0058, "plus_symbol", "dark_ink")
        elif label == "-":
            _add_minus(part, label_x, label_y, label_z, 0.0064, "minus_symbol", "dark_ink")
        elif label == "<":
            _add_chevron(part, label_x, label_y, label_z, 0.0068, "left_arrow", "dark_ink", right=False)
        elif label == ">":
            _add_chevron(part, label_x, label_y, label_z, 0.0068, "right_arrow", "dark_ink", right=True)
        model.articulation(
            joint_name,
            ArticulationType.PRISMATIC,
            parent=body,
            child=part,
            origin=Origin(xyz=(0.0, dpad_y, dpad_base_z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=press_limits,
        )

    mode = model.part("mode_button")
    mode.visual(
        Cylinder(radius=0.0144, length=0.0032),
        origin=Origin(xyz=(0.0, 0.0, 0.0016)),
        material="button_grey",
        name="cap",
    )
    mode_label_z = 0.00325
    for i, ch in enumerate("MODE"):
        _add_letter(mode, ch, -0.0069 + i * 0.0046, 0.0, mode_label_z, 0.0042, f"mode_{ch}_{i}", "dark_ink")
    model.articulation(
        "body_to_mode_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=mode,
        origin=Origin(xyz=(0.0, dpad_y, dpad_base_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=press_limits,
    )

    # Flip-down cover over the dpad cluster, hinged at the top edge.
    cover_w = 0.054
    cover_h = 0.062
    cover_t = 0.0018
    cover_r = 0.006
    cover = model.part("cover")
    cover_plate = (
        cq.Workplane("XY")
        .box(cover_w, cover_h, cover_t)
        .edges("|Z")
        .fillet(cover_r)
        .translate((0.0, -cover_h / 2.0, -cover_t / 2.0))
    )
    cover.visual(
        mesh_from_cadquery(cover_plate, "cover_plate", tolerance=0.00035),
        material="soft_black",
        name="cover_plate",
    )

    model.articulation(
        "body_to_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.5, lower=0.0, upper=1.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lcd = ctx.part_element_world_aabb(body, elem="lcd_glass")
    shell = ctx.part_element_world_aabb(body, elem="body_shell")
    ctx.check(
        "orange lcd sits in upper half of black handheld body",
        lcd is not None
        and shell is not None
        and lcd[0][1] > 0.015
        and (lcd[1][0] - lcd[0][0]) < (shell[1][0] - shell[0][0]) * 0.9,
        details=f"lcd={lcd}, shell={shell}",
    )

    control_names = (
        "power_button",
        "light_button",
        "dpad_up",
        "dpad_down",
        "dpad_left",
        "dpad_right",
        "mode_button",
    )
    ctx.check(
        "all visible user controls are separate pressable parts",
        all(object_model.get_part(name) is not None for name in control_names)
        and len(object_model.articulations) == len(control_names) + 1,  # +1 for body_to_cover hinge
        details=f"controls={control_names}, articulations={len(object_model.articulations)}",
    )

    for part_name in ("power_button", "light_button"):
        ctx.expect_gap(
            part_name,
            body,
            axis="z",
            max_gap=0.0002,
            max_penetration=0.0,
            negative_elem="body_shell",
            name=f"{part_name} cap rests on remote face",
        )

    for part_name in ("dpad_up", "dpad_down", "dpad_left", "dpad_right", "mode_button"):
        ctx.expect_gap(
            part_name,
            body,
            axis="z",
            max_gap=0.0002,
            max_penetration=0.0,
            negative_elem="dpad_well",
            name=f"{part_name} rests on recessed dpad well",
        )

    def cap_center(part_name: str):
        aabb = ctx.part_element_world_aabb(object_model.get_part(part_name), elem="cap")
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) / 2.0 for i in range(3))

    positions = {name: cap_center(name) for name in ("dpad_up", "dpad_down", "dpad_left", "dpad_right", "mode_button")}
    ctx.check(
        "directional pad surrounds the central MODE button",
        all(pos is not None for pos in positions.values())
        and positions["dpad_up"][1] > positions["mode_button"][1] + 0.014
        and positions["dpad_down"][1] < positions["mode_button"][1] - 0.014
        and positions["dpad_right"][0] > positions["mode_button"][0] + 0.014
        and positions["dpad_left"][0] < positions["mode_button"][0] - 0.014,
        details=f"positions={positions}",
    )

    power_joint = object_model.get_articulation("body_to_power_button")
    rest_pos = ctx.part_world_position("power_button")
    with ctx.pose({power_joint: 0.0018}):
        pressed_pos = ctx.part_world_position("power_button")
    ctx.check(
        "button prismatic travel depresses into the remote face",
        rest_pos is not None and pressed_pos is not None and pressed_pos[2] < rest_pos[2] - 0.0010,
        details=f"rest={rest_pos}, pressed={pressed_pos}",
    )

    # Cover hinge tests.
    cover = object_model.get_part("cover")
    cover_hinge = object_model.get_articulation("body_to_cover")
    ctx.check(
        "cover part and body_to_cover hinge exist",
        cover is not None and cover_hinge is not None,
        details="cover or hinge missing",
    )
    ctx.check(
        "hinge knuckle boss is visible on the body at the pivot line",
        ctx.part_element_world_aabb(body, elem="hinge_knuckle") is not None,
        details="hinge_knuckle visual missing from body",
    )

    # Allow the local knuckle/cover nesting at the pivot axis.
    ctx.allow_overlap(
        body,
        cover,
        elem_a="hinge_knuckle",
        elem_b="cover_plate",
        reason="The cover wraps against the hinge knuckle at the pivot axis; small local nesting is intentional.",
    )

    # At rest (closed), the cover lies flat over the dpad cluster.
    with ctx.pose({cover_hinge: 0.0}):
        closed_aabb = ctx.part_world_aabb("cover")
        ctx.expect_gap(
            cover,
            body,
            axis="z",
            min_gap=-0.001,
            max_gap=0.010,
            negative_elem="body_shell",
            name="cover rests flat above the body front face when closed",
        )

    # At open (~100°), the cover bottom edge swings up in Z.
    with ctx.pose({cover_hinge: 1.75}):
        open_aabb = ctx.part_world_aabb("cover")

    ctx.check(
        "cover hinge opens the lid upward away from the dpad cluster",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.010,
        details=f"closed_max_z={closed_aabb[1][2] if closed_aabb else None}, open_max_z={open_aabb[1][2] if open_aabb else None}",
    )

    return ctx.report()


object_model = build_object_model()
