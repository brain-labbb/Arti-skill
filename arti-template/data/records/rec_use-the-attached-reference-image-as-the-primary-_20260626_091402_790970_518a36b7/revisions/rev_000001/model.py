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


BOX_L = 0.180
BOX_W = 0.130
BOX_H = 0.070
WALL = 0.007
BOTTOM = 0.004
COVER_L = 0.198
COVER_D = 0.148
COVER_T = 0.012
HINGE_Y = BOX_W / 2.0 + 0.008
HINGE_Z = BOX_H + 0.010
OPEN_ANGLE = math.radians(105.0)
OPEN_ROLL = -OPEN_ANGLE


def _mat(name: str, rgba: tuple[float, float, float, float]) -> Material:
    return Material(name=name, rgba=rgba)


BLACK = _mat("molded_black_plastic", (0.015, 0.017, 0.017, 1.0))
DARK = _mat("dark_graphite_plastic", (0.055, 0.060, 0.060, 1.0))
RUBBER = _mat("black_rubber_jacket", (0.005, 0.006, 0.005, 1.0))
YELLOW = _mat("yellow_gasket", (1.0, 0.78, 0.02, 1.0))
LIGHT = _mat("pale_terminal_plastic", (0.86, 0.86, 0.82, 1.0))
METAL = _mat("galvanized_screw_metal", (0.62, 0.64, 0.62, 1.0))
BRASS = _mat("brass_terminal_lug", (0.95, 0.66, 0.22, 1.0))
COPPER = _mat("copper_conductor", (0.92, 0.42, 0.18, 1.0))
GREEN = _mat("green_wire_insulation", (0.02, 0.55, 0.14, 1.0))
RED = _mat("red_wire_insulation", (0.80, 0.03, 0.025, 1.0))
BLUE = _mat("blue_wire_insulation", (0.02, 0.13, 0.78, 1.0))
BROWN = _mat("brown_wire_insulation", (0.37, 0.17, 0.06, 1.0))
WHITE = _mat("white_label_plastic", (0.92, 0.90, 0.78, 1.0))
WARNING = _mat("warning_label_yellow", (1.0, 0.86, 0.08, 1.0))


def _rx_xyz(xyz: tuple[float, float, float], angle: float = OPEN_ROLL) -> tuple[float, float, float]:
    x, y, z = xyz
    c = math.cos(angle)
    s = math.sin(angle)
    return (x, y * c - z * s, y * s + z * c)


def _cover_origin(xyz: tuple[float, float, float], *, roll: float = 0.0) -> Origin:
    """Place a cover-attached item in the default open pose around the hinge axis."""
    return Origin(xyz=_rx_xyz(xyz), rpy=(OPEN_ROLL + roll, 0.0, 0.0))


def _axis_origin(
    xyz: tuple[float, float, float],
    *,
    axis: str,
    rpy_extra: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> Origin:
    if axis == "x":
        return Origin(xyz=xyz, rpy=(rpy_extra[0], math.pi / 2.0 + rpy_extra[1], rpy_extra[2]))
    if axis == "y":
        return Origin(xyz=xyz, rpy=(-math.pi / 2.0 + rpy_extra[0], rpy_extra[1], rpy_extra[2]))
    return Origin(xyz=xyz, rpy=rpy_extra)


def _add_box(part, name, size, xyz, material):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_cylinder(part, name, radius, length, xyz, material, *, axis="z"):
    part.visual(Cylinder(radius, length), origin=_axis_origin(xyz, axis=axis), material=material, name=name)


def _add_gland(part, stem: str, side: str, offset: float, z: float) -> None:
    """Cable gland with neck, two compression nuts, thread rings, rubber cable and dark bore."""
    sign = 1.0 if side in ("+x", "+y") else -1.0
    if side.endswith("x"):
        axis = "x"
        x_wall = sign * BOX_L / 2.0
        y = offset
        xyz = lambda d: (x_wall + sign * d, y, z)
    else:
        axis = "y"
        y_wall = sign * BOX_W / 2.0
        x = offset
        xyz = lambda d: (x, y_wall + sign * d, z)

    _add_cylinder(part, f"{stem}_neck", 0.014, 0.032, xyz(0.010), BLACK, axis=axis)
    _add_cylinder(part, f"{stem}_nut", 0.017, 0.020, xyz(0.028), DARK, axis=axis)
    _add_cylinder(part, f"{stem}_cap", 0.015, 0.014, xyz(0.045), BLACK, axis=axis)
    _add_cylinder(part, f"{stem}_cable", 0.0085, 0.060, xyz(0.078), RUBBER, axis=axis)
    for i, d in enumerate((0.018, 0.038, 0.052)):
        _add_cylinder(part, f"{stem}_thread_{i}", 0.0178, 0.003, xyz(d), DARK, axis=axis)
    _add_cylinder(part, f"{stem}_bore", 0.009, 0.002, xyz(0.109), BLACK, axis=axis)


def _add_wire_part(model: ArticulatedObject, name: str, points, material: Material, radius=0.0022):
    wire = model.part(name)
    mesh = mesh_from_geometry(
        tube_from_spline_points(points, radius=radius, samples_per_segment=10, radial_segments=16),
        name,
    )
    wire.visual(mesh, material=material, name="insulated_conductor")
    return wire


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="electrical_junction_box",
        meta={"small_class": "Junction box", "domain": "Electrical_Wiring"},
    )

    enclosure = model.part("enclosure")

    # One molded open-top enclosure: floor, four walls, corner screw bosses and ears.
    _add_box(enclosure, "floor_plate", (BOX_L, BOX_W, BOTTOM), (0.0, 0.0, BOTTOM / 2.0), BLACK)
    _add_box(enclosure, "front_wall", (BOX_L, WALL, BOX_H), (0.0, -BOX_W / 2.0 + WALL / 2.0, BOX_H / 2.0), BLACK)
    _add_box(enclosure, "rear_wall", (BOX_L, WALL, BOX_H), (0.0, BOX_W / 2.0 - WALL / 2.0, BOX_H / 2.0), BLACK)
    _add_box(enclosure, "side_wall_0", (WALL, BOX_W, BOX_H), (-BOX_L / 2.0 + WALL / 2.0, 0.0, BOX_H / 2.0), BLACK)
    _add_box(enclosure, "side_wall_1", (WALL, BOX_W, BOX_H), (BOX_L / 2.0 - WALL / 2.0, 0.0, BOX_H / 2.0), BLACK)

    corner_xy = [
        (-BOX_L / 2.0 + 0.018, -BOX_W / 2.0 + 0.018),
        (BOX_L / 2.0 - 0.018, -BOX_W / 2.0 + 0.018),
        (-BOX_L / 2.0 + 0.018, BOX_W / 2.0 - 0.018),
        (BOX_L / 2.0 - 0.018, BOX_W / 2.0 - 0.018),
    ]
    for i, (x, y) in enumerate(corner_xy):
        _add_cylinder(enclosure, f"screw_boss_{i}", 0.012, BOX_H, (x, y, BOX_H / 2.0), BLACK)
        _add_cylinder(enclosure, f"boss_hole_{i}", 0.0042, 0.002, (x, y, BOX_H + 0.001), DARK)

    # Yellow elastomer gasket lip sitting proud around the open mouth.
    gasket_z = BOX_H + 0.0015
    _add_box(enclosure, "gasket_front", (BOX_L - 0.025, 0.005, 0.003), (0.0, -BOX_W / 2.0 + WALL + 0.003, gasket_z), YELLOW)
    _add_box(enclosure, "gasket_rear", (BOX_L - 0.025, 0.005, 0.003), (0.0, BOX_W / 2.0 - WALL - 0.003, gasket_z), YELLOW)
    _add_box(enclosure, "gasket_side_0", (0.005, BOX_W - 0.025, 0.003), (-BOX_L / 2.0 + WALL + 0.003, 0.0, gasket_z), YELLOW)
    _add_box(enclosure, "gasket_side_1", (0.005, BOX_W - 0.025, 0.003), (BOX_L / 2.0 - WALL - 0.003, 0.0, gasket_z), YELLOW)

    # Mounting ears with screw holes molded into the same enclosure.
    for i, (xsign, y) in enumerate(((-1, -0.043), (-1, 0.043), (1, -0.043), (1, 0.043))):
        x = xsign * (BOX_L / 2.0 + 0.012)
        _add_box(enclosure, f"mount_bridge_{i}", (0.018, 0.014, 0.008), (xsign * (BOX_L / 2.0 + 0.002), y, 0.012), BLACK)
        _add_cylinder(enclosure, f"mount_ear_{i}", 0.013, 0.008, (x, y, 0.012), BLACK)
        _add_cylinder(enclosure, f"mount_hole_{i}", 0.0045, 0.002, (x, y, 0.017), DARK)

    # External cable glands/strain reliefs on multiple sides like the reference image.
    _add_gland(enclosure, "left_gland", "-x", -0.032, 0.037)
    _add_gland(enclosure, "right_gland_0", "+x", -0.032, 0.037)
    _add_gland(enclosure, "right_gland_1", "+x", 0.032, 0.037)
    _add_gland(enclosure, "front_gland_0", "-y", -0.042, 0.036)
    _add_gland(enclosure, "front_gland_1", "-y", 0.042, 0.036)
    _add_gland(enclosure, "rear_gland", "+y", -0.045, 0.040)

    # Rear hinge knuckles carried by the enclosure wall.
    for i, x in enumerate((-0.060, 0.060)):
        _add_box(enclosure, f"hinge_leaf_{i}", (0.050, 0.010, 0.010), (x, HINGE_Y - 0.005, HINGE_Z - 0.005), BLACK)
    for i, x in enumerate((-0.060, 0.060)):
        _add_cylinder(enclosure, f"base_barrel_{i}", 0.007, 0.044, (x, HINGE_Y, HINGE_Z), BLACK, axis="x")
    _add_cylinder(enclosure, "hinge_pin_tip_0", 0.004, 0.006, (-0.085, HINGE_Y, HINGE_Z), METAL, axis="x")
    _add_cylinder(enclosure, "hinge_pin_tip_1", 0.004, 0.006, (0.085, HINGE_Y, HINGE_Z), METAL, axis="x")

    # Small molded/painted labels and warning stripes on the shell.
    _add_box(enclosure, "rating_label", (0.050, 0.0015, 0.018), (-0.025, -BOX_W / 2.0 - 0.0008, 0.050), WHITE)
    for i in range(3):
        _add_box(enclosure, f"label_stripe_{i}", (0.036, 0.0018, 0.002), (-0.025, -BOX_W / 2.0 - 0.002, 0.045 + i * 0.004), DARK)

    # Internal terminal strip and screw terminals.
    terminals = model.part("terminal_strip")
    _add_box(terminals, "terminal_base", (0.106, 0.046, 0.006), (0.0, -0.006, BOTTOM + 0.003), LIGHT)
    for j, y in enumerate((-0.017, 0.006)):
        _add_box(terminals, f"brass_bus_{j}", (0.086, 0.004, 0.003), (0.0, y, BOTTOM + 0.0095), BRASS)
    socket_x = (-0.036, -0.012, 0.012, 0.036)
    socket_y = (-0.018, 0.007)
    idx = 0
    for y in socket_y:
        for x in socket_x:
            _add_cylinder(terminals, f"terminal_socket_{idx}", 0.0072, 0.022, (x, y, BOTTOM + 0.015), LIGHT)
            _add_cylinder(terminals, f"terminal_screw_{idx}", 0.0035, 0.004, (x, y, BOTTOM + 0.024), METAL)
            idx += 1

    # Brass ground lug fastened to the floor, with a visible green screw head.
    ground = model.part("ground_lug")
    _add_box(ground, "lug_plate", (0.026, 0.014, 0.003), (-0.064, 0.036, BOTTOM + 0.0015), BRASS)
    _add_cylinder(ground, "lug_screw", 0.005, 0.003, (-0.064, 0.036, BOTTOM + 0.0045), GREEN)
    _add_box(ground, "earth_symbol_bar", (0.016, 0.002, 0.002), (-0.064, 0.029, BOTTOM + 0.0035), GREEN)

    # Believable bundled wire ends routed from glands into the terminal strip.
    wire_specs = [
        ("wire_red", [(-0.082, -0.032, 0.037), (-0.055, -0.021, 0.050), (-0.036, -0.018, 0.029)], RED),
        ("wire_blue", [(0.082, -0.032, 0.037), (0.058, -0.018, 0.050), (0.036, -0.018, 0.029)], BLUE),
        ("wire_brown", [(-0.042, -0.058, 0.036), (-0.027, -0.038, 0.049), (-0.012, 0.007, 0.029)], BROWN),
        ("wire_green", [(-0.082, 0.030, 0.040), (-0.070, 0.036, 0.036), (-0.064, 0.036, 0.011)], GREEN),
        ("wire_copper", [(0.042, -0.058, 0.036), (0.022, -0.035, 0.048), (0.012, 0.007, 0.029)], COPPER),
    ]
    wires = [_add_wire_part(model, name, points, material) for name, points, material in wire_specs]

    # Hinged/removable cover, authored at the same hinge axis and pre-rotated open.
    cover = model.part("cover")
    cover.visual(
        Box((COVER_L, COVER_D, COVER_T)),
        origin=_cover_origin((0.0, -COVER_D / 2.0 - 0.016, 0.0)),
        material=DARK,
        name="cover_panel",
    )
    # Rounded-looking molded corner pads and retained screw heads.
    for i, (x, y) in enumerate(
        (
            (-COVER_L / 2.0 + 0.016, -COVER_D + 0.016),
            (COVER_L / 2.0 - 0.016, -COVER_D + 0.016),
            (-COVER_L / 2.0 + 0.016, -0.016),
            (COVER_L / 2.0 - 0.016, -0.016),
        )
    ):
        cover.visual(Cylinder(0.011, COVER_T + 0.001), origin=_cover_origin((x, y, 0.0)), material=DARK, name=f"cover_corner_{i}")
        cover.visual(Cylinder(0.005, 0.0025), origin=_cover_origin((x, y, COVER_T / 2.0 + 0.0015)), material=BLACK, name=f"cover_screw_{i}")
        cover.visual(Cylinder(0.0026, 0.0028), origin=_cover_origin((x, y, COVER_T / 2.0 + 0.003), material=METAL) if False else _cover_origin((x, y, COVER_T / 2.0 + 0.003)), material=METAL, name=f"screw_slot_{i}")

    # Raised cover recess/ribs and a yellow caution tag as real geometry, not paint alone.
    cover.visual(Box((0.140, 0.004, 0.002)), origin=_cover_origin((0.0, -0.034, COVER_T / 2.0 + 0.001)), material=BLACK, name="cover_recess_rear")
    cover.visual(Box((0.140, 0.004, 0.002)), origin=_cover_origin((0.0, -0.104, COVER_T / 2.0 + 0.001)), material=BLACK, name="cover_recess_front")
    cover.visual(Box((0.004, 0.074, 0.002)), origin=_cover_origin((-0.070, -0.069, COVER_T / 2.0 + 0.001)), material=BLACK, name="cover_recess_side_0")
    cover.visual(Box((0.004, 0.074, 0.002)), origin=_cover_origin((0.070, -0.069, COVER_T / 2.0 + 0.001)), material=BLACK, name="cover_recess_side_1")
    cover.visual(Box((0.038, 0.018, 0.0015)), origin=_cover_origin((0.0, -0.067, COVER_T / 2.0 + 0.00075)), material=WARNING, name="cover_warning_label")
    for i in range(3):
        cover.visual(Box((0.030, 0.002, 0.0012)), origin=_cover_origin((0.0, -0.073 + i * 0.006, COVER_T / 2.0 + 0.0018)), material=BLACK, name=f"warning_line_{i}")
    for i, x in enumerate((-0.055, -0.033, -0.011, 0.011, 0.033, 0.055)):
        cover.visual(Box((0.010, 0.005, 0.003)), origin=_cover_origin((x, -COVER_D + 0.021, COVER_T / 2.0 + 0.0015)), material=BLACK, name=f"finger_rib_{i}")

    cover.visual(Box((0.070, 0.012, 0.005)), origin=_cover_origin((0.0, -0.010, -0.003)), material=BLACK, name="cover_hinge_leaf")
    cover.visual(Cylinder(0.0065, 0.060), origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=BLACK, name="cover_barrel")

    hinge = model.articulation(
        "cover_hinge",
        ArticulationType.REVOLUTE,
        parent=enclosure,
        child=cover,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-OPEN_ANGLE, upper=0.0),
    )
    hinge.meta["description"] = "Default pose shows the lid open; positive motion from the lower limit opens the cover."

    # Fixed mounted subassemblies.
    model.articulation("enclosure_to_terminals", ArticulationType.FIXED, parent=enclosure, child=terminals)
    model.articulation("enclosure_to_ground", ArticulationType.FIXED, parent=enclosure, child=ground)
    for wire in wires:
        model.articulation(f"enclosure_to_{wire.name}", ArticulationType.FIXED, parent=enclosure, child=wire)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    enclosure = object_model.get_part("enclosure")
    cover = object_model.get_part("cover")
    terminals = object_model.get_part("terminal_strip")
    ground = object_model.get_part("ground_lug")
    hinge = object_model.get_articulation("cover_hinge")

    ctx.check(
        "classified as electrical junction box",
        object_model.name == "electrical_junction_box"
        and object_model.meta.get("small_class") == "Junction box"
        and object_model.meta.get("domain") == "Electrical_Wiring",
        details=f"name={object_model.name}, meta={object_model.meta}",
    )
    ctx.check(
        "one meaningful cover hinge",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and hinge.motion_limits is not None
        and hinge.motion_limits.lower < -1.0
        and hinge.motion_limits.upper == 0.0,
        details=str(hinge),
    )
    ctx.check(
        "multiple cable glands on several sides",
        sum(1 for v in enclosure.visuals if "gland" in (v.name or "")) >= 6,
        details=[v.name for v in enclosure.visuals if "gland" in (v.name or "")],
    )
    ctx.check(
        "eight terminal sockets are modeled",
        sum(1 for v in terminals.visuals if "terminal_socket" in (v.name or "")) == 8,
        details=[v.name for v in terminals.visuals],
    )
    ctx.check(
        "ground lug and green earth lead present",
        ground.get_visual("lug_plate") is not None and object_model.get_part("wire_green") is not None,
        details="missing ground lug or green wire",
    )
    ctx.check(
        "cover has retained screws and warning label",
        sum(1 for v in cover.visuals if "cover_screw" in (v.name or "")) == 4
        and cover.get_visual("cover_warning_label") is not None,
        details=[v.name for v in cover.visuals],
    )

    ctx.expect_contact(
        terminals,
        enclosure,
        elem_a="terminal_base",
        elem_b="floor_plate",
        contact_tol=0.001,
        name="terminal strip sits on enclosure floor",
    )
    ctx.expect_overlap(
        terminals,
        enclosure,
        axes="xy",
        elem_a="terminal_base",
        elem_b="floor_plate",
        min_overlap=0.040,
        name="terminal strip is inside junction box footprint",
    )
    ctx.expect_contact(
        ground,
        enclosure,
        elem_a="lug_plate",
        elem_b="floor_plate",
        contact_tol=0.001,
        name="ground lug is screwed to the floor",
    )

    open_aabb = ctx.part_element_world_aabb(cover, elem="cover_panel")
    with ctx.pose({hinge: hinge.motion_limits.lower}):
        closed_aabb = ctx.part_element_world_aabb(cover, elem="cover_panel")
        ctx.expect_overlap(
            cover,
            enclosure,
            axes="xy",
            elem_a="cover_panel",
            elem_b="floor_plate",
            min_overlap=0.080,
            name="closed cover spans the box opening",
        )
    ctx.check(
        "default pose shows cover opened upward",
        open_aabb is not None
        and closed_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.045,
        details=f"open_aabb={open_aabb}, closed_aabb={closed_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
