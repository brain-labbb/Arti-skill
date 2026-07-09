from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


BOX_X = 0.220
BOX_Y = 0.160
BOX_Z = 0.055
WALL = 0.012
HINGE_Y = BOX_Y / 2 + 0.004
HINGE_Z = BOX_Z + 0.004
COVER_CLOSE = -math.pi / 2


def _mat(model: ArticulatedObject, name: str, rgba: tuple[float, float, float, float]):
    return model.material(name, rgba=rgba)


def _box(part, name, size, xyz, material, rpy=(0.0, 0.0, 0.0)):
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _cyl(part, name, radius, length, xyz, material, rpy=(0.0, 0.0, 0.0)):
    part.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _wire(part, name, points, radius, material):
    geom = tube_from_spline_points(
        points,
        radius=radius,
        samples_per_segment=10,
        radial_segments=14,
        cap_ends=True,
    )
    part.visual(mesh_from_geometry(geom, name), material=material, name=name)


def _side_gland(enclosure, prefix, x_sign, y, conduit_mat, gland_mat, rubber_mat):
    """One side cable gland, with visible conduit, ribbed compression nut, and inner bushing."""
    outward = x_sign
    # Galvanized conduit leaves the box; it slips under the white gland cap.
    _cyl(
        enclosure,
        f"{prefix}_conduit",
        0.012,
        0.155,
        (outward * (BOX_X / 2 + 0.075), y, 0.031),
        conduit_mat,
        rpy=(0.0, math.pi / 2, 0.0),
    )
    _cyl(
        enclosure,
        f"{prefix}_rubber_jacket",
        0.010,
        0.030,
        (outward * (BOX_X / 2 + 0.016), y, 0.031),
        rubber_mat,
        rpy=(0.0, math.pi / 2, 0.0),
    )
    _cyl(
        enclosure,
        f"{prefix}_locknut",
        0.018,
        0.010,
        (outward * (BOX_X / 2 + 0.002), y, 0.031),
        gland_mat,
        rpy=(0.0, math.pi / 2, 0.0),
    )
    _cyl(
        enclosure,
        f"{prefix}_cap",
        0.019,
        0.030,
        (outward * (BOX_X / 2 + 0.025), y, 0.031),
        gland_mat,
        rpy=(0.0, math.pi / 2, 0.0),
    )
    for i, dx in enumerate((0.013, 0.019, 0.025, 0.031, 0.037)):
        _cyl(
            enclosure,
            f"{prefix}_grip_rib_{i}",
            0.0205,
            0.0025,
            (outward * (BOX_X / 2 + dx), y, 0.031),
            gland_mat,
            rpy=(0.0, math.pi / 2, 0.0),
        )
    _cyl(
        enclosure,
        f"{prefix}_inner_bushing",
        0.014,
        0.010,
        (outward * (BOX_X / 2 - 0.011), y, 0.031),
        gland_mat,
        rpy=(0.0, math.pi / 2, 0.0),
    )


def _add_screw_boss(enclosure, name, x, y, plastic, dark, metal):
    _cyl(enclosure, f"{name}_boss", 0.010, 0.018, (x, y, 0.017), plastic)
    _cyl(enclosure, f"{name}_hole", 0.0043, 0.002, (x, y, 0.0262), dark)
    _cyl(enclosure, f"{name}_screw_head", 0.0058, 0.002, (x, y, 0.0274), metal)
    _box(enclosure, f"{name}_screw_slot", (0.0085, 0.0014, 0.0009), (x, y, 0.0288), dark)


def _add_terminal_row(enclosure, row, y, metal, dark, white):
    for side, x in (("left", -0.020), ("right", 0.020)):
        _box(enclosure, f"terminal_{side}_clamp_{row}", (0.024, 0.009, 0.004), (x, y, 0.026), metal)
        _cyl(enclosure, f"terminal_{side}_screw_{row}", 0.0042, 0.0022, (x, y, 0.0284), metal)
        _box(enclosure, f"terminal_{side}_slot_{row}", (0.0062, 0.0010, 0.0008), (x, y, 0.0296), dark)
        sleeve_x = x - 0.020 if side == "left" else x + 0.020
        _box(enclosure, f"terminal_{side}_ferrule_{row}", (0.018, 0.006, 0.005), (sleeve_x, y, 0.027), white)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="electrical_junction_box",
        meta={
            "class": "Junction box",
            "domain": "Electrical_Wiring",
            "description": "Open electrical junction box with side cable glands, terminal strip, bundled conductors, and a hinged service cover.",
        },
    )

    grey = _mat(model, "hammered_gray_plastic", (0.52, 0.55, 0.57, 1.0))
    dark_grey = _mat(model, "dark_shadow_recess", (0.035, 0.038, 0.040, 1.0))
    rubber = _mat(model, "black_rubber", (0.005, 0.006, 0.007, 1.0))
    white = _mat(model, "white_plastic", (0.93, 0.90, 0.84, 1.0))
    metal = _mat(model, "zinc_screw_metal", (0.78, 0.80, 0.80, 1.0))
    galvanized = _mat(model, "galvanized_conduit", (0.62, 0.66, 0.68, 1.0))
    brass = _mat(model, "brass_ground_lug", (0.78, 0.57, 0.23, 1.0))
    copper = _mat(model, "copper_bus_bar", (0.84, 0.39, 0.15, 1.0))
    label_yellow = _mat(model, "yellow_warning_label", (0.98, 0.82, 0.08, 1.0))
    label_black = _mat(model, "label_print_black", (0.02, 0.02, 0.018, 1.0))
    red = _mat(model, "red_wire", (0.88, 0.04, 0.02, 1.0))
    blue = _mat(model, "blue_wire", (0.02, 0.18, 0.78, 1.0))
    green = _mat(model, "green_wire", (0.02, 0.55, 0.20, 1.0))
    yellow = _mat(model, "yellow_wire", (0.96, 0.72, 0.04, 1.0))
    orange = _mat(model, "orange_wire", (0.96, 0.28, 0.06, 1.0))
    brown = _mat(model, "brown_wire", (0.45, 0.20, 0.08, 1.0))
    black_wire = _mat(model, "black_wire", (0.01, 0.01, 0.012, 1.0))
    white_wire = _mat(model, "white_wire", (0.96, 0.96, 0.93, 1.0))

    enclosure = model.part("enclosure")

    # Hollow molded tray: explicit bottom, four walls, raised gasket rim, and rounded-looking corner pads.
    _box(enclosure, "bottom_plate", (0.198, 0.138, 0.008), (0.0, 0.0, 0.004), grey)
    _box(enclosure, "left_wall", (WALL, BOX_Y, BOX_Z), (-BOX_X / 2 + WALL / 2, 0.0, BOX_Z / 2), grey)
    _box(enclosure, "right_wall", (WALL, BOX_Y, BOX_Z), (BOX_X / 2 - WALL / 2, 0.0, BOX_Z / 2), grey)
    _box(enclosure, "front_wall", (BOX_X, WALL, BOX_Z), (0.0, -BOX_Y / 2 + WALL / 2, BOX_Z / 2), grey)
    _box(enclosure, "rear_wall", (BOX_X, WALL, BOX_Z), (0.0, BOX_Y / 2 - WALL / 2, BOX_Z / 2), grey)
    _box(enclosure, "front_gasket_seat", (0.175, 0.004, 0.004), (0.0, -0.060, 0.0565), rubber)
    _box(enclosure, "rear_gasket_seat", (0.175, 0.004, 0.004), (0.0, 0.060, 0.0565), rubber)
    _box(enclosure, "left_gasket_seat", (0.004, 0.118, 0.004), (-0.088, 0.0, 0.0565), rubber)
    _box(enclosure, "right_gasket_seat", (0.004, 0.118, 0.004), (0.088, 0.0, 0.0565), rubber)
    for ix, x in enumerate((-0.097, 0.097)):
        for iy, y in enumerate((-0.067, 0.067)):
            _cyl(enclosure, f"molded_corner_post_{ix}_{iy}", 0.018, BOX_Z, (x, y, BOX_Z / 2), grey)
            _cyl(enclosure, f"corner_knockout_{ix}_{iy}", 0.008, 0.003, (x, y, BOX_Z + 0.001), dark_grey)
            _cyl(enclosure, f"corner_screw_bore_{ix}_{iy}", 0.0042, 0.004, (x, y, 0.0075), dark_grey)

    # Internal supports and grounded electrical hardware.
    for name, x, y in (
        ("inside_front_left", -0.072, -0.048),
        ("inside_front_right", 0.072, -0.048),
        ("inside_rear_left", -0.072, 0.048),
        ("inside_rear_right", 0.072, 0.048),
    ):
        _add_screw_boss(enclosure, name, x, y, grey, dark_grey, metal)

    _box(enclosure, "terminal_base", (0.068, 0.116, 0.010), (0.0, 0.0, 0.013), white)
    _box(enclosure, "terminal_center_bar", (0.014, 0.122, 0.026), (0.0, 0.0, 0.028), white)
    _box(enclosure, "left_copper_bus", (0.004, 0.108, 0.004), (-0.033, 0.0, 0.024), copper)
    _box(enclosure, "right_copper_bus", (0.004, 0.108, 0.004), (0.033, 0.0, 0.024), copper)
    row_ys = [-0.047, -0.033, -0.019, -0.005, 0.009, 0.023, 0.037, 0.051]
    for row, y in enumerate(row_ys):
        _add_terminal_row(enclosure, row, y, metal, dark_grey, white)

    _box(enclosure, "ground_lug_plate", (0.030, 0.014, 0.004), (-0.070, -0.058, 0.014), brass)
    _cyl(enclosure, "ground_lug_screw", 0.005, 0.002, (-0.070, -0.058, 0.0175), metal)
    _box(enclosure, "ground_symbol_label", (0.026, 0.010, 0.0012), (-0.041, -0.058, 0.0095), label_yellow)
    _box(enclosure, "ground_symbol_bar", (0.016, 0.0012, 0.0014), (-0.041, -0.058, 0.0104), label_black)
    _box(enclosure, "rating_label", (0.038, 0.018, 0.0012), (0.060, 0.058, 0.0095), label_yellow)
    _box(enclosure, "rating_label_stripe_0", (0.030, 0.002, 0.0014), (0.060, 0.053, 0.0104), label_black)
    _box(enclosure, "rating_label_stripe_1", (0.030, 0.002, 0.0014), (0.060, 0.058, 0.0104), label_black)
    _box(enclosure, "rating_label_stripe_2", (0.030, 0.002, 0.0014), (0.060, 0.063, 0.0104), label_black)

    # Four side cable entries match the reference image: threaded white glands and long gray conduits.
    for prefix, x_sign, y in (
        ("left_upper", -1, 0.035),
        ("left_lower", -1, -0.035),
        ("right_upper", 1, 0.035),
        ("right_lower", 1, -0.035),
    ):
        _side_gland(enclosure, prefix, x_sign, y, galvanized, white, rubber)

    # Bundled conductors from glands into a two-column terminal strip.
    left_specs = [
        ("wire_black_left_0", 0.035, row_ys[7], black_wire, -0.006),
        ("wire_white_left_1", 0.035, row_ys[6], white_wire, 0.002),
        ("wire_blue_left_2", 0.035, row_ys[5], blue, 0.010),
        ("wire_orange_left_3", 0.035, row_ys[4], orange, -0.014),
        ("wire_green_left_4", -0.035, row_ys[3], green, 0.006),
        ("wire_red_left_5", -0.035, row_ys[2], red, -0.006),
        ("wire_yellow_left_6", -0.035, row_ys[1], yellow, 0.014),
        ("wire_brown_left_7", -0.035, row_ys[0], brown, -0.012),
    ]
    for name, y0, yt, mat, bow in left_specs:
        _wire(
            enclosure,
            name,
            [(-0.100, y0, 0.032), (-0.074, y0 + bow, 0.043), (-0.052, yt - bow, 0.044), (-0.030, yt, 0.030)],
            0.0022,
            mat,
        )

    right_specs = [
        ("wire_blue_right_0", 0.035, row_ys[7], blue, -0.010),
        ("wire_orange_right_1", 0.035, row_ys[6], orange, 0.008),
        ("wire_white_right_2", 0.035, row_ys[5], white_wire, -0.002),
        ("wire_grey_right_3", 0.035, row_ys[4], galvanized, 0.014),
        ("wire_red_right_4", -0.035, row_ys[3], red, -0.010),
        ("wire_yellow_right_5", -0.035, row_ys[2], yellow, 0.008),
        ("wire_green_right_6", -0.035, row_ys[1], green, -0.016),
        ("wire_black_right_7", -0.035, row_ys[0], black_wire, 0.010),
    ]
    for name, y0, yt, mat, bow in right_specs:
        _wire(
            enclosure,
            name,
            [(0.100, y0, 0.032), (0.074, y0 + bow, 0.043), (0.052, yt - bow, 0.044), (0.030, yt, 0.030)],
            0.0022,
            mat,
        )
    _wire(
        enclosure,
        "ground_wire_to_lug",
        [(-0.100, -0.035, 0.032), (-0.090, -0.054, 0.035), (-0.070, -0.058, 0.020)],
        0.0025,
        green,
    )

    # Exposed hinge hardware on the rear edge; base knuckles are interleaved with moving cover knuckles.
    _box(enclosure, "base_hinge_leaf", (0.190, 0.009, 0.004), (0.0, HINGE_Y - 0.002, HINGE_Z - 0.004), metal)
    for i, (x, length) in enumerate(((-0.074, 0.040), (0.0, 0.040), (0.074, 0.040))):
        _cyl(enclosure, f"base_hinge_knuckle_{i}", 0.0048, length, (x, HINGE_Y, HINGE_Z), metal, rpy=(0.0, math.pi / 2, 0.0))
    _cyl(enclosure, "hinge_pin_axis_visible", 0.0017, 0.204, (0.0, HINGE_Y, HINGE_Z), dark_grey, rpy=(0.0, math.pi / 2, 0.0))

    cover = model.part("cover")
    # The default pose is the service-open pose shown in the reference; q=COVER_CLOSE swings it down over the gasket.
    _box(cover, "cover_panel", (0.214, 0.006, 0.166), (0.0, 0.006, 0.083), grey)
    _box(cover, "cover_inner_lip_front", (0.168, 0.004, 0.004), (0.0, 0.001, 0.022), rubber)
    _box(cover, "cover_inner_lip_rear", (0.168, 0.004, 0.004), (0.0, 0.001, 0.144), rubber)
    _box(cover, "cover_inner_lip_left", (0.004, 0.004, 0.126), (-0.084, 0.001, 0.083), rubber)
    _box(cover, "cover_inner_lip_right", (0.004, 0.004, 0.126), (0.084, 0.001, 0.083), rubber)
    _box(cover, "cover_hinge_leaf", (0.190, 0.008, 0.010), (0.0, 0.004, 0.005), metal)
    for i, (x, length) in enumerate(((-0.037, 0.026), (0.037, 0.026))):
        _cyl(cover, f"cover_hinge_knuckle_{i}", 0.0046, length, (x, 0.0, 0.0), metal, rpy=(0.0, math.pi / 2, 0.0))
    for i, (x, z) in enumerate(((-0.087, 0.025), (0.087, 0.025), (-0.087, 0.141), (0.087, 0.141))):
        _cyl(cover, f"cover_retained_screw_recess_{i}", 0.0062, 0.002, (x, 0.0026, z), dark_grey, rpy=(-math.pi / 2, 0.0, 0.0))
        _cyl(cover, f"cover_retained_screw_head_{i}", 0.0042, 0.002, (x, 0.0015, z), metal, rpy=(-math.pi / 2, 0.0, 0.0))
    _box(cover, "cover_warning_label", (0.060, 0.0014, 0.022), (0.0, 0.0027, 0.084), label_yellow)
    _box(cover, "cover_label_line_0", (0.045, 0.0016, 0.002), (0.0, 0.0025, 0.078), label_black)
    _box(cover, "cover_label_line_1", (0.045, 0.0016, 0.002), (0.0, 0.0025, 0.084), label_black)
    _box(cover, "cover_label_line_2", (0.045, 0.0016, 0.002), (0.0, 0.0025, 0.090), label_black)
    _box(cover, "cover_latch_tab", (0.045, 0.010, 0.014), (0.0, 0.006, 0.166), grey)
    _cyl(cover, "cover_pull_dimple", 0.004, 0.0016, (0.0, 0.0020, 0.162), dark_grey, rpy=(-math.pi / 2, 0.0, 0.0))

    model.articulation(
        "base_to_cover",
        ArticulationType.REVOLUTE,
        parent=enclosure,
        child=cover,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=1.5, lower=COVER_CLOSE, upper=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    enclosure = object_model.get_part("enclosure")
    cover = object_model.get_part("cover")
    hinge = object_model.get_articulation("base_to_cover")

    ctx.check("small class is junction box", object_model.meta.get("class") == "Junction box")
    ctx.check("object remains electrical wiring domain", object_model.meta.get("domain") == "Electrical_Wiring")
    ctx.check("one moving cover hinge", hinge.articulation_type == ArticulationType.REVOLUTE)
    ctx.check(
        "cover hinge has service limits",
        hinge.motion_limits is not None
        and hinge.motion_limits.lower <= COVER_CLOSE + 1e-6
        and hinge.motion_limits.upper == 0.0,
    )
    names = [v.name for v in enclosure.visuals]
    ctx.check("four cable glands with conduits", sum(name.endswith("_conduit") for name in names) == 4)
    ctx.check("terminal strip has many screw terminals", sum("terminal_" in name and "_screw_" in name for name in names) >= 16)
    ctx.check("bundled colored wire ends present", sum(name.startswith("wire_") for name in names) >= 16)
    ctx.check("ground lug modeled", any(name == "ground_lug_plate" for name in names))
    ctx.check("warning and rating labels are geometry", {"rating_label", "ground_symbol_label"}.issubset(set(names)))
    ctx.expect_contact(
        enclosure,
        enclosure,
        elem_a="terminal_base",
        elem_b="bottom_plate",
        name="terminal strip is mounted to box floor",
    )
    ctx.expect_within(
        enclosure,
        enclosure,
        axes="xy",
        inner_elem="terminal_base",
        outer_elem="bottom_plate",
        margin=0.0,
        name="terminal strip sits inside enclosure footprint",
    )

    # In the authored service pose the cover stands clear of the open box, showing the wired interior.
    ctx.expect_gap(
        cover,
        enclosure,
        axis="y",
        min_gap=0.001,
        positive_elem="cover_panel",
        negative_elem="rear_wall",
        name="open cover clears rear wall",
    )

    # At the lower limit the same cover rotates down over the gasket seat without hiding the fact that it is hinged.
    with ctx.pose({hinge: COVER_CLOSE}):
        ctx.expect_overlap(
            cover,
            enclosure,
            axes="xy",
            elem_a="cover_panel",
            elem_b="bottom_plate",
            min_overlap=0.10,
            name="closed cover spans the enclosure mouth",
        )
        ctx.expect_gap(
            cover,
            enclosure,
            axis="z",
            max_gap=0.014,
            max_penetration=0.0,
            positive_elem="cover_panel",
            negative_elem="front_wall",
            name="closed cover seats just above front rim",
        )

    return ctx.report()


object_model = build_object_model()
