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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="electrical_junction_box",
        meta={
            "class": "Junction box",
            "domain": "Electrical_Wiring",
            "description": "Empty pass-through electrical junction box with side cable glands, bare gasketed enclosure, ground lug, and a hinged service cover.",
        },
    )

    grey = _mat(model, "hammered_gray_plastic", (0.52, 0.55, 0.57, 1.0))
    dark_grey = _mat(model, "dark_shadow_recess", (0.035, 0.038, 0.040, 1.0))
    rubber = _mat(model, "black_rubber", (0.005, 0.006, 0.007, 1.0))
    white = _mat(model, "white_plastic", (0.93, 0.90, 0.84, 1.0))
    metal = _mat(model, "zinc_screw_metal", (0.78, 0.80, 0.80, 1.0))
    galvanized = _mat(model, "galvanized_conduit", (0.62, 0.66, 0.68, 1.0))
    brass = _mat(model, "brass_ground_lug", (0.78, 0.57, 0.23, 1.0))
    label_yellow = _mat(model, "yellow_warning_label", (0.98, 0.82, 0.08, 1.0))
    label_black = _mat(model, "label_print_black", (0.02, 0.02, 0.018, 1.0))

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

    # Internal screw bosses for cover retention — no terminal strip in this pass-through variant.
    for name, x, y in (
        ("inside_front_left", -0.072, -0.048),
        ("inside_front_right", 0.072, -0.048),
        ("inside_rear_left", -0.072, 0.048),
        ("inside_rear_right", 0.072, 0.048),
    ):
        _add_screw_boss(enclosure, name, x, y, grey, dark_grey, metal)

    # Ground lug remains as the sole internal hardware on the enclosure floor.
    # Bottom plate top is at z=0.008; seat the lug plate directly on it.
    _box(enclosure, "ground_lug_plate", (0.030, 0.014, 0.004), (-0.070, -0.058, 0.010), brass)
    _cyl(enclosure, "ground_lug_screw", 0.005, 0.002, (-0.070, -0.058, 0.0135), metal)
    # Labels sit flush on the bottom plate surface (top at z=0.008, half-thickness embed for contact).
    _box(enclosure, "ground_symbol_label", (0.026, 0.010, 0.001), (-0.041, -0.058, 0.0085), label_yellow)
    _box(enclosure, "ground_symbol_bar", (0.016, 0.0012, 0.001), (-0.041, -0.058, 0.0091), label_black)
    _box(enclosure, "rating_label", (0.038, 0.018, 0.001), (0.060, 0.058, 0.0085), label_yellow)
    _box(enclosure, "rating_label_stripe_0", (0.030, 0.002, 0.001), (0.060, 0.053, 0.0091), label_black)
    _box(enclosure, "rating_label_stripe_1", (0.030, 0.002, 0.001), (0.060, 0.058, 0.0091), label_black)
    _box(enclosure, "rating_label_stripe_2", (0.030, 0.002, 0.001), (0.060, 0.063, 0.0091), label_black)

    # Four side cable entries: threaded white glands and long gray conduits — open pass-through configuration.
    for prefix, x_sign, y in (
        ("left_upper", -1, 0.035),
        ("left_lower", -1, -0.035),
        ("right_upper", 1, 0.035),
        ("right_lower", 1, -0.035),
    ):
        _side_gland(enclosure, prefix, x_sign, y, galvanized, white, rubber)

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
    ctx.check("ground lug modeled", any(name == "ground_lug_plate" for name in names))
    ctx.check("warning and rating labels are geometry", {"rating_label", "ground_symbol_label"}.issubset(set(names)))

    # Pass-through variant: no terminal strip, no copper bus, no bundled wires.
    ctx.check(
        "interior is empty pass-through (no terminal_base)",
        not any(name == "terminal_base" for name in names),
    )
    ctx.check(
        "no copper bus bars in pass-through variant",
        not any("copper_bus" in name for name in names),
    )
    ctx.check(
        "no bundled wire ends in pass-through variant",
        not any(name.startswith("wire_") for name in names),
    )

    # Ground lug is the only internal hardware, mounted on the enclosure floor.
    ctx.expect_contact(
        enclosure,
        enclosure,
        elem_a="ground_lug_plate",
        elem_b="bottom_plate",
        name="ground lug is mounted to box floor",
    )
    ctx.expect_within(
        enclosure,
        enclosure,
        axes="xy",
        inner_elem="ground_lug_plate",
        outer_elem="bottom_plate",
        margin=0.0,
        name="ground lug sits inside enclosure footprint",
    )

    # In the authored service pose the cover stands clear of the open box.
    ctx.expect_gap(
        cover,
        enclosure,
        axis="y",
        min_gap=0.001,
        positive_elem="cover_panel",
        negative_elem="rear_wall",
        name="open cover clears rear wall",
    )

    # At the lower limit the same cover rotates down over the gasket seat.
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
