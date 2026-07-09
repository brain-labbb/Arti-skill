from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


BOX_L = 0.140
BOX_W = 0.100
BOX_H = 0.042
WALL = 0.004
HINGE_Y = BOX_W / 2.0 + 0.006
HINGE_Z = 0.047


def _visual(part, geometry, *, xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0), material=None, name=None):
    part.visual(geometry, origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _add_phillips_head(part, x: float, y: float, z: float, *, metal, slot, prefix: str) -> None:
    _visual(part, Cylinder(radius=0.0058, length=0.0024), xyz=(x, y, z), material=metal, name=f"{prefix}_head")
    _visual(part, Box((0.0090, 0.0014, 0.00045)), xyz=(x, y, z + 0.00135), material=slot, name=f"{prefix}_slot_a")
    _visual(part, Box((0.0014, 0.0090, 0.00045)), xyz=(x, y, z + 0.00136), material=slot, name=f"{prefix}_slot_b")


def _add_terminal_screw(part, x: float, y: float, *, white, brass, slot, prefix: str) -> None:
    _visual(part, Cylinder(radius=0.0042, length=0.010), xyz=(x, y, 0.021), material=white, name=f"{prefix}_insulator")
    _visual(part, Cylinder(radius=0.0026, length=0.0009), xyz=(x, y, 0.0264), material=brass, name=f"{prefix}_brass_cup")
    _visual(part, Cylinder(radius=0.0015, length=0.0007), xyz=(x, y, 0.0272), material=slot, name=f"{prefix}_dark_bore")


def _add_gland(part, *, x: float, y: float, front: bool, ribbed_mesh, clear, black, dark, name: str) -> None:
    # Cylinders are authored on local Z; rotate them so their axes run through the box wall.
    axis_rpy = (math.pi / 2.0, 0.0, 0.0)
    sign = -1.0 if front else 1.0
    _visual(part, Cylinder(radius=0.0115, length=0.020), xyz=(x, y + sign * 0.002, 0.026), rpy=axis_rpy, material=clear, name=f"{name}_clear_thread")
    for i, off in enumerate((-0.007, -0.003, 0.001, 0.005)):
        _visual(part, Cylinder(radius=0.0127, length=0.0012), xyz=(x, y + sign * off, 0.026), rpy=axis_rpy, material=clear, name=f"{name}_thread_ring_{i}")
    cap_center = y + sign * 0.027
    _visual(part, ribbed_mesh, xyz=(x, cap_center, 0.026), rpy=axis_rpy, material=black, name=f"{name}_ribbed_nut")
    _visual(part, Cylinder(radius=0.0155, length=0.0055), xyz=(x, y + sign * 0.0125, 0.026), rpy=axis_rpy, material=black, name=f"{name}_collar")
    _visual(part, Cylinder(radius=0.0135, length=0.006), xyz=(x, y + sign * 0.040, 0.026), rpy=axis_rpy, material=black, name=f"{name}_domed_end")
    _visual(part, Cylinder(radius=0.0070, length=0.0014), xyz=(x, y + sign * 0.043, 0.026), rpy=axis_rpy, material=dark, name=f"{name}_cable_hole")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="electrical_junction_box")

    clear = model.material("clear_polycarbonate", rgba=(0.76, 0.84, 1.00, 0.40))
    clear_edge = model.material("thick_clear_edges", rgba=(0.70, 0.78, 1.00, 0.56))
    rubber = model.material("black_rubber", rgba=(0.006, 0.006, 0.008, 1.0))
    dark = model.material("dark_cavity", rgba=(0.0, 0.0, 0.0, 1.0))
    white = model.material("white_nylon_terminal", rgba=(0.92, 0.91, 0.86, 1.0))
    brass = model.material("brass_terminal", rgba=(0.95, 0.68, 0.25, 1.0))
    screw_metal = model.material("polished_screw_steel", rgba=(0.84, 0.86, 0.86, 1.0))
    galvanized = model.material("galvanized_conduit", rgba=(0.50, 0.53, 0.54, 1.0))
    yellow = model.material("yellow_warning_label", rgba=(1.0, 0.84, 0.05, 1.0))
    red = model.material("red_wire_jacket", rgba=(0.86, 0.03, 0.03, 1.0))
    blue = model.material("blue_wire_jacket", rgba=(0.02, 0.14, 0.78, 1.0))
    green = model.material("green_yellow_wire", rgba=(0.08, 0.62, 0.12, 1.0))

    gland_mesh = mesh_from_geometry(
        KnobGeometry(
            0.030,
            0.026,
            body_style="cylindrical",
            edge_radius=0.001,
            grip=KnobGrip(style="ribbed", count=32, depth=0.0013, width=0.0012),
        ),
        "ribbed_cable_gland_nut",
    )

    enclosure = model.part("enclosure")

    # Hollow translucent molded box: separate walls and bottom leave a real open interior.
    _visual(enclosure, Box((BOX_L, BOX_W, 0.004)), xyz=(0.0, 0.0, 0.002), material=clear, name="bottom_plate")
    _visual(enclosure, Box((BOX_L - 0.018, WALL, BOX_H)), xyz=(0.0, -BOX_W / 2 + WALL / 2, BOX_H / 2), material=clear, name="front_wall")
    _visual(enclosure, Box((BOX_L - 0.018, WALL, BOX_H)), xyz=(0.0, BOX_W / 2 - WALL / 2, BOX_H / 2), material=clear, name="rear_wall")
    _visual(enclosure, Box((WALL, BOX_W - 0.018, BOX_H)), xyz=(-BOX_L / 2 + WALL / 2, 0.0, BOX_H / 2), material=clear, name="side_wall_0")
    _visual(enclosure, Box((WALL, BOX_W - 0.018, BOX_H)), xyz=(BOX_L / 2 - WALL / 2, 0.0, BOX_H / 2), material=clear, name="side_wall_1")
    for ix, x in enumerate((-BOX_L / 2 + 0.009, BOX_L / 2 - 0.009)):
        for iy, y in enumerate((-BOX_W / 2 + 0.009, BOX_W / 2 - 0.009)):
            _visual(enclosure, Cylinder(radius=0.009, length=BOX_H), xyz=(x, y, BOX_H / 2), material=clear_edge, name=f"rounded_corner_{ix}_{iy}")

    # Compressed gasket lip the cover seats against.
    _visual(enclosure, Box((BOX_L - 0.020, 0.0022, 0.0013)), xyz=(0.0, -BOX_W / 2 + 0.007, BOX_H + 0.00065), material=rubber, name="gasket_front")
    _visual(enclosure, Box((BOX_L - 0.020, 0.0022, 0.0013)), xyz=(0.0, BOX_W / 2 - 0.007, BOX_H + 0.00065), material=rubber, name="gasket_rear")
    _visual(enclosure, Box((0.0022, BOX_W - 0.020, 0.0013)), xyz=(-BOX_L / 2 + 0.007, 0.0, BOX_H + 0.00065), material=rubber, name="gasket_side_0")
    _visual(enclosure, Box((0.0022, BOX_W - 0.020, 0.0013)), xyz=(BOX_L / 2 - 0.007, 0.0, BOX_H + 0.00065), material=rubber, name="gasket_side_1")

    # Screw bosses rising from the base floor.
    boss_points = [(-0.055, -0.035), (0.055, -0.035), (-0.055, 0.035), (0.055, 0.035), (0.000, -0.035)]
    for i, (x, y) in enumerate(boss_points):
        _visual(enclosure, Cylinder(radius=0.0066, length=0.036), xyz=(x, y, 0.022), material=clear_edge, name=f"screw_boss_{i}")
        _visual(enclosure, Cylinder(radius=0.0022, length=0.0010), xyz=(x, y, 0.0405), material=dark, name=f"boss_hole_{i}")

    # External mounting ears with screw holes.
    for i, (x, y) in enumerate(((-BOX_L / 2 - 0.008, -0.026), (-BOX_L / 2 - 0.008, 0.026), (BOX_L / 2 + 0.008, -0.026), (BOX_L / 2 + 0.008, 0.026))):
        _visual(enclosure, Box((0.020, 0.014, 0.004)), xyz=(x, y, 0.006), material=clear_edge, name=f"mounting_ear_{i}")
        _visual(enclosure, Cylinder(radius=0.0031, length=0.0044), xyz=(x, y, 0.0084), material=dark, name=f"mounting_hole_{i}")

    # Three cable glands and side knockouts: two front entries and one rear entry.
    _add_gland(enclosure, x=-0.032, y=-BOX_W / 2, front=True, ribbed_mesh=gland_mesh, clear=clear_edge, black=rubber, dark=dark, name="front_gland_0")
    _add_gland(enclosure, x=0.032, y=-BOX_W / 2, front=True, ribbed_mesh=gland_mesh, clear=clear_edge, black=rubber, dark=dark, name="front_gland_1")
    _add_gland(enclosure, x=0.034, y=BOX_W / 2, front=False, ribbed_mesh=gland_mesh, clear=clear_edge, black=rubber, dark=dark, name="rear_gland")
    for i, (x, side) in enumerate(((-BOX_L / 2, -1.0), (BOX_L / 2, 1.0))):
        _visual(enclosure, Cylinder(radius=0.010, length=0.0012), xyz=(x, 0.0, 0.025), rpy=(0.0, math.pi / 2.0, 0.0), material=clear_edge, name=f"side_knockout_{i}")

    # Internal terminal strip, bus details, and ground lug.
    _visual(enclosure, Box((0.076, 0.018, 0.012)), xyz=(0.0, -0.002, 0.010), material=white, name="terminal_strip")
    _visual(enclosure, Box((0.070, 0.003, 0.0014)), xyz=(0.0, -0.002, 0.0167), material=brass, name="brass_bus_bar")
    n = 0
    for x in (-0.026, 0.0, 0.026):
        for y in (-0.008, 0.004):
            _add_terminal_screw(enclosure, x, y, white=white, brass=brass, slot=dark, prefix=f"terminal_{n}")
            n += 1
    _visual(enclosure, Box((0.018, 0.010, 0.003)), xyz=(-0.045, 0.024, 0.0065), material=brass, name="ground_lug")
    _visual(enclosure, Cylinder(radius=0.0028, length=0.0012), xyz=(-0.045, 0.024, 0.0088), material=screw_metal, name="ground_screw")

    # Believable wire ends entering from the glands and landing on terminals.
    wire_specs = [
        ("red_wire", red, [(-0.032, -0.046, 0.026), (-0.034, -0.027, 0.031), (-0.026, -0.010, 0.027)]),
        ("blue_wire", blue, [(0.032, -0.046, 0.026), (0.028, -0.025, 0.030), (0.026, 0.004, 0.027)]),
        ("green_wire", green, [(0.034, 0.046, 0.026), (0.005, 0.030, 0.031), (-0.045, 0.024, 0.010)]),
        ("black_wire", rubber, [(-0.032, -0.046, 0.022), (-0.010, -0.028, 0.026), (0.000, -0.008, 0.027)]),
    ]
    for wire_name, mat, pts in wire_specs:
        wire_mesh = mesh_from_geometry(
            tube_from_spline_points(pts, radius=0.00145, samples_per_segment=10, radial_segments=12, cap_ends=True),
            wire_name,
        )
        _visual(enclosure, wire_mesh, material=mat, name=wire_name)

    # Static half of the hinge: a rear leaf and alternating knuckles.
    _visual(enclosure, Box((BOX_L, 0.0035, 0.006)), xyz=(0.0, BOX_W / 2 + 0.00175, HINGE_Z - 0.002), material=galvanized, name="hinge_leaf")
    for i, (x, length) in enumerate(((-0.048, 0.026), (0.000, 0.020), (0.050, 0.026))):
        _visual(enclosure, Cylinder(radius=0.0037, length=length), xyz=(x, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0), material=galvanized, name=f"base_hinge_barrel_{i}")

    cover = model.part("cover")
    # Child frame is the hinge axis.  At q=0 the transparent cover is closed and extends along local -Y.
    _visual(cover, Box((0.148, 0.106, 0.006)), xyz=(0.0, -0.055, 0.0), material=clear, name="lid_panel")
    _visual(cover, Box((0.142, 0.006, 0.002)), xyz=(0.0, -0.005, 0.004), material=clear_edge, name="lid_rear_rim")
    _visual(cover, Box((0.142, 0.006, 0.002)), xyz=(0.0, -0.105, 0.004), material=clear_edge, name="lid_front_rim")
    _visual(cover, Box((0.006, 0.096, 0.002)), xyz=(-0.071, -0.055, 0.004), material=clear_edge, name="lid_side_rim_0")
    _visual(cover, Box((0.006, 0.096, 0.002)), xyz=(0.071, -0.055, 0.004), material=clear_edge, name="lid_side_rim_1")
    _visual(cover, Box((0.105, 0.0012, 0.0012)), xyz=(0.0, -0.055, 0.0048), material=clear_edge, name="molded_center_seam_x")
    _visual(cover, Box((0.0012, 0.078, 0.0012)), xyz=(0.0, -0.055, 0.0049), material=clear_edge, name="molded_center_seam_y")

    # Moving hinge knuckles and leaf, aligned to alternate with the base barrels.
    _visual(cover, Box((0.116, 0.003, 0.004)), xyz=(0.0, -0.0045, -0.001), material=galvanized, name="cover_hinge_leaf")
    for i, (x, length) in enumerate(((-0.024, 0.020), (0.025, 0.026))):
        _visual(cover, Cylinder(radius=0.0033, length=length), xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0), material=galvanized, name=f"cover_hinge_barrel_{i}")

    # Retained cover screws and molded pads.
    lid_screws = [(-0.055, -0.091), (0.055, -0.091), (-0.055, -0.021), (0.055, -0.021), (0.0, -0.091)]
    for i, (x, y) in enumerate(lid_screws):
        _visual(cover, Cylinder(radius=0.0082, length=0.0020), xyz=(x, y, 0.0038), material=clear_edge, name=f"lid_screw_pad_{i}")
        _add_phillips_head(cover, x, y, 0.0052, metal=screw_metal, slot=dark, prefix=f"cover_screw_{i}")

    # Labels and molded warnings on the transparent lid.
    _visual(cover, Box((0.034, 0.014, 0.0007)), xyz=(0.022, -0.052, 0.00335), material=yellow, name="warning_label")
    _visual(cover, Box((0.003, 0.010, 0.0009)), xyz=(0.014, -0.052, 0.00395), material=dark, name="warning_bolt_0")
    _visual(cover, Box((0.003, 0.010, 0.0009)), xyz=(0.020, -0.052, 0.00395), rpy=(0.0, 0.0, -0.55), material=dark, name="warning_bolt_1")
    _visual(cover, Box((0.026, 0.004, 0.0008)), xyz=(-0.023, -0.067, 0.0034), material=clear_edge, name="raised_ip66_mark")
    _visual(cover, Box((0.020, 0.003, 0.0008)), xyz=(-0.025, -0.043, 0.0034), material=clear_edge, name="raised_rating_mark")

    model.articulation(
        "cover_hinge",
        ArticulationType.REVOLUTE,
        parent=enclosure,
        child=cover,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=1.5, lower=0.0, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    enclosure = object_model.get_part("enclosure")
    cover = object_model.get_part("cover")
    hinge = object_model.get_articulation("cover_hinge")

    ctx.check("asset is named as a junction box", "junction_box" in object_model.name)
    ctx.check("primary hinged cover joint exists", hinge is not None)
    ctx.check("junction box has exactly one non-fixed cover joint", len(object_model.articulations) == 1)

    electrical_visuals = {v.name for v in enclosure.visuals}
    required_details = {
        "terminal_strip",
        "brass_bus_bar",
        "ground_lug",
        "front_gland_0_ribbed_nut",
        "front_gland_1_ribbed_nut",
        "rear_gland_ribbed_nut",
        "red_wire",
        "blue_wire",
        "green_wire",
        "hinge_leaf",
    }
    ctx.check(
        "electrical junction details are modeled as geometry",
        required_details.issubset(electrical_visuals),
        details=f"missing={sorted(required_details - electrical_visuals)}",
    )
    cover_visuals = {v.name for v in cover.visuals}
    ctx.check(
        "transparent cover has screws gasket-facing rim and labels",
        {"lid_panel", "warning_label", "cover_screw_0_head", "lid_front_rim", "cover_hinge_barrel_0"}.issubset(cover_visuals),
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            cover,
            enclosure,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="gasket_front",
            min_gap=0.0,
            max_gap=0.002,
            name="closed lid seats just above gasket without penetrating",
        )
        ctx.expect_overlap(
            cover,
            enclosure,
            axes="xy",
            elem_a="lid_panel",
            elem_b="bottom_plate",
            min_overlap=0.090,
            name="closed cover spans the enclosure footprint",
        )

    closed_aabb = ctx.part_element_world_aabb(cover, elem="lid_panel")
    with ctx.pose({hinge: 1.15}):
        open_aabb = ctx.part_element_world_aabb(cover, elem="lid_panel")
        ctx.check(
            "hinge opens the cover upward with a realistic limit",
            closed_aabb is not None and open_aabb is not None and open_aabb[1][2] > closed_aabb[1][2] + 0.045,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
