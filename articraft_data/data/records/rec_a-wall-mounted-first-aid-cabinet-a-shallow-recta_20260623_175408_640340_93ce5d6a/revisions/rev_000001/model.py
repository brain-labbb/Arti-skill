from __future__ import annotations

from math import pi

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mounted_first_aid_cabinet")

    steel = Material("white_painted_steel", rgba=(0.94, 0.95, 0.92, 1.0))
    red = Material("first_aid_red", rgba=(0.90, 0.04, 0.03, 1.0))
    glass = Material("slightly_blue_glass", rgba=(0.72, 0.88, 0.95, 0.38))
    metal = Material("brushed_metal", rgba=(0.62, 0.64, 0.62, 1.0))
    wall_mat = Material("painted_wall", rgba=(0.68, 0.70, 0.64, 1.0))
    shadow = Material("dark_recess", rgba=(0.08, 0.09, 0.09, 1.0))
    blue = Material("blue_supply_box", rgba=(0.05, 0.21, 0.58, 1.0))
    teal = Material("teal_supply_box", rgba=(0.02, 0.55, 0.50, 1.0))
    yellow = Material("yellow_label", rgba=(0.98, 0.76, 0.12, 1.0))
    orange = Material("orange_label", rgba=(0.95, 0.36, 0.09, 1.0))
    green = Material("green_label", rgba=(0.12, 0.58, 0.20, 1.0))

    cabinet = model.part("cabinet_body")

    # A thin wall plane, rear plate, side/top/bottom walls, and front flanges make
    # the shallow metal cabinet read as a wall-hung, open-front box.
    cabinet.visual(
        Box((0.42, 0.006, 0.45)),
        origin=Origin(xyz=(0.0, -0.003, 0.205)),
        material=wall_mat,
        name="wall_panel",
    )
    cabinet.visual(
        Box((0.300, 0.008, 0.350)),
        origin=Origin(xyz=(0.0, 0.004, 0.175)),
        material=steel,
        name="back_panel",
    )
    cabinet.visual(
        Box((0.012, 0.120, 0.350)),
        origin=Origin(xyz=(-0.144, 0.060, 0.175)),
        material=steel,
        name="left_wall",
    )
    cabinet.visual(
        Box((0.012, 0.120, 0.350)),
        origin=Origin(xyz=(0.144, 0.060, 0.175)),
        material=steel,
        name="right_wall",
    )
    cabinet.visual(
        Box((0.300, 0.120, 0.012)),
        origin=Origin(xyz=(0.0, 0.060, 0.344)),
        material=steel,
        name="top_wall",
    )
    cabinet.visual(
        Box((0.300, 0.120, 0.012)),
        origin=Origin(xyz=(0.0, 0.060, 0.006)),
        material=steel,
        name="bottom_wall",
    )
    cabinet.visual(
        Box((0.012, 0.008, 0.350)),
        origin=Origin(xyz=(-0.150, 0.122, 0.175)),
        material=steel,
        name="left_front_lip",
    )
    cabinet.visual(
        Box((0.012, 0.008, 0.350)),
        origin=Origin(xyz=(0.150, 0.122, 0.175)),
        material=steel,
        name="right_front_lip",
    )
    cabinet.visual(
        Box((0.300, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, 0.122, 0.344)),
        material=steel,
        name="top_front_lip",
    )
    cabinet.visual(
        Box((0.300, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, 0.122, 0.006)),
        material=steel,
        name="bottom_front_lip",
    )

    # Two real shelves divide the interior into three stocked levels.
    shelf_zs = [0.124, 0.238]
    for i, z in enumerate(shelf_zs):
        cabinet.visual(
            Box((0.288, 0.105, 0.008)),
            origin=Origin(xyz=(0.0, 0.063, z)),
            material=steel,
            name=f"shelf_{i}",
        )

    # Wall mounting ears above the cabinet emphasize that the box is hung.
    for i, x in enumerate((-0.095, 0.095)):
        cabinet.visual(
            Box((0.030, 0.006, 0.035)),
            origin=Origin(xyz=(x, -0.001, 0.363)),
            material=metal,
            name=f"hanger_tab_{i}",
        )
        cabinet.visual(
            Cylinder(radius=0.006, length=0.002),
            origin=Origin(xyz=(x, 0.003, 0.370), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=shadow,
            name=f"hanger_hole_{i}",
        )

    # Stationary hinge knuckles and a small striker plate on the cabinet frame.
    for name, z in (("hinge_barrel_bottom", 0.055), ("hinge_barrel_top", 0.295)):
        cabinet.visual(
            Cylinder(radius=0.005, length=0.070),
            origin=Origin(xyz=(-0.156, 0.131, z)),
            material=metal,
            name=name,
        )
    for name, z in (("hinge_leaf_bottom", 0.055), ("hinge_leaf_top", 0.295)):
        cabinet.visual(
            Box((0.006, 0.014, 0.045)),
            origin=Origin(xyz=(-0.153, 0.124, z)),
            material=metal,
            name=name,
        )
    cabinet.visual(
        Box((0.012, 0.004, 0.030)),
        origin=Origin(xyz=(0.145, 0.119, 0.070)),
        material=metal,
        name="latch_striker",
    )

    # Interior supplies emitted with loops: narrow cartons, bottles, and stacked
    # glove boxes.  Each item sits directly on a shelf or on another item.
    upper_colors = [yellow, red, green, teal, orange]
    upper_top = shelf_zs[1] + 0.004
    for i in range(9):
        x = -0.105 + i * 0.026
        h = 0.078
        cabinet.visual(
            Box((0.020, 0.032, h)),
            origin=Origin(xyz=(x, 0.032, upper_top + h / 2.0)),
            material=steel,
            name=f"supply_box_upper_{i}",
        )
        cabinet.visual(
            Box((0.004, 0.001, h * 0.86)),
            origin=Origin(xyz=(x - 0.004, 0.0485, upper_top + h / 2.0)),
            material=upper_colors[i % len(upper_colors)],
            name=f"supply_label_upper_{i}",
        )
    for i in range(5):
        x = 0.035 + i * 0.024
        cabinet.visual(
            Cylinder(radius=0.007, length=0.060),
            origin=Origin(xyz=(x, 0.086, upper_top + 0.030)),
            material=Material(f"white_bottle_{i}", rgba=(0.96, 0.96, 0.90, 1.0)),
            name=f"supply_bottle_{i}",
        )
        cabinet.visual(
            Cylinder(radius=0.0075, length=0.008),
            origin=Origin(xyz=(x, 0.086, upper_top + 0.064)),
            material=upper_colors[(i + 2) % len(upper_colors)],
            name=f"bottle_cap_{i}",
        )

    middle_top = shelf_zs[0] + 0.004
    for i in range(8):
        x = -0.100 + i * 0.028
        h = 0.070
        cabinet.visual(
            Box((0.022, 0.038, h)),
            origin=Origin(xyz=(x, 0.042, middle_top + h / 2.0)),
            material=steel,
            name=f"supply_box_middle_{i}",
        )
        cabinet.visual(
            Box((0.018, 0.001, 0.009)),
            origin=Origin(xyz=(x, 0.0608, middle_top + h - 0.014)),
            material=upper_colors[(i + 1) % len(upper_colors)],
            name=f"supply_band_middle_{i}",
        )

    lower_top = 0.012
    for row in range(2):
        for col in range(3):
            x = -0.078 + col * 0.078
            z_base = lower_top + row * 0.045
            cabinet.visual(
                Box((0.068, 0.036, 0.045)),
                origin=Origin(xyz=(x, 0.040, z_base + 0.0225)),
                material=blue if (row + col) % 2 == 0 else teal,
                name=f"supply_box_lower_{row}_{col}",
            )
            cabinet.visual(
                Box((0.058, 0.001, 0.010)),
                origin=Origin(xyz=(x, 0.0585, z_base + 0.027)),
                material=steel,
                name=f"supply_label_lower_{row}_{col}",
            )

    door = model.part("door")
    door_width = 0.312

    # Door part frame is on the vertical hinge axis.  At q=0 the panel extends
    # along local +X and lies just in front of the cabinet front flanges.
    door.visual(
        Box((0.018, 0.010, 0.330)),
        origin=Origin(xyz=(0.014, 0.0, 0.175)),
        material=steel,
        name="left_stile",
    )
    door.visual(
        Box((0.018, 0.010, 0.330)),
        origin=Origin(xyz=(door_width - 0.009, 0.0, 0.175)),
        material=steel,
        name="right_stile",
    )
    door.visual(
        Box((door_width - 0.018, 0.010, 0.024)),
        origin=Origin(xyz=((door_width + 0.018) / 2.0, 0.0, 0.334)),
        material=steel,
        name="top_rail",
    )
    door.visual(
        Box((door_width - 0.018, 0.010, 0.024)),
        origin=Origin(xyz=((door_width + 0.018) / 2.0, 0.0, 0.016)),
        material=steel,
        name="bottom_rail",
    )
    door.visual(
        Box((0.260, 0.004, 0.300)),
        origin=Origin(xyz=(door_width / 2.0, 0.004, 0.176)),
        material=glass,
        name="glass_pane",
    )
    door.visual(
        Box((0.116, 0.004, 0.132)),
        origin=Origin(xyz=(door_width / 2.0, 0.007, 0.180)),
        material=steel,
        name="cross_panel",
    )
    door.visual(
        Box((0.024, 0.0015, 0.092)),
        origin=Origin(xyz=(door_width / 2.0, 0.009, 0.180)),
        material=red,
        name="red_cross_vertical",
    )
    door.visual(
        Box((0.092, 0.0015, 0.024)),
        origin=Origin(xyz=(door_width / 2.0, 0.009, 0.180)),
        material=red,
        name="red_cross_horizontal",
    )

    # Door-side hinge barrel, pull handle, and round latch/lock.
    door.visual(
        Cylinder(radius=0.005, length=0.104),
        origin=Origin(xyz=(0.0, 0.0, 0.175)),
        material=metal,
        name="hinge_barrel_middle",
    )
    for i, z in enumerate((0.132, 0.218)):
        door.visual(
            Cylinder(radius=0.0025, length=0.022),
            origin=Origin(xyz=(door_width - 0.038, 0.016, z), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"handle_post_{i}",
        )
    door.visual(
        Cylinder(radius=0.0032, length=0.092),
        origin=Origin(xyz=(door_width - 0.038, 0.028, 0.175)),
        material=metal,
        name="pull_handle",
    )
    door.visual(
        Cylinder(radius=0.009, length=0.008),
        origin=Origin(xyz=(door_width - 0.036, 0.009, 0.070), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="round_latch",
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(-0.156, 0.131, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet_body")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("body_to_door")

    ctx.check(
        "single hinged front door",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and hinge.motion_limits is not None
        and hinge.motion_limits.lower == 0.0
        and hinge.motion_limits.upper >= 1.5,
        details=f"hinge={hinge}",
    )

    shelf_names = [v.name for v in cabinet.visuals if v.name and v.name.startswith("shelf_")]
    supply_names = [v.name for v in cabinet.visuals if v.name and v.name.startswith("supply_")]
    ctx.check("two horizontal interior shelves", len(shelf_names) == 2, details=str(shelf_names))
    ctx.check("loop emitted interior supplies", len(supply_names) >= 25, details=str(len(supply_names)))
    ctx.check(
        "door carries red cross decal",
        door.get_visual("red_cross_vertical") is not None
        and door.get_visual("red_cross_horizontal") is not None,
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            door,
            cabinet,
            axis="y",
            positive_elem="right_stile",
            negative_elem="right_front_lip",
            min_gap=0.0,
            max_gap=0.002,
            name="closed door sits just proud of cabinet front",
        )
        ctx.expect_overlap(
            door,
            cabinet,
            axes="x",
            elem_a="top_rail",
            elem_b="top_front_lip",
            min_overlap=0.25,
            name="closed door spans the cabinet opening",
        )
        closed_aabb = ctx.part_element_world_aabb(door, elem="right_stile")

    with ctx.pose({hinge: 1.35}):
        open_aabb = ctx.part_element_world_aabb(door, elem="right_stile")
        ctx.expect_gap(
            door,
            cabinet,
            axis="y",
            positive_elem="right_stile",
            negative_elem="right_front_lip",
            min_gap=0.12,
            name="opened door swings outward from wall",
        )

    ctx.check(
        "positive hinge motion opens the free edge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][1] > closed_aabb[1][1] + 0.15,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
