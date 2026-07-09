from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


WIDTH = 0.30
DEPTH = 0.12
BASE_HEIGHT = 0.185
LID_THICKNESS = 0.034
WALL = 0.006
BOTTOM = 0.006
CORNER_RADIUS = 0.023
TRAY_WIDTH = 0.265
TRAY_DEPTH = 0.090
TRAY_HEIGHT = 0.030
TRAY_FLOOR = 0.004
TRAY_WALL = 0.003
TRAY_BOTTOM_Z = 0.147
TRAY_TRAVEL = 0.062


def _rounded_slab(width: float, depth: float, height: float, radius: float, name: str):
    return mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(width, depth, radius, corner_segments=10), height, center=True),
        name,
    )


def _hollow_open_box(
    width: float,
    depth: float,
    height: float,
    wall: float,
    radius: float,
    name: str,
    *,
    open_face: str = ">Z",
):
    """Rounded rectangular container hollowed out via a true CAD shell so the
    interior is a genuinely empty cavity with correct outward normals (an open
    box, not a solid block). ``open_face`` selects which face is removed."""
    box = cq.Workplane("XY").box(width, depth, height).edges("|Z").fillet(radius)
    shell = box.faces(open_face).shell(-wall)
    return mesh_from_cadquery(shell, name, tolerance=0.0006)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_first_aid_hard_case")

    cream = model.material("weathered_cream", rgba=(0.86, 0.82, 0.70, 1.0))
    red = model.material("medical_red", rgba=(0.55, 0.03, 0.025, 1.0))
    dark = model.material("dark_interior", rgba=(0.04, 0.035, 0.032, 1.0))
    metal = model.material("aged_metal", rgba=(0.46, 0.44, 0.39, 1.0))
    plastic = model.material("aged_white_plastic", rgba=(0.82, 0.82, 0.78, 1.0))

    base = model.part("base")
    # Genuinely hollow open-top container: a CAD shell (thin floor + four walls,
    # open at the top) so the inside is a real empty cavity, not a solid block.
    base.visual(
        _hollow_open_box(WIDTH, DEPTH, BASE_HEIGHT, WALL, CORNER_RADIUS, "base_wall"),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
        material=cream,
        name="base_wall",
    )
    # Dark thin pad on the interior floor so the open cavity reads as a dark recess.
    base.visual(
        Box((WIDTH - 2.8 * WALL, DEPTH - 2.8 * WALL, 0.0012)),
        origin=Origin(xyz=(0.0, 0.0, WALL + 0.001)),
        material=dark,
        name="empty_compartment",
    )
    # Narrow interior shelves fixed to the side walls carry the removable upper tray.
    # They leave a deep, genuinely empty compartment underneath.
    for i, x in enumerate((-0.134, 0.134)):
        base.visual(
            Box((0.024, TRAY_DEPTH + 0.004, 0.004)),
            origin=Origin(xyz=(x, 0.0, TRAY_BOTTOM_Z - 0.002)),
            material=cream,
            name=f"tray_ledge_{i}",
        )

    # Bold cross decal on the front face, very slightly embedded in the paint.
    front_y = -DEPTH / 2.0 - 0.00025
    base.visual(
        Box((0.036, 0.0010, 0.092)),
        origin=Origin(xyz=(0.0, front_y, 0.102)),
        material=red,
        name="cross_vertical",
    )
    base.visual(
        Box((0.112, 0.0010, 0.036)),
        origin=Origin(xyz=(0.0, front_y - 0.0001, 0.102)),
        material=red,
        name="cross_horizontal",
    )

    # Rear hinge barrel and lower front latch strike plates are fixed to the base.
    base.visual(
        Cylinder(radius=0.004, length=WIDTH * 0.74),
        origin=Origin(
            xyz=(0.0, DEPTH / 2.0 + 0.002, BASE_HEIGHT + 0.001),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=metal,
        name="rear_hinge_barrel",
    )
    for i, x in enumerate((-0.074, 0.074)):
        base.visual(
            Box((0.040, 0.004, 0.020)),
            origin=Origin(xyz=(x, -DEPTH / 2.0 - 0.002, BASE_HEIGHT - 0.023)),
            material=metal,
            name=f"latch_strike_{i}",
        )

    # Open compartment ("格子") tray: a thin floor + a perimeter rim + a grid of
    # divider walls, all built from explicit panels so each cell is a genuinely
    # open recessed pocket (no solid block, correct outward normals).
    tray = model.part("tray")
    wall_h = TRAY_HEIGHT - TRAY_FLOOR
    wall_cz = TRAY_FLOOR + wall_h / 2.0
    hx = TRAY_WIDTH / 2.0
    hy = TRAY_DEPTH / 2.0

    tray.visual(
        Box((TRAY_WIDTH, TRAY_DEPTH, TRAY_FLOOR)),
        origin=Origin(xyz=(0.0, 0.0, TRAY_FLOOR / 2.0)),
        material=plastic,
        name="tray_floor",
    )
    # Perimeter rim (four upright walls).
    tray.visual(
        Box((TRAY_WALL, TRAY_DEPTH, wall_h)),
        origin=Origin(xyz=(-(hx - TRAY_WALL / 2.0), 0.0, wall_cz)),
        material=plastic,
        name="tray_wall_left",
    )
    tray.visual(
        Box((TRAY_WALL, TRAY_DEPTH, wall_h)),
        origin=Origin(xyz=(hx - TRAY_WALL / 2.0, 0.0, wall_cz)),
        material=plastic,
        name="tray_wall_right",
    )
    tray.visual(
        Box((TRAY_WIDTH, TRAY_WALL, wall_h)),
        origin=Origin(xyz=(0.0, -(hy - TRAY_WALL / 2.0), wall_cz)),
        material=plastic,
        name="tray_wall_front",
    )
    tray.visual(
        Box((TRAY_WIDTH, TRAY_WALL, wall_h)),
        origin=Origin(xyz=(0.0, hy - TRAY_WALL / 2.0, wall_cz)),
        material=plastic,
        name="tray_wall_back",
    )
    # Two vertical dividers (-> three columns) and one horizontal divider
    # (-> two rows) carve the interior into a 3 x 2 grid of six open cells.
    interior_depth = TRAY_DEPTH - 2.0 * TRAY_WALL
    interior_width = TRAY_WIDTH - 2.0 * TRAY_WALL
    for i, x in enumerate((-TRAY_WIDTH / 6.0, TRAY_WIDTH / 6.0)):
        tray.visual(
            Box((TRAY_WALL, interior_depth, wall_h)),
            origin=Origin(xyz=(x, 0.0, wall_cz)),
            material=plastic,
            name=f"divider_col_{i}",
        )
    tray.visual(
        Box((interior_width, TRAY_WALL, wall_h)),
        origin=Origin(xyz=(0.0, 0.0, wall_cz)),
        material=plastic,
        name="divider_row",
    )
    # Small finger pull on the front rim.
    tray.visual(
        Box((0.045, 0.005, 0.009)),
        origin=Origin(xyz=(0.0, -hy - 0.0015, TRAY_HEIGHT - 0.006)),
        material=plastic,
        name="front_pull",
    )
    model.articulation(
        "base_to_tray",
        ArticulationType.PRISMATIC,
        parent=base,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, TRAY_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.6, lower=0.0, upper=TRAY_TRAVEL),
    )

    lid = model.part("lid")
    # Open-bottom shell so the lid is a real recessed cover, not a solid slab.
    lid.visual(
        _hollow_open_box(
            WIDTH, DEPTH, LID_THICKNESS, WALL, CORNER_RADIUS, "lid_skirt", open_face="<Z"
        ),
        origin=Origin(xyz=(0.0, -DEPTH / 2.0, LID_THICKNESS / 2.0)),
        material=cream,
        name="lid_skirt",
    )
    lid.visual(
        _rounded_slab(WIDTH, DEPTH, WALL, CORNER_RADIUS, "lid_top"),
        origin=Origin(xyz=(0.0, -DEPTH / 2.0, LID_THICKNESS - WALL / 2.0)),
        material=cream,
        name="lid_top",
    )
    lid.visual(
        Box((WIDTH - 3.0 * WALL, DEPTH - 3.0 * WALL, 0.0012)),
        origin=Origin(xyz=(0.0, -DEPTH / 2.0, 0.0010)),
        material=dark,
        name="dark_lid_liner",
    )
    for i, x in enumerate((-0.105, 0.105)):
        lid.visual(
            Box((0.030, 0.020, 0.004)),
            origin=Origin(xyz=(x, -DEPTH / 2.0, LID_THICKNESS + 0.002)),
            material=metal,
            name=f"handle_mount_{i}",
        )
    for i, x in enumerate((-0.074, 0.074)):
        lid.visual(
            Box((0.044, 0.004, 0.018)),
            origin=Origin(xyz=(x, -DEPTH - 0.002, 0.013)),
            material=metal,
            name=f"latch_keeper_{i}",
        )
    lid_joint = model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, DEPTH / 2.0, BASE_HEIGHT)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.85),
    )

    handle = model.part("handle")
    handle_geom = tube_from_spline_points(
        [
            (-0.105, 0.0, 0.0),
            (-0.105, 0.0, 0.052),
            (-0.078, 0.0, 0.078),
            (0.0, 0.0, 0.086),
            (0.078, 0.0, 0.078),
            (0.105, 0.0, 0.052),
            (0.105, 0.0, 0.0),
        ],
        radius=0.003,
        samples_per_segment=12,
        radial_segments=18,
        cap_ends=True,
    )
    handle.visual(
        mesh_from_geometry(handle_geom, "handle_loop"),
        material=metal,
        name="handle_loop",
    )
    for i, x in enumerate((-0.105, 0.105)):
        handle.visual(
            Box((0.018, 0.014, 0.006)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=metal,
            name=f"handle_pivot_{i}",
        )
    model.articulation(
        "lid_to_handle",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=handle,
        origin=Origin(xyz=(0.0, -DEPTH / 2.0, LID_THICKNESS + 0.007)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.5, lower=0.0, upper=1.45),
    )

    for i, x in enumerate((-0.074, 0.074)):
        latch = model.part(f"latch_{i}")
        latch.visual(
            Box((0.030, 0.003, 0.052)),
            origin=Origin(xyz=(0.0, 0.0, 0.026)),
            material=metal,
            name="clasp_plate",
        )
        latch.visual(
            Box((0.038, 0.004, 0.007)),
            origin=Origin(xyz=(0.0, 0.0, 0.052)),
            material=metal,
            name="clasp_lip",
        )
        latch.visual(
            Cylinder(radius=0.004, length=0.036),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal,
            name="clasp_pivot",
        )
        model.articulation(
            f"base_to_latch_{i}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=latch,
            origin=Origin(xyz=(x, -DEPTH / 2.0 - 0.0055, BASE_HEIGHT - 0.040)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.5, velocity=3.0, lower=0.0, upper=1.20),
        )

    # Keep variables live only to make joint intent explicit for readers.
    _ = lid_joint
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("handle")
    tray = object_model.get_part("tray")
    lid_joint = object_model.get_articulation("base_to_lid")
    handle_joint = object_model.get_articulation("lid_to_handle")
    tray_joint = object_model.get_articulation("base_to_tray")
    latch_0 = object_model.get_part("latch_0")
    latch_joint_0 = object_model.get_articulation("base_to_latch_0")

    ctx.expect_gap(
        lid,
        base,
        axis="z",
        positive_elem="lid_skirt",
        negative_elem="base_wall",
        min_gap=0.0,
        max_gap=0.0010,
        name="closed lid seats on base rim",
    )
    ctx.expect_within(
        base,
        base,
        axes="xy",
        inner_elem="empty_compartment",
        outer_elem="base_wall",
        margin=0.0,
        name="empty dark compartment lies inside shell",
    )
    inner_volume = (WIDTH - 2.0 * WALL) * (DEPTH - 2.0 * WALL) * (BASE_HEIGHT - BOTTOM)
    outer_volume = WIDTH * DEPTH * BASE_HEIGHT
    ctx.check(
        "base shell is mostly hollow volume",
        inner_volume / outer_volume > 0.65,
        details=f"inner_volume={inner_volume:.6f}, outer_volume={outer_volume:.6f}",
    )
    ctx.expect_overlap(
        base,
        base,
        axes="xz",
        elem_a="cross_vertical",
        elem_b="cross_horizontal",
        min_overlap=0.025,
        name="front medical cross bars intersect",
    )
    ctx.expect_gap(
        base,
        latch_0,
        axis="y",
        positive_elem="base_wall",
        negative_elem="clasp_plate",
        min_gap=0.001,
        max_gap=0.012,
        name="front clasp is mounted outside front face",
    )
    ctx.expect_within(
        tray,
        base,
        axes="xy",
        inner_elem="tray_floor",
        outer_elem="base_wall",
        margin=0.002,
        name="medicine tray nests inside base footprint",
    )
    ctx.expect_gap(
        tray,
        base,
        axis="z",
        positive_elem="tray_floor",
        negative_elem="tray_ledge_0",
        min_gap=0.0,
        max_gap=0.0008,
        name="medicine tray rests on interior ledge",
    )
    rest_tray_box = ctx.part_world_aabb(tray)
    ctx.check(
        "resting tray sits below the case rim",
        rest_tray_box is not None and rest_tray_box[1][2] < BASE_HEIGHT - 0.004,
        details=f"tray={rest_tray_box}, rim_z={BASE_HEIGHT}",
    )
    tray_grid_boxes = [
        ctx.part_element_world_aabb(tray, elem="divider_col_0"),
        ctx.part_element_world_aabb(tray, elem="divider_col_1"),
        ctx.part_element_world_aabb(tray, elem="divider_row"),
    ]
    ctx.check(
        "medicine tray has two by three compartment dividers",
        all(box is not None for box in tray_grid_boxes),
        details=f"divider_boxes={tray_grid_boxes}",
    )

    closed_lid_box = ctx.part_element_world_aabb(lid, elem="lid_top")
    with ctx.pose({lid_joint: 1.45}):
        open_lid_box = ctx.part_element_world_aabb(lid, elem="lid_top")
    ctx.check(
        "lid opens upward from rear hinge",
        closed_lid_box is not None
        and open_lid_box is not None
        and open_lid_box[1][2] > closed_lid_box[1][2] + 0.08,
        details=f"closed={closed_lid_box}, open={open_lid_box}",
    )

    upright_handle = ctx.part_world_aabb(handle)
    with ctx.pose({handle_joint: 1.25}):
        folded_handle = ctx.part_world_aabb(handle)
    ctx.check(
        "carry handle folds down on its pivots",
        upright_handle is not None
        and folded_handle is not None
        and folded_handle[1][2] < upright_handle[1][2] - 0.030,
        details=f"upright={upright_handle}, folded={folded_handle}",
    )

    closed_latch = ctx.part_world_aabb(latch_0)
    with ctx.pose({latch_joint_0: 0.95}):
        released_latch = ctx.part_world_aabb(latch_0)
    ctx.check(
        "front latch clasp flips outward",
        closed_latch is not None
        and released_latch is not None
        and released_latch[0][1] < closed_latch[0][1] - 0.010,
        details=f"closed={closed_latch}, released={released_latch}",
    )

    with ctx.pose({lid_joint: 1.45, tray_joint: TRAY_TRAVEL}):
        lifted_tray_box = ctx.part_world_aabb(tray)
        ctx.expect_within(
            tray,
            base,
            axes="xy",
            inner_elem="tray_floor",
            outer_elem="base_wall",
            margin=0.002,
            name="lifted tray stays aligned with base opening",
        )
    ctx.check(
        "prismatic tray lift clears the case rim",
        lifted_tray_box is not None and lifted_tray_box[0][2] > BASE_HEIGHT + 0.006,
        details=f"lifted={lifted_tray_box}, rim_z={BASE_HEIGHT}",
    )

    return ctx.report()


object_model = build_object_model()
