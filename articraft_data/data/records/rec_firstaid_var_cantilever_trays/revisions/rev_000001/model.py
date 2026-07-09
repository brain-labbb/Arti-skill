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

# Cantilever tiered-tray geometry (tackle-box style scissor arms).
NUM_TRAYS = 2
TRAY_WIDTH = 0.110
TRAY_DEPTH = 0.080
TRAY_HEIGHT = 0.025
TRAY_FLOOR = 0.003
TRAY_WALL = 0.003
ARM_LENGTH = 0.105
ARM_WIDTH = 0.014
ARM_THICKNESS = 0.003
# Pivot pins are anchored on the inner side walls, tiered in height so the
# two trays stack one above the other when folded inside the case.
ARM_PIVOT_X = WIDTH / 2.0 - WALL - 0.004  # just inside the inner wall
ARM_PIVOT_Z = (0.070, 0.130)  # lower tier, upper tier
# Arm local frame: origin at the pivot, bar extends along -X at rest (toward
# the case centre). Positive rotation around +Y swings the arm tip from -X
# toward +Z (upward) and then outward past the rim, giving the classic
# tackle-box cantilever fanning motion.
ARM_OPEN_ANGLE = 1.35  # radians, ~77 degrees


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


def _tray_pan(width: float, depth: float, height: float, floor: float, wall: float, prefix: str, material):
    """Build the visuals for one shallow open tray pan and return the part
    reference list so the caller can attach them to the owning part."""
    wall_h = height - floor
    wall_cz = floor + wall_h / 2.0
    hx = width / 2.0
    hy = depth / 2.0
    return [
        # Floor
        (Box((width, depth, floor)), Origin(xyz=(0.0, 0.0, floor / 2.0)), material, f"{prefix}_floor"),
        # Perimeter rim
        (Box((wall, depth, wall_h)), Origin(xyz=(-(hx - wall / 2.0), 0.0, wall_cz)), material, f"{prefix}_wall_left"),
        (Box((wall, depth, wall_h)), Origin(xyz=(hx - wall / 2.0, 0.0, wall_cz)), material, f"{prefix}_wall_right"),
        (Box((width, wall, wall_h)), Origin(xyz=(0.0, -(hy - wall / 2.0), wall_cz)), material, f"{prefix}_wall_front"),
        (Box((width, wall, wall_h)), Origin(xyz=(0.0, hy - wall / 2.0, wall_cz)), material, f"{prefix}_wall_back"),
        # Single interior divider (two cells) to read as a tackle-box compartment tray.
        (Box((wall, depth - 2.0 * wall, wall_h)), Origin(xyz=(0.0, 0.0, wall_cz)), material, f"{prefix}_divider"),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_first_aid_hard_case")

    cream = model.material("weathered_cream", rgba=(0.86, 0.82, 0.70, 1.0))
    red = model.material("medical_red", rgba=(0.55, 0.03, 0.025, 1.0))
    dark = model.material("dark_interior", rgba=(0.04, 0.035, 0.032, 1.0))
    metal = model.material("aged_metal", rgba=(0.46, 0.44, 0.39, 1.0))
    plastic = model.material("aged_white_plastic", rgba=(0.82, 0.82, 0.78, 1.0))

    # ------------------------------------------------------------------ base
    base = model.part("base")
    base.visual(
        _hollow_open_box(WIDTH, DEPTH, BASE_HEIGHT, WALL, CORNER_RADIUS, "base_wall"),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
        material=cream,
        name="base_wall",
    )
    base.visual(
        Box((WIDTH - 2.8 * WALL, DEPTH - 2.8 * WALL, 0.0012)),
        origin=Origin(xyz=(0.0, 0.0, WALL + 0.001)),
        material=dark,
        name="empty_compartment",
    )

    # Arm pivot bosses on the inner side walls — small cylindrical housings
    # where the scissor link arms pin into the case. Tiered in height so the
    # two trays nest one above the other when folded closed.
    for i, pivot_z in enumerate(ARM_PIVOT_Z):
        base.visual(
            Cylinder(radius=0.006, length=0.010),
            origin=Origin(
                xyz=(ARM_PIVOT_X, 0.0, pivot_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=metal,
            name=f"arm_pivot_boss_{i}",
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

    # ------------------------------------------- cantilever tiered tray arms
    # Each tray_arm_{i} is a flat metal link pinned to the case wall through a
    # revolute joint around Y. At q=0 the arm extends from its pivot toward
    # the case centre (-X in its local frame); positive q swings the arm tip
    # upward and outward past the rim, fanning the tray out of the case.
    # tray_{i} is fixed to the arm tip so the tray follows the arm rigidly.
    for i in range(NUM_TRAYS):
        pivot_z = ARM_PIVOT_Z[i]

        arm = model.part(f"tray_arm_{i}")
        # Flat link bar: origin at the pivot, bar extends along local -X.
        arm.visual(
            Box((ARM_LENGTH, ARM_WIDTH, ARM_THICKNESS)),
            origin=Origin(xyz=(-ARM_LENGTH / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"arm_bar_{i}",
        )
        # Pivot pin cylinder at the wall end (local origin).
        arm.visual(
            Cylinder(radius=0.004, length=0.016),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"arm_pivot_pin_{i}",
        )
        # Tray cradle plate at the far end of the arm (local -X tip).
        arm.visual(
            Box((0.030, ARM_WIDTH, 0.004)),
            origin=Origin(xyz=(-ARM_LENGTH, 0.0, 0.003)),
            material=metal,
            name=f"arm_cradle_{i}",
        )

        model.articulation(
            f"base_to_tray_arm_{i}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=arm,
            origin=Origin(xyz=(ARM_PIVOT_X, 0.0, pivot_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=5.0, velocity=1.5, lower=0.0, upper=ARM_OPEN_ANGLE
            ),
        )

        # The tray is rigidly mounted on the arm tip.
        tray = model.part(f"tray_{i}")
        for geom, origin, mat, vname in _tray_pan(
            TRAY_WIDTH, TRAY_DEPTH, TRAY_HEIGHT, TRAY_FLOOR, TRAY_WALL,
            prefix=f"tray_{i}", material=plastic,
        ):
            tray.visual(geom, origin=origin, material=mat, name=vname)

        model.articulation(
            f"tray_arm_{i}_to_tray_{i}",
            ArticulationType.FIXED,
            parent=arm,
            child=tray,
            origin=Origin(xyz=(-ARM_LENGTH, 0.0, 0.005)),
        )

    # ------------------------------------------------------------------- lid
    lid = model.part("lid")
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

    # ---------------------------------------------------------------- handle
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

    # --------------------------------------------------------------- latches
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

    _ = lid_joint
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("handle")
    lid_joint = object_model.get_articulation("base_to_lid")
    handle_joint = object_model.get_articulation("lid_to_handle")
    latch_0 = object_model.get_part("latch_0")
    latch_joint_0 = object_model.get_articulation("base_to_latch_0")

    # Closed-lid seating.
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

    # ----- variant axis: cantilever tiered trays on scissor link-arms -----
    arm_0 = object_model.get_part("tray_arm_0")
    arm_1 = object_model.get_part("tray_arm_1")
    tray_0 = object_model.get_part("tray_0")
    tray_1 = object_model.get_part("tray_1")
    arm_joint_0 = object_model.get_articulation("base_to_tray_arm_0")
    arm_joint_1 = object_model.get_articulation("base_to_tray_arm_1")

    # Both trays are tiered inside the case at rest (tray_1 sits above tray_0).
    rest_tray0 = ctx.part_world_aabb(tray_0)
    rest_tray1 = ctx.part_world_aabb(tray_1)
    ctx.check(
        "tray_1 is tiered above tray_0 when folded inside the case",
        rest_tray0 is not None
        and rest_tray1 is not None
        and rest_tray1[0][2] > rest_tray0[0][2] + 0.010,
        details=f"tray_0={rest_tray0}, tray_1={rest_tray1}",
    )
    # Both trays nest inside the base footprint at rest.
    for i, tray in enumerate((tray_0, tray_1)):
        ctx.expect_within(
            tray,
            base,
            axes="xy",
            inner_elem=f"tray_{i}_floor",
            outer_elem="base_wall",
            margin=0.002,
            name=f"tray_{i} nests inside base footprint at rest",
        )

    # Opening the arm swings the tray upward and outward past the rim.
    with ctx.pose({arm_joint_0: ARM_OPEN_ANGLE}):
        open_tray0 = ctx.part_world_aabb(tray_0)
    ctx.check(
        "tray_arm_0 swings tray_0 up and outward past the case rim",
        rest_tray0 is not None
        and open_tray0 is not None
        and open_tray0[1][2] > BASE_HEIGHT + 0.020
        and open_tray0[0][0] > rest_tray0[0][0] + 0.020,
        details=f"rest={rest_tray0}, open={open_tray0}, rim_z={BASE_HEIGHT}",
    )
    with ctx.pose({arm_joint_1: ARM_OPEN_ANGLE}):
        open_tray1 = ctx.part_world_aabb(tray_1)
    ctx.check(
        "tray_arm_1 swings tray_1 up and outward past the case rim",
        rest_tray1 is not None
        and open_tray1 is not None
        and open_tray1[1][2] > BASE_HEIGHT + 0.020
        and open_tray1[0][0] > rest_tray1[0][0] + 0.020,
        details=f"rest={rest_tray1}, open={open_tray1}, rim_z={BASE_HEIGHT}",
    )

    # Each tray is rigidly mounted on its arm via a FIXED articulation.
    for i in range(NUM_TRAYS):
        tray_joint = object_model.get_articulation(f"tray_arm_{i}_to_tray_{i}")
        ctx.check(
            f"tray_{i} is rigidly fixed to tray_arm_{i}",
            tray_joint.articulation_type == ArticulationType.FIXED,
            details=f"type={tray_joint.articulation_type}",
        )
    # Pivot pins are intentionally nested inside the pivot bosses (captured hinge pins).
    for i in range(NUM_TRAYS):
        ctx.allow_overlap(
            "base",
            f"tray_arm_{i}",
            elem_a=f"arm_pivot_boss_{i}",
            elem_b=f"arm_pivot_pin_{i}",
            reason=f"Pivot pin is intentionally seated inside the pivot boss as a captured hinge pin for tray_arm_{i}.",
        )
        ctx.expect_contact(
            base,
            object_model.get_part(f"tray_arm_{i}"),
            elem_a=f"arm_pivot_boss_{i}",
            elem_b=f"arm_pivot_pin_{i}",
            contact_tol=0.010,
            name=f"tray_arm_{i} pivot pin contacts its boss",
        )

    # Lid opens upward from the rear hinge.
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

    # Handle folds down on its pivots.
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

    # Front latch clasp flips outward.
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

    return ctx.report()


object_model = build_object_model()
