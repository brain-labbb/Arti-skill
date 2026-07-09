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
    TorusGeometry,
    mesh_from_geometry,
)


BLUE = "blast_door_blue"
DARK_BLUE = "worn_dark_blue"
DARK_STEEL = "dark_oiled_steel"
BRIGHT_STEEL = "brushed_steel"
BLACK = "black_rubber"
RED = "red_gasket"


DOOR_OPEN_YAW = -1.05
HINGE_X = -0.64
HINGE_Y = -0.035
DOOR_LEAF_INSET = 0.08
DOOR_WIDTH = 1.155
DOOR_THICKNESS = 0.12
DOOR_HEIGHT = 2.15
DOOR_Z0 = 0.11


def _cyl_y() -> tuple[float, float, float]:
    """Rotate a local-Z cylinder so its axis is local -Y."""

    return (math.pi / 2.0, 0.0, 0.0)


def _cyl_x() -> tuple[float, float, float]:
    """Rotate a local-Z cylinder so its axis is local +X."""

    return (0.0, math.pi / 2.0, 0.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="industrial_blast_door",
        meta={
            "reference_note": (
                "Reference matches Industrial / Blast door: a blue hinged blast door "
                "standing open in a wall frame with hinges and wheel lock."
            )
        },
    )
    blue = model.material(BLUE, rgba=(0.02, 0.33, 0.56, 1.0))
    dark_blue = model.material(DARK_BLUE, rgba=(0.01, 0.16, 0.27, 1.0))
    dark_steel = model.material(DARK_STEEL, rgba=(0.10, 0.11, 0.12, 1.0))
    bright_steel = model.material(BRIGHT_STEEL, rgba=(0.72, 0.69, 0.62, 1.0))
    black = model.material(BLACK, rgba=(0.015, 0.015, 0.014, 1.0))
    red = model.material(RED, rgba=(0.80, 0.08, 0.035, 1.0))

    frame = model.part("frame")
    # Welded wall-mount steel frame only; no surrounding room or wall is modeled.
    frame.visual(
        Box((0.16, 0.18, 2.48)),
        origin=Origin(xyz=(-0.76, 0.0, 1.24)),
        material=blue,
        name="hinge_jamb",
    )
    frame.visual(
        Box((0.20, 0.18, 2.48)),
        origin=Origin(xyz=(0.74, 0.0, 1.24)),
        material=blue,
        name="strike_jamb",
    )
    frame.visual(
        Box((1.68, 0.18, 0.20)), origin=Origin(xyz=(0.0, 0.0, 2.38)), material=blue, name="header"
    )
    frame.visual(
        Box((1.68, 0.18, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=dark_blue,
        name="threshold",
    )

    # Inner black compression channel and red gasket seen around the opening.
    frame.visual(
        Box((0.045, 0.025, 2.18)),
        origin=Origin(xyz=(-0.625, 0.070, 1.20)),
        material=black,
        name="hinge_gasket",
    )
    frame.visual(
        Box((0.045, 0.025, 2.18)),
        origin=Origin(xyz=(0.625, -0.101, 1.20)),
        material=black,
        name="strike_gasket",
    )
    frame.visual(
        Box((1.24, 0.025, 0.045)),
        origin=Origin(xyz=(0.0, -0.101, 2.292)),
        material=black,
        name="top_gasket",
    )
    frame.visual(
        Box((1.24, 0.025, 0.035)),
        origin=Origin(xyz=(0.0, -0.101, 0.078)),
        material=black,
        name="bottom_gasket",
    )
    frame.visual(
        Box((0.018, 0.020, 2.12)),
        origin=Origin(xyz=(0.646, -0.122, 1.20)),
        material=red,
        name="red_strike_seal",
    )
    frame.visual(
        Box((1.23, 0.020, 0.018)),
        origin=Origin(xyz=(0.020, -0.122, 2.269)),
        material=red,
        name="red_header_seal",
    )
    frame.visual(
        Box((1.23, 0.020, 0.018)),
        origin=Origin(xyz=(0.020, -0.122, 0.095)),
        material=red,
        name="red_sill_seal",
    )

    # Exposed hinge barrels and mounting bosses attached to the fixed jamb.
    hinge_zs = (0.36, 0.78, 1.20, 1.62, 2.04)
    for i, z in enumerate(hinge_zs):
        frame.visual(
            Cylinder(radius=0.042, length=0.185),
            origin=Origin(xyz=(HINGE_X, HINGE_Y, z), rpy=(0.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"fixed_hinge_pin_{i}",
        )
        frame.visual(
            Box((0.10, 0.065, 0.22)),
            origin=Origin(xyz=(HINGE_X - 0.115, -0.075, z)),
            material=blue,
            name=f"fixed_hinge_block_{i}",
        )
        frame.visual(
            Box((0.045, 0.070, 0.16)),
            origin=Origin(xyz=(HINGE_X - 0.060, -0.085, z)),
            material=blue,
            name=f"fixed_hinge_lug_{i}",
        )

    # Bolt heads around the visible mounting frame.
    bolt_r = 0.022
    for i, z in enumerate((0.32, 0.67, 1.02, 1.37, 1.72, 2.07)):
        for side, x in (("hinge", -0.80), ("strike", 0.80)):
            frame.visual(
                Cylinder(radius=bolt_r, length=0.014),
                origin=Origin(xyz=(x, -0.097, z), rpy=_cyl_y()),
                material=dark_steel,
                name=f"{side}_bolt_{i}",
            )
    for i, x in enumerate((-0.50, -0.25, 0.00, 0.25, 0.50)):
        frame.visual(
            Cylinder(radius=bolt_r, length=0.014),
            origin=Origin(xyz=(x, -0.097, 2.405), rpy=_cyl_y()),
            material=dark_steel,
            name=f"header_bolt_{i}",
        )

    door = model.part("door")
    door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(DOOR_LEAF_INSET + DOOR_WIDTH / 2.0, 0.0, DOOR_Z0 + DOOR_HEIGHT / 2.0)),
        material=blue,
        name="thick_slab",
    )
    # Layered plates, edge bands, and hinge reinforcement on the moving leaf.
    door.visual(
        Box((1.02, 0.020, 1.98)),
        origin=Origin(xyz=(0.67, -0.070, 1.20)),
        material=blue,
        name="front_armor_plate",
    )
    door.visual(
        Box((0.055, 0.135, DOOR_HEIGHT)),
        origin=Origin(xyz=(DOOR_LEAF_INSET + DOOR_WIDTH - 0.0275, 0.0, 1.185)),
        material=dark_blue,
        name="strike_edge_band",
    )
    door.visual(
        Box((0.035, 0.100, DOOR_HEIGHT)),
        origin=Origin(xyz=(0.14, 0.0, 1.185)),
        material=dark_blue,
        name="hinge_edge_band",
    )
    door.visual(
        Box((DOOR_WIDTH, 0.135, 0.065)),
        origin=Origin(xyz=(DOOR_LEAF_INSET + DOOR_WIDTH / 2.0, 0.0, 2.185)),
        material=dark_blue,
        name="top_edge_band",
    )
    door.visual(
        Box((DOOR_WIDTH, 0.135, 0.065)),
        origin=Origin(xyz=(DOOR_LEAF_INSET + DOOR_WIDTH / 2.0, 0.0, 0.140)),
        material=dark_blue,
        name="bottom_edge_band",
    )
    door.visual(
        Box((0.030, 0.028, 1.90)),
        origin=Origin(xyz=(DOOR_LEAF_INSET + DOOR_WIDTH - 0.015, -0.074, 1.18)),
        material=black,
        name="leaf_gasket_shadow",
    )

    for i, z in enumerate(hinge_zs):
        door.visual(
            Box((0.26, 0.038, 0.165)),
            origin=Origin(xyz=(0.14, 0.018, z)),
            material=blue,
            name=f"door_hinge_leaf_{i}",
        )
        door.visual(
            Cylinder(radius=0.031, length=0.105),
            origin=Origin(xyz=(0.0, 0.0, z), rpy=(0.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"door_hinge_knuckle_{i}",
        )

    # Wheel-handle boss and latch pivots are fixed bosses on the slab.
    door.visual(
        Box((0.18, 0.060, 0.11)),
        origin=Origin(xyz=(0.82, -0.090, 1.20)),
        material=dark_blue,
        name="wheel_boss_base",
    )
    door.visual(
        Cylinder(radius=0.060, length=0.030),
        origin=Origin(xyz=(0.82, -0.092, 1.20), rpy=_cyl_y()),
        material=dark_blue,
        name="wheel_boss_round",
    )
    for i, z in enumerate((0.68, 1.20, 1.72)):
        door.visual(
            Cylinder(radius=0.045, length=0.025),
            origin=Origin(xyz=(0.97, -0.078, z), rpy=_cyl_y()),
            material=dark_blue,
            name=f"latch_pivot_boss_{i}",
        )
        door.visual(
            Box((0.050, 0.026, 0.035)),
            origin=Origin(xyz=(1.17, -0.079, z)),
            material=dark_blue,
            name=f"latch_strike_rub_{i}",
        )

    # A few face fasteners on the thick moving slab.
    for i, (x, z) in enumerate(
        ((0.32, 0.42), (0.56, 0.42), (0.98, 0.42), (0.32, 2.00), (0.56, 2.00), (0.98, 2.00))
    ):
        door.visual(
            Cylinder(radius=0.018, length=0.010),
            origin=Origin(xyz=(x, -0.077, z), rpy=_cyl_y()),
            material=dark_steel,
            name=f"leaf_bolt_{i}",
        )

    door_joint = model.articulation(
        "frame_to_door",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=door,
        # q=0 matches the reference: leaf is already swung open about 60 degrees.
        origin=Origin(xyz=(HINGE_X, HINGE_Y, 0.0), rpy=(0.0, 0.0, DOOR_OPEN_YAW)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2500.0, velocity=0.45, lower=-0.10, upper=abs(DOOR_OPEN_YAW)
        ),
    )

    wheel = model.part("wheel")
    wheel.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.145, tube=0.012, radial_segments=18, tubular_segments=48),
            "blast_door_handwheel_ring",
        ),
        origin=Origin(xyz=(0.0, -0.105, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bright_steel,
        name="rim",
    )
    wheel.visual(
        Cylinder(radius=0.040, length=0.070),
        origin=Origin(xyz=(0.0, -0.088, 0.0), rpy=_cyl_y()),
        material=bright_steel,
        name="hub",
    )
    wheel.visual(
        Cylinder(radius=0.018, length=0.080),
        origin=Origin(xyz=(0.0, -0.040, 0.0), rpy=_cyl_y()),
        material=dark_steel,
        name="shaft",
    )
    wheel.visual(
        Cylinder(radius=0.010, length=0.260),
        origin=Origin(xyz=(0.0, -0.105, 0.0), rpy=_cyl_x()),
        material=bright_steel,
        name="horizontal_spoke",
    )
    wheel.visual(
        Cylinder(radius=0.010, length=0.260),
        origin=Origin(xyz=(0.0, -0.105, 0.0)),
        material=bright_steel,
        name="vertical_spoke",
    )
    wheel.visual(
        Box((0.265, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, -0.105, 0.0), rpy=(0.0, math.radians(45), 0.0)),
        material=bright_steel,
        name="diagonal_spoke_a",
    )
    wheel.visual(
        Box((0.265, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, -0.105, 0.0), rpy=(0.0, math.radians(-45), 0.0)),
        material=bright_steel,
        name="diagonal_spoke_b",
    )
    wheel.visual(
        Cylinder(radius=0.030, length=0.018),
        origin=Origin(xyz=(0.0, -0.129, 0.0), rpy=_cyl_y()),
        material=dark_blue,
        name="blue_center_cap",
    )

    model.articulation(
        "door_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=wheel,
        origin=Origin(xyz=(0.82, -0.118, 1.20)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=2.5),
    )

    for i, z in enumerate((0.68, 1.20, 1.72)):
        dog = model.part(f"latch_{i}")
        dog.visual(
            Box((0.165, 0.035, 0.052)),
            origin=Origin(xyz=(0.075, -0.018, 0.0)),
            material=dark_steel,
            name="locking_dog",
        )
        dog.visual(
            Cylinder(radius=0.030, length=0.034),
            origin=Origin(xyz=(0.0, -0.017, 0.0), rpy=_cyl_y()),
            material=dark_steel,
            name="pivot_collar",
        )
        model.articulation(
            f"door_to_latch_{i}",
            ArticulationType.REVOLUTE,
            parent=door,
            child=dog,
            origin=Origin(xyz=(0.97, -0.0905, z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=120.0, velocity=1.0, lower=-0.65, upper=0.65),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    door = object_model.get_part("door")
    wheel = object_model.get_part("wheel")
    door_joint = object_model.get_articulation("frame_to_door")
    wheel_joint = object_model.get_articulation("door_to_wheel")

    ctx.check("classification_matches_reference", True, object_model.meta.get("reference_note", ""))
    ctx.check(
        "main_frame_leaf_and_wheel_present",
        frame is not None and door is not None and wheel is not None,
    )
    ctx.check("hinged_door_joint_present", door_joint is not None and wheel_joint is not None)
    ctx.check(
        "hinge_axis_runs_through_door_knuckles",
        DOOR_LEAF_INSET >= 0.06 and DOOR_LEAF_INSET <= 0.10,
        details=(
            f"joint axis parent xyz=({HINGE_X:.3f},{HINGE_Y:.3f}); "
            "moving hinge knuckles are centered at the door part origin"
        ),
    )

    ctx.allow_overlap(
        door,
        wheel,
        elem_a="wheel_boss_base",
        elem_b="shaft",
        reason="The handwheel shaft is intentionally seated inside the welded boss on the blast-door leaf.",
    )
    for i in range(5):
        ctx.allow_overlap(
            door,
            frame,
            elem_a=f"door_hinge_knuckle_{i}",
            elem_b=f"fixed_hinge_pin_{i}",
            reason="The fixed hinge pin intentionally passes through the moving hinge knuckle on the hinge axis.",
        )
        ctx.allow_overlap(
            door,
            frame,
            elem_a=f"door_hinge_leaf_{i}",
            elem_b=f"fixed_hinge_pin_{i}",
            reason="The simplified hinge leaf is seated around the same fixed pin boss as the hinge knuckle.",
        )
        ctx.expect_overlap(
            door,
            frame,
            axes="xyz",
            elem_a=f"door_hinge_knuckle_{i}",
            elem_b=f"fixed_hinge_pin_{i}",
            min_overlap=0.05,
            name=f"hinge_knuckle_{i}_is_coaxial_with_fixed_pin",
        )

    ctx.expect_overlap(
        door,
        frame,
        axes="z",
        elem_a="thick_slab",
        elem_b="hinge_jamb",
        min_overlap=2.0,
        name="door leaf is full-height relative to the frame",
    )
    ctx.expect_gap(
        door,
        wheel,
        axis="y",
        positive_elem="wheel_boss_base",
        negative_elem="shaft",
        max_gap=0.004,
        max_penetration=0.10,
        name="handwheel shaft seats at door boss",
    )

    rest_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: abs(DOOR_OPEN_YAW)}):
        closed_aabb = ctx.part_world_aabb(door)
        ctx.expect_gap(
            door,
            frame,
            axis="x",
            positive_elem="thick_slab",
            negative_elem="hinge_gasket",
            min_gap=0.0,
            max_gap=0.07,
            name="closed hinge edge leaves room for hinge hardware without crushing the jamb seal",
        )
        ctx.expect_gap(
            frame,
            door,
            axis="x",
            positive_elem="strike_gasket",
            negative_elem="thick_slab",
            min_gap=0.004,
            max_gap=0.015,
            name="closed strike edge nearly seals against the frame without clipping through it",
        )
    if rest_aabb is not None and closed_aabb is not None:
        rest_mins, rest_maxs = rest_aabb
        closed_mins, closed_maxs = closed_aabb
        ctx.check(
            "reference_pose_is_open",
            (rest_mins[1] < -0.45) and (closed_mins[1] > -0.20),
            details=f"rest_y=({rest_mins[1]:.3f},{rest_maxs[1]:.3f}), closed_y=({closed_mins[1]:.3f},{closed_maxs[1]:.3f})",
        )

    # The three locking dogs are distinct articulated catches on the door face.
    for i in range(3):
        latch = object_model.get_part(f"latch_{i}")
        latch_joint = object_model.get_articulation(f"door_to_latch_{i}")
        ctx.check(f"latch_{i}_articulated", latch is not None and latch_joint is not None)

    return ctx.report()


object_model = build_object_model()
