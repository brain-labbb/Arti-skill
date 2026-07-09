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
    TorusGeometry,
    mesh_from_geometry,
)


STEEL_DARK = Material("dark_blast_steel", rgba=(0.28, 0.30, 0.30, 1.0))
STEEL_MID = Material("painted_gray_steel", rgba=(0.48, 0.50, 0.49, 1.0))
STEEL_LIGHT = Material("worn_light_steel", rgba=(0.72, 0.74, 0.72, 1.0))
MOUNT_BLUE = Material("blue_primed_mount_plate", rgba=(0.52, 0.68, 0.72, 1.0))
RUBBER = Material("black_compression_gasket", rgba=(0.02, 0.023, 0.022, 1.0))
HANDLE_WHITE = Material("white_grip_sleeve", rgba=(0.92, 0.91, 0.86, 1.0))

OPENING_X_MIN = -0.565
OPENING_X_MAX = 0.565
OPENING_Z_MIN = 0.245
OPENING_Z_MAX = 1.755
HINGE_X = -0.615
DOOR_PANEL_X_SHIFT = 0.000


def _add_bolt(part, name: str, x: float, y: float, z: float, radius: float = 0.018) -> None:
    """Low round bolt head, facing out from the door plane along +Y."""
    part.visual(
        Cylinder(radius=radius, length=0.014),
        origin=Origin(xyz=(x, y, z), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=STEEL_LIGHT,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="industrial_blast_door",
        meta={
            "reference_note": (
                "Image shows a wall-mounted hinged industrial blast door; no "
                "category mismatch suspected."
            )
        },
    )

    # Root: a heavy wall-mount frame with the doorway left open through the
    # center. The mount plate is segmented so the open door does not reveal a
    # solid wall behind the threshold.
    frame = model.part("frame")
    frame.visual(
        Box((0.210, 0.060, 1.85)),
        origin=Origin(xyz=(-0.670, -0.055, 1.00)),
        material=MOUNT_BLUE,
        name="mount_plate_left",
    )
    frame.visual(
        Box((0.210, 0.060, 1.85)),
        origin=Origin(xyz=(0.670, -0.055, 1.00)),
        material=MOUNT_BLUE,
        name="mount_plate_right",
    )
    frame.visual(
        Box((1.55, 0.060, 0.170)),
        origin=Origin(xyz=(0.0, -0.055, 1.840)),
        material=MOUNT_BLUE,
        name="mount_plate_top",
    )
    frame.visual(
        Box((1.55, 0.060, 0.170)),
        origin=Origin(xyz=(0.0, -0.055, 0.160)),
        material=MOUNT_BLUE,
        name="mount_plate_bottom",
    )
    frame.visual(
        Box((0.095, 0.072, 1.64)),
        origin=Origin(xyz=(-0.66, -0.006, 1.00)),
        material=STEEL_MID,
        name="left_jamb",
    )
    frame.visual(
        Box((0.095, 0.072, 1.64)),
        origin=Origin(xyz=(0.66, -0.006, 1.00)),
        material=STEEL_MID,
        name="right_jamb",
    )
    frame.visual(
        Box((1.42, 0.072, 0.095)),
        origin=Origin(xyz=(0.0, -0.006, 1.82)),
        material=STEEL_MID,
        name="top_header",
    )
    frame.visual(
        Box((1.42, 0.072, 0.095)),
        origin=Origin(xyz=(0.0, -0.006, 0.18)),
        material=STEEL_MID,
        name="bottom_sill",
    )
    frame.visual(
        Box((0.030, 0.012, 1.49)),
        origin=Origin(xyz=(OPENING_X_MIN, 0.017, 1.00)),
        material=RUBBER,
        name="gasket_left",
    )
    frame.visual(
        Box((0.030, 0.012, 1.49)),
        origin=Origin(xyz=(0.565, 0.017, 1.00)),
        material=RUBBER,
        name="gasket_right",
    )
    frame.visual(
        Box((1.14, 0.012, 0.040)),
        origin=Origin(xyz=(0.0, 0.024, 1.755)),
        material=RUBBER,
        name="gasket_top",
    )
    frame.visual(
        Box((1.14, 0.012, 0.040)),
        origin=Origin(xyz=(0.0, 0.024, 0.245)),
        material=RUBBER,
        name="gasket_bottom",
    )

    # Reference-style hinges sit in the narrow reveal gap: short pin barrels,
    # slim jamb leaves, small caps, and visible fasteners instead of a long rod.
    for hinge_name, zc in (("upper", 1.565), ("lower", 0.435)):
        frame.visual(
            Cylinder(radius=0.014, length=0.305),
            origin=Origin(xyz=(HINGE_X, 0.083, zc)),
            material=STEEL_LIGHT,
            name=f"{hinge_name}_hinge_pin",
        )
        frame.visual(
            Cylinder(radius=0.045, length=0.088),
            origin=Origin(xyz=(HINGE_X, 0.083, zc + 0.102)),
            material=STEEL_DARK,
            name=f"{hinge_name}_frame_barrel_top",
        )
        frame.visual(
            Cylinder(radius=0.045, length=0.088),
            origin=Origin(xyz=(HINGE_X, 0.083, zc - 0.102)),
            material=STEEL_DARK,
            name=f"{hinge_name}_frame_barrel_bottom",
        )
        for lug_idx, dz in enumerate((0.102, -0.102)):
            # Leaf reaches from the barrel back through the jamb face and is
            # recessed into the jamb body so it reads as mortised into the
            # frame rather than floating in front of it.
            frame.visual(
                Box((0.105, 0.100, 0.040)),
                origin=Origin(xyz=(HINGE_X - 0.054, 0.035, zc + dz)),
                material=STEEL_MID,
                name=f"{hinge_name}_jamb_leaf_{lug_idx}",
            )
            _add_bolt(
                frame,
                f"{hinge_name}_jamb_leaf_bolt_{lug_idx}",
                HINGE_X - 0.086,
                0.088,
                zc + dz,
                0.008,
            )
        frame.visual(
            Box((0.030, 0.060, 0.270)),
            origin=Origin(xyz=(HINGE_X - 0.104, 0.010, zc)),
            material=STEEL_DARK,
            name=f"{hinge_name}_jamb_leaf_backer",
        )
        frame.visual(
            Cylinder(radius=0.049, length=0.026),
            origin=Origin(xyz=(HINGE_X, 0.083, zc + 0.178)),
            material=STEEL_MID,
            name=f"{hinge_name}_barrel_top_cap",
        )
        frame.visual(
            Cylinder(radius=0.049, length=0.026),
            origin=Origin(xyz=(HINGE_X, 0.083, zc - 0.178)),
            material=STEEL_MID,
            name=f"{hinge_name}_barrel_bottom_cap",
        )

    for idx, z in enumerate((1.48, 0.52)):
        frame.visual(
            Box((0.090, 0.040, 0.070)),
            origin=Origin(xyz=(0.628, 0.050, z)),
            material=STEEL_DARK,
            name=f"dog_receiver_{idx}",
        )
        _add_bolt(frame, f"receiver_bolt_{idx}_0", 0.628, 0.077, z + 0.022, 0.010)
        _add_bolt(frame, f"receiver_bolt_{idx}_1", 0.628, 0.077, z - 0.022, 0.010)

    # Door leaf frame sits to the right of the hinge line, matching the
    # left-hinged reference door.
    door = model.part("door_leaf")
    door.visual(
        Box((1.110, 0.120, 1.450)),
        origin=Origin(xyz=(0.615 + DOOR_PANEL_X_SHIFT, 0.000, 0.000)),
        material=STEEL_MID,
        name="leaf_slab",
    )
    door.visual(
        Box((0.055, 0.026, 1.395)),
        origin=Origin(xyz=(0.085 + DOOR_PANEL_X_SHIFT, 0.073, 0.000)),
        material=STEEL_DARK,
        name="front_border_left",
    )
    door.visual(
        Box((0.055, 0.026, 1.395)),
        origin=Origin(xyz=(1.145 + DOOR_PANEL_X_SHIFT, 0.073, 0.000)),
        material=STEEL_DARK,
        name="front_border_right",
    )
    door.visual(
        Box((1.105, 0.026, 0.055)),
        origin=Origin(xyz=(0.615 + DOOR_PANEL_X_SHIFT, 0.073, 0.695)),
        material=STEEL_DARK,
        name="front_border_top",
    )
    door.visual(
        Box((1.105, 0.026, 0.055)),
        origin=Origin(xyz=(0.615 + DOOR_PANEL_X_SHIFT, 0.073, -0.695)),
        material=STEEL_DARK,
        name="front_border_bottom",
    )
    door.visual(
        Box((1.020, 0.020, 0.650)),
        origin=Origin(xyz=(0.625 + DOOR_PANEL_X_SHIFT, 0.084, 0.010)),
        material=Material("subtle_center_plate", rgba=(0.42, 0.43, 0.42, 1.0)),
        name="inset_center_plate",
    )

    for hinge_name, zc in (("upper", 0.565), ("lower", -0.565)):
        door.visual(
            Cylinder(radius=0.037, length=0.096),
            origin=Origin(xyz=(0.000, -0.006, zc)),
            material=STEEL_DARK,
            name=f"{hinge_name}_door_barrel",
        )
        # Leaf reaches all the way back to the barrel (no gap) so the
        # knuckle always reads as attached to the door leaf as it swings.
        door.visual(
            Box((0.084, 0.024, 0.042)),
            origin=Origin(xyz=(0.078, 0.010, zc + 0.034)),
            material=STEEL_MID,
            name=f"{hinge_name}_door_leaf_upper",
        )
        door.visual(
            Box((0.084, 0.024, 0.042)),
            origin=Origin(xyz=(0.078, 0.010, zc - 0.074)),
            material=STEEL_MID,
            name=f"{hinge_name}_door_leaf_lower",
        )
        for lug_idx, dz in enumerate((0.034, -0.074)):
            _add_bolt(
                door,
                f"{hinge_name}_door_leaf_bolt_{lug_idx}",
                0.114,
                0.027,
                zc + dz,
                0.007,
            )
        door.visual(
            Cylinder(radius=0.041, length=0.008),
            origin=Origin(xyz=(0.000, -0.006, zc + 0.052)),
            material=STEEL_MID,
            name=f"{hinge_name}_door_barrel_top_collar",
        )
        door.visual(
            Cylinder(radius=0.041, length=0.008),
            origin=Origin(xyz=(0.000, -0.006, zc - 0.052)),
            material=STEEL_MID,
            name=f"{hinge_name}_door_barrel_bottom_collar",
        )

    # Standoff saddles for the external locking rod and bottom operating bar.
    for idx, z in enumerate((0.47, -0.47)):
        door.visual(
            Box((0.145, 0.055, 0.080)),
            origin=Origin(xyz=(0.980 + DOOR_PANEL_X_SHIFT, 0.0725, z)),
            material=STEEL_DARK,
            name=f"rod_saddle_{idx}",
        )
        _add_bolt(door, f"rod_saddle_bolt_{idx}_0", 0.928 + DOOR_PANEL_X_SHIFT, 0.0915, z, 0.010)
        _add_bolt(door, f"rod_saddle_bolt_{idx}_1", 1.032 + DOOR_PANEL_X_SHIFT, 0.0915, z, 0.010)
    door.visual(
        Box((0.105, 0.042, 0.075)),
        origin=Origin(xyz=(0.180 + DOOR_PANEL_X_SHIFT, 0.081, -0.560)),
        material=STEEL_DARK,
        name="lower_bar_pivot_plate",
    )

    # Door bolts around the reinforced perimeter.
    bolt_positions = [
        (0.18 + DOOR_PANEL_X_SHIFT, 0.067, 0.60),
        (0.50 + DOOR_PANEL_X_SHIFT, 0.067, 0.60),
        (0.82 + DOOR_PANEL_X_SHIFT, 0.067, 0.60),
        (1.08 + DOOR_PANEL_X_SHIFT, 0.067, 0.36),
        (1.08 + DOOR_PANEL_X_SHIFT, 0.067, -0.36),
        (0.31 + DOOR_PANEL_X_SHIFT, 0.067, -0.60),
        (0.50 + DOOR_PANEL_X_SHIFT, 0.067, -0.60),
        (0.82 + DOOR_PANEL_X_SHIFT, 0.067, -0.60),
    ]
    for idx, (x, y, z) in enumerate(bolt_positions):
        _add_bolt(door, f"door_bolt_{idx}", x, y, z, 0.012)

    main_hinge = model.articulation(
        "frame_to_door",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=door,
        origin=Origin(xyz=(HINGE_X, 0.085, 1.000)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1800.0, velocity=0.45, lower=0.0, upper=1.25),
    )

    # External vertical locking rod with two dog lugs.
    lock_bar = model.part("lock_bar")
    lock_bar.visual(
        Cylinder(radius=0.018, length=1.120),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=STEEL_LIGHT,
        name="vertical_rod",
    )
    for idx, z in enumerate((0.480, -0.480)):
        lock_bar.visual(
            Box((0.155, 0.035, 0.040)),
            origin=Origin(xyz=(0.075, 0.000, z)),
            material=STEEL_DARK,
            name=f"locking_dog_{idx}",
        )
        lock_bar.visual(
            Cylinder(radius=0.030, length=0.050),
            origin=Origin(xyz=(0.0, 0.025, z), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=STEEL_DARK,
            name=f"dog_collar_{idx}",
        )
    model.articulation(
        "door_to_lock_bar",
        ArticulationType.REVOLUTE,
        parent=door,
        child=lock_bar,
        origin=Origin(xyz=(0.980 + DOOR_PANEL_X_SHIFT, 0.118, 0.000)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.0, lower=-0.35, upper=0.35),
    )

    # Round hand wheel driving the lock shaft.
    handwheel = model.part("handwheel")
    wheel_ring = TorusGeometry(radius=0.105, tube=0.008, radial_segments=16, tubular_segments=48)
    wheel_ring.rotate_x(pi / 2.0)
    handwheel.visual(
        mesh_from_geometry(wheel_ring, "handwheel_ring"),
        origin=Origin(),
        material=STEEL_LIGHT,
        name="wheel_ring",
    )
    handwheel.visual(
        Cylinder(radius=0.027, length=0.060),
        origin=Origin(xyz=(0.0, -0.034, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=STEEL_DARK,
        name="wheel_shaft",
    )
    handwheel.visual(
        Cylinder(radius=0.034, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=STEEL_DARK,
        name="wheel_hub",
    )
    handwheel.visual(
        Box((0.210, 0.014, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=STEEL_LIGHT,
        name="wheel_spoke_x",
    )
    handwheel.visual(
        Box((0.014, 0.014, 0.210)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=STEEL_LIGHT,
        name="wheel_spoke_z",
    )
    model.articulation(
        "door_to_handwheel",
        ArticulationType.REVOLUTE,
        parent=door,
        child=handwheel,
        origin=Origin(xyz=(0.735 + DOOR_PANEL_X_SHIFT, 0.158, 0.265)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=3.0, lower=-3.1416, upper=3.1416),
    )

    # Lower horizontal operating bar / latch lever.
    lower_bar = model.part("lower_bar")
    lower_bar.visual(
        Cylinder(radius=0.017, length=0.560),
        origin=Origin(xyz=(0.280, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=STEEL_LIGHT,
        name="bar_rod",
    )
    lower_bar.visual(
        Cylinder(radius=0.026, length=0.180),
        origin=Origin(xyz=(0.470, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=HANDLE_WHITE,
        name="white_grip",
    )
    lower_bar.visual(
        Cylinder(radius=0.040, length=0.040),
        origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=STEEL_DARK,
        name="bar_pivot_hub",
    )
    model.articulation(
        "door_to_lower_bar",
        ArticulationType.REVOLUTE,
        parent=door,
        child=lower_bar,
        origin=Origin(xyz=(0.180 + DOOR_PANEL_X_SHIFT, 0.134, -0.560)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=65.0, velocity=1.4, lower=-0.25, upper=0.85),
    )

    # Keep a reference to the primary articulation name in metadata for consumers.
    model.meta["primary_articulation"] = main_hinge.name
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    door = object_model.get_part("door_leaf")
    lock_bar = object_model.get_part("lock_bar")
    lower_bar = object_model.get_part("lower_bar")
    main_hinge = object_model.get_articulation("frame_to_door")
    lever_joint = object_model.get_articulation("door_to_lower_bar")

    for hinge_name in ("upper", "lower"):
        for door_elem in (
            f"{hinge_name}_door_barrel",
            f"{hinge_name}_door_barrel_top_collar",
            f"{hinge_name}_door_barrel_bottom_collar",
        ):
            ctx.allow_overlap(
                door,
                frame,
                elem_a=door_elem,
                elem_b=f"{hinge_name}_hinge_pin",
                reason="The hinge pin is intentionally seated coaxially inside the matching door barrel.",
            )
            ctx.expect_overlap(
                door,
                frame,
                axes="xy",
                elem_a=door_elem,
                elem_b=f"{hinge_name}_hinge_pin",
                min_overlap=0.020,
                name=f"{door_elem} captures hinge pin",
            )

    ctx.expect_gap(
        door,
        frame,
        axis="y",
        positive_elem="leaf_slab",
        negative_elem="gasket_left",
        min_gap=0.002,
        max_gap=0.012,
        name="closed slab sits just proud of gasket",
    )
    ctx.expect_overlap(
        door,
        frame,
        axes="xz",
        elem_a="leaf_slab",
        elem_b="gasket_left",
        min_overlap=0.004,
        name="door leaf covers the gasket line",
    )
    leaf_closed_aabb = ctx.part_element_world_aabb(door, elem="leaf_slab")
    ctx.check(
        "closed leaf covers the open doorway from the left hinge",
        leaf_closed_aabb is not None
        and leaf_closed_aabb[0][0] <= OPENING_X_MIN + 0.015
        and leaf_closed_aabb[1][0] >= OPENING_X_MAX - 0.015,
        details=f"leaf={leaf_closed_aabb}, opening_x=({OPENING_X_MIN}, {OPENING_X_MAX})",
    )
    ctx.expect_gap(
        lock_bar,
        door,
        axis="y",
        positive_elem="vertical_rod",
        negative_elem="rod_saddle_0",
        min_gap=0.0,
        max_gap=0.002,
        name="locking rod is captured just in front of its saddles",
    )

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({main_hinge: 0.85}):
        open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "main leaf swings outward on hinge",
        closed_aabb is not None
        and open_aabb is not None
        and (
            open_aabb[1][1] > closed_aabb[1][1] + 0.25 or open_aabb[0][1] < closed_aabb[0][1] - 0.25
        ),
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    blocked_plate_names: list[str] = []
    for visual in frame.visuals:
        if not (visual.name or "").startswith("mount_plate"):
            continue
        if not isinstance(visual.geometry, Box):
            continue
        x_size, _, z_size = visual.geometry.size
        x_center, _, z_center = visual.origin.xyz
        x_min = x_center - x_size / 2.0
        x_max = x_center + x_size / 2.0
        z_min = z_center - z_size / 2.0
        z_max = z_center + z_size / 2.0
        overlaps_opening = (
            x_min < OPENING_X_MAX
            and x_max > OPENING_X_MIN
            and z_min < OPENING_Z_MAX
            and z_max > OPENING_Z_MIN
        )
        if overlaps_opening:
            blocked_plate_names.append(str(visual.name))
    ctx.check(
        "doorway opening is not blocked by a solid backing plate",
        not blocked_plate_names,
        details=f"blocking mount plates={blocked_plate_names}",
    )

    rest_bar = ctx.part_world_aabb(lower_bar)
    with ctx.pose({lever_joint: 0.65}):
        raised_bar = ctx.part_world_aabb(lower_bar)
    ctx.check(
        "lower latch lever rotates upward",
        rest_bar is not None
        and raised_bar is not None
        and raised_bar[1][2] > rest_bar[1][2] + 0.10,
        details=f"rest={rest_bar}, raised={raised_bar}",
    )

    return ctx.report()


object_model = build_object_model()
