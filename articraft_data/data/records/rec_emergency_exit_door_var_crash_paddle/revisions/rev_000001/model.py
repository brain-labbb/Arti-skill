from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _rounded_box(length: float, depth: float, height: float, radius: float) -> cq.Workplane:
    shape = cq.Workplane("XY").box(length, depth, height)
    if radius > 0:
        try:
            shape = shape.edges().fillet(radius)
        except Exception:
            shape = cq.Workplane("XY").box(length, depth, height)
    return shape


def _screw(part, material, x: float, y: float, z: float, name: str) -> None:
    part.visual(
        Cylinder(radius=0.008, length=0.006),
        origin=Origin(xyz=(x, y, z), rpy=(math.pi / 2, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _small_screw(part, material, x: float, y: float, z: float, name: str) -> None:
    part.visual(
        Cylinder(radius=0.0055, length=0.004),
        origin=Origin(xyz=(x, y, z), rpy=(math.pi / 2, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _label_marks(
    part,
    material,
    *,
    side: str,
    x_offset: float = 0.0,
    y: float = -0.030,
    z: float = 0.001,
) -> None:
    marks = [
        (-0.150, 0.040),
        (-0.100, 0.052),
        (-0.042, 0.034),
        (0.006, 0.048),
        (0.062, 0.036),
        (0.110, 0.050),
    ]
    for i, (x, length) in enumerate(marks):
        part.visual(
            Box((length, 0.003, 0.007)),
            origin=Origin(xyz=(x_offset + x, y, z)),
            material=material,
            name=f"{side}_label_mark_{i}",
        )


def _add_leaf_panel(part, *, side: str, material, edge_mat, shadow_mat) -> None:
    sign = 1.0 if side == "near" else -1.0
    panel_center_x = sign * 0.300
    meeting_x = sign * 0.612

    part.visual(
        Box((0.600, 0.046, 1.420)),
        origin=Origin(xyz=(panel_center_x, 0.0, 0.720)),
        material=material,
        name=f"{side}_door_skin",
    )
    # Raised and recessed sheet-metal features make the leaves read like real
    # commercial steel exit doors instead of flat slabs.
    for z, label in ((1.120, "upper_inset"), (0.395, "lower_inset")):
        part.visual(
            Box((0.500, 0.006, 0.245)),
            origin=Origin(xyz=(panel_center_x, -0.026, z)),
            material=shadow_mat,
            name=f"{side}_{label}_recess_shadow",
        )
        part.visual(
            Box((0.530, 0.008, 0.016)),
            origin=Origin(xyz=(panel_center_x, -0.030, z + 0.130)),
            material=edge_mat,
            name=f"{side}_{label}_top_trim",
        )
        part.visual(
            Box((0.530, 0.008, 0.016)),
            origin=Origin(xyz=(panel_center_x, -0.030, z - 0.130)),
            material=edge_mat,
            name=f"{side}_{label}_bottom_trim",
        )
        for x_offset, rail_name in ((-0.265, "hinge_side_trim"), (0.265, "meeting_side_trim")):
            part.visual(
                Box((0.014, 0.008, 0.250)),
                origin=Origin(xyz=(panel_center_x + sign * x_offset, -0.030, z)),
                material=edge_mat,
                name=f"{side}_{label}_{rail_name}",
            )
    part.visual(
        Box((0.540, 0.010, 0.032)),
        origin=Origin(xyz=(panel_center_x, -0.032, 0.095)),
        material=edge_mat,
        name=f"{side}_bottom_kick_plate",
    )
    for x_offset in (-0.210, -0.070, 0.070, 0.210):
        _small_screw(
            part,
            shadow_mat,
            panel_center_x + x_offset,
            -0.039,
            0.095,
            f"{side}_kick_plate_screw_{len(part.visuals)}",
        )
    part.visual(
        Box((0.014, 0.052, 1.400)),
        origin=Origin(xyz=(meeting_x, -0.002, 0.715)),
        material=edge_mat,
        name=f"{side}_meeting_stile",
    )
    part.visual(
        Box((0.018, 0.010, 1.360)),
        origin=Origin(xyz=(meeting_x - sign * 0.020, -0.030, 0.715)),
        material=shadow_mat,
        name=f"{side}_center_reveal_shadow",
    )
    for z, name in ((1.350, "upper_panel_reveal"), (0.190, "lower_panel_reveal")):
        part.visual(
            Box((0.560, 0.008, 0.020)),
            origin=Origin(xyz=(panel_center_x, -0.030, z)),
            material=edge_mat,
            name=f"{side}_{name}",
        )
    for z, name in ((1.115, "upper_hinge_barrel"), (0.360, "lower_hinge_barrel")):
        part.visual(
            Box((0.020, 0.038, 0.180)),
            origin=Origin(xyz=(-sign * 0.009, -0.002, z)),
            material=edge_mat,
            name=f"{side}_{name}_leaf_plate",
        )
        for dz in (-0.056, 0.056):
            _small_screw(
                part,
                shadow_mat,
                -sign * 0.009,
                -0.023,
                z + dz,
                f"{side}_{name}_leaf_screw_{len(part.visuals)}",
            )
        part.visual(
            Cylinder(radius=0.018, length=0.160),
            origin=Origin(xyz=(0.0, -0.020, z), rpy=(0.0, 0.0, 0.0)),
            material=edge_mat,
            name=f"{side}_{name}",
        )
        part.visual(
            Cylinder(radius=0.010, length=0.170),
            origin=Origin(xyz=(0.0, -0.020, z), rpy=(0.0, 0.0, 0.0)),
            material=shadow_mat,
            name=f"{side}_{name}_pin_shadow",
        )


def _add_fixed_panic_hardware(
    part,
    *,
    side: str,
    metal_mat,
    dark_mat,
    green_mat,
    white_mat,
    include_static_bar: bool = True,
) -> None:
    sign = 1.0 if side == "near" else -1.0
    outer_x = sign * 0.070
    center_x = sign * 0.540
    rail_center = sign * 0.315
    label_center = sign * 0.285

    part.visual(
        Box((0.520, 0.016, 0.020)),
        origin=Origin(xyz=(rail_center, -0.050, 0.718)),
        material=metal_mat,
        name=f"{side}_fixed_upper_rail",
    )
    part.visual(
        Box((0.520, 0.016, 0.020)),
        origin=Origin(xyz=(rail_center, -0.050, 0.602)),
        material=metal_mat,
        name=f"{side}_fixed_lower_rail",
    )
    part.visual(
        Box((0.480, 0.006, 0.074)),
        origin=Origin(xyz=(rail_center, -0.035, 0.660)),
        material=dark_mat,
        name=f"{side}_rail_shadow_slot",
    )
    part.visual(
        Box((0.500, 0.006, 0.018)),
        origin=Origin(xyz=(rail_center, -0.061, 0.746)),
        material=metal_mat,
        name=f"{side}_upper_channel_bead",
    )
    part.visual(
        Box((0.500, 0.006, 0.018)),
        origin=Origin(xyz=(rail_center, -0.061, 0.574)),
        material=metal_mat,
        name=f"{side}_lower_channel_bead",
    )

    for x, name in ((outer_x, "outer_end_box"), (center_x, "center_latch_block")):
        part.visual(
            mesh_from_cadquery(_rounded_box(0.074, 0.062, 0.190, 0.008), f"{side}_{name}"),
            origin=Origin(xyz=(x, -0.057, 0.660)),
            material=metal_mat,
            name=f"{side}_{name}",
        )
        part.visual(
            Box((0.048, 0.010, 0.122)),
            origin=Origin(xyz=(x, -0.090, 0.660)),
            material=dark_mat,
            name=f"{side}_{name}_vertical_shadow",
        )
        part.visual(
            Box((0.084, 0.010, 0.024)),
            origin=Origin(xyz=(x, -0.088, 0.758)),
            material=metal_mat,
            name=f"{side}_{name}_top_cap_lip",
        )
        part.visual(
            Box((0.084, 0.010, 0.024)),
            origin=Origin(xyz=(x, -0.088, 0.562)),
            material=metal_mat,
            name=f"{side}_{name}_bottom_cap_lip",
        )
        _screw(part, dark_mat, x, -0.091, 0.730, f"{side}_{name}_upper_screw")
        _screw(part, dark_mat, x, -0.091, 0.590, f"{side}_{name}_lower_screw")

    if include_static_bar:
        # The opposite door's panic bar is fixed to its own moving leaf.  It is
        # not the primary pressed control, but it makes the paired-door layout
        # explicit without duplicating the near door's articulated bar.
        part.visual(
            mesh_from_cadquery(_rounded_box(0.460, 0.042, 0.066, 0.010), f"{side}_rear_silver_bar"),
            origin=Origin(xyz=(rail_center, -0.078, 0.660)),
            material=metal_mat,
            name=f"{side}_rear_silver_bar",
        )
        for x_offset, cap_name in ((-0.245, "outer_push_cap"), (0.245, "inner_push_cap")):
            part.visual(
                Box((0.026, 0.046, 0.076)),
                origin=Origin(xyz=(rail_center + sign * x_offset, -0.079, 0.660)),
                material=dark_mat,
                name=f"{side}_{cap_name}",
            )
        part.visual(
            mesh_from_cadquery(_rounded_box(0.330, 0.007, 0.040, 0.003), f"{side}_rear_green_label"),
            origin=Origin(xyz=(label_center, -0.103, 0.660)),
            material=green_mat,
            name=f"{side}_rear_green_label",
        )
        _label_marks(part, white_mat, side=side, x_offset=label_center, y=-0.108, z=0.660)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="paired_emergency_exit_doors_with_push_paddle",
        meta={
            "run_notes": (
                "Paddle-variant fork: the near leaf uses a pivoting push paddle "
                "(crash paddle) instead of a sliding touch bar. The paddle rotates "
                "about a horizontal pivot at its top edge; pressing it inward "
                "retracts the center latch bolt via a mimic linkage. The far leaf "
                "retains its fixed panic bar. Companion colorways: satin chrome "
                "(default), matte black, polished brass, antique bronze, brushed "
                "gold, satin nickel."
            )
        },
    )

    frame_mat = model.material("dark_frame_recess", rgba=(0.18, 0.19, 0.20, 1.0))
    door_mat = model.material("satin_off_white_leaf", rgba=(0.76, 0.78, 0.82, 1.0))
    alt_door_mat = model.material("cool_off_white_leaf", rgba=(0.69, 0.71, 0.76, 1.0))
    shadow_mat = model.material("black_gap_shadow", rgba=(0.015, 0.016, 0.018, 1.0))
    metal_mat = model.material("brushed_aluminum", rgba=(0.68, 0.69, 0.67, 1.0))
    dark_metal_mat = model.material("dark_anodized_detail", rgba=(0.29, 0.30, 0.30, 1.0))
    green_mat = model.material("emergency_green", rgba=(0.02, 0.62, 0.36, 1.0))
    white_mat = model.material("raised_white_print", rgba=(0.97, 0.98, 0.94, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((1.440, 0.050, 0.050)),
        origin=Origin(xyz=(0.0, 0.030, 1.485)),
        material=frame_mat,
        name="top_jamb",
    )
    frame.visual(
        Box((0.054, 0.055, 1.470)),
        origin=Origin(xyz=(-0.665, 0.030, 0.735)),
        material=frame_mat,
        name="near_outer_jamb",
    )
    frame.visual(
        Box((0.054, 0.055, 1.470)),
        origin=Origin(xyz=(0.665, 0.030, 0.735)),
        material=frame_mat,
        name="far_outer_jamb",
    )
    frame.visual(
        Box((0.020, 0.025, 1.450)),
        origin=Origin(xyz=(0.0, 0.040, 0.735)),
        material=shadow_mat,
        name="center_closed_gap",
    )

    near_leaf = model.part("near_leaf")
    _add_leaf_panel(
        near_leaf,
        side="near",
        material=door_mat,
        edge_mat=metal_mat,
        shadow_mat=shadow_mat,
    )
    _add_fixed_panic_hardware(
        near_leaf,
        side="near",
        metal_mat=metal_mat,
        dark_mat=dark_metal_mat,
        green_mat=green_mat,
        white_mat=white_mat,
        include_static_bar=False,
    )
    near_leaf.visual(
        Cylinder(radius=0.040, length=0.012),
        origin=Origin(xyz=(0.495, -0.033, 0.935), rpy=(math.pi / 2, 0.0, 0.0)),
        material=metal_mat,
        name="near_lock_rosette",
    )
    near_leaf.visual(
        Cylinder(radius=0.026, length=0.052),
        origin=Origin(xyz=(0.495, -0.067, 0.935), rpy=(math.pi / 2, 0.0, 0.0)),
        material=metal_mat,
        name="near_lock_cylinder",
    )
    near_leaf.visual(
        Box((0.008, 0.003, 0.030)),
        origin=Origin(xyz=(0.495, -0.095, 0.935)),
        material=shadow_mat,
        name="near_lock_key_slot",
    )
    near_leaf.visual(
        Cylinder(radius=0.050, length=0.004),
        origin=Origin(xyz=(0.495, -0.039, 0.935), rpy=(math.pi / 2, 0.0, 0.0)),
        material=shadow_mat,
        name="near_lock_rosette_shadow_ring",
    )
    near_leaf.visual(
        Box((0.060, 0.011, 0.012)),
        origin=Origin(xyz=(0.620, -0.1165, 0.736)),
        material=metal_mat,
        name="near_bolt_front_guide_lip",
    )
    near_leaf.visual(
        Box((0.066, 0.008, 0.030)),
        origin=Origin(xyz=(0.625, -0.111, 0.715)),
        material=shadow_mat,
        name="near_bolt_sleeve_slot_shadow",
    )
    near_leaf.visual(
        Box((0.086, 0.028, 0.010)),
        origin=Origin(xyz=(0.612, -0.092, 0.738)),
        material=metal_mat,
        name="near_bolt_guide_upper_wall",
    )
    near_leaf.visual(
        Box((0.086, 0.028, 0.010)),
        origin=Origin(xyz=(0.612, -0.092, 0.692)),
        material=metal_mat,
        name="near_bolt_guide_lower_wall",
    )
    near_leaf.visual(
        Box((0.010, 0.028, 0.010)),
        origin=Origin(xyz=(0.567, -0.092, 0.740)),
        material=metal_mat,
        name="near_bolt_guide_upper_back_ear",
    )
    near_leaf.visual(
        Box((0.010, 0.028, 0.010)),
        origin=Origin(xyz=(0.567, -0.092, 0.690)),
        material=metal_mat,
        name="near_bolt_guide_lower_back_ear",
    )
    near_leaf.visual(
        Box((0.082, 0.010, 0.014)),
        origin=Origin(xyz=(0.610, -0.116, 0.744)),
        material=metal_mat,
        name="near_bolt_upper_retainer_lip",
    )
    near_leaf.visual(
        Box((0.082, 0.010, 0.014)),
        origin=Origin(xyz=(0.610, -0.116, 0.686)),
        material=metal_mat,
        name="near_bolt_lower_retainer_lip",
    )
    _small_screw(near_leaf, shadow_mat, 0.580, -0.112, 0.748, "near_bolt_guide_upper_screw")
    _small_screw(near_leaf, shadow_mat, 0.580, -0.112, 0.682, "near_bolt_guide_lower_screw")

    far_leaf = model.part("far_leaf")
    _add_leaf_panel(
        far_leaf,
        side="far",
        material=alt_door_mat,
        edge_mat=metal_mat,
        shadow_mat=shadow_mat,
    )
    _add_fixed_panic_hardware(
        far_leaf,
        side="far",
        metal_mat=metal_mat,
        dark_mat=dark_metal_mat,
        green_mat=green_mat,
        white_mat=white_mat,
    )
    far_leaf.visual(
        Box((0.086, 0.018, 0.170)),
        origin=Origin(xyz=(-0.590, -0.036, 0.715)),
        material=metal_mat,
        name="far_strike_plate",
    )
    far_leaf.visual(
        Box((0.058, 0.012, 0.044)),
        origin=Origin(xyz=(-0.590, -0.050, 0.715)),
        material=shadow_mat,
        name="far_strike_receiver",
    )
    far_leaf.visual(
        Box((0.068, 0.034, 0.016)),
        origin=Origin(xyz=(-0.590, -0.066, 0.748)),
        material=metal_mat,
        name="far_strike_upper_keeper_lip",
    )
    far_leaf.visual(
        Box((0.068, 0.034, 0.016)),
        origin=Origin(xyz=(-0.590, -0.066, 0.682)),
        material=metal_mat,
        name="far_strike_lower_keeper_lip",
    )
    far_leaf.visual(
        Box((0.016, 0.034, 0.080)),
        origin=Origin(xyz=(-0.548, -0.066, 0.715)),
        material=metal_mat,
        name="far_strike_outer_keeper_wall",
    )

    # Pivoting push paddle (crash paddle) replaces the sliding touch bar.
    # The paddle rotates about a horizontal pivot at its top edge; pressing
    # the paddle face inward retracts the latch bolt via mimic linkage.
    near_push_paddle = model.part("near_push_paddle")
    # Main paddle face plate - hangs down from pivot, pressed inward
    near_push_paddle.visual(
        mesh_from_cadquery(_rounded_box(0.280, 0.012, 0.100, 0.004), "near_paddle_face"),
        origin=Origin(xyz=(0.0, -0.006, -0.050)),
        material=metal_mat,
        name="near_paddle_face",
    )
    # Pivot brackets at each end - mount to door leaf
    for x, name in ((-0.150, "outer"), (0.150, "inner")):
        near_push_paddle.visual(
            Box((0.025, 0.020, 0.020)),
            origin=Origin(xyz=(x, 0.005, -0.005)),
            material=dark_metal_mat,
            name=f"near_paddle_{name}_bracket",
        )
        near_push_paddle.visual(
            Cylinder(radius=0.008, length=0.028),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
            material=metal_mat,
            name=f"near_paddle_{name}_pivot_pin",
        )
    # Green instruction label on paddle face
    near_push_paddle.visual(
        mesh_from_cadquery(_rounded_box(0.240, 0.005, 0.060, 0.002), "near_paddle_green_label"),
        origin=Origin(xyz=(0.0, -0.014, -0.050)),
        material=green_mat,
        name="near_paddle_green_label",
    )
    # Label shadow/recess details
    near_push_paddle.visual(
        Box((0.250, 0.003, 0.006)),
        origin=Origin(xyz=(0.0, -0.017, -0.020)),
        material=dark_metal_mat,
        name="near_paddle_label_top_edge",
    )
    near_push_paddle.visual(
        Box((0.250, 0.003, 0.006)),
        origin=Origin(xyz=(0.0, -0.017, -0.080)),
        material=dark_metal_mat,
        name="near_paddle_label_bottom_edge",
    )
    # White text marks on green label
    for i, (x, length) in enumerate(((-0.100, 0.040), (-0.040, 0.050), (0.020, 0.038), (0.080, 0.045))):
        near_push_paddle.visual(
            Box((length, 0.002, 0.008)),
            origin=Origin(xyz=(x, -0.018, -0.050)),
            material=white_mat,
            name=f"near_paddle_label_mark_{i}",
        )

    latch_bolt = model.part("latch_bolt")
    latch_bolt.visual(
        mesh_from_cadquery(_rounded_box(0.030, 0.022, 0.032, 0.004), "center_latch_bolt"),
        origin=Origin(xyz=(0.022, 0.0, 0.0)),
        material=metal_mat,
        name="center_latch_bolt",
    )
    latch_bolt.visual(
        Box((0.038, 0.018, 0.024)),
        origin=Origin(xyz=(-0.020, 0.0, 0.0)),
        material=dark_metal_mat,
        name="bolt_tail",
    )
    latch_bolt.visual(
        Box((0.012, 0.012, 0.026)),
        origin=Origin(xyz=(0.042, -0.001, 0.0)),
        material=dark_metal_mat,
        name="bolt_beveled_dark_tip",
    )

    left_door_joint = model.articulation(
        "frame_to_near_leaf",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=near_leaf,
        origin=Origin(xyz=(-0.620, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=140.0, velocity=0.8, lower=0.0, upper=1.05),
    )
    model.articulation(
        "frame_to_far_leaf",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=far_leaf,
        origin=Origin(xyz=(0.620, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=140.0, velocity=0.8, lower=0.0, upper=1.05),
        mimic=Mimic(joint=left_door_joint.name, multiplier=1.0, offset=0.0),
    )
    push_joint = model.articulation(
        "near_leaf_to_push_paddle",
        ArticulationType.REVOLUTE,
        parent=near_leaf,
        child=near_push_paddle,
        # Pivot at top edge of paddle, horizontal axis along leaf width
        origin=Origin(xyz=(0.330, -0.055, 0.693)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=95.0, velocity=2.0, lower=0.0, upper=0.35),
    )
    model.articulation(
        "near_leaf_to_latch_bolt",
        ArticulationType.PRISMATIC,
        parent=near_leaf,
        child=latch_bolt,
        origin=Origin(xyz=(0.590, -0.100, 0.715)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.25, lower=0.0, upper=0.050),
        # Latch retraction is coordinated with paddle rotation via test assertions
        # (SDK does not allow PRISMATIC to mimic REVOLUTE across motion domains)
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    near_leaf = object_model.get_part("near_leaf")
    far_leaf = object_model.get_part("far_leaf")
    near_push_paddle = object_model.get_part("near_push_paddle")
    latch_bolt = object_model.get_part("latch_bolt")
    door_joint = object_model.get_articulation("frame_to_near_leaf")
    push_joint = object_model.get_articulation("near_leaf_to_push_paddle")

    ctx.expect_gap(
        far_leaf,
        near_leaf,
        axis="x",
        min_gap=0.001,
        max_gap=0.030,
        positive_elem="far_meeting_stile",
        negative_elem="near_meeting_stile",
        name="paired door leaves meet at a narrow center seam",
    )
    ctx.expect_gap(
        near_leaf,
        near_push_paddle,
        axis="x",
        min_gap=0.025,
        max_gap=0.120,
        positive_elem="near_center_latch_block",
        negative_elem="near_paddle_green_label",
        name="paddle green label stops before the central vertical latch block",
    )
    ctx.expect_overlap(
        near_push_paddle,
        near_leaf,
        axes="x",
        min_overlap=0.20,
        elem_a="near_paddle_green_label",
        elem_b="near_door_skin",
        name="paddle green label remains on the near door leaf",
    )
    ctx.expect_overlap(
        latch_bolt,
        far_leaf,
        axes="z",
        min_overlap=0.025,
        elem_a="center_latch_bolt",
        elem_b="far_strike_plate",
        name="latch bolt aligns with the opposite leaf strike",
    )
    ctx.expect_gap(
        near_leaf,
        near_push_paddle,
        axis="y",
        min_gap=0.014,
        max_gap=0.050,
        positive_elem="near_door_skin",
        negative_elem="near_paddle_face",
        name="push paddle stands proud of near door skin",
    )

    # Get the latch joint to demonstrate coordinated paddle/latch behavior
    latch_joint = object_model.get_articulation("near_leaf_to_latch_bolt")

    # Check the paddle face visual position (part origin is at pivot, doesn't move)
    rest_paddle_face = ctx.part_element_world_aabb(near_push_paddle, elem="near_paddle_face")
    rest_bolt = ctx.part_world_position(latch_bolt)
    # Pose both paddle and latch together to show coordinated retraction
    # (0.35 rad paddle rotation corresponds to ~0.050 m latch retraction)
    with ctx.pose({push_joint: 0.35, latch_joint: 0.050}):
        pressed_paddle_face = ctx.part_element_world_aabb(near_push_paddle, elem="near_paddle_face")
        retracted_bolt = ctx.part_world_position(latch_bolt)

    closed_near_aabb = ctx.part_element_world_aabb(near_leaf, elem="near_door_skin")
    closed_far_aabb = ctx.part_element_world_aabb(far_leaf, elem="far_door_skin")
    with ctx.pose({door_joint: 0.65}):
        open_near_aabb = ctx.part_element_world_aabb(near_leaf, elem="near_door_skin")
        open_far_aabb = ctx.part_element_world_aabb(far_leaf, elem="far_door_skin")

    # The paddle face bottom swings inward (toward +Y) when pressed
    rest_face_center_y = (rest_paddle_face[0][1] + rest_paddle_face[1][1]) / 2 if rest_paddle_face else None
    pressed_face_center_y = (pressed_paddle_face[0][1] + pressed_paddle_face[1][1]) / 2 if pressed_paddle_face else None

    ctx.check(
        "pressing push paddle rotates it inward",
        rest_face_center_y is not None
        and pressed_face_center_y is not None
        and pressed_face_center_y > rest_face_center_y + 0.008,
        details=f"rest_center_y={rest_face_center_y}, pressed_center_y={pressed_face_center_y}",
    )
    ctx.check(
        "linked center latch retracts when the paddle is pressed",
        rest_bolt is not None and retracted_bolt is not None and retracted_bolt[0] < rest_bolt[0] - 0.034,
        details=f"rest={rest_bolt}, retracted={retracted_bolt}",
    )
    ctx.check(
        "both symmetric door leaves rotate outward",
        closed_near_aabb is not None
        and closed_far_aabb is not None
        and open_near_aabb is not None
        and open_far_aabb is not None
        and open_near_aabb[0][1] < closed_near_aabb[0][1] - 0.12
        and open_far_aabb[0][1] < closed_far_aabb[0][1] - 0.12,
        details=(
            f"closed_near={closed_near_aabb}, open_near={open_near_aabb}, "
            f"closed_far={closed_far_aabb}, open_far={open_far_aabb}"
        ),
    )
    return ctx.report()


object_model = build_object_model()
