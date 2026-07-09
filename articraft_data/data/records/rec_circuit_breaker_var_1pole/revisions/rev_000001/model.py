from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

# Single-pole (1P) DIN MCB — one module wide on the centerline.
# Derived from the two-pole parent by collapsing the N=2 pole loop to N=1,
# narrowing the body to one module, and shortening shared members.
N_POLES = 1
MODULE_WIDTH = 0.037          # one DIN module width
POLE_SPACING = 0.038          # unused beyond N=1; kept for documentation
BODY_DEPTH = 0.064
BODY_HEIGHT = 0.110


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_pole_din_circuit_breaker")

    molded_black = model.material("molded_black", rgba=(0.015, 0.017, 0.016, 1.0))
    graphite = model.material("graphite_plastic", rgba=(0.055, 0.060, 0.055, 1.0))
    dark_recess = model.material("deep_black_recess", rgba=(0.0, 0.0, 0.0, 1.0))
    handle_gray = model.material("handle_gray", rgba=(0.26, 0.27, 0.26, 1.0))
    label_gray = model.material("engraved_label_gray", rgba=(0.30, 0.32, 0.31, 1.0))
    print_white = model.material("printed_white", rgba=(0.82, 0.84, 0.80, 1.0))
    brass = model.material("brass_terminal", rgba=(0.78, 0.57, 0.24, 1.0))
    galvanized = model.material("galvanized_steel", rgba=(0.58, 0.61, 0.58, 1.0))
    copper = model.material("copper_conductor", rgba=(0.85, 0.36, 0.15, 1.0))
    red_rubber = model.material("red_wire_jacket", rgba=(0.55, 0.035, 0.025, 1.0))

    # ── Housing (molded case) ────────────────────────────────────────────
    housing = model.part("housing")

    # Single-module body: rounded-rect profile extruded to standard height.
    body_profile = rounded_rect_profile(MODULE_WIDTH, BODY_DEPTH, 0.005)
    body_shell = mesh_from_geometry(ExtrudeGeometry(body_profile, BODY_HEIGHT), "body_shell")
    housing.visual(
        body_shell,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT / 2.0)),
        material=molded_black,
        name="body_shell",
    )

    # Emit pole-specific features via the N-pole loop (N=1 here).
    pole_xs = [0.0]  # single pole on centerline
    for i, x in enumerate(pole_xs[:N_POLES]):
        housing.visual(
            Box((0.030, 0.059, 0.012)),
            origin=Origin(xyz=(x, 0.000, BODY_HEIGHT + 0.006)),
            material=graphite,
            name=f"terminal_tower_{i}",
        )
        housing.visual(
            Box((0.018, 0.034, 0.0022)),
            origin=Origin(xyz=(x, -0.002, BODY_HEIGHT + 0.0132)),
            material=dark_recess,
            name=f"top_wire_port_{i}",
        )
        housing.visual(
            Cylinder(radius=0.0042, length=0.038),
            origin=Origin(xyz=(x, 0.009, BODY_HEIGHT + 0.033)),
            material=red_rubber,
            name=f"wire_jacket_{i}",
        )
        housing.visual(
            Cylinder(radius=0.0023, length=0.010),
            origin=Origin(xyz=(x, 0.009, BODY_HEIGHT + 0.009)),
            material=copper,
            name=f"copper_core_{i}",
        )
        housing.visual(
            Box((0.028, 0.050, 0.010)),
            origin=Origin(xyz=(x, 0.000, 0.005)),
            material=graphite,
            name=f"lower_terminal_foot_{i}",
        )

    # Front face details — label band, toggle shadow, rating labels.
    housing.visual(
        Box((MODULE_WIDTH - 0.008, 0.0024, 0.011)),
        origin=Origin(xyz=(0.0, -(BODY_DEPTH / 2.0 + 0.0008), 0.075)),
        material=label_gray,
        name="front_arc_label_band",
    )
    housing.visual(
        Box((MODULE_WIDTH - 0.008, 0.0024, 0.009)),
        origin=Origin(xyz=(0.0, -(BODY_DEPTH / 2.0 + 0.0008), 0.052)),
        material=dark_recess,
        name="toggle_window_shadow",
    )

    for i, x in enumerate(pole_xs[:N_POLES]):
        housing.visual(
            Box((0.027, 0.0024, 0.031)),
            origin=Origin(xyz=(x, -(BODY_DEPTH / 2.0 + 0.0008), 0.033)),
            material=label_gray,
            name=f"rating_label_{i}",
        )
        housing.visual(
            Box((0.022, 0.0012, 0.0016)),
            origin=Origin(xyz=(x, -(BODY_DEPTH / 2.0 + 0.0020), 0.046)),
            material=print_white,
            name=f"off_print_bar_{i}",
        )
        housing.visual(
            Cylinder(radius=0.0029, length=0.0012),
            origin=Origin(xyz=(x + 0.008, -(BODY_DEPTH / 2.0 + 0.0020), 0.046),
                           rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=print_white,
            name=f"off_o_mark_{i}",
        )
        for j in range(6):
            housing.visual(
                Box((0.018 - 0.0015 * (j % 2), 0.0012, 0.0010)),
                origin=Origin(xyz=(x, -(BODY_DEPTH / 2.0 + 0.0020), 0.037 - j * 0.0040)),
                material=print_white,
                name=f"rating_text_{i}_{j}",
            )
        # Terminal screws (front-facing brass).
        housing.visual(
            Cylinder(radius=0.0062, length=0.0036),
            origin=Origin(xyz=(x, -(BODY_DEPTH / 2.0 + 0.0010), 0.096),
                           rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brass,
            name=f"top_terminal_screw_{i}",
        )
        housing.visual(
            Box((0.014, 0.0020, 0.004)),
            origin=Origin(xyz=(x, -(BODY_DEPTH / 2.0 + 0.0028), 0.096)),
            material=dark_recess,
            name=f"top_screw_slot_{i}",
        )
        housing.visual(
            Cylinder(radius=0.0057, length=0.0036),
            origin=Origin(xyz=(x, -(BODY_DEPTH / 2.0 + 0.0010), 0.017),
                           rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brass,
            name=f"bottom_terminal_screw_{i}",
        )
        housing.visual(
            Box((0.014, 0.0020, 0.004)),
            origin=Origin(xyz=(x, -(BODY_DEPTH / 2.0 + 0.0028), 0.017)),
            material=dark_recess,
            name=f"bottom_screw_slot_{i}",
        )
        housing.visual(
            Box((0.014, 0.0024, 0.008)),
            origin=Origin(xyz=(x, -(BODY_DEPTH / 2.0 + 0.0008), 0.086)),
            material=dark_recess,
            name=f"upper_wire_entry_{i}",
        )
        housing.visual(
            Box((0.014, 0.0024, 0.008)),
            origin=Origin(xyz=(x, -(BODY_DEPTH / 2.0 + 0.0008), 0.014)),
            material=dark_recess,
            name=f"lower_wire_entry_{i}",
        )

    # Side arc-chute vent slots and side label.
    side_vent = SlotPatternPanelGeometry(
        (0.033, 0.030),
        0.0022,
        slot_size=(0.017, 0.0032),
        pitch=(0.021, 0.008),
        frame=0.004,
        corner_radius=0.0015,
    )
    housing.visual(
        mesh_from_geometry(side_vent, "side_vent_panel"),
        origin=Origin(xyz=(-(MODULE_WIDTH / 2.0 + 0.0010), -0.002, 0.082),
                       rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=graphite,
        name="side_vent_panel",
    )
    housing.visual(
        Box((0.0022, 0.032, 0.052)),
        origin=Origin(xyz=(-(MODULE_WIDTH / 2.0 + 0.0009), -0.006, 0.050)),
        material=label_gray,
        name="side_rating_label",
    )
    for j in range(7):
        housing.visual(
            Box((0.0011, 0.020 - 0.001 * (j % 2), 0.0010)),
            origin=Origin(xyz=(-(MODULE_WIDTH / 2.0 + 0.0019), -0.006, 0.070 - j * 0.0050)),
            material=print_white,
            name=f"side_label_text_{j}",
        )
    for j, z in enumerate((0.026, 0.061, 0.095)):
        housing.visual(
            Cylinder(radius=0.0044, length=0.0022),
            origin=Origin(xyz=(-(MODULE_WIDTH / 2.0 + 0.0004), 0.023, z),
                           rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_recess,
            name=f"side_knockout_{j}",
        )

    # DIN rail catch and rear metal spring clip (one-module width).
    housing.visual(
        Box((MODULE_WIDTH - 0.010, 0.0040, 0.034)),
        origin=Origin(xyz=(0.0, BODY_DEPTH / 2.0 + 0.0017, 0.052)),
        material=galvanized,
        name="din_clip_backplate",
    )
    housing.visual(
        Box((MODULE_WIDTH - 0.010, 0.0080, 0.006)),
        origin=Origin(xyz=(0.0, BODY_DEPTH / 2.0 + 0.0050, 0.069)),
        material=galvanized,
        name="din_upper_hook",
    )
    housing.visual(
        Box((MODULE_WIDTH - 0.018, 0.0080, 0.006)),
        origin=Origin(xyz=(0.0, BODY_DEPTH / 2.0 + 0.0050, 0.033)),
        material=galvanized,
        name="din_lower_latch",
    )
    housing.visual(
        Box((0.018, 0.010, 0.005)),
        origin=Origin(xyz=(0.0, BODY_DEPTH / 2.0 + 0.007, 0.020)),
        material=graphite,
        name="din_latch_tab",
    )
    housing.visual(
        Box((0.012, 0.008, 0.018)),
        origin=Origin(xyz=(0.0, BODY_DEPTH / 2.0 + 0.006, 0.028)),
        material=graphite,
        name="din_latch_stem",
    )

    # Pivot bearing cheeks and detent stops that retain the moving toggle.
    cheek_x = MODULE_WIDTH / 2.0 - 0.001
    for i, x in enumerate((-cheek_x, cheek_x)):
        housing.visual(
            Box((0.0052, 0.0070, 0.014)),
            origin=Origin(xyz=(x, -(BODY_DEPTH / 2.0 + 0.0028), 0.064)),
            material=graphite,
            name=f"pivot_cheek_{i}",
        )
        housing.visual(
            Cylinder(radius=0.0060, length=0.0045),
            origin=Origin(xyz=(x, -(BODY_DEPTH / 2.0 + 0.0070), 0.064),
                           rpy=(0.0, math.pi / 2.0, 0.0)),
            material=graphite,
            name=f"pivot_bearing_{i}",
        )

    # Slot end walls bounding the exposed toggle window.
    housing.visual(
        Box((MODULE_WIDTH - 0.010, 0.0035, 0.0030)),
        origin=Origin(xyz=(0.0, -(BODY_DEPTH / 2.0 + 0.0015), 0.082)),
        material=graphite,
        name="detent_stop_on",
    )
    housing.visual(
        Box((MODULE_WIDTH - 0.010, 0.0035, 0.0030)),
        origin=Origin(xyz=(0.0, -(BODY_DEPTH / 2.0 + 0.0015), 0.0515)),
        material=graphite,
        name="detent_stop_off",
    )

    # Round bore behind rotor so the drum reads as seated in the face.
    for i, x in enumerate(pole_xs[:N_POLES]):
        housing.visual(
            Cylinder(radius=0.0108, length=0.0030),
            origin=Origin(xyz=(x, -(BODY_DEPTH / 2.0 + 0.0012), 0.064),
                           rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_recess,
            name=f"rotor_pocket_{i}",
        )

    # ── Toggle (moving switch assembly) ──────────────────────────────────
    toggle = model.part("toggle")
    rotor_r = 0.0095
    shaft_half = (MODULE_WIDTH - 0.006) / 2.0  # shaft spans inner housing width

    toggle.visual(
        Cylinder(radius=0.0034, length=2.0 * shaft_half),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="pivot_shaft",
    )

    for i, x in enumerate(pole_xs[:N_POLES]):
        toggle.visual(
            Cylinder(radius=rotor_r, length=0.024),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=graphite,
            name=f"rotor_drum_{i}",
        )
        toggle.visual(
            Cylinder(radius=0.0040, length=0.0252),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_recess,
            name=f"rotor_hub_{i}",
        )
        toggle.visual(
            Box((0.024, 0.0022, 0.0022)),
            origin=Origin(xyz=(x, -rotor_r + 0.0009, 0.0)),
            material=handle_gray,
            name=f"rotor_index_rib_{i}",
        )

    # Single-module tie bar and finger grip.
    tie_width = MODULE_WIDTH - 0.008
    toggle.visual(
        Box((tie_width, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, -0.010, -0.006)),
        material=handle_gray,
        name="common_tie_bar",
    )
    toggle.visual(
        Box((tie_width + 0.002, 0.016, 0.006)),
        origin=Origin(xyz=(0.0, -0.019, -0.013)),
        material=handle_gray,
        name="finger_ridge",
    )

    for i, x in enumerate(pole_xs[:N_POLES]):
        toggle.visual(
            Box((0.026, 0.012, 0.019)),
            origin=Origin(xyz=(x, -0.012, -0.001)),
            material=handle_gray,
            name=f"thumb_paddle_{i}",
        )
        toggle.visual(
            Box((0.014, 0.0011, 0.0015)),
            origin=Origin(xyz=(x - 0.003, -0.0177, 0.003)),
            material=print_white,
            name=f"on_print_bar_{i}",
        )
        toggle.visual(
            Box((0.0018, 0.0011, 0.0075)),
            origin=Origin(xyz=(x + 0.008, -0.0177, 0.001)),
            material=print_white,
            name=f"on_i_mark_{i}",
        )

    model.articulation(
        "housing_to_toggle",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=toggle,
        origin=Origin(xyz=(0.0, -(BODY_DEPTH / 2.0 + 0.0070), 0.064)),
        # Handle hangs down/forward at q=0; -X rotation raises it to ON.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=5.0,
            lower=0.0,
            upper=0.50,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    toggle = object_model.get_part("toggle")
    joint = object_model.get_articulation("housing_to_toggle")
    housing_visual_names = {v.name for v in housing.visuals}
    toggle_visual_names = {v.name for v in toggle.visuals}

    ctx.check(
        "asset remains a circuit breaker",
        object_model.name == "single_pole_din_circuit_breaker"
        and housing is not None
        and toggle is not None,
        details=f"name={object_model.name}",
    )

    # Variant-specific: single-pole layout assertions.
    ctx.check(
        "single pole has exactly one rotor_drum and one thumb_paddle",
        "rotor_drum_0" in toggle_visual_names
        and "thumb_paddle_0" in toggle_visual_names
        and "rotor_drum_1" not in toggle_visual_names
        and "thumb_paddle_1" not in toggle_visual_names,
        details=f"toggle visuals={sorted(toggle_visual_names)}",
    )
    ctx.check(
        "single pole has no center_pole_seam",
        "center_pole_seam" not in housing_visual_names,
        details=f"unexpected seam in housing visuals",
    )

    ctx.check(
        "breaker has front toggle revolute joint",
        joint.articulation_type == ArticulationType.REVOLUTE
        and joint.motion_limits is not None
        and joint.motion_limits.lower == 0.0
        and joint.motion_limits.upper is not None
        and 0.30 <= joint.motion_limits.upper <= 0.65,
        details=f"type={joint.articulation_type}, limits={joint.motion_limits}",
    )
    ctx.check(
        "electrical details are modeled as geometry",
        {
            "top_terminal_screw_0",
            "bottom_terminal_screw_0",
            "top_wire_port_0",
            "upper_wire_entry_0",
            "copper_core_0",
            "wire_jacket_0",
            "din_clip_backplate",
            "side_vent_panel",
            "rating_label_0",
            "detent_stop_on",
            "detent_stop_off",
        }.issubset(housing_visual_names),
        details=f"missing={sorted({'top_terminal_screw_0', 'bottom_terminal_screw_0', 'top_wire_port_0', 'upper_wire_entry_0', 'copper_core_0', 'wire_jacket_0', 'din_clip_backplate', 'side_vent_panel', 'rating_label_0', 'detent_stop_on', 'detent_stop_off'} - housing_visual_names)}",
    )
    ctx.check(
        "toggle includes a rotor wheel that carries the handle hardware",
        {
            "pivot_shaft",
            "rotor_drum_0",
            "rotor_index_rib_0",
            "common_tie_bar",
            "finger_ridge",
            "thumb_paddle_0",
            "on_i_mark_0",
        }.issubset(toggle_visual_names),
        details=f"toggle visuals={sorted(toggle_visual_names)}",
    )
    ctx.expect_gap(
        housing,
        toggle,
        axis="y",
        positive_elem="body_shell",
        negative_elem="pivot_shaft",
        min_gap=0.0035,
        max_gap=0.012,
        name="toggle pivot sits proud of molded front",
    )
    ctx.expect_overlap(
        toggle,
        housing,
        axes="x",
        elem_a="pivot_shaft",
        elem_b="body_shell",
        min_overlap=0.020,
        name="toggle shaft spans the single module width",
    )

    lower = joint.motion_limits.lower or 0.0
    upper = joint.motion_limits.upper or 0.0
    with ctx.pose({joint: lower}):
        lower_aabb = ctx.part_world_aabb(toggle)
    with ctx.pose({joint: upper}):
        upper_aabb = ctx.part_world_aabb(toggle)
    lower_center_z = None if lower_aabb is None else (lower_aabb[0][2] + lower_aabb[1][2]) / 2.0
    upper_center_z = None if upper_aabb is None else (upper_aabb[0][2] + upper_aabb[1][2]) / 2.0
    ctx.check(
        "toggle rotates upward toward ON",
        lower_center_z is not None
        and upper_center_z is not None
        and upper_center_z > lower_center_z + 0.004,
        details=f"lower_z={lower_center_z}, upper_z={upper_center_z}",
    )

    # The rotor sits on the pivot axis and carries the grip on its rim.
    def center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return ((lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5, (lo[2] + hi[2]) * 0.5)

    def yz_dist(a, b):
        return None if a is None or b is None else math.hypot(a[1] - b[1], a[2] - b[2])

    drum_off = center(ctx.part_element_world_aabb(toggle, elem="rotor_drum_0"))
    grip_off = center(ctx.part_element_world_aabb(toggle, elem="thumb_paddle_0"))
    with ctx.pose({joint: upper}):
        drum_on = center(ctx.part_element_world_aabb(toggle, elem="rotor_drum_0"))
        grip_on = center(ctx.part_element_world_aabb(toggle, elem="thumb_paddle_0"))
    ctx.check(
        "rotor stays on its axis while it carries the grip through an arc",
        yz_dist(drum_off, drum_on) < 0.0025
        and yz_dist(grip_off, grip_on) > 0.004
        and yz_dist(grip_off, drum_off) > 0.0095,
        details=f"drum_travel={yz_dist(drum_off, drum_on)}, "
        f"grip_travel={yz_dist(grip_off, grip_on)}, "
        f"grip_radius={yz_dist(grip_off, drum_off)}",
    )

    # Grip stays within the exposed slot opening for the entire throw.
    off_wall_top = ctx.part_element_world_aabb(housing, elem="detent_stop_off")[1][2]
    on_wall_bot = ctx.part_element_world_aabb(housing, elem="detent_stop_on")[0][2]
    grip_lo, grip_hi = [], []
    for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
        with ctx.pose({joint: upper * frac}):
            ab = ctx.part_element_world_aabb(toggle, elem="thumb_paddle_0")
            grip_lo.append(ab[0][2])
            grip_hi.append(ab[1][2])
    ctx.check(
        "grip stays within the exposed slot opening for the entire throw",
        min(grip_lo) >= off_wall_top - 0.001 and max(grip_hi) <= on_wall_bot + 0.001,
        details=f"grip_z[{min(grip_lo):.4f},{max(grip_hi):.4f}], "
        f"slot[{off_wall_top:.4f},{on_wall_bot:.4f}]",
    )
    ctx.check(
        "throw fills the opening (grip reaches the ON wall, no arbitrary range)",
        max(grip_hi) >= on_wall_bot - 0.004,
        details=f"grip_top={max(grip_hi):.4f}, on_wall_bot={on_wall_bot:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
