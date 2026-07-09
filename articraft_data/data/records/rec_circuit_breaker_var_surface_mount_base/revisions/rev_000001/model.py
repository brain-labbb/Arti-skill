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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_pole_surface_mount_circuit_breaker")

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
    black_rubber = model.material("black_wire_jacket", rgba=(0.008, 0.008, 0.007, 1.0))

    housing = model.part("housing")

    # Molded two-pole breaker body: a single rounded extrusion with raised
    # terminal towers and front cover details mounted into the shell.
    body_profile = rounded_rect_profile(0.074, 0.064, 0.006)
    body_shell = mesh_from_geometry(ExtrudeGeometry(body_profile, 0.110), "body_shell")
    housing.visual(
        body_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=molded_black,
        name="body_shell",
    )

    for i, x in enumerate((-0.019, 0.019)):
        housing.visual(
            Box((0.030, 0.059, 0.012)),
            origin=Origin(xyz=(x, 0.000, 0.116)),
            material=graphite,
            name=f"terminal_tower_{i}",
        )
        housing.visual(
            Box((0.018, 0.034, 0.0022)),
            origin=Origin(xyz=(x, -0.002, 0.1232)),
            material=dark_recess,
            name=f"top_wire_port_{i}",
        )
        housing.visual(
            Cylinder(radius=0.0042, length=0.038),
            origin=Origin(xyz=(x, 0.009, 0.143), rpy=(0.0, 0.0, 0.0)),
            material=red_rubber if i == 0 else black_rubber,
            name=f"wire_jacket_{i}",
        )
        housing.visual(
            Cylinder(radius=0.0023, length=0.010),
            origin=Origin(xyz=(x, 0.009, 0.119), rpy=(0.0, 0.0, 0.0)),
            material=copper,
            name=f"copper_core_{i}",
        )
        housing.visual(
            Box((0.028, 0.050, 0.010)),
            origin=Origin(xyz=(x, 0.000, 0.005)),
            material=graphite,
            name=f"lower_terminal_foot_{i}",
        )

    # Front seams, raised cover plates, label recesses, and terminal details.
    housing.visual(
        Box((0.0024, 0.0030, 0.106)),
        origin=Origin(xyz=(0.0, -0.0325, 0.056)),
        material=dark_recess,
        name="center_pole_seam",
    )
    housing.visual(
        Box((0.066, 0.0024, 0.011)),
        origin=Origin(xyz=(0.0, -0.0328, 0.075)),
        material=label_gray,
        name="front_arc_label_band",
    )
    housing.visual(
        Box((0.066, 0.0024, 0.009)),
        origin=Origin(xyz=(0.0, -0.0328, 0.052)),
        material=dark_recess,
        name="toggle_window_shadow",
    )

    for i, x in enumerate((-0.019, 0.019)):
        housing.visual(
            Box((0.027, 0.0024, 0.031)),
            origin=Origin(xyz=(x, -0.0328, 0.033)),
            material=label_gray,
            name=f"rating_label_{i}",
        )
        housing.visual(
            Box((0.022, 0.0012, 0.0016)),
            origin=Origin(xyz=(x, -0.0340, 0.046)),
            material=print_white,
            name=f"off_print_bar_{i}",
        )
        housing.visual(
            Cylinder(radius=0.0029, length=0.0012),
            origin=Origin(xyz=(x + 0.008, -0.0340, 0.046), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=print_white,
            name=f"off_o_mark_{i}",
        )
        for j in range(6):
            housing.visual(
                Box((0.018 - 0.0015 * (j % 2), 0.0012, 0.0010)),
                origin=Origin(xyz=(x, -0.0340, 0.037 - j * 0.0040)),
                material=print_white,
                name=f"rating_text_{i}_{j}",
            )
        housing.visual(
            Cylinder(radius=0.0062, length=0.0036),
            origin=Origin(xyz=(x, -0.0330, 0.096), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brass,
            name=f"top_terminal_screw_{i}",
        )
        housing.visual(
            Box((0.014, 0.0020, 0.004)),
            origin=Origin(xyz=(x, -0.0348, 0.096)),
            material=dark_recess,
            name=f"top_screw_slot_{i}",
        )
        housing.visual(
            Cylinder(radius=0.0057, length=0.0036),
            origin=Origin(xyz=(x, -0.0330, 0.017), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brass,
            name=f"bottom_terminal_screw_{i}",
        )
        housing.visual(
            Box((0.014, 0.0020, 0.004)),
            origin=Origin(xyz=(x, -0.0348, 0.017)),
            material=dark_recess,
            name=f"bottom_screw_slot_{i}",
        )
        housing.visual(
            Box((0.014, 0.0024, 0.008)),
            origin=Origin(xyz=(x, -0.0328, 0.086)),
            material=dark_recess,
            name=f"upper_wire_entry_{i}",
        )
        housing.visual(
            Box((0.014, 0.0024, 0.008)),
            origin=Origin(xyz=(x, -0.0328, 0.014)),
            material=dark_recess,
            name=f"lower_wire_entry_{i}",
        )

    # Side label, screw bosses, and arc-chute vent slots visible on the molded case.
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
        origin=Origin(xyz=(-0.0380, -0.002, 0.082), rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=graphite,
        name="side_vent_panel",
    )
    housing.visual(
        Box((0.0022, 0.032, 0.052)),
        origin=Origin(xyz=(-0.0379, -0.006, 0.050)),
        material=label_gray,
        name="side_rating_label",
    )
    for j in range(7):
        housing.visual(
            Box((0.0011, 0.020 - 0.001 * (j % 2), 0.0010)),
            origin=Origin(xyz=(-0.0389, -0.006, 0.070 - j * 0.0050)),
            material=print_white,
            name=f"side_label_text_{j}",
        )
    for j, z in enumerate((0.026, 0.061, 0.095)):
        housing.visual(
            Cylinder(radius=0.0044, length=0.0022),
            origin=Origin(xyz=(-0.0374, 0.023, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_recess,
            name=f"side_knockout_{j}",
        )

    # Surface-mount flange: a flat plate with corner screw holes for bolting
    # the breaker directly onto a panel or junction box wall.
    flange_y = 0.0340  # center of flange plate, just behind body rear face
    housing.visual(
        Box((0.090, 0.004, 0.130)),
        origin=Origin(xyz=(0.0, flange_y, 0.055)),
        material=galvanized,
        name="mounting_foot",
    )
    # Corner screw holes — 4 through-holes for panel screws.
    screw_positions = [
        (-0.036, flange_y, 0.114),
        ( 0.036, flange_y, 0.114),
        (-0.036, flange_y, -0.004),
        ( 0.036, flange_y, -0.004),
    ]
    for idx, (sx, sy, sz) in enumerate(screw_positions):
        # Countersunk recess around each hole (wider shallow cylinder)
        housing.visual(
            Cylinder(radius=0.0055, length=0.002),
            origin=Origin(xyz=(sx, sy - 0.002, sz), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_recess,
            name=f"mounting_hole_recess_{idx}",
        )
        # Through-hole (narrow cylinder through the flange)
        housing.visual(
            Cylinder(radius=0.0025, length=0.006),
            origin=Origin(xyz=(sx, sy, sz), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_recess,
            name=f"mounting_hole_{idx}",
        )

    # Pivot bearing cheeks and detent stops that retain the moving toggle.
    for i, x in enumerate((-0.0350, 0.0350)):
        housing.visual(
            Box((0.0052, 0.0070, 0.014)),
            origin=Origin(xyz=(x, -0.0348, 0.064)),
            material=graphite,
            name=f"pivot_cheek_{i}",
        )
        housing.visual(
            Cylinder(radius=0.0060, length=0.0045),
            origin=Origin(xyz=(x, -0.0390, 0.064), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=graphite,
            name=f"pivot_bearing_{i}",
        )
    # Slot end walls bounding the exposed toggle window.  The joint limit is set
    # so the grip travels from the OFF wall to the ON wall and no further.
    housing.visual(
        Box((0.058, 0.0035, 0.0030)),
        origin=Origin(xyz=(0.0, -0.0335, 0.082)),
        material=graphite,
        name="detent_stop_on",
    )
    housing.visual(
        Box((0.058, 0.0035, 0.0030)),
        origin=Origin(xyz=(0.0, -0.0335, 0.0515)),
        material=graphite,
        name="detent_stop_off",
    )
    # Round bore behind each rotor wheel so the drum reads as seated in the face.
    for i, x in enumerate((-0.019, 0.019)):
        housing.visual(
            Cylinder(radius=0.0108, length=0.0030),
            origin=Origin(xyz=(x, -0.0332, 0.064), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_recess,
            name=f"rotor_pocket_{i}",
        )

    # Moving switch assembly.  A cylindrical ROTOR (转轮) turns on the pivot line
    # and CARRIES the common trip LEVER (闸), which is fixed to its outer rim.
    toggle = model.part("toggle")
    rotor_r = 0.0095
    toggle.visual(
        Cylinder(radius=0.0034, length=0.0655),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="pivot_shaft",
    )
    for i, x in enumerate((-0.019, 0.019)):
        # The rotor wheel for this pole, centered on the pivot axis.
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
        # Index rib on the rim so the wheel's rotation reads as it turns.
        toggle.visual(
            Box((0.024, 0.0022, 0.0022)),
            origin=Origin(xyz=(x, -rotor_r + 0.0009, 0.0)),
            material=handle_gray,
            name=f"rotor_index_rib_{i}",
        )
    toggle.visual(
        Box((0.065, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, -0.010, -0.006)),
        material=handle_gray,
        name="common_tie_bar",
    )
    toggle.visual(
        Box((0.067, 0.016, 0.006)),
        origin=Origin(xyz=(0.0, -0.019, -0.013)),
        material=handle_gray,
        name="finger_ridge",
    )
    for i, x in enumerate((-0.019, 0.019)):
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
        origin=Origin(xyz=(0.0, -0.0390, 0.064)),
        # The handle hangs down and forward at q=0; rotating around -X raises it
        # through the molded detent window into the ON position.
        axis=(-1.0, 0.0, 0.0),
        # Bounded to the exposed slot window: OFF wall to ON wall, ~29 degrees.
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
        object_model.name == "two_pole_surface_mount_circuit_breaker"
        and housing is not None
        and toggle is not None,
        details=f"name={object_model.name}",
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
            "bottom_terminal_screw_1",
            "top_wire_port_0",
            "upper_wire_entry_1",
            "copper_core_0",
            "wire_jacket_1",
            "mounting_foot",
            "mounting_hole_0",
            "mounting_hole_3",
            "side_vent_panel",
            "rating_label_0",
            "center_pole_seam",
            "detent_stop_on",
            "detent_stop_off",
        }.issubset(housing_visual_names),
        details=f"missing={sorted({'top_terminal_screw_0', 'bottom_terminal_screw_1', 'top_wire_port_0', 'upper_wire_entry_1', 'copper_core_0', 'wire_jacket_1', 'mounting_foot', 'mounting_hole_0', 'mounting_hole_3', 'side_vent_panel', 'rating_label_0', 'center_pole_seam', 'detent_stop_on', 'detent_stop_off'} - housing_visual_names)}",
    )
    ctx.check(
        "toggle includes a rotor wheel that carries the handle hardware",
        {
            "pivot_shaft",
            "rotor_drum_0",
            "rotor_drum_1",
            "rotor_index_rib_0",
            "common_tie_bar",
            "finger_ridge",
            "thumb_paddle_0",
            "thumb_paddle_1",
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
        min_overlap=0.055,
        name="toggle shaft spans both breaker poles",
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

    # The rotor sits on the pivot axis and carries the grip on its rim: the wheel
    # spins in place while the grip sweeps an arc.
    def center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return ((lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5, (lo[2] + hi[2]) * 0.5)

    def yz_dist(a, b):
        return None if a is None or b is None else math.hypot(a[1] - b[1], a[2] - b[2])

    drum_off = center(ctx.part_element_world_aabb(toggle, elem="rotor_drum_1"))
    grip_off = center(ctx.part_element_world_aabb(toggle, elem="thumb_paddle_1"))
    with ctx.pose({joint: upper}):
        drum_on = center(ctx.part_element_world_aabb(toggle, elem="rotor_drum_1"))
        grip_on = center(ctx.part_element_world_aabb(toggle, elem="thumb_paddle_1"))
    ctx.check(
        "rotor stays on its axis while it carries the grip through an arc",
        yz_dist(drum_off, drum_on) < 0.0025
        and yz_dist(grip_off, grip_on) > 0.004
        and yz_dist(grip_off, drum_off) > 0.0095,
        details=f"drum_travel={yz_dist(drum_off, drum_on)}, "
        f"grip_travel={yz_dist(grip_off, grip_on)}, "
        f"grip_radius={yz_dist(grip_off, drum_off)}",
    )

    # Rotation amplitude is bounded by the exposed slot: the grip must stay
    # between the OFF and ON slot walls through the entire throw, and reach the
    # ON wall at the limit (a switch cannot over-rotate past its window).
    off_wall_top = ctx.part_element_world_aabb(housing, elem="detent_stop_off")[1][2]
    on_wall_bot = ctx.part_element_world_aabb(housing, elem="detent_stop_on")[0][2]
    grip_lo, grip_hi = [], []
    for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
        with ctx.pose({joint: upper * frac}):
            ab = ctx.part_element_world_aabb(toggle, elem="thumb_paddle_1")
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

    # Surface-mount variant: flange is behind the body and wider than the shell.
    flange_aabb = ctx.part_element_world_aabb(housing, elem="mounting_foot")
    body_aabb = ctx.part_element_world_aabb(housing, elem="body_shell")
    ctx.check(
        "mounting foot extends beyond body for panel screw access",
        flange_aabb is not None
        and body_aabb is not None
        and flange_aabb[0][0] < body_aabb[0][0] - 0.005
        and flange_aabb[1][0] > body_aabb[1][0] + 0.005,
        details=f"flange_x=[{flange_aabb[0][0]:.4f},{flange_aabb[1][0]:.4f}], "
        f"body_x=[{body_aabb[0][0]:.4f},{body_aabb[1][0]:.4f}]",
    )
    ctx.check(
        "surface-mount flange has all four corner screw holes",
        {"mounting_hole_0", "mounting_hole_1", "mounting_hole_2", "mounting_hole_3"}.issubset(
            housing_visual_names
        ),
        details=f"missing={sorted({'mounting_hole_0', 'mounting_hole_1', 'mounting_hole_2', 'mounting_hole_3'} - housing_visual_names)}",
    )

    return ctx.report()


object_model = build_object_model()
