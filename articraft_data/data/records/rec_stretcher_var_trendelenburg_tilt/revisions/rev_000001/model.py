from __future__ import annotations

import math

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


def _tube_origin(p0: tuple[float, float, float], p1: tuple[float, float, float]) -> tuple[Origin, float]:
    """Return an Origin that aligns a URDF +Z cylinder between two local points."""
    x0, y0, z0 = p0
    x1, y1, z1 = p1
    dx, dy, dz = x1 - x0, y1 - y0, z1 - z0
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("tube endpoints must be distinct")
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    return Origin(xyz=((x0 + x1) * 0.5, (y0 + y1) * 0.5, (z0 + z1) * 0.5), rpy=(0.0, pitch, yaw)), length


def _tube(part, name: str, p0, p1, radius: float, material: Material) -> None:
    origin, length = _tube_origin(p0, p1)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def _box(part, name: str, size, xyz, material: Material, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="ambulance_stretcher_gurney",
        meta={
            "run_notes": (
                "Primary reference reads as a wheeled ambulance stretcher/gurney; "
                "no category conflict was detected, so the stretcher is modeled directly."
            )
        },
    )

    orange = model.material("safety_orange_vinyl", rgba=(0.95, 0.22, 0.06, 1.0))
    black = model.material("black_webbing_rubber", rgba=(0.015, 0.015, 0.012, 1.0))
    metal = model.material("brushed_stainless_tube", rgba=(0.72, 0.74, 0.74, 1.0))
    dark_metal = model.material("dark_grey_hubs", rgba=(0.16, 0.16, 0.16, 1.0))
    grey = model.material("pale_grey_plastic", rgba=(0.50, 0.52, 0.52, 1.0))
    red = model.material("red_release_handle", rgba=(0.90, 0.02, 0.01, 1.0))

    # Root: the lower wheeled carriage and static caster forks.
    lower = model.part("lower_carriage")
    rail_z = 0.30
    for y in (-0.25, 0.25):
        _tube(lower, f"lower_side_rail_{y:+.2f}", (-0.86, y, rail_z), (0.86, y, rail_z), 0.014, metal)
    for x in (-0.86, 0.0, 0.86):
        _tube(lower, f"lower_cross_rail_{x:+.2f}", (x, -0.25, rail_z), (x, 0.25, rail_z), 0.012, metal)

    # Four caster forks and stems are fixed to the carriage. The wheels themselves spin.
    caster_locations = [(-0.78, -0.24), (-0.78, 0.24), (0.78, -0.24), (0.78, 0.24)]
    for i, (x, y) in enumerate(caster_locations):
        _tube(lower, f"caster_stem_{i}", (x, y, 0.14), (x, y, rail_z), 0.012, metal)
        _box(lower, f"fork_plate_{i}_0", (0.030, 0.006, 0.115), (x, y - 0.032, 0.095), metal)
        _box(lower, f"fork_plate_{i}_1", (0.030, 0.006, 0.115), (x, y + 0.032, 0.095), metal)
        _tube(lower, f"fork_crown_{i}", (x - 0.028, y, 0.150), (x + 0.028, y, 0.150), 0.008, metal)
        _tube(lower, f"fork_bridge_{i}", (x, y - 0.032, 0.150), (x, y + 0.032, 0.150), 0.008, metal)
    _box(lower, "scissor_pivot_saddle_front", (0.070, 0.540, 0.030), (-0.525, 0.0, 0.310), metal)
    _box(lower, "scissor_pivot_saddle_rear", (0.070, 0.540, 0.030), (0.525, 0.0, 0.310), metal)

    # Tilt frame: intermediate pivot cradle carried by the height slide.
    # This is the structural platform that allows the whole patient deck to
    # pitch head-down (Trendelenburg) or head-up as one rigid plane.
    tilt = model.part("tilt_frame")
    _box(tilt, "pivot_plate", (1.10, 0.54, 0.018), (0.0, 0.0, 0.009), metal)
    # Pivot bearing blocks on each side where the litter trunnions seat.
    for y in (-0.29, 0.29):
        _box(tilt, f"pivot_bearing_{y:+.2f}", (0.10, 0.06, 0.040), (0.0, y, 0.038), dark_metal)
    # Lateral pivot crossbar connecting the two bearing blocks.
    _tube(tilt, "pivot_crossbar", (0.0, -0.29, 0.038), (0.0, 0.29, 0.038), 0.009, metal)

    # Height-adjusting litter frame: deck rails, handles, fixed mattress cushions, and foot controls.
    litter = model.part("litter_frame")
    frame_z = 0.035
    for y in (-0.315, 0.315):
        _tube(litter, f"upper_side_tube_{y:+.2f}", (-1.03, y, frame_z), (1.03, y, frame_z), 0.015, metal)
        _tube(litter, f"inner_mattress_support_{y:+.2f}", (-0.98, y * 0.72, 0.065), (0.29, y * 0.72, 0.065), 0.009, metal)
    _box(litter, "deck_pan", (1.48, 0.545, 0.014), (-0.355, 0.0, 0.078), dark_metal)
    for y in (-0.335, 0.335):
        _box(litter, f"side_rail_saddle_{y:+.2f}", (1.230, 0.040, 0.050), (-0.05, y, 0.069), metal)
    for y in (-0.225, 0.225):
        _tube(litter, f"scissor_roller_track_{y:+.2f}", (-0.56, y, 0.020), (0.56, y, 0.020), 0.010, metal)
    for x in (-1.03, -0.45, 0.24, 1.03):
        _tube(litter, f"upper_cross_tube_{x:+.2f}", (x, -0.315, frame_z), (x, 0.315, frame_z), 0.013, metal)

    # Foot-end push hoop and lower release lever like the reference image.
    _tube(litter, "foot_push_crossbar", (-1.16, -0.235, 0.120), (-1.16, 0.235, 0.120), 0.014, metal)
    _tube(litter, "foot_push_side_0", (-1.03, -0.235, 0.055), (-1.16, -0.235, 0.120), 0.012, metal)
    _tube(litter, "foot_push_side_1", (-1.03, 0.235, 0.055), (-1.16, 0.235, 0.120), 0.012, metal)
    _tube(litter, "red_release_lever", (-1.04, -0.220, 0.045), (-1.15, -0.260, -0.055), 0.009, red)
    _tube(litter, "black_release_grip", (-1.15, -0.260, -0.055), (-1.20, -0.285, -0.085), 0.010, black)

    # Orange segmented pads with black restraint straps.
    _box(litter, "foot_pad", (0.55, 0.525, 0.070), (-0.72, 0.0, 0.120), orange)
    _box(litter, "seat_pad", (0.60, 0.525, 0.070), (-0.100, 0.0, 0.120), orange)
    _box(litter, "foot_strap", (0.045, 0.555, 0.010), (-0.52, 0.0, 0.160), black)
    _box(litter, "seat_strap", (0.045, 0.555, 0.010), (0.090, 0.0, 0.160), black)
    _box(litter, "hinge_block", (0.060, 0.590, 0.030), (0.270, 0.0, 0.070), dark_metal)
    _box(litter, "back_hinge_lug_0", (0.080, 0.050, 0.036), (0.290, -0.295, 0.103), dark_metal)
    _box(litter, "back_hinge_lug_1", (0.080, 0.050, 0.036), (0.290, 0.295, 0.103), dark_metal)
    # Pivot trunnion lugs that seat into the tilt frame bearing blocks,
    # providing the physical connection for the deck tilt mechanism.
    for y in (-0.29, 0.29):
        _box(litter, f"tilt_trunnion_{y:+.2f}", (0.08, 0.05, 0.038), (0.0, y, 0.019), dark_metal)

    model.articulation(
        "height_slide",
        ArticulationType.PRISMATIC,
        parent=lower,
        child=tilt,
        origin=Origin(xyz=(0.0, 0.0, 0.670)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.18, lower=0.0, upper=0.240),
    )

    # Whole-deck Trendelenburg tilt: revolute pitch joint between the tilt
    # frame cradle and the litter frame. Positive q pitches head-end up
    # (reverse Trendelenburg); negative q pitches head-end down (Trendelenburg).
    model.articulation(
        "deck_tilt",
        ArticulationType.REVOLUTE,
        parent=tilt,
        child=litter,
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=200.0, velocity=0.12, lower=-0.26, upper=0.26),
    )

    # Hinged backrest, currently angled up to match the reference; negative q lowers it flat.
    backrest = model.part("backrest")
    back_angle = math.radians(34.0)
    back_len = 0.78
    cx = 0.5 * back_len * math.cos(back_angle)
    cz = 0.5 * back_len * math.sin(back_angle) + 0.045
    _box(backrest, "back_pad", (back_len, 0.525, 0.075), (cx, 0.0, cz), orange, rpy=(0.0, -back_angle, 0.0))
    _box(
        backrest,
        "back_strap",
        (0.045, 0.555, 0.010),
        (0.42 * math.cos(back_angle), 0.0, 0.42 * math.sin(back_angle) + 0.092),
        black,
        rpy=(0.0, -back_angle, 0.0),
    )
    # Perimeter tube around the elevated head panel.
    for y in (-0.305, 0.305):
        _tube(backrest, f"back_side_tube_{y:+.2f}", (0.02, y, 0.030), (back_len * math.cos(back_angle), y, back_len * math.sin(back_angle) + 0.030), 0.012, metal)
        _tube(backrest, f"back_pad_standoff_{y:+.2f}", (0.04, y * 0.86, 0.060), (0.04, y, 0.035), 0.007, metal)
        _tube(
            backrest,
            f"head_standoff_{y:+.2f}",
            (back_len * math.cos(back_angle) - 0.03, y * 0.86, back_len * math.sin(back_angle) + 0.060),
            (back_len * math.cos(back_angle) - 0.03, y, back_len * math.sin(back_angle) + 0.035),
            0.007,
            metal,
        )
    _tube(
        backrest,
        "head_end_tube",
        (back_len * math.cos(back_angle), -0.305, back_len * math.sin(back_angle) + 0.030),
        (back_len * math.cos(back_angle), 0.305, back_len * math.sin(back_angle) + 0.030),
        0.012,
        metal,
    )
    _tube(backrest, "back_hinge_pin", (0.020, -0.305, 0.030), (0.020, 0.305, 0.030), 0.011, dark_metal)

    model.articulation(
        "backrest_hinge",
        ArticulationType.REVOLUTE,
        parent=litter,
        child=backrest,
        origin=Origin(xyz=(0.270, 0.0, 0.102)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.8, lower=-0.58, upper=0.55),
    )

    # Folding tubular side guard rails on each side of the litter.
    for idx, side in enumerate((1.0, -1.0)):
        rail = model.part(f"side_rail_{idx}")
        _tube(rail, "hinge_tube", (-0.58, 0.0, 0.0), (0.58, 0.0, 0.0), 0.011, metal)
        _tube(rail, "top_guard_tube", (-0.52, 0.0, 0.185), (0.52, 0.0, 0.185), 0.012, metal)
        for x in (-0.52, 0.0, 0.52):
            _tube(rail, f"guard_upright_{x:+.2f}", (x, 0.0, 0.0), (x, 0.0, 0.185), 0.010, metal)
        axis = (-1.0, 0.0, 0.0) if side > 0.0 else (1.0, 0.0, 0.0)
        model.articulation(
            f"side_rail_hinge_{idx}",
            ArticulationType.REVOLUTE,
            parent=litter,
            child=rail,
            origin=Origin(xyz=(-0.05, side * 0.335, 0.105)),
            axis=axis,
            motion_limits=MotionLimits(effort=25.0, velocity=1.2, lower=0.0, upper=1.55),
        )

    # Scissor lift linkages: two crossing tubular pairs with central spacers.
    arm_a = model.part("scissor_arm_0")
    arm_b = model.part("scissor_arm_1")
    _tube(arm_a, "diagonal_tube_inner", (0.0, -0.185, 0.0), (1.05, -0.185, 0.335), 0.012, metal)
    _tube(arm_a, "diagonal_tube_outer", (0.0, 0.185, 0.0), (1.05, 0.185, 0.335), 0.012, metal)
    _tube(arm_b, "diagonal_tube_inner", (0.0, -0.245, 0.0), (-1.05, -0.245, 0.335), 0.012, metal)
    _tube(arm_b, "diagonal_tube_outer", (0.0, 0.245, 0.0), (-1.05, 0.245, 0.335), 0.012, metal)
    _tube(arm_a, "lower_pivot_crossbar", (0.0, -0.205, 0.0), (0.0, 0.205, 0.0), 0.010, metal)
    _tube(arm_a, "upper_roller_crossbar", (1.05, -0.205, 0.335), (1.05, 0.205, 0.335), 0.010, metal)
    _tube(arm_b, "lower_pivot_crossbar", (0.0, -0.265, 0.0), (0.0, 0.265, 0.0), 0.010, metal)
    _tube(arm_b, "upper_roller_crossbar", (-1.05, -0.265, 0.335), (-1.05, 0.265, 0.335), 0.010, metal)

    model.articulation(
        "scissor_hinge_0",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=arm_a,
        origin=Origin(xyz=(-0.525, 0.0, 0.335)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=150.0, velocity=0.7, lower=-0.25, upper=0.35),
    )
    model.articulation(
        "scissor_hinge_1",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=arm_b,
        origin=Origin(xyz=(0.525, 0.0, 0.335)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=150.0, velocity=0.7, lower=-0.25, upper=0.35),
    )

    # Four independent caster wheel spin joints.
    for i, (x, y) in enumerate(caster_locations):
        wheel = model.part(f"caster_{i}")
        wheel.visual(Cylinder(radius=0.062, length=0.058), origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)), material=black, name="tire")
        wheel.visual(Cylinder(radius=0.030, length=0.064), origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)), material=grey, name="hub")
        _box(wheel, "spin_marker", (0.012, 0.046, 0.006), (0.037, 0.0, 0.0), grey)
        model.articulation(
            f"caster_spin_{i}",
            ArticulationType.CONTINUOUS,
            parent=lower,
            child=wheel,
            origin=Origin(xyz=(x, y, 0.065)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=12.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower = object_model.get_part("lower_carriage")
    tilt = object_model.get_part("tilt_frame")
    litter = object_model.get_part("litter_frame")
    backrest = object_model.get_part("backrest")
    side_rail = object_model.get_part("side_rail_0")
    scissor = object_model.get_part("scissor_arm_0")
    scissor_b = object_model.get_part("scissor_arm_1")
    height = object_model.get_articulation("height_slide")
    deck_tilt = object_model.get_articulation("deck_tilt")
    back_hinge = object_model.get_articulation("backrest_hinge")
    rail_hinge = object_model.get_articulation("side_rail_hinge_0")
    scissor_hinge = object_model.get_articulation("scissor_hinge_0")

    ctx.expect_overlap(
        lower,
        scissor,
        axes="xy",
        min_overlap=0.04,
        name="front scissor pair is captured at its lower saddle",
    )
    ctx.expect_overlap(
        lower,
        scissor_b,
        axes="xy",
        min_overlap=0.04,
        name="rear scissor pair is captured at its lower saddle",
    )

    # The scissor arm diagonal tubes and upper roller crossbars nest under the
    # tilt frame pivot plate (intentional mechanical interface: the lift
    # supports the cradle from below).
    for arm_name in ("scissor_arm_0", "scissor_arm_1"):
        for elem in ("diagonal_tube_inner", "diagonal_tube_outer", "upper_roller_crossbar"):
            ctx.allow_overlap(
                arm_name, "tilt_frame",
                elem_a=elem, elem_b="pivot_plate",
                reason=f"Scissor arm {elem} is intentionally nested under the tilt frame pivot plate; the lift supports the cradle from below.",
            )
    ctx.expect_contact(
        scissor, tilt,
        name="scissor arm top contacts the tilt frame underside",
    )

    # The litter trunnion lugs seat into the tilt frame bearings (small local
    # overlap representing a seated pivot joint).
    ctx.allow_overlap(
        "litter_frame", "tilt_frame",
        elem_a="tilt_trunnion_-0.29", elem_b="pivot_bearing_-0.29",
        reason="Litter trunnion lug is intentionally seated into the tilt frame bearing block for the pivot joint.",
    )
    ctx.allow_overlap(
        "litter_frame", "tilt_frame",
        elem_a="tilt_trunnion_+0.29", elem_b="pivot_bearing_+0.29",
        reason="Litter trunnion lug is intentionally seated into the tilt frame bearing block for the pivot joint.",
    )
    ctx.expect_contact(
        litter, tilt,
        elem_a="tilt_trunnion_-0.29", elem_b="pivot_bearing_-0.29",
        name="litter trunnion is seated on tilt frame bearing (left)",
    )
    ctx.expect_contact(
        litter, tilt,
        elem_a="tilt_trunnion_+0.29", elem_b="pivot_bearing_+0.29",
        name="litter trunnion is seated on tilt frame bearing (right)",
    )

    required = [
        "lower_carriage",
        "tilt_frame",
        "litter_frame",
        "backrest",
        "side_rail_0",
        "side_rail_1",
        "scissor_arm_0",
        "scissor_arm_1",
        "caster_0",
        "caster_1",
        "caster_2",
        "caster_3",
    ]
    ctx.check("stretcher subassemblies present", all(object_model.get_part(name) for name in required))

    # Verify deck_tilt joint exists, is revolute, and has correct axis.
    ctx.check(
        "deck_tilt is a revolute pitch joint",
        deck_tilt.articulation_type == ArticulationType.REVOLUTE and abs(deck_tilt.axis[1]) > 0.9,
        details=f"type={deck_tilt.articulation_type}, axis={deck_tilt.axis}",
    )

    ctx.expect_overlap(litter, lower, axes="xy", min_overlap=0.45, name="litter frame stays above wheeled carriage footprint")
    ctx.expect_overlap(backrest, litter, axes="y", min_overlap=0.40, name="backrest spans the mattress width")

    rest_litter_pos = ctx.part_world_position(litter)
    with ctx.pose({height: 0.24}):
        raised_litter_pos = ctx.part_world_position(litter)
    ctx.check(
        "height adjustment raises the litter deck",
        rest_litter_pos is not None
        and raised_litter_pos is not None
        and raised_litter_pos[2] > rest_litter_pos[2] + 0.20,
        details=f"rest={rest_litter_pos}, raised={raised_litter_pos}",
    )

    # Prove that deck_tilt pitches the entire patient deck as one rigid plane.
    # At positive q the head end (positive-X side) rises; at negative q it drops.
    rest_litter_aabb = ctx.part_world_aabb(litter)
    with ctx.pose({deck_tilt: 0.24}):
        tilted_up_aabb = ctx.part_world_aabb(litter)
    with ctx.pose({deck_tilt: -0.24}):
        tilted_down_aabb = ctx.part_world_aabb(litter)
    ctx.check(
        "deck_tilt pitches the litter head-up at positive angle",
        rest_litter_aabb is not None
        and tilted_up_aabb is not None
        and tilted_up_aabb[1][2] > rest_litter_aabb[1][2] + 0.02,
        details=f"rest_max_z={rest_litter_aabb[1][2] if rest_litter_aabb else None}, "
                f"tilted_up_max_z={tilted_up_aabb[1][2] if tilted_up_aabb else None}",
    )
    ctx.check(
        "deck_tilt pitches the litter head-down at negative angle (Trendelenburg)",
        rest_litter_aabb is not None
        and tilted_down_aabb is not None
        and tilted_down_aabb[0][2] < rest_litter_aabb[0][2] - 0.02,
        details=f"rest_min_z={rest_litter_aabb[0][2] if rest_litter_aabb else None}, "
                f"tilted_down_min_z={tilted_down_aabb[0][2] if tilted_down_aabb else None}",
    )

    # Backrest still articulates independently on top of the tilted deck.
    with ctx.pose({deck_tilt: 0.20}):
        backrest_on_tilt_aabb = ctx.part_world_aabb(backrest)
    with ctx.pose({deck_tilt: 0.20, back_hinge: -0.55}):
        combined_back_aabb = ctx.part_world_aabb(backrest)
    ctx.check(
        "backrest hinge operates independently above a tilted deck",
        backrest_on_tilt_aabb is not None
        and combined_back_aabb is not None
        and combined_back_aabb[1][2] < backrest_on_tilt_aabb[1][2] - 0.10,
        details=f"backrest_upright={backrest_on_tilt_aabb}, backrest_lowered={combined_back_aabb}",
    )

    raised_back_aabb = ctx.part_world_aabb(backrest)
    with ctx.pose({back_hinge: -0.55}):
        lowered_back_aabb = ctx.part_world_aabb(backrest)
    ctx.check(
        "hinged backrest lowers from the reference raised pose",
        raised_back_aabb is not None
        and lowered_back_aabb is not None
        and ((lowered_back_aabb[0][2] + lowered_back_aabb[1][2]) * 0.5)
        < ((raised_back_aabb[0][2] + raised_back_aabb[1][2]) * 0.5) - 0.18,
        details=f"raised={raised_back_aabb}, lowered={lowered_back_aabb}",
    )

    upright_rail_aabb = ctx.part_world_aabb(side_rail)
    with ctx.pose({rail_hinge: 1.45}):
        folded_rail_aabb = ctx.part_world_aabb(side_rail)
    ctx.check(
        "folding side rail drops outward",
        upright_rail_aabb is not None
        and folded_rail_aabb is not None
        and folded_rail_aabb[1][2] < upright_rail_aabb[1][2] - 0.10
        and folded_rail_aabb[1][1] > upright_rail_aabb[1][1] + 0.10,
        details=f"upright={upright_rail_aabb}, folded={folded_rail_aabb}",
    )

    scissor_rest_aabb = ctx.part_world_aabb(scissor)
    with ctx.pose({scissor_hinge: 0.30}):
        scissor_moved_aabb = ctx.part_world_aabb(scissor)
    ctx.check(
        "scissor linkage pivots for height adjustment",
        scissor_rest_aabb is not None
        and scissor_moved_aabb is not None
        and abs(scissor_moved_aabb[1][2] - scissor_rest_aabb[1][2]) > 0.05,
        details=f"rest={scissor_rest_aabb}, moved={scissor_moved_aabb}",
    )

    caster_joints = [object_model.get_articulation(f"caster_spin_{i}") for i in range(4)]
    ctx.check(
        "all four caster wheels have spin joints",
        all(j.articulation_type == ArticulationType.CONTINUOUS and tuple(j.axis) == (0.0, 1.0, 0.0) for j in caster_joints),
    )

    return ctx.report()


object_model = build_object_model()
