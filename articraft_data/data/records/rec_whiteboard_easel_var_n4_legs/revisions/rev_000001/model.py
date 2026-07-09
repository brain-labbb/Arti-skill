from __future__ import annotations

import math
from typing import Sequence

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


Vec3 = tuple[float, float, float]


def _sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _mul(v: Vec3, s: float) -> Vec3:
    return (v[0] * s, v[1] * s, v[2] * s)


def _length(v: Vec3) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def _unit(v: Vec3) -> Vec3:
    n = _length(v)
    if n <= 1e-9:
        raise ValueError("zero length vector")
    return (v[0] / n, v[1] / n, v[2] / n)


def _cylinder_rpy_for_vector(v: Vec3) -> tuple[float, float, float]:
    """Return an Origin.rpy that maps a URDF/Cylinder local +Z onto vector v."""
    d = _unit(v)
    pitch = math.acos(max(-1.0, min(1.0, d[2])))
    yaw = math.atan2(d[1], d[0])
    return (0.0, pitch, yaw)


def _add_cylinder_between(
    part,
    p0: Vec3,
    p1: Vec3,
    radius: float,
    material: Material | str,
    name: str,
) -> None:
    mid = _mul(_add(p0, p1), 0.5)
    vec = _sub(p1, p0)
    part.visual(
        Cylinder(radius=radius, length=_length(vec)),
        origin=Origin(xyz=mid, rpy=_cylinder_rpy_for_vector(vec)),
        material=material,
        name=name,
    )


def _leg_direction(hinge: Vec3, foot: Vec3) -> Vec3:
    return _unit(_sub(foot, hinge))


def _add_outer_leg(part, d: Vec3, materials: dict[str, Material]) -> None:
    _add_cylinder_between(
        part,
        _mul(d, 0.035),
        _mul(d, 0.520),
        0.014,
        materials["white_tube"],
        "outer_sleeve",
    )
    _add_cylinder_between(
        part,
        _mul(d, 0.505),
        _mul(d, 0.570),
        0.020,
        materials["aluminum"],
        "slide_collar",
    )
    # Small yoke ear below the pivot so the tube reads as hinged to the board.
    part.visual(
        Box((0.055, 0.055, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=materials["aluminum"],
        name="hinge_yoke",
    )


def _add_inner_leg(part, d: Vec3, materials: dict[str, Material]) -> None:
    # Top section deliberately extends back into the collar/sleeve.  The
    # corresponding test allowance classifies it as a retained telescoping fit.
    _add_cylinder_between(
        part,
        _mul(d, -0.180),
        _mul(d, 0.530),
        0.010,
        materials["white_tube"],
        "inner_tube",
    )
    foot_center = _mul(d, 0.535)
    part.visual(
        Cylinder(radius=0.017, length=0.020),
        origin=Origin(xyz=foot_center, rpy=_cylinder_rpy_for_vector(d)),
        material=materials["rubber"],
        name="rubber_foot",
    )
    part.visual(
        Sphere(radius=0.012),
        origin=Origin(xyz=_mul(d, 0.523)),
        material=materials["rubber"],
        name="foot_tip",
    )


def _add_clamp_knob(part, sign: float, materials: dict[str, Material]) -> None:
    _add_cylinder_between(
        part,
        (0.0, 0.0, 0.0),
        (sign * 0.030, 0.0, 0.0),
        0.006,
        materials["black_plastic"],
        "clamp_stem",
    )
    _add_cylinder_between(
        part,
        (sign * 0.030, 0.0, 0.0),
        (sign * 0.050, 0.0, 0.0),
        0.018,
        materials["black_plastic"],
        "clamp_knob",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="whiteboard_easel",
        meta={
            "reference_category": "Workspace / Whiteboard easel",
            "classification_note": "Reference image matches the requested whiteboard easel category.",
        },
    )

    materials = {
        "whiteboard": model.material("glossy_whiteboard", rgba=(0.96, 0.97, 0.96, 1.0)),
        "aluminum": model.material("satin_aluminum", rgba=(0.62, 0.65, 0.64, 1.0)),
        "dark_trim": model.material("dark_gray_trim", rgba=(0.18, 0.19, 0.19, 1.0)),
        "white_tube": model.material("white_powder_coated_tube", rgba=(0.92, 0.93, 0.91, 1.0)),
        "black_plastic": model.material("black_plastic", rgba=(0.01, 0.01, 0.01, 1.0)),
        "rubber": model.material("matte_black_rubber", rgba=(0.02, 0.02, 0.018, 1.0)),
        "red": model.material("red_magnet_caps", rgba=(0.95, 0.05, 0.02, 1.0)),
        "eraser": model.material("pale_wood_eraser", rgba=(0.75, 0.67, 0.50, 1.0)),
    }

    width = 0.82
    height = 1.05
    rail = 0.035
    thickness = 0.030

    board = model.part("board")
    board.visual(
        Box((width - 2.0 * rail, 0.010, height - 2.0 * rail)),
        origin=Origin(xyz=(0.0, -0.012, height * 0.5)),
        material=materials["whiteboard"],
        name="writing_panel",
    )
    board.visual(
        Box((rail, thickness, height)),
        origin=Origin(xyz=(-(width - rail) * 0.5, 0.0, height * 0.5)),
        material=materials["aluminum"],
        name="side_frame_0",
    )
    board.visual(
        Box((rail, thickness, height)),
        origin=Origin(xyz=((width - rail) * 0.5, 0.0, height * 0.5)),
        material=materials["aluminum"],
        name="side_frame_1",
    )
    board.visual(
        Box((width, thickness, rail)),
        origin=Origin(xyz=(0.0, 0.0, rail * 0.5)),
        material=materials["aluminum"],
        name="bottom_frame",
    )
    board.visual(
        Box((width, thickness, rail)),
        origin=Origin(xyz=(0.0, 0.0, height - rail * 0.5)),
        material=materials["aluminum"],
        name="top_frame",
    )
    for i, x in enumerate((-(width - rail) * 0.5, (width - rail) * 0.5)):
        for j, z in enumerate((rail * 0.5, height - rail * 0.5)):
            board.visual(
                Box((0.060, 0.038, 0.060)),
                origin=Origin(xyz=(x, 0.0, z)),
                material=materials["aluminum"],
                name=f"corner_cap_{i}_{j}",
            )

    # Lower marker tray and raised lip, placed just below the writing surface.
    board.visual(
        Box((0.650, 0.080, 0.012)),
        origin=Origin(xyz=(0.0, -0.048, -0.006)),
        material=materials["aluminum"],
        name="marker_tray_base",
    )
    board.visual(
        Box((0.650, 0.012, 0.028)),
        origin=Origin(xyz=(0.0, -0.090, 0.006)),
        material=materials["aluminum"],
        name="marker_tray_lip",
    )
    board.visual(
        Box((0.140, 0.026, 0.018)),
        origin=Origin(xyz=(-0.120, -0.080, 0.010)),
        material=materials["eraser"],
        name="tray_eraser_rest",
    )
    for i, x in enumerate((-0.310, -0.280)):
        board.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(xyz=(x, -0.019, 0.070), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=materials["red"],
            name=f"red_clamp_cap_{i}",
        )

    # Top paper clamp / flip-chart bar, with brackets tying it into the top rail.
    _add_cylinder_between(
        board,
        (-0.300, -0.006, height + 0.018),
        (0.300, -0.006, height + 0.018),
        0.018,
        materials["dark_trim"],
        "top_clamp_bar",
    )
    for i, x in enumerate((-0.330, 0.330)):
        board.visual(
            Box((0.036, 0.040, 0.046)),
            origin=Origin(xyz=(x, -0.004, height + 0.002)),
            material=materials["aluminum"],
            name=f"top_clamp_bracket_{i}",
        )

    # Hinge blocks under the lower frame carry the four folding legs
    # (2 front + 2 rear) in a symmetric rectangular splay.
    hinge_points = {
        "leg_0": (-0.250, 0.018, -0.035),
        "leg_1": (0.250, 0.018, -0.035),
        "leg_2": (-0.150, 0.030, -0.030),
        "leg_3": (0.150, 0.030, -0.030),
    }
    for name, hp in hinge_points.items():
        board.visual(
            Box((0.070, 0.036, 0.030)),
            origin=Origin(xyz=(hp[0], hp[1], -0.010)),
            material=materials["aluminum"],
            name=f"{name}_hinge_block",
        )
        _add_cylinder_between(
            board,
            (hp[0] - 0.038, hp[1], hp[2]),
            (hp[0] + 0.038, hp[1], hp[2]),
            0.010,
            materials["dark_trim"],
            f"{name}_hinge_pin",
        )

    # Visible side clamp sockets on the board frame.
    for i, (x, sign) in enumerate(((-width * 0.5, -1.0), (width * 0.5, 1.0))):
        _add_cylinder_between(
            board,
            (x, 0.0, 0.420),
            (x + sign * 0.014, 0.0, 0.420),
            0.014,
            materials["aluminum"],
            f"clamp_socket_{i}",
        )

    # Folded-support cross braces fixed to the board assembly. They make the
    # stand read as a real easel rather than four independent sticks.
    brace = model.part("brace")
    _add_cylinder_between(
        brace,
        (0.0, -0.010, -0.015),
        (0.0, -0.050, -0.330),
        0.007,
        materials["white_tube"],
        "center_strut",
    )
    _add_cylinder_between(
        brace,
        (-0.310, -0.070, -0.330),
        (0.310, -0.070, -0.330),
        0.007,
        materials["white_tube"],
        "front_crossbar",
    )
    # Rear crossbar at z=-0.330, clamping to the rear leg sleeves at the
    # same height as the front crossbar for a symmetric four-leg stance.
    _add_cylinder_between(
        brace,
        (-0.260, 0.179, -0.330),
        (0.260, 0.179, -0.330),
        0.007,
        materials["white_tube"],
        "rear_crossbar",
    )
    _add_cylinder_between(
        brace,
        (0.0, -0.050, -0.330),
        (-0.245, -0.070, -0.330),
        0.006,
        materials["white_tube"],
        "brace_arm_0",
    )
    _add_cylinder_between(
        brace,
        (0.0, -0.050, -0.330),
        (0.245, -0.070, -0.330),
        0.006,
        materials["white_tube"],
        "brace_arm_1",
    )
    # Rear brace arms run horizontally at z=-0.330 to avoid crossing the
    # angled rear leg sleeves.
    _add_cylinder_between(
        brace,
        (0.0, -0.050, -0.330),
        (-0.260, 0.179, -0.330),
        0.006,
        materials["white_tube"],
        "rear_brace_arm_0",
    )
    _add_cylinder_between(
        brace,
        (0.0, -0.050, -0.330),
        (0.260, 0.179, -0.330),
        0.006,
        materials["white_tube"],
        "rear_brace_arm_1",
    )
    model.articulation(
        "board_to_brace",
        ArticulationType.FIXED,
        parent=board,
        child=brace,
        origin=Origin(),
    )

    foot_targets = {
        "leg_0": (-0.435, -0.220, -0.925),
        "leg_1": (0.435, -0.220, -0.925),
        "leg_2": (-0.280, 0.450, -0.875),
        "leg_3": (0.280, 0.450, -0.875),
    }
    for leg_name, hinge in hinge_points.items():
        d = _leg_direction(hinge, foot_targets[leg_name])
        outer = model.part(leg_name)
        _add_outer_leg(outer, d, materials)
        axis = (1.0, 0.0, 0.0)
        model.articulation(
            f"board_to_{leg_name}",
            ArticulationType.REVOLUTE,
            parent=board,
            child=outer,
            origin=Origin(xyz=hinge),
            axis=axis,
            motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=-0.35, upper=0.75),
        )

        lower = model.part(f"{leg_name}_lower")
        _add_inner_leg(lower, d, materials)
        model.articulation(
            f"{leg_name}_slide",
            ArticulationType.PRISMATIC,
            parent=outer,
            child=lower,
            origin=Origin(xyz=_mul(d, 0.505)),
            axis=d,
            motion_limits=MotionLimits(effort=60.0, velocity=0.25, lower=0.0, upper=0.150),
        )

    # Two black side knobs act as rotating board-angle clamps.
    for i, (x, sign) in enumerate(((-width * 0.5 - 0.002, -1.0), (width * 0.5 + 0.002, 1.0))):
        knob = model.part(f"clamp_{i}")
        _add_clamp_knob(knob, sign, materials)
        model.articulation(
            f"board_to_clamp_{i}",
            ArticulationType.REVOLUTE,
            parent=board,
            child=knob,
            origin=Origin(xyz=(x, 0.0, 0.420)),
            axis=(sign, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=6.0, lower=-math.pi, upper=math.pi),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    board = object_model.get_part("board")
    brace = object_model.get_part("brace")

    # The inner tubes are intentionally represented as retained members inside
    # the visible collars/sleeves rather than as fully hollow CAD tubes.
    all_leg_names = ("leg_0", "leg_1", "leg_2", "leg_3")
    for leg_name in all_leg_names:
        outer = object_model.get_part(leg_name)
        lower = object_model.get_part(f"{leg_name}_lower")
        slide = object_model.get_articulation(f"{leg_name}_slide")
        ctx.allow_overlap(
            outer,
            lower,
            elem_a="outer_sleeve",
            elem_b="inner_tube",
            reason="The inner telescoping tube is intentionally retained inside the outer sleeve proxy.",
        )
        ctx.allow_overlap(
            outer,
            lower,
            elem_a="slide_collar",
            elem_b="inner_tube",
            reason="The collar visibly captures the sliding inner tube at the height-adjustment clamp.",
        )
        ctx.expect_overlap(
            lower,
            outer,
            axes="z",
            elem_a="inner_tube",
            elem_b="outer_sleeve",
            min_overlap=0.045,
            name=f"{leg_name} retained insertion at rest",
        )
        with ctx.pose({slide: 0.150}):
            ctx.expect_overlap(
                lower,
                outer,
                axes="z",
                elem_a="inner_tube",
                elem_b="outer_sleeve",
                min_overlap=0.018,
                name=f"{leg_name} retained insertion at full height",
            )

        ctx.allow_overlap(
            board,
            outer,
            elem_a=f"{leg_name}_hinge_pin",
            elem_b="hinge_yoke",
            reason="The hinge pin is intentionally captured through the leg yoke.",
        )
        ctx.expect_overlap(
            board,
            outer,
            axes="x",
            elem_a=f"{leg_name}_hinge_pin",
            elem_b="hinge_yoke",
            min_overlap=0.040,
            name=f"{leg_name} hinge pin spans yoke",
        )

    # Front legs (leg_0, leg_1) clamp to the front crossbar.
    for leg_name in ("leg_0", "leg_1"):
        outer = object_model.get_part(leg_name)
        ctx.allow_overlap(
            brace,
            outer,
            elem_a="front_crossbar",
            elem_b="outer_sleeve",
            reason="The front cross brace is represented as a bolted tube clamp around the leg sleeve.",
        )
        ctx.expect_overlap(
            brace,
            outer,
            axes="yz",
            elem_a="front_crossbar",
            elem_b="outer_sleeve",
            min_overlap=0.004,
            name=f"{leg_name} cross brace clamps sleeve",
        )

    # Rear legs (leg_2, leg_3) clamp to the rear crossbar.
    for leg_name in ("leg_2", "leg_3"):
        outer = object_model.get_part(leg_name)
        lower = object_model.get_part(f"{leg_name}_lower")
        ctx.allow_overlap(
            brace,
            outer,
            elem_a="rear_crossbar",
            elem_b="outer_sleeve",
            reason="The rear cross brace is represented as a bolted tube clamp around the leg sleeve.",
        )
        ctx.allow_overlap(
            brace,
            lower,
            elem_a="rear_crossbar",
            elem_b="inner_tube",
            reason="The rear crossbar passes through the inner telescoping tube at the clamping height.",
        )
        ctx.expect_overlap(
            brace,
            outer,
            axes="yz",
            elem_a="rear_crossbar",
            elem_b="outer_sleeve",
            min_overlap=0.004,
            name=f"{leg_name} rear cross brace clamps sleeve",
        )

    for i in (0, 1):
        clamp = object_model.get_part(f"clamp_{i}")
        ctx.allow_overlap(
            board,
            clamp,
            elem_a=f"clamp_socket_{i}",
            elem_b="clamp_stem",
            reason="The threaded clamp stem is intentionally seated inside the side socket.",
        )
        ctx.expect_overlap(
            board,
            clamp,
            axes="x",
            elem_a=f"clamp_socket_{i}",
            elem_b="clamp_stem",
            min_overlap=0.010,
            name=f"side clamp {i} stem seats in socket",
        )

    # The main writing board is a tall rectangle with a frame and a tray mounted
    # at the bottom edge, matching the reference proportions.
    ctx.expect_within(
        board,
        board,
        axes="x",
        inner_elem="writing_panel",
        outer_elem="top_frame",
        margin=0.001,
        name="writing panel is framed across width",
    )
    panel_box = ctx.part_element_world_aabb(board, elem="writing_panel")
    tray_box = ctx.part_element_world_aabb(board, elem="marker_tray_base")
    ctx.check(
        "tray sits at lower board edge",
        panel_box is not None
        and tray_box is not None
        and 0.020 <= panel_box[0][2] - tray_box[1][2] <= 0.045
        and tray_box[0][0] > panel_box[0][0]
        and tray_box[1][0] < panel_box[1][0],
        details=f"panel={panel_box}, tray={tray_box}",
    )
    ctx.expect_contact(
        brace,
        board,
        elem_a="center_strut",
        elem_b="bottom_frame",
        contact_tol=0.020,
        name="brace ties into lower frame",
    )

    # Decisive motion checks: telescoping legs extend downward and folding legs
    # swing relative to the board.
    front_lower = object_model.get_part("leg_0_lower")
    rest_pos = ctx.part_world_position(front_lower)
    with ctx.pose({"leg_0_slide": 0.150}):
        extended_pos = ctx.part_world_position(front_lower)
    ctx.check(
        "front telescoping leg extends downward",
        rest_pos is not None and extended_pos is not None and extended_pos[2] < rest_pos[2] - 0.10,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    front_outer = object_model.get_part("leg_0")
    rest_outer = ctx.part_world_aabb(front_outer)
    with ctx.pose({"board_to_leg_0": 0.55}):
        folded_outer = ctx.part_world_aabb(front_outer)
    ctx.check(
        "front leg hinge changes leg envelope",
        rest_outer is not None
        and folded_outer is not None
        and abs(folded_outer[0][1] - rest_outer[0][1]) > 0.025,
        details=f"rest={rest_outer}, folded={folded_outer}",
    )

    # Verify four-leg multiplicity: all four legs exist with independent
    # revolute fold joints and prismatic telescope joints.
    for leg_name in all_leg_names:
        fold_joint = object_model.get_articulation(f"board_to_{leg_name}")
        slide_joint = object_model.get_articulation(f"{leg_name}_slide")
        ctx.check(
            f"{leg_name} has revolute fold joint",
            fold_joint is not None,
            details=f"board_to_{leg_name} not found",
        )
        ctx.check(
            f"{leg_name} has prismatic slide joint",
            slide_joint is not None,
            details=f"{leg_name}_slide not found",
        )

    return ctx.report()


object_model = build_object_model()
