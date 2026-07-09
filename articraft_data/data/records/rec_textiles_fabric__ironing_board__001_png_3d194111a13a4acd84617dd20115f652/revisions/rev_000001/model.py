from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _signed_area(profile: list[tuple[float, float]]) -> float:
    return 0.5 * sum(
        x0 * y1 - x1 * y0
        for (x0, y0), (x1, y1) in zip(profile, profile[1:] + profile[:1])
    )


def _ccw(profile: list[tuple[float, float]]) -> list[tuple[float, float]]:
    return profile if _signed_area(profile) > 0.0 else list(reversed(profile))


def _board_outline(scale: float = 1.0) -> list[tuple[float, float]]:
    """Long ironing-board planform: rounded pointed nose, tapered sides, squared tail."""
    pts = [
        (-0.705, 0.000),
        (-0.675, 0.070),
        (-0.560, 0.148),
        (-0.320, 0.193),
        (0.220, 0.186),
        (0.610, 0.166),
        (0.690, 0.135),
        (0.705, 0.070),
        (0.705, -0.070),
        (0.690, -0.135),
        (0.610, -0.166),
        (0.220, -0.186),
        (-0.320, -0.193),
        (-0.560, -0.148),
        (-0.675, -0.070),
    ]
    return _ccw([(x * scale, y * scale) for x, y in pts])


def _half_width_at(x: float) -> float:
    outline = sorted(_board_outline(), key=lambda p: p[0])
    upper = [p for p in outline if p[1] >= -1e-9]
    upper = sorted(upper, key=lambda p: p[0])
    for (x0, y0), (x1, y1) in zip(upper, upper[1:]):
        if x0 <= x <= x1:
            t = 0.0 if abs(x1 - x0) < 1e-9 else (x - x0) / (x1 - x0)
            return y0 + t * (y1 - y0)
    return 0.0


def _circle_profile(cx: float, cy: float, r: float, segments: int = 14) -> list[tuple[float, float]]:
    pts = [
        (cx + math.cos(2.0 * math.pi * i / segments) * r, cy + math.sin(2.0 * math.pi * i / segments) * r)
        for i in range(segments)
    ]
    # Holes are opposite orientation to the outer profile.
    return pts if _signed_area(pts) < 0.0 else list(reversed(pts))


def _perforation_holes() -> list[list[tuple[float, float]]]:
    holes: list[list[tuple[float, float]]] = []
    xs = [-0.48, -0.36, -0.24, -0.12, 0.00, 0.12, 0.24, 0.36, 0.48, 0.58]
    for row, y_abs in enumerate((0.045, 0.092, 0.136)):
        for x in xs:
            hw = _half_width_at(x)
            if y_abs + 0.028 < hw:
                holes.append(_circle_profile(x, y_abs, 0.014))
                holes.append(_circle_profile(x + (0.035 if row % 2 else 0.0), -y_abs, 0.014))
    # A central line of larger vent holes reads through gaps around the fabric edge.
    for x in (-0.42, -0.30, -0.18, -0.06, 0.06, 0.18, 0.30, 0.42, 0.54):
        if 0.020 < _half_width_at(x):
            holes.append(_circle_profile(x, 0.0, 0.012))
    return holes


def _tube(points, radius: float, *, name: str, radial_segments: int = 18):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=10,
            radial_segments=radial_segments,
            cap_ends=True,
        ),
        name,
    )


def _add_surface_strip(
    board,
    *,
    material,
    name: str,
    x: float,
    y: float,
    length: float,
    width: float,
    yaw: float,
    z: float = 0.864,
) -> None:
    board.visual(
        Box((length, width, 0.0016)),
        origin=Origin(xyz=(x, y, z), rpy=(0.0, 0.0, yaw)),
        material=material,
        name=name,
    )


def _add_palm_frond(
    board,
    *,
    material,
    prefix: str,
    x: float,
    y: float,
    yaw: float,
    length: float,
    leaflets: int,
    side: float = 1.0,
) -> None:
    """Low-contrast printed palm frond matching the reference fabric cover."""
    _add_surface_strip(
        board,
        material=material,
        name=f"{prefix}_midrib",
        x=x,
        y=y,
        length=length,
        width=0.004,
        yaw=yaw,
        z=0.865,
    )
    dx = math.cos(yaw)
    dy = math.sin(yaw)
    normal_x = -math.sin(yaw)
    normal_y = math.cos(yaw)
    start = -0.42 * length
    step = (0.84 * length) / max(leaflets - 1, 1)
    for i in range(leaflets):
        t = start + i * step
        taper = 1.0 - abs(t) / (0.46 * length)
        leaflet_len = max(0.026, length * (0.135 + 0.070 * taper))
        cx = x + dx * t + normal_x * side * leaflet_len * 0.34
        cy = y + dy * t + normal_y * side * leaflet_len * 0.34
        if abs(cy) + 0.010 > _half_width_at(cx):
            continue
        _add_surface_strip(
            board,
            material=material,
            name=f"{prefix}_leaflet_{i:02d}",
            x=cx,
            y=cy,
            length=leaflet_len,
            width=0.006,
            yaw=yaw + side * 0.78,
            z=0.866,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="folding_ironing_board",
        meta={
            "note": "Reference and category agree: a folding ironing board is modeled as the visible core object.",
        },
    )

    fabric = model.material("warm_white_fabric", rgba=(0.92, 0.93, 0.91, 1.0))
    fabric_print = model.material("low_contrast_gray_palm_print", rgba=(0.70, 0.72, 0.71, 1.0))
    fabric_print_light = model.material("faint_gray_weave_print", rgba=(0.79, 0.81, 0.80, 1.0))
    deck_mat = model.material("perforated_steel", rgba=(0.72, 0.76, 0.80, 1.0))
    dark_metal = model.material("black_powder_coated_steel", rgba=(0.015, 0.018, 0.026, 1.0))
    rubber = model.material("ribbed_black_plastic", rgba=(0.005, 0.005, 0.007, 1.0))
    latch_metal = model.material("zinc_pivot_hardware", rgba=(0.76, 0.72, 0.66, 1.0))

    board = model.part("board")
    board.visual(
        mesh_from_geometry(ExtrudeWithHolesGeometry(_board_outline(0.985), _perforation_holes(), 0.012), "perforated_deck"),
        origin=Origin(xyz=(0.0, 0.0, 0.828)),
        material=deck_mat,
        name="perforated_deck",
    )
    board.visual(
        mesh_from_geometry(ExtrudeGeometry(_board_outline(1.015), 0.026), "fabric_cover"),
        origin=Origin(xyz=(0.0, 0.0, 0.850)),
        material=fabric,
        name="fabric_cover",
    )
    board.visual(
        mesh_from_geometry(ExtrudeGeometry(_board_outline(1.030), 0.018), "padded_edge_band"),
        origin=Origin(xyz=(0.0, 0.0, 0.832)),
        material=fabric,
        name="padded_edge_band",
    )

    # Reference cover: warm white cloth with a faint grey palm-leaf print.
    for i, x in enumerate((-0.55, -0.47, -0.39, -0.31, -0.23, -0.15, -0.07)):
        _add_surface_strip(
            board,
            material=fabric_print_light,
            name=f"nose_fine_diagonal_weave_{i}",
            x=x,
            y=0.020 + 0.018 * (i % 3),
            length=0.28,
            width=0.003,
            yaw=-0.58,
            z=0.8648,
        )
    for i, y in enumerate((-0.120, -0.100, -0.080, 0.080, 0.100, 0.120)):
        _add_surface_strip(
            board,
            material=fabric_print_light,
            name=f"long_faint_fabric_line_{i}",
            x=0.050,
            y=y,
            length=0.78,
            width=0.0024,
            yaw=0.050,
            z=0.8646,
        )
    _add_palm_frond(
        board,
        material=fabric_print,
        prefix="left_front_palm",
        x=-0.335,
        y=-0.080,
        yaw=0.38,
        length=0.42,
        leaflets=11,
        side=1.0,
    )
    _add_palm_frond(
        board,
        material=fabric_print_light,
        prefix="left_upper_palm",
        x=-0.365,
        y=0.095,
        yaw=-0.35,
        length=0.34,
        leaflets=9,
        side=-1.0,
    )
    _add_palm_frond(
        board,
        material=fabric_print,
        prefix="center_palm",
        x=-0.060,
        y=0.010,
        yaw=-0.12,
        length=0.46,
        leaflets=12,
        side=1.0,
    )
    _add_palm_frond(
        board,
        material=fabric_print_light,
        prefix="right_rear_palm",
        x=0.305,
        y=0.095,
        yaw=0.46,
        length=0.38,
        leaflets=10,
        side=-1.0,
    )
    _add_palm_frond(
        board,
        material=fabric_print,
        prefix="right_lower_palm",
        x=0.360,
        y=-0.080,
        yaw=-0.48,
        length=0.32,
        leaflets=8,
        side=1.0,
    )

    # Rear iron rest and handle-like rail, as a welded black tubular rack.
    board.visual(
        _tube(
            [
                (0.600, -0.166, 0.858),
                (0.790, -0.215, 0.910),
                (1.010, -0.205, 0.912),
                (1.060, 0.000, 0.912),
                (1.010, 0.205, 0.912),
                (0.790, 0.215, 0.910),
                (0.600, 0.166, 0.858),
            ],
            0.010,
            name="rear_iron_rest_rail",
        ),
        material=dark_metal,
        name="rear_iron_rest_rail",
    )
    for i, (y, rail_x) in enumerate(((-0.095, 1.038), (0.0, 1.060), (0.095, 1.038))):
        board.visual(
            _tube(
                [
                    (0.760, y, 0.879),
                    (0.880, y, 0.896),
                    (rail_x, y, 0.912),
                ],
                0.006,
                name=f"iron_rest_crossbar_{i}",
                radial_segments=14,
            ),
            material=dark_metal,
            name=f"iron_rest_crossbar_{i}",
        )
    board.visual(
        Box((0.240, 0.205, 0.010)),
        origin=Origin(xyz=(0.675, 0.0, 0.871)),
        material=dark_metal,
        name="rear_rest_plate",
    )
    for i, y in enumerate((-0.045, 0.045)):
        board.visual(
            Cylinder(radius=0.006, length=0.070),
            origin=Origin(xyz=(0.665, y, 0.866)),
            material=dark_metal,
            name=f"iron_rest_support_{i}",
        )

    # Underside brackets, braces, and the toothed height-adjustment rack.
    for i, x in enumerate((-0.35, -0.05, 0.27)):
        board.visual(
            Cylinder(radius=0.008, length=0.315),
            origin=Origin(xyz=(x, 0.0, 0.815), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"underside_cross_tube_{i}",
        )
    board.visual(
        Box((0.480, 0.026, 0.020)),
        origin=Origin(xyz=(0.050, -0.060, 0.812)),
        material=dark_metal,
        name="height_notch_rail",
    )
    for i, x in enumerate((-0.135, -0.075, -0.015, 0.045, 0.105, 0.165, 0.225)):
        board.visual(
            Box((0.026, 0.032, 0.010)),
            origin=Origin(xyz=(x, -0.060, 0.826), rpy=(0.0, 0.0, 0.38)),
            material=latch_metal,
            name=f"height_notch_{i}",
        )
    board.visual(
        Box((0.055, 0.012, 0.075)),
        origin=Origin(xyz=(0.300, -0.176, 0.805)),
        material=dark_metal,
        name="front_hinge_bracket_0",
    )
    board.visual(
        Box((0.055, 0.012, 0.075)),
        origin=Origin(xyz=(0.300, 0.176, 0.805)),
        material=dark_metal,
        name="front_hinge_bracket_1",
    )
    board.visual(
        Box((0.075, 0.012, 0.055)),
        origin=Origin(xyz=(-0.200, -0.135, 0.805)),
        material=dark_metal,
        name="slider_guide_bracket_0",
    )
    board.visual(
        Box((0.075, 0.012, 0.055)),
        origin=Origin(xyz=(-0.200, 0.135, 0.805)),
        material=dark_metal,
        name="slider_guide_bracket_1",
    )

    slider = model.part("height_slider")
    slider.visual(
        Cylinder(radius=0.014, length=0.300),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=latch_metal,
        name="sliding_hinge_pin",
    )
    slider.visual(
        Box((0.210, 0.028, 0.018)),
        origin=Origin(xyz=(0.105, -0.060, 0.006)),
        material=dark_metal,
        name="locking_tongue",
    )
    slider.visual(
        Cylinder(radius=0.016, length=0.030),
        origin=Origin(xyz=(0.205, -0.060, 0.006), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=latch_metal,
        name="latch_nose",
    )

    front_leg = model.part("front_leg")
    front_leg.visual(
        _tube(
            [
                (0.000, -0.160, 0.000),
                (-0.220, -0.158, -0.245),
                (-0.475, -0.156, -0.535),
                (-0.720, -0.160, -0.785),
                (-0.720, 0.160, -0.785),
                (-0.475, 0.156, -0.535),
                (-0.220, 0.158, -0.245),
                (0.000, 0.160, 0.000),
            ],
            0.013,
            name="front_tube_frame",
        ),
        material=dark_metal,
        name="front_tube_frame",
    )
    front_leg.visual(
        Cylinder(radius=0.023, length=0.090),
        origin=Origin(xyz=(-0.720, -0.205, -0.785), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="front_foot_0",
    )
    front_leg.visual(
        Cylinder(radius=0.023, length=0.090),
        origin=Origin(xyz=(-0.720, 0.205, -0.785), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="front_foot_1",
    )
    front_leg.visual(
        Box((0.020, 0.020, 0.018)),
        origin=Origin(xyz=(-0.340, 0.160, -0.390), rpy=(0.0, 0.0, 0.35)),
        material=latch_metal,
        name="front_cross_pivot_washer",
    )

    rear_leg = model.part("rear_leg")
    rear_leg.visual(
        _tube(
            [
                (0.000, -0.112, 0.000),
                (0.205, -0.112, -0.260),
                (0.435, -0.112, -0.535),
                (0.680, -0.112, -0.785),
                (0.680, 0.112, -0.785),
                (0.435, 0.112, -0.535),
                (0.205, 0.112, -0.260),
                (0.000, 0.112, 0.000),
            ],
            0.012,
            name="rear_tube_frame",
        ),
        material=dark_metal,
        name="rear_tube_frame",
    )
    rear_leg.visual(
        Cylinder(radius=0.023, length=0.090),
        origin=Origin(xyz=(0.680, -0.157, -0.785), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="rear_foot_0",
    )
    rear_leg.visual(
        Cylinder(radius=0.023, length=0.090),
        origin=Origin(xyz=(0.680, 0.157, -0.785), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="rear_foot_1",
    )
    rear_leg.visual(
        Cylinder(radius=0.006, length=0.245),
        origin=Origin(xyz=(0.335, 0.0, -0.405), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=latch_metal,
        name="rear_cross_pivot_pin",
    )

    model.articulation(
        "board_to_height_slider",
        ArticulationType.PRISMATIC,
        parent=board,
        child=slider,
        origin=Origin(xyz=(-0.200, 0.0, 0.790)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.120, effort=45.0, velocity=0.15),
    )
    model.articulation(
        "board_to_front_leg",
        ArticulationType.REVOLUTE,
        parent=board,
        child=front_leg,
        origin=Origin(xyz=(0.300, 0.0, 0.790)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.05, effort=35.0, velocity=1.2),
    )
    model.articulation(
        "slider_to_rear_leg",
        ArticulationType.REVOLUTE,
        parent=slider,
        child=rear_leg,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.82, effort=35.0, velocity=1.2),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    board = object_model.get_part("board")
    front_leg = object_model.get_part("front_leg")
    rear_leg = object_model.get_part("rear_leg")
    slider = object_model.get_part("height_slider")
    front_hinge = object_model.get_articulation("board_to_front_leg")
    rear_hinge = object_model.get_articulation("slider_to_rear_leg")
    height_slide = object_model.get_articulation("board_to_height_slider")

    ctx.allow_overlap(
        board,
        slider,
        elem_a="height_notch_rail",
        elem_b="latch_nose",
        reason="The spring latch nose is intentionally seated into the notched height rail.",
    )
    ctx.allow_overlap(
        slider,
        rear_leg,
        elem_a="sliding_hinge_pin",
        elem_b="rear_tube_frame",
        reason="The rear leg tube is represented as a solid tube captured around the hinge pin.",
    )
    ctx.allow_overlap(
        board,
        front_leg,
        elem_a="front_hinge_bracket_0",
        elem_b="front_tube_frame",
        reason="The front leg tube is intentionally cradled inside the underside hinge bracket.",
    )
    ctx.allow_overlap(
        board,
        front_leg,
        elem_a="front_hinge_bracket_1",
        elem_b="front_tube_frame",
        reason="The opposite front leg tube is intentionally cradled inside its underside hinge bracket.",
    )
    ctx.allow_overlap(
        board,
        slider,
        elem_a="slider_guide_bracket_0",
        elem_b="sliding_hinge_pin",
        reason="The sliding cross-pin is intentionally captured in the underside guide bracket.",
    )
    ctx.allow_overlap(
        board,
        slider,
        elem_a="slider_guide_bracket_1",
        elem_b="sliding_hinge_pin",
        reason="The opposite end of the sliding cross-pin is intentionally captured in the guide bracket.",
    )

    ctx.check(
        "classification note",
        object_model.meta.get("note") == "Reference and category agree: a folding ironing board is modeled as the visible core object.",
        details=str(object_model.meta),
    )
    ctx.expect_overlap(
        board,
        front_leg,
        axes="xy",
        elem_a="front_hinge_bracket_0",
        elem_b="front_tube_frame",
        min_overlap=0.005,
        name="front leg visually reaches underside hinge brackets",
    )
    ctx.expect_overlap(
        board,
        slider,
        axes="xy",
        elem_a="height_notch_rail",
        elem_b="locking_tongue",
        min_overlap=0.020,
        name="height latch aligns with notched rail",
    )
    ctx.expect_overlap(
        board,
        slider,
        axes="xy",
        elem_a="height_notch_rail",
        elem_b="latch_nose",
        min_overlap=0.015,
        name="latch nose is seated in a height notch",
    )
    ctx.expect_overlap(
        slider,
        rear_leg,
        axes="yz",
        elem_a="sliding_hinge_pin",
        elem_b="rear_tube_frame",
        min_overlap=0.010,
        name="rear leg is captured on sliding hinge pin",
    )
    ctx.expect_overlap(
        board,
        front_leg,
        axes="yz",
        elem_a="front_hinge_bracket_0",
        elem_b="front_tube_frame",
        min_overlap=0.010,
        name="front hinge bracket captures leg tube",
    )
    ctx.expect_overlap(
        board,
        front_leg,
        axes="yz",
        elem_a="front_hinge_bracket_1",
        elem_b="front_tube_frame",
        min_overlap=0.010,
        name="opposite front hinge bracket captures leg tube",
    )
    ctx.expect_overlap(
        board,
        slider,
        axes="yz",
        elem_a="slider_guide_bracket_0",
        elem_b="sliding_hinge_pin",
        min_overlap=0.010,
        name="sliding pin stays in guide bracket",
    )
    ctx.expect_overlap(
        board,
        slider,
        axes="yz",
        elem_a="slider_guide_bracket_1",
        elem_b="sliding_hinge_pin",
        min_overlap=0.010,
        name="opposite sliding pin stays in guide bracket",
    )
    ctx.expect_gap(
        board,
        front_leg,
        axis="z",
        positive_elem="fabric_cover",
        negative_elem="front_foot_0",
        min_gap=0.760,
        name="open board stands high above front feet",
    )
    ctx.expect_gap(
        board,
        rear_leg,
        axis="z",
        positive_elem="fabric_cover",
        negative_elem="rear_foot_0",
        min_gap=0.760,
        name="open board stands high above rear feet",
    )

    rest_front_aabb = ctx.part_element_world_aabb(front_leg, elem="front_foot_0")
    rest_rear_aabb = ctx.part_element_world_aabb(rear_leg, elem="rear_foot_0")
    rest_slider_pos = ctx.part_world_position(slider)
    with ctx.pose({front_hinge: 0.85, rear_hinge: 0.70, height_slide: 0.10}):
        folded_front_aabb = ctx.part_element_world_aabb(front_leg, elem="front_foot_0")
        folded_rear_aabb = ctx.part_element_world_aabb(rear_leg, elem="rear_foot_0")
        slid_pos = ctx.part_world_position(slider)
        ctx.expect_gap(
            board,
            front_leg,
            axis="z",
            positive_elem="fabric_cover",
            negative_elem="front_foot_0",
            max_gap=0.260,
            name="folded front foot approaches underside",
        )
        ctx.expect_gap(
            board,
            rear_leg,
            axis="z",
            positive_elem="fabric_cover",
            negative_elem="rear_foot_0",
            max_gap=0.360,
            name="folded rear foot approaches underside",
        )

    ctx.check(
        "front foot rises when folding",
        rest_front_aabb is not None
        and folded_front_aabb is not None
        and folded_front_aabb[0][2] > rest_front_aabb[0][2] + 0.10,
        details=f"rest={rest_front_aabb}, folded={folded_front_aabb}",
    )
    ctx.check(
        "rear foot rises when folding",
        rest_rear_aabb is not None
        and folded_rear_aabb is not None
        and folded_rear_aabb[0][2] > rest_rear_aabb[0][2] + 0.10,
        details=f"rest={rest_rear_aabb}, folded={folded_rear_aabb}",
    )
    ctx.check(
        "height slider travels along notched rail",
        rest_slider_pos is not None and slid_pos is not None and slid_pos[0] > rest_slider_pos[0] + 0.08,
        details=f"rest={rest_slider_pos}, slid={slid_pos}",
    )

    return ctx.report()


object_model = build_object_model()
