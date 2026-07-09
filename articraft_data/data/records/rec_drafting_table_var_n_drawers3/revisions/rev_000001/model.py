from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    KnobRelief,
    Material,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)

TABLE_TILT = 0.30


def _circle_profile(radius: float, cx: float = 0.0, cy: float = 0.0, segments: int = 32):
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cy + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _origin_on_tilted_top(
    x: float,
    y_from_hinge: float,
    z_from_hinge: float,
    *,
    tilt: float = TABLE_TILT,
) -> Origin:
    """Place a tabletop-mounted visual in a board-local frame rotated about +X."""
    c = math.cos(tilt)
    s = math.sin(tilt)
    return Origin(
        xyz=(x, y_from_hinge * c - z_from_hinge * s, y_from_hinge * s + z_from_hinge * c),
        rpy=(tilt, 0.0, 0.0),
    )


def _add_tube(part, start, end, thickness: float, name: str, material: Material) -> None:
    """Rectangular metal tube aligned with the vector from start to end."""
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    pitch = -math.asin(dz / length) if length else 0.0
    part.visual(
        Box((length, thickness, thickness)),
        origin=Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def _add_tilted_box(
    part,
    size,
    x: float,
    y_from_hinge: float,
    z_from_hinge: float,
    name: str,
    material: Material,
) -> None:
    part.visual(
        Box(size),
        origin=_origin_on_tilted_top(x, y_from_hinge, z_from_hinge),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="tilting_drafting_table",
        meta={
            "category": "Workspace / Drafting table",
            "reference_note": "Image and folder agree: drafting table with tilting wood top and black metal base.",
        },
    )

    wood = model.material("pale_wood", rgba=(0.92, 0.70, 0.40, 1.0))
    wood_edge = model.material("wood_edge", rgba=(0.74, 0.45, 0.22, 1.0))
    grain = model.material("subtle_wood_grain", rgba=(0.58, 0.37, 0.16, 1.0))
    grain_light = model.material("fine_golden_grain", rgba=(0.78, 0.55, 0.26, 1.0))
    black = model.material("satin_black_metal", rgba=(0.02, 0.022, 0.020, 1.0))
    dark = model.material("dark_recess", rgba=(0.0, 0.0, 0.0, 1.0))
    rubber = model.material("black_rubber", rgba=(0.006, 0.006, 0.005, 1.0))
    steel = model.material("brushed_steel", rgba=(0.82, 0.78, 0.70, 1.0))

    frame = model.part("frame")

    # Grounded metal stand: splayed legs, feet, crossbars, drawer rail, and mesh basket.
    _add_tube(frame, (-0.52, -0.48, 0.045), (-0.52, 0.36, 0.045), 0.045, "foot_bar_0", black)
    _add_tube(frame, (0.52, -0.48, 0.045), (0.52, 0.36, 0.045), 0.045, "foot_bar_1", black)
    _add_tube(frame, (-0.52, -0.42, 0.07), (-0.49, -0.24, 0.67), 0.046, "front_leg_0", black)
    _add_tube(frame, (0.52, -0.42, 0.07), (0.49, -0.24, 0.67), 0.046, "front_leg_1", black)
    _add_tube(frame, (-0.52, 0.32, 0.07), (-0.49, 0.18, 0.72), 0.046, "rear_leg_0", black)
    _add_tube(frame, (0.52, 0.32, 0.07), (0.49, 0.18, 0.72), 0.046, "rear_leg_1", black)
    _add_tube(frame, (-0.55, -0.24, 0.67), (0.55, -0.24, 0.67), 0.045, "front_top_rail", black)
    _add_tube(frame, (-0.55, 0.18, 0.72), (0.55, 0.18, 0.72), 0.045, "rear_top_rail", black)
    _add_tube(frame, (-0.51, -0.25, 0.67), (-0.51, 0.19, 0.72), 0.040, "side_top_rail_0", black)
    _add_tube(frame, (0.51, -0.25, 0.67), (0.51, 0.19, 0.72), 0.040, "side_top_rail_1", black)
    _add_tube(frame, (-0.52, -0.32, 0.33), (0.52, -0.32, 0.33), 0.032, "basket_bottom_rail", black)
    _add_tube(frame, (-0.52, -0.33, 0.54), (0.52, -0.33, 0.54), 0.035, "drawer_lower_rail", black)
    _add_tube(frame, (-0.51, -0.32, 0.33), (-0.51, -0.32, 0.54), 0.030, "basket_side_0", black)
    _add_tube(frame, (0.51, -0.32, 0.33), (0.51, -0.32, 0.54), 0.030, "basket_side_1", black)
    _add_tube(frame, (-0.34, -0.31, 0.52), (-0.27, 0.05, 0.67), 0.026, "ratchet_brace_0", black)
    _add_tube(frame, (0.34, -0.31, 0.52), (0.27, 0.05, 0.67), 0.026, "ratchet_brace_1", black)

    # Front drawer apron with three ring pulls, evenly spaced.
    frame.visual(
        Box((1.02, 0.060, 0.170)),
        origin=Origin(xyz=(0.0, -0.365, 0.585)),
        material=black,
        name="drawer_apron",
    )
    drawer_count = 3
    apron_half = 0.51
    cell_width = (2.0 * apron_half) / drawer_count
    drawer_centers = [-apron_half + cell_width * (i + 0.5) for i in range(drawer_count)]
    # Dividers between adjacent drawer cells.
    for i in range(drawer_count - 1):
        div_x = -apron_half + cell_width * (i + 1)
        frame.visual(
            Box((0.012, 0.064, 0.170)),
            origin=Origin(xyz=(div_x, -0.398, 0.585)),
            material=dark,
            name=f"drawer_divider_{i}",
        )
    ring_mesh = mesh_from_geometry(TorusGeometry(0.035, 0.006, radial_segments=18, tubular_segments=36), "drawer_pull_ring")
    for i, x in enumerate(drawer_centers):
        frame.visual(
            Cylinder(radius=0.030, length=0.012),
            origin=Origin(xyz=(x, -0.396, 0.585), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"pull_mount_{i}",
        )
        frame.visual(
            Cylinder(radius=0.021, length=0.014),
            origin=Origin(xyz=(x, -0.404, 0.585), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark,
            name=f"pull_dark_center_{i}",
        )
        frame.visual(
            ring_mesh,
            origin=Origin(xyz=(x, -0.404, 0.585), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"pull_ring_{i}",
        )

    basket_mesh = mesh_from_geometry(
        PerforatedPanelGeometry(
            (0.96, 0.23),
            0.006,
            hole_diameter=0.010,
            pitch=(0.024, 0.020),
            frame=0.014,
            stagger=True,
            center=True,
        ),
        "front_mesh_basket",
    )
    frame.visual(
        basket_mesh,
        origin=Origin(xyz=(0.0, -0.404, 0.425), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="perforated_basket",
    )

    # Hinge rail and visible pivot bosses under the front edge of the tilting board.
    frame.visual(
        Cylinder(radius=0.025, length=1.12),
        origin=Origin(xyz=(0.0, -0.300, 0.720), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="front_hinge_tube",
    )
    for i, x in enumerate((-0.48, 0.48)):
        frame.visual(
            Box((0.060, 0.080, 0.090)),
            origin=Origin(xyz=(x, -0.305, 0.680)),
            material=black,
            name=f"hinge_cheek_{i}",
        )

    # Left side fixed auxiliary shelf from the reference, carried by black metal brackets.
    frame.visual(
        Box((0.46, 0.42, 0.030)),
        origin=Origin(xyz=(-0.83, -0.13, 0.625)),
        material=wood,
        name="side_shelf",
    )
    frame.visual(
        Box((0.46, 0.018, 0.040)),
        origin=Origin(xyz=(-0.83, -0.34, 0.613)),
        material=wood_edge,
        name="side_shelf_front_edge",
    )
    _add_tube(frame, (-0.54, -0.34, 0.555), (-0.89, -0.30, 0.618), 0.025, "shelf_bracket_0", black)
    _add_tube(frame, (-0.54, -0.24, 0.640), (-0.89, 0.02, 0.618), 0.025, "shelf_bracket_1", black)
    for i, (x, y) in enumerate(((-0.89, -0.30), (-0.89, 0.02))):
        frame.visual(
            Box((0.070, 0.036, 0.030)),
            origin=Origin(xyz=(x, y, 0.626)),
            material=black,
            name=f"shelf_bracket_outer_mount_{i}",
        )

    # Small leveling feet/casters below the foot bars.
    for i, (x, y) in enumerate(((-0.52, -0.48), (-0.52, 0.36), (0.52, -0.48), (0.52, 0.36))):
        frame.visual(
            Cylinder(radius=0.040, length=0.020),
            origin=Origin(xyz=(x, y, 0.0125)),
            material=rubber,
            name=f"leveling_foot_{i}",
        )

    # The moving tabletop is authored in a hinge-local frame and pre-posed to the
    # reference's moderate drafting angle.  The revolute joint can flatten or steepen it.
    top = model.part("tabletop")
    depth = 0.76
    thickness = 0.035
    top_lift = 0.028
    wood_width = 1.08
    _add_tilted_box(top, (wood_width, depth, thickness), -0.075, depth * 0.5, thickness * 0.5 + top_lift, "wood_panel", wood)
    _add_tilted_box(top, (wood_width, 0.034, 0.044), -0.075, 0.016, 0.052 + top_lift, "front_paper_stop", black)
    _add_tilted_box(top, (wood_width, 0.030, 0.046), -0.075, depth - 0.010, 0.012 + top_lift, "rear_wood_edge", wood_edge)
    _add_tilted_box(top, (0.035, depth, 0.040), -0.625, depth * 0.5, 0.012 + top_lift, "side_wood_edge", wood_edge)

    # Right-side black tool rail with through-holes like the photographed tray strip.
    rail_outer = rounded_rect_profile(0.170, 0.730, 0.012, corner_segments=5)
    rail_holes = [
        _circle_profile(0.046, 0.0, 0.220, 36),
        _circle_profile(0.027, 0.0, 0.020, 30),
        _circle_profile(0.024, 0.0, -0.105, 30),
        _circle_profile(0.020, 0.0, -0.215, 28),
        _circle_profile(0.010, -0.040, -0.025, 20),
        _circle_profile(0.010, 0.040, -0.025, 20),
    ]
    rail_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(rail_outer, rail_holes, 0.018, center=True),
        "tool_tray_rail",
    )
    top.visual(
        rail_mesh,
        origin=_origin_on_tilted_top(0.550, depth * 0.5, thickness + 0.008 + top_lift),
        material=black,
        name="tool_tray_rail",
    )
    # Hollow cup sleeve in the tool rail, represented by an open black cylinder.
    top.visual(
        Cylinder(radius=0.050, length=0.055),
        origin=_origin_on_tilted_top(0.550, 0.590, thickness + 0.018 + top_lift),
        material=dark,
        name="cup_socket",
    )

    # Tabletop hinge knuckles and underside adjustment/ratchet bars.
    top.visual(
        Cylinder(radius=0.024, length=0.185),
        origin=Origin(xyz=(-0.34, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="hinge_barrel_0",
    )
    top.visual(
        Box((0.170, 0.066, 0.014)),
        origin=_origin_on_tilted_top(-0.34, 0.057, 0.031),
        material=black,
        name="hinge_leaf_0",
    )
    top.visual(
        Box((0.160, 0.032, 0.030)),
        origin=Origin(xyz=(-0.34, 0.0365, 0.025)),
        material=black,
        name="hinge_web_0",
    )
    top.visual(
        Cylinder(radius=0.024, length=0.185),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="center_hinge_barrel",
    )
    top.visual(
        Box((0.170, 0.066, 0.014)),
        origin=_origin_on_tilted_top(0.0, 0.057, 0.031),
        material=black,
        name="center_hinge_leaf",
    )
    top.visual(
        Box((0.160, 0.032, 0.030)),
        origin=Origin(xyz=(0.0, 0.0365, 0.025)),
        material=black,
        name="center_hinge_web",
    )
    top.visual(
        Cylinder(radius=0.024, length=0.185),
        origin=Origin(xyz=(0.34, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="hinge_barrel_2",
    )
    top.visual(
        Box((0.170, 0.066, 0.014)),
        origin=_origin_on_tilted_top(0.34, 0.057, 0.031),
        material=black,
        name="hinge_leaf_2",
    )
    top.visual(
        Box((0.160, 0.032, 0.030)),
        origin=Origin(xyz=(0.34, 0.0365, 0.025)),
        material=black,
        name="hinge_web_2",
    )
    for i, x in enumerate((-0.48, 0.48)):
        _add_tilted_box(top, (0.035, 0.450, 0.026), x, 0.250, 0.015, f"under_brace_{i}", black)

    # Fine low-relief wood streaks stay inside the usable board face.
    fine_grain = (
        (-0.17, 0.105, 0.72, 0.004, grain_light),
        (-0.20, 0.155, 0.66, 0.005, grain),
        (-0.10, 0.205, 0.74, 0.004, grain_light),
        (-0.16, 0.255, 0.62, 0.004, grain),
        (-0.06, 0.310, 0.70, 0.005, grain_light),
        (-0.18, 0.365, 0.60, 0.004, grain),
        (-0.05, 0.420, 0.68, 0.004, grain_light),
        (-0.19, 0.475, 0.58, 0.004, grain),
        (-0.07, 0.530, 0.64, 0.005, grain_light),
        (-0.21, 0.585, 0.54, 0.004, grain),
        (-0.03, 0.640, 0.56, 0.004, grain_light),
    )
    for i, (x, y, length, width_y, material) in enumerate(fine_grain):
        top.visual(
            Box((length, width_y, 0.0016)),
            origin=_origin_on_tilted_top(x, y, thickness + 0.001 + top_lift),
            material=material,
            name=f"wood_grain_{i}",
        )

    model.articulation(
        "frame_to_tabletop",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=top,
        origin=Origin(xyz=(0.0, -0.300, 0.720)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.7, lower=-0.30, upper=0.45),
    )

    # Hand-tightened side lock knobs are separate rotating child links, mounted on the side cheeks.
    knob_geom = KnobGeometry(
        0.075,
        0.035,
        body_style="lobed",
        base_diameter=0.052,
        top_diameter=0.070,
        grip=KnobGrip(style="ribbed", count=10, depth=0.002),
        bore=KnobBore(style="round", diameter=0.014),
        body_reliefs=(KnobRelief(style="top_recess", width=0.026, depth=0.002),),
        center=False,
    )
    knob_mesh = mesh_from_geometry(knob_geom, "side_lock_knob")
    for name, x, stem_x, rot_y in (
        ("lock_knob_0", -0.525, 0.0125, -math.pi / 2.0),
        ("lock_knob_1", 0.525, -0.0125, math.pi / 2.0),
    ):
        knob = model.part(name)
        knob.visual(
            Cylinder(radius=0.012, length=0.025),
            origin=Origin(xyz=(stem_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name="threaded_stem",
        )
        knob.visual(
            knob_mesh,
            origin=Origin(rpy=(0.0, rot_y, 0.0)),
            material=black,
            name="lobed_knob",
        )
        model.articulation(
            f"frame_to_{name}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=knob,
            origin=Origin(xyz=(x, -0.325, 0.635)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=6.0, velocity=4.0, lower=-math.pi, upper=math.pi),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    tabletop = object_model.get_part("tabletop")
    tilt = object_model.get_articulation("frame_to_tabletop")

    ctx.check(
        "drafting table has articulated tilt top",
        tilt.motion_limits is not None
        and tilt.motion_limits.lower is not None
        and tilt.motion_limits.upper is not None
        and tilt.motion_limits.lower < 0.0
        and tilt.motion_limits.upper > 0.35,
        details=f"limits={tilt.motion_limits}",
    )
    for joint_name in ("frame_to_lock_knob_0", "frame_to_lock_knob_1"):
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} is a rotary lock control",
            joint.articulation_type == ArticulationType.REVOLUTE
            and joint.motion_limits is not None
            and joint.motion_limits.upper is not None
            and joint.motion_limits.upper >= math.pi,
            details=f"type={joint.articulation_type}, limits={joint.motion_limits}",
        )

    # Verify the three-drawer apron bank: N=3 ring pulls and N-1=2 dividers.
    for i in range(3):
        ctx.check(
            f"drawer_{i} pull_ring_{i} exists on apron",
            any(v.name == f"pull_ring_{i}" for v in frame.visuals),
            details=f"pull_ring_{i} not found on frame",
        )
    for i in range(2):
        ctx.check(
            f"drawer_divider_{i} exists between cells",
            any(v.name == f"drawer_divider_{i}" for v in frame.visuals),
            details=f"drawer_divider_{i} not found on frame",
        )

    ctx.allow_overlap(
        frame,
        tabletop,
        elem_a="front_hinge_tube",
        elem_b="center_hinge_barrel",
        reason="The tabletop hinge barrel is a solid proxy around the frame hinge pin.",
    )
    ctx.allow_overlap(
        frame,
        tabletop,
        elem_a="front_hinge_tube",
        elem_b="hinge_barrel_0",
        reason="The tabletop hinge barrel is a solid proxy around the frame hinge pin.",
    )
    ctx.allow_overlap(
        frame,
        tabletop,
        elem_a="front_hinge_tube",
        elem_b="hinge_barrel_2",
        reason="The tabletop hinge barrel is a solid proxy around the frame hinge pin.",
    )
    ctx.expect_overlap(
        tabletop,
        frame,
        axes="x",
        elem_a="center_hinge_barrel",
        elem_b="front_hinge_tube",
        min_overlap=0.10,
        name="tabletop hinge barrels align across the frame hinge tube",
    )
    ctx.expect_overlap(
        tabletop,
        frame,
        axes="x",
        elem_a="hinge_barrel_0",
        elem_b="front_hinge_tube",
        min_overlap=0.10,
        name="outer hinge barrel 0 aligns with the frame hinge tube",
    )
    ctx.expect_overlap(
        tabletop,
        frame,
        axes="x",
        elem_a="hinge_barrel_2",
        elem_b="front_hinge_tube",
        min_overlap=0.10,
        name="outer hinge barrel 2 aligns with the frame hinge tube",
    )
    for knob_name, cheek_name in (("lock_knob_0", "hinge_cheek_0"), ("lock_knob_1", "hinge_cheek_1")):
        ctx.allow_overlap(
            frame,
            knob_name,
            elem_a=cheek_name,
            elem_b="threaded_stem",
            reason="The threaded lock stem intentionally enters the side hinge cheek.",
        )
        ctx.expect_overlap(
            knob_name,
            frame,
            axes="x",
            elem_a="threaded_stem",
            elem_b=cheek_name,
            min_overlap=0.008,
            name=f"{knob_name} threaded stem is retained in the side cheek",
        )

    rest_aabb = ctx.part_element_world_aabb(tabletop, elem="wood_panel")
    with ctx.pose({tilt: 0.35}):
        raised_aabb = ctx.part_element_world_aabb(tabletop, elem="wood_panel")
    ctx.check(
        "positive tabletop tilt raises the rear edge",
        rest_aabb is not None
        and raised_aabb is not None
        and raised_aabb[1][2] > rest_aabb[1][2] + 0.12,
        details=f"rest={rest_aabb}, raised={raised_aabb}",
    )

    frame_aabb = ctx.part_world_aabb(frame)
    top_aabb = ctx.part_world_aabb(tabletop)
    ctx.check(
        "full size drafting table proportions",
        frame_aabb is not None
        and top_aabb is not None
        and (top_aabb[1][0] - top_aabb[0][0]) > 1.15
        and (frame_aabb[1][2] - frame_aabb[0][2]) > 0.65,
        details=f"frame={frame_aabb}, tabletop={top_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
