from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
)


def _rounded_box_mesh(sx: float, sy: float, sz: float, radius: float, name: str):
    """Small manufactured rounded rectangle, authored with its bottom on z=0."""
    r = min(radius, sx * 0.45, sy * 0.45, sz * 0.45)
    solid = (
        cq.Workplane("XY")
        .box(sx, sy, sz)
        .translate((0.0, 0.0, sz / 2.0))
        .edges("|Z")
        .fillet(r)
        .edges(">Z")
        .fillet(min(r * 0.45, sz * 0.35))
    )
    return mesh_from_cadquery(solid, name, tolerance=0.00055, angular_tolerance=0.08)


def _mouse_footprint(scale_x: float, scale_y: float) -> list[tuple[float, float]]:
    # Smooth, symmetric gaming-mouse outline in top view: narrow nose, full palm,
    # rounded rear, and slightly pinched waist.
    base = [
        (0.000, 0.064),
        (0.017, 0.061),
        (0.029, 0.048),
        (0.034, 0.020),
        (0.031, -0.018),
        (0.024, -0.048),
        (0.010, -0.064),
        (0.000, -0.068),
        (-0.010, -0.064),
        (-0.024, -0.048),
        (-0.031, -0.018),
        (-0.034, 0.020),
        (-0.029, 0.048),
        (-0.017, 0.061),
    ]
    return [(x * scale_x, y * scale_y) for x, y in base]


def _mouse_body_mesh():
    # Lofted shell rather than a rectangular proxy: the sections taper inward and
    # upward to read as a single ergonomic matte-plastic housing.
    body = (
        cq.Workplane("XY")
        .spline(_mouse_footprint(1.00, 1.00))
        .close()
        .workplane(offset=0.014)
        .spline(_mouse_footprint(0.96, 0.985))
        .close()
        .workplane(offset=0.012)
        .spline(_mouse_footprint(0.84, 0.88))
        .close()
        .workplane(offset=0.004)
        .spline(_mouse_footprint(0.58, 0.55))
        .close()
        .loft(combine=True)
    )
    return mesh_from_cadquery(body, "main_shell", tolerance=0.00065, angular_tolerance=0.08)


def _palm_hump_mesh():
    # A subtle raised rear palm pad, separate visual but fused into the root part.
    hump = (
        cq.Workplane("XY")
        .ellipse(0.023, 0.039)
        .workplane(offset=0.006)
        .ellipse(0.021, 0.035)
        .workplane(offset=0.007)
        .ellipse(0.013, 0.022)
        .loft(combine=True)
        .translate((0.0, -0.024, 0.026))
    )
    return mesh_from_cadquery(hump, "palm_hump", tolerance=0.00065, angular_tolerance=0.08)


def _slot_mesh(length: float, width: float, thickness: float, name: str):
    # Pill-shaped cap/insert with its bottom on z=0.  The slot length is overall.
    slot_len = max(0.0, length - width)
    solid = cq.Workplane("XY").slot2D(slot_len, width).extrude(thickness)
    return mesh_from_cadquery(solid, name, tolerance=0.00045, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="white_wireless_gaming_mouse")

    matte_white = model.material("matte_white", rgba=(0.94, 0.94, 0.90, 1.0))
    seam_black = model.material("shadow_seam", rgba=(0.015, 0.016, 0.017, 1.0))
    rubber_black = model.material("ribbed_black_rubber", rgba=(0.025, 0.026, 0.027, 1.0))
    dark_gray = model.material("side_button_gray", rgba=(0.11, 0.12, 0.13, 1.0))
    cyan = model.material("rgb_cyan", rgba=(0.0, 0.75, 1.0, 1.0))
    magenta = model.material("rgb_magenta", rgba=(0.95, 0.10, 0.85, 1.0))
    soft_blue = model.material("rgb_soft_blue", rgba=(0.12, 0.42, 1.0, 1.0))
    satin_metal = model.material("satin_axle", rgba=(0.50, 0.52, 0.54, 1.0))

    shell = model.part("shell")
    shell.visual(_mouse_body_mesh(), material=matte_white, name="main_shell")
    shell.visual(_palm_hump_mesh(), material=matte_white, name="palm_hump")

    # Recesses, seams, and illuminated accent inserts are authored on the root
    # housing so the mouse reads correctly from the requested top view.
    shell.visual(
        _slot_mesh(0.034, 0.014, 0.0012, "wheel_well"),
        origin=Origin(xyz=(0.0, 0.043, 0.0312)),
        material=seam_black,
        name="wheel_well",
    )
    shell.visual(
        Box((0.0030, 0.058, 0.0010)),
        origin=Origin(xyz=(0.0, 0.026, 0.0322)),
        material=seam_black,
        name="center_split",
    )
    shell.visual(
        Box((0.062, 0.0012, 0.0010)),
        origin=Origin(xyz=(0.0, 0.011, 0.0315)),
        material=seam_black,
        name="button_rear_seam",
    )
    shell.visual(
        _slot_mesh(0.017, 0.005, 0.0016, "dpi_socket_glow"),
        origin=Origin(xyz=(0.0, 0.006, 0.0320)),
        material=cyan,
        name="dpi_socket_glow",
    )
    for i, (x, y, mat) in enumerate(
        [
            (-0.0245, 0.028, cyan),
            (0.0245, 0.028, magenta),
            (-0.0200, -0.035, soft_blue),
            (0.0200, -0.035, cyan),
        ]
    ):
        shell.visual(
            Box((0.0020, 0.027, 0.0022)),
            origin=Origin(xyz=(x, y, 0.0290), rpy=(0.0, 0.0, 0.10 if x < 0 else -0.10)),
            material=mat,
            name=f"rgb_strip_{i}",
        )

    # Static fork cheeks around the wheel well, visible as the real cradle that
    # carries the wheel axle.
    for x in (-0.0115, 0.0115):
        shell.visual(
            Box((0.0030, 0.023, 0.010)),
            origin=Origin(xyz=(x, 0.042, 0.0345)),
            material=seam_black,
            name=f"wheel_fork_{'neg' if x < 0 else 'pos'}",
        )
    shell.visual(
        Box((0.0050, 0.045, 0.010)),
        origin=Origin(xyz=(-0.0310, 0.0, 0.0245)),
        material=seam_black,
        name="side_button_recess",
    )

    left_click = model.part("click_0")
    left_click.visual(
        _rounded_box_mesh(0.0265, 0.049, 0.0042, 0.0032, "click_0_plate"),
        origin=Origin(xyz=(0.0, 0.0245, 0.0)),
        material=matte_white,
        name="click_plate",
    )
    right_click = model.part("click_1")
    right_click.visual(
        _rounded_box_mesh(0.0265, 0.049, 0.0042, 0.0032, "click_1_plate"),
        origin=Origin(xyz=(0.0, 0.0245, 0.0)),
        material=matte_white,
        name="click_plate",
    )

    scroll_wheel = model.part("scroll_wheel")
    scroll_wheel.visual(
        Cylinder(radius=0.0066, length=0.0135),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_black,
        name="wheel_tire",
    )
    scroll_wheel.visual(
        Cylinder(radius=0.0020, length=0.0220),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="wheel_axle",
    )
    for j, y in enumerate([-0.0048, -0.0032, -0.0016, 0.0, 0.0016, 0.0032, 0.0048]):
        rib_z = math.sqrt(max(0.0, 0.0066 * 0.0066 - y * y)) + 0.00035
        scroll_wheel.visual(
            Box((0.0140, 0.00055, 0.0009)),
            origin=Origin(xyz=(0.0, y, rib_z)),
            material=Material(f"rib_highlight_{j}", rgba=(0.20, 0.21, 0.21, 1.0)),
            name=f"tread_rib_{j}",
        )

    dpi_button = model.part("dpi_button")
    dpi_button.visual(
        _slot_mesh(0.013, 0.0062, 0.0030, "dpi_button_cap"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=matte_white,
        name="dpi_cap",
    )
    dpi_button.visual(
        Box((0.007, 0.003, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, -0.0010)),
        material=soft_blue,
        name="dpi_light_edge",
    )

    for idx, y in enumerate((0.010, -0.010)):
        side = model.part(f"side_button_{idx}")
        side.visual(
            _rounded_box_mesh(0.0042, 0.0185, 0.0065, 0.0015, f"side_button_{idx}_cap"),
            origin=Origin(xyz=(-0.0040, 0.0, -0.00325), rpy=(0.0, 0.0, 0.0)),
            material=dark_gray,
            name="side_cap",
        )
        model.articulation(
            f"shell_to_side_button_{idx}",
            ArticulationType.PRISMATIC,
            parent=shell,
            child=side,
            origin=Origin(xyz=(-0.0316, y, 0.0245)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.8, velocity=0.08, lower=0.0, upper=0.003),
        )

    # Individual user controls are articulated: mouse buttons depress, the wheel
    # scrolls, and the DPI/side buttons have short tactile travel.
    model.articulation(
        "shell_to_click_0",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=left_click,
        origin=Origin(xyz=(-0.0162, 0.012, 0.0320)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=1.5, lower=0.0, upper=0.075),
    )
    model.articulation(
        "shell_to_click_1",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=right_click,
        origin=Origin(xyz=(0.0162, 0.012, 0.0320)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=1.5, lower=0.0, upper=0.075),
    )
    model.articulation(
        "shell_to_scroll_wheel",
        ArticulationType.CONTINUOUS,
        parent=shell,
        child=scroll_wheel,
        origin=Origin(xyz=(0.0, 0.043, 0.040)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.15, velocity=20.0),
    )
    model.articulation(
        "shell_to_dpi_button",
        ArticulationType.PRISMATIC,
        parent=shell,
        child=dpi_button,
        origin=Origin(xyz=(0.0, 0.006, 0.0336)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=0.06, lower=0.0, upper=0.002),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    shell = object_model.get_part("shell")
    click_0 = object_model.get_part("click_0")
    click_1 = object_model.get_part("click_1")
    dpi_button = object_model.get_part("dpi_button")
    side_button_0 = object_model.get_part("side_button_0")

    click_joint_0 = object_model.get_articulation("shell_to_click_0")
    dpi_joint = object_model.get_articulation("shell_to_dpi_button")
    side_joint = object_model.get_articulation("shell_to_side_button_0")

    ctx.check(
        "separate articulated gaming controls",
        all(
            object_model.get_part(name) is not None
            for name in (
                "click_0",
                "click_1",
                "scroll_wheel",
                "dpi_button",
                "side_button_0",
                "side_button_1",
            )
        ),
        details="mouse should expose left/right clicks, wheel, DPI, and two side buttons as parts",
    )
    ctx.expect_gap(
        click_0,
        shell,
        axis="z",
        min_gap=0.001,
        max_gap=0.007,
        positive_elem="click_plate",
        negative_elem="main_shell",
        name="click plates sit just above shell",
    )
    ctx.expect_overlap(
        click_0,
        shell,
        axes="xy",
        min_overlap=0.020,
        elem_a="click_plate",
        elem_b="main_shell",
        name="left click panel follows the shell footprint",
    )
    ctx.expect_overlap(
        click_1,
        shell,
        axes="xy",
        min_overlap=0.020,
        elem_a="click_plate",
        elem_b="main_shell",
        name="right click panel follows the shell footprint",
    )

    rest_aabb = ctx.part_world_aabb(click_0)
    with ctx.pose({click_joint_0: 0.065}):
        pressed_aabb = ctx.part_world_aabb(click_0)
    ctx.check(
        "left click depresses downward",
        rest_aabb is not None
        and pressed_aabb is not None
        and pressed_aabb[0][2] < rest_aabb[0][2] - 0.0005,
        details=f"rest={rest_aabb}, pressed={pressed_aabb}",
    )

    dpi_rest = ctx.part_world_position(dpi_button)
    with ctx.pose({dpi_joint: 0.002}):
        dpi_pressed = ctx.part_world_position(dpi_button)
    ctx.check(
        "dpi button has short downward travel",
        dpi_rest is not None and dpi_pressed is not None and dpi_pressed[2] < dpi_rest[2] - 0.0015,
        details=f"rest={dpi_rest}, pressed={dpi_pressed}",
    )

    side_rest = ctx.part_world_position(side_button_0)
    with ctx.pose({side_joint: 0.003}):
        side_pressed = ctx.part_world_position(side_button_0)
    ctx.check(
        "side button presses inward",
        side_rest is not None and side_pressed is not None and side_pressed[0] > side_rest[0] + 0.0025,
        details=f"rest={side_rest}, pressed={side_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
