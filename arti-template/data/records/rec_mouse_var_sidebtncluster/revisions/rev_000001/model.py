from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    KnobGeometry,
    KnobGrip,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _mouse_section(
    x: float,
    left_width: float,
    right_width: float,
    height: float,
    *,
    y_bias: float = 0.0,
    z_base: float = 0.003,
    segments: int = 56,
) -> list[tuple[float, float, float]]:
    """Closed asymmetric superellipse cross-section in the YZ plane."""
    pts: list[tuple[float, float, float]] = []
    exponent = 2.55
    for i in range(segments):
        t = 2.0 * math.pi * i / segments
        c = math.cos(t)
        s = math.sin(t)
        # Slightly broader thumb side (+Y) and fuller right-hand palm side (-Y).
        half_w = left_width if c >= 0.0 else right_width
        y = y_bias + math.copysign(abs(c) ** (2.0 / exponent), c) * half_w
        # Squash the bottom so the mouse has a low usable underside, but keep a
        # continuous shell instead of a placeholder rectangular block.
        z01 = 0.5 + 0.5 * math.copysign(abs(s) ** (2.0 / exponent), s)
        z = z_base + height * z01
        if z < z_base + 0.002 and abs(c) < 0.75:
            z = z_base
        pts.append((x, y, z))
    return pts


def _make_body_shell() -> MeshGeometry:
    sections = [
        _mouse_section(-0.062, 0.022, 0.021, 0.012, y_bias=-0.0015),
        _mouse_section(-0.050, 0.033, 0.031, 0.026, y_bias=-0.0020),
        _mouse_section(-0.028, 0.039, 0.036, 0.040, y_bias=-0.0025),
        _mouse_section(-0.004, 0.040, 0.037, 0.043, y_bias=-0.0030),
        _mouse_section(0.020, 0.038, 0.035, 0.036, y_bias=-0.0020),
        _mouse_section(0.045, 0.033, 0.031, 0.024, y_bias=-0.0010),
        _mouse_section(0.065, 0.023, 0.022, 0.011, y_bias=0.0000),
    ]

    geom = MeshGeometry()
    rows: list[list[int]] = []
    for section in sections:
        rows.append([geom.add_vertex(*p) for p in section])

    count = len(rows[0])
    for a, b in zip(rows[:-1], rows[1:]):
        for i in range(count):
            j = (i + 1) % count
            geom.add_face(a[i], b[i], b[j])
            geom.add_face(a[i], b[j], a[j])

    # End caps.
    rear_center = geom.add_vertex(sections[0][0][0], 0.0, 0.012)
    front_center = geom.add_vertex(sections[-1][0][0], 0.0, 0.009)
    for i in range(count):
        j = (i + 1) % count
        geom.add_face(rear_center, rows[0][j], rows[0][i])
        geom.add_face(front_center, rows[-1][i], rows[-1][j])

    return geom


def _rounded_button(width: float, depth: float, radius: float, thickness: float, name: str):
    # Low-profile rounded rectangle button, exported as a mesh so it reads like
    # molded plastic rather than a hard-edged placeholder.
    seg = 5
    pts: list[tuple[float, float]] = []
    centers = [
        (width / 2 - radius, depth / 2 - radius, 0.0),
        (-width / 2 + radius, depth / 2 - radius, math.pi / 2),
        (-width / 2 + radius, -depth / 2 + radius, math.pi),
        (width / 2 - radius, -depth / 2 + radius, 3 * math.pi / 2),
    ]
    for cx, cy, start in centers:
        for k in range(seg + 1):
            a = start + (math.pi / 2) * k / seg
            pts.append((cx + radius * math.cos(a), cy + radius * math.sin(a)))
    return mesh_from_geometry(ExtrudeGeometry(pts, thickness, center=True), name)


def _click_panel_mesh(is_left: bool, name: str):
    # Trapezoidal top button footprint with an angled outside edge and central
    # slot clearance for the scroll wheel.
    if is_left:
        pts = [
            (-0.032, -0.012),
            (0.030, -0.012),
            (0.034, 0.010),
            (0.020, 0.017),
            (-0.026, 0.016),
            (-0.034, 0.006),
        ]
    else:
        pts = [
            (-0.032, 0.012),
            (0.030, 0.012),
            (0.034, -0.010),
            (0.020, -0.017),
            (-0.026, -0.016),
            (-0.034, -0.006),
        ]
    return mesh_from_geometry(ExtrudeGeometry(pts, 0.0032, center=True), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="black_wireless_gaming_mouse")

    matte_black = model.material("matte_black", rgba=(0.005, 0.005, 0.006, 1.0))
    satin_black = model.material("satin_black", rgba=(0.018, 0.018, 0.020, 1.0))
    rubber_black = model.material("textured_rubber", rgba=(0.001, 0.001, 0.001, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.045, 0.045, 0.050, 1.0))
    logo_grey = model.material("subtle_logo_grey", rgba=(0.55, 0.55, 0.58, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(_make_body_shell(), "ergonomic_contoured_shell"),
        material=matte_black,
        name="ergonomic_shell",
    )
    # A slightly recessed front deck makes a physical seat below the click
    # panels and wheel opening without colliding with the moving caps.
    body.visual(
        Box((0.070, 0.052, 0.003)),
        origin=Origin(xyz=(0.031, 0.000, 0.0340)),
        material=satin_black,
        name="front_shelf",
    )
    body.visual(
        Box((0.020, 0.016, 0.0025)),
        origin=Origin(xyz=(-0.006, 0.000, 0.0450)),
        material=satin_black,
        name="dpi_seat",
    )
    # Wheel bearing cheeks leave the rubber wheel exposed between the buttons.
    body.visual(
        Box((0.017, 0.003, 0.010)),
        origin=Origin(xyz=(0.022, -0.0105, 0.0355)),
        material=satin_black,
        name="wheel_cheek_0",
    )
    body.visual(
        Box((0.017, 0.003, 0.010)),
        origin=Origin(xyz=(0.022, 0.0105, 0.0355)),
        material=satin_black,
        name="wheel_cheek_1",
    )

    # Textured rubber grips are molded into both sides; raised ribs overlap the
    # panel backing within the same root part so the detail reads supported.
    for side, y, grip_name in ((1.0, 0.0380, "left_grip"), (-1.0, -0.0380, "right_grip")):
        body.visual(
            Box((0.065, 0.003, 0.019)),
            origin=Origin(xyz=(-0.004, y, 0.0195), rpy=(0.0, 0.0, side * 0.06)),
            material=rubber_black,
            name=grip_name,
        )
        for i in range(9):
            x = -0.032 + i * 0.0076
            body.visual(
                Box((0.0022, 0.0038, 0.020)),
                origin=Origin(xyz=(x, y + side * 0.0012, 0.020), rpy=(0.0, side * 0.55, 0.0)),
                material=dark_grey,
                name=f"{grip_name}_rib_{i}",
            )

    # A raised rail/recess spanning the full 2-row × 3-column thumb button
    # cluster on the left (+Y) side (MMO-style 6-button grid).
    body.visual(
        Box((0.054, 0.003, 0.018)),
        origin=Origin(xyz=(0.009, 0.0388, 0.0290)),
        material=satin_black,
        name="thumb_recess",
    )

    # Per-button accent colors for the 6-button side cluster.
    thumb_accent_rgba = (
        (0.35, 0.06, 0.06, 1.0),   # 0 crimson
        (0.06, 0.28, 0.08, 1.0),   # 1 forest
        (0.06, 0.10, 0.35, 1.0),   # 2 cobalt
        (0.35, 0.25, 0.04, 1.0),   # 3 amber
        (0.22, 0.06, 0.30, 1.0),   # 4 violet
        (0.04, 0.25, 0.25, 1.0),   # 5 teal
    )
    thumb_accents = [
        model.material(f"thumb_accent_{i}", rgba=rgba)
        for i, rgba in enumerate(thumb_accent_rgba)
    ]

    # Subtle brand mark on the palm rest: a low-contrast stylized G-like glyph.
    body.visual(
        Cylinder(radius=0.0065, length=0.00045),
        origin=Origin(xyz=(-0.030, -0.010, 0.0413)),
        material=logo_grey,
        name="logo_disc",
    )
    body.visual(
        Box((0.006, 0.0012, 0.00055)),
        origin=Origin(xyz=(-0.027, -0.006, 0.04155), rpy=(0.0, 0.0, -0.25)),
        material=matte_black,
        name="logo_cut_stroke",
    )

    left_click = model.part("left_click")
    left_click.visual(
        _click_panel_mesh(True, "left_click_panel_mesh"),
        origin=Origin(),
        material=satin_black,
        name="button_plate",
    )
    right_click = model.part("right_click")
    right_click.visual(
        _click_panel_mesh(False, "right_click_panel_mesh"),
        origin=Origin(),
        material=satin_black,
        name="button_plate",
    )

    dpi_button = model.part("dpi_button")
    dpi_button.visual(
        _rounded_button(0.011, 0.016, 0.0025, 0.003, "dpi_button_cap_mesh"),
        origin=Origin(),
        material=dark_grey,
        name="button_cap",
    )
    dpi_button.visual(
        Box((0.006, 0.001, 0.0007)),
        origin=Origin(xyz=(0.0, 0.003, 0.0018)),
        material=logo_grey,
        name="dpi_mark",
    )

    scroll_wheel = model.part("scroll_wheel")
    wheel_geom = KnobGeometry(
        0.014,
        0.012,
        body_style="cylindrical",
        grip=KnobGrip(style="ribbed", count=18, depth=0.0008, width=0.0012),
        edge_radius=0.0007,
    )
    scroll_wheel.visual(
        mesh_from_geometry(wheel_geom, "ribbed_scroll_wheel"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber_black,
        name="rubber_wheel",
    )
    scroll_wheel.visual(
        Cylinder(radius=0.0017, length=0.018),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_grey,
        name="axle",
    )

    # 6-button MMO-style thumb cluster: 3 columns (X) × 2 rows (Z).
    _thumb_col_xs = (-0.007, 0.009, 0.025)
    _thumb_row_zs = (0.025, 0.034)
    for i in range(6):
        row = i // 3
        col = i % 3
        tx = _thumb_col_xs[col]
        tz = _thumb_row_zs[row]
        thumb = model.part(f"thumb_button_{i}")
        thumb.visual(
            _rounded_button(0.013, 0.005, 0.0018, 0.0035, f"thumb_button_{i}_mesh"),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_grey,
            name="button_cap",
        )
        # Small colored accent pip on each button face.
        thumb.visual(
            Box((0.004, 0.0006, 0.0012)),
            origin=Origin(xyz=(0.0, -0.002, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=thumb_accents[i],
            name=f"accent_{i}",
        )
        model.articulation(
            f"body_to_thumb_button_{i}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=thumb,
            origin=Origin(xyz=(tx, 0.0423, tz), rpy=(0.0, 0.0, 0.0)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=1.2, velocity=0.05, lower=0.0, upper=0.0025),
        )

    model.articulation(
        "body_to_left_click",
        ArticulationType.PRISMATIC,
        parent=body,
        child=left_click,
        origin=Origin(xyz=(0.030, 0.019, 0.0390)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=0.05, lower=0.0, upper=0.0020),
    )
    model.articulation(
        "body_to_right_click",
        ArticulationType.PRISMATIC,
        parent=body,
        child=right_click,
        origin=Origin(xyz=(0.030, -0.019, 0.0390)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=0.05, lower=0.0, upper=0.0020),
    )
    model.articulation(
        "body_to_dpi_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=dpi_button,
        origin=Origin(xyz=(-0.007, 0.000, 0.04775)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=0.04, lower=0.0, upper=0.0015),
    )
    model.articulation(
        "body_to_scroll_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=scroll_wheel,
        origin=Origin(xyz=(0.024, 0.000, 0.0408)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.15, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    left_click = object_model.get_part("left_click")
    right_click = object_model.get_part("right_click")
    dpi_button = object_model.get_part("dpi_button")
    scroll_wheel = object_model.get_part("scroll_wheel")
    thumb_buttons = [object_model.get_part(f"thumb_button_{i}") for i in range(6)]
    thumb_0 = thumb_buttons[0]

    ctx.allow_overlap(
        body,
        scroll_wheel,
        elem_a="ergonomic_shell",
        elem_b="rubber_wheel",
        reason=(
            "The scroll wheel is intentionally captured in a narrow top slot; "
            "the continuous shell mesh leaves the hidden slot throat simplified."
        ),
    )

    visual_names = {v.name for v in body.visuals}
    ctx.check(
        "mouse has textured side grips and subtle brand mark",
        {"left_grip", "right_grip", "logo_disc", "thumb_recess"}.issubset(visual_names)
        and sum(1 for name in visual_names if name.startswith("left_grip_rib_")) >= 8
        and sum(1 for name in visual_names if name.startswith("right_grip_rib_")) >= 8,
        details=f"body visuals={sorted(visual_names)}",
    )

    for part, joint_name in (
        (left_click, "body_to_left_click"),
        (right_click, "body_to_right_click"),
        (dpi_button, "body_to_dpi_button"),
    ):
        ctx.expect_gap(
            part,
            body,
            axis="z",
            min_gap=0.0,
            max_gap=0.008,
            positive_elem="button_plate" if part in (left_click, right_click) else "button_cap",
            negative_elem="front_shelf" if part in (left_click, right_click) else "dpi_seat",
            name=f"{joint_name} cap rests proud of its seat",
        )

    for thumb in thumb_buttons:
        ctx.expect_gap(
            thumb,
            body,
            axis="y",
            max_gap=0.006,
            max_penetration=0.0001,
            positive_elem="button_cap",
            negative_elem="thumb_recess",
            name=f"{thumb.name} sits just outside left thumb recess",
        )

    # Verify the 6-button MMO thumb cluster exists with per-button accents.
    ctx.check(
        "6-button thumb cluster present (thumb_button_0..5)",
        all(object_model.get_part(f"thumb_button_{i}") is not None for i in range(6)),
        details="All six thumb button parts must exist",
    )
    ctx.check(
        "thumb recess spans full cluster",
        "thumb_recess" in visual_names,
        details=f"body visuals={sorted(visual_names)}",
    )

    # Verify each button in the grid has its own PRISMATIC joint.
    for i in range(6):
        jname = f"body_to_thumb_button_{i}"
        art = object_model.get_articulation(jname)
        ctx.check(
            f"{jname} is PRISMATIC -Y press",
            art.articulation_type == ArticulationType.PRISMATIC,
            details=f"articulation_type={art.articulation_type}",
        )

    # Pose: press the top-right button (index 5) inward to prove articulation.
    left_joint = object_model.get_articulation("body_to_left_click")
    thumb_joint = object_model.get_articulation("body_to_thumb_button_0")
    thumb_5_joint = object_model.get_articulation("body_to_thumb_button_5")
    wheel_joint = object_model.get_articulation("body_to_scroll_wheel")
    left_rest = ctx.part_world_position(left_click)
    thumb_rest = ctx.part_world_position(thumb_0)
    thumb_5 = thumb_buttons[5]
    thumb5_rest = ctx.part_world_position(thumb_5)
    with ctx.pose({left_joint: 0.0015, thumb_joint: 0.0020, thumb_5_joint: 0.0020, wheel_joint: 0.8}):
        left_pressed = ctx.part_world_position(left_click)
        thumb_pressed = ctx.part_world_position(thumb_0)
        thumb5_pressed = ctx.part_world_position(thumb_5)
    ctx.check(
        "primary buttons press in realistic directions",
        left_rest is not None
        and left_pressed is not None
        and thumb_rest is not None
        and thumb_pressed is not None
        and left_pressed[2] < left_rest[2] - 0.001
        and thumb_pressed[1] < thumb_rest[1] - 0.0015,
        details=f"left rest={left_rest}, left pressed={left_pressed}, thumb rest={thumb_rest}, thumb pressed={thumb_pressed}",
    )
    ctx.check(
        "thumb_button_5 (top-right cluster) presses inward on -Y",
        thumb5_rest is not None
        and thumb5_pressed is not None
        and thumb5_pressed[1] < thumb5_rest[1] - 0.0015,
        details=f"rest={thumb5_rest}, pressed={thumb5_pressed}",
    )
    ctx.expect_overlap(
        scroll_wheel,
        body,
        axes="xy",
        min_overlap=0.010,
        elem_a="rubber_wheel",
        elem_b="ergonomic_shell",
        name="scroll wheel sits in the central top slot",
    )
    ctx.expect_contact(
        scroll_wheel,
        body,
        contact_tol=0.0006,
        elem_a="axle",
        elem_b="wheel_cheek_0",
        name="scroll wheel axle touches rear bearing cheek",
    )
    ctx.expect_contact(
        scroll_wheel,
        body,
        contact_tol=0.0006,
        elem_a="axle",
        elem_b="wheel_cheek_1",
        name="scroll wheel axle touches front bearing cheek",
    )

    return ctx.report()


object_model = build_object_model()
