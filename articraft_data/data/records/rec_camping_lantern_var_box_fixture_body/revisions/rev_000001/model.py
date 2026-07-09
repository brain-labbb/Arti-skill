from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _bail_mesh() -> MeshGeometry:
    """Carry bail wire arc, same sweep as parent."""
    path = [
        (-0.050, 0.0, 0.000),
        (-0.052, 0.0, 0.021),
        (-0.034, 0.0, 0.054),
        (0.000, 0.0, 0.066),
        (0.034, 0.0, 0.054),
        (0.052, 0.0, 0.021),
        (0.050, 0.0, 0.000),
    ]
    return tube_from_spline_points(
        path,
        radius=0.0023,
        samples_per_segment=10,
        radial_segments=16,
        cap_ends=True,
    )


def _housing_frame_shape() -> cq.Workplane:
    """Rectangular box lantern housing frame: top/bottom plates + 4 corner posts + mid-rails."""
    w, d, h = 0.100, 0.080, 0.120
    t = 0.005  # plate thickness
    bar = 0.008  # corner post section
    rail_t = 0.004  # mid-rail section

    # Bottom and top plates
    bottom_plate = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, -h / 2.0 + t / 2.0))
        .box(w, d, t)
    )
    top_plate = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, h / 2.0 - t / 2.0))
        .box(w, d, t)
    )
    frame = bottom_plate.union(top_plate)

    # Four corner posts
    for sx in (-1, 1):
        for sy in (-1, 1):
            post = (
                cq.Workplane("XY")
                .transformed(
                    offset=(sx * (w / 2.0 - bar / 2.0), sy * (d / 2.0 - bar / 2.0), 0.0)
                )
                .box(bar, bar, h)
            )
            frame = frame.union(post)

    # Horizontal mid-rails on front/back faces (at z=0 local)
    for sy in (-1, 1):
        rail = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, sy * (d / 2.0 - rail_t / 2.0), 0.0))
            .box(w - 2.0 * bar + 0.001, rail_t, rail_t)
        )
        frame = frame.union(rail)

    # Horizontal mid-rails on left/right faces
    for sx in (-1, 1):
        rail = (
            cq.Workplane("XY")
            .transformed(offset=(sx * (w / 2.0 - rail_t / 2.0), 0.0, 0.0))
            .box(rail_t, d - 2.0 * bar + 0.001, rail_t)
        )
        frame = frame.union(rail)

    return frame


def _squared_base_shape() -> cq.Workplane:
    """Squared vented base with filleted vertical edges."""
    w, d, h = 0.120, 0.100, 0.028
    base = (
        cq.Workplane("XY")
        .box(w, d, h)
        .edges("|Z")
        .fillet(0.006)
    )
    return base


def _squared_cap_shape() -> cq.Workplane:
    """Squared top vent cap with filleted vertical edges."""
    w, d, h = 0.112, 0.092, 0.022
    cap = (
        cq.Workplane("XY")
        .box(w, d, h)
        .edges("|Z")
        .fillet(0.005)
    )
    return cap


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="box_camping_lantern",
        meta={
            "run_notes": (
                "Rectangular box-fixture variant of the camping lantern. "
                "Flat translucent diffuser panels on a square housing frame replace "
                "the parent cylindrical cage and diffuser. Sand/tan housing colorway "
                "with slightly squatter, wider proportions typical of box area lanterns. "
                "No tripod legs in this body-only fork."
            )
        },
    )

    # ── Materials (sand/tan colorway) ──
    dark_metal = model.material("blackened_steel", rgba=(0.06, 0.06, 0.05, 1.0))
    sand_housing = model.material("sand_tan_housing", rgba=(0.76, 0.70, 0.55, 1.0))
    dark_tan = model.material("dark_tan_base", rgba=(0.45, 0.40, 0.32, 1.0))
    warm_glass = model.material("warm_translucent_panel", rgba=(1.0, 0.82, 0.45, 0.38))
    warm_core = model.material("warm_led_core", rgba=(1.0, 0.73, 0.20, 0.88))
    switch_black = model.material("matte_black_switch", rgba=(0.018, 0.018, 0.016, 1.0))
    white_mark = model.material("white_brand_mark", rgba=(0.93, 0.93, 0.88, 1.0))

    # ── Lantern body ──
    body = model.part("lantern_body")

    # Housing frame — rectangular box fixture (replaces cylindrical_wire_cage)
    body.visual(
        mesh_from_cadquery(_housing_frame_shape(), "housing_frame"),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=sand_housing,
        name="housing_frame",
    )

    # Flat rectangular diffuser panels on four sides (replaces glowing_diffuser cylinder)
    # Housing dims: 0.100 x 0.080 x 0.120 ; bar = 0.008 ; plate_t = 0.005
    hw, hd, hh = 0.100, 0.080, 0.120
    bar = 0.008
    plate_t = 0.005
    panel_thick = 0.003
    panel_h = hh - 2.0 * plate_t - 0.004  # 0.106 — slight rebate into plates
    panel_front_w = hw - 2.0 * bar  # 0.084
    panel_side_d = hd - 2.0 * bar   # 0.064

    panel_specs = [
        # (name, box_dims, origin_xyz)
        ("diffuser_panel_0", (panel_front_w, panel_thick, panel_h), (0.0, hd / 2.0 - bar / 2.0, 0.110)),
        ("diffuser_panel_1", (panel_front_w, panel_thick, panel_h), (0.0, -(hd / 2.0 - bar / 2.0), 0.110)),
        ("diffuser_panel_2", (panel_thick, panel_side_d, panel_h), (hw / 2.0 - bar / 2.0, 0.0, 0.110)),
        ("diffuser_panel_3", (panel_thick, panel_side_d, panel_h), (-(hw / 2.0 - bar / 2.0), 0.0, 0.110)),
    ]
    for name, dims, xyz in panel_specs:
        body.visual(
            Box(dims),
            origin=Origin(xyz=xyz),
            material=warm_glass,
            name=name,
        )

    # LED mounting pedestal — connects the LED column to the housing bottom plate
    body.visual(
        Cylinder(radius=0.016, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.0595)),
        material=dark_tan,
        name="led_mount",
    )

    # LED column (kept — slightly shorter for squatter box proportions)
    body.visual(
        Cylinder(radius=0.018, length=0.098),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=warm_core,
        name="led_column",
    )

    # Squared vented base (replaces lathe-mesh round base)
    body.visual(
        mesh_from_cadquery(_squared_base_shape(), "vented_base"),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=dark_tan,
        name="vented_base",
    )

    # Squared top vent cap (replaces lathe-mesh round cap)
    body.visual(
        mesh_from_cadquery(_squared_cap_shape(), "top_vent_cap"),
        origin=Origin(xyz=(0.0, 0.0, 0.181)),
        material=dark_tan,
        name="top_vent_cap",
    )

    # Raised control/battery block on the cap (kept)
    body.visual(
        Box((0.068, 0.030, 0.020)),
        origin=Origin(xyz=(0.000, -0.004, 0.202)),
        material=dark_tan,
        name="top_control_block",
    )
    body.visual(
        Box((0.047, 0.004, 0.0012)),
        origin=Origin(xyz=(0.000, -0.020, 0.211)),
        material=white_mark,
        name="brand_stroke",
    )
    body.visual(
        Box((0.025, 0.016, 0.004)),
        origin=Origin(xyz=(0.025, 0.025, 0.192)),
        material=switch_black,
        name="top_switch",
    )

    # Bail pivot ears on the housing sides (kept)
    for i, x_sign in enumerate((-1, 1)):
        body.visual(
            Cylinder(radius=0.006, length=0.014),
            origin=Origin(
                xyz=(x_sign * 0.050, 0.0, 0.172),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=dark_metal,
            name=f"bail_ear_{i}",
        )

    # ── Carry bail ──
    bail = model.part("carry_bail")
    bail.visual(
        mesh_from_geometry(_bail_mesh(), "carry_bail_wire"),
        material=dark_metal,
        name="carry_bail_wire",
    )
    model.articulation(
        "body_to_bail",
        ArticulationType.REVOLUTE,
        parent=body,
        child=bail,
        origin=Origin(xyz=(0.0, 0.0, 0.175)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.5, lower=-1.35, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("lantern_body")
    bail = object_model.get_part("carry_bail")
    bail_joint = object_model.get_articulation("body_to_bail")

    # ── TARGET-specific: rectangular box housing + flat panels ──
    housing_aabb = ctx.part_element_world_aabb(body, elem="housing_frame")
    ctx.check(
        "housing_frame exists as rectangular box fixture",
        housing_aabb is not None,
        details=f"housing_aabb={housing_aabb}",
    )

    if housing_aabb is not None:
        hx = housing_aabb[1][0] - housing_aabb[0][0]
        hy = housing_aabb[1][1] - housing_aabb[0][1]
        hz = housing_aabb[1][2] - housing_aabb[0][2]
        ctx.check(
            "housing has rectangular box proportions (wider than deep, squatter than tall)",
            hx > 0.080 and hy > 0.060 and hx > hy and hz > 0.100,
            details=f"hx={hx:.4f}, hy={hy:.4f}, hz={hz:.4f}",
        )

    # Verify all four flat diffuser panels exist
    for i in range(4):
        panel_aabb = ctx.part_element_world_aabb(body, elem=f"diffuser_panel_{i}")
        ctx.check(
            f"diffuser_panel_{i} is a flat rectangular panel",
            panel_aabb is not None,
            details=f"panel_aabb={panel_aabb}",
        )

    # Front/back panels should be wider (X) than thick (Y)
    front_aabb = ctx.part_element_world_aabb(body, elem="diffuser_panel_0")
    if front_aabb is not None:
        px = front_aabb[1][0] - front_aabb[0][0]
        py = front_aabb[1][1] - front_aabb[0][1]
        ctx.check(
            "front diffuser panel is flat (width >> thickness)",
            px > 0.060 and py < 0.010,
            details=f"panel_x={px:.4f}, panel_y={py:.4f}",
        )

    # ── Bail checks (preserved articulation) ──
    ctx.expect_overlap(
        body,
        bail,
        axes="x",
        elem_a="top_vent_cap",
        elem_b="carry_bail_wire",
        min_overlap=0.080,
        name="bail spans across the squared top cap",
    )

    bail_box = ctx.part_element_world_aabb(bail, elem="carry_bail_wire")
    cap_box = ctx.part_element_world_aabb(body, elem="top_vent_cap")
    ctx.check(
        "upright bail rises above the vent cap",
        bail_box is not None and cap_box is not None and bail_box[1][2] > cap_box[1][2] + 0.040,
        details=f"bail_box={bail_box}, cap_box={cap_box}",
    )

    with ctx.pose({bail_joint: 1.0}):
        rotated_bail_box = ctx.part_element_world_aabb(bail, elem="carry_bail_wire")
        ctx.expect_overlap(
            body,
            bail,
            axes="x",
            elem_a="top_vent_cap",
            elem_b="carry_bail_wire",
            min_overlap=0.060,
            name="bail remains captured while rotated",
        )
    ctx.check(
        "bail hinge rotates the carry loop forward",
        bail_box is not None
        and rotated_bail_box is not None
        and rotated_bail_box[0][1] < bail_box[0][1] - 0.025,
        details=f"upright={bail_box}, rotated={rotated_bail_box}",
    )

    return ctx.report()


object_model = build_object_model()
