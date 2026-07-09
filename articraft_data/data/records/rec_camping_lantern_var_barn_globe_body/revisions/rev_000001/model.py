from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


TAU = 2.0 * math.pi


def _radial_xy(radius: float, angle: float) -> tuple[float, float]:
    return (radius * math.cos(angle), radius * math.sin(angle))


def _barn_globe_mesh() -> LatheGeometry:
    """Bulged barn/hurricane glass globe profile: narrow necks at top and bottom,
    wide domed belly in the middle."""
    return LatheGeometry(
        [
            (0.000, 0.052),
            (0.028, 0.052),
            (0.034, 0.060),
            (0.044, 0.074),
            (0.054, 0.092),
            (0.061, 0.108),
            (0.064, 0.118),
            (0.063, 0.132),
            (0.056, 0.148),
            (0.046, 0.162),
            (0.036, 0.172),
            (0.028, 0.176),
            (0.000, 0.176),
        ],
        segments=72,
    )


def _vented_base_mesh() -> LatheGeometry:
    return LatheGeometry(
        [
            (0.000, 0.020),
            (0.044, 0.020),
            (0.052, 0.026),
            (0.050, 0.036),
            (0.037, 0.044),
            (0.032, 0.049),
            (0.021, 0.052),
            (0.000, 0.052),
        ],
        segments=72,
    )


def _top_cap_mesh() -> LatheGeometry:
    return LatheGeometry(
        [
            (0.000, 0.176),
            (0.053, 0.176),
            (0.063, 0.181),
            (0.062, 0.190),
            (0.053, 0.195),
            (0.026, 0.195),
            (0.018, 0.198),
            (0.018, 0.203),
            (0.013, 0.207),
            (0.000, 0.207),
        ],
        segments=72,
    )


def _bail_mesh() -> LatheGeometry:
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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="barn_globe_camping_lantern",
        meta={
            "run_notes": (
                "Barn/hurricane globe camping lantern fork: replaced the straight cylindrical "
                "diffuser and wire cage with a LatheGeometry bulged glass globe profile. "
                "Vintage brass/copper frame finish applied. Tripod legs removed per fork spec."
            )
        },
    )

    # ⑥ vintage brass/copper finish instead of olive/blackened steel.
    antique_brass = model.material("antique_brass", rgba=(0.62, 0.45, 0.14, 1.0))
    burnished_copper = model.material("burnished_copper", rgba=(0.55, 0.36, 0.16, 1.0))
    warm_glass = model.material("warm_translucent_glass", rgba=(1.0, 0.54, 0.08, 0.34))
    warm_core = model.material("warm_led_core", rgba=(1.0, 0.73, 0.20, 0.88))
    switch_black = model.material("matte_black_switch", rgba=(0.018, 0.018, 0.016, 1.0))
    white_mark = model.material("white_brand_mark", rgba=(0.93, 0.93, 0.88, 1.0))

    body = model.part("lantern_body")

    # Barn/hurricane globe: bulged domed glass envelope replacing the cylindrical cage.
    body.visual(
        mesh_from_geometry(_barn_globe_mesh(), "barn_globe"),
        material=warm_glass,
        name="barn_globe",
    )

    # Warm LED column inside the globe (kept from parent).
    body.visual(
        Cylinder(radius=0.020, length=0.128),
        origin=Origin(xyz=(0.0, 0.0, 0.112)),
        material=warm_core,
        name="led_column",
    )

    # Flared vented base skirt (kept from parent, brass recolor).
    body.visual(
        mesh_from_geometry(_vented_base_mesh(), "vented_base"),
        material=antique_brass,
        name="vented_base",
    )

    # Peaked vented top cap (kept from parent, brass recolor).
    body.visual(
        mesh_from_geometry(_top_cap_mesh(), "top_vent_cap"),
        material=antique_brass,
        name="top_vent_cap",
    )

    # Control/battery block and switch on the cap.
    body.visual(
        Box((0.068, 0.030, 0.020)),
        origin=Origin(xyz=(0.000, -0.004, 0.206)),
        material=burnished_copper,
        name="top_control_block",
    )
    body.visual(
        Box((0.047, 0.004, 0.0012)),
        origin=Origin(xyz=(0.000, -0.020, 0.2155)),
        material=white_mark,
        name="brand_stroke",
    )
    body.visual(
        Box((0.025, 0.016, 0.004)),
        origin=Origin(xyz=(0.025, 0.025, 0.196)),
        material=switch_black,
        name="top_switch",
    )

    # Bail pivot ears on the cap (kept from parent).
    for i, x_sign in enumerate((-1.0, 1.0)):
        body.visual(
            Cylinder(radius=0.006, length=0.014),
            origin=Origin(
                xyz=(x_sign * 0.050, 0.0, 0.193),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=burnished_copper,
            name=f"bail_ear_{i}",
        )

    # Carry bail (hinged handle).
    bail = model.part("carry_bail")
    bail.visual(
        mesh_from_geometry(_bail_mesh(), "carry_bail_wire"),
        material=burnished_copper,
        name="carry_bail_wire",
    )
    model.articulation(
        "body_to_bail",
        ArticulationType.REVOLUTE,
        parent=body,
        child=bail,
        origin=Origin(xyz=(0.0, 0.0, 0.196)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.5, lower=-1.35, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("lantern_body")
    bail = object_model.get_part("carry_bail")
    bail_joint = object_model.get_articulation("body_to_bail")

    # --- Barn globe shape verification (TARGET axis) ---
    globe_box = ctx.part_element_world_aabb(body, elem="barn_globe")
    led_box = ctx.part_element_world_aabb(body, elem="led_column")
    ctx.check(
        "barn_globe exists with bulged profile wider than led_column",
        globe_box is not None
        and led_box is not None
        and (globe_box[1][0] - globe_box[0][0]) > (led_box[1][0] - led_box[0][0]) + 0.06,
        details=f"globe_box={globe_box}, led_box={led_box}",
    )
    # The globe belly must be substantially wider than a straight cylinder of the same neck radius.
    globe_x_span = (globe_box[1][0] - globe_box[0][0]) if globe_box else 0.0
    ctx.check(
        "barn_globe belly exceeds 0.10 m diameter proving bulged form",
        globe_x_span > 0.10,
        details=f"globe_x_span={globe_x_span:.4f}",
    )

    # --- Bail articulation tests ---
    ctx.expect_overlap(
        body,
        bail,
        axes="x",
        elem_a="top_vent_cap",
        elem_b="carry_bail_wire",
        min_overlap=0.085,
        name="bail spans across the top cap",
    )
    bail_box = ctx.part_element_world_aabb(bail, elem="carry_bail_wire")
    cap_box = ctx.part_element_world_aabb(body, elem="top_vent_cap")
    ctx.check(
        "upright bail rises above the vent cap",
        bail_box is not None and cap_box is not None and bail_box[1][2] > cap_box[1][2] + 0.045,
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
            min_overlap=0.065,
            name="bail remains captured while rotated",
        )
    ctx.check(
        "bail hinge rotates the carry loop forward",
        bail_box is not None
        and rotated_bail_box is not None
        and rotated_bail_box[0][1] < bail_box[0][1] - 0.030,
        details=f"upright={bail_box}, rotated={rotated_bail_box}",
    )

    return ctx.report()


object_model = build_object_model()
