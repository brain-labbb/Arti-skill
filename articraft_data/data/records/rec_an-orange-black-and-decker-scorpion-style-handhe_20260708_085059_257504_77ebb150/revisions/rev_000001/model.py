from __future__ import annotations

"""Orange Scorpion-style handheld electric powered saw.

Canonical frame: +X is forward (toward the blade), +Z is up, the tool is
bilaterally symmetric about the XZ plane. The motor housing carries a closed
D-loop top handle, a squeezable trigger inside the loop, a side lock-off
button, a downward-angled gearbox nose with a gray blade clamp bezel and a
gray guard shoe, and a silver toothed saw blade that reciprocates along the
nose axis.
"""

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)

# Nose / blade axis pitch below horizontal (radians).
NOSE_PITCH = 0.4363  # ~25 degrees
COS_P = math.cos(NOSE_PITCH)
SIN_P = math.sin(NOSE_PITCH)

# Nose front-face center = blade joint frame origin (world).
NOSE_FRONT = (0.168, 0.0, 0.0909)


def _yz_section(width_y: float, height_z: float, radius: float, x: float, zc: float):
    """Rounded-rect loft section in the YZ plane at station x, centered at z=zc."""
    return [(x, y, z + zc) for z, y in rounded_rect_profile(height_z, width_y, radius)]


def _blade_profile() -> list[tuple[float, float]]:
    """Side profile of the saw blade in local (length, height) coordinates.

    Local +x runs along the blade toward the tip; +y is the blade's back-edge
    direction. The shank (x < 0) stays clamped inside the gearbox nose. The
    cutting edge carries a serrated tooth row; the tip is angled like a
    real Scorpion wood blade. Counter-clockwise polygon.
    """
    pts: list[tuple[float, float]] = []
    # Bottom of the clamped shank, rear to blade root.
    pts.append((-0.045, -0.010))
    pts.append((0.005, -0.010))
    # Serrated cutting edge: 20 teeth, 8 mm pitch, 5 mm deep.
    x = 0.005
    tooth_pitch = 0.008
    while x + tooth_pitch <= 0.165 + 1e-9:
        pts.append((x + 0.5 * tooth_pitch, -0.015))
        pts.append((x + tooth_pitch, -0.010))
        x += tooth_pitch
    # Angled tip.
    pts.append((0.168, -0.008))
    pts.append((0.175, -0.002))
    pts.append((0.165, 0.008))
    # Tapered back edge returning to the shank.
    pts.append((0.000, 0.020))
    pts.append((-0.045, 0.014))
    return pts


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="scorpion_electric_saw")

    orange = model.material("housing_orange", rgba=(0.90, 0.30, 0.06, 1.0))
    black = model.material("plastic_black", rgba=(0.10, 0.10, 0.10, 1.0))
    gray = model.material("metal_gray", rgba=(0.48, 0.49, 0.51, 1.0))
    steel = model.material("blade_steel", rgba=(0.78, 0.79, 0.81, 1.0))

    # ------------------------------------------------------------------ body
    body = model.part("body")

    # Main motor housing: lofted orange shell tapering toward the nose.
    housing_geom = section_loft(
        [
            _yz_section(0.058, 0.082, 0.020, -0.115, 0.140),
            _yz_section(0.074, 0.106, 0.026, -0.040, 0.140),
            _yz_section(0.066, 0.094, 0.024, 0.045, 0.135),
            _yz_section(0.052, 0.070, 0.018, 0.095, 0.125),
        ]
    )
    body.visual(
        mesh_from_geometry(housing_geom, "housing_shell"),
        material=orange,
        name="housing_shell",
    )

    # Gearbox nose, pitched downward-forward; the blade exits its front face.
    body.visual(
        Box((0.095, 0.048, 0.058)),
        origin=Origin(xyz=(0.125, 0.0, 0.111), rpy=(0.0, NOSE_PITCH, 0.0)),
        material=orange,
        name="nose_housing",
    )
    # Gray blade-clamp bezel on the nose front face.
    body.visual(
        Box((0.010, 0.046, 0.056)),
        origin=Origin(
            xyz=(
                NOSE_FRONT[0] - 0.004 * COS_P,
                0.0,
                NOSE_FRONT[2] + 0.004 * SIN_P,
            ),
            rpy=(0.0, NOSE_PITCH, 0.0),
        ),
        material=gray,
        name="nose_cap",
    )

    # Gray guard shoe under the nose: cross bar plus two plates flanking the blade.
    body.visual(
        Box((0.050, 0.044, 0.012)),
        origin=Origin(xyz=(0.127, 0.0, 0.075), rpy=(0.0, NOSE_PITCH, 0.0)),
        material=gray,
        name="guard_bar",
    )
    for i, sy in enumerate((1.0, -1.0)):
        body.visual(
            Box((0.052, 0.006, 0.040)),
            origin=Origin(xyz=(0.135, sy * 0.0125, 0.062), rpy=(0.0, NOSE_PITCH, 0.0)),
            material=gray,
            name=f"guard_plate_{i}",
        )

    # Closed D-loop top handle: front post, top grip, rear post.
    body.visual(
        Box((0.034, 0.036, 0.080)),
        origin=Origin(xyz=(0.020, 0.0, 0.220)),
        material=orange,
        name="handle_front_post",
    )
    body.visual(
        Box((0.150, 0.036, 0.036)),
        origin=Origin(xyz=(-0.043, 0.0, 0.268)),
        material=orange,
        name="handle_grip",
    )
    body.visual(
        Box((0.034, 0.036, 0.085)),
        origin=Origin(xyz=(-0.100, 0.0, 0.217)),
        material=orange,
        name="handle_rear_post",
    )

    # Black brand label panels on both housing flanks (symmetric, numbered).
    for i, sy in enumerate((1.0, -1.0)):
        body.visual(
            Box((0.055, 0.004, 0.045)),
            origin=Origin(xyz=(-0.020, sy * 0.0355, 0.135)),
            material=black,
            name=f"label_panel_{i}",
        )

    # Motor vent slats near the nose on both flanks.
    for i, sy in enumerate((1.0, -1.0)):
        for j, vx in enumerate((0.022, 0.030, 0.038)):
            body.visual(
                Box((0.004, 0.006, 0.024)),
                origin=Origin(xyz=(vx, sy * 0.0335, 0.145)),
                material=black,
                name=f"vent_slat_{i}_{j}",
            )

    # Power cord: strain-relief boot leaving the rear of the grip, then cord.
    boot_dir = (-0.5, 0.0, 0.8660)
    boot_start = (-0.116, 0.0, 0.277)
    body.visual(
        Cylinder(radius=0.007, length=0.035),
        origin=Origin(
            xyz=(
                boot_start[0] + 0.0175 * boot_dir[0],
                0.0,
                boot_start[2] + 0.0175 * boot_dir[2],
            ),
            rpy=(0.0, -0.5236, 0.0),
        ),
        material=black,
        name="cord_boot",
    )
    cord_geom = tube_from_spline_points(
        [
            (-0.132, 0.0, 0.303),
            (-0.140, 0.0, 0.327),
            (-0.136, 0.0, 0.347),
            (-0.126, 0.0, 0.360),
        ],
        radius=0.004,
    )
    body.visual(
        mesh_from_geometry(cord_geom, "power_cord"),
        material=black,
        name="power_cord",
    )

    # --------------------------------------------------------------- trigger
    # Pivot under the grip at the front of the finger opening; squeezing
    # (positive q about +Y) swings the paddle rearward toward the palm.
    trigger = model.part("trigger")
    trigger.visual(
        Cylinder(radius=0.008, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="trigger_boss",
    )
    trigger.visual(
        Box((0.014, 0.026, 0.048)),
        origin=Origin(xyz=(-0.004, 0.0, -0.026)),
        material=black,
        name="trigger_paddle",
    )
    model.articulation(
        "trigger_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=trigger,
        origin=Origin(xyz=(-0.005, 0.0, 0.245)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0, lower=0.0, upper=0.30),
    )

    # -------------------------------------------------------- lock-off button
    # Side safety button on the handle front post; positive q presses it
    # inward (toward -Y) into the handle.
    button = model.part("lock_off_button")
    button.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.0, 0.001, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="button_stem",
    )
    button.visual(
        Cylinder(radius=0.0095, length=0.006),
        origin=Origin(xyz=(0.0, 0.009, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="button_cap",
    )
    model.articulation(
        "lock_off_press",
        ArticulationType.PRISMATIC,
        parent=body,
        child=button,
        origin=Origin(xyz=(0.020, 0.018, 0.235)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.1, lower=0.0, upper=0.004),
    )

    # ----------------------------------------------------------------- blade
    # Joint frame sits on the nose front face, pitched with the nose so the
    # prismatic axis is the reciprocating stroke direction. The shank and
    # clamp block stay engaged inside the nose across the full stroke.
    blade = model.part("blade")
    blade_geom = ExtrudeGeometry(_blade_profile(), 0.0022)
    blade.visual(
        mesh_from_geometry(blade_geom, "blade_plate"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="blade_plate",
    )
    blade.visual(
        Box((0.030, 0.014, 0.030)),
        origin=Origin(xyz=(-0.030, 0.0, 0.002)),
        material=gray,
        name="blade_clamp",
    )
    model.articulation(
        "blade_stroke",
        ArticulationType.PRISMATIC,
        parent=body,
        child=blade,
        origin=Origin(xyz=NOSE_FRONT, rpy=(0.0, NOSE_PITCH, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=3.0, lower=-0.009, upper=0.009),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    trigger = object_model.get_part("trigger")
    blade = object_model.get_part("blade")
    button = object_model.get_part("lock_off_button")
    trigger_pivot = object_model.get_articulation("trigger_pivot")
    blade_stroke = object_model.get_articulation("blade_stroke")
    lock_off_press = object_model.get_articulation("lock_off_press")

    # Intentional local embeddings that make the mechanisms read as mounted.
    ctx.allow_overlap(
        body,
        trigger,
        elem_a="handle_grip",
        elem_b="trigger_boss",
        reason="Trigger pivot boss is seated in the underside of the grip bar.",
    )
    ctx.allow_overlap(
        body,
        button,
        elem_a="handle_front_post",
        elem_b="button_stem",
        reason="Lock-off button stem rides in its bore in the handle post.",
    )
    for body_elem in ("nose_housing", "nose_cap"):
        for blade_elem in ("blade_plate", "blade_clamp"):
            ctx.allow_overlap(
                body,
                blade,
                elem_a=body_elem,
                elem_b=blade_elem,
                reason="Blade shank and clamp block reciprocate inside the gearbox nose.",
            )

    # Hero silhouette: blade reaches forward and angles below the housing.
    blade_aabb = ctx.part_world_aabb(blade)
    ctx.check(
        "blade reaches forward of the nose",
        blade_aabb is not None and blade_aabb[1][0] > 0.30,
        details=f"blade_aabb={blade_aabb}",
    )
    ctx.check(
        "blade angles downward below the housing",
        blade_aabb is not None and blade_aabb[0][2] < 0.05,
        details=f"blade_aabb={blade_aabb}",
    )

    # Blade shank retains insertion in the nose at rest and at full stroke.
    ctx.expect_overlap(
        blade,
        body,
        axes="x",
        elem_a="blade_plate",
        elem_b="nose_housing",
        min_overlap=0.025,
        name="blade shank stays clamped in the nose at rest",
    )
    rest_blade = ctx.part_world_position(blade)
    with ctx.pose({blade_stroke: 0.009}):
        ctx.expect_overlap(
            blade,
            body,
            axes="x",
            elem_a="blade_plate",
            elem_b="nose_housing",
            min_overlap=0.015,
            name="blade shank stays clamped at full extension",
        )
        ext_blade = ctx.part_world_position(blade)
    ctx.check(
        "positive stroke drives the blade forward-downward along the nose axis",
        rest_blade is not None
        and ext_blade is not None
        and ext_blade[0] > rest_blade[0] + 0.005
        and ext_blade[2] < rest_blade[2] - 0.002,
        details=f"rest={rest_blade}, extended={ext_blade}",
    )

    # Guard shoe flanks the blade with clearance on both sides.
    ctx.expect_within(
        blade,
        body,
        axes="y",
        inner_elem="blade_plate",
        outer_elem="guard_bar",
        margin=0.0,
        name="blade runs centered through the guard shoe",
    )
    ctx.expect_gap(
        body,
        blade,
        axis="y",
        positive_elem="guard_plate_0",
        negative_elem="blade_plate",
        min_gap=0.004,
        name="guard plate clears the blade flank",
    )

    # Trigger hangs inside the handle loop and squeezes rearward.
    ctx.expect_within(
        trigger,
        body,
        axes="x",
        inner_elem="trigger_paddle",
        outer_elem="handle_grip",
        margin=0.0,
        name="trigger paddle sits inside the handle loop",
    )
    ctx.expect_contact(
        body,
        trigger,
        elem_a="handle_grip",
        elem_b="trigger_boss",
        name="trigger boss is seated against the grip",
    )
    rest_paddle = ctx.part_element_world_aabb(trigger, elem="trigger_paddle")
    with ctx.pose({trigger_pivot: 0.30}):
        squeezed_paddle = ctx.part_element_world_aabb(trigger, elem="trigger_paddle")
    ctx.check(
        "squeezing the trigger swings the paddle rearward",
        rest_paddle is not None
        and squeezed_paddle is not None
        and squeezed_paddle[0][0] < rest_paddle[0][0] - 0.005,
        details=f"rest={rest_paddle}, squeezed={squeezed_paddle}",
    )

    # Lock-off button protrudes from the post and presses inward.
    ctx.expect_contact(
        body,
        button,
        elem_a="handle_front_post",
        elem_b="button_stem",
        name="lock-off button stem engages the handle post",
    )
    rest_button = ctx.part_world_position(button)
    with ctx.pose({lock_off_press: 0.004}):
        pressed_button = ctx.part_world_position(button)
    ctx.check(
        "pressing the lock-off button moves it into the handle",
        rest_button is not None
        and pressed_button is not None
        and pressed_button[1] < rest_button[1] - 0.003,
        details=f"rest={rest_button}, pressed={pressed_button}",
    )

    # Power cord rises from the rear of the grip.
    cord_aabb = ctx.part_element_world_aabb(body, elem="power_cord")
    ctx.check(
        "power cord exits above and behind the grip",
        cord_aabb is not None and cord_aabb[1][2] > 0.33 and cord_aabb[1][0] < -0.10,
        details=f"cord_aabb={cord_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
