from __future__ import annotations

"""Corded electric chainsaw with a sage-green polymer motor housing.

Canonical frame: +X points forward along the guide bar, +Z is up, +Y is the
left (motor-cap) side. Real-world scale: about 0.84 m overall length.

Articulated mechanisms:
- trigger_pivot: squeeze trigger inside the rear loop handle (revolute).
- hand_guard_pivot: chain-brake hand guard that pivots forward (revolute).
- lock_button_slide: side lock-off button on the rear handle (prismatic).
"""

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
    rounded_rect_profile,
    tube_from_spline_points,
)

HALF_PI = math.pi / 2.0


def _side_extrusion(profile, width: float, name: str):
    """Extrude an XZ side-silhouette profile along the Y (width) axis."""
    geom = ExtrudeGeometry(profile, width, cap=True, center=True)
    geom.rotate_x(HALF_PI)
    return mesh_from_geometry(geom, name)


def _side_loop(outer, hole, width: float, name: str):
    """Extrude an XZ loop profile with a through opening along Y."""
    geom = ExtrudeWithHolesGeometry(outer, [hole], width, cap=True, center=True)
    geom.rotate_x(HALF_PI)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="corded_electric_chainsaw")

    sage_green = model.material("sage_green_polymer", rgba=(0.55, 0.60, 0.50, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.09, 0.09, 0.10, 1.0))
    charcoal = model.material("charcoal_plastic", rgba=(0.17, 0.18, 0.19, 1.0))
    chrome = model.material("chrome_steel", rgba=(0.82, 0.83, 0.85, 1.0))
    chain_steel = model.material("chain_steel", rgba=(0.30, 0.31, 0.33, 1.0))
    white_badge = model.material("white_badge", rgba=(0.93, 0.93, 0.92, 1.0))

    # ------------------------------------------------------------------ body
    body = model.part("body")

    # Main motor housing block (rounded side silhouette, extruded across Y).
    body.visual(
        _side_extrusion(rounded_rect_profile(0.26, 0.17, 0.04), 0.10, "housing_block"),
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
        material=sage_green,
        name="housing_shell",
    )

    # Cross-mounted motor end bulge on the left side, with a black end cap.
    body.visual(
        Cylinder(radius=0.052, length=0.045),
        origin=Origin(xyz=(-0.01, 0.062, 0.14), rpy=(HALF_PI, 0.0, 0.0)),
        material=sage_green,
        name="motor_bulge",
    )
    body.visual(
        Cylinder(radius=0.040, length=0.012),
        origin=Origin(xyz=(-0.01, 0.088, 0.14), rpy=(HALF_PI, 0.0, 0.0)),
        material=black_plastic,
        name="motor_end_cap",
    )

    # Sprocket-side cover plate on the right side that clamps the bar tail.
    body.visual(
        _side_extrusion(rounded_rect_profile(0.16, 0.13, 0.03), 0.022, "sprocket_cover"),
        origin=Origin(xyz=(0.10, -0.058, 0.13)),
        material=charcoal,
        name="sprocket_cover",
    )
    # Bar-nut bosses on the cover face.
    for i, bx in enumerate((0.065, 0.125)):
        body.visual(
            Cylinder(radius=0.010, length=0.008),
            origin=Origin(xyz=(bx, -0.071, 0.105), rpy=(HALF_PI, 0.0, 0.0)),
            material=black_plastic,
            name=f"bar_nut_{i}",
        )
    # White brand plate on the cover.
    body.visual(
        Box((0.10, 0.005, 0.030)),
        origin=Origin(xyz=(0.10, -0.070, 0.155)),
        material=white_badge,
        name="brand_plate",
    )

    # Rear loop handle (green, through opening for the trigger hand).
    body.visual(
        _side_loop(
            [(x - 0.21, z + 0.115) for x, z in rounded_rect_profile(0.22, 0.17, 0.035)],
            [(x - 0.21, z + 0.115) for x, z in rounded_rect_profile(0.12, 0.09, 0.022)],
            0.036,
            "rear_handle",
        ),
        material=sage_green,
        name="rear_handle",
    )

    # Black top carry handle arching over the housing.
    body.visual(
        _side_loop(
            [(x, z + 0.25) for x, z in rounded_rect_profile(0.20, 0.12, 0.030)],
            [(x, z + 0.245) for x, z in rounded_rect_profile(0.13, 0.05, 0.020)],
            0.030,
            "top_handle",
        ),
        material=black_plastic,
        name="top_handle",
    )

    # Guard mount pedestal on the housing top front.
    body.visual(
        Box((0.050, 0.110, 0.030)),
        origin=Origin(xyz=(0.115, 0.0, 0.215)),
        material=sage_green,
        name="guard_pedestal",
    )
    # Hinge ears on the pedestal that carry the hand-guard shaft.
    body.visual(
        Box((0.024, 0.014, 0.030)),
        origin=Origin(xyz=(0.125, 0.045, 0.2425)),
        material=black_plastic,
        name="guard_ear_left",
    )
    body.visual(
        Box((0.024, 0.014, 0.030)),
        origin=Origin(xyz=(0.125, -0.045, 0.2425)),
        material=black_plastic,
        name="guard_ear_right",
    )

    # Power cord exiting the rear of the handle.
    cord_geom = tube_from_spline_points(
        [
            (-0.295, 0.0, 0.055),
            (-0.345, 0.0, 0.045),
            (-0.405, 0.0, 0.028),
            (-0.465, 0.0, 0.018),
        ],
        radius=0.006,
        samples_per_segment=10,
    )
    body.visual(
        mesh_from_geometry(cord_geom, "power_cord"),
        material=black_plastic,
        name="power_cord",
    )

    # ------------------------------------------------------------- guide bar
    guide_bar = model.part("guide_bar")

    # Chrome bar plate: stadium silhouette, thin across Y.
    guide_bar.visual(
        _side_extrusion(rounded_rect_profile(0.40, 0.07, 0.035), 0.008, "bar_plate"),
        material=chrome,
        name="bar_plate",
    )
    # Cutting chain: closed loop riding in the bar edge groove.
    chain_outer = rounded_rect_profile(0.424, 0.094, 0.047)
    chain_hole = rounded_rect_profile(0.394, 0.064, 0.032)
    guide_bar.visual(
        _side_loop(chain_outer, chain_hole, 0.011, "chain_loop"),
        material=chain_steel,
        name="chain_loop",
    )

    model.articulation(
        "body_to_guide_bar",
        ArticulationType.FIXED,
        parent=body,
        child=guide_bar,
        # Bar tail sits under the sprocket cover; bar extends forward past x=0.5.
        origin=Origin(xyz=(0.30, -0.058, 0.135)),
    )

    # ------------------------------------------------------------ hand guard
    hand_guard = model.part("hand_guard")

    # Pivot shaft captured between the housing hinge ears.
    hand_guard.visual(
        Cylinder(radius=0.009, length=0.096),
        origin=Origin(rpy=(HALF_PI, 0.0, 0.0)),
        material=black_plastic,
        name="guard_shaft",
    )
    # Narrow riser between the ears that carries the paddle off the shaft.
    hand_guard.visual(
        Box((0.012, 0.070, 0.045)),
        origin=Origin(
            xyz=(0.0225 * math.sin(0.35), 0.0, 0.0225 * math.cos(0.35)),
            rpy=(0.0, 0.35, 0.0),
        ),
        material=black_plastic,
        name="guard_riser",
    )
    # Broad brake paddle leaning forward above the shaft.
    hand_guard.visual(
        Box((0.012, 0.130, 0.150)),
        origin=Origin(
            xyz=(0.092 * math.sin(0.35), 0.0, 0.092 * math.cos(0.35)),
            rpy=(0.0, 0.35, 0.0),
        ),
        material=black_plastic,
        name="guard_paddle",
    )
    # Raised horizontal ribs on the front face that read as the vented grid.
    rib_proud = 0.008  # half paddle thickness + half rib thickness
    for i, t in enumerate((0.050, 0.092, 0.134)):
        hand_guard.visual(
            Box((0.006, 0.118, 0.012)),
            origin=Origin(
                xyz=(
                    rib_proud * math.cos(0.35) + t * math.sin(0.35),
                    0.0,
                    -rib_proud * math.sin(0.35) + t * math.cos(0.35),
                ),
                rpy=(0.0, 0.35, 0.0),
            ),
            material=charcoal,
            name=f"guard_rib_{i}",
        )

    model.articulation(
        "hand_guard_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=hand_guard,
        origin=Origin(xyz=(0.125, 0.0, 0.245)),
        # Positive q tips the paddle top toward +X (forward = brake engage).
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=4.0, lower=-0.15, upper=0.55),
    )

    # --------------------------------------------------------------- trigger
    trigger = model.part("trigger")

    # Pivot boss seated into the front strut of the rear handle loop.
    trigger.visual(
        Box((0.020, 0.020, 0.016)),
        origin=Origin(xyz=(0.003, 0.0, 0.0)),
        material=black_plastic,
        name="trigger_boss",
    )
    # Finger blade hanging down inside the handle opening.
    trigger.visual(
        Box((0.016, 0.024, 0.066)),
        origin=Origin(xyz=(-0.012, 0.0, -0.038)),
        material=black_plastic,
        name="trigger_blade",
    )
    trigger.visual(
        Box((0.022, 0.024, 0.012)),
        origin=Origin(xyz=(-0.018, 0.0, -0.067)),
        material=black_plastic,
        name="trigger_toe",
    )

    model.articulation(
        "trigger_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=trigger,
        # Top-front corner of the handle opening.
        origin=Origin(xyz=(-0.153, 0.0, 0.150)),
        # Positive q swings the blade rearward (squeeze).
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=6.0, lower=0.0, upper=0.40),
    )

    # ------------------------------------------------------- lock-off button
    lock_button = model.part("lock_button")
    lock_button.visual(
        Cylinder(radius=0.008, length=0.014),
        origin=Origin(rpy=(HALF_PI, 0.0, 0.0)),
        material=charcoal,
        name="lock_button_cap",
    )

    model.articulation(
        "lock_button_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lock_button,
        # Left face of the rear handle top strut; stem embeds 2 mm at rest.
        origin=Origin(xyz=(-0.21, 0.023, 0.18)),
        # Positive q presses the button inward (toward -Y).
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.1, lower=0.0, upper=0.005),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    guide_bar = object_model.get_part("guide_bar")
    hand_guard = object_model.get_part("hand_guard")
    trigger = object_model.get_part("trigger")
    lock_button = object_model.get_part("lock_button")

    guard_joint = object_model.get_articulation("hand_guard_pivot")
    trigger_joint = object_model.get_articulation("trigger_pivot")
    lock_joint = object_model.get_articulation("lock_button_slide")

    # ---------------------------------------------------- intentional fits
    ctx.allow_overlap(
        body,
        guide_bar,
        elem_a="sprocket_cover",
        elem_b="bar_plate",
        reason="Bar tail is intentionally clamped under the sprocket cover.",
    )
    ctx.allow_overlap(
        body,
        guide_bar,
        elem_a="sprocket_cover",
        elem_b="chain_loop",
        reason="Chain loop tail wraps the drive sprocket hidden inside the cover.",
    )
    ctx.allow_overlap(
        body,
        hand_guard,
        elem_a="guard_ear_left",
        elem_b="guard_shaft",
        reason="Hinge shaft is captured inside the left housing ear.",
    )
    ctx.allow_overlap(
        body,
        hand_guard,
        elem_a="guard_ear_right",
        elem_b="guard_shaft",
        reason="Hinge shaft is captured inside the right housing ear.",
    )
    ctx.allow_overlap(
        body,
        trigger,
        elem_a="rear_handle",
        elem_b="trigger_boss",
        reason="Trigger pivot boss is seated into the handle front strut.",
    )
    ctx.allow_overlap(
        body,
        lock_button,
        elem_a="rear_handle",
        elem_b="lock_button_cap",
        reason="Lock-off button stem embeds slightly into the handle side wall.",
    )

    # --------------------------------------------------- prompt-level shape
    bar_aabb = ctx.part_world_aabb(guide_bar)
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "guide bar extends well forward of the housing",
        bar_aabb is not None and body_aabb is not None and bar_aabb[1][0] > 0.45,
        details=f"bar_aabb={bar_aabb}",
    )
    ctx.expect_within(
        guide_bar,
        guide_bar,
        axes="xz",
        inner_elem="bar_plate",
        outer_elem="chain_loop",
        margin=0.001,
        name="chain loop wraps the guide bar silhouette",
    )
    ctx.expect_overlap(
        body,
        guide_bar,
        axes="x",
        elem_a="sprocket_cover",
        elem_b="bar_plate",
        min_overlap=0.04,
        name="bar tail stays clamped under the sprocket cover",
    )
    ctx.expect_within(
        trigger,
        body,
        axes="xz",
        inner_elem="trigger_blade",
        outer_elem="rear_handle",
        margin=0.0,
        name="trigger blade sits inside the rear handle loop",
    )
    ctx.expect_contact(
        body,
        hand_guard,
        elem_a="guard_ear_left",
        elem_b="guard_shaft",
        contact_tol=1e-4,
        name="guard shaft engages the hinge ears",
    )
    ctx.check(
        "hand guard rises above the housing front",
        (lambda g: g is not None and g[1][2] > 0.30)(ctx.part_world_aabb(hand_guard)),
        details=f"guard_aabb={ctx.part_world_aabb(hand_guard)}",
    )

    # ------------------------------------------------------ decisive poses
    guard_rest = ctx.part_world_aabb(hand_guard)
    with ctx.pose({guard_joint: 0.55}):
        guard_fwd = ctx.part_world_aabb(hand_guard)
    ctx.check(
        "hand guard pivots forward toward the bar",
        guard_rest is not None
        and guard_fwd is not None
        and guard_fwd[1][0] > guard_rest[1][0] + 0.03,
        details=f"rest={guard_rest}, engaged={guard_fwd}",
    )

    trig_rest = ctx.part_world_aabb(trigger)
    with ctx.pose({trigger_joint: 0.40}):
        trig_squeezed = ctx.part_world_aabb(trigger)
        ctx.expect_within(
            trigger,
            body,
            axes="xz",
            inner_elem="trigger_blade",
            outer_elem="rear_handle",
            margin=0.0,
            name="squeezed trigger stays inside the handle loop",
        )
    ctx.check(
        "squeezing swings the trigger rearward",
        trig_rest is not None
        and trig_squeezed is not None
        and trig_squeezed[0][0] < trig_rest[0][0] - 0.01,
        details=f"rest={trig_rest}, squeezed={trig_squeezed}",
    )

    btn_rest = ctx.part_world_position(lock_button)
    with ctx.pose({lock_joint: 0.005}):
        btn_pressed = ctx.part_world_position(lock_button)
    ctx.check(
        "lock-off button presses inward",
        btn_rest is not None
        and btn_pressed is not None
        and btn_pressed[1] < btn_rest[1] - 0.003,
        details=f"rest={btn_rest}, pressed={btn_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
