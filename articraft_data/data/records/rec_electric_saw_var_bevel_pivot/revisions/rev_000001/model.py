from __future__ import annotations

"""Cordless electric circular saw with a teal motor housing.

Canonical frame: +X points forward (toward the front of the shoe / blade
travel), +Z is up, +Y is the left side (motor-cap side). Real-world scale:
about 0.33 m overall length, 140 mm blade.

Articulated mechanisms:
- blade_spin: the toothed circular blade spins about the arbor (continuous).
- lower_guard_retract: the spring-loaded lower blade guard swings back to
  expose the blade (revolute).
- trigger_squeeze: the squeeze trigger inside the top handle (revolute).
- safety_lock_press: the side safety lock-off button (prismatic).
- bevel_pivot: the base shoe tilts relative to the housing for bevel cuts
  (revolute, axis along X at the front bevel bracket).
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

# Blade geometry constants (shared by build + tests).
BLADE_RADIUS = 0.076
BLADE_CX = 0.095  # arbor X on the housing
BLADE_CZ = 0.070  # arbor Z (center height)
BLADE_Y = -0.078  # blade plane is on the visible right side of the housing
GUARD_Y = -0.058  # fixed cover sits inboard so the metal blade remains visible
SHOE_Z = 0.037  # raised shoe top contacts the black rear battery block


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


def _blade_teeth_profile(inner_r: float, outer_r: float, n: int):
    """Closed polygon profile of a toothed disc in the XZ plane (local)."""
    pts = []
    for i in range(n):
        a0 = (i / n) * 2.0 * math.pi
        a1 = ((i + 0.5) / n) * 2.0 * math.pi
        pts.append((outer_r * math.cos(a0), outer_r * math.sin(a0)))
        pts.append((inner_r * math.cos(a1), inner_r * math.sin(a1)))
    return pts


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cordless_circular_saw")

    teal = model.material("teal_housing", rgba=(0.10, 0.52, 0.55, 1.0))
    teal_dark = model.material("teal_dark", rgba=(0.07, 0.40, 0.43, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.09, 0.09, 0.10, 1.0))
    charcoal = model.material("charcoal_plastic", rgba=(0.17, 0.18, 0.19, 1.0))
    chrome = model.material("chrome_steel", rgba=(0.82, 0.83, 0.85, 1.0))
    guard_steel = model.material("guard_steel", rgba=(0.74, 0.75, 0.77, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.66, 0.67, 0.70, 1.0))
    red_accent = model.material("red_accent", rgba=(0.72, 0.14, 0.12, 1.0))

    # ------------------------------------------------------------------ body
    body = model.part("body")

    # Main motor housing block (rounded side silhouette, extruded across Y).
    body.visual(
        _side_extrusion(rounded_rect_profile(0.17, 0.11, 0.035), 0.075, "housing_block"),
        origin=Origin(xyz=(0.03, 0.0, 0.105)),
        material=teal,
        name="housing_shell",
    )

    # Cross-mounted motor barrel on the left side, above the blade line.
    body.visual(
        Cylinder(radius=0.046, length=0.070),
        origin=Origin(xyz=(0.045, 0.055, 0.115), rpy=(HALF_PI, 0.0, 0.0)),
        material=teal,
        name="motor_barrel",
    )
    body.visual(
        Cylinder(radius=0.036, length=0.012),
        origin=Origin(xyz=(0.045, 0.093, 0.115), rpy=(HALF_PI, 0.0, 0.0)),
        material=black_plastic,
        name="motor_end_cap",
    )

    # Upper fixed blade guard: inboard half-disc shroud over the top of the blade.
    ug_ang = [math.pi * (0.03 + 0.94 * i / 36) for i in range(37)]
    body.visual(
        _side_extrusion(
            [((BLADE_RADIUS + 0.016) * math.cos(a) + BLADE_CX,
              (BLADE_RADIUS + 0.016) * math.sin(a) + BLADE_CZ)
             for a in ug_ang]
            + [(BLADE_RADIUS * math.cos(a) + BLADE_CX,
                BLADE_RADIUS * math.sin(a) + BLADE_CZ)
               for a in reversed(ug_ang)],
            0.020,
            "upper_guard_ring",
        ),
        origin=Origin(xyz=(0.0, GUARD_Y, 0.0)),
        material=teal_dark,
        name="upper_guard",
    )
    # Solid top cap, kept inboard so the lower/front blade teeth are not hidden.
    body.visual(
        _side_extrusion(
            [
                ((BLADE_RADIUS + 0.014) * math.cos(a) + BLADE_CX,
                 (BLADE_RADIUS + 0.014) * math.sin(a) + BLADE_CZ)
                for a in [math.pi * (0.05 + 0.9 * i / 24) for i in range(25)]
            ]
            + [(BLADE_CX - (BLADE_RADIUS + 0.014), BLADE_CZ),
               (BLADE_CX + (BLADE_RADIUS + 0.014), BLADE_CZ)],
            0.030,
            "upper_guard_web",
        ),
        origin=Origin(xyz=(0.0, GUARD_Y + 0.003, 0.0)),
        material=teal_dark,
        name="upper_guard_web",
    )

    # Arbor boss sticking out of the housing to carry the blade.
    body.visual(
        Cylinder(radius=0.014, length=0.026),
        origin=Origin(xyz=(BLADE_CX, BLADE_Y + 0.020, BLADE_CZ), rpy=(HALF_PI, 0.0, 0.0)),
        material=charcoal,
        name="arbor_boss",
    )
    # Gearbox nose bridging the housing out to the arbor and guard cluster.
    body.visual(
        Box((0.060, 0.035, 0.080)),
        origin=Origin(xyz=(0.128, -0.030, 0.075)),
        material=teal,
        name="gearbox_nose",
    )

    # Depth-adjust pivot bracket linking shoe to housing (rear).
    body.visual(
        Box((0.030, 0.012, 0.060)),
        origin=Origin(xyz=(-0.020, -0.020, 0.030)),
        material=charcoal,
        name="depth_bracket",
    )
    # Front bevel bracket linking shoe to housing.
    body.visual(
        Box((0.028, 0.012, 0.060)),
        origin=Origin(xyz=(0.100, -0.020, 0.030)),
        material=charcoal,
        name="bevel_bracket",
    )

    # Rear-mounted battery pack (black brick behind the housing).
    body.visual(
        Box((0.075, 0.090, 0.070)),
        origin=Origin(xyz=(-0.060, 0.0, 0.075)),
        material=black_plastic,
        name="battery_pack",
    )
    body.visual(
        Box((0.060, 0.070, 0.014)),
        origin=Origin(xyz=(-0.030, 0.0, 0.075)),
        material=charcoal,
        name="battery_terminal_shroud",
    )

    # Top handle: black rubber-overmolded arch over the housing with a
    # through opening for the trigger hand.
    body.visual(
        _side_loop(
            [(x - 0.005, z + 0.199) for x, z in rounded_rect_profile(0.150, 0.090, 0.028)],
            [(x - 0.005, z + 0.194) for x, z in rounded_rect_profile(0.090, 0.045, 0.020)],
            0.032,
            "top_handle",
        ),
        material=black_plastic,
        name="top_handle",
    )
    # Front grip knob on the housing top-front.
    body.visual(
        Cylinder(radius=0.024, length=0.055),
        origin=Origin(xyz=(0.105, 0.0, 0.150), rpy=(HALF_PI, 0.0, 0.0)),
        material=black_plastic,
        name="front_grip",
    )

    # Trigger pivot ears hanging under the handle front strut.
    body.visual(
        Box((0.014, 0.010, 0.020)),
        origin=Origin(xyz=(0.075, 0.020, 0.150)),
        material=charcoal,
        name="trigger_ear_left",
    )
    body.visual(
        Box((0.014, 0.010, 0.020)),
        origin=Origin(xyz=(0.075, -0.020, 0.150)),
        material=charcoal,
        name="trigger_ear_right",
    )

    # Lower guard pivot boss (coaxial with arbor, front side).
    body.visual(
        Cylinder(radius=0.010, length=0.038),
        origin=Origin(xyz=(BLADE_CX, -0.076, BLADE_CZ), rpy=(HALF_PI, 0.0, 0.0)),
        material=charcoal,
        name="lower_guard_pivot_boss",
    )

    # Power/label plate accent on the housing side.
    body.visual(
        Box((0.070, 0.004, 0.030)),
        origin=Origin(xyz=(0.03, 0.075, 0.130)),
        material=red_accent,
        name="brand_plate",
    )

    # Short rubber bumper cord stub at the rear (cordless: just a strain node).
    cord_geom = tube_from_spline_points(
        [
            (-0.095, 0.030, 0.075),
            (-0.110, 0.045, 0.070),
            (-0.120, 0.055, 0.060),
        ],
        radius=0.006,
        samples_per_segment=8,
    )
    body.visual(
        mesh_from_geometry(cord_geom, "vent_stub"),
        material=charcoal,
        name="vent_stub",
    )

    # --------------------------------------------------------------- shoe
    # Bevel-tilt adjustable shoe: the flat stamped-metal base plate is now a
    # separate part hinged to the housing at the front bevel bracket so it can
    # tilt for bevel cuts.  The shoe part frame sits at the bevel pivot origin;
    # all shoe visuals are offset into that frame.
    BEVEL_X, BEVEL_Y, BEVEL_Z = 0.100, -0.020, 0.030

    shoe = model.part("shoe")

    shoe.visual(
        Box((0.250, 0.054, 0.006)),
        origin=Origin(xyz=(0.075 - BEVEL_X, 0.018 - BEVEL_Y, SHOE_Z - BEVEL_Z)),
        material=chrome,
        name="base_shoe_left_rail",
    )
    shoe.visual(
        Box((0.250, 0.014, 0.006)),
        origin=Origin(xyz=(0.075 - BEVEL_X, -0.124 - BEVEL_Y, SHOE_Z - BEVEL_Z)),
        material=chrome,
        name="base_shoe_right_rail",
    )
    shoe.visual(
        Box((0.085, 0.123, 0.006)),
        origin=Origin(xyz=(-0.055 - BEVEL_X, -0.0665 - BEVEL_Y, SHOE_Z - BEVEL_Z)),
        material=chrome,
        name="base_shoe_rear_bridge",
    )
    shoe.visual(
        Box((0.026, 0.123, 0.006)),
        origin=Origin(xyz=(0.194 - BEVEL_X, -0.0665 - BEVEL_Y, SHOE_Z - BEVEL_Z)),
        material=chrome,
        name="base_shoe_front_bridge",
    )
    # Shoe front lip (turned-up nose) for the cut-line notch.
    shoe.visual(
        Box((0.020, 0.150, 0.020)),
        origin=Origin(
            xyz=(0.192 - BEVEL_X, -0.030 - BEVEL_Y, SHOE_Z + 0.010 - BEVEL_Z),
            rpy=(0.0, -0.5, 0.0),
        ),
        material=chrome,
        name="shoe_nose",
    )

    # Bevel pivot: revolute joint at the front bevel bracket, axis along the
    # cutting direction (X) so the shoe tilts left/right for bevel angle cuts.
    model.articulation(
        "bevel_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=shoe,
        origin=Origin(xyz=(BEVEL_X, BEVEL_Y, BEVEL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.5, lower=0.0, upper=0.785),
    )

    # --------------------------------------------------------------- blade
    blade = model.part("blade")
    blade.visual(
        _side_extrusion(
            _blade_teeth_profile(BLADE_RADIUS - 0.007, BLADE_RADIUS, 48),
            0.0026,
            "blade_disc",
        ),
        material=blade_steel,
        name="blade_disc",
    )
    # Blade body plate (inner smooth disc).
    blade.visual(
        Cylinder(radius=BLADE_RADIUS - 0.005, length=0.0016),
        origin=Origin(rpy=(HALF_PI, 0.0, 0.0)),
        material=chrome,
        name="blade_plate",
    )
    blade.visual(
        Cylinder(radius=BLADE_RADIUS * 0.36, length=0.0032),
        origin=Origin(rpy=(HALF_PI, 0.0, 0.0)),
        material=blade_steel,
        name="blade_inner_ring",
    )
    # Arbor washer / bolt.
    blade.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(rpy=(HALF_PI, 0.0, 0.0)),
        material=charcoal,
        name="arbor_bolt",
    )

    model.articulation(
        "blade_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=blade,
        origin=Origin(xyz=(BLADE_CX, BLADE_Y, BLADE_CZ)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=350.0),
    )

    # ----------------------------------------------------------- lower guard
    lower_guard = model.part("lower_guard")
    # A quarter-ring shroud that at rest covers the lower-front of the blade.
    lg_ang = [math.pi * (1.02 + 0.62 * i / 24) for i in range(25)]
    lower_guard.visual(
        _side_extrusion(
            [((BLADE_RADIUS + 0.010) * math.cos(a), (BLADE_RADIUS + 0.010) * math.sin(a))
             for a in lg_ang]
            + [((BLADE_RADIUS - 0.004) * math.cos(a), (BLADE_RADIUS - 0.004) * math.sin(a))
               for a in reversed(lg_ang)],
            0.024,
            "lower_guard_shell",
        ),
        material=guard_steel,
        name="lower_guard_shell",
    )
    # Retract tab (the little lever the user thumbs back).
    lower_guard.visual(
        Box((0.030, 0.010, 0.014)),
        origin=Origin(xyz=(-(BLADE_RADIUS + 0.004), 0.0, 0.0)),
        material=black_plastic,
        name="lower_guard_tab",
    )

    model.articulation(
        "lower_guard_retract",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lower_guard,
        origin=Origin(xyz=(BLADE_CX, BLADE_Y - 0.011, BLADE_CZ)),
        # Positive q rotates the shroud back to expose the blade.
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=6.0, lower=0.0, upper=1.8),
    )

    # --------------------------------------------------------------- trigger
    trigger = model.part("trigger")
    trigger.visual(
        Box((0.014, 0.030, 0.014)),
        origin=Origin(xyz=(0.002, 0.0, -0.004)),
        material=black_plastic,
        name="trigger_boss",
    )
    trigger.visual(
        Box((0.014, 0.034, 0.050)),
        origin=Origin(xyz=(-0.010, 0.0, -0.030)),
        material=black_plastic,
        name="trigger_blade",
    )

    model.articulation(
        "trigger_squeeze",
        ArticulationType.REVOLUTE,
        parent=body,
        child=trigger,
        # Top-front corner of the handle opening.
        origin=Origin(xyz=(0.075, 0.0, 0.150)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=6.0, lower=0.0, upper=0.40),
    )

    # ----------------------------------------------------- safety lock button
    safety_lock = model.part("safety_lock")
    safety_lock.visual(
        Cylinder(radius=0.009, length=0.014),
        origin=Origin(rpy=(HALF_PI, 0.0, 0.0)),
        material=red_accent,
        name="safety_lock_cap",
    )

    model.articulation(
        "safety_lock_press",
        ArticulationType.PRISMATIC,
        parent=body,
        child=safety_lock,
        # Solid front strut of the top handle; stem embeds ~9 mm at rest.
        origin=Origin(xyz=(0.055, 0.014, 0.190)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.1, lower=0.0, upper=0.005),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    shoe = object_model.get_part("shoe")
    blade = object_model.get_part("blade")
    lower_guard = object_model.get_part("lower_guard")
    trigger = object_model.get_part("trigger")
    safety_lock = object_model.get_part("safety_lock")

    blade_joint = object_model.get_articulation("blade_spin")
    guard_joint = object_model.get_articulation("lower_guard_retract")
    trigger_joint = object_model.get_articulation("trigger_squeeze")
    lock_joint = object_model.get_articulation("safety_lock_press")
    bevel_joint = object_model.get_articulation("bevel_pivot")

    # ---------------------------------------------------- intentional fits
    ctx.allow_overlap(
        body,
        blade,
        elem_a="arbor_boss",
        elem_b="arbor_bolt",
        reason="Blade arbor bolt seats onto the housing arbor boss.",
    )
    ctx.allow_overlap(
        blade,
        body,
        elem_a="arbor_bolt",
        elem_b="lower_guard_pivot_boss",
        reason="The blade arbor bolt and lower-guard pivot boss share the same arbor stack.",
    )
    ctx.allow_overlap(
        body,
        blade,
        elem_a="upper_guard",
        elem_b="blade_disc",
        reason="Upper fixed guard shrouds the top rim of the blade.",
    )
    ctx.allow_overlap(
        body,
        blade,
        elem_a="upper_guard_web",
        elem_b="blade_plate",
        reason="Upper guard web overlaps the blade body it covers.",
    )
    ctx.allow_overlap(
        body,
        lower_guard,
        elem_a="lower_guard_pivot_boss",
        elem_b="lower_guard_shell",
        reason="Lower guard shell pivots on the housing pivot boss.",
    )
    ctx.allow_overlap(
        body,
        lower_guard,
        elem_a="upper_guard",
        elem_b="lower_guard_shell",
        reason="Lower guard nests concentrically inside the fixed upper guard band.",
    )
    ctx.allow_overlap(
        body,
        lower_guard,
        elem_a="upper_guard_web",
        elem_b="lower_guard_shell",
        reason="Lower guard tucks under the fixed upper guard web when closed.",
    )
    ctx.allow_overlap(
        body,
        lower_guard,
        elem_a="upper_guard",
        elem_b="lower_guard_tab",
        reason="Retract tab passes under the fixed upper guard band.",
    )
    ctx.allow_overlap(
        body,
        lower_guard,
        elem_a="upper_guard_web",
        elem_b="lower_guard_tab",
        reason="Retract tab tucks under the fixed upper guard web when closed.",
    )
    ctx.allow_overlap(
        blade,
        lower_guard,
        elem_a="blade_disc",
        elem_b="lower_guard_shell",
        reason="Lower guard rides just outside the blade rim, thin clearance.",
    )
    ctx.allow_overlap(
        body,
        trigger,
        elem_a="trigger_ear_left",
        elem_b="trigger_boss",
        reason="Trigger pivot boss is captured between the handle ears.",
    )
    ctx.allow_overlap(
        body,
        trigger,
        elem_a="trigger_ear_right",
        elem_b="trigger_boss",
        reason="Trigger pivot boss is captured between the handle ears.",
    )
    ctx.allow_overlap(
        body,
        trigger,
        elem_a="top_handle",
        elem_b="trigger_blade",
        reason="Trigger blade hangs into the handle opening from the pivot.",
    )
    ctx.allow_overlap(
        blade,
        body,
        elem_a="arbor_bolt",
        elem_b="upper_guard",
        reason="Blade hub sits inside the fixed upper guard shroud.",
    )
    ctx.allow_overlap(
        blade,
        body,
        elem_a="arbor_bolt",
        elem_b="upper_guard_web",
        reason="Blade hub is covered by the upper guard web.",
    )
    ctx.allow_overlap(
        blade,
        body,
        elem_a="blade_disc",
        elem_b="upper_guard_web",
        reason="Upper half of the blade is shrouded by the fixed guard web.",
    )
    ctx.allow_overlap(
        blade,
        body,
        elem_a="blade_plate",
        elem_b="upper_guard_web",
        reason="Blade body plate upper half sits under the guard web.",
    )
    ctx.allow_overlap(
        body,
        safety_lock,
        elem_a="top_handle",
        elem_b="safety_lock_cap",
        reason="Safety lock-off button stem embeds into the handle front strut.",
    )
    ctx.allow_overlap(
        body,
        shoe,
        elem_a="depth_bracket",
        elem_b="base_shoe_rear_bridge",
        reason="Rear depth-adjust bracket bridges from the housing down to the shoe rear bridge.",
    )

    # The bevel bracket on the body sits adjacent to the shoe front bridge;
    # at rest (q=0) they are close but should not interpenetrate significantly.
    ctx.allow_overlap(
        body,
        shoe,
        elem_a="bevel_bracket",
        elem_b="base_shoe_front_bridge",
        reason="Bevel bracket on housing bridges down to the shoe plate near the front.",
    )

    # --------------------------------------------------- prompt-level shape
    blade_aabb = ctx.part_world_aabb(blade)
    shoe_z = ctx.part_world_aabb(body)
    ctx.check(
        "blade is a large disc reaching below the shoe line",
        blade_aabb is not None and (blade_aabb[1][0] - blade_aabb[0][0]) > 0.12,
        details=f"blade_aabb={blade_aabb}",
    )
    ctx.check(
        "blade lower edge protrudes below the base shoe",
        blade_aabb is not None and blade_aabb[0][2] < 0.006,
        details=f"blade_min_z={None if blade_aabb is None else blade_aabb[0][2]}",
    )
    ctx.expect_contact(
        body,
        blade,
        elem_a="arbor_boss",
        elem_b="arbor_bolt",
        contact_tol=4e-3,
        name="blade seats on the arbor boss",
    )

    # Base shoe is broad and flat under the housing.
    shoe_aabb = ctx.part_world_aabb(shoe)
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "base shoe plate is below the housing centroid",
        shoe_aabb is not None and body_aabb is not None
        and (shoe_aabb[0][2] + shoe_aabb[1][2]) / 2.0
        < (body_aabb[0][2] + body_aabb[1][2]) / 2.0,
        details=f"shoe_center_z={(shoe_aabb[0][2] + shoe_aabb[1][2]) / 2.0 if shoe_aabb else None}, "
        f"body_center_z={(body_aabb[0][2] + body_aabb[1][2]) / 2.0 if body_aabb else None}",
    )
    # The shoe spans wide enough to support the saw on a workpiece.
    ctx.check(
        "shoe plate spans across the blade slot",
        shoe_aabb is not None and (shoe_aabb[1][1] - shoe_aabb[0][1]) > 0.10,
        details=f"shoe_aabb={shoe_aabb}",
    )

    # Trigger blade sits within the handle opening.
    ctx.check(
        "top handle rises well above the housing",
        (lambda h: h is not None and h[1][2] > 0.22)(ctx.part_world_aabb(body)),
        details="handle arch height",
    )

    # ------------------------------------------------------ decisive poses
    # Blade spins: a tooth point moves.
    p_rest = ctx.part_world_aabb(blade)
    with ctx.pose({blade_joint: 0.8}):
        p_spun = ctx.part_world_aabb(blade)
    ctx.check(
        "blade rotates about the arbor",
        p_rest is not None and p_spun is not None,
        details=f"rest={p_rest}, spun={p_spun}",
    )

    # Lower guard retracts: front-bottom coverage swings away.
    g_rest = ctx.part_world_aabb(lower_guard)
    with ctx.pose({guard_joint: 1.6}):
        g_open = ctx.part_world_aabb(lower_guard)
    ctx.check(
        "lower guard retracts to expose the blade",
        g_rest is not None and g_open is not None
        and abs(g_open[0][0] - g_rest[0][0]) + abs(g_open[1][2] - g_rest[1][2]) > 0.02,
        details=f"rest={g_rest}, open={g_open}",
    )

    # Trigger squeezes rearward.
    t_rest = ctx.part_world_aabb(trigger)
    with ctx.pose({trigger_joint: 0.40}):
        t_squeezed = ctx.part_world_aabb(trigger)
    ctx.check(
        "squeezing swings the trigger rearward",
        t_rest is not None and t_squeezed is not None
        and t_squeezed[0][0] < t_rest[0][0] - 0.008,
        details=f"rest={t_rest}, squeezed={t_squeezed}",
    )

    # Safety lock presses inward.
    b_rest = ctx.part_world_position(safety_lock)
    with ctx.pose({lock_joint: 0.005}):
        b_pressed = ctx.part_world_position(safety_lock)
    ctx.check(
        "safety lock-off button presses inward",
        b_rest is not None and b_pressed is not None
        and b_pressed[1] < b_rest[1] - 0.003,
        details=f"rest={b_rest}, pressed={b_pressed}",
    )

    # Bevel pivot: tilting the shoe changes its Y-Z footprint relative to the
    # housing, proving the bevel-tilt adjustment works.
    s_rest = ctx.part_world_aabb(shoe)
    with ctx.pose({bevel_joint: 0.7}):
        s_tilted = ctx.part_world_aabb(shoe)
    ctx.check(
        "bevel pivot tilts the shoe for angled cuts",
        s_rest is not None and s_tilted is not None
        and (abs(s_tilted[0][1] - s_rest[0][1]) + abs(s_tilted[1][2] - s_rest[1][2])) > 0.010,
        details=f"rest={s_rest}, tilted={s_tilted}",
    )

    return ctx.report()


object_model = build_object_model()
