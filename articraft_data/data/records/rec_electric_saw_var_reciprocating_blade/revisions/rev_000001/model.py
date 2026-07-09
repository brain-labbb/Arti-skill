from __future__ import annotations

"""Cordless electric reciprocating saw with a teal motor housing.

Canonical frame: +X points forward (toward the front of the tool / blade
travel direction), +Z is up, +Y is the left side (motor-cap side). Real-world
scale: about 0.40 m overall length with a 150 mm reciprocating blade.

Articulated mechanisms:
- blade_stroke: the straight reciprocating blade strokes back and forth
  along +X (prismatic).
- trigger_squeeze: the squeeze trigger inside the top handle (revolute).
- safety_lock_press: the side safety lock-off button (prismatic).
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
BLADE_LENGTH = 0.150       # reciprocating blade cutting length
BLADE_HEIGHT = 0.022       # blade width (top to bottom edge)
BLADE_THICKNESS = 0.0014   # blade plate thickness
BLADE_STROKE = 0.028       # full stroke (back-and-forth travel)
# Clamp/chuck location: front face of the gearbox nose.
CLAMP_X = 0.158            # front of gearbox_nose (0.128 + 0.030)
CLAMP_Y = -0.030           # centered on gearbox_nose Y
CLAMP_Z = 0.075            # centered on gearbox_nose Z
SHOE_Z = 0.037             # raised shoe top


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


def _reciprocating_blade_profile(length: float, height: float, n_teeth: int):
    """Side-view (XZ) profile of a reciprocating saw blade with teeth.

    The profile extends from X=0 (shank) to X=length (tip).
    The top edge is straight. The bottom edge has n_teeth triangular teeth.
    The tip tapers to a point.
    """
    half_h = height / 2.0
    tooth_depth = height * 0.22
    tooth_pitch = length * 0.85 / n_teeth  # teeth cover 85% of length
    tooth_start_x = length * 0.05  # first tooth starts 5% in from shank

    # Build the outline clockwise from the shank end (top-left corner).
    pts = []

    # Shank end (top-left) to top edge
    pts.append((0.0, half_h))
    # Top edge to near tip
    pts.append((length * 0.92, half_h))
    # Tip (tapers to a point)
    pts.append((length, 0.0))

    # Bottom edge with teeth, from tip back to shank
    for i in range(n_teeth):
        x0 = tooth_start_x + (n_teeth - 1 - i) * tooth_pitch + tooth_pitch
        x_mid = x0 - tooth_pitch * 0.5
        x1 = x0 - tooth_pitch
        # Tooth valley (deeper cut)
        pts.append((x0, -half_h))
        # Tooth peak (tooth tip)
        pts.append((x_mid, -half_h - tooth_depth))
        # Next valley
        pts.append((x1, -half_h))

    # Close back to shank
    pts.append((0.0, -half_h))
    # Shank tang (narrower section at rear for clamping)
    pts.append((0.0, half_h))

    return pts


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cordless_reciprocating_saw")

    teal = model.material("teal_housing", rgba=(0.10, 0.52, 0.55, 1.0))
    teal_dark = model.material("teal_dark", rgba=(0.07, 0.40, 0.43, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.09, 0.09, 0.10, 1.0))
    charcoal = model.material("charcoal_plastic", rgba=(0.17, 0.18, 0.19, 1.0))
    chrome = model.material("chrome_steel", rgba=(0.82, 0.83, 0.85, 1.0))
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

    # Gearbox nose bridging the housing out to the blade clamp area.
    body.visual(
        Box((0.060, 0.035, 0.080)),
        origin=Origin(xyz=(0.128, -0.030, 0.075)),
        material=teal,
        name="gearbox_nose",
    )

    # Blade clamp / chuck at front of gearbox (where blade inserts).
    body.visual(
        Cylinder(radius=0.016, length=0.022),
        origin=Origin(xyz=(CLAMP_X, CLAMP_Y, CLAMP_Z), rpy=(0.0, HALF_PI, 0.0)),
        material=charcoal,
        name="blade_clamp",
    )
    # Clamp collar ring around the chuck.
    body.visual(
        Cylinder(radius=0.020, length=0.008),
        origin=Origin(xyz=(CLAMP_X + 0.005, CLAMP_Y, CLAMP_Z), rpy=(0.0, HALF_PI, 0.0)),
        material=chrome,
        name="clamp_collar",
    )

    # Front shoe / foot — a pivoting metal foot that rests against the workpiece.
    # Two side plates flanking the blade slot, connected by a front bridge.
    body.visual(
        Box((0.050, 0.012, 0.050)),
        origin=Origin(xyz=(CLAMP_X + 0.020, CLAMP_Y + 0.024, CLAMP_Z - 0.012)),
        material=chrome,
        name="shoe_left_plate",
    )
    body.visual(
        Box((0.050, 0.012, 0.050)),
        origin=Origin(xyz=(CLAMP_X + 0.020, CLAMP_Y - 0.024, CLAMP_Z - 0.012)),
        material=chrome,
        name="shoe_right_plate",
    )
    body.visual(
        Box((0.008, 0.060, 0.050)),
        origin=Origin(xyz=(CLAMP_X + 0.048, CLAMP_Y, CLAMP_Z - 0.012)),
        material=chrome,
        name="shoe_front_bridge",
    )
    # Shoe top plate connecting the side plates above the blade slot.
    body.visual(
        Box((0.050, 0.036, 0.006)),
        origin=Origin(xyz=(CLAMP_X + 0.020, CLAMP_Y, CLAMP_Z + 0.014)),
        material=chrome,
        name="shoe_top_plate",
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

    # --------------------------------------------------------------- blade
    blade = model.part("blade")

    # Reciprocating saw blade — straight elongated blade with teeth on one edge.
    # Blade part origin is at the clamp point; blade extends forward in +X.
    blade_profile = _reciprocating_blade_profile(BLADE_LENGTH, BLADE_HEIGHT, 18)
    blade.visual(
        _side_extrusion(blade_profile, BLADE_THICKNESS, "blade_body"),
        material=blade_steel,
        name="blade_body",
    )

    # Shank (the flat tang that inserts into the chuck, at the blade rear).
    # Overlaps with the blade body rear edge for geometric connectivity.
    blade.visual(
        Box((0.022, BLADE_THICKNESS + 0.001, 0.013)),
        origin=Origin(xyz=(-0.008, 0.0, 0.0)),
        material=chrome,
        name="blade_shank",
    )

    model.articulation(
        "blade_stroke",
        ArticulationType.PRISMATIC,
        parent=body,
        child=blade,
        origin=Origin(xyz=(CLAMP_X, CLAMP_Y, CLAMP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.8,
            lower=-BLADE_STROKE / 2.0,
            upper=BLADE_STROKE / 2.0,
        ),
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
    blade = object_model.get_part("blade")
    trigger = object_model.get_part("trigger")
    safety_lock = object_model.get_part("safety_lock")

    blade_joint = object_model.get_articulation("blade_stroke")
    trigger_joint = object_model.get_articulation("trigger_squeeze")
    lock_joint = object_model.get_articulation("safety_lock_press")

    # ---------------------------------------------------- intentional fits
    # Blade shank inserts into the clamp chuck — small overlap is intentional.
    ctx.allow_overlap(
        body,
        blade,
        elem_a="blade_clamp",
        elem_b="blade_shank",
        reason="Blade shank inserts into the clamp chuck for retention.",
    )
    ctx.allow_overlap(
        body,
        blade,
        elem_a="clamp_collar",
        elem_b="blade_shank",
        reason="Clamp collar surrounds the blade shank insertion point.",
    )
    # Trigger mounting fits.
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
        body,
        safety_lock,
        elem_a="top_handle",
        elem_b="safety_lock_cap",
        reason="Safety lock-off button stem embeds into the handle front strut.",
    )

    # --------------------------------------------------- prompt-level shape
    # The blade is a straight elongated reciprocating blade projecting forward.
    blade_aabb = ctx.part_world_aabb(blade)
    ctx.check(
        "blade is a straight elongated reciprocating blade",
        blade_aabb is not None and (blade_aabb[1][0] - blade_aabb[0][0]) > 0.10,
        details=f"blade X extent should be >0.10m, got {blade_aabb}",
    )
    # The blade should be thin (not a disc) — Z extent is the blade height.
    ctx.check(
        "blade is narrow in height (reciprocating, not a disc)",
        blade_aabb is not None and (blade_aabb[1][2] - blade_aabb[0][2]) < 0.04,
        details=f"blade Z extent should be <0.04m, got {blade_aabb}",
    )
    # Blade projects forward past the gearbox nose.
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "blade tip extends forward past the gearbox nose",
        blade_aabb is not None and body_aabb is not None
        and blade_aabb[1][0] > body_aabb[1][0] - 0.01,
        details=f"blade_max_x={blade_aabb[1][0] if blade_aabb else None}, body_max_x={body_aabb[1][0] if body_aabb else None}",
    )

    # Base shoe sits at the bottom of the assembly.
    ctx.check(
        "shoe sits at the bottom of the assembly",
        body_aabb is not None and body_aabb[0][2] < 0.05,
        details=f"body_min_z={None if body_aabb is None else body_aabb[0][2]}",
    )

    # Top handle rises well above the housing.
    ctx.check(
        "top handle rises well above the housing",
        (lambda h: h is not None and h[1][2] > 0.22)(ctx.part_world_aabb(body)),
        details="handle arch height",
    )

    # Verify there is no upper_guard or lower_guard (disc-only shrouds removed).
    body_visual_names = [v.name for v in body.visuals]
    ctx.check(
        "no disc-only upper_guard on reciprocating saw",
        "upper_guard" not in body_visual_names,
        details=f"body visuals: {body_visual_names}",
    )
    part_names = [p.name for p in object_model.parts]
    ctx.check(
        "no disc-only lower_guard part on reciprocating saw",
        "lower_guard" not in part_names,
        details=f"parts: {part_names}",
    )

    # ------------------------------------------------------ decisive poses
    # Blade strokes forward: at upper limit, blade tip moves forward.
    p_rest = ctx.part_world_aabb(blade)
    with ctx.pose({blade_joint: BLADE_STROKE / 2.0}):
        p_forward = ctx.part_world_aabb(blade)
    ctx.check(
        "blade strokes forward along +X (prismatic reciprocating drive)",
        p_rest is not None and p_forward is not None
        and p_forward[1][0] > p_rest[1][0] + 0.005,
        details=f"rest={p_rest}, forward={p_forward}",
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

    return ctx.report()


object_model = build_object_model()
