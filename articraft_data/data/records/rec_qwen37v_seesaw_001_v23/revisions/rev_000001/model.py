from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    SphereGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Spring-assisted modern playground seesaw (variant 23)
#
# World frame: beam runs along X, pivot axis along Y, Z up.
# - Central pedestal base with visible compression spring around the column.
# - Pivot bracket on top of the pedestal carries the horizontal axle.
# - Beam is a modern steel bar with molded bucket seats (raised lips),
#   rounded-grip handles, and rubber bumpers.
# - Locking pin slides horizontally near the central bracket (prismatic joint)
#   to lock the beam at the pivot when fully inserted.
# - Single revolute joint at the pivot axle, axis (0, 1, 0), +/- 20 degrees.
# ---------------------------------------------------------------------------

BEAM_HALF = 1.50  # 3.0 m total beam
BEAM_W = 0.08
BEAM_T = 0.04
PIVOT_Z = 0.76  # axle height (~0.8 m)

# Pedestal
PEDESTAL_R = 0.06
PEDESTAL_H = 0.60
BASE_PLATE_R = 0.35
BASE_PLATE_H = 0.04

# Spring (helical coil around pedestal)
SPRING_WIRE_R = 0.012
SPRING_R = PEDESTAL_R + SPRING_WIRE_R  # wire inner surface contacts pedestal
SPRING_TURNS = 5
SPRING_Z_BOT = BASE_PLATE_H + SPRING_WIRE_R
SPRING_Z_TOP = PEDESTAL_H - 0.04

# Bracket at top of pedestal
BRACKET_W = 0.14
BRACKET_D = 0.10
BRACKET_H = 0.12

# Axle
AXLE_R = 0.016
AXLE_LEN = 0.18

# Beam local frame: origin at axle center
BAR_BOT = 0.05
BAR_CTR = BAR_BOT + BEAM_T / 2.0
BAR_TOP = BAR_BOT + BEAM_T

# End fittings
SEAT_X = 1.25
HANDLE_X = 0.95
BUMPER_X = 1.42

TILT = math.radians(20.0)

# Locking pin
PIN_R = 0.012
PIN_LEN = 0.10
PIN_HANDLE_R = 0.018
PIN_TRAVEL = 0.06  # prismatic travel


def _helix_points(
    radius: float,
    z_bot: float,
    z_top: float,
    turns: float,
    samples_per_turn: int = 24,
) -> list[tuple[float, float, float]]:
    """Centerline of a helical spring coil."""
    total_samples = int(turns * samples_per_turn) + 1
    pts: list[tuple[float, float, float]] = []
    for i in range(total_samples):
        t = i / (total_samples - 1)
        angle = 2.0 * math.pi * turns * t
        z = z_bot + (z_top - z_bot) * t
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        pts.append((x, y, z))
    return pts


def _molded_seat_geometry(index: int) -> object:
    """Molded bucket seat with raised lip: a dish-shaped lathed shell."""
    # Profile: (radius, z) points from center-bottom outward to raised rim
    profile = [
        (0.000, 0.000),   # center bottom
        (0.040, 0.002),   # inner floor
        (0.080, 0.008),   # seat bowl rising
        (0.110, 0.020),   # bowl wall
        (0.130, 0.040),   # side wall
        (0.140, 0.060),   # raised lip outer
        (0.138, 0.065),   # lip top
        (0.132, 0.058),   # lip inner drop
        (0.120, 0.038),   # inner wall back down
        (0.080, 0.018),
        (0.040, 0.012),
        (0.000, 0.010),   # inner floor back
    ]
    geom = LatheGeometry(profile, segments=28, closed=True)
    return mesh_from_geometry(geom, f"molded_seat_{index}")


def _handle_with_grip_geometry(index: int):
    """Bent rod handle with a rounded spherical grip on top."""
    half_w = 0.035
    leg_bot = BAR_TOP - 0.010
    arc_z = 0.260
    pts: list[tuple[float, float, float]] = [
        (0.0, -half_w, leg_bot),
        (0.0, -half_w, 0.180),
    ]
    for k in range(7):
        a = math.pi * k / 6.0
        pts.append((0.0, -half_w * math.cos(a), arc_z + half_w * math.sin(a)))
    pts.append((0.0, half_w, 0.180))
    pts.append((0.0, half_w, leg_bot))
    rod = tube_from_spline_points(
        pts,
        radius=0.009,
        samples_per_segment=8,
        radial_segments=16,
        cap_ends=True,
    )
    return mesh_from_geometry(rod, f"handle_rod_{index}")


def _bumper_geometry(x: float, index: int):
    """Curved rubber bumper under beam tip."""
    r_out = 0.060
    r_in = 0.045
    profile: list[tuple[float, float]] = []
    n = 16
    for k in range(n + 1):
        a = math.pi + math.pi * k / n
        profile.append((r_out * math.cos(a), r_out * math.sin(a)))
    for k in range(n + 1):
        a = 2.0 * math.pi - math.pi * k / n
        profile.append((r_in * math.cos(a), r_in * math.sin(a)))
    geom = ExtrudeGeometry(profile, 0.09, cap=True, center=True)
    geom.rotate_x(math.pi / 2.0)
    geom.translate(x, 0.0, BAR_BOT + 0.002)
    return mesh_from_geometry(geom, f"bumper_{index}")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="spring_modern_seesaw")

    # Materials - modern bright playground colors
    blue_steel = model.material("powder_blue_steel", rgba=(0.18, 0.38, 0.62, 1.0))
    red_beam = model.material("safety_red_paint", rgba=(0.82, 0.15, 0.12, 1.0))
    yellow_seat = model.material("bright_yellow_plastic", rgba=(0.95, 0.82, 0.10, 1.0))
    dark_grey = model.material("dark_grey_metal", rgba=(0.25, 0.25, 0.27, 1.0))
    spring_steel = model.material("spring_steel", rgba=(0.55, 0.58, 0.52, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    green_grip = model.material("rubber_grip_green", rgba=(0.12, 0.55, 0.22, 1.0))
    silver_pin = model.material("zinc_plated_pin", rgba=(0.72, 0.72, 0.70, 1.0))
    orange_knob = model.material("safety_orange", rgba=(0.95, 0.45, 0.05, 1.0))

    # --------------------------------------------------------------- base ---
    base = model.part("pedestal_base")

    # Wide ground plate
    base.visual(
        Cylinder(radius=BASE_PLATE_R, length=BASE_PLATE_H),
        origin=Origin(xyz=(0.0, 0.0, BASE_PLATE_H / 2.0), rpy=(0.0, 0.0, 0.0)),
        material=dark_grey,
        name="ground_plate",
    )

    # Central pedestal column
    base.visual(
        Cylinder(radius=PEDESTAL_R, length=PEDESTAL_H),
        origin=Origin(xyz=(0.0, 0.0, BASE_PLATE_H + PEDESTAL_H / 2.0), rpy=(0.0, 0.0, 0.0)),
        material=blue_steel,
        name="pedestal_column",
    )

    # Helical compression spring around pedestal
    base.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                _helix_points(SPRING_R, SPRING_Z_BOT, SPRING_Z_TOP, SPRING_TURNS),
                radius=SPRING_WIRE_R,
                samples_per_segment=6,
                radial_segments=12,
                cap_ends=True,
            ),
            "spring_coil",
        ),
        material=spring_steel,
        name="spring_coil",
    )

    # Pivot bracket (two side plates + top cap) at pedestal top
    bracket_z = BASE_PLATE_H + PEDESTAL_H
    for side in (1.0, -1.0):
        base.visual(
            Box((BRACKET_D, 0.012, BRACKET_H)),
            origin=Origin(
                xyz=(0.0, side * (BRACKET_W / 2.0 - 0.006), bracket_z + BRACKET_H / 2.0)
            ),
            material=blue_steel,
            name=f"bracket_plate_{0 if side > 0 else 1}",
        )
    # Bracket bottom plate (floor of the bracket cradle, below beam)
    base.visual(
        Box((BRACKET_D, BRACKET_W, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, bracket_z + 0.005)),
        material=blue_steel,
        name="bracket_cap",
    )

    # Horizontal axle through bracket
    axle_z = bracket_z + BRACKET_H * 0.5
    base.visual(
        Cylinder(radius=AXLE_R, length=AXLE_LEN),
        origin=Origin(xyz=(0.0, 0.0, axle_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_grey,
        name="pivot_axle",
    )

    # --------------------------------------------------------------- beam ---
    # Beam part frame at axle center so the revolute joint is at its origin.
    beam = model.part("beam")

    # Pivot sleeve (bushing around axle)
    beam.visual(
        Cylinder(radius=0.024, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_grey,
        name="pivot_sleeve",
    )

    # Main beam bar
    beam.visual(
        Box((2.0 * BEAM_HALF, BEAM_W, BEAM_T)),
        origin=Origin(xyz=(0.0, 0.0, BAR_CTR)),
        material=red_beam,
        name="beam_bar",
    )

    # Gusset/bracket plates connecting pivot sleeve to beam bar
    # They bridge from the sleeve top (z=0.024) to the bar top (z=BAR_TOP)
    gusset_h = BAR_TOP - 0.016  # spans from just above sleeve center to bar top
    gusset_cz = 0.016 + gusset_h / 2.0  # center z
    for side in (1.0, -1.0):
        beam.visual(
            Box((0.12, 0.010, gusset_h)),
            origin=Origin(xyz=(0.0, side * 0.020, gusset_cz)),
            material=red_beam,
            name=f"beam_gusset_{0 if side > 0 else 1}",
        )

    # End fittings for each side
    for i, side in enumerate((1.0, -1.0)):
        # Molded bucket seat with raised lips
        beam.visual(
            _molded_seat_geometry(i),
            origin=Origin(xyz=(side * SEAT_X, 0.0, BAR_TOP)),
            material=yellow_seat,
            name=f"seat_{i}",
        )

        # Handle rod (inverted U)
        beam.visual(
            _handle_with_grip_geometry(i),
            origin=Origin(xyz=(side * HANDLE_X, 0.0, 0.0)),
            material=dark_grey,
            name=f"handle_{i}",
        )

        # Rounded grip sphere on top of each handle (seated on arc peak)
        grip_z = 0.260 + 0.035 + 0.020  # arc_z + half_w + grip_r - small embed
        beam.visual(
            Sphere(radius=0.022),
            origin=Origin(xyz=(side * HANDLE_X, 0.0, grip_z)),
            material=green_grip,
            name=f"grip_ball_{i}",
        )

        # Rubber bumper under tip
        beam.visual(
            _bumper_geometry(side * BUMPER_X, i),
            material=black_rubber,
            name=f"bumper_{i}",
        )

    # -------------------------------------------------------- locking pin ---
    # The locking pin slides along Y near the pivot bracket.
    # When inserted (q=0), it passes through a hole in the bracket side plate
    # and the beam to lock the beam. When retracted (q=PIN_TRAVEL), the beam
    # is free to rock. Pin is placed below the axle to avoid interference.
    locking_pin = model.part("locking_pin")

    # Pin shaft (cylinder along Y, default Z-axis cylinder rotated)
    locking_pin.visual(
        Cylinder(radius=PIN_R, length=PIN_LEN),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=silver_pin,
        name="pin_shaft",
    )

    # Pin pull-knob at the outer end (+Y side)
    locking_pin.visual(
        Sphere(radius=PIN_HANDLE_R),
        origin=Origin(xyz=(0.0, PIN_LEN / 2.0 + PIN_HANDLE_R * 0.5, 0.0)),
        material=orange_knob,
        name="pin_knob",
    )

    # -------------------------------------------------------------- joints ---
    # Beam pivot: revolute about Y at the axle height
    model.articulation(
        "beam_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, axle_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=200.0, velocity=2.5, lower=-TILT, upper=TILT
        ),
    )

    # Locking pin: prismatic along Y, slides in/out of bracket side plate
    # At q=0 the pin is "inserted" through the bracket; at q=PIN_TRAVEL retracted
    # Pin is below the axle to avoid interference with the pivot axle
    pin_local_z = -0.035  # beam-local Z, below the axle center
    model.articulation(
        "pin_lock",
        ArticulationType.PRISMATIC,
        parent=beam,
        child=locking_pin,
        origin=Origin(xyz=(0.05, BRACKET_W / 2.0 - 0.01, pin_local_z)),
        axis=(0.0, 1.0, 0.0),  # slides along Y (in/out of bracket)
        motion_limits=MotionLimits(
            effort=30.0, velocity=0.5, lower=0.0, upper=PIN_TRAVEL
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("pedestal_base")
    beam = object_model.get_part("beam")
    locking_pin = object_model.get_part("locking_pin")
    pivot = object_model.get_articulation("beam_pivot")
    pin_joint = object_model.get_articulation("pin_lock")

    # --- Pivot sleeve captures the axle ---
    ctx.allow_overlap(
        beam,
        base,
        elem_a="pivot_sleeve",
        elem_b="pivot_axle",
        reason="Pivot sleeve is a bushing intentionally nested around the axle bolt.",
    )

    # --- Locking pin passes through bracket plate (intentional) ---
    ctx.allow_overlap(
        locking_pin,
        base,
        elem_a="pin_shaft",
        elem_b="bracket_plate_0",
        reason="Locking pin shaft passes through a hole in the bracket side plate when inserted.",
    )
    ctx.expect_contact(
        beam,
        base,
        elem_a="pivot_sleeve",
        elem_b="pivot_axle",
        name="pivot sleeve is seated on the axle bolt",
    )
    ctx.expect_within(
        beam,
        base,
        axes="y",
        inner_elem="pivot_sleeve",
        outer_elem="pivot_axle",
        margin=0.001,
        name="pivot sleeve stays inside the axle span",
    )

    # --- Beam bar above bracket bottom plate ---
    ctx.expect_gap(
        beam,
        base,
        axis="z",
        positive_elem="beam_bar",
        negative_elem="bracket_cap",
        min_gap=0.005,
        max_gap=0.12,
        name="beam bar clears the bracket bottom plate",
    )

    # --- Pivot joint: horizontal Y axis, +/- 20 deg ---
    ax = pivot.axis
    ctx.check(
        "pivot axis is horizontal and perpendicular to the beam",
        abs(ax[0]) < 1e-9 and abs(ax[1] - 1.0) < 1e-9 and abs(ax[2]) < 1e-9,
        details=f"axis={ax}",
    )
    lim = pivot.motion_limits
    ctx.check(
        "rocking limits are about +/- 20 degrees",
        lim is not None
        and lim.lower is not None
        and lim.upper is not None
        and abs(lim.lower + TILT) < 1e-6
        and abs(lim.upper - TILT) < 1e-6,
        details=f"limits=({lim.lower}, {lim.upper})",
    )

    # --- Locking pin joint: prismatic, non-fixed ---
    pin_ax = pin_joint.axis
    ctx.check(
        "locking pin joint is prismatic",
        pin_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={pin_joint.articulation_type}",
    )
    pin_lim = pin_joint.motion_limits
    ctx.check(
        "locking pin has non-zero travel range",
        pin_lim is not None
        and pin_lim.upper is not None
        and pin_lim.upper > 0.01,
        details=f"limits=({pin_lim.lower}, {pin_lim.upper})",
    )

    # --- Locking pin slides when posed ---
    pin_rest = ctx.part_world_position(locking_pin)
    with ctx.pose({pin_joint: PIN_TRAVEL}):
        pin_ext = ctx.part_world_position(locking_pin)
        ctx.check(
            "locking pin translates when posed to max travel",
            pin_rest is not None
            and pin_ext is not None
            and abs(pin_ext[1] - pin_rest[1]) > 0.02,
            details=f"rest={pin_rest}, extended={pin_ext}",
        )

    # --- Spring coil is visible on the base ---
    spring_box = ctx.part_element_world_aabb(base, elem="spring_coil")
    ctx.check(
        "spring coil is present around the pedestal",
        spring_box is not None
        and spring_box[1][2] - spring_box[0][2] > 0.3,
        details=f"spring aabb={spring_box}",
    )

    # --- Molded seats with raised lips (taller than a flat plate) ---
    for i in range(2):
        seat_box = ctx.part_element_world_aabb(beam, elem=f"seat_{i}")
        bar_box = ctx.part_element_world_aabb(beam, elem="beam_bar")
        ctx.check(
            f"molded seat_{i} has raised structure above the beam bar",
            seat_box is not None
            and bar_box is not None
            and seat_box[1][2] > bar_box[1][2] + 0.03,
            details=f"seat top={seat_box[1][2]:.4f}, bar top={bar_box[1][2]:.4f}",
        )

    # --- Rounded grip balls at both ends ---
    for i in range(2):
        grip_box = ctx.part_element_world_aabb(beam, elem=f"grip_ball_{i}")
        handle_box = ctx.part_element_world_aabb(beam, elem=f"handle_{i}")
        ctx.check(
            f"rounded grip_ball_{i} sits atop the handle",
            grip_box is not None
            and handle_box is not None
            and grip_box[0][2] > handle_box[1][2] - 0.03,
            details=f"grip={grip_box}, handle={handle_box}",
        )

    # --- Hero geometry: scale, base grounded ---
    bar_box = ctx.part_element_world_aabb(beam, elem="beam_bar")
    base_box = ctx.part_world_aabb(base)
    ctx.check(
        "beam is about 3.0 m long",
        bar_box is not None and abs((bar_box[1][0] - bar_box[0][0]) - 3.0) < 0.02,
        details=f"bar aabb={bar_box}",
    )
    ctx.check(
        "base rests on the ground",
        base_box is not None and -0.01 <= base_box[0][2] <= 0.02,
        details=f"base aabb={base_box}",
    )

    # --- Decisive pose: rocking lowers each end alternately ---
    rest_b0 = ctx.part_element_world_aabb(beam, elem="bumper_0")
    with ctx.pose({pivot: TILT}):
        down_b0 = ctx.part_element_world_aabb(beam, elem="bumper_0")
        up_b1 = ctx.part_element_world_aabb(beam, elem="bumper_1")
        ctx.check(
            "positive rock lowers the +X end near the ground",
            rest_b0 is not None
            and down_b0 is not None
            and down_b0[0][2] < rest_b0[0][2] - 0.35
            and down_b0[0][2] > 0.0,
            details=f"rest={rest_b0}, tilted={down_b0}",
        )
        ctx.check(
            "positive rock raises the -X end",
            up_b1 is not None and up_b1[0][2] > 1.0,
            details=f"raised bumper aabb={up_b1}",
        )

    return ctx.report()


object_model = build_object_model()
