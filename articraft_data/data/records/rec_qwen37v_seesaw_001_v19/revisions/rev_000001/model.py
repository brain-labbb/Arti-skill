from __future__ import annotations

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
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Vintage playground seesaw – variant 19
#
# Central A-frame support with visible axle brackets and axle caps.
# A central spring on a prismatic joint sits under the beam.
# Molded seats with raised lips at each end.
#
# World frame: beam along X, pivot axis along Y, Z up.
# ---------------------------------------------------------------------------

BEAM_HALF = 1.50
BEAM_W = 0.08
BEAM_T = 0.04
PIVOT_Z = 0.76

TUBE_R = 0.025

AXLE_R = 0.016
AXLE_LEN = 0.24

BAR_BOT = 0.05
BAR_CTR = BAR_BOT + BEAM_T / 2.0
BAR_TOP = BAR_BOT + BEAM_T

SEAT_X = 1.30
HANDLE_X = 1.04
BUMPER_X = 1.42
TILT = math.radians(20.0)

# A-frame geometry
AFOOT_Y = 0.36
AFOOT_Z = 0.02
ALEG_R = 0.025
CROSSBAR_Z = 0.40

# Bracket plates at apex
BRACKET_W = 0.06
BRACKET_H = 0.12
BRACKET_T = 0.008
BRACKET_Y = 0.056

# Axle caps
CAP_R = 0.022
CAP_T = 0.008

# Spring (offset along beam to avoid pivot sleeve overlap)
SPRING_X = 0.14
SPRING_MOUNT_Z = 0.62
SPRING_R = 0.024
SPRING_H = 0.12
SPRING_TRAVEL = 0.02

# Seat dimensions
SEAT_LEN = 0.28
SEAT_WID = 0.22
SEAT_THK = 0.016
LIP_H = 0.025
LIP_T = 0.010


def _aframe_leg_points(side: float) -> list[tuple[float, float, float]]:
    """One A-frame leg: foot to bracket top (not to pivot center).

    Each leg rises from its foot to the top of its bracket plate, stopping
    at y = side * BRACKET_Y so it merges with the bracket rather than
    reaching the pivot sleeve.
    """
    leg_top_z = PIVOT_Z + 0.04  # bracket plate top region
    rise = leg_top_z - AFOOT_Z
    pts: list[tuple[float, float, float]] = []
    for i in range(7):
        t = i / 6.0
        y = side * (AFOOT_Y * (1.0 - t) + BRACKET_Y * t)
        z = AFOOT_Z + rise * t
        pts.append((0.0, y, z))
    return pts


def _handle_points(x: float) -> list[tuple[float, float, float]]:
    """Inverted-U grab-handle rod centerline."""
    half_w = 0.035
    leg_bot = BAR_TOP - 0.010
    arc_z = 0.275
    pts: list[tuple[float, float, float]] = [
        (x, -half_w, leg_bot),
        (x, -half_w, 0.190),
    ]
    for k in range(7):
        a = math.pi * k / 6.0
        pts.append((x, -half_w * math.cos(a), arc_z + half_w * math.sin(a)))
    pts.append((x, half_w, 0.190))
    pts.append((x, half_w, leg_bot))
    return pts


def _bumper_geometry(x: float, index: int):
    """Curved tire-section bumper: half-annulus shell."""
    r_out = 0.065
    r_in = 0.048
    profile: list[tuple[float, float]] = []
    n = 20
    for k in range(n + 1):
        a = math.pi + math.pi * k / n
        profile.append((r_out * math.cos(a), r_out * math.sin(a)))
    for k in range(n + 1):
        a = 2.0 * math.pi - math.pi * k / n
        profile.append((r_in * math.cos(a), r_in * math.sin(a)))
    geom = ExtrudeGeometry(profile, 0.10, cap=True, center=True)
    geom.rotate_x(math.pi / 2.0)
    geom.translate(x, 0.0, BAR_BOT + 0.002)
    return mesh_from_geometry(geom, f"seesaw_bumper_{index}")


def _gusset_geometry():
    """Triangular gusset plate joining beam bar to pivot sleeve."""
    profile = [(-0.10, 0.055), (0.10, 0.055), (0.0, 0.020)]
    geom = ExtrudeGeometry(profile, 0.020, cap=True, center=True)
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, "seesaw_gusset_plate")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_playground_seesaw")

    # Materials
    galvanized = model.material("weathered_galvanized", rgba=(0.55, 0.58, 0.56, 1.0))
    rust = model.material("rust_steel", rgba=(0.42, 0.25, 0.13, 1.0))
    mustard = model.material("rusty_mustard_paint", rgba=(0.74, 0.53, 0.12, 1.0))
    pale_steel = model.material("pale_weathered_steel", rgba=(0.70, 0.66, 0.58, 1.0))
    rubber = model.material("black_rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    spring_steel = model.material("spring_steel", rgba=(0.40, 0.44, 0.48, 1.0))
    seat_green = model.material("molded_seat", rgba=(0.22, 0.42, 0.28, 1.0))
    bracket_steel = model.material("bracket_plate", rgba=(0.48, 0.48, 0.46, 1.0))
    cap_chrome = model.material("axle_cap", rgba=(0.72, 0.72, 0.70, 1.0))

    # --------------------------------------------------------- A-frame base ---
    base = model.part("aframe_base")

    # Two A-frame legs (bent tubes from feet to apex)
    for i, side in enumerate((1.0, -1.0)):
        base.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _aframe_leg_points(side),
                    radius=ALEG_R,
                    samples_per_segment=8,
                    radial_segments=18,
                    cap_ends=True,
                ),
                f"aframe_leg_{i}",
            ),
            material=galvanized,
            name=f"leg_{i}",
        )

    # Crossbar connecting the two legs partway up
    # Compute leg Y position at crossbar height using same interpolation as legs
    leg_top_z = PIVOT_Z + 0.04
    leg_t_at_crossbar = (CROSSBAR_Z - AFOOT_Z) / (leg_top_z - AFOOT_Z)
    leg_y_at_crossbar = AFOOT_Y * (1.0 - leg_t_at_crossbar) + BRACKET_Y * leg_t_at_crossbar
    crossbar_len = 2.0 * leg_y_at_crossbar  # spans between leg centers
    base.visual(
        Cylinder(radius=0.018, length=crossbar_len),
        origin=Origin(xyz=(0.0, 0.0, CROSSBAR_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="crossbar",
    )

    # Bracket plates at apex (two vertical plates flanking the beam)
    for i, side in enumerate((1.0, -1.0)):
        base.visual(
            Box((BRACKET_W, BRACKET_T, BRACKET_H)),
            origin=Origin(
                xyz=(0.0, side * BRACKET_Y, PIVOT_Z - BRACKET_H / 2.0 + 0.04)
            ),
            material=bracket_steel,
            name=f"bracket_{i}",
        )

    # Pivot axle through brackets
    base.visual(
        Cylinder(radius=AXLE_R, length=AXLE_LEN),
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rust,
        name="pivot_axle",
    )

    # Visible axle caps at outside of each bracket
    for i, side in enumerate((1.0, -1.0)):
        cap_y = side * (BRACKET_Y + BRACKET_T / 2.0 + CAP_T / 2.0)
        base.visual(
            Cylinder(radius=CAP_R, length=CAP_T),
            origin=Origin(xyz=(0.0, cap_y, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=cap_chrome,
            name=f"axle_cap_{i}",
        )

    # Foot plates (raised to embed into leg tube bottoms)
    for i, side in enumerate((1.0, -1.0)):
        base.visual(
            Cylinder(radius=0.05, length=0.016),
            origin=Origin(xyz=(0.0, side * AFOOT_Y, 0.018)),
            material=galvanized,
            name=f"foot_plate_{i}",
        )

    # Spring mount arm: horizontal box from leg junction to spring position
    arm_len = SPRING_X
    base.visual(
        Box((arm_len, 0.030, 0.020)),
        origin=Origin(xyz=(arm_len / 2.0, 0.0, SPRING_MOUNT_Z - 0.010)),
        material=bracket_steel,
        name="spring_arm",
    )

    # Vertical post connecting crossbar to spring arm (overlaps both)
    post_z_bot = CROSSBAR_Z
    post_z_top = SPRING_MOUNT_Z  # extend into spring arm
    post_h = post_z_top - post_z_bot
    if post_h > 0.005:
        base.visual(
            Box((0.020, 0.020, post_h)),
            origin=Origin(xyz=(0.0, 0.0, post_z_bot + post_h / 2.0)),
            material=bracket_steel,
            name="spring_post",
        )

    # Spring mount plate
    base.visual(
        Cylinder(radius=SPRING_R + 0.008, length=0.006),
        origin=Origin(xyz=(SPRING_X, 0.0, SPRING_MOUNT_Z - 0.003)),
        material=bracket_steel,
        name="spring_mount_plate",
    )

    # --------------------------------------------------------- spring ---
    spring = model.part("central_spring")

    # Spring body (cylinder housing)
    spring.visual(
        Cylinder(radius=SPRING_R, length=SPRING_H),
        origin=Origin(xyz=(0.0, 0.0, SPRING_H / 2.0)),
        material=spring_steel,
        name="spring_body",
    )

    # Coil ring details (torus rings stacked along the spring body)
    n_coils = 6
    for k in range(n_coils):
        ring_z = SPRING_H * (k + 0.5) / n_coils
        coil_mesh = mesh_from_geometry(
            TorusGeometry(SPRING_R, 0.005, radial_segments=12, tubular_segments=24),
            f"spring_coil_{k}",
        )
        spring.visual(
            coil_mesh,
            origin=Origin(xyz=(0.0, 0.0, ring_z)),
            material=spring_steel,
            name=f"spring_coil_{k}",
        )

    # Spring top contact plate
    spring.visual(
        Cylinder(radius=SPRING_R + 0.006, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, SPRING_H + 0.0025)),
        material=pale_steel,
        name="spring_top_plate",
    )

    # --------------------------------------------------------- beam ---
    beam = model.part("beam")

    # Pivot sleeve (bushing around axle)
    beam.visual(
        Cylinder(radius=0.026, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rust,
        name="pivot_sleeve",
    )
    beam.visual(_gusset_geometry(), material=mustard, name="gusset_plate")

    # Beam bar
    beam.visual(
        Box((2.0 * BEAM_HALF, BEAM_W, BEAM_T)),
        origin=Origin(xyz=(0.0, 0.0, BAR_CTR)),
        material=mustard,
        name="beam_bar",
    )

    # Rust streak patches
    for i, px in enumerate((-0.85, -0.30, 0.45, 0.95)):
        beam.visual(
            Box((0.16, BEAM_W + 0.004, 0.012)),
            origin=Origin(xyz=(px, 0.0, BAR_TOP - 0.004)),
            material=rust,
            name=f"rust_patch_{i}",
        )

    # End fittings: molded seats with raised lips, handles, bumpers
    for i, side in enumerate((1.0, -1.0)):
        sx = side * SEAT_X

        # Molded seat base
        beam.visual(
            Box((SEAT_LEN, SEAT_WID, SEAT_THK)),
            origin=Origin(xyz=(sx, 0.0, BAR_TOP + SEAT_THK / 2.0)),
            material=seat_green,
            name=f"seat_base_{i}",
        )

        # Raised lip walls around the seat edges
        seat_top = BAR_TOP + SEAT_THK
        lip_cz = seat_top + LIP_H / 2.0
        half_l = SEAT_LEN / 2.0
        half_w = SEAT_WID / 2.0

        # Front lip (+Y edge)
        beam.visual(
            Box((SEAT_LEN, LIP_T, LIP_H)),
            origin=Origin(xyz=(sx, half_w - LIP_T / 2.0, lip_cz)),
            material=seat_green,
            name=f"seat_lip_front_{i}",
        )
        # Back lip (-Y edge)
        beam.visual(
            Box((SEAT_LEN, LIP_T, LIP_H)),
            origin=Origin(xyz=(sx, -(half_w - LIP_T / 2.0), lip_cz)),
            material=seat_green,
            name=f"seat_lip_back_{i}",
        )
        # Outer lip (away from center, at the beam tip end)
        beam.visual(
            Box((LIP_T, SEAT_WID, LIP_H)),
            origin=Origin(xyz=(sx + side * (half_l - LIP_T / 2.0), 0.0, lip_cz)),
            material=seat_green,
            name=f"seat_lip_outer_{i}",
        )
        # Inner lip (toward center)
        beam.visual(
            Box((LIP_T, SEAT_WID, LIP_H)),
            origin=Origin(xyz=(sx - side * (half_l - LIP_T / 2.0), 0.0, lip_cz)),
            material=seat_green,
            name=f"seat_lip_inner_{i}",
        )

        # Grab handle
        beam.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _handle_points(side * HANDLE_X),
                    radius=0.009,
                    samples_per_segment=8,
                    radial_segments=16,
                    cap_ends=True,
                ),
                f"seesaw_handle_{i}",
            ),
            material=pale_steel,
            name=f"handle_{i}",
        )

        # Tire-section bumper
        beam.visual(
            _bumper_geometry(side * BUMPER_X, i),
            material=rubber,
            name=f"bumper_{i}",
        )

    # --------------------------------------------------------- joints ---
    # Spring prismatic joint: vertical compression under the beam
    model.articulation(
        "spring_compress",
        ArticulationType.PRISMATIC,
        parent=base,
        child=spring,
        origin=Origin(xyz=(SPRING_X, 0.0, SPRING_MOUNT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=300.0, velocity=0.10, lower=0.0, upper=SPRING_TRAVEL
        ),
    )

    # Beam revolute pivot joint
    model.articulation(
        "beam_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=200.0, velocity=2.5, lower=-TILT, upper=TILT
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("aframe_base")
    spring = object_model.get_part("central_spring")
    beam = object_model.get_part("beam")
    pivot = object_model.get_articulation("beam_pivot")
    spring_joint = object_model.get_articulation("spring_compress")

    # ---- Pivot sleeve captures axle (intentional overlap) ----
    ctx.allow_overlap(
        beam,
        base,
        elem_a="pivot_sleeve",
        elem_b="pivot_axle",
        reason="Pivot sleeve is a bushing intentionally nested around the axle bolt.",
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

    # ---- Beam bar clears the A-frame ----
    ctx.expect_gap(
        beam,
        base,
        axis="z",
        positive_elem="beam_bar",
        negative_elem="bracket_0",
        min_gap=0.001,
        max_gap=0.08,
        name="beam bar clears the bracket plates",
    )

    # ---- A-frame bracket plates exist at apex ----
    bracket_box = ctx.part_element_world_aabb(base, elem="bracket_0")
    ctx.check(
        "bracket plate exists at the A-frame apex",
        bracket_box is not None and bracket_box[1][2] > PIVOT_Z - 0.04,
        details=f"bracket aabb={bracket_box}",
    )

    # ---- Axle caps outside brackets ----
    for i in range(2):
        cap_box = ctx.part_element_world_aabb(base, elem=f"axle_cap_{i}")
        brk_box = ctx.part_element_world_aabb(base, elem=f"bracket_{i}")
        ctx.check(
            f"axle_cap_{i} is outside bracket_{i}",
            cap_box is not None
            and brk_box is not None
            and (
                cap_box[0][1] > brk_box[1][1] - 0.002
                or cap_box[1][1] < brk_box[0][1] + 0.002
            ),
            details=f"cap={cap_box}, bracket={brk_box}",
        )

    # ---- Spring prismatic joint configuration ----
    s_axis = spring_joint.axis
    ctx.check(
        "spring joint axis is vertical (Z)",
        abs(s_axis[0]) < 1e-9 and abs(s_axis[1]) < 1e-9 and abs(s_axis[2] - 1.0) < 1e-9,
        details=f"axis={s_axis}",
    )
    s_lim = spring_joint.motion_limits
    ctx.check(
        "spring joint has valid compression limits",
        s_lim is not None
        and s_lim.lower is not None
        and s_lim.upper is not None
        and s_lim.lower >= 0.0
        and s_lim.upper > s_lim.lower,
        details=f"limits=({s_lim.lower}, {s_lim.upper})",
    )

    # ---- Spring sits between base and beam ----
    spring_box = ctx.part_element_world_aabb(spring, elem="spring_body")
    bar_box = ctx.part_element_world_aabb(beam, elem="beam_bar")
    ctx.check(
        "spring body is below the beam bar",
        spring_box is not None
        and bar_box is not None
        and spring_box[1][2] < bar_box[1][2],
        details=f"spring={spring_box}, bar={bar_box}",
    )

    # ---- Spring compression moves upward ----
    rest_spring = ctx.part_element_world_aabb(spring, elem="spring_top_plate")
    with ctx.pose({spring_joint: SPRING_TRAVEL}):
        compressed_spring = ctx.part_element_world_aabb(spring, elem="spring_top_plate")
        ctx.check(
            "spring compresses upward at max travel",
            rest_spring is not None
            and compressed_spring is not None
            and compressed_spring[0][2] > rest_spring[0][2] + 0.005,
            details=f"rest={rest_spring}, compressed={compressed_spring}",
        )

    # ---- Molded seats have raised lips above seat surface ----
    for i in range(2):
        seat_box = ctx.part_element_world_aabb(beam, elem=f"seat_base_{i}")
        lip_box = ctx.part_element_world_aabb(beam, elem=f"seat_lip_front_{i}")
        ctx.check(
            f"seat_{i} has raised lip above the seat surface",
            seat_box is not None
            and lip_box is not None
            and lip_box[1][2] > seat_box[1][2] + 0.010,
            details=f"seat={seat_box}, lip={lip_box}",
        )

    # ---- Revolute pivot joint configuration ----
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

    # ---- Scale and proportions ----
    base_box = ctx.part_world_aabb(base)
    axle_box = ctx.part_element_world_aabb(base, elem="pivot_axle")
    ctx.check(
        "beam is about 3.0 m long",
        bar_box is not None and abs((bar_box[1][0] - bar_box[0][0]) - 3.0) < 0.02,
        details=f"bar aabb={bar_box}",
    )
    ctx.check(
        "A-frame base feet rest on the ground",
        base_box is not None and -0.01 <= base_box[0][2] <= 0.02,
        details=f"base aabb={base_box}",
    )
    ctx.check(
        "pivot axle sits about 0.8 m high",
        axle_box is not None and 0.70 <= (axle_box[0][2] + axle_box[1][2]) / 2.0 <= 0.82,
        details=f"axle aabb={axle_box}",
    )

    # ---- End fittings positioning ----
    for i in range(2):
        handle = ctx.part_element_world_aabb(beam, elem=f"handle_{i}")
        bumper = ctx.part_element_world_aabb(beam, elem=f"bumper_{i}")
        ctx.check(
            f"handle_{i} stands above the beam",
            handle is not None
            and bar_box is not None
            and handle[1][2] > bar_box[1][2] + 0.18
            and handle[0][2] < bar_box[1][2],
            details=f"handle aabb={handle}",
        )
        ctx.check(
            f"bumper_{i} hangs below the beam tip",
            bumper is not None
            and bar_box is not None
            and bumper[0][2] < bar_box[0][2]
            and min(abs(bumper[0][0]), abs(bumper[1][0])) > 1.3,
            details=f"bumper aabb={bumper}",
        )

    # ---- Decisive rocking poses ----
    rest_b0 = ctx.part_element_world_aabb(beam, elem="bumper_0")
    with ctx.pose({pivot: TILT}):
        down_b0 = ctx.part_element_world_aabb(beam, elem="bumper_0")
        up_b1 = ctx.part_element_world_aabb(beam, elem="bumper_1")
        ctx.check(
            "positive rock lowers the +X end near the ground",
            rest_b0 is not None
            and down_b0 is not None
            and down_b0[0][2] < rest_b0[0][2] - 0.40
            and down_b0[0][2] > 0.0,
            details=f"rest={rest_b0}, tilted={down_b0}",
        )
        ctx.check(
            "positive rock raises the -X end",
            up_b1 is not None and up_b1[0][2] > 1.0,
            details=f"raised bumper aabb={up_b1}",
        )
    with ctx.pose({pivot: -TILT}):
        down_b1 = ctx.part_element_world_aabb(beam, elem="bumper_1")
        ctx.check(
            "negative rock lowers the -X end near the ground",
            down_b1 is not None and 0.0 < down_b1[0][2] < 0.32,
            details=f"tilted bumper aabb={down_b1}",
        )

    return ctx.report()


object_model = build_object_model()
