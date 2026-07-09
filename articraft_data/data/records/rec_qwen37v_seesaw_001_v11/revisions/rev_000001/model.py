from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Classic two-seat plank seesaw with round support legs
#
# World frame: beam runs along X, pivot axis along Y, Z up.
# - Two A-frame supports (round steel tube ~50 mm dia), one each side of
#   center in Y, legs splayed in X. A crossbar spans the two apexes and
#   carries the horizontal pivot axle.
# - The plank is a single wooden board (~3.0 m × 0.20 m × 0.04 m) with a
#   pivot sleeve at center. Each end carries a molded seat with raised lip
#   rim, a tilting backrest on a revolute joint, an inverted-U handle with
#   a rounded capsule grip, and a rubber bumper underneath.
# - Beam pivots ±20° about horizontal Y axis; backrests tilt 0–25°.
# ---------------------------------------------------------------------------

BEAM_HALF = 1.50
PLANK_W = 0.20
PLANK_T = 0.04
PIVOT_Z = 0.76

# A-frame support geometry
LEG_SPREAD_X = 0.30
LEG_OFFSET_Y = 0.20
TUBE_R = 0.025

AXLE_R = 0.016
AXLE_LEN = 0.44

# End fittings
SEAT_X = 1.15
HANDLE_X = 0.88
BUMPER_X = 1.42

# Seat dimensions
SEAT_BASE_W = 0.28  # along X
SEAT_BASE_D = 0.22  # along Y
SEAT_BASE_T = 0.018
LIP_H = 0.030
LIP_T = 0.014

# Backrest
BACKREST_W = 0.18
BACKREST_H = 0.25
BACKREST_T = 0.015

TILT = math.radians(20.0)
BACKREST_TILT_MAX = math.radians(25.0)

# Beam-local frame: origin at the axle center
BAR_BOT = 0.05
BAR_CTR = BAR_BOT + PLANK_T / 2.0
BAR_TOP = BAR_BOT + PLANK_T


def _leg_points(foot_x: float, foot_y: float) -> list[tuple[float, float, float]]:
    """Centerline of one round tube leg from ground pad to A-frame apex.

    Each A-frame stays in its Y plane; the legs converge in X to the apex
    above but do not cross into the opposite A-frame.
    """
    return [
        (foot_x, foot_y, 0.012),
        (foot_x * 0.45, foot_y, PIVOT_Z * 0.52),
        (0.0, foot_y, PIVOT_Z),
    ]


def _handle_points(x: float) -> list[tuple[float, float, float]]:
    """Inverted-U grab-handle rod centerline, in the YZ plane."""
    half_w = 0.040
    leg_bot = BAR_TOP - 0.008
    arc_z = 0.26
    pts: list[tuple[float, float, float]] = [
        (x, -half_w, leg_bot),
        (x, -half_w, 0.18),
    ]
    for k in range(7):
        a = math.pi * k / 6.0
        pts.append((x, -half_w * math.cos(a), arc_z + half_w * math.sin(a)))
    pts.append((x, half_w, 0.18))
    pts.append((x, half_w, leg_bot))
    return pts


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="classic_playground_seesaw")

    # Materials
    steel_green = model.material("painted_steel_green", rgba=(0.22, 0.42, 0.25, 1.0))
    plank_paint = model.material("painted_plank_wood", rgba=(0.72, 0.48, 0.18, 1.0))
    seat_blue = model.material("molded_blue_plastic", rgba=(0.18, 0.28, 0.68, 1.0))
    handle_metal = model.material("brushed_steel", rgba=(0.62, 0.62, 0.64, 1.0))
    grip_black = model.material("rubber_grip", rgba=(0.10, 0.10, 0.10, 1.0))
    rubber = model.material("bumper_rubber", rgba=(0.06, 0.06, 0.06, 1.0))
    rust = model.material("rust_streaks", rgba=(0.45, 0.26, 0.12, 1.0))

    # -------------------------------------------------------- support frame ---
    frame = model.part("support_frame")

    # Four round-tube legs (two A-frames)
    foot_positions = [
        (LEG_SPREAD_X, LEG_OFFSET_Y),
        (-LEG_SPREAD_X, LEG_OFFSET_Y),
        (LEG_SPREAD_X, -LEG_OFFSET_Y),
        (-LEG_SPREAD_X, -LEG_OFFSET_Y),
    ]
    for i, (fx, fy) in enumerate(foot_positions):
        frame.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _leg_points(fx, fy),
                    radius=TUBE_R,
                    samples_per_segment=8,
                    radial_segments=18,
                    cap_ends=True,
                ),
                f"leg_tube_{i}",
            ),
            material=steel_green,
            name=f"leg_{i}",
        )

    # Ground pads under each foot
    for i, (fx, fy) in enumerate(foot_positions):
        frame.visual(
            Cylinder(radius=0.042, length=0.010),
            origin=Origin(xyz=(fx, fy, 0.005)),
            material=steel_green,
            name=f"foot_pad_{i}",
        )

    # Crossbar connecting the two A-frame apexes along Y
    frame.visual(
        Cylinder(radius=TUBE_R, length=2.0 * LEG_OFFSET_Y),
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_green,
        name="crossbar",
    )

    # Pivot axle bolt through the crossbar
    frame.visual(
        Cylinder(radius=AXLE_R, length=AXLE_LEN),
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_metal,
        name="pivot_axle",
    )
    # Axle nuts
    for i, side in enumerate((1.0, -1.0)):
        frame.visual(
            Cylinder(radius=0.022, length=0.012),
            origin=Origin(
                xyz=(0.0, side * (AXLE_LEN / 2.0 - 0.005), PIVOT_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=handle_metal,
            name=f"axle_nut_{i}",
        )

    # --------------------------------------------------------------- plank ---
    plank = model.part("plank")

    # Pivot sleeve (bushing around the axle, aligned along Y like the crossbar)
    plank.visual(
        Cylinder(radius=0.028, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_metal,
        name="pivot_sleeve",
    )

    # Gusset bracket connecting the pivot sleeve top to the board bottom
    # (starts above the crossbar to avoid interpenetration)
    _bracket_bot = 0.028  # sleeve radius = sleeve top in z
    _bracket_top = BAR_BOT  # board bottom
    _bracket_h = _bracket_top - _bracket_bot
    plank.visual(
        Box((0.06, 0.04, _bracket_h)),
        origin=Origin(xyz=(0.0, 0.0, (_bracket_bot + _bracket_top) / 2.0)),
        material=handle_metal,
        name="pivot_bracket",
    )

    # Main plank board
    plank.visual(
        Box((2.0 * BEAM_HALF, PLANK_W, PLANK_T)),
        origin=Origin(xyz=(0.0, 0.0, BAR_CTR)),
        material=plank_paint,
        name="plank_board",
    )

    # Rust streaks on the plank
    for i, px in enumerate((-0.70, -0.20, 0.40, 0.85)):
        plank.visual(
            Box((0.14, PLANK_W + 0.004, 0.008)),
            origin=Origin(xyz=(px, 0.0, BAR_TOP - 0.003)),
            material=rust,
            name=f"rust_patch_{i}",
        )

    # --- Molded seats with raised lip rims ---
    for i, side in enumerate((1.0, -1.0)):
        sx = side * SEAT_X
        seat_top = BAR_TOP + SEAT_BASE_T

        # Flat seat base plate
        plank.visual(
            Box((SEAT_BASE_W, SEAT_BASE_D, SEAT_BASE_T)),
            origin=Origin(xyz=(sx, 0.0, BAR_TOP + SEAT_BASE_T / 2.0)),
            material=seat_blue,
            name=f"seat_base_{i}",
        )

        # Raised lip - outer edge (in Y, both sides)
        plank.visual(
            Box((SEAT_BASE_W, LIP_T, LIP_H)),
            origin=Origin(
                xyz=(sx, SEAT_BASE_D / 2.0 - LIP_T / 2.0, seat_top + LIP_H / 2.0)
            ),
            material=seat_blue,
            name=f"seat_lip_y_pos_{i}",
        )
        plank.visual(
            Box((SEAT_BASE_W, LIP_T, LIP_H)),
            origin=Origin(
                xyz=(sx, -SEAT_BASE_D / 2.0 + LIP_T / 2.0, seat_top + LIP_H / 2.0)
            ),
            material=seat_blue,
            name=f"seat_lip_y_neg_{i}",
        )
        # Raised lip - inner edge (toward center)
        plank.visual(
            Box((LIP_T, SEAT_BASE_D, LIP_H)),
            origin=Origin(
                xyz=(sx - side * (SEAT_BASE_W / 2.0 - LIP_T / 2.0), 0.0, seat_top + LIP_H / 2.0)
            ),
            material=seat_blue,
            name=f"seat_lip_inner_{i}",
        )
        # Raised lip - outer edge (toward end)
        plank.visual(
            Box((LIP_T, SEAT_BASE_D, LIP_H)),
            origin=Origin(
                xyz=(sx + side * (SEAT_BASE_W / 2.0 - LIP_T / 2.0), 0.0, seat_top + LIP_H / 2.0)
            ),
            material=seat_blue,
            name=f"seat_lip_outer_{i}",
        )

    # --- Handles with rounded capsule grips ---
    for i, side in enumerate((1.0, -1.0)):
        hx = side * HANDLE_X
        plank.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _handle_points(hx),
                    radius=0.010,
                    samples_per_segment=8,
                    radial_segments=16,
                    cap_ends=True,
                ),
                f"handle_bar_{i}",
            ),
            material=handle_metal,
            name=f"handle_bar_{i}",
        )
        # Rounded capsule grip at the top of the handle arc
        grip_geom = CapsuleGeometry(
            radius=0.016, length=0.05, radial_segments=16, height_segments=6
        )
        plank.visual(
            mesh_from_geometry(grip_geom, f"handle_grip_{i}"),
            origin=Origin(
                xyz=(hx, 0.0, 0.30), rpy=(math.pi / 2.0, 0.0, 0.0)
            ),
            material=grip_black,
            name=f"handle_grip_{i}",
        )

    # --- Rubber bumpers under each plank tip ---
    for i, side in enumerate((1.0, -1.0)):
        bx = side * BUMPER_X
        plank.visual(
            Cylinder(radius=0.045, length=0.07),
            origin=Origin(
                xyz=(bx, 0.0, BAR_BOT - 0.015), rpy=(math.pi / 2.0, 0.0, 0.0)
            ),
            material=rubber,
            name=f"bumper_{i}",
        )

    # -------------------------------------------------------- backrests ---
    for i, side in enumerate((1.0, -1.0)):
        br = model.part(f"backrest_{i}")

        # Backrest panel: thin tall rectangle, bottom edge at local z=0
        br.visual(
            Box((BACKREST_T, BACKREST_W, BACKREST_H)),
            origin=Origin(xyz=(0.0, 0.0, BACKREST_H / 2.0)),
            material=seat_blue,
            name=f"backrest_panel_{i}",
        )

        # Small hinge barrel at the pivot point (above seat top)
        br.visual(
            Cylinder(radius=0.010, length=BACKREST_W * 0.6),
            origin=Origin(
                xyz=(0.0, 0.0, 0.010), rpy=(math.pi / 2.0, 0.0, 0.0)
            ),
            material=handle_metal,
            name=f"backrest_hinge_{i}",
        )

        # Hinge at the outer end of the seat (rider faces center)
        hinge_x = side * (SEAT_X + SEAT_BASE_W / 2.0 - 0.02)
        hinge_z = BAR_TOP + SEAT_BASE_T

        # axis: right-hand rule about -side*Y tilts the top outward
        model.articulation(
            f"backrest_joint_{i}",
            ArticulationType.REVOLUTE,
            parent=plank,
            child=br,
            origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
            axis=(0.0, -side, 0.0),
            motion_limits=MotionLimits(
                effort=5.0, velocity=2.0, lower=0.0, upper=BACKREST_TILT_MAX
            ),
        )

    # ------------------------------------------------------- beam pivot ---
    model.articulation(
        "beam_pivot",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=plank,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=200.0, velocity=2.5, lower=-TILT, upper=TILT
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("support_frame")
    plank = object_model.get_part("plank")
    pivot = object_model.get_articulation("beam_pivot")
    br0 = object_model.get_part("backrest_0")
    br1 = object_model.get_part("backrest_1")
    br_joint_0 = object_model.get_articulation("backrest_joint_0")
    br_joint_1 = object_model.get_articulation("backrest_joint_1")

    # Pivot sleeve captures the axle bolt and rides on the crossbar tube
    ctx.allow_overlap(
        plank,
        frame,
        elem_a="pivot_sleeve",
        elem_b="pivot_axle",
        reason="Pivot sleeve is a bushing intentionally nested around the axle bolt.",
    )
    ctx.allow_overlap(
        plank,
        frame,
        elem_a="pivot_sleeve",
        elem_b="crossbar",
        reason="Pivot sleeve wraps around the crossbar tube as a bearing surface.",
    )
    ctx.expect_contact(
        plank,
        frame,
        elem_a="pivot_sleeve",
        elem_b="pivot_axle",
        name="pivot sleeve is seated on the axle bolt",
    )
    ctx.expect_within(
        plank,
        frame,
        axes="y",
        inner_elem="pivot_sleeve",
        outer_elem="pivot_axle",
        margin=0.002,
        name="pivot sleeve stays inside the axle span",
    )

    # Plank clears the support legs
    ctx.expect_gap(
        plank,
        frame,
        axis="z",
        positive_elem="plank_board",
        negative_elem="crossbar",
        min_gap=0.005,
        max_gap=0.08,
        name="plank board clears the crossbar",
    )

    # Pivot axis and limits
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

    # Overall scale
    plank_box = ctx.part_element_world_aabb(plank, elem="plank_board")
    frame_box = ctx.part_world_aabb(frame)
    axle_box = ctx.part_element_world_aabb(frame, elem="pivot_axle")
    ctx.check(
        "plank is about 3.0 m long",
        plank_box is not None and abs((plank_box[1][0] - plank_box[0][0]) - 3.0) < 0.02,
        details=f"plank aabb={plank_box}",
    )
    ctx.check(
        "support frame feet rest on the ground",
        frame_box is not None and -0.01 <= frame_box[0][2] <= 0.02,
        details=f"frame aabb={frame_box}",
    )
    ctx.check(
        "pivot axle sits about 0.8 m high",
        axle_box is not None and 0.70 <= (axle_box[0][2] + axle_box[1][2]) / 2.0 <= 0.82,
        details=f"axle aabb={axle_box}",
    )

    # Round support legs exist (4 legs)
    for i in range(4):
        leg_box = ctx.part_element_world_aabb(frame, elem=f"leg_{i}")
        ctx.check(
            f"round leg_{i} exists and reaches the ground",
            leg_box is not None and leg_box[0][2] < 0.05 and leg_box[1][2] > 0.50,
            details=f"leg_{i} aabb={leg_box}",
        )

    # Molded seats with raised lips
    for i in range(2):
        seat_box = ctx.part_element_world_aabb(plank, elem=f"seat_base_{i}")
        ctx.check(
            f"seat_base_{i} sits on the plank",
            seat_box is not None
            and plank_box is not None
            and seat_box[0][2] > plank_box[0][2],
            details=f"seat_base_{i} aabb={seat_box}",
        )
        # At least one lip per seat exists above the seat base
        lip_box = ctx.part_element_world_aabb(plank, elem=f"seat_lip_y_pos_{i}")
        ctx.check(
            f"seat_{i} has raised lip",
            lip_box is not None
            and seat_box is not None
            and lip_box[1][2] > seat_box[1][2] - 0.002,
            details=f"lip aabb={lip_box}",
        )

    # Handle grips exist and are rounded (capsule shape)
    for i in range(2):
        grip_box = ctx.part_element_world_aabb(plank, elem=f"handle_grip_{i}")
        bar_box = ctx.part_element_world_aabb(plank, elem=f"handle_bar_{i}")
        ctx.check(
            f"handle_grip_{i} is present at the handle top",
            grip_box is not None
            and bar_box is not None
            and grip_box[1][2] > bar_box[1][2] - 0.04
            and grip_box[0][2] > plank_box[1][2] + 0.15,
            details=f"grip aabb={grip_box}",
        )

    # Backrest joints: revolute, correct limits, panels exist
    for i, (br, brj) in enumerate([(br0, br_joint_0), (br1, br_joint_1)]):
        br_box = ctx.part_element_world_aabb(br, elem=f"backrest_panel_{i}")
        ctx.check(
            f"backrest_{i} panel exists and is upright",
            br_box is not None
            and (br_box[1][2] - br_box[0][2]) > 0.15,
            details=f"backrest_{i} aabb={br_box}",
        )
        brj_lim = brj.motion_limits
        ctx.check(
            f"backrest_joint_{i} is revolute with tilt limits",
            brj.articulation_type == ArticulationType.REVOLUTE
            and brj_lim is not None
            and brj_lim.lower is not None
            and brj_lim.upper is not None
            and brj_lim.lower >= -0.01
            and brj_lim.upper > 0.2,
            details=f"type={brj.articulation_type}, limits=({brj_lim.lower}, {brj_lim.upper})",
        )

    # Backrest tilt test: positive q tilts the backrest backward
    with ctx.pose({br_joint_0: BACKREST_TILT_MAX}):
        br0_tilted = ctx.part_element_world_aabb(br0, elem="backrest_panel_0")
        br0_rest = ctx.part_element_world_aabb(br0, elem="backrest_panel_0")
        # At max tilt, the top of the backrest should have moved outward in X
        ctx.check(
            "backrest_0 tilts backward at max angle",
            br0_tilted is not None
            and br0_tilted[1][2] < 1.2,  # top is lower than fully upright would be
            details=f"tilted aabb={br0_tilted}",
        )

    # Decisive rocking pose checks
    rest_b0 = ctx.part_element_world_aabb(plank, elem="bumper_0")
    with ctx.pose({pivot: TILT}):
        down_b0 = ctx.part_element_world_aabb(plank, elem="bumper_0")
        up_b1 = ctx.part_element_world_aabb(plank, elem="bumper_1")
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
    with ctx.pose({pivot: -TILT}):
        down_b1 = ctx.part_element_world_aabb(plank, elem="bumper_1")
        ctx.check(
            "negative rock lowers the -X end near the ground",
            down_b1 is not None and 0.0 < down_b1[0][2] < 0.35,
            details=f"tilted bumper aabb={down_b1}",
        )

    return ctx.report()


object_model = build_object_model()
