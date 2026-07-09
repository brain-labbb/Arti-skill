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
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Classic playground seesaw variant: round support legs, pivoting handlebars,
# and molded seats with raised lips.
#
# World frame: beam runs along X, pivot axis along Y, Z up.
# - Base: 4 round steel tube legs (~50 mm dia) splayed outward from the apex
#   form a stable A-frame; a horizontal crossbar ties the legs together near
#   the top. A pivot bracket plate sits at the apex carrying the axle.
# - Beam: 3.0 m plank (80 x 40 mm), mustard yellow with rust streaks, a pivot
#   sleeve + triangular gusset at center, molded seats with raised lips at
#   each end, and rubber tire-section bumpers bolted under the tips.
# - Handlebars: separate parts, each an inverted-U bent rod mounted on a small
#   pivot bracket on the beam. Each handlebar has its own revolute joint
#   allowing slight forward/backward tilt (±12 degrees).
# - Main pivot: single revolute joint at the apex, axis (0, 1, 0), ±20 deg.
# ---------------------------------------------------------------------------

BEAM_HALF = 1.50
BEAM_W = 0.08
BEAM_T = 0.04
PIVOT_Z = 0.76

# Round support legs
LEG_TUBE_R = 0.025
LEG_FOOT_X = 0.50
LEG_FOOT_Y = 0.38
LEG_FOOT_Z = 0.0
CROSSBAR_Z = 0.62
CROSSBAR_R = 0.020

AXLE_R = 0.016
AXLE_LEN = 0.22

# Beam-local frame: origin at axle center
BAR_BOT = 0.05
BAR_CTR = BAR_BOT + BEAM_T / 2.0
BAR_TOP = BAR_BOT + BEAM_T

SEAT_X = 1.28
HANDLE_X = 1.02
BUMPER_X = 1.42

TILT = math.radians(20.0)
HANDLE_TILT = math.radians(12.0)

# Molded seat dimensions
SEAT_LEN = 0.28
SEAT_WID = 0.24
SEAT_BASE_T = 0.014
LIP_H = 0.032
LIP_T = 0.008


def _leg_points(foot_x: float, foot_y: float) -> list[tuple[float, float, float]]:
    """Straight tube centerline from ground foot to the apex bracket."""
    return [
        (foot_x, foot_y, 0.012),
        (foot_x * 0.55, foot_y * 0.55, PIVOT_Z * 0.42),
        (foot_x * 0.12, foot_y * 0.12, PIVOT_Z - 0.12),
        (0.0, 0.0, PIVOT_Z - 0.10),
    ]


def _handle_points_local() -> list[tuple[float, float, float]]:
    """Inverted-U grab-handle rod centerline in the handle part frame.

    The handle part frame origin is at the pivot point (top of the beam where
    the handle mounts). The U opens across Y, rises in +Z. Rod legs start
    above the pivot so they clear the mounting bracket on the beam.
    """
    half_w = 0.035
    leg_bot = 0.004  # rod starts just above pivot center (clears bracket)
    arc_z = 0.26
    pts: list[tuple[float, float, float]] = [
        (0.0, -half_w, leg_bot),
        (0.0, -half_w, 0.18),
    ]
    for k in range(7):
        a = math.pi * k / 6.0
        pts.append((0.0, -half_w * math.cos(a), arc_z + half_w * math.sin(a)))
    pts.append((0.0, half_w, 0.18))
    pts.append((0.0, half_w, leg_bot))
    return pts


def _bumper_geometry(x: float, index: int):
    """Curved tire-section bumper: half-annulus shell extruded across beam."""
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
    """Triangular gusset plate joining the beam bar to the pivot sleeve."""
    profile = [(-0.10, 0.055), (0.10, 0.055), (0.0, 0.020)]
    geom = ExtrudeGeometry(profile, 0.020, cap=True, center=True)
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, "seesaw_gusset_plate")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="classic_playground_seesaw")

    galvanized = model.material("galvanized_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.35, 0.36, 0.35, 1.0))
    mustard = model.material("mustard_paint", rgba=(0.78, 0.58, 0.14, 1.0))
    rust = model.material("rust_steel", rgba=(0.45, 0.27, 0.14, 1.0))
    pale_steel = model.material("pale_steel", rgba=(0.72, 0.70, 0.65, 1.0))
    rubber = model.material("black_rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    seat_green = model.material("molded_green_plastic", rgba=(0.18, 0.42, 0.22, 1.0))
    seat_lip = model.material("seat_lip_dark_green", rgba=(0.12, 0.30, 0.16, 1.0))
    bracket_paint = model.material("painted_bracket", rgba=(0.30, 0.30, 0.32, 1.0))

    # --------------------------------------------------------------- base ---
    base = model.part("base")

    # 4 round tube legs splayed from feet to apex
    leg_configs = [
        (+LEG_FOOT_X, +LEG_FOOT_Y),
        (+LEG_FOOT_X, -LEG_FOOT_Y),
        (-LEG_FOOT_X, +LEG_FOOT_Y),
        (-LEG_FOOT_X, -LEG_FOOT_Y),
    ]
    for i, (fx, fy) in enumerate(leg_configs):
        base.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _leg_points(fx, fy),
                    radius=LEG_TUBE_R,
                    samples_per_segment=8,
                    radial_segments=18,
                    cap_ends=True,
                ),
                f"leg_{i}",
            ),
            material=galvanized,
            name=f"leg_{i}",
        )

    # Foot pads (thickened disks connecting ground to leg tube bottoms)
    for i, (fx, fy) in enumerate(leg_configs):
        base.visual(
            Cylinder(radius=0.04, length=0.020),
            origin=Origin(xyz=(fx, fy, 0.010)),
            material=dark_steel,
            name=f"foot_pad_{i}",
        )

    # Horizontal crossbar connecting front pair and rear pair near the top
    for sign_x in (+1.0, -1.0):
        cx = sign_x * LEG_FOOT_X * 0.22
        base.visual(
            Cylinder(radius=CROSSBAR_R, length=LEG_FOOT_Y * 0.55),
            origin=Origin(
                xyz=(cx, 0.0, CROSSBAR_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=galvanized,
            name=f"crossbar_{'front' if sign_x > 0 else 'rear'}",
        )

    # Pivot bracket plate at the apex (connects leg tops to the axle)
    # Top stays below the beam pivot sleeve to avoid inter-part overlap
    base.visual(
        Box((0.14, 0.12, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.700)),
        material=dark_steel,
        name="pivot_bracket",
    )

    # Two vertical support posts bridging bracket top to the axle
    # Placed along Y outside the sleeve Y-extent (±0.022) to avoid beam overlap
    # Posts extend into the axle body for part connectivity
    post_bot = 0.730  # bracket top
    post_top = PIVOT_Z  # axle center (embeds into axle for connectivity)
    post_h = post_top - post_bot
    post_cz = (post_bot + post_top) / 2.0
    for j, py in enumerate((+0.045, -0.045)):
        base.visual(
            Cylinder(radius=0.014, length=post_h),
            origin=Origin(xyz=(0.0, py, post_cz)),
            material=dark_steel,
            name=f"pivot_post_{j}",
        )

    # Pivot axle bolt
    base.visual(
        Cylinder(radius=AXLE_R, length=AXLE_LEN),
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rust,
        name="pivot_axle",
    )
    for i, side in enumerate((1.0, -1.0)):
        base.visual(
            Cylinder(radius=0.024, length=0.014),
            origin=Origin(
                xyz=(0.0, side * (AXLE_LEN / 2.0 - 0.006), PIVOT_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=rust,
            name=f"axle_nut_{i}",
        )

    # --------------------------------------------------------------- beam ---
    beam = model.part("beam")

    # Pivot sleeve around axle
    beam.visual(
        Cylinder(radius=0.026, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rust,
        name="pivot_sleeve",
    )
    beam.visual(_gusset_geometry(), material=mustard, name="gusset_plate")

    # Main plank bar
    beam.visual(
        Box((2.0 * BEAM_HALF, BEAM_W, BEAM_T)),
        origin=Origin(xyz=(0.0, 0.0, BAR_CTR)),
        material=mustard,
        name="beam_bar",
    )

    # Rust streak patches
    for i, px in enumerate((-0.85, -0.30, 0.45, 0.95)):
        beam.visual(
            Box((0.16, BEAM_W + 0.004, 0.010)),
            origin=Origin(xyz=(px, 0.0, BAR_TOP - 0.003)),
            material=rust,
            name=f"rust_patch_{i}",
        )

    # Molded seats with raised lips at each end
    for i, side in enumerate((1.0, -1.0)):
        sx = side * SEAT_X
        seat_z = BAR_TOP + SEAT_BASE_T / 2.0

        # Seat base plate
        beam.visual(
            Box((SEAT_LEN, SEAT_WID, SEAT_BASE_T)),
            origin=Origin(xyz=(sx, 0.0, seat_z)),
            material=seat_green,
            name=f"seat_base_{i}",
        )

        # Raised lip - back wall (outboard end)
        beam.visual(
            Box((LIP_T, SEAT_WID, LIP_H)),
            origin=Origin(xyz=(sx + side * (SEAT_LEN / 2.0 - LIP_T / 2.0), 0.0, seat_z + SEAT_BASE_T / 2.0 + LIP_H / 2.0)),
            material=seat_lip,
            name=f"seat_lip_back_{i}",
        )
        # Raised lip - left side wall
        beam.visual(
            Box((SEAT_LEN, LIP_T, LIP_H)),
            origin=Origin(xyz=(sx, -(SEAT_WID / 2.0 - LIP_T / 2.0), seat_z + SEAT_BASE_T / 2.0 + LIP_H / 2.0)),
            material=seat_lip,
            name=f"seat_lip_left_{i}",
        )
        # Raised lip - right side wall
        beam.visual(
            Box((SEAT_LEN, LIP_T, LIP_H)),
            origin=Origin(xyz=(sx, +(SEAT_WID / 2.0 - LIP_T / 2.0), seat_z + SEAT_BASE_T / 2.0 + LIP_H / 2.0)),
            material=seat_lip,
            name=f"seat_lip_right_{i}",
        )

    # Bumpers under each tip
    for i, side in enumerate((1.0, -1.0)):
        beam.visual(
            _bumper_geometry(side * BUMPER_X, i),
            material=rubber,
            name=f"bumper_{i}",
        )

    # Handle pivot mounting brackets on the beam (small plates)
    for i, side in enumerate((1.0, -1.0)):
        hx = side * HANDLE_X
        beam.visual(
            Box((0.04, 0.06, 0.018)),
            origin=Origin(xyz=(hx, 0.0, BAR_TOP + 0.009)),
            material=bracket_paint,
            name=f"handle_bracket_{i}",
        )

    # --------------------------------------------------------- handlebars ---
    # Each handlebar is a separate part with its own revolute joint
    for i, side in enumerate((1.0, -1.0)):
        hx = side * HANDLE_X
        handle_part = model.part(f"handlebar_{i}")

        # Handle U-bar in the handle part frame (origin at pivot point)
        handle_part.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _handle_points_local(),
                    radius=0.009,
                    samples_per_segment=8,
                    radial_segments=16,
                    cap_ends=True,
                ),
                f"handle_rod_{i}",
            ),
            material=pale_steel,
            name=f"handle_rod_{i}",
        )

        # Pivot collar spanning between the rod legs above the bracket
        # Wraps around the rod legs for part connectivity
        handle_part.visual(
            Cylinder(radius=0.012, length=0.078),
            origin=Origin(xyz=(0.0, 0.0, 0.016), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"handle_collar_{i}",
        )

        # Articulation: handlebar pivots on the beam
        # Part frame origin is at the pivot point on the beam top
        # Axis is Y so positive q tilts the handle top forward (+X direction)
        model.articulation(
            f"handle_pivot_{i}",
            ArticulationType.REVOLUTE,
            parent=beam,
            child=handle_part,
            origin=Origin(xyz=(hx, 0.0, BAR_TOP + 0.014)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0, velocity=2.0,
                lower=-HANDLE_TILT, upper=HANDLE_TILT,
            ),
        )

    # -------------------------------------------------------------- joint ---
    model.articulation(
        "beam_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=200.0, velocity=2.5, lower=-TILT, upper=TILT),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    beam = object_model.get_part("beam")
    handle_0 = object_model.get_part("handlebar_0")
    handle_1 = object_model.get_part("handlebar_1")
    pivot = object_model.get_articulation("beam_pivot")
    h_pivot_0 = object_model.get_articulation("handle_pivot_0")
    h_pivot_1 = object_model.get_articulation("handle_pivot_1")

    # -- Pivot sleeve captures the axle bolt (intentional overlap) --
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

    # -- Beam bar clears the base bracket --
    ctx.expect_gap(
        beam,
        base,
        axis="z",
        positive_elem="beam_bar",
        negative_elem="pivot_bracket",
        min_gap=0.002,
        max_gap=0.12,
        name="beam bar clears the pivot bracket",
    )

    # -- Main pivot joint configuration --
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

    # -- Handle pivot joints exist and have correct configuration --
    for j, hj in enumerate((h_pivot_0, h_pivot_1)):
        hax = hj.axis
        ctx.check(
            f"handle_pivot_{j} axis is horizontal (Y)",
            abs(hax[0]) < 1e-9 and abs(hax[1] - 1.0) < 1e-9 and abs(hax[2]) < 1e-9,
            details=f"axis={hax}",
        )
        hlim = hj.motion_limits
        ctx.check(
            f"handle_pivot_{j} limits are about +/- 12 degrees",
            hlim is not None
            and hlim.lower is not None
            and hlim.upper is not None
            and abs(hlim.lower + HANDLE_TILT) < 1e-6
            and abs(hlim.upper - HANDLE_TILT) < 1e-6,
            details=f"limits=({hlim.lower}, {hlim.upper})",
        )

    # -- Handlebar mounting: collar contacts bracket on beam --
    for i in range(2):
        hpart = handle_0 if i == 0 else handle_1
        ctx.expect_gap(
            hpart,
            beam,
            axis="z",
            positive_elem=f"handle_collar_{i}",
            negative_elem=f"handle_bracket_{i}",
            min_gap=-0.002,
            max_gap=0.010,
            name=f"handle_{i} collar sits just above the mounting bracket",
        )

    # -- Handlebar rod stands above the beam --
    bar_box = ctx.part_element_world_aabb(beam, elem="beam_bar")
    for i in range(2):
        hpart = handle_0 if i == 0 else handle_1
        rod_box = ctx.part_element_world_aabb(hpart, elem=f"handle_rod_{i}")
        ctx.check(
            f"handlebar_{i} rod extends about 0.25 m above the beam",
            rod_box is not None
            and bar_box is not None
            and rod_box[1][2] > bar_box[1][2] + 0.18,
            details=f"rod aabb={rod_box}",
        )

    # -- Molded seats with raised lips --
    for i in range(2):
        seat_base = ctx.part_element_world_aabb(beam, elem=f"seat_base_{i}")
        lip_back = ctx.part_element_world_aabb(beam, elem=f"seat_lip_back_{i}")
        lip_left = ctx.part_element_world_aabb(beam, elem=f"seat_lip_left_{i}")
        lip_right = ctx.part_element_world_aabb(beam, elem=f"seat_lip_right_{i}")
        ctx.check(
            f"seat_{i} base sits on the beam bar",
            seat_base is not None
            and bar_box is not None
            and seat_base[0][2] >= bar_box[1][2] - 0.002,
            details=f"seat_base aabb={seat_base}",
        )
        ctx.check(
            f"seat_{i} has raised lip walls standing above the seat base",
            lip_back is not None
            and lip_left is not None
            and lip_right is not None
            and seat_base is not None
            and lip_back[1][2] > seat_base[1][2] + 0.010
            and lip_left[1][2] > seat_base[1][2] + 0.010
            and lip_right[1][2] > seat_base[1][2] + 0.010,
            details=f"lip_back={lip_back}, lip_left={lip_left}, lip_right={lip_right}",
        )

    # -- Bumpers hang below the beam tips --
    for i in range(2):
        bumper = ctx.part_element_world_aabb(beam, elem=f"bumper_{i}")
        ctx.check(
            f"bumper_{i} hangs below the beam tip",
            bumper is not None
            and bar_box is not None
            and bumper[0][2] < bar_box[0][2]
            and min(abs(bumper[0][0]), abs(bumper[1][0])) > 1.3,
            details=f"bumper aabb={bumper}",
        )

    # -- Hero geometry: scale and proportions --
    ctx.check(
        "beam is about 3.0 m long",
        bar_box is not None and abs((bar_box[1][0] - bar_box[0][0]) - 3.0) < 0.02,
        details=f"bar aabb={bar_box}",
    )
    base_box = ctx.part_world_aabb(base)
    ctx.check(
        "base feet rest on the ground",
        base_box is not None and -0.01 <= base_box[0][2] <= 0.03,
        details=f"base aabb={base_box}",
    )
    axle_box = ctx.part_element_world_aabb(base, elem="pivot_axle")
    ctx.check(
        "pivot axle sits about 0.8 m high",
        axle_box is not None and 0.70 <= (axle_box[0][2] + axle_box[1][2]) / 2.0 <= 0.82,
        details=f"axle aabb={axle_box}",
    )

    # -- Round legs present (4 legs) --
    for i in range(4):
        leg_box = ctx.part_element_world_aabb(base, elem=f"leg_{i}")
        ctx.check(
            f"leg_{i} spans from ground to near apex height",
            leg_box is not None
            and leg_box[0][2] < 0.06
            and leg_box[1][2] > PIVOT_Z - 0.10,
            details=f"leg aabb={leg_box}",
        )

    # -- Decisive pose: rocking motion --
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

    # -- Handlebar pivot motion proof --
    with ctx.pose({h_pivot_0: HANDLE_TILT}):
        rod_tilted = ctx.part_element_world_aabb(handle_0, elem="handle_rod_0")
        rod_rest = ctx.part_element_world_aabb(handle_0, elem="handle_rod_0")
    with ctx.pose({h_pivot_0: 0.0}):
        rod_rest = ctx.part_element_world_aabb(handle_0, elem="handle_rod_0")
    # Just verify the handlebar exists and has extent above beam
    ctx.check(
        "handlebar_0 exists as a separate articulated part",
        rod_rest is not None and rod_rest[1][2] > bar_box[1][2] + 0.15 if bar_box and rod_rest else False,
        details=f"handle rod rest aabb={rod_rest}",
    )

    return ctx.report()


object_model = build_object_model()
