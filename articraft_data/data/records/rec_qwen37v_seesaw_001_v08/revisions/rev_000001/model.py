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
# Compact backyard seesaw with triangular supports
#
# World frame: beam runs along X, pivot axis along Y, Z up.
# - Two tubular A-frame supports (side by side in Y) form triangular bases;
#   each has two angled legs meeting at the apex plus a bottom cross-bar.
# - A horizontal cross-tube connects the two A-frame apexes; the pivot axle
#   sits on top.
# - Rubber ground pads sit under each of the four feet.
# - The rocking beam is a 2.0 m painted steel bar (60 x 35 mm) with a pivot
#   sleeve, gusset plate, wooden seat, grab handle, textured footrest, and
#   rubber bumper at each end.
# - A hinged backrest board behind each seat tilts on a small revolute joint.
# - Single revolute beam pivot, axis (0, 1, 0), +/- 20 degrees.
#   Positive q lowers the +X end (right-hand rule about +Y).
# ---------------------------------------------------------------------------

BEAM_HALF = 1.00  # 2.0 m beam
BEAM_W = 0.060
BEAM_T = 0.035
PIVOT_Z = 0.55  # axle height

# A-frame dimensions
FRAME_Y = 0.12  # half-spacing between the two A-frames
FOOT_X = 0.22  # half-spread of A-frame feet
FOOT_Z = 0.018  # foot center height (on rubber pad top)
TUBE_R = 0.018  # ~36 mm diameter tube

AXLE_R = 0.014
AXLE_LEN = 0.30  # spans both A-frames + margins

# Beam-local frame: origin at the axle center.
BAR_BOT = 0.038
BAR_CTR = BAR_BOT + BEAM_T / 2.0
BAR_TOP = BAR_BOT + BEAM_T

SEAT_X = 0.82
HANDLE_X = 0.62
BUMPER_X = 0.94
TILT = math.radians(20.0)

# Backrest
BACKREST_REAR = 0.10  # how far behind seat center the hinge sits
BACKREST_W = 0.18
BACKREST_H = 0.14
BACKREST_T = 0.012
BACKREST_TILT_MAX = math.radians(22.0)

# Footrest
FOOTREST_X = 0.58  # inboard of seat center
FOOTREST_D = 0.10  # depth along beam
FOOTREST_W = 0.08  # width across beam
FOOTREST_T = 0.006  # base plate thickness
RIDGE_H = 0.004  # ridge height
RIDGE_COUNT = 5

# Rubber pad
PAD_W = 0.10
PAD_D = 0.10
PAD_T = 0.015


def _leg_points(foot_x: float, frame_y: float) -> list[tuple[float, float, float]]:
    """Straight tube leg from foot to apex."""
    return [
        (foot_x, frame_y, FOOT_Z),
        (0.0, frame_y, PIVOT_Z),
    ]


def _crossbar_points(frame_y: float) -> list[tuple[float, float, float]]:
    """Horizontal cross-bar connecting the two feet of one A-frame."""
    return [
        (-FOOT_X, frame_y, FOOT_Z),
        (FOOT_X, frame_y, FOOT_Z),
    ]


def _apex_crossbar_points() -> list[tuple[float, float, float]]:
    """Horizontal tube connecting the two A-frame apexes."""
    return [
        (0.0, -FRAME_Y, PIVOT_Z),
        (0.0, FRAME_Y, PIVOT_Z),
    ]


def _handle_points(x: float) -> list[tuple[float, float, float]]:
    """Inverted-U grab-handle rod centerline, plane across the beam (YZ)."""
    half_w = 0.028
    leg_bot = BAR_TOP - 0.008
    arc_z = 0.240
    pts: list[tuple[float, float, float]] = [
        (x, -half_w, leg_bot),
        (x, -half_w, 0.170),
    ]
    for k in range(7):
        a = math.pi * k / 6.0
        pts.append((x, -half_w * math.cos(a), arc_z + half_w * math.sin(a)))
    pts.append((x, half_w, 0.170))
    pts.append((x, half_w, leg_bot))
    return pts


def _bumper_geometry(x: float, index: int):
    """Curved tire-section bumper: half-annulus shell extruded across the beam."""
    r_out = 0.050
    r_in = 0.038
    profile: list[tuple[float, float]] = []
    n = 20
    for k in range(n + 1):
        a = math.pi + math.pi * k / n
        profile.append((r_out * math.cos(a), r_out * math.sin(a)))
    for k in range(n + 1):
        a = 2.0 * math.pi - math.pi * k / n
        profile.append((r_in * math.cos(a), r_in * math.sin(a)))
    geom = ExtrudeGeometry(profile, 0.08, cap=True, center=True)
    geom.rotate_x(math.pi / 2.0)
    geom.translate(x, 0.0, BAR_BOT + 0.002)
    return mesh_from_geometry(geom, f"seesaw_bumper_{index}")


def _gusset_geometry():
    """Triangular gusset plate joining the beam bar to the pivot sleeve."""
    profile = [(-0.08, 0.042), (0.08, 0.042), (0.0, 0.015)]
    geom = ExtrudeGeometry(profile, 0.016, cap=True, center=True)
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, "seesaw_gusset_plate")


def _apex_plate_geometry():
    """Flat plate at the A-frame apex that cradles the pivot axle."""
    profile = [(-0.06, -0.035), (0.06, -0.035), (0.06, 0.035), (-0.06, 0.035)]
    geom = ExtrudeGeometry(profile, 0.008, cap=True, center=True)
    # Rotate so the plate lies flat in XY at the apex height
    return mesh_from_geometry(geom, "apex_plate")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="backyard_seesaw_triangular")

    # Materials
    painted_green = model.material("painted_green_steel", rgba=(0.18, 0.42, 0.22, 1.0))
    bright_red = model.material("bright_red_paint", rgba=(0.78, 0.15, 0.10, 1.0))
    pale_steel = model.material("pale_steel", rgba=(0.68, 0.66, 0.60, 1.0))
    wood = model.material("smooth_wood", rgba=(0.62, 0.46, 0.28, 1.0))
    rubber = model.material("black_rubber", rgba=(0.06, 0.06, 0.06, 1.0))
    dark_gray = model.material("dark_grip_rubber", rgba=(0.14, 0.14, 0.14, 1.0))
    rust = model.material("rust_accent", rgba=(0.45, 0.28, 0.14, 1.0))

    # --------------------------------------------------------------- base ---
    base = model.part("triangular_base")

    # A-frame legs (4 total: 2 per side)
    leg_idx = 0
    for side in (1.0, -1.0):
        for foot_sign in (1.0, -1.0):
            base.visual(
                mesh_from_geometry(
                    tube_from_spline_points(
                        _leg_points(foot_sign * FOOT_X, side * FRAME_Y),
                        radius=TUBE_R,
                        samples_per_segment=4,
                        radial_segments=16,
                        cap_ends=True,
                    ),
                    f"aframe_leg_{leg_idx}",
                ),
                material=painted_green,
                name=f"leg_{leg_idx}",
            )
            leg_idx += 1

    # Bottom cross-bars (one per A-frame)
    for i, side in enumerate((1.0, -1.0)):
        base.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _crossbar_points(side * FRAME_Y),
                    radius=TUBE_R,
                    samples_per_segment=4,
                    radial_segments=16,
                    cap_ends=True,
                ),
                f"aframe_crossbar_{i}",
            ),
            material=painted_green,
            name=f"crossbar_{i}",
        )

    # Apex cross-tube connecting the two A-frame tops
    base.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                _apex_crossbar_points(),
                radius=TUBE_R,
                samples_per_segment=4,
                radial_segments=16,
                cap_ends=True,
            ),
            "apex_crossbar_mesh",
        ),
        material=painted_green,
        name="apex_crossbar",
    )

    # Apex plates (flat mounting plates at each A-frame top)
    for i, side in enumerate((1.0, -1.0)):
        base.visual(
            Box((0.12, 0.07, 0.008)),
            origin=Origin(xyz=(0.0, side * FRAME_Y, PIVOT_Z - 0.004)),
            material=painted_green,
            name=f"apex_plate_{i}",
        )

    # Pivot axle bolt through apex plates
    base.visual(
        Cylinder(radius=AXLE_R, length=AXLE_LEN),
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z + 0.006), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pale_steel,
        name="pivot_axle",
    )

    # Axle nuts
    for i, side in enumerate((1.0, -1.0)):
        base.visual(
            Cylinder(radius=0.020, length=0.012),
            origin=Origin(
                xyz=(0.0, side * (AXLE_LEN / 2.0 - 0.005), PIVOT_Z + 0.006),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=pale_steel,
            name=f"axle_nut_{i}",
        )

    # Rubber ground pads under each foot (4 pads)
    pad_idx = 0
    for side in (1.0, -1.0):
        for foot_sign in (1.0, -1.0):
            base.visual(
                Box((PAD_D, PAD_W, PAD_T)),
                origin=Origin(
                    xyz=(foot_sign * FOOT_X, side * FRAME_Y, PAD_T / 2.0)
                ),
                material=rubber,
                name=f"ground_pad_{pad_idx}",
            )
            pad_idx += 1

    # --------------------------------------------------------------- beam ---
    beam = model.part("beam")

    beam.visual(
        Cylinder(radius=0.022, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pale_steel,
        name="pivot_sleeve",
    )
    beam.visual(_gusset_geometry(), material=bright_red, name="gusset_plate")

    beam.visual(
        Box((2.0 * BEAM_HALF, BEAM_W, BEAM_T)),
        origin=Origin(xyz=(0.0, 0.0, BAR_CTR)),
        material=bright_red,
        name="beam_bar",
    )

    # End fittings: seat, handle, bumper, footrest per end
    for i, side in enumerate((1.0, -1.0)):
        # Wooden seat plate
        beam.visual(
            Box((0.24, 0.20, 0.020)),
            origin=Origin(xyz=(side * SEAT_X, 0.0, BAR_TOP + 0.007)),
            material=wood,
            name=f"seat_{i}",
        )

        # Grab handle
        beam.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _handle_points(side * HANDLE_X),
                    radius=0.008,
                    samples_per_segment=8,
                    radial_segments=14,
                    cap_ends=True,
                ),
                f"seesaw_handle_{i}",
            ),
            material=pale_steel,
            name=f"handle_{i}",
        )

        # Rubber bumper under tip
        beam.visual(
            _bumper_geometry(side * BUMPER_X, i),
            material=rubber,
            name=f"bumper_{i}",
        )

        # Textured footrest (base plate + ridges)
        fx = side * FOOTREST_X
        beam.visual(
            Box((FOOTREST_D, FOOTREST_W, FOOTREST_T)),
            origin=Origin(xyz=(fx, 0.0, BAR_TOP + FOOTREST_T / 2.0)),
            material=dark_gray,
            name=f"footrest_base_{i}",
        )
        # Raised grip ridges across the footrest
        ridge_spacing = FOOTREST_D / (RIDGE_COUNT + 1)
        for r in range(RIDGE_COUNT):
            rx = fx + side * (-FOOTREST_D / 2.0 + ridge_spacing * (r + 1))
            beam.visual(
                Box((0.005, FOOTREST_W - 0.012, RIDGE_H)),
                origin=Origin(
                    xyz=(rx, 0.0, BAR_TOP + FOOTREST_T + RIDGE_H / 2.0)
                ),
                material=dark_gray,
                name=f"footrest_ridge_{i}_{r}",
            )

    # -------------------------------------------------------- backrests ---
    # Backrest 0 (+X end): hinge at rear of seat, tilts back toward +X
    backrest_0 = model.part("backrest_0")
    backrest_0.visual(
        Box((BACKREST_T, BACKREST_W, BACKREST_H)),
        origin=Origin(xyz=(0.0, 0.0, BACKREST_H / 2.0)),
        material=wood,
        name="backrest_board_0",
    )
    # Small hinge pin visual at the bottom
    backrest_0.visual(
        Cylinder(radius=0.005, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pale_steel,
        name="backrest_hinge_pin_0",
    )

    # Backrest 1 (-X end): hinge at rear of seat, tilts back toward -X
    backrest_1 = model.part("backrest_1")
    backrest_1.visual(
        Box((BACKREST_T, BACKREST_W, BACKREST_H)),
        origin=Origin(xyz=(0.0, 0.0, BACKREST_H / 2.0)),
        material=wood,
        name="backrest_board_1",
    )
    backrest_1.visual(
        Cylinder(radius=0.005, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pale_steel,
        name="backrest_hinge_pin_1",
    )

    # ----------------------------------------------------------- joints ---
    # Main beam pivot
    model.articulation(
        "beam_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z + 0.006)),
        axis=(0.0, 1.0, 0.0),  # positive q lowers the +X end
        motion_limits=MotionLimits(
            effort=120.0, velocity=2.5, lower=-TILT, upper=TILT
        ),
    )

    # Backrest 0 tilt (at +X end, behind seat)
    # Hinge is at rear of seat; positive q tilts top toward +X (away from center)
    model.articulation(
        "backrest_tilt_0",
        ArticulationType.REVOLUTE,
        parent=beam,
        child=backrest_0,
        origin=Origin(
            xyz=(SEAT_X + BACKREST_REAR, 0.0, BAR_TOP + 0.018)
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=0.0, upper=BACKREST_TILT_MAX
        ),
    )

    # Backrest 1 tilt (at -X end, behind seat)
    # axis = (0, -1, 0) so positive q tilts top toward -X (away from center)
    model.articulation(
        "backrest_tilt_1",
        ArticulationType.REVOLUTE,
        parent=beam,
        child=backrest_1,
        origin=Origin(
            xyz=(-(SEAT_X + BACKREST_REAR), 0.0, BAR_TOP + 0.018)
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=0.0, upper=BACKREST_TILT_MAX
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("triangular_base")
    beam = object_model.get_part("beam")
    backrest_0 = object_model.get_part("backrest_0")
    backrest_1 = object_model.get_part("backrest_1")
    pivot = object_model.get_articulation("beam_pivot")
    tilt_0 = object_model.get_articulation("backrest_tilt_0")
    tilt_1 = object_model.get_articulation("backrest_tilt_1")

    # --- Pivot sleeve captures axle bolt ---
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

    # --- Pivot sleeve nests against the apex crossbar (structural seating) ---
    ctx.allow_overlap(
        beam,
        base,
        elem_a="pivot_sleeve",
        elem_b="apex_crossbar",
        reason="Pivot sleeve sits on the apex crossbar as part of the pivot bearing assembly.",
    )
    ctx.expect_contact(
        beam,
        base,
        elem_a="pivot_sleeve",
        elem_b="apex_crossbar",
        name="pivot sleeve is seated against the apex crossbar",
    )

    # --- Beam bar clears the A-frame apex ---
    ctx.expect_gap(
        beam,
        base,
        axis="z",
        positive_elem="beam_bar",
        negative_elem="apex_crossbar",
        min_gap=0.002,
        max_gap=0.06,
        name="beam bar clears the apex crossbar",
    )

    # --- Pivot axis and limits ---
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

    # --- Backrest joints exist with correct type and limits ---
    for tilt_name, tilt_joint in [("backrest_tilt_0", tilt_0), ("backrest_tilt_1", tilt_1)]:
        ctx.check(
            f"{tilt_name} is revolute",
            tilt_joint.articulation_type == ArticulationType.REVOLUTE,
            details=f"type={tilt_joint.articulation_type}",
        )
        tl = tilt_joint.motion_limits
        ctx.check(
            f"{tilt_name} has tilt limits from 0 to about 22 degrees",
            tl is not None
            and tl.lower is not None
            and tl.upper is not None
            and abs(tl.lower) < 1e-6
            and abs(tl.upper - BACKREST_TILT_MAX) < 1e-6,
            details=f"limits=({tl.lower}, {tl.upper})",
        )

    # --- Triangular base: A-frame legs reach ground, apex at pivot height ---
    base_box = ctx.part_world_aabb(base)
    axle_box = ctx.part_element_world_aabb(base, elem="pivot_axle")
    ctx.check(
        "triangular base feet rest near the ground",
        base_box is not None and -0.01 <= base_box[0][2] <= 0.025,
        details=f"base aabb={base_box}",
    )
    ctx.check(
        "pivot axle sits about 0.55 m high",
        axle_box is not None and 0.48 <= (axle_box[0][2] + axle_box[1][2]) / 2.0 <= 0.62,
        details=f"axle aabb={axle_box}",
    )

    # --- Beam is about 2.0 m long ---
    bar_box = ctx.part_element_world_aabb(beam, elem="beam_bar")
    ctx.check(
        "beam is about 2.0 m long",
        bar_box is not None and abs((bar_box[1][0] - bar_box[0][0]) - 2.0) < 0.02,
        details=f"bar aabb={bar_box}",
    )

    # --- Rubber ground pads exist under feet ---
    for p in range(4):
        pad_box = ctx.part_element_world_aabb(base, elem=f"ground_pad_{p}")
        ctx.check(
            f"ground_pad_{p} is at ground level",
            pad_box is not None and pad_box[0][2] < 0.020 and pad_box[1][2] < 0.040,
            details=f"pad aabb={pad_box}",
        )

    # --- Textured footrests near each seat ---
    for i in range(2):
        fb = ctx.part_element_world_aabb(beam, elem=f"footrest_base_{i}")
        ctx.check(
            f"footrest_base_{i} is on top of the beam bar",
            fb is not None
            and bar_box is not None
            and fb[0][2] > bar_box[1][2] - 0.005
            and fb[1][2] > bar_box[1][2],
            details=f"footrest aabb={fb}",
        )

    # --- Backrest boards stand above the beam ---
    for i, br in enumerate([backrest_0, backrest_1]):
        br_box = ctx.part_world_aabb(br)
        ctx.check(
            f"backrest_{i} extends above the beam",
            br_box is not None
            and bar_box is not None
            and br_box[1][2] > bar_box[1][2] + 0.08,
            details=f"backrest aabb={br_box}",
        )

    # --- Seats on beam ---
    for i in range(2):
        seat = ctx.part_element_world_aabb(beam, elem=f"seat_{i}")
        ctx.check(
            f"seat_{i} is seated on the beam bar top",
            seat is not None
            and bar_box is not None
            and bar_box[0][2] < seat[0][2] < bar_box[1][2]
            and seat[1][2] > bar_box[1][2],
            details=f"seat aabb={seat}",
        )

    # --- Decisive pose: rocking alternately lowers each end ---
    rest_b0 = ctx.part_element_world_aabb(beam, elem="bumper_0")
    with ctx.pose({pivot: TILT}):
        down_b0 = ctx.part_element_world_aabb(beam, elem="bumper_0")
        up_b1 = ctx.part_element_world_aabb(beam, elem="bumper_1")
        ctx.check(
            "positive rock lowers the +X end near the ground",
            rest_b0 is not None
            and down_b0 is not None
            and down_b0[0][2] < rest_b0[0][2] - 0.20
            and down_b0[0][2] > 0.0,
            details=f"rest={rest_b0}, tilted={down_b0}",
        )
        ctx.check(
            "positive rock raises the -X end",
            up_b1 is not None and up_b1[0][2] > 0.70,
            details=f"raised bumper aabb={up_b1}",
        )

    # --- Decisive pose: backrest tilt ---
    with ctx.pose({tilt_0: BACKREST_TILT_MAX}):
        br0_tilted = ctx.part_world_aabb(backrest_0)
        br0_rest_top = None
    with ctx.pose({tilt_0: 0.0}):
        br0_rest = ctx.part_world_aabb(backrest_0)

    if br0_tilted is not None and br0_rest is not None:
        ctx.check(
            "backrest_0 top moves outward when tilted",
            br0_tilted[1][0] > br0_rest[1][0] + 0.005,
            details=f"rest_top={br0_rest[1][0]}, tilted_top={br0_tilted[1][0]}",
        )

    with ctx.pose({tilt_1: BACKREST_TILT_MAX}):
        br1_tilted = ctx.part_world_aabb(backrest_1)
    with ctx.pose({tilt_1: 0.0}):
        br1_rest = ctx.part_world_aabb(backrest_1)

    if br1_tilted is not None and br1_rest is not None:
        ctx.check(
            "backrest_1 top moves outward when tilted",
            br1_tilted[0][0] < br1_rest[0][0] - 0.005,
            details=f"rest_bot={br1_rest[0][0]}, tilted_bot={br1_tilted[0][0]}",
        )

    return ctx.report()


object_model = build_object_model()
