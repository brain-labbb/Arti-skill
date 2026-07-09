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
# Four-seat cross playground seesaw
#
# World frame: two perpendicular beams cross at the pivot forming an X shape.
# Beam A runs along X, beam B along Y. Z is up.
# - Two bent galvanized-steel tube arches (~50 mm dia) form an A-saddle base;
#   rubber ground pads sit under each of the four feet.
# - The cross beam is two 3.0 m mustard-yellow steel bars (80 x 40 mm) rigidly
#   joined at the center, with a pivot sleeve + gusset hub.
# - Each of the four beam ends carries a wooden seat plate, an inverted-U grab
#   handle, a safety bump stop underneath, and a tilting backrest on a small
#   revolute joint.
# - Main articulation: revolute pivot at the apex (axis Y, +/- 20 degrees).
# - Secondary articulations: four backrest tilt joints (revolute, 0-30 deg).
# ---------------------------------------------------------------------------

BEAM_HALF = 1.50  # 3.0 m beams
BEAM_W = 0.08
BEAM_T = 0.04
PIVOT_Z = 0.76

ARCH_FOOT_X = 0.66
ARCH_FOOT_Y = 0.34
ARCH_APEX_Y = 0.05
ARCH_FOOT_Z = 0.028
TUBE_R = 0.025

AXLE_R = 0.016
AXLE_LEN = 0.22

BAR_BOT = 0.05
BAR_CTR = BAR_BOT + BEAM_T / 2.0
BAR_TOP = BAR_BOT + BEAM_T

SEAT_D = 1.30  # seat distance from center along each beam
HANDLE_D = 1.04
BUMP_D = 1.42
TILT = math.radians(20.0)
BACKREST_TILT_MAX = math.radians(30.0)

SEAT_PLATE_T = 0.022
SEAT_TOP_Z = BAR_TOP + SEAT_PLATE_T / 2.0 + 0.008  # seat plate center Z
BACKREST_Z = BAR_TOP + SEAT_PLATE_T + 0.008  # bottom of backrest (top of seat)

GROUND_PAD_R = 0.065
GROUND_PAD_T = 0.014


def _arch_points(side: float) -> list[tuple[float, float, float]]:
    """Centerline of one bent-tube arch."""
    pts: list[tuple[float, float, float]] = []
    rise = PIVOT_Z - ARCH_FOOT_Z
    for i in range(11):
        t = -1.0 + 0.2 * i
        s = 1.0 - t * t
        x = ARCH_FOOT_X * t
        z = ARCH_FOOT_Z + rise * s
        y = side * ARCH_FOOT_Y + (-side * ARCH_APEX_Y - side * ARCH_FOOT_Y) * s
        pts.append((x, y, z))
    return pts


def _handle_points(axial_pos: float, beam_axis: str) -> list[tuple[float, float, float]]:
    """Inverted-U grab-handle rod centerline across the beam."""
    half_w = 0.035
    leg_bot = BAR_TOP - 0.010
    arc_z = 0.275
    if beam_axis == "x":
        # Handle in YZ plane at given X
        pts: list[tuple[float, float, float]] = [
            (axial_pos, -half_w, leg_bot),
            (axial_pos, -half_w, 0.190),
        ]
        for k in range(7):
            a = math.pi * k / 6.0
            pts.append((axial_pos, -half_w * math.cos(a), arc_z + half_w * math.sin(a)))
        pts.append((axial_pos, half_w, 0.190))
        pts.append((axial_pos, half_w, leg_bot))
    else:
        # Handle in XZ plane at given Y
        pts = [
            (-half_w, axial_pos, leg_bot),
            (-half_w, axial_pos, 0.190),
        ]
        for k in range(7):
            a = math.pi * k / 6.0
            pts.append((-half_w * math.cos(a), axial_pos, arc_z + half_w * math.sin(a)))
        pts.append((half_w, axial_pos, 0.190))
        pts.append((half_w, axial_pos, leg_bot))
    return pts


def _bump_stop_mesh(index: int, pos: tuple[float, float, float]):
    """Safety bump stop: rubber block under a beam end."""
    geom = ExtrudeGeometry(
        [(-0.035, -0.030), (0.035, -0.030), (0.035, 0.030), (-0.035, 0.030)],
        0.055,
        cap=True,
        center=True,
    )
    # Profile in XY, extruded along Z
    geom.translate(pos[0], pos[1], BAR_BOT - 0.025)
    return mesh_from_geometry(geom, f"bump_stop_mesh_{index}")


def _gusset_hub_geometry():
    """Central hub plate tying the two beams together at the crossing."""
    profile = [(-0.07, -0.07), (0.07, -0.07), (0.07, 0.07), (-0.07, 0.07)]
    geom = ExtrudeGeometry(profile, 0.012, cap=True, center=True)
    geom.translate(0.0, 0.0, BAR_TOP + 0.006)
    return mesh_from_geometry(geom, "hub_plate")


def _gusset_triangle():
    """Triangular gusset plate under the hub connecting beams to pivot sleeve."""
    profile = [(-0.10, 0.055), (0.10, 0.055), (0.0, 0.020)]
    geom = ExtrudeGeometry(profile, 0.020, cap=True, center=True)
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, "gusset_plate")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cross_playground_seesaw")

    galvanized = model.material("weathered_galvanized", rgba=(0.55, 0.58, 0.56, 1.0))
    rust = model.material("rust_steel", rgba=(0.42, 0.25, 0.13, 1.0))
    mustard = model.material("rusty_mustard_paint", rgba=(0.74, 0.53, 0.12, 1.0))
    pale_steel = model.material("pale_weathered_steel", rgba=(0.70, 0.66, 0.58, 1.0))
    wood = model.material("worn_wood", rgba=(0.60, 0.45, 0.28, 1.0))
    rubber = model.material("black_rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    backrest_mat = model.material("faded_green_paint", rgba=(0.22, 0.38, 0.24, 1.0))

    # --------------------------------------------------------------- base ---
    base = model.part("arched_base")
    for i, side in enumerate((1.0, -1.0)):
        base.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _arch_points(side),
                    radius=TUBE_R,
                    samples_per_segment=8,
                    radial_segments=18,
                    cap_ends=True,
                ),
                f"arch_{i}",
            ),
            material=galvanized,
            name=f"arch_{i}",
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

    # Rubber ground pads under each arch foot (4 pads total)
    foot_positions = []
    for side in (1.0, -1.0):
        for sx in (-1.0, 1.0):
            foot_positions.append((sx * ARCH_FOOT_X, side * ARCH_FOOT_Y))
    for i, (fx, fy) in enumerate(foot_positions):
        base.visual(
            Cylinder(radius=GROUND_PAD_R, length=GROUND_PAD_T),
            origin=Origin(xyz=(fx, fy, GROUND_PAD_T / 2.0)),
            material=rubber,
            name=f"ground_pad_{i}",
        )

    # --------------------------------------------------------- cross beam ---
    beam = model.part("cross_beam")

    # Pivot sleeve
    beam.visual(
        Cylinder(radius=0.026, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rust,
        name="pivot_sleeve",
    )
    beam.visual(_gusset_triangle(), material=mustard, name="gusset_plate")
    beam.visual(_gusset_hub_geometry(), material=mustard, name="hub_plate")

    # X-beam bar (along X)
    beam.visual(
        Box((2.0 * BEAM_HALF, BEAM_W, BEAM_T)),
        origin=Origin(xyz=(0.0, 0.0, BAR_CTR)),
        material=mustard,
        name="beam_bar_x",
    )
    # Y-beam bar (along Y)
    beam.visual(
        Box((BEAM_W, 2.0 * BEAM_HALF, BEAM_T)),
        origin=Origin(xyz=(0.0, 0.0, BAR_CTR)),
        material=mustard,
        name="beam_bar_y",
    )

    # Rust streak patches on both beams
    for i, px in enumerate((-0.85, -0.30, 0.45, 0.95)):
        beam.visual(
            Box((0.14, BEAM_W + 0.004, 0.010)),
            origin=Origin(xyz=(px, 0.0, BAR_TOP - 0.003)),
            material=rust,
            name=f"rust_patch_x_{i}",
        )
    for i, py in enumerate((-0.85, -0.30, 0.45, 0.95)):
        beam.visual(
            Box((BEAM_W + 0.004, 0.14, 0.010)),
            origin=Origin(xyz=(0.0, py, BAR_TOP - 0.003)),
            material=rust,
            name=f"rust_patch_y_{i}",
        )

    # Seat positions: 0=+X, 1=-X, 2=+Y, 3=-Y
    seat_configs = [
        (SEAT_D, 0.0, "x", 1.0),    # +X end
        (-SEAT_D, 0.0, "x", -1.0),  # -X end
        (0.0, SEAT_D, "y", 1.0),    # +Y end
        (0.0, -SEAT_D, "y", -1.0),  # -Y end
    ]

    for i, (sx, sy, beam_axis, side) in enumerate(seat_configs):
        # Seat plate
        if beam_axis == "x":
            seat_size = (0.28, 0.24, SEAT_PLATE_T)
        else:
            seat_size = (0.24, 0.28, SEAT_PLATE_T)
        beam.visual(
            Box(seat_size),
            origin=Origin(xyz=(sx, sy, BAR_TOP + 0.008)),
            material=wood,
            name=f"seat_{i}",
        )

        # Handle
        handle_pos = (HANDLE_D * (1.0 if beam_axis == "x" else 0.0) * side,
                      HANDLE_D * (1.0 if beam_axis == "y" else 0.0) * side)
        beam.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _handle_points(handle_pos[0] if beam_axis == "x" else handle_pos[1], beam_axis),
                    radius=0.009,
                    samples_per_segment=8,
                    radial_segments=16,
                    cap_ends=True,
                ),
                f"handle_mesh_{i}",
            ),
            material=pale_steel,
            name=f"handle_{i}",
        )

        # Safety bump stop
        bump_pos = (BUMP_D * (1.0 if beam_axis == "x" else 0.0) * side,
                    BUMP_D * (1.0 if beam_axis == "y" else 0.0) * side)
        beam.visual(
            _bump_stop_mesh(i, bump_pos),
            material=rubber,
            name=f"bump_stop_{i}",
        )

    # -------------------------------------------------------- backrests ---
    # Each backrest is a separate part with a revolute tilt joint.
    # Hinge brackets are welded to the beam; backrest hinge pin sits in bracket.
    # Backrest 0: +X seat, tilts around Y axis
    # Backrest 1: -X seat, tilts around -Y axis
    # Backrest 2: +Y seat, tilts around -X axis
    # Backrest 3: -Y seat, tilts around +X axis

    BRACKET_H = 0.030  # bracket height above beam bar top
    HINGE_R = 0.008    # hinge pin radius
    HINGE_LEN = 0.048  # hinge pin length

    # Joint origin Z = beam top + bracket height (hinge axis height)
    HINGE_Z = BAR_TOP + BRACKET_H  # 0.828

    backrest_configs = [
        # (joint_xyz, axis, plate_size, plate_offset, hinge_rpy, bracket_pos)
        # Backrest 0: +X end, axis Y
        (
            (SEAT_D, 0.0, HINGE_Z),
            (0.0, 1.0, 0.0),
            (0.015, 0.20, 0.22),
            (0.010, 0.0, 0.110),
            (math.pi / 2.0, 0.0, 0.0),
            (SEAT_D, 0.0, BAR_TOP + BRACKET_H / 2.0),
        ),
        # Backrest 1: -X end, axis -Y
        (
            (-SEAT_D, 0.0, HINGE_Z),
            (0.0, -1.0, 0.0),
            (0.015, 0.20, 0.22),
            (-0.010, 0.0, 0.110),
            (math.pi / 2.0, 0.0, 0.0),
            (-SEAT_D, 0.0, BAR_TOP + BRACKET_H / 2.0),
        ),
        # Backrest 2: +Y end, axis -X
        (
            (0.0, SEAT_D, HINGE_Z),
            (-1.0, 0.0, 0.0),
            (0.20, 0.015, 0.22),
            (0.0, 0.010, 0.110),
            (0.0, math.pi / 2.0, 0.0),
            (0.0, SEAT_D, BAR_TOP + BRACKET_H / 2.0),
        ),
        # Backrest 3: -Y end, axis +X
        (
            (0.0, -SEAT_D, HINGE_Z),
            (1.0, 0.0, 0.0),
            (0.20, 0.015, 0.22),
            (0.0, -0.010, 0.110),
            (0.0, math.pi / 2.0, 0.0),
            (0.0, -SEAT_D, BAR_TOP + BRACKET_H / 2.0),
        ),
    ]

    backrest_parts = []
    backrest_joints = []

    for i, (joint_xyz, axis, plate_size, plate_offset, hinge_rpy, bracket_pos) in enumerate(backrest_configs):
        # Hinge bracket welded to beam (part of cross_beam)
        beam.visual(
            Box((0.036, 0.036, BRACKET_H)),
            origin=Origin(xyz=bracket_pos),
            material=pale_steel,
            name=f"hinge_bracket_{i}",
        )

        bp = model.part(f"backrest_{i}")
        # Backrest plate
        bp.visual(
            Box(plate_size),
            origin=Origin(xyz=plate_offset),
            material=backrest_mat,
            name=f"backrest_plate_{i}",
        )
        # Hinge pin sitting in the bracket
        bp.visual(
            Cylinder(radius=HINGE_R, length=HINGE_LEN),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=hinge_rpy),
            material=rust,
            name=f"backrest_hinge_{i}",
        )
        backrest_parts.append(bp)

        jnt = model.articulation(
            f"backrest_tilt_{i}",
            ArticulationType.REVOLUTE,
            parent=beam,
            child=bp,
            origin=Origin(xyz=joint_xyz),
            axis=axis,
            motion_limits=MotionLimits(
                effort=8.0, velocity=2.0, lower=0.0, upper=BACKREST_TILT_MAX
            ),
        )
        backrest_joints.append(jnt)

    # --------------------------------------------------------- main pivot ---
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
    base = object_model.get_part("arched_base")
    beam = object_model.get_part("cross_beam")
    pivot = object_model.get_articulation("beam_pivot")

    # --- Pivot sleeve captures axle bolt (intentional nested bushing) ---
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

    # --- Beam bars clear the arch saddle ---
    ctx.expect_gap(
        beam,
        base,
        axis="z",
        positive_elem="beam_bar_x",
        negative_elem="arch_0",
        min_gap=0.005,
        max_gap=0.06,
        name="X-beam bar clears the arch saddle",
    )

    # --- Main pivot: axis and limits ---
    ax = pivot.axis
    ctx.check(
        "pivot axis is horizontal and perpendicular to X-beam",
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

    # --- Cross beam: two perpendicular bars ---
    bar_x_box = ctx.part_element_world_aabb(beam, elem="beam_bar_x")
    bar_y_box = ctx.part_element_world_aabb(beam, elem="beam_bar_y")
    ctx.check(
        "X-beam is about 3.0 m long",
        bar_x_box is not None and abs((bar_x_box[1][0] - bar_x_box[0][0]) - 3.0) < 0.02,
        details=f"bar_x aabb={bar_x_box}",
    )
    ctx.check(
        "Y-beam is about 3.0 m long",
        bar_y_box is not None and abs((bar_y_box[1][1] - bar_y_box[0][1]) - 3.0) < 0.02,
        details=f"bar_y aabb={bar_y_box}",
    )
    ctx.check(
        "X-beam is narrow in Y (80 mm)",
        bar_x_box is not None and abs((bar_x_box[1][1] - bar_x_box[0][1]) - BEAM_W) < 0.01,
        details=f"bar_x Y extent={bar_x_box}",
    )
    ctx.check(
        "Y-beam is narrow in X (80 mm)",
        bar_y_box is not None and abs((bar_y_box[1][0] - bar_y_box[0][0]) - BEAM_W) < 0.01,
        details=f"bar_y X extent={bar_y_box}",
    )

    # --- Base: arches grounded, ground pads present ---
    base_box = ctx.part_world_aabb(base)
    ctx.check(
        "arched base rests on the ground",
        base_box is not None and -0.01 <= base_box[0][2] <= 0.02,
        details=f"base aabb={base_box}",
    )
    for i in range(4):
        pad_box = ctx.part_element_world_aabb(base, elem=f"ground_pad_{i}")
        ctx.check(
            f"ground_pad_{i} sits on the ground",
            pad_box is not None and pad_box[0][2] < 0.002 and pad_box[1][2] < 0.03,
            details=f"pad aabb={pad_box}",
        )

    # --- Four seats present ---
    for i in range(4):
        seat_box = ctx.part_element_world_aabb(beam, elem=f"seat_{i}")
        ctx.check(
            f"seat_{i} is present above the beam",
            seat_box is not None and bar_x_box is not None and seat_box[1][2] > bar_x_box[1][2] - 0.01,
            details=f"seat aabb={seat_box}",
        )

    # --- Four bump stops below beam ends ---
    for i in range(4):
        bump_box = ctx.part_element_world_aabb(beam, elem=f"bump_stop_{i}")
        ctx.check(
            f"bump_stop_{i} hangs below the beam",
            bump_box is not None and bar_x_box is not None and bump_box[0][2] < bar_x_box[0][2],
            details=f"bump_stop aabb={bump_box}",
        )

    # --- Backrest hinge pins seated in brackets (intentional overlap) ---
    for i in range(4):
        bp_i = object_model.get_part(f"backrest_{i}")
        ctx.allow_overlap(
            bp_i,
            beam,
            elem_a=f"backrest_hinge_{i}",
            elem_b=f"hinge_bracket_{i}",
            reason=f"Backrest {i} hinge pin is a captured pivot seated inside the bracket bore.",
        )
        ctx.expect_contact(
            bp_i,
            beam,
            elem_a=f"backrest_hinge_{i}",
            elem_b=f"hinge_bracket_{i}",
            name=f"backrest_{i} hinge pin contacts the bracket",
        )

    # --- Backrest tilt joints exist and have correct configuration ---
    for i in range(4):
        bp = object_model.get_part(f"backrest_{i}")
        bj = object_model.get_articulation(f"backrest_tilt_{i}")
        ctx.check(
            f"backrest_tilt_{i} is a non-fixed revolute joint",
            bj.articulation_type == ArticulationType.REVOLUTE,
            details=f"type={bj.articulation_type}",
        )
        blim = bj.motion_limits
        ctx.check(
            f"backrest_tilt_{i} has positive tilt range up to ~30 degrees",
            blim is not None
            and blim.upper is not None
            and blim.upper > 0.3
            and blim.upper < 0.7,
            details=f"upper={blim.upper if blim else None}",
        )
        # Backrest plate is present
        bp_box = ctx.part_world_aabb(bp)
        ctx.check(
            f"backrest_{i} plate is present near its seat",
            bp_box is not None and bp_box[1][2] > PIVOT_Z + 0.05,
            details=f"backrest aabb={bp_box}",
        )

    # --- Decisive pose: main pivot rocks the beam ---
    rest_bump0 = ctx.part_element_world_aabb(beam, elem="bump_stop_0")
    with ctx.pose({pivot: TILT}):
        down_bump0 = ctx.part_element_world_aabb(beam, elem="bump_stop_0")
        up_bump1 = ctx.part_element_world_aabb(beam, elem="bump_stop_1")
        ctx.check(
            "positive rock lowers the +X end near the ground",
            rest_bump0 is not None
            and down_bump0 is not None
            and down_bump0[0][2] < rest_bump0[0][2] - 0.30
            and down_bump0[0][2] > 0.0,
            details=f"rest={rest_bump0}, tilted={down_bump0}",
        )
        ctx.check(
            "positive rock raises the -X end",
            up_bump1 is not None and up_bump1[0][2] > 1.0,
            details=f"raised bump_stop aabb={up_bump1}",
        )
    with ctx.pose({pivot: -TILT}):
        down_bump1 = ctx.part_element_world_aabb(beam, elem="bump_stop_1")
        ctx.check(
            "negative rock lowers the -X end near the ground",
            down_bump1 is not None and 0.0 < down_bump1[0][2] < 0.35,
            details=f"tilted bump_stop aabb={down_bump1}",
        )

    # --- Decisive pose: backrest tilts backward ---
    backrest_0 = object_model.get_part("backrest_0")
    backrest_0_joint = object_model.get_articulation("backrest_tilt_0")
    rest_br0 = ctx.part_world_aabb(backrest_0)
    with ctx.pose({backrest_0_joint: BACKREST_TILT_MAX}):
        tilted_br0 = ctx.part_world_aabb(backrest_0)
        ctx.check(
            "backrest_0 tilts backward when joint is positive",
            rest_br0 is not None
            and tilted_br0 is not None
            and tilted_br0[1][0] > rest_br0[1][0] + 0.01,
            details=f"rest={rest_br0}, tilted={tilted_br0}",
        )

    return ctx.report()


object_model = build_object_model()
