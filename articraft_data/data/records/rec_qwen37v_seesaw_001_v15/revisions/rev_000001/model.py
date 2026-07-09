from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Heavy commercial steel playground seesaw with molded seats
#
# World frame: beam runs along X, pivot axis along Y, Z up.
# - Two bent galvanized-steel tube arches (~50 mm dia) form an A-shaped saddle;
#   the apex carries a horizontal pivot axle bolt.
# - The rocking beam is a 3.0 m heavy commercial steel bar (100 x 50 mm) with
#   a pivot sleeve + triangular gusset at center, a molded seat with raised
#   lips, an inverted-U grab handle, and a curved rubber tire-section bumper
#   per end.
# - Single revolute joint at the apex, axis (0, 1, 0), +/- 20 degrees.
#   Positive q lowers the +X end (right-hand rule about +Y).
# ---------------------------------------------------------------------------

BEAM_HALF = 1.50  # 3.0 m beam
BEAM_W = 0.10     # 100 mm wide (heavy commercial)
BEAM_T = 0.05     # 50 mm thick
PIVOT_Z = 0.76    # axle height (about 0.8 m tall at the pivot)

ARCH_FOOT_X = 0.66
ARCH_FOOT_Y = 0.34
ARCH_APEX_Y = 0.05
ARCH_FOOT_Z = 0.028
TUBE_R = 0.025    # ~50 mm diameter bent tube

AXLE_R = 0.016
AXLE_LEN = 0.24

# Beam-local frame: origin at the axle center; the bar bottom sits 50 mm above.
BAR_BOT = 0.05
BAR_CTR = BAR_BOT + BEAM_T / 2.0  # 0.075
BAR_TOP = BAR_BOT + BEAM_T        # 0.10

SEAT_X = 1.25
HANDLE_X = 1.00
BUMPER_X = 1.42
TILT = math.radians(20.0)

# Molded seat dimensions
SEAT_LEN = 0.30
SEAT_WID = 0.24
SEAT_H = 0.042       # total outer height (lip height)
SEAT_WALL = 0.018    # lip wall thickness
SEAT_BASIN = 0.026   # basin cut depth


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


def _handle_points(x: float) -> list[tuple[float, float, float]]:
    """Inverted-U grab-handle rod centerline, plane across the beam (YZ)."""
    half_w = 0.040
    leg_bot = BAR_TOP - 0.012  # rod tip embedded in the beam bar
    arc_z = 0.280
    pts: list[tuple[float, float, float]] = [
        (x, -half_w, leg_bot),
        (x, -half_w, 0.200),
    ]
    for k in range(7):  # semicircular top bend
        a = math.pi * k / 6.0
        pts.append((x, -half_w * math.cos(a), arc_z + half_w * math.sin(a)))
    pts.append((x, half_w, 0.200))
    pts.append((x, half_w, leg_bot)
    )
    return pts


def _bumper_geometry(x: float, index: int):
    """Curved tire-section bumper: half-annulus shell extruded across the beam."""
    r_out = 0.070
    r_in = 0.050
    profile: list[tuple[float, float]] = []
    n = 20
    for k in range(n + 1):  # outer arc, bottom half (pi .. 2*pi)
        a = math.pi + math.pi * k / n
        profile.append((r_out * math.cos(a), r_out * math.sin(a)))
    for k in range(n + 1):  # inner arc back (2*pi .. pi)
        a = 2.0 * math.pi - math.pi * k / n
        profile.append((r_in * math.cos(a), r_in * math.sin(a)))
    geom = ExtrudeGeometry(profile, 0.12, cap=True, center=True)
    geom.rotate_x(math.pi / 2.0)
    geom.translate(x, 0.0, BAR_BOT + 0.002)
    return mesh_from_geometry(geom, f"seesaw_bumper_{index}")


def _gusset_geometry():
    """Triangular gusset plate joining the beam bar to the pivot sleeve."""
    profile = [(-0.12, 0.055), (0.12, 0.055), (0.0, 0.020)]
    geom = ExtrudeGeometry(profile, 0.024, cap=True, center=True)
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, "seesaw_gusset_plate")


def _molded_seat_mesh(index: int):
    """Molded seat with raised lips: a shallow rectangular dish.

    The outer box defines the lip tops. The basin is cut from the top face,
    leaving raised lip walls on all four sides.
    """
    seat = (
        cq.Workplane("XY")
        .box(SEAT_LEN, SEAT_WID, SEAT_H)
        .faces(">Z").workplane()
        .rect(SEAT_LEN - 2.0 * SEAT_WALL, SEAT_WID - 2.0 * SEAT_WALL)
        .cutBlind(-SEAT_BASIN)
    )
    return mesh_from_cadquery(seat, f"molded_seat_{index}")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_steel_seesaw")

    galvanized = model.material("weathered_galvanized", rgba=(0.52, 0.55, 0.53, 1.0))
    rust = model.material("rust_steel", rgba=(0.42, 0.25, 0.13, 1.0))
    dark_steel = model.material("commercial_steel", rgba=(0.30, 0.32, 0.34, 1.0))
    pale_steel = model.material("pale_weathered_steel", rgba=(0.65, 0.62, 0.56, 1.0))
    molded_rubber = model.material("molded_rubber_seat", rgba=(0.12, 0.12, 0.14, 1.0))
    rubber = model.material("black_rubber", rgba=(0.06, 0.06, 0.06, 1.0))

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
                f"seesaw_arch_{i}",
            ),
            material=galvanized,
            name=f"arch_{i}",
        )

    # Pivot axle bolt through both arch apexes, axis along Y.
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

    beam.visual(
        Cylinder(radius=0.028, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rust,
        name="pivot_sleeve",
    )
    beam.visual(_gusset_geometry(), material=dark_steel, name="gusset_plate")

    beam.visual(
        Box((2.0 * BEAM_HALF, BEAM_W, BEAM_T)),
        origin=Origin(xyz=(0.0, 0.0, BAR_CTR)),
        material=dark_steel,
        name="beam_bar",
    )

    # Rust streak patches on the steel bar.
    for i, px in enumerate((-0.85, -0.30, 0.45, 0.95)):
        beam.visual(
            Box((0.16, BEAM_W + 0.004, 0.012)),
            origin=Origin(xyz=(px, 0.0, BAR_TOP - 0.004)),
            material=rust,
            name=f"rust_patch_{i}",
        )

    # End fittings: molded seats with raised lips, grab handles, rubber bumpers.
    for i, side in enumerate((1.0, -1.0)):
        # Molded seat with raised lips — sits on the beam bar top.
        beam.visual(
            _molded_seat_mesh(i),
            origin=Origin(xyz=(side * SEAT_X, 0.0, BAR_TOP + SEAT_H / 2.0)),
            material=molded_rubber,
            name=f"seat_{i}",
        )
        # Grab handle just inboard of the seat.
        beam.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _handle_points(side * HANDLE_X),
                    radius=0.010,
                    samples_per_segment=8,
                    radial_segments=16,
                    cap_ends=True,
                ),
                f"seesaw_handle_{i}",
            ),
            material=pale_steel,
            name=f"handle_{i}",
        )
        # Rubber tire-section bumper under the beam tip.
        beam.visual(
            _bumper_geometry(side * BUMPER_X, i),
            material=rubber,
            name=f"bumper_{i}",
        )

    # -------------------------------------------------------------- joint ---
    model.articulation(
        "beam_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=2.0, lower=-TILT, upper=TILT),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("arched_base")
    beam = object_model.get_part("beam")
    pivot = object_model.get_articulation("beam_pivot")

    # Pivot sleeve captures the base axle bolt (intentional nesting).
    ctx.allow_overlap(
        beam,
        base,
        elem_a="pivot_sleeve",
        elem_b="pivot_axle",
        reason="Pivot sleeve is a bushing intentionally nested around the axle bolt.",
    )
    # Pivot sleeve passes through the arch saddle apex where the two arches
    # cross close to the axle centerline; small local tessellation overlap
    # at the tangent contact between the sleeve OD and the arch tube ID.
    ctx.allow_overlap(
        base,
        beam,
        elem_a="arch_0",
        elem_b="pivot_sleeve",
        reason="Arch tube tangent contact with the pivot sleeve at the saddle apex; local embedding from tessellation of the cylindrical crossing.",
    )
    ctx.allow_overlap(
        base,
        beam,
        elem_a="arch_1",
        elem_b="pivot_sleeve",
        reason="Arch tube tangent contact with the pivot sleeve at the saddle apex; local embedding from tessellation of the cylindrical crossing.",
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

    # Beam bar clears the arch saddle.
    ctx.expect_gap(
        beam,
        base,
        axis="z",
        positive_elem="beam_bar",
        negative_elem="arch_0",
        min_gap=0.005,
        max_gap=0.06,
        name="beam bar clears the arch saddle",
    )

    # Joint: horizontal Y axis, +/- 20 degree rocking limits.
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

    # Hero geometry: beam length, saddle height, grounded feet.
    bar_box = ctx.part_element_world_aabb(beam, elem="beam_bar")
    base_box = ctx.part_world_aabb(base)
    axle_box = ctx.part_element_world_aabb(base, elem="pivot_axle")
    ctx.check(
        "beam is about 3.0 m long",
        bar_box is not None and abs((bar_box[1][0] - bar_box[0][0]) - 3.0) < 0.02,
        details=f"bar aabb={bar_box}",
    )
    ctx.check(
        "arched base feet rest on the ground",
        base_box is not None and -0.01 <= base_box[0][2] <= 0.02,
        details=f"base aabb={base_box}",
    )
    ctx.check(
        "pivot axle sits about 0.8 m high",
        axle_box is not None and 0.70 <= (axle_box[0][2] + axle_box[1][2]) / 2.0 <= 0.82,
        details=f"axle aabb={axle_box}",
    )

    # Heavy commercial beam: width >= 0.095 m and thickness >= 0.045 m.
    ctx.check(
        "beam bar is heavy commercial section (>=95 mm wide)",
        bar_box is not None and (bar_box[1][1] - bar_box[0][1]) >= 0.095,
        details=f"bar width={bar_box[1][1] - bar_box[0][1]:.3f}" if bar_box else "no bar",
    )
    ctx.check(
        "beam bar is heavy commercial section (>=45 mm thick)",
        bar_box is not None and (bar_box[1][2] - bar_box[0][2]) >= 0.045,
        details=f"bar thickness={bar_box[1][2] - bar_box[0][2]:.3f}" if bar_box else "no bar",
    )

    # Molded seats with raised lips: each seat must have significant Z extent
    # (lip height) and the lip top must extend above the beam bar top.
    for i in range(2):
        seat_box = ctx.part_element_world_aabb(beam, elem=f"seat_{i}")
        handle_box = ctx.part_element_world_aabb(beam, elem=f"handle_{i}")
        bumper_box = ctx.part_element_world_aabb(beam, elem=f"bumper_{i}")

        ctx.check(
            f"seat_{i} is a molded dish with raised lips (Z extent >= {SEAT_H - 0.005:.3f} m)",
            seat_box is not None
            and (seat_box[1][2] - seat_box[0][2]) >= SEAT_H - 0.005,
            details=f"seat aabb={seat_box}",
        )
        ctx.check(
            f"seat_{i} lip top extends above the beam bar top",
            seat_box is not None
            and bar_box is not None
            and seat_box[1][2] > bar_box[1][2] + 0.025,
            details=f"seat top={seat_box[1][2]:.4f}, bar top={bar_box[1][2]:.4f}" if seat_box and bar_box else "missing",
        )
        ctx.check(
            f"seat_{i} bottom is seated on the beam bar top",
            seat_box is not None
            and bar_box is not None
            and abs(seat_box[0][2] - bar_box[1][2]) < 0.008,
            details=f"seat bot={seat_box[0][2]:.4f}, bar top={bar_box[1][2]:.4f}" if seat_box and bar_box else "missing",
        )
        ctx.check(
            f"handle_{i} stands above the beam",
            handle_box is not None
            and bar_box is not None
            and handle_box[1][2] > bar_box[1][2] + 0.18
            and handle_box[0][2] < bar_box[1][2],
            details=f"handle aabb={handle_box}",
        )
        ctx.check(
            f"bumper_{i} hangs below the beam tip",
            bumper_box is not None
            and bar_box is not None
            and bumper_box[0][2] < bar_box[0][2]
            and min(abs(bumper_box[0][0]), abs(bumper_box[1][0])) > 1.3,
            details=f"bumper aabb={bumper_box}",
        )

    # Decisive pose checks: rocking alternately lowers each end.
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
