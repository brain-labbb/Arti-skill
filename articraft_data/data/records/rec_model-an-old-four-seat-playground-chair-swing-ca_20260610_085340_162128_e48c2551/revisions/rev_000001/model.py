from __future__ import annotations

# Old four-seat playground chair-swing carousel.
# Reference image: picture/Playground/playground chair swing carousel/002.png
#
# Layout (world frame, meters):
# - Rusty orange-red center column (~0.15 m dia) on a ground plate, with a
#   bearing collar and an exposed spindle bolt + hex nut on top.
# - A rotor turns on the column via a CONTINUOUS Z joint: a hollow hub sleeve
#   seated on the bearing collar, an annular top cap, four tangential pivot
#   bars at radius 1.40 m, and eight cream tubes. Each seat is carried by two
#   tubes that leave the hub splayed +/-55 deg from the seat direction, so
#   adjacent pairs cross mid-span into a continuous X-lattice in plan view.
# - Each of the four seats hangs from its pivot bar on a REVOLUTE joint with a
#   horizontal tangential axis (+/-30 deg outward/inward swing): two collar
#   bushings captured on the bar, two drop straps, a slatted sheet-metal
#   platform (~0.5 x 0.4 m) on side rails, and a thin bent-tube guard rail
#   wrapping the outboard and lateral sides as backrest and armrests.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Shared dimensions
# ---------------------------------------------------------------------------
COLUMN_R = 0.075
HUB_Z = 0.90  # world height of the rotor joint frame
SEAT_R = 1.40  # radius of the seat pivot bars
BAR_Z = -0.10  # pivot bar height in the rotor frame (world 0.80)
TUBE_R = 0.026
TUBE_END_OFF = 0.26  # tangential half-spacing of the twin tubes at a corner
SPLAY = math.radians(55.0)  # hub-side splay of each tube pair
BAR_HALF = 0.30
SWING = math.radians(30.0)

SEAT_ANGLES = tuple(i * math.pi / 2.0 for i in range(4))


def _polar(radius: float, angle: float) -> tuple[float, float]:
    return (radius * math.cos(angle), radius * math.sin(angle))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chair_swing_carousel")

    cream = model.material("cream_weathered", rgba=(0.87, 0.84, 0.76, 1.0))
    cream_dirty = model.material("cream_dirty", rgba=(0.79, 0.75, 0.66, 1.0))
    slat_pale = model.material("slat_pale", rgba=(0.85, 0.81, 0.74, 1.0))
    slat_stained = model.material("slat_stained", rgba=(0.76, 0.70, 0.61, 1.0))
    rust_red = model.material("rust_red", rgba=(0.66, 0.28, 0.15, 1.0))
    rust_brown = model.material("rust_brown", rgba=(0.43, 0.27, 0.18, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.38, 0.35, 0.32, 1.0))

    # ------------------------------------------------------------------
    # Column (root): ground plate, shaft, bearing collar, spindle bolt.
    # ------------------------------------------------------------------
    column = model.part("column")
    column.visual(
        Cylinder(radius=0.17, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=rust_brown,
        name="base_plate",
    )
    column.visual(
        Cylinder(radius=COLUMN_R, length=0.91),
        origin=Origin(xyz=(0.0, 0.0, 0.475)),
        material=rust_red,
        name="column_shaft",
    )
    column.visual(
        Cylinder(radius=0.10, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.73)),
        material=rust_brown,
        name="bearing_collar",
    )
    column.visual(
        Cylinder(radius=0.020, length=0.23),
        origin=Origin(xyz=(0.0, 0.0, 1.015)),
        material=steel_dark,
        name="spindle",
    )
    nut_shape = cq.Workplane("XY").polygon(6, 0.066).extrude(0.030)
    column.visual(
        mesh_from_cadquery(nut_shape, "spindle_nut"),
        origin=Origin(xyz=(0.0, 0.0, 1.10)),
        material=steel_dark,
        name="spindle_nut",
    )

    # ------------------------------------------------------------------
    # Rotor: hollow hub sleeve, annular cap, X-lattice tubes, pivot bars.
    # Rotor frame z=0 sits at world HUB_Z.
    # ------------------------------------------------------------------
    rotor = model.part("rotor")

    sleeve_shape = cq.Workplane("XY").circle(0.105).circle(0.080).extrude(0.315)
    rotor.visual(
        mesh_from_cadquery(sleeve_shape, "hub_sleeve"),
        origin=Origin(xyz=(0.0, 0.0, -0.155)),  # bottom rests on bearing collar
        material=rust_red,
        name="hub_sleeve",
    )
    cap_shape = cq.Workplane("XY").circle(0.115).circle(0.030).extrude(0.030)
    rotor.visual(
        mesh_from_cadquery(cap_shape, "hub_cap"),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material=rust_red,
        name="hub_cap",
    )

    for i, theta in enumerate(SEAT_ANGLES):
        cx, cy = _polar(SEAT_R, theta)
        tx, ty = (-math.sin(theta), math.cos(theta))  # tangential unit vector

        # Tangential pivot bar carrying the hanging seat.
        rotor.visual(
            Cylinder(radius=0.022, length=2.0 * BAR_HALF),
            origin=Origin(xyz=(cx, cy, BAR_Z), rpy=(math.pi / 2.0, 0.0, theta)),
            material=cream_dirty,
            name=f"pivot_bar_{i}",
        )

        # Twin support tubes, splayed at the hub so neighboring pairs cross.
        for k, s in enumerate((1.0, -1.0)):
            sx, sy = _polar(0.095, theta + s * SPLAY)
            start = (sx, sy, 0.07)
            end = (cx + s * TUBE_END_OFF * tx, cy + s * TUBE_END_OFF * ty, BAR_Z)
            mid = (
                0.5 * (start[0] + end[0]),
                0.5 * (start[1] + end[1]),
                0.5 * (start[2] + end[2]) - 0.02,
            )
            tube = tube_from_spline_points(
                [start, mid, end],
                radius=TUBE_R,
                samples_per_segment=14,
                radial_segments=16,
                cap_ends=True,
            )
            rotor.visual(
                mesh_from_geometry(tube, f"arm_tube_{i}_{k}"),
                material=cream,
                name=f"arm_tube_{i}_{k}",
            )

    model.articulation(
        "column_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=column,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, HUB_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=400.0, velocity=2.0),
    )

    # ------------------------------------------------------------------
    # Seats: hung from each pivot bar on a tangential revolute axis.
    # Seat frame: origin at bar center, +x radially outboard, +y tangential.
    # ------------------------------------------------------------------
    strap_tilt = 0.122  # lean of the drop straps (top at collar, foot wider)
    for i, theta in enumerate(SEAT_ANGLES):
        seat = model.part(f"seat_{i}")

        for j, sy in enumerate((1.0, -1.0)):
            # Collar bushing captured around the pivot bar.
            seat.visual(
                Cylinder(radius=0.034, length=0.05),
                origin=Origin(xyz=(0.0, sy * 0.19, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=rust_brown,
                name=f"collar_{j}",
            )
            # Flat steel drop strap from the collar to the platform edge.
            seat.visual(
                Box((0.032, 0.010, 0.24)),
                origin=Origin(
                    xyz=(0.0, sy * 0.2075, -0.146),
                    rpy=(sy * strap_tilt, 0.0, 0.0),
                ),
                material=cream_dirty,
                name=f"strap_{j}",
            )
            # Side rail under the slat ends.
            seat.visual(
                Box((0.40, 0.025, 0.035)),
                origin=Origin(xyz=(0.0, sy * 0.2375, -0.285)),
                material=cream_dirty,
                name=f"side_rail_{j}",
            )

        # Slatted sheet-metal platform, slats running tangentially.
        slat_xs = (-0.1725, -0.1035, -0.0345, 0.0345, 0.1035, 0.1725)
        for j, sx in enumerate(slat_xs):
            seat.visual(
                Box((0.055, 0.50, 0.012)),
                origin=Origin(xyz=(sx, 0.0, -0.2655)),
                material=slat_pale if j % 2 == 0 else slat_stained,
                name=f"slat_{j}",
            )

        # Wraparound bent-tube guard rail: armrest sides + raised backrest
        # on the outboard edge, open toward the center column.
        rail_pts = [
            (-0.175, -0.18, -0.265),
            (-0.175, -0.18, -0.10),
            (-0.175, -0.18, 0.01),
            (-0.13, -0.18, 0.045),
            (0.0, -0.18, 0.05),
            (0.13, -0.18, 0.05),
            (0.185, -0.18, 0.055),
            (0.20, -0.12, 0.06),
            (0.20, 0.0, 0.075),
            (0.20, 0.12, 0.06),
            (0.185, 0.18, 0.055),
            (0.13, 0.18, 0.05),
            (0.0, 0.18, 0.05),
            (-0.13, 0.18, 0.045),
            (-0.175, 0.18, 0.01),
            (-0.175, 0.18, -0.10),
            (-0.175, 0.18, -0.265),
        ]
        rail = tube_from_spline_points(
            rail_pts,
            radius=0.013,
            samples_per_segment=8,
            radial_segments=14,
            cap_ends=True,
        )
        seat.visual(
            mesh_from_geometry(rail, f"guard_rail_{i}"),
            material=cream,
            name="guard_rail",
        )
        for j, sy in enumerate((1.0, -1.0)):
            seat.visual(
                Cylinder(radius=0.011, length=0.317),
                origin=Origin(xyz=(0.175, sy * 0.18, -0.1065)),
                material=cream,
                name=f"rail_post_{j}",
            )

        cx, cy = _polar(SEAT_R, theta)
        model.articulation(
            f"rotor_to_seat_{i}",
            ArticulationType.REVOLUTE,
            parent=rotor,
            child=seat,
            origin=Origin(xyz=(cx, cy, BAR_Z), rpy=(0.0, 0.0, theta)),
            # Tangential horizontal axis; -y so positive q swings the hanging
            # seat radially outward.
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=200.0, velocity=1.5, lower=-SWING, upper=SWING
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    column = object_model.get_part("column")
    rotor = object_model.get_part("rotor")
    seats = [object_model.get_part(f"seat_{i}") for i in range(4)]
    spin = object_model.get_articulation("column_to_rotor")
    swing_0 = object_model.get_articulation("rotor_to_seat_0")

    # Hinge bushings are intentionally captured around the pivot bars.
    for i, seat in enumerate(seats):
        for j in range(2):
            ctx.allow_overlap(
                seat,
                rotor,
                elem_a=f"collar_{j}",
                elem_b=f"pivot_bar_{i}",
                reason="Seat hanger bushing is intentionally captured around the rotor pivot bar.",
            )
            ctx.expect_contact(
                seat,
                rotor,
                elem_a=f"collar_{j}",
                elem_b=f"pivot_bar_{i}",
                name=f"seat_{i} collar_{j} rides on its pivot bar",
            )

    # Rotor hub sleeve is seated on the column bearing collar (not floating).
    ctx.expect_contact(
        rotor,
        column,
        elem_a="hub_sleeve",
        elem_b="bearing_collar",
        contact_tol=2e-3,
        name="hub sleeve seats on the bearing collar",
    )

    # Exposed spindle bolt: nut sits above the rotor cap, spindle centered
    # through the cap bore.
    ctx.expect_gap(
        column,
        rotor,
        axis="z",
        positive_elem="spindle_nut",
        negative_elem="hub_cap",
        min_gap=0.005,
        max_gap=0.05,
        name="spindle nut exposed above the hub cap",
    )
    ctx.expect_within(
        column,
        rotor,
        axes="xy",
        inner_elem="spindle",
        outer_elem="hub_cap",
        name="spindle stays centered through the hub cap bore",
    )

    # Eight lattice tubes radiate from the hub (two per seat).
    n_tubes = sum(
        1 for v in rotor.visuals if v.name is not None and v.name.startswith("arm_tube_")
    )
    ctx.check(
        "rotor carries eight crossing lattice tubes",
        n_tubes == 8,
        details=f"found {n_tubes} arm tubes",
    )

    # Four seats hang at the four corners, ~3.2 m overall diameter.
    ctx.expect_origin_distance(seats[0], seats[2], axes="xy", min_dist=2.7, max_dist=2.9)
    ctx.expect_origin_distance(seats[1], seats[3], axes="xy", min_dist=2.7, max_dist=2.9)
    ctx.expect_origin_distance(seats[0], seats[1], axes="xy", min_dist=1.9, max_dist=2.1)
    a0 = ctx.part_world_aabb(seats[0])
    a2 = ctx.part_world_aabb(seats[2])
    span = a0[1][0] - a2[0][0] if a0 is not None and a2 is not None else 0.0
    ctx.check(
        "overall diameter is about 3.2 m",
        3.0 <= span <= 3.4,
        details=f"seat-to-seat x span = {span:.3f} m",
    )

    # Structure height: column + spindle bolt top out near 1.1-1.2 m.
    col_aabb = ctx.part_world_aabb(column)
    ctx.check(
        "structure stands about 1.1-1.2 m tall",
        col_aabb is not None and 1.05 <= col_aabb[1][2] <= 1.30,
        details=f"column aabb = {col_aabb}",
    )

    # Seat platform hangs below the pivot bar; guard rail rises above it.
    slat = ctx.part_element_world_aabb(seats[0], elem="slat_0")
    rail = ctx.part_element_world_aabb(seats[0], elem="guard_rail")
    ctx.check(
        "seat platform hangs below the pivot bar",
        slat is not None and slat[1][2] < HUB_Z + BAR_Z - 0.15,
        details=f"slat aabb = {slat}",
    )
    ctx.check(
        "guard rail rises well above the seat platform",
        slat is not None and rail is not None and rail[1][2] > slat[1][2] + 0.20,
        details=f"slat top = {slat}, rail = {rail}",
    )

    # Decisive pose checks. The outboard slat both translates outward and
    # rises as the hanging seat pendulums outward about the tangential axis.
    rest = ctx.part_element_world_aabb(seats[0], elem="slat_5")
    with ctx.pose({swing_0: SWING}):
        out = ctx.part_element_world_aabb(seats[0], elem="slat_5")
    ctx.check(
        "positive swing moves the seat platform outward and up",
        rest is not None
        and out is not None
        and out[0][0] > rest[0][0] + 0.05
        and out[0][2] > rest[0][2] + 0.05,
        details=f"rest = {rest}, swung = {out}",
    )

    rest_pos = ctx.part_world_position(seats[0])
    with ctx.pose({spin: math.pi / 2.0}):
        spun_pos = ctx.part_world_position(seats[0])
    ctx.check(
        "rotor spin carries the seats around the column",
        rest_pos is not None
        and spun_pos is not None
        and abs(spun_pos[1] - SEAT_R) < 0.05
        and abs(spun_pos[0]) < 0.05,
        details=f"rest = {rest_pos}, spun = {spun_pos}",
    )

    return ctx.report()


object_model = build_object_model()
