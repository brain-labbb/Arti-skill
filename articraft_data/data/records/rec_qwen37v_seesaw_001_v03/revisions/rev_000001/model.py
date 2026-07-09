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
# Spring-assisted modern playground seesaw
#
# World frame: beam runs along X, pivot axis along Y, Z up.
# - Modern welded steel base: flat ground plate with a central upright
#   pedestal and a cylindrical pivot housing. A visible coil spring wraps
#   the pivot axle to provide restoring force.
# - Rocking beam: 3.0 m rectangular steel tube painted bright red with
#   slight weathering. Each end carries a molded HDPE seat with raised
#   perimeter lips, an inboard grab handle, and a rubber bumper on a
#   short prismatic compression joint.
# - Main articulation: REVOLUTE at the pivot, axis (0, 1, 0), ±20°.
# - Bumper articulations: PRISMATIC along (0, 0, 1), 0–0.04 m travel.
# ---------------------------------------------------------------------------

BEAM_HALF = 1.50
BEAM_W = 0.080
BEAM_T = 0.040
PIVOT_Z = 0.76

# Base dimensions
BASE_PLATE_W = 0.80  # along X
BASE_PLATE_D = 0.50  # along Y
BASE_PLATE_T = 0.020
PEDESTAL_W = 0.10
PEDESTAL_D = 0.10
# Pedestal stops below the pivot sleeve (sleeve bottom = PIVOT_Z - 0.024)
PEDESTAL_TOP = PIVOT_Z - 0.024 - 0.004  # 4mm clearance below sleeve
PEDESTAL_H = PEDESTAL_TOP - BASE_PLATE_T

AXLE_R = 0.016
AXLE_LEN = 0.20

# Beam-local frame: origin at axle center
BAR_BOT = 0.050
BAR_CTR = BAR_BOT + BEAM_T / 2.0
BAR_TOP = BAR_BOT + BEAM_T

SEAT_X = 1.25
HANDLE_X = 0.98
BUMPER_X = 1.42
TILT = math.radians(20.0)
BUMPER_TRAVEL = 0.040


def _spring_points(
    coils: int = 5,
    radius: float = 0.034,
    height: float = 0.10,
    n_per_coil: int = 28,
) -> list[tuple[float, float, float]]:
    """Helical centerline for the coil spring around the pivot."""
    pts: list[tuple[float, float, float]] = []
    total = coils * n_per_coil
    for i in range(total + 1):
        t = i / total
        angle = 2.0 * math.pi * coils * t
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        z = height * t - height / 2.0
        pts.append((x, y, z))
    return pts


def _handle_points(x: float) -> list[tuple[float, float, float]]:
    """Inverted-U grab-handle rod centerline in the YZ plane."""
    half_w = 0.035
    leg_bot = BAR_TOP - 0.010
    arc_z = 0.260
    pts: list[tuple[float, float, float]] = [
        (x, -half_w, leg_bot),
        (x, -half_w, 0.180),
    ]
    for k in range(7):
        a = math.pi * k / 6.0
        pts.append((x, -half_w * math.cos(a), arc_z + half_w * math.sin(a)))
    pts.append((x, half_w, 0.180))
    pts.append((x, half_w, leg_bot))
    return pts


def _make_molded_seat(side: float) -> object:
    """CadQuery molded HDPE seat with raised perimeter lips.

    The seat is a shallow open-top tray: 300×240 mm outer, 38 mm deep
    with 7 mm walls and a 10 mm floor. The raised lips are the 28 mm
    tall walls above the seat floor.
    """
    outer_w = 0.30
    outer_d = 0.24
    outer_h = 0.038
    wall = 0.007
    floor = 0.010

    seat = (
        cq.Workplane("XY")
        .rect(outer_w, outer_d)
        .extrude(outer_h)
        .edges("|Z")
        .fillet(0.012)
    )
    # Hollow out the interior from the top, leaving walls and floor.
    inner_w = outer_w - 2.0 * wall
    inner_d = outer_d - 2.0 * wall
    cut_depth = outer_h - floor
    seat = (
        seat
        .faces(">Z")
        .workplane()
        .rect(inner_w, inner_d)
        .cutBlind(-cut_depth)
    )
    # Position: center the seat at x = side * SEAT_X, on top of the beam bar.
    seat = seat.translate((side * SEAT_X, 0.0, BAR_TOP))
    return seat


def _make_bumper_block() -> object:
    """CadQuery rubber bumper: rounded block with a mounting boss on top."""
    # Main rubber body
    body = (
        cq.Workplane("XY")
        .rect(0.10, 0.08)
        .extrude(0.045)
        .edges("|Z")
        .fillet(0.008)
        .edges(">Z")
        .fillet(0.004)
    )
    # Mounting boss (steel plate on top, connects to beam bracket)
    boss = (
        cq.Workplane("XY")
        .workplane(offset=0.045)
        .rect(0.06, 0.05)
        .extrude(0.008)
    )
    return body.union(boss)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="spring_assisted_seesaw")

    # Materials
    dark_gray = model.material("powder_coat_gray", rgba=(0.30, 0.32, 0.33, 1.0))
    bright_red = model.material("safety_red", rgba=(0.80, 0.12, 0.10, 1.0))
    seat_blue = model.material("molded_hdpe_blue", rgba=(0.15, 0.38, 0.65, 1.0))
    rubber = model.material("black_rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    spring_steel = model.material("spring_steel", rgba=(0.52, 0.55, 0.50, 1.0))
    chrome = model.material("chrome_axle", rgba=(0.72, 0.74, 0.73, 1.0))
    handle_gray = model.material("grip_gray", rgba=(0.45, 0.46, 0.44, 1.0))

    # --------------------------------------------------------------- base ---
    base = model.part("base")

    # Ground plate
    base.visual(
        Box((BASE_PLATE_W, BASE_PLATE_D, BASE_PLATE_T)),
        origin=Origin(xyz=(0.0, 0.0, BASE_PLATE_T / 2.0)),
        material=dark_gray,
        name="ground_plate",
    )

    # Four anchor bolt heads on the ground plate corners
    for i, (sx, sy) in enumerate(
        [(1, 1), (1, -1), (-1, 1), (-1, -1)]
    ):
        base.visual(
            Cylinder(radius=0.012, length=0.008),
            origin=Origin(
                xyz=(
                    sx * (BASE_PLATE_W / 2.0 - 0.04),
                    sy * (BASE_PLATE_D / 2.0 - 0.04),
                    BASE_PLATE_T + 0.004,
                )
            ),
            material=chrome,
            name=f"anchor_bolt_{i}",
        )

    # Central pedestal (square tube upright)
    base.visual(
        Box((PEDESTAL_W, PEDESTAL_D, PEDESTAL_H)),
        origin=Origin(
            xyz=(0.0, 0.0, BASE_PLATE_T + PEDESTAL_H / 2.0)
        ),
        material=dark_gray,
        name="pedestal",
    )

    # Pivot housing (bearing block at top of pedestal, below sleeve)
    # Top of housing must clear the bottom of the beam's pivot sleeve.
    housing_h = 0.06
    sleeve_bottom = PIVOT_Z - 0.024  # sleeve radius = 0.024
    housing_center_z = sleeve_bottom - housing_h / 2.0 - 0.002
    base.visual(
        Box((0.14, 0.14, housing_h)),
        origin=Origin(xyz=(0.0, 0.0, housing_center_z)),
        material=dark_gray,
        name="pivot_housing",
    )

    # Pivot axle
    base.visual(
        Cylinder(radius=AXLE_R, length=AXLE_LEN),
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="pivot_axle",
    )

    # Coil spring around the axle (visible spring-assist)
    spring_geom = tube_from_spline_points(
        _spring_points(),
        radius=0.005,
        samples_per_segment=6,
        radial_segments=12,
        cap_ends=True,
    )
    spring_geom.translate(0.0, 0.0, PIVOT_Z)
    base.visual(
        mesh_from_geometry(spring_geom, "coil_spring"),
        material=spring_steel,
        name="coil_spring",
    )

    # Axle end caps
    for i, side in enumerate((1.0, -1.0)):
        base.visual(
            Cylinder(radius=0.022, length=0.010),
            origin=Origin(
                xyz=(0.0, side * (AXLE_LEN / 2.0 - 0.004), PIVOT_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=chrome,
            name=f"axle_cap_{i}",
        )

    # --------------------------------------------------------------- beam ---
    beam = model.part("beam")

    # Pivot sleeve (bushing around axle)
    beam.visual(
        Cylinder(radius=0.024, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="pivot_sleeve",
    )

    # Pivot collar: vertical box bridging sleeve top to beam bar bottom
    collar_h = BAR_BOT - 0.024  # 0.050 - 0.024 = 0.026
    beam.visual(
        Box((0.048, 0.048, collar_h)),
        origin=Origin(xyz=(0.0, 0.0, 0.024 + collar_h / 2.0)),
        material=dark_gray,
        name="pivot_collar",
    )

    # Gusset plates connecting beam bar to pivot sleeve
    for i, side in enumerate((1.0, -1.0)):
        profile = [
            (side * 0.04, 0.055),
            (side * 0.16, 0.055),
            (side * 0.04, 0.015),
        ]
        g = ExtrudeGeometry(profile, 0.012, cap=True, center=True)
        g.rotate_x(math.pi / 2.0)
        beam.visual(
            mesh_from_geometry(g, f"gusset_{i}"),
            material=bright_red,
            name=f"gusset_{i}",
        )

    # Main beam bar
    beam.visual(
        Box((2.0 * BEAM_HALF, BEAM_W, BEAM_T)),
        origin=Origin(xyz=(0.0, 0.0, BAR_CTR)),
        material=bright_red,
        name="beam_bar",
    )

    # Molded seats with raised lips (CadQuery)
    for i, side in enumerate((1.0, -1.0)):
        seat_cq = _make_molded_seat(side)
        beam.visual(
            mesh_from_cadquery(seat_cq, f"molded_seat_{i}"),
            material=seat_blue,
            name=f"seat_{i}",
        )

    # Grab handles
    for i, side in enumerate((1.0, -1.0)):
        beam.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _handle_points(side * HANDLE_X),
                    radius=0.010,
                    samples_per_segment=8,
                    radial_segments=16,
                    cap_ends=True,
                ),
                f"handle_{i}",
            ),
            material=handle_gray,
            name=f"handle_{i}",
        )

    # Bumper mounting brackets (steel plates under beam tips)
    for i, side in enumerate((1.0, -1.0)):
        beam.visual(
            Box((0.12, 0.06, 0.008)),
            origin=Origin(xyz=(side * BUMPER_X, 0.0, BAR_BOT - 0.004)),
            material=dark_gray,
            name=f"bracket_{i}",
        )
        # Guide rails (two thin rods for the bumper to slide on)
        for j, ry in enumerate((-0.020, 0.020)):
            beam.visual(
                Cylinder(radius=0.004, length=0.060),
                origin=Origin(
                    xyz=(side * BUMPER_X, ry, BAR_BOT - 0.008 - 0.030),
                ),
                material=chrome,
                name=f"guide_rail_{i}_{j}",
            )

    # -------------------------------------------------------- bumpers ---
    # Each bumper is a separate part on a prismatic compression joint.
    bumper_cq = _make_bumper_block()

    for i, side in enumerate((1.0, -1.0)):
        bumper = model.part(f"bumper_{i}")
        # The bumper visual: rubber block hanging below the beam tip.
        # Part origin is at the joint location; visual offset downward.
        bumper.visual(
            mesh_from_cadquery(bumper_cq, f"bumper_block_{i}"),
            origin=Origin(
                xyz=(0.0, 0.0, -0.053 - 0.045)  # top of boss at joint, body hangs down
            ),
            material=rubber,
            name=f"bumper_body",
        )

    # ------------------------------------------------------ joints ---
    # Main pivot: revolute, axis along Y, ±20°
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

    # Bumper prismatic joints: vertical compression
    for i, side in enumerate((1.0, -1.0)):
        model.articulation(
            f"bumper_{i}_slide",
            ArticulationType.PRISMATIC,
            parent=beam,
            child=f"bumper_{i}",
            origin=Origin(
                xyz=(side * BUMPER_X, 0.0, BAR_BOT - 0.008)
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=80.0,
                velocity=0.5,
                lower=0.0,
                upper=BUMPER_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    beam = object_model.get_part("beam")
    bumper_0 = object_model.get_part("bumper_0")
    bumper_1 = object_model.get_part("bumper_1")
    pivot = object_model.get_articulation("beam_pivot")
    slide_0 = object_model.get_articulation("bumper_0_slide")
    slide_1 = object_model.get_articulation("bumper_1_slide")

    # Pivot sleeve captures the axle bolt (intentional nesting).
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

    # Coil spring wraps around the pivot area, encircling the sleeve and collar (intentional).
    ctx.allow_overlap(
        base,
        beam,
        elem_a="coil_spring",
        elem_b="pivot_sleeve",
        reason="The coil spring encircles the pivot mechanism; it intentionally overlaps the sleeve.",
    )
    ctx.allow_overlap(
        base,
        beam,
        elem_a="coil_spring",
        elem_b="pivot_collar",
        reason="The coil spring encircles the pivot mechanism; it intentionally overlaps the collar.",
    )
    ctx.expect_overlap(
        base,
        beam,
        axes="xy",
        elem_a="coil_spring",
        elem_b="pivot_sleeve",
        min_overlap=0.020,
        name="coil spring surrounds the pivot sleeve in XY",
    )

    # Guide rails pass through the bumper body for sliding (intentional nesting).
    for i in range(2):
        for j in range(2):
            ctx.allow_overlap(
                beam,
                f"bumper_{i}",
                elem_a=f"guide_rail_{i}_{j}",
                elem_b="bumper_body",
                reason=f"Guide rail passes through the bumper mounting plate for prismatic sliding.",
            )
        ctx.expect_overlap(
            beam,
            f"bumper_{i}",
            axes="z",
            elem_a=f"guide_rail_{i}_0",
            elem_b="bumper_body",
            min_overlap=0.005,
            name=f"bumper_{i} body overlaps guide rail on Z axis (retained on rail)",
        )

    # Beam bar clears the pedestal top
    ctx.expect_gap(
        beam,
        base,
        axis="z",
        positive_elem="beam_bar",
        negative_elem="pivot_housing",
        min_gap=0.002,
        max_gap=0.12,
        name="beam bar clears the pivot housing",
    )

    # Joint configuration checks
    ax = pivot.axis
    ctx.check(
        "pivot axis is horizontal and perpendicular to beam",
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

    # Bumper prismatic joints: axis and travel
    for slide in (slide_0, slide_1):
        sa = slide.axis
        ctx.check(
            f"{slide.name} axis is vertical (Z)",
            abs(sa[0]) < 1e-9 and abs(sa[1]) < 1e-9 and abs(sa[2] - 1.0) < 1e-9,
            details=f"axis={sa}",
        )
        sl = slide.motion_limits
        ctx.check(
            f"{slide.name} has short compression travel",
            sl is not None
            and sl.lower is not None
            and sl.upper is not None
            and abs(sl.lower) < 1e-6
            and 0.02 <= sl.upper <= 0.06,
            details=f"limits=({sl.lower}, {sl.upper})",
        )

    # Hero geometry: scale, base grounded, pivot height
    bar_box = ctx.part_element_world_aabb(beam, elem="beam_bar")
    base_box = ctx.part_world_aabb(base)
    ctx.check(
        "beam is about 3.0 m long",
        bar_box is not None and abs((bar_box[1][0] - bar_box[0][0]) - 3.0) < 0.05,
        details=f"bar aabb={bar_box}",
    )
    ctx.check(
        "base rests on the ground",
        base_box is not None and -0.01 <= base_box[0][2] <= 0.02,
        details=f"base aabb={base_box}",
    )

    # Molded seats: raised lips extend above beam bar top
    for i in range(2):
        seat_box = ctx.part_element_world_aabb(beam, elem=f"seat_{i}")
        ctx.check(
            f"seat_{i} has raised lips above beam bar",
            seat_box is not None
            and bar_box is not None
            and seat_box[1][2] > bar_box[1][2] + 0.020  # lips rise ≥20mm above bar
            and seat_box[0][2] < bar_box[1][2] + 0.005,  # seat floor near bar top
            details=f"seat aabb={seat_box}, bar top={bar_box[1][2] if bar_box else None}",
        )

    # Bumpers hang below the beam and are near the beam tips
    for i in range(2):
        bumper_box = ctx.part_world_aabb(f"bumper_{i}")
        ctx.check(
            f"bumper_{i} hangs below the beam",
            bumper_box is not None
            and bar_box is not None
            and bumper_box[0][2] < bar_box[0][2],
            details=f"bumper aabb={bumper_box}",
        )

    # Bumper compression: prismatic joint moves bumper upward
    rest_box_0 = ctx.part_world_aabb(bumper_0)
    with ctx.pose({slide_0: BUMPER_TRAVEL}):
        compressed_box_0 = ctx.part_world_aabb(bumper_0)
        ctx.check(
            "bumper_0 compresses upward on prismatic joint",
            rest_box_0 is not None
            and compressed_box_0 is not None
            and compressed_box_0[0][2] > rest_box_0[0][2] + 0.02,
            details=f"rest={rest_box_0}, compressed={compressed_box_0}",
        )

    # Rocking pose: positive tilt lowers +X end
    rest_b0 = ctx.part_element_world_aabb(beam, elem="seat_0")
    with ctx.pose({pivot: TILT}):
        down_s0 = ctx.part_element_world_aabb(beam, elem="seat_0")
        up_s1 = ctx.part_element_world_aabb(beam, elem="seat_1")
        ctx.check(
            "positive rock lowers the +X seat",
            rest_b0 is not None
            and down_s0 is not None
            and down_s0[0][2] < rest_b0[0][2] - 0.30,
            details=f"rest={rest_b0}, tilted={down_s0}",
        )
        ctx.check(
            "positive rock raises the -X seat",
            up_s1 is not None and up_s1[0][2] > 1.0,
            details=f"raised seat aabb={up_s1}",
        )

    return ctx.report()


object_model = build_object_model()
