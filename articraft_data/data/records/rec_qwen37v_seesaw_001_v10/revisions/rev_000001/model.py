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
# Variant-10 vintage playground seesaw
#
# World frame: beam runs along X, pivot axis along Y, Z up.
# - Two bent galvanized-steel tube arches (~50 mm dia) cross side by side and
#   form an A-shaped saddle; the apex carries a horizontal pivot axle bolt.
# - Rubber ground pads sit under each arch foot for stability.
# - The rocking beam is a 3.0 m mustard-yellow steel bar (80 x 40 mm) with a
#   pivot sleeve + triangular gusset at center.
# - Asymmetric seats: +X end seat at standard height, -X end seat raised on a
#   steel riser block (for riders of different sizes).
# - Each grab handle is an inverted-U bent rod on its own revolute pivot
#   (±12° forward/backward tilt).
# - Tire-section bumpers under each tip, plus safety bump stops below each
#   beam end to limit ground-contact travel.
# - Single revolute joint at the apex, axis (0, 1, 0), +/- 20 degrees.
#   Positive q lowers the +X end (right-hand rule about +Y).
# ---------------------------------------------------------------------------

BEAM_HALF = 1.50  # 3.0 m beam
BEAM_W = 0.08
BEAM_T = 0.04
PIVOT_Z = 0.76  # axle height (about 0.8 m tall at the pivot)

ARCH_FOOT_X = 0.66
ARCH_FOOT_Y = 0.34
ARCH_APEX_Y = 0.05  # each arch crosses past center so the pair reads as an A
ARCH_FOOT_Z = 0.028
TUBE_R = 0.025  # ~50 mm diameter bent tube

AXLE_R = 0.016
AXLE_LEN = 0.22

# Beam-local frame: origin at the axle center; the bar bottom sits 50 mm above.
BAR_BOT = 0.05
BAR_CTR = BAR_BOT + BEAM_T / 2.0  # 0.07
BAR_TOP = BAR_BOT + BEAM_T  # 0.09

SEAT_X = 1.30
HANDLE_X = 1.04
BUMPER_X = 1.42
BUMP_STOP_X = 1.36
TILT = math.radians(20.0)

# Asymmetric seat heights: +X seat standard, -X seat raised ~40 mm on a riser.
SEAT_RISER_H = 0.040
HANDLE_TILT = math.radians(12.0)  # handlebar pivot range


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


def _handle_points_local() -> list[tuple[float, float, float]]:
    """Inverted-U grab-handle rod centerline in handle-local frame.

    Origin is at the beam top surface where the handle mounts.
    The rod extends upward in local +Z and across in local Y.
    """
    half_w = 0.035
    leg_bot = 0.000  # rod sits flush on the beam top surface
    arc_top_local = 0.275 - BAR_TOP  # ~0.185 above the beam top
    leg_bend = 0.190 - BAR_TOP  # ~0.100
    pts: list[tuple[float, float, float]] = [
        (0.0, -half_w, leg_bot),
        (0.0, -half_w, leg_bend),
    ]
    for k in range(7):  # semicircular top bend
        a = math.pi * k / 6.0
        pts.append((0.0, -half_w * math.cos(a), arc_top_local + half_w * math.sin(a) * 0.5))
    pts.append((0.0, half_w, leg_bend))
    pts.append((0.0, half_w, leg_bot))
    return pts


def _bumper_geometry(x: float, index: int):
    """Curved tire-section bumper: half-annulus shell extruded across the beam."""
    r_out = 0.065
    r_in = 0.048
    profile: list[tuple[float, float]] = []
    n = 20
    for k in range(n + 1):  # outer arc, bottom half (pi .. 2*pi)
        a = math.pi + math.pi * k / n
        profile.append((r_out * math.cos(a), r_out * math.sin(a)))
    for k in range(n + 1):  # inner arc back (2*pi .. pi)
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
    model = ArticulatedObject(name="vintage_playground_seesaw_v10")

    galvanized = model.material("weathered_galvanized", rgba=(0.55, 0.58, 0.56, 1.0))
    rust = model.material("rust_steel", rgba=(0.42, 0.25, 0.13, 1.0))
    mustard = model.material("rusty_mustard_paint", rgba=(0.74, 0.53, 0.12, 1.0))
    pale_steel = model.material("pale_weathered_steel", rgba=(0.70, 0.66, 0.58, 1.0))
    wood = model.material("worn_wood", rgba=(0.60, 0.45, 0.28, 1.0))
    rubber = model.material("black_rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    dark_rubber = model.material("ground_pad_rubber", rgba=(0.12, 0.10, 0.09, 1.0))

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

    # Pivot axle bolt through both flattened arch apexes, axis along Y.
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

    # Rubber ground pads under each arch foot (4 feet total).
    pad_r = 0.050
    pad_h = 0.016  # contacts arch tube bottom (rubber compressed ~2 mm)
    foot_idx = 0
    for side in (1.0, -1.0):
        for sx in (-1.0, 1.0):
            fx = sx * ARCH_FOOT_X
            fy = side * ARCH_FOOT_Y
            base.visual(
                Cylinder(radius=pad_r, length=pad_h),
                origin=Origin(xyz=(fx, fy, pad_h / 2.0)),
                material=dark_rubber,
                name=f"ground_pad_{foot_idx}",
            )
            foot_idx += 1

    # --------------------------------------------------------------- beam ---
    beam = model.part("beam")

    beam.visual(
        Cylinder(radius=0.026, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rust,
        name="pivot_sleeve",
    )
    beam.visual(_gusset_geometry(), material=mustard, name="gusset_plate")

    beam.visual(
        Box((2.0 * BEAM_HALF, BEAM_W, BEAM_T)),
        origin=Origin(xyz=(0.0, 0.0, BAR_CTR)),
        material=mustard,
        name="beam_bar",
    )

    # Rust streak patches wrapping the painted bar.
    for i, px in enumerate((-0.85, -0.30, 0.45, 0.95)):
        beam.visual(
            Box((0.16, BEAM_W + 0.004, 0.012)),
            origin=Origin(xyz=(px, 0.0, BAR_TOP - 0.004)),
            material=rust,
            name=f"rust_patch_{i}",
        )

    # --- Asymmetric seats ---
    # +X end: standard height seat (seat_0).
    beam.visual(
        Box((0.30, 0.24, 0.022)),
        origin=Origin(xyz=(SEAT_X, 0.0, BAR_TOP + 0.008)),
        material=wood,
        name="seat_0",
    )
    # -X end: raised seat on a steel riser block (seat_1).
    beam.visual(
        Box((0.28, 0.22, SEAT_RISER_H)),
        origin=Origin(xyz=(-SEAT_X, 0.0, BAR_TOP + SEAT_RISER_H / 2.0)),
        material=pale_steel,
        name="seat_riser",
    )
    beam.visual(
        Box((0.30, 0.24, 0.022)),
        origin=Origin(xyz=(-SEAT_X, 0.0, BAR_TOP + SEAT_RISER_H + 0.008)),
        material=wood,
        name="seat_1",
    )

    # --- Tire-section bumpers under each tip ---
    for i, side in enumerate((1.0, -1.0)):
        beam.visual(
            _bumper_geometry(side * BUMPER_X, i),
            material=rubber,
            name=f"bumper_{i}",
        )

    # --- Safety bump stops below each beam end ---
    bump_w = 0.10
    bump_d = 0.06
    bump_h = 0.040
    for i, side in enumerate((1.0, -1.0)):
        beam.visual(
            Box((bump_w, bump_d, bump_h)),
            origin=Origin(xyz=(side * BUMP_STOP_X, 0.0, BAR_BOT - bump_h / 2.0 - 0.004)),
            material=rubber,
            name=f"bump_stop_{i}",
        )

    # -------------------------------------------------------- handle parts ---
    # Each handle is a separate part with a revolute joint for slight pivot.
    for i, side in enumerate((1.0, -1.0)):
        handle = model.part(f"handle_{i}")
        handle.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _handle_points_local(),
                    radius=0.009,
                    samples_per_segment=8,
                    radial_segments=16,
                    cap_ends=True,
                ),
                f"seesaw_handle_{i}",
            ),
            material=pale_steel,
            name=f"handle_bar_{i}",
        )

        # Revolute joint: handle pivots forward/backward about Y axis.
        model.articulation(
            f"handle_pivot_{i}",
            ArticulationType.REVOLUTE,
            parent=beam,
            child=handle,
            origin=Origin(xyz=(side * HANDLE_X, 0.0, BAR_TOP)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=2.0,
                lower=-HANDLE_TILT,
                upper=HANDLE_TILT,
            ),
        )

    # -------------------------------------------------------- main pivot ---
    model.articulation(
        "beam_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),  # positive q lowers the +X end
        motion_limits=MotionLimits(effort=200.0, velocity=2.5, lower=-TILT, upper=TILT),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("arched_base")
    beam = object_model.get_part("beam")
    handle_0 = object_model.get_part("handle_0")
    handle_1 = object_model.get_part("handle_1")
    pivot = object_model.get_articulation("beam_pivot")
    handle_pivot_0 = object_model.get_articulation("handle_pivot_0")
    handle_pivot_1 = object_model.get_articulation("handle_pivot_1")

    # ---- Pivot sleeve / axle ----
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

    # ---- Handle rod legs welded onto beam bar (small local embed) ----
    for i in range(2):
        ctx.allow_overlap(
            beam,
            object_model.get_part(f"handle_{i}"),
            elem_a="beam_bar",
            elem_b=f"handle_bar_{i}",
            reason=f"Handle rod legs are welded onto the beam bar top surface; tube radius creates small local contact overlap.",
        )
        ctx.expect_contact(
            beam,
            object_model.get_part(f"handle_{i}"),
            elem_a="beam_bar",
            elem_b=f"handle_bar_{i}",
            name=f"handle_{i} rod contacts the beam bar top",
        )

    # ---- Beam bar clearance above arch saddle ----
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

    # ---- Main pivot joint configuration ----
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

    # ---- Hero geometry: scale, saddle height, grounded feet ----
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

    # ---- Variant 10: Asymmetric seat heights ----
    seat_0_box = ctx.part_element_world_aabb(beam, elem="seat_0")
    seat_1_box = ctx.part_element_world_aabb(beam, elem="seat_1")
    riser_box = ctx.part_element_world_aabb(beam, elem="seat_riser")
    ctx.check(
        "seat_0 is at standard height on the beam",
        seat_0_box is not None
        and bar_box is not None
        and bar_box[0][2] < seat_0_box[0][2] < bar_box[1][2]
        and seat_0_box[1][2] > bar_box[1][2],
        details=f"seat_0 aabb={seat_0_box}",
    )
    ctx.check(
        "seat_1 is raised above seat_0 (asymmetric heights)",
        seat_0_box is not None
        and seat_1_box is not None
        and seat_1_box[0][2] > seat_0_box[0][2] + 0.02,
        details=f"seat_0_z={seat_0_box[0][2]}, seat_1_z={seat_1_box[0][2]}",
    )
    ctx.check(
        "seat riser sits between beam bar top and raised seat",
        riser_box is not None
        and bar_box is not None
        and seat_1_box is not None
        and riser_box[0][2] >= bar_box[1][2] - 0.005
        and riser_box[1][2] <= seat_1_box[0][2] + 0.005,
        details=f"riser={riser_box}, seat_1={seat_1_box}",
    )

    # ---- Variant 10: Ground pads under support legs ----
    for pad_i in range(4):
        pad_box = ctx.part_element_world_aabb(base, elem=f"ground_pad_{pad_i}")
        ctx.check(
            f"ground_pad_{pad_i} sits at ground level under an arch foot",
            pad_box is not None
            and pad_box[0][2] < 0.015
            and pad_box[1][2] < 0.03,
            details=f"pad aabb={pad_box}",
        )

    # ---- Variant 10: Safety bump stops below beam ends ----
    for i in range(2):
        bs_box = ctx.part_element_world_aabb(beam, elem=f"bump_stop_{i}")
        ctx.check(
            f"bump_stop_{i} hangs below the beam bar near the tip",
            bs_box is not None
            and bar_box is not None
            and bs_box[1][2] < bar_box[0][2] + 0.005
            and bs_box[0][2] < bar_box[0][2],
            details=f"bump_stop aabb={bs_box}",
        )

    # ---- Variant 10: Handlebar revolute joints ----
    for i, hp in enumerate((handle_pivot_0, handle_pivot_1)):
        hax = hp.axis
        ctx.check(
            f"handle_pivot_{i} axis is horizontal (Y)",
            abs(hax[0]) < 1e-9 and abs(hax[1] - 1.0) < 1e-9 and abs(hax[2]) < 1e-9,
            details=f"axis={hax}",
        )
        hlim = hp.motion_limits
        ctx.check(
            f"handle_pivot_{i} has small symmetric tilt limits",
            hlim is not None
            and hlim.lower is not None
            and hlim.upper is not None
            and hlim.lower < 0.0
            and hlim.upper > 0.0
            and abs(hlim.upper) < math.radians(20.0),
            details=f"limits=({hlim.lower}, {hlim.upper})",
        )

    # Handle bars stand above the beam at rest (rooted at beam top surface).
    for i in range(2):
        handle_bar_box = ctx.part_element_world_aabb(
            object_model.get_part(f"handle_{i}"), elem=f"handle_bar_{i}"
        )
        ctx.check(
            f"handle_bar_{i} extends above the beam",
            handle_bar_box is not None
            and bar_box is not None
            and handle_bar_box[1][2] > bar_box[1][2] + 0.15
            and handle_bar_box[0][2] <= bar_box[1][2] + 0.005,
            details=f"handle aabb={handle_bar_box}",
        )

    # Handle pivot pose check: tilting the handle moves its top.
    h0_rest = ctx.part_element_world_aabb(handle_0, elem="handle_bar_0")
    with ctx.pose({handle_pivot_0: HANDLE_TILT}):
        h0_tilted = ctx.part_element_world_aabb(handle_0, elem="handle_bar_0")
        ctx.check(
            "handle_0 tilts when its revolute joint is posed",
            h0_rest is not None
            and h0_tilted is not None
            and abs(h0_tilted[1][0] - h0_rest[1][0]) > 0.005,
            details=f"rest={h0_rest}, tilted={h0_tilted}",
        )

    # ---- Per-end fittings: bumpers ----
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

    # ---- Decisive pose checks: rocking alternately lowers each end ----
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
