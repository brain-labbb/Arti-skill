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
# Vintage playground seesaw – variant 30
#
# World frame: beam runs along X, pivot axis along Y, Z up.
# - Two bent galvanized-steel tube arches (~50 mm dia) form an A-shaped saddle.
# - Rocking beam: 3.0 m mustard-yellow steel bar (80 x 40 mm).
# - ASYMMETRIC SEATS: +X end has a steel riser block lifting the seat 35 mm
#   above the bar; -X end has a standard low seat directly on the bar.
# - CENTRAL SPRING: helical coil spring under the beam on a prismatic joint
#   (axis Z, 0 to 10 mm compression travel).
# - RUBBER GROUND PADS: flat rubber disks under each arch foot.
# - Single revolute joint at the apex, axis (0, 1, 0), +/- 20 degrees.
# ---------------------------------------------------------------------------

BEAM_HALF = 1.50  # 3.0 m beam
BEAM_W = 0.08
BEAM_T = 0.04
PIVOT_Z = 0.76  # axle height

ARCH_FOOT_X = 0.66
ARCH_FOOT_Y = 0.34
ARCH_APEX_Y = 0.05
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
TILT = math.radians(20.0)

# --- Variant 30 constants ---
SEAT_RISER_H = 0.035  # riser block height for the high seat
SEAT_THICK = 0.022  # wooden seat plate thickness

SPRING_X = 0.12  # spring offset from center (clears pivot hardware)
SPRING_COIL_R = 0.020
SPRING_WIRE_R = 0.004
SPRING_TURNS = 4
SPRING_FREE_H = 0.040  # free (uncompressed) coil height
SPRING_COMPRESS_MAX = 0.010  # max prismatic compression travel

PAD_R = 0.050  # ground pad radius
PAD_T = 0.012  # ground pad thickness


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
    """Curved tire-section bumper: half-annulus shell extruded across the beam."""
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


def _spring_coil_points() -> list[tuple[float, float, float]]:
    """Helical coil spring centerline, extending downward through both end plates."""
    pts: list[tuple[float, float, float]] = []
    coil_top = -0.002  # wire top enters the top plate for connectivity
    n_pts = SPRING_TURNS * 16 + 1
    for i in range(n_pts):
        t = i / (n_pts - 1)
        a = 2.0 * math.pi * SPRING_TURNS * t
        x = SPRING_COIL_R * math.cos(a)
        y = SPRING_COIL_R * math.sin(a)
        z = coil_top - SPRING_FREE_H * t
        pts.append((x, y, z))
    return pts


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_playground_seesaw_v30")

    galvanized = model.material("weathered_galvanized", rgba=(0.55, 0.58, 0.56, 1.0))
    rust = model.material("rust_steel", rgba=(0.42, 0.25, 0.13, 1.0))
    mustard = model.material("rusty_mustard_paint", rgba=(0.74, 0.53, 0.12, 1.0))
    pale_steel = model.material("pale_weathered_steel", rgba=(0.70, 0.66, 0.58, 1.0))
    wood = model.material("worn_wood", rgba=(0.60, 0.45, 0.28, 1.0))
    rubber = model.material("black_rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    spring_steel = model.material("spring_steel", rgba=(0.45, 0.48, 0.50, 1.0))

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

    # Rubber ground pads under each arch foot (4 pads total).
    # The arch tube lowest surface is at ~0.0135 m (from probe), not at
    # ARCH_FOOT_Z - TUBE_R, because the tube tangent tilts upward at the feet.
    ARCH_TUBE_BOTTOM_Z = 0.0135
    pad_positions = [
        (-ARCH_FOOT_X, ARCH_FOOT_Y),   # arch 0, -X foot
        (ARCH_FOOT_X, ARCH_FOOT_Y),    # arch 0, +X foot
        (-ARCH_FOOT_X, -ARCH_FOOT_Y),  # arch 1, -X foot
        (ARCH_FOOT_X, -ARCH_FOOT_Y),   # arch 1, +X foot
    ]
    for i, (fx, fy) in enumerate(pad_positions):
        # Pad top embeds slightly into the arch tube bottom for connectivity.
        pad_z = ARCH_TUBE_BOTTOM_Z - PAD_T / 2.0 + 0.002
        base.visual(
            Cylinder(radius=PAD_R, length=PAD_T),
            origin=Origin(xyz=(fx, fy, pad_z)),
            material=rubber,
            name=f"ground_pad_{i}",
        )

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

    # ---- Asymmetric seats ----
    # seat_0 (+X end): HIGH seat with steel riser block
    beam.visual(
        Box((0.22, 0.20, SEAT_RISER_H)),
        origin=Origin(xyz=(SEAT_X, 0.0, BAR_TOP + SEAT_RISER_H / 2.0)),
        material=pale_steel,
        name="seat_riser_0",
    )
    beam.visual(
        Box((0.30, 0.24, SEAT_THICK)),
        origin=Origin(xyz=(SEAT_X, 0.0, BAR_TOP + SEAT_RISER_H + SEAT_THICK / 2.0)),
        material=wood,
        name="seat_0",
    )

    # seat_1 (-X end): standard LOW seat directly on the bar
    beam.visual(
        Box((0.30, 0.24, SEAT_THICK)),
        origin=Origin(xyz=(-SEAT_X, 0.0, BAR_TOP + SEAT_THICK / 2.0)),
        material=wood,
        name="seat_1",
    )

    # Grab handles (same height on both sides).
    for i, side in enumerate((1.0, -1.0)):
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

    # Tire-section bumpers.
    for i, side in enumerate((1.0, -1.0)):
        beam.visual(
            _bumper_geometry(side * BUMPER_X, i),
            material=rubber,
            name=f"bumper_{i}",
        )

    # ------------------------------------------------------------- spring ---
    spring = model.part("spring")

    # Helical coil
    spring.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                _spring_coil_points(),
                radius=SPRING_WIRE_R,
                samples_per_segment=8,
                radial_segments=12,
                cap_ends=True,
            ),
            "spring_coil_mesh",
        ),
        material=spring_steel,
        name="spring_coil",
    )
    # Top plate (spring seat pressed against beam underside; top embeds into bar)
    spring.visual(
        Cylinder(radius=0.024, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=spring_steel,
        name="spring_top_plate",
    )
    # Bottom plate (ground-facing spring seat; coil wire enters it)
    spring.visual(
        Cylinder(radius=0.024, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, -0.047)),
        material=spring_steel,
        name="spring_bottom_plate",
    )

    # -------------------------------------------------------------- joints ---
    model.articulation(
        "beam_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),  # positive q lowers the +X end
        motion_limits=MotionLimits(
            effort=200.0, velocity=2.5, lower=-TILT, upper=TILT,
        ),
    )

    model.articulation(
        "spring_compress",
        ArticulationType.PRISMATIC,
        parent=beam,
        child=spring,
        origin=Origin(xyz=(SPRING_X, 0.0, BAR_BOT)),
        axis=(0.0, 0.0, 1.0),  # positive q = upward = compression
        motion_limits=MotionLimits(
            effort=500.0, velocity=0.10, lower=0.0, upper=SPRING_COMPRESS_MAX,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("arched_base")
    beam = object_model.get_part("beam")
    spring = object_model.get_part("spring")
    pivot = object_model.get_articulation("beam_pivot")
    spring_joint = object_model.get_articulation("spring_compress")

    # --- Pivot sleeve captures the axle bolt ---
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

    # --- Spring coil/plates may enter beam socket during compression ---
    ctx.allow_overlap(
        spring,
        beam,
        elem_a="spring_coil",
        elem_b="beam_bar",
        reason="Spring coil top enters the beam socket recess during compression travel.",
    )
    ctx.allow_overlap(
        spring,
        beam,
        elem_a="spring_top_plate",
        elem_b="beam_bar",
        reason="Spring top plate enters the beam socket recess during compression travel.",
    )

    # Proof: at rest, spring coil is at or below the beam bar (wire may contact).
    ctx.expect_gap(
        beam,
        spring,
        axis="z",
        positive_elem="beam_bar",
        negative_elem="spring_coil",
        max_penetration=0.002,
        name="spring coil stays near beam bar underside at rest",
    )

    # --- Ground pads are seated against the arch tube feet ---
    for i, arch_idx in enumerate((0, 0, 1, 1)):
        ctx.allow_overlap(
            base,
            base,
            elem_a=f"ground_pad_{i}",
            elem_b=f"arch_{arch_idx}",
            reason=f"Ground pad {i} is pressed into the arch tube foot for secure ground contact.",
        )
        ctx.expect_contact(
            base,
            base,
            elem_a=f"ground_pad_{i}",
            elem_b=f"arch_{arch_idx}",
            name=f"ground_pad_{i} contacts arch_{arch_idx} foot",
        )

    # --- Beam bar clears the arch saddle ---
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

    # --- Pivot axis and rocking limits ---
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

    # --- Hero geometry ---
    bar_box = ctx.part_element_world_aabb(beam, elem="beam_bar")
    base_box = ctx.part_world_aabb(base)
    axle_box = ctx.part_element_world_aabb(base, elem="pivot_axle")
    ctx.check(
        "beam is about 3.0 m long",
        bar_box is not None and abs((bar_box[1][0] - bar_box[0][0]) - 3.0) < 0.02,
        details=f"bar aabb={bar_box}",
    )
    ctx.check(
        "arched base rests near the ground (pads may extend slightly below)",
        base_box is not None and -0.02 <= base_box[0][2] <= 0.02,
        details=f"base aabb={base_box}",
    )
    ctx.check(
        "pivot axle sits about 0.8 m high",
        axle_box is not None and 0.70 <= (axle_box[0][2] + axle_box[1][2]) / 2.0 <= 0.82,
        details=f"axle aabb={axle_box}",
    )

    # --- Asymmetric seat heights ---
    seat_0_box = ctx.part_element_world_aabb(beam, elem="seat_0")
    seat_1_box = ctx.part_element_world_aabb(beam, elem="seat_1")
    ctx.check(
        "seat_0 (high side) top is clearly above seat_1 (low side) top",
        seat_0_box is not None
        and seat_1_box is not None
        and seat_0_box[1][2] > seat_1_box[1][2] + 0.020,
        details=(
            f"seat_0 top={seat_0_box[1][2] if seat_0_box else None}, "
            f"seat_1 top={seat_1_box[1][2] if seat_1_box else None}"
        ),
    )
    ctx.check(
        "high seat has a visible riser block",
        "seat_riser_0" in [v.name for v in beam.visuals],
        details="seat_riser_0 visual not found on beam",
    )

    # --- Spring prismatic joint ---
    ctx.check(
        "spring joint is prismatic",
        spring_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={spring_joint.articulation_type}",
    )
    sj_ax = spring_joint.axis
    ctx.check(
        "spring joint axis is vertical (Z)",
        abs(sj_ax[0]) < 1e-9 and abs(sj_ax[1]) < 1e-9 and abs(sj_ax[2] - 1.0) < 1e-9,
        details=f"axis={sj_ax}",
    )
    sj_lim = spring_joint.motion_limits
    ctx.check(
        "spring has non-trivial compression travel limits",
        sj_lim is not None
        and sj_lim.lower is not None
        and sj_lim.upper is not None
        and sj_lim.lower >= 0.0
        and sj_lim.upper > 0.005,
        details=f"limits=({sj_lim.lower}, {sj_lim.upper})",
    )

    # Spring coil hangs below the beam bar
    spring_coil_box = ctx.part_element_world_aabb(spring, elem="spring_coil")
    ctx.check(
        "spring coil hangs below the beam bar",
        spring_coil_box is not None
        and bar_box is not None
        and spring_coil_box[1][2] < bar_box[0][2] + 0.005,
        details=f"spring coil aabb={spring_coil_box}",
    )

    # Spring compression pose: bottom plate moves up when compressed
    spring_bot_rest = ctx.part_element_world_aabb(spring, elem="spring_bottom_plate")
    with ctx.pose({spring_joint: SPRING_COMPRESS_MAX}):
        spring_bot_compressed = ctx.part_element_world_aabb(
            spring, elem="spring_bottom_plate"
        )
        ctx.check(
            "spring bottom plate moves upward when compressed",
            spring_bot_rest is not None
            and spring_bot_compressed is not None
            and spring_bot_compressed[0][2] > spring_bot_rest[0][2] + 0.003,
            details=f"rest={spring_bot_rest}, compressed={spring_bot_compressed}",
        )

    # --- Ground pads ---
    for i in range(4):
        pad_box = ctx.part_element_world_aabb(base, elem=f"ground_pad_{i}")
        ctx.check(
            f"ground_pad_{i} exists at ground level under an arch foot",
            pad_box is not None and -0.02 <= pad_box[0][2] <= 0.01,
            details=f"pad aabb={pad_box}",
        )

    # --- Handles and bumpers ---
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

    # --- Decisive rocking pose checks ---
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
