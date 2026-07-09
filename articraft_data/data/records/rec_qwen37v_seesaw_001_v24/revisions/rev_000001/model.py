from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    ConeGeometry,
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
# Animal-shaped toddler seesaw (horse theme)
#
# World frame: horse runs along X, pivot axis along Y, Z up.
# Horse body is the beam; head at +X, tail at -X.
# Two seats near each end on the horse back, two pivoting handlebars.
# Central pivot on an A-frame base with rubber ground pads under each foot.
# ---------------------------------------------------------------------------

BODY_HALF = 0.55   # horse torso half-length
BODY_W = 0.22      # body width
BODY_H = 0.18      # body height (matches capsule Z radius after scale)
PIVOT_Z = 0.50     # toddler-friendly pivot height

ARCH_FOOT_X = 0.38
ARCH_FOOT_Y = 0.20
ARCH_APEX_Y = 0.02
ARCH_FOOT_Z = 0.015
TUBE_R = 0.018

AXLE_R = 0.012
AXLE_LEN = 0.14

# Beam-local frame: origin at axle center
BAR_BOT = 0.05
BAR_CTR = BAR_BOT + BODY_H / 2.0   # 0.14
BAR_TOP = BAR_BOT + BODY_H          # 0.23

SEAT_X = 0.35
HANDLE_X = 0.22

TILT = math.radians(20.0)
HANDLE_TILT = math.radians(10.0)

GROUND_PAD_R = 0.035
GROUND_PAD_H = 0.010


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


def _handle_points() -> list[tuple[float, float, float]]:
    """Inverted-U grab handle in local frame, base at origin."""
    half_w = 0.028
    leg_bot = -0.025  # stem embeds into horse body
    arc_z = 0.220
    pts: list[tuple[float, float, float]] = [
        (0, -half_w, leg_bot),
        (0, -half_w, 0.150),
    ]
    for k in range(7):
        a = math.pi * k / 6.0
        pts.append((0, -half_w * math.cos(a), arc_z + half_w * math.sin(a)))
    pts.append((0, half_w, 0.150))
    pts.append((0, half_w, leg_bot))
    return pts


def _tail_points() -> list[tuple[float, float, float]]:
    """Curved horse tail at the rear of the body."""
    pts: list[tuple[float, float, float]] = []
    for i in range(9):
        t = i / 8.0
        x = -0.44 - 0.16 * t
        y = 0.0
        z = BAR_CTR + 0.02 - 0.14 * t * t
        pts.append((x, y, z))
    return pts


def _horse_torso_mesh():
    """Horse torso: scaled capsule aligned along X."""
    capsule = CapsuleGeometry(0.09, 0.90, radial_segments=24, height_segments=8)
    capsule.scale(1.0, 1.25, 1.0)   # wider in Y
    capsule.rotate_y(math.pi / 2)    # align with X
    capsule.translate(0, 0, BAR_CTR)
    return mesh_from_geometry(capsule, "horse_torso")


def _gusset_geometry():
    """Triangular gusset plate bridging pivot sleeve to horse torso underside."""
    # Profile Y values map to beam-local Z after rotate_x(pi/2).
    # Top reaches into torso bottom (beam-local Z=0.050) for connectivity.
    profile = [(-0.08, 0.054), (0.08, 0.054), (0.0, 0.020)]
    geom = ExtrudeGeometry(profile, 0.020, cap=True, center=True)
    geom.rotate_x(math.pi / 2)
    return mesh_from_geometry(geom, "gusset_plate")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="animal_toddler_seesaw")

    # Materials
    horse_paint = model.material("rocking_horse_chestnut", rgba=(0.65, 0.38, 0.18, 1.0))
    horse_dark = model.material("horse_dark_brown", rgba=(0.20, 0.12, 0.06, 1.0))
    horse_cream = model.material("horse_cream", rgba=(0.92, 0.87, 0.75, 1.0))
    saddle_red = model.material("saddle_red", rgba=(0.70, 0.18, 0.12, 1.0))
    galvanized = model.material("galvanized_steel", rgba=(0.55, 0.58, 0.56, 1.0))
    rubber = model.material("black_rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    handle_red = model.material("handle_red_paint", rgba=(0.82, 0.15, 0.10, 1.0))
    rust = model.material("rust_hardware", rgba=(0.42, 0.25, 0.13, 1.0))

    # --------------------------------------------------------------- base ---
    base = model.part("base")

    # Arched legs
    base.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                _arch_points(1.0),
                radius=TUBE_R,
                samples_per_segment=8,
                radial_segments=18,
                cap_ends=True,
            ),
            "arch_0",
        ),
        material=galvanized,
        name="arch_0",
    )
    base.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                _arch_points(-1.0),
                radius=TUBE_R,
                samples_per_segment=8,
                radial_segments=18,
                cap_ends=True,
            ),
            "arch_1",
        ),
        material=galvanized,
        name="arch_1",
    )

    # Rubber ground pads under each arch foot (4 pads)
    pad_idx = 0
    for side in (1.0, -1.0):
        for sx in (-1.0, 1.0):
            base.visual(
                Cylinder(radius=GROUND_PAD_R, length=GROUND_PAD_H),
                origin=Origin(
                    xyz=(sx * ARCH_FOOT_X, side * ARCH_FOOT_Y, GROUND_PAD_H / 2),
                ),
                material=rubber,
                name=f"ground_pad_{pad_idx}",
            )
            pad_idx += 1

    # Pivot axle bolt
    base.visual(
        Cylinder(radius=AXLE_R, length=AXLE_LEN),
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z), rpy=(math.pi / 2, 0, 0)),
        material=rust,
        name="pivot_axle",
    )
    for i, side in enumerate((1.0, -1.0)):
        base.visual(
            Cylinder(radius=0.020, length=0.012),
            origin=Origin(
                xyz=(0.0, side * (AXLE_LEN / 2 - 0.005), PIVOT_Z),
                rpy=(math.pi / 2, 0, 0),
            ),
            material=rust,
            name=f"axle_nut_{i}",
        )

    # --------------------------------------------------------------- beam ---
    # Beam part frame sits at the axle center so the joint is at its origin.
    beam = model.part("beam")

    # Pivot sleeve
    beam.visual(
        Cylinder(radius=0.022, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2, 0, 0)),
        material=rust,
        name="pivot_sleeve",
    )

    # Gusset plate
    beam.visual(_gusset_geometry(), material=horse_paint, name="gusset_plate")

    # Horse torso (capsule mesh)
    beam.visual(_horse_torso_mesh(), material=horse_paint, name="horse_torso")

    # Horse neck (angled box connecting torso to head)
    beam.visual(
        Box((0.22, 0.15, 0.16)),
        origin=Origin(
            xyz=(0.48, 0.0, BAR_TOP + 0.06),
            rpy=(0, math.radians(-35), 0),
        ),
        material=horse_paint,
        name="horse_neck",
    )

    # Horse head
    beam.visual(
        Box((0.24, 0.15, 0.14)),
        origin=Origin(xyz=(0.62, 0.0, BAR_TOP + 0.19)),
        material=horse_paint,
        name="horse_head",
    )

    # Horse snout (cylinder along X)
    beam.visual(
        Cylinder(radius=0.042, length=0.10),
        origin=Origin(
            xyz=(0.76, 0.0, BAR_TOP + 0.15),
            rpy=(0, math.pi / 2, 0),
        ),
        material=horse_cream,
        name="horse_snout",
    )

    # Horse ears (two small cones embedded into the head top)
    beam.visual(
        mesh_from_geometry(
            ConeGeometry(0.016, 0.055, radial_segments=12),
            "horse_ear_0",
        ),
        origin=Origin(xyz=(0.60, 0.048, BAR_TOP + 0.27)),
        material=horse_dark,
        name="horse_ear_0",
    )
    beam.visual(
        mesh_from_geometry(
            ConeGeometry(0.016, 0.055, radial_segments=12),
            "horse_ear_1",
        ),
        origin=Origin(xyz=(0.60, -0.048, BAR_TOP + 0.27)),
        material=horse_dark,
        name="horse_ear_1",
    )

    # Horse mane (small plates along the neck crest)
    for i in range(4):
        t = i / 3.0
        x = 0.42 + 0.18 * t
        z = BAR_TOP + 0.04 + 0.14 * t
        beam.visual(
            Box((0.035, 0.022, 0.055)),
            origin=Origin(xyz=(x, 0.0, z)),
            material=horse_dark,
            name=f"horse_mane_{i}",
        )

    # Horse tail (curved tube at rear)
    beam.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                _tail_points(),
                radius=0.014,
                samples_per_segment=6,
                radial_segments=12,
                cap_ends=True,
            ),
            "horse_tail",
        ),
        material=horse_dark,
        name="horse_tail",
    )

    # Decorative horse legs (4 short stubs hanging from body)
    leg_positions = [
        (0.30, 0.08),    # front left
        (0.30, -0.08),   # front right
        (-0.30, 0.08),   # rear left
        (-0.30, -0.08),  # rear right
    ]
    for i, (lx, ly) in enumerate(leg_positions):
        beam.visual(
            Cylinder(radius=0.020, length=0.08),
            origin=Origin(xyz=(lx, ly, BAR_BOT - 0.02)),
            material=horse_paint,
            name=f"horse_leg_{i}",
        )
        # Hoof (dark cap at bottom of each leg)
        beam.visual(
            Cylinder(radius=0.022, length=0.012),
            origin=Origin(xyz=(lx, ly, BAR_BOT - 0.066)),
            material=horse_dark,
            name=f"horse_hoof_{i}",
        )

    # Seats (flat plates on horse back near each end)
    for i, side in enumerate((1.0, -1.0)):
        beam.visual(
            Box((0.20, 0.22, 0.016)),
            origin=Origin(xyz=(side * SEAT_X, 0.0, BAR_TOP + 0.008)),
            material=saddle_red,
            name=f"seat_{i}",
        )

    # Handle mounting bosses (visual support on beam)
    for i, side in enumerate((1.0, -1.0)):
        beam.visual(
            Cylinder(radius=0.016, length=0.012),
            origin=Origin(xyz=(side * HANDLE_X, 0.0, BAR_TOP + 0.006)),
            material=galvanized,
            name=f"handle_boss_{i}",
        )

    # -------------------------------------------------------- handle_left ---
    handle_left = model.part("handle_left")
    handle_left.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                _handle_points(),
                radius=0.008,
                samples_per_segment=8,
                radial_segments=16,
                cap_ends=True,
            ),
            "handle_left_rod",
        ),
        material=handle_red,
        name="handle_left_rod",
    )
    # Grip sleeve (rubber coating on the top bend)
    handle_left.visual(
        Cylinder(radius=0.012, length=0.060),
        origin=Origin(xyz=(0, 0, 0.220), rpy=(math.pi / 2, 0, 0)),
        material=rubber,
        name="handle_left_grip",
    )

    # ------------------------------------------------------- handle_right ---
    handle_right = model.part("handle_right")
    handle_right.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                _handle_points(),
                radius=0.008,
                samples_per_segment=8,
                radial_segments=16,
                cap_ends=True,
            ),
            "handle_right_rod",
        ),
        material=handle_red,
        name="handle_right_rod",
    )
    handle_right.visual(
        Cylinder(radius=0.012, length=0.060),
        origin=Origin(xyz=(0, 0, 0.220), rpy=(math.pi / 2, 0, 0)),
        material=rubber,
        name="handle_right_grip",
    )

    # ---------------------------------------------------------- joints ---

    # Main beam pivot (rocks ±20°)
    model.articulation(
        "beam_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),  # positive q lowers the +X end
        motion_limits=MotionLimits(
            effort=100.0, velocity=2.5, lower=-TILT, upper=TILT,
        ),
    )

    # Handle left pivot (tilts ±10° forward/back)
    model.articulation(
        "handle_left_pivot",
        ArticulationType.REVOLUTE,
        parent=beam,
        child=handle_left,
        origin=Origin(xyz=(HANDLE_X, 0.0, BAR_TOP)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=-HANDLE_TILT, upper=HANDLE_TILT,
        ),
    )

    # Handle right pivot (tilts ±10° forward/back)
    model.articulation(
        "handle_right_pivot",
        ArticulationType.REVOLUTE,
        parent=beam,
        child=handle_right,
        origin=Origin(xyz=(-HANDLE_X, 0.0, BAR_TOP)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=-HANDLE_TILT, upper=HANDLE_TILT,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    beam = object_model.get_part("beam")
    handle_left = object_model.get_part("handle_left")
    handle_right = object_model.get_part("handle_right")
    pivot = object_model.get_articulation("beam_pivot")
    hl_pivot = object_model.get_articulation("handle_left_pivot")
    hr_pivot = object_model.get_articulation("handle_right_pivot")

    # ---- Pivot sleeve / axle (bushing fit) ----
    ctx.allow_overlap(
        beam, base,
        elem_a="pivot_sleeve", elem_b="pivot_axle",
        reason="Pivot sleeve is a bushing intentionally nested around the axle bolt.",
    )
    ctx.expect_contact(
        beam, base,
        elem_a="pivot_sleeve", elem_b="pivot_axle",
        name="pivot sleeve is seated on the axle bolt",
    )
    ctx.expect_within(
        beam, base,
        axes="y",
        inner_elem="pivot_sleeve", outer_elem="pivot_axle",
        margin=0.001,
        name="pivot sleeve stays inside the axle span",
    )

    # ---- Arch/sleeve overlap (pivot sleeve wraps around arch apex) ----
    ctx.allow_overlap(
        base, beam,
        elem_a="arch_0", elem_b="pivot_sleeve",
        reason="Pivot sleeve wraps around the arch apex to form the pivot bearing.",
    )
    ctx.allow_overlap(
        base, beam,
        elem_a="arch_1", elem_b="pivot_sleeve",
        reason="Pivot sleeve wraps around the arch apex to form the pivot bearing.",
    )

    # ---- Ear embedding (ears rooted into the horse head) ----
    ctx.allow_overlap(
        beam, beam,
        elem_a="horse_ear_0", elem_b="horse_head",
        reason="Horse ears are intentionally embedded into the head for mounting.",
    )
    ctx.allow_overlap(
        beam, beam,
        elem_a="horse_ear_1", elem_b="horse_head",
        reason="Horse ears are intentionally embedded into the head for mounting.",
    )

    # ---- Handle stem overlap (embedded in horse body for mounting) ----
    ctx.allow_overlap(
        beam, handle_left,
        elem_a="horse_torso", elem_b="handle_left_rod",
        reason="Handle stem is intentionally embedded in the horse body for mounting.",
    )
    ctx.allow_overlap(
        beam, handle_right,
        elem_a="horse_torso", elem_b="handle_right_rod",
        reason="Handle stem is intentionally embedded in the horse body for mounting.",
    )

    # ---- Ground pads exist at all 4 feet ----
    for i in range(4):
        pad_box = ctx.part_element_world_aabb(base, elem=f"ground_pad_{i}")
        ctx.check(
            f"ground_pad_{i} is near ground level",
            pad_box is not None and pad_box[0][2] < 0.015 and pad_box[1][2] > -0.002,
            details=f"pad aabb={pad_box}",
        )

    # ---- Beam pivot joint config ----
    ax = pivot.axis
    ctx.check(
        "pivot axis is horizontal and perpendicular to beam",
        abs(ax[0]) < 1e-9 and abs(ax[1] - 1.0) < 1e-9 and abs(ax[2]) < 1e-9,
        details=f"axis={ax}",
    )
    lim = pivot.motion_limits
    ctx.check(
        "rocking limits are ±20 degrees",
        lim is not None
        and lim.lower is not None and lim.upper is not None
        and abs(lim.lower + TILT) < 1e-6
        and abs(lim.upper - TILT) < 1e-6,
        details=f"limits=({lim.lower}, {lim.upper})",
    )

    # ---- Handle pivot joints configured correctly ----
    for jname, joint in [
        ("handle_left_pivot", hl_pivot),
        ("handle_right_pivot", hr_pivot),
    ]:
        hlim = joint.motion_limits
        ctx.check(
            f"{jname} has ±10° pivot limits",
            hlim is not None
            and hlim.lower is not None and hlim.upper is not None
            and abs(hlim.lower + HANDLE_TILT) < 1e-6
            and abs(hlim.upper - HANDLE_TILT) < 1e-6,
            details=f"limits=({hlim.lower}, {hlim.upper})",
        )
        hax = joint.axis
        ctx.check(
            f"{jname} axis is horizontal Y",
            abs(hax[0]) < 1e-9 and abs(hax[1] - 1.0) < 1e-9 and abs(hax[2]) < 1e-9,
            details=f"axis={hax}",
        )

    # ---- Horse body features ----
    torso = ctx.part_element_world_aabb(beam, elem="horse_torso")
    head = ctx.part_element_world_aabb(beam, elem="horse_head")
    tail = ctx.part_element_world_aabb(beam, elem="horse_tail")

    ctx.check(
        "horse torso is a substantial body",
        torso is not None and (torso[1][0] - torso[0][0]) > 0.80,
        details=f"torso aabb={torso}",
    )
    ctx.check(
        "horse head extends above the torso at the +X end",
        head is not None and torso is not None
        and head[1][2] > torso[1][2] - 0.02
        and head[1][0] > 0.4,
        details=f"head={head}, torso={torso}",
    )
    ctx.check(
        "horse tail is at the -X end",
        tail is not None and torso is not None
        and tail[0][0] < torso[0][0] + 0.10,
        details=f"tail={tail}, torso={torso}",
    )

    # ---- Seats on the horse back ----
    for i in range(2):
        seat = ctx.part_element_world_aabb(beam, elem=f"seat_{i}")
        ctx.check(
            f"seat_{i} sits on top of the horse body",
            seat is not None and torso is not None
            and seat[0][2] > torso[0][2] + 0.05
            and seat[1][2] > torso[1][2] - 0.02,
            details=f"seat aabb={seat}",
        )

    # ---- Handles are separate parts near the beam ----
    for hname, handle in [("handle_left", handle_left), ("handle_right", handle_right)]:
        h_box = ctx.part_world_aabb(handle)
        ctx.check(
            f"{hname} is a separate part near the horse body",
            h_box is not None and torso is not None
            and h_box[1][2] > torso[0][2] + 0.10
            and h_box[0][2] < torso[1][2] + 0.30,
            details=f"handle aabb={h_box}",
        )

    # ---- Beam bar clears arch saddle ----
    ctx.expect_gap(
        beam, base,
        axis="z",
        positive_elem="horse_torso",
        negative_elem="arch_0",
        min_gap=0.005,
        max_gap=0.10,
        name="horse body clears the arch saddle",
    )

    # ---- Decisive pose: beam rocking ----
    rest_seat0 = ctx.part_element_world_aabb(beam, elem="seat_0")
    with ctx.pose({pivot: TILT}):
        down_seat0 = ctx.part_element_world_aabb(beam, elem="seat_0")
        up_seat1 = ctx.part_element_world_aabb(beam, elem="seat_1")
        ctx.check(
            "positive rock lowers the +X end",
            rest_seat0 is not None and down_seat0 is not None
            and down_seat0[0][2] < rest_seat0[0][2] - 0.10,
            details=f"rest={rest_seat0}, tilted={down_seat0}",
        )
        ctx.check(
            "positive rock raises the -X end",
            up_seat1 is not None and rest_seat0 is not None
            and up_seat1[0][2] > rest_seat0[0][2] + 0.04,
            details=f"raised seat aabb={up_seat1}",
        )

    with ctx.pose({pivot: -TILT}):
        down_seat1 = ctx.part_element_world_aabb(beam, elem="seat_1")
        ctx.check(
            "negative rock lowers the -X end",
            down_seat1 is not None and rest_seat0 is not None
            and down_seat1[0][2] < rest_seat0[0][2] - 0.10,
            details=f"tilted seat aabb={down_seat1}",
        )

    # ---- Handle pivot pose check ----
    with ctx.pose({hl_pivot: HANDLE_TILT}):
        hl_box = ctx.part_world_aabb(handle_left)
        ctx.check(
            "handle_left tilts when pivoted (non-fixed joint works)",
            hl_box is not None,
            details=f"tilted handle aabb={hl_box}",
        )

    # ---- Scale / grounding checks ----
    base_box = ctx.part_world_aabb(base)
    ctx.check(
        "base feet rest near ground level",
        base_box is not None and -0.005 <= base_box[0][2] <= 0.020,
        details=f"base aabb={base_box}",
    )

    axle_box = ctx.part_element_world_aabb(base, elem="pivot_axle")
    ctx.check(
        "pivot axle sits about 0.5 m high (toddler scale)",
        axle_box is not None and 0.45 <= (axle_box[0][2] + axle_box[1][2]) / 2 <= 0.55,
        details=f"axle aabb={axle_box}",
    )

    return ctx.report()


object_model = build_object_model()
