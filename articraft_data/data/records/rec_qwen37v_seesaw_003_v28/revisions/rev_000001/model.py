from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    ExtrudeGeometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Shared dimensions (meters). World: X along the seesaw length, Z up.
# Compact backyard seesaw: ~1.5 m long, ~0.55 m tall.
# ---------------------------------------------------------------------------
BEAM_HALF = 0.75          # half-length of beam
BEAM_R = 0.038            # beam tube radius (76 mm diameter)
PIVOT_Z = 0.44            # world height of the rocking axis

# A-frame triangular support
LEG_SPREAD_Y = 0.26       # foot offset from center in Y
LEG_TOP_Z = 0.38          # legs meet below beam (bracket carries beam)
LEG_FOOT_Z = 0.02         # feet slightly above ground (pad takes up space)
LEG_R = 0.022             # leg tube radius (44 mm)
LEG_DY = LEG_SPREAD_Y
LEG_DZ = LEG_TOP_Z - LEG_FOOT_Z
LEG_LEN = math.sqrt(LEG_DY**2 + LEG_DZ**2)
LEG_ANGLE = math.atan2(LEG_DY, LEG_DZ)

# Cross-brace at mid-height
CROSS_BRACE_Z = 0.18
_BRACE_FRAC = (CROSS_BRACE_Z - LEG_FOOT_Z) / (LEG_TOP_Z - LEG_FOOT_Z)
_BRACE_Y = LEG_SPREAD_Y * (1.0 - _BRACE_FRAC)
BRACE_LEN = 2.0 * _BRACE_Y + 0.02  # slightly longer to ensure contact

# Ground pads
PAD_THICK = 0.024
PAD_SIZE = (0.12, 0.10, PAD_THICK)

# Pivot bracket
BRACKET_SIZE = (0.10, 0.08, 0.10)
BRACKET_Z = 0.42           # bracket center (spans from leg apex to above beam)

# Seats (moved inward to avoid bumper overlap)
SEAT_X = 0.52
SEAT_DROP = 0.13           # how far below beam centerline the seat sits
SEAT_PLATE_T = 0.012

# Handles
HANDLE_X = 0.44
HANDLE_POST_H = 0.22       # handle post height above beam

# Bumpers (prismatic compression)
BUMPER_MOUNT_X = BEAM_HALF - 0.06  # 0.69
BUMPER_R = 0.032
BUMPER_H = 0.042
BUMPER_TRAVEL = 0.030       # max compression travel

# Bump stops (fixed to frame)
BUMP_STOP_X = 0.62
BUMP_STOP_TOP_Z = 0.22     # top of bump stop (well below seat plates)
BUMP_STOP_SIZE = (0.07, 0.06, 0.05)

ROCK_LIMIT = 0.262         # ~15 degrees each way


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="backyard_seesaw")

    # Materials
    model.material("powder_blue", rgba=(0.22, 0.42, 0.68, 1.0))
    model.material("bright_green", rgba=(0.30, 0.75, 0.22, 1.0))
    model.material("dark_gray", rgba=(0.28, 0.29, 0.31, 1.0))
    model.material("charcoal_rubber", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("red_rubber", rgba=(0.55, 0.15, 0.12, 1.0))
    model.material("silver_bolt", rgba=(0.72, 0.73, 0.76, 1.0))
    model.material("black_plastic", rgba=(0.08, 0.08, 0.09, 1.0))

    # -----------------------------------------------------------------
    # Support frame (root): A-frame triangular legs, cross-brace,
    # pivot bracket, ground pads, bump-stop arms and bump stops.
    # -----------------------------------------------------------------
    frame = model.part("support_frame")

    # Two A-frame legs splaying in Y (triangular profile from end view)
    for i, sign in enumerate((1.0, -1.0)):
        leg_cy = sign * LEG_SPREAD_Y / 2.0
        leg_cz = (LEG_TOP_Z + LEG_FOOT_Z) / 2.0
        frame.visual(
            Cylinder(radius=LEG_R, length=LEG_LEN),
            origin=Origin(
                xyz=(0.0, leg_cy, leg_cz),
                rpy=(sign * LEG_ANGLE, 0.0, 0.0),
            ),
            material="powder_blue",
            name=f"frame_leg_{i}",
        )

    # Cross-brace connecting legs at mid-height (oriented along Y)
    frame.visual(
        Cylinder(radius=0.016, length=BRACE_LEN),
        origin=Origin(
            xyz=(0.0, 0.0, CROSS_BRACE_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="powder_blue",
        name="cross_brace",
    )

    # Pivot bracket at top of A-frame
    frame.visual(
        Box(BRACKET_SIZE),
        origin=Origin(xyz=(0.0, 0.0, BRACKET_Z)),
        material="dark_gray",
        name="pivot_bracket",
    )

    # Pivot bosses on bracket cheeks
    for i, sy in enumerate((1.0, -1.0)):
        frame.visual(
            Cylinder(radius=0.030, length=0.014),
            origin=Origin(
                xyz=(0.0, sy * 0.047, PIVOT_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material="dark_gray",
            name=f"pivot_boss_{i}",
        )
        frame.visual(
            Cylinder(radius=0.008, length=0.008),
            origin=Origin(
                xyz=(0.0, sy * 0.056, PIVOT_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material="silver_bolt",
            name=f"boss_bolt_{i}",
        )

    # Rubber ground pads under each leg foot
    for i, sign in enumerate((1.0, -1.0)):
        foot_y = sign * LEG_SPREAD_Y
        frame.visual(
            Box(PAD_SIZE),
            origin=Origin(xyz=(0.0, foot_y, PAD_THICK / 2.0)),
            material="charcoal_rubber",
            name=f"ground_pad_{i}",
        )
        # Pad grip ribs on top surface
        for j, dx in enumerate((-0.03, 0.0, 0.03)):
            frame.visual(
                Box((0.005, PAD_SIZE[1] * 0.75, 0.004)),
                origin=Origin(xyz=(dx, foot_y, PAD_THICK + 0.002)),
                material="charcoal_rubber",
                name=f"pad_rib_{i}_{j}",
            )

    # Bump-stop support arms: diagonal tubes from bracket down to bump-stop positions
    bracket_bottom_z = BRACKET_Z - BRACKET_SIZE[2] / 2.0
    bump_stop_cz = BUMP_STOP_TOP_Z - BUMP_STOP_SIZE[2] / 2.0
    for i, sign in enumerate((1.0, -1.0)):
        arm_pts = [
            (sign * 0.04, 0.0, bracket_bottom_z),
            (sign * 0.22, 0.0, 0.24),
            (sign * BUMP_STOP_X, 0.0, BUMP_STOP_TOP_Z),
        ]
        frame.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    arm_pts, radius=0.014, samples_per_segment=6, radial_segments=14,
                ),
                f"bump_arm_{i}",
            ),
            material="powder_blue",
            name=f"bump_arm_{i}",
        )

        # Rubber bump stop block at the end of each arm
        frame.visual(
            Box(BUMP_STOP_SIZE),
            origin=Origin(xyz=(sign * BUMP_STOP_X, 0.0, bump_stop_cz)),
            material="red_rubber",
            name=f"bump_stop_{i}",
        )

    # -----------------------------------------------------------------
    # Beam: straight green tube with seats and handle assemblies.
    # Part frame at the pivot axis; geometry relative to that frame.
    # -----------------------------------------------------------------
    beam = model.part("beam")

    # Main beam tube (oriented along X)
    beam.visual(
        Cylinder(radius=BEAM_R, length=BEAM_HALF * 2.0),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="bright_green",
        name="beam_tube",
    )

    # End caps on beam tube
    for sign in (1.0, -1.0):
        beam.visual(
            Cylinder(radius=BEAM_R + 0.003, length=0.008),
            origin=Origin(
                xyz=(sign * BEAM_HALF, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material="dark_gray",
            name=f"end_cap_{'right' if sign > 0 else 'left'}",
        )

    # Seat profile: rounded rectangular plate
    seat_profile = rounded_rect_profile(0.22, 0.20, 0.04)

    for i, s in enumerate((1.0, -1.0)):
        # Seat support post from beam down to seat plate
        post_len = SEAT_DROP - BEAM_R
        post_cz = -(BEAM_R + post_len / 2.0)
        beam.visual(
            Cylinder(radius=0.018, length=post_len),
            origin=Origin(xyz=(s * SEAT_X, 0.0, post_cz)),
            material="bright_green",
            name=f"seat_post_{i}",
        )

        # Seat plate at SEAT_DROP below beam center
        seat_z = -SEAT_DROP
        seat = ExtrudeGeometry(seat_profile, SEAT_PLATE_T, cap=True, center=True)
        seat.translate(s * SEAT_X, 0.0, seat_z)
        beam.visual(
            mesh_from_geometry(seat, f"seat_plate_{i}"),
            material="dark_gray",
            name=f"seat_plate_{i}",
        )

        # Seat back lip (raised edge at the outer end, overlaps plate)
        beam.visual(
            Box((0.015, 0.18, 0.045)),
            origin=Origin(xyz=(s * (SEAT_X + 0.10), 0.0, seat_z + 0.005)),
            material="dark_gray",
            name=f"seat_back_{i}",
        )

        # Handle post rising from beam
        beam.visual(
            Cylinder(radius=0.014, length=HANDLE_POST_H),
            origin=Origin(xyz=(s * HANDLE_X, 0.0, HANDLE_POST_H / 2.0)),
            material="bright_green",
            name=f"handle_post_{i}",
        )

        # Handle grip bar (rubber-covered)
        beam.visual(
            Cylinder(radius=0.017, length=0.13),
            origin=Origin(
                xyz=(s * HANDLE_X, 0.0, HANDLE_POST_H),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material="black_plastic",
            name=f"handle_grip_{i}",
        )

        # Grip end caps
        for j, gy in enumerate((0.068, -0.068)):
            beam.visual(
                Cylinder(radius=0.020, length=0.010),
                origin=Origin(
                    xyz=(s * HANDLE_X, gy, HANDLE_POST_H),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material="black_plastic",
                name=f"grip_cap_{i}_{j}",
            )

    # -----------------------------------------------------------------
    # Bumpers: rubber cylinders on prismatic joints under beam ends.
    # -----------------------------------------------------------------
    for i, s in enumerate((1.0, -1.0)):
        bumper = model.part(f"bumper_{i}")

        # Rubber bumper body (hangs below beam, overlaps plate for connectivity)
        bumper.visual(
            Cylinder(radius=BUMPER_R, length=BUMPER_H),
            origin=Origin(xyz=(0.0, 0.0, -BUMPER_H / 2.0 - 0.003)),
            material="red_rubber",
            name=f"bumper_body_{i}",
        )

        # Metal mounting plate (flush-seated into beam underside)
        bumper.visual(
            Cylinder(radius=BUMPER_R + 0.005, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, -0.001)),
            material="dark_gray",
            name=f"bumper_plate_{i}",
        )

        # Prismatic joint: bumper compresses upward into beam
        # axis=(0,0,1): positive q moves bumper up (compression)
        model.articulation(
            f"bumper_{i}_slide",
            ArticulationType.PRISMATIC,
            parent=beam,
            child=bumper,
            origin=Origin(xyz=(s * BUMPER_MOUNT_X, 0.0, -BEAM_R)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=200.0,
                velocity=0.5,
                lower=0.0,
                upper=BUMPER_TRAVEL,
            ),
        )

    # -----------------------------------------------------------------
    # Main rocking pivot: horizontal axis perpendicular to beam length.
    # -----------------------------------------------------------------
    model.articulation(
        "beam_pivot",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=300.0,
            velocity=1.5,
            lower=-ROCK_LIMIT,
            upper=ROCK_LIMIT,
        ),
    )

    return model


def _intersects(a, b, tol: float = 1e-4) -> bool:
    if a is None or b is None:
        return False
    return all(a[0][i] <= b[1][i] + tol and b[0][i] <= a[1][i] + tol for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("support_frame")
    beam = object_model.get_part("beam")
    bumper_0 = object_model.get_part("bumper_0")
    bumper_1 = object_model.get_part("bumper_1")
    pivot = object_model.get_articulation("beam_pivot")
    bslide_0 = object_model.get_articulation("bumper_0_slide")
    bslide_1 = object_model.get_articulation("bumper_1_slide")

    # --- Beam tube passes through the pivot bracket (intentional pivot fit) ---
    ctx.allow_overlap(
        beam,
        frame,
        elem_a="beam_tube",
        elem_b="pivot_bracket",
        reason="The beam tube rotates within the pivot bracket that captures the rocking axle.",
    )
    ctx.expect_overlap(
        beam,
        frame,
        axes="z",
        elem_a="beam_tube",
        elem_b="pivot_bracket",
        min_overlap=0.02,
        name="beam tube captured by pivot bracket",
    )
    ctx.expect_within(
        beam,
        frame,
        axes="y",
        inner_elem="beam_tube",
        outer_elem="pivot_bracket",
        margin=0.02,
        name="beam tube centered in pivot bracket laterally",
    )

    # --- Bumper plates flush-mount against beam bottom (intentional local embed) ---
    ctx.allow_overlap(
        beam,
        bumper_0,
        elem_a="beam_tube",
        elem_b="bumper_plate_0",
        reason="The bumper mounting plate is flush-seated against the beam underside.",
    )
    ctx.allow_overlap(
        beam,
        bumper_1,
        elem_a="beam_tube",
        elem_b="bumper_plate_1",
        reason="The bumper mounting plate is flush-seated against the beam underside.",
    )
    ctx.expect_contact(
        beam,
        bumper_0,
        elem_a="beam_tube",
        elem_b="bumper_plate_0",
        name="bumper_0 plate contacts beam underside",
    )
    ctx.expect_contact(
        beam,
        bumper_1,
        elem_a="beam_tube",
        elem_b="bumper_plate_1",
        name="bumper_1 plate contacts beam underside",
    )

    # --- Triangular A-frame supports exist ---
    leg0 = ctx.part_element_world_aabb(frame, elem="frame_leg_0")
    leg1 = ctx.part_element_world_aabb(frame, elem="frame_leg_1")
    ctx.check(
        "A-frame has two triangular support legs",
        leg0 is not None and leg1 is not None,
        details=f"leg0={leg0}, leg1={leg1}",
    )

    # Legs splay apart in Y (triangular profile when viewed from beam end)
    def _cy(aabb):
        return 0.5 * (aabb[0][1] + aabb[1][1])

    ctx.check(
        "support legs splay apart to form triangle",
        leg0 is not None
        and leg1 is not None
        and _cy(leg0) > 0.05
        and _cy(leg1) < -0.05,
        details=f"leg0_cy={None if leg0 is None else _cy(leg0)}, "
                f"leg1_cy={None if leg1 is None else _cy(leg1)}",
    )

    # Cross-brace connects the two legs
    brace = ctx.part_element_world_aabb(frame, elem="cross_brace")
    ctx.check(
        "cross-brace connects the support legs",
        brace is not None
        and leg0 is not None
        and leg1 is not None
        and _intersects(brace, leg0)
        and _intersects(brace, leg1),
        details=f"brace={brace}, leg0={leg0}, leg1={leg1}",
    )

    # --- Rubber ground pads under support legs ---
    pad0 = ctx.part_element_world_aabb(frame, elem="ground_pad_0")
    pad1 = ctx.part_element_world_aabb(frame, elem="ground_pad_1")
    ctx.check(
        "rubber ground pads present under both legs",
        pad0 is not None and pad1 is not None,
        details=f"pad0={pad0}, pad1={pad1}",
    )

    # Pads sit at ground level
    ctx.check(
        "ground pads sit at ground level",
        pad0 is not None
        and pad1 is not None
        and pad0[0][2] < 0.015
        and pad1[0][2] < 0.015,
        details=f"pad0_z_min={None if pad0 is None else pad0[0][2]}, "
                f"pad1_z_min={None if pad1 is None else pad1[0][2]}",
    )

    # Pads positioned at the feet (outer Y positions)
    ctx.check(
        "ground pads positioned at leg feet",
        pad0 is not None
        and pad1 is not None
        and abs(_cy(pad0)) > 0.15
        and abs(_cy(pad1)) > 0.15
        and _cy(pad0) * _cy(pad1) < 0,  # opposite sides
        details=f"pad0_cy={None if pad0 is None else _cy(pad0)}, "
                f"pad1_cy={None if pad1 is None else _cy(pad1)}",
    )

    # Pads contact leg feet
    ctx.check(
        "ground pads contact leg feet",
        pad0 is not None
        and pad1 is not None
        and leg0 is not None
        and leg1 is not None
        and _intersects(pad0, leg0)
        and _intersects(pad1, leg1),
        details=f"pad0={pad0}, leg0={leg0}, pad1={pad1}, leg1={leg1}",
    )

    # --- Safety bump stops below each beam end ---
    bstop0 = ctx.part_element_world_aabb(frame, elem="bump_stop_0")
    bstop1 = ctx.part_element_world_aabb(frame, elem="bump_stop_1")
    ctx.check(
        "safety bump stops mounted below beam travel",
        bstop0 is not None and bstop1 is not None,
        details=f"bstop0={bstop0}, bstop1={bstop1}",
    )

    # Bump stops are outboard near beam ends
    def _cx(aabb):
        return 0.5 * (aabb[0][0] + aabb[1][0])

    ctx.check(
        "bump stops positioned near beam ends",
        bstop0 is not None
        and bstop1 is not None
        and _cx(bstop0) > 0.4
        and _cx(bstop1) < -0.4,
        details=f"bstop0_cx={None if bstop0 is None else _cx(bstop0)}, "
                f"bstop1_cx={None if bstop1 is None else _cx(bstop1)}",
    )

    # Bump stops are below beam level
    beam_aabb = ctx.part_element_world_aabb(beam, elem="beam_tube")
    ctx.check(
        "bump stops sit below beam height",
        bstop0 is not None
        and bstop1 is not None
        and beam_aabb is not None
        and bstop0[1][2] < beam_aabb[0][2] + 0.05
        and bstop1[1][2] < beam_aabb[0][2] + 0.05,
        details=f"bstop0={bstop0}, bstop1={bstop1}, beam={beam_aabb}",
    )

    # Bump stop arms connect to bump stops
    barm0 = ctx.part_element_world_aabb(frame, elem="bump_arm_0")
    barm1 = ctx.part_element_world_aabb(frame, elem="bump_arm_1")
    ctx.check(
        "bump stop arms reach the bump stops",
        barm0 is not None
        and barm1 is not None
        and bstop0 is not None
        and bstop1 is not None
        and _intersects(barm0, bstop0)
        and _intersects(barm1, bstop1),
        details=f"barm0={barm0}, bstop0={bstop0}, barm1={barm1}, bstop1={bstop1}",
    )

    # --- Rubber bumpers on prismatic joints ---
    bbody0 = ctx.part_element_world_aabb(bumper_0, elem="bumper_body_0")
    bbody1 = ctx.part_element_world_aabb(bumper_1, elem="bumper_body_1")
    ctx.check(
        "rubber bumpers mounted at beam ends",
        bbody0 is not None and bbody1 is not None,
        details=f"bbody0={bbody0}, bbody1={bbody1}",
    )

    # Bumpers hang below the beam
    ctx.check(
        "bumpers hang below the beam",
        bbody0 is not None
        and bbody1 is not None
        and beam_aabb is not None
        and bbody0[1][2] < beam_aabb[0][2] + 0.02
        and bbody1[1][2] < beam_aabb[0][2] + 0.02,
        details=f"bbody0={bbody0}, bbody1={bbody1}, beam={beam_aabb}",
    )

    # Bumpers near beam ends in X
    ctx.check(
        "bumpers positioned near beam ends",
        bbody0 is not None
        and bbody1 is not None
        and _cx(bbody0) > 0.5
        and _cx(bbody1) < -0.5,
        details=f"bbody0_cx={None if bbody0 is None else _cx(bbody0)}, "
                f"bbody1_cx={None if bbody1 is None else _cx(bbody1)}",
    )

    # Prismatic joint limits for bumper compression
    lim0 = bslide_0.motion_limits
    lim1 = bslide_1.motion_limits
    ctx.check(
        "bumper prismatic joints have compression travel",
        lim0 is not None
        and lim1 is not None
        and lim0.upper > 0.01
        and lim1.upper > 0.01
        and lim0.lower >= 0.0
        and lim1.lower >= 0.0,
        details=f"bslide_0=({lim0.lower}, {lim0.upper}), bslide_1=({lim1.lower}, {lim1.upper})",
    )

    # --- Revolute pivot joint ---
    plim = pivot.motion_limits
    ctx.check(
        "beam pivot is revolute with symmetric rocking range",
        plim is not None
        and abs(plim.lower + ROCK_LIMIT) < 0.02
        and abs(plim.upper - ROCK_LIMIT) < 0.02,
        details=f"limits=({plim.lower}, {plim.upper})",
    )

    # --- Pose: beam tilts correctly ---
    beam_rest = ctx.part_world_aabb(beam)
    with ctx.pose({pivot: ROCK_LIMIT}):
        beam_tilted = ctx.part_world_aabb(beam)
        frame_posed = ctx.part_world_aabb(frame)

        ctx.check(
            "positive rock tilts beam",
            beam_tilted is not None
            and beam_rest is not None
            and abs(beam_tilted[1][2] - beam_rest[1][2]) > 0.01,
            details=f"rest={beam_rest}, tilted={beam_tilted}",
        )

        ctx.check(
            "frame stays fixed during rocking",
            frame_posed is not None
            and abs(frame_posed[0][2]) < 0.01,
            details=f"frame_posed={frame_posed}",
        )

    # --- Pose: bumper compression ---
    with ctx.pose({bslide_0: BUMPER_TRAVEL}):
        b0_compressed = ctx.part_element_world_aabb(bumper_0, elem="bumper_body_0")
        ctx.check(
            "bumper_0 compresses upward at max prismatic travel",
            b0_compressed is not None
            and bbody0 is not None
            and b0_compressed[0][2] > bbody0[0][2] + 0.01,
            details=f"rest={bbody0}, compressed={b0_compressed}",
        )

    with ctx.pose({bslide_1: BUMPER_TRAVEL}):
        b1_compressed = ctx.part_element_world_aabb(bumper_1, elem="bumper_body_1")
        ctx.check(
            "bumper_1 compresses upward at max prismatic travel",
            b1_compressed is not None
            and bbody1 is not None
            and b1_compressed[0][2] > bbody1[0][2] + 0.01,
            details=f"rest={bbody1}, compressed={b1_compressed}",
        )

    # --- Seats at opposite ends ---
    seat0 = ctx.part_element_world_aabb(beam, elem="seat_plate_0")
    seat1 = ctx.part_element_world_aabb(beam, elem="seat_plate_1")
    ctx.check(
        "seats at opposite ends of beam",
        seat0 is not None
        and seat1 is not None
        and _cx(seat0) > 0.3
        and _cx(seat1) < -0.3,
        details=f"seat0={seat0}, seat1={seat1}",
    )

    # --- Overall compact size ---
    full_aabb = ctx.part_world_aabb(beam)
    ctx.check(
        "compact seesaw: beam length about 1.4-1.7 m",
        full_aabb is not None and 1.4 <= (full_aabb[1][0] - full_aabb[0][0]) <= 1.8,
        details=f"beam_aabb={full_aabb}",
    )

    frame_aabb = ctx.part_world_aabb(frame)
    ctx.check(
        "overall height about 0.40-0.70 m",
        frame_aabb is not None and full_aabb is not None
        and 0.40 <= max(frame_aabb[1][2], full_aabb[1][2]) <= 0.72,
        details=f"frame={frame_aabb}, beam={full_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
