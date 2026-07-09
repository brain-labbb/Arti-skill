from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CylinderGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# ----------------------------------------------------------------------------
# Compact backyard seesaw with triangular A-frame supports.
#
# Layout (world frame, Z up, base centered on the origin):
# - Sky-blue painted A-frame base: two angled tube legs converging near the
#   top pivot bar (~0.45 m tall), connected by a horizontal cross member and
#   outrigger arms carrying safety bump stops.
# - Rubber ground pads under each foot.
# - One yellow rocking beam (~1.5 m long) pivots on top of the A-frame.
#   Each end carries a seat plate and a T-handlebar.
# - Safety bump stops: rubber blocks on short posts from the outrigger arms,
#   positioned below where beam ends travel at max tilt.
# - Rubber end bumpers: cylindrical rubber pads mounted under each beam end
#   on short prismatic joints (vertical compression, ~25 mm travel).
# - Articulation: beam pivots revolute about the horizontal axis at the
#   A-frame apex, +/- 15 degrees. Each bumper has a prismatic joint for
#   vertical compression (0 to 0.025 m).
# ----------------------------------------------------------------------------

TUBE_R = 0.020       # ~40 mm diameter main tubing
BEAM_TUBE_R = 0.018  # beam tube slightly smaller
BRACE_R = 0.016
HANDLE_R = 0.014
SUPPORT_R = 0.016

PIVOT_HEIGHT = 0.45   # A-frame apex height (pivot point)
LEG_SPREAD = 0.28     # half-spread of A-frame feet on the ground
BEAM_LEN = 1.50       # total beam length
BEAM_HALF = BEAM_LEN / 2.0

SEAT_X = 0.56         # seat center distance from beam midpoint
SEAT_SIZE = (0.20, 0.24, 0.012)
HANDLE_X = 0.42       # T-handlebar post distance from beam midpoint
HANDLE_TOP_Z = 0.28   # crossbar height above beam centerline (beam local z)
MAIN_Z = 0.06         # beam tube center height above pivot axis

CROSS_Z = 0.12        # cross-member height on the A-frame

# Bump stop and bumper geometry
BUMP_STOP_X = 0.68    # distance from center where bump stops sit
BUMP_STOP_POST_H = 0.18  # height of bump stop post top above ground
BUMP_STOP_SIZE = (0.06, 0.06, 0.03)

BUMPER_R = 0.025      # bumper disc half-size
BUMPER_H = 0.035      # bumper disc height (uncompressed)
BUMPER_X = 0.72       # bumper position along beam from center

TILT = math.radians(15.0)  # rocking range
BUMPER_TRAVEL = 0.025      # prismatic compression travel

# Materials
SKY_BLUE = Material("sky_blue_paint", rgba=(0.33, 0.62, 0.84, 1.0))
WORN_YELLOW = Material("worn_yellow_paint", rgba=(0.87, 0.74, 0.12, 1.0))
RUST_BROWN = Material("rust_brown_steel", rgba=(0.42, 0.21, 0.13, 1.0))
RUBBER_BLACK = Material("rubber_black", rgba=(0.12, 0.12, 0.12, 1.0))
PAD_GREEN = Material("rubber_pad_green", rgba=(0.18, 0.32, 0.16, 1.0))


def _tube_between(
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    radius: float,
    *,
    radial_segments: int = 16,
) -> MeshGeometry:
    """Straight capped tube between two 3D points."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    geom = CylinderGeometry(radius, length, radial_segments=radial_segments)
    ux, uy, uz = dx / length, dy / length, dz / length
    ax, ay, az = -uy, ux, 0.0
    s = math.sqrt(ax * ax + ay * ay + az * az)
    if s > 1e-9:
        geom.rotate((ax / s, ay / s, az / s), math.atan2(s, uz))
    elif uz < 0.0:
        geom.rotate_x(math.pi)
    geom.translate(
        (p0[0] + p1[0]) / 2.0,
        (p0[1] + p1[1]) / 2.0,
        (p0[2] + p1[2]) / 2.0,
    )
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="backyard_seesaw")

    # =========================================================================
    # BASE: Triangular A-frame support with ground pads and bump stops
    # =========================================================================
    base = model.part("base")

    # A-frame legs: two angled tubes from ground feet converging at the apex.
    foot_left = (-LEG_SPREAD, 0.0, 0.015)
    foot_right = (LEG_SPREAD, 0.0, 0.015)
    apex = (0.0, 0.0, PIVOT_HEIGHT)

    base.visual(
        mesh_from_geometry(
            _tube_between(foot_left, apex, TUBE_R),
            "leg_left",
        ),
        material=SKY_BLUE,
        name="leg_left",
    )
    base.visual(
        mesh_from_geometry(
            _tube_between(foot_right, apex, TUBE_R),
            "leg_right",
        ),
        material=SKY_BLUE,
        name="leg_right",
    )

    # Short pivot bar at the apex (horizontal, along Y, serves as the axle)
    pivot_bar_half = 0.08
    base.visual(
        mesh_from_geometry(
            _tube_between(
                (0.0, -pivot_bar_half, PIVOT_HEIGHT),
                (0.0, pivot_bar_half, PIVOT_HEIGHT),
                TUBE_R,
            ),
            "pivot_bar",
        ),
        material=SKY_BLUE,
        name="pivot_bar",
    )

    # Cross member connecting the two legs partway up
    leg_frac = (CROSS_Z - 0.015) / (PIVOT_HEIGHT - 0.015)
    cross_left_x = foot_left[0] * (1.0 - leg_frac)
    cross_right_x = foot_right[0] * (1.0 - leg_frac)
    base.visual(
        mesh_from_geometry(
            _tube_between(
                (cross_left_x, 0.0, CROSS_Z),
                (cross_right_x, 0.0, CROSS_Z),
                BRACE_R,
            ),
            "cross_member",
        ),
        material=SKY_BLUE,
        name="cross_member",
    )

    # Outrigger arms: horizontal tubes extending from the cross member area
    # outward to the bump stop positions, providing structural support for
    # the bump stops.
    outrigger_inner = max(abs(cross_left_x), abs(cross_right_x)) + 0.02
    for sx in (-1.0, 1.0):
        outrigger = _tube_between(
            (sx * outrigger_inner, 0.0, CROSS_Z),
            (sx * BUMP_STOP_X, 0.0, CROSS_Z),
            BRACE_R * 0.9,
        )
        base.visual(
            mesh_from_geometry(outrigger, f"outrigger_{'left' if sx < 0 else 'right'}"),
            material=SKY_BLUE,
            name=f"outrigger_{'left' if sx < 0 else 'right'}",
        )

    # Diagonal braces from cross member to near the apex for triangulation
    for sx in (-1.0, 1.0):
        brace_top = (sx * 0.03, 0.0, PIVOT_HEIGHT - 0.06)
        brace_bot = (sx * abs(cross_right_x) * 0.85, 0.0, CROSS_Z)
        base.visual(
            mesh_from_geometry(
                _tube_between(brace_bot, brace_top, BRACE_R * 0.8),
                f"diag_brace_{'left' if sx < 0 else 'right'}",
            ),
            material=SKY_BLUE,
            name=f"diag_brace_{'left' if sx < 0 else 'right'}",
        )

    # Rubber ground pads under each foot
    pad_size = (0.12, 0.08, 0.015)
    for sx, name in ((-1.0, "ground_pad_0"), (1.0, "ground_pad_1")):
        base.visual(
            Box(pad_size),
            origin=Origin(xyz=(sx * LEG_SPREAD, 0.0, pad_size[2] / 2.0)),
            material=PAD_GREEN,
            name=name,
        )

    # Safety bump stops: vertical posts from outrigger arms with rubber blocks on top
    for sx, name_post, name_block in (
        (-1.0, "bump_post_0", "bump_stop_0"),
        (1.0, "bump_post_1", "bump_stop_1"),
    ):
        post_x = sx * BUMP_STOP_X
        post_base_z = CROSS_Z
        post_top_z = BUMP_STOP_POST_H
        post_len = post_top_z - post_base_z
        # Vertical post tube from outrigger up to bump stop
        base.visual(
            mesh_from_geometry(
                CylinderGeometry(SUPPORT_R * 0.7, post_len, radial_segments=12)
                .translate(post_x, 0.0, post_base_z + post_len / 2.0),
                name_post,
            ),
            material=SKY_BLUE,
            name=name_post,
        )
        # Rubber bump stop block on top of the post
        base.visual(
            Box(BUMP_STOP_SIZE),
            origin=Origin(xyz=(post_x, 0.0, post_top_z + BUMP_STOP_SIZE[2] / 2.0)),
            material=RUBBER_BLACK,
            name=name_block,
        )

    # =========================================================================
    # BEAM: Single rocking beam with seats and handlebars
    # =========================================================================
    beam = model.part("beam")

    # Main beam tube (along X, centered at origin, above pivot axis)
    beam_tube = (
        CylinderGeometry(BEAM_TUBE_R, BEAM_LEN, radial_segments=18)
        .rotate_y(math.pi / 2.0)
        .translate(0.0, 0.0, MAIN_Z)
    )

    # Axle sleeve at center (wraps around the pivot bar)
    sleeve_len = 0.12
    sleeve_r = 0.030
    axle_sleeve = (
        CylinderGeometry(sleeve_r, sleeve_len, radial_segments=20)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, 0.0, 0.0)
    )
    # Weld post from sleeve top to beam tube bottom
    weld_post_len = MAIN_Z - BEAM_TUBE_R - sleeve_r
    if weld_post_len > 0.005:
        weld_post = CylinderGeometry(0.012, weld_post_len, radial_segments=12).translate(
            0.0, 0.0, sleeve_r + weld_post_len / 2.0
        )
        axle_sleeve.merge(weld_post)

    beam.visual(
        mesh_from_geometry(beam_tube, "beam_tube"),
        material=WORN_YELLOW,
        name="beam_tube",
    )
    beam.visual(
        mesh_from_geometry(axle_sleeve, "axle_sleeve"),
        material=WORN_YELLOW,
        name="axle_sleeve",
    )

    # Seat plates and handlebars at each end
    seat_z = MAIN_Z - BEAM_TUBE_R - SEAT_SIZE[2] / 2.0 + 0.002
    for sx, end_idx in ((1.0, 0), (-1.0, 1)):
        # Seat plate
        beam.visual(
            Box(SEAT_SIZE),
            origin=Origin(xyz=(sx * SEAT_X, 0.0, seat_z)),
            material=RUST_BROWN,
            name=f"seat_plate_{end_idx}",
        )

        # Short seat support bracket from beam tube to seat
        support_mesh = _tube_between(
            (sx * (SEAT_X - 0.05), 0.0, MAIN_Z - BEAM_TUBE_R),
            (sx * SEAT_X, 0.0, seat_z + SEAT_SIZE[2] / 2.0),
            SUPPORT_R,
        )
        beam.visual(
            mesh_from_geometry(support_mesh, f"seat_support_{end_idx}"),
            material=WORN_YELLOW,
            name=f"seat_support_{end_idx}",
        )

        # T-handlebar: vertical post + horizontal crossbar
        post_base_z = MAIN_Z + BEAM_TUBE_R
        post_top_z = HANDLE_TOP_Z  # in beam local frame
        post_h = post_top_z - post_base_z
        if post_h > 0.01:
            post = CylinderGeometry(HANDLE_R, post_h, radial_segments=14).translate(
                sx * HANDLE_X, 0.0, post_base_z + post_h / 2.0
            )
            bar = (
                CylinderGeometry(HANDLE_R, 0.26, radial_segments=14)
                .rotate_x(math.pi / 2.0)
                .translate(sx * HANDLE_X, 0.0, post_top_z)
            )
            handlebar = post.merge(bar)
            beam.visual(
                mesh_from_geometry(handlebar, f"handlebar_{end_idx}"),
                material=WORN_YELLOW,
                name=f"handlebar_{end_idx}",
            )

    # =========================================================================
    # RUBBER END BUMPERS (prismatic compression)
    # =========================================================================
    # Each bumper is a rubber disc mounted under the beam near each end,
    # past the seat. Prismatic joint allows vertical compression.
    # Bumper center in beam-local Z: bumper top slightly overlaps beam tube
    # bottom for a seated contact fit (tiny intentional embed).
    bumper_local_z = MAIN_Z - BEAM_TUBE_R - BUMPER_H / 2.0 + 0.002
    for sx, name in ((1.0, "bumper_0"), (-1.0, "bumper_1")):
        bumper = model.part(name)
        # Rubber disc (box approximation)
        bumper.visual(
            Box((BUMPER_R * 2, BUMPER_R * 2, BUMPER_H)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=RUBBER_BLACK,
            name=f"{name}_pad",
        )
        # Mounting stem: thin cylinder from bumper top up into beam tube
        stem_h = 0.015
        stem_top_local = BUMPER_H / 2.0 + stem_h
        bumper.visual(
            mesh_from_geometry(
                CylinderGeometry(0.008, stem_h, radial_segments=10)
                .translate(0.0, 0.0, BUMPER_H / 2.0 + stem_h / 2.0),
                f"{name}_stem",
            ),
            material=RUBBER_BLACK,
            name=f"{name}_stem",
        )

    # =========================================================================
    # ARTICULATIONS
    # =========================================================================

    # Beam pivot: revolute joint at the A-frame apex
    model.articulation(
        "beam_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_HEIGHT)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=100.0, velocity=2.0, lower=-TILT, upper=TILT),
    )

    # Bumper prismatic joints: vertical compression under beam ends
    for idx, (sx, bumper_name) in enumerate(zip((1.0, -1.0), ("bumper_0", "bumper_1"))):
        bumper_part = model.get_part(bumper_name)
        model.articulation(
            f"bumper_{idx}_slide",
            ArticulationType.PRISMATIC,
            parent=beam,
            child=bumper_part,
            origin=Origin(xyz=(sx * BUMPER_X, 0.0, bumper_local_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=200.0,
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
    beam_pivot = object_model.get_articulation("beam_pivot")
    bumper_0_slide = object_model.get_articulation("bumper_0_slide")
    bumper_1_slide = object_model.get_articulation("bumper_1_slide")

    # --- Triangular A-frame support -------------------------------------------
    leg_left = base.get_visual("leg_left")
    leg_right = base.get_visual("leg_right")
    ctx.check(
        "base has triangular support legs",
        leg_left is not None and leg_right is not None,
        details="leg_left and leg_right visuals must exist on the base",
    )

    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "base apex is about 0.45 m tall",
        base_aabb is not None and 0.40 <= base_aabb[1][2] <= 0.55,
        details=f"base aabb top z={base_aabb[1][2] if base_aabb else None}",
    )
    ctx.check(
        "base feet rest on the ground",
        base_aabb is not None and -0.01 <= base_aabb[0][2] <= 0.02,
        details=f"base aabb bottom z={base_aabb[0][2] if base_aabb else None}",
    )

    # --- Rubber ground pads under support legs --------------------------------
    pad_0 = base.get_visual("ground_pad_0")
    pad_1 = base.get_visual("ground_pad_1")
    ctx.check(
        "rubber ground pads exist under both support legs",
        pad_0 is not None and pad_1 is not None,
        details="ground_pad_0 and ground_pad_1 must exist on the base",
    )
    pad_0_aabb = ctx.part_element_world_aabb(base, elem="ground_pad_0")
    pad_1_aabb = ctx.part_element_world_aabb(base, elem="ground_pad_1")
    ctx.check(
        "ground pads sit at ground level",
        pad_0_aabb is not None
        and pad_1_aabb is not None
        and pad_0_aabb[0][2] < 0.02
        and pad_1_aabb[0][2] < 0.02,
        details=f"pad_0 z={pad_0_aabb}, pad_1 z={pad_1_aabb}",
    )

    # --- Safety bump stops below beam ends ------------------------------------
    bump_stop_0 = base.get_visual("bump_stop_0")
    bump_stop_1 = base.get_visual("bump_stop_1")
    ctx.check(
        "safety bump stops exist on the base below beam ends",
        bump_stop_0 is not None and bump_stop_1 is not None,
        details="bump_stop_0 and bump_stop_1 must exist on the base",
    )
    bs_0_aabb = ctx.part_element_world_aabb(base, elem="bump_stop_0")
    bs_1_aabb = ctx.part_element_world_aabb(base, elem="bump_stop_1")
    ctx.check(
        "bump stops are positioned between ground and pivot height",
        bs_0_aabb is not None
        and bs_1_aabb is not None
        and 0.05 < bs_0_aabb[1][2] < PIVOT_HEIGHT
        and 0.05 < bs_1_aabb[1][2] < PIVOT_HEIGHT,
        details=f"bump_stop_0 top={bs_0_aabb[1][2] if bs_0_aabb else None}, "
                f"bump_stop_1 top={bs_1_aabb[1][2] if bs_1_aabb else None}",
    )

    # --- Beam pivot (revolute joint) ------------------------------------------
    ctx.check(
        "beam pivot is a revolute joint",
        beam_pivot.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={beam_pivot.articulation_type}",
    )
    lim = beam_pivot.motion_limits
    ctx.check(
        "beam pivot rocks +/- 15 degrees",
        lim is not None
        and lim.lower is not None
        and lim.upper is not None
        and abs(lim.lower + TILT) < 1e-6
        and abs(lim.upper - TILT) < 1e-6,
        details=f"limits=({lim.lower if lim else None}, {lim.upper if lim else None})",
    )

    # --- Rubber end bumpers on prismatic joints -------------------------------
    ctx.check(
        "bumper_0 slide is a prismatic joint",
        bumper_0_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={bumper_0_slide.articulation_type}",
    )
    ctx.check(
        "bumper_1 slide is a prismatic joint",
        bumper_1_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={bumper_1_slide.articulation_type}",
    )
    for slide, bname in ((bumper_0_slide, "bumper_0"), (bumper_1_slide, "bumper_1")):
        slim = slide.motion_limits
        ctx.check(
            f"{bname} prismatic joint has compression travel",
            slim is not None
            and slim.lower is not None
            and slim.upper is not None
            and abs(slim.lower) < 1e-6
            and slim.upper > 0.01,
            details=f"limits=({slim.lower if slim else None}, {slim.upper if slim else None})",
        )

    # --- Seats and handlebars exist ------------------------------------------
    for end in (0, 1):
        seat = ctx.part_element_world_aabb(beam, elem=f"seat_plate_{end}")
        handle = ctx.part_element_world_aabb(beam, elem=f"handlebar_{end}")
        ctx.check(
            f"beam end {end} has a seat plate and handlebar",
            seat is not None and handle is not None,
            details=f"seat={seat}, handle={handle}",
        )

    # --- Axle sleeve wraps pivot bar and meets converging legs ----------------
    # The axle sleeve intentionally wraps the pivot bar, and the legs converge
    # to the same apex point where the sleeve sits.
    ctx.allow_overlap(
        beam,
        base,
        elem_a="axle_sleeve",
        elem_b="pivot_bar",
        reason="Beam axle sleeve intentionally wraps the pivot bar at the A-frame apex.",
    )
    ctx.allow_overlap(
        beam,
        base,
        elem_a="axle_sleeve",
        elem_b="leg_left",
        reason="Left leg converges to the apex pivot point wrapped by the axle sleeve.",
    )
    ctx.allow_overlap(
        beam,
        base,
        elem_a="axle_sleeve",
        elem_b="leg_right",
        reason="Right leg converges to the apex pivot point wrapped by the axle sleeve.",
    )
    ctx.expect_contact(
        beam,
        base,
        elem_a="axle_sleeve",
        elem_b="pivot_bar",
        name="beam sleeve rides on the pivot bar",
    )

    # Bumper stems are seated into the beam tube (prismatic mounting).
    for bname in ("bumper_0", "bumper_1"):
        bumper_part = object_model.get_part(bname)
        ctx.allow_overlap(
            beam,
            bumper_part,
            elem_a="beam_tube",
            elem_b=f"{bname}_stem",
            reason=f"{bname} mounting stem is intentionally seated into the beam tube for prismatic guidance.",
        )
        ctx.expect_contact(
            beam,
            bumper_part,
            elem_a="beam_tube",
            elem_b=f"{bname}_pad",
            name=f"{bname} pad is seated against the beam tube",
        )

    # --- Decisive pose check: beam rocks correctly ----------------------------
    rest_seat_0 = ctx.part_element_world_aabb(beam, elem="seat_plate_0")
    rest_seat_1 = ctx.part_element_world_aabb(beam, elem="seat_plate_1")

    with ctx.pose({beam_pivot: TILT}):
        tilt_seat_0 = ctx.part_element_world_aabb(beam, elem="seat_plate_0")
        tilt_seat_1 = ctx.part_element_world_aabb(beam, elem="seat_plate_1")
        beam_aabb = ctx.part_world_aabb(beam)
        ctx.check(
            "beam seesaws: one end drops while the other rises",
            rest_seat_0 is not None
            and tilt_seat_0 is not None
            and rest_seat_1 is not None
            and tilt_seat_1 is not None
            and tilt_seat_0[0][2] < rest_seat_0[0][2] - 0.10
            and tilt_seat_1[0][2] > rest_seat_1[0][2] + 0.10,
            details=f"seat0 {rest_seat_0[0][2]:.3f}->{tilt_seat_0[0][2]:.3f}, "
                    f"seat1 {rest_seat_1[0][2]:.3f}->{tilt_seat_1[0][2]:.3f}",
        )
        ctx.check(
            "tilted beam stays clear of the ground",
            beam_aabb is not None and beam_aabb[0][2] > 0.02,
            details=f"beam aabb bottom z={beam_aabb[0][2] if beam_aabb else None}",
        )
        ctx.expect_contact(
            beam,
            base,
            elem_a="axle_sleeve",
            elem_b="pivot_bar",
            name="tilted beam sleeve stays on pivot bar",
        )

    with ctx.pose({beam_pivot: -TILT}):
        tilt_neg_seat_0 = ctx.part_element_world_aabb(beam, elem="seat_plate_0")
        ctx.check(
            "beam seesaws the other way: seat 0 rises",
            rest_seat_0 is not None
            and tilt_neg_seat_0 is not None
            and tilt_neg_seat_0[0][2] > rest_seat_0[0][2] + 0.10,
            details=f"seat0 rest={rest_seat_0[0][2]:.3f}, neg tilt={tilt_neg_seat_0[0][2]:.3f}",
        )

    # --- Bumper compression check ---------------------------------------------
    rest_bumper_0_pos = ctx.part_world_position(bumper_0)
    with ctx.pose({bumper_0_slide: BUMPER_TRAVEL}):
        compressed_bumper_0_pos = ctx.part_world_position(bumper_0)
    ctx.check(
        "bumper_0 compresses upward when prismatic joint actuated",
        rest_bumper_0_pos is not None
        and compressed_bumper_0_pos is not None
        and compressed_bumper_0_pos[2] > rest_bumper_0_pos[2] + 0.005,
        details=f"rest z={rest_bumper_0_pos[2] if rest_bumper_0_pos else None}, "
                f"compressed z={compressed_bumper_0_pos[2] if compressed_bumper_0_pos else None}",
    )

    return ctx.report()


object_model = build_object_model()
