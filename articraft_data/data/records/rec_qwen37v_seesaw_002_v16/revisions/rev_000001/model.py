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
    tube_from_spline_points,
)

# ----------------------------------------------------------------------------
# Variant 16 – Curved-beam playground seesaw with spring, bump stops, and
# rubber ground pads.
#
# Layout (world frame, Z up, base centered on the origin):
# - Sky-blue base: two parallel inverted-U arches (in YZ plane, spaced in X)
#   joined by cross braces, about 0.65 m tall. Four rubber ground pads sit
#   under the arch feet. Two bump-stop brackets extend downward from the arch
#   tops to catch the beam at max tilt.
# - One curved yellow beam (~2.6 m) running in X, with both ends raised ~0.15 m
#   above center (banana / smile shape). A seat plate and T-handlebar at each
#   end. The beam pivots on a revolute joint at the arch top, with an axle
#   sleeve wrapping both arches at the pivot.
# - A central helical coil spring hangs below the beam on vertical support
#   tubes from the arch top, on a prismatic joint (vertical compression).
# ----------------------------------------------------------------------------

TUBE_R = 0.020          # ~40 mm diameter main tubing
BRACE_R = 0.016
SUPPORT_R = 0.018
HANDLE_R = 0.016

PIVOT_Z = 0.65          # pivot axis height
ARCH_HALF_SPAN = 0.36   # arch leg spread in Y
ARCH_X_SEP = 0.08       # half-spacing of the two parallel arches in X

BEAM_HALF_LEN = 1.30
BEAM_RISE = 0.15        # end rise above center (curved beam)
SEAT_X = 1.22           # seat plate center along beam from pivot
HANDLE_X = 0.88         # handlebar post, inboard of seat
HANDLE_TOP_Z = 0.37     # handlebar crossbar height above pivot (in beam local)
SEAT_Z_OFFSET = 0.10    # seat plate above beam tube at end

SEAT_SIZE = (0.26, 0.30, 0.012)
TILT = math.radians(18.0)

# Axle sleeve at the pivot
SLEEVE_R = 0.032
SLEEVE_LEN = 0.20       # spans both arches (from -0.08 to +0.08 plus margin)

# Spring
SPRING_COIL_R = 0.028
SPRING_WIRE_R = 0.005
SPRING_HEIGHT = 0.10
SPRING_TURNS = 5
SPRING_TRAVEL = 0.04    # max compression distance

# Bump stops
BUMP_X = 1.15           # x position along beam axis
BUMP_Z = 0.33           # world z of bump stop top surface
BUMP_SIZE = (0.06, 0.06, 0.04)

# Ground pads
PAD_SIZE = (0.08, 0.10, 0.012)

# Materials
SKY_BLUE = Material("sky_blue_paint", rgba=(0.33, 0.62, 0.84, 1.0))
WORN_YELLOW = Material("worn_yellow_paint", rgba=(0.87, 0.74, 0.12, 1.0))
RUST_BROWN = Material("rust_brown_steel", rgba=(0.42, 0.21, 0.13, 1.0))
DARK_RUBBER = Material("dark_rubber", rgba=(0.12, 0.12, 0.10, 1.0))
SPRING_STEEL = Material("spring_steel", rgba=(0.55, 0.55, 0.52, 1.0))


# ── helpers ────────────────────────────────────────────────────────────────

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


def _arch_mesh(x_offset: float, top_z: float) -> MeshGeometry:
    """One inverted-U arch tube in the YZ plane, shifted to x_offset in X."""
    shoulder = top_z - 0.13
    profile_yz = [
        (-ARCH_HALF_SPAN - 0.05, 0.022),
        (-ARCH_HALF_SPAN - 0.02, 0.030),
        (-0.34, 0.11),
        (-0.28, 0.30),
        (-0.20, shoulder),
        (-0.07, top_z),
        (0.0, top_z),
        (0.07, top_z),
        (0.20, shoulder),
        (0.28, 0.30),
        (0.34, 0.11),
        (ARCH_HALF_SPAN + 0.02, 0.030),
        (ARCH_HALF_SPAN + 0.05, 0.022),
    ]
    points = [(x_offset, y, z) for (y, z) in profile_yz]
    return tube_from_spline_points(
        points,
        radius=TUBE_R,
        samples_per_segment=10,
        radial_segments=16,
        cap_ends=True,
    )


def _curved_beam_mesh() -> MeshGeometry:
    """Curved tube beam with raised ends (banana shape), local X along beam."""
    pts = [
        (-BEAM_HALF_LEN, 0.0, BEAM_RISE),
        (-BEAM_HALF_LEN * 0.75, 0.0, BEAM_RISE * 0.45),
        (-BEAM_HALF_LEN * 0.50, 0.0, BEAM_RISE * 0.12),
        (-BEAM_HALF_LEN * 0.25, 0.0, BEAM_RISE * 0.02),
        (0.0, 0.0, 0.0),
        (BEAM_HALF_LEN * 0.25, 0.0, BEAM_RISE * 0.02),
        (BEAM_HALF_LEN * 0.50, 0.0, BEAM_RISE * 0.12),
        (BEAM_HALF_LEN * 0.75, 0.0, BEAM_RISE * 0.45),
        (BEAM_HALF_LEN, 0.0, BEAM_RISE),
    ]
    beam = tube_from_spline_points(
        pts,
        radius=TUBE_R,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )
    # Diagonal brace tubes from center region toward each end for truss look
    for sx in (1.0, -1.0):
        beam.merge(
            _tube_between(
                (sx * 0.04, 0.0, 0.005),
                (sx * 0.55, 0.0, BEAM_RISE * 0.15),
                BRACE_R,
            )
        )
        beam.merge(
            _tube_between(
                (sx * 0.50, 0.0, BEAM_RISE * 0.12),
                (sx * 0.95, 0.0, BEAM_RISE * 0.60),
                BRACE_R,
            )
        )
        # Seat support tube from beam tube up into the seat plate
        seat_z_local = BEAM_RISE + SEAT_Z_OFFSET  # seat plate center z
        beam.merge(
            tube_from_spline_points(
                [
                    (sx * 1.10, 0.0, BEAM_RISE * 0.80),
                    (sx * 1.18, 0.0, BEAM_RISE * 0.95),
                    (sx * SEAT_X, 0.0, seat_z_local),
                ],
                radius=SUPPORT_R,
                samples_per_segment=8,
                radial_segments=14,
                cap_ends=True,
            )
        )
    return beam


def _spring_mesh() -> MeshGeometry:
    """Helical coil spring, axis along local Z, base at z=0."""
    points = []
    n = SPRING_TURNS * 36 + 1
    for i in range(n):
        t = i / (n - 1)
        angle = t * SPRING_TURNS * 2.0 * math.pi
        x = SPRING_COIL_R * math.cos(angle)
        y = SPRING_COIL_R * math.sin(angle)
        z = t * SPRING_HEIGHT
        points.append((x, y, z))
    return tube_from_spline_points(
        points,
        radius=SPRING_WIRE_R,
        samples_per_segment=3,
        radial_segments=8,
        cap_ends=False,
    )


# ── build ──────────────────────────────────────────────────────────────────

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="curved_beam_spring_seesaw")

    # ── base (static) ────────────────────────────────────────────────────
    base = model.part("base")

    # Two parallel arches
    base.visual(
        mesh_from_geometry(_arch_mesh(-ARCH_X_SEP, PIVOT_Z), "arch_front"),
        material=SKY_BLUE,
        name="arch_front",
    )
    base.visual(
        mesh_from_geometry(_arch_mesh(ARCH_X_SEP, PIVOT_Z), "arch_rear"),
        material=SKY_BLUE,
        name="arch_rear",
    )

    # Cross braces tying the two arches together
    for y_frac, z_height in [(0.75, 0.30), (0.55, 0.48)]:
        y_pos = ARCH_HALF_SPAN * y_frac
        for sy in (1.0, -1.0):
            brace = _tube_between(
                (-ARCH_X_SEP - 0.01, sy * y_pos, z_height),
                (ARCH_X_SEP + 0.01, sy * y_pos, z_height),
                BRACE_R,
            )
            base.visual(
                mesh_from_geometry(brace, f"cross_brace_{sy:.0f}_{z_height:.2f}"),
                material=SKY_BLUE,
                name=f"cross_brace_{sy:.0f}_{z_height:.2f}",
            )

    # Spring support: V-shaped diagonal tubes from the upper cross braces
    # converging at a central mount point below the beam. This avoids the
    # axle sleeve region at the arch top.
    spring_top_z = PIVOT_Z - TUBE_R - 0.020  # clearance below beam bottom
    spring_mount_z = spring_top_z - SPRING_HEIGHT  # base of spring coil
    spring_junction_z = spring_mount_z  # where the V-supports meet
    for sy in (1.0, -1.0):
        diag_tube = _tube_between(
            (0.0, sy * ARCH_HALF_SPAN * 0.55, 0.48),
            (0.0, 0.0, spring_junction_z),
            SUPPORT_R,
        )
        base.visual(
            mesh_from_geometry(diag_tube, f"spring_support_{'pos' if sy > 0 else 'neg'}"),
            material=SKY_BLUE,
            name=f"spring_support_{'pos' if sy > 0 else 'neg'}",
        )
    # Spring mount plate at the junction
    base.visual(
        Box((0.08, 0.08, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, spring_mount_z - 0.004)),
        material=SKY_BLUE,
        name="spring_mount_plate",
    )

    # Rubber ground pads under each arch foot (4 pads total)
    arch_foot_y = ARCH_HALF_SPAN + 0.04
    pad_idx = 0
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            foot_x = sx * ARCH_X_SEP
            foot_y = sy * arch_foot_y
            base.visual(
                Box(PAD_SIZE),
                origin=Origin(xyz=(foot_x, foot_y, PAD_SIZE[2] / 2.0)),
                material=DARK_RUBBER,
                name=f"ground_pad_{pad_idx}",
            )
            pad_idx += 1

    # Bump-stop brackets: diagonal tubes from the arch legs (below the beam
    # path) outward to rubber blocks positioned below the beam travel at max
    # tilt. Starting from the arch legs avoids intersection with the beam tube.
    for sx in (-1.0, 1.0):
        bracket_z = BUMP_Z - BUMP_SIZE[2] / 2.0
        # Start from the arch leg at z≈0.30, offset in Y from the beam path
        bracket_arm = _tube_between(
            (sx * ARCH_X_SEP, 0.28, 0.30),
            (sx * BUMP_X, 0.0, bracket_z),
            SUPPORT_R,
        )
        base.visual(
            mesh_from_geometry(bracket_arm, f"bump_bracket_{'pos' if sx > 0 else 'neg'}"),
            material=SKY_BLUE,
            name=f"bump_bracket_{'pos' if sx > 0 else 'neg'}",
        )
        # Rubber bump stop block at the end of the bracket
        base.visual(
            Box(BUMP_SIZE),
            origin=Origin(xyz=(sx * BUMP_X, 0.0, BUMP_Z - BUMP_SIZE[2] / 2.0)),
            material=DARK_RUBBER,
            name=f"bump_stop_{'pos' if sx > 0 else 'neg'}",
        )

    # ── curved beam ──────────────────────────────────────────────────────
    beam = model.part("beam")
    beam.visual(
        mesh_from_geometry(_curved_beam_mesh(), "beam_truss"),
        material=WORN_YELLOW,
        name="beam_truss",
    )

    # Axle sleeve at the pivot, wrapping both arch tubes
    sleeve = (
        CylinderGeometry(SLEEVE_R, SLEEVE_LEN, radial_segments=20)
        .rotate_y(math.pi / 2.0)
    )
    beam.visual(
        mesh_from_geometry(sleeve, "axle_sleeve"),
        material=WORN_YELLOW,
        name="axle_sleeve",
    )

    # Seats and handlebars at each end
    beam_end_z = BEAM_RISE  # beam tube center z at the end, in local frame
    for idx, sx in enumerate((1.0, -1.0)):
        seat_z = beam_end_z + SEAT_Z_OFFSET
        beam.visual(
            Box(SEAT_SIZE),
            origin=Origin(xyz=(sx * SEAT_X, 0.0, seat_z)),
            material=RUST_BROWN,
            name=f"seat_plate_{idx}",
        )
        # Handlebar post and crossbar
        post_h = HANDLE_TOP_Z
        post = CylinderGeometry(HANDLE_R, post_h, radial_segments=14).translate(
            sx * HANDLE_X, 0.0, post_h / 2.0
        )
        bar = (
            CylinderGeometry(HANDLE_R, 0.30, radial_segments=14)
            .rotate_x(math.pi / 2.0)
            .translate(sx * HANDLE_X, 0.0, HANDLE_TOP_Z)
        )
        beam.visual(
            mesh_from_geometry(post.merge(bar), f"handlebar_{idx}"),
            material=WORN_YELLOW,
            name=f"handlebar_{idx}",
        )

    # ── spring ───────────────────────────────────────────────────────────
    spring = model.part("spring")
    spring.visual(
        mesh_from_geometry(_spring_mesh(), "spring_coil"),
        material=SPRING_STEEL,
        name="spring_coil",
    )
    # Small cap plate on top of the spring (contacts beam underside)
    spring.visual(
        mesh_from_geometry(
            CylinderGeometry(0.025, 0.006, radial_segments=16)
            .translate(0.0, 0.0, SPRING_HEIGHT + 0.003),
            "spring_cap",
        ),
        material=SPRING_STEEL,
        name="spring_cap",
    )

    # ── articulations ────────────────────────────────────────────────────

    # 1. Beam revolute pivot: horizontal axis perpendicular to beam (Y axis)
    model.articulation(
        "beam_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=150.0, velocity=2.5, lower=-TILT, upper=TILT
        ),
    )

    # 2. Spring prismatic: vertical compression under the beam
    model.articulation(
        "spring_compress",
        ArticulationType.PRISMATIC,
        parent=base,
        child=spring,
        origin=Origin(xyz=(0.0, 0.0, spring_mount_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=200.0, velocity=0.5, lower=0.0, upper=SPRING_TRAVEL
        ),
    )

    return model


# ── tests ──────────────────────────────────────────────────────────────────

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    beam = object_model.get_part("beam")
    spring = object_model.get_part("spring")
    beam_pivot = object_model.get_articulation("beam_pivot")
    spring_joint = object_model.get_articulation("spring_compress")

    # ── axle sleeve and beam tube intentionally intersect arch pivot tubes ──
    ctx.allow_overlap(
        beam,
        base,
        elem_a="axle_sleeve",
        elem_b="arch_front",
        reason="Beam axle sleeve wraps the front arch top tube as its pivot axle.",
    )
    ctx.allow_overlap(
        beam,
        base,
        elem_a="axle_sleeve",
        elem_b="arch_rear",
        reason="Beam axle sleeve wraps the rear arch top tube as its pivot axle.",
    )
    ctx.allow_overlap(
        beam,
        base,
        elem_a="beam_truss",
        elem_b="arch_front",
        reason="Beam main tube crosses through the front arch top at the pivot intersection.",
    )
    ctx.allow_overlap(
        beam,
        base,
        elem_a="beam_truss",
        elem_b="arch_rear",
        reason="Beam main tube crosses through the rear arch top at the pivot intersection.",
    )
    # Spring coil sits at the junction of the V-support tubes; small local
    # overlap at the mounting interface is expected.
    ctx.allow_overlap(
        base,
        spring,
        elem_a="spring_support_pos",
        elem_b="spring_coil",
        reason="Spring coil base sits at the V-support junction; local contact overlap at mount interface.",
    )
    ctx.allow_overlap(
        base,
        spring,
        elem_a="spring_support_neg",
        elem_b="spring_coil",
        reason="Spring coil base sits at the V-support junction; local contact overlap at mount interface.",
    )
    ctx.expect_contact(
        beam,
        base,
        elem_a="axle_sleeve",
        elem_b="arch_front",
        name="beam sleeve rides on the front arch axle",
    )
    ctx.expect_contact(
        beam,
        base,
        elem_a="axle_sleeve",
        elem_b="arch_rear",
        name="beam sleeve rides on the rear arch axle",
    )

    # ── curved beam has raised ends ──────────────────────────────────────
    seat0 = ctx.part_element_world_aabb(beam, elem="seat_plate_0")
    seat1 = ctx.part_element_world_aabb(beam, elem="seat_plate_1")
    truss = ctx.part_element_world_aabb(beam, elem="beam_truss")
    ctx.check(
        "curved beam has raised ends: seats sit above beam center region",
        seat0 is not None and seat1 is not None and truss is not None
        and seat0[0][2] > truss[0][2] + 0.05
        and seat1[0][2] > truss[0][2] + 0.05,
        details=f"seat0={seat0}, seat1={seat1}, truss={truss}",
    )

    # ── rubber ground pads exist under base feet ────────────────────────
    pad_count = 0
    for pad_name in ("ground_pad_0", "ground_pad_1", "ground_pad_2", "ground_pad_3"):
        pad_aabb = ctx.part_element_world_aabb(base, elem=pad_name)
        if pad_aabb is not None and pad_aabb[0][2] < 0.02:
            pad_count += 1
    ctx.check(
        "rubber ground pads sit on the ground under base feet",
        pad_count >= 4,
        details=f"found {pad_count} ground pads at ground level",
    )

    # ── bump stops exist below beam ends ─────────────────────────────────
    for side in ("pos", "neg"):
        bump = ctx.part_element_world_aabb(base, elem=f"bump_stop_{side}")
        ctx.check(
            f"bump stop on {side} side exists below beam travel path",
            bump is not None
            and bump[1][2] < PIVOT_Z  # below pivot height
            and bump[0][2] > 0.05,    # above ground
            details=f"bump_stop_{side}={bump}",
        )

    # ── spring prismatic joint ──────────────────────────────────────────
    ctx.check(
        "spring has a prismatic compression joint",
        spring_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"spring joint type={spring_joint.articulation_type}",
    )
    lim = spring_joint.motion_limits
    ctx.check(
        "spring joint has compression travel",
        lim is not None and lim.upper > 0.0 and lim.upper <= SPRING_TRAVEL + 1e-6,
        details=f"spring limits=({lim.lower if lim else None}, {lim.upper if lim else None})",
    )

    # Spring part is near the beam center, below pivot
    spring_aabb = ctx.part_world_aabb(spring)
    ctx.check(
        "spring sits centrally below the pivot axis",
        spring_aabb is not None
        and abs(spring_aabb[0][0] + spring_aabb[1][0]) / 2.0 < 0.10
        and spring_aabb[1][2] < PIVOT_Z + 0.01
        and spring_aabb[0][2] > 0.20,
        details=f"spring aabb={spring_aabb}",
    )

    # ── beam revolute pivot ─────────────────────────────────────────────
    ctx.check(
        "beam pivot is revolute with +/-18 degree limits",
        beam_pivot.articulation_type == ArticulationType.REVOLUTE
        and beam_pivot.motion_limits is not None
        and abs(beam_pivot.motion_limits.lower + TILT) < 1e-6
        and abs(beam_pivot.motion_limits.upper - TILT) < 1e-6,
        details=f"type={beam_pivot.articulation_type}, limits=({beam_pivot.motion_limits.lower if beam_pivot.motion_limits else None}, {beam_pivot.motion_limits.upper if beam_pivot.motion_limits else None})",
    )

    # ── base is the right height and grounded ───────────────────────────
    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "base arch is about 0.65 m tall",
        base_aabb is not None and 0.60 <= base_aabb[1][2] <= 0.72,
        details=f"base aabb={base_aabb}",
    )
    ctx.check(
        "base feet rest on the ground plane",
        base_aabb is not None and -0.01 <= base_aabb[0][2] <= 0.02,
        details=f"base aabb={base_aabb}",
    )

    # ── seesaw rocking: decisive pose checks ─────────────────────────────
    rest_seat0 = ctx.part_element_world_aabb(beam, elem="seat_plate_0")
    rest_seat1 = ctx.part_element_world_aabb(beam, elem="seat_plate_1")

    with ctx.pose({beam_pivot: TILT}):
        tilt_seat0 = ctx.part_element_world_aabb(beam, elem="seat_plate_0")
        tilt_seat1 = ctx.part_element_world_aabb(beam, elem="seat_plate_1")
        ctx.check(
            "beam seesaws: positive tilt drops seat 0 and raises seat 1",
            rest_seat0 is not None and tilt_seat0 is not None
            and rest_seat1 is not None and tilt_seat1 is not None
            and tilt_seat0[1][2] < rest_seat0[1][2] - 0.20
            and tilt_seat1[1][2] > rest_seat1[1][2] + 0.20,
            details=f"seat0 {rest_seat0} -> {tilt_seat0}, seat1 {rest_seat1} -> {tilt_seat1}",
        )
        beam_aabb = ctx.part_world_aabb(beam)
        ctx.check(
            "fully tilted beam stays clear of the ground",
            beam_aabb is not None and beam_aabb[0][2] > 0.02,
            details=f"beam aabb={beam_aabb}",
        )
        ctx.expect_contact(
            beam,
            base,
            elem_a="axle_sleeve",
            elem_b="arch_front",
            name="tilted beam sleeve stays on its axle",
        )

    with ctx.pose({beam_pivot: -TILT}):
        tilt_neg_seat0 = ctx.part_element_world_aabb(beam, elem="seat_plate_0")
        tilt_neg_seat1 = ctx.part_element_world_aabb(beam, elem="seat_plate_1")
        ctx.check(
            "beam seesaws: negative tilt raises seat 0 and drops seat 1",
            rest_seat0 is not None and tilt_neg_seat0 is not None
            and rest_seat1 is not None and tilt_neg_seat1 is not None
            and tilt_neg_seat0[1][2] > rest_seat0[1][2] + 0.20
            and tilt_neg_seat1[1][2] < rest_seat1[1][2] - 0.20,
            details=f"seat0 {rest_seat0} -> {tilt_neg_seat0}, seat1 {rest_seat1} -> {tilt_neg_seat1}",
        )

    # ── spring compresses in prismatic pose ─────────────────────────────
    rest_spring = ctx.part_world_aabb(spring)
    with ctx.pose({spring_joint: SPRING_TRAVEL}):
        compressed_spring = ctx.part_world_aabb(spring)
        ctx.check(
            "spring compresses downward at max prismatic travel",
            rest_spring is not None and compressed_spring is not None
            and compressed_spring[1][2] < rest_spring[1][2] - 0.02,
            details=f"rest={rest_spring}, compressed={compressed_spring}",
        )

    # ── seats and handlebars exist at beam ends ──────────────────────────
    for idx in (0, 1):
        seat_aabb = ctx.part_element_world_aabb(beam, elem=f"seat_plate_{idx}")
        handle_aabb = ctx.part_element_world_aabb(beam, elem=f"handlebar_{idx}")
        ctx.check(
            f"beam end {idx} carries a seat and handlebar",
            seat_aabb is not None and handle_aabb is not None,
            details=f"seat={seat_aabb}, handle={handle_aabb}",
        )
        if seat_aabb and handle_aabb:
            ctx.check(
                f"handlebar {idx} extends above its seat",
                handle_aabb[1][2] > seat_aabb[1][2] + 0.08,
                details=f"handle top={handle_aabb[1][2]:.3f}, seat top={seat_aabb[1][2]:.3f}",
            )

    return ctx.report()


object_model = build_object_model()
