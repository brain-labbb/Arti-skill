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
# Four-seat CROSS seesaw with perpendicular beams (variant 22).
#
# Base: sky-blue tube frame with two arches along world X (low, pivot 0.56 m)
# and world Y (high, pivot 0.74 m), joined by diagonal cross braces. Short
# pivot-axle stubs sit on each arch top, perpendicular to their beam.
# Rubber ground pads under the four feet. Rubber bump stops on each axle.
#
# Two perpendicular yellow beams cross above the base. Each carries seats
# with tilting backrests (revolute joints) and T-handlebars.
# Each beam pivots +/-18° on its own revolute joint.
# ----------------------------------------------------------------------------

TUBE_R = 0.020
BRACE_R = 0.016
SUPPORT_R = 0.018
HANDLE_R = 0.016

YAW = math.radians(45.0)  # perpendicular cross beams
TILT = math.radians(18.0)

# Arch geometry: arches run along world axes to avoid crossing beam paths.
ARCH_TOP_LOW = 0.52   # arch tube center Z (arch along X)
ARCH_TOP_HIGH = 0.70  # arch tube center Z (arch along Y)
ARCH_HALF_SPAN = 0.36
CROSS_BRACE_Z = 0.28

# Pivot axle stubs sit on arch tops; overlap arch by ~3 mm for connectivity.
AXLE_R = TUBE_R
AXLE_LEN = 0.22
LOW_PIVOT_Z = ARCH_TOP_LOW + TUBE_R + AXLE_R - 0.003   # ≈ 0.557
HIGH_PIVOT_Z = ARCH_TOP_HIGH + TUBE_R + AXLE_R - 0.003  # ≈ 0.737

BEAM_LEN = 2.60
MAIN_Z = 0.08
SLEEVE_R = 0.032
SLEEVE_LEN = 0.13
SEAT_X = 1.43
SEAT_Z = 0.038
SEAT_SIZE = (0.26, 0.30, 0.012)
HANDLE_X = 1.04
HANDLE_TOP_Z = 0.34

BACKREST_SIZE = (0.010, 0.22, 0.20)  # thick(x), wide(y), tall(z)
BACKREST_LOWER = math.radians(-25.0)
BACKREST_UPPER = math.radians(15.0)

PAD_R = 0.050
PAD_H = 0.010

BUMP_SIZE = (0.024, 0.024, 0.016)

SKY_BLUE = Material("sky_blue_paint", rgba=(0.33, 0.62, 0.84, 1.0))
WORN_YELLOW = Material("worn_yellow_paint", rgba=(0.87, 0.74, 0.12, 1.0))
RUST_BROWN = Material("rust_brown_steel", rgba=(0.42, 0.21, 0.13, 1.0))
RUBBER_BLACK = Material("rubber_black", rgba=(0.12, 0.12, 0.12, 1.0))
BACKREST_GREEN = Material("backrest_green", rgba=(0.20, 0.45, 0.25, 1.0))


def _tube_between(
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    radius: float,
    *,
    radial_segments: int = 16,
) -> MeshGeometry:
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


def _arch_mesh_x(top_z: float) -> MeshGeometry:
    """Inverted-U arch tube running along the X axis."""
    shoulder = top_z - 0.04
    profile = [
        (-ARCH_HALF_SPAN - 0.055, 0.022),
        (-ARCH_HALF_SPAN - 0.03, 0.028),
        (-0.35, 0.10),
        (-0.27, 0.44 if top_z > 0.55 else 0.36),
        (-0.18, shoulder),
        (-0.07, top_z),
        (0.0, top_z),
        (0.07, top_z),
        (0.18, shoulder),
        (0.27, 0.44 if top_z > 0.55 else 0.36),
        (0.35, 0.10),
        (ARCH_HALF_SPAN + 0.03, 0.028),
        (ARCH_HALF_SPAN + 0.055, 0.022),
    ]
    points = [(u, 0.0, z) for (u, z) in profile]
    return tube_from_spline_points(
        points, radius=TUBE_R, samples_per_segment=10,
        radial_segments=16, cap_ends=True,
    )


def _arch_mesh_y(top_z: float) -> MeshGeometry:
    """Inverted-U arch tube running along the Y axis."""
    shoulder = top_z - 0.04
    profile = [
        (-ARCH_HALF_SPAN - 0.055, 0.022),
        (-ARCH_HALF_SPAN - 0.03, 0.028),
        (-0.35, 0.10),
        (-0.27, 0.44 if top_z > 0.55 else 0.36),
        (-0.18, shoulder),
        (-0.07, top_z),
        (0.0, top_z),
        (0.07, top_z),
        (0.18, shoulder),
        (0.27, 0.44 if top_z > 0.55 else 0.36),
        (0.35, 0.10),
        (ARCH_HALF_SPAN + 0.03, 0.028),
        (ARCH_HALF_SPAN + 0.055, 0.022),
    ]
    points = [(0.0, u, z) for (u, z) in profile]
    return tube_from_spline_points(
        points, radius=TUBE_R, samples_per_segment=10,
        radial_segments=16, cap_ends=True,
    )


def _axle_stub(axle_dir: tuple[float, float], z: float) -> MeshGeometry:
    """Short pivot axle tube along axle_dir at height z."""
    half = AXLE_LEN / 2.0
    return _tube_between(
        (-half * axle_dir[0], -half * axle_dir[1], z),
        (half * axle_dir[0], half * axle_dir[1], z),
        AXLE_R,
    )


def _beam_meshes() -> tuple[MeshGeometry, MeshGeometry, MeshGeometry, MeshGeometry]:
    truss = (
        CylinderGeometry(TUBE_R, BEAM_LEN, radial_segments=18)
        .rotate_y(math.pi / 2.0)
        .translate(0.0, 0.0, MAIN_Z)
    )
    for sx in (1.0, -1.0):
        truss.merge(
            _tube_between(
                (sx * 0.04, 0.0, 0.005),
                (sx * 0.60, 0.0, MAIN_Z),
                BRACE_R,
            )
        )
        truss.merge(
            tube_from_spline_points(
                [
                    (sx * 1.24, 0.0, MAIN_Z),
                    (sx * 1.34, 0.0, 0.055),
                    (sx * 1.42, 0.0, 0.020),
                    (sx * 1.49, 0.0, 0.012),
                ],
                radius=SUPPORT_R,
                samples_per_segment=10,
                radial_segments=14,
                cap_ends=True,
            )
        )

    sleeve = (
        CylinderGeometry(SLEEVE_R, SLEEVE_LEN, radial_segments=20)
        .rotate_x(math.pi / 2.0)
    )
    weld_post = CylinderGeometry(0.014, MAIN_Z - 0.024, radial_segments=14).translate(
        0.0, 0.0, (MAIN_Z + 0.024) / 2.0
    )
    sleeve.merge(weld_post)

    handlebars: list[MeshGeometry] = []
    for sx in (1.0, -1.0):
        post = CylinderGeometry(HANDLE_R, 0.28, radial_segments=14).translate(
            sx * HANDLE_X, 0.0, MAIN_Z + 0.13
        )
        bar = (
            CylinderGeometry(HANDLE_R, 0.30, radial_segments=14)
            .rotate_x(math.pi / 2.0)
            .translate(sx * HANDLE_X, 0.0, HANDLE_TOP_Z)
        )
        handlebars.append(post.merge(bar))

    return truss, sleeve, handlebars[0], handlebars[1]


def _backrest_barrel_mesh() -> MeshGeometry:
    return (
        CylinderGeometry(0.009, 0.24, radial_segments=14)
        .rotate_x(math.pi / 2.0)
    )


def _add_beam_part(model: ArticulatedObject, part_name: str):
    truss, sleeve, hb0, hb1 = _beam_meshes()
    beam = model.part(part_name)
    beam.visual(
        mesh_from_geometry(truss, f"{part_name}_truss"),
        material=WORN_YELLOW,
        name="truss_tube",
    )
    beam.visual(
        mesh_from_geometry(sleeve, f"{part_name}_sleeve"),
        material=WORN_YELLOW,
        name="axle_sleeve",
    )
    beam.visual(
        mesh_from_geometry(hb0, f"{part_name}_handlebar_0"),
        material=WORN_YELLOW,
        name="handlebar_0",
    )
    beam.visual(
        mesh_from_geometry(hb1, f"{part_name}_handlebar_1"),
        material=WORN_YELLOW,
        name="handlebar_1",
    )
    beam.visual(
        Box(SEAT_SIZE),
        origin=Origin(xyz=(SEAT_X, 0.0, SEAT_Z)),
        material=RUST_BROWN,
        name="seat_plate_0",
    )
    beam.visual(
        Box(SEAT_SIZE),
        origin=Origin(xyz=(-SEAT_X, 0.0, SEAT_Z)),
        material=RUST_BROWN,
        name="seat_plate_1",
    )
    return beam


def _add_backrest(
    model: ArticulatedObject,
    beam,
    beam_idx: int,
    seat_idx: int,
    sign: float,
) -> tuple:
    barrel = _backrest_barrel_mesh()
    name = f"backrest_{beam_idx}_{seat_idx}"
    backrest = model.part(name)
    backrest.visual(
        Box(BACKREST_SIZE),
        origin=Origin(xyz=(0.0, 0.0, BACKREST_SIZE[2] / 2.0)),
        material=BACKREST_GREEN,
        name="plate",
    )
    backrest.visual(
        mesh_from_geometry(barrel, f"{name}_barrel"),
        material=RUST_BROWN,
        name="hinge_barrel",
    )
    # Hinge at back edge of seat; barrel intentionally overlaps seat edge
    # so the backrest reads as physically mounted on the seat.
    hinge_x = sign * (SEAT_X + 0.13)
    hinge_z = SEAT_Z + 0.006  # seat top surface
    joint = model.articulation(
        f"{name}_tilt",
        ArticulationType.REVOLUTE,
        parent=beam,
        child=backrest,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0,
            lower=BACKREST_LOWER, upper=BACKREST_UPPER,
        ),
    )
    return backrest, joint


def _foot_positions() -> list[tuple[float, float, float]]:
    foot_u = ARCH_HALF_SPAN + 0.055
    positions = []
    # Low arch (along X): feet at (±foot_u, 0, z_foot)
    for u in (-foot_u, foot_u):
        positions.append((u, 0.0, PAD_H / 2.0))
    # High arch (along Y): feet at (0, ±foot_u, z_foot)
    for u in (-foot_u, foot_u):
        positions.append((0.0, u, PAD_H / 2.0))
    return positions


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cross_seesaw")

    # --- base ---------------------------------------------------------------
    base = model.part("base")

    # Arches along world X and Y (avoids crossing beam paths).
    base.visual(
        mesh_from_geometry(_arch_mesh_x(ARCH_TOP_LOW), "low_arch"),
        material=SKY_BLUE,
        name="low_arch",
    )
    base.visual(
        mesh_from_geometry(_arch_mesh_y(ARCH_TOP_HIGH), "high_arch"),
        material=SKY_BLUE,
        name="high_arch",
    )

    # Pivot axle stubs on arch tops, perpendicular to their beams.
    low_axle_dir = (-math.sin(YAW), math.cos(YAW))   # perp to lower beam
    high_axle_dir = (math.sin(YAW), math.cos(YAW))   # perp to upper beam
    base.visual(
        mesh_from_geometry(_axle_stub(low_axle_dir, LOW_PIVOT_Z), "low_axle"),
        material=SKY_BLUE,
        name="low_axle",
    )
    base.visual(
        mesh_from_geometry(_axle_stub(high_axle_dir, HIGH_PIVOT_Z), "high_axle"),
        material=SKY_BLUE,
        name="high_axle",
    )

    # Cross braces: connect adjacent arch legs diagonally.
    brace_u = 0.32
    for sx, sy in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
        brace = _tube_between(
            (sx * brace_u, 0.0, CROSS_BRACE_Z),
            (0.0, sy * brace_u, CROSS_BRACE_Z),
            SUPPORT_R,
        )
        base.visual(
            mesh_from_geometry(brace, f"cross_brace_{sx}_{sy}"),
            material=SKY_BLUE,
            name=f"cross_brace_{sx}_{sy}",
        )

    # Ground pads under each foot.
    for idx, (px, py, pz) in enumerate(_foot_positions()):
        pad = CylinderGeometry(PAD_R, PAD_H, radial_segments=20)
        pad.translate(px, py, pz)
        base.visual(
            mesh_from_geometry(pad, f"ground_pad_{idx}"),
            material=RUBBER_BLACK,
            name=f"ground_pad_{idx}",
        )

    # Bump stops on axle stubs (beyond sleeve ends, on each side).
    bump_offset = SLEEVE_LEN / 2.0 + 0.030  # past sleeve end
    bump_idx = 0
    for axle_dir, pivot_z in ((low_axle_dir, LOW_PIVOT_Z), (high_axle_dir, HIGH_PIVOT_Z)):
        bz = pivot_z + AXLE_R + BUMP_SIZE[2] / 2.0 - 0.005
        for sign in (1.0, -1.0):
            bx = sign * bump_offset * axle_dir[0]
            by = sign * bump_offset * axle_dir[1]
            base.visual(
                Box(BUMP_SIZE),
                origin=Origin(xyz=(bx, by, bz)),
                material=RUBBER_BLACK,
                name=f"bump_stop_{bump_idx}",
            )
            bump_idx += 1

    # --- beams --------------------------------------------------------------
    lower_beam = _add_beam_part(model, "lower_beam")
    upper_beam = _add_beam_part(model, "upper_beam")

    limits = MotionLimits(effort=150.0, velocity=2.5, lower=-TILT, upper=TILT)
    model.articulation(
        "lower_beam_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_beam,
        origin=Origin(xyz=(0.0, 0.0, LOW_PIVOT_Z), rpy=(0.0, 0.0, YAW)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=limits,
    )
    model.articulation(
        "upper_beam_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_beam,
        origin=Origin(xyz=(0.0, 0.0, HIGH_PIVOT_Z), rpy=(0.0, 0.0, -YAW)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=limits,
    )

    # --- backrests ----------------------------------------------------------
    for beam_idx, beam in enumerate((lower_beam, upper_beam)):
        for seat_idx, sign in enumerate((1.0, -1.0)):
            _add_backrest(model, beam, beam_idx, seat_idx, sign)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lower_beam = object_model.get_part("lower_beam")
    upper_beam = object_model.get_part("upper_beam")
    lower_pivot = object_model.get_articulation("lower_beam_pivot")
    upper_pivot = object_model.get_articulation("upper_beam_pivot")

    # --- axle fits: sleeve wraps pivot axle stubs ---------------------------
    ctx.allow_overlap(
        lower_beam, base,
        elem_a="axle_sleeve", elem_b="low_axle",
        reason="Lower beam sleeve wraps the low pivot axle stub.",
    )
    ctx.allow_overlap(
        upper_beam, base,
        elem_a="axle_sleeve", elem_b="high_axle",
        reason="Upper beam sleeve wraps the high pivot axle stub.",
    )
    # Sleeve also envelops the arch tube beneath the axle (sleeve radius
    # 0.032 > axle radius 0.020, and the axle sits on the arch surface).
    ctx.allow_overlap(
        lower_beam, base,
        elem_a="axle_sleeve", elem_b="low_arch",
        reason="Lower beam sleeve envelops the arch tube beneath the axle it wraps.",
    )
    ctx.allow_overlap(
        upper_beam, base,
        elem_a="axle_sleeve", elem_b="high_arch",
        reason="Upper beam sleeve envelops the arch tube beneath the axle it wraps.",
    )
    # Backrest hinge barrels intentionally overlap seat edges (seated hinge).
    for bi in range(2):
        beam = lower_beam if bi == 0 else upper_beam
        for si in range(2):
            br = object_model.get_part(f"backrest_{bi}_{si}")
            ctx.allow_overlap(
                br, beam,
                elem_a="hinge_barrel", elem_b=f"seat_plate_{si}",
                reason=f"Backrest hinge barrel seats into the seat edge as a hinge mount.",
            )

    ctx.expect_contact(
        lower_beam, base,
        elem_a="axle_sleeve", elem_b="low_axle",
        name="lower beam sleeve rides on low axle",
    )
    ctx.expect_contact(
        upper_beam, base,
        elem_a="axle_sleeve", elem_b="high_axle",
        name="upper beam sleeve rides on high axle",
    )
    ctx.expect_contact(
        lower_beam, base,
        elem_a="axle_sleeve", elem_b="low_arch",
        name="lower beam sleeve contacts low arch beneath axle",
    )
    ctx.expect_contact(
        upper_beam, base,
        elem_a="axle_sleeve", elem_b="high_arch",
        name="upper beam sleeve contacts high arch beneath axle",
    )

    # --- base proportions ---------------------------------------------------
    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "base is a support stand about 0.7 m tall",
        base_aabb is not None and 0.60 <= base_aabb[1][2] <= 0.85,
        details=f"base aabb={base_aabb}",
    )
    ctx.check(
        "base feet rest near the ground plane",
        base_aabb is not None and -0.02 <= base_aabb[0][2] <= 0.025,
        details=f"base aabb={base_aabb}",
    )

    # --- perpendicular beams (cross layout) ---------------------------------
    ctx.expect_overlap(
        lower_beam, upper_beam,
        axes="xy", min_overlap=0.5,
        name="beams cross above the base in plan view",
    )
    lo_seat0 = ctx.part_element_world_aabb(lower_beam, elem="seat_plate_0")
    up_seat0 = ctx.part_element_world_aabb(upper_beam, elem="seat_plate_0")
    if lo_seat0 is not None and up_seat0 is not None:
        lo_cx = (lo_seat0[0][0] + lo_seat0[1][0]) / 2.0
        lo_cy = (lo_seat0[0][1] + lo_seat0[1][1]) / 2.0
        up_cx = (up_seat0[0][0] + up_seat0[1][0]) / 2.0
        up_cy = (up_seat0[0][1] + up_seat0[1][1]) / 2.0
        # Perpendicular: dot product of beam seat vectors near zero.
        dot = lo_cx * up_cx + lo_cy * up_cy
        cross = abs(lo_cx * up_cy - lo_cy * up_cx)
        ctx.check(
            "beams are perpendicular (cross layout)",
            abs(dot) < cross * 0.3,
            details=f"dot={dot:.3f}, cross={cross:.3f}",
        )

    # Upper beam pivots above lower beam.
    lo_sleeve = ctx.part_element_world_aabb(lower_beam, elem="axle_sleeve")
    up_sleeve = ctx.part_element_world_aabb(upper_beam, elem="axle_sleeve")
    ctx.check(
        "upper beam pivots above lower beam",
        lo_sleeve is not None
        and up_sleeve is not None
        and (up_sleeve[0][2] + up_sleeve[1][2]) / 2.0
        > (lo_sleeve[0][2] + lo_sleeve[1][2]) / 2.0 + 0.10,
        details=f"lower sleeve={lo_sleeve}, upper sleeve={up_sleeve}",
    )

    # --- rocking range +/- 18° on both pivots --------------------------------
    for pivot in (lower_pivot, upper_pivot):
        lim = pivot.motion_limits
        ctx.check(
            f"{pivot.name} rocks +/- 18 degrees",
            lim is not None
            and abs(lim.lower + TILT) < 1e-6
            and abs(lim.upper - TILT) < 1e-6,
            details=f"limits=({lim.lower if lim else None}, {lim.upper if lim else None})",
        )

    # --- seats and handlebars on all 4 ends ---------------------------------
    for beam, lo_z, hi_z in ((lower_beam, 0.40, 0.70), (upper_beam, 0.55, 0.90)):
        for end in (0, 1):
            seat = ctx.part_element_world_aabb(beam, elem=f"seat_plate_{end}")
            handle = ctx.part_element_world_aabb(beam, elem=f"handlebar_{end}")
            ok = seat is not None and handle is not None
            ctx.check(
                f"{beam.name} end {end} carries a seat and handlebar",
                ok,
                details=f"seat={seat}, handle={handle}",
            )
            if not ok:
                continue
            scz = (seat[0][2] + seat[1][2]) / 2.0
            ctx.check(
                f"{beam.name} seat {end} at sit height",
                lo_z <= scz <= hi_z,
                details=f"seat z={scz:.3f}",
            )
            ctx.check(
                f"{beam.name} handlebar {end} stands upright above seat",
                handle[1][2] > seat[1][2] + 0.15,
                details=f"handle top={handle[1][2]:.3f}",
            )

    # --- ground pads --------------------------------------------------------
    pad_count = sum(
        1 for idx in range(4)
        if (p := ctx.part_element_world_aabb(base, elem=f"ground_pad_{idx}")) is not None
        and p[0][2] < 0.02 and p[1][2] < 0.03
    )
    ctx.check(
        "four rubber ground pads under support legs",
        pad_count == 4,
        details=f"found {pad_count} pads near ground",
    )

    # --- bump stops ---------------------------------------------------------
    bump_count = sum(
        1 for idx in range(4)
        if (b := ctx.part_element_world_aabb(base, elem=f"bump_stop_{idx}")) is not None
        and 0.40 < (b[0][2] + b[1][2]) / 2.0 < 0.85
    )
    ctx.check(
        "four safety bump stops on base below beam paths",
        bump_count == 4,
        details=f"found {bump_count} bump stops at beam height",
    )

    # --- backrest tilt joints -----------------------------------------------
    backrest_joints = []
    for bi in range(2):
        for si in range(2):
            try:
                j = object_model.get_articulation(f"backrest_{bi}_{si}_tilt")
                backrest_joints.append(j)
            except Exception:
                pass
    ctx.check(
        "four backrest tilt joints exist",
        len(backrest_joints) == 4,
        details=f"found {len(backrest_joints)}",
    )
    for j in backrest_joints:
        lim = j.motion_limits
        ctx.check(
            f"{j.name} has non-trivial tilt range",
            lim is not None and lim.upper - lim.lower > math.radians(10.0),
            details=f"range={math.degrees(lim.upper - lim.lower):.1f} deg",
        )

    # Backrest plates exist and are tall enough.
    for bi in range(2):
        for si in range(2):
            bp = object_model.get_part(f"backrest_{bi}_{si}")
            pa = ctx.part_element_world_aabb(bp, elem="plate")
            ctx.check(
                f"backrest_{bi}_{si} has a visible plate",
                pa is not None and (pa[1][2] - pa[0][2]) > 0.10,
                details=f"plate aabb={pa}",
            )

    # --- decisive pose: lower beam seesaws ----------------------------------
    rest_lo0 = ctx.part_element_world_aabb(lower_beam, elem="seat_plate_0")
    rest_lo1 = ctx.part_element_world_aabb(lower_beam, elem="seat_plate_1")
    rest_up0 = ctx.part_element_world_aabb(upper_beam, elem="seat_plate_0")
    with ctx.pose({lower_pivot: TILT}):
        tilt_lo0 = ctx.part_element_world_aabb(lower_beam, elem="seat_plate_0")
        tilt_lo1 = ctx.part_element_world_aabb(lower_beam, elem="seat_plate_1")
        tilt_up0 = ctx.part_element_world_aabb(upper_beam, elem="seat_plate_0")
        beam_aabb = ctx.part_world_aabb(lower_beam)
        ctx.check(
            "lower beam seesaws: one seat drops, opposite rises",
            rest_lo0 is not None and tilt_lo0 is not None
            and rest_lo1 is not None and tilt_lo1 is not None
            and tilt_lo0[0][2] < rest_lo0[0][2] - 0.25
            and tilt_lo1[0][2] > rest_lo1[0][2] + 0.25,
            details=f"seat0 {rest_lo0}->{tilt_lo0}, seat1 {rest_lo1}->{tilt_lo1}",
        )
        ctx.check(
            "tilted lower beam stays above ground",
            beam_aabb is not None and beam_aabb[0][2] > 0.005,
            details=f"lower beam aabb={beam_aabb}",
        )
        ctx.check(
            "beams rock independently: upper holds while lower rocks",
            rest_up0 is not None and tilt_up0 is not None
            and abs(tilt_up0[0][2] - rest_up0[0][2]) < 1e-6,
            details=f"upper seat0 {rest_up0}->{tilt_up0}",
        )
        ctx.expect_contact(
            lower_beam, base,
            elem_a="axle_sleeve", elem_b="low_axle",
            name="tilted lower beam sleeve stays on axle",
        )
    # --- decisive pose: upper beam seesaws ----------------------------------
    with ctx.pose({upper_pivot: -TILT}):
        tilt_up0 = ctx.part_element_world_aabb(upper_beam, elem="seat_plate_0")
        beam_aabb = ctx.part_world_aabb(upper_beam)
        ctx.check(
            "upper beam seesaws: near seat rises",
            rest_up0 is not None and tilt_up0 is not None
            and tilt_up0[0][2] > rest_up0[0][2] + 0.25,
            details=f"upper seat0 {rest_up0}->{tilt_up0}",
        )
        ctx.check(
            "tilted upper beam stays above ground",
            beam_aabb is not None and beam_aabb[0][2] > 0.005,
            details=f"upper beam aabb={beam_aabb}",
        )
        ctx.expect_contact(
            upper_beam, base,
            elem_a="axle_sleeve", elem_b="high_axle",
            name="tilted upper beam sleeve stays on axle",
        )

    # --- backrest tilt pose check -------------------------------------------
    br00 = object_model.get_part("backrest_0_0")
    br00_tilt = object_model.get_articulation("backrest_0_0_tilt")
    rest_plate = ctx.part_element_world_aabb(br00, elem="plate")
    with ctx.pose({br00_tilt: BACKREST_UPPER}):
        tilted_plate = ctx.part_element_world_aabb(br00, elem="plate")
        if rest_plate is not None and tilted_plate is not None:
            rest_cx = (rest_plate[0][0] + rest_plate[1][0]) / 2.0
            tilt_cx = (tilted_plate[0][0] + tilted_plate[1][0]) / 2.0
            rest_cz = (rest_plate[0][2] + rest_plate[1][2]) / 2.0
            tilt_cz = (tilted_plate[0][2] + tilted_plate[1][2]) / 2.0
            ctx.check(
                "backrest_0_0 plate moves when tilted",
                abs(tilt_cx - rest_cx) > 0.003 or abs(tilt_cz - rest_cz) > 0.003,
                details=f"rest center=({rest_cx:.4f},{rest_cz:.4f}), tilted=({tilt_cx:.4f},{tilt_cz:.4f})",
            )

    return ctx.report()


object_model = build_object_model()
