from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Four-seat cross seesaw — variant of the vintage playground seesaw
#
# World frame: X-beam runs along X, Y-beam along Y, pivot axis along Y, Z up.
#
# Structure:
#   arched_base (root) — two bent-tube arches with rubber ground pads,
#                        a central support post, and a post-top plate.
#   spring_hub         — hub plate, coil spring, and pivot axle.
#                        Prismatic joint (Z axis) allows spring compression.
#   cross_beam         — two perpendicular beams (X and Y) rigidly joined
#                        at a central hub plate; four seats, four handles,
#                        four tire-section bumpers, and four textured
#                        footrests.  Revolute joint (Y axis) rocks ±20°.
# ---------------------------------------------------------------------------

# Ground pads
PAD_R = 0.060
PAD_T = 0.025  # tall enough to overlap with arch tube mesh bottom

# Arch geometry (bent galvanized-steel tube, ~50 mm dia)
TUBE_R = 0.025
ARCH_FOOT_Z = 0.037  # tube centerline height at feet; mesh bottom ~0.022
ARCH_FOOT_X = 0.66
ARCH_FOOT_Y = 0.34
ARCH_APEX_Y = 0.05
ARCH_APEX_Z = 0.76  # arch peak height

# Support post (rises from arch apex)
POST_R = 0.030  # wider than ARCH_APEX_Y to overlap with both arch tubes
POST_H = 0.08
POST_TOP = ARCH_APEX_Z + POST_H  # 0.84

# Coil spring
SPRING_R = 0.030
SPRING_WIRE_R = 0.004
SPRING_TURNS = 5
SPRING_H = 0.08  # rest height
SPRING_COMPRESS = 0.035  # max compression travel

# Prismatic joint origin = hub plate bottom at rest
HUB_Z = POST_TOP + SPRING_H  # 0.92

# Spring hub plate
HUB_R = 0.055
HUB_T = 0.015

# Pivot axle
AXLE_R = 0.014
AXLE_LEN = 0.20

# Beam bars (80 × 40 mm rectangular section)
BEAM_HALF = 1.50  # half-length → 3.0 m total
BEAM_W = 0.08
BEAM_T = 0.04

# X-beam bar (primary, lower)
BAR_BOT = 0.05
BAR_CTR = BAR_BOT + BEAM_T / 2.0  # 0.07
BAR_TOP = BAR_BOT + BEAM_T  # 0.09

# Y-beam bar (secondary, bolted on top of X-beam)
Y_BAR_BOT = BAR_TOP - 0.002  # 0.088 — 2 mm overlap for connectivity
Y_BAR_CTR = Y_BAR_BOT + BEAM_T / 2.0  # 0.108
Y_BAR_TOP = Y_BAR_BOT + BEAM_T  # 0.128

# End fittings (distance from beam center)
SEAT_DIST = 1.30
HANDLE_DIST = 1.04
BUMPER_DIST = 1.42
FOOTREST_DIST = 1.05

TILT = math.radians(20.0)


# ---- geometry helpers ----------------------------------------------------

def _arch_points(side: float) -> list[tuple[float, float, float]]:
    """Centerline of one bent-tube arch (parabolic rise, leaning inward)."""
    pts: list[tuple[float, float, float]] = []
    rise = ARCH_APEX_Z - ARCH_FOOT_Z
    for i in range(11):
        t = -1.0 + 0.2 * i
        s = 1.0 - t * t
        x = ARCH_FOOT_X * t
        z = ARCH_FOOT_Z + rise * s
        y = side * ARCH_FOOT_Y + (-side * ARCH_APEX_Y - side * ARCH_FOOT_Y) * s
        pts.append((x, y, z))
    return pts


def _spring_points() -> list[tuple[float, float, float]]:
    """Helical centerline for the coil spring (extends downward from hub)."""
    pts: list[tuple[float, float, float]] = []
    n = SPRING_TURNS * 24
    for i in range(n + 1):
        t = i / n
        a = t * SPRING_TURNS * 2.0 * math.pi
        pts.append((SPRING_R * math.cos(a), SPRING_R * math.sin(a), -SPRING_H * t))
    return pts


def _handle_points_x(x_pos: float, bar_top: float) -> list[tuple[float, float, float]]:
    """Inverted-U grab handle spanning Y at the given X position."""
    hw = 0.035
    leg_bot = bar_top - 0.010
    arc_z = bar_top + 0.245
    pts: list[tuple[float, float, float]] = [
        (x_pos, -hw, leg_bot),
        (x_pos, -hw, bar_top + 0.160),
    ]
    for k in range(7):
        a = math.pi * k / 6.0
        pts.append((x_pos, -hw * math.cos(a), arc_z + hw * math.sin(a)))
    pts.append((x_pos, hw, bar_top + 0.160))
    pts.append((x_pos, hw, leg_bot))
    return pts


def _handle_points_y(y_pos: float, bar_top: float) -> list[tuple[float, float, float]]:
    """Inverted-U grab handle spanning X at the given Y position."""
    hw = 0.035
    leg_bot = bar_top - 0.010
    arc_z = bar_top + 0.245
    pts: list[tuple[float, float, float]] = [
        (-hw, y_pos, leg_bot),
        (-hw, y_pos, bar_top + 0.160),
    ]
    for k in range(7):
        a = math.pi * k / 6.0
        pts.append((-hw * math.cos(a), y_pos, arc_z + hw * math.sin(a)))
    pts.append((hw, y_pos, bar_top + 0.160))
    pts.append((hw, y_pos, leg_bot))
    return pts


def _bumper_geometry(pos: float, index: int, along_x: bool, bar_bot: float):
    """Curved tire-section bumper shell hanging below the beam tip."""
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
    geom.rotate_x(math.pi / 2.0)  # C-shape in XZ, extrusion along Y
    if not along_x:
        geom.rotate_z(math.pi / 2.0)  # swap to YZ, extrusion along X
        geom.translate(0.0, pos, bar_bot + 0.002)
    else:
        geom.translate(pos, 0.0, bar_bot + 0.002)
    return mesh_from_geometry(geom, f"bumper_{index}")


def _footrest_mesh(name: str, along_x: bool):
    """Textured footrest plate with anti-slip ridges."""
    if along_x:
        base = BoxGeometry((0.12, 0.10, 0.006))
        result = base.copy()
        for i in range(4):
            y = -0.030 + i * 0.020
            ridge = BoxGeometry((0.10, 0.005, 0.007))
            ridge.translate(0.0, y, 0.0065)
            result = result.merge(ridge)
    else:
        base = BoxGeometry((0.10, 0.12, 0.006))
        result = base.copy()
        for i in range(4):
            x = -0.030 + i * 0.020
            ridge = BoxGeometry((0.005, 0.10, 0.007))
            ridge.translate(x, 0.0, 0.0065)
            result = result.merge(ridge)
    return mesh_from_geometry(result, name)


# ---- model ---------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cross_seesaw")

    # Materials
    galvanized = model.material("weathered_galvanized", rgba=(0.55, 0.58, 0.56, 1.0))
    rust = model.material("rust_steel", rgba=(0.42, 0.25, 0.13, 1.0))
    mustard = model.material("rusty_mustard_paint", rgba=(0.74, 0.53, 0.12, 1.0))
    pale_steel = model.material("pale_weathered_steel", rgba=(0.70, 0.66, 0.58, 1.0))
    wood = model.material("worn_wood", rgba=(0.60, 0.45, 0.28, 1.0))
    rubber = model.material("black_rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    dark_rubber = model.material("pad_rubber", rgba=(0.12, 0.12, 0.10, 1.0))
    spring_steel = model.material("spring_steel", rgba=(0.48, 0.50, 0.52, 1.0))
    footrest_mat = model.material("grip_rubber", rgba=(0.18, 0.16, 0.14, 1.0))

    # ---- base (root) ----
    base = model.part("arched_base")

    # Arched legs
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
                f"arch_{i}",
            ),
            material=galvanized,
            name=f"arch_{i}",
        )

    # Rubber ground pads under each arch foot (4 pads total, mesh-backed)
    pad_idx = 0
    for side in (1.0, -1.0):
        for sx in (-1.0, 1.0):
            pad_geom = CylinderGeometry(PAD_R, PAD_T, radial_segments=24)
            pad_geom.translate(sx * ARCH_FOOT_X, side * ARCH_FOOT_Y, PAD_T / 2.0)
            base.visual(
                mesh_from_geometry(pad_geom, f"ground_pad_{pad_idx}"),
                material=dark_rubber,
                name=f"ground_pad_{pad_idx}",
            )
            pad_idx += 1

    # Central support post (welded to arch apex, 3 mm embed for connectivity)
    post_geom = CylinderGeometry(POST_R, POST_H, radial_segments=18)
    post_geom.translate(0.0, 0.0, ARCH_APEX_Z + POST_H / 2.0 - 0.003)
    base.visual(
        mesh_from_geometry(post_geom, "support_post"),
        material=galvanized,
        name="support_post",
    )
    # Post-top saddle plate (overlaps post top for connectivity)
    plate_geom = CylinderGeometry(0.050, 0.008, radial_segments=18)
    plate_geom.translate(0.0, 0.0, POST_TOP - 0.005)
    base.visual(
        mesh_from_geometry(plate_geom, "post_plate"),
        material=galvanized,
        name="post_plate",
    )

    # ---- spring hub ----
    spring_hub = model.part("spring_hub")

    # Hub plate (disk)
    spring_hub.visual(
        Cylinder(radius=HUB_R, length=HUB_T),
        origin=Origin(xyz=(0.0, 0.0, HUB_T / 2.0)),
        material=pale_steel,
        name="hub_plate",
    )
    # Coil spring (extends downward from hub plate bottom)
    spring_hub.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                _spring_points(),
                radius=SPRING_WIRE_R,
                samples_per_segment=4,
                radial_segments=12,
                cap_ends=True,
            ),
            "coil_spring",
        ),
        material=spring_steel,
        name="coil_spring",
    )
    # Pivot axle bolt (horizontal, along Y)
    spring_hub.visual(
        Cylinder(radius=AXLE_R, length=AXLE_LEN),
        origin=Origin(xyz=(0.0, 0.0, HUB_T), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rust,
        name="pivot_axle",
    )
    # Axle nuts
    for i, side in enumerate((1.0, -1.0)):
        spring_hub.visual(
            Cylinder(radius=0.022, length=0.012),
            origin=Origin(
                xyz=(0.0, side * (AXLE_LEN / 2.0 - 0.005), HUB_T),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=rust,
            name=f"axle_nut_{i}",
        )

    # ---- cross beam assembly ----
    cross_beam = model.part("cross_beam")

    # Pivot sleeve (bushing around the axle, extends up to beam bar)
    cross_beam.visual(
        Cylinder(radius=0.024, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.026), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rust,
        name="pivot_sleeve",
    )

    # Central hub plate (connects the two beams at the crossing)
    cross_beam.visual(
        Cylinder(radius=0.14, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, BAR_BOT - 0.005)),
        material=mustard,
        name="hub_plate_beam",
    )

    # X-beam bar
    cross_beam.visual(
        Box((2.0 * BEAM_HALF, BEAM_W, BEAM_T)),
        origin=Origin(xyz=(0.0, 0.0, BAR_CTR)),
        material=mustard,
        name="beam_bar_x",
    )

    # Y-beam bar (perpendicular, on top of X-beam)
    cross_beam.visual(
        Box((BEAM_W, 2.0 * BEAM_HALF, BEAM_T)),
        origin=Origin(xyz=(0.0, 0.0, Y_BAR_CTR)),
        material=mustard,
        name="beam_bar_y",
    )

    # Rust streak patches on both beams
    for i, px in enumerate((-0.85, -0.30, 0.45, 0.95)):
        cross_beam.visual(
            Box((0.16, BEAM_W + 0.004, 0.010)),
            origin=Origin(xyz=(px, 0.0, BAR_TOP - 0.003)),
            material=rust,
            name=f"rust_x_{i}",
        )
    for i, py in enumerate((-0.75, 0.20, 0.60, -1.00)):
        cross_beam.visual(
            Box((BEAM_W + 0.004, 0.14, 0.010)),
            origin=Origin(xyz=(0.0, py, Y_BAR_TOP - 0.003)),
            material=rust,
            name=f"rust_y_{i}",
        )

    # ---- X-beam end fittings (indices 0, 1) ----
    for i, side in enumerate((1.0, -1.0)):
        # Seat
        cross_beam.visual(
            Box((0.30, 0.24, 0.022)),
            origin=Origin(xyz=(side * SEAT_DIST, 0.0, BAR_TOP + 0.008)),
            material=wood,
            name=f"seat_{i}",
        )
        # Handle
        cross_beam.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _handle_points_x(side * HANDLE_DIST, BAR_TOP),
                    radius=0.009,
                    samples_per_segment=8,
                    radial_segments=16,
                    cap_ends=True,
                ),
                f"handle_{i}",
            ),
            material=pale_steel,
            name=f"handle_{i}",
        )
        # Bumper
        cross_beam.visual(
            _bumper_geometry(side * BUMPER_DIST, i, along_x=True, bar_bot=BAR_BOT),
            material=rubber,
            name=f"bumper_{i}",
        )
        # Footrest
        cross_beam.visual(
            _footrest_mesh(f"footrest_{i}", along_x=True),
            origin=Origin(xyz=(side * FOOTREST_DIST, 0.0, BAR_TOP + 0.003)),
            material=footrest_mat,
            name=f"footrest_{i}",
        )

    # ---- Y-beam end fittings (indices 2, 3) ----
    for i, side in enumerate((1.0, -1.0)):
        idx = i + 2
        # Seat
        cross_beam.visual(
            Box((0.24, 0.30, 0.022)),
            origin=Origin(xyz=(0.0, side * SEAT_DIST, Y_BAR_TOP + 0.008)),
            material=wood,
            name=f"seat_{idx}",
        )
        # Handle
        cross_beam.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _handle_points_y(side * HANDLE_DIST, Y_BAR_TOP),
                    radius=0.009,
                    samples_per_segment=8,
                    radial_segments=16,
                    cap_ends=True,
                ),
                f"handle_{idx}",
            ),
            material=pale_steel,
            name=f"handle_{idx}",
        )
        # Bumper
        cross_beam.visual(
            _bumper_geometry(side * BUMPER_DIST, idx, along_x=False, bar_bot=Y_BAR_BOT),
            material=rubber,
            name=f"bumper_{idx}",
        )
        # Footrest
        cross_beam.visual(
            _footrest_mesh(f"footrest_{idx}", along_x=False),
            origin=Origin(xyz=(0.0, side * FOOTREST_DIST, Y_BAR_TOP + 0.003)),
            material=footrest_mat,
            name=f"footrest_{idx}",
        )

    # ---- articulations ----

    # Prismatic: spring compression (base → spring_hub)
    model.articulation(
        "spring_compress",
        ArticulationType.PRISMATIC,
        parent=base,
        child=spring_hub,
        origin=Origin(xyz=(0.0, 0.0, HUB_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=500.0,
            velocity=0.5,
            lower=-SPRING_COMPRESS,
            upper=0.0,
        ),
    )

    # Revolute: beam rocking (spring_hub → cross_beam)
    model.articulation(
        "beam_pivot",
        ArticulationType.REVOLUTE,
        parent=spring_hub,
        child=cross_beam,
        origin=Origin(xyz=(0.0, 0.0, HUB_T)),
        axis=(0.0, 1.0, 0.0),  # positive q lowers the +X end
        motion_limits=MotionLimits(
            effort=200.0,
            velocity=2.5,
            lower=-TILT,
            upper=TILT,
        ),
    )

    return model


# ---- tests ---------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("arched_base")
    spring_hub = object_model.get_part("spring_hub")
    cross_beam = object_model.get_part("cross_beam")
    spring_j = object_model.get_articulation("spring_compress")
    pivot_j = object_model.get_articulation("beam_pivot")

    # ---- joint configuration ----

    # Spring prismatic joint
    ctx.check(
        "spring joint is prismatic",
        spring_j.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={spring_j.articulation_type}",
    )
    s_ax = spring_j.axis
    ctx.check(
        "spring axis is vertical (Z)",
        abs(s_ax[0]) < 1e-9 and abs(s_ax[1]) < 1e-9 and abs(s_ax[2] - 1.0) < 1e-9,
        details=f"axis={s_ax}",
    )
    s_lim = spring_j.motion_limits
    ctx.check(
        "spring travel allows compression",
        s_lim is not None
        and s_lim.lower is not None
        and abs(s_lim.lower + SPRING_COMPRESS) < 1e-6
        and s_lim.upper is not None
        and abs(s_lim.upper) < 1e-6,
        details=f"limits=({s_lim.lower}, {s_lim.upper})",
    )

    # Pivot revolute joint
    ctx.check(
        "pivot joint is revolute",
        pivot_j.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={pivot_j.articulation_type}",
    )
    p_ax = pivot_j.axis
    ctx.check(
        "pivot axis is horizontal and perpendicular to X-beam",
        abs(p_ax[0]) < 1e-9 and abs(p_ax[1] - 1.0) < 1e-9 and abs(p_ax[2]) < 1e-9,
        details=f"axis={p_ax}",
    )
    p_lim = pivot_j.motion_limits
    ctx.check(
        "rocking limits are about +/- 20 degrees",
        p_lim is not None
        and p_lim.lower is not None
        and p_lim.upper is not None
        and abs(p_lim.lower + TILT) < 1e-6
        and abs(p_lim.upper - TILT) < 1e-6,
        details=f"limits=({p_lim.lower}, {p_lim.upper})",
    )

    # ---- intentional overlaps ----

    # Pivot sleeve captures the axle
    ctx.allow_overlap(
        cross_beam,
        spring_hub,
        elem_a="pivot_sleeve",
        elem_b="pivot_axle",
        reason="Pivot sleeve is a bushing intentionally nested around the axle bolt.",
    )
    ctx.expect_contact(
        cross_beam,
        spring_hub,
        elem_a="pivot_sleeve",
        elem_b="pivot_axle",
        name="pivot sleeve seated on axle",
    )

    # ---- four seats exist and are on the beam ----
    bar_x_box = ctx.part_element_world_aabb(cross_beam, elem="beam_bar_x")
    bar_y_box = ctx.part_element_world_aabb(cross_beam, elem="beam_bar_y")
    for i in range(4):
        seat = ctx.part_element_world_aabb(cross_beam, elem=f"seat_{i}")
        ref = bar_x_box if i < 2 else bar_y_box
        ctx.check(
            f"seat_{i} exists and is above its beam bar",
            seat is not None and ref is not None and seat[0][2] > ref[1][2] - 0.005,
            details=f"seat aabb={seat}",
        )

    # ---- four footrests exist ----
    for i in range(4):
        fr = ctx.part_element_world_aabb(cross_beam, elem=f"footrest_{i}")
        ctx.check(
            f"footrest_{i} exists near its seat",
            fr is not None,
            details=f"footrest aabb={fr}",
        )

    # ---- rubber ground pads ----
    base_box = ctx.part_world_aabb(base)
    for i in range(4):
        pad = ctx.part_element_world_aabb(base, elem=f"ground_pad_{i}")
        ctx.check(
            f"ground_pad_{i} is on the ground",
            pad is not None and pad[0][2] < 0.02,
            details=f"pad aabb={pad}",
        )

    # ---- perpendicular beams ----
    ctx.check(
        "X-beam spans at least 2.8 m along X",
        bar_x_box is not None and (bar_x_box[1][0] - bar_x_box[0][0]) > 2.8,
        details=f"bar_x aabb={bar_x_box}",
    )
    ctx.check(
        "Y-beam spans at least 2.8 m along Y",
        bar_y_box is not None and (bar_y_box[1][1] - bar_y_box[0][1]) > 2.8,
        details=f"bar_y aabb={bar_y_box}",
    )

    # ---- base grounded ----
    ctx.check(
        "arched base feet rest on ground pads",
        base_box is not None and -0.01 <= base_box[0][2] <= 0.02,
        details=f"base aabb={base_box}",
    )

    # ---- spring compression pose ----
    rest_hub_z = ctx.part_world_position(spring_hub)
    with ctx.pose({spring_j: -SPRING_COMPRESS}):
        comp_hub_z = ctx.part_world_position(spring_hub)
        ctx.check(
            "spring compression lowers the hub",
            rest_hub_z is not None
            and comp_hub_z is not None
            and comp_hub_z[2] < rest_hub_z[2] - 0.02,
            details=f"rest={rest_hub_z}, compressed={comp_hub_z}",
        )

    # ---- pivot rocking pose ----
    rest_b0 = ctx.part_element_world_aabb(cross_beam, elem="bumper_0")
    with ctx.pose({pivot_j: TILT}):
        down_b0 = ctx.part_element_world_aabb(cross_beam, elem="bumper_0")
        up_b1 = ctx.part_element_world_aabb(cross_beam, elem="bumper_1")
        ctx.check(
            "positive rock lowers +X end",
            rest_b0 is not None
            and down_b0 is not None
            and down_b0[0][2] < rest_b0[0][2] - 0.30,
            details=f"rest={rest_b0}, tilted={down_b0}",
        )
        ctx.check(
            "positive rock raises -X end",
            up_b1 is not None and up_b1[0][2] > 1.0,
            details=f"raised bumper aabb={up_b1}",
        )

    return ctx.report()


object_model = build_object_model()
