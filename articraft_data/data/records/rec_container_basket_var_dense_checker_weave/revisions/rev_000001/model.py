from __future__ import annotations

# Squat oval rattan basket with a dense checkerboard over-under weave and a
# fitted lift-off lid.
#
# Variant of the Container/Basket family:
# - squat rounded-rectangle/oval footprint, lower and wider than the seed;
# - dense side wall made from close vertical stakes and close horizontal bands;
# - row-to-row radial phase alternation gives an over-under checker weave;
# - fitted woven lid lifts upward on a prismatic Z joint.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points as _sdk_tube_from_spline_points,
)

BODY_H = 0.128
Z_FOOT = 0.006
Z_FLOOR = 0.008

HX_BOTTOM = 0.132
HY_BOTTOM = 0.086
HX_TOP = 0.146
HY_TOP = 0.096
HX_BELLY = 0.156
HY_BELLY = 0.103

HX_LID = 0.158
HY_LID = 0.106
HX_LID_WEAVE = 0.153
HY_LID_WEAVE = 0.101
LID_DOME = 0.017
LID_SEAT_Z = BODY_H + 0.008
LID_LIFT = 0.125

HX_PULL = 0.045
HY_PULL = 0.029
PULL_BASE_Z = 0.021
PULL_TOP_Z = 0.036

SUPER_N = 3.6
PATH_SAMPLES = 240

VERTICAL_STAKES = 44
CHECKER_ROWS = 30
FLOOR_STRIPS_X = 13
FLOOR_STRIPS_Y = 17
CHECKER_AMP = 0.0021

T_STAKE = 0.0022
T_ROW = 0.0026
T_RIM = 0.0055
T_LID = 0.0026
T_PULL = 0.0033


def _sgn(value: float) -> float:
    return -1.0 if value < 0.0 else 1.0


def _lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def _super_point(hx: float, hy: float, s: float) -> tuple[float, float]:
    theta = 2.0 * math.pi * (s % 1.0)
    c = math.cos(theta)
    q = math.sin(theta)
    power = 2.0 / SUPER_N
    return hx * _sgn(c) * (abs(c) ** power), hy * _sgn(q) * (abs(q) ** power)


def _outward_xy(x: float, y: float, hx: float, hy: float) -> tuple[float, float]:
    nx = x / max(hx, 1e-6)
    ny = y / max(hy, 1e-6)
    length = math.hypot(nx, ny)
    if length <= 1e-9:
        return 1.0, 0.0
    return nx / length, ny / length


def _body_half_extents(z: float) -> tuple[float, float]:
    t = max(0.0, min(1.0, (z - Z_FOOT) / (BODY_H - Z_FOOT)))
    belly = math.sin(math.pi * t)
    hx = _lerp(HX_BOTTOM, HX_TOP, t) + (HX_BELLY - 0.5 * (HX_BOTTOM + HX_TOP)) * belly
    hy = _lerp(HY_BOTTOM, HY_TOP, t) + (HY_BELLY - 0.5 * (HY_BOTTOM + HY_TOP)) * belly
    return hx, hy


def _rounded_oval_path(
    hx: float,
    hy: float,
    z: float,
    *,
    samples: int = PATH_SAMPLES,
    outward_amp: float = 0.0,
    z_amp: float = 0.0,
    wave_count: int = 0,
    phase: float = 0.0,
) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for i in range(samples):
        s = i / samples
        x, y = _super_point(hx, hy, s)
        nx, ny = _outward_xy(x, y, hx, hy)
        wave = math.cos(2.0 * math.pi * wave_count * s + phase) if wave_count else 0.0
        points.append((x + outward_amp * wave * nx, y + outward_amp * wave * ny, z + z_amp * wave))
    return points


def _perimeter_point(hx: float, hy: float, s: float) -> tuple[float, float]:
    return _super_point(hx, hy, s)


def _checker_vertical_path(s: float, index: int) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    samples = 46
    for i in range(samples):
        t = i / (samples - 1)
        z = Z_FOOT + (BODY_H - Z_FOOT) * t
        hx, hy = _body_half_extents(z)
        x, y = _perimeter_point(hx, hy, s)
        nx, ny = _outward_xy(x, y, hx, hy)
        checker = math.cos(2.0 * math.pi * CHECKER_ROWS * t + index * math.pi)
        flutter = 0.0005 * math.sin(2.0 * math.pi * 7.0 * t + index * 0.31)
        points.append((x + (0.0010 - 0.55 * CHECKER_AMP * checker + flutter) * nx, y + (0.0010 - 0.55 * CHECKER_AMP * checker + flutter) * ny, z))
    return points


def _checker_row_path(row_index: int) -> list[tuple[float, float, float]]:
    z0 = Z_FOOT + 0.011
    z1 = BODY_H - 0.012
    t = row_index / (CHECKER_ROWS - 1)
    z = _lerp(z0, z1, t)
    hx, hy = _body_half_extents(z)
    phase = row_index * math.pi
    return _rounded_oval_path(
        hx,
        hy,
        z,
        outward_amp=CHECKER_AMP,
        z_amp=0.0007,
        wave_count=VERTICAL_STAKES,
        phase=phase,
    )


def _half_x_for_y(y: float, hx: float, hy: float) -> float:
    ratio = min(1.0, abs(y) / max(hy, 1e-6))
    return hx * max(0.0, 1.0 - ratio**SUPER_N) ** (1.0 / SUPER_N)


def _half_y_for_x(x: float, hx: float, hy: float) -> float:
    ratio = min(1.0, abs(x) / max(hx, 1e-6))
    return hy * max(0.0, 1.0 - ratio**SUPER_N) ** (1.0 / SUPER_N)


def _floor_chord(
    offset: float,
    *,
    orientation: str,
    samples: int = 10,
) -> list[tuple[float, float, float]]:
    hx, hy = _body_half_extents(Z_FOOT)
    points: list[tuple[float, float, float]] = []
    if orientation == "x":
        half = _half_x_for_y(offset, hx * 1.01, hy * 1.01)
        for i in range(samples):
            x = _lerp(-half, half, i / (samples - 1))
            points.append((x, offset, Z_FLOOR))
    else:
        half = _half_y_for_x(offset, hx * 1.01, hy * 1.01)
        for i in range(samples):
            y = _lerp(-half, half, i / (samples - 1))
            points.append((offset, y, Z_FLOOR + 0.0012))
    return points


def _lid_z(x: float, y: float) -> float:
    edge = max(abs(x) / max(HX_LID_WEAVE, 1e-6), abs(y) / max(HY_LID_WEAVE, 1e-6))
    lift = max(0.0, 1.0 - edge)
    return 0.004 + LID_DOME * (lift**0.78)


def _lid_chord(
    offset: float,
    *,
    orientation: str,
    samples: int = 22,
    phase: float = 0.0,
) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    if orientation == "x":
        half = _half_x_for_y(offset, HX_LID_WEAVE, HY_LID_WEAVE)
        for i in range(samples):
            t = i / (samples - 1)
            x = _lerp(-half, half, t)
            y = offset
            ripple = 0.00075 * math.cos(10.0 * math.pi * t + phase)
            points.append((x, y, _lid_z(x, y) + ripple))
    else:
        half = _half_y_for_x(offset, HX_LID_WEAVE, HY_LID_WEAVE)
        for i in range(samples):
            t = i / (samples - 1)
            x = offset
            y = _lerp(-half, half, t)
            ripple = 0.00075 * math.cos(10.0 * math.pi * t + phase)
            points.append((x, y, _lid_z(x, y) + ripple))
    return points


def _pull_chord(
    offset: float,
    *,
    orientation: str,
    samples: int = 8,
) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    if orientation == "x":
        half = _half_x_for_y(offset, HX_PULL * 0.95, HY_PULL * 0.95)
        for i in range(samples):
            points.append((_lerp(-half, half, i / (samples - 1)), offset, PULL_TOP_Z + 0.001))
    else:
        half = _half_y_for_x(offset, HX_PULL * 0.95, HY_PULL * 0.95)
        for i in range(samples):
            points.append((offset, _lerp(-half, half, i / (samples - 1)), PULL_TOP_Z + 0.002))
    return points


_TUBE_MESH_COUNTER = 0


def tube_from_spline_points(points, **kwargs):
    global _TUBE_MESH_COUNTER
    _TUBE_MESH_COUNTER += 1
    geom = _sdk_tube_from_spline_points(points, **kwargs)
    return mesh_from_geometry(geom, f"dense_checker_basket_tube_{_TUBE_MESH_COUNTER:03d}")


def _add_closed_tube(part, points, *, radius: float, material: str, name: str, radial_segments: int = 8):
    part.visual(
        tube_from_spline_points(
            points,
            radius=radius,
            closed_spline=True,
            samples_per_segment=1,
            radial_segments=radial_segments,
            cap_ends=False,
        ),
        material=material,
        name=name,
    )


def _add_open_tube(part, points, *, radius: float, material: str, name: str, radial_segments: int = 7):
    part.visual(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=2,
            radial_segments=radial_segments,
            cap_ends=True,
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="squat_dense_checker_woven_basket_with_lift_off_lid")

    model.material("rattan_light", rgba=(0.88, 0.66, 0.34, 1.0))
    model.material("rattan_mid", rgba=(0.70, 0.46, 0.20, 1.0))
    model.material("rattan_shadow", rgba=(0.42, 0.25, 0.09, 1.0))
    model.material("rattan_dark", rgba=(0.28, 0.16, 0.05, 1.0))

    body = model.part("basket_body")

    foot_hx, foot_hy = _body_half_extents(Z_FOOT)
    _add_closed_tube(
        body,
        _rounded_oval_path(foot_hx, foot_hy, Z_FOOT, outward_amp=0.0015, wave_count=36),
        radius=T_RIM,
        material="rattan_mid",
        name="squat_oval_bottom_foot_ring",
        radial_segments=9,
    )

    for i in range(FLOOR_STRIPS_X):
        off = _lerp(-HY_BOTTOM * 0.78, HY_BOTTOM * 0.78, i / (FLOOR_STRIPS_X - 1))
        _add_open_tube(
            body,
            _floor_chord(off, orientation="x"),
            radius=0.0027,
            material="rattan_light" if i % 2 == 0 else "rattan_mid",
            name=f"floor_checker_x_{i:02d}",
            radial_segments=6,
        )
    for i in range(FLOOR_STRIPS_Y):
        off = _lerp(-HX_BOTTOM * 0.80, HX_BOTTOM * 0.80, i / (FLOOR_STRIPS_Y - 1))
        _add_open_tube(
            body,
            _floor_chord(off, orientation="y"),
            radius=0.0027,
            material="rattan_mid" if i % 2 == 0 else "rattan_light",
            name=f"floor_checker_y_{i:02d}",
            radial_segments=6,
        )

    for j in range(VERTICAL_STAKES):
        _add_open_tube(
            body,
            _checker_vertical_path(j / VERTICAL_STAKES, j),
            radius=T_STAKE,
            material="rattan_shadow" if j % 4 == 0 else "rattan_mid",
            name=f"checker_vertical_stake_{j:02d}",
            radial_segments=7,
        )

    for i in range(CHECKER_ROWS):
        _add_closed_tube(
            body,
            _checker_row_path(i),
            radius=T_ROW,
            material="rattan_light" if i % 2 == 0 else "rattan_mid",
            name=f"checker_horizontal_band_{i:02d}",
            radial_segments=7,
        )

    top_hx, top_hy = _body_half_extents(BODY_H)
    for strand, phase in enumerate((0.0, math.pi)):
        _add_closed_tube(
            body,
            _rounded_oval_path(
                top_hx,
                top_hy,
                BODY_H,
                outward_amp=0.0025,
                z_amp=0.0011,
                wave_count=38,
                phase=phase,
            ),
            radius=T_RIM,
            material="rattan_light" if strand == 0 else "rattan_mid",
            name=f"body_braided_oval_mouth_rim_{strand}",
            radial_segments=10,
        )

    lid = model.part("basket_lid")

    x_offsets = [_lerp(-HY_LID_WEAVE * 0.88, HY_LID_WEAVE * 0.88, i / 18) for i in range(19)]
    for i, off in enumerate(x_offsets):
        _add_open_tube(
            lid,
            _lid_chord(off, orientation="x", phase=i * math.pi),
            radius=T_LID,
            material="rattan_light" if i % 2 == 0 else "rattan_mid",
            name=f"lid_checker_x_strip_{i:02d}",
            radial_segments=7,
        )
    y_offsets = [_lerp(-HX_LID_WEAVE * 0.88, HX_LID_WEAVE * 0.88, i / 24) for i in range(25)]
    for i, off in enumerate(y_offsets):
        _add_open_tube(
            lid,
            _lid_chord(off, orientation="y", phase=i * math.pi + math.pi),
            radius=T_LID,
            material="rattan_mid" if i % 2 == 0 else "rattan_light",
            name=f"lid_checker_y_strip_{i:02d}",
            radial_segments=7,
        )

    for strand, phase in enumerate((0.0, math.pi)):
        _add_closed_tube(
            lid,
            _rounded_oval_path(
                HX_LID,
                HY_LID,
                0.002,
                outward_amp=0.0028,
                z_amp=0.0014,
                wave_count=40,
                phase=phase,
            ),
            radius=0.0060,
            material="rattan_light" if strand == 0 else "rattan_mid",
            name=f"braided_oval_lid_outer_rim_{strand}",
            radial_segments=10,
        )
    for k, z in enumerate((-0.001, 0.005)):
        _add_closed_tube(
            lid,
            _rounded_oval_path(HX_LID * 0.976, HY_LID * 0.976, z, outward_amp=0.0010, wave_count=32, phase=k),
            radius=0.0038,
            material="rattan_mid",
            name=f"lid_fitted_lower_edge_row_{k}",
            radial_segments=8,
        )

    for i in range(20):
        s = i / 20.0
        outer_x, outer_y = _super_point(HX_LID * 0.985, HY_LID * 0.985, s)
        inner_x, inner_y = _super_point(HX_LID_WEAVE * 0.96, HY_LID_WEAVE * 0.96, s)
        _add_open_tube(
            lid,
            [
                (outer_x, outer_y, 0.001),
                (_lerp(outer_x, inner_x, 0.45), _lerp(outer_y, inner_y, 0.45), _lid_z(inner_x, inner_y) * 0.45),
                (inner_x, inner_y, _lid_z(inner_x, inner_y)),
            ],
            radius=0.0028,
            material="rattan_shadow" if i % 4 == 0 else "rattan_mid",
            name=f"lid_edge_to_checker_connector_{i:02d}",
            radial_segments=7,
        )

    for k, z in enumerate((PULL_BASE_Z, PULL_TOP_Z)):
        _add_closed_tube(
            lid,
            _rounded_oval_path(HX_PULL, HY_PULL, z, samples=100, outward_amp=0.0007, wave_count=14, phase=k),
            radius=T_PULL,
            material="rattan_mid" if k == 0 else "rattan_light",
            name=f"low_oval_lid_pull_rim_{k}",
            radial_segments=8,
        )
    for j in range(14):
        x, y = _super_point(HX_PULL, HY_PULL, j / 14.0)
        _add_open_tube(
            lid,
            [(x, y, _lid_z(x, y) - 0.001), (x, y, PULL_BASE_Z), (x, y, PULL_TOP_Z)],
            radius=0.0025,
            material="rattan_shadow" if j % 4 == 0 else "rattan_mid",
            name=f"lid_pull_vertical_stake_{j:02d}",
            radial_segments=7,
        )
    for i, off in enumerate((-0.018, -0.009, 0.000, 0.009, 0.018)):
        _add_open_tube(
            lid,
            _pull_chord(off, orientation="x"),
            radius=0.0024,
            material="rattan_light" if i % 2 == 0 else "rattan_mid",
            name=f"lid_pull_top_weave_x_{i:02d}",
            radial_segments=6,
        )
    for i, off in enumerate((-0.030, -0.018, -0.006, 0.006, 0.018, 0.030)):
        _add_open_tube(
            lid,
            _pull_chord(off, orientation="y"),
            radius=0.0024,
            material="rattan_mid" if i % 2 == 0 else "rattan_light",
            name=f"lid_pull_top_weave_y_{i:02d}",
            radial_segments=6,
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, LID_SEAT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.35, lower=0.0, upper=LID_LIFT),
    )

    return model


def _span(aabb, axis: int) -> float:
    return aabb[1][axis] - aabb[0][axis]


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("basket_body")
    lid = object_model.get_part("basket_lid")
    joint = object_model.get_articulation("body_to_lid")

    verticals = sum(1 for v in body.visuals if (v.name or "").startswith("checker_vertical_stake"))
    rows = sum(1 for v in body.visuals if (v.name or "").startswith("checker_horizontal_band"))
    diagonals = sum(1 for v in body.visuals if "diagonal" in (v.name or ""))
    lattice = sum(1 for v in body.visuals if "lattice" in (v.name or ""))
    ctx.check(
        "dense_checker_wall_weave",
        verticals >= VERTICAL_STAKES and rows >= CHECKER_ROWS and diagonals == 0 and lattice == 0,
        details=f"verticals={verticals}, rows={rows}, diagonals={diagonals}, lattice={lattice}",
    )
    ctx.check(
        "checker_weave_denser_than_open_lattice_seed",
        verticals >= 40 and rows >= 28,
        details=f"verticals={verticals}, horizontal_rows={rows}",
    )

    lid_x = sum(1 for v in lid.visuals if (v.name or "").startswith("lid_checker_x_strip"))
    lid_y = sum(1 for v in lid.visuals if (v.name or "").startswith("lid_checker_y_strip"))
    lid_rim = sum(1 for v in lid.visuals if (v.name or "").startswith("braided_oval_lid_outer_rim"))
    pull = sum(1 for v in lid.visuals if (v.name or "").startswith("low_oval_lid_pull_rim"))
    ctx.check(
        "fitted_checker_lid_present",
        lid_x >= 18 and lid_y >= 24 and lid_rim == 2 and pull == 2,
        details=f"lid_x={lid_x}, lid_y={lid_y}, lid_rim={lid_rim}, pull={pull}",
    )

    full_body = ctx.part_world_aabb(body)
    full_lid = ctx.part_world_aabb(lid)
    width = max(_span(full_body, 0), _span(full_lid, 0))
    depth = max(_span(full_body, 1), _span(full_lid, 1))
    height = max(full_body[1][2], full_lid[1][2]) - min(full_body[0][2], full_lid[0][2])
    ctx.check(
        "squat_rounded_oval_proportions",
        0.29 <= width <= 0.34 and 0.19 <= depth <= 0.24 and 0.14 <= height <= 0.18 and width > height * 1.7,
        details=f"width={width:.3f}, depth={depth:.3f}, height={height:.3f}",
    )

    limits = joint.motion_limits
    ctx.check(
        "body_to_lid_upward_prismatic_joint",
        joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(joint.axis) == (0.0, 0.0, 1.0)
        and limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and limits.upper > 0.05,
        details=f"type={joint.articulation_type}, axis={joint.axis}, limits={limits}",
    )

    ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.19, name="lid_covers_oval_mouth")

    with ctx.pose({joint: LID_LIFT}):
        ctx.expect_gap(lid, body, axis="z", min_gap=0.004, name="lid_lifts_clear_in_raised_pose")

    return ctx.report()


object_model = build_object_model()
