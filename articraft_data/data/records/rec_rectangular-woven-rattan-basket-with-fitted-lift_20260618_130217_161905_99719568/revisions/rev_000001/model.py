from __future__ import annotations

# Rectangular woven rattan basket with a fitted lift-off woven lid.
#
# Reference traits from picture/Container/Basket/004.png:
# - rounded rectangular storage basket, about 30.5 cm x 23 cm x 21 cm;
# - natural honey rattan with dense horizontal over-under side weaving;
# - thick braided top/body rim and thick lid edge;
# - shallow tented woven lid;
# - raised small rectangular woven handle centered on the lid;
# - lid lifts straight off as the primary articulation.

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

BODY_H = 0.170
Z_FOOT = 0.006
Z_FLOOR = 0.011

HX_BOTTOM = 0.142
HY_BOTTOM = 0.106
HX_TOP = 0.149
HY_TOP = 0.111
HX_BELLY = 0.153
HY_BELLY = 0.115

HX_LID = 0.160
HY_LID = 0.121
HX_LID_WEAVE = 0.158
HY_LID_WEAVE = 0.119
LID_DOME = 0.024
LID_SEAT_Z = BODY_H + 0.011
LID_LIFT = 0.140

HX_HANDLE = 0.044
HY_HANDLE = 0.027
HANDLE_BASE_Z = 0.026
HANDLE_TOP_Z = 0.047

SUPER_N = 5.2
PATH_SAMPLES = 240
BODY_ROWS = 24
STAKE_COUNT = 56

T_SIDE = 0.0036
T_STAKE = 0.0027
T_RIM = 0.0055
T_LID = 0.0030
T_HANDLE = 0.0037
WEAVE_RELIEF = 0.0022


def _sgn(value: float) -> float:
    return -1.0 if value < 0.0 else 1.0


def _lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def _super_point(hx: float, hy: float, s: float) -> tuple[float, float]:
    theta = 2.0 * math.pi * (s % 1.0)
    c = math.cos(theta)
    q = math.sin(theta)
    power = 2.0 / SUPER_N
    x = hx * _sgn(c) * (abs(c) ** power)
    y = hy * _sgn(q) * (abs(q) ** power)
    return x, y


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


def _rounded_rect_path(
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
        wave = math.sin(2.0 * math.pi * wave_count * s + phase) if wave_count else 0.0
        points.append((x + outward_amp * wave * nx, y + outward_amp * wave * ny, z + z_amp * wave))
    return points


def _perimeter_point(hx: float, hy: float, s: float) -> tuple[float, float]:
    return _super_point(hx, hy, s)


def _upright_path(s: float, index: int) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    samples = 28
    for i in range(samples):
        t = i / (samples - 1)
        z = Z_FOOT + (BODY_H - Z_FOOT) * t
        hx, hy = _body_half_extents(z)
        x, y = _perimeter_point(hx, hy, s)
        nx, ny = _outward_xy(x, y, hx, hy)
        flutter = 0.0008 * math.sin(2.0 * math.pi * BODY_ROWS * t + index * 0.37)
        points.append((x + (0.0012 + flutter) * nx, y + (0.0012 + flutter) * ny, z))
    return points


def _half_x_for_y(y: float, hx: float, hy: float) -> float:
    ratio = min(1.0, abs(y) / max(hy, 1e-6))
    return hx * max(0.0, 1.0 - ratio**SUPER_N) ** (1.0 / SUPER_N)


def _half_y_for_x(x: float, hx: float, hy: float) -> float:
    ratio = min(1.0, abs(x) / max(hx, 1e-6))
    return hy * max(0.0, 1.0 - ratio**SUPER_N) ** (1.0 / SUPER_N)


def _lid_z(x: float, y: float) -> float:
    edge = max(abs(x) / max(HX_LID_WEAVE, 1e-6), abs(y) / max(HY_LID_WEAVE, 1e-6))
    lift = max(0.0, 1.0 - edge)
    return 0.003 + LID_DOME * (lift**0.75)


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
            ripple = 0.0008 * math.sin(9.0 * math.pi * t + phase)
            points.append((x, y, _lid_z(x, y) + ripple))
    else:
        half = _half_y_for_x(offset, HX_LID_WEAVE, HY_LID_WEAVE)
        for i in range(samples):
            t = i / (samples - 1)
            x = offset
            y = _lerp(-half, half, t)
            ripple = 0.0008 * math.sin(9.0 * math.pi * t + phase)
            points.append((x, y, _lid_z(x, y) + ripple))
    return points


def _floor_chord(
    offset: float,
    *,
    orientation: str,
    samples: int = 10,
) -> list[tuple[float, float, float]]:
    hx, hy = _body_half_extents(Z_FOOT)
    points: list[tuple[float, float, float]] = []
    if orientation == "x":
        half = _half_x_for_y(offset, hx * 0.96, hy * 0.96)
        for i in range(samples):
            x = _lerp(-half, half, i / (samples - 1))
            points.append((x, offset, Z_FLOOR))
    else:
        half = _half_y_for_x(offset, hx * 0.96, hy * 0.96)
        for i in range(samples):
            y = _lerp(-half, half, i / (samples - 1))
            points.append((offset, y, Z_FLOOR + 0.0015))
    return points


def _handle_chord(
    offset: float,
    *,
    orientation: str,
    samples: int = 8,
) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    if orientation == "x":
        half = _half_x_for_y(offset, HX_HANDLE * 0.95, HY_HANDLE * 0.95)
        for i in range(samples):
            x = _lerp(-half, half, i / (samples - 1))
            points.append((x, offset, HANDLE_TOP_Z + 0.001))
    else:
        half = _half_y_for_x(offset, HX_HANDLE * 0.95, HY_HANDLE * 0.95)
        for i in range(samples):
            y = _lerp(-half, half, i / (samples - 1))
            points.append((offset, y, HANDLE_TOP_Z + 0.002))
    return points


def _path_segment(
    points: list[tuple[float, float, float]],
    start: int,
    count: int,
) -> list[tuple[float, float, float]]:
    n = len(points)
    return [points[(start + i) % n] for i in range(count)]


_TUBE_MESH_COUNTER = 0


def tube_from_spline_points(points, **kwargs):
    global _TUBE_MESH_COUNTER
    _TUBE_MESH_COUNTER += 1
    geom = _sdk_tube_from_spline_points(points, **kwargs)
    return mesh_from_geometry(geom, f"rect_woven_basket_tube_{_TUBE_MESH_COUNTER:03d}")


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
    model = ArticulatedObject(name="rectangular_woven_rattan_basket_with_lift_off_lid")

    model.material("rattan_light", rgba=(0.91, 0.70, 0.38, 1.0))
    model.material("rattan_mid", rgba=(0.76, 0.50, 0.21, 1.0))
    model.material("rattan_shadow", rgba=(0.49, 0.30, 0.11, 1.0))
    model.material("rattan_dark", rgba=(0.34, 0.20, 0.07, 1.0))

    body = model.part("basket_body")

    foot_hx, foot_hy = _body_half_extents(Z_FOOT)
    _add_closed_tube(
        body,
        _rounded_rect_path(foot_hx, foot_hy, Z_FOOT, outward_amp=0.0014, wave_count=34),
        radius=T_RIM,
        material="rattan_mid",
        name="rounded_rect_bottom_foot",
        radial_segments=9,
    )

    for i, off in enumerate(
        [-0.092, -0.079, -0.066, -0.053, -0.040, -0.027, -0.014, -0.001, 0.012, 0.025, 0.038, 0.051, 0.064, 0.077, 0.090]
    ):
        _add_open_tube(
            body,
            _floor_chord(off, orientation="x"),
            radius=0.0030,
            material="rattan_light" if i % 2 == 0 else "rattan_mid",
            name=f"floor_weave_x_{i:02d}",
            radial_segments=6,
        )
    for i, off in enumerate(
        [-0.126, -0.108, -0.090, -0.072, -0.054, -0.036, -0.018, 0.000, 0.018, 0.036, 0.054, 0.072, 0.090, 0.108, 0.126]
    ):
        _add_open_tube(
            body,
            _floor_chord(off, orientation="y"),
            radius=0.0030,
            material="rattan_mid" if i % 2 == 0 else "rattan_light",
            name=f"floor_weave_y_{i:02d}",
            radial_segments=6,
        )

    for j in range(STAKE_COUNT):
        _add_open_tube(
            body,
            _upright_path(j / STAKE_COUNT, j),
            radius=T_STAKE,
            material="rattan_shadow" if j % 6 == 0 else "rattan_mid",
            name=f"vertical_rect_weave_stake_{j:02d}",
            radial_segments=7,
        )

    z0 = 0.014
    z1 = BODY_H - 0.016
    for i in range(BODY_ROWS):
        t = i / (BODY_ROWS - 1)
        z = _lerp(z0, z1, t)
        hx, hy = _body_half_extents(z)
        _add_closed_tube(
            body,
            _rounded_rect_path(
                hx,
                hy,
                z,
                outward_amp=WEAVE_RELIEF,
                z_amp=0.0005,
                wave_count=STAKE_COUNT // 2,
                phase=(i % 2) * math.pi,
            ),
            radius=T_SIDE,
            material="rattan_light" if i % 2 == 0 else "rattan_mid",
            name=f"horizontal_rect_weave_band_{i:02d}",
            radial_segments=8,
        )

    top_hx, top_hy = _body_half_extents(BODY_H)
    for strand, phase in enumerate((0.0, math.pi)):
        _add_closed_tube(
            body,
            _rounded_rect_path(
                top_hx,
                top_hy,
                BODY_H,
                outward_amp=0.0026,
                z_amp=0.0014,
                wave_count=34,
                phase=phase,
            ),
            radius=T_RIM,
            material="rattan_light" if strand == 0 else "rattan_mid",
            name=f"body_braided_rect_mouth_rim_{strand}",
            radial_segments=10,
        )

    # Short darker stitches tucked under the lid line, visible in the reference
    # where the lid edge overlaps the basket wall.
    stitch_path = _rounded_rect_path(top_hx * 1.005, top_hy * 1.005, BODY_H - 0.013, samples=136)
    for d, start in enumerate(range(2, len(stitch_path), 11)):
        _add_open_tube(
            body,
            _path_segment(stitch_path, start, 4),
            radius=0.0032,
            material="rattan_dark",
            name=f"dark_lidline_stitch_{d:02d}",
            radial_segments=7,
        )

    lid = model.part("basket_lid")

    # Tented woven lid surface: crossed strips clip to a rounded rectangle and
    # rise toward the central handle.
    x_offsets = [-0.112, -0.098, -0.084, -0.070, -0.056, -0.042, -0.028, -0.014, 0.000, 0.014, 0.028, 0.042, 0.056, 0.070, 0.084, 0.098, 0.112]
    for i, off in enumerate(x_offsets):
        _add_open_tube(
            lid,
            _lid_chord(off, orientation="x", phase=i * 0.7),
            radius=T_LID,
            material="rattan_light" if i % 2 == 0 else "rattan_mid",
            name=f"lid_weave_x_strip_{i:02d}",
            radial_segments=7,
        )
    y_offsets = [-0.148, -0.130, -0.112, -0.094, -0.076, -0.058, -0.040, -0.022, -0.004, 0.014, 0.032, 0.050, 0.068, 0.086, 0.104, 0.122, 0.140]
    for i, off in enumerate(y_offsets):
        _add_open_tube(
            lid,
            _lid_chord(off, orientation="y", phase=i * 0.7 + math.pi),
            radius=T_LID,
            material="rattan_mid" if i % 2 == 0 else "rattan_light",
            name=f"lid_weave_y_strip_{i:02d}",
            radial_segments=7,
        )

    for strand, phase in enumerate((0.0, math.pi)):
        _add_closed_tube(
            lid,
            _rounded_rect_path(
                HX_LID,
                HY_LID,
                0.001,
                outward_amp=0.0030,
                z_amp=0.0015,
                wave_count=36,
                phase=phase,
            ),
            radius=0.0063,
            material="rattan_light" if strand == 0 else "rattan_mid",
            name=f"braided_rect_lid_outer_rim_{strand}",
            radial_segments=10,
        )
    for k, z in enumerate((-0.001, 0.005)):
        _add_closed_tube(
            lid,
            _rounded_rect_path(HX_LID * 0.982, HY_LID * 0.982, z, outward_amp=0.0012, wave_count=30, phase=k),
            radius=0.0042,
            material="rattan_mid",
            name=f"lid_thick_edge_weave_row_{k}",
            radial_segments=8,
        )

    for i in range(24):
        s = i / 24.0
        outer_x, outer_y = _super_point(HX_LID * 0.985, HY_LID * 0.985, s)
        mid_x, mid_y = _super_point((HX_LID + HX_LID_WEAVE) * 0.5, (HY_LID + HY_LID_WEAVE) * 0.5, s)
        inner_x, inner_y = _super_point(HX_LID_WEAVE * 0.985, HY_LID_WEAVE * 0.985, s)
        _add_open_tube(
            lid,
            [
                (outer_x, outer_y, 0.000),
                (mid_x, mid_y, _lid_z(mid_x, mid_y) * 0.55),
                (inner_x, inner_y, _lid_z(inner_x, inner_y)),
            ],
            radius=0.0033,
            material="rattan_mid" if i % 2 else "rattan_light",
            name=f"lid_rim_to_face_connector_{i:02d}",
            radial_segments=7,
        )

    # Four subtle slope ribs make the lid read like the shallow raised cover in
    # the reference image.
    for i, (x, y) in enumerate(((0.0, HY_LID_WEAVE), (HX_LID_WEAVE, 0.0), (0.0, -HY_LID_WEAVE), (-HX_LID_WEAVE, 0.0))):
        _add_open_tube(
            lid,
            [
                (x, y, _lid_z(x, y)),
                (x * 0.48, y * 0.48, _lid_z(x * 0.48, y * 0.48) + 0.001),
                (0.0, 0.0, HANDLE_BASE_Z),
            ],
            radius=0.0034,
            material="rattan_mid",
            name=f"lid_tented_slope_rib_{i}",
            radial_segments=7,
        )

    # Raised small rectangular woven handle, built as connected rattan rows and
    # vertical supports so it is visibly mounted on the lid.
    for k, z in enumerate((HANDLE_BASE_Z, HANDLE_TOP_Z)):
        _add_closed_tube(
            lid,
            _rounded_rect_path(HX_HANDLE, HY_HANDLE, z, samples=100, outward_amp=0.0008, wave_count=14, phase=k),
            radius=T_HANDLE,
            material="rattan_mid" if k == 0 else "rattan_light",
            name=f"raised_rect_handle_rim_{k}",
            radial_segments=8,
        )
    for j in range(16):
        x, y = _super_point(HX_HANDLE, HY_HANDLE, j / 16.0)
        _add_open_tube(
            lid,
            [(x, y, _lid_z(x, y) - 0.001), (x, y, HANDLE_BASE_Z), (x, y, HANDLE_TOP_Z)],
            radius=0.0028,
            material="rattan_shadow" if j % 4 == 0 else "rattan_mid",
            name=f"handle_vertical_stake_{j:02d}",
            radial_segments=7,
        )
    for i, off in enumerate((-0.017, -0.008, 0.001, 0.010, 0.019)):
        _add_open_tube(
            lid,
            _handle_chord(off, orientation="x"),
            radius=0.0027,
            material="rattan_light" if i % 2 == 0 else "rattan_mid",
            name=f"handle_top_weave_x_{i:02d}",
            radial_segments=6,
        )
    for i, off in enumerate((-0.030, -0.018, -0.006, 0.006, 0.018, 0.030)):
        _add_open_tube(
            lid,
            _handle_chord(off, orientation="y"),
            radius=0.0027,
            material="rattan_mid" if i % 2 == 0 else "rattan_light",
            name=f"handle_top_weave_y_{i:02d}",
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

    body_rows = sum(1 for v in body.visuals if (v.name or "").startswith("horizontal_rect_weave"))
    stakes = sum(1 for v in body.visuals if (v.name or "").startswith("vertical_rect_weave"))
    floor = sum(1 for v in body.visuals if (v.name or "").startswith("floor_weave"))
    ctx.check(
        "rounded_rect_dense_woven_body",
        body_rows >= BODY_ROWS and stakes >= STAKE_COUNT and floor >= 30,
        details=f"rows={body_rows}, stakes={stakes}, floor={floor}",
    )

    lid_x = sum(1 for v in lid.visuals if (v.name or "").startswith("lid_weave_x"))
    lid_y = sum(1 for v in lid.visuals if (v.name or "").startswith("lid_weave_y"))
    handle_rim = sum(1 for v in lid.visuals if (v.name or "").startswith("raised_rect_handle_rim"))
    handle_stakes = sum(1 for v in lid.visuals if (v.name or "").startswith("handle_vertical_stake"))
    handle_top = sum(1 for v in lid.visuals if (v.name or "").startswith("handle_top_weave"))
    ctx.check(
        "woven_lid_with_raised_rect_handle",
        lid_x >= 17 and lid_y >= 17 and handle_rim == 2 and handle_stakes >= 16 and handle_top >= 10,
        details=(
            f"lid_x={lid_x}, lid_y={lid_y}, handle_rim={handle_rim}, "
            f"handle_stakes={handle_stakes}, handle_top={handle_top}"
        ),
    )

    ctx.check(
        "lid_weave_strips_reach_edge",
        HX_LID_WEAVE >= HX_LID * 0.985 and HY_LID_WEAVE >= HY_LID * 0.980,
        details=(
            f"lid_weave=({HX_LID_WEAVE:.3f}, {HY_LID_WEAVE:.3f}), "
            f"lid_outer=({HX_LID:.3f}, {HY_LID:.3f})"
        ),
    )

    full_body = ctx.part_world_aabb(body)
    full_lid = ctx.part_world_aabb(lid)
    width = max(_span(full_body, 0), _span(full_lid, 0))
    depth = max(_span(full_body, 1), _span(full_lid, 1))
    height = max(full_body[1][2], full_lid[1][2]) - min(full_body[0][2], full_lid[0][2])
    ctx.check(
        "reference_like_rectangular_proportions",
        0.29 <= width <= 0.34 and 0.22 <= depth <= 0.27 and 0.19 <= height <= 0.24,
        details=f"width={width:.3f}, depth={depth:.3f}, height={height:.3f}",
    )

    ctx.check(
        "lift_off_lid_prismatic_axis",
        joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={joint.articulation_type}, axis={joint.axis}",
    )

    ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.22, name="lid_covers_rect_mouth")

    with ctx.pose({joint: LID_LIFT}):
        ctx.expect_gap(lid, body, axis="z", min_gap=0.006, name="lid_lifts_clear")

    return ctx.report()


object_model = build_object_model()
