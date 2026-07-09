from __future__ import annotations

# Rectangular woven rattan basket with patterned lattice walls and fitted lift-off lid.
#
# Variant of the parent dense-weave basket (rec_rectangular-woven-rattan-basket-with-fitted-lift):
# - same rounded rectangular footprint (~30.5 cm x 23 cm x 21 cm);
# - same woven lid with raised rectangular handle and lift-off prismatic joint;
# - CHANGED: walls use a dense patterned weave: stacked wavy horizontal bands,
#   short alternating vertical stitches, and compact herringbone inserts around
#   all four sides instead of sparse long diagonal canes.

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

# Dense patterned wall parameters. The weave reads as an all-around basket
# surface: horizontal bands provide body, short uprights lock the bands, and
# small herringbone inserts give the sides a distinct decorative pattern.
WALL_BANDS = 18
WALL_STITCH_COLUMNS = 48
HERRINGBONE_COLUMNS = 44
DIAGONAL_SHIFT = 0.018

T_STAKE = 0.0024
T_DIAGONAL = 0.0025
T_WALL_BAND = 0.0032
T_WALL_STITCH = 0.0025
T_RIM = 0.0055
T_LID = 0.0030
T_HANDLE = 0.0037
LID_SKIRT_DROP = 0.012


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
        flutter = 0.0005 * math.sin(2.0 * math.pi * 10 * t + index * 0.37)
        points.append((x + (0.0012 + flutter) * nx, y + (0.0012 + flutter) * ny, z))
    return points


def _diagonal_cane_path(
    s_start: float,
    direction: int,
    *,
    samples: int = 36,
    z_margin: float = 0.012,
) -> list[tuple[float, float, float]]:
    """Generate a diagonal cane path spiraling around the body wall.

    The cane runs from near the bottom to near the top while shifting along
    the perimeter by ``DIAGONAL_SHIFT * direction``, producing a visible
    diagonal lean on the wall surface. Two sets of these (direction +1 and -1)
    cross each other to form tight diamond pinholes.
    """
    points: list[tuple[float, float, float]] = []
    z0 = Z_FOOT + z_margin
    z1 = BODY_H - z_margin
    ds = DIAGONAL_SHIFT * direction
    for i in range(samples):
        t = i / (samples - 1)
        z = z0 + (z1 - z0) * t
        s = (s_start + ds * t) % 1.0
        hx, hy = _body_half_extents(z)
        x, y = _perimeter_point(hx, hy, s)
        nx, ny = _outward_xy(x, y, hx, hy)
        points.append((x + 0.0014 * nx, y + 0.0014 * ny, z))
    return points


def _wall_band_path(z: float, index: int) -> list[tuple[float, float, float]]:
    hx, hy = _body_half_extents(z)
    return _rounded_rect_path(
        hx,
        hy,
        z,
        outward_amp=0.0020,
        z_amp=0.0008,
        wave_count=42,
        phase=index * 0.65,
    )


def _short_upright_stitch_path(
    s: float,
    z0: float,
    z1: float,
    index: int,
    *,
    samples: int = 7,
) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for i in range(samples):
        t = i / (samples - 1)
        z = _lerp(z0, z1, t)
        hx, hy = _body_half_extents(z)
        x, y = _perimeter_point(hx, hy, s)
        nx, ny = _outward_xy(x, y, hx, hy)
        tuck = 0.0014 + 0.0006 * math.sin(math.pi * t + index * 0.41)
        points.append((x + tuck * nx, y + tuck * ny, z))
    return points


def _herringbone_stitch_path(
    s_mid: float,
    z0: float,
    direction: int,
    index: int,
    *,
    samples: int = 9,
) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    z1 = z0 + 0.022
    for i in range(samples):
        t = i / (samples - 1)
        z = _lerp(z0, z1, t)
        s = (s_mid + direction * DIAGONAL_SHIFT * (t - 0.5)) % 1.0
        hx, hy = _body_half_extents(z)
        x, y = _perimeter_point(hx, hy, s)
        nx, ny = _outward_xy(x, y, hx, hy)
        over_under = 0.0016 + 0.0007 * math.sin(2.0 * math.pi * t + index * 0.31)
        points.append((x + over_under * nx, y + over_under * ny, z))
    return points


def _upper_lidline_lock_path(s: float, index: int) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for i in range(8):
        t = i / 7.0
        z = _lerp(BODY_H - 0.018, BODY_H - 0.004, t)
        hx, hy = _body_half_extents(z)
        x, y = _perimeter_point(hx, hy, s)
        nx, ny = _outward_xy(x, y, hx, hy)
        wiggle = 0.0008 * math.sin(math.pi * t + index * 0.43)
        points.append((x + (0.0020 + wiggle) * nx, y + (0.0020 + wiggle) * ny, z))
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


def _lid_skirt_stake_path(s: float, index: int) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for i in range(5):
        t = i / 4.0
        z = _lerp(0.004, -LID_SKIRT_DROP, t)
        hx = _lerp(HX_LID * 0.988, HX_TOP * 1.010, t)
        hy = _lerp(HY_LID * 0.988, HY_TOP * 1.010, t)
        x, y = _super_point(hx, hy, s)
        nx, ny = _outward_xy(x, y, hx, hy)
        braid = 0.0007 * math.sin(math.pi * t + index * 0.53)
        points.append((x + braid * nx, y + braid * ny, z))
    return points


def _handle_lashing_path(s: float, direction: int, index: int) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for i in range(7):
        t = i / 6.0
        z = _lerp(HANDLE_BASE_Z + 0.002, HANDLE_TOP_Z - 0.001, t)
        x, y = _super_point(HX_HANDLE, HY_HANDLE, (s + direction * 0.035 * t) % 1.0)
        nx, ny = _outward_xy(x, y, HX_HANDLE, HY_HANDLE)
        points.append((x + 0.0006 * nx, y + 0.0006 * ny, z + 0.0005 * math.sin(index + t * math.pi)))
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
    model = ArticulatedObject(name="rectangular_openwork_lattice_basket_with_lift_off_lid")

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

    # Dense all-around side weave. Horizontal wavy bands make the wall read
    # filled-in; short vertical stitches and compact herringbone inserts break
    # up the surface so it is not just a simple striped basket.
    wall_z0 = Z_FOOT + 0.014
    wall_z1 = BODY_H - 0.014
    for row in range(WALL_BANDS):
        t = row / (WALL_BANDS - 1)
        z = _lerp(wall_z0, wall_z1, t)
        _add_closed_tube(
            body,
            _wall_band_path(z, row),
            radius=T_WALL_BAND,
            material="rattan_light" if row % 3 == 0 else "rattan_mid",
            name=f"woven_wall_wave_band_{row:02d}",
            radial_segments=8,
        )

    for col in range(WALL_STITCH_COLUMNS):
        s = (col + 0.5 * (col % 2)) / WALL_STITCH_COLUMNS
        band = col % (WALL_BANDS - 2)
        z0 = _lerp(wall_z0, wall_z1, band / (WALL_BANDS - 1)) + 0.003
        z1 = _lerp(wall_z0, wall_z1, (band + 1) / (WALL_BANDS - 1)) - 0.001
        _add_open_tube(
            body,
            _short_upright_stitch_path(s, z0, z1, col),
            radius=T_WALL_STITCH,
            material="rattan_shadow" if col % 5 == 0 else "rattan_mid",
            name=f"woven_wall_short_vertical_stitch_{col:02d}",
            radial_segments=6,
        )

    for col in range(HERRINGBONE_COLUMNS):
        s = (col + 0.25) / HERRINGBONE_COLUMNS
        band = col % (WALL_BANDS - 4)
        z0 = _lerp(wall_z0, wall_z1, (band + 1) / (WALL_BANDS - 1))
        for direction, label in ((1, "right"), (-1, "left")):
            _add_open_tube(
                body,
                _herringbone_stitch_path(s, z0, direction, col),
                radius=T_DIAGONAL,
                material="rattan_light" if (col + direction) % 2 else "rattan_mid",
                name=f"woven_wall_herringbone_{label}_{col:02d}",
                radial_segments=6,
            )

    for k, z in enumerate((BODY_H - 0.016, BODY_H - 0.011, BODY_H - 0.006)):
        hx, hy = _body_half_extents(z)
        _add_closed_tube(
            body,
            _rounded_rect_path(
                hx,
                hy,
                z,
                outward_amp=0.0022,
                z_amp=0.0007,
                wave_count=44,
                phase=k * 0.8,
            ),
            radius=0.0030,
            material="rattan_shadow" if k == 1 else "rattan_mid",
            name=f"upper_lidline_tie_band_{k}",
            radial_segments=8,
        )

    for col in range(32):
        _add_open_tube(
            body,
            _upper_lidline_lock_path(col / 32.0, col),
            radius=0.0024,
            material="rattan_dark" if col % 4 == 0 else "rattan_mid",
            name=f"upper_lidline_lock_stitch_{col:02d}",
            radial_segments=6,
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

    # Downturned woven skirt: in the closed pose its lower row reaches the basket
    # mouth rim, so the lid reads as fitted instead of a loose floating plate.
    for k, z in enumerate((0.002, -0.004, -LID_SKIRT_DROP)):
        hx = _lerp(HX_LID * 0.986, HX_TOP * 1.010, min(1.0, k / 2.0))
        hy = _lerp(HY_LID * 0.986, HY_TOP * 1.010, min(1.0, k / 2.0))
        _add_closed_tube(
            lid,
            _rounded_rect_path(hx, hy, z, outward_amp=0.0017, z_amp=0.0007, wave_count=38, phase=k * 0.9),
            radius=0.0037 if k < 2 else 0.0043,
            material="rattan_shadow" if k == 2 else "rattan_mid",
            name=f"lid_downturned_skirt_braid_row_{k}",
            radial_segments=8,
        )
    for i in range(36):
        _add_open_tube(
            lid,
            _lid_skirt_stake_path(i / 36.0, i),
            radius=0.0026,
            material="rattan_light" if i % 3 else "rattan_mid",
            name=f"lid_skirt_vertical_lock_stake_{i:02d}",
            radial_segments=6,
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
            [(x, y, _lid_z(x, y) - 0.001), (x, y, HANDLE_BASE_Z), (x, y, HANDLE_TOP_Z + 0.002)],
            radius=0.0028,
            material="rattan_shadow" if j % 4 == 0 else "rattan_mid",
            name=f"handle_vertical_stake_{j:02d}",
            radial_segments=7,
        )
    for k, z in enumerate((HANDLE_BASE_Z + 0.008, HANDLE_BASE_Z + 0.016, HANDLE_TOP_Z - 0.007)):
        _add_closed_tube(
            lid,
            _rounded_rect_path(HX_HANDLE * 1.006, HY_HANDLE * 1.006, z, samples=100, outward_amp=0.0007, wave_count=14, phase=k + 0.4),
            radius=0.0025,
            material="rattan_shadow" if k == 1 else "rattan_mid",
            name=f"handle_side_woven_guard_rail_{k}",
            radial_segments=7,
        )
    for j in range(20):
        for direction, label in ((1, "right"), (-1, "left")):
            _add_open_tube(
                lid,
                _handle_lashing_path(j / 20.0, direction, j),
                radius=0.0017,
                material="rattan_light" if (j + direction) % 2 else "rattan_mid",
                name=f"handle_diagonal_lashing_{label}_{j:02d}",
                radial_segments=6,
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

    wall_bands = sum(1 for v in body.visuals if (v.name or "").startswith("woven_wall_wave_band"))
    wall_stitches = sum(1 for v in body.visuals if (v.name or "").startswith("woven_wall_short_vertical_stitch"))
    herring_right = sum(1 for v in body.visuals if (v.name or "").startswith("woven_wall_herringbone_right"))
    herring_left = sum(1 for v in body.visuals if (v.name or "").startswith("woven_wall_herringbone_left"))
    upper_tie_bands = sum(1 for v in body.visuals if (v.name or "").startswith("upper_lidline_tie_band"))
    upper_locks = sum(1 for v in body.visuals if (v.name or "").startswith("upper_lidline_lock_stitch"))
    dense_rows = sum(1 for v in body.visuals if (v.name or "").startswith("horizontal_rect_weave"))
    floor = sum(1 for v in body.visuals if (v.name or "").startswith("floor_weave"))

    ctx.check(
        "dense_patterned_wall_weave_replaces_sparse_lattice",
        (
            wall_bands >= WALL_BANDS
            and wall_stitches >= WALL_STITCH_COLUMNS
            and herring_right >= HERRINGBONE_COLUMNS
            and herring_left >= HERRINGBONE_COLUMNS
            and upper_tie_bands >= 3
            and upper_locks >= 32
            and dense_rows == 0
        ),
        details=(
            f"bands={wall_bands}, stitches={wall_stitches}, "
            f"herring_right={herring_right}, herring_left={herring_left}, "
            f"upper_tie_bands={upper_tie_bands}, upper_locks={upper_locks}, "
            f"dense_rows={dense_rows}"
        ),
    )
    ctx.check(
        "wall_weave_density_not_sparse",
        (wall_bands + wall_stitches + herring_right + herring_left) >= 150,
        details=(
            f"wall_elements={wall_bands + wall_stitches + herring_right + herring_left}"
        ),
    )
    ctx.check(
        "floor_weave_preserved",
        floor >= 28,
        details=f"floor={floor}",
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
