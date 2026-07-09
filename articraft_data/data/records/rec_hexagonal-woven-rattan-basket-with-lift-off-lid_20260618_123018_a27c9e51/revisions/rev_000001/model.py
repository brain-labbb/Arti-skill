from __future__ import annotations

# Hexagonal woven rattan basket with a fitted lift-off lid.
#
# Reference traits from picture/Container/Basket/002.png:
# - low, broad basket box with a rounded/soft hexagonal footprint;
# - natural tan over-under rattan weave over the sides;
# - a teal woven band around the body with short red accent stitches;
# - a thick near-hexagonal lift-off lid, overhanging the body;
# - red/teal decorative hexagonal lines and two raised leaf-like woven panels on
#   the lid top. The lid has no big visible locating ring underneath.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points as _sdk_tube_from_spline_points,
)

H_BODY = 0.165
Z_FOOT = 0.006
Z_BOTTOM_FLOOR = 0.008

HX_BOTTOM = 0.130
HY_BOTTOM = 0.096
HX_MID = 0.150
HY_MID = 0.112
HX_TOP = 0.142
HY_TOP = 0.106

HX_LID = 0.172
HY_LID = 0.128
LID_WEAVE_HX = 0.174
LID_WEAVE_HY = 0.130
LID_DOME = 0.014
LID_SEAT_Z = H_BODY - 0.003
LID_LIFT = 0.135

BODY_ROWS = 18
STAKE_COUNT = 48
SAMPLES_PER_EDGE = 14

T_HORIZONTAL = 0.0042
T_VERTICAL = 0.0030
T_BODY_RIM = 0.0050
T_LID_RIM = 0.0065
T_LID_STRIP = 0.0031
WEAVE_RELIEF = 0.0020

LID_X_STRIP_OFFSETS = [
    -0.118,
    -0.104,
    -0.090,
    -0.076,
    -0.062,
    -0.048,
    -0.034,
    -0.020,
    -0.006,
    0.008,
    0.022,
    0.036,
    0.050,
    0.064,
    0.078,
    0.092,
    0.106,
    0.120,
]
LID_Y_STRIP_OFFSETS = [
    -0.154,
    -0.136,
    -0.118,
    -0.100,
    -0.082,
    -0.064,
    -0.046,
    -0.028,
    -0.010,
    0.008,
    0.026,
    0.044,
    0.062,
    0.080,
    0.098,
    0.116,
    0.134,
    0.152,
]


def _lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def _hex_vertices(hx: float, hy: float) -> list[tuple[float, float]]:
    shoulder = 0.58 * hx
    return [
        (hx, 0.0),
        (shoulder, hy),
        (-shoulder, hy),
        (-hx, 0.0),
        (-shoulder, -hy),
        (shoulder, -hy),
    ]


def _body_half_extents(z: float) -> tuple[float, float]:
    t = max(0.0, min(1.0, (z - Z_FOOT) / (H_BODY - Z_FOOT)))
    belly = math.sin(math.pi * t)
    hx = _lerp(HX_BOTTOM, HX_TOP, t) + (HX_MID - 0.5 * (HX_BOTTOM + HX_TOP)) * belly
    hy = _lerp(HY_BOTTOM, HY_TOP, t) + (HY_MID - 0.5 * (HY_BOTTOM + HY_TOP)) * belly
    return hx, hy


def _outward_xy(x: float, y: float) -> tuple[float, float]:
    length = math.hypot(x, y)
    if length <= 1e-9:
        return 1.0, 0.0
    return x / length, y / length


def _hex_path(
    hx: float,
    hy: float,
    z: float,
    *,
    samples_per_edge: int = SAMPLES_PER_EDGE,
    outward_amp: float = 0.0,
    z_amp: float = 0.0,
    wave_count: int = 0,
    phase: float = 0.0,
) -> list[tuple[float, float, float]]:
    verts = _hex_vertices(hx, hy)
    points: list[tuple[float, float, float]] = []
    total_samples = len(verts) * samples_per_edge
    for edge, (a, b) in enumerate(zip(verts, verts[1:] + verts[:1])):
        for j in range(samples_per_edge):
            u = j / samples_per_edge
            x = _lerp(a[0], b[0], u)
            y = _lerp(a[1], b[1], u)
            sample_i = edge * samples_per_edge + j
            wave = math.sin(2.0 * math.pi * wave_count * sample_i / total_samples + phase)
            nx, ny = _outward_xy(x, y)
            points.append((x + outward_amp * wave * nx, y + outward_amp * wave * ny, z + z_amp * wave))
    return points


def _perimeter_point(hx: float, hy: float, s: float) -> tuple[float, float]:
    verts = _hex_vertices(hx, hy)
    pos = (s % 1.0) * len(verts)
    edge = int(math.floor(pos)) % len(verts)
    u = pos - math.floor(pos)
    a = verts[edge]
    b = verts[(edge + 1) % len(verts)]
    return (_lerp(a[0], b[0], u), _lerp(a[1], b[1], u))


def _upright_path(s: float, index: int) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    samples = 24
    for i in range(samples):
        t = i / (samples - 1)
        z = Z_FOOT + (H_BODY - Z_FOOT) * t
        hx, hy = _body_half_extents(z)
        x, y = _perimeter_point(hx, hy, s)
        nx, ny = _outward_xy(x, y)
        flutter = 0.0008 * math.sin(2.0 * math.pi * BODY_ROWS * t + index * 0.41)
        points.append((x + (0.0010 + flutter) * nx, y + (0.0010 + flutter) * ny, z))
    return points


def _hex_half_x_for_y(y: float, hx: float, hy: float) -> float:
    ay = abs(y)
    if ay >= hy:
        return 0.0
    return hx - (hx - 0.58 * hx) * (ay / hy)


def _hex_half_y_for_x(x: float, hx: float, hy: float) -> float:
    ax = abs(x)
    shoulder = 0.58 * hx
    if ax <= shoulder:
        return hy
    if ax >= hx:
        return 0.0
    return hy * (hx - ax) / (hx - shoulder)


def _lid_z(x: float, y: float) -> float:
    # A shallow, cushiony woven lid surface.
    rx = x / max(LID_WEAVE_HX, 1e-6)
    ry = y / max(LID_WEAVE_HY, 1e-6)
    falloff = max(0.0, 1.0 - 0.50 * (rx * rx + ry * ry))
    return 0.004 + LID_DOME * falloff


def _lid_chord(
    offset: float,
    *,
    orientation: str,
    samples: int = 18,
    phase: float = 0.0,
) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    if orientation == "x":
        half = _hex_half_x_for_y(offset, LID_WEAVE_HX, LID_WEAVE_HY)
        for i in range(samples):
            t = i / (samples - 1)
            x = _lerp(-half, half, t)
            y = offset
            ripple = 0.0008 * math.sin(8.0 * math.pi * t + phase)
            points.append((x, y, _lid_z(x, y) + ripple))
    else:
        half = _hex_half_y_for_x(offset, LID_WEAVE_HX, LID_WEAVE_HY)
        for i in range(samples):
            t = i / (samples - 1)
            x = offset
            y = _lerp(-half, half, t)
            ripple = 0.0008 * math.sin(8.0 * math.pi * t + phase)
            points.append((x, y, _lid_z(x, y) + ripple))
    return points


def _offset_hex_path(
    hx: float,
    hy: float,
    z: float,
    *,
    offset: float,
    samples_per_edge: int = SAMPLES_PER_EDGE,
) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for x, y, zz in _hex_path(hx, hy, z, samples_per_edge=samples_per_edge):
        nx, ny = _outward_xy(x, y)
        points.append((x + offset * nx, y + offset * ny, zz))
    return points


def _path_segment(
    points: list[tuple[float, float, float]],
    start: int,
    count: int,
) -> list[tuple[float, float, float]]:
    n = len(points)
    return [points[(start + i) % n] for i in range(count)]


def _leaf_loop(
    *,
    center: tuple[float, float],
    length: float,
    width: float,
    angle: float,
    z: float,
    samples: int = 72,
) -> list[tuple[float, float, float]]:
    ca = math.cos(angle)
    sa = math.sin(angle)
    points: list[tuple[float, float, float]] = []
    for i in range(samples):
        th = 2.0 * math.pi * i / samples
        # Pointed oval leaf: width collapses near both tips.
        lx = 0.5 * length * math.cos(th)
        ly = 0.5 * width * math.sin(th) * (0.65 + 0.35 * abs(math.sin(th)))
        x = center[0] + ca * lx - sa * ly
        y = center[1] + sa * lx + ca * ly
        points.append((x, y, z + 0.0015 * math.sin(th)))
    return points


def _leaf_rib(
    *,
    center: tuple[float, float],
    length: float,
    angle: float,
    z: float,
) -> list[tuple[float, float, float]]:
    ca = math.cos(angle)
    sa = math.sin(angle)
    return [
        (center[0] - ca * length * 0.42, center[1] - sa * length * 0.42, z),
        (center[0], center[1], z + 0.002),
        (center[0] + ca * length * 0.42, center[1] + sa * length * 0.42, z),
    ]


_TUBE_MESH_COUNTER = 0


def tube_from_spline_points(points, **kwargs):
    global _TUBE_MESH_COUNTER
    _TUBE_MESH_COUNTER += 1
    geom = _sdk_tube_from_spline_points(points, **kwargs)
    return mesh_from_geometry(geom, f"hex_woven_basket_tube_{_TUBE_MESH_COUNTER:03d}")


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
    model = ArticulatedObject(name="hexagonal_woven_rattan_basket_with_lift_off_lid")

    model.material("rattan_light", rgba=(0.90, 0.68, 0.36, 1.0))
    model.material("rattan_mid", rgba=(0.76, 0.50, 0.22, 1.0))
    model.material("rattan_shadow", rgba=(0.50, 0.31, 0.12, 1.0))
    model.material("rattan_dark", rgba=(0.36, 0.21, 0.08, 1.0))
    model.material("woven_teal", rgba=(0.02, 0.48, 0.41, 1.0))
    model.material("woven_red", rgba=(0.78, 0.06, 0.13, 1.0))
    model.material("woven_coral", rgba=(0.88, 0.28, 0.18, 1.0))

    body = model.part("basket_body")

    # Foot and floor are also hexagonal so the object reads as a box basket, not
    # a round pot.
    bottom_hx, bottom_hy = _body_half_extents(Z_FOOT)
    _add_closed_tube(
        body,
        _hex_path(bottom_hx, bottom_hy, Z_FOOT, outward_amp=0.0016, wave_count=24),
        radius=T_BODY_RIM,
        material="rattan_mid",
        name="hexagonal_bottom_foot_ring",
        radial_segments=9,
    )

    # Real woven bottom: two crossed families of cane strips span the full
    # hexagonal footprint and physically touch the foot ring at their ends.
    floor_y_offsets = [
        -0.080,
        -0.068,
        -0.056,
        -0.044,
        -0.032,
        -0.020,
        -0.008,
        0.004,
        0.016,
        0.028,
        0.040,
        0.052,
        0.064,
        0.076,
        0.088,
    ]
    for i, off in enumerate(floor_y_offsets):
        half = _hex_half_x_for_y(off, bottom_hx * 0.98, bottom_hy * 0.98)
        if half <= 0.0:
            continue
        _add_open_tube(
            body,
            [
                (-half - 0.004, off, Z_BOTTOM_FLOOR),
                (0.0, off, Z_BOTTOM_FLOOR + (0.0015 if i % 2 == 0 else -0.0005)),
                (half + 0.004, off, Z_BOTTOM_FLOOR),
            ],
            radius=0.0028,
            material="rattan_light" if i % 2 == 0 else "rattan_mid",
            name=f"woven_bottom_floor_x_strip_{i:02d}",
            radial_segments=7,
        )
    floor_x_offsets = [
        -0.112,
        -0.096,
        -0.080,
        -0.064,
        -0.048,
        -0.032,
        -0.016,
        0.000,
        0.016,
        0.032,
        0.048,
        0.064,
        0.080,
        0.096,
        0.112,
    ]
    for i, off in enumerate(floor_x_offsets):
        half = _hex_half_y_for_x(off, bottom_hx * 0.98, bottom_hy * 0.98)
        if half <= 0.0:
            continue
        _add_open_tube(
            body,
            [
                (off, -half - 0.004, Z_BOTTOM_FLOOR + 0.0010),
                (off, 0.0, Z_BOTTOM_FLOOR + (0.0020 if i % 2 else 0.0000)),
                (off, half + 0.004, Z_BOTTOM_FLOOR + 0.0010),
            ],
            radius=0.0027,
            material="rattan_mid" if i % 2 == 0 else "rattan_light",
            name=f"woven_bottom_floor_y_strip_{i:02d}",
            radial_segments=7,
        )

    for j in range(STAKE_COUNT):
        _add_open_tube(
            body,
            _upright_path(j / STAKE_COUNT, j),
            radius=T_VERTICAL,
            material="rattan_shadow" if j % 5 == 0 else "rattan_mid",
            name=f"vertical_hex_weave_stake_{j:02d}",
            radial_segments=7,
        )

    row_z0 = 0.022
    row_z1 = H_BODY - 0.018
    for i in range(BODY_ROWS):
        t = i / (BODY_ROWS - 1)
        z = _lerp(row_z0, row_z1, t)
        hx, hy = _body_half_extents(z)
        _add_closed_tube(
            body,
            _hex_path(
                hx,
                hy,
                z,
                outward_amp=WEAVE_RELIEF,
                z_amp=0.0006,
                wave_count=STAKE_COUNT // 2,
                phase=(i % 2) * math.pi,
            ),
            radius=T_HORIZONTAL,
            material="rattan_light" if i % 2 == 0 else "rattan_mid",
            name=f"hex_horizontal_weave_band_{i:02d}",
            radial_segments=8,
        )

    # Teal middle sash and red/coral stitches, matching the reference photo.
    for k, z in enumerate((0.074, 0.083, 0.092)):
        hx, hy = _body_half_extents(z)
        _add_closed_tube(
            body,
            _hex_path(hx, hy, z, outward_amp=0.0025, wave_count=STAKE_COUNT // 2, phase=k * 0.7),
            radius=0.0046,
            material="woven_teal",
            name=f"teal_middle_woven_band_{k}",
            radial_segments=8,
        )
    dash_source = _hex_path(*_body_half_extents(0.096), 0.096, outward_amp=0.0030, wave_count=0)
    for d, start in enumerate(range(2, len(dash_source), 9)):
        material = "woven_red" if d % 2 == 0 else "woven_coral"
        _add_open_tube(
            body,
            _path_segment(dash_source, start, 4),
            radius=0.0044,
            material=material,
            name=f"red_body_accent_stitch_{d:02d}",
            radial_segments=7,
        )

    top_hx, top_hy = _body_half_extents(H_BODY)
    for strand, phase in enumerate((0.0, math.pi)):
        _add_closed_tube(
            body,
            _hex_path(
                top_hx,
                top_hy,
                H_BODY,
                outward_amp=0.0024,
                z_amp=0.0012,
                wave_count=30,
                phase=phase,
            ),
            radius=0.0052,
            material="rattan_light" if strand == 0 else "rattan_mid",
            name=f"body_braided_hex_mouth_rim_{strand}",
            radial_segments=9,
        )

    lid = model.part("basket_lid")

    # Shallow woven lid face.
    for i, off in enumerate(LID_X_STRIP_OFFSETS):
        _add_open_tube(
            lid,
            _lid_chord(off, orientation="x", phase=i * math.pi),
            radius=T_LID_STRIP,
            material="rattan_light" if i % 2 == 0 else "rattan_mid",
            name=f"lid_hex_weave_x_strip_{i:02d}",
            radial_segments=7,
        )
    for i, off in enumerate(LID_Y_STRIP_OFFSETS):
        _add_open_tube(
            lid,
            _lid_chord(off, orientation="y", phase=i * math.pi + math.pi * 0.5),
            radius=T_LID_STRIP,
            material="rattan_mid" if i % 2 == 0 else "rattan_light",
            name=f"lid_hex_weave_y_strip_{i:02d}",
            radial_segments=7,
        )

    # Thick overhanging hexagonal lid rim and two lower side rows. These are the
    # visible lid edge, not an underside locating ring.
    for strand, phase in enumerate((0.0, math.pi)):
        _add_closed_tube(
            lid,
            _hex_path(HX_LID, HY_LID, 0.001, outward_amp=0.0032, z_amp=0.0016, wave_count=30, phase=phase),
            radius=T_LID_RIM,
            material="rattan_light" if strand == 0 else "rattan_mid",
            name=f"braided_hex_lid_outer_rim_{strand}",
            radial_segments=10,
        )
    for k, z in enumerate((-0.010, -0.004)):
        _add_closed_tube(
            lid,
            _hex_path(HX_LID * 0.975, HY_LID * 0.975, z, outward_amp=0.0010, wave_count=24, phase=k),
            radius=0.0042,
            material="rattan_mid",
            name=f"lid_outer_side_weave_row_{k}",
            radial_segments=8,
        )

    # Top red/teal decorative hexagonal lines.
    for name, mat, hx, hy, z in (
        ("lid_red_outer_hex_decoration", "woven_red", HX_LID * 0.78, HY_LID * 0.77, 0.018),
        ("lid_teal_inner_hex_decoration", "woven_teal", HX_LID * 0.69, HY_LID * 0.68, 0.019),
        ("lid_red_inner_hex_decoration", "woven_red", HX_LID * 0.59, HY_LID * 0.58, 0.020),
    ):
        _add_closed_tube(
            lid,
            _offset_hex_path(hx, hy, z, offset=0.0005, samples_per_edge=12),
            radius=0.0028,
            material=mat,
            name=name,
            radial_segments=7,
        )

    lid_dash_source = _hex_path(HX_LID * 0.955, HY_LID * 0.955, -0.003, outward_amp=0.0020)
    for d, start in enumerate(range(1, len(lid_dash_source), 10)):
        _add_open_tube(
            lid,
            _path_segment(lid_dash_source, start, 4),
            radius=0.0038,
            material="woven_red",
            name=f"red_lid_side_stitch_{d:02d}",
            radial_segments=7,
        )

    for i in range(12):
        s = i / 12.0
        outer_x, outer_y = _perimeter_point(HX_LID * 0.965, HY_LID * 0.965, s)
        inner_x, inner_y = _perimeter_point(LID_WEAVE_HX * 0.965, LID_WEAVE_HY * 0.965, s)
        mid_x = (outer_x + inner_x) * 0.5
        mid_y = (outer_y + inner_y) * 0.5
        _add_open_tube(
            lid,
            [
                (outer_x, outer_y, 0.000),
                (mid_x, mid_y, _lid_z(mid_x, mid_y) * 0.65),
                (inner_x, inner_y, _lid_z(inner_x, inner_y)),
            ],
            radius=0.0034,
            material="rattan_mid" if i % 2 else "rattan_light",
            name=f"lid_rim_to_face_connector_{i:02d}",
            radial_segments=7,
        )

    # Two raised leaf-like woven panels on the lid top.
    leaf_specs = (
        ((-0.045, 0.002), 0.080, 0.043, 0.18, "left"),
        ((0.045, 0.002), 0.080, 0.043, -0.18, "right"),
    )
    for center, length, width, angle, label in leaf_specs:
        _add_closed_tube(
            lid,
            _leaf_loop(center=center, length=length, width=width, angle=angle, z=0.022),
            radius=0.0035,
            material="rattan_light",
            name=f"raised_leaf_woven_panel_{label}",
            radial_segments=8,
        )
        _add_open_tube(
            lid,
            _leaf_rib(center=center, length=length * 0.82, angle=angle, z=0.021),
            radius=0.0031,
            material="rattan_mid",
            name=f"raised_leaf_center_rib_{label}",
            radial_segments=7,
        )
    _add_open_tube(
        lid,
        [(0.0, -0.062, 0.021), (0.0, 0.0, 0.023), (0.0, 0.062, 0.021)],
        radius=0.0036,
        material="rattan_mid",
        name="central_lid_spine_between_leaf_panels",
        radial_segments=8,
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


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("basket_body")
    lid = object_model.get_part("basket_lid")
    joint = object_model.get_articulation("body_to_lid")

    body_horizontal = sum(1 for v in body.visuals if (v.name or "").startswith("hex_horizontal_weave"))
    body_vertical = sum(1 for v in body.visuals if (v.name or "").startswith("vertical_hex_weave"))
    teal_bands = sum(1 for v in body.visuals if (v.name or "").startswith("teal_middle"))
    red_body = sum(1 for v in body.visuals if (v.name or "").startswith("red_body"))
    bottom_floor = sum(1 for v in body.visuals if (v.name or "").startswith("woven_bottom_floor"))
    ctx.check(
        "hexagonal_interlaced_body_with_color_band",
        body_horizontal >= BODY_ROWS
        and body_vertical >= STAKE_COUNT
        and teal_bands >= 3
        and red_body >= 8
        and bottom_floor >= 26,
        details=(
            f"horizontal={body_horizontal}, vertical={body_vertical}, "
            f"teal={teal_bands}, red={red_body}, bottom={bottom_floor}"
        ),
    )

    lid_x = sum(1 for v in lid.visuals if (v.name or "").startswith("lid_hex_weave_x"))
    lid_y = sum(1 for v in lid.visuals if (v.name or "").startswith("lid_hex_weave_y"))
    leaf_panels = sum(1 for v in lid.visuals if (v.name or "").startswith("raised_leaf_woven_panel"))
    rim_count = sum(1 for v in lid.visuals if (v.name or "").startswith("braided_hex_lid_outer_rim"))
    color_hex = sum(1 for v in lid.visuals if "hex_decoration" in (v.name or ""))
    ctx.check(
        "hexagonal_woven_lid_with_leaf_panels",
        lid_x >= len(LID_X_STRIP_OFFSETS)
        and lid_y >= len(LID_Y_STRIP_OFFSETS)
        and leaf_panels == 2
        and rim_count >= 2
        and color_hex >= 3,
        details=(
            f"lid_x={lid_x}, lid_y={lid_y}, leaves={leaf_panels}, "
            f"rim={rim_count}, color_hex={color_hex}"
        ),
    )

    ctx.check(
        "lift_off_lid_prismatic_axis",
        joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={joint.articulation_type}, axis={joint.axis}",
    )

    ctx.allow_overlap(
        lid,
        body,
        reason=(
            "Closed lift-off basket lid: the overhanging woven lid edge sits down "
            "onto the mouth rim with a small intentional seating overlap."
        ),
    )
    ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.22, name="hex_lid_covers_mouth")

    with ctx.pose({joint: LID_LIFT}):
        ctx.expect_gap(lid, body, axis="z", min_gap=0.005, name="hex_lid_lifts_clear")

    return ctx.report()


object_model = build_object_model()
