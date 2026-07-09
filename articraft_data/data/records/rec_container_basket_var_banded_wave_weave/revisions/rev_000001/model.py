from __future__ import annotations

# Tall oval hourglass rattan basket with alternating horizontal banded wave weave.
#
# Seeded from the lift-off-lid basket family, but the visible basket language is
# changed substantially: an oval pinched-waist body, stacked wave bands in
# contrasting reed colors, and a fitted oval lid that lifts straight upward on a
# positive-Z prismatic joint.

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

H_BODY = 0.345
Z_FOOT = 0.008
Z_FLOOR = 0.018

HX_FOOT = 0.104
HY_FOOT = 0.070
HX_BELLY_LO = 0.136
HY_BELLY_LO = 0.092
HX_WAIST = 0.105
HY_WAIST = 0.069
HX_SHOULDER = 0.130
HY_SHOULDER = 0.087
HX_MOUTH = 0.119
HY_MOUTH = 0.079

HX_LID = 0.136
HY_LID = 0.092
HX_LID_WEAVE = 0.128
HY_LID_WEAVE = 0.085
LID_DOME = 0.026
LID_SEAT_Z = H_BODY
LID_LIFT = 0.185

SUPER_N = 3.0
RING_SAMPLES = 224
VERTICAL_STAKES = 44
WAVE_BANDS = 18
WAVE_LOBES = 8

T_STAKE = 0.0031
T_WAVE = 0.0040
T_RIM = 0.0058
T_FINE = 0.0026
T_LID = 0.0031
T_KNOB = 0.0036


def _sgn(value: float) -> float:
    return -1.0 if value < 0.0 else 1.0


def _lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def _profile_value(z: float, samples: list[tuple[float, float]]) -> float:
    if z <= samples[0][0]:
        return samples[0][1]
    if z >= samples[-1][0]:
        return samples[-1][1]
    for (z0, v0), (z1, v1) in zip(samples, samples[1:]):
        if z0 <= z <= z1:
            t = (z - z0) / max(z1 - z0, 1e-6)
            smooth = t * t * (3.0 - 2.0 * t)
            return _lerp(v0, v1, smooth)
    return samples[-1][1]


def _body_half_extents(z: float) -> tuple[float, float]:
    hx = _profile_value(
        z,
        [
            (0.000, HX_FOOT),
            (0.070, HX_BELLY_LO),
            (0.185, HX_WAIST),
            (0.285, HX_SHOULDER),
            (H_BODY, HX_MOUTH),
        ],
    )
    hy = _profile_value(
        z,
        [
            (0.000, HY_FOOT),
            (0.070, HY_BELLY_LO),
            (0.185, HY_WAIST),
            (0.285, HY_SHOULDER),
            (H_BODY, HY_MOUTH),
        ],
    )
    return hx, hy


def _super_point(hx: float, hy: float, s: float) -> tuple[float, float]:
    theta = 2.0 * math.pi * (s % 1.0)
    c = math.cos(theta)
    q = math.sin(theta)
    power = 2.0 / SUPER_N
    return hx * _sgn(c) * abs(c) ** power, hy * _sgn(q) * abs(q) ** power


def _outward_xy(x: float, y: float, hx: float, hy: float) -> tuple[float, float]:
    nx = x / max(hx, 1e-6)
    ny = y / max(hy, 1e-6)
    length = math.hypot(nx, ny)
    if length <= 1e-9:
        return 1.0, 0.0
    return nx / length, ny / length


def _oval_ring_path(
    hx: float,
    hy: float,
    z: float,
    *,
    samples: int = RING_SAMPLES,
    wave_count: int = 0,
    phase: float = 0.0,
    radial_amp: float = 0.0,
    z_amp: float = 0.0,
) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for i in range(samples):
        s = i / samples
        x, y = _super_point(hx, hy, s)
        nx, ny = _outward_xy(x, y, hx, hy)
        wave = math.sin(2.0 * math.pi * wave_count * s + phase) if wave_count else 0.0
        points.append((x + radial_amp * wave * nx, y + radial_amp * wave * ny, z + z_amp * wave))
    return points


def _body_wave_path(row: int) -> list[tuple[float, float, float]]:
    t = row / max(WAVE_BANDS - 1, 1)
    z = 0.038 + (H_BODY - 0.074) * t
    hx, hy = _body_half_extents(z)
    phase = (row % 4) * math.pi / 2.0
    return _oval_ring_path(
        hx + 0.0018,
        hy + 0.0018,
        z,
        wave_count=WAVE_LOBES,
        phase=phase,
        radial_amp=0.0034,
        z_amp=0.0042,
    )


def _secondary_wave_path(row: int) -> list[tuple[float, float, float]]:
    t = row / max(WAVE_BANDS - 1, 1)
    z = 0.038 + (H_BODY - 0.074) * t + 0.0065
    hx, hy = _body_half_extents(min(z, H_BODY - 0.028))
    phase = (row % 4) * math.pi / 2.0 + math.pi
    return _oval_ring_path(
        hx + 0.0004,
        hy + 0.0004,
        z,
        wave_count=WAVE_LOBES,
        phase=phase,
        radial_amp=0.0026,
        z_amp=0.0034,
    )


def _stake_path(s: float, index: int) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for i in range(38):
        t = i / 37.0
        z = Z_FOOT + (H_BODY - Z_FOOT) * t
        hx, hy = _body_half_extents(z)
        x, y = _super_point(hx, hy, s)
        nx, ny = _outward_xy(x, y, hx, hy)
        weave_relief = 0.0010 * math.sin(2.0 * math.pi * WAVE_BANDS * t + index * 0.43)
        points.append((x + (0.0012 + weave_relief) * nx, y + (0.0012 + weave_relief) * ny, z))
    return points


def _half_x_for_y(y: float, hx: float, hy: float) -> float:
    ratio = min(1.0, abs(y) / max(hy, 1e-6))
    return hx * max(0.0, 1.0 - ratio**SUPER_N) ** (1.0 / SUPER_N)


def _half_y_for_x(x: float, hx: float, hy: float) -> float:
    ratio = min(1.0, abs(x) / max(hx, 1e-6))
    return hy * max(0.0, 1.0 - ratio**SUPER_N) ** (1.0 / SUPER_N)


def _floor_chord(offset: float, *, orientation: str, samples: int = 10) -> list[tuple[float, float, float]]:
    hx, hy = _body_half_extents(Z_FOOT)
    points: list[tuple[float, float, float]] = []
    if orientation == "x":
        half = _half_x_for_y(offset, hx * 0.88, hy * 0.88)
        for i in range(samples):
            x = _lerp(-half, half, i / (samples - 1))
            points.append((x, offset, Z_FLOOR + 0.0006 * (i % 2)))
    else:
        half = _half_y_for_x(offset, hx * 0.88, hy * 0.88)
        for i in range(samples):
            y = _lerp(-half, half, i / (samples - 1))
            points.append((offset, y, Z_FLOOR + 0.0020 - 0.0006 * (i % 2)))
    return points


def _lid_z(x: float, y: float) -> float:
    edge = max(abs(x) / max(HX_LID_WEAVE, 1e-6), abs(y) / max(HY_LID_WEAVE, 1e-6))
    lift = max(0.0, 1.0 - edge)
    return 0.004 + LID_DOME * lift**0.72


def _lid_chord(offset: float, *, orientation: str, samples: int = 20) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    if orientation == "x":
        half = _half_x_for_y(offset, HX_LID_WEAVE, HY_LID_WEAVE)
        for i in range(samples):
            u = i / (samples - 1)
            x = _lerp(-half, half, u)
            y = offset
            ripple = 0.0008 * math.sin(10.0 * math.pi * u + offset * 40.0)
            points.append((x, y, _lid_z(x, y) + ripple))
    else:
        half = _half_y_for_x(offset, HX_LID_WEAVE, HY_LID_WEAVE)
        for i in range(samples):
            u = i / (samples - 1)
            x = offset
            y = _lerp(-half, half, u)
            ripple = 0.0008 * math.cos(10.0 * math.pi * u + offset * 40.0)
            points.append((x, y, _lid_z(x, y) + ripple + 0.0015))
    return points


def _knob_arch(samples: int = 56) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for i in range(samples):
        theta = 2.0 * math.pi * i / samples
        x = 0.029 * math.cos(theta)
        y = 0.014 * math.sin(theta)
        z = 0.039 + 0.010 * (1.0 + math.sin(theta)) + 0.004 * math.cos(theta) ** 2
        points.append((x, y, z))
    return points


_TUBE_COUNTER = 0


def tube_from_spline_points(points, **kwargs):
    global _TUBE_COUNTER
    _TUBE_COUNTER += 1
    geom = _sdk_tube_from_spline_points(points, **kwargs)
    return mesh_from_geometry(geom, f"banded_wave_basket_tube_{_TUBE_COUNTER:03d}")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="oval_hourglass_banded_wave_basket_with_lift_off_lid")

    model.material("rattan_honey", rgba=(0.88, 0.66, 0.34, 1.0))
    model.material("rattan_wheat", rgba=(0.96, 0.80, 0.48, 1.0))
    model.material("rattan_dark_reed", rgba=(0.40, 0.24, 0.10, 1.0))
    model.material("rattan_umber_band", rgba=(0.56, 0.31, 0.13, 1.0))
    model.material("rattan_lid_light", rgba=(0.90, 0.69, 0.38, 1.0))
    model.material("rattan_lid_dark", rgba=(0.48, 0.28, 0.12, 1.0))
    model.material("deep_shadow", rgba=(0.07, 0.045, 0.025, 1.0))

    body = model.part("basket_body")

    floor_offsets = [-0.054 + i * 0.018 for i in range(7)]
    for i, off in enumerate(floor_offsets):
        body.visual(
            tube_from_spline_points(
                _floor_chord(off, orientation="x"),
                radius=T_FINE,
                samples_per_segment=1,
                radial_segments=6,
                cap_ends=True,
            ),
            material="rattan_wheat" if i % 2 == 0 else "rattan_honey",
            name=f"oval_bottom_floor_x_{i:02d}",
        )
        body.visual(
            tube_from_spline_points(
                _floor_chord(off, orientation="y"),
                radius=T_FINE,
                samples_per_segment=1,
                radial_segments=6,
                cap_ends=True,
            ),
            material="rattan_honey" if i % 2 == 0 else "rattan_wheat",
            name=f"oval_bottom_floor_y_{i:02d}",
        )
    body.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, Z_FLOOR - 0.004)),
        material="deep_shadow",
        name="oval_floor_dark_center_shadow",
    )

    for phase_index, phase in enumerate((0.0, math.pi)):
        hx, hy = _body_half_extents(Z_FOOT)
        body.visual(
            tube_from_spline_points(
                _oval_ring_path(hx + 0.002, hy + 0.002, Z_FOOT, wave_count=22, phase=phase, radial_amp=0.0019, z_amp=0.0012),
                radius=T_RIM,
                closed_spline=True,
                samples_per_segment=1,
                radial_segments=9,
                cap_ends=False,
            ),
            material="rattan_honey" if phase_index == 0 else "rattan_dark_reed",
            name=f"braided_oval_foot_ring_{phase_index}",
        )

    for j in range(VERTICAL_STAKES):
        s = j / VERTICAL_STAKES
        body.visual(
            tube_from_spline_points(
                _stake_path(s, j),
                radius=T_STAKE,
                samples_per_segment=2,
                radial_segments=6,
                cap_ends=True,
                up_hint=(0.0, 0.0, 1.0),
            ),
            material="rattan_dark_reed" if j % 5 == 0 else "rattan_umber_band",
            name=f"hourglass_vertical_stake_{j:02d}",
        )

    for row in range(WAVE_BANDS):
        primary_dark = (row // 3) % 2 == 1
        body.visual(
            tube_from_spline_points(
                _body_wave_path(row),
                radius=T_WAVE,
                closed_spline=True,
                samples_per_segment=1,
                radial_segments=8,
                cap_ends=False,
            ),
            material="rattan_dark_reed" if primary_dark else "rattan_wheat",
            name=f"primary_horizontal_wave_band_{row:02d}",
        )
        body.visual(
            tube_from_spline_points(
                _secondary_wave_path(row),
                radius=T_WAVE * 0.72,
                closed_spline=True,
                samples_per_segment=1,
                radial_segments=7,
                cap_ends=False,
            ),
            material="rattan_umber_band" if primary_dark else "rattan_honey",
            name=f"contrasting_reverse_wave_band_{row:02d}",
        )

    for k, z in enumerate((0.078, 0.184, 0.286)):
        hx, hy = _body_half_extents(z)
        body.visual(
            tube_from_spline_points(
                _oval_ring_path(hx + 0.003, hy + 0.003, z, wave_count=WAVE_LOBES, phase=k * math.pi / 3.0, radial_amp=0.0020, z_amp=0.0020),
                radius=0.0050,
                closed_spline=True,
                samples_per_segment=1,
                radial_segments=9,
                cap_ends=False,
            ),
            material="rattan_dark_reed" if k == 1 else "rattan_honey",
            name=f"wide_contrast_girdle_wave_band_{k}",
        )

    for phase_index, phase in enumerate((0.0, math.pi)):
        body.visual(
            tube_from_spline_points(
                _oval_ring_path(HX_MOUTH + 0.004, HY_MOUTH + 0.004, H_BODY, wave_count=26, phase=phase, radial_amp=0.0024, z_amp=0.0016),
                radius=T_RIM,
                closed_spline=True,
                samples_per_segment=1,
                radial_segments=10,
                cap_ends=False,
            ),
            material="rattan_wheat" if phase_index == 0 else "rattan_dark_reed",
            name=f"braided_oval_mouth_rim_{phase_index}",
        )

    lid = model.part("basket_lid")

    lid_offsets_y = [-0.071 + i * 0.011 for i in range(14)]
    for i, off in enumerate(lid_offsets_y):
        lid.visual(
            tube_from_spline_points(
                _lid_chord(off, orientation="x"),
                radius=T_LID,
                samples_per_segment=1,
                radial_segments=7,
                cap_ends=True,
            ),
            material="rattan_lid_light" if i % 2 == 0 else "rattan_lid_dark",
            name=f"fitted_lid_lengthwise_wave_strip_{i:02d}",
        )

    lid_offsets_x = [-0.104 + i * 0.016 for i in range(14)]
    for i, off in enumerate(lid_offsets_x):
        lid.visual(
            tube_from_spline_points(
                _lid_chord(off, orientation="y"),
                radius=T_LID * 0.92,
                samples_per_segment=1,
                radial_segments=7,
                cap_ends=True,
            ),
            material="rattan_lid_dark" if i % 2 == 0 else "rattan_lid_light",
            name=f"fitted_lid_cross_wave_strip_{i:02d}",
        )

    for row, local_z in enumerate((0.010, 0.016, 0.023)):
        lid.visual(
            tube_from_spline_points(
                _oval_ring_path(
                    HX_LID_WEAVE * (0.42 + 0.17 * row),
                    HY_LID_WEAVE * (0.42 + 0.17 * row),
                    local_z,
                    wave_count=WAVE_LOBES,
                    phase=row * math.pi / 2.0,
                    radial_amp=0.0022,
                    z_amp=0.0015,
                ),
                radius=0.0024,
                closed_spline=True,
                samples_per_segment=1,
                radial_segments=6,
                cap_ends=False,
            ),
            material="rattan_lid_dark" if row % 2 else "rattan_lid_light",
            name=f"lid_concentric_echo_wave_{row}",
        )

    for phase_index, phase in enumerate((0.0, math.pi)):
        lid.visual(
            tube_from_spline_points(
                _oval_ring_path(HX_LID, HY_LID, 0.005, wave_count=24, phase=phase, radial_amp=0.0025, z_amp=0.0013),
                radius=0.0060,
                closed_spline=True,
                samples_per_segment=1,
                radial_segments=10,
                cap_ends=False,
            ),
            material="rattan_lid_light" if phase_index == 0 else "rattan_lid_dark",
            name=f"fitted_lid_braided_outer_rim_{phase_index}",
        )

    lid.visual(
        tube_from_spline_points(
            _knob_arch(),
            radius=T_KNOB,
            closed_spline=True,
            samples_per_segment=1,
            radial_segments=8,
            cap_ends=False,
            up_hint=(0.0, 0.0, 1.0),
        ),
        material="rattan_lid_dark",
        name="raised_oval_lift_knob_loop",
    )
    for i, x in enumerate((-0.022, 0.022)):
        lid.visual(
            tube_from_spline_points(
                [(x, -0.006, 0.033), (x * 0.72, 0.000, 0.043), (x, 0.006, 0.033)],
                radius=0.0028,
                samples_per_segment=2,
                radial_segments=7,
                cap_ends=True,
            ),
            material="rattan_lid_dark",
            name=f"raised_oval_lift_knob_mount_{i}",
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, LID_SEAT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.38, lower=0.0, upper=LID_LIFT),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("basket_body")
    lid = object_model.get_part("basket_lid")
    joint = object_model.get_articulation("body_to_lid")

    stakes = sum(1 for v in body.visuals if (v.name or "").startswith("hourglass_vertical_stake_"))
    primary_bands = sum(1 for v in body.visuals if (v.name or "").startswith("primary_horizontal_wave_band_"))
    reverse_bands = sum(1 for v in body.visuals if (v.name or "").startswith("contrasting_reverse_wave_band_"))
    girdles = sum(1 for v in body.visuals if (v.name or "").startswith("wide_contrast_girdle_wave_band_"))
    ctx.check(
        "alternating_banded_wave_weave_present",
        stakes >= VERTICAL_STAKES
        and primary_bands >= WAVE_BANDS
        and reverse_bands >= WAVE_BANDS
        and girdles == 3,
        f"stakes={stakes}, primary={primary_bands}, reverse={reverse_bands}, girdles={girdles}",
    )

    open_lattice_names = sum(1 for v in body.visuals if "lattice" in (v.name or "").lower())
    checker_names = sum(1 for v in body.visuals if "checker" in (v.name or "").lower())
    diagonal_names = sum(1 for v in body.visuals if "diagonal" in (v.name or "").lower())
    ctx.check(
        "not_lattice_checker_or_diagonal_twill",
        open_lattice_names == 0 and checker_names == 0 and diagonal_names == 0,
        f"lattice={open_lattice_names}, checker={checker_names}, diagonal={diagonal_names}",
    )

    foot_hx, foot_hy = _body_half_extents(Z_FOOT)
    waist_hx, waist_hy = _body_half_extents(0.185)
    shoulder_hx, shoulder_hy = _body_half_extents(0.285)
    ctx.check(
        "tall_oval_hourglass_profile",
        H_BODY > 0.32
        and shoulder_hx > waist_hx * 1.18
        and foot_hx < shoulder_hx
        and (shoulder_hx / shoulder_hy) > 1.35
        and (foot_hx / foot_hy) > 1.35,
        (
            f"foot=({foot_hx:.3f},{foot_hy:.3f}), waist=({waist_hx:.3f},{waist_hy:.3f}), "
            f"shoulder=({shoulder_hx:.3f},{shoulder_hy:.3f}), height={H_BODY:.3f}"
        ),
    )

    lid_x = sum(1 for v in lid.visuals if (v.name or "").startswith("fitted_lid_lengthwise_wave_strip_"))
    lid_y = sum(1 for v in lid.visuals if (v.name or "").startswith("fitted_lid_cross_wave_strip_"))
    lid_rim = sum(1 for v in lid.visuals if (v.name or "").startswith("fitted_lid_braided_outer_rim_"))
    knob = sum(1 for v in lid.visuals if (v.name or "").startswith("raised_oval_lift_knob"))
    ctx.check(
        "fitted_woven_lid_with_lift_knob",
        lid_x >= 14 and lid_y >= 14 and lid_rim >= 2 and knob >= 3,
        f"lid_x={lid_x}, lid_y={lid_y}, lid_rim={lid_rim}, knob={knob}",
    )

    ctx.check(
        "body_to_lid_lifts_upward",
        joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(joint.axis) == (0.0, 0.0, 1.0)
        and joint.motion_limits is not None
        and joint.motion_limits.lower == 0.0
        and joint.motion_limits.upper >= LID_LIFT
        and joint.motion_limits.upper > 0.0,
        f"type={joint.articulation_type}, axis={joint.axis}, limits={joint.motion_limits}",
    )

    ctx.allow_overlap(
        lid,
        body,
        reason=(
            "Closed fitted lift-off lid: the oval braided lid rim sits down over "
            "the basket mouth rim with a small seating overlap."
        ),
    )
    ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.18, name="oval_lid_covers_mouth")

    with ctx.pose({joint: LID_LIFT}):
        ctx.expect_gap(lid, body, axis="z", min_gap=0.010, name="raised_lid_lifts_clear_of_body")

    return ctx.report()


object_model = build_object_model()
