from __future__ import annotations

# Oval woven rattan basket with a fitted woven lid and raised central handle.
#
# Reference image: a squat oval/rounded-rect rattan basket, dense horizontal
# side weaving, a full top lid with a braided rim, and a small raised woven
# handle centered on the lid. The body includes a woven bottom panel, and the
# lid lifts off on a vertical prismatic joint.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_profile,
    tube_from_spline_points as _sdk_tube_from_spline_points,
)

SUPER_N = 3.15
BODY_H = 0.158
LID_LIFT = 0.135

RING_SAMPLES = 216
BODY_ROWS = 24
BODY_STAKES = 44

T_SIDE = 0.0034
T_STAKE = 0.0026
T_RIM = 0.0047
T_LID = 0.0029
T_HANDLE = 0.0038

LID_A = 0.188
LID_B = 0.125
LID_DOME = 0.026
LID_BASE_Z = 0.0005

HANDLE_A = 0.043
HANDLE_B = 0.023
HANDLE_BASE_Z = LID_BASE_Z + LID_DOME + 0.004
HANDLE_TOP_Z = HANDLE_BASE_Z + 0.018

_BODY_PROFILE = [
    (0.000, 0.126, 0.080),
    (0.018, 0.156, 0.101),
    (0.050, 0.181, 0.119),
    (0.088, 0.190, 0.126),
    (0.122, 0.189, 0.125),
    (0.148, 0.186, 0.123),
    (BODY_H, 0.184, 0.121),
]


def _signed_pow(value: float, power: float) -> float:
    return math.copysign(abs(value) ** power, value)


def _super_point(theta: float, a: float, b: float, n: float = SUPER_N) -> tuple[float, float]:
    return (
        a * _signed_pow(math.cos(theta), 2.0 / n),
        b * _signed_pow(math.sin(theta), 2.0 / n),
    )


def _axes_at_z(z: float) -> tuple[float, float]:
    pts = _BODY_PROFILE
    if z <= pts[0][0]:
        return pts[0][1], pts[0][2]
    if z >= pts[-1][0]:
        return pts[-1][1], pts[-1][2]
    for (z0, a0, b0), (z1, a1, b1) in zip(pts, pts[1:]):
        if z0 <= z <= z1:
            t = (z - z0) / (z1 - z0)
            return a0 + (a1 - a0) * t, b0 + (b1 - b0) * t
    return pts[-1][1], pts[-1][2]


def _closed_super_path(
    a: float,
    b: float,
    z: float,
    *,
    samples: int = RING_SAMPLES,
    weave_count: int = 0,
    phase: float = 0.0,
    amp: float = 0.0,
    z_amp: float = 0.0,
    n: float = SUPER_N,
) -> list[tuple[float, float, float]]:
    pts: list[tuple[float, float, float]] = []
    for i in range(samples):
        theta = 2.0 * math.pi * i / samples
        wave = math.cos(weave_count * theta + phase) if weave_count else 0.0
        aa = a + amp * wave
        bb = b + amp * 0.68 * wave
        x, y = _super_point(theta, aa, bb, n=n)
        zz = z + (z_amp * math.sin(weave_count * theta + phase) if weave_count else 0.0)
        pts.append((x, y, zz))
    return pts


def _braid_path(a: float, b: float, z: float, *, phase: float, turns: int) -> list[tuple[float, float, float]]:
    pts: list[tuple[float, float, float]] = []
    for i in range(RING_SAMPLES):
        theta = 2.0 * math.pi * i / RING_SAMPLES
        twist = turns * theta + phase
        aa = a + 0.0024 * math.sin(twist)
        bb = b + 0.0017 * math.sin(twist)
        x, y = _super_point(theta, aa, bb)
        pts.append((x, y, z + 0.0016 * math.cos(twist)))
    return pts


def _stake_path(theta: float, stake_index: int) -> list[tuple[float, float, float]]:
    pts: list[tuple[float, float, float]] = []
    for i in range(28):
        t = i / 27.0
        z = 0.006 + (BODY_H - 0.010) * t
        a, b = _axes_at_z(z)
        flutter = 0.0008 * math.sin(2.0 * math.pi * BODY_ROWS * t + stake_index * 0.37)
        x, y = _super_point(theta, a + 0.0020 + flutter, b + 0.0014 + flutter * 0.6)
        pts.append((x, y, z))
    return pts


def _upper_edge_support_path(theta: float, support_index: int) -> list[tuple[float, float, float]]:
    pts: list[tuple[float, float, float]] = []
    z0 = BODY_H - 0.034
    z1 = BODY_H + 0.001
    for i in range(10):
        t = i / 9.0
        z = z0 + (z1 - z0) * t
        a, b = _axes_at_z(z)
        sway = 0.0018 * math.sin(math.pi * t + support_index * 0.41)
        x, y = _super_point(theta + sway, a + 0.0035, b + 0.0026)
        pts.append((x, y, z))
    return pts


def _upper_edge_lashing_path(theta: float, support_index: int, direction: float) -> list[tuple[float, float, float]]:
    pts: list[tuple[float, float, float]] = []
    z0 = BODY_H - 0.032
    z1 = BODY_H - 0.004
    for i in range(9):
        t = i / 8.0
        z = z0 + (z1 - z0) * t
        a, b = _axes_at_z(z)
        theta_t = theta + direction * 0.050 * t
        x, y = _super_point(theta_t, a + 0.0042, b + 0.0030)
        flutter = 0.0005 * math.sin(support_index * 0.53 + i)
        pts.append((x, y, z + flutter))
    return pts


def _extent_for_offset(offset: float, a: float, b: float, n: float, orientation: str) -> float:
    if orientation == "x":
        ratio = abs(offset / b)
        return a * max(0.0, 1.0 - ratio**n) ** (1.0 / n)
    ratio = abs(offset / a)
    return b * max(0.0, 1.0 - ratio**n) ** (1.0 / n)


def _lid_strip_points(
    offset: float,
    *,
    orientation: str,
    z_offset: float,
    phase: float,
    samples: int = 18,
) -> list[tuple[float, float, float]]:
    extent = _extent_for_offset(offset, LID_A, LID_B, SUPER_N, orientation)
    pts: list[tuple[float, float, float]] = []
    for i in range(samples):
        u = -extent + 2.0 * extent * i / (samples - 1)
        if orientation == "x":
            x, y = u, offset
        else:
            x, y = offset, u
        metric = min(1.0, abs(x / LID_A) ** SUPER_N + abs(y / LID_B) ** SUPER_N)
        dome_z = LID_DOME * (1.0 - metric)
        over_under = 0.0009 * math.sin(i * 1.7 + phase)
        pts.append((x, y, LID_BASE_Z + z_offset + dome_z + over_under))
    return pts


def _floor_strip_points(
    offset: float,
    *,
    a: float,
    b: float,
    orientation: str,
    z: float,
    phase: float,
    samples: int = 14,
) -> list[tuple[float, float, float]]:
    extent = _extent_for_offset(offset, a, b, SUPER_N, orientation)
    pts: list[tuple[float, float, float]] = []
    for i in range(samples):
        u = -extent + 2.0 * extent * i / (samples - 1)
        if orientation == "x":
            x, y = u, offset
        else:
            x, y = offset, u
        over_under = 0.0006 * math.sin(i * 1.4 + phase)
        pts.append((x, y, z + over_under))
    return pts


def _handle_riser(theta: float) -> list[tuple[float, float, float]]:
    x0, y0 = _super_point(theta, HANDLE_A, HANDLE_B, n=2.65)
    x1, y1 = _super_point(theta, HANDLE_A * 0.92, HANDLE_B * 0.92, n=2.65)
    return [(x0, y0, HANDLE_BASE_Z - 0.001), (x1, y1, HANDLE_TOP_Z)]


_TUBE_MESH_COUNTER = 0


def tube(points, **kwargs):
    global _TUBE_MESH_COUNTER
    _TUBE_MESH_COUNTER += 1
    return mesh_from_geometry(
        _sdk_tube_from_spline_points(points, **kwargs),
        f"oval_basket_tube_{_TUBE_MESH_COUNTER:03d}",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="oval_woven_rattan_basket_with_lift_off_lid")

    model.material("rattan_light", rgba=(0.86, 0.57, 0.25, 1.0))
    model.material("rattan_mid", rgba=(0.70, 0.42, 0.16, 1.0))
    model.material("rattan_dark", rgba=(0.43, 0.24, 0.09, 1.0))
    model.material("rattan_lid_light", rgba=(0.83, 0.54, 0.23, 1.0))
    model.material("rattan_lid_shadow", rgba=(0.55, 0.31, 0.12, 1.0))

    body = model.part("basket_body")

    a0, b0 = _axes_at_z(0.006)
    body.visual(
        tube(
            _closed_super_path(a0, b0, 0.006),
            radius=T_RIM,
            closed_spline=True,
            samples_per_segment=1,
            radial_segments=9,
            cap_ends=False,
        ),
        material="rattan_mid",
        name="oval_bottom_braided_foot",
    )

    for i, off in enumerate(-0.064 + j * 0.016 for j in range(9)):
        body.visual(
            tube(
                _floor_strip_points(
                    off,
                    a=a0 * 0.985,
                    b=b0 * 0.985,
                    orientation="x",
                    z=0.0090,
                    phase=i * 0.7,
                ),
                radius=0.0032,
                samples_per_segment=1,
                radial_segments=7,
                cap_ends=True,
            ),
            material="rattan_light" if i % 2 == 0 else "rattan_mid",
            name=f"bottom_bamboo_strip_{i:02d}",
        )

    for i, off in enumerate((-0.086, -0.043, 0.0, 0.043, 0.086)):
        body.visual(
            tube(
                _floor_strip_points(
                    off,
                    a=a0 * 0.985,
                    b=b0 * 0.985,
                    orientation="y",
                    z=0.0125,
                    phase=i * 0.8 + math.pi,
                ),
                radius=0.0022,
                samples_per_segment=1,
                radial_segments=6,
                cap_ends=True,
            ),
            material="rattan_mid" if i % 2 == 0 else "rattan_light",
            name=f"bottom_retaining_strip_{i:02d}",
        )

    for j in range(BODY_STAKES):
        theta = 2.0 * math.pi * j / BODY_STAKES
        body.visual(
            tube(
                _stake_path(theta, j),
                radius=T_STAKE,
                samples_per_segment=2,
                radial_segments=6,
                cap_ends=True,
                up_hint=(1.0, 0.0, 0.0),
            ),
            material="rattan_dark" if j % 5 == 0 else "rattan_mid",
            name=f"vertical_side_stake_{j:02d}",
        )

    for i in range(BODY_ROWS):
        t = i / (BODY_ROWS - 1)
        z = 0.020 + (BODY_H - 0.030) * t
        a, b = _axes_at_z(z)
        body.visual(
            tube(
                _closed_super_path(
                    a,
                    b,
                    z,
                    weave_count=BODY_STAKES,
                    phase=(i % 2) * math.pi,
                    amp=0.0022,
                    z_amp=0.0007,
                ),
                radius=T_SIDE,
                closed_spline=True,
                samples_per_segment=1,
                radial_segments=7,
                cap_ends=False,
            ),
            material="rattan_light" if i % 2 == 0 else "rattan_mid",
            name=f"horizontal_side_weave_{i:02d}",
        )

    # Short woven stakes, lashings, and extra rows carry the side weave up to the rim.
    for j in range(BODY_STAKES):
        theta = 2.0 * math.pi * j / BODY_STAKES
        body.visual(
            tube(
                _upper_edge_support_path(theta, j),
                radius=T_STAKE * 1.22,
                samples_per_segment=2,
                radial_segments=7,
                cap_ends=True,
                up_hint=(1.0, 0.0, 0.0),
            ),
            material="rattan_dark" if j % 4 == 0 else "rattan_mid",
            name=f"upper_edge_support_stake_{j:02d}",
        )

    for j in range(0, BODY_STAKES, 4):
        theta = 2.0 * math.pi * j / BODY_STAKES
        for direction in (-1.0, 1.0):
            body.visual(
                tube(
                    _upper_edge_lashing_path(theta, j, direction),
                    radius=T_STAKE * 0.82,
                    samples_per_segment=2,
                    radial_segments=6,
                    cap_ends=True,
                    up_hint=(1.0, 0.0, 0.0),
                ),
                material="rattan_light" if direction > 0 else "rattan_dark",
                name=f"upper_edge_diagonal_lashing_{j:02d}_{int(direction > 0)}",
            )

    for i, z in enumerate((BODY_H - 0.028, BODY_H - 0.022, BODY_H - 0.016, BODY_H - 0.010, BODY_H - 0.004)):
        a, b = _axes_at_z(z)
        body.visual(
            tube(
                _closed_super_path(
                    a + 0.0026,
                    b + 0.0019,
                    z,
                    weave_count=BODY_STAKES,
                    phase=(i + 1) * math.pi * 0.5,
                    amp=0.0022,
                    z_amp=0.0006,
                ),
                radius=T_SIDE * 1.02,
                closed_spline=True,
                samples_per_segment=1,
                radial_segments=8,
                cap_ends=False,
            ),
            material="rattan_light" if i % 2 == 0 else "rattan_mid",
            name=f"upper_edge_support_weave_{i:02d}",
        )

    # Strong horizontal seam band just below the lid, visible in the photo.
    for band_idx, z in enumerate((0.100, 0.108)):
        a, b = _axes_at_z(z)
        body.visual(
            tube(
                _closed_super_path(a + 0.001, b + 0.001, z),
                radius=0.0031,
                closed_spline=True,
                samples_per_segment=1,
                radial_segments=8,
                cap_ends=False,
            ),
            material="rattan_dark" if band_idx == 0 else "rattan_light",
            name=f"side_lid_seam_band_{band_idx}",
        )

    a_top, b_top = _axes_at_z(BODY_H)
    for strand, phase in enumerate((0.0, math.pi)):
        body.visual(
            tube(
                _braid_path(a_top + 0.004, b_top + 0.003, BODY_H, phase=phase, turns=34),
                radius=T_RIM,
                closed_spline=True,
                samples_per_segment=1,
                radial_segments=9,
                cap_ends=False,
            ),
            material="rattan_light" if strand == 0 else "rattan_mid",
            name=f"body_top_braided_rim_{strand}",
        )

    lid = model.part("basket_lid")

    lid.visual(
        mesh_from_geometry(
            ExtrudeGeometry.from_z0(
                superellipse_profile(LID_A * 1.92, LID_B * 1.90, exponent=SUPER_N, segments=96),
                0.0020,
                cap=True,
            ).translate(0.0, 0.0, LID_BASE_Z + 0.0003),
            "dense_lid_woven_underlay",
        ),
        material="rattan_lid_shadow",
        name="dense_lid_woven_underlay",
    )

    strip_offsets_y = [-0.106 + i * 0.0101 for i in range(22)]
    for i, off in enumerate(strip_offsets_y):
        lid.visual(
            tube(
                _lid_strip_points(off, orientation="x", z_offset=0.000, phase=i * 0.7),
                radius=T_LID,
                samples_per_segment=1,
                radial_segments=6,
                cap_ends=True,
            ),
            material="rattan_lid_light" if i % 2 == 0 else "rattan_lid_shadow",
            name=f"lid_lengthwise_weave_{i:02d}",
        )

    strip_offsets_x = [-0.164 + i * 0.0149 for i in range(23)]
    for i, off in enumerate(strip_offsets_x):
        lid.visual(
            tube(
                _lid_strip_points(off, orientation="y", z_offset=0.002, phase=i * 0.9 + math.pi),
                radius=T_LID * 0.92,
                samples_per_segment=1,
                radial_segments=6,
                cap_ends=True,
            ),
            material="rattan_lid_shadow" if i % 2 == 0 else "rattan_lid_light",
            name=f"lid_cross_weave_{i:02d}",
        )

    for strand, phase in enumerate((0.0, math.pi)):
        lid.visual(
            tube(
                _braid_path(LID_A, LID_B, LID_BASE_Z + 0.001, phase=phase, turns=36),
                radius=0.0055,
                closed_spline=True,
                samples_per_segment=1,
                radial_segments=9,
                cap_ends=False,
            ),
            material="rattan_lid_light" if strand == 0 else "rattan_lid_shadow",
            name=f"lid_outer_braided_rim_{strand}",
        )

    # Low raised woven oval handle/knob at the center of the lid.
    for strand, phase in enumerate((0.0, math.pi)):
        lid.visual(
            tube(
                _braid_path(HANDLE_A, HANDLE_B, HANDLE_BASE_Z, phase=phase, turns=12),
                radius=T_HANDLE,
                closed_spline=True,
                samples_per_segment=1,
                radial_segments=8,
                cap_ends=False,
            ),
            material="rattan_lid_shadow" if strand == 0 else "rattan_lid_light",
            name=f"raised_handle_base_braid_{strand}",
        )
        lid.visual(
            tube(
                _braid_path(HANDLE_A * 0.92, HANDLE_B * 0.92, HANDLE_TOP_Z, phase=phase, turns=12),
                radius=T_HANDLE,
                closed_spline=True,
                samples_per_segment=1,
                radial_segments=8,
                cap_ends=False,
            ),
            material="rattan_lid_light" if strand == 0 else "rattan_lid_shadow",
            name=f"raised_handle_top_braid_{strand}",
        )

    for i in range(12):
        theta = 2.0 * math.pi * i / 12
        lid.visual(
            tube(
                _handle_riser(theta),
                radius=0.0024,
                samples_per_segment=1,
                radial_segments=6,
                cap_ends=True,
                up_hint=(1.0, 0.0, 0.0),
            ),
            material="rattan_lid_shadow" if i % 3 == 0 else "rattan_lid_light",
            name=f"raised_handle_vertical_weave_{i:02d}",
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, BODY_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.35, lower=0.0, upper=LID_LIFT),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("basket_body")
    lid = object_model.get_part("basket_lid")
    joint = object_model.get_articulation("body_to_lid")

    mats = {
        (v.material.name if hasattr(v.material, "name") else v.material)
        for p in (body, lid)
        for v in p.visuals
    }
    ctx.check(
        "all_visible_materials_are_rattan",
        all(isinstance(m, str) and "rattan" in m for m in mats),
        f"materials={sorted(mats)}",
    )

    side_rows = sum(1 for v in body.visuals if (v.name or "").startswith("horizontal_side_weave"))
    side_stakes = sum(1 for v in body.visuals if (v.name or "").startswith("vertical_side_stake"))
    upper_supports = sum(1 for v in body.visuals if (v.name or "").startswith("upper_edge_support"))
    ctx.check(
        "dense_oval_body_weave",
        side_rows >= BODY_ROWS and side_stakes >= BODY_STAKES and upper_supports >= 25,
        f"rows={side_rows}, stakes={side_stakes}, upper_supports={upper_supports}",
    )

    bottom_weave = sum(
        1
        for v in body.visuals
        if (v.name or "").startswith(("bottom_bamboo_strip", "bottom_retaining_strip"))
    )
    ctx.check(
        "bamboo_strip_bottom_present",
        bottom_weave >= 12,
        f"bottom_weave={bottom_weave}",
    )

    lid_length = sum(1 for v in lid.visuals if (v.name or "").startswith("lid_lengthwise_weave"))
    lid_cross = sum(1 for v in lid.visuals if (v.name or "").startswith("lid_cross_weave"))
    handle_parts = sum(1 for v in lid.visuals if "raised_handle" in (v.name or ""))
    ctx.check(
        "woven_lid_with_raised_handle",
        lid_length >= 20 and lid_cross >= 20 and handle_parts >= 16,
        f"lid_length={lid_length}, lid_cross={lid_cross}, handle_parts={handle_parts}",
    )

    ctx.check(
        "lift_off_lid_joint",
        joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(joint.axis) == (0.0, 0.0, 1.0),
        f"type={joint.articulation_type}, axis={joint.axis}",
    )

    ctx.allow_overlap(
        lid,
        body,
        reason="The fitted lift-off lid's braided rim rests over the basket mouth when closed.",
    )
    ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.18, name="oval_lid_covers_body")

    with ctx.pose({joint: LID_LIFT}):
        ctx.expect_gap(lid, body, axis="z", min_gap=0.0, name="oval_lid_lifts_clear")

    return ctx.report()


object_model = build_object_model()
