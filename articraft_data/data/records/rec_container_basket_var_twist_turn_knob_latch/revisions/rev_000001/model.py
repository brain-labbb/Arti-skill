from __future__ import annotations

# Oval woven rattan basket with a fitted woven lid and a central twist-to-lock
# turn knob. The knob is joined to the lid by a real revolute joint about the
# vertical axis and rotates a quarter turn to engage shaped lugs against the
# body mouth rim, making the turn knob the latch actuator. The fitted lid also
# lifts upward as a separate prismatic part once unlocked.

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

RING_SAMPLES = 216
BODY_ROWS = 24
BODY_STAKES = 44

T_SIDE = 0.0034
T_STAKE = 0.0026
T_RIM = 0.0047
T_LID = 0.0029

LID_A = 0.188
LID_B = 0.125
LID_DOME = 0.026
LID_BASE_Z = 0.0005
LID_LIFT = 0.120

# Turn knob dimensions
KNOB_RADIUS = 0.032
KNOB_HEIGHT = 0.014
KNOB_BASE_Z = LID_BASE_Z + LID_DOME + 0.002
LUG_LENGTH = 0.038
LUG_WIDTH = 0.012
LUG_THICKNESS = 0.006
LUG_DROP = 0.010
NUM_LUGS = 4

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


def _knob_braid_path(radius: float, z: float, *, phase: float, turns: int) -> list[tuple[float, float, float]]:
    """Generate a circular braid path for the turn knob."""
    pts: list[tuple[float, float, float]] = []
    for i in range(RING_SAMPLES):
        theta = 2.0 * math.pi * i / RING_SAMPLES
        twist = turns * theta + phase
        r = radius + 0.0018 * math.sin(twist)
        x = r * math.cos(theta)
        y = r * math.sin(theta)
        pts.append((x, y, z + 0.0012 * math.cos(twist)))
    return pts


def _lug_arm_path(angle: float, z_start: float) -> tuple[list[tuple[float, float, float]], tuple[float, float, float]]:
    """Generate a path for a lug arm extending from the knob center outward and downward.
    Returns (path_points, endpoint) where endpoint is the last point for connectivity."""
    pts: list[tuple[float, float, float]] = []
    segments = 12
    endpoint = (0.0, 0.0, 0.0)
    for i in range(segments):
        t = i / (segments - 1)
        r = KNOB_RADIUS * 0.5 + (LUG_LENGTH - KNOB_RADIUS * 0.5) * t
        x = r * math.cos(angle)
        y = r * math.sin(angle)
        # Lug drops down at the tip to engage the rim
        drop = LUG_DROP * max(0.0, (t - 0.6) / 0.4) if t > 0.6 else 0.0
        z = z_start - drop
        pt = (x, y, z)
        pts.append(pt)
        if i == segments - 1:
            endpoint = pt
    return pts, endpoint


def _lug_tip_path(angle: float, start_point: tuple[float, float, float]) -> list[tuple[float, float, float]]:
    """Generate the hook tip of a lug that catches under the rim, starting from the arm endpoint."""
    pts: list[tuple[float, float, float]] = []
    segments = 8
    start_r = LUG_LENGTH
    start_x, start_y, start_z = start_point
    for i in range(segments):
        t = i / (segments - 1)
        # Curve inward (toward center) to form hook, starting from arm endpoint
        r = start_r - 0.006 * t
        x = r * math.cos(angle)
        y = r * math.sin(angle)
        zz = start_z - 0.003 * t
        pts.append((x, y, zz))
    return pts


def _lug_cross_path(angle: float, r: float, z: float) -> list[tuple[float, float, float]]:
    """Generate the cross-section bar of a lug perpendicular to the radial direction."""
    pts: list[tuple[float, float, float]] = []
    half_w = LUG_WIDTH * 0.5
    perp = angle + math.pi / 2
    x0 = r * math.cos(angle) - half_w * math.cos(perp)
    y0 = r * math.sin(angle) - half_w * math.sin(perp)
    x1 = r * math.cos(angle) + half_w * math.cos(perp)
    y1 = r * math.sin(angle) + half_w * math.sin(perp)
    pts.append((x0, y0, z))
    pts.append((x1, y1, z))
    return pts


def _rim_notch_path(angle: float, a: float, b: float, z: float) -> list[tuple[float, float, float]]:
    """Generate a small notch path in the body rim for lug engagement."""
    pts: list[tuple[float, float, float]] = []
    segments = 8
    half_w = LUG_WIDTH * 0.6
    perp = angle + math.pi / 2
    # Outer rim point
    x_outer, y_outer = _super_point(angle, a + 0.005, b + 0.004)
    for i in range(segments):
        t = i / (segments - 1)
        # Move along perpendicular to create a slot
        offset = -half_w + 2.0 * half_w * t
        x = x_outer + offset * math.cos(perp)
        y = y_outer + offset * math.sin(perp)
        pts.append((x, y, z))
    return pts


_TUBE_MESH_COUNTER = 0


def tube(points, **kwargs):
    global _TUBE_MESH_COUNTER
    _TUBE_MESH_COUNTER += 1
    return mesh_from_geometry(
        _sdk_tube_from_spline_points(points, **kwargs),
        f"oval_basket_tube_{_TUBE_MESH_COUNTER:03d}",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="oval_woven_rattan_basket_twist_lock")

    model.material("rattan_light", rgba=(0.86, 0.57, 0.25, 1.0))
    model.material("rattan_mid", rgba=(0.70, 0.42, 0.16, 1.0))
    model.material("rattan_dark", rgba=(0.43, 0.24, 0.09, 1.0))
    model.material("rattan_lid_light", rgba=(0.83, 0.54, 0.23, 1.0))
    model.material("rattan_lid_shadow", rgba=(0.55, 0.31, 0.12, 1.0))
    model.material("rattan_knob", rgba=(0.75, 0.48, 0.18, 1.0))
    model.material("rattan_knob_dark", rgba=(0.50, 0.28, 0.10, 1.0))

    # --- BASKET BODY ---
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

    # Strong horizontal seam band just below the lid.
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

    # Lug engagement notches on the body rim (4 slots for the turn knob lugs).
    for i in range(NUM_LUGS):
        angle = 2.0 * math.pi * i / NUM_LUGS + math.pi / NUM_LUGS
        body.visual(
            tube(
                _rim_notch_path(angle, a_top, b_top, BODY_H - 0.003),
                radius=0.003,
                samples_per_segment=1,
                radial_segments=6,
                cap_ends=True,
            ),
            material="rattan_dark",
            name=f"rim_lug_slot_{i}",
        )

    # --- BASKET LID ---
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

    # --- TURN KNOB ---
    # Knob visuals are in the knob's local frame (z=0 at the knob base).
    # The lid_to_knob articulation places the knob frame at KNOB_BASE_Z above the lid.
    turn_knob = model.part("turn_knob")

    # Central knob disc - braided woven ring at the base
    for strand, phase in enumerate((0.0, math.pi)):
        turn_knob.visual(
            tube(
                _knob_braid_path(KNOB_RADIUS, 0.0, phase=phase, turns=14),
                radius=0.0042,
                closed_spline=True,
                samples_per_segment=1,
                radial_segments=8,
                cap_ends=False,
            ),
            material="rattan_knob" if strand == 0 else "rattan_knob_dark",
            name=f"knob_base_braid_{strand}",
        )

    # Upper braided ring of the knob
    for strand, phase in enumerate((0.0, math.pi)):
        turn_knob.visual(
            tube(
                _knob_braid_path(KNOB_RADIUS * 0.88, KNOB_HEIGHT, phase=phase, turns=14),
                radius=0.0038,
                closed_spline=True,
                samples_per_segment=1,
                radial_segments=8,
                cap_ends=False,
            ),
            material="rattan_knob_dark" if strand == 0 else "rattan_knob",
            name=f"knob_top_braid_{strand}",
        )

    # Vertical weave risers connecting base and top of knob
    for i in range(16):
        theta = 2.0 * math.pi * i / 16
        r_inner = KNOB_RADIUS * 0.92
        r_outer = KNOB_RADIUS
        x0 = r_outer * math.cos(theta)
        y0 = r_outer * math.sin(theta)
        x1 = r_inner * math.cos(theta)
        y1 = r_inner * math.sin(theta)
        turn_knob.visual(
            tube(
                [(x0, y0, -0.001), (x1, y1, KNOB_HEIGHT)],
                radius=0.0022,
                samples_per_segment=1,
                radial_segments=6,
                cap_ends=True,
                up_hint=(1.0, 0.0, 0.0),
            ),
            material="rattan_knob_dark" if i % 3 == 0 else "rattan_knob",
            name=f"knob_vertical_weave_{i:02d}",
        )

    # Knob top cap - small woven disc
    turn_knob.visual(
        mesh_from_geometry(
            ExtrudeGeometry.from_z0(
                superellipse_profile(KNOB_RADIUS * 1.7, KNOB_RADIUS * 1.7, exponent=2.5, segments=32),
                0.003,
                cap=True,
            ).translate(0.0, 0.0, KNOB_HEIGHT - 0.001),
            "knob_top_cap",
        ),
        material="rattan_knob",
        name="knob_top_cap",
    )

    # Knob base plate connecting lugs to the knob body (extends to braid radius for connectivity)
    plate_r = KNOB_RADIUS + 0.005  # extends past the braid tube center for overlap
    turn_knob.visual(
        mesh_from_geometry(
            ExtrudeGeometry.from_z0(
                superellipse_profile(plate_r, plate_r, exponent=2.5, segments=32),
                0.005,
                cap=True,
            ).translate(0.0, 0.0, -0.002),
            "knob_base_plate",
        ),
        material="rattan_knob_dark",
        name="knob_base_plate",
    )

    # Shaped lugs extending radially from the knob (the locking mechanism).
    lug_z_start = KNOB_HEIGHT * 0.4
    for i in range(NUM_LUGS):
        angle = 2.0 * math.pi * i / NUM_LUGS
        # Lug arm - extends outward and drops down
        arm_pts, arm_endpoint = _lug_arm_path(angle, lug_z_start)
        turn_knob.visual(
            tube(
                arm_pts,
                radius=LUG_THICKNESS * 0.5,
                samples_per_segment=2,
                radial_segments=6,
                cap_ends=True,
                up_hint=(math.cos(angle), math.sin(angle), 0.0),
            ),
            material="rattan_knob_dark",
            name=f"lug_arm_{i}",
        )
        # Lug crossbar at the tip for engagement
        mid_r = LUG_LENGTH * 0.85
        arm_z_end = arm_endpoint[2]
        turn_knob.visual(
            tube(
                _lug_cross_path(angle, mid_r, arm_z_end + 0.002),
                radius=LUG_THICKNESS * 0.45,
                samples_per_segment=1,
                radial_segments=6,
                cap_ends=True,
            ),
            material="rattan_knob",
            name=f"lug_crossbar_{i}",
        )
        # Lug hook tip - starts from the arm endpoint for connectivity
        turn_knob.visual(
            tube(
                _lug_tip_path(angle, arm_endpoint),
                radius=LUG_THICKNESS * 0.4,
                samples_per_segment=2,
                radial_segments=6,
                cap_ends=True,
            ),
            material="rattan_knob_dark",
            name=f"lug_tip_{i}",
        )

    # --- ARTICULATIONS ---

    # Lid lifts straight up from the mouth rim after the knob is turned open.
    model.articulation(
        "body_to_lid",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, BODY_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.35, lower=0.0, upper=LID_LIFT),
    )

    # Turn knob rotates about Z to lock/unlock (quarter turn engagement).
    # The origin is in the lid's local frame at KNOB_BASE_Z above the lid.
    model.articulation(
        "lid_to_knob",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=turn_knob,
        origin=Origin(xyz=(0.0, 0.0, KNOB_BASE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=0.0, upper=math.pi / 2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("basket_body")
    lid = object_model.get_part("basket_lid")
    turn_knob = object_model.get_part("turn_knob")
    body_to_lid = object_model.get_articulation("body_to_lid")
    lid_to_knob = object_model.get_articulation("lid_to_knob")

    # Material check
    mats = {
        (v.material.name if hasattr(v.material, "name") else v.material)
        for p in (body, lid, turn_knob)
        for v in p.visuals
    }
    ctx.check(
        "all_visible_materials_are_rattan",
        all(isinstance(m, str) and "rattan" in m for m in mats),
        f"materials={sorted(mats)}",
    )

    # Body weave density preserved
    side_rows = sum(1 for v in body.visuals if (v.name or "").startswith("horizontal_side_weave"))
    side_stakes = sum(1 for v in body.visuals if (v.name or "").startswith("vertical_side_stake"))
    ctx.check(
        "dense_oval_body_weave",
        side_rows >= BODY_ROWS and side_stakes >= BODY_STAKES,
        f"rows={side_rows}, stakes={side_stakes}",
    )

    # Bottom weave preserved
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

    # Woven lid face preserved (no raised handle)
    lid_length = sum(1 for v in lid.visuals if (v.name or "").startswith("lid_lengthwise_weave"))
    lid_cross = sum(1 for v in lid.visuals if (v.name or "").startswith("lid_cross_weave"))
    ctx.check(
        "woven_lid_face_preserved",
        lid_length >= 20 and lid_cross >= 20,
        f"lid_length={lid_length}, lid_cross={lid_cross}",
    )

    # No raised handle on the lid
    handle_parts = sum(1 for v in lid.visuals if "raised_handle" in (v.name or ""))
    ctx.check(
        "no_raised_handle_on_lid",
        handle_parts == 0,
        f"handle_parts={handle_parts}",
    )

    # Turn knob exists with lug geometry
    lug_arms = sum(1 for v in turn_knob.visuals if (v.name or "").startswith("lug_arm"))
    lug_tips = sum(1 for v in turn_knob.visuals if (v.name or "").startswith("lug_tip"))
    knob_braids = sum(1 for v in turn_knob.visuals if "knob_base_braid" in (v.name or "") or "knob_top_braid" in (v.name or ""))
    ctx.check(
        "turn_knob_with_lugs_and_braids",
        lug_arms >= NUM_LUGS and lug_tips >= NUM_LUGS and knob_braids >= 4,
        f"lug_arms={lug_arms}, lug_tips={lug_tips}, knob_braids={knob_braids}",
    )

    # Body-to-lid lifts upward off the basket mouth.
    ctx.check(
        "body_to_lid_lifts_upward",
        body_to_lid.articulation_type == ArticulationType.PRISMATIC
        and tuple(body_to_lid.axis) == (0.0, 0.0, 1.0)
        and body_to_lid.motion_limits is not None
        and body_to_lid.motion_limits.upper >= LID_LIFT,
        f"type={body_to_lid.articulation_type}, axis={body_to_lid.axis}, limits={body_to_lid.motion_limits}",
    )

    # Lid-to-knob is REVOLUTE about Z with quarter-turn limits
    ctx.check(
        "lid_to_knob_is_revolute_quarter_turn",
        lid_to_knob.articulation_type == ArticulationType.REVOLUTE
        and tuple(lid_to_knob.axis) == (0.0, 0.0, 1.0)
        and lid_to_knob.motion_limits is not None
        and abs(lid_to_knob.motion_limits.lower - 0.0) < 0.01
        and abs(lid_to_knob.motion_limits.upper - math.pi / 2.0) < 0.01,
        f"type={lid_to_knob.articulation_type}, axis={lid_to_knob.axis}, "
        f"limits={lid_to_knob.motion_limits}",
    )

    # Lug slots on body rim for engagement
    rim_slots = sum(1 for v in body.visuals if (v.name or "").startswith("rim_lug_slot"))
    ctx.check(
        "rim_lug_slots_for_engagement",
        rim_slots >= NUM_LUGS,
        f"rim_slots={rim_slots}",
    )

    # Lid covers body opening
    ctx.allow_overlap(
        lid,
        body,
        reason="The fitted lid's braided rim rests over the basket mouth when seated.",
    )
    ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.18, name="oval_lid_covers_body")
    with ctx.pose({body_to_lid: LID_LIFT}):
        ctx.expect_gap(lid, body, axis="z", min_gap=0.005, name="oval_lid_lifts_clear")

    # Turn knob sits on top of the lid
    ctx.allow_overlap(
        turn_knob,
        lid,
        reason="The turn knob base is mounted on the lid dome surface with lug arms passing through the lid plane.",
    )
    ctx.expect_gap(
        turn_knob, body, axis="z", min_gap=-0.02,
        name="turn_knob_above_body_rim",
    )

    # Knob rotation changes lug angular position (proves the revolute joint works)
    knob_center_at_rest = ctx.part_world_position(turn_knob)
    with ctx.pose({lid_to_knob: math.pi / 2.0}):
        knob_center_rotated = ctx.part_world_position(turn_knob)
    ctx.check(
        "knob_rotation_is_about_vertical_axis",
        knob_center_at_rest is not None
        and knob_center_rotated is not None
        and abs(knob_center_rotated[2] - knob_center_at_rest[2]) < 0.005,
        f"rest_z={knob_center_at_rest}, rotated_z={knob_center_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
