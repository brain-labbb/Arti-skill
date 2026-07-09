from __future__ import annotations

# Cylindrical woven rattan basket with a fitted lift-off lid and black ring
# handle, matching picture/Container/Basket/005.png.
#
# The reference is a straight-sided round hamper: a vertical cylindrical woven
# wall, dense rectangular side openings, a flat woven lid, and a small black
# upright ring handle centered on the lid. The lid is the articulated part and
# lifts off vertically.

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

BODY_H = 0.300
R_BODY = 0.124
R_LID = 0.132
R_LID_WEAVE = 0.124
LID_LIFT = 0.155

RING_SAMPLES = 240
BODY_ROWS = 27
BODY_STAKES = 48
LID_STRIPS = 23
BOTTOM_STRIPS = 23

T_SIDE = 0.0038
T_STAKE = 0.0028
T_RIM = 0.0052
T_LID = 0.0028
T_HANDLE = 0.0032

LID_DOME = 0.010
LID_BASE_Z = 0.001
HANDLE_HALF_W = 0.018
HANDLE_HALF_H = 0.015
HANDLE_CENTER_Z = LID_BASE_Z + LID_DOME + 0.018


def _radius_at_z(z: float) -> float:
    # Nearly cylindrical, with a tiny real-basket taper and softly proud middle.
    t = max(0.0, min(1.0, z / BODY_H))
    belly = math.sin(math.pi * t)
    return R_BODY + 0.004 * belly - 0.003 * t


def _ring_path(
    radius: float,
    z: float,
    *,
    samples: int = RING_SAMPLES,
    weave_count: int = 0,
    phase: float = 0.0,
    radial_amp: float = 0.0,
    z_amp: float = 0.0,
) -> list[tuple[float, float, float]]:
    pts: list[tuple[float, float, float]] = []
    for i in range(samples):
        theta = 2.0 * math.pi * i / samples
        wave = math.cos(weave_count * theta + phase) if weave_count else 0.0
        r = radius + radial_amp * wave
        zz = z + (z_amp * math.sin(weave_count * theta + phase) if weave_count else 0.0)
        pts.append((r * math.cos(theta), r * math.sin(theta), zz))
    return pts


def _braid_path(radius: float, z: float, *, phase: float, turns: int) -> list[tuple[float, float, float]]:
    pts: list[tuple[float, float, float]] = []
    for i in range(RING_SAMPLES):
        theta = 2.0 * math.pi * i / RING_SAMPLES
        twist = turns * theta + phase
        r = radius + 0.0023 * math.sin(twist)
        pts.append((r * math.cos(theta), r * math.sin(theta), z + 0.0015 * math.cos(twist)))
    return pts


def _stake_path(theta: float, index: int) -> list[tuple[float, float, float]]:
    pts: list[tuple[float, float, float]] = []
    for i in range(34):
        t = i / 33.0
        z = 0.009 + (BODY_H - 0.004) * t
        flutter = 0.0007 * math.sin(2.0 * math.pi * BODY_ROWS * t + index * 0.41)
        r = _radius_at_z(z) + 0.0012 + flutter
        pts.append((r * math.cos(theta), r * math.sin(theta), z))
    return pts


def _disc_chord_points(
    offset: float,
    *,
    radius: float,
    orientation: str,
    z_offset: float,
    phase: float,
    samples: int = 18,
) -> list[tuple[float, float, float]]:
    half = math.sqrt(max(radius * radius - offset * offset, 0.0))
    pts: list[tuple[float, float, float]] = []
    for i in range(samples):
        u = -half + 2.0 * half * i / (samples - 1)
        if orientation == "x":
            x, y = u, offset
        else:
            x, y = offset, u
        rho2 = x * x + y * y
        dome_z = LID_DOME * max(0.0, 1.0 - rho2 / (radius * radius))
        over_under = 0.0008 * math.sin(i * 1.6 + phase)
        pts.append((x, y, LID_BASE_Z + z_offset + dome_z + over_under))
    return pts


def _bottom_chord_points(
    offset: float,
    *,
    radius: float,
    orientation: str,
    z: float,
    phase: float,
    samples: int = 12,
) -> list[tuple[float, float, float]]:
    half = math.sqrt(max(radius * radius - offset * offset, 0.0))
    pts: list[tuple[float, float, float]] = []
    for i in range(samples):
        u = -half + 2.0 * half * i / (samples - 1)
        if orientation == "x":
            x, y = u, offset
        else:
            x, y = offset, u
        over_under = 0.00045 * math.sin(i * 1.9 + phase)
        pts.append((x, y, z + over_under))
    return pts


def _handle_loop_path(samples: int = 96) -> list[tuple[float, float, float]]:
    pts: list[tuple[float, float, float]] = []
    for i in range(samples):
        theta = 2.0 * math.pi * i / samples
        x = HANDLE_HALF_W * math.cos(theta)
        z = HANDLE_CENTER_Z + HANDLE_HALF_H * math.sin(theta)
        # A barely arced y coordinate makes the black loop visible from oblique
        # views while still reading as an upright ring handle.
        y = 0.003 * math.sin(theta) ** 2
        pts.append((x, y, z))
    return pts


def _handle_post(x: float) -> list[tuple[float, float, float]]:
    return [
        (x * 0.70, 0.000, LID_BASE_Z + LID_DOME - 0.001),
        (x * 0.86, 0.001, HANDLE_CENTER_Z - HANDLE_HALF_H + 0.003),
    ]


_TUBE_COUNTER = 0


def tube(points, **kwargs):
    global _TUBE_COUNTER
    _TUBE_COUNTER += 1
    return mesh_from_geometry(
        _sdk_tube_from_spline_points(points, **kwargs),
        f"cylindrical_basket_tube_{_TUBE_COUNTER:03d}",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cylindrical_woven_rattan_basket_with_lift_off_lid")

    model.material("rattan_light", rgba=(0.78, 0.53, 0.28, 1.0))
    model.material("rattan_mid", rgba=(0.62, 0.39, 0.18, 1.0))
    model.material("rattan_dark", rgba=(0.36, 0.22, 0.10, 1.0))
    model.material("rattan_lid_light", rgba=(0.76, 0.51, 0.25, 1.0))
    model.material("rattan_lid_shadow", rgba=(0.52, 0.31, 0.13, 1.0))
    model.material("black_ring_handle", rgba=(0.015, 0.012, 0.010, 1.0))

    body = model.part("basket_body")

    bottom_offsets = [
        -0.114 + i * (0.228 / (BOTTOM_STRIPS - 1)) for i in range(BOTTOM_STRIPS)
    ]
    for i, off in enumerate(bottom_offsets):
        body.visual(
            tube(
                _bottom_chord_points(
                    off,
                    radius=R_BODY - 0.004,
                    orientation="x",
                    z=0.0055,
                    phase=i * 0.6,
                ),
                radius=0.0032,
                samples_per_segment=1,
                radial_segments=6,
                cap_ends=True,
            ),
            material="rattan_light" if i % 2 == 0 else "rattan_mid",
            name=f"dense_bottom_weave_x_{i:02d}",
        )
        body.visual(
            tube(
                _bottom_chord_points(
                    off,
                    radius=R_BODY - 0.004,
                    orientation="y",
                    z=0.0082,
                    phase=i * 0.8 + math.pi,
                ),
                radius=0.0030,
                samples_per_segment=1,
                radial_segments=6,
                cap_ends=True,
            ),
            material="rattan_mid" if i % 2 == 0 else "rattan_light",
            name=f"dense_bottom_weave_y_{i:02d}",
        )

    for k, r in enumerate((0.022, 0.040, 0.058, 0.076, 0.094, 0.110, R_BODY - 0.006)):
        body.visual(
            tube(
                _ring_path(r, 0.0105 + 0.0004 * (k % 2)),
                radius=0.0028,
                closed_spline=True,
                samples_per_segment=1,
                radial_segments=6,
                cap_ends=False,
            ),
            material="rattan_dark" if k % 2 else "rattan_light",
            name=f"dense_bottom_concentric_weave_{k}",
        )

    for strand, phase in enumerate((0.0, math.pi)):
        body.visual(
            tube(
                _braid_path(_radius_at_z(0.006), 0.006, phase=phase, turns=36),
                radius=T_RIM,
                closed_spline=True,
                samples_per_segment=1,
                radial_segments=9,
                cap_ends=False,
            ),
            material="rattan_mid" if strand == 0 else "rattan_dark",
            name=f"bottom_braided_foot_{strand}",
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
            material="rattan_dark" if j % 6 == 0 else "rattan_mid",
            name=f"vertical_rattan_stake_{j:02d}",
        )

    for i in range(BODY_ROWS):
        t = i / (BODY_ROWS - 1)
        z = 0.023 + (BODY_H - 0.045) * t
        body.visual(
            tube(
                _ring_path(
                    _radius_at_z(z),
                    z,
                    weave_count=BODY_STAKES,
                    phase=(i % 2) * math.pi,
                    radial_amp=0.0021,
                    z_amp=0.0007,
                ),
                radius=T_SIDE,
                closed_spline=True,
                samples_per_segment=1,
                radial_segments=7,
                cap_ends=False,
            ),
            material="rattan_light" if i % 2 == 0 else "rattan_mid",
            name=f"horizontal_rattan_weave_{i:02d}",
        )

    # Narrow darker seam just below the lid, matching the horizontal shadow band
    # visible near the top of the reference.
    for k, z in enumerate((BODY_H - 0.030, BODY_H - 0.021)):
        body.visual(
            tube(
                _ring_path(_radius_at_z(z) + 0.001, z),
                radius=0.0030,
                closed_spline=True,
                samples_per_segment=1,
                radial_segments=7,
                cap_ends=False,
            ),
            material="rattan_dark" if k == 0 else "rattan_light",
            name=f"upper_dark_lid_seam_{k}",
        )

    for strand, phase in enumerate((0.0, math.pi)):
        body.visual(
            tube(
                _braid_path(_radius_at_z(BODY_H) + 0.004, BODY_H, phase=phase, turns=38),
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

    offsets = [-0.108 + i * (0.216 / (LID_STRIPS - 1)) for i in range(LID_STRIPS)]
    for i, off in enumerate(offsets):
        lid.visual(
            tube(
                _disc_chord_points(
                    off,
                    radius=R_LID_WEAVE,
                    orientation="x",
                    z_offset=0.000,
                    phase=i * 0.7,
                ),
                radius=T_LID,
                samples_per_segment=1,
                radial_segments=6,
                cap_ends=True,
            ),
            material="rattan_lid_light" if i % 2 == 0 else "rattan_lid_shadow",
            name=f"lid_lengthwise_flat_weave_{i:02d}",
        )
        lid.visual(
            tube(
                _disc_chord_points(
                    off,
                    radius=R_LID_WEAVE,
                    orientation="y",
                    z_offset=0.0018,
                    phase=i * 0.9 + math.pi,
                ),
                radius=T_LID * 0.92,
                samples_per_segment=1,
                radial_segments=6,
                cap_ends=True,
            ),
            material="rattan_lid_shadow" if i % 2 == 0 else "rattan_lid_light",
            name=f"lid_cross_flat_weave_{i:02d}",
        )

    # Fine concentric strands help the lid read as a woven round mat rather than
    # only a square grid.
    for k, r in enumerate((0.036, 0.060, 0.084, 0.108)):
        lid.visual(
            tube(
                _ring_path(r, LID_BASE_Z + 0.002 + LID_DOME * (1.0 - (r / R_LID_WEAVE) ** 2)),
                radius=0.0019,
                closed_spline=True,
                samples_per_segment=1,
                radial_segments=6,
                cap_ends=False,
            ),
            material="rattan_lid_shadow" if k % 2 else "rattan_lid_light",
            name=f"lid_concentric_woven_strand_{k}",
        )

    for strand, phase in enumerate((0.0, math.pi)):
        lid.visual(
            tube(
                _braid_path(R_LID, LID_BASE_Z + 0.001, phase=phase, turns=38),
                radius=0.0055,
                closed_spline=True,
                samples_per_segment=1,
                radial_segments=9,
                cap_ends=False,
            ),
            material="rattan_lid_light" if strand == 0 else "rattan_lid_shadow",
            name=f"lid_braided_outer_rim_{strand}",
        )

    lid.visual(
        tube(
            _handle_loop_path(),
            radius=T_HANDLE,
            closed_spline=True,
            samples_per_segment=1,
            radial_segments=8,
            cap_ends=False,
            up_hint=(0.0, 1.0, 0.0),
        ),
        material="black_ring_handle",
        name="black_ring_handle_loop",
    )
    for idx, x in enumerate((-HANDLE_HALF_W * 0.55, HANDLE_HALF_W * 0.55)):
        lid.visual(
            tube(
                _handle_post(x),
                radius=0.0026,
                samples_per_segment=1,
                radial_segments=7,
                cap_ends=True,
                up_hint=(0.0, 1.0, 0.0),
            ),
            material="black_ring_handle",
            name=f"black_ring_handle_mount_{idx}",
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
        "rattan_and_black_handle_materials",
        all(isinstance(m, str) and ("rattan" in m or m == "black_ring_handle") for m in mats),
        f"materials={sorted(mats)}",
    )

    rows = sum(1 for v in body.visuals if (v.name or "").startswith("horizontal_rattan_weave"))
    stakes = sum(1 for v in body.visuals if (v.name or "").startswith("vertical_rattan_stake"))
    bottom_floor = sum(1 for v in body.visuals if (v.name or "").startswith("dense_bottom"))
    ctx.check(
        "straight_cylindrical_side_weave",
        rows >= BODY_ROWS and stakes >= BODY_STAKES and bottom_floor >= (BOTTOM_STRIPS * 2 + 6),
        f"rows={rows}, stakes={stakes}, bottom_floor={bottom_floor}",
    )

    lid_length = sum(1 for v in lid.visuals if (v.name or "").startswith("lid_lengthwise"))
    lid_cross = sum(1 for v in lid.visuals if (v.name or "").startswith("lid_cross"))
    black_handle = sum(1 for v in lid.visuals if "black_ring_handle" in (v.name or ""))
    ctx.check(
        "flat_woven_lid_with_black_ring_handle",
        lid_length >= LID_STRIPS and lid_cross >= LID_STRIPS and black_handle >= 3,
        f"lid_length={lid_length}, lid_cross={lid_cross}, black_handle={black_handle}",
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
        reason="The fitted lift-off lid's woven outer rim rests over the basket mouth when closed.",
    )
    ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.20, name="round_lid_covers_cylindrical_body")

    with ctx.pose({joint: LID_LIFT}):
        ctx.expect_gap(lid, body, axis="z", min_gap=0.0, name="round_lid_lifts_clear")

    return ctx.report()


object_model = build_object_model()
