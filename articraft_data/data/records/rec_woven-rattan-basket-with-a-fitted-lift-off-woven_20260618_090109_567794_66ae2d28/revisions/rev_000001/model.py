from __future__ import annotations

# Woven rattan basket with a fitted lift-off woven lid.
#
# The reference image is a bulbous round rattan storage basket, not a simple
# wire basket and not a coil-only pot. The body is modeled as interlaced cane:
# horizontal bands weave in and out around vertical stakes, leaving visible
# gaps. The lift-off lid is a shallow woven disc with a braided rim and no
# visible underside locating ring.

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

# ---------------------------------------------------------------------------
# Absolute dimensions (meters). The basket is roughly small-floor-basket scale.
# ---------------------------------------------------------------------------

H_BODY = 0.300
Z_FOOT = 0.006
Z_BASE = 0.017
R_MOUTH = 0.108

HORIZONTAL_ROWS = 23
VERTICAL_STAKES = 40
RING_SAMPLES = 240

T_HORIZ = 0.0048
T_VERTICAL = 0.0034
T_RIM = 0.0055
T_LID = 0.0030
WEAVE_WAVE = 0.0024

# Outer silhouette control points (height z, centerline radius). This makes the
# reference-like rounded belly and smaller open mouth.
_BODY_PROFILE = [
    (0.000, 0.076),
    (0.020, 0.101),
    (0.050, 0.132),
    (0.085, 0.153),
    (0.125, 0.164),
    (0.165, 0.162),
    (0.205, 0.151),
    (0.240, 0.135),
    (0.275, 0.119),
    (0.300, R_MOUTH),
]

# Lid dimensions. The lid local frame is attached at the mouth plane; the
# underside stays visually open so no thick ring shows below the cover.
R_BRIM = 0.130
R_LID_WEAVE = 0.122
LID_DOME = 0.021
LID_SEAT_Z = H_BODY
LID_LIFT = 0.160


def _base_radius(z: float) -> float:
    pts = _BODY_PROFILE
    if z <= pts[0][0]:
        return pts[0][1]
    if z >= pts[-1][0]:
        return pts[-1][1]
    for (z0, r0), (z1, r1) in zip(pts, pts[1:]):
        if z0 <= z <= z1:
            t = (z - z0) / (z1 - z0)
            return r0 + t * (r1 - r0)
    return pts[-1][1]


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
    points: list[tuple[float, float, float]] = []
    for i in range(samples):
        theta = 2.0 * math.pi * i / samples
        wave = math.cos(weave_count * theta + phase) if weave_count else 0.0
        r = radius + radial_amp * wave
        zz = z + z_amp * math.sin(weave_count * theta + phase) if weave_count else z
        points.append((r * math.cos(theta), r * math.sin(theta), zz))
    return points


def _braid_path(
    radius: float,
    z: float,
    *,
    phase: float,
    turns: int,
    amp: float,
    samples: int = RING_SAMPLES,
) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for i in range(samples):
        theta = 2.0 * math.pi * i / samples
        twist = turns * theta + phase
        r = radius + amp * math.sin(twist)
        zz = z + 0.65 * amp * math.cos(twist)
        points.append((r * math.cos(theta), r * math.sin(theta), zz))
    return points


def _upright_path(theta: float, index: int) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    samples = 34
    for i in range(samples):
        t = i / (samples - 1)
        z = Z_FOOT + (H_BODY - Z_FOOT) * t
        # A tiny radial shim keeps the upright stakes visible between rows and
        # gives the over-under weave a real relief instead of a flat drawing.
        flutter = 0.0010 * math.sin(2.0 * math.pi * HORIZONTAL_ROWS * t + index * 0.55)
        r = _base_radius(z) + 0.0012 + flutter
        points.append((r * math.cos(theta), r * math.sin(theta), z))
    return points


def _disc_chord_points(
    offset: float,
    *,
    radius: float,
    orientation: str,
    z_base: float,
    dome: float,
    phase: float,
    samples: int = 18,
) -> list[tuple[float, float, float]]:
    half = math.sqrt(max(radius * radius - offset * offset, 0.0))
    points: list[tuple[float, float, float]] = []
    for i in range(samples):
        u = -half + 2.0 * half * i / (samples - 1)
        if orientation == "x":
            x, y = u, offset
        else:
            x, y = offset, u
        rho2 = x * x + y * y
        dome_z = dome * max(0.0, 1.0 - rho2 / (radius * radius))
        over_under = 0.0010 * math.cos((i / (samples - 1)) * math.pi * 8.0 + phase)
        points.append((x, y, z_base + dome_z + over_under))
    return points


_TUBE_MESH_COUNTER = 0


def tube_from_spline_points(points, **kwargs):
    global _TUBE_MESH_COUNTER
    _TUBE_MESH_COUNTER += 1
    geom = _sdk_tube_from_spline_points(points, **kwargs)
    return mesh_from_geometry(geom, f"woven_rattan_tube_{_TUBE_MESH_COUNTER:03d}")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="woven_rattan_basket_with_lift_off_lid")

    model.material("rattan_light", rgba=(0.91, 0.72, 0.40, 1.0))
    model.material("rattan_mid", rgba=(0.80, 0.58, 0.28, 1.0))
    model.material("rattan_shadow", rgba=(0.56, 0.35, 0.14, 1.0))
    model.material("rattan_lid_light", rgba=(0.88, 0.67, 0.35, 1.0))
    model.material("rattan_lid_shadow", rgba=(0.68, 0.45, 0.20, 1.0))

    body = model.part("basket_body")

    # Bottom foot and woven floor. The floor is a low cross weave so the basket
    # still reads as hollow through the open mouth.
    body.visual(
        tube_from_spline_points(
            _ring_path(_base_radius(Z_FOOT), Z_FOOT),
            radius=T_RIM,
            closed_spline=True,
            samples_per_segment=1,
            radial_segments=10,
            cap_ends=False,
        ),
        material="rattan_mid",
        name="bottom_braided_foot_ring",
    )
    floor_offsets = [(-0.060 + i * 0.020) for i in range(7)]
    for i, off in enumerate(floor_offsets):
        body.visual(
            tube_from_spline_points(
                _disc_chord_points(
                    off,
                    radius=0.081,
                    orientation="x",
                    z_base=Z_BASE - 0.004,
                    dome=0.0,
                    phase=i * math.pi,
                    samples=8,
                ),
                radius=0.0028,
                samples_per_segment=1,
                radial_segments=6,
                cap_ends=True,
            ),
            material="rattan_light" if i % 2 == 0 else "rattan_mid",
            name=f"floor_weave_x_{i:02d}",
        )
        body.visual(
            tube_from_spline_points(
                _disc_chord_points(
                    off,
                    radius=0.081,
                    orientation="y",
                    z_base=Z_BASE - 0.002,
                    dome=0.0,
                    phase=i * math.pi + math.pi,
                    samples=8,
                ),
                radius=0.0028,
                samples_per_segment=1,
                radial_segments=6,
                cap_ends=True,
            ),
            material="rattan_mid" if i % 2 == 0 else "rattan_light",
            name=f"floor_weave_y_{i:02d}",
        )
    body.visual(
        Cylinder(radius=0.012, length=0.007),
        origin=Origin(xyz=(0.0, 0.0, Z_BASE - 0.004)),
        material="rattan_shadow",
        name="dark_floor_center_shadow",
    )

    # Vertical stakes first, then horizontal rows weave around them. The wave on
    # each row flips phase, creating the alternating over-under basket texture.
    for j in range(VERTICAL_STAKES):
        theta = 2.0 * math.pi * j / VERTICAL_STAKES
        body.visual(
            tube_from_spline_points(
                _upright_path(theta, j),
                radius=T_VERTICAL,
                samples_per_segment=2,
                radial_segments=7,
                cap_ends=True,
                up_hint=(1.0, 0.0, 0.0),
            ),
            material="rattan_shadow" if j % 4 == 0 else "rattan_mid",
            name=f"vertical_weave_stake_{j:02d}",
        )

    z0 = 0.030
    z1 = H_BODY - 0.026
    for i in range(HORIZONTAL_ROWS):
        t = i / (HORIZONTAL_ROWS - 1)
        z = z0 + (z1 - z0) * t
        r = _base_radius(z)
        phase = (i % 2) * math.pi
        body.visual(
            tube_from_spline_points(
                _ring_path(
                    r,
                    z,
                    weave_count=VERTICAL_STAKES,
                    phase=phase,
                    radial_amp=WEAVE_WAVE,
                    z_amp=0.0008,
                ),
                radius=T_HORIZ,
                closed_spline=True,
                samples_per_segment=1,
                radial_segments=8,
                cap_ends=False,
            ),
            material="rattan_light" if i % 2 == 0 else "rattan_mid",
            name=f"horizontal_weave_band_{i:02d}",
        )

    # Thick braided body rim around the smaller mouth, like the rope-like lip in
    # the reference. Two strands with opposite phase read as a twist.
    for strand, phase in enumerate((0.0, math.pi)):
        body.visual(
            tube_from_spline_points(
                _braid_path(R_MOUTH + 0.005, H_BODY, phase=phase, turns=28, amp=0.0024),
                radius=T_RIM,
                closed_spline=True,
                samples_per_segment=1,
                radial_segments=10,
                cap_ends=False,
            ),
            material="rattan_light" if strand == 0 else "rattan_mid",
            name=f"body_braided_mouth_rim_{strand}",
        )

    lid = model.part("basket_lid")

    # Woven lid face: two crossed families of cane strips clipped to a circular
    # panel and gently domed. This replaces the old concentric-only lid.
    lid_offsets = [(-0.104 + i * 0.0138) for i in range(16)]
    for i, off in enumerate(lid_offsets):
        lid.visual(
            tube_from_spline_points(
                _disc_chord_points(
                    off,
                    radius=R_LID_WEAVE,
                    orientation="x",
                    z_base=0.006,
                    dome=LID_DOME,
                    phase=i * math.pi,
                ),
                radius=T_LID,
                samples_per_segment=1,
                radial_segments=7,
                cap_ends=True,
            ),
            material="rattan_lid_light" if i % 2 == 0 else "rattan_lid_shadow",
            name=f"lid_weave_x_strip_{i:02d}",
        )
        lid.visual(
            tube_from_spline_points(
                _disc_chord_points(
                    off,
                    radius=R_LID_WEAVE,
                    orientation="y",
                    z_base=0.008,
                    dome=LID_DOME,
                    phase=i * math.pi + math.pi,
                ),
                radius=T_LID,
                samples_per_segment=1,
                radial_segments=7,
                cap_ends=True,
            ),
            material="rattan_lid_shadow" if i % 2 == 0 else "rattan_lid_light",
            name=f"lid_weave_y_strip_{i:02d}",
        )

    for strand, phase in enumerate((0.0, math.pi)):
        lid.visual(
            tube_from_spline_points(
                _braid_path(R_BRIM, 0.005, phase=phase, turns=30, amp=0.0030),
                radius=0.0060,
                closed_spline=True,
                samples_per_segment=1,
                radial_segments=10,
                cap_ends=False,
            ),
            material="rattan_lid_light" if strand == 0 else "rattan_lid_shadow",
            name=f"braided_lid_outer_rim_{strand}",
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, LID_SEAT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.4, lower=0.0, upper=LID_LIFT),
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
        "rattan_materials",
        all(isinstance(m, str) and "rattan" in m for m in mats),
        f"expected rattan materials, got {sorted(mats)}",
    )

    horizontal_count = sum(1 for v in body.visuals if (v.name or "").startswith("horizontal_weave"))
    vertical_count = sum(1 for v in body.visuals if (v.name or "").startswith("vertical_weave"))
    ctx.check(
        "interlaced_body_weave",
        horizontal_count >= HORIZONTAL_ROWS and vertical_count >= VERTICAL_STAKES,
        f"horizontal={horizontal_count}, vertical={vertical_count}",
    )

    lid_x = sum(1 for v in lid.visuals if (v.name or "").startswith("lid_weave_x"))
    lid_y = sum(1 for v in lid.visuals if (v.name or "").startswith("lid_weave_y"))
    lid_braid = sum(1 for v in lid.visuals if "braided_lid_outer_rim" in (v.name or ""))
    ctx.check(
        "woven_lid_face_and_braided_rim",
        lid_x >= 10 and lid_y >= 10 and lid_braid >= 2,
        f"lid_x={lid_x}, lid_y={lid_y}, braided_rim={lid_braid}",
    )

    ctx.check(
        "lift_off_joint_axis",
        joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(joint.axis) == (0.0, 0.0, 1.0),
        f"type={joint.articulation_type}, axis={joint.axis}",
    )

    ctx.allow_overlap(
        lid,
        body,
        reason=(
            "Fitted lift-off lid: the braided outer lid rim rests lightly over "
            "the basket mouth rim at the closed pose."
        ),
    )
    ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.12, name="lid_covers_mouth")

    with ctx.pose({joint: LID_LIFT}):
        ctx.expect_gap(lid, body, axis="z", min_gap=0.0, name="lid_lifts_clear")

    return ctx.report()


object_model = build_object_model()
