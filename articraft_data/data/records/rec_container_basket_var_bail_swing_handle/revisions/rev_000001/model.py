from __future__ import annotations

# Woven rattan basket with a fitted lift-off woven lid and a swinging carry bail.
#
# Fork variant of the round bulbous lift-off basket. The body weave and lid
# geometry are identical to the parent. The single structural change is the
# addition of an arched woven carry bail that spans across the basket mouth,
# mounted to two opposite anchor lugs on the body mouth rim by a single
# horizontal revolute joint. The bail swings up for carrying and folds down
# flat against the body. The lid remains a fitted cover, but it lifts upward
# on a prismatic joint like the other lidded basket variants.

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

# Bail dimensions. The arch spans between two anchor lugs on opposite sides of
# the mouth rim. Half-span is slightly wider than the mouth radius so the pivot
# axis passes through the lug centers. Arch drop is the downward extent of the
# bail when folded flat against the body.
LUG_Y = R_MOUTH + 0.012  # Y position of each lug center
BAIL_HALF_SPAN = LUG_Y
BAIL_ARCH_DROP = 0.155
BAIL_STRAND_R = 0.0040
BAIL_BRAID_R = 0.0032
BAIL_BRAID_AMP = 0.0055
BAIL_BRAID_TURNS = 16
BAIL_UPPER = math.pi  # bail swings to fully upright for carrying

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
LID_LIFT = 0.145


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


# ---------------------------------------------------------------------------
# Bail geometry helpers. The bail is a woven arch in the YZ plane (in bail
# local frame). At q=0 the arch extends downward alongside the body; at
# q=upper it arches upright above the basket for carrying.
# ---------------------------------------------------------------------------


def _bail_arch_path(
    *,
    y_offset: float = 0.0,
    x_offset: float = 0.0,
    height_scale: float = 1.0,
    samples: int = 48,
) -> list[tuple[float, float, float]]:
    """Semicircular arch from one lug to the other in the bail local frame."""
    points: list[tuple[float, float, float]] = []
    for i in range(samples):
        t = i / (samples - 1)
        angle = math.pi * t
        y = BAIL_HALF_SPAN * math.cos(angle) + y_offset
        z = -BAIL_ARCH_DROP * height_scale * math.sin(angle)
        points.append((x_offset, y, z))
    return points


def _bail_braid_path(
    *,
    phase: float = 0.0,
    turns: int = BAIL_BRAID_TURNS,
    amp: float = BAIL_BRAID_AMP,
    samples: int = 240,
) -> list[tuple[float, float, float]]:
    """Helical wrapping around the bail arch for a woven rattan look."""
    points: list[tuple[float, float, float]] = []
    for i in range(samples):
        t = i / (samples - 1)
        angle = math.pi * t
        y = BAIL_HALF_SPAN * math.cos(angle)
        z_base = -BAIL_ARCH_DROP * math.sin(angle)
        twist = turns * 2.0 * math.pi * t + phase
        x = amp * math.cos(twist)
        z = z_base + amp * math.sin(twist)
        points.append((x, y, z))
    return points


def _lug_loop_path(
    y_sign: int,
    *,
    loop_radius: float = 0.009,
    samples: int = 24,
) -> list[tuple[float, float, float]]:
    """Small braided anchor loop protruding outward from the mouth rim."""
    y_center = y_sign * LUG_Y
    points: list[tuple[float, float, float]] = []
    for i in range(samples):
        t = i / (samples - 1)
        angle = 2.0 * math.pi * t
        # Loop extends radially outward (X) and vertically (Z) from the rim
        x = loop_radius * math.cos(angle)
        y = y_center + 0.3 * loop_radius * math.sin(angle)
        z = loop_radius * math.sin(angle)
        points.append((x, y, z))
    return points


_TUBE_MESH_COUNTER = 0


def tube_from_spline_points(points, **kwargs):
    global _TUBE_MESH_COUNTER
    _TUBE_MESH_COUNTER += 1
    geom = _sdk_tube_from_spline_points(points, **kwargs)
    return mesh_from_geometry(geom, f"woven_rattan_tube_{_TUBE_MESH_COUNTER:03d}")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="woven_rattan_basket_with_carry_bail")

    model.material("rattan_light", rgba=(0.91, 0.72, 0.40, 1.0))
    model.material("rattan_mid", rgba=(0.80, 0.58, 0.28, 1.0))
    model.material("rattan_shadow", rgba=(0.56, 0.35, 0.14, 1.0))
    model.material("rattan_lid_light", rgba=(0.88, 0.67, 0.35, 1.0))
    model.material("rattan_lid_shadow", rgba=(0.68, 0.45, 0.20, 1.0))
    model.material("rattan_bail", rgba=(0.76, 0.54, 0.24, 1.0))
    model.material("rattan_bail_wrap", rgba=(0.85, 0.65, 0.32, 1.0))

    # -----------------------------------------------------------------------
    # BODY — identical to parent, plus two anchor lug loops on the mouth rim
    # -----------------------------------------------------------------------
    body = model.part("basket_body")

    # Bottom foot and woven floor
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

    # Vertical stakes
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

    # Horizontal weave rows
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

    # Thick braided body rim
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

    # Anchor lugs — two braided loops on opposite sides of the mouth rim where
    # the bail pivots. These are inline body visuals (no separate parts).
    for i, y_sign in enumerate([1, -1]):
        body.visual(
            tube_from_spline_points(
                _lug_loop_path(y_sign),
                radius=0.0050,
                closed_spline=True,
                samples_per_segment=1,
                radial_segments=8,
                cap_ends=False,
            ),
            origin=Origin(xyz=(0.0, 0.0, H_BODY)),
            material="rattan_shadow",
            name=f"bail_anchor_lug_{i}",
        )

    # -----------------------------------------------------------------------
    # LID — identical woven geometry to parent, now a simple fitted cover
    # -----------------------------------------------------------------------
    lid = model.part("basket_lid")

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

    # -----------------------------------------------------------------------
    # BAIL — woven arched carry handle, swings on a horizontal revolute joint
    # -----------------------------------------------------------------------
    bail = model.part("carry_bail")

    # Three main structural arch strands for a laminated cane look
    for i in range(3):
        x_off = (i - 1) * 0.005
        bail.visual(
            tube_from_spline_points(
                _bail_arch_path(x_offset=x_off, height_scale=1.0 + 0.04 * (i - 1)),
                radius=BAIL_STRAND_R,
                samples_per_segment=2,
                radial_segments=8,
                cap_ends=True,
            ),
            material="rattan_bail" if i != 1 else "rattan_mid",
            name=f"bail_arch_strand_{i}",
        )

    # Two braided wraps around the arch for the woven rattan texture
    for i, phase in enumerate((0.0, math.pi)):
        bail.visual(
            tube_from_spline_points(
                _bail_braid_path(phase=phase),
                radius=BAIL_BRAID_R,
                samples_per_segment=1,
                radial_segments=7,
                cap_ends=True,
            ),
            material="rattan_bail_wrap" if i == 0 else "rattan_light",
            name=f"bail_braided_wrap_{i}",
        )

    # Small end caps where the bail meets the lugs (pivot bearing wraps)
    for i, y_sign in enumerate([1, -1]):
        bail.visual(
            tube_from_spline_points(
                _ring_path(0.007, 0.0, samples=20),
                radius=0.0045,
                closed_spline=True,
                samples_per_segment=1,
                radial_segments=6,
                cap_ends=False,
            ),
            origin=Origin(xyz=(0.0, y_sign * BAIL_HALF_SPAN, 0.0), rpy=(math.pi / 2, 0, 0)),
            material="rattan_shadow",
            name=f"bail_pivot_wrap_{i}",
        )

    # -----------------------------------------------------------------------
    # ARTICULATIONS
    # -----------------------------------------------------------------------

    # Lid: fitted cover that lifts straight upward off the mouth rim.
    model.articulation(
        "body_to_lid",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, LID_SEAT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.35, lower=0.0, upper=LID_LIFT),
    )

    # Bail: horizontal revolute joint through both anchor lugs.
    # Axis along Y connects the two lugs at (0, ±LUG_Y, H_BODY).
    # At q=0 the bail arch hangs downward alongside the body (folded flat).
    # At q=upper the bail arch points upward above the basket for carrying.
    model.articulation(
        "body_to_bail",
        ArticulationType.REVOLUTE,
        parent=body,
        child=bail,
        origin=Origin(xyz=(0.0, 0.0, H_BODY)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=2.0, lower=0.0, upper=BAIL_UPPER
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("basket_body")
    lid = object_model.get_part("basket_lid")
    bail = object_model.get_part("carry_bail")
    lid_joint = object_model.get_articulation("body_to_lid")
    bail_joint = object_model.get_articulation("body_to_bail")

    # --- Material check ---
    mats = {
        (v.material.name if hasattr(v.material, "name") else v.material)
        for p in (body, lid, bail)
        for v in p.visuals
    }
    ctx.check(
        "rattan_materials",
        all(isinstance(m, str) and "rattan" in m for m in mats),
        f"expected rattan materials, got {sorted(mats)}",
    )

    # --- Body weave unchanged from parent ---
    horizontal_count = sum(
        1 for v in body.visuals if (v.name or "").startswith("horizontal_weave")
    )
    vertical_count = sum(
        1 for v in body.visuals if (v.name or "").startswith("vertical_weave")
    )
    ctx.check(
        "interlaced_body_weave",
        horizontal_count >= HORIZONTAL_ROWS and vertical_count >= VERTICAL_STAKES,
        f"horizontal={horizontal_count}, vertical={vertical_count}",
    )

    # --- Lid weave unchanged from parent ---
    lid_x = sum(1 for v in lid.visuals if (v.name or "").startswith("lid_weave_x"))
    lid_y = sum(1 for v in lid.visuals if (v.name or "").startswith("lid_weave_y"))
    lid_braid = sum(
        1 for v in lid.visuals if "braided_lid_outer_rim" in (v.name or "")
    )
    ctx.check(
        "woven_lid_face_and_braided_rim",
        lid_x >= 10 and lid_y >= 10 and lid_braid >= 2,
        f"lid_x={lid_x}, lid_y={lid_y}, braided_rim={lid_braid}",
    )

    # --- Lid lifts upward off the basket mouth ---
    ctx.check(
        "lift_off_lid_prismatic_axis",
        lid_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(lid_joint.axis) == (0.0, 0.0, 1.0)
        and lid_joint.motion_limits is not None
        and lid_joint.motion_limits.upper >= LID_LIFT,
        f"type={lid_joint.articulation_type}, axis={lid_joint.axis}, limits={lid_joint.motion_limits}",
    )

    # --- Bail arch exists with woven strands and braided wraps ---
    arch_strands = sum(
        1 for v in bail.visuals if (v.name or "").startswith("bail_arch_strand")
    )
    braid_wraps = sum(
        1 for v in bail.visuals if (v.name or "").startswith("bail_braided_wrap")
    )
    ctx.check(
        "woven_bail_arch",
        arch_strands >= 3 and braid_wraps >= 2,
        f"arch_strands={arch_strands}, braid_wraps={braid_wraps}",
    )

    # --- Anchor lugs exist on body rim ---
    lug_count = sum(
        1 for v in body.visuals if "bail_anchor_lug" in (v.name or "")
    )
    ctx.check(
        "anchor_lugs_on_rim",
        lug_count >= 2,
        f"expected >=2 lugs, got {lug_count}",
    )

    # --- Bail revolute joint with horizontal axis ---
    ctx.check(
        "bail_revolute_horizontal_axis",
        bail_joint.articulation_type == ArticulationType.REVOLUTE
        and abs(bail_joint.axis[1]) > 0.9
        and abs(bail_joint.axis[0]) < 0.1
        and abs(bail_joint.axis[2]) < 0.1,
        f"type={bail_joint.articulation_type}, axis={bail_joint.axis}",
    )

    # --- Bail joint origin at mouth rim height ---
    joint_z = bail_joint.origin.xyz[2] if bail_joint.origin else 0.0
    ctx.check(
        "bail_joint_at_mouth_rim",
        abs(joint_z - H_BODY) < 0.005,
        f"joint_z={joint_z}, expected ~{H_BODY}",
    )

    # --- Lid overlap with body (fitted cover sits on mouth) ---
    ctx.allow_overlap(
        lid,
        body,
        reason=(
            "Fitted lid: the braided outer lid rim rests over the basket "
            "mouth rim at the closed pose."
        ),
    )
    ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.12, name="lid_covers_mouth")
    with ctx.pose({lid_joint: LID_LIFT}):
        ctx.expect_gap(lid, body, axis="z", min_gap=0.005, name="lid_lifts_clear_of_mouth")

    # --- Bail folds down alongside body at rest ---
    ctx.allow_overlap(
        bail,
        body,
        reason=(
            "Carry bail arch rests against the body exterior when folded "
            "down; the woven arch contour contacts the bulbous body weave."
        ),
    )
    ctx.allow_overlap(
        bail,
        lid,
        reason=(
            "Bail pivot wraps sit at the mouth rim adjacent to the lid "
            "braided rim; small local contact at the pivot bearing."
        ),
    )

    # At rest (q=0), bail arch extends downward. The AABB min Z should be well
    # below the mouth rim, proving the bail hangs alongside the body.
    bail_rest_aabb = ctx.part_world_aabb(bail)
    ctx.check(
        "bail_rest_hangs_down",
        bail_rest_aabb is not None
        and bail_rest_aabb[0][2] < H_BODY - 0.08,
        f"bail_rest_aabb_min_z={bail_rest_aabb[0][2] if bail_rest_aabb else None},"
        f" expected below {H_BODY - 0.08}",
    )

    # --- Bail swings up for carrying ---
    with ctx.pose({bail_joint: BAIL_UPPER}):
        bail_up_aabb = ctx.part_world_aabb(bail)
        ctx.check(
            "bail_swings_up_for_carrying",
            bail_up_aabb is not None
            and bail_up_aabb[1][2] > H_BODY + 0.05,
            f"bail_up_aabb_max_z={bail_up_aabb[1][2] if bail_up_aabb else None},"
            f" expected above rim+0.05={H_BODY + 0.05}",
        )

    # --- Bail stays connected to body through motion ---
    ctx.expect_overlap(
        bail,
        body,
        axes="xy",
        min_overlap=0.01,
        name="bail_stays_connected_to_body",
    )

    return ctx.report()


object_model = build_object_model()
