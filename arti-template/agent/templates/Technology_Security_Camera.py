"""Security camera procedural template — modular (`__modular__ = True`).

Category identity (spec ``specs_modular_v1/Technology_Security_Camera.md``): a
**2-DOF articulated surveillance camera** = a rigid mount + a pan/tilt gimbal +
a camera head. Every seed shares ONE abstract gimbal spine

    rigid_mount --[pan REVOLUTE]--> carriage --[tilt REVOLUTE]--> camera_head

(the pan axis is the mount-face normal; the tilt axis is horizontal Y). The
pan/tilt joints are the heart of the object — a security camera is distinguished
from a webcam (no articulated mount) or a floodlight (no lens/imager) by exactly
this articulated mount + imaging head.

Sources (all 10 read in full; see spec §14). Origins:
  S1 silver bullet desk cam  (rec_a-silver-bullet-...1da3777f)  — ① desk / ③ bullet / D short_lip / IR N=20
  S2 white bullet wall-arm   (rec_a-white-bullet-...03a3be03)  — ① wall_arm / ② tilt_only / IR N=4
  S3 gray bullet wall-direct (rec_gray-cctv-bullet-...4e6dbba3) — ① wall_direct / D long_hood (cadquery)
  S4 white PTZ speed dome    (rec_a-white-ptz-...1892c604)     — ① ceiling / ③ speed_dome / IR N=8
Forks: turret_eyeball@S4 · box_housing@S3 · dual_lens@S1 · wall_arm_pan@S2 ·
ceiling_bullet@S3 · wall_arm_dome@S4.

Slots (spec §4):
  A ① mount_form:   desk / wall_arm / wall_direct / ceiling  (4 source anchors)
  B ② gimbal_dof:   tilt_only (drops the pan part+joint) / full_pan_tilt
  C ③ housing_form: bullet / box / speed_dome / turret_eyeball  (Primary Form Family)
  D   sunshield:    none / short_lip / long_hood  (bullet/box only)
Multiplicity: IR-ring N in {4,8,20}; lens_module N in {1,2}.

Motion safety (spec §5/§7, AUTHORING Rule 5): pan axis = mount normal and passes
through the (axisymmetric) pan knuckle bearing; tilt axis passes through the
captured cross-pin between the yoke cheeks. Both joints' realized limits are
*solved* by ``clamp_joint_limits`` against the mount/yoke keep-out so the head
clears the mount across the whole swept range (no hand-tuned angles). Captured
bearing overlaps (knuckle-in-seat, pin-in-trunnion, trunnion-in-cheek) are
declared element-scoped in ``run_tests`` and exempted in the solver from the
same single source, so the two never disagree.

Mating model: every joint is a mechanical pivot (knuckle-in-socket,
pin-through-yoke), so — like monitor_mount / cctv_mast — no ``MatingContract`` is
declared (grandfathered through ``fail_if_joint_mating_has_gap``); the intended
overlaps are allow-listed instead.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import Literal

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    MotionProperties,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    clamp_joint_limits,
    min_clearance_at_pose,
)

__modular__ = True

MountForm = Literal["desk", "wall_arm", "wall_direct", "ceiling"]
GimbalDof = Literal["tilt_only", "full_pan_tilt"]
HousingForm = Literal["bullet", "box", "speed_dome", "turret_eyeball"]
Sunshield = Literal["none", "short_lip", "long_hood"]
PaletteStyle = Literal["silver", "white", "gray", "black"]


# --------------------------------------------------------------------------- #
# Palette presets (≥3; §8.5 ⑥). Keys shared by every module.
# --------------------------------------------------------------------------- #
PALETTES: dict[str, dict[str, tuple[float, float, float, float]]] = {
    "silver": {
        "body": (0.72, 0.72, 0.68, 1.0),
        "mount": (0.55, 0.55, 0.53, 1.0),
        "dark": (0.06, 0.06, 0.06, 1.0),
        "glass": (0.03, 0.05, 0.08, 0.78),
        "led": (0.86, 0.95, 1.0, 0.82),
    },
    "white": {
        "body": (0.94, 0.95, 0.94, 1.0),
        "mount": (0.86, 0.88, 0.88, 1.0),
        "dark": (0.02, 0.02, 0.022, 1.0),
        "glass": (0.05, 0.08, 0.10, 0.72),
        "led": (0.86, 0.90, 0.92, 0.78),
    },
    "gray": {
        "body": (0.62, 0.66, 0.68, 1.0),
        "mount": (0.50, 0.53, 0.55, 1.0),
        "dark": (0.012, 0.013, 0.014, 1.0),
        "glass": (0.025, 0.035, 0.045, 0.82),
        "led": (0.90, 0.10, 0.05, 1.0),
    },
    "black": {
        "body": (0.085, 0.088, 0.092, 1.0),
        "mount": (0.13, 0.13, 0.14, 1.0),
        "dark": (0.015, 0.016, 0.018, 1.0),
        "glass": (0.05, 0.08, 0.10, 0.72),
        "led": (0.86, 0.90, 0.92, 0.78),
    },
}

_IR_COUNTS = (4, 8, 20)
_IR_WEIGHTS = (0.42, 0.42, 0.16)
_LENS_COUNTS = (1, 2)
_LENS_WEIGHTS = (0.78, 0.22)

# Legal housings per mount (spec §9 compatibility matrix / source-map gates:
# desk excludes dome/turret; wall_direct excludes dome; ceiling excludes box).
_LEGAL_HOUSINGS: dict[str, tuple[str, ...]] = {
    "desk": ("bullet", "box"),
    "wall_arm": ("bullet", "box", "speed_dome", "turret_eyeball"),
    "wall_direct": ("bullet", "box", "turret_eyeball"),
    "ceiling": ("bullet", "speed_dome", "turret_eyeball"),
}

# Single-sourced gimbal geometry (Contract 3c).
_TILT_DX = 0.064  # seat -> tilt pivot offset along +X (yoke reach). Kept larger
#                   than seat_boss_r + trunnion_r so the tilt axle clears the boss.
_X_REAR = 0.010  # camera-frame rear datum: all housing geometry sits at x >= this,
#                  forward of the yoke bridge (which occupies camera-x [-TILT_DX, 0]).
_KNUCKLE_R = 0.024
_KNUCKLE_L = 0.050
_SEAT_BOSS_R = 0.026
_SEAT_BOSS_L = 0.042
_PAN_MARGIN = 0.003
_TILT_MARGIN = 0.002


@dataclass(frozen=True)
class SecurityCameraConfig:
    mount_form: MountForm = "desk"
    gimbal_dof: GimbalDof = "full_pan_tilt"
    housing_form: HousingForm = "bullet"
    sunshield: Sunshield = "short_lip"
    palette_style: PaletteStyle = "silver"
    ir_count: int = 8
    lens_count: int = 1
    body_len_scale: float = 1.0
    body_radius_scale: float = 1.0
    mount_scale: float = 1.0


@dataclass(frozen=True)
class ResolvedSecurityCameraConfig:
    mount_form: MountForm
    gimbal_dof: GimbalDof
    housing_form: HousingForm
    sunshield: Sunshield
    palette_style: PaletteStyle
    ir_count: int
    lens_count: int
    body_len_scale: float
    body_radius_scale: float
    mount_scale: float
    palette: dict[str, tuple[float, float, float, float]] = field(default_factory=dict)


# --------------------------------------------------------------------------- #
# Sampling + gating.
# --------------------------------------------------------------------------- #
def config_from_seed(seed: int) -> SecurityCameraConfig:
    """Deterministic procedural sampling (seed 0 is not special)."""
    rng = random.Random(seed)
    mount: MountForm = rng.choice(("desk", "wall_arm", "wall_direct", "ceiling"))
    housing: HousingForm = rng.choice(_LEGAL_HOUSINGS[mount])  # type: ignore[assignment]

    # speed_dome has no meaningful still image without pan (source-map gate:
    # "tilt_only 不配 dome").
    if housing == "speed_dome":
        gimbal: GimbalDof = "full_pan_tilt"
    else:
        gimbal = rng.choices(("full_pan_tilt", "tilt_only"), weights=(0.62, 0.38), k=1)[0]

    if housing in ("bullet", "box"):
        sunshield: Sunshield = rng.choice(("none", "short_lip", "long_hood"))
    else:
        sunshield = "none"  # dome/turret carry no sunshield (source-map gate)

    ir_count = rng.choices(_IR_COUNTS, weights=_IR_WEIGHTS, k=1)[0]
    lens_count = rng.choices(_LENS_COUNTS, weights=_LENS_WEIGHTS, k=1)[0]

    return SecurityCameraConfig(
        mount_form=mount,
        gimbal_dof=gimbal,
        housing_form=housing,
        sunshield=sunshield,
        palette_style=rng.choice(("silver", "white", "gray", "black")),
        ir_count=ir_count,
        lens_count=lens_count,
        body_len_scale=round(rng.uniform(0.85, 1.18), 4),
        body_radius_scale=round(rng.uniform(0.88, 1.12), 4),
        mount_scale=round(rng.uniform(0.9, 1.15), 4),
    )


def _coerce_legal(
    mount: str, gimbal: str, housing: str, sunshield: str
) -> tuple[str, str, str, str]:
    """Project any (possibly corner-seed) combo onto the legal region."""
    legal = _LEGAL_HOUSINGS[mount]
    if housing not in legal:
        # Prefer bullet (universal), else the mount's first legal housing.
        housing = "bullet" if "bullet" in legal else legal[0]
    if housing == "speed_dome":
        gimbal = "full_pan_tilt"
    if housing in ("speed_dome", "turret_eyeball"):
        sunshield = "none"
    return mount, gimbal, housing, sunshield


def resolve_config(config: SecurityCameraConfig) -> ResolvedSecurityCameraConfig:
    if config.mount_form not in _LEGAL_HOUSINGS:
        raise ValueError(f"Unsupported mount_form: {config.mount_form}")
    if config.gimbal_dof not in ("tilt_only", "full_pan_tilt"):
        raise ValueError(f"Unsupported gimbal_dof: {config.gimbal_dof}")
    if config.housing_form not in ("bullet", "box", "speed_dome", "turret_eyeball"):
        raise ValueError(f"Unsupported housing_form: {config.housing_form}")
    if config.sunshield not in ("none", "short_lip", "long_hood"):
        raise ValueError(f"Unsupported sunshield: {config.sunshield}")
    if config.palette_style not in PALETTES:
        raise ValueError(f"Unsupported palette_style: {config.palette_style}")

    mount, gimbal, housing, sunshield = _coerce_legal(
        str(config.mount_form),
        str(config.gimbal_dof),
        str(config.housing_form),
        str(config.sunshield),
    )
    ir_count = int(config.ir_count) if int(config.ir_count) in _IR_COUNTS else 8
    lens_count = 2 if int(config.lens_count) >= 2 else 1

    return ResolvedSecurityCameraConfig(
        mount_form=mount,  # type: ignore[arg-type]
        gimbal_dof=gimbal,  # type: ignore[arg-type]
        housing_form=housing,  # type: ignore[arg-type]
        sunshield=sunshield,  # type: ignore[arg-type]
        palette_style=config.palette_style,
        ir_count=ir_count,
        lens_count=lens_count,
        body_len_scale=max(0.8, min(float(config.body_len_scale), 1.25)),
        body_radius_scale=max(0.82, min(float(config.body_radius_scale), 1.18)),
        mount_scale=max(0.85, min(float(config.mount_scale), 1.2)),
        palette=dict(PALETTES[config.palette_style]),
    )


def slot_choices_for_config(
    config: SecurityCameraConfig | ResolvedSecurityCameraConfig,
) -> tuple[tuple[str, str], ...]:
    r = config if isinstance(config, ResolvedSecurityCameraConfig) else resolve_config(config)
    return (
        ("mount_form", r.mount_form),
        ("gimbal_dof", r.gimbal_dof),
        ("housing_form", r.housing_form),
        ("sunshield", r.sunshield),
        ("palette_style", r.palette_style),
        ("ir_count", f"ir_{r.ir_count}"),
        ("lens_count", f"lens_{r.lens_count}"),
    )


def slot_choices_for_seed(seed: int) -> tuple[tuple[str, str], ...]:
    return slot_choices_for_config(config_from_seed(seed))


# --------------------------------------------------------------------------- #
# Geometry helpers.
# --------------------------------------------------------------------------- #
def _pan_axis(mount_form: str, housing_form: str) -> tuple[float, float, float]:
    """Pan axis.

    Bullet/box cameras pan about the mount-face normal (the source records'
    convention): +Z for desk/ceiling, +X (roll about the boresight) for wall
    knuckles. Dome/turret cameras hang on a pendant and pan about the vertical
    +Z on every mount (matching S4 / wall_arm_dome / turret sources) — a sphere
    rolled about its own boresight is motion-invisible, so that would be a false
    DOF. The axis always coincides with the axisymmetric pan knuckle bearing.
    """
    if housing_form in ("speed_dome", "turret_eyeball"):
        return (0.0, 0.0, 1.0)
    return (0.0, 0.0, 1.0) if mount_form in ("desk", "ceiling") else (1.0, 0.0, 0.0)


def _axis_rpy(axis: tuple[float, float, float]) -> tuple[float, float, float]:
    """rpy that rotates a default-Z cylinder onto ``axis`` (Z or X only)."""
    return (0.0, math.pi / 2.0, 0.0) if axis[0] > 0.5 else (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class _CameraFace:
    front_x: float  # x of the front lens plane
    face_r: float  # usable radius on the front face for lens/IR ring
    body_half: float  # lateral half-width (for yoke cheek offset)
    top_z: float  # z of the housing crown (sunshield seat)


def _camera_face(r: ResolvedSecurityCameraConfig) -> _CameraFace:
    """Front-face geometry, single-sourced so build + tests agree. Every housing
    rear sits at ``_X_REAR`` (forward of the yoke) and grows +X from there."""
    ls = r.body_len_scale
    rs = r.body_radius_scale
    if r.housing_form == "box":
        bl, bw, bh = 0.185 * ls, 0.100 * rs, 0.100 * rs
        return _CameraFace(
            front_x=_X_REAR + bl, face_r=min(bw, bh) * 0.5, body_half=bw * 0.5, top_z=bh * 0.5
        )
    if r.housing_form == "speed_dome":
        dome_r = 0.058 * rs
        return _CameraFace(
            front_x=_X_REAR + 1.72 * dome_r, face_r=0.044 * rs, body_half=dome_r, top_z=dome_r
        )
    if r.housing_form == "turret_eyeball":
        eye_r = 0.050 * rs
        return _CameraFace(
            front_x=_X_REAR + 1.35 * eye_r, face_r=0.034 * rs, body_half=eye_r, top_z=eye_r
        )
    # bullet
    br = 0.045 * rs
    bl = 0.170 * ls
    return _CameraFace(front_x=_X_REAR + bl, face_r=br, body_half=br, top_z=br)


def _cheek_offset(r: ResolvedSecurityCameraConfig) -> float:
    """Lateral half-gap between the yoke cheeks (single-sourced)."""
    return _camera_face(r).body_half + 0.020


def _emit_pan_knuckle(part, axis: tuple[float, float, float], mat) -> None:
    """Axisymmetric pan bearing barrel at the carriage origin (contains 0,0,0)."""
    part.visual(
        Cylinder(radius=_KNUCKLE_R, length=_KNUCKLE_L),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=_axis_rpy(axis)),
        material=mat,
        name="pan_knuckle",
    )


def _emit_yoke(
    part, base: tuple[float, float, float], r: ResolvedSecurityCameraConfig, *, metal, dark
) -> tuple[float, float, float]:
    """Bridge + two cheeks + captured cross-pin straddling the tilt pivot.

    Emitted on the pan carriage (full_pan_tilt) or directly on the mount
    (tilt_only). Returns the tilt pivot in the part's frame.
    """
    bx, by, bz = base
    cy = _cheek_offset(r)
    pivot = (bx + _TILT_DX, by, bz)
    # Bridge spans base -> pivot exactly (front edge at the pivot) so it never
    # pokes forward of the tilt axle into the camera body.
    part.visual(
        Box((_TILT_DX, 0.032, 0.020)),
        origin=Origin(xyz=(bx + _TILT_DX * 0.5, by, bz)),
        material=metal,
        name="yoke_bridge",
    )
    for i, sy in enumerate((cy, -cy)):
        part.visual(
            Box((0.030, 0.011, 0.052)),
            origin=Origin(xyz=(pivot[0], by + sy, bz)),
            material=metal,
            name=f"yoke_cheek_{i}",
        )
    part.visual(
        Cylinder(radius=0.006, length=2.0 * cy + 0.020),
        origin=Origin(xyz=pivot, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="tilt_pin",
    )
    return pivot


def _build_mount(
    part, r: ResolvedSecurityCameraConfig, *, metal, dark
) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[str, ...]]:
    """Build the rigid mount. Returns (seat_xyz, pan_axis, seat_element_names).

    ``seat`` is the pan (or tilt-only) gimbal attach point; the near-seat neck
    elements are axisymmetric about the pan axis so their overlap with the pan
    knuckle is rotation-invariant.
    """
    ms = r.mount_scale
    axis = _pan_axis(r.mount_form, r.housing_form)
    seat_boss_rpy = _axis_rpy(axis)

    if r.mount_form == "desk":
        part.visual(
            Cylinder(radius=0.060 * ms, length=0.014),
            origin=Origin(xyz=(0.0, 0.0, 0.007)),
            material=metal,
            name="base_plate",
        )
        for i, ang in enumerate((0.6, 2.6, 4.6)):
            part.visual(
                Cylinder(radius=0.005, length=0.006),
                origin=Origin(xyz=(0.040 * ms * math.cos(ang), 0.040 * ms * math.sin(ang), 0.013)),
                material=dark,
                name=f"base_screw_{i}",
            )
        seat_z = 0.107 * ms
        part.visual(
            Cylinder(radius=0.018, length=seat_z - 0.014),
            origin=Origin(xyz=(0.0, 0.0, 0.007 + (seat_z - 0.014) * 0.5)),
            material=metal,
            name="pedestal",
        )
        seat = (0.0, 0.0, seat_z)
        neck = "pedestal"
    elif r.mount_form == "ceiling":
        part.visual(
            Cylinder(radius=0.064 * ms, length=0.014),
            origin=Origin(xyz=(0.0, 0.0, -0.007)),
            material=metal,
            name="ceiling_plate",
        )
        for i in range(4):
            ang = i * math.pi / 2.0 + math.pi / 4.0
            part.visual(
                Cylinder(radius=0.0055, length=0.008),
                origin=Origin(xyz=(0.044 * ms * math.cos(ang), 0.044 * ms * math.sin(ang), -0.014)),
                material=dark,
                name=f"ceiling_screw_{i}",
            )
        drop = 0.075 * ms
        part.visual(
            Cylinder(radius=0.020, length=drop),
            origin=Origin(xyz=(0.0, 0.0, -0.014 - drop * 0.5)),
            material=metal,
            name="drop_stem",
        )
        seat = (0.0, 0.0, -0.014 - drop)
        neck = "drop_stem"
    elif r.mount_form == "wall_arm":
        part.visual(
            Box((0.015, 0.140 * ms, 0.160 * ms)),
            origin=Origin(xyz=(0.0075, 0.0, 0.0)),
            material=metal,
            name="wall_plate",
        )
        for i, (yy, zz) in enumerate(((0.05, 0.06), (-0.05, 0.06), (0.05, -0.06), (-0.05, -0.06))):
            part.visual(
                Cylinder(
                    radius=0.006,
                    length=0.010,
                ),
                origin=Origin(xyz=(0.010, yy * ms, zz * ms), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=dark,
                name=f"wall_screw_{i}",
            )
        arm_x1 = 0.125 * ms
        part.visual(
            Cylinder(radius=0.019, length=arm_x1 - 0.015),
            origin=Origin(
                xyz=(0.015 + (arm_x1 - 0.015) * 0.5, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)
            ),
            material=metal,
            name="support_arm",
        )
        seat = (arm_x1, 0.0, 0.0)
        neck = "support_arm"
    else:  # wall_direct
        part.visual(
            Box((0.015, 0.120 * ms, 0.120 * ms)),
            origin=Origin(xyz=(0.0075, 0.0, 0.0)),
            material=metal,
            name="wall_plate",
        )
        for i, (yy, zz) in enumerate(((0.045, 0.045), (-0.045, -0.045))):
            part.visual(
                Cylinder(radius=0.006, length=0.010),
                origin=Origin(xyz=(0.010, yy * ms, zz * ms), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=dark,
                name=f"wall_screw_{i}",
            )
        stub_x1 = 0.052 * ms
        part.visual(
            Cylinder(radius=0.024, length=stub_x1 - 0.015),
            origin=Origin(
                xyz=(0.015 + (stub_x1 - 0.015) * 0.5, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)
            ),
            material=metal,
            name="wall_stub",
        )
        seat = (stub_x1, 0.0, 0.0)
        neck = "wall_stub"

    # Pan bearing seat boss (coaxial with the pan axis) at the gimbal attach
    # point — the pan knuckle seats into it.
    part.visual(
        Cylinder(radius=_SEAT_BOSS_R, length=_SEAT_BOSS_L),
        origin=Origin(xyz=seat, rpy=seat_boss_rpy),
        material=dark,
        name="seat_boss",
    )
    part.inertial = Inertial.from_geometry(
        Box((0.14 * ms, 0.14 * ms, 0.14 * ms)),
        mass=1.4,
        origin=Origin(xyz=(seat[0] * 0.5, 0.0, seat[2] * 0.5)),
    )
    return seat, axis, ("seat_boss", neck)


def _build_camera_head(part, r: ResolvedSecurityCameraConfig, *, body, dark, glass, led) -> None:
    """Camera head: housing (③) + IR ring (N) + lens module(s) (N) + sunshield.

    Built in its own frame with the tilt pivot at the origin and boresight +X.
    ``tilt_trunnion`` (about Y at the origin) is the captured tilt axle.
    """
    face = _camera_face(r)
    fx = face.front_x

    # Captured tilt axle at the origin (contains 0,0,0 for the chain joint).
    cy = _cheek_offset(r)
    part.visual(
        Cylinder(radius=0.020, length=2.0 * cy - 0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="tilt_trunnion",
    )
    for i, sy in enumerate((cy - 0.004, -(cy - 0.004))):
        part.visual(
            Box((0.026, 0.010, 0.040)),
            origin=Origin(xyz=(0.010, sy, 0.0)),
            material=dark,
            name=f"mount_ear_{i}",
        )

    # ── ③ housing form (all geometry grows +X from the _X_REAR datum) ──────
    if r.housing_form == "box":
        bl, bw, bh = (
            0.185 * r.body_len_scale,
            0.100 * r.body_radius_scale,
            0.100 * r.body_radius_scale,
        )
        part.visual(
            Box((bl, bw, bh)),
            origin=Origin(xyz=(_X_REAR + bl * 0.5, 0.0, 0.0)),
            material=body,
            name="camera_body",
        )
        part.visual(
            Box((0.014, bw * 0.92, bh * 0.92)),
            origin=Origin(xyz=(_X_REAR + 0.007, 0.0, 0.0)),
            material=dark,
            name="rear_plate",
        )
    elif r.housing_form == "speed_dome":
        dome_r = 0.058 * r.body_radius_scale
        part.visual(
            Cylinder(radius=dome_r * 1.02, length=0.045),
            origin=Origin(xyz=(_X_REAR + 0.022, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=body,
            name="dome_canopy",
        )
        part.visual(
            Sphere(radius=dome_r),
            origin=Origin(xyz=(_X_REAR + dome_r, 0.0, 0.0)),
            material=glass,
            name="dome_cover",
        )
        part.visual(
            Cylinder(radius=dome_r * 0.70, length=0.030),
            origin=Origin(xyz=(_X_REAR + 0.020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name="camera_core",
        )
    elif r.housing_form == "turret_eyeball":
        eye_r = 0.050 * r.body_radius_scale
        part.visual(
            Cylinder(radius=eye_r * 1.06, length=0.020),
            origin=Origin(xyz=(_X_REAR + 0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name="socket_ring",
        )
        part.visual(
            Sphere(radius=eye_r),
            origin=Origin(xyz=(_X_REAR + eye_r * 0.8, 0.0, 0.0)),
            material=body,
            name="eyeball",
        )
    else:  # bullet
        br = 0.045 * r.body_radius_scale
        bl = 0.170 * r.body_len_scale
        part.visual(
            Cylinder(radius=br, length=bl),
            origin=Origin(xyz=(_X_REAR + bl * 0.5, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=body,
            name="camera_body",
        )
        part.visual(
            Cylinder(radius=br * 0.92, length=0.016),
            origin=Origin(xyz=(_X_REAR + 0.008, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name="rear_cap",
        )

    # ── front bezel plate (host surface the lens/IR ring embed into) ───────
    part.visual(
        Cylinder(radius=face.face_r * 1.02, length=0.006),
        origin=Origin(xyz=(fx - 0.003, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="front_bezel",
    )

    # ── lens module(s) (multiplicity N) ────────────────────────────────────
    lens_r = 0.014
    offsets = (0.0,) if r.lens_count == 1 else (-0.024, 0.024)
    for j, yoff in enumerate(offsets):
        part.visual(
            Cylinder(radius=lens_r, length=0.016),
            origin=Origin(xyz=(fx + 0.005, yoff, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name=f"lens_barrel_{j}",
        )
        part.visual(
            Cylinder(radius=lens_r * 0.72, length=0.006),
            origin=Origin(xyz=(fx + 0.012, yoff, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=glass,
            name=f"front_glass_{j}",
        )

    # ── IR-LED ring (multiplicity N), host-conformal on the front face ─────
    lens_spread = 0.0 if r.lens_count == 1 else 0.024
    ir_r = min(max(0.024, lens_spread + 0.014), face.face_r - 0.006)
    for i in range(r.ir_count):
        ang = 2.0 * math.pi * i / r.ir_count + math.pi / r.ir_count
        part.visual(
            Cylinder(radius=0.0038, length=0.008),
            origin=Origin(
                xyz=(fx + 0.004, ir_r * math.cos(ang), ir_r * math.sin(ang)),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=led,
            name=f"ir_led_{i}",
        )

    # ── sunshield (bullet/box only) ────────────────────────────────────────
    if r.sunshield in ("short_lip", "long_hood"):
        top = face.top_z
        half_w = face.body_half + 0.010
        if r.sunshield == "short_lip":
            part.visual(
                Box((0.075, 2.0 * half_w, 0.005)),
                origin=Origin(xyz=(fx * 0.55, 0.0, top + 0.001)),
                material=dark,
                name="sunshield_top",
            )
        else:  # long_hood: overhanging plate + two side cheeks (U-channel).
            # Kept forward of the pivot (rear at x=0.020) so it never dips into
            # the tilt yoke as the head nods.
            hood_rear = 0.020
            hood_front = fx + 0.045
            hood_len = hood_front - hood_rear
            hood_cx = 0.5 * (hood_rear + hood_front)
            part.visual(
                Box((hood_len, 2.0 * half_w, 0.006)),
                origin=Origin(xyz=(hood_cx, 0.0, top + 0.002)),
                material=dark,
                name="sunshield_top",
            )
            for i, sy in enumerate((half_w, -half_w)):
                part.visual(
                    Box((hood_len * 0.94, 0.006, 0.028)),
                    origin=Origin(xyz=(hood_cx, sy, top - 0.011)),
                    material=dark,
                    name=f"sunshield_side_{i}",
                )

    part.inertial = Inertial.from_geometry(
        Box((max(0.12, fx + 0.05), 0.12, 0.12)),
        mass=0.55,
        origin=Origin(xyz=(fx * 0.45, 0.0, 0.0)),
    )


# --------------------------------------------------------------------------- #
# Captured-interface bookkeeping (single source for solver + allow_overlap).
# --------------------------------------------------------------------------- #
def _pan_exempt_pairs(seat_elems: tuple[str, ...]) -> list[tuple[str, str, str, str]]:
    """Carriage knuckle/bridge vs the mount's axisymmetric seat/neck near the
    pan axis (rotation-invariant contacts)."""
    pairs: list[tuple[str, str, str, str]] = []
    for near in ("pan_knuckle", "yoke_bridge"):
        for e in seat_elems:
            pairs.append(("carriage", "mount", near, e))
    return pairs


# Camera-head elements near the tilt axle whose contact with the yoke is a
# captured-bearing engagement (rotation-invariant about the tilt axis): the
# trunnion, mount ears, rear caps, and the dome/eyeball shells that wrap the
# axle. The forward camera_body is deliberately excluded so a real body-into-yoke
# 穿模 is still caught.
_CAM_AXLE_ELEMS = (
    "tilt_trunnion",
    "mount_ear_0",
    "mount_ear_1",
    "rear_cap",
    "rear_plate",
    "socket_ring",
    "eyeball",
    "dome_cover",
    "dome_canopy",
    "camera_core",
)
_YOKE_ELEMS = ("yoke_cheek_0", "yoke_cheek_1", "tilt_pin", "yoke_bridge", "pan_knuckle")


def _tilt_exempt_pairs(carrier: str) -> list[tuple[str, str, str, str]]:
    """Camera axle/rear shells captured by the yoke (coaxial with the tilt axis)."""
    return [("camera_head", carrier, cam, yoke) for cam in _CAM_AXLE_ELEMS for yoke in _YOKE_ELEMS]


# --------------------------------------------------------------------------- #
# Assembly.
# --------------------------------------------------------------------------- #
def build_security_camera(
    config: SecurityCameraConfig | None = None,
    *,
    assets: AssetContext | None = None,
) -> ArticulatedObject:
    config = config or SecurityCameraConfig()
    r = resolve_config(config)
    model = ArticulatedObject(name="security_camera", assets=assets)
    p = r.palette
    body = model.material("cam_body", rgba=p["body"])
    metal = model.material("cam_mount", rgba=p["mount"])
    dark = model.material("cam_dark", rgba=p["dark"])
    glass = model.material("cam_glass", rgba=p["glass"])
    led = model.material("cam_led", rgba=p["led"])

    mount = model.part("mount")
    seat, axis, seat_elems = _build_mount(mount, r, metal=metal, dark=dark)

    if r.gimbal_dof == "full_pan_tilt":
        carriage = model.part("carriage")
        _emit_pan_knuckle(carriage, axis, metal)
        pivot_local = _emit_yoke(carriage, (0.0, 0.0, 0.0), r, metal=metal, dark=dark)
        carriage.inertial = Inertial.from_geometry(
            Box((0.10, 0.10, 0.08)), mass=0.30, origin=Origin(xyz=(_TILT_DX * 0.5, 0.0, 0.0))
        )
        model.articulation(
            "pan_joint",
            ArticulationType.REVOLUTE,
            parent=mount,
            child=carriage,
            origin=Origin(xyz=seat),
            axis=axis,
            motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-math.pi, upper=math.pi),
            motion_properties=MotionProperties(damping=0.15, friction=0.10),
        )
        tilt_parent = carriage
        tilt_origin = pivot_local  # in carriage frame
        carrier_name = "carriage"
    else:
        pivot_local = _emit_yoke(mount, seat, r, metal=metal, dark=dark)
        tilt_parent = mount
        tilt_origin = pivot_local  # in mount frame
        carrier_name = "mount"

    camera = model.part("camera_head")
    _build_camera_head(camera, r, body=body, dark=dark, glass=glass, led=led)

    model.articulation(
        "tilt_joint",
        ArticulationType.REVOLUTE,
        parent=tilt_parent,
        child=camera,
        origin=Origin(xyz=tilt_origin),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.4, lower=-1.0, upper=0.7),
        motion_properties=MotionProperties(damping=0.25, friction=0.16),
    )

    _solve_gimbal_limits(model, r, seat_elems, carrier_name)

    model.meta["template_slug"] = "Technology_Security_Camera"
    model.meta["slot_choices"] = [list(t) for t in slot_choices_for_config(r)]
    model.meta["adopted_source_ids"] = ("S1", "S2", "S3", "S4")
    return model


def _solve_gimbal_limits(
    model: ArticulatedObject,
    r: ResolvedSecurityCameraConfig,
    seat_elems: tuple[str, ...],
    carrier: str,
) -> None:
    """Derive pan/tilt travel from the actual mount/yoke keep-out (Contract 3d).

    The head must clear the mount across the full swept range; rather than
    hand-tune angles, the clearance solver shrinks the candidate range until the
    moving subtree stays clear, exempting only the captured bearing contacts.
    """
    tilt_exempt = _tilt_exempt_pairs(carrier)
    # Solve tilt FIRST so the pan solve uses the *clamped* worst-case down-tilt
    # (the raw candidate over-dips the nose and would falsely collapse pan).
    clamp_joint_limits(
        model,
        "tilt_joint",
        margin=_TILT_MARGIN,
        keepout=[carrier, "mount"],
        allowed_pairs=tilt_exempt,
    )
    if r.gimbal_dof == "full_pan_tilt":
        pan_exempt = _pan_exempt_pairs(seat_elems)
        clamp_joint_limits(
            model,
            "pan_joint",
            margin=_PAN_MARGIN,
            keepout=["mount"],
            allowed_pairs=pan_exempt + tilt_exempt,
            base_pose={"tilt_joint": model.get_articulation("tilt_joint").motion_limits.lower},
        )


def build_seeded_security_camera(
    seed: int, *, assets: AssetContext | None = None
) -> ArticulatedObject:
    return build_security_camera(config_from_seed(seed), assets=assets)


# --------------------------------------------------------------------------- #
# Author tests.
# --------------------------------------------------------------------------- #
def _declare_captured_overlaps(ctx, model, r: ResolvedSecurityCameraConfig, carrier: str) -> None:
    for part_a, part_b, elem_a, elem_b in _tilt_exempt_pairs(carrier):
        ctx.allow_overlap(
            part_a,
            part_b,
            elem_a=elem_a,
            elem_b=elem_b,
            reason=f"{elem_a} captured in tilt yoke ({elem_b})",
        )
    if r.gimbal_dof == "full_pan_tilt":
        # Recover the mount's seat/neck element names from the built model.
        seat_elems = tuple(
            v.name
            for v in model.get_part("mount").visuals
            if getattr(v, "name", None)
            in {"seat_boss", "pedestal", "drop_stem", "support_arm", "wall_stub"}
        )
        for part_a, part_b, elem_a, elem_b in _pan_exempt_pairs(seat_elems):
            ctx.allow_overlap(
                part_a,
                part_b,
                elem_a=elem_a,
                elem_b=elem_b,
                reason=f"pan knuckle seats in {elem_b} (coaxial)",
            )


def _aabb_center(ab) -> tuple[float, float, float]:
    (lo, hi) = ab
    return ((lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5, (lo[2] + hi[2]) * 0.5)


def _led_world(ctx, model) -> tuple[float, float, float]:
    """World center of an off-axis IR LED — a good motion witness for both the
    pan (which for wall knuckles rolls about the boresight, leaving on-axis
    points fixed) and tilt joints."""
    cam = model.get_part("camera_head")
    ab = ctx.part_element_world_aabb(cam, elem="ir_led_0")
    if ab is None:
        ab = ctx.part_world_aabb(cam)
    return _aabb_center(ab)


def _expect_tilt_moves_head(ctx, model) -> None:
    tilt = model.get_articulation("tilt_joint")
    rest = _led_world(ctx, model)
    lo, hi = float(tilt.motion_limits.lower), float(tilt.motion_limits.upper)
    angle = lo if abs(lo) >= abs(hi) else hi
    with ctx.pose({tilt: angle * 0.85}):
        tilted = _led_world(ctx, model)
    ctx.check(
        "tilt_moves_camera_head", abs(tilted[2] - rest[2]) > 0.01, f"rest={rest}, tilted={tilted}"
    )


def _expect_pan_moves_head(ctx, model) -> None:
    if not any(j.name == "pan_joint" for j in model.articulations):
        return
    pan = model.get_articulation("pan_joint")
    rest = _led_world(ctx, model)
    with ctx.pose({pan: float(pan.motion_limits.upper) * 0.85}):
        panned = _led_world(ctx, model)
    disp = math.dist(rest, panned)
    ctx.check("pan_moves_camera_head", disp > 0.01, f"rest={rest}, panned={panned}, |Δ|={disp:.4f}")


def _expect_solved_joints_bounded(ctx, model, r, seat_elems, carrier) -> None:
    tilt_exempt = _tilt_exempt_pairs(carrier)
    specs: list[tuple[str, list, list[str], float]] = [
        ("tilt_joint", tilt_exempt, [carrier, "mount"], _TILT_MARGIN),
    ]
    if r.gimbal_dof == "full_pan_tilt":
        specs.append(
            ("pan_joint", _pan_exempt_pairs(seat_elems) + tilt_exempt, ["mount"], _PAN_MARGIN)
        )
    base = {j.name: 0.0 for j in model.articulations}
    for joint_name, allowed, keepout, margin in specs:
        joint = model.get_articulation(joint_name)
        lo, hi = float(joint.motion_limits.lower), float(joint.motion_limits.upper)
        ctx.check(
            f"{joint_name}_is_revolute",
            joint.articulation_type == ArticulationType.REVOLUTE,
            str(joint.articulation_type),
        )
        ctx.check(
            f"{joint_name}_travel_nontrivial", lo < hi and (hi - lo) > 0.1, f"[{lo:.4f}, {hi:.4f}]"
        )
        for edge, val in (("lower", lo), ("upper", hi)):
            pose = dict(base)
            pose[joint_name] = val
            clear = min_clearance_at_pose(
                model,
                joint=joint_name,
                joint_positions=pose,
                keepout=keepout,
                allowed_pairs=allowed,
            )
            ctx.check(
                f"{joint_name}_{edge}_clear",
                clear >= margin - 0.002,
                f"{joint_name}={val:.4f} clear={clear:.4f}",
            )


def run_security_camera_tests(model: ArticulatedObject, config: SecurityCameraConfig) -> TestReport:
    r = resolve_config(config)
    ctx = TestContext(model)
    carrier = "carriage" if r.gimbal_dof == "full_pan_tilt" else "mount"

    part_names = {part.name for part in model.parts}
    ctx.check("identity_parts", {"mount", "camera_head"}.issubset(part_names), str(part_names))
    ctx.check(
        "carriage_present_iff_pan",
        ("carriage" in part_names) == (r.gimbal_dof == "full_pan_tilt"),
        f"{r.gimbal_dof}: {part_names}",
    )

    cam_visuals = {v.name for v in model.get_part("camera_head").visuals}
    ctx.check(
        "camera_has_lens",
        any(n.startswith("front_glass") for n in cam_visuals),
        str(sorted(cam_visuals)),
    )
    ctx.check(
        "camera_has_ir_ring",
        sum(1 for n in cam_visuals if n.startswith("ir_led_")) == r.ir_count,
        f"expected {r.ir_count}",
    )

    tilt = model.get_articulation("tilt_joint")
    ctx.check(
        "tilt_axis_horizontal",
        abs(tilt.axis[2]) < 1e-6 and (abs(tilt.axis[0]) > 0.9 or abs(tilt.axis[1]) > 0.9),
        str(tilt.axis),
    )
    ctx.check("tilt_child_is_camera", tilt.child == "camera_head", f"{tilt.parent}->{tilt.child}")
    if r.gimbal_dof == "full_pan_tilt":
        pan = model.get_articulation("pan_joint")
        expected_axis = _pan_axis(r.mount_form, r.housing_form)
        ctx.check(
            "pan_axis_is_mount_normal",
            tuple(pan.axis) == expected_axis,
            f"{pan.axis} vs {expected_axis}",
        )
        ctx.check("pan_parent_is_mount", pan.parent == "mount", pan.parent)

    _declare_captured_overlaps(ctx, model, r, carrier)

    ctx.check_model_valid()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.015)
    ctx.fail_if_joint_mating_has_gap()
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=64, ignore_fixed=True)

    _expect_tilt_moves_head(ctx, model)
    _expect_pan_moves_head(ctx, model)
    seat_elems = tuple(
        v.name
        for v in model.get_part("mount").visuals
        if getattr(v, "name", None)
        in {"seat_boss", "pedestal", "drop_stem", "support_arm", "wall_stub"}
    )
    _expect_solved_joints_bounded(ctx, model, r, seat_elems, carrier)

    return ctx.report()


__all__ = [
    "__modular__",
    "MountForm",
    "GimbalDof",
    "HousingForm",
    "Sunshield",
    "PaletteStyle",
    "SecurityCameraConfig",
    "ResolvedSecurityCameraConfig",
    "build_security_camera",
    "build_seeded_security_camera",
    "config_from_seed",
    "resolve_config",
    "run_security_camera_tests",
    "slot_choices_for_config",
    "slot_choices_for_seed",
]
