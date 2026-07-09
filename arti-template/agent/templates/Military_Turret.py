"""Procedural modular template for the ``sentry_turret`` category.

Follows ``articraft_template_authoring/specs_modular_v1/Military_Turret.md``.

An automated sentry gun turret: a grounded (or ceiling-inverted) mount carries a
**continuously yawing** stage; that stage cradles, on trunnion ears, a
**revolute-pitching** receiver (-10..+60 deg upright; ceiling mounts cap the
up-pitch at the geometry-derived plate-clearance angle, see
``_ceiling_pitch_clearance_limit``); the receiver fronts a cluster of
**prismatic-recoiling** barrels (0..RECOIL_TRAVEL along the barrel axis). The
2-DOF aiming chain (pan yaw + tilt pitch) plus recoil is preserved across every
base / barrel-count / optics candidate.

World frame: +X is the barrel (fire) direction at yaw = 0, +Z is up.

Articulation chain (joint names follow the base prefix)::

    base_mount --(<prefix>_to_collar_yaw: CONTINUOUS +Z)--> support_collar
    support_collar --(collar_to_receiver_pitch: REVOLUTE -Y, -10..+60)--> receiver
    receiver --(receiver_to_barrels_recoil: PRISMATIC -X, 0..RECOIL_TRAVEL)--> barrel_cluster

Two multiplicity axes:
  * ``barrel_count`` (all bases) — a single ``BARREL_OFFSETS`` offset-list loop
    drives N=1..6 barrels through one shared recoil prismatic (the parent's
    nested 2x2 loop is rewritten as a single offset-list loop per single1/twin2).
  * ``leg_count`` {3,4,6} — only for leg-type bases (splayed_quad_legs /
    tripod_3legs); gated off for solid_pedestal / ceiling_mount.

Slot C optics (none / camera / laser_pod) emit into the ``receiver`` part (no
new joint) and ride with pitch.

Adopted source modules (spec Module Source Index):
  S1 d996dd05 — splayed_quad_legs leg loop + platform + receiver/collar/barrel
                blueprint (the parent).
  S2 tripod3  — leg_count N=3.
  S3 legs6    — leg_count N=6.
  S4 pedestal — lathe-revolved solid pedestal drum.
  S5 ceiling  — inverted ceiling plate + boss + perforated shroud + inverted collar.
  S6 single1  — offset-list single-loop barrel copy logic (copy-logic main source).
  S7 twin2    — N=2 horizontal offset placement.
  S8 d996dd05 — 2x2 grid offset values (rewritten as a 4-entry offset-list).
  S9 cameraoptic — recessed-lens targeting camera sight subtree.
  S10 laserpod   — side laser/sensor pod + bracket-to-sidewall bridge.
  S11 d996dd05   — support_collar + trunnion captured-pin + recoil prismatic.
"""

from __future__ import annotations

import math
import random
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

__modular__ = True

# --------------------------------------------------------------------------- #
# Enums
# --------------------------------------------------------------------------- #

BaseChoice = Literal["splayed_quad_legs", "tripod_3legs", "solid_pedestal", "ceiling_mount"]
OpticsChoice = Literal["none", "camera", "laser_pod"]
PaletteStyle = Literal[
    "safety_yellow_gunmetal",
    "industrial_yellow_black",
    "military_green",
    "gunmetal_mono",
    "white_drone",
]

_BASE_CHOICES: tuple[BaseChoice, ...] = (
    "splayed_quad_legs",
    "tripod_3legs",
    "solid_pedestal",
    "ceiling_mount",
)
_LEG_TYPE_BASES: frozenset[str] = frozenset({"splayed_quad_legs", "tripod_3legs"})
_OPTICS_CHOICES: tuple[OpticsChoice, ...] = ("none", "camera", "laser_pod")
_PALETTE_STYLES: tuple[PaletteStyle, ...] = (
    "safety_yellow_gunmetal",
    "industrial_yellow_black",
    "military_green",
    "gunmetal_mono",
    "white_drone",
)

# barrel_count weighted toward small N (4 lightly favored = parent baseline).
_BARREL_COUNTS: tuple[int, ...] = (1, 2, 3, 4, 5, 6)
_BARREL_WEIGHTS: tuple[float, ...] = (3.0, 3.0, 2.5, 3.5, 1.5, 1.5)
_LEG_COUNTS: tuple[int, ...] = (3, 4, 6)
_LEG_WEIGHTS: tuple[float, ...] = (2.5, 3.5, 2.0)

# --------------------------------------------------------------------------- #
# Palette: each colorway = >=3 named materials + a finish descriptor that drives
# nothing structural. Every visual gets one of these materials.
# --------------------------------------------------------------------------- #

PALETTES: dict[PaletteStyle, dict[str, tuple[float, float, float, float]]] = {
    "safety_yellow_gunmetal": {
        "body": (0.95, 0.76, 0.10, 1.0),
        "metal": (0.15, 0.16, 0.18, 1.0),
        "dark": (0.24, 0.25, 0.27, 1.0),
        "barrel": (0.46, 0.47, 0.50, 1.0),
    },
    "industrial_yellow_black": {
        "body": (0.96, 0.74, 0.08, 1.0),
        "metal": (0.09, 0.09, 0.10, 1.0),
        "dark": (0.18, 0.18, 0.20, 1.0),
        "barrel": (0.44, 0.45, 0.48, 1.0),
    },
    "military_green": {
        "body": (0.27, 0.31, 0.18, 1.0),
        "metal": (0.15, 0.16, 0.18, 1.0),
        "dark": (0.12, 0.14, 0.09, 1.0),
        "barrel": (0.40, 0.41, 0.43, 1.0),
    },
    "gunmetal_mono": {
        "body": (0.20, 0.21, 0.23, 1.0),
        "metal": (0.13, 0.13, 0.15, 1.0),
        "dark": (0.10, 0.10, 0.12, 1.0),
        "barrel": (0.46, 0.47, 0.50, 1.0),
    },
    "white_drone": {
        "body": (0.90, 0.91, 0.92, 1.0),
        "metal": (0.55, 0.56, 0.58, 1.0),
        "dark": (0.40, 0.41, 0.43, 1.0),
        "barrel": (0.48, 0.49, 0.52, 1.0),
    },
}

# Optics lens colors are fixed by the optics module (identity, not palette).
_CAMERA_HOUSING = (0.10, 0.11, 0.13, 1.0)
_CAMERA_LENS = (0.04, 0.05, 0.12, 1.0)
_CAMERA_LED = (0.85, 0.20, 0.10, 1.0)
_LASER_LENS = (0.40, 0.06, 0.06, 1.0)

# --------------------------------------------------------------------------- #
# Fixed structural constants (from the parent / variants). Scaled at build via
# the resolved config's scale fields.
# --------------------------------------------------------------------------- #

# Leg geometry (leg-local frame: +X radial outward, +Z up).
_HUB_PT = (0.10, 0.74)
_KNEE_PT = (0.50, 0.30)
_FOOT_PT = (0.72, 0.06)
_LEG_W = 0.085
_LEG_T = 0.052
_LEG_WALL = 0.009
_FOOT_PAD_R = 0.075
_FOOT_PAD_H = 0.05

# Column / platform (leg base).
_PLATFORM_R = 0.18
_PLATFORM_Z0, _PLATFORM_Z1 = 0.70, 0.78
_COLUMN_R = 0.085
_COLUMN_Z0, _COLUMN_Z1 = 0.26, 0.72
_SHELL_R_OUT, _SHELL_R_IN = 0.112, 0.103
_SHELL_Z0, _SHELL_Z1 = 0.30, 0.71
_LEG_TOP_Z = _PLATFORM_Z1  # 0.78 — yaw interface plane for leg base

# Pedestal drum.
_PED_TOP_Z = 0.78
_PED_BASE_PLATE_R = 0.40
_PED_BASE_PLATE_H = 0.040
_PED_SHAFT_R = 0.26
_PED_SHAFT_BAND_R = 0.275
_PED_TOP_FLANGE_R = 0.31
_PED_TOP_FLANGE_H = 0.06
_PED_N_BOLTS = 8

# Ceiling plate (inverted).
_CEIL_Z_TOP = 1.15
_CEIL_PLATE_R = 0.25
_CEIL_PLATE_H = 0.04
_CEIL_PLATE_BOTTOM = _CEIL_Z_TOP - _CEIL_PLATE_H  # 1.11
_CEIL_BOSS_R = 0.12
_CEIL_BOSS_H = 0.06
_CEIL_BOSS_BOTTOM = _CEIL_PLATE_BOTTOM - _CEIL_BOSS_H  # 1.05
_CEIL_SHROUD_R_OUT = 0.145
_CEIL_SHROUD_R_IN = 0.135
_CEIL_SHROUD_TOP = _CEIL_PLATE_BOTTOM - 0.005
_CEIL_SHROUD_BOT = _CEIL_BOSS_BOTTOM + 0.005
_CEIL_SHROUD_H = _CEIL_SHROUD_TOP - _CEIL_SHROUD_BOT
_CEIL_N_BOLTS = 8
_CEIL_BOLT_CIRCLE_R = _CEIL_PLATE_R - 0.035
_CEIL_BOLT_HOLE_R = 0.010
_CEIL_BOLT_HEAD_R = 0.014
_CEIL_BOLT_HEAD_H = 0.008
_CEIL_YAW_Z = _CEIL_BOSS_BOTTOM  # 1.05

# Support collar (shared structural layer, local frame origin on the yaw plane).
_HEX_FLAT_HALF = 0.135
_HEX_CIRCUM_R = _HEX_FLAT_HALF / math.cos(math.pi / 6.0)
_HEX_H = 0.05
_CHEEK_Y_IN, _CHEEK_Y_OUT = 0.152, 0.182
_CHEEK_X = 0.13
_CHEEK_Z0, _CHEEK_Z1 = 0.045, 0.29
_PIVOT_Z_LOCAL = 0.24  # pitch axis height above collar frame (upright bases)
_PIVOT_Z_LOCAL_INV = 0.36  # ceiling drop: deeper so the pitching receiver clears the collar
_CHEEK_FLANGE_H = 0.03
_CHEEK_FLANGE_Z_INV = -0.055  # flange center z on the inverted collar

# Receiver (local frame origin on the pitch axis).
_RECV_X0, _RECV_X1 = -0.11, 0.25
_RECV_HALF_Y = 0.149
_RECV_HALF_Z = 0.13
_RECV_WALL = 0.012
_BORE_R = 0.0278

# Barrel geometry (relative to the pitch axis / receiver frame).
_BARREL_DY = 0.07
_BARREL_DZ = 0.058
_BARREL_R_OUT, _BARREL_R_IN = 0.028, 0.0185
_BARREL_X1 = 0.66  # tube tip x at scale 1.0 (breech x is recoil-derived: _breech_x0)
_SHROUD_R_OUT, _SHROUD_R_IN = 0.047, 0.0278
_SHROUD_X0, _SHROUD_LEN = 0.53, 0.115
_CAP_R_OUT, _CAP_R_IN = 0.048, 0.0295
_CAP_X0, _CAP_LEN = 0.636, 0.018

_RECOIL_TRAVEL = 0.06
_PITCH_LO = math.radians(-10.0)
_PITCH_HI = math.radians(60.0)

# Minimum clearance kept between moving parts at joint extremes (motion QC
# overlap_tol is 5 mm; 10 mm keeps a tessellation-safe margin).
_MOTION_CLEARANCE = 0.010
_TRUNNION_HUB_R = 0.040  # receiver trunnion hub (widest on-axis pitch hardware)
_BEARING_SHAFT_R = 0.030  # collar bearing cross-shaft (also on the pitch axis)
_CARRIAGE_LEN = 0.18
_RECV_INNER_REAR_X = _RECV_X0 + _RECV_WALL  # inner face of the receiver rear wall

# Camera sight block (receiver-local).
_SIGHT_BODY_L = 0.085
_SIGHT_BODY_W = 0.026
_SIGHT_BODY_H = 0.038
_SIGHT_BASE_L = 0.060
_SIGHT_BASE_W = 0.028
_SIGHT_BASE_H = 0.005
_SIGHT_LENS_R = 0.0105
_SIGHT_LENS_DEPTH = 0.006
_SIGHT_VISOR_L = 0.022
_SIGHT_VISOR_W = 0.030
_SIGHT_VISOR_H = 0.004
_SIGHT_X0 = 0.07 - _SIGHT_BODY_L / 2.0  # sight rear x on the receiver deck

# Laser sensor pod (receiver-local).
_SENSOR_R = 0.025
_SENSOR_LEN = 0.20
_SENSOR_X_CENTER = 0.12
_SENSOR_X_CENTER_INV = 0.19  # pushed forward on ceiling mounts (see _sensor_x_center)
_POD_BRACKET_L = 0.08
_POD_BRACKET_H = 0.035
_SENSOR_Y = 0.24
_SENSOR_Z = -0.03
_SENSOR_LENS_R = 0.020
_SENSOR_LENS_T = 0.004
_SENSOR_CAP_R = 0.026
_SENSOR_CAP_T = 0.008

# Coarse-ish mesh tessellation to keep per-seed compile light.
_MESH_TOL = 0.0018
_MESH_ANG_TOL = 0.20


# --------------------------------------------------------------------------- #
# Config
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class SentryTurretConfig:
    """Public, JSON-serialisable knobs."""

    base_choice: BaseChoice = "splayed_quad_legs"
    barrel_count: int = 4
    optics_choice: OpticsChoice = "none"
    leg_count: int = 4
    palette_style: PaletteStyle = "safety_yellow_gunmetal"
    pitch_range_scale: float = 1.0
    recoil_travel_scale: float = 1.0
    barrel_len_scale: float = 1.0
    support_scale: float = 1.0
    footprint_scale: float = 1.0
    name: str = "seeded_sentry_turret"


@dataclass(frozen=True)
class ResolvedSentryTurretConfig:
    """Clamped + derived configuration consumed by the build helpers."""

    base_choice: BaseChoice
    barrel_count: int
    optics_choice: OpticsChoice
    leg_count: int
    palette_style: PaletteStyle
    pitch_upper: float
    recoil_travel: float
    barrel_len_scale: float
    support_scale: float
    footprint_scale: float
    # derived placement
    barrel_offsets: tuple[tuple[float, float], ...]
    barrel_x0: float
    barrel_x1: float
    shroud_x0: float
    cap_x0: float
    yaw_z: float
    inverted: bool
    name: str


def _clamp(value: float, lo: float, hi: float) -> float:
    if lo > hi:
        lo, hi = hi, lo
    return max(lo, min(hi, float(value)))


def config_from_seed(seed: int) -> SentryTurretConfig:
    """Deterministic procedural sampling; seed 0 is not special.

    Order: base -> optics -> barrel_count(small-weighted) -> leg_count(only for
    leg-type bases) -> palette -> continuous scales.
    """

    rng = random.Random(seed * 2654435761 + 12345)

    base_choice: BaseChoice = rng.choice(_BASE_CHOICES)
    optics_choice: OpticsChoice = rng.choice(_OPTICS_CHOICES)
    barrel_count = rng.choices(_BARREL_COUNTS, weights=_BARREL_WEIGHTS, k=1)[0]
    if base_choice in _LEG_TYPE_BASES:
        leg_count = rng.choices(_LEG_COUNTS, weights=_LEG_WEIGHTS, k=1)[0]
    else:
        leg_count = 4  # ignored for non-leg bases

    palette_style: PaletteStyle = rng.choice(_PALETTE_STYLES)

    pitch_range_scale = round(rng.uniform(0.85, 1.15), 4)
    recoil_travel_scale = round(rng.uniform(0.85, 1.15), 4)
    barrel_len_scale = round(rng.uniform(0.90, 1.12), 4)
    support_scale = round(rng.uniform(0.92, 1.10), 4)
    footprint_scale = round(rng.uniform(0.90, 1.15), 4)

    return SentryTurretConfig(
        base_choice=base_choice,
        barrel_count=barrel_count,
        optics_choice=optics_choice,
        leg_count=leg_count,
        palette_style=palette_style,
        pitch_range_scale=pitch_range_scale,
        recoil_travel_scale=recoil_travel_scale,
        barrel_len_scale=barrel_len_scale,
        support_scale=support_scale,
        footprint_scale=footprint_scale,
        name=f"seeded_sentry_turret_{seed}",
    )


def _barrel_offsets_for_n(n: int) -> tuple[tuple[float, float], ...]:
    """Procedurally place N barrel bores within the receiver front wall.

    The receiver front wall admits offsets of radius <= _RECV_HALF - _BORE_R in
    both y and z. N=1 centered; N=2 horizontal pair; N=4 the parent 2x2 grid;
    N=3/5/6 symmetric rows kept inside the wall.
    """

    dy = _BARREL_DY
    dz = _BARREL_DZ
    if n == 1:
        return ((0.0, 0.0),)
    if n == 2:
        return ((-dy, 0.0), (dy, 0.0))
    if n == 3:
        # one centered top, two lower outboard (triangular)
        return ((0.0, dz), (-dy, -dz), (dy, -dz))
    if n == 4:
        return ((-dy, dz), (dy, dz), (-dy, -dz), (dy, -dz))
    if n == 5:
        # 2x2 grid + center
        return ((-dy, dz), (dy, dz), (-dy, -dz), (dy, -dz), (0.0, 0.0))
    # n == 6: two columns of three (3 rows of 2)
    return (
        (-dy, dz),
        (dy, dz),
        (-dy, 0.0),
        (dy, 0.0),
        (-dy, -dz),
        (dy, -dz),
    )


def _carriage_x_span(recoil_travel: float) -> tuple[float, float]:
    """Recoil-carriage x extent (cluster frame), derived from the recoil travel.

    Single source (Contract 3c) shared by ``_build_barrel_cluster`` and
    ``run_sentry_turret_tests``: at full recoil the carriage rear face still
    clears the receiver's inner rear wall by >= ``_MOTION_CLEARANCE`` so the
    telescoping carriage never punches through the shell. The span always
    straddles x=0 so the recoil joint origin lies on real carriage geometry
    (recoil_travel <= 0.075 keeps x0 <= -0.013).
    """
    x0 = _RECV_INNER_REAR_X + recoil_travel + _MOTION_CLEARANCE
    return x0, x0 + _CARRIAGE_LEN


def _breech_x0(recoil_travel: float) -> float:
    """Barrel breech (rear end) x at rest, derived from the recoil travel.

    Single source (Contract 3c): at full recoil the breech still clears the
    on-axis pitch hardware (receiver trunnion hub / collar bearing cross-shaft,
    both spanning |x| <= radius) by >= ``_MOTION_CLEARANCE`` so barrels never
    retract through the trunnion shaft.
    """
    return max(_TRUNNION_HUB_R, _BEARING_SHAFT_R) + recoil_travel + _MOTION_CLEARANCE


def _pivot_z_local(inverted: bool) -> float:
    """Pitch-axis offset from the collar frame (single source for the collar
    builder, the pitch joint origin, and the ceiling clearance model). The
    inverted collar hangs the receiver deeper so its pitching top face keeps a
    useful clearance to the hex plate / cheek flanges above it."""
    return _PIVOT_Z_LOCAL_INV if inverted else _PIVOT_Z_LOCAL


def _cheek_z1(inverted: bool) -> float:
    """Trunnion cheek extent beyond the collar frame: keeps the upright 0.05
    overhang past the pitch axis (single source for the collar builder and the
    ceiling clearance model)."""
    return _pivot_z_local(inverted) + (_CHEEK_Z1 - _PIVOT_Z_LOCAL)


def _sensor_x_center(inverted: bool) -> float:
    """Laser pod x center: pushed forward on the inverted ceiling mount so the
    bracket's rear corners swing clear of the cheek prisms (which sit ABOVE the
    pivot when inverted) at full up-pitch. Single source for the optics builder
    and the ceiling clearance model."""
    return _SENSOR_X_CENTER_INV if inverted else _SENSOR_X_CENTER


def _ceiling_underside_discs() -> tuple[tuple[float, float], ...]:
    """(radius, underside_z) of every surface hanging above the pitching
    subtree on the ceiling mount (world frame): plate, bolt-head ring,
    perforated shroud, mounting boss, the inverted hex collar and its cheek
    flanges (modelled as an axis-centred disc of the flange x half-extent,
    conservative in y)."""
    return (
        (_CEIL_PLATE_R, _CEIL_PLATE_BOTTOM),
        (_CEIL_BOLT_CIRCLE_R + _CEIL_BOLT_HEAD_R, _CEIL_PLATE_BOTTOM - _CEIL_BOLT_HEAD_H),
        (_CEIL_SHROUD_R_OUT, _CEIL_SHROUD_BOT),
        (_CEIL_BOSS_R, _CEIL_BOSS_BOTTOM),
        (_HEX_CIRCUM_R, _CEIL_YAW_Z - _HEX_H),
        (0.5 * _CHEEK_X, _CEIL_YAW_Z + _CHEEK_FLANGE_Z_INV - 0.5 * _CHEEK_FLANGE_H),
    )


def _ceiling_pitch_clearance_limit(
    *,
    barrel_offsets: tuple[tuple[float, float], ...],
    barrel_x0: float,
    barrel_x1: float,
    shroud_x0: float,
    cap_x0: float,
    recoil_travel: float,
    optics_choice: OpticsChoice,
) -> float:
    """Max ceiling-mount pitch (rad) at which the pitching subtree keeps
    >= ``_MOTION_CLEARANCE`` below every base/collar underside disc, across the
    full recoil range.

    The pitching envelope is sampled as receiver-frame points (x, |y|, z_top):
    the top surface line of the highest barrel row (shroud/cap outer radius
    where those sleeves can sit, incl. recoiled positions), the receiver top
    face, the camera sight top, and the laser pod if fitted. A pitched point at
    joint angle q maps to world z = pivot_z + x*sin(q) + z*cos(q) and radial
    distance hypot(x*cos(q) - z*sin(q), y); it must not sit under any disc
    (radial <= R + 8 mm) while closer than the clearance to that underside.
    The limit is found by bisection (clearance violation is monotone in q over
    [0, 70 deg] for this forward/upward geometry).
    """
    dz_max = max(dz for _dy, dz in barrel_offsets)
    x_tip = max(barrel_x1, cap_x0 + _CAP_LEN)
    x_lo = barrel_x0 - recoil_travel
    shroud_lo = shroud_x0 - recoil_travel  # recoiled shroud rear (widest sleeve)
    pts: list[tuple[float, float, float]] = []

    def _top_line(x_a: float, x_b: float, py: float, pz) -> None:
        n = max(2, int(math.ceil((x_b - x_a) / 0.005)))
        for i in range(n + 1):
            x = x_a + (x_b - x_a) * i / n
            pts.append((x, py, pz(x) if callable(pz) else pz))

    _top_line(
        x_lo, x_tip, 0.0, lambda x: dz_max + (_CAP_R_OUT if x >= shroud_lo else _BARREL_R_OUT)
    )
    _top_line(_RECV_X0, _RECV_X1, 0.0, _RECV_HALF_Z)
    cheek_pts: list[tuple[float, float]] = []
    if optics_choice == "camera":
        sight_top = _RECV_HALF_Z + _SIGHT_BASE_H + _SIGHT_BODY_H + _SIGHT_VISOR_H
        _top_line(_SIGHT_X0, _SIGHT_X0 + _SIGHT_BODY_L + _SIGHT_VISOR_L, 0.0, sight_top)
    elif optics_choice == "laser_pod":
        pod_xc = _sensor_x_center(True)
        pod_top = _SENSOR_Z + _SENSOR_CAP_R
        _top_line(
            pod_xc - 0.5 * _SENSOR_LEN - _SENSOR_CAP_T,
            pod_xc + 0.5 * _SENSOR_LEN + _SENSOR_LENS_T,
            _SENSOR_Y,
            pod_top,
        )
        # The bracket's y span (receiver wall .. pod) crosses the cheek prisms'
        # y band, so its corners must also swing clear of the cheek x/z extent.
        for px in (pod_xc - 0.5 * _POD_BRACKET_L, pod_xc + 0.5 * _POD_BRACKET_L):
            for pz in (_SENSOR_Z - 0.5 * _POD_BRACKET_H, _SENSOR_Z + 0.5 * _POD_BRACKET_H):
                cheek_pts.append((px, pz))

    pivot_z = _CEIL_YAW_Z - _pivot_z_local(True)
    discs = _ceiling_underside_discs()
    cheek_x_reach = 0.5 * _CHEEK_X + 0.008
    cheek_bot_w = _CEIL_YAW_Z - _cheek_z1(True)  # lowest cheek face (world z)

    def _clears(q: float) -> bool:
        s, c = math.sin(q), math.cos(q)
        for px, py, pz in pts:
            wz = pivot_z + px * s + pz * c
            wr = math.hypot(px * c - pz * s, py)
            for disc_r, under_z in discs:
                if wr <= disc_r + 0.008 and wz > under_z - _MOTION_CLEARANCE:
                    return False
        for px, pz in cheek_pts:
            wz = pivot_z + px * s + pz * c
            wx = abs(px * c - pz * s)
            if wx <= cheek_x_reach and wz > cheek_bot_w - _MOTION_CLEARANCE:
                return False
        return True

    hi = math.radians(70.0)
    if _clears(hi):
        return hi
    lo = 0.0
    for _ in range(48):
        mid = 0.5 * (lo + hi)
        if _clears(mid):
            lo = mid
        else:
            hi = mid
    return lo


def resolve_config(config: SentryTurretConfig) -> ResolvedSentryTurretConfig:
    """Validate enums, clamp scales, derive placement + interface heights."""

    if config.base_choice not in _BASE_CHOICES:
        raise ValueError(f"Unsupported base_choice: {config.base_choice}")
    if config.optics_choice not in _OPTICS_CHOICES:
        raise ValueError(f"Unsupported optics_choice: {config.optics_choice}")
    if config.palette_style not in PALETTES:
        raise ValueError(f"Unsupported palette_style: {config.palette_style}")

    barrel_count = int(config.barrel_count)
    barrel_count = max(1, min(6, barrel_count))

    leg_count = int(config.leg_count)
    if leg_count not in (3, 4, 6):
        leg_count = 4

    support_scale = _clamp(config.support_scale, 0.92, 1.10)
    footprint_scale = _clamp(config.footprint_scale, 0.90, 1.15)
    barrel_len_scale = _clamp(config.barrel_len_scale, 0.90, 1.12)

    # Pitch: lower fixed at -10 deg; upper scaled and clamped to [+45, +70].
    pitch_upper = _clamp(
        _PITCH_HI * _clamp(config.pitch_range_scale, 0.85, 1.15),
        math.radians(45.0),
        math.radians(70.0),
    )

    # Recoil travel scaled + clamped; carriage retention checked structurally.
    recoil_travel = _clamp(
        _RECOIL_TRAVEL * _clamp(config.recoil_travel_scale, 0.85, 1.15), 0.045, 0.075
    )

    # Barrel length scaling: shift the forward tip and the shroud anchor so bore
    # engagement (barrel front - receiver front wall) stays >= 0.10 m. The cap is
    # anchored to the shroud's forward end (overlapping it) so the cap always
    # stays connected to the cluster regardless of scale.
    barrel_x1 = _BARREL_X1 * barrel_len_scale
    shroud_x0 = _SHROUD_X0 * barrel_len_scale
    cap_x0 = shroud_x0 + _SHROUD_LEN - 0.010  # overlap the shroud front by 10 mm

    barrel_offsets = _barrel_offsets_for_n(barrel_count)
    # Breech x derived from recoil travel so recoiled barrels clear the on-axis
    # trunnion hardware (single-sourced with the cluster builder).
    barrel_x0 = _breech_x0(recoil_travel)

    inverted = config.base_choice == "ceiling_mount"
    if inverted:
        yaw_z = _CEIL_YAW_Z  # ceiling boss underside (fixed mounting height)
        # Cap up-pitch at the geometry-derived clearance angle so the pitching
        # subtree never sweeps into the plate/boss/shroud/collar undersides.
        pitch_upper = min(
            pitch_upper,
            _ceiling_pitch_clearance_limit(
                barrel_offsets=barrel_offsets,
                barrel_x0=barrel_x0,
                barrel_x1=barrel_x1,
                shroud_x0=shroud_x0,
                cap_x0=cap_x0,
                recoil_travel=recoil_travel,
                optics_choice=config.optics_choice,
            ),
        )
    elif config.base_choice == "solid_pedestal":
        yaw_z = _PED_TOP_Z  # drum top flange is fixed geometry, not support-scaled
    else:
        # leg base platform top rises with support_scale (legs are scaled to match)
        yaw_z = _LEG_TOP_Z * support_scale

    return ResolvedSentryTurretConfig(
        base_choice=config.base_choice,
        barrel_count=barrel_count,
        optics_choice=config.optics_choice,
        leg_count=leg_count,
        palette_style=config.palette_style,
        pitch_upper=pitch_upper,
        recoil_travel=recoil_travel,
        barrel_len_scale=barrel_len_scale,
        support_scale=support_scale,
        footprint_scale=footprint_scale,
        barrel_offsets=barrel_offsets,
        barrel_x0=barrel_x0,
        barrel_x1=barrel_x1,
        shroud_x0=shroud_x0,
        cap_x0=cap_x0,
        yaw_z=yaw_z,
        inverted=inverted,
        name=config.name,
    )


def _leg_angles(n: int) -> tuple[float, ...]:
    return tuple(round(k * 360.0 / n, 6) for k in range(n))


# --------------------------------------------------------------------------- #
# Cadquery geometry helpers (preserve source primitive sophistication)
# --------------------------------------------------------------------------- #


def _rect_tube(length: float, w: float, t: float, wall: float) -> cq.Workplane:
    return cq.Workplane("XY").rect(t, w).rect(t - 2.0 * wall, w - 2.0 * wall).extrude(length)


def _segment_tube(p0, p1, ext0: float, ext1: float) -> cq.Workplane:
    dx, dz = p1[0] - p0[0], p1[1] - p0[1]
    seg_len = math.hypot(dx, dz)
    ux, uz = dx / seg_len, dz / seg_len
    tube = _rect_tube(seg_len + ext0 + ext1, _LEG_W, _LEG_T, _LEG_WALL)
    ang_deg = math.degrees(math.atan2(ux, uz))
    start = (p0[0] - ux * ext0, 0.0, p0[1] - uz * ext0)
    return tube.rotate((0, 0, 0), (0, 1, 0), ang_deg).translate(start)


def _round_tube(r_out: float, r_in: float, length: float) -> cq.Workplane:
    tube = cq.Workplane("XY").circle(r_out).circle(r_in).extrude(length)
    return tube.rotate((0, 0, 0), (0, 1, 0), 90.0)


def _perforated_shell() -> cq.Workplane:
    h = _SHELL_Z1 - _SHELL_Z0
    shell = cq.Workplane("XY").circle(_SHELL_R_OUT).circle(_SHELL_R_IN).extrude(h)
    cutters = []
    hole = 0.024
    rows = [0.045 + i * 0.055 for i in range(6)]
    for i, z in enumerate(rows):
        offset = 10.0 if i % 2 else 0.0
        for k in range(9):
            ang = offset + 20.0 * k
            cutter = (
                cq.Workplane("XY")
                .box(2.0 * _SHELL_R_OUT + 0.05, hole, hole)
                .translate((0.0, 0.0, z))
                .rotate((0, 0, 0), (0, 0, 1), ang)
            )
            cutters.append(cutter.val())
    return shell.cut(cq.Compound.makeCompound(cutters))


def _pedestal_drum() -> cq.Workplane:
    profile = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0)
        .lineTo(_PED_BASE_PLATE_R, 0.0)
        .lineTo(_PED_BASE_PLATE_R, _PED_BASE_PLATE_H)
        .lineTo(_PED_SHAFT_R + 0.02, _PED_BASE_PLATE_H + 0.025)
        .lineTo(_PED_SHAFT_R, _PED_BASE_PLATE_H + 0.06)
        .lineTo(_PED_SHAFT_R, _PED_TOP_Z - _PED_TOP_FLANGE_H)
        .lineTo(_PED_TOP_FLANGE_R, _PED_TOP_Z - _PED_TOP_FLANGE_H + 0.02)
        .lineTo(_PED_TOP_FLANGE_R, _PED_TOP_Z)
        .lineTo(0.0, _PED_TOP_Z)
        .close()
    )
    return profile.revolve(360, (0, 0), (0, 1))


def _ceiling_plate_mesh() -> cq.Workplane:
    plate = cq.Workplane("XY").circle(_CEIL_PLATE_R).extrude(_CEIL_PLATE_H)
    plate = plate.cut(cq.Workplane("XY").circle(0.035).extrude(_CEIL_PLATE_H))
    for i in range(_CEIL_N_BOLTS):
        ang = 2.0 * math.pi * i / _CEIL_N_BOLTS
        x = _CEIL_BOLT_CIRCLE_R * math.cos(ang)
        y = _CEIL_BOLT_CIRCLE_R * math.sin(ang)
        hole = (
            cq.Workplane("XY").circle(_CEIL_BOLT_HOLE_R).extrude(_CEIL_PLATE_H).translate((x, y, 0))
        )
        plate = plate.cut(hole)
    return plate


def _boss_shroud_mesh() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .circle(_CEIL_SHROUD_R_OUT)
        .circle(_CEIL_SHROUD_R_IN)
        .extrude(_CEIL_SHROUD_H)
    )
    cutters = []
    hole = 0.016
    rows = [0.006 + i * 0.014 for i in range(3)]
    for i, z in enumerate(rows):
        offset = 8.0 if i % 2 else 0.0
        for k in range(12):
            ang = offset + 15.0 * k
            cutter = (
                cq.Workplane("XY")
                .box(2.0 * _CEIL_SHROUD_R_OUT + 0.04, hole, hole)
                .translate((0.0, 0.0, z))
                .rotate((0, 0, 0), (0, 0, 1), ang)
            )
            cutters.append(cutter.val())
    return shell.cut(cq.Compound.makeCompound(cutters))


def _shroud_ring_mesh() -> cq.Workplane:
    return cq.Workplane("XY").circle(_CEIL_SHROUD_R_OUT).circle(_CEIL_BOSS_R - 0.003).extrude(0.006)


def _receiver_shell(r: ResolvedSentryTurretConfig) -> cq.Workplane:
    """Hollow boxy receiver with one barrel bore per offset + 2 top pockets."""
    depth = _RECV_X1 - _RECV_X0
    cx = 0.5 * (_RECV_X0 + _RECV_X1)
    shell = (
        cq.Workplane("XY")
        .box(depth, 2.0 * _RECV_HALF_Y, 2.0 * _RECV_HALF_Z)
        .translate((cx, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.012)
    )
    cavity = (
        cq.Workplane("XY")
        .box(
            depth - 2.0 * _RECV_WALL,
            2.0 * (_RECV_HALF_Y - _RECV_WALL),
            2.0 * (_RECV_HALF_Z - _RECV_WALL),
        )
        .translate((cx, 0.0, 0.0))
    )
    shell = shell.cut(cavity)
    for dy, dz in r.barrel_offsets:
        bore = (
            cq.Workplane("XY")
            .circle(_BORE_R)
            .extrude(0.10)
            .rotate((0, 0, 0), (0, 1, 0), 90.0)
            .translate((_RECV_X1 - 0.05, dy, dz))
        )
        shell = shell.cut(bore)
    for sy in (1.0, -1.0):
        pocket = (
            cq.Workplane("XY")
            .box(0.15, 0.115, 0.02)
            .translate((0.07, sy * 0.0725, _RECV_HALF_Z + 0.002))
        )
        shell = shell.cut(pocket)
    return shell


def _camera_sight_body() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(_SIGHT_BASE_L, _SIGHT_BASE_W, _SIGHT_BASE_H)
        .translate((_SIGHT_BODY_L / 2.0, 0.0, _SIGHT_BASE_H / 2.0))
    )
    body_solid = (
        cq.Workplane("XY")
        .box(_SIGHT_BODY_L, _SIGHT_BODY_W, _SIGHT_BODY_H)
        .edges("|Z")
        .fillet(0.004)
    )
    body = body_solid.translate((_SIGHT_BODY_L / 2.0, 0.0, _SIGHT_BASE_H + _SIGHT_BODY_H / 2.0))
    result = base.union(body)
    lens_cutter = (
        cq.Workplane("YZ")
        .workplane(offset=_SIGHT_BODY_L - _SIGHT_LENS_DEPTH)
        .circle(_SIGHT_LENS_R + 0.001)
        .extrude(_SIGHT_LENS_DEPTH + 0.002)
    )
    result = result.cut(lens_cutter)
    visor = (
        cq.Workplane("XY")
        .box(_SIGHT_VISOR_L, _SIGHT_VISOR_W, _SIGHT_VISOR_H)
        .translate(
            (
                _SIGHT_BODY_L + _SIGHT_VISOR_L / 2.0 - 0.005,
                0.0,
                _SIGHT_BASE_H + _SIGHT_BODY_H + _SIGHT_VISOR_H / 2.0,
            )
        )
    )
    result = result.union(visor)
    gland = (
        cq.Workplane("YZ")
        .workplane(offset=-0.005)
        .circle(0.008)
        .extrude(0.008)
        .translate((0.0, 0.0, _SIGHT_BASE_H + _SIGHT_BODY_H * 0.4))
    )
    result = result.union(gland)
    return result


# --------------------------------------------------------------------------- #
# Slot A — base modules (root + yaw stage carrier)
# --------------------------------------------------------------------------- #


def _build_leg_base(base, r: ResolvedSentryTurretConfig, *, mats, assets) -> None:
    """Splayed multi-leg base (splayed_quad_legs / tripod_3legs).

    Adopted: S1 parent leg loop (L186-261) + S2/S3 LEG_ANGLES (leg-count axis).
    ``support_scale`` lifts the platform/column/leg-roots so the yaw plane
    rises with the chain; ``footprint_scale`` widens the foot radius.
    """
    body = mats["body"]
    metal = mats["metal"]
    dark = mats["dark"]
    s = r.support_scale
    fp = r.footprint_scale

    knee_pt = (_KNEE_PT[0] * fp, _KNEE_PT[1] * s)
    foot_pt = (_FOOT_PT[0] * fp, _FOOT_PT[1])
    hub_pt = (_HUB_PT[0], _HUB_PT[1] * s)

    leg_solid = _segment_tube(hub_pt, knee_pt, 0.02, 0.03).union(
        _segment_tube(knee_pt, foot_pt, 0.03, 0.03)
    )
    leg_mesh = mesh_from_cadquery(
        leg_solid, "leg_strut", assets=assets, tolerance=_MESH_TOL, angular_tolerance=_MESH_ANG_TOL
    )
    knee_dir = math.atan2((hub_pt[1] - foot_pt[1]), (foot_pt[0] - hub_pt[0]))

    for i, ang in enumerate(_leg_angles(r.leg_count)):
        yaw = math.radians(ang)
        base.visual(
            leg_mesh, origin=Origin(rpy=(0.0, 0.0, yaw)), material=body, name=f"leg_strut_{i}"
        )
        kx = knee_pt[0] * math.cos(yaw)
        ky = knee_pt[0] * math.sin(yaw)
        base.visual(
            Box((0.15, 0.10, 0.11)),
            origin=Origin(xyz=(kx, ky, knee_pt[1]), rpy=(0.0, knee_dir, yaw)),
            material=dark,
            name=f"knee_gusset_{i}",
        )
        fx = foot_pt[0] * math.cos(yaw)
        fy = foot_pt[0] * math.sin(yaw)
        base.visual(
            Cylinder(radius=_FOOT_PAD_R, length=_FOOT_PAD_H),
            origin=Origin(xyz=(fx, fy, _FOOT_PAD_H / 2.0)),
            material=dark,
            name=f"foot_pad_{i}",
        )

    col_z0, col_z1 = _COLUMN_Z0 * s, _COLUMN_Z1 * s
    base.visual(
        Cylinder(radius=_COLUMN_R, length=col_z1 - col_z0),
        origin=Origin(xyz=(0.0, 0.0, 0.5 * (col_z0 + col_z1))),
        material=metal,
        name="column_core",
    )
    base.visual(
        Cylinder(radius=0.10, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.245 * s)),
        material=metal,
        name="column_base_cap",
    )
    base.visual(
        mesh_from_cadquery(
            _perforated_shell(),
            "perforated_wrap",
            assets=assets,
            tolerance=_MESH_TOL,
            angular_tolerance=_MESH_ANG_TOL,
        ),
        origin=Origin(xyz=(0.0, 0.0, _SHELL_Z0 * s)),
        material=body,
        name="perforated_wrap",
    )
    base.visual(
        Cylinder(radius=0.135, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.685 * s)),
        material=metal,
        name="hub_ring",
    )
    plat_z0, plat_z1 = _PLATFORM_Z0 * s, _PLATFORM_Z1 * s
    base.visual(
        Cylinder(radius=_PLATFORM_R, length=plat_z1 - plat_z0),
        origin=Origin(xyz=(0.0, 0.0, 0.5 * (plat_z0 + plat_z1))),
        material=body,
        name="ring_platform",
    )
    base.visual(
        Cylinder(radius=0.19, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, plat_z1 - 0.0125)),
        material=body,
        name="platform_lip",
    )


def _build_pedestal_base(base, r: ResolvedSentryTurretConfig, *, mats, assets) -> None:
    """Lathe-revolved solid pedestal drum. Adopted: S4 pedestal (L79-187)."""
    body = mats["body"]
    metal = mats["metal"]
    dark = mats["dark"]

    drum = mesh_from_cadquery(
        _pedestal_drum(),
        "pedestal_drum",
        assets=assets,
        tolerance=_MESH_TOL,
        angular_tolerance=_MESH_ANG_TOL,
    )
    base.visual(drum, origin=Origin(xyz=(0.0, 0.0, 0.0)), material=body, name="pedestal_drum")
    band_z = 0.5 * (_PED_BASE_PLATE_H + 0.06 + _PED_TOP_Z - _PED_TOP_FLANGE_H)
    base.visual(
        Cylinder(radius=_PED_SHAFT_BAND_R, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, band_z)),
        material=dark,
        name="shaft_band",
    )
    base.visual(
        Cylinder(radius=_PED_BASE_PLATE_R + 0.005, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark,
        name="base_rim",
    )
    bolt_r = _PED_TOP_FLANGE_R - 0.035
    bolt_z = _PED_TOP_Z - 0.003
    for i in range(_PED_N_BOLTS):
        ang = 2.0 * math.pi * i / _PED_N_BOLTS
        base.visual(
            Cylinder(radius=0.012, length=0.008),
            origin=Origin(xyz=(bolt_r * math.cos(ang), bolt_r * math.sin(ang), bolt_z)),
            material=metal,
            name=f"flange_bolt_{i}",
        )


def _build_ceiling_base(base, r: ResolvedSentryTurretConfig, *, mats, assets) -> None:
    """Inverted ceiling plate + boss + perforated shroud. Adopted: S5 ceiling."""
    body = mats["body"]
    metal = mats["metal"]
    dark = mats["dark"]

    base.visual(
        mesh_from_cadquery(
            _ceiling_plate_mesh(),
            "plate_body",
            assets=assets,
            tolerance=_MESH_TOL,
            angular_tolerance=_MESH_ANG_TOL,
        ),
        origin=Origin(xyz=(0.0, 0.0, _CEIL_PLATE_BOTTOM)),
        material=dark,
        name="plate_body",
    )
    boss_len = _CEIL_BOSS_H + 0.003
    base.visual(
        mesh_from_cadquery(
            cq.Workplane("XY").circle(_CEIL_BOSS_R).extrude(boss_len),
            "mounting_boss",
            assets=assets,
            tolerance=_MESH_TOL,
            angular_tolerance=_MESH_ANG_TOL,
        ),
        origin=Origin(xyz=(0.0, 0.0, _CEIL_BOSS_BOTTOM)),
        material=metal,
        name="mounting_boss",
    )
    base.visual(
        mesh_from_cadquery(
            _boss_shroud_mesh(),
            "boss_shroud",
            assets=assets,
            tolerance=_MESH_TOL,
            angular_tolerance=_MESH_ANG_TOL,
        ),
        origin=Origin(xyz=(0.0, 0.0, _CEIL_SHROUD_BOT)),
        material=body,
        name="boss_shroud",
    )
    ring_mesh = mesh_from_cadquery(
        _shroud_ring_mesh(),
        "shroud_ring",
        assets=assets,
        tolerance=_MESH_TOL,
        angular_tolerance=_MESH_ANG_TOL,
    )
    base.visual(
        ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, _CEIL_SHROUD_TOP - 0.006)),
        material=body,
        name="shroud_ring_top",
    )
    base.visual(
        ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, _CEIL_SHROUD_BOT)),
        material=body,
        name="shroud_ring_bot",
    )
    for i in range(_CEIL_N_BOLTS):
        ang = 2.0 * math.pi * i / _CEIL_N_BOLTS
        x = _CEIL_BOLT_CIRCLE_R * math.cos(ang)
        y = _CEIL_BOLT_CIRCLE_R * math.sin(ang)
        base.visual(
            Cylinder(radius=_CEIL_BOLT_HEAD_R, length=_CEIL_BOLT_HEAD_H),
            origin=Origin(xyz=(x, y, _CEIL_PLATE_BOTTOM - 0.5 * _CEIL_BOLT_HEAD_H)),
            material=metal,
            name=f"bolt_head_{i}",
        )


# --------------------------------------------------------------------------- #
# Support collar (fixed structural layer, shared; inverts for ceiling)
# --------------------------------------------------------------------------- #


def _build_collar(collar, r: ResolvedSentryTurretConfig, *, mats, assets) -> None:
    body = mats["body"]
    metal = mats["metal"]
    inv = -1.0 if r.inverted else 1.0

    if r.inverted:
        hex_solid = (
            cq.Workplane("XY")
            .polygon(6, 2.0 * _HEX_CIRCUM_R)
            .extrude(_HEX_H)
            .translate((0, 0, -_HEX_H))
        )
    else:
        hex_solid = cq.Workplane("XY").polygon(6, 2.0 * _HEX_CIRCUM_R).extrude(_HEX_H)
    collar.visual(
        mesh_from_cadquery(
            hex_solid,
            "hex_collar",
            assets=assets,
            tolerance=_MESH_TOL,
            angular_tolerance=_MESH_ANG_TOL,
        ),
        material=metal,
        name="hex_collar",
    )

    yc = 0.5 * (_CHEEK_Y_IN + _CHEEK_Y_OUT)
    pivot = _pivot_z_local(r.inverted)
    # Cheeks keep the upright 0.05 overhang beyond the pitch axis; the inverted
    # collar's deeper pivot (_PIVOT_Z_LOCAL_INV) stretches them to match.
    cheek_z1 = _cheek_z1(r.inverted)
    cheek_h = cheek_z1 - _CHEEK_Z0
    if r.inverted:
        cheek_zc = -0.5 * (_CHEEK_Z0 + cheek_z1)
        flange_z = _CHEEK_FLANGE_Z_INV
        disc_z = -pivot
    else:
        cheek_zc = 0.5 * (_CHEEK_Z0 + cheek_z1)
        flange_z = 0.06
        disc_z = pivot

    for i, sy in enumerate((1.0, -1.0)):
        collar.visual(
            Box((_CHEEK_X, _CHEEK_Y_OUT - _CHEEK_Y_IN, cheek_h)),
            origin=Origin(xyz=(0.0, sy * yc, cheek_zc)),
            material=body,
            name=f"trunnion_cheek_{i}",
        )
        collar.visual(
            Box((_CHEEK_X, 0.085, _CHEEK_FLANGE_H)),
            origin=Origin(xyz=(0.0, sy * 0.1425, flange_z)),
            material=body,
            name=f"cheek_flange_{i}",
        )
        collar.visual(
            Cylinder(radius=0.055, length=0.022),
            origin=Origin(xyz=(0.0, sy * 0.193, disc_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"trunnion_disc_{i}",
        )
    # Bearing cross-shaft spanning the two cheeks ON the pitch axis. It places
    # real collar geometry at the pitch joint origin (0, 0, disc_z) so
    # fail_if_articulation_origin_far_from_geometry is satisfied; the receiver's
    # trunnion hub rides this shaft (captured-pin, whitelisted in run_tests).
    collar.visual(
        Cylinder(radius=_BEARING_SHAFT_R, length=2.0 * _CHEEK_Y_OUT),
        origin=Origin(xyz=(0.0, 0.0, disc_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="trunnion_bearing_shaft",
    )
    _ = inv  # documented sign convention; disc_z already carries inversion


# --------------------------------------------------------------------------- #
# Receiver (+ optics subtree) and barrel cluster
# --------------------------------------------------------------------------- #


def _build_receiver(receiver, r: ResolvedSentryTurretConfig, *, mats, assets) -> None:
    body = mats["body"]
    metal = mats["metal"]

    receiver.visual(
        mesh_from_cadquery(
            _receiver_shell(r),
            "receiver_shell",
            assets=assets,
            tolerance=_MESH_TOL,
            angular_tolerance=_MESH_ANG_TOL,
        ),
        material=body,
        name="receiver_shell",
    )
    # Transverse trunnion hub through the receiver center ON the pitch axis. It
    # places real receiver geometry at the receiver part-frame origin (0,0,0) so
    # both the pitch (child) and recoil (parent) joint origins lie on real
    # hardware; it rides the collar bearing shaft (captured-pin, whitelisted).
    receiver.visual(
        Cylinder(radius=_TRUNNION_HUB_R, length=2.0 * _RECV_HALF_Y),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="trunnion_hub",
    )
    for i, sy in enumerate((1.0, -1.0)):
        receiver.visual(
            Box((0.146, 0.111, 0.006)),
            origin=Origin(xyz=(0.07, sy * 0.0725, _RECV_HALF_Z - 0.0055)),
            material=metal,
            name=f"top_panel_{i}",
        )
        receiver.visual(
            Cylinder(radius=0.032, length=0.06),
            origin=Origin(xyz=(0.0, sy * 0.17, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"trunnion_pin_{i}",
        )

    if r.optics_choice == "camera":
        _build_camera_optics(receiver, r, mats=mats, assets=assets)
    elif r.optics_choice == "laser_pod":
        _build_laser_optics(receiver, r, mats=mats)


def _build_camera_optics(receiver, r: ResolvedSentryTurretConfig, *, mats, assets) -> None:
    """Targeting camera sight on the receiver top deck. Adopted: S9 cameraoptic.

    On ceiling mounts the sight top is part of the pitch clearance envelope
    (see _ceiling_pitch_clearance_limit), so it never sweeps into the collar.
    """
    camera_mat = mats["camera"]
    lens_mat = mats["camera_lens"]
    led_mat = mats["camera_led"]

    sight_x = _SIGHT_X0
    sight_z = _RECV_HALF_Z
    receiver.visual(
        mesh_from_cadquery(
            _camera_sight_body(),
            "sight_body",
            assets=assets,
            tolerance=_MESH_TOL,
            angular_tolerance=_MESH_ANG_TOL,
        ),
        origin=Origin(xyz=(sight_x, 0.0, sight_z)),
        material=camera_mat,
        name="sight_body",
    )
    lens_len = 0.004
    receiver.visual(
        Cylinder(radius=_SIGHT_LENS_R + 0.004, length=lens_len),
        origin=Origin(
            xyz=(
                sight_x + _SIGHT_BODY_L + lens_len / 2.0,
                0.0,
                sight_z + _SIGHT_BASE_H + _SIGHT_BODY_H / 2.0,
            ),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=lens_mat,
        name="sight_lens",
    )
    receiver.visual(
        Cylinder(radius=0.003, length=0.002),
        origin=Origin(
            xyz=(
                sight_x + _SIGHT_BODY_L * 0.65,
                _SIGHT_BODY_W / 2.0 + 0.001,
                sight_z + _SIGHT_BASE_H + _SIGHT_BODY_H * 0.7,
            ),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=led_mat,
        name="sight_led",
    )


def _build_laser_optics(receiver, r: ResolvedSentryTurretConfig, *, mats) -> None:
    """Side laser/sensor pod bridged to the receiver sidewall. Adopted: S10 laserpod."""
    metal = mats["metal"]
    dark = mats["dark"]
    lens_mat = mats["laser_lens"]

    pod_xc = _sensor_x_center(r.inverted)
    sensor_front_x = pod_xc + 0.5 * _SENSOR_LEN
    sensor_rear_x = pod_xc - 0.5 * _SENSOR_LEN
    receiver.visual(
        Cylinder(radius=_SENSOR_R, length=_SENSOR_LEN),
        origin=Origin(xyz=(pod_xc, _SENSOR_Y, _SENSOR_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="sensor_pod_body",
    )
    receiver.visual(
        Cylinder(radius=_SENSOR_LENS_R, length=_SENSOR_LENS_T),
        origin=Origin(
            xyz=(sensor_front_x + 0.5 * _SENSOR_LENS_T, _SENSOR_Y, _SENSOR_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=lens_mat,
        name="sensor_lens",
    )
    receiver.visual(
        Cylinder(radius=_SENSOR_CAP_R, length=_SENSOR_CAP_T),
        origin=Origin(
            xyz=(sensor_rear_x - 0.5 * _SENSOR_CAP_T, _SENSOR_Y, _SENSOR_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=metal,
        name="sensor_rear_cap",
    )
    bracket_y_center = 0.5 * (_RECV_HALF_Y + _SENSOR_Y)
    bracket_y_span = _SENSOR_Y - _RECV_HALF_Y
    receiver.visual(
        Box((_POD_BRACKET_L, bracket_y_span, _POD_BRACKET_H)),
        origin=Origin(xyz=(pod_xc, bracket_y_center, _SENSOR_Z)),
        material=metal,
        name="sensor_mount_bracket",
    )


def _build_barrel_cluster(cluster, r: ResolvedSentryTurretConfig, *, mats, assets) -> None:
    """N-barrel cluster via a single offset-list loop. Adopted: S6 single1
    copy-logic + S7/S8 placement. One shared recoil prismatic upstream."""
    body = mats["body"]
    metal = mats["metal"]
    barrel = mats["barrel"]

    # Carriage sized to enclose the offset cluster (snug for N=1, full for
    # grids). Its x-span is recoil-derived (_carriage_x_span): it straddles the
    # cluster part-frame origin so the recoil joint origin lies on real carriage
    # geometry, and at full recoil its rear face still clears the receiver's
    # inner rear wall by _MOTION_CLEARANCE (no punch-through).
    carriage_x0, carriage_x1 = _carriage_x_span(r.recoil_travel)
    if r.barrel_count == 1:
        carriage = Box((_CARRIAGE_LEN, 0.12, 0.12))
    else:
        carriage = Box((_CARRIAGE_LEN, 0.24, 0.20))
    cluster.visual(
        carriage,
        origin=Origin(xyz=(0.5 * (carriage_x0 + carriage_x1), 0.0, 0.0)),
        material=metal,
        name="carriage",
    )

    barrel_mesh = mesh_from_cadquery(
        _round_tube(_BARREL_R_OUT, _BARREL_R_IN, r.barrel_x1 - r.barrel_x0),
        "barrel_tube",
        assets=assets,
        tolerance=_MESH_TOL,
        angular_tolerance=_MESH_ANG_TOL,
    )
    shroud_mesh = mesh_from_cadquery(
        _round_tube(_SHROUD_R_OUT, _SHROUD_R_IN, _SHROUD_LEN),
        "muzzle_shroud",
        assets=assets,
        tolerance=_MESH_TOL,
        angular_tolerance=_MESH_ANG_TOL,
    )
    cap_mesh = mesh_from_cadquery(
        _round_tube(_CAP_R_OUT, _CAP_R_IN, _CAP_LEN),
        "muzzle_cap",
        assets=assets,
        tolerance=_MESH_TOL,
        angular_tolerance=_MESH_ANG_TOL,
    )

    for i, (dy, dz) in enumerate(r.barrel_offsets):
        cluster.visual(
            barrel_mesh,
            origin=Origin(xyz=(r.barrel_x0, dy, dz)),
            material=barrel,
            name=f"barrel_{i}",
        )
        cluster.visual(
            shroud_mesh,
            origin=Origin(xyz=(r.shroud_x0, dy, dz)),
            material=body,
            name=f"muzzle_shroud_{i}",
        )
        cluster.visual(
            cap_mesh, origin=Origin(xyz=(r.cap_x0, dy, dz)), material=metal, name=f"muzzle_cap_{i}"
        )


# --------------------------------------------------------------------------- #
# Assembly
# --------------------------------------------------------------------------- #

_BASE_BUILDERS = {
    "splayed_quad_legs": _build_leg_base,
    "tripod_3legs": _build_leg_base,
    "solid_pedestal": _build_pedestal_base,
    "ceiling_mount": _build_ceiling_base,
}
_BASE_PART_NAMES = {
    "splayed_quad_legs": "leg_base",
    "tripod_3legs": "leg_base",
    "solid_pedestal": "pedestal",
    "ceiling_mount": "ceiling_plate",
}
_YAW_JOINT_NAMES = {
    "splayed_quad_legs": "base_to_collar_yaw",
    "tripod_3legs": "base_to_collar_yaw",
    "solid_pedestal": "pedestal_to_collar_yaw",
    "ceiling_mount": "plate_to_collar_yaw",
}


def build_sentry_turret(
    config: SentryTurretConfig | None = None,
    *,
    assets: AssetContext | None = None,
) -> ArticulatedObject:
    config = config or SentryTurretConfig()
    r = resolve_config(config)
    if assets is None:
        assets = AssetContext(Path(tempfile.mkdtemp(prefix="articraft-sentry-turret-")))
    model = ArticulatedObject(name=r.name, assets=assets)

    palette = PALETTES[r.palette_style]
    mats = {
        "body": model.material("turret_body", rgba=palette["body"]),
        "metal": model.material("turret_metal", rgba=palette["metal"]),
        "dark": model.material("turret_dark", rgba=palette["dark"]),
        "barrel": model.material("turret_barrel", rgba=palette["barrel"]),
        # optics-fixed materials (identity colors, palette-independent)
        "camera": model.material("camera_housing", rgba=_CAMERA_HOUSING),
        "camera_lens": model.material("camera_lens", rgba=_CAMERA_LENS),
        "camera_led": model.material("status_led", rgba=_CAMERA_LED),
        "laser_lens": model.material("laser_lens", rgba=_LASER_LENS),
    }

    # ---- Slot A base (root) ----
    base_part_name = _BASE_PART_NAMES[r.base_choice]
    base = model.part(base_part_name)
    _BASE_BUILDERS[r.base_choice](base, r, mats=mats, assets=assets)

    # ---- support collar (fixed structural layer) ----
    collar = model.part("support_collar")
    _build_collar(collar, r, mats=mats, assets=assets)

    # ---- receiver (+ optics) ----
    receiver = model.part("receiver")
    _build_receiver(receiver, r, mats=mats, assets=assets)

    # ---- barrel cluster ----
    cluster = model.part("barrel_cluster")
    _build_barrel_cluster(cluster, r, mats=mats, assets=assets)

    # ---- yaw (CONTINUOUS +Z) base -> collar ----
    model.articulation(
        _YAW_JOINT_NAMES[r.base_choice],
        ArticulationType.CONTINUOUS,
        parent=base,
        child=collar,
        origin=Origin(xyz=(0.0, 0.0, r.yaw_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=2.0),
    )

    # ---- pitch (REVOLUTE -Y) collar -> receiver ----
    pivot_abs = _pivot_z_local(r.inverted)
    pivot_local_z = -pivot_abs if r.inverted else pivot_abs
    model.articulation(
        "collar_to_receiver_pitch",
        ArticulationType.REVOLUTE,
        parent=collar,
        child=receiver,
        origin=Origin(xyz=(0.0, 0.0, pivot_local_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.5, lower=_PITCH_LO, upper=r.pitch_upper),
    )

    # ---- recoil (PRISMATIC -X) receiver -> barrels (shared by all N) ----
    model.articulation(
        "receiver_to_barrels_recoil",
        ArticulationType.PRISMATIC,
        parent=receiver,
        child=cluster,
        origin=Origin(),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=1.0, lower=0.0, upper=r.recoil_travel),
    )

    return model


def build_seeded_sentry_turret(
    seed: int, *, assets: AssetContext | None = None
) -> ArticulatedObject:
    return build_sentry_turret(config_from_seed(seed), assets=assets)


# --------------------------------------------------------------------------- #
# Slot choices (module_topology_diversity gate)
# --------------------------------------------------------------------------- #


def slot_choices_for_config(r: ResolvedSentryTurretConfig) -> list[tuple[str, str]]:
    choices: list[tuple[str, str]] = [
        ("base_mount", r.base_choice),
        ("barrel_count", f"{r.barrel_count}_barrel"),
        ("optics", r.optics_choice),
    ]
    if r.base_choice in _LEG_TYPE_BASES:
        choices.append(("leg_count", f"{r.leg_count}_leg"))
    return choices


def slot_choices_for_seed(seed: int) -> list[tuple[str, str]]:
    return slot_choices_for_config(resolve_config(config_from_seed(seed)))


# --------------------------------------------------------------------------- #
# Author tests (run by `articraft external check`, not by the sweep baseline)
# --------------------------------------------------------------------------- #


def run_sentry_turret_tests(
    object_model: ArticulatedObject, config: SentryTurretConfig
) -> TestReport:
    ctx = TestContext(object_model)
    r = resolve_config(config)

    base_name = _BASE_PART_NAMES[r.base_choice]
    base = object_model.get_part(base_name)
    collar = object_model.get_part("support_collar")
    receiver = object_model.get_part("receiver")
    cluster = object_model.get_part("barrel_cluster")
    yaw = object_model.get_articulation(_YAW_JOINT_NAMES[r.base_choice])
    pitch = object_model.get_articulation("collar_to_receiver_pitch")
    recoil = object_model.get_articulation("receiver_to_barrels_recoil")

    # Captured trunnion pins ride inside the collar cheek bearings / discs, and
    # the receiver trunnion hub rides the collar bearing cross-shaft on the
    # pitch axis (the intended pitch bearing contact).
    for i in range(2):
        for cheek_elem in (f"trunnion_cheek_{i}", f"trunnion_disc_{i}"):
            ctx.allow_overlap(
                receiver,
                collar,
                elem_a=f"trunnion_pin_{i}",
                elem_b=cheek_elem,
                reason="trunnion pin is intentionally captured inside the cheek bearing and disc",
            )
        ctx.expect_overlap(
            receiver,
            collar,
            axes="y",
            elem_a=f"trunnion_pin_{i}",
            elem_b=f"trunnion_cheek_{i}",
            min_overlap=0.02,
            name=f"trunnion pin {i} is seated through the cheek bearing",
        )
    ctx.allow_overlap(
        receiver,
        collar,
        elem_a="trunnion_hub",
        elem_b="trunnion_bearing_shaft",
        reason="receiver trunnion hub rides the collar bearing cross-shaft on the pitch axis",
    )
    for cheek_elem in (
        "trunnion_cheek_0",
        "trunnion_cheek_1",
        "trunnion_disc_0",
        "trunnion_disc_1",
    ):
        ctx.allow_overlap(
            receiver,
            collar,
            elem_a="trunnion_hub",
            elem_b=cheek_elem,
            reason="trunnion hub spans into the cheek bearings (captured pitch bearing)",
        )

    # The recoil carriage straddles the receiver trunnion hub and the collar
    # bearing cross-shaft (both lie on the pitch axis the carriage slides along).
    ctx.allow_overlap(
        cluster,
        receiver,
        elem_a="carriage",
        elem_b="trunnion_hub",
        reason="recoil carriage straddles the receiver trunnion hub on the pitch axis",
    )
    ctx.allow_overlap(
        cluster,
        collar,
        elem_a="carriage",
        elem_b="trunnion_bearing_shaft",
        reason="recoil carriage straddles the collar bearing cross-shaft on the pitch axis",
    )
    # The collar bearing cross-shaft passes through the receiver shell (captured
    # pitch bearing through the receiver center).
    ctx.allow_overlap(
        receiver,
        collar,
        elem_a="receiver_shell",
        elem_b="trunnion_bearing_shaft",
        reason="collar bearing cross-shaft passes through the receiver shell on the pitch axis",
    )
    for i in range(2):
        ctx.allow_overlap(
            receiver,
            collar,
            elem_a=f"trunnion_pin_{i}",
            elem_b="trunnion_bearing_shaft",
            reason="receiver trunnion pins flank the collar bearing cross-shaft",
        )

    # Barrels slide through snug receiver front-wall bores (captured slide).
    for i in range(r.barrel_count):
        ctx.allow_overlap(
            cluster,
            receiver,
            elem_a=f"barrel_{i}",
            elem_b="receiver_shell",
            reason="barrel slides through a snug guide bore in the receiver front wall",
        )
        ctx.expect_overlap(
            cluster,
            receiver,
            axes="x",
            elem_a=f"barrel_{i}",
            elem_b="receiver_shell",
            min_overlap=0.10,
            name=f"barrel {i} stays engaged in its receiver guide bore",
        )

    # --- baseline structural / collision gates (run in-test, not just the
    # motion-QC sweep gate) ---
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_joint_mating_has_gap()

    # --- joint plan ---
    ctx.check(
        "yaw joint is continuous about +Z",
        yaw.articulation_type == ArticulationType.CONTINUOUS and tuple(yaw.axis) == (0.0, 0.0, 1.0),
        details=f"type={yaw.articulation_type}, axis={yaw.axis}",
    )
    pl = pitch.motion_limits
    # Upright bases keep the spec range; the inverted ceiling mount caps the
    # up-pitch at the geometry-derived plate-clearance angle (single-sourced
    # via resolve_config / _ceiling_pitch_clearance_limit).
    pitch_upper_floor = math.radians(15.0) if r.inverted else math.radians(44.0)
    ctx.check(
        "pitch joint is revolute about -Y, lower=-10 deg, upper single-sourced",
        pitch.articulation_type == ArticulationType.REVOLUTE
        and tuple(pitch.axis) == (0.0, -1.0, 0.0)
        and pl is not None
        and abs(pl.lower - _PITCH_LO) < 1e-6
        and abs(pl.upper - r.pitch_upper) < 1e-6
        and pitch_upper_floor <= pl.upper <= math.radians(71.0),
        details=f"type={pitch.articulation_type}, axis={pitch.axis}, limits={pl}, "
        f"inverted={r.inverted}",
    )
    rl = recoil.motion_limits
    ctx.check(
        "recoil joint is prismatic along -X",
        recoil.articulation_type == ArticulationType.PRISMATIC
        and tuple(recoil.axis) == (-1.0, 0.0, 0.0)
        and rl is not None
        and rl.lower == 0.0
        and 0.044 <= rl.upper <= 0.076,
        details=f"type={recoil.articulation_type}, axis={recoil.axis}, limits={rl}",
    )

    # --- grounding / mount ---
    base_aabb = ctx.part_world_aabb(base)
    if r.inverted:
        ctx.check(
            "ceiling plate is fixed at the mounting elevation (top near 1.15)",
            base_aabb is not None and base_aabb[1][2] >= _CEIL_PLATE_BOTTOM - 0.001,
            details=f"base aabb={base_aabb}",
        )
    else:
        ctx.check(
            "base rests on the ground plane",
            base_aabb is not None and -0.002 <= base_aabb[0][2] <= 0.006,
            details=f"base aabb={base_aabb}",
        )

    # --- barrel count matches BARREL_OFFSETS ---
    n_present = 0
    for i in range(r.barrel_count):
        if ctx.part_element_world_aabb(cluster, elem=f"barrel_{i}") is not None:
            n_present += 1
    ctx.check(
        "all N barrels present per offset list",
        n_present == r.barrel_count,
        details=f"present={n_present}, expected={r.barrel_count}",
    )

    # --- optics presence / pitching with receiver ---
    if r.optics_choice == "camera":
        ctx.check(
            "camera sight subtree present on receiver",
            ctx.part_element_world_aabb(receiver, elem="sight_body") is not None
            and ctx.part_element_world_aabb(receiver, elem="sight_lens") is not None,
            details="camera optics",
        )
    elif r.optics_choice == "laser_pod":
        ctx.check(
            "laser pod subtree present on receiver",
            ctx.part_element_world_aabb(receiver, elem="sensor_pod_body") is not None
            and ctx.part_element_world_aabb(receiver, elem="sensor_mount_bracket") is not None,
            details="laser optics",
        )

    # --- mechanism pose: positive pitch elevates muzzle ---
    muzzle_rest = ctx.part_element_world_aabb(cluster, elem="muzzle_cap_0")
    with ctx.pose({pitch: min(r.pitch_upper, 1.0)}):
        muzzle_up = ctx.part_element_world_aabb(cluster, elem="muzzle_cap_0")
    ctx.check(
        "positive pitch elevates the muzzles",
        muzzle_rest is not None
        and muzzle_up is not None
        and muzzle_up[0][2] > muzzle_rest[0][2] + 0.15,
        details=f"rest={muzzle_rest}, elevated={muzzle_up}",
    )

    # --- continuous yaw swings off-axis cluster to +Y ---
    cl_rest = ctx.part_world_aabb(cluster)
    with ctx.pose({yaw: math.pi / 2.0}):
        cl_yawed = ctx.part_world_aabb(cluster)
    ctx.check(
        "quarter-turn yaw swings the off-axis barrel cluster to +Y",
        cl_rest is not None
        and cl_yawed is not None
        and cl_rest[1][0] > 0.4
        and cl_yawed[1][1] > 0.4,
        details=f"rest={cl_rest}, yawed={cl_yawed}",
    )

    # --- recoil slides cluster rearward ---
    with ctx.pose({recoil: r.recoil_travel}):
        cl_back = ctx.part_world_aabb(cluster)
        carriage_back = ctx.part_element_world_aabb(cluster, elem="carriage")
        breech_back = ctx.part_element_world_aabb(cluster, elem="barrel_0")
        shell_aabb = ctx.part_element_world_aabb(receiver, elem="receiver_shell")
        hub_aabb = ctx.part_element_world_aabb(receiver, elem="trunnion_hub")
    ctx.check(
        "recoil slides the barrel cluster rearward by recoil_travel",
        cl_rest is not None
        and cl_back is not None
        and abs((cl_rest[1][0] - cl_back[1][0]) - r.recoil_travel) < 0.006,
        details=f"rest={cl_rest}, recoiled={cl_back}",
    )
    # Derived-clearance guards (see _carriage_x_span / _breech_x0): the fully
    # recoiled carriage stays inside the receiver shell, and the breeches stay
    # clear of the on-axis trunnion hardware.
    ctx.check(
        "fully recoiled carriage clears the receiver inner rear wall",
        carriage_back is not None
        and shell_aabb is not None
        and carriage_back[0][0] >= shell_aabb[0][0] + _RECV_WALL + 0.5 * _MOTION_CLEARANCE,
        details=f"carriage={carriage_back}, shell={shell_aabb}",
    )
    ctx.check(
        "fully recoiled breech clears the trunnion hub",
        breech_back is not None
        and hub_aabb is not None
        and breech_back[0][0] >= hub_aabb[1][0] + 0.5 * _MOTION_CLEARANCE,
        details=f"barrel_0={breech_back}, hub={hub_aabb}",
    )

    return ctx.report()
