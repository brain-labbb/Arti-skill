"""Decorative two-pan balance scale (scales-of-justice) modular template.

NOTE on the slug name: "scale" here = **decorative two-pan balance scale**
(scales-of-justice style, commonly matte black cast iron), NOT a bathroom /
kitchen platform scale, spring / hanging hook scale, measuring ruler, or fish
scale. The structure family is a pedestal balance: a static grounded
``pedestal`` (column + clevis pivot head) carries a ``beam`` that pivots about
+Y (weighing tilt), and two pan slings ``pan_0`` / ``pan_1`` each hang on a
beam tip and swing about +Y (so the pans stay level as the beam tilts).

Sourced from the reviewed modular spec
``articraft_template_authoring/specs_modular_v1/Other_Scale.md`` and the
``picture/Other/Scale`` 5-star sample pool (1 parent + 5 slot-fork variants +
2 chain_count multiplicity variants), all synced under ``data/records/``.

FIXED IDENTITY INVARIANT: the 3-joint mechanism (1 beam-pivot REVOLUTE +Y at
the column top + 2 pan-swing REVOLUTE +Y at the beam tips) and ``pan_count==2``
are constant across every sample. The three named slots only swap visual /
mesh content, never joint topology:

  * ``pedestal_column`` (3): turned_baluster_square_plinth / tripod_legs /
    round_drum_base — the ``pedestal`` part's visual composition. The
    ``pivot_head`` clevis is the shared, invariant pedestal->beam interface.
  * ``beam_form`` (2): straight_rect / ornate_scroll_beam — the ``beam`` bar
    mesh. Both keep the same pivot_axle + pivot_hub + tip hang_ball hardware.
    (Documented degrade to 2 candidates: the sample pool has only one beam
    fork; spec §4.)
  * ``pan_hanger`` (3): three_chain_dished / flat_plate_single_rod /
    deep_bucket_scoop — the ``pan_{idx}`` hanger form + vessel.
  * ``chain_count`` (N): a per-pan multiplicity axis — N converging rod chains
    inlined as non-moving visuals (Rule 1) along the pan rim, encoded into the
    slot_choice tuple as ``("chain_count", f"n{N}")``. flat_plate_single_rod is
    gated to N=1 (a single central rod, no converging chains).

All pivots are captured-pin / hook-on-ring geometry (pivot_axle through the
clevis fork; hanger top ring threaded over the beam-tip hang_ball), so those
joints omit ``MatingContract`` (grandfathered) and are guarded by the flat
articulation-origin baseline + element-scoped ``allow_overlap`` (mirroring each
source record's run_tests allow_overlap block).

Compatibility gating (resolve_config, spec §9):
  * flat_plate_single_rod has only a single central rod -> force N=1.
  * The three named slots are otherwise orthogonal (the pivot_head and tip
    hang_ball interfaces are identical), so all combinations are legal.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, replace
from functools import lru_cache
from typing import Literal

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)

__modular__ = True

PedestalColumn = Literal["turned_baluster_square_plinth", "tripod_legs", "round_drum_base"]
PanHanger = Literal["three_chain_dished", "flat_plate_single_rod", "deep_bucket_scoop"]
BeamForm = Literal["straight_rect", "ornate_scroll_beam"]
PaletteStyle = Literal[
    "matte_black_cast_iron",
    "antique_brass",
    "polished_chrome_silver",
    "aged_bronze_verdigris",
    "weathered_pewter",
]

PEDESTAL_COLUMNS: tuple[PedestalColumn, ...] = (
    "turned_baluster_square_plinth",
    "tripod_legs",
    "round_drum_base",
)
PAN_HANGERS: tuple[PanHanger, ...] = (
    "three_chain_dished",
    "flat_plate_single_rod",
    "deep_bucket_scoop",
)
BEAM_FORMS: tuple[BeamForm, ...] = ("straight_rect", "ornate_scroll_beam")
PALETTE_STYLES: tuple[PaletteStyle, ...] = (
    "matte_black_cast_iron",
    "antique_brass",
    "polished_chrome_silver",
    "aged_bronze_verdigris",
    "weathered_pewter",
)

N_MIN = 2
N_MAX = 5
# Chain-count sampling weights (spec §8: 2/3 high frequency, 4 common, 5 tail).
CHAIN_COUNT_WEIGHTS = (0.34, 0.34, 0.20, 0.12)  # for N in (2, 3, 4, 5)

# Palette: four cast-iron-style colorways driving base/beam/chain/pan materials.
PALETTES: dict[PaletteStyle, dict[str, tuple[float, float, float, float]]] = {
    "matte_black_cast_iron": {
        "base": (0.075, 0.075, 0.082, 1.0),
        "beam": (0.105, 0.105, 0.115, 1.0),
        "chain": (0.135, 0.135, 0.145, 1.0),
        "pan": (0.060, 0.060, 0.067, 1.0),
    },
    "antique_brass": {
        "base": (0.52, 0.40, 0.16, 1.0),
        "beam": (0.62, 0.48, 0.20, 1.0),
        "chain": (0.46, 0.35, 0.14, 1.0),
        "pan": (0.58, 0.45, 0.18, 1.0),
    },
    "polished_chrome_silver": {
        "base": (0.66, 0.68, 0.71, 1.0),
        "beam": (0.74, 0.76, 0.79, 1.0),
        "chain": (0.58, 0.60, 0.63, 1.0),
        "pan": (0.70, 0.72, 0.75, 1.0),
    },
    "aged_bronze_verdigris": {
        "base": (0.26, 0.36, 0.30, 1.0),
        "beam": (0.34, 0.30, 0.20, 1.0),
        "chain": (0.30, 0.40, 0.34, 1.0),
        "pan": (0.38, 0.46, 0.40, 1.0),
    },
    "weathered_pewter": {
        "base": (0.40, 0.41, 0.43, 1.0),
        "beam": (0.46, 0.47, 0.49, 1.0),
        "chain": (0.36, 0.37, 0.39, 1.0),
        "pan": (0.43, 0.44, 0.46, 1.0),
    },
}

# ---------------------------------------------------------------------------
# Base layout constants (meters) — verbatim from the parent record 58c45b51.
# Frame: +Z up, ground at z=0, beam along X, all pivot axes along +Y.
# ---------------------------------------------------------------------------
PLINTH1_W, PLINTH1_H = 0.130, 0.018  # bottom square step
PLINTH2_W, PLINTH2_H = 0.114, 0.014  # second square step
CAP_BASE_W, CAP_TOP_R = 0.100, 0.040  # square -> round molding loft
CAP_Z0, CAP_Z1 = 0.032, 0.052

COLUMN_TOP = 0.342  # top of the turned column capital
PIVOT_Z = 0.357  # beam pivot height (clevis center)

# Tripod support (variant 5bfc3418).
LEG_COUNT = 3
HUB_TOP_Z = 0.343
HUB_BOT_Z = 0.275
FOOT_SPREAD_R = 0.115
FOOT_BALL_R = 0.010
FOOT_CENTER_Z = FOOT_BALL_R

# Round drum base (variant 9e45efb9).
DRUM_R = 0.065
DRUM_TOP = 0.046

BEAM_LEN = 0.300
BEAM_SEC_Y = 0.012
BEAM_SEC_Z = 0.014
KNOB_R = 0.008
PIN_R = 0.0025
BALL_R = 0.0050
HANG_Z = -0.0215  # hang-ball center in beam frame

TILT = math.radians(15.0)  # nominal +/- joint range for all three joints

# Pan sling (parent three_chain_dished).
RING_R = 0.006  # hanger top ring (torus) major radius
RING_TUBE = 0.0017
ROD_R = 0.0017
DISH_R = 0.055  # pan radius (0.11 m diameter)
DISH_SPHERE_R = 0.135
DISH_BOT = -0.192
DISH_RIM_TOP = -0.1795
ATTACH_R = 0.050  # rod attachment radius on the rim
ATTACH_Z = -0.1805
HUB_Z = -0.008  # hanger hub center (where chains converge)

# flat_plate_single_rod (variant 54123985).
PLATE_R = 0.055
PLATE_T = 0.003
RIM_H = 0.004
RIM_W = 0.002
ROD_LEN = 0.168
PLATE_TOP_Z = HUB_Z - ROD_LEN  # = -0.176
PLATE_BOT_Z = PLATE_TOP_Z - PLATE_T

# deep_bucket_scoop (variant 2f16a2b1).
BUCKET_TOP_R = 0.055
BUCKET_BOT_R = 0.048
BUCKET_RIM_TOP = -0.177
BUCKET_WALL_TOP = -0.182
BUCKET_BOT = -0.232
BUCKET_WALL_T = 0.003
BUCKET_BOT_T = 0.003
BUCKET_RIM_LIP = 0.004

FLUTE_COUNT = 14

# Column base anchor: the turned column / flute ring start at this z (sitting on
# the plinth cap / drum top). Column-height scaling stretches ABOVE this anchor
# so the column bottom stays glued to the base.
COLUMN_BASE_Z = 0.046


def _col_z(z: float, z_scale: float) -> float:
    """Anchored vertical scaling: keep COLUMN_BASE_Z fixed, stretch above it."""
    return COLUMN_BASE_Z + (z - COLUMN_BASE_Z) * z_scale


def _scaled_column_top(z_scale: float) -> float:
    return _col_z(COLUMN_TOP, z_scale)


# ---------------------------------------------------------------------------
# Motion-limit derivation (spec §7 clearance; Contract 3c single-sourced).
#
# The pans hang far below the beam-tip swing pivots, so both the beam-tilt joint
# (pedestal_to_beam) and the pan-swing joints (beam_to_pan_*) rotate each vessel
# about +Y. Because the axes are parallel, the vessel's world orientation is the
# SUM of the two joint angles: a vessel point (Px, Pz) in the pan frame maps to
# world x = tip_x*cos(qb) + HANG_Z*sin(qb) + (Px*cos(qb+qp) + Pz*sin(qb+qp)).
# The inboard-lowest vessel point (Px = -pan_r, Pz = z_bot) is what swings into
# the pedestal legs / column — exactly the census 穿模 pattern. We therefore
# derive the joint MotionLimits so this worst point clears the pedestal keep-out
# radius by >= CLEAR_MARGIN across the whole joint envelope (a real balance-scale
# beam stop), rather than blanket-allowing the pan-into-leg contact.
# ---------------------------------------------------------------------------
LEG_ROD_HALF = 0.016  # horizontal half-width envelope of a tapered tripod leg rod
COL_MAX_R = 0.032  # max radius of the turned baluster / drum column
FLUTE_MAX_R = 0.046  # max radius of the decorative flute ring
CLEAR_MARGIN = 0.008  # target pan<->pedestal clearance (>= 8 mm)
SAFE_MARGIN = 0.006  # hard floor clearance (still overlap-free / motion-QC safe)
REST_BUFFER = 0.003  # extra rest clearance so the near-zero joint floor still clears
PAN_FLOOR = 0.80  # min pan_size_scale (connectivity-safe) when buying weighing room
TILT_BUDGET_TARGET = math.radians(14.0)  # aim for >= ~14deg combined tilt+swing


def _vessel_profile(pan_hanger: PanHanger, ps: float) -> tuple[tuple[float, float], ...]:
    """Inboard silhouette of the vessel as (outer_radius, z) points in the pan
    frame. The vessel is a horizontal cup/disk, so its inboard-most point at any
    height is (-outer_radius, z); the deep centre of a bucket/dish is narrow, so
    we track the real taper instead of assuming full radius at the lowest z."""
    if pan_hanger == "deep_bucket_scoop":
        return (
            ((BUCKET_TOP_R + BUCKET_RIM_LIP) * ps, BUCKET_RIM_TOP),
            (BUCKET_TOP_R * ps, BUCKET_WALL_TOP),
            (BUCKET_BOT_R * ps, BUCKET_BOT),
        )
    if pan_hanger == "flat_plate_single_rod":
        return (
            (PLATE_R * ps, PLATE_TOP_Z),
            (PLATE_R * ps, PLATE_BOT_Z),
        )
    # three_chain_dished: shallow spherical cap, widest at the rim.
    return (
        (DISH_R * ps, DISH_RIM_TOP),
        (0.55 * DISH_R * ps, DISH_BOT),
    )


def _rest_clearance(
    pan_hanger: PanHanger,
    ps: float,
    pedestal_column: PedestalColumn,
    fs: float,
    zs: float,
    pivot_z: float,
    tip_x: float,
) -> float:
    """Min clearance of the resting vessel silhouette vs the pedestal envelope."""
    worst = math.inf
    for rp, zp in _vessel_profile(pan_hanger, ps):
        xw = tip_x - rp
        zw = pivot_z + HANG_Z + zp
        worst = min(worst, xw - _pedestal_radius_at(zw, pedestal_column, fs, zs))
    return worst


def _pedestal_radius_at(z: float, pedestal_column: PedestalColumn, fs: float, zs: float) -> float:
    """Conservative horizontal radius of pedestal material at height ``z`` on the
    pan-facing side. Over-estimates real geometry so the analytic clearance is a
    lower bound on the true mesh clearance (env >= 8 mm  =>  QC-safe)."""
    if pedestal_column == "tripod_legs":
        # leg_0 sits at azimuth 0 (directly under pan_0's swing plane): its
        # centerline radius shrinks linearly from the foot spread at the ground
        # to ~0 at the hub, plus the tapered rod half-width.
        rad = 0.0
        if FOOT_CENTER_Z <= z <= HUB_BOT_Z:
            t = (HUB_BOT_Z - z) / (HUB_BOT_Z - FOOT_CENTER_Z)
            rad = FOOT_SPREAD_R * fs * t + LEG_ROD_HALF
        elif z < FOOT_CENTER_Z:
            rad = FOOT_SPREAD_R * fs + FOOT_BALL_R
        if HUB_BOT_Z - 0.03 <= z <= HUB_TOP_Z + 0.02:
            rad = max(rad, 0.030)
        return rad
    # baluster / drum families share the turned column.
    col_top = _scaled_column_top(zs)
    rad = 0.0
    if COLUMN_BASE_Z - 0.005 <= z <= col_top + 0.010:
        rad = COL_MAX_R
    if _col_z(0.050, zs) <= z <= _col_z(0.078, zs):
        rad = max(rad, FLUTE_MAX_R)
    if pedestal_column == "round_drum_base":
        if z <= DRUM_TOP + 0.004:
            rad = max(rad, DRUM_R * fs)
    else:  # turned_baluster_square_plinth
        if z <= CAP_Z1 + 0.004:
            rad = max(rad, (CAP_BASE_W / 2.0) * fs)
        if z <= PLINTH1_H + 0.004:
            rad = max(rad, (PLINTH1_W / 2.0) * fs)
    if z >= col_top - 0.010:
        rad = max(rad, 0.020)
    return rad


def _pan_pedestal_clearance(
    qb: float,
    qp: float,
    profile: tuple[tuple[float, float], ...],
    pedestal_column: PedestalColumn,
    fs: float,
    zs: float,
    pivot_z: float,
    tip_x: float,
) -> float:
    """Signed clearance (m) of pan_0's inboard vessel silhouette vs the pedestal
    keep-out radius, at beam tilt ``qb`` and pan swing ``qp``."""
    phi = qb + qp
    cphi, sphi = math.cos(phi), math.sin(phi)
    base_x = tip_x * math.cos(qb) + HANG_Z * math.sin(qb)
    base_z = pivot_z - tip_x * math.sin(qb) + HANG_Z * math.cos(qb)
    worst = math.inf
    for rp, zp in profile:
        # inboard silhouette point (-rp, zp) rotated by phi about +Y.
        xw = base_x + (-rp * cphi + zp * sphi)
        zw = base_z + (rp * sphi + zp * cphi)
        worst = min(worst, xw - _pedestal_radius_at(zw, pedestal_column, fs, zs))
    return worst


def _grid(a: float, b: float, n: int) -> list[float]:
    if n <= 1 or a == b:
        return [a]
    return [a + (b - a) * k / (n - 1) for k in range(n)]


def _min_clearance(
    T_b: float,
    T_p: float,
    profile: tuple[tuple[float, float], ...],
    pedestal_column: PedestalColumn,
    fs: float,
    zs: float,
    pivot_z: float,
    tip_x: float,
) -> float:
    """Worst-case pan/pedestal clearance over the full [-T_b,T_b]x[-T_p,T_p] box."""
    worst = math.inf
    for qb in _grid(-T_b, T_b, 5):
        for qp in _grid(-T_p, T_p, 5):
            worst = min(
                worst,
                _pan_pedestal_clearance(qb, qp, profile, pedestal_column, fs, zs, pivot_z, tip_x),
            )
    return worst


def _derive_limits(
    profile: tuple[tuple[float, float], ...],
    pedestal_column: PedestalColumn,
    fs: float,
    zs: float,
    pivot_z: float,
    tip_x: float,
    tilt_nom: float,
) -> tuple[float, float]:
    """Largest ``(beam_tilt, pan_swing) <= tilt_nom`` that keep pan_0 clear of the
    pedestal across the full joint envelope (the beam stop).

    Keeps BOTH motions usable by maximising the smaller of the two ranges first
    (so neither the weighing tilt nor the pan swing is starved), then favouring
    beam tilt. Targets ``CLEAR_MARGIN`` (8 mm); if no non-degenerate range meets
    that within the connectivity-safe pan range (a deep bucket on a wide short
    tripod), it falls back to ``SAFE_MARGIN`` (still overlap-free, motion-QC
    safe), then to the smallest non-degenerate range."""
    lo = math.radians(0.1)
    hi = max(lo, tilt_nom)
    steps = 18
    candidates = [lo + (hi - lo) * i / steps for i in range(steps + 1)]
    for margin in (CLEAR_MARGIN, SAFE_MARGIN):
        best: tuple[tuple[float, float], float, float] | None = None
        for T_b in candidates:
            for T_p in candidates:
                if (
                    _min_clearance(T_b, T_p, profile, pedestal_column, fs, zs, pivot_z, tip_x)
                    >= margin
                ):
                    score = (round(min(T_b, T_p), 6), round(T_b, 6), round(T_p, 6))
                    if best is None or score > best[0]:
                        best = (score, T_b, T_p)
        if best is not None:
            return best[1], best[2]
    return lo, lo


@dataclass(frozen=True)
class ScaleConfig:
    pedestal_column: PedestalColumn | None = None
    pan_hanger: PanHanger | None = None
    beam_form: BeamForm | None = None
    chain_count: int | None = None
    palette_style: PaletteStyle = "matte_black_cast_iron"
    column_height_scale: float = 1.0
    base_footprint_scale: float = 1.0
    beam_len_scale: float = 1.0
    pan_size_scale: float = 1.0
    tilt_angle_scale: float = 1.0
    name: str = "scale"


@dataclass(frozen=True)
class ResolvedScaleConfig:
    pedestal_column: PedestalColumn
    pan_hanger: PanHanger
    beam_form: BeamForm
    chain_count: int
    palette_style: PaletteStyle
    column_height_scale: float
    base_footprint_scale: float
    beam_len_scale: float
    pan_size_scale: float
    beam_tilt: float
    pan_swing: float
    # Derived interface geometry.
    pivot_z: float  # beam pivot height (clevis center)
    tip_x: float  # beam half-span (hang_ball x)
    name: str


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _pick(value, choices):
    return value if value in choices else choices[0]


def config_from_seed(seed: int) -> ScaleConfig:
    rng = random.Random(seed)
    pan_hanger = rng.choice(PAN_HANGERS)
    if pan_hanger == "flat_plate_single_rod":
        chain_count = 1
    else:
        chain_count = rng.choices((2, 3, 4, 5), weights=CHAIN_COUNT_WEIGHTS, k=1)[0]
    return ScaleConfig(
        pedestal_column=rng.choice(PEDESTAL_COLUMNS),
        pan_hanger=pan_hanger,
        beam_form=rng.choice(BEAM_FORMS),
        chain_count=chain_count,
        palette_style=rng.choice(PALETTE_STYLES),
        column_height_scale=round(rng.uniform(0.90, 1.12), 4),
        base_footprint_scale=round(rng.uniform(0.90, 1.15), 4),
        beam_len_scale=round(rng.uniform(0.88, 1.12), 4),
        pan_size_scale=round(rng.uniform(0.88, 1.12), 4),
        tilt_angle_scale=round(rng.uniform(0.80, 1.20), 4),
        name=f"seeded_scale_{seed}",
    )


def resolve_config(config: ScaleConfig | None = None) -> ResolvedScaleConfig:
    # ScaleConfig is a frozen (hashable) dataclass; cache the derivation (which
    # runs a per-config clearance grid-search) so build + run_tests + collision
    # target don't recompute it within one compile.
    return _resolve_config_cached(config or ScaleConfig())


@lru_cache(maxsize=256)
def _resolve_config_cached(cfg: ScaleConfig) -> ResolvedScaleConfig:
    palette_style = _pick(cfg.palette_style, PALETTE_STYLES)
    pedestal_column = _pick(cfg.pedestal_column, PEDESTAL_COLUMNS)
    pan_hanger = _pick(cfg.pan_hanger, PAN_HANGERS)
    beam_form = _pick(cfg.beam_form, BEAM_FORMS)

    # --- chain_count gating (spec §9). flat_plate has only a single central
    #     rod -> force N=1. Otherwise clamp into [2, 5].
    if pan_hanger == "flat_plate_single_rod":
        chain_count = 1
    else:
        n = int(cfg.chain_count) if cfg.chain_count is not None else 3
        chain_count = int(_clamp(n, N_MIN, N_MAX))

    column_height_scale = _clamp(cfg.column_height_scale, 0.90, 1.12)
    base_footprint_scale = _clamp(cfg.base_footprint_scale, 0.90, 1.15)
    beam_len_scale = _clamp(cfg.beam_len_scale, 0.88, 1.12)
    pan_size_scale = _clamp(cfg.pan_size_scale, 0.88, 1.12)
    tilt_nom = _clamp(TILT * _clamp(cfg.tilt_angle_scale, 0.80, 1.20), 0.0, math.radians(25.0))

    # Derived interfaces: PIVOT_Z follows the column height; TIP_X follows the
    # beam length. The clevis offset above the column capital is held constant
    # so the pivot head still caps the column. The tripod hub is a fixed-height
    # turned part (no scaled column), so column_height_scale only applies to the
    # baluster / drum families.
    clevis_offset = PIVOT_Z - COLUMN_TOP  # 0.015
    if pedestal_column == "tripod_legs":
        column_height_scale = 1.0
        pivot_z = PIVOT_Z
    else:
        pivot_z = _scaled_column_top(column_height_scale) + clevis_offset
    tip_x = (BEAM_LEN * beam_len_scale) / 2.0

    # --- Rest clearance + motion budget (spec §7, Contract 3c): shrink the pan
    #     vessel until (a) at rest its inboard silhouette clears the pedestal
    #     keep-out envelope by CLEAR_MARGIN + a buffer (so the near-zero joint
    #     floor still clears >= 8 mm), and (b) the derived joint range gives a
    #     usable weighing budget. This uses the geometry-derived pedestal
    #     envelope (fs-scaled legs / column), not a flat footprint half-width,
    #     so wide tripod feet and tall columns are handled per config. Wide-leg
    #     tripods trade a little pan size for the room to actually tilt/swing.
    beam_tilt = pan_swing = 0.0
    for _ in range(60):
        rest_clr = _rest_clearance(
            pan_hanger,
            pan_size_scale,
            pedestal_column,
            base_footprint_scale,
            column_height_scale,
            pivot_z,
            tip_x,
        )
        profile = _vessel_profile(pan_hanger, pan_size_scale)
        beam_tilt, pan_swing = _derive_limits(
            profile,
            pedestal_column,
            base_footprint_scale,
            column_height_scale,
            pivot_z,
            tip_x,
            tilt_nom,
        )
        ok_rest = rest_clr >= CLEAR_MARGIN + REST_BUFFER
        ok_budget = (beam_tilt + pan_swing) >= TILT_BUDGET_TARGET
        if (ok_rest and ok_budget) or pan_size_scale <= PAN_FLOOR:
            break
        pan_size_scale = max(PAN_FLOOR, round(pan_size_scale - 0.01, 4))

    return ResolvedScaleConfig(
        pedestal_column=pedestal_column,
        pan_hanger=pan_hanger,
        beam_form=beam_form,
        chain_count=chain_count,
        palette_style=palette_style,
        column_height_scale=column_height_scale,
        base_footprint_scale=base_footprint_scale,
        beam_len_scale=beam_len_scale,
        pan_size_scale=pan_size_scale,
        beam_tilt=beam_tilt,
        pan_swing=pan_swing,
        pivot_z=pivot_z,
        tip_x=tip_x,
        name=cfg.name or "scale",
    )


def with_overrides(config: ScaleConfig, **kwargs: object) -> ScaleConfig:
    return replace(config, **kwargs)


def _chain_attach_angles(n: int) -> tuple[float, ...]:
    """Even angular spread of N converging chains around the pan rim.

    Absolute placement (angle = base + j*360/N) so the layout is N-invariant.
    Base offset 90 deg matches the parent / chain-count variants.
    """
    return tuple(90.0 + j * 360.0 / n for j in range(n))


def slot_choices_for_config(
    config: ScaleConfig | ResolvedScaleConfig,
) -> tuple[tuple[str, str], ...]:
    r = config if isinstance(config, ResolvedScaleConfig) else resolve_config(config)
    return (
        ("pedestal_column", r.pedestal_column),
        ("beam_form", r.beam_form),
        ("pan_hanger", r.pan_hanger),
        ("chain_count", f"n{r.chain_count}"),
    )


def slot_choices_for_seed(seed: int) -> tuple[tuple[str, str], ...]:
    return slot_choices_for_config(config_from_seed(seed))


# ---------------------------------------------------------------------------
# Shared cadquery helpers (verbatim source primitives; LatheGeometry / revolve
# / torus / sweep preserved — never downgraded to Box/Cylinder).
# ---------------------------------------------------------------------------
def _rod_solid(p0: tuple, p1: tuple, radius: float) -> cq.Solid:
    v = (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2])
    length = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
    direction = cq.Vector(v[0] / length, v[1] / length, v[2] / length)
    return cq.Solid.makeCylinder(radius, length, cq.Vector(*p0), direction)


def _build_column(z_scale: float) -> cq.Workplane:
    """Turned baluster column (lathe revolve). z_scale stretches the height."""
    pts = [
        (0.0, 0.046),
        (0.042, 0.046),
        (0.040, 0.052),
        (0.033, 0.062),
        (0.025, 0.076),
        (0.021, 0.090),
        (0.020, 0.104),
        (0.0245, 0.110),
        (0.0245, 0.118),
        (0.020, 0.124),
        (0.027, 0.142),
        (0.0285, 0.162),
        (0.024, 0.186),
        (0.019, 0.206),
        (0.023, 0.212),
        (0.023, 0.220),
        (0.018, 0.226),
        (0.016, 0.250),
        (0.014, 0.280),
        (0.017, 0.286),
        (0.017, 0.292),
        (0.013, 0.297),
        (0.012, 0.320),
        (0.016, 0.328),
        (0.016, 0.338),
        (0.011, COLUMN_TOP),
        (0.0, COLUMN_TOP),
    ]
    scaled = [(r, _col_z(z, z_scale)) for (r, z) in pts]
    return cq.Workplane("XZ").polyline(scaled).close().revolve(360.0, (0, 0), (0, 1))


def _build_flute_ring(z_scale: float) -> cq.Workplane:
    """Ring of slanted ribs decorating the bell flare above the base."""
    solids = []
    for k in range(FLUTE_COUNT):
        ang = math.radians(k * 360.0 / FLUTE_COUNT)
        p0 = (0.0405 * math.cos(ang), 0.0405 * math.sin(ang), _col_z(0.050, z_scale))
        p1 = (0.026 * math.cos(ang), 0.026 * math.sin(ang), _col_z(0.078, z_scale))
        solids.append(_rod_solid(p0, p1, 0.0035))
    return cq.Workplane(obj=cq.Compound.makeCompound(solids))


def _build_pivot_head(z_scale: float) -> cq.Workplane:
    """Clevis fork at the column top, bridge cap, and dome finial.

    Shifted vertically so the clevis center stays at z = COLUMN_TOP*z_scale +
    clevis_offset (the invariant pedestal->beam interface).
    """
    dz = _scaled_column_top(z_scale) - COLUMN_TOP
    plate_a = (
        cq.Workplane("XY")
        .box(0.024, 0.006, 0.034, centered=(True, True, False))
        .translate((0.0, 0.011, 0.340 + dz))
    )
    plate_b = plate_a.mirror("XZ")
    bridge = (
        cq.Workplane("XY")
        .box(0.024, 0.028, 0.006, centered=(True, True, False))
        .translate((0.0, 0.0, 0.374 + dz))
    )
    neck = (
        cq.Workplane("XY")
        .cylinder(0.009, 0.011, centered=(True, True, False))
        .translate((0.0, 0.0, 0.379 + dz))
    )
    dome = cq.Workplane("XY").sphere(0.0115).translate((0.0, 0.0, 0.389 + dz))
    return plate_a.union(plate_b).union(bridge).union(neck).union(dome)


def _build_plinth(footprint_scale: float) -> cq.Workplane:
    """Stepped square plinth with a square-to-round molding cap."""
    fs = footprint_scale
    step1 = cq.Workplane("XY").box(
        PLINTH1_W * fs, PLINTH1_W * fs, PLINTH1_H, centered=(True, True, False)
    )
    step2 = (
        cq.Workplane("XY")
        .workplane(offset=PLINTH1_H)
        .box(PLINTH2_W * fs, PLINTH2_W * fs, PLINTH2_H, centered=(True, True, False))
    )
    cap = (
        cq.Workplane("XY")
        .workplane(offset=CAP_Z0)
        .rect(CAP_BASE_W * fs, CAP_BASE_W * fs)
        .workplane(offset=CAP_Z1 - CAP_Z0)
        .circle(CAP_TOP_R)
        .loft(ruled=True)
    )
    return step1.union(step2).union(cap)


def _build_drum_base(footprint_scale: float) -> cq.Workplane:
    """Round cylindrical drum base with lathe-turned moldings."""
    dr = DRUM_R * footprint_scale
    pts = [
        (0.0, 0.0),
        (dr, 0.0),
        (dr, 0.004),
        (dr - 0.003, 0.007),
        (dr - 0.005, 0.010),
        (dr - 0.005, 0.036),
        (dr - 0.003, 0.038),
        (dr, 0.040),
        (dr - 0.003, 0.043),
        (0.044, DRUM_TOP),
        (0.0, DRUM_TOP),
    ]
    return cq.Workplane("XZ").polyline(pts).close().revolve(360.0, (0, 0), (0, 1))


def _build_hub() -> cq.Workplane:
    """Turned tripod central hub (lathe): legs below, pivot head above."""
    pts = [
        (0.0, HUB_BOT_Z),
        (0.026, HUB_BOT_Z),
        (0.028, HUB_BOT_Z + 0.006),
        (0.024, HUB_BOT_Z + 0.018),
        (0.020, HUB_BOT_Z + 0.034),
        (0.023, HUB_BOT_Z + 0.050),
        (0.018, HUB_TOP_Z - 0.005),
        (0.016, HUB_TOP_Z),
        (0.0, HUB_TOP_Z),
    ]
    return cq.Workplane("XZ").polyline(pts).close().revolve(360.0, (0, 0), (0, 1))


def _build_positioned_leg(leg_index: int, footprint_scale: float) -> cq.Workplane:
    """One turned tripod leg (lathe) with a ball foot, rotated into place."""
    foot_spread = FOOT_SPREAD_R * footprint_scale
    leg_dz = HUB_BOT_Z - FOOT_CENTER_Z
    leg_length = math.sqrt(foot_spread**2 + leg_dz**2)
    splay = math.degrees(math.atan2(foot_spread, leg_dz))
    L = leg_length
    pts = [
        (0.0, 0.0),
        (0.015, 0.0),
        (0.017, -0.006),
        (0.014, -0.018),
        (0.018, -0.048),
        (0.016, -0.068),
        (0.013, -0.100),
        (0.015, -0.118),
        (0.015, -0.124),
        (0.012, -0.130),
        (0.011, -0.175),
        (0.013, -0.200),
        (0.011, -0.235),
        (FOOT_BALL_R, -L),
        (0.0, -L),
    ]
    leg = cq.Workplane("XZ").polyline(pts).close().revolve(360.0, (0, 0), (0, 1))
    ball = cq.Workplane("XY").sphere(FOOT_BALL_R).translate((0, 0, -L))
    leg = leg.union(ball)
    leg = leg.rotate((0, 0, 0), (0, 1, 0), -splay)
    leg = leg.translate((0, 0, HUB_BOT_Z))
    azimuth = leg_index * 360.0 / LEG_COUNT
    if azimuth != 0.0:
        leg = leg.rotate((0, 0, 0), (0, 0, 1), azimuth)
    return leg


# ---------------------------------------------------------------------------
# Pan hanger / vessel helpers.
# ---------------------------------------------------------------------------
def _build_hanger_top() -> cq.Workplane:
    """Top ring (torus) + converging hub sphere (shared across pan slings)."""
    ring = cq.Solid.makeTorus(RING_R, RING_TUBE, cq.Vector(0, 0, 0), cq.Vector(0, 1, 0))
    hub = cq.Solid.makeSphere(0.0035, cq.Vector(0, 0, HUB_Z), angleDegrees1=-90, angleDegrees2=90)
    return cq.Workplane(obj=ring).union(cq.Workplane(obj=hub))


def _build_single_chain(ang_deg: float, pan_size_scale: float) -> cq.Workplane:
    """One thin rod chain from the hub to a rim attachment point + a rim ball."""
    ang = math.radians(ang_deg)
    ar = ATTACH_R * pan_size_scale
    tip = (ar * math.cos(ang), ar * math.sin(ang), ATTACH_Z)
    rod = _rod_solid((0.0, 0.0, HUB_Z), tip, ROD_R)
    ball = cq.Solid.makeSphere(0.0028, cq.Vector(*tip), angleDegrees1=-90, angleDegrees2=90)
    return cq.Workplane(obj=rod).union(cq.Workplane(obj=ball))


def _build_single_rod() -> cq.Workplane:
    """Single central hanging rod (flat_plate_single_rod, N=1)."""
    rod = (
        cq.Workplane("XY")
        .cylinder(ROD_LEN, ROD_R, centered=(True, True, False))
        .translate((0.0, 0.0, PLATE_TOP_Z))
    )
    return rod


def _build_dish(pan_size_scale: float) -> cq.Workplane:
    """Shallow concave dished pan with a thin rim (spherical-cap shell)."""
    ps = pan_size_scale
    dish_r = DISH_R * ps
    outer = cq.Workplane("XY").sphere(DISH_SPHERE_R).translate((0.0, 0.0, DISH_BOT + DISH_SPHERE_R))
    inner = (
        cq.Workplane("XY")
        .sphere(DISH_SPHERE_R)
        .translate((0.0, 0.0, DISH_BOT + DISH_SPHERE_R + 0.0028))
    )
    slab = (
        cq.Workplane("XY")
        .cylinder(DISH_RIM_TOP - (DISH_BOT - 0.001), dish_r, centered=(True, True, False))
        .translate((0.0, 0.0, DISH_BOT - 0.001))
    )
    return outer.cut(inner).intersect(slab)


def _build_flat_plate(pan_size_scale: float) -> cq.Workplane:
    """Flat circular weighing plate with a raised rim (flat_plate_single_rod)."""
    pr = PLATE_R * pan_size_scale
    plate = (
        cq.Workplane("XY")
        .cylinder(PLATE_T, pr, centered=(True, True, False))
        .translate((0.0, 0.0, PLATE_BOT_Z))
    )
    rim_outer = (
        cq.Workplane("XY")
        .cylinder(RIM_H, pr, centered=(True, True, False))
        .translate((0.0, 0.0, PLATE_TOP_Z))
    )
    rim_inner = (
        cq.Workplane("XY")
        .cylinder(RIM_H, pr - RIM_W, centered=(True, True, False))
        .translate((0.0, 0.0, PLATE_TOP_Z))
    )
    return plate.union(rim_outer.cut(rim_inner))


def _build_bucket(pan_size_scale: float) -> cq.Workplane:
    """Deep bucket/scoop pan (lathe revolve outer cut inner hollow)."""
    ps = pan_size_scale
    top_r = BUCKET_TOP_R * ps
    bot_r = BUCKET_BOT_R * ps
    outer_pts = [
        (0.0, BUCKET_RIM_TOP),
        (top_r + BUCKET_RIM_LIP, BUCKET_RIM_TOP),
        (top_r + BUCKET_RIM_LIP, BUCKET_WALL_TOP),
        (top_r, BUCKET_WALL_TOP),
        (bot_r, BUCKET_BOT),
        (0.0, BUCKET_BOT),
    ]
    outer = cq.Workplane("XZ").polyline(outer_pts).close().revolve(360.0, (0, 0), (0, 1))
    inner_pts = [
        (0.0, BUCKET_WALL_TOP),
        (top_r - BUCKET_WALL_T, BUCKET_WALL_TOP),
        (bot_r - BUCKET_WALL_T, BUCKET_BOT + BUCKET_BOT_T),
        (0.0, BUCKET_BOT + BUCKET_BOT_T),
    ]
    inner = cq.Workplane("XZ").polyline(inner_pts).close().revolve(360.0, (0, 0), (0, 1))
    return outer.cut(inner)


# ---------------------------------------------------------------------------
# Ornate scroll beam helpers (sweep / tube meshes — preserved, not downgraded).
# ---------------------------------------------------------------------------
def _scroll_arm_path(sign: float, tip_x: float) -> list[tuple[float, float, float]]:
    s = sign
    return [
        (s * 0.005, 0.0, 0.0),
        (s * 0.030 * (tip_x / 0.150), 0.0, 0.006),
        (s * 0.055 * (tip_x / 0.150), 0.0, 0.013),
        (s * 0.080 * (tip_x / 0.150), 0.0, 0.015),
        (s * 0.105 * (tip_x / 0.150), 0.0, 0.010),
        (s * 0.130 * (tip_x / 0.150), 0.0, 0.003),
        (s * tip_x, 0.0, 0.0),
    ]


def _build_scroll_arm_mesh(sign: float, tip_x: float):
    profile = rounded_rect_profile(0.010, 0.016, radius=0.003, corner_segments=6)
    return sweep_profile_along_spline(
        _scroll_arm_path(sign, tip_x),
        profile=profile,
        samples_per_segment=18,
        cap_profile=True,
        up_hint=(0.0, 0.0, 1.0),
    )


def _scroll_volute_path(sign: float, tip_x: float) -> list[tuple[float, float, float]]:
    s = sign
    cx = s * tip_x
    return [
        (cx - s * 0.003, 0.0, 0.005),
        (cx - s * 0.001, 0.0, 0.014),
        (cx + s * 0.005, 0.0, 0.022),
        (cx + s * 0.011, 0.0, 0.020),
        (cx + s * 0.013, 0.0, 0.014),
        (cx + s * 0.009, 0.0, 0.010),
        (cx + s * 0.004, 0.0, 0.013),
    ]


def _build_scroll_volute_mesh(sign: float, tip_x: float):
    return tube_from_spline_points(
        _scroll_volute_path(sign, tip_x),
        radius=0.003,
        samples_per_segment=14,
        radial_segments=14,
        cap_ends=True,
        up_hint=(0.0, 1.0, 0.0),
    )


def _build_center_rosette_mesh():
    torus = cq.Solid.makeTorus(0.011, 0.0025, cq.Vector(0, 0, 0), cq.Vector(0, 1, 0))
    return mesh_from_cadquery(cq.Workplane(obj=torus), "beam_center_rosette")


# ---------------------------------------------------------------------------
# Pedestal part (root) — pedestal_column slot.
# ---------------------------------------------------------------------------
def _build_pedestal(model: ArticulatedObject, r: ResolvedScaleConfig, mats, *, assets):
    pedestal = model.part("pedestal")
    zs = r.column_height_scale
    fs = r.base_footprint_scale

    if r.pedestal_column == "tripod_legs":
        pedestal.visual(
            mesh_from_cadquery(_build_hub(), "pedestal_hub"),
            material=mats["base"],
            name="hub",
        )
        for i in range(LEG_COUNT):
            pedestal.visual(
                mesh_from_cadquery(_build_positioned_leg(i, fs), f"pedestal_leg_{i}"),
                material=mats["base"],
                name=f"leg_{i}",
            )
    elif r.pedestal_column == "round_drum_base":
        pedestal.visual(
            mesh_from_cadquery(_build_drum_base(fs), "pedestal_drum"),
            material=mats["base"],
            name="drum",
        )
        pedestal.visual(
            mesh_from_cadquery(_build_flute_ring(zs), "pedestal_flute_ring"),
            material=mats["base"],
            name="flute_ring",
        )
        pedestal.visual(
            mesh_from_cadquery(_build_column(zs), "pedestal_column"),
            material=mats["base"],
            name="column",
        )
    else:  # turned_baluster_square_plinth
        pedestal.visual(
            mesh_from_cadquery(_build_plinth(fs), "pedestal_plinth"),
            material=mats["base"],
            name="plinth",
        )
        pedestal.visual(
            mesh_from_cadquery(_build_flute_ring(zs), "pedestal_flute_ring"),
            material=mats["base"],
            name="flute_ring",
        )
        pedestal.visual(
            mesh_from_cadquery(_build_column(zs), "pedestal_column"),
            material=mats["base"],
            name="column",
        )

    # Pivot head clevis — the invariant pedestal->beam interface hardware.
    pedestal.visual(
        mesh_from_cadquery(_build_pivot_head(zs), "pedestal_pivot_head"),
        material=mats["base"],
        name="pivot_head",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((2.0 * PLINTH1_W * fs, 2.0 * PLINTH1_W * fs, r.pivot_z)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, r.pivot_z * 0.4)),
    )
    return pedestal


# ---------------------------------------------------------------------------
# Beam part — beam_form slot. Pivots about +Y at the column top.
# ---------------------------------------------------------------------------
def _build_beam(model: ArticulatedObject, r: ResolvedScaleConfig, pedestal, mats):
    beam = model.part("beam")
    tip_x = r.tip_x

    if r.beam_form == "ornate_scroll_beam":
        for idx, sx in ((0, 1.0), (1, -1.0)):
            beam.visual(
                mesh_from_geometry(_build_scroll_arm_mesh(sx, tip_x), f"scroll_arm_{idx}"),
                material=mats["beam"],
                name=f"scroll_arm_{idx}",
            )
            beam.visual(
                mesh_from_geometry(_build_scroll_volute_mesh(sx, tip_x), f"scroll_volute_{idx}"),
                material=mats["beam"],
                name=f"scroll_volute_{idx}",
            )
        beam.visual(
            _build_center_rosette_mesh(),
            material=mats["beam"],
            name="center_rosette",
        )
    else:  # straight_rect
        beam.visual(
            Box((2.0 * tip_x, BEAM_SEC_Y, BEAM_SEC_Z)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=mats["beam"],
            name="bar",
        )

    # Common pivot + tip hardware (constant across both beam forms).
    beam.visual(
        Cylinder(radius=0.0095, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=mats["beam"],
        name="pivot_hub",
    )
    beam.visual(
        Cylinder(radius=0.0035, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=mats["beam"],
        name="pivot_axle",
    )
    for idx, sx in ((0, 1.0), (1, -1.0)):
        beam.visual(
            Sphere(radius=KNOB_R),
            origin=Origin(xyz=(sx * tip_x, 0.0, 0.0)),
            material=mats["beam"],
            name=f"end_knob_{idx}",
        )
        beam.visual(
            Cylinder(radius=PIN_R, length=0.016),
            origin=Origin(xyz=(sx * tip_x, 0.0, -0.012)),
            material=mats["beam"],
            name=f"hang_pin_{idx}",
        )
        beam.visual(
            Sphere(radius=BALL_R),
            origin=Origin(xyz=(sx * tip_x, 0.0, HANG_Z)),
            material=mats["beam"],
            name=f"hang_ball_{idx}",
        )
    beam.inertial = Inertial.from_geometry(
        Box((2.0 * tip_x, BEAM_SEC_Y, BEAM_SEC_Z)),
        mass=0.20,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    model.articulation(
        "pedestal_to_beam",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, r.pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.0, lower=-r.beam_tilt, upper=r.beam_tilt),
    )
    return beam


# ---------------------------------------------------------------------------
# Pan parts — pan_hanger slot + chain_count multiplicity. Each swings +Y.
# ---------------------------------------------------------------------------
def _build_pans(model: ArticulatedObject, r: ResolvedScaleConfig, beam, mats):
    ps = r.pan_size_scale
    for idx, sx in ((0, 1.0), (1, -1.0)):
        pan = model.part(f"pan_{idx}")
        pan.visual(
            mesh_from_cadquery(_build_hanger_top(), f"pan_hanger_top_{idx}"),
            material=mats["chain"],
            name="hanger_top",
        )
        if r.pan_hanger == "flat_plate_single_rod":
            # N=1: a single central rod, no converging chains.
            pan.visual(
                mesh_from_cadquery(_build_single_rod(), f"pan_rod_{idx}"),
                material=mats["chain"],
                name="chain_0",
            )
            pan.visual(
                mesh_from_cadquery(_build_flat_plate(ps), f"pan_plate_{idx}"),
                material=mats["pan"],
                name="vessel",
            )
        else:
            # N converging rod chains along the rim (Rule 1: non-moving visuals).
            for j, ang_deg in enumerate(_chain_attach_angles(r.chain_count)):
                pan.visual(
                    mesh_from_cadquery(_build_single_chain(ang_deg, ps), f"pan_chain_{j}_{idx}"),
                    material=mats["chain"],
                    name=f"chain_{j}",
                )
            if r.pan_hanger == "deep_bucket_scoop":
                pan.visual(
                    mesh_from_cadquery(_build_bucket(ps), f"pan_bucket_{idx}"),
                    material=mats["pan"],
                    name="vessel",
                )
            else:  # three_chain_dished
                pan.visual(
                    mesh_from_cadquery(_build_dish(ps), f"pan_dish_{idx}"),
                    material=mats["pan"],
                    name="vessel",
                )
        pan.inertial = Inertial.from_geometry(
            Box((2.0 * DISH_R * ps, 2.0 * DISH_R * ps, 0.20)),
            mass=0.08,
            origin=Origin(xyz=(0.0, 0.0, -0.10)),
        )
        model.articulation(
            f"beam_to_pan_{idx}",
            ArticulationType.REVOLUTE,
            parent=beam,
            child=pan,
            origin=Origin(xyz=(sx * r.tip_x, 0.0, HANG_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.0, velocity=1.5, lower=-r.pan_swing, upper=r.pan_swing
            ),
        )


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------
def build_scale(
    config: ScaleConfig | None = None,
    *,
    assets: AssetContext | None = None,
) -> ArticulatedObject:
    r = resolve_config(config)
    model = ArticulatedObject(name=r.name, assets=assets)
    mats = {
        key: model.material(f"scale_{key}_{r.palette_style}", rgba=rgba)
        for key, rgba in PALETTES[r.palette_style].items()
    }

    pedestal = _build_pedestal(model, r, mats, assets=assets)
    beam = _build_beam(model, r, pedestal, mats)
    _build_pans(model, r, beam, mats)

    model.meta["slot_choices"] = slot_choices_for_config(r)
    return model


def build_seeded_scale(
    seed: int,
    *,
    assets: AssetContext | None = None,
) -> ArticulatedObject:
    return build_scale(config_from_seed(seed), assets=assets)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_scale_tests(
    object_model: ArticulatedObject,
    config: ScaleConfig,
) -> TestReport:
    r = resolve_config(config)
    ctx = TestContext(object_model)

    pedestal = object_model.get_part("pedestal")
    beam = object_model.get_part("beam")
    pan_0 = object_model.get_part("pan_0")
    pan_1 = object_model.get_part("pan_1")

    pivot = object_model.get_articulation("pedestal_to_beam")
    swing_0 = object_model.get_articulation("beam_to_pan_0")
    swing_1 = object_model.get_articulation("beam_to_pan_1")

    # ---- Captured-pin / hook-on-ring allowances (element-scoped). ----
    ctx.allow_overlap(
        beam,
        pedestal,
        elem_a="pivot_axle",
        elem_b="pivot_head",
        reason="Beam pivot axle passes through the clevis fork plates at the column top.",
    )
    for pan, ball, pin in (
        (pan_0, "hang_ball_0", "hang_pin_0"),
        (pan_1, "hang_ball_1", "hang_pin_1"),
    ):
        ctx.allow_overlap(
            pan,
            beam,
            elem_a="hanger_top",
            elem_b=ball,
            reason="Hanger top ring is threaded around the beam-tip hang ball (hook-on-ring joint).",
        )
        ctx.allow_overlap(
            pan,
            beam,
            elem_a="hanger_top",
            elem_b=pin,
            reason="Hanger top ring encircles the drop pin above the hang ball.",
        )

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_joint_mating_has_gap()

    # ---- Structure / identity. ----
    part_names = {p.name for p in object_model.parts}
    ctx.check(
        "pedestal + beam + 2 pans present (pan_count==2 invariant)",
        {"pedestal", "beam", "pan_0", "pan_1"} <= part_names
        and sum(1 for n in part_names if n.startswith("pan_")) == 2,
        details=str(sorted(part_names)),
    )

    # ---- Pedestal grounded. ----
    ped_aabb = ctx.part_world_aabb(pedestal)
    if ped_aabb is not None:
        ctx.check(
            "pedestal sits on the ground plane",
            ped_aabb[0][2] < 0.012,
            details=f"pedestal zmin={ped_aabb[0][2]:.4f}",
        )

    # ---- 3 REVOLUTE joints about +Y with symmetric geometry-derived ranges.
    #      The beam-tilt and pan-swing limits are derived (Contract 3c) so the
    #      swung/tilted pans clear the pedestal; both are strictly positive. ----
    for joint, lim_mag in ((pivot, r.beam_tilt), (swing_0, r.pan_swing), (swing_1, r.pan_swing)):
        lim = joint.motion_limits
        ctx.check(
            f"{joint.name} is REVOLUTE about +Y, symmetric range +/-{lim_mag:.4f}",
            joint.articulation_type == ArticulationType.REVOLUTE
            and abs(joint.axis[1]) > 0.99
            and lim is not None
            and lim_mag > 0.0
            and abs(lim.lower + lim_mag) < 1e-6
            and abs(lim.upper - lim_mag) < 1e-6,
            details=f"{joint.name}: axis={tuple(joint.axis)} lower={lim.lower if lim else None} upper={lim.upper if lim else None} expected=+/-{lim_mag:.4f}",
        )
    ctx.check(
        "beam pivot origin at the column top (PIVOT_Z)",
        abs(pivot.origin.xyz[2] - r.pivot_z) < 1e-6,
        details=f"origin={pivot.origin.xyz} pivot_z={r.pivot_z:.4f}",
    )
    for idx, sx, swing in ((0, 1.0, swing_0), (1, -1.0, swing_1)):
        ctx.check(
            f"beam_to_pan_{idx} origin at beam tip x={sx * r.tip_x:+.3f}, z=HANG_Z",
            abs(swing.origin.xyz[0] - sx * r.tip_x) < 1e-6
            and abs(swing.origin.xyz[2] - HANG_Z) < 1e-6,
            details=f"origin={swing.origin.xyz}",
        )

    # ---- chain_count encoded as a non-moving multiplicity (Rule 1). ----
    chain_visuals = [v.name for v in pan_0.visuals if v.name.startswith("chain_")]
    ctx.check(
        "N chain visuals inlined on the pan (Rule 1, no extra joints)",
        len(chain_visuals) == r.chain_count,
        details=f"chains={sorted(chain_visuals)} expected N={r.chain_count}",
    )

    # ---- pans hang on the beam tips (ring threaded over the hang ball). ----
    ctx.expect_overlap(
        pan_0, beam, axes="z", min_overlap=0.004, name="pan_0 ring hangs on beam tip"
    )
    ctx.expect_overlap(
        pan_1, beam, axes="z", min_overlap=0.004, name="pan_1 ring hangs on beam tip"
    )

    # ---- weighing tilt: at the derived beam-tilt limit +q drops one pan and
    #      raises the other; the drop tracks the analytic prediction (locks the
    #      derived limit to real geometry). ----
    rest_z0 = ctx.part_world_position(pan_0)[2]
    rest_z1 = ctx.part_world_position(pan_1)[2]
    with ctx.pose({pivot: r.beam_tilt}):
        tilt_z0 = ctx.part_world_position(pan_0)[2]
        tilt_z1 = ctx.part_world_position(pan_1)[2]
    expected_drop = r.tip_x * math.sin(r.beam_tilt)
    ctx.check(
        "tilting the beam lowers one pan and raises the other (beam-tilt limit)",
        tilt_z0 < rest_z0 - 0.4 * expected_drop
        and tilt_z1 > rest_z1 + 0.4 * expected_drop
        and expected_drop > 0.0,
        details=f"rest=({rest_z0:.4f},{rest_z1:.4f}) tilted=({tilt_z0:.4f},{tilt_z1:.4f}) expected_drop={expected_drop:.4f}",
    )

    # ---- pan swing: at the derived swing limit the vessel displaces off-axis in
    #      X, tracking the analytic prediction. ----
    rest_aabb = ctx.part_world_aabb(pan_0)
    rest_cx = 0.5 * (rest_aabb[0][0] + rest_aabb[1][0])
    with ctx.pose({swing_0: r.pan_swing}):
        swung_aabb = ctx.part_world_aabb(pan_0)
    swung_cx = 0.5 * (swung_aabb[0][0] + swung_aabb[1][0])
    z_ref = min(zp for _, zp in _vessel_profile(r.pan_hanger, r.pan_size_scale))
    expected_dx = abs(z_ref) * math.sin(r.pan_swing)
    ctx.check(
        "swinging pan_0 displaces the vessel sideways (off-axis proof, swing limit)",
        abs(swung_cx - rest_cx) > 0.4 * expected_dx and expected_dx > 0.0,
        details=f"rest cx={rest_cx:.4f}, swung cx={swung_cx:.4f}, expected_dx={expected_dx:.4f}",
    )

    # ---- MOTION-QC LOCK: at the binding joint-limit corners the pans clear the
    #      pedestal (no pan-into-leg / pan-into-column 穿模). pan_0 swings worst
    #      toward the pedestal at (+beam_tilt, +swing) and pan_1 at
    #      (-beam_tilt, -swing); we check both same-sign corners (the census 穿模
    #      poses) so the derived beam stop is enforced on the real mesh, not just
    #      analytically. The full envelope is covered cheaply by the analytic
    #      clearance lock below and by the external motion-QC sweep. ----
    corner_poses = [
        {pivot: r.beam_tilt, swing_0: r.pan_swing, swing_1: r.pan_swing},
        {pivot: -r.beam_tilt, swing_0: -r.pan_swing, swing_1: -r.pan_swing},
        {pivot: r.beam_tilt, swing_0: r.pan_swing, swing_1: -r.pan_swing},
        {pivot: -r.beam_tilt, swing_0: -r.pan_swing, swing_1: r.pan_swing},
    ]
    corner_ok = True
    for pose in corner_poses:
        with ctx.pose(pose):
            if not ctx.fail_if_parts_overlap_in_current_pose(
                name="pan clears pedestal at joint-limit corner"
            ):
                corner_ok = False
    ctx.check(
        "pans clear the pedestal at the binding joint-limit corners (motion-QC lock)",
        corner_ok,
        details=f"beam_tilt={r.beam_tilt:.4f} pan_swing={r.pan_swing:.4f}",
    )

    # ---- analytic clearance lock: the derived envelope keeps >= CLEAR_MARGIN. ----
    min_clr = _min_clearance(
        r.beam_tilt,
        r.pan_swing,
        _vessel_profile(r.pan_hanger, r.pan_size_scale),
        r.pedestal_column,
        r.base_footprint_scale,
        r.column_height_scale,
        r.pivot_z,
        r.tip_x,
    )
    ctx.check(
        "derived motion limits keep pans clear of the pedestal envelope",
        min_clr >= SAFE_MARGIN - 1e-6,
        details=f"min_clearance={min_clr * 1000:.2f}mm target={CLEAR_MARGIN * 1000:.1f}mm safe_floor={SAFE_MARGIN * 1000:.1f}mm",
    )

    # ---- pan clearance: the two pans don't collide with each other (the
    #      resolve_config clearance inequality keeps each pan vessel inside its
    #      half-span; actual pedestal collision is caught by the overlap
    #      baseline above). ----
    a0 = ctx.part_world_aabb(pan_0)
    a1 = ctx.part_world_aabb(pan_1)
    if a0 is not None and a1 is not None:
        ctx.check(
            "the two pans do not overlap each other in X at rest",
            a0[0][0] > a1[1][0] - 1e-6,
            details=f"pan_0 xmin={a0[0][0]:.4f} pan_1 xmax={a1[1][0]:.4f}",
        )
    # Clearance inequality proven from authored dims (pan vessel inside span).
    pan_r = DISH_R * r.pan_size_scale
    ctx.check(
        "pan vessel radius fits within the beam half-span",
        pan_r < r.tip_x,
        details=f"pan_r={pan_r:.4f} tip_x={r.tip_x:.4f}",
    )

    # ---- slot_choices recorded with N encoded. ----
    ctx.check(
        "slot_choices recorded with chain_count encoded",
        tuple(object_model.meta.get("slot_choices", ())) == slot_choices_for_config(r),
        details=str(object_model.meta.get("slot_choices")),
    )

    return ctx.report()


__all__ = (
    "ScaleConfig",
    "ResolvedScaleConfig",
    "build_scale",
    "build_seeded_scale",
    "config_from_seed",
    "resolve_config",
    "run_scale_tests",
    "slot_choices_for_config",
    "slot_choices_for_seed",
    "with_overrides",
)
