"""Modular sheathed sword template.

A cold-weapon sword (`sword` = blade + hilt as one rigid part) that slides
prismatically out of a grounded hollow scabbard (`scabbard` chassis), with N
swinging suspension rings (REVOLUTE) hung off lug pins along the scabbard body.

Slot graph (pattern = mixed):

    scabbard (grounded chassis: body⊃cavity / chape / throat / N×lug)
      │
      ├─ [sword_draw  PRISMATIC +X, 0..0.50·length_scale]  →  sword
      │       Slot A (blade_profile)  ┐  both emit visuals onto the SAME
      │       Slot B (hilt)           ┘  `sword` part (parallel functional
      │                                   layers, no inter-slot chain joint)
      └─ [band_i_pivot REVOLUTE +Y ±60°] × N  →  band_i_ring (multiplicity)

Slot A — blade_profile (4): leaf_double_edge / straight_double_edge /
    curved_saber / broad_triangular.  The profile sizes the scabbard cavity /
    body / throat and, for ``curved_saber``, bows the whole scabbard.
Slot B — hilt (3): box_bead / cruciform_disc / knuckle_scentstop.
Multiplicity — ring_count N ∈ [1, 8] (weighted toward small N).

Identity invariants on every seed: exactly one ``sword_draw`` PRISMATIC along
+X (the blade draws out of the scabbard), the sheathed blade nests inside the
hollow cavity, and N captured suspension rings swing about +Y pins.

5-star module sources (see specs_modular_v1/Military_Military_sword.md → Module Source Index):
S_parent (gladius), S_straight, S_saber, S_broad, S_cross, S_knuckle, S_bands4,
S_bands6.
"""

from __future__ import annotations

import math
import random
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

import cadquery as cq
from cadquery.func import loft as _cq_loft
from cadquery.func import segment as _seg
from cadquery.func import wire as _wire

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

__modular__ = True

# --------------------------------------------------------------------------- #
# Shared scabbard constants (S_parent layout, ~0.75 m, lying along +X).
# --------------------------------------------------------------------------- #
ZC = 0.01475          # scabbard centreline height (throat half-thickness)
MOUTH_X = 0.50        # scabbard mouth plane (prismatic origin x)
DRAW_UPPER = 0.50     # nominal sword draw travel (scaled by length_scale)

PIN_Z = 0.02275
RING_R = 0.0105
RING_TUBE = 0.0022
PIN_R = 0.0022
# Ring hangs RING_HANG below the pin (off-axis) so the swing AABB is visible
# and the torus inner-top seats 0.4 mm into the pin top (captured contact).
RING_HANG = RING_R - RING_TUBE - PIN_R + 0.0004   # 0.0065 ring-center drop
RING_LIMIT = math.radians(60.0)
SWING_AXIS_Y = (0.0, 1.0, 0.0)

# Banded-ring placement window (clear of chape ball + throat band) — S_bands6.
RING_X_START = 0.13
RING_X_END = 0.42

# Lug geometry (S_bands4 _lug_positions helper).
_FLANGE_R = 0.005
_FLANGE_LEN = 0.003
_PIN_LEN = 0.0155
_HEAD_R = 0.0048
_HEAD_LEN = 0.0035
_FLANGE_GAP = 0.0027
_PIN_INSET = 0.0015

# --------------------------------------------------------------------------- #
# Slot enumerations.
# --------------------------------------------------------------------------- #
BladeProfile = Literal[
    "leaf_double_edge", "straight_double_edge", "curved_saber", "broad_triangular"
]
HiltChoice = Literal["box_bead", "cruciform_disc", "knuckle_scentstop"]
PaletteStyle = Literal["steel_bronze", "blackened_iron", "brass_leather", "silver"]

BLADE_PROFILES: tuple[BladeProfile, ...] = (
    "leaf_double_edge",
    "straight_double_edge",
    "curved_saber",
    "broad_triangular",
)
HILT_CHOICES: tuple[HiltChoice, ...] = (
    "box_bead",
    "cruciform_disc",
    "knuckle_scentstop",
)
PALETTE_STYLES: tuple[PaletteStyle, ...] = (
    "steel_bronze",
    "blackened_iron",
    "brass_leather",
    "silver",
)

# Hilts with bulky transverse hardware near the mouth — gated against high bow.
_WIDE_HILTS = frozenset({"cruciform_disc", "knuckle_scentstop"})

RING_COUNT_MIN, RING_COUNT_MAX = 1, 8
_RING_COUNT_WEIGHTS = {1: 0.10, 2: 0.34, 3: 0.18, 4: 0.16, 5: 0.08, 6: 0.07, 7: 0.04, 8: 0.03}

# --------------------------------------------------------------------------- #
# Per-profile blade + cavity geometry (sword-local frame: base ~0, tip ~ -0.45).
# --------------------------------------------------------------------------- #
BLADE_X_BASE = 0.005
BLADE_TIP_X = -0.45
BLADE_LEN = BLADE_X_BASE - BLADE_TIP_X     # 0.455
BLADE_SHOULDER_X = -0.39
BLADE_THICK_NOM = 0.006

SABER_BOW_MAX = 0.018         # full lateral bow at blade midpoint
SABER_BOW_GATED = 0.010       # capped bow when paired with a wide hilt
SABER_CUT_HW = 0.026          # cutting-edge half-width at base (+Y)
SABER_SPINE_HW = 0.014        # spine half-width at base (-Y)

# Palettes — each maps the named material tokens used by every visual. ≥3
# distinct colours per palette so per-seed palette diversity is real.
PALETTES: dict[PaletteStyle, dict[str, tuple[float, float, float, float]]] = {
    "steel_bronze": {
        "steel": (0.62, 0.63, 0.66, 1.0),
        "steel_dark": (0.46, 0.48, 0.53, 1.0),
        "brass": (0.71, 0.54, 0.20, 1.0),
        "brass_dark": (0.52, 0.38, 0.13, 1.0),
        "amber": (0.80, 0.42, 0.10, 1.0),
        "burl": (0.42, 0.22, 0.12, 1.0),
        "tan": (0.79, 0.63, 0.44, 1.0),
        "gold": (0.87, 0.68, 0.24, 1.0),
    },
    "blackened_iron": {
        "steel": (0.40, 0.41, 0.44, 1.0),
        "steel_dark": (0.20, 0.21, 0.24, 1.0),
        "brass": (0.30, 0.30, 0.33, 1.0),
        "brass_dark": (0.16, 0.16, 0.18, 1.0),
        "amber": (0.28, 0.18, 0.12, 1.0),
        "burl": (0.18, 0.14, 0.12, 1.0),
        "tan": (0.45, 0.40, 0.34, 1.0),
        "gold": (0.66, 0.56, 0.30, 1.0),
    },
    "brass_leather": {
        "steel": (0.70, 0.70, 0.72, 1.0),
        "steel_dark": (0.50, 0.50, 0.54, 1.0),
        "brass": (0.78, 0.60, 0.24, 1.0),
        "brass_dark": (0.58, 0.42, 0.15, 1.0),
        "amber": (0.55, 0.30, 0.14, 1.0),
        "burl": (0.36, 0.20, 0.10, 1.0),
        "tan": (0.62, 0.44, 0.28, 1.0),
        "gold": (0.90, 0.74, 0.34, 1.0),
    },
    "silver": {
        "steel": (0.78, 0.79, 0.82, 1.0),
        "steel_dark": (0.58, 0.60, 0.64, 1.0),
        "brass": (0.74, 0.75, 0.78, 1.0),
        "brass_dark": (0.50, 0.51, 0.55, 1.0),
        "amber": (0.66, 0.50, 0.40, 1.0),
        "burl": (0.30, 0.26, 0.28, 1.0),
        "tan": (0.70, 0.70, 0.74, 1.0),
        "gold": (0.85, 0.84, 0.70, 1.0),
    },
}


# --------------------------------------------------------------------------- #
# Config dataclasses.
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class SwordConfig:
    """Public configuration sampled by ``config_from_seed`` or supplied directly."""

    blade_profile: BladeProfile = "leaf_double_edge"
    hilt: HiltChoice = "box_bead"
    ring_count: int = 2
    palette_style: PaletteStyle = "steel_bronze"
    length_scale: float = 1.0
    blade_thick_scale: float = 1.0
    ring_x_spacing_scale: float = 1.0
    name: str = "reference_sword"


@dataclass(frozen=True)
class ResolvedSwordConfig:
    blade_profile: BladeProfile
    hilt: HiltChoice
    ring_count: int
    palette_style: PaletteStyle
    length_scale: float
    blade_thick_scale: float
    ring_x_spacing_scale: float
    name: str
    # derived
    palette: dict[str, tuple[float, float, float, float]]
    is_curved: bool
    bow_max: float           # 0.0 for straight profiles
    blade_thick: float
    draw_upper: float        # = DRAW_UPPER · length_scale
    alternate_sides: bool    # ring side policy: alternate +Y/-Y when True


def _clamp(value: float, lo: float, hi: float) -> float:
    if lo > hi:
        lo, hi = hi, lo
    return max(lo, min(hi, float(value)))


def _pick(value, choices):
    return value if value in choices else choices[0]


def _weighted_ring_count(rng: random.Random) -> int:
    counts = list(_RING_COUNT_WEIGHTS.keys())
    weights = [_RING_COUNT_WEIGHTS[c] for c in counts]
    n = rng.choices(counts, weights=weights, k=1)[0]
    return int(_clamp(n, RING_COUNT_MIN, RING_COUNT_MAX))


def config_from_seed(seed: int) -> SwordConfig:
    """Deterministic per-seed procedural sampling (seed=0 not special).

    Order: weighted ring_count → uniform blade_profile & hilt → palette →
    continuous scales.  The compatibility gate (saber × wide hilt) is applied
    in ``resolve_config`` by capping the bow, never by rejecting a seed.
    """
    rng = random.Random(seed)

    ring_count = _weighted_ring_count(rng)
    blade_profile: BladeProfile = rng.choice(BLADE_PROFILES)
    hilt: HiltChoice = rng.choice(HILT_CHOICES)
    palette: PaletteStyle = rng.choice(PALETTE_STYLES)

    length_scale = round(rng.uniform(0.92, 1.10), 4)
    blade_thick_scale = round(rng.uniform(0.85, 1.20), 4)
    ring_x_spacing_scale = round(rng.uniform(0.85, 1.10), 4)

    return SwordConfig(
        blade_profile=blade_profile,
        hilt=hilt,
        ring_count=ring_count,
        palette_style=palette,
        length_scale=length_scale,
        blade_thick_scale=blade_thick_scale,
        ring_x_spacing_scale=ring_x_spacing_scale,
        name=f"seeded_sword_{seed}",
    )


def resolve_config(config: SwordConfig) -> ResolvedSwordConfig:
    if config.palette_style not in PALETTES:
        raise ValueError(f"Unsupported palette_style: {config.palette_style}")

    blade_profile = _pick(config.blade_profile, BLADE_PROFILES)
    hilt = _pick(config.hilt, HILT_CHOICES)
    is_curved = blade_profile == "curved_saber"

    # Compatibility gate (conditional): curved saber bows the whole scabbard;
    # paired with a bulky transverse hilt the bowed mouth + wide hardware risk
    # collision, so cap the bow to the gated value.  Otherwise full bow.
    if is_curved:
        bow_max = SABER_BOW_GATED if hilt in _WIDE_HILTS else SABER_BOW_MAX
    else:
        bow_max = 0.0

    ring_count = int(_clamp(config.ring_count, RING_COUNT_MIN, RING_COUNT_MAX))

    length_scale = _clamp(config.length_scale, 0.92, 1.10)
    blade_thick_scale = _clamp(config.blade_thick_scale, 0.85, 1.20)
    ring_x_spacing_scale = _clamp(config.ring_x_spacing_scale, 0.85, 1.10)

    blade_thick = BLADE_THICK_NOM * blade_thick_scale
    draw_upper = DRAW_UPPER * length_scale

    # Side policy (module-local fixed strategy, not a new slot): single +Y side
    # for small N (parent look), alternate +Y/-Y for N >= 4 (S_bands6 balance).
    alternate_sides = ring_count >= 4

    return ResolvedSwordConfig(
        blade_profile=blade_profile,
        hilt=hilt,
        ring_count=ring_count,
        palette_style=config.palette_style,
        length_scale=length_scale,
        blade_thick_scale=blade_thick_scale,
        ring_x_spacing_scale=ring_x_spacing_scale,
        name=config.name,
        palette=dict(PALETTES[config.palette_style]),
        is_curved=is_curved,
        bow_max=bow_max,
        blade_thick=blade_thick,
        draw_upper=draw_upper,
        alternate_sides=alternate_sides,
    )


# --------------------------------------------------------------------------- #
# Profile geometry parameter tables (outer body / chape / cavity / throat).
# Each profile returns (body_secs, chape_secs, cav_secs, throat_dims) where
# sections are (x, width, thickness) tuples (straight loft) — these are bowed
# in +Y by ``_scabbard_bow`` for the curved saber.
# --------------------------------------------------------------------------- #
def _profile_sections(profile: BladeProfile):
    if profile == "broad_triangular":
        body = [(0.085, 0.042, 0.0215), (0.500, 0.094, 0.0255)]
        chape = [(0.016, 0.018, 0.009), (0.055, 0.044, 0.020), (0.095, 0.060, 0.0295)]
        cav = [
            (0.045, 0.016, 0.009),
            (0.115, 0.048, 0.0105),
            (0.300, 0.066, 0.0105),
            (0.505, 0.082, 0.0105),
        ]
        throat = dict(w=0.100, t=0.0295, hole_w=0.086, hole_t=0.025)
    elif profile == "straight_double_edge":
        body = [(0.085, 0.050, 0.0215), (0.500, 0.056, 0.0255)]
        chape = [(0.016, 0.013, 0.009), (0.055, 0.032, 0.020), (0.095, 0.050, 0.0295)]
        cav = [
            (0.045, 0.008, 0.008),
            (0.100, 0.044, 0.011),
            (0.505, 0.044, 0.011),
        ]
        throat = dict(w=0.062, t=0.0295, hole_w=0.0555, hole_t=0.025)
    elif profile == "curved_saber":
        body = [(0.085, 0.046, 0.024), (0.500, 0.062, 0.028)]
        # Deepest chape section thinned to 0.030 so its underside stays at/above
        # the ground plane at the shared centreline height ZC (was 0.033 → dip).
        chape = [(0.016, 0.015, 0.011), (0.055, 0.036, 0.023), (0.095, 0.054, 0.030)]
        cav = [
            (0.048, 0.010, 0.010),
            (0.100, 0.030, 0.011),
            (0.250, 0.044, 0.011),
            (0.505, 0.054, 0.011),
        ]
        throat = dict(w=0.066, t=0.032, hole_w=0.058, hole_t=0.014)
    else:  # leaf_double_edge (parent)
        body = [(0.085, 0.042, 0.0215), (0.500, 0.056, 0.0255)]
        chape = [(0.016, 0.013, 0.009), (0.055, 0.032, 0.020), (0.095, 0.050, 0.0295)]
        cav = [
            (0.045, 0.0075, 0.0075),
            (0.115, 0.0360, 0.0105),
            (0.505, 0.0515, 0.0105),
        ]
        throat = dict(w=0.062, t=0.0295, hole_w=0.0555, hole_t=0.025)
    return body, chape, cav, throat


# --------------------------------------------------------------------------- #
# Geometry helpers (straight loft = S_parent; offset loft = S_saber).
# --------------------------------------------------------------------------- #
def _lerp_sections(secs, x: float) -> tuple[float, float]:
    for (xa, wa, ta), (xb, wb, tb) in zip(secs, secs[1:]):
        if xa <= x <= xb:
            f = (x - xa) / (xb - xa)
            return wa + (wb - wa) * f, ta + (tb - ta) * f
    raise ValueError(f"x={x} outside section range")


def _loft(secs) -> cq.Workplane:
    """Ruled loft of centred rectangles in YZ planes along +X at height ZC."""
    wp = cq.Workplane("YZ", origin=(secs[0][0], 0.0, ZC)).rect(secs[0][1], secs[0][2])
    prev_x = secs[0][0]
    for x, w, t in secs[1:]:
        wp = wp.workplane(offset=x - prev_x).rect(w, t)
        prev_x = x
    return wp.loft(ruled=True)


def _offset_loft(sections_xywt) -> cq.Workplane:
    """Loft rectangles in YZ planes at (x, y_off, ZC). sections=(x, y_off, w, t)."""
    profiles = []
    for x, y_off, w, t in sections_xywt:
        hw, ht = w / 2.0, t / 2.0
        e1 = _seg((x, y_off - hw, ZC - ht), (x, y_off + hw, ZC - ht))
        e2 = _seg((x, y_off + hw, ZC - ht), (x, y_off + hw, ZC + ht))
        e3 = _seg((x, y_off + hw, ZC + ht), (x, y_off - hw, ZC + ht))
        e4 = _seg((x, y_off - hw, ZC + ht), (x, y_off - hw, ZC - ht))
        profiles.append(_wire(e1, e2, e3, e4))
    solid = _cq_loft(*profiles, cap=True)
    return cq.Workplane("XY").newObject([solid])


def _bow(x: float, x_base: float, x_tip: float, length: float, bow_max: float) -> float:
    if x >= x_base or x <= x_tip:
        return 0.0
    t = (x_base - x) / length
    return bow_max * 4.0 * t * (1.0 - t)


def _make_scabbard_bow(bow_max: float):
    """Scabbard-frame bow(x) (x is scabbard-world; blade base maps to MOUTH_X)."""
    x_base = MOUTH_X + BLADE_X_BASE
    x_tip = MOUTH_X + BLADE_TIP_X

    def bow(x: float) -> float:
        return _bow(x, x_base, x_tip, BLADE_LEN, bow_max)

    return bow


def _make_sword_bow(bow_max: float):
    def bow(x: float) -> float:
        return _bow(x, BLADE_X_BASE, BLADE_TIP_X, BLADE_LEN, bow_max)

    return bow


def _curved_sections(base_xwt, bow_fn):
    return [(x, bow_fn(x), w, t) for x, w, t in base_xwt]


def _densify(secs, n: int):
    """Resample width/thickness piecewise-linearly into n+1 stations along x."""
    x0 = secs[0][0]
    x1 = secs[-1][0]
    out = []
    for i in range(n + 1):
        x = x0 + (x1 - x0) * i / n
        w, t = _lerp_sections(secs, x)
        out.append((x, w, t))
    return out


def _ring_solid() -> cq.Shape:
    return cq.Solid.makeTorus(RING_R, RING_TUBE, cq.Vector(0, 0, 0), cq.Vector(0, 1, 0))


def _band_wrap(bx: float, bw: float, bt: float) -> cq.Workplane:
    """Hollow rectangular brass band wrapping the body at a ring station."""
    pad = 0.0015
    outer = (
        cq.Workplane("YZ", origin=(bx, 0.0, ZC))
        .rect(bw + 2.0 * pad, bt + 2.0 * pad)
        .extrude(0.004, both=True)
    )
    inner = (
        cq.Workplane("YZ", origin=(bx, 0.0, ZC))
        .rect(bw - 0.0002, bt - 0.0002)
        .extrude(0.005, both=True)
    )
    return outer.cut(inner)


def _lug_positions(body_half_w: float) -> dict[str, float]:
    """Y centres for flange/pin/head + joint origin on the +Y side (S_bands4)."""
    flange_inner = body_half_w + _FLANGE_GAP
    flange_cy = flange_inner + _FLANGE_LEN / 2.0
    pin_start = flange_inner - _PIN_INSET
    pin_cy = pin_start + _PIN_LEN / 2.0
    head_start = pin_start + _PIN_LEN
    head_cy = head_start + _HEAD_LEN / 2.0
    joint_cy = pin_cy + 0.00125
    return {"flange_cy": flange_cy, "pin_cy": pin_cy, "head_cy": head_cy, "joint_cy": joint_cy}


# --------------------------------------------------------------------------- #
# Blade module factories (Slot A) — emit blade visuals onto the `sword` part.
# --------------------------------------------------------------------------- #
def _leaf_blade_solid(thick: float) -> cq.Workplane:
    hw_base, hw_shoulder = 0.023, 0.015
    pts = [
        (0.005, hw_base),
        (BLADE_SHOULDER_X, hw_shoulder),
        (BLADE_TIP_X, 0.0),
        (BLADE_SHOULDER_X, -hw_shoulder),
        (0.005, -hw_base),
    ]
    return cq.Workplane("XY").polyline(pts).close().extrude(thick / 2.0, both=True)


def _straight_blade_solid(thick: float) -> cq.Workplane:
    hw, taper_x = 0.020, -0.39
    pts = [
        (0.005, hw),
        (taper_x, hw),
        (BLADE_TIP_X, 0.0),
        (taper_x, -hw),
        (0.005, -hw),
    ]
    return cq.Workplane("XY").polyline(pts).close().extrude(thick / 2.0, both=True)


def _broad_blade_solid(thick: float) -> cq.Workplane:
    hw_base = 0.038
    pts = [(0.005, hw_base), (BLADE_TIP_X, 0.0), (0.005, -hw_base)]
    return cq.Workplane("XY").polyline(pts).close().extrude(thick / 2.0, both=True)


def _broad_spine_solid(thick: float) -> cq.Workplane:
    spine_hw = 0.008
    # Proud ridge half-thickness held to a fixed cap (independent of the blade
    # thickness scale) so the spine never exceeds the broad cavity z-half
    # (0.00525) and grazes the chape roof.  Half-thickness = 0.0046 max.
    half_t = min(thick / 2.0 + 0.0010, 0.0046)
    pts = [(0.005, spine_hw), (BLADE_TIP_X + 0.02, 0.0), (0.005, -spine_hw)]
    return cq.Workplane("XY").polyline(pts).close().extrude(half_t, both=True)


def _curved_blade_solid(thick: float, bow_fn) -> cq.Workplane:
    n = 32
    edge_pts: list[tuple[float, float]] = []
    spine_pts: list[tuple[float, float]] = []
    for i in range(n + 1):
        t = i / n
        x = BLADE_X_BASE - t * BLADE_LEN
        bow = bow_fn(x)
        taper = max(1.0 - 0.88 * t**0.7, 0.0)
        edge_pts.append((x, bow + SABER_CUT_HW * taper))
        spine_pts.append((x, bow - SABER_SPINE_HW * taper))
    outline = edge_pts + list(reversed(spine_pts))
    return cq.Workplane("XY").polyline(outline).close().extrude(thick / 2.0, both=True)


def _build_blade(sword, r: ResolvedSwordConfig, mats: dict) -> None:
    thick = r.blade_thick
    if r.blade_profile == "leaf_double_edge":
        sword.visual(
            mesh_from_cadquery(_leaf_blade_solid(thick), "blade"),
            material=mats["steel"], name="blade",
        )
        sword.visual(
            Box((0.385, 0.005, 0.009)),
            origin=Origin(xyz=(-0.1875, 0.0, 0.0)),
            material=mats["steel_dark"], name="blade_spine",
        )
    elif r.blade_profile == "straight_double_edge":
        sword.visual(
            mesh_from_cadquery(_straight_blade_solid(thick), "blade"),
            material=mats["steel"], name="blade",
        )
        sword.visual(
            Box((0.385, 0.005, 0.009)),
            origin=Origin(xyz=(-0.1875, 0.0, 0.0)),
            material=mats["steel_dark"], name="blade_spine",
        )
    elif r.blade_profile == "broad_triangular":
        sword.visual(
            mesh_from_cadquery(_broad_blade_solid(thick), "blade"),
            material=mats["steel"], name="blade",
        )
        sword.visual(
            mesh_from_cadquery(_broad_spine_solid(thick), "blade_spine"),
            material=mats["steel_dark"], name="blade_spine",
        )
    else:  # curved_saber — no separate blade_spine (spine is part of the loft)
        bow_fn = _make_sword_bow(r.bow_max)
        sword.visual(
            mesh_from_cadquery(_curved_blade_solid(thick, bow_fn), "blade"),
            material=mats["steel"], name="blade",
        )


# --------------------------------------------------------------------------- #
# Hilt module factories (Slot B) — emit hilt visuals onto the `sword` part.
# All visuals live on the +X half of the `sword` local frame.
# --------------------------------------------------------------------------- #
def _pommel_oblate_solid() -> cq.Shape:
    sphere = cq.Workplane().sphere(1.0).val()
    mat = cq.Matrix([
        [0.024, 0.0, 0.0, 0.0],
        [0.0, 0.024, 0.0, 0.0],
        [0.0, 0.0, 0.0145, 0.0],
    ])
    return sphere.transformGeometry(mat)


def _crossguard_solid() -> cq.Workplane:
    bar = cq.Workplane("XZ").rect(0.020, 0.012).extrude(0.10, both=True)
    center = cq.Workplane("XZ").rect(0.030, 0.018).extrude(0.019, both=True)
    return bar.union(center)


def _disc_pommel_solid() -> cq.Workplane:
    disc = cq.Workplane("YZ").circle(0.014).extrude(0.007, both=True)
    boss_outer = cq.Workplane("YZ", origin=(0.007, 0.0, 0.0)).circle(0.005).extrude(0.003)
    boss_inner = cq.Workplane("YZ", origin=(-0.007, 0.0, 0.0)).circle(0.005).extrude(-0.003)
    return disc.union(boss_outer).union(boss_inner)


def _scent_stopper_pommel(secs) -> cq.Workplane:
    x0, r0 = secs[0]
    wp = cq.Workplane("YZ", origin=(x0, 0.0, 0.0)).polygon(8, r0 * 2.0)
    prev_x = x0
    for x, rr in secs[1:]:
        wp = wp.workplane(offset=x - prev_x).polygon(8, rr * 2.0)
        prev_x = x
    return wp.loft(ruled=True)


def _emit_grip_collar(sword, mats: dict) -> None:
    """Twisted gold spiral collar — shared across hilts (S_parent L262-L272)."""
    for i in range(4):
        ang = math.radians(30.0 + i * 120.0)
        sword.visual(
            Sphere(0.0125),
            origin=Origin(xyz=(0.082 + i * 0.016, 0.0012 * math.cos(ang), 0.0012 * math.sin(ang))),
            material=mats["gold"], name=f"grip_collar_bead_{i}",
        )


def _build_hilt_box_bead(sword, mats: dict) -> None:
    sword.visual(
        Box((0.050, 0.060, 0.028)),
        origin=Origin(xyz=(0.0255, 0.0, 0.0)),
        material=mats["brass"], name="guard",
    )
    for i, gx in enumerate((0.014, 0.037)):
        sword.visual(
            Box((0.016, 0.040, 0.0016)),
            origin=Origin(xyz=(gx, 0.0, 0.0143)),
            material=mats["brass_dark"], name=f"guard_relief_{i}",
        )
    sword.visual(
        Cylinder(radius=0.009, length=0.127),
        origin=Origin(xyz=(0.1085, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mats["amber"], name="grip_core",
    )
    sword.visual(
        Cylinder(radius=0.0148, length=0.029),
        origin=Origin(xyz=(0.0635, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mats["amber"], name="grip_lower",
    )
    _emit_grip_collar(sword, mats)
    sword.visual(
        Cylinder(radius=0.0148, length=0.034),
        origin=Origin(xyz=(0.151, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mats["amber"], name="grip_upper",
    )
    sword.visual(
        mesh_from_cadquery(_pommel_oblate_solid(), "pommel"),
        origin=Origin(xyz=(0.190, 0.0, 0.0)),
        material=mats["amber"], name="pommel",
    )
    sword.visual(
        Cylinder(radius=0.0055, length=0.021),
        origin=Origin(xyz=(0.2205, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mats["brass"], name="finial_stem",
    )
    sword.visual(
        Sphere(0.0078),
        origin=Origin(xyz=(0.236, 0.0, 0.0)),
        material=mats["brass"], name="finial_ball",
    )


def _build_hilt_cruciform_disc(sword, mats: dict) -> None:
    guard_x = 0.015
    sword.visual(
        mesh_from_cadquery(_crossguard_solid(), "crossguard"),
        origin=Origin(xyz=(guard_x, 0.0, 0.0)),
        material=mats["brass"], name="guard",
    )
    for i, gz in enumerate((0.0065, -0.0065)):
        sword.visual(
            Cylinder(radius=0.0015, length=0.18),
            origin=Origin(xyz=(guard_x, 0.0, gz), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=mats["brass_dark"], name=f"guard_groove_{i}",
        )
    for i, sy in enumerate((-1.0, 1.0)):
        sword.visual(
            Cylinder(radius=0.008, length=0.008),
            origin=Origin(xyz=(guard_x, sy * 0.104, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=mats["brass_dark"], name=f"guard_tip_{i}",
        )
    sword.visual(
        Cylinder(radius=0.009, length=0.175),
        origin=Origin(xyz=(0.1125, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mats["amber"], name="grip_core",
    )
    sword.visual(
        Cylinder(radius=0.0148, length=0.029),
        origin=Origin(xyz=(0.0635, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mats["amber"], name="grip_lower",
    )
    _emit_grip_collar(sword, mats)
    sword.visual(
        Cylinder(radius=0.0148, length=0.034),
        origin=Origin(xyz=(0.151, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mats["amber"], name="grip_upper",
    )
    sword.visual(
        Cylinder(radius=0.011, length=0.012),
        origin=Origin(xyz=(0.205, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mats["brass_dark"], name="pommel_collar",
    )
    sword.visual(
        mesh_from_cadquery(_disc_pommel_solid(), "disc_pommel"),
        origin=Origin(xyz=(0.221, 0.0, 0.0)),
        material=mats["brass"], name="pommel",
    )


def _build_hilt_knuckle_scentstop(sword, mats: dict) -> None:
    cg_x, cg_len, cg_w, cg_h = 0.008, 0.016, 0.066, 0.018
    sword.visual(
        Box((cg_len, cg_w, cg_h)),
        origin=Origin(xyz=(cg_x, 0.0, 0.0)),
        material=mats["brass"], name="crossguard",
    )
    for i, zs in enumerate((1.0, -1.0)):
        sword.visual(
            Box((0.012, 0.042, 0.0014)),
            origin=Origin(xyz=(cg_x, 0.0, zs * (cg_h / 2.0 + 0.0002))),
            material=mats["brass_dark"], name=f"crossguard_relief_{i}",
        )
    knuckle_points = [
        (0.008, 0.0, 0.009),
        (0.045, 0.0, 0.038),
        (0.095, 0.0, 0.048),
        (0.145, 0.0, 0.038),
        (0.174, 0.0, 0.008),
    ]
    knuckle_mesh = tube_from_spline_points(
        knuckle_points, radius=0.0045, samples_per_segment=18, radial_segments=16, cap_ends=True
    )
    sword.visual(
        mesh_from_geometry(knuckle_mesh, "knuckle_guard"),
        material=mats["brass"], name="knuckle_guard",
    )
    sword.visual(
        Cylinder(radius=0.009, length=0.127),
        origin=Origin(xyz=(0.1085, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mats["amber"], name="grip_core",
    )
    sword.visual(
        Cylinder(radius=0.0148, length=0.029),
        origin=Origin(xyz=(0.0635, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mats["amber"], name="grip_lower",
    )
    _emit_grip_collar(sword, mats)
    sword.visual(
        Cylinder(radius=0.0148, length=0.038),
        origin=Origin(xyz=(0.155, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mats["amber"], name="grip_upper",
    )
    pommel_secs = [
        (-0.020, 0.007),
        (-0.008, 0.013),
        (0.004, 0.0145),
        (0.018, 0.011),
        (0.028, 0.005),
    ]
    pommel_x = 0.190
    sword.visual(
        mesh_from_cadquery(_scent_stopper_pommel(pommel_secs), "pommel"),
        origin=Origin(xyz=(pommel_x, 0.0, 0.0)),
        material=mats["brass"], name="pommel",
    )
    finial_base = pommel_x + pommel_secs[-1][0]
    sword.visual(
        Cylinder(radius=0.0055, length=0.021),
        origin=Origin(xyz=(finial_base + 0.0105, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mats["brass"], name="finial_stem",
    )
    sword.visual(
        Sphere(0.0078),
        origin=Origin(xyz=(finial_base + 0.021 + 0.0078, 0.0, 0.0)),
        material=mats["brass"], name="finial_ball",
    )


_HILT_BUILDERS = {
    "box_bead": _build_hilt_box_bead,
    "cruciform_disc": _build_hilt_cruciform_disc,
    "knuckle_scentstop": _build_hilt_knuckle_scentstop,
}


# --------------------------------------------------------------------------- #
# Scabbard chassis (straight or bowed) + N lug stations.
# --------------------------------------------------------------------------- #
def _ring_station_xs(n: int, spacing_scale: float) -> list[float]:
    """Even x-centres for the N ring stations within [RING_X_START, RING_X_END].

    Spacing scales about the window midpoint; clamped so stations stay inside
    the window (clear of chape ball + throat band) at every N and scale.
    """
    mid = 0.5 * (RING_X_START + RING_X_END)
    if n <= 1:
        return [mid]
    full_span = RING_X_END - RING_X_START
    span = min(full_span, full_span * spacing_scale)
    start = mid - span / 2.0
    step = span / (n - 1)
    return [start + i * step for i in range(n)]


def _build_scabbard(model: ArticulatedObject, r: ResolvedSwordConfig, mats: dict):
    scabbard = model.part("scabbard")
    body_secs, chape_secs, cav_secs, throat = _profile_sections(r.blade_profile)

    if r.is_curved:
        bow = _make_scabbard_bow(r.bow_max)
        cavity = _offset_loft(_curved_sections(_densify(cav_secs, 8), bow))
        body = _offset_loft(_curved_sections(_densify(body_secs, 16), bow)).cut(cavity)
        chape = _offset_loft(_curved_sections(chape_secs, bow)).cut(cavity)
    else:
        bow = lambda x: 0.0  # noqa: E731
        cavity = _loft(cav_secs)
        body = _loft(body_secs).cut(cavity)
        chape = _loft(chape_secs).cut(cavity)

    scabbard.visual(mesh_from_cadquery(body, "scabbard_body"), material=mats["burl"], name="body")
    scabbard.visual(mesh_from_cadquery(chape, "scabbard_chape"), material=mats["brass"], name="chape")
    scabbard.visual(
        Sphere(0.009), origin=Origin(xyz=(0.009, bow(0.009), ZC)),
        material=mats["brass"], name="chape_ball",
    )
    for i, bx in enumerate((0.030, 0.042)):
        w, t = _lerp_sections(chape_secs, bx)
        scabbard.visual(
            Box((0.004, w + 0.0028, t + 0.0028)),
            origin=Origin(xyz=(bx, bow(bx), ZC)),
            material=mats["brass_dark"], name=f"chape_ridge_{i}",
        )

    # Throat band (hollow brass collar at the mouth).
    throat_x0, throat_x1 = 0.43, 0.50
    if r.is_curved:
        n_throat = 6
        outer, inner = [], []
        for i in range(n_throat + 1):
            t = i / n_throat
            x = throat_x0 + t * (throat_x1 - throat_x0)
            y_off = bow(x)
            outer.append((x, y_off, throat["w"], throat["t"]))
            inner.append((x, y_off, throat["hole_w"], throat["hole_t"]))
        throat_solid = _offset_loft(outer).cut(_offset_loft(inner))
    else:
        throat_solid = (
            cq.Workplane("YZ", origin=((throat_x0 + throat_x1) / 2.0, 0.0, ZC))
            .rect(throat["w"], throat["t"])
            .rect(throat["hole_w"], throat["hole_t"])
            .extrude((throat_x1 - throat_x0) / 2.0, both=True)
        )
    scabbard.visual(
        mesh_from_cadquery(throat_solid, "scabbard_throat"), material=mats["brass"], name="throat"
    )
    for i, px in enumerate((0.448, 0.480)):
        scabbard.visual(
            Box((0.020, 0.044, 0.0016)),
            origin=Origin(xyz=(px, bow(px), ZC + throat["t"] / 2.0 + 0.0003)),
            material=mats["brass_dark"], name=f"throat_relief_{i}",
        )

    # Tan strap X-patterns (decorative, body broad faces).
    for xi, xc in enumerate((0.19, 0.33)):
        _, bt = _lerp_sections(body_secs, xc)
        y_off = bow(xc)
        for face, zs in (("top", 1.0), ("bottom", -1.0)):
            for di, yaw in enumerate((0.42, -0.42)):
                scabbard.visual(
                    Box((0.075, 0.012, 0.0024)),
                    origin=Origin(xyz=(xc, y_off, ZC + zs * (bt / 2.0 + 0.0002)), rpy=(0.0, 0.0, yaw)),
                    material=mats["tan"], name=f"strap_x{xi}_{face}_{di}",
                )

    # N banded suspension-ring lug stations.
    station_xs = _ring_station_xs(r.ring_count, r.ring_x_spacing_scale)
    joint_origins: list[tuple[float, float, float]] = []
    for i in range(r.ring_count):
        bx = station_xs[i]
        bw, bt = _lerp_sections(body_secs, bx)
        half_w = bw / 2.0
        pos = _lug_positions(half_w)
        y_bow = bow(bx)
        side = -1.0 if (r.alternate_sides and i % 2 == 1) else 1.0

        scabbard.visual(
            mesh_from_cadquery(_band_wrap(bx, bw, bt), f"band_{i}_wrap"),
            origin=Origin(xyz=(0.0, y_bow, 0.0)),
            material=mats["brass_dark"], name=f"band_{i}_wrap",
        )
        scabbard.visual(
            Cylinder(radius=_FLANGE_R, length=_FLANGE_LEN),
            origin=Origin(xyz=(bx, side * pos["flange_cy"] + y_bow, PIN_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=mats["brass_dark"], name=f"band_{i}_flange",
        )
        scabbard.visual(
            Cylinder(radius=PIN_R, length=_PIN_LEN),
            origin=Origin(xyz=(bx, side * pos["pin_cy"] + y_bow, PIN_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=mats["brass"], name=f"band_{i}_pin",
        )
        scabbard.visual(
            Cylinder(radius=_HEAD_R, length=_HEAD_LEN),
            origin=Origin(xyz=(bx, side * pos["head_cy"] + y_bow, PIN_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=mats["brass_dark"], name=f"band_{i}_head",
        )
        joint_origins.append((bx, side * pos["joint_cy"] + y_bow, PIN_Z))

    return scabbard, joint_origins


def _build_rings(model: ArticulatedObject, r: ResolvedSwordConfig, mats: dict,
                 scabbard, joint_origins) -> None:
    for i in range(r.ring_count):
        ring = model.part(f"band_{i}_ring")
        ring.visual(
            mesh_from_cadquery(_ring_solid(), f"suspension_ring_{i}"),
            origin=Origin(xyz=(0.0, 0.0, -RING_HANG)),
            material=mats["brass"], name="ring",
        )
        model.articulation(
            f"band_{i}_pivot",
            ArticulationType.REVOLUTE,
            parent=scabbard,
            child=ring,
            origin=Origin(xyz=joint_origins[i]),
            axis=SWING_AXIS_Y,
            motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=-RING_LIMIT, upper=RING_LIMIT),
        )


# --------------------------------------------------------------------------- #
# Materials.
# --------------------------------------------------------------------------- #
def _materials(model: ArticulatedObject, r: ResolvedSwordConfig) -> dict:
    out = {}
    for key, rgba in r.palette.items():
        out[key] = model.material(f"sword_{key}_{r.palette_style}", rgba=rgba)
    return out


# --------------------------------------------------------------------------- #
# Top-level builder.
# --------------------------------------------------------------------------- #
def build_sword(config: SwordConfig | None = None, *, assets: AssetContext | None = None) -> ArticulatedObject:
    config = config or SwordConfig()
    r = resolve_config(config)
    if assets is None:
        assets = AssetContext(Path(tempfile.mkdtemp(prefix="articraft-sword-")))
    model = ArticulatedObject(name=r.name, assets=assets)
    model.meta["slot_choices"] = [list(t) for t in slot_choices_for_config(r)]

    mats = _materials(model, r)

    # --- grounded scabbard chassis (+ N lug stations) ---------------------- #
    scabbard, joint_origins = _build_scabbard(model, r, mats)

    # --- sword (blade + hilt, single rigid part) --------------------------- #
    sword = model.part("sword")
    _build_blade(sword, r, mats)          # Slot A
    _HILT_BUILDERS[r.hilt](sword, mats)   # Slot B

    # The identity joint: blade draws prismatically out of the scabbard mouth.
    model.articulation(
        "sword_draw",
        ArticulationType.PRISMATIC,
        parent=scabbard,
        child=sword,
        origin=Origin(xyz=(MOUTH_X, 0.0, ZC)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.8, lower=0.0, upper=r.draw_upper),
    )

    # --- N swinging suspension rings --------------------------------------- #
    _build_rings(model, r, mats, scabbard, joint_origins)

    return model


def build_seeded_sword(seed: int, *, assets: AssetContext | None = None) -> ArticulatedObject:
    return build_sword(config_from_seed(seed), assets=assets)


# --------------------------------------------------------------------------- #
# Slot choices (consumed by module_topology_diversity).
# --------------------------------------------------------------------------- #
def slot_choices_for_config(r: ResolvedSwordConfig) -> tuple[tuple[str, str], ...]:
    return (
        ("blade_profile", r.blade_profile),
        ("hilt", r.hilt),
        ("rings", f"{r.ring_count}_ring_set"),
    )


def slot_choices_for_seed(seed: int) -> tuple[tuple[str, str], ...]:
    return slot_choices_for_config(resolve_config(config_from_seed(seed)))


# --------------------------------------------------------------------------- #
# Tests.
# --------------------------------------------------------------------------- #
def _declare_overlap_allowances(ctx: TestContext, model: ArticulatedObject, r: ResolvedSwordConfig) -> None:
    scabbard = model.get_part("scabbard")
    for i in range(r.ring_count):
        ring = model.get_part(f"band_{i}_ring")
        ctx.allow_overlap(
            ring, scabbard,
            elem_a="ring", elem_b=f"band_{i}_pin",
            reason="suspension ring is threaded (captured) on its lug pin",
        )


def run_sword_tests(object_model: ArticulatedObject, config: SwordConfig) -> TestReport:
    r = resolve_config(config)
    ctx = TestContext(object_model)
    _declare_overlap_allowances(ctx, object_model, r)

    scabbard = object_model.get_part("scabbard")
    sword = object_model.get_part("sword")
    draw = object_model.get_articulation("sword_draw")
    rings = [object_model.get_part(f"band_{i}_ring") for i in range(r.ring_count)]
    pivots = [object_model.get_articulation(f"band_{i}_pivot") for i in range(r.ring_count)]

    # --- identity joint: exactly one prismatic, +X draw ------------------- #
    prismatics = [
        a for a in object_model.articulations
        if a.articulation_type == ArticulationType.PRISMATIC
    ]
    ctx.check(
        "exactly one prismatic joint (the sword draw)",
        len(prismatics) == 1,
        details=f"n_prismatic={len(prismatics)}",
    )
    ctx.check(
        "sword draws on a prismatic joint along +X",
        draw.articulation_type == ArticulationType.PRISMATIC and draw.axis == (1.0, 0.0, 0.0),
        details=f"type={draw.articulation_type}, axis={draw.axis}",
    )
    ctx.check(
        "draw travel starts at 0 and is ~0.5 m",
        draw.motion_limits is not None
        and abs(draw.motion_limits.lower) < 1e-9
        and abs(draw.motion_limits.upper - r.draw_upper) < 0.02,
        details=f"limits=({draw.motion_limits.lower}, {draw.motion_limits.upper})",
    )
    for i, pivot in enumerate(pivots):
        ctx.check(
            f"band_{i} ring is REVOLUTE about +Y, ~±60 deg",
            pivot.articulation_type == ArticulationType.REVOLUTE
            and pivot.axis == (0.0, 1.0, 0.0)
            and pivot.motion_limits is not None
            and abs(pivot.motion_limits.lower + RING_LIMIT) < 0.05
            and abs(pivot.motion_limits.upper - RING_LIMIT) < 0.05,
            details=f"type={pivot.articulation_type}, axis={pivot.axis}",
        )
    ctx.check(
        f"ring count matches the sampled multiplicity ({r.ring_count})",
        len(rings) == r.ring_count and len(pivots) == r.ring_count,
        details=f"rings={len(rings)}, pivots={len(pivots)}",
    )

    # --- scale and grounding ---------------------------------------------- #
    scab_aabb = ctx.part_world_aabb(scabbard)
    sword_aabb = ctx.part_world_aabb(sword)
    ctx.check(
        "scabbard rests on the ground (brass fittings at z~0)",
        scab_aabb is not None and -0.0015 <= scab_aabb[0][2] <= 0.003,
        details=f"scabbard zmin={scab_aabb[0][2] if scab_aabb else None}",
    )
    ctx.check(
        "sword never dips below the ground plane",
        sword_aabb is not None and sword_aabb[0][2] >= -0.0015,
        details=f"sword zmin={sword_aabb[0][2] if sword_aabb else None}",
    )
    overall = (
        max(scab_aabb[1][0], sword_aabb[1][0]) - min(scab_aabb[0][0], sword_aabb[0][0])
        if scab_aabb and sword_aabb else 0.0
    )
    ctx.check(
        "sheathed assembly is ~0.7-0.83 m long overall",
        0.68 <= overall <= 0.85,
        details=f"x span={overall}",
    )

    # --- blade dimensions + sheathed nesting ------------------------------ #
    blade_aabb = ctx.part_element_world_aabb(sword, elem="blade")
    ctx.check(
        "blade is ~0.45 m long",
        blade_aabb is not None and 0.42 <= blade_aabb[1][0] - blade_aabb[0][0] <= 0.48,
        details=f"blade aabb={blade_aabb}",
    )
    ctx.check(
        "sheathed blade is hidden inside the scabbard (tip near the chape)",
        blade_aabb is not None and blade_aabb[1][0] <= 0.515 and blade_aabb[0][0] >= 0.025,
        details=f"blade aabb={blade_aabb}",
    )
    ctx.expect_within(
        sword, scabbard, axes="yz", inner_elem="blade", outer_elem="body", margin=0.0005,
        name="sheathed blade nests inside the hollow body cross-section",
    )
    ctx.expect_overlap(
        sword, scabbard, axes="x", elem_a="blade", elem_b="body", min_overlap=0.35,
        name="sheathed blade is inserted along nearly the full scabbard length",
    )

    if r.is_curved:
        body_aabb = ctx.part_element_world_aabb(scabbard, elem="body")
        body_y_center = (body_aabb[0][1] + body_aabb[1][1]) / 2.0 if body_aabb else 0.0
        ctx.check(
            "scabbard body bows laterally to match the saber blade",
            body_y_center > 0.001,
            details=f"body Y centre={body_y_center}",
        )

    # --- drawing the sword ------------------------------------------------- #
    with ctx.pose({draw: 0.25 * r.length_scale}):
        ctx.expect_within(
            sword, scabbard, axes="yz", inner_elem="blade", outer_elem="body", margin=0.0005,
            name="half-drawn blade stays centred in the cavity",
        )
        ctx.expect_overlap(
            sword, scabbard, axes="x", elem_a="blade", elem_b="body", min_overlap=0.12,
            name="half-drawn blade retains insertion in the scabbard",
        )
    with ctx.pose({draw: r.draw_upper}):
        drawn_blade = ctx.part_element_world_aabb(sword, elem="blade")
        ctx.check(
            "fully drawn blade clears the scabbard mouth completely",
            drawn_blade is not None and drawn_blade[0][0] >= MOUTH_X + 0.005,
            details=f"drawn blade aabb={drawn_blade}",
        )

    # --- hilt composition (pommel sits at the +X hilt end) ----------------- #
    pommel_aabb = ctx.part_element_world_aabb(sword, elem="pommel")
    ctx.check(
        "pommel sits at the hilt end (high +X)",
        pommel_aabb is not None and 0.66 <= 0.5 * (pommel_aabb[0][0] + pommel_aabb[1][0]) <= 0.75,
        details=f"pommel aabb={pommel_aabb}",
    )
    bead_aabb = ctx.part_element_world_aabb(sword, elem="grip_collar_bead_1")
    ctx.check(
        "gold collar occupies the middle of the grip",
        bead_aabb is not None and 0.55 <= 0.5 * (bead_aabb[0][0] + bead_aabb[1][0]) <= 0.65,
        details=f"bead aabb={bead_aabb}",
    )

    # --- suspension rings: captured on pins + swinging --------------------- #
    for i, (ring, pivot) in enumerate(zip(rings, pivots)):
        ctx.expect_contact(
            ring, scabbard, elem_a="ring", elem_b=f"band_{i}_pin", contact_tol=0.0015,
            name=f"band_{i} ring is threaded onto its mounting pin",
        )
        closed = ctx.part_world_aabb(ring)
        with ctx.pose({pivot: 1.0}):
            swung = ctx.part_world_aabb(ring)
        ctx.check(
            f"band_{i} ring swings about its pin (off-axis hang proves rotation)",
            closed is not None and swung is not None
            and swung[0][0] < closed[0][0] - 0.003
            and swung[1][2] > closed[1][2] + 0.003,
            details=f"closed={closed}, swung={swung}",
        )

    return ctx.report()


__all__ = [
    "SwordConfig",
    "ResolvedSwordConfig",
    "config_from_seed",
    "resolve_config",
    "build_sword",
    "build_seeded_sword",
    "slot_choices_for_seed",
    "slot_choices_for_config",
    "run_sword_tests",
    "__modular__",
]
