"""bucket2 — Wooden staved bucket / pail (open hollow container, modular template).

Category identity: a HOLLOW coopered **wooden bucket / keg** WITH a visible LID.
The body is a surface of revolution (lathed staved shell) chosen by
``body_profile`` (bulged barrel / straight keg / tapered pail), hooped by N
dark-metal bands (the multiplicity axis), finished at the mouth by a named
``closure``:

  * ``hinged_lid`` (PRIMARY) — a rear-rim flip lid that RESTS closed on the
    mouth at q=0 and then swings up & back ~105 deg as the REVOLUTE joint
    opens.
  * ``slide_off_lid`` (PRIMARY) — a removable lift-off lid that RESTS closed on
    the mouth at q=0 and then lifts straight upward along PRISMATIC +Z.
  * ``open_top`` (MINORITY) — an OPEN mouth with NO lid at all; a finished
    rim/lip ledge rings the wide mouth. Its required non-FIXED joint comes from
    the carry HANDLE (swing bail / side ear rings, both REVOLUTE).
  * ``bunghole_plug`` (MINORITY) — an OPEN-mouthed keg with a radial side bung.
    The mouth stays open/hollow; the tapered bung is retained only as the side
    plug passing through a true shell-wall bunghole.

Modular templates render at the joint rest pose (q=0). ``hinged_lid`` and
``slide_off_lid`` both start closed and open under positive actuation. The
open_top no-lid pails and the side-bung kegs remain deliberate minorities. The
non-FIXED joint (closure or handle) carries the bucket/keg action.

Structure (pattern = ``mixed``): a single root ``barrel_body`` part (lathe mesh
chosen by ``body_profile``, with N_STAVES cosmetic stave grooves as Rule-1
parent visuals + rim_ledge or rim_lip), with three named module axes attaching
as parallel children of the body:

  * ``body_profile`` (3) — the ROOT ``barrel_body`` lathe mesh. All downstream
    rim/lid/hoop/bung/ear radii derive from ``_outer_radius(z)`` so swapping the
    profile never breaks the interface.
  * ``hoop_count`` (N in [2,4]) — N metal hoop bands, loop-emitted FIXED
    children wrapping the body at even z-pitch (the multiplicity axis). Encoded
    in slot_choices as ``("hoop_count", f"n{N}")``.
  * ``closure`` (4) — the mouth finish / mechanism:
      - hinged_lid    : REVOLUTE -Y rear-rim flip lid; RESTS closed, opens ~105 deg.
      - slide_off_lid : PRISMATIC +Z lift-off wooden lid; RESTS closed, lifts upward.
      - open_top      : OPEN mouth, NO lid; finished rim/lip ledge + visible deep
                        hollow cavity. Emits NO joint (handle carries it).
      - bunghole_plug : open mouth + radial PRISMATIC tapered side bung through a shell hole.
  * ``handle`` (4) — the grip/carry feature (gated by closure):
      - lid_arched_grip : arched grip FIXED on the lid (lid closures only).
      - swing_bail      : 2 FIXED pivot ears on the body + bail REVOLUTE +X.
      - side_ear_rings  : 2 FIXED fork ears + 2 fold-out rings each REVOLUTE +X.
      - no_handle       : side-bung keg with no carry handle.

Compatibility gating (resolve_config, spec §9):
  * open_top has NO lid and NO joint of its own -> its required >=1 non-FIXED
    joint MUST come from the handle; gated to {swing_bail, side_ear_rings}
    (both REVOLUTE). It is NOT paired with lid_arched_grip (needs a lid) or
    no_handle (no joint).
  * lid_arched_grip is a FIXED child of the lid -> only valid with a lid
    closure (slide_off_lid / hinged_lid).
  * bunghole_plug has no lid -> a lid grip would be a phantom anchor; handle is
    gated to {swing_bail, side_ear_rings, no_handle}.
  * no_handle is only a fallback when closure == bunghole_plug.

All hoop / ear / lid-plug / handle overlaps are coopered-fit / captured-pin
geometry, declared element-scoped in run_bucket2_tests.

Canonical spec: ``articraft_template_authoring/specs_modular_v1/Urban_Environment_bucket2.md``
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, replace
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
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

__modular__ = True

# ---------------------------------------------------------------------------
# Slot domains
# ---------------------------------------------------------------------------
BodyProfile = Literal["bulged_barrel", "straight_keg", "tapered_pail"]
Closure = Literal["open_top", "slide_off_lid", "hinged_lid", "bunghole_plug"]
Handle = Literal["lid_arched_grip", "swing_bail", "side_ear_rings", "no_handle"]
PaletteStyle = Literal[
    "oak_iron",
    "dark_stain_iron",
    "weathered_iron",
    "oak_brass",
    "light_ash_iron",
    "dark_stain_brass",
]

BODY_PROFILES: tuple[BodyProfile, ...] = ("bulged_barrel", "straight_keg", "tapered_pail")
CLOSURES: tuple[Closure, ...] = ("open_top", "slide_off_lid", "hinged_lid", "bunghole_plug")
# PRIMARY identity = a hollow wooden bucket/keg WITH a visible lid. The lid
# closures remain the MAJORITY, but they now rest closed at q=0 and reveal the
# cavity through actuation instead of a baked-open pose. A minority stays for
# diversity: a few side-bung kegs and a few open_top no-lid pails. Keep bunghole as a true
# minority so default seeded runs more reliably read as open hollow buckets/kegs.
# order: (open_top, slide_off_lid, hinged_lid, bunghole_plug)
CLOSURE_WEIGHTS: tuple[float, ...] = (0.20, 0.33, 0.42, 0.05)
HANDLES: tuple[Handle, ...] = ("lid_arched_grip", "swing_bail", "side_ear_rings", "no_handle")
PALETTE_STYLES: tuple[PaletteStyle, ...] = (
    "oak_iron",
    "dark_stain_iron",
    "weathered_iron",
    "oak_brass",
    "light_ash_iron",
    "dark_stain_brass",
)

LID_CLOSURES: tuple[Closure, ...] = ("slide_off_lid", "hinged_lid")

# Handles valid on a lid closure (lid grip OK; bail/rings always OK).
LID_HANDLES: tuple[Handle, ...] = ("lid_arched_grip", "swing_bail", "side_ear_rings")
# Handles valid on a side-bung open-mouth closure (no lid grip; no_handle OK).
BUNG_HANDLES: tuple[Handle, ...] = ("swing_bail", "side_ear_rings", "no_handle")
# open_top has NO lid and emits NO joint -> its required non-FIXED joint MUST
# come from the handle, so only the joint-bearing carry handles are valid here
# (NO lid_arched_grip = needs a lid, NO no_handle = no joint).
OPEN_HANDLES: tuple[Handle, ...] = ("swing_bail", "side_ear_rings")
OPEN_HANDLE_WEIGHTS: tuple[float, ...] = (0.55, 0.45)
LID_HANDLE_WEIGHTS: tuple[float, ...] = (0.42, 0.30, 0.28)
BUNG_HANDLE_WEIGHTS: tuple[float, ...] = (0.48, 0.40, 0.12)
BUNG_SEEDED_BODY_PROFILES: tuple[BodyProfile, ...] = ("bulged_barrel", "straight_keg")

# hoop_count multiplicity domain (distinct N in {2,3,4}; small-N weighted).
HOOP_COUNTS: tuple[int, ...] = (2, 3, 4)
HOOP_COUNT_WEIGHTS: tuple[float, ...] = (0.45, 0.40, 0.15)

N_STAVES = 16

# ---------------------------------------------------------------------------
# Palettes. Each colorway drives every .visual(material=...): wood body, lid /
# head wood, metal hoops, and handle/bail accent.
# ---------------------------------------------------------------------------
PALETTES: dict[PaletteStyle, dict[str, tuple[float, float, float, float]]] = {
    "oak_iron": {
        "wood": (0.74, 0.52, 0.32, 1.0),
        "lid_wood": (0.80, 0.60, 0.39, 1.0),
        "inner": (0.30, 0.20, 0.12, 1.0),
        "metal": (0.16, 0.17, 0.19, 1.0),
        "accent": (0.20, 0.30, 0.45, 1.0),
    },
    "dark_stain_iron": {
        "wood": (0.40, 0.26, 0.16, 1.0),
        "lid_wood": (0.46, 0.30, 0.19, 1.0),
        "inner": (0.16, 0.10, 0.06, 1.0),
        "metal": (0.16, 0.17, 0.19, 1.0),
        "accent": (0.22, 0.22, 0.24, 1.0),
    },
    "weathered_iron": {
        "wood": (0.62, 0.50, 0.38, 1.0),
        "lid_wood": (0.66, 0.54, 0.42, 1.0),
        "inner": (0.24, 0.19, 0.14, 1.0),
        "metal": (0.34, 0.22, 0.16, 1.0),
        "accent": (0.28, 0.28, 0.30, 1.0),
    },
    "oak_brass": {
        "wood": (0.74, 0.52, 0.32, 1.0),
        "lid_wood": (0.80, 0.60, 0.39, 1.0),
        "inner": (0.30, 0.20, 0.12, 1.0),
        "metal": (0.72, 0.58, 0.24, 1.0),
        "accent": (0.72, 0.58, 0.24, 1.0),
    },
    "light_ash_iron": {
        "wood": (0.82, 0.70, 0.52, 1.0),
        "lid_wood": (0.86, 0.74, 0.56, 1.0),
        "inner": (0.34, 0.28, 0.20, 1.0),
        "metal": (0.16, 0.17, 0.19, 1.0),
        "accent": (0.20, 0.30, 0.45, 1.0),
    },
    "dark_stain_brass": {
        "wood": (0.40, 0.26, 0.16, 1.0),
        "lid_wood": (0.46, 0.30, 0.19, 1.0),
        "inner": (0.16, 0.10, 0.06, 1.0),
        "metal": (0.72, 0.58, 0.24, 1.0),
        "accent": (0.72, 0.58, 0.24, 1.0),
    },
}

# ---------------------------------------------------------------------------
# Base real-world dimensions (meters). Body base sits at z=0, mouth at z=BODY_H.
# ---------------------------------------------------------------------------
_BODY_H = 0.360
# Deep cavity for ALL closures: a THIN inner floor sunk low in the shell so the
# hollow staved interior reads as a real bucket cavity through the open mouth in
# EVERY config (hinged_lid / slide_off_lid / open_top alike), not just open_top.
# A thicker floor used to make the lidded kegs read flat-bottomed / capped.
_FLOOR_T = 0.014  # thin sunk wood floor (deep cavity) — used by ALL closures
_WALL = 0.016  # shell wall thickness (scaled by radius)

# bulged barrel radii (end / mid)
_BULGE_R_END = 0.150
_BULGE_RATIO = 1.24
# straight keg radius
_STRAIGHT_R = 0.150
# tapered pail radii (base / mouth)
_TAPER_R_MOUTH = 0.180
_TAPER_RATIO = 0.67

_HOOP_MARGIN = 0.050  # first/last hoop distance from each end
_HOOP_HALF_H = 0.013  # hoop band half-height
_HOOP_BAND_T = 0.008  # hoop radial thickness (sits proud of the wall)

_LID_FLANGE_T = 0.020
_LID_PLUG_DROP = 0.026
_LID_TRAVEL = 0.120
_RIM_LEDGE_T = 0.010

# --- Lid rest-pose geometry (modular templates render at joint q=0). ---
# Hinged lids now REST closed at q=0 and swing open under positive actuation.
# Slide-off lids REST seated/level at q=0 and lift straight upward under positive
# actuation. See _emit_hinged_lid / _emit_slide_off_lid.
# Hinged lid: open swing angle (radians) about the rear-rim hinge (~105 deg),
# like an open chest/keg lid standing back off the mouth.
_HINGE_REST_OPEN = math.radians(105.0)
# Slide-off lid: lift travel for the removable cap.
# open_top mouth lip: a finished rolled rim ring sitting proud of the mouth so
# the open mouth reads as a deliberate bucket lip (not a raw cut edge).
_OPEN_RIM_T = 0.014  # rim band height (z)
_OPEN_RIM_PROUD = 0.010  # rim band radial overhang outside the outer wall
# All closures now share the same deep (thin) floor (see _FLOOR_T) so the hollow
# reads through the open mouth for lidded kegs too; kept as an alias for clarity.
_OPEN_FLOOR_T = _FLOOR_T

_HINGE_OPEN = 1.50

_BUNG_TRAVEL = 0.065
_BUNG_R = 0.022
_BUNG_LEN = 0.040

_RING_OPEN = 1.80

# --- Swing bail (FIX 2): a TALL round carry bail spanning the mouth. ---
# Pivot ears sit just below the mouth rim (so the bail spans the full mouth
# diameter) and the bail arcs WELL ABOVE the rim. Targets: apex above the rim by
# ~0.27 m, bail width ~ the mouth diameter (pivots near ±mouth_r). These used to
# be a short ~0.16-tall arc pivoting mid-body that barely reached the rim.
_BAIL_EAR_BELOW_RIM = 0.045  # ear pivot drop below the mouth rim
_BAIL_APEX_ABOVE_RIM = 0.27  # bail apex height above the rim (in 0.22..0.32)
_BAIL_WIRE_R = 0.007  # bail wire radius (a touch thicker so it reads)


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _pick(value, choices):
    return value if value in choices else choices[0]


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Bucket2Config:
    body_profile: BodyProfile | None = None
    closure: Closure | None = None
    handle: Handle | None = None
    hoop_count: int | None = None
    palette_style: PaletteStyle = "oak_iron"
    body_height_scale: float = 1.0
    body_radius_scale: float = 1.0
    bulge_ratio: float = _BULGE_RATIO
    taper_ratio: float = _TAPER_RATIO
    lid_travel_scale: float = 1.0
    hinge_open_angle: float = _HINGE_OPEN
    bung_travel_scale: float = 1.0
    ring_open_angle: float = _RING_OPEN
    name: str = "bucket2"


@dataclass(frozen=True)
class ResolvedBucket2Config:
    body_profile: BodyProfile
    closure: Closure
    handle: Handle
    hoop_count: int
    palette_style: PaletteStyle
    body_h: float
    wall: float
    floor_t: float
    # profile radii (already radius-scaled)
    r_end: float  # bulged end radius
    r_mid: float  # bulged mid radius
    r_straight: float
    r_base: float  # tapered base
    r_mouth: float  # tapered mouth
    lid_travel: float
    hinge_open: float
    bung_travel: float
    bung_z: float
    ring_open: float
    name: str


def config_from_seed(seed: int) -> Bucket2Config:
    rng = random.Random(seed)
    body_profile = rng.choice(BODY_PROFILES)
    # The dominant seeded identity should stay "open hollow bucket/keg":
    # open-mouthed/lidded closures are the strong majority, while bunghole is a
    # small side-bung minority for diversity.
    closure = rng.choices(CLOSURES, weights=CLOSURE_WEIGHTS, k=1)[0]
    if closure == "bunghole_plug" and body_profile not in BUNG_SEEDED_BODY_PROFILES:
        # Seeded bungholes stay on keg-like shells; a sealed tapered pail more
        # easily reads as a solid plug-topped stump than as a coopered vessel.
        body_profile = rng.choice(BUNG_SEEDED_BODY_PROFILES)
    # Handle pool depends on closure (compatibility gate); sample within pool.
    if closure == "open_top":
        # open mouth has no lid joint -> handle MUST provide the non-FIXED joint.
        handle = rng.choices(OPEN_HANDLES, weights=OPEN_HANDLE_WEIGHTS, k=1)[0]
    elif closure in LID_CLOSURES:
        handle = rng.choices(LID_HANDLES, weights=LID_HANDLE_WEIGHTS, k=1)[0]
    else:
        # Keep no_handle available but uncommon so sealed kegs still usually read
        # as bucket/keg variants rather than plain capped barrels.
        handle = rng.choices(BUNG_HANDLES, weights=BUNG_HANDLE_WEIGHTS, k=1)[0]
    return Bucket2Config(
        body_profile=body_profile,
        closure=closure,
        handle=handle,
        hoop_count=rng.choices(HOOP_COUNTS, weights=HOOP_COUNT_WEIGHTS, k=1)[0],
        palette_style=rng.choice(PALETTE_STYLES),
        body_height_scale=round(rng.uniform(0.85, 1.20), 4),
        body_radius_scale=round(rng.uniform(0.85, 1.20), 4),
        bulge_ratio=round(rng.uniform(1.05, 1.30), 4),
        taper_ratio=round(rng.uniform(0.55, 0.85), 4),
        lid_travel_scale=round(rng.uniform(0.8, 1.3), 4),
        hinge_open_angle=round(rng.uniform(1.20, 1.55), 4),
        bung_travel_scale=round(rng.uniform(0.8, 1.4), 4),
        ring_open_angle=round(rng.uniform(1.40, 1.85), 4),
        name=f"seeded_bucket2_{seed}",
    )


def resolve_config(config: Bucket2Config | None = None) -> ResolvedBucket2Config:
    cfg = config or Bucket2Config()
    palette_style = _pick(cfg.palette_style, PALETTE_STYLES)
    body_profile = _pick(cfg.body_profile, BODY_PROFILES)
    closure = _pick(cfg.closure, CLOSURES)
    handle = _pick(cfg.handle, HANDLES)

    hoop_count = int(cfg.hoop_count) if cfg.hoop_count is not None else 2
    hoop_count = int(_clamp(hoop_count, 2, 4))

    # --- Compatibility gating (spec §9). ---
    if closure == "open_top":
        # No lid + no closure joint -> handle MUST carry a real non-FIXED joint.
        if handle not in OPEN_HANDLES:
            handle = "swing_bail"
    elif closure in LID_CLOSURES:
        if handle not in LID_HANDLES:
            handle = "swing_bail"
    else:  # bunghole_plug: no lid -> no lid grip
        if handle not in BUNG_HANDLES:
            handle = "swing_bail"

    # --- Continuous params (clamped). ---
    hs = _clamp(cfg.body_height_scale, 0.85, 1.20)
    rs = _clamp(cfg.body_radius_scale, 0.85, 1.20)
    bulge_ratio = _clamp(cfg.bulge_ratio, 1.05, 1.30)
    taper_ratio = _clamp(cfg.taper_ratio, 0.55, 0.85)

    body_h = _BODY_H * hs
    wall = _WALL * rs
    # EVERY closure now drops the inner floor low (thin floor) so the hollow
    # staved cavity reads deep — a tall visible inner wall through the open mouth
    # for hinged_lid / slide_off_lid / open_top / bunghole alike.
    floor_t = _FLOOR_T

    r_end = _BULGE_R_END * rs
    r_mid = r_end * bulge_ratio
    r_straight = _STRAIGHT_R * rs
    r_mouth = _TAPER_R_MOUTH * rs
    r_base = r_mouth * taper_ratio

    # --- wall / inner_r inequality: wall must stay a thin shell. ---
    mouth_r = _mouth_radius_for(body_profile, r_end, r_straight, r_mouth)
    # inner_r > 0.05 (small keg); wall < 0.5*mouth_r.
    max_wall = min(0.5 * mouth_r - 0.001, mouth_r - 0.045)
    wall = _clamp(wall, 0.008, max(0.009, max_wall))

    lid_travel = _clamp(_LID_TRAVEL * _clamp(cfg.lid_travel_scale, 0.8, 1.3), 0.080, 0.180)
    hinge_open = _clamp(cfg.hinge_open_angle, 1.20, 1.55)
    bung_travel = _clamp(_BUNG_TRAVEL * _clamp(cfg.bung_travel_scale, 0.8, 1.4), 0.045, 0.100)
    ring_open = _clamp(cfg.ring_open_angle, 1.40, 1.85)

    # --- bunghole z: place in a hoop-free vertical window on the shell. ---
    z_positions = _hoop_z_positions(hoop_count, body_h)
    bung_z = _resolve_bung_z(body_h=body_h, floor_t=floor_t, hoop_z_positions=z_positions)

    return ResolvedBucket2Config(
        body_profile=body_profile,
        closure=closure,
        handle=handle,
        hoop_count=hoop_count,
        palette_style=palette_style,
        body_h=body_h,
        wall=wall,
        floor_t=floor_t,
        r_end=r_end,
        r_mid=r_mid,
        r_straight=r_straight,
        r_base=r_base,
        r_mouth=r_mouth,
        lid_travel=lid_travel,
        hinge_open=hinge_open,
        bung_travel=bung_travel,
        bung_z=bung_z,
        ring_open=ring_open,
        name=cfg.name or "bucket2",
    )


def _mouth_radius_for(profile: str, r_end: float, r_straight: float, r_mouth: float) -> float:
    if profile == "straight_keg":
        return r_straight
    if profile == "tapered_pail":
        return r_mouth
    return r_end  # bulged: mouth radius = end radius


def with_overrides(config: Bucket2Config, **kwargs: object) -> Bucket2Config:
    return replace(config, **kwargs)


# ---------------------------------------------------------------------------
# Profile radius function — the surface of revolution interface.
# ---------------------------------------------------------------------------
def _outer_radius(r: ResolvedBucket2Config, z: float) -> float:
    """Outer body radius at height z (0..body_h). Drives all downstream radii."""
    h = r.body_h
    z = _clamp(z, 0.0, h)
    if r.body_profile == "straight_keg":
        return r.r_straight
    if r.body_profile == "tapered_pail":
        t = z / h
        return r.r_base + t * (r.r_mouth - r.r_base)
    # bulged barrel: symmetric parabola, end radius at z=0/h, mid at z=h/2.
    t = z / h
    return r.r_end + (r.r_mid - r.r_end) * (1.0 - (2.0 * t - 1.0) ** 2)


def _mouth_radius(r: ResolvedBucket2Config) -> float:
    return _outer_radius(r, r.body_h)


def _hoop_z_positions(n: int, body_h: float) -> list[float]:
    margin = _HOOP_MARGIN
    if n >= 4:
        margin = 0.040  # tighten margins so 4 hoops keep pitch >= 2*half_h
    if n == 1:
        return [body_h * 0.5]
    span = body_h - 2.0 * margin
    return [margin + span * i / (n - 1) for i in range(n)]


def _resolve_bung_z(*, body_h: float, floor_t: float, hoop_z_positions: list[float]) -> float:
    """Choose a hoop-clear bunghole height with a stable, readable shell placement."""
    low = floor_t + 0.04
    high = body_h - 0.04
    target = body_h * 0.5
    clear_z = _HOOP_HALF_H + 0.5 * _BUNG_R + 0.006

    windows: list[tuple[float, float]] = []
    cursor = low
    for hz in sorted(hoop_z_positions):
        window_hi = min(high, hz - clear_z)
        if window_hi >= cursor:
            windows.append((cursor, window_hi))
        cursor = max(cursor, hz + clear_z)
    if cursor <= high:
        windows.append((cursor, high))

    if windows:
        def _window_rank(window: tuple[float, float]) -> tuple[float, float]:
            lo, hi = window
            chosen = _clamp(target, lo, hi)
            # Prefer the window whose clamped target stays closest to mid-body;
            # ties break toward larger windows for more geometric margin.
            return (abs(chosen - target), -(hi - lo))

        best_lo, best_hi = min(windows, key=_window_rank)
        return _clamp(target, best_lo, best_hi)

    # Fallback: if the hoop layout leaves no full safe window, pick the point in
    # range that maximizes distance from the nearest hoop centreline.
    candidates = [low, target, high]
    boundaries = [low, *sorted(hoop_z_positions), high]
    candidates.extend((a + b) * 0.5 for a, b in zip(boundaries, boundaries[1:]))
    return max(
        (_clamp(c, low, high) for c in candidates),
        key=lambda z: min(abs(z - hz) for hz in hoop_z_positions) if hoop_z_positions else high - low,
    )


# ---------------------------------------------------------------------------
# Body lathe mesh (Slot A). All three profiles share one revolve/floor builder.
# ---------------------------------------------------------------------------
def _body_profile_pts(r: ResolvedBucket2Config, *, outer: bool) -> list[tuple[float, float]]:
    """(radius, z) control points along the wall, bottom->top."""
    h = r.body_h
    steps = 9
    pts: list[tuple[float, float]] = []
    for i in range(steps + 1):
        z = h * i / steps
        rad = _outer_radius(r, z)
        if not outer:
            rad = max(rad - r.wall, 0.02)
        pts.append((rad, z))
    return pts


def _body_solid(r: ResolvedBucket2Config) -> cq.Workplane:
    """Hollow staved shell: a single closed (radius, z) profile revolved 360°.

    Profile path (CCW in the XZ half-plane): axis@0 -> out along the floor ->
    up the OUTER wall -> across the rim top -> down the INNER wall -> across the
    floor TOP -> back to the axis. Points above the floor only on the inner run.
    """
    outer_pts = _body_profile_pts(r, outer=True)
    inner_pts = _body_profile_pts(r, outer=True)  # reuse for inner subtract
    h = r.body_h
    floor_t = r.floor_t

    pts: list[tuple[float, float]] = []
    pts.append((0.0, 0.0))
    pts.append((outer_pts[0][0], 0.0))
    for rad, z in outer_pts[1:]:
        pts.append((rad, z))
    # rim top: across from outer mouth radius to inner mouth radius
    mouth_r = _mouth_radius(r)
    inner_mouth_r = max(mouth_r - r.wall, 0.03)
    pts.append((inner_mouth_r, h))
    # inner wall downward to the floor top
    for rad, z in reversed(inner_pts):
        if z <= floor_t + 1e-4:
            break
        inner_rad = max(rad - r.wall, 0.02)
        pts.append((inner_rad, z))
    # close across the floor top back to the axis
    pts.append((max(_outer_radius(r, floor_t) - r.wall, 0.02), floor_t))
    pts.append((0.0, floor_t))

    # Drop consecutive duplicate / near-duplicate points (degenerate edges).
    clean: list[tuple[float, float]] = []
    for p in pts:
        if not clean or (abs(p[0] - clean[-1][0]) > 1e-5 or abs(p[1] - clean[-1][1]) > 1e-5):
            clean.append(p)

    wp = cq.Workplane("XZ").moveTo(clean[0][0], clean[0][1])
    for rad, z in clean[1:]:
        wp = wp.lineTo(rad, z)
    return wp.close().revolve(360.0, (0, -1, 0), (0, 1, 0))


def _stave_groove_solid(r: ResolvedBucket2Config) -> cq.Workplane | None:
    """Cosmetic vertical stave grooves cut into the outer shell (Rule 1 visual).

    Returns a union of thin vertical slots at the widest radius; cut from the
    body so the grooves read as coopered staves. Kept low-poly (16 grooves).
    """
    h = r.body_h
    groove_w = 0.004
    grooves = None
    # cut at the widest point so all profiles get visible vertical seams
    for i in range(N_STAVES):
        a = 2.0 * math.pi * i / N_STAVES
        slot = (
            cq.Workplane("XY")
            .box(groove_w, groove_w, h * 0.92)
            .translate((0.0, 0.0, h * 0.5))
        )
        # place at the mid-height radius, just at the surface
        rad = _outer_radius(r, h * 0.5)
        slot = slot.translate((rad * math.cos(a), rad * math.sin(a), 0.0))
        grooves = slot if grooves is None else grooves.union(slot)
    return grooves


def _build_body(model: ArticulatedObject, r: ResolvedBucket2Config, mats) -> object:
    body = model.part("barrel_body")
    solid = _body_solid(r)
    if r.closure == "bunghole_plug":
        try:
            solid = solid.cut(_bunghole_bore_cut(r))
        except Exception:
            pass
    grooves = _stave_groove_solid(r)
    if grooves is not None:
        try:
            solid = solid.cut(grooves)
        except Exception:
            pass
    body.visual(
        mesh_from_cadquery(solid, "staved_shell"),
        material=mats["wood"],
        name="staved_shell",
    )
    mouth_r = _mouth_radius(r)
    body.inertial = Inertial.from_geometry(
        Cylinder(radius=mouth_r, length=r.body_h),
        mass=2.5,
        origin=Origin(xyz=(0.0, 0.0, r.body_h / 2.0)),
    )
    # Closure-dependent top visual:
    #   open_top / bunghole_plug -> finished rolled rim LIP ring (open mouth)
    #   lid closures             -> thin ANNULAR rim ledge ring the lid seats on
    if r.closure in ("open_top", "bunghole_plug"):
        body.visual(
            _open_rim_mesh(r, mouth_r),
            material=mats["lid_wood"],
            name="rim_lip",
        )
    else:
        # FIX 1: the lid-seat rim ledge is an ANNULAR ring (open bore), NOT a
        # solid disc — a solid disc filling the mouth used to visually CAP the
        # cavity so the lidded kegs read flat/non-hollow even with the lid open.
        # The ring's bore stays the inner cavity radius, so when the lid rests
        # OPEN the deep hollow staved interior is plainly visible through the
        # mouth. The lid still seats on the ring's inner shoulder.
        body.visual(
            _rim_ledge_mesh(r, mouth_r),
            material=mats["lid_wood"],
            name="rim_ledge",
        )
    # FIX 1: a DARK recessed inner cavity (floor disc + inner-wall liner), fused
    # on the body as Rule-1 visuals, for every config whose mouth is open at rest.
    # The dark shade makes the recessed hollow read unmistakably as a deep empty
    # interior (looking in you see a dark inner wall dropping to a dark floor),
    # so the vessels no longer look flat/solid.
    _build_inner_cavity(body, r, mats)
    return body


def _build_inner_cavity(body, r: ResolvedBucket2Config, mats) -> None:
    """Dark recessed inner liner so the deep hollow reads in EVERY open config.

    Adds two fused body visuals: a dark floor disc at the cavity bottom and a
    thin dark inner-wall liner hugging the inner shell from the floor up to just
    below the rim. Both sit strictly inside the staved shell wall (radius =
    inner cavity radius), so no new part-vs-part overlap is introduced."""
    mouth_r = _mouth_radius(r)
    floor_top = r.floor_t
    # inner cavity radius near the mouth (slightly inside the wall to avoid
    # z-fighting with the staved shell's own inner surface).
    inner_r = max(mouth_r - r.wall - 0.003, 0.030)
    # dark floor disc sitting on top of the structural wood floor.
    body.visual(
        Cylinder(radius=inner_r, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, floor_top + 0.004)),
        material=mats["inner"],
        name="cavity_floor",
    )
    # dark inner-wall liner: a thin tube from just above the floor up to just
    # below the rim, giving the tall dark inner wall you see down the mouth.
    liner_bot = floor_top + 0.006
    liner_top = r.body_h - 0.010
    liner_h = max(liner_top - liner_bot, 0.05)
    base_r = max(_outer_radius(r, 0.5 * (liner_bot + liner_top)) - r.wall - 0.002, 0.028)
    liner = (
        cq.Workplane("XY")
        .workplane(offset=liner_bot)
        .circle(base_r)
        .circle(max(base_r - 0.006, 0.022))
        .extrude(liner_h)
    )
    body.visual(
        mesh_from_cadquery(liner, "cavity_liner"),
        material=mats["inner"],
        name="cavity_liner",
    )


def _open_rim_mesh(r: ResolvedBucket2Config, mouth_r: float) -> object:
    """Finished rolled mouth lip for an OPEN-top bucket: an annular ring (lathe)
    straddling the mouth rim. It sits proud of the outer wall and its bore is the
    inner cavity radius, so the deep hollow staved interior stays fully visible
    through the open mouth (no covering disc)."""
    inner_r = max(mouth_r - r.wall, 0.03)
    outer_r = mouth_r + _OPEN_RIM_PROUD
    ring = (
        cq.Workplane("XY")
        .workplane(offset=r.body_h - _OPEN_RIM_T * 0.5)
        .circle(outer_r)
        .circle(inner_r)
        .extrude(_OPEN_RIM_T)
    )
    return mesh_from_cadquery(ring, "rim_lip")


def _rim_ledge_mesh(r: ResolvedBucket2Config, mouth_r: float) -> object:
    """Lid-seat rim ledge for a lidded keg: a thin ANNULAR ring at the mouth.

    FIX 1 — this REPLACES the old solid disc that filled the bore and visually
    capped the cavity. The bore is the inner cavity radius, so the deep hollow
    staved interior stays fully visible through the mouth when the lid rests
    OPEN. The lid's plug/flange still seats on the ring's inner shoulder."""
    inner_r = max(mouth_r - r.wall, 0.03)
    # ledge sits just inside the wall (small inward shoulder for the lid to seat).
    outer_r = inner_r + 0.006
    ring = (
        cq.Workplane("XY")
        .workplane(offset=r.body_h - _RIM_LEDGE_T)
        .circle(outer_r)
        .circle(max(inner_r - 0.004, 0.028))
        .extrude(_RIM_LEDGE_T)
    )
    return mesh_from_cadquery(ring, "rim_ledge")


def _bunghole_bore_cut(r: ResolvedBucket2Config) -> cq.Workplane:
    """Radial shell-wall bore for the tapered side bung."""
    az = _bung_azimuth(r)
    wall_r = _outer_radius(r, r.bung_z)
    inner_r = max(wall_r - r.wall, 0.02)
    bore_len = r.wall + 0.024
    bore = (
        cq.Workplane("YZ")
        .circle(max(_BUNG_R - 0.003, 0.012))
        .extrude(bore_len)
        .translate((inner_r - 0.012, 0.0, r.bung_z))
    )
    return bore.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), math.degrees(az))


# ---------------------------------------------------------------------------
# Hoop multiplicity (Slot B). Loop-emitted NON-MOVING metal rings FUSED onto the
# barrel_body as parent.visual elements (Rule 1: hoops are coopered decorations,
# not articulated parts). The multiplicity axis is now a *visual-count* axis:
# N hoop_{i} ring visuals at even z-pitch, no joints. The defining articulation
# is carried by the closure (lid / bung) / handle (bail / ring) joints.
# ---------------------------------------------------------------------------
def _build_hoop_mesh(r: ResolvedBucket2Config, name: str, z: float) -> object:
    """One metal hoop band: a thin ring sitting proud of the wall at height z,
    centered at world z so it fuses onto the staved body at its stave radius."""
    rad = _outer_radius(r, z)
    band = (
        cq.Workplane("XY")
        .workplane(offset=z - _HOOP_HALF_H)
        .circle(rad + _HOOP_BAND_T)
        .circle(max(rad - 0.004, 0.02))
        .extrude(2.0 * _HOOP_HALF_H)
    )
    return mesh_from_cadquery(band, name)


def _emit_hoops(model: ArticulatedObject, body, r: ResolvedBucket2Config, mats) -> list[str]:
    """Emit N metal hoop rings as fused visuals on the barrel_body (no joints)."""
    names: list[str] = []
    for i, z in enumerate(_hoop_z_positions(r.hoop_count, r.body_h)):
        body.visual(
            _build_hoop_mesh(r, f"hoop_{i}", z),
            material=mats["metal"],
            name=f"hoop_{i}",
        )
        names.append(f"hoop_{i}")
    return names


# ---------------------------------------------------------------------------
# Closures (Slot C). Each declares >=1 non-FIXED joint with a MatingContract.
# ---------------------------------------------------------------------------
def _emit_open_top(model, body, r: ResolvedBucket2Config, mats) -> list[str]:
    """OPEN-top bucket: no lid, no closure part, no closure joint. The finished
    rim lip is already a body.visual (see _build_body); the wide open mouth
    reveals the deep hollow staved cavity. The required non-FIXED joint is
    supplied entirely by the carry handle (swing_bail / side_ear_rings)."""
    return []


def _emit_slide_off_lid(model, body, r: ResolvedBucket2Config, mats) -> list[str]:
    mouth_r = _mouth_radius(r)
    anchor_x = mouth_r - 0.006  # +X rim point used as the lift-off joint datum
    lid = model.part("barrel_lid")
    # CLOSED-at-rest: the removable lid sits level on the mouth at q=0. The lid
    # is authored in a frame whose origin sits on the +X flange edge (disc
    # extending toward -X), so the PRISMATIC joint origin stays on real rim
    # geometry while positive +Z actuation lifts the whole lid straight up.
    flange = (
        cq.Workplane("XY")
        .workplane(offset=-_LID_FLANGE_T * 0.5)
        .circle(mouth_r - 0.003)
        .extrude(_LID_FLANGE_T)
        .translate((-(mouth_r - 0.003), 0.0, 0.0))
    )
    plug = (
        cq.Workplane("XY")
        .workplane(offset=-_LID_FLANGE_T * 0.5 - _LID_PLUG_DROP)
        .circle(max(mouth_r - r.wall - 0.002, 0.03))
        .extrude(_LID_PLUG_DROP)
        .translate((-(mouth_r - 0.003), 0.0, 0.0))
    )
    lid.visual(
        mesh_from_cadquery(flange.union(plug), "lid_body"),
        material=mats["lid_wood"],
        name="lid_body",
    )
    lid.inertial = Inertial.from_geometry(
        Cylinder(radius=mouth_r, length=_LID_FLANGE_T),
        mass=0.5,
        origin=Origin(xyz=(-(mouth_r - 0.003), 0.0, 0.0)),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lid,
        # Origin stays on the +X rim point; the lid remains level/closed at q=0.
        origin=Origin(xyz=(anchor_x, 0.0, r.body_h)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.3, lower=0.0, upper=r.lid_travel),
    )
    return ["barrel_lid"]


def _emit_hinged_lid(model, body, r: ResolvedBucket2Config, mats) -> list[str]:
    mouth_r = _mouth_radius(r)
    hinge_x = -(mouth_r - 0.006)
    lid = model.part("barrel_lid")
    # Authored in the hinge pivot frame: shift the lid disc so its rear edge
    # sits at the part-local origin (the REVOLUTE pivot line).
    flange = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .circle(mouth_r - 0.003)
        .extrude(_LID_FLANGE_T)
        .translate((-hinge_x, 0.0, 0.0))
    )
    lid.visual(mesh_from_cadquery(flange, "lid_body"), material=mats["lid_wood"], name="lid_body")
    lid.inertial = Inertial.from_geometry(
        Cylinder(radius=mouth_r, length=_LID_FLANGE_T),
        mass=0.5,
        origin=Origin(xyz=(-hinge_x, 0.0, _LID_FLANGE_T / 2.0)),
    )
    # CLOSED-at-rest: the lid starts seated on the mouth at q=0. Positive motion
    # about -Y lifts the front edge UP and back over the hinge, opening the lid.
    rest_open = _HINGE_REST_OPEN
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(hinge_x, 0.0, r.body_h)),
        axis=(0.0, -1.0, 0.0),
        # q in [0, rest_open]: q=0 -> closed on the rim, q=rest_open -> open.
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=rest_open),
    )
    return ["barrel_lid"]


def _bung_azimuth(r: ResolvedBucket2Config) -> float:
    """Azimuth of the bunghole around the body. Default +X; rotated to +Y when a
    side-mounted handle occupies the ±X faces so the bung clears the ears."""
    if r.handle in ("swing_bail", "side_ear_rings"):
        return math.pi / 2.0  # +Y
    return 0.0  # +X


def _emit_bunghole_plug(model, body, r: ResolvedBucket2Config, mats) -> list[str]:
    wall_r = _outer_radius(r, r.bung_z)
    az = _bung_azimuth(r)
    bung = model.part("bung_plug")
    # Tapered bung built along +X (inner tip at part origin, extends +X). The
    # whole part is spun to the chosen azimuth via the joint origin rpy so the
    # bung extends radially outward through the wall.
    taper = (
        cq.Workplane("YZ")
        .circle(_BUNG_R - 0.004)
        .workplane(offset=_BUNG_LEN)
        .circle(_BUNG_R)
        .loft()
    )
    bung.visual(mesh_from_cadquery(taper, "bung_body"), material=mats["lid_wood"], name="bung_body")
    bung.inertial = Inertial.from_geometry(
        Box((_BUNG_LEN, 2.0 * _BUNG_R, 2.0 * _BUNG_R)),
        mass=0.08,
        origin=Origin(xyz=(_BUNG_LEN / 2.0, 0.0, 0.0)),
    )
    ox = (wall_r - 0.010) * math.cos(az)
    oy = (wall_r - 0.010) * math.sin(az)
    axis = (math.cos(az), math.sin(az), 0.0)
    model.articulation(
        "body_to_bung",
        ArticulationType.PRISMATIC,
        parent=body,
        child=bung,
        origin=Origin(xyz=(ox, oy, r.bung_z), rpy=(0.0, 0.0, az)),
        axis=axis,
        motion_limits=MotionLimits(effort=3.0, velocity=0.3, lower=0.0, upper=r.bung_travel),
    )
    return ["bung_plug"]


_CLOSURE_BUILDERS = {
    "open_top": _emit_open_top,
    "slide_off_lid": _emit_slide_off_lid,
    "hinged_lid": _emit_hinged_lid,
    "bunghole_plug": _emit_bunghole_plug,
}


# ---------------------------------------------------------------------------
# Handles (Slot D).
# ---------------------------------------------------------------------------
def _emit_lid_arched_grip(model, body, r: ResolvedBucket2Config, mats) -> list[str]:
    """Arched grip FIXED on the lid (lid closures only). Rule 1 exception:
    the grip is a separate part so it rides the lid's PRISMATIC/REVOLUTE frame.
    """
    lid = model.get_part("barrel_lid")
    mouth_r = _mouth_radius(r)
    # The grip sits at the lid disc CENTER (lid-local x), which differs by closure:
    #   slide_off_lid -> disc shifted to local x = -(mouth_r-0.003) (rim edge at 0)
    #   hinged_lid    -> disc shifted to local x = +(mouth_r-0.006) (-hinge_x)
    if r.closure == "slide_off_lid":
        cx = -(mouth_r - 0.003)
    else:
        cx = mouth_r - 0.006
    grip = model.part("lid_handle")
    # FIX 2: a TALLER + WIDER arched grip (was leg_h=0.040, span<=0.110). The
    # grip now stands well above the lid and spans most of the lid disc so it
    # reads as a real carry handle.
    leg_h = 0.072
    span = min(mouth_r * 1.25, 0.150)
    # The FIXED joint origin is anchored on a real contact point: the lid TOP face
    # (lid-local z) directly under the left grip leg. The flange-top z differs by
    # closure: slide_off_lid flange spans [-T/2,+T/2] -> top at +T/2; hinged_lid
    # flange spans [0,T] -> top at T. The grip geometry is authored in the
    # child-local frame whose origin sits at that anchor, so the left leg base
    # straddles the part origin (origin-on-geometry baseline).
    anchor_x = cx - span * 0.5
    flange_top_z = _LID_FLANGE_T * 0.5 if r.closure == "slide_off_lid" else _LID_FLANGE_T
    anchor_z = flange_top_z
    # leg local z-center so each leg base sits just below the anchor (planted in
    # the lid) and rises by leg_h: base at z=-0.004, top at z=leg_h-0.004.
    leg_zc = leg_h * 0.5 - 0.004
    top_zc = leg_h - 0.004 - 0.008
    for i, sx in enumerate((-1.0, 1.0)):
        # left leg (sx=-1) sits at local x=0 (on the anchor); right leg at +span.
        leg_x = 0.0 if sx < 0 else span
        grip.visual(
            Box((0.022, 0.022, leg_h)),
            origin=Origin(xyz=(leg_x, 0.0, leg_zc)),
            material=mats["accent"],
            name=f"grip_leg_{i}",
        )
    grip.visual(
        Box((span + 0.022, 0.022, 0.020)),
        origin=Origin(xyz=(span * 0.5, 0.0, top_zc)),
        material=mats["accent"],
        name="grip_beam",
    )
    grip.inertial = Inertial.from_geometry(
        Box((span, 0.02, leg_h)),
        mass=0.10,
        origin=Origin(xyz=(span * 0.5, 0.0, leg_h * 0.5)),
    )
    model.articulation(
        "lid_to_handle",
        ArticulationType.FIXED,
        parent=lid,
        child=grip,
        origin=Origin(xyz=(anchor_x, 0.0, anchor_z)),
    )
    return ["lid_handle"]


def _ear_mesh(r: ResolvedBucket2Config, name: str) -> object:
    """Pivot ear: a plate + protruding pin boss, authored so the plate
    straddles the part origin (the FIXED joint origin sits on real material)."""
    plate = (
        cq.Workplane("XY")
        .box(0.018, 0.026, 0.030)
        .translate((-0.006, 0.0, 0.0))
    )
    pin = (
        cq.Workplane("YZ")
        .circle(0.004)
        .extrude(0.024)
        .translate((0.0, 0.0, 0.0))
    )
    return mesh_from_cadquery(plate.union(pin), name)


def _swing_ear_z(r: ResolvedBucket2Config) -> float:
    """Ear pivot height: centered in the highest inter-hoop gap so the ear
    plate clears the metal hoop bands, with a margin from the mouth."""
    zpos = sorted(_hoop_z_positions(r.hoop_count, r.body_h))
    ear_half = 0.016
    # Candidate: midpoint of the top inter-hoop gap.
    if len(zpos) >= 2:
        gap_mid = 0.5 * (zpos[-1] + zpos[-2])
    else:
        gap_mid = r.body_h * 0.78
    z = gap_mid
    # Ensure clearance from the nearest hoop band; if the gap is too tight,
    # fall back to just below the mouth (above the top hoop).
    def clears(zc: float) -> bool:
        return all(abs(zc - hz) > (_HOOP_HALF_H + ear_half + 0.002) for hz in zpos)

    if not clears(z):
        z = max(zpos) + _HOOP_HALF_H + ear_half + 0.004
    return _clamp(z, zpos[0] + 0.02, r.body_h - 0.030)


def _bail_ear_z(r: ResolvedBucket2Config) -> float:
    """Swing-bail pivot height: just BELOW the mouth rim so the bail spans the
    full mouth diameter and arcs over the open mouth (FIX 2). Kept clear of the
    top hoop band; falls just under it if the top hoop sits at the rim."""
    zpos = sorted(_hoop_z_positions(r.hoop_count, r.body_h))
    ear_half = 0.016
    z = r.body_h - _BAIL_EAR_BELOW_RIM
    # If the chosen height collides with the top hoop, drop just below that hoop.
    top_hoop = zpos[-1]
    if abs(z - top_hoop) <= (_HOOP_HALF_H + ear_half + 0.002):
        z = top_hoop - (_HOOP_HALF_H + ear_half + 0.004)
    # Never go below the second hoop / mid-body.
    return _clamp(z, max(zpos[0] + 0.02, r.body_h * 0.55), r.body_h - 0.024)


def _emit_swing_bail(model, body, r: ResolvedBucket2Config, mats) -> list[str]:
    """Two FIXED pivot ears just below the rim + a TALL round bail REVOLUTE +X.

    FIX 2: the bail is now a tall semicircular carry bail — pivot ears sit just
    below the mouth on opposite sides so the bail spans ~the mouth diameter, and
    it arcs WELL ABOVE the rim (apex ~ rim + _BAIL_APEX_ABOVE_RIM). It is still a
    single REVOLUTE bail on real pivot ears (joint origin on a real ear pin)."""
    z_ear = _bail_ear_z(r)
    wall_r = _outer_radius(r, z_ear)
    names: list[str] = []
    for i, sx in enumerate((-1.0, 1.0)):
        ear = model.part(f"ear_{i}")
        ear.visual(_ear_mesh(r, f"ear_body_{i}"), material=mats["metal"], name=f"ear_body_{i}")
        ear.inertial = Inertial.from_geometry(
            Box((0.02, 0.026, 0.030)), mass=0.05, origin=Origin(xyz=(0.0, 0.0, 0.0))
        )
        spin = 0.0 if sx > 0 else math.pi
        model.articulation(
            f"body_to_ear_{i}",
            ArticulationType.FIXED,
            parent=body,
            child=ear,
            origin=Origin(xyz=(sx * (wall_r - 0.004), 0.0, z_ear), rpy=(0.0, 0.0, spin)),
        )
        names.append(f"ear_{i}")

    # Swept bail wire arching between the two pins. The REVOLUTE pivots about the
    # world X line at z=z_ear; for an X-axis revolute the joint origin may sit at
    # *any* x along that line without changing kinematics, so we anchor it on the
    # +X pin (real ear geometry) at pin_x and author the bail in a local frame
    # whose origin coincides with that pin (the +X bail end), so both
    # dist_parent (ear) and dist_child (bail end) are ~0.
    # span = half the bail width; pin-to-pin spans the mouth (~2*wall_r) so the
    # bail width ~ the mouth diameter (FIX 2). The bail arcs UP to an apex well
    # above the rim: apex_z = rim + _BAIL_APEX_ABOVE_RIM, so the rise from the
    # ear (which sits just below the rim) is a tall round bail, not a flat arc.
    span = wall_r + 0.010
    apex_z = r.body_h + _BAIL_APEX_ABOVE_RIM
    pin_x = wall_r - 0.004  # +X ear pin (FIXED ear is mounted here)
    arch = apex_z - z_ear  # local z-rise of the bail apex above the pivot line
    # Local frame: +X bail end at origin (0,0,0); -X end at (-2*span, 0, 0). The
    # bail rises near-vertically off each pin then rounds over a flat-topped apex
    # so it reads as a tall semicircular round bail spanning the mouth.
    bail = model.part("bail_handle")
    bail.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.0, 0.0, 0.0),
                    (-span * 0.16, 0.0, arch * 0.72),
                    (-span * 0.5, 0.0, arch),
                    (-span, 0.0, arch + 0.012),
                    (-span * 1.5, 0.0, arch),
                    (-span * 1.84, 0.0, arch * 0.72),
                    (-2.0 * span, 0.0, 0.0),
                ],
                radius=_BAIL_WIRE_R,
                samples_per_segment=10,
                radial_segments=10,
                cap_ends=True,
            ),
            "bail_wire",
        ),
        material=mats["accent"],
        name="bail_wire",
    )
    bail.inertial = Inertial.from_geometry(
        Box((2.0 * span, 0.02, arch)),
        mass=0.10,
        origin=Origin(xyz=(-span, 0.0, arch * 0.5)),
    )
    model.articulation(
        "body_to_bail",
        ArticulationType.REVOLUTE,
        parent=body,
        child=bail,
        origin=Origin(xyz=(pin_x, 0.0, z_ear)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=-1.4, upper=1.4),
    )
    names.append("bail_handle")
    return names


def _fork_ear_mesh(r: ResolvedBucket2Config, name: str) -> object:
    """Fork bracket ear: a backing plate + two prongs, straddling part origin."""
    plate = cq.Workplane("XY").box(0.016, 0.030, 0.026).translate((-0.005, 0.0, 0.0))
    prongs = None
    for sy in (-0.011, 0.011):
        prong = cq.Workplane("XY").box(0.018, 0.006, 0.010).translate((0.012, sy, 0.0))
        prongs = prong if prongs is None else prongs.union(prong)
    return mesh_from_cadquery(plate.union(prongs), name)


def _emit_side_ear_rings(model, body, r: ResolvedBucket2Config, mats) -> list[str]:
    """Two FIXED fork ears on ±X + a LARGE fold-out carry ring REVOLUTE +X on each.

    FIX 2: the carry rings are noticeably larger (bigger ring radius + thicker
    tube) so they read as clearly graspable hand rings, not tiny pin rings."""
    z_ear = _swing_ear_z(r)
    wall_r = _outer_radius(r, z_ear)
    ring_r = 0.052  # was 0.030 — a clearly graspable carry ring
    ring_tube = 0.007  # was 0.004 — thicker so the ring reads
    names: list[str] = []
    for i, sx in enumerate((-1.0, 1.0)):
        ear = model.part(f"ear_{i}")
        ear.visual(_fork_ear_mesh(r, f"ear_body_{i}"), material=mats["metal"], name=f"ear_body_{i}")
        ear.inertial = Inertial.from_geometry(
            Box((0.03, 0.030, 0.026)), mass=0.05, origin=Origin(xyz=(0.0, 0.0, 0.0))
        )
        spin = 0.0 if sx > 0 else math.pi
        model.articulation(
            f"body_to_ear_{i}",
            ArticulationType.FIXED,
            parent=body,
            child=ear,
            origin=Origin(xyz=(sx * (wall_r - 0.004), 0.0, z_ear), rpy=(0.0, 0.0, spin)),
        )
        names.append(f"ear_{i}")

        # Fold-out ring: a torus looping around the X axis (folds about X). The
        # torus is centered on the part origin (0,0,0) so the REVOLUTE origin
        # lies on real ring geometry; it hangs down (-Z) at rest.
        ring = model.part(f"ring_{i}")
        ring_torus = TorusGeometry(ring_r, ring_tube, radial_segments=12, tubular_segments=20)
        ring.visual(
            mesh_from_geometry(ring_torus, f"ring_body_{i}"),
            origin=Origin(xyz=(0.0, 0.0, -ring_r), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=mats["metal"],
            name=f"ring_body_{i}",
        )
        ring.inertial = Inertial.from_geometry(
            Box((0.012, 2.0 * ring_r, 2.0 * ring_r)),
            mass=0.04,
            origin=Origin(xyz=(0.0, 0.0, -ring_r)),
        )
        model.articulation(
            f"ear_{i}_to_ring_{i}",
            ArticulationType.REVOLUTE,
            parent=ear,
            child=ring,
            origin=Origin(xyz=(0.018, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=r.ring_open),
        )
        names.append(f"ring_{i}")
    return names


def _emit_no_handle(model, body, r: ResolvedBucket2Config, mats) -> list[str]:
    return []


_HANDLE_BUILDERS = {
    "lid_arched_grip": _emit_lid_arched_grip,
    "swing_bail": _emit_swing_bail,
    "side_ear_rings": _emit_side_ear_rings,
    "no_handle": _emit_no_handle,
}


# ---------------------------------------------------------------------------
# slot_choices
# ---------------------------------------------------------------------------
def slot_choices_for_config(
    config: Bucket2Config | ResolvedBucket2Config,
) -> tuple[tuple[str, str], ...]:
    r = config if isinstance(config, ResolvedBucket2Config) else resolve_config(config)
    return (
        ("body_profile", r.body_profile),
        ("hoop_count", f"n{r.hoop_count}"),
        ("closure", r.closure),
        ("handle", r.handle),
    )


def slot_choices_for_seed(seed: int) -> tuple[tuple[str, str], ...]:
    return slot_choices_for_config(config_from_seed(seed))


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------
def build_bucket2(
    config: Bucket2Config | None = None,
    *,
    assets: AssetContext | None = None,
) -> ArticulatedObject:
    r = resolve_config(config)
    model = ArticulatedObject(name=r.name, assets=assets)
    mats = {
        key: model.material(f"bucket2_{key}_{r.palette_style}", rgba=rgba)
        for key, rgba in PALETTES[r.palette_style].items()
    }

    body = _build_body(model, r, mats)
    _emit_hoops(model, body, r, mats)
    _CLOSURE_BUILDERS[r.closure](model, body, r, mats)
    _HANDLE_BUILDERS[r.handle](model, body, r, mats)

    model.meta["slot_choices"] = slot_choices_for_config(r)
    return model


def build_seeded_bucket2(
    seed: int,
    *,
    assets: AssetContext | None = None,
) -> ArticulatedObject:
    return build_bucket2(config_from_seed(seed), assets=assets)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_bucket2_tests(
    object_model: ArticulatedObject,
    config: Bucket2Config,
) -> TestReport:
    r = resolve_config(config)
    ctx = TestContext(object_model)
    body = object_model.get_part("barrel_body")

    # ---- Coopered-fit / captured-pin allowances (element-scoped). ----
    # Hoops are now FUSED visuals on the body itself (no separate parts), so no
    # part-vs-part overlap allowance is needed for the coopered hoop fit.

    if r.closure == "open_top":
        # open_top has no lid/bung part -> nothing seats on the mouth; the rim
        # lip is a body visual. No closure-vs-body overlap allowance needed.
        pass
    elif r.closure == "slide_off_lid":
        # The removable lid RESTS seated on the mouth at q=0. Allow the rim seat
        # contacts plus the seating plug entering the cavity bore.
        ctx.allow_overlap(
            object_model.get_part("barrel_lid"), body, elem_a="lid_body", elem_b="rim_ledge",
            reason="lift-off lid seats on the rim ledge when closed.",
        )
        ctx.allow_overlap(
            object_model.get_part("barrel_lid"), body, elem_a="lid_body", elem_b="staved_shell",
            reason="lift-off lid seats on the shell rim when closed.",
        )
        ctx.allow_overlap(
            object_model.get_part("barrel_lid"), body, elem_a="lid_body", elem_b="cavity_liner",
            reason="lift-off lid's seating plug dips into the open cavity bore it seals against.",
        )
    elif r.closure == "hinged_lid":
        # The lid RESTS closed about the rear-rim hinge; allow the seated lid/rim
        # contact plus the hinge edge meeting the inner liner at the rear rim.
        ctx.allow_overlap(
            object_model.get_part("barrel_lid"), body, elem_a="lid_body", elem_b="rim_ledge",
            reason="hinged lid seats on the rim ledge when closed.",
        )
        ctx.allow_overlap(
            object_model.get_part("barrel_lid"), body, elem_a="lid_body", elem_b="staved_shell",
            reason="hinged lid seats on the shell rim when closed.",
        )
        ctx.allow_overlap(
            object_model.get_part("barrel_lid"), body, elem_a="lid_body", elem_b="cavity_liner",
            reason="hinged lid's rear hinge edge meets the inner cavity face at the rim.",
        )
    else:  # bunghole_plug
        ctx.allow_overlap(
            object_model.get_part("bung_plug"), body, elem_a="bung_body", elem_b="staved_shell",
            reason="tapered bung is driven into the bunghole bore through the shell wall.",
        )

    if r.handle == "lid_arched_grip":
        grip = object_model.get_part("lid_handle")
        lid = object_model.get_part("barrel_lid")
        for i in range(2):
            ctx.allow_overlap(
                grip, lid, elem_a=f"grip_leg_{i}", elem_b="lid_body",
                reason=f"grip leg {i} is screwed into the lid top.",
            )
    elif r.handle == "swing_bail":
        for i in range(2):
            ctx.allow_overlap(
                object_model.get_part(f"ear_{i}"), body, elem_a=f"ear_body_{i}", elem_b="staved_shell",
                reason=f"pivot ear {i} is fastened to the shell wall.",
            )
            # ear is fastened through the wall just below the rim -> its inner
            # face meets the dark cavity liner (the inner shell surface).
            ctx.allow_overlap(
                object_model.get_part(f"ear_{i}"), body, elem_a=f"ear_body_{i}", elem_b="cavity_liner",
                reason=f"pivot ear {i} is fastened through the wall to the inner cavity face.",
            )
            ctx.allow_overlap(
                object_model.get_part("bail_handle"), object_model.get_part(f"ear_{i}"),
                elem_a="bail_wire", elem_b=f"ear_body_{i}",
                reason=f"bail wire end {i} is captured on the ear pin.",
            )
        # The swung-down bail rests against the staved shell (and the closed lid
        # at the mouth) along its length — a real coopered-keg bail contact.
        ctx.allow_overlap(
            object_model.get_part("bail_handle"), body, elem_a="bail_wire", elem_b="staved_shell",
            reason="the bail handle hangs against the barrel shell at rest.",
        )
        # the bail's pin ends sit at the mouth wall where the inner cavity liner
        # is, so the wire crosses the inner cavity face near the pins.
        ctx.allow_overlap(
            object_model.get_part("bail_handle"), body, elem_a="bail_wire", elem_b="cavity_liner",
            reason="the bail's pin ends cross the inner cavity face at the mouth wall.",
        )
        for hi in range(r.hoop_count):
            ctx.allow_overlap(
                object_model.get_part("bail_handle"), body,
                elem_a="bail_wire", elem_b=f"hoop_{hi}",
                reason=f"the bail sweeps past hoop band {hi} when raised.",
            )
        if r.closure in LID_CLOSURES:
            ctx.allow_overlap(
                object_model.get_part("bail_handle"), object_model.get_part("barrel_lid"),
                elem_a="bail_wire", elem_b="lid_body",
                reason="the bail arches over the closed lid at the mouth.",
            )
            ctx.allow_overlap(
                object_model.get_part("bail_handle"), body,
                elem_a="bail_wire", elem_b="rim_ledge",
                reason="the bail passes over the mouth rim ledge.",
            )
        if r.closure == "open_top":
            ctx.allow_overlap(
                object_model.get_part("bail_handle"), body,
                elem_a="bail_wire", elem_b="rim_lip",
                reason="the bail arches over the open mouth rim lip.",
            )
        if r.closure == "bunghole_plug":
            ctx.allow_overlap(
                object_model.get_part("bail_handle"), body,
                elem_a="bail_wire", elem_b="rim_lip",
                reason="the bail passes over the open mouth rim lip on the bunghole keg.",
            )
    elif r.handle == "side_ear_rings":
        for i in range(2):
            ctx.allow_overlap(
                object_model.get_part(f"ear_{i}"), body, elem_a=f"ear_body_{i}", elem_b="staved_shell",
                reason=f"fork ear {i} is fastened to the shell wall.",
            )
            ctx.allow_overlap(
                object_model.get_part(f"ear_{i}"), body, elem_a=f"ear_body_{i}", elem_b="cavity_liner",
                reason=f"fork ear {i} is fastened through the wall to the inner cavity face.",
            )
            ctx.allow_overlap(
                object_model.get_part(f"ring_{i}"), object_model.get_part(f"ear_{i}"),
                elem_a=f"ring_body_{i}", elem_b=f"ear_body_{i}",
                reason=f"ring {i} pivot pin is captured between the fork prongs.",
            )
            # The hanging ring rests against the shell at rest.
            ctx.allow_overlap(
                object_model.get_part(f"ring_{i}"), body, elem_a=f"ring_body_{i}", elem_b="staved_shell",
                reason=f"fold-out ring {i} hangs against the barrel shell at rest.",
            )
            # ...and may drape over a metal hoop band on the way down.
            for hi in range(r.hoop_count):
                ctx.allow_overlap(
                    object_model.get_part(f"ring_{i}"), body,
                    elem_a=f"ring_body_{i}", elem_b=f"hoop_{hi}",
                    reason=f"fold-out ring {i} drapes over hoop band {hi} as it hangs.",
                )

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_joint_mating_has_gap()

    # ---- Structure / identity checks. ----
    part_names = {p.name for p in object_model.parts}
    ctx.check("barrel_body root present", "barrel_body" in part_names, details=str(sorted(part_names)))

    # N hoops loop-emitted as FUSED visuals on the body (visual-count
    # multiplicity axis; hoops are non-moving coopered decorations, not parts).
    body_elem_names = {v.name for v in body.visuals}
    hoop_visuals = [n for n in body_elem_names if n.startswith("hoop_")]
    ctx.check(
        "N hoops loop-emitted as fused body visuals (multiplicity axis)",
        len(hoop_visuals) == r.hoop_count and r.hoop_count >= 2,
        details=f"hoop_visuals={sorted(hoop_visuals)} N={r.hoop_count}",
    )
    # No hoop is a separate jointed part (Rule 1: hoops do not move).
    hoop_parts = [n for n in part_names if n.startswith("hoop_")]
    ctx.check(
        "hoops are not separate articulated parts",
        len(hoop_parts) == 0,
        details=f"stray hoop parts={sorted(hoop_parts)}",
    )

    # Closure joint topology.
    if r.closure == "open_top":
        # open_top emits NO closure part and NO closure joint: the open mouth is
        # finished only by the body's rim_lip visual. Assert there is no covering
        # lid/head mesh anywhere (the mouth must read OPEN) and that the rim lip
        # is present.
        all_part_names = {p.name for p in object_model.parts}
        ctx.check(
            "open_top has no lid / bung / head part (open mouth, nothing covers it)",
            "barrel_lid" not in all_part_names and "bung_plug" not in all_part_names,
            details=f"parts={sorted(all_part_names)}",
        )
        joint_names = {a.name for a in object_model.articulations}
        ctx.check(
            "open_top has no body_to_lid / body_to_bung closure joint",
            "body_to_lid" not in joint_names and "body_to_bung" not in joint_names,
            details=f"closure joint must come from the handle, not the mouth; joints={sorted(joint_names)}",
        )
        body_vis_names = {v.name for v in body.visuals}
        ctx.check(
            "open_top mouth has a finished rim lip and NO sealing top_head",
            "rim_lip" in body_vis_names and "top_head" not in body_vis_names,
            details=f"body_visuals={sorted(body_vis_names)}",
        )
    elif r.closure == "slide_off_lid":
        j = object_model.get_articulation("body_to_lid")
        mouth_r = _mouth_radius(r)
        slide_anchor_r = math.hypot(j.origin.xyz[0], j.origin.xyz[1])
        ctx.check(
            "slide-off lid is a lift-off PRISMATIC about +Z",
            j.articulation_type == ArticulationType.PRISMATIC
            and abs(j.axis[2]) > 0.99
            and j.motion_limits is not None
            and j.motion_limits.lower == 0.0
            and j.motion_limits.upper is not None
            and j.motion_limits.upper > 0.0,
            details=(
                f"type={j.articulation_type} axis={tuple(j.axis)} "
                f"limits={j.motion_limits}"
            ),
        )
        ctx.check(
            "slide-off lid joint origin stays on the mouth rim plane",
            abs(j.origin.xyz[2] - r.body_h) <= 1e-6
            and abs(slide_anchor_r - (mouth_r - 0.006)) <= 0.020,
            details=(
                f"origin_xyz={tuple(j.origin.xyz)} rim_z={r.body_h:.4f} "
                f"anchor_r={slide_anchor_r:.4f} mouth_r={mouth_r:.4f}"
            ),
        )
    elif r.closure == "hinged_lid":
        j = object_model.get_articulation("body_to_lid")
        ctx.check(
            "hinged lid is REVOLUTE about Y",
            j.articulation_type == ArticulationType.REVOLUTE and abs(j.axis[1]) > 0.99,
            details=f"type={j.articulation_type} axis={tuple(j.axis)}",
        )
    else:
        j = object_model.get_articulation("body_to_bung")
        radial = abs(j.axis[2]) < 0.01 and (abs(j.axis[0]) > 0.99 or abs(j.axis[1]) > 0.99)
        ctx.check(
            "bunghole plug is PRISMATIC radial (horizontal, through the wall)",
            j.articulation_type == ArticulationType.PRISMATIC and radial,
            details=f"type={j.articulation_type} axis={tuple(j.axis)}",
        )

    # Handle joint topology.
    if r.handle == "swing_bail":
        j = object_model.get_articulation("body_to_bail")
        ctx.check(
            "swing bail is REVOLUTE about X",
            j.articulation_type == ArticulationType.REVOLUTE and abs(j.axis[0]) > 0.99,
            details=f"type={j.articulation_type} axis={tuple(j.axis)}",
        )
    elif r.handle == "side_ear_rings":
        for i in range(2):
            j = object_model.get_articulation(f"ear_{i}_to_ring_{i}")
            ctx.check(
                f"ring {i} is REVOLUTE about X",
                j.articulation_type == ArticulationType.REVOLUTE and abs(j.axis[0]) > 0.99,
                details=f"type={j.articulation_type} axis={tuple(j.axis)}",
            )
    elif r.handle == "lid_arched_grip":
        ctx.check(
            "lid_arched_grip only on lid closures",
            r.closure in LID_CLOSURES,
            details=f"closure={r.closure}",
        )
    elif r.handle == "no_handle":
        ctx.check(
            "no_handle only on bunghole closure",
            r.closure == "bunghole_plug",
            details=f"closure={r.closure}",
        )

    # ---- Closure actuation opens / exposes the keg. ----
    if r.closure == "open_top":
        # No closure joint to actuate; instead assert the mouth is ALREADY open
        # (no covering lid mesh) and the deep hollow staved cavity is visible, and
        # that the required >=1 non-FIXED joint is carried by the handle.
        ctx.check(
            "open_top is paired with a joint-bearing carry handle (bail / ring)",
            r.handle in OPEN_HANDLES,
            details=f"handle={r.handle}",
        )
        non_fixed = [
            a for a in object_model.articulations
            if a.articulation_type != ArticulationType.FIXED
        ]
        ctx.check(
            "open_top has >=1 non-FIXED joint (from the handle)",
            len(non_fixed) >= 1,
            details=f"non_fixed={[a.name for a in non_fixed]}",
        )
        # Mouth opening width (open mouth) and inner-wall cavity depth must read.
        # FIX 1: cavity is now DEEP for every config (thin floor) — threshold
        # raised from 0.10 to 0.20 so the hollow is a real deep cavity, not a
        # shallow tray.
        mouth_r = _mouth_radius(r)
        inner_mouth_r = max(mouth_r - r.wall, 0.03)
        cavity_depth = r.body_h - r.floor_t
        ctx.check(
            "open_top exposes a wide open mouth and a deep hollow cavity",
            inner_mouth_r > 0.045 and cavity_depth > 0.20,
            details=f"inner_mouth_r={inner_mouth_r:.4f} cavity_depth={cavity_depth:.4f}",
        )
        # The body shell AABB top must coincide with the open mouth (no lid mesh
        # sitting above the rim lip) — confirms nothing covers the mouth.
        shell_aabb = ctx.part_world_aabb(body)
        if shell_aabb is not None:
            (_, _, _), (_, _, body_top_z) = shell_aabb
            ctx.check(
                "open mouth: nothing rises above the body rim lip",
                body_top_z <= r.body_h + _OPEN_RIM_T + 0.012,
                details=f"body_top_z={body_top_z:.4f} mouth_z={r.body_h:.4f}",
            )
    elif r.closure == "slide_off_lid":
        # PRIMARY identity: a lidded keg whose removable lid starts CLOSED/level
        # on the mouth at q=0 and then lifts straight up along +Z.
        j = object_model.get_articulation("body_to_lid")
        lid = object_model.get_part("barrel_lid")
        lid_rest = ctx.part_world_aabb(lid)
        if lid_rest is not None:
            (_, _, lid_zmn), (_, _, lid_zmx) = lid_rest
            z_span = lid_zmx - lid_zmn
            ctx.check(
                "slide-off lid RESTS closed and level over the mouth at q=0",
                lid_zmx <= r.body_h + 0.040 and z_span <= (_LID_FLANGE_T + _LID_PLUG_DROP + 0.030),
                details=(
                    f"lid_z=[{lid_zmn:.3f},{lid_zmx:.3f}] z_span={z_span:.3f} "
                    f"rim_z={r.body_h:.3f} "
                    f"(closed lift-off lid should sit near rim height with modest z-span)"
                ),
            )
        p0 = ctx.part_world_position(lid)
        with ctx.pose({j: r.lid_travel * 0.9}):
            p1 = ctx.part_world_position(lid)
        if p0 is not None and p1 is not None:
            xy_drift = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
            z_lift = p1[2] - p0[2]
            ctx.check(
                "slide-off lid lifts upward with minimal lateral drift when actuated",
                z_lift > 0.040 and z_lift >= 3.0 * xy_drift and abs(p1[1] - p0[1]) < 0.020,
                details=(
                    f"rest={tuple(round(v, 3) for v in p0)} "
                    f"lifted={tuple(round(v, 3) for v in p1)} "
                    f"z_lift={z_lift:.4f} xy_drift={xy_drift:.4f}"
                ),
            )
    elif r.closure == "hinged_lid":
        # PRIMARY identity: a lidded keg whose rear-rim flip lid starts CLOSED at
        # q=0 and then swings OPEN ~105 deg under positive actuation.
        j = object_model.get_articulation("body_to_lid")
        lid = object_model.get_part("barrel_lid")
        rest = ctx.part_world_aabb(lid)
        mouth_r = _mouth_radius(r)
        if rest is not None:
            (lid_xmn, _, lid_zmn), (lid_xmx, _, lid_zmx) = rest
            ctx.check(
                "hinged lid RESTS closed over the mouth at q=0",
                lid_zmx <= r.body_h + 0.040 and lid_xmx > 0.35 * mouth_r and lid_xmn < -0.35 * mouth_r,
                details=(
                    f"lid_z=[{lid_zmn:.3f},{lid_zmx:.3f}] rim_z={r.body_h:.3f} "
                    f"lid_x=[{lid_xmn:.3f},{lid_xmx:.3f}] mouth_r={mouth_r:.3f} "
                    f"(closed lid should sit near rim height and span the mouth)"
                ),
            )
        # Actuating toward the upper limit swings the lid OPEN above and back off
        # the mouth so the cavity is exposed.
        rest_top = rest[1][2] if rest is not None else None
        with ctx.pose({j: _HINGE_REST_OPEN * 0.9}):
            opened = ctx.part_world_aabb(lid)
        if rest_top is not None and opened is not None:
            (open_xmn, _, open_zmn), (open_xmx, _, open_zmx) = opened
            ctx.check(
                "hinged lid rides a real movable REVOLUTE joint (swings open)",
                open_zmx > rest_top + 0.120 and open_xmx < 0.5 * mouth_r,
                details=(
                    f"closed_top={rest_top:.3f} open_z=[{open_zmn:.3f},{open_zmx:.3f}] "
                    f"open_x=[{open_xmn:.3f},{open_xmx:.3f}] mouth_r={mouth_r:.3f}"
                ),
            )
    else:
        j = object_model.get_articulation("body_to_bung")
        bung = object_model.get_part("bung_plug")
        p0 = ctx.part_world_position(bung)
        with ctx.pose({j: r.bung_travel * 0.9}):
            p1 = ctx.part_world_position(bung)
        if p0 is not None and p1 is not None:
            radial_disp = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
            ctx.check(
                "bung pulls radially out through the wall",
                radial_disp > 0.030,
                details=f"seated=({p0[0]:.3f},{p0[1]:.3f}) pulled=({p1[0]:.3f},{p1[1]:.3f})",
            )

    # ---- Lidded identity: a real lid PART on a real non-FIXED joint. ----
    # For lid closures, assert the lid part exists and the model carries >=1
    # non-FIXED joint (the lid's own REVOLUTE/PRISMATIC closure joint qualifies).
    if r.closure in LID_CLOSURES:
        all_part_names = {p.name for p in object_model.parts}
        ctx.check(
            "lidded seed has a real lid PART present",
            "barrel_lid" in all_part_names,
            details=f"parts={sorted(all_part_names)}",
        )
        lid_joint = object_model.get_articulation("body_to_lid")
        ctx.check(
            "lid rides a real non-FIXED joint",
            lid_joint.articulation_type != ArticulationType.FIXED,
            details=f"body_to_lid type={lid_joint.articulation_type}",
        )
    non_fixed_all = [
        a for a in object_model.articulations if a.articulation_type != ArticulationType.FIXED
    ]
    ctx.check(
        "model has >=1 non-FIXED joint",
        len(non_fixed_all) >= 1,
        details=f"non_fixed={[a.name for a in non_fixed_all]}",
    )

    # ---- Body base on the ground (z=0). ----
    aabb = ctx.part_world_aabb(body)
    if aabb is not None:
        (axmn, aymn, azmn), (axmx, aymx, azmx) = aabb
        ctx.check("barrel base near the ground", azmn < 0.03, details=f"z_min={azmn:.4f}")

    # ---- Wall is a thin shell (identity: hollow coopered keg). ----
    mouth_r = _mouth_radius(r)
    ctx.check(
        "wall is a thin shell (inner_r > 0)",
        r.wall < 0.5 * mouth_r and (mouth_r - r.wall) > 0.04,
        details=f"wall={r.wall:.4f} mouth_r={mouth_r:.4f}",
    )

    # ---- FIX 1: DEEP hollow cavity in EVERY config (not just open_top). ----
    # The inner floor is sunk low (thin floor) for hinged_lid / slide_off_lid /
    # open_top / bunghole alike, so the staved interior is a real deep cavity.
    cavity_depth = r.body_h - r.floor_t
    ctx.check(
        "deep hollow cavity in every config (thin sunk floor)",
        cavity_depth > 0.20 and r.floor_t <= 0.016,
        details=f"cavity_depth={cavity_depth:.4f} floor_t={r.floor_t:.4f}",
    )
    # Every config now carries the dark recessed inner liner + floor disc so the
    # hollow reads consistently, including bunghole kegs. Lidded closures keep an
    # open annular rim ledge; open-top and bunghole use a finished rim lip.
    body_vis_names = {v.name for v in body.visuals}
    ctx.check(
        "every config has a dark inner cavity liner + floor (reads hollow)",
        "cavity_liner" in body_vis_names and "cavity_floor" in body_vis_names,
        details=f"body_visuals={sorted(body_vis_names)}",
    )
    if r.closure in LID_CLOSURES:
        # The lid-seat rim ledge must be the OPEN annular ring built by
        # _rim_ledge_mesh (a thin ring whose bore is the cavity radius), NOT a
        # solid disc filling the mouth.
        ledge = next((v for v in body.visuals if v.name == "rim_ledge"), None)
        ledge_is_ring = ledge is not None and not isinstance(ledge.geometry, Cylinder)
        ctx.check(
            "lidded rim ledge is an open annular ring (does NOT cap the bore)",
            ledge_is_ring,
            details=(
                f"rim_ledge present={ledge is not None} "
                f"is_solid_cylinder={isinstance(getattr(ledge, 'geometry', None), Cylinder)}"
            ),
        )
    if r.closure == "bunghole_plug":
        ctx.check(
            "bunghole keg keeps an open mouth rim and no sealed top head visuals",
            "rim_lip" in body_vis_names and "top_head" not in body_vis_names and "head_recess" not in body_vis_names,
            details=f"body_visuals={sorted(body_vis_names)}",
        )

    # ---- FIX 2: swing bail is a TALL ROUND bail spanning the mouth. ----
    if r.handle == "swing_bail":
        bail = object_model.get_part("bail_handle")
        bail_aabb = ctx.part_world_aabb(bail)
        if bail_aabb is not None:
            (bxmn, _, bzmn), (bxmx, _, bzmx) = bail_aabb
            apex_above_rim = bzmx - r.body_h
            bail_width = bxmx - bxmn
            mouth_diam = 2.0 * mouth_r
            ctx.check(
                "swing bail arcs TALL above the rim (apex >= rim + 0.20)",
                apex_above_rim >= 0.20,
                details=f"apex_above_rim={apex_above_rim:.4f} rim_z={r.body_h:.4f}",
            )
            ctx.check(
                "swing bail spans ~ the mouth diameter (wide carry bail)",
                bail_width >= 0.80 * mouth_diam,
                details=f"bail_width={bail_width:.4f} mouth_diam={mouth_diam:.4f}",
            )
        # Still a single REVOLUTE bail on real pivot ears (not a Box downgrade).
        jb = object_model.get_articulation("body_to_bail")
        ctx.check(
            "swing bail is one REVOLUTE bail (no Box downgrade)",
            jb.articulation_type == ArticulationType.REVOLUTE
            and "bail_handle" in {p.name for p in object_model.parts},
            details=f"type={jb.articulation_type}",
        )

    # ---- Hoop pitch no-overlap (N>=2). ----
    zpos = _hoop_z_positions(r.hoop_count, r.body_h)
    if len(zpos) >= 2:
        pitch = min(zpos[i + 1] - zpos[i] for i in range(len(zpos) - 1))
        ctx.check(
            "hoop pitch exceeds band height",
            pitch >= 2.0 * _HOOP_HALF_H,
            details=f"pitch={pitch:.4f} band={2.0*_HOOP_HALF_H:.4f}",
        )

    # ---- slot_choices recorded with N encoded. ----
    ctx.check(
        "slot_choices recorded with hoop_count encoded",
        tuple(object_model.meta.get("slot_choices", ())) == slot_choices_for_config(r),
        details=str(object_model.meta.get("slot_choices")),
    )

    return ctx.report()


__all__ = (
    "Bucket2Config",
    "ResolvedBucket2Config",
    "build_bucket2",
    "build_seeded_bucket2",
    "config_from_seed",
    "resolve_config",
    "run_bucket2_tests",
    "slot_choices_for_config",
    "slot_choices_for_seed",
    "with_overrides",
)
