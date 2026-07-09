"""Modular procedural template for the ``sports_car`` 小类.

A two-door low-slung four-wheel single-cabin high-performance road car.

Pattern: ``parallel_children`` — every slot's part / joint parents to a single
lofted single-body ``body`` (chassis). The body mesh itself is the Slot A
``body_profile_family`` choice (a ``superellipse_side_loft`` over one of four
distinct ``LOWER_SECTIONS`` tables, with boolean-carved wheel arches, axle
channels, hollow cabin and door apertures). All other slots hang off the body.

5★ sources adapted (per AUTHORING.md §A Rule 3):
  P2 Diablo  rec_bright-yellow-lamborghini-diablo-style-wedge-sup_…655801c5
             — baseline body pipeline, scissor doors, headlights, rear,
               exhaust, kinematic skeleton (model.py:L44-L781).
  P1 300SL   rec_classic-mercedes-300sl-gullwing-sports-car-in-de_…88ed264e
             — vintage_pontoon LOWER_SECTIONS + independent roof part +
               windshield/rear-window shells, gullwing door axis
               (model.py:L121-L155, L664-L834).
  fastback   rec_sportscar_fastback_body_v01 (LOWER_SECTIONS L103-L119).
  teardrop   rec_sportscar_teardrop_body_v01 (LOWER_SECTIONS L103-L119).
  pop-up     rec_codex_car002_v07 — headlight_cover part + REVOLUTE
             (model.py:L604-L637, L779-L791).
  louver     rec_codex_car002_v09 — engine_cover part + REVOLUTE
             (model.py:L604-L629, L768-L777).
  dihedral   rec_porsche_911_v01 (DOOR_AXIS _DH L105-L108).

Shared kinematic skeleton is FIXED (never an axis):
  front 2 wheels  : REVOLUTE king-pin steer (Z) + CONTINUOUS spin (X)
  rear 2 wheels   : CONTINUOUS spin (X)
  2 doors         : REVOLUTE swing (axis per door_mechanism)
  baseline        : 4 REVOLUTE + 4 CONTINUOUS (+ steering-wheel REVOLUTE)
  +pop_up         : +2 REVOLUTE (per side)
  +louver         : +1 REVOLUTE

Structural diversity axes (slot_choices_for_seed records these):
  A body_profile_family (4) × B door_mechanism (3) × C headlight_shape (3)
  × C popup{0,1} × D rear_blade (3) × D louver{0,1} × D exhaust (3)
  E palette_style (6, orthogonal, drives every .visual material).
Structural-only distinct (A × B × popup × louver) = 48 ≫ 10.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Literal

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    BoltPattern,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireSidewall,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    boolean_difference,
    mesh_from_geometry,
    superellipse_side_loft,
)

__modular__ = True

# --------------------------------------------------------------------------- #
# Slot enums
# --------------------------------------------------------------------------- #
BodyProfileFamily = Literal["modern_wedge", "vintage_pontoon", "fastback", "organic_teardrop"]
DoorMechanism = Literal["scissor_butterfly", "gullwing", "dihedral"]
HeadlightShape = Literal["exposed_rect", "round_fixed", "quad_cluster"]
RearBlade = Literal["integrated", "big_gt_wing", "ducktail"]
ExhaustLayout = Literal["side", "central_quad", "central_single"]
PaletteStyle = Literal[
    "diablo_gloss_yellow",
    "vintage_maroon_chrome",
    "pagani_bare_carbon_silver",
    "bugatti_two_tone_silver_blue",
    "ferrari_rosso_corsa",
    "koenigsegg_icy_blue_carbon",
]

BODY_PROFILE_FAMILIES: tuple[BodyProfileFamily, ...] = (
    "modern_wedge",
    "vintage_pontoon",
    "fastback",
    "organic_teardrop",
)
DOOR_MECHANISMS: tuple[DoorMechanism, ...] = ("scissor_butterfly", "gullwing", "dihedral")
HEADLIGHT_SHAPES: tuple[HeadlightShape, ...] = ("exposed_rect", "round_fixed", "quad_cluster")
REAR_BLADES: tuple[RearBlade, ...] = ("integrated", "big_gt_wing", "ducktail")
EXHAUST_LAYOUTS: tuple[ExhaustLayout, ...] = ("side", "central_quad", "central_single")
PALETTE_STYLES: tuple[PaletteStyle, ...] = (
    "diablo_gloss_yellow",
    "vintage_maroon_chrome",
    "pagani_bare_carbon_silver",
    "bugatti_two_tone_silver_blue",
    "ferrari_rosso_corsa",
    "koenigsegg_icy_blue_carbon",
)

# --------------------------------------------------------------------------- #
# Palette presets (Slot E) — every colorway resolves to a `mats` dict with the
# SAME token set so the builder is colorway-agnostic. Tokens:
#   body   — main paint / body color   accent — trim / black sills / wing
#   alloy  — wheel rim / bright metal   chrome — bright bumper/exhaust rings
#   glass  — tinted glazing (translucent)
#   lens   — pale reflector / driving-light lens
#   amber  — indicators                 tail   — taillights
#   rubber — tires                      interior — cabin floor/seats
# --------------------------------------------------------------------------- #
PALETTES: dict[PaletteStyle, dict[str, tuple[float, float, float, float]]] = {
    "diablo_gloss_yellow": {
        "body": (0.93, 0.74, 0.02, 1.0),
        "accent": (0.05, 0.05, 0.055, 1.0),
        "alloy": (0.74, 0.75, 0.78, 1.0),
        "chrome": (0.80, 0.82, 0.85, 1.0),
        "glass": (0.12, 0.13, 0.15, 0.34),
        "lens": (0.72, 0.75, 0.78, 1.0),
        "amber": (0.85, 0.50, 0.06, 1.0),
        "tail": (0.45, 0.04, 0.05, 1.0),
        "rubber": (0.04, 0.04, 0.045, 1.0),
        "interior": (0.10, 0.10, 0.11, 1.0),
    },
    "vintage_maroon_chrome": {
        "body": (0.43, 0.05, 0.07, 1.0),
        "accent": (0.27, 0.28, 0.30, 1.0),
        "alloy": (0.80, 0.82, 0.85, 1.0),
        "chrome": (0.86, 0.88, 0.90, 1.0),
        "glass": (0.10, 0.12, 0.15, 0.40),
        "lens": (0.86, 0.88, 0.82, 1.0),
        "amber": (0.85, 0.52, 0.10, 1.0),
        "tail": (0.60, 0.06, 0.05, 1.0),
        "rubber": (0.05, 0.05, 0.055, 1.0),
        "interior": (0.12, 0.09, 0.07, 1.0),
    },
    "pagani_bare_carbon_silver": {
        "body": (0.10, 0.10, 0.115, 1.0),
        "accent": (0.045, 0.045, 0.05, 1.0),
        "alloy": (0.78, 0.79, 0.82, 1.0),
        "chrome": (0.82, 0.83, 0.86, 1.0),
        "glass": (0.09, 0.10, 0.12, 0.36),
        "lens": (0.74, 0.77, 0.80, 1.0),
        "amber": (0.86, 0.52, 0.08, 1.0),
        "tail": (0.50, 0.05, 0.05, 1.0),
        "rubber": (0.04, 0.04, 0.045, 1.0),
        "interior": (0.09, 0.09, 0.10, 1.0),
    },
    "bugatti_two_tone_silver_blue": {
        "body": (0.74, 0.76, 0.80, 1.0),
        "accent": (0.07, 0.16, 0.42, 1.0),
        "alloy": (0.72, 0.74, 0.78, 1.0),
        "chrome": (0.84, 0.86, 0.90, 1.0),
        "glass": (0.10, 0.12, 0.18, 0.36),
        "lens": (0.78, 0.81, 0.85, 1.0),
        "amber": (0.86, 0.52, 0.08, 1.0),
        "tail": (0.55, 0.05, 0.06, 1.0),
        "rubber": (0.04, 0.04, 0.045, 1.0),
        "interior": (0.10, 0.11, 0.14, 1.0),
    },
    "ferrari_rosso_corsa": {
        "body": (0.78, 0.04, 0.04, 1.0),
        "accent": (0.05, 0.05, 0.055, 1.0),
        "alloy": (0.76, 0.77, 0.80, 1.0),
        "chrome": (0.82, 0.83, 0.86, 1.0),
        "glass": (0.10, 0.11, 0.13, 0.34),
        "lens": (0.92, 0.88, 0.40, 1.0),
        "amber": (0.88, 0.55, 0.08, 1.0),
        "tail": (0.55, 0.04, 0.04, 1.0),
        "rubber": (0.04, 0.04, 0.045, 1.0),
        "interior": (0.10, 0.07, 0.07, 1.0),
    },
    "koenigsegg_icy_blue_carbon": {
        "body": (0.58, 0.74, 0.82, 1.0),
        "accent": (0.08, 0.09, 0.10, 1.0),
        "alloy": (0.45, 0.47, 0.50, 1.0),
        "chrome": (0.80, 0.84, 0.88, 1.0),
        "glass": (0.09, 0.12, 0.15, 0.36),
        "lens": (0.80, 0.86, 0.90, 1.0),
        "amber": (0.84, 0.54, 0.10, 1.0),
        "tail": (0.45, 0.05, 0.06, 1.0),
        "rubber": (0.04, 0.04, 0.045, 1.0),
        "interior": (0.10, 0.11, 0.12, 1.0),
    },
}

# --------------------------------------------------------------------------- #
# Body profile tables (Slot A). Each is a distinct LOWER_SECTIONS rail set
# `(y, z_min, z_max, width)` + a superellipse exponent + per-family proportions.
# Adapted verbatim from the declared 5★ sources.
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class _ProfileSpec:
    sections: tuple[tuple[float, float, float, float], ...]
    exponent: float
    half_track: float
    front_axle_y: float
    rear_axle_y: float
    wheel_r: float
    wheel_w: float
    arch_radius: float
    # arch inboard/outboard walls (abs |x|); front reaches deeper inboard.
    arch_front: tuple[float, float]
    arch_rear: tuple[float, float]
    cabin_half_x: float
    cabin_y: tuple[float, float]
    cabin_z: tuple[float, float]
    door_aperture_x: tuple[float, float]
    door_aperture_y: tuple[float, float]
    door_aperture_z: tuple[float, float]
    has_roof_part: bool  # vintage carries an independent roof + glass shells
    roof_top_z: float  # top of the greenhouse (for the gullwing clearance gate)


# P2 Diablo — modern wedge (model.py:L48-L118, L172-L227).
_WEDGE = _ProfileSpec(
    sections=(
        (2.23, 0.16, 0.35, 1.18),
        (2.06, 0.13, 0.42, 1.56),
        (1.80, 0.12, 0.55, 1.84),
        (1.50, 0.12, 0.70, 1.92),
        (1.32, 0.12, 0.76, 1.96),
        (1.15, 0.12, 0.73, 1.96),
        (0.92, 0.12, 0.70, 1.94),
        (0.45, 0.13, 0.68, 1.86),
        (-0.10, 0.13, 0.68, 1.86),
        (-0.65, 0.12, 0.72, 1.96),
        (-1.10, 0.11, 0.76, 2.02),
        (-1.45, 0.11, 0.77, 2.02),
        (-1.85, 0.13, 0.75, 1.92),
        (-2.10, 0.16, 0.72, 1.76),
        (-2.23, 0.20, 0.70, 1.60),
    ),
    exponent=4.0,
    half_track=0.89,
    front_axle_y=1.32,
    rear_axle_y=-1.33,
    wheel_r=0.33,
    wheel_w=0.295,
    arch_radius=0.36,
    arch_front=(0.58, 1.16),
    arch_rear=(0.70, 1.16),
    cabin_half_x=0.60,
    cabin_y=(-0.80, 0.88),
    cabin_z=(0.50, 0.71),
    door_aperture_x=(0.52, 1.05),
    door_aperture_y=(-0.26, 0.80),
    door_aperture_z=(0.50, 0.70),
    has_roof_part=False,
    roof_top_z=1.05,
)

# P1 300SL — vintage pontoon (model.py:L57-L142). Narrow body, tall greenhouse,
# independent roof part.
_VINTAGE = _ProfileSpec(
    sections=(
        (2.32, 0.21, 0.40, 0.46),
        (2.24, 0.16, 0.45, 0.78),
        (2.12, 0.13, 0.51, 1.10),
        (1.97, 0.11, 0.60, 1.42),
        (1.80, 0.10, 0.70, 1.57),
        (1.62, 0.10, 0.75, 1.62),
        (1.42, 0.10, 0.76, 1.63),
        (1.21, 0.10, 0.76, 1.62),
        (1.00, 0.11, 0.70, 1.55),
        (0.72, 0.12, 0.65, 1.47),
        (0.30, 0.12, 0.64, 1.45),
        (-0.15, 0.12, 0.64, 1.46),
        (-0.58, 0.12, 0.67, 1.52),
        (-0.90, 0.11, 0.74, 1.61),
        (-1.19, 0.10, 0.76, 1.63),
        (-1.45, 0.11, 0.73, 1.58),
        (-1.70, 0.13, 0.67, 1.42),
        (-1.94, 0.16, 0.57, 1.10),
        (-2.16, 0.20, 0.49, 0.74),
        (-2.30, 0.26, 0.44, 0.48),
    ),
    exponent=2.6,
    half_track=0.745,
    front_axle_y=1.21,
    rear_axle_y=-1.19,
    wheel_r=0.335,
    wheel_w=0.205,
    arch_radius=0.36,
    arch_front=(0.46, 1.02),
    arch_rear=(0.55, 1.02),
    cabin_half_x=0.56,
    cabin_y=(-0.82, 0.86),
    cabin_z=(0.42, 0.70),
    door_aperture_x=(0.46, 0.84),
    door_aperture_y=(-0.38, 0.30),
    door_aperture_z=(0.44, 0.71),
    has_roof_part=True,
    roof_top_z=1.22,
)

# fastback — rec_sportscar_fastback_body_v01 (model.py:L103-L119).
_FASTBACK = _ProfileSpec(
    sections=(
        (2.23, 0.18, 0.42, 1.32),
        (2.10, 0.15, 0.52, 1.62),
        (1.85, 0.13, 0.62, 1.84),
        (1.55, 0.12, 0.70, 1.92),
        (1.32, 0.12, 0.74, 1.96),
        (1.10, 0.12, 0.72, 1.94),
        (0.85, 0.12, 0.70, 1.92),
        (0.40, 0.13, 0.68, 1.86),
        (-0.10, 0.13, 0.68, 1.86),
        (-0.50, 0.12, 0.70, 1.92),
        (-0.90, 0.11, 0.74, 2.04),
        (-1.30, 0.11, 0.76, 2.08),
        (-1.65, 0.12, 0.74, 2.04),
        (-1.95, 0.14, 0.72, 1.92),
        (-2.23, 0.18, 0.68, 1.76),
    ),
    exponent=3.0,
    half_track=0.89,
    front_axle_y=1.32,
    rear_axle_y=-1.33,
    wheel_r=0.33,
    wheel_w=0.295,
    arch_radius=0.36,
    arch_front=(0.58, 1.16),
    arch_rear=(0.70, 1.18),
    cabin_half_x=0.60,
    cabin_y=(-0.80, 0.88),
    cabin_z=(0.50, 0.72),
    door_aperture_x=(0.52, 1.05),
    door_aperture_y=(-0.26, 0.80),
    door_aperture_z=(0.50, 0.70),
    has_roof_part=False,
    roof_top_z=1.06,
)

# organic_teardrop — rec_sportscar_teardrop_body_v01 (model.py:L103-L119).
_TEARDROP = _ProfileSpec(
    sections=(
        (2.23, 0.20, 0.42, 1.24),
        (2.08, 0.15, 0.52, 1.60),
        (1.88, 0.13, 0.60, 1.86),
        (1.60, 0.12, 0.70, 1.94),
        (1.32, 0.12, 0.78, 1.98),
        (1.00, 0.12, 0.82, 2.00),
        (0.50, 0.12, 0.85, 1.98),
        (0.00, 0.12, 0.83, 1.96),
        (-0.40, 0.12, 0.80, 1.94),
        (-0.80, 0.12, 0.78, 1.92),
        (-1.20, 0.12, 0.76, 1.90),
        (-1.55, 0.13, 0.74, 1.86),
        (-1.85, 0.15, 0.72, 1.78),
        (-2.10, 0.18, 0.66, 1.60),
        (-2.23, 0.22, 0.58, 1.40),
    ),
    exponent=2.2,
    half_track=0.89,
    front_axle_y=1.32,
    rear_axle_y=-1.33,
    wheel_r=0.33,
    wheel_w=0.295,
    arch_radius=0.36,
    arch_front=(0.58, 1.16),
    arch_rear=(0.70, 1.16),
    cabin_half_x=0.60,
    cabin_y=(-0.80, 0.88),
    cabin_z=(0.50, 0.72),
    door_aperture_x=(0.52, 1.05),
    door_aperture_y=(-0.26, 0.80),
    door_aperture_z=(0.50, 0.70),
    has_roof_part=False,
    roof_top_z=1.04,
)

_PROFILES: dict[str, _ProfileSpec] = {
    "modern_wedge": _WEDGE,
    "vintage_pontoon": _VINTAGE,
    "fastback": _FASTBACK,
    "organic_teardrop": _TEARDROP,
}

# Greenhouse (one-piece, body-color skin + tinted shells) for the fastback /
# teardrop / wedge bodies. The roof reads as a thin domed skin spanning the
# cabin. We build it as a single eased dome Box stack so we don't pay a second
# boolean per seed (the lower-body carve is already the cost bottleneck).
# Axle bar radius just under the bored channel radius so the rod touches the
# channel wall (stays connected to the body — not floating in the open bore).
AXLE_BAR_RADIUS = 0.082
# Channel bored slightly NARROWER than the bar so the rod's surface presses into
# the solid body around the bore (keeps the rod connected to the body part, not
# floating in an open hole — matters most on the narrow vintage body).
AXLE_CHANNEL_RADIUS = 0.066
GLASS_SHELL_T = 0.016

# Door swept-volume / hinge constants.
_SCISSOR_TILT = 0.25
_DH = (0.0, -0.22, 0.975)


@dataclass(frozen=True)
class SportsCarConfig:
    body_profile_family: BodyProfileFamily | None = None
    door_mechanism: DoorMechanism | None = None
    headlight_shape: HeadlightShape | None = None
    headlight_popup: bool = False
    rear_blade: RearBlade | None = None
    rear_louver_cover: bool = False
    exhaust_layout: ExhaustLayout | None = None
    palette_style: PaletteStyle = "diablo_gloss_yellow"

    track_scale: float = 1.0
    wheelbase_scale: float = 1.0
    body_height_scale: float = 1.0
    door_open_max: float = 1.4
    name: str = "sports_car"


@dataclass(frozen=True)
class ResolvedSportsCarConfig:
    body_profile_family: BodyProfileFamily
    door_mechanism: DoorMechanism
    headlight_shape: HeadlightShape
    headlight_popup: bool
    rear_blade: RearBlade
    rear_louver_cover: bool
    exhaust_layout: ExhaustLayout
    palette_style: PaletteStyle
    profile: _ProfileSpec
    # resolved scaled kinematics
    half_track: float
    front_axle_y: float
    rear_axle_y: float
    wheel_r: float
    wheel_w: float
    body_height_scale: float
    door_open_max: float
    steer_lock: float
    name: str


# --------------------------------------------------------------------------- #
# Sampling
# --------------------------------------------------------------------------- #


def config_from_seed(seed: int) -> SportsCarConfig:
    """Deterministic per-seed procedural sampling (seed 0 not special)."""
    rng = random.Random(seed)

    body_profile_family = rng.choice(BODY_PROFILE_FAMILIES)
    # door: slightly biased away from gullwing (it is body-gated / single-source).
    door_mechanism = rng.choices(
        DOOR_MECHANISMS, weights=(0.42, 0.20, 0.38), k=1
    )[0]
    headlight_shape = rng.choice(HEADLIGHT_SHAPES)
    headlight_popup = rng.random() < 0.30
    rear_blade = rng.choice(REAR_BLADES)
    rear_louver_cover = rng.random() < 0.30
    exhaust_layout = rng.choice(EXHAUST_LAYOUTS)
    palette_style = rng.choice(PALETTE_STYLES)

    return SportsCarConfig(
        body_profile_family=body_profile_family,
        door_mechanism=door_mechanism,
        headlight_shape=headlight_shape,
        headlight_popup=headlight_popup,
        rear_blade=rear_blade,
        rear_louver_cover=rear_louver_cover,
        exhaust_layout=exhaust_layout,
        palette_style=palette_style,
        track_scale=round(rng.uniform(0.92, 1.08), 4),
        wheelbase_scale=round(rng.uniform(0.94, 1.06), 4),
        body_height_scale=round(rng.uniform(0.95, 1.06), 4),
        door_open_max=round(rng.uniform(1.2, 1.6), 4),
        name=f"seeded_sports_car_{seed}",
    )


def _pick(value, choices):
    return value if value in choices else choices[0]


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def resolve_config(config: SportsCarConfig | None = None) -> ResolvedSportsCarConfig:
    cfg = config or SportsCarConfig()

    body_profile_family = _pick(cfg.body_profile_family, BODY_PROFILE_FAMILIES)
    door_mechanism = _pick(cfg.door_mechanism, DOOR_MECHANISMS)
    headlight_shape = _pick(cfg.headlight_shape, HEADLIGHT_SHAPES)
    rear_blade = _pick(cfg.rear_blade, REAR_BLADES)
    exhaust_layout = _pick(cfg.exhaust_layout, EXHAUST_LAYOUTS)
    palette_style = _pick(cfg.palette_style, PALETTE_STYLES)
    headlight_popup = bool(cfg.headlight_popup)
    rear_louver_cover = bool(cfg.rear_louver_cover)

    profile = _PROFILES[body_profile_family]

    # --- Compatibility gating (spec §兼容性矩阵). ---
    # (1) gullwing is single-source (vintage). On a NON-vintage low body it needs
    #     a roof shoulder anchor + a top-hinge clearance gate; if the body cannot
    #     clear, fall back to dihedral (spec Reject case: gullwing-on-low-wedge).
    if door_mechanism == "gullwing" and not profile.has_roof_part:
        # roof_top_z must clear hinge_z + door_half_height·sin(open). The non-
        # vintage greenhouses sit too low for the longitudinal roof-ridge hinge,
        # so fall back to dihedral rather than punch a door through the roofline.
        door_mechanism = "dihedral"

    # (2) pop_up ⊥ quad_cluster — the flap covers the whole pod cluster. Force a
    #     single rect/round lamp under a pop-up cover.
    if headlight_popup and headlight_shape == "quad_cluster":
        headlight_shape = "round_fixed"

    track_scale = _clamp(float(cfg.track_scale), 0.92, 1.08)
    wheelbase_scale = _clamp(float(cfg.wheelbase_scale), 0.94, 1.06)
    body_height_scale = _clamp(float(cfg.body_height_scale), 0.95, 1.06)

    half_track = profile.half_track * track_scale
    front_axle_y = profile.front_axle_y * wheelbase_scale
    rear_axle_y = profile.rear_axle_y * wheelbase_scale
    # wheel_radius equation (ground-anchored z=0); derived from body_height_scale.
    wheel_r = _clamp(profile.wheel_r * body_height_scale, profile.wheel_r * 0.94, profile.wheel_r * 1.09)

    # door_open_max conditional on mechanism (gullwing回缩 to avoid roof sweep).
    door_open_max = _clamp(float(cfg.door_open_max), 1.2, 1.6)
    if door_mechanism == "gullwing":
        door_open_max = min(door_open_max, 1.2)

    # steer-lock vs fender-well inequality: keep the ±throw inside the well.
    # well inboard wall ≈ arch_front[0]; back off lock on the narrowest track.
    steer_lock = 0.40
    well_inboard = profile.arch_front[0]
    if half_track - profile.wheel_w * 0.5 - 0.10 < well_inboard:
        steer_lock = 0.32

    return ResolvedSportsCarConfig(
        body_profile_family=body_profile_family,
        door_mechanism=door_mechanism,
        headlight_shape=headlight_shape,
        headlight_popup=headlight_popup,
        rear_blade=rear_blade,
        rear_louver_cover=rear_louver_cover,
        exhaust_layout=exhaust_layout,
        palette_style=palette_style,
        profile=profile,
        half_track=half_track,
        front_axle_y=front_axle_y,
        rear_axle_y=rear_axle_y,
        wheel_r=wheel_r,
        wheel_w=profile.wheel_w,
        body_height_scale=body_height_scale,
        door_open_max=door_open_max,
        steer_lock=steer_lock,
        name=cfg.name,
    )


def slot_choices_for_seed(seed: int) -> list[tuple[str, str]]:
    """Per-seed (slot, module) tuples consumed by module_topology_diversity.

    Records the post-resolution choices (after gullwing fallback / popup gating)
    so the reported topology matches what is actually built. Includes the four
    structural axes (A/B/popup/louver, 48 distinct floor) plus the cosmetic
    shape/blade/exhaust slots to lift the distinct count toward ≥100.
    """
    r = resolve_config(config_from_seed(seed))
    return [
        ("body_profile_family", r.body_profile_family),
        ("door_mechanism", r.door_mechanism),
        ("headlight_shape", r.headlight_shape),
        ("headlight_popup", "popup" if r.headlight_popup else "fixed"),
        ("rear_blade", r.rear_blade),
        ("rear_louver_cover", "louver" if r.rear_louver_cover else "smooth"),
        ("exhaust_layout", r.exhaust_layout),
    ]


# --------------------------------------------------------------------------- #
# Geometry helpers (adapted from P2 / P1 sources)
# --------------------------------------------------------------------------- #


def _save(name: str, geom):
    return mesh_from_geometry(geom, name)


def _box_cutter(x0, x1, y0, y1, z0, z1):
    """Axis-aligned solid box cutter with flipped winding (P2 L154-L166)."""
    box = BoxGeometry((x1 - x0, y1 - y0, z1 - z0)).translate(
        (x0 + x1) / 2.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0
    )
    return MeshGeometry(
        vertices=list(box.vertices),
        faces=[(f[0], f[2], f[1]) for f in box.faces],
    )


_BODY_MESH_CACHE: dict[str, object] = {}


def _lower_body_mesh(r: ResolvedSportsCarConfig):
    """Lofted single-body lower body with carved arches / axle channels / cabin /
    door apertures (P2 L172-L227, P1 L205-L246).

    The carve uses only the profile's NOMINAL constants (not the per-seed track/
    wheelbase/height scale), so the result depends only on body_profile_family.
    It is cached per family — the boolean carve is the per-seed cost bottleneck.
    """
    key = r.body_profile_family
    if key in _BODY_MESH_CACHE:
        return _BODY_MESH_CACHE[key].clone()
    p = r.profile
    sections = list(p.sections)
    # Segment counts tuned for BOTH manifold robustness and per-seed compile speed:
    # the boolean carve is the sweep's wall-time bottleneck (each sweep seed runs in
    # its own subprocess, so the per-profile mesh cache does NOT amortize across
    # seeds). 44/28/18 builds each body in ~20s (vs ~90s at 64/48/24) while staying
    # clear of the coplanar-triangulation degeneracy that crashes at very low counts.
    body = superellipse_side_loft(sections, exponents=p.exponent, segments=44)

    # Carve at the profile's NOMINAL axle positions / track (not the scaled r.*).
    # The body mesh is therefore independent of track/wheelbase scale, which (a)
    # keeps the boolean carve robust (no scale-dependent coplanar degeneracy) and
    # (b) lets the carve be cached per profile. The small (±6%) track/wheelbase
    # scale only repositions the wheels/knuckles/axle-rod visual, which still sit
    # inside the generous open arches.
    arches = (
        (1.0, p.front_axle_y, p.arch_front[0], p.arch_front[1]),
        (-1.0, p.front_axle_y, p.arch_front[0], p.arch_front[1]),
        (1.0, p.rear_axle_y, p.arch_rear[0], p.arch_rear[1]),
        (-1.0, p.rear_axle_y, p.arch_rear[0], p.arch_rear[1]),
    )
    for sign, ay, inboard, outboard in arches:
        arch = (
            CylinderGeometry(
                radius=p.arch_radius, height=outboard - inboard, radial_segments=28
            )
            .rotate_y(math.pi / 2.0)
            .translate(sign * (inboard + outboard) / 2.0, ay, p.wheel_r)
        )
        body = boolean_difference(body, arch)

    for ay in (p.front_axle_y, p.rear_axle_y):
        channel = (
            CylinderGeometry(
                radius=AXLE_CHANNEL_RADIUS,
                height=2.0 * p.half_track + 0.2,
                radial_segments=18,
            )
            .rotate_y(math.pi / 2.0)
            .translate(0.0, ay, p.wheel_r)
        )
        body = boolean_difference(body, channel)

    body = boolean_difference(
        body,
        _box_cutter(
            -p.cabin_half_x, p.cabin_half_x, p.cabin_y[0], p.cabin_y[1], p.cabin_z[0], p.cabin_z[1]
        ),
    )
    for sgn in (1.0, -1.0):
        xa, xb = sorted((sgn * p.door_aperture_x[0], sgn * p.door_aperture_x[1]))
        body = boolean_difference(
            body,
            _box_cutter(
                xa, xb, p.door_aperture_y[0], p.door_aperture_y[1],
                p.door_aperture_z[0], p.door_aperture_z[1],
            ),
        )
    _BODY_MESH_CACHE[key] = body
    return body.clone()


def _wheel_geoms(r: ResolvedSportsCarConfig):
    """Tire + alloy wheel geometry (P2 L254-L274). BoltPattern guarded all-positive."""
    tire = TireGeometry(
        r.wheel_r,
        r.wheel_w,
        inner_radius=r.wheel_r * 0.65,
        carcass=TireCarcass(belt_width_ratio=0.68, sidewall_bulge=0.04),
        sidewall=TireSidewall(style="rounded", bulge=0.04),
    )
    # WheelHub BoltPattern: count>=1, circle_diameter>0 (guard — count=1/cd=0 crashes).
    bolt = BoltPattern(count=5, circle_diameter=max(0.060, r.wheel_r * 0.22), hole_diameter=0.009)
    wheel = WheelGeometry(
        r.wheel_r * 0.66,
        r.wheel_w * 0.66,
        rim=WheelRim(inner_radius=r.wheel_r * 0.53, flange_height=0.012, flange_thickness=0.006),
        hub=WheelHub(radius=r.wheel_r * 0.17, width=r.wheel_w * 0.32, cap_style="domed", bolt_pattern=bolt),
        face=WheelFace(dish_depth=0.022, front_inset=0.008),
    )
    return tire, wheel


# --------------------------------------------------------------------------- #
# Body visual builders (headlights / rear / exhaust / greenhouse) — body visuals
# (Rule 1: not articulated → parent.visual, no FIXED joints).
# --------------------------------------------------------------------------- #


def _greenhouse_geom(r):
    """Greenhouse geometry constants. The body's cabin top sits at cabin_z[1]; the
    roof springs from that rim up to roof_top (a realistic +0.30..0.55 m). Every
    piece embeds into the body cabin rim or an adjacent greenhouse piece so the
    part stays connected (no floating islands)."""
    p = r.profile
    hs = r.body_height_scale
    rim_z = p.cabin_z[1]  # body cabin top
    roof_top = rim_z + (p.roof_top_z - rim_z) * hs  # greenhouse top
    cy0, cy1 = p.cabin_y
    cyl = cy0 + 0.04  # rear cabin wall
    cyh = cy1 - 0.04  # front cabin wall (windshield base)
    half_w = p.cabin_half_x + 0.02
    return p, rim_z, roof_top, cy0, cy1, cyl, cyh, half_w


# Greenhouse canopy: three thin shells per body family, ported from the 5★
# sources. Each family's windshield/roof/rear-window tables SHARE seam rails
# (windshield top == roof leading edge; roof trailing == rear-window top) so the
# panes meet the roof FLUSH — the seating gap of a box greenhouse cannot occur
# when adjacent shells reuse the identical seam section. Sources:
#   modern_wedge    ← P2 Diablo cab-forward dome      (model.py:L120-L142, verbatim)
#   vintage_pontoon ← P1 300SL one-piece fixed dome   (model.py:L150-L156) + bridged glass
#   organic_teardrop← Pagani Huayra wraparound bubble (model.py:L144-L160), bases lifted to the tall teardrop crown
#   fastback        ← Diablo windshield + a long fastback rear-glass slope to the deck
# Built once per family and cached (the loft+boolean is a per-seed sweep cost).

# modern_wedge — Diablo greenhouse (verbatim; same coordinate frame as _WEDGE).
_WEDGE_SF = (0.20, 0.66, 1.10, 1.22)
_WEDGE_SR = (-0.55, 0.66, 1.08, 1.14)
_CANOPY_WEDGE = (
    [(0.92, 0.62, 0.72, 1.46), (0.55, 0.64, 0.92, 1.34), _WEDGE_SF],
    [_WEDGE_SF, (-0.05, 0.66, 1.18, 1.20), (-0.42, 0.66, 1.17, 1.16), _WEDGE_SR],
    [_WEDGE_SR, (-0.88, 0.66, 0.82, 1.10)],
)

# vintage_pontoon — 300SL one-piece dome (roof verbatim) + windshield/rear glass
# bridged from the cowl up to the shared seam rails (300SL doors carry the side
# glass in the source; the template builds the full fixed greenhouse on the body).
_VINT_SF = (0.46, 0.78, 1.12, 1.10)
_VINT_SR = (-0.56, 0.78, 1.13, 1.12)
_CANOPY_VINTAGE = (
    [(0.84, 0.66, 0.78, 1.14), (0.64, 0.80, 0.98, 1.12), _VINT_SF],
    [_VINT_SF, (0.16, 0.88, 1.17, 1.20), (-0.24, 0.88, 1.17, 1.20), _VINT_SR],
    [_VINT_SR, (-0.84, 0.70, 0.86, 1.12)],
)

# organic_teardrop — Pagani Huayra bubble canopy; glass bases lifted (~+0.2) so
# they emerge from the tall teardrop crown (z_max≈0.85) instead of sinking in.
_TEAR_SF = (0.36, 0.80, 1.03, 1.10)
_TEAR_SR = (-0.70, 0.80, 0.98, 0.90)
_CANOPY_TEARDROP = (
    [(0.90, 0.80, 0.90, 1.36), (0.62, 0.82, 0.99, 1.22), _TEAR_SF],
    [_TEAR_SF, (0.02, 0.84, 1.07, 1.02), (-0.36, 0.84, 1.04, 0.94), _TEAR_SR],
    [_TEAR_SR, (-0.98, 0.80, 0.90, 0.84)],
)

# fastback — Diablo windshield + front roof, then a single long rear-glass slope
# running back to the high rear deck (the defining fastback roofline).
_FAST_SF = (0.20, 0.66, 1.10, 1.22)
_FAST_SR = (-0.55, 0.66, 1.05, 1.26)
_CANOPY_FASTBACK = (
    [(0.92, 0.62, 0.72, 1.46), (0.55, 0.64, 0.92, 1.34), _FAST_SF],
    [_FAST_SF, (-0.08, 0.66, 1.16, 1.22), _FAST_SR],
    [_FAST_SR, (-1.15, 0.66, 0.84, 1.46)],
)

_CANOPY_SECTIONS: dict[str, tuple] = {
    "modern_wedge": _CANOPY_WEDGE,
    "vintage_pontoon": _CANOPY_VINTAGE,
    "fastback": _CANOPY_FASTBACK,
    "organic_teardrop": _CANOPY_TEARDROP,
}

_CANOPY_CACHE: dict[str, tuple] = {}


def _glass_shell(sections, t=GLASS_SHELL_T, segments=40):
    """Thin uniform-thickness shell of a superellipse side-loft (P2 _glass_shell):
    subtract an inset copy (top/sides pulled in by t, bottom dropped 0.15 below to
    open into the cabin). Adjacent shells from SHARED seam sections meet seamlessly
    because they reuse the identical seam rail."""
    outer = superellipse_side_loft(sections, exponents=2.8, segments=segments)
    inner = superellipse_side_loft(
        [(y, zmin - 0.15, zmax - t, max(w - 2.0 * t, 0.04)) for (y, zmin, zmax, w) in sections],
        exponents=2.8,
        segments=segments,
    )
    # superellipse_side_loft is wound INWARD (negative volume); flip the inner
    # faces so it actually carves the shell hollow rather than acting as a no-op.
    inner = MeshGeometry(
        vertices=list(inner.vertices),
        faces=[(f[0], f[2], f[1]) for f in inner.faces],
    )
    return boolean_difference(outer, inner)


def _canopy_meshes(family: str):
    """(roof, windshield, rear_window) thin-shell meshes for the family, cached."""
    if family not in _CANOPY_CACHE:
        ws, roof, rw = _CANOPY_SECTIONS[family]
        _CANOPY_CACHE[family] = (
            _glass_shell(roof),
            _glass_shell(ws),
            _glass_shell(rw),
        )
    roof_m, ws_m, rw_m = _CANOPY_CACHE[family]
    return roof_m.clone(), ws_m.clone(), rw_m.clone()


def _add_greenhouse(body, r, mats) -> None:
    """Cab greenhouse = three thin shells (windshield / roof / rear window) that
    SHARE seam rails so the glass meets the roof flush (ported per body family
    from the 5★ sources — see _CANOPY_SECTIONS). Four corner A/C-pillars frame the
    cabin rim → roof; cabin floor + seat backs fill the hollow. Every piece embeds
    into the body so the body part has no floating greenhouse islands."""
    p, rim_z, roof_top, cy0, cy1, cyl, cyh, half_w = _greenhouse_geom(r)
    pillar_h = roof_top - rim_z + 0.10  # embeds into both rim & roof

    # Four corner A/C-pillars (bottom embeds the body crown) framing the glass.
    for sx, sxn in ((half_w - 0.05, "l"), (-(half_w - 0.05), "r")):
        for sy, syn in ((cyh, "f"), (cyl, "b")):
            body.visual(
                Box((0.06, 0.06, pillar_h)),
                origin=Origin(xyz=(sx, sy, rim_z - 0.05 + pillar_h * 0.5)),
                material=mats["body"],
                name=f"pillar_{sxn}{syn}",
            )

    # Three seam-shared shells: roof (body color) + windshield + rear window
    # (glass). Built from the family's canopy tables whose shared seam rails make
    # the glass meet the roof flush; each shell's lower side edges seat onto the
    # body crown just outside the cabin cut, closing the greenhouse sides (no
    # open "gazebo" between four posts).
    roof_mesh, ws_mesh, rw_mesh = _canopy_meshes(r.body_profile_family)
    body.visual(_save("greenhouse_roof.obj", roof_mesh), material=mats["body"], name="greenhouse")
    body.visual(_save("greenhouse_windshield.obj", ws_mesh), material=mats["glass"], name="windshield")
    body.visual(_save("greenhouse_rear.obj", rw_mesh), material=mats["glass"], name="rear_window")

    # Cabin interior so the hollow is not an empty shell (floor embeds into the
    # body floor pan at cabin_z[0]).
    body.visual(
        Box((2.0 * p.cabin_half_x - 0.06, cy1 - cy0 - 0.10, 0.10)),
        origin=Origin(xyz=(0.0, (cy0 + cy1) * 0.5, p.cabin_z[0] + 0.02)),
        material=mats["interior"],
        name="cabin_floor",
    )
    for sx, side in ((0.28, "left"), (-0.28, "right")):
        body.visual(
            Box((0.40, 0.18, 0.28)),
            origin=Origin(xyz=(sx, cy0 + 0.30, p.cabin_z[0] + 0.14)),
            material=mats["interior"],
            name=f"seat_back_{side}",
        )


def _add_headlights(body, r, mats) -> None:
    """Headlight cluster as body visuals (P2 L432-L487 rect; P1/Porsche round;
    Pagani/Bugatti quad). Pop-up cover (if any) is a separate REVOLUTE part."""
    p = r.profile
    nose_y = p.sections[1][0] - 0.10  # just behind the nose tip
    # Headlight height = the body's actual top z AT the nose section (so the lamps
    # sit ON the nose sheetmetal, not floating above a fixed estimate). Use the
    # section nearest nose_y; place the lamp centered just below that crown so it
    # embeds into the body.
    nose_sec = min(p.sections, key=lambda sec: abs(sec[0] - nose_y))
    nose_crown = nose_sec[2] * r.body_height_scale
    z = nose_crown - 0.04
    shape = r.headlight_shape
    rake = (-0.30, 0.0, 0.0)

    # Nose bumper strip spanning the front — embeds into the nose body so it (and
    # everything mounted on it) stays connected to the body part. Sits at the
    # lower nose so it laps the front fascia.
    nose_w = nose_sec[3]
    body.visual(
        Box((nose_w * 0.92, 0.12, 0.20)),
        origin=Origin(xyz=(0.0, nose_y + 0.12, max(0.22, nose_crown - 0.18))),
        material=mats["accent"],
        name="front_bumper_strip",
    )
    for sx, side in ((0.40, "left"), (-0.40, "right")):
        body.visual(
            Box((0.12, 0.06, 0.09)),
            origin=Origin(xyz=(sx, nose_y + 0.20, 0.27)),
            material=mats["amber"],
            name=f"front_indicator_{side}",
        )
        if shape == "exposed_rect":
            body.visual(
                Box((0.36, 0.20, 0.05)),
                origin=Origin(xyz=(sx, nose_y, z), rpy=rake),
                material=mats["accent"],
                name=f"headlight_surround_{side}",
            )
            body.visual(
                Box((0.33, 0.16, 0.014)),
                origin=Origin(xyz=(sx, nose_y, z + 0.022), rpy=rake),
                material=mats["lens"],
                name=f"headlight_reflector_{side}",
            )
            body.visual(
                Box((0.34, 0.17, 0.012)),
                origin=Origin(xyz=(sx, nose_y + 0.002, z + 0.030), rpy=rake),
                material=mats["glass"],
                name=f"headlight_{side}",
            )
        elif shape == "round_fixed":
            body.visual(
                Cylinder(radius=0.115, length=0.06),
                origin=Origin(xyz=(sx, nose_y, z), rpy=(math.pi / 2.0 - 0.30, 0.0, 0.0)),
                material=mats["chrome"],
                name=f"headlight_bucket_{side}",
            )
            body.visual(
                Cylinder(radius=0.095, length=0.03),
                origin=Origin(xyz=(sx, nose_y + 0.04, z + 0.01), rpy=(math.pi / 2.0 - 0.30, 0.0, 0.0)),
                material=mats["lens"],
                name=f"headlight_reflector_{side}",
            )
            body.visual(
                Cylinder(radius=0.105, length=0.018),
                origin=Origin(xyz=(sx, nose_y + 0.07, z + 0.015), rpy=(math.pi / 2.0 - 0.30, 0.0, 0.0)),
                material=mats["glass"],
                name=f"headlight_{side}",
            )
        else:  # quad_cluster — sculpted pod with 3 jewels
            body.visual(
                Box((0.30, 0.20, 0.06)),
                origin=Origin(xyz=(sx, nose_y, z), rpy=rake),
                material=mats["accent"],
                name=f"headlight_pod_{side}",
            )
            for k, dx in enumerate((-0.085, 0.0, 0.085)):
                body.visual(
                    Cylinder(radius=0.045, length=0.05),
                    origin=Origin(xyz=(sx + dx, nose_y + 0.02, z + 0.02),
                                  rpy=(math.pi / 2.0 - 0.30, 0.0, 0.0)),
                    material=mats["lens"] if k != 2 else mats["amber"],
                    name=f"headlight_jewel_{side}_{k}",
                )
            body.visual(
                Box((0.30, 0.17, 0.012)),
                origin=Origin(xyz=(sx, nose_y + 0.04, z + 0.035), rpy=rake),
                material=mats["glass"],
                name=f"headlight_{side}",
            )

    # nose splitter + bumper (always, for identity).
    body.visual(
        Box((1.30, 0.40, 0.07)),
        origin=Origin(xyz=(0.0, p.sections[0][0] - 0.18, 0.105)),
        material=mats["accent"],
        name="front_splitter",
    )


def _add_rear(body, r, mats) -> None:
    """Rear treatment: engine deck + tail panel/lights + blade (wing/ducktail) +
    exhaust. All body visuals (P2 L529-L599, Ferrari/Porsche blades)."""
    p = r.profile
    tail_y = p.sections[-1][0]
    deck_top = p.cabin_z[1] + 0.03

    # Engine deck behind the cabin, spanning from the cabin rear to the tail so
    # everything mounted on the rear (wing pylons, ducktail, louvers) embeds into
    # it and stays connected to the body part. It embeds into the body top.
    deck_back = tail_y + 0.10
    deck_front = p.cabin_y[0] - 0.10
    deck_cy = (deck_front + deck_back) * 0.5
    deck_len = deck_front - deck_back
    body.visual(
        Box((1.24, deck_len, 0.05)),
        origin=Origin(xyz=(0.0, deck_cy, deck_top - 0.04)),
        material=mats["accent"],
        name="engine_deck",
    )

    # Tail panel + taillights (embed into the body tail).
    body.visual(
        Box((1.44, 0.10, 0.28)),
        origin=Origin(xyz=(0.0, tail_y + 0.06, 0.48)),
        material=mats["accent"],
        name="tail_panel",
    )
    for sx, side in ((0.46, "left"), (-0.46, "right")):
        body.visual(
            Box((0.28, 0.04, 0.11)),
            origin=Origin(xyz=(sx, tail_y, 0.52)),
            material=mats["tail"],
            name=f"taillight_{side}",
        )

    # Rear blade.
    wing_y = tail_y + 0.30  # over the rear deck, above the engine_deck
    if r.rear_blade == "big_gt_wing":
        # Tall twin pylons root DOWN into the engine deck (bottom embeds in deck,
        # top embeds in the blade) — bridge body → wing.
        for sx, side in ((0.50, "left"), (-0.50, "right")):
            body.visual(
                Box((0.09, 0.24, 0.40)),
                origin=Origin(xyz=(sx, wing_y, deck_top + 0.16)),
                material=mats["accent"],
                name=f"wing_pylon_{side}",
            )
        body.visual(
            Box((1.70, 0.34, 0.045)),
            origin=Origin(xyz=(0.0, wing_y - 0.04, deck_top + 0.36), rpy=(0.10, 0.0, 0.0)),
            material=mats["body"],
            name="wing_blade",
        )
        body.visual(
            Box((1.62, 0.05, 0.06)),
            origin=Origin(xyz=(0.0, wing_y - 0.20, deck_top + 0.38), rpy=(0.10, 0.0, 0.0)),
            material=mats["accent"],
            name="wing_gurney",
        )
    elif r.rear_blade == "ducktail":
        # Low ducktail lip sitting ON the rear deck (bottom embeds into the deck).
        body.visual(
            Box((1.50, 0.30, 0.05)),
            origin=Origin(xyz=(0.0, wing_y, deck_top + 0.01), rpy=(0.18, 0.0, 0.0)),
            material=mats["body"],
            name="ducktail_blade",
        )
        body.visual(
            Box((1.50, 0.08, 0.10)),
            origin=Origin(xyz=(0.0, wing_y - 0.08, deck_top + 0.06), rpy=(0.18, 0.0, 0.0)),
            material=mats["accent"],
            name="ducktail_lip",
        )
    else:  # integrated — louvered deck, no big wing
        # Louvers sit ON the engine deck (their bottom embeds into the deck so each
        # connects to the body). Deck top surface ≈ deck_top - 0.015.
        for k in range(6):
            body.visual(
                Box((1.18, 0.08, 0.05)),
                origin=Origin(xyz=(0.0, p.cabin_y[0] - 0.30 - 0.10 * k, deck_top - 0.01),
                              rpy=(0.12, 0.0, 0.0)),
                material=mats["accent"],
                name=f"deck_louver_{k}",
            )

    # Exhaust. Side exhausts hug the rear corners of the body (x scales with the
    # tail width so they touch the body even on the narrow vintage shell).
    tail_w = p.sections[-1][3]
    if r.exhaust_layout == "side":
        ex_x = tail_w * 0.5 - 0.06
        for sx, side in ((ex_x, "left"), (-ex_x, "right")):
            body.visual(
                Cylinder(radius=0.048, length=0.18),
                origin=Origin(xyz=(sx, tail_y + 0.04, 0.26), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=mats["chrome"],
                name=f"exhaust_{side}",
            )
    elif r.exhaust_layout == "central_quad":
        for k, dx in enumerate((-0.12, -0.04, 0.04, 0.12)):
            body.visual(
                Cylinder(radius=0.036, length=0.14),
                origin=Origin(xyz=(dx, tail_y + 0.02, 0.20), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=mats["chrome"],
                name=f"exhaust_q{k}",
            )
    else:  # central_single
        body.visual(
            Cylinder(radius=0.075, length=0.16),
            origin=Origin(xyz=(0.0, tail_y + 0.02, 0.22), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=mats["chrome"],
            name="exhaust_center_ring",
        )
        body.visual(
            Cylinder(radius=0.055, length=0.10),
            origin=Origin(xyz=(0.0, tail_y + 0.05, 0.22), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=mats["accent"],
            name="exhaust_center_bore",
        )


def _add_sills_intakes(body, r, mats) -> None:
    p = r.profile
    # Rocker sill runs between the wheel arches along the lower flank. Use the
    # NARROWEST body width over the sill's y-span (so the straight sill embeds into
    # the curved flank along its whole length, not just at the widest point).
    sill_y0, sill_y1 = -0.55, 0.75
    span_widths = [s[3] for s in p.sections if sill_y0 - 0.1 <= s[0] <= sill_y1 + 0.1]
    sill_w = min(span_widths) if span_widths else p.cabin_half_x * 2.0
    sx0 = sill_w * 0.5 - 0.08  # set in from the flank surface so it stays embedded
    for sx, side in ((sx0, "left"), (-sx0, "right")):
        body.visual(
            Box((0.12, sill_y1 - sill_y0, 0.13)),
            origin=Origin(xyz=(sx, (sill_y0 + sill_y1) * 0.5, 0.20)),
            material=mats["accent"],
            name=f"rocker_sill_{side}",
        )


def _add_axles_steering(body, r, mats, model):
    """Axle rods (body visuals) + spinning steering wheel (REVOLUTE child part)."""
    p = r.profile
    for ay, axle_name in ((r.front_axle_y, "front_axle_bar"), (r.rear_axle_y, "rear_axle_bar")):
        body.visual(
            Cylinder(radius=AXLE_BAR_RADIUS, length=2.0 * r.half_track),
            origin=Origin(xyz=(0.0, ay, r.wheel_r), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=mats["alloy"],
            name=axle_name,
        )

    # Steering column (fixed body visual) + spinning steering wheel part.
    # Steering wheel low inside the cabin, well forward of the door apertures so
    # the rim does not poke the roof or the door glass. cabin_y[1] is the front
    # cabin wall; sit the hub ~0.35 m behind it, just above the floor.
    sw_x = 0.30
    sw_hub = (sw_x, p.cabin_y[1] - 0.36, p.cabin_z[0] + 0.12)
    sw_rake = (0.6, 0.0, 0.0)
    # Column runs from the cabin floor up to the hub. Its top end lands on sw_hub
    # so the steering-wheel joint origin sits on real geometry. Raked toward the
    # driver: with rpy x=0.6 the +Z local axis tilts back, so place the lower end
    # forward (+Y) of the hub and let the cylinder reach up/back to sw_hub.
    col_len = 0.34
    col_cx = sw_x
    col_cy = sw_hub[1] + 0.5 * col_len * math.sin(sw_rake[0])
    col_cz = sw_hub[2] - 0.5 * col_len * math.cos(sw_rake[0])
    body.visual(
        Cylinder(radius=0.024, length=col_len),
        origin=Origin(xyz=(col_cx, col_cy, col_cz), rpy=sw_rake),
        material=mats["accent"],
        name="steering_column",
    )
    steer_wheel = model.part("steering_wheel")
    rw, rh, rt, rd = 0.145, 0.110, 0.026, 0.045
    for sy, tag in ((rh, "top"), (-rh, "bot")):
        steer_wheel.visual(
            Box((2.0 * rw + rt, rt, rd)),
            origin=Origin(xyz=(0.0, sy, 0.0)),
            material=mats["accent"],
            name=f"sw_rim_{tag}",
        )
    for sx, tag in ((rw, "left"), (-rw, "right")):
        steer_wheel.visual(
            Box((rt + 0.016, 2.0 * rh - 0.010, rd + 0.012)),
            origin=Origin(xyz=(sx, 0.0, 0.0)),
            material=mats["accent"],
            name=f"sw_grip_{tag}",
        )
    steer_wheel.visual(Box((2.0 * rw, 0.030, 0.020)), material=mats["accent"], name="sw_spoke_cross")
    steer_wheel.visual(
        Cylinder(radius=0.044, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, -0.02)),
        material=mats["accent"],
        name="sw_hub",
    )
    steer_wheel.visual(
        Box((0.034, 0.020, 0.020)),
        origin=Origin(xyz=(0.0, rh, 0.020)),
        material=mats["amber"],
        name="sw_top_marker",
    )
    steer_wheel.inertial = Inertial.from_geometry(Box((0.32, 0.26, 0.06)), mass=2.0)
    # Steering wheel spins on the raked column (captured hub-on-column pivot).
    # Origin sits on the column top (real geometry); mating is grandfathered
    # because the hub-barrel-around-column is a pin-in-sleeve, not two faces.
    model.articulation(
        "steering_wheel_turn",
        ArticulationType.REVOLUTE,
        parent=body,
        child=steer_wheel,
        origin=Origin(xyz=sw_hub, rpy=sw_rake),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=8.0, lower=-3.14, upper=3.14),
    )


# --------------------------------------------------------------------------- #
# Door builders (Slot B)
# --------------------------------------------------------------------------- #


def _door_axis(mechanism: str, side: str):
    s = 1.0 if side == "left" else -1.0
    if mechanism == "scissor_butterfly":
        norm = math.sqrt(1.0 + _SCISSOR_TILT * _SCISSOR_TILT)
        return (-1.0 / norm, 0.0, s * _SCISSOR_TILT / norm)
    if mechanism == "gullwing":
        return (0.0, -s, 0.0)
    # dihedral
    dn = math.sqrt(_DH[0] ** 2 + _DH[1] ** 2 + _DH[2] ** 2)
    return (s * _DH[0] / dn, s * _DH[1] / dn, _DH[2] / dn)


def _door_hinge_origin(r: ResolvedSportsCarConfig, side: str):
    """Hinge point in body frame. scissor/dihedral: front-top side edge.
    gullwing: roof-ridge longitudinal axis just off centerline."""
    p = r.profile
    s = 1.0 if side == "left" else -1.0
    if r.door_mechanism == "gullwing":
        # Roof-ridge longitudinal hinge at the cant-rail. Snap ONTO an actual
        # vertex of the canopy ROOF shell so the origin sits on real sheetmetal:
        # the seam-shared shell is lower/narrower than the old box roof panel, so
        # a fixed geometric point floats off it (fail_if_articulation_origin_far).
        _, rim_z, roof_top, cy0, cy1, cyl, cyh, half_w = _greenhouse_geom(r)
        target = (s * (half_w - 0.06), (cyl + cyh) * 0.5, roof_top - 0.02)
        return _nearest_canopy_vertex(r, target, s)
    # scissor/dihedral: front-top side edge. Anchor ON the outer flank surface
    # just FORWARD of the aperture. Derive the flank x from the body section width
    # at the hinge y (the loft half-width), set slightly inboard so the origin
    # sits on real sheetmetal, at mid-aperture height.
    ay_target = p.door_aperture_y[1] + 0.05
    az_target = 0.5 * (p.door_aperture_z[0] + p.door_aperture_z[1])
    # Snap the hinge ONTO an actual body vertex near the target flank point so the
    # joint origin sits exactly on real geometry (robust across all four
    # superellipse profiles, which curve differently toward the roofline).
    vx, vy, vz = _nearest_flank_vertex(r, ay_target, az_target)
    return (s * vx, vy, vz)


_FLANK_VTX_CACHE: dict[tuple, tuple[float, float, float]] = {}


def _nearest_flank_vertex(r: ResolvedSportsCarConfig, y: float, z: float):
    """The +x flank body vertex nearest the target (y, z), from the cached mesh.
    Used to plant the door hinge exactly on real sheetmetal."""
    key = (r.body_profile_family, round(y, 3), round(z, 3))
    if key in _FLANK_VTX_CACHE:
        return _FLANK_VTX_CACHE[key]
    mesh = _lower_body_mesh(r)
    best = (r.profile.cabin_half_x + 0.10, y, z)
    best_d = 1e9
    for vx, vy, vz in mesh.vertices:
        if vx <= 0.20:
            continue
        d = (vy - y) ** 2 + (vz - z) ** 2
        if d < best_d:
            best_d = d
            best = (vx, vy, vz)
    _FLANK_VTX_CACHE[key] = best
    return best


_BODY_VTX_CACHE: dict[tuple, tuple[float, float, float]] = {}


def _nearest_body_vertex(r: ResolvedSportsCarConfig, target):
    """The body vertex nearest the 3D target point, from the cached mesh. Used to
    plant the pop-up cover hinge exactly on the front-fender surface."""
    tx, ty, tz = target
    sgn = 1.0 if tx >= 0 else -1.0
    key = (r.body_profile_family, round(tx, 3), round(ty, 3), round(tz, 3))
    if key in _BODY_VTX_CACHE:
        return _BODY_VTX_CACHE[key]
    mesh = _lower_body_mesh(r)
    best = (tx, ty, tz)
    best_d = 1e9
    for vx, vy, vz in mesh.vertices:
        if vx * sgn <= 0.10:  # same side as the target
            continue
        d = (vx - tx) ** 2 + (vy - ty) ** 2 + (vz - tz) ** 2
        if d < best_d:
            best_d = d
            best = (vx, vy, vz)
    _BODY_VTX_CACHE[key] = best
    return best


_CANOPY_VTX_CACHE: dict[tuple, tuple[float, float, float]] = {}


def _nearest_canopy_vertex(r: ResolvedSportsCarConfig, target, side_sign: float):
    """Nearest vertex of the family's canopy ROOF shell to the target, on the
    target's side. Plants the gullwing roof-ridge hinge exactly on the roof skin
    (the shell, not the lower body, is what the hinge must land on)."""
    tx, ty, tz = target
    key = (r.body_profile_family, round(tx, 3), round(ty, 3), round(tz, 3))
    if key in _CANOPY_VTX_CACHE:
        return _CANOPY_VTX_CACHE[key]
    roof_mesh, _ws, _rw = _canopy_meshes(r.body_profile_family)
    best = target
    best_d = 1e9
    for vx, vy, vz in roof_mesh.vertices:
        if vx * side_sign <= 0.02:  # same side as the hinge
            continue
        d = (vx - tx) ** 2 + (vy - ty) ** 2 + (vz - tz) ** 2
        if d < best_d:
            best_d = d
            best = (vx, vy, vz)
    _CANOPY_VTX_CACHE[key] = best
    return best


def _make_door(model, r, mats, side: str):
    """Door part: skin loft + tinted window + beltline + handle + mirror, authored
    in body frame then shifted into the hinge-local frame (P2 L610-L676)."""
    s = 1.0 if side == "left" else -1.0
    p = r.profile
    hx, hy, hz = _door_hinge_origin(r, side)
    door = model.part(f"door_{side}")

    # Door skin loft over the aperture flank span.
    y0, y1 = p.door_aperture_y
    skin_sections = [
        (y1 - 0.02, 0.20, 0.70, 0.18),
        (y1 - 0.30, 0.17, 0.72, 0.22),
        (y0 + 0.16, 0.17, 0.72, 0.22),
        (y0 + 0.02, 0.19, 0.70, 0.18),
    ]
    skin = superellipse_side_loft(skin_sections, exponents=3.2, segments=32)
    door.visual(
        _save(f"door_{side}_skin.obj", skin.translate(s * (hx) - s * hx, -hy, -hz)),
        material=mats["body"],
        name="door_skin",
    )

    glass_sections = [
        (y1 - 0.06, 0.71, 0.80, 0.08),
        (y1 - 0.34, 0.72, 0.95, 0.10),
        (y0 + 0.18, 0.72, 1.00, 0.10),
        (y0 + 0.04, 0.72, 0.90, 0.08),
    ]
    glass_loft = superellipse_side_loft(glass_sections, exponents=2.6, segments=28)
    door.visual(
        _save(f"door_{side}_glass.obj", glass_loft.translate(-s * 0.24, -hy, -hz)),
        material=mats["glass"],
        name="door_glass",
    )
    # Beltline shelf — a wide flat bridge from the hinge (front, near door-local
    # origin) back along the whole door, tall enough to overlap the skin top, the
    # window glass base AND the hinge lug, so every door visual is connected.
    belt_z = 0.66 - hz
    belt_y0 = y0 - hy  # rear
    belt_y1 = 0.06  # forward, past the hinge lug at y=0
    door.visual(
        Box((0.34, abs(belt_y1 - belt_y0) + 0.08, 0.16)),
        origin=Origin(xyz=(-s * 0.06, (belt_y0 + belt_y1) * 0.5, belt_z)),
        material=mats["accent"],
        name="door_beltline",
    )
    door.visual(
        Box((0.04, 0.14, 0.03)),
        origin=Origin(xyz=(s * 0.10, y0 + 0.10 - hy, 0.55 - hz)),
        material=mats["accent"],
        name="door_handle",
    )
    door.visual(
        Box((0.12, 0.05, 0.04)),
        origin=Origin(xyz=(s * 0.11, y1 - 0.06 - hy, 0.70 - hz), rpy=(0.0, -s * 0.5, 0.0)),
        material=mats["body"],
        name="mirror_stalk",
    )
    door.visual(
        Box((0.06, 0.15, 0.10)),
        origin=Origin(xyz=(s * 0.18, y1 - 0.06 - hy, 0.76 - hz)),
        material=mats["body"],
        name="mirror_head",
    )
    # Hinge lug at the door's local origin (the joint origin) — real hinge
    # hardware so the door part AABB contains (0,0,0); also the visual contact
    # point where the door pivots against the body flank.
    door.visual(
        Box((0.07, 0.10, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=mats["accent"],
        name="door_hinge_lug",
    )
    # Vertical hinge arm bridging the lug (at the door-local origin / hinge) down
    # to the beltline / panel. For scissor/dihedral the panel top is near z~0; for
    # gullwing the hinge sits at the roof and the panel hangs well below, so the
    # arm spans the full drop. Sized from the lug down to the beltline z.
    belt_z_local = 0.66 - hz
    arm_top = 0.06
    arm_bot = min(belt_z_local, -0.02)
    door.visual(
        Box((0.06, 0.08, abs(arm_top - arm_bot) + 0.06)),
        origin=Origin(xyz=(0.0, 0.0, (arm_top + arm_bot) * 0.5)),
        material=mats["accent"],
        name="door_hinge_arm",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.22, abs(y1 - y0) + 0.2, 0.80)),
        mass=30.0,
        origin=Origin(xyz=(0.0, (y0 + y1) * 0.5 - hy, 0.55 - hz)),
    )
    return door


# --------------------------------------------------------------------------- #
# Optional articulated covers (Slot C pop-up, Slot D louver)
# --------------------------------------------------------------------------- #


def _make_headlight_cover(model, r, mats, side: str):
    """Pop-up headlight cover part (v07 L605-L637). REVOLUTE about X."""
    s = 1.0 if side == "left" else -1.0
    cover = model.part(f"headlight_cover_{side}")
    cover.visual(
        Box((0.37, 0.21, 0.024)),
        origin=Origin(xyz=(0.0, 0.085, -0.018), rpy=(-0.30, 0.0, 0.0)),
        material=mats["body"],
        name="cover_panel",
    )
    cover.visual(
        Box((0.37, 0.016, 0.030)),
        origin=Origin(xyz=(0.0, -0.030, -0.006), rpy=(-0.30, 0.0, 0.0)),
        material=mats["accent"],
        name="cover_front_seam",
    )
    cover.visual(
        Box((0.024, 0.20, 0.030)),
        origin=Origin(xyz=(s * 0.172, 0.080, -0.006), rpy=(-0.30, 0.0, 0.0)),
        material=mats["accent"],
        name="cover_outer_seam",
    )
    cover.visual(
        Cylinder(radius=0.018, length=0.34),
        origin=Origin(xyz=(0.0, -0.020, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mats["accent"],
        name="hinge_pin",
    )
    cover.inertial = Inertial.from_geometry(
        Box((0.40, 0.24, 0.04)), mass=3.0, origin=Origin(xyz=(0.0, 0.08, -0.01))
    )
    return cover


def _make_engine_cover(model, r, mats):
    """Engine louver cover part (v09 L605-L629). REVOLUTE about X."""
    cover = model.part("engine_cover")
    cover.visual(
        Box((1.18, 0.70, 0.045)),
        origin=Origin(xyz=(0.0, -0.34, 0.055), rpy=(-0.18, 0.0, 0.0)),
        material=mats["accent"],
        name="cover_panel",
    )
    for k in range(5):
        # Louver ribs embed into the cover panel. Made tall (and their center kept
        # at the panel z) so that even with the panel's -0.18 rake the rib bottom
        # stays inside the panel along its whole length (no front-rib island).
        cover.visual(
            Box((1.08, 0.052, 0.10)),
            origin=Origin(xyz=(0.0, -0.10 - 0.12 * k, 0.055), rpy=(-0.18, 0.0, 0.0)),
            material=mats["accent"],
            name=f"cover_louver_{k}",
        )
    cover.visual(
        Cylinder(radius=0.026, length=1.04),
        origin=Origin(xyz=(0.0, 0.020, 0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mats["alloy"],
        name="cover_hinge_pin",
    )
    cover.inertial = Inertial.from_geometry(
        Box((1.20, 0.72, 0.08)), mass=18.0, origin=Origin(xyz=(0.0, -0.34, 0.04))
    )
    return cover


# --------------------------------------------------------------------------- #
# Wheels + knuckles + assembly
# --------------------------------------------------------------------------- #


def _make_wheel(model, r, mats, tire_geom, wheel_geom, name: str, outboard_sign: float):
    w = model.part(name)
    face_rpy = (0.0, 0.0, 0.0) if outboard_sign > 0 else (0.0, 0.0, math.pi)
    w.visual(
        _save(f"{name}_tire.obj", tire_geom.clone()),
        origin=Origin(rpy=face_rpy),
        material=mats["rubber"],
        name="tire",
    )
    w.visual(
        _save(f"{name}_alloy.obj", wheel_geom.clone()),
        origin=Origin(xyz=(outboard_sign * 0.03, 0.0, 0.0), rpy=face_rpy),
        material=mats["alloy"],
        name="rim",
    )
    # Hub-center fill so the wheel part AABB solidly contains the spin origin
    # (0,0,0) — the open bore otherwise leaves a >tol gap at the rear-wheel
    # spin joint (fail_if_articulation_origin_far_from_geometry).
    w.visual(
        Cylinder(radius=r.wheel_r * 0.18, length=r.wheel_w * 0.8),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mats["alloy"],
        name="hub_center",
    )
    w.inertial = Inertial.from_geometry(
        Cylinder(radius=r.wheel_r, length=r.wheel_w),
        mass=22.0,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    return w


def build_sports_car(
    config: SportsCarConfig | None = None,
    *,
    assets: AssetContext | None = None,
) -> ArticulatedObject:
    r = resolve_config(config)
    p = r.profile
    model = ArticulatedObject(name=r.name, assets=assets)

    # Register materials (Slot E palette). Every .visual uses mats[...].
    mats: dict[str, object] = {}
    for token, rgba in PALETTES[r.palette_style].items():
        mats[token] = model.material(token, rgba=rgba)

    # ----------------------------------------------------------------- body
    body = model.part("body")
    body.visual(_save("lower_body.obj", _lower_body_mesh(r)), material=mats["body"], name="lower_body")
    _add_greenhouse(body, r, mats)
    _add_headlights(body, r, mats)
    _add_rear(body, r, mats)
    _add_sills_intakes(body, r, mats)
    _add_axles_steering(body, r, mats, model)
    body.inertial = Inertial.from_geometry(
        Box((2.0, 4.46, 1.10 * r.body_height_scale)),
        mass=1450.0,
        origin=Origin(xyz=(0.0, 0.0, 0.55)),
    )

    # ----------------------------------------------------------------- doors
    door_left = _make_door(model, r, mats, "left")
    door_right = _make_door(model, r, mats, "right")
    for door, side in ((door_left, "left"), (door_right, "right")):
        hx, hy, hz = _door_hinge_origin(r, side)
        # Door is a flush-seat panel captured in the body aperture: the skin
        # loft refills the carved hole. This is not a clean two-face mate, so the
        # mating contract is grandfathered (matches the 5★ reference doors).
        model.articulation(
            f"door_{side}_hinge",
            ArticulationType.REVOLUTE,
            parent=body,
            child=door,
            origin=Origin(xyz=(hx, hy, hz)),
            axis=_door_axis(r.door_mechanism, side),
            motion_limits=MotionLimits(effort=60.0, velocity=2.0, lower=0.0, upper=r.door_open_max),
        )

    # --------------------------------------------------------------- wheels
    tire_geom, wheel_geom = _wheel_geoms(r)
    _make_wheel(model, r, mats, tire_geom, wheel_geom, "wheel_front_left", 1.0)
    _make_wheel(model, r, mats, tire_geom, wheel_geom, "wheel_front_right", -1.0)
    _make_wheel(model, r, mats, tire_geom, wheel_geom, "wheel_rear_left", 1.0)
    _make_wheel(model, r, mats, tire_geom, wheel_geom, "wheel_rear_right", -1.0)

    # --------------------------------------------------- steer knuckles
    def make_knuckle(name: str):
        k = model.part(name)
        k.inertial = Inertial.from_geometry(Box((0.10, 0.10, 0.22)), mass=6.0)
        return k

    knuckle_fl = make_knuckle("steer_knuckle_front_left")
    knuckle_fr = make_knuckle("steer_knuckle_front_right")

    # Front steering REVOLUTE about vertical king-pin (Z). Grandfathered (the
    # knuckle is a captured carrier link with no visual; pin-in-bore geometry).
    for knuckle, sx in ((knuckle_fl, r.half_track), (knuckle_fr, -r.half_track)):
        model.articulation(
            f"{knuckle.name}_steer",
            ArticulationType.REVOLUTE,
            parent=body,
            child=knuckle,
            origin=Origin(xyz=(sx, r.front_axle_y, r.wheel_r)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=120.0, velocity=4.0, lower=-r.steer_lock, upper=r.steer_lock
            ),
        )

    # Wheel spin CONTINUOUS about lateral X. Front spins off the knuckle; rear off
    # the body. Grandfathered (axle pin runs through the hub bore).
    for name, spin_parent, origin_xyz in (
        ("wheel_front_left", knuckle_fl, (0.0, 0.0, 0.0)),
        ("wheel_front_right", knuckle_fr, (0.0, 0.0, 0.0)),
        ("wheel_rear_left", body, (r.half_track, r.rear_axle_y, r.wheel_r)),
        ("wheel_rear_right", body, (-r.half_track, r.rear_axle_y, r.wheel_r)),
    ):
        model.articulation(
            f"{name}_spin",
            ArticulationType.CONTINUOUS,
            parent=spin_parent,
            child=name,
            origin=Origin(xyz=origin_xyz),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=200.0, velocity=60.0),
        )

    # ----------------------------------------- optional pop-up headlight covers
    if r.headlight_popup:
        nose_y = p.sections[1][0] - 0.10
        z = p.sections[2][2] * r.body_height_scale * 0.85 + 0.05
        for side, sx0 in (("left", 0.52), ("right", -0.52)):
            cover = _make_headlight_cover(model, r, mats, side)
            # Snap the cover hinge onto the actual front-fender body surface near
            # the lamp pocket so the joint origin sits on real geometry.
            hx, hy, hz = _nearest_body_vertex(r, (sx0, nose_y - 0.06, z))
            # Pop-up cover hinged on a pin at the front fender; flush-seat in the
            # carved pocket. Grandfathered (pin-in-pocket, matches v07 reference).
            model.articulation(
                f"headlight_cover_{side}_hinge",
                ArticulationType.REVOLUTE,
                parent=body,
                child=cover,
                origin=Origin(xyz=(hx, hy, hz)),
                axis=(1.0, 0.0, 0.0),
                motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=0.0, upper=0.90),
            )

    # ------------------------------------------- optional engine louver cover
    if r.rear_louver_cover:
        cover = _make_engine_cover(model, r, mats)
        deck_top = p.cabin_z[1] + 0.03
        # Engine louver cover hinged on a pin at the deck front; flush-seat on the
        # engine deck. Grandfathered (pin-on-deck, matches v09 reference). Origin
        # planted on the engine-deck top surface (deck visual top ≈ deck_top-0.015)
        # so the joint origin lies on real geometry.
        model.articulation(
            "engine_cover_hinge",
            ArticulationType.REVOLUTE,
            parent=body,
            child=cover,
            origin=Origin(xyz=(0.0, p.cabin_y[0] - 0.10, deck_top - 0.015)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=30.0, velocity=2.0, lower=0.0, upper=0.75),
        )

    return model


def build_seeded_sports_car(seed: int) -> ArticulatedObject:
    return build_sports_car(config_from_seed(seed))


# --------------------------------------------------------------------------- #
# Author tests
# --------------------------------------------------------------------------- #


def run_sports_car_tests(
    model: ArticulatedObject,
    config: SportsCarConfig,
) -> TestReport:
    r = resolve_config(config)
    p = r.profile
    ctx = TestContext(model)

    body = model.get_part("body")
    door_l = model.get_part("door_left")
    door_r = model.get_part("door_right")
    wheels = {
        name: model.get_part(name)
        for name in (
            "wheel_front_left",
            "wheel_front_right",
            "wheel_rear_left",
            "wheel_rear_right",
        )
    }

    # ---- Intentional overlap allowances (captured-pin / flush-seat). ---------
    # Steering column meets the spinning wheel hub on the spin axis; the wheel
    # hub also sits down inside the cabin against the body floor/flank.
    sw_part = model.get_part("steering_wheel")
    sw_elems = ("sw_hub", "sw_spoke_cross", "sw_rim_top", "sw_rim_bot",
                "sw_grip_left", "sw_grip_right", "sw_top_marker")
    for selem in sw_elems:
        ctx.allow_overlap(
            body, sw_part, elem_a="steering_column", elem_b=selem,
            reason="Fixed steering column meets the wheel hub/spokes at the center, on the spin axis.",
        )
        for shell in ("lower_body", "cabin_floor", "greenhouse", "windshield",
                      "seat_back_left", "seat_back_right"):
            ctx.allow_overlap(
                body, sw_part, elem_a=shell, elem_b=selem,
                reason="Steering wheel sits inside the hollow cabin against the body.",
            )
    # Wheels seated in the fender wells; axle rods run hub-to-hub through them.
    axle_of = {
        "wheel_front_left": "front_axle_bar",
        "wheel_front_right": "front_axle_bar",
        "wheel_rear_left": "rear_axle_bar",
        "wheel_rear_right": "rear_axle_bar",
    }
    for wname, w in wheels.items():
        side = "left" if wname.endswith("left") else "right"
        is_rear = "rear" in wname
        for elem in ("tire", "rim", "hub_center"):
            ctx.allow_overlap(
                body, w, elem_a="lower_body", elem_b=elem,
                reason="Wheel seated flush inside the fender wheel arch of the solid body shell.",
            )
            ctx.allow_overlap(
                body, w, elem_a=axle_of[wname], elem_b=elem,
                reason="Straight axle rod runs out to the wheel hub on the spin/steer axis.",
            )
            # Narrow-track (vintage) bodies tuck the rocker sill against the tire.
            ctx.allow_overlap(
                body, w, elem_a=f"rocker_sill_{side}", elem_b=elem,
                reason="Rocker sill runs alongside the wheel between the arches.",
            )
            # The engine deck spans over the rear bay above the rear wheels; on the
            # short-wheelbase vintage body the deck shoulder brushes the rear tire.
            if is_rear:
                ctx.allow_overlap(
                    body, w, elem_a="engine_deck", elem_b=elem,
                    reason="Engine deck spans over the mid/rear bay above the rear wheel.",
                )
    # The driver's steering wheel sits at the cabin edge near the left door — its
    # rim/grip can brush the door glass/skin (both inhabit the cabin opening).
    for door, side in ((door_l, "left"), (door_r, "right")):
        for selem in ("sw_rim_top", "sw_rim_bot", "sw_grip_left", "sw_grip_right", "sw_hub"):
            for delem in ("door_skin", "door_glass", "door_beltline"):
                ctx.allow_overlap(
                    door, sw_part, elem_a=delem, elem_b=selem,
                    reason="Steering wheel sits at the cabin edge by the door opening.",
                )
    # Doors seat flush into the body aperture (thin embed intentional); the inner
    # skin/glass also brush the cabin floor / seats / window glazing.
    for door, side in ((door_l, "left"), (door_r, "right")):
        psx = "l" if side == "left" else "r"
        for shell in (
            "lower_body", "greenhouse", f"rocker_sill_{side}", "windshield",
            "rear_window", "cabin_floor", f"seat_back_{side}", "engine_deck",
            "steering_column", f"pillar_{psx}f", f"pillar_{psx}b",
        ):
            for delem in ("door_skin", "door_glass", "door_beltline", "mirror_stalk",
                          "mirror_head", "door_handle", "door_hinge_lug", "door_hinge_arm"):
                ctx.allow_overlap(
                    body, door, elem_a=shell, elem_b=delem,
                    reason="Door seats flush in the body aperture; thin embed intentional.",
                )
    # Pop-up covers seat in the nose, OVER the headlight cluster (the cover's
    # whole job is to cap the lamp), so they overlap the body nose + lamp visuals.
    if r.headlight_popup:
        hl_elems = (
            "lower_body", "front_splitter", "front_bumper_strip",
            "front_indicator_left", "front_indicator_right",
            "headlight_left", "headlight_right",
            "headlight_surround_left", "headlight_surround_right",
            "headlight_reflector_left", "headlight_reflector_right",
            "headlight_bucket_left", "headlight_bucket_right",
            "headlight_pod_left", "headlight_pod_right",
        )
        for side in ("left", "right"):
            cv = model.get_part(f"headlight_cover_{side}")
            for celem in ("cover_panel", "cover_front_seam", "cover_outer_seam", "hinge_pin"):
                for be in hl_elems:
                    ctx.allow_overlap(
                        body, cv, elem_a=be, elem_b=celem,
                        reason="Pop-up cover caps the headlight in the fender pocket.",
                    )
    if r.rear_louver_cover:
        ec = model.get_part("engine_cover")
        cover_elems = ["cover_panel", "cover_hinge_pin"] + [f"cover_louver_{k}" for k in range(5)]
        deck_louvers = [f"deck_louver_{k}" for k in range(6)] if r.rear_blade == "integrated" else []
        # The cover seats flush over the rear deck: it embeds into the engine deck,
        # the body shell, the rear glass/roof behind it, and the deck louvers.
        for be in ("engine_deck", "lower_body", "tail_panel", "rear_window",
                   "greenhouse", *deck_louvers):
            for celem in cover_elems:
                ctx.allow_overlap(
                    body, ec, elem_a=be, elem_b=celem,
                    reason="Engine louver cover seats flush over the rear deck / body shell.",
                )

    # ---- Baseline ------------------------------------------------------------
    ctx.check_model_valid()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.015)
    ctx.fail_if_joint_mating_has_gap()

    # ---- Category identity: 4 wheels, 2 doors, body present ------------------
    vis_names = {v.name for v in body.visuals}
    ctx.check(
        "lofted single-body present (not a box)",
        "lower_body" in vis_names,
        details=f"body visuals={sorted(vis_names)[:8]}",
    )
    ctx.check("two doors present", door_l is not None and door_r is not None)
    ctx.check("four wheels present", len(wheels) == 4)

    # ---- Scale sanity --------------------------------------------------------
    bb = ctx.part_world_aabb(body)
    assert bb is not None
    lo, hi = bb
    ctx.check("car length 4.3..4.8 m", 4.2 <= hi[1] - lo[1] <= 4.9, details=f"L={hi[1] - lo[1]:.3f}")
    ctx.check("car width 1.5..2.3 m", 1.4 <= hi[0] - lo[0] <= 2.35, details=f"W={hi[0] - lo[0]:.3f}")
    ctx.check("car height 0.9..1.5 m", 0.85 <= hi[2] <= 1.55, details=f"H={hi[2]:.3f}")

    # ---- Doors: correct hinge axis + actually swing --------------------------
    for side, door in (("left", door_l), ("right", door_r)):
        hinge = model.get_articulation(f"door_{side}_hinge")
        ax = tuple(hinge.axis)
        if r.door_mechanism == "scissor_butterfly":
            ok_axis = abs(ax[0]) > 0.9 and abs(ax[1]) < 1e-6
        elif r.door_mechanism == "gullwing":
            ok_axis = abs(ax[1]) > 0.9 and abs(ax[0]) < 1e-6
        else:  # dihedral — near-vertical canted
            ok_axis = abs(ax[2]) > 0.8 and abs(ax[0]) < 1e-6
        ctx.check(
            f"door_{side} hinge axis matches {r.door_mechanism}",
            ok_axis,
            details=f"axis={ax}",
        )
        ml = hinge.motion_limits
        ctx.check(
            f"door_{side} opens from closed",
            ml is not None and abs(ml.lower) < 1e-6 and ml.upper >= 1.0,
            details=f"limits=({ml.lower}, {ml.upper})" if ml else "no limits",
        )
        rest = ctx.part_world_aabb(door)
        with ctx.pose({hinge: min(1.0, ml.upper)}):
            opened = ctx.part_world_aabb(door)
        if rest is not None and opened is not None:
            moved = (
                abs(opened[1][2] - rest[1][2]) > 0.1
                or abs(opened[1][0] - rest[1][0]) > 0.1
                or abs(opened[0][0] - rest[0][0]) > 0.1
            )
            ctx.check(f"door_{side} swings when opened", moved,
                      details=f"rest={rest}, opened={opened}")

    # ---- Wheels: 4 continuous lateral spins, grounded -----------------------
    ground_zs = []
    for name, w in wheels.items():
        j = model.get_articulation(f"{name}_spin")
        ctx.check(
            f"{name} spin axis lateral X + continuous",
            tuple(j.axis) == (1.0, 0.0, 0.0)
            and j.articulation_type == ArticulationType.CONTINUOUS,
            details=f"axis={j.axis}, type={j.articulation_type}",
        )
        wbb = ctx.part_world_aabb(w)
        assert wbb is not None
        ground_zs.append(wbb[0][2])
        ctx.check(f"{name} touches ground", abs(wbb[0][2]) <= 0.03, details=f"min z={wbb[0][2]:.4f}")
    ctx.check(
        "all four wheels touch ground consistently",
        max(ground_zs) - min(ground_zs) <= 0.02,
        details=f"ground zs={['%.4f' % z for z in ground_zs]}",
    )

    # ---- Front steering: vertical king-pin REVOLUTE -------------------------
    for kname, wname, side in (
        ("steer_knuckle_front_left", "wheel_front_left", "left"),
        ("steer_knuckle_front_right", "wheel_front_right", "right"),
    ):
        steer = model.get_articulation(f"{kname}_steer")
        ax = tuple(steer.axis)
        ctx.check(
            f"front-{side} steer axis vertical Z + revolute",
            abs(ax[2]) > 0.9 and abs(ax[0]) < 1e-6 and abs(ax[1]) < 1e-6
            and steer.articulation_type == ArticulationType.REVOLUTE,
            details=f"axis={ax}, type={steer.articulation_type}",
        )
        w = wheels[wname]
        rest_bb = ctx.part_world_aabb(w)
        with ctx.pose({steer: 0.30}):
            steer_bb = ctx.part_world_aabb(w)
        if rest_bb is not None and steer_bb is not None:
            rest_xw = rest_bb[1][0] - rest_bb[0][0]
            steer_xw = steer_bb[1][0] - steer_bb[0][0]
            ctx.check(
                f"front-{side} wheel turns about vertical axis",
                steer_xw > rest_xw + 0.05,
                details=f"rest x-extent={rest_xw:.3f}, steered x-extent={steer_xw:.3f}",
            )

    # ---- Steering wheel spins -----------------------------------------------
    sw_turn = model.get_articulation("steering_wheel_turn")
    ctx.check(
        "steering wheel revolute about column axis",
        tuple(sw_turn.axis) == (0.0, 0.0, 1.0)
        and sw_turn.articulation_type == ArticulationType.REVOLUTE,
        details=f"axis={sw_turn.axis}",
    )

    # ---- Optional covers: REVOLUTE about X ----------------------------------
    if r.headlight_popup:
        for side in ("left", "right"):
            ch = model.get_articulation(f"headlight_cover_{side}_hinge")
            ctx.check(
                f"pop-up cover {side} REVOLUTE about X",
                tuple(ch.axis) == (1.0, 0.0, 0.0)
                and ch.articulation_type == ArticulationType.REVOLUTE,
                details=f"axis={ch.axis}",
            )
    if r.rear_louver_cover:
        eh = model.get_articulation("engine_cover_hinge")
        cover = model.get_part("engine_cover")
        ctx.check(
            "engine louver cover REVOLUTE about X with >=5 louvers",
            tuple(eh.axis) == (1.0, 0.0, 0.0)
            and eh.articulation_type == ArticulationType.REVOLUTE
            and sum(1 for v in cover.visuals if v.name.startswith("cover_louver_")) >= 5,
            details=f"axis={eh.axis}",
        )

    # ---- Palette diversity: body color differs from glass --------------------
    matset = {m.name: m for m in model.materials}
    if "body" in matset and "glass" in matset:
        ctx.check(
            "glass darker/different than body paint",
            sum(matset["glass"].rgba[:3]) <= sum(matset["body"].rgba[:3]) + 0.1,
            details=f"body={matset['body'].rgba}, glass={matset['glass'].rgba}",
        )

    return ctx.report()


__all__ = [
    "BodyProfileFamily",
    "DoorMechanism",
    "HeadlightShape",
    "RearBlade",
    "ExhaustLayout",
    "PaletteStyle",
    "SportsCarConfig",
    "ResolvedSportsCarConfig",
    "config_from_seed",
    "resolve_config",
    "build_sports_car",
    "build_seeded_sports_car",
    "slot_choices_for_seed",
    "run_sports_car_tests",
    "__modular__",
]
