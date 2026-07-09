"""Handgun (pistol) modular procedural template.

`mixed` pattern: the upstream ``action`` slot selects one of TWO completely
disjoint motion spines, each driving its own conditional barrel/sights/grip
candidate pools.

  * ``revolver_swingout`` — swing-out cylinder revolver. Parts
    ``frame / crane / cylinder / trigger / hammer / grip``; joints
    ``crane_swing`` (REVOLUTE,-X) / ``cylinder_spin`` (CONTINUOUS,+X) /
    ``trigger_pull`` (REVOLUTE,+Y) / ``hammer_cock`` (REVOLUTE,-Y) /
    ``grip_mount`` (FIXED). Carries a
    ``chamber_count`` multiplicity axis: the 6 hard-coded chambers/flutes/liners
    of the parent are refactored into a CHAMBER_COUNT loop with
    ``_chamber_position(k)`` / ``_flute_position(k)`` helpers (adopted from the
    rev5shot / rev8shot 5-star variants). All chambers are rigid inside ONE
    cylinder part that rides a single CONTINUOUS ``cylinder_spin``.
  * ``semi_auto_slide`` — reciprocating-slide semi-automatic pistol. Parts
    ``frame / slide / trigger / takedown_lever / magazine``; joints
    ``frame_to_slide`` (PRISMATIC,-X) / ``frame_to_trigger`` (REVOLUTE,+Y) /
    ``frame_to_takedown_lever`` (REVOLUTE,+Y) / ``frame_to_magazine``
    (PRISMATIC, along the raked grip axis). No multiplicity axis.

The two spines share NO geometry helpers (per spec §模板实现备注): the revolver
path uses ``_xcyl`` / ``_bbox`` / ``_frame_body_solid`` / ``_barrel_solid`` /
``_crane_solid`` / ``_cylinder_solid``; the semi-auto path uses
``_build_frame_solid`` / ``_build_rail_solid`` / ``_build_slide_solid`` /
``_build_trigger_solid``. ``build_handgun`` dispatches on ``action``.

5-star source records (Module Source Index of the reviewed spec):
  * revolver spine + revolver_mid + revolver_fixed + revolver_square:
      rec_model-a-classic-double-action-revolver-colt-pyth_20260610_081456_135585_9e7d2f05
  * chamber-count refactor prototype: rec_handgun_var_rev5shot / rev8shot
  * revolver_snub: rec_handgun_var_revsnub
  * revolver_adjustable: rec_handgun_var_revadjsight
  * revolver_roundbutt: rec_handgun_var_revroundbutt
  * semi-auto spine + pistol_mid + pistol_fixed + pistol_straight:
      rec_model-a-modern-striker-fired-semi-automatic-pist_20260610_081229_238472_7155f244
  * pistol_long: rec_handgun_var_pistlong
  * pistol_optic_cut: rec_handgun_var_pistoptic
  * pistol_compact: rec_handgun_var_pistcompact

Primitive types are preserved per AUTHORING.md §A Rule 3: every solid
that the source records build with cadquery is built with cadquery here
(``mesh_from_cadquery``); only literal dimensions, enum branches and the chamber
multiplicity are parameterised. Rule 1 (decorations are visuals, not parts) and
Rule 2 (every separate child part has a real MatingContract or a grandfathered
captured-pin joint with element-scoped allow_overlap) are upheld.
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

# Mesh tessellation: coarse enough to keep the revolver's ~10 cadquery solids
# fast under a parallel sweep (avoids the appliance mesh-perf SIGKILL), fine
# enough to preserve the sculpted frame / cylinder / hammer identity.
MESH_TOL = 0.0009
MESH_ANG_TOL = 0.16

Action = Literal["revolver_swingout", "semi_auto_slide"]
BarrelLength = Literal["revolver_mid", "revolver_snub", "pistol_mid", "pistol_long"]
Sights = Literal["revolver_fixed", "revolver_adjustable", "pistol_fixed", "pistol_optic_cut"]
Grip = Literal["revolver_square", "revolver_roundbutt", "pistol_straight", "pistol_compact"]
PaletteStyle = Literal[
    "blued_steel",
    "stainless",
    "two_tone",
    "walnut_panel",
    "polymer_olive",
    "optic_black",
]

# Compatibility matrix: action -> legal candidate pools (spec §Compatibility).
_REVOLVER_BARRELS = ("revolver_mid", "revolver_snub")
_REVOLVER_SIGHTS = ("revolver_fixed", "revolver_adjustable")
_REVOLVER_GRIPS = ("revolver_square", "revolver_roundbutt")
_PISTOL_BARRELS = ("pistol_mid", "pistol_long")
_PISTOL_SIGHTS = ("pistol_fixed", "pistol_optic_cut")
_PISTOL_GRIPS = ("pistol_straight", "pistol_compact")

CHAMBER_MIN, CHAMBER_MAX = 5, 8

# ---------------------------------------------------------------------------
# Palette presets (≥3; all RGB extracted from the two parents + variant
# materials — see spec palette_style table). Every preset defines the full key
# set so any spine can register all materials it needs.
# ---------------------------------------------------------------------------

_PALETTE_PRESETS: dict[str, dict[str, tuple[float, float, float, float]]] = {
    "blued_steel": {
        "body": (0.13, 0.13, 0.15, 1.0),
        "body_mid": (0.10, 0.10, 0.12, 1.0),
        "body_bright": (0.20, 0.20, 0.22, 1.0),
        "dark": (0.05, 0.05, 0.06, 1.0),
        "bore": (0.03, 0.03, 0.04, 1.0),
        "grip": (0.07, 0.07, 0.08, 1.0),
        "grip_dark": (0.04, 0.04, 0.05, 1.0),
        "accent": (0.30, 0.30, 0.32, 1.0),
        "lens": (0.12, 0.18, 0.16, 1.0),
    },
    "stainless": {
        "body": (0.78, 0.79, 0.81, 1.0),
        "body_mid": (0.71, 0.72, 0.74, 1.0),
        "body_bright": (0.83, 0.84, 0.86, 1.0),
        "dark": (0.16, 0.16, 0.18, 1.0),
        "bore": (0.08, 0.08, 0.09, 1.0),
        "grip": (0.45, 0.28, 0.15, 1.0),
        "grip_dark": (0.28, 0.16, 0.09, 1.0),
        "accent": (0.46, 0.46, 0.48, 1.0),
        "lens": (0.15, 0.22, 0.18, 1.0),
    },
    "two_tone": {
        "body": (0.10, 0.10, 0.11, 1.0),
        "body_mid": (0.28, 0.28, 0.30, 1.0),
        "body_bright": (0.42, 0.43, 0.36, 1.0),
        "dark": (0.05, 0.05, 0.06, 1.0),
        "bore": (0.04, 0.04, 0.05, 1.0),
        "grip": (0.09, 0.09, 0.09, 1.0),
        "grip_dark": (0.06, 0.06, 0.06, 1.0),
        "accent": (0.46, 0.46, 0.48, 1.0),
        "lens": (0.15, 0.22, 0.18, 1.0),
    },
    "walnut_panel": {
        "body": (0.71, 0.72, 0.74, 1.0),
        "body_mid": (0.66, 0.67, 0.69, 1.0),
        "body_bright": (0.80, 0.81, 0.83, 1.0),
        "dark": (0.16, 0.16, 0.18, 1.0),
        "bore": (0.08, 0.08, 0.09, 1.0),
        "grip": (0.45, 0.28, 0.15, 1.0),
        "grip_dark": (0.28, 0.16, 0.09, 1.0),
        "accent": (0.46, 0.46, 0.48, 1.0),
        "lens": (0.15, 0.22, 0.18, 1.0),
    },
    "polymer_olive": {
        "body": (0.42, 0.43, 0.36, 1.0),
        "body_mid": (0.36, 0.37, 0.31, 1.0),
        "body_bright": (0.48, 0.49, 0.42, 1.0),
        "dark": (0.07, 0.07, 0.07, 1.0),
        "bore": (0.05, 0.05, 0.06, 1.0),
        "grip": (0.20, 0.20, 0.21, 1.0),
        "grip_dark": (0.07, 0.07, 0.07, 1.0),
        "accent": (0.05, 0.05, 0.05, 1.0),
        "lens": (0.15, 0.22, 0.18, 1.0),
    },
    "optic_black": {
        "body": (0.10, 0.10, 0.11, 1.0),
        "body_mid": (0.16, 0.16, 0.18, 1.0),
        "body_bright": (0.26, 0.26, 0.28, 1.0),
        "dark": (0.05, 0.05, 0.06, 1.0),
        "bore": (0.04, 0.04, 0.05, 1.0),
        "grip": (0.09, 0.09, 0.09, 1.0),
        "grip_dark": (0.06, 0.06, 0.06, 1.0),
        "accent": (0.46, 0.46, 0.48, 1.0),
        "lens": (0.15, 0.22, 0.18, 1.0),
    },
}

_REVOLVER_PALETTE_PREF = ("stainless", "blued_steel", "walnut_panel")
_PISTOL_PALETTE_PREF = ("two_tone", "polymer_olive", "blued_steel", "optic_black")


# ---------------------------------------------------------------------------
# Config dataclasses
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class HandgunConfig:
    action: Action = "revolver_swingout"
    barrel_length: BarrelLength = "revolver_mid"
    sights: Sights = "revolver_fixed"
    grip: Grip = "revolver_square"
    palette_style: PaletteStyle = "stainless"

    # revolver-only multiplicity axis
    chamber_count: int = 6

    # continuous local scales
    barrel_len_scale: float = 1.0
    grip_height_scale: float = 1.0


@dataclass(frozen=True)
class ResolvedHandgunConfig:
    action: Action
    barrel_length: BarrelLength
    sights: Sights
    grip: Grip
    palette_style: PaletteStyle
    chamber_count: int
    barrel_len_scale: float
    grip_height_scale: float
    # derived
    cyl_radius_scale: float
    chamber_r: float
    mag_travel: float
    palette: dict[str, tuple[float, float, float, float]]


# ---------------------------------------------------------------------------
# Seed -> config (deterministic procedural sampling; seed 0 not special)
# ---------------------------------------------------------------------------


def config_from_seed(seed: int) -> HandgunConfig:
    rng = random.Random(seed)

    # action: roughly balanced spine selection.
    action: Action = rng.choice(("revolver_swingout", "semi_auto_slide"))

    if action == "revolver_swingout":
        barrel_length = rng.choice(_REVOLVER_BARRELS)
        sights = rng.choice(_REVOLVER_SIGHTS)
        grip = rng.choice(_REVOLVER_GRIPS)
        # chamber_count weighted: 6 most common, 5/7 next, 8 rare.
        chamber_count = rng.choices(
            (5, 6, 7, 8), weights=(0.22, 0.42, 0.22, 0.14), k=1
        )[0]
        palette_pref = _REVOLVER_PALETTE_PREF
    else:
        barrel_length = rng.choice(_PISTOL_BARRELS)
        sights = rng.choice(_PISTOL_SIGHTS)
        grip = rng.choice(_PISTOL_GRIPS)
        chamber_count = 6  # ignored for semi-auto
        palette_pref = _PISTOL_PALETTE_PREF

    # palette: prefer spine defaults, allow free others; honour gating.
    palette_style = rng.choices(
        palette_pref + ("blued_steel",),
        weights=tuple(3.0 for _ in palette_pref) + (1.0,),
        k=1,
    )[0]
    # gating: walnut_panel needs revolver_square; optic_black prefers optic sights.
    if palette_style == "walnut_panel" and grip != "revolver_square":
        palette_style = "stainless" if action == "revolver_swingout" else "two_tone"
    if action == "semi_auto_slide" and sights == "pistol_optic_cut" and rng.random() < 0.5:
        palette_style = "optic_black"

    barrel_len_scale = round(rng.uniform(0.85, 1.15), 4)
    grip_height_scale = round(rng.uniform(0.92, 1.08), 4)

    return HandgunConfig(
        action=action,
        barrel_length=barrel_length,
        sights=sights,
        grip=grip,
        palette_style=palette_style,
        chamber_count=chamber_count,
        barrel_len_scale=barrel_len_scale,
        grip_height_scale=grip_height_scale,
    )


def resolve_config(config: HandgunConfig) -> ResolvedHandgunConfig:
    action = config.action
    if action not in ("revolver_swingout", "semi_auto_slide"):
        raise ValueError(f"Unsupported action: {action}")

    # Compatibility gate: clamp candidate pools to the chosen spine.
    if action == "revolver_swingout":
        barrel_length = (
            config.barrel_length
            if config.barrel_length in _REVOLVER_BARRELS
            else "revolver_mid"
        )
        sights = config.sights if config.sights in _REVOLVER_SIGHTS else "revolver_fixed"
        grip = config.grip if config.grip in _REVOLVER_GRIPS else "revolver_square"
    else:
        barrel_length = (
            config.barrel_length if config.barrel_length in _PISTOL_BARRELS else "pistol_mid"
        )
        sights = config.sights if config.sights in _PISTOL_SIGHTS else "pistol_fixed"
        grip = config.grip if config.grip in _PISTOL_GRIPS else "pistol_straight"

    palette_style = config.palette_style
    if palette_style not in _PALETTE_PRESETS:
        palette_style = "stainless" if action == "revolver_swingout" else "two_tone"
    # palette gating fallback (defensive — sampler already enforces this).
    if palette_style == "walnut_panel" and grip != "revolver_square":
        palette_style = "stainless"

    chamber_count = int(config.chamber_count)
    chamber_count = max(CHAMBER_MIN, min(chamber_count, CHAMBER_MAX))

    barrel_len_scale = max(0.85, min(float(config.barrel_len_scale), 1.15))
    grip_height_scale = max(0.92, min(float(config.grip_height_scale), 1.08))

    # Derived: cyl_radius_scale grows mildly with chamber count to hold the
    # inter-chamber wall thickness; chamber_r shrinks for high counts so the
    # chambers never interpenetrate on the chamber circle.
    cyl_radius_scale = 1.0 + 0.045 * (chamber_count - 6)
    cyl_radius_scale = max(0.92, min(cyl_radius_scale, 1.12))
    # base chamber radius 0.0052 (6-shot); scale down for more chambers.
    chamber_r = 0.0052 * (6.0 / chamber_count) ** 0.5
    chamber_r = max(0.0040, min(chamber_r, 0.0056))
    # clearance check: chambers must not touch on the (scaled) chamber circle.
    circle_r = 0.013 * cyl_radius_scale
    max_chamber_r = math.sin(math.pi / chamber_count) * circle_r * 0.92
    chamber_r = min(chamber_r, max_chamber_r)

    # Derived: mag travel by grip module, scaled by grip height.
    base_mag = 0.10 if grip == "pistol_straight" else 0.07
    mag_travel = round(base_mag * grip_height_scale, 4)
    if action == "revolver_swingout":
        mag_travel = 0.0

    palette = dict(_PALETTE_PRESETS[palette_style])

    return ResolvedHandgunConfig(
        action=action,
        barrel_length=barrel_length,
        sights=sights,
        grip=grip,
        palette_style=palette_style,
        chamber_count=chamber_count,
        barrel_len_scale=barrel_len_scale,
        grip_height_scale=grip_height_scale,
        cyl_radius_scale=cyl_radius_scale,
        chamber_r=chamber_r,
        mag_travel=mag_travel,
        palette=palette,
    )


def slot_choices_for_seed(seed: int) -> list[tuple[str, str]]:
    r = resolve_config(config_from_seed(seed))
    choices: list[tuple[str, str]] = [
        ("action", r.action),
        ("barrel_length", r.barrel_length),
        ("sights", r.sights),
        ("grip", r.grip),
    ]
    if r.action == "revolver_swingout":
        choices.append(("chamber_count", f"chambers_{r.chamber_count}"))
    return choices


# ===========================================================================
# REVOLVER SPINE  (cadquery helpers — disjoint from the semi-auto path)
# Adapted verbatim from the Colt-Python parent (model.py:L19-L488) with
# literals parameterised by barrel/sights/grip modules + CHAMBER_COUNT.
# ===========================================================================

# World frame: +X muzzle, +Y shooter's left, +Z up; grip butt on z=0.
_R_BORE_Z = 0.108
_R_BARREL_R = 0.011
_R_BARREL_X0 = -0.030
_R_FRAME_FRONT = -0.026

_R_CYL_Z = 0.095
_R_CYL_R = 0.0205
_R_CYL_X0, _R_CYL_X1 = -0.0745, -0.0325
_R_CYL_CX = 0.5 * (_R_CYL_X0 + _R_CYL_X1)
_R_CYL_LEN = _R_CYL_X1 - _R_CYL_X0
_R_CHAMBER_CIRCLE_R = 0.013
_R_CYL_BORE_R = 0.0056

_R_FRAME_HALF_W = 0.012
_R_RAIL_Z0, _R_RAIL_Z1 = 0.064, 0.070
_R_STRAP_Z0, _R_STRAP_Z1 = 0.118, 0.126
_R_FRAME_REAR_BOT = -0.118

_R_LUG_Z0, _R_LUG_Z1 = 0.083, 0.100
_R_LUG_HALF_W = 0.010

_R_CRANE_PIVOT = (-0.0305, 0.010, 0.062)
_R_TRIGGER_PIVOT = (-0.0630, 0.0, 0.067)
_R_HAMMER_PIVOT = (-0.0960, 0.0, 0.088)

_R_CRANE_OPEN = 0.785
_R_TRIGGER_PULL = 0.44
_R_HAMMER_COCK = 0.52

_R_GUARD_C = (-0.070, 0.047)
_R_GUARD_OUTER_R = 0.021
_R_GUARD_INNER_R = 0.0165

# Grip part-local frame anchor = frame-tang contact point (top of the grip,
# inside both grip and frame geometry). The grip is authored in absolute coords
# then re-based by -_GRIP_ANCHOR; the FIXED grip_mount origin is _GRIP_ANCHOR.
_GRIP_ANCHOR = (-0.107, 0.0, 0.064)


def _xcyl(x0: float, x1: float, r: float, y: float = 0.0, z: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("YZ", origin=(x0, 0.0, 0.0)).center(y, z).circle(r).extrude(x1 - x0)
    )


def _bbox(x0: float, x1: float, y0: float, y1: float, z0: float, z1: float) -> cq.Workplane:
    return cq.Workplane(
        "XY", origin=(0.5 * (x0 + x1), 0.5 * (y0 + y1), 0.5 * (z0 + z1))
    ).box(x1 - x0, y1 - y0, z1 - z0)


def _r_muzzle_x(r: ResolvedHandgunConfig) -> float:
    base = 0.025 if r.barrel_length == "revolver_snub" else 0.126
    return base * r.barrel_len_scale


def _r_lug_x1(r: ResolvedHandgunConfig) -> float:
    base = 0.020 if r.barrel_length == "revolver_snub" else 0.122
    return base * r.barrel_len_scale


def _frame_body_solid() -> cq.Workplane:
    profile = [
        (_R_FRAME_FRONT, _R_RAIL_Z0),
        (_R_FRAME_REAR_BOT, _R_RAIL_Z0),
        (_R_FRAME_REAR_BOT, 0.082),
        (-0.108, 0.100),
        (-0.105, _R_STRAP_Z1),
        (_R_FRAME_FRONT, _R_STRAP_Z1),
    ]
    body = cq.Workplane("XZ").polyline(profile).close().extrude(_R_FRAME_HALF_W, both=True)
    body = body.cut(_bbox(-0.078, -0.024, -0.020, 0.020, _R_RAIL_Z1, _R_STRAP_Z0))
    body = body.cut(_bbox(-0.032, -0.0255, 0.0045, 0.0140, 0.054, 0.0705))
    body = body.cut(_bbox(-0.072, -0.056, -0.0035, 0.0035, 0.060, 0.0705))
    body = body.cut(_bbox(-0.115, -0.085, -0.0045, 0.0045, 0.076, 0.127))
    return body


def _barrel_solid(r: ResolvedHandgunConfig) -> cq.Workplane:
    muzzle_x = _r_muzzle_x(r)
    lug_x1 = _r_lug_x1(r)
    snub = r.barrel_length == "revolver_snub"

    barrel = _xcyl(_R_BARREL_X0, muzzle_x, _R_BARREL_R, 0.0, _R_BORE_Z)
    # Solid underlug under the barrel.
    barrel = barrel.union(
        _bbox(_R_FRAME_FRONT, lug_x1, -_R_LUG_HALF_W, _R_LUG_HALF_W, _R_LUG_Z0, _R_LUG_Z1)
    )
    # Vented rib on top.
    barrel = barrel.union(_bbox(_R_BARREL_X0, muzzle_x, -0.0045, 0.0045, 0.117, 0.1255))

    if r.sights == "revolver_adjustable":
        # Raised ramp-style front sight with tall blade (revadjsight L115-120).
        ramp = _bbox(muzzle_x - 0.022, muzzle_x, -0.005, 0.005, 0.1255, 0.131)
        front_blade = _bbox(muzzle_x - 0.013, muzzle_x - 0.006, -0.003, 0.003, 0.131, 0.141)
        barrel = barrel.union(ramp).union(front_blade)
    else:
        # Plain front sight blade at the muzzle.
        barrel = barrel.union(
            _bbox(muzzle_x - 0.014, muzzle_x - 0.002, -0.0035, 0.0035, 0.125, 0.133)
        )

    if snub:
        # Single elongated vent slot through the shortened rib web.
        barrel = barrel.cut(_bbox(muzzle_x - 0.040, muzzle_x - 0.020, -0.015, 0.015, 0.120, 0.1235))
    else:
        # Four elongated vent slots through the rib web.
        for k in range(4):
            cx = 0.012 + 0.028 * k
            if cx + 0.010 > muzzle_x - 0.004:
                break
            barrel = barrel.cut(_bbox(cx - 0.010, cx + 0.010, -0.015, 0.015, 0.120, 0.1235))

    # Muzzle bore.
    barrel = barrel.cut(_xcyl(muzzle_x - 0.008, muzzle_x + 0.001, 0.0045, 0.0, _R_BORE_Z))
    # Lightening channel through the underlug.
    channel_x1 = min(0.058, lug_x1 - 0.008)
    barrel = barrel.cut(_xcyl(-0.0315, max(channel_x1, -0.020), 0.0062, 0.0, _R_CYL_Z))
    return barrel


def _guard_solid() -> cq.Workplane:
    guard = (
        cq.Workplane("XZ", origin=(_R_GUARD_C[0], 0.0, _R_GUARD_C[1]))
        .circle(_R_GUARD_OUTER_R)
        .circle(_R_GUARD_INNER_R)
        .extrude(0.005, both=True)
    )
    guard = guard.cut(_bbox(-0.072, -0.056, -0.0035, 0.0035, 0.056, 0.0705))
    return guard


def _rear_sight_solid(r: ResolvedHandgunConfig) -> cq.Workplane:
    if r.sights == "revolver_adjustable":
        # Tall fully-adjustable sight assembly (revadjsight L146-179).
        base = _bbox(-0.107, -0.085, -0.0075, 0.0075, 0.1255, 0.1295)
        bridge = _bbox(-0.104, -0.088, -0.006, 0.006, 0.1285, 0.133)
        housing = _bbox(-0.102, -0.090, -0.0045, 0.0045, 0.133, 0.141)
        sight = base.union(bridge).union(housing)
        windage_stem = (
            cq.Workplane("XZ", origin=(-0.096, 0.0, 0.1315)).circle(0.0025).extrude(0.012)
        )
        windage_head = (
            cq.Workplane("XZ", origin=(-0.096, 0.009, 0.1315)).circle(0.004).extrude(0.003)
        )
        sight = sight.union(windage_stem).union(windage_head)
        elev_screw = (
            cq.Workplane("XY", origin=(-0.096, 0.0, 0.1405)).circle(0.0025).extrude(0.003)
        )
        sight = sight.union(elev_screw)
        sight = sight.cut(_bbox(-0.104, -0.088, -0.0013, 0.0013, 0.137, 0.145))
        return sight
    # Low fixed notch sight (parent L142-145).
    sight = _bbox(-0.103, -0.090, -0.0065, 0.0065, 0.1255, 0.1285)
    sight = sight.cut(_bbox(-0.1035, -0.0895, -0.0012, 0.0012, 0.127, 0.129))
    return sight


def _crane_solid() -> cq.Workplane:
    px, py, pz = _R_CRANE_PIVOT
    knuckle = _xcyl(-0.0305, -0.014, 0.005, py, pz)
    lug = _bbox(-0.0305, -0.0265, 0.0055, 0.0135, 0.060, 0.075)
    block = _bbox(-0.0305, -0.0265, -0.0115, 0.0115, 0.0695, 0.0965)
    arbor = _xcyl(-0.058, -0.0295, 0.0059, 0.0, _R_CYL_Z)
    crane = knuckle.union(lug).union(block).union(arbor)
    crane = crane.cut(_xcyl(-0.060, -0.025, 0.0040, 0.0, _R_CYL_Z))
    return crane.translate((-px, -py, -pz))


def _chamber_position(k: int, circle_r: float, step_deg: float) -> tuple[float, float]:
    """(y, z) for chamber k; the top chamber (k=0) is on the bore axis (+Z)."""
    a = math.radians(90.0 + step_deg * k)
    return circle_r * math.cos(a), circle_r * math.sin(a)


def _flute_position(k: int, flute_r: float, step_deg: float) -> tuple[float, float]:
    """(y, z) for flute k, centered between chambers k and k+1."""
    a = math.radians(90.0 + step_deg * 0.5 + step_deg * k)
    return flute_r * math.cos(a), flute_r * math.sin(a)


def _cylinder_solid(r: ResolvedHandgunConfig) -> cq.Workplane:
    n = r.chamber_count
    step = 360.0 / n
    cyl_r = _R_CYL_R * r.cyl_radius_scale
    circle_r = _R_CHAMBER_CIRCLE_R * r.cyl_radius_scale
    flute_r = circle_r + 0.0085
    half = _R_CYL_LEN / 2.0
    cyl = cq.Workplane("YZ", origin=(-half, 0.0, 0.0)).circle(cyl_r).extrude(_R_CYL_LEN)
    # Center bore for the crane arbor.
    cyl = cyl.cut(_xcyl(-half - 0.001, half + 0.001, _R_CYL_BORE_R))
    # N chambers; the top chamber aligns with the barrel bore at q = 0.
    for k in range(n):
        cy, cz = _chamber_position(k, circle_r, step)
        cyl = cyl.cut(_xcyl(-half - 0.001, half + 0.001, r.chamber_r, cy, cz))
    # Rear face recess around the cylinder arbor.
    cyl = cyl.cut(_xcyl(-half - 0.001, -half + 0.0025, 0.0065))
    # N flutes between the chambers.
    for k in range(n):
        fy, fz = _flute_position(k, flute_r, step)
        cyl = cyl.cut(_xcyl(-0.016, 0.013, 0.0045, fy, fz))
    return cyl


def _trigger_solid() -> cq.Workplane:
    pts = [
        (-0.0598, 0.0690),
        (-0.0592, 0.0600),
        (-0.0610, 0.0500),
        (-0.0652, 0.0420),
        (-0.0688, 0.0395),
        (-0.0703, 0.0420),
        (-0.0676, 0.0512),
        (-0.0658, 0.0610),
        (-0.0663, 0.0690),
    ]
    blade = cq.Workplane("XZ").polyline(pts).close().extrude(0.003, both=True)
    px, py, pz = _R_TRIGGER_PIVOT
    pin_hole = cq.Workplane("XZ", origin=(px, 0.0, pz)).circle(0.0018).extrude(0.01, both=True)
    blade = blade.cut(pin_hole)
    return blade.translate((-px, -py, -pz))


def _hammer_solid() -> cq.Workplane:
    pts = [
        (-0.0865, 0.0840),
        (-0.0865, 0.0935),
        (-0.0960, 0.1000),
        (-0.1010, 0.1060),
        (-0.1040, 0.1160),
        (-0.1100, 0.1205),
        (-0.1210, 0.1240),
        (-0.1225, 0.1195),
        (-0.1130, 0.1135),
        (-0.1065, 0.1040),
        (-0.1045, 0.0950),
        (-0.1040, 0.0840),
        (-0.0960, 0.0790),
    ]
    body = cq.Workplane("XZ").polyline(pts).close().extrude(0.004, both=True)
    pad = _bbox(-0.1225, -0.110, -0.0055, 0.0055, 0.1175, 0.1245)
    body = body.union(pad)
    px, py, pz = _R_HAMMER_PIVOT
    pin_hole = cq.Workplane("XZ", origin=(px, 0.0, pz)).circle(0.0026).extrude(0.012, both=True)
    body = body.cut(pin_hole)
    return body.translate((-px, -py, -pz))


def _grip_solid(r: ResolvedHandgunConfig) -> cq.Workplane:
    hs = r.grip_height_scale
    if r.grip == "revolver_roundbutt":
        # Spline-like round-butt outline (revroundbutt L249-268): smaller, more
        # curved heel; lowest point kisses z=0 at a single location.
        pts = [
            (-0.096, 0.0645),
            (-0.101, 0.0480),
            (-0.108, 0.0280),
            (-0.115, 0.0120),
            (-0.121, 0.0030),
            (-0.128, 0.0000),
            (-0.136, 0.0020),
            (-0.141, 0.0080),
            (-0.145, 0.0180),
            (-0.146, 0.0300),
            (-0.143, 0.0420),
            (-0.137, 0.0520),
            (-0.129, 0.0600),
            (-0.121, 0.0640),
            (-0.118, 0.0645),
        ]
    else:
        # Square target-style butt (parent L249-263): flat heel.
        pts = [
            (-0.096, 0.0645),
            (-0.103, 0.0420),
            (-0.111, 0.0180),
            (-0.118, 0.0050),
            (-0.124, 0.0000),
            (-0.150, 0.0000),
            (-0.156, 0.0060),
            (-0.158, 0.0160),
            (-0.1545, 0.0300),
            (-0.146, 0.0440),
            (-0.135, 0.0560),
            (-0.124, 0.0625),
            (-0.118, 0.0645),
        ]
    if abs(hs - 1.0) > 1e-6:
        # Scale grip height about the frame-tang top (z≈0.0645) so the tang
        # contact is preserved and only the butt drops/rises.
        top_z = 0.0645
        pts = [(x, top_z - (top_z - z) * hs) for (x, z) in pts]
    grip = cq.Workplane("XZ").polyline(pts).close().extrude(0.017, both=True)
    try:
        grip = grip.edges(">Y or <Y").fillet(0.0035)
    except Exception:
        pass
    # Re-base into the grip part-local frame: the FIXED grip_mount joint origin
    # is GRIP_ANCHOR (the frame-tang contact), so the part frame must contain
    # (0,0,0) on real geometry. Authoring is absolute, so subtract the anchor.
    return grip.translate((-_GRIP_ANCHOR[0], -_GRIP_ANCHOR[1], -_GRIP_ANCHOR[2]))


def _build_revolver(model: ArticulatedObject, r: ResolvedHandgunConfig, assets) -> None:
    # Materials are referenced by name in part.visual(material=...); all palette
    # keys were already registered via model.material(...) in build_handgun.

    def _mesh(solid, name):
        return mesh_from_cadquery(
            solid, name, assets=assets, tolerance=MESH_TOL, angular_tolerance=MESH_ANG_TOL
        )

    # ----- frame (root) -----
    frame = model.part("frame")
    frame.visual(_mesh(_frame_body_solid(), "frame_body"), material="body", name="frame_body")
    frame.visual(_mesh(_barrel_solid(r), "barrel_assembly"), material="body", name="barrel_assembly")
    frame.visual(_mesh(_guard_solid(), "trigger_guard"), material="body", name="trigger_guard")
    frame.visual(
        _mesh(_rear_sight_solid(r), "rear_sight"), material="dark", name="rear_sight"
    )
    frame.visual(
        Box((0.018, 0.0025, 0.010)),
        origin=Origin(xyz=(-0.091, 0.01325, 0.101)),
        material="body_mid",
        name="cylinder_latch",
    )
    frame.visual(
        Cylinder(radius=0.0022, length=0.0080),
        origin=Origin(
            xyz=(_R_TRIGGER_PIVOT[0], 0.0, _R_TRIGGER_PIVOT[2]), rpy=(math.pi / 2.0, 0.0, 0.0)
        ),
        material="body_mid",
        name="trigger_pin",
    )
    frame.visual(
        Cylinder(radius=0.0030, length=0.0100),
        origin=Origin(
            xyz=(_R_HAMMER_PIVOT[0], 0.0, _R_HAMMER_PIVOT[2]), rpy=(math.pi / 2.0, 0.0, 0.0)
        ),
        material="body_mid",
        name="hammer_pin",
    )

    # ----- crane -----
    crane = model.part("crane")
    crane.visual(_mesh(_crane_solid(), "crane_body"), material="body", name="crane_body")
    model.articulation(
        "crane_swing",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=crane,
        origin=Origin(xyz=_R_CRANE_PIVOT),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=_R_CRANE_OPEN),
    )

    # ----- cylinder (single part; chambers/liners rigid inside) -----
    cylinder = model.part("cylinder")
    cylinder.visual(
        _mesh(_cylinder_solid(r), "cylinder_body"), material="body_mid", name="cylinder_body"
    )
    n = r.chamber_count
    step = 360.0 / n
    circle_r = _R_CHAMBER_CIRCLE_R * r.cyl_radius_scale
    liner_r = r.chamber_r + 0.0003
    for k in range(n):
        cy, cz = _chamber_position(k, circle_r, step)
        cylinder.visual(
            Cylinder(radius=liner_r, length=0.0405),
            origin=Origin(xyz=(0.0, cy, cz), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="bore",
            name=f"chamber_liner_{k}",
        )
    model.articulation(
        "cylinder_spin",
        ArticulationType.CONTINUOUS,
        parent=crane,
        child=cylinder,
        origin=Origin(
            xyz=(
                _R_CYL_CX - _R_CRANE_PIVOT[0],
                0.0 - _R_CRANE_PIVOT[1],
                _R_CYL_Z - _R_CRANE_PIVOT[2],
            )
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=12.0),
    )

    # ----- trigger -----
    trigger = model.part("trigger")
    trigger.visual(
        _mesh(_trigger_solid(), "trigger_blade"), material="body_bright", name="trigger_blade"
    )
    model.articulation(
        "trigger_pull",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=trigger,
        origin=Origin(xyz=_R_TRIGGER_PIVOT),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=4.0, lower=0.0, upper=_R_TRIGGER_PULL),
    )

    # ----- hammer -----
    hammer = model.part("hammer")
    hammer.visual(_mesh(_hammer_solid(), "hammer_body"), material="body", name="hammer_body")
    model.articulation(
        "hammer_cock",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=hammer,
        origin=Origin(xyz=_R_HAMMER_PIVOT),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=4.0, lower=0.0, upper=_R_HAMMER_COCK),
    )

    # ----- grip (separate part, FIXED to frame tang) -----
    grip = model.part("grip")
    grip.visual(_mesh(_grip_solid(r), "grip_body"), material="grip", name="grip_body")
    ax, ay, az = _GRIP_ANCHOR  # grip visuals are also re-based into part frame
    if r.grip == "revolver_square":
        # walnut panel + screw decorations (parent L467-479); only on square grip.
        for side, sy in (("left", 1.0), ("right", -1.0)):
            grip.visual(
                Box((0.024, 0.0008, 0.044)),
                origin=Origin(xyz=(-0.131 - ax, sy * 0.0172 - ay, 0.0315 - az), rpy=(0.0, 0.50, 0.0)),
                material="grip_dark",
                name=f"{side}_grip_panel",
            )
            grip.visual(
                Cylinder(radius=0.0025, length=0.0012),
                origin=Origin(
                    xyz=(-0.131 - ax, sy * 0.01765 - ay, 0.0315 - az), rpy=(math.pi / 2.0, 0.0, 0.0)
                ),
                material="dark",
                name=f"{side}_grip_screw",
            )
    else:
        for side, sy in (("left", 1.0), ("right", -1.0)):
            grip.visual(
                Box((0.022, 0.0008, 0.038)),
                origin=Origin(xyz=(-0.127 - ax, sy * 0.0172 - ay, 0.0330 - az), rpy=(0.0, 0.42, 0.0)),
                material="grip_dark",
                name=f"{side}_grip_panel",
            )
            grip.visual(
                Cylinder(radius=0.0025, length=0.0012),
                origin=Origin(
                    xyz=(-0.127 - ax, sy * 0.01765 - ay, 0.0330 - az), rpy=(math.pi / 2.0, 0.0, 0.0)
                ),
                material="dark",
                name=f"{side}_grip_screw",
            )
    # grip_body geometry overlaps the frame tang region (captured tang), so this
    # FIXED joint is grandfathered (no MatingContract); contact is enforced by
    # the expect_contact in run_handgun_tests + allow_overlap. The origin sits at
    # _GRIP_ANCHOR (frame-tang contact); the grip part frame was re-based so it
    # contains (0,0,0) on real geometry there, keeping the grip world position.
    model.articulation(
        "grip_mount",
        ArticulationType.FIXED,
        parent=frame,
        child=grip,
        origin=Origin(xyz=_GRIP_ANCHOR),
    )


# ===========================================================================
# SEMI-AUTO SPINE  (cadquery helpers — disjoint from the revolver path)
# Adapted verbatim from the striker-fired parent (model.py:L34-L331) with
# literals parameterised by barrel/sights/grip modules.
# ===========================================================================

_S_RAKE = 0.32
_S_SIN_R = math.sin(_S_RAKE)
_S_COS_R = math.cos(_S_RAKE)

_S_FRAME_HW = 0.014
_S_SLIDE_HW = 0.015

_S_SLIDE_X0 = -0.105
_S_SLIDE_Z0, _S_SLIDE_Z1 = 0.118, 0.150
_S_SLIDE_H = _S_SLIDE_Z1 - _S_SLIDE_Z0
_S_FRAME_TOP = _S_SLIDE_Z0

_S_BORE_Z = 0.133
_S_BORE_R = 0.0055

_S_TRIG_PIVOT = (0.013, 0.0, 0.086)
_S_TRIG_PULL = math.radians(25.0)
_S_TKD_PIVOT = (0.004, _S_FRAME_HW, 0.104)
_S_SLIDE_TRAVEL = 0.045


def _s_slide_x1(r: ResolvedHandgunConfig) -> float:
    base = 0.130 if r.barrel_length == "pistol_long" else 0.105
    return base * r.barrel_len_scale


def _s_butt(r: ResolvedHandgunConfig) -> tuple[tuple[float, float], tuple[float, float], float, float]:
    """Return (BUTT_F, BUTT_G, BUTT_CX, BUTT_CZ) for the chosen grip module,
    scaled along the grip height by grip_height_scale about the front corner."""
    if r.grip == "pistol_compact":
        bf = (-0.046, 0.000)
        bg = (-0.108, 0.021)
        cx, cz = -0.077, 0.0105
    else:  # pistol_straight
        bf = (-0.054, 0.0074)
        bg = (-0.116, 0.0279)
        cx, cz = -0.085, 0.01765
    return bf, bg, cx, cz


def _build_frame_solid(r: ResolvedHandgunConfig) -> cq.Workplane:
    bf, bg, cx, cz = _s_butt(r)
    front_x = 0.125 if r.barrel_length == "pistol_long" else 0.100
    front_x = front_x  # frame dust-cover front
    magwell_len = 0.065 if r.grip == "pistol_compact" else 0.10

    outline = [
        (front_x, _S_FRAME_TOP),
        (front_x, 0.098),
        (0.046, 0.098),
        (0.038, 0.056),
        (-0.044, 0.052),
        bf,
        bg,
        (-0.0925, 0.090),
        (-0.112, 0.103),
        (-0.112, 0.111),
        (-0.100, _S_FRAME_TOP),
    ]
    frame = cq.Workplane("XZ").polyline(outline).close().extrude(_S_FRAME_HW, both=True)

    hole = [
        (0.035, 0.085),
        (0.029, 0.062),
        (0.004, 0.058),
        (-0.028, 0.064),
        (-0.020, 0.085),
    ]
    guard_hole = cq.Workplane("XZ").polyline(hole).close().extrude(0.02, both=True)
    frame = frame.cut(guard_hole)

    well = (
        cq.Workplane("XY")
        .box(0.025, 0.024, magwell_len)
        .rotate((0, 0, 0), (0, 1, 0), math.degrees(_S_RAKE))
        .translate((cx + _S_SIN_R * 0.03, 0.0, cz + _S_COS_R * 0.03))
    )
    frame = frame.cut(well)
    return frame


def _build_rail_solid(r: ResolvedHandgunConfig) -> cq.Workplane:
    if r.barrel_length == "pistol_long":
        front_x = 0.125 * r.barrel_len_scale
        rail_rear = 0.046
        rail_front = front_x - 0.002
        rail_len = max(0.040, rail_front - rail_rear)
        rail_cx = (rail_rear + rail_front) / 2.0
        rail = cq.Workplane("XY").box(rail_len, 0.022, 0.010).translate((rail_cx, 0.0, 0.094))
        slot_pitch = rail_len / 6.0
        for i in range(5):
            xs = rail_rear + slot_pitch * (i + 1)
            slot = cq.Workplane("XY").box(0.0055, 0.030, 0.0075).translate((xs, 0.0, 0.09275))
            rail = rail.cut(slot)
        return rail
    rail = cq.Workplane("XY").box(0.052, 0.022, 0.010).translate((0.072, 0.0, 0.094))
    for xs in (0.057, 0.070, 0.083):
        slot = cq.Workplane("XY").box(0.0055, 0.030, 0.0075).translate((xs, 0.0, 0.09275))
        rail = rail.cut(slot)
    return rail


def _build_slide_solid(r: ResolvedHandgunConfig) -> cq.Workplane:
    slide_x1 = _s_slide_x1(r)
    slide_len = slide_x1 - _S_SLIDE_X0
    slide_mid_x = (_S_SLIDE_X0 + slide_x1) / 2.0

    s = cq.Workplane("XY").box(slide_len, 2 * _S_SLIDE_HW, _S_SLIDE_H)
    s = s.edges("|X and >Z").chamfer(0.004)

    # Hollow bore at the muzzle (0.015 m behind the front face).
    bore_world_x = slide_x1 - 0.015
    bore = (
        cq.Workplane("XY")
        .cylinder(0.050, _S_BORE_R)
        .rotate((0, 0, 0), (0, 1, 0), 90)
        .translate((bore_world_x - slide_mid_x, 0.0, _S_BORE_Z - (_S_SLIDE_Z0 + _S_SLIDE_H / 2.0)))
    )
    s = s.cut(bore)

    # Ejection port (world x ~ 0.035).
    port = (
        cq.Workplane("XY")
        .box(0.046, 0.015, 0.017)
        .translate((0.035 - slide_mid_x, -0.0105, 0.011 - 0.016))
    )
    s = s.cut(port)

    if r.sights == "pistol_optic_cut":
        # Optic-ready milled pocket on the top rear deck (pistoptic L148-169).
        pocket_depth = 0.008
        pocket_z = _S_SLIDE_H / 2.0 - pocket_depth / 2.0
        pocket = (
            cq.Workplane("XY").box(0.060, 0.025, pocket_depth).translate((-0.070, 0.0, pocket_z))
        )
        s = s.cut(pocket)
        hole_depth = 0.005
        hole_z = _S_SLIDE_H / 2.0 - pocket_depth - hole_depth / 2.0
        for dx in (-0.018, 0.018):
            hole = (
                cq.Workplane("XY").cylinder(hole_depth, 0.0015).translate((-0.070 + dx, 0.0, hole_z))
            )
            s = s.cut(hole)

    return s.translate((slide_mid_x, 0.0, _S_SLIDE_H / 2.0))


def _build_trigger_solid() -> cq.Workplane:
    tr = (
        cq.Workplane("XZ")
        .moveTo(-0.0045, 0.002)
        .lineTo(0.0045, 0.002)
        .threePointArc((0.0078, -0.010), (0.0008, -0.0215))
        .lineTo(-0.0030, -0.0190)
        .threePointArc((-0.0026, -0.009), (-0.0045, 0.002))
        .close()
        .extrude(0.004, both=True)
    )
    return tr


def _build_semi_auto(model: ArticulatedObject, r: ResolvedHandgunConfig, assets) -> None:
    # Materials referenced by name (registered in build_handgun).
    _bf, _bg, cx, cz = _s_butt(r)
    slide_x1 = _s_slide_x1(r)
    slide_mid_x = (_S_SLIDE_X0 + slide_x1) / 2.0

    def _mesh(solid, name):
        return mesh_from_cadquery(
            solid, name, assets=assets, tolerance=MESH_TOL, angular_tolerance=MESH_ANG_TOL
        )

    # ----- frame (root) -----
    frame = model.part("frame")
    frame.visual(_mesh(_build_frame_solid(r), "frame_body"), material="body", name="frame_body")
    frame.visual(
        _mesh(_build_rail_solid(r), "accessory_rail"), material="body", name="accessory_rail"
    )
    # grip panels: shortened on compact (pistcompact L196-204).
    if r.grip == "pistol_compact":
        panel_h = 0.036
        panel_cx, panel_cz = -0.068, 0.040
    else:
        panel_h = 0.052
        panel_cx, panel_cz = -0.0766, 0.048
    for tag, sy in (("left", 1.0), ("right", -1.0)):
        frame.visual(
            Box((0.032, 0.003, panel_h)),
            origin=Origin(xyz=(panel_cx, sy * 0.0155, panel_cz), rpy=(0.0, _S_RAKE, 0.0)),
            material="grip",
            name=f"{tag}_grip_panel",
        )
    frame.visual(
        Box((0.024, 0.0025, 0.0045)),
        origin=Origin(xyz=(-0.032, 0.0146, 0.110)),
        material="dark",
        name="slide_stop_lever",
    )

    # ----- slide -----
    slide = model.part("slide")
    slide.visual(_mesh(_build_slide_solid(r), "slide_body"), material="body", name="slide_body")
    # Barrel block visible through ejection port.
    slide.visual(
        Cylinder(radius=0.0085, length=0.054),
        origin=Origin(xyz=(0.035 - slide_mid_x, 0.0, _S_BORE_Z - _S_SLIDE_Z0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="accent",
        name="barrel_block",
    )
    # rear sight position follows the slide heel; front sight at the muzzle.
    front_sight_x = (slide_x1 - 0.008) - slide_mid_x
    if r.sights == "pistol_optic_cut":
        # red-dot optic seated in the milled pocket (pistoptic L245-274).
        slide.visual(
            Box((0.006, 0.0065, 0.004)),
            origin=Origin(xyz=(front_sight_x, 0.0, 0.0335)),
            material="dark",
            name="front_sight",
        )
        slide.visual(
            Box((0.036, 0.022, 0.010)),
            origin=Origin(xyz=(-0.070, 0.0, 0.030)),
            material="dark",
            name="optic_sight_block",
        )
        slide.visual(
            Box((0.018, 0.016, 0.003)),
            origin=Origin(xyz=(-0.072, 0.0, 0.034)),
            material="lens",
            name="optic_lens_window",
        )
        for i in range(2):
            dx = -0.018 + i * 0.036
            slide.visual(
                Cylinder(radius=0.002, length=0.002),
                origin=Origin(xyz=(-0.070 + dx, 0.0, 0.025)),
                material="dark",
                name=f"optic_screw_{i}",
            )
    else:
        slide.visual(
            Box((0.012, 0.020, 0.005)),
            origin=Origin(xyz=(-0.097, 0.0, 0.034)),
            material="dark",
            name="rear_sight",
        )
        slide.visual(
            Box((0.006, 0.0065, 0.004)),
            origin=Origin(xyz=(front_sight_x, 0.0, 0.0335)),
            material="dark",
            name="front_sight",
        )
    # Cocking serrations (always present): rear + front groups, both flanks.
    rear_xs = [-0.094 + i * 0.0046 for i in range(8)]
    front_xs = [(0.062 + i * 0.005) for i in range(6)]
    for group, xs_list in (("rear", rear_xs), ("front", front_xs)):
        for i, sx in enumerate(xs_list):
            if sx > slide_x1 - 0.006:
                continue
            for tag, sy in (("left", 1.0), ("right", -1.0)):
                slide.visual(
                    Box((0.0015, 0.0014, 0.020)),
                    origin=Origin(xyz=(sx, sy * 0.0148, 0.015), rpy=(0.0, 0.21, 0.0)),
                    material="body_mid",
                    name=f"{group}_serration_{i}_{tag}",
                )
    model.articulation(
        "frame_to_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=slide,
        origin=Origin(xyz=(0.0, 0.0, _S_SLIDE_Z0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=2.5, lower=0.0, upper=_S_SLIDE_TRAVEL),
    )

    # ----- trigger -----
    trigger = model.part("trigger")
    trigger.visual(
        _mesh(_build_trigger_solid(), "trigger_blade"), material="dark", name="trigger_blade"
    )
    model.articulation(
        "frame_to_trigger",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=trigger,
        origin=Origin(xyz=_S_TRIG_PIVOT),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=6.0, lower=0.0, upper=_S_TRIG_PULL),
    )

    # ----- takedown lever -----
    lever = model.part("takedown_lever")
    lever.visual(
        Cylinder(radius=0.0075, length=0.006),
        origin=Origin(xyz=(0.0, 0.0015, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="body",
        name="lever_boss",
    )
    lever.visual(
        Cylinder(radius=0.0028, length=0.002),
        origin=Origin(xyz=(0.0, 0.0045, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="dark",
        name="lever_screw",
    )
    lever.visual(
        Box((0.024, 0.003, 0.0075)),
        origin=Origin(xyz=(0.0135, 0.003, 0.0)),
        material="body",
        name="lever_tab",
    )
    model.articulation(
        "frame_to_takedown_lever",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=lever,
        origin=Origin(xyz=_S_TKD_PIVOT),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=0.0, upper=math.pi / 2.0),
    )

    # ----- magazine -----
    magazine = model.part("magazine")
    if r.grip == "pistol_compact":
        magazine.visual(
            Box((0.024, 0.023, 0.050)),
            origin=Origin(xyz=(0.0, 0.0, 0.027)),
            material="grip",
            name="mag_body",
        )
        magazine.visual(
            Box((0.026, 0.028, 0.012)),
            origin=Origin(xyz=(0.0, 0.0, -0.0025)),
            material="dark",
            name="mag_baseplate",
        )
    else:
        magazine.visual(
            Box((0.021, 0.021, 0.076)),
            origin=Origin(xyz=(0.0, 0.0, 0.039)),
            material="grip",
            name="mag_body",
        )
        magazine.visual(
            Box((0.040, 0.030, 0.013)),
            origin=Origin(xyz=(0.0, 0.0, -0.0055)),
            material="dark",
            name="mag_baseplate",
        )
    model.articulation(
        "frame_to_magazine",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=magazine,
        origin=Origin(xyz=(cx, 0.0, cz), rpy=(0.0, _S_RAKE, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.0, lower=0.0, upper=r.mag_travel),
    )


# ===========================================================================
# Top-level builder
# ===========================================================================


def build_handgun(
    config: HandgunConfig | None = None, *, assets: AssetContext | None = None
) -> ArticulatedObject:
    r = resolve_config(config if config is not None else HandgunConfig())
    if assets is None:
        assets = AssetContext(Path(tempfile.mkdtemp(prefix="articraft-handgun-assets-")))
    model = ArticulatedObject(name=f"handgun_{r.action}", assets=assets)
    for mat_name, rgba in r.palette.items():
        model.material(mat_name, rgba=rgba)

    if r.action == "revolver_swingout":
        _build_revolver(model, r, assets)
    else:
        _build_semi_auto(model, r, assets)
    return model


def build_seeded_handgun(seed: int, *, assets: AssetContext | None = None) -> ArticulatedObject:
    return build_handgun(config_from_seed(seed), assets=assets)


# ===========================================================================
# Tests
# ===========================================================================


def _declare_revolver_allowances(ctx: TestContext, model: ArticulatedObject) -> None:
    """Captured press-fits (intentional, element-scoped). Declared BEFORE the
    baseline overlap check runs so the rest-pose press fits are not flagged."""
    frame = model.get_part("frame")
    crane = model.get_part("crane")
    cylinder = model.get_part("cylinder")
    grip = model.get_part("grip")
    ctx.allow_overlap(
        crane, cylinder, elem_a="crane_body", elem_b="cylinder_body",
        reason="crane arbor is a light press fit inside the cylinder center bore",
    )
    ctx.allow_overlap(
        crane, frame, elem_a="crane_body", elem_b="frame_body",
        reason="closed crane block seats 0.5 mm into the frame bottom rail top",
    )
    ctx.allow_overlap(
        frame, grip, elem_a="frame_body", elem_b="grip_body",
        reason="grip front strap is captured against the frame backstrap tang",
    )


def _run_revolver_tests(ctx: TestContext, model: ArticulatedObject, r: ResolvedHandgunConfig) -> None:
    frame = model.get_part("frame")
    crane = model.get_part("crane")
    cylinder = model.get_part("cylinder")
    trigger = model.get_part("trigger")
    hammer = model.get_part("hammer")
    grip = model.get_part("grip")

    crane_swing = model.get_articulation("crane_swing")
    cylinder_spin = model.get_articulation("cylinder_spin")
    hammer_cock = model.get_articulation("hammer_cock")

    # Joint plan.
    ctx.check(
        "cylinder_spin is CONTINUOUS about +X",
        cylinder_spin.articulation_type == ArticulationType.CONTINUOUS
        and cylinder_spin.axis == (1.0, 0.0, 0.0)
        and cylinder_spin.motion_limits is not None
        and cylinder_spin.motion_limits.lower is None,
        details=f"type={cylinder_spin.articulation_type}, axis={cylinder_spin.axis}",
    )
    ctx.check(
        "crane_swing is REVOLUTE about -X with ~45 deg",
        crane_swing.articulation_type == ArticulationType.REVOLUTE
        and crane_swing.axis == (-1.0, 0.0, 0.0)
        and crane_swing.motion_limits is not None
        and 0.52 <= crane_swing.motion_limits.upper <= 0.80,
        details=f"axis={crane_swing.axis}",
    )
    # Multiplicity: chamber liners equal chamber_count; top chamber on bore axis.
    liners = [v for v in cylinder.visuals if v.name and v.name.startswith("chamber_liner_")]
    ctx.check(
        "chamber liner count equals chamber_count",
        len(liners) == r.chamber_count,
        details=f"n={len(liners)} expected={r.chamber_count}",
    )

    # Scale / grounding (relaxed for snub + scales).
    parts = [frame, crane, cylinder, trigger, hammer, grip]
    lo = [math.inf] * 3
    hi = [-math.inf] * 3
    for p in parts:
        aabb = ctx.part_world_aabb(p)
        if aabb is None:
            continue
        for i in range(3):
            lo[i] = min(lo[i], aabb[0][i])
            hi[i] = max(hi[i], aabb[1][i])
    height = hi[2] - lo[2]
    ctx.check("overall height ~0.13-0.15 m", 0.11 <= height <= 0.16, details=f"height={height:.4f}")
    ctx.check("grip butt grounded near z=0", abs(lo[2]) <= 0.006, details=f"zmin={lo[2]:.4f}")

    # Seating relationships.
    ctx.expect_contact(grip, frame, contact_tol=0.0015, name="grip seats against the frame tang")
    ctx.expect_within(cylinder, frame, axes="x", margin=0.001, name="cylinder seated in frame window")
    ctx.expect_overlap(
        crane, cylinder, axes="x", min_overlap=0.012, name="crane arbor retained in the cylinder"
    )

    # Off-axis witness of continuous spin: top liner drops to the bottom.
    liner_rest = ctx.part_element_world_aabb(cylinder, elem="chamber_liner_0")
    with ctx.pose({cylinder_spin: math.pi}):
        liner_spun = ctx.part_element_world_aabb(cylinder, elem="chamber_liner_0")
    if liner_rest is not None and liner_spun is not None:
        rest_z = 0.5 * (liner_rest[0][2] + liner_rest[1][2])
        spun_z = 0.5 * (liner_spun[0][2] + liner_spun[1][2])
        ctx.check(
            "half-turn spin carries the top chamber to the bottom",
            (rest_z - spun_z) > 0.018,
            details=f"rest_z={rest_z:.4f}, spun_z={spun_z:.4f}",
        )

    # Swing-out pose: cylinder swings out to the left, no collision.
    rest_cyl = ctx.part_world_position(cylinder)
    with ctx.pose({crane_swing: _R_CRANE_OPEN}):
        open_cyl = ctx.part_world_position(cylinder)
        ctx.fail_if_parts_overlap_in_current_pose(name="open_crane_no_overlap")
    if rest_cyl is not None and open_cyl is not None:
        ctx.check(
            "open crane swings the cylinder out to the left",
            open_cyl[1] > rest_cyl[1] + 0.015,
            details=f"rest={rest_cyl}, open={open_cyl}",
        )

    # Hammer / trigger rotate.
    rest_hammer = ctx.part_world_aabb(hammer)
    with ctx.pose({hammer_cock: _R_HAMMER_COCK}):
        cocked = ctx.part_world_aabb(hammer)
    if rest_hammer is not None and cocked is not None:
        ctx.check(
            "cocking rotates the hammer spur rearward",
            cocked[0][0] < rest_hammer[0][0] - 0.002,
            details=f"rest={rest_hammer}, cocked={cocked}",
        )


def _declare_semi_auto_allowances(ctx: TestContext, model: ArticulatedObject) -> None:
    """Intentional local embeddings. Declared BEFORE the baseline overlap check."""
    frame = model.get_part("frame")
    trigger = model.get_part("trigger")
    lever = model.get_part("takedown_lever")
    magazine = model.get_part("magazine")
    ctx.allow_overlap(
        frame, trigger, elem_a="frame_body", elem_b="trigger_blade",
        reason="trigger root passes up through the frame slot to its hidden pivot pin",
    )
    ctx.allow_overlap(
        frame, lever, elem_a="frame_body", elem_b="lever_boss",
        reason="takedown lever boss seats into its bore in the frame side wall",
    )
    ctx.allow_overlap(
        frame, magazine, elem_a="frame_body", elem_b="mag_baseplate",
        reason="magazine baseplate seats flush against the grip heel",
    )


def _run_semi_auto_tests(ctx: TestContext, model: ArticulatedObject, r: ResolvedHandgunConfig) -> None:
    frame = model.get_part("frame")
    slide = model.get_part("slide")
    magazine = model.get_part("magazine")

    j_slide = model.get_articulation("frame_to_slide")
    j_trigger = model.get_articulation("frame_to_trigger")
    j_mag = model.get_articulation("frame_to_magazine")

    # Joint plan.
    ctx.check(
        "slide is PRISMATIC about -X with 0.045 m travel",
        j_slide.articulation_type == ArticulationType.PRISMATIC
        and abs(j_slide.axis[0] + 1.0) < 1e-6
        and j_slide.motion_limits is not None
        and abs(j_slide.motion_limits.upper - 0.045) < 1e-9,
        details=f"axis={j_slide.axis}",
    )
    ctx.check(
        "trigger is REVOLUTE about +Y",
        j_trigger.articulation_type == ArticulationType.REVOLUTE
        and abs(j_trigger.axis[1] - 1.0) < 1e-6,
        details=f"axis={j_trigger.axis}",
    )
    ml = j_mag.motion_limits
    ctx.check(
        "magazine is PRISMATIC along the raked grip axis",
        j_mag.articulation_type == ArticulationType.PRISMATIC
        and abs(j_mag.axis[2] + 1.0) < 1e-6
        and abs(j_mag.origin.rpy[1] - _S_RAKE) < 1e-9
        and ml is not None
        and ml.upper > 0.0,
        details=f"axis={j_mag.axis}, rpy={j_mag.origin.rpy}, upper={ml.upper if ml else None}",
    )

    # Scale / grounding.
    slide_aabb = ctx.part_world_aabb(slide)
    ctx.check(
        "slide top + height ~0.15 m",
        slide_aabb is not None and 0.146 <= slide_aabb[1][2] <= 0.162,
        details=f"slide aabb={slide_aabb}",
    )
    mag_aabb = ctx.part_world_aabb(magazine)
    ctx.check(
        "magazine baseplate grounds near z=0",
        mag_aabb is not None and -0.004 <= mag_aabb[0][2] <= 0.006,
        details=f"magazine aabb={mag_aabb}",
    )

    # Assembly relationships.
    ctx.expect_gap(
        slide, frame, axis="z", max_penetration=0.0008, max_gap=0.0020,
        name="slide rides directly on the frame rails",
    )
    ctx.expect_within(
        magazine, frame, axes="x", margin=0.006, name="seated magazine stays in the grip footprint"
    )
    ctx.expect_overlap(
        magazine, frame, axes="z", min_overlap=0.04, name="seated magazine retained in the magwell"
    )

    # Decisive poses.
    rest_slide = ctx.part_world_aabb(slide)
    with ctx.pose({j_slide: _S_SLIDE_TRAVEL}):
        open_slide = ctx.part_world_aabb(slide)
        ctx.fail_if_parts_overlap_in_current_pose(name="retracted_slide_no_overlap")
    if rest_slide is not None and open_slide is not None:
        ctx.check(
            "retracted slide moves ~0.045 m rearward",
            abs((rest_slide[0][0] - open_slide[0][0]) - _S_SLIDE_TRAVEL) < 1e-5,
            details=f"rest={rest_slide}, open={open_slide}",
        )

    with ctx.pose({j_mag: r.mag_travel}):
        dropped_mag = ctx.part_world_aabb(magazine)
    if mag_aabb is not None and dropped_mag is not None:
        ctx.check(
            "magazine drops down and rearward along the raked grip axis",
            dropped_mag[0][2] < mag_aabb[0][2] - 0.04
            and dropped_mag[0][0] < mag_aabb[0][0] - 0.01,
            details=f"rest={mag_aabb}, dropped={dropped_mag}",
        )


def run_handgun_tests(model: ArticulatedObject, config: HandgunConfig) -> TestReport:
    r = resolve_config(config)
    ctx = TestContext(model)

    # Declare intentional press-fit / embed allowances BEFORE the baseline
    # overlap check so the rest-pose captured pins are not flagged.
    if r.action == "revolver_swingout":
        _declare_revolver_allowances(ctx, model)
    else:
        _declare_semi_auto_allowances(ctx, model)

    # Baseline-equivalent checks (deduplicated against the compiler baseline).
    ctx.check_model_valid()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.015)
    ctx.fail_if_joint_mating_has_gap()

    if r.action == "revolver_swingout":
        _run_revolver_tests(ctx, model, r)
    else:
        _run_semi_auto_tests(ctx, model, r)

    return ctx.report()


__all__ = [
    "Action",
    "BarrelLength",
    "Sights",
    "Grip",
    "PaletteStyle",
    "HandgunConfig",
    "ResolvedHandgunConfig",
    "build_handgun",
    "build_seeded_handgun",
    "config_from_seed",
    "resolve_config",
    "slot_choices_for_seed",
    "run_handgun_tests",
]
