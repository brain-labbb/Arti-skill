"""Decorative Japanese katana display set (modular template).

A rack (or wall panel) holding N sheathed katanas. Each katana keeps a FIXED
``saya_mount`` to the rack and a PRISMATIC ``blade_draw`` that slides the blade
assembly (blade + habaki + tsuba + fuchi + tsuka + kashira) out of its hollow
white scabbard along the sword long axis, travel 0 .. blade_travel.

Slots (all derived from the 10 five-star katana sources):

- Slot A ``stand_style`` (root spine): ``tiered_crescent_rack`` (base box +
  pillar crescent cradles, parent), ``vertical_post`` (base box + U-channel
  posts, near-vertical swords with a 2 deg lean), ``wall_mount`` (flat wall
  panel + L-bracket hooks, no base box, swords float above ground).
- Slot B ``tsuba`` (guard plate on the blade): ``round_disc`` (red disc +
  gold rim, parent), ``square_mokko`` (mokko four-lobe plate + square rim),
  ``pierced_sukashi`` (openwork disc, boolean-cut tang/petal/cardinal holes).
- Slot C ``tsuka_wrap`` (grip wrap on the blade): ``pink_diamond_ito`` (pink
  grip + dark diamond lozenges, parent), ``smooth_samegawa`` (bare samegawa
  grip), ``cord_ring_bands`` (pink grip + evenly spaced dark ring bands).

Multiplicity axis: ``n_swords`` in [1, 5]. The parent's hand-tuned 3-key
SWORD_MOUNTS dict is rewritten into a single ``for i in range(n)`` loop with
regular tier spacing (var_n1/n2/n3loop form), generalized to arbitrary N.

World frame: +X is the sword long axis (handles toward +X), +Z up, the kanji
plaque faces -Y (front). Ground plane at z = 0 for the base-box racks; the
wall panel floats with its bottom at z ~= 0.08.
"""

from __future__ import annotations

import random
from dataclasses import dataclass, field
from math import cos, pi, sin

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Part,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

__modular__ = True

# ---------------------------------------------------------------- slot enums
STAND_STYLES: tuple[str, ...] = ("tiered_crescent_rack", "vertical_post", "wall_mount")
TSUBA_STYLES: tuple[str, ...] = ("round_disc", "square_mokko", "pierced_sukashi")
TSUKA_WRAPS: tuple[str, ...] = ("pink_diamond_ito", "smooth_samegawa", "cord_ring_bands")
PALETTE_STYLES: tuple[str, ...] = (
    "black_lacquer_red",
    "black_lacquer_pink",
    "natural_wood_gold",
    "crimson_gold",
    "charcoal_steel",
)

N_MIN, N_MAX = 1, 5

# ---------------------------------------------------------------- fixed geometry constants
SAYA_LEN = 0.73  # scabbard length along X
SAYA_R = 0.019  # scabbard outer radius
BORE_R = 0.0145  # scabbard inner bore radius
BORE_DEPTH = SAYA_LEN - 0.005  # closed tail end keeps a 5 mm cap

# Saya/blade local frame: the FIXED mount joint origin (saya-local 0,0,0) is the
# rack SEAT point on the scabbard body, not the mouth. The mouth sits forward at
# +SEAT_BACK so the joint origin lands on real cradle/post/hook geometry (keeps
# fail_if_articulation_origin_far_from_geometry within tol). All saya/blade X
# coordinates below are written mouth-relative then offset by +SEAT_BACK.
SEAT_BACK = 0.45  # seat point is this far behind the mouth (bare tube, between bands)

BLADE_ROOT_X = -0.005  # blade loft root (near the mouth) in mouth-relative frame

GRIP_X0, GRIP_X1 = 0.015, 0.2555  # wrapped handle span
KASHIRA_X0, KASHIRA_X1 = 0.2545, 0.2665  # dark pommel cap span
GRIP_BAND_WIDTH = 0.006  # cord ring-band width (cord_ring_bands wrap)
GRIP_BAND_COUNT = 12  # cord ring-band count

# tiered_crescent_rack geometry
PILLAR_T = 0.022  # pillar plank thickness along X
PILLAR_X = 0.09  # pillar center offset from stand center
NOTCH_R = 0.021  # cradle crescent radius
CRADLE_Y = -0.0375  # cradle seat center, forward of the pillar column
SEAT_DROP = 0.0025  # saya center sits this far below the notch center
BOX_TOP_Z = 0.12  # base box overall height
RACK_TIER_BASE_Z = BOX_TOP_Z + 0.005  # tier 0 = box-top seat (on the base cap)
RACK_TIER_SPACING = 0.14  # nominal vertical pitch between rack tiers

# vertical_post geometry
POST_W = 0.085  # post width (X)
POST_D = 0.025  # post depth (Y)
POST_H = 0.650  # post body height (Z)
WALL_T = 0.008  # side-wall thickness
BACK_T = 0.008  # back-panel thickness
FOOT_T = 0.008  # foot thickness
LEAN = 0.0349  # radians ~= 2 deg lean from vertical toward +X
PITCH = -(pi / 2.0 - LEAN)  # mount rpy pitch (near-vertical sword)
MOUTH_Z = 0.87  # saya-mouth height in the stand frame (near-vertical)
_BACK_FRONT_Y = POST_D / 2.0 - BACK_T
MOUTH_Y = _BACK_FRONT_Y - SAYA_R + 0.0005  # 0.5 mm seat into post back panel
POST_X_SPAN = 0.20  # nominal full X span the posts occupy
_MID_POST_Z = BOX_TOP_Z + POST_H / 2.0

# wall_mount geometry
PANEL_W = 0.30  # panel width (X)
PANEL_H = 0.58  # panel height (Z)
PANEL_T = 0.018  # panel thickness (Y)
PANEL_BOTTOM_Z = 0.08  # panel bottom above ground (wall-mounted)
HOOK_T = 0.020  # hook thickness along X (extrusion)
ARM_REACH = 0.048  # arm length from cradle center toward panel (+Y local)
ARM_OUTBOARD = 0.028  # arm extension past cradle center toward -Y
ARM_H = 0.028  # arm height (Z extent below cradle center)
HOOK_BACK_T = 0.006  # mounting back-plate thickness along Y
HOOK_BACK_H = 0.058  # back-plate height along Z
RIB_LEN = 0.034  # stiffener rib length along Y
RIB_DROP = 0.016  # rib extends below arm bottom
WALL_CRADLE_Y = -ARM_REACH  # cradle center world Y (saya centerline)
SWORD_MOUTH_X = 0.28  # saya mouth X for the horizontal racks
WALL_TIER_BASE_Z = 0.24  # bottom hook tier height
WALL_TIER_SPACING = 0.14  # nominal vertical pitch between wall tiers
HOOK_X_HALF = 0.09  # half X span of the hook pair

# mokko (square) tsuba
MOKKO_HW = 0.039  # half-width in Y
MOKKO_HH = 0.043  # half-height in Z
MOKKO_T = 0.006  # plate thickness along X
MOKKO_CR = 0.019  # corner fillet radius (creates the 4 lobes)

# blossom / dot decoration spots in saya-local coords: (x, azimuth, big?)
BLOSSOM_SPOTS = ((-0.55, 0.30, True), (-0.36, -0.15, True), (-0.47, 0.65, True))
DOT_SPOTS = ((-0.52, -0.35), (-0.40, 0.45), (-0.61, 0.10))

# ---------------------------------------------------------------- palettes (>= 3 styles)
# Each palette names every material referenced by .visual calls so palette
# diversity flows from per-seed palette -> registered materials -> every visual.
PALETTES: dict[str, dict[str, tuple[float, float, float, float]]] = {
    "black_lacquer_red": {
        "stand": (0.06, 0.05, 0.06, 1.0),
        "gold": (0.80, 0.64, 0.25, 1.0),
        "plaque_white": (0.93, 0.92, 0.88, 1.0),
        "ink": (0.10, 0.09, 0.10, 1.0),
        "saya_white": (0.94, 0.93, 0.94, 1.0),
        "blossom_pink": (0.95, 0.65, 0.76, 1.0),
        "deep_pink": (0.89, 0.45, 0.62, 1.0),
        "grip": (0.92, 0.58, 0.70, 1.0),
        "samegawa": (0.91, 0.88, 0.80, 1.0),
        "tsuba": (0.40, 0.10, 0.12, 1.0),
        "dark_fitting": (0.13, 0.11, 0.13, 1.0),
        "steel": (0.83, 0.85, 0.88, 1.0),
    },
    "black_lacquer_pink": {
        "stand": (0.07, 0.06, 0.08, 1.0),
        "gold": (0.82, 0.66, 0.30, 1.0),
        "plaque_white": (0.94, 0.92, 0.90, 1.0),
        "ink": (0.12, 0.10, 0.12, 1.0),
        "saya_white": (0.95, 0.94, 0.95, 1.0),
        "blossom_pink": (0.97, 0.70, 0.80, 1.0),
        "deep_pink": (0.92, 0.40, 0.62, 1.0),
        "grip": (0.95, 0.55, 0.72, 1.0),
        "samegawa": (0.92, 0.89, 0.82, 1.0),
        "tsuba": (0.55, 0.16, 0.28, 1.0),
        "dark_fitting": (0.14, 0.12, 0.15, 1.0),
        "steel": (0.84, 0.86, 0.89, 1.0),
    },
    "natural_wood_gold": {
        "stand": (0.36, 0.24, 0.14, 1.0),
        "gold": (0.86, 0.70, 0.32, 1.0),
        "plaque_white": (0.90, 0.86, 0.78, 1.0),
        "ink": (0.16, 0.12, 0.08, 1.0),
        "saya_white": (0.90, 0.88, 0.84, 1.0),
        "blossom_pink": (0.92, 0.62, 0.66, 1.0),
        "deep_pink": (0.78, 0.36, 0.40, 1.0),
        "grip": (0.50, 0.30, 0.18, 1.0),
        "samegawa": (0.88, 0.84, 0.74, 1.0),
        "tsuba": (0.42, 0.30, 0.14, 1.0),
        "dark_fitting": (0.20, 0.15, 0.10, 1.0),
        "steel": (0.82, 0.84, 0.86, 1.0),
    },
    "crimson_gold": {
        "stand": (0.32, 0.06, 0.08, 1.0),
        "gold": (0.88, 0.72, 0.34, 1.0),
        "plaque_white": (0.95, 0.92, 0.86, 1.0),
        "ink": (0.14, 0.06, 0.06, 1.0),
        "saya_white": (0.93, 0.90, 0.90, 1.0),
        "blossom_pink": (0.96, 0.66, 0.70, 1.0),
        "deep_pink": (0.86, 0.30, 0.36, 1.0),
        "grip": (0.62, 0.12, 0.16, 1.0),
        "samegawa": (0.90, 0.86, 0.78, 1.0),
        "tsuba": (0.50, 0.10, 0.12, 1.0),
        "dark_fitting": (0.16, 0.10, 0.10, 1.0),
        "steel": (0.85, 0.86, 0.88, 1.0),
    },
    "charcoal_steel": {
        "stand": (0.14, 0.15, 0.17, 1.0),
        "gold": (0.62, 0.64, 0.66, 1.0),
        "plaque_white": (0.82, 0.84, 0.86, 1.0),
        "ink": (0.08, 0.09, 0.10, 1.0),
        "saya_white": (0.86, 0.88, 0.90, 1.0),
        "blossom_pink": (0.74, 0.78, 0.82, 1.0),
        "deep_pink": (0.46, 0.50, 0.56, 1.0),
        "grip": (0.30, 0.33, 0.37, 1.0),
        "samegawa": (0.80, 0.82, 0.84, 1.0),
        "tsuba": (0.36, 0.38, 0.42, 1.0),
        "dark_fitting": (0.10, 0.11, 0.13, 1.0),
        "steel": (0.80, 0.83, 0.86, 1.0),
    },
}


# ---------------------------------------------------------------- config
@dataclass(frozen=True)
class KatanaConfig:
    stand_style: str | None = None
    tsuba: str | None = None
    tsuka_wrap: str | None = None
    palette_style: str | None = None
    n_swords: int = 3
    tier_spacing_scale: float = 1.0
    saya_length_scale: float = 1.0
    tsuba_diameter_scale: float = 1.0
    stand_width_scale: float = 1.0
    name: str = "seeded_katana"
    palette: dict[str, tuple[float, float, float, float]] = field(default_factory=dict)


@dataclass(frozen=True)
class ResolvedKatanaConfig:
    stand_style: str
    tsuba: str
    tsuka_wrap: str
    palette_style: str
    n_swords: int
    tier_spacing_scale: float
    saya_length_scale: float
    tsuba_diameter_scale: float
    stand_width_scale: float
    saya_len: float
    blade_travel: float
    name: str
    palette: dict[str, tuple[float, float, float, float]]


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _require(value: str, allowed: tuple[str, ...], *, field_name: str) -> str:
    if value not in allowed:
        raise ValueError(f"unknown {field_name}={value!r}; expected one of {allowed}")
    return value


# ---------------------------------------------------------------- procedural sampling
# Per-N weights over [1,5]: mode at the canonical 3-sword set, tail downweighted.
_N_WEIGHTS = {1: 0.18, 2: 0.27, 3: 0.33, 4: 0.15, 5: 0.07}

# Sparse reviewer-selected regression overrides (<=4 seeds): the curated parent
# (3-sword crescent rack x round disc x pink ito) and the n1/n2/n3loop canonical
# single-axis layouts. These are NOT the main seed domain.
_REGRESSION_OVERRIDES: dict[int, KatanaConfig] = {
    0: KatanaConfig(
        stand_style="tiered_crescent_rack",
        tsuba="round_disc",
        tsuka_wrap="pink_diamond_ito",
        palette_style="black_lacquer_red",
        n_swords=3,
        name="seeded_katana_0",
    ),
    1: KatanaConfig(
        stand_style="tiered_crescent_rack",
        tsuba="round_disc",
        tsuka_wrap="pink_diamond_ito",
        palette_style="black_lacquer_pink",
        n_swords=1,
        name="seeded_katana_1",
    ),
    2: KatanaConfig(
        stand_style="tiered_crescent_rack",
        tsuba="round_disc",
        tsuka_wrap="pink_diamond_ito",
        palette_style="black_lacquer_red",
        n_swords=2,
        name="seeded_katana_2",
    ),
    3: KatanaConfig(
        stand_style="vertical_post",
        tsuba="square_mokko",
        tsuka_wrap="cord_ring_bands",
        palette_style="natural_wood_gold",
        n_swords=3,
        name="seeded_katana_3",
    ),
}


def _draw_n(rng: random.Random) -> int:
    counts = list(_N_WEIGHTS.keys())
    weights = list(_N_WEIGHTS.values())
    return rng.choices(counts, weights=weights, k=1)[0]


def config_from_seed(seed: int) -> KatanaConfig:
    if seed in _REGRESSION_OVERRIDES:
        return _REGRESSION_OVERRIDES[seed]

    rng = random.Random(seed)
    n_swords = _draw_n(rng)
    stand_style = rng.choice(STAND_STYLES)
    tsuba = rng.choice(TSUBA_STYLES)
    tsuka_wrap = rng.choice(TSUKA_WRAPS)
    palette_style = rng.choice(PALETTE_STYLES)

    return KatanaConfig(
        stand_style=stand_style,
        tsuba=tsuba,
        tsuka_wrap=tsuka_wrap,
        palette_style=palette_style,
        n_swords=n_swords,
        tier_spacing_scale=rng.uniform(0.85, 1.20),
        saya_length_scale=rng.uniform(0.90, 1.10),
        tsuba_diameter_scale=rng.uniform(0.95, 1.20),
        stand_width_scale=rng.uniform(0.90, 1.15),
        name=f"seeded_katana_{seed}",
    )


def resolve_config(config: KatanaConfig) -> ResolvedKatanaConfig:
    stand_style = _require(
        config.stand_style or "tiered_crescent_rack", STAND_STYLES, field_name="stand_style"
    )
    tsuba = _require(config.tsuba or "round_disc", TSUBA_STYLES, field_name="tsuba")
    tsuka_wrap = _require(
        config.tsuka_wrap or "pink_diamond_ito", TSUKA_WRAPS, field_name="tsuka_wrap"
    )
    palette_style = _require(
        config.palette_style or "black_lacquer_red", PALETTE_STYLES, field_name="palette_style"
    )

    n_swords = int(_clamp(config.n_swords, N_MIN, N_MAX))
    tier_spacing_scale = _clamp(config.tier_spacing_scale, 0.85, 1.20)
    saya_length_scale = _clamp(config.saya_length_scale, 0.90, 1.10)
    tsuba_diameter_scale = _clamp(config.tsuba_diameter_scale, 0.95, 1.20)
    stand_width_scale = _clamp(config.stand_width_scale, 0.90, 1.15)

    # saya length and blade travel are shape-locked: travel keeps >= 0.010 m of
    # retained insertion at full draw (travel = saya_len - safety margin).
    saya_len = SAYA_LEN * saya_length_scale
    blade_travel = round(_clamp(saya_len - 0.030, 0.30, saya_len - 0.020), 6)

    palette = dict(PALETTES[palette_style])
    if config.palette:
        palette.update(config.palette)

    return ResolvedKatanaConfig(
        stand_style=stand_style,
        tsuba=tsuba,
        tsuka_wrap=tsuka_wrap,
        palette_style=palette_style,
        n_swords=n_swords,
        tier_spacing_scale=tier_spacing_scale,
        saya_length_scale=saya_length_scale,
        tsuba_diameter_scale=tsuba_diameter_scale,
        stand_width_scale=stand_width_scale,
        saya_len=saya_len,
        blade_travel=blade_travel,
        name=config.name,
        palette=palette,
    )


# ---------------------------------------------------------------- cadquery shapes (shared)
def _saya_tube_shape(saya_len: float) -> cq.Workplane:
    """Hollow scabbard tube in the SEAT frame: open mouth at x=+SEAT_BACK,
    closed tail cap at SEAT_BACK-saya_len. The seat point (x=0) is on the body."""
    outer = cq.Workplane("YZ", origin=(SEAT_BACK, 0.0, 0.0)).circle(SAYA_R).extrude(-saya_len)
    bore = cq.Workplane("YZ", origin=(SEAT_BACK, 0.0, 0.0)).circle(BORE_R).extrude(
        -(saya_len - 0.005)
    )
    return outer.cut(bore)


def _ring_shape(r_out: float, r_in: float, length: float) -> cq.Workplane:
    """Annular collar spanning local x in [0, length], axis along +X."""
    outer = cq.Workplane("YZ").circle(r_out).extrude(length)
    inner = cq.Workplane("YZ").circle(r_in).extrude(length)
    return outer.cut(inner)


def _blade_shape(saya_len: float) -> cq.Workplane:
    """Tapered katana blade lofted along -X (root near mouth, tip deep inside).

    Loft offsets scale with saya length so the blade always nests fully inside
    the bore at q=0 (tip stays ~0.005 m inside the closed tail cap).
    """
    span = (saya_len - 0.005) - (-BLADE_ROOT_X)  # available bore length from root
    main = span * (0.600 / 0.715)
    tail = span * (0.115 / 0.715)
    return (
        cq.Workplane("YZ", origin=(BLADE_ROOT_X + SEAT_BACK, 0.0, 0.0))
        .rect(0.0060, 0.0260)
        .workplane(offset=-main)
        .rect(0.0050, 0.0220)
        .workplane(offset=-tail)
        .rect(0.0018, 0.0070)
        .loft(ruled=True)
    )


def _pillar_shape(cradle_zs: tuple[float, ...]) -> cq.Workplane:
    """Stand pillar plank with one forward cradle arm + crescent per cradle tier.

    ``cradle_zs`` are the saya-mouth seat heights of the cradle tiers (i.e. the
    pillar-borne tiers, excluding the box-top tier 0). The plank column spans
    from the box top up to the topmost cradle, so a forward cradle arm and an
    upward crescent notch can be cut at every tier (var_n1/n2 regular-spacing
    generalization of the parent's two fixed crescents).
    """
    if not cradle_zs:
        cradle_zs = (0.32 - SEAT_DROP,)
    z_lo = BOX_TOP_Z
    z_hi = max(cradle_zs) + 0.05
    col_h = z_hi - z_lo
    col_cz = (z_lo + z_hi) / 2.0
    shape = cq.Workplane("YZ").center(0.015, col_cz).rect(0.050, col_h).extrude(PILLAR_T)
    for seat_z in cradle_zs:
        notch_z = seat_z + SEAT_DROP  # crescent center sits SEAT_DROP above the seat
        arm = (
            cq.Workplane("YZ").center(-0.0125, notch_z - 0.03).rect(0.105, 0.060).extrude(PILLAR_T)
        )
        shape = shape.union(arm)
    for seat_z in cradle_zs:
        notch_z = seat_z + SEAT_DROP
        notch = cq.Workplane("YZ").center(CRADLE_Y, notch_z).circle(NOTCH_R).extrude(PILLAR_T)
        shape = shape.cut(notch)
    return shape


def _post_shape() -> cq.Workplane:
    """Upright U-channel post: back panel + two side walls + solid foot."""
    back_cy = POST_D / 2.0 - BACK_T / 2.0
    shape = cq.Workplane("XY").center(0.0, back_cy).rect(POST_W, BACK_T).extrude(POST_H)
    for sign in (-1, 1):
        wall_cx = sign * (POST_W / 2.0 - WALL_T / 2.0)
        wall = cq.Workplane("XY").center(wall_cx, 0.0).rect(WALL_T, POST_D).extrude(POST_H)
        shape = shape.union(wall)
    foot = cq.Workplane("XY").rect(POST_W + 0.012, POST_D + 0.012).extrude(FOOT_T)
    shape = shape.union(foot)
    return shape


def _hook_shape() -> cq.Workplane:
    """L-bracket hook with cradle notch and stiffener rib."""
    arm_y_lo, arm_y_hi = -ARM_OUTBOARD, ARM_REACH
    arm_z_lo, arm_z_hi = -ARM_H, 0.0
    arm = (
        cq.Workplane("YZ")
        .center((arm_y_lo + arm_y_hi) / 2.0, (arm_z_lo + arm_z_hi) / 2.0)
        .rect(arm_y_hi - arm_y_lo, arm_z_hi - arm_z_lo)
        .extrude(HOOK_T)
    )
    back_y_lo, back_y_hi = ARM_REACH - HOOK_BACK_T, ARM_REACH
    back_z_lo = arm_z_lo - 0.005
    back_z_hi = back_z_lo + HOOK_BACK_H
    back = (
        cq.Workplane("YZ")
        .center((back_y_lo + back_y_hi) / 2.0, (back_z_lo + back_z_hi) / 2.0)
        .rect(back_y_hi - back_y_lo, back_z_hi - back_z_lo)
        .extrude(HOOK_T)
    )
    shape = arm.union(back)
    rib_y_hi = ARM_REACH - HOOK_BACK_T
    rib_y_lo = rib_y_hi - RIB_LEN
    rib_z_lo = arm_z_lo - RIB_DROP
    rib_z_hi = arm_z_lo
    rib = (
        cq.Workplane("YZ")
        .center((rib_y_lo + rib_y_hi) / 2.0, (rib_z_lo + rib_z_hi) / 2.0)
        .rect(rib_y_hi - rib_y_lo, rib_z_hi - rib_z_lo)
        .extrude(HOOK_T)
    )
    shape = shape.union(rib)
    notch = cq.Workplane("YZ").center(0.0, 0.0).circle(NOTCH_R).extrude(HOOK_T)
    return shape.cut(notch)


def _mokko_tsuba_shape() -> cq.Workplane:
    """Mokko four-lobed near-square guard plate with center tang slot."""
    plate = cq.Workplane("YZ").rect(MOKKO_HW * 2, MOKKO_HH * 2).extrude(MOKKO_T)
    plate = plate.edges("|X").fillet(MOKKO_CR)
    slot = cq.Workplane("YZ").rect(0.008, 0.025).extrude(MOKKO_T)
    return plate.cut(slot)


def _mokko_tsuba_rim_shape() -> cq.Workplane:
    """Gold rim frame following the mokko tsuba outline."""
    rim_t = 0.004
    hw_out, hh_out = MOKKO_HW + 0.002, MOKKO_HH + 0.002
    hw_in, hh_in = MOKKO_HW - 0.002, MOKKO_HH - 0.002
    cr_out, cr_in = MOKKO_CR + 0.001, MOKKO_CR - 0.001
    outer = cq.Workplane("YZ").rect(hw_out * 2, hh_out * 2).extrude(rim_t).edges("|X").fillet(cr_out)
    inner = cq.Workplane("YZ").rect(hw_in * 2, hh_in * 2).extrude(rim_t).edges("|X").fillet(cr_in)
    return outer.cut(inner)


def _tsuba_sukashi_shape() -> cq.Workplane:
    """Pierced sukashi openwork tsuba (round plate, axis along Z, caller rotates)."""
    t = 0.007
    r = 0.041
    ht = t / 2.0
    overcut = 0.002
    disc = cq.Workplane("XY", origin=(0.0, 0.0, -ht)).circle(r).extrude(t)
    tang = cq.Workplane("XY", origin=(0.0, 0.0, -ht - overcut / 2.0)).rect(0.008, 0.022).extrude(
        t + overcut
    )
    disc = disc.cut(tang)
    for i in range(4):
        ang_deg = i * 90.0 + 45.0
        ang_rad = ang_deg * pi / 180.0
        cx, cy = 0.025 * cos(ang_rad), 0.025 * sin(ang_rad)
        slot = (
            cq.Workplane("XY", origin=(cx, cy, -ht - overcut / 2.0))
            .slot2D(0.018, 0.007, angle=ang_deg)
            .extrude(t + overcut)
        )
        disc = disc.cut(slot)
    for i in range(4):
        ang_rad = i * pi / 2.0
        cx, cy = 0.028 * cos(ang_rad), 0.028 * sin(ang_rad)
        hole = (
            cq.Workplane("XY", origin=(cx, cy, -ht - overcut / 2.0)).circle(0.004).extrude(
                t + overcut
            )
        )
        disc = disc.cut(hole)
    return disc


# ---------------------------------------------------------------- materials
def _register_materials(
    model: ArticulatedObject, palette: dict[str, tuple[float, float, float, float]]
) -> None:
    for name, rgba in palette.items():
        model.material(name, rgba=rgba)


# ---------------------------------------------------------------- station layout
@dataclass(frozen=True)
class _Stations:
    mounts: list[tuple[float, float, float]]  # mount xyz per sword (saya mouth in stand frame)
    rpy: tuple[float, float, float]  # mount rpy (lean for vertical_post)
    near_vertical: bool


def _tier_z_values(base: float, spacing: float, n: int) -> list[float]:
    return [base + i * spacing for i in range(n)]


def _rack_tier_zs(r: ResolvedKatanaConfig) -> list[float]:
    """Saya-mouth seat heights for the crescent rack: tier 0 = box-top seat,
    tiers 1..N-1 = pillar cradle seats at regular spacing."""
    spacing = RACK_TIER_SPACING * r.tier_spacing_scale
    zs = [RACK_TIER_BASE_Z]
    for i in range(1, r.n_swords):
        zs.append((0.32 - SEAT_DROP) + (i - 1) * spacing)
    return zs


def _rack_cradle_zs(r: ResolvedKatanaConfig) -> tuple[float, ...]:
    """Pillar-borne cradle seat heights (tiers 1..N-1; excludes box-top tier 0)."""
    return tuple(_rack_tier_zs(r)[1:])


# minimum X pitch between adjacent posts so neighbouring tsuba guards
# (diameter ~0.082) do not collide (N-fit inequality).
POST_MIN_PITCH = 0.095


def _post_x_positions(r: ResolvedKatanaConfig) -> list[float]:
    n = r.n_swords
    if n == 1:
        return [0.0]
    pitch = max(POST_X_SPAN * r.stand_width_scale / (n - 1), POST_MIN_PITCH)
    span = pitch * (n - 1)
    return [-span / 2.0 + pitch * i for i in range(n)]


# vertical_post: the SEAT point sits on the post channel at this height; the
# mouth then projects up+forward along the leaned saya axis.
POST_SEAT_Z = BOX_TOP_Z + POST_H - 0.10  # near the top of the post body


def _stations_for(r: ResolvedKatanaConfig) -> _Stations:
    n = r.n_swords
    if r.stand_style == "tiered_crescent_rack":
        zs = _rack_tier_zs(r)
        # The FIXED mount origin is the SEAT point on the saya body (x=0 local),
        # placed at the rack center X so it lands inside the pillar/box geometry.
        # The mouth ends up at world x = SEAT_BACK forward of the seat.
        mounts = []
        for i, z in enumerate(zs):
            if i == 0:
                mounts.append((0.0, -0.06, z))  # box-top seat (on base_cap)
            else:
                # anchor the seat ON one pillar plank (x=+PILLAR_X) so the joint
                # origin sits on real cradle geometry; the saya body spans the
                # full X and crosses the other pillar too. Nudge Z down onto the
                # crescent rim material (the seat contact line, out of the notch
                # void) so the origin lands on the pillar surface.
                px = PILLAR_X * r.stand_width_scale
                seat_z = z - (NOTCH_R - SAYA_R) - 0.002
                mounts.append((px, CRADLE_Y, seat_z))
        return _Stations(mounts=mounts, rpy=(0.0, 0.0, 0.0), near_vertical=False)

    if r.stand_style == "vertical_post":
        xs = _post_x_positions(r)
        # Seat just in front of the post back-panel face so the joint origin is
        # within tol of real post material (not the open channel void); mouth
        # projects up the leaned axis.
        seat_y = MOUTH_Y + 0.006
        mounts = [(px, seat_y, POST_SEAT_Z) for px in xs]
        return _Stations(mounts=mounts, rpy=(0.0, PITCH, 0.0), near_vertical=True)

    # wall_mount
    spacing = WALL_TIER_SPACING * r.tier_spacing_scale
    zs = _tier_z_values(WALL_TIER_BASE_Z, spacing, n)
    # Seat ON one hook (x=+HOOK_X_HALF) so the origin sits on real bracket
    # geometry; the saya body spans the full X and crosses the other hook too.
    # Anchor Z at the notch-bottom rim where the hook arm is solid (out of the
    # cradle-notch void) so the joint origin lands on real material.
    mounts = [(HOOK_X_HALF, WALL_CRADLE_Y, z - NOTCH_R + 0.002) for z in zs]
    return _Stations(mounts=mounts, rpy=(0.0, 0.0, 0.0), near_vertical=False)


# ---------------------------------------------------------------- stand builders (Slot A)
def _build_base_box(stand: Part, width_scale: float, *, min_width: float = 0.0) -> None:
    w = max(0.30 * width_scale, min_width)
    plinth_w = w + 0.01
    stand.visual(
        Box((plinth_w, 0.19, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material="stand",
        name="base_plinth",
    )
    stand.visual(
        Box((w, 0.18, 0.100)),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material="stand",
        name="base_body",
    )
    stand.visual(
        Box((plinth_w, 0.19, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.113)),
        material="stand",
        name="base_cap",
    )


def _build_kanji_plaque(stand: Part, *, y_front: float, z_center: float, name_prefix: str) -> None:
    stand.visual(
        Box((0.088, 0.005, 0.058)),
        origin=Origin(xyz=(0.0, y_front, z_center)),
        material="plaque_white",
        name=f"{name_prefix}plaque_panel",
    )
    frame_bars = (
        ("frame_top", Box((0.105, 0.007, 0.009)), (0.0, y_front - 0.002, z_center + 0.033)),
        ("frame_bottom", Box((0.105, 0.007, 0.009)), (0.0, y_front - 0.002, z_center - 0.033)),
        ("frame_left", Box((0.009, 0.007, 0.075)), (-0.048, y_front - 0.002, z_center)),
        ("frame_right", Box((0.009, 0.007, 0.075)), (0.048, y_front - 0.002, z_center)),
    )
    for bar_name, bar_geom, bar_xyz in frame_bars:
        stand.visual(
            bar_geom, origin=Origin(xyz=bar_xyz), material="gold", name=f"{name_prefix}{bar_name}"
        )
    kanji = (
        (Box((0.034, 0.002, 0.0045)), (0.0, y_front - 0.003, z_center + 0.017), (0.0, 0.0, 0.0)),
        (Box((0.046, 0.002, 0.0045)), (0.0, y_front - 0.003, z_center + 0.003), (0.0, 0.0, 0.0)),
        (Box((0.030, 0.002, 0.0045)), (-0.012, y_front - 0.003, z_center - 0.015), (0.0, 0.9, 0.0)),
        (Box((0.030, 0.002, 0.0045)), (0.012, y_front - 0.003, z_center - 0.015), (0.0, -0.9, 0.0)),
    )
    for idx, (geom, xyz, rpy) in enumerate(kanji):
        stand.visual(
            geom, origin=Origin(xyz=xyz, rpy=rpy), material="ink", name=f"{name_prefix}kanji_{idx}"
        )


def _build_tiered_crescent_rack(model: ArticulatedObject, r: ResolvedKatanaConfig) -> Part:
    stand = model.part("display_stand")
    _build_base_box(stand, r.stand_width_scale)
    pillar_mesh = mesh_from_cadquery(_pillar_shape(_rack_cradle_zs(r)), "stand_pillar")
    px_off = PILLAR_X * r.stand_width_scale
    for index, px in enumerate((-px_off, px_off)):
        stand.visual(
            pillar_mesh,
            origin=Origin(xyz=(px - PILLAR_T / 2.0, 0.0, 0.0)),
            material="stand",
            name=f"pillar_{index}",
        )
    _build_kanji_plaque(stand, y_front=-0.0905, z_center=0.065, name_prefix="")
    return stand


def _build_vertical_post(model: ArticulatedObject, r: ResolvedKatanaConfig) -> Part:
    stand = model.part("display_stand")
    xs = _post_x_positions(r)
    # widen the base box so all posts rest fully on the box top.
    box_min_w = (max(abs(x) for x in xs) * 2.0) + POST_W + 0.02
    _build_base_box(stand, r.stand_width_scale, min_width=box_min_w)
    post_mesh = mesh_from_cadquery(_post_shape(), "stand_post")
    for i, px in enumerate(xs):
        stand.visual(
            post_mesh,
            origin=Origin(xyz=(px, 0.0, BOX_TOP_Z)),
            material="stand",
            name=f"post_{i}",
        )
    _build_kanji_plaque(stand, y_front=-0.0905, z_center=0.065, name_prefix="")
    return stand


def _build_wall_mount(model: ArticulatedObject, r: ResolvedKatanaConfig) -> Part:
    panel = model.part("wall_panel")
    panel_w = PANEL_W * r.stand_width_scale

    # Panel height is N-aware: it must cover the plaque below the bottom tier up
    # past the top hook tier (N-fit inequality so no hook floats off the panel).
    spacing = WALL_TIER_SPACING * r.tier_spacing_scale
    zs = _tier_z_values(WALL_TIER_BASE_Z, spacing, r.n_swords)
    top_tier_z = zs[-1]
    plaque_z = PANEL_BOTTOM_Z + 0.055
    panel_h = max(PANEL_H, (top_tier_z + 0.08) - PANEL_BOTTOM_Z)
    panel_cz = PANEL_BOTTOM_Z + panel_h / 2.0
    panel.visual(
        Box((panel_w, PANEL_T, panel_h)),
        origin=Origin(xyz=(0.0, PANEL_T / 2.0, panel_cz)),
        material="stand",
        name="panel_body",
    )
    frame_specs = (
        ("frame_top", Box((panel_w + 0.006, 0.004, 0.010)),
         (0.0, -0.001, PANEL_BOTTOM_Z + panel_h - 0.005)),
        ("frame_bottom", Box((panel_w + 0.006, 0.004, 0.010)),
         (0.0, -0.001, PANEL_BOTTOM_Z + 0.005)),
        ("frame_left", Box((0.010, 0.004, panel_h - 0.010)),
         (-panel_w / 2.0 + 0.005, -0.001, panel_cz)),
        ("frame_right", Box((0.010, 0.004, panel_h - 0.010)),
         (panel_w / 2.0 - 0.005, -0.001, panel_cz)),
    )
    for f_name, f_geom, f_xyz in frame_specs:
        panel.visual(f_geom, origin=Origin(xyz=f_xyz), material="gold", name=f_name)

    _build_kanji_plaque(panel, y_front=-0.001, z_center=plaque_z, name_prefix="")

    # L-bracket hooks: one pair per sword tier, placed at each tier z.
    hook_mesh = mesh_from_cadquery(_hook_shape(), "bracket_hook")
    hook_idx = 0
    for tier_z in zs:
        for hook_x in (-HOOK_X_HALF, HOOK_X_HALF):
            panel.visual(
                hook_mesh,
                origin=Origin(xyz=(hook_x - HOOK_T / 2.0, WALL_CRADLE_Y, tier_z)),
                material="stand",
                name=f"hook_{hook_idx}",
            )
            hook_idx += 1
    return panel


_STAND_BUILDERS = {
    "tiered_crescent_rack": _build_tiered_crescent_rack,
    "vertical_post": _build_vertical_post,
    "wall_mount": _build_wall_mount,
}


# ---------------------------------------------------------------- saya builder (shared)
def _build_saya(model: ArticulatedObject, name: str, meshes: dict, mouth_len: float, band_len: float) -> Part:
    saya = model.part(name)
    saya.visual(meshes["saya"], material="saya_white", name="saya_tube")
    saya.visual(
        meshes["mouth"],
        origin=Origin(xyz=(SEAT_BACK - mouth_len, 0.0, 0.0)),
        material="deep_pink",
        name="mouth_ring",
    )
    saya_len = meshes["saya_len"]
    band_xs = (-saya_len * (0.665 / 0.73), -saya_len * (0.30 / 0.73))
    for index, band_x in enumerate(band_xs):
        saya.visual(
            meshes["band"],
            origin=Origin(xyz=(SEAT_BACK + band_x - band_len / 2.0, 0.0, 0.0)),
            material="deep_pink",
            name=f"band_{index}",
        )
    for index, (bx, az, _big) in enumerate(BLOSSOM_SPOTS):
        bx_scaled = SEAT_BACK + bx * (saya_len / 0.73)
        radial = 0.019
        saya.visual(
            Cylinder(radius=0.0075, length=0.003),
            origin=Origin(xyz=(bx_scaled, -radial * sin(az), radial * cos(az)), rpy=(az, 0.0, 0.0)),
            material="blossom_pink",
            name=f"blossom_{index}",
        )
        saya.visual(
            Cylinder(radius=0.0027, length=0.0036),
            origin=Origin(
                xyz=(bx_scaled, -0.0195 * sin(az), 0.0195 * cos(az)), rpy=(az, 0.0, 0.0)
            ),
            material="gold",
            name=f"blossom_center_{index}",
        )
    for index, (dx, az) in enumerate(DOT_SPOTS):
        dx_scaled = SEAT_BACK + dx * (saya_len / 0.73)
        saya.visual(
            Cylinder(radius=0.004, length=0.003),
            origin=Origin(xyz=(dx_scaled, -0.019 * sin(az), 0.019 * cos(az)), rpy=(az, 0.0, 0.0)),
            material="blossom_pink",
            name=f"petal_dot_{index}",
        )
    return saya


# ---------------------------------------------------------------- blade builder (Slot B + C)
def _build_blade_assembly(
    model: ArticulatedObject,
    name: str,
    meshes: dict,
    r: ResolvedKatanaConfig,
) -> Part:
    # All X coords are written mouth-relative then offset by SEAT_BACK so the
    # blade-assembly local frame matches the saya seat frame.
    dx0 = SEAT_BACK
    blade = model.part(name)
    blade.visual(meshes["blade"], material="steel", name="blade_body")
    blade.visual(
        Box((0.0305, 0.010, 0.024)),
        origin=Origin(xyz=(dx0 - 0.01475, 0.0, 0.0)),
        material="gold",
        name="habaki_collar",
    )

    # --- Slot B: tsuba guard ---
    dia = r.tsuba_diameter_scale
    if r.tsuba == "square_mokko":
        blade.visual(
            meshes["mokko_plate"],
            origin=Origin(xyz=(dx0 + 0.0, 0.0, 0.0)),
            material="tsuba",
            name="tsuba_plate",
        )
        blade.visual(
            meshes["mokko_rim"],
            origin=Origin(xyz=(dx0 - 0.001, 0.0, 0.0)),
            material="gold",
            name="tsuba_rim",
        )
    elif r.tsuba == "pierced_sukashi":
        blade.visual(
            meshes["sukashi"],
            origin=Origin(xyz=(dx0 + 0.0035, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material="tsuba",
            name="tsuba_disc",
        )
        blade.visual(
            meshes["rim"],
            origin=Origin(xyz=(dx0 - 0.001, 0.0, 0.0)),
            material="gold",
            name="tsuba_rim",
        )
    else:  # round_disc
        blade.visual(
            Cylinder(radius=0.041 * dia, length=0.007),
            origin=Origin(xyz=(dx0 + 0.0035, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material="tsuba",
            name="tsuba_disc",
        )
        blade.visual(
            meshes["rim"],
            origin=Origin(xyz=(dx0 - 0.001, 0.0, 0.0)),
            material="gold",
            name="tsuba_rim",
        )

    # fuchi collar spans from the tsuba station back to the grip so it always
    # bridges the guard island to the grip island for every tsuba type.
    fuchi_len = 0.016
    blade.visual(
        Cylinder(radius=0.0160, length=fuchi_len),
        origin=Origin(xyz=(dx0 + 0.0100, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="dark_fitting",
        name="fuchi_collar",
    )

    # --- Slot C: tsuka grip wrap ---
    grip_len = GRIP_X1 - GRIP_X0
    grip_material = "samegawa" if r.tsuka_wrap == "smooth_samegawa" else "grip"
    blade.visual(
        Cylinder(radius=0.0150, length=grip_len),
        origin=Origin(xyz=(dx0 + (GRIP_X0 + GRIP_X1) / 2.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_material,
        name="tsuka_grip",
    )
    blade.visual(
        Cylinder(radius=0.0158, length=KASHIRA_X1 - KASHIRA_X0),
        origin=Origin(
            xyz=(dx0 + (KASHIRA_X0 + KASHIRA_X1) / 2.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)
        ),
        material="dark_fitting",
        name="kashira_pommel",
    )

    if r.tsuka_wrap == "pink_diamond_ito":
        for index, dx in enumerate((0.060, 0.135, 0.210)):
            blade.visual(
                Box((0.016, 0.016, 0.0024)),
                origin=Origin(xyz=(dx0 + dx, 0.0, 0.0150), rpy=(0.0, 0.0, pi / 4.0)),
                material="dark_fitting",
                name=f"wrap_diamond_top_{index}",
            )
        for index, dx in enumerate((0.0975, 0.1725)):
            blade.visual(
                Box((0.016, 0.0024, 0.016)),
                origin=Origin(xyz=(dx0 + dx, -0.0150, 0.0), rpy=(0.0, pi / 4.0, 0.0)),
                material="dark_fitting",
                name=f"wrap_diamond_front_{index}",
            )
    elif r.tsuka_wrap == "cord_ring_bands":
        grip_span = GRIP_X1 - GRIP_X0
        band_margin = 0.012
        band_spacing = (grip_span - 2.0 * band_margin) / (GRIP_BAND_COUNT - 1)
        for i in range(GRIP_BAND_COUNT):
            bx = GRIP_X0 + band_margin + i * band_spacing
            blade.visual(
                meshes["grip_band"],
                origin=Origin(xyz=(dx0 + bx - GRIP_BAND_WIDTH / 2.0, 0.0, 0.0)),
                material="dark_fitting",
                name=f"grip_band_{i}",
            )
    # smooth_samegawa: bare grip, no accents

    return blade


# ---------------------------------------------------------------- shared meshes
def _build_meshes(r: ResolvedKatanaConfig, mouth_len: float, band_len: float, rim_len: float) -> dict:
    saya_len = r.saya_len
    meshes: dict = {
        "saya_len": saya_len,
        "saya": mesh_from_cadquery(_saya_tube_shape(saya_len), "saya_tube"),
        "blade": mesh_from_cadquery(_blade_shape(saya_len), "katana_blade"),
        "band": mesh_from_cadquery(_ring_shape(0.0198, 0.0146, band_len), "saya_band"),
        "mouth": mesh_from_cadquery(_ring_shape(0.0205, 0.0146, mouth_len), "saya_mouth_ring"),
        "rim": mesh_from_cadquery(_ring_shape(0.0435, 0.0375, rim_len), "tsuba_rim"),
    }
    if r.tsuba == "square_mokko":
        meshes["mokko_plate"] = mesh_from_cadquery(_mokko_tsuba_shape(), "mokko_tsuba_plate")
        meshes["mokko_rim"] = mesh_from_cadquery(_mokko_tsuba_rim_shape(), "mokko_tsuba_rim")
    elif r.tsuba == "pierced_sukashi":
        meshes["sukashi"] = mesh_from_cadquery(_tsuba_sukashi_shape(), "tsuba_sukashi")
    if r.tsuka_wrap == "cord_ring_bands":
        meshes["grip_band"] = mesh_from_cadquery(
            _ring_shape(0.0168, 0.0148, GRIP_BAND_WIDTH), "grip_band"
        )
    return meshes


# ---------------------------------------------------------------- top-level builder
def build_katana(
    config: KatanaConfig | None = None,
    *,
    assets: AssetContext | None = None,
) -> ArticulatedObject:
    cfg = config or KatanaConfig()
    r = resolve_config(cfg)
    model = ArticulatedObject(name=r.name, assets=assets)
    _register_materials(model, r.palette)

    stand = _STAND_BUILDERS[r.stand_style](model, r)

    mouth_len = 0.012
    band_len = 0.022
    rim_len = 0.009
    meshes = _build_meshes(r, mouth_len, band_len, rim_len)

    stations = _stations_for(r)
    for i in range(r.n_swords):
        mount_xyz = stations.mounts[i]
        saya = _build_saya(model, f"sword_{i}_saya", meshes, mouth_len, band_len)
        blade = _build_blade_assembly(model, f"sword_{i}_blade_assembly", meshes, r)
        model.articulation(
            f"sword_{i}_saya_mount",
            ArticulationType.FIXED,
            parent=stand,
            child=saya,
            origin=Origin(xyz=mount_xyz, rpy=stations.rpy),
        )
        model.articulation(
            f"sword_{i}_blade_draw",
            ArticulationType.PRISMATIC,
            parent=saya,
            child=blade,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=25.0, velocity=0.6, lower=0.0, upper=r.blade_travel
            ),
        )

    return model


def build_seeded_katana(seed: int, *, assets: AssetContext | None = None) -> ArticulatedObject:
    return build_katana(config_from_seed(seed), assets=assets)


# ---------------------------------------------------------------- slot choices
def slot_choices_for_config(config: KatanaConfig) -> list[tuple[str, str]]:
    r = resolve_config(config)
    return [
        ("stand_style", r.stand_style),
        ("tsuba", r.tsuba),
        ("tsuka_wrap", r.tsuka_wrap),
        ("n_swords", f"n{r.n_swords}"),
    ]


def slot_choices_for_seed(seed: int) -> list[tuple[str, str]]:
    return slot_choices_for_config(config_from_seed(seed))


# ---------------------------------------------------------------- tests
def run_katana_tests(
    object_model: ArticulatedObject,
    config: KatanaConfig,
) -> TestReport:
    r = resolve_config(config)
    ctx = TestContext(object_model)

    root_name = "wall_panel" if r.stand_style == "wall_mount" else "display_stand"
    stand = object_model.get_part(root_name)
    travel = r.blade_travel

    # ----- per-sword: scoped seating allowances + joint plan + sheathed/draw pose
    for i in range(r.n_swords):
        saya = object_model.get_part(f"sword_{i}_saya")
        blade = object_model.get_part(f"sword_{i}_blade_assembly")
        mount = object_model.get_articulation(f"sword_{i}_saya_mount")
        draw = object_model.get_articulation(f"sword_{i}_blade_draw")

        assert mount.articulation_type == ArticulationType.FIXED
        assert draw.articulation_type == ArticulationType.PRISMATIC
        assert draw.axis == (1.0, 0.0, 0.0)
        limits = draw.motion_limits
        assert limits is not None and limits.lower == 0.0
        assert limits.upper is not None and abs(limits.upper - travel) < 1e-9

        # saya seated on the chosen spine: scoped seating overlaps per style.
        # The sheathed sword body (saya_tube + bands and the contained blade)
        # rests across the rack's cradle/post/hook geometry by design.
        if r.stand_style == "tiered_crescent_rack":
            if i == 0:
                # box-top tier: the sword body + bands lie across the base cap.
                for sword_elem in ("saya_tube", "band_0", "band_1", "blade_body"):
                    ctx.allow_overlap(
                        saya if sword_elem != "blade_body" else blade,
                        stand, elem_a=sword_elem, elem_b="base_cap",
                        reason="box-top scabbard/blade rests on the base box top",
                    )
            else:
                for pillar_elem in ("pillar_0", "pillar_1"):
                    for sword_elem in ("saya_tube", "band_0", "band_1"):
                        ctx.allow_overlap(
                            saya, stand, elem_a=sword_elem, elem_b=pillar_elem,
                            reason="scabbard seats into the crescent cradle notch",
                        )
                    ctx.allow_overlap(
                        blade, stand, elem_a="blade_body", elem_b=pillar_elem,
                        reason="sheathed blade rides through the cradle column",
                    )
        elif r.stand_style == "vertical_post":
            for sword_elem in ("saya_tube", "band_0", "band_1"):
                ctx.allow_overlap(
                    saya, stand, elem_a=sword_elem, elem_b=f"post_{i}",
                    reason=f"saya_{i} seats into the post_{i} channel back panel",
                )
            ctx.allow_overlap(
                blade, stand, elem_a="blade_body", elem_b=f"post_{i}",
                reason=f"sheathed blade rides inside the post_{i} channel",
            )
        else:  # wall_mount
            for j in range(2):
                hook_elem = f"hook_{2 * i + j}"
                for sword_elem in ("saya_tube", "band_0", "band_1"):
                    ctx.allow_overlap(
                        saya, stand, elem_a=sword_elem, elem_b=hook_elem,
                        reason=f"saya_{i} seats into the {hook_elem} cradle notch",
                    )
                ctx.allow_overlap(
                    blade, stand, elem_a="blade_body", elem_b=hook_elem,
                    reason=f"sheathed blade rides through the {hook_elem} cradle",
                )
        ctx.expect_contact(saya, stand, name=f"sword_{i}_saya seated on the stand")

        # sheathed insertion: the blade nests inside the hollow saya bore.
        ctx.allow_overlap(
            blade, saya, elem_a="blade_body", elem_b="saya_tube",
            reason="sheathed blade nests inside the hollow saya bore (seated insertion)",
        )

        # sheathed pose at q=0: blade fully hidden, tsuba at the mouth.
        ctx.expect_origin_distance(
            blade, saya, axes="x", min_dist=0.0, max_dist=1e-6,
            name=f"sword_{i} blade fully sheathed at q=0",
        )
        ctx.expect_within(
            blade, saya, axes="yz", inner_elem="blade_body", outer_elem="saya_tube",
            name=f"sword_{i} blade nests inside the saya bore",
        )
        ctx.expect_within(
            blade, saya, axes="x", inner_elem="blade_body", outer_elem="saya_tube",
            margin=0.001, name=f"sword_{i} blade hidden along the saya length",
        )

        # draw pose: blade translates `travel` m along the saya long axis and
        # keeps retained insertion. The translation magnitude is checked in 3D
        # (Euclidean) so it holds whether the sword is horizontal or leaned.
        with ctx.pose({draw: travel}):
            ctx.expect_origin_distance(
                blade, saya, axes="xyz", min_dist=travel - 0.001, max_dist=travel + 0.001,
                name=f"sword_{i} draw moves the blade {travel:.3f} m along its axis",
            )
            ctx.expect_overlap(
                blade, saya, axes="x", elem_a="blade_body", elem_b="saya_tube",
                min_overlap=0.010, name=f"sword_{i} blade stays engaged at full draw",
            )

        # ~1.0 m sheathed katana with the recognizable parts.
        saya_aabb = ctx.part_world_aabb(saya)
        blade_aabb = ctx.part_world_aabb(blade)
        assert saya_aabb is not None and blade_aabb is not None
        if r.stand_style == "vertical_post":
            # near-vertical sword: Z extent dominates.
            total_len = max(saya_aabb[1][2], blade_aabb[1][2]) - min(
                saya_aabb[0][2], blade_aabb[0][2]
            )
        else:
            total_len = max(saya_aabb[1][0], blade_aabb[1][0]) - min(
                saya_aabb[0][0], blade_aabb[0][0]
            )
        assert 0.85 <= total_len <= 1.15, f"sword_{i} katana ~1.0 m, got {total_len:.3f}"

        grip = ctx.part_element_world_aabb(blade, elem="tsuka_grip")
        assert grip is not None

        # tsuba guard wider than the saya (off-axis disc/plate identity).
        if r.tsuba == "square_mokko":
            tsuba = ctx.part_element_world_aabb(blade, elem="tsuba_plate")
        else:
            tsuba = ctx.part_element_world_aabb(blade, elem="tsuba_disc")
        assert tsuba is not None
        tsuba_y = tsuba[1][1] - tsuba[0][1]
        tsuba_z = tsuba[1][2] - tsuba[0][2]
        assert max(tsuba_y, tsuba_z) > 2.2 * SAYA_R, "tsuba guard wider than the saya"

        # decoration present on every saya.
        assert saya.get_visual("blossom_0") is not None
        assert saya.get_visual("band_0") is not None

    # ----- multiplicity: emitted sword count matches resolved n_swords.
    emitted = sum(1 for p in object_model.parts if p.name.endswith("_saya"))
    assert emitted == r.n_swords, f"expected {r.n_swords} swords, got {emitted}"

    # ----- regular tier separation between adjacent stations (rack / wall only).
    if r.stand_style != "vertical_post" and r.n_swords >= 2:
        # tiers are emitted bottom->top (sword 0 lowest), so the higher-indexed
        # sword is the positive (above) link for a positive z gap.
        sayas = [object_model.get_part(f"sword_{i}_saya") for i in range(r.n_swords)]
        for i in range(r.n_swords - 1):
            ctx.expect_origin_gap(
                sayas[i + 1], sayas[i], axis="z",
                min_gap=0.05, max_gap=0.30,
                name=f"sword {i + 1} rides a clear tier above sword {i}",
            )

    # ----- grounding branches on stand_style.
    if r.stand_style == "wall_mount":
        panel_aabb = ctx.part_world_aabb(stand)
        assert panel_aabb is not None
        assert panel_aabb[0][2] >= 0.06, "wall panel floats above ground (wall-mounted)"
        for i in range(r.n_swords):
            saya_aabb = ctx.part_world_aabb(object_model.get_part(f"sword_{i}_saya"))
            assert saya_aabb is not None
            assert saya_aabb[0][2] > 0.10, f"saya_{i} well above ground (wall-mounted)"
    else:
        stand_aabb = ctx.part_world_aabb(stand)
        assert stand_aabb is not None
        assert -1e-6 <= stand_aabb[0][2] <= 0.003, "base box rests on the ground plane"

    return ctx.report()


object_model = build_katana(config_from_seed(0))
