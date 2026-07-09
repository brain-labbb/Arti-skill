"""Tactical / military mechanism-knife procedural template (modular).

Identity: a tactical/combat knife whose blade is *controllably deployed or
stowed* by a mechanism (NOT a bare fixed-blade, NOT a balisong, NOT a
spring-automatic). A single grounded ``handle`` (root: gunmetal chamfered
shell + tan grip block + orange accents + spine serration axis) carries one of
three mutually-exclusive deployment mechanisms, each of which keeps >= 1
non-fixed joint:

  Slot A — deployment (decides the part tree + main joint spine):
    * otf_prismatic_slide  : 3 parts (handle/blade/thumb_slider), 2 PRISMATIC
                             joints (blade 0->0.115 +X, slider 0->0.04 +X Mimic
                             of the blade).
    * side_folding_pivot   : 4 parts (handle/main blade/two nested secondary
                             blades), 3 REVOLUTE joints (each blade pivots
                             about Z from the shared front pin).
    * sliding_sheath_fixedblade : 2 parts (handle/sheath), 1 PRISMATIC joint
                             (sheath collar 0->0.05 +X). The blade is a FIXED
                             visual on the handle; the sheath PRISMATIC keeps
                             the object articulated.

  Slot B — blade_profile : tanto / drop_point / dagger / clip_point. Mesh-only
                           (no part/joint change). dagger is symmetric double
                           edged -> serration_count forced to 0.

  Slot D — pommel_clip   : plain / glassbreak / pocketclip. All are FIXED
                           handle visuals (no part / no joint).

  Multiplicity — spine serration notch count N in [2, 20] (a cut loop on the
                 blade; no part, no joint). dagger gates N=0.

Adapted verbatim (no downgrade) from the 10 five-star module sources in
``data/records/`` (see Module Source Index in
``specs_modular_v1/Military_Military_Handtools_Knife.md``): the parent OTF
(``rec_model-a-futuristic-military-otf-out-the-front-kn_...fae188ee``) plus
``rec_military_knife_var_{foldpivot,sheathslide,droppoint,dagger,clippoint,
glassbreak,pocketclip,serr3,serr10}``. cadquery build helpers, part tree,
joint origins/axes/limits and the Mimic coupling are inherited verbatim; only
literal sizes, the named-slot selection and the serration count are made
configurable.

Sliding / captured interfaces (blade-in-channel, blade-pivot-pin,
sheath-over-blade, pommel-captured) are non-mating-face slip fits, so per the
spec they are *grandfathered* (no MatingContract): the articulation origin is
guarded by ``fail_if_articulation_origin_far_from_geometry(0.015)`` and the
captured/sliding overlaps are declared with element-scoped ``allow_overlap``.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Literal

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

__modular__ = True

# --------------------------------------------------------------------------- #
# Slot enums
# --------------------------------------------------------------------------- #
Deployment = Literal[
    "otf_prismatic_slide", "side_folding_pivot", "sliding_sheath_fixedblade"
]
BladeProfile = Literal["tanto", "drop_point", "dagger", "clip_point"]
PommelClip = Literal["plain", "glassbreak", "pocketclip"]
PaletteStyle = Literal[
    "gunmetal_orange",
    "blacked_out",
    "od_green",
    "stonewash",
    "desert_tan",
    "multicam_grip",
]

DEPLOYMENTS: tuple[Deployment, ...] = (
    "otf_prismatic_slide",
    "side_folding_pivot",
    "sliding_sheath_fixedblade",
)
BLADE_PROFILES: tuple[BladeProfile, ...] = (
    "tanto",
    "drop_point",
    "dagger",
    "clip_point",
)
POMMEL_CLIPS: tuple[PommelClip, ...] = ("plain", "glassbreak", "pocketclip")
PALETTE_STYLES: tuple[PaletteStyle, ...] = (
    "gunmetal_orange",
    "blacked_out",
    "od_green",
    "stonewash",
    "desert_tan",
    "multicam_grip",
)

# Per-seed palette: every style declares all seven semantic keys (>= 3 distinct
# materials), each registered and referenced by name on every .visual.
PALETTES: dict[PaletteStyle, dict[str, tuple[float, float, float, float]]] = {
    "gunmetal_orange": {
        "metal": (0.27, 0.29, 0.32, 1.0),
        "dark": (0.15, 0.16, 0.18, 1.0),
        "steel": (0.74, 0.76, 0.78, 1.0),
        "accent": (0.95, 0.45, 0.10, 1.0),
        "grip": (0.56, 0.40, 0.24, 1.0),
        "grip_dark": (0.36, 0.25, 0.15, 1.0),
        "spike": (0.38, 0.38, 0.40, 1.0),
    },
    "blacked_out": {
        "metal": (0.15, 0.16, 0.18, 1.0),
        "dark": (0.08, 0.08, 0.09, 1.0),
        "steel": (0.18, 0.19, 0.21, 1.0),
        "accent": (0.45, 0.10, 0.08, 1.0),
        "grip": (0.10, 0.10, 0.12, 1.0),
        "grip_dark": (0.05, 0.05, 0.06, 1.0),
        "spike": (0.30, 0.30, 0.32, 1.0),
    },
    "od_green": {
        "metal": (0.30, 0.34, 0.22, 1.0),
        "dark": (0.16, 0.18, 0.12, 1.0),
        "steel": (0.55, 0.57, 0.58, 1.0),
        "accent": (0.12, 0.12, 0.13, 1.0),
        "grip": (0.26, 0.30, 0.18, 1.0),
        "grip_dark": (0.16, 0.19, 0.11, 1.0),
        "spike": (0.34, 0.34, 0.30, 1.0),
    },
    "stonewash": {
        "metal": (0.58, 0.60, 0.62, 1.0),
        "dark": (0.34, 0.36, 0.38, 1.0),
        "steel": (0.62, 0.64, 0.66, 1.0),
        "accent": (0.44, 0.46, 0.48, 1.0),
        "grip": (0.45, 0.43, 0.40, 1.0),
        "grip_dark": (0.30, 0.29, 0.27, 1.0),
        "spike": (0.50, 0.50, 0.52, 1.0),
    },
    "desert_tan": {
        "metal": (0.62, 0.52, 0.36, 1.0),
        "dark": (0.30, 0.25, 0.16, 1.0),
        "steel": (0.55, 0.57, 0.58, 1.0),
        "accent": (0.12, 0.12, 0.13, 1.0),
        "grip": (0.70, 0.58, 0.40, 1.0),
        "grip_dark": (0.44, 0.36, 0.24, 1.0),
        "spike": (0.42, 0.40, 0.36, 1.0),
    },
    "multicam_grip": {
        "metal": (0.27, 0.29, 0.32, 1.0),
        "dark": (0.15, 0.16, 0.18, 1.0),
        "steel": (0.74, 0.76, 0.78, 1.0),
        "accent": (0.18, 0.18, 0.16, 1.0),
        "grip": (0.42, 0.40, 0.28, 1.0),
        "grip_dark": (0.28, 0.27, 0.18, 1.0),
        "spike": (0.38, 0.38, 0.40, 1.0),
    },
}


# --------------------------------------------------------------------------- #
# Fixed base dimensions (shared across all 10 sources; world frame, +X = tip).
# --------------------------------------------------------------------------- #
HANDLE_LEN = 0.14
HANDLE_W = 0.03
HANDLE_T = 0.02
HANDLE_CHAMFER = 0.004

GRIP_X0 = 0.0
GRIP_X1 = 0.044
SHELL_X0 = 0.042  # 2 mm overlap seam with the grip block (same part)
SHELL_X1 = HANDLE_LEN

# Internal blade channel (hollow front), open through the front face.
CHANNEL_Y_HALF = 0.0135
CHANNEL_Z0 = 0.0085
CHANNEL_Z1 = 0.0130
CHANNEL_X0 = 0.008

# Recessed top track (OTF thumb-slider groove).
TRACK_X0 = 0.048
TRACK_X1 = 0.110
TRACK_Y_HALF = 0.005
TRACK_FLOOR_Z = 0.017
SLIDER_TRAVEL = 0.040
SLIDER_LEN = 0.018
SLIDER_Y_HALF = 0.0045

# Blade nominal geometry.
BLADE_LEN = 0.126
BLADE_X_REAR = -0.127
BLADE_X_TIP = -0.001
BLADE_Y_HALF = 0.0125
BLADE_T = 0.003
BLADE_TRAVEL = 0.115

# Folding pivot.
PIVOT_X = 0.133
PIVOT_Z = HANDLE_T / 2.0
PIN_RADIUS = 0.003
PIN_EXTRA = 0.002
FOLDING_LAYER_Y = (-0.0025, 0.0, 0.0025)
FOLDING_LAYER_Z = (-0.0040, 0.0, 0.0040)

# Sliding-sheath fixed-blade.
BLADE_Z_BOT = 0.0085
BLADE_CENTER_Z = BLADE_Z_BOT + BLADE_T / 2.0  # 0.01
SHEATH_LEN = 0.080
SHEATH_BORE_Y_HALF = 0.014
SHEATH_BORE_Z_HALF = 0.0025
SHEATH_WALL = 0.0025
SHEATH_OUTER_Y_HALF = SHEATH_BORE_Y_HALF + SHEATH_WALL  # 0.0165
SHEATH_OUTER_Z_HALF = SHEATH_BORE_Z_HALF + SHEATH_WALL  # 0.005
SHEATH_TRAVEL = 0.050
GUARD_THICK = 0.004
GUARD_Y_HALF = HANDLE_W / 2.0 + 0.004  # 0.019, wider than the handle
GUARD_Z_HALF = HANDLE_T / 2.0

# Pocket clip dimensions.
CLIP_THICKNESS = 0.0008
CLIP_STANDOFF = 0.002
CLIP_ANCHOR_X0 = 0.006
CLIP_ANCHOR_LEN = 0.016
CLIP_ANCHOR_W = 0.010
CLIP_BEND_LEN = 0.004
CLIP_ARM_LEN = 0.058
CLIP_ARM_W = 0.008
CLIP_LIP_LEN = 0.004

# Serration multiplicity defaults.
SERRATION_W = 0.005
SERRATION_D = 0.006


# --------------------------------------------------------------------------- #
# Config dataclasses
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class MilitaryKnifeConfig:
    deployment: Deployment = "otf_prismatic_slide"
    blade_profile: BladeProfile = "tanto"
    pommel_clip: PommelClip = "plain"
    palette_style: PaletteStyle = "gunmetal_orange"
    serration_count: int = 6

    blade_len_scale: float = 1.0
    blade_width_scale: float = 1.0
    handle_len_scale: float = 1.0
    handle_thick_scale: float = 1.0
    deploy_travel_scale: float = 1.0
    pivot_x_scale: float = 1.0
    spike_len_scale: float = 1.0
    clip_arm_scale: float = 1.0


@dataclass(frozen=True)
class ResolvedMilitaryKnifeConfig:
    deployment: Deployment
    blade_profile: BladeProfile
    pommel_clip: PommelClip
    palette_style: PaletteStyle
    serration_count: int

    blade_len_scale: float
    blade_width_scale: float
    handle_len_scale: float
    handle_thick_scale: float
    deploy_travel_scale: float
    pivot_x_scale: float
    spike_len_scale: float
    clip_arm_scale: float

    # derived effective dimensions
    handle_len: float
    handle_t: float
    blade_y_half: float
    blade_travel: float
    sheath_travel: float
    pivot_x: float


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(float(v), hi))


def _pick(value, options):
    return value if value in options else options[0]


# --------------------------------------------------------------------------- #
# Sampling
# --------------------------------------------------------------------------- #
def _max_serration_for_profile(profile: BladeProfile) -> int:
    """drop_point / clip_point only have a short rear straight spine segment
    available for notches, so cap N below the tanto maximum."""
    if profile == "dagger":
        return 0
    if profile in ("drop_point", "clip_point"):
        return 10
    return 20


def _sample_serration(rng: random.Random, profile: BladeProfile) -> int:
    if profile == "dagger":
        return 0
    cap = _max_serration_for_profile(profile)
    # per-N weighting: small N high frequency, large N rare.
    buckets = (
        ([n for n in range(3, 9)], 0.70),
        ([2] + [n for n in range(9, 13)], 0.25),
        ([n for n in range(13, 21)], 0.05),
    )
    pools, weights = zip(*buckets)
    pools = [[n for n in pool if 2 <= n <= cap] for pool in pools]
    avail = [(pool, w) for pool, w in zip(pools, weights) if pool]
    if not avail:
        return min(6, cap)
    pool = rng.choices([p for p, _ in avail], weights=[w for _, w in avail])[0]
    return rng.choice(pool)


def config_from_seed(seed: int) -> MilitaryKnifeConfig:
    rng = random.Random(seed)

    deployment: Deployment = rng.choice(DEPLOYMENTS)
    blade_profile: BladeProfile = rng.choice(BLADE_PROFILES)
    pommel_clip: PommelClip = rng.choice(POMMEL_CLIPS)
    palette_style: PaletteStyle = rng.choice(PALETTE_STYLES)

    serration_count = _sample_serration(rng, blade_profile)

    blade_len_scale = rng.uniform(0.88, 1.12)
    blade_width_scale = rng.uniform(0.90, 1.12)
    handle_len_scale = rng.uniform(0.90, 1.10)
    handle_thick_scale = rng.uniform(0.92, 1.10)
    deploy_travel_scale = rng.uniform(0.85, 1.10)
    pivot_x_scale = rng.uniform(0.95, 1.04)
    spike_len_scale = rng.uniform(0.85, 1.20)
    clip_arm_scale = rng.uniform(0.88, 1.12)

    return MilitaryKnifeConfig(
        deployment=deployment,
        blade_profile=blade_profile,
        pommel_clip=pommel_clip,
        palette_style=palette_style,
        serration_count=serration_count,
        blade_len_scale=round(blade_len_scale, 4),
        blade_width_scale=round(blade_width_scale, 4),
        handle_len_scale=round(handle_len_scale, 4),
        handle_thick_scale=round(handle_thick_scale, 4),
        deploy_travel_scale=round(deploy_travel_scale, 4),
        pivot_x_scale=round(pivot_x_scale, 4),
        spike_len_scale=round(spike_len_scale, 4),
        clip_arm_scale=round(clip_arm_scale, 4),
    )


def resolve_config(config: MilitaryKnifeConfig) -> ResolvedMilitaryKnifeConfig:
    deployment = _pick(config.deployment, DEPLOYMENTS)
    blade_profile = _pick(config.blade_profile, BLADE_PROFILES)
    pommel_clip = _pick(config.pommel_clip, POMMEL_CLIPS)
    palette_style = _pick(config.palette_style, PALETTE_STYLES)

    # --- serration gate: dagger -> 0; else clamp to per-profile available range
    if blade_profile == "dagger":
        serration_count = 0
    else:
        cap = _max_serration_for_profile(blade_profile)
        serration_count = int(max(2, min(int(config.serration_count), cap)))

    # --- continuous scales, clamped to declared ranges
    blade_len_scale = _clamp(config.blade_len_scale, 0.88, 1.12)
    blade_width_scale = _clamp(config.blade_width_scale, 0.90, 1.12)
    handle_len_scale = _clamp(config.handle_len_scale, 0.90, 1.10)
    handle_thick_scale = _clamp(config.handle_thick_scale, 0.92, 1.10)
    deploy_travel_scale = _clamp(config.deploy_travel_scale, 0.85, 1.10)
    pivot_x_scale = _clamp(config.pivot_x_scale, 0.95, 1.04)
    spike_len_scale = _clamp(config.spike_len_scale, 0.85, 1.20)
    clip_arm_scale = _clamp(config.clip_arm_scale, 0.88, 1.12)

    # conditional scales only apply to their owning module; neutralize otherwise
    if deployment != "side_folding_pivot":
        pivot_x_scale = 1.0
    if pommel_clip != "glassbreak":
        spike_len_scale = 1.0
    if pommel_clip != "pocketclip":
        clip_arm_scale = 1.0

    handle_len = HANDLE_LEN * handle_len_scale
    handle_t = HANDLE_T * handle_thick_scale

    # --- inequality 1: blade must hide inside the channel (OTF/folding)
    #     BLADE_Y_HALF * scale <= CHANNEL_Y_HALF - 0.0005
    blade_y_half = BLADE_Y_HALF * blade_width_scale
    max_blade_y_half = CHANNEL_Y_HALF - 0.0005
    if deployment in ("otf_prismatic_slide", "side_folding_pivot"):
        if blade_y_half > max_blade_y_half:
            blade_y_half = max_blade_y_half

    blade_len = BLADE_LEN * blade_len_scale

    # --- pivot (folding only)
    pivot_x = PIVOT_X * pivot_x_scale
    # Keep the pivot pin fully inside the shell footprint (the source had the
    # pin 7 mm behind the 0.14 shell front). When handle_len scales down the pin
    # must follow, otherwise its Cylinder floats past the shell front face and
    # reads as a disconnected geometry island. Cap it a few mm inside the front.
    if deployment == "side_folding_pivot":
        pivot_x = min(pivot_x, handle_len - 0.006)

    # --- inequality 4: folding stow — blade folded must not poke past the
    #     handle pocket. Keep the rear tip well inside x=0 instead of allowing
    #     the earlier near-full-length blade to graze through the handle end.
    if deployment == "side_folding_pivot":
        folding_max_len = min(pivot_x - CHANNEL_X0 - 0.010, handle_len * 0.72)
        blade_len = min(blade_len, max(0.075, folding_max_len))

    # --- blade travel (OTF) / sheath travel (sheath)
    blade_travel = BLADE_TRAVEL * deploy_travel_scale
    # inequality 2: OTF deploy keeps >= 8 mm retained insertion overlap.
    if deployment == "otf_prismatic_slide":
        if blade_len - blade_travel < 0.008:
            blade_travel = blade_len - 0.008

    sheath_travel = SHEATH_TRAVEL * deploy_travel_scale
    # inequality 3: sheath must cover past the blade tip. blade tip world-x is
    # HANDLE_LEN + blade_len; the sheath rest front is near the guard; ensure
    # the collar (SHEATH_LEN) plus travel reaches past the tip.
    if deployment == "sliding_sheath_fixedblade":
        guard_front_x = handle_len + GUARD_THICK
        needed = (handle_len + blade_len) - (guard_front_x + SHEATH_LEN)
        if sheath_travel < needed + 0.002:
            sheath_travel = needed + 0.004
        sheath_travel = max(0.030, sheath_travel)

    # recompute the effective blade_len_scale that the chosen blade_len implies
    blade_len_scale_eff = blade_len / BLADE_LEN

    return ResolvedMilitaryKnifeConfig(
        deployment=deployment,
        blade_profile=blade_profile,
        pommel_clip=pommel_clip,
        palette_style=palette_style,
        serration_count=serration_count,
        blade_len_scale=blade_len_scale_eff,
        blade_width_scale=blade_y_half / BLADE_Y_HALF,
        handle_len_scale=handle_len_scale,
        handle_thick_scale=handle_thick_scale,
        deploy_travel_scale=deploy_travel_scale,
        pivot_x_scale=pivot_x_scale,
        spike_len_scale=spike_len_scale,
        clip_arm_scale=clip_arm_scale,
        handle_len=handle_len,
        handle_t=handle_t,
        blade_y_half=blade_y_half,
        blade_travel=blade_travel,
        sheath_travel=sheath_travel,
        pivot_x=pivot_x,
    )


# --------------------------------------------------------------------------- #
# Slot-choice reporting (drives module_topology_diversity)
# --------------------------------------------------------------------------- #
def slot_choices_for_config(
    config: MilitaryKnifeConfig | ResolvedMilitaryKnifeConfig,
) -> tuple[tuple[str, str], ...]:
    r = (
        config
        if isinstance(config, ResolvedMilitaryKnifeConfig)
        else resolve_config(config)
    )
    return (
        ("deployment", r.deployment),
        ("blade_profile", r.blade_profile),
        ("pommel_clip", r.pommel_clip),
        ("serration", f"serr_{r.serration_count}"),
    )


def slot_choices_for_seed(seed: int) -> tuple[tuple[str, str], ...]:
    return slot_choices_for_config(config_from_seed(seed))


# --------------------------------------------------------------------------- #
# cadquery build helpers (adapted verbatim from the 5-star sources)
# --------------------------------------------------------------------------- #
def _chamfered_bar(length: float, cx: float, h_t: float) -> cq.Workplane:
    """Handle bar segment with chamfered long facets, centered at (y=0,
    z=h_t/2), x-center cx. (parent ``_chamfered_bar``.)"""
    bar = cq.Workplane("XY").box(length, HANDLE_W, h_t)
    bar = bar.edges("|X").chamfer(HANDLE_CHAMFER)
    return bar.translate((cx, 0.0, h_t / 2.0))


def _channel_cut(shell_x1: float) -> cq.Workplane:
    """Blade channel void, open through the front face. (parent ``_channel_cut``.)"""
    length = (shell_x1 + 0.01) - CHANNEL_X0
    return (
        cq.Workplane("XY")
        .box(length, 2.0 * CHANNEL_Y_HALF, CHANNEL_Z1 - CHANNEL_Z0)
        .translate(
            (CHANNEL_X0 + length / 2.0, 0.0, (CHANNEL_Z0 + CHANNEL_Z1) / 2.0)
        )
    )


def _folding_side_pocket_cut(r: ResolvedMilitaryKnifeConfig) -> cq.Workplane:
    """Visible side relief for the folding blade pocket.

    The shared channel is an internal blade void. A side-opening folder also
    needs a visible pocket cut through the handle scale so the blade reads as
    rotating into the handle, not through a solid side wall.
    """
    x0 = 0.010 * r.handle_len_scale
    x1 = min(r.handle_len + 0.004, r.pivot_x + 0.010)
    length = max(0.030, x1 - x0)
    # Open the near-side scale into the internal channel while leaving the far
    # scale as a structural bridge, like a simplified liner-lock knife handle.
    depth = HANDLE_W * 0.58
    z_height = max(0.010, r.handle_t * 0.58)
    return (
        cq.Workplane("XY")
        .box(length, depth, z_height)
        .translate(
            (
                x0 + length / 2.0,
                -HANDLE_W / 2.0 + depth / 2.0 - 0.001,
                r.handle_t / 2.0,
            )
        )
    )


def _build_shell_shape(r: ResolvedMilitaryKnifeConfig) -> cq.Workplane:
    """Front metal body. OTF/folding hollow the blade channel + recess a top
    groove/track; sheath keeps the shell solid (fixed full-tang blade)."""
    shell_x1 = r.handle_len
    shell_x0 = SHELL_X0 * r.handle_len_scale
    shell = _chamfered_bar(shell_x1 - shell_x0, (shell_x0 + shell_x1) / 2.0, r.handle_t)
    if r.deployment == "sliding_sheath_fixedblade":
        return shell
    shell = shell.cut(_channel_cut(shell_x1))
    if r.deployment == "otf_prismatic_slide":
        groove = (
            cq.Workplane("XY")
            .box(TRACK_X1 - TRACK_X0, 2.0 * TRACK_Y_HALF, 0.008)
            .translate(((TRACK_X0 + TRACK_X1) / 2.0, 0.0, TRACK_FLOOR_Z + 0.004))
        )
    else:  # side_folding_pivot — decorative jimping groove
        groove = (
            cq.Workplane("XY")
            .box(0.062, 0.010, 0.008)
            .translate((0.079, 0.0, 0.017 + 0.004))
        )
        shell = shell.cut(_folding_side_pocket_cut(r))
    return shell.cut(groove)


def _build_grip_shape(r: ResolvedMilitaryKnifeConfig) -> cq.Workplane:
    """Rear grip block. Hollowed where the stowed blade reaches (OTF/folding),
    solid for the sheath fixed-blade design."""
    grip_x1 = GRIP_X1 * r.handle_len_scale
    grip = _chamfered_bar(grip_x1 - GRIP_X0, (GRIP_X0 + grip_x1) / 2.0, r.handle_t)
    if r.deployment == "sliding_sheath_fixedblade":
        return grip
    grip = grip.cut(_channel_cut(r.handle_len))
    if r.deployment == "side_folding_pivot":
        grip = grip.cut(_folding_side_pocket_cut(r))
    return grip


def _build_blade_local_otf(r: ResolvedMilitaryKnifeConfig) -> cq.Workplane:
    """OTF blade: local frame x=0 at the front slot plane, body extends to -X.
    (parent / droppoint / dagger / clippoint ``_build_blade_shape``.)"""
    by = r.blade_y_half
    x_rear = -(BLADE_LEN * r.blade_len_scale) - 0.001
    x_tip = BLADE_X_TIP
    blade = _profile_extrude(r, x_rear, x_tip, by, z0=0.0)
    return _serrate(r, blade, x_rear, x_tip, by, z_center=BLADE_T / 2.0)


def _build_blade_local_fold(
    r: ResolvedMilitaryKnifeConfig,
    *,
    length_scale: float = 1.0,
    width_scale: float = 1.0,
    secondary: bool = False,
) -> cq.Workplane:
    """Folding blade: pivot at local origin; body extends along -X when closed.

    The folding module uses three narrow, side-by-side blade layers so the
    closed stack fits the handle pocket instead of one overlong blade cutting
    through the bottom/side of the handle.
    """
    by = min(r.blade_y_half * width_scale, HANDLE_W * 0.17)
    blade_len = BLADE_LEN * r.blade_len_scale * length_scale
    if secondary:
        pts = [
            (0.0025, -by * 0.72),
            (-0.009, -by),
            (-(blade_len * 0.82), -by),
            (-blade_len, 0.0),
            (-(blade_len * 0.82), by),
            (-0.009, by),
            (0.0025, by * 0.72),
        ]
    else:
        pts = [
            (0.003, -by * 0.78),
            (-0.010, -by),
            (-(blade_len * 0.76), -by),
            (-blade_len, 0.007),
            (-blade_len + 0.010, by),
            (-0.010, by),
            (0.003, by * 0.78),
        ]
    blade = cq.Workplane("XY").polyline(pts).close().extrude(BLADE_T)
    blade = blade.translate((0.0, 0.0, -BLADE_T / 2.0))
    # Keep serrations only on the main blade; secondary blades are small tools.
    if secondary:
        return blade
    x_rear = -blade_len
    x_tip = 0.0
    return _serrate(r, blade, x_rear, x_tip, by, z_center=0.0, fold=True)


def _build_blade_world_sheath(r: ResolvedMilitaryKnifeConfig) -> cq.Workplane:
    """Fixed full-tang blade in world coords: extends from x=handle_len forward.
    (sheathslide ``_build_blade_shape``.)"""
    by = r.blade_y_half
    x0 = r.handle_len
    x1 = r.handle_len + BLADE_LEN * r.blade_len_scale
    blade = _profile_extrude_world(r, x0, x1, by)
    blade = blade.translate((0.0, 0.0, BLADE_Z_BOT))
    return _serrate(r, blade, x0, x1, by, z_center=BLADE_Z_BOT + BLADE_T / 2.0)


def _profile_extrude(
    r: ResolvedMilitaryKnifeConfig,
    x_rear: float,
    x_tip: float,
    by: float,
    z0: float,
) -> cq.Workplane:
    """Build the chosen blade profile in a local frame where rear<tip, extruded
    +z from 0. Used by the OTF version."""
    profile = r.blade_profile
    if profile == "drop_point":
        # smooth convex spine + gentle belly converge at a centered tip.
        wp = (
            cq.Workplane("XY")
            .moveTo(x_rear, -by)
            .lineTo(x_tip - 0.034, -by)
            .spline(
                [(x_tip - 0.021, -0.011), (x_tip - 0.009, -0.004), (x_tip, 0.0)],
                includeCurrent=True,
            )
            .spline(
                [
                    (x_tip - 0.011, 0.004),
                    (x_tip - 0.027, 0.011),
                    (x_tip - 0.047, by),
                ],
                includeCurrent=True,
            )
            .lineTo(x_rear, by)
            .close()
        )
        return wp.extrude(BLADE_T)
    if profile == "dagger":
        taper_start_x = x_tip - 0.041
        pts = [
            (x_rear, -by),
            (taper_start_x, -by),
            (x_tip, 0.0),
            (taper_start_x, by),
            (x_rear, by),
        ]
        blade = cq.Workplane("XY").polyline(pts).close().extrude(BLADE_T)
        # central spine ridge along y=0 on the top face
        ridge_len = abs(x_rear) - 0.015
        ridge_cx = (x_rear + x_tip + 0.015) / 2.0
        ridge_half_w = 0.0018
        ridge_height = 0.0007
        top_ridge = (
            cq.Workplane("XY")
            .box(ridge_len, 2.0 * ridge_half_w, ridge_height)
            .translate((ridge_cx, 0.0, BLADE_T + ridge_height / 2.0))
        )
        return blade.union(top_ridge)
    if profile == "clip_point":
        blade_len = x_tip - x_rear
        clip_start_x = x_tip - blade_len / 3.0
        n_clip = 8
        clip_pts = []
        for j in range(n_clip + 1):
            t = j / n_clip
            x = x_tip + t * (clip_start_x - x_tip)
            y_linear = -0.002 + t * (by - (-0.002))
            dip = 0.005 * 4.0 * t * (1.0 - t)
            clip_pts.append((x, y_linear - dip))
        wp = (
            cq.Workplane("XY")
            .moveTo(x_rear, -by)
            .lineTo(x_tip - 0.019, -by)
            .lineTo(x_tip, -0.002)
        )
        for pt in clip_pts[1:]:
            wp = wp.lineTo(pt[0], pt[1])
        wp = wp.lineTo(x_rear, by).close()
        return wp.extrude(BLADE_T)
    # tanto (default): straight main edge + angled tanto secondary + clipped tip
    pts = [
        (x_rear, -by),
        (x_tip - 0.029, -by),
        (x_tip, 0.0090),
        (x_tip - 0.015, by),
        (x_rear, by),
    ]
    return cq.Workplane("XY").polyline(pts).close().extrude(BLADE_T)


def _profile_extrude_world(
    r: ResolvedMilitaryKnifeConfig, x0: float, x1: float, by: float
) -> cq.Workplane:
    """Blade profile for the sheath fixed-blade (world coords, rear at x0,
    tip at x1)."""
    profile = r.blade_profile
    if profile == "drop_point":
        wp = (
            cq.Workplane("XY")
            .moveTo(x0, -by)
            .lineTo(x1 - 0.034, -by)
            .spline(
                [(x1 - 0.021, -0.011), (x1 - 0.009, -0.004), (x1, 0.0)],
                includeCurrent=True,
            )
            .spline(
                [(x1 - 0.011, 0.004), (x1 - 0.027, 0.011), (x1 - 0.047, by)],
                includeCurrent=True,
            )
            .lineTo(x0, by)
            .close()
        )
        return wp.extrude(BLADE_T)
    if profile == "dagger":
        taper_start_x = x1 - 0.041
        pts = [
            (x0, -by),
            (taper_start_x, -by),
            (x1, 0.0),
            (taper_start_x, by),
            (x0, by),
        ]
        blade = cq.Workplane("XY").polyline(pts).close().extrude(BLADE_T)
        ridge_len = (x1 - x0) - 0.015
        ridge_cx = (x0 + x1) / 2.0
        ridge_half_w = 0.0018
        ridge_height = 0.0007
        top_ridge = (
            cq.Workplane("XY")
            .box(ridge_len, 2.0 * ridge_half_w, ridge_height)
            .translate((ridge_cx, 0.0, BLADE_T + ridge_height / 2.0))
        )
        return blade.union(top_ridge)
    if profile == "clip_point":
        blade_len = x1 - x0
        clip_start_x = x1 - blade_len / 3.0
        n_clip = 8
        clip_pts = []
        for j in range(n_clip + 1):
            t = j / n_clip
            x = x1 + t * (clip_start_x - x1)
            y_linear = -0.002 + t * (by - (-0.002))
            dip = 0.005 * 4.0 * t * (1.0 - t)
            clip_pts.append((x, y_linear - dip))
        wp = (
            cq.Workplane("XY")
            .moveTo(x0, -by)
            .lineTo(x1 - 0.019, -by)
            .lineTo(x1, -0.002)
        )
        for pt in clip_pts[1:]:
            wp = wp.lineTo(pt[0], pt[1])
        wp = wp.lineTo(x0, by).close()
        return wp.extrude(BLADE_T)
    # tanto
    pts = [
        (x0, -by),
        (x1 - 0.030, -by),
        (x1, 0.0090),
        (x1 - 0.016, by),
        (x0, by),
    ]
    return cq.Workplane("XY").polyline(pts).close().extrude(BLADE_T)


def _serrate(
    r: ResolvedMilitaryKnifeConfig,
    blade: cq.Workplane,
    x_rear: float,
    x_tip: float,
    by: float,
    z_center: float,
    fold: bool = False,
) -> cq.Workplane:
    """Cut N equidistant rectangular notches into the +y spine on the rear
    straight segment. Pure subtraction — no part, no joint. dagger -> N=0."""
    n = r.serration_count
    if n <= 0:
        return blade
    # available rear straight region: keep notches clear of the front
    # curve/clip zone (~ rear 0.55 of the blade for drop/clip, more for tanto).
    blade_len = abs(x_tip - x_rear)
    if r.blade_profile in ("drop_point", "clip_point"):
        straight_frac = 0.45
    else:
        straight_frac = 0.70
    straight_len = blade_len * straight_frac
    # start a little forward of the rear corner
    x_start = x_rear + 0.006
    usable = straight_len - 0.004
    if fold:
        # Folding blades are much narrower to fit the stacked side pocket; keep
        # jimping shallow so the notches don't sever the blade mesh.
        n = min(n, 6)
        notch_w = SERRATION_W * 0.65
        notch_d = min(SERRATION_D * 0.45, by * 0.45)
        notch_y = by + notch_d * 0.35
    else:
        notch_w = SERRATION_W
        notch_d = SERRATION_D
        notch_y = by
    spacing = usable / max(1, n) if n > 1 else 0.0
    spacing = min(spacing, 0.012)
    for i in range(n):
        cx = x_start + i * spacing + spacing * 0.5
        notch = (
            cq.Workplane("XY")
            .box(notch_w, notch_d, BLADE_T * 4.0)
            .translate((cx, notch_y, z_center))
        )
        blade = blade.cut(notch)
    return blade


def _build_slider_shape() -> cq.Workplane:
    """OTF thumb-slider button: base plug + ridged cap. (parent ``_build_slider_shape``.)"""
    base = (
        cq.Workplane("XY")
        .box(SLIDER_LEN, 2.0 * SLIDER_Y_HALF, 0.004)
        .translate((SLIDER_LEN / 2.0, 0.0, 0.002))
    )
    cap = (
        cq.Workplane("XY")
        .box(0.015, 0.008, 0.0025)
        .translate((SLIDER_LEN / 2.0, 0.0, 0.004 + 0.00125))
    )
    button = base.union(cap)
    for i in range(3):
        cx = SLIDER_LEN / 2.0 + (i - 1) * 0.004
        rib = (
            cq.Workplane("XY")
            .box(0.0015, 0.008, 0.001)
            .translate((cx, 0.0, 0.0065 + 0.0005))
        )
        button = button.union(rib)
    return button


def _build_guard_shape(r: ResolvedMilitaryKnifeConfig) -> cq.Workplane:
    """Front guard stop plate (wider than the sheath). (sheathslide ``_build_guard_shape``.)"""
    guard_x = r.handle_len + GUARD_THICK / 2.0
    return (
        cq.Workplane("XY")
        .box(GUARD_THICK, 2.0 * GUARD_Y_HALF, 2.0 * GUARD_Z_HALF)
        .translate((guard_x, 0.0, r.handle_t / 2.0))
    )


def _build_sheath_shape(r: ResolvedMilitaryKnifeConfig) -> cq.Workplane:
    """Rectangular tubular sheath collar sliding over the blade. Local frame:
    origin at rear-face center. (sheathslide ``_build_sheath_shape``.)"""
    outer = (
        cq.Workplane("XY")
        .box(SHEATH_LEN, 2.0 * SHEATH_OUTER_Y_HALF, 2.0 * SHEATH_OUTER_Z_HALF)
        .translate((SHEATH_LEN / 2.0, 0.0, 0.0))
    )
    outer = outer.edges("|X").chamfer(0.001)
    bore = (
        cq.Workplane("XY")
        .box(SHEATH_LEN + 0.002, 2.0 * SHEATH_BORE_Y_HALF, 2.0 * SHEATH_BORE_Z_HALF)
        .translate((SHEATH_LEN / 2.0, 0.0, 0.0))
    )
    sheath = outer.cut(bore)
    tab = (
        cq.Workplane("XY")
        .box(0.008, 2.0 * (SHEATH_OUTER_Y_HALF + 0.002), 0.003)
        .translate((0.004, 0.0, SHEATH_OUTER_Z_HALF + 0.0015))
    )
    sheath = sheath.union(tab)
    for i in range(2):
        sx = 0.015 + i * 0.025
        for sgn in (1.0, -1.0):
            ridge = (
                cq.Workplane("XY")
                .box(0.012, 0.001, 2.0 * SHEATH_OUTER_Z_HALF - 0.002)
                .translate((sx, sgn * (SHEATH_OUTER_Y_HALF + 0.0005), 0.0))
            )
            sheath = sheath.union(ridge)
    return sheath


def _build_glass_breaker(r: ResolvedMilitaryKnifeConfig) -> cq.Workplane:
    """Glass-breaker spike: short cone at the grip rear + mounting collar.
    (glassbreak ``_build_glass_breaker``.)"""
    base_r = 0.004
    tip_r = 0.0004
    length = 0.011 * r.spike_len_scale
    cone = cq.Solid.makeCone(base_r, tip_r, length)
    wp = (
        cq.Workplane("XY")
        .newObject([cone])
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -90.0)
        .translate((0.0, 0.0, r.handle_t / 2.0))
    )
    collar = (
        cq.Workplane("YZ")
        .circle(base_r + 0.001)
        .extrude(0.002)
        .translate((0.0, 0.0, r.handle_t / 2.0))
    )
    return wp.union(collar)


def _build_pocket_clip(r: ResolvedMilitaryKnifeConfig) -> cq.Workplane:
    """Sprung pocket clip on the +Y side face. (pocketclip ``_build_pocket_clip``.)"""
    t = CLIP_THICKNESS
    side_y = HANDLE_W / 2.0
    z_mid = r.handle_t / 2.0
    arm_len = CLIP_ARM_LEN * r.clip_arm_scale
    anchor = (
        cq.Workplane("XY")
        .box(CLIP_ANCHOR_LEN, t, CLIP_ANCHOR_W)
        .translate((CLIP_ANCHOR_X0 + CLIP_ANCHOR_LEN / 2.0, side_y + t / 2.0, z_mid))
    )
    bend = (
        cq.Workplane("XY")
        .box(CLIP_BEND_LEN, t + CLIP_STANDOFF, CLIP_ARM_W)
        .translate(
            (
                CLIP_ANCHOR_X0 + CLIP_ANCHOR_LEN + CLIP_BEND_LEN / 2.0,
                side_y + (t + CLIP_STANDOFF) / 2.0,
                z_mid,
            )
        )
    )
    arm_x0 = CLIP_ANCHOR_X0 + CLIP_ANCHOR_LEN + CLIP_BEND_LEN
    arm = (
        cq.Workplane("XY")
        .box(arm_len, t, CLIP_ARM_W)
        .translate((arm_x0 + arm_len / 2.0, side_y + CLIP_STANDOFF + t / 2.0, z_mid))
    )
    lip = (
        cq.Workplane("XY")
        .box(CLIP_LIP_LEN, CLIP_STANDOFF, CLIP_ARM_W * 0.6)
        .translate(
            (
                arm_x0 + arm_len - CLIP_LIP_LEN / 2.0,
                side_y + CLIP_STANDOFF / 2.0,
                z_mid,
            )
        )
    )
    return anchor.union(bend).union(arm).union(lip)


# --------------------------------------------------------------------------- #
# Handle assembly (shared baseline + pommel/clip handle visuals)
# --------------------------------------------------------------------------- #
def _build_handle(
    handle, r: ResolvedMilitaryKnifeConfig, *, assets: AssetContext | None
) -> None:
    h_t = r.handle_t
    handle.visual(
        mesh_from_cadquery(_build_shell_shape(r), "handle_shell", assets=assets),
        material="metal",
        name="metal_body",
    )
    handle.visual(
        mesh_from_cadquery(_build_grip_shape(r), "handle_grip", assets=assets),
        material="grip",
        name="grip_block",
    )
    # grip ridges (fixed N=4 decorative array)
    for i in range(4):
        cx = 0.010 + i * 0.008
        handle.visual(
            Box((0.0025, 0.020, 0.0011)),
            origin=Origin(xyz=(cx, 0.0, h_t - 0.0003 + 0.00055)),
            material="grip_dark",
            name=f"grip_ridge_{i}",
        )
    # orange accent strips near the front (top + both side facets)
    accent_xs = (0.117 * r.handle_len_scale, 0.127 * r.handle_len_scale)
    for j, sx in enumerate(accent_xs):
        handle.visual(
            Box((0.005, 0.020, 0.0011)),
            origin=Origin(xyz=(sx, 0.0, h_t - 0.0003 + 0.00055)),
            material="accent",
            name=f"accent_top_{j}",
        )
        for side, sgn in (("left", 1.0), ("right", -1.0)):
            if r.deployment == "side_folding_pivot" and side == "right":
                continue
            handle.visual(
                Box((0.005, 0.0011, 0.010)),
                origin=Origin(
                    xyz=(sx, sgn * (HANDLE_W / 2.0 - 0.0003 + 0.00055), h_t / 2.0)
                ),
                material="accent",
                name=f"accent_{side}_{j}",
            )

    # ---- sheath deployment: fixed full-tang blade + guard are handle visuals
    if r.deployment == "sliding_sheath_fixedblade":
        handle.visual(
            mesh_from_cadquery(_build_blade_world_sheath(r), "fixed_blade", assets=assets),
            material="steel",
            name="tanto_blade",
        )
        handle.visual(
            mesh_from_cadquery(_build_guard_shape(r), "guard_plate", assets=assets),
            material="dark",
            name="guard",
        )

    # ---- folding deployment: pivot pin is a handle visual
    if r.deployment == "side_folding_pivot":
        pin_len = h_t + PIN_EXTRA
        handle.visual(
            Cylinder(radius=PIN_RADIUS, length=pin_len),
            origin=Origin(xyz=(r.pivot_x, 0.0, PIVOT_Z + PIN_EXTRA / 2.0)),
            material="dark",
            name="pivot_pin",
        )

    # ---- pommel / clip handle visuals
    if r.pommel_clip == "glassbreak":
        handle.visual(
            mesh_from_cadquery(_build_glass_breaker(r), "glass_breaker", assets=assets),
            material="spike",
            name="glass_breaker_spike",
        )
    elif r.pommel_clip == "pocketclip":
        handle.visual(
            mesh_from_cadquery(_build_pocket_clip(r), "pocket_clip", assets=assets),
            material="dark",
            name="pocket_clip",
        )


# --------------------------------------------------------------------------- #
# Builder
# --------------------------------------------------------------------------- #
def build_military_knife(
    config: MilitaryKnifeConfig,
    *,
    assets: AssetContext | None = None,
) -> ArticulatedObject:
    r = resolve_config(config)
    model = ArticulatedObject(name="military_knife", assets=assets)

    palette = PALETTES[r.palette_style]
    for key, rgba in palette.items():
        model.material(key, rgba=rgba)

    model.meta["slot_choices"] = slot_choices_for_config(r)

    handle = model.part("handle")
    _build_handle(handle, r, assets=assets)

    if r.deployment == "otf_prismatic_slide":
        blade = model.part("blade")
        blade.visual(
            mesh_from_cadquery(_build_blade_local_otf(r), "blade", assets=assets),
            material="steel",
            name="tanto_blade",
        )
        slider = model.part("thumb_slider")
        slider.visual(
            mesh_from_cadquery(_build_slider_shape(), "thumb_slider", assets=assets),
            material="dark",
            name="slider_button",
        )
        model.articulation(
            "handle_to_blade",
            ArticulationType.PRISMATIC,
            parent=handle,
            child=blade,
            origin=Origin(xyz=(r.handle_len, 0.0, CHANNEL_Z0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=30.0, velocity=1.0, lower=0.0, upper=r.blade_travel
            ),
        )
        slider_travel = SLIDER_TRAVEL * r.deploy_travel_scale
        model.articulation(
            "handle_to_thumb_slider",
            ArticulationType.PRISMATIC,
            parent=handle,
            child=slider,
            origin=Origin(xyz=(0.050, 0.0, TRACK_FLOOR_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=5.0, velocity=0.5, lower=0.0, upper=slider_travel
            ),
            mimic=Mimic(
                joint="handle_to_blade", multiplier=slider_travel / r.blade_travel
            ),
        )

    elif r.deployment == "side_folding_pivot":
        folding_blades = (
            ("blade", "tanto_blade", "handle_to_blade", 1.00, 1.00, FOLDING_LAYER_Y[0], FOLDING_LAYER_Z[0], False),
            ("secondary_blade_1", "secondary_blade_1", "handle_to_secondary_blade_1", 0.74, 0.70, FOLDING_LAYER_Y[1], FOLDING_LAYER_Z[1], True),
            ("secondary_blade_2", "secondary_blade_2", "handle_to_secondary_blade_2", 0.58, 0.58, FOLDING_LAYER_Y[2], FOLDING_LAYER_Z[2], True),
        )
        for part_name, visual_name, joint_name, len_s, wid_s, y_off, z_off, secondary in folding_blades:
            blade_part = model.part(part_name)
            blade_part.visual(
                mesh_from_cadquery(
                    _build_blade_local_fold(
                        r,
                        length_scale=len_s,
                        width_scale=wid_s,
                        secondary=secondary,
                    ),
                    visual_name,
                    assets=assets,
                ),
                material="steel",
                name=visual_name,
            )
            if not secondary:
                blade_part.visual(
                    Cylinder(radius=0.0022, length=0.0035),
                    origin=Origin(
                        xyz=(-0.012, min(r.blade_y_half * 0.55, 0.006), 0.0),
                        rpy=(math.pi / 2.0, 0.0, 0.0),
                    ),
                    material="dark",
                    name="thumb_stud",
                )
            model.articulation(
                joint_name,
                ArticulationType.REVOLUTE,
                parent=handle,
                child=blade_part,
                origin=Origin(xyz=(r.pivot_x, y_off, PIVOT_Z + z_off)),
                axis=(0.0, 0.0, 1.0),
                motion_limits=MotionLimits(
                    effort=10.0, velocity=2.0, lower=0.0, upper=math.pi
                ),
            )

    else:  # sliding_sheath_fixedblade
        sheath = model.part("sheath")
        sheath.visual(
            mesh_from_cadquery(_build_sheath_shape(r), "sheath_collar", assets=assets),
            material="dark",
            name="sheath_tube",
        )
        guard_front_x = r.handle_len + GUARD_THICK
        model.articulation(
            "handle_to_sheath",
            ArticulationType.PRISMATIC,
            parent=handle,
            child=sheath,
            origin=Origin(xyz=(guard_front_x, 0.0, BLADE_CENTER_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=10.0, velocity=0.5, lower=0.0, upper=r.sheath_travel
            ),
        )

    return model


def build_seeded_military_knife(
    seed: int, *, assets: AssetContext | None = None
) -> ArticulatedObject:
    return build_military_knife(config_from_seed(seed), assets=assets)


# --------------------------------------------------------------------------- #
# Author-layer QC
# --------------------------------------------------------------------------- #
def _declare_overlaps(ctx: TestContext, handle, r: ResolvedMilitaryKnifeConfig) -> None:
    """Element-scoped allow_overlap for the captured / sliding interfaces that
    are grandfathered (no MatingContract)."""
    if r.deployment == "otf_prismatic_slide":
        blade = ctx.model.get_part("blade")
        slider = ctx.model.get_part("thumb_slider")
        ctx.allow_overlap(
            handle, blade, elem_a="metal_body", elem_b="tanto_blade",
            reason="OTF blade rides captured inside the hollow front channel of the shell.",
        )
        ctx.allow_overlap(
            handle, blade, elem_a="grip_block", elem_b="tanto_blade",
            reason="Stowed OTF blade reaches into the hollowed rear grip block.",
        )
        ctx.allow_overlap(
            handle, slider, elem_a="metal_body", elem_b="slider_button",
            reason="Thumb slider button seats captured in the recessed top track.",
        )
    elif r.deployment == "side_folding_pivot":
        for part_name, elem_name in (
            ("blade", "tanto_blade"),
            ("secondary_blade_1", "secondary_blade_1"),
            ("secondary_blade_2", "secondary_blade_2"),
        ):
            blade = ctx.model.get_part(part_name)
            ctx.allow_overlap(
                handle, blade, elem_a="pivot_pin", elem_b=elem_name,
                reason="The shared pivot pin passes through the nested folding blade eye.",
            )
            ctx.allow_overlap(
                handle, blade, elem_a="metal_body", elem_b=elem_name,
                reason="Folded blade nests inside the hollow side pocket of the shell.",
            )
            ctx.allow_overlap(
                handle, blade, elem_a="grip_block", elem_b=elem_name,
                reason="Folded blade reaches into the hollowed rear grip pocket.",
            )
        blade = ctx.model.get_part("blade")
        ctx.allow_overlap(
            handle, blade, elem_a="metal_body", elem_b="thumb_stud",
            reason="Thumb stud on the folded main blade spine nests within the handle channel.",
        )
    else:  # sliding_sheath_fixedblade
        sheath = ctx.model.get_part("sheath")
        ctx.allow_overlap(
            handle, sheath, elem_a="tanto_blade", elem_b="sheath_tube",
            reason="Tubular sheath collar slides over the fixed full-tang blade (bore capture).",
        )
        ctx.allow_overlap(
            handle, sheath, elem_a="guard", elem_b="sheath_tube",
            reason="Sheath collar butts against the front guard stop at the rest position.",
        )

    if r.pommel_clip == "glassbreak":
        ctx.allow_overlap(
            handle, handle, elem_a="grip_block", elem_b="glass_breaker_spike",
            reason="Glass-breaker spike collar is captured into the rear grip block.",
        )
    elif r.pommel_clip == "pocketclip":
        ctx.allow_overlap(
            handle, handle, elem_a="metal_body", elem_b="pocket_clip",
            reason="Pocket-clip anchor plate is fastened flush against the handle side face.",
        )
        ctx.allow_overlap(
            handle, handle, elem_a="grip_block", elem_b="pocket_clip",
            reason="Pocket-clip anchor plate is fastened flush against the handle side face.",
        )


def run_military_knife_tests(
    model: ArticulatedObject,
    config: MilitaryKnifeConfig,
) -> TestReport:
    r = resolve_config(config)
    ctx = TestContext(model)

    handle = model.get_part("handle")
    _declare_overlaps(ctx, handle, r)

    # ---- compiler-owned baseline (deduplicated against the auto baseline)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.015)

    # ---- handle scale + grounded on z=0
    hbb = ctx.part_world_aabb(handle)
    ctx.check(
        "handle is a hand-held knife handle, grounded at z=0",
        hbb is not None
        and 0.090 < (hbb[1][0] - hbb[0][0]) < 0.30
        and 0.020 < (hbb[1][1] - hbb[0][1]) < 0.045
        and abs(hbb[0][2]) < 0.0020,
        details=f"handle aabb={hbb}",
    )

    # ---- >= 1 non-fixed joint (the core identity requirement)
    non_fixed = [
        a for a in model.articulations
        if a.articulation_type != ArticulationType.FIXED
    ]
    ctx.check(
        "knife has at least one non-fixed deployment joint",
        len(non_fixed) >= 1,
        details=f"non-fixed joints: {[j.name for j in non_fixed]}",
    )

    # ---- slot_choices recorded and consistent
    ctx.check(
        "slot_choices recorded on meta",
        tuple(model.meta.get("slot_choices", ())) == slot_choices_for_config(r),
        details=str(model.meta.get("slot_choices")),
    )

    # ---- per-deployment topology + motion
    if r.deployment == "otf_prismatic_slide":
        blade = model.get_part("blade")
        slider = model.get_part("thumb_slider")
        bj = model.get_articulation("handle_to_blade")
        sj = model.get_articulation("handle_to_thumb_slider")
        bl = bj.motion_limits
        ctx.check(
            "OTF blade joint PRISMATIC +X with correct travel",
            bj.articulation_type == ArticulationType.PRISMATIC
            and abs(bj.axis[0]) > 0.99
            and bl is not None
            and abs(bl.lower) < 1e-9
            and abs(bl.upper - r.blade_travel) < 1e-6,
            details=f"axis={bj.axis}, limits=({bl.lower},{bl.upper})",
        )
        sl = sj.motion_limits
        ctx.check(
            "OTF slider joint PRISMATIC +X",
            sj.articulation_type == ArticulationType.PRISMATIC
            and abs(sj.axis[0]) > 0.99
            and sl is not None
            and abs(sl.lower) < 1e-9,
            details=f"axis={sj.axis}, limits=({sl.lower},{sl.upper})",
        )
        mim = sj.mimic
        ctx.check(
            "OTF slider mimics the blade slide at reduced ratio",
            mim is not None
            and mim.joint == "handle_to_blade"
            and 0.0 < mim.multiplier < 1.0,
            details=f"mimic={mim}",
        )
        # rest pose: blade stowed inside the handle envelope
        ctx.expect_within(
            blade, handle, axes="y", margin=0.0,
            name="OTF blade clears the channel side walls when stowed",
        )
        # deployed pose advances the blade out the front (+X)
        rest = ctx.part_world_position(blade)
        with ctx.pose({bj: r.blade_travel}):
            ctx.fail_if_isolated_parts(name="otf_deployed_no_floating")
            ext = ctx.part_world_position(blade)
        ctx.check(
            "OTF positive travel deploys the blade out the front (+X)",
            rest is not None and ext is not None
            and ext[0] - rest[0] > r.blade_travel - 1e-3,
            details=f"rest={rest}, ext={ext}",
        )
        rest_s = ctx.part_world_position(slider)
        with ctx.pose({bj: r.blade_travel}):
            ext_s = ctx.part_world_position(slider)
        ctx.check(
            "OTF mimic drives the slider forward as the blade deploys",
            rest_s is not None and ext_s is not None
            and ext_s[0] - rest_s[0] > 0.01,
            details=f"rest={rest_s}, ext={ext_s}",
        )

    elif r.deployment == "side_folding_pivot":
        fold_joints = [
            model.get_articulation("handle_to_blade"),
            model.get_articulation("handle_to_secondary_blade_1"),
            model.get_articulation("handle_to_secondary_blade_2"),
        ]
        ctx.check(
            "folding deployment has three independently hinged blades",
            {j.name for j in non_fixed}
            == {
                "handle_to_blade",
                "handle_to_secondary_blade_1",
                "handle_to_secondary_blade_2",
            },
            details=f"non-fixed: {[j.name for j in non_fixed]}",
        )
        for idx, bj in enumerate(fold_joints):
            bl = bj.motion_limits
            ctx.check(
                f"folding blade {idx} joint REVOLUTE about Z, 0..pi",
                bj.articulation_type == ArticulationType.REVOLUTE
                and abs(bj.axis[2]) > 0.99
                and bl is not None
                and abs(bl.lower) < 1e-9
                and abs(bl.upper - math.pi) < 1e-6,
                details=f"axis={bj.axis}, limits=({bl.lower},{bl.upper})",
            )
        for part_name, bj in (
            ("blade", fold_joints[0]),
            ("secondary_blade_1", fold_joints[1]),
            ("secondary_blade_2", fold_joints[2]),
        ):
            blade = model.get_part(part_name)
            rest_bb = ctx.part_world_aabb(blade)
            with ctx.pose({bj: math.pi}):
                ctx.fail_if_isolated_parts(name=f"{part_name}_open_no_floating")
                open_bb = ctx.part_world_aabb(blade)
            ctx.check(
                f"positive q swings {part_name} forward (tip to +X)",
                rest_bb is not None and open_bb is not None
                and open_bb[1][0] > rest_bb[1][0] + 0.025,
                details=(
                    f"rest_max_x={rest_bb[1][0]:.4f}, "
                    f"open_max_x={open_bb[1][0]:.4f}"
                ),
            )

    else:  # sliding_sheath_fixedblade
        sheath = model.get_part("sheath")
        sj = model.get_articulation("handle_to_sheath")
        sl = sj.motion_limits
        ctx.check(
            "sheath joint PRISMATIC +X with positive travel",
            sj.articulation_type == ArticulationType.PRISMATIC
            and abs(sj.axis[0]) > 0.99
            and sl is not None
            and abs(sl.lower) < 1e-9
            and sl.upper > 0.02,
            details=f"axis={sj.axis}, limits=({sl.lower},{sl.upper})",
        )
        part_names = {p.name for p in model.parts}
        ctx.check(
            "sheath design has no separate blade/thumb_slider part (fixed blade)",
            "blade" not in part_names and "thumb_slider" not in part_names,
            details=f"parts={sorted(part_names)}",
        )
        ctx.check(
            "fixed full-tang blade is a handle visual",
            handle.get_visual("tanto_blade") is not None,
            details="tanto_blade handle visual present",
        )
        rest_s = ctx.part_world_position(sheath)
        with ctx.pose({sj: sl.upper}):
            ctx.fail_if_isolated_parts(name="sheath_fwd_no_floating")
            ext_s = ctx.part_world_position(sheath)
        ctx.check(
            "positive sheath travel pushes the collar forward (+X) over the blade",
            rest_s is not None and ext_s is not None
            and ext_s[0] - rest_s[0] > sl.upper - 1e-3,
            details=f"rest={rest_s}, ext={ext_s}",
        )

    # ---- blade profile / serration gate
    if r.blade_profile == "dagger":
        ctx.check(
            "dagger profile carries no spine serrations",
            r.serration_count == 0,
            details=f"serration_count={r.serration_count}",
        )

    # ---- palette: >= 3 distinct materials, all referenced
    mat_names = {m.name for m in model.materials}
    ctx.check(
        "per-seed palette has >= 3 distinct materials",
        len(mat_names) >= 3 and {"metal", "steel", "grip", "accent"} <= mat_names,
        details=f"materials={sorted(mat_names)}",
    )
    accents = [v for v in handle.visuals if v.name and v.name.startswith("accent_")]
    ctx.check(
        "orange accent strips near the handle front",
        len(accents) >= 4 and all(v.origin.xyz[0] > 0.09 for v in accents),
        details=f"accent count={len(accents)}",
    )

    return ctx.report()


__all__ = [
    "Deployment",
    "BladeProfile",
    "PommelClip",
    "PaletteStyle",
    "MilitaryKnifeConfig",
    "ResolvedMilitaryKnifeConfig",
    "build_military_knife",
    "build_seeded_military_knife",
    "config_from_seed",
    "resolve_config",
    "slot_choices_for_config",
    "slot_choices_for_seed",
    "run_military_knife_tests",
]
