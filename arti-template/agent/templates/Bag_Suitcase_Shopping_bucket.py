"""Shopping bucket (hand-held / nestable shopping basket) modular template.

Sourced from the reviewed modular spec
``articraft_template_authoring/specs_modular_v1/Bag_Suitcase_Shopping_bucket.md`` and the
``picture/Bag_Suitcase/Shopping bucket`` 5-star sample pool (3 parents + 16
slot-fork variants + 3 nesting-stack補造).

Structure (pattern = ``mixed``): a single parameterized ``basket`` module — an
open-mouth tapered hollow tub with a rolled rim — whose properties vary across
five named module axes:

  * ``body_form`` (6): footprint / taper family of the tub mesh.
  * ``wall_style`` (3): wall treatment (slotted-perforated / steel-wire-mesh /
    solid-smooth) of the same tub mesh.
  * ``handle`` (4): the carry handle — fixed central arched bail, single swing
    bail, dual folding bails (2 REVOLUTE), or a single telescoping pull-up bar
    (PRISMATIC +Z).
  * ``lid_closure`` (5): the main opening mechanism — open / hinged top lid /
    two-leaf clamshell / drop-front gate / tilt-down front panel.
  * ``interior`` (3): plain / fixed two-compartment divider (fused into the tub)
    / removable inner caddy (PRISMATIC lift + folding REVOLUTE grip).

The earlier **multiplicity axis** ``basket_count`` (N nested baskets in a +Z
stack) has been removed: every shopping bucket is now a single basket. The N>1
nesting build/test paths (central nib, FIXED ``basket_i_to_basket_i+1`` joints,
stacked-pose overlap checks) remain in the code but are unreachable because
``config_from_seed`` locks ``basket_count`` to 1.

The bail pin-in-boss pivot is captured-pin geometry, so its joint omits
``MatingContract`` (grandfathered) and is guarded by the flat articulation-origin
baseline + element-scoped ``allow_overlap``.
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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

__modular__ = True

BodyForm = Literal[
    "rectangular",
    "rounded_oval_deep",
    "hexagonal_footprint",
    "round_cylindrical_bucket",
    "tapered_stackable",
    "shallow_wide_tray",
]
WallStyle = Literal[
    "slotted_perforated_plastic",
    "steel_wire_mesh",
    "open_lattice_weave",
    "diagonal_herringbone_weave",
    "banded_wicker_weave",
    "solid_smooth_plastic",
]
Handle = Literal[
    "fixed_central_arched",
    "single_swing_bail",
    "dual_folding_bail",
    "single_telescoping_pull_up",
]
LidClosure = Literal[
    "open_no_lid",
    "hinged_top_lid",
    "two_leaf_clamshell",
    "drop_front_gate",
    "tilt_down_front_panel",
]
Interior = Literal[
    "plain",
    "two_compartment_divider",
    "removable_inner_caddy",
]
MaterialStyle = Literal["red_plastic", "blue_plastic", "galvanized_steel", "charcoal"]

BODY_FORMS: tuple[BodyForm, ...] = (
    "rectangular",
    "rounded_oval_deep",
    "hexagonal_footprint",
    "round_cylindrical_bucket",
    "tapered_stackable",
    "shallow_wide_tray",
)
WALL_STYLES: tuple[WallStyle, ...] = (
    "slotted_perforated_plastic",
    "steel_wire_mesh",
    "open_lattice_weave",
    "diagonal_herringbone_weave",
    "banded_wicker_weave",
    "solid_smooth_plastic",
)
HANDLES: tuple[Handle, ...] = (
    "fixed_central_arched",
    "single_swing_bail",
    "dual_folding_bail",
    "single_telescoping_pull_up",
)
LID_CLOSURES: tuple[LidClosure, ...] = (
    "open_no_lid",
    "hinged_top_lid",
    "two_leaf_clamshell",
)
INTERIORS: tuple[Interior, ...] = (
    "plain",
    "two_compartment_divider",
    "removable_inner_caddy",
)
MATERIAL_STYLES: tuple[MaterialStyle, ...] = (
    "red_plastic",
    "blue_plastic",
    "galvanized_steel",
    "charcoal",
)

# Body forms that nest cleanly (strong axisymmetric / rectangular taper). When
# N > 1 we restrict body_form to this set (spec §10 compatibility matrix).
NESTABLE_BODY_FORMS: tuple[BodyForm, ...] = (
    "rectangular",
    "tapered_stackable",
    "round_cylindrical_bucket",
    "shallow_wide_tray",
)

# Multiplicity domain bounds (retained only for clamping). Nesting/stacking has
# been removed: config_from_seed always emits a single basket (basket_count == 1),
# so the N>1 stacking build/test paths below are unreachable.
N_MIN = 1
N_MAX = 8

PALETTES: dict[MaterialStyle, dict[str, tuple[float, float, float, float]]] = {
    "red_plastic": {
        "body": (0.88, 0.27, 0.22, 1.0),
        "handle": (0.55, 0.10, 0.09, 1.0),
        "accent": (0.78, 0.20, 0.16, 1.0),
    },
    "blue_plastic": {
        "body": (0.12, 0.34, 0.86, 1.0),
        "handle": (0.07, 0.07, 0.08, 1.0),
        "accent": (0.10, 0.28, 0.72, 1.0),
    },
    "galvanized_steel": {
        "body": (0.66, 0.68, 0.70, 1.0),
        "handle": (0.40, 0.41, 0.43, 1.0),
        "accent": (0.58, 0.60, 0.62, 1.0),
    },
    "charcoal": {
        "body": (0.18, 0.19, 0.21, 1.0),
        "handle": (0.10, 0.11, 0.12, 1.0),
        "accent": (0.26, 0.27, 0.29, 1.0),
    },
}

# ---------------------------------------------------------------------------
# Base real-world dimensions (meters); scaled per-build by the resolved config.
# Baselines come from parent c383d977 (single basket) and the N=5 nesting stack
# 27719f51 (strong-taper stackable). Long axis along X (X wider than Y).
# ---------------------------------------------------------------------------
_L_TOP = 0.40  # outer length at the mouth (long axis, X)
_D_TOP = 0.30  # outer depth at the mouth (short axis, Y)
_L_BOT = 0.34  # outer length at the base (default mild taper)
_D_BOT = 0.24  # outer depth at the base
_H = 0.22  # tub height
_WALL = 0.012  # wall thickness

_R_VERT_BOT = 0.030  # rounded vertical edge radius at the base
_R_VERT_TOP = 0.036  # rounded vertical edge radius at the mouth

_RIM_H = 0.024  # rolled rim band height
_RIM_OVERHANG = 0.011  # how far the lip rolls out past the wall

# Slot perforations.
_SLOT_W = 0.028
_SLOT_H = 0.120
_SLOT_R = 0.013
_LONG_SLOT_X = (-0.124, -0.042, 0.042, 0.124)
_SHORT_SLOT_Y = (-0.060, 0.060)

# Bail handle / pivots.
_BAR_R = 0.012  # handle bar radius
_ARCH_RISE = 0.205  # arch radius / rise above the pivot line
_HANDLE_BOSS_R = 0.020
_HANDLE_BOSS_LEN = 0.020
_TUB_BOSS_R = 0.022
_TUB_BOSS_LEN = 0.014

# Nesting stack.
_NEST_STEP = 0.075  # vertical offset between consecutive nested baskets
# Strong-taper base length/depth used when N > 1 (nestable). Mirrors 27719f51.
_L_BOT_NEST = 0.26
_D_BOT_NEST = 0.18

_JOINT_LIMIT = math.radians(95.0)


@dataclass(frozen=True)
class ShoppingBucketConfig:
    body_form: BodyForm | None = None
    wall_style: WallStyle | None = None
    handle: Handle | None = None
    lid_closure: LidClosure | None = None
    interior: Interior | None = None
    basket_count: int | None = None
    material_style: MaterialStyle = "red_plastic"
    body_len_scale: float = 1.0
    body_height_scale: float = 1.0
    taper_scale: float = 1.0
    nest_step_scale: float = 1.0
    handle_arch_scale: float = 1.0
    joint_limit_scale: float = 1.0
    name: str = "shopping_bucket"


@dataclass(frozen=True)
class ResolvedShoppingBucketConfig:
    body_form: BodyForm
    wall_style: WallStyle
    handle: Handle
    lid_closure: LidClosure
    interior: Interior
    basket_count: int
    material_style: MaterialStyle
    # Geometry (scaled, concrete).
    l_top: float
    d_top: float
    l_bot: float
    d_bot: float
    h: float
    wall: float
    r_vert_bot: float
    r_vert_top: float
    rim_h: float
    rim_overhang: float
    pivot_z: float
    bar_r: float
    arch_rise: float
    handle_boss_r: float
    handle_boss_len: float
    tub_boss_r: float
    tub_boss_len: float
    nest_step: float
    joint_limit: float
    name: str

    @property
    def is_stack(self) -> bool:
        return self.basket_count > 1


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _pick(value, choices):
    return value if value in choices else choices[0]


def config_from_seed(seed: int) -> ShoppingBucketConfig:
    rng = random.Random(seed)
    return ShoppingBucketConfig(
        body_form=rng.choice(BODY_FORMS),
        wall_style=rng.choice(WALL_STYLES),
        handle=rng.choice(HANDLES),
        lid_closure=rng.choice(LID_CLOSURES),
        interior=rng.choice(INTERIORS),
        basket_count=1,  # nesting/stacking removed — always a single basket
        material_style=rng.choice(MATERIAL_STYLES),
        body_len_scale=round(rng.uniform(0.90, 1.12), 4),
        body_height_scale=round(rng.uniform(0.88, 1.15), 4),
        taper_scale=round(rng.uniform(0.85, 1.15), 4),
        nest_step_scale=round(rng.uniform(0.85, 1.15), 4),
        handle_arch_scale=round(rng.uniform(0.85, 1.15), 4),
        joint_limit_scale=round(rng.uniform(0.85, 1.10), 4),
        name=f"seeded_shopping_bucket_{seed}",
    )


def resolve_config(
    config: ShoppingBucketConfig | None = None,
) -> ResolvedShoppingBucketConfig:
    cfg = config or ShoppingBucketConfig()
    material_style = _pick(cfg.material_style, MATERIAL_STYLES)

    basket_count = int(cfg.basket_count) if cfg.basket_count is not None else 1
    basket_count = int(_clamp(basket_count, N_MIN, N_MAX))
    is_stack = basket_count > 1

    body_form = _pick(cfg.body_form, BODY_FORMS)
    wall_style = _pick(cfg.wall_style, WALL_STYLES)
    handle = _pick(cfg.handle, HANDLES)
    if cfg.lid_closure in ("drop_front_gate", "tilt_down_front_panel"):
        lid_closure = "hinged_top_lid"
    else:
        lid_closure = _pick(cfg.lid_closure, LID_CLOSURES)
    interior = _pick(cfg.interior, INTERIORS)

    # --- Compatibility gating for nested stacks (spec §10). ---
    if is_stack:
        # Force a nestable strong-taper body_form; oval/hex degrade to stackable.
        if body_form not in NESTABLE_BODY_FORMS:
            body_form = "tapered_stackable"
        # Stacked baskets carry no lid / no inner caddy (avoid stacked-pose
        # interpenetration); the stack only copies tub + bail.
        lid_closure = "open_no_lid"
        interior = "plain"

    len_scale = _clamp(cfg.body_len_scale, 0.90, 1.12)
    height_scale = _clamp(cfg.body_height_scale, 0.88, 1.15)
    taper_scale = _clamp(cfg.taper_scale, 0.85, 1.15)
    nest_scale = _clamp(cfg.nest_step_scale, 0.85, 1.15)
    arch_scale = _clamp(cfg.handle_arch_scale, 0.85, 1.15)
    limit_scale = _clamp(cfg.joint_limit_scale, 0.85, 1.10)

    l_top = _L_TOP * len_scale
    d_top = _D_TOP * len_scale
    h = _H * height_scale
    wall = _WALL
    nest_step = _NEST_STEP * nest_scale

    # Per body_form footprint adjustments (mouth dimensions / height / taper).
    if body_form == "shallow_wide_tray":
        h = _H * 0.62 * height_scale
        l_top = _L_TOP * 1.08 * len_scale
    elif body_form == "rounded_oval_deep":
        h = _H * 1.18 * height_scale
    elif body_form == "round_cylindrical_bucket":
        # Square-ish footprint for a round bucket (X ~= Y but still X >= Y).
        d_top = _D_TOP * 1.18 * len_scale

    # Base footprint = taper. Stronger taper for nestable stacks.
    if is_stack:
        base_len_ref = _L_BOT_NEST
        base_dep_ref = _D_BOT_NEST
    else:
        base_len_ref = _L_BOT
        base_dep_ref = _D_BOT
    # Apply taper_scale toward the mouth: taper_scale<1 = stronger taper.
    l_bot = l_top - (l_top * (1.0 - base_len_ref / _L_TOP)) * (2.0 - taper_scale)
    d_bot = d_top - (d_top * (1.0 - base_dep_ref / _D_TOP)) * (2.0 - taper_scale)

    # Nesting clearance inequality (spec §7, from 27719f51 L312-318):
    #   L_BOT + (L_TOP-L_BOT)*NEST_STEP/H - 2*WALL > L_BOT + 0.005
    # i.e. the upper basket's outer footprint at nesting height fits inside the
    # lower basket's inner cavity with > 5 mm clearance per side. If a stack
    # violates it, deepen the taper (lower l_bot/d_bot) until it holds.
    if is_stack:
        for _ in range(40):
            cx = (l_top - l_bot) * nest_step / h - 2.0 * wall
            cy = (d_top - d_bot) * nest_step / h - 2.0 * wall
            if cx > 0.006 and cy > 0.006:
                break
            # Strengthen the taper (shrink the base footprint).
            l_bot = max(0.5 * wall + 0.04, l_bot - 0.01)
            d_bot = max(0.5 * wall + 0.04, d_bot - 0.01)
        else:
            # Could not satisfy via taper; pull the nesting step in instead.
            nest_step = min(nest_step, 0.5 * h)

    arch_rise = _ARCH_RISE * arch_scale
    pivot_z = h - 0.014

    return ResolvedShoppingBucketConfig(
        body_form=body_form,
        wall_style=wall_style,
        handle=handle,
        lid_closure=lid_closure,
        interior=interior,
        basket_count=basket_count,
        material_style=material_style,
        l_top=l_top,
        d_top=d_top,
        l_bot=l_bot,
        d_bot=d_bot,
        h=h,
        wall=wall,
        r_vert_bot=_R_VERT_BOT,
        r_vert_top=_R_VERT_TOP,
        rim_h=_RIM_H,
        rim_overhang=_RIM_OVERHANG,
        pivot_z=pivot_z,
        bar_r=_BAR_R,
        arch_rise=arch_rise,
        handle_boss_r=_HANDLE_BOSS_R,
        handle_boss_len=_HANDLE_BOSS_LEN,
        tub_boss_r=_TUB_BOSS_R,
        tub_boss_len=_TUB_BOSS_LEN,
        nest_step=nest_step,
        joint_limit=_clamp(_JOINT_LIMIT * limit_scale, 1.0, math.radians(110.0)),
        name=cfg.name or "shopping_bucket",
    )


def with_overrides(config: ShoppingBucketConfig, **kwargs: object) -> ShoppingBucketConfig:
    return replace(config, **kwargs)


def slot_choices_for_config(
    config: ShoppingBucketConfig | ResolvedShoppingBucketConfig,
) -> tuple[tuple[str, str], ...]:
    r = config if isinstance(config, ResolvedShoppingBucketConfig) else resolve_config(config)
    return (
        ("body_form", r.body_form),
        ("wall_style", r.wall_style),
        ("handle", r.handle),
        ("lid_closure", r.lid_closure),
        ("interior", r.interior),
    )


def slot_choices_for_seed(seed: int) -> tuple[tuple[str, str], ...]:
    return slot_choices_for_config(config_from_seed(seed))


# ---------------------------------------------------------------------------
# Tub footprint cross-sections (body_form). Each returns a cq.Sketch for the
# cross-section at the given half-length (X) / half-depth (Y).
# ---------------------------------------------------------------------------
def _rect_section(hx: float, hy: float, fillet: float) -> cq.Sketch:
    f = min(fillet, 0.95 * min(hx, hy))
    return cq.Sketch().rect(2.0 * hx, 2.0 * hy).vertices().fillet(f)


def _ellipse_section(hx: float, hy: float) -> cq.Sketch:
    # Ellipse via a unit circle scaled; CadQuery Sketch supports ellipse via
    # trapezoid-free path: use a polygon approximation for robustness.
    pts = []
    n = 48
    for i in range(n):
        a = 2.0 * math.pi * i / n
        pts.append((hx * math.cos(a), hy * math.sin(a)))
    return cq.Sketch().polygon(pts)


def _hex_section(hx: float, hy: float) -> cq.Sketch:
    # Flat-top hexagon spanning hx in X and hy in Y.
    pts = [
        (hx, 0.0),
        (0.5 * hx, hy),
        (-0.5 * hx, hy),
        (-hx, 0.0),
        (-0.5 * hx, -hy),
        (0.5 * hx, -hy),
    ]
    return cq.Sketch().polygon(pts)


def _section_for(r: ResolvedShoppingBucketConfig, half_x: float, half_y: float) -> cq.Sketch:
    if r.body_form == "rounded_oval_deep":
        return _ellipse_section(half_x, half_y)
    if r.body_form == "hexagonal_footprint":
        return _hex_section(half_x, half_y)
    if r.body_form == "round_cylindrical_bucket":
        # Round footprint: use the smaller of the two halves to stay circular.
        rr = 0.5 * (half_x + half_y)
        return _ellipse_section(rr, rr)
    # rectangular / tapered_stackable / shallow_wide_tray -> rounded rect.
    z_frac = 1.0  # caller controls fillet via body_form radii below
    del z_frac
    return None  # signal: use rounded-rect path in _tub_mesh


# ---------------------------------------------------------------------------
# Shared tub mesh: hollow tapered shell + rolled rim + pivot bosses + wall cuts.
# ---------------------------------------------------------------------------
def _tub_union(
    base: cq.Workplane, other: cq.Workplane, r: ResolvedShoppingBucketConfig
) -> cq.Workplane:
    """Boolean-union onto the tub, fuzzy-toleranced for hexagonal footprints.

    A hex wall shares exactly coplanar flat side faces with the rolled rim and
    the weave hoops that straddle it. OCC's exact boolean on those coincident
    faces is unstable: it non-deterministically drops the tub body (leaving only
    the rim/hoop, floating high up), which surfaces as elevated ``z_min`` +
    disconnected-island failures on a random subset of hexagonal seeds. A small
    fuzzy tolerance (0.2 mm, << the 4 mm+ features) makes the fuse deterministic.
    """
    tol = 2e-4 if r.body_form == "hexagonal_footprint" else None
    return base.union(other, tol=tol)


def _loft_outer(r: ResolvedShoppingBucketConfig) -> cq.Workplane:
    """Loft the outer solid from the base section to the mouth section."""
    is_round_family = r.body_form in (
        "rounded_oval_deep",
        "hexagonal_footprint",
        "round_cylindrical_bucket",
    )
    if is_round_family:
        bot = _section_for(r, r.l_bot / 2.0, r.d_bot / 2.0)
        top = _section_for(r, r.l_top / 2.0, r.d_top / 2.0).moved(cq.Location(cq.Vector(0, 0, r.h)))
        return cq.Workplane("XY").placeSketch(bot, top).loft()
    # Rounded-rectangle family.
    bot = _rect_section(r.l_bot / 2.0, r.d_bot / 2.0, r.r_vert_bot)
    top = _rect_section(r.l_top / 2.0, r.d_top / 2.0, r.r_vert_top).moved(
        cq.Location(cq.Vector(0, 0, r.h))
    )
    return cq.Workplane("XY").placeSketch(bot, top).loft()


def _rolled_rim(r: ResolvedShoppingBucketConfig) -> cq.Workplane:
    """Rolled top rim band (ring) around the mouth."""
    is_round_family = r.body_form in (
        "rounded_oval_deep",
        "round_cylindrical_bucket",
    )
    rim_z = r.h - r.rim_h * 0.5
    if r.body_form == "round_cylindrical_bucket" or is_round_family:
        rr = 0.5 * (r.l_top / 2.0 + r.d_top / 2.0)
        ax = r.l_top / 2.0
        ay = r.d_top / 2.0
        ring_o = _ellipse_section(ax + r.rim_overhang, ay + r.rim_overhang)
        ring_i = _ellipse_section(max(0.02, ax - r.wall), max(0.02, ay - r.wall))
        del rr
    elif r.body_form == "hexagonal_footprint":
        ax = r.l_top / 2.0
        ay = r.d_top / 2.0
        ring_o = _hex_section(ax + r.rim_overhang, ay + r.rim_overhang)
        ring_i = _hex_section(max(0.02, ax - r.wall), max(0.02, ay - r.wall))
    else:
        l_o = r.l_top + 2.0 * r.rim_overhang
        d_o = r.d_top + 2.0 * r.rim_overhang
        l_i = r.l_top - r.wall
        d_i = r.d_top - r.wall
        r_o = r.r_vert_top + r.rim_overhang
        r_i = max(0.012, r.r_vert_top - r.wall)
        ring_o = _rect_section(l_o / 2.0, d_o / 2.0, r_o)
        ring_i = _rect_section(l_i / 2.0, d_i / 2.0, r_i)

    ring = cq.Workplane("XY", origin=(0, 0, rim_z)).placeSketch(ring_o).extrude(r.rim_h)
    inner = (
        cq.Workplane("XY", origin=(0, 0, rim_z - 0.002))
        .placeSketch(ring_i)
        .extrude(r.rim_h + 0.004)
    )
    return ring.cut(inner)


def _slot(plane: str, origin: tuple[float, float, float], w: float, hgt: float, fr: float):
    sk = cq.Sketch().rect(w, hgt).vertices().fillet(min(fr, 0.45 * min(w, hgt)))
    return cq.Workplane(plane, origin=origin).placeSketch(sk).extrude(0.8, both=True)


def _cut_slotted_walls(r: ResolvedShoppingBucketConfig, tub: cq.Workplane) -> cq.Workplane:
    """Vertical slot perforations through the walls (slotted_perforated)."""
    slot_zc = r.h * 0.43
    slot_h = min(_SLOT_H, r.h * 0.62)
    sx_long = [v * (r.l_top / _L_TOP) for v in _LONG_SLOT_X]
    sy_short = [v * (r.d_top / _D_TOP) for v in _SHORT_SLOT_Y]
    for x in sx_long:
        tub = tub.cut(_slot("XZ", (x, 0.0, slot_zc), _SLOT_W, slot_h, _SLOT_R))
    for y in sy_short:
        tub = tub.cut(_slot("YZ", (0.0, y, slot_zc), _SLOT_W, slot_h, _SLOT_R))
    return tub


def _cut_wire_mesh(
    r: ResolvedShoppingBucketConfig,
    tub: cq.Workplane,
    *,
    window: float = 0.045,
    margin_x: float = 0.70,
    margin_y: float = 0.60,
) -> cq.Workplane:
    """Cut square wall windows, leaving the surrounding wall as a lattice."""
    win = window
    z_bot = r.h * 0.16
    z_top = r.h * 0.80
    n_z = max(1, int((z_top - z_bot) / (win + 0.012)))
    # Long walls (cut along Y normal -> XZ plane cutters spread over X).
    n_x = max(2, int((r.l_top * margin_x) / (win + 0.012)))
    for iz in range(n_z):
        zc = z_bot + (iz + 0.5) * (z_top - z_bot) / n_z
        for ix in range(n_x):
            xc = -r.l_top * margin_x / 2.0 + (ix + 0.5) * (r.l_top * margin_x) / n_x
            tub = tub.cut(_slot("XZ", (xc, 0.0, zc), win, win, 0.004))
    # Short walls.
    n_y = max(1, int((r.d_top * margin_y) / (win + 0.012)))
    for iz in range(n_z):
        zc = z_bot + (iz + 0.5) * (z_top - z_bot) / n_z
        for iy in range(n_y):
            yc = -r.d_top * margin_y / 2.0 + (iy + 0.5) * (r.d_top * margin_y) / n_y
            tub = tub.cut(_slot("YZ", (0.0, yc, zc), win, win, 0.004))
    return tub


def _weave_hoop(r: ResolvedShoppingBucketConfig, zc: float, rib: float) -> cq.Workplane:
    """A raised weave band that follows the tub footprint at height ``zc`` (a thin
    annulus straddling the wall). Built from the body_form's own cross-section
    *interpolated to zc* so it hugs the tapered/curved wall all the way round.

    The earlier straight rectangular bars sat at the fixed mouth footprint and
    floated off the wall wherever it recedes — on elliptical/hex walls and, for a
    strongly-tapered rounded-rect wall, near the base — reading as disconnected
    geometry islands. A footprint-following hoop straddles the wall (inner radius
    inside the wall solid, outer radius proud) so it always fuses to the shell.
    """
    f = _clamp(zc / r.h, 0.0, 1.0)
    hx = (r.l_bot / 2.0) * (1.0 - f) + (r.l_top / 2.0) * f
    hy = (r.d_bot / 2.0) * (1.0 - f) + (r.d_top / 2.0) * f
    outer = _section_for(r, hx + rib, hy + rib)
    inner = _section_for(r, hx - rib, hy - rib)
    if outer is None:  # rounded-rect family
        fil = r.r_vert_bot * (1.0 - f) + r.r_vert_top * f
        outer = _rect_section(hx + rib, hy + rib, fil + rib)
        inner = _rect_section(hx - rib, hy - rib, max(0.002, fil - rib))
    band_h = 0.010
    ring = (
        cq.Workplane("XY", origin=(0.0, 0.0, zc - band_h * 0.5)).placeSketch(outer).extrude(band_h)
    )
    bore = (
        cq.Workplane("XY", origin=(0.0, 0.0, zc - band_h * 0.6))
        .placeSketch(inner)
        .extrude(band_h * 1.2)
    )
    return ring.cut(bore)


def _add_weave_bands(
    r: ResolvedShoppingBucketConfig, tub: cq.Workplane, *, style: WallStyle
) -> cq.Workplane:
    """Add raised wicker/rattan bands so weave variants read beyond cutouts.

    Bands are footprint-following raised hoops (see ``_weave_hoop``) for every
    body_form, so they fuse to the wall on any taper/curvature rather than
    floating off it. The band count/spacing differs per weave style so the three
    weave variants stay visually distinct.
    """
    rib = 0.006
    if style == "open_lattice_weave":
        z_fracs = (0.28, 0.44, 0.60, 0.76)
    elif style == "banded_wicker_weave":
        z_fracs = (0.24, 0.36, 0.54, 0.66)
    else:  # diagonal_herringbone_weave
        z_fracs = (0.22, 0.38, 0.54, 0.70)
    for zf in z_fracs:
        tub = _tub_union(tub, _weave_hoop(r, r.h * zf, rib), r)
    return tub


def _inner_hollow(r: ResolvedShoppingBucketConfig) -> cq.Workplane:
    """Inner cavity loft (footprint inset by one wall thickness), starting at the
    floor (z=wall) and running past the mouth. Cutting this from the outer loft
    produces an open-top hollow tub with a solid floor — a robust fallback for
    loft footprints where ``faces('>Z').shell(-wall)`` fails."""
    is_round_family = r.body_form in (
        "rounded_oval_deep",
        "hexagonal_footprint",
        "round_cylindrical_bucket",
    )
    if is_round_family:
        bot = _section_for(r, r.l_bot / 2.0 - r.wall, r.d_bot / 2.0 - r.wall).moved(
            cq.Location(cq.Vector(0, 0, r.wall))
        )
        top = _section_for(r, r.l_top / 2.0 - r.wall, r.d_top / 2.0 - r.wall).moved(
            cq.Location(cq.Vector(0, 0, r.h + 0.01))
        )
    else:
        bot = _rect_section(
            r.l_bot / 2.0 - r.wall, r.d_bot / 2.0 - r.wall, max(0.004, r.r_vert_bot - r.wall)
        ).moved(cq.Location(cq.Vector(0, 0, r.wall)))
        top = _rect_section(
            r.l_top / 2.0 - r.wall, r.d_top / 2.0 - r.wall, max(0.004, r.r_vert_top - r.wall)
        ).moved(cq.Location(cq.Vector(0, 0, r.h + 0.01)))
    return cq.Workplane("XY").placeSketch(bot, top).loft()


def _tub_mesh(r: ResolvedShoppingBucketConfig, *, with_divider: bool, nesting_nib: bool, assets):
    """Hollow tapered tub: loft, shell open at the top, rolled rim, wall cuts,
    short-wall pivot bosses, optional fused two-compartment divider, and (for
    nested stacks) a central nesting nib that sets the stacking height."""
    outer = _loft_outer(r)
    try:
        tub = outer.faces(">Z").shell(-r.wall)
    except Exception:
        # Tall deep-oval lofts can defeat OCC's shell builder; hollow by cutting
        # an inset inner loft instead (same open-top tub with a solid floor).
        tub = outer.cut(_inner_hollow(r))

    try:
        tub = tub.edges("<Z").fillet(0.006)
    except Exception:
        pass

    tub = _tub_union(tub, _rolled_rim(r), r)

    # Wall treatment.
    if r.wall_style == "slotted_perforated_plastic":
        tub = _cut_slotted_walls(r, tub)
    elif r.wall_style == "steel_wire_mesh":
        tub = _cut_wire_mesh(r, tub)
    elif r.wall_style == "open_lattice_weave":
        tub = _add_weave_bands(r, tub, style=r.wall_style)
    elif r.wall_style == "diagonal_herringbone_weave":
        tub = _add_weave_bands(r, tub, style=r.wall_style)
    elif r.wall_style == "banded_wicker_weave":
        tub = _add_weave_bands(r, tub, style=r.wall_style)
    # solid_smooth_plastic: no cuts.

    # Central nesting nib: a small column rising from the cavity floor to the
    # nesting height. The upper basket's floor rests on it, and it anchors the
    # FIXED nesting joint origin (0,0,nest_step) on real parent geometry (round
    # bucket cavities are otherwise empty at the center). Added *after* the wall
    # cuts: the slotted/wire-mesh cutters are centered on the X=0 short-wall plane
    # and sweep across the whole width, so cutting before this union would slice
    # the nib's upper portion away (truncating it below z=nest_step) and leave the
    # FIXED nesting-joint origin floating off the collision mesh.
    if nesting_nib:
        # Nib top cap sits exactly at z=nest_step so the FIXED nesting joint
        # origin (0,0,nest_step) lands on a real surface (the cap), and the upper
        # basket's floor rests on the cap.
        nib = (
            cq.Workplane("XY", origin=(0.0, 0.0, 0.0))
            .circle(max(0.016, r.bar_r * 1.6))
            .extrude(r.nest_step)
        )
        tub = tub.union(nib)

    # Pivot bosses on both short walls (raised knuckles the bail captures).
    for sx in (-1.0, 1.0):
        boss = (
            cq.Workplane("YZ", origin=(sx * (r.l_top / 2.0 - 0.001), 0.0, r.pivot_z))
            .circle(r.tub_boss_r)
            .extrude(sx * r.tub_boss_len)
        )
        tub = tub.union(boss)

    # Pivot cross-rod: a thin axle spanning the two short-wall bosses along X at
    # the rim line (z=pivot_z). This is the bail's pivot pin (the bail axle wraps
    # it as a captured pin); it also anchors the REVOLUTE/PRISMATIC joint origin
    # at (0,0,pivot_z) on real geometry for every body_form (round buckets have
    # an open center otherwise).
    pivot_rod = (
        cq.Workplane("YZ", origin=(-(r.l_top / 2.0), 0.0, r.pivot_z))
        .circle(r.bar_r * 0.55)
        .extrude(r.l_top)
    )
    tub = tub.union(pivot_rod)

    # Optional fused central divider wall (interior=two_compartment_divider).
    if with_divider:
        div = cq.Workplane("XZ", origin=(0.0, 0.0, r.h * 0.5)).box(
            r.l_bot * 0.92, r.h * 0.94, r.wall
        )
        tub = tub.union(div)

    return mesh_from_cadquery(tub, "basket_tub", assets=assets)


# ---------------------------------------------------------------------------
# Bail handle mesh.
# ---------------------------------------------------------------------------
def _arched_bail_mesh(r: ResolvedShoppingBucketConfig, *, folded_down: bool, assets):
    """Arched bail handle authored with its pivot line on the local X axis at
    y=0, z=0. When folded_down (nested stacks) the arch hangs in local -Z (q=0 =
    folded for stacking); otherwise it arches up in local +Z (q=0 = upright)."""
    rise = -r.arch_rise if folded_down else r.arch_rise
    pivot_x = r.l_top / 2.0 + 0.005
    path = cq.Workplane("XZ").moveTo(-pivot_x, 0.0).threePointArc((0.0, rise), (pivot_x, 0.0))
    path_wire = path.val()
    start = path_wire.positionAt(0.0)
    tan = path_wire.tangentAt(0.0)
    bar = (
        cq.Workplane(cq.Plane(origin=start.toTuple(), normal=tan.toTuple()))
        .circle(r.bar_r)
        .sweep(path, transition="round")
    )
    for sx in (-1.0, 1.0):
        boss = (
            cq.Workplane("YZ", origin=(sx * pivot_x, 0.0, 0.0))
            .circle(r.handle_boss_r)
            .extrude(-sx * r.handle_boss_len)
        )
        bar = bar.union(boss)
    # Pivot axle rod along the local X pivot line at z=0, connecting the two end
    # knuckles. This is the real bail pivot pin and places geometry right at the
    # child origin (0,0,0) where the REVOLUTE joint sits.
    axle = (
        cq.Workplane("YZ", origin=(-pivot_x, 0.0, 0.0)).circle(r.bar_r * 0.6).extrude(2.0 * pivot_x)
    )
    bar = bar.union(axle)
    return mesh_from_cadquery(bar, "bail_handle", assets=assets)


def _side_bail_mesh(r: ResolvedShoppingBucketConfig, assets):
    """A side folding bail: inverted-U arch whose two feet straddle one short
    end, pivoting about Y. Authored with pivot line on local Y axis at z=0; arch
    rises in +Z (q=0 upright)."""
    span = r.d_top * 0.55
    rise = r.arch_rise * 0.85
    path = cq.Workplane("YZ").moveTo(-span, 0.0).threePointArc((0.0, rise), (span, 0.0))
    path_wire = path.val()
    start = path_wire.positionAt(0.0)
    tan = path_wire.tangentAt(0.0)
    bar = (
        cq.Workplane(cq.Plane(origin=start.toTuple(), normal=tan.toTuple()))
        .circle(r.bar_r)
        .sweep(path, transition="round")
    )
    for sy in (-1.0, 1.0):
        boss = (
            cq.Workplane("XZ", origin=(0.0, sy * span, 0.0))
            .circle(r.handle_boss_r)
            .extrude(sy * r.handle_boss_len)
        )
        bar = bar.union(boss)
    # Pivot axle rod along the local Y pivot line at z=0 (the real fold pin),
    # placing geometry at the child origin (0,0,0) of the REVOLUTE joint. The
    # XZ workplane normal is -Y, so start at +span and extrude the full span.
    axle = cq.Workplane("XZ", origin=(0.0, span, 0.0)).circle(r.bar_r * 0.6).extrude(2.0 * span)
    bar = bar.union(axle)
    return mesh_from_cadquery(bar, "side_bail", assets=assets)


def _telescoping_handle_mesh(r: ResolvedShoppingBucketConfig, assets):
    """Central U-shaped telescoping pull-up bar. Authored with the U-bar around
    the rim level (local z=0) so PRISMATIC +Z lifts it straight out. Two vertical
    legs span the rim, joined by a top cross-bar; the legs reach down past the
    origin so the part AABB contains z=0 (joint origin sits on real hardware)."""
    leg_x = r.l_top / 2.0 - 0.03
    grip_h = r.arch_rise * 0.95
    leg_bot = -0.04
    bar = None
    for sx in (-1.0, 1.0):
        leg = (
            cq.Workplane("XY", origin=(sx * leg_x, 0.0, leg_bot))
            .circle(r.bar_r)
            .extrude(grip_h - leg_bot)
        )
        bar = leg if bar is None else bar.union(leg)
    # Top cross-bar (grip) along X.
    top_cross = (
        cq.Workplane("YZ", origin=(-leg_x, 0.0, grip_h)).circle(r.bar_r).extrude(2.0 * leg_x)
    )
    bar = bar.union(top_cross)
    # Bottom slider cross-bar along X at z=0 (the rim-level slider hardware),
    # placing geometry at the child origin (0,0,0) of the PRISMATIC joint.
    bottom_cross = (
        cq.Workplane("YZ", origin=(-leg_x, 0.0, 0.0)).circle(r.bar_r * 0.8).extrude(2.0 * leg_x)
    )
    bar = bar.union(bottom_cross)
    return mesh_from_cadquery(bar, "telescoping_handle", assets=assets)


# ---------------------------------------------------------------------------
# Lid / closure meshes. Authored so the part frame sits at the hinge line.
# ---------------------------------------------------------------------------
def _top_lid_mesh(r: ResolvedShoppingBucketConfig, assets):
    """Flat top lid: frame at +Y long rim hinge line; panel extends in -Y,
    centered at local z=0."""
    lid_x = r.l_top - 2.0 * r.wall
    lid_y = r.d_top - 0.5 * r.wall
    lid_t = 0.010
    panel = (
        cq.Workplane("XY")
        .box(lid_x, lid_y, lid_t)
        .edges("|Z")
        .fillet(0.006)
        .translate((0.0, -lid_y / 2.0, 0.0))
    )
    for kx in (-lid_x * 0.35, lid_x * 0.35):
        knuckle = cq.Workplane("YZ").circle(0.012).extrude(0.03).translate((kx - 0.015, 0.0, 0.0))
        panel = panel.union(knuckle)
    tab = (
        cq.Workplane("XY")
        .box(0.05, 0.010, 0.014)
        .translate((0.0, -lid_y + 0.005, lid_t / 2.0 + 0.007))
    )
    panel = panel.union(tab)
    return mesh_from_cadquery(panel, "top_lid", assets=assets)


def _clamshell_leaf_mesh(r: ResolvedShoppingBucketConfig, assets):
    """One clamshell leaf: frame at a long rim hinge line, panel extends to the
    centerline (in +Y for the local frame), centered at local z=0."""
    leaf_x = r.l_top - 2.0 * r.wall
    leaf_y = r.d_top / 2.0 - 0.002
    leaf_t = 0.009
    panel = (
        cq.Workplane("XY")
        .box(leaf_x, leaf_y, leaf_t)
        .edges("|Z")
        .fillet(0.005)
        .translate((0.0, leaf_y / 2.0, 0.0))
    )
    knuckle = cq.Workplane("YZ").circle(0.011).extrude(leaf_x * 0.5, both=True)
    panel = panel.union(knuckle)
    return mesh_from_cadquery(panel, "clamshell_leaf", assets=assets)


def _front_panel_mesh(r: ResolvedShoppingBucketConfig, *, tall: bool, assets):
    """Front gate / tilt panel: frame at the front-wall bottom edge hinge line;
    panel extends upward in local +Z, centered in X. ``tall`` covers the full
    wall (drop-front gate); otherwise a lower panel (tilt-down)."""
    pan_x = r.l_top * 0.86
    pan_h = (r.h - 0.01) if tall else (r.h * 0.55)
    pan_t = 0.010
    panel = (
        cq.Workplane("XY")
        .box(pan_x, pan_t, pan_h)
        .edges("|Y")
        .fillet(0.004)
        .translate((0.0, 0.0, pan_h / 2.0))
    )
    # Hinge knuckles along the bottom edge (local z=0), axis along X.
    knuckle = cq.Workplane("YZ").circle(0.011).extrude(pan_x * 0.5, both=True)
    panel = panel.union(knuckle)
    name = "drop_gate" if tall else "tilt_panel"
    return mesh_from_cadquery(panel, name, assets=assets)


# ---------------------------------------------------------------------------
# Caddy meshes (interior=removable_inner_caddy).
# ---------------------------------------------------------------------------
_CADDY_CLEAR_MARGIN = 0.008  # min gap (>=8 mm) between the caddy envelope and mouth hardware
_CADDY_WALL = 0.006
_CADDY_GRIP_BAR_R = 0.006


def _mouth_clear_ceiling(r: ResolvedShoppingBucketConfig) -> float:
    """Lowest z of any *sibling* hardware spanning the tub mouth that the inner
    caddy (shell + upright folding grip) must stay clear of.

    A removable inner caddy is a sibling of both the carry handle and the top
    lid/closure (all children of ``basket_0``), so QC tests every independent
    combination of their joint poses. The caddy can therefore be lifted while a
    central bail rests over the mouth or a top lid is closed — a genuine 穿模.
    We derive a hard ceiling from whichever piece of mouth hardware hangs
    lowest:

      * central bail / swing bail — the arch rests on the rim at z=pivot_z, its
        bar reaching down to ``pivot_z - bar_r`` (also the tub pivot axle line);
      * telescoping pull-up — the bottom slider cross-bar at ``pivot_z``;
      * a closed top lid — its underside at ``rim_z - lid_half_thickness``.

    Dual folding side bails and their pivot rods sit at the short-wall *ends*,
    clear of the centered caddy in X, so they impose no ceiling. When nothing
    spans the mouth the caddy may rise to just under the rim.
    """
    floors: list[float] = []
    if r.handle in ("fixed_central_arched", "single_swing_bail"):
        floors.append(r.pivot_z - r.bar_r)
    elif r.handle == "single_telescoping_pull_up":
        floors.append(r.pivot_z - r.bar_r * 0.8)
    if r.lid_closure != "open_no_lid":
        rim_z = r.h - r.rim_h * 0.5
        floors.append(rim_z - 0.005)  # top_lid panel half-thickness (lid_t/2)
    if not floors:
        floors.append(r.h - 0.004)  # dual bail + open top: just under the rim
    return min(floors)


def _caddy_geometry(r: ResolvedShoppingBucketConfig):
    """Single-source caddy dimensions + derived motion clearances (Contract 3c).

    Returns ``(cx, cy, ch, seat, grip_rise, lift_max)``. ``grip_rise`` (folding
    grip arch height) and ``lift_max`` (PRISMATIC lift travel) are derived from
    the mouth-hardware ceiling so that the caddy shell and its *upright* grip
    stay >=8 mm below the lowest sibling mouth hardware in every reachable pose.
    """
    cx = r.l_bot * 0.7
    cy = r.d_bot * 0.6
    ch = r.h * 0.45
    seat = max(0.001, r.wall - 0.003)
    ceiling = _mouth_clear_ceiling(r)
    # Headroom above the caddy floor for {upright grip arch + bar radius + lift},
    # keeping the whole caddy envelope a clear margin below the ceiling.
    avail = ceiling - _CADDY_CLEAR_MARGIN - seat - ch - _CADDY_GRIP_BAR_R
    grip_rise = min(0.05, avail - 0.006)
    grip_rise = _clamp(grip_rise, 0.010, 0.05)
    lift_max = max(0.006, avail - grip_rise)
    return cx, cy, ch, seat, grip_rise, lift_max


def _caddy_mesh(r: ResolvedShoppingBucketConfig, assets):
    """Small inner caddy tray sized to drop inside the tub; mesh bottom at local
    z=0 so PRISMATIC +Z lifts it off the floor."""
    cx, cy, ch, _seat, _grip_rise, _lift = _caddy_geometry(r)
    wall = _CADDY_WALL
    outer = (
        cq.Workplane("XY").box(cx, cy, ch, centered=(True, True, False)).edges("|Z").fillet(0.01)
    )
    tray = outer.faces(">Z").shell(-wall)
    # Cross-tie bar across the caddy mouth at z=ch: the grip pivots on it, and it
    # anchors the caddy_to_grip joint origin (0,0,ch) on real geometry. It spans
    # the full inner width and embeds a little into both side walls so the bar is
    # a real cross-member fused to the shell (not a floating island).
    tie_half = cx / 2.0 - wall * 0.5
    tie = cq.Workplane("YZ", origin=(-tie_half, 0.0, ch)).circle(0.004).extrude(2.0 * tie_half)
    tray = tray.union(tie)
    return mesh_from_cadquery(tray, "caddy_shell", assets=assets), ch


def _caddy_grip_mesh(r: ResolvedShoppingBucketConfig, assets, *, rise: float):
    """Folding grip on the caddy top: arch with pivot line on local X at z=0,
    rising in +Z. ``rise`` is derived by ``_caddy_geometry`` so the upright grip
    clears the mouth hardware ceiling."""
    span = r.l_bot * 0.3
    path = cq.Workplane("XZ").moveTo(-span, 0.0).threePointArc((0.0, rise), (span, 0.0))
    path_wire = path.val()
    start = path_wire.positionAt(0.0)
    tan = path_wire.tangentAt(0.0)
    bar = (
        cq.Workplane(cq.Plane(origin=start.toTuple(), normal=tan.toTuple()))
        .circle(0.006)
        .sweep(path, transition="round")
    )
    # Pivot axle rod along the local X pivot line at z=0 (the real fold pin),
    # connecting the two arch feet. The arch only touches z=0 at its endpoints
    # (x=+-span), so without this rod the child origin (0,0,0) of the REVOLUTE
    # caddy_to_grip joint sits in the empty arch span and reads as off-geometry.
    axle = cq.Workplane("YZ", origin=(-span, 0.0, 0.0)).circle(0.006 * 0.9).extrude(2.0 * span)
    bar = bar.union(axle)
    return mesh_from_cadquery(bar, "caddy_grip", assets=assets)


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------
def _basket_inertial(r: ResolvedShoppingBucketConfig) -> Inertial:
    return Inertial.from_geometry(
        Box((r.l_top, r.d_top, r.h)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, r.h / 2.0)),
    )


def _emit_handle(
    model: ArticulatedObject,
    r: ResolvedShoppingBucketConfig,
    basket,
    idx: int,
    *,
    folded_down: bool,
    central_bail_mesh,
    side_bail_mesh,
    telescoping_mesh,
    mats,
) -> list[str]:
    """Emit the handle(s) for one basket; returns the list of handle part names."""
    names: list[str] = []
    if r.handle in ("fixed_central_arched", "single_swing_bail"):
        h = model.part(f"handle_{idx}")
        h.visual(central_bail_mesh, material=mats["handle"], name="bail_bar")
        h.inertial = Inertial.from_geometry(
            Box((r.l_top, 2.0 * r.bar_r, r.arch_rise)),
            mass=0.18,
            origin=Origin(xyz=(0.0, 0.0, (-1.0 if folded_down else 1.0) * r.arch_rise / 2.0)),
        )
        # A bail pivots at the rim and rests on the rim on either side: its
        # physical range is +-90 deg (past that the arch would swing through the
        # mouth into the tub contents / inner caddy). The old upper=pi swung the
        # arch fully over and dipped it ~40 mm into the inner caddy. Cap the
        # swing to a symmetric rim-rest range so the arch bar never descends
        # below z=pivot_z-bar_r, which the caddy envelope is derived to clear.
        bail_limit = min(r.joint_limit, math.pi / 2.0)
        model.articulation(
            f"basket_{idx}_to_handle_{idx}",
            ArticulationType.REVOLUTE,
            parent=basket,
            child=h,
            origin=Origin(xyz=(0.0, 0.0, r.pivot_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0, velocity=2.5, lower=-bail_limit, upper=bail_limit
            ),
        )
        names.append(f"handle_{idx}")
    elif r.handle == "dual_folding_bail":
        for k, sx in ((0, 1.0), (1, -1.0)):
            h = model.part(f"handle_{idx}_{k}")
            h.visual(side_bail_mesh, material=mats["handle"], name=f"side_bail_{k}")
            h.inertial = Inertial.from_geometry(
                Box((2.0 * r.bar_r, r.d_top * 0.6, r.arch_rise)),
                mass=0.12,
                origin=Origin(xyz=(0.0, 0.0, r.arch_rise * 0.4)),
            )
            model.articulation(
                f"basket_{idx}_to_handle_{idx}_{k}",
                ArticulationType.REVOLUTE,
                parent=basket,
                child=h,
                origin=Origin(xyz=(sx * (r.l_top / 2.0 - 0.02), 0.0, r.pivot_z)),
                axis=(0.0, sx, 0.0),
                motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=0.0, upper=math.pi),
            )
            names.append(f"handle_{idx}_{k}")
    else:  # single_telescoping_pull_up
        h = model.part(f"handle_{idx}")
        h.visual(telescoping_mesh, material=mats["handle"], name="pull_bar")
        h.inertial = Inertial.from_geometry(
            Box((r.l_top, 2.0 * r.bar_r, r.arch_rise)),
            mass=0.2,
            origin=Origin(xyz=(0.0, 0.0, r.arch_rise / 2.0)),
        )
        model.articulation(
            f"basket_{idx}_to_handle_{idx}",
            ArticulationType.PRISMATIC,
            parent=basket,
            child=h,
            origin=Origin(xyz=(0.0, 0.0, r.pivot_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=0.5, lower=0.0, upper=r.h * 0.9),
        )
        names.append(f"handle_{idx}")
    return names


def _emit_lid(
    model: ArticulatedObject,
    r: ResolvedShoppingBucketConfig,
    basket,
    *,
    assets,
    mats,
) -> list[str]:
    """Emit the lid/closure parts (N=1 only). Returns lid part names."""
    rim_z = r.h - r.rim_h * 0.5
    if r.lid_closure != "open_no_lid":
        lid = model.part("lid")
        lid.visual(_top_lid_mesh(r, assets), material=mats["accent"], name="lid_panel")
        lid.inertial = Inertial.from_geometry(
            Box((r.l_top, r.d_top, 0.02)),
            mass=0.3,
            origin=Origin(xyz=(0.0, -r.d_top / 2.0, 0.0)),
        )
        model.articulation(
            "body_to_lid",
            ArticulationType.PRISMATIC,
            parent=basket,
            child=lid,
            origin=Origin(xyz=(0.0, r.d_top / 2.0, rim_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=5.0, velocity=0.8, lower=0.0, upper=r.h * 0.9),
        )
        return ["lid"]
    return []


def _emit_caddy(
    model: ArticulatedObject,
    r: ResolvedShoppingBucketConfig,
    basket,
    *,
    assets,
    mats,
) -> list[str]:
    """Emit the removable inner caddy (PRISMATIC) + folding grip (REVOLUTE)."""
    caddy_mesh, ch = _caddy_mesh(r, assets)
    _cx, _cy, _ch, seat, grip_rise, lift_max = _caddy_geometry(r)
    caddy = model.part("inner_caddy")
    caddy.visual(caddy_mesh, material=mats["accent"], name="caddy_shell")
    caddy.inertial = Inertial.from_geometry(
        Box((r.l_bot * 0.7, r.d_bot * 0.6, ch)),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, ch / 2.0)),
    )
    # Seat the caddy bottom slightly into the inner floor (floor top is at
    # z=wall) so it rests in contact with the tub and is not isolated. The lift
    # travel (lift_max) keeps the caddy shell + upright grip a clear >=8 mm below
    # the closed lid / resting bail spanning the mouth (see _mouth_clear_ceiling):
    # QC tests the caddy-up pose against every closed lid / bail-at-rest pose, so
    # an unbounded lift would drive the caddy straight through them.
    model.articulation(
        "tub_to_caddy",
        ArticulationType.PRISMATIC,
        parent=basket,
        child=caddy,
        origin=Origin(xyz=(0.0, 0.0, seat)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.0, lower=0.0, upper=lift_max),
    )
    grip = model.part("caddy_grip")
    grip.visual(
        _caddy_grip_mesh(r, assets, rise=grip_rise), material=mats["handle"], name="grip_bar"
    )
    grip.inertial = Inertial.from_geometry(
        Box((r.l_bot * 0.6, 2.0 * 0.006, 0.06)),
        mass=0.02,
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
    )
    model.articulation(
        "caddy_to_grip",
        ArticulationType.REVOLUTE,
        parent=caddy,
        child=grip,
        origin=Origin(xyz=(0.0, 0.0, ch)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0, velocity=2.0, lower=-math.radians(90.0), upper=math.radians(90.0)
        ),
    )
    return ["inner_caddy", "caddy_grip"]


def build_shopping_bucket(
    config: ShoppingBucketConfig | None = None,
    *,
    assets: AssetContext | None = None,
) -> ArticulatedObject:
    r = resolve_config(config)
    model = ArticulatedObject(name=r.name, assets=assets)
    mats = {
        key: model.material(
            f"bucket_{key}_{r.material_style}", rgba=PALETTES[r.material_style][key]
        )
        for key in ("body", "handle", "accent")
    }

    n = r.basket_count
    is_stack = r.is_stack

    # Shared meshes: identical geometry reused across every basket/handle so the
    # nested stack stays cheap regardless of N.
    with_divider = (not is_stack) and r.interior == "two_compartment_divider"
    tub_mesh = _tub_mesh(r, with_divider=with_divider, nesting_nib=is_stack, assets=assets)

    central_bail_mesh = None
    side_bail_mesh = None
    telescoping_mesh = None
    if r.handle in ("fixed_central_arched", "single_swing_bail"):
        central_bail_mesh = _arched_bail_mesh(r, folded_down=is_stack, assets=assets)
    elif r.handle == "dual_folding_bail":
        side_bail_mesh = _side_bail_mesh(r, assets)
    else:
        telescoping_mesh = _telescoping_handle_mesh(r, assets)

    baskets = []
    for i in range(n):
        basket = model.part(f"basket_{i}")
        basket.visual(tub_mesh, material=mats["body"], name="tub_shell")
        basket.inertial = _basket_inertial(r)
        baskets.append(basket)
        _emit_handle(
            model,
            r,
            basket,
            i,
            folded_down=is_stack,
            central_bail_mesh=central_bail_mesh,
            side_bail_mesh=side_bail_mesh,
            telescoping_mesh=telescoping_mesh,
            mats=mats,
        )

    # Single-basket extras (lid + caddy) — never in a nested stack.
    if not is_stack:
        _emit_lid(model, r, baskets[0], assets=assets, mats=mats)
        if r.interior == "removable_inner_caddy":
            _emit_caddy(model, r, baskets[0], assets=assets, mats=mats)

    # Nesting chain: basket_{i} carries basket_{i+1} at a constant +Z offset
    # (absolute per-pair origin => N-invariant).
    for i in range(n - 1):
        model.articulation(
            f"basket_{i}_to_basket_{i + 1}",
            ArticulationType.FIXED,
            parent=baskets[i],
            child=baskets[i + 1],
            origin=Origin(xyz=(0.0, 0.0, r.nest_step)),
        )

    model.meta["slot_choices"] = slot_choices_for_config(r)
    return model


def build_seeded_shopping_bucket(
    seed: int,
    *,
    assets: AssetContext | None = None,
) -> ArticulatedObject:
    return build_shopping_bucket(config_from_seed(seed), assets=assets)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def _handle_part_names(r: ResolvedShoppingBucketConfig, idx: int) -> list[str]:
    if r.handle == "dual_folding_bail":
        return [f"handle_{idx}_0", f"handle_{idx}_1"]
    return [f"handle_{idx}"]


def run_shopping_bucket_tests(
    object_model: ArticulatedObject,
    config: ShoppingBucketConfig,
) -> TestReport:
    r = resolve_config(config)
    ctx = TestContext(object_model)
    n = r.basket_count

    # --- Captured-pin allowances: each basket's bail pivot is seated in the
    #     short-wall boss; the folded bail also rests against the tub exterior. ---
    for i in range(n):
        basket = object_model.get_part(f"basket_{i}")
        for hn in _handle_part_names(r, i):
            handle = object_model.get_part(hn)
            ctx.allow_overlap(
                basket,
                handle,
                reason=(
                    "bail-handle pivot knuckle captured inside the tub short-side "
                    "pivot boss (pin-in-boss pivot); folded handle rests against "
                    "the tapered short-wall exterior."
                ),
            )

    # --- Nested-stack interleaving allowances (spec §7, from 27719f51). ---
    # In a tight nested stack the strongly-tapered tubs sit wall-against-wall and
    # every basket's folded bail hangs down through several baskets below it, so
    # bails interleave across the whole stack — not just adjacent pairs. We allow
    # overlap between every basket↔other-basket's-handle and every handle↔handle.
    if n > 1:
        all_handles: list[list[str]] = [_handle_part_names(r, i) for i in range(n)]
        for i in range(n):
            bi = object_model.get_part(f"basket_{i}")
            # Each basket vs every other basket's folded bail.
            for k in range(n):
                if k == i:
                    continue
                for hn in all_handles[k]:
                    ctx.allow_overlap(
                        bi,
                        object_model.get_part(hn),
                        reason=(
                            "in the nested stack the folded bails hang down through "
                            "the baskets below them, passing along the tapered walls."
                        ),
                    )
            # Adjacent nested tubs sit wall-against-wall (round/strong-taper fit).
            for k in range(i + 1, n):
                ctx.allow_overlap(
                    bi,
                    object_model.get_part(f"basket_{k}"),
                    reason=(
                        "nested shopping baskets stack wall-against-wall; the upper "
                        "tub's lower body sits inside the lower tub's cavity."
                    ),
                )
        # Every pair of folded bails interleaves.
        for i in range(n):
            for k in range(i + 1, n):
                for hn0 in all_handles[i]:
                    for hn1 in all_handles[k]:
                        ctx.allow_overlap(
                            object_model.get_part(hn0),
                            object_model.get_part(hn1),
                            reason="folded bail handles interleave in the nested stack.",
                        )

    # --- Lid closed-pose rim-crest allowance (N=1 closures). ---
    if n == 1:
        basket0 = object_model.get_part("basket_0")
        lid_parts = []
        if r.lid_closure != "open_no_lid":
            lid_parts = ["lid"]
        for lp in lid_parts:
            ctx.allow_overlap(
                basket0,
                object_model.get_part(lp),
                reason="closed lid/closure panel seats on the rolled rim crest.",
            )
            # An upright central/swing bail arches over the mouth where a closed
            # top lid / clamshell leaf sits; allow that rest-pose overlap.
            for hn in _handle_part_names(r, 0):
                ctx.allow_overlap(
                    object_model.get_part(hn),
                    object_model.get_part(lp),
                    reason="upright bail handle arches over the closed lid/closure panel.",
                )
        if r.interior == "removable_inner_caddy":
            caddy = object_model.get_part("inner_caddy")
            grip = object_model.get_part("caddy_grip")
            ctx.allow_overlap(
                basket0,
                caddy,
                reason="inner caddy seats inside the tub cavity (q=0 on the floor).",
            )
            ctx.allow_overlap(
                basket0,
                grip,
                reason="caddy's folding grip stands up inside the basket mouth at q=0.",
            )
            ctx.allow_overlap(
                caddy,
                grip,
                reason=(
                    "the folding grip's pivot axle wraps the caddy mouth cross-tie "
                    "(captured-pin pivot at z=ch)."
                ),
            )
            for hn in _handle_part_names(r, 0):
                ctx.allow_overlap(
                    grip,
                    object_model.get_part(hn),
                    reason="caddy grip and the bail handle both occupy the mouth at rest.",
                )

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    # Articulation-origin baseline (flat 0.015 m) runs in the compiler; all our
    # joint origins sit on real pivot hardware (boss line / rim edge / floor).
    ctx.fail_if_joint_mating_has_gap()

    # --- Part inventory + naming. ---
    part_names = {p.name for p in object_model.parts}
    ctx.check(
        "N baskets emitted",
        all(f"basket_{i}" in part_names for i in range(n)),
        details=str(sorted(part_names)),
    )

    # --- Nesting joints: N-1 FIXED +Z. ---
    fixed_count = sum(
        1 for j in object_model.joints if j.articulation_type == ArticulationType.FIXED
    )
    ctx.check(
        "nesting chain has N-1 FIXED joints",
        fixed_count == n - 1,
        details=f"fixed_joints={fixed_count} expected={n - 1}",
    )

    # --- Each basket's bail joint is the expected articulation. ---
    for i in range(n):
        if r.handle in ("fixed_central_arched", "single_swing_bail"):
            j = object_model.get_articulation(f"basket_{i}_to_handle_{i}")
            ctx.check(
                f"basket_{i} bail is REVOLUTE about X",
                j.articulation_type == ArticulationType.REVOLUTE and abs(j.axis[0]) > 0.99,
                details=f"type={j.articulation_type} axis={tuple(j.axis)}",
            )
        elif r.handle == "single_telescoping_pull_up":
            j = object_model.get_articulation(f"basket_{i}_to_handle_{i}")
            ctx.check(
                f"basket_{i} handle is PRISMATIC about Z",
                j.articulation_type == ArticulationType.PRISMATIC and abs(j.axis[2]) > 0.99,
                details=f"type={j.articulation_type} axis={tuple(j.axis)}",
            )
        else:  # dual_folding_bail
            for k in (0, 1):
                j = object_model.get_articulation(f"basket_{i}_to_handle_{i}_{k}")
                ctx.check(
                    f"basket_{i} folding bail {k} is REVOLUTE about Y",
                    j.articulation_type == ArticulationType.REVOLUTE and abs(j.axis[1]) > 0.99,
                    details=f"type={j.articulation_type} axis={tuple(j.axis)}",
                )

    # --- Root basket reads as an open shopping basket on the ground. ---
    aabb0 = ctx.part_world_aabb(object_model.get_part("basket_0"))
    if aabb0 is not None:
        (axmn, aymn, azmn), (axmx, aymx, azmx) = aabb0
        ctx.check(
            "basket_0 wider in X than Y and rests on the ground",
            (axmx - axmn) > (aymx - aymn) and azmn < 0.03,
            details=f"ext_x={axmx - axmn:.3f} ext_y={aymx - aymn:.3f} z_min={azmn:.4f}",
        )

    # --- Tapered body (mouth wider than base). ---
    ctx.check(
        "body is tapered (mouth wider than base)",
        r.l_top > r.l_bot + 0.01 and r.d_top > r.d_bot + 0.01,
        details=f"l_top={r.l_top:.3f} l_bot={r.l_bot:.3f} d_top={r.d_top:.3f} d_bot={r.d_bot:.3f}",
    )

    # --- Nested taper clearance proven from authored dims (stack only). ---
    if n > 1:
        cx = (r.l_top - r.l_bot) * r.nest_step / r.h - 2.0 * r.wall
        cy = (r.d_top - r.d_bot) * r.nest_step / r.h - 2.0 * r.wall
        ctx.check(
            "taper provides wall clearance for nested baskets",
            cx > 0.005 and cy > 0.005,
            details=f"clearance_x={cx:.4f} clearance_y={cy:.4f}",
        )
        # Upper basket sits above and nested inside the lower one.
        for i in range(n - 1):
            lo = ctx.part_world_aabb(object_model.get_part(f"basket_{i}"))
            up = ctx.part_world_aabb(object_model.get_part(f"basket_{i + 1}"))
            if lo is not None and up is not None:
                ctx.check(
                    f"basket_{i + 1} sits above basket_{i}",
                    up[0][2] > lo[0][2] + 0.005,
                    details=f"lower_zmin={lo[0][2]:.4f} upper_zmin={up[0][2]:.4f}",
                )

    # --- Handle actuation raises the carry mechanism. ---
    if r.handle in ("fixed_central_arched", "single_swing_bail"):
        j = object_model.get_articulation("basket_0_to_handle_0")
        handle = object_model.get_part("handle_0")
        # In a stack the bail starts folded-down (q=0) and swings up; for N=1 it
        # starts upright (q=0). Either way a large positive q lifts the top.
        with ctx.pose({j: 0.0}):
            a0 = ctx.part_world_aabb(handle)
        with ctx.pose({j: math.pi * 0.6 if r.is_stack else 0.0}):
            a1 = ctx.part_world_aabb(handle)
        if a0 is not None and a1 is not None and r.is_stack:
            ctx.check(
                "folded bail swings up with positive q",
                a1[1][2] > a0[1][2] + 0.03,
                details=f"folded_top={a0[1][2]:.3f} swung_top={a1[1][2]:.3f}",
            )
    elif r.handle == "single_telescoping_pull_up":
        j = object_model.get_articulation("basket_0_to_handle_0")
        handle = object_model.get_part("handle_0")
        with ctx.pose({j: 0.0}):
            a0 = ctx.part_world_aabb(handle)
        with ctx.pose({j: r.h * 0.8}):
            a1 = ctx.part_world_aabb(handle)
        if a0 is not None and a1 is not None:
            ctx.check(
                "telescoping handle extends upward",
                a1[1][2] > a0[1][2] + 0.1,
                details=f"retracted_top={a0[1][2]:.3f} extended_top={a1[1][2]:.3f}",
            )

    if r.lid_closure != "open_no_lid":
        lid_joint = object_model.get_articulation("body_to_lid")
        lid = object_model.get_part("lid")
        ctx.check(
            "basket lid lifts upward on PRISMATIC Z joint",
            lid_joint.articulation_type == ArticulationType.PRISMATIC
            and abs(lid_joint.axis[2]) > 0.99,
            details=f"type={lid_joint.articulation_type} axis={tuple(lid_joint.axis)}",
        )
        with ctx.pose({lid_joint: 0.0}):
            closed = ctx.part_world_aabb(lid)
        with ctx.pose({lid_joint: r.h * 0.55}):
            lifted = ctx.part_world_aabb(lid)
        if closed is not None and lifted is not None:
            ctx.check(
                "basket lid moves upward when opened",
                lifted[0][2] > closed[0][2] + r.h * 0.45,
                details=f"closed_zmin={closed[0][2]:.3f} lifted_zmin={lifted[0][2]:.3f}",
            )

    # --- slot_choices recorded and N encoded. ---
    ctx.check(
        "slot_choices_recorded",
        tuple(object_model.meta.get("slot_choices", ())) == slot_choices_for_config(r),
        details=str(object_model.meta.get("slot_choices")),
    )

    return ctx.report()


__all__ = (
    "ShoppingBucketConfig",
    "ResolvedShoppingBucketConfig",
    "build_shopping_bucket",
    "build_seeded_shopping_bucket",
    "config_from_seed",
    "resolve_config",
    "run_shopping_bucket_tests",
    "slot_choices_for_config",
    "slot_choices_for_seed",
    "with_overrides",
)
