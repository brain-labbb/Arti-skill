"""Modular procedural template for ``rifle``.

Follows ``articraft_template_authoring/specs_modular_v1/Military_Rifle.md``.

Single-soldier shoulder-fired military carbine (M4 / AR-pattern straight
in-line layout). A bore axis runs the full length along world +X (muzzle
toward +X, buttstock toward -X). World +Z is up; the rifle rests with the
seated magazine floorplate near z=0.

Fixed structural spine (identical across every seed, written as fixed
structure, not a slot):
- ``receiver`` part: upper/lower receiver, flat-top picatinny rail base+ribs,
  canted magwell, trigger guard loop, buffer tube + castle nut (or the
  folding-stock stub + hinge bracket/pin when ``buttstock=side_folding``),
  pistol grip, delta ring.
- ``barrel`` part (FIXED to receiver) carrying the muzzle device visual.
- ``front_sight_tower`` (FIXED to barrel) and ``vertical_foregrip``
  (FIXED to handguard).
- Five spine joints that are always present except where a slot module
  removes one: ``receiver_to_barrel`` FIXED, ``trigger_pull`` REVOLUTE +Y,
  ``charging_handle_slide`` PRISMATIC -X, ``magazine_release`` PRISMATIC
  down a forward-canted axis, ``safety_selector_rotate`` REVOLUTE -Y.

Four parallel-children slots hang off that spine (pattern = mixed):

    A buttstock  — collapsible / fixed_A2 / side_folding / pdw_wire
    B handguard  — quad_picatinny / smooth_tube / mlok / keymod
                   (carries the top-rail Picatinny rib-count multiplicity N)
    C optic      — red_dot / scope / iron_sights
    D muzzle     — birdcage / suppressor / brake (barrel visual, no joint)

Multiplicity axis: handguard top-rail Picatinny rib count N in [6, 30],
active only for handguard modules that keep a top rail
(quad_picatinny / mlok / keymod); ``smooth_tube`` gates it off.

Adopted sources (spec Module Source Index, adapted verbatim):
- rec_model-an-m4-style-military-carbine-rifle-all-mat_...497c83bd —
  spine + collapsible / quad_picatinny / red_dot / birdcage baselines.
- rec_rifle_var_fixedstock — A2 fixed buttstock.
- rec_rifle_var_foldstock — side-folding stock + receiver stub/hinge branch.
- rec_rifle_var_pdwstock — PDW wire stock.
- rec_rifle_var_smoothtube / _mlokrail / _keymod — handguard surfaces.
- rec_rifle_var_scope / _ironsights — optic alternatives.
- rec_rifle_var_suppressor / _brake — muzzle device alternatives.
- rec_rifle_var_rails7 / _rails18 — top-rail rib-count multiplicity bounds.
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
    KnobGeometry,
    KnobGrip,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

__modular__ = True

# ---------------------------------------------------------------- enums
ButtstockChoice = Literal["collapsible", "fixed_A2", "side_folding", "pdw_wire"]
HandguardChoice = Literal["quad_picatinny", "smooth_tube", "mlok", "keymod"]
OpticChoice = Literal["red_dot", "scope", "iron_sights"]
MuzzleChoice = Literal["birdcage", "suppressor", "brake"]
PaletteStyle = Literal["black", "fde", "od_green", "two_tone", "gray_wolf"]

BUTTSTOCK_CHOICES: tuple[ButtstockChoice, ...] = (
    "collapsible",
    "fixed_A2",
    "side_folding",
    "pdw_wire",
)
HANDGUARD_CHOICES: tuple[HandguardChoice, ...] = (
    "quad_picatinny",
    "smooth_tube",
    "mlok",
    "keymod",
)
OPTIC_CHOICES: tuple[OpticChoice, ...] = ("red_dot", "scope", "iron_sights")
MUZZLE_CHOICES: tuple[MuzzleChoice, ...] = ("birdcage", "suppressor", "brake")
PALETTE_STYLES: tuple[PaletteStyle, ...] = (
    "black",
    "fde",
    "od_green",
    "two_tone",
    "gray_wolf",
)

# Light weighting toward the classic baselines; every value retains plenty of
# probability so the 144-combo topology pool is exercised within 0-49.
_BUTTSTOCK_WEIGHTS = (0.34, 0.24, 0.20, 0.22)
_HANDGUARD_WEIGHTS = (0.34, 0.20, 0.24, 0.22)
_OPTIC_WEIGHTS = (0.42, 0.30, 0.28)
_MUZZLE_WEIGHTS = (0.40, 0.32, 0.28)

# ---------------------------------------------------------------- constants
BORE_Z = 0.175  # bore axis height so the seated magazine just reaches the floor

MAG_CANT_DEG = 10.0  # magazine / magwell cant off vertical (bottom forward)
MAG_TRAVEL = 0.06
STOCK_TRAVEL = 0.09
# Side-folding stock crank: the arm is offset this far in -Y from the pivot so
# it lies beside (not over) the bore line when folded. Sized just under the
# knuckle bridge width so the offset arm stays welded to the on-axis knuckle.
STOCK_CRANK_Y = -0.026
CHARGING_TRAVEL = 0.07
TRIGGER_PULL_RAD = math.radians(25.0)
SELECTOR_THROW_RAD = math.pi / 2.0
STOCK_FOLD_RAD = math.radians(175.0)
SIGHT_FLIP_RAD = math.pi / 2.0

_D10 = math.radians(MAG_CANT_DEG)
_D22 = math.radians(22.0)

# World anchors for the two reanchored intermediate parts (used so that their
# children's joint origins can be expressed in the reanchored parent frame).
BARREL_ANCHOR = (0.0795, 0.0, BORE_Z)
# On the handguard tube root face / bore inner wall AND inside the receiver
# delta-ring annulus, so the FIXED joint origin sits on both parts' geometry.
HG_ANCHOR = (0.1055, 0.0, BORE_Z - 0.018)

# Handguard top-rail rib-count multiplicity bounds.
RIB_COUNT_MIN = 6
RIB_COUNT_MAX = 30
RIB_COUNT_DEFAULT = 11

# Top-rail rib placement: fixed start x and fixed first/last span (per spec).
RIB_X_START = 0.112
RIB_SPAN = 0.178


# --------------------------------------------------------------------------- #
# Palettes — every seed has >=3 distinct material colors mapped onto visuals.
# --------------------------------------------------------------------------- #

PALETTES: dict[PaletteStyle, dict[str, tuple[float, float, float, float]]] = {
    "black": {
        "receiver": (0.100, 0.100, 0.105, 1.0),
        "polymer": (0.070, 0.070, 0.075, 1.0),
        "rail": (0.135, 0.135, 0.140, 1.0),
        "barrel": (0.160, 0.160, 0.170, 1.0),
        "optic": (0.055, 0.055, 0.060, 1.0),
        "control": (0.200, 0.200, 0.210, 1.0),
        "accent": (0.040, 0.040, 0.045, 1.0),
    },
    "fde": {
        "receiver": (0.150, 0.150, 0.155, 1.0),
        "polymer": (0.560, 0.470, 0.330, 1.0),
        "rail": (0.180, 0.180, 0.185, 1.0),
        "barrel": (0.165, 0.165, 0.175, 1.0),
        "optic": (0.060, 0.060, 0.065, 1.0),
        "control": (0.210, 0.205, 0.190, 1.0),
        "accent": (0.380, 0.320, 0.230, 1.0),
    },
    "od_green": {
        "receiver": (0.150, 0.170, 0.120, 1.0),
        "polymer": (0.220, 0.260, 0.150, 1.0),
        "rail": (0.130, 0.140, 0.110, 1.0),
        "barrel": (0.150, 0.155, 0.150, 1.0),
        "optic": (0.055, 0.060, 0.050, 1.0),
        "control": (0.190, 0.200, 0.160, 1.0),
        "accent": (0.100, 0.120, 0.080, 1.0),
    },
    "two_tone": {
        "receiver": (0.110, 0.110, 0.120, 1.0),
        "polymer": (0.330, 0.300, 0.230, 1.0),
        "rail": (0.380, 0.390, 0.420, 1.0),
        "barrel": (0.170, 0.170, 0.180, 1.0),
        "optic": (0.055, 0.055, 0.060, 1.0),
        "control": (0.450, 0.460, 0.490, 1.0),
        "accent": (0.380, 0.390, 0.420, 1.0),
    },
    "gray_wolf": {
        "receiver": (0.330, 0.335, 0.350, 1.0),
        "polymer": (0.280, 0.285, 0.300, 1.0),
        "rail": (0.230, 0.235, 0.250, 1.0),
        "barrel": (0.300, 0.300, 0.315, 1.0),
        "optic": (0.080, 0.080, 0.090, 1.0),
        "control": (0.420, 0.425, 0.440, 1.0),
        "accent": (0.180, 0.185, 0.200, 1.0),
    },
}


# --------------------------------------------------------------------------- #
# Config
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class RifleConfig:
    """Public configuration sampled by ``config_from_seed`` or supplied directly."""

    buttstock_choice: ButtstockChoice = "collapsible"
    handguard_choice: HandguardChoice = "quad_picatinny"
    optic_choice: OpticChoice = "red_dot"
    muzzle_choice: MuzzleChoice = "birdcage"
    top_rail_rib_count: int = RIB_COUNT_DEFAULT
    palette_style: PaletteStyle = "black"
    stock_travel_scale: float = 1.0
    handguard_len_scale: float = 1.0
    optic_height_scale: float = 1.0
    name: str = "reference_rifle"


@dataclass(frozen=True)
class ResolvedRifleConfig:
    buttstock_choice: ButtstockChoice
    handguard_choice: HandguardChoice
    optic_choice: OpticChoice
    muzzle_choice: MuzzleChoice
    top_rail_rib_count: int  # effective N (0 when handguard has no top rail)
    palette_style: PaletteStyle
    stock_travel_scale: float
    handguard_len_scale: float
    optic_height_scale: float
    name: str
    # derived
    palette: dict[str, tuple[float, float, float, float]]
    has_top_rail: bool
    stock_travel: float
    optic_dz: float


def _clamp(value: float, lo: float, hi: float) -> float:
    if lo > hi:
        lo, hi = hi, lo
    return max(lo, min(hi, float(value)))


def _pick(value, choices):
    return value if value in choices else choices[0]


def _weighted_rib_count(rng: random.Random) -> int:
    """Per-axis weighted draw of the top-rail rib count N in [6, 30].

    Typical carbine rail lengths (N in [9,15]) dominate; the [6,8] and
    [16,30] tails are rare. Always clamped to [RIB_COUNT_MIN, RIB_COUNT_MAX].
    """
    band = rng.choices(("low", "typical", "high"), weights=(0.18, 0.62, 0.20), k=1)[0]
    if band == "low":
        n = rng.randint(6, 8)
    elif band == "typical":
        n = rng.randint(9, 15)
    else:
        n = rng.randint(16, 30)
    return int(_clamp(n, RIB_COUNT_MIN, RIB_COUNT_MAX))


def config_from_seed(seed: int) -> RifleConfig:
    """Deterministic per-seed sampling of all four slots, N, palette, scales.

    ``seed=0`` is not special. Each slot is an independent weighted draw; the
    top-rail rib count N is drawn (per-axis weighted) only when the chosen
    handguard keeps a top rail (``smooth_tube`` gates it off). Continuous
    scales are uniform within range and clamped/projected in ``resolve_config``.
    """
    rng = random.Random(seed)

    buttstock: ButtstockChoice = rng.choices(BUTTSTOCK_CHOICES, weights=_BUTTSTOCK_WEIGHTS, k=1)[0]
    handguard: HandguardChoice = rng.choices(HANDGUARD_CHOICES, weights=_HANDGUARD_WEIGHTS, k=1)[0]
    optic: OpticChoice = rng.choices(OPTIC_CHOICES, weights=_OPTIC_WEIGHTS, k=1)[0]
    muzzle: MuzzleChoice = rng.choices(MUZZLE_CHOICES, weights=_MUZZLE_WEIGHTS, k=1)[0]

    rib_count = _weighted_rib_count(rng)
    palette: PaletteStyle = rng.choice(PALETTE_STYLES)

    stock_travel_scale = round(rng.uniform(0.85, 1.15), 4)
    handguard_len_scale = round(rng.uniform(0.92, 1.08), 4)
    optic_height_scale = round(rng.uniform(0.95, 1.10), 4)

    return RifleConfig(
        buttstock_choice=buttstock,
        handguard_choice=handguard,
        optic_choice=optic,
        muzzle_choice=muzzle,
        top_rail_rib_count=rib_count,
        palette_style=palette,
        stock_travel_scale=stock_travel_scale,
        handguard_len_scale=handguard_len_scale,
        optic_height_scale=optic_height_scale,
        name=f"seeded_rifle_{seed}",
    )


def resolve_config(config: RifleConfig) -> ResolvedRifleConfig:
    if config.palette_style not in PALETTES:
        raise ValueError(f"Unsupported palette_style: {config.palette_style}")

    buttstock = _pick(config.buttstock_choice, BUTTSTOCK_CHOICES)
    handguard = _pick(config.handguard_choice, HANDGUARD_CHOICES)
    optic = _pick(config.optic_choice, OPTIC_CHOICES)
    muzzle = _pick(config.muzzle_choice, MUZZLE_CHOICES)

    has_top_rail = handguard in ("quad_picatinny", "mlok", "keymod")

    # Conditional N: gated off (0) when the handguard has no top rail.
    if has_top_rail:
        n = int(_clamp(config.top_rail_rib_count, RIB_COUNT_MIN, RIB_COUNT_MAX))
    else:
        n = 0

    stock_travel_scale = _clamp(config.stock_travel_scale, 0.85, 1.15)
    handguard_len_scale = _clamp(config.handguard_len_scale, 0.92, 1.08)
    optic_height_scale = _clamp(config.optic_height_scale, 0.95, 1.10)

    # stock_travel inequality: 0.09 * scale clamped to [0.06, 0.11]; only the
    # sliding modules (collapsible / pdw_wire) use it.
    stock_travel = _clamp(STOCK_TRAVEL * stock_travel_scale, 0.06, 0.11)

    # optic height projection: extra z offset of the optic above its rail,
    # clamped so the total height stays under the 0.27 m cap. The nominal
    # optic top sits ~0.25 m; a +z bump of (scale-1)*0.05 keeps headroom.
    optic_dz = (optic_height_scale - 1.0) * 0.05
    optic_dz = _clamp(optic_dz, -0.010, 0.018)

    return ResolvedRifleConfig(
        buttstock_choice=buttstock,
        handguard_choice=handguard,
        optic_choice=optic,
        muzzle_choice=muzzle,
        top_rail_rib_count=n,
        palette_style=config.palette_style,
        stock_travel_scale=stock_travel_scale,
        handguard_len_scale=handguard_len_scale,
        optic_height_scale=optic_height_scale,
        name=config.name,
        palette=dict(PALETTES[config.palette_style]),
        has_top_rail=has_top_rail,
        stock_travel=stock_travel,
        optic_dz=optic_dz,
    )


# --------------------------------------------------------------------------- #
# Shared geometry helpers.
# --------------------------------------------------------------------------- #


def _box_compound(boxes: list[tuple[tuple[float, float, float], tuple[float, float, float]]]):
    """Compound of axis-aligned boxes: [(size, center), ...]."""
    solids = []
    for size, center in boxes:
        solids.append(cq.Workplane("XY").box(*size).translate(center).val())
    return cq.Compound.makeCompound(solids)


def _reanchor_part(part, anchor: tuple[float, float, float]) -> None:
    """Shift every visual of ``part`` by ``-anchor``.

    Child geometry is authored in world coordinates for readability, but the
    URDF child link frame sits at the joint origin. To keep both the world
    placement correct AND the joint origin on real geometry (baseline 0.015 m
    ``fail_if_articulation_origin_far_from_geometry``), we move the child's
    local frame to ``anchor`` by subtracting it from each visual origin and
    set the joint origin to ``anchor`` at the call site. After this, the part's
    local (0,0,0) lies exactly at ``anchor`` in world space.
    """
    ax, ay, az = anchor
    for v in part.visuals:
        ox, oy, oz = v.origin.xyz
        v.origin = Origin(xyz=(ox - ax, oy - ay, oz - az), rpy=v.origin.rpy)


def _rib_xs(n: int, *, hg_len_scale: float) -> list[float]:
    """Equally spaced top-rail rib x-centers.

    Fixed start ``RIB_X_START``; the first/last span scales with the handguard
    length but is clamped so every rib (incl. the last) stays over the
    handguard tube / rail base (spec inequality: rib span <= handguard len -
    end margin), so large-N ribs never run off the end into a floating island.
    """
    # Max span so the last rib center stays within the tube end minus a 6 mm
    # margin (tube spans HG_X_START .. HG_X_START+HG_LENGTH).
    max_span = (HG_X_START + HG_LENGTH - 0.006) - RIB_X_START
    span = min(RIB_SPAN * hg_len_scale, max_span)
    if n <= 1:
        return [RIB_X_START + span / 2.0]
    step = span / (n - 1)
    return [RIB_X_START + i * step for i in range(n)]


def _rib_size_top(n: int) -> tuple[float, float, float]:
    """Per-N rib thickness for the top rail (thinner for large N so ribs stay
    visually separable, per spec rails18 note)."""
    if n > 15:
        return (0.005, 0.034, 0.006)
    return (0.0075, 0.034, 0.0075)


# --------------------------------------------------------------------------- #
# Materials
# --------------------------------------------------------------------------- #


def _materials(model: ArticulatedObject, r: ResolvedRifleConfig) -> dict:
    out = {}
    for key, rgba in r.palette.items():
        out[key] = model.material(f"rifle_{key}_{r.palette_style}", rgba=rgba)
    return out


# --------------------------------------------------------------------------- #
# Fixed spine: receiver (branches on buttstock_choice) + barrel + muzzle.
# --------------------------------------------------------------------------- #


def _build_receiver(model: ArticulatedObject, r: ResolvedRifleConfig, mats: dict):
    receiver = model.part("receiver")
    receiver_black = mats["receiver"]
    polymer_black = mats["polymer"]
    rail_black = mats["rail"]

    receiver.visual(
        Box((0.200, 0.034, 0.044)),
        origin=Origin(xyz=(-0.020, 0.0, 0.176)),
        material=receiver_black,
        name="upper_receiver",
    )
    receiver.visual(
        Box((0.200, 0.030, 0.008)),
        origin=Origin(xyz=(-0.020, 0.0, 0.2015)),
        material=rail_black,
        name="rail_base",
    )
    # flat-top picatinny ribs (one compound mesh, embedded into the rail base)
    rib_xs = [-0.106 + i * 0.0186 for i in range(10)]
    receiver.visual(
        mesh_from_cadquery(
            _box_compound([((0.0075, 0.032, 0.0075), (x, 0.0, 0.2055)) for x in rib_xs]),
            "rail_ribs",
        ),
        material=rail_black,
        name="rail_ribs",
    )
    # lower receiver with a canted pocket so the seated magazine continues up
    lower_shape = (
        cq.Workplane("XY")
        .box(0.160, 0.032, 0.043)
        .translate((-0.030, 0.0, 0.133))
        .cut(
            cq.Workplane("XY")
            .box(0.0675, 0.0235, 0.034)
            .rotate((0, 0, 0), (0, 1, 0), -MAG_CANT_DEG)
            .translate((0.025, 0.0, 0.107))
        )
    )
    receiver.visual(
        mesh_from_cadquery(lower_shape, "lower_receiver"),
        material=receiver_black,
        name="lower_receiver",
    )

    # magwell: solid flare with a canted through-cavity the magazine seats into
    magwell_shape = (
        cq.Workplane("XY")
        .box(0.076, 0.034, 0.030)
        .cut(
            cq.Workplane("XY")
            .box(0.0675, 0.0235, 0.070)
            .rotate((0, 0, 0), (0, 1, 0), -MAG_CANT_DEG)
        )
        .translate((0.025, 0.0, 0.112))
    )
    receiver.visual(
        mesh_from_cadquery(magwell_shape, "magwell"),
        material=receiver_black,
        name="magwell",
    )

    # trigger guard loop
    receiver.visual(
        Box((0.060, 0.010, 0.007)),
        origin=Origin(xyz=(-0.0425, 0.0, 0.0885)),
        material=receiver_black,
        name="guard_bar",
    )
    receiver.visual(
        Box((0.006, 0.010, 0.0245)),
        origin=Origin(xyz=(-0.016, 0.0, 0.0998)),
        material=receiver_black,
        name="guard_front_post",
    )
    receiver.visual(
        Box((0.006, 0.010, 0.0245)),
        origin=Origin(xyz=(-0.0685, 0.0, 0.0998)),
        material=receiver_black,
        name="guard_rear_post",
    )

    # --- buffer-tube branch: full tube (default) vs folding-stock stub+hinge.
    if r.buttstock_choice == "side_folding":
        # short buffer-tube stub (folding-stock adapter base) + castle nut
        receiver.visual(
            Cylinder(radius=0.015, length=0.048),
            origin=Origin(xyz=(-0.144, 0.0, 0.170), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=receiver_black,
            name="buffer_tube_stub",
        )
        receiver.visual(
            Cylinder(radius=0.0175, length=0.012),
            origin=Origin(xyz=(-0.121, 0.0, 0.170), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=receiver_black,
            name="castle_nut",
        )
        # side-folding hinge bracket on the left (+Y) receiver wall
        hinge_bracket_shape = (
            cq.Workplane("XY")
            .box(0.028, 0.020, 0.046)
            .edges("|Z")
            .fillet(0.002)
            .translate((-0.115, 0.024, 0.170))
        )
        receiver.visual(
            mesh_from_cadquery(hinge_bracket_shape, "hinge_bracket"),
            material=receiver_black,
            name="hinge_bracket",
        )
        # hinge pin (along Z, through the bracket) — the fold pivot hardware
        receiver.visual(
            Cylinder(radius=0.0035, length=0.054),
            origin=Origin(xyz=(-0.115, 0.024, 0.170)),
            material=mats["control"],
            name="hinge_pin",
        )
    else:
        # full buffer tube + castle nut (the sliding/fixed stocks ride this tube)
        receiver.visual(
            Cylinder(radius=0.015, length=0.200),
            origin=Origin(xyz=(-0.2095, 0.0, 0.170), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=receiver_black,
            name="buffer_tube",
        )
        receiver.visual(
            Cylinder(radius=0.0175, length=0.012),
            origin=Origin(xyz=(-0.121, 0.0, 0.170), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=receiver_black,
            name="castle_nut",
        )

    # angled pistol grip (static, part of the lower receiver assembly)
    receiver.visual(
        Box((0.032, 0.030, 0.008)),
        origin=Origin(xyz=(-0.076, 0.0, 0.1085)),
        material=polymer_black,
        name="grip_plate",
    )
    grip_shape = (
        cq.Workplane("XY")
        .box(0.030, 0.026, 0.105)
        .edges("|Z")
        .fillet(0.005)
        .translate((0.0, 0.0, -0.0525))
        .rotate((0, 0, 0), (0, 1, 0), 20.0)
        .translate((-0.076, 0.0, 0.108))
    )
    receiver.visual(
        mesh_from_cadquery(grip_shape, "grip_body"),
        material=polymer_black,
        name="grip_body",
    )

    # delta ring / barrel nut (annulus the barrel passes through)
    delta_ring = (
        cq.Workplane("YZ")
        .circle(0.019)
        .circle(0.0135)
        .extrude(0.0265)
        .translate((0.0795, 0.0, BORE_Z))
    )
    receiver.visual(
        mesh_from_cadquery(delta_ring, "delta_ring"),
        material=receiver_black,
        name="delta_ring",
    )
    return receiver


def _build_barrel(model: ArticulatedObject, r: ResolvedRifleConfig, mats: dict, receiver):
    barrel = model.part("barrel")
    barrel_steel = mats["barrel"]
    barrel_blank = (
        cq.Solid.makeCone(0.013, 0.010, 0.3255)
        .rotate(cq.Vector(0, 0, 0), cq.Vector(0, 1, 0), 90)
        .translate(cq.Vector(0.0795, 0.0, BORE_Z))
    )
    barrel.visual(
        mesh_from_cadquery(barrel_blank, "barrel_blank"),
        material=barrel_steel,
        name="barrel_blank",
    )
    _add_muzzle_device(barrel, r, mats)
    # Re-anchor on the delta-ring / barrel-root contact face so the FIXED joint
    # origin lies on real geometry of both parts (baseline 0.015 m tol) while
    # the world placement is preserved.
    _reanchor_part(barrel, BARREL_ANCHOR)
    model.articulation(
        "receiver_to_barrel",
        ArticulationType.FIXED,
        parent=receiver,
        child=barrel,
        origin=Origin(xyz=BARREL_ANCHOR),
    )
    return barrel


def _add_muzzle_device(barrel, r: ResolvedRifleConfig, mats: dict) -> None:
    """Slot D: a single barrel visual at the muzzle end (no joint)."""
    barrel_steel = mats["barrel"]
    if r.muzzle_choice == "suppressor":
        suppressor_mat = mats["accent"]

        def _ring(r_outer, r_inner, length, x_start):
            return (
                cq.Workplane("YZ")
                .circle(r_outer)
                .circle(r_inner)
                .extrude(length)
                .translate((x_start, 0.0, BORE_Z))
                .val()
            )

        parts: list = []
        # collar inner radius bites under the barrel OD (~0.0101 at x=0.387) so
        # the suppressor wraps and connects to the barrel_blank (no floating ring).
        parts.append(_ring(0.0215, 0.0090, 0.028, 0.387))  # thread-mount collar
        parts.append(_ring(0.0195, 0.0075, 0.155, 0.415))  # smooth tube body
        parts.append(_ring(0.0205, 0.0055, 0.007, 0.570))  # front end-cap
        for i in range(3):  # internal baffle/wipe rings
            parts.append(_ring(0.0185, 0.0080, 0.004, 0.435 + i * 0.045))
        for i in range(6):  # knurl grooves on the collar
            parts.append(_ring(0.0220, 0.0208, 0.003, 0.389 + i * 0.004))
        solid = parts[0]
        for s in parts[1:]:
            solid = solid.fuse(s)
        barrel.visual(
            mesh_from_cadquery(cq.Compound.makeCompound([solid]), "suppressor"),
            material=suppressor_mat,
            name="suppressor",
        )
    elif r.muzzle_choice == "brake":
        brake = (
            _muzzle_brake_body().rotate((0, 0, 0), (0, 1, 0), 90).translate((0.400, 0.0, BORE_Z))
        )
        barrel.visual(
            mesh_from_cadquery(brake, "muzzle_brake"),
            material=barrel_steel,
            name="muzzle_brake",
        )
    else:  # birdcage (baseline)
        hider = (
            cq.Workplane("XY")
            .circle(0.0115)
            .extrude(0.050)
            .cut(cq.Workplane("XY").box(0.040, 0.0048, 0.022).translate((0, 0, 0.0275)))
            .cut(cq.Workplane("XY").box(0.0048, 0.040, 0.022).translate((0, 0, 0.0275)))
            .cut(cq.Workplane("XY").circle(0.0045).extrude(0.030).translate((0, 0, 0.025)))
            .rotate((0, 0, 0), (0, 1, 0), 90)
            .translate((0.400, 0.0, BORE_Z))
        )
        barrel.visual(
            mesh_from_cadquery(hider, "flash_hider"),
            material=barrel_steel,
            name="flash_hider",
        )


def _muzzle_brake_body(
    body_radius: float = 0.014,
    body_length: float = 0.054,
    bore_radius: float = 0.0055,
    n_ports: int = 6,
    port_height: float = 0.018,
    port_width: float = 0.0035,
    port_start: float = 0.010,
    port_pitch: float = 0.007,
):
    """Multi-port muzzle brake with a for-loop of compensator side slots.

    Built in local coords: extrusion along +Z, side slots cut in +/-Y. The
    caller rotates 90 deg about Y and translates to the muzzle position.
    """
    body = (
        cq.Workplane("XY")
        .circle(body_radius)
        .extrude(body_length)
        .cut(cq.Workplane("XY").circle(bore_radius).extrude(body_length))
    )
    collar = (
        cq.Workplane("XY")
        .circle(body_radius + 0.002)
        .extrude(0.008)
        .cut(cq.Workplane("XY").circle(bore_radius).extrude(0.008))
    )
    body = body.union(collar)
    front_ring = (
        cq.Workplane("XY")
        .circle(body_radius + 0.001)
        .extrude(0.004)
        .cut(cq.Workplane("XY").circle(bore_radius + 0.001).extrude(0.004))
        .translate((0, 0, body_length))
    )
    body = body.union(front_ring)
    slot_cutter_width = body_radius * 2 + 0.012
    for i in range(n_ports):
        z = port_start + i * port_pitch
        slot = (
            cq.Workplane("XY").box(port_height, slot_cutter_width, port_width).translate((0, 0, z))
        )
        body = body.cut(slot)
    return body


def _build_front_sight_tower(model: ArticulatedObject, mats: dict, barrel):
    tower = model.part("front_sight_tower")
    tower_shape = (
        cq.Workplane("XZ")
        .polyline([(0.297, 0.158), (0.342, 0.158), (0.3265, 0.230), (0.3125, 0.230)])
        .close()
        .extrude(0.007, both=True)
        .union(cq.Workplane("YZ").circle(0.0145).extrude(0.030).translate((0.3045, 0.0, BORE_Z)))
        .union(cq.Workplane("XY").box(0.0045, 0.012, 0.024).translate((0.3195, 0.0, 0.240)))
        # bore cut bites under the barrel OD (~0.0109 at x=0.3045) so the clamp
        # boss grips the barrel instead of floating around it.
        .cut(cq.Workplane("YZ").circle(0.0100).extrude(0.10).translate((0.290, 0.0, BORE_Z)))
    )
    tower.visual(
        mesh_from_cadquery(tower_shape, "sight_tower"),
        material=mats["receiver"],
        name="sight_tower",
    )
    tower_anchor_world = (0.3045, 0.0, BORE_Z)
    _reanchor_part(tower, tower_anchor_world)
    # joint origin is expressed in the (reanchored) barrel local frame.
    model.articulation(
        "barrel_to_front_sight_tower",
        ArticulationType.FIXED,
        parent=barrel,
        child=tower,
        origin=Origin(
            xyz=(
                tower_anchor_world[0] - BARREL_ANCHOR[0],
                tower_anchor_world[1] - BARREL_ANCHOR[1],
                tower_anchor_world[2] - BARREL_ANCHOR[2],
            )
        ),
    )
    return tower


def _build_foregrip(model: ArticulatedObject, mats: dict, handguard):
    foregrip = model.part("vertical_foregrip")
    polymer_black = mats["polymer"]
    foregrip.visual(
        Box((0.034, 0.024, 0.010)),
        origin=Origin(xyz=(0.200, 0.0, 0.1435)),
        material=polymer_black,
        name="foregrip_mount",
    )
    fg = cq.Workplane("XY").circle(0.0135).extrude(0.095).translate((0, 0, 0.045))
    for ring_z in (0.058, 0.074, 0.090, 0.106, 0.122):
        fg = fg.union(cq.Workplane("XY").circle(0.0155).extrude(0.0065).translate((0, 0, ring_z)))
    fg = fg.union(cq.Solid.makeCone(0.008, 0.0135, 0.009).translate(cq.Vector(0, 0, 0.036)))
    fg = fg.translate((0.200, 0.0, 0.0))
    foregrip.visual(
        mesh_from_cadquery(fg, "foregrip_body"),
        material=polymer_black,
        name="foregrip_body",
    )
    # Anchor on the bottom-rail / foregrip-mount contact face.
    fg_anchor_world = (0.200, 0.0, 0.150)
    _reanchor_part(foregrip, fg_anchor_world)
    # joint origin is expressed in the (reanchored) handguard local frame.
    model.articulation(
        "handguard_to_vertical_foregrip",
        ArticulationType.FIXED,
        parent=handguard,
        child=foregrip,
        origin=Origin(
            xyz=(
                fg_anchor_world[0] - HG_ANCHOR[0],
                fg_anchor_world[1] - HG_ANCHOR[1],
                fg_anchor_world[2] - HG_ANCHOR[2],
            )
        ),
    )
    return foregrip


# --------------------------------------------------------------------------- #
# Spine controls: trigger / charging handle / magazine / safety selector.
# --------------------------------------------------------------------------- #


def _build_charging_handle(model: ArticulatedObject, mats: dict, receiver):
    charging = model.part("charging_handle")
    control_steel = mats["control"]
    charging.visual(
        Box((0.120, 0.012, 0.007)),
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
        material=control_steel,
        name="handle_shaft",
    )
    charging.visual(
        Box((0.016, 0.052, 0.0085)),
        origin=Origin(xyz=(-0.0075, 0.0, 0.0)),
        material=control_steel,
        name="handle_t_wings",
    )
    charging.visual(
        Box((0.016, 0.016, 0.0105)),
        origin=Origin(xyz=(-0.0075, 0.0, 0.001)),
        material=control_steel,
        name="handle_t_spine",
    )
    model.articulation(
        "charging_handle_slide",
        ArticulationType.PRISMATIC,
        parent=receiver,
        child=charging,
        origin=Origin(xyz=(-0.120, 0.0, 0.1935)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.8, lower=0.0, upper=CHARGING_TRAVEL),
    )
    return charging


def _build_trigger(model: ArticulatedObject, mats: dict, receiver):
    trigger = model.part("trigger")
    trigger_shape = (
        cq.Workplane("XY")
        .box(0.0075, 0.010, 0.0118)
        .translate((0.0005, 0.0, -0.0051))
        .union(
            cq.Workplane("XY")
            .box(0.007, 0.010, 0.009)
            .rotate((0, 0, 0), (0, 1, 0), 14.0)
            .translate((-0.0006, 0.0, -0.0140))
        )
    )
    trigger.visual(
        mesh_from_cadquery(trigger_shape, "trigger_blade"),
        material=mats["control"],
        name="trigger_blade",
    )
    model.articulation(
        "trigger_pull",
        ArticulationType.REVOLUTE,
        parent=receiver,
        child=trigger,
        origin=Origin(xyz=(-0.032, 0.0, 0.1115)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=4.0, lower=0.0, upper=TRIGGER_PULL_RAD),
    )
    return trigger


def _build_magazine(model: ArticulatedObject, mats: dict, receiver):
    magazine = model.part("magazine")
    polymer_black = mats["polymer"]
    d10 = (math.sin(_D10), 0.0, -math.cos(_D10))
    mag_top_center = (0.0140 * d10[0], 0.0, 0.012 + 0.0140 * d10[2])
    mag_top = (
        cq.Workplane("XY")
        .box(0.068, 0.024, 0.026)
        .rotate((0, 0, 0), (0, 1, 0), -MAG_CANT_DEG)
        .translate(mag_top_center)
    )
    magazine.visual(
        mesh_from_cadquery(mag_top, "mag_top"),
        material=polymer_black,
        name="mag_top",
    )
    top_bottom = tuple(mag_top_center[i] + 0.013 * d10[i] for i in range(3))
    c1 = tuple(top_bottom[i] + 0.018 * d10[i] for i in range(3))
    b1 = tuple(c1[i] + 0.020 * d10[i] for i in range(3))
    d22 = (math.sin(_D22), 0.0, -math.cos(_D22))
    c2 = tuple(b1[i] + 0.016 * d22[i] for i in range(3))
    b2 = tuple(c2[i] + 0.018 * d22[i] for i in range(3))
    plate_c = tuple(b2[i] - 0.001 * d22[i] for i in range(3))
    mag_body = (
        cq.Workplane("XY")
        .box(0.068, 0.024, 0.040)
        .rotate((0, 0, 0), (0, 1, 0), -MAG_CANT_DEG)
        .translate(c1)
        .union(
            cq.Workplane("XY")
            .box(0.068, 0.024, 0.036)
            .rotate((0, 0, 0), (0, 1, 0), -22.0)
            .translate(c2)
        )
        .union(
            cq.Workplane("XY")
            .box(0.071, 0.027, 0.008)
            .rotate((0, 0, 0), (0, 1, 0), -22.0)
            .translate(plate_c)
        )
    )
    magazine.visual(
        mesh_from_cadquery(mag_body, "mag_body"),
        material=polymer_black,
        name="mag_body",
    )
    model.articulation(
        "magazine_release",
        ArticulationType.PRISMATIC,
        parent=receiver,
        child=magazine,
        origin=Origin(xyz=(0.025, 0.0, 0.100)),
        axis=d10,
        motion_limits=MotionLimits(effort=20.0, velocity=0.5, lower=0.0, upper=MAG_TRAVEL),
    )
    return magazine


def _build_safety_selector(model: ArticulatedObject, mats: dict, receiver):
    selector = model.part("safety_selector")
    control_steel = mats["control"]
    selector.visual(
        Cylinder(radius=0.005, length=0.0035),
        origin=Origin(xyz=(0.0, -0.001, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=control_steel,
        name="selector_axle",
    )
    lever_shape = (
        cq.Workplane("XY")
        .box(0.030, 0.0045, 0.0085)
        .translate((0.011, 0.00125, 0.0))
        .union(cq.Workplane("XY").box(0.007, 0.0045, 0.014).translate((0.0255, 0.00125, 0.0)))
    )
    selector.visual(
        mesh_from_cadquery(lever_shape, "selector_lever"),
        material=control_steel,
        name="selector_lever",
    )
    model.articulation(
        "safety_selector_rotate",
        ArticulationType.REVOLUTE,
        parent=receiver,
        child=selector,
        origin=Origin(xyz=(-0.085, 0.0185, 0.140)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=4.0, lower=0.0, upper=SELECTOR_THROW_RAD),
    )
    return selector


# --------------------------------------------------------------------------- #
# Slot A: buttstock.
# --------------------------------------------------------------------------- #


def _build_buttstock(model: ArticulatedObject, r: ResolvedRifleConfig, mats: dict, receiver):
    if r.buttstock_choice == "fixed_A2":
        return _build_buttstock_fixed_a2(model, r, mats, receiver)
    if r.buttstock_choice == "side_folding":
        return _build_buttstock_side_folding(model, r, mats, receiver)
    if r.buttstock_choice == "pdw_wire":
        return _build_buttstock_pdw_wire(model, r, mats, receiver)
    return _build_buttstock_collapsible(model, r, mats, receiver)


def _build_buttstock_collapsible(model, r: ResolvedRifleConfig, mats: dict, receiver):
    """Six-position collapsible polymer stock sliding on the buffer tube."""
    buttstock = model.part("buttstock")
    stock_shape = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.0, 0.018),
                (-0.165, 0.018),
                (-0.165, -0.082),
                (-0.150, -0.082),
                (-0.045, -0.026),
                (0.0, -0.026),
            ]
        )
        .close()
        .extrude(0.020, both=True)
        .cut(
            cq.Workplane("XZ")
            .polyline([(-0.145, -0.070), (-0.145, -0.034), (-0.062, -0.030)])
            .close()
            .extrude(0.021, both=True)
        )
        .union(cq.Workplane("XY").box(0.012, 0.044, 0.106).translate((-0.163, 0.0, -0.032)))
        .union(cq.Workplane("XY").box(0.028, 0.013, 0.008).translate((-0.022, 0.0, -0.0285)))
        .cut(cq.Workplane("XY").box(0.009, 0.060, 0.028).translate((-0.1635, 0.0, -0.058)))
        .cut(cq.Workplane("YZ").circle(0.0145).extrude(0.180).translate((-0.175, 0.0, 0.0)))
    )
    buttstock.visual(
        mesh_from_cadquery(stock_shape, "stock_body"),
        material=mats["polymer"],
        name="stock_body",
    )
    model.articulation(
        "stock_slide",
        ArticulationType.PRISMATIC,
        parent=receiver,
        child=buttstock,
        origin=Origin(xyz=(-0.240, 0.0, 0.170)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.4, lower=0.0, upper=r.stock_travel),
    )
    return buttstock


def _build_buttstock_fixed_a2(model, r: ResolvedRifleConfig, mats: dict, receiver):
    """One-piece A2 fixed stock fully enclosing the buffer tube (FIXED joint)."""
    buttstock = model.part("buttstock")
    polymer_black = mats["polymer"]
    stock_profile = (
        cq.Workplane("XZ")
        .moveTo(-0.121, 0.190)
        .lineTo(-0.335, 0.186)
        .lineTo(-0.348, 0.178)
        .lineTo(-0.350, 0.092)
        .lineTo(-0.338, 0.080)
        .lineTo(-0.280, 0.088)
        .lineTo(-0.200, 0.100)
        .lineTo(-0.145, 0.128)
        .lineTo(-0.121, 0.148)
        .close()
    )
    stock_body = stock_profile.extrude(0.018, both=True).edges("|Y").fillet(0.004)
    stock_body = stock_body.cut(
        cq.Workplane("XY").box(0.100, 0.026, 0.005).translate((-0.245, 0.0, 0.087))
    )
    stock_body = stock_body.union(
        cq.Workplane("XY").box(0.190, 0.012, 0.004).translate((-0.225, 0.0, 0.191))
    )
    buttstock.visual(
        mesh_from_cadquery(stock_body, "stock_body"),
        material=polymer_black,
        name="stock_body",
    )
    butt_pad = cq.Workplane("YZ").rect(0.036, 0.094).extrude(0.006).translate((-0.353, 0.0, 0.135))
    buttstock.visual(
        mesh_from_cadquery(butt_pad, "butt_pad"),
        material=mats["accent"],
        name="butt_pad",
    )
    buttstock.visual(
        Cylinder(radius=0.003, length=0.010),
        origin=Origin(xyz=(-0.310, 0.0, 0.083), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=mats["control"],
        name="sling_swivel",
    )
    # Anchor on the buffer-tube / stock-front contact (bore axis rear face).
    # This A2 shell is authored in world coords, so re-anchor it to keep the
    # FIXED joint origin on real geometry.
    a2_anchor = (-0.128, 0.0, 0.170)
    _reanchor_part(buttstock, a2_anchor)
    model.articulation(
        "receiver_to_stock",
        ArticulationType.FIXED,
        parent=receiver,
        child=buttstock,
        origin=Origin(xyz=a2_anchor),
    )
    return buttstock


def _build_buttstock_side_folding(model, r: ResolvedRifleConfig, mats: dict, receiver):
    """Skeleton side-folding stock; pivots about the left-wall hinge pin (REVOLUTE).

    The stock arm is CRANKED off the pivot toward -Y (``STOCK_CRANK_Y``) so that,
    deployed, the buttpad sits on the bore centerline (world Y~0), and when the
    stock swings ~175 deg about the vertical hinge the whole arm lands on the +Y
    side and lies flat ALONGSIDE the receiver/handguard instead of sweeping over
    the bore line into them. Without the crank the arm folds straight forward over
    the handguard_tube / lower_receiver (a genuine 穿模). A short crank neck welds
    the offset arm to the on-axis knuckle so the part stays one connected mesh.
    """
    buttstock = model.part("buttstock")
    polymer_black = mats["polymer"]
    # hinge knuckle eye mating the receiver bracket pin (+ crank neck bridging to
    # the -Y-offset arm so the folded arm clears the bore-line geometry).
    stock_knuckle = (
        cq.Workplane("XY")
        .circle(0.008)
        .extrude(0.020)
        .cut(cq.Workplane("XY").circle(0.004).extrude(0.022))
        .translate((0.0, 0.0, -0.010))
    )
    buttstock.visual(
        mesh_from_cadquery(stock_knuckle, "stock_knuckle"),
        material=polymer_black,
        name="stock_knuckle",
    )
    stock_arm = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.0, 0.018),
                (-0.285, 0.018),
                (-0.285, -0.082),
                (-0.270, -0.082),
                (-0.045, -0.026),
                (0.0, -0.026),
            ]
        )
        .close()
        .extrude(0.020, both=True)
        .cut(cq.Workplane("XY").box(0.015, 0.025, 0.050).translate((-0.006, -0.028, -0.005)))
        .cut(
            cq.Workplane("XZ")
            .polyline([(-0.265, -0.070), (-0.265, -0.034), (-0.062, -0.030)])
            .close()
            .extrude(0.021, both=True)
        )
        .union(cq.Workplane("XY").box(0.012, 0.044, 0.106).translate((-0.283, 0.0, -0.032)))
        .union(cq.Workplane("XY").box(0.028, 0.013, 0.008).translate((-0.022, 0.0, -0.0285)))
        .cut(cq.Workplane("XY").box(0.009, 0.060, 0.028).translate((-0.2835, 0.0, -0.058)))
        # crank the whole arm to the -Y side of the pivot (see docstring).
        .translate((0.0, STOCK_CRANK_Y, 0.0))
        # crank neck: welds the -Y-offset arm root back to the on-axis knuckle so
        # the part stays one mesh. Built at fixed local coords (after the arm
        # translate) so it reaches Y=0; it nests against the receiver rear wall at
        # the pivot, covered by the upper_receiver<->stock_body allowance.
        .union(
            # Neck height (0.016) is kept BELOW the knuckle height (0.020) so the
            # knuckle pokes through top/bottom and the two meshes' surfaces cross
            # (a fully-enclosing neck reads as a disconnected island instead).
            cq.Workplane("XY")
            .box(0.024, abs(STOCK_CRANK_Y) + 0.030, 0.016)
            .translate((0.0, STOCK_CRANK_Y / 2.0, 0.0))
        )
    )
    buttstock.visual(
        mesh_from_cadquery(stock_arm, "stock_body"),
        material=polymer_black,
        name="stock_body",
    )
    model.articulation(
        "stock_fold",
        ArticulationType.REVOLUTE,
        parent=receiver,
        child=buttstock,
        origin=Origin(xyz=(-0.115, 0.024, 0.170)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.0, lower=0.0, upper=STOCK_FOLD_RAD),
    )
    return buttstock


def _build_buttstock_pdw_wire(model, r: ResolvedRifleConfig, mats: dict, receiver):
    """Compact skeletonized PDW wire stock sliding on the buffer tube (PRISMATIC)."""
    buttstock = model.part("buttstock")
    polymer_black = mats["polymer"]

    stock_collar = (
        cq.Workplane("XY")
        .box(0.028, 0.042, 0.038)
        .translate((-0.014, 0.0, 0.0))
        .cut(cq.Workplane("YZ").circle(0.0155).extrude(0.034).translate((-0.030, 0.0, 0.0)))
    )
    buttstock.visual(
        mesh_from_cadquery(stock_collar, "stock_collar"),
        material=polymer_black,
        name="stock_collar",
    )

    _pad_body = cq.Workplane("XY").box(0.018, 0.052, 0.064).edges("|X").fillet(0.005).val()
    _pad_bore = cq.Workplane("YZ").circle(0.016).extrude(0.024).translate((-0.012, 0.0, 0.0)).val()
    stock_pad_shape = _pad_body.cut(_pad_bore).translate(cq.Vector(-0.134, 0.0, 0.0))
    buttstock.visual(
        mesh_from_cadquery(stock_pad_shape, "stock_pad"),
        material=polymer_black,
        name="stock_pad",
    )

    for i in range(2):
        y_off = -0.018 + i * 0.036  # +/-0.018
        rail = cq.Workplane("YZ").circle(0.003).extrude(0.129).translate((-0.127, y_off, 0.0))
        buttstock.visual(
            mesh_from_cadquery(rail, f"stock_rail_{i}"),
            material=polymer_black,
            name=f"stock_rail_{i}",
        )

    model.articulation(
        "stock_slide",
        ArticulationType.PRISMATIC,
        parent=receiver,
        child=buttstock,
        origin=Origin(xyz=(-0.240, 0.0, 0.170)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.4, lower=0.0, upper=r.stock_travel),
    )
    return buttstock


# --------------------------------------------------------------------------- #
# Slot B: handguard surface (carries the top-rail rib-count multiplicity).
# --------------------------------------------------------------------------- #

HG_X_START = 0.1055
HG_LENGTH = 0.192
HG_OUTER_R = 0.027
HG_INNER_R = 0.018


def _build_handguard(model: ArticulatedObject, r: ResolvedRifleConfig, mats: dict, receiver):
    if r.handguard_choice == "smooth_tube":
        handguard = _build_handguard_smooth(model, r, mats)
    elif r.handguard_choice == "mlok":
        handguard = _build_handguard_mlok(model, r, mats)
    elif r.handguard_choice == "keymod":
        handguard = _build_handguard_keymod(model, r, mats)
    else:
        handguard = _build_handguard_quad(model, r, mats)
    # Anchor on the delta-ring / handguard-tube-root contact (bore axis, +X face).
    # NOTE: the handguard part is re-anchored by HG_ANCHOR at the END of
    # build_rifle, after the optic slot may have added a front_sight_base
    # visual to it, so every handguard visual is shifted consistently.
    model.articulation(
        "receiver_to_handguard",
        ArticulationType.FIXED,
        parent=receiver,
        child=handguard,
        origin=Origin(xyz=HG_ANCHOR),
    )
    return handguard


def _add_top_rail(handguard, r: ResolvedRifleConfig, mats: dict, z_top: float) -> None:
    """Emit the N-rib top Picatinny rail (multiplicity axis) at height z_top.

    A thin continuous base plate runs under all N ribs so they always bridge
    to one connected mesh (otherwise high-N thin ribs separate into islands,
    and round-tube modules like mlok have no flat surface to embed the ribs).
    The base also embeds into the handguard tube top.
    """
    n = r.top_rail_rib_count
    size = _rib_size_top(n)
    xs = _rib_xs(n, hg_len_scale=r.handguard_len_scale)
    rib_h = size[2]
    boxes = [(size, (x, 0.0, z_top)) for x in xs]
    # continuous base plate: spans the rib x-range, reaches from just below the
    # rib bottoms down into the tube top (z = 0.202 for every handguard module).
    x_lo = min(xs) - 0.5 * size[0]
    x_hi = max(xs) + 0.5 * size[0]
    base_top = z_top - 0.5 * rib_h + 0.001  # overlaps the rib bottoms
    base_bottom = (BORE_Z + 0.027) - 0.002  # embed 2 mm into the tube top
    base_h = max(0.003, base_top - base_bottom)
    base_cz = base_bottom + 0.5 * base_h
    boxes.append(((x_hi - x_lo, size[1], base_h), (0.5 * (x_lo + x_hi), 0.0, base_cz)))
    handguard.visual(
        mesh_from_cadquery(_box_compound(boxes), "hg_rail_top"),
        material=mats["rail"],
        name="hg_rail_top",
    )


def _build_handguard_quad(model, r: ResolvedRifleConfig, mats: dict):
    """Quad-rail Picatinny handguard: four full rib rails (top carries N)."""
    handguard = model.part("handguard")
    polymer_black = mats["polymer"]
    rail_black = mats["rail"]
    hg_tube = (
        cq.Workplane("YZ")
        .rect(0.054, 0.054)
        .circle(0.018)
        .extrude(0.192)
        .translate((HG_X_START, 0.0, BORE_Z))
    )
    handguard.visual(
        mesh_from_cadquery(hg_tube, "handguard_tube"),
        material=polymer_black,
        name="handguard_tube",
    )
    # top rail = multiplicity N
    _add_top_rail(handguard, r, mats, z_top=BORE_Z + 0.027)
    # the other three rails use the same x stations (fixed appearance baseline N).
    side_xs = _rib_xs(r.top_rail_rib_count, hg_len_scale=r.handguard_len_scale)
    bsize = _rib_size_top(r.top_rail_rib_count)
    handguard.visual(
        mesh_from_cadquery(
            _box_compound(
                [(bsize, (x, 0.0, BORE_Z - 0.027)) for x in side_xs if not (0.178 < x < 0.222)]
            ),
            "hg_rail_bottom",
        ),
        material=rail_black,
        name="hg_rail_bottom",
    )
    lr_size = (bsize[0], bsize[2], bsize[1])  # rotate cross section for side rails
    handguard.visual(
        mesh_from_cadquery(
            _box_compound([(lr_size, (x, 0.027, BORE_Z)) for x in side_xs]),
            "hg_rail_left",
        ),
        material=rail_black,
        name="hg_rail_left",
    )
    handguard.visual(
        mesh_from_cadquery(
            _box_compound([(lr_size, (x, -0.027, BORE_Z)) for x in side_xs]),
            "hg_rail_right",
        ),
        material=rail_black,
        name="hg_rail_right",
    )
    return handguard


def _build_handguard_smooth(model, r: ResolvedRifleConfig, mats: dict):
    """Smooth free-float round tube: no rails at all (N axis gated off)."""
    handguard = model.part("handguard")
    hg_tube = (
        cq.Workplane("YZ")
        .circle(0.027)
        .circle(0.019)
        .extrude(0.192)
        .translate((HG_X_START, 0.0, BORE_Z))
    )
    handguard.visual(
        mesh_from_cadquery(hg_tube, "handguard_tube"),
        material=mats["polymer"],
        name="handguard_tube",
    )
    return handguard


def _build_handguard_mlok(model, r: ResolvedRifleConfig, mats: dict):
    """Slim round M-LOK tube: oval slots on 3 faces + surviving top rail (N)."""
    handguard = model.part("handguard")
    polymer_black = mats["polymer"]

    SLOT_L = 0.033
    SLOT_W = 0.008
    SLOT_CUT_DEPTH = (HG_OUTER_R - HG_INNER_R) + 0.004
    N_SLOTS_PER_ROW = 4
    SLOT_ROW_ANGLES = (90.0, -90.0, 180.0)  # left, right, bottom
    SLOT_X_MARGIN = 0.026
    SLOT_X0 = HG_X_START + SLOT_X_MARGIN
    SLOT_X1 = HG_X_START + HG_LENGTH - SLOT_X_MARGIN
    SLOT_SPACING = (SLOT_X1 - SLOT_X0) / max(N_SLOTS_PER_ROW - 1, 1)
    R_CUT_CENTER = (HG_OUTER_R + HG_INNER_R) / 2.0

    def _cutter(x_c, angle_deg):
        return (
            cq.Workplane("XY")
            .box(SLOT_L, SLOT_W, SLOT_CUT_DEPTH)
            .translate((x_c, 0.0, BORE_Z + R_CUT_CENTER))
            .rotate((0, 0, BORE_Z), (1, 0, BORE_Z), angle_deg)
        )

    def _inset(x_c, angle_deg):
        r_pos = HG_OUTER_R - 0.001
        return (
            cq.Workplane("XY")
            .box(SLOT_L, SLOT_W, 0.002)
            .translate((x_c, 0.0, BORE_Z + r_pos))
            .rotate((0, 0, BORE_Z), (1, 0, BORE_Z), angle_deg)
        )

    hg_tube_wp = (
        cq.Workplane("YZ")
        .circle(HG_OUTER_R)
        .circle(HG_INNER_R)
        .extrude(HG_LENGTH)
        .translate((HG_X_START, 0.0, BORE_Z))
    )
    mlok_dark = mats["accent"]
    n_slot = 0
    for angle in SLOT_ROW_ANGLES:
        for i in range(N_SLOTS_PER_ROW):
            x_c = SLOT_X0 + i * SLOT_SPACING
            hg_tube_wp = hg_tube_wp.cut(_cutter(x_c, angle))
            handguard.visual(
                mesh_from_cadquery(_inset(x_c, angle), f"mlok_slot_{n_slot}"),
                material=mlok_dark,
                name=f"mlok_slot_{n_slot}",
            )
            n_slot += 1
    handguard.visual(
        mesh_from_cadquery(hg_tube_wp, "handguard_tube"),
        material=polymer_black,
        name="handguard_tube",
    )
    # surviving top Picatinny rail = multiplicity N
    _add_top_rail(handguard, r, mats, z_top=BORE_Z + HG_OUTER_R + 0.003)
    return handguard


def _build_handguard_keymod(model, r: ResolvedRifleConfig, mats: dict):
    """Rounded square KeyMod tube: keyhole slots on 3 faces + top rail (N)."""
    handguard = model.part("handguard")
    polymer_black = mats["polymer"]
    rail_black = mats["rail"]
    slot_dark = mats["accent"]

    HG_HALF = 0.027
    HG_FILLET = 0.006
    HG_BORE_R = 0.018

    def _keymod_slot(length=0.024, width=0.006, key_dia=0.008, thickness=0.003):
        pill = cq.Workplane("XY").slot2D(length, width).extrude(thickness)
        key_cx = -length / 2 + key_dia * 0.30
        key = cq.Workplane("XY").circle(key_dia / 2).extrude(thickness).translate((key_cx, 0, 0))
        return pill.union(key)

    hg_tube = (
        cq.Workplane("YZ")
        .rect(HG_HALF * 2, HG_HALF * 2)
        .extrude(HG_LENGTH)
        .edges("|X")
        .fillet(HG_FILLET)
        .cut(cq.Workplane("YZ").circle(HG_BORE_R).extrude(HG_LENGTH))
        .translate((HG_X_START, 0.0, BORE_Z))
    )

    KM_SLOT_LEN = 0.024
    KM_SLOT_W = 0.006
    KM_KEY_DIA = 0.008
    KM_N_SLOTS = 7
    KM_PITCH = 0.025
    KM_X_START = HG_X_START + (HG_LENGTH - (KM_N_SLOTS - 1) * KM_PITCH) / 2
    INNER_SKIN = 0.002
    WALL_CUT_DEPTH = (HG_HALF + 0.003) - (HG_BORE_R + INNER_SKIN)
    VIS_THICK = 0.002
    INNER_WALL_Y = HG_BORE_R + INNER_SKIN

    slot_idx = 0
    for i in range(KM_N_SLOTS):
        sx = KM_X_START + i * KM_PITCH

        cutter_l = (
            _keymod_slot(KM_SLOT_LEN, KM_SLOT_W, KM_KEY_DIA, WALL_CUT_DEPTH)
            .rotate((0, 0, 0), (1, 0, 0), 90)
            .translate((sx, HG_HALF + 0.003, BORE_Z))
        )
        hg_tube = hg_tube.cut(cutter_l)
        vis_l = (
            _keymod_slot(KM_SLOT_LEN * 0.94, KM_SLOT_W * 0.88, KM_KEY_DIA * 0.88, VIS_THICK)
            .rotate((0, 0, 0), (1, 0, 0), 90)
            .translate((sx, INNER_WALL_Y + VIS_THICK, BORE_Z))
        )
        handguard.visual(
            mesh_from_cadquery(vis_l, f"keymod_slot_{slot_idx}"),
            material=slot_dark,
            name=f"keymod_slot_{slot_idx}",
        )
        slot_idx += 1

        cutter_r = (
            _keymod_slot(KM_SLOT_LEN, KM_SLOT_W, KM_KEY_DIA, WALL_CUT_DEPTH)
            .rotate((0, 0, 0), (1, 0, 0), -90)
            .translate((sx, -(HG_HALF + 0.003), BORE_Z))
        )
        hg_tube = hg_tube.cut(cutter_r)
        vis_r = (
            _keymod_slot(KM_SLOT_LEN * 0.94, KM_SLOT_W * 0.88, KM_KEY_DIA * 0.88, VIS_THICK)
            .rotate((0, 0, 0), (1, 0, 0), -90)
            .translate((sx, -(INNER_WALL_Y + VIS_THICK), BORE_Z))
        )
        handguard.visual(
            mesh_from_cadquery(vis_r, f"keymod_slot_{slot_idx}"),
            material=slot_dark,
            name=f"keymod_slot_{slot_idx}",
        )
        slot_idx += 1

        if not (0.178 < sx < 0.222):
            cutter_b = _keymod_slot(KM_SLOT_LEN, KM_SLOT_W, KM_KEY_DIA, WALL_CUT_DEPTH).translate(
                (sx, 0.0, BORE_Z - HG_HALF - 0.003)
            )
            hg_tube = hg_tube.cut(cutter_b)
            vis_b = _keymod_slot(
                KM_SLOT_LEN * 0.94, KM_SLOT_W * 0.88, KM_KEY_DIA * 0.88, VIS_THICK
            ).translate((sx, 0.0, BORE_Z - INNER_WALL_Y - VIS_THICK))
            handguard.visual(
                mesh_from_cadquery(vis_b, f"keymod_slot_{slot_idx}"),
                material=slot_dark,
                name=f"keymod_slot_{slot_idx}",
            )
            slot_idx += 1

    handguard.visual(
        mesh_from_cadquery(hg_tube, "handguard_tube"),
        material=polymer_black,
        name="handguard_tube",
    )

    # continuous top-rail base + N ribs (multiplicity)
    rail_x_center = HG_X_START + HG_LENGTH / 2
    handguard.visual(
        Box((HG_LENGTH, 0.028, 0.005)),
        origin=Origin(xyz=(rail_x_center, 0.0, BORE_Z + HG_HALF + 0.0025)),
        material=rail_black,
        name="hg_rail_top_base",
    )
    n = r.top_rail_rib_count
    rib_w = 0.005 if n > 15 else 0.0075
    xs = _rib_xs(n, hg_len_scale=r.handguard_len_scale)
    handguard.visual(
        mesh_from_cadquery(
            _box_compound(
                [((rib_w, 0.026, 0.006), (x, 0.0, BORE_Z + HG_HALF + 0.008)) for x in xs]
            ),
            "hg_rail_top_ribs",
        ),
        material=rail_black,
        name="hg_rail_top_ribs",
    )
    return handguard


# --------------------------------------------------------------------------- #
# Slot C: optic.
# --------------------------------------------------------------------------- #


def _build_optic(model: ArticulatedObject, r: ResolvedRifleConfig, mats: dict, receiver, handguard):
    if r.optic_choice == "scope":
        return _build_optic_scope(model, r, mats, receiver)
    if r.optic_choice == "iron_sights":
        return _build_optic_iron_sights(model, r, mats, receiver, handguard)
    return _build_optic_red_dot(model, r, mats, receiver)


def _build_optic_red_dot(model, r: ResolvedRifleConfig, mats: dict, receiver):
    """Compact red-dot reflex sight clamped on the receiver top rail (FIXED)."""
    optic = model.part("reflex_sight")
    optic_black = mats["optic"]
    dz = r.optic_dz
    # The mount is a riser whose BOTTOM stays seated on the receiver rail
    # (z=0.2087, embedding ~0.6 mm into the rib top) while its TOP tracks the
    # dz-raised body, so raising the optic never lifts the mount off the rail.
    mount_bottom = 0.2087
    mount_top = 0.2167 + dz  # body bottom = 0.2227+dz - 0.0065
    mount_h = mount_top - mount_bottom
    optic.visual(
        Box((0.042, 0.030, mount_h)),
        origin=Origin(xyz=(-0.064, 0.0, 0.5 * (mount_bottom + mount_top))),
        material=optic_black,
        name="optic_mount",
    )
    optic.visual(
        Box((0.044, 0.032, 0.013)),
        origin=Origin(xyz=(-0.064, 0.0, 0.2227 + dz)),
        material=optic_black,
        name="optic_body",
    )
    hood = (
        cq.Workplane("XY")
        .box(0.012, 0.030, 0.020)
        .cut(cq.Workplane("YZ").rect(0.020, 0.013).extrude(0.05).translate((-0.025, 0.0, 0.0)))
        .translate((-0.052, 0.0, 0.2387 + dz))
    )
    optic.visual(
        mesh_from_cadquery(hood, "optic_hood"),
        material=optic_black,
        name="optic_hood",
    )
    optic.visual(
        Box((0.012, 0.030, 0.012)),
        origin=Origin(xyz=(-0.076, 0.0, 0.2347 + dz)),
        material=optic_black,
        name="optic_rear_housing",
    )
    optic.visual(
        Cylinder(radius=0.004, length=0.006),
        origin=Origin(xyz=(-0.064, -0.0175, 0.2227 + dz), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=optic_black,
        name="optic_knob",
    )
    # Anchor on the rail-rib / optic-mount contact (z normal, at the rail top).
    rd_anchor = (-0.064, 0.0, 0.209)
    _reanchor_part(optic, rd_anchor)
    model.articulation(
        "receiver_to_reflex_sight",
        ArticulationType.FIXED,
        parent=receiver,
        child=optic,
        origin=Origin(xyz=rd_anchor),
    )
    return optic


def _build_optic_scope(model, r: ResolvedRifleConfig, mats: dict, receiver):
    """Long magnified scope on two ring mounts clamped to the top rail (FIXED)."""
    scope = model.part("scope")
    optic_black = mats["optic"]
    scope_ring_mat = mats["control"]

    SCOPE_X = -0.010
    SCOPE_Z = 0.232 + r.optic_dz
    SCOPE_R = 0.014
    SCOPE_LEN = 0.140

    main_tube = (
        cq.Workplane("YZ")
        .circle(SCOPE_R)
        .extrude(SCOPE_LEN)
        .translate((SCOPE_X - SCOPE_LEN / 2, 0.0, SCOPE_Z))
    )
    obj_bell = (
        cq.Solid.makeCone(SCOPE_R, 0.020, 0.038)
        .rotate(cq.Vector(0, 0, 0), cq.Vector(0, 1, 0), 90)
        .translate(cq.Vector(SCOPE_X + SCOPE_LEN / 2, 0.0, SCOPE_Z))
    )
    eyepiece = (
        cq.Workplane("YZ")
        .circle(0.017)
        .extrude(0.032)
        .translate((SCOPE_X - SCOPE_LEN / 2 - 0.032, 0.0, SCOPE_Z))
    )
    eyecup = (
        cq.Workplane("YZ")
        .circle(0.020)
        .circle(0.016)
        .extrude(0.009)
        .translate((SCOPE_X - SCOPE_LEN / 2 - 0.041, 0.0, SCOPE_Z))
    )
    obj_rim = (
        cq.Workplane("YZ")
        .circle(0.0215)
        .circle(0.019)
        .extrude(0.006)
        .translate((SCOPE_X + SCOPE_LEN / 2 + 0.038, 0.0, SCOPE_Z))
    )
    mag_ring = (
        cq.Workplane("YZ")
        .circle(SCOPE_R + 0.003)
        .circle(SCOPE_R)
        .extrude(0.016)
        .translate((SCOPE_X - SCOPE_LEN / 2 + 0.004, 0.0, SCOPE_Z))
    )
    scope_body = (
        main_tube.union(obj_bell).union(eyepiece).union(eyecup).union(obj_rim).union(mag_ring)
    )
    scope.visual(
        mesh_from_cadquery(scope_body, "scope_body"),
        material=optic_black,
        name="scope_body",
    )

    RAIL_TOP_Z = 0.20925
    RING_XS = [SCOPE_X - 0.035, SCOPE_X + 0.035]

    def _ring_mount(center_x: float):
        base_h = 0.010
        base = (
            cq.Workplane("XY")
            .box(0.020, 0.032, base_h)
            .translate((center_x, 0.0, RAIL_TOP_Z + base_h / 2))
        )
        riser_h = SCOPE_Z - (RAIL_TOP_Z + base_h) - SCOPE_R
        if riser_h > 0.002:
            riser = (
                cq.Workplane("XY")
                .box(0.016, 0.020, riser_h)
                .translate((center_x, 0.0, RAIL_TOP_Z + base_h + riser_h / 2))
            )
            base = base.union(riser)
        ring_outer = SCOPE_R + 0.005
        ring_inner = SCOPE_R - 0.0005
        ring = (
            cq.Workplane("YZ")
            .circle(ring_outer)
            .circle(ring_inner)
            .extrude(0.020)
            .translate((center_x - 0.010, 0.0, SCOPE_Z))
        )
        screw = (
            cq.Workplane("XY")
            .circle(0.0035)
            .extrude(0.007)
            .translate((center_x, 0.0, SCOPE_Z + ring_outer - 0.002))
        )
        return base.union(ring).union(screw)

    for i, rx in enumerate(RING_XS):
        scope.visual(
            mesh_from_cadquery(_ring_mount(rx), f"ring_mount_{i}"),
            material=scope_ring_mat,
            name=f"ring_mount_{i}",
        )

    elev_turret = KnobGeometry(
        0.018,
        0.014,
        body_style="cylindrical",
        grip=KnobGrip(style="knurled", count=28, depth=0.0006),
    )
    scope.visual(
        mesh_from_geometry(elev_turret, "elevation_turret"),
        origin=Origin(xyz=(SCOPE_X + 0.002, 0.0, SCOPE_Z + SCOPE_R + 0.005)),
        material=optic_black,
        name="elevation_turret",
    )
    wind_turret = KnobGeometry(
        0.016,
        0.012,
        body_style="cylindrical",
        grip=KnobGrip(style="knurled", count=24, depth=0.0005),
    )
    scope.visual(
        mesh_from_geometry(wind_turret, "windage_turret"),
        origin=Origin(
            xyz=(SCOPE_X + 0.002, -(SCOPE_R + 0.004), SCOPE_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=optic_black,
        name="windage_turret",
    )

    # Anchor on the rail-rib / ring-mount base contact (z normal, ~rail top).
    scope_anchor = (SCOPE_X, 0.0, RAIL_TOP_Z + 0.004)
    _reanchor_part(scope, scope_anchor)
    model.articulation(
        "receiver_to_scope",
        ArticulationType.FIXED,
        parent=receiver,
        child=scope,
        origin=Origin(xyz=scope_anchor),
    )
    return scope


def _sight_base_clamp(length: float, width: float, height: float):
    return (
        cq.Workplane("XY")
        .box(length, width, height)
        .edges("|Z")
        .fillet(min(0.002, 0.4 * min(length, width)))
    )


def _build_optic_iron_sights(model, r: ResolvedRifleConfig, mats: dict, receiver, handguard):
    """Flip-up BUIS pair: rear aperture on receiver, front post on handguard.

    Two REVOLUTE flip joints about the cross-bore (Y) axis. No reflex_sight.
    Returns the rear_sight part (front_sight is also created).
    """
    rail_black = mats["rail"]
    sight_black = mats["optic"]

    _RECV_RIB_TOP = 0.20925
    # The front BUIS base seats on the handguard TUBE top (z≈0.202 for every
    # handguard module incl. smooth_tube, which has no ribs), embedding 2 mm so
    # it is never a floating island regardless of the rail surface above it.
    _HG_TUBE_TOP = BORE_Z + 0.027  # 0.202

    _rear_base_h = 0.012
    _rear_base_ctr = (-0.100, 0.0, _RECV_RIB_TOP + 0.5 * _rear_base_h)
    _rear_pivot_z = _RECV_RIB_TOP + _rear_base_h

    _front_base_h = 0.012
    _front_base_bottom = _HG_TUBE_TOP - 0.002
    _front_base_ctr = (0.250, 0.0, _front_base_bottom + 0.5 * _front_base_h)
    _front_pivot_z = _front_base_bottom + _front_base_h

    # inline non-moving base clamps (parent visuals on receiver / handguard)
    receiver.visual(
        mesh_from_cadquery(
            _sight_base_clamp(0.026, 0.032, _rear_base_h).translate(_rear_base_ctr),
            "rear_sight_base",
        ),
        material=rail_black,
        name="rear_sight_base",
    )
    handguard.visual(
        mesh_from_cadquery(
            _sight_base_clamp(0.022, 0.028, _front_base_h).translate(_front_base_ctr),
            "front_sight_base",
        ),
        material=rail_black,
        name="front_sight_base",
    )

    # --- rear aperture sight (flip-up child part) ---
    rear_sight = model.part("rear_sight")
    _rear_frame = (
        cq.Workplane("XY")
        .box(0.005, 0.005, 0.030)
        .translate((0, 0.0105, 0.015))
        .union(cq.Workplane("XY").box(0.005, 0.026, 0.008).translate((0, 0, 0.004)))
        .union(cq.Workplane("XY").box(0.005, 0.005, 0.030).translate((0, -0.0105, 0.015)))
        .union(cq.Workplane("XY").box(0.005, 0.022, 0.014).translate((0, 0, 0.022)))
        .cut(cq.Workplane("XZ").circle(0.0015).extrude(0.028).translate((0, -0.014, 0.022)))
    )
    rear_sight.visual(
        mesh_from_cadquery(_rear_frame, "rear_sight_frame"),
        material=sight_black,
        name="rear_sight_frame",
    )
    for i, y_off in enumerate((-0.012, 0.012)):
        rear_sight.visual(
            Box((0.006, 0.003, 0.008)),
            origin=Origin(xyz=(0.0, y_off, 0.008)),
            material=sight_black,
            name=f"rear_grip_{i}",
        )
    model.articulation(
        "rear_sight_flip",
        ArticulationType.REVOLUTE,
        parent=receiver,
        child=rear_sight,
        origin=Origin(xyz=(_rear_base_ctr[0], 0.0, _rear_pivot_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=5.0, lower=0.0, upper=SIGHT_FLIP_RAD),
    )

    # --- front post sight (flip-up child part) ---
    front_sight = model.part("front_sight")
    _front_body = (
        cq.Workplane("XY")
        .box(0.005, 0.008, 0.030)
        .translate((0, 0, 0.012))
        .union(cq.Workplane("XY").box(0.005, 0.026, 0.006).translate((0, 0, 0.003)))
        .union(cq.Workplane("XY").box(0.003, 0.005, 0.014).translate((0, 0, 0.026)))
    )
    front_sight.visual(
        mesh_from_cadquery(_front_body, "front_sight_body"),
        material=sight_black,
        name="front_sight_body",
    )
    for i, y_off in enumerate((-0.003, 0.003)):
        front_sight.visual(
            Cylinder(radius=0.002, length=0.005),
            origin=Origin(xyz=(0.0, y_off, 0.005), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=sight_black,
            name=f"front_knob_{i}",
        )
    # parent (handguard) is reanchored by HG_ANCHOR; express origin in its frame.
    model.articulation(
        "front_sight_flip",
        ArticulationType.REVOLUTE,
        parent=handguard,
        child=front_sight,
        origin=Origin(xyz=(_front_base_ctr[0] - HG_ANCHOR[0], 0.0, _front_pivot_z - HG_ANCHOR[2])),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=5.0, lower=0.0, upper=SIGHT_FLIP_RAD),
    )
    return rear_sight


# --------------------------------------------------------------------------- #
# Top-level build.
# --------------------------------------------------------------------------- #


def build_rifle(
    config: RifleConfig | None = None,
    *,
    assets: AssetContext | None = None,
) -> ArticulatedObject:
    config = config or RifleConfig()
    r = resolve_config(config)
    if assets is None:
        assets = AssetContext(Path(tempfile.mkdtemp(prefix="articraft-rifle-")))
    model = ArticulatedObject(name=r.name, assets=assets)
    model.meta["slot_choices"] = [list(t) for t in slot_choices_for_config(r)]

    mats = _materials(model, r)

    # --- fixed spine ------------------------------------------------------- #
    receiver = _build_receiver(model, r, mats)
    barrel = _build_barrel(model, r, mats, receiver)
    _build_front_sight_tower(model, mats, barrel)

    # --- Slot B: handguard (carries N multiplicity) ------------------------ #
    handguard = _build_handguard(model, r, mats, receiver)
    _build_foregrip(model, mats, handguard)

    # --- spine controls (always present) ----------------------------------- #
    _build_charging_handle(model, mats, receiver)
    _build_trigger(model, mats, receiver)
    _build_magazine(model, mats, receiver)
    _build_safety_selector(model, mats, receiver)

    # --- Slot A: buttstock ------------------------------------------------- #
    _build_buttstock(model, r, mats, receiver)

    # --- Slot C: optic ----------------------------------------------------- #
    _build_optic(model, r, mats, receiver, handguard)

    # --- Slot D: muzzle is a barrel visual emitted in _build_barrel -------- #

    # Re-anchor the handguard LAST: its visuals (tube + rails, plus any
    # iron-sight front_sight_base added by the optic slot) were all authored in
    # world coords; shifting them by HG_ANCHOR puts the part's local (0,0,0) on
    # the receiver_to_handguard joint origin while preserving world placement.
    _reanchor_part(handguard, HG_ANCHOR)

    return model


def build_seeded_rifle(seed: int, *, assets: AssetContext | None = None) -> ArticulatedObject:
    return build_rifle(config_from_seed(seed), assets=assets)


# --------------------------------------------------------------------------- #
# Slot choices.
# --------------------------------------------------------------------------- #


def slot_choices_for_config(r: ResolvedRifleConfig) -> tuple[tuple[str, str], ...]:
    return (
        ("buttstock", r.buttstock_choice),
        ("handguard", r.handguard_choice),
        ("optic", r.optic_choice),
        ("muzzle", r.muzzle_choice),
        ("top_rail_rib_count", str(r.top_rail_rib_count)),
    )


def slot_choices_for_seed(seed: int) -> tuple[tuple[str, str], ...]:
    return slot_choices_for_config(resolve_config(config_from_seed(seed)))


# --------------------------------------------------------------------------- #
# Tests.
# --------------------------------------------------------------------------- #


def _declare_overlap_allowances(
    ctx: TestContext, model: ArticulatedObject, r: ResolvedRifleConfig
) -> None:
    receiver = model.get_part("receiver")
    charging = model.get_part("charging_handle")
    magazine = model.get_part("magazine")
    buttstock = model.get_part("buttstock")
    barrel = model.get_part("barrel")
    tower = model.get_part("front_sight_tower")

    # front sight tower clamp boss grips the barrel (captured around the bore).
    ctx.allow_overlap(
        barrel,
        tower,
        elem_a="barrel_blank",
        elem_b="sight_tower",
        reason="front sight tower barrel-clamp boss grips the barrel around the bore",
    )

    # charging handle shaft slides captured inside the upper receiver channel
    ctx.allow_overlap(
        receiver,
        charging,
        elem_a="upper_receiver",
        elem_b="handle_shaft",
        reason="charging handle shaft is captured inside the upper receiver channel and slides rearward",
    )
    # magazine seated inside the canted magwell / lower receiver pocket
    ctx.allow_overlap(
        receiver,
        magazine,
        elem_a="magwell",
        elem_b="mag_top",
        reason="magazine top is seated inside the canted magwell with a snug insertion fit",
    )
    ctx.allow_overlap(
        receiver,
        magazine,
        elem_a="lower_receiver",
        elem_b="mag_top",
        reason="seated magazine continues up into the lower receiver mag pocket with a snug fit",
    )

    # buttstock-on-buffer-tube sliding fit (collapsible / pdw_wire),
    # fixed A2 enclosing the tube, or folding-stock hinge pin capture.
    if r.buttstock_choice == "collapsible":
        ctx.allow_overlap(
            receiver,
            buttstock,
            elem_a="buffer_tube",
            elem_b="stock_body",
            reason="six-position collapsible stock collar rides the buffer tube as a snug sliding fit",
        )
    elif r.buttstock_choice == "pdw_wire":
        for elem in ("stock_collar", "stock_pad"):
            ctx.allow_overlap(
                receiver,
                buttstock,
                elem_a="buffer_tube",
                elem_b=elem,
                reason="PDW wire stock clearance-bore collar/pad rides the buffer tube as a sliding fit",
            )
    elif r.buttstock_choice == "fixed_A2":
        ctx.allow_overlap(
            receiver,
            buttstock,
            elem_a="buffer_tube",
            elem_b="stock_body",
            reason="A2 fixed stock shell fully encloses the buffer tube + castle nut",
        )
        ctx.allow_overlap(
            receiver,
            buttstock,
            elem_a="castle_nut",
            elem_b="stock_body",
            reason="A2 fixed stock shell fully encloses the buffer tube + castle nut",
        )
    elif r.buttstock_choice == "side_folding":
        ctx.allow_overlap(
            receiver,
            buttstock,
            elem_a="hinge_pin",
            elem_b="stock_knuckle",
            reason="folding-stock knuckle eye is captured around the receiver hinge pin",
        )
        ctx.allow_overlap(
            receiver,
            buttstock,
            elem_a="hinge_pin",
            elem_b="stock_body",
            reason="folding-stock arm crank neck wraps the receiver hinge pin at the pivot",
        )
        ctx.allow_overlap(
            receiver,
            buttstock,
            elem_a="hinge_bracket",
            elem_b="stock_knuckle",
            reason="folding-stock knuckle nests inside the receiver hinge bracket",
        )
        ctx.allow_overlap(
            receiver,
            buttstock,
            elem_a="hinge_bracket",
            elem_b="stock_body",
            reason="folding-stock arm root wraps the receiver hinge bracket at the pivot",
        )
        ctx.allow_overlap(
            receiver,
            buttstock,
            elem_a="buffer_tube_stub",
            elem_b="stock_body",
            reason="extended folding-stock arm overlaps the short buffer-tube stub at rest",
        )
        ctx.allow_overlap(
            receiver,
            buttstock,
            elem_a="castle_nut",
            elem_b="stock_body",
            reason="extended folding-stock arm passes over the castle nut at rest",
        )
        ctx.allow_overlap(
            receiver,
            buttstock,
            elem_a="upper_receiver",
            elem_b="stock_body",
            reason="folded-back stock arm lies alongside the upper receiver at rest",
        )

    # iron-sight bases sit on receiver/handguard rails; the flip arms can graze them.
    if r.optic_choice == "iron_sights":
        rear_sight = model.get_part("rear_sight")
        front_sight = model.get_part("front_sight")
        handguard = model.get_part("handguard")
        ctx.allow_overlap(
            receiver,
            rear_sight,
            elem_a="rear_sight_base",
            elem_b="rear_sight_frame",
            reason="rear BUIS frame pivots on its receiver-rail base clamp",
        )
        ctx.allow_overlap(
            handguard,
            front_sight,
            elem_a="front_sight_base",
            elem_b="front_sight_body",
            reason="front BUIS post pivots on its handguard-rail base clamp",
        )


def run_rifle_tests(object_model: ArticulatedObject, config: RifleConfig) -> TestReport:
    r = resolve_config(config)
    ctx = TestContext(object_model)
    _declare_overlap_allowances(ctx, object_model, r)

    receiver = object_model.get_part("receiver")
    barrel = object_model.get_part("barrel")
    handguard = object_model.get_part("handguard")
    buttstock = object_model.get_part("buttstock")
    charging = object_model.get_part("charging_handle")
    magazine = object_model.get_part("magazine")
    part_names = {p.name for p in object_model.parts}
    joint_names = {a.name for a in object_model.articulations}

    trigger_pull = object_model.get_articulation("trigger_pull")
    charging_slide = object_model.get_articulation("charging_handle_slide")
    mag_release = object_model.get_articulation("magazine_release")
    selector_rotate = object_model.get_articulation("safety_selector_rotate")

    # ------------------------------------------------ rifle identity: spine ---
    ctx.check(
        "receiver/barrel/handguard spine present",
        {"receiver", "barrel", "handguard"} <= part_names,
        details=str(sorted(part_names)),
    )
    ctx.check(
        "barrel FIXED to receiver",
        object_model.get_articulation("receiver_to_barrel").articulation_type
        == ArticulationType.FIXED,
    )
    ctx.check(
        "handguard FIXED to receiver",
        object_model.get_articulation("receiver_to_handguard").articulation_type
        == ArticulationType.FIXED,
    )

    # ------------------------------------------------ four ever-present joints ---
    ctx.check(
        "trigger is revolute about +Y with ~25 deg pull",
        trigger_pull.articulation_type == ArticulationType.REVOLUTE
        and abs(trigger_pull.motion_limits.upper - TRIGGER_PULL_RAD) < 1e-6
        and trigger_pull.motion_limits.lower == 0.0
        and trigger_pull.axis[1] == 1.0,
    )
    ctx.check(
        "charging handle is prismatic, 0.07 m rearward along -X bore",
        charging_slide.articulation_type == ArticulationType.PRISMATIC
        and abs(charging_slide.motion_limits.upper - CHARGING_TRAVEL) < 1e-9
        and charging_slide.axis[0] == -1.0,
    )
    ctx.check(
        "magazine is prismatic, 0.06 m down a forward-canted axis",
        mag_release.articulation_type == ArticulationType.PRISMATIC
        and abs(mag_release.motion_limits.upper - MAG_TRAVEL) < 1e-9
        and mag_release.axis[2] < -0.9
        and mag_release.axis[0] > 0.1,
    )
    ctx.check(
        "safety selector is revolute about -Y with a 90 deg throw",
        selector_rotate.articulation_type == ArticulationType.REVOLUTE
        and abs(selector_rotate.motion_limits.upper - SELECTOR_THROW_RAD) < 1e-9
        and selector_rotate.axis[1] == -1.0,
    )

    # ------------------------------------------------ Slot A: buttstock topology ---
    if r.buttstock_choice in ("collapsible", "pdw_wire"):
        stock_slide = object_model.get_articulation("stock_slide")
        ctx.check(
            f"{r.buttstock_choice} stock is prismatic along +X (0..{r.stock_travel:.3f})",
            stock_slide.articulation_type == ArticulationType.PRISMATIC
            and abs(stock_slide.axis[0]) == 1.0
            and stock_slide.axis[2] == 0.0
            and abs(stock_slide.motion_limits.upper - r.stock_travel) < 1e-9,
            details=f"axis={stock_slide.axis}",
        )
        rest_stock = ctx.part_world_aabb(buttstock)
        with ctx.pose({stock_slide: r.stock_travel}):
            collapsed = ctx.part_world_aabb(buttstock)
            ctx.check(
                "stock collapses forward along the buffer tube",
                collapsed[0][0] > rest_stock[0][0] + 0.5 * r.stock_travel,
                details=f"rest xmin={rest_stock[0][0]:.4f}, collapsed xmin={collapsed[0][0]:.4f}",
            )
        ctx.check("full buffer tube present", "buffer_tube" in {v.name for v in receiver.visuals})
    elif r.buttstock_choice == "fixed_A2":
        ctx.check(
            "fixed A2 stock uses a FIXED receiver_to_stock joint",
            object_model.get_articulation("receiver_to_stock").articulation_type
            == ArticulationType.FIXED,
        )
        ctx.check(
            "no sliding/folding stock joint for fixed A2",
            "stock_slide" not in joint_names and "stock_fold" not in joint_names,
        )
    else:  # side_folding
        stock_fold = object_model.get_articulation("stock_fold")
        ctx.check(
            "side-folding stock is revolute about (0,0,-1), 0..175 deg",
            stock_fold.articulation_type == ArticulationType.REVOLUTE
            and abs(stock_fold.axis[2] + 1.0) < 1e-9
            and abs(stock_fold.motion_limits.upper - STOCK_FOLD_RAD) < 1e-6,
            details=f"axis={stock_fold.axis}",
        )
        # receiver rewrite: stub instead of full buffer tube
        recv_elems = {v.name for v in receiver.visuals}
        ctx.check(
            "folding stock rewrites receiver to a buffer-tube stub + hinge",
            "buffer_tube_stub" in recv_elems
            and "hinge_pin" in recv_elems
            and "buffer_tube" not in recv_elems,
            details=str(sorted(recv_elems)),
        )
        # fold identity: at full fold the stock swings to the left (+Y) side and
        # off the bore centerline. (True 3D non-interpenetration with the
        # magazine is enforced by the compiler-owned overlap baseline; this AABB
        # check only verifies the fold direction, not conservative AABB overlap.)
        rest_stock = ctx.part_world_aabb(buttstock)
        rest_cy = 0.5 * (rest_stock[0][1] + rest_stock[1][1])
        with ctx.pose({stock_fold: STOCK_FOLD_RAD}):
            folded = ctx.part_world_aabb(buttstock)
            folded_cy = 0.5 * (folded[0][1] + folded[1][1])
            ctx.check(
                "folded stock swings to the +Y side off the bore line",
                folded_cy > rest_cy + 0.005 and folded[1][1] > 0.03,
                details=f"rest_cy={rest_cy:.4f}, folded_cy={folded_cy:.4f}, ymax={folded[1][1]:.4f}",
            )

    # ------------------------------------------------ Slot B: handguard topology ---
    hg_elems = {v.name for v in handguard.visuals}
    if r.has_top_rail:
        top_name = "hg_rail_top_ribs" if r.handguard_choice == "keymod" else "hg_rail_top"
        ctx.check(
            f"{r.handguard_choice} keeps a top rail with N={r.top_rail_rib_count}",
            top_name in hg_elems and RIB_COUNT_MIN <= r.top_rail_rib_count <= RIB_COUNT_MAX,
            details=str(sorted(hg_elems)),
        )
    else:
        ctx.check(
            "smooth_tube has no rail elements (N gated off)",
            r.handguard_choice == "smooth_tube"
            and r.top_rail_rib_count == 0
            and not any("rail" in n for n in hg_elems),
            details=str(sorted(hg_elems)),
        )

    # ------------------------------------------------ Slot C: optic topology ---
    if r.optic_choice == "red_dot":
        ctx.check("red dot reflex_sight present", "reflex_sight" in part_names)
        ctx.check(
            "reflex sight FIXED to receiver",
            object_model.get_articulation("receiver_to_reflex_sight").articulation_type
            == ArticulationType.FIXED,
        )
        ctx.check("no iron-sight flip joints", "rear_sight_flip" not in joint_names)
    elif r.optic_choice == "scope":
        ctx.check("scope present", "scope" in part_names)
        ctx.check(
            "scope FIXED to receiver",
            object_model.get_articulation("receiver_to_scope").articulation_type
            == ArticulationType.FIXED,
        )
        ctx.check("no iron-sight flip joints", "rear_sight_flip" not in joint_names)
    else:  # iron_sights
        ctx.check(
            "iron sights add no powered optic",
            "reflex_sight" not in part_names and "scope" not in part_names,
            details=str(sorted(part_names)),
        )
        for jname, parent in (("rear_sight_flip", "receiver"), ("front_sight_flip", "handguard")):
            j = object_model.get_articulation(jname)
            ctx.check(
                f"{jname} REVOLUTE about (0,-1,0) on {parent}, 0..90 deg",
                j.articulation_type == ArticulationType.REVOLUTE
                and abs(j.axis[1] + 1.0) < 1e-9
                and j.parent == parent
                and abs(j.motion_limits.upper - SIGHT_FLIP_RAD) < 1e-6,
                details=f"axis={j.axis}, parent={j.parent}",
            )

    # ------------------------------------------------ Slot D: muzzle device ---
    barrel_elems = {v.name for v in barrel.visuals}
    expected_muzzle = {
        "birdcage": "flash_hider",
        "suppressor": "suppressor",
        "brake": "muzzle_brake",
    }[r.muzzle_choice]
    ctx.check(
        f"{r.muzzle_choice} muzzle device is a barrel visual ({expected_muzzle})",
        expected_muzzle in barrel_elems,
        details=str(sorted(barrel_elems)),
    )
    ctx.check(
        "muzzle device adds no extra joint (rides receiver_to_barrel FIXED)",
        "receiver_to_muzzle" not in joint_names,
    )

    # ------------------------------------------------ real-world scale ---
    bb_barrel = ctx.part_world_aabb(barrel)
    bb_stock = ctx.part_world_aabb(buttstock)
    oal = bb_barrel[1][0] - bb_stock[0][0]
    ctx.check(
        "overall length plausible (muzzle to butt)",
        0.55 <= oal <= 1.05,
        details=f"oal={oal:.4f}",
    )
    ctx.check(
        "muzzle device reaches the muzzle end (+X)",
        bb_barrel[1][0] > 0.44,
        details=f"barrel xmax={bb_barrel[1][0]:.4f}",
    )

    # total-height cap: optic top must stay under 0.27 m
    optic_part = None
    if "reflex_sight" in part_names:
        optic_part = object_model.get_part("reflex_sight")
    elif "scope" in part_names:
        optic_part = object_model.get_part("scope")
    if optic_part is not None:
        bb_optic = ctx.part_world_aabb(optic_part)
        ctx.check(
            "optic top stays under the 0.27 m height cap",
            bb_optic[1][2] <= 0.27,
            details=f"optic zmax={bb_optic[1][2]:.4f}",
        )

    # ------------------------------------------------ seated / mounted, not floating ---
    ctx.expect_contact(barrel, receiver, name="barrel root seats in the upper receiver")
    ctx.expect_contact(handguard, receiver, name="handguard seats against the delta ring")
    ctx.expect_contact(
        object_model.get_part("trigger"), receiver, name="trigger root enters the lower receiver"
    )

    # charging handle pulls cleanly rearward at constant height
    rest_ch = ctx.part_world_aabb(charging)
    with ctx.pose({charging_slide: CHARGING_TRAVEL}):
        pulled = ctx.part_world_aabb(charging)
        ctx.check(
            "charging handle pulls rearward ~0.07 m at constant height",
            pulled[0][0] < rest_ch[0][0] - 0.06 and abs(pulled[1][2] - rest_ch[1][2]) < 1e-6,
            details=f"rest xmin={rest_ch[0][0]:.4f}, pulled xmin={pulled[0][0]:.4f}",
        )

    # magazine drops clear when released
    rest_mag = ctx.part_world_aabb(magazine)
    with ctx.pose({mag_release: MAG_TRAVEL}):
        dropped = ctx.part_world_aabb(magazine)
        ctx.check(
            "released magazine drops down + slightly forward",
            dropped[0][2] < rest_mag[0][2] - 0.04 and dropped[0][0] > rest_mag[0][0] + 0.003,
            details=f"dropped zmin={dropped[0][2]:.4f}, xmin={dropped[0][0]:.4f}",
        )

    return ctx.report()


__all__ = [
    "RifleConfig",
    "ResolvedRifleConfig",
    "build_rifle",
    "build_seeded_rifle",
    "config_from_seed",
    "resolve_config",
    "run_rifle_tests",
    "slot_choices_for_seed",
    "__modular__",
]
