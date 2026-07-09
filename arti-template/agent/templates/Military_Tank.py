# ruff: noqa: E501
"""Modular procedural template for category ``armored_vehicle``.

Identity = **wheeled (or tracked) armored personnel carrier (APC)**: an armored
hull + a weapon station (turret / RWS / pintle) + running gear with at least one
non-fixed joint (wheel spin / turret yaw / gun elevation). This is NOT a main
battle tank — the ``tracked`` candidate keeps a tall boxy APC hull and a low
small-calibre autocannon turret rather than a low glacis + big-bore gun.

Slot graph (parallel children off a single ``hull`` root)::

    hull (root, [Slot C hull-armor])
      |- [Slot A running-gear]  {side}_wheel_{i} (CONTINUOUS lateral axle, AXLE_X
      |                          tuple loop)  |  tracked bogies + bands
      `- [Slot B weapon-station] turret(yaw CONT) -> autocannon(elev REV)
                                 |  rws_pod(yaw) -> rws_gun(elev)
                                 |  pintle_mount(yaw)

Slots / candidates
  A running-gear : wheeled_6x6 / wheeled_8x8 / wheeled_10x10 (axle_pairs N in
                   {3,4,5} via AXLE_X tuple loop, each wheel CONTINUOUS) | tracked
  B weapon-station: autocannon_turret | remote_weapon_station | open_pintle
  C hull-armor    : sloped_welded | slab_mrap | applique_panels

Multiplicity: ``axle_pairs`` (per-side wheel pairs) only applies to wheeled
running gear; tracked bogie count is its own internal copy-logic and does NOT
participate in the N sweep.

Source records (see spec Module Source Index, all under data/records/):
  parent  rec_model-an-eight-wheeled-armored-personnel-carrier...407e2520
  6x6     rec_armored_vehicle_var_6x6     10x10  rec_armored_vehicle_var_10x10
  tracked rec_armored_vehicle_var_tracked rws    rec_armored_vehicle_var_rws
  pintle  rec_armored_vehicle_var_pintle  slab   rec_armored_vehicle_var_slab
  applique rec_armored_vehicle_var_applique

Geometry adapted verbatim from those records (no downgrade): hull/turret are
CadQuery lofts, wheels are TireGeometry/WheelGeometry meshes.

Canonical spec: ``articraft_template_authoring/specs_modular_v1/Military_Tank.md``
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from math import cos, pi, sin
from typing import Literal

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    BoltPattern,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    TrunnionYokeGeometry,
    WheelBore,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_cadquery,
    mesh_from_geometry,
)

__modular__ = True

RunningGear = Literal["wheeled", "tracked"]
WeaponStation = Literal["autocannon_turret", "remote_weapon_station", "open_pintle"]
HullArmor = Literal["sloped_welded", "slab_mrap", "applique_panels"]
PaletteStyle = Literal["nato_green", "desert_tan", "woodland_camo", "urban_grey"]

RUNNING_GEARS: tuple[RunningGear, ...] = ("wheeled", "tracked")
WEAPON_STATIONS: tuple[WeaponStation, ...] = (
    "autocannon_turret",
    "remote_weapon_station",
    "open_pintle",
)
HULL_ARMORS: tuple[HullArmor, ...] = ("sloped_welded", "slab_mrap", "applique_panels")
PALETTE_STYLES: tuple[PaletteStyle, ...] = (
    "nato_green",
    "desert_tan",
    "woodland_camo",
    "urban_grey",
)

# axle_pairs N in {3,4,5}; only meaningful for wheeled running gear.
AXLE_PAIRS_MIN = 3
AXLE_PAIRS_MAX = 5

# ----------------------------------------------------------------- palettes
# >=3 palette styles; every .visual() pulls a material from the active palette.
PALETTES: dict[PaletteStyle, dict[str, tuple[float, float, float, float]]] = {
    "nato_green": {
        "body": (0.345, 0.375, 0.255, 1.0),
        "body_dark": (0.245, 0.270, 0.180, 1.0),
        "hub": (0.310, 0.355, 0.225, 1.0),
        "rubber": (0.115, 0.115, 0.125, 1.0),
        "gunmetal": (0.160, 0.170, 0.180, 1.0),
        "accent": (0.620, 0.540, 0.380, 1.0),
        "marking_light": (0.900, 0.900, 0.880, 1.0),
        "marking_red": (0.720, 0.110, 0.100, 1.0),
        "glass": (0.080, 0.095, 0.110, 1.0),
        "track": (0.220, 0.220, 0.200, 1.0),
    },
    "desert_tan": {
        "body": (0.660, 0.580, 0.420, 1.0),
        "body_dark": (0.520, 0.450, 0.320, 1.0),
        "hub": (0.600, 0.530, 0.380, 1.0),
        "rubber": (0.110, 0.110, 0.115, 1.0),
        "gunmetal": (0.200, 0.200, 0.205, 1.0),
        "accent": (0.400, 0.430, 0.300, 1.0),
        "marking_light": (0.920, 0.910, 0.870, 1.0),
        "marking_red": (0.700, 0.130, 0.110, 1.0),
        "glass": (0.090, 0.100, 0.110, 1.0),
        "track": (0.280, 0.255, 0.210, 1.0),
    },
    "woodland_camo": {
        "body": (0.270, 0.330, 0.230, 1.0),
        "body_dark": (0.165, 0.150, 0.115, 1.0),
        "hub": (0.230, 0.270, 0.190, 1.0),
        "rubber": (0.100, 0.100, 0.108, 1.0),
        "gunmetal": (0.150, 0.158, 0.165, 1.0),
        "accent": (0.430, 0.360, 0.250, 1.0),
        "marking_light": (0.870, 0.870, 0.840, 1.0),
        "marking_red": (0.680, 0.120, 0.105, 1.0),
        "glass": (0.075, 0.090, 0.105, 1.0),
        "track": (0.190, 0.195, 0.175, 1.0),
    },
    "urban_grey": {
        "body": (0.420, 0.435, 0.450, 1.0),
        "body_dark": (0.270, 0.280, 0.290, 1.0),
        "hub": (0.360, 0.375, 0.390, 1.0),
        "rubber": (0.095, 0.095, 0.100, 1.0),
        "gunmetal": (0.180, 0.190, 0.205, 1.0),
        "accent": (0.560, 0.575, 0.590, 1.0),
        "marking_light": (0.910, 0.915, 0.920, 1.0),
        "marking_red": (0.690, 0.140, 0.130, 1.0),
        "glass": (0.085, 0.095, 0.110, 1.0),
        "track": (0.230, 0.235, 0.245, 1.0),
    },
}

# --------------------------------------------------------------- dimensions
HULL_HALF_W = 1.42
HULL_BOTTOM = 0.50
HULL_ROOF = 2.20

WHEEL_RADIUS = 0.55  # ~1.1 m diameter
TIRE_CARCASS_R = 0.525
WHEEL_WIDTH = 0.35
AXLE_Z = WHEEL_RADIUS
WHEEL_Y = 1.22
ARCH_RADIUS = 0.62

# Track running gear (tracked candidate)
N_BOGEYS = 5
BOGEY_R = 0.26
BOGEY_W = 0.14
BOGEY_CZ = 0.30
SPROCKET_R = 0.30
SPROCKET_X = -2.85
SPROCKET_CZ = 0.55
IDLER_R = 0.28
IDLER_X = 2.85
IDLER_CZ = 0.42
TRACK_W = 0.40
TRACK_Y = 1.30

# Turret (autocannon_turret)
TURRET_X = 0.40
TURRET_BASE_R = 0.88
TURRET_TOP_R = 0.66
TURRET_H = 0.44

# Trunnion raised (0.23 -> 0.30) so the breech swing envelope clears the hull
# deck with a usable elevation arc (see derived motion limits below).
GUN_TRUNNION = (0.88, 0.0, 0.30)
GUN_TUBE_LEN = 3.35
GUN_TUBE_R = 0.052
BREECH_LEN = 0.50  # breech_block length along the bore
BREECH_HALF_H = 0.15  # breech_block half height / half width
BREECH_REAR_X = 0.43  # gun-local |x| of the breech rear face
BEARING_RING_H = 0.07  # turret bearing ring race height
BARREL_REACH = GUN_TUBE_LEN + 0.40  # trunnion -> muzzle end cap front face
MUZZLE_R = 0.092  # widest barrel-chain radius (muzzle device)

ELEV_LOWER = -5.0 * pi / 180.0  # depression floor; per-config limit derived below

# Engine-deck grilles on the rear roof: (x_center/hull_len_scale, size, z_center).
# Shared by the hull build and the derived gun-depression limit.
DECK_GRILLES = (
    (-2.55, (0.85, 1.05, 0.10), 2.18),
    (-3.25, (0.45, 0.85, 0.10), 2.17),
)

# RWS (remote_weapon_station)
RWS_X = 0.40
RWS_BASE_R = 0.24
RWS_BASE_H = 0.10
RWS_POD_R = 0.22
RWS_POD_H = 0.26
RWS_POD_LEN = 0.42
RWS_GUN_TRUNNION = (0.22, 0.0, 0.14)
RWS_BARREL_LEN = 1.15
RWS_BARREL_R = 0.022
RWS_BOSS_LEN = RWS_BASE_H + 0.06  # bearing boss under the rws_traverse origin
RWS_BOSS_CZ = HULL_ROOF + RWS_BASE_H / 2.0 - 0.02
RWS_RECEIVER_REAR_X = 0.18  # gun-local |x| of the gun_receiver rear face
RWS_RECEIVER_HALF_H = 0.05

# Pintle (open_pintle)
PINTLE_X = 0.80
PINTLE_Z = HULL_ROOF
RING_OUTER_R = 0.40
RING_INNER_R = 0.28
RING_H = 0.050
POST_H = 0.42
RING_TOP_Z = RING_H - 0.008
SPIDER_Z = RING_TOP_Z - 0.012
SPIDER_R = RING_INNER_R + 0.020
POST_TOP_Z = RING_TOP_Z + POST_H
PLATE_THICK = 0.018
YOKE_TRUNNION_Z = 0.16
MG_TRUNNION_Z = POST_TOP_Z + PLATE_THICK + YOKE_TRUNNION_Z  # 0.640


# ------------------------------------------------- derived weapon motion limits
# Single source (Contract 3c): the constants above drive both the built
# geometry and the elevation/depression MotionLimits, so the swept gun
# envelopes stay clear of the hull deck by construction with >= GUN_CLEARANCE
# margin (real gun mounts have exactly these breech-vs-deck stops).
GUN_CLEARANCE = 0.008


def _max_swing_elevation(rear_x: float, half_h: float, drop: float) -> float:
    """Largest elevation keeping a breech corner (``rear_x`` behind and
    ``half_h`` below the trunnion) from swinging more than ``drop`` below the
    trunnion: corner drop = rear_x*sin(t) + half_h*cos(t)."""
    reach = math.hypot(rear_x, half_h)
    phi = math.atan2(half_h, rear_x)
    return math.asin(min(1.0, drop / reach)) - phi


# Autocannon: breech rear-bottom corner must stay >= GUN_CLEARANCE above the
# hull deck (turret-local z=0 at HULL_ROOF).  ~0.359 rad (=20.6 deg).
GUN_ELEV_UPPER = _max_swing_elevation(BREECH_REAR_X, BREECH_HALF_H, GUN_TRUNNION[2] - GUN_CLEARANCE)
# RWS: receiver rear-bottom corner must stay >= GUN_CLEARANCE above the roof
# bearing boss top.  ~0.440 rad (=25.2 deg).
RWS_ELEV_UPPER = _max_swing_elevation(
    RWS_RECEIVER_REAR_X,
    RWS_RECEIVER_HALF_H,
    RWS_GUN_TRUNNION[2]
    - (RWS_BOSS_CZ + RWS_BOSS_LEN / 2.0 - (HULL_ROOF + RWS_BASE_H))
    - GUN_CLEARANCE,
)


def _bearing_ring_bore_r() -> float:
    """Bore radius of the annular turret bearing ring.

    At high elevation the breech legitimately dips below the ring-top plane
    into the ring opening (a real turret race is annular).  The bore is sized
    so the deepest-reaching breech point that crosses the ring-top plane at
    GUN_ELEV_UPPER stays radially inside the bore with clearance.
    """
    s, c = sin(GUN_ELEV_UPPER), cos(GUN_ELEV_UPPER)
    # gun-local x on the breech bottom edge (z=-BREECH_HALF_H) crossing the plane
    x_cross = (BEARING_RING_H + GUN_CLEARANCE - GUN_TRUNNION[2] + BREECH_HALF_H * c) / s
    x_cross = max(-BREECH_REAR_X, min(x_cross, 0.0))
    x_turret = GUN_TRUNNION[0] + x_cross * c + BREECH_HALF_H * s
    return math.hypot(x_turret, BREECH_HALF_H) + 0.018


def _gun_elev_lower(r: "ResolvedArmoredVehicleConfig") -> float:
    """Deepest depression keeping the barrel envelope >= GUN_CLEARANCE above
    every rear-deck obstacle at any traverse angle (worst-case yaw: obstacle
    directly in the gun's vertical plane)."""
    sx = r.hull_len_scale
    trz = HULL_ROOF + GUN_TRUNNION[2]
    if r.hull_armor == "slab_mrap":
        roof_reach = math.hypot(3.75 * sx + TURRET_X, HULL_HALF_W)
    else:
        roof_reach = math.hypot(3.55 * sx + TURRET_X, 1.18)
    obstacles = [(roof_reach, HULL_ROOF)]
    for gx, (glen, gwid, ghei), gz in DECK_GRILLES:
        obstacles.append(
            (math.hypot(abs(gx) * sx + glen / 2.0 + TURRET_X, gwid / 2.0), gz + ghei / 2.0)
        )
    lo = ELEV_LOWER
    for dist, top in obstacles:
        d = min(dist - GUN_TRUNNION[0], BARREL_REACH)
        if d > 0.0:
            lo = max(lo, math.asin(max(-1.0, (top + GUN_CLEARANCE + MUZZLE_R - trz) / d)))
    return lo


# ----------------------------------------------------------------- config
@dataclass(frozen=True)
class ArmoredVehicleConfig:
    running_gear: RunningGear | None = None
    weapon_station: WeaponStation | None = None
    hull_armor: HullArmor | None = None
    palette_style: PaletteStyle = "nato_green"
    axle_pairs: int = 4
    hull_len_scale: float = 1.0
    name: str = "armored_vehicle"


@dataclass(frozen=True)
class ResolvedArmoredVehicleConfig:
    running_gear: RunningGear
    weapon_station: WeaponStation
    hull_armor: HullArmor
    palette_style: PaletteStyle
    axle_pairs: int
    hull_len_scale: float
    name: str
    palette: dict[str, tuple[float, float, float, float]]
    axle_xs: tuple[float, ...]


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _clamp_int(value: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, int(value)))


def _choice(value: str | None, allowed: tuple[str, ...], fallback: str, field: str) -> str:
    if value is None:
        return fallback
    if value not in allowed:
        raise ValueError(f"Unsupported {field}: {value}")
    return value


def _axle_xs_for(n: int, hull_len_scale: float) -> tuple[float, ...]:
    """N evenly spaced axle pairs along X, centred on the hull, scaled by length."""
    span = 5.0 * hull_len_scale  # front..rear axle span
    if n == 1:
        return (0.0,)
    half = span / 2.0
    step = span / (n - 1)
    return tuple(round(half - i * step, 4) for i in range(n))


def config_from_seed(seed: int) -> ArmoredVehicleConfig:
    rng = random.Random(seed)
    running: RunningGear = rng.choice(RUNNING_GEARS)
    weapon: WeaponStation = rng.choice(WEAPON_STATIONS)
    hull: HullArmor = rng.choice(HULL_ARMORS)
    palette: PaletteStyle = rng.choice(PALETTE_STYLES)
    # axle_pairs only meaningful for wheeled; sampled uniformly over {3,4,5}.
    axle_pairs = rng.randint(AXLE_PAIRS_MIN, AXLE_PAIRS_MAX)
    hull_len_scale = round(rng.uniform(0.92, 1.12), 4)
    return ArmoredVehicleConfig(
        running_gear=running,
        weapon_station=weapon,
        hull_armor=hull,
        palette_style=palette,
        axle_pairs=axle_pairs,
        hull_len_scale=hull_len_scale,
        name=f"seeded_armored_vehicle_{seed}",
    )


def resolve_config(config: ArmoredVehicleConfig) -> ResolvedArmoredVehicleConfig:
    running = _choice(config.running_gear, RUNNING_GEARS, "wheeled", "running_gear")
    weapon = _choice(config.weapon_station, WEAPON_STATIONS, "autocannon_turret", "weapon_station")
    hull = _choice(config.hull_armor, HULL_ARMORS, "sloped_welded", "hull_armor")
    palette = _choice(config.palette_style, PALETTE_STYLES, "nato_green", "palette_style")
    axle_pairs = _clamp_int(config.axle_pairs, AXLE_PAIRS_MIN, AXLE_PAIRS_MAX)
    hull_len_scale = _clamp(float(config.hull_len_scale), 0.92, 1.12)
    axle_xs = _axle_xs_for(axle_pairs, hull_len_scale)
    return ResolvedArmoredVehicleConfig(
        running_gear=running,  # type: ignore[arg-type]
        weapon_station=weapon,  # type: ignore[arg-type]
        hull_armor=hull,  # type: ignore[arg-type]
        palette_style=palette,  # type: ignore[arg-type]
        axle_pairs=axle_pairs,
        hull_len_scale=hull_len_scale,
        name=config.name or "armored_vehicle",
        palette=dict(PALETTES[palette]),
        axle_xs=axle_xs,
    )


def slot_choices_for_config(
    config: ArmoredVehicleConfig | ResolvedArmoredVehicleConfig,
) -> tuple[tuple[str, str], ...]:
    r = config if isinstance(config, ResolvedArmoredVehicleConfig) else resolve_config(config)
    if r.running_gear == "wheeled":
        running_module = f"wheeled_{2 * r.axle_pairs}x{2 * r.axle_pairs}"
    else:
        running_module = "tracked"
    return (
        ("running_gear", running_module),
        ("weapon_station", r.weapon_station),
        ("hull_armor", r.hull_armor),
        ("palette", r.palette_style),
    )


def slot_choices_for_seed(seed: int) -> tuple[tuple[str, str], ...]:
    return slot_choices_for_config(config_from_seed(seed))


# --------------------------------------------------------------- materials
def _register_materials(
    model: ArticulatedObject, r: ResolvedArmoredVehicleConfig
) -> dict[str, object]:
    return {key: model.material(f"av_{key}", rgba=rgba) for key, rgba in r.palette.items()}


# --------------------------------------------------------------- hull solids
def _hull_sloped_welded(sx: float, axle_xs: tuple[float, ...], hb: float) -> cq.Workplane:
    front = 3.80 * sx
    rear = -3.80 * sx
    profile = [
        (-3.60 * sx, hb),
        (2.55 * sx, hb),
        (front, 1.30),
        (1.55 * sx, HULL_ROOF),
        (-3.55 * sx, HULL_ROOF),
        (rear, 1.40),
    ]
    body = cq.Workplane("XZ").polyline(profile).close().extrude(HULL_HALF_W, both=True)

    def yz_prism(points: list[tuple[float, float]]) -> cq.Workplane:
        return cq.Workplane("YZ").polyline(points).close().extrude(4.2, both=True)

    lower_cut = [(1.03, hb - 0.05), (HULL_HALF_W, 1.10), (1.80, 1.10), (1.80, hb - 0.05)]
    body = body.cut(yz_prism(lower_cut))
    body = body.cut(yz_prism([(-y, z) for y, z in lower_cut]))
    upper_cut = [(HULL_HALF_W, 1.80), (1.18, HULL_ROOF), (1.18, 2.50), (1.80, 2.50), (1.80, 1.80)]
    body = body.cut(yz_prism(upper_cut))
    body = body.cut(yz_prism([(-y, z) for y, z in upper_cut]))

    nose_cut = [(2.70 * sx, HULL_HALF_W), (3.90 * sx, 0.92), (4.30 * sx, 2.20), (2.70 * sx, 2.20)]
    body = body.cut(cq.Workplane("XY").polyline(nose_cut).close().extrude(3.0))
    body = body.cut(
        cq.Workplane("XY").polyline([(x, -y) for x, y in nose_cut]).close().extrude(3.0)
    )

    body = _cut_wheel_arches(body, axle_xs)
    return body


def _hull_slab_mrap(sx: float, axle_xs: tuple[float, ...], hb: float) -> cq.Workplane:
    profile = [
        (-3.75 * sx, hb),
        (3.55 * sx, hb),
        (3.78 * sx, 0.85),
        (3.75 * sx, HULL_ROOF),
        (-3.75 * sx, HULL_ROOF),
    ]
    body = cq.Workplane("XZ").polyline(profile).close().extrude(HULL_HALF_W, both=True)
    body = _cut_wheel_arches(body, axle_xs)
    return body


def _arch_radius(axle_xs: tuple[float, ...]) -> float:
    """Wheel-arch cut radius, capped so adjacent arch circles never touch.

    At axle_pairs=5 with small hull_len_scale the axle step (5.0*sx/4) drops
    below the full arch diameter; overlapping cut circles make a degenerate
    boolean that crashes OCC (NCollection_Sequence::ChangeValue in clean()).
    The cap keeps >=0.01 between arches while still clearing the 0.555 tire
    outer radius (worst case step 1.15 -> radius 0.570).
    """
    if len(axle_xs) < 2:
        return ARCH_RADIUS
    step = abs(axle_xs[0] - axle_xs[1])
    return min(ARCH_RADIUS, step / 2.0 - 0.005)


def _cut_wheel_arches(body: cq.Workplane, axle_xs: tuple[float, ...]) -> cq.Workplane:
    radius = _arch_radius(axle_xs)
    left_arches = (
        cq.Workplane("XZ")
        .workplane(offset=-1.60)
        .pushPoints([(x, AXLE_Z) for x in axle_xs])
        .circle(radius)
        .extrude(0.68)
    )
    right_arches = (
        cq.Workplane("XZ")
        .workplane(offset=0.92)
        .pushPoints([(x, AXLE_Z) for x in axle_xs])
        .circle(radius)
        .extrude(0.68)
    )
    return body.cut(left_arches).cut(right_arches)


def _build_hull_solid(r: ResolvedArmoredVehicleConfig) -> cq.Workplane:
    sx = r.hull_len_scale
    # Tracked running gear rides its road wheels higher, so raise the belly to
    # keep the hull shell clear of the bogie tires.
    hb = 0.66 if r.running_gear == "tracked" else HULL_BOTTOM
    # applique_panels rides on a sloped_welded base hull.
    if r.hull_armor == "slab_mrap":
        return _hull_slab_mrap(sx, r.axle_xs, hb)
    return _hull_sloped_welded(sx, r.axle_xs, hb)


# --------------------------------------------------------------- turret cq
def _frustum(r0: float, r1: float, z0: float, z1: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .workplane(offset=z0)
        .circle(r0)
        .workplane(offset=z1 - z0)
        .circle(r1)
        .loft()
    )


def _turret_shell() -> cq.Workplane:
    return _frustum(TURRET_BASE_R, TURRET_TOP_R, 0.0, TURRET_H)


def _camo_patches() -> cq.Workplane:
    outer = _frustum(TURRET_BASE_R + 0.022, TURRET_TOP_R + 0.022, 0.05, 0.36)
    inner = _frustum(TURRET_BASE_R - 0.005, TURRET_TOP_R - 0.005, 0.0, TURRET_H)
    shell = outer.cut(inner)

    def sector(center_deg: float, span_deg: float) -> cq.Workplane:
        a0 = (center_deg - span_deg / 2.0) * pi / 180.0
        a1 = (center_deg + span_deg / 2.0) * pi / 180.0
        am = (a0 + a1) / 2.0
        pts = [
            (0.0, 0.0),
            (1.2 * cos(a0), 1.2 * sin(a0)),
            (1.2 * cos(am), 1.2 * sin(am)),
            (1.2 * cos(a1), 1.2 * sin(a1)),
        ]
        return cq.Workplane("XY").polyline(pts).close().extrude(0.45)

    patches = shell.intersect(sector(40.0, 55.0))
    patches = patches.union(shell.intersect(sector(150.0, 45.0)))
    patches = patches.union(shell.intersect(sector(255.0, 60.0)))
    return patches


def _hatch_ring() -> cq.Workplane:
    ring = cq.Workplane("XY").workplane(offset=0.43).circle(0.25).circle(0.185).extrude(0.07)
    return ring.translate((-0.22, 0.18, 0.0))


# --------------------------------------------------------------- hull build
def _build_hull(model: ArticulatedObject, r: ResolvedArmoredVehicleConfig, mats: dict[str, object]):
    hull = model.part("hull")
    hull.visual(
        mesh_from_cadquery(_build_hull_solid(r), "av_hull"),
        material=mats["body"],
        name="hull_shell",
    )
    sx = r.hull_len_scale

    # Fender strips above the wheel arches (wheeled) / track skirts (tracked).
    if r.running_gear == "wheeled":
        for sgn, nm in ((1.0, "left_fender_strip"), (-1.0, "right_fender_strip")):
            hull.visual(
                Box((5.70 * sx, 0.07, 0.06)),
                origin=Origin(xyz=(-0.25 * sx, sgn * 1.445, 1.16)),
                material=mats["body_dark"],
                name=nm,
            )
    else:
        for sgn, nm in ((1.0, "left_track_skirt"), (-1.0, "right_track_skirt")):
            hull.visual(
                Box((5.40 * sx, 0.025, 0.55)),
                origin=Origin(xyz=(0.0, sgn * 1.445, 0.88)),
                material=mats["body_dark"],
                name=nm,
            )

    # Side door panels (mirrored furniture — not multiplicity).
    for sgn in (1.0, -1.0):
        side = "left" if sgn > 0 else "right"
        hull.visual(
            Box((0.72, 0.05, 0.40)),
            origin=Origin(xyz=(0.05, sgn * 1.43, 1.59)),
            material=mats["body_dark"],
            name=f"{side}_door_upper_panel",
        )
        hull.visual(
            Box((0.72, 0.05, 0.26)),
            origin=Origin(xyz=(0.05, sgn * 1.43, 1.26)),
            material=mats["body_dark"],
            name=f"{side}_door_lower_panel",
        )
        hull.visual(
            Box((0.14, 0.03, 0.04)),
            origin=Origin(xyz=(0.30, sgn * 1.465, 1.55)),
            material=mats["gunmetal"],
            name=f"{side}_door_handle",
        )

    # Stowage boxes.
    for sgn in (1.0, -1.0):
        side = "left" if sgn > 0 else "right"
        hull.visual(
            Box((0.85, 0.10, 0.42)),
            origin=Origin(xyz=(-1.70 * sx, sgn * 1.445, 1.44)),
            material=mats["body_dark"],
            name=f"{side}_stowage_box_mid",
        )
        hull.visual(
            Box((0.62, 0.10, 0.38)),
            origin=Origin(xyz=(-3.00 * sx, sgn * 1.445, 1.44)),
            material=mats["body_dark"],
            name=f"{side}_stowage_box_rear",
        )
        hull.visual(
            Box((0.55, 0.10, 0.34)),
            origin=Origin(xyz=(1.65 * sx, sgn * 1.445, 1.42)),
            material=mats["body_dark"],
            name=f"{side}_stowage_box_front",
        )

    # Grab rails with standoff blocks.
    for sgn in (1.0, -1.0):
        side = "left" if sgn > 0 else "right"
        for rx, rn in ((-1.70 * sx, "rear"), (0.30 * sx, "front")):
            hull.visual(
                Cylinder(radius=0.016, length=0.95),
                origin=Origin(xyz=(rx, sgn * 1.41, 1.95), rpy=(0.0, pi / 2.0, 0.0)),
                material=mats["body_dark"],
                name=f"{side}_{rn}_grab_rail",
            )
            for ex in (rx - 0.40, rx + 0.40):
                hull.visual(
                    Box((0.04, 0.12, 0.04)),
                    origin=Origin(xyz=(ex, sgn * 1.37, 1.95)),
                    material=mats["body_dark"],
                )

    # Red/white tactical marking on the right hull side.
    hull.visual(
        Box((0.30, 0.030, 0.24)),
        origin=Origin(xyz=(1.30 * sx, -1.437, 1.52)),
        material=mats["marking_light"],
        name="tactical_marking_plate",
    )
    hull.visual(
        Box((0.14, 0.036, 0.12)),
        origin=Origin(xyz=(1.30 * sx, -1.439, 1.52)),
        material=mats["marking_red"],
        name="tactical_marking_emblem",
    )

    # Front/rear surface X at a given Z depend on hull style. Furniture is
    # placed slightly INSIDE the surface so it always touches the hull mesh.
    is_slab = r.hull_armor == "slab_mrap"

    def front_surface_x(z: float) -> float:
        if is_slab:
            return 3.55 * sx  # near-vertical slab front
        if z <= 1.30:
            return 3.80 * sx
        if z >= 2.20:
            return 1.55 * sx
        frac = (z - 1.30) / (2.20 - 1.30)
        return (3.80 - frac * (3.80 - 1.55)) * sx

    def rear_surface_x(z: float) -> float:
        if is_slab:
            return -3.75 * sx
        if z >= 2.20:
            return -3.55 * sx
        if z <= 1.40:
            return -3.80 * sx
        frac = (2.20 - z) / (2.20 - 1.40)
        return (-3.55 - frac * (3.80 - 3.55)) * sx

    # Driver vision blocks: embed up into the roof (z spans across 2.20).
    for sgn in (1.0, -1.0):
        side = "left" if sgn > 0 else "right"
        hull.visual(
            Box((0.22, 0.30, 0.20)),
            origin=Origin(xyz=(1.90 * sx, sgn * 0.45, 2.14)),
            material=mats["glass"],
            name=f"{side}_vision_block",
        )
    # (Headlight + brush-guard greebles omitted: front-mounted cosmetic clusters whose
    # surface contact is unstable across the sloped/slab/tracked glacis and hull_len scaling.)
    # (Trim-vane greeble omitted: a purely-cosmetic lower-front splash guard whose
    # surface contact is unstable across the sloped/slab/tracked glacis profiles.)

    # Crew hatches on the forward roof (sit on the roof, z spanning 2.20).
    for sgn in (1.0, -1.0):
        side = "left" if sgn > 0 else "right"
        hull.visual(
            Cylinder(radius=0.21, length=0.10),
            origin=Origin(xyz=(1.28 * sx, sgn * 0.52, 2.18)),
            material=mats["body_dark"],
            name=f"{side}_crew_hatch",
        )
    # Engine deck grilles (DECK_GRILLES also feeds the derived gun-depression limit).
    for gi, (gx, gsize, gz) in enumerate(DECK_GRILLES):
        hull.visual(
            Box(gsize),
            origin=Origin(xyz=(gx * sx, 0.0, gz)),
            material=mats["body_dark"],
            name="engine_deck_grille" if gi == 0 else None,
        )
    # (Tail-light greebles omitted: cosmetic rear markers whose surface contact is
    # unstable across the receding sloped/slab rear plates.)

    if r.hull_armor == "applique_panels":
        _add_applique_panels(hull, mats, sx)

    return hull


# --------------------------------------------------------------- applique
def _add_applique_panels(hull, mats: dict[str, object], sx: float) -> None:
    """sloped_welded base hull + standoff applique armor plate array (FIXED hull visuals).

    Bosses are embedded DEEP into the hull (negative offset along the surface
    normal) so every plate is solidly connected to the hull mesh.
    """
    _AP_W, _AP_H, _AP_T = 0.60, 0.38, 0.040
    _BOSS_H, _BOSS_S = 0.12, 0.048  # taller boss so it penetrates the hull surface
    _side_xs = tuple(x * sx for x in (-2.90, -2.05, -1.20, -0.35, 0.50, 1.25))
    _side_z = 1.58
    idx = 0
    for sgn in (1.0, -1.0):
        # boss spans from inside the hull (y < HULL_HALF_W) out past it; plate on top.
        boss_cy = sgn * (HULL_HALF_W - 0.06 + _BOSS_H / 2.0)
        plate_cy = sgn * (HULL_HALF_W - 0.06 + _BOSS_H + _AP_T / 2.0 - 0.005)
        for px in _side_xs:
            for dx, dz in ((-0.22, -0.14), (0.22, -0.14), (-0.22, 0.14), (0.22, 0.14)):
                hull.visual(
                    Box((_BOSS_S, _BOSS_H, _BOSS_S)),
                    origin=Origin(xyz=(px + dx, boss_cy, _side_z + dz)),
                    material=mats["body_dark"],
                )
            hull.visual(
                Box((_AP_W, _AP_T, _AP_H)),
                origin=Origin(xyz=(px, plate_cy, _side_z)),
                material=mats["body"],
                name=f"panel_{idx}",
            )
            idx += 1

    # Glacis appliqué panels tilted to the upper-glacis slope. Rows sit ON the
    # scaled glacis surface; bosses are pushed INWARD (negative normal offset) so
    # they bury into the hull, then the plate stands proud on top of the boss.
    _gp = 0.38
    _gn_x, _gn_z = sin(_gp), cos(_gp)
    # glacis surface x at z: 3.80*sx at z=1.30 down to 1.55*sx at z=2.20
    _glacis_rows = ((2.80 * sx, 1.70), (2.20 * sx, 1.94))
    _glacis_ys = (-0.65, -0.22, 0.22, 0.65)
    _boss_in = -0.04  # push boss centre inward along the normal (embeds in hull)
    _plate_off = _BOSS_H - 0.10 + _AP_T / 2.0  # plate stands just proud of the surface
    for gx, gz in _glacis_rows:
        for gy in _glacis_ys:
            for ds, dy in ((-0.12, -0.18), (0.12, -0.18), (-0.12, 0.18), (0.12, 0.18)):
                bx = gx + ds * cos(_gp) + _gn_x * _boss_in
                bz = gz - ds * sin(_gp) + _gn_z * _boss_in
                hull.visual(
                    Box((_BOSS_S, _BOSS_S, _BOSS_H)),
                    origin=Origin(xyz=(bx, gy + dy, bz), rpy=(0.0, _gp, 0.0)),
                    material=mats["body_dark"],
                )
            pcx = gx + _gn_x * _plate_off
            pcz = gz + _gn_z * _plate_off
            hull.visual(
                Box((_AP_H, _AP_W, _AP_T)),
                origin=Origin(xyz=(pcx, gy, pcz), rpy=(0.0, _gp, 0.0)),
                material=mats["body"],
                name=f"panel_{idx}",
            )
            idx += 1


# --------------------------------------------------------- wheeled running gear
def _tire_geom() -> TireGeometry:
    return TireGeometry(
        TIRE_CARCASS_R,
        WHEEL_WIDTH,
        inner_radius=0.31,
        carcass=TireCarcass(belt_width_ratio=0.62, sidewall_bulge=0.06),
        tread=TireTread(style="block", depth=0.030, count=24, land_ratio=0.55),
        grooves=(TireGroove(center_offset=0.0, width=0.020, depth=0.012),),
        sidewall=TireSidewall(style="rounded", bulge=0.05),
        shoulder=TireShoulder(width=0.018, radius=0.008),
    )


def _wheel_geom() -> WheelGeometry:
    return WheelGeometry(
        0.315,
        0.30,
        rim=WheelRim(
            inner_radius=0.245, flange_height=0.020, flange_thickness=0.014, bead_seat_depth=0.005
        ),
        hub=WheelHub(
            radius=0.105,
            width=0.32,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=8, circle_diameter=0.150, hole_diameter=0.016),
        ),
        spokes=WheelSpokes(style="straight", count=6, thickness=0.030, window_radius=0.040),
        bore=WheelBore(style="round", diameter=0.040),
    )


def _build_wheeled(
    model: ArticulatedObject, hull, r: ResolvedArmoredVehicleConfig, mats: dict[str, object]
) -> list[str]:
    """Emit axle stubs + {side}_wheel_{i} CONTINUOUS lateral spin joints over AXLE_X tuple."""
    axle_xs = r.axle_xs
    # Axle stubs on the hull (anchor the wheel hubs). The stub spans from deep
    # inside the solid hull belly (inboard end y~0.80, inside the body between the
    # wheel arches) out to the wheel hub, so it is connected to the hull AND its
    # outer end face reaches the spin joint origin at y=WHEEL_Y.
    stub_inner_y = 0.80  # inside solid hull (tumblehome cut starts at 1.03)
    stub_outer_y = WHEEL_Y + 0.02
    stub_len = stub_outer_y - stub_inner_y
    stub_center_y = (stub_inner_y + stub_outer_y) / 2.0
    for idx, ax in enumerate(axle_xs):
        for sgn, side in ((1.0, "left"), (-1.0, "right")):
            hull.visual(
                Cylinder(radius=0.07, length=stub_len),
                origin=Origin(xyz=(ax, sgn * stub_center_y, AXLE_Z), rpy=(pi / 2.0, 0.0, 0.0)),
                material=mats["gunmetal"],
                name=f"{side}_axle_stub_{idx}",
            )

    # Meshes built once per side and shared across all wheels (perf-safe).
    tire_meshes = {
        "left": mesh_from_geometry(_tire_geom().rotate_z(pi / 2.0), "av_tire_left"),
        "right": mesh_from_geometry(_tire_geom().rotate_z(-pi / 2.0), "av_tire_right"),
    }
    wheel_meshes = {
        "left": mesh_from_geometry(_wheel_geom().rotate_z(pi / 2.0), "av_rim_left"),
        "right": mesh_from_geometry(_wheel_geom().rotate_z(-pi / 2.0), "av_rim_right"),
    }

    wheel_names: list[str] = []
    for side_sgn, side in ((1.0, "left"), (-1.0, "right")):
        for idx, ax in enumerate(axle_xs):
            wp = model.part(f"{side}_wheel_{idx}")
            wp.visual(tire_meshes[side], material=mats["rubber"], name="tire")
            wp.visual(wheel_meshes[side], material=mats["hub"], name="rim")
            # Solid hub plug spanning the rim bore through the wheel centre, so
            # the spin-joint origin (child-local 0,0,0) lies on real geometry.
            wp.visual(
                Cylinder(radius=0.10, length=WHEEL_WIDTH + 0.02),
                origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
                material=mats["hub"],
                name="hub_plug",
            )
            wp.visual(
                Cylinder(radius=0.012, length=0.08),
                origin=Origin(xyz=(0.30, side_sgn * 0.14, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
                material=mats["gunmetal"],
                name="valve_stem",
            )
            model.articulation(
                f"{side}_wheel_{idx}_spin",
                ArticulationType.CONTINUOUS,
                parent=hull,
                child=wp,
                origin=Origin(xyz=(ax, side_sgn * WHEEL_Y, AXLE_Z)),
                axis=(0.0, 1.0, 0.0),
                motion_limits=MotionLimits(effort=900.0, velocity=30.0),
            )
            wheel_names.append(f"{side}_wheel_{idx}")
    return wheel_names


# --------------------------------------------------------- tracked running gear
def _build_track_band() -> cq.Workplane:
    outer = [
        (3.18, -0.02),
        (-3.20, -0.02),
        (-3.28, 0.22),
        (-3.18, 0.58),
        (-2.98, 0.85),
        (-2.55, 0.90),
        (2.45, 0.72),
        (2.88, 0.62),
        (3.16, 0.30),
    ]
    inner = [
        (3.10, 0.04),
        (-3.12, 0.04),
        (-3.18, 0.24),
        (-3.08, 0.55),
        (-2.90, 0.79),
        (-2.51, 0.84),
        (2.41, 0.66),
        (2.82, 0.57),
        (3.08, 0.28),
    ]
    outer_solid = cq.Workplane("XZ").polyline(outer).close().extrude(TRACK_W / 2.0, both=True)
    inner_void = cq.Workplane("XZ").polyline(inner).close().extrude(TRACK_W / 2.0 + 0.10, both=True)
    return outer_solid.cut(inner_void)


def _build_sprocket() -> cq.Workplane:
    disc = cq.Workplane("XY").circle(SPROCKET_R).extrude(0.10)
    hub = cq.Workplane("XY").circle(0.08).extrude(0.14).translate((0, 0, -0.02))
    disc = disc.union(hub)
    for i in range(5):
        a = 2.0 * pi * i / 5.0
        hole = (
            cq.Workplane("XY")
            .circle(0.045)
            .extrude(0.12)
            .translate((0.16 * cos(a), 0.16 * sin(a), -0.01))
        )
        disc = disc.cut(hole)
    ring = cq.Workplane("XY").circle(SPROCKET_R + 0.025).circle(SPROCKET_R - 0.015).extrude(0.10)
    return disc.union(ring)


def _build_idler() -> cq.Workplane:
    disc = cq.Workplane("XY").circle(IDLER_R).extrude(0.08)
    hub = cq.Workplane("XY").circle(0.06).extrude(0.12).translate((0, 0, -0.02))
    disc = disc.union(hub)
    for i in range(4):
        a = 2.0 * pi * i / 4.0
        hole = (
            cq.Workplane("XY")
            .circle(0.04)
            .extrude(0.10)
            .translate((0.13 * cos(a), 0.13 * sin(a), -0.01))
        )
        disc = disc.cut(hole)
    return disc


def _road_wheel_geom() -> WheelGeometry:
    return WheelGeometry(
        0.19,
        BOGEY_W - 0.02,
        rim=WheelRim(
            inner_radius=0.145, flange_height=0.014, flange_thickness=0.010, bead_seat_depth=0.004
        ),
        hub=WheelHub(
            radius=0.060,
            width=BOGEY_W,
            cap_style="flat",
            bolt_pattern=BoltPattern(count=6, circle_diameter=0.085, hole_diameter=0.011),
        ),
        spokes=WheelSpokes(style="straight", count=5, thickness=0.018, window_radius=0.022),
        bore=WheelBore(style="round", diameter=0.028),
    )


def _road_tire_geom() -> TireGeometry:
    return TireGeometry(
        BOGEY_R,
        BOGEY_W,
        inner_radius=0.19,
        carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.03),
        tread=TireTread(style="block", depth=0.012, count=20, land_ratio=0.60),
        sidewall=TireSidewall(style="square", bulge=0.02),
        shoulder=TireShoulder(width=0.010, radius=0.004),
    )


def _build_tracked(
    model: ArticulatedObject, hull, r: ResolvedArmoredVehicleConfig, mats: dict[str, object]
) -> list[str]:
    """Two track bands + sprocket/idler/return rollers (hull visuals) + N_BOGEYS spinning bogies/side."""
    bogey_xs = tuple(2.20 - i * 1.10 for i in range(N_BOGEYS))
    track_band_mesh = mesh_from_cadquery(_build_track_band(), "av_track_band")
    sprocket_mesh = mesh_from_cadquery(_build_sprocket(), "av_drive_sprocket")
    idler_mesh = mesh_from_cadquery(_build_idler(), "av_idler_disc")

    for sgn, side in ((1.0, "left"), (-1.0, "right")):
        hull.visual(
            track_band_mesh,
            origin=Origin(xyz=(0.0, sgn * TRACK_Y, 0.0)),
            material=mats["track"],
            name=f"{side}_track_band",
        )
        hull.visual(
            sprocket_mesh,
            origin=Origin(xyz=(SPROCKET_X, sgn * TRACK_Y, SPROCKET_CZ), rpy=(pi / 2.0, 0.0, 0.0)),
            material=mats["gunmetal"],
            name=f"{side}_drive_sprocket",
        )
        hull.visual(
            idler_mesh,
            origin=Origin(xyz=(IDLER_X, sgn * TRACK_Y, IDLER_CZ), rpy=(pi / 2.0, 0.0, 0.0)),
            material=mats["gunmetal"],
            name=f"{side}_idler_disc",
        )
        hull.visual(
            Box((0.45, 0.18, 0.40)),
            origin=Origin(xyz=(SPROCKET_X + 0.30, sgn * 1.30, 0.82)),
            material=mats["body_dark"],
            name=f"{side}_final_drive_housing",
        )
        hull.visual(
            Cylinder(radius=0.04, length=0.35),
            origin=Origin(xyz=(IDLER_X - 0.20, sgn * TRACK_Y, IDLER_CZ), rpy=(0.0, pi / 2.0, 0.0)),
            material=mats["gunmetal"],
            name=f"{side}_track_tensioner",
        )
        for ri, rx in enumerate((1.30, 0.0, -1.30)):
            rz = 0.66 + (SPROCKET_CZ - IDLER_CZ) * (1.30 - rx) / 5.10
            hull.visual(
                Box((0.08, 0.62, 0.06)),
                origin=Origin(xyz=(rx, sgn * (TRACK_Y - 0.18), rz)),
                material=mats["body_dark"],
                name=f"{side}_return_roller_bracket_{ri}",
            )
            hull.visual(
                Cylinder(radius=0.06, length=0.12),
                origin=Origin(xyz=(rx, sgn * TRACK_Y, rz), rpy=(pi / 2.0, 0.0, 0.0)),
                material=mats["gunmetal"],
                name=f"{side}_return_roller_{ri}",
            )
        for idx, bx in enumerate(bogey_xs):
            # Suspension: an inboard swing-arm blade (y<=1.21, clear of the road
            # wheel inner face at y=1.23) plus an outboard bracket panel riding
            # ABOVE the tire crown (z>=0.58 > tire top 0.56) out to the track
            # skirt.  Neither element enters the wheel disc, so the off-axis
            # axle marker (orbit radius <=0.20 about the swing axle, y
            # 1.33..1.39) sweeps clear at every bogie spin angle.
            arm_top, arm_bot = 0.95, BOGEY_CZ - 0.02
            arm_h, arm_cz = arm_top - arm_bot, (arm_top + arm_bot) / 2.0
            hull.visual(
                Box((0.20, 0.08, arm_h)),
                origin=Origin(xyz=(bx, sgn * 1.17, arm_cz)),
                material=mats["body_dark"],
                name=f"{side}_suspension_arm_{idx}",
            )
            hull.visual(
                Box((0.20, 0.255, 0.37)),
                origin=Origin(xyz=(bx, sgn * 1.3275, 0.765)),
                material=mats["body_dark"],
                name=f"{side}_suspension_bracket_{idx}",
            )
            # Bump stop embedded on the arm blade, inboard of the wheel face.
            hull.visual(
                Cylinder(radius=0.035, length=0.12),
                origin=Origin(xyz=(bx, sgn * 1.17, 0.90)),
                material=mats["rubber"],
                name=f"{side}_bump_stop_{idx}",
            )
            # Bogie axle stub: from the arm at y=1.20 out to the road-wheel hub at
            # y~TRACK_Y, so the spin-joint origin sits on real suspension hardware.
            bstub_len = 0.34
            bstub_cy = (
                TRACK_Y - bstub_len / 2.0 + 0.02
            )  # inner end y=1.15 (in arm), outer y=TRACK_Y+0.02
            hull.visual(
                Cylinder(radius=0.045, length=bstub_len),
                origin=Origin(xyz=(bx, sgn * bstub_cy, BOGEY_CZ), rpy=(pi / 2.0, 0.0, 0.0)),
                material=mats["gunmetal"],
                name=f"{side}_bogie_axle_stub_{idx}",
            )

    wheel_meshes = {
        "left": mesh_from_geometry(_road_wheel_geom().rotate_z(pi / 2.0), "av_road_wheel_left"),
        "right": mesh_from_geometry(_road_wheel_geom().rotate_z(-pi / 2.0), "av_road_wheel_right"),
    }
    tire_meshes = {
        "left": mesh_from_geometry(_road_tire_geom().rotate_z(pi / 2.0), "av_road_tire_left"),
        "right": mesh_from_geometry(_road_tire_geom().rotate_z(-pi / 2.0), "av_road_tire_right"),
    }

    bogie_names: list[str] = []
    for side_sgn, side in ((1.0, "left"), (-1.0, "right")):
        for i in range(N_BOGEYS):
            bp = model.part(f"{side}_bogie_{i}")
            bp.visual(tire_meshes[side], material=mats["rubber"], name="tire")
            bp.visual(wheel_meshes[side], material=mats["hub"], name="rim")
            # Solid hub plug so the spin-joint origin (child-local 0,0,0) is on geometry.
            bp.visual(
                Cylinder(radius=0.06, length=BOGEY_W + 0.02),
                origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
                material=mats["hub"],
                name="hub_plug",
            )
            bp.visual(
                Cylinder(radius=0.010, length=0.06),
                origin=Origin(xyz=(0.18, side_sgn * 0.06, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
                material=mats["gunmetal"],
                name="axle_marker",
            )
            model.articulation(
                f"{side}_bogie_{i}_spin",
                ArticulationType.CONTINUOUS,
                parent=hull,
                child=bp,
                origin=Origin(xyz=(bogey_xs[i], side_sgn * TRACK_Y, BOGEY_CZ)),
                axis=(0.0, 1.0, 0.0),
                motion_limits=MotionLimits(effort=400.0, velocity=25.0),
            )
            bogie_names.append(f"{side}_bogie_{i}")
    return bogie_names


def _build_running_gear(
    model: ArticulatedObject, hull, r: ResolvedArmoredVehicleConfig, mats: dict[str, object]
) -> list[str]:
    if r.running_gear == "wheeled":
        return _build_wheeled(model, hull, r, mats)
    return _build_tracked(model, hull, r, mats)


# --------------------------------------------------- weapon: autocannon_turret
def _build_autocannon_turret(
    model: ArticulatedObject, hull, r: ResolvedArmoredVehicleConfig, mats: dict[str, object]
) -> None:
    turret = model.part("turret")
    turret.visual(
        mesh_from_cadquery(_turret_shell(), "av_turret_shell"),
        material=mats["body"],
        name="turret_shell",
    )
    turret.visual(
        mesh_from_cadquery(_camo_patches(), "av_turret_camo"),
        material=mats["accent"],
        name="camo_patches",
    )
    # Annular bearing race: the breech dips into the ring bore at high
    # elevation, so the ring must be a true annulus (bore from _bearing_ring_bore_r).
    ring_solid = (
        cq.Workplane("XY")
        .circle(TURRET_BASE_R + 0.05)
        .circle(_bearing_ring_bore_r())
        .extrude(BEARING_RING_H)
    )
    turret.visual(
        mesh_from_cadquery(ring_solid, "av_turret_bearing_ring"),
        material=mats["body_dark"],
        name="turret_bearing_ring",
    )
    turret.visual(
        Box((0.95, 1.28, 0.28)),
        origin=Origin(xyz=(-0.55, 0.0, 0.29)),
        material=mats["body_dark"],
        name="turret_rear_bustle_box",
    )
    for post_i, px in enumerate((-0.98, -0.68, -0.38)):
        for sgn, side in ((1.0, "left"), (-1.0, "right")):
            turret.visual(
                Box((0.055, 0.055, 0.34)),
                origin=Origin(xyz=(px, sgn * 0.69, 0.36)),
                material=mats["gunmetal"],
                name=f"{side}_basket_post_{post_i}",
            )
    turret.visual(
        Box((0.82, 0.05, 0.07)),
        origin=Origin(xyz=(-0.62, 0.69, 0.48)),
        material=mats["gunmetal"],
        name="left_bustle_basket_rail",
    )
    turret.visual(
        Box((0.82, 0.05, 0.07)),
        origin=Origin(xyz=(-0.62, -0.69, 0.48)),
        material=mats["gunmetal"],
        name="right_bustle_basket_rail",
    )
    turret.visual(
        Box((0.08, 1.42, 0.07)),
        origin=Origin(xyz=(-1.01, 0.0, 0.48)),
        material=mats["gunmetal"],
        name="rear_bustle_basket_rail",
    )
    for sgn, side in ((1.0, "left"), (-1.0, "right")):
        turret.visual(
            Box((0.34, 0.045, 0.29)),
            origin=Origin(xyz=(0.05, sgn * 0.715, 0.25)),
            material=mats["accent"],
            name=f"{side}_turret_side_camo_plate",
        )
        turret.visual(
            Box((0.20, 0.05, 0.20)),
            origin=Origin(xyz=(-0.55, sgn * 0.70, 0.35)),
            material=mats["gunmetal"],
            name=f"{side}_bustle_grille",
        )
        turret.visual(
            Box((0.48, 0.08, 0.34)),
            origin=Origin(xyz=(0.31, sgn * 0.615, 0.27)),
            material=mats["body_dark"],
            name=f"{side}_smoke_launcher_backing_plate",
        )
        for j, z in enumerate((0.18, 0.27, 0.36)):
            turret.visual(
                Cylinder(radius=0.038, length=0.34),
                origin=Origin(xyz=(0.32, sgn * 0.67, z), rpy=(0.0, pi / 2.0, 0.0)),
                material=mats["gunmetal"],
                name=f"{side}_smoke_launcher_{j}",
            )
            turret.visual(
                Box((0.08, 0.035, 0.095)),
                origin=Origin(xyz=(0.16, sgn * 0.635, z)),
                material=mats["body"],
                name=f"{side}_smoke_launcher_rear_clamp_{j}",
            )
            turret.visual(
                Box((0.08, 0.035, 0.095)),
                origin=Origin(xyz=(0.48, sgn * 0.635, z)),
                material=mats["body"],
                name=f"{side}_smoke_launcher_front_clamp_{j}",
            )
    turret.visual(
        mesh_from_cadquery(_hatch_ring(), "av_hatch_ring"),
        material=mats["body_dark"],
        name="hatch_ring",
    )
    turret.visual(
        Cylinder(radius=0.18, length=0.035),
        origin=Origin(xyz=(-0.22, 0.18, 0.475)),
        material=mats["body_dark"],
        name="hatch_lid",
    )
    turret.visual(
        Cylinder(radius=0.40, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.455)),
        material=mats["body"],
        name="turret_roof_disc",
    )
    turret.visual(
        Box((0.62, 0.70, 0.42)),
        origin=Origin(xyz=(0.82, 0.0, 0.24)),
        material=mats["body"],
        name="trunnion_housing",
    )
    turret.visual(
        Box((0.34, 0.76, 0.16)),
        origin=Origin(xyz=(0.95, 0.0, 0.41), rpy=(0.0, -0.18, 0.0)),
        material=mats["body_dark"],
        name="upper_mantlet_brow",
    )
    turret.visual(
        Box((0.30, 0.70, 0.11)),
        origin=Origin(xyz=(0.98, 0.0, 0.04), rpy=(0.0, 0.16, 0.0)),
        material=mats["body_dark"],
        name="lower_mantlet_lip",
    )
    turret.visual(
        Cylinder(radius=0.27, length=0.24),
        origin=Origin(xyz=(1.10, 0.0, GUN_TRUNNION[2]), rpy=(0.0, pi / 2.0, 0.0)),
        material=mats["body_dark"],
        name="mantlet_collaring_ring",
    )
    coax_y, coax_z = -0.45, 0.205
    turret.visual(
        Box((0.20, 0.19, 0.18)),
        origin=Origin(xyz=(0.585, coax_y + 0.02, coax_z)),
        material=mats["body_dark"],
        name="coax_mount_bracket",
    )
    turret.visual(
        Cylinder(radius=0.052, length=0.12),
        origin=Origin(xyz=(0.70, coax_y, coax_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=mats["gunmetal"],
        name="coax_cradle_clamp",
    )
    turret.visual(
        Cylinder(radius=0.038, length=0.42),
        origin=Origin(xyz=(0.93, coax_y, coax_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=mats["gunmetal"],
        name="coax_barrel_jacket",
    )
    turret.visual(
        Cylinder(radius=0.019, length=0.62),
        origin=Origin(xyz=(1.35, coax_y, coax_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=mats["gunmetal"],
        name="coax_barrel",
    )
    turret.visual(
        Cylinder(radius=0.032, length=0.11),
        origin=Origin(xyz=(1.64, coax_y, coax_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=mats["gunmetal"],
        name="coax_flash_hider",
    )
    turret.visual(
        Box((0.13, 0.11, 0.13)),
        origin=Origin(xyz=(0.73, coax_y - 0.10, coax_z - 0.015)),
        material=mats["body_dark"],
        name="coax_ammo_can",
    )
    turret.visual(
        Box((0.24, 0.18, 0.13)),
        origin=Origin(xyz=(0.32, 0.33, 0.51)),
        material=mats["body_dark"],
        name="gunner_sight",
    )
    turret.visual(
        Box((0.035, 0.12, 0.065)),
        origin=Origin(xyz=(0.45, 0.33, 0.51)),
        material=mats["glass"],
        name="gunner_sight_glass",
    )
    turret.visual(
        Box((0.10, 0.20, 0.028)),
        origin=Origin(xyz=(0.455, 0.33, 0.578), rpy=(0.0, -0.34, 0.0)),
        material=mats["body_dark"],
        name="gunner_sight_hood",
    )
    turret.visual(
        Box((0.30, 0.24, 0.045)),
        origin=Origin(xyz=(0.32, 0.33, 0.455)),
        material=mats["body"],
        name="gunner_sight_base_plate",
    )
    turret.visual(
        Cylinder(radius=0.13, length=0.055),
        origin=Origin(xyz=(-0.18, -0.38, 0.49)),
        material=mats["body_dark"],
        name="aa_mg_mount_base",
    )
    turret.visual(
        Cylinder(radius=0.045, length=0.14),
        origin=Origin(xyz=(-0.18, -0.38, 0.55)),
        material=mats["gunmetal"],
        name="aa_mg_pintle",
    )
    turret.visual(
        Box((0.08, 0.035, 0.22)),
        origin=Origin(xyz=(-0.02, -0.315, 0.62)),
        material=mats["gunmetal"],
        name="aa_mg_left_yoke",
    )
    turret.visual(
        Box((0.08, 0.035, 0.22)),
        origin=Origin(xyz=(-0.02, -0.445, 0.62)),
        material=mats["gunmetal"],
        name="aa_mg_right_yoke",
    )
    turret.visual(
        Box((0.24, 0.11, 0.10)),
        origin=Origin(xyz=(0.02, -0.38, 0.64)),
        material=mats["gunmetal"],
        name="aa_mg_receiver",
    )
    turret.visual(
        Box((0.16, 0.11, 0.16)),
        origin=Origin(xyz=(-0.13, -0.52, 0.61)),
        material=mats["body_dark"],
        name="aa_mg_ammo_box",
    )
    turret.visual(
        Box((0.12, 0.12, 0.055)),
        origin=Origin(xyz=(-0.06, -0.455, 0.63)),
        material=mats["gunmetal"],
        name="aa_mg_feed_bridge",
    )
    turret.visual(
        Cylinder(radius=0.018, length=1.00),
        origin=Origin(xyz=(0.42, -0.38, 0.70), rpy=(0.0, pi / 2.0 - 0.18, 0.0)),
        material=mats["gunmetal"],
        name="aa_mg_barrel",
    )
    turret.inertial = Inertial.from_geometry(
        Box((1.9, 1.5, 0.9)), mass=900.0, origin=Origin(xyz=(0.1, 0.0, 0.25))
    )

    model.articulation(
        "turret_traverse",
        ArticulationType.CONTINUOUS,
        parent=hull,
        child=turret,
        origin=Origin(xyz=(TURRET_X, 0.0, HULL_ROOF)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=600.0, velocity=1.5),
    )

    gun = model.part("autocannon")
    gun.visual(
        Box((BREECH_LEN, 2.0 * BREECH_HALF_H, 2.0 * BREECH_HALF_H)),
        origin=Origin(xyz=(BREECH_LEN / 2.0 - BREECH_REAR_X, 0.0, 0.0)),
        material=mats["gunmetal"],
        name="breech_block",
    )
    gun.visual(
        Cylinder(radius=0.090, length=0.52),
        origin=Origin(xyz=(0.18, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=mats["body_dark"],
        name="recoil_sleeve",
    )
    for sgn, side in ((1.0, "left"), (-1.0, "right")):
        gun.visual(
            Cylinder(radius=0.030, length=0.64),
            origin=Origin(xyz=(0.22, sgn * 0.115, -0.055), rpy=(0.0, pi / 2.0, 0.0)),
            material=mats["gunmetal"],
            name=f"{side}_recoil_rod",
        )
    gun.visual(
        Cylinder(radius=GUN_TUBE_R, length=GUN_TUBE_LEN),
        origin=Origin(xyz=(GUN_TUBE_LEN / 2.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=mats["gunmetal"],
        name="barrel_tube",
    )
    gun.visual(
        Cylinder(radius=0.080, length=1.05),
        origin=Origin(xyz=(0.70, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=mats["body_dark"],
        name="thermal_sleeve",
    )
    gun.visual(
        Cylinder(radius=0.058, length=0.34),
        origin=Origin(xyz=(1.46, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=mats["gunmetal"],
        name="fume_extractor",
    )
    gun.visual(
        Cylinder(radius=0.092, length=0.40),
        origin=Origin(xyz=(GUN_TUBE_LEN + 0.14, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=mats["gunmetal"],
        name="muzzle_device",
    )
    gun.visual(
        Cylinder(radius=0.064, length=0.060),
        origin=Origin(xyz=(GUN_TUBE_LEN + 0.35, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=mats["gunmetal"],
        name="muzzle_end_cap",
    )
    for sgn, name in ((1.0, "left"), (-1.0, "right")):
        gun.visual(
            Box((0.15, 0.022, 0.070)),
            origin=Origin(xyz=(GUN_TUBE_LEN + 0.14, sgn * 0.098, 0.0)),
            material=mats["glass"],
            name=f"{name}_muzzle_brake_port",
        )
    gun.inertial = Inertial.from_geometry(
        Box((3.8, 0.4, 0.4)), mass=180.0, origin=Origin(xyz=(1.6, 0.0, 0.0))
    )

    model.articulation(
        "gun_elevation",
        ArticulationType.REVOLUTE,
        parent=turret,
        child=gun,
        origin=Origin(xyz=GUN_TRUNNION),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=300.0, velocity=0.8, lower=_gun_elev_lower(r), upper=GUN_ELEV_UPPER
        ),
    )


# ------------------------------------------------ weapon: remote_weapon_station
def _rws_pod_shell() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .workplane(offset=-0.01)
        .rect(RWS_POD_LEN, RWS_POD_R * 2.0)
        .extrude(RWS_POD_H + 0.01)
        .edges("|Z")
        .fillet(0.04)
    )
    cradle = (
        cq.Workplane("XY")
        .workplane(offset=0.03)
        .center(RWS_POD_LEN / 2.0 + 0.06, 0.0)
        .rect(0.18, 0.16)
        .extrude(0.18)
    )
    return body.union(cradle)


def _build_remote_weapon_station(
    model: ArticulatedObject, hull, r: ResolvedArmoredVehicleConfig, mats: dict[str, object]
) -> None:
    # Fixed pedestal ring on the roof.
    pedestal = (
        cq.Workplane("XY").circle(RWS_BASE_R + 0.04).circle(RWS_BASE_R - 0.02).extrude(RWS_BASE_H)
    )
    hull.visual(
        mesh_from_cadquery(pedestal, "av_rws_pedestal"),
        origin=Origin(xyz=(RWS_X, 0.0, HULL_ROOF - 0.01)),
        material=mats["body_dark"],
        name="rws_pedestal",
    )
    # Solid bearing boss at the pedestal centre so the rws_traverse joint origin
    # sits on real hardware (the hollow ring centre would otherwise be empty).
    hull.visual(
        Cylinder(radius=0.10, length=RWS_BOSS_LEN),
        origin=Origin(xyz=(RWS_X, 0.0, RWS_BOSS_CZ)),
        material=mats["gunmetal"],
        name="rws_bearing_boss",
    )

    pod = model.part("rws_pod")
    pod.visual(
        mesh_from_cadquery(_rws_pod_shell(), "av_rws_pod"), material=mats["body"], name="pod_shell"
    )
    pod.visual(
        Cylinder(radius=RWS_BASE_R, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, -0.04)),
        material=mats["body_dark"],
        name="pod_bearing_ring",
    )
    pod.visual(
        Box((0.16, 0.18, 0.06)),
        origin=Origin(xyz=(-0.08, 0.0, RWS_POD_H + 0.02)),
        material=mats["body_dark"],
        name="sensor_housing",
    )
    pod.visual(
        Box((0.14, 0.10, 0.025)),
        origin=Origin(xyz=(-0.07, -0.05, RWS_POD_H - 0.005)),
        material=mats["glass"],
        name="sensor_window",
    )
    # Sensor cluster on the pod's UPPER forward face (above the gun line at
    # z=0.14 so it clears the gun), each visual straddling the shell surface.
    _pf = RWS_POD_LEN / 2.0  # forward face x
    pod.visual(
        Cylinder(radius=0.025, length=0.06),
        origin=Origin(xyz=(_pf - 0.02, 0.10, 0.23), rpy=(0.0, pi / 2.0, 0.0)),
        material=mats["glass"],
        name="day_camera",
    )
    pod.visual(
        Cylinder(radius=0.020, length=0.06),
        origin=Origin(xyz=(_pf - 0.02, -0.10, 0.23), rpy=(0.0, pi / 2.0, 0.0)),
        material=mats["glass"],
        name="thermal_camera",
    )
    pod.visual(
        Box((0.05, 0.04, 0.03)),
        origin=Origin(xyz=(_pf - 0.01, 0.0, 0.24)),
        material=mats["glass"],
        name="lrf_window",
    )
    # Cradle bearings straddle the gun trunnion axis (x=RWS_GUN_TRUNNION[0]) so
    # the rws_gun_elevation joint origin sits on the bearing hardware.
    _crd_x = RWS_GUN_TRUNNION[0]
    pod.visual(
        Cylinder(radius=0.040, length=0.06),
        origin=Origin(xyz=(_crd_x, 0.08, 0.14), rpy=(pi / 2.0, 0.0, 0.0)),
        material=mats["gunmetal"],
        name="left_cradle_bearing",
    )
    pod.visual(
        Cylinder(radius=0.040, length=0.06),
        origin=Origin(xyz=(_crd_x, -0.08, 0.14), rpy=(pi / 2.0, 0.0, 0.0)),
        material=mats["gunmetal"],
        name="right_cradle_bearing",
    )
    # Trunnion cross-pin bridging the two cradle bearings at the elevation axis.
    pod.visual(
        Cylinder(radius=0.022, length=0.16),
        origin=Origin(xyz=(_crd_x, 0.0, 0.14), rpy=(pi / 2.0, 0.0, 0.0)),
        material=mats["gunmetal"],
        name="cradle_trunnion_pin",
    )
    pod.visual(
        Box((0.12, 0.04, 0.10)),
        origin=Origin(xyz=(0.0, RWS_POD_R - 0.005, 0.13)),
        material=mats["accent"],
        name="pod_side_marking",
    )
    pod.visual(
        Box((0.18, 0.14, 0.14)),
        origin=Origin(xyz=(-0.05, -(RWS_POD_R + 0.02), 0.10)),
        material=mats["body_dark"],
        name="ammo_box",
    )
    pod.visual(
        Box((0.10, 0.06, 0.05)),
        origin=Origin(xyz=(0.06, -(RWS_POD_R - 0.01), 0.14)),
        material=mats["gunmetal"],
        name="ammo_chute",
    )
    pod.inertial = Inertial.from_geometry(
        Box((0.6, 0.5, 0.3)), mass=120.0, origin=Origin(xyz=(0.0, 0.0, 0.13))
    )

    model.articulation(
        "rws_traverse",
        ArticulationType.CONTINUOUS,
        parent=hull,
        child=pod,
        origin=Origin(xyz=(RWS_X, 0.0, HULL_ROOF + RWS_BASE_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=200.0, velocity=2.0),
    )

    gun = model.part("rws_gun")
    gun.visual(
        Box((0.26, 0.10, 2.0 * RWS_RECEIVER_HALF_H)),
        origin=Origin(xyz=(0.13 - RWS_RECEIVER_REAR_X, 0.0, 0.0)),
        material=mats["gunmetal"],
        name="gun_receiver",
    )
    gun.visual(
        Cylinder(radius=0.032, length=0.40),
        origin=Origin(xyz=(0.28, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=mats["gunmetal"],
        name="barrel_jacket",
    )
    gun.visual(
        Cylinder(radius=RWS_BARREL_R, length=RWS_BARREL_LEN - 0.40),
        origin=Origin(
            xyz=(0.48 + (RWS_BARREL_LEN - 0.40) / 2.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)
        ),
        material=mats["gunmetal"],
        name="barrel_tube",
    )
    gun.visual(
        Cylinder(radius=0.035, length=0.10),
        origin=Origin(
            xyz=(0.48 + (RWS_BARREL_LEN - 0.40) + 0.03, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)
        ),
        material=mats["gunmetal"],
        name="flash_hider",
    )
    gun.visual(
        Box((0.22, 0.04, 0.03)),
        origin=Origin(xyz=(0.28, 0.0, 0.045)),
        material=mats["body_dark"],
        name="carrying_handle",
    )
    gun.visual(
        Box((0.02, 0.02, 0.05)),
        origin=Origin(xyz=(0.46, 0.0, 0.055)),
        material=mats["gunmetal"],
        name="front_sight",
    )
    gun.inertial = Inertial.from_geometry(
        Box((1.2, 0.12, 0.12)), mass=22.0, origin=Origin(xyz=(0.4, 0.0, 0.0))
    )

    model.articulation(
        "rws_gun_elevation",
        ArticulationType.REVOLUTE,
        parent=pod,
        child=gun,
        origin=Origin(xyz=RWS_GUN_TRUNNION),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=100.0, velocity=1.0, lower=ELEV_LOWER, upper=RWS_ELEV_UPPER
        ),
    )


# --------------------------------------------------------- weapon: open_pintle
def _build_open_pintle(
    model: ArticulatedObject, hull, r: ResolvedArmoredVehicleConfig, mats: dict[str, object]
) -> None:
    # Fixed ring race on the roof.
    race = cq.Workplane("XY").circle(RING_OUTER_R).circle(RING_INNER_R).extrude(RING_H)
    hull.visual(
        mesh_from_cadquery(race, "av_pintle_race"),
        origin=Origin(xyz=(PINTLE_X, 0.0, PINTLE_Z)),
        material=mats["body_dark"],
        name="pintle_ring_race",
    )

    mount = model.part("pintle_mount")
    rotating_ring = (
        cq.Workplane("XY")
        .circle(RING_OUTER_R - 0.015)
        .circle(RING_INNER_R + 0.015)
        .extrude(RING_TOP_Z)
    )
    mount.visual(
        mesh_from_cadquery(rotating_ring, "av_pintle_ring"),
        material=mats["body_dark"],
        name="rotating_ring",
    )
    # Solid hub boss at the pintle centre so the pintle_traverse joint origin
    # (child-local 0,0,0) lies on real geometry.
    mount.visual(
        Cylinder(radius=0.06, length=POST_TOP_Z + 0.04),
        origin=Origin(xyz=(0.0, 0.0, (POST_TOP_Z + 0.04) / 2.0 - 0.02)),
        material=mats["body_dark"],
        name="pintle_hub_boss",
    )
    spider = cq.Workplane("XY").workplane(offset=SPIDER_Z).circle(SPIDER_R).extrude(0.020)
    mount.visual(
        mesh_from_cadquery(spider, "av_pintle_spider"),
        material=mats["body_dark"],
        name="spider_plate",
    )
    post = cq.Workplane("XY").workplane(offset=SPIDER_Z).circle(0.035).extrude(POST_H + 0.012)
    mount.visual(
        mesh_from_cadquery(post, "av_pintle_post"), material=mats["body_dark"], name="pintle_post"
    )
    yoke_base = (
        cq.Workplane("XY").workplane(offset=POST_TOP_Z).rect(0.12, 0.24).extrude(PLATE_THICK)
    )
    mount.visual(
        mesh_from_cadquery(yoke_base, "av_pintle_yoke_base"),
        material=mats["body_dark"],
        name="yoke_base_plate",
    )
    yoke_geom = TrunnionYokeGeometry(
        (0.20, 0.30, 0.22),
        span_width=0.14,
        trunnion_diameter=0.030,
        trunnion_center_z=0.16,
        base_thickness=0.020,
        center=False,
    )
    mount.visual(
        mesh_from_geometry(yoke_geom, "av_pintle_yoke"),
        origin=Origin(xyz=(0.0, 0.0, POST_TOP_Z + PLATE_THICK)),
        material=mats["gunmetal"],
        name="yoke",
    )
    # MG receiver + barrel chain riding the yoke.
    recv_cx = 0.04
    mount.visual(
        Box((0.34, 0.12, 0.14)),
        origin=Origin(xyz=(recv_cx, 0.0, MG_TRUNNION_Z)),
        material=mats["gunmetal"],
        name="mg_receiver",
    )
    recv_front_x = recv_cx + 0.17
    mount.visual(
        Cylinder(radius=0.038, length=0.55),
        origin=Origin(xyz=(recv_front_x, 0.0, MG_TRUNNION_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=mats["body_dark"],
        name="mg_barrel_jacket",
    )
    jacket_end_x = recv_front_x + 0.55
    mount.visual(
        Cylinder(radius=0.022, length=1.05),
        origin=Origin(xyz=(jacket_end_x, 0.0, MG_TRUNNION_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=mats["gunmetal"],
        name="mg_barrel",
    )
    mount.visual(
        Cylinder(radius=0.035, length=0.10),
        origin=Origin(xyz=(jacket_end_x + 0.50, 0.0, MG_TRUNNION_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=mats["gunmetal"],
        name="mg_flash_hider",
    )
    recv_back_x = recv_cx - 0.17
    mount.visual(
        Box((0.03, 0.10, 0.12)),
        origin=Origin(xyz=(recv_back_x - 0.005, 0.0, MG_TRUNNION_Z)),
        material=mats["gunmetal"],
        name="mg_backplate",
    )
    mount.visual(
        Cylinder(radius=0.014, length=0.14),
        origin=Origin(
            xyz=(recv_back_x - 0.05, 0.05, MG_TRUNNION_Z - 0.03), rpy=(0.0, pi / 2.0, 0.0)
        ),
        material=mats["gunmetal"],
        name="mg_left_spade_grip",
    )
    mount.visual(
        Cylinder(radius=0.014, length=0.14),
        origin=Origin(
            xyz=(recv_back_x - 0.05, -0.05, MG_TRUNNION_Z - 0.03), rpy=(0.0, pi / 2.0, 0.0)
        ),
        material=mats["gunmetal"],
        name="mg_right_spade_grip",
    )
    mount.visual(
        Box((0.16, 0.14, 0.14)),
        origin=Origin(xyz=(-0.02, 0.10, MG_TRUNNION_Z)),
        material=mats["body_dark"],
        name="mg_ammo_box",
    )
    mount.visual(
        Box((0.08, 0.05, 0.06)),
        origin=Origin(xyz=(0.02, 0.065, MG_TRUNNION_Z + 0.06)),
        material=mats["gunmetal"],
        name="mg_feed_chute",
    )
    # Traverse handle ring at the spider.
    handle_half = SPIDER_R + 0.010
    mount.visual(
        Cylinder(radius=0.014, length=2.0 * handle_half),
        origin=Origin(xyz=(0.0, 0.0, SPIDER_Z + 0.010), rpy=(pi / 2.0, 0.0, 0.0)),
        material=mats["gunmetal"],
        name="traverse_handle",
    )
    mount.visual(
        Cylinder(radius=0.020, length=0.06),
        origin=Origin(xyz=(0.0, handle_half, SPIDER_Z + 0.010)),
        material=mats["body_dark"],
        name="traverse_grip_left",
    )
    mount.visual(
        Cylinder(radius=0.020, length=0.06),
        origin=Origin(xyz=(0.0, -handle_half, SPIDER_Z + 0.010)),
        material=mats["body_dark"],
        name="traverse_grip_right",
    )
    mount.inertial = Inertial.from_geometry(
        Box((2.4, 0.7, 0.9)), mass=70.0, origin=Origin(xyz=(0.4, 0.0, 0.4))
    )

    # Yaw only — the MG is rigid within pintle_mount (no separate elevation joint).
    model.articulation(
        "pintle_traverse",
        ArticulationType.CONTINUOUS,
        parent=hull,
        child=mount,
        origin=Origin(xyz=(PINTLE_X, 0.0, PINTLE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=200.0, velocity=1.5),
    )


def _build_weapon_station(
    model: ArticulatedObject, hull, r: ResolvedArmoredVehicleConfig, mats: dict[str, object]
) -> None:
    if r.weapon_station == "autocannon_turret":
        _build_autocannon_turret(model, hull, r, mats)
    elif r.weapon_station == "remote_weapon_station":
        _build_remote_weapon_station(model, hull, r, mats)
    else:
        _build_open_pintle(model, hull, r, mats)


# ----------------------------------------------------------------- build
def build_armored_vehicle(
    config: ArmoredVehicleConfig | None = None,
    *,
    assets: AssetContext | None = None,
) -> ArticulatedObject:
    cfg = config or ArmoredVehicleConfig()
    r = resolve_config(cfg)
    model = ArticulatedObject(name=r.name, assets=assets)
    mats = _register_materials(model, r)

    hull = _build_hull(model, r, mats)
    hull.inertial = Inertial.from_geometry(
        Box((7.6, 2.9, 1.7)), mass=14000.0, origin=Origin(xyz=(0.0, 0.0, 1.35))
    )

    _build_running_gear(model, hull, r, mats)
    _build_weapon_station(model, hull, r, mats)

    model.meta["slot_choices"] = slot_choices_for_config(r)
    return model


def build_seeded_armored_vehicle(
    seed: int, *, assets: AssetContext | None = None
) -> ArticulatedObject:
    return build_armored_vehicle(config_from_seed(seed), assets=assets)


# ----------------------------------------------------------------- tests
def _allow_running_gear_overlaps(
    ctx: TestContext, model: ArticulatedObject, r: ResolvedArmoredVehicleConfig
) -> None:
    if r.running_gear == "wheeled":
        for side in ("left", "right"):
            for i in range(r.axle_pairs):
                for welem in ("rim", "hub_plug"):
                    ctx.allow_overlap(
                        "hull",
                        f"{side}_wheel_{i}",
                        elem_a=f"{side}_axle_stub_{i}",
                        elem_b=welem,
                        reason="The fixed axle stub is captured inside the spinning wheel hub.",
                    )
    else:
        for side in ("left", "right"):
            for i in range(N_BOGEYS):
                for belem in ("tire", "rim", "hub_plug"):
                    ctx.allow_overlap(
                        "hull",
                        f"{side}_bogie_{i}",
                        elem_a=f"{side}_track_band",
                        elem_b=belem,
                        reason="The bogie road wheel rides inside the track band loop.",
                    )
                    ctx.allow_overlap(
                        "hull",
                        f"{side}_bogie_{i}",
                        elem_a=f"{side}_bogie_axle_stub_{i}",
                        elem_b=belem,
                        reason="The fixed bogie axle stub is captured inside the spinning road wheel hub.",
                    )


def _allow_weapon_overlaps(
    ctx: TestContext, model: ArticulatedObject, r: ResolvedArmoredVehicleConfig
) -> None:
    if r.weapon_station == "autocannon_turret":
        mantlet_elems = (
            "trunnion_housing",
            "mantlet_collaring_ring",
            "upper_mantlet_brow",
            "lower_mantlet_lip",
            "turret_shell",
        )
        gun_thru_elems = (
            "breech_block",
            "recoil_sleeve",
            "thermal_sleeve",
            "barrel_tube",
            "left_recoil_rod",
            "right_recoil_rod",
        )
        for m_elem in mantlet_elems:
            for g_elem in gun_thru_elems:
                ctx.allow_overlap(
                    "turret",
                    "autocannon",
                    elem_a=m_elem,
                    elem_b=g_elem,
                    reason=f"The {g_elem} is seated through the {m_elem} at the gun port.",
                )
        for base_elem in (
            "turret_shell",
            "turret_bearing_ring",
            "lower_mantlet_lip",
            "mantlet_collaring_ring",
        ):
            for roof_elem in ("hull_shell", "left_crew_hatch", "right_crew_hatch"):
                ctx.allow_overlap(
                    "hull",
                    "turret",
                    elem_a=roof_elem,
                    elem_b=base_elem,
                    reason=f"The {base_elem} seats onto the {roof_elem} at the turret ring.",
                )
    elif r.weapon_station == "remote_weapon_station":
        for pod_elem in (
            "pod_shell",
            "cradle_trunnion_pin",
            "left_cradle_bearing",
            "right_cradle_bearing",
            "day_camera",
        ):
            for gun_elem in ("gun_receiver", "barrel_jacket"):
                ctx.allow_overlap(
                    "rws_pod",
                    "rws_gun",
                    elem_a=pod_elem,
                    elem_b=gun_elem,
                    reason=f"The {gun_elem} is cradled on the {pod_elem} at the RWS trunnion.",
                )
        for hull_elem in ("rws_pedestal", "rws_bearing_boss"):
            for pod_elem in ("pod_bearing_ring", "pod_shell"):
                ctx.allow_overlap(
                    "hull",
                    "rws_pod",
                    elem_a=hull_elem,
                    elem_b=pod_elem,
                    reason="The RWS pod seats on the roof pedestal bearing boss.",
                )
    else:  # open_pintle — intra-mount captured contacts + race
        for race_elem in (
            "rotating_ring",
            "spider_plate",
            "traverse_handle",
            "traverse_grip_left",
            "traverse_grip_right",
        ):
            ctx.allow_overlap(
                "hull",
                "pintle_mount",
                elem_a="pintle_ring_race",
                elem_b=race_elem,
                reason=f"The {race_elem} sweeps over/inside the fixed pintle ring race.",
            )
        # Central bearing boss seats down through the ring race into the roof.
        ctx.allow_overlap(
            "hull",
            "pintle_mount",
            elem_a="hull_shell",
            elem_b="pintle_hub_boss",
            reason="The pintle bearing boss seats into the roof ring.",
        )
        ctx.allow_overlap(
            "hull",
            "pintle_mount",
            elem_a="pintle_ring_race",
            elem_b="pintle_hub_boss",
            reason="The pintle bearing boss rides in the ring race bore.",
        )
        intra = (
            ("rotating_ring", "spider_plate"),
            ("spider_plate", "pintle_post"),
            ("spider_plate", "traverse_handle"),
            ("pintle_post", "yoke_base_plate"),
            ("yoke_base_plate", "yoke"),
            ("yoke", "mg_receiver"),
            ("mg_receiver", "mg_backplate"),
            ("mg_receiver", "mg_left_spade_grip"),
            ("mg_receiver", "mg_right_spade_grip"),
            ("mg_receiver", "mg_ammo_box"),
            ("mg_receiver", "mg_feed_chute"),
            ("mg_receiver", "mg_barrel_jacket"),
            # central hub boss captures the post/spider/yoke stack
            ("pintle_hub_boss", "spider_plate"),
            ("pintle_hub_boss", "pintle_post"),
            ("pintle_hub_boss", "rotating_ring"),
            ("pintle_hub_boss", "yoke_base_plate"),
            ("pintle_hub_boss", "yoke"),
            ("pintle_hub_boss", "traverse_handle"),
        )
        for a, b in intra:
            ctx.allow_overlap(
                "pintle_mount",
                "pintle_mount",
                elem_a=a,
                elem_b=b,
                reason=f"{a} captures/mounts {b} on the pintle assembly.",
            )


def run_armored_vehicle_tests(
    object_model: ArticulatedObject, config: ArmoredVehicleConfig
) -> TestReport:
    r = resolve_config(config)
    ctx = TestContext(object_model)

    _allow_running_gear_overlaps(ctx, object_model, r)
    _allow_weapon_overlaps(ctx, object_model, r)

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.015)

    # ---- overall scale (APC, not MBT: tall boxy hull ~7.6 m long, ~2.9 m wide) ----
    hull_bb = ctx.part_world_aabb("hull")
    if hull_bb is not None:
        hull_len = hull_bb[1][0] - hull_bb[0][0]
        hull_w = hull_bb[1][1] - hull_bb[0][1]
        hull_h = hull_bb[1][2] - hull_bb[0][2]
        ctx.check("hull length ~7-8.5 m", 6.8 <= hull_len <= 8.6, details=f"len={hull_len:.3f}")
        ctx.check("overall width ~2.9 m", 2.70 <= hull_w <= 3.10, details=f"w={hull_w:.3f}")
        # APC identity: hull is TALL (>1.4 m), not a low MBT glacis.
        ctx.check("APC tall boxy hull (not MBT)", hull_h >= 1.45, details=f"h={hull_h:.3f}")

    # ---- running gear ----
    part_names = {p.name for p in object_model.parts}
    if r.running_gear == "wheeled":
        n = r.axle_pairs
        wheel_names = [f"{side}_wheel_{i}" for side in ("left", "right") for i in range(n)]
        ctx.check(
            "wheel count == 2*axle_pairs", len(wheel_names) == 2 * n, details=f"axle_pairs={n}"
        )
        ctx.check("axle_pairs in {3,4,5}", AXLE_PAIRS_MIN <= n <= AXLE_PAIRS_MAX, details=str(n))
        for wn in wheel_names:
            ctx.check(f"{wn} present", wn in part_names)
            bb = ctx.part_world_aabb(wn)
            ctx.check(
                f"{wn} grounded at z=0", bb is not None and abs(bb[0][2]) < 0.02, details=f"{bb}"
            )
            ctx.check(
                f"{wn} diameter ~1.1 m",
                bb is not None and abs((bb[1][2] - bb[0][2]) - 1.10) < 0.06,
                details=f"{bb}",
            )
            art = object_model.get_articulation(f"{wn}_spin")
            ctx.check(
                f"{wn} continuous lateral axle",
                art.articulation_type == ArticulationType.CONTINUOUS and abs(art.axis[1]) == 1.0,
                details=f"axis={art.axis}",
            )
        # multiplicity proven: front + rear wheels are spaced apart along X
        front = ctx.part_world_position("left_wheel_0")
        rear = ctx.part_world_position(f"left_wheel_{n - 1}")
        if front is not None and rear is not None:
            ctx.check(
                "axle pairs span the hull",
                abs(front[0] - rear[0]) > 3.0,
                details=f"front={front[0]:.2f} rear={rear[0]:.2f}",
            )
        # off-axis valve stem proves spin
        spin = object_model.get_articulation("left_wheel_0_spin")
        vr = ctx.part_element_world_aabb("left_wheel_0", elem="valve_stem")
        with ctx.pose({spin: pi}):
            vh = ctx.part_element_world_aabb("left_wheel_0", elem="valve_stem")
        ctx.check(
            "wheel spin carries the valve stem",
            vr is not None and vh is not None and abs(vr[0][0] - vh[0][0]) > 0.4,
            details=f"rest={vr}, half={vh}",
        )
    else:
        bogie_names = [f"{side}_bogie_{i}" for side in ("left", "right") for i in range(N_BOGEYS)]
        ctx.check("ten bogie road wheels exist", len(bogie_names) == 10)
        for bn in bogie_names:
            ctx.check(f"{bn} present", bn in part_names)
            art = object_model.get_articulation(f"{bn}_spin")
            ctx.check(
                f"{bn} continuous lateral axle",
                art.articulation_type == ArticulationType.CONTINUOUS and abs(art.axis[1]) == 1.0,
                details=f"axis={art.axis}",
            )
        for side in ("left", "right"):
            tb = ctx.part_element_world_aabb("hull", elem=f"{side}_track_band")
            ctx.check(
                f"{side} track band visible",
                tb is not None and (tb[1][0] - tb[0][0]) > 5.5,
                details=f"{tb}",
            )
        # The off-axis axle marker must orbit clear of the suspension arm blade.
        b_spin = object_model.get_articulation("left_bogie_0_spin")
        arm_bb = ctx.part_element_world_aabb("hull", elem="left_suspension_arm_0")
        with ctx.pose({b_spin: -pi / 2.0}):
            mk_bb = ctx.part_element_world_aabb("left_bogie_0", elem="axle_marker")
        ctx.check(
            "axle marker sweeps clear of the suspension arm",
            arm_bb is not None and mk_bb is not None and mk_bb[0][1] > arm_bb[1][1] + 0.005,
            details=f"marker={mk_bb}, arm={arm_bb}",
        )
        # axle_pairs must NOT have created wheel parts on tracked
        ctx.check(
            "no wheeled parts on tracked", not any(n_.endswith("_wheel_0") for n_ in part_names)
        )

    # ---- weapon station: >= 1 non-fixed joint with yaw ----
    if r.weapon_station == "autocannon_turret":
        traverse = object_model.get_articulation("turret_traverse")
        elevation = object_model.get_articulation("gun_elevation")
        ctx.check(
            "turret traverse continuous about Z",
            traverse.articulation_type == ArticulationType.CONTINUOUS
            and tuple(traverse.axis) == (0.0, 0.0, 1.0),
            details=f"{traverse.axis}",
        )
        ctx.check(
            "gun elevation revolute",
            elevation.articulation_type == ArticulationType.REVOLUTE,
            details=str(elevation.articulation_type),
        )
        # low autocannon (APC), not a big MBT main gun: barrel tube radius is slim.
        gun_bb_rest = ctx.part_world_aabb("autocannon")
        with ctx.pose({traverse: pi / 2.0}):
            gun_bb_yaw = ctx.part_world_aabb("autocannon")
        ctx.check(
            "turret yaw swings the gun to +Y",
            gun_bb_rest is not None
            and gun_bb_yaw is not None
            and gun_bb_yaw[1][1] > 2.3
            and gun_bb_yaw[1][0] < 1.7,
            details=f"rest={gun_bb_rest}, yaw={gun_bb_yaw}",
        )
        muzzle_rest = ctx.part_element_world_aabb("autocannon", elem="muzzle_device")
        with ctx.pose({elevation: GUN_ELEV_UPPER}):
            muzzle_up = ctx.part_element_world_aabb("autocannon", elem="muzzle_device")
            breech_up = ctx.part_element_world_aabb("autocannon", elem="breech_block")
        ctx.check(
            "positive elevation raises the muzzle",
            muzzle_rest is not None
            and muzzle_up is not None
            and (muzzle_up[1][2] - muzzle_rest[1][2]) > 1.0,
            details=f"rest={muzzle_rest}, up={muzzle_up}",
        )
        # Derived-limit guarantee: the breech never dips into the hull deck.
        ctx.check(
            "breech clears the hull deck at full elevation",
            breech_up is not None and breech_up[0][2] >= HULL_ROOF + 0.004,
            details=f"breech={breech_up}",
        )
        # APC turret sits low: turret roof only modestly above hull roof.
        tb = ctx.part_element_world_aabb("turret", elem="turret_shell")
        ctx.check(
            "low APC turret (not tall MBT turret)",
            tb is not None and tb[1][2] < 2.85,
            details=f"turret top={tb[1][2] if tb else None}",
        )
    elif r.weapon_station == "remote_weapon_station":
        traverse = object_model.get_articulation("rws_traverse")
        elevation = object_model.get_articulation("rws_gun_elevation")
        ctx.check(
            "rws traverse continuous about Z",
            traverse.articulation_type == ArticulationType.CONTINUOUS
            and tuple(traverse.axis) == (0.0, 0.0, 1.0),
            details=f"{traverse.axis}",
        )
        ctx.check(
            "rws gun elevation revolute",
            elevation.articulation_type == ArticulationType.REVOLUTE,
            details=str(elevation.articulation_type),
        )
        ctx.check("no big turret on RWS", "turret" not in part_names and "rws_pod" in part_names)
        gun_rest = ctx.part_world_position("rws_gun")
        with ctx.pose({traverse: pi / 2.0}):
            gun_yaw = ctx.part_world_position("rws_gun")
        ctx.check(
            "rws yaw swings the gun",
            gun_rest is not None and gun_yaw is not None and abs(gun_yaw[1] - gun_rest[1]) > 0.1,
            details=f"rest={gun_rest}, yaw={gun_yaw}",
        )
        # Derived-limit guarantee: the receiver never dips into the bearing boss.
        boss_top = RWS_BOSS_CZ + RWS_BOSS_LEN / 2.0
        with ctx.pose({elevation: RWS_ELEV_UPPER}):
            recv_up = ctx.part_element_world_aabb("rws_gun", elem="gun_receiver")
        ctx.check(
            "rws receiver clears the bearing boss at full elevation",
            recv_up is not None and recv_up[0][2] >= boss_top + 0.002,
            details=f"recv={recv_up}, boss_top={boss_top:.3f}",
        )
    else:  # open_pintle
        traverse = object_model.get_articulation("pintle_traverse")
        ctx.check(
            "pintle traverse continuous about Z",
            traverse.articulation_type == ArticulationType.CONTINUOUS
            and tuple(traverse.axis) == (0.0, 0.0, 1.0),
            details=f"{traverse.axis}",
        )
        ctx.check(
            "open pintle mount present (no turret)",
            "pintle_mount" in part_names and "turret" not in part_names,
        )
        mg_rest = ctx.part_element_world_aabb("pintle_mount", elem="mg_barrel")
        with ctx.pose({traverse: pi / 2.0}):
            mg_yaw = ctx.part_element_world_aabb("pintle_mount", elem="mg_barrel")
        ctx.check(
            "pintle yaw swings the MG",
            mg_rest is not None and mg_yaw is not None and mg_yaw[1][1] > 0.8,
            details=f"rest={mg_rest}, yaw={mg_yaw}",
        )

    # ---- hull-armor candidate ----
    if r.hull_armor == "applique_panels":
        panels = [
            v.name
            for v in object_model.get_part("hull").visuals
            if v.name and v.name.startswith("panel_")
        ]
        ctx.check("applique panels present", len(panels) >= 18, details=f"n={len(panels)}")
    elif r.hull_armor == "slab_mrap":
        # slab hull has no plan-view nose cuts: bow stays near full width up high.
        ctx.check("slab hull selected", "hull" in part_names)

    # ---- palette diversity: >=3 distinct rgba materials applied ----
    rgbas = {tuple(round(c, 4) for c in m.rgba) for m in object_model.materials}
    ctx.check("palette has >=3 distinct colors", len(rgbas) >= 3, details=f"distinct={len(rgbas)}")

    # ---- key placement ----
    marking_bb = ctx.part_element_world_aabb("hull", elem="tactical_marking_plate")
    ctx.check(
        "tactical marking proud of the right hull side",
        marking_bb is not None and marking_bb[0][1] < -1.42,
        details=f"{marking_bb}",
    )

    # ---- slot_choices recorded ----
    expected_slots = dict(slot_choices_for_config(r))
    recorded = dict(object_model.meta.get("slot_choices", ()))
    ctx.check(
        "slot_choices_recorded",
        recorded == expected_slots,
        details=f"{recorded} vs {expected_slots}",
    )

    return ctx.report()


__all__ = [
    "ArmoredVehicleConfig",
    "ResolvedArmoredVehicleConfig",
    "build_armored_vehicle",
    "build_seeded_armored_vehicle",
    "config_from_seed",
    "resolve_config",
    "run_armored_vehicle_tests",
    "slot_choices_for_config",
    "slot_choices_for_seed",
]
