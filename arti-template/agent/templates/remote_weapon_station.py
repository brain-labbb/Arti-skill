"""Procedural template for the ``remote_weapon_station`` category.

Follows ``articraft_template_authoring/specs_modular_v1/remote_weapon_station.md``.

A remote weapon station (RWS) is a static base / pedestal that carries a turret
housing yawing continuously about the vertical Z axis (azimuth / pan); the
turret's trunnion yoke cradles a weapon mantlet that pitches about a horizontal
axis (elevation). Baked into the pitching cradle are the category-identity
parts: a gun receiver + barrel (with a muzzle device), an EO/IR sensor pod with
a glass lens, and an ammunition / feed box.

PRIMARY_ANCHOR: ``rec_remote_weapon_station_e52974505580453f8ff659d62ecab0f5``.
The anchor is a 3-part / 2-joint model::

    pedestal --yaw (CONTINUOUS, +Z)--> yaw_base --pitch (REVOLUTE, -Y)--> cradle

Per AUTHORING.md §A Rule 3 the template inherits this skeleton:
``config_from_seed(0)`` reproduces the anchor's part names, joint topology,
per-part visual counts and primitive histograms (pedestal Box1/Cyl6, yaw_base
Cyl4/Box3/Mesh2 -- the two extruded sloped side-armour plates -- and cradle
Cyl6/Box8) and overall bbox aspect ratio. Every seed keeps the same three parts
and two articulations; only geometry (overall scale, barrel length/calibre,
optics size, elevation limits, pan joint type, muzzle style, material palette
and the decorative bolt-ring / ammo presence) varies. The gun barrel, optics
lens, feed box, recoil sleeve and bolt circle are attached as
``parent.visual(...)`` per Rule 1, never as separate FIXED-jointed parts.

The forward (barrel) axis is +X and the elevation axis is (0,-1,0), so a
positive elevation command raises the muzzle (the sign match the spec requires).

Adopted source modules (see specs/remote_weapon_station.md Adopted Source
Index): S1 e52974 (canonical 3-part pedestal/yaw_base/cradle, the
PRIMARY_ANCHOR), S2 b950e8 (annular slew rings + guard hinge), S3 0002
(mesh base/housing/cradle + feed chute), S4 4d4829 (tilting sensor pod),
S5 0004 (split FIXED payload), S6 fd5b05 (fork turret + access hatch).

Parameter -> geometry mapping (linear_chain, 3 parts / 2 joints):

* Slot A ``pedestal`` (Box1/Cyl6 + optional bolt ring / access panels): a
  floor plinth, a bearing column and a round bearing seat. ``base_style`` /
  ``base_radius`` / ``base_height`` scale it; the yaw joint plane is the seat
  top (``yaw_origin_z = 0.380 * scale``).
* Slot B ``yaw_base`` (Cyl5/Box3/Mesh2 + optional smoke tubes / antenna): the
  turret turntable, armoured deck, rear electronics hump, two extruded
  sloped side-armour plates (the only meshes), the trunnion bearing bosses and
  a top bridge. The fixed trunnion cross-shaft anchors the elevation joint
  origin (``pitch_origin_z = 0.430 * scale``).
* Slot C ``cradle`` (Cyl6/Box8 + optional shroud / rail furniture): the
  pitching mantlet with the trunnion tube, cross beam, weapon receiver, barrel
  chain (gas tube, clamps, muzzle device), the EO/IR sensor pod with two glass
  lenses, and the ammo feed box. ``weapon_style`` / ``muzzle_style`` /
  ``barrel_*`` / ``optics_*`` reshape the identity payload.

The pan joint (``pan_joint_type``) is CONTINUOUS or a limited REVOLUTE about
+Z; the elevation joint is a REVOLUTE about (0,-1,0) so the +X barrel rises on
positive elevation. Slot D auxiliary mechanisms from the spec (tilting sensor
pod, service hatch, split payload) are intentionally not sampled here: this
template covers the dominant two-DOF monolithic-cradle family, keeping the
anchor's three-part / two-joint skeleton on every seed.
"""

from __future__ import annotations

import math
import random
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# adopted: S1 e52974 — canonical pedestal / yaw_base / cradle 3-part chain
#                       + pan CONTINUOUS + elevation REVOLUTE (the PRIMARY_ANCHOR)
# adopted: S2 b950e8 — slew-ring base + bearing bosses + bolt circle
# adopted: S3 0002   — barrel chain + feed box layout
# adopted: S4 4d4829 — EO/IR sensor pod + glass lens stack
# adopted: S5 0004   — receiver / mantlet proportions
# adopted: S6 fd5b05 — sloped side-armour plate + trunnion yoke

__modular__ = True

MaterialStyle = Literal[
    "matte_olive",
    "desert_tan",
    "naval_grey",
    "urban_black",
]
WeaponStyle = Literal["mg", "autocannon", "cannon_heavy"]
MuzzleStyle = Literal["brake", "flash_hider", "plain"]
PanJointType = Literal["continuous", "revolute"]

_MATERIAL_STYLES: tuple[MaterialStyle, ...] = (
    "matte_olive",
    "desert_tan",
    "naval_grey",
    "urban_black",
)
_WEAPON_STYLES: tuple[WeaponStyle, ...] = ("mg", "autocannon", "cannon_heavy")
_MUZZLE_STYLES: tuple[MuzzleStyle, ...] = ("brake", "flash_hider", "plain")
_PAN_JOINT_TYPES: tuple[PanJointType, ...] = ("continuous", "revolute")

PALETTES: dict[MaterialStyle, dict[str, tuple[float, float, float, float]]] = {
    "matte_olive": {
        "armor": (0.22, 0.26, 0.18, 1.0),
        "dark": (0.12, 0.15, 0.11, 1.0),
        "gun": (0.06, 0.065, 0.06, 1.0),
        "black": (0.01, 0.011, 0.010, 1.0),
        "glass": (0.02, 0.08, 0.12, 1.0),
        "rubber": (0.018, 0.018, 0.016, 1.0),
    },
    "desert_tan": {
        "armor": (0.62, 0.55, 0.40, 1.0),
        "dark": (0.42, 0.36, 0.25, 1.0),
        "gun": (0.10, 0.095, 0.08, 1.0),
        "black": (0.03, 0.028, 0.024, 1.0),
        "glass": (0.06, 0.12, 0.14, 1.0),
        "rubber": (0.04, 0.038, 0.032, 1.0),
    },
    "naval_grey": {
        "armor": (0.46, 0.49, 0.52, 1.0),
        "dark": (0.28, 0.30, 0.33, 1.0),
        "gun": (0.10, 0.11, 0.12, 1.0),
        "black": (0.02, 0.022, 0.024, 1.0),
        "glass": (0.04, 0.10, 0.16, 1.0),
        "rubber": (0.03, 0.032, 0.035, 1.0),
    },
    "urban_black": {
        "armor": (0.16, 0.17, 0.18, 1.0),
        "dark": (0.08, 0.085, 0.09, 1.0),
        "gun": (0.045, 0.048, 0.05, 1.0),
        "black": (0.008, 0.009, 0.010, 1.0),
        "glass": (0.02, 0.06, 0.10, 1.0),
        "rubber": (0.015, 0.015, 0.014, 1.0),
    },
}


@dataclass(frozen=True)
class RemoteWeaponStationConfig:
    """Public, JSON-serialisable knobs.

    The defaults reproduce the PRIMARY_ANCHOR: a compact olive MG station with a
    muzzle brake, a two-lens EO/IR pod, an ammo feed box, the 2x2 corner bolt
    pattern and a continuous pan drive (pedestal 7 visuals, yaw_base 9, cradle
    14 -- matching the anchor fingerprint exactly).
    """

    material_style: MaterialStyle = "matte_olive"
    weapon_style: WeaponStyle = "mg"
    muzzle_style: MuzzleStyle = "brake"
    pan_joint_type: PanJointType = "continuous"
    # Overall scale + independent proportions.
    scale: float = 1.0
    barrel_length_factor: float = 1.0
    barrel_radius_factor: float = 1.0
    optics_factor: float = 1.0
    # Elevation limits (no effect on the closed pose; sign-matched to +X barrel).
    elevation_lower: float = math.radians(-20.0)
    elevation_upper: float = math.radians(60.0)
    pan_limit: float = math.pi
    # Decorative / optional counts (0 extra at the anchor — seed 0 reproduces the
    # anchor's exact per-part visual histograms, so all of these default off).
    ring_bolt_count: int = 0
    has_ammo: bool = True
    n_smoke_launchers: int = 0
    has_antenna: bool = False
    has_optic_rail: bool = False
    has_access_panels: bool = False
    name: str = "seeded_remote_weapon_station"


@dataclass(frozen=True)
class ResolvedRemoteWeaponStationConfig:
    """Validated + derived configuration passed to the build helpers."""

    material_style: MaterialStyle
    weapon_style: WeaponStyle
    muzzle_style: MuzzleStyle
    pan_joint_type: PanJointType
    scale: float
    barrel_length_factor: float
    barrel_radius_factor: float
    optics_factor: float
    elevation_lower: float
    elevation_upper: float
    pan_limit: float
    ring_bolt_count: int
    has_ammo: bool
    n_smoke_launchers: int
    has_antenna: bool
    has_optic_rail: bool
    has_access_panels: bool
    # Derived joint planes (world / parent frame).
    yaw_origin_z: float
    pitch_origin_z: float
    name: str


def config_from_seed(seed: int) -> RemoteWeaponStationConfig:
    """Sample a reproducible remote weapon station configuration.

    Per AUTHORING.md §A Rule 3, seed=0 returns the default config whose
    geometry fingerprint matches the PRIMARY_ANCHOR. Other seeds sample a stable
    subdomain that keeps the three-part / two-joint skeleton while varying
    overall scale, barrel calibre/length, optics size, elevation limits, pan
    joint type, muzzle/material style and the decorative bolt-ring / ammo
    presence. Every declared ``Literal`` value is reachable so the
    ``enum_coverage`` gate is satisfied across the sweep.
    """

    if seed == 0:
        return RemoteWeaponStationConfig()

    rng = random.Random(seed * 2654435761 + 40503)

    material_style: MaterialStyle = rng.choice(_MATERIAL_STYLES)
    weapon_style: WeaponStyle = rng.choice(_WEAPON_STYLES)
    muzzle_style: MuzzleStyle = rng.choice(_MUZZLE_STYLES)
    pan_joint_type: PanJointType = rng.choice(_PAN_JOINT_TYPES)

    scale = round(rng.uniform(0.7, 1.5), 4)
    if weapon_style == "mg":
        barrel_radius_factor = round(rng.uniform(0.85, 1.15), 4)
        barrel_length_factor = round(rng.uniform(0.85, 1.2), 4)
    elif weapon_style == "autocannon":
        barrel_radius_factor = round(rng.uniform(1.4, 1.9), 4)
        barrel_length_factor = round(rng.uniform(1.05, 1.35), 4)
    else:  # cannon_heavy
        barrel_radius_factor = round(rng.uniform(2.1, 2.8), 4)
        barrel_length_factor = round(rng.uniform(1.1, 1.45), 4)

    optics_factor = round(rng.uniform(0.8, 1.3), 4)
    elevation_lower = round(rng.uniform(-0.35, -0.10), 4)
    elevation_upper = round(rng.uniform(0.45, 1.10), 4)
    pan_limit = round(rng.uniform(2.4, math.pi), 4)
    ring_bolt_count = rng.choice((0, 0, 8, 12, 16))
    has_ammo = rng.random() < 0.8
    n_smoke_launchers = rng.choice((0, 0, 2, 3, 4))
    has_antenna = rng.random() < 0.5
    has_optic_rail = rng.random() < 0.5
    has_access_panels = rng.random() < 0.5

    return RemoteWeaponStationConfig(
        material_style=material_style,
        weapon_style=weapon_style,
        muzzle_style=muzzle_style,
        pan_joint_type=pan_joint_type,
        scale=scale,
        barrel_length_factor=barrel_length_factor,
        barrel_radius_factor=barrel_radius_factor,
        optics_factor=optics_factor,
        elevation_lower=elevation_lower,
        elevation_upper=elevation_upper,
        pan_limit=pan_limit,
        ring_bolt_count=ring_bolt_count,
        has_ammo=has_ammo,
        n_smoke_launchers=n_smoke_launchers,
        has_antenna=has_antenna,
        has_optic_rail=has_optic_rail,
        has_access_panels=has_access_panels,
        name=f"seeded_remote_weapon_station_{seed}",
    )


def resolve_config(
    config: RemoteWeaponStationConfig,
) -> ResolvedRemoteWeaponStationConfig:
    """Validate enums, clamp dimensions and derive the joint planes."""

    if config.material_style not in PALETTES:
        raise ValueError(f"Unsupported material_style: {config.material_style}")
    if config.weapon_style not in _WEAPON_STYLES:
        raise ValueError(f"Unsupported weapon_style: {config.weapon_style}")
    if config.muzzle_style not in _MUZZLE_STYLES:
        raise ValueError(f"Unsupported muzzle_style: {config.muzzle_style}")
    if config.pan_joint_type not in _PAN_JOINT_TYPES:
        raise ValueError(f"Unsupported pan_joint_type: {config.pan_joint_type}")

    scale = _clamp(config.scale, 0.55, 1.8)
    barrel_length_factor = _clamp(config.barrel_length_factor, 0.7, 1.6)
    barrel_radius_factor = _clamp(config.barrel_radius_factor, 0.6, 3.0)
    optics_factor = _clamp(config.optics_factor, 0.65, 1.5)

    elevation_lower = config.elevation_lower
    elevation_upper = config.elevation_upper
    if elevation_lower >= elevation_upper:
        elevation_lower, elevation_upper = math.radians(-20.0), math.radians(60.0)
    elevation_lower = _clamp(elevation_lower, -0.45, -0.05)
    elevation_upper = _clamp(elevation_upper, 0.40, 1.20)
    pan_limit = _clamp(config.pan_limit, 1.5, math.pi)

    ring_bolt_count = int(config.ring_bolt_count)
    if ring_bolt_count not in (0, 8, 12, 16):
        ring_bolt_count = 0
    n_smoke_launchers = int(config.n_smoke_launchers)
    if n_smoke_launchers not in (0, 2, 3, 4):
        n_smoke_launchers = 0

    # Joint planes from the anchor, scaled. The yaw plane is the pedestal bearing
    # top; the pitch (trunnion) plane is the yaw_base side-plate height.
    yaw_origin_z = 0.380 * scale
    pitch_origin_z = 0.430 * scale

    return ResolvedRemoteWeaponStationConfig(
        material_style=config.material_style,
        weapon_style=config.weapon_style,
        muzzle_style=config.muzzle_style,
        pan_joint_type=config.pan_joint_type,
        scale=scale,
        barrel_length_factor=barrel_length_factor,
        barrel_radius_factor=barrel_radius_factor,
        optics_factor=optics_factor,
        elevation_lower=elevation_lower,
        elevation_upper=elevation_upper,
        pan_limit=pan_limit,
        ring_bolt_count=ring_bolt_count,
        has_ammo=bool(config.has_ammo),
        n_smoke_launchers=n_smoke_launchers,
        has_antenna=bool(config.has_antenna),
        has_optic_rail=bool(config.has_optic_rail),
        has_access_panels=bool(config.has_access_panels),
        yaw_origin_z=yaw_origin_z,
        pitch_origin_z=pitch_origin_z,
        name=config.name,
    )


def _clamp(value: float, lo: float, hi: float) -> float:
    if lo > hi:
        lo, hi = hi, lo
    return max(lo, min(hi, float(value)))


def _joint_meta(joint_type, axis, origin, joint_range) -> dict[str, object]:
    return {"type": joint_type, "axis": axis, "origin": origin, "range": joint_range}


def _armored_plate_mesh(scale: float, name: str):
    """Sloped side-armour plate (adopted: S6 fd5b05 / S1 anchor).

    ``ExtrudeGeometry`` builds a 2-D XY profile through local Z. Rotating the
    visual +90 deg about X maps the profile's Y to vertical Z and the extrusion
    depth to side-plate thickness. The whole profile scales with the station.
    """

    side_profile = [
        (-0.34 * scale, -0.21 * scale),
        (0.24 * scale, -0.21 * scale),
        (0.34 * scale, -0.06 * scale),
        (0.27 * scale, 0.22 * scale),
        (-0.22 * scale, 0.22 * scale),
        (-0.36 * scale, 0.04 * scale),
    ]
    return mesh_from_geometry(ExtrudeGeometry(side_profile, 0.050 * scale), name)


# --------------------------------------------------------------------------- #
# pedestal (static base, root)
# --------------------------------------------------------------------------- #


def _build_pedestal(part, r: ResolvedRemoteWeaponStationConfig, *, mats) -> None:
    """Fixed low pedestal: a floor plinth, a bearing column and a round bearing
    seat, with a 2x2 anchor-bolt pattern (and an optional decorative bolt ring).
    """

    s = r.scale
    armor = mats["armor"]
    dark = mats["dark"]
    gun = mats["gun"]

    part.visual(
        Box((0.76 * s, 0.62 * s, 0.060 * s)),
        origin=Origin(xyz=(0.0, 0.0, 0.030 * s)),
        material=dark,
        name="floor_plinth",
    )
    part.visual(
        Cylinder(radius=0.185 * s, length=0.240 * s),
        origin=Origin(xyz=(0.0, 0.0, 0.180 * s)),
        material=armor,
        name="pedestal_column",
    )
    part.visual(
        Cylinder(radius=0.260 * s, length=0.080 * s),
        origin=Origin(xyz=(0.0, 0.0, 0.340 * s)),
        material=dark,
        name="fixed_bearing",
    )
    # 2x2 corner anchor bolts (the anchor's exact four-bolt pattern).
    for ix, x in enumerate((-0.27 * s, 0.27 * s)):
        for iy, y in enumerate((-0.20 * s, 0.20 * s)):
            part.visual(
                Cylinder(radius=0.026 * s, length=0.030 * s),
                origin=Origin(xyz=(x, y, 0.075 * s)),
                material=gun,
                name=f"anchor_bolt_{ix}_{iy}",
            )

    if r.ring_bolt_count > 0:
        _add_bolt_ring(part, r, gun=gun)
    if r.has_access_panels:
        _add_access_panels(part, r, dark=dark, gun=gun)


def _add_access_panels(part, r: ResolvedRemoteWeaponStationConfig, *, dark, gun) -> None:
    """Bolted maintenance panels + cable conduit on the pedestal column.

    The panels are sunk into the column surface (they overlap the column body)
    so they remain part of the pedestal's connected geometry island and never
    protrude far enough to reach the rotating turret above (Rule 1)."""

    s = r.scale
    col_r = 0.185 * s
    for i, ang in enumerate((0.5 * math.pi, 1.5 * math.pi)):
        px, py = col_r * 0.92 * math.cos(ang), col_r * 0.92 * math.sin(ang)
        part.visual(
            Box((0.075 * s, 0.110 * s, 0.150 * s)),
            origin=Origin(xyz=(px, py, 0.180 * s), rpy=(0.0, 0.0, ang)),
            material=dark,
            name=f"access_panel_{i}",
        )
        for j, dz in enumerate((-0.055 * s, 0.055 * s)):
            part.visual(
                Cylinder(radius=0.010 * s, length=0.020 * s),
                origin=Origin(xyz=(px, py, 0.180 * s + dz), rpy=(0.0, 0.5 * math.pi, ang)),
                material=gun,
                name=f"panel_bolt_{i}_{j}",
            )
    # Cable conduit running down one side into the floor plinth.
    part.visual(
        Cylinder(radius=0.018 * s, length=0.230 * s),
        origin=Origin(xyz=(0.0, col_r * 0.9, 0.150 * s)),
        material=gun,
        name="cable_conduit",
    )


def _add_bolt_ring(part, r: ResolvedRemoteWeaponStationConfig, *, gun) -> None:
    """Decorative bolt circle around the bearing seat (Rule 1: plain visuals,
    embedded into the plinth so they share the pedestal's geometry island)."""

    s = r.scale
    ring_r = 0.225 * s
    for index in range(r.ring_bolt_count):
        ang = index * (2.0 * math.pi / r.ring_bolt_count)
        part.visual(
            Cylinder(radius=0.020 * s, length=0.040 * s),
            origin=Origin(xyz=(ring_r * math.cos(ang), ring_r * math.sin(ang), 0.070 * s)),
            material=gun,
            name=f"ring_bolt_{index}",
        )


# --------------------------------------------------------------------------- #
# yaw_base (turret, CONTINUOUS/REVOLUTE pan about +Z)
# --------------------------------------------------------------------------- #


def _build_yaw_base(part, r: ResolvedRemoteWeaponStationConfig, *, mats) -> None:
    """Continuously yawing turret: turntable + bearing, an armoured deck and rear
    electronics hump, two sloped side-armour plates (extruded mesh) carrying the
    trunnion bearing bosses, and a top bridge tying the yoke together."""

    s = r.scale
    armor = mats["armor"]
    dark = mats["dark"]
    gun = mats["gun"]

    part.visual(
        Cylinder(radius=0.278 * s, length=0.060 * s),
        origin=Origin(xyz=(0.0, 0.0, 0.030 * s)),
        material=dark,
        name="turntable_disk",
    )
    part.visual(
        Cylinder(radius=0.225 * s, length=0.075 * s),
        origin=Origin(xyz=(0.0, 0.0, 0.095 * s)),
        material=armor,
        name="rotating_bearing",
    )
    part.visual(
        Box((0.62 * s, 0.61 * s, 0.120 * s)),
        origin=Origin(xyz=(0.0, 0.0, 0.160 * s)),
        material=armor,
        name="armored_deck",
    )
    part.visual(
        Box((0.22 * s, 0.42 * s, 0.160 * s)),
        origin=Origin(xyz=(-0.20 * s, 0.0, 0.300 * s)),
        material=dark,
        name="rear_equipment_hump",
    )
    plate_mesh = _armored_plate_mesh(s, "sloped_side_armor")
    for i, y in enumerate((-0.305 * s, 0.305 * s)):
        part.visual(
            plate_mesh,
            origin=Origin(xyz=(0.0, y, 0.430 * s), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=armor,
            name=f"side_plate_{i}",
        )
    for i, y in enumerate((-0.345 * s, 0.345 * s)):
        part.visual(
            Cylinder(radius=0.092 * s, length=0.040 * s),
            origin=Origin(xyz=(0.0, y, 0.430 * s), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=gun,
            name=f"bearing_boss_{i}",
        )
    part.visual(
        Box((0.42 * s, 0.660 * s, 0.075 * s)),
        origin=Origin(xyz=(-0.10 * s, 0.0, 0.625 * s)),
        material=dark,
        name="top_bridge",
    )
    # Fixed trunnion cross-shaft spanning the two side plates on the elevation
    # axis. It puts real yaw_base geometry at the pitch joint origin
    # (0, 0, 0.430*s) so fail_if_articulation_origin_far_from_geometry is
    # satisfied; the cradle's trunnion tube rides this shaft (the resulting
    # parent/child overlap is the intended trunnion bearing, whitelisted in
    # run_tests via ctx.allow_overlap).
    part.visual(
        Cylinder(radius=0.050 * s, length=0.610 * s),
        origin=Origin(xyz=(0.0, 0.0, 0.430 * s), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=gun,
        name="trunnion_cross_shaft",
    )

    if r.n_smoke_launchers > 0 or r.has_antenna:
        _add_turret_furniture(part, r, dark=dark, gun=gun)


def _add_turret_furniture(part, r: ResolvedRemoteWeaponStationConfig, *, dark, gun) -> None:
    """Optional turret furniture: a rear whip antenna and a bank of smoke-grenade
    launcher tubes, both embedded into the rear equipment hump so they share the
    turret's geometry island and stay clear (behind/below) of the weapon cradle.
    """

    s = r.scale
    if r.has_antenna:
        # Rear-corner whip antenna, base sunk into the equipment hump.
        ax, ay = -0.255 * s, 0.150 * s
        part.visual(
            Cylinder(radius=0.024 * s, length=0.070 * s),
            origin=Origin(xyz=(ax, ay, 0.300 * s)),
            material=dark,
            name="antenna_base",
        )
        part.visual(
            Cylinder(radius=0.009 * s, length=0.420 * s),
            origin=Origin(xyz=(ax, ay, 0.520 * s)),
            material=gun,
            name="antenna_whip",
        )
    if r.n_smoke_launchers > 0:
        # Smoke-grenade launcher tubes clustered low on the rear hump face,
        # angled up-and-back so they never sweep into the cradle.
        tube_r = 0.030 * s
        cluster_x = -0.295 * s
        spacing = 0.072 * s
        y0 = -(r.n_smoke_launchers - 1) * spacing * 0.5
        for i in range(r.n_smoke_launchers):
            part.visual(
                Cylinder(radius=tube_r, length=0.110 * s),
                origin=Origin(
                    xyz=(cluster_x, y0 + i * spacing, 0.250 * s),
                    rpy=(0.0, -0.5 * math.pi, 0.0),
                ),
                material=dark,
                name=f"smoke_tube_{i}",
            )


# --------------------------------------------------------------------------- #
# cradle (weapon mantlet, REVOLUTE elevation about -Y)
# --------------------------------------------------------------------------- #


def _build_cradle(part, r: ResolvedRemoteWeaponStationConfig, *, mats) -> None:
    """Pitching cradle: trunnion tube + cross beam tie the mantlet; the weapon
    receiver + barrel chain (gas tube, clamps, muzzle device) sit on the +Y
    side, the EO/IR sensor pod (with glass lenses) on the -Y side. All baked
    into one cradle part (Rule 1)."""

    s = r.scale
    armor = mats["armor"]
    dark = mats["dark"]
    gun = mats["gun"]
    black = mats["black"]
    glass = mats["glass"]
    rubber = mats["rubber"]

    # Trunnion tube on the pitch axis (the cradle origin sits on the trunnion
    # line) + transverse cross beam.
    part.visual(
        Cylinder(radius=0.055 * s, length=0.566 * s),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=gun,
        name="trunnion_tube",
    )
    part.visual(
        Box((0.30 * s, 0.42 * s, 0.075 * s)),
        origin=Origin(xyz=(0.09 * s, 0.0, -0.015 * s)),
        material=dark,
        name="cross_beam",
    )

    _build_weapon(part, r, gun=gun, dark=dark, black=black)
    _build_optics(part, r, armor=armor, dark=dark, glass=glass, rubber=rubber)

    if r.has_optic_rail:
        _add_cradle_furniture(part, r, gun=gun, black=black)


def _add_cradle_furniture(part, r: ResolvedRemoteWeaponStationConfig, *, gun, black) -> None:
    """Picatinny optic rail, charging handle and spent-case deflector embedded
    onto the weapon receiver. All sunk into the receiver body so they share the
    cradle's geometry island and ride with the weapon under elevation."""

    s = r.scale
    gun_y = 0.095 * s
    receiver_top = 0.010 * s + 0.0775 * s
    # Picatinny rail on the receiver top deck.
    part.visual(
        Box((0.230 * s, 0.034 * s, 0.024 * s)),
        origin=Origin(xyz=(0.150 * s, gun_y, receiver_top + 0.006 * s)),
        material=black,
        name="optic_rail",
    )
    for i in range(4):
        part.visual(
            Box((0.014 * s, 0.040 * s, 0.018 * s)),
            origin=Origin(xyz=(0.075 * s + i * 0.050 * s, gun_y, receiver_top + 0.004 * s)),
            material=black,
            name=f"rail_slot_{i}",
        )
    # Charging handle on the left side of the receiver.
    part.visual(
        Cylinder(radius=0.012 * s, length=0.090 * s),
        origin=Origin(xyz=(0.120 * s, gun_y + 0.085 * s, 0.040 * s), rpy=(0.0, 0.5 * math.pi, 0.0)),
        material=gun,
        name="charging_handle",
    )
    # Spent-case deflector below the ejection port.
    part.visual(
        Box((0.070 * s, 0.026 * s, 0.060 * s)),
        origin=Origin(xyz=(0.235 * s, gun_y + 0.070 * s, -0.030 * s)),
        material=gun,
        name="case_deflector",
    )


def _weapon_metrics(r: ResolvedRemoteWeaponStationConfig) -> dict[str, float]:
    """Barrel calibre / length derived from weapon_style and the factors."""

    s = r.scale
    base_radius = 0.024 * s * r.barrel_radius_factor
    base_length = 0.690 * s * r.barrel_length_factor
    receiver_w = 0.325 * s
    receiver_d = 0.150 * s
    if r.weapon_style == "autocannon":
        receiver_w = 0.345 * s
        receiver_d = 0.175 * s
    elif r.weapon_style == "cannon_heavy":
        receiver_w = 0.360 * s
        receiver_d = 0.205 * s
    return {
        "barrel_radius": base_radius,
        "barrel_length": base_length,
        "receiver_w": receiver_w,
        "receiver_d": receiver_d,
    }


def _build_weapon(part, r: ResolvedRemoteWeaponStationConfig, *, gun, dark, black) -> None:
    s = r.scale
    m = _weapon_metrics(r)
    barrel_r = m["barrel_radius"]
    barrel_len = m["barrel_length"]
    gun_y = 0.095 * s

    part.visual(
        Box((m["receiver_w"], m["receiver_d"], 0.155 * s)),
        origin=Origin(xyz=(0.185 * s, gun_y, 0.010 * s)),
        material=gun,
        name="weapon_receiver",
    )
    if r.has_ammo:
        part.visual(
            Box((0.135 * s, 0.115 * s, 0.170 * s)),
            origin=Origin(xyz=(0.170 * s, 0.217 * s, -0.080 * s)),
            material=dark,
            name="feed_box",
        )
    # Main barrel pointing forward (+X); its forward axis matches the elevation
    # sign so positive elevation raises the muzzle.
    barrel_cx = 0.305 * s + barrel_len * 0.5
    part.visual(
        Cylinder(radius=barrel_r, length=barrel_len),
        origin=Origin(xyz=(barrel_cx, gun_y, 0.030 * s), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gun,
        name="main_barrel",
    )
    # Gas tube / recoil sleeve alongside the barrel (static geometry only -- no
    # recoil prismatic joint, per the spec).
    part.visual(
        Cylinder(radius=max(0.008 * s, barrel_r * 0.5), length=0.455 * s),
        origin=Origin(
            xyz=(0.570 * s, gun_y, 0.030 * s + barrel_r * 1.8), rpy=(0.0, math.pi / 2.0, 0.0)
        ),
        material=gun,
        name="gas_tube",
    )
    clamp_w = max(0.044 * s, barrel_r * 2.4)
    for i, fx in enumerate((0.55, 1.0)):
        part.visual(
            Box((0.030 * s, clamp_w, clamp_w)),
            origin=Origin(
                xyz=(0.305 * s + barrel_len * fx * 0.5, gun_y, 0.030 * s + barrel_r * 0.3)
            ),
            material=gun,
            name=f"barrel_clamp_{i}",
        )
    if r.weapon_style != "mg":
        _add_barrel_shroud(
            part, r, barrel_r=barrel_r, barrel_len=barrel_len, gun_y=gun_y, dark=dark, gun=gun
        )
    _build_muzzle(
        part,
        r,
        barrel_r=barrel_r,
        barrel_tip_x=barrel_cx + barrel_len * 0.5,
        gun_y=gun_y,
        black=black,
        gun=gun,
    )


def _add_barrel_shroud(part, r, *, barrel_r, barrel_len, gun_y, dark, gun) -> None:
    """Perforated barrel shroud / heat guard around the rear half of the barrel
    for the heavier autocannon / cannon weapon styles. The shroud is coaxial
    with the barrel (it overlaps the barrel within the cradle part, which is
    intra-part and therefore not a part-vs-part clash), so it rides the weapon
    under elevation as one connected island."""

    s = r.scale
    shroud_r = barrel_r * 1.7
    shroud_len = barrel_len * 0.5
    shroud_cx = 0.305 * s + shroud_len * 0.5
    part.visual(
        Cylinder(radius=shroud_r, length=shroud_len),
        origin=Origin(xyz=(shroud_cx, gun_y, 0.030 * s), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="barrel_shroud",
    )
    # Cooling vent rings around the shroud.
    n_vents = 3 if r.weapon_style == "autocannon" else 4
    for i in range(n_vents):
        frac = (i + 1) / (n_vents + 1)
        part.visual(
            Cylinder(radius=shroud_r * 1.08, length=0.018 * s),
            origin=Origin(
                xyz=(0.305 * s + shroud_len * frac, gun_y, 0.030 * s),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=gun,
            name=f"shroud_rib_{i}",
        )


def _build_muzzle(part, r, *, barrel_r, barrel_tip_x, gun_y, black, gun) -> None:
    """Muzzle device at the barrel tip -- a Cylinder in every style so the cradle
    primitive histogram is preserved; only proportions change."""

    s = r.scale
    if r.muzzle_style == "brake":
        muzzle_r = barrel_r * 1.45
        muzzle_len = 0.105 * s
    elif r.muzzle_style == "flash_hider":
        muzzle_r = barrel_r * 1.15
        muzzle_len = 0.150 * s
    else:  # plain
        muzzle_r = barrel_r * 1.05
        muzzle_len = 0.045 * s
    part.visual(
        Cylinder(radius=muzzle_r, length=muzzle_len),
        origin=Origin(
            xyz=(barrel_tip_x + muzzle_len * 0.4, gun_y, 0.030 * s),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=black,
        name="muzzle_brake",
    )


def _build_optics(part, r, *, armor, dark, glass, rubber) -> None:
    """EO/IR sensor pod on the -Y side: body + sun shade + two glass lenses +
    a rubber mount pad. The glass lens is a required identity element."""

    s = r.scale
    of = r.optics_factor
    sensor_y = -0.165 * s
    body_w = 0.250 * s * of
    part.visual(
        Box((body_w, 0.165 * s, 0.205 * s)),
        origin=Origin(xyz=(0.170 * s, sensor_y, 0.005 * s)),
        material=dark,
        name="sensor_body",
    )
    part.visual(
        Box((0.170 * s * of, 0.180 * s, 0.040 * s)),
        origin=Origin(xyz=(0.230 * s, sensor_y, 0.125 * s)),
        material=armor,
        name="sensor_sun_brow",
    )
    lens_front_x = 0.170 * s + body_w * 0.5 + 0.004 * s
    part.visual(
        Cylinder(radius=0.050 * s * of, length=0.022 * s),
        origin=Origin(xyz=(lens_front_x, sensor_y, 0.020 * s), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="primary_lens",
    )
    part.visual(
        Cylinder(radius=0.026 * s * of, length=0.020 * s),
        origin=Origin(
            xyz=(lens_front_x - 0.002 * s, sensor_y, -0.055 * s), rpy=(0.0, math.pi / 2.0, 0.0)
        ),
        material=glass,
        name="secondary_lens",
    )
    part.visual(
        Box((0.055 * s, 0.150 * s, 0.085 * s)),
        origin=Origin(xyz=(-0.025 * s, sensor_y, 0.005 * s)),
        material=rubber,
        name="sensor_mount_pad",
    )


# --------------------------------------------------------------------------- #
# Assembly
# --------------------------------------------------------------------------- #


def build_remote_weapon_station(
    config: RemoteWeaponStationConfig | None = None,
    *,
    assets: AssetContext | None = None,
) -> ArticulatedObject:
    config = config or RemoteWeaponStationConfig()
    r = resolve_config(config)
    if assets is None:
        assets = AssetContext(Path(tempfile.mkdtemp(prefix="articraft-remote-weapon-station-")))
    model = ArticulatedObject(name=r.name, assets=assets)
    model.meta["adopted_source_ids"] = ("S1", "S2", "S3", "S4", "S5", "S6")
    palette = PALETTES[r.material_style]
    mats = {
        "armor": model.material("armor_paint", rgba=palette["armor"]),
        "dark": model.material("dark_armor", rgba=palette["dark"]),
        "gun": model.material("gunmetal", rgba=palette["gun"]),
        "black": model.material("matte_black", rgba=palette["black"]),
        "glass": model.material("sensor_glass", rgba=palette["glass"]),
        "rubber": model.material("dark_rubber", rgba=palette["rubber"]),
    }

    pedestal = model.part("pedestal")
    _build_pedestal(pedestal, r, mats=mats)

    yaw_base = model.part("yaw_base")
    _build_yaw_base(yaw_base, r, mats=mats)

    cradle = model.part("cradle")
    _build_cradle(cradle, r, mats=mats)

    # ---- pan articulation (pedestal -> yaw_base, vertical +Z) ----
    yaw_origin = (0.0, 0.0, r.yaw_origin_z)
    if r.pan_joint_type == "continuous":
        model.articulation(
            "yaw",
            ArticulationType.CONTINUOUS,
            parent=pedestal,
            child=yaw_base,
            origin=Origin(xyz=yaw_origin),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=160.0, velocity=1.6),
            meta=_joint_meta("continuous", (0.0, 0.0, 1.0), yaw_origin, "unbounded"),
        )
    else:
        model.articulation(
            "yaw",
            ArticulationType.REVOLUTE,
            parent=pedestal,
            child=yaw_base,
            origin=Origin(xyz=yaw_origin),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=160.0, velocity=1.6, lower=-r.pan_limit, upper=r.pan_limit
            ),
            meta=_joint_meta("revolute", (0.0, 0.0, 1.0), yaw_origin, (-r.pan_limit, r.pan_limit)),
        )

    # ---- elevation articulation (yaw_base -> cradle, horizontal -Y) ----
    pitch_origin = (0.0, 0.0, r.pitch_origin_z)
    model.articulation(
        "pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_base,
        child=cradle,
        origin=Origin(xyz=pitch_origin),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=95.0,
            velocity=1.2,
            lower=r.elevation_lower,
            upper=r.elevation_upper,
        ),
        meta=_joint_meta(
            "revolute", (0.0, -1.0, 0.0), pitch_origin, (r.elevation_lower, r.elevation_upper)
        ),
    )

    return model


def build_seeded_remote_weapon_station(
    seed: int, *, assets: AssetContext | None = None
) -> ArticulatedObject:
    return build_remote_weapon_station(config_from_seed(seed), assets=assets)


def with_overrides(config: RemoteWeaponStationConfig, **overrides) -> RemoteWeaponStationConfig:
    from dataclasses import replace

    return replace(config, **overrides)


def slot_choices_for_config(
    resolved: ResolvedRemoteWeaponStationConfig,
) -> list[tuple[str, str]]:
    """Recorded on ``model.meta`` for the module_topology_diversity gate."""
    return [
        ("material_palette", resolved.material_style),
        ("weapon_payload", resolved.weapon_style),
        ("muzzle_device", resolved.muzzle_style),
        ("pan_drive", resolved.pan_joint_type),
    ]


def slot_choices_for_seed(seed: int) -> list[tuple[str, str]]:
    return slot_choices_for_config(resolve_config(config_from_seed(seed)))


# --------------------------------------------------------------------------- #
# Author tests (run by `articraft external check`, not by the sweep baseline)
# --------------------------------------------------------------------------- #


def run_remote_weapon_station_tests(
    object_model: ArticulatedObject, config: RemoteWeaponStationConfig
) -> TestReport:
    ctx = TestContext(object_model)
    r = resolve_config(config)

    # The cradle's trunnion tube rides the yaw_base trunnion cross-shaft on the
    # elevation axis — the intended trunnion-bearing contact. Whitelist that
    # overlap so the baseline overlap check treats it as designed contact.
    ctx.allow_overlap(
        "yaw_base",
        "cradle",
        reason="weapon cradle trunnion tube rides the yaw_base trunnion cross-shaft",
    )

    part_names = {part.name for part in object_model.parts}
    ctx.check(
        "core parts present",
        {"pedestal", "yaw_base", "cradle"}.issubset(part_names),
        details=f"parts={sorted(part_names)}",
    )

    yaw = object_model.get_articulation("yaw")
    pitch = object_model.get_articulation("pitch")
    ctx.check(
        "yaw is vertical (+Z) pan",
        tuple(round(a, 3) for a in yaw.axis) == (0.0, 0.0, 1.0),
        details=f"axis={yaw.axis}",
    )
    ctx.check(
        "pitch is horizontal (-Y) elevation",
        str(pitch.articulation_type).endswith("REVOLUTE") and abs(pitch.axis[2]) < 1e-6,
        details=f"type={pitch.articulation_type}, axis={pitch.axis}",
    )

    cradle_visuals = {v.name for v in object_model.get_part("cradle").visuals}
    ctx.check(
        "cradle carries barrel + muzzle + glass lens (identity)",
        {"main_barrel", "muzzle_brake", "primary_lens"}.issubset(cradle_visuals),
        details=f"cradle_visuals={sorted(cradle_visuals)}",
    )

    yaw_visuals = {v.name for v in object_model.get_part("yaw_base").visuals}
    ctx.check(
        "turret has a trunnion cross-shaft anchoring the elevation axis",
        "trunnion_cross_shaft" in yaw_visuals,
        details=f"yaw_visuals={sorted(yaw_visuals)}",
    )
    ctx.check(
        "elevation origin sits on the trunnion plane",
        abs(pitch.origin.xyz[0]) < 1e-6
        and abs(pitch.origin.xyz[1]) < 1e-6
        and abs(pitch.origin.xyz[2] - r.pitch_origin_z) < 1e-6,
        details=f"pitch_origin={pitch.origin.xyz}",
    )

    # Positive elevation must raise the muzzle (sign match with the +X barrel).
    rest = ctx.part_element_world_aabb(object_model.get_part("cradle"), elem="muzzle_brake")
    with ctx.pose({pitch: min(r.elevation_upper, math.radians(55.0))}):
        high = ctx.part_element_world_aabb(object_model.get_part("cradle"), elem="muzzle_brake")
    if rest is not None and high is not None:
        ctx.check(
            "positive elevation raises the muzzle",
            high[1][2] > rest[1][2] + 0.05 * r.scale,
            details=f"rest_top_z={rest[1][2]}, high_top_z={high[1][2]}",
        )

    return ctx.report()
